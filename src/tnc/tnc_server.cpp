#include "tnc/tnc_server.hpp"

#include "ultra/logging.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstring>
#include <limits>

#include <utility>

#ifndef _WIN32
#include <fcntl.h>
#endif

namespace ultra::tnc {
namespace {

constexpr int kBacklog = 4;
constexpr int kPollTimeoutMs = 100;
constexpr uint32_t kSessionTickMs = 100;
constexpr uint32_t kDefaultIAmAliveMs = 60000;
constexpr uint32_t kDefaultBufferRateMs = 1000;
constexpr size_t kReadBufferSize = 4096;
constexpr size_t kMaxControlLine = 4096;

int sendFlags() {
#ifdef MSG_NOSIGNAL
    return MSG_NOSIGNAL;
#else
    return 0;
#endif
}

bool isWouldBlock() {
    return isWouldBlockSocketError();
}

void closeFd(socket_t& fd) {
    if (!isInvalidSocket(fd)) {
        closeSocket(fd);
        fd = kInvalidSocket;
    }
}

void closeClientFd(socket_t& fd) {
    if (!isInvalidSocket(fd)) {
        shutdownSocket(fd);
        closeSocket(fd);
        fd = kInvalidSocket;
    }
}

void setCloseOnExec(socket_t fd) {
#ifndef _WIN32
    const int flags = fcntl(fd, F_GETFD, 0);
    if (flags >= 0) {
        (void)fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
    }
#else
    (void)fd;
#endif
}

void applySocketOptions(socket_t fd) {
    const int one = 1;
    const auto* optval = reinterpret_cast<const char*>(&one);
    const socket_len_t optlen = static_cast<socket_len_t>(sizeof(one));
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, optval, optlen);
    (void)setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, optval, optlen);
#ifdef SO_NOSIGPIPE
    (void)setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, optval, optlen);
#endif
}

uint16_t getBoundPort(socket_t fd) {
    sockaddr_in addr {};
    socket_len_t len = static_cast<socket_len_t>(sizeof(addr));
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        return 0;
    }
    return ntohs(addr.sin_port);
}

uint16_t nextPort(uint16_t port) {
    return port == std::numeric_limits<uint16_t>::max() ? 1 : static_cast<uint16_t>(port + 1);
}

bool parseBindAddress(const std::string& bind_address, in_addr& out) {
    if (inet_pton(AF_INET, bind_address.c_str(), &out) == 1) {
        return true;
    }
    if (bind_address == "localhost") {
        return inet_pton(AF_INET, "127.0.0.1", &out) == 1;
    }
    return false;
}

socket_t openListenerSocket(const std::string& bind_address, uint16_t port) {
    in_addr parsed_addr {};
    if (!parseBindAddress(bind_address, parsed_addr)) {
        return kInvalidSocket;
    }

    socket_t fd = socket(AF_INET, SOCK_STREAM, 0);
    if (isInvalidSocket(fd)) {
        return kInvalidSocket;
    }

    setCloseOnExec(fd);
    applySocketOptions(fd);

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr = parsed_addr;
    addr.sin_port = htons(port);

    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
        listen(fd, kBacklog) != 0 ||
        !setNonblocking(fd)) {
        closeFd(fd);
        return kInvalidSocket;
    }

    return fd;
}

bool prepareAcceptedSocket(socket_t fd) {
    setCloseOnExec(fd);
    applySocketOptions(fd);
    return setNonblocking(fd);
}

std::chrono::steady_clock::duration nextTimerDelay(uint32_t accum_ms, uint32_t interval_ms) {
    if (interval_ms == 0 || accum_ms >= interval_ms) {
        return std::chrono::milliseconds(0);
    }
    return std::chrono::milliseconds(interval_ms - accum_ms);
}

} // namespace

TNCServer::TNCServer(ModemAdapter& modem, TNCServerConfig config)
    : modem_(modem),
      config_(std::move(config)) {
    static WinsockInit winsock;
    (void)winsock;
    resetSession();
}

TNCServer::~TNCServer() {
    stop();
}

bool TNCServer::start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load()) {
        return false;
    }

#ifndef _WIN32
    signal(SIGPIPE, SIG_IGN);
#endif
    stop_requested_.store(false);
    real_tick_accum_ms_ = 0;
    iamalive_override_accum_ms_ = 0;
    buffer_override_accum_ms_ = 0;
    cmd_line_buffer_.clear();
    cmd_tx_buffer_.clear();
    data_tx_buffer_.clear();
    {
        std::lock_guard<std::mutex> queue_lock(queue_mutex_);
        modem_events_.clear();
    }
    resetSession();

    if (!openWakeupPipe()) {
        closeServerFds();
        return false;
    }

    if (!openListeners()) {
        closeServerFds();
        return false;
    }

    running_.store(true);
    reactor_thread_ = std::thread(&TNCServer::reactorLoop, this);
    return true;
}

void TNCServer::stop() {
    std::unique_lock<std::mutex> lock(lifecycle_mutex_);
    if (!running_.load() && !reactor_thread_.joinable()) {
        return;
    }

    stop_requested_.store(true);
    wakeReactor();
    std::thread local_thread;
    if (reactor_thread_.joinable()) {
        local_thread = std::move(reactor_thread_);
    }
    lock.unlock();

    if (local_thread.joinable()) {
        local_thread.join();
    }

    running_.store(false);
}

bool TNCServer::isRunning() const {
    return running_.load();
}

uint16_t TNCServer::getCmdPort() const {
    return bound_cmd_port_.load();
}

uint16_t TNCServer::getDataPort() const {
    return bound_data_port_.load();
}

const TNCSession& TNCServer::getSession() const {
    return *session_;
}

void TNCServer::postModemConnected(const std::string& src, const std::string& dst, int bw) {
    TNCEvent event;
    event.type = TNCEventType::Connected;
    event.connect.source = src;
    event.connect.dest = dst;
    event.connect.bandwidth_hz = bw;
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemDisconnected() {
    TNCEvent event;
    event.type = TNCEventType::Disconnected;
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemPTT(bool on) {
    TNCEvent event;
    event.type = TNCEventType::PTT;
    event.ptt_on = on;
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemDataReceived(std::vector<uint8_t> bytes) {
    TNCEvent event;
    event.type = TNCEventType::DataReceived;
    event.data = std::move(bytes);
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemBufferLevel(int bytes) {
    TNCEvent event;
    event.type = TNCEventType::BufferLevel;
    event.bytes = bytes;
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemSNR(float db) {
    TNCEvent event;
    event.type = TNCEventType::SNR;
    event.snr_db = db;
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemBitrate(int bps) {
    TNCEvent event;
    event.type = TNCEventType::Bitrate;
    event.bitrate_bps = bps;
    enqueueModemEvent(std::move(event));
}

void TNCServer::postModemIncomingCall(std::string peer) {
    TNCEvent event;
    event.type = TNCEventType::IncomingCall;
    event.peer = std::move(peer);
    enqueueModemEvent(std::move(event));
}

void TNCServer::reactorLoop() {
    auto last_timer = std::chrono::steady_clock::now();

    while (!stop_requested_.load()) {
        std::vector<pollfd> poll_fds;
        std::vector<PollTarget> poll_targets;

        auto add_fd = [&](socket_t fd, short events, PollTarget target) {
            if (!isInvalidSocket(fd)) {
                poll_fds.push_back({fd, events, 0});
                poll_targets.push_back(target);
            }
        };

        // Windows WSAPoll only accepts POLLIN/POLLOUT/POLLPRI/POLLRDBAND/
        // POLLRDNORM/POLLWRNORM in `events`. POLLERR/POLLHUP/POLLNVAL are
        // output-only and are returned in `revents` regardless of what
        // we request. Putting them in `events` is harmless on POSIX but
        // some Windows builds reject the call with WSAEINVAL.
        add_fd(cmd_listener_fd_, POLLIN, PollTarget::CmdListener);
        add_fd(data_listener_fd_, POLLIN, PollTarget::DataListener);
        add_fd(cmd_client_fd_,
               static_cast<short>(POLLIN |
                                  (cmd_tx_buffer_.empty() ? 0 : POLLOUT)),
               PollTarget::CmdClient);
        add_fd(data_client_fd_,
               static_cast<short>(POLLIN |
                                  (data_tx_buffer_.empty() ? 0 : POLLOUT)),
               PollTarget::DataClient);
        add_fd(wakeup_read_fd_, POLLIN, PollTarget::Wakeup);

        int timeout_ms = kPollTimeoutMs;
        if (config_.iamalive_interval_ms != kDefaultIAmAliveMs && config_.iamalive_interval_ms > 0) {
            const auto delay = nextTimerDelay(iamalive_override_accum_ms_, config_.iamalive_interval_ms);
            timeout_ms = std::min(timeout_ms, static_cast<int>(
                                                  std::chrono::duration_cast<std::chrono::milliseconds>(delay).count()));
        }
        if (config_.buffer_rate_limit_ms != kDefaultBufferRateMs && config_.buffer_rate_limit_ms > 0) {
            const auto delay = nextTimerDelay(buffer_override_accum_ms_, config_.buffer_rate_limit_ms);
            timeout_ms = std::min(timeout_ms, static_cast<int>(
                                                  std::chrono::duration_cast<std::chrono::milliseconds>(delay).count()));
        }

        const int poll_rc = pollSockets(poll_fds.data(), static_cast<poll_count_t>(poll_fds.size()), timeout_ms);
        if (poll_rc < 0 && !isInterruptedSocketError()) {
            break;
        }

        for (size_t i = 0; i < poll_fds.size(); ++i) {
            const short events = poll_fds[i].revents;
            if (events == 0) {
                continue;
            }

            switch (poll_targets[i]) {
            case PollTarget::CmdListener:
                if (poll_fds[i].fd == cmd_listener_fd_ && (events & POLLIN)) {
                    onCmdListenerReady();
                }
                break;
            case PollTarget::DataListener:
                if (poll_fds[i].fd == data_listener_fd_ && (events & POLLIN)) {
                    onDataListenerReady();
                }
                break;
            case PollTarget::CmdClient:
                if (poll_fds[i].fd == cmd_client_fd_) {
                    // Treat POLLHUP as a hint to drain via recv(); if the
                    // peer truly closed, recv() returns 0 and onCmdClientReady
                    // evicts. Windows WSAPoll can transiently set POLLHUP on
                    // healthy connections, so evicting on POLLHUP alone drops
                    // valid clients before we read their data.
                    if (events & (POLLIN | POLLHUP)) {
                        onCmdClientReady();
                    }
                    if (poll_fds[i].fd == cmd_client_fd_ && (events & POLLOUT)) {
                        if (!flushClientOutput(cmd_client_fd_, cmd_tx_buffer_)) {
                            evictCmdClient();
                        }
                    }
                    if (poll_fds[i].fd == cmd_client_fd_ && (events & (POLLERR | POLLNVAL))) {
                        evictCmdClient();
                    }
                }
                break;
            case PollTarget::DataClient:
                if (poll_fds[i].fd == data_client_fd_) {
                    if (events & (POLLIN | POLLHUP)) {
                        onDataClientReady();
                    }
                    if (poll_fds[i].fd == data_client_fd_ && (events & POLLOUT)) {
                        if (!flushClientOutput(data_client_fd_, data_tx_buffer_)) {
                            evictDataClient();
                        }
                    }
                    if (poll_fds[i].fd == data_client_fd_ && (events & (POLLERR | POLLNVAL))) {
                        evictDataClient();
                    }
                }
                break;
            case PollTarget::Wakeup:
                if (poll_fds[i].fd == wakeup_read_fd_ && (events & POLLIN)) {
                    drainWakeupPipe();
                }
                break;
            }
        }

        drainModemQueue();

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_timer).count();
        if (elapsed_ms > 0) {
            const uint32_t clamped_elapsed =
                elapsed_ms > std::numeric_limits<uint32_t>::max()
                    ? std::numeric_limits<uint32_t>::max()
                    : static_cast<uint32_t>(elapsed_ms);
            onTimerTick(clamped_elapsed);
            last_timer = now;
        }

        flushAllOutputs();
    }

    closeServerFds();
    running_.store(false);
}

void TNCServer::onCmdListenerReady() {
    while (true) {
        const socket_t fd = accept(cmd_listener_fd_, nullptr, nullptr);
        if (isInvalidSocket(fd)) {
            if (isInterruptedSocketError()) {
                continue;
            }
            return;
        }

        if (!prepareAcceptedSocket(fd)) {
            socket_t close_fd = fd;
            closeClientFd(close_fd);
            continue;
        }

        evictCmdClient();
        resetSession();
        cmd_client_fd_ = fd;
        cmd_line_buffer_.clear();
        real_tick_accum_ms_ = 0;
        iamalive_override_accum_ms_ = 0;
        buffer_override_accum_ms_ = 0;
    }
}

void TNCServer::onDataListenerReady() {
    while (true) {
        const socket_t fd = accept(data_listener_fd_, nullptr, nullptr);
        if (isInvalidSocket(fd)) {
            if (isInterruptedSocketError()) {
                continue;
            }
            return;
        }

        if (!prepareAcceptedSocket(fd)) {
            socket_t close_fd = fd;
            closeClientFd(close_fd);
            continue;
        }

        evictDataClient();
        data_client_fd_ = fd;
    }
}

void TNCServer::onCmdClientReady() {
    uint8_t buffer[kReadBufferSize];
    while (!isInvalidSocket(cmd_client_fd_)) {
        const socket_io_result_t n = recvSocket(cmd_client_fd_, buffer, sizeof(buffer), 0);
        if (n > 0) {
            processControlBytes(buffer, static_cast<size_t>(n));
            continue;
        }
        if (n == 0) {
            evictCmdClient();
            return;
        }
        if (isInterruptedSocketError()) {
            continue;
        }
        if (isWouldBlock()) {
            return;
        }
        evictCmdClient();
        return;
    }
}

void TNCServer::onDataClientReady() {
    uint8_t buffer[kReadBufferSize];
    while (!isInvalidSocket(data_client_fd_)) {
        const socket_io_result_t n = recvSocket(data_client_fd_, buffer, sizeof(buffer), 0);
        if (n > 0) {
            session_->handleDataBytes(std::vector<uint8_t>(buffer, buffer + static_cast<std::ptrdiff_t>(n)));
            continue;
        }
        if (n == 0) {
            evictDataClient();
            return;
        }
        if (isInterruptedSocketError()) {
            continue;
        }
        if (isWouldBlock()) {
            return;
        }
        evictDataClient();
        return;
    }
}

void TNCServer::onTimerTick(uint32_t elapsed_ms) {
    auto addClamped = [](uint32_t value, uint32_t delta) {
        return value > std::numeric_limits<uint32_t>::max() - delta
                   ? std::numeric_limits<uint32_t>::max()
                   : value + delta;
    };

    real_tick_accum_ms_ = addClamped(real_tick_accum_ms_, elapsed_ms);
    while (real_tick_accum_ms_ >= kSessionTickMs) {
        session_->tick(kSessionTickMs);
        real_tick_accum_ms_ -= kSessionTickMs;
    }

    if (config_.iamalive_interval_ms != kDefaultIAmAliveMs && config_.iamalive_interval_ms > 0) {
        iamalive_override_accum_ms_ = addClamped(iamalive_override_accum_ms_, elapsed_ms);
        while (iamalive_override_accum_ms_ >= config_.iamalive_interval_ms) {
            session_->tick(kDefaultIAmAliveMs);
            iamalive_override_accum_ms_ -= config_.iamalive_interval_ms;
        }
    }

    if (config_.buffer_rate_limit_ms != kDefaultBufferRateMs && config_.buffer_rate_limit_ms > 0) {
        buffer_override_accum_ms_ = addClamped(buffer_override_accum_ms_, elapsed_ms);
        while (buffer_override_accum_ms_ >= config_.buffer_rate_limit_ms) {
            session_->tick(kDefaultBufferRateMs);
            buffer_override_accum_ms_ -= config_.buffer_rate_limit_ms;
        }
    }
}

void TNCServer::evictCmdClient() {
    // The host that owned the radio session is gone (closed, or replaced by a new connection):
    // abort the session, drop its queued audio and unkey. Only a fresh TNCSession used to be made,
    // so the connect retries ran on underneath it and the next host's CONNECT was refused with
    // "state=CONNECTING" until a restart (GitHub #38).
    if (!isInvalidSocket(cmd_client_fd_)) {
        modem_.abort();
    }
    closeClientFd(cmd_client_fd_);
    closeClientFd(data_client_fd_);
    cmd_line_buffer_.clear();
    cmd_tx_buffer_.clear();
    data_tx_buffer_.clear();
    resetSession();
    real_tick_accum_ms_ = 0;
    iamalive_override_accum_ms_ = 0;
    buffer_override_accum_ms_ = 0;
}

void TNCServer::evictDataClient() {
    closeClientFd(data_client_fd_);
    data_tx_buffer_.clear();
}

void TNCServer::emitToCmdClient(std::string_view line) {
    if (isInvalidSocket(cmd_client_fd_) || line.empty()) {
        return;
    }
    // Trace the host<-TNC side of the VARA-HF command dialogue for debugging Winlink/Pat.
    // (matches the host->TNC trace in processControlBytes). Strip the trailing CR/LF.
    std::string_view shown = line;
    while (!shown.empty() && (shown.back() == '\r' || shown.back() == '\n')) {
        shown.remove_suffix(1);
    }
    if (!shown.empty()) {
        LOG_INFO("TNC", "tnc->host: %.*s", static_cast<int>(shown.size()), shown.data());
        ultra::flushLog();  // low-volume control dialogue -> flush so it's visible live for debugging
    }
    cmd_tx_buffer_.insert(cmd_tx_buffer_.end(), line.begin(), line.end());
}

void TNCServer::emitToDataClient(const std::vector<uint8_t>& bytes) {
    if (isInvalidSocket(data_client_fd_) || bytes.empty()) {
        return;
    }
    data_tx_buffer_.insert(data_tx_buffer_.end(), bytes.begin(), bytes.end());
}

bool TNCServer::openListeners() {
    closeFd(cmd_listener_fd_);
    closeFd(data_listener_fd_);
    bound_cmd_port_.store(0);
    bound_data_port_.store(0);

    const bool adjacent_ports =
        config_.data_port == 0 || (config_.cmd_port == 0 && config_.data_port == TNCServerConfig{}.data_port);
    constexpr int kOsEphemeralAttempts = 4;
    constexpr int kAdjacentScanAttempts = 4096;
    constexpr uint16_t kScanBasePort = 49152;
    constexpr uint16_t kScanPortCount = 16000;

    auto requestedCmdPortForAttempt = [&](int attempt) -> uint16_t {
        if (config_.cmd_port != 0) {
            return config_.cmd_port;
        }
        if (!adjacent_ports || attempt < kOsEphemeralAttempts) {
            return 0;
        }

        const auto seed = static_cast<uint32_t>(
            std::chrono::steady_clock::now().time_since_epoch().count() % kScanPortCount);
        return static_cast<uint16_t>(kScanBasePort +
                                     ((seed + static_cast<uint32_t>(attempt - kOsEphemeralAttempts)) %
                                      kScanPortCount));
    };

    const int attempts = config_.cmd_port == 0 && adjacent_ports
                             ? kOsEphemeralAttempts + kAdjacentScanAttempts
                             : 1;
    for (int attempt = 0; attempt < attempts; ++attempt) {
        const uint16_t requested_cmd = requestedCmdPortForAttempt(attempt);
        if (requested_cmd == std::numeric_limits<uint16_t>::max()) {
            continue;
        }

        cmd_listener_fd_ = openListenerSocket(config_.bind_address, requested_cmd);
        if (isInvalidSocket(cmd_listener_fd_)) {
            if (config_.cmd_port == 0 && adjacent_ports) {
                continue;
            }
            return false;
        }

        const uint16_t actual_cmd = getBoundPort(cmd_listener_fd_);
        const uint16_t requested_data = adjacent_ports ? nextPort(actual_cmd) : config_.data_port;
        data_listener_fd_ = openListenerSocket(config_.bind_address, requested_data);
        if (!isInvalidSocket(data_listener_fd_)) {
            bound_cmd_port_.store(actual_cmd);
            bound_data_port_.store(getBoundPort(data_listener_fd_));
            return true;
        }

        closeFd(cmd_listener_fd_);
    }

    return false;
}

bool TNCServer::openWakeupPipe() {
    closeFd(wakeup_read_fd_);
    closeFd(wakeup_write_fd_);

    socket_t fds[2] = {kInvalidSocket, kInvalidSocket};
    if (!socketPair(fds)) {
        return false;
    }

    wakeup_read_fd_ = fds[0];
    wakeup_write_fd_ = fds[1];
    setCloseOnExec(wakeup_read_fd_);
    setCloseOnExec(wakeup_write_fd_);
    if (!setNonblocking(wakeup_read_fd_) || !setNonblocking(wakeup_write_fd_)) {
        closeFd(wakeup_read_fd_);
        closeFd(wakeup_write_fd_);
        return false;
    }
    return true;
}

void TNCServer::closeServerFds() {
    closeClientFd(cmd_client_fd_);
    closeClientFd(data_client_fd_);
    closeFd(cmd_listener_fd_);
    closeFd(data_listener_fd_);
    closeFd(wakeup_read_fd_);
    closeFd(wakeup_write_fd_);
    cmd_line_buffer_.clear();
    cmd_tx_buffer_.clear();
    data_tx_buffer_.clear();
    bound_cmd_port_.store(0);
    bound_data_port_.store(0);
}

void TNCServer::resetSession() {
    session_ = std::make_unique<TNCSession>(
        modem_,
        [this](std::string_view line) { emitToCmdClient(line); },
        [this](const std::vector<uint8_t>& bytes) { emitToDataClient(bytes); });
}

void TNCServer::enqueueModemEvent(TNCEvent event) {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        modem_events_.push_back(std::move(event));
    }
    wakeReactor();
}

void TNCServer::wakeReactor() {
    if (isInvalidSocket(wakeup_write_fd_)) {
        return;
    }

    const uint8_t byte = 0;
    while (true) {
        const socket_io_result_t n = writeSocketPair(wakeup_write_fd_, &byte, sizeof(byte));
        if (n == 1) {
            return;
        }
        if (n < 0 && isInterruptedSocketError()) {
            continue;
        }
        return;
    }
}

void TNCServer::drainWakeupPipe() {
    uint8_t buffer[128];
    while (!isInvalidSocket(wakeup_read_fd_)) {
        const socket_io_result_t n = readSocketPair(wakeup_read_fd_, buffer, sizeof(buffer));
        if (n > 0) {
            continue;
        }
        if (n < 0 && isInterruptedSocketError()) {
            continue;
        }
        return;
    }
}

void TNCServer::drainModemQueue() {
    std::deque<TNCEvent> local_events;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        local_events.swap(modem_events_);
    }

    for (const TNCEvent& event : local_events) {
        dispatchModemEvent(event);
    }
}

void TNCServer::dispatchModemEvent(const TNCEvent& event) {
    switch (event.type) {
    case TNCEventType::Connected:
        session_->onModemConnected(event.connect.source, event.connect.dest, event.connect.bandwidth_hz);
        break;
    case TNCEventType::Disconnected:
        session_->onModemDisconnected();
        break;
    case TNCEventType::PTT:
        session_->onModemPTT(event.ptt_on);
        break;
    case TNCEventType::BufferLevel:
        session_->onModemBufferLevel(event.bytes);
        break;
    case TNCEventType::SNR:
        session_->onModemSNR(event.snr_db);
        break;
    case TNCEventType::Bitrate:
        session_->onModemBitrate(event.bitrate_bps);
        break;
    case TNCEventType::IncomingCall:
        session_->onModemIncomingCall(event.peer);
        break;
    case TNCEventType::DataReceived:
        session_->onModemDataReceived(event.data);
        break;
    case TNCEventType::Pending:
    case TNCEventType::CancelPending:
    case TNCEventType::IAmAlive:
        break;
    }
}

void TNCServer::processControlBytes(const uint8_t* bytes, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        const char ch = static_cast<char>(bytes[i]);
        if (ch == '\r') {
            // Trace every host->TNC command (VARA-HF host commands from Winlink Express / Pat)
            // for debugging the dialogue. Low volume (a few per connection); INFO so it shows
            // under the "TNC" log category without --log-level debug.
            if (!cmd_line_buffer_.empty()) {
                LOG_INFO("TNC", "host->tnc: %s", cmd_line_buffer_.c_str());
                ultra::flushLog();  // low-volume control dialogue -> flush so it's visible live
            }
            session_->handleControlLine(cmd_line_buffer_);
            cmd_line_buffer_.clear();
            continue;
        }
        if (ch == '\n') {
            continue;
        }
        if (cmd_line_buffer_.size() >= kMaxControlLine) {
            session_->handleControlLine("");
            cmd_line_buffer_.clear();
            continue;
        }
        cmd_line_buffer_.push_back(ch);
    }
}

void TNCServer::flushAllOutputs() {
    if (!isInvalidSocket(cmd_client_fd_) && !cmd_tx_buffer_.empty() &&
        !flushClientOutput(cmd_client_fd_, cmd_tx_buffer_)) {
        evictCmdClient();
    }
    if (!isInvalidSocket(data_client_fd_) && !data_tx_buffer_.empty() &&
        !flushClientOutput(data_client_fd_, data_tx_buffer_)) {
        evictDataClient();
    }
}

bool TNCServer::flushClientOutput(socket_t fd, std::vector<uint8_t>& buffer) {
    while (!buffer.empty()) {
        const socket_io_result_t n = sendSocket(fd, buffer.data(), buffer.size(), sendFlags());
        if (n > 0) {
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(n));
            continue;
        }
        if (n < 0 && isInterruptedSocketError()) {
            continue;
        }
        if (n < 0 && isWouldBlock()) {
            return true;
        }
        return false;
    }
    return true;
}

} // namespace ultra::tnc
