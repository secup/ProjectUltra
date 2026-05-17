#include "otasim_client/ota_audio_backend.hpp"

#include "ota_simulator_service/audio_packet.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <utility>

#include <grpcpp/grpcpp.h>

#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <netdb.h>
#endif

namespace ultra::otasim_client {
namespace {

namespace otasim = projectultra::otasim::v1;
namespace service = ultra::ota_simulator_service;

constexpr uint32_t kFallbackMaxPacketSamples =
    static_cast<uint32_t>((8192 - service::kOtaAudioHeaderBytes) / sizeof(float));
constexpr size_t kPrimeSamples = 8;
constexpr size_t kMaxRxBufferSamples = 960000;
constexpr auto kRpcDeadline = std::chrono::milliseconds(1500);
constexpr auto kHeartbeatInterval = std::chrono::seconds(2);
constexpr auto kReconnectInterval = std::chrono::seconds(5);
constexpr auto kReconnectBudget = std::chrono::seconds(60);

void addToken(grpc::ClientContext& context, const std::string& token) {
    context.AddMetadata("authorization", "Bearer " + token);
}

void setDeadline(grpc::ClientContext& context, std::chrono::milliseconds duration) {
    context.set_deadline(std::chrono::system_clock::now() + duration);
}

void closeSocketIfValid(ultra::tnc::socket_t& socket) {
    if (!ultra::tnc::isInvalidSocket(socket)) {
        (void)ultra::tnc::shutdownSocket(socket);
        (void)ultra::tnc::closeSocket(socket);
        socket = ultra::tnc::kInvalidSocket;
    }
}

std::string grpcStatusMessage(const grpc::Status& status) {
    std::string message = status.error_message();
    if (message.empty()) {
        message = status.error_details();
    }
    if (message.empty()) {
        message = std::to_string(static_cast<int>(status.error_code()));
    }
    return message;
}

bool copySockaddr(const addrinfo* info,
                  sockaddr_storage* out,
                  ultra::tnc::socket_len_t* out_len) {
    if (!info || !out || !out_len ||
        info->ai_addrlen > static_cast<socklen_t>(sizeof(sockaddr_storage))) {
        return false;
    }
    std::memset(out, 0, sizeof(*out));
    std::memcpy(out, info->ai_addr, info->ai_addrlen);
    *out_len = static_cast<ultra::tnc::socket_len_t>(info->ai_addrlen);
    return true;
}

}  // namespace

OtaAudioBackend::OtaAudioBackend() = default;

OtaAudioBackend::~OtaAudioBackend() {
    close();
}

bool OtaAudioBackend::start(OtaAudioBackendConfig config, std::string* error) {
    close();
    if (config.grpc_target.empty()) {
        if (error) *error = "ota grpc target is required";
        return false;
    }
    if (config.token.empty()) {
        if (error) *error = "ota token is required";
        return false;
    }
    if (config.station_id.empty()) {
        if (error) *error = "ota station id is required";
        return false;
    }
    if (config.session_id.empty()) {
        config.session_id = "lobby";
    }
    if (!winsock_.ok()) {
        if (error) *error = "socket runtime initialization failed";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = std::move(config);
        running_ = true;
        rx_started_ = false;
        rx_next_sample_ = 0;
        rx_pending_.clear();
        rx_buffer_.clear();
        tx_sample_cursor_ = 0;
        tx_seq_ = 0;
        setStatusLocked(OtaAudioConnectionState::Connecting, "Connecting...");
    }

    control_thread_ = std::thread(&OtaAudioBackend::controlLoop, this);
    return true;
}

void OtaAudioBackend::close() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
    }
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    leaveSession();
    closeTransport();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
        joined_session_ = false;
        setStatusLocked(OtaAudioConnectionState::Disconnected, "Disconnected");
    }
}

bool OtaAudioBackend::queueTxSamples(std::span<const float> samples, std::string* error) {
    if (samples.empty()) {
        if (error) *error = "no samples to send";
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_ || ultra::tnc::isInvalidSocket(udp_socket_)) {
        if (error) *error = "ota audio is not connected";
        return false;
    }

    size_t offset = 0;
    while (offset < samples.size()) {
        const uint32_t limit = max_packet_samples_ == 0 ? kFallbackMaxPacketSamples : max_packet_samples_;
        const size_t chunk = std::min<size_t>(samples.size() - offset, limit);
        if (!sendSamplesLocked(tx_sample_cursor_, samples.subspan(offset, chunk), error)) {
            return false;
        }
        tx_sample_cursor_ += chunk;
        offset += chunk;
    }
    return true;
}

std::vector<float> OtaAudioBackend::getRxSamples(size_t max_samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t count = std::min(max_samples, rx_buffer_.size());
    std::vector<float> out(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(count));
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(count));
    return out;
}

size_t OtaAudioBackend::getRxBufferSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return rx_buffer_.size();
}

OtaAudioStatus OtaAudioBackend::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

bool OtaAudioBackend::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_;
}

bool OtaAudioBackend::waitForConnected(std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (isConnected()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return isConnected();
}

bool OtaAudioBackend::connectOnce(std::string* error) {
    OtaAudioBackendConfig config;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        config = config_;
    }

    auto channel = grpc::CreateChannel(config.grpc_target, grpc::InsecureChannelCredentials());
    if (!channel->WaitForConnected(std::chrono::system_clock::now() + kRpcDeadline)) {
        if (error) *error = "gRPC channel did not connect";
        return false;
    }
    auto stub = otasim::OtaSimulatorControl::NewStub(channel);

    otasim::RegisterStationRequest register_request;
    register_request.set_station_id(config.station_id);
    otasim::StationLease station_lease;
    grpc::ClientContext register_context;
    addToken(register_context, config.token);
    setDeadline(register_context, kRpcDeadline);
    auto status = stub->RegisterStation(&register_context, register_request, &station_lease);
    if (!status.ok()) {
        if (error) *error = "RegisterStation failed: " + grpcStatusMessage(status);
        return false;
    }

    const std::string station_id = station_lease.station_id().empty()
        ? config.station_id
        : station_lease.station_id();

    otasim::JoinSessionRequest join_request;
    join_request.set_session_id(config.session_id);
    join_request.set_station_id(station_id);
    otasim::JoinSessionResponse join_response;
    grpc::ClientContext join_context;
    addToken(join_context, config.token);
    setDeadline(join_context, kRpcDeadline);
    status = stub->JoinSession(&join_context, join_request, &join_response);
    if (!status.ok()) {
        if (error) *error = "JoinSession failed: " + grpcStatusMessage(status);
        return false;
    }
    auto leave_local_join = [&]() {
        otasim::LeaveSessionRequest request;
        request.set_session_id(config.session_id);
        request.set_station_id(station_id);
        google::protobuf::Empty response;
        grpc::ClientContext context;
        addToken(context, config.token);
        setDeadline(context, std::chrono::milliseconds(800));
        (void)stub->LeaveSession(&context, request, &response);
    };

    otasim::NegotiateAudioRequest audio_request;
    audio_request.set_session_id(config.session_id);
    audio_request.set_station_id(station_id);
    otasim::AudioLease audio_lease;
    grpc::ClientContext audio_context;
    addToken(audio_context, config.token);
    setDeadline(audio_context, kRpcDeadline);
    status = stub->NegotiateAudio(&audio_context, audio_request, &audio_lease);
    if (!status.ok()) {
        if (error) *error = "NegotiateAudio failed: " + grpcStatusMessage(status);
        leave_local_join();
        return false;
    }

    std::optional<UdpTarget> target;
    if (!config.udp_target.empty()) {
        target = parseHostPort(config.udp_target, error);
    } else {
        target = UdpTarget{.host = audio_lease.udp_host(),
                           .port = static_cast<uint16_t>(audio_lease.udp_port())};
    }
    if (!target || target->host.empty() || target->port == 0) {
        if (error && error->empty()) *error = "audio lease did not include a UDP endpoint";
        leave_local_join();
        return false;
    }

    auto resolved = resolveUdpTarget(target->host, target->port, error);
    if (!resolved) {
        leave_local_join();
        return false;
    }

    closeTransport();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        channel_ = std::move(channel);
        stub_ = std::move(stub);
        config_.station_id = station_id;
        lease_id_ = audio_lease.lease_id();
        joined_session_ = true;
        max_packet_samples_ = audio_lease.max_packet_samples() == 0
            ? kFallbackMaxPacketSamples
            : audio_lease.max_packet_samples();
        tx_sample_cursor_ = 0;
        tx_seq_ = 0;
    }

    if (!openUdpSocket(*resolved, error)) {
        leaveSession();
        closeTransport();
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!sendPrimeLocked(error)) {
            leaveSession();
            closeTransport();
            return false;
        }
        connected_ = true;
        setStatusLocked(OtaAudioConnectionState::Connected,
                        "Connected to " + config_.grpc_target +
                            " as " + config_.station_id +
                            " in session " + config_.session_id);
    }
    return true;
}

bool OtaAudioBackend::openUdpSocket(const UdpTarget& target, std::string* error) {
    auto socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ultra::tnc::isInvalidSocket(socket)) {
        if (error) *error = "failed to create UDP socket";
        return false;
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(0);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(socket, reinterpret_cast<sockaddr*>(&local), sizeof(local)) != 0) {
        ultra::tnc::closeSocket(socket);
        if (error) *error = "failed to bind UDP socket";
        return false;
    }

    sockaddr_storage server{};
    ultra::tnc::socket_len_t server_len = 0;
    auto resolved = resolveUdpTarget(target.host, target.port, error);
    if (!resolved) {
        ultra::tnc::closeSocket(socket);
        return false;
    }
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo* result = nullptr;
    const std::string port = std::to_string(resolved->port);
    if (::getaddrinfo(resolved->host.c_str(), port.c_str(), &hints, &result) != 0 || !result) {
        ultra::tnc::closeSocket(socket);
        if (error) *error = "failed to resolve UDP endpoint";
        return false;
    }
    const bool copied = copySockaddr(result, &server, &server_len);
    ::freeaddrinfo(result);
    if (!copied) {
        ultra::tnc::closeSocket(socket);
        if (error) *error = "failed to copy UDP endpoint";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        udp_socket_ = socket;
        udp_server_ = server;
        udp_server_len_ = server_len;
    }

    rx_thread_ = std::thread(&OtaAudioBackend::rxLoop, this);
    return true;
}

bool OtaAudioBackend::sendSamplesLocked(uint64_t start_sample,
                                        std::span<const float> samples,
                                        std::string* error) {
    service::OtaAudioPacket packet;
    packet.header.lease_id = lease_id_;
    packet.header.seq = tx_seq_++;
    packet.header.start_sample = start_sample;
    packet.samples.assign(samples.begin(), samples.end());
    const auto bytes = service::serializeAudioPacket(packet);

    const auto sent = ::sendto(udp_socket_,
                               reinterpret_cast<const char*>(bytes.data()),
                               bytes.size(),
                               0,
                               reinterpret_cast<const sockaddr*>(&udp_server_),
                               udp_server_len_);
    if (sent != static_cast<ultra::tnc::socket_io_result_t>(bytes.size())) {
        if (error) *error = "UDP audio send failed";
        return false;
    }
    return true;
}

bool OtaAudioBackend::sendPrimeLocked(std::string* error) {
    std::array<float, kPrimeSamples> prime{};
    if (!sendSamplesLocked(0, std::span<const float>(prime.data(), prime.size()), error)) {
        return false;
    }
    tx_sample_cursor_ = prime.size();
    return true;
}

bool OtaAudioBackend::heartbeat(std::string* error) {
    Stub::StubInterface* stub = nullptr;
    OtaAudioBackendConfig config;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!stub_) {
            if (error) *error = "gRPC stub is not connected";
            return false;
        }
        stub = stub_.get();
        config = config_;
    }

    otasim::HeartbeatRequest request;
    request.set_session_id(config.session_id);
    request.set_station_id(config.station_id);
    otasim::HeartbeatResponse response;
    grpc::ClientContext context;
    addToken(context, config.token);
    setDeadline(context, std::chrono::milliseconds(800));
    auto status = stub->Heartbeat(&context, request, &response);
    if (!status.ok() || !response.ok()) {
        if (error) {
            *error = status.ok() ? response.message()
                                 : "Heartbeat failed: " + grpcStatusMessage(status);
        }
        return false;
    }
    return true;
}

void OtaAudioBackend::leaveSession() {
    Stub::StubInterface* stub = nullptr;
    OtaAudioBackendConfig config;
    bool should_leave = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stub = stub_.get();
        config = config_;
        should_leave = joined_session_ && stub != nullptr;
        joined_session_ = false;
    }
    if (!should_leave) {
        return;
    }

    otasim::LeaveSessionRequest request;
    request.set_session_id(config.session_id);
    request.set_station_id(config.station_id);
    google::protobuf::Empty response;
    grpc::ClientContext context;
    addToken(context, config.token);
    setDeadline(context, std::chrono::milliseconds(800));
    (void)stub->LeaveSession(&context, request, &response);
}

void OtaAudioBackend::closeTransport() {
    ultra::tnc::socket_t socket = ultra::tnc::kInvalidSocket;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        socket = udp_socket_;
        udp_socket_ = ultra::tnc::kInvalidSocket;
        connected_ = false;
    }
    closeSocketIfValid(socket);
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lease_id_ = 0;
        channel_.reset();
        stub_.reset();
        max_packet_samples_ = 0;
    }
}

void OtaAudioBackend::controlLoop() {
    auto reconnect_started = std::chrono::steady_clock::now();
    auto last_heartbeat = std::chrono::steady_clock::time_point{};
    int attempt = 0;
    bool first_connect = true;

    while (true) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_) {
                return;
            }
        }

        if (!isConnected()) {
            ++attempt;
            const auto state = first_connect ? OtaAudioConnectionState::Connecting
                                             : OtaAudioConnectionState::Reconnecting;
            setStatus(state,
                      first_connect ? "Connecting..."
                                    : "Reconnecting (attempt " + std::to_string(attempt) + ")...",
                      attempt);
            std::string error;
            if (connectOnce(&error)) {
                attempt = 0;
                first_connect = false;
                reconnect_started = std::chrono::steady_clock::now();
                last_heartbeat = std::chrono::steady_clock::now();
                continue;
            }

            const auto now = std::chrono::steady_clock::now();
            if (!first_connect && now - reconnect_started >= kReconnectBudget) {
                setStatus(OtaAudioConnectionState::Failed,
                          "Disconnected: " + error + " (restart or check server)",
                          attempt);
                return;
            }
            if (first_connect) {
                first_connect = false;
                reconnect_started = now;
            }
            setStatus(OtaAudioConnectionState::Reconnecting,
                      "Reconnecting (attempt " + std::to_string(attempt) + ")...",
                      attempt);
            std::this_thread::sleep_for(kReconnectInterval);
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        if (last_heartbeat == std::chrono::steady_clock::time_point{} ||
            now - last_heartbeat >= kHeartbeatInterval) {
            std::string error;
            if (!heartbeat(&error)) {
                leaveSession();
                closeTransport();
                reconnect_started = std::chrono::steady_clock::now();
                attempt = 0;
                setStatus(OtaAudioConnectionState::Reconnecting,
                          "Reconnecting (attempt 1)...",
                          1);
                continue;
            }
            last_heartbeat = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void OtaAudioBackend::rxLoop() {
    std::array<uint8_t, 8192> buffer{};
    while (true) {
        ultra::tnc::socket_t socket = ultra::tnc::kInvalidSocket;
        uint64_t lease_id = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_ || ultra::tnc::isInvalidSocket(udp_socket_)) {
                break;
            }
            socket = udp_socket_;
            lease_id = lease_id_;
        }

        pollfd pfd{};
        pfd.fd = socket;
        pfd.events = POLLIN;
        const int selected = ultra::tnc::pollSockets(&pfd, 1, 100);
        if (selected < 0) {
            if (ultra::tnc::isInterruptedSocketError()) {
                continue;
            }
            break;
        }
        if (selected == 0 || !(pfd.revents & POLLIN)) {
            continue;
        }

        sockaddr_storage peer{};
        ultra::tnc::socket_len_t peer_len = sizeof(peer);
        const auto n = ::recvfrom(socket,
                                  reinterpret_cast<char*>(buffer.data()),
                                  buffer.size(),
                                  0,
                                  reinterpret_cast<sockaddr*>(&peer),
                                  &peer_len);
        if (n <= 0) {
            continue;
        }
        auto packet = service::parseAudioPacket(
            std::span<const uint8_t>(buffer.data(), static_cast<size_t>(n)));
        if (!packet || packet->header.lease_id != lease_id) {
            continue;
        }
        pushRxPacket(packet->header.start_sample, packet->samples);
    }
}

void OtaAudioBackend::pushRxPacket(uint64_t start_sample, std::span<const float> samples) {
    if (samples.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!rx_started_) {
        rx_started_ = true;
        rx_next_sample_ = start_sample;
    }

    const uint64_t end_sample = start_sample + samples.size();
    if (end_sample <= rx_next_sample_) {
        return;
    }

    size_t offset = 0;
    if (start_sample < rx_next_sample_) {
        offset = static_cast<size_t>(rx_next_sample_ - start_sample);
        start_sample = rx_next_sample_;
    }

    std::vector<float> copy(samples.begin() + static_cast<std::ptrdiff_t>(offset), samples.end());
    rx_pending_.emplace(start_sample, std::move(copy));
    drainReadyRxLocked();
}

void OtaAudioBackend::drainReadyRxLocked() {
    while (!rx_pending_.empty()) {
        auto it = rx_pending_.begin();
        if (it->first != rx_next_sample_) {
            break;
        }
        rx_buffer_.insert(rx_buffer_.end(), it->second.begin(), it->second.end());
        rx_next_sample_ = it->first + it->second.size();
        rx_pending_.erase(it);
    }

    if (rx_buffer_.size() > kMaxRxBufferSamples) {
        const size_t drop = rx_buffer_.size() - kMaxRxBufferSamples;
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(drop));
    }
}

void OtaAudioBackend::setStatus(OtaAudioConnectionState state, std::string text, int attempt) {
    std::lock_guard<std::mutex> lock(mutex_);
    setStatusLocked(state, std::move(text), attempt);
}

void OtaAudioBackend::setStatusLocked(OtaAudioConnectionState state, std::string text, int attempt) {
    status_.state = state;
    status_.text = std::move(text);
    status_.reconnect_attempt = attempt;
}

std::optional<OtaAudioBackend::UdpTarget>
OtaAudioBackend::parseHostPort(const std::string& value, std::string* error) {
    const auto pos = value.rfind(':');
    if (pos == std::string::npos || pos == 0 || pos + 1 >= value.size()) {
        if (error) *error = "UDP endpoint must be host:port";
        return std::nullopt;
    }
    unsigned long port_value = 0;
    try {
        port_value = std::stoul(value.substr(pos + 1));
    } catch (const std::exception&) {
        if (error) *error = "UDP port is invalid";
        return std::nullopt;
    }
    if (port_value == 0 || port_value > std::numeric_limits<uint16_t>::max()) {
        if (error) *error = "UDP port out of range";
        return std::nullopt;
    }
    return UdpTarget{.host = value.substr(0, pos),
                     .port = static_cast<uint16_t>(port_value)};
}

std::optional<OtaAudioBackend::UdpTarget>
OtaAudioBackend::resolveUdpTarget(const std::string& host, uint16_t port, std::string* error) {
    if (host.empty() || port == 0) {
        if (error) *error = "UDP endpoint is empty";
        return std::nullopt;
    }
    return UdpTarget{.host = host, .port = port};
}

}  // namespace ultra::otasim_client
