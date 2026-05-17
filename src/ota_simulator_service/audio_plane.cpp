#include "ota_simulator_service/audio_plane.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <utility>

namespace ultra::ota_simulator_service {
namespace {

constexpr size_t kMaxDatagramBytes = 8192;

bool fillAddress(const std::string& host,
                 uint16_t port,
                 sockaddr_in* addr,
                 std::string* error) {
    std::memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    if (host.empty() || host == "0.0.0.0") {
        addr->sin_addr.s_addr = htonl(INADDR_ANY);
        return true;
    }
    if (::inet_pton(AF_INET, host.c_str(), &addr->sin_addr) != 1) {
        if (error) {
            *error = "UDP bind host must be an IPv4 address";
        }
        return false;
    }
    return true;
}

bool isRunningLocked(bool running, ultra::tnc::socket_t socket) {
    return running && !ultra::tnc::isInvalidSocket(socket);
}

}  // namespace

bool OrderedAudioQueue::push(uint64_t start_sample, std::span<const float> samples) {
    if (samples.empty()) {
        return false;
    }
    const uint64_t end_sample = start_sample + samples.size();
    if (end_sample <= next_sample_ || start_sample < next_sample_) {
        return false;
    }
    auto [_, inserted] = pending_.emplace(
        start_sample, std::vector<float>(samples.begin(), samples.end()));
    return inserted;
}

std::vector<OrderedAudioQueue::Block> OrderedAudioQueue::drainReady() {
    std::vector<Block> ready;
    while (!pending_.empty()) {
        auto it = pending_.begin();
        if (it->first != next_sample_) {
            break;
        }
        Block block{.start_sample = it->first, .samples = std::move(it->second)};
        next_sample_ = block.start_sample + block.samples.size();
        pending_.erase(it);
        ready.push_back(std::move(block));
    }
    return ready;
}

std::vector<float> OrderedAudioQueue::readWindow(uint64_t start_sample, size_t count) {
    std::vector<float> out(count, 0.0f);
    const uint64_t end_sample = start_sample + count;
    for (const auto& [block_start, samples] : pending_) {
        const uint64_t block_end = block_start + samples.size();
        if (block_end <= start_sample) {
            continue;
        }
        if (block_start >= end_sample) {
            break;
        }
        const uint64_t overlap_begin = std::max(start_sample, block_start);
        const uint64_t overlap_end = std::min(end_sample, block_end);
        const size_t out_offset = static_cast<size_t>(overlap_begin - start_sample);
        const size_t in_offset = static_cast<size_t>(overlap_begin - block_start);
        const size_t n = static_cast<size_t>(overlap_end - overlap_begin);
        std::copy_n(samples.begin() + static_cast<std::ptrdiff_t>(in_offset),
                    static_cast<std::ptrdiff_t>(n),
                    out.begin() + static_cast<std::ptrdiff_t>(out_offset));
    }
    next_sample_ = std::max(next_sample_, end_sample);
    for (auto it = pending_.begin(); it != pending_.end();) {
        if (it->first + it->second.size() <= next_sample_) {
            it = pending_.erase(it);
        } else {
            ++it;
        }
    }
    return out;
}

UdpAudioPlane::UdpAudioPlane() = default;

UdpAudioPlane::~UdpAudioPlane() {
    stop();
}

bool UdpAudioPlane::start(const std::string& bind_host,
                          uint16_t bind_port,
                          PacketCallback callback,
                          std::string* error) {
    stop();
    if (!winsock_.ok()) {
        if (error) {
            *error = "socket runtime initialization failed";
        }
        return false;
    }

    sockaddr_in addr{};
    if (!fillAddress(bind_host, bind_port, &addr, error)) {
        return false;
    }

    auto socket = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ultra::tnc::isInvalidSocket(socket)) {
        if (error) {
            *error = "failed to create UDP socket";
        }
        return false;
    }

    const int reuse = 1;
    (void)::setsockopt(socket, SOL_SOCKET, SO_REUSEADDR,
                       reinterpret_cast<const char*>(&reuse), sizeof(reuse));
    if (::bind(socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ultra::tnc::closeSocket(socket);
        if (error) {
            *error = "failed to bind UDP socket";
        }
        return false;
    }

    sockaddr_in actual{};
    ultra::tnc::socket_len_t actual_len = sizeof(actual);
    if (::getsockname(socket, reinterpret_cast<sockaddr*>(&actual), &actual_len) != 0) {
        ultra::tnc::closeSocket(socket);
        if (error) {
            *error = "failed to inspect UDP socket";
        }
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        socket_ = socket;
        host_ = bind_host.empty() ? "0.0.0.0" : bind_host;
        port_ = ntohs(actual.sin_port);
        callback_ = std::move(callback);
        running_ = true;
    }

    thread_ = std::thread(&UdpAudioPlane::run, this);
    return true;
}

void UdpAudioPlane::stop() {
    ultra::tnc::socket_t socket = ultra::tnc::kInvalidSocket;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        socket = socket_;
        running_ = false;
        socket_ = ultra::tnc::kInvalidSocket;
    }
    if (!ultra::tnc::isInvalidSocket(socket)) {
        (void)ultra::tnc::shutdownSocket(socket);
        (void)ultra::tnc::closeSocket(socket);
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool UdpAudioPlane::running() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return isRunningLocked(running_, socket_);
}

uint16_t UdpAudioPlane::port() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return port_;
}

std::string UdpAudioPlane::host() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return host_;
}

uint64_t UdpAudioPlane::addLease(std::string session_id, std::string station_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t lease_id = next_lease_id_++;
    leases_.emplace(lease_id, LeaseState{
        .lease_id = lease_id,
        .session_id = std::move(session_id),
        .station_id = std::move(station_id),
    });
    return lease_id;
}

std::vector<UdpAudioPlane::LeaseSnapshot> UdpAudioPlane::leasesForSession(
    std::string_view session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<LeaseSnapshot> out;
    for (const auto& [_, lease] : leases_) {
        if (lease.session_id == session_id) {
            out.push_back({
                .lease_id = lease.lease_id,
                .session_id = lease.session_id,
                .station_id = lease.station_id,
                .has_endpoint = lease.endpoint.has_value(),
            });
        }
    }
    return out;
}

bool UdpAudioPlane::sendAudio(uint64_t lease_id,
                              uint64_t start_sample,
                              std::span<const float> samples) {
    Endpoint endpoint;
    uint64_t seq = 0;
    ultra::tnc::socket_t socket = ultra::tnc::kInvalidSocket;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = leases_.find(lease_id);
        if (it == leases_.end() || !it->second.endpoint ||
            !isRunningLocked(running_, socket_)) {
            return false;
        }
        endpoint = *it->second.endpoint;
        seq = it->second.next_rx_seq++;
        socket = socket_;
    }

    OtaAudioPacket packet;
    packet.header.lease_id = lease_id;
    packet.header.seq = seq;
    packet.header.start_sample = start_sample;
    packet.samples.assign(samples.begin(), samples.end());
    const auto bytes = serializeAudioPacket(packet);

    const auto sent = ::sendto(socket,
                               reinterpret_cast<const char*>(bytes.data()),
                               bytes.size(),
                               0,
                               reinterpret_cast<const sockaddr*>(&endpoint.addr),
                               endpoint.len);
    return sent == static_cast<ultra::tnc::socket_io_result_t>(bytes.size());
}

void UdpAudioPlane::run() {
    std::array<uint8_t, kMaxDatagramBytes> buffer{};
    while (true) {
        ultra::tnc::socket_t socket = ultra::tnc::kInvalidSocket;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_) {
                break;
            }
            socket = socket_;
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
            std::lock_guard<std::mutex> lock(mutex_);
            if (!running_) {
                break;
            }
            continue;
        }
        handleDatagram(std::span<const uint8_t>(buffer.data(), static_cast<size_t>(n)),
                       peer,
                       peer_len);
    }
}

void UdpAudioPlane::handleDatagram(std::span<const uint8_t> bytes,
                                   const sockaddr_storage& peer,
                                   ultra::tnc::socket_len_t peer_len) {
    auto packet = parseAudioPacket(bytes);
    if (!packet) {
        return;
    }

    ReceivedAudioPacket ready;
    PacketCallback callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = leases_.find(packet->header.lease_id);
        if (it == leases_.end()) {
            return;
        }
        it->second.endpoint = Endpoint{.addr = peer, .len = peer_len};
        ready = {
            .lease_id = it->second.lease_id,
            .seq = packet->header.seq,
            .session_id = it->second.session_id,
            .station_id = it->second.station_id,
            .start_sample = packet->header.start_sample,
            .samples = std::move(packet->samples),
        };
        callback = callback_;
    }

    if (!callback) {
        return;
    }
    callback(ready);
}

}  // namespace ultra::ota_simulator_service
