#pragma once

#include "ota_simulator_service/audio_packet.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "tnc/socket_compat.hpp"

namespace ultra::ota_simulator_service {

class OrderedAudioQueue {
public:
    struct Block {
        uint64_t start_sample = 0;
        std::vector<float> samples;
    };

    bool push(uint64_t start_sample, std::span<const float> samples);
    std::vector<Block> drainReady();
    std::vector<float> readWindow(uint64_t start_sample, size_t count);
    uint64_t nextSample() const { return next_sample_; }

private:
    uint64_t next_sample_ = 0;
    std::map<uint64_t, std::vector<float>> pending_;
};

struct ReceivedAudioPacket {
    uint64_t lease_id = 0;
    uint64_t seq = 0;
    std::string session_id;
    std::string station_id;
    uint64_t start_sample = 0;
    std::vector<float> samples;
};

class UdpAudioPlane {
public:
    struct LeaseSnapshot {
        uint64_t lease_id = 0;
        std::string session_id;
        std::string station_id;
        bool has_endpoint = false;
    };

    using PacketCallback = std::function<void(const ReceivedAudioPacket&)>;

    UdpAudioPlane();
    ~UdpAudioPlane();

    UdpAudioPlane(const UdpAudioPlane&) = delete;
    UdpAudioPlane& operator=(const UdpAudioPlane&) = delete;

    bool start(const std::string& bind_host,
               uint16_t bind_port,
               PacketCallback callback,
               std::string* error = nullptr);
    void stop();
    bool running() const;
    uint16_t port() const;
    std::string host() const;

    uint64_t addLease(std::string session_id, std::string station_id);
    std::vector<LeaseSnapshot> leasesForSession(std::string_view session_id) const;
    bool sendAudio(uint64_t lease_id, uint64_t start_sample, std::span<const float> samples);

private:
    struct Endpoint {
        sockaddr_storage addr{};
        ultra::tnc::socket_len_t len = 0;
    };

    struct LeaseState {
        uint64_t lease_id = 0;
        std::string session_id;
        std::string station_id;
        OrderedAudioQueue queue;
        std::optional<Endpoint> endpoint;
        uint64_t next_rx_seq = 0;
    };

    void run();
    void handleDatagram(std::span<const uint8_t> bytes,
                        const sockaddr_storage& peer,
                        ultra::tnc::socket_len_t peer_len);

    mutable std::mutex mutex_;
    ultra::tnc::WinsockInit winsock_;
    ultra::tnc::socket_t socket_ = ultra::tnc::kInvalidSocket;
    std::string host_ = "127.0.0.1";
    uint16_t port_ = 0;
    bool running_ = false;
    uint64_t next_lease_id_ = 1;
    std::map<uint64_t, LeaseState> leases_;
    PacketCallback callback_;
    std::thread thread_;
};

}  // namespace ultra::ota_simulator_service
