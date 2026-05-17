#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "ota_simulator.grpc.pb.h"
#include "tnc/socket_compat.hpp"

namespace ultra::gui {

struct OtaAudioBackendConfig {
    std::string grpc_target;
    std::string udp_target;
    std::string token;
    std::string station_id;
    std::string session_id = "lobby";
};

enum class OtaAudioConnectionState {
    Disconnected,
    Connecting,
    Connected,
    Reconnecting,
    Failed
};

struct OtaAudioStatus {
    OtaAudioConnectionState state = OtaAudioConnectionState::Disconnected;
    std::string text = "Disconnected";
    int reconnect_attempt = 0;
};

class OtaAudioBackend {
public:
    OtaAudioBackend();
    ~OtaAudioBackend();

    OtaAudioBackend(const OtaAudioBackend&) = delete;
    OtaAudioBackend& operator=(const OtaAudioBackend&) = delete;

    bool start(OtaAudioBackendConfig config, std::string* error = nullptr);
    void close();

    bool queueTxSamples(std::span<const float> samples, std::string* error = nullptr);
    std::vector<float> getRxSamples(size_t max_samples);
    size_t getRxBufferSize() const;

    OtaAudioStatus status() const;
    bool isConnected() const;
    bool waitForConnected(std::chrono::milliseconds timeout);

private:
    using Stub = projectultra::otasim::v1::OtaSimulatorControl;

    struct UdpTarget {
        std::string host;
        uint16_t port = 0;
    };

    bool connectOnce(std::string* error);
    bool openUdpSocket(const UdpTarget& target, std::string* error);
    bool sendSamplesLocked(uint64_t start_sample, std::span<const float> samples, std::string* error);
    bool sendPrimeLocked(std::string* error);
    bool heartbeat(std::string* error);
    void leaveSession();
    void closeTransport();
    void controlLoop();
    void rxLoop();
    void pushRxPacket(uint64_t start_sample, std::span<const float> samples);
    void drainReadyRxLocked();
    void setStatus(OtaAudioConnectionState state, std::string text, int attempt = 0);
    void setStatusLocked(OtaAudioConnectionState state, std::string text, int attempt = 0);

    static std::optional<UdpTarget> parseHostPort(const std::string& value, std::string* error);
    static std::optional<UdpTarget> resolveUdpTarget(const std::string& host, uint16_t port, std::string* error);

    OtaAudioBackendConfig config_;
    mutable std::mutex mutex_;
    OtaAudioStatus status_;
    bool running_ = false;
    bool connected_ = false;
    bool joined_session_ = false;
    uint64_t lease_id_ = 0;
    uint32_t max_packet_samples_ = 0;
    uint64_t tx_sample_cursor_ = 0;
    uint64_t tx_seq_ = 0;

    ultra::tnc::WinsockInit winsock_;
    ultra::tnc::socket_t udp_socket_ = ultra::tnc::kInvalidSocket;
    sockaddr_storage udp_server_{};
    ultra::tnc::socket_len_t udp_server_len_ = 0;

    std::shared_ptr<grpc::Channel> channel_;
    std::unique_ptr<Stub::StubInterface> stub_;

    std::thread control_thread_;
    std::thread rx_thread_;

    bool rx_started_ = false;
    uint64_t rx_next_sample_ = 0;
    std::map<uint64_t, std::vector<float>> rx_pending_;
    std::vector<float> rx_buffer_;
};

}  // namespace ultra::gui
