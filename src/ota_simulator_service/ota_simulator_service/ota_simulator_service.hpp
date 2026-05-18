#pragma once

#include "ota_channel_core/session_manager.hpp"
#include "ota_simulator.grpc.pb.h"
#include "ota_simulator_service/auth_allowlist.hpp"
#include "ota_simulator_service/audio_plane.hpp"
#include "ota_simulator_service/capture_writer.hpp"

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <grpcpp/grpcpp.h>

namespace ultra::ota_simulator_service {

struct OtaSimulatorServiceConfig {
    std::string udp_bind_host = "127.0.0.1";
    uint16_t udp_bind_port = 0;
    std::filesystem::path capture_root = "captures";
    std::shared_ptr<const std::vector<float>> real_hf_loop_noise;
    ultra::ota_channel_core::SessionConfig lobby_config =
        ultra::ota_channel_core::SessionManager::defaultLobbyConfig();
};

class OtaSimulatorService final
    : public projectultra::otasim::v1::OtaSimulatorControl::Service {
public:
    OtaSimulatorService(AuthAllowlist auth, OtaSimulatorServiceConfig config);
    ~OtaSimulatorService() override;

    OtaSimulatorService(const OtaSimulatorService&) = delete;
    OtaSimulatorService& operator=(const OtaSimulatorService&) = delete;

    bool start(std::string* error = nullptr);
    void beginDraining();
    void shutdown();
    bool healthy() const;
    uint16_t audioPort() const { return audio_plane_.port(); }
    const std::string& audioHost() const { return config_.udp_bind_host; }
    ultra::ota_channel_core::SessionManager& sessionManager() { return sessions_; }

    grpc::Status RegisterStation(grpc::ServerContext* context,
                                 const projectultra::otasim::v1::RegisterStationRequest* request,
                                 projectultra::otasim::v1::StationLease* response) override;
    grpc::Status NegotiateAudio(grpc::ServerContext* context,
                                const projectultra::otasim::v1::NegotiateAudioRequest* request,
                                projectultra::otasim::v1::AudioLease* response) override;
    grpc::Status Heartbeat(grpc::ServerContext* context,
                           const projectultra::otasim::v1::HeartbeatRequest* request,
                           projectultra::otasim::v1::HeartbeatResponse* response) override;
    grpc::Status CreateSession(grpc::ServerContext* context,
                               const projectultra::otasim::v1::CreateSessionRequest* request,
                               projectultra::otasim::v1::SessionInfo* response) override;
    grpc::Status ListSessions(grpc::ServerContext* context,
                              const projectultra::otasim::v1::ListSessionsRequest* request,
                              projectultra::otasim::v1::ListSessionsResponse* response) override;
    grpc::Status JoinSession(grpc::ServerContext* context,
                             const projectultra::otasim::v1::JoinSessionRequest* request,
                             projectultra::otasim::v1::JoinSessionResponse* response) override;
    grpc::Status LeaveSession(grpc::ServerContext* context,
                              const projectultra::otasim::v1::LeaveSessionRequest* request,
                              google::protobuf::Empty* response) override;
    grpc::Status GetChannel(grpc::ServerContext* context,
                            const projectultra::otasim::v1::GetChannelRequest* request,
                            projectultra::otasim::v1::ChannelState* response) override;
    grpc::Status SetChannel(grpc::ServerContext* context,
                            const projectultra::otasim::v1::SetChannelRequest* request,
                            projectultra::otasim::v1::CommandAck* response) override;
    grpc::Status InjectEffect(grpc::ServerContext* context,
                              const projectultra::otasim::v1::InjectEffectRequest* request,
                              projectultra::otasim::v1::CommandAck* response) override;
    grpc::Status CancelEffect(grpc::ServerContext* context,
                              const projectultra::otasim::v1::CancelEffectRequest* request,
                              projectultra::otasim::v1::CommandAck* response) override;
    grpc::Status StartCapture(grpc::ServerContext* context,
                              const projectultra::otasim::v1::StartCaptureRequest* request,
                              projectultra::otasim::v1::CaptureInfo* response) override;
    grpc::Status StopCapture(grpc::ServerContext* context,
                             const projectultra::otasim::v1::StopCaptureRequest* request,
                             projectultra::otasim::v1::CaptureInfo* response) override;
    grpc::Status StreamEvents(
        grpc::ServerContext* context,
        const projectultra::otasim::v1::StreamEventsRequest* request,
        grpc::ServerWriter<projectultra::otasim::v1::ServerEvent>* writer) override;
    grpc::Status Health(grpc::ServerContext* context,
                        const projectultra::otasim::v1::HealthRequest* request,
                        projectultra::otasim::v1::HealthResponse* response) override;

private:
    struct RegisteredStation {
        AuthPrincipal principal;
    };

    struct StoredEvent {
        projectultra::otasim::v1::ServerEvent proto;
        std::string payload_json;
    };

    grpc::Status authenticate(grpc::ServerContext* context,
                              AuthPrincipal* principal) const;
    grpc::Status requireAdmin(const AuthPrincipal& principal) const;
    std::string stationIdFor(const std::string& requested,
                             const AuthPrincipal& principal) const;
    projectultra::otasim::v1::SessionInfo sessionInfo(
        const std::shared_ptr<ultra::ota_channel_core::SessionContext>& session) const;
    projectultra::otasim::v1::ChannelState channelState(
        const std::shared_ptr<ultra::ota_channel_core::SessionContext>& session) const;
    void fillStationLease(const std::string& station_id,
                          const AuthPrincipal& principal,
                          projectultra::otasim::v1::StationLease* lease) const;
    void emitEvent(std::string session_id,
                   uint64_t sample_index,
                   std::string type,
                   std::string payload_json);
    void onAudioPacket(const ReceivedAudioPacket& packet);
    void startSessionClock();
    void stopSessionClock();
    void runSessionClock();
    void processSessionClockTick(
        const std::shared_ptr<ultra::ota_channel_core::SessionContext>& session);
    void stopActiveCaptures();
    SessionCaptureWriter* captureForSessionLocked(std::string_view session_id);

    AuthAllowlist auth_;
    OtaSimulatorServiceConfig config_;
    ultra::ota_channel_core::SessionManager sessions_;
    UdpAudioPlane audio_plane_;
    std::atomic<bool> draining_{false};
    std::atomic<bool> session_clock_running_{false};
    std::thread session_clock_thread_;
    mutable std::mutex mutex_;
    std::map<std::string, RegisteredStation> registered_;
    std::map<std::string, SessionCaptureWriter> captures_;
    std::map<std::string, std::vector<projectultra::otasim::v1::ActiveEffect>> active_effects_;
    std::vector<StoredEvent> events_;
    std::condition_variable events_cv_;
    uint64_t next_event_id_ = 1;
    uint64_t next_command_id_ = 1;
};

}  // namespace ultra::ota_simulator_service
