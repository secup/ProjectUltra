#include "ota_simulator_service/ota_simulator_service.hpp"

#include "ota_channel_core/models.hpp"
#include "ultra/version.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <sstream>
#include <utility>

namespace ultra::ota_simulator_service {
namespace {

namespace pb = projectultra::otasim::v1;

google::protobuf::Timestamp nowTimestamp() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now);
    const auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now - seconds);
    google::protobuf::Timestamp ts;
    ts.set_seconds(seconds.count());
    ts.set_nanos(static_cast<int32_t>(nanos.count()));
    return ts;
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::optional<ultra::ota_channel_core::ChannelType> parseChannelType(std::string value) {
    value = lower(std::move(value));
    if (value.empty() || value == "passthrough" || value == "null") {
        return ultra::ota_channel_core::ChannelType::PASSTHROUGH;
    }
    if (value == "awgn") {
        return ultra::ota_channel_core::ChannelType::AWGN;
    }
    if (value == "good" || value == "watterson_good") {
        return ultra::ota_channel_core::ChannelType::GOOD;
    }
    if (value == "moderate" || value == "watterson_moderate") {
        return ultra::ota_channel_core::ChannelType::MODERATE;
    }
    if (value == "poor" || value == "watterson_poor") {
        return ultra::ota_channel_core::ChannelType::POOR;
    }
    if (value == "flutter" || value == "watterson_flutter") {
        return ultra::ota_channel_core::ChannelType::FLUTTER;
    }
    return std::nullopt;
}

std::string jsonPair(std::string_view key, std::string_view value) {
    std::ostringstream out;
    out << "{\"" << key << "\":\"" << value << "\"}";
    return out.str();
}

void fillCaptureInfo(const CaptureSummary& summary, pb::CaptureInfo* response) {
    response->set_session_id(summary.session_id);
    response->set_active(summary.active);
    response->set_capture_path(summary.capture_path.string());
    response->set_manifest_path(summary.manifest_path.string());
    response->set_started_at_sample(summary.started_at_sample);
    response->set_stopped_at_sample(summary.stopped_at_sample);
    response->set_tx_sample_count(summary.tx_sample_count);
    response->set_rx_sample_count(summary.rx_sample_count);
}

}  // namespace

OtaSimulatorService::OtaSimulatorService(AuthAllowlist auth,
                                         OtaSimulatorServiceConfig config)
    : auth_(std::move(auth)),
      config_(std::move(config)),
      sessions_(config_.lobby_config) {}

OtaSimulatorService::~OtaSimulatorService() {
    shutdown();
}

bool OtaSimulatorService::start(std::string* error) {
    draining_.store(false);
    return audio_plane_.start(
        config_.udp_bind_host,
        config_.udp_bind_port,
        [this](const ReceivedAudioPacket& packet) { onAudioPacket(packet); },
        error);
}

void OtaSimulatorService::beginDraining() {
    draining_.store(true);
    events_cv_.notify_all();
}

void OtaSimulatorService::shutdown() {
    beginDraining();
    audio_plane_.stop();
    stopActiveCaptures();
    events_cv_.notify_all();
}

bool OtaSimulatorService::healthy() const {
    return !draining_.load() && audio_plane_.running();
}

grpc::Status OtaSimulatorService::RegisterStation(
    grpc::ServerContext* context,
    const pb::RegisterStationRequest* request,
    pb::StationLease* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }

    const std::string station_id = stationIdFor(request->station_id(), principal);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = registered_.find(station_id);
        if (it != registered_.end() && it->second.principal.token != principal.token) {
            return grpc::Status(grpc::StatusCode::ALREADY_EXISTS,
                                "station id is registered by another token");
        }
        registered_[station_id] = RegisteredStation{.principal = principal};
    }

    fillStationLease(station_id, principal, response);
    emitEvent({}, 0, "station_registered", jsonPair("station_id", station_id));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::NegotiateAudio(
    grpc::ServerContext* context,
    const pb::NegotiateAudioRequest* request,
    pb::AudioLease* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }

    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }
    const std::string station_id = stationIdFor(request->station_id(), principal);
    if (!session->hasStation(station_id)) {
        return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                            "station must join the session before negotiating audio");
    }

    const uint64_t lease_id = audio_plane_.addLease(request->session_id(), station_id);
    response->set_lease_id(lease_id);
    response->set_session_id(request->session_id());
    response->set_station_id(station_id);
    response->set_udp_host(config_.udp_bind_host);
    response->set_udp_port(audio_plane_.port());
    response->set_sample_rate(ultra::ota_channel_core::kDefaultSampleRate);
    response->set_sample_format(kOtaAudioFormatF32LE);
    response->set_channel_count(kOtaAudioChannelCountMono);
    response->set_max_packet_samples(
        static_cast<uint32_t>((8192 - kOtaAudioHeaderBytes) / sizeof(float)));
    emitEvent(request->session_id(), 0, "audio_negotiated", jsonPair("station_id", station_id));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::Heartbeat(
    grpc::ServerContext* context,
    const pb::HeartbeatRequest* request,
    pb::HeartbeatResponse* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    (void)request;
    response->set_ok(true);
    response->set_message("ok");
    *response->mutable_server_time() = nowTimestamp();
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::CreateSession(
    grpc::ServerContext* context,
    const pb::CreateSessionRequest* request,
    pb::SessionInfo* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    (void)principal;

    if (request->session_id().empty()) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "session_id is required");
    }
    auto channel_type = parseChannelType(request->channel_model());
    if (!channel_type) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "unknown channel model");
    }

    ultra::ota_channel_core::SessionConfig config;
    config.display_name = request->display_name().empty()
        ? request->session_id()
        : request->display_name();
    config.default_channel_model = *channel_type;
    config.default_snr_db = request->snr_db() == 0.0 ? 20.0f
                                                     : static_cast<float>(request->snr_db());
    config.seed = request->seed();
    config.station_cap = request->station_cap() == 0 ? 16 : request->station_cap();

    auto session = sessions_.createSession(request->session_id(), std::move(config));
    if (!session) {
        return grpc::Status(grpc::StatusCode::ALREADY_EXISTS, "session already exists");
    }
    *response = sessionInfo(session);
    emitEvent(request->session_id(), 0, "session_created", "{}");
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::ListSessions(
    grpc::ServerContext* context,
    const pb::ListSessionsRequest* request,
    pb::ListSessionsResponse* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    (void)request;

    for (const auto& id : sessions_.listSessions()) {
        if (auto session = sessions_.getSession(id)) {
            *response->add_sessions() = sessionInfo(session);
        }
    }
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::JoinSession(
    grpc::ServerContext* context,
    const pb::JoinSessionRequest* request,
    pb::JoinSessionResponse* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }

    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }

    const std::string station_id = stationIdFor(request->station_id(), principal);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        registered_[station_id] = RegisteredStation{.principal = principal};
    }
    if (!session->hasStation(station_id) && !session->registerStation(station_id)) {
        return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED,
                            "station cap reached or duplicate station");
    }

    *response->mutable_session() = sessionInfo(session);
    fillStationLease(station_id, principal, response->mutable_station());
    emitEvent(request->session_id(), 0, "station_joined", jsonPair("station_id", station_id));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::LeaveSession(
    grpc::ServerContext* context,
    const pb::LeaveSessionRequest* request,
    google::protobuf::Empty* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    (void)response;

    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }
    const std::string station_id = stationIdFor(request->station_id(), principal);
    if (!session->leaveStation(station_id)) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "station not in session");
    }
    emitEvent(request->session_id(), 0, "station_left", jsonPair("station_id", station_id));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::GetChannel(
    grpc::ServerContext* context,
    const pb::GetChannelRequest* request,
    pb::ChannelState* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }
    *response = channelState(session);
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::SetChannel(
    grpc::ServerContext* context,
    const pb::SetChannelRequest* request,
    pb::CommandAck* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }
    auto channel_type = parseChannelType(request->model());
    if (!channel_type) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "unknown channel model");
    }

    session->setChannel({
        .type = *channel_type,
        .snr_db = static_cast<float>(request->snr_db()),
        .seed = request->seed(),
        .sample_rate = ultra::ota_channel_core::kDefaultSampleRate,
    });

    uint64_t command_id = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        command_id = next_command_id_++;
        active_effects_[request->session_id()].clear();
    }
    response->set_accepted(true);
    response->set_message("channel updated");
    response->set_command_id("cmd-" + std::to_string(command_id));
    emitEvent(request->session_id(), 0, "channel_set", jsonPair("model", request->model()));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::InjectEffect(
    grpc::ServerContext* context,
    const pb::InjectEffectRequest* request,
    pb::CommandAck* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }
    if (request->effect_type().empty()) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, "effect_type is required");
    }

    uint64_t command_id = 0;
    std::string effect_id;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        command_id = next_command_id_++;
        effect_id = "effect-" + std::to_string(command_id);
        pb::ActiveEffect effect;
        effect.set_effect_id(effect_id);
        effect.set_effect_type(request->effect_type());
        effect.set_start_sample(request->start_sample());
        effect.set_duration_samples(request->duration_samples());
        effect.set_params_json(request->params_json());
        active_effects_[request->session_id()].push_back(std::move(effect));
    }
    session->appendEvent("effect_injected", {}, request->start_sample());
    response->set_accepted(true);
    response->set_message("effect recorded");
    response->set_command_id(effect_id);
    emitEvent(request->session_id(), request->start_sample(),
              "effect_injected", jsonPair("effect_id", effect_id));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::CancelEffect(
    grpc::ServerContext* context,
    const pb::CancelEffectRequest* request,
    pb::CommandAck* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }

    bool removed = false;
    uint64_t command_id = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        command_id = next_command_id_++;
        auto& effects = active_effects_[request->session_id()];
        for (auto it = effects.begin(); it != effects.end(); ++it) {
            if (it->effect_id() == request->effect_id()) {
                effects.erase(it);
                removed = true;
                break;
            }
        }
    }
    if (removed) {
        session->appendEvent("effect_cancelled", {}, 0);
        emitEvent(request->session_id(), 0, "effect_cancelled",
                  jsonPair("effect_id", request->effect_id()));
    }
    response->set_accepted(removed);
    response->set_message(removed ? "effect cancelled" : "effect not found");
    response->set_command_id("cmd-" + std::to_string(command_id));
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::StartCapture(
    grpc::ServerContext* context,
    const pb::StartCaptureRequest* request,
    pb::CaptureInfo* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }

    std::string error;
    CaptureSummary summary;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& writer = captures_[request->session_id()];
        if (!writer.start(config_.capture_root,
                          request->session_id(),
                          request->start_sample(),
                          session->config(),
                          session->listStations(),
                          &error)) {
            return grpc::Status(grpc::StatusCode::INTERNAL, error);
        }
        summary = writer.summary();
    }
    session->setCaptureEnabled(true);
    fillCaptureInfo(summary, response);
    emitEvent(request->session_id(), request->start_sample(), "capture_started", "{}");
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::StopCapture(
    grpc::ServerContext* context,
    const pb::StopCaptureRequest* request,
    pb::CaptureInfo* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    auto session = sessions_.getSession(request->session_id());
    if (!session) {
        return grpc::Status(grpc::StatusCode::NOT_FOUND, "session not found");
    }

    session->setCaptureEnabled(false);
    std::string error;
    CaptureSummary summary;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = captures_.find(request->session_id());
        if (it == captures_.end()) {
            return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION, "capture not active");
        }
        summary = it->second.stop(session->config(), session->listStations(), &error);
    }
    if (!error.empty()) {
        return grpc::Status(grpc::StatusCode::INTERNAL, error);
    }
    fillCaptureInfo(summary, response);
    emitEvent(request->session_id(), summary.stopped_at_sample, "capture_stopped", "{}");
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::StreamEvents(
    grpc::ServerContext* context,
    const pb::StreamEventsRequest* request,
    grpc::ServerWriter<pb::ServerEvent>* writer) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }

    size_t cursor = 0;
    while (!context->IsCancelled()) {
        StoredEvent event;
        bool have_event = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            events_cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {
                return cursor < events_.size() || context->IsCancelled();
            });
            while (cursor < events_.size()) {
                const auto& candidate = events_[cursor++];
                if (request->session_id().empty() ||
                    candidate.proto.session_id() == request->session_id()) {
                    event = candidate;
                    have_event = true;
                    break;
                }
            }
        }
        if (have_event && !writer->Write(event.proto)) {
            break;
        }
    }
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::Health(
    grpc::ServerContext* context,
    const pb::HealthRequest* request,
    pb::HealthResponse* response) {
    AuthPrincipal principal;
    auto status = authenticate(context, &principal);
    if (!status.ok()) {
        return status;
    }
    (void)request;
    const bool ok = healthy();
    response->set_ok(ok);
    response->set_message(ok ? "ok" : "not serving");
    *response->mutable_server_time() = nowTimestamp();
    return grpc::Status::OK;
}

grpc::Status OtaSimulatorService::authenticate(grpc::ServerContext* context,
                                               AuthPrincipal* principal) const {
    const auto& metadata = context->client_metadata();
    const auto it = metadata.find("authorization");
    if (it == metadata.end()) {
        return grpc::Status(grpc::StatusCode::UNAUTHENTICATED,
                            "missing authorization bearer token");
    }
    const std::string authorization(it->second.data(), it->second.length());
    auto token = bearerTokenFromAuthorization(authorization);
    if (!token) {
        return grpc::Status(grpc::StatusCode::UNAUTHENTICATED,
                            "invalid authorization bearer token");
    }
    auto authenticated = auth_.authenticate(*token);
    if (!authenticated) {
        return grpc::Status(grpc::StatusCode::UNAUTHENTICATED,
                            "unknown authorization bearer token");
    }
    if (principal) {
        *principal = *authenticated;
    }
    return grpc::Status::OK;
}

std::string OtaSimulatorService::stationIdFor(const std::string& requested,
                                              const AuthPrincipal& principal) const {
    return requested.empty() ? principal.callsign : requested;
}

pb::SessionInfo OtaSimulatorService::sessionInfo(
    const std::shared_ptr<ultra::ota_channel_core::SessionContext>& session) const {
    pb::SessionInfo info;
    info.set_session_id(session->id());
    info.set_display_name(session->config().display_name);
    info.set_is_lobby(session->config().is_lobby);
    info.set_station_count(static_cast<uint32_t>(session->stationCount()));
    info.set_station_cap(static_cast<uint32_t>(session->config().station_cap));
    *info.mutable_channel() = channelState(session);
    info.set_state(pb::SESSION_STATE_ACTIVE);
    *info.mutable_created_at() = nowTimestamp();
    for (const auto& station : session->listStations()) {
        info.add_stations(station);
    }
    return info;
}

pb::ChannelState OtaSimulatorService::channelState(
    const std::shared_ptr<ultra::ota_channel_core::SessionContext>& session) const {
    const auto config = session->currentChannelConfig();
    pb::ChannelState state;
    state.set_session_id(session->id());
    state.set_model(ultra::ota_channel_core::channelTypeName(config.type));
    state.set_snr_db(config.snr_db);
    state.set_channel_seed(config.seed);
    state.set_applied_at_sample(0);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = active_effects_.find(session->id());
    if (it != active_effects_.end()) {
        for (const auto& effect : it->second) {
            *state.add_active_effects() = effect;
        }
    }
    return state;
}

void OtaSimulatorService::fillStationLease(const std::string& station_id,
                                           const AuthPrincipal& principal,
                                           pb::StationLease* lease) const {
    lease->set_station_id(station_id);
    lease->set_callsign(principal.callsign);
    lease->set_label(principal.label);
}

void OtaSimulatorService::emitEvent(std::string session_id,
                                    uint64_t sample_index,
                                    std::string type,
                                    std::string payload_json) {
    StoredEvent stored;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stored.proto.set_event_id("ev-" + std::to_string(next_event_id_++));
        stored.proto.set_session_id(session_id);
        stored.proto.set_sample_index(sample_index);
        stored.proto.set_type(type);
        stored.proto.set_payload_json(payload_json);
        stored.payload_json = payload_json;
        events_.push_back(stored);
        if (!session_id.empty()) {
            if (auto* capture = captureForSessionLocked(session_id)) {
                capture->recordEvent({
                    .event_id = stored.proto.event_id(),
                    .session_id = session_id,
                    .sample_index = sample_index,
                    .type = type,
                    .payload_json = payload_json,
                });
            }
        }
    }
    events_cv_.notify_all();
}

void OtaSimulatorService::onAudioPacket(const ReceivedAudioPacket& packet) {
    auto session = sessions_.getSession(packet.session_id);
    if (!session) {
        return;
    }
    if (!session->submitTransmit(packet.station_id, packet.start_sample, packet.samples)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (auto* capture = captureForSessionLocked(packet.session_id)) {
            capture->recordTx(packet.station_id, packet.start_sample, packet.samples);
        }
    }

    const auto leases = audio_plane_.leasesForSession(packet.session_id);
    for (const auto& lease : leases) {
        if (!lease.has_endpoint) {
            continue;
        }
        if (lease.station_id == packet.station_id) {
            continue;
        }
        std::vector<float> rx;
        if (!session->receiveForStation(lease.station_id,
                                        packet.start_sample,
                                        packet.samples.size(),
                                        rx)) {
            continue;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (auto* capture = captureForSessionLocked(packet.session_id)) {
                capture->recordRx(lease.station_id, packet.start_sample, rx);
            }
        }
        (void)audio_plane_.sendAudio(lease.lease_id, packet.start_sample, rx);
    }
    session->discardBefore(packet.start_sample + packet.samples.size());
}

void OtaSimulatorService::stopActiveCaptures() {
    std::vector<std::string> session_ids;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        session_ids.reserve(captures_.size());
        for (const auto& [session_id, _] : captures_) {
            session_ids.push_back(session_id);
        }
    }

    for (const auto& session_id : session_ids) {
        auto session = sessions_.getSession(session_id);
        if (!session) {
            continue;
        }
        const auto config = session->config();
        const auto stations = session->listStations();
        std::string error;
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = captures_.find(session_id);
        if (it != captures_.end() && it->second.active()) {
            (void)it->second.stop(config, stations, &error);
        }
    }
}

SessionCaptureWriter* OtaSimulatorService::captureForSessionLocked(
    std::string_view session_id) {
    auto it = captures_.find(std::string(session_id));
    if (it == captures_.end()) {
        return nullptr;
    }
    return &it->second;
}

}  // namespace ultra::ota_simulator_service
