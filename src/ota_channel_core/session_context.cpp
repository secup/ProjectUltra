#include "ota_channel_core/session_context.hpp"

#include <stdexcept>
#include <utility>

namespace ultra::ota_channel_core {

namespace {

std::string streamNameForSession(std::string_view session_id,
                                 std::string_view stream_name) {
    std::string name;
    name.reserve(session_id.size() + stream_name.size() + 1);
    name.append(session_id);
    name.push_back(':');
    name.append(stream_name);
    return name;
}

}  // namespace

SessionContext::SessionContext(SessionConfig config)
    : config_(std::move(config)),
      rng_root_(config_.seed),
      channel_(createChannelModel(config_.channelConfig(),
                                  rng_root_,
                                  streamNameForSession(config_.session_id, "channel"))) {
    if (config_.session_id.empty()) {
        throw std::invalid_argument("session_id is required");
    }
    if (config_.display_name.empty()) {
        config_.display_name = config_.session_id;
    }
    appendEventLocked("session_created", {}, 0);
}

bool SessionContext::registerStation(std::string station_id) {
    if (station_id.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (stations_.size() >= config_.station_cap) {
        return false;
    }

    auto [_, inserted] = stations_.insert(station_id);
    if (!inserted) {
        return false;
    }
    appendEventLocked("station_registered", std::move(station_id), 0);
    return true;
}

bool SessionContext::leaveStation(std::string_view station_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = stations_.find(std::string(station_id));
    if (it == stations_.end()) {
        return false;
    }
    stations_.erase(it);
    appendEventLocked("station_left", std::string(station_id), 0);
    return true;
}

bool SessionContext::hasStation(std::string_view station_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stations_.find(std::string(station_id)) != stations_.end();
}

ChannelConfig SessionContext::currentChannelConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.channelConfig();
}

void SessionContext::setChannel(ChannelConfig config) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.default_channel_model = config.type;
    config_.default_snr_db = config.snr_db;
    config_.seed = config.seed;
    config_.sample_rate = config.sample_rate;
    rng_root_ = RngRoot(config_.seed);
    channel_ = createChannelModel(config_.channelConfig(),
                                  rng_root_,
                                  streamNameForSession(config_.session_id, "channel"));
    appendEventLocked("channel_set", {}, 0);
}

size_t SessionContext::stationCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stations_.size();
}

std::vector<std::string> SessionContext::listStations() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {stations_.begin(), stations_.end()};
}

bool SessionContext::submitTransmit(std::string_view station_id,
                                    uint64_t start_sample,
                                    std::span<const float> samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stations_.find(std::string(station_id)) == stations_.end()) {
        return false;
    }
    mixer_.submit(std::string(station_id), start_sample, samples);
    if (capture_.enabled) {
        capture_.tx_samples += samples.size();
    }
    appendEventLocked("tx", std::string(station_id), start_sample);
    return true;
}

bool SessionContext::receiveForStation(std::string_view station_id,
                                       uint64_t start_sample,
                                       size_t count,
                                       std::vector<float>& output) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stations_.find(std::string(station_id)) == stations_.end()) {
        output.clear();
        return false;
    }

    std::vector<float> mixed;
    mixer_.mixForReceiver(station_id, start_sample, count, mixed);
    channel_->process(mixed, output);
    if (capture_.enabled) {
        capture_.rx_samples += output.size();
    }
    appendEventLocked("rx", std::string(station_id), start_sample);
    return true;
}

std::vector<float> SessionContext::receiveForStation(std::string_view station_id,
                                                     uint64_t start_sample,
                                                     size_t count) {
    std::vector<float> output;
    receiveForStation(station_id, start_sample, count, output);
    return output;
}

void SessionContext::discardBefore(uint64_t sample_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    mixer_.discardBefore(sample_index);
    appendEventLocked("discard_before", {}, sample_index);
}

size_t SessionContext::pendingAudioBlocks() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mixer_.pendingBlocks();
}

RngStream SessionContext::rngStream(std::string_view name, uint64_t index) const {
    return rng_root_.stream(streamNameForSession(config_.session_id, name), index);
}

uint32_t SessionContext::rngChildSeed(std::string_view name, uint64_t index) const {
    return rng_root_.childSeed(streamNameForSession(config_.session_id, name), index);
}

void SessionContext::setCaptureEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    capture_.enabled = enabled;
    appendEventLocked(enabled ? "capture_enabled" : "capture_disabled", {}, 0);
}

CaptureState SessionContext::captureState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return capture_;
}

void SessionContext::appendEvent(std::string type,
                                 std::string station_id,
                                 uint64_t sample_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    appendEventLocked(std::move(type), std::move(station_id), sample_index);
}

std::vector<SessionEvent> SessionContext::eventLog() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
}

void SessionContext::appendEventLocked(std::string type,
                                       std::string station_id,
                                       uint64_t sample_index) {
    events_.push_back({
        .sequence = next_event_sequence_++,
        .sample_index = sample_index,
        .type = std::move(type),
        .station_id = std::move(station_id),
    });
}

}  // namespace ultra::ota_channel_core
