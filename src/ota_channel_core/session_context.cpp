#include "ota_channel_core/session_context.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ultra::ota_channel_core {

namespace {

constexpr uint32_t kDefaultTickIntervalMs = 10;
constexpr uint32_t kMaxTxQueuedAudioMs = 20'000;
constexpr uint32_t kMaxRxQueuedAudioMs = 200;

std::string streamNameForSession(std::string_view session_id,
                                 std::string_view stream_name) {
    std::string name;
    name.reserve(session_id.size() + stream_name.size() + 1);
    name.append(session_id);
    name.push_back(':');
    name.append(stream_name);
    return name;
}

size_t samplesForMs(uint32_t sample_rate, uint32_t ms) {
    return std::max<size_t>(1, static_cast<size_t>(
        (static_cast<uint64_t>(sample_rate) * ms) / 1000u));
}

}  // namespace

SessionContext::SessionContext(SessionConfig config)
    : config_(std::move(config)),
      rng_root_(config_.seed),
      channel_(createChannelModel(config_.channelConfig(),
                                  rng_root_,
                                  streamNameForSession(config_.session_id, "channel"))),
      tick_samples_(samplesForMs(config_.sample_rate, kDefaultTickIntervalMs)),
      max_tx_queue_samples_(samplesForMs(config_.sample_rate, kMaxTxQueuedAudioMs)),
      max_rx_queue_samples_(samplesForMs(config_.sample_rate, kMaxRxQueuedAudioMs)) {
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
    audio_queues_.try_emplace(station_id);
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
    audio_queues_.erase(std::string(station_id));
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
    config_.real_hf_loop_noise = std::move(config.real_hf_loop_noise);
    tick_samples_ = samplesForMs(config_.sample_rate, kDefaultTickIntervalMs);
    max_tx_queue_samples_ = samplesForMs(config_.sample_rate, kMaxTxQueuedAudioMs);
    max_rx_queue_samples_ = samplesForMs(config_.sample_rate, kMaxRxQueuedAudioMs);
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

bool SessionContext::enqueueTransmit(std::string_view station_id,
                                     std::span<const float> samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = audio_queues_.find(std::string(station_id));
    if (it == audio_queues_.end()) {
        return false;
    }
    if (samples.empty()) {
        return true;
    }
    auto& queue = it->second.tx_inbox;
    queue.insert(queue.end(), samples.begin(), samples.end());
    trimQueueLocked(queue);
    appendEventLocked("tx_enqueued", std::string(station_id), session_clock_samples_);
    return true;
}

SessionClockTick SessionContext::advanceSessionClock() {
    std::lock_guard<std::mutex> lock(mutex_);

    SessionClockTick tick;
    tick.start_sample = session_clock_samples_;
    tick.sample_count = tick_samples_;
    if (tick_samples_ == 0) {
        return tick;
    }

    std::map<std::string, std::vector<float>> tx_by_station;
    for (const auto& station_id : stations_) {
        auto& queue = audio_queues_[station_id].tx_inbox;
        std::vector<float> samples(tick_samples_, 0.0f);
        const size_t available = std::min(tick_samples_, queue.size());
        for (size_t i = 0; i < available; ++i) {
            samples[i] = queue.front();
            queue.pop_front();
        }
        if (available > 0) {
            tick.tx_blocks.push_back({
                .station_id = station_id,
                .start_sample = tick.start_sample,
                .samples = samples,
            });
            if (capture_.enabled) {
                capture_.tx_samples += samples.size();
            }
        }
        tx_by_station.emplace(station_id, std::move(samples));
    }

    for (const auto& receiver_id : stations_) {
        std::vector<float> mixed(tick_samples_, 0.0f);
        for (const auto& [sender_id, samples] : tx_by_station) {
            if (sender_id == receiver_id) {
                continue;
            }
            for (size_t i = 0; i < tick_samples_; ++i) {
                mixed[i] += samples[i];
            }
        }

        std::vector<float> rx;
        channel_->process(mixed, rx);
        auto& outbox = audio_queues_[receiver_id].rx_outbox;
        outbox.push_back({
            .start_sample = tick.start_sample,
            .samples = rx,
        });
        trimOutboxLocked(outbox);
        tick.rx_blocks.push_back({
            .station_id = receiver_id,
            .start_sample = tick.start_sample,
            .samples = std::move(rx),
        });
        if (capture_.enabled) {
            capture_.rx_samples += tick_samples_;
        }
    }

    session_clock_samples_ += tick_samples_;
    return tick;
}

std::vector<SessionAudioBlock> SessionContext::drainReceiveOutbox() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SessionAudioBlock> out;
    for (auto& [station_id, queues] : audio_queues_) {
        while (!queues.rx_outbox.empty()) {
            auto block = std::move(queues.rx_outbox.front());
            queues.rx_outbox.pop_front();
            out.push_back({
                .station_id = station_id,
                .start_sample = block.start_sample,
                .samples = std::move(block.samples),
            });
        }
    }
    return out;
}

uint64_t SessionContext::sessionClockSamples() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_clock_samples_;
}

size_t SessionContext::pendingTransmitSamples(std::string_view station_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = audio_queues_.find(std::string(station_id));
    return it == audio_queues_.end() ? 0 : it->second.tx_inbox.size();
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

void SessionContext::trimQueueLocked(std::deque<float>& queue) const {
    while (queue.size() > max_tx_queue_samples_) {
        queue.pop_front();
    }
}

void SessionContext::trimOutboxLocked(std::deque<QueuedAudioBlock>& queue) const {
    size_t total = 0;
    for (const auto& block : queue) {
        total += block.samples.size();
    }
    while (total > max_rx_queue_samples_ && !queue.empty()) {
        total -= queue.front().samples.size();
        queue.pop_front();
    }
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
