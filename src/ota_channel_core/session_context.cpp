#include "ota_channel_core/session_context.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
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

size_t samplesForMs(uint32_t sample_rate, uint32_t ms) {
    return std::max<size_t>(1, static_cast<size_t>(
        (static_cast<uint64_t>(sample_rate) * ms) / 1000u));
}

void e2eDebugLine(const std::string& line) {
    const char* path = std::getenv("ULTRA_E2E_DEBUG_LOG");
    if (path == nullptr || *path == '\0') {
        return;
    }
    static std::mutex mutex;
    std::lock_guard<std::mutex> lock(mutex);
    static std::string open_path;
    static std::ofstream out;
    if (!out.is_open() || open_path != path) {
        out.close();
        open_path = path;
        out.open(open_path, std::ios::app);
    }
    if (!out) {
        return;
    }
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto epoch_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    out << "epoch_ms=" << epoch_ms << ' ' << line << '\n';
}

float rms(std::span<const float> samples) {
    if (samples.empty()) {
        return 0.0f;
    }
    double sum = 0.0;
    for (float sample : samples) {
        sum += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return static_cast<float>(std::sqrt(sum / static_cast<double>(samples.size())));
}

bool allZero(std::span<const float> samples) {
    return std::all_of(samples.begin(), samples.end(), [](float sample) {
        return sample == 0.0f;
    });
}

}  // namespace

SessionContext::SessionContext(SessionConfig config)
    : config_(std::move(config)),
      rng_root_(config_.seed),
      tick_samples_(samplesForMs(config_.sample_rate, kSessionTickIntervalMs)),
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
    if (station_id.empty() || station_id == kInjectedAudioStationId) {
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
    const std::string station_key = station_id;
    audio_queues_.try_emplace(station_key);
    rx_channels_[station_key] = createRxChannelLocked(station_key);
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
    rx_channels_.erase(std::string(station_id));
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
    tick_samples_ = samplesForMs(config_.sample_rate, kSessionTickIntervalMs);
    max_tx_queue_samples_ = samplesForMs(config_.sample_rate, kMaxTxQueuedAudioMs);
    max_rx_queue_samples_ = samplesForMs(config_.sample_rate, kMaxRxQueuedAudioMs);
    rng_root_ = RngRoot(config_.seed);
    channel_epoch_set_ = false;
    channel_epoch_sample_ = 0;
    if (stations_.empty()) {
        session_clock_samples_ = 0;
        mixer_.clear();
        audio_queues_.clear();
        rx_channels_.clear();
    } else {
        rx_channels_.clear();
        for (const auto& station_id : stations_) {
            rx_channels_[station_id] = createRxChannelLocked(station_id);
        }
    }
    appendEventLocked("channel_set", {}, 0);
}

std::unique_ptr<IChannelModel> SessionContext::createRxChannelLocked(
    std::string_view receiver_id) const {
    return createChannelModel(
        config_.channelConfigFor(receiver_id),
        rng_root_,
        streamNameForSession(config_.session_id,
                             std::string("channel:rx:").append(receiver_id)));
}

IChannelModel* SessionContext::rxChannelForLocked(std::string_view receiver_id) {
    auto it = rx_channels_.find(std::string(receiver_id));
    if (it != rx_channels_.end()) {
        return it->second.get();
    }
    auto channel = createRxChannelLocked(receiver_id);
    auto* ptr = channel.get();
    rx_channels_[std::string(receiver_id)] = std::move(channel);
    return ptr;
}

void SessionContext::maybeSetChannelEpochLocked(uint64_t start_sample,
                                                std::span<const float> samples) {
    if (channel_epoch_set_ || samples.empty() || allZero(samples)) {
        return;
    }
    channel_epoch_set_ = true;
    channel_epoch_sample_ = start_sample;
    std::ostringstream oss;
    oss << "session_channel_epoch session=" << config_.session_id
        << " start=" << channel_epoch_sample_
        << " rms=" << rms(samples);
    e2eDebugLine(oss.str());
}

uint64_t SessionContext::channelSampleIndexLocked(uint64_t session_sample) const {
    if (!channel_epoch_set_ || session_sample < channel_epoch_sample_) {
        return session_sample;
    }
    return session_sample - channel_epoch_sample_;
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
    maybeSetChannelEpochLocked(start_sample, samples);
    mixer_.submit(std::string(station_id), start_sample, samples);
    if (capture_.enabled) {
        capture_.tx_samples += samples.size();
    }
    appendEventLocked("tx", std::string(station_id), start_sample);
    return true;
}

bool SessionContext::injectAudio(uint64_t start_sample,
                                 std::span<const float> samples) {
    if (samples.empty()) {
        return false;
    }
    if (start_sample > std::numeric_limits<uint64_t>::max() -
                           static_cast<uint64_t>(samples.size())) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    maybeSetChannelEpochLocked(start_sample, samples);
    mixer_.submit(std::string(kInjectedAudioStationId), start_sample, samples);
    appendEventLocked("audio_injected", std::string(kInjectedAudioStationId), start_sample);
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
    rxChannelForLocked(station_id)->process(mixed, channelSampleIndexLocked(start_sample), output);
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
    auto& queues = it->second;
    queues.tx_inbox.insert(queues.tx_inbox.end(), samples.begin(), samples.end());
    {
        std::ostringstream oss;
        oss << "session_tx_inbox station=" << station_id
            << " session=" << config_.session_id
            << " clock=" << session_clock_samples_
            << " samples=" << samples.size()
            << " rms=" << rms(samples)
            << " queued_after=" << queues.tx_inbox.size();
        e2eDebugLine(oss.str());
    }
    trimTxQueueLocked(queues);
    appendEventLocked("tx_enqueued", std::string(station_id), session_clock_samples_);
    return true;
}

SessionClockTick SessionContext::advanceSessionClock() {
    std::lock_guard<std::mutex> lock(mutex_);

    SessionClockTick tick;
    tick.start_sample = session_clock_samples_;
    tick.sample_count = tick_samples_;
    if (tick_samples_ == 0 || stations_.empty()) {
        return tick;
    }

    for (const auto& station_id : stations_) {
        auto& queues = audio_queues_[station_id];
        auto& queue = queues.tx_inbox;
        std::vector<float> samples(tick_samples_, 0.0f);
        const size_t available = std::min(tick_samples_, queue.size());
        for (size_t i = 0; i < available; ++i) {
            samples[i] = queue.front();
            queue.pop_front();
        }
        if (available > 0) {
            maybeSetChannelEpochLocked(tick.start_sample, samples);
            if (capture_.enabled) {
                capture_.tx_samples += samples.size();
            }
            mixer_.submit(station_id, tick.start_sample, samples);
        }
    }

    for (const auto& station_id : stations_) {
        std::vector<float> own_tx;
        if (mixer_.mixForStation(station_id, tick.start_sample, tick_samples_, own_tx)) {
            const bool all_zero = std::all_of(own_tx.begin(), own_tx.end(), [](float sample) {
                return sample == 0.0f;
            });
            if (!all_zero) {
                tick.tx_blocks.push_back({
                    .station_id = station_id,
                    .start_sample = tick.start_sample,
                    .samples = std::move(own_tx),
                });
            }
        }
    }

    for (const auto& receiver_id : stations_) {
        std::vector<float> mixed(tick_samples_, 0.0f);
        mixer_.mixForReceiver(receiver_id, tick.start_sample, tick_samples_, mixed);

        std::vector<float> rx;
        rxChannelForLocked(receiver_id)->process(
            mixed, channelSampleIndexLocked(tick.start_sample), rx);
        auto& outbox = audio_queues_[receiver_id].rx_outbox;
        outbox.push_back({
            .start_sample = tick.start_sample,
            .samples = rx,
        });
        const float rx_rms = rms(rx);
        if (receiver_id == "BRAVO" || rx_rms > 0.001f) {
            std::ostringstream oss;
            oss << "session_outbox_enqueue receiver=" << receiver_id
                << " session=" << config_.session_id
                << " start=" << tick.start_sample
                << " samples=" << rx.size()
                << " rms=" << rx_rms
                << " outbox_blocks_before_trim=" << outbox.size();
            e2eDebugLine(oss.str());
        }
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
    mixer_.discardBefore(session_clock_samples_);
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

void SessionContext::trimTxQueueLocked(StationAudioQueues& queues) const {
    while (queues.tx_inbox.size() > max_tx_queue_samples_) {
        queues.tx_inbox.pop_front();
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
