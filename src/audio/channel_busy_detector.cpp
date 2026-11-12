#include "audio/channel_busy_detector.hpp"
#include "sync/signal_policy.hpp"

#include "ultra/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <vector>

namespace ultra::audio {

namespace {

// ULTRA_CARRIER_SENSE_DEBUG=1 enables per-event logging at WARN level
// for state transitions and waits. Off by default (production silent).
// Matches the pattern of ULTRA_HARQ_DEBUG_LOG / ULTRA_CFO_DEBUG_LOG.
bool csDebugEnabled() {
    static const bool enabled = [] {
        const char* env = std::getenv("ULTRA_CARRIER_SENSE_DEBUG");
        return env && env[0] != '\0' && !(env[0] == '0' && env[1] == '\0');
    }();
    return enabled;
}

float rmsOf(std::span<const float> samples) {
    if (samples.empty()) {
        return 0.0f;
    }

    double sum_sq = 0.0;
    for (float sample : samples) {
        sum_sq += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(samples.size())));
}

size_t sanitizedFirTaps(uint32_t taps) {
    size_t sanitized = static_cast<size_t>(std::max<uint32_t>(3, taps));
    if ((sanitized % 2) == 0) {
        ++sanitized;
    }
    return sanitized;
}

}  // namespace

ChannelBusyDetector::ChannelBusyDetector(ChannelBusyDetectorConfig config)
    : config_(config),
      receive_band_filter_(FIRFilter::bandpass(
          sanitizedFirTaps(config_.receive_band_filter_taps),
          config_.receive_band_low_hz,
          config_.receive_band_high_hz,
          config_.sample_rate_hz)) {
    reset();
}

void ChannelBusyDetector::reset(TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    rms_window_.clear();
    noise_floor_window_.clear();
    receive_band_filter_.reset();
    rms_window_sum_ = 0.0;
    current_rms_ = 0.0f;
    cached_noise_floor_rms_ = 0.0f;
    cached_noise_floor_valid_ = false;
    const auto initial_quiet_age =
        std::chrono::hours(24) +
        std::chrono::milliseconds(config_.quiet_hold_ms + config_.max_wait_for_idle_ms);
    quiet_since_ = now - initial_quiet_age;
    last_busy_at_ = quiet_since_;
    busy_since_ = TimePoint{};
    cv_.notify_all();
}

void ChannelBusyDetector::observeSamples(std::span<const float> samples,
                                         bool local_rx_blackout,
                                         TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (local_rx_blackout) {
        observeRmsLocked(0.0f, true, now);
        return;
    }
    observeRmsLocked(receiveBandRmsLocked(samples), local_rx_blackout, now);
}

void ChannelBusyDetector::observeRms(float rms,
                                     bool local_rx_blackout,
                                     TimePoint now) {
    std::lock_guard<std::mutex> lock(mutex_);
    observeRmsLocked(rms, local_rx_blackout, now);
}

void ChannelBusyDetector::setNoiseFloorEstimateRmsCeiling(float ceiling) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.noise_floor_estimate_rms_ceiling = std::isfinite(ceiling)
        ? std::max(0.0f, ceiling)
        : 0.0f;
}

void ChannelBusyDetector::observeRmsLocked(float rms,
                                           bool local_rx_blackout,
                                           TimePoint now) {
    current_rms_ = std::max(0.0f, rms);
    const bool was_quiet = (quiet_since_ != TimePoint{});

    if (local_rx_blackout) {
        rms_window_.clear();
        rms_window_sum_ = 0.0;
        last_busy_at_ = now;
        quiet_since_ = TimePoint{};
        // Our own keyed TX is not a channel-noise observation: do not let it
        // accrue toward the sustained-elevation relearn (it would relearn the
        // floor up to our own signal level).
        busy_since_ = TimePoint{};
        cv_.notify_all();
        if (was_quiet && csDebugEnabled()) {
            LOG_MODEM(WARN, "CS quiet->busy (local TX blackout) rms=%.4f", current_rms_);
        }
        return;
    }

    rms_window_.emplace_back(now, current_rms_);
    rms_window_sum_ += current_rms_;
    pruneWindowLocked(now);
    pruneNoiseFloorLocked(now);

    const float window_rms = rms_window_.empty()
        ? current_rms_
        : static_cast<float>(rms_window_sum_ / static_cast<double>(rms_window_.size()));
    const float threshold = quietThresholdLocked();

    // AGC-SETTLE HOLDOFF (F129, scoped by the CI catch): after burst-level audio,
    // the rig RX chain's AGC recovery produces SUB-AMBIENT dips (measured
    // 0.008-0.026 vs true band noise ~0.030). The one-way admission gate loves
    // exactly those samples, the median collapses below the real floor, and
    // post-burst ambient then reads "busy" forever (both F129 ACK drops, 2x
    // ~19 s RTO stalls). Noise cannot fade — SUB-FLOOR readings after strong
    // audio are artifacts: suspend their admission for 2.5 s.
    // SCOPE (testRelearnsRisenNoiseFloorAfterLowSeed, caught by CI Debug — the
    // original form gated ALL admission during the holdoff): steady RISEN noise
    // at >4x a stale low floor re-arms the holdoff on every sample, which would
    // starve the floor window forever and defeat the sustained-elevation relearn
    // — re-latching the exact IONOS false-busy lockup that test guards. The
    // AGC-artifact class is only samples BELOW the cached floor; at-or-above-
    // floor samples must always stay admissible so the relearn can re-seed a
    // genuinely risen band.
    if (cached_noise_floor_valid_ &&
        current_rms_ > cached_noise_floor_rms_ * 4.0f) {
        agc_holdoff_until_ = now + std::chrono::milliseconds(2500);
    }
    const bool agc_dip_suspect =
        now < agc_holdoff_until_ && cached_noise_floor_valid_ &&
        current_rms_ < cached_noise_floor_rms_;
    // Digital silence (below one 24-bit converter step) is no receiver, not channel noise: a
    // half-duplex radio's receive while its own transmitter is keyed, a muted or unplugged input
    // (INV-RX-LEVEL-001: the one absolute test). Learned as noise, an own burst longer than the
    // floor window left only its zeros there: floor 0, the 0 x 2 admission gate then refused every
    // real noise sample, and the channel read busy until the relearn (GUI half-duplex gate: the
    // sender's threshold read 0 for 75-95 % of a session; a MODE_CHANGE retry was held 17 s).
    const bool digital_silence = current_rms_ < ultra::sync::signal_policy::kDigitalSilenceRms;
    if (!digital_silence && !agc_dip_suspect && shouldRecordNoiseFloorSampleLocked(current_rms_)) {
        noise_floor_window_.emplace_back(now, current_rms_);
        pruneNoiseFloorLocked(now);
    }
    const size_t required_noise_samples =
        static_cast<size_t>(std::max<uint32_t>(1, config_.min_noise_floor_observations));
    if (noise_floor_window_.size() >= required_noise_samples) {
        cached_noise_floor_rms_ = noiseFloorEstimateLocked();
        cached_noise_floor_valid_ = true;
    }

    if (window_rms > threshold) {
        // Sustained-elevation relearn ("squelch unstick"). A real transmission is
        // time-bounded; a risen noise floor is not. If the channel has read busy
        // continuously for longer than the longest legitimate transmission, the
        // elevation is the noise floor itself moving up (the one-way admission
        // gate has starved the floor window and latched the cached floor below
        // the true noise -> permanent false-busy). Drop the stale floor so the
        // next samples re-bootstrap to the new level.
        if (config_.noise_floor_relearn_after_ms > 0) {
            if (busy_since_ == TimePoint{}) {
                busy_since_ = now;
            } else if (now - busy_since_ >= std::chrono::milliseconds(
                                                config_.noise_floor_relearn_after_ms)) {
                noise_floor_window_.clear();
                cached_noise_floor_valid_ = false;
                cached_noise_floor_rms_ = 0.0f;
                busy_since_ = now;  // re-arm; re-bootstrap over the next window
                if (csDebugEnabled()) {
                    LOG_MODEM(WARN,
                              "CS relearn: floor latched below noise, re-seeding "
                              "(window_rms=%.4f thresh=%.4f)",
                              window_rms, threshold);
                }
            }
        }
        last_busy_at_ = now;
        quiet_since_ = TimePoint{};
        cv_.notify_all();
        if (was_quiet && csDebugEnabled()) {
            LOG_MODEM(WARN, "CS quiet->busy window_rms=%.4f thresh=%.4f",
                      window_rms, threshold);
        }
        return;
    }

    busy_since_ = TimePoint{};  // back to quiet: reset the relearn timer
    if (quiet_since_ == TimePoint{}) {
        quiet_since_ = now;
        if (csDebugEnabled()) {
            LOG_MODEM(WARN, "CS busy->quiet window_rms=%.4f thresh=%.4f",
                      window_rms, threshold);
        }
    }
    cv_.notify_all();
}

float ChannelBusyDetector::receiveBandRmsLocked(std::span<const float> samples) {
    if (!config_.receive_band_rms) {
        return rmsOf(samples);
    }
    if (samples.empty()) {
        return 0.0f;
    }

    double sum_sq = 0.0;
    for (float sample : samples) {
        const float filtered = receive_band_filter_.process(sample);
        sum_sq += static_cast<double>(filtered) * static_cast<double>(filtered);
    }
    return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(samples.size())));
}

bool ChannelBusyDetector::isIdle() const {
    return isIdleFor(std::chrono::milliseconds(0));
}

bool ChannelBusyDetector::isIdleFor(std::chrono::milliseconds guard) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return idleForLocked(Clock::now(), guard);
}

bool ChannelBusyDetector::isIdleFor(std::chrono::milliseconds guard,
                                    TimePoint now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return idleForLocked(now, guard);
}

ChannelBusyDetector::TimePoint ChannelBusyDetector::lastBusyAt() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_busy_at_;
}

std::chrono::milliseconds ChannelBusyDetector::timeSinceQuiet(TimePoint now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (quiet_since_ == TimePoint{} || now < quiet_since_) {
        return std::chrono::milliseconds(0);
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - quiet_since_);
}

float ChannelBusyDetector::currentRms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_rms_;
}

float ChannelBusyDetector::quietThreshold() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return quietThresholdLocked();
}

bool ChannelBusyDetector::waitUntilIdle(std::chrono::milliseconds guard) {
    return waitUntilIdle(
        guard,
        std::chrono::milliseconds(std::max<uint32_t>(1, config_.max_wait_for_idle_ms)));
}

bool ChannelBusyDetector::waitUntilIdle(std::chrono::milliseconds guard,
                                        std::chrono::milliseconds max_wait) {
    std::unique_lock<std::mutex> lock(mutex_);
    const TimePoint start = Clock::now();
    const TimePoint deadline = start + max_wait;
    const bool debug = csDebugEnabled();
    const bool was_idle = idleForLocked(start, guard);

    if (debug && !was_idle) {
        LOG_MODEM(WARN, "CS wait_until_idle: blocking guard=%dms max_wait=%dms rms=%.4f",
                  static_cast<int>(guard.count()),
                  static_cast<int>(max_wait.count()),
                  current_rms_);
    }

    while (!idleForLocked(Clock::now(), guard)) {
        const TimePoint ready_at = idleReadyAtLocked(guard);
        const TimePoint wait_until = ready_at == TimePoint{}
            ? deadline
            : std::min(ready_at, deadline);

        if (cv_.wait_until(lock, wait_until) == std::cv_status::timeout &&
            Clock::now() >= deadline) {
            const bool final_idle = idleForLocked(Clock::now(), guard);
            if (debug) {
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    Clock::now() - start).count();
                LOG_MODEM(WARN, "CS wait_until_idle: timeout after %dms (idle=%d)",
                          static_cast<int>(elapsed), final_idle ? 1 : 0);
            }
            return final_idle;
        }
    }

    if (debug && !was_idle) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            Clock::now() - start).count();
        LOG_MODEM(WARN, "CS wait_until_idle: released after %dms", static_cast<int>(elapsed));
    }
    return true;
}

bool ChannelBusyDetector::idleForLocked(TimePoint now,
                                        std::chrono::milliseconds guard) const {
    if (quiet_since_ == TimePoint{} || now < quiet_since_) {
        return false;
    }
    return now - quiet_since_ >= requiredQuietDuration(guard);
}

std::chrono::milliseconds ChannelBusyDetector::requiredQuietDuration(
    std::chrono::milliseconds guard) const {
    return std::chrono::milliseconds(config_.quiet_hold_ms) + std::max(guard, std::chrono::milliseconds(0));
}

ChannelBusyDetector::TimePoint ChannelBusyDetector::idleReadyAtLocked(
    std::chrono::milliseconds guard) const {
    if (quiet_since_ == TimePoint{}) {
        return TimePoint{};
    }
    return quiet_since_ + requiredQuietDuration(guard);
}

void ChannelBusyDetector::pruneWindowLocked(TimePoint now) {
    const auto window = std::chrono::milliseconds(std::max<uint32_t>(1, config_.rms_window_ms));
    while (!rms_window_.empty() && now - rms_window_.front().first > window) {
        rms_window_sum_ -= rms_window_.front().second;
        rms_window_.pop_front();
    }

    if (rms_window_.empty()) {
        rms_window_sum_ = 0.0;
    }
}

void ChannelBusyDetector::pruneNoiseFloorLocked(TimePoint now) {
    const auto window =
        std::chrono::milliseconds(std::max<uint32_t>(1, config_.noise_floor_window_ms));
    while (!noise_floor_window_.empty() && now - noise_floor_window_.front().first > window) {
        noise_floor_window_.pop_front();
    }
}

bool ChannelBusyDetector::hasNoiseFloorEstimateLocked() const {
    const size_t required =
        static_cast<size_t>(std::max<uint32_t>(1, config_.min_noise_floor_observations));
    return config_.adaptive_noise_floor &&
           (noise_floor_window_.size() >= required || cached_noise_floor_valid_);
}

float ChannelBusyDetector::noiseFloorEstimateLocked() const {
    const size_t required =
        static_cast<size_t>(std::max<uint32_t>(1, config_.min_noise_floor_observations));
    if (noise_floor_window_.size() < required) {
        return cached_noise_floor_valid_ ? cached_noise_floor_rms_ : 0.0f;
    }
    if (noise_floor_window_.empty()) {
        return 0.0f;
    }

    std::vector<float> values;
    values.reserve(noise_floor_window_.size());
    for (const auto& entry : noise_floor_window_) {
        values.push_back(entry.second);
    }

    const float percentile = std::clamp(config_.noise_floor_percentile, 0.0f, 1.0f);
    const size_t index = std::min(
        values.size() - 1,
        static_cast<size_t>(percentile * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index),
                     values.end());
    return std::max(0.0f, values[index]);
}

float ChannelBusyDetector::noiseFloorEstimateCeilingLocked() const {
    return std::isfinite(config_.noise_floor_estimate_rms_ceiling)
        ? std::max(0.0f, config_.noise_floor_estimate_rms_ceiling)
        : 0.0f;
}

bool ChannelBusyDetector::shouldRecordNoiseFloorSampleLocked(float rms) const {
    if (!config_.adaptive_noise_floor || !std::isfinite(rms)) {
        return false;
    }

    const float sample_rms = std::max(0.0f, rms);
    const float multiplier = std::max(1.0f, config_.quiet_noise_multiplier);
    const float sample_ceiling_config = noiseFloorEstimateCeilingLocked();
    const float sample_ceiling = sample_ceiling_config > 0.0f
        ? sample_ceiling_config
        : std::numeric_limits<float>::infinity();
    if (!hasNoiseFloorEstimateLocked()) {
        return sample_rms <= std::min(
            std::max(0.0f, config_.noise_floor_bootstrap_rms_ceiling),
            sample_ceiling);
    }

    const float noise_floor = noiseFloorEstimateLocked();
    const float candidate_gate = std::max(
        std::max(0.0f, config_.quiet_rms_threshold),
        noise_floor * multiplier);
    return sample_rms <= std::min(candidate_gate, sample_ceiling);
}

float ChannelBusyDetector::quietThresholdLocked() const {
    float threshold = std::max(0.0f, config_.quiet_rms_threshold);
    if (!hasNoiseFloorEstimateLocked()) {
        return threshold;
    }

    const float noise_floor = noiseFloorEstimateLocked();
    const float adaptive_threshold =
        noise_floor * std::max(1.0f, config_.quiet_noise_multiplier);
    return std::max(threshold, adaptive_threshold);
}

}  // namespace ultra::audio
