#pragma once

#include "ultra/dsp.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <span>
#include <utility>

namespace ultra::audio {

struct ChannelBusyDetectorConfig {
    float quiet_rms_threshold = 0.005f;
    uint32_t quiet_hold_ms = 30;
    uint32_t rms_window_ms = 30;
    bool adaptive_noise_floor = true;
    uint32_t noise_floor_window_ms = 5000;
    uint32_t min_noise_floor_observations = 10;
    float noise_floor_percentile = 0.10f;
    float quiet_noise_multiplier = 1.5f;
    float noise_floor_bootstrap_rms_ceiling = 0.11f;
    // Optional absolute ceiling for samples admitted to the adaptive noise
    // floor. Values <= 0 disable the cap. This is separate from the bootstrap
    // ceiling so calibrated noisy-idle virtual channels can seed at low SNR
    // without ever training on a near-reference modem carrier.
    float noise_floor_estimate_rms_ceiling = 0.0f;
    bool receive_band_rms = true;
    float sample_rate_hz = 48000.0f;
    uint32_t receive_band_filter_taps = 101;
    float receive_band_low_hz = 50.0f;
    float receive_band_high_hz = 2950.0f;
    uint32_t max_wait_for_idle_ms = 15000;
    // Sustained-elevation relearn ("squelch unstick"). The ratiometric admission
    // gate (a sample is only learned as noise if it is <= floor x multiplier)
    // keeps a transient signal out of the floor, but it is ONE-WAY: if the floor
    // seeds LOW (device warmup / band noise not yet flowing at session start) and
    // the real in-band noise then rises above floor x multiplier, every real
    // sample is rejected as "signal", the window starves, and the cached floor
    // latches forever -> permanent false-busy on a noisy band. A real signal is
    // time-bounded; a risen noise floor is not. So if the channel reads BUSY
    // continuously for longer than the longest legitimate continuous
    // transmission, the elevation is the noise floor moving up, not a signal:
    // force-relearn the floor to the new level. 0 disables (legacy/default cfg).
    // MUST exceed kMaxBurstAirtimeMs (<=12000) so a real OFDM burst is never
    // mistaken for a floor rise.
    uint32_t noise_floor_relearn_after_ms = 0;
};

// Ratiometric, level-independent carrier-sense calibration for HF channels.
// Every knob here is dimensionless, so the busy decision works at any absolute
// noise level / AF-gain setting:
//   - median floor (percentile 0.50) tracks the typical noise, robust to bursty
//     real-HF excursions and to a transient signal (minority of the window);
//   - busy threshold = floor x 2.0 (+6 dB);
//   - a real signal is kept out of the floor by the detector's ratiometric
//     admission gate (samples > floor x multiplier are not learned as noise), so
//     no absolute estimate ceiling is needed (0 = disabled);
//   - bootstrap admits any startup level (ceiling ~ full scale): assume idle at
//     startup / listen-before-talk, so the floor seeds to whatever the noise is.
// Shared by ModemEngine (GUI) and exercised by test_channel_busy_detector so the
// calibration cannot drift between the production path and its test.
inline ChannelBusyDetectorConfig ratiometricHfCarrierSenseConfig() {
    ChannelBusyDetectorConfig config;
    // No absolute quiet floor (BUG-RX-ABSOLUTE-LEVEL, INV-RX-LEVEL-001): the 0.005 RMS default
    // took over below -52 dBFS of noise, so a peer heard under 0.005 read as QUIET and we keyed
    // over it. Before the floor seeds (~0.3 s) only digital silence reads quiet.
    config.quiet_rms_threshold = 0.0f;
    config.noise_floor_percentile = 0.50f;
    config.quiet_noise_multiplier = 2.0f;
    config.noise_floor_bootstrap_rms_ceiling = 2.0f;
    config.noise_floor_estimate_rms_ceiling = 0.0f;
    // 13 s > kMaxBurstAirtimeMs ceiling (12 s): a real burst never trips relearn,
    // but a latched-low floor on a steady (e.g. WGN) band recovers within 13 s.
    // INVARIANT (GROUP-SIZE co-fix #3): this must stay > 2× the burst airtime
    // ceiling (escalated ceiling 11,500 ms → 23,000 < 25,000, margin 2 s). A
    // future ceiling raise past 12.5 s silently re-opens the F129 mid-burst
    // floor-relearn crater — re-derive BOTH together.
    // 25 s (was 13): F129 falsified the "13 s > longest burst" premise two ways —
    // a false-busy stretch CONCATENATES with a real burst, and the sender can queue
    // back-to-back bursts (measured 16.4 s continuous TX). Mid-burst relearn seeds
    // the floor from burst-body RMS -> CCA reads idle DURING bursts (the F129
    // self-TX-crater enabler). 25 s > 2x the burst airtime cap.
    config.noise_floor_relearn_after_ms = 25000;
    return config;
}

class ChannelBusyDetector {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit ChannelBusyDetector(ChannelBusyDetectorConfig config = {});

    void reset(TimePoint now = Clock::now());

    void observeSamples(std::span<const float> samples,
                        bool local_rx_blackout,
                        TimePoint now = Clock::now());
    void observeRms(float rms,
                    bool local_rx_blackout,
                    TimePoint now = Clock::now());
    void setNoiseFloorEstimateRmsCeiling(float ceiling);

    bool isIdle() const;
    bool isIdleFor(std::chrono::milliseconds guard) const;
    bool isIdleFor(std::chrono::milliseconds guard, TimePoint now) const;
    TimePoint lastBusyAt() const;
    std::chrono::milliseconds timeSinceQuiet(TimePoint now = Clock::now()) const;
    float currentRms() const;

    bool waitUntilIdle(std::chrono::milliseconds guard);
    bool waitUntilIdle(std::chrono::milliseconds guard,
                       std::chrono::milliseconds max_wait);
    float quietThreshold() const;

    const ChannelBusyDetectorConfig& config() const { return config_; }

private:
    void observeRmsLocked(float rms, bool local_rx_blackout, TimePoint now);
    float receiveBandRmsLocked(std::span<const float> samples);
    bool idleForLocked(TimePoint now, std::chrono::milliseconds guard) const;
    std::chrono::milliseconds requiredQuietDuration(std::chrono::milliseconds guard) const;
    TimePoint idleReadyAtLocked(std::chrono::milliseconds guard) const;
    void pruneWindowLocked(TimePoint now);
    void pruneNoiseFloorLocked(TimePoint now);
    bool hasNoiseFloorEstimateLocked() const;
    float noiseFloorEstimateLocked() const;
    float noiseFloorEstimateCeilingLocked() const;
    bool shouldRecordNoiseFloorSampleLocked(float rms) const;
    float quietThresholdLocked() const;

    ChannelBusyDetectorConfig config_;
    FIRFilter receive_band_filter_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::pair<TimePoint, float>> rms_window_;
    std::deque<std::pair<TimePoint, float>> noise_floor_window_;
    double rms_window_sum_ = 0.0;
    float current_rms_ = 0.0f;
    std::chrono::steady_clock::time_point agc_holdoff_until_{};  // F129 AGC-settle
    float cached_noise_floor_rms_ = 0.0f;
    bool cached_noise_floor_valid_ = false;
    TimePoint last_busy_at_{};
    TimePoint quiet_since_{};
    // Start of the current uninterrupted busy stretch (unset = currently quiet).
    // Drives the sustained-elevation relearn in noise_floor_relearn_after_ms.
    TimePoint busy_since_{};
};

}  // namespace ultra::audio
