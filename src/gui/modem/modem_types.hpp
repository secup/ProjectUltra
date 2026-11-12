#pragma once

// ModemEngine shared types and helper classes
// Separated for cleaner organization and potential reuse

#include "protocol/frame_v2.hpp"  // WaveformMode, FrameType
#include "ultra/types.hpp"
#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <cmath>

namespace ultra {
namespace gui {

// Statistics for display (local to modem engine)
struct LoopbackStats {
    float snr_db = 0.0f;  // Consumer-facing routed value; see snr_source.
    SNRSource snr_source = SNRSource::NONE;
    bool mcdpsk_snr_routed_data_aided = false;  // snr_db is the data-aided MC-DPSK estimate
    bool has_idle_in_band_snr_db = false;
    float idle_in_band_snr_db = 0.0f;
    bool has_ofdm_broadband_snr_db = false;
    float ofdm_broadband_snr_db = 0.0f;
    float ofdm_internal_snr_db = 0.0f;
    float sync_quality_db = 0.0f;
    bool has_frame_physical_snr_db = false;  // the last delivered frame's physical SNR
    float frame_physical_snr_db = 0.0f;
    float ber = 0.0f;
    int frames_sent = 0;
    int frames_received = 0;
    int frames_failed = 0;
    bool synced = false;
    int throughput_bps = 0;
};

struct OperatorSNRDisplay {
    bool valid = false;
    float snr_db = 0.0f;
    SNRSource source = SNRSource::NONE;
};

inline bool isOperatorPhysicalSNRSource(SNRSource source) {
    return source == SNRSource::IDLE_IN_BAND ||
           source == SNRSource::OFDM_BROADBAND ||
           source == SNRSource::MCDPSK_IN_BAND;
}

// prefer_ofdm_broadband: set during an ACTIVE OFDM connection so the meter shows the OFDM in-band
// DECODE SNR whenever it is available — the real link SNR the decoder sees (~22 dB on the IONOS
// link that ran R5/6) — held steady across between-burst gaps (where snr_source momentarily flips
// to the idle noise-floor estimate that under-reads to ~10 dB on a noisy band). When IDLE/
// disconnected (default false) the OFDM value is STALE, so prefer the LIVE current source (idle)
// over it — see test_snr_source_routing::idleMeterPrefersInBandSource. sync_quality is never a
// physical SNR and never drives the meter.
inline OperatorSNRDisplay selectOperatorSNRDisplay(const LoopbackStats& stats,
                                                   bool prefer_ofdm_broadband = false) {
    if (prefer_ofdm_broadband &&
        stats.has_ofdm_broadband_snr_db && std::isfinite(stats.ofdm_broadband_snr_db)) {
        return {true, stats.ofdm_broadband_snr_db, SNRSource::OFDM_BROADBAND};
    }
    if (isOperatorPhysicalSNRSource(stats.snr_source) &&
        std::isfinite(stats.snr_db)) {
        return {true, stats.snr_db, stats.snr_source};
    }
    if (stats.has_idle_in_band_snr_db && std::isfinite(stats.idle_in_band_snr_db)) {
        return {true, stats.idle_in_band_snr_db, SNRSource::IDLE_IN_BAND};
    }
    if (stats.has_ofdm_broadband_snr_db && std::isfinite(stats.ofdm_broadband_snr_db)) {
        return {true, stats.ofdm_broadband_snr_db, SNRSource::OFDM_BROADBAND};
    }
    return {};
}

// ============================================================================
// RX ARCHITECTURE: Clean separation of concerns
// ============================================================================
//
// 1. ACQUISITION THREAD: Only finds preambles, outputs DetectedFrame to queue
// 2. RX/DECODE THREAD: Takes from queue, decodes CW0 to get count, decodes rest
// 3. SAMPLE BUFFER: Thread-safe, supports snapshots for acquisition
//
// ============================================================================

// Output from acquisition thread - minimal info, just "where" and "what type"
struct DetectedFrame {
    int data_start = -1;                    // Sample position where data starts (buffer-relative)
    size_t absolute_sample_pos = 0;         // Absolute position in audio stream (for CFO phase)
    protocol::WaveformMode waveform = protocol::WaveformMode::OFDM_CHIRP;
    std::chrono::steady_clock::time_point timestamp;  // When detected
    bool has_chirp_preamble = false;        // True if chirp-based (has training sequence)
    float cfo_hz = 0.0f;                    // Estimated CFO from dual chirp (Hz)

    bool valid() const { return data_start >= 0; }
};

// Thread-safe queue for detected frames (acquisition -> RX)
class FrameQueue {
public:
    void push(const DetectedFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(frame);
        cv_.notify_one();
    }

    bool tryPop(DetectedFrame& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) return false;
        frame = queue_.front();
        queue_.pop();
        return true;
    }

    // Wait for frame with timeout, returns false if timeout
    bool waitPop(DetectedFrame& frame, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, timeout, [this] { return !queue_.empty() || stopped_; })) {
            return false;
        }
        if (stopped_ || queue_.empty()) return false;
        frame = queue_.front();
        queue_.pop();
        return true;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) queue_.pop();
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = true;
        cv_.notify_all();
    }

    void start() {
        std::lock_guard<std::mutex> lock(mutex_);
        stopped_ = false;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::queue<DetectedFrame> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool stopped_ = false;
};

// Audio filter configuration
struct FilterConfig {
    bool enabled = false;          // Disabled by default (radio's SSB filter sufficient)
    float center_freq = 1500.0f;   // Center frequency in Hz
    float bandwidth = 2900.0f;     // Bandwidth in Hz (covers 2.8 kHz modem + margin)
    int taps = 101;                // FIR filter taps (more = sharper cutoff)

    // Computed passband edges
    float lowFreq() const { return center_freq - bandwidth / 2.0f; }
    float highFreq() const { return center_freq + bandwidth / 2.0f; }
};

// Current frame being decoded by RX thread (RX thread's private state)
struct RxFrameState {
    bool active = false;
    DetectedFrame frame;
    int expected_codewords = 0;            // Learned from CW0 header
    protocol::v2::FrameType frame_type = protocol::v2::FrameType::PROBE;

    void clear() {
        active = false;
        expected_codewords = 0;
        frame_type = protocol::v2::FrameType::PROBE;
    }
};

} // namespace gui
} // namespace ultra
