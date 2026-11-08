// tone_burst_ack_monitor.hpp — sliding-window detector for the tone-burst ACK.
// PHY_ADAPTATION_DESIGN §15 step 4a.
//
// Wraps the ToneBurstDetector with the streaming-audio machinery the
// production path will need:
//   - feedAudio() interface matching StreamingDecoder's pattern
//   - sliding circular buffer big enough to hold one full burst at the
//     longest staircase duration + guard time
//   - runs detection at a configurable cadence (every N samples) to keep
//     CPU bounded
//   - emits a callback when a tone-burst ACK is decoded
//
// Designed so step 4b can construct one inside StreamingDecoder and route
// feedAudio() into it with a single line; integration risk stays low.

#pragma once

#include "tone_burst_constants.hpp"
#include "tone_burst_detector.hpp"
#include "tone_burst_payload.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

// Callback invoked when a complete tone-burst ACK is decoded.
//   payload         — the decoded payload (group_seq, mask, rate_hint, type)
//   detected_offset — sample offset (within the monitor's internal stream
//                     counter) of the first Costas symbol
//   correlation_peak — Costas matched-filter score at the detection
struct ToneBurstAckDetection {
    ToneBurstAckPayload payload;
    uint64_t detected_stream_offset = 0;
    float correlation_peak = 0.0f;
    // Strongest/second-strongest tone ratio of the weakest payload symbol.
    // Kept as telemetry only: no production cliff has been calibrated for it.
    float min_symbol_confidence = 0.0f;
    int hamming_corrected_blocks = 0;
    uint32_t symbol_ms_used = 0;
    // Carrier-offset hypothesis (Hz) the ACK decoded at — a direct measurement of the
    // reverse-link dial offset — and the prior it was searched around.
    float frequency_offset_hz = 0.0f;
    float frequency_prior_hz = 0.0f;
};
using ToneBurstAckCallback = std::function<void(const ToneBurstAckDetection&)>;

class ToneBurstAckMonitor {
public:
    struct Config {
        // Symbol durations to scan for. The detector tries each in order
        // and stops at the first CRC-passing decode. Default = the §15.5
        // staircase rung set (kStaircaseSymbolMs, 12 / 25 / 50 / 100 ms).
        std::vector<uint32_t> symbol_durations_ms{kStaircaseSymbolMs.begin(),
                                                  kStaircaseSymbolMs.end()};
        // §15 step 4d-late: when armed_only=true, detection only runs while
        // the monitor is "armed" (via arm()). Outside the armed window,
        // runDetectionPass() returns immediately with zero work. This makes
        // the audio-thread CPU cost bounded by the active ACK window
        // (typically ~3 s out of every ~10 s in the burst transport),
        // eliminating the always-on polling jitter that the test-default
        // configuration accepts.
        //
        // armed_only=false (default) preserves the original polling
        // behavior — required by the unit tests, which don't have a
        // protocol layer to arm the monitor.
        bool armed_only = false;
        // Run detection every `detect_interval_samples` samples (cadence-
        // limiter). Default: every 240 samples = 5 ms at 48 kHz. Detection
        // window is ~850 ms (kTotalSymbols × kBaselineSymbolMs), so we
        // overlap windows densely without re-scanning at sample granularity.
        size_t detect_interval_samples = 240;
        // §15 step 4d-late: cadence to use WHILE armed. Defaults to the
        // same value as detect_interval_samples so callers that only set
        // armed_only get sensible behavior. Production overrides this to
        // a tight value (~100 ms = 4800 samples) so detection latency is
        // dominated by the burst airtime (~850 ms) not the cadence wait.
        size_t detect_interval_samples_armed = 240;
        // Coarse sweep step inside the detector for each cadence-triggered
        // pass. 8 samples is the default; smaller = finer + slower.
        uint32_t sweep_step_samples = 8;
        // TAIL-WINDOW sweep (2026-07-04, Pi5 ARM regression fix): when true, each
        // detection pass sweeps only the newest needed+cadence+spp window instead
        // of the whole buffer — per-pass CPU stops scaling with buffer capacity
        // (the 100 ms-rung capacity growth pushed the Pi5 ~19 s behind live and
        // the sender went ACK-deaf). Default OFF = legacy whole-buffer semantics
        // (always-on polling tests pin them); the PRODUCTION armed path enables it.
        bool tail_window_sweep = false;
        // Maximum samples the internal sliding buffer holds. Needs to be at
        // least kTotalSymbols × max_symbol_ms × kSampleRate / 1000 + slack.
        // For 100 ms max symbol: 34 × 100 × 48 = 163,200 (34 symbols since the
        // 2026-07-02 frame_mask 8->16 widen; was 27). Default = 180,000
        // (~3.75 s) still accommodates 100 ms + guard.
        size_t buffer_capacity_samples = 180000;
        // Suppress duplicate detections within this many samples of the
        // previous successful decode. Prevents re-firing on the same burst
        // when the next cadence tick lands inside it.
        size_t suppress_window_samples = static_cast<size_t>(kSampleRate) * 1000u / 1000u;  // 1 s

        // Minimum Costas correlation peak as a FRACTION of the ideal
        // clean-channel value, below which we reject the decode as a
        // false positive. Costas+Hamming+CRC alone can occasionally pass
        // on random noise; pairing them with a peak floor closes that
        // gap. Real bursts at SNR ≥ -10 dB sit at ~30%+ of ideal; 5% is
        // a generous cliff that no real burst dips below. See
        // computeMinPeakForSymbolMs() for the per-duration absolute floor.
        float min_peak_fraction = 0.05f;

        // GAPLESS armed sweep (BUG-POSTTX-ACK-MISS, 2026-07-05). The tail-window
        // sweep's invariant ("a pass only needs bursts whose TAIL arrived since
        // the previous pass") breaks when ONE feedAudio() append exceeds a
        // bin's tail window (~25k samples / 520 ms for the 12 ms bin): all the
        // cadence passes triggered inside that call scan the SAME end-anchored
        // window, so a tone deeper in the chunk is never scanned — the rig's
        // post-TX capture-resume backlog produced exactly this (F76/F77: the
        // first ACK after our own key-down arrives strong, is captured, and is
        // missed; ~19 s RTO each). When true (production sets it from
        // ULTRA_ACK_MONITOR_GAPLESS), each armed pass extends the bin's window
        // to also cover everything since the scan high-water mark plus one
        // full burst of context — gapless by induction (hw jumps to the buffer
        // end each pass; a tone straddling the end is re-covered on the next
        // pass because its start sits above hw − needed). Steady-state cost ≈
        // 2×needed per small bin (the 100 ms bin already sweeps ~the full
        // buffer today); a backlog chunk pays a one-off proportional sweep,
        // bounded by buffer_capacity_samples.
        bool gapless_armed_sweep = false;
    };

    // Ideal-clean-channel Costas peak magnitude at the given symbol
    // duration. Used both internally as a gate and exposed for tests.
    //   sum over kCostasSymbols of (samples_per_symbol/2) × kToneAmplitude
    // (Goertzel magnitude for a matched tone of amplitude A over N
    // samples is N·A/2.)
    static float idealCostasPeak(uint32_t symbol_ms);

    ToneBurstAckMonitor();
    explicit ToneBurstAckMonitor(Config cfg);

    // Audio thread: append samples to the internal buffer. Cheap (memcpy
    // into a circular buffer). Triggers detection passes at the configured
    // cadence; on a successful decode invokes the callback synchronously.
    void feedAudio(const float* samples, size_t count);
    void feedAudio(const std::vector<float>& samples) {
        feedAudio(samples.data(), samples.size());
    }

    // Carrier-offset prior for the detector (see ToneBurstDetector::setFrequencyPrior): the
    // receiver's tracked reverse-link CFO and the half-width of its uncertainty. Cheap; the
    // owner refreshes it before each feedAudio.
    void setFrequencyPrior(float offset_hz, float halfwidth_hz) {
        detector_.setFrequencyPrior(offset_hz, halfwidth_hz);
    }
    float frequencyPriorHz() const { return detector_.frequencyPriorHz(); }
    float frequencyHalfWidthHz() const { return detector_.frequencyHalfWidthHz(); }

    // Install the callback. May be called once; subsequent calls replace.
    void setCallback(ToneBurstAckCallback cb) { callback_ = std::move(cb); }

    // Install a side-effect-free protocol plausibility gate. The detector applies it
    // to every CRC-valid timing candidate before declaring success, and continues its
    // bounded timing search after a rejection. With no predicate, legacy PHY-only
    // behavior is preserved for standalone detector/monitor users.
    void setAcceptancePredicate(ToneBurstAckAcceptancePredicate predicate) {
        acceptance_predicate_ = std::move(predicate);
    }

    // §15 step 4d-late: arm the detector for an expected ACK arrival.
    // While armed, detection runs every detect_interval_samples_armed (the
    // tight production cadence). When the armed window elapses without a
    // successful decode, the monitor disarms automatically and goes idle.
    // Calling arm() while already armed extends the window to the larger
    // of (current deadline, new deadline).
    //
    // window_samples is relative to the current stream offset. Pass
    // (window_ms × kSampleRate / 1000) for a duration-based window.
    //
    // Has no effect when Config.armed_only is false (the always-on
    // polling configuration; arm() is a no-op there).
    void arm(size_t window_samples);

    // Start a new physical ACK round with exactly this relative window, replacing any
    // older/larger deadline. Connection uses this after a DATA group is committed: in
    // stop-and-wait there is only one current ACK expectation, and retaining a prior
    // full-burst deadline would defeat a one-frame tail timeout.
    void rearm(size_t window_samples);

    // True iff a successful decode has not yet fired AND the armed window
    // has not yet elapsed. Always false when Config.armed_only is false.
    bool isArmed() const { return armed_; }

    // Reset internal state — drops the audio buffer + suppression window.
    // Use between protocol sessions.
    void reset();

    // For tests + diagnostics.
    uint64_t totalSamplesFed() const { return total_samples_fed_; }
    uint64_t detectionsEmitted() const { return detections_emitted_; }
    uint64_t suppressedAttempts() const { return suppressed_attempts_; }
    uint64_t semanticCandidatesRejected() const {
        return semantic_candidates_rejected_;
    }
    const Config& config() const { return cfg_; }

private:
    // Run one detection pass over the current buffer contents. Iterates
    // through symbol_durations_ms in order; stops at the first decode that
    // passes CRC.
    void runDetectionPass();

    Config cfg_;
    ToneBurstDetector detector_;
    ToneBurstAckCallback callback_;
    ToneBurstAckAcceptancePredicate acceptance_predicate_;

    // Linear buffer (not strictly circular — we drop the oldest samples
    // when over capacity). Simpler than ring buffer; size cap keeps memory
    // bounded.
    std::vector<float> buffer_;
    // Stream-counter offset of buffer_[0] in the overall feedAudio stream.
    uint64_t buffer_start_stream_offset_ = 0;

    // Last triggered cadence position (relative to start of buffer_).
    size_t last_detect_pos_in_buffer_ = 0;
    // Cumulative samples fed since construction.
    uint64_t total_samples_fed_ = 0;
    // Stream offset of the last successful detection (for suppression).
    uint64_t last_detection_stream_offset_ = static_cast<uint64_t>(-1);

    uint64_t detections_emitted_ = 0;
    uint64_t suppressed_attempts_ = 0;
    uint64_t semantic_candidates_rejected_ = 0;
    uint64_t last_semantic_rejection_stream_offset_ =
        static_cast<uint64_t>(-1);

    // §15 step 4d-late: armed-window state. When armed_, runDetectionPass()
    // runs at cfg_.detect_interval_samples_armed cadence. When unset,
    // detection is gated by cfg_.armed_only (false = original always-on;
    // true = idle until armed again).
    bool armed_ = false;
    // 2026-07-04: one-shot latch for the capacity-skip WARN (runDetectionPass) —
    // the mismatch is structural per config, so once per monitor lifetime suffices.
    bool capacity_skip_warned_ = false;
    uint64_t arm_deadline_stream_offset_ = 0;

    // BUG-POSTTX-ACK-MISS diagnostics + gapless-sweep state (2026-07-05).
    // scan_high_water_stream_: stream offset the armed sweep has fully covered
    // (with per-bin burst context) — the gapless anchor. Reset on arm().
    uint64_t scan_high_water_stream_ = 0;
    // Per-armed-window forensic counters: the largest single feedAudio chunk
    // (the tail-hole trigger) and the number of detection passes run. Logged
    // by the window-expiry INFO line so an undetected window is diagnosable
    // from a standard rig log.
    size_t max_feed_chunk_while_armed_ = 0;
    uint64_t passes_while_armed_ = 0;
    uint64_t armed_at_stream_ = 0;
};

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
