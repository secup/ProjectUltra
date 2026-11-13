#pragma once

// ChirpSync - Linear FM chirp for robust HF presence detection
//
// Why chirp instead of Barker-13:
// - Spreads energy across entire bandwidth (300-2700 Hz)
// - Frequency-selective fades only affect part of the chirp
// - Matched filter integrates across time-frequency for robust detection
// - Proven technique used by industry-leading commercial HF modems
//
// Design:
// - Linear up-chirp: 300 Hz -> 2700 Hz over 500ms
// - Repeated 2-3 times with short gaps
// - Total duration: ~1.5 seconds
// - Detection via FFT-based matched filter correlation (fast!)

#include "ultra/types.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include <atomic>
#include <vector>
#include <algorithm>
#include <cmath>
#include <complex>
#include <memory>
#include <mutex>

namespace ultra {
namespace sync {

struct ChirpConfig {
    float sample_rate = 48000.0f;
    float f_start = 300.0f;      // Start frequency (Hz)
    float f_end = 2700.0f;       // End frequency (Hz)
    float duration_ms = 500.0f;  // Chirp duration (ms)
    float gap_ms = 100.0f;       // Gap between up and down chirps (ms)
    // Raised-cosine ramp at each end of every chirp (ms; 0 = hard gating). A hard on/off at full
    // amplitude is a broadband click in any spectrogram row that contains it: -25..-29 dB of
    // splatter beyond +-300 Hz of the chirp in a 2048-sample waterfall row, the full-width
    // lines on the waterfall. 10 ms takes that to -51 dB for 0.11 dB of chirp energy (5 ms -49,
    // 2 ms does nothing: under one cycle at 300 Hz); the ramp's main lobe (2/ramp = 200 Hz)
    // stays inside the band that row is judged on. The matched-filter templates carry the same
    // envelope, so detection stays matched.
    float edge_ramp_ms = 10.0f;
    float amplitude = 0.5f;      // Output amplitude
    float tx_cfo_hz = 0.0f;      // TX simulation: frequency offset in Hz
    bool use_dual_chirp = true;  // Use up+down chirp pair for robust CFO/timing
};

class ChirpSync {
public:
    explicit ChirpSync(const ChirpConfig& config = ChirpConfig())
        : config_(config)
    {
        generateTemplate();
    }

    // Generate chirp signal for transmission
    // Returns: samples for complete chirp sequence
    //
    // If use_dual_chirp=true (default):
    //   [UP-CHIRP: 300→2700 Hz] [GAP] [DOWN-CHIRP: 2700→300 Hz] [GAP]
    //   Total duration: ~1.1 seconds
    //   Benefit: Allows exact CFO and timing recovery!
    //
    // If use_dual_chirp=false:
    //   [UP-CHIRP] [GAP]
    //   Total duration: ~600ms
    //   Limitation: CFO and timing are coupled, cannot be separated
    //
    Samples generate() const {
        size_t chirp_samples = static_cast<size_t>(config_.sample_rate * config_.duration_ms / 1000.0f);
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);

        // Calculate total size
        size_t total;
        if (config_.use_dual_chirp) {
            // [up-chirp][gap][down-chirp][gap]
            total = 2 * chirp_samples + 2 * gap_samples;
        } else {
            // [up-chirp][gap]
            total = chirp_samples + gap_samples;
        }

        Samples output(total, 0.0f);

        float T = config_.duration_ms / 1000.0f;
        float k = (config_.f_end - config_.f_start) / T;  // Chirp rate (Hz/s) = 4800

        // Apply CFO offset (for testing - simulates radio frequency error)
        float cfo = config_.tx_cfo_hz;
        if (std::abs(cfo) > 0.001f) {
            LOG_SYNC(INFO, "ChirpSync: Generating chirp with CFO=%.1f Hz (f_start=%.0f+%.0f, f_end=%.0f+%.0f)",
                     cfo, config_.f_start, cfo, config_.f_end, cfo);
        }

        // ===== Generate UP-CHIRP (300 → 2700 Hz) =====
        float f_start_up = config_.f_start + cfo;  // 300 + CFO
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = 2.0f * M_PI * (f_start_up * t + 0.5f * k * t * t);
            output[i] = config_.amplitude * edgeGain(i, chirp_samples) * std::sin(phase);
        }

        if (config_.use_dual_chirp) {
            // ===== Generate DOWN-CHIRP (2700 → 300 Hz) =====
            size_t down_start = chirp_samples + gap_samples;
            float f_start_down = config_.f_end + cfo;  // 2700 + CFO
            for (size_t i = 0; i < chirp_samples; i++) {
                float t = static_cast<float>(i) / config_.sample_rate;
                // Down-chirp: starts at f_end, decreases by k*t
                float phase = 2.0f * M_PI * (f_start_down * t - 0.5f * k * t * t);
                output[down_start + i] = config_.amplitude * edgeGain(i, chirp_samples) * std::sin(phase);
            }
        }

        return output;
    }

    // Detect chirp in received signal
    // Returns: sample offset of chirp start, or -1 if not found
    // Also returns peak correlation value via out parameter
    int detect(SampleSpan samples, float& peak_correlation, float threshold = 0.3f) const {
        if (samples.size() < up_chirp_template_.size()) {
            peak_correlation = 0.0f;
            return -1;
        }

        // Matched filter correlation
        size_t search_len = samples.size() - up_chirp_template_.size();
        float best_corr = 0.0f;
        int best_offset = -1;

        // Coarse search - step of 24 samples (0.5ms) for reliable detection
        // Chirp correlation is very narrow - even 48 samples offset drops to ~0.01
        constexpr size_t step = 24;

        for (size_t start = 0; start < search_len; start += step) {
            float corr = computeCorrelation(samples, start);
            if (corr > best_corr) {
                best_corr = corr;
                best_offset = static_cast<int>(start);
            }
        }

        // Fine search around best coarse position
        if (best_offset >= 0) {
            int fine_start = std::max(0, best_offset - static_cast<int>(step));
            int fine_end = std::min(static_cast<int>(search_len), best_offset + static_cast<int>(step));

            for (int start = fine_start; start < fine_end; start++) {
                float corr = computeCorrelation(samples, start);
                if (corr > best_corr) {
                    best_corr = corr;
                    best_offset = start;
                }
            }
        }

        // If we found a peak, search backward for an earlier repetition
        // (chirp signal may have multiple repetitions, we want the first)
        if (best_offset >= 0 && best_corr >= threshold) {
            size_t chirp_plus_gap = getChirpSamples() +
                static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);

            // Check one repetition earlier
            if (best_offset >= (int)chirp_plus_gap) {
                int earlier_center = best_offset - static_cast<int>(chirp_plus_gap);
                int search_start = std::max(0, earlier_center - static_cast<int>(step));
                int search_end = std::min(static_cast<int>(search_len),
                                          earlier_center + static_cast<int>(step));

                for (int start = search_start; start < search_end; start++) {
                    float corr = computeCorrelation(samples, start);
                    // Accept earlier peak if it's at least 90% as good
                    if (corr >= threshold && corr >= best_corr * 0.9f) {
                        best_corr = corr;
                        best_offset = start;
                        break;  // Found earlier peak, use it
                    }
                }
            }
        }

        peak_correlation = best_corr;

        if (best_corr >= threshold) {
            return best_offset;
        }
        return -1;
    }

    // ========================================================================
    // ROBUST CFO-TOLERANT DETECTION (works at -5 dB SNR)
    // ========================================================================
    //
    // PROBLEM: Correlation finds where frequencies MATCH, not where chirp STARTS.
    // With CFO, these differ by: Δn = -CFO * (Fs / chirp_rate) = -CFO * 10 samples
    //
    // SOLUTION: Two-stage detect + correct
    // 1. Detect with correlation → get position (shifted by CFO)
    // 2. Estimate CFO at that position
    // 3. Correct: true_pos = detected_pos + round(CFO * 10)
    //
    // Returns: sample offset of TRUE chirp start, or -1 if not found

    // ========================================================================
    // ROBUST CHIRP DETECTION
    // ========================================================================
    //
    // KNOWN LIMITATION: With a single linear chirp, position and CFO are coupled:
    //   - Correlation peak shifts by: Δn = -CFO * Fs / chirp_rate = -CFO * 10
    //   - This shift cannot be separated from CFO using the chirp alone
    //
    // SOLUTION: Return correlation peak position. The OFDM demodulator can
    // estimate CFO from training symbols and correct both CFO and timing.
    //
    // For protocols needing precise timing before OFDM, consider:
    //   1. Up-chirp + down-chirp pair (shifts cancel in average)
    //   2. Multiple chirps with different rates
    //
    // Returns: Best correlation peak position

    int detectRobust(SampleSpan samples, float& peak_correlation, float threshold = 0.3f) const {
        const size_t chirp_len = up_chirp_template_.size();
        if (samples.size() < chirp_len) {
            peak_correlation = 0.0f;
            return -1;
        }

        const size_t search_len = samples.size() - chirp_len;

        // Coarse search with complex correlation (CFO-tolerant magnitude)
        constexpr size_t CORR_WINDOW = 480;  // 10ms window
        constexpr size_t COARSE_STEP = 48;   // 1ms steps

        float best_corr = 0.0f;
        int best_offset = -1;

        for (size_t pos = 0; pos < search_len; pos += COARSE_STEP) {
            float m = computeComplexCorrelation(samples, pos, CORR_WINDOW);
            if (m > best_corr) {
                best_corr = m;
                best_offset = static_cast<int>(pos);
            }
        }

        if (best_offset < 0 || best_corr < threshold * 0.3f) {
            peak_correlation = best_corr;
            return -1;
        }

        // Fine search around coarse peak
        int fine_start = std::max(0, best_offset - static_cast<int>(COARSE_STEP));
        int fine_end = std::min(static_cast<int>(search_len), best_offset + static_cast<int>(COARSE_STEP));

        for (int pos = fine_start; pos <= fine_end; pos++) {
            float m = computeComplexCorrelation(samples, pos, CORR_WINDOW);
            if (m > best_corr) {
                best_corr = m;
                best_offset = pos;
            }
        }

        peak_correlation = best_corr;

        return (best_corr >= threshold) ? best_offset : -1;
    }

    // Alias for backward compatibility
    int detectCFOTolerant(SampleSpan samples, float& peak_correlation, float threshold = 0.3f) const {
        return detectRobust(samples, peak_correlation, threshold);
    }

    // ========================================================================
    // CFO ESTIMATION
    // ========================================================================
    //
    // Uses de-chirping: multiply received signal by conjugate of reference chirp.
    // If CFO is present, the de-chirped signal is a pure tone at CFO frequency.
    // Find this frequency using DFT.
    //
    // IMPORTANT: chirp_start must be accurate (use detectRobust() first).
    //
    // Returns: estimated CFO in Hz (typically ±50 Hz range)

    // Estimate CFO using correlation peak shift method
    // chirp_start should be from detectRobust() (energy onset = true position)
    // We find the correlation peak which is shifted by -CFO*10 samples
    // CFO = (true_position - corr_peak) / 10
    float estimateCFO(SampleSpan samples, size_t chirp_start) const {
        const size_t chirp_len = up_chirp_template_.size();
        if (chirp_start + chirp_len > samples.size()) {
            return 0.0f;
        }

        // Find correlation peak around the given position
        constexpr int CORR_WINDOW = 480;  // 10ms
        constexpr int SEARCH_RANGE = 600;  // ±12.5ms = ±60 Hz CFO range

        // Use signed arithmetic to avoid underflow
        int search_start = std::max(0, static_cast<int>(chirp_start) - SEARCH_RANGE);
        int search_end = std::max(0, std::min(static_cast<int>(samples.size()) - static_cast<int>(chirp_len),
                                               static_cast<int>(chirp_start) + SEARCH_RANGE));

        float best_corr = 0.0f;
        int corr_peak = static_cast<int>(chirp_start);

        for (int pos = search_start; pos < static_cast<int>(search_end); pos += 6) {
            float m = computeComplexCorrelation(samples, pos, CORR_WINDOW);
            if (m > best_corr) {
                best_corr = m;
                corr_peak = pos;
            }
        }

        // Fine search around peak
        int fine_start = std::max(search_start, corr_peak - 6);
        int fine_end = std::min(static_cast<int>(search_end), corr_peak + 6);
        for (int pos = fine_start; pos <= fine_end; pos++) {
            float m = computeComplexCorrelation(samples, pos, CORR_WINDOW);
            if (m > best_corr) {
                best_corr = m;
                corr_peak = pos;
            }
        }

        // CFO from position shift:
        // Correlation peak is shifted by -CFO * 10 samples from true position
        // So: corr_peak = chirp_start - CFO * 10
        // CFO = (chirp_start - corr_peak) / 10
        float cfo = static_cast<float>(static_cast<int>(chirp_start) - corr_peak) / 10.0f;

        return cfo;
    }

    // ========================================================================
    // DUAL CHIRP CFO ESTIMATION (Radar Technique)
    // ========================================================================
    //
    // Uses up-chirp + down-chirp pair for robust CFO and timing recovery.
    // Works reliably down to -20 dB SNR!
    //
    // How it works:
    //   - UP chirp correlation peak shifts by: Δn_up = -CFO × Fs / k
    //   - DOWN chirp correlation peak shifts by: Δn_down = +CFO × Fs / k
    //   - CFO = (Δn_down - Δn_up) / (2 × Fs / k)
    //   - True position = average (shifts cancel out)
    //
    // Returns: true if both chirps detected, false otherwise
    // Out parameters: cfo_hz, up_chirp_start (true position after CFO correction)

    struct DualChirpResult {
        bool success = false;
        float cfo_hz = 0.0f;           // Estimated CFO
        int up_chirp_start = -1;       // True up-chirp start (CFO-corrected)
        int down_chirp_start = -1;     // True down-chirp start (CFO-corrected)
        // Raw strongest UP peak considered by this detector pass.  Unlike
        // up_chirp_start, this is populated even when the matching DOWN chirp is
        // not yet present (or the pair fails the CFO sanity gate).  It lets the
        // chronological wrapper below exclude an incomplete, later anchor and
        // retry the already-complete prefix instead of advancing past it.
        int strongest_up_candidate_start = -1;
        float up_correlation = 0.0f;   // Up-chirp correlation value
        float down_correlation = 0.0f; // Down-chirp correlation value
        float gap_error_samples = 0.0f; // actual_gap - expected_gap, before CFO correction
    };

    // detectDualChirp() deliberately returns the strongest global UP peak,
    // which is useful for an isolated signal but is unsafe when one 2.5 s
    // streaming window contains both:
    //
    //   [weaker complete BURST_HEADER anchor] ... [stronger next-group anchor]
    //
    // If the later DOWN chirp is incomplete, the strongest-only detector reports
    // a miss and the search cursor advances beyond the descriptor.  If it is
    // complete but its announced training start is beyond the copied window, it
    // defers the wrong anchor.  Only in those unusable-strongest cases, peel
    // successively earlier prefixes and recover the first complete, CFO-sane
    // pair whose training start is readable.  A normal usable detection returns
    // immediately with zero extra FFT passes; that both preserves strongest-peak
    // semantics and prevents a marginal earlier false pair outranking it.
    DualChirpResult detectDualChirpWithEarlierRecovery(
        SampleSpan samples, float threshold = 0.15f) const {
        const DualChirpResult strongest_window_result =
            detectDualChirp(samples, threshold);
        if (!config_.use_dual_chirp) {
            return strongest_window_result;
        }

        const size_t chirp_len = up_chirp_template_.size();
        const size_t gap_samples = static_cast<size_t>(
            config_.sample_rate * config_.gap_ms / 1000.0f);
        const size_t minimum_pair_span = 2 * chirp_len + gap_samples;
        const auto training_start_is_readable =
            [chirp_len, gap_samples](const DualChirpResult& candidate,
                                    size_t readable_samples) {
                if (!candidate.success || candidate.down_chirp_start < 0) {
                    return false;
                }
                const size_t down_start =
                    static_cast<size_t>(candidate.down_chirp_start);
                return down_start <= readable_samples &&
                       chirp_len + gap_samples <= readable_samples - down_start;
            };

        if (training_start_is_readable(strongest_window_result, samples.size())) {
            return strongest_window_result;
        }

        int prefix_end = strongest_window_result.strongest_up_candidate_start;
        // A 120k full-anchor window can physically contain at most a handful of
        // non-overlapping dual chirps.  Keep a hard ceiling anyway so malformed
        // audio cannot turn candidate peeling into unbounded FFT work.
        constexpr int kMaxEarlierCandidatePasses = 4;
        for (int pass = 0;
             pass < kMaxEarlierCandidatePasses &&
             prefix_end >= 0 &&
             static_cast<size_t>(prefix_end) >= minimum_pair_span;
             ++pass) {
            const size_t prefix_size = static_cast<size_t>(prefix_end);
            const DualChirpResult earlier = detectDualChirp(
                samples.first(prefix_size), threshold);
            if (training_start_is_readable(earlier, samples.size())) {
                LOG_SYNC(INFO,
                         "ChirpSync: earlier-anchor recovery accepted "
                         "up=%d before unusable strongest_up=%d (pass=%d)",
                         earlier.up_chirp_start,
                         strongest_window_result.strongest_up_candidate_start,
                         pass + 1);
                return earlier;
            }

            const int next_prefix_end = earlier.strongest_up_candidate_start;
            if (next_prefix_end < 0 || next_prefix_end >= prefix_end) {
                break;
            }
            prefix_end = next_prefix_end;
        }

        // Preserve the strongest-window candidate and diagnostics when no
        // earlier usable pair exists.  A successful-but-future result continues
        // into the decoder's exact-window deferral path; a genuine miss still
        // exposes up_correlation for threshold diagnostics.
        return strongest_window_result;
    }

    DualChirpResult detectDualChirp(SampleSpan samples, float threshold = 0.15f) const {
        DualChirpResult result;

        if (!config_.use_dual_chirp) {
            // Fall back to single chirp detection using full template correlation
            // (more accurate timing than detectRobust's short-window correlation)
            auto [pos, corr] = detectChirpTemplate(samples, up_chirp_template_, up_chirp_template_cos_,
                                                    template_energy_, threshold);
            if (pos >= 0) {
                result.success = true;
                result.up_chirp_start = pos;
                result.up_correlation = corr;
            }
            return result;
        }

        const size_t chirp_len = up_chirp_template_.size();
        const size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);

        if (samples.size() < 2 * chirp_len + gap_samples) {
            LOG_SYNC(DEBUG, "ChirpSync: Not enough samples: %zu < %zu",
                     samples.size(), 2 * chirp_len + gap_samples);
            return result;  // Not enough samples
        }

        // DEBUG: Check template and signal energy
        static std::atomic<int> debug_count{0};
        if (++debug_count % 10 == 1) {  // Log occasionally
            float tmpl_energy = 0.0f;
            for (float s : up_chirp_template_) tmpl_energy += s * s;

            // Check signal energy at a few positions
            float sig_energy_start = 0.0f, sig_energy_mid = 0.0f;
            size_t mid = samples.size() / 4;  // ~25% into buffer
            for (size_t i = 0; i < std::min(chirp_len, (size_t)1000); i++) {
                sig_energy_start += samples[i] * samples[i];
                if (mid + i < samples.size()) sig_energy_mid += samples[mid + i] * samples[mid + i];
            }
            LOG_SYNC(DEBUG, "ChirpSync: tmpl_energy=%.1f, sig@0=%.4f, sig@%zu=%.4f",
                     tmpl_energy, std::sqrt(sig_energy_start/1000), mid, std::sqrt(sig_energy_mid/1000));
            LOG_SYNC(DEBUG, "ChirpSync: config: f_start=%.0f, f_end=%.0f, duration=%.0f ms, chirp_len=%zu",
                     config_.f_start, config_.f_end, config_.duration_ms, chirp_len);

            // Compute correlation at expected chirp position (~20000)
            if (chirp_len < samples.size()) {
                size_t test_pos = 20000;  // Expected chirp position
                if (test_pos + chirp_len < samples.size()) {
                    float test_corr = 0.0f, test_sig_energy = 0.0f;
                    for (size_t i = 0; i < chirp_len; i++) {
                        float s = samples[test_pos + i];
                        float t = up_chirp_template_[i];
                        test_corr += s * t;
                        test_sig_energy += s * s;
                    }
                    float denom = std::sqrt(test_sig_energy * tmpl_energy);
                    float norm_corr = (denom > 1e-10f) ? std::abs(test_corr) / denom : 0.0f;
                    LOG_SYNC(DEBUG, "ChirpSync: corr at pos 20000: %.4f (sig_energy=%.1f)",
                             norm_corr, test_sig_energy);
                }
            }
        }

        // Detect UP chirp using COMPLEX correlation (CFO-tolerant!)
        auto [up_pos, up_corr] = detectChirpTemplate(samples, up_chirp_template_, up_chirp_template_cos_,
                                                      template_energy_, threshold);
        result.up_correlation = up_corr;  // Always store for debugging
        if (up_pos < 0) {
            return result;  // Up chirp not found
        }
        result.strongest_up_candidate_start = up_pos;

        // Detect DOWN chirp (search in limited window after UP chirp)
        // Expected down chirp is at: up_pos + chirp_len + gap_samples
        // With ±100 Hz CFO, position shifts by up to ±1000 samples (100 Hz * 10 samples/Hz)
        size_t down_search_start = up_pos + chirp_len / 2;
        size_t expected_down_pos = up_pos + chirp_len + gap_samples;
        // Search margin: ensure buffer is large enough to use FFT (>= 2 * chirp_len = 48000)
        // This avoids slow time-domain fallback
        size_t min_search_len = 2 * chirp_len + 1000;  // 49000 samples to ensure FFT path
        size_t down_search_end = std::min(samples.size(),
            std::max(expected_down_pos + 10000 + chirp_len, down_search_start + min_search_len));

        if (down_search_start >= samples.size()) {
            return result;
        }
        // Ensure we have enough samples to search
        if (down_search_end <= down_search_start + chirp_len) {
            down_search_end = std::min(samples.size(), down_search_start + 2 * chirp_len);
        }

        size_t down_search_len = down_search_end - down_search_start;
        SampleSpan down_search(samples.data() + down_search_start, down_search_len);
        // Detect DOWN chirp using COMPLEX correlation (CFO-tolerant!)
        auto [down_pos_rel, down_corr] = detectChirpTemplate(down_search, down_chirp_template_, down_chirp_template_cos_,
                                                              down_template_energy_, threshold);

        if (down_pos_rel < 0) {
            LOG_SYNC(INFO, "ChirpSync: Down chirp NOT found (up was at %d, corr=%.3f)",
                     up_pos, up_corr);
            return result;  // Down chirp not found
        }

        int down_pos = down_pos_rel + static_cast<int>(down_search_start);
        result.down_correlation = down_corr;

        // Calculate CFO from position difference
        // CFO sensitivity: Fs / chirp_rate samples per Hz
        float T = config_.duration_ms / 1000.0f;
        float chirp_rate = (config_.f_end - config_.f_start) / T;
        float cfo_to_samples = config_.sample_rate / chirp_rate;

        // Expected positions without CFO
        // Gap is between end of up-chirp and start of down-chirp
        int expected_gap = static_cast<int>(chirp_len + gap_samples);

        // Actual gap from detected positions
        int actual_gap = down_pos - up_pos;

        // Position errors from expected
        // up_pos should be at true_start, but shifts by -CFO * cfo_to_samples
        // down_pos should be at true_start + chirp_len + gap, but shifts by +CFO * cfo_to_samples
        // So: actual_gap - expected_gap = 2 * CFO * cfo_to_samples
        float gap_error = static_cast<float>(actual_gap - expected_gap);
        result.gap_error_samples = gap_error;
        result.cfo_hz = gap_error / (2.0f * cfo_to_samples);

        LOG_SYNC(INFO, "ChirpSync: Dual chirp: up_pos=%d, down_pos=%d, gap=%d (expected=%d), gap_error=%.1f",
                 up_pos, down_pos, actual_gap, expected_gap, gap_error);
        LOG_SYNC(INFO, "ChirpSync: CFO estimate: %.2f Hz (cfo_to_samples=%.1f, correction=%.1f samples)",
                 result.cfo_hz, cfo_to_samples, result.cfo_hz * cfo_to_samples);

        // Sanity check: reject if estimated CFO is unreasonably large
        // Real HF radios have at most ±100 Hz frequency error
        constexpr float MAX_REASONABLE_CFO_HZ = 100.0f;
        if (std::abs(result.cfo_hz) > MAX_REASONABLE_CFO_HZ) {
            LOG_SYNC(WARN, "ChirpSync: Rejecting detection: CFO=%.1f Hz exceeds max reasonable (+/-%.0f Hz)",
                     result.cfo_hz, MAX_REASONABLE_CFO_HZ);
            return result;  // success = false
        }


        // Apply position correction for CFO-induced chirp peak shift.
        //
        // The complex correlation is CFO-tolerant in amplitude but the peak
        // position shifts with CFO. The raw shift is approximately:
        //   up_chirp: shifts by -CFO × cfo_to_samples
        //   down_chirp: shifts by +CFO × cfo_to_samples
        // where cfo_to_samples = sample_rate / chirp_rate ≈ 10 samples/Hz.
        //
        // Note: The test harness now uses FFT-based Hilbert transform which has
        // NO group delay, so we don't need any delay correction here.

        float up_correction = result.cfo_hz * cfo_to_samples;
        float down_correction = -result.cfo_hz * cfo_to_samples;

        result.up_chirp_start = static_cast<int>(std::round(up_pos + up_correction));
        result.down_chirp_start = static_cast<int>(std::round(down_pos + down_correction));

        LOG_SYNC(INFO, "ChirpSync: Position correction: up=%d (raw=%d, corr=%+.0f), down=%d",
                 result.up_chirp_start, up_pos, up_correction, result.down_chirp_start);

        result.success = true;
        return result;
    }

    // Convenience method: detect and get CFO-corrected position
    int detectWithCFO(SampleSpan samples, float& cfo_hz, float& peak_correlation,
                      float threshold = 0.15f) const {
        auto result = detectDualChirp(samples, threshold);
        if (!result.success) {
            cfo_hz = 0.0f;
            peak_correlation = 0.0f;
            return -1;
        }
        cfo_hz = result.cfo_hz;
        peak_correlation = std::max(result.up_correlation, result.down_correlation);
        return result.up_chirp_start;
    }

    // Get the chirp template (for debugging/visualization)
    const Samples& getTemplate() const { return up_chirp_template_; }
    const Samples& getUpTemplate() const { return up_chirp_template_; }
    const Samples& getDownTemplate() const { return down_chirp_template_; }

    // Get expected chirp duration in samples
    size_t getChirpSamples() const {
        return static_cast<size_t>(config_.sample_rate * config_.duration_ms / 1000.0f);
    }

    // Get total signal duration in samples (including gaps)
    // This matches the output of generate() exactly
    size_t getTotalSamples() const {
        size_t chirp_samples = getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);
        if (config_.use_dual_chirp) {
            // [up-chirp][gap][down-chirp][gap]
            return 2 * chirp_samples + 2 * gap_samples;
        } else {
            // [up-chirp][gap]
            return chirp_samples + gap_samples;
        }
    }

    const ChirpConfig& getConfig() const { return config_; }

private:
    ChirpConfig config_;
    Samples up_chirp_template_;        // Up-chirp sin (300→2700 Hz)
    Samples up_chirp_template_cos_;    // Up-chirp cos (for complex correlation)
    Samples down_chirp_template_;      // Down-chirp sin (2700→300 Hz)
    Samples down_chirp_template_cos_;  // Down-chirp cos (for complex correlation)
    float template_energy_ = 0.0f;
    float down_template_energy_ = 0.0f;

    // FFT-based correlation (fast!)
    // Pre-computed template spectra for O(N log N) correlation instead of O(N*M)
    static constexpr size_t FFT_SIZE = 131072;  // 128K samples (~2.7s at 48kHz)
    mutable std::unique_ptr<FFT> fft_;          // Lazy-initialized
    mutable std::vector<Complex> up_template_fft_;    // FFT of complex up-chirp template
    mutable std::vector<Complex> down_template_fft_;  // FFT of complex down-chirp template
    // Double-checked lazy init: the flag is published with release after the templates are
    // written and read with acquire, so a thread that sees it set sees them too. A plain bool
    // let a weakly ordered CPU (ARM) observe the flag before the template data and read a null
    // vector (two decode-thread SIGSEGVs at address 0 in detectChirpTemplateFFT, 2026-11-11).
    mutable std::atomic<bool> fft_initialized_{false};
    mutable std::mutex fft_init_mutex_;  // Protects FFT initialization

    // Initialize FFT for fast correlation (lazy, thread-safe)
    void initFFT() const {
        if (fft_initialized_.load(std::memory_order_acquire)) return;  // Fast path

        std::lock_guard<std::mutex> lock(fft_init_mutex_);
        if (fft_initialized_.load(std::memory_order_relaxed)) return;  // Double-check

        fft_ = std::make_unique<FFT>(FFT_SIZE);

        // Create complex chirp templates (analytic signal: cos + j*sin)
        // For cross-correlation via FFT: R = IFFT(FFT(signal) * conj(FFT(template)))
        // We store conj(FFT(template)) directly for efficiency
        const size_t chirp_len = up_chirp_template_.size();

        // Up-chirp: create complex template and compute conj(FFT)
        std::vector<Complex> up_complex(FFT_SIZE, Complex(0, 0));
        for (size_t i = 0; i < chirp_len; i++) {
            // Complex template: cos + j*sin (analytic signal)
            up_complex[i] = Complex(up_chirp_template_cos_[i], up_chirp_template_[i]);
        }
        std::vector<Complex> up_fft(FFT_SIZE);
        fft_->forward(up_complex.data(), up_fft.data());
        // Store conjugate for correlation
        up_template_fft_.resize(FFT_SIZE);
        for (size_t i = 0; i < FFT_SIZE; i++) {
            up_template_fft_[i] = std::conj(up_fft[i]);
        }

        // Down-chirp: same treatment
        std::vector<Complex> down_complex(FFT_SIZE, Complex(0, 0));
        for (size_t i = 0; i < chirp_len; i++) {
            down_complex[i] = Complex(down_chirp_template_cos_[i], down_chirp_template_[i]);
        }
        std::vector<Complex> down_fft(FFT_SIZE);
        fft_->forward(down_complex.data(), down_fft.data());
        // Store conjugate for correlation
        down_template_fft_.resize(FFT_SIZE);
        for (size_t i = 0; i < FFT_SIZE; i++) {
            down_template_fft_[i] = std::conj(down_fft[i]);
        }

        fft_initialized_.store(true, std::memory_order_release);

        // Debug: verify templates
        float up_energy = 0, down_energy = 0;
        for (size_t i = 0; i < FFT_SIZE; i++) {
            up_energy += std::norm(up_template_fft_[i]);
            down_energy += std::norm(down_template_fft_[i]);
        }
        LOG_SYNC(DEBUG, "ChirpSync: FFT templates initialized: up_fft_energy=%.1f, down_fft_energy=%.1f, chirp_len=%zu",
                 up_energy, down_energy, chirp_len);
    }

    // FFT-based chirp detection - O(N log N) instead of O(N*M)
    // Returns: (position, correlation) where position is the chirp start
    std::pair<int, float> detectChirpTemplateFFT(SampleSpan samples,
                                                  const std::vector<Complex>& tmpl_fft,
                                                  float tmpl_energy,
                                                  float threshold) const {
        const size_t chirp_len = up_chirp_template_.size();
        if (samples.size() < chirp_len) {
            return {-1, 0.0f};
        }

        // Ensure FFT is initialized
        initFFT();

        // Limit search to FFT_SIZE - chirp_len to avoid wrap-around artifacts
        const size_t search_len = std::min(samples.size(), FFT_SIZE) - chirp_len;
        const size_t fft_input_len = std::min(samples.size(), FFT_SIZE);

        // Zero-pad signal and compute FFT
        std::vector<Complex> signal_complex(FFT_SIZE, Complex(0, 0));
        for (size_t i = 0; i < fft_input_len; i++) {
            signal_complex[i] = Complex(samples[i], 0);
        }

        std::vector<Complex> signal_fft(FFT_SIZE);
        fft_->forward(signal_complex.data(), signal_fft.data());

        // Multiply in frequency domain
        std::vector<Complex> product(FFT_SIZE);
        for (size_t i = 0; i < FFT_SIZE; i++) {
            product[i] = signal_fft[i] * tmpl_fft[i];
        }

        // IFFT to get correlation
        std::vector<Complex> corr_complex(FFT_SIZE);
        fft_->inverse(product.data(), corr_complex.data());

        // Window energies for the normalization, from a running sum. In double: in float, the
        // difference of two large partial sums cancels to rounding garbage over quiet stretches.
        std::vector<double> cumsum_energy(fft_input_len + 1, 0.0);
        for (size_t i = 0; i < fft_input_len; i++) {
            cumsum_energy[i + 1] =
                cumsum_energy[i] + static_cast<double>(samples[i]) * samples[i];
        }
        // The float FFT correlation carries rounding noise of about u log2(N) sqrt(E_buf E_tmpl)
        // at EVERY lag (u = 2^-24), however quiet the window at that lag is. Over a window far
        // quieter than the buffer (digital silence, the near-zero gaps of a resampled file), that
        // noise divided by the window's own tiny energy gave normalized correlations far above 1
        // (3477 measured on a 44.1 kHz WAV) and false chirps that hid the real ones
        // (BUG-CHIRP-NORM-SILENCE). Flooring the window energy at 1e-6 of the buffer's keeps the
        // noise under ~1e-3 of a normalized correlation; only windows 60 dB under the buffer are
        // affected, and float FFT correlation cannot resolve a chirp there anyway.
        const double window_energy_floor = 1e-6 * cumsum_energy[fft_input_len];

        // Find peak correlation (magnitude for CFO tolerance)
        // Cross-correlation R[k] = sum_n s[n] * conj(h[n-k])
        // Peak at position k means template starts at position k in signal
        float best_corr = 0.0f;
        int best_pos = -1;
        // Noise reference for the threshold (BUG-NB-CHIRP-FALSE-LOCK-FILTERED): the same lags,
        // sampled once per quarter of the template's coherence length (1 / bandwidth).
        const float band_hz = std::max(1.0f, std::fabs(config_.f_end - config_.f_start));
        const size_t noise_stride = std::max<size_t>(
            1, static_cast<size_t>(config_.sample_rate / (4.0f * band_hz)));
        std::vector<float> noise_cells;
        noise_cells.reserve(search_len / noise_stride + 1);

        for (size_t pos = 0; pos < search_len; pos++) {
            // Correlation magnitude (CFO-invariant)
            // The correlation output at index pos corresponds to template starting at pos
            float corr_mag = std::abs(corr_complex[pos]);

            // Normalize by signal and template energy
            const double sig_energy = std::max(cumsum_energy[pos + chirp_len] - cumsum_energy[pos],
                                               window_energy_floor);
            const double denom = std::sqrt(sig_energy * static_cast<double>(tmpl_energy));
            const float norm_corr =
                (denom > 1e-10) ? static_cast<float>(corr_mag / denom) : 0.0f;

            if (norm_corr > best_corr) {
                best_corr = norm_corr;
                best_pos = static_cast<int>(pos);
            }
            if (pos % noise_stride == 0) noise_cells.push_back(norm_corr);
        }
        const float noise_floor = noiseReferencedCorrelationFloor(noise_cells, band_hz);
        last_noise_floor_correlation_.store(noise_floor, std::memory_order_relaxed);

        // Debug: show FFT correlation result and check for signal
        static std::atomic<int> fft_debug_count{0};
        if (++fft_debug_count % 20 == 1 || best_corr >= threshold) {
            // Check RMS at a few positions to find where signal is
            float rms_start = 0, rms_mid = 0;
            for (size_t i = 0; i < 1000 && i < fft_input_len; i++) {
                rms_start += samples[i] * samples[i];
                size_t mid = fft_input_len / 2;
                if (mid + i < fft_input_len) rms_mid += samples[mid + i] * samples[mid + i];
            }
            rms_start = std::sqrt(rms_start / 1000);
            rms_mid = std::sqrt(rms_mid / 1000);
            LOG_SYNC(DEBUG, "ChirpSync: FFT buf=%zu, best_corr=%.3f at pos=%d, rms_start=%.3f, rms_mid=%.3f",
                     samples.size(), best_corr, best_pos, rms_start, rms_mid);
        }

        // The fixed threshold stays the floor; the noise-referenced one rules only where the noise
        // in the chirp's band would otherwise put its peaks above it (a narrow IF filter).
        if (best_corr < std::max(threshold, noise_floor)) {
            if (best_corr >= threshold) {
                static std::atomic<uint32_t> refused{0};
                const uint32_t n = ++refused;
                if (n == 1 || n % 100 == 0) {
                    LOG_SYNC(INFO,
                             "ChirpSync: peak %.3f refused under the noise-referenced floor %.3f "
                             "(fixed %.2f, %u refused so far)",
                             best_corr, noise_floor, threshold, n);
                }
            }
            return {-1, best_corr};
        }

        return {best_pos, best_corr};
    }

    // The correlation a peak must clear for the noise in THIS window to produce it about once per
    // hour of audio (BUG-NB-CHIRP-FALSE-LOCK-FILTERED). Under noise the complex correlation is
    // circular Gaussian, so its normalized magnitude is Rayleigh: the median of the lags gives the
    // scale (median = s sqrt(2 ln 2), robust to the few lags a real chirp occupies) and
    // P(|c| > x) = exp(-x^2 / 2s^2) per independent cell, one cell per 1 / bandwidth of audio.
    // Its value depends on how much of the noise lies in the chirp's band: about 0.04 for the
    // 500 Hz chirp in white noise over the 24 kHz stream, about 0.27 behind a 500 Hz IF filter.
    static float noiseReferencedCorrelationFloor(std::vector<float>& cells, float band_hz) {
        if (cells.size() < 64) return 0.0f;
        const auto mid = cells.begin() + static_cast<std::ptrdiff_t>(cells.size() / 2);
        std::nth_element(cells.begin(), mid, cells.end());
        const double rayleigh_scale =
            static_cast<double>(*mid) / std::sqrt(2.0 * std::log(2.0));
        constexpr double kFalseLocksPerHour = 1.0;
        const double cells_per_false_lock =
            std::max(2.0, static_cast<double>(band_hz) * 3600.0 / kFalseLocksPerHour);
        return static_cast<float>(rayleigh_scale *
                                  std::sqrt(2.0 * std::log(cells_per_false_lock)));
    }

public:
    // The noise-referenced correlation floor of the last FFT correlation (diagnostics, tests).
    float lastNoiseFloorCorrelation() const {
        return last_noise_floor_correlation_.load(std::memory_order_relaxed);
    }

private:
    mutable std::atomic<float> last_noise_floor_correlation_{0.0f};

    // Detect chirp using COMPLEX correlation (CFO-tolerant!)
    // Uses FFT-based correlation for speed, falls back to time-domain for small buffers
    // Returns: (position, correlation) where position is the TRUE chirp start
    std::pair<int, float> detectChirpTemplate(SampleSpan samples,
                                               const Samples& tmpl_sin,
                                               const Samples& tmpl_cos,
                                               float tmpl_energy,
                                               float threshold) const {
        const size_t chirp_len = tmpl_sin.size();
        if (samples.size() < chirp_len) {
            return {-1, 0.0f};
        }

        // Use FFT-based correlation for large buffers (much faster)
        // For small buffers, time-domain is fine and avoids FFT overhead
        if (samples.size() >= chirp_len * 2) {
            // Determine which template FFT to use
            const std::vector<Complex>* tmpl_fft = nullptr;
            if (&tmpl_sin == &up_chirp_template_) {
                initFFT();
                tmpl_fft = &up_template_fft_;
            } else if (&tmpl_sin == &down_chirp_template_) {
                initFFT();
                tmpl_fft = &down_template_fft_;
            }

            if (tmpl_fft && !tmpl_fft->empty()) {
                return detectChirpTemplateFFT(samples, *tmpl_fft, tmpl_energy, threshold);
            }
        }

        // Fallback: time-domain correlation for small buffers or if FFT not available
        const size_t search_len = samples.size() - chirp_len;

        float best_corr = 0.0f;
        int best_pos = -1;

        // Coarse search with complex correlation (CFO-tolerant!)
        constexpr size_t COARSE_STEP = 48;  // 1ms at 48kHz

        // Debug: check signal at start
        float rms_check = 0;
        for (size_t i = 0; i < 1000 && i < samples.size(); i++) {
            rms_check += samples[i] * samples[i];
        }
        rms_check = std::sqrt(rms_check / 1000);

        for (size_t pos = 0; pos < search_len; pos += COARSE_STEP) {
            float corr = computeComplexTemplateCorrelation(samples, pos, tmpl_sin, tmpl_cos, tmpl_energy);
            if (corr > best_corr) {
                best_corr = corr;
                best_pos = static_cast<int>(pos);
            }
        }

        // Debug: print search results and check correlation at different positions
        static std::atomic<int> td_debug_count{0};
        if (++td_debug_count % 10 == 1 || best_corr > 0.3f) {
            float corr_at_0 = computeComplexTemplateCorrelation(samples, 0, tmpl_sin, tmpl_cos, tmpl_energy);
            float corr_at_7200 = search_len > 7200 ? computeComplexTemplateCorrelation(samples, 7200, tmpl_sin, tmpl_cos, tmpl_energy) : 0;
            LOG_SYNC(DEBUG, "ChirpSync: TD buf=%zu, best=%.3f@%d, @0=%.3f, @7200=%.3f, rms=%.3f",
                     samples.size(), best_corr, best_pos, corr_at_0, corr_at_7200, rms_check);
        }

        if (best_pos < 0 || best_corr < threshold * 0.3f) {
            if (samples.size() > 50000) {
                LOG_SYNC(INFO, "ChirpSync: FAIL buf=%zu, max_corr=%.3f at pos=%d (threshold=%.2f)",
                         samples.size(), best_corr, best_pos, threshold);
            }
            return {-1, best_corr};
        }

        // Fine search around coarse peak
        int fine_start = std::max(0, best_pos - static_cast<int>(COARSE_STEP));
        int fine_end = std::min(static_cast<int>(search_len), best_pos + static_cast<int>(COARSE_STEP));

        for (int pos = fine_start; pos <= fine_end; pos++) {
            float corr = computeComplexTemplateCorrelation(samples, pos, tmpl_sin, tmpl_cos, tmpl_energy);
            if (corr > best_corr) {
                best_corr = corr;
                best_pos = pos;
            }
        }

        // Parabolic interpolation for sub-sample accuracy
        if (best_pos > 0 && best_pos < static_cast<int>(search_len) - 1) {
            float c0 = computeComplexTemplateCorrelation(samples, best_pos - 1, tmpl_sin, tmpl_cos, tmpl_energy);
            float c1 = best_corr;
            float c2 = computeComplexTemplateCorrelation(samples, best_pos + 1, tmpl_sin, tmpl_cos, tmpl_energy);

            float denom = 2.0f * (c0 - 2.0f * c1 + c2);
            if (std::abs(denom) > 1e-10f) {
                float delta = (c0 - c2) / denom;
                delta = std::max(-1.0f, std::min(1.0f, delta));
                best_pos = static_cast<int>(std::round(best_pos + delta));
            }
        }

        if (best_corr < threshold && samples.size() > 50000) {
            LOG_SYNC(INFO, "ChirpSync: WEAK buf=%zu, best_corr=%.3f at pos=%d (need %.2f)",
                     samples.size(), best_corr, best_pos, threshold);
        }
        return (best_corr >= threshold) ? std::make_pair(best_pos, best_corr)
                                        : std::make_pair(-1, best_corr);
    }

    // Compute normalized COMPLEX correlation with sin/cos templates
    // Returns magnitude sqrt(I² + Q²) which is CFO-invariant!
    //
    // Why this works:
    // - Real correlation oscillates at beat frequency (CFO Hz)
    // - Complex correlation: correlate with exp(-j*phase) = cos - j*sin
    // - Magnitude |I + jQ| doesn't oscillate - it's the envelope
    // - Peak position is at true chirp start, regardless of CFO
    //
    float computeComplexTemplateCorrelation(SampleSpan samples, size_t offset,
                                             const Samples& tmpl_sin,
                                             const Samples& tmpl_cos,
                                             float tmpl_energy) const {
        if (offset + tmpl_sin.size() > samples.size()) return 0.0f;

        float corr_I = 0.0f;  // In-phase (cos correlation)
        float corr_Q = 0.0f;  // Quadrature (sin correlation)
        float sig_energy = 0.0f;

        for (size_t i = 0; i < tmpl_sin.size(); i++) {
            float s = samples[offset + i];
            // Complex correlation: s * exp(-j*phase) = s*cos(phase) - j*s*sin(phase)
            corr_I += s * tmpl_cos[i];
            corr_Q += s * tmpl_sin[i];
            sig_energy += s * s;
        }

        float denom = std::sqrt(sig_energy * tmpl_energy);
        if (denom < 1e-10f) return 0.0f;

        // Return magnitude of complex correlation (CFO-invariant!)
        return std::sqrt(corr_I * corr_I + corr_Q * corr_Q) / denom;
    }

    // Legacy real correlation (kept for reference, not used)
    float computeTemplateCorrelation(SampleSpan samples, size_t offset,
                                      const Samples& chirp_template,
                                      float tmpl_energy) const {
        if (offset + chirp_template.size() > samples.size()) return 0.0f;

        float corr = 0.0f;
        float sig_energy = 0.0f;

        for (size_t i = 0; i < chirp_template.size(); i++) {
            float s = samples[offset + i];
            float t = chirp_template[i];
            corr += s * t;
            sig_energy += s * s;
        }

        float denom = std::sqrt(sig_energy * tmpl_energy);
        if (denom < 1e-10f) return 0.0f;

        return std::abs(corr) / denom;
    }

    // Chirp envelope at sample i of n: a raised-cosine ramp over edge_ramp_ms at each end, 1 between.
    float edgeGain(size_t i, size_t n) const {
        const size_t ramp = std::min(
            n / 2, static_cast<size_t>(config_.sample_rate * config_.edge_ramp_ms / 1000.0f));
        if (ramp == 0) return 1.0f;
        const size_t from_edge = std::min(i, n - 1 - i);
        if (from_edge >= ramp) return 1.0f;
        return 0.5f - 0.5f * std::cos(static_cast<float>(M_PI) * static_cast<float>(from_edge) /
                                      static_cast<float>(ramp));
    }

    // Generate phase for UP-chirp at time t
    float generateUpChirpPhase(float t) const {
        float T = config_.duration_ms / 1000.0f;
        float k = (config_.f_end - config_.f_start) / T;  // +4800 Hz/s
        return 2.0f * M_PI * (config_.f_start * t + 0.5f * k * t * t);
    }

    // Generate phase for DOWN-chirp at time t
    float generateDownChirpPhase(float t) const {
        float T = config_.duration_ms / 1000.0f;
        float k = (config_.f_end - config_.f_start) / T;  // 4800 Hz/s
        // Down-chirp starts at f_end and decreases
        return 2.0f * M_PI * (config_.f_end * t - 0.5f * k * t * t);
    }

    // Alias for backward compatibility
    float generateChirpPhase(float t) const {
        return generateUpChirpPhase(t);
    }

    void generateTemplate() {
        size_t chirp_samples = getChirpSamples();

        // Generate up-chirp template (sin + cos for complex correlation)
        up_chirp_template_.resize(chirp_samples);
        up_chirp_template_cos_.resize(chirp_samples);
        template_energy_ = 0.0f;
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateUpChirpPhase(t);
            const float g = edgeGain(i, chirp_samples);
            up_chirp_template_[i] = g * std::sin(phase);
            up_chirp_template_cos_[i] = g * std::cos(phase);
            template_energy_ += up_chirp_template_[i] * up_chirp_template_[i];
        }

        // Generate down-chirp template (sin + cos for complex correlation)
        down_chirp_template_.resize(chirp_samples);
        down_chirp_template_cos_.resize(chirp_samples);
        down_template_energy_ = 0.0f;
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateDownChirpPhase(t);
            const float g = edgeGain(i, chirp_samples);
            down_chirp_template_[i] = g * std::sin(phase);
            down_chirp_template_cos_[i] = g * std::cos(phase);
            down_template_energy_ += down_chirp_template_[i] * down_chirp_template_[i];
        }
    }

    // Internal CFO estimation - works even at shifted position
    // Uses de-chirping: multiply by reference conjugate → tone at CFO frequency
    // Then find peak frequency using DFT
    float estimateCFOInternal(SampleSpan samples, size_t pos) const {
        const size_t chirp_len = up_chirp_template_.size();
        if (pos + chirp_len > samples.size()) {
            return 0.0f;
        }

        // Use first 100ms of chirp for CFO estimation (enough for good resolution)
        constexpr size_t EST_WINDOW = 4800;  // 100ms
        const size_t use_len = std::min(EST_WINDOW, chirp_len);

        // De-chirp: multiply by reference chirp conjugate
        std::vector<float> dechirp_I(use_len);
        std::vector<float> dechirp_Q(use_len);

        for (size_t i = 0; i < use_len; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateChirpPhase(t);
            float rx = samples[pos + i];
            dechirp_I[i] = rx * std::cos(phase);
            dechirp_Q[i] = rx * std::sin(phase);
        }

        // Find peak frequency using DFT
        // Search ±60 Hz range with ~2.4 Hz resolution (100ms window)
        constexpr float MAX_CFO = 60.0f;
        const float freq_res = config_.sample_rate / use_len;  // ~10 Hz for 100ms

        float best_power = 0.0f;
        float best_freq = 0.0f;

        // Coarse search
        for (float freq = -MAX_CFO; freq <= MAX_CFO; freq += freq_res * 0.5f) {
            float sum_I = 0.0f, sum_Q = 0.0f;
            for (size_t i = 0; i < use_len; i++) {
                float dft_phase = 2.0f * M_PI * freq * i / config_.sample_rate;
                sum_I += dechirp_I[i] * std::cos(dft_phase) + dechirp_Q[i] * std::sin(dft_phase);
                sum_Q += dechirp_Q[i] * std::cos(dft_phase) - dechirp_I[i] * std::sin(dft_phase);
            }
            float power = sum_I * sum_I + sum_Q * sum_Q;
            if (power > best_power) {
                best_power = power;
                best_freq = freq;
            }
        }

        // Fine search around peak
        float fine_start = best_freq - freq_res;
        float fine_end = best_freq + freq_res;
        for (float freq = fine_start; freq <= fine_end; freq += 0.5f) {
            float sum_I = 0.0f, sum_Q = 0.0f;
            for (size_t i = 0; i < use_len; i++) {
                float dft_phase = 2.0f * M_PI * freq * i / config_.sample_rate;
                sum_I += dechirp_I[i] * std::cos(dft_phase) + dechirp_Q[i] * std::sin(dft_phase);
                sum_Q += dechirp_Q[i] * std::cos(dft_phase) - dechirp_I[i] * std::sin(dft_phase);
            }
            float power = sum_I * sum_I + sum_Q * sum_Q;
            if (power > best_power) {
                best_power = power;
                best_freq = freq;
            }
        }

        // De-chirping produces tone at -CFO, so negate
        return -best_freq;
    }

    // Compute normalized correlation at given offset (full template)
    float computeCorrelation(SampleSpan samples, size_t offset) const {
        float corr = 0.0f;
        float sig_energy = 0.0f;

        for (size_t i = 0; i < up_chirp_template_.size(); i++) {
            float s = samples[offset + i];
            float t = up_chirp_template_[i];
            corr += s * t;
            sig_energy += s * s;
        }

        float denom = std::sqrt(sig_energy * template_energy_);
        if (denom < 1e-10f) return 0.0f;

        return std::abs(corr) / denom;  // abs() because phase might be inverted
    }

    // Compute CFO-tolerant complex correlation
    //
    // Key insight: For a chirp signal with CFO:
    //   s(t) = A·sin(chirp_phase + 2π·CFO·t + φ₀)
    // Complex correlation with reference chirp:
    //   I = Σ s(t)·cos(chirp_phase)
    //   Q = Σ s(t)·sin(chirp_phase)
    // The magnitude |I + jQ| is INVARIANT to CFO because:
    //   - CFO adds constant phase rotation per sample
    //   - Over the window, this rotates the correlation phasor
    //   - But the MAGNITUDE stays the same!
    //
    // Returns normalized correlation magnitude (0 to 1)
    float computeComplexCorrelation(SampleSpan samples, size_t offset, size_t window_len) const {
        if (offset + window_len > samples.size()) return 0.0f;

        float corr_I = 0.0f;
        float corr_Q = 0.0f;
        float sig_energy = 0.0f;

        for (size_t i = 0; i < window_len; i++) {
            float s = samples[offset + i];
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateChirpPhase(t);

            // Complex correlation: s * exp(-j*phase) = s*(cos - j*sin)
            corr_I += s * std::cos(phase);
            corr_Q += s * std::sin(phase);
            sig_energy += s * s;
        }

        // Normalize by sqrt(signal_energy * reference_energy)
        // Reference energy for unit amplitude chirp = window_len * 0.5 (avg of sin²)
        float ref_energy = window_len * 0.5f;
        float denom = std::sqrt(sig_energy * ref_energy);
        if (denom < 1e-10f) return 0.0f;

        // Return magnitude of complex correlation (CFO-invariant)
        return std::sqrt(corr_I * corr_I + corr_Q * corr_Q) / denom;
    }
};  // class ChirpSync

} // namespace sync
} // namespace ultra
