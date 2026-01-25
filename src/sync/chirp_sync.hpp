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
// - Detection via matched filter correlation

#include "ultra/types.hpp"
#include "ultra/dsp.hpp"
#include <vector>
#include <cmath>
#include <complex>

namespace ultra {
namespace sync {

struct ChirpConfig {
    float sample_rate = 48000.0f;
    float f_start = 300.0f;      // Start frequency (Hz)
    float f_end = 2700.0f;       // End frequency (Hz)
    float duration_ms = 500.0f;  // Chirp duration (ms)
    float gap_ms = 100.0f;       // Gap between up and down chirps (ms)
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
            fprintf(stderr, "[CHIRP-TX] Generating chirp with CFO=%.1f Hz (f_start=%.0f+%.0f, f_end=%.0f+%.0f)\n",
                    cfo, config_.f_start, cfo, config_.f_end, cfo);
        }

        // ===== Generate UP-CHIRP (300 → 2700 Hz) =====
        float f_start_up = config_.f_start + cfo;  // 300 + CFO
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = 2.0f * M_PI * (f_start_up * t + 0.5f * k * t * t);
            output[i] = config_.amplitude * std::sin(phase);
        }

        if (config_.use_dual_chirp) {
            // ===== Generate DOWN-CHIRP (2700 → 300 Hz) =====
            size_t down_start = chirp_samples + gap_samples;
            float f_start_down = config_.f_end + cfo;  // 2700 + CFO
            for (size_t i = 0; i < chirp_samples; i++) {
                float t = static_cast<float>(i) / config_.sample_rate;
                // Down-chirp: starts at f_end, decreases by k*t
                float phase = 2.0f * M_PI * (f_start_down * t - 0.5f * k * t * t);
                output[down_start + i] = config_.amplitude * std::sin(phase);
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
        constexpr size_t CORR_WINDOW = 480;  // 10ms
        constexpr size_t SEARCH_RANGE = 600;  // ±12.5ms = ±60 Hz CFO range

        int search_start = std::max(size_t(0), chirp_start - SEARCH_RANGE);
        int search_end = std::min(samples.size() - chirp_len, chirp_start + SEARCH_RANGE);

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
        float up_correlation = 0.0f;   // Up-chirp correlation value
        float down_correlation = 0.0f; // Down-chirp correlation value
    };

    DualChirpResult detectDualChirp(SampleSpan samples, float threshold = 0.15f) const {
        DualChirpResult result;

        if (!config_.use_dual_chirp) {
            // Fall back to single chirp detection using full template correlation
            // (more accurate timing than detectRobust's short-window correlation)
            auto [pos, corr] = detectChirpTemplate(samples, up_chirp_template_, threshold);
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
            return result;  // Not enough samples
        }

        // Detect UP chirp
        auto [up_pos, up_corr] = detectChirpTemplate(samples, up_chirp_template_, threshold);
        if (up_pos < 0) {
            return result;  // Up chirp not found
        }
        result.up_correlation = up_corr;

        // Detect DOWN chirp (search after UP chirp)
        size_t down_search_start = up_pos + chirp_len / 2;  // Start searching after up chirp
        if (down_search_start >= samples.size()) {
            return result;
        }

        SampleSpan down_search(samples.data() + down_search_start,
                               samples.size() - down_search_start);
        auto [down_pos_rel, down_corr] = detectChirpTemplate(down_search, down_chirp_template_, threshold);

        if (down_pos_rel < 0) {
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

        // DEBUG: Print positions and gaps
        fprintf(stderr, "[CHIRP] up_pos=%d, down_pos=%d, actual_gap=%d, expected_gap=%d, cfo_to_samples=%.1f\n",
                up_pos, down_pos, actual_gap, expected_gap, cfo_to_samples);

        // Position errors from expected
        // up_pos should be at true_start, but shifts by -CFO * cfo_to_samples
        // down_pos should be at true_start + chirp_len + gap, but shifts by +CFO * cfo_to_samples
        // So: actual_gap - expected_gap = 2 * CFO * cfo_to_samples
        float gap_error = static_cast<float>(actual_gap - expected_gap);
        result.cfo_hz = gap_error / (2.0f * cfo_to_samples);
        fprintf(stderr, "[CHIRP] gap_error=%.1f, estimated CFO=%.1f Hz\n", gap_error, result.cfo_hz);

        // Correct positions using estimated CFO
        float up_correction = result.cfo_hz * cfo_to_samples;
        result.up_chirp_start = static_cast<int>(std::round(up_pos + up_correction));
        result.down_chirp_start = static_cast<int>(std::round(down_pos - up_correction));

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
    Samples up_chirp_template_;    // Up-chirp (300→2700 Hz)
    Samples down_chirp_template_;  // Down-chirp (2700→300 Hz)
    float template_energy_ = 0.0f;
    float down_template_energy_ = 0.0f;

    // Detect chirp using specific template, returns (position, correlation)
    std::pair<int, float> detectChirpTemplate(SampleSpan samples,
                                               const Samples& chirp_template,
                                               float threshold) const {
        const size_t chirp_len = chirp_template.size();
        if (samples.size() < chirp_len) {
            return {-1, 0.0f};
        }

        const size_t search_len = samples.size() - chirp_len;

        // Calculate template energy
        float tmpl_energy = 0.0f;
        for (float s : chirp_template) tmpl_energy += s * s;

        float best_corr = 0.0f;
        int best_pos = -1;

        // Coarse search
        constexpr size_t COARSE_STEP = 48;  // 1ms at 48kHz
        for (size_t pos = 0; pos < search_len; pos += COARSE_STEP) {
            float corr = computeTemplateCorrelation(samples, pos, chirp_template, tmpl_energy);
            if (corr > best_corr) {
                best_corr = corr;
                best_pos = static_cast<int>(pos);
            }
        }

        if (best_pos < 0 || best_corr < threshold * 0.3f) {
            return {-1, best_corr};
        }

        // Fine search around coarse peak
        int fine_start = std::max(0, best_pos - static_cast<int>(COARSE_STEP));
        int fine_end = std::min(static_cast<int>(search_len), best_pos + static_cast<int>(COARSE_STEP));

        for (int pos = fine_start; pos <= fine_end; pos++) {
            float corr = computeTemplateCorrelation(samples, pos, chirp_template, tmpl_energy);
            if (corr > best_corr) {
                best_corr = corr;
                best_pos = pos;
            }
        }

        // Parabolic interpolation for sub-sample accuracy
        if (best_pos > 0 && best_pos < static_cast<int>(search_len) - 1) {
            float c0 = computeTemplateCorrelation(samples, best_pos - 1, chirp_template, tmpl_energy);
            float c1 = best_corr;
            float c2 = computeTemplateCorrelation(samples, best_pos + 1, chirp_template, tmpl_energy);

            float denom = 2.0f * (c0 - 2.0f * c1 + c2);
            if (std::abs(denom) > 1e-10f) {
                float delta = (c0 - c2) / denom;
                delta = std::max(-1.0f, std::min(1.0f, delta));
                // Return integer position (sub-sample precision tracked internally)
                best_pos = static_cast<int>(std::round(best_pos + delta));
            }
        }

        return (best_corr >= threshold) ? std::make_pair(best_pos, best_corr)
                                        : std::make_pair(-1, best_corr);
    }

    // Compute normalized correlation with specific template
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

        // Generate up-chirp template
        up_chirp_template_.resize(chirp_samples);
        template_energy_ = 0.0f;
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateUpChirpPhase(t);
            up_chirp_template_[i] = std::sin(phase);
            template_energy_ += up_chirp_template_[i] * up_chirp_template_[i];
        }

        // Generate down-chirp template
        down_chirp_template_.resize(chirp_samples);
        down_template_energy_ = 0.0f;
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateDownChirpPhase(t);
            down_chirp_template_[i] = std::sin(phase);
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
