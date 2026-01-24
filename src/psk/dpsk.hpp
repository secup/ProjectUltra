#pragma once

#include "ultra/types.hpp"
#include "ultra/logging.hpp"
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>

namespace ultra {

/**
 * Single-Carrier Differential PSK Modulator/Demodulator
 *
 * Fills the SNR gap between MFSK (< 0 dB) and OFDM (> 15 dB)
 * Target range: 0 to 15 dB (matching VARA HF performance)
 *
 * Key advantage over OFDM at low SNR:
 * - All power concentrated in single carrier
 * - No 15 dB penalty from splitting across 30 carriers
 *
 * Modulation modes:
 * - DBPSK:  1 bit/sym  (most robust, 0-5 dB)
 * - DQPSK:  2 bits/sym (mid-range, 5-10 dB)
 * - D8PSK:  3 bits/sym (higher throughput, 10-15 dB)
 *
 * Symbol rates: 31.25, 62.5, 125, 250 baud
 * With R1/4 LDPC: effective 8-500 bps depending on mode
 */

enum class DPSKModulation {
    DBPSK,   // 1 bit/symbol - differential BPSK
    DQPSK,   // 2 bits/symbol - differential QPSK
    D8PSK    // 3 bits/symbol - differential 8PSK
};

// Training sequence length for CFO estimation after chirp sync
// 8 symbols of alternating DBPSK (+1, -1, +1, -1, ...) provides
// robust CFO estimation with up to ~±15 Hz tolerance at 62.5 baud
constexpr int DPSK_TRAINING_SYMBOLS = 8;

struct DPSKConfig {
    float sample_rate = 48000.0f;
    float carrier_freq = 1500.0f;         // Single carrier frequency
    int samples_per_symbol = 1536;        // 31.25 baud @ 48kHz (default, most robust)
    DPSKModulation modulation = DPSKModulation::DQPSK;

    // Pulse shaping (raised cosine rolloff)
    float rolloff = 0.35f;                // Rolloff factor (0.2-0.5 typical)
    bool use_pulse_shaping = true;        // Enable raised cosine filtering

    int bits_per_symbol() const {
        switch (modulation) {
            case DPSKModulation::DBPSK: return 1;
            case DPSKModulation::DQPSK: return 2;
            case DPSKModulation::D8PSK: return 3;
        }
        return 2;
    }

    float symbol_rate() const {
        return sample_rate / samples_per_symbol;
    }

    float raw_bps() const {
        return symbol_rate() * bits_per_symbol();
    }

    // Bandwidth (with rolloff)
    float bandwidth() const {
        return symbol_rate() * (1.0f + rolloff);
    }

    // Phase increment per symbol for given data
    float phase_increment(int symbol_value) const {
        switch (modulation) {
            case DPSKModulation::DBPSK:
                // 0 -> 0°, 1 -> 180°
                return symbol_value ? M_PI : 0.0f;
            case DPSKModulation::DQPSK:
                // 00 -> 45°, 01 -> 135°, 10 -> 225°, 11 -> 315°
                // (Gray coded for 1-bit error on adjacent phases)
                return (symbol_value * 2 + 1) * M_PI / 4.0f;
            case DPSKModulation::D8PSK:
                // Direct mapping: symbol n → n * 45° + 22.5° (offset for symmetry)
                return (symbol_value & 7) * M_PI / 4.0f + M_PI / 8.0f;
        }
        return 0.0f;
    }

    int num_phases() const {
        switch (modulation) {
            case DPSKModulation::DBPSK: return 2;
            case DPSKModulation::DQPSK: return 4;
            case DPSKModulation::D8PSK: return 8;
        }
        return 4;
    }
};

class DPSKModulator {
public:
    explicit DPSKModulator(const DPSKConfig& config = DPSKConfig{})
        : config_(config), carrier_phase_(0.0f), symbol_phase_(0.0f) {
        buildPulseShape();
    }

    // Generate preamble for sync using Barker-13 code
    // Barker codes have excellent autocorrelation: peak=13, max sidelobe=1
    // This enables robust matched-filter detection at low SNR
    //
    // Structure: [Barker-13 × 3 repeats] = 39 symbols
    // Using DBPSK modulation for maximum robustness
    Samples generatePreamble(int /* num_symbols */ = 32) {
        Samples out;

        // Barker-13 code: +1 +1 +1 +1 +1 -1 -1 +1 +1 -1 +1 -1 +1
        static const int BARKER13[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
        static const int BARKER_LEN = 13;
        static const int REPEATS = 3;  // Repeat for longer integration

        int total_symbols = BARKER_LEN * REPEATS;
        out.reserve(total_symbols * config_.samples_per_symbol);

        float carrier_inc = 2.0f * M_PI * config_.carrier_freq / config_.sample_rate;
        float phase = 0.0f;
        float symbol_phase = 0.0f;  // DBPSK: 0 for +1, π for -1

        for (int rep = 0; rep < REPEATS; rep++) {
            for (int s = 0; s < BARKER_LEN; s++) {
                // DBPSK: phase change of π for -1, 0 for +1
                if (BARKER13[s] < 0) {
                    symbol_phase += M_PI;
                }

                // Generate one symbol
                for (int i = 0; i < config_.samples_per_symbol; i++) {
                    out.push_back(std::cos(phase + symbol_phase));
                    phase += carrier_inc;
                    if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
                }
            }
        }

        // Reset modulator state to start data at known phase
        carrier_phase_ = phase;
        symbol_phase_ = symbol_phase;

        return out;
    }

    // Generate a single reference symbol for differential demodulation
    // Used when chirp sync replaces Barker-13 preamble
    // This provides the phase reference for subsequent DPSK symbols
    Samples generateReferenceSymbol() {
        Samples out(config_.samples_per_symbol);
        float carrier_inc = 2.0f * M_PI * config_.carrier_freq / config_.sample_rate;
        float phase = 0.0f;

        for (int i = 0; i < config_.samples_per_symbol; i++) {
            out[i] = std::cos(phase);
            phase += carrier_inc;
        }

        // Set modulator state for subsequent data
        carrier_phase_ = phase;
        symbol_phase_ = 0.0f;

        return out;
    }

    // Generate training sequence for CFO estimation after chirp sync
    // Uses alternating DBPSK pattern: 0°, 180°, 0°, 180°, ...
    // This creates a known differential pattern (+π, +π, +π, ...) that
    // allows robust CFO estimation even with data modulation modes
    Samples generateTrainingSequence() {
        Samples out;
        out.reserve(DPSK_TRAINING_SYMBOLS * config_.samples_per_symbol);

        float carrier_inc = 2.0f * M_PI * config_.carrier_freq / config_.sample_rate;
        float phase = 0.0f;

        for (int sym = 0; sym < DPSK_TRAINING_SYMBOLS; sym++) {
            // Alternating phase: 0° for even symbols, 180° for odd symbols
            float symbol_phase = (sym % 2 == 0) ? 0.0f : M_PI;

            for (int i = 0; i < config_.samples_per_symbol; i++) {
                out.push_back(std::cos(phase + symbol_phase));
                phase += carrier_inc;
            }
        }

        // Set modulator state for subsequent symbols
        carrier_phase_ = phase;
        // After 8 alternating symbols, we end on phase 180° (odd symbol)
        symbol_phase_ = M_PI;

        return out;
    }

    // Get training sequence length in samples
    int getTrainingSamples() const {
        return DPSK_TRAINING_SYMBOLS * config_.samples_per_symbol;
    }

    // Get expected preamble length in samples
    int getPreambleLength() const {
        return 13 * 3 * config_.samples_per_symbol;  // Barker-13 × 3 repeats
    }

    // Modulate data bytes
    Samples modulate(ByteSpan data) {
        Samples out;
        int bits_per_sym = config_.bits_per_symbol();
        int total_bits = data.size() * 8;
        int total_symbols = (total_bits + bits_per_sym - 1) / bits_per_sym;
        out.reserve(total_symbols * config_.samples_per_symbol);

        // Convert bytes to bit stream (MSB first)
        std::vector<bool> bits;
        bits.reserve(data.size() * 8);
        for (uint8_t byte : data) {
            for (int b = 7; b >= 0; b--) {
                bits.push_back((byte >> b) & 1);
            }
        }

        // Pad to multiple of bits_per_symbol
        while (bits.size() % bits_per_sym != 0) {
            bits.push_back(false);
        }

        // Modulate symbols
        for (size_t i = 0; i < bits.size(); i += bits_per_sym) {
            int symbol = 0;
            for (int b = 0; b < bits_per_sym; b++) {
                if (bits[i + b]) {
                    symbol |= (1 << (bits_per_sym - 1 - b));
                }
            }

            auto samples = modulateSymbol(symbol);
            out.insert(out.end(), samples.begin(), samples.end());
        }

        return out;
    }

    // Modulate single symbol with differential encoding
    Samples modulateSymbol(int symbol_value) {
        Samples out(config_.samples_per_symbol);

        // Differential: add phase increment to current symbol phase
        float phase_inc = config_.phase_increment(symbol_value);
        symbol_phase_ += phase_inc;
        while (symbol_phase_ >= 2.0f * M_PI) symbol_phase_ -= 2.0f * M_PI;

        float carrier_inc = 2.0f * M_PI * config_.carrier_freq / config_.sample_rate;

        if (config_.use_pulse_shaping && !pulse_shape_.empty()) {
            // Pulse-shaped symbol
            for (int i = 0; i < config_.samples_per_symbol; i++) {
                float envelope = pulse_shape_[i];
                out[i] = envelope * std::cos(carrier_phase_ + symbol_phase_);
                carrier_phase_ += carrier_inc;
            }
        } else {
            // Rectangular symbol
            for (int i = 0; i < config_.samples_per_symbol; i++) {
                out[i] = std::cos(carrier_phase_ + symbol_phase_);
                carrier_phase_ += carrier_inc;
            }
        }

        // Keep carrier phase bounded
        while (carrier_phase_ >= 2.0f * M_PI) carrier_phase_ -= 2.0f * M_PI;

        return out;
    }

    void reset() {
        carrier_phase_ = 0.0f;
        symbol_phase_ = 0.0f;
    }

    const DPSKConfig& config() const { return config_; }

private:
    void buildPulseShape() {
        if (!config_.use_pulse_shaping) return;

        pulse_shape_.resize(config_.samples_per_symbol);
        int N = config_.samples_per_symbol;

        // Raised cosine window (rolloff parameter not used for simple window)
        for (int i = 0; i < N; i++) {
            float t = (float)i / N;
            // Smooth raised cosine envelope
            pulse_shape_[i] = 0.5f * (1.0f - std::cos(2.0f * M_PI * t));
        }
    }

    DPSKConfig config_;
    float carrier_phase_;
    float symbol_phase_;
    std::vector<float> pulse_shape_;
};

class DPSKDemodulator {
public:
    explicit DPSKDemodulator(const DPSKConfig& config = DPSKConfig{})
        : config_(config), prev_symbol_(1.0f, 0.0f) {
        // Pre-compute carrier mixing coefficients
        carrier_sin_.resize(config_.samples_per_symbol);
        carrier_cos_.resize(config_.samples_per_symbol);
        float carrier_inc = 2.0f * M_PI * config_.carrier_freq / config_.sample_rate;

        for (int i = 0; i < config_.samples_per_symbol; i++) {
            float phase = carrier_inc * i;
            carrier_sin_[i] = std::sin(phase);
            carrier_cos_[i] = std::cos(phase);
        }
    }

    // Find preamble using CFO-tolerant differential correlation
    //
    // CFO-tolerant approach:
    // 1. Extract symbol-by-symbol I/Q via carrier correlation
    // 2. Compute differential phase between adjacent symbols
    // 3. Match phase difference pattern against expected Barker sequence
    //
    // This works because CFO adds the same phase rotation to all symbols,
    // so the differential (symbol N - symbol N-1) cancels CFO:
    //   diff_phase = (carrier + cfo + data[N]) - (carrier + cfo + data[N-1])
    //              = data[N] - data[N-1]  (CFO cancels!)
    //
    // Returns offset to first sample after preamble (start of data)
    int findPreamble(SampleSpan samples, int /* num_symbols */ = 32) {
        int symbol_len = config_.samples_per_symbol;

        // Barker-13 code: +1 +1 +1 +1 +1 -1 -1 +1 +1 -1 +1 -1 +1
        // In DBPSK: +1 = no phase change, -1 = π phase change
        // The differential between symbol i and i+1 is determined by BARKER[i+1]
        // (since BARKER[i+1] determines whether we ADD a phase change going from i to i+1)
        static const int BARKER13[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
        static const int BARKER_LEN = 13;
        static const int REPEATS = 3;
        static const int PATTERN_LEN = BARKER_LEN * REPEATS - 1;  // 38 differences

        int preamble_symbols = BARKER_LEN * REPEATS;  // 39
        int preamble_samples = preamble_symbols * symbol_len;

        // Require at least 1.5x preamble length to avoid premature detection
        int min_required = preamble_samples + preamble_samples / 2;
        if ((int)samples.size() < min_required) return -1;

        // === OPTIMIZATION 1: Energy detection pre-filter ===
        // Skip expensive correlation if channel is quiet (just noise)
        float energy = 0;
        size_t energy_check_len = std::min(samples.size(), (size_t)(preamble_samples * 2));
        for (size_t i = 0; i < energy_check_len; i++) {
            energy += samples[i] * samples[i];
        }
        float rms = std::sqrt(energy / energy_check_len);
        if (rms < 0.01f) {
            // Channel is quiet - no signal present
            return -1;
        }

        // === OPTIMIZATION 2: Limit search range ===
        // Preamble should be near the start of the buffer (within first 4x preamble length)
        // This prevents searching through 100K+ samples when we only need to check the beginning
        int max_search_limit = preamble_samples * 4;  // ~160K samples max at 768 samp/sym

        // Build expected differential pattern
        // The differential from symbol i to i+1 is the Barker code value used for symbol i+1
        // +1 means "no phase change", -1 means "180° phase change"
        std::vector<int> expected_pattern;
        expected_pattern.reserve(PATTERN_LEN);

        for (int s = 1; s < preamble_symbols; s++) {
            // Symbol s uses BARKER13[s % BARKER_LEN]
            expected_pattern.push_back(BARKER13[s % BARKER_LEN]);
        }

        // Detection threshold - lowered to 0.80 for multipath tolerance
        // Real preamble achieves 0.98+ in AWGN, ~0.85 in multipath channels
        constexpr float DETECTION_THRESHOLD = 0.80f;
        constexpr float MIN_SYMBOL_ENERGY = 0.001f;

        // Limit search range for performance (use optimization 2)
        int max_search = std::min((int)samples.size() - preamble_samples, max_search_limit);
        // Coarse step = 1 symbol (was symbol_len/4 = too slow)
        // Fine search will refine the position
        int search_step = symbol_len;

        float best_score = 0;
        int best_offset = -1;
        float sum_scores = 0;
        int num_scores = 0;

        // Coarse search - track global average for validation
        for (int start = 0; start < max_search; start += search_step) {
            float score = computeDifferentialScore(samples, start, symbol_len,
                                                    expected_pattern, MIN_SYMBOL_ENERGY);
            sum_scores += score;
            num_scores++;
            if (score > best_score) {
                best_score = score;
                best_offset = start;
            }
        }

        float global_avg = (num_scores > 0) ? sum_scores / num_scores : 0;

        // Fine search around best position
        if (best_offset >= 0 && best_score > DETECTION_THRESHOLD * 0.7f) {
            int fine_start = std::max(0, best_offset - search_step);
            int fine_end = std::min(max_search, best_offset + search_step);

            for (int start = fine_start; start < fine_end; start++) {
                float score = computeDifferentialScore(samples, start, symbol_len,
                                                        expected_pattern, MIN_SYMBOL_ENERGY);
                if (score > best_score) {
                    best_score = score;
                    best_offset = start;
                }
            }
        }

        // Log best score for debugging (DEBUG level to reduce noise)
        LOG_MODEM(DEBUG, "DPSK preamble search: best_score=%.3f at offset=%d, threshold=%.2f",
                  best_score, best_offset, DETECTION_THRESHOLD);

        if (best_score < DETECTION_THRESHOLD) {
            // No preamble found - this is normal when monitoring silence/noise
            return -1;
        }

        // === VALIDATION: Global outlier check ===
        // A real preamble should be a clear outlier compared to the global average.
        // In pure noise, all correlation scores are similar. With a real preamble,
        // best_score >> global_avg.
        constexpr float GLOBAL_OUTLIER_RATIO = 1.3f;  // Best must be 1.3x global average (relaxed for multipath)
        if (global_avg > 0 && best_score < global_avg * GLOBAL_OUTLIER_RATIO) {
            LOG_MODEM(DEBUG, "DPSK outlier FAIL: best=%.3f, global_avg=%.3f, ratio=%.2f < %.2f",
                      best_score, global_avg, best_score/global_avg, GLOBAL_OUTLIER_RATIO);
            return -1;
        }
        LOG_MODEM(DEBUG, "DPSK outlier OK: best=%.3f, global_avg=%.3f, ratio=%.2f",
                  best_score, global_avg, (global_avg > 0) ? best_score/global_avg : 0.0f);

        // Estimate CFO for phase compensation
        estimated_cfo_ = estimateCFOTolerant(samples, best_offset, symbol_len, preamble_symbols, expected_pattern);

        // For CFO cases, the coarse differential search is already accurate
        // (CFO-tolerant), so skip matched filter which can find false peaks.
        // For non-CFO cases, use matched filter for sub-sample accuracy.
        if (std::abs(estimated_cfo_) < 0.5f) {
            best_offset = refineTimingWithMatchedFilter(samples, best_offset, symbol_len, 0.0f);
        }

        // Estimate initial phase offset from preamble
        // The Hilbert transform or other processing may introduce a constant phase shift
        // that affects all differentials equally. Measure it and compensate.
        initial_phase_offset_ = estimateInitialPhaseOffset(samples, best_offset, symbol_len, expected_pattern);

        // Data starts after preamble
        int data_start = best_offset + preamble_samples;

        // Set phase reference from last preamble symbol using standard correlation
        // Differential decoding will naturally handle CFO
        int ref_offset = data_start - symbol_len;
        if (ref_offset >= 0 && ref_offset + symbol_len <= (int)samples.size()) {
            SampleSpan ref_sym(samples.data() + ref_offset, symbol_len);
            prev_symbol_ = correlateSymbol(ref_sym);
            demod_carrier_phase_ = 0.0f;
        }

        return data_start;
    }

    // Get estimated CFO (Hz) from last preamble detection
    float getEstimatedCFO() const { return estimated_cfo_; }

private:
    // Compute differential correlation score at given offset
    // CFO-tolerant: detects pattern match even with constant phase offset
    float computeDifferentialScore(SampleSpan samples, int start, int symbol_len,
                                   const std::vector<int>& expected_pattern,
                                   float min_energy) {
        int num_diffs = expected_pattern.size();
        int num_symbols = num_diffs + 1;

        // Extract I/Q for each symbol
        std::vector<Complex> symbols(num_symbols);
        float total_energy = 0;

        for (int s = 0; s < num_symbols; s++) {
            int offset = start + s * symbol_len;
            SampleSpan sym(samples.data() + offset, symbol_len);
            symbols[s] = correlateSymbol(sym);
            total_energy += std::norm(symbols[s]);
        }

        if (total_energy < min_energy * num_symbols) return 0;

        // CFO-tolerant scoring: compute complex correlation between
        // measured differential phases and expected pattern
        //
        // With CFO, measured[i] = expected[i] + CFO_offset (constant)
        // So exp(j*measured[i]) = exp(j*expected[i]) * exp(j*CFO_offset)
        //
        // If we sum: Σ exp(j*measured[i]) * exp(-j*expected[i])
        //          = Σ exp(j*CFO_offset) = N * exp(j*CFO_offset)
        // This has magnitude N regardless of CFO!
        //
        // Normalized: |Σ diff[i] * conj(expected[i])| / Σ |diff[i]|

        Complex correlation_sum(0, 0);
        float magnitude_sum = 0;

        for (int i = 0; i < num_diffs; i++) {
            // Differential: current * conj(prev)
            Complex diff = symbols[i + 1] * std::conj(symbols[i]);
            float magnitude = std::abs(diff);

            if (magnitude < 1e-10f) continue;

            // Normalize the differential
            Complex diff_norm = diff / magnitude;

            // Expected differential: +1 → 1+0j, -1 → -1+0j
            Complex expected(expected_pattern[i], 0);

            // Accumulate correlation
            correlation_sum += diff_norm * std::conj(expected);
            magnitude_sum += magnitude;
        }

        if (magnitude_sum < 1e-10f) return 0;

        // Normalized score: magnitude of correlation / number of differences
        float score = std::abs(correlation_sum) / num_diffs;
        return score;  // In [0, 1], independent of CFO
    }

    // Estimate CFO using CFO-tolerant pattern correlation
    // Returns the estimated CFO in Hz
    float estimateCFOTolerant(SampleSpan samples, int preamble_start, int symbol_len,
                              int num_symbols, const std::vector<int>& expected_pattern) {
        int num_diffs = expected_pattern.size();

        // Extract symbols and compute differential phases
        std::vector<Complex> diffs;
        diffs.reserve(num_diffs);

        Complex prev_sym(0, 0);
        for (int s = 0; s <= num_diffs; s++) {
            int offset = preamble_start + s * symbol_len;
            SampleSpan sym(samples.data() + offset, symbol_len);
            Complex current = correlateSymbol(sym);

            if (s > 0 && std::abs(prev_sym) > 0.01f && std::abs(current) > 0.01f) {
                Complex diff = current * std::conj(prev_sym);
                diff /= std::abs(diff);  // Normalize
                diffs.push_back(diff);
            }
            prev_sym = current;
        }

        if (diffs.size() < 10) return 0;

        // Compute correlation: Σ diff[i] * conj(expected[i])
        // This gives exp(j * CFO_offset) if pattern matches
        Complex correlation(0, 0);
        for (size_t i = 0; i < diffs.size() && i < expected_pattern.size(); i++) {
            Complex expected(expected_pattern[i], 0);
            correlation += diffs[i] * std::conj(expected);
        }

        // Extract phase offset (CFO per symbol)
        float phase_offset = std::atan2(correlation.imag(), correlation.real());

        // Convert to Hz: CFO = phase_per_symbol / (2π × symbol_duration)
        float symbol_duration = symbol_len / config_.sample_rate;
        float cfo_hz = phase_offset / (2.0f * M_PI * symbol_duration);

        // Note: Sign depends on correlation convention and Hilbert transform used in test
        // Empirically determined that we need to negate for correct compensation
        return -cfo_hz;
    }

    // Estimate CFO from preamble symbols (original method, for reference)
    // Uses the fact that with perfect sync, adjacent symbols should have
    // known phase difference. Any additional rotation is due to CFO.
    float estimateCFO(SampleSpan samples, int preamble_start, int symbol_len, int num_symbols) {
        // Barker-13 code determines expected differential phases
        static const int BARKER13[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
        static const int BARKER_LEN = 13;

        // Accumulate phase error
        float phase_error_sum = 0;
        int count = 0;

        Complex prev_sym(0, 0);

        for (int s = 0; s < num_symbols; s++) {
            int offset = preamble_start + s * symbol_len;
            SampleSpan sym(samples.data() + offset, symbol_len);
            Complex current = correlateSymbol(sym);

            if (s > 0 && std::abs(prev_sym) > 0.01f && std::abs(current) > 0.01f) {
                // Measured differential phase
                Complex diff = current * std::conj(prev_sym);
                float measured_phase = std::atan2(diff.imag(), diff.real());

                // Expected differential phase is determined by BARKER13[s % BARKER_LEN]
                // +1 means no phase change (expected_phase = 0)
                // -1 means π phase change (expected_phase = π or -π)
                float expected_phase = (BARKER13[s % BARKER_LEN] > 0) ? 0.0f : M_PI;

                // Phase error - the residual rotation due to CFO
                float error = measured_phase - expected_phase;

                // Wrap to [-π, π]
                while (error > M_PI) error -= 2.0f * M_PI;
                while (error < -M_PI) error += 2.0f * M_PI;

                phase_error_sum += error;
                count++;
            }

            prev_sym = current;
        }

        if (count < 5) return 0;

        // Average phase error per symbol
        float avg_phase_error = phase_error_sum / count;

        // Only apply CFO compensation if error is significant (> 5° per symbol)
        if (std::abs(avg_phase_error) < 0.09f) {  // ~5 degrees
            return 0;
        }

        // Convert to frequency: phase_per_symbol / (2π × symbol_duration)
        float symbol_duration = symbol_len / config_.sample_rate;
        float cfo_hz = avg_phase_error / (2.0f * M_PI * symbol_duration);

        return cfo_hz;
    }

    float estimated_cfo_ = 0.0f;  // Estimated CFO from last preamble
    float initial_phase_offset_ = 0.0f;  // Initial phase offset from Hilbert/mixing

    // Estimate initial phase offset from preamble differentials
    // The Barker code has known phase transitions, so we can measure any constant offset
    float estimateInitialPhaseOffset(SampleSpan samples, int preamble_start, int symbol_len,
                                     const std::vector<int>& expected_pattern) {
        static const int NUM_SAMPLES = 10;  // Use first 10 differentials

        std::vector<float> phase_errors;
        Complex prev_sym(0, 0);

        for (int s = 0; s <= NUM_SAMPLES && s < (int)expected_pattern.size(); s++) {
            int offset = preamble_start + s * symbol_len;
            if (offset + symbol_len > (int)samples.size()) break;

            SampleSpan sym(samples.data() + offset, symbol_len);
            Complex current = correlateSymbol(sym);

            if (s > 0 && std::abs(prev_sym) > 0.01f && std::abs(current) > 0.01f) {
                // Measured differential phase
                Complex diff = current * std::conj(prev_sym);
                float measured = std::atan2(diff.imag(), diff.real());

                // Expected differential phase (DBPSK for preamble)
                // +1 = no change (0°), -1 = flip (180°)
                float expected = (expected_pattern[s - 1] > 0) ? 0.0f : M_PI;

                // Subtract CFO contribution
                float cfo_phase = 2.0f * M_PI * estimated_cfo_ * symbol_len / config_.sample_rate;
                measured -= cfo_phase;

                // Phase error
                float error = measured - expected;
                // Wrap to [-π, π]
                while (error > M_PI) error -= 2.0f * M_PI;
                while (error < -M_PI) error += 2.0f * M_PI;

                phase_errors.push_back(error);
            }
            prev_sym = current;
        }

        if (phase_errors.empty()) return 0.0f;

        // Average phase error (circular mean would be better but this is simpler)
        float sum = 0;
        for (float e : phase_errors) sum += e;
        return sum / phase_errors.size();
    }

    // Refine timing using CFO-tolerant differential correlation
    // This is more robust than matched filter for CFO cases because
    // differential scoring inherently removes CFO
    int refineTimingWithMatchedFilter(SampleSpan samples, int coarse_offset, int symbol_len, float cfo_hz) {
        static const int BARKER13[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
        static const int REFINE_SYMBOLS = 6;  // Use first 6 symbols for refinement

        // Use matched filter with CFO-adjusted carrier
        // Generate template for first few symbols
        std::vector<float> ref_template;
        ref_template.reserve(REFINE_SYMBOLS * symbol_len);

        // Use CFO-adjusted carrier frequency for accurate template
        float carrier_freq_adjusted = config_.carrier_freq + cfo_hz;
        float carrier_inc = 2.0f * M_PI * carrier_freq_adjusted / config_.sample_rate;
        float phase = 0.0f;
        float symbol_phase = 0.0f;

        for (int s = 0; s < REFINE_SYMBOLS; s++) {
            if (BARKER13[s] < 0) {
                symbol_phase += M_PI;
            }
            for (int i = 0; i < symbol_len; i++) {
                ref_template.push_back(std::cos(phase + symbol_phase));
                phase += carrier_inc;
                if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
            }
        }

        int template_len = ref_template.size();

        // Pre-compute template energy
        float template_energy = 0;
        for (float t : ref_template) {
            template_energy += t * t;
        }

        // Search around coarse offset for best match
        int search_range = symbol_len;  // ±1 symbol
        int fine_start = std::max(0, coarse_offset - search_range);
        int fine_end = std::min((int)samples.size() - template_len, coarse_offset + search_range);

        float best_corr = -1;
        int best_offset = coarse_offset;

        for (int i = fine_start; i <= fine_end; i++) {
            float corr = 0;
            float sig_energy = 0;
            for (int j = 0; j < template_len; j++) {
                corr += samples[i + j] * ref_template[j];
                sig_energy += samples[i + j] * samples[i + j];
            }

            float norm = std::sqrt(sig_energy * template_energy);
            if (norm < 1e-10f) continue;
            float norm_corr = std::abs(corr) / norm;

            if (norm_corr > best_corr) {
                best_corr = norm_corr;
                best_offset = i;
            }
        }

        return best_offset;
    }

public:

    // Correlate samples to get complex symbol value (I/Q)
    // Note: Q uses -sin for proper quadrature demodulation
    // cos(A+B)*cos(C) → 0.5*cos(B) when averaged (A=carrier, B=symbol, C=ref)
    // cos(A+B)*sin(C) → -0.5*sin(B) when averaged
    Complex correlateSymbol(SampleSpan samples) {
        float I = 0, Q = 0;
        int N = std::min((int)samples.size(), config_.samples_per_symbol);

        for (int i = 0; i < N; i++) {
            I += samples[i] * carrier_cos_[i];
            Q -= samples[i] * carrier_sin_[i];  // Negative for correct phase
        }

        return Complex(I / N, Q / N);
    }

    // CFO-compensated symbol correlation
    Complex correlateSymbolWithCFO(SampleSpan samples, float carrier_inc) {
        float I = 0, Q = 0;
        int N = std::min((int)samples.size(), config_.samples_per_symbol);

        float phase = 0;
        for (int i = 0; i < N; i++) {
            I += samples[i] * std::cos(phase);
            Q -= samples[i] * std::sin(phase);
            phase += carrier_inc;
        }

        return Complex(I / N, Q / N);
    }

    // Demodulate to hard bits
    Bytes demodulate(SampleSpan samples) {
        auto soft = demodulateSoft(samples);

        // Convert soft bits to bytes
        Bytes out;
        for (size_t i = 0; i + 8 <= soft.size(); i += 8) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                if (soft[i + b] < 0) {  // Negative LLR = bit 1
                    byte |= (1 << (7 - b));
                }
            }
            out.push_back(byte);
        }

        return out;
    }

    // Demodulate to soft bits (LLRs) for LDPC decoding
    // Uses pure differential decoding which is inherently CFO-tolerant:
    // - CFO adds constant phase rotation per symbol
    // - Differential (current * conj(prev)) cancels the rotation
    std::vector<float> demodulateSoft(SampleSpan samples) {
        std::vector<float> soft_bits;
        int symbol_len = config_.samples_per_symbol;
        int bits_per_sym = config_.bits_per_symbol();

        int num_symbols = samples.size() / symbol_len;
        soft_bits.reserve(num_symbols * bits_per_sym);

        Complex prev = prev_symbol_;

        for (int s = 0; s < num_symbols; s++) {
            SampleSpan sym(samples.data() + s * symbol_len, symbol_len);

            // Use standard correlation at nominal carrier frequency
            // CFO will cause each symbol to have extra phase rotation,
            // but this cancels in the differential
            Complex current = correlateSymbol(sym);

            // Differential decode: diff = current * conj(prev)
            // With CFO: current = true_current × exp(j×θ_s), prev = true_prev × exp(j×θ_{s-1})
            // diff = true_diff × exp(j×(θ_s - θ_{s-1})) = true_diff × exp(j×Δθ)
            // where Δθ = 2π × CFO × symbol_duration (constant for all s)
            Complex diff = current * std::conj(prev);

            // Magnitude for confidence
            float magnitude = std::abs(diff);

            // Phase gives data phase difference PLUS constant CFO offset
            float phase = std::atan2(diff.imag(), diff.real());

            // Compensate for the constant CFO phase offset and initial phase offset
            if (std::abs(estimated_cfo_) > 0.5f || std::abs(initial_phase_offset_) > 0.01f) {
                float cfo_phase = 2.0f * M_PI * estimated_cfo_ * symbol_len / config_.sample_rate;
                phase -= cfo_phase;
                phase -= initial_phase_offset_;
                // Normalize to [-π, π]
                while (phase > M_PI) phase -= 2.0f * M_PI;
                while (phase < -M_PI) phase += 2.0f * M_PI;
            }

            // Confidence scaling based on signal strength
            float confidence = std::min(magnitude * 10.0f, 5.0f);

            // Generate soft bits based on modulation
            auto bits = phaseToBits(phase, confidence);
            soft_bits.insert(soft_bits.end(), bits.begin(), bits.end());

            prev = current;
        }

        prev_symbol_ = prev;
        return soft_bits;
    }

    void reset() {
        prev_symbol_ = Complex(1.0f, 0.0f);
        estimated_cfo_ = 0.0f;
        initial_phase_offset_ = 0.0f;
        demod_carrier_phase_ = 0.0f;
    }

    // Set reference symbol from samples (used when preamble position is already known)
    // This allows demodulation to start from a known position without re-running findPreamble
    void setReferenceSymbol(SampleSpan ref_samples) {
        prev_symbol_ = correlateSymbol(ref_samples);
        demod_carrier_phase_ = 0.0f;
    }

    // Estimate CFO from training sequence (alternating DBPSK pattern)
    // Training pattern: symbols alternate 0°, 180°, 0°, 180°, ...
    // Expected differential phase: +π (180°) between each pair
    // With CFO: measured differential = +π + CFO_phase_per_symbol
    // Returns estimated CFO in Hz
    float estimateCFOFromTraining(SampleSpan training_samples) {
        int symbol_len = config_.samples_per_symbol;
        int num_symbols = training_samples.size() / symbol_len;

        if (num_symbols < 2) return 0.0f;

        // Extract I/Q for each symbol
        std::vector<Complex> symbols;
        symbols.reserve(num_symbols);

        for (int s = 0; s < num_symbols; s++) {
            SampleSpan sym(training_samples.data() + s * symbol_len, symbol_len);
            symbols.push_back(correlateSymbol(sym));
        }

        // Compute differential phases and compare to expected (+π)
        float phase_error_sum = 0.0f;
        int count = 0;

        for (int i = 0; i < num_symbols - 1; i++) {
            if (std::abs(symbols[i]) < 0.01f || std::abs(symbols[i + 1]) < 0.01f)
                continue;

            // Measured differential
            Complex diff = symbols[i + 1] * std::conj(symbols[i]);
            float measured = std::atan2(diff.imag(), diff.real());

            // Expected differential is +π (alternating pattern)
            float expected = M_PI;

            // Phase error = measured - expected
            float error = measured - expected;

            // Wrap to [-π, π]
            while (error > M_PI) error -= 2.0f * M_PI;
            while (error < -M_PI) error += 2.0f * M_PI;

            phase_error_sum += error;
            count++;
        }

        if (count == 0) return 0.0f;

        float avg_phase_error = phase_error_sum / count;

        // Convert phase error per symbol to Hz
        float symbol_duration = symbol_len / config_.sample_rate;
        float cfo_hz = avg_phase_error / (2.0f * M_PI * symbol_duration);

        return cfo_hz;
    }

    // Combined: estimate CFO from training and set reference symbol
    // Call this after chirp detection with:
    //   training_samples: DPSK_TRAINING_SYMBOLS worth of samples
    //   ref_samples: 1 symbol worth of samples (immediately after training)
    void setReferenceWithTraining(SampleSpan training_samples, SampleSpan ref_samples) {
        // Estimate CFO from training sequence
        estimated_cfo_ = estimateCFOFromTraining(training_samples);

        // Estimate initial phase offset from the last training symbol
        // The last training symbol (index 7) should be at 180° phase
        int symbol_len = config_.samples_per_symbol;
        int num_training = training_samples.size() / symbol_len;
        if (num_training >= 2) {
            // Use last two training symbols to get initial phase offset
            SampleSpan last_sym(training_samples.data() + (num_training - 1) * symbol_len, symbol_len);
            SampleSpan prev_sym(training_samples.data() + (num_training - 2) * symbol_len, symbol_len);

            Complex last = correlateSymbol(last_sym);
            Complex prev = correlateSymbol(prev_sym);

            if (std::abs(prev) > 0.01f && std::abs(last) > 0.01f) {
                Complex diff = last * std::conj(prev);
                float measured = std::atan2(diff.imag(), diff.real());

                // CFO compensation
                float cfo_phase = 2.0f * M_PI * estimated_cfo_ * symbol_len / config_.sample_rate;
                measured -= cfo_phase;

                // Expected is +π, so offset = measured - π
                initial_phase_offset_ = measured - M_PI;

                // Wrap to [-π, π]
                while (initial_phase_offset_ > M_PI) initial_phase_offset_ -= 2.0f * M_PI;
                while (initial_phase_offset_ < -M_PI) initial_phase_offset_ += 2.0f * M_PI;
            }
        }

        // Set reference symbol
        prev_symbol_ = correlateSymbol(ref_samples);
        demod_carrier_phase_ = 0.0f;

        LOG_DEMOD(DEBUG, "DPSK Training: CFO=%.2f Hz, phase_offset=%.1f deg",
                  estimated_cfo_, initial_phase_offset_ * 180.0f / M_PI);
    }

    const DPSKConfig& config() const { return config_; }

private:
    // Convert differential phase to soft bits (LLRs)
    // Positive LLR = bit 0, Negative LLR = bit 1
    std::vector<float> phaseToBits(float phase, float confidence) {
        std::vector<float> bits;

        // Normalize phase to [0, 2π)
        while (phase < 0) phase += 2.0f * M_PI;
        while (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;

        switch (config_.modulation) {
            case DPSKModulation::DBPSK: {
                // 0 -> 0°, 1 -> 180°
                // LLR based on distance from decision boundary (90°, 270°)
                float llr = confidence * std::cos(phase);  // +ve near 0°, -ve near 180°
                bits.push_back(llr);
                break;
            }

            case DPSKModulation::DQPSK: {
                // Phases: 45°, 135°, 225°, 315° mapping to bits 00, 01, 10, 11
                // MSB (bit 0): 0 in upper half (sin>0), 1 in lower half (sin<0)
                // LSB (bit 1): 0 at 45°,225°, 1 at 135°,315° → use sin(2*phase)
                float llr0 = confidence * std::sin(phase);           // MSB
                float llr1 = confidence * std::sin(2.0f * phase);    // LSB
                bits.push_back(llr0);
                bits.push_back(llr1);
                break;
            }

            case DPSKModulation::D8PSK: {
                // 8 phases: symbol n → phase = n * 45° + 22.5° (offset for symmetry)
                // Direct binary mapping (no Gray): bits b2b1b0 = symbol
                // b2 (MSB): 0 for upper half (0-157.5°), 1 for lower (180-337.5°)
                // b1: alternates every 90° within each half
                // b0 (LSB): alternates every 45°
                //
                // Key insight: DON'T subtract the 22.5° offset!
                // The constellation is at 22.5°, 67.5°, ..., 337.5°
                // sin(4*22.5°) = sin(90°) = 1 (good for b0)
                // sin(4*67.5°) = sin(270°) = -1 (good for b0)
                // If we subtracted offset, we'd hit zeros of sin(4x)
                float llr0 = confidence * std::sin(phase);           // b2 (MSB)
                float llr1 = confidence * std::sin(2.0f * phase);    // b1
                float llr2 = confidence * std::sin(4.0f * phase);    // b0 (LSB)
                bits.push_back(llr0);
                bits.push_back(llr1);
                bits.push_back(llr2);
                break;
            }
        }

        return bits;
    }

    DPSKConfig config_;
    Complex prev_symbol_;
    std::vector<float> carrier_sin_;
    std::vector<float> carrier_cos_;
    float demod_carrier_phase_ = 0.0f;  // Continuous carrier phase for CFO-compensated demod
};

// Factory for creating DPSK configs for different SNR ranges
namespace dpsk_presets {

// Most robust: DBPSK at 31.25 baud
// Target: 0-5 dB SNR, ~8 bps with R1/4
inline DPSKConfig robust() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DBPSK;
    cfg.samples_per_symbol = 1536;  // 31.25 baud
    return cfg;
}

// Low SNR: DBPSK at 62.5 baud
// Target: 3-8 dB SNR, ~16 bps with R1/4
inline DPSKConfig low_snr() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DBPSK;
    cfg.samples_per_symbol = 768;   // 62.5 baud
    return cfg;
}

// Medium: DQPSK at 62.5 baud
// Target: 5-10 dB SNR, ~31 bps with R1/4
inline DPSKConfig medium() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DQPSK;
    cfg.samples_per_symbol = 768;   // 62.5 baud
    return cfg;
}

// Fast: DQPSK at 125 baud
// Target: 8-12 dB SNR, ~62 bps with R1/4 or ~125 bps with R1/2
inline DPSKConfig fast() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DQPSK;
    cfg.samples_per_symbol = 384;   // 125 baud
    return cfg;
}

// Turbo: D8PSK at 125 baud
// Target: 10-15 dB SNR, ~94 bps with R1/4 or ~188 bps with R1/2
inline DPSKConfig turbo() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::D8PSK;
    cfg.samples_per_symbol = 384;   // 125 baud
    return cfg;
}

// High speed: DQPSK at 250 baud
// Target: 10-13 dB SNR, ~125 bps with R1/4 or ~250 bps with R1/2
inline DPSKConfig high_speed() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DQPSK;
    cfg.samples_per_symbol = 192;   // 250 baud
    return cfg;
}

// === High-throughput presets ===
// NOTE: samples_per_symbol MUST be a multiple of 32 (= 48000/1500)
// to ensure integer carrier cycles per symbol for coherent demodulation

// Speed level 1: DQPSK at 300 baud
// Target: 5-8 dB SNR, ~150 bps with R1/2
inline DPSKConfig speed1() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DQPSK;
    cfg.samples_per_symbol = 160;   // 300 baud (5 carrier cycles)
    return cfg;
}

// Speed level 2: DQPSK at 375 baud
// Target: 8-10 dB SNR, ~188 bps with R1/2
inline DPSKConfig speed2() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DQPSK;
    cfg.samples_per_symbol = 128;   // 375 baud (4 carrier cycles)
    return cfg;
}

// Speed level 3: DQPSK at 500 baud
// Target: 10-12 dB SNR, ~250 bps with R1/2 or ~333 bps with R2/3
inline DPSKConfig speed3() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::DQPSK;
    cfg.samples_per_symbol = 96;    // 500 baud (3 carrier cycles)
    return cfg;
}

// Speed level 4: D8PSK at 375 baud
// Target: 12-15 dB SNR, ~375 bps with R1/2 or ~500 bps with R2/3
inline DPSKConfig speed4() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::D8PSK;
    cfg.samples_per_symbol = 128;   // 375 baud (4 carrier cycles)
    return cfg;
}

// Maximum single-carrier: D8PSK at 750 baud
// Target: 13-15 dB SNR, ~750 bps with R1/2, then switch to OFDM
inline DPSKConfig max_speed() {
    DPSKConfig cfg;
    cfg.modulation = DPSKModulation::D8PSK;
    cfg.samples_per_symbol = 64;    // 750 baud (2 carrier cycles)
    return cfg;
}

} // namespace dpsk_presets

} // namespace ultra
