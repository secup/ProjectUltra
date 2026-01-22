#pragma once

#include "ultra/types.hpp"
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>

namespace ultra {

/**
 * Adaptive MFSK Modulator/Demodulator
 *
 * Supports 2/4/8/16/32 tones for adaptive throughput vs robustness
 * Designed to fill the SNR gap where OFDM fails (-12 dB to +5 dB reported)
 *
 * With 50 Hz tone spacing in 2.8 kHz bandwidth:
 * - 2FSK:  1 bit/sym,  ~31 bps (most robust, -12 dB)
 * - 4FSK:  2 bits/sym, ~62 bps (-8 dB)
 * - 8FSK:  3 bits/sym, ~94 bps (-4 dB)
 * - 16FSK: 4 bits/sym, ~125 bps (0 dB)
 * - 32FSK: 5 bits/sym, ~156 bps (+3 dB, then switch to OFDM)
 */

struct MFSKConfig {
    float sample_rate = 48000.0f;
    float center_freq = 1500.0f;       // Center of tone group
    float tone_spacing = 50.0f;        // Hz between adjacent tones
    int num_tones = 8;                 // 2, 4, 8, 16, or 32
    int samples_per_symbol = 1536;     // ~31.25 baud @ 48kHz
    int repetition = 2;                // Repeat symbols for robustness

    int bits_per_symbol() const {
        return (int)std::log2(num_tones);
    }

    float symbol_rate() const {
        return sample_rate / samples_per_symbol;
    }

    float raw_bps() const {
        return symbol_rate() * bits_per_symbol();
    }

    float effective_bps() const {
        return raw_bps() / repetition;
    }

    // Get frequency for tone index (centered around center_freq)
    float tone_freq(int tone_idx) const {
        float offset = (tone_idx - (num_tones - 1) / 2.0f) * tone_spacing;
        return center_freq + offset;
    }

    // Total bandwidth used
    float bandwidth() const {
        return (num_tones - 1) * tone_spacing + tone_spacing; // Include guard
    }
};

class MFSKModulator {
public:
    explicit MFSKModulator(const MFSKConfig& config = MFSKConfig{})
        : config_(config), phase_(0.0f) {}

    // Generate preamble: cycle through all tones for sync and tone calibration
    Samples generatePreamble(int cycles = 2) {
        Samples out;
        int total_symbols = cycles * config_.num_tones;
        out.reserve(total_symbols * config_.samples_per_symbol);

        for (int c = 0; c < cycles; c++) {
            for (int t = 0; t < config_.num_tones; t++) {
                auto sym = modulateTone(t);
                out.insert(out.end(), sym.begin(), sym.end());
            }
        }
        return out;
    }

    // Modulate data bytes
    Samples modulate(ByteSpan data) {
        Samples out;
        int bits_per_sym = config_.bits_per_symbol();
        int total_bits = data.size() * 8;
        int total_symbols = (total_bits + bits_per_sym - 1) / bits_per_sym;
        out.reserve(total_symbols * config_.repetition * config_.samples_per_symbol);

        // Convert bytes to bit stream
        std::vector<bool> bits;
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
            // Convert bits to tone index
            int tone_idx = 0;
            for (int b = 0; b < bits_per_sym; b++) {
                if (bits[i + b]) {
                    tone_idx |= (1 << (bits_per_sym - 1 - b));
                }
            }

            // Repeat for robustness
            for (int r = 0; r < config_.repetition; r++) {
                auto sym = modulateTone(tone_idx);
                out.insert(out.end(), sym.begin(), sym.end());
            }
        }

        return out;
    }

    // Modulate single tone
    Samples modulateTone(int tone_idx) {
        Samples out(config_.samples_per_symbol);
        float freq = config_.tone_freq(tone_idx);
        float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

        for (int i = 0; i < config_.samples_per_symbol; i++) {
            out[i] = std::sin(phase_);
            phase_ += phase_inc;
            if (phase_ > 2.0f * M_PI) phase_ -= 2.0f * M_PI;
        }
        return out;
    }

    void reset() { phase_ = 0.0f; }
    const MFSKConfig& config() const { return config_; }

private:
    MFSKConfig config_;
    float phase_;
};

class MFSKDemodulator {
public:
    explicit MFSKDemodulator(const MFSKConfig& config = MFSKConfig{})
        : config_(config) {
        // Pre-compute Goertzel coefficients for all tones
        coeffs_.resize(config_.num_tones);
        omegas_.resize(config_.num_tones);

        for (int t = 0; t < config_.num_tones; t++) {
            float freq = config_.tone_freq(t);
            float k = freq * config_.samples_per_symbol / config_.sample_rate;
            coeffs_[t] = 2.0f * std::cos(2.0f * M_PI * k / config_.samples_per_symbol);
            omegas_[t] = 2.0f * M_PI * k / config_.samples_per_symbol;
        }
    }

    // Find preamble (tone sweep pattern) - optimized for low SNR
    //
    // Returns: DATA START position (first sample after preamble), or -1 if not found
    //          This is consistent with DPSK/BFSK findPreamble() interface.
    //
    // Uses two-stage detection:
    // 1. Coarse: Find regions with narrowband energy (any tone activity)
    // 2. Fine: Verify tone sweep pattern matches expected sequence
    //
    // At very low SNR (-17 dB reported = 0 dB actual), we need:
    // - Lower thresholds
    // - More integration (longer preamble)
    // - Robust energy detection
    int findPreamble(SampleSpan samples, int cycles = 2) {
        int symbol_len = config_.samples_per_symbol;
        int preamble_len = cycles * config_.num_tones * symbol_len;

        if (samples.size() < (size_t)preamble_len) return -1;

        float best_score = 0;
        int best_offset = -1;

        int search_step = symbol_len / 4;
        int max_search = std::min((int)samples.size() - preamble_len, preamble_len * 2);

        // Very low energy threshold (for -17 dB operation)
        // Signal power is ~0.02 at 0 dB actual SNR in narrowband
        constexpr float MIN_ENERGY = 1.0f;

        // Lower dominance ratio for low SNR (noise raises baseline)
        constexpr float MIN_DOMINANCE_RATIO = 0.2f;

        // Stage 1: Coarse search - find regions with narrowband activity
        std::vector<int> candidate_offsets;
        for (int offset = 0; offset <= max_search; offset += search_step) {
            // Check first symbol for any tone activity
            SampleSpan first_sym(samples.data() + offset, symbol_len);
            auto powers = getTonePowers(first_sym);

            float max_power = 0;
            float total_power = 0;
            for (int i = 0; i < config_.num_tones; i++) {
                total_power += powers[i];
                if (powers[i] > max_power) max_power = powers[i];
            }

            // Check if there's significant narrowband energy
            if (total_power > MIN_ENERGY && max_power / (total_power + 1e-10f) > MIN_DOMINANCE_RATIO) {
                candidate_offsets.push_back(offset);
            }
        }

        // Stage 2: Fine search - verify tone sequence at candidates
        for (int offset : candidate_offsets) {
            float score = 0;
            int valid_symbols = 0;

            for (int c = 0; c < cycles; c++) {
                for (int t = 0; t < config_.num_tones; t++) {
                    int sym_offset = offset + (c * config_.num_tones + t) * symbol_len;
                    if (sym_offset + symbol_len > (int)samples.size()) continue;

                    SampleSpan sym(samples.data() + sym_offset, symbol_len);

                    // Get all tone powers
                    auto powers = getTonePowers(sym);

                    // Find max tone
                    float max_power = 0;
                    int max_idx = 0;
                    float total_power = 0;
                    for (int i = 0; i < config_.num_tones; i++) {
                        total_power += powers[i];
                        if (powers[i] > max_power) {
                            max_power = powers[i];
                            max_idx = i;
                        }
                    }

                    // At low SNR, we may not always detect the right tone
                    // but we should detect SOMETHING narrowband
                    if (total_power > MIN_ENERGY * 0.5f) {
                        valid_symbols++;

                        // Score based on how close detected tone is to expected
                        // Perfect match = 1.0, adjacent = 0.5, farther = 0
                        int tone_error = std::abs(max_idx - t);
                        if (tone_error == 0) score += 1.0f;
                        else if (tone_error == 1) score += 0.5f;
                        else if (tone_error == 2) score += 0.25f;
                    }
                }
            }

            // Require at least 30% valid symbols for low SNR operation
            int total_symbols = cycles * config_.num_tones;
            if (valid_symbols < total_symbols * 3 / 10) continue;

            score /= total_symbols;

            if (score > best_score) {
                best_score = score;
                best_offset = offset;
            }
        }

        // Lower threshold for low SNR (60% instead of 80%)
        // False positives are handled by LDPC decode failure
        if (best_score < 0.6f) return -1;

        // No CFO compensation needed - MFSK max-detection is inherently CFO-tolerant
        // All tones shift by the same amount, so relative ranking is preserved

        // Return DATA start position (after preamble), not preamble start
        // This is consistent with DPSK findPreamble() interface
        int data_start = best_offset + preamble_len;
        return data_start;
    }

    // Get preamble length in samples (useful for callers that need it)
    int getPreambleLength(int cycles = 2) const {
        return cycles * config_.num_tones * config_.samples_per_symbol;
    }

    // CFO estimation not needed - MFSK max-detection is inherently CFO-tolerant
    float getEstimatedCFO() const { return 0.0f; }

    // Detect which tone is present in a symbol
    int detectTone(SampleSpan symbol) {
        float max_power = -1;
        int max_tone = 0;

        for (int t = 0; t < config_.num_tones; t++) {
            float power = goertzelPower(symbol, t);
            if (power > max_power) {
                max_power = power;
                max_tone = t;
            }
        }

        return max_tone;
    }

    // Get soft decisions for all tones (for soft decoding)
    std::vector<float> getTonePowers(SampleSpan symbol) {
        std::vector<float> powers(config_.num_tones);
        for (int t = 0; t < config_.num_tones; t++) {
            powers[t] = goertzelPower(symbol, t);
        }
        return powers;
    }

    // Demodulate to soft bits (LLRs) for LDPC decoding
    std::vector<float> demodulateSoft(SampleSpan samples) {
        std::vector<float> soft_bits;
        int symbol_len = config_.samples_per_symbol;
        int bits_per_sym = config_.bits_per_symbol();

        for (size_t i = 0; i + config_.repetition * symbol_len <= samples.size();
             i += config_.repetition * symbol_len) {

            // Combine power across repetitions
            std::vector<float> combined_power(config_.num_tones, 0.0f);
            for (int r = 0; r < config_.repetition; r++) {
                SampleSpan sym(samples.data() + i + r * symbol_len, symbol_len);
                auto powers = getTonePowers(sym);
                for (int t = 0; t < config_.num_tones; t++) {
                    combined_power[t] += powers[t];
                }
            }

            // Convert tone powers to LLRs for each bit position
            auto llrs = tonePowersToLLR(combined_power, bits_per_sym);
            soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
        }

        return soft_bits;
    }

    // Demodulate with repetition combining
    Bytes demodulate(SampleSpan samples) {
        int symbol_len = config_.samples_per_symbol;
        int bits_per_sym = config_.bits_per_symbol();

        // Detect tones with repetition combining
        std::vector<int> detected_tones;

        for (size_t i = 0; i + config_.repetition * symbol_len <= samples.size();
             i += config_.repetition * symbol_len) {

            // Combine power across repetitions
            std::vector<float> combined_power(config_.num_tones, 0.0f);

            for (int r = 0; r < config_.repetition; r++) {
                SampleSpan sym(samples.data() + i + r * symbol_len, symbol_len);
                auto powers = getTonePowers(sym);
                for (int t = 0; t < config_.num_tones; t++) {
                    combined_power[t] += powers[t];
                }
            }

            // Find max
            int best_tone = 0;
            float best_power = combined_power[0];
            for (int t = 1; t < config_.num_tones; t++) {
                if (combined_power[t] > best_power) {
                    best_power = combined_power[t];
                    best_tone = t;
                }
            }

            detected_tones.push_back(best_tone);
        }

        // Convert tones to bits
        std::vector<bool> bits;
        for (int tone : detected_tones) {
            for (int b = bits_per_sym - 1; b >= 0; b--) {
                bits.push_back((tone >> b) & 1);
            }
        }

        // Convert bits to bytes
        Bytes out;
        for (size_t i = 0; i + 8 <= bits.size(); i += 8) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                if (bits[i + b]) {
                    byte |= (1 << (7 - b));
                }
            }
            out.push_back(byte);
        }

        return out;
    }

    const MFSKConfig& config() const { return config_; }

    // Reset demodulator state (for API consistency with other demodulators)
    void reset() {
        estimated_cfo_ = 0;
        // Restore original Goertzel coefficients
        for (int t = 0; t < config_.num_tones; t++) {
            float freq = config_.tone_freq(t);
            float k = freq * config_.samples_per_symbol / config_.sample_rate;
            coeffs_[t] = 2.0f * std::cos(2.0f * M_PI * k / config_.samples_per_symbol);
            omegas_[t] = 2.0f * M_PI * k / config_.samples_per_symbol;
        }
    }

private:
    // Estimate CFO from preamble tone sweep
    // Since we know the expected tone sequence (0, 1, 2, ... num_tones-1, repeated),
    // we can measure the actual frequency of each detected tone and compare
    float estimateCFO(SampleSpan samples, int preamble_offset, int symbol_len, int cycles) {
        // Search for each tone with sub-bin resolution
        // Using parabolic interpolation on Goertzel powers
        float total_error_hz = 0;
        int count = 0;

        for (int c = 0; c < cycles; c++) {
            for (int t = 0; t < config_.num_tones; t++) {
                int sym_offset = preamble_offset + (c * config_.num_tones + t) * symbol_len;
                if (sym_offset + symbol_len > (int)samples.size()) continue;

                SampleSpan sym(samples.data() + sym_offset, symbol_len);

                // Get powers for expected tone and neighbors
                float expected_freq = config_.tone_freq(t);

                // Measure power at 3 frequencies: expected, expected±(spacing/2)
                float power_center = goertzelPowerAtFreq(sym, expected_freq);
                float power_low = goertzelPowerAtFreq(sym, expected_freq - config_.tone_spacing * 0.5f);
                float power_high = goertzelPowerAtFreq(sym, expected_freq + config_.tone_spacing * 0.5f);

                // Only use measurements with significant energy
                float max_power = std::max({power_center, power_low, power_high});
                if (max_power < 0.01f) continue;

                // Parabolic interpolation to find peak
                // If all three powers are valid, interpolate
                if (power_center > 0 && power_low > 0 && power_high > 0) {
                    // Convert to dB for better interpolation
                    float db_low = std::log(power_low);
                    float db_center = std::log(power_center);
                    float db_high = std::log(power_high);

                    // Parabolic peak location: x = 0.5 * (db_low - db_high) / (db_low - 2*db_center + db_high)
                    float denom = db_low - 2.0f * db_center + db_high;
                    if (std::abs(denom) > 0.001f) {
                        float x = 0.5f * (db_low - db_high) / denom;  // x in [-1, 1]
                        float freq_error = x * (config_.tone_spacing * 0.5f);

                        // Sanity check: error should be less than half spacing
                        if (std::abs(freq_error) < config_.tone_spacing * 0.5f) {
                            total_error_hz += freq_error;
                            count++;
                        }
                    }
                }
            }
        }

        if (count < 3) return 0;  // Not enough measurements
        return total_error_hz / count;
    }

    // Compute Goertzel power at arbitrary frequency
    float goertzelPowerAtFreq(SampleSpan samples, float freq) {
        float k = freq * samples.size() / config_.sample_rate;
        float coeff = 2.0f * std::cos(2.0f * M_PI * k / samples.size());
        float omega = 2.0f * M_PI * k / samples.size();

        float s0 = 0, s1 = 0, s2 = 0;
        for (float sample : samples) {
            s0 = sample + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        float real = s1 - s2 * std::cos(omega);
        float imag = s2 * std::sin(omega);
        return real * real + imag * imag;
    }

    // Update Goertzel coefficients with CFO compensation
    void updateGoertzelForCFO(float cfo_hz) {
        for (int t = 0; t < config_.num_tones; t++) {
            float freq = config_.tone_freq(t) + cfo_hz;  // Compensate by adding CFO to expected freq
            float k = freq * config_.samples_per_symbol / config_.sample_rate;
            coeffs_[t] = 2.0f * std::cos(2.0f * M_PI * k / config_.samples_per_symbol);
            omegas_[t] = 2.0f * M_PI * k / config_.samples_per_symbol;
        }
    }

    float estimated_cfo_ = 0.0f;
    // Convert tone powers to LLRs for LDPC soft decoding
    std::vector<float> tonePowersToLLR(const std::vector<float>& powers, int bits) {
        std::vector<float> llrs(bits);

        for (int b = 0; b < bits; b++) {
            float p0 = 0, p1 = 0;
            int mask = 1 << (bits - 1 - b);

            for (int t = 0; t < (int)powers.size(); t++) {
                if (t & mask) {
                    p1 += powers[t];
                } else {
                    p0 += powers[t];
                }
            }

            // Compute LLR with small epsilon to avoid log(0)
            // Positive LLR = bit 0 more likely, Negative LLR = bit 1 more likely
            float llr = std::log((p0 + 1e-10f) / (p1 + 1e-10f));
            // Clamp to reasonable range
            llrs[b] = std::clamp(llr, -10.0f, 10.0f);
        }

        return llrs;
    }

    float goertzelPower(SampleSpan samples, int tone_idx) {
        float coeff = coeffs_[tone_idx];
        float omega = omegas_[tone_idx];

        float s0 = 0, s1 = 0, s2 = 0;
        for (float sample : samples) {
            s0 = sample + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        float real = s1 - s2 * std::cos(omega);
        float imag = s2 * std::sin(omega);
        return real * real + imag * imag;
    }

    MFSKConfig config_;
    std::vector<float> coeffs_;
    std::vector<float> omegas_;
};

// Factory for creating MFSK configs for different SNR ranges
namespace mfsk_presets {

inline MFSKConfig robust() {  // -12 dB reported, ~30 bps
    MFSKConfig cfg;
    cfg.num_tones = 2;
    cfg.repetition = 4;
    return cfg;
}

inline MFSKConfig low_snr() {  // -8 dB reported, ~45 bps
    MFSKConfig cfg;
    cfg.num_tones = 4;
    cfg.repetition = 3;
    return cfg;
}

inline MFSKConfig medium() {  // -4 dB reported, ~62 bps
    MFSKConfig cfg;
    cfg.num_tones = 8;
    cfg.repetition = 2;
    return cfg;
}

inline MFSKConfig fast() {  // 0 dB reported, ~94 bps
    MFSKConfig cfg;
    cfg.num_tones = 16;
    cfg.repetition = 2;
    return cfg;
}

inline MFSKConfig turbo() {  // +3 dB reported, ~156 bps (before switching to OFDM)
    MFSKConfig cfg;
    cfg.num_tones = 32;
    cfg.repetition = 1;
    return cfg;
}

} // namespace mfsk_presets

} // namespace ultra
