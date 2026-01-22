#pragma once

#include "ultra/types.hpp"
#include <vector>
#include <cmath>
#include <complex>

namespace ultra {

/**
 * BFSK Modulator/Demodulator for weak signal fallback
 *
 * Design targets:
 * - Bandwidth: ~50 Hz (narrow for weak signal operation)
 * - Symbol rate: 31.25 baud (1536 samples/symbol @ 48kHz)
 * - Raw throughput: ~31 bps
 * - With 4x repetition coding: ~8 bps (very robust)
 * - Target: work at -4 dB reported SNR (= +13 dB actual in 50 Hz)
 */

struct FSKConfig {
    float sample_rate = 48000.0f;
    float center_freq = 1500.0f;      // Center frequency (Hz)
    float freq_separation = 50.0f;    // Mark-Space separation (Hz)
    int samples_per_symbol = 1536;    // 31.25 baud @ 48kHz
    int repetition = 4;               // Repeat each bit N times for robustness

    float mark_freq() const { return center_freq + freq_separation / 2; }  // 1525 Hz
    float space_freq() const { return center_freq - freq_separation / 2; } // 1475 Hz
    float symbol_rate() const { return sample_rate / samples_per_symbol; } // 31.25 baud
    float raw_bps() const { return symbol_rate(); }                        // 31.25 bps
    float effective_bps() const { return raw_bps() / repetition; }         // ~8 bps with 4x rep
};

class FSKModulator {
public:
    explicit FSKModulator(const FSKConfig& config = FSKConfig{})
        : config_(config), phase_(0.0f) {}

    // Generate preamble for synchronization (alternating tones)
    // Pattern: 0,1,0,1,0,1... (space, mark, space, mark...)
    Samples generatePreamble(int num_symbols = 16) {
        Samples out;
        out.reserve(num_symbols * config_.samples_per_symbol);

        for (int i = 0; i < num_symbols; i++) {
            bool bit = (i % 2 == 1);  // Alternating 0,1,0,1... (even=0, odd=1)
            auto sym = modulateSymbol(bit);
            out.insert(out.end(), sym.begin(), sym.end());
        }
        return out;
    }

    // Modulate data bytes with repetition coding
    Samples modulate(ByteSpan data) {
        Samples out;
        out.reserve(data.size() * 8 * config_.repetition * config_.samples_per_symbol);

        for (uint8_t byte : data) {
            for (int b = 7; b >= 0; b--) {
                bool bit = (byte >> b) & 1;
                // Repeat each bit for robustness
                for (int r = 0; r < config_.repetition; r++) {
                    auto sym = modulateSymbol(bit);
                    out.insert(out.end(), sym.begin(), sym.end());
                }
            }
        }
        return out;
    }

    // Modulate single bit (for streaming)
    Samples modulateSymbol(bool bit) {
        Samples out(config_.samples_per_symbol);
        float freq = bit ? config_.mark_freq() : config_.space_freq();
        float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

        for (int i = 0; i < config_.samples_per_symbol; i++) {
            out[i] = std::sin(phase_);
            phase_ += phase_inc;
            if (phase_ > 2.0f * M_PI) phase_ -= 2.0f * M_PI;
        }
        return out;
    }

    void reset() { phase_ = 0.0f; }
    const FSKConfig& config() const { return config_; }

private:
    FSKConfig config_;
    float phase_;
};

class FSKDemodulator {
public:
    explicit FSKDemodulator(const FSKConfig& config = FSKConfig{})
        : config_(config) {
        // Pre-compute Goertzel coefficients
        float k_mark = config_.mark_freq() * config_.samples_per_symbol / config_.sample_rate;
        float k_space = config_.space_freq() * config_.samples_per_symbol / config_.sample_rate;
        coeff_mark_ = 2.0f * std::cos(2.0f * M_PI * k_mark / config_.samples_per_symbol);
        coeff_space_ = 2.0f * std::cos(2.0f * M_PI * k_space / config_.samples_per_symbol);
        omega_mark_ = 2.0f * M_PI * k_mark / config_.samples_per_symbol;
        omega_space_ = 2.0f * M_PI * k_space / config_.samples_per_symbol;
    }

    // Find preamble (alternating mark/space pattern)
    //
    // Returns: DATA START position (first sample after preamble), or -1 if not found
    //          This is consistent with DPSK/MFSK findPreamble() interface.
    int findPreamble(SampleSpan samples, int num_preamble_symbols = 16) {
        int symbol_len = config_.samples_per_symbol;
        int preamble_len = num_preamble_symbols * symbol_len;

        if (samples.size() < (size_t)preamble_len) return -1;

        float best_score = 0;
        int best_offset = -1;

        // Search for alternating pattern with fine resolution
        int search_step = symbol_len / 8;  // Fine search
        int max_search = std::min((int)samples.size() - preamble_len, preamble_len * 2);

        for (int offset = 0; offset <= max_search; offset += search_step) {
            float score = 0;
            float total_power = 0;

            for (int i = 0; i < num_preamble_symbols; i++) {
                SampleSpan sym(samples.data() + offset + i * symbol_len, symbol_len);
                float mark_power = goertzelPower(sym, true);
                float space_power = goertzelPower(sym, false);
                total_power += mark_power + space_power;

                // Expect alternating: even=space (0), odd=mark (1)
                if (i % 2 == 0) {
                    score += (space_power - mark_power);
                } else {
                    score += (mark_power - space_power);
                }
            }

            // Normalize by total power
            float norm_score = score / (total_power + 1e-10f);

            if (norm_score > best_score) {
                best_score = norm_score;
                best_offset = offset;
            }
        }

        // Require minimum score
        if (best_score < 0.3f) return -1;

        // Return DATA start position (after preamble), not preamble start
        // This is consistent with DPSK findPreamble() interface
        int data_start = best_offset + preamble_len;
        return data_start;
    }

    // Get preamble length in samples
    int getPreambleLength(int num_preamble_symbols = 16) const {
        return num_preamble_symbols * config_.samples_per_symbol;
    }

    // Demodulate symbols to soft bits (LLR-like values)
    std::vector<float> demodulateToSoft(SampleSpan samples) {
        std::vector<float> soft_bits;
        int symbol_len = config_.samples_per_symbol;

        for (size_t i = 0; i + symbol_len <= samples.size(); i += symbol_len) {
            SampleSpan sym(samples.data() + i, symbol_len);
            float mark_power = goertzelPower(sym, true);
            float space_power = goertzelPower(sym, false);

            // Soft decision: positive = mark (1), negative = space (0)
            float soft = (mark_power - space_power) / (mark_power + space_power + 1e-10f);
            soft_bits.push_back(soft * 5.0f);  // Scale for LLR-like magnitude
        }
        return soft_bits;
    }

    // Demodulate with repetition decoding
    Bytes demodulate(SampleSpan samples) {
        auto soft = demodulateToSoft(samples);

        // Combine repetitions
        std::vector<float> combined;
        for (size_t i = 0; i + config_.repetition <= soft.size(); i += config_.repetition) {
            float sum = 0;
            for (int r = 0; r < config_.repetition; r++) {
                sum += soft[i + r];
            }
            combined.push_back(sum);
        }

        // Hard decision
        Bytes out;
        for (size_t i = 0; i + 8 <= combined.size(); i += 8) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                if (combined[i + b] > 0) {
                    byte |= (1 << (7 - b));
                }
            }
            out.push_back(byte);
        }
        return out;
    }

    const FSKConfig& config() const { return config_; }

private:
    // Goertzel algorithm for single-frequency power detection
    float goertzelPower(SampleSpan samples, bool mark) {
        float coeff = mark ? coeff_mark_ : coeff_space_;
        float omega = mark ? omega_mark_ : omega_space_;

        float s0 = 0, s1 = 0, s2 = 0;
        for (float sample : samples) {
            s0 = sample + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }

        // Compute power
        float real = s1 - s2 * std::cos(omega);
        float imag = s2 * std::sin(omega);
        return real * real + imag * imag;
    }

    FSKConfig config_;
    float coeff_mark_, coeff_space_;
    float omega_mark_, omega_space_;
};

} // namespace ultra
