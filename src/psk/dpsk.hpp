#pragma once

#include "ultra/types.hpp"
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
        float beta = config_.rolloff;

        // Raised cosine window
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

    // Find preamble using matched-filter correlation
    //
    // Matched filter approach:
    // 1. Generate expected preamble waveform (Barker-13 × 3)
    // 2. Cross-correlate received signal with template
    // 3. Find peak of normalized correlation
    // 4. Peak location indicates sync point
    //
    // This is optimal for low SNR detection (matched filter = optimal detector)
    //
    // Returns offset to first sample after preamble (start of data)
    int findPreamble(SampleSpan samples, int /* num_symbols */ = 32) {
        // Generate preamble template if not cached
        if (preamble_template_.empty()) {
            buildPreambleTemplate();
        }

        int preamble_len = preamble_template_.size();
        int symbol_len = config_.samples_per_symbol;

        if ((int)samples.size() < preamble_len + symbol_len) return -1;

        // Detection threshold - normalized correlation must exceed this
        // At 0 dB SNR, expect ~0.5 correlation; at 10 dB expect ~0.9
        constexpr float DETECTION_THRESHOLD = 0.35f;

        // Minimum energy threshold (reject silence)
        constexpr float MIN_ENERGY = 10.0f;

        // Pre-compute template energy
        float template_energy = 0;
        for (float t : preamble_template_) {
            template_energy += t * t;
        }

        int max_search = samples.size() - preamble_len;
        int search_step = symbol_len / 4;  // Coarse search first

        float best_corr = 0;
        int best_offset = -1;

        // Coarse search
        for (int i = 0; i < max_search; i += search_step) {
            // Quick energy check
            float energy = 0;
            for (int j = 0; j < preamble_len; j += 8) {
                energy += samples[i + j] * samples[i + j];
            }
            energy *= 8;  // Approximate total energy

            if (energy < MIN_ENERGY) continue;

            // Cross-correlate with template
            float corr = 0;
            float sig_energy = 0;
            for (int j = 0; j < preamble_len; j++) {
                corr += samples[i + j] * preamble_template_[j];
                sig_energy += samples[i + j] * samples[i + j];
            }

            // Normalize
            float norm = std::sqrt(sig_energy * template_energy);
            if (norm < 1e-10f) continue;
            float norm_corr = std::abs(corr) / norm;

            if (norm_corr > best_corr) {
                best_corr = norm_corr;
                best_offset = i;
            }
        }

        // Fine search around best coarse position
        if (best_offset >= 0 && best_corr > DETECTION_THRESHOLD * 0.8f) {
            int fine_start = std::max(0, best_offset - search_step);
            int fine_end = std::min(max_search, best_offset + search_step);

            for (int i = fine_start; i < fine_end; i++) {
                float corr = 0;
                float sig_energy = 0;
                for (int j = 0; j < preamble_len; j++) {
                    corr += samples[i + j] * preamble_template_[j];
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
        }

        // Check if we found a valid sync
        if (best_corr < DETECTION_THRESHOLD) {
            return -1;
        }

        // Data starts after preamble
        int data_start = best_offset + preamble_len;

        // Set phase reference from last preamble symbol
        int ref_offset = data_start - symbol_len;
        if (ref_offset >= 0 && ref_offset + symbol_len <= (int)samples.size()) {
            SampleSpan ref_sym(samples.data() + ref_offset, symbol_len);
            prev_symbol_ = correlateSymbol(ref_sym);
        }

        return data_start;
    }

private:
    // Build preamble template for matched filter
    void buildPreambleTemplate() {
        // Barker-13 code
        static const int BARKER13[] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
        static const int BARKER_LEN = 13;
        static const int REPEATS = 3;

        int total_symbols = BARKER_LEN * REPEATS;
        preamble_template_.clear();
        preamble_template_.reserve(total_symbols * config_.samples_per_symbol);

        float carrier_inc = 2.0f * M_PI * config_.carrier_freq / config_.sample_rate;
        float phase = 0.0f;
        float symbol_phase = 0.0f;

        for (int rep = 0; rep < REPEATS; rep++) {
            for (int s = 0; s < BARKER_LEN; s++) {
                if (BARKER13[s] < 0) {
                    symbol_phase += M_PI;
                }

                for (int i = 0; i < config_.samples_per_symbol; i++) {
                    preamble_template_.push_back(std::cos(phase + symbol_phase));
                    phase += carrier_inc;
                    if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
                }
            }
        }
    }

    Samples preamble_template_;  // Cached preamble for matched filter

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
    std::vector<float> demodulateSoft(SampleSpan samples) {
        std::vector<float> soft_bits;
        int symbol_len = config_.samples_per_symbol;
        int bits_per_sym = config_.bits_per_symbol();

        int num_symbols = samples.size() / symbol_len;
        soft_bits.reserve(num_symbols * bits_per_sym);

        Complex prev = prev_symbol_;

        for (int s = 0; s < num_symbols; s++) {
            SampleSpan sym(samples.data() + s * symbol_len, symbol_len);
            Complex current = correlateSymbol(sym);

            // Differential decode: diff = current * conj(prev)
            Complex diff = current * std::conj(prev);
            float phase = std::atan2(diff.imag(), diff.real());
            float magnitude = std::abs(diff);

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
