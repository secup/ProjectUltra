// multi_carrier_dpsk.hpp - Multi-Carrier DPSK for mid-SNR range
//
// Fills the gap between single-carrier DPSK (very low SNR) and OFDM (high SNR)
// Based on VARA HF levels 5-10: 3-13 carriers at ~94 baud
//
// Features:
// - Configurable carrier count (3-16)
// - ~94 baud symbol rate per carrier
// - DQPSK modulation (differential, no pilots needed)
// - Chirp sync (works at low SNR)
// - Frequency diversity (survives selective fading)

#pragma once

#include "ultra/types.hpp"
#include "ultra/dsp.hpp"
#include <vector>
#include <complex>
#include <cmath>
#include <random>

namespace ultra {

// Configuration for multi-carrier DPSK
struct MultiCarrierDPSKConfig {
    float sample_rate = 48000.0f;

    // Carrier configuration
    int num_carriers = 8;           // 3-16 carriers
    float freq_low = 500.0f;        // Lowest carrier frequency
    float freq_high = 2500.0f;      // Highest carrier frequency

    // Symbol rate: ~94 baud = 510 samples/symbol at 48kHz
    int samples_per_symbol = 512;   // 93.75 baud

    // Modulation
    int bits_per_symbol = 2;        // 2 = DQPSK, 1 = DBPSK

    // Training
    int training_symbols = 8;       // Training symbols for sync

    // Get carrier frequencies (evenly spaced)
    std::vector<float> getCarrierFreqs() const {
        std::vector<float> freqs(num_carriers);
        if (num_carriers == 1) {
            freqs[0] = (freq_low + freq_high) / 2.0f;
        } else {
            float spacing = (freq_high - freq_low) / (num_carriers - 1);
            for (int i = 0; i < num_carriers; i++) {
                freqs[i] = freq_low + i * spacing;
            }
        }
        return freqs;
    }

    // Get symbol rate in baud
    float getSymbolRate() const {
        return sample_rate / samples_per_symbol;
    }

    // Get raw bit rate (before FEC)
    float getRawBitRate() const {
        return getSymbolRate() * num_carriers * bits_per_symbol;
    }
};

// Multi-Carrier DPSK Modulator
class MultiCarrierDPSKModulator {
public:
    explicit MultiCarrierDPSKModulator(const MultiCarrierDPSKConfig& cfg)
        : config_(cfg)
        , carrier_freqs_(cfg.getCarrierFreqs())
        , carrier_phases_(cfg.num_carriers, 0.0f)
        , prev_symbols_(cfg.num_carriers, Complex(1.0f, 0.0f))
    {
    }

    // Generate training sequence (known pattern for all carriers)
    Samples generateTrainingSequence() {
        Samples output(config_.samples_per_symbol * config_.training_symbols, 0.0f);

        // Training pattern: alternating +1, +j, -1, -j for each carrier
        // This creates orthogonal training across carriers
        for (int sym = 0; sym < config_.training_symbols; sym++) {
            for (int c = 0; c < config_.num_carriers; c++) {
                // Phase rotation for training: carrier index * symbol index * 90°
                float phase_offset = (c * sym) * M_PI / 2.0f;
                Complex training_sym = std::polar(1.0f, phase_offset);

                // Generate this symbol for this carrier
                float freq = carrier_freqs_[c];
                float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

                for (int i = 0; i < config_.samples_per_symbol; i++) {
                    int idx = sym * config_.samples_per_symbol + i;
                    float t = i * phase_inc;  // Start at 0 each symbol
                    Complex carrier = std::polar(1.0f, t);
                    Complex modulated = training_sym * carrier;
                    output[idx] += modulated.real() / config_.num_carriers;
                }
            }
        }

        // Update reference symbols for differential encoding
        for (int c = 0; c < config_.num_carriers; c++) {
            float phase_offset = (c * (config_.training_symbols - 1)) * M_PI / 2.0f;
            prev_symbols_[c] = std::polar(1.0f, phase_offset);
        }

        return output;
    }

    // Generate reference symbol (initial phase reference for all carriers)
    Samples generateReferenceSymbol() {
        Samples output(config_.samples_per_symbol, 0.0f);

        for (int c = 0; c < config_.num_carriers; c++) {
            float freq = carrier_freqs_[c];
            float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

            // Reference symbol is +1 (0° phase) for all carriers
            Complex ref_sym(1.0f, 0.0f);
            prev_symbols_[c] = ref_sym;

            for (int i = 0; i < config_.samples_per_symbol; i++) {
                float t = i * phase_inc;  // Start at 0
                Complex carrier = std::polar(1.0f, t);
                Complex modulated = ref_sym * carrier;
                output[i] += modulated.real() / config_.num_carriers;
            }
        }

        return output;
    }

    // Modulate data bytes
    Samples modulate(const Bytes& data) {
        // Convert bytes to bits
        std::vector<int> bits;
        for (uint8_t byte : data) {
            for (int b = 7; b >= 0; b--) {
                bits.push_back((byte >> b) & 1);
            }
        }

        // Calculate symbols needed
        int bits_per_ofdm_symbol = config_.num_carriers * config_.bits_per_symbol;
        int num_symbols = (bits.size() + bits_per_ofdm_symbol - 1) / bits_per_ofdm_symbol;

        // Pad bits if needed
        bits.resize(num_symbols * bits_per_ofdm_symbol, 0);

        Samples output(num_symbols * config_.samples_per_symbol, 0.0f);

        int bit_idx = 0;
        for (int sym = 0; sym < num_symbols; sym++) {
            for (int c = 0; c < config_.num_carriers; c++) {
                // Get bits for this carrier
                int symbol_bits = 0;
                for (int b = 0; b < config_.bits_per_symbol; b++) {
                    symbol_bits = (symbol_bits << 1) | bits[bit_idx++];
                }

                // DQPSK: map bits to phase change
                float phase_change = 0.0f;
                if (config_.bits_per_symbol == 2) {
                    // DQPSK: 00=+45°, 01=+135°, 11=-135°, 10=-45°
                    static const float dqpsk_phases[] = {
                        M_PI/4, 3*M_PI/4, -3*M_PI/4, -M_PI/4
                    };
                    phase_change = dqpsk_phases[symbol_bits];
                } else {
                    // DBPSK: 0=0°, 1=180°
                    phase_change = symbol_bits ? M_PI : 0.0f;
                }

                // Differential encoding
                Complex diff = std::polar(1.0f, phase_change);
                Complex current = prev_symbols_[c] * diff;
                current /= std::abs(current);  // Normalize
                prev_symbols_[c] = current;

                // Generate carrier with this symbol
                // Important: Each symbol starts at carrier phase 0 for proper
                // differential demodulation. The modulation is in 'current'.
                float freq = carrier_freqs_[c];
                float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

                for (int i = 0; i < config_.samples_per_symbol; i++) {
                    int idx = sym * config_.samples_per_symbol + i;
                    float t = i * phase_inc;  // Start at 0 each symbol
                    Complex carrier = std::polar(1.0f, t);
                    Complex modulated = current * carrier;
                    output[idx] += modulated.real() / config_.num_carriers;
                }
            }
        }

        return output;
    }

    // Reset state
    void reset() {
        carrier_phases_.assign(config_.num_carriers, 0.0f);
        prev_symbols_.assign(config_.num_carriers, Complex(1.0f, 0.0f));
    }

    const MultiCarrierDPSKConfig& getConfig() const { return config_; }

private:
    MultiCarrierDPSKConfig config_;
    std::vector<float> carrier_freqs_;
    std::vector<float> carrier_phases_;
    std::vector<Complex> prev_symbols_;
};

// Multi-Carrier DPSK Demodulator
class MultiCarrierDPSKDemodulator {
public:
    explicit MultiCarrierDPSKDemodulator(const MultiCarrierDPSKConfig& cfg)
        : config_(cfg)
        , carrier_freqs_(cfg.getCarrierFreqs())
        , prev_symbols_(cfg.num_carriers, Complex(1.0f, 0.0f))
        , cfo_hz_(0.0f)
    {
        // Pre-compute mixing oscillators for each carrier
        for (int c = 0; c < config_.num_carriers; c++) {
            carrier_ncos_.emplace_back(carrier_freqs_[c], config_.sample_rate);
        }
    }

    // Process training sequence for CFO estimation and phase sync
    void processTraining(SampleSpan training) {
        if (training.size() < (size_t)(config_.samples_per_symbol * config_.training_symbols)) {
            return;
        }

        // Estimate CFO from training sequence phase progression
        // For now, simple approach: measure phase rotation per symbol
        std::vector<Complex> sym0(config_.num_carriers);
        std::vector<Complex> sym1(config_.num_carriers);

        // Demodulate first and second training symbols
        for (int c = 0; c < config_.num_carriers; c++) {
            sym0[c] = demodulateOneSymbol(training.data(), c);
            sym1[c] = demodulateOneSymbol(training.data() + config_.samples_per_symbol, c);
        }

        // Estimate CFO from phase difference (averaged across carriers)
        float phase_diff_sum = 0.0f;
        for (int c = 0; c < config_.num_carriers; c++) {
            // Expected phase change between sym0 and sym1 in training
            float expected_phase = (c * 1 - c * 0) * M_PI / 2.0f;  // From training pattern
            Complex expected_diff = std::polar(1.0f, expected_phase);

            // Actual phase change
            Complex actual_diff = sym1[c] * std::conj(sym0[c]);

            // Error is the difference
            Complex error = actual_diff * std::conj(expected_diff);
            phase_diff_sum += std::arg(error);
        }

        float avg_phase_error = phase_diff_sum / config_.num_carriers;
        float symbol_duration = config_.samples_per_symbol / config_.sample_rate;
        cfo_hz_ = avg_phase_error / (2.0f * M_PI * symbol_duration);

        // Clamp CFO estimate
        cfo_hz_ = std::max(-50.0f, std::min(50.0f, cfo_hz_));
    }

    // Set reference from reference symbol
    void setReference(SampleSpan ref_symbol) {
        if (ref_symbol.size() < (size_t)config_.samples_per_symbol) {
            return;
        }

        // Demodulate reference symbol for each carrier
        for (int c = 0; c < config_.num_carriers; c++) {
            prev_symbols_[c] = demodulateOneSymbol(ref_symbol.data(), c);
            // Normalize
            if (std::abs(prev_symbols_[c]) > 0.001f) {
                prev_symbols_[c] /= std::abs(prev_symbols_[c]);
            } else {
                prev_symbols_[c] = Complex(1.0f, 0.0f);
            }
        }
    }

    // Demodulate data to soft bits
    std::vector<float> demodulateSoft(SampleSpan data) {
        int num_symbols = data.size() / config_.samples_per_symbol;
        std::vector<float> soft_bits;
        soft_bits.reserve(num_symbols * config_.num_carriers * config_.bits_per_symbol);

        // Track signal power for proper LLR scaling
        float avg_power = 0.0f;
        int power_count = 0;

        for (int sym = 0; sym < num_symbols; sym++) {
            const float* sym_data = data.data() + sym * config_.samples_per_symbol;

            for (int c = 0; c < config_.num_carriers; c++) {
                // Demodulate this carrier
                Complex current = demodulateOneSymbol(sym_data, c);
                float mag = std::abs(current);

                // Track average power
                avg_power += mag * mag;
                power_count++;

                // Normalize for differential decode
                Complex normalized = (mag > 0.0001f) ? current / mag : Complex(1.0f, 0.0f);

                // Differential decode
                Complex diff = normalized * std::conj(prev_symbols_[c]);
                prev_symbols_[c] = normalized;

                // Extract soft bits based on phase
                float phase = std::arg(diff);

                // Confidence based on how "clean" the phase is
                // Scale to produce LLR-like values in reasonable range for LDPC
                // With 8 carriers at 1/8 amplitude each, mag is ~0.125
                // Scale up to get soft bits in [-5, +5] range
                float confidence = mag * config_.num_carriers * 4.0f;

                // Normalize phase to [0, 2π) like the existing DPSK code
                while (phase < 0) phase += 2.0f * M_PI;
                while (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;

                if (config_.bits_per_symbol == 2) {
                    // DQPSK: Phases 45°, 135°, 225°, 315° → bits 00, 01, 10, 11
                    // MSB (bit 0): 0 in upper half (sin>0), 1 in lower half (sin<0)
                    // LSB (bit 1): 0 at 45°/225°, 1 at 135°/315° → sin(2*phase)
                    float sb0 = confidence * std::sin(phase);
                    float sb1 = confidence * std::sin(2.0f * phase);

                    soft_bits.push_back(std::max(-10.0f, std::min(10.0f, sb0)));
                    soft_bits.push_back(std::max(-10.0f, std::min(10.0f, sb1)));
                } else {
                    // DBPSK: 0 → 0°, 1 → 180°
                    float sb = confidence * std::cos(phase);
                    soft_bits.push_back(std::max(-10.0f, std::min(10.0f, sb)));
                }
            }
        }

        return soft_bits;
    }

    // Get estimated CFO
    float getEstimatedCFO() const { return cfo_hz_; }

    // Reset state
    void reset() {
        prev_symbols_.assign(config_.num_carriers, Complex(1.0f, 0.0f));
        cfo_hz_ = 0.0f;
        for (auto& nco : carrier_ncos_) {
            nco.reset();
        }
    }

    const MultiCarrierDPSKConfig& getConfig() const { return config_; }

private:
    // Demodulate one symbol period for one carrier
    Complex demodulateOneSymbol(const float* samples, int carrier_idx) {
        float freq = carrier_freqs_[carrier_idx] + cfo_hz_;
        float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

        // Matched filter: mix down and integrate
        Complex sum(0.0f, 0.0f);
        float phase = 0.0f;

        for (int i = 0; i < config_.samples_per_symbol; i++) {
            Complex mixer = std::polar(1.0f, -phase);
            sum += samples[i] * mixer;
            phase += phase_inc;
        }

        return sum / (float)config_.samples_per_symbol;
    }

    // Simple NCO class
    class NCO {
    public:
        NCO(float freq, float sample_rate)
            : phase_inc_(2.0f * M_PI * freq / sample_rate), phase_(0.0f) {}

        Complex next() {
            Complex out = std::polar(1.0f, phase_);
            phase_ += phase_inc_;
            if (phase_ > 2.0f * M_PI) phase_ -= 2.0f * M_PI;
            return out;
        }

        void reset() { phase_ = 0.0f; }

    private:
        float phase_inc_;
        float phase_;
    };

    MultiCarrierDPSKConfig config_;
    std::vector<float> carrier_freqs_;
    std::vector<Complex> prev_symbols_;
    std::vector<NCO> carrier_ncos_;
    float cfo_hz_;
};

// Preset configurations matching VARA speed levels
namespace mc_dpsk_presets {

// Level 5 equivalent: 3 carriers, ~270 bps raw
inline MultiCarrierDPSKConfig level5() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 3;
    cfg.samples_per_symbol = 512;  // 93.75 baud
    cfg.bits_per_symbol = 2;       // DQPSK
    // Raw: 93.75 * 3 * 2 = 562.5 bps
    // With R1/4: 140 bps, R1/2: 281 bps
    return cfg;
}

// Level 6 equivalent: 4 carriers, ~363 bps raw
inline MultiCarrierDPSKConfig level6() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 4;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    // Raw: 93.75 * 4 * 2 = 750 bps
    return cfg;
}

// Level 7 equivalent: 6 carriers, ~549 bps raw
inline MultiCarrierDPSKConfig level7() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 6;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    // Raw: 93.75 * 6 * 2 = 1125 bps
    return cfg;
}

// Level 8 equivalent: 8 carriers, ~735 bps raw
inline MultiCarrierDPSKConfig level8() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 8;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    // Raw: 93.75 * 8 * 2 = 1500 bps
    return cfg;
}

// Level 9 equivalent: 10 carriers, ~922 bps raw
inline MultiCarrierDPSKConfig level9() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 10;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    // Raw: 93.75 * 10 * 2 = 1875 bps
    return cfg;
}

// Level 10 equivalent: 13 carriers, ~1203 bps raw
inline MultiCarrierDPSKConfig level10() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 13;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    // Raw: 93.75 * 13 * 2 = 2437.5 bps
    return cfg;
}

} // namespace mc_dpsk_presets

} // namespace ultra
