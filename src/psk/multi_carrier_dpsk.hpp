// multi_carrier_dpsk.hpp - Multi-Carrier DPSK for mid-SNR range
//
// Fills the gap between single-carrier DPSK (very low SNR) and OFDM (high SNR)
// Based on commercial HF modem levels 5-10: 3-13 carriers at ~94 baud
//
// Features:
// - Configurable carrier count (3-16)
// - ~94 baud symbol rate per carrier
// - DQPSK modulation (differential, no pilots needed)
// - Integrated chirp sync (follows OFDM demodulator pattern)
// - Frequency diversity (survives selective fading)

#pragma once

#include "ultra/types.hpp"
#include "ultra/dsp.hpp"
#include "sync/chirp_sync.hpp"
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

    // Chirp sync config
    float chirp_f_start = 300.0f;
    float chirp_f_end = 2700.0f;
    float chirp_duration_ms = 500.0f;  // 500ms each for up/down chirps
    bool use_dual_chirp = true;        // Up+down chirp for CFO estimation
    float chirp_threshold = 0.15f;     // Detection threshold (lower for dual chirp)

    // TX CFO for simulation (simulates radio tuning error)
    float tx_cfo_hz = 0.0f;

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

    // Get chirp config for sync
    sync::ChirpConfig getChirpConfig() const {
        sync::ChirpConfig cfg;
        cfg.sample_rate = sample_rate;
        cfg.f_start = chirp_f_start;
        cfg.f_end = chirp_f_end;
        cfg.duration_ms = chirp_duration_ms;
        cfg.gap_ms = 100.0f;  // Gap between up and down chirps
        cfg.use_dual_chirp = use_dual_chirp;  // Use configured value
        cfg.tx_cfo_hz = tx_cfo_hz;  // Pass TX CFO for simulation
        return cfg;
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
        , chirp_sync_(cfg.getChirpConfig())
    {
    }

    // Generate complete preamble: chirp + training + reference
    Samples generatePreamble() {
        Samples chirp = chirp_sync_.generate();
        Samples training = generateTrainingSequence();
        Samples ref = generateReferenceSymbol();

        Samples preamble;
        preamble.reserve(chirp.size() + training.size() + ref.size());
        preamble.insert(preamble.end(), chirp.begin(), chirp.end());
        preamble.insert(preamble.end(), training.begin(), training.end());
        preamble.insert(preamble.end(), ref.begin(), ref.end());
        return preamble;
    }

    // Generate training sequence (known pattern for all carriers)
    Samples generateTrainingSequence() {
        Samples output(config_.samples_per_symbol * config_.training_symbols, 0.0f);

        // Training pattern: alternating +1, +j, -1, -j for each carrier
        // This creates orthogonal training across carriers
        for (int sym = 0; sym < config_.training_symbols; sym++) {
            for (int c = 0; c < config_.num_carriers; c++) {
                // Phase rotation for training: carrier index * symbol index * 90deg
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

            // Reference symbol is +1 (0 deg phase) for all carriers
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
                    // DQPSK: 00=+45deg, 01=+135deg, 11=-135deg, 10=-45deg
                    static const float dqpsk_phases[] = {
                        M_PI/4, 3*M_PI/4, -3*M_PI/4, -M_PI/4
                    };
                    phase_change = dqpsk_phases[symbol_bits];
                } else {
                    // DBPSK: 0=0deg, 1=180deg
                    phase_change = symbol_bits ? M_PI : 0.0f;
                }

                // Differential encoding
                Complex diff = std::polar(1.0f, phase_change);
                Complex current = prev_symbols_[c] * diff;
                current /= std::abs(current);  // Normalize
                prev_symbols_[c] = current;

                // Generate carrier with this symbol
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
    const sync::ChirpSync& getChirpSync() const { return chirp_sync_; }

private:
    MultiCarrierDPSKConfig config_;
    std::vector<float> carrier_freqs_;
    std::vector<float> carrier_phases_;
    std::vector<Complex> prev_symbols_;
    sync::ChirpSync chirp_sync_;
};

// Multi-Carrier DPSK Demodulator
// Follows OFDM demodulator pattern: process() feeds samples, returns true when frame ready
class MultiCarrierDPSKDemodulator {
public:
    enum class State {
        IDLE,           // Looking for chirp preamble
        GOT_CHIRP,      // Chirp found, waiting for training + ref + data
        FRAME_READY     // Frame demodulated, soft bits available
    };

    explicit MultiCarrierDPSKDemodulator(const MultiCarrierDPSKConfig& cfg)
        : config_(cfg)
        , carrier_freqs_(cfg.getCarrierFreqs())
        , prev_symbols_(cfg.num_carriers, Complex(1.0f, 0.0f))
        , chirp_sync_(cfg.getChirpConfig())
        , state_(State::IDLE)
        , cfo_hz_(0.0f)
        , chirp_position_(-1)
        , last_chirp_corr_(0.0f)
    {
        // Calculate expected sizes
        chirp_samples_ = chirp_sync_.getTotalSamples();
        training_samples_ = cfg.training_symbols * cfg.samples_per_symbol;
        ref_samples_ = cfg.samples_per_symbol;
        preamble_samples_ = chirp_samples_ + training_samples_ + ref_samples_;
    }

    // Process incoming samples (like OFDM demodulator)
    // Returns true when a frame is ready (soft bits available)
    bool process(SampleSpan samples) {
        // Append to internal buffer
        sample_buffer_.insert(sample_buffer_.end(), samples.begin(), samples.end());

        // State machine
        switch (state_) {
            case State::IDLE:
                return processIdle();

            case State::GOT_CHIRP:
                return processGotChirp();

            case State::FRAME_READY:
                // Already have a frame ready, don't process more until getSoftBits() called
                return true;
        }
        return false;
    }

    // Get soft bits from demodulated frame
    std::vector<float> getSoftBits() {
        auto result = std::move(soft_bits_);
        soft_bits_.clear();
        if (state_ == State::FRAME_READY) {
            state_ = State::IDLE;
        }
        return result;
    }

    // Check if synchronized (found chirp)
    bool isSynced() const {
        return state_ == State::GOT_CHIRP || state_ == State::FRAME_READY;
    }

    // Check if frame is ready
    bool isFrameReady() const {
        return state_ == State::FRAME_READY;
    }

    // Check if has pending data to process
    bool hasPendingData() const {
        return !sample_buffer_.empty() || state_ != State::IDLE;
    }

    // Get estimated CFO
    float getEstimatedCFO() const { return cfo_hz_; }

    // Set CFO (from external estimation like dual chirp)
    void setCFO(float cfo_hz) { cfo_hz_ = cfo_hz; }

    // Apply CFO correction to external samples (public wrapper for applyCFOCorrection)
    // Call this on samples BEFORE demodulation when using direct demodulateSoft() path
    // Note: This preserves cfo_hz_ so it can be called multiple times on different spans
    void applyCFO(Samples& samples) {
        if (std::abs(cfo_hz_) > 0.1f) {
            float saved_cfo = cfo_hz_;
            applyCFOCorrection(samples, cfo_hz_);
            cfo_hz_ = saved_cfo;  // Restore for subsequent spans
        }
    }

    // Set chirp detected externally (bypass internal chirp detection)
    // Call this when using external detectSync() and passing training+ref+data samples
    // (not data-only - the demodulator needs training and ref for CFO refinement and reference)
    void setChirpDetected(float cfo_hz = 0.0f) {
        cfo_hz_ = cfo_hz;
        state_ = State::GOT_CHIRP;
        external_chirp_detected_ = true;  // Flag to adjust offsets in processGotChirp
        sample_buffer_.clear();  // Clear any stale samples
    }

    // Get last chirp correlation value
    float getLastChirpCorrelation() const { return last_chirp_corr_; }

    // Set expected data size in bytes (to know when frame is complete)
    void setExpectedDataBytes(size_t bytes) {
        expected_data_bytes_ = bytes;
    }

    // Reset state
    void reset() {
        state_ = State::IDLE;
        sample_buffer_.clear();
        soft_bits_.clear();
        prev_symbols_.assign(config_.num_carriers, Complex(1.0f, 0.0f));
        cfo_hz_ = 0.0f;
        external_chirp_detected_ = false;
        chirp_position_ = -1;
        last_chirp_corr_ = 0.0f;
        expected_data_bytes_ = 0;
    }

    const MultiCarrierDPSKConfig& getConfig() const { return config_; }

    // --- Legacy API for backward compatibility ---
    // These are used by existing code that manually handles chirp detection

    void processTraining(SampleSpan training) {
        if (training.size() < training_samples_) return;

        // Estimate RESIDUAL CFO from training sequence phase progression
        // This measures what's LEFT after any pre-set CFO (from dual chirp) is applied
        // Note: demodulateOneSymbol uses current cfo_hz_, so if dual chirp pre-set it,
        // this will estimate the residual (should be near zero if dual chirp is accurate)
        std::vector<Complex> sym0(config_.num_carriers);
        std::vector<Complex> sym1(config_.num_carriers);

        for (int c = 0; c < config_.num_carriers; c++) {
            sym0[c] = demodulateOneSymbol(training.data(), c);
            sym1[c] = demodulateOneSymbol(training.data() + config_.samples_per_symbol, c);
        }

        float phase_diff_sum = 0.0f;
        for (int c = 0; c < config_.num_carriers; c++) {
            float expected_phase = (c * 1 - c * 0) * M_PI / 2.0f;
            Complex expected_diff = std::polar(1.0f, expected_phase);
            Complex actual_diff = sym1[c] * std::conj(sym0[c]);
            Complex error = actual_diff * std::conj(expected_diff);
            phase_diff_sum += std::arg(error);
        }

        float avg_phase_error = phase_diff_sum / config_.num_carriers;
        float symbol_duration = config_.samples_per_symbol / config_.sample_rate;
        float residual_cfo = avg_phase_error / (2.0f * M_PI * symbol_duration);

        // ADD residual to existing CFO (don't replace)
        // This allows dual chirp to pre-set rough CFO, training refines it
        cfo_hz_ += residual_cfo;
        cfo_hz_ = std::max(-50.0f, std::min(50.0f, cfo_hz_));
    }

    void setReference(SampleSpan ref_symbol) {
        if (ref_symbol.size() < (size_t)config_.samples_per_symbol) return;

        for (int c = 0; c < config_.num_carriers; c++) {
            prev_symbols_[c] = demodulateOneSymbol(ref_symbol.data(), c);
            if (std::abs(prev_symbols_[c]) > 0.001f) {
                prev_symbols_[c] /= std::abs(prev_symbols_[c]);
            } else {
                prev_symbols_[c] = Complex(1.0f, 0.0f);
            }
        }
    }

    std::vector<float> demodulateSoft(SampleSpan data) {
        int num_symbols = data.size() / config_.samples_per_symbol;
        std::vector<float> soft_bits;
        soft_bits.reserve(num_symbols * config_.num_carriers * config_.bits_per_symbol);

        for (int sym = 0; sym < num_symbols; sym++) {
            const float* sym_data = data.data() + sym * config_.samples_per_symbol;

            for (int c = 0; c < config_.num_carriers; c++) {
                Complex current = demodulateOneSymbol(sym_data, c);
                float mag = std::abs(current);

                Complex normalized = (mag > 0.0001f) ? current / mag : Complex(1.0f, 0.0f);
                Complex diff = normalized * std::conj(prev_symbols_[c]);
                prev_symbols_[c] = normalized;

                float phase = std::arg(diff);
                float confidence = mag * config_.num_carriers * 4.0f;

                while (phase < 0) phase += 2.0f * M_PI;
                while (phase >= 2.0f * M_PI) phase -= 2.0f * M_PI;

                if (config_.bits_per_symbol == 2) {
                    float sb0 = confidence * std::sin(phase);
                    float sb1 = confidence * std::sin(2.0f * phase);
                    soft_bits.push_back(std::max(-10.0f, std::min(10.0f, sb0)));
                    soft_bits.push_back(std::max(-10.0f, std::min(10.0f, sb1)));
                } else {
                    float sb = confidence * std::cos(phase);
                    soft_bits.push_back(std::max(-10.0f, std::min(10.0f, sb)));
                }
            }
        }

        return soft_bits;
    }

private:
    // Process in IDLE state - look for chirp
    bool processIdle() {
        // Need enough samples to search for chirp
        // In connected mode, frames arrive predictably so we use a smaller window
        // chirp (24000) + training (4096) + ref (512) + some data margin
        size_t min_samples = chirp_samples_ + training_samples_ + ref_samples_ + 4000;
        if (sample_buffer_.size() < min_samples) {
            // Trim buffer if too large (keep 4x min_samples as search window)
            if (sample_buffer_.size() > 4 * min_samples) {
                size_t trim = sample_buffer_.size() - 2 * min_samples;
                sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + trim);
            }
            return false;
        }

        // Search for chirp using dual chirp detection for CFO estimation
        SampleSpan search_span(sample_buffer_.data(), sample_buffer_.size());
        auto chirp_result = chirp_sync_.detectDualChirp(search_span, config_.chirp_threshold);
        int chirp_start = chirp_result.success ? chirp_result.up_chirp_start : -1;
        float corr = std::max(chirp_result.up_correlation, chirp_result.down_correlation);

        if (chirp_start >= 0) {
            // Check if there's signal energy after chirp (not just PING)
            size_t chirp_end = chirp_start + chirp_samples_;
            if (chirp_end + training_samples_ + ref_samples_ + 1000 < sample_buffer_.size()) {
                float energy = 0.0f;
                for (size_t i = chirp_end; i < chirp_end + training_samples_; i++) {
                    energy += sample_buffer_[i] * sample_buffer_[i];
                }
                float rms = std::sqrt(energy / training_samples_);

                if (rms > 0.05f) {
                    // Found DPSK frame - save CFO estimate from dual chirp
                    chirp_position_ = chirp_start;
                    last_chirp_corr_ = corr;
                    cfo_hz_ = chirp_result.cfo_hz;  // Use dual chirp CFO estimate
                    state_ = State::GOT_CHIRP;

                    // Remove samples before chirp
                    if (chirp_start > 0) {
                        sample_buffer_.erase(sample_buffer_.begin(),
                                            sample_buffer_.begin() + chirp_start);
                        chirp_position_ = 0;
                    }
                    return processGotChirp();
                }
            }
        }

        // No chirp found - trim old samples
        if (sample_buffer_.size() > min_samples) {
            size_t trim = sample_buffer_.size() - min_samples / 2;
            sample_buffer_.erase(sample_buffer_.begin(), sample_buffer_.begin() + trim);
        }
        return false;
    }

    // Process in GOT_CHIRP state - wait for complete frame
    bool processGotChirp() {
        // When external chirp detection is used, buffer contains training+ref+data
        // (chirp is NOT in buffer). Otherwise buffer contains chirp+training+ref+data.
        size_t chirp_offset = external_chirp_detected_ ? 0 : chirp_samples_;
        size_t local_preamble = training_samples_ + ref_samples_;  // Training + ref (no chirp)
        size_t full_preamble = chirp_offset + local_preamble;

        // Calculate how many samples we need
        size_t data_samples = 0;
        if (expected_data_bytes_ > 0) {
            int bits_per_symbol = config_.num_carriers * config_.bits_per_symbol;
            int num_symbols = (expected_data_bytes_ * 8 + bits_per_symbol - 1) / bits_per_symbol;
            data_samples = num_symbols * config_.samples_per_symbol;
        } else {
            // Default: demodulate all remaining samples after preamble
            // The header decoder will determine how many codewords are present
            if (sample_buffer_.size() > full_preamble) {
                data_samples = sample_buffer_.size() - full_preamble;
            } else {
                // Need at least 1 LDPC codeword (648 bits) minimum
                int bits_per_symbol = config_.num_carriers * config_.bits_per_symbol;
                int num_symbols = (648 + bits_per_symbol - 1) / bits_per_symbol;
                data_samples = num_symbols * config_.samples_per_symbol;
            }
        }

        size_t total_needed = full_preamble + data_samples;

        if (sample_buffer_.size() < total_needed) {
            return false;  // Wait for more samples
        }

        // Apply CFO correction to samples BEFORE demodulation (like OFDM does)
        if (std::abs(cfo_hz_) > 0.1f) {
            applyCFOCorrection(sample_buffer_, cfo_hz_);
            fprintf(stderr, "[MC-DPSK-DEMOD] Applied CFO correction: %.1f Hz to %zu samples\n",
                    cfo_hz_, sample_buffer_.size());
        }

        // Save dual chirp CFO estimate before training processing
        float dual_chirp_cfo = cfo_hz_;
        fprintf(stderr, "[MC-DPSK-DEMOD] processGotChirp: external=%d, cfo_before=%.1f\n",
                external_chirp_detected_, cfo_hz_);

        // Process training sequence to refine CFO estimate
        // Always run processTraining for now - it may help with timing/phase alignment
        // even when we have good CFO from chirp detection
        {
            size_t training_start = chirp_offset;  // 0 if external chirp, chirp_samples_ otherwise
            SampleSpan train_span(sample_buffer_.data() + training_start, training_samples_);

            // If we have good CFO from chirp, save it and restore after training
            // (processTraining adds residual, which can be wrong)
            float saved_cfo = cfo_hz_;
            bool has_chirp_cfo = external_chirp_detected_ && std::abs(dual_chirp_cfo) > 0.1f;

            processTraining(train_span);
            fprintf(stderr, "[MC-DPSK-DEMOD] after processTraining: cfo=%.1f (was %.1f)\n", cfo_hz_, saved_cfo);

            if (has_chirp_cfo) {
                // Restore chirp CFO - it's more accurate than training estimate
                cfo_hz_ = saved_cfo;
                fprintf(stderr, "[MC-DPSK-DEMOD] restored chirp CFO=%.1f\n", cfo_hz_);
            }

            // CFO sanity check: only reject if NO dual chirp CFO but high training CFO
            if (std::abs(dual_chirp_cfo) < 0.1f && std::abs(cfo_hz_) > 5.0f) {
                // False positive - reset and keep searching
                size_t consume = external_chirp_detected_ ? training_samples_ : chirp_samples_;
                sample_buffer_.erase(sample_buffer_.begin(),
                                    sample_buffer_.begin() + consume);
                state_ = State::IDLE;
                external_chirp_detected_ = false;
                return false;
            }
        }

        // Process reference symbol
        size_t ref_start = chirp_offset + training_samples_;
        SampleSpan ref_span(sample_buffer_.data() + ref_start, ref_samples_);
        setReference(ref_span);

        // Demodulate data
        size_t data_start = full_preamble;
        SampleSpan data_span(sample_buffer_.data() + data_start, data_samples);
        soft_bits_ = demodulateSoft(data_span);

        // Consume processed samples
        sample_buffer_.erase(sample_buffer_.begin(),
                            sample_buffer_.begin() + total_needed);

        state_ = State::FRAME_READY;
        external_chirp_detected_ = false;  // Reset for next frame
        return true;
    }

    // Apply CFO correction to samples using Hilbert transform (proper SSB frequency shift)
    // 1. Convert real signal to analytic (complex) via Hilbert transform
    // 2. Multiply by e^{-j*2*pi*cfo*t} to shift frequency
    // 3. Take real part
    void applyCFOCorrection(Samples& samples, float cfo_hz) {
        if (std::abs(cfo_hz) < 0.01f || samples.size() < 128) return;

        // Use Hilbert transform to get analytic signal
        HilbertTransform hilbert(127);  // 127 taps for good accuracy
        SampleSpan span(samples.data(), samples.size());
        auto analytic = hilbert.process(span);

        // Apply frequency shift: multiply by e^{-j*2*pi*cfo*t}
        float phase_inc = -2.0f * M_PI * cfo_hz / config_.sample_rate;
        float phase = 0.0f;

        for (size_t i = 0; i < samples.size() && i < analytic.size(); i++) {
            Complex rotation(std::cos(phase), std::sin(phase));
            Complex shifted = analytic[i] * rotation;
            samples[i] = shifted.real();  // Take real part

            phase += phase_inc;
            if (phase > M_PI) phase -= 2.0f * M_PI;
            if (phase < -M_PI) phase += 2.0f * M_PI;
        }

        // Reset CFO since it's now been applied to samples
        cfo_hz_ = 0.0f;
    }

    // Demodulate one symbol period for one carrier
    // CFO correction: mix at carrier frequency, but the samples have already been
    // frequency-corrected if cfo_hz_ != 0 (done in applyCFOCorrection)
    Complex demodulateOneSymbol(const float* samples, int carrier_idx) {
        // Mix at carrier frequency only - CFO already corrected in samples
        float freq = carrier_freqs_[carrier_idx];
        float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

        Complex sum(0.0f, 0.0f);
        float phase = 0.0f;

        for (int i = 0; i < config_.samples_per_symbol; i++) {
            Complex mixer = std::polar(1.0f, -phase);
            sum += samples[i] * mixer;
            phase += phase_inc;
        }

        return sum / (float)config_.samples_per_symbol;
    }

    MultiCarrierDPSKConfig config_;
    std::vector<float> carrier_freqs_;
    std::vector<Complex> prev_symbols_;
    sync::ChirpSync chirp_sync_;

    // State machine
    State state_;
    Samples sample_buffer_;
    std::vector<float> soft_bits_;
    float cfo_hz_;
    int chirp_position_;
    float last_chirp_corr_;
    size_t expected_data_bytes_ = 0;
    bool external_chirp_detected_ = false;  // True when chirp detected via external detectSync()

    // Precomputed sizes
    size_t chirp_samples_;
    size_t training_samples_;
    size_t ref_samples_;
    size_t preamble_samples_;
};

// Preset configurations matching commercial HF modem speed levels
namespace mc_dpsk_presets {

// Level 5 equivalent: 3 carriers, ~270 bps raw
inline MultiCarrierDPSKConfig level5() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 3;
    cfg.samples_per_symbol = 512;  // 93.75 baud
    cfg.bits_per_symbol = 2;       // DQPSK
    return cfg;
}

// Level 6 equivalent: 4 carriers, ~363 bps raw
inline MultiCarrierDPSKConfig level6() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 4;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

// Level 7 equivalent: 6 carriers, ~549 bps raw
inline MultiCarrierDPSKConfig level7() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 6;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

// Level 8 equivalent: 8 carriers, ~735 bps raw
inline MultiCarrierDPSKConfig level8() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 8;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

// Level 9 equivalent: 10 carriers, ~922 bps raw
inline MultiCarrierDPSKConfig level9() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 10;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

// Level 10 equivalent: 13 carriers, ~1203 bps raw
inline MultiCarrierDPSKConfig level10() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 13;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

// Ultra levels: More carriers for higher throughput
// These are ProjectUltra-specific, tested on HF fading channels

// Level 11 (Ultra): 20 carriers, ~3750 bps raw
// TESTED: 100% success on moderate/poor fading @ 10-20 dB
// Sweet spot for fading channels - ~105 Hz carrier spacing
inline MultiCarrierDPSKConfig level11_ultra() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 20;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

// Level 12 (Ultra): 30 carriers, ~5625 bps raw
// TESTED: 100% success on GOOD channels @ 25 dB only
// Carrier spacing too tight (~68 Hz) for fading
inline MultiCarrierDPSKConfig level12_ultra() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 30;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;
    return cfg;
}

} // namespace mc_dpsk_presets

} // namespace ultra
