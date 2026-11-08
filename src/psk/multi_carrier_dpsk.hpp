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
#include "ultra/logging.hpp"
#include "sync/chirp_sync.hpp"
#include <vector>
#include <complex>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

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

    // Receiver sample-clock + residual-CFO tracking in the differential demod.
    // Two stations never share a sample clock (independent soundcard crystals, ppm
    // offset). That offset rotates each carrier's per-symbol differential phase
    // PROPORTIONALLY to carrier frequency (a dial CFO rotates all carriers equally);
    // the demod fits/removes both before soft-bit mapping so a normal cheap-card ppm
    // offset is a non-event -- like any commercial soundcard modem. Default ON; a
    // deadband makes it a strict no-op when there is nothing to correct.
    bool track_clock_offset = true;

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
    // Untracked counterpart of the last demodulateSoft() output (empty unless the residual
    // tracker applied a correction). Consumed once.
    std::vector<float> takeUntrackedSoftBits() { return std::move(untracked_soft_bits_); }

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
    bool hasEstimatedSNR() const { return last_snr_valid_; }
    float getEstimatedSNR() const { return last_snr_db_; }

    // Data-aided fade-averaged SNR over the last demodulated DATA span (#58 /
    // BUG-CONNECT-SNR-VARIANCE). Unlike the training estimate (~170 ms = ONE
    // fade state), this averages per-symbol differential SNR linearly across
    // the whole frame (a CONNECT frame spans seconds ~ multiple coherence
    // times) -> fade-averaged by construction. Validity here means "enough
    // data symbols were measured"; the CONSUMER must additionally gate on the
    // frame's LDPC decode success (decode-then-measure) before trusting it.
    bool hasDataAidedSNR() const { return last_data_aided_snr_valid_; }
    float getDataAidedSNRdB() const { return last_data_aided_snr_db_; }

    // PHYSICAL in-band channel SNR (handoff §2): pure power ratio of the
    // received training span vs an externally supplied noise reference —
    // no phase model, so unlike the training/data-aided estimators it reads
    // the CHANNEL's S:N, not the demod-usable margin. The reference must be
    // measured at BURST TIME on the same chain (the decoder's inter-chirp-gap
    // RMS): S:N-holding channel simulators scale idle noise differently from
    // burst noise, and real-radio fade state matches only within the frame.
    void setNoiseReferenceRMS(float rms) { noise_ref_rms_ = rms; }
    bool hasPhysicalSNR() const { return last_physical_snr_valid_; }
    float getPhysicalSNRdB() const { return last_physical_snr_db_; }

    // Set CFO (from external estimation like dual chirp)
    void setCFO(float cfo_hz) { cfo_hz_ = cfo_hz; cfo_initial_phase_ = 0.0f; }

    // Set CFO with initial phase (for continuous audio streams where CFO has accumulated)
    // initial_phase_rad: the accumulated CFO phase at the start of samples
    void setCFOWithPhase(float cfo_hz, float initial_phase_rad) {
        cfo_hz_ = cfo_hz;
        cfo_initial_phase_ = initial_phase_rad;
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

    // Get combined fading index (frequency selectivity + temporal variation)
    // Combines frequency CV (multipath) with temporal CV (Doppler spread)
    // This separates Good (low Doppler) from Moderate (high Doppler) channels
    float getFadingIndex() const {
        float freq_cv = getFrequencyFadingIndex();
        // beta=1.0 weights temporal variation equally with frequency selectivity
        // Temporal CV is the key differentiator: Good ~0.02-0.05, Moderate ~0.10-0.20
        constexpr float beta = 1.0f;
        return freq_cv + beta * temporal_fading_index_;
    }

    // Get frequency-domain fading index only (CV of per-carrier magnitudes)
    float getFrequencyFadingIndex() const {
        if (carrier_magnitudes_.empty()) return 0.0f;

        // Calculate mean
        float sum = 0.0f;
        for (float m : carrier_magnitudes_) sum += m;
        float mean = sum / carrier_magnitudes_.size();

        if (mean < 0.001f) return 0.0f;  // No signal

        // Calculate standard deviation
        float var_sum = 0.0f;
        for (float m : carrier_magnitudes_) {
            float diff = m - mean;
            var_sum += diff * diff;
        }
        float std_dev = std::sqrt(var_sum / carrier_magnitudes_.size());

        // Coefficient of variation (normalized std dev)
        return std_dev / mean;
    }

    // Get temporal fading index only (Doppler-related magnitude variation over time)
    float getTemporalFadingIndex() const { return temporal_fading_index_; }

    // Check if channel appears to be fading based on combined fading index
    // threshold: fading index above this is considered "fading" (default 0.65)
    bool isFading(float threshold = 0.65f) const {
        return getFadingIndex() > threshold;
    }

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
        last_physical_snr_valid_ = false;
        last_physical_snr_db_ = 0.0f;
        noise_ref_rms_ = 0.0f;
        cfo_initial_phase_ = 0.0f;
        corr_cfo_hz_ = 0.0f;
        corr_origin_ = nullptr;
        corr_origin_phase_ = 0.0;
        corr_next_phase_ = 0.0;
        external_chirp_detected_ = false;
        chirp_position_ = -1;
        last_chirp_corr_ = 0.0f;
        expected_data_bytes_ = 0;
        carrier_magnitudes_.clear();
        temporal_fading_index_ = 0.0f;
        last_snr_valid_ = false;
        last_snr_db_ = 0.0f;
        last_data_aided_snr_valid_ = false;
        last_data_aided_snr_db_ = 0.0f;
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

        // Track per-carrier magnitudes for fading detection
        std::vector<float> carrier_mag_sum(config_.num_carriers, 0.0f);
        std::vector<float> carrier_mag_sq_sum(config_.num_carriers, 0.0f);  // For temporal variance

        // Per-symbol total magnitude for silence detection
        std::vector<float> sym_total_mag(num_symbols, 0.0f);

        // Cache differential phases for two-pass demodulation
        std::vector<float> cached_phases(num_symbols * config_.num_carriers);

        // Phase noise estimation accumulators
        float noise_sum = 0.0f;
        int noise_count = 0;

        // Pass 1: Demodulate all symbols, cache phases, estimate phase noise
        for (int sym = 0; sym < num_symbols; sym++) {
            const float* sym_data = data.data() + sym * config_.samples_per_symbol;

            for (int c = 0; c < config_.num_carriers; c++) {
                Complex current = demodulateOneSymbol(sym_data, c);
                float mag = std::abs(current);

                sym_total_mag[sym] += mag;

                // Accumulate magnitude for fading detection
                carrier_mag_sum[c] += mag;
                carrier_mag_sq_sum[c] += mag * mag;  // For temporal variance

                Complex normalized = (mag > 0.0001f) ? current / mag : Complex(1.0f, 0.0f);
                Complex diff = normalized * std::conj(prev_symbols_[c]);
                prev_symbols_[c] = normalized;

                float phase = std::arg(diff);
                cached_phases[sym * config_.num_carriers + c] = phase;

                // Estimate phase noise: find nearest ideal constellation point
                if (config_.bits_per_symbol == 2) {
                    // DQPSK ideal phases: ±π/4, ±3π/4
                    // Quantize to nearest multiple of π/2, offset by π/4
                    float shifted = phase - (float)M_PI / 4.0f;
                    float nearest_idx = std::round(shifted / ((float)M_PI / 2.0f));
                    float ideal_phase = nearest_idx * (float)M_PI / 2.0f + (float)M_PI / 4.0f;
                    float phase_error = phase - ideal_phase;
                    // Wrap to [-π, π]
                    while (phase_error > (float)M_PI) phase_error -= 2.0f * (float)M_PI;
                    while (phase_error < -(float)M_PI) phase_error += 2.0f * (float)M_PI;
                    noise_sum += phase_error * phase_error;
                    noise_count++;
                } else {
                    // DBPSK ideal phases: 0, π
                    float nearest_idx = std::round(phase / (float)M_PI);
                    float ideal_phase = nearest_idx * (float)M_PI;
                    float phase_error = phase - ideal_phase;
                    while (phase_error > (float)M_PI) phase_error -= 2.0f * (float)M_PI;
                    while (phase_error < -(float)M_PI) phase_error += 2.0f * (float)M_PI;
                    noise_sum += phase_error * phase_error;
                    noise_count++;
                }
            }
        }

        // --- Sample-clock-offset + residual-CFO tracking (multi-carrier-native) ---
        // A transmit/receive sample-CLOCK mismatch (ppm) is physically distinct from a
        // dial/carrier-frequency offset. A clock offset eps rotates each carrier's
        // per-symbol differential phase by 2*pi*f_c*eps*T_sym -- PROPORTIONAL to the
        // carrier frequency f_c -- whereas a dial CFO rotates every carrier by the SAME
        // constant. The upstream single-band CFO correction removes only the constant
        // part, leaving a frequency-proportional residual that this fixed integer symbol
        // grid cannot otherwise track, so the tail of a multi-second frame decodes to
        // garbage on a real soundcard pair (the asymmetric handshake failure that never
        // appears in the shared-clock simulator, where both stations share one clock).
        //
        // Recover it with NO per-modulation magic constant: decision-direct each carrier's
        // mean residual differential phase (circular mean, wrap-robust), then do a
        // magnitude-weighted least-squares fit  residual(f) = a + b*f  across carriers.
        // Intercept a = residual dial-CFO term; slope b = clock offset. Subtract the FITTED
        // per-carrier rotation theta_c = a + b*f_c from every symbol -- removing both terms
        // by construction for DBPSK/DQPSK/D8PSK alike. Deadband-gated: when the fit is ~0
        // (the simulator, or a well-clocked pair) theta_c == 0 and this is a strict no-op,
        // so existing behavior and tests are unchanged.
        std::vector<float> theta_c(config_.num_carriers, 0.0f);
        bool clock_corr_active = false;
        if (config_.track_clock_offset && num_symbols >= 8 && config_.num_carriers >= 3) {
            const float two_pi = 2.0f * (float)M_PI;
            const float quad = (float)M_PI / 2.0f;
            std::vector<float> resid(config_.num_carriers, 0.0f);
            std::vector<float> wgt(config_.num_carriers, 0.0f);
            for (int c = 0; c < config_.num_carriers; c++) {
                Complex acc(0.0f, 0.0f);
                for (int sym = 0; sym < num_symbols; sym++) {
                    float phase = cached_phases[sym * config_.num_carriers + c];
                    float r;
                    if (config_.bits_per_symbol == 2) {
                        float shifted = phase - (float)M_PI / 4.0f;
                        float k = std::round(shifted / quad);
                        r = phase - (k * quad + (float)M_PI / 4.0f);
                    } else {
                        float k = std::round(phase / (float)M_PI);
                        r = phase - k * (float)M_PI;
                    }
                    while (r > (float)M_PI) r -= two_pi;
                    while (r < -(float)M_PI) r += two_pi;
                    acc += std::polar(1.0f, r);
                }
                resid[c] = std::arg(acc);
                // Weight by signal strength AND estimate concentration (|R|/N in [0,1]):
                // faded or noisy carriers (low concentration) contribute little to the fit.
                float conc = std::abs(acc) / (float)num_symbols;
                float mag = carrier_mag_sum[c] / (float)num_symbols;
                wgt[c] = mag * conc;
            }
            double Sw = 0, Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
            int used = 0;
            for (int c = 0; c < config_.num_carriers; c++) {
                if (wgt[c] <= 1e-6f) continue;
                double w = wgt[c], x = carrier_freqs_[c], y = resid[c];
                Sw += w; Sx += w * x; Sy += w * y; Sxx += w * x * x; Sxy += w * x * y;
                used++;
            }
            if (used >= 3 && Sw > 0.0) {
                double denom = Sw * Sxx - Sx * Sx;
                if (std::abs(denom) > 1e-9) {
                    double b = (Sw * Sxy - Sx * Sy) / denom;  // rad / Hz / symbol (clock)
                    double a = (Sy - b * Sx) / Sw;            // rad / symbol      (dial CFO)
                    float max_theta = 0.0f;
                    for (int c = 0; c < config_.num_carriers; c++) {
                        theta_c[c] = (float)(a + b * carrier_freqs_[c]);
                        max_theta = std::max(max_theta, std::abs(theta_c[c]));
                    }
                    // Deadband ~2 deg/symbol (~0.26 Hz CFO, ~100 ppm at the band edge):
                    // below this the fixed grid already tolerates the offset, so do nothing.
                    constexpr float kClockDeadbandRad = 0.035f;
                    if (max_theta >= kClockDeadbandRad) {
                        clock_corr_active = true;
                        float t_sym = (float)config_.samples_per_symbol / config_.sample_rate;
                        LOG_DEMOD(DEBUG,
                                  "MC-DPSK clock-track ACTIVE: dial=%.2f Hz, edge rotation=%.1f deg/sym",
                                  (double)((float)a / (two_pi * t_sym)),
                                  (double)(max_theta * 180.0f / (float)M_PI));
                    } else {
                        std::fill(theta_c.begin(), theta_c.end(), 0.0f);
                    }
                }
            }
        }

        // --- Per-symbol common-phase tracking (slow carrier jitter) ---
        // A cheap transmitter's oscillator/clock jitter wanders the carrier phase across
        // the frame (measured ~+-7 Hz on a real USB dongle). It is COMMON to all carriers
        // and time-varying, so the per-frame theta_c above cannot remove it; left in place
        // it pushes the DQPSK differential past its +-45 deg decision boundary on the slow
        // swings and corrupts the payload while the short chirp survives. Track the common
        // differential phase decision-directed per symbol with a full-gain first-order loop
        // (re-anchors each symbol => drift-free, low lag). Three independent, physically
        // grounded gates decide whether to APPLY the result -- so it can help but never hurt
        // a frame the untracked demod would have decoded:
        //   * coherence  (mean |sum|/sum_w across carriers): a true common jitter is coherent
        //     across carriers (~1); per-carrier FADING is incoherent (~0) -> require high
        //     coherence, so this tracks jitter without chasing a fading channel.
        //   * activity   (RMS of the tracked phase): real jitter swings; a clean channel's
        //     few-degree decision-noise wobble does not -> deadband => strict no-op on clean.
        //   * lock       (RMS of the per-symbol prediction residual): a slow jitter tracks
        //     (small residual); jitter near/above the symbol rate cannot be followed (large
        //     residual) -> discard, so near-aliasing fast jitter falls back to untracked.
        std::vector<float> psi_per_sym(num_symbols, 0.0f);
        bool jitter_corr_active = false;
        if (config_.track_clock_offset && num_symbols >= 8 && config_.num_carriers >= 3) {
            const float two_pi = 2.0f * (float)M_PI;
            // Decision-FREE M-th-power common-phase estimate: raising the differential to
            // the M-th power (M=4 for DQPSK, 2 for DBPSK) annihilates the data modulation,
            // leaving M*(common phase) + a known data constant -- no decisions, so no
            // decision-directed cycle slips. Then LOW-PASS the M-th-power phasor across
            // symbols (centered moving average) BEFORE unwrapping: a narrow window cuts the
            // M-th-power noise (~sqrt(W), so the slow swing unwraps cleanly without
            // noise-induced slips) AND attenuates jitter near/above the symbol rate so it
            // cannot be mis-tracked -- it simply falls below the activity deadband and
            // becomes a no-op. Unwrap toward the previous estimate (slow jitter moves < the
            // M-th-power ambiguity per symbol, so the unwrap is unambiguous).
            const int Mpow = (config_.bits_per_symbol == 2) ? 4 : 2;
            const float Mf = (float)Mpow;
            const float data_const = (config_.bits_per_symbol == 2) ? (float)M_PI : 0.0f;
            const float ambig = two_pi / Mf;        // unwrap ambiguity: pi/2 (DQPSK), pi (DBPSK)
            float wsum = 0.0f;
            for (int c = 0; c < config_.num_carriers; c++) wsum += carrier_mag_sum[c];
            std::vector<Complex> Z(num_symbols, Complex(0.0f, 0.0f));
            for (int sym = 0; sym < num_symbols; sym++) {
                Complex z(0.0f, 0.0f);
                for (int c = 0; c < config_.num_carriers; c++) {
                    float ang = Mf * (cached_phases[sym * config_.num_carriers + c] - theta_c[c]);
                    z += carrier_mag_sum[c] * std::polar(1.0f, ang);
                }
                Z[sym] = z;
            }
            constexpr int kSmoothHalf = 3;          // +-3 => 7-tap MA, first null ~ baud/7
            float psi = 0.0f;
            double conc_sum = 0.0, dpsi_sq = 0.0, psi_sq = 0.0;
            for (int sym = 0; sym < num_symbols; sym++) {
                Complex zs(0.0f, 0.0f);
                int cnt = 0;
                for (int j = -kSmoothHalf; j <= kSmoothHalf; j++) {
                    int s = sym + j;
                    if (s >= 0 && s < num_symbols) { zs += Z[s]; ++cnt; }
                }
                float conc = (wsum > 1e-9f) ? std::abs(zs) / ((float)cnt * wsum) : 0.0f;
                float raw = (std::abs(zs) > 1e-12f) ? (std::arg(zs) - data_const) / Mf : psi;
                float cand = raw, d = cand - psi;
                while (d > ambig * 0.5f) { cand -= ambig; d -= ambig; }
                while (d < -ambig * 0.5f) { cand += ambig; d += ambig; }
                float dpsi = cand - psi;
                psi = cand;
                psi_per_sym[sym] = psi;
                conc_sum += conc;
                dpsi_sq += (double)dpsi * dpsi;
                psi_sq += (double)psi * psi;
            }
            const float inv_n = 1.0f / (float)num_symbols;
            float mean_conc = (float)(conc_sum * inv_n);
            float rms_dpsi = (float)std::sqrt(dpsi_sq * inv_n);
            float rms_psi = (float)std::sqrt(psi_sq * inv_n);
            // Apply the correction only when the smoothed M-th-power estimate is trustworthy:
            //   * coherence (mean |sum|/sum_w): high for a true common jitter the loop is
            //     following; LOW for jitter the smoother attenuates / cannot resolve and for
            //     per-carrier fading -> discard => those fall back to untracked (no harm).
            //   * activity (RMS of psi): real swing present, else clean-channel no-op.
            //   * sanity (RMS of psi bounded): a true +-7 Hz jitter is <=~38 deg RMS; a much
            //     larger value means the unwrap slipped/diverged -> discard.
            //   * lock (RMS unwrap step small): per-symbol estimate is smooth.
            // Net: helps slow carrier jitter, provably never corrupts a frame the untracked
            // demod would have decoded.
            constexpr float kCoherenceMin    = 0.40f;  // trackable common jitter vs fading/aliased
            constexpr float kActivityRad     = 0.12f;  // ~7 deg RMS: a real swing is present
            constexpr float kMaxSanePsiRad   = 1.05f;  // ~60 deg RMS ceiling (>physical => diverged)
            constexpr float kLockResidualRad = 0.70f;  // small unwrap step => trackable/slow
            if (mean_conc >= kCoherenceMin && rms_psi >= kActivityRad &&
                rms_psi <= kMaxSanePsiRad && rms_dpsi <= kLockResidualRad) {
                jitter_corr_active = true;
                LOG_DEMOD(DEBUG,
                          "MC-DPSK jitter-track ACTIVE: phaseRMS=%.1f deg coh=%.2f lockResid=%.1f deg",
                          (double)(rms_psi * 180.0f / (float)M_PI), (double)mean_conc,
                          (double)(rms_dpsi * 180.0f / (float)M_PI));
            } else {
                std::fill(psi_per_sym.begin(), psi_per_sym.end(), 0.0f);
            }
        }

        // Pass-1 (uncorrected) noise statistics, kept for the UNTRACKED soft bits below.
        const float untracked_noise_sum = noise_sum;
        const int untracked_noise_count = noise_count;

        // When residual correction is active, recompute the phase-noise variance from the
        // CORRECTED residuals so the LLR scale stays calibrated (lower residual noise ->
        // higher confidence). When inactive this is skipped and pass-1's value is used.
        if (clock_corr_active || jitter_corr_active) {
            float corr_noise_sum = 0.0f;
            int corr_noise_count = 0;
            for (int sym = 0; sym < num_symbols; sym++) {
                for (int c = 0; c < config_.num_carriers; c++) {
                    float phase = cached_phases[sym * config_.num_carriers + c]
                                  - theta_c[c] - psi_per_sym[sym];
                    while (phase > (float)M_PI) phase -= 2.0f * (float)M_PI;
                    while (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
                    float phase_error;
                    if (config_.bits_per_symbol == 2) {
                        float shifted = phase - (float)M_PI / 4.0f;
                        float nearest_idx = std::round(shifted / ((float)M_PI / 2.0f));
                        float ideal = nearest_idx * (float)M_PI / 2.0f + (float)M_PI / 4.0f;
                        phase_error = phase - ideal;
                    } else {
                        float nearest_idx = std::round(phase / (float)M_PI);
                        phase_error = phase - nearest_idx * (float)M_PI;
                    }
                    while (phase_error > (float)M_PI) phase_error -= 2.0f * (float)M_PI;
                    while (phase_error < -(float)M_PI) phase_error += 2.0f * (float)M_PI;
                    corr_noise_sum += phase_error * phase_error;
                    corr_noise_count++;
                }
            }
            noise_sum = corr_noise_sum;
            noise_count = corr_noise_count;
        }

        // Compute SNR-proportional scale from phase noise variance
        float phase_noise_var = (noise_count > 0) ? noise_sum / noise_count : 0.5f;
        phase_noise_var = std::max(0.01f, phase_noise_var);   // Floor: prevents inf at high SNR
        float scale = 2.0f * std::sqrt(1.0f / phase_noise_var);
        scale = std::min(scale, 20.0f);                        // Cap: prevents overconfident LLRs


        // NO-HARM GUARANTEE (2026-11-08). The gates above are heuristics: a tracker that
        // cycle-slips (per-symbol common-phase change beyond the M-th-power unwrap's +-pi/M
        // range, e.g. 5 Hz-peak jitter at 3 Hz) rotates every later symbol by 2 pi/M, which no
        // decision-directed metric can see — the slipped frame looks clean. Only the LDPC can
        // arbitrate, so when a correction is applied the UNTRACKED soft bits are kept too and
        // StreamingDecoder::decodeFrame decodes them if the tracked frame fails.
        untracked_soft_bits_.clear();
        if (clock_corr_active || jitter_corr_active) {
            float var_u = (untracked_noise_count > 0)
                              ? untracked_noise_sum / untracked_noise_count : 0.5f;
            var_u = std::max(0.01f, var_u);
            const float scale_u = std::min(2.0f * std::sqrt(1.0f / var_u), 20.0f);
            untracked_soft_bits_.reserve(static_cast<size_t>(num_symbols) * config_.num_carriers *
                                         config_.bits_per_symbol);
            for (int sym = 0; sym < num_symbols; sym++) {
                for (int c = 0; c < config_.num_carriers; c++) {
                    float phase = cached_phases[sym * config_.num_carriers + c];
                    while (phase > (float)M_PI) phase -= 2.0f * (float)M_PI;
                    while (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;
                    if (config_.bits_per_symbol == 2) {
                        untracked_soft_bits_.push_back(std::max(-20.0f, std::min(20.0f, scale_u * std::sin(phase))));
                        untracked_soft_bits_.push_back(std::max(-20.0f, std::min(20.0f, scale_u * std::sin(2.0f * phase))));
                    } else {
                        untracked_soft_bits_.push_back(std::max(-20.0f, std::min(20.0f, scale_u * std::cos(phase))));
                    }
                }
            }
        }

        // Pass 2: Compute soft bits using calibrated scale. theta_c removes the fitted
        // dial+clock per-carrier rotation; psi_per_sym removes the tracked common-phase
        // jitter. Both are zero when their deadband says there is nothing to correct.
        for (int sym = 0; sym < num_symbols; sym++) {
            for (int c = 0; c < config_.num_carriers; c++) {
                float phase = cached_phases[sym * config_.num_carriers + c]
                              - theta_c[c] - psi_per_sym[sym];
                while (phase > (float)M_PI) phase -= 2.0f * (float)M_PI;
                while (phase < -(float)M_PI) phase += 2.0f * (float)M_PI;

                if (config_.bits_per_symbol == 2) {
                    float sb0 = scale * std::sin(phase);
                    float sb1 = scale * std::sin(2.0f * phase);
                    soft_bits.push_back(std::max(-20.0f, std::min(20.0f, sb0)));
                    soft_bits.push_back(std::max(-20.0f, std::min(20.0f, sb1)));
                } else {
                    float sb = scale * std::cos(phase);
                    soft_bits.push_back(std::max(-20.0f, std::min(20.0f, sb)));
                }
            }
        }

        // Detect and exclude trailing silence symbols from fading measurement
        // Use first few symbols as reference energy, exclude symbols below 20% of reference
        int valid_symbols = num_symbols;
        if (num_symbols >= 4) {
            // Reference: average magnitude of first 4 symbols
            float ref_mag = 0.0f;
            for (int s = 0; s < 4; s++) ref_mag += sym_total_mag[s];
            ref_mag /= 4.0f;

            if (ref_mag > 0.001f) {
                float threshold = ref_mag * 0.2f;
                // Scan from end to find last valid symbol
                while (valid_symbols > 4 && sym_total_mag[valid_symbols - 1] < threshold) {
                    valid_symbols--;
                }
                if (valid_symbols < num_symbols) {
                    // Recompute carrier_mag_sum/sq_sum without silence symbols
                    std::fill(carrier_mag_sum.begin(), carrier_mag_sum.end(), 0.0f);
                    std::fill(carrier_mag_sq_sum.begin(), carrier_mag_sq_sum.end(), 0.0f);
                    for (int sym = 0; sym < valid_symbols; sym++) {
                        const float* sym_data = data.data() + sym * config_.samples_per_symbol;
                        for (int c = 0; c < config_.num_carriers; c++) {
                            float mag = std::abs(demodulateOneSymbol(sym_data, c));
                            carrier_mag_sum[c] += mag;
                            carrier_mag_sq_sum[c] += mag * mag;
                        }
                    }
                }
            }
        }

        // Store average per-carrier magnitudes for fading detection (using valid symbols only)
        carrier_magnitudes_.resize(config_.num_carriers);
        for (int c = 0; c < config_.num_carriers; c++) {
            carrier_magnitudes_[c] = (valid_symbols > 0) ? carrier_mag_sum[c] / valid_symbols : 0.0f;
        }

        // Compute temporal fading index from symbol-to-symbol magnitude variance
        // Uses E[x^2] - E[x]^2 formula per carrier, then averages CV across carriers
        if (valid_symbols >= 4) {
            float temporal_cv_sum = 0.0f;
            int valid_carriers = 0;
            for (int c = 0; c < config_.num_carriers; c++) {
                float mean = carrier_mag_sum[c] / valid_symbols;
                if (mean < 0.001f) continue;  // Skip dead carriers
                float mean_sq = carrier_mag_sq_sum[c] / valid_symbols;
                float variance = std::max(0.0f, mean_sq - mean * mean);
                float cv = std::sqrt(variance) / mean;
                temporal_cv_sum += cv;
                valid_carriers++;
            }
            temporal_fading_index_ = (valid_carriers > 0) ? temporal_cv_sum / valid_carriers : 0.0f;
        } else {
            temporal_fading_index_ = 0.0f;
        }

        updateDataAidedSNREstimate(cached_phases, theta_c, psi_per_sym, valid_symbols);

        return soft_bits;
    }

    std::vector<float> demodulateDataOnly(SampleSpan data) {
        // A data-only burst continuation is contiguous with what was consumed before it: the
        // same dial offset, the phase continuing where the previous block ended.
        corr_origin_ = data.data();
        corr_origin_phase_ = corr_next_phase_;
        auto bits = demodulateSoft(data);
        corr_next_phase_ += 2.0 * M_PI * static_cast<double>(corr_cfo_hz_) *
                            static_cast<double>(data.size()) /
                            static_cast<double>(config_.sample_rate);
        corr_origin_ = nullptr;
        return bits;
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

        // CFO is removed INSIDE the per-carrier correlator (demodulateOneSymbol), exactly, with a
        // phase continuous from this frame origin through every burst continuation (2026-11-08).
        // Replaced a 127-tap Hilbert pre-shift whose 63-sample group delay was never compensated
        // (every symbol window slid 63 samples), whose image sat at -22 dB on the 500 Hz carrier,
        // and which ZEROED cfo_hz_ — so data-only burst continuations were demodulated with no
        // CFO correction at all (measured: +7.3 Hz dial offset, AWGN@12, every 8-CW continuation
        // lost; training SNR read 6.4 dB on a 12.1 dB link). The 0 Hz simulator never ran it.
        corr_cfo_hz_ = cfo_hz_;
        corr_origin_ = sample_buffer_.data();
        corr_origin_phase_ = cfo_initial_phase_;
        LOG_DEMOD(DEBUG, "MC-DPSK: CFO %.2f Hz removed in the correlator (%zu samples)",
                  cfo_hz_, sample_buffer_.size());

        // Save dual chirp CFO estimate before training processing
        float dual_chirp_cfo = cfo_hz_;
        LOG_DEMOD(DEBUG, "MC-DPSK: processGotChirp: external=%d, cfo_before=%.1f",
                  external_chirp_detected_, cfo_hz_);

        // Process training sequence to refine CFO estimate
        // Always run processTraining for now - it may help with timing/phase alignment
        // even when we have good CFO from chirp detection
        {
            size_t training_start = chirp_offset;  // 0 if external chirp, chirp_samples_ otherwise
            SampleSpan train_span(sample_buffer_.data() + training_start, training_samples_);
            updateTrainingSNREstimate(train_span);

            // If external chirp detection was used, ALWAYS trust the chirp CFO
            // (processTraining's CFO estimate is unreliable - it can give spurious values
            // even when processing correct training samples, especially at low SNR)
            // The dual chirp CFO measurement is more robust.
            float saved_cfo = cfo_hz_;

            processTraining(train_span);
            LOG_DEMOD(DEBUG, "MC-DPSK: after processTraining: cfo=%.1f (was %.1f)", cfo_hz_, saved_cfo);

            if (external_chirp_detected_) {
                // Restore chirp CFO - it's more accurate than training estimate
                // even when chirp CFO is ~0 Hz
                cfo_hz_ = saved_cfo;
                LOG_DEMOD(DEBUG, "MC-DPSK: restored chirp CFO=%.1f (external chirp detected)", cfo_hz_);
            }

            // CFO sanity check: only reject if NO dual chirp CFO but high training CFO
            // Skip this check when external chirp detection was used - the chirp detector
            // already validated with good correlation, so trust it even with zero CFO
            if (!external_chirp_detected_ &&
                std::abs(dual_chirp_cfo) < 0.1f && std::abs(cfo_hz_) > 5.0f) {
                // False positive - reset and keep searching
                size_t consume = chirp_samples_;  // Internal detection only
                sample_buffer_.erase(sample_buffer_.begin(),
                                    sample_buffer_.begin() + consume);
                state_ = State::IDLE;
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

        // Consume processed samples. The CFO phase continues at the next sample (a data-only
        // burst continuation starts exactly there).
        corr_next_phase_ = corr_origin_phase_ + 2.0 * M_PI * static_cast<double>(corr_cfo_hz_) *
                                                    static_cast<double>(total_needed) /
                                                    static_cast<double>(config_.sample_rate);
        corr_origin_ = nullptr;  // the buffer is about to move
        sample_buffer_.erase(sample_buffer_.begin(),
                            sample_buffer_.begin() + total_needed);

        state_ = State::FRAME_READY;
        external_chirp_detected_ = false;  // Reset for next frame
        return true;
    }

    void updateTrainingSNREstimate(SampleSpan training) {
        last_snr_valid_ = false;
        last_snr_db_ = 0.0f;
        // Per-frame semantics (external review 2026-07-10, finding 4): the
        // physical latch is valid ONLY for the frame that measured it — an
        // abstaining later frame (signal-presence gate refused) must not
        // republish the previous frame's value into the distribution ring
        // and the SNR-SANITY comparison.
        last_physical_snr_valid_ = false;
        last_physical_snr_db_ = 0.0f;
        if (training.size() < training_samples_ ||
            config_.training_symbols <= 0 ||
            config_.num_carriers <= 0) {
            return;
        }

        std::vector<Complex> channel(static_cast<size_t>(config_.num_carriers),
                                     Complex(0.0f, 0.0f));
        for (int c = 0; c < config_.num_carriers; ++c) {
            Complex sum(0.0f, 0.0f);
            for (int sym = 0; sym < config_.training_symbols; ++sym) {
                const float phase = static_cast<float>(c * sym) *
                                    static_cast<float>(M_PI) / 2.0f;
                const Complex expected = std::polar(1.0f, phase);
                const Complex observed = demodulateOneSymbol(
                    training.data() + sym * config_.samples_per_symbol, c);
                sum += observed * std::conj(expected);
            }
            channel[static_cast<size_t>(c)] =
                sum / static_cast<float>(config_.training_symbols);
        }

        double signal_power = 0.0;
        double residual_power = 0.0;
        double rx_power = 0.0;  // raw received in-band power (physical SNR)
        size_t sample_count = 0;
        FIRFilter signal_filter = FIRFilter::bandpass(101, 50.0f, 2950.0f,
                                                       config_.sample_rate);
        FIRFilter residual_filter = FIRFilter::bandpass(101, 50.0f, 2950.0f,
                                                         config_.sample_rate);
        FIRFilter rx_filter = FIRFilter::bandpass(101, 50.0f, 2950.0f,
                                                   config_.sample_rate);
        for (int sym = 0; sym < config_.training_symbols; ++sym) {
            for (int i = 0; i < config_.samples_per_symbol; ++i) {
                Complex fitted_complex(0.0f, 0.0f);
                for (int c = 0; c < config_.num_carriers; ++c) {
                    const float train_phase = static_cast<float>(c * sym) *
                                              static_cast<float>(M_PI) / 2.0f;
                    const Complex training_symbol = std::polar(1.0f, train_phase);
                    const float carrier_phase =
                        2.0f * static_cast<float>(M_PI) * carrier_freqs_[static_cast<size_t>(c)] *
                        static_cast<float>(i) / config_.sample_rate;
                    const Complex carrier = std::polar(1.0f, carrier_phase);
                    fitted_complex += channel[static_cast<size_t>(c)] *
                                      training_symbol * carrier;
                }

                const float fitted = 2.0f * fitted_complex.real();
                const float sample =
                    training[static_cast<size_t>(sym * config_.samples_per_symbol + i)];
                const float residual = sample - fitted;
                const float signal_in_band = signal_filter.process(fitted);
                const float residual_in_band = residual_filter.process(residual);
                const float rx_in_band = rx_filter.process(sample);
                signal_power += static_cast<double>(signal_in_band) * signal_in_band;
                residual_power += static_cast<double>(residual_in_band) * residual_in_band;
                rx_power += static_cast<double>(rx_in_band) * rx_in_band;
                ++sample_count;
            }
        }

        if (sample_count == 0) {
            return;
        }
        signal_power /= static_cast<double>(sample_count);
        residual_power /= static_cast<double>(sample_count);
        rx_power /= static_cast<double>(sample_count);

        // PHYSICAL channel SNR (handoff §2): (P_rx − N)/N over the exact
        // training span, N = the burst-time noise reference the decoder set
        // from THIS frame's inter-chirp gap. Signal-presence gate ≥3 dB above
        // the reference: a noise-only PING "training" span must never latch
        // (rig F228: a latched noise span poisoned the readout all session).
        // Definition: PAYLOAD-REFERENCED channel SNR (handoff §2 option (a)):
        // the training/data-section signal power over in-band noise — the SNR
        // the payload actually experiences. Measured at sim awgn@10: reads
        // 9.2-9.6 (−0.4..−0.8 vs dial; the residual is the noise-ref's known
        // −0.66 dB, filed). NOTE: the +1.72 dB dial-equivalent constant from
        // the 07-07 audit OVERSHOOTS on the live CONNECT profile (measured
        // +1.0..+1.3 at truth) — it was derived for a different geometry. A
        // dial-equivalent display needs the per-profile derivation test
        // (generate PING + training at equal drive, measure both in-band
        // powers) before any constant is added here. Never tune it by eye.
        if (noise_ref_rms_ > 0.0f) {
            const double p_n = static_cast<double>(noise_ref_rms_) *
                               static_cast<double>(noise_ref_rms_);
            if (p_n > 0.0 && rx_power > 2.0 * p_n) {
                last_physical_snr_db_ = static_cast<float>(
                    10.0 * std::log10((rx_power - p_n) / p_n));
                last_physical_snr_valid_ = true;
            }
        }
        if (signal_power <= 0.0 || residual_power <= std::numeric_limits<double>::min()) {
            return;
        }

        const float snr_db = static_cast<float>(
            10.0 * std::log10(signal_power / residual_power));
        if (!std::isfinite(snr_db)) {
            return;
        }
        last_snr_db_ = std::clamp(snr_db, -20.0f, 60.0f);
        last_snr_valid_ = true;
    }

    // Data-aided fade-averaged SNR over the demodulated DATA span (#58 /
    // BUG-CONNECT-SNR-VARIANCE). updateTrainingSNREstimate above measures ONLY
    // the ~170 ms training preamble = ONE fade state (Tc ~ 4.2 s on Good), so
    // connect-time snapshots spread ~10 dB pick-to-pick on a fading rig. A
    // CONNECT frame's 4 CWs span seconds (> Tc), so a per-symbol estimate
    // averaged LINEARLY over the whole frame is fade-averaged by construction,
    // with zero handshake latency (decode-then-measure).
    //
    // Differential-level design (immune to channel drift across the frame — no
    // static channel reconstruction): per (symbol, carrier) the differential
    // product d = curr * conj(prev) is already reduced to a UNIT phasor
    // (pass 1 normalizes carrier outputs), so the error to the nearest
    // constellation point (hard decision on d's phase; DBPSK 2 points, DQPSK 4
    // points, from config_.bits_per_symbol) is the chord
    //   |e|^2 = |exp(j*phi) - exp(j*phi_hat)|^2 = 4 sin^2(phi_err/2) ~ phi_err^2,
    // a PHASE-only residual. Calibration from first principles: with
    // per-carrier post-correlator SNR rho, each coherent phase has variance
    // 1/(2*rho) (only the tangential noise half moves the phase); the
    // differential doubles it -> E[phi_err^2] ~ 1/rho. So
    //   SNR_diff(sym) = signal/error = num_carriers / sum_c |e_c|^2
    // estimates rho DIRECTLY: the magnitude normalization discards the radial
    // noise half, which exactly cancels the differential +3.01 dB doubling (the
    // naive "SNR = 2 x SNR_diff" applies only to a non-normalized error vector).
    // Three analytic corrections, all derived from parameters in scope (no
    // per-modulation constants — CLAUDE.md adaptivity rule):
    //   * deterministic ICI: the carriers are NOT orthogonal on the symbol grid
    //     (spacing is not a multiple of the symbol rate), so each carrier's
    //     correlator leaks neighbor power |G(df)|^2 = (sin(pi*df*T) /
    //     (Nsps*sin(pi*df/fs)))^2 — a geometry-computable error floor
    //     (~-29 dB/carrier for the 8x1024 CONNECT geometry, CONFIRMED: the
    //     measured high-SNR excess error 1.1-1.4e-3 matches the computed
    //     1.25e-3). It enters the phase-error metric exactly like noise
    //     (tangential half, doubled by the differential), so subtract the
    //     computed sum from the measured error power.
    //   * block averaging: per-symbol inversion has only num_carriers
    //     chi-square dof -> heavy-tailed 1/x bias and an unstable ICI
    //     subtraction. Average the error over ~0.2 s blocks first: >= 8x
    //     symbols the dof, still << Tc of every supported channel (Good ~4.2 s,
    //     Moderate ~1 s), so the linear block-SNR average stays fade-averaged
    //     by construction. Remaining inverse-chi-square bias k/(k-2) (k = block
    //     dof) is corrected analytically.
    //   * in-band basis: each carrier's symbol correlator has noise bandwidth
    //     = symbol_rate and the TX power splits across num_carriers, so
    //     SNR_inband = rho * num_carriers * symbol_rate / 2900 (same 50-2950 Hz
    //     passband convention as updateTrainingSNREstimate / the AWGN truth).
    // Measured residual calibration + hard-decision bias: see
    // kDataAidedResidualCalDb below and tests/test_mcdpsk_snr_calibration.cpp.
    void updateDataAidedSNREstimate(const std::vector<float>& cached_phases,
                                    const std::vector<float>& theta_c,
                                    const std::vector<float>& psi_per_sym,
                                    int valid_symbols) {
        last_data_aided_snr_valid_ = false;
        last_data_aided_snr_db_ = 0.0f;

        const int nc = config_.num_carriers;
        // Need >2 carriers and enough symbols for a meaningful average
        // (mirrors the fading-index gate).
        if (valid_symbols < 4 || nc < 3) {
            return;
        }

        // Deterministic ICI floor of this carrier/symbol geometry (see header
        // comment): sum over carrier pairs of the correlator leakage power,
        // per symbol (relative to the unit-normalized per-carrier signal).
        const double nsps = static_cast<double>(config_.samples_per_symbol);
        const double fs = static_cast<double>(config_.sample_rate);
        double ici_sum = 0.0;
        for (int c = 0; c < nc; ++c) {
            for (int o = 0; o < nc; ++o) {
                if (o == c) continue;
                const double df = static_cast<double>(carrier_freqs_[static_cast<size_t>(o)]) -
                                  static_cast<double>(carrier_freqs_[static_cast<size_t>(c)]);
                const double den = nsps * std::sin(M_PI * df / fs);
                if (std::abs(den) < 1e-12) continue;
                const double g = std::sin(M_PI * df * nsps / fs) / den;
                ici_sum += g * g;
            }
        }

        // Block length: ~0.2 s of symbols (min 8) — small vs every supported
        // channel's coherence time, large enough for stable statistics.
        const float symbol_rate = config_.getSymbolRate();
        const int block_syms = std::max(
            8, static_cast<int>(std::lround(0.2f * symbol_rate)));

        const float two_pi = 2.0f * static_cast<float>(M_PI);
        const float quad = static_cast<float>(M_PI) / 2.0f;
        double snr_sum = 0.0;
        int block_count = 0;
        int sym = 0;
        while (sym < valid_symbols) {
            // Fold a short trailing remainder into the final block.
            int len = std::min(block_syms, valid_symbols - sym);
            if (valid_symbols - (sym + len) < block_syms / 2) {
                len = valid_symbols - sym;
            }
            double err_power = 0.0;
            for (int s = sym; s < sym + len; ++s) {
                for (int c = 0; c < nc; ++c) {
                    float phase = cached_phases[static_cast<size_t>(s) * nc + c]
                                  - theta_c[static_cast<size_t>(c)]
                                  - psi_per_sym[static_cast<size_t>(s)];
                    while (phase > static_cast<float>(M_PI)) phase -= two_pi;
                    while (phase < -static_cast<float>(M_PI)) phase += two_pi;
                    float phase_error;
                    if (config_.bits_per_symbol == 2) {
                        // DQPSK differential constellation: +-pi/4, +-3pi/4
                        const float shifted = phase - static_cast<float>(M_PI) / 4.0f;
                        const float k = std::round(shifted / quad);
                        phase_error = phase - (k * quad + static_cast<float>(M_PI) / 4.0f);
                    } else {
                        // DBPSK differential constellation: 0, pi
                        const float k = std::round(phase / static_cast<float>(M_PI));
                        phase_error = phase - k * static_cast<float>(M_PI);
                    }
                    while (phase_error > static_cast<float>(M_PI)) phase_error -= two_pi;
                    while (phase_error < -static_cast<float>(M_PI)) phase_error += two_pi;
                    const float half_sin = std::sin(0.5f * phase_error);
                    err_power += 4.0 * static_cast<double>(half_sin) * half_sin;
                }
            }
            // Subtract the deterministic ICI floor. Keep >= 20% of the raw
            // measurement so a lucky block cannot go negative/explode: this
            // bounds the correction at +7 dB, i.e. the estimator saturates
            // ~7 dB above the raw ICI ceiling instead of recovering unbounded
            // SNR from an ICI-dominated residual (honest saturation).
            const double err_noise =
                std::max(err_power - ici_sum * len, 0.2 * err_power);
            // Signal power per (symbol, carrier) is 1 by construction (unit
            // phasor). Inverse-chi-square correction (k-2)/k, k = block dof
            // (1 phase dof per carrier per symbol).
            const double dof = static_cast<double>(len) * nc;
            const double signal_power = static_cast<double>(len) * nc;
            snr_sum += (signal_power / std::max(err_noise, signal_power * 1e-6)) *
                       ((dof - 2.0) / dof);
            ++block_count;
            sym += len;
        }
        if (block_count == 0) {
            return;
        }

        const double mean_rho = snr_sum / block_count;
        // 50-2950 Hz in-band noise bandwidth: the same passband the training
        // estimator (and the AWGN truth calibration) ratios over.
        constexpr double kInBandNoiseBandwidthHz = 2900.0;
        // Residual calibration, MEASURED on tests/test_mcdpsk_snr_calibration
        // (AWGN, CONNECT geometry 8 x DQPSK @ 46.875 baud, 5 seeds/point):
        // see the measured table in the test file. Covers the second-order
        // differential noise term the first-order derivation drops.
        constexpr double kDataAidedResidualCalDb = 0.5;
        const double snr_inband =
            mean_rho * static_cast<double>(nc) *
            static_cast<double>(symbol_rate) / kInBandNoiseBandwidthHz *
            std::pow(10.0, kDataAidedResidualCalDb / 10.0);
        if (snr_inband <= 0.0) {
            return;
        }
        const float snr_db = static_cast<float>(10.0 * std::log10(snr_inband));
        if (!std::isfinite(snr_db)) {
            return;
        }
        last_data_aided_snr_db_ = std::clamp(snr_db, -20.0f, 60.0f);
        last_data_aided_snr_valid_ = true;
    }

    // Demodulate one symbol period for one carrier
    // CFO correction: mix at carrier frequency, but the samples have already been
    // (the CFO is removed inside demodulateOneSymbol; see corr_cfo_hz_)
    Complex demodulateOneSymbol(const float* samples, int carrier_idx) {
        // The carrier term restarts at 0 every symbol (the modulator's convention); the CFO term
        // is CONTINUOUS from the frame origin (corr_origin_), so a dial offset corr_cfo_hz_ is
        // removed exactly — mixing a REAL signal with a complex exponential needs no Hilbert.
        const double fs = static_cast<double>(config_.sample_rate);
        double cfo_phase0 = 0.0;
        if (corr_cfo_hz_ != 0.0f && corr_origin_ != nullptr) {
            cfo_phase0 = corr_origin_phase_ + 2.0 * M_PI * static_cast<double>(corr_cfo_hz_) *
                                                  static_cast<double>(samples - corr_origin_) / fs;
        }
        const float phase_inc = static_cast<float>(
            2.0 * M_PI * (static_cast<double>(carrier_freqs_[carrier_idx]) +
                          static_cast<double>(corr_cfo_hz_)) / fs);

        Complex sum(0.0f, 0.0f);
        float phase = static_cast<float>(std::fmod(cfo_phase0, 2.0 * M_PI));

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
    float cfo_initial_phase_ = 0.0f;  // Initial phase for CFO correction (accumulated at frame start)
    // CFO removed inside demodulateOneSymbol: value, the sample that carries phase
    // corr_origin_phase_, and the phase at the first sample after the last consumed block.
    float corr_cfo_hz_ = 0.0f;
    std::vector<float> untracked_soft_bits_;
    const float* corr_origin_ = nullptr;
    double corr_origin_phase_ = 0.0;
    double corr_next_phase_ = 0.0;
    int chirp_position_;
    float last_chirp_corr_;
    size_t expected_data_bytes_ = 0;
    bool external_chirp_detected_ = false;  // True when chirp detected via external detectSync()

    // Per-carrier signal magnitudes (for fading detection)
    std::vector<float> carrier_magnitudes_;

    // Temporal fading index (Doppler-related magnitude variation over symbols)
    float temporal_fading_index_ = 0.0f;
    bool last_snr_valid_ = false;
    float last_snr_db_ = 0.0f;
    float noise_ref_rms_ = 0.0f;         // burst-time N (set per frame by the decoder)
    bool last_physical_snr_valid_ = false;
    float last_physical_snr_db_ = 0.0f;  // physical channel SNR (dial convention)
    // #58: data-aided fade-averaged SNR over the last demodulated data span.
    bool last_data_aided_snr_valid_ = false;
    float last_data_aided_snr_db_ = 0.0f;

    // Precomputed sizes
    size_t chirp_samples_;
    size_t training_samples_;
    size_t ref_samples_;
    size_t preamble_samples_;
};

// Preset configurations matching commercial HF modem speed levels
namespace mc_dpsk_presets {

// Robust-Low: 8 carriers, DBPSK, 23.4 baud (2048 samples/symbol),
// R1/4 LDPC. Targets SNR -3 to -8 dB Moderate fading. Trades 4x
// frame airtime for +6 dB symbol energy and +3 dB DBPSK margin.
inline MultiCarrierDPSKConfig robust_low() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 8;
    cfg.samples_per_symbol = 2048;  // 23.4375 baud, +6 dB vs 512
    cfg.bits_per_symbol = 1;        // DBPSK, +3 dB vs DQPSK
    return cfg;
}

// Robust-Mid: 8 carriers, DBPSK, 46.9 baud (1024 samples/symbol),
// R1/4 LDPC. Targets SNR -3 to 0 dB Moderate fading. Same
// modulation/FEC as Robust-Low; halved samples_per_symbol gives
// 2x airtime for 3 dB margin cost. Same 3-CW bounded variable
// frame geometry.
inline MultiCarrierDPSKConfig robust_mid() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 8;
    cfg.samples_per_symbol = 1024;  // 46.875 baud, 2x faster than Robust-Low
    cfg.bits_per_symbol = 1;        // DBPSK
    return cfg;
}

// Robust: 8 carriers, DQPSK, 46.9 baud (1024 samples/symbol),
// R1/4 LDPC. Targets SNR 0 to +3 dB Moderate fading. Same SPS
// as Robust-Mid; flip DBPSK -> DQPSK for 2x bps (-3 dB margin).
inline MultiCarrierDPSKConfig robust() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 8;
    cfg.samples_per_symbol = 1024;
    cfg.bits_per_symbol = 2;  // DQPSK
    return cfg;
}

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

// Narrowband preset: 4 carriers in 500 Hz band (1300-1700 Hz)
// Uses narrowband chirp (1250-1750 Hz, 1000ms) for dual-listen compatibility
inline MultiCarrierDPSKConfig narrowband() {
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 4;
    cfg.freq_low = 1300.0f;
    cfg.freq_high = 1700.0f;
    cfg.samples_per_symbol = 512;
    cfg.bits_per_symbol = 2;       // DQPSK
    cfg.chirp_f_start = 1250.0f;
    cfg.chirp_f_end = 1750.0f;
    cfg.chirp_duration_ms = 1000.0f;
    return cfg;
}

} // namespace mc_dpsk_presets

} // namespace ultra
