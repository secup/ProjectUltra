// OFDM Synchronization - Schmidl-Cox, LTS timing, CFO estimation
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <cmath>
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "ultra/fec.hpp"  // For LDPCDecoder in hunting mode
#include "ultra/logging.hpp"
#include <algorithm>

namespace ultra {

using namespace demod_constants;

// =============================================================================
// ENERGY DETECTION
// =============================================================================

bool OFDMDemodulator::Impl::hasMinimumEnergy(size_t offset, size_t window_len) {
    if (offset + window_len > rx_buffer.size()) return false;

    // Sample every 16th point for speed
    float sum_sq = 0;
    size_t count = 0;

    for (size_t i = 0; i < window_len; i += SAMPLE_STEP_ENERGY_CHECK) {
        float s = rx_buffer[offset + i];
        sum_sq += s * s;
        ++count;
    }

    float energy = sum_sq / count;

    // Amplitude-independent energy detection using noise floor tracking
    if (noise_floor_energy < 1e-20f) {
        noise_floor_energy = energy * 0.1f;
    }

    // Track minimum energy (likely noise floor)
    if (energy < noise_floor_energy) {
        noise_floor_energy = energy;
    } else if (energy < noise_floor_energy * 3.0f) {
        noise_floor_energy = (1.0f - NOISE_FLOOR_ALPHA) * noise_floor_energy
                           + NOISE_FLOOR_ALPHA * energy;
    }

    float threshold = noise_floor_energy * ENERGY_RATIO_THRESHOLD;
    return energy >= threshold;
}

// =============================================================================
// HILBERT TRANSFORM (Analytic Signal)
// =============================================================================

std::vector<Complex> OFDMDemodulator::Impl::toAnalytic(const float* samples, size_t len) {
    // Pad to next power of 2 for FFT efficiency
    size_t fft_len = 1;
    while (fft_len < len) fft_len *= 2;

    FFT hilbert_fft(fft_len);

    std::vector<Complex> time_in(fft_len, Complex(0, 0));
    for (size_t i = 0; i < len; ++i) {
        time_in[i] = Complex(samples[i], 0);
    }

    std::vector<Complex> freq(fft_len);
    hilbert_fft.forward(time_in, freq);

    // Create analytic signal by zeroing negative frequencies
    for (size_t i = 1; i < fft_len / 2; ++i) {
        freq[i] *= 2.0f;
    }
    for (size_t i = fft_len / 2 + 1; i < fft_len; ++i) {
        freq[i] = Complex(0, 0);
    }

    std::vector<Complex> analytic(fft_len);
    hilbert_fft.inverse(freq, analytic);

    analytic.resize(len);
    return analytic;
}

// =============================================================================
// CORRELATION FUNCTIONS
// =============================================================================

float OFDMDemodulator::Impl::measureRealCorrelation(size_t offset, float* out_energy) {
    if (offset + symbol_samples * 2 > rx_buffer.size()) return 0.0f;

    size_t len = config.fft_size + config.getCyclicPrefix();

    float P = 0.0f;
    float E1 = 0.0f;
    float E2 = 0.0f;

    for (size_t i = 0; i < len; ++i) {
        float s1 = rx_buffer[offset + i];
        float s2 = rx_buffer[offset + i + len];
        P += s1 * s2;
        E1 += s1 * s1;
        E2 += s2 * s2;
    }

    if (out_energy) *out_energy = E2;

    if (E1 < MIN_ENERGY_THRESHOLD || E2 < MIN_ENERGY_THRESHOLD) {
        return 0.0f;
    }

    float normalization = std::sqrt(E1 * E2);
    return P / normalization;
}

// Schmidl-Cox correlation: correlate first half with second half of FFT portion
float OFDMDemodulator::Impl::measureSchmidlCoxCorrelation(size_t offset, Complex* out_P, float* out_energy) {
    size_t cp_len = config.getCyclicPrefix();
    size_t fft_len = config.fft_size;
    size_t half_len = fft_len / 2;

    if (offset + cp_len + fft_len > rx_buffer.size()) {
        if (out_energy) *out_energy = 0.0f;
        return 0.0f;
    }

    size_t data_start = offset + cp_len;

    // Remove DC offset before correlation
    float dc_sum = 0.0f;
    for (size_t i = 0; i < fft_len; ++i) {
        dc_sum += rx_buffer[data_start + i];
    }
    float dc_offset = dc_sum / fft_len;

    std::vector<float> dc_removed(fft_len);
    for (size_t i = 0; i < fft_len; ++i) {
        dc_removed[i] = rx_buffer[data_start + i] - dc_offset;
    }

    auto analytic = toAnalytic(dc_removed.data(), fft_len);

    Complex P(0.0f, 0.0f);
    float R1 = 0.0f;
    float R2 = 0.0f;

    for (size_t i = 0; i < half_len; ++i) {
        P += std::conj(analytic[i]) * analytic[i + half_len];
        R1 += std::norm(analytic[i]);
        R2 += std::norm(analytic[i + half_len]);
    }

    if (out_P) *out_P = P;
    if (out_energy) *out_energy = R2;

    float normalization = std::sqrt(R1 * R2);
    if (normalization < 1e-10f) {
        return 0.0f;
    }

    return std::abs(P) / normalization;
}

float OFDMDemodulator::Impl::measureAnalyticCorrelation(size_t offset, Complex* out_P, float* out_energy) {
    if (offset + symbol_samples * 2 > rx_buffer.size()) return 0.0f;

    size_t len = config.fft_size + config.getCyclicPrefix();
    auto analytic = toAnalytic(&rx_buffer[offset], len * 2);

    Complex P(0.0f, 0.0f);
    float E1 = 0.0f;
    float E2 = 0.0f;

    for (size_t i = 0; i < len; ++i) {
        P += std::conj(analytic[i]) * analytic[i + len];
        E1 += std::norm(analytic[i]);
        E2 += std::norm(analytic[i + len]);
    }

    if (out_P) *out_P = P;
    if (out_energy) *out_energy = E2;

    float normalization = std::sqrt(E1 * E2);
    return std::abs(P) / (normalization + 1e-10f);
}

float OFDMDemodulator::Impl::measureCorrelation(size_t offset, float* out_energy) {
    return measureSchmidlCoxCorrelation(offset, nullptr, out_energy);
}

bool OFDMDemodulator::Impl::detectSync(size_t offset) {
    float energy = 0;
    float normalized = measureCorrelation(offset, &energy);

    // Update noise floor estimate when correlation is low (likely noise)
    if (normalized < 0.3f) {
        if (noise_floor_energy < 1e-10f) {
            noise_floor_energy = energy;
        } else {
            noise_floor_energy = (1.0f - NOISE_FLOOR_ALPHA) * noise_floor_energy
                               + NOISE_FLOOR_ALPHA * energy;
        }
    }

    float energy_threshold = noise_floor_energy * SIGNAL_TO_NOISE_RATIO_THRESHOLD;
    energy_threshold = std::max(energy_threshold, MIN_ABSOLUTE_ENERGY);

    if (offset % 5000 == 0 || (normalized > 0.5f && energy > energy_threshold)) {
        LOG_DEMOD(TRACE, "detectSync offset=%zu corr=%.3f energy=%.3g noise_floor=%.3g thresh=%.3g",
                offset, normalized, energy, noise_floor_energy, energy_threshold);
    }

    if (energy < energy_threshold) {
        return false;
    }

    if (offset % 500 == 0 || normalized > 0.5f) {
        LOG_DEMOD(TRACE, "detectSync offset=%zu normalized=%.3f energy=%.3g noise_floor=%.3g threshold=%.2f",
                offset, normalized, energy, noise_floor_energy, sync_threshold);
    }

    return normalized > sync_threshold;
}

// =============================================================================
// CFO ESTIMATION
// =============================================================================

float OFDMDemodulator::Impl::estimateCoarseCFO(size_t sync_offset) {
    size_t cp_len = config.getCyclicPrefix();
    size_t fft_len = config.fft_size;
    size_t half_len = fft_len / 2;

    size_t data_start = sync_offset + cp_len;

    if (data_start + fft_len > rx_buffer.size()) {
        return 0.0f;
    }

    auto analytic = toAnalytic(&rx_buffer[data_start], fft_len);

    Complex P(0.0f, 0.0f);
    for (size_t i = 0; i < half_len; ++i) {
        P += std::conj(analytic[i]) * analytic[i + half_len];
    }

    float phase = std::atan2(P.imag(), P.real());

    // Schmidl-Cox CFO formula: CFO = phase × fs / (π × N)
    float cfo_hz = phase * config.sample_rate / (M_PI * fft_len);

    // Clamp to unambiguous range
    float max_cfo = config.sample_rate / fft_len;
    cfo_hz = std::max(-max_cfo, std::min(max_cfo, cfo_hz));

    LOG_DEMOD(DEBUG, "Schmidl-Cox CFO: phase=%.3f rad, cfo=%.1f Hz (max ±%.1f Hz)",
              phase, cfo_hz, max_cfo);

    return cfo_hz;
}

// =============================================================================
// CFO ESTIMATION FROM TRAINING SYMBOLS (for chirp-sync mode)
// =============================================================================
//
// Uses cyclic prefix correlation to estimate CFO.
// CP is a copy of the last part of the symbol, so correlation gives CFO phase.
// This method is robust to frequency-selective fading (unlike carrier comparison).
//
float OFDMDemodulator::Impl::estimateCFOFromTraining(const float* samples, size_t num_symbols) {
    if (num_symbols < 1) {
        return 0.0f;
    }

    size_t fft_len = config.fft_size;
    size_t cp_len = config.getCyclicPrefix();
    size_t sym_len = symbol_samples;  // CP + FFT + guard

    // Convert to analytic signal for complex correlation
    auto analytic = toAnalytic(samples, sym_len);

    // Cyclic prefix correlation:
    // CP (first cp_len samples) should match end of FFT (last cp_len samples)
    // CP starts at: 0
    // Matching portion starts at: cp_len + (fft_len - cp_len) = fft_len
    //
    // With CFO, there's a phase difference: exp(j * 2π * cfo * fft_len / fs)
    Complex P(0.0f, 0.0f);
    for (size_t i = 0; i < cp_len; ++i) {
        // CP sample at position i
        // Corresponding FFT sample at position fft_len + i (end of symbol)
        // But symbol layout is: [CP | FFT | guard]
        // So CP sample is at index i, FFT end sample is at index cp_len + fft_len - cp_len + i = fft_len + i
        // Wait, that's wrong. Let me think again...
        //
        // Symbol layout: [CP (cp_len) | FFT body (fft_len - cp_len) | end (cp_len) | guard]
        // No wait, CP is copy of END of FFT:
        // FFT output: [0...fft_len-1]
        // CP = FFT[fft_len-cp_len ... fft_len-1]
        // TX symbol: [CP | FFT]
        //
        // So in received symbol:
        // - CP is at indices [0, cp_len)
        // - Matching FFT portion is at indices [fft_len, fft_len + cp_len) = [cp_len + fft_len - cp_len, cp_len + fft_len)
        //   Wait, that's also wrong.
        //
        // TX symbol: CP (cp_len samples) + full FFT (fft_len samples) = cp_len + fft_len total
        // CP = last cp_len samples of FFT output
        // So in received symbol (ignoring guard):
        // - CP at indices [0, cp_len)
        // - Full FFT at indices [cp_len, cp_len + fft_len)
        // - The END of FFT (which matches CP) is at indices [cp_len + fft_len - cp_len, cp_len + fft_len) = [fft_len, cp_len + fft_len)
        //
        // Correlation: conj(CP[i]) * FFT_end[i] where FFT_end[i] = sample[fft_len + i]

        if (i < analytic.size() && fft_len + i < analytic.size()) {
            P += std::conj(analytic[i]) * analytic[fft_len + i];
        }
    }

    // Phase is due to CFO over fft_len samples
    float phase = std::atan2(P.imag(), P.real());

    // CFO = phase × fs / (2π × fft_len)
    float cfo_hz = phase * config.sample_rate / (2.0f * M_PI * fft_len);

    // Clamp to reasonable range (±50 Hz for typical HF radios)
    cfo_hz = std::max(-50.0f, std::min(50.0f, cfo_hz));

    LOG_SYNC(INFO, "CFO from CP correlation: phase=%.3f rad, CFO=%.1f Hz",
             phase, cfo_hz);

    return cfo_hz;
}

// =============================================================================
// LTS FINE TIMING
// =============================================================================

size_t OFDMDemodulator::Impl::refineLTSTiming(size_t coarse_sts_start) {
    // Expected LTS position: after 4 STS symbols
    size_t preamble_sym_len = config.fft_size + config.getCyclicPrefix();
    size_t coarse_lts_start = coarse_sts_start + 4 * preamble_sym_len;

    // Search window: cover Schmidl-Cox plateau uncertainty
    int SEARCH_BACK = 3 * preamble_sym_len;
    int SEARCH_FWD = preamble_sym_len / 2;

    if (coarse_lts_start < (size_t)SEARCH_BACK ||
        coarse_lts_start + SEARCH_FWD + lts_passband_I.size() > rx_buffer.size()) {
        LOG_SYNC(DEBUG, "LTS refinement: not enough data, using coarse timing");
        return coarse_lts_start;
    }

    // PASSBAND correlation for phase-invariant timing
    float best_corr = 0.0f;
    size_t best_offset = coarse_lts_start;

    // Pre-calculate reference template energy
    float energy_ref = 0.0f;
    for (size_t i = 0; i < lts_passband_I.size(); ++i) {
        energy_ref += lts_passband_I[i] * lts_passband_I[i];
        energy_ref += lts_passband_Q[i] * lts_passband_Q[i];
    }
    energy_ref *= 0.5f;

    size_t expected_lts = 4 * preamble_sym_len;
    float corr_at_expected = 0.0f;

    for (int delta = -SEARCH_BACK; delta <= SEARCH_FWD; ++delta) {
        size_t offset = coarse_lts_start + delta;

        float corr_I = 0.0f;
        float corr_Q = 0.0f;
        float energy_rx = 0.0f;

        for (size_t i = 0; i < lts_passband_I.size(); ++i) {
            float rx_sample = rx_buffer[offset + i];
            corr_I += rx_sample * lts_passband_I[i];
            corr_Q += rx_sample * lts_passband_Q[i];
            energy_rx += rx_sample * rx_sample;
        }

        float corr_mag = std::sqrt(corr_I * corr_I + corr_Q * corr_Q);
        float norm = std::sqrt(energy_rx * energy_ref);
        float corr = (norm > 1e-6f) ? corr_mag / norm : 0.0f;

        if (corr > best_corr) {
            best_corr = corr;
            best_offset = offset;
        }

        if (offset == expected_lts) {
            corr_at_expected = corr;
        }
    }

    LOG_SYNC(DEBUG, "LTS search: expected=%zu (corr=%.3f), found=%zu (corr=%.3f), diff=%d",
             expected_lts, corr_at_expected, best_offset, best_corr,
             (int)best_offset - (int)expected_lts);

    int timing_correction = (int)best_offset - (int)coarse_lts_start;
    LOG_SYNC(DEBUG, "LTS fine timing: coarse=%zu, refined=%zu, correction=%+d samples, corr=%.3f",
             coarse_lts_start, best_offset, timing_correction, best_corr);

    // LTS correlation threshold (scales with FFT size)
    float LTS_CORRELATION_THRESHOLD = (config.fft_size >= 1024) ? 0.05f : 0.35f;
    if (best_corr < LTS_CORRELATION_THRESHOLD) {
        LOG_SYNC(WARN, "LTS correlation too low (%.3f < %.1f) - likely false sync, aborting",
                 best_corr, LTS_CORRELATION_THRESHOLD);
        return SIZE_MAX;  // Signal failure
    }

    return best_offset;
}

// =============================================================================
// DECODE HUNTING (LDPC-based timing validation)
// =============================================================================

static constexpr int HUNT_NEED_MORE_SAMPLES = -9999;

std::vector<float> OFDMDemodulator::Impl::trialDemodulate(size_t data_start_offset, size_t num_symbols) {
    std::vector<float> trial_soft_bits;

    size_t samples_needed = num_symbols * symbol_samples;
    if (data_start_offset + samples_needed > rx_buffer.size()) {
        return trial_soft_bits;
    }

    NCO trial_mixer(config.center_freq, config.sample_rate);
    std::vector<Complex> trial_channel_estimate(config.fft_size, Complex(1, 0));
    bool channel_initialized = false;
    std::vector<Complex> trial_prev_equalized;

    for (size_t sym = 0; sym < num_symbols; ++sym) {
        size_t sym_offset = data_start_offset + sym * symbol_samples;

        // Mix to baseband
        std::vector<Complex> baseband(symbol_samples);
        for (size_t i = 0; i < symbol_samples; ++i) {
            Complex osc = trial_mixer.next();
            baseband[i] = rx_buffer[sym_offset + i] * std::conj(osc);
        }

        // Extract symbol
        size_t cp_len = config.getCyclicPrefix();
        std::vector<Complex> symbol(config.fft_size);
        for (size_t i = 0; i < config.fft_size; ++i) {
            symbol[i] = baseband[cp_len + i];
        }
        std::vector<Complex> freq_domain;
        fft.forward(symbol, freq_domain);

        // Update channel estimate from pilots (simplified)
        if (!channel_initialized) {
            for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                int idx = pilot_carrier_indices[i];
                Complex rx_pilot = freq_domain[idx];
                Complex tx_pilot = pilot_sequence[i];
                if (std::abs(tx_pilot) > 1e-6f) {
                    trial_channel_estimate[idx] = rx_pilot / tx_pilot;
                }
            }
            // Interpolate to data carriers (nearest pilot)
            for (int idx : data_carrier_indices) {
                int nearest = pilot_carrier_indices[0];
                int min_dist = std::abs(idx - nearest);
                for (int pidx : pilot_carrier_indices) {
                    int dist = std::abs(idx - pidx);
                    if (dist < min_dist) {
                        min_dist = dist;
                        nearest = pidx;
                    }
                }
                trial_channel_estimate[idx] = trial_channel_estimate[nearest];
            }
            channel_initialized = true;
        }

        // Equalize data carriers
        std::vector<Complex> equalized;
        equalized.reserve(data_carrier_indices.size());
        for (int idx : data_carrier_indices) {
            Complex H = trial_channel_estimate[idx];
            float H_mag_sq = std::norm(H);
            if (H_mag_sq < 1e-6f) H_mag_sq = 1e-6f;
            Complex eq = freq_domain[idx] * std::conj(H) / H_mag_sq;
            equalized.push_back(eq);
        }

        // DQPSK demodulation
        if (config.modulation == Modulation::DQPSK) {
            if (!trial_prev_equalized.empty()) {
                for (size_t i = 0; i < equalized.size() && i < trial_prev_equalized.size(); ++i) {
                    Complex diff = equalized[i] * std::conj(trial_prev_equalized[i]);
                    float phase = std::atan2(diff.imag(), diff.real());
                    float signal_power = std::abs(equalized[i]) * std::abs(trial_prev_equalized[i]);

                    if (signal_power < 1e-6f) {
                        trial_soft_bits.push_back(0.0f);
                        trial_soft_bits.push_back(0.0f);
                    } else {
                        float scale = 2.0f * signal_power / noise_variance;
                        static const float pi = 3.14159265358979f;
                        trial_soft_bits.push_back(std::max(-10.0f, std::min(10.0f,
                            scale * std::sin(phase + pi/4))));
                        trial_soft_bits.push_back(std::max(-10.0f, std::min(10.0f,
                            scale * std::cos(2 * phase))));
                    }
                }
            }
            trial_prev_equalized = equalized;
        } else {
            // BPSK fallback
            for (const auto& eq : equalized) {
                float llr = -2.0f * eq.real() / noise_variance;
                trial_soft_bits.push_back(std::max(-10.0f, std::min(10.0f, llr)));
            }
        }
    }

    return trial_soft_bits;
}

std::pair<bool, int> OFDMDemodulator::Impl::huntForCodeword(size_t candidate_sync_pos) {
    size_t preamble_len = symbol_samples * 6;

    static const int offsets[] = {0, -50, 50, -100, 100, -150, 150};
    static const size_t num_offsets = sizeof(offsets) / sizeof(offsets[0]);

    size_t bits_per_symbol = (config.modulation == Modulation::DQPSK) ?
        (data_carrier_indices.size() * 2) : data_carrier_indices.size();
    size_t symbols_needed = (LDPC_BLOCK_SIZE + bits_per_symbol - 1) / bits_per_symbol + 1;

    size_t samples_for_hunt = candidate_sync_pos + preamble_len + 150 + symbols_needed * symbol_samples;
    if (samples_for_hunt > rx_buffer.size()) {
        LOG_SYNC(INFO, "Hunt at %zu: need %zu samples, have %zu - waiting for more",
                 candidate_sync_pos, samples_for_hunt, rx_buffer.size());
        return {false, HUNT_NEED_MORE_SAMPLES};
    }

    bool any_ldpc_attempted = false;

    for (size_t i = 0; i < num_offsets; ++i) {
        int offset = offsets[i];

        size_t nominal_data_start = candidate_sync_pos + preamble_len;
        size_t data_start;
        if (offset < 0 && static_cast<size_t>(-offset) > nominal_data_start) {
            continue;
        }
        data_start = nominal_data_start + offset;

        auto soft_bits_trial = trialDemodulate(data_start, symbols_needed);

        if (soft_bits_trial.size() < LDPC_BLOCK_SIZE) {
            LOG_SYNC(INFO, "Hunt offset %+d: only %zu soft bits (need %zu)",
                     offset, soft_bits_trial.size(), LDPC_BLOCK_SIZE);
            continue;
        }

        any_ldpc_attempted = true;

        LDPCDecoder decoder(CodeRate::R1_4);
        std::vector<float> codeword_bits(soft_bits_trial.begin(),
                                          soft_bits_trial.begin() + LDPC_BLOCK_SIZE);
        auto decoded = decoder.decodeSoft(codeword_bits);

        if (decoded.empty()) {
            LOG_SYNC(INFO, "Hunt offset %+d: LDPC decode failed", offset);
            continue;
        }

        if (decoded.size() >= 2) {
            uint16_t magic = (static_cast<uint16_t>(decoded[0]) << 8) | decoded[1];
            LOG_SYNC(INFO, "Hunt offset %+d: LDPC OK, magic=0x%04X (bytes: %02X %02X %02X %02X)",
                     offset, magic, decoded[0], decoded[1],
                     decoded.size() > 2 ? decoded[2] : 0,
                     decoded.size() > 3 ? decoded[3] : 0);
            if (magic == 0x554C) {  // "UL" - v2 frame magic
                LOG_SYNC(INFO, "Hunt SUCCESS at offset %+d: valid v2 frame magic found!", offset);
                return {true, offset};
            }
        }
    }

    if (!any_ldpc_attempted) {
        LOG_SYNC(DEBUG, "Hunt at %zu: no LDPC attempts made (insufficient samples at all offsets)", candidate_sync_pos);
        return {false, HUNT_NEED_MORE_SAMPLES};
    }

    LOG_SYNC(DEBUG, "Hunt FAILED: no valid codeword found at any offset");
    return {false, 0};
}

} // namespace ultra
