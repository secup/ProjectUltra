// Channel Estimation and Equalization for OFDM
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <cmath>
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "ultra/logging.hpp"
#include <algorithm>

namespace ultra {

using namespace demod_constants;

// =============================================================================
// BASEBAND CONVERSION
// =============================================================================

std::vector<Complex> OFDMDemodulator::Impl::toBaseband(SampleSpan samples) {
    std::vector<Complex> baseband(samples.size());

    // Phase increment per sample for frequency correction
    float phase_increment = -2.0f * M_PI * freq_offset_hz / config.sample_rate;

    for (size_t i = 0; i < samples.size(); ++i) {
        Complex osc = mixer.next();
        Complex mixed = samples[i] * std::conj(osc);

        // Apply frequency offset correction
        if (std::abs(freq_offset_hz) > 0.01f) {
            Complex correction(std::cos(freq_correction_phase),
                               std::sin(freq_correction_phase));
            mixed *= correction;
            freq_correction_phase += phase_increment;

            // Wrap phase
            if (freq_correction_phase > M_PI) {
                freq_correction_phase -= 2.0f * M_PI;
            } else if (freq_correction_phase < -M_PI) {
                freq_correction_phase += 2.0f * M_PI;
            }
        }

        baseband[i] = mixed;
    }
    return baseband;
}

std::vector<Complex> OFDMDemodulator::Impl::extractSymbol(const std::vector<Complex>& baseband, size_t offset) {
    size_t start = offset + config.getCyclicPrefix();

    std::vector<Complex> symbol(config.fft_size);
    for (size_t i = 0; i < config.fft_size && (start + i) < baseband.size(); ++i) {
        symbol[i] = baseband[start + i];
    }

    std::vector<Complex> freq;
    fft.forward(symbol, freq);

    return freq;
}

// =============================================================================
// CHANNEL ESTIMATION
// =============================================================================

void OFDMDemodulator::Impl::updateChannelEstimate(const std::vector<Complex>& freq_domain) {
    // For FIRST symbol after sync, use pilot estimate directly (no smoothing)
    // If snr_symbol_count > 0, we have a pre-existing estimate (from training or previous symbols)
    float alpha = (snr_symbol_count == 0) ? 1.0f : 0.9f;

    // First pass: compute all LS estimates and their average
    std::vector<Complex> h_ls_all(pilot_carrier_indices.size());
    Complex h_sum(0, 0);

    for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
        int idx = pilot_carrier_indices[i];
        Complex rx = freq_domain[idx];
        Complex tx = pilot_sequence[i];
        h_ls_all[i] = rx / tx;
        h_sum += h_ls_all[i];
    }

    // Carrier phase recovery: compute average phase offset on first symbol
    if (!carrier_phase_initialized && !pilot_carrier_indices.empty()) {
        Complex h_avg = h_sum / float(pilot_carrier_indices.size());
        float avg_mag = std::abs(h_avg);
        if (avg_mag > 0.01f) {
            carrier_phase_correction = std::conj(h_avg) / avg_mag;
            carrier_phase_initialized = true;
            LOG_DEMOD(DEBUG, "Carrier phase recovery: avg_phase=%.1f°, applying correction",
                      std::arg(h_avg) * 180.0f / M_PI);
        }
    }

    // Apply carrier phase correction to all H estimates
    for (size_t i = 0; i < h_ls_all.size(); ++i) {
        h_ls_all[i] *= carrier_phase_correction;
    }
    h_sum *= carrier_phase_correction;

    // DEBUG: Log first symbol's pilot analysis
    if (soft_bits.empty()) {
        LOG_DEMOD(DEBUG, "=== First symbol pilot analysis ===");
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            LOG_DEMOD(DEBUG, "Pilot[%zu] idx=%d: tx=(%.1f,%.1f) rx=(%.2f,%.2f) H=(%.2f,%.2f) |H|=%.2f phase=%.1f°",
                      i, idx,
                      pilot_sequence[i].real(), pilot_sequence[i].imag(),
                      freq_domain[idx].real(), freq_domain[idx].imag(),
                      h_ls_all[i].real(), h_ls_all[i].imag(),
                      std::abs(h_ls_all[i]),
                      std::arg(h_ls_all[i]) * 180.0f / M_PI);
        }
        LOG_DEMOD(DEBUG, "H avg: (%.2f,%.2f), |H|=%.2f, phase=%.1f deg",
                  (h_sum / float(pilot_carrier_indices.size())).real(),
                  (h_sum / float(pilot_carrier_indices.size())).imag(),
                  std::abs(h_sum / float(pilot_carrier_indices.size())),
                  std::arg(h_sum / float(pilot_carrier_indices.size())) * 180.0f / M_PI);
    }

    // Compute average signal power from pilots
    float signal_power_sum = 0.0f;
    for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
        signal_power_sum += std::norm(h_ls_all[i]);
    }
    float signal_power = signal_power_sum / pilot_carrier_indices.size();

    // Measure noise using TEMPORAL comparison
    float noise_power_sum = 0.0f;
    size_t noise_count = 0;

    for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
        int idx = pilot_carrier_indices[i];

        if (!prev_pilot_phases.empty() && i < prev_pilot_phases.size()) {
            Complex prev_h = prev_pilot_phases[i];
            Complex curr_h = h_ls_all[i];

            if (std::norm(prev_h) > 1e-6f && std::norm(curr_h) > 1e-6f) {
                Complex diff = curr_h - prev_h;
                noise_power_sum += std::norm(diff);
                noise_count++;
            }
        }

        // Update smoothed channel estimate
        Complex h_old = channel_estimate[idx];
        channel_estimate[idx] = alpha * h_ls_all[i] + (1.0f - alpha) * h_old;
    }

    // First symbol fallback: assume 15 dB SNR
    if (noise_count == 0) {
        noise_power_sum = signal_power / DEFAULT_SNR_LINEAR;
        noise_count = 1;
    }

    // === Frequency offset estimation from pilot phase differences ===
    if (!prev_pilot_phases.empty() && prev_pilot_phases.size() == h_ls_all.size()) {
        Complex phase_diff_sum(0, 0);
        int valid_count = 0;

        for (size_t i = 0; i < h_ls_all.size(); ++i) {
            Complex diff = h_ls_all[i] * std::conj(prev_pilot_phases[i]);

            if (std::norm(prev_pilot_phases[i]) > 1e-6f &&
                std::norm(h_ls_all[i]) > 1e-6f) {
                float mag = std::abs(diff);
                if (mag > 1e-6f) {
                    phase_diff_sum += diff / mag;
                    valid_count++;
                }
            }
        }

        if (valid_count > 0) {
            Complex avg_diff = phase_diff_sum / static_cast<float>(valid_count);
            float avg_phase_diff = std::atan2(avg_diff.imag(), avg_diff.real());

            pilot_phase_correction = Complex(std::cos(-avg_phase_diff), std::sin(-avg_phase_diff));

            float symbol_duration = static_cast<float>(config.getSymbolDuration()) /
                                   static_cast<float>(config.sample_rate);
            float residual_cfo = avg_phase_diff / (2.0f * M_PI * symbol_duration);
            float total_cfo = freq_offset_hz + residual_cfo;

            // Adaptive alpha for CFO tracking
            float adaptive_alpha = FREQ_OFFSET_ALPHA;
            if (symbols_since_sync < CFO_ACQUISITION_SYMBOLS) {
                float progress = static_cast<float>(symbols_since_sync) / CFO_ACQUISITION_SYMBOLS;
                adaptive_alpha = 0.9f * (1.0f - progress) + FREQ_OFFSET_ALPHA * progress;
            }
            if (std::abs(residual_cfo) > 10.0f) {
                adaptive_alpha = std::max(adaptive_alpha, 0.9f);
            }
            symbols_since_sync++;

            freq_offset_filtered = adaptive_alpha * total_cfo +
                                  (1.0f - adaptive_alpha) * freq_offset_filtered;

            freq_offset_hz = std::max(-MAX_CFO_HZ, std::min(MAX_CFO_HZ, freq_offset_filtered));

            LOG_DEMOD(TRACE, "Freq offset: residual=%.2f Hz, total=%.2f Hz, filtered=%.2f Hz",
                     residual_cfo, total_cfo, freq_offset_hz);
        }
    } else {
        pilot_phase_correction = Complex(1, 0);
    }

    // === Symbol timing recovery from pilot phase slope ===
    if (snr_symbol_count >= 3) {
        float sum_k = 0, sum_k2 = 0, sum_phase = 0, sum_k_phase = 0;
        int timing_valid_count = 0;

        for (size_t i = 0; i < h_ls_all.size(); ++i) {
            if (std::norm(h_ls_all[i]) < 1e-6f) continue;

            int k = pilot_carrier_indices[i];
            if (k > (int)config.fft_size / 2) k -= config.fft_size;

            float phase = std::arg(h_ls_all[i]);

            sum_k += k;
            sum_k2 += k * k;
            sum_phase += phase;
            sum_k_phase += k * phase;
            timing_valid_count++;
        }

        if (timing_valid_count >= 3) {
            float n = timing_valid_count;
            float denom = n * sum_k2 - sum_k * sum_k;
            if (std::abs(denom) > 1e-6f) {
                float slope = (n * sum_k_phase - sum_k * sum_phase) / denom;
                float instantaneous_timing = slope * config.fft_size / (2.0f * M_PI);

                timing_offset_samples = TIMING_ALPHA * instantaneous_timing +
                                       (1.0f - TIMING_ALPHA) * timing_offset_samples;

                float max_timing = 50.0f * (config.fft_size / 512.0f);
                timing_offset_samples = std::max(-max_timing, std::min(max_timing, timing_offset_samples));

                LOG_DEMOD(DEBUG, "Timing recovery: instant=%.2f samp, filtered=%.2f samp (slope=%.4f rad/bin)",
                         instantaneous_timing, timing_offset_samples, slope);
            }
        }
    }

    // Store current pilots for next symbol
    prev_pilot_phases = h_ls_all;

    // === COHERENT MODE FIX: Remove/add timing phase for correct interpolation ===
    bool is_coherent = (config.modulation != Modulation::DBPSK &&
                       config.modulation != Modulation::DQPSK &&
                       config.modulation != Modulation::D8PSK);

    if (is_coherent && snr_symbol_count < 3) {
        LOG_DEMOD(INFO, "Coherent timing fix: is_coherent=%d, timing_offset=%.2f, sym=%d",
                  is_coherent ? 1 : 0, timing_offset_samples, snr_symbol_count);
    }

    // Step 1: Remove timing phase from pilots before interpolation
    if (is_coherent && std::abs(timing_offset_samples) > 0.1f) {
        for (int idx : pilot_carrier_indices) {
            int k = idx;
            if (k > (int)config.fft_size / 2) k -= config.fft_size;
            float timing_phase = 2.0f * M_PI * k * timing_offset_samples / config.fft_size;
            Complex timing_removal = std::exp(Complex(0, -timing_phase));

            if (snr_symbol_count == 0 && k >= -2 && k <= 3) {
                Complex before = channel_estimate[idx];
                Complex after = before * timing_removal;
                LOG_DEMOD(DEBUG, "TimingFix k=%d: before=(%.2f,%.2f) phase=%.1f° "
                          "remove=%.1f° after=(%.2f,%.2f) phase=%.1f°",
                          k, before.real(), before.imag(),
                          std::arg(before) * 180.0f / M_PI,
                          -timing_phase * 180.0f / M_PI,
                          after.real(), after.imag(),
                          std::arg(after) * 180.0f / M_PI);
            }

            channel_estimate[idx] *= timing_removal;
        }
    }

    // Interpolate between pilots
    interpolateChannel();

    // Step 2: Add back correct timing phase for each carrier
    if (is_coherent && std::abs(timing_offset_samples) > 0.1f) {
        for (int idx : pilot_carrier_indices) {
            int k = idx;
            if (k > (int)config.fft_size / 2) k -= config.fft_size;
            float timing_phase = 2.0f * M_PI * k * timing_offset_samples / config.fft_size;
            Complex timing_correction = std::exp(Complex(0, timing_phase));
            channel_estimate[idx] *= timing_correction;
        }
        for (int idx : data_carrier_indices) {
            int k = idx;
            if (k > (int)config.fft_size / 2) k -= config.fft_size;
            float timing_phase = 2.0f * M_PI * k * timing_offset_samples / config.fft_size;
            Complex timing_correction = std::exp(Complex(0, timing_phase));
            channel_estimate[idx] *= timing_correction;
        }
    }

    // Initialize adaptive equalizer weights from pilot-based estimate
    if (config.adaptive_eq_enabled) {
        for (int idx : data_carrier_indices) {
            if (snr_symbol_count < 3) {
                lms_weights[idx] = channel_estimate[idx];
            }
        }
        for (int idx : pilot_carrier_indices) {
            if (snr_symbol_count < 3) {
                lms_weights[idx] = channel_estimate[idx];
            }
        }
    }

    // Update noise variance and SNR
    if (noise_count > 1 && noise_power_sum > 0.0f) {
        noise_variance = noise_power_sum / (noise_count - 1);
        if (noise_variance < 1e-6f) noise_variance = 1e-6f;

        float instantaneous_snr = signal_power / noise_variance;
        instantaneous_snr = std::max(0.1f, std::min(10000.0f, instantaneous_snr));

        estimated_snr_linear = snr_alpha * instantaneous_snr + (1.0f - snr_alpha) * estimated_snr_linear;
    }

    snr_symbol_count++;
}

// =============================================================================
// CHANNEL INTERPOLATION
// =============================================================================

void OFDMDemodulator::Impl::interpolateChannel() {
    for (size_t dc = 0; dc < interp_table.size(); ++dc) {
        const auto& info = interp_table[dc];
        if (info.lower_pilot >= 0 && info.upper_pilot >= 0) {
            Complex H1 = channel_estimate[info.lower_pilot];
            Complex H2 = channel_estimate[info.upper_pilot];

            // Check phase difference between pilots
            Complex phase_diff_complex = H2 * std::conj(H1);
            float phase_diff = std::abs(std::atan2(phase_diff_complex.imag(), phase_diff_complex.real()));

            // If phase difference > 90°, use nearest pilot
            if (phase_diff > PHASE_INTERPOLATION_THRESHOLD) {
                if (info.alpha < 0.5f) {
                    channel_estimate[info.fft_idx] = H1;
                } else {
                    channel_estimate[info.fft_idx] = H2;
                }
                LOG_DEMOD(DEBUG, "Nearest-pilot for bin %d: phase_diff=%.1f°",
                          info.fft_idx, phase_diff * 180.0f / 3.14159f);
            } else {
                // Linear interpolation
                channel_estimate[info.fft_idx] = (1.0f - info.alpha) * H1 + info.alpha * H2;
            }
        } else if (info.lower_pilot >= 0) {
            channel_estimate[info.fft_idx] = channel_estimate[info.lower_pilot];
        } else if (info.upper_pilot >= 0) {
            channel_estimate[info.fft_idx] = channel_estimate[info.upper_pilot];
        }
    }
}

// =============================================================================
// HARD DECISION SLICER
// =============================================================================

Complex OFDMDemodulator::Impl::hardDecision(Complex sym, Modulation mod) const {
    switch (mod) {
        case Modulation::BPSK:
            return Complex(sym.real() > 0 ? 1.0f : -1.0f, 0);

        case Modulation::QPSK: {
            float I = sym.real() > 0 ? 0.7071f : -0.7071f;
            float Q = sym.imag() > 0 ? 0.7071f : -0.7071f;
            return Complex(I, Q);
        }

        case Modulation::QAM16: {
            auto slice = [](float x) -> float {
                if (x < -0.4f) return -0.9487f;
                if (x < 0.0f) return -0.3162f;
                if (x < 0.4f) return 0.3162f;
                return 0.9487f;
            };
            return Complex(slice(sym.real()), slice(sym.imag()));
        }

        case Modulation::QAM32: {
            auto slice_i = [](float x) -> float {
                constexpr float d = QAM32_SCALE;
                if (x < -2*d) return -3*d;
                if (x < 0) return -d;
                if (x < 2*d) return d;
                return 3*d;
            };
            auto slice_q = [](float x) -> float {
                constexpr float d = QAM32_SCALE;
                if (x < -6*d) return -7*d;
                if (x < -4*d) return -5*d;
                if (x < -2*d) return -3*d;
                if (x < 0) return -d;
                if (x < 2*d) return d;
                if (x < 4*d) return 3*d;
                if (x < 6*d) return 5*d;
                return 7*d;
            };
            return Complex(slice_i(sym.real()), slice_q(sym.imag()));
        }

        case Modulation::QAM64: {
            auto slice = [](float x) -> float {
                constexpr float d = 0.1543f;
                if (x < -6*d) return -7*d;
                if (x < -4*d) return -5*d;
                if (x < -2*d) return -3*d;
                if (x < 0) return -d;
                if (x < 2*d) return d;
                if (x < 4*d) return 3*d;
                if (x < 6*d) return 5*d;
                return 7*d;
            };
            return Complex(slice(sym.real()), slice(sym.imag()));
        }

        default:
            return Complex(sym.real() > 0 ? 0.7071f : -0.7071f,
                          sym.imag() > 0 ? 0.7071f : -0.7071f);
    }
}

// =============================================================================
// ADAPTIVE EQUALIZER UPDATES
// =============================================================================

void OFDMDemodulator::Impl::lmsUpdate(int idx, Complex received, Complex reference) {
    float mu = config.lms_mu;
    Complex error = received - lms_weights[idx] * reference;
    lms_weights[idx] += mu * std::conj(reference) * error;
}

void OFDMDemodulator::Impl::rlsUpdate(int idx, Complex received, Complex reference) {
    float lambda = config.rls_lambda;
    float P = rls_P[idx];
    float ref_norm = std::norm(reference);

    float k = P / (lambda + P * ref_norm);
    Complex error = received - lms_weights[idx] * reference;

    lms_weights[idx] += k * std::conj(reference) * error;
    rls_P[idx] = (P - k * ref_norm * P) / lambda;
    rls_P[idx] = std::max(ADAPTIVE_EQ_P_MIN, std::min(ADAPTIVE_EQ_P_MAX, rls_P[idx]));
}

// =============================================================================
// EQUALIZATION
// =============================================================================

std::vector<Complex> OFDMDemodulator::Impl::equalize(const std::vector<Complex>& freq_domain, Modulation mod) {
    std::vector<Complex> equalized(data_carrier_indices.size());
    carrier_noise_var.resize(data_carrier_indices.size());

    // For differential modulation, skip equalization entirely
    bool is_differential = (mod == Modulation::DBPSK || mod == Modulation::DQPSK || mod == Modulation::D8PSK);

    if (is_differential) {
        // Return raw symbols with phase/timing corrections
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];

            int k = idx;
            if (k > (int)config.fft_size / 2) k -= config.fft_size;

            float timing_phase = 2.0f * M_PI * k * timing_offset_samples / config.fft_size;
            Complex timing_correction = std::exp(Complex(0, timing_phase));

            equalized[i] = freq_domain[idx] * pilot_phase_correction * timing_correction;

            float h_power = std::norm(channel_estimate[idx]);
            if (h_power < 1e-6f) {
                carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
            } else {
                carrier_noise_var[i] = noise_variance / h_power;
                carrier_noise_var[i] = std::max(MIN_CARRIER_NOISE_VAR, std::min(MAX_CARRIER_NOISE_VAR, carrier_noise_var[i]));
            }
        }
        return equalized;
    }

    bool use_adaptive = config.adaptive_eq_enabled;

    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        int idx = data_carrier_indices[i];
        Complex received = freq_domain[idx];

        if (use_adaptive) {
            Complex h = lms_weights[idx];
            float h_power = std::norm(h);

            // MMSE equalization
            float mmse_denom = h_power + noise_variance;
            if (mmse_denom < 1e-10f) {
                equalized[i] = Complex(0, 0);
                carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
            } else {
                equalized[i] = std::conj(h) * received / mmse_denom;
                carrier_noise_var[i] = noise_variance / (h_power + 1e-6f);
            }

            // Decision-directed update
            if (config.decision_directed) {
                Complex decision = hardDecision(equalized[i], mod);

                if (config.adaptive_eq_use_rls) {
                    rlsUpdate(idx, received, decision);
                } else {
                    lmsUpdate(idx, received, decision);
                }

                last_decisions[idx] = decision;
            }
        } else {
            // MMSE equalization with pilot-based channel estimate
            Complex h = channel_estimate[idx];
            float h_power = std::norm(h);

            float mmse_denom = h_power + noise_variance;
            if (mmse_denom < 1e-10f) {
                equalized[i] = Complex(0, 0);
                carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
            } else {
                equalized[i] = std::conj(h) * received / mmse_denom;
                carrier_noise_var[i] = noise_variance / (h_power + 1e-6f);
                carrier_noise_var[i] = std::max(MIN_CARRIER_NOISE_VAR, std::min(MAX_CARRIER_NOISE_VAR, carrier_noise_var[i]));
            }
        }
    }

    // Detect deep fades and apply soft erasure
    float avg_h_power = 0.0f;
    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        int idx = data_carrier_indices[i];
        avg_h_power += std::norm(channel_estimate[idx]);
    }
    avg_h_power /= data_carrier_indices.size();

    float fade_threshold = FADE_THRESHOLD_RATIO * avg_h_power;
    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        int idx = data_carrier_indices[i];
        float h_power = std::norm(channel_estimate[idx]);
        if (h_power < fade_threshold) {
            carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;  // Soft erasure
        }
    }

    return equalized;
}

std::vector<Complex> OFDMDemodulator::Impl::equalize(const std::vector<Complex>& freq_domain) {
    return equalize(freq_domain, config.modulation);
}

} // namespace ultra
