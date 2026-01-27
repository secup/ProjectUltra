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

    // Log CFO correction on first symbol (when phase is near zero)
    if (std::abs(freq_correction_phase) < 0.01f && std::abs(freq_offset_hz) > 0.01f) {
        LOG_DEMOD(INFO, "toBaseband: CFO=%.2f Hz, phase_inc=%.6f rad/sample, samples=%zu",
                  freq_offset_hz, phase_increment, samples.size());
    }

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

void OFDMDemodulator::Impl::estimateChannelFromLTS(const float* training_samples, size_t num_symbols) {
    // Estimate channel response from LTS (Long Training Sequence) symbols
    // This is used by processPresynced() for chirp-synced modes where we have
    // training symbols for initial channel estimation.
    //
    // The LTS carries:
    //   - sync_sequence on data carriers
    //   - pilot_sequence on pilot carriers (if use_pilots=true)
    //
    // We estimate H for BOTH data and pilot carriers so that subsequent
    // updateChannelEstimate() calls can use pilots for tracking.
    //
    // We average over multiple training symbols for robustness.
    // We also estimate noise variance from the variance of H estimates.

    LOG_DEMOD(DEBUG, "estimateChannelFromLTS: num_symbols=%zu, symbol_samples=%zu, first_sample=%.6f",
             num_symbols, symbol_samples, training_samples[0]);

    // DEBUG: Print carrier indices
    fprintf(stderr, "[LTS-DBG] RX carrier indices (first 5): ");
    for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
        fprintf(stderr, "%d ", data_carrier_indices[i]);
    }
    fprintf(stderr, "(total %zu)\n", data_carrier_indices.size());

    if (num_symbols == 0 || data_carrier_indices.empty()) return;

    // Store per-symbol channel estimates for noise estimation (data carriers only for now)
    std::vector<std::vector<Complex>> h_per_symbol(num_symbols);
    for (auto& v : h_per_symbol) v.resize(data_carrier_indices.size());

    // Accumulate channel estimates from each training symbol
    std::vector<Complex> h_sum_data(data_carrier_indices.size(), Complex(0, 0));
    std::vector<Complex> h_sum_pilot(pilot_carrier_indices.size(), Complex(0, 0));
    size_t valid_symbols = 0;

    // Process each training symbol using the main mixer (it will be advanced)
    const float* ptr = training_samples;
    for (size_t sym = 0; sym < num_symbols; ++sym) {
        // Use toBaseband and extractSymbol like normal demodulation
        SampleSpan sym_span(ptr, symbol_samples);
        auto baseband = toBaseband(sym_span);
        auto freq = extractSymbol(baseband, 0);

        // DEBUG: Print first few freq domain values on first training symbol
        if (sym == 0) {
            fprintf(stderr, "[LTS-DBG] RX freq[idx] for first 5 carriers: ");
            for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
                int idx = data_carrier_indices[i];
                fprintf(stderr, "[%d]=(%.3f,%.3f) ", idx, freq[idx].real(), freq[idx].imag());
            }
            fprintf(stderr, "\n");
            fprintf(stderr, "[LTS-DBG] TX sync_seq for first 5 carriers: ");
            for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
                Complex tx = sync_sequence[i % sync_sequence.size()];
                fprintf(stderr, "(%.3f,%.3f) ", tx.real(), tx.imag());
            }
            fprintf(stderr, "\n");
        }

        // Estimate H for each data carrier
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            Complex rx = freq[idx];
            Complex tx = sync_sequence[i % sync_sequence.size()];

            // H = rx / tx (LS estimate)
            if (std::abs(tx) > 0.01f) {
                Complex h_ls = rx / tx;
                h_sum_data[i] += h_ls;
                h_per_symbol[sym][i] = h_ls;
            }
        }

        // Estimate H for each pilot carrier (LTS includes pilots!)
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            Complex rx = freq[idx];
            Complex tx = pilot_sequence[i];  // Pilots use pilot_sequence, not sync_sequence

            // H = rx / tx (LS estimate)
            if (std::abs(tx) > 0.01f) {
                Complex h_ls = rx / tx;
                h_sum_pilot[i] += h_ls;
            }

            // DEBUG: Log raw pilot values for each training symbol
            if (sym == 0 && i < 4) {
                LOG_DEMOD(DEBUG, "LTS sym=%zu pilot[%zu] idx=%d: rx=(%.4f,%.4f) |rx|=%.4f tx=(%.1f,%.1f)",
                         sym, i, idx, rx.real(), rx.imag(), std::abs(rx), tx.real(), tx.imag());
            }
        }

        valid_symbols++;
        ptr += symbol_samples;
    }

    if (valid_symbols == 0) return;

    // For data carriers: use first symbol's H estimate instead of averaging
    // Averaging causes magnitude reduction when CFO/timing causes phase rotation between symbols
    // Using just the first symbol avoids this magnitude cancellation problem
    if (valid_symbols > 0) {
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            // Use first symbol's H estimate (avoid magnitude cancellation from averaging)
            channel_estimate[idx] = h_per_symbol[0][i];
        }
    }

    // Average and store channel estimate for pilot carriers
    if (valid_symbols > 0) {
        float inv_count = 1.0f / valid_symbols;
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            channel_estimate[idx] = h_sum_pilot[i] * inv_count;
        }
    }

    LOG_DEMOD(INFO, "LTS channel estimate: %zu data + %zu pilot carriers",
              data_carrier_indices.size(), pilot_carrier_indices.size());

    // For coherent modes with pilots, skip noise/CFO estimation from LTS.
    // The pilot tracking in updateChannelEstimate() will handle these.
    // LTS gives us initial channel estimate; pilots refine it per symbol.
    //
    // Note: For differential modes (DQPSK) without pilots, the old noise/CFO
    // estimation code could be re-enabled, but differential modes don't need
    // as precise channel tracking anyway.

    // Compute average channel response for logging
    Complex h_avg(0, 0);
    float h_mag_sum = 0;
    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        int idx = data_carrier_indices[i];
        h_avg += channel_estimate[idx];
        h_mag_sum += std::abs(channel_estimate[idx]);
    }
    h_avg /= float(data_carrier_indices.size());
    float h_mag_avg = h_mag_sum / data_carrier_indices.size();

    // Estimate SNR from channel estimate and noise
    if (h_mag_avg > 1e-6f && noise_variance > 1e-10f) {
        float signal_power = h_mag_avg * h_mag_avg;
        estimated_snr_linear = signal_power / noise_variance;
        estimated_snr_linear = std::max(0.1f, std::min(10000.0f, estimated_snr_linear));
        LOG_DEMOD(INFO, "LTS SNR estimate: %.1f dB",
                  10.0f * std::log10(estimated_snr_linear));
    }

    LOG_DEMOD(INFO, "LTS channel estimate: %zu symbols, |H|_avg=%.3f, phase_avg=%.1f°",
              valid_symbols, h_mag_avg, std::arg(h_avg) * 180.0f / M_PI);

    // DEBUG: Print first few channel estimates
    fprintf(stderr, "[LTS-DBG] First 5 H estimates: ");
    for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
        int idx = data_carrier_indices[i];
        fprintf(stderr, "(%.3f,%.3f) ", channel_estimate[idx].real(), channel_estimate[idx].imag());
    }
    fprintf(stderr, "\n");

    // === DQPSK PER-CARRIER PHASE REFERENCES ===
    // With CFO and timing errors, different carriers have different phase offsets in the
    // H estimates. The key insight is that the EQUALIZED training symbol captures all these
    // phase errors, and using it as the DQPSK reference will cancel them in differential decoding.
    //
    // Derivation:
    //   1. Training: RX_train = sync_seq × H
    //   2. H_est = RX_train / sync_seq = H (but with timing error, H_est = H × e^{jφ})
    //   3. Equalized training: eq_train = RX_train / H_est = sync_seq × e^{-jφ}
    //   4. First data: TX_data = sync_seq × DQPSK_sym, RX_data = TX_data × H
    //   5. Equalized data: eq_data = RX_data / H_est = sync_seq × DQPSK_sym × e^{-jφ}
    //   6. Differential: diff = eq_data × conj(eq_train)
    //      = sync_seq × DQPSK_sym × e^{-jφ} × conj(sync_seq × e^{-jφ})
    //      = DQPSK_sym  ✓  (phase errors cancel!)
    //
    // If we use sync_sequence directly (without phase error):
    //      diff = sync_seq × DQPSK_sym × e^{-jφ} × conj(sync_seq)
    //      = DQPSK_sym × e^{-jφ}  ✗  (phase error remains!)
    //
    // Therefore, we store the ACTUAL equalized training symbol, not sync_sequence.

    lts_carrier_phases.resize(data_carrier_indices.size());

    // Reprocess the LAST training symbol to get the equalized values
    // (This is what we want as the DQPSK reference for the first data symbol)
    // Note: The mixer has already advanced through all training symbols, so we need
    // to use the freq values from the last symbol that we processed in the loop above.
    // We already have h_per_symbol which contains the raw freq values for each symbol.
    // To get equalized: eq[i] = RX[i] / H_est[i] = RX[i] × conj(H) / |H|²

    // Actually, we can compute it directly from the last training symbol's freq values
    // and the channel estimate. But h_per_symbol stores H estimates (RX/TX), not raw RX.
    // So: RX = H_est × TX = h_per_symbol × sync_sequence
    // And: eq = RX / H_est = sync_seq (ideally)
    // But with phase errors in H_est: eq = sync_seq × e^{-jφ}
    // This is exactly: eq = sync_seq × conj(H)/|H| when H has phase φ

    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        int idx = data_carrier_indices[i];
        Complex h = channel_estimate[idx];
        Complex tx = sync_sequence[i % sync_sequence.size()];

        // The equalized training symbol is: eq = RX / H = (TX × H) / H
        // With timing error: H_est has phase error φ, so:
        //   eq = (TX × H_true) / H_est = TX × H_true / (H_true × e^{jφ}) = TX × e^{-jφ}
        //   eq = sync_seq[i] × e^{-jφ} = sync_seq[i] × conj(H) / |H|  (since H has phase φ)
        if (std::abs(h) > 1e-6f) {
            Complex h_unit = h / std::abs(h);
            // eq_train = sync_seq × e^{-j×arg(H)} = sync_seq × conj(h_unit)
            lts_carrier_phases[i] = tx * std::conj(h_unit);
        } else {
            lts_carrier_phases[i] = tx;  // Fallback to sync_sequence
        }
    }

    // Also compute a single phase offset for backwards compatibility
    // (used if lts_carrier_phases is empty)
    Complex avg_h(0, 0);
    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        int idx = data_carrier_indices[i];
        avg_h += channel_estimate[idx] / std::abs(channel_estimate[idx] + Complex(1e-10f, 0));
    }
    avg_h /= static_cast<float>(data_carrier_indices.size());
    lts_phase_offset = avg_h / std::abs(avg_h + Complex(1e-10f, 0));

    fprintf(stderr, "[LTS-DBG] DQPSK eq_train phases (first 5): ");
    for (size_t i = 0; i < std::min(size_t(5), lts_carrier_phases.size()); ++i) {
        fprintf(stderr, "%.0f° ", std::arg(lts_carrier_phases[i]) * 180.0f / M_PI);
    }
    fprintf(stderr, "(H avg phase=%.0f°)\n", std::arg(lts_phase_offset) * 180.0f / M_PI);

    // DON'T set carrier_phase_initialized here - let updateChannelEstimate() do it
    // on the first data symbol. This ensures we use fresh pilot data for phase
    // recovery instead of potentially noisy LTS estimates.
    //
    // The LTS channel estimates provide magnitude and approximate phase.
    // The first data symbol's pilots will refine the common phase offset.

    // Mark that we have a valid channel estimate (for smoothing factor selection)
    snr_symbol_count = num_symbols;
}

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
        LOG_DEMOD(DEBUG, "=== First DATA symbol pilot analysis (snr_symbol_count=%d) ===", snr_symbol_count);
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            LOG_DEMOD(DEBUG, "DATA pilot[%zu] idx=%d: rx=(%.4f,%.4f) |rx|=%.4f tx=(%.1f,%.1f) H=(%.2f,%.2f) |H|=%.2f",
                      i, idx,
                      freq_domain[idx].real(), freq_domain[idx].imag(), std::abs(freq_domain[idx]),
                      pilot_sequence[i].real(), pilot_sequence[i].imag(),
                      h_ls_all[i].real(), h_ls_all[i].imag(),
                      std::abs(h_ls_all[i]));
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

    // For differential modulation: apply pilot_phase_correction to track common phase drift
    // This is updated after each symbol via decision-directed tracking
    bool is_differential = (mod == Modulation::DBPSK || mod == Modulation::DQPSK || mod == Modulation::D8PSK);

    if (is_differential) {
        // For differential modes on fading channels, use ZF equalization with the
        // channel estimate from LTS + decision-directed updates.
        //
        // Key insight: DQPSK measures phase DIFFERENCES between consecutive symbols.
        // ZF equalization divides by H, so if H changes between symbols, we get:
        //   diff = (rx[n]/H[n]) * conj(rx[n-1]/H[n-1])
        // If we use the SAME H estimate for both, errors cancel somewhat.
        //
        // The channel_estimate[] is updated by decision-directed tracking in
        // demodulateSymbol(), so it adapts over the frame.
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            Complex received = freq_domain[idx];
            Complex h = channel_estimate[idx];
            float h_power = std::norm(h);

            // Apply timing correction
            int k = idx;
            if (k > (int)config.fft_size / 2) k -= config.fft_size;
            float timing_phase = 2.0f * M_PI * k * timing_offset_samples / config.fft_size;
            Complex timing_correction = std::exp(Complex(0, timing_phase));

            // ZF equalization per carrier, then common phase correction
            if (h_power > 1e-6f) {
                equalized[i] = received * std::conj(h) / h_power * pilot_phase_correction * timing_correction;
                carrier_noise_var[i] = noise_variance / h_power;
            } else {
                // Deep fade - just apply phase correction, mark as unreliable
                equalized[i] = received * pilot_phase_correction * timing_correction;
                carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
            }
            carrier_noise_var[i] = std::max(MIN_CARRIER_NOISE_VAR, std::min(MAX_CARRIER_NOISE_VAR, carrier_noise_var[i]));
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
    auto result = equalize(freq_domain, config.modulation);

    // DEBUG: Print first few equalized symbols on first data symbol
    if (soft_bits.empty() && !result.empty()) {
        fprintf(stderr, "[EQ-DBG] First 5 equalized (sym 0): ");
        for (size_t i = 0; i < std::min(size_t(5), result.size()); ++i) {
            fprintf(stderr, "(%.3f,%.3f) ", result[i].real(), result[i].imag());
        }
        fprintf(stderr, "\n");
    }

    return result;
}

} // namespace ultra
