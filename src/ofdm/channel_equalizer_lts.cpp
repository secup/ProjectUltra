// OFDM LTS channel estimation
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <algorithm>
#include "lts_cfo_update.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "sim/channel_calibration.hpp"
#include "ultra/logging.hpp"
#include "ofdm/frequency_selectivity.hpp"
#include "ofdm/genie_true_h.hpp"

namespace ultra {

using namespace demod_constants;

namespace {

// ULTRA_CHEST_NOISE_SCALE — corrects the 3.01 dB decode-path noise-scale error (see the
// derivation at the assignment site). DEFAULT-OFF. Deliberately NOT a function-local
// `static const`: that latches the env on first call, so one process could only ever see one
// value and an A/B test in a single binary would be impossible. This is read once per LTS
// estimate, never on a per-symbol hot path.
bool chestNoiseScaleFixEnabled() {
    const char* e = std::getenv("ULTRA_CHEST_NOISE_SCALE");
    return e != nullptr && e[0] != '\0' && e[0] != '0';
}

}  // namespace

void OFDMDemodulator::Impl::updateLastSNREstimate(float signal_power,
                                                  float noise_power,
                                                  size_t independent_bins,
                                                  float alpha,
                                                  bool fitted_common_gain,
                                                  bool noise_reference_only,
                                                  float noise_power_reference_scale) {
    if (signal_power <= 0.0f || noise_power <= 0.0f || independent_bins == 0) {
        return;
    }

    // LTS/pilot residuals are measured in FFT-bin channel-estimate units.
    // Near-AWGN pilot residuals use signal/noise so real signal fades remain
    // visible. Under selective fading, temporal pilot residuals are not a clean
    // noise proxy, so the caller can keep the LTS residual and reference it to
    // the calibrated broadband noise floor instead.
    //
    // Absolute SNR calibration derivation:
    //
    //   TX mapping in src/ofdm/modulator.cpp:
    //     - data, LTS, and pilots are unit-magnitude complex subcarrier symbols
    //       (DQPSK differential states, Zadoff-Chu LTS, and BPSK pilots all have
    //       |X_k| = 1), so the pilot-vs-data power ratio is 1.0 (0 dB).
    //     - FFT::inverse() applies 1/N, and FFT::forward() applies 1.0.
    //     - real passband modulation makes each received positive-frequency bin
    //       carry output_scale/2, but that signal factor cancels in the LS
    //       residual R_k / X_k used here.
    //
    //   AWGN broadband reference:
    //     - SimulatedChannel sizes real white audio noise from the calibrated
    //       in-band PING reference:
    //       sigma^2 = kModemReferencePower / SNR_broadband.
    //     - after downconversion, an unnormalized N-point FFT gives each active
    //       complex bin noise power N * sigma^2.
    //     - therefore SNR_broadband = N * kModemReferencePower / noise_bin.
    //     - operator/rate-selection SNR is in-band, matching the idle-noise
    //       estimator. For white noise, in-band noise is the 50-2950 Hz FIR
    //       output power, so SNR_in_band = SNR_broadband / FIR_energy.
    //
    //   Two-LTS residual normalization:
    //     - estimateChannelFromLTS() forms noise_power = E|H1 - H0|^2 / 4.
    //     - for independent per-symbol FFT-bin noises, E|H1 - H0|^2 = 2N*sigma^2.
    //     - so the stored noise_power is N*sigma^2/2, i.e. 3.0103 dB too small
    //       for a single-symbol FFT-bin noise reference.
    //
    // The caller-supplied reference scale converts its residual to a
    // single-symbol FFT-bin noise reference. For the repeated-LTS difference
    // estimator this scale is 2.0, which accounts for the observed +2.71 dB
    // AWGN bias without a fitted offset; the remaining ~0.3 dB is
    // finite-sample/channel-search variance. Guard-bin FFT noise estimates
    // already use single-symbol bin power, so they use scale 1.0.
    if (noise_reference_only) {
        const float corrected_noise_power =
            std::max(noise_power_reference_scale, 1.0e-6f) * noise_power;
        // LEVEL-INDEPENDENT calibration. The old form used a FIXED
        //   broadband_snr = fft_size * kModemReferencePower / corrected_noise
        // which ASSUMES the received signal sits at the sim's calibrated reference power.
        // On real hardware the RX operating level differs from that reference (e.g. ~12 dB
        // low on IONOS), so it credited signal power the signal lacked and over-read SNR by
        // exactly that deficit. Instead, take the MEASURED per-carrier signal/noise ratio
        // (signal_power is the LTS |H|² already passed in; both terms scale with the operating
        // level, so the ratio is invariant) and apply the SAME constant measurement-gain
        // offset the fitted-gain path below uses to reach the in-band operator convention.
        // In the sim (signal at reference, |H|² ≈ output_scale²·0.25·cp) this is unchanged.
        // 2026-07-07 CALIBRATION FIX (+3.2 dB AWGN optimism root-caused):
        // (a) DEBIAS the signal power: signal_power is the mean of RAW
        //     per-symbol LS |h|^2, whose expectation is |H|^2 + N*sigma^2 —
        //     the per-bin estimation noise INFLATES it by exactly the
        //     (scale-corrected) noise power. Subtract it; floor keeps deep
        //     fades finite (-20 dB per-carrier) instead of log(<=0).
        //     This term (+10log10(1+1/SNR_c)) GREW under fading — the
        //     "reads 16.4 while the link can't hold QPSK R1/2" optimism.
        // GLITCH GUARD (rig F237, 2026-07-13): a frame that achieved LTS sync
        // has processing-gain-proven signal presence — a same-frame noise
        // estimate at/above the signal power is a MEASUREMENT FAULT (timing
        // slip / CFO glitch corrupting the LTS pair), not channel state.
        // Without this, the debias floors at 0.01x and one glitch frame
        // injects a -19.5 dB reading into the EMA (rig meter collapsed to
        // -9.2 at a 20 dB channel -> authority demote churn + ACK-staircase
        // flapping). Skip the update; the EMA holds and the next clean frame
        // measures honestly. A GENUINE deep fade produces no LTS sync and
        // therefore no reading — same outcome, no lie either way.
        if (signal_power <= 2.0f * corrected_noise_power) {
            return;
        }
        const float debiased_signal_power =
            signal_power - corrected_noise_power;
        const float snr_per_carrier_db = 10.0f * std::log10(
            debiased_signal_power / std::max(corrected_noise_power, 1.0e-12f));
        // (b) GEOMETRIC per-carrier -> in-band conversion. The old
        //     measurement_gain implicitly credited the OFDM signal with the
        //     PING-chirp reference power (output_scale^2/4 vs
        //     kModemReferencePower), over-reporting by a structural
        //     +2.758 dB: a num_carriers-strong OFDM waveform at bin power
        //     |H|^2 carries total audio signal power 2*Ncar*|H|^2/N^2 while
        //     the bin noise is N*sigma^2, so
        //       SNR_broadband = per_carrier * (2*Ncar / N)
        //     with NO reference-power or CP term (the CP is a signal copy —
        //     continuous signal power is unchanged by it; noise persists
        //     through it identically). broadbandToInBandSnrDb then applies
        //     the shared 50-2950 Hz convention (net +0.26 dB at 59/1024).
        //     Level-invariant by construction — pure measured ratio.
        const float broadband_snr_db =
            snr_per_carrier_db +
            10.0f * std::log10(2.0f * static_cast<float>(config.num_carriers) /
                               static_cast<float>(config.fft_size));
        const float in_band_snr_db =
            sim::broadbandToInBandSnrDb(broadband_snr_db);

        if (!std::isfinite(in_band_snr_db)) {
            return;
        }

        if (!last_snr_db_estimate_valid) {
            last_snr_db_estimate = in_band_snr_db;
            last_snr_db_estimate_valid = true;
        } else {
            const float a = std::clamp(alpha, 0.0f, 1.0f);
            last_snr_db_estimate =
                a * in_band_snr_db + (1.0f - a) * last_snr_db_estimate;
        }
        return;
    }

    // signal/noise is a frequency-bin SNR. Convert it through the modem's
    // calibrated broadband-audio reference and then to the shared in-band
    // operator/rate-selection SNR convention:
    //
    //   TX real passband carrier amplitude at the FFT bin is output_scale/2.
    //   White audio noise is integrated by the N-point FFT.
    //   The configured/reference SNR uses the calibrated in-band PING reference
    //   over the full OFDM symbol duration including CP.
    //
    // This subtracts OFDM measurement gain while retaining true signal fades in
    // signal_power. Averaging bins reduces estimator variance; it is not RF
    // gain. The temporal pilot path also fits one common complex gain, removing
    // one complex degree of freedom, so undo that small residual-noise bias.
    // Same 2026-07-07 calibration fix as the noise_reference_only path above:
    // debias the LS signal power, then the GEOMETRIC per-carrier -> in-band
    // conversion (2*Ncar/N), no reference-power/CP terms. The fitted-common-
    // gain dof correction on the residual is kept (one complex dof removed).
    float dof_corrected_noise = std::max(noise_power, 1.0e-12f);
    if (fitted_common_gain && independent_bins > 1) {
        dof_corrected_noise *= static_cast<float>(independent_bins) /
                               static_cast<float>(independent_bins - 1);
    }
    // Same glitch guard as the noise_reference_only path above.
    if (signal_power <= 2.0f * dof_corrected_noise) {
        return;
    }
    const float debiased_signal_power = signal_power - dof_corrected_noise;
    const float snr_per_carrier_db =
        10.0f * std::log10(debiased_signal_power / dof_corrected_noise);
    const float broadband_snr_db =
        snr_per_carrier_db +
        10.0f * std::log10(2.0f * static_cast<float>(config.num_carriers) /
                           static_cast<float>(config.fft_size));
    const float in_band_snr_db =
        sim::broadbandToInBandSnrDb(broadband_snr_db);

    if (!std::isfinite(in_band_snr_db)) {
        return;
    }

    // Temporal pilot residuals can contain residual channel motion even when
    // the magnitude-variance gate says "near AWGN". Do not let that path
    // overrule the same-frame LTS noise estimate with a large downward jump.
    if (fitted_common_gain && last_snr_db_estimate_valid &&
        in_band_snr_db < last_snr_db_estimate - 3.0f) {
        return;
    }

    if (!last_snr_db_estimate_valid) {
        last_snr_db_estimate = in_band_snr_db;
        last_snr_db_estimate_valid = true;
    } else {
        const float a = std::clamp(alpha, 0.0f, 1.0f);
        last_snr_db_estimate =
            a * in_band_snr_db + (1.0f - a) * last_snr_db_estimate;
    }
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

    activateCarrierPattern(0);

    LOG_DEMOD(DEBUG, "estimateChannelFromLTS: num_symbols=%zu, symbol_samples=%zu, first_sample=%.6f",
             num_symbols, symbol_samples, training_samples[0]);

    // Print carrier indices
    {
        char idx_buf[128] = "";
        int pos = 0;
        for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
            pos += snprintf(idx_buf + pos, sizeof(idx_buf) - pos, "%d ", data_carrier_indices[i]);
        }
        LOG_DEMOD(DEBUG, "LTS RX carrier indices (first 5): %s(total %zu)", idx_buf, data_carrier_indices.size());
    }

    if (num_symbols == 0 || data_carrier_indices.empty()) return;

    // Store per-symbol data-carrier channel estimates for LTS difference noise
    // and residual-CFO estimation.
    std::vector<std::vector<Complex>> h_per_symbol(num_symbols);
    for (auto& v : h_per_symbol) v.resize(data_carrier_indices.size());
    // Per-symbol PILOT estimates too, so CFO-clean averaging (lever ①) can average
    // pilots aligned to the same last-symbol phase frame as the data carriers.
    std::vector<std::vector<Complex>> h_per_symbol_pilot(num_symbols);
    for (auto& v : h_per_symbol_pilot) v.resize(pilot_carrier_indices.size());

    // Accumulate channel estimates from each training symbol
    std::vector<Complex> h_sum_data(data_carrier_indices.size(), Complex(0, 0));
    std::vector<Complex> h_sum_pilot(pilot_carrier_indices.size(), Complex(0, 0));
    std::vector<Complex> h_last_pilot(pilot_carrier_indices.size(), Complex(0, 0));
    size_t valid_symbols = 0;
    const float phase_at_training_start = freq_correction_phase;

    double guard_noise_power_sum = 0.0;
    size_t guard_noise_bin_count = 0;

    auto accumulate_guard_noise = [&](const std::vector<Complex>& freq_domain) {
        // After downconversion, the desired OFDM signal occupies active positive
        // bins +1..+ceil(Ncarriers/2). The real-passband image lands on the
        // negative side near -2*center_freq, so the adjacent positive guard bins
        // are signal-free FFT-bin noise samples with the same N*sigma^2 scaling
        // as active-carrier LS residuals.
        const int positive_active_edge = static_cast<int>((config.num_carriers + 1) / 2);
        const int guard_start = positive_active_edge + 2;  // one-bin cushion for residual CFO
        const int nyquist_bin = static_cast<int>(config.fft_size / 2);
        const int guard_stop = std::min(
            nyquist_bin - 1,
            guard_start + static_cast<int>(config.num_carriers) - 1);
        for (int bin = guard_start; bin <= guard_stop; ++bin) {
            if (bin >= 0 && static_cast<size_t>(bin) < freq_domain.size()) {
                guard_noise_power_sum += static_cast<double>(std::norm(freq_domain[bin]));
                ++guard_noise_bin_count;
            }
        }
    };

    // Process each training symbol using the main mixer (it will be advanced)
    const float* ptr = training_samples;
    for (size_t sym = 0; sym < num_symbols; ++sym) {
        // Use toBaseband and extractSymbol like normal demodulation
        SampleSpan sym_span(ptr, symbol_samples);
        const auto& baseband = toBaseband(sym_span);
        const auto& freq = extractSymbol(baseband, 0);

        // DEBUG: Print first few freq domain values on first training symbol
        if (sym == 0) {
            char rx_buf[256] = "", tx_buf[256] = "";
            int rp = 0, tp = 0;
            for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
                int idx = data_carrier_indices[i];
                rp += snprintf(rx_buf + rp, sizeof(rx_buf) - rp, "[%d]=(%.3f,%.3f) ", idx, freq[idx].real(), freq[idx].imag());
                Complex tx = sync_sequence[i % sync_sequence.size()];
                tp += snprintf(tx_buf + tp, sizeof(tx_buf) - tp, "(%.3f,%.3f) ", tx.real(), tx.imag());
            }
            LOG_DEMOD(DEBUG, "LTS RX freq[idx] first 5 carriers: %s", rx_buf);
            LOG_DEMOD(DEBUG, "LTS TX sync_seq first 5 carriers: %s", tx_buf);
        }

        accumulate_guard_noise(freq);

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
                h_last_pilot[i] = h_ls;  // Keep last symbol's estimate
                h_per_symbol_pilot[sym][i] = h_ls;
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

    // === RESIDUAL CFO ESTIMATION FROM LTS ===
    // Even when chirp-based CFO is applied, fading can cause chirp peak position errors
    // that result in a wrong CFO estimate (e.g., -1.4 Hz when actual is 0).
    // The toBaseband() above applied this wrong CFO, so we can detect the residual
    // by measuring the phase rotation between training symbols.
    //
    // If H[sym0] and H[sym1] differ by a consistent phase across carriers,
    // that phase = residual_CFO × T_symbol.
    last_lts_residual_cfo_hz = 0.0f;
    last_lts_pair_chi = 0.0f;
    if (valid_symbols >= 2) {
        Complex phase_diff_sum(0, 0);
        int cfo_valid = 0;

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            Complex h0 = h_per_symbol[0][i];
            Complex h1 = h_per_symbol[1][i];
            if (std::abs(h0) > 0.01f && std::abs(h1) > 0.01f) {
                Complex diff = h1 * std::conj(h0);
                float mag = std::abs(diff);
                if (mag > 1e-6f) {
                    phase_diff_sum += diff / mag;  // Normalized unit vector
                    cfo_valid++;
                }
            }
        }

        if (cfo_valid > 0) {
            last_lts_pair_chi = std::norm(phase_diff_sum) / static_cast<float>(cfo_valid);
        }
        if (cfo_valid > 10) {
            float avg_phase = std::atan2(phase_diff_sum.imag(), phase_diff_sum.real());
            float symbol_duration = static_cast<float>(symbol_samples) / config.sample_rate;
            float residual_cfo = avg_phase / (2.0f * M_PI * symbol_duration);
            last_lts_residual_cfo_hz = residual_cfo;

            const float cfo_coherence =
                std::abs(phase_diff_sum) / static_cast<float>(cfo_valid);

            // FRAME-CONSTANT CFO UPDATE (2026-11-08; derivations in src/ofdm/lts_cfo_update.hpp).
            // Correction K * r only when the prior is UNCERTIFIED (set per frame by the decoder
            // through setCfoPriorSigmaHz) and r is significant against this LTS pair's own
            // coherence-derived standard error (|r| > 3 sigma_r); K = P / (P + R) ~ 1. Applied
            // only when the pair is evidence of a real training pair (chi >= ln 1000). Replaces the seed/flatness gate, which never fired on
            // fading (its seed branch was dead after CFO pre-correction) and so left a wrong
            // chirp CFO uncorrected for whole groups. The |r| < 5 Hz sanity bound stays: it is
            // the uncertified prior's own phantom bound (CP / (2 kappa) = 4.8 Hz) rounded up.
            const double sigma_r = lts_cfo::residualSigmaHz(
                cfo_coherence, cfo_valid, static_cast<double>(symbol_duration));
            const double cfo_gain = lts_cfo::gain(cfo_prior_sigma_hz, sigma_r);
            const bool pair_evidence = last_lts_pair_chi >= lts_cfo::kEvidenceChi;
            const float old_cfo = freq_offset_hz;
            const float applied_cfo = static_cast<float>(
                lts_cfo::frameCorrectionHz(residual_cfo, sigma_r, cfo_prior_sigma_hz, cfo_valid));
            last_lts_cfo_gain = applied_cfo != 0.0f ? static_cast<float>(cfo_gain) : 0.0f;
            if (pair_evidence && applied_cfo != 0.0f && std::abs(residual_cfo) < 5.0f) {
                freq_offset_hz += applied_cfo;
                freq_offset_filtered = freq_offset_hz;

                LOG_DEMOD(WARN,
                          "LTS residual CFO: %.2f Hz detected (coh=%.2f sigma=%.2f Hz, prior "
                          "sigma=%.2f Hz, K=%.3f) %.2f -> %.2f Hz",
                          residual_cfo, cfo_coherence, sigma_r, cfo_prior_sigma_hz, cfo_gain,
                          old_cfo, freq_offset_hz);

                // Re-process training symbols with corrected CFO for accurate channel estimate
                // Reset mixer to start position and re-run
                mixer.reset();
                // Preserve original phase baseline at training start.
                // Resetting to zero here breaks phase consistency when processing
                // starts at non-zero absolute sample positions.
                freq_correction_phase = phase_at_training_start;

                // Recompute phase increment with corrected CFO
                const float* ptr2 = training_samples;
                for (auto& v : h_per_symbol) std::fill(v.begin(), v.end(), Complex(0, 0));
                for (auto& v : h_per_symbol_pilot) std::fill(v.begin(), v.end(), Complex(0, 0));
                std::fill(h_sum_data.begin(), h_sum_data.end(), Complex(0, 0));
                std::fill(h_sum_pilot.begin(), h_sum_pilot.end(), Complex(0, 0));
                guard_noise_power_sum = 0.0;
                guard_noise_bin_count = 0;

                for (size_t sym = 0; sym < num_symbols; ++sym) {
                    SampleSpan sym_span(ptr2, symbol_samples);
                    const auto& baseband = toBaseband(sym_span);
                    const auto& freq = extractSymbol(baseband, 0);

                    accumulate_guard_noise(freq);

                    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
                        int idx = data_carrier_indices[i];
                        Complex rx = freq[idx];
                        Complex tx = sync_sequence[i % sync_sequence.size()];
                        if (std::abs(tx) > 0.01f) {
                            Complex h_ls = rx / tx;
                            h_sum_data[i] += h_ls;
                            h_per_symbol[sym][i] = h_ls;
                        }
                    }
                    for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                        int idx = pilot_carrier_indices[i];
                        Complex rx = freq[idx];
                        Complex tx = pilot_sequence[i];
                        if (std::abs(tx) > 0.01f) {
                            Complex h_ls = rx / tx;
                            h_sum_pilot[i] += h_ls;
                            h_last_pilot[i] = h_ls;
                            h_per_symbol_pilot[sym][i] = h_ls;
                        }
                    }
                    ptr2 += symbol_samples;
                }

                LOG_DEMOD(INFO, "LTS re-processed with corrected CFO=%.2f Hz", freq_offset_hz);
            } else if (std::abs(residual_cfo) > 0.1f) {
                LOG_DEMOD(DEBUG,
                          "LTS residual CFO %.2f Hz NOT applied (%s; coh=%.2f chi=%.1f K=%.3f)",
                          residual_cfo,
                          !pair_evidence ? "no pair evidence"
                              : (cfo_gain <= 0.0 ? "certified prior"
                                 : (std::abs(residual_cfo) >= 5.0f ? "beyond phantom bound"
                                                                     : "within LTS noise")),
                          cfo_coherence, last_lts_pair_chi, cfo_gain);
            }
        }
    }

    // === COMBINE PER-SYMBOL LTS ESTIMATES INTO THE FINAL H ===
    //
    // DEFAULT (lts_cfo_avg_enabled_ == false): use the LAST symbol's H for both data and
    // pilot carriers. The last symbol is closest in time to the first data symbol, which
    // minimizes CFO-induced phase mismatch; naive (un-aligned) averaging of the two symbols
    // caused decode failures at CFO=30 Hz from 2-symbol phase drift, which is why the
    // original code threw the first symbol away. Pilots use the last symbol too so pilot and
    // data H share one phase reference (complex pilot interpolation needs that).
    //
    // LEVER ① CFO-clean averaging (ULTRA_LTS_CFO_AVG): the N LTS symbols are IDENTICAL
    // training, so after a (coherent-over-48ms) channel they differ ONLY by a common
    // per-symbol phase from residual CFO — a TIME rotation, identical on every carrier (data
    // AND pilot). So we estimate that rotation as the magnitude-weighted cross-correlation to
    // the LAST symbol, align each earlier symbol[s] -> exp(jφ_s)·h[s], then average. That
    // averages down the per-carrier LS noise by ~10·log10(N) dB (−3 dB for N=2) WITHOUT any
    // phase mismatch, because the average is taken in the LAST symbol's frame — so it stays
    // consistent with the first data symbol and with the pilots, removing the exact reason
    // last-symbol-only was used. Magnitude weighting is the ML estimate of the common phase
    // (strong carriers dominate, nulled carriers contribute ~nothing) and is robust at low
    // SNR (averaged over ~47 data carriers). Design lever ①:
    // docs/CHANNEL_ESTIMATE_REINFORCEMENT_DESIGN_2026_05_30.md.
    const bool cfo_clean_average = lts_cfo_avg_enabled_ && valid_symbols >= 2;
    if (cfo_clean_average) {
        const size_t last = num_symbols - 1;
        // Common per-symbol alignment phasor exp(jφ_s) bringing symbol s into the last
        // symbol's frame. Derived from the DATA carriers (most of them, most robust) and
        // reused for pilots, since the CFO rotation is common to all carriers.
        std::vector<Complex> align(num_symbols, Complex(1.0f, 0.0f));
        for (size_t s = 0; s + 1 < num_symbols; ++s) {
            Complex cross(0, 0);
            for (size_t j = 0; j < data_carrier_indices.size(); ++j)
                cross += h_per_symbol[last][j] * std::conj(h_per_symbol[s][j]);
            const float m = std::abs(cross);
            align[s] = (m > 1e-9f) ? cross / m : Complex(1.0f, 0.0f);
        }
        const float inv_n = 1.0f / static_cast<float>(num_symbols);
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            Complex acc = h_per_symbol[last][i];
            for (size_t s = 0; s + 1 < num_symbols; ++s)
                acc += h_per_symbol[s][i] * align[s];
            channel_estimate[data_carrier_indices[i]] = acc * inv_n;
        }
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            Complex acc = h_per_symbol_pilot[last][i];
            for (size_t s = 0; s + 1 < num_symbols; ++s)
                acc += h_per_symbol_pilot[s][i] * align[s];
            channel_estimate[pilot_carrier_indices[i]] = acc * inv_n;
        }
        LOG_DEMOD(INFO, "LTS CFO-clean avg: N=%zu symbols, align[0]=%.3f rad",
                  num_symbols,
                  num_symbols >= 2 ? std::atan2(align[0].imag(), align[0].real()) : 0.0f);
    } else if (valid_symbols > 0) {
        for (size_t i = 0; i < data_carrier_indices.size(); ++i)
            channel_estimate[data_carrier_indices[i]] = h_per_symbol[num_symbols - 1][i];
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i)
            channel_estimate[pilot_carrier_indices[i]] = h_last_pilot[i];
    }

    // Estimate per-carrier phase slope from LTS (timing offset effect).
    // On AWGN, H[k] = A × exp(-j × 2π × k × Δn/N_fft), giving a linear phase slope.
    // This slope is ~19°/carrier for a typical ~54-sample timing offset.
    // Knowing this slope allows de-sloping before pilot interpolation (removing the
    // timing contribution so that only channel-phase variation remains, which is
    // smooth enough for 10-carrier pilot spacing).
    {
        int neg_limit = config.num_carriers / 2;
        Complex slope_sum(0, 0);
        int slope_count = 0;
        // Use ALL carrier H (data + pilot) for a robust slope estimate
        for (size_t i = 0; i + 1 < all_carrier_fft_indices.size(); ++i) {
            int idx0 = all_carrier_fft_indices[i];
            int idx1 = all_carrier_fft_indices[i + 1];
            Complex h0 = channel_estimate[idx0];
            Complex h1 = channel_estimate[idx1];
            if (std::abs(h0) > 0.01f && std::abs(h1) > 0.01f) {
                Complex diff = h1 * std::conj(h0);
                float mag = std::abs(diff);
                if (mag > 1e-6f) {
                    slope_sum += diff / mag;  // Normalized to unit magnitude
                    slope_count++;
                }
            }
        }
        if (slope_count > 0) {
            lts_phase_slope = std::arg(slope_sum / static_cast<float>(slope_count));
            timing_offset_samples = -lts_phase_slope * config.fft_size / (2.0f * M_PI);
            LOG_DEMOD(INFO, "LTS phase slope: %.2f°/carrier (timing offset ~%.1f samples)",
                      lts_phase_slope * 180.0f / M_PI,
                      timing_offset_samples);
        }
    }

    // 2026-05-30 (ULTRA_LTS_DFT_DENOISE): frequency-domain denoise of the LTS H by
    // Gaussian neighbor-smoothing across carriers. The channel impulse response is short
    // (delay spread << OFDM symbol), so the channel is smooth in frequency (coherence BW >>
    // carrier spacing) — adjacent carriers' H are highly correlated and a weighted average
    // over neighbors averages down the per-carrier LS noise without touching the (slowly
    // varying) real channel. The offline H-MSE harness (tools/lts_estimate_mse) proves this
    // at the PRODUCTION 1024/59 geometry: 3-6x lower normalized H-MSE across SNR 12/16/20
    // with self_distortion ~1e-4 (a clean channel passes through untouched).
    //
    // This REPLACED an earlier DFT-window approach (IFFT -> zero taps beyond a delay window
    // -> FFT) that the SAME harness disproved: a finite N-of-1024-bin sub-band slice IDFTs to
    // a sinc that cannot be cleanly windowed, leaving a ~14% self-distortion FLOOR (it threw
    // away real channel energy). Smoothing has no transform/leakage and degrades gracefully at
    // band edges (the window just shrinks). The half-width W is a CHANNEL property (set by the
    // coherence bandwidth / delay spread), NOT an SNR-tuned constant — it must stay well below
    // the channel's frequency-ripple period or it washes out real selectivity. Harness optimum
    // for Good (~0.5 ms delay) is W~3 and is SNR-INDEPENDENT, confirming W is geometry- not
    // noise-driven; a longer-delay channel (Moderate/Poor) wiggles faster in frequency and
    // needs a SMALLER W (future: derive W = f(negotiated delay spread)). Tunable via
    // ULTRA_LTS_DFT_DENOISE_TAPS. Design: docs/CHANNEL_ESTIMATE_REINFORCEMENT_DESIGN_2026_05_30.md
    // (lever 2). Default off. Gate + W come from Impl members (env-initialized in the
    // constructor, overridable from a test via OFDMDemodulator::setLtsDftDenoise) so the
    // offline harness can flip denoise on/off per call without re-reading the env.
    if (dft_denoise_enabled_ && valid_symbols > 0 && all_carrier_fft_indices.size() >= 4) {
        // Frequency-domain Gaussian neighbor-smoothing of the LTS H. The channel is smooth
        // in frequency (coherence BW >> carrier spacing on Good), so adjacent carriers' H are
        // highly correlated -> a weighted average over neighbors averages down the per-carrier
        // LS noise. This REPLACES an earlier sub-band DFT-window denoise that the offline
        // H-MSE harness (tools/lts_estimate_mse) proved corrupts the estimate (~14%
        // self-distortion FLOOR: a finite 48-of-512-bin slice IDFTs to a sinc that cannot be
        // cleanly windowed). Smoothing has no transform/leakage and handles band edges by
        // shrinking the window. Half-width W (carriers) must stay << the channel's
        // frequency-ripple period so it does not wash out real selectivity; it is a CHANNEL
        // property (coherence bandwidth), tunable via ULTRA_LTS_DFT_DENOISE_TAPS. SNR-adaptive
        // by construction: same W removes proportionally more noise at low SNR.
        const size_t M = all_carrier_fft_indices.size();
        int W = dft_denoise_taps_ > 0 ? dft_denoise_taps_ : 2;  // half-width; W=2 is the harness-proven
                                                                // SAFE width (preserves deep nulls <=1.2x;
                                                                // W>=3 smears them -> 16QAM poison). Interim
                                                                // smoother; principled estimator is delay-domain LS.
        W = std::max(1, std::min<int>(W, static_cast<int>(M) / 4));
        const float sigma = std::max(0.5f, static_cast<float>(W) / 1.5f);
        std::vector<Complex> H_in(M);
        for (size_t i = 0; i < M; ++i)
            H_in[i] = channel_estimate[all_carrier_fft_indices[i]];
        for (size_t i = 0; i < M; ++i) {
            Complex acc(0, 0);
            float wsum = 0.0f;
            for (int d = -W; d <= W; ++d) {
                const long j = static_cast<long>(i) + d;
                if (j < 0 || j >= static_cast<long>(M)) continue;
                const float w =
                    std::exp(-static_cast<float>(d * d) / (2.0f * sigma * sigma));
                acc += w * H_in[static_cast<size_t>(j)];
                wsum += w;
            }
            if (wsum > 0.0f)
                channel_estimate[all_carrier_fft_indices[i]] = acc / wsum;
        }
        LOG_DEMOD(INFO, "LTS H freq-smooth: M=%zu W=%d sigma=%.2f", M, W, sigma);
    }

    LOG_DEMOD(INFO, "LTS channel estimate: %zu data + %zu pilot carriers",
              data_carrier_indices.size(), pilot_carrier_indices.size());

    // 2026-05-29 diag (ULTRA_GENIE_LTS_FREEZE): snapshot the full-band LTS channel
    // estimate so the genie test can hold it across this frame's data symbols
    // instead of re-interpolating from sparse pilots. See demodulator_impl.hpp.
    if (genieLtsFreezeEnabled()) {
        genie_lts_h_ = channel_estimate;
    }

    // 2026-07-29 diag (genie_true_h): on the CLEAN pass the received signal is
    // H(x)X with negligible noise, so this LTS estimate IS the true H, already
    // in the equalizer's own domain and scaling. Store it for the noisy pass.
    // See src/ofdm/genie_true_h.hpp for why the two older oracles are invalid.
    // FIRST capture per frame only. The clean pass is pumped for a fixed budget
    // and re-syncs several times per frame (~4.5 measured), so a last-wins buffer
    // ends up holding an H from the trailing dead air -- which, injected into
    // every data symbol, destroys decoding even when the two passes are fed a
    // bit-identical signal. The harness clears the buffer at each frame boundary.
    if (ultra::ofdm::genie_true_h::mode() == ultra::ofdm::genie_true_h::Mode::Capture) {
        // Only the pass whose geometry matches the transmitted profile. See the
        // CAPTURE PROVENANCE note in genie_true_h.hpp: the control-first hypothesis
        // decodes at QPSK R1/4 / spacing 5 before the data profile, and capturing
        // THAT stores H_channel * X_data / X_control, not the channel.
        const auto& x = ultra::ofdm::genie_true_h::expect();
        const bool geometry_matches =
            !x.active ||
            (static_cast<uint32_t>(config.pilot_spacing) == x.pilot_spacing &&
             static_cast<int>(config.modulation) == x.modulation &&
             static_cast<uint32_t>(config.num_carriers) == x.num_carriers &&
             static_cast<uint32_t>(config.fft_size) == x.fft_size);
        if (!geometry_matches) {
            ++ultra::ofdm::genie_true_h::stats().rejected_profile;
        } else if (ultra::ofdm::genie_true_h::buffer().empty()) {
            ultra::ofdm::genie_true_h::buffer() = channel_estimate;
            ++ultra::ofdm::genie_true_h::stats().captures;
            // NB: noise_variance is NOT captured here -- it is assigned FURTHER DOWN
            // this function (both the 2-LTS and 1-LTS branches). Capturing it at this
            // point recorded the PREVIOUS frame's value (or the 0.1f reset default),
            // which made gamma = |H|^2/nv independent of SNR: measured per-frame mean
            // gamma spanned only 4634..9129 across a 6..28 dB sweep, where it should
            // move ~160x. Fourth instance of the same provenance class of bug in this
            // diagnostic. It is now captured at the END of the function instead.
        }
    }

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
    last_lts_channel_magnitude = h_mag_avg;
    last_lts_signal_power = h_mag_avg * h_mag_avg;

    const float guard_noise_var = guard_noise_bin_count > 0
        ? static_cast<float>(guard_noise_power_sum /
                             static_cast<double>(guard_noise_bin_count))
        : 0.0f;
    const bool guard_noise_valid =
        std::isfinite(guard_noise_var) && guard_noise_var > 0.0f;

    // Estimate noise variance from LTS training symbols
    // With 2 training symbols, noise = (H1 - H2) / 2, variance = E[|H1-H2|²] / 4
    // At 0.1 Hz Doppler, channel barely changes between training symbols (~0.001 coherence),
    // so H1-H2 is almost entirely noise.
    //
    // Lever ① (ULTRA_LTS_CFO_AVG): a residual inter-symbol CFO φ surviving the stage-2 refine
    // puts a SIGNAL-dependent term H·(1−e^{jφ}) (≈ |H|·φ) into the raw H1−H0, biasing σ² HIGH.
    // That biased σ² then under-confidences the LLRs and is INCONSISTENT with the CFO-cleaned
    // averaged H produced above. So when ① is on we de-rotate symbol 0 into symbol 1's frame
    // (the SAME magnitude-weighted phasor the averaging uses) before differencing → the H term
    // cancels and the difference is pure noise (n1 − n0·e^{jφ}, still 2σ²). Gated on the flag so
    // ① OFF reproduces the exact prior σ² (clean A/B).
    if (valid_symbols >= 2) {
        Complex noise_align(1.0f, 0.0f);
        if (lts_cfo_avg_enabled_) {
            Complex cross(0, 0);
            for (size_t i = 0; i < data_carrier_indices.size(); ++i)
                cross += h_per_symbol[1][i] * std::conj(h_per_symbol[0][i]);
            const float m = std::abs(cross);
            if (m > 1e-9f) noise_align = cross / m;  // exp(jφ): h0·noise_align ≈ h1
        }
        float noise_sum = 0.0f;
        float signal_sum = 0.0f;
        int count = 0;
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            Complex h0 = h_per_symbol[0][i];
            Complex h1 = h_per_symbol[1][i];
            if (std::abs(h0) > 1e-6f && std::abs(h1) > 1e-6f) {
                Complex diff = h1 - h0 * noise_align;  // CFO-aligned (lever ①) or raw (φ=0)
                noise_sum += std::norm(diff);  // |H1-H2|²
                signal_sum += (std::norm(h0) + std::norm(h1)) / 2.0f;
                count++;
            }
        }
        if (count > 0) {
            // noise_variance = |H1-H2|²/4 per carrier (each H has noise variance σ²)
            float lts_noise_var = noise_sum / (4.0f * count);
            float lts_signal_power = signal_sum / count;
            last_lts_signal_power = lts_signal_power;

            // Clamp SNR estimate to reasonable range (5 dB to 40 dB)
            float lts_snr = lts_signal_power / std::max(lts_noise_var, 1e-10f);
            lts_snr = std::max(3.16f, std::min(10000.0f, lts_snr));

            // ULTRA_CHEST_NOISE_SCALE (2026-07-29, DEFAULT-OFF) — the 3.01 dB scale error.
            //
            // `lts_noise_var` is NOT the per-symbol data noise. With h0 = H + n0 and
            // h1 = H + n1 (E|n|² = σ²_bin each), diff = n1 - n0 so E|diff|² = 2σ²_bin, and
            // dividing by 4 gives σ²_bin/2 — which is exactly var((n0+n1)/2), the error
            // variance of the AVERAGED two-LTS channel estimate (σ²_Havg). The meter path
            // already knows this and doubles it (`lts_diff_single_symbol_var` below). The
            // DECODE path does not, so every consumer receives a value 3.01 dB optimistic:
            //   equalize.cpp:612,639  MMSE denominator h_power + noise_variance  (under-
            //                         regularised, worst exactly in the nulls that matter)
            //   equalize.cpp:619,690  carrier_noise_var -> LLR sigma^2  (over-confident LLRs)
            //   equalize.cpp:541      reliability_noise_var -> softGrayZone + erasure gate
            // All three want single-symbol sigma^2_bin = 2 * lts_noise_var.
            //
            // WHY THIS MIGHT MATTER MORE THAN IT LOOKS: it makes the MMSE denominator
            // INTERNALLY INCONSISTENT with its numerator. That is a candidate explanation
            // for the standing anomaly that ULTRA_GENIE_LTS_FREEZE — genuinely perfect
            // full-band frequency CSI — measures WORSE than baseline (2/30 vs 14/30,
            // Good@20 s7): supplying an exact H while the noise term stays mis-scaled by 2x
            // leaves conj(H)Y/(|H|^2+nv) using a denominator that no longer matches. Fixing
            // the scale and re-running LTS-freeze is the discriminating experiment.
            //
            // The no-second-LTS fallback at :774 (signal_power / DEFAULT_SNR_LINEAR) already
            // carries data-noise semantics and is untouched.
            noise_variance = chestNoiseScaleFixEnabled() ? (2.0f * lts_noise_var)
                                                         : lts_noise_var;
            estimated_snr_linear = lts_snr;
            // NOISE SOURCE (external review 2026-07-10, finding 1): the meter
            // must measure noise IN THE OCCUPIED BAND. The guard bins sit just
            // ABOVE it (~3 kHz+ audio) — on white sim noise they read the same
            // density, but real-radio SSB filtering and hardware channel-sim
            // LPFs leave the guard band nearly empty (checked-in HF capture:
            // occupied-band noise 20.9 dB above guard) → guard-preferred SNR
            // over-reads by that whole margin, and the AWGN calibration can
            // never catch it. The repeated-LTS difference measures the noise
            // the demodulator actually sees, on any noise spectrum; its
            // channel-motion term on fast fading only ADDS noise (SNR reads
            // conservative — the safe direction). Guard bins remain only as
            // the no-second-LTS fallback below. When BOTH exist, take the MAX
            // (physics: in-band noise cannot be lower than either measurement;
            // guard > diff would indicate broadband interference above the
            // band leaking in — still noise the FIR passes).
            const float lts_diff_single_symbol_var = 2.0f * lts_noise_var;
            const float meter_noise_var =
                guard_noise_valid
                    ? std::max(guard_noise_var, lts_diff_single_symbol_var)
                    : lts_diff_single_symbol_var;

            updateLastSNREstimate(lts_signal_power, meter_noise_var,
                                  static_cast<size_t>(count), 1.0f,
                                  false, true, 1.0f);

            LOG_DEMOD(INFO, "LTS SNR estimate: internal=%.1f dB in_band=%.1f dB "
                      "(measured noise_var=%.6f, meter_noise=%.6f source=%s, signal=%.4f)",
                      10.0f * std::log10(estimated_snr_linear),
                      last_snr_db_estimate_valid ? last_snr_db_estimate : 0.0f,
                      noise_variance, meter_noise_var,
                      (guard_noise_valid && guard_noise_var > lts_diff_single_symbol_var)
                          ? "guard" : "lts-diff",
                      lts_signal_power);
        }
    } else if (h_mag_avg > 1e-6f) {
        // Fallback: only 1 training symbol, use default assumption
        float signal_power = h_mag_avg * h_mag_avg;
        last_lts_signal_power = signal_power;
        noise_variance = signal_power / DEFAULT_SNR_LINEAR;
        estimated_snr_linear = DEFAULT_SNR_LINEAR;
        updateLastSNREstimate(signal_power, noise_variance,
                              std::max<size_t>(1, data_carrier_indices.size()),
                              1.0f);
        LOG_DEMOD(INFO, "LTS SNR estimate: %.1f dB (fallback, 1 training symbol)",
                  10.0f * std::log10(estimated_snr_linear));
    }

    LOG_DEMOD(INFO, "LTS channel estimate: %zu symbols, |H|_avg=%.3f, phase_avg=%.1f°",
              valid_symbols, h_mag_avg, std::arg(h_avg) * 180.0f / M_PI);

    // DEBUG: Print first few channel estimates from first and last training symbols
    {
        char h0_buf[128] = "", hn_buf[128] = "";
        int p0 = 0, pn = 0;
        for (size_t i = 0; i < std::min(size_t(5), data_carrier_indices.size()); ++i) {
            p0 += snprintf(h0_buf + p0, sizeof(h0_buf) - p0, "%.0f deg ", std::arg(h_per_symbol[0][i]) * 180.0f / M_PI);
            pn += snprintf(hn_buf + pn, sizeof(hn_buf) - pn, "%.0f deg ", std::arg(h_per_symbol[num_symbols - 1][i]) * 180.0f / M_PI);
        }
        LOG_DEMOD(DEBUG, "LTS H from sym0 (first 5): %s", h0_buf);
        LOG_DEMOD(DEBUG, "LTS H from sym%zu (last, first 5): %s", num_symbols - 1, hn_buf);
    }

    // DON'T set carrier_phase_initialized here - let updateChannelEstimate() do it
    // on the first data symbol. This ensures we use fresh pilot data for phase
    // recovery instead of potentially noisy LTS estimates.
    //
    // The LTS channel estimates provide magnitude and approximate phase.
    // The first data symbol's pilots will refine the common phase offset.

    // Compute fading index from the LTS channel estimate. Use the averaged LTS
    // H for measurement to reduce AWGN estimator variance, but keep the
    // ── FIRST-FRAME CHANNEL-CLASS DISCRIMINATOR (frequency selectivity) ───────────
    // Read-only measurement, computed from the SAME averaged LTS H as the fading index below.
    // The fading index is an across-carrier CV, which converges to a constant for ANY Rayleigh
    // channel and so cannot separate Good from Moderate (BUG-FADING-INDEX-BLIND). This measures
    // the RIPPLE STRUCTURE instead: |H(f)|^2 of a 2-path channel ripples at cos(2 pi f D), so
    // its normalized autocorrelation at carrier lag L reads cos(2 pi L df D) — a direct read on
    // differential delay. See ofdm/frequency_selectivity.hpp for the derivation and the
    // geometry-derived lags. NOT consumed by any decision yet; logged for rig validation.
    {
        // Build |H|^2 on a UNIFORM, ASCENDING-FREQUENCY grid. Both data and pilot carriers are
        // used: the lag -> frequency mapping is only valid on an evenly spaced grid, and the
        // data-only list has pilot-shaped gaps. FFT bins wrap (bins above fft_size/2 are
        // NEGATIVE frequencies), so sort by SIGNED bin index, not by raw bin.
        const int fft_n = static_cast<int>(config.fft_size);
        std::vector<std::pair<int, float>> grid;  // (signed bin, |H|^2)
        grid.reserve(data_carrier_indices.size() + pilot_carrier_indices.size());
        auto add_carrier = [&](int bin, const Complex& h) {
            const float p = std::norm(h);
            if (!std::isfinite(p) || p <= 0.0f) return;
            const int signed_bin = (bin > fft_n / 2) ? (bin - fft_n) : bin;
            grid.emplace_back(signed_bin, p);
        };
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            Complex h = channel_estimate[data_carrier_indices[i]];
            if (valid_symbols > 0 && i < h_sum_data.size()) {
                h = h_sum_data[i] / static_cast<float>(valid_symbols);
            }
            add_carrier(data_carrier_indices[i], h);
        }
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            Complex h = channel_estimate[pilot_carrier_indices[i]];
            if (valid_symbols > 0 && i < h_sum_pilot.size()) {
                h = h_sum_pilot[i] / static_cast<float>(valid_symbols);
            }
            add_carrier(pilot_carrier_indices[i], h);
        }
        std::sort(grid.begin(), grid.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        // Split into MAXIMAL uniformly-spaced (stride-1) runs. The wideband grid straddles DC
        // with the DC bin unused, so there are normally TWO: negative and positive frequencies.
        // Both are passed to the segmented estimator — taking only the longest wasted half the
        // band (rig: N=30 of 59), which raised the single-frame confidence threshold from 0.274
        // to 0.418, right on top of the real MPM readings (-0.35..-0.46), so genuine Moderate
        // frames fell back to "not confident".
        std::vector<std::vector<float>> runs;
        for (size_t i = 0; i < grid.size(); ++i) {
            if (i == 0 || grid[i].first != grid[i - 1].first + 1) runs.emplace_back();
            runs.back().push_back(grid[i].second);
        }
        std::vector<const float*> seg_ptr;
        std::vector<size_t> seg_len;
        seg_ptr.reserve(runs.size());
        seg_len.reserve(runs.size());
        for (const auto& r : runs) {
            if (r.size() < 2) continue;  // cannot contribute any lag pair
            seg_ptr.push_back(r.data());
            seg_len.push_back(r.size());
        }

        const double df_hz = (fft_n > 0)
            ? static_cast<double>(config.sample_rate) / static_cast<double>(fft_n)
            : 0.0;
        const auto fs = seg_ptr.empty()
            ? ofdm::FrequencySelectivity{}
            : ofdm::measureFrequencySelectivitySegmented(seg_ptr.data(), seg_len.data(),
                                                         seg_ptr.size(), df_hz);
        last_freq_selectivity = fs;
        if (fs.valid) {
            const auto cls = ofdm::classifySelectivity(fs);
            LOG_DEMOD(INFO,
                      "LTS freq-selectivity: S_gm=%+.3f (lag %d dof %zu) S_mp=%+.3f (lag %d dof %zu) "
                      "N=%zu segs=%zu df=%.2f Hz -> class=%s",
                      fs.s_gm, fs.lag_gm, fs.dof_gm, fs.s_mp, fs.lag_mp, fs.dof_mp,
                      fs.carriers, seg_ptr.size(), df_hz,
                      ofdm::selectivityClassName(cls));
        }
    }

    // last-symbol H above for equalization phase consistency. Feeds last_fading_index
    // (LLR fading scaling, DD gating).
    {
        float h_mag_mean = 0.0f;
        size_t h_mag_count = 0;
        std::vector<float> lts_measure_mags;
        lts_measure_mags.reserve(data_carrier_indices.size());
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            Complex h = channel_estimate[data_carrier_indices[i]];
            if (valid_symbols > 0 && i < h_sum_data.size()) {
                h = h_sum_data[i] / static_cast<float>(valid_symbols);
            }
            const float mag = std::abs(h);
            if (mag > 0.01f && std::isfinite(mag)) {
                h_mag_mean += mag;
                lts_measure_mags.push_back(mag);
                ++h_mag_count;
            }
        }
        if (h_mag_count > 0) {
            h_mag_mean /= static_cast<float>(h_mag_count);
        }

        float h_mag_var = 0.0f;
        for (float mag : lts_measure_mags) {
            const float diff = mag - h_mag_mean;
            h_mag_var += diff * diff;
        }
        if (h_mag_count > 0) {
            h_mag_var /= static_cast<float>(h_mag_count);
        }

        const float raw_cv2 = (h_mag_mean > 0.01f)
            ? h_mag_var / (h_mag_mean * h_mag_mean)
            : 0.0f;
        const float snr_linear = last_snr_db_estimate_valid
            ? std::pow(10.0f, last_snr_db_estimate / 10.0f)
            : estimated_snr_linear;
        const float lts_average_count = std::max<size_t>(1, valid_symbols);
        const float lts_mag_noise_cv2 =
            (snr_linear > 1.0f && std::isfinite(snr_linear))
                ? 0.25f / (snr_linear * static_cast<float>(lts_average_count))
                : 0.0f;
        const float corrected_cv2 =
            std::max(0.0f, raw_cv2 - lts_mag_noise_cv2);
        last_fading_index = std::sqrt(corrected_cv2);
        public_fading_index = last_fading_index;
        LOG_DEMOD(INFO, "LTS fading index: %.3f raw=%.3f noise_cv2=%.6f "
                  "in_band_snr=%.1f dB (threshold: LLR>0.15, two-pass>0.30)",
                  last_fading_index,
                  std::sqrt(std::max(0.0f, raw_cv2)),
                  lts_mag_noise_cv2,
                  last_snr_db_estimate_valid ? last_snr_db_estimate : 0.0f);
    }

    // ── Frequency-selectivity (delay spread) from the LTS channel estimate (2026-06-08) ──
    // The fading_index above is the across-carrier CV of |H| — it measures fade DEPTH,
    // which is identical for the equal-gain 2-path Good/Moderate/Poor presets, so it
    // cannot discriminate delay spread (measured: Good 0.55 == Moderate 0.58). The
    // discriminating quantity is the COHERENCE BANDWIDTH of H(f): how fast the channel
    // decorrelates across frequency. Measure the frequency correlation rho(L) (the FT of
    // the power-delay profile), averaged over ALL carrier pairs at lag L — noise-suppressed
    // ~sqrt(N), no IDFT sidelobes — find the 0.5-correlation bandwidth Bc, map to RMS delay
    // spread. Single-snapshot, receiver-side, from the connect frame; feeds
    // last_delay_spread_ms as the frequency-selectivity base for rate selection.
    //
    // REFINEMENT (2026-06-09): measure ONLY on a full preamble (>=2 raw LTS symbols). The
    // warm/light data preambles carry an equalized/smoothed H that reads artificially flat
    // (the frequency correlation never crosses 0.5 -> coh_bw pins high -> tau under-read), which
    // is what dragged the per-run median down and overlapped good/moderate. Also reject a
    // physically-implausible blow-up (coh_bw < kMinPlausibleCohBwHz): a deep fade can decorrelate
    // adjacent carriers and crash coh_bw to ~34 Hz (~4.6 ms "delay") — keep the prior good value.
    constexpr float kMinPlausibleCohBwHz = 80.0f;  // ~2 ms delay-spread ceiling (beyond Poor HF)
    if (valid_symbols >= 2) {
        const float df_hz = static_cast<float>(config.sample_rate) /
                            static_cast<float>(config.fft_size);
        const int n_fft = static_cast<int>(config.fft_size);
        // Carriers sit AROUND DC: negative-frequency carriers are high FFT bins (wrap). Convert
        // each bin to SIGNED baseband frequency and sort by frequency so adjacent entries are
        // adjacent in frequency (required for the lag-based correlation to be valid).
        std::vector<std::pair<float, Complex>> car;  // (freq_hz, averaged H)
        car.reserve(data_carrier_indices.size());
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            Complex h = channel_estimate[data_carrier_indices[i]];
            if (valid_symbols > 0 && i < h_sum_data.size()) {
                h = h_sum_data[i] / static_cast<float>(valid_symbols);
            }
            if (!(std::isfinite(h.real()) && std::isfinite(h.imag()) && std::abs(h) > 1e-6f)) {
                continue;
            }
            int bin = static_cast<int>(data_carrier_indices[i]);
            if (bin > n_fft / 2) bin -= n_fft;        // unwrap to signed frequency
            car.emplace_back(static_cast<float>(bin) * df_hz, h);
        }
        std::sort(car.begin(), car.end(),
                  [](const std::pair<float, Complex>& a, const std::pair<float, Complex>& b) {
                      return a.first < b.first;
                  });
        float delay_spread_ms = 0.0f, coh_bw_hz = 0.0f;
        if (car.size() >= 8) {
            // Average carrier spacing in Hz (now positive; absorbs DC + pilot gaps).
            const float avg_df = (car.back().first - car.front().first) /
                                 static_cast<float>(car.size() - 1);
            // |rho(L)| = |sum_i H_i conj(H_{i+L})| / sum_i |H_i|^2  -> first 0.5 crossing.
            const int Lmax = std::min<int>(static_cast<int>(car.size()) - 1, 24);
            float prev = 1.0f;
            float L_star = static_cast<float>(Lmax);  // default: never decorrelates (very flat)
            for (int L = 1; L <= Lmax; ++L) {
                Complex num(0.0f, 0.0f);
                float den = 0.0f;
                for (size_t i = 0; i + static_cast<size_t>(L) < car.size(); ++i) {
                    num += car[i].second * std::conj(car[i + static_cast<size_t>(L)].second);
                    den += std::norm(car[i].second);
                }
                const float rho = (den > 1e-9f) ? std::abs(num) / den : 0.0f;
                if (rho < 0.5f) {
                    const float frac = (prev - 0.5f) / std::max(1e-6f, prev - rho);
                    L_star = static_cast<float>(L - 1) + frac;  // interpolated 0.5-crossing lag
                    break;
                }
                prev = rho;
            }
            coh_bw_hz = L_star * avg_df;  // 0.5-correlation coherence bandwidth
            // tau_rms ~ 1/(2 pi Bc): standard rough relation. Absolute scale is approximate;
            // what matters for classification is that Bc (hence tau_rms) SEPARATES the classes.
            delay_spread_ms = (coh_bw_hz > 1e-3f)
                                  ? 1000.0f / (2.0f * static_cast<float>(M_PI) * coh_bw_hz)
                                  : 0.0f;
        }
        if (coh_bw_hz >= kMinPlausibleCohBwHz) {
            last_delay_spread_ms = delay_spread_ms;
            LOG_DEMOD(INFO, "LTS delay spread: tau_rms=%.3f ms coh_bw=%.0f Hz carriers=%zu "
                      "[vs fading_index=%.3f]",
                      delay_spread_ms, coh_bw_hz, car.size(), last_fading_index);
        } else {
            LOG_DEMOD(INFO, "LTS delay spread: REJECTED coh_bw=%.0f Hz (< %.0f, fade/noise "
                      "artifact) carriers=%zu — keeping prior %.3f ms",
                      coh_bw_hz, kMinPlausibleCohBwHz, car.size(), last_delay_spread_ms);
        }
    }

    // Mark that we have a valid channel estimate (for smoothing factor selection)
    // Time axis is ABSOLUTE when the data-aided carry is armed (see
    // demodulator_impl.hpp: wiener_symbol_base_); -1 places the LTS one symbol
    // before this frame's first data symbol either way.
    seedWienerPilotHistoryFromCurrentChannel(wiener_symbol_base_ - 1);
    snr_symbol_count = num_symbols;

    // Phase 5 diag: pair the captured LTS H with the noise variance that was
    // actually derived for THIS frame. Must run after every noise_variance
    // assignment above; see the note at the capture site.
    if (ultra::ofdm::genie_true_h::mode() == ultra::ofdm::genie_true_h::Mode::Capture &&
        !ultra::ofdm::genie_true_h::buffer().empty()) {
        ultra::ofdm::genie_true_h::capturedNoiseVar() = noise_variance;
    }
}


} // namespace ultra
