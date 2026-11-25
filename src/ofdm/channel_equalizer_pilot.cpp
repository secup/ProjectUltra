// OFDM pilot tracking and channel interpolation
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "pilot_pattern.hpp"
#include "wiener_interpolator.hpp"
#include "ofdm/genie_true_h.hpp"
#include "ultra/logging.hpp"
#include "ultra/phy_diagnostics.hpp"

namespace ultra {

using namespace demod_constants;

namespace {

// 2026-05-28: these are MODERATE-HF assumptions baked into the Wiener
// correlation model. Good HF is 0.5 ms / 0.1 Hz — the time-correlation
// model was throwing away older pilot observations 5x too aggressively
// on Good, hurting sparse-pilot performance. Env-tunable so we can
// test the Good-tuned values against the failing sp10_s2 seed.
// Read-once: these are called inside the per-carrier Wiener interpolation loop
// (lines ~243/287). getenv() is a linear scan of environ and not thread-safe, so
// re-reading per carrier is a real-time hazard on the equalizer hot path. The env
// is set at process start only (no mid-run putenv of PHY knobs), so a function-
// local static cache is behavior-neutral — same value, read once. Matches the
// read-once idiom in channel_equalizer_equalize.cpp (kRelFadeOnset).
inline float robustDelaySpreadS() {
    static const float cached = []() {
        if (const char* env = std::getenv("ULTRA_WIENER_DELAY_SPREAD_S")) {
            const float v = static_cast<float>(std::atof(env));
            if (v > 0.0f && v < 0.01f) return v;
        }
        return 1.0e-3f;
    }();
    return cached;
}
inline float robustDopplerHz() {
    static const float cached = []() {
        if (const char* env = std::getenv("ULTRA_WIENER_DOPPLER_HZ")) {
            const float v = static_cast<float>(std::atof(env));
            if (v > 0.0f && v < 10.0f) return v;
        }
        return 0.5f;
    }();
    return cached;
}
constexpr size_t kWienerMaxHistoryPerCarrier = 6;
constexpr size_t kWienerMaxTimeObs = 4;
constexpr size_t kWienerMaxFreqObs = 16;

bool coherentPublicFadingUsesLTS(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK:
        case Modulation::QPSK:
        case Modulation::QAM8:
        case Modulation::QAM16:
        case Modulation::QAM32:
        case Modulation::QAM64:
        case Modulation::QAM256:
            return true;
        default:
            return false;
    }
}

int diagnosticTwoPathDelaySamples() {
    // Read-once (genie/diag oracle, off by default). See robustDelaySpreadS note.
    static const int cached = []() {
        const char* value = std::getenv("ULTRA_QAM16_GENIE_CHANNEL_DELAY_SAMPLES");
        if (!value || value[0] == '\0') {
            return 24;  // Good channel: 0.5 ms at 48 kHz.
        }
        char* end = nullptr;
        const long parsed = std::strtol(value, &end, 10);
        if (end == value || parsed < 0 || parsed > 512) {
            return 24;
        }
        return static_cast<int>(parsed);
    }();
    return cached;
}

}  // namespace

void OFDMDemodulator::Impl::resetWienerPilotHistory() {
    if (wiener_pilot_history_.size() != config.num_carriers) {
        wiener_pilot_history_.assign(config.num_carriers, {});
    } else {
        for (auto& history : wiener_pilot_history_) {
            history.clear();
        }
    }
    wiener_time_estimate_.assign(config.num_carriers, Complex(0, 0));
    wiener_time_error_var_.assign(config.num_carriers, 1.0f);
    wiener_time_valid_.assign(config.num_carriers, 0);
    wiener_time_symbol_index_ = -999999;
}

int OFDMDemodulator::Impl::signedBinForLogicalCarrier(size_t logical_carrier) const {
    if (logical_carrier >= all_carrier_fft_indices.size()) return 0;
    const int fft_idx = all_carrier_fft_indices[logical_carrier];
    const int half_fft = static_cast<int>(config.fft_size / 2);
    return (fft_idx <= half_fft) ? fft_idx
                                 : fft_idx - static_cast<int>(config.fft_size);
}

void OFDMDemodulator::Impl::addWienerPilotObservation(size_t logical_carrier,
                                                      int64_t symbol_index,
                                                      Complex h,
                                                      float noise_norm) {
    if (logical_carrier >= config.num_carriers ||
        !std::isfinite(h.real()) || !std::isfinite(h.imag())) {
        return;
    }
    // FLAT STORAGE DOMAIN (ULTRA_ITERATIVE_CHEST only). Observations that will be
    // read by a LATER frame must not carry this frame's LTS timing ramp, so store
    // them already de-sloped; estimateWienerChannel() then skips its de-slope and
    // re-slopes only at the target. Off => untouched => byte-identical.
    if (wiener_history_flat_) {
        const float phase =
            -lts_phase_slope * static_cast<float>(signedBinForLogicalCarrier(logical_carrier));
        h *= Complex(std::cos(phase), std::sin(phase));
    }
    if (wiener_pilot_history_.size() != config.num_carriers) {
        wiener_pilot_history_.resize(config.num_carriers);
    }

    auto& history = wiener_pilot_history_[logical_carrier];
    if (!history.empty() && history.back().symbol_index == symbol_index) {
        history.back().h = h;
        history.back().noise_norm = std::clamp(noise_norm, 1.0e-5f, 10.0f);
    } else {
        history.push_back(WienerPilotHistorySample{
            symbol_index,
            h,
            std::clamp(noise_norm, 1.0e-5f, 10.0f)});
    }
    while (history.size() > kWienerMaxHistoryPerCarrier) {
        history.erase(history.begin());
    }
    wiener_time_symbol_index_ = -999999;
}

// ---------------------------------------------------------------------------
// COMMON-PHASE RE-REFERENCING for carried channel observations
// (ULTRA_ITERATIVE_CHEST only; called just before the new frame's LTS seed).
//
// Each frame is acquired independently: its mixer is reset, its CFO phase
// reference is re-derived from the frame's own absolute position, and its timing
// is re-solved from its own LTS. So the channel estimate of frame k and frame k+1
// describe the SAME physical channel in two different phase references:
//
//     H_k(f) = H_true(f) * exp(j*phi_k)      phi_k arbitrary per frame
//
// De-sloping removes the part of that which is LINEAR in carrier index (the
// timing ramp). It does NOT remove the COMMON phase, and averaging observations
// that differ by an unknown common phase destroys the estimate rather than
// improving it — the carried sample becomes confidently wrong, which is exactly
// what 16QAM's absolute-amplitude ring bit cannot absorb.
//
// The correction is the maximum-likelihood estimate of that single unknown:
// phi = arg( sum_k H_new(k) * conj(H_carried(k)) ) over every carrier where both
// exist. One complex accumulator, one atan2 — a 1-parameter LS fit, no threshold,
// no tuned constant, no per-modulation branch. Degenerate case (|sum| == 0, i.e.
// no overlap or a dead band) leaves the history untouched.
void OFDMDemodulator::Impl::rereferenceCarriedHistoryPhase() {
    if (wiener_pilot_history_.empty()) return;
    const size_t n = std::min<size_t>(config.num_carriers, all_carrier_fft_indices.size());
    Complex acc(0.0f, 0.0f);
    size_t overlap = 0;
    for (size_t logical = 0; logical < n && logical < wiener_pilot_history_.size();
         ++logical) {
        const auto& hist = wiener_pilot_history_[logical];
        if (hist.empty()) continue;
        const int idx = all_carrier_fft_indices[logical];
        Complex h_new = channel_estimate[idx];
        if (wiener_history_flat_) {
            const float phase =
                -lts_phase_slope * static_cast<float>(signedBinForLogicalCarrier(logical));
            h_new *= Complex(std::cos(phase), std::sin(phase));
        }
        acc += h_new * std::conj(hist.back().h);
        ++overlap;
    }
    const float mag = std::abs(acc);
    if (!(mag > 0.0f) || overlap == 0) return;
    const Complex rot = acc / mag;

    static std::atomic<int> log_count{0};
    if (log_count < 12) {
        LOG_DEMOD(INFO,
                  "ITERATIVE_CHEST: re-referencing %zu carried carriers by %+.1f deg "
                  "(|sum|=%.4f)",
                  overlap, std::arg(acc) * 180.0f / static_cast<float>(M_PI), mag);
        ++log_count;
    }

    for (auto& hist : wiener_pilot_history_) {
        for (auto& sample : hist) {
            sample.h *= rot;
        }
    }
    wiener_time_symbol_index_ = -999999;  // invalidate the cached time stage
}

void OFDMDemodulator::Impl::seedWienerPilotHistoryFromCurrentChannel(int64_t symbol_index) {
    // With the carry armed (ULTRA_ITERATIVE_CHEST) the previous frame's verified
    // data-aided observations are the whole point — do NOT wipe them here; the new
    // LTS simply joins the history as the freshest observation and the Wiener's own
    // rho(dt) decides how much each one is worth. Unarmed => reset as before.
    if (!wiener_carry_armed_) {
        resetWienerPilotHistory();
    } else {
        rereferenceCarriedHistoryPhase();
    }
    const size_t n = std::min<size_t>(config.num_carriers, all_carrier_fft_indices.size());
    for (size_t logical = 0; logical < n; ++logical) {
        const int idx = all_carrier_fft_indices[logical];
        const Complex h = channel_estimate[idx];
        const float signal_ref = std::max(std::norm(h), MIN_CARRIER_NOISE_VAR);
        const float noise_norm = noise_variance / signal_ref;
        addWienerPilotObservation(logical, symbol_index, h, noise_norm);
    }
}

void OFDMDemodulator::Impl::applyDiagnosticTwoPathChannelOracle(
        const std::vector<Complex>& h_ls_all) {
    // 2026-05-29: extend the genie two-path channel oracle to QAM8 as well, so we
    // can A/B whether the LTS+pilot H estimate is what breaks 8PSK on Good fading
    // (test-only, env-gated by ULTRA_QAM16_GENIE_CHANNEL_TWOPATH_LS).
    if (!qam16GenieChannelTwoPathEnabled() ||
        (config.modulation != Modulation::QAM16 &&
         config.modulation != Modulation::QAM8) ||
        h_ls_all.size() != pilot_carrier_indices.size() ||
        h_ls_all.size() < 2) {
        return;
    }

    const int delay_samples = diagnosticTwoPathDelaySamples();
    const float fft_size_f = static_cast<float>(config.fft_size);
    const float phase_per_bin =
        -2.0f * static_cast<float>(M_PI) *
        static_cast<float>(delay_samples) / fft_size_f;

    Complex s01(0.0f, 0.0f);
    Complex rhs0(0.0f, 0.0f);
    Complex rhs1(0.0f, 0.0f);
    const int half_fft = config.fft_size / 2;
    for (size_t i = 0; i < h_ls_all.size(); ++i) {
        const int idx = pilot_carrier_indices[i];
        const int signed_bin = (idx <= half_fft) ? idx : idx - config.fft_size;
        const Complex p = std::exp(Complex(0.0f, phase_per_bin * signed_bin));
        s01 += p;
        rhs0 += h_ls_all[i];
        rhs1 += std::conj(p) * h_ls_all[i];
    }

    const float n = static_cast<float>(h_ls_all.size());
    const Complex s00(n, 0.0f);
    const Complex s11(n, 0.0f);
    const Complex s10 = std::conj(s01);
    const Complex det = s00 * s11 - s01 * s10;
    if (std::abs(det) < 1.0e-6f) {
        return;
    }

    const Complex tap0 = (rhs0 * s11 - s01 * rhs1) / det;
    const Complex tap1 = (s00 * rhs1 - s10 * rhs0) / det;

    auto model_h = [&](int idx) {
        const int signed_bin = (idx <= half_fft) ? idx : idx - config.fft_size;
        const Complex p = std::exp(Complex(0.0f, phase_per_bin * signed_bin));
        return tap0 + tap1 * p;
    };

    for (int idx : data_carrier_indices) {
        channel_estimate[idx] = model_h(idx);
    }
    for (int idx : pilot_carrier_indices) {
        channel_estimate[idx] = model_h(idx);
    }

    static std::atomic<int> log_count{0};
    if (log_count < 8) {
        LOG_DEMOD(WARN,
                  "DIAG genie-channel two-path LS applied: delay=%d samples "
                  "symbol=%zu pilots=%zu tap0=%.3f%+.3fj tap1=%.3f%+.3fj",
                  delay_samples,
                  current_data_symbol_index_,
                  h_ls_all.size(),
                  tap0.real(), tap0.imag(),
                  tap1.real(), tap1.imag());
        ++log_count;
    }
}

Complex OFDMDemodulator::Impl::estimateWienerChannel(size_t logical_carrier,
                                                     int64_t symbol_index,
                                                     float noise_norm,
                                                     Complex fallback,
                                                     float* out_error_var) {
    if (logical_carrier >= config.num_carriers ||
        wiener_pilot_history_.empty()) {
        if (out_error_var) {
            *out_error_var = 1.0f;
        }
        return fallback;
    }

    const float symbol_period_s =
        static_cast<float>(config.getSymbolDuration()) /
        static_cast<float>(config.sample_rate);
    const float carrier_spacing_hz =
        static_cast<float>(config.sample_rate) /
        static_cast<float>(config.fft_size);

    if (wiener_time_symbol_index_ != symbol_index) {
        if (wiener_time_estimate_.size() != config.num_carriers) {
            wiener_time_estimate_.assign(config.num_carriers, Complex(0, 0));
            wiener_time_error_var_.assign(config.num_carriers, 1.0f);
            wiener_time_valid_.assign(config.num_carriers, 0);
        }

        for (size_t logical = 0; logical < wiener_pilot_history_.size(); ++logical) {
            std::vector<ofdm_wiener::Observation1D> time_obs;
            time_obs.reserve(wiener_pilot_history_[logical].size());
            for (const auto& sample : wiener_pilot_history_[logical]) {
                time_obs.push_back(ofdm_wiener::Observation1D{
                    static_cast<float>(sample.symbol_index),
                    sample.h,
                    sample.noise_norm});
            }
            const auto estimate = ofdm_wiener::estimate1D(
                time_obs,
                static_cast<float>(symbol_index),
                kWienerMaxTimeObs,
                [&](float delta_symbols) {
                    const float doppler = wiener_params_override_active_
                                              ? wiener_doppler_hz_override_
                                              : robustDopplerHz();
                    return ofdm_wiener::timeCorrelation(
                        delta_symbols, symbol_period_s, doppler);
                });
            if (estimate.valid) {
                wiener_time_estimate_[logical] = estimate.value;
                wiener_time_error_var_[logical] = estimate.error_var;
                wiener_time_valid_[logical] = 1;
            } else {
                wiener_time_valid_[logical] = 0;
                wiener_time_error_var_[logical] = 1.0f;
            }
        }
        wiener_time_symbol_index_ = symbol_index;
    }

    // ULTRA_PILOT_DFT_INTERP: the frequency axis of the separable Wiener is where
    // the reconstruction filter lives. Default (OFF) positions observations by
    // LOGICAL carrier index and correlates with sinc(pi*df*tau) — the transform of a
    // flat CONTINUOUS delay window, and blind to the DC gap (logical delta 1 across
    // k=-1 -> k=+1 is really 2 carriers of frequency). ON positions them by PHYSICAL
    // carrier index k and correlates with the Dirichlet kernel of an integer-sample
    // delay window: the exact "IDFT -> truncate CIR to its support -> DFT back"
    // operator for this (non-uniform, DC-gapped) pilot geometry, in closed form.
    // The time axis and the observation set are untouched, so the knob is a clean
    // reconstruction-filter A/B.
    const bool use_delay_domain =
        delay_domain_interp_ && delay_domain_window_.valid();

    std::vector<ofdm_wiener::Observation1D> freq_obs;
    freq_obs.reserve(config.num_carriers);
    const int half_fft = static_cast<int>(config.fft_size / 2);
    for (size_t logical = 0;
         logical < wiener_time_valid_.size() &&
         logical < all_carrier_fft_indices.size();
         ++logical) {
        if (!wiener_time_valid_[logical]) {
            continue;
        }
        const int fft_idx = all_carrier_fft_indices[logical];
        const int k = (fft_idx <= half_fft)
            ? fft_idx
            : fft_idx - static_cast<int>(config.fft_size);
        // Samples stored in the FLAT domain were already de-sloped at capture (see
        // addWienerPilotObservation) — de-sloping again would inject the ramp twice.
        const float phase = -lts_phase_slope * static_cast<float>(k);
        const Complex desloped =
            wiener_history_flat_
                ? wiener_time_estimate_[logical]
                : wiener_time_estimate_[logical] *
                      Complex(std::cos(phase), std::sin(phase));
        freq_obs.push_back(ofdm_wiener::Observation1D{
            use_delay_domain ? static_cast<float>(k)
                             : static_cast<float>(logical),
            desloped,
            std::max(noise_norm, wiener_time_error_var_[logical])});
    }

    const int target_fft_for_pos = all_carrier_fft_indices[logical_carrier];
    const int target_k_for_pos = (target_fft_for_pos <= half_fft)
        ? target_fft_for_pos
        : target_fft_for_pos - static_cast<int>(config.fft_size);

    const auto freq_estimate = use_delay_domain
        ? ofdm_cir::reconstruct(freq_obs,
                                static_cast<float>(target_k_for_pos),
                                delay_domain_window_.taps,
                                config.fft_size,
                                kWienerMaxFreqObs)
        : ofdm_wiener::estimate1D(
              freq_obs,
              static_cast<float>(logical_carrier),
              kWienerMaxFreqObs,
              [&](float delta_logical) {
                  const float delay = wiener_params_override_active_
                                          ? wiener_delay_spread_s_override_
                                          : robustDelaySpreadS();
                  return ofdm_wiener::frequencyCorrelation(
                      delta_logical, carrier_spacing_hz, delay);
              });
    if (!freq_estimate.valid) {
        if (out_error_var) {
            *out_error_var = 1.0f;
        }
        return fallback;
    }

    const float phase = lts_phase_slope * static_cast<float>(target_k_for_pos);
    if (out_error_var) {
        *out_error_var = freq_estimate.error_var;
    }
    return freq_estimate.value * Complex(std::cos(phase), std::sin(phase));
}

void OFDMDemodulator::Impl::resetPilotFadingStats() {
    pilot_mag_sum_.clear();
    pilot_mag_sq_sum_.clear();
    pilot_symbol_mean_sum_ = 0.0f;
    pilot_symbol_mean_sq_sum_ = 0.0f;
    pilot_fading_symbol_count_ = 0;
    last_pilot_frequency_cv = 0.0f;
    last_pilot_temporal_cv = 0.0f;
    last_pilot_symbol_mean_cv = 0.0f;
    public_fading_index = 0.0f;
}

float OFDMDemodulator::Impl::computePilotFadingIndexFromStats() const {
    if (pilot_fading_symbol_count_ == 0 ||
        pilot_mag_sum_.empty() ||
        pilot_mag_sum_.size() != pilot_mag_sq_sum_.size()) {
        return last_fading_index;
    }

    const float inv_count =
        1.0f / static_cast<float>(pilot_fading_symbol_count_);
    const float snr_linear = last_snr_db_estimate_valid
        ? std::pow(10.0f, last_snr_db_estimate / 10.0f)
        : 0.0f;
    const float pilot_mag_noise_cv2 =
        (snr_linear > 1.0f && std::isfinite(snr_linear))
            ? 0.25f / snr_linear
            : 0.0f;

    float freq_mean_sum = 0.0f;
    size_t freq_count = 0;
    for (float sum : pilot_mag_sum_) {
        const float mean = sum * inv_count;
        if (mean > 0.01f && std::isfinite(mean)) {
            freq_mean_sum += mean;
            ++freq_count;
        }
    }
    if (freq_count == 0) {
        return 0.0f;
    }

    const float freq_mean = freq_mean_sum / static_cast<float>(freq_count);
    float freq_var_sum = 0.0f;
    float temporal_cv_sum = 0.0f;
    float temporal_raw_cv_sum = 0.0f;
    float temporal_cv2_max = 0.0f;
    float freq_min = std::numeric_limits<float>::max();
    float freq_max = 0.0f;
    size_t temporal_count = 0;
    for (size_t i = 0; i < pilot_mag_sum_.size(); ++i) {
        const float mean = pilot_mag_sum_[i] * inv_count;
        if (mean <= 0.01f || !std::isfinite(mean)) {
            continue;
        }
        freq_min = std::min(freq_min, mean);
        freq_max = std::max(freq_max, mean);

        const float freq_diff = mean - freq_mean;
        freq_var_sum += freq_diff * freq_diff;

        const float mean_sq = pilot_mag_sq_sum_[i] * inv_count;
        const float temporal_var = std::max(0.0f, mean_sq - mean * mean);
        const float temporal_cv2 = temporal_var / (mean * mean);
        temporal_raw_cv_sum += std::sqrt(std::max(0.0f, temporal_cv2));
        temporal_cv2_max = std::max(temporal_cv2_max, temporal_cv2);
        temporal_cv_sum +=
            std::sqrt(std::max(0.0f, temporal_cv2 - pilot_mag_noise_cv2));
        ++temporal_count;
    }

    const float freq_cv2 = (freq_mean > 0.01f)
        ? (freq_var_sum / static_cast<float>(freq_count)) / (freq_mean * freq_mean)
        : 0.0f;
    const float freq_noise_cv2 = pilot_mag_noise_cv2 * inv_count;
    const float freq_cv_raw = std::sqrt(std::max(0.0f, freq_cv2));
    const float freq_cv = std::sqrt(std::max(0.0f, freq_cv2 - freq_noise_cv2));
    const float temporal_cv_raw = (temporal_count > 0)
        ? temporal_raw_cv_sum / static_cast<float>(temporal_count)
        : 0.0f;
    const float temporal_cv = (temporal_count > 0)
        ? temporal_cv_sum / static_cast<float>(temporal_count)
        : 0.0f;
    float symbol_mean_cv_raw = 0.0f;
    if (pilot_fading_symbol_count_ > 0) {
        const float symbol_mean =
            pilot_symbol_mean_sum_ / static_cast<float>(pilot_fading_symbol_count_);
        const float symbol_mean_sq =
            pilot_symbol_mean_sq_sum_ / static_cast<float>(pilot_fading_symbol_count_);
        const float symbol_mean_var =
            std::max(0.0f, symbol_mean_sq - symbol_mean * symbol_mean);
        symbol_mean_cv_raw = (symbol_mean > 0.01f)
            ? std::sqrt(symbol_mean_var) / symbol_mean
            : 0.0f;
    }

    const float fading_index = freq_cv + temporal_cv;
    last_pilot_frequency_cv = freq_cv;
    last_pilot_temporal_cv = temporal_cv;
    last_pilot_symbol_mean_cv = symbol_mean_cv_raw;
    // Phase 5 diag: publish the intra-frame time-selectivity measure. LAST-write-wins
    // is correct here (the stats accumulate across the frame); see genie_true_h.hpp.
    if (ultra::ofdm::genie_true_h::mode() == ultra::ofdm::genie_true_h::Mode::Capture) {
        ultra::ofdm::genie_true_h::capturedSymbolMeanCv() = symbol_mean_cv_raw;
    }
    if (phyDiagnosticsEnabled()) {
        char line[768];
        std::snprintf(line, sizeof(line),
                      "event=pilot_fading_stats mod=%d rate=%d symbols=%zu "
                      "pilots=%zu snr_db=%.2f snr_linear=%.3f "
                      "pilot_noise_cv2=%.6f freq_noise_cv2=%.6f "
                      "freq_mean=%.6f freq_min=%.6f freq_max=%.6f "
                      "freq_cv_raw=%.6f freq_cv=%.6f "
                      "temporal_cv_raw=%.6f temporal_cv=%.6f "
                      "temporal_cv2_max=%.6f symbol_mean_cv_raw=%.6f "
                      "public_fading=%.6f "
                      "instant_fading=%.6f",
                      static_cast<int>(config.modulation),
                      static_cast<int>(config.code_rate),
                      pilot_fading_symbol_count_,
                      temporal_count,
                      last_snr_db_estimate_valid ? last_snr_db_estimate : 0.0f,
                      snr_linear,
                      pilot_mag_noise_cv2,
                      freq_noise_cv2,
                      freq_mean,
                      freq_min == std::numeric_limits<float>::max() ? 0.0f : freq_min,
                      freq_max,
                      freq_cv_raw,
                      freq_cv,
                      temporal_cv_raw,
                      temporal_cv,
                      temporal_cv2_max,
                      symbol_mean_cv_raw,
                      fading_index,
                      last_fading_index);
        phyDiagLine(line);
    }

    return fading_index;
}

void OFDMDemodulator::Impl::updatePilotFadingStats(const std::vector<Complex>& h_ls_all) {
    if (h_ls_all.empty()) {
        public_fading_index = last_fading_index;
        return;
    }

    if (pilot_mag_sum_.size() != h_ls_all.size() ||
        pilot_mag_sq_sum_.size() != h_ls_all.size()) {
        pilot_mag_sum_.assign(h_ls_all.size(), 0.0f);
        pilot_mag_sq_sum_.assign(h_ls_all.size(), 0.0f);
        pilot_fading_symbol_count_ = 0;
    }

    float symbol_mag_sum = 0.0f;
    float symbol_mag_min = std::numeric_limits<float>::max();
    float symbol_mag_max = 0.0f;
    for (const Complex& h : h_ls_all) {
        const float mag = std::abs(h);
        symbol_mag_sum += mag;
        symbol_mag_min = std::min(symbol_mag_min, mag);
        symbol_mag_max = std::max(symbol_mag_max, mag);
    }
    const float symbol_mag_mean =
        symbol_mag_sum / static_cast<float>(h_ls_all.size());
    // PSYM-DIAG (read-only, env ULTRA_PSYM_DIAG): per-symbol carrier-averaged |H| for the
    // within-frame coherence redesign (task #57 / WITHIN_FRAME_COHERENCE_DESIGN_2026_06_17).
    // Symbol-cadence |H|(t) to find the discriminating sub-frame lag (~0.5-0.85s) that
    // separates Good from Moderate where per-frame snapshots can't. Frame boundaries are
    // recovered offline from the interleaved per-frame COH-DIAG lines in the same log.
    {
        static const bool kPsymDiag = [] {
            const char* e = std::getenv("ULTRA_PSYM_DIAG");
            return e && e[0] != '\0' && !(e[0] == '0' && e[1] == '\0');
        }();
        if (kPsymDiag && std::isfinite(symbol_mag_mean)) {
            static thread_local unsigned long long psym_counter = 0;
            LOG_DEMOD(INFO, "PSYM-DIAG sym=%llu hmag=%.6g", psym_counter++, symbol_mag_mean);
        }
    }
    if (symbol_mag_mean <= 0.01f || !std::isfinite(symbol_mag_mean)) {
        public_fading_index = last_fading_index;
        return;
    }

    for (size_t i = 0; i < h_ls_all.size(); ++i) {
        const float mag = std::abs(h_ls_all[i]);
        pilot_mag_sum_[i] += mag;
        pilot_mag_sq_sum_[i] += mag * mag;
    }
    pilot_symbol_mean_sum_ += symbol_mag_mean;
    pilot_symbol_mean_sq_sum_ += symbol_mag_mean * symbol_mag_mean;
    ++pilot_fading_symbol_count_;

    public_fading_index = computePilotFadingIndexFromStats();

    if (phyDiagnosticsEnabled()) {
        float symbol_mag_var = 0.0f;
        for (const Complex& h : h_ls_all) {
            const float diff = std::abs(h) - symbol_mag_mean;
            symbol_mag_var += diff * diff;
        }
        symbol_mag_var /= static_cast<float>(h_ls_all.size());
        const float symbol_cv = (symbol_mag_mean > 0.01f)
            ? std::sqrt(symbol_mag_var) / symbol_mag_mean
            : 0.0f;
        char line[512];
        std::snprintf(line, sizeof(line),
                      "event=pilot_fading_symbol mod=%d rate=%d symbol=%zu "
                      "pilots=%zu raw_mean=%.6f raw_min=%.6f raw_max=%.6f "
                      "raw_cv=%.6f public_fading=%.6f instant_fading=%.6f",
                      static_cast<int>(config.modulation),
                      static_cast<int>(config.code_rate),
                      pilot_fading_symbol_count_,
                      h_ls_all.size(),
                      symbol_mag_mean,
                      symbol_mag_min == std::numeric_limits<float>::max() ? 0.0f : symbol_mag_min,
                      symbol_mag_max,
                      symbol_cv,
                      public_fading_index,
                      last_fading_index);
        phyDiagLine(line);
    }
}

void OFDMDemodulator::Impl::updateChannelEstimate(const std::vector<Complex>& freq_domain) {
    // Smoothing factor for channel estimate update (coherent-only OFDM):
    // - First data symbol: alpha=1.0 (use the pilot estimate directly; the channel
    //   has changed since the LTS).
    // - Subsequent symbols: alpha=0.9 to track channel changes quickly.
    bool has_pilots = !pilot_carrier_indices.empty();
    if (!has_pilots) {
        pilot_phase_correction = Complex(1, 0);
        prev_pilot_phases.clear();
        prev_pilot_logical_indices.clear();
        ++snr_symbol_count;
        return;
    }

    // Use soft_bits.empty() to detect first DATA symbol (snr_symbol_count may be > 0 from LTS)
    bool is_first_data_symbol = soft_bits.empty();


    float alpha;
    if (is_first_data_symbol) {
        alpha = 1.0f;  // First data symbol: use pilot estimate directly (channel changed since LTS)
    } else {
        alpha = 0.9f;  // Coherent: track channel changes
    }

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
    // Skip for coherent modes (QPSK, BPSK) — the LTS provides accurate H that includes
    // the correct channel phase. MMSE equalization (conj(H)*rx / |H|²+σ²) naturally
    // removes the phase. Applying carrier_phase_correction removes phase from H but NOT
    // from the received signal, leaving a residual rotation in the equalized output.
    if (!carrier_phase_initialized && !pilot_carrier_indices.empty()) {
        carrier_phase_initialized = true;  // Mark as done (identity correction)
        LOG_DEMOD(DEBUG, "Carrier phase recovery: SKIPPED for coherent mode (LTS provides accurate H)");
    }

    // Apply carrier phase correction to all H estimates (identity for coherent modes)
    for (size_t i = 0; i < h_ls_all.size(); ++i) {
        h_ls_all[i] *= carrier_phase_correction;
    }
    h_sum *= carrier_phase_correction;

    // CPE (Common Phase Error) correction. Estimate average phase drift from pilot
    // LS vs current H, apply to all carriers. This tracks residual CFO and slow
    // oscillator drift without modifying freq_offset_hz. Standard approach used in
    // WiFi 802.11a/g/n receivers.
    {
        Complex cpe_sum(0, 0);
        float cpe_weight_sum = 0.0f;
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            Complex h_old = channel_estimate[idx];
            float h_old_mag = std::abs(h_old);
            if (h_old_mag > 0.01f) {
                // Phase difference between new pilot LS and current channel estimate
                Complex ratio = h_ls_all[i] * std::conj(h_old);
                float mag = std::abs(ratio);
                if (mag > 1e-6f) {
                    cpe_sum += (ratio / mag) * h_old_mag;  // Weight by channel strength
                    cpe_weight_sum += h_old_mag;
                }
            }
        }
        if (cpe_weight_sum > 0.01f) {
            float cpe_phase = std::arg(cpe_sum);

            if (std::abs(cpe_phase) > 0.001f) {  // Skip if negligible
                Complex cpe_correction = std::exp(Complex(0.0f, cpe_phase));
                // Apply CPE to ALL carrier H estimates (pilot + data)
                for (int idx : data_carrier_indices) {
                    channel_estimate[idx] *= cpe_correction;
                }
                for (int idx : pilot_carrier_indices) {
                    channel_estimate[idx] *= cpe_correction;
                }
                static std::atomic<int> cpe_log_count{0};
                if (cpe_log_count < 10) {
                    LOG_DEMOD(DEBUG, "CPE correction: %.2f° (from %zu pilots)",
                              cpe_phase * 180.0f / M_PI, pilot_carrier_indices.size());
                    cpe_log_count++;
                }
            }
        }
    }

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
    const float wiener_noise_norm =
        noise_variance / std::max(signal_power, MIN_CARRIER_NOISE_VAR);
    // Data-aided observations must be pushed on the SAME normalisation as the pilot
    // observations they will be combined with, or the Wiener's relative weighting is
    // meaningless. Latch the pilot-referenced value; the per-carrier 1/|X|^2 factor
    // is applied at ingest (iterative_chest.hpp).
    da_noise_norm_ = wiener_noise_norm;
    for (size_t i = 0;
         i < h_ls_all.size() && i < pilot_logical_carrier_indices.size();
         ++i) {
        addWienerPilotObservation(pilot_logical_carrier_indices[i],
                                  wiener_symbol_base_ +
                                      static_cast<int64_t>(current_data_symbol_index_),
                                  h_ls_all[i],
                                  wiener_noise_norm);
        // F166 Stage B: persist the RAW observation power (pre-Wiener) — the
        // interpolator's sinc prior + diagonal loading smooths parked notches
        // SHALLOW (its own comment: "may smooth THROUGH deep nulls"), which is
        // the D1 confident-wrong mechanism. Always written; consumers gate.
        if (i < pilot_carrier_indices.size()) {
            const int fft_idx = pilot_carrier_indices[i];
            if (fft_idx >= 0 &&
                static_cast<size_t>(fft_idx) < per_carrier_raw_obs_power_.size()) {
                per_carrier_raw_obs_power_[fft_idx] = std::norm(h_ls_all[i]);
                per_carrier_raw_obs_symbol_[fft_idx] =
                    static_cast<int64_t>(current_data_symbol_index_);
            }
        }
    }

    // Fading index: normalized magnitude variance (0 = flat, >0.1 = fading).
    // Compute it before temporal SNR estimation because H[n]-H[n-1] is a valid
    // noise proxy only when the pilot channel is near-static across symbols.
    float h_mag_mean = 0.0f;
    for (size_t i = 0; i < h_ls_all.size(); ++i) {
        h_mag_mean += std::abs(h_ls_all[i]);
    }
    h_mag_mean /= h_ls_all.size();

    float h_mag_variance = 0.0f;
    for (size_t i = 0; i < h_ls_all.size(); ++i) {
        float diff = std::abs(h_ls_all[i]) - h_mag_mean;
        h_mag_variance += diff * diff;
    }
    h_mag_variance /= h_ls_all.size();

    const float fading_index =
        (h_mag_mean > 0.01f) ? std::sqrt(h_mag_variance) / h_mag_mean : 0.0f;
    last_fading_index = fading_index;
    if (coherentPublicFadingUsesLTS(config.modulation)) {
        // Coherent QAM/BPSK data-symbol pilot magnitudes are useful for
        // equalization, but the real-passband coherent payload can imprint
        // symbol-dependent pilot magnitude ripple that is not RF fading. Keep
        // the public meter on the payload-independent LTS channel estimate
        // while still retaining pilot frequency/temporal components as a
        // measurement-validity cross-check for static notches.
        const float lts_public_fading = public_fading_index;
        updatePilotFadingStats(h_ls_all);
        public_fading_index = lts_public_fading;
    } else {
        updatePilotFadingStats(h_ls_all);
    }

    // Phase-1 transfer runs showed temporal pilot-channel residuals are too
    // easily contaminated by channel-estimate motion during long OFDM frames.
    // Keep the in-band SNR estimate anchored to the same-frame LTS residual;
    // pilots continue to update fading/equalization below.
    size_t pilot_residual_count = 0;
    constexpr float kPilotSNRFadingLimit = 0.0f;
    if (fading_index < kPilotSNRFadingLimit &&
        !prev_pilot_phases.empty() && prev_pilot_phases.size() == h_ls_all.size() &&
        prev_pilot_logical_indices.size() == pilot_logical_carrier_indices.size()) {
        Complex temporal_fit_num(0.0f, 0.0f);
        float temporal_fit_den = 0.0f;
        for (size_t i = 0; i < h_ls_all.size(); ++i) {
            if (prev_pilot_logical_indices[i] != pilot_logical_carrier_indices[i]) {
                continue;
            }
            const Complex prev_h = prev_pilot_phases[i];
            const Complex curr_h = h_ls_all[i];
            if (std::norm(prev_h) > 1.0e-8f && std::norm(curr_h) > 1.0e-8f) {
                temporal_fit_num += curr_h * std::conj(prev_h);
                temporal_fit_den += std::norm(prev_h);
            }
        }

        if (temporal_fit_den > 1.0e-8f) {
            const Complex common_gain = temporal_fit_num / temporal_fit_den;
            float pilot_residual_signal_sum = 0.0f;
            float pilot_residual_noise_sum = 0.0f;
            for (size_t i = 0; i < h_ls_all.size(); ++i) {
                if (prev_pilot_logical_indices[i] != pilot_logical_carrier_indices[i]) {
                    continue;
                }
                const Complex prev_h = prev_pilot_phases[i];
                const Complex curr_h = h_ls_all[i];
                if (std::norm(prev_h) <= 1.0e-8f || std::norm(curr_h) <= 1.0e-8f) {
                    continue;
                }

                const Complex predicted = common_gain * prev_h;
                const Complex residual = curr_h - predicted;
                pilot_residual_signal_sum += std::norm(curr_h);
                pilot_residual_noise_sum += 0.5f * std::norm(residual);
                ++pilot_residual_count;
            }

            if (pilot_residual_count > 0) {
                updateLastSNREstimate(
                    pilot_residual_signal_sum / static_cast<float>(pilot_residual_count),
                    pilot_residual_noise_sum / static_cast<float>(pilot_residual_count),
                    pilot_residual_count,
                    0.35f,
                    true);
            }
        }
    }

    // Measure noise using TEMPORAL comparison
    float noise_power_sum = 0.0f;
    size_t noise_count = 0;

    for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
        int idx = pilot_carrier_indices[i];

        if (!prev_pilot_phases.empty() && i < prev_pilot_phases.size() &&
            i < prev_pilot_logical_indices.size() &&
            i < pilot_logical_carrier_indices.size() &&
            prev_pilot_logical_indices[i] == pilot_logical_carrier_indices[i]) {
            Complex prev_h = prev_pilot_phases[i];
            Complex curr_h = h_ls_all[i];

            if (std::norm(prev_h) > 1e-6f && std::norm(curr_h) > 1e-6f) {
                Complex diff = curr_h - prev_h;
                noise_power_sum += std::norm(diff);
                noise_count++;
            }
        }

        // Update smoothed channel estimate (coherent complex LS smoothing)
        Complex h_old = channel_estimate[idx];
        channel_estimate[idx] = alpha * h_ls_all[i] + (1.0f - alpha) * h_old;
    }

    // First symbol fallback: assume 15 dB SNR
    if (noise_count == 0) {
        noise_power_sum = signal_power / DEFAULT_SNR_LINEAR;
        noise_count = 1;
    }

    // === Frequency offset estimation from pilot phase differences ===
    // DISABLED for all modes:
    // - Differential: fading-induced pilot phase changes corrupt CFO estimate.
    // - Coherent: noise causes progressive freq_offset_hz drift → growing phase error
    //   (measured 22° on AWGN at SNR=100, growing to 25° by symbol 10).
    // CPE correction (above) handles residual phase drift for coherent modes.
    // Chirp/LTS CFO provides the initial frequency offset.
    constexpr bool enable_pilot_cfo_tracking = false;
    if (enable_pilot_cfo_tracking && !prev_pilot_phases.empty() && prev_pilot_phases.size() == h_ls_all.size() &&
        prev_pilot_logical_indices.size() == pilot_logical_carrier_indices.size()) {
        Complex phase_diff_sum(0, 0);
        int valid_count = 0;

        for (size_t i = 0; i < h_ls_all.size(); ++i) {
            if (prev_pilot_logical_indices[i] != pilot_logical_carrier_indices[i]) {
                continue;
            }
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

    // Store current pilots for next symbol
    prev_pilot_phases = h_ls_all;
    prev_pilot_logical_indices = pilot_logical_carrier_indices;

    // Interpolate between pilots
    // 2026-05-28 REVERTED: QPSK DD attempted but regressed baseline (sp5 12-pilot
    // went FAIL on seed 2 with DD on — bad hard decisions during fades poison
    // the channel estimate and cascade). DD is only safe on QAM8/QAM16 today.
    // Worth re-attempting with a reliability gate (only DD a symbol when its
    // EVM is well inside the 95% Gaussian noise radius), but that's bigger work.
    // Optional opt-in via ULTRA_QPSK_DD=1 for further experimentation.
    const bool qpsk_dd_optin = []() {
        if (const char* env = std::getenv("ULTRA_QPSK_DD")) {
            return std::atoi(env) == 1;
        }
        return false;
    }();
    // 2026-05-29 test knob: ULTRA_COHERENT_DD_OFF=1 disables decision-directed
    // channel tracking, to A/B whether DD error-propagation on fading is what
    // breaks 8PSK on Good@20 (the comment above warns DD poisons H on bad
    // decisions; this lets us test that hypothesis directly).
    const bool dd_force_off = []() {
        if (const char* env = std::getenv("ULTRA_COHERENT_DD_OFF")) {
            return std::atoi(env) == 1;
        }
        return false;
    }();
    // 2026-05-29 BUG-8PSK-001 — channel-adaptive DD gating. Decision-directed (DD)
    // channel tracking is the WRONG TOOL on a slowly-fading, frequency-selective
    // channel. Good HF (~0.1 Hz Doppler) is frozen over the whole burst (coherence
    // time ~4 s), so there is no time variation for DD to track; its ~0.5 ms delay
    // spread instead puts a frequency-selective NULL in the band. In the null,
    // per-carrier SNR is low, hard decisions go wrong (tight for QAM8/QAM16), and DD
    // feeds those confident-wrong decisions back into H — poisoning it and cascading
    // into confident-wrong bits. Measured: DD-on FAILs 8PSK Good@20 while DD-off
    // delivers (1010 bps). A per-symbol innovation gate is ineffective here: a
    // wrong-decision rotation and a legitimate between-pilot interpolation error both
    // produce large innovations, so no per-symbol test separates them (verified — a
    // 4x gate-tightness sweep was flat). DD only earns its keep where hard decisions
    // are reliable: a frequency-FLAT channel (AWGN, or a momentarily-flat fade). So
    // gate it on the measured frequency-selectivity (last_fading_index, pilot
    // magnitude CV): AWGN reads ~0.02 (max ~0.07), Good ~0.34 (median), so 0.15
    // cleanly separates and matches the codebase's existing "faded" boundary (LLR
    // scaling onset). Adapts per-frame and per-modulation by construction — no
    // per-mode special-case. See docs/ADAPTIVITY_AUDIT_2026_05_29.md. Env-overridable
    // while validating; 0.15 is the principled default. DD remains the right tool for
    // a genuinely fast-fading channel (high Doppler) — that is its proper domain.
    const float dd_fading_max = []() {
        if (const char* env = std::getenv("ULTRA_DD_FADING_MAX")) {
            const float v = static_cast<float>(std::atof(env));
            if (v > 0.0f) return v;
        }
        return 0.15f;
    }();
    const bool channel_flat_enough_for_dd = last_fading_index < dd_fading_max;
    const bool use_coherent_dd =
        !dd_force_off &&
        channel_flat_enough_for_dd &&
        (config.modulation == Modulation::QAM8 ||
         config.modulation == Modulation::QAM16 ||
         (config.modulation == Modulation::QPSK && qpsk_dd_optin));
    const bool have_qam16_dd =
        use_coherent_dd &&
        dd_qam16_channel_observations_.size() == config.fft_size &&
        dd_qam16_measurement_var_.size() == config.fft_size &&
        dd_qam16_reliability_.size() == config.fft_size;

    // ULTRA_PILOT_DFT_INTERP (default OFF): delay-domain (CIR) reconstruction of the
    // data-carrier channel. Re-read every symbol (NOT a latching `static const`) so a
    // single binary can run both arms of an A/B. One getenv per OFDM symbol, matching
    // the dd_force_off / dd_fading_max idiom directly above; the per-CARRIER hot loop
    // never touches the environment.
    delay_domain_interp_ = []() {
        if (const char* env = std::getenv("ULTRA_PILOT_DFT_INTERP")) {
            return std::atoi(env) == 1;
        }
        return false;
    }();
    if (delay_domain_interp_) {
        // Window span = the SAME channel delay assumption the Wiener frequency prior
        // uses (sinc(pi*df*tau) is the transform of a flat delay window of width tau),
        // so the knob swaps the RECONSTRUCTION FILTER and not the channel model:
        // channel-class override when the decoder has a Doppler-coherence verdict,
        // else ULTRA_WIENER_DELAY_SPREAD_S / 1 ms. ULTRA_PILOT_DFT_SPAN_MS overrides
        // it for sweeping the window independently of the Wiener prior.
        float span_s = wiener_params_override_active_
                           ? wiener_delay_spread_s_override_
                           : robustDelaySpreadS();
        if (const char* env = std::getenv("ULTRA_PILOT_DFT_SPAN_MS")) {
            const float ms = static_cast<float>(std::atof(env));
            if (ms > 0.0f && ms < 20.0f) {
                span_s = ms * 1.0e-3f;
            }
        }
        delay_domain_window_ = ofdm_cir::chooseDelayWindow(
            span_s, config.sample_rate, config.fft_size,
            config.pilot_spacing, config.getCyclicPrefix());
        static std::atomic<int> window_log_count{0};
        if (window_log_count < 3) {
            LOG_DEMOD(INFO,
                      "Delay-domain pilot interp: span=%.2f ms -> W=%d taps "
                      "(+-%.2f ms), alias_clamped=%d cp_clamped=%d, "
                      "unambiguous=%d samples (fft=%u, pilot_spacing=%u, cp=%u)",
                      span_s * 1.0e3f, delay_domain_window_.taps,
                      1.0e3f * delay_domain_window_.half_taps /
                          static_cast<float>(config.sample_rate),
                      delay_domain_window_.alias_clamped ? 1 : 0,
                      delay_domain_window_.cp_clamped ? 1 : 0,
                      config.pilot_spacing > 0
                          ? static_cast<int>(config.fft_size / config.pilot_spacing)
                          : static_cast<int>(config.fft_size),
                      config.fft_size, config.pilot_spacing,
                      config.getCyclicPrefix());
            ++window_log_count;
        }
    } else {
        delay_domain_window_ = ofdm_cir::DelayWindow{};
    }
    {
        // Coherent pilot interpolation: phase-slope-compensated complex interpolation
        // (OFDM is coherent-only; the differential magnitude-only branch was removed).
        // The timing offset introduces a phase gradient across carriers; de-sloping before
        // interpolation prevents phase wrapping. When slope is near zero (good timing),
        // de-slope is identity — complex interpolation still preserves phase info that
        // magnitude-only interpolation would discard.
        int half_fft = config.fft_size / 2;
        if (use_coherent_dd && dd_qam16_channel_var_.size() != config.fft_size) {
            dd_qam16_channel_var_.assign(
                config.fft_size,
                std::max(noise_variance, MIN_CARRIER_NOISE_VAR));
        }

        // Precompute de-sloped pilot H values
        std::vector<Complex> pilot_desloped(pilot_carrier_indices.size());
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int fft_idx = pilot_carrier_indices[i];
            int k = (fft_idx <= half_fft) ? fft_idx : fft_idx - config.fft_size;
            float phase = -lts_phase_slope * k;
            pilot_desloped[i] = channel_estimate[fft_idx] * Complex(std::cos(phase), std::sin(phase));
        }

        const bool use_wiener_interpolation =
            ofdm_pilots::scatteredPilotsActive(config);

        // ULTRA_PILOT_DFT_INTERP: when the Wiener path is NOT active (non-scattered
        // pilots / differential modulation), the reconstruction filter is the plain
        // linear interpolation above. Replace it with the delay-domain MMSE fit over
        // THIS symbol's de-sloped pilots, positioned by PHYSICAL carrier index so the
        // DC gap (k = -1 -> +1 is two carriers, not one) is honoured.
        // When the Wiener path IS active the swap happens inside
        // estimateWienerChannel() instead — same kernel, richer observation set
        // (time-extrapolated per-carrier estimates rather than one symbol's pilots).
        delay_domain_pilot_obs_.clear();
        if (delay_domain_interp_ && delay_domain_window_.valid() &&
            !use_wiener_interpolation && pilot_carrier_indices.size() >= 2) {
            delay_domain_pilot_obs_.reserve(pilot_carrier_indices.size());
            for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                const int fft_idx = pilot_carrier_indices[i];
                const int k = (fft_idx <= half_fft) ? fft_idx
                                                    : fft_idx - config.fft_size;
                delay_domain_pilot_obs_.push_back(ofdm_wiener::Observation1D{
                    static_cast<float>(k),
                    pilot_desloped[i],
                    wiener_noise_norm});
            }
        }

        for (size_t dc = 0; dc < interp_table.size(); ++dc) {
            const auto& info = interp_table[dc];

            // Find desloped values for the neighboring pilots
            Complex h_lower(0, 0), h_upper(0, 0);
            if (info.lower_pilot >= 0) {
                for (size_t p = 0; p < pilot_carrier_indices.size(); ++p) {
                    if (pilot_carrier_indices[p] == info.lower_pilot) {
                        h_lower = pilot_desloped[p];
                        break;
                    }
                }
            }
            if (info.upper_pilot >= 0) {
                for (size_t p = 0; p < pilot_carrier_indices.size(); ++p) {
                    if (pilot_carrier_indices[p] == info.upper_pilot) {
                        h_upper = pilot_desloped[p];
                        break;
                    }
                }
            }

            // Complex linear interpolation in de-sloped domain
            Complex interp_h(0, 0);
            if (info.lower_pilot >= 0 && info.upper_pilot >= 0) {
                interp_h = (1.0f - info.alpha) * h_lower + info.alpha * h_upper;
            } else if (info.lower_pilot >= 0) {
                interp_h = h_lower;
            } else {
                interp_h = h_upper;
            }

            // Re-slope at data carrier position
            int k = (info.fft_idx <= half_fft) ? info.fft_idx : info.fft_idx - config.fft_size;
            float phase = lts_phase_slope * k;
            const Complex reslope(std::cos(phase), std::sin(phase));
            Complex pilot_h = interp_h * reslope;
            float wiener_error_var = 1.0f;
            bool have_model_error_var = false;
            if (use_wiener_interpolation &&
                dc < data_logical_carrier_indices.size()) {
                pilot_h = estimateWienerChannel(
                    data_logical_carrier_indices[dc],
                    wiener_symbol_base_ +
                        static_cast<int64_t>(current_data_symbol_index_),
                    wiener_noise_norm,
                    pilot_h,
                    &wiener_error_var);
                have_model_error_var = true;
            } else if (!delay_domain_pilot_obs_.empty()) {
                const auto est = ofdm_cir::reconstruct(
                    delay_domain_pilot_obs_,
                    static_cast<float>(k),
                    delay_domain_window_.taps,
                    config.fft_size,
                    delay_domain_pilot_obs_.size());
                if (est.valid && std::isfinite(est.value.real()) &&
                    std::isfinite(est.value.imag())) {
                    // Reconstructed in the de-sloped domain; re-slope like the
                    // linear path it replaces.
                    pilot_h = est.value * reslope;
                    wiener_error_var = est.error_var;
                    have_model_error_var = true;
                }
            }

            // 2026-06-12 Phase 2b: persist the per-carrier normalized H-estimate error
            // variance for the eps_H LLR term in equalize() (ULTRA_HERR_LLR_K). Wiener
            // returns the normalized MMSE residual in [0,1]; the non-Wiener path uses the
            // same 0.04 (=0.20^2) floor the DD-Kalman fallback uses below. Always written
            // (the consume side gates on the env knob, so default behavior is unchanged).
            if (info.fft_idx >= 0 &&
                static_cast<size_t>(info.fft_idx) < per_carrier_h_error_var_.size()) {
                per_carrier_h_error_var_[info.fft_idx] =
                    have_model_error_var
                        ? std::clamp(wiener_error_var, 0.0f, 1.0f)
                        : 0.04f;
            }

            if (use_coherent_dd) {
                // Coherent 8PSK/16-QAM DD tracking. Each data carrier is a scalar
                // complex Kalman state H[k]. The previous H is predicted with a
                // Good-channel Doppler random walk, pilot interpolation is the
                // first measurement/anchor, and a reliable hard decision is the
                // second measurement H_dd=Y/X_hat.
                //
                // Noise model:
                // - process variance: Good/Watterson channel changes about
                //   5% per OFDM data symbol in the existing simulator model.
                // - pilot interpolation variance: pilot LS noise plus a
                //   frequency-selective interpolation floor for 0.5 ms HF delay
                //   spread over a five-carrier pilot gap.
                // - DD measurement variance: raw FFT-bin noise divided by
                //   |X_hat|^2 and inflated by the posterior/EVM reliability.
                // The DD innovation is additionally clipped to a 30% H step so
                // one wrong coherent decision cannot rotate or scale a carrier
                // without bound.
                constexpr float kGoodDopplerSigmaFrac = 0.05f;
                constexpr float kMaxDdStepFrac = 0.30f;

                const Complex prior_h = channel_estimate[info.fft_idx];
                const float h_ref_mag = std::max(
                    1.0e-3f,
                    std::max(std::abs(prior_h), std::abs(pilot_h)));
                const float process_sigma = kGoodDopplerSigmaFrac * h_ref_mag;
                const float process_var = process_sigma * process_sigma;
                const float pilot_interp_sigma = have_model_error_var
                    ? std::sqrt(std::clamp(wiener_error_var, 0.0f, 1.0f)) * h_ref_mag
                    : 0.20f * h_ref_mag;
                const float pilot_measurement_var =
                    std::max(noise_variance + pilot_interp_sigma * pilot_interp_sigma,
                             MIN_CARRIER_NOISE_VAR);

                float prior_var = pilot_measurement_var;
                const size_t state_idx = static_cast<size_t>(info.fft_idx);
                if (state_idx < dd_qam16_channel_var_.size() &&
                    std::isfinite(dd_qam16_channel_var_[state_idx]) &&
                    dd_qam16_channel_var_[state_idx] > 0.0f) {
                    prior_var = dd_qam16_channel_var_[state_idx] + process_var;
                }

                const float pilot_gain = prior_var / (prior_var + pilot_measurement_var);
                Complex tracked_h = prior_h + pilot_gain * (pilot_h - prior_h);
                float tracked_var = (1.0f - pilot_gain) * prior_var;

                if (have_qam16_dd &&
                    state_idx < dd_qam16_reliability_.size() &&
                    dd_qam16_reliability_[state_idx] > 0.0f) {
                    const float reliability = std::max(dd_qam16_reliability_[state_idx], 0.05f);
                    const float measurement_var =
                        std::max(dd_qam16_measurement_var_[state_idx] / reliability,
                                 MIN_CARRIER_NOISE_VAR);
                    const float kalman_gain =
                        tracked_var / (tracked_var + measurement_var);

                    Complex step = dd_qam16_channel_observations_[state_idx] - tracked_h;
                    const float step_mag = std::abs(step);
                    const float max_step = kMaxDdStepFrac * h_ref_mag;
                    if (step_mag > max_step && step_mag > 1.0e-6f) {
                        step *= max_step / step_mag;
                    }

                    tracked_h += kalman_gain * step;
                    tracked_var *= (1.0f - kalman_gain);
                }

                if (state_idx < dd_qam16_channel_var_.size()) {
                    dd_qam16_channel_var_[state_idx] =
                        std::max(tracked_var, MIN_CARRIER_NOISE_VAR);
                }

                channel_estimate[info.fft_idx] = tracked_h;
            } else {
                channel_estimate[info.fft_idx] = pilot_h;
            }
        }
    }

    applyDiagnosticTwoPathChannelOracle(h_ls_all);

    // 2026-05-29 diag (ULTRA_GENIE_LTS_FREEZE): overwrite the sparse-pilot-interpolated
    // data-symbol estimate with the frozen full-band LTS H. Splits the 16QAM wall:
    // genie -> 16QAM decodes => sparse-pilot interpolation was the limiter (estimation);
    // genie -> still fails    => post-equalization (demap/CFO). On a noiseless frozen
    // channel the stored LTS H is the exact true H, so this is a true genie.
    if (genieLtsFreezeEnabled() && genie_lts_h_.size() == channel_estimate.size()) {
        channel_estimate = genie_lts_h_;
    }

    // 2026-07-29 diag (genie_true_h): overwrite the data-symbol estimate with the
    // NOISELESS H captured from the clean pass over the same fade realization.
    // Unlike GENIE_LTS_FREEZE above, the stored H here carries no estimation
    // noise, so this is a genuine perfect-CSI bound. The data path's own noise is
    // untouched. See src/ofdm/genie_true_h.hpp.
    if (ultra::ofdm::genie_true_h::mode() == ultra::ofdm::genie_true_h::Mode::Inject) {
        if (ultra::ofdm::genie_true_h::haveTrueH(channel_estimate.size())) {
            // Diagnostic: how far is the injected H from the estimate it replaces?
            // Under the null control (identical signal) this must be ~0. A large
            // value means the two are in different representations, not merely
            // different realizations.
            double num = 0.0;
            double den = 0.0;
            for (size_t gi = 0; gi < channel_estimate.size(); ++gi) {
                const auto d = ultra::ofdm::genie_true_h::buffer()[gi] - channel_estimate[gi];
                num += static_cast<double>(std::norm(d));
                den += static_cast<double>(std::norm(channel_estimate[gi]));
            }
            if (den > 0.0) {
                ultra::ofdm::genie_true_h::stats().nmse_sum += num / den;
                ++ultra::ofdm::genie_true_h::stats().nmse_count;
            }
            channel_estimate = ultra::ofdm::genie_true_h::buffer();
            ++ultra::ofdm::genie_true_h::stats().injections;
        } else {
            ++ultra::ofdm::genie_true_h::stats().misses;
        }
    }

    if (have_qam16_dd) {
        std::fill(dd_qam16_reliability_.begin(), dd_qam16_reliability_.end(), 0.0f);
    }

    // Apply DD (decision-directed) phase corrections from previous symbol.
    // These are snapshot corrections computed in equalize() after hard-decision.
    // Applied after interpolation so both pilot-based and DD tracking contribute:
    // - Interpolation: fresh magnitude + phase baseline from 6 pilots
    // - DD corrections: per-carrier phase refinement from 53 data decisions
    if (dd_phase_corrections.size() == data_carrier_indices.size()
        && snr_symbol_count >= 3 &&
        !ofdm_pilots::scatteredPilotsActive(config)) {
        float dd_blend = 0.3f;
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            float corr = dd_phase_corrections[i];
            if (std::abs(corr) > 0.001f) {
                float phase_adj = corr * dd_blend;
                channel_estimate[data_carrier_indices[i]] *=
                    std::exp(Complex(0.0f, phase_adj));
            }
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
    // Note: noise_count == 1 means first symbol fallback (no prev_pilot_phases yet)
    // In that case, noise_power_sum = signal_power / DEFAULT_SNR_LINEAR is already the variance
    //
    // CRITICAL FIX FOR FADING CHANNELS:
    // On fading channels, H[n] - H[n-1] includes BOTH noise AND fading variation.
    // The temporal comparison cannot distinguish them, causing noise_variance to be
    // massively overestimated → compressed LLRs → LDPC decode failure.
    //
    // Solution: Detect fading from pilot magnitude variance and use LTS-based SNR
    // estimate when fading is significant. The pilots still track the channel
    // (for equalization), but we don't let fading contaminate noise_variance.

    // Store for internal OFDM demapper access. The externally reported fading
    // index is accumulated from raw pilot LS magnitudes before interpolation or
    // decision-directed tracking can modify data-carrier channel estimates.
    LOG_DEMOD(DEBUG, "Pilot quality: fading_index=%.3f in_band_snr=%.1f dB "
              "(public=%.3f pilot_residuals=%zu)",
              last_fading_index,
              last_snr_db_estimate_valid ? last_snr_db_estimate : 0.0f,
              public_fading_index,
              pilot_residual_count);

    if (noise_count > 0 && noise_power_sum > 0.0f) {
        // Noise variance strategy: preserve the LTS-based estimate.
        //
        // The LTS estimate (from 2 training symbols, averaged over ~53 carriers) is accurate
        // and reliable. The temporal pilot comparison (curr_h - prev_h) is problematic because:
        // - Coherent first symbol: fallback to 15 dB overwrites accurate LTS (e.g., at SNR=20+)
        // - Coherent subsequent: includes fading variation on fading channels
        //
        // Only update estimated_snr_linear for display/rate-adaptation purposes.
        if (noise_count > 1) {
            float instantaneous_snr = signal_power / std::max(noise_variance, 1e-6f);
            instantaneous_snr = std::max(0.1f, std::min(10000.0f, instantaneous_snr));
            estimated_snr_linear = snr_alpha * instantaneous_snr + (1.0f - snr_alpha) * estimated_snr_linear;
        }
    }

    snr_symbol_count++;
}

// =============================================================================
// CHANNEL INTERPOLATION
// =============================================================================


// =============================================================================
// POST-FEC DATA-AIDED CHANNEL OBSERVATIONS (ULTRA_ITERATIVE_CHEST, default OFF)
// =============================================================================
//
// x_grid is the EXACT transmitted data-carrier constellation of the frame whose
// receive grid is retained here, recovered by re-encoding the frame's verified
// bytes and replaying the production transmit chain (see
// OFDMChirpWaveform::remodulateDataCarrierSymbols). H_dd = Y/X is then the exact
// per-carrier per-symbol channel, noise-limited and model-free — the achievable
// form of the 2026-07-28 genie.
//
// Every safety property lives here or in the caller:
//   * caller gates on LDPC parity for EVERY codeword AND the frame CRC (an
//     LDPC false positive is a LEGAL but wrong codeword, i.e. confident poison);
//   * expect_origin must match the retained grid's origin, so a stale Y can never
//     be divided by a fresh X;
//   * geometry must match symbol-for-symbol and carrier-for-carrier;
//   * |X|^2 > 0 (skips masked carriers and the last symbol's zero pad);
//   * the noise weight is DERIVED (noise_norm/|X|^2), not branched per modulation.
// Any failure leaves the production estimate untouched: degrade to baseline,
// never to garbage.
size_t OFDMDemodulator::Impl::ingestDataAidedGrid(const Symbol& x_grid,
                                                  long long expect_origin) {
    if (!data_aided_enabled_ || da_rx_grid_.empty()) return 0;
    if (da_grid_origin_ < 0 || da_grid_origin_ != expect_origin) return 0;

    // The transmit grid is symbol-major with a CONSTANT number of data carriers per
    // symbol (the scattered pattern rotates position, never count), so the receive
    // grid's own per-symbol width is the only legal stride.
    const size_t carriers = da_rx_grid_[0].size();
    if (carriers == 0) return 0;
    const size_t tx_symbols = x_grid.size() / carriers;
    if (tx_symbols == 0 || x_grid.size() % carriers != 0) return 0;

    // A short transmit grid means the re-encode produced fewer symbols than were
    // demodulated (or vice versa) — a geometry disagreement, not a partial frame.
    const size_t n = std::min(tx_symbols, da_rx_grid_.size());

    size_t pushed = 0;
    for (size_t s = 0; s < n; ++s) {
        const auto& y = da_rx_grid_[s];
        const auto& logical = da_logical_[s];
        if (y.size() != carriers || logical.size() != carriers) break;
        const int64_t sym_index = wiener_symbol_base_ + static_cast<int64_t>(s);
        for (size_t c = 0; c < carriers; ++c) {
            const Complex x = x_grid[s * carriers + c];
            const float x_pow = std::norm(x);
            if (!(x_pow > 1.0e-12f)) continue;  // masked carrier / trailing zero pad
            const Complex h = y[c] / x;
            if (!std::isfinite(h.real()) || !std::isfinite(h.imag())) continue;
            // ADAPTIVITY: H_dd = H + N/X, so the observation noise scales by
            // 1/|X|^2. Every constellation in mapBits() is unit-AVERAGE-power, so
            // this single expression is exactly right for QPSK/8PSK (unit modulus,
            // factor 1) and for the 16/32/64/256-QAM rings (+7.0 dB on the 16QAM
            // inner points, -2.6 dB on its corners) with no per-modulation branch.
            const float noise_norm = da_noise_norm_ / x_pow;
            addWienerPilotObservation(logical[c], sym_index, h, noise_norm);
            ++pushed;
        }
    }
    // One generation only: the grid is consumed so a frame can never be fed back
    // twice, and a later frame's ingest cannot resurrect this one's Y.
    da_rx_grid_.clear();
    da_logical_.clear();
    da_grid_origin_ = -1;
    return pushed;
}

} // namespace ultra
