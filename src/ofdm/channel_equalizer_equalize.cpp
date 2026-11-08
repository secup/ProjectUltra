// OFDM equalization helpers
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "genie_tx_capture.hpp"
#include "measured_llr_noise.hpp"
#include "soft_demap.hpp"
#include "ultra/logging.hpp"

namespace ultra {

using namespace demod_constants;

namespace {

constexpr float kQam16MinPosteriorOdds = 2.1972246f;   // ln(9): >= 9:1 bit odds.
constexpr float kQam16FullPosteriorOdds = 4.5951199f;  // ln(99): near-certain DD.
constexpr float kQam16DecisionChiSq95 = 2.9957323f;    // 95% radius for 2-D Gaussian EVM.

float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

float qam16MinAbsLLRNoClip(Complex sym, float noise_var) {
    const float scale = 2.0f / std::max(noise_var, MIN_CARRIER_NOISE_VAR);
    const float i_abs = std::abs(sym.real());
    const float q_abs = std::abs(sym.imag());
    const float llr_i_sign = std::abs(scale * sym.real());
    const float llr_i_ring = std::abs(scale * (i_abs - QAM16_THRESHOLD));
    const float llr_q_sign = std::abs(scale * sym.imag());
    const float llr_q_ring = std::abs(scale * (q_abs - QAM16_THRESHOLD));
    return std::min(std::min(llr_i_sign, llr_i_ring),
                    std::min(llr_q_sign, llr_q_ring));
}

float psk8MinAbsLLRNoClip(Complex sym, float noise_var) {
    return soft_demap::psk8MinAbsLLRNoClip(sym, noise_var);
}

bool failureAttributionEligible(Modulation mod) {
    return mod == Modulation::QAM8 ||
           mod == Modulation::QAM16 ||
           mod == Modulation::QAM32 ||
           mod == Modulation::QAM64;
}

bool useSoftGrayZoneCsi(Modulation mod) {
    return mod == Modulation::QAM16 ||
           mod == Modulation::QAM32 ||
           mod == Modulation::QAM64;
}

// Relative-depth anti-poison (2026-05-26): the global-noise-var gray-zone (above) never
// fires on a deep frequency-selective null when the AVERAGE SNR is high — a carrier 27 dB
// below the frame mean still has |H|² >> global σ², so it reads "clean" and emits a
// confident-WRONG LLR that poisons the LDPC (measured: QPSK R3/4 fade seeds, |llr|~14 +
// unsat 25-51). Fix: reference the FRAME's mean |H|², not the global noise floor. A carrier
// whose |H|² is a fraction `rel` of the frame mean has a ~1/rel less trustworthy channel
// estimate, so inflate its LLR noise var ~proportionally. On a frequency-FLAT channel
// (AWGN or flat fade) every rel≈1 → no inflation → no AWGN regression; it acts ONLY on
// frequency-selective relative nulls, which is exactly the poison source.
float relativeFadeNoiseInflation(float h_power, float frame_mean_h_power) {
    if (!std::isfinite(h_power) || !std::isfinite(frame_mean_h_power) ||
        frame_mean_h_power <= 0.0f || h_power < 0.0f) {
        return 1.0f;
    }
    // 2026-05-28: env-tunable for sparse-pilot experiment. Default kRelFadeOnset
    // 0.25 (~6 dB below mean) and kMaxRelInflation 30 (cap at 15 dB down-weight)
    // were tuned for the dense 12-pilot baseline. With 6 pilots the Wiener may
    // smooth THROUGH deep nulls so h_power looks normal — pushing the onset
    // higher (e.g. 0.50 = -3 dB) makes the down-weight act earlier on borderline
    // carriers, and a higher cap lets really-bad carriers turn into near-erasures.
    static const float kRelFadeOnset = []() {
        if (const char* env = std::getenv("ULTRA_REL_FADE_ONSET")) {
            const float v = static_cast<float>(std::atof(env));
            if (v > 0.0f && v < 1.0f) return v;
        }
        return 0.25f;
    }();
    static const float kMaxRelInflation = []() {
        if (const char* env = std::getenv("ULTRA_REL_FADE_MAX")) {
            const float v = static_cast<float>(std::atof(env));
            if (v >= 1.0f && v <= 1000.0f) return v;
        }
        return 30.0f;
    }();
    const float rel = h_power / frame_mean_h_power;  // 1.0 = average carrier
    if (rel >= kRelFadeOnset) {
        return 1.0f;
    }
    return std::clamp(kRelFadeOnset / std::max(rel, 1.0e-6f), 1.0f, kMaxRelInflation);
}

float softGrayZoneNoiseInflation(float h_power, float noise_var) {
    if (!std::isfinite(h_power) || !std::isfinite(noise_var) ||
        h_power < 0.0f || noise_var <= MIN_CARRIER_NOISE_VAR) {
        return 1.0f;
    }

    // Smoothly reduce trust in carriers near the deep-fade floor without the
    // old hard-erasure cliff. gamma0=0.5 starts acting below about -3 dB and
    // is nearly identity for clean carriers (gamma >> 1).
    constexpr float kGamma0 = 0.5f;
    constexpr float kMaxInflation = 12.0f;
    const float gamma = h_power / noise_var;
    const float inflation = (gamma + kGamma0) / std::max(gamma, 1.0e-6f);
    return std::clamp(inflation, 1.0f, kMaxInflation);
}

float wrapPi(float phase) {
    while (phase > static_cast<float>(M_PI)) phase -= 2.0f * static_cast<float>(M_PI);
    while (phase < -static_cast<float>(M_PI)) phase += 2.0f * static_cast<float>(M_PI);
    return phase;
}

} // namespace

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

        case Modulation::QAM8: {
            static const float pi = 3.14159265358979f;
            float phase = std::atan2(sym.imag(), sym.real());
            float phase_minus_offset = phase - pi / 8.0f;
            int octant = static_cast<int>(std::round(phase_minus_offset * 4.0f / pi));
            octant = ((octant % 8) + 8) % 8;
            const float angle = octant * (pi / 4.0f) + pi / 8.0f;
            return Complex(std::cos(angle), std::sin(angle));
        }

        case Modulation::QAM16: {
            auto slice = [](float x) -> float {
                if (x < -QAM16_THRESHOLD) return -0.9487f;
                if (x < 0.0f) return -0.3162f;
                if (x < QAM16_THRESHOLD) return 0.3162f;
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

void OFDMDemodulator::Impl::recordFailureAttributionSymbol(
    const std::vector<Complex>& equalized,
    Modulation mod) {
    const bool deep_diag =
        qam16FailureAttributionDiagEnabled() ||
        qam16GenieSigmaEmpiricalEnabled();
    const bool psk_diag =
        deep_diag && (mod == Modulation::BPSK || mod == Modulation::QPSK);
    if (!(failureAttributionEligible(mod) || psk_diag) ||
        equalized.size() != data_carrier_indices.size() ||
        carrier_noise_var.size() != data_carrier_indices.size()) {
        return;
    }

    if (failure_diag_carriers_.size() != data_carrier_indices.size()) {
        failure_diag_carriers_.clear();
        failure_diag_carriers_.reserve(data_carrier_indices.size());
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            FailureAttributionCarrier carrier;
            carrier.logical_carrier = static_cast<int>(i);
            carrier.fft_index = data_carrier_indices[i];
            failure_diag_carriers_.push_back(carrier);
        }
    }

    FailureAttributionSymbol symbol;
    symbol.symbol_index = current_data_symbol_index_;
    symbol.samples = equalized.size();
    symbol.min_abs_h = std::numeric_limits<float>::max();

    double abs_h_sum = 0.0;
    double snr_db_sum = 0.0;
    double evm2_sum = 0.0;
    double phase_unit_re = 0.0;
    double phase_unit_im = 0.0;
    std::vector<float> phase_errors;
    std::vector<float> phase_bins;
    const float ce_margin = soft_demap::getCEErrorMargin(mod);
    const float noise_ref = std::max(noise_variance, MIN_CARRIER_NOISE_VAR);
    if (deep_diag) {
        phase_errors.reserve(equalized.size());
        phase_bins.reserve(equalized.size());
    }

    for (size_t i = 0; i < equalized.size(); ++i) {
        const int idx = data_carrier_indices[i];
        const Complex decision = hardDecision(equalized[i], mod);
        const float evm2 = std::norm(equalized[i] - decision);
        const float evm = std::sqrt(evm2);
        const float effective_noise =
            std::max(carrier_noise_var[i] * ce_margin, MIN_CARRIER_NOISE_VAR);
        const float norm_evm = (evm * evm) / effective_noise;
        const float abs_h = std::abs(channel_estimate[idx]);
        const float gamma = std::norm(channel_estimate[idx]) / noise_ref;
        const float snr_db = 10.0f * std::log10(std::max(gamma, 1.0e-6f));
        const bool inside_noise_model = norm_evm <= kQam16DecisionChiSq95;

        FailureAttributionCarrier& carrier = failure_diag_carriers_[i];
        ++carrier.samples;
        carrier.evm_sum += evm;
        carrier.norm_evm_sum += norm_evm;
        carrier.abs_h_sum += abs_h;
        carrier.snr_db_sum += snr_db;
        if (inside_noise_model) {
            ++carrier.inside_noise;
            ++symbol.inside_noise;
        }

        symbol.evm_sum += evm;
        symbol.norm_evm_sum += norm_evm;
        symbol.min_abs_h = std::min(symbol.min_abs_h, abs_h);
        abs_h_sum += abs_h;
        snr_db_sum += snr_db;
        evm2_sum += evm2;
        failure_diag_evm_.push_back(evm);
        failure_diag_norm_evm_.push_back(norm_evm);

        if (deep_diag && std::norm(decision) > 1.0e-6f) {
            const Complex phase_ratio = equalized[i] * std::conj(decision);
            const float phase_mag = std::abs(phase_ratio);
            if (phase_mag > 1.0e-6f) {
                const float phase = std::arg(phase_ratio);
                phase_unit_re += std::cos(phase);
                phase_unit_im += std::sin(phase);
                const int half_fft = config.fft_size / 2;
                const int signed_bin = (idx <= half_fft) ? idx : idx - config.fft_size;
                phase_bins.push_back(static_cast<float>(signed_bin));
                phase_errors.push_back(phase);
            }
        }

        if (mod == Modulation::QAM16) {
            const bool outer_point =
                std::abs(decision.real()) > 0.6f ||
                std::abs(decision.imag()) > 0.6f;
            if (outer_point) {
                failure_diag_outer_evm2_sum_ += evm2;
                ++failure_diag_outer_evm2_count_;
            } else {
                failure_diag_inner_evm2_sum_ += evm2;
                ++failure_diag_inner_evm2_count_;
            }
        }
    }

    if (symbol.samples > 0) {
        const double inv = 1.0 / static_cast<double>(symbol.samples);
        symbol.mean_abs_h = static_cast<float>(abs_h_sum * inv);
        symbol.mean_snr_db = static_cast<float>(snr_db_sum * inv);
        failure_diag_last_symbol_empirical_var_ =
            static_cast<float>(evm2_sum * inv);
        failure_diag_last_symbol_empirical_valid_ = true;
        failure_diag_empirical_var_sum_ += failure_diag_last_symbol_empirical_var_;
        ++failure_diag_empirical_var_count_;
    } else {
        symbol.min_abs_h = 0.0f;
        failure_diag_last_symbol_empirical_valid_ = false;
    }

    if (deep_diag && phase_errors.size() >= 2) {
        const float cpe = std::atan2(static_cast<float>(phase_unit_im),
                                     static_cast<float>(phase_unit_re));
        double x_sum = 0.0;
        double y_sum = 0.0;
        double xx_sum = 0.0;
        double xy_sum = 0.0;
        float min_bin = std::numeric_limits<float>::max();
        float max_bin = -std::numeric_limits<float>::max();
        for (size_t i = 0; i < phase_errors.size(); ++i) {
            const float x = phase_bins[i];
            const float y = wrapPi(phase_errors[i] - cpe);
            x_sum += x;
            y_sum += y;
            xx_sum += static_cast<double>(x) * x;
            xy_sum += static_cast<double>(x) * y;
            min_bin = std::min(min_bin, x);
            max_bin = std::max(max_bin, x);
        }
        const double n = static_cast<double>(phase_errors.size());
        const double denom = n * xx_sum - x_sum * x_sum;
        float slope = 0.0f;
        if (std::abs(denom) > 1.0e-9) {
            slope = static_cast<float>((n * xy_sum - x_sum * y_sum) / denom);
        }
        symbol.cpe_rad = cpe;
        symbol.phase_slope_rad_per_bin = slope;
        symbol.phase_ramp_edge_rad = slope * (max_bin - min_bin);
    }
    failure_diag_symbols_.push_back(symbol);
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

const std::vector<Complex>& OFDMDemodulator::Impl::equalize(const std::vector<Complex>& freq_domain, Modulation mod) {
    auto& equalized = equalized_scratch;
    equalized.resize(data_carrier_indices.size());
    carrier_noise_var.resize(data_carrier_indices.size());
    carrier_erasure_flags_.assign(data_carrier_indices.size(), 0);

    // 2026-07-28 DIAGNOSTIC (ULTRA_EQ_TRACE=1): equalize() call-tree tracer.
    {
        auto& tr = ultra::genie::eqTrace();
        if (tr.enabled) {
            const auto& pc = ultra::genie::passContext();
            const std::size_t abs_sym =
                pc.abs_train +
                static_cast<std::size_t>(pc.training_symbols + static_cast<int>(current_data_symbol_index_)) *
                    pc.symbol_samples;
            std::fprintf(stderr,
                         "[eqtrace] eq#%ld pass=%ld site=%-14s sym=%zu synced=%d mod=%d "
                         "nData=%zu abs=%zu presynced=%d softbits=%zu\n",
                         ++tr.calls, tr.pass, tr.site, current_data_symbol_index_,
                         synced_symbol_count.load(), static_cast<int>(mod),
                         data_carrier_indices.size(), abs_sym, pc.presynced ? 1 : 0,
                         soft_bits.size());
        }
    }

    // 2026-05-29 diag (ULTRA_GENIE_DATA_AIDED): true per-symbol channel genie.
    // Overwrite channel_estimate with the EXACT effective channel H[k] = Y[k]/X[k]
    // using the actual transmitted constellation captured from the modulator (see
    // genie_tx_capture.hpp). No model, no interpolation, no temporal staleness — the
    // only unconfounded genie. Splits the 16QAM wall: genie -> 16QAM decodes =>
    // estimation is the limiter; genie -> still fails => post-equalization (demap).
    //
    // 2026-07-28 ALIGNMENT FIX. The capture is addressed by ABSOLUTE SAMPLE POSITION,
    // never by a FIFO cursor: this decoder equalizes the same physical symbol 2-6
    // times per frame (control-first peek at the CONTROL carrier geometry, CW0 header
    // peek, authoritative pass, smallframe/sync-recovery/cw-discovery retries), so a
    // consuming cursor over-ran the capture by +14 entries per frame and every genie
    // number taken through it was meaningless. Three guards make a mispair impossible
    // rather than merely unlikely:
    //   1. presynced-only — the streaming SYNCED path has no absolute anchor.
    //   2. position match within half a symbol — a re-slice retry (+/-8 samples) still
    //      resolves to the right symbol; a symbol the encoder never sent resolves to
    //      nothing.
    //   3. carrier-geometry match — this is what declines the control-first peek,
    //      which runs the DATA frame's audio through the CONTROL profile (47 data /
    //      12 pilots vs 51/8) and would otherwise divide by the wrong carrier set.
    // A miss leaves the production estimate untouched (degrade to baseline, never to
    // garbage) and is COUNTED — ULTRA_GENIE_DEBUG prints hits/misses/geom_rejects per
    // chunk, because a silent mispair is what made the old numbers unfalsifiable.
    {
        auto& cap = ultra::genie::txCapture();
        const auto& pc = ultra::genie::passContext();
        if (cap.enabled && pc.presynced && pc.symbol_samples > 0) {
            // Frame anchor = this pass's absolute position for the frame's FIRST data
            // symbol; the symbol within the frame is current_data_symbol_index_, which
            // is frame-relative on the presynced path and identical to the encoder's
            // per-modulate() symbol_index. Anchoring at FRAME granularity is what fixes
            // the residual misalignment: the receiver's sync convention puts the RX
            // position up to ~0.7 symbol away from the transmitted one (measured +796
            // samples on ITU Good), which a nearest-SYMBOL match aliases by exactly one
            // symbol — small residual, right geometry, wrong data.
            const long long anchor =
                static_cast<long long>(pc.abs_train) +
                static_cast<long long>(pc.training_symbols) *
                    static_cast<long long>(pc.symbol_samples);
            const ultra::genie::TxSymbol* tx =
                cap.lookup(anchor, static_cast<int>(current_data_symbol_index_),
                           static_cast<long long>(pc.symbol_samples),
                           data_carrier_indices.size(), pilot_carrier_indices.size());
            // INDEPENDENT ALIGNMENT VALIDATOR (ULTRA_GENIE_VERIFY=1). The position key
            // is a claim; this checks it against the CONTENT. Quadrant agreement
            // between the PRODUCTION-equalized symbol (Y/H_prod, which decodes QPSK at
            // 75% on this channel) and the looked-up X: ~1.0 iff the entry is the
            // symbol actually transmitted here, ~0.25 for any wrong entry. Never
            // used to select an entry — only to falsify the key.
            static const bool genie_verify = [] {
                const char* e = std::getenv("ULTRA_GENIE_VERIFY");
                return e && e[0] == '1';
            }();
            if (genie_verify && tx != nullptr && tx->x.size() == channel_estimate.size()) {
                auto score = [&](const std::vector<Complex>& xs) {
                    int agree = 0, n = 0;
                    for (int idx : data_carrier_indices) {
                        if (std::norm(xs[idx]) < 1.0e-12f) continue;
                        if (std::norm(channel_estimate[idx]) < 1.0e-20f) continue;
                        const Complex eq = freq_domain[idx] / channel_estimate[idx];
                        agree += (((eq.real() >= 0) == (xs[idx].real() >= 0)) &&
                                  ((eq.imag() >= 0) == (xs[idx].imag() >= 0))) ? 1 : 0;
                        ++n;
                    }
                    return n > 0 ? static_cast<double>(agree) / n : -1.0;
                };
                const double got = score(tx->x);
                double best = -1.0;
                long long best_off = 0;
                const long long matched_off =
                    tx->frame_base + static_cast<long long>(tx->sym_in_frame) *
                                         static_cast<long long>(pc.symbol_samples);
                if (got < 0.80) {
                    for (const auto& cand : cap.symbols) {
                        if (cand.x.size() != channel_estimate.size()) continue;
                        const double s = score(cand.x);
                        if (s > best) {
                            best = s;
                            best_off = cand.frame_base +
                                       static_cast<long long>(cand.sym_in_frame) *
                                           static_cast<long long>(pc.symbol_samples);
                        }
                    }
                }
                std::fprintf(stderr,
                             "[genie-verify] dsi=%zu nData=%zu anchor=%lld matched=%lld "
                             "agree=%.3f | best=%.3f best_off=%lld delta_sym=%.2f\n",
                             current_data_symbol_index_, data_carrier_indices.size(),
                             anchor, matched_off, got, best, best_off,
                             static_cast<double>(best_off - matched_off) /
                                 static_cast<double>(pc.symbol_samples));
            }
            if (tx != nullptr && tx->x.size() == channel_estimate.size()) {
                for (int idx : data_carrier_indices) {
                    if (std::norm(tx->x[idx]) > 1.0e-12f)
                        channel_estimate[idx] = freq_domain[idx] / tx->x[idx];
                }
                for (int idx : pilot_carrier_indices) {
                    if (std::norm(tx->x[idx]) > 1.0e-12f)
                        channel_estimate[idx] = freq_domain[idx] / tx->x[idx];
                }
            }
        }
    }

    // Coherent-only OFDM (thread A 2026-05-31): the differential MMSE-equalize
    // early-return (magnitude-tracked |H|, frozen LTS phase) was removed here —
    // OFDM never carries a differential modulation now. Coherent equalization
    // (pilot/LMS/RLS MMSE + soft-CSI + coherent DD) follows.
    bool use_adaptive = config.adaptive_eq_enabled;
    const float reliability_noise_var = std::max(noise_variance, MIN_CARRIER_NOISE_VAR);
    const bool soft_gray_zone_csi = useSoftGrayZoneCsi(mod);

    // 2026-06-12 Phase 2b: per-carrier channel-estimate-error LLR term (eps_H). The MMSE
    // carrier_noise_var below models THERMAL noise only (sigma^2/(|H|^2+sigma^2)); on a
    // fading channel the true post-eq residual is dominated by per-carrier H-ESTIMATE
    // error, which the demapper otherwise asserts at full confidence -> confident-wrong
    // LLRs that poison the LDPC (fatal for tight 16QAM, absorbed by QPSK's margins).
    // ULTRA_HERR_LLR_K (default 1.0 = ON since 2026-06-17; set =0 to DISABLE) scales the
    // pilot-anchored Wiener error_var into the noise NUMERATOR: nv =
    // (sigma^2 + k*err_var*|H|^2)/(|H|^2+sigma^2). QPSK/QAM8 ONLY (gated !soft_gray_zone_csi).
    // Production form of the (net-negative) single-symbol ULTRA_LLR_NOISE_EMP_FLOOR —
    // pilot-anchored, not single-symbol. k=1.0 is the VALIDATED value: a k-tune (0.5/1.0/2.0)
    // on 16QAM R2/3 sp8 Good@20 peaks at 1.0; k=2.0 over-inflates (suppresses good carriers ->
    // LDPC starves, CW-fails spike), k=0.5 under-weights. The Wiener error_var IS a calibrated
    // variance, so trust it 1:1. data_phase2b_epsH_ktune_2026-06-12.tsv.
    static const float kHerrLlrK = []() {
        if (const char* env = std::getenv("ULTRA_HERR_LLR_K")) {
            const float v = static_cast<float>(std::atof(env));
            if (v >= 0.0f) return v;  // env wins, including 0 to DISABLE
        }
        // DEFAULT ON (2026-06-17): feed the calibrated per-carrier Wiener estimate-error
        // variance into the LLR noise. Controlled OTASim A/B (same seed, moderate@20, only
        // this knob): Mode-B confident-wrong CW-fails 147->21 (-86%), failed groups -75%,
        // 50KB delivered in 2.4x fewer group-attempts. Principled (uses a variance the RX
        // already computes; ~0 on flat/AWGN so no AWGN regression by construction). k=1.0 is
        // the in-code-validated value (data_phase2b_epsH_ktune). See project_retx_modeB memory.
        return 1.0f;
    }();

    // Relative-depth anti-poison: reference the frame's mean |H|² so deep
    // frequency-selective nulls (which the global-noise-var gates miss at high avg SNR)
    // get their LLR confidence cut. Scoped to QPSK/QAM8 — the rungs that had NO gray-zone
    // weighting. QAM16+ already have softGrayZoneNoiseInflation; stacking the relative one
    // on top double-counted and regressed AWGN@30 QAM16 (estimation noise on a flat channel
    // tripped the relative inflation and wrongly down-weighted good carriers). Leave QAM16+
    // on their existing path; unify into one reliability model in the deferred refactor.
    const bool apply_relative_csi =
        (mod == Modulation::QPSK || mod == Modulation::QAM8);
    float frame_mean_h_power = 0.0f;
    if (apply_relative_csi && !data_carrier_indices.empty()) {
        for (int idx : data_carrier_indices) {
            frame_mean_h_power += std::norm(channel_estimate[idx]);
        }
        frame_mean_h_power /= static_cast<float>(data_carrier_indices.size());
    }

    // ULTRA_NULL_DIAG (Phase 2b fading-null forensics, default off): per-relative-depth-bin
    // reliability summary on the RX. Reveals whether carriers in relative nulls get an
    // appropriately HIGH noise-var (-> erasure LLRs) or stay overconfident, and whether the
    // eps_H error_var actually flags them. Bins by |H|^2 / frame-mean-|H|^2.
    static const bool null_diag = []() {
        const char* e = std::getenv("ULTRA_NULL_DIAG");
        return e && e[0] && !(e[0] == '0' && e[1] == '\0');
    }();
    float diag_mean_h_power = frame_mean_h_power;
    if (null_diag && diag_mean_h_power <= 0.0f && !data_carrier_indices.empty()) {
        for (int didx : data_carrier_indices) diag_mean_h_power += std::norm(channel_estimate[didx]);
        diag_mean_h_power /= static_cast<float>(data_carrier_indices.size());
    }
    double nd_cnt[5] = {0}, nd_err[5] = {0}, nd_therm[5] = {0}, nd_eps[5] = {0}, nd_tot[5] = {0};

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
                // MMSE post-equalization noise variance: σ²/(|H|²+σ²)
                carrier_noise_var[i] = noise_variance / mmse_denom;
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
                // MMSE post-equalization noise variance: σ²/(|H|²+σ²), plus the optional
                // per-carrier eps_H estimate-error term (ULTRA_HERR_LLR_K; k=0 -> unchanged).
                float h_err_var = 0.0f;
                // QPSK/QAM8 ONLY: skip when soft_gray_zone_csi (QAM16+) is active — QAM16
                // already inflates per-carrier noise via softGrayZoneNoiseInflation, and
                // STACKING eps_H on top double-counts → over-inflates → LDPC starves.
                // Controlled OTASim gate (QAM16 good@24, same seed): eps_H ON regressed
                // goodput 2720->2020 bps and CW-fails 135->512. So eps_H is QPSK/QAM8-only.
                if (kHerrLlrK > 0.0f && !soft_gray_zone_csi &&
                    static_cast<size_t>(idx) < per_carrier_h_error_var_.size()) {
                    h_err_var = kHerrLlrK * per_carrier_h_error_var_[idx] * h_power;
                }
                // ── STAGE B: notch reliability floor (ULTRA_NOTCH_NV, default
                // OFF) ── D1: the Wiener smooths a parked notch SHALLOW, |H_est|²
                // over-reads, nv collapses, and the carrier emits confident-wrong
                // LLRs (F142: |LLR| 8-14 where erasures belong). The raw direct
                // LS pilot observation is the one signal that sees the notch
                // (E[O_k] = P_true + σ²): the reliability power may never exceed
                // what was directly OBSERVED plus its own noise allowance,
                //   P_rel = min(|H_est|², O_k + σ²).
                // The equalizer TAP stays on the smoothed estimate (phase
                // quality); only the noise-variance denominator uses P_rel. On
                // AWGN/flat channels O_k ≈ P_est → min() is a no-op — no
                // clean-channel regression by construction. Age bound = 3
                // scattered-pattern revisit cycles (≪ the seconds a notch parks).
                static const bool kNotchNv = []() {
                    const char* e = std::getenv("ULTRA_NOTCH_NV");
                    return e && e[0] != '\0' && !(e[0] == '0' && e[1] == '\0');
                }();
                float nv_denom = mmse_denom;
                if (kNotchNv &&
                    static_cast<size_t>(idx) < per_carrier_raw_obs_power_.size() &&
                    per_carrier_raw_obs_symbol_[idx] >= 0) {
                    const int64_t obs_age =
                        static_cast<int64_t>(current_data_symbol_index_) -
                        per_carrier_raw_obs_symbol_[idx];
                    const int64_t max_age =
                        3 * static_cast<int64_t>(std::max(1u, config.pilot_spacing));
                    if (obs_age >= 0 && obs_age <= max_age) {
                        const float p_rel = std::min(
                            h_power,
                            per_carrier_raw_obs_power_[idx] + noise_variance);
                        nv_denom = p_rel + noise_variance;
                    }
                }
                carrier_noise_var[i] = (noise_variance + h_err_var) / nv_denom;
                carrier_noise_var[i] = std::max(MIN_CARRIER_NOISE_VAR, std::min(MAX_CARRIER_NOISE_VAR, carrier_noise_var[i]));
            }
        }

        float h_power = std::norm(channel_estimate[idx]);
        if (soft_gray_zone_csi) {
            carrier_noise_var[i] *= softGrayZoneNoiseInflation(h_power, reliability_noise_var);
            carrier_noise_var[i] =
                std::max(MIN_CARRIER_NOISE_VAR,
                         std::min(MAX_CARRIER_NOISE_VAR, carrier_noise_var[i]));
        }
        if (apply_relative_csi && frame_mean_h_power > 0.0f) {
            carrier_noise_var[i] *= relativeFadeNoiseInflation(h_power, frame_mean_h_power);
            carrier_noise_var[i] =
                std::max(MIN_CARRIER_NOISE_VAR,
                         std::min(MAX_CARRIER_NOISE_VAR, carrier_noise_var[i]));
        }
        if (rx_carrier_erasure_enabled_ &&
            h_power < RX_ERASURE_GAMMA_FLOOR_LINEAR * reliability_noise_var) {
            carrier_erasure_flags_[i] = 1;
            carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
        }

        if (null_diag && diag_mean_h_power > 0.0f) {
            const float depth = h_power / diag_mean_h_power;  // |H|^2 relative to frame mean
            const int b = depth < 0.1f ? 0 : depth < 0.3f ? 1 : depth < 0.7f ? 2
                          : depth < 1.5f ? 3 : 4;
            const float denom = h_power + noise_variance;
            const float ev = (static_cast<size_t>(idx) < per_carrier_h_error_var_.size())
                                 ? per_carrier_h_error_var_[idx] : 0.0f;
            nd_cnt[b] += 1.0;
            nd_err[b] += ev;                                  // eps_H normalized error_var
            nd_therm[b] += noise_variance / denom;            // thermal nv part
            nd_eps[b] += kHerrLlrK * ev * h_power / denom;    // eps_H nv contribution
            nd_tot[b] += carrier_noise_var[i];                // final nv (post all inflations)
        }
    }

    if (null_diag && diag_mean_h_power > 0.0f) {
        static int nd_symcount = 0;
        if ((++nd_symcount % 20) == 1) {  // throttle ~every 20th symbol
            std::ostringstream oss;
            const char* nm[5] = {"deep", "null", "fade", "norm", "strong"};
            oss << "mod=" << static_cast<int>(mod) << " mean_hp=" << diag_mean_h_power;
            for (int b = 0; b < 5; ++b) {
                oss << " " << nm[b] << "[n=" << static_cast<int>(nd_cnt[b]);
                if (nd_cnt[b] > 0) {
                    oss << " ev=" << (nd_err[b] / nd_cnt[b])
                        << " therm=" << (nd_therm[b] / nd_cnt[b])
                        << " eps=" << (nd_eps[b] / nd_cnt[b])
                        << " tot=" << (nd_tot[b] / nd_cnt[b]);
                }
                oss << "]";
            }
            LOG_DEMOD(WARN, "[NULLDIAG] %s", oss.str().c_str());
        }
    }

    // 2026-05-29 DIAG (ULTRA_LLR_NOISE_EMP_FLOOR): empirical post-eq noise-variance
    // FLOOR. The analytic MMSE carrier_noise_var above is a |H|²/thermal model; the
    // FAILURE_ATTRIBUTION eq_diag showed it under-estimates the true post-eq residual
    // by ~4-14x on Good@20 (un-modeled phase ramp / residual CFO / ISI / estimate
    // error), so demapQAM16 (scale=2/noise_var) emits over-confident, confident-WRONG
    // LLRs that poison the LDPC — fatal for tight 16QAM, absorbed by QPSK's margins.
    // Floor noise_var at k*|equalized - hardDecision|^2: on a clean carrier this ≈
    // the true residual (calibrates the LLR); on a deep null the equalized point lands
    // far from EVERY constellation point so the residual is huge → auto-erasure. Only
    // RAISES noise_var (never lowers), so it cannot make a well-calibrated carrier
    // over-confident. Default off (k=0). If this rescues 16QAM on Good@20 the
    // over-confident-LLR diagnosis is proven; the production form is a smoothed
    // per-carrier empirical estimate (pilot-anchored), not single-symbol hard-decision.
    static const float kLLREmpFloor = []() {
        if (const char* env = std::getenv("ULTRA_LLR_NOISE_EMP_FLOOR")) {
            const float v = static_cast<float>(std::atof(env));
            if (v > 0.0f) return v;
        }
        return 0.0f;
    }();
    if (kLLREmpFloor > 0.0f && equalized.size() == carrier_noise_var.size()) {
        for (size_t i = 0; i < equalized.size(); ++i) {
            const Complex dec = hardDecision(equalized[i], mod);
            const float emp = std::norm(equalized[i] - dec);  // |eq - decision|^2
            carrier_noise_var[i] = std::min(
                MAX_CARRIER_NOISE_VAR,
                std::max(carrier_noise_var[i], kLLREmpFloor * emp));
        }
    }

    recordFailureAttributionSymbol(equalized, mod);

    // ── Radio-agnostic decision-directed EVM SNR accumulation (Stage 1) ──
    // Sum the gain-corrected EVM statistics (S_dd, S_ee, S_de) of the equalized data
    // constellation against its own hard decisions, over the NON-ERASED data carriers.
    // Always on (unlike the diagnostics above); finalized per burst in
    // resetFailureAttributionDiagnostics. Constant-free: signal power is measured
    // (Sum|decision|^2) and the MMSE scalar gain is fit out, so the ratio is the true
    // usable SNR on ANY radio/noise shape — no reference level, no offset.
    if (equalized.size() == data_carrier_indices.size()) {
        for (size_t i = 0; i < equalized.size(); ++i) {
            if (i < carrier_erasure_flags_.size() && carrier_erasure_flags_[i] != 0) {
                continue;  // deep-null carrier: not usable, excluded from the SNR
            }
            const Complex decision = hardDecision(equalized[i], mod);
            const float dp = std::norm(decision);
            if (dp <= 1.0e-9f) {
                continue;  // no valid decision (origin / dead carrier)
            }
            // Gain-corrected EVM sums: S_dd, S_ee, S_de. The finalizer removes the MMSE
            // scalar gain-shrinkage (g = S_de/S_ee) so only genuine scatter counts as noise.
            evm_dd_accum_ += dp;                              // |decision|^2
            evm_ee_accum_ += static_cast<double>(std::norm(equalized[i]));  // |eq|^2
            evm_de_accum_ += static_cast<double>(decision.real() * equalized[i].real() +
                                                 decision.imag() * equalized[i].imag());  // Re(dec*conj(eq))
            ++evm_carrier_count_;
        }
    }

    // Coherent 8PSK/16-QAM decision-directed channel observations. The noise
    // reference is the post-equalizer carrier noise variance used by the LLRs,
    // inflated by the same CE margin as demapping. A carrier is accepted only when:
    // - its EVM is inside the 95% 2-D Gaussian noise radius,
    // - it remains inside half the constellation decision-cell spacing, and
    // - every bit has at least 9:1 posterior odds.
    //
    // The actual H update is consumed in updateChannelEstimate() after the next
    // symbol's pilots have re-anchored the channel, which matches the receiver
    // loop order: pilot update -> equalize -> demap/decision.
    const bool use_coherent_dd =
        (mod == Modulation::QAM8 || mod == Modulation::QAM16);
    if (use_coherent_dd && data_carrier_indices.size() == equalized.size()) {
        dd_qam16_channel_observations_.assign(config.fft_size, Complex(0, 0));
        dd_qam16_measurement_var_.assign(config.fft_size, 0.0f);
        dd_qam16_reliability_.assign(config.fft_size, 0.0f);

        float norm_evm_sum = 0.0f;
        size_t norm_evm_count = 0;
        size_t reliable_count = 0;
        const float decision_cell_guard2 =
            (mod == Modulation::QAM8)
                ? std::pow(2.0f * std::sin(static_cast<float>(M_PI) / 16.0f), 2.0f)
                : 0.25f * QAM16_THRESHOLD * QAM16_THRESHOLD;

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            if (i < carrier_erasure_flags_.size() && carrier_erasure_flags_[i] != 0) {
                continue;
            }

            const int idx = data_carrier_indices[i];
            const Complex decision = hardDecision(equalized[i], mod);
            const float decision_power = std::norm(decision);
            if (decision_power <= 1.0e-6f) {
                continue;
            }

            const float ce_margin =
                (mod == Modulation::QAM8) ? CE_MARGIN_8PSK : CE_MARGIN_QAM16;
            const float effective_noise =
                std::max(carrier_noise_var[i] * ce_margin, MIN_CARRIER_NOISE_VAR);
            const float evm2 = std::norm(equalized[i] - decision);
            const float norm_evm = evm2 / effective_noise;
            norm_evm_sum += norm_evm;
            ++norm_evm_count;

            const float min_abs_llr = (mod == Modulation::QAM8)
                ? psk8MinAbsLLRNoClip(equalized[i], effective_noise)
                : qam16MinAbsLLRNoClip(equalized[i], effective_noise);
            const bool inside_noise_model = norm_evm <= kQam16DecisionChiSq95;
            const bool inside_decision_cell = evm2 <= decision_cell_guard2;
            const bool enough_posterior_margin = min_abs_llr >= kQam16MinPosteriorOdds;
            if (!inside_noise_model || !inside_decision_cell || !enough_posterior_margin) {
                continue;
            }

            const float evm_reliability = clamp01(1.0f - norm_evm / kQam16DecisionChiSq95);
            const float llr_reliability = clamp01(
                (min_abs_llr - kQam16MinPosteriorOdds) /
                (kQam16FullPosteriorOdds - kQam16MinPosteriorOdds));
            const float reliability = std::min(evm_reliability, llr_reliability);
            if (reliability <= 0.0f) {
                continue;
            }

            const size_t state_idx = static_cast<size_t>(idx);
            dd_qam16_channel_observations_[state_idx] = freq_domain[idx] / decision;
            dd_qam16_measurement_var_[state_idx] =
                std::max(noise_variance / decision_power, MIN_CARRIER_NOISE_VAR);
            dd_qam16_reliability_[state_idx] = reliability;
            ++reliable_count;
        }

        const float mean_norm_evm = (norm_evm_count > 0)
            ? norm_evm_sum / static_cast<float>(norm_evm_count)
            : kQam16DecisionChiSq95 + 1.0f;
        const size_t min_reliable_carriers = std::max(
            pilot_carrier_indices.size(),
            static_cast<size_t>(1));

        // If the whole symbol is outside the documented noise model and the
        // number of reliable decisions is below the pilot anchor density, freeze
        // DD for this symbol and let the next update use pilot interpolation.
        if (mean_norm_evm > kQam16DecisionChiSq95 &&
            reliable_count < min_reliable_carriers) {
            std::fill(dd_qam16_reliability_.begin(), dd_qam16_reliability_.end(), 0.0f);
        }
    } else if (!use_coherent_dd) {
        dd_qam16_channel_observations_.clear();
        dd_qam16_measurement_var_.clear();
        dd_qam16_reliability_.clear();
        dd_qam16_channel_var_.clear();
    }

    // Decision-directed per-carrier phase tracking for coherent modes.
    // Store the current symbol's phase correction for next symbol's updateChannelEstimate()
    // to apply AFTER interpolation. Each symbol gets a fresh estimate (no accumulation,
    // which would diverge due to positive feedback from the correction-measure loop).
    if ((mod == Modulation::QPSK || mod == Modulation::BPSK) && snr_symbol_count >= 2) {
        dd_phase_corrections.resize(data_carrier_indices.size(), 0.0f);

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            float eq_mag = std::abs(equalized[i]);
            if (eq_mag < 0.3f) {
                dd_phase_corrections[i] = 0.0f;
                continue;
            }

            Complex decision = hardDecision(equalized[i], mod);
            Complex error_ratio = equalized[i] * std::conj(decision);
            float phase_err = std::arg(error_ratio);

            // Only store correction if hard decision is likely correct
            if (std::abs(phase_err) < 0.61f) {
                dd_phase_corrections[i] = -phase_err;
            } else {
                dd_phase_corrections[i] = 0.0f;
            }
        }
    }

    // ── STAGE A: ZF-consistent LLR unbias (ULTRA_ZF_LLR_UNBIAS, default OFF) ──
    // F142/F165 notch forensics (docs: docs/LLR_NOTCH_CALIBRATION_2026_07_06.md, D3): the MMSE
    // output eq = conj(H)·Y/(|H|²+σ²) is SHRUNK by β = |H|²/(|H|²+σ²), but the
    // demapper compares |eq| against UNBIASED amplitude thresholds (16QAM ring
    // 2/√10) — on a low-γ carrier outer points systematically read as inner:
    // deterministic wrong-sign ring bits with confident magnitude. Dividing the
    // (eq, nv) pair by β restores the unbiased statistic:
    //   eq' = conj(H)·Y/|H|²   nv' = σ²_eff/|H|²
    // Sign-bit LLRs (QPSK/BPSK, and every sign bit of QAM) are ∝ eq/nv —
    // bit-identical under the common scale — while amplitude-bit scale 2/nv'
    // → 0 as the true notch deepens: near-erasure with no gate, no cap, no
    // modulation branch. Runs LAST so every in-function DD/EVM/chi-sq consumer
    // above sees today's MMSE statistics unchanged.
    // PAIRING GUARD (mandatory): nv' beyond MAX_CARRIER_NOISE_VAR cannot ride
    // the clamp (eq' would keep growing while nv' saturates — manufacturing the
    // exact confident-wrong failure this fixes): that deep (γ < −20 dB) is a
    // declared erasure.
    static const bool kZfLlrUnbias = []() {
        const char* e = std::getenv("ULTRA_ZF_LLR_UNBIAS");
        return e && e[0] != '\0' && !(e[0] == '0' && e[1] == '\0');
    }();
    if (kZfLlrUnbias) {
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            const int idx = data_carrier_indices[i];
            const Complex h_used =
                use_adaptive ? lms_weights[idx] : channel_estimate[idx];
            const float p = std::norm(h_used);
            const float denom = p + noise_variance;
            if (p <= 1e-10f || denom <= 1e-10f) {
                if (i < carrier_erasure_flags_.size()) carrier_erasure_flags_[i] = 1;
                equalized[i] = Complex(0, 0);
                carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
                continue;
            }
            const float inv_beta = denom / p;  // ≥ 1 by construction
            const float nv_unbiased = carrier_noise_var[i] * inv_beta;
            if (nv_unbiased >= MAX_CARRIER_NOISE_VAR) {
                if (i < carrier_erasure_flags_.size()) carrier_erasure_flags_[i] = 1;
                equalized[i] = Complex(0, 0);
                carrier_noise_var[i] = MAX_CARRIER_NOISE_VAR;
            } else {
                equalized[i] *= inv_beta;
                carrier_noise_var[i] = std::max(MIN_CARRIER_NOISE_VAR, nv_unbiased);
            }
        }
    }

    // ── MEASURED TOTAL-ERROR LLR NOISE (ULTRA_LLR_MEASURED_NV, default ON) ──
    // Technique adopted from an open-source HF modem's receiver (LLR noise from
    // the frame's own decision residual), generalized to a Gaussian scale
    // mixture — derivation in measured_llr_noise.hpp.
    //
    // Why. Every model above sets the LLR sigma^2 from THERMAL noise (+ modelled
    // estimate error). The true post-equalization error also holds what no model
    // here captures exactly: channel-estimate interpolation/staleness error,
    // residual CFO/timing phase, TX/RX EVM. When the model under-reads the
    // truth, LLRs exceed MAX_LLR and clip to +/-20. The LDPC is NORMALIZED
    // MIN-SUM — invariant to a uniform LLR scale — so the scale matters ONLY
    // through that clip, and a fully clipped frame decodes as hard decisions.
    // Measured 2026-11-08 (measure_ack_fer, ITU Good, 96 frames/cell): 8PSK R3/4
    // FER 29% @24 dB RISING to 40% @40-60 dB; FER climbing with SNR is that
    // saturation's signature.
    //
    // What.
    //  1. Received-domain residual r_c = Y - H*xhat (xhat = hard decision of the
    //     unbiased point Y/H). E|r|^2 = sigma_n^2 + E|x|^2 E|H-H_est|^2 + ...,
    //     ~white across carriers, so one running per-pass mean s2 maps to every
    //     carrier by the SAME MMSE form the model uses, with thermal sigma^2
    //     replaced by the total: floor_c = s2 / (|H_c|^2 + sigma_n^2).
    //  2. Expected variance v_c = max(model_c, floor_c): both are LOWER bounds
    //     (thermal-only; decision errors shorten the measured distance), so the
    //     larger is the tightest available bound on the truth.
    //  3. Tail weight alpha from the standardized residual u_c = e_c / v_c
    //     (exactly R = 2 for a correct Gaussian model -> alpha = inf -> no change).
    //  4. Cell variance = first-order Student-t blend of v_c and the cell's own
    //     residual; never below the model (it cannot make a carrier more
    //     confident than the existing model allows).
    // Modulation-agnostic by construction: hardDecision of the in-scope
    // constellation, no per-mod constant, no tuned constant. Runs LAST so the DD
    // tracker gates, EVM meter and ZF stage above all see today's statistics.
    static const bool kMeasuredNv = [] {
        if (const char* e = std::getenv("ULTRA_LLR_MEASURED_NV")) {
            return !(e[0] == '0' && e[1] == '\0');
        }
        return true;
    }();
    if (kMeasuredNv && equalized.size() == data_carrier_indices.size() &&
        carrier_noise_var.size() == data_carrier_indices.size()) {
        const size_t nd = data_carrier_indices.size();
        measured_cell_r2_.assign(nd, -1.0f);
        double sym_sum = 0.0;
        size_t sym_cells = 0;
        for (size_t i = 0; i < nd; ++i) {
            const int idx = data_carrier_indices[i];
            const Complex h = use_adaptive ? lms_weights[idx] : channel_estimate[idx];
            const float p = std::norm(h);
            if (!(p > 1.0e-12f) || !std::isfinite(p)) {
                continue;
            }
            const Complex y = freq_domain[idx];
            const Complex xhat = hardDecision(y / h, mod);
            if (!(std::norm(xhat) > 1.0e-9f)) {
                continue;
            }
            const float r2 = std::norm(y - h * xhat);
            if (!std::isfinite(r2)) {
                continue;
            }
            measured_cell_r2_[i] = r2;
            sym_sum += r2;
            ++sym_cells;
        }
        measured_residual_sum_ += sym_sum;
        measured_residual_cells_ += static_cast<double>(sym_cells);
        if (measured_residual_cells_ > 0.0) {
            const double s2 = measured_residual_sum_ / measured_residual_cells_;
            // Same domain as the (eq, nv) pair the demapper reads: ZF-unbiased
            // when that stage ran, MMSE otherwise.
            auto denomOf = [&](size_t i) {
                const int idx = data_carrier_indices[i];
                const Complex h = use_adaptive ? lms_weights[idx] : channel_estimate[idx];
                const float p = std::norm(h);
                return static_cast<double>(kZfLlrUnbias ? p : (p + noise_variance));
            };
            // Pass A: expected variance per cell, and the standardized moments.
            for (size_t i = 0; i < nd; ++i) {
                if (measured_cell_r2_[i] < 0.0f ||
                    (i < carrier_erasure_flags_.size() && carrier_erasure_flags_[i] != 0)) {
                    continue;
                }
                const double denom = denomOf(i);
                if (!(denom > 1.0e-12) || !std::isfinite(denom)) {
                    continue;
                }
                const double v = std::max(static_cast<double>(carrier_noise_var[i]), s2 / denom);
                const double u = (static_cast<double>(measured_cell_r2_[i]) / denom) / v;
                if (std::isfinite(u)) {
                    measured_u_sum_ += u;
                    measured_u2_sum_ += u * u;
                    measured_u_cells_ += 1.0;
                }
            }
            const double alpha =
                measured_llr::tailAlpha(measured_u_sum_, measured_u2_sum_, measured_u_cells_);
            // Pass B: write the cell variances.
            for (size_t i = 0; i < nd; ++i) {
                if (i < carrier_erasure_flags_.size() && carrier_erasure_flags_[i] != 0) {
                    continue;  // already a declared erasure (nv = MAX)
                }
                const double denom = denomOf(i);
                if (!(denom > 1.0e-12) || !std::isfinite(denom)) {
                    continue;
                }
                const double model = static_cast<double>(carrier_noise_var[i]);
                const double v = std::max(model, s2 / denom);
                double nv = v;
                if (measured_cell_r2_[i] >= 0.0f) {
                    nv = measured_llr::cellVariance(
                        v, static_cast<double>(measured_cell_r2_[i]) / denom, alpha);
                }
                nv = std::max(nv, model);
                carrier_noise_var[i] =
                    static_cast<float>(std::min(static_cast<double>(MAX_CARRIER_NOISE_VAR), nv));
            }
        }
    }

    return equalized;
}

const std::vector<Complex>& OFDMDemodulator::Impl::equalize(const std::vector<Complex>& freq_domain) {
    const auto& result = equalize(freq_domain, config.modulation);

    // DEBUG: Print first few equalized symbols on first data symbol
    if (soft_bits.empty() && !result.empty()) {
        char eq_buf[256] = "";
        int ep = 0;
        for (size_t i = 0; i < std::min(size_t(5), result.size()); ++i) {
            ep += snprintf(eq_buf + ep, sizeof(eq_buf) - ep, "(%.3f,%.3f) ", result[i].real(), result[i].imag());
        }
        LOG_DEMOD(DEBUG, "EQ first 5 equalized (sym 0): %s", eq_buf);
    }

    return result;
}

} // namespace ultra
