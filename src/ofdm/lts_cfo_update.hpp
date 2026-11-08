#pragma once

// Frame-constant CFO update from the LTS pair (2026-11-08). Replaces the 2026-07-29 "seed
// gate" (`|chirp CFO| >= 0.75 Hz || across-carrier CV < 0.20`), which was a heuristic on the
// wrong quantities: its seed branch was DEAD in production (the decoder pre-corrects every
// OFDM frame by the tracked CFO and hands the demod 0 Hz, so |old CFO| >= 0.75 never held),
// and its flatness branch is false on every Rayleigh channel. Net effect: on fading the LTS
// never corrected a wrong chirp CFO. Measured (measure_ack_fer, production geometry, ITU
// Moderate, 3 seeds x 48 frames): QPSK R1/2 14.6 % / 17.4 % FER at 20 / 26 dB with the gate,
// 0.0 % / 0.0 % with the sync CFO forced to the simulator's true value.
//
// Model. Each frame starts from a PRIOR CFO (the tracked value, applied by pre-correction).
// The LTS pair (two identical training symbols one symbol period T apart) measures the
// residual r = arg(sum_l unit(H2_l H1_l*)) / (2 pi T) with standard error sigma_r (below).
// The prior's error is SPARSE, not Gaussian: an uncertified dual-chirp CFO is accurate almost
// always (every AWGN frame; every multipath frame whose dominant path did not change between
// the two chirps) and occasionally carries a large phantom (below). Under such a two-component
// prior the posterior-mean correction is ~0 while |r| is within the LTS's own noise and ~r once
// |r| is clearly beyond it, so the frame correction is:
//     significant = |r| > kSignificanceSigmas * sigma_r        (is the prior wrong?)
//     correction  = significant ? K r : 0,   K = P / (P + R)    (MMSE given that it is)
// MEASURED why the test is needed: applying K r unconditionally (K ~ 1) took AWGN QPSK R1/2 at
// 6 dB from 0 % to 36.5 % FER — a 0.7 Hz-sigma LTS estimate replaced a chirp estimate that is
// accurate to << 0.1 Hz on a single-path channel (time-bandwidth product 1200).
//
// R. r is the circular mean of n unit phasors with mean resultant length c. For wrapped-normal
//   per-carrier phase noise the variance of the mean direction is (1 - c^4) / (2 n c^2) rad^2
//   (standard circular statistics: rho_2 = rho^4), converted to Hz^2 by 1 / (2 pi T)^2.
//
// P. The receiver knows which of two situations it is in.
//   - CERTIFIED (CFOTracker::pilotSeeded(): a decoded frame proved the tracked CFO since the
//     last reset/revoke; pilot-refined to < 0.1 Hz): P << R on any fading channel, K -> 0. The
//     frame keeps the certified prior; the pilots track the rest and the per-frame feedback
//     carries oscillator drift. (This is also exactly the pre-existing behaviour on fading.)
//   - UNCERTIFIED (a fresh dual-chirp estimate, or a revoked / rolled-back value): the
//     dual-chirp gap estimator reads CFO from the up- and down-chirp correlation-peak
//     POSITIONS. On multipath the dominant path can differ between the two chirps (they are
//     ~0.6 s apart, longer than the coherence time on ITU Moderate), moving one peak by up to
//     the delay spread tau: a phantom of up to tau / (2 kappa) Hz, kappa = fs / chirp_rate
//     samples per Hz. Any link OFDM can decode has tau <= CP, so the phantom is bounded by
//     CP / (2 kappa); a uniform (maximum-entropy) prior over that bound has
//     sigma = CP / (2 kappa sqrt 3). MEDIUM CP 96, kappa 10: bound 4.8 Hz, sigma 2.77 Hz
//     (measured phantoms: 2.4-3.1 Hz on ITU Moderate, 1 ms = 48 samples -> 2.4 Hz bound).
//     With R of order 0.1-0.5 Hz, K = 0.97-0.999: the LTS measurement takes over.
//
// Evidence. On noise the unit-phasor sum is complex Gaussian, so chi = |S|^2 / n ~ Exp(1); a
// correction is only made when chi >= ln(1000) (false-correction probability 1e-3 per frame),
// the probability-derived replacement of the old fixed coherence threshold 0.70. It is the
// same frame-evidence threshold the turbo re-anchor exemption uses.

#include <algorithm>
#include <cmath>
#include <limits>

namespace ultra {
namespace lts_cfo {

// chi = |sum of n unit phasors|^2 / n ~ Exp(1) on noise; ln(1000) => P_fa = 1e-3.
constexpr double kEvidenceChi = 6.907755278982137;

// "The prior is wrong" test at a two-sided false-correction probability of 0.27 % per frame
// when the prior is right (the normal 3-sigma level; a probability constant). sigma_r is
// estimated from the SAME n phasors, so r / sigma_r is a t-statistic with n - 1 degrees of
// freedom: the threshold is the matching t quantile (Cornish-Fisher expansion of the normal
// quantile z = 3; at n = 47 it is 3.17). Measured without the t correction: 0.51 % instead of
// 0.27 % (test LtsCfoUpdate §5).
constexpr double kSignificanceSigmas = 3.0;

inline double significanceThreshold(int n) {
    const double z = kSignificanceSigmas;
    if (n <= 2) return std::numeric_limits<double>::infinity();
    const double df = static_cast<double>(n - 1);
    return z + (z * z * z + z) / (4.0 * df) +
           (5.0 * z * z * z * z * z + 16.0 * z * z * z + 3.0 * z) / (96.0 * df * df);
}

// Standard error (Hz) of the LTS-pair residual CFO from the across-carrier coherence c of n
// unit phasors, one symbol period `symbol_seconds` apart. +inf when undefined.
inline double residualSigmaHz(double coherence, int n, double symbol_seconds) {
    if (n <= 0 || !(coherence > 0.0) || !(symbol_seconds > 0.0)) {
        return std::numeric_limits<double>::infinity();
    }
    const double c = std::min(coherence, 1.0);
    const double c2 = c * c;
    const double var_rad2 = std::max(0.0, (1.0 - c2 * c2) / (2.0 * n * c2));
    return std::sqrt(var_rad2) / (2.0 * M_PI * symbol_seconds);
}

// Prior sigma (Hz) of an UNCERTIFIED, dual-chirp-derived CFO: uniform over the multipath
// phantom bound CP / (2 kappa), kappa = sample_rate / chirp_rate samples per Hz.
inline double uncertifiedPriorSigmaHz(double cp_samples, double sample_rate, double f_start_hz,
                                      double f_end_hz, double chirp_seconds) {
    const double rate = std::abs(f_end_hz - f_start_hz) / chirp_seconds;  // Hz per second
    if (!(rate > 0.0) || !(sample_rate > 0.0) || !(cp_samples > 0.0)) {
        return std::numeric_limits<double>::infinity();
    }
    const double kappa = sample_rate / rate;  // samples of peak shift per Hz of CFO
    return cp_samples / (2.0 * kappa * std::sqrt(3.0));
}

// Frame correction (Hz) for residual r with standard error sigma_r (from n phasors) and prior
// sigma: 0 unless the prior is uncertified (prior > 0) and r is significant; then the MMSE K r.
inline double frameCorrectionHz(double residual_hz, double residual_sigma_hz,
                                double prior_sigma_hz, int n);

// MMSE gain K = P / (P + R). prior 0 (certified) -> 0; prior +inf (no information) -> 1.
inline double gain(double prior_sigma_hz, double residual_sigma_hz) {
    if (!(prior_sigma_hz > 0.0) || !std::isfinite(residual_sigma_hz)) {
        return 0.0;
    }
    if (!std::isfinite(prior_sigma_hz)) {
        return 1.0;
    }
    const double P = prior_sigma_hz * prior_sigma_hz;
    const double R = residual_sigma_hz * residual_sigma_hz;
    return P / (P + R);
}

inline double frameCorrectionHz(double residual_hz, double residual_sigma_hz,
                                double prior_sigma_hz, int n) {
    const double k = gain(prior_sigma_hz, residual_sigma_hz);
    if (!(k > 0.0) ||
        !(std::abs(residual_hz) > significanceThreshold(n) * residual_sigma_hz)) {
        return 0.0;
    }
    return k * residual_hz;
}

}  // namespace lts_cfo
}  // namespace ultra
