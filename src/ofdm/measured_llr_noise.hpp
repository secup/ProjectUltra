#pragma once

// Measured total-error LLR noise model (ULTRA_LLR_MEASURED_NV).
//
// Technique adopted from an open-source HF modem's receiver, which sets its LLR
// noise variance from the frame's own decision residual instead of a thermal
// model; generalized here to a Gaussian SCALE MIXTURE so that heavy-tailed
// (channel-estimate-error) residuals are handled from the data, with no
// tuned constant.
//
// Model. The equalized error of cell c is complex Gaussian with variance
// v_c * w_c, where v_c is the cell's EXPECTED variance (thermal model, or the
// measured frame floor, whichever is larger) and w_c is a random scale with
// an inverse-gamma(alpha) distribution of mean 1. alpha -> infinity is the
// plain Gaussian; small alpha means some cells err far more than expected.
//
// Tail weight from the data. With u_c = |e_c|^2 / v_c (the standardized
// residual) the moment ratio R = E[u^2] / E[u]^2 is exactly 2 for a complex
// Gaussian (|e|^2 is exponential) and 2(alpha-1)/(alpha-2) for the mixture, so
//     alpha = 2(R-1)/(R-2),   R > 2;     alpha = infinity,   R <= 2.
// Standardizing first matters: carrier-to-carrier |H|^2 spread is already in
// v_c, so it must not be read as excess tail (that mistake regressed the
// near-threshold SNR region by up to 6 frames in 96 on ITU Good).
//
// Per-cell variance. Integrating the Gaussian likelihood over the mixture
// gives a Student-t likelihood, -ln p = (alpha+1) ln((alpha-1) v + d^2) + c.
// To first order in the distance difference between the competing
// constellation points, the max-log LLR equals the Gaussian one with the
// effective variance
//     v_eff = ((alpha - 1) v + e_c) / (alpha + 1),
// i.e. the cell's own residual e_c is evidence about ITS variance, weighted
// by how heavy the measured tails are. alpha -> infinity recovers v.

#include <algorithm>
#include <cmath>

namespace ultra {
namespace measured_llr {

// Larger than any alpha the data can produce; means "Gaussian, no blend".
constexpr double kGaussianAlpha = 1.0e9;

// Evidence gate. Under the Gaussian null (u ~ Exp(1)) the delta method gives
// Var(R_hat) = [Var(u^2) + 16 Var(u) - 8 Cov(u, u^2)] / n = (20 + 16 - 32) / n = 4/n,
// so the standard error of R is 2/sqrt(n). Excess tail is only acted on when it is
// 3 standard errors above 2 (a probability constant, not a tuned threshold): with few
// cells (early in a frame) only strong tails count; sampling noise never triggers the
// blend. Without this gate a correctly-modelled low-SNR frame (where the Gaussian LLR
// is optimal) had its boundary cells softened on noise-level kurtosis — measured
// +3..7 frames/96 at AWGN 4 dB, QPSK R1/4-R1/2.
constexpr double kTailEvidenceSigmas = 3.0;

// alpha from the running sums of the standardized residual u (n cells).
inline double tailAlpha(double sum_u, double sum_u2, double n) {
    if (!(n > 0.0) || !(sum_u > 0.0) || !std::isfinite(sum_u) || !std::isfinite(sum_u2)) {
        return kGaussianAlpha;
    }
    const double mean_u = sum_u / n;
    const double R = (sum_u2 / n) / (mean_u * mean_u);
    const double gate = kTailEvidenceSigmas * 2.0 / std::sqrt(n);
    if (!(R > 2.0 + gate) || !std::isfinite(R)) {
        return kGaussianAlpha;  // no significant excess tail (DD truncation pushes R below 2)
    }
    return std::min(kGaussianAlpha, 2.0 * (R - 1.0) / (R - 2.0));
}

// First-order Student-t effective variance of one cell.
//   expected_var : v, the cell's expected equalized-domain variance (> 0)
//   cell_residual: e_c, the cell's own equalized-domain squared residual
inline double cellVariance(double expected_var, double cell_residual, double alpha) {
    if (!(alpha < kGaussianAlpha) || !(cell_residual >= 0.0) || !std::isfinite(cell_residual)) {
        return expected_var;
    }
    return ((alpha - 1.0) * expected_var + cell_residual) / (alpha + 1.0);
}

}  // namespace measured_llr
}  // namespace ultra
