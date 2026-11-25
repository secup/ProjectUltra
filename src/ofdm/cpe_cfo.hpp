#pragma once

// Frame-level residual CFO from the common-phase (CPE) trajectory (2026-11-25).
//
// The per-symbol pilot CFO tracker is disabled (channel_equalizer_pilot.cpp: its IIR drifted on
// noise), so after the LTS stage nothing in a frame MEASURES its frequency: CPE correction derotates
// each symbol's common phase and the frame decodes with an error of ~2 Hz (16 deg per symbol at
// T = 23.3 ms) — a CRC pass proves only that the error was inside the CPE capture. The CPE increments
// themselves are the measurement: their running sum theta_k is the frame's common phase at symbol k,
// theta_k = theta_0 + 2 pi f_res T k + noise (+ slow Doppler wander), so the least-squares slope over
// the frame's K data symbols is the residual CFO, and the regression's own residuals give its
// standard error (empirical, so fading phase wander widens it honestly instead of being assumed away):
//     f = sum (k - kbar)(theta_k - thetabar) / (2 pi T Sxx),  Sxx = sum (k - kbar)^2
//     sigma_f = sqrt(RSS / (K - 2) / Sxx) / (2 pi T)
// Order of magnitude: 12 pilots at 6 dB give sigma_theta ~0.1 rad per symbol; K = 10 gives
// sigma_f ~0.07 Hz, an order of magnitude below the 2-symbol LTS pair's ~1 Hz on fading.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ultra {
namespace cpe_cfo {

struct Estimate {
    double cfo_hz = std::numeric_limits<double>::quiet_NaN();     // residual CFO over the frame
    double sigma_hz = std::numeric_limits<double>::infinity();    // its standard error
    int symbols = 0;
};

// What one demodulated frame measured about its own CFO error (relative to the CFO it ran at).
struct FrameMeasurement {
    Estimate cpe;                                                  // CPE-trajectory slope
    double lts_sigma_hz = std::numeric_limits<double>::infinity(); // the LTS pair's standard error
};

// "Is the frame's CFO wrong?" at the LTS stage's 3-sigma level, with the regression's K - 2
// degrees of freedom (lts_cfo::significanceThreshold(n) uses n - 1).
inline bool significant(const Estimate& e, double threshold_sigmas) {
    return std::isfinite(e.cfo_hz) && std::isfinite(e.sigma_hz) && e.sigma_hz > 0.0 &&
           std::abs(e.cfo_hz) > threshold_sigmas * e.sigma_hz;
}

// A certificate stops the LTS stage from correcting the next frames (lts_cfo::gain -> 0). The LTS
// would correct an error only beyond its 3-sigma test, |delta| > 3 sigma_lts; a certificate whose
// own error is delta ~ N(0, sigma_cert^2) blocks such a correction with probability
// P(|z| > 3 sigma_lts / sigma_cert). With sigma_cert <= sigma_lts that is <= 0.27 %, the LTS test's
// own false-correction rate: the certificate is then never the weaker of the two. A frame whose CFO
// measurement is coarser decoded inside the CPE capture range and proves no more. (AWGN, 12 pilots
// vs 47 LTS carriers: sigma_cpe / sigma_lts ~ 1.4 sqrt(12 / K^3), 0.26 at K = 7 symbols.)
// A CFO known to 1 mHz drifts < 4 degrees over a 10 s burst: exact for any frame. The floor keeps
// a noise-free link (both sigmas ~0, the CPE's at float rounding) certifiable.
constexpr double kExactCfoSigmaHz = 1e-3;
inline bool certifies(const FrameMeasurement& m) {
    return std::isfinite(m.cpe.sigma_hz) && std::isfinite(m.lts_sigma_hz) &&
           m.cpe.sigma_hz <= std::max(m.lts_sigma_hz, kExactCfoSigmaHz);
}

// The frame confirms the value it certifies: after a significant reading was applied, or with the
// reading inside the plain 3-sigma band. Between 3 sigma and the small-sample t threshold the
// reading neither corrects nor confirms (TNC Good 8 dB, 7 symbols: -0.60 +- 0.16 Hz is 3.75 sigma,
// below t(5) = 5.2, and the -6.60 Hz prior of a -7.30 Hz link was certified): no certificate.
inline bool confirms(const Estimate& e, bool applied, double sigmas) {
    return applied || (std::isfinite(e.cfo_hz) && std::abs(e.cfo_hz) <= sigmas * e.sigma_hz);
}

// theta: cumulative common phase (rad) per data symbol, one symbol period `symbol_seconds` apart.
// Fewer than 4 symbols leave too few degrees of freedom for an error estimate: NaN / +inf.
inline Estimate fromTrajectory(const std::vector<float>& theta, double symbol_seconds) {
    Estimate e;
    const size_t K = theta.size();
    e.symbols = static_cast<int>(K);
    if (K < 4 || !(symbol_seconds > 0.0)) return e;
    const double kbar = (static_cast<double>(K) - 1.0) / 2.0;
    double tbar = 0.0;
    for (float t : theta) tbar += t;
    tbar /= static_cast<double>(K);
    double sxx = 0.0, sxy = 0.0;
    for (size_t k = 0; k < K; ++k) {
        const double dk = static_cast<double>(k) - kbar;
        sxx += dk * dk;
        sxy += dk * (theta[k] - tbar);
    }
    const double slope = sxy / sxx;  // rad per symbol
    double rss = 0.0;
    for (size_t k = 0; k < K; ++k) {
        const double r = theta[k] - tbar - slope * (static_cast<double>(k) - kbar);
        rss += r * r;
    }
    const double w = 2.0 * M_PI * symbol_seconds;
    e.cfo_hz = slope / w;
    e.sigma_hz = std::sqrt(rss / (static_cast<double>(K) - 2.0) / sxx) / w;
    return e;
}

}  // namespace cpe_cfo
}  // namespace ultra
