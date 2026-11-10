#pragma once
// ============================================================================
// DECISION-DIRECTED EVM SNR: what the receiver reads at a given true SNR (2026-11-10)
// ============================================================================
//
// The per-frame EVM SNR (channel_equalizer_equalize.cpp, "Radio-agnostic decision-directed EVM
// SNR") scores each equalized data cell y against its own hard decision d with the gain-corrected
// estimator
//     snr_dd = S_dd / (S_dd - S_de^2 / S_ee),
//     S_dd = sum |d|^2,  S_ee = sum |y|^2,  S_de = sum Re(d conj(y)).
// It reads above the true SNR for two reasons, both fixed by the constellation:
//   - the scalar-gain fit: with every decision right it returns 1 + SNR (linear);
//   - decision errors: a wrong decision lies nearer the sample than the transmitted point, so
//     near a constellation's own operating point the scatter looks smaller than it is.
// The second depends on the constellation. At a true 12 dB, QPSK reads 12.3 dB, 8PSK 12.5,
// 16QAM 13.0 and the rectangular 32QAM 15.2. A decode threshold measured on one constellation's
// frames is therefore not the threshold on another's, and the EVM rate selector compares every
// rung's threshold with the EVMs of frames sent on whatever rung the link is on.
//
// expectedReadingDb(mod, true_snr_db) is the expected reading for complex Gaussian noise on the
// equalized cells (true SNR = Es/N0 of a unit-energy constellation):
//   - rectangular QAM, QPSK, BPSK: independent PAM on each axis, so every term is a Gaussian
//     probability or partial mean over the slicer's decision intervals (exact);
//   - M-PSK: the slicer decides by angle; the radial integral is closed form and the angular one
//     is integrated per decision sector (Simpson, 256 intervals per sector).
// The estimator's noise term is a small difference of two near-equal numbers at high SNR, so it
// is built from the decision-error terms alone. Per cell,
//     E S_dd = 1 + D_A,  E S_de = 1 + D_B,  E S_ee = 1 + sigma^2,
//     D_A = E[|d|^2 - |s|^2],  D_B = E[Re((d - s) conj(y))]   (both vanish with the error rate),
//     snr_dd = (1 + D_A)(1 + sigma^2) / (sigma^2 + D_A (1 + sigma^2) - 2 D_B - D_B^2).
//
// MEASURED CHECK (measure_ack_fer, AWGN, 2026-11-10): the true SNR inferred from 16QAM frames
// predicts the EVM of 8PSK and 32QAM frames on the same channel within 0.3 dB up to an EVM of
// ~16 dB. Above that each frame geometry reaches its own channel-estimation ceiling (8PSK cw12 and
// 32QAM cw8 read 0.5-3 dB under 16QAM cw16 at 22-30 dB), which no decision model describes; every
// rung's decode threshold sits below it.
//
// The geometry here must match the transmitter (src/ofdm/modulator.cpp): QPSK and 16QAM are
// square, 32QAM is the 4 x 8 rectangle (I in {+-1, +-3}, Q in {+-1 .. +-7}), 8PSK is 8 points on
// the unit circle. CTest DdEvmModel checks every curve against a brute-force nearest-point
// simulation of the receiver's estimator.

#include "ultra/types.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace ultra {
namespace dd_evm {

namespace detail {

struct RectGeometry {
    std::vector<double> i_levels;  // integer grid levels; empty axis = no decision on it
    std::vector<double> q_levels;
};

inline bool rectGeometry(Modulation mod, RectGeometry& g) {
    switch (mod) {
        case Modulation::BPSK:
            g = {{-1, 1}, {}};
            return true;
        case Modulation::QPSK:
            g = {{-1, 1}, {-1, 1}};
            return true;
        case Modulation::QAM16:
            g = {{-3, -1, 1, 3}, {-3, -1, 1, 3}};
            return true;
        case Modulation::QAM32:
            g = {{-3, -1, 1, 3}, {-7, -5, -3, -1, 1, 3, 5, 7}};
            return true;
        case Modulation::QAM64:
            g = {{-7, -5, -3, -1, 1, 3, 5, 7}, {-7, -5, -3, -1, 1, 3, 5, 7}};
            return true;
        default:
            return false;
    }
}

inline int pskOrder(Modulation mod) {
    return mod == Modulation::QAM8 ? 8 : 0;  // QAM8 is coherent 8PSK in this modem
}

inline double meanSquare(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (double x : v) s += x * x;
    return s / static_cast<double>(v.size());
}

// Upper-tail Gaussian probability Q(z); Q(+inf) = 0, Q(-inf) = 1.
inline double qfunc(double z) { return 0.5 * std::erfc(z / std::sqrt(2.0)); }
inline double pdf(double z) {
    return std::isinf(z) ? 0.0 : std::exp(-0.5 * z * z) / std::sqrt(2.0 * M_PI);
}

// Decision-error terms of one PAM axis with levels c * L (sorted), per-axis noise std s.
// Adds this axis's share of D_A and D_B (each averaged over the axis's equiprobable levels).
inline void pamErrorTerms(const std::vector<double>& L, double c, double s, double& d_a,
                          double& d_b) {
    const size_t m = L.size();
    if (m < 2) return;  // no decision on this axis: d = s = 0 there
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> lev(m), thr(m + 1);
    for (size_t j = 0; j < m; ++j) lev[j] = c * L[j];
    thr[0] = -inf;
    thr[m] = inf;
    for (size_t j = 1; j < m; ++j) thr[j] = 0.5 * (lev[j - 1] + lev[j]);
    for (size_t k = 0; k < m; ++k) {
        const double a = lev[k];
        for (size_t j = 0; j < m; ++j) {
            if (j == k) continue;
            const double z_lo = (thr[j] - a) / s;      // region j = (thr[j], thr[j+1])
            const double z_hi = (thr[j + 1] - a) / s;
            // Region j lies wholly above or below a, so each probability is a difference of two
            // tails of the same sign: no cancellation against 1.
            const double p = (j > k) ? qfunc(z_lo) - qfunc(z_hi) : qfunc(-z_hi) - qfunc(-z_lo);
            const double mean_noise = s * (pdf(z_lo) - pdf(z_hi));  // E[n 1{region}]
            const double mean_y = a * p + mean_noise;               // E[y 1{region}]
            d_a += (lev[j] * lev[j] - a * a) * p / static_cast<double>(m);
            d_b += (lev[j] - a) * mean_y / static_cast<double>(m);
        }
    }
}

// M-PSK decision-error term D_B (D_A = 0 on the unit circle). Transmitted point at angle 0;
// sector k is centred on 2 pi k / M. With a = 1/sigma^2 and c = cos(phi), the radial integral
//     R(phi) = int_0^inf r^2 p(r, phi) dr
//            = [e^{-a sin^2 phi} sqrt(pi/a)/2 erfc(-c sqrt a) (c^2 + 1/(2a)) + e^{-a} c/(2a)] / (pi sigma^2)
// and D_B = sum_{k != 0} int_sector_k (cos(phi - theta_k) - cos phi) R(phi) dphi.
inline double pskErrorTerm(int m, double sigma2) {
    const double a = 1.0 / sigma2;
    const double sa = std::sqrt(a);
    auto radial = [&](double phi) {
        const double c = std::cos(phi);
        const double s = std::sin(phi);
        const double t1 = std::exp(-a * s * s) * 0.5 * std::sqrt(M_PI / a) *
                          std::erfc(-c * sa) * (c * c + 0.5 / a);
        const double t2 = std::exp(-a) * c * 0.5 / a;
        return (t1 + t2) / (M_PI * sigma2);
    };
    constexpr int kIntervals = 256;  // even (Simpson)
    const double half = M_PI / static_cast<double>(m);
    double d_b = 0.0;
    for (int k = 1; k < m; ++k) {
        const double theta = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(m);
        const double lo = theta - half;
        const double h = 2.0 * half / kIntervals;
        double acc = 0.0;
        for (int i = 0; i <= kIntervals; ++i) {
            const double phi = lo + h * i;
            const double f = (std::cos(phi - theta) - std::cos(phi)) * radial(phi);
            acc += f * ((i == 0 || i == kIntervals) ? 1.0 : (i % 2 ? 4.0 : 2.0));
        }
        d_b += acc * h / 3.0;
    }
    return d_b;
}

}  // namespace detail

// True for the coherent constellations this model describes.
inline bool modelled(Modulation mod) {
    detail::RectGeometry g;
    return detail::rectGeometry(mod, g) || detail::pskOrder(mod) > 0;
}

// Expected decision-directed EVM SNR (dB) at a true SNR (dB). NaN if the modulation is not
// modelled.
inline double expectedReadingDb(Modulation mod, double true_snr_db) {
    const double sigma2 = std::pow(10.0, -true_snr_db / 10.0);
    double d_a = 0.0, d_b = 0.0;
    detail::RectGeometry g;
    if (detail::rectGeometry(mod, g)) {
        const double c =
            1.0 / std::sqrt(detail::meanSquare(g.i_levels) + detail::meanSquare(g.q_levels));
        const double s = std::sqrt(0.5 * sigma2);  // per-axis noise std
        detail::pamErrorTerms(g.i_levels, c, s, d_a, d_b);
        detail::pamErrorTerms(g.q_levels, c, s, d_a, d_b);
    } else if (const int m = detail::pskOrder(mod); m > 0) {
        d_b = detail::pskErrorTerm(m, sigma2);
    } else {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double noise = sigma2 + d_a * (1.0 + sigma2) - 2.0 * d_b - d_b * d_b;
    if (!(noise > 0.0)) return std::numeric_limits<double>::infinity();
    return 10.0 * std::log10((1.0 + d_a) * (1.0 + sigma2) / noise);
}

// Range over which readings are inverted; the reading is monotone in the true SNR on it.
inline constexpr double kMinTrueSnrDb = -10.0;
inline constexpr double kMaxTrueSnrDb = 45.0;

// True SNR (dB) whose expected reading is `reading_db`, clamped to [kMinTrueSnrDb,
// kMaxTrueSnrDb]. NaN if the modulation is not modelled.
inline double trueSnrDb(Modulation mod, double reading_db) {
    if (!modelled(mod)) return std::numeric_limits<double>::quiet_NaN();
    double lo = kMinTrueSnrDb, hi = kMaxTrueSnrDb;
    if (reading_db <= expectedReadingDb(mod, lo)) return lo;
    if (reading_db >= expectedReadingDb(mod, hi)) return hi;
    for (int it = 0; it < 50; ++it) {  // 55 dB / 2^50: far below any meaningful resolution
        const double mid = 0.5 * (lo + hi);
        (expectedReadingDb(mod, mid) < reading_db ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

// The reading frames of `to` give on the channel where frames of `from` read `reading_db`
// (same true SNR). Identity when from == to; NaN if either modulation is not modelled.
inline double translateReadingDb(Modulation from, double reading_db, Modulation to) {
    if (from == to) return reading_db;
    if (!modelled(from) || !modelled(to)) return std::numeric_limits<double>::quiet_NaN();
    return expectedReadingDb(to, trueSnrDb(from, reading_db));
}

}  // namespace dd_evm
}  // namespace ultra
