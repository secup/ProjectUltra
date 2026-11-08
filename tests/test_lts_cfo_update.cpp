// LTS-pair frame CFO update (src/ofdm/lts_cfo_update.hpp): the derived quantities match their
// stated laws.
//  1. residualSigmaHz: the circular-mean standard error from the coherence matches a Monte
//     Carlo of n unit phasors with wrapped-normal per-carrier phase noise (within 12 %);
//  2. evidence: on pure-noise unit phasors chi = |S|^2/n exceeds ln(1000) with probability
//     ~1e-3 (Exp(1) law);
//  3. uncertifiedPriorSigmaHz: the production chirp (300->2700 Hz, 500 ms) with the MEDIUM CP
//     (96 samples) gives CP/(2 kappa sqrt 3) = 96/(20 sqrt 3) = 2.771 Hz;
//  4. gain: certified prior -> 0, no-information prior -> 1, uncertified chirp vs a typical
//     LTS sigma -> > 0.95 (the LTS takes over), and a noisy LTS shrinks the gain.
#include "ofdm/lts_cfo_update.hpp"

#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

using namespace ultra;

namespace {
int g_fail = 0;
void check(bool ok, const char* msg) {
    std::printf("%s %s\n", ok ? "ok:  " : "FAIL:", msg);
    if (!ok) ++g_fail;
}
}  // namespace

int main() {
    const double T = 1120.0 / 48000.0;  // production symbol period (1024 + MEDIUM CP 96)
    std::mt19937_64 rng(2026);

    // 1. Standard error vs Monte Carlo.
    for (double phase_sigma : {0.15, 0.35, 0.7}) {
        const int n = 47;
        const int trials = 4000;
        std::normal_distribution<double> nd(0.0, phase_sigma);
        double sum_sq_err = 0.0, sum_pred = 0.0;
        for (int t = 0; t < trials; ++t) {
            std::complex<double> S(0, 0);
            for (int l = 0; l < n; ++l) S += std::polar(1.0, nd(rng));
            const double c = std::abs(S) / n;
            const double est_hz = std::arg(S) / (2.0 * M_PI * T);
            sum_sq_err += est_hz * est_hz;
            sum_pred += lts_cfo::residualSigmaHz(c, n, T);
        }
        const double mc = std::sqrt(sum_sq_err / trials);
        const double pred = sum_pred / trials;
        char m[160];
        std::snprintf(m, sizeof(m), "phase sigma %.2f rad: predicted SE %.3f Hz vs Monte Carlo %.3f Hz",
                      phase_sigma, pred, mc);
        check(std::abs(pred - mc) / mc < 0.12, m);
    }

    // 2. Evidence threshold false-alarm rate on noise.
    {
        const int n = 47;
        const int trials = 200000;
        std::uniform_real_distribution<double> ud(-M_PI, M_PI);
        int over = 0;
        for (int t = 0; t < trials; ++t) {
            std::complex<double> S(0, 0);
            for (int l = 0; l < n; ++l) S += std::polar(1.0, ud(rng));
            if (std::norm(S) / n >= lts_cfo::kEvidenceChi) ++over;
        }
        const double p = static_cast<double>(over) / trials;
        char m[128];
        std::snprintf(m, sizeof(m), "noise chi >= ln(1000): P = %.5f (expected 0.001)", p);
        check(p > 0.0006 && p < 0.0015, m);
    }

    // 3. Uncertified (dual-chirp) prior sigma from geometry.
    {
        const double s = lts_cfo::uncertifiedPriorSigmaHz(96.0, 48000.0, 300.0, 2700.0, 0.5);
        char m[96];
        std::snprintf(m, sizeof(m), "uncertified prior sigma = %.3f Hz (expected 2.771)", s);
        check(std::abs(s - 96.0 / (20.0 * std::sqrt(3.0))) < 1e-9, m);
    }

    // 4. Gain limits.
    {
        const double chirp = lts_cfo::uncertifiedPriorSigmaHz(96.0, 48000.0, 300.0, 2700.0, 0.5);
        check(lts_cfo::gain(0.0, 0.2) == 0.0, "certified prior: K = 0 (frame keeps the certified CFO)");
        check(lts_cfo::gain(INFINITY, 0.2) == 1.0, "no prior information: K = 1");
        check(lts_cfo::gain(chirp, 0.2) > 0.99, "uncertified chirp vs LTS sigma 0.2 Hz: K > 0.99");
        check(lts_cfo::gain(chirp, 0.5) > 0.96, "uncertified chirp vs LTS sigma 0.5 Hz: K > 0.96");
        check(lts_cfo::gain(chirp, 3.0) < 0.5, "a very noisy LTS (sigma 3 Hz) gets less than half the weight");
        check(lts_cfo::gain(chirp, INFINITY) == 0.0, "undefined LTS sigma: K = 0");
    }

    // 5. The full decision on simulated LTS pairs (47 carriers, 0.35 rad per-carrier noise):
    //    a right prior (true residual 0) is almost never corrected; a 2.5 Hz chirp phantom is
    //    always corrected to within 3 sigma; a certified prior is never corrected.
    {
        const double chirp = lts_cfo::uncertifiedPriorSigmaHz(96.0, 48000.0, 300.0, 2700.0, 0.5);
        std::normal_distribution<double> nd(0.0, 0.35);
        const int n = 47, trials = 20000;
        auto measure = [&](double true_hz, double& r, double& sig) {
            std::complex<double> S(0, 0);
            const double rot = 2.0 * M_PI * true_hz * T;
            for (int l = 0; l < n; ++l) S += std::polar(1.0, rot + nd(rng));
            r = std::arg(S) / (2.0 * M_PI * T);
            sig = lts_cfo::residualSigmaHz(std::abs(S) / n, n, T);
        };
        int false_corr = 0, phantom_fixed = 0, certified_moved = 0;
        for (int t = 0; t < trials; ++t) {
            double r, sig;
            measure(0.0, r, sig);
            if (lts_cfo::frameCorrectionHz(r, sig, chirp, n) != 0.0) ++false_corr;
            if (lts_cfo::frameCorrectionHz(r, sig, 0.0, n) != 0.0) ++certified_moved;
            measure(2.5, r, sig);
            const double c = lts_cfo::frameCorrectionHz(r, sig, chirp, n);
            if (std::abs(c - 2.5) < 3.0 * sig) ++phantom_fixed;
        }
        char m[160];
        std::snprintf(m, sizeof(m), "right prior: corrected on %.3f %% of frames (expected 0.27 %%)",
                      100.0 * false_corr / trials);
        check(false_corr > trials * 0.0012 && false_corr < trials * 0.0045, m);
        std::snprintf(m, sizeof(m), "2.5 Hz chirp phantom: corrected on %.2f %% of frames", 100.0 * phantom_fixed / trials);
        check(phantom_fixed > trials * 0.99, m);
        check(certified_moved == 0, "certified prior: never corrected");
    }

    if (g_fail == 0) {
        std::printf("PASS: LTS CFO update\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
