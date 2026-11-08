// Measured total-error LLR noise model (src/ofdm/measured_llr_noise.hpp).
//
// Proves the three properties the equalizer relies on:
//   1. A correctly-modelled complex-Gaussian residual (standardized |e|^2 ~ Exp(1),
//      moment ratio R = 2) yields alpha = "Gaussian", so the per-cell blend is a no-op.
//   2. A Gaussian scale mixture with inverse-gamma(alpha0) variances is recognised and
//      alpha0 is recovered from the data alone (R = 2(alpha0-1)/(alpha0-2)).
//   3. A decision-truncated residual (R < 2, what DD measurement produces at low SNR)
//      never triggers the blend — so the model cannot soften cells when the noise
//      really is Gaussian, which is the low-SNR regression this standardization fixed.
#include "ofdm/measured_llr_noise.hpp"

#include <cmath>
#include <cstdio>
#include <random>

namespace ml = ultra::measured_llr;

namespace {
int g_failures = 0;
void check(bool ok, const char* msg) {
    if (!ok) {
        std::printf("FAIL: %s\n", msg);
        ++g_failures;
    } else {
        std::printf("ok:   %s\n", msg);
    }
}

struct Sums {
    double u = 0.0, u2 = 0.0, n = 0.0;
    void add(double x) { u += x; u2 += x * x; n += 1.0; }
};

}  // namespace

int main() {
    std::mt19937_64 rng(0x5EED1234ULL);

    // 1. Gaussian residual: u ~ Exp(1).
    {
        std::exponential_distribution<double> ex(1.0);
        Sums s;
        for (int i = 0; i < 2000000; ++i) s.add(ex(rng));
        const double a = ml::tailAlpha(s.u, s.u2, s.n);
        std::printf("      gaussian: R=%.4f alpha=%.3g\n", (s.u2 / s.n) / std::pow(s.u / s.n, 2), a);
        check(a >= 100.0, "Gaussian residual => alpha >= 100 (blend within 1% of v)");
        const double v = 0.01;
        const double blended = ml::cellVariance(v, v, a);
        check(std::abs(blended - v) <= 0.01 * v, "Gaussian alpha: typical-cell blend is a no-op");
    }

    // 2. Inverse-gamma scale mixture, mean scale 1: w = (alpha0-1)/G, G ~ Gamma(alpha0,1).
    //    alpha0 > 4 so the sample estimate of E[u^2] has finite variance.
    for (double alpha0 : {6.0, 10.0}) {
        std::gamma_distribution<double> g(alpha0, 1.0);
        std::exponential_distribution<double> ex(1.0);
        Sums s;
        for (int i = 0; i < 4000000; ++i) {
            const double w = (alpha0 - 1.0) / g(rng);
            s.add(w * ex(rng));
        }
        const double a = ml::tailAlpha(s.u, s.u2, s.n);
        std::printf("      mixture alpha0=%.0f: alpha_hat=%.3f\n", alpha0, a);
        char msg[96];
        std::snprintf(msg, sizeof(msg), "scale mixture alpha0=%.0f recovered within 25%%", alpha0);
        check(std::abs(a - alpha0) <= 0.25 * alpha0, msg);
    }

    // 3. Decision-truncated residual (|e|^2 cannot exceed the decision-cell radius):
    //    exponential truncated at u <= 2 has R < 2 -> must stay Gaussian.
    {
        std::exponential_distribution<double> ex(1.0);
        Sums s;
        while (s.n < 1000000) {
            const double x = ex(rng);
            if (x <= 2.0) s.add(x);
        }
        const double a = ml::tailAlpha(s.u, s.u2, s.n);
        check(a == ml::kGaussianAlpha, "decision-truncated residual (R<2) => no blend");
    }

    // 3b. Evidence gate: a Gaussian residual over few cells (one OFDM symbol's worth)
    //     must essentially never trigger the blend on sampling-noise kurtosis.
    {
        std::exponential_distribution<double> ex(1.0);
        int triggered = 0;
        const int trials = 2000;
        for (int t = 0; t < trials; ++t) {
            Sums s;
            for (int i = 0; i < 51; ++i) s.add(ex(rng));
            if (ml::tailAlpha(s.u, s.u2, s.n) != ml::kGaussianAlpha) ++triggered;
        }
        std::printf("      gate: %d / %d single-symbol Gaussian frames triggered\n", triggered, trials);
        check(triggered <= trials / 50, "evidence gate: <= 2% false tail triggers at n=51 Gaussian cells");
    }

    // 4. Blend shape and edge cases.
    {
        const double v = 0.02;
        check(ml::cellVariance(v, 5.0 * v, ml::kGaussianAlpha) == v,
              "Gaussian alpha returns the expected variance exactly");
        const double a = 4.0;
        const double lo = ml::cellVariance(v, 0.0, a);
        const double mid = ml::cellVariance(v, v, a);
        const double hi = ml::cellVariance(v, 10.0 * v, a);
        check(lo < mid && mid < hi, "blend is monotone in the cell's own residual");
        check(std::abs(lo - v * (a - 1.0) / (a + 1.0)) < 1e-12,
              "zero-residual cell gets (alpha-1)/(alpha+1) of v (caller floors at the model)");
        check(ml::tailAlpha(0.0, 0.0, 0.0) == ml::kGaussianAlpha, "no cells => Gaussian");
        check(ml::tailAlpha(NAN, 1.0, 10.0) == ml::kGaussianAlpha, "non-finite sums => Gaussian");
        check(ml::cellVariance(v, -1.0, a) == v, "invalid residual => expected variance");
    }

    if (g_failures == 0) {
        std::printf("PASS: measured LLR noise model\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
