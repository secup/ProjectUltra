#include "ultra/peak_control.hpp"

#define POCKETFFT_CACHE_SIZE 16
#if defined(__APPLE__) || defined(__unix__)
#define POCKETFFT_USE_POSIX_MEMALIGN
#endif
#include "pocketfft/pocketfft_hdronly.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>

namespace ultra::peak_control {
namespace {

using Cx = std::complex<float>;

size_t nextPow2(size_t n) {
    size_t p = 1;
    while (p < n) p <<= 1;
    return p;
}

void fftInPlace(std::vector<Cx>& v, bool forward) {
    const pocketfft::shape_t shape{v.size()};
    const pocketfft::stride_t stride{static_cast<std::ptrdiff_t>(sizeof(Cx))};
    pocketfft::c2c<float>(shape, stride, stride, {0}, forward, v.data(), v.data(),
                          forward ? 1.0f : 1.0f / static_cast<float>(v.size()));
}

// Keep the positive-frequency bins inside [lo, hi] (the spectrum is in `v`, forward-transformed).
void maskBand(std::vector<Cx>& spec, float fs, float lo, float hi) {
    const size_t L = spec.size();
    for (size_t b = 0; b < L; ++b) {
        const float f = (b <= L / 2) ? fs * static_cast<float>(b) / static_cast<float>(L) : -1.0f;
        if (!(f >= lo && f <= hi)) spec[b] = Cx(0.0f, 0.0f);
    }
}

}  // namespace

const std::vector<CurvePoint>& wideband5PassCurve() {
    // tools/peak_control_measure --edge-bins 3 (2026-11-21), mean over the 10 wideband rungs;
    // spread across rungs: G ±0.5 dB (the unclipped crest is an extreme value), D ±0.3 dB.
    static const std::vector<CurvePoint> curve = {
        {0.0f, 7.08f, 11.69f}, {1.0f, 6.84f, 12.85f}, {2.0f, 6.51f, 14.31f}, {3.0f, 6.09f, 16.14f},
        {4.0f, 5.54f, 18.45f}, {5.0f, 4.88f, 21.36f}, {6.0f, 4.10f, 24.94f}, {7.0f, 3.22f, 28.97f},
        {8.0f, 2.30f, 32.45f}, {9.0f, 1.35f, 34.10f}, {10.0f, 0.49f, 34.42f},
    };
    return curve;
}

Choice chooseHeadroom(float eta_db) {
    Choice best;
    const float eta = std::pow(10.0f, eta_db / 10.0f);
    for (const auto& p : wideband5PassCurve()) {
        const float d = std::pow(10.0f, p.sdr_db / 10.0f);
        if (!(d > eta)) continue;
        const float g = p.gain_db + 10.0f * std::log10(1.0f - eta / d);
        if (g > best.gain_db) {
            best.gain_db = g;
            best.h_db = p.h_db;
            best.sdr_db = p.sdr_db;
            best.curve_g_db = p.gain_db;
        }
    }
    return best;
}

namespace {
double invDb(float db) { return std::pow(10.0, -static_cast<double>(db) / 10.0); }
double inv(const Choice& c) { return c.h_db < 0.0f ? 0.0 : invDb(c.sdr_db); }  // 1/D
double g(const Choice& c) { return c.h_db < 0.0f ? 1.0 : std::pow(10.0, c.curve_g_db / 10.0); }
}  // namespace

float predictEvmAcrossRungsDb(float evm_a_db, const Choice& a, const Choice& b) {
    const double th_a = invDb(evm_a_db) - inv(a);  // 1/gamma_thermal at rung a
    if (!(th_a > 0.0)) return -99.0f;
    const double th_b = th_a * g(a) / g(b);
    return static_cast<float>(-10.0 * std::log10(th_b + inv(b)));
}

float thresholdOnFramesOfRungDb(float eta_b_db, const Choice& a, const Choice& b) {
    const double th_b = invDb(eta_b_db) - inv(b);
    if (!(th_b > 0.0)) return 99.0f;
    const double inv_e = th_b * g(b) / g(a) + inv(a);
    return static_cast<float>(-10.0 * std::log10(inv_e));
}

Result apply(std::vector<float>& x, float h_db, float fs, float lo_hz, float hi_hz) {
    Result r;
    r.h_db = h_db;
    if (x.empty() || !(h_db >= 0.0f)) return r;

    // Active span: everything between the first and last non-silent sample.
    size_t a = 0, e = x.size();
    while (a < e && x[a] == 0.0f) ++a;
    while (e > a && x[e - 1] == 0.0f) --e;
    if (e <= a) return r;

    const size_t L = nextPow2(2 * x.size());
    std::vector<Cx> z(L, Cx(0.0f, 0.0f));
    for (size_t i = 0; i < x.size(); ++i) z[i] = Cx(x[i], 0.0f);
    fftInPlace(z, true);
    // analytic + band limit in one mask (positive frequencies, doubled)
    maskBand(z, fs, lo_hz, hi_hz);
    for (auto& c : z) c *= 2.0f;
    fftInPlace(z, false);

    auto crest = [&](const std::vector<Cx>& v) {
        double sum = 0.0, pk = 0.0;
        for (size_t i = a; i < e; ++i) {
            const double p = std::norm(v[i]);
            sum += p;
            pk = std::max(pk, p);
        }
        const double mean = sum / static_cast<double>(e - a);
        return std::pair<double, double>(mean, mean > 0.0 ? pk / mean : 0.0);
    };
    const auto [mean_env, pre] = crest(z);
    r.pre_crest_db = pre > 0.0 ? static_cast<float>(10.0 * std::log10(pre)) : 0.0f;
    if (!(mean_env > 0.0)) return r;
    const float A = static_cast<float>(std::sqrt(mean_env)) * std::pow(10.0f, h_db / 20.0f);

    static const float kOvershoot[] = {1.0f, 1.25f, 1.5f, 1.75f, 2.0f};
    for (float k : kOvershoot) {
        for (auto& c : z) {
            const float m = std::abs(c);
            if (m > A) c *= std::pow(A / m, k);
        }
        fftInPlace(z, true);
        maskBand(z, fs, lo_hz, hi_hz);
        fftInPlace(z, false);
    }
    const auto post = crest(z).second;
    r.post_crest_db = post > 0.0 ? static_cast<float>(10.0 * std::log10(post)) : 0.0f;
    for (size_t i = 0; i < x.size(); ++i) x[i] = z[i].real();
    // outside the original span the band limit leaves ringing below the clip level; keep the
    // T/R silence exact (the half-duplex model and the receiver's digital-silence test read it)
    for (size_t i = 0; i < a; ++i) x[i] = 0.0f;
    for (size_t i = e; i < x.size(); ++i) x[i] = 0.0f;
    r.applied = true;
    return r;
}

bool enabled() {
    static const bool on = [] {
        const char* v = std::getenv("ULTRA_PEAK_CONTROL");
        return v && v[0] == '1';
    }();
    return on;
}

}  // namespace ultra::peak_control
