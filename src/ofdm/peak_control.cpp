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

const std::vector<CurvePoint>& mcDpsk8x1024Curve() {
    // peak_control_measure --mcdpsk (2026-11-21): G from the DBPSK burst (DQPSK within 0.1 dB), D
    // the LOWER of the two (DQPSK's, 0.5-1.4 dB below DBPSK's) - conservative for both.
    static const std::vector<CurvePoint> curve = {
        {0.0f, 5.62f, 11.92f}, {1.0f, 5.40f, 13.11f}, {2.0f, 5.15f, 14.67f}, {3.0f, 4.76f, 16.58f},
        {4.0f, 4.21f, 19.18f}, {5.0f, 3.52f, 22.55f}, {6.0f, 2.68f, 27.04f}, {7.0f, 1.74f, 32.20f},
        {8.0f, 0.78f, 38.81f},
    };
    return curve;
}

float mcDpskToneBandThresholdDb(bool dqpsk, int carriers, int samples_per_symbol) {
    constexpr float kDbpskInBandFloorDb = -6.0f;  // GUI gate 2026-11-11 (CLAUDE.md floors table)
    constexpr float kDqpskOverDbpskDb = 2.5f;     // connection_policy.hpp: BPSK->QPSK gap
    const float baud = 48000.0f / static_cast<float>(samples_per_symbol);
    const float tone_bw = static_cast<float>(carriers) * 2.0f * baud;
    return kDbpskInBandFloorDb + (dqpsk ? kDqpskOverDbpskDb : 0.0f) +
           10.0f * std::log10(3000.0f / tone_bw);
}

Choice chooseHeadroom(float eta_db) { return chooseHeadroomOnCurve(wideband5PassCurve(), eta_db); }

Choice chooseHeadroomOnCurve(const std::vector<CurvePoint>& curve, float eta_db) {
    Choice best;
    const float eta = std::pow(10.0f, eta_db / 10.0f);
    for (const auto& p : curve) {
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

float curveGainAtDb(const std::vector<CurvePoint>& curve, float h_db) {
    if (curve.empty()) return 0.0f;
    if (h_db <= curve.front().h_db) return curve.front().gain_db;
    for (size_t i = 1; i < curve.size(); ++i) {
        if (h_db <= curve[i].h_db) {
            const float t = (h_db - curve[i - 1].h_db) / (curve[i].h_db - curve[i - 1].h_db);
            return curve[i - 1].gain_db + t * (curve[i].gain_db - curve[i - 1].gain_db);
        }
    }
    return curve.back().gain_db;
}

float mcDpskDbpskDataOverControlReadingDb() {
    if (!enabled()) return 0.0f;
    const auto& curve = mcDpsk8x1024Curve();
    const Choice data = chooseHeadroomOnCurve(
        curve, mcDpskToneBandThresholdDb(/*dqpsk=*/false, 8, 1024));
    const float g_data = data.h_db < 0.0f ? 0.0f : data.curve_g_db;
    return g_data - curveGainAtDb(curve, kMcDpskControlHeadroomDb);
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

    // Designed silence - the T/R lead-in and tail, MC-DPSK's inter-chirp gap (the receiver's noise
    // reference) - is a run of exact zeros. The band limit rings into it; the clipper must not
    // create signal where the waveform specifies none, so those runs are restored afterwards and
    // left out of the level reference. 1 ms of exact 0.0f never occurs inside a real signal.
    constexpr size_t kMinSilentRun = 48;
    std::vector<uint8_t> silent(x.size(), 0);
    for (size_t i = 0; i < x.size();) {
        if (x[i] != 0.0f) { ++i; continue; }
        size_t j = i;
        while (j < x.size() && x[j] == 0.0f) ++j;
        if (j - i >= kMinSilentRun) std::fill(silent.begin() + i, silent.begin() + j, uint8_t{1});
        i = j;
    }

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
        size_t n = 0;
        for (size_t i = a; i < e; ++i) {
            if (silent[i]) continue;
            const double p = std::norm(v[i]);
            sum += p;
            pk = std::max(pk, p);
            ++n;
        }
        const double mean = n > 0 ? sum / static_cast<double>(n) : 0.0;
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
    for (size_t i = 0; i < x.size(); ++i) x[i] = silent[i] ? 0.0f : z[i].real();
    // the edges of the active span (shorter zero runs never occur there either)
    for (size_t i = 0; i < a; ++i) x[i] = 0.0f;
    for (size_t i = e; i < x.size(); ++i) x[i] = 0.0f;
    r.applied = true;
    return r;
}

bool enabled() {
    static const bool on = [] {
        // Default ON since 2026-11-21 (GUI A/B on a peak-limited TX: AWGN +24 % at 20 dB peak SNR,
        // ITU Good +33 % at 20 dB, no loss on clean channels). ULTRA_PEAK_CONTROL=0 turns it off;
        // same setting on both stations (the receiver's rate selector carries the clip floors).
        const char* v = std::getenv("ULTRA_PEAK_CONTROL");
        return !(v && v[0] == '0');
    }();
    return on;
}

}  // namespace ultra::peak_control
