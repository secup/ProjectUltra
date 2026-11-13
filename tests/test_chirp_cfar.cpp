// BUG-NB-CHIRP-FALSE-LOCK-FILTERED (2026-11-12): the dual-chirp detector's threshold must be
// referenced to the noise actually in the chirp's band.
//
// A normalized matched-filter correlation against noise has a spread set by the noise inside the
// template's band. White noise over the whole 24 kHz stream leaves a 500 Hz chirp's noise peaks
// near 0.03; a radio's 500 Hz IF filter puts all the noise in that band and lifts them to
// 0.16-0.21 (GUI narrowband behind --rx-filter 1250-1750: 16-21 false "anchors" per station per
// 4 KB transfer at the fixed 0.15 threshold, against 0.99 for every real one).
//
// 1. Band-limited noise alone (1250-1750 Hz): the narrowband detector reports no chirp.
// 2. A real narrowband dual chirp in that noise at an in-band SNR of 0 and -6 dB is still found.
// 3. White noise: no chirp for the narrowband or the wideband detector, and the noise-referenced
//    floor stays under the fixed 0.15 - every decision there is the one the fixed threshold made.
// 4. A wideband chirp at an in-band SNR of -4 dB in white noise is still found (the fixed
//    threshold's own reach there: -10 dB in-band is -20 dB over the whole 24 kHz, correlation 0.1).
#include "sync/chirp_sync.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace ultra;
using namespace ultra::sync;

namespace {
int failures = 0;
#define CHECK(cond, msg)                                         \
    do {                                                         \
        if (!(cond)) {                                           \
            ++failures;                                          \
            std::cout << "FAIL: " << msg << "\n";                \
        }                                                        \
    } while (0)

constexpr float kFs = 48000.0f;
constexpr size_t kWindow = 120000;  // the receiver's 2.5 s full-anchor search window

std::vector<float> whiteNoise(size_t n, uint32_t seed, float sigma) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> g(0.0f, sigma);
    std::vector<float> v(n);
    for (auto& x : v) x = g(rng);
    return v;
}

// White Gaussian noise brick-wall filtered to [lo, hi] Hz (one power-of-2 block, the first n
// samples kept), scaled to unit in-band RMS.
std::vector<float> bandNoise(size_t n, uint32_t seed, float lo, float hi) {
    size_t m = 1;
    while (m < n) m <<= 1;
    const auto w = whiteNoise(m, seed, 1.0f);
    FFT fft(m);
    std::vector<Complex> in(m), spec(m), out(m);
    for (size_t i = 0; i < m; ++i) in[i] = Complex(w[i], 0.0f);
    fft.forward(in, spec);
    for (size_t k = 0; k < m; ++k) {
        const float f = static_cast<float>(k) * kFs / static_cast<float>(m);
        const float f_abs = (k <= m / 2) ? f : kFs - f;
        if (f_abs < lo || f_abs > hi) spec[k] = Complex(0.0f, 0.0f);
    }
    fft.inverse(spec, out);
    std::vector<float> v(n);
    double e = 0.0;
    for (size_t i = 0; i < n; ++i) {
        v[i] = out[i].real();
        e += static_cast<double>(v[i]) * v[i];
    }
    const float rms = static_cast<float>(std::sqrt(e / static_cast<double>(n)));
    for (auto& x : v) x /= rms;
    return v;
}

ChirpConfig narrowConfig() {
    ChirpConfig c;
    c.f_start = 1250.0f;
    c.f_end = 1750.0f;
    return c;
}

// Mean power of the chirp burst while it is on (the in-band signal power the SNR refers to).
float chirpOnPower(const std::vector<float>& chirp) {
    double e = 0.0;
    size_t on = 0;
    for (float s : chirp) {
        if (std::fabs(s) > 1e-6f) {
            e += static_cast<double>(s) * s;
            ++on;
        }
    }
    return on ? static_cast<float>(e / static_cast<double>(on)) : 0.0f;
}

int falseLocks(const ChirpSync& sync, bool band_limited, float lo, float hi, int windows,
               uint32_t seed0) {
    int locks = 0;
    for (int w = 0; w < windows; ++w) {
        const auto noise = band_limited
                               ? bandNoise(kWindow, seed0 + static_cast<uint32_t>(w), lo, hi)
                               : whiteNoise(kWindow, seed0 + static_cast<uint32_t>(w), 1.0f);
        if (sync.detectDualChirp(SampleSpan(noise.data(), noise.size()), 0.15f).success) ++locks;
    }
    return locks;
}

bool findsChirp(const ChirpSync& sync, bool band_limited, float lo, float hi, float snr_db,
                uint32_t seed) {
    const auto chirp = sync.generate();
    auto noise = band_limited ? bandNoise(kWindow, seed, lo, hi) : whiteNoise(kWindow, seed, 1.0f);
    // White noise is scaled so that its power INSIDE the chirp's band sets the SNR.
    const float in_band_fraction = band_limited ? 1.0f : (hi - lo) / (kFs / 2.0f);
    const float noise_in_band = in_band_fraction;  // unit total power
    const float signal_power = chirpOnPower(chirp);
    const float gain = std::sqrt(noise_in_band * std::pow(10.0f, snr_db / 10.0f) / signal_power);
    const size_t offset = 20000;
    for (size_t i = 0; i < chirp.size() && offset + i < noise.size(); ++i) {
        noise[offset + i] += gain * chirp[i];
    }
    const auto r = sync.detectDualChirp(SampleSpan(noise.data(), noise.size()), 0.15f);
    return r.success && std::abs(r.up_chirp_start - static_cast<int>(offset)) < 400;
}
}  // namespace

int main() {
    ultra::setLogLevel(ultra::LogLevel::ERROR);
    const ChirpSync narrow(narrowConfig());
    const ChirpSync wide;  // 300-2700 Hz

    const int nb_locks = falseLocks(narrow, /*band_limited=*/true, 1250.0f, 1750.0f, 30, 100);
    std::cout << "band-limited noise, narrowband detector: " << nb_locks << " false locks in 30 "
              << "windows (floor " << narrow.lastNoiseFloorCorrelation() << ")\n";
    CHECK(nb_locks == 0, "noise filtered to the chirp's band must not read as a chirp");

    for (float snr : {0.0f, -6.0f}) {
        int found = 0;
        for (uint32_t s = 0; s < 6; ++s) found += findsChirp(narrow, true, 1250.0f, 1750.0f, snr, 500 + s);
        std::cout << "narrowband chirp in band-limited noise at " << snr << " dB: " << found
                  << "/6 found\n";
        CHECK(found == 6, "a real narrowband chirp at " << snr << " dB in-band must be found");
    }

    const int nb_white = falseLocks(narrow, false, 0, 0, 10, 200);
    const float nb_white_floor = narrow.lastNoiseFloorCorrelation();
    const int wb_white = falseLocks(wide, false, 0, 0, 10, 300);
    const float wb_white_floor = wide.lastNoiseFloorCorrelation();
    std::cout << "white noise: narrowband " << nb_white << " / wideband " << wb_white
              << " false locks; floors " << nb_white_floor << " / " << wb_white_floor << "\n";
    CHECK(nb_white == 0 && wb_white == 0, "white noise must not read as a chirp");
    CHECK(nb_white_floor < 0.15f && wb_white_floor < 0.15f,
          "in white noise the noise-referenced floor stays under the fixed threshold");

    int wb_found = 0;
    for (uint32_t s = 0; s < 6; ++s) wb_found += findsChirp(wide, false, 300.0f, 2700.0f, -4.0f, 700 + s);
    std::cout << "wideband chirp in white noise at -4 dB in-band: " << wb_found << "/6 found\n";
    CHECK(wb_found == 6, "the wideband chirp's sensitivity in white noise is unchanged");

    if (failures) {
        std::cout << "ChirpCfar: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "ChirpCfar: all checks passed\n";
    return 0;
}
