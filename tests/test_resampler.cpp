// Tests for ultra::Resampler (src/dsp/resampler.cpp): arbitrary-ratio windowed-sinc.
//
//   1. Upsampling (12 -> 48 kHz, KiwiSDR audio) and a non-integer ratio (44.1 -> 48 kHz) give the
//      ideal 48 kHz signal: error 60 dB under the signal away from the ends.
//   2. Downsampling (96 -> 48 kHz) keeps an in-band tone and rejects one above the new Nyquist.
//   3. Streaming in blocks equals one-shot processing; process + flush owes exactly
//      ceil(n x out / in) samples.

#include "ultra/dsp.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace ultra;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { ++tests_failed; std::cout << "FAIL: " << msg << "\n"; } \
    } while (0)

Samples tones(const std::vector<double>& freqs, double rate, size_t n) {
    Samples x(n);
    for (size_t i = 0; i < n; ++i) {
        double v = 0.0;
        for (double f : freqs) v += std::sin(2.0 * M_PI * f * static_cast<double>(i) / rate);
        x[i] = static_cast<float>(v / static_cast<double>(freqs.size()));
    }
    return x;
}

Samples resampleAll(uint32_t in_rate, uint32_t out_rate, const Samples& x, size_t block) {
    Resampler r(in_rate, out_rate);
    Samples y;
    for (size_t i = 0; i < x.size(); i += block) {
        const size_t n = std::min(block, x.size() - i);
        const Samples part = r.process(SampleSpan(x.data() + i, n));
        y.insert(y.end(), part.begin(), part.end());
    }
    const Samples tail = r.flush();
    y.insert(y.end(), tail.begin(), tail.end());
    return y;
}

// Error of y against the ideal tones at out_rate, in dB under the ideal's power, over the
// middle of the signal (the ends see the stream's zero edges).
double errorDb(const Samples& y, const std::vector<double>& freqs, double out_rate) {
    const Samples ideal = tones(freqs, out_rate, y.size());
    const size_t skip = y.size() / 10;
    double e = 0.0, p = 0.0;
    for (size_t i = skip; i + skip < y.size(); ++i) {
        const double d = static_cast<double>(y[i]) - ideal[i];
        e += d * d;
        p += static_cast<double>(ideal[i]) * ideal[i];
    }
    return 10.0 * std::log10(e / p);
}

void test_upsampling_matches_ideal() {
    const std::vector<double> f = {1000.0, 2500.0};
    for (uint32_t in_rate : {12000u, 44100u, 8000u, 16000u}) {
        const Samples x = tones(f, in_rate, in_rate);  // 1 s
        const Samples y = resampleAll(in_rate, 48000, x, x.size());
        const double err = errorDb(y, f, 48000.0);
        CHECK(err < -60.0, std::to_string(in_rate) + " -> 48000 error " + std::to_string(err) +
                               " dB");
        CHECK(y.size() == static_cast<size_t>(std::ceil(x.size() * 48000.0 / in_rate)),
              std::to_string(in_rate) + " -> 48000 length " + std::to_string(y.size()));
    }
}

void test_downsampling_rejects_above_nyquist() {
    const Samples in_band = tones({1000.0}, 96000.0, 96000);
    const Samples y = resampleAll(96000, 48000, in_band, in_band.size());
    CHECK(errorDb(y, {1000.0}, 48000.0) < -60.0, "96000 -> 48000 keeps a 1 kHz tone");
    const Samples out_of_band = tones({30000.0}, 96000.0, 96000);
    const Samples z = resampleAll(96000, 48000, out_of_band, out_of_band.size());
    double p = 0.0;
    for (size_t i = z.size() / 10; i + z.size() / 10 < z.size(); ++i) p += z[i] * z[i];
    p /= static_cast<double>(z.size()) * 0.8;
    CHECK(10.0 * std::log10(p / 0.5) < -60.0,
          "96000 -> 48000 rejects a 30 kHz tone by 60 dB (" +
              std::to_string(10.0 * std::log10(p / 0.5)) + " dB)");
}

void test_streaming_equals_one_shot() {
    const Samples x = tones({1300.0}, 44100.0, 44100);
    const Samples a = resampleAll(44100, 48000, x, x.size());
    const Samples b = resampleAll(44100, 48000, x, 517);  // odd block size
    bool same = a.size() == b.size();
    for (size_t i = 0; same && i < a.size(); ++i) same = std::fabs(a[i] - b[i]) < 1e-6f;
    CHECK(same, "block-by-block resampling equals one-shot");
}

}  // namespace

int main() {
    std::cout << "=== Resampler ===\n";
    test_upsampling_matches_ideal();
    test_downsampling_rejects_above_nyquist();
    test_streaming_equals_one_shot();
    std::cout << (tests_failed == 0 ? "PASS" : "FAIL") << ": "
              << (tests_run - tests_failed) << "/" << tests_run << " checks\n";
    return tests_failed == 0 ? 0 : 1;
}
