#include "ultra/dsp.hpp"

#include <cmath>
#include <iostream>
#include <vector>

using namespace ultra;

namespace {

constexpr float PI = 3.14159265358979323846f;

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
            return; \
        } \
    } while (0)

bool near(float a, float b, float tol = 1e-4f) {
    return std::abs(a - b) <= tol;
}

void checkFinite(const Samples& samples, const std::string& label) {
    CHECK(!samples.empty(), label + " should produce output");
    for (float sample : samples) {
        CHECK(std::isfinite(sample), label + " output should be finite");
    }
}

void test_dsp_utilities() {
    Samples samples = {-1.0f, 0.5f, 1.0f, -0.5f};
    CHECK(near(dsp::peak(SampleSpan(samples)), 1.0f), "peak should find max absolute sample");
    CHECK(near(dsp::rms(SampleSpan(samples)), std::sqrt(0.625f)), "rms should compute mean square root");
    CHECK(dsp::peak(SampleSpan{}) == 0.0f, "empty peak should be zero");
    CHECK(dsp::rms(SampleSpan{}) == 0.0f, "empty rms should be zero");

    auto norm = dsp::normalize(SampleSpan(samples), 0.5f);
    CHECK(near(dsp::peak(SampleSpan(norm)), 0.5f), "normalize should set target peak");

    Samples zeros(8, 0.0f);
    auto norm_zeros = dsp::normalize(SampleSpan(zeros), 0.5f);
    CHECK(norm_zeros == zeros, "normalize should preserve near-silent input");

    CHECK(near(dsp::fromDb(dsp::toDb(0.25f)), 0.25f, 1e-3f), "dB conversion round-trip");
}

void test_windows() {
    for (auto window : {dsp::Window::Hann, dsp::Window::Hamming,
                        dsp::Window::Blackman, dsp::Window::Kaiser}) {
        auto coeffs = dsp::createWindow(17, window);
        CHECK(coeffs.size() == 17, "window size should match request");
        for (float coeff : coeffs) {
            CHECK(std::isfinite(coeff), "window coefficients should be finite");
        }

        Samples ones(17, 1.0f);
        auto applied = dsp::applyWindow(SampleSpan(ones), window);
        CHECK(applied.size() == ones.size(), "applyWindow should preserve sample count");
        for (size_t i = 0; i < applied.size(); ++i) {
            CHECK(near(applied[i], coeffs[i]), "windowed unit samples should equal coefficients");
        }
    }
}

void test_fir_filter() {
    FIRFilter fir({0.25f, 0.5f, 0.25f});
    CHECK(near(fir.process(1.0f), 0.25f), "FIR impulse first tap");
    CHECK(near(fir.process(0.0f), 0.5f), "FIR impulse second tap");
    CHECK(near(fir.process(0.0f), 0.25f), "FIR impulse third tap");
    CHECK(near(fir.process(0.0f), 0.0f), "FIR impulse tail");

    fir.reset();
    Samples impulse = {1.0f, 0.0f, 0.0f, 0.0f};
    auto block = fir.process(SampleSpan(impulse));
    CHECK(block.size() == impulse.size(), "FIR block output size");
    CHECK(near(block[0], 0.25f) && near(block[1], 0.5f) &&
          near(block[2], 0.25f) && near(block[3], 0.0f),
          "FIR block impulse response");

    auto low = FIRFilter::lowpass(31, 1200.0f, 48000.0f);
    auto high = FIRFilter::highpass(31, 1200.0f, 48000.0f);
    auto band = FIRFilter::bandpass(31, 300.0f, 2700.0f, 48000.0f);
    Samples step(64, 1.0f);
    checkFinite(low.process(SampleSpan(step)), "lowpass FIR");
    checkFinite(high.process(SampleSpan(step)), "highpass FIR");
    checkFinite(band.process(SampleSpan(step)), "bandpass FIR");
}

void test_biquad_filter() {
    Samples impulse(64, 0.0f);
    impulse[0] = 1.0f;

    auto low = BiquadFilter::lowpass(1200.0f, 0.707f, 48000.0f);
    auto high = BiquadFilter::highpass(1200.0f, 0.707f, 48000.0f);
    auto band = BiquadFilter::bandpass(1200.0f, 1.0f, 48000.0f);
    auto notch = BiquadFilter::notch(1200.0f, 10.0f, 48000.0f);

    checkFinite(low.process(SampleSpan(impulse)), "lowpass biquad");
    checkFinite(high.process(SampleSpan(impulse)), "highpass biquad");
    checkFinite(band.process(SampleSpan(impulse)), "bandpass biquad");
    checkFinite(notch.process(SampleSpan(impulse)), "notch biquad");

    low.reset();
    float first = low.process(1.0f);
    low.reset();
    CHECK(near(low.process(1.0f), first), "biquad reset should restore deterministic state");
}

void test_nco() {
    NCO dc(0.0f, 48000.0f);
    for (int i = 0; i < 4; ++i) {
        auto sample = dc.next();
        CHECK(near(sample.real(), 1.0f) && near(sample.imag(), 0.0f),
              "zero-frequency NCO should stay at unit phase");
    }

    NCO quarter(12000.0f, 48000.0f);
    auto s0 = quarter.next();
    auto s1 = quarter.next();
    auto s2 = quarter.next();
    auto s3 = quarter.next();
    CHECK(near(s0.real(), 1.0f) && near(s0.imag(), 0.0f), "quarter-rate NCO phase 0");
    CHECK(std::abs(s1.real()) < 1e-4f && near(s1.imag(), 1.0f), "quarter-rate NCO phase 90");
    CHECK(near(s2.real(), -1.0f) && std::abs(s2.imag()) < 1e-4f, "quarter-rate NCO phase 180");
    CHECK(std::abs(s3.real()) < 1e-4f && near(s3.imag(), -1.0f), "quarter-rate NCO phase 270");

    quarter.reset();
    CHECK(near(quarter.next().real(), 1.0f), "NCO reset should return to phase zero");

    Samples real_samples = {1.0f, 1.0f, 1.0f};
    auto mixed_real = quarter.mix(SampleSpan(real_samples));
    CHECK(mixed_real.size() == real_samples.size(), "NCO real mix size");

    std::vector<Complex> complex_samples = {Complex(1, 0), Complex(0, 1)};
    auto mixed_complex = quarter.mix(complex_samples);
    CHECK(mixed_complex.size() == complex_samples.size(), "NCO complex mix size");

    quarter.setFrequency(-12000.0f);
    CHECK(std::isfinite(quarter.next().real()), "NCO negative frequency should be finite");
}

void test_hilbert_transform() {
    Samples sine(256);
    for (size_t i = 0; i < sine.size(); ++i) {
        sine[i] = std::sin(2.0f * PI * 1000.0f *
                           static_cast<float>(i) / 48000.0f);
    }

    HilbertTransform hilbert(64);  // Even request should be rounded to odd internally.
    auto analytic = hilbert.process(SampleSpan(sine));
    CHECK(analytic.size() == sine.size(), "Hilbert output size");

    bool has_quadrature = false;
    for (const auto& sample : analytic) {
        CHECK(std::isfinite(sample.real()) && std::isfinite(sample.imag()),
              "Hilbert output should be finite");
        if (std::abs(sample.imag()) > 1e-3f) {
            has_quadrature = true;
        }
    }
    CHECK(has_quadrature, "Hilbert output should contain quadrature component");
}

}  // namespace

int main() {
    test_dsp_utilities();
    test_windows();
    test_fir_filter();
    test_biquad_filter();
    test_nco();
    test_hilbert_transform();

    if (tests_failed != 0) {
        std::cout << "DSPPrimitives: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "DSPPrimitives: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
