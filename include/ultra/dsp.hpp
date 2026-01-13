#pragma once

#include "types.hpp"
#include <memory>

namespace ultra {

/**
 * FFT wrapper
 *
 * Abstracts FFTW3 (if available) or fallback implementation.
 * Provides both complex and real FFTs.
 */
class FFT {
public:
    explicit FFT(size_t size);
    ~FFT();

    // Complex forward FFT: time -> frequency
    void forward(const Complex* in, Complex* out);
    void forward(const std::vector<Complex>& in, std::vector<Complex>& out);

    // Complex inverse FFT: frequency -> time
    void inverse(const Complex* in, Complex* out);
    void inverse(const std::vector<Complex>& in, std::vector<Complex>& out);

    // Real forward FFT: N real -> N/2+1 complex
    void forwardReal(const Sample* in, Complex* out);

    // Real inverse FFT: N/2+1 complex -> N real
    void inverseReal(const Complex* in, Sample* out);

    size_t size() const { return size_; }

private:
    size_t size_;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * FIR Filter
 */
class FIRFilter {
public:
    // Create from coefficients
    explicit FIRFilter(std::vector<Sample> coeffs);

    // Create standard filters
    static FIRFilter lowpass(size_t taps, float cutoff, float sample_rate);
    static FIRFilter highpass(size_t taps, float cutoff, float sample_rate);
    static FIRFilter bandpass(size_t taps, float low, float high, float sample_rate);

    // Filter a single sample (stateful)
    Sample process(Sample in);

    // Filter a block of samples
    Samples process(SampleSpan in);

    // Reset filter state
    void reset();

private:
    std::vector<Sample> coeffs_;
    std::vector<Sample> delay_line_;
    size_t delay_idx_ = 0;
};

/**
 * IIR Biquad Filter
 */
class BiquadFilter {
public:
    struct Coeffs {
        float b0, b1, b2;  // Numerator
        float a1, a2;      // Denominator (a0 normalized to 1)
    };

    explicit BiquadFilter(const Coeffs& coeffs);

    // Standard filter types
    static BiquadFilter lowpass(float freq, float q, float sample_rate);
    static BiquadFilter highpass(float freq, float q, float sample_rate);
    static BiquadFilter bandpass(float freq, float q, float sample_rate);
    static BiquadFilter notch(float freq, float q, float sample_rate);

    Sample process(Sample in);
    Samples process(SampleSpan in);
    void reset();

private:
    Coeffs coeffs_;
    float z1_ = 0, z2_ = 0;  // State
};

/**
 * Resampler
 *
 * High-quality sample rate conversion.
 */
class Resampler {
public:
    Resampler(uint32_t input_rate, uint32_t output_rate);
    ~Resampler();

    // Resample a block
    Samples process(SampleSpan in);

    // Get expected output size for given input size
    size_t outputSize(size_t input_size) const;

    // Reset state
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * AGC (Automatic Gain Control)
 */
class AGC {
public:
    AGC(float target_level = 0.5f, float attack = 0.01f, float decay = 0.001f);

    Sample process(Sample in);
    Samples process(SampleSpan in);

    float getGain() const { return gain_; }
    void reset();

private:
    float target_;
    float attack_;
    float decay_;
    float gain_ = 1.0f;
};

/**
 * Hilbert Transform (for SSB processing)
 */
class HilbertTransform {
public:
    explicit HilbertTransform(size_t taps = 65);

    // Returns analytic signal (I + jQ)
    std::vector<Complex> process(SampleSpan in);

private:
    std::vector<Sample> coeffs_;
    std::vector<Sample> delay_line_;
    size_t delay_idx_ = 0;
    size_t delay_samples_;
};

/**
 * NCO (Numerically Controlled Oscillator)
 */
class NCO {
public:
    NCO(float frequency, float sample_rate);

    // Get next sample (complex exponential)
    Complex next();

    // Mix signal up/down in frequency
    std::vector<Complex> mix(SampleSpan in);
    std::vector<Complex> mix(const std::vector<Complex>& in);

    // Adjust frequency
    void setFrequency(float freq);

    // Reset phase
    void reset();

private:
    float phase_ = 0;
    float phase_inc_;
    float sample_rate_;
};

// Utility functions
namespace dsp {

// Compute RMS level
float rms(SampleSpan samples);

// Compute peak level
float peak(SampleSpan samples);

// Normalize to target peak level
Samples normalize(SampleSpan samples, float target = 1.0f);

// Apply window function
enum class Window { Hann, Hamming, Blackman, Kaiser };
Samples applyWindow(SampleSpan samples, Window type);
std::vector<Sample> createWindow(size_t size, Window type);

// Convert between linear and dB
inline float toDb(float linear) { return 20.0f * std::log10(linear + 1e-10f); }
inline float fromDb(float db) { return std::pow(10.0f, db / 20.0f); }

} // namespace dsp

} // namespace ultra
