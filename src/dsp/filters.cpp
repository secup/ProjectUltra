#include "ultra/dsp.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
#include <numeric>

// M_PI may not be defined on MSVC even with _USE_MATH_DEFINES if cmath was included earlier
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ultra {

// ============ FIR Filter ============

FIRFilter::FIRFilter(std::vector<Sample> coeffs)
    : coeffs_(std::move(coeffs))
    , delay_line_(coeffs_.size(), 0)
    , delay_idx_(0) {}

FIRFilter FIRFilter::lowpass(size_t taps, float cutoff, float sample_rate) {
    std::vector<Sample> coeffs(taps);
    float fc = cutoff / sample_rate;
    int M = (taps - 1) / 2;

    for (int n = 0; n < static_cast<int>(taps); ++n) {
        if (n == M) {
            coeffs[n] = 2.0f * fc;
        } else {
            float x = M_PI * (n - M);
            coeffs[n] = std::sin(2.0f * fc * x) / x;
        }
        // Apply Hamming window
        coeffs[n] *= 0.54f - 0.46f * std::cos(2.0f * M_PI * n / (taps - 1));
    }

    // Normalize
    float sum = std::accumulate(coeffs.begin(), coeffs.end(), 0.0f);
    for (auto& c : coeffs) c /= sum;

    return FIRFilter(std::move(coeffs));
}

FIRFilter FIRFilter::highpass(size_t taps, float cutoff, float sample_rate) {
    auto lp = lowpass(taps, cutoff, sample_rate);
    std::vector<Sample> coeffs = lp.coeffs_;

    // Spectral inversion
    int M = (taps - 1) / 2;
    for (size_t n = 0; n < taps; ++n) {
        coeffs[n] = -coeffs[n];
    }
    coeffs[M] += 1.0f;

    return FIRFilter(std::move(coeffs));
}

FIRFilter FIRFilter::bandpass(size_t taps, float low, float high, float sample_rate) {
    std::vector<Sample> coeffs(taps);
    float fc_low = low / sample_rate;
    float fc_high = high / sample_rate;
    int M = (taps - 1) / 2;

    for (int n = 0; n < static_cast<int>(taps); ++n) {
        if (n == M) {
            coeffs[n] = 2.0f * (fc_high - fc_low);
        } else {
            float x = M_PI * (n - M);
            coeffs[n] = (std::sin(2.0f * fc_high * x) - std::sin(2.0f * fc_low * x)) / x;
        }
        // Apply Blackman window for better stopband
        float w = 2.0f * M_PI * n / (taps - 1);
        coeffs[n] *= 0.42f - 0.5f * std::cos(w) + 0.08f * std::cos(2.0f * w);
    }

    return FIRFilter(std::move(coeffs));
}

Sample FIRFilter::process(Sample in) {
    delay_line_[delay_idx_] = in;

    Sample out = 0;
    size_t j = delay_idx_;
    for (size_t i = 0; i < coeffs_.size(); ++i) {
        out += coeffs_[i] * delay_line_[j];
        if (j == 0) j = coeffs_.size();
        --j;
    }

    delay_idx_ = (delay_idx_ + 1) % coeffs_.size();
    return out;
}

Samples FIRFilter::process(SampleSpan in) {
    Samples out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out[i] = process(in[i]);
    }
    return out;
}

void FIRFilter::reset() {
    std::fill(delay_line_.begin(), delay_line_.end(), 0);
    delay_idx_ = 0;
}

// ============ Biquad Filter ============

BiquadFilter::BiquadFilter(const Coeffs& coeffs) : coeffs_(coeffs) {}

BiquadFilter BiquadFilter::lowpass(float freq, float q, float sample_rate) {
    float w0 = 2.0f * M_PI * freq / sample_rate;
    float alpha = std::sin(w0) / (2.0f * q);
    float cos_w0 = std::cos(w0);

    float a0 = 1.0f + alpha;
    Coeffs c;
    c.b0 = ((1.0f - cos_w0) / 2.0f) / a0;
    c.b1 = (1.0f - cos_w0) / a0;
    c.b2 = c.b0;
    c.a1 = (-2.0f * cos_w0) / a0;
    c.a2 = (1.0f - alpha) / a0;

    return BiquadFilter(c);
}

BiquadFilter BiquadFilter::highpass(float freq, float q, float sample_rate) {
    float w0 = 2.0f * M_PI * freq / sample_rate;
    float alpha = std::sin(w0) / (2.0f * q);
    float cos_w0 = std::cos(w0);

    float a0 = 1.0f + alpha;
    Coeffs c;
    c.b0 = ((1.0f + cos_w0) / 2.0f) / a0;
    c.b1 = -(1.0f + cos_w0) / a0;
    c.b2 = c.b0;
    c.a1 = (-2.0f * cos_w0) / a0;
    c.a2 = (1.0f - alpha) / a0;

    return BiquadFilter(c);
}

BiquadFilter BiquadFilter::bandpass(float freq, float q, float sample_rate) {
    float w0 = 2.0f * M_PI * freq / sample_rate;
    float alpha = std::sin(w0) / (2.0f * q);
    float cos_w0 = std::cos(w0);

    float a0 = 1.0f + alpha;
    Coeffs c;
    c.b0 = alpha / a0;
    c.b1 = 0;
    c.b2 = -alpha / a0;
    c.a1 = (-2.0f * cos_w0) / a0;
    c.a2 = (1.0f - alpha) / a0;

    return BiquadFilter(c);
}

BiquadFilter BiquadFilter::notch(float freq, float q, float sample_rate) {
    float w0 = 2.0f * M_PI * freq / sample_rate;
    float alpha = std::sin(w0) / (2.0f * q);
    float cos_w0 = std::cos(w0);

    float a0 = 1.0f + alpha;
    Coeffs c;
    c.b0 = 1.0f / a0;
    c.b1 = (-2.0f * cos_w0) / a0;
    c.b2 = 1.0f / a0;
    c.a1 = c.b1;
    c.a2 = (1.0f - alpha) / a0;

    return BiquadFilter(c);
}

Sample BiquadFilter::process(Sample in) {
    Sample out = coeffs_.b0 * in + z1_;
    z1_ = coeffs_.b1 * in - coeffs_.a1 * out + z2_;
    z2_ = coeffs_.b2 * in - coeffs_.a2 * out;
    return out;
}

Samples BiquadFilter::process(SampleSpan in) {
    Samples out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out[i] = process(in[i]);
    }
    return out;
}

void BiquadFilter::reset() {
    z1_ = z2_ = 0;
}

// ============ AGC ============

AGC::AGC(float target_level, float attack, float decay)
    : target_(target_level), attack_(attack), decay_(decay) {}

Sample AGC::process(Sample in) {
    float abs_in = std::abs(in);

    // Update gain
    if (abs_in * gain_ > target_) {
        gain_ -= attack_ * (abs_in * gain_ - target_);
    } else {
        gain_ += decay_ * (target_ - abs_in * gain_);
    }

    // Clamp gain to reasonable range
    gain_ = std::max(0.001f, std::min(gain_, 1000.0f));

    return in * gain_;
}

Samples AGC::process(SampleSpan in) {
    Samples out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out[i] = process(in[i]);
    }
    return out;
}

void AGC::reset() {
    gain_ = 1.0f;
}

// ============ NCO ============

NCO::NCO(float frequency, float sample_rate)
    : phase_inc_(2.0f * M_PI * frequency / sample_rate)
    , sample_rate_(sample_rate) {}

Complex NCO::next() {
    Complex out(std::cos(phase_), std::sin(phase_));
    phase_ += phase_inc_;
    if (phase_ > 2.0f * M_PI) phase_ -= 2.0f * M_PI;
    if (phase_ < 0) phase_ += 2.0f * M_PI;
    return out;
}

std::vector<Complex> NCO::mix(SampleSpan in) {
    std::vector<Complex> out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out[i] = in[i] * next();
    }
    return out;
}

std::vector<Complex> NCO::mix(const std::vector<Complex>& in) {
    std::vector<Complex> out(in.size());
    for (size_t i = 0; i < in.size(); ++i) {
        out[i] = in[i] * next();
    }
    return out;
}

void NCO::setFrequency(float freq) {
    phase_inc_ = 2.0f * M_PI * freq / sample_rate_;
}

void NCO::reset() {
    phase_ = 0;
}

// ============ Hilbert Transform ============

HilbertTransform::HilbertTransform(size_t taps) {
    // Must be odd
    if (taps % 2 == 0) ++taps;

    coeffs_.resize(taps);
    delay_samples_ = (taps - 1) / 2;

    // Generate Hilbert transformer coefficients
    int M = (taps - 1) / 2;
    for (int n = 0; n < static_cast<int>(taps); ++n) {
        int k = n - M;
        if (k == 0) {
            coeffs_[n] = 0;
        } else if (k % 2 != 0) {
            coeffs_[n] = 2.0f / (M_PI * k);
        } else {
            coeffs_[n] = 0;
        }
        // Apply Blackman window
        float w = 2.0f * M_PI * n / (taps - 1);
        coeffs_[n] *= 0.42f - 0.5f * std::cos(w) + 0.08f * std::cos(2.0f * w);
    }

    delay_line_.resize(taps, 0);
    delay_idx_ = 0;
}

std::vector<Complex> HilbertTransform::process(SampleSpan in) {
    std::vector<Complex> out(in.size());

    for (size_t i = 0; i < in.size(); ++i) {
        delay_line_[delay_idx_] = in[i];

        // Compute Hilbert (Q) output
        float q = 0;
        size_t j = delay_idx_;
        for (size_t k = 0; k < coeffs_.size(); ++k) {
            q += coeffs_[k] * delay_line_[j];
            if (j == 0) j = coeffs_.size();
            --j;
        }

        // I output is delayed input
        size_t delay_pos = (delay_idx_ + coeffs_.size() - delay_samples_) % coeffs_.size();
        float real = delay_line_[delay_pos];

        out[i] = Complex(real, q);
        delay_idx_ = (delay_idx_ + 1) % coeffs_.size();
    }

    return out;
}

// ============ Utility functions ============

namespace dsp {

float rms(SampleSpan samples) {
    if (samples.empty()) return 0;
    float sum_sq = 0;
    for (auto s : samples) sum_sq += s * s;
    return std::sqrt(sum_sq / samples.size());
}

float peak(SampleSpan samples) {
    float max_val = 0;
    for (auto s : samples) {
        float abs_s = std::abs(s);
        if (abs_s > max_val) max_val = abs_s;
    }
    return max_val;
}

Samples normalize(SampleSpan samples, float target) {
    float pk = peak(samples);
    if (pk < 1e-10f) return Samples(samples.begin(), samples.end());

    float scale = target / pk;
    Samples out(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        out[i] = samples[i] * scale;
    }
    return out;
}

std::vector<Sample> createWindow(size_t size, Window type) {
    std::vector<Sample> w(size);
    for (size_t n = 0; n < size; ++n) {
        float x = static_cast<float>(n) / (size - 1);
        switch (type) {
            case Window::Hann:
                w[n] = 0.5f * (1.0f - std::cos(2.0f * M_PI * x));
                break;
            case Window::Hamming:
                w[n] = 0.54f - 0.46f * std::cos(2.0f * M_PI * x);
                break;
            case Window::Blackman:
                w[n] = 0.42f - 0.5f * std::cos(2.0f * M_PI * x)
                       + 0.08f * std::cos(4.0f * M_PI * x);
                break;
            case Window::Kaiser:
                // Simplified Kaiser with beta=8
                // Full implementation would use Bessel function
                w[n] = 0.40243f - 0.49804f * std::cos(2.0f * M_PI * x)
                       + 0.09831f * std::cos(4.0f * M_PI * x);
                break;
        }
    }
    return w;
}

Samples applyWindow(SampleSpan samples, Window type) {
    auto w = createWindow(samples.size(), type);
    Samples out(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        out[i] = samples[i] * w[i];
    }
    return out;
}

} // namespace dsp

} // namespace ultra
