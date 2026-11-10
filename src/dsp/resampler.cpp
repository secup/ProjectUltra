#include "ultra/dsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ultra {

// Arbitrary-ratio resampler: windowed-sinc interpolation at the fractional input position of
// every output sample (2026-11-10). The previous zero-stuffing polyphase designed its 64-tap
// anti-alias filter for max(in, out) but ran it at in x up (12 -> 48 kHz happened to match;
// 44.1 -> 48 kHz ran it at 7 MHz and destroyed the audio), and its cost grew with the ratio's
// numerator. Here the cost is fixed per output sample at any ratio.
//
// y(t) = sum_n x[n] h(t - n),  t in input samples,
// h(u) = 2 fc sinc(2 fc u) w(u / half),  fc = 0.46 x min(1, out/in) cycles per input sample,
// w = Blackman, half = kZeroCrossings / (2 fc) input samples. Unity DC gain, linear phase, no
// delay (output sample k sits at input time k * in/out). The kernel is tabulated at kPhases
// points per input sample and interpolated linearly.
struct Resampler::Impl {
    static constexpr int kZeroCrossings = 16;
    static constexpr int kPhases = 512;

    uint32_t input_rate;
    uint32_t output_rate;
    double step;      // input samples per output sample
    int half;         // kernel half-width, input samples
    std::vector<float> table;  // h(u) at u = -half + j / kPhases, j = 0 .. 2 half kPhases

    std::vector<float> buf;    // pending input; buf[0] is absolute input sample buf_start
    int64_t buf_start = 0;
    int64_t input_total = 0;
    int64_t output_total = 0;
    double t = 0.0;            // absolute input position of the next output sample

    Impl(uint32_t in_rate, uint32_t out_rate)
        : input_rate(in_rate),
          output_rate(out_rate),
          step(static_cast<double>(in_rate) / static_cast<double>(out_rate)) {
        const double fc = 0.46 * std::min(1.0, static_cast<double>(out_rate) / in_rate);
        half = static_cast<int>(std::ceil(kZeroCrossings / (2.0 * fc)));
        const int n = 2 * half * kPhases + 1;
        table.resize(static_cast<size_t>(n));
        for (int j = 0; j < n; ++j) {
            const double u = -half + static_cast<double>(j) / kPhases;
            const double x = 2.0 * fc * u;
            const double sinc = (std::fabs(x) < 1e-12) ? 1.0 : std::sin(M_PI * x) / (M_PI * x);
            const double v = (u + half) / (2.0 * half);  // 0 .. 1 across the window
            const double w = 0.42 - 0.5 * std::cos(2.0 * M_PI * v) + 0.08 * std::cos(4.0 * M_PI * v);
            table[static_cast<size_t>(j)] = static_cast<float>(2.0 * fc * sinc * w);
        }
    }

    float kernel(double u) const {  // |u| <= half
        const double pos = (u + half) * kPhases;
        const int j = static_cast<int>(pos);
        if (j < 0 || j + 1 >= static_cast<int>(table.size())) return 0.0f;
        const float a = static_cast<float>(pos - j);
        return table[static_cast<size_t>(j)] * (1.0f - a) + table[static_cast<size_t>(j + 1)] * a;
    }

    float sampleAt(int64_t n) const {  // absolute input index; zero outside what was fed
        if (n < buf_start || n >= buf_start + static_cast<int64_t>(buf.size())) return 0.0f;
        return buf[static_cast<size_t>(n - buf_start)];
    }

    // Emit every output whose kernel support is fed (or, when finishing, every output the
    // input length owes).
    void emit(Samples& out, bool finishing) {
        const int64_t owed = static_cast<int64_t>(
            std::ceil(static_cast<double>(input_total) * output_rate / input_rate));
        while (true) {
            const int64_t center = static_cast<int64_t>(std::floor(t));
            if (!finishing && center + half >= buf_start + static_cast<int64_t>(buf.size())) break;
            if (finishing && output_total >= owed) break;
            const double frac = t - static_cast<double>(center);
            double acc = 0.0;
            for (int k = -half + 1; k <= half; ++k) {
                acc += static_cast<double>(sampleAt(center + k)) * kernel(frac - k);
            }
            out.push_back(static_cast<Sample>(acc));
            ++output_total;
            t += step;
        }
        // Drop input no later output can reach.
        const int64_t keep_from = static_cast<int64_t>(std::floor(t)) - half;
        if (keep_from > buf_start) {
            const int64_t drop =
                std::min<int64_t>(keep_from - buf_start, static_cast<int64_t>(buf.size()));
            buf.erase(buf.begin(), buf.begin() + drop);
            buf_start += drop;
        }
    }
};

Resampler::Resampler(uint32_t input_rate, uint32_t output_rate)
    : impl_(std::make_unique<Impl>(input_rate, output_rate)) {}

Resampler::~Resampler() = default;

size_t Resampler::outputSize(size_t input_size) const {
    return static_cast<size_t>(std::ceil(static_cast<double>(input_size) * impl_->output_rate /
                                         impl_->input_rate));
}

Samples Resampler::process(SampleSpan in) {
    if (impl_->input_rate == impl_->output_rate) {
        return Samples(in.begin(), in.end());
    }
    impl_->buf.insert(impl_->buf.end(), in.begin(), in.end());
    impl_->input_total += static_cast<int64_t>(in.size());
    Samples out;
    out.reserve(outputSize(in.size()) + 1);
    impl_->emit(out, /*finishing=*/false);
    return out;
}

Samples Resampler::flush() {
    Samples out;
    if (impl_->input_rate == impl_->output_rate) return out;
    impl_->emit(out, /*finishing=*/true);
    return out;
}

void Resampler::reset() {
    impl_->buf.clear();
    impl_->buf_start = 0;
    impl_->input_total = 0;
    impl_->output_total = 0;
    impl_->t = 0.0;
}

} // namespace ultra
