#include "ultra/dsp.hpp"
#include <cmath>

namespace ultra {

struct Resampler::Impl {
    uint32_t input_rate;
    uint32_t output_rate;
    uint32_t gcd;
    uint32_t up_factor;
    uint32_t down_factor;

    FIRFilter anti_alias;
    std::vector<Sample> upsample_buffer;
    size_t phase = 0;

    Impl(uint32_t in_rate, uint32_t out_rate)
        : input_rate(in_rate)
        , output_rate(out_rate)
        , anti_alias(FIRFilter::lowpass(64,
            std::min(in_rate, out_rate) * 0.45f,
            static_cast<float>(std::max(in_rate, out_rate))))
    {
        // Find GCD to simplify ratio
        uint32_t a = in_rate, b = out_rate;
        while (b != 0) {
            uint32_t t = b;
            b = a % b;
            a = t;
        }
        gcd = a;
        up_factor = out_rate / gcd;
        down_factor = in_rate / gcd;
    }
};

Resampler::Resampler(uint32_t input_rate, uint32_t output_rate)
    : impl_(std::make_unique<Impl>(input_rate, output_rate)) {}

Resampler::~Resampler() = default;

size_t Resampler::outputSize(size_t input_size) const {
    return (input_size * impl_->up_factor + impl_->down_factor - 1) / impl_->down_factor;
}

Samples Resampler::process(SampleSpan in) {
    if (impl_->input_rate == impl_->output_rate) {
        return Samples(in.begin(), in.end());
    }

    // Simple polyphase resampler
    // For HF modem, we typically go 48000 -> 8000 or vice versa

    Samples out;
    out.reserve(outputSize(in.size()));

    for (size_t i = 0; i < in.size(); ++i) {
        // Upsample: insert zeros
        for (uint32_t j = 0; j < impl_->up_factor; ++j) {
            Sample s = (j == 0) ? in[i] * impl_->up_factor : 0;
            s = impl_->anti_alias.process(s);

            // Downsample: keep every down_factor sample
            if (impl_->phase == 0) {
                out.push_back(s);
            }
            impl_->phase = (impl_->phase + 1) % impl_->down_factor;
        }
    }

    return out;
}

void Resampler::reset() {
    impl_->anti_alias.reset();
    impl_->phase = 0;
}

} // namespace ultra
