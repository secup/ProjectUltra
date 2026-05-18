#include "ota_channel_core/models.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace channel = ultra::ota_channel_core;

namespace {

void assertNear(float actual, float expected, float eps = 1.0e-6f) {
    assert(std::abs(actual - expected) <= eps);
}

void assertNear(const std::vector<float>& actual,
                const std::vector<float>& expected,
                float eps = 1.0e-6f) {
    assert(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        assertNear(actual[i], expected[i], eps);
    }
}

float rms(const std::vector<float>& samples) {
    double sum = 0.0;
    for (float sample : samples) {
        sum += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return static_cast<float>(std::sqrt(sum / static_cast<double>(samples.size())));
}

}  // namespace

int main() {
    const float high = std::sqrt(1.75f);
    const std::vector<float> unit_rms_loop{0.5f, -0.5f, high, -high};

    {
        const float snr_db = 10.0f;
        const float scale = channel::modemReferenceNoiseStddev(snr_db);
        channel::RealHfLoopChannelModel model(snr_db, unit_rms_loop, 3);
        const auto output = model.process(std::vector<float>(4, 0.0f));

        assertNear(rms(output), scale);
        assertNear(output, {
            -high * scale,
            0.5f * scale,
            -0.5f * scale,
            high * scale,
        });

        const float new_scale = channel::modemReferenceNoiseStddev(20.0f);
        model.setSNR(20.0f);
        assertNear(model.process(std::vector<float>(2, 0.0f)), {
            -high * new_scale,
            0.5f * new_scale,
        });
    }

    {
        const float scale = channel::modemReferenceNoiseStddev(12.0f);
        channel::RealHfLoopChannelModel model(12.0f, unit_rms_loop, 2);
        assertNear(model.process(std::vector<float>(6, 0.0f)), {
            high * scale,
            -high * scale,
            0.5f * scale,
            -0.5f * scale,
            high * scale,
            -high * scale,
        });
    }

    {
        channel::RealHfLoopChannelModel empty(12.0f, std::vector<float>{});
        const std::vector<float> input{0.25f, -0.75f, 0.0f};
        assertNear(empty.process(input), input, 0.0f);
    }

    {
        const auto parsed = channel::parseChannelType("real_hf_loop");
        assert(parsed && *parsed == channel::ChannelType::REAL_HF_LOOP);
        assert(std::string(channel::channelTypeName(channel::ChannelType::REAL_HF_LOOP)) ==
               "real_hf_loop");
    }

    std::cout << "real_hf_loop channel scaling and wrap behavior verified\n";
    return 0;
}
