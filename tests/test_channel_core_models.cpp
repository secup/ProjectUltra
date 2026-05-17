#include "ota_channel_core/models.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using ultra::ota_channel_core::AWGNChannelModel;
using ultra::ota_channel_core::ChannelConfig;
using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::PassthroughChannelModel;
using ultra::ota_channel_core::RngRoot;
using ultra::ota_channel_core::WattersonChannel;
using ultra::ota_channel_core::createChannelModel;

namespace {

void assertNear(const std::vector<float>& a,
                const std::vector<float>& b,
                float eps = 1.0e-6f) {
    assert(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        assert(std::abs(a[i] - b[i]) <= eps);
    }
}

std::vector<float> runModel(ChannelType type, uint64_t seed) {
    const std::vector<float> input{0.0f, 0.1f, -0.2f, 0.3f, -0.4f, 0.0f, 0.25f, -0.15f};
    RngRoot root(seed);
    auto model = createChannelModel(
        ChannelConfig{.type = type, .snr_db = 18.0f, .seed = seed},
        root,
        "model:test");
    return model->process(input);
}

void checkRepeat(ChannelType type) {
    const auto first = runModel(type, 0xfeedbeefu);
    const auto second = runModel(type, 0xfeedbeefu);
    assertNear(first, second, 0.0f);
}

void checkDelaySpread(ChannelType type, size_t expected_delay) {
    auto cfg = ultra::ota_channel_core::configForWatterson(type, 80.0f);
    cfg.noise_enabled = false;
    cfg.fading_enabled = false;
    WattersonChannel channel(cfg, 123);

    std::vector<float> impulse(expected_delay + 8, 0.0f);
    impulse[0] = 1.0f;
    const auto out = channel.process(impulse);

    assert(std::abs(out[0] - cfg.path1_gain) <= 1.0e-6f);
    assert(std::abs(out[expected_delay] - cfg.path2_gain) <= 1.0e-6f);
}

}  // namespace

int main() {
    {
        PassthroughChannelModel model;
        const std::vector<float> input{0.0f, 1.0f, -0.5f, 0.25f};
        assertNear(model.process(input), input, 0.0f);
    }

    {
        RngRoot root(0xabcdefu);
        AWGNChannelModel a(15.0f, root.stream("awgn"));
        AWGNChannelModel b(15.0f, root.stream("awgn"));
        const std::vector<float> zeros(16, 0.0f);
        assertNear(a.process(zeros), b.process(zeros), 0.0f);
    }

    checkRepeat(ChannelType::AWGN);
    checkRepeat(ChannelType::GOOD);
    checkRepeat(ChannelType::MODERATE);
    checkRepeat(ChannelType::POOR);

    checkDelaySpread(ChannelType::GOOD, 24);
    checkDelaySpread(ChannelType::MODERATE, 48);
    checkDelaySpread(ChannelType::POOR, 96);

    const auto awgn = runModel(ChannelType::AWGN, 0x1234u);
    const std::vector<float> expected_awgn{
        -0.047623415f, 0.101872496f, -0.267377317f, 0.318847477f,
        -0.318579853f, -0.035867874f, 0.214578167f, -0.121521235f};
    assertNear(awgn, expected_awgn, 1.0e-6f);

    std::cout << "channel core models deterministic\n";
    return 0;
}
