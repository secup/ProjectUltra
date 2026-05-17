#include "ota_channel_core/channel.hpp"
#include "ota_channel_core/mixer.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::SampleIndexedMixer;
using ultra::ota_channel_core::SimulatedChannel;

namespace {

void assertEqual(const std::vector<float>& a, const std::vector<float>& b) {
    assert(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        assert(std::abs(a[i] - b[i]) <= 1.0e-7f);
    }
}

}  // namespace

int main() {
    {
        SampleIndexedMixer mixer;
        const std::vector<float> tx{0.25f, -0.5f, 0.75f, 1.0f};
        mixer.submit("alice", 0, tx);
        assertEqual(mixer.mixForReceiver("bob", 0, tx.size()), tx);
    }

    {
        SampleIndexedMixer mixer;
        mixer.submit("alice", 0, std::vector<float>{1.0f, 2.0f, 3.0f});
        mixer.submit("carol", 0, std::vector<float>{0.5f, 1.5f, 2.5f});
        assertEqual(mixer.mixForReceiver("bob", 0, 3),
                    std::vector<float>{1.5f, 3.5f, 5.5f});
        assertEqual(mixer.mixForReceiver("alice", 0, 3),
                    std::vector<float>{0.5f, 1.5f, 2.5f});
    }

    {
        SampleIndexedMixer mixer;
        mixer.submit("alice", 4, std::vector<float>{4.0f, 5.0f});
        mixer.submit("alice", 0, std::vector<float>{0.0f, 1.0f, 2.0f, 3.0f});
        assertEqual(mixer.mixForReceiver("bob", 0, 6),
                    std::vector<float>{0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f});
    }

    {
        SimulatedChannel channel;
        channel.configure(80.0f, ChannelType::PASSTHROUGH);
        const std::vector<float> tx{0.1f, 0.2f, -0.3f, 0.4f};
        channel.transmitFromA(tx);
        assertEqual(channel.receiveForB(tx.size()), tx);
    }

    std::cout << "channel core mixer deterministic\n";
    return 0;
}
