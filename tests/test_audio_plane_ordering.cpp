#include "ota_simulator_service/audio_plane.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using ultra::ota_simulator_service::OrderedAudioQueue;

namespace {

void assertVectorNear(const std::vector<float>& actual,
                      const std::vector<float>& expected) {
    assert(actual.size() == expected.size());
    for (size_t i = 0; i < actual.size(); ++i) {
        assert(std::abs(actual[i] - expected[i]) < 1.0e-6f);
    }
}

}  // namespace

int main() {
    OrderedAudioQueue queue;
    const std::vector<float> late_block{4.0f, 5.0f};
    const std::vector<float> first_block{0.0f, 1.0f, 2.0f, 3.0f};
    assert(queue.push(4, late_block));
    assert(queue.drainReady().empty());

    assert(queue.push(0, first_block));
    auto ready = queue.drainReady();
    assert(ready.size() == 2);
    assert(ready[0].start_sample == 0);
    assertVectorNear(ready[0].samples, first_block);
    assert(ready[1].start_sample == 4);
    assertVectorNear(ready[1].samples, late_block);
    assert(queue.nextSample() == 6);

    assert(!queue.push(0, first_block));
    assert(!queue.push(4, late_block));

    const std::vector<float> future{10.0f, 11.0f};
    assert(queue.push(10, future));
    const auto window = queue.readWindow(6, 6);
    assertVectorNear(window, {0.0f, 0.0f, 0.0f, 0.0f, 10.0f, 11.0f});
    assert(queue.nextSample() == 12);

    std::cout << "audio plane queue reorders, drops late duplicates, fills gaps\n";
    return 0;
}
