#include "ota_channel_core/rng.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

using ultra::ota_channel_core::RngRoot;

namespace {

bool near(float a, float b, float eps = 1.0e-6f) {
    return std::abs(a - b) <= eps;
}

}  // namespace

int main() {
    RngRoot root(0x123456789abcdef0ull);

    auto a1 = root.stream("session:alpha:rx");
    auto a2 = root.stream("session:alpha:rx");
    auto b = root.stream("session:bravo:rx");

    assert(root.childSeed("session:alpha:rx") == root.childSeed("session:alpha:rx"));
    assert(root.childSeed("session:alpha:rx") != root.childSeed("session:bravo:rx"));

    for (int i = 0; i < 32; ++i) {
        assert(a1.nextU32() == a2.nextU32());
    }

    bool any_different = false;
    auto a3 = root.stream("session:alpha:rx");
    for (int i = 0; i < 32; ++i) {
        if (a3.nextU32() != b.nextU32()) {
            any_different = true;
            break;
        }
    }
    assert(any_different);

    auto n1 = root.stream("channel:awgn");
    auto n2 = root.stream("channel:awgn");
    for (int i = 0; i < 32; ++i) {
        assert(near(n1.normal(), n2.normal()));
    }

    const uint32_t expected_seed = root.childSeed("session:alpha:rx");
    assert(expected_seed == 0xf19a4a5au);

    std::cout << "channel core RNG deterministic\n";
    return 0;
}
