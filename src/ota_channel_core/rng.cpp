#include "ota_channel_core/rng.hpp"

#include <cmath>
#include <limits>

namespace ultra::ota_channel_core {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ull;
constexpr uint64_t kFnvPrime = 1099511628211ull;

}  // namespace

RngStream::RngStream(uint32_t seed)
    : engine_(seed) {}

uint32_t RngStream::nextU32() {
    return engine_();
}

float RngStream::uniformOpen01() {
    constexpr double scale = 1.0 / 4294967296.0;
    return static_cast<float>((static_cast<double>(nextU32()) + 0.5) * scale);
}

float RngStream::normal() {
    if (has_spare_) {
        has_spare_ = false;
        return spare_;
    }

    const double u1 = std::max(static_cast<double>(uniformOpen01()),
                               std::numeric_limits<double>::min());
    const double u2 = static_cast<double>(uniformOpen01());
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double theta = 6.28318530717958647692 * u2;
    spare_ = static_cast<float>(r * std::sin(theta));
    has_spare_ = true;
    return static_cast<float>(r * std::cos(theta));
}

RngRoot::RngRoot(uint64_t master_seed)
    : master_seed_(master_seed) {}

RngStream RngRoot::stream(std::string_view name, uint64_t index) const {
    return RngStream(childSeed(name, index));
}

uint32_t RngRoot::childSeed(std::string_view name, uint64_t index) const {
    const uint64_t mixed = splitmix64(master_seed_ ^ stableHash(name) ^
                                     splitmix64(index + 0x9e3779b97f4a7c15ull));
    return static_cast<uint32_t>(mixed ^ (mixed >> 32));
}

uint64_t stableHash(std::string_view text) {
    uint64_t hash = kFnvOffset;
    for (unsigned char c : text) {
        hash ^= c;
        hash *= kFnvPrime;
    }
    return hash;
}

uint64_t splitmix64(uint64_t value) {
    value += 0x9e3779b97f4a7c15ull;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ull;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebull;
    return value ^ (value >> 31);
}

}  // namespace ultra::ota_channel_core
