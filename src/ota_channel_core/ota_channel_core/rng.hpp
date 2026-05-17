#pragma once

#include <cstdint>
#include <random>
#include <string_view>

namespace ultra::ota_channel_core {

class RngStream {
public:
    explicit RngStream(uint32_t seed = 0);

    uint32_t nextU32();
    float uniformOpen01();
    float normal();

private:
    std::mt19937 engine_;
    bool has_spare_ = false;
    float spare_ = 0.0f;
};

class RngRoot {
public:
    explicit RngRoot(uint64_t master_seed = 0);

    uint64_t masterSeed() const { return master_seed_; }
    RngStream stream(std::string_view name, uint64_t index = 0) const;
    uint32_t childSeed(std::string_view name, uint64_t index = 0) const;

private:
    uint64_t master_seed_;
};

uint64_t stableHash(std::string_view text);
uint64_t splitmix64(uint64_t value);

}  // namespace ultra::ota_channel_core
