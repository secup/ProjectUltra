#pragma once

#include "ota_channel_core/models.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ultra::ota_channel_core {

struct SessionConfig {
    std::string session_id;
    std::string display_name;
    ChannelType default_channel_model = ChannelType::PASSTHROUGH;
    float default_snr_db = 20.0f;
    uint64_t seed = 42;
    size_t station_cap = 16;
    bool is_lobby = false;
    uint32_t sample_rate = kDefaultSampleRate;
    std::shared_ptr<const std::vector<float>> real_hf_loop_noise;

    ChannelConfig channelConfig() const {
        return {.type = default_channel_model,
                .snr_db = default_snr_db,
                .seed = seed,
                .sample_rate = sample_rate,
                .real_hf_loop_noise = real_hf_loop_noise};
    }
};

}  // namespace ultra::ota_channel_core
