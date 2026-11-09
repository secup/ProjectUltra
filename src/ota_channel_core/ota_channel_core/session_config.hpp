#pragma once

#include "ota_channel_core/models.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string_view>
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
    // Per-RECEIVER carrier frequency offset (station id -> Hz): everything that station hears
    // is shifted by this much (sender oscillator minus receiver oscillator). For two
    // transceivers whose oscillators differ by D: {B: +D, A: -D}. Absent = 0 Hz.
    std::map<std::string, float> rx_cfo_hz;
    // Per-RECEIVER sample-clock offset (station id -> ppm): sender soundcard clock minus the
    // receiver's. For two stations whose clocks differ by D ppm: {B: +D, A: -D}. Absent = 0.
    std::map<std::string, float> rx_clock_ppm;
    // The sending radios' ALC/PA (one radio model for every sender; disabled = none).
    TxPaConfig tx_pa;

    ChannelConfig channelConfig() const {
        return {.type = default_channel_model,
                .snr_db = default_snr_db,
                .seed = seed,
                .sample_rate = sample_rate,
                .real_hf_loop_noise = real_hf_loop_noise};
    }

    ChannelConfig channelConfigFor(std::string_view receiver_id) const {
        ChannelConfig c = channelConfig();
        const auto it = rx_cfo_hz.find(std::string(receiver_id));
        if (it != rx_cfo_hz.end()) c.cfo_hz = it->second;
        const auto ck = rx_clock_ppm.find(std::string(receiver_id));
        if (ck != rx_clock_ppm.end()) c.clock_ppm = ck->second;
        c.tx_pa = tx_pa;
        return c;
    }
};

}  // namespace ultra::ota_channel_core
