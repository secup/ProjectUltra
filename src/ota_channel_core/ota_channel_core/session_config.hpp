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
    // Every receiving radio's IF filter passband (Hz); high <= low = none.
    float rx_filter_low_hz = 0.0f;
    float rx_filter_high_hz = 0.0f;
    // Every receiving station's audio level (dB, 0 = the sender's transmit level).
    float rx_gain_db = 0.0f;
    // The path's SNR over time, both directions (empty = default_snr_db for good). The noise
    // floor stays at default_snr_db's level; set default_snr_db to the profile's maximum.
    std::vector<SnrProfilePoint> snr_profile;
    // Every station is a half-duplex radio: its receiver is deaf while its own transmitter is
    // keyed. Stations stream their audio continuously (zeros when idle), so the PTT is read from
    // the content the way the modem keys it: down ptt_lead_ms before its first non-zero sample
    // (the TX lead-in silence), up ptt_tail_ms after its last (the TX tail), then tr_recovery_ms
    // more of deafness (T/R relay + receiver recovery; rigs 5-30 ms, IC-7300 ~15, FT-891 ~20).
    // Defaults = ModemEngine's TX guards (kDefaultTxLeadInMs / kDefaultTxTailMs). The medium still
    // carries every signal; only the keyed station's receive is blanked. false = the full-duplex
    // stations the GUI simulator had from 2026-05-30 (client-side radio model retired) to
    // 2026-11-11.
    bool half_duplex_radios = true;
    float ptt_lead_ms = 150.0f;
    float ptt_tail_ms = 50.0f;
    float tr_recovery_ms = 20.0f;

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
        c.rx_filter_low_hz = rx_filter_low_hz;
        c.rx_filter_high_hz = rx_filter_high_hz;
        c.rx_gain_db = rx_gain_db;
        c.snr_profile = snr_profile;
        return c;
    }
};

}  // namespace ultra::ota_channel_core
