#pragma once

#include "ota_channel_core/models.hpp"

namespace ultra::sim {

using WattersonChannel = ultra::ota_channel_core::WattersonChannel;
using HFChannel = WattersonChannel;

namespace itu_r_f1487 = ultra::ota_channel_core::itu_r_f1487;

namespace ccir {

inline WattersonChannel::Config good(float snr_db = 25.0f) {
    auto cfg = ultra::ota_channel_core::itu_r_f1487::good(snr_db);
    return cfg;
}

inline WattersonChannel::Config moderate(float snr_db = 15.0f) {
    auto cfg = ultra::ota_channel_core::itu_r_f1487::moderate(snr_db);
    return cfg;
}

inline WattersonChannel::Config poor(float snr_db = 10.0f) {
    auto cfg = ultra::ota_channel_core::itu_r_f1487::poor(snr_db);
    return cfg;
}

inline WattersonChannel::Config flutter(float snr_db = 8.0f) {
    auto cfg = ultra::ota_channel_core::itu_r_f1487::flutter(snr_db);
    return cfg;
}

inline WattersonChannel::Config awgn(float snr_db = 15.0f) {
    auto cfg = ultra::ota_channel_core::itu_r_f1487::awgn(snr_db);
    return cfg;
}

}  // namespace ccir

}  // namespace ultra::sim
