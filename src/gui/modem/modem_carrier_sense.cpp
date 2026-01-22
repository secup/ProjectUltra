// modem_carrier_sense.cpp - Carrier sense (Listen Before Talk) for ModemEngine

#include "modem_engine.hpp"
#include <cmath>
#include <algorithm>

namespace ultra {
namespace gui {

void ModemEngine::updateChannelEnergy(const std::vector<float>& samples) {
    if (samples.empty()) return;

    // Calculate RMS energy of samples
    float sum_sq = 0.0f;
    for (float s : samples) {
        sum_sq += s * s;
    }
    float rms = std::sqrt(sum_sq / samples.size());

    // Smooth the energy estimate (exponential moving average)
    float current = channel_energy_.load();
    float smoothed = ENERGY_SMOOTHING * rms + (1.0f - ENERGY_SMOOTHING) * current;
    channel_energy_.store(smoothed);
}

bool ModemEngine::isChannelBusy() const {
    // Channel is busy if energy is above threshold (someone is transmitting)
    // Note: We don't check isSynced() here because:
    // 1. Sync state persists after decode (would block response TX)
    // 2. Energy detection is the true carrier sense - if there's RF energy, channel is busy
    // 3. Protocol layer handles half-duplex timing separately
    return channel_energy_.load() > carrier_sense_threshold_;
}

float ModemEngine::getChannelEnergy() const {
    return channel_energy_.load();
}

void ModemEngine::setCarrierSenseThreshold(float threshold) {
    carrier_sense_threshold_ = std::max(0.0f, std::min(1.0f, threshold));
}

float ModemEngine::getCarrierSenseThreshold() const {
    return carrier_sense_threshold_;
}

bool ModemEngine::isTurnaroundActive() const {
    if (turnaround_delay_ms_ == 0) return false;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_complete_time_).count();
    return elapsed < turnaround_delay_ms_;
}

uint32_t ModemEngine::getTurnaroundRemaining() const {
    if (turnaround_delay_ms_ == 0) return 0;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_complete_time_).count();
    if (elapsed >= turnaround_delay_ms_) return 0;
    return static_cast<uint32_t>(turnaround_delay_ms_ - elapsed);
}

} // namespace gui
} // namespace ultra
