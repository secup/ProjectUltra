#include "adaptive_mode.hpp"
#include <cmath>

namespace ultra {
namespace gui {

AdaptiveModeController::AdaptiveModeController() {
    reset();
}

void AdaptiveModeController::reset() {
    // Start with balanced settings (64-QAM R3/4)
    current_mod_ = Modulation::QAM64;
    current_rate_ = CodeRate::R3_4;
    last_switch_snr_ = 0.0f;
    frames_at_current_ = 0;
}

void AdaptiveModeController::recommendMode(float snr_db, Modulation& mod, CodeRate& rate) {
    // Thresholds calibrated for pilot-based SNR measurement
    // Our pilot SNR is higher than channel SNR due to measurement methodology
    // These thresholds are based on empirical testing
    //
    // Pilot SNR thresholds (approximate mapping):
    //   Pilot > 38 dB: Excellent channel, use highest modes
    //   Pilot > 32 dB: Good channel
    //   Pilot > 28 dB: Moderate channel
    //   Pilot > 24 dB: Fair channel
    //   Pilot > 20 dB: Poor channel
    //   Pilot < 20 dB: Very poor, use most robust mode

    if (snr_db > 38.0f) {
        mod = Modulation::QAM64;
        rate = CodeRate::R5_6;
    } else if (snr_db > 34.0f) {
        mod = Modulation::QAM64;
        rate = CodeRate::R3_4;
    } else if (snr_db > 30.0f) {
        mod = Modulation::QAM16;
        rate = CodeRate::R3_4;
    } else if (snr_db > 26.0f) {
        mod = Modulation::QAM16;
        rate = CodeRate::R2_3;
    } else if (snr_db > 24.0f) {
        mod = Modulation::QPSK;
        rate = CodeRate::R2_3;
    } else if (snr_db > 22.0f) {
        mod = Modulation::QPSK;
        rate = CodeRate::R1_2;
    } else if (snr_db > 18.0f) {
        mod = Modulation::BPSK;
        rate = CodeRate::R1_2;
    } else {
        mod = Modulation::BPSK;
        rate = CodeRate::R1_4;
    }
}

bool AdaptiveModeController::update(float snr_db) {
    // Get recommended mode for current SNR
    Modulation rec_mod;
    CodeRate rec_rate;
    recommendMode(snr_db, rec_mod, rec_rate);

    // Check if mode change is recommended
    if (rec_mod == current_mod_ && rec_rate == current_rate_) {
        // Same mode - increment stability counter
        frames_at_current_++;
        return false;
    }

    // Mode change recommended - apply hysteresis
    float snr_change = std::abs(snr_db - last_switch_snr_);

    // Require minimum SNR change to switch
    if (snr_change < HYSTERESIS_DB && frames_at_current_ < MIN_FRAMES_BEFORE_SWITCH * 10) {
        // Not enough change or not stable enough at current mode
        frames_at_current_++;
        return false;
    }

    // Allow switch if:
    // 1. SNR has changed significantly (> hysteresis)
    // 2. OR we've been stable at current mode long enough
    if (snr_change >= HYSTERESIS_DB || frames_at_current_ >= MIN_FRAMES_BEFORE_SWITCH) {
        current_mod_ = rec_mod;
        current_rate_ = rec_rate;
        last_switch_snr_ = snr_db;
        frames_at_current_ = 0;
        return true;  // Mode changed
    }

    frames_at_current_++;
    return false;
}

const char* AdaptiveModeController::getModeString() const {
    // Return a string describing current mode
    switch (current_mod_) {
        case Modulation::BPSK:
            return current_rate_ == CodeRate::R1_4 ? "BPSK 1/4" : "BPSK 1/2";
        case Modulation::QPSK:
            return current_rate_ == CodeRate::R1_2 ? "QPSK 1/2" : "QPSK 2/3";
        case Modulation::QAM16:
            return current_rate_ == CodeRate::R2_3 ? "16QAM 2/3" : "16QAM 3/4";
        case Modulation::QAM64:
            return current_rate_ == CodeRate::R3_4 ? "64QAM 3/4" : "64QAM 5/6";
        default:
            return "Unknown";
    }
}

} // namespace gui
} // namespace ultra
