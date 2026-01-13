#pragma once

#include "ultra/types.hpp"

namespace ultra {
namespace gui {

/**
 * Adaptive Modulation Controller
 *
 * Automatically selects modulation and code rate based on measured SNR.
 * Uses hysteresis to prevent rapid switching at threshold boundaries.
 *
 * Note: Thresholds are calibrated for pilot-based SNR estimation,
 * which has an offset from theoretical channel SNR.
 */
class AdaptiveModeController {
public:
    AdaptiveModeController();

    /**
     * Update with current SNR measurement.
     * @param snr_db Pilot SNR in dB (from demodulator)
     * @return true if mode changed
     */
    bool update(float snr_db);

    // Get current recommended settings
    Modulation getModulation() const { return current_mod_; }
    CodeRate getCodeRate() const { return current_rate_; }

    // Reset to initial state
    void reset();

    // Get mode description string
    const char* getModeString() const;

private:
    // Current mode
    Modulation current_mod_;
    CodeRate current_rate_;

    // Hysteresis state
    float last_switch_snr_ = 0.0f;
    int frames_at_current_ = 0;

    // Configuration
    static constexpr float HYSTERESIS_DB = 2.0f;  // Don't switch unless SNR changes by this much
    static constexpr int MIN_FRAMES_BEFORE_SWITCH = 3;  // Stability requirement

    // Helper to recommend mode based on SNR
    void recommendMode(float snr_db, Modulation& mod, CodeRate& rate);
};

} // namespace gui
} // namespace ultra
