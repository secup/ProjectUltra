#pragma once

// WaveformFactory - Factory for creating IWaveform instances
//
// Centralizes waveform creation and provides:
// - Creation by WaveformMode enum
// - List of available/supported modes
// - Mode recommendation based on SNR
// - Mode validation

#include "waveform_interface.hpp"
#include "protocol/frame_v2.hpp"
#include <memory>
#include <vector>

namespace ultra {

class WaveformFactory {
public:
    // Create a waveform for the given mode
    // Returns nullptr if mode is not supported
    static WaveformPtr create(protocol::WaveformMode mode);

    // Create with specific configuration
    static WaveformPtr create(protocol::WaveformMode mode, const ModemConfig& config);

    // Create MC-DPSK with specific carrier count
    static WaveformPtr createMCDPSK(int num_carriers);

    // Get list of all available waveform modes
    static std::vector<protocol::WaveformMode> getAvailableModes();

    // Check if a mode is supported
    static bool isSupported(protocol::WaveformMode mode);

    // Get mode name as string
    static std::string getModeName(protocol::WaveformMode mode);

    // ========================================================================
    // MODE SELECTION
    // ========================================================================

    // Recommend a waveform mode based on measured SNR
    // Uses conservative thresholds calibrated for real HF channels
    static protocol::WaveformMode recommendMode(float snr_db);

    // Recommend modulation and code rate for given SNR
    static void recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate);

    // Get minimum SNR for a mode to work reliably
    static float getMinSNR(protocol::WaveformMode mode);

    // Get maximum throughput for a mode (at highest code rate)
    static float getMaxThroughput(protocol::WaveformMode mode);

    // ========================================================================
    // MC-DPSK CARRIER SELECTION
    // ========================================================================

    // Get recommended carrier count for MC-DPSK based on SNR
    // More carriers = higher throughput but need better SNR
    static int recommendMCDPSKCarriers(float snr_db);
};

} // namespace ultra
