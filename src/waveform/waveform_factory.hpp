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
#include "psk/multi_carrier_dpsk.hpp"
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

    // Create MC-DPSK with a full PHY preset
    static WaveformPtr createMCDPSK(const MultiCarrierDPSKConfig& config);

    // Create narrowband MC-DPSK (4 carriers, 500 Hz band, narrowband chirp)
    static WaveformPtr createNarrowbandMCDPSK();
    // The session's MC-DPSK waveform on the narrowband geometry: carriers, symbol length and
    // chirp from mc_dpsk_presets::narrowband(), the constellation (bits per symbol) from the
    // session config, as createMCDPSK(session) takes it. Every modem-side narrowband waveform is
    // built this way so it carries the same constellation as its wideband counterpart.
    static WaveformPtr createNarrowbandMCDPSK(const MultiCarrierDPSKConfig& session);

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

    // NOTE: For modulation/rate selection, use protocol::recommendDataMode()
    // from waveform_selection.hpp instead

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
