#pragma once

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
// Forward declare HFChannel config (actual header is in src/sim/)
namespace ultra { namespace sim {
struct HFChannelConfig {
    float snr_db;
    bool multipath_enabled;
    float path1_delay_ms;
    float path1_gain;
    float path2_delay_ms;
    float path2_gain;
    bool fading_enabled;
    float doppler_spread_hz;
    float freq_offset_hz;
    uint32_t sample_rate;
};
}}

#include <vector>
#include <complex>
#include <memory>
#include <random>
#include <string>

namespace ultra {
namespace gui {

// Simulation results for display
struct SimulationResult {
    std::vector<std::complex<float>> tx_symbols;   // Transmitted constellation points
    std::vector<std::complex<float>> rx_symbols;   // Received (noisy) constellation points
    std::vector<uint8_t> tx_data;                  // Original data
    std::vector<uint8_t> rx_data;                  // Decoded data

    float snr_db = 0;
    float ber = 0;              // Bit error rate
    int bit_errors = 0;
    int total_bits = 0;
    bool decode_success = false;
    int ldpc_iterations = 0;
    float throughput_bps = 0;
};

// Channel condition presets
enum class ChannelPreset {
    AWGN_ONLY,      // Just noise, no multipath
    GOOD,           // Light multipath, low Doppler
    MODERATE,       // Typical HF
    POOR,           // Heavy multipath, high Doppler
    CUSTOM          // Use manual settings
};

class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine();

    // Run one frame through the simulation
    SimulationResult runFrame(const ModemConfig& config);

    // Settings
    void setChannelPreset(ChannelPreset preset);
    void setSNR(float snr_db);
    void setMultipathDelay(float ms);
    void setDopplerSpread(float hz);

    float getSNR() const { return snr_db_; }
    ChannelPreset getPreset() const { return preset_; }

    // Statistics
    uint64_t getTotalFrames() const { return total_frames_; }
    uint64_t getSuccessFrames() const { return success_frames_; }
    float getAverageBER() const;

    void resetStats();

private:
    // Channel settings
    ChannelPreset preset_ = ChannelPreset::MODERATE;
    float snr_db_ = 15.0f;
    float multipath_delay_ms_ = 2.0f;
    float doppler_hz_ = 1.0f;

    // Statistics
    uint64_t total_frames_ = 0;
    uint64_t success_frames_ = 0;
    uint64_t total_bit_errors_ = 0;
    uint64_t total_bits_ = 0;

    // Components
    std::unique_ptr<LDPCEncoder> encoder_;
    std::unique_ptr<LDPCDecoder> decoder_;
    std::mt19937 rng_{42};

    std::vector<std::complex<float>> extractSymbols(const Bytes& encoded, Modulation mod);
};

} // namespace gui
} // namespace ultra
