#pragma once

#include "types.hpp"
#include <memory>
#include <vector>
#include <functional>
#include <complex>

namespace ultra {

// Forward declarations
class OFDMModulator;
class OFDMDemodulator;
class OTFSModulator;
class OTFSDemodulator;

/**
 * Modulation Mode for adaptive selection
 *
 * Based on ITU-R F.1487 channel characterization:
 *   - OFDM: Best for Moderate channels (per-symbol pilot tracking)
 *   - OTFS_EQ: Best for Good/stable channels (preamble-based TF equalization)
 *   - OTFS_RAW: Best for Poor/severe fading (DD diversity, no stale estimates)
 */
enum class ModulationMode : uint8_t {
    OFDM = 0,       // Standard OFDM with per-symbol pilots
    OTFS_EQ = 1,    // OTFS with TF equalization (stable channels)
    OTFS_RAW = 2,   // OTFS without TF equalization (fading channels)
    AUTO = 3,       // Automatic selection based on channel estimation
};

/**
 * Channel characteristics estimated from preamble
 */
struct PreambleChannelEstimate {
    float delay_spread_ms = 0.0f;    // RMS delay spread in ms
    float doppler_spread_hz = 0.0f;  // Estimated Doppler spread in Hz
    float snr_db = 0.0f;             // Estimated SNR in dB
    float coherence_time_ms = 0.0f;  // Channel coherence time in ms

    // Classification thresholds (from ITU-R F.1487)
    bool isGoodChannel() const {
        return delay_spread_ms < 0.75f && doppler_spread_hz < 0.3f;
    }

    bool isModerateChannel() const {
        return delay_spread_ms >= 0.75f && delay_spread_ms < 1.5f &&
               doppler_spread_hz >= 0.3f && doppler_spread_hz < 2.0f;
    }

    bool isPoorChannel() const {
        return delay_spread_ms >= 1.5f || doppler_spread_hz >= 2.0f;
    }

    bool isFlutterChannel() const {
        return doppler_spread_hz >= 5.0f;
    }

    // Get recommended mode based on channel estimate
    ModulationMode getRecommendedMode() const {
        if (isFlutterChannel()) {
            // Flutter: Nothing works well, OFDM is most robust fallback
            return ModulationMode::OFDM;
        } else if (isPoorChannel()) {
            // Poor: High delay/Doppler - OTFS diversity without stale estimates
            return ModulationMode::OTFS_RAW;
        } else if (isModerateChannel()) {
            // Moderate: OFDM wins due to per-symbol pilot tracking
            return ModulationMode::OFDM;
        } else {
            // Good: Stable channel - OTFS with TF equalization
            return ModulationMode::OTFS_EQ;
        }
    }

    const char* getChannelConditionName() const {
        if (isFlutterChannel()) return "Flutter";
        if (isPoorChannel()) return "Poor";
        if (isModerateChannel()) return "Moderate";
        if (isGoodChannel()) return "Good";
        return "Unknown";
    }
};

/**
 * Preamble-based Channel Characterizer
 *
 * Estimates delay spread and Doppler spread from preamble symbols.
 * Used by AdaptiveModem to select the best modulation mode.
 *
 * This is different from the per-symbol ChannelEstimator in ofdm.hpp -
 * that one tracks channel during the frame, this one characterizes it
 * from the preamble for mode selection.
 */
class ChannelCharacterizer {
public:
    struct Config {
        uint32_t fft_size = 512;
        uint32_t cp_length = 64;
        uint32_t sample_rate = 48000;
        float center_freq = 1500.0f;
        uint32_t num_subcarriers = 32;  // M for OTFS
        uint32_t preamble_symbols = 4;  // Number of preamble symbols
    };

    explicit ChannelCharacterizer(const Config& config);
    ~ChannelCharacterizer();

    /**
     * Characterize channel from preamble samples
     *
     * @param preamble_samples Raw audio samples containing preamble
     * @param num_samples Number of samples
     * @param known_sequence The known preamble sequence (Zadoff-Chu)
     * @return Channel estimate with delay spread, Doppler, SNR
     */
    PreambleChannelEstimate characterize(const float* preamble_samples, size_t num_samples,
                                         const std::vector<Complex>& known_sequence);

    /**
     * Estimate delay spread from frequency-domain channel response
     * Uses IFFT to get impulse response, then calculates RMS delay spread
     */
    float estimateDelaySpread(const std::vector<Complex>& H_freq);

    /**
     * Estimate Doppler spread from multiple channel snapshots
     * Compares H(f) across preamble symbols to measure variation rate
     */
    float estimateDopplerSpread(const std::vector<std::vector<Complex>>& H_snapshots,
                                 float symbol_duration_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Adaptive Modem
 *
 * Unified interface that automatically selects between OFDM and OTFS
 * based on estimated channel conditions.
 *
 * Usage:
 *   AdaptiveModem modem(config);
 *   modem.setMode(ModulationMode::AUTO);  // Enable adaptive selection
 *
 *   // TX side
 *   Samples preamble = modem.generatePreamble();
 *   Samples data = modem.modulate(encoded_data, modulation);
 *
 *   // RX side
 *   bool ready = modem.process(rx_samples);
 *   if (ready) {
 *       auto soft_bits = modem.getSoftBits();
 *       auto mode_used = modem.getActiveMode();  // Which mode was selected
 *   }
 */
class AdaptiveModem {
public:
    struct Config {
        // Common parameters
        uint32_t sample_rate = 48000;
        uint32_t fft_size = 512;
        float center_freq = 1500.0f;

        // OFDM parameters
        uint32_t num_carriers = 30;
        uint32_t pilot_spacing = 6;
        CyclicPrefixMode cp_mode = CyclicPrefixMode::MEDIUM;

        // OTFS parameters
        uint32_t otfs_M = 32;  // Delay bins
        uint32_t otfs_N = 16;  // Doppler bins
        uint32_t otfs_cp = 64;

        // Mode selection
        ModulationMode default_mode = ModulationMode::AUTO;

        // Adaptive equalizer (for OFDM)
        bool adaptive_eq_enabled = true;
        float lms_mu = 0.1f;
    };

    explicit AdaptiveModem(const Config& config);
    ~AdaptiveModem();

    // Mode control
    void setMode(ModulationMode mode);
    ModulationMode getMode() const;
    ModulationMode getActiveMode() const;  // Actual mode used for last frame

    // TX interface
    Samples generatePreamble();
    Samples modulate(const Bytes& data, Modulation mod);

    // RX interface
    bool process(SampleSpan samples);
    std::vector<float> getSoftBits();
    void reset();

    // Channel information
    PreambleChannelEstimate getLastChannelEstimate() const;
    bool isSynced() const;

    // Callback for mode changes (for UI/logging)
    using ModeChangeCallback = std::function<void(ModulationMode old_mode, ModulationMode new_mode,
                                                   const PreambleChannelEstimate& estimate)>;
    void setModeChangeCallback(ModeChangeCallback callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Mode selection helper
 *
 * Given channel characteristics, returns the recommended mode.
 * This codifies the empirical results from ITU-R F.1487 benchmarking:
 *   - Good (0.5ms, 0.1Hz): OTFS-EQ wins (90% vs OFDM 66%)
 *   - Moderate (1.0ms, 0.5Hz): OFDM wins (82% vs OTFS 42%)
 *   - Poor (2.0ms, 1.0Hz): OTFS-RAW wins (20% vs OFDM 10%)
 *   - Flutter (0.5ms, 10Hz): All struggle, OFDM fallback
 */
ModulationMode selectMode(float delay_spread_ms, float doppler_spread_hz);

const char* modeToString(ModulationMode mode);

} // namespace ultra
