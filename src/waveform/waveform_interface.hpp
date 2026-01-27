#pragma once

// IWaveform - Abstract interface for all waveform types
//
// This interface enables plugin-style architecture where adding a new waveform
// (e.g., OTFS, AFDM) only requires implementing this interface.
//
// Design principles:
// 1. Waveform owns its sync method (ChirpSync, Schmidl-Cox, etc.)
// 2. Waveform handles both TX (modulation) and RX (demodulation)
// 3. Waveform exposes status for GUI display
// 4. CFO correction is handled internally by the waveform

#include "ultra/types.hpp"
#include "protocol/frame_v2.hpp"
#include <string>
#include <vector>
#include <memory>

namespace ultra {

// Types are defined in ultra/types.hpp - included above

// Capabilities exposed by a waveform for mode selection
struct WaveformCapabilities {
    bool supports_cfo_correction = false;   // Can correct carrier frequency offset
    bool supports_doppler_correction = false; // Can handle Doppler spread
    bool requires_pilots = false;           // Needs pilot symbols (coherent modes)
    bool supports_differential = true;      // Supports differential modulation
    float min_snr_db = 0.0f;               // Minimum SNR for reliable operation
    float max_snr_db = 30.0f;              // Maximum useful SNR (above this, use higher mode)
    float max_throughput_bps = 1000.0f;    // Maximum achievable throughput
    float preamble_duration_ms = 500.0f;   // Preamble length for this waveform
};

// Result from sync detection
struct SyncResult {
    bool detected = false;
    int start_sample = -1;          // Sample position where data starts
    float correlation = 0.0f;       // Peak correlation value (0-1)
    float cfo_hz = 0.0f;            // Estimated carrier frequency offset
    float snr_estimate = 0.0f;      // Estimated SNR from preamble
    bool has_training = false;      // True if preamble includes training sequence
};

// Abstract waveform interface
class IWaveform {
public:
    virtual ~IWaveform() = default;

    // ========================================================================
    // IDENTITY
    // ========================================================================

    // Human-readable name (e.g., "MC-DPSK", "OFDM-COX")
    virtual std::string getName() const = 0;

    // Protocol waveform mode enum value
    virtual protocol::WaveformMode getMode() const = 0;

    // Capabilities for mode selection logic
    virtual WaveformCapabilities getCapabilities() const = 0;

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    // Set modulation scheme (QPSK, 8PSK, 16QAM, etc.) and code rate
    virtual void configure(Modulation mod, CodeRate rate) = 0;

    // Apply carrier frequency offset correction (Hz)
    // Called after sync detection with estimated CFO
    virtual void setFrequencyOffset(float cfo_hz) = 0;

    // Set TX frequency offset for testing (simulates radio tuning error)
    // Must be called BEFORE generatePreamble() and modulate()
    virtual void setTxFrequencyOffset(float cfo_hz) = 0;

    // Get current configuration
    virtual Modulation getModulation() const = 0;
    virtual CodeRate getCodeRate() const = 0;
    virtual float getFrequencyOffset() const = 0;

    // ========================================================================
    // TX PATH
    // ========================================================================

    // Generate preamble (sync sequence + training if applicable)
    virtual Samples generatePreamble() = 0;

    // Modulate encoded data (LDPC-encoded bits packed as bytes)
    // Returns audio samples ready for transmission
    virtual Samples modulate(const Bytes& encoded_data) = 0;

    // ========================================================================
    // RX PATH
    // ========================================================================

    // Detect sync/preamble in sample buffer
    // Returns true if sync detected, fills result with details
    virtual bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.3f) = 0;

    // Process samples after sync detection
    // Returns true if a complete symbol/frame is ready
    // Call getSoftBits() to retrieve demodulated data
    virtual bool process(SampleSpan samples) = 0;

    // Get soft bits from last process() call
    // Returns LLR values for LDPC decoder
    virtual std::vector<float> getSoftBits() = 0;

    // Reset internal state (call between frames)
    virtual void reset() = 0;

    // ========================================================================
    // STATUS
    // ========================================================================

    // Is demodulator currently synced?
    virtual bool isSynced() const = 0;

    // Does demodulator have data ready?
    virtual bool hasData() const = 0;

    // Estimated SNR from current signal (dB)
    virtual float estimatedSNR() const = 0;

    // Estimated CFO from current signal (Hz)
    virtual float estimatedCFO() const = 0;

    // Get constellation symbols for GUI display
    virtual std::vector<std::complex<float>> getConstellationSymbols() const = 0;

    // ========================================================================
    // GUI DISPLAY
    // ========================================================================

    // Status string for GUI (e.g., "MC-DPSK 8 carriers @ 375 bps")
    virtual std::string getStatusString() const = 0;

    // Number of carriers (for multi-carrier modes)
    virtual int getCarrierCount() const = 0;

    // Calculate throughput for given code rate (bps)
    virtual float getThroughput(CodeRate rate) const = 0;

    // Get samples required for one complete symbol
    virtual int getSamplesPerSymbol() const = 0;

    // Get total preamble duration in samples
    virtual int getPreambleSamples() const = 0;

    // Get minimum samples needed AFTER sync detection for one complete frame
    // This includes training, reference, and data for at least one codeword
    // Used by RxPipeline to know when enough samples are available
    virtual int getMinSamplesForFrame() const = 0;
};

// Convenience alias
using WaveformPtr = std::unique_ptr<IWaveform>;

} // namespace ultra
