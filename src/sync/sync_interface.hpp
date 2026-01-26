#pragma once

// ISyncMethod - Abstract interface for synchronization methods
//
// Separates sync detection from waveform modulation/demodulation.
// Each sync method can be reused across multiple waveforms.
//
// Current implementations:
// - ChirpSync: Linear FM chirp (500ms), works at -10 dB, CFO-tolerant
// - SchmidlCoxSync: OFDM preamble, needs ~17 dB, fast acquisition
// - BarkerSync: Barker-13 code, works at -8 dB, short duration
// - ZadoffChuSync: OTFS preamble, good autocorrelation

#include "ultra/types.hpp"
#include <string>
#include <vector>

namespace ultra {
namespace sync {

// Result from sync detection (matches the one in waveform_interface.hpp for compatibility)
struct SyncResult {
    bool detected = false;
    int start_sample = -1;          // Sample position where sync was found
    float correlation = 0.0f;       // Peak correlation value (0-1)
    float cfo_hz = 0.0f;            // Estimated carrier frequency offset
    float snr_estimate = 0.0f;      // Estimated SNR from preamble (dB)
    float timing_confidence = 0.0f; // Confidence in timing estimate (0-1)
};

// Abstract sync method interface
class ISyncMethod {
public:
    virtual ~ISyncMethod() = default;

    // ========================================================================
    // IDENTITY
    // ========================================================================

    // Human-readable name (e.g., "Chirp", "Schmidl-Cox", "Barker-13")
    virtual std::string getName() const = 0;

    // Minimum SNR required for reliable detection (dB)
    virtual float getMinSNR() const = 0;

    // Duration of sync sequence in samples
    virtual int getSyncDurationSamples() const = 0;

    // ========================================================================
    // TX: PREAMBLE GENERATION
    // ========================================================================

    // Generate preamble/sync sequence for transmission
    virtual Samples generatePreamble() = 0;

    // ========================================================================
    // RX: SYNC DETECTION
    // ========================================================================

    // Detect sync sequence in sample buffer
    // threshold: detection threshold (typically 0.1 to 0.5)
    // Returns: result struct with detection details
    virtual SyncResult detect(SampleSpan samples, float threshold = 0.3f) = 0;

    // Estimate CFO from samples at given sync position
    // sync_position: sample index from detect()
    // Returns: estimated CFO in Hz
    virtual float estimateCFO(SampleSpan samples, int sync_position) = 0;

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    // Set detection threshold (for tuning)
    virtual void setThreshold(float threshold) = 0;

    // Get current threshold
    virtual float getThreshold() const = 0;

    // Reset internal state
    virtual void reset() = 0;
};

// Convenience alias
using SyncMethodPtr = std::unique_ptr<ISyncMethod>;

} // namespace sync
} // namespace ultra
