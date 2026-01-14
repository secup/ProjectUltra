#pragma once

#include "types.hpp"
#include <memory>

namespace ultra {

// Forward declarations
class FFT;

/**
 * OFDM Modulator
 *
 * Converts data bits into OFDM audio waveform.
 *
 * Design for HF:
 * - Narrow subcarrier spacing (~47 Hz) for Doppler tolerance
 * - Long symbols with cyclic prefix for multipath
 * - Scattered pilots for channel tracking
 * - Supports BPSK through QAM64
 */
class OFDMModulator {
public:
    explicit OFDMModulator(const ModemConfig& config);
    ~OFDMModulator();

    // Modulate data bytes into audio samples
    Samples modulate(ByteSpan data, Modulation mod);

    // Generate a sync/preamble sequence
    Samples generatePreamble();

    // Generate channel probe signal
    Samples generateProbe();

    // Get samples per symbol (including CP)
    size_t samplesPerSymbol() const;

    // Get data bits per symbol at given modulation
    size_t bitsPerSymbol(Modulation mod) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * OFDM Demodulator
 *
 * Converts received audio back to data bits.
 * Includes channel estimation and equalization.
 */
class OFDMDemodulator {
public:
    explicit OFDMDemodulator(const ModemConfig& config);
    ~OFDMDemodulator();

    // Process incoming samples, returns true if frame ready
    bool process(SampleSpan samples);

    // Get demodulated data (call after process returns true)
    Bytes getData();

    // Get soft bits for FEC decoder (better than hard decisions)
    std::vector<float> getSoftBits();

    // Get current channel estimate
    ChannelQuality getChannelQuality() const;

    // Get estimated SNR in dB (from pilot measurements)
    float getEstimatedSNR() const;

    // Get estimated frequency offset in Hz (from pilot phase tracking)
    float getFrequencyOffset() const;

    // Get equalized symbols for constellation display
    Symbol getConstellationSymbols() const;

    // Check if demodulator is currently synchronized (processing a frame)
    bool isSynced() const;

    // Check if demodulator has pending data (samples in buffer or accumulated soft bits)
    // Used to avoid premature reset between codewords of multi-codeword frames
    bool hasPendingData() const;

    // Reset state (e.g., after sync loss)
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Channel Estimator
 *
 * Estimates and tracks HF channel response using pilots.
 * Critical for good performance on fading channels.
 */
class ChannelEstimator {
public:
    explicit ChannelEstimator(const ModemConfig& config);
    ~ChannelEstimator();

    // Update estimate from received pilots
    void updateFromPilots(const Symbol& received_pilots,
                          const Symbol& expected_pilots);

    // Equalize a received symbol
    Symbol equalize(const Symbol& received);

    // Get channel quality metrics
    ChannelQuality getQuality() const;

    // Interpolate channel between pilots
    void interpolate();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace ultra
