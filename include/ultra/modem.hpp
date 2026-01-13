#pragma once

#include "types.hpp"
#include <functional>
#include <memory>

namespace ultra {

/**
 * Main Modem Interface
 *
 * High-level API for sending and receiving data over HF.
 * Handles all the complexity internally:
 * - OFDM modulation/demodulation
 * - FEC encoding/decoding
 * - ARQ (automatic repeat request)
 * - Rate adaptation
 * - Synchronization
 */
class Modem {
public:
    // Callbacks
    using DataCallback = std::function<void(Bytes data)>;
    using AudioCallback = std::function<void(Samples audio)>;
    using StatsCallback = std::function<void(const ModemStats& stats)>;

    explicit Modem(const ModemConfig& config);
    ~Modem();

    // --- Transmit side ---

    // Queue data for transmission
    void send(ByteSpan data);

    // Get audio samples to transmit (call from audio callback)
    // Returns number of samples written
    size_t getTxSamples(MutableSampleSpan buffer);

    // Check if transmitter has data pending
    bool txPending() const;

    // --- Receive side ---

    // Feed received audio samples
    void rxSamples(SampleSpan samples);

    // Set callback for received data
    void setDataCallback(DataCallback cb);

    // --- Control ---

    // Start/stop modem
    void start();
    void stop();
    bool isRunning() const;

    // Connect to remote station (initiates handshake)
    void connect();

    // Disconnect gracefully
    void disconnect();

    bool isConnected() const;

    // --- Configuration ---

    // Force specific modulation (disable adaptation)
    void setModulation(Modulation mod);

    // Force specific code rate (disable adaptation)
    void setCodeRate(CodeRate rate);

    // Enable/disable rate adaptation
    void setAdaptive(bool enable);

    // --- Statistics ---

    ModemStats getStats() const;
    void setStatsCallback(StatsCallback cb);

    // Get current effective data rate in bps
    float getDataRate() const;

    // Get current channel quality
    ChannelQuality getChannelQuality() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Calculate theoretical max data rate for given parameters
 */
float calculateMaxDataRate(const ModemConfig& config, Modulation mod, CodeRate rate);

/**
 * Recommend modulation/coding for given channel quality
 */
std::pair<Modulation, CodeRate> recommendMode(const ChannelQuality& quality);

} // namespace ultra
