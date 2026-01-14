#pragma once

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"  // LDPCEncoder, LDPCDecoder, Interleaver
#include "ultra/dsp.hpp"  // FIRFilter
#include "adaptive_mode.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <functional>
#include <string>

namespace ultra {
namespace gui {

// Statistics for display (local to modem engine)
struct LoopbackStats {
    float snr_db = 0.0f;
    float ber = 0.0f;
    int frames_sent = 0;
    int frames_received = 0;
    int frames_failed = 0;
    bool synced = false;
    int throughput_bps = 0;
};

// Audio filter configuration
struct FilterConfig {
    bool enabled = false;          // Disabled by default (radio's SSB filter sufficient)
    float center_freq = 1500.0f;   // Center frequency in Hz
    float bandwidth = 2900.0f;     // Bandwidth in Hz (covers 2.8 kHz modem + margin)
    int taps = 101;                // FIR filter taps (more = sharper cutoff)

    // Computed passband edges
    float lowFreq() const { return center_freq - bandwidth / 2.0f; }
    float highFreq() const { return center_freq + bandwidth / 2.0f; }
};

// Real modem engine using OFDM modulator/demodulator and LDPC codec
class ModemEngine {
public:
    ModemEngine();
    ~ModemEngine();

    // Configuration
    void setConfig(const ModemConfig& config);
    const ModemConfig& getConfig() const { return config_; }

    // Filter configuration
    void setFilterConfig(const FilterConfig& config);
    const FilterConfig& getFilterConfig() const { return filter_config_; }
    void setFilterEnabled(bool enabled);
    bool isFilterEnabled() const { return filter_config_.enabled; }

    // TX: Convert data to audio samples
    // Returns samples ready to play (includes preamble)
    std::vector<float> transmit(const std::string& text);
    std::vector<float> transmit(const Bytes& data);

    // RX: Queue incoming audio samples (fast - safe to call from audio callback)
    // Call this with chunks of audio from the input
    void receiveAudio(const std::vector<float>& samples);

    // Process queued audio samples (slow - call from main loop, NOT audio callback)
    // Returns true if there's more data to process
    bool pollRxAudio();

    // Check if we have decoded data ready
    bool hasReceivedData() const;

    // Get received data (clears the buffer)
    std::string getReceivedText();
    Bytes getReceivedData();

    // Status
    LoopbackStats getStats() const;
    bool isSynced() const;
    float getCurrentSNR() const;

    // Get symbols for constellation display
    std::vector<std::complex<float>> getConstellationSymbols() const;

    // Callback when data is received (filtered text for display)
    using DataCallback = std::function<void(const std::string&)>;
    void setDataCallback(DataCallback callback) { data_callback_ = callback; }

    // Callback for raw bytes (for protocol layer, not filtered)
    using RawDataCallback = std::function<void(const Bytes&)>;
    void setRawDataCallback(RawDataCallback callback) { raw_data_callback_ = callback; }

    // Reset state (e.g., when switching modes)
    void reset();

private:
    ModemConfig config_;

    // TX chain
    std::unique_ptr<LDPCEncoder> encoder_;
    std::unique_ptr<OFDMModulator> modulator_;

    // RX chain
    std::unique_ptr<LDPCDecoder> decoder_;
    std::unique_ptr<OFDMDemodulator> demodulator_;

    // RX state
    std::vector<float> rx_sample_buffer_;      // Main processing buffer
    std::vector<float> rx_pending_samples_;    // Pending samples from callback (fast push)
    mutable std::mutex rx_pending_mutex_;      // Protects pending buffer only (fast lock)
    std::queue<Bytes> rx_data_queue_;
    mutable std::mutex rx_mutex_;              // Protects processing buffer and queue

    // Statistics
    LoopbackStats stats_;
    mutable std::mutex stats_mutex_;

    // Callbacks
    DataCallback data_callback_;
    RawDataCallback raw_data_callback_;

    // Adaptive modulation controller
    AdaptiveModeController adaptive_;

    // Time interleaver (24x27 block = 648 bits = matches LDPC block size)
    // NOTE: Testing showed interleaving provides minimal benefit with LDPC's
    // pseudo-random structure, so it's disabled by default.
    Interleaver interleaver_{24, 27};
    bool interleaving_enabled_ = false;  // Disabled by default per test results

    // Audio filters (TX and RX share same settings but separate state)
    FilterConfig filter_config_;
    std::unique_ptr<FIRFilter> tx_filter_;
    std::unique_ptr<FIRFilter> rx_filter_;

    // Rebuild filters when config changes
    void rebuildFilters();

    // Process accumulated samples through demodulator
    void processRxBuffer();
};

} // namespace gui
} // namespace ultra
