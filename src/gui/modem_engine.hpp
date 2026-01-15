#pragma once

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/otfs.hpp"
#include "ultra/fec.hpp"  // LDPCEncoder, LDPCDecoder, Interleaver
#include "ultra/dsp.hpp"  // FIRFilter
#include "protocol/frame.hpp"  // WaveformMode, FrameType
#include "adaptive_mode.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <functional>
#include <string>
#include <atomic>  // For std::atomic<float>

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

    // Get full channel quality (for link establishment/probing)
    ChannelQuality getChannelQuality() const;

    // Carrier Sense - Listen Before Talk (LBT)
    bool isChannelBusy() const;           // True if signal energy detected above threshold
    float getChannelEnergy() const;       // Current RMS energy level (0.0 - 1.0)
    void setCarrierSenseThreshold(float threshold);  // Set energy threshold (default 0.05)
    float getCarrierSenseThreshold() const;

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

    // === Waveform Mode Control ===
    // Control frames (CONNECT, CONNECT_ACK, etc.) always use OFDM
    // Data frames use the negotiated waveform mode

    // Set the waveform mode for data frames (called when connection negotiates mode)
    void setWaveformMode(protocol::WaveformMode mode);
    protocol::WaveformMode getWaveformMode() const { return waveform_mode_; }

    // Set connection state (affects which demodulator is used for RX)
    void setConnected(bool connected);
    bool isConnected() const { return connected_; }

    // Set negotiated data modulation (called after probing determines channel quality)
    // Both TX and RX will use this mode for DATA frames after connection
    void setDataMode(Modulation mod, CodeRate rate);
    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    // Compute recommended data mode from SNR (uses AdaptiveModeController thresholds)
    static void recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate);

private:
    ModemConfig config_;

    // Waveform mode state
    protocol::WaveformMode waveform_mode_ = protocol::WaveformMode::OFDM;
    bool connected_ = false;

    // Data frame modulation (negotiated after probing)
    // Link establishment frames always use BPSK R1/4
    Modulation data_modulation_ = Modulation::QPSK;
    CodeRate data_code_rate_ = CodeRate::R1_2;

    // TX chain - OFDM (always available, used for control frames)
    std::unique_ptr<LDPCEncoder> encoder_;
    std::unique_ptr<OFDMModulator> ofdm_modulator_;

    // TX chain - OTFS (used for data frames when negotiated)
    std::unique_ptr<OTFSModulator> otfs_modulator_;
    OTFSConfig otfs_config_;

    // RX chain - OFDM (always available)
    std::unique_ptr<LDPCDecoder> decoder_;
    std::unique_ptr<OFDMDemodulator> ofdm_demodulator_;

    // RX chain - OTFS (used when negotiated)
    std::unique_ptr<OTFSDemodulator> otfs_demodulator_;

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

    // Carrier sense state
    std::atomic<float> channel_energy_{0.0f};      // Current RMS energy (0.0 - 1.0)
    float carrier_sense_threshold_ = 0.02f;        // Energy threshold for "busy"
    static constexpr float ENERGY_SMOOTHING = 0.3f; // Smoothing factor for energy calculation

    // Rebuild filters when config changes
    void rebuildFilters();

    // Update channel energy from samples
    void updateChannelEnergy(const std::vector<float>& samples);

    // Process accumulated samples through demodulator
    void processRxBuffer();
};

} // namespace gui
} // namespace ultra
