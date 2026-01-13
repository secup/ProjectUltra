#pragma once

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
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

// Real modem engine using OFDM modulator/demodulator and LDPC codec
class ModemEngine {
public:
    ModemEngine();
    ~ModemEngine();

    // Configuration
    void setConfig(const ModemConfig& config);
    const ModemConfig& getConfig() const { return config_; }

    // TX: Convert data to audio samples
    // Returns samples ready to play (includes preamble)
    std::vector<float> transmit(const std::string& text);
    std::vector<float> transmit(const Bytes& data);

    // RX: Process incoming audio samples
    // Call this with chunks of audio from the input
    void receiveAudio(const std::vector<float>& samples);

    // Check if we have decoded data ready
    bool hasReceivedData() const;

    // Get received data (clears the buffer)
    std::string getReceivedText();
    Bytes getReceivedData();

    // Status
    LoopbackStats getStats() const;
    bool isSynced() const;
    float getCurrentSNR() const;

    // Callback when data is received
    using DataCallback = std::function<void(const std::string&)>;
    void setDataCallback(DataCallback callback) { data_callback_ = callback; }

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
    std::vector<float> rx_sample_buffer_;
    std::queue<Bytes> rx_data_queue_;
    mutable std::mutex rx_mutex_;

    // Statistics
    LoopbackStats stats_;
    mutable std::mutex stats_mutex_;

    // Callback
    DataCallback data_callback_;

    // Process accumulated samples through demodulator
    void processRxBuffer();
};

} // namespace gui
} // namespace ultra
