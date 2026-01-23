#pragma once

// ModemEngine - Main modem interface class
// Handles TX/RX audio processing with OFDM, DPSK, MFSK modulation

#include "modem_types.hpp"
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/otfs.hpp"
#include "ultra/fec.hpp"  // LDPCEncoder, LDPCDecoder, Interleaver
#include "ultra/dsp.hpp"  // FIRFilter
#include "fsk/mfsk.hpp"   // MFSKModulator, MFSKDemodulator
#include "psk/dpsk.hpp"   // DPSKModulator, DPSKDemodulator
#include "../adaptive_mode.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <functional>
#include <string>
#include <atomic>
#include <thread>

namespace ultra {
namespace gui {

// Real modem engine using OFDM modulator/demodulator and LDPC codec
class ModemEngine {
public:
    ModemEngine();
    ~ModemEngine();

    // Set a name/prefix for logging (e.g., "OUR" or "SIM")
    void setLogPrefix(const std::string& prefix) { log_prefix_ = prefix; }
    const std::string& getLogPrefix() const { return log_prefix_; }

    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    void setConfig(const ModemConfig& config);
    const ModemConfig& getConfig() const { return config_; }

    void setFilterConfig(const FilterConfig& config);
    const FilterConfig& getFilterConfig() const { return filter_config_; }
    void setFilterEnabled(bool enabled);
    bool isFilterEnabled() const { return filter_config_.enabled; }

    // ========================================================================
    // TX: Convert data to audio samples
    // ========================================================================
    std::vector<float> transmit(const std::string& text);
    std::vector<float> transmit(const Bytes& data);

    // Minimal ping/pong probe (fast presence check, ~1 sec vs ~16 sec CONNECT)
    // Returns: preamble + raw DPSK "ULTR" bytes (no LDPC encoding)
    std::vector<float> transmitPing();
    std::vector<float> transmitPong();  // Same as ping, context determines meaning

    // Test signal generation
    std::vector<float> generateTestTone(float duration_sec = 1.0f);
    std::vector<float> transmitTestPattern(int pattern = 0);
    std::vector<float> transmitRawOFDM(int pattern = 0);

    // ========================================================================
    // RX: Audio input (thread-safe, non-blocking)
    // ========================================================================
    // Feed audio samples - call from any thread (audio callback, simulator, etc.)
    // Samples are buffered internally and processed by background threads.
    // Decoded frames trigger the RawDataCallback automatically.
    void feedAudio(const float* samples, size_t count);
    void feedAudio(const std::vector<float>& samples);

    // Inject test signal from file (for debugging/testing)
    size_t injectSignalFromFile(const std::string& filepath);

    // Check if we have decoded data ready (for polling if not using callbacks)
    bool hasReceivedData() const;

    // Get received data (clears the buffer) - use callbacks instead when possible
    std::string getReceivedText();
    Bytes getReceivedData();

    // ========================================================================
    // STATUS & CALLBACKS
    // ========================================================================
    LoopbackStats getStats() const;
    bool isSynced() const;
    float getCurrentSNR() const;
    ChannelQuality getChannelQuality() const;
    std::vector<std::complex<float>> getConstellationSymbols() const;

    using DataCallback = std::function<void(const std::string&)>;
    void setDataCallback(DataCallback callback) { data_callback_ = callback; }

    using RawDataCallback = std::function<void(const Bytes&)>;
    void setRawDataCallback(RawDataCallback callback) { raw_data_callback_ = callback; }

    using StatusCallback = std::function<void(const std::string&)>;
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }

    // Ping received callback - called when "ULTR" magic detected via DPSK
    // The measured_snr is estimated from preamble energy
    using PingReceivedCallback = std::function<void(float measured_snr)>;
    void setPingReceivedCallback(PingReceivedCallback callback) { ping_received_callback_ = callback; }

    void reset();

    // ========================================================================
    // CARRIER SENSE (Listen Before Talk)
    // ========================================================================
    bool isChannelBusy() const;
    float getChannelEnergy() const;
    void setCarrierSenseThreshold(float threshold);
    float getCarrierSenseThreshold() const;

    // Half-duplex turnaround delay
    void setTurnaroundDelay(uint32_t delay_ms) { turnaround_delay_ms_ = delay_ms; }
    uint32_t getTurnaroundDelay() const { return turnaround_delay_ms_; }
    bool isTurnaroundActive() const;
    uint32_t getTurnaroundRemaining() const;

    // ========================================================================
    // WAVEFORM & MODE CONTROL
    // ========================================================================
    void setWaveformMode(protocol::WaveformMode mode);
    protocol::WaveformMode getWaveformMode() const { return waveform_mode_; }

    void setConnected(bool connected);
    bool isConnected() const { return connected_; }

    void setUseConnectedWaveformOnce() { use_connected_waveform_once_ = true; }

    void setConnectWaveform(protocol::WaveformMode mode);
    protocol::WaveformMode getConnectWaveform() const { return connect_waveform_; }

    void setLastRxWaveform(protocol::WaveformMode mode) { last_rx_waveform_ = mode; }
    protocol::WaveformMode getLastRxWaveform() const { return last_rx_waveform_; }

    void setHandshakeComplete(bool complete);
    bool isHandshakeComplete() const { return handshake_complete_; }

    void setDataMode(Modulation mod, CodeRate rate);
    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    static void recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate);
    static protocol::WaveformMode recommendWaveformMode(float snr_db);

    void setMFSKMode(int num_tones);
    int getMFSKTones() const { return mfsk_config_.num_tones; }

    void setDPSKMode(DPSKModulation mod, int samples_per_symbol = 384);
    DPSKModulation getDPSKModulation() const { return dpsk_config_.modulation; }
    const DPSKConfig& getDPSKConfig() const { return dpsk_config_; }

private:
    ModemConfig config_;
    std::string log_prefix_ = "MODEM";

    // Waveform mode state
    protocol::WaveformMode waveform_mode_ = protocol::WaveformMode::OFDM;
    protocol::WaveformMode connect_waveform_ = protocol::WaveformMode::DPSK;
    protocol::WaveformMode last_rx_waveform_ = protocol::WaveformMode::DPSK;
    bool connected_ = false;
    bool handshake_complete_ = false;
    bool use_connected_waveform_once_ = false;

    // Data frame modulation (negotiated after probing)
    Modulation data_modulation_ = Modulation::QPSK;
    CodeRate data_code_rate_ = CodeRate::R1_2;

    // TX chain - OFDM
    std::unique_ptr<LDPCEncoder> encoder_;
    std::unique_ptr<OFDMModulator> ofdm_modulator_;

    // TX chain - OTFS
    std::unique_ptr<OTFSModulator> otfs_modulator_;
    OTFSConfig otfs_config_;

    // RX chain - OFDM
    std::unique_ptr<LDPCDecoder> decoder_;
    std::unique_ptr<OFDMDemodulator> ofdm_demodulator_;

    // RX chain - OTFS
    std::unique_ptr<OTFSDemodulator> otfs_demodulator_;

    // TX/RX chain - MFSK
    std::unique_ptr<MFSKModulator> mfsk_modulator_;
    std::unique_ptr<MFSKDemodulator> mfsk_demodulator_;
    MFSKConfig mfsk_config_;

    // TX/RX chain - DPSK
    std::unique_ptr<DPSKModulator> dpsk_modulator_;
    std::unique_ptr<DPSKDemodulator> dpsk_demodulator_;
    DPSKConfig dpsk_config_;

    // ========================================================================
    // RX ARCHITECTURE
    // ========================================================================

    // Sample buffer (feedAudio writes directly here, threads read from here)
    std::vector<float> rx_sample_buffer_;
    mutable std::mutex rx_buffer_mutex_;

    // Frame queue (acquisition -> RX)
    FrameQueue detected_frame_queue_;

    // RX thread state
    RxFrameState rx_frame_state_;

    // Acquisition thread
    std::thread acquisition_thread_;
    std::atomic<bool> acquisition_running_{false};
    std::condition_variable acquisition_cv_;
    std::mutex acquisition_mutex_;

    void acquisitionLoop();
    void startAcquisitionThread();
    void stopAcquisitionThread();

    // RX/Decode thread
    std::thread rx_decode_thread_;
    std::atomic<bool> rx_decode_running_{false};
    std::condition_variable rx_decode_cv_;
    std::mutex rx_decode_mutex_;

    void rxDecodeLoop();
    void startRxDecodeThread();
    void stopRxDecodeThread();

    // RX decode helpers
    bool rxDecodeDPSK(const DetectedFrame& frame);
    bool rxDecodeMFSK(const DetectedFrame& frame);
    bool rxDecodeOFDM();

    // OFDM multi-codeword accumulation
    std::vector<float> ofdm_accumulated_soft_bits_;
    int ofdm_expected_codewords_ = 0;

    // Multi-detect rate limiting
    size_t last_multidetect_buffer_size_ = 0;
    static constexpr size_t MULTIDETECT_MIN_NEW_SAMPLES = 4800;

    // Buffer limit
    static constexpr size_t MAX_PENDING_SAMPLES = 960000;
    std::queue<Bytes> rx_data_queue_;
    mutable std::mutex rx_mutex_;

    // Statistics
    LoopbackStats stats_;
    mutable std::mutex stats_mutex_;

    // Callbacks
    DataCallback data_callback_;
    RawDataCallback raw_data_callback_;
    StatusCallback status_callback_;
    PingReceivedCallback ping_received_callback_;

    // Adaptive modulation controller
    AdaptiveModeController adaptive_;

    // Interleaver
    Interleaver interleaver_{6, 108};
    bool interleaving_enabled_ = false;

    // Audio filters
    FilterConfig filter_config_;
    std::unique_ptr<FIRFilter> tx_filter_;
    std::unique_ptr<FIRFilter> rx_filter_;

    // Carrier sense
    std::atomic<float> channel_energy_{0.0f};
    float carrier_sense_threshold_ = 0.02f;
    static constexpr float ENERGY_SMOOTHING = 0.3f;

    // Half-duplex turnaround
    std::chrono::steady_clock::time_point last_rx_complete_time_;
    uint32_t turnaround_delay_ms_ = 200;

    // Helper methods
    void rebuildFilters();
    void updateChannelEnergy(const std::vector<float>& samples);
    std::vector<float> getBufferSnapshot() const;
    size_t getBufferSize() const;
    void consumeSamples(size_t count);
    void processRxBuffer_OFDM();
};

} // namespace gui
} // namespace ultra
