#pragma once

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/otfs.hpp"
#include "ultra/fec.hpp"  // LDPCEncoder, LDPCDecoder, Interleaver
#include "ultra/dsp.hpp"  // FIRFilter
#include "fsk/mfsk.hpp"   // MFSKModulator, MFSKDemodulator
#include "psk/dpsk.hpp"   // DPSKModulator, DPSKDemodulator
#include "protocol/frame_v2.hpp"  // WaveformMode, FrameType
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

    // Set a name/prefix for logging (e.g., "OUR" or "SIM")
    void setLogPrefix(const std::string& prefix) { log_prefix_ = prefix; }
    const std::string& getLogPrefix() const { return log_prefix_; }

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

    // RX: Inject test signal from file (for debugging/testing)
    // Returns number of samples injected, 0 on error
    size_t injectSignalFromFile(const std::string& filepath);

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

    // Half-duplex turnaround delay (prevents TX too soon after RX)
    void setTurnaroundDelay(uint32_t delay_ms) { turnaround_delay_ms_ = delay_ms; }
    uint32_t getTurnaroundDelay() const { return turnaround_delay_ms_; }
    bool isTurnaroundActive() const;      // True if we should wait before TX
    uint32_t getTurnaroundRemaining() const;  // Milliseconds until TX allowed

    // Get symbols for constellation display
    std::vector<std::complex<float>> getConstellationSymbols() const;

    // Callback when data is received (filtered text for display)
    using DataCallback = std::function<void(const std::string&)>;
    void setDataCallback(DataCallback callback) { data_callback_ = callback; }

    // Callback for raw bytes (for protocol layer, not filtered)
    using RawDataCallback = std::function<void(const Bytes&)>;
    void setRawDataCallback(RawDataCallback callback) { raw_data_callback_ = callback; }

    // Callback for RX status messages (codeword progress, etc.)
    using StatusCallback = std::function<void(const std::string&)>;
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }

    // Reset state (e.g., when switching modes)
    void reset();

    // Test signal generation for audio path verification
    // Returns 1 second of 1500 Hz tone
    std::vector<float> generateTestTone(float duration_sec = 1.0f);

    // Generate test frame with known data pattern for debugging
    // Pattern: 0=all zeros, 1=all ones, 2=alternating 0101
    std::vector<float> transmitTestPattern(int pattern = 0);

    // Generate RAW OFDM test (NO LDPC) for layer-by-layer debugging
    // pattern: 0 = 0xAA 0x55 alternating, 1 = DEADBEEF
    std::vector<float> transmitRawOFDM(int pattern = 0);

    // === Waveform Mode Control ===
    // Control frames (CONNECT, CONNECT_ACK, etc.) always use OFDM
    // Data frames use the negotiated waveform mode

    // Set the waveform mode for data frames (called when connection negotiates mode)
    void setWaveformMode(protocol::WaveformMode mode);
    protocol::WaveformMode getWaveformMode() const { return waveform_mode_; }

    // Set connection state (affects which demodulator is used for RX)
    void setConnected(bool connected);
    bool isConnected() const { return connected_; }

    // Set waveform for connection attempts (DPSK -> MFSK fallback)
    void setConnectWaveform(protocol::WaveformMode mode);
    protocol::WaveformMode getConnectWaveform() const { return connect_waveform_; }

    // Handshake state - responder stays on RX waveform until first post-ACK frame
    void setLastRxWaveform(protocol::WaveformMode mode) { last_rx_waveform_ = mode; }
    protocol::WaveformMode getLastRxWaveform() const { return last_rx_waveform_; }
    void setHandshakeComplete(bool complete);
    bool isHandshakeComplete() const { return handshake_complete_; }

    // Set negotiated data modulation (called after probing determines channel quality)
    // Both TX and RX will use this mode for DATA frames after connection
    void setDataMode(Modulation mod, CodeRate rate);
    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    // Compute recommended data mode from SNR (uses AdaptiveModeController thresholds)
    static void recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate);

    // Compute recommended waveform mode from SNR
    // Returns MFSK for very low SNR (<5 dB actual = -12 dB reported)
    // Returns OFDM for higher SNR
    static protocol::WaveformMode recommendWaveformMode(float snr_db);

    // Set MFSK mode based on conditions (2/4/8/16/32 tones)
    void setMFSKMode(int num_tones);
    int getMFSKTones() const { return mfsk_config_.num_tones; }

    // Set DPSK mode (for mid-SNR range: 0-15 dB)
    void setDPSKMode(DPSKModulation mod, int samples_per_symbol = 384);
    DPSKModulation getDPSKModulation() const { return dpsk_config_.modulation; }
    const DPSKConfig& getDPSKConfig() const { return dpsk_config_; }

private:
    ModemConfig config_;
    std::string log_prefix_ = "MODEM";  // Callsign used in log messages

    // Waveform mode state
    protocol::WaveformMode waveform_mode_ = protocol::WaveformMode::OFDM;
    protocol::WaveformMode connect_waveform_ = protocol::WaveformMode::DPSK;  // For CONNECT frames
    protocol::WaveformMode last_rx_waveform_ = protocol::WaveformMode::DPSK;  // Last waveform we received on
    bool connected_ = false;
    bool handshake_complete_ = false;  // True after first post-CONNECT_ACK frame received

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

    // TX/RX chain - MFSK (used for very low SNR, -17 to +5 dB reported)
    std::unique_ptr<MFSKModulator> mfsk_modulator_;
    std::unique_ptr<MFSKDemodulator> mfsk_demodulator_;
    MFSKConfig mfsk_config_;  // Current MFSK mode (adaptive: 2/4/8/16/32 tones)

    // TX/RX chain - DPSK (used for low-mid SNR, 0-15 dB)
    std::unique_ptr<DPSKModulator> dpsk_modulator_;
    std::unique_ptr<DPSKDemodulator> dpsk_demodulator_;
    DPSKConfig dpsk_config_;  // Current DPSK mode (DBPSK, DQPSK, D8PSK)

    // RX state
    std::vector<float> rx_sample_buffer_;      // Main processing buffer
    std::vector<float> rx_pending_samples_;    // Pending samples from callback (fast push)
    mutable std::mutex rx_pending_mutex_;      // Protects pending buffer only (fast lock)

    // Unified RX frame state (shared by DPSK/MFSK - OFDM has internal state)
    struct PendingFrame {
        bool active = false;                   // True if we detected a frame but need more samples
        protocol::WaveformMode waveform = protocol::WaveformMode::OFDM;
        int data_start = 0;                    // Position where data starts
        int expected_codewords = 0;            // How many codewords we're expecting
        size_t buffer_size = 0;                // Buffer size when frame was detected
        protocol::v2::FrameType frame_type = protocol::v2::FrameType::PROBE;  // For adaptive rate selection

        void clear() { active = false; frame_type = protocol::v2::FrameType::PROBE; }
        void set(protocol::WaveformMode wf, int start, int cw, size_t buf_size,
                 protocol::v2::FrameType ft = protocol::v2::FrameType::PROBE) {
            active = true;
            waveform = wf;
            data_start = start;
            expected_codewords = cw;
            buffer_size = buf_size;
            frame_type = ft;
        }
    };
    PendingFrame pending_frame_;

    // OFDM multi-codeword frame accumulation
    // Accumulates soft bits across multiple process() calls until we have all expected CWs
    std::vector<float> ofdm_accumulated_soft_bits_;
    int ofdm_expected_codewords_ = 0;  // Set from CW0 header

    // Multi-detect rate limiting (avoid repeated expensive preamble scans)
    size_t last_multidetect_buffer_size_ = 0;  // Buffer size at last detection attempt
    static constexpr size_t MULTIDETECT_MIN_NEW_SAMPLES = 4800;  // 100ms of new audio

    // Buffer limit (prevent unbounded growth if main loop stalls)
    static constexpr size_t MAX_PENDING_SAMPLES = 96000;  // 2 seconds at 48kHz
    std::queue<Bytes> rx_data_queue_;
    mutable std::mutex rx_mutex_;              // Protects processing buffer and queue

    // Statistics
    LoopbackStats stats_;
    mutable std::mutex stats_mutex_;

    // Callbacks
    DataCallback data_callback_;
    RawDataCallback raw_data_callback_;
    StatusCallback status_callback_;

    // Adaptive modulation controller
    AdaptiveModeController adaptive_;

    // Frequency-time interleaver (6x108 = 648 bits)
    // 6×108 provides optimal frequency diversity for HF channels:
    // - Consecutive LDPC bits separated by 6 positions in output
    // - With 30 bits/OFDM symbol, bits spread across different carriers
    // Testing showed 43% improvement at marginal SNR (16dB Good channel)
    Interleaver interleaver_{6, 108};
    bool interleaving_enabled_ = false;  // TESTING: disabled to debug CW1 decode issue

    // Audio filters (TX and RX share same settings but separate state)
    FilterConfig filter_config_;
    std::unique_ptr<FIRFilter> tx_filter_;
    std::unique_ptr<FIRFilter> rx_filter_;

    // Carrier sense state
    std::atomic<float> channel_energy_{0.0f};      // Current RMS energy (0.0 - 1.0)
    float carrier_sense_threshold_ = 0.02f;        // Energy threshold for "busy"
    static constexpr float ENERGY_SMOOTHING = 0.3f; // Smoothing factor for energy calculation

    // Half-duplex turnaround delay (prevents TX too soon after RX)
    std::chrono::steady_clock::time_point last_rx_complete_time_;
    uint32_t turnaround_delay_ms_ = 200;           // Delay after RX before TX allowed

    // Rebuild filters when config changes
    void rebuildFilters();

    // Update channel energy from samples
    void updateChannelEnergy(const std::vector<float>& samples);

    // Process accumulated samples through demodulator
    void processRxBuffer();

    // Waveform-specific RX processing (called by processRxBuffer)
    void processRxBuffer_OFDM();
    void processRxBuffer_DPSK(int pre_detected_start = -1);   // -1 = detect, else use provided
    void processRxBuffer_MFSK(int pre_detected_start = -1);   // -1 = detect, else use provided
    void processRxBuffer_MultiDetect();  // Multi-waveform detection when disconnected
};

} // namespace gui
} // namespace ultra
