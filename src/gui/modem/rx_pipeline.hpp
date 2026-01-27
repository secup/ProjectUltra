#pragma once

// RxPipeline - Streaming RX pipeline for frame decoding
//
// Provides feedAudio() interface for continuous audio streaming:
// 1. Accumulate audio samples in buffer
// 2. Periodically detect sync using IWaveform
// 3. When sync found, demodulate and decode
// 4. Queue decoded frames for retrieval
//
// This replaces the manual buffer management in ModemEngine's
// processRxBuffer_* methods with a clean streaming interface.
//
// Usage:
//   RxPipeline rx("RX");
//   rx.setWaveform(waveform.get());
//   rx.feedAudio(samples, count);  // Call repeatedly
//   while (rx.hasFrame()) {
//       auto result = rx.getFrame();
//       // Process result
//   }

#include "waveform/waveform_interface.hpp"
#include "fec/codec_interface.hpp"
#include "modem_types.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/fec.hpp"  // ChannelInterleaver
#include <functional>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <memory>

namespace ultra {
namespace gui {

// Result of processing a frame
struct RxFrameResult {
    bool success = false;
    Bytes frame_data;
    protocol::v2::FrameType frame_type = protocol::v2::FrameType::PROBE;
    int codewords_ok = 0;
    int codewords_failed = 0;
    float snr_estimate = 0.0f;
    float cfo_estimate = 0.0f;
    bool is_ping = false;
};

// Callback types for frame delivery
using FrameDeliveryCallback = std::function<void(const Bytes& frame_data, protocol::v2::FrameType type)>;
using PingReceivedCallback = std::function<void(float snr_db)>;
using StatusCallback = std::function<void(const std::string& message)>;

// RX Pipeline for frame decoding
// Handles the common decode pattern across all waveform types
class RxPipeline {
public:
    RxPipeline(const std::string& log_prefix = "RX");

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    // Set the waveform to use for sync detection and demodulation
    // The waveform must remain valid for the lifetime of the pipeline
    void setWaveform(IWaveform* waveform) { waveform_ = waveform; }
    IWaveform* getWaveform() const { return waveform_; }

    // Set callbacks for frame delivery (alternative to polling with hasFrame/getFrame)
    void setFrameCallback(FrameDeliveryCallback callback) { frame_callback_ = callback; }
    void setPingCallback(PingReceivedCallback callback) { ping_callback_ = callback; }
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }

    // Set data mode for adaptive rate decoding
    void setDataMode(CodeRate rate, bool connected);

    // Enable/disable interleaving
    void setInterleavingEnabled(bool enabled) { interleaving_enabled_ = enabled; }

    // Set interleaver parameters (must match TX)
    void setInterleaverConfig(size_t bits_per_symbol);

    // ========================================================================
    // STREAMING INTERFACE (NEW - replaces ModemEngine's feedAudio)
    // ========================================================================

    // Feed audio samples into the pipeline
    // Samples are buffered and processed automatically
    // Call this continuously with incoming audio data
    void feedAudio(const float* samples, size_t count);
    void feedAudio(const std::vector<float>& samples) { feedAudio(samples.data(), samples.size()); }

    // Check if any decoded frames are available
    bool hasFrame() const;

    // Get the next decoded frame (removes it from queue)
    // Returns empty result if no frames available
    RxFrameResult getFrame();

    // Get number of frames in queue
    size_t getFrameCount() const;

    // Get current buffer size (for monitoring)
    size_t getBufferSize() const;

    // ========================================================================
    // LEGACY INTERFACE (for compatibility with existing code)
    // ========================================================================

    // Process a detected frame using the given waveform
    // frame: detection result from acquisition thread
    // samples: audio samples (buffer snapshot)
    // waveform: the waveform to use for demodulation
    // Returns: result of frame processing
    RxFrameResult processFrame(const DetectedFrame& frame,
                               SampleSpan samples,
                               IWaveform* waveform);

    // Reset internal state (between frames)
    void reset();

    // Clear the audio buffer (call when switching modes)
    void clearBuffer();

    // ========================================================================
    // STATE
    // ========================================================================

    // Current accumulation state
    int getExpectedCodewords() const { return expected_codewords_; }
    int getAccumulatedCodewords() const;
    bool isAccumulating() const { return expected_codewords_ > 0; }

private:
    // Internal helpers

    // Try to detect and process a frame from the buffer
    // Returns true if a frame was found and processed
    bool tryProcessBuffer();

    // Decode soft bits to frame data
    RxFrameResult decodeFrame(const std::vector<float>& soft_bits, int num_codewords);

    // Check for PING frame (raw "ULTR" magic)
    bool detectPing(const std::vector<float>& soft_bits);

    // Deinterleave codewords if enabled
    std::vector<float> deinterleaveCodewords(const std::vector<float>& soft_bits);

    // Decode a single codeword
    bool decodeSingleCodeword(const std::vector<float>& soft_bits,
                               CodeRate rate, size_t expected_bytes,
                               Bytes& out_data);

    // Parse header from CW0
    struct HeaderInfo {
        bool valid = false;
        int total_cw = 1;
        protocol::v2::FrameType frame_type = protocol::v2::FrameType::PROBE;
    };
    HeaderInfo parseHeader(const Bytes& cw0_data);

    // State
    std::string log_prefix_;
    std::vector<float> accumulated_soft_bits_;
    int expected_codewords_ = 0;

    // Streaming state
    IWaveform* waveform_ = nullptr;
    std::vector<float> rx_buffer_;
    std::queue<RxFrameResult> frame_queue_;
    mutable std::mutex queue_mutex_;

    // Interleaver
    std::unique_ptr<ChannelInterleaver> interleaver_;
    size_t interleaver_bits_per_symbol_ = 60;  // Default for OFDM (30 carriers × 2 bits)

    // Configuration
    CodeRate data_code_rate_ = CodeRate::R1_4;
    bool connected_ = false;
    bool interleaving_enabled_ = true;

    // Callbacks
    FrameDeliveryCallback frame_callback_;
    PingReceivedCallback ping_callback_;
    StatusCallback status_callback_;

    // Constants
    static constexpr size_t MIN_SAMPLES_FOR_SYNC = 48000;  // 1 second minimum
    static constexpr size_t MAX_BUFFER_SIZE = 960000;      // 20 seconds max
};

} // namespace gui
} // namespace ultra
