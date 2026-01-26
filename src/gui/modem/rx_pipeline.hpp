#pragma once

// RxPipeline - Template method pattern for RX frame decoding
//
// Extracts the common decode flow from processRxBuffer_* methods:
// 1. Detect preamble/sync
// 2. Demodulate symbols → soft bits
// 3. Decode CW0 to get frame info
// 4. Accumulate remaining codewords
// 5. Decode complete frame
// 6. Deliver frame data
//
// Each waveform provides its specific sync detection and demodulation,
// but the overall flow and LDPC decoding is shared.

#include "waveform/waveform_interface.hpp"
#include "fec/codec_interface.hpp"
#include "modem_types.hpp"
#include "protocol/frame_v2.hpp"
#include <functional>
#include <vector>
#include <string>

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

    // Set callbacks for frame delivery
    void setFrameCallback(FrameDeliveryCallback callback) { frame_callback_ = callback; }
    void setPingCallback(PingReceivedCallback callback) { ping_callback_ = callback; }
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }

    // Set data mode for adaptive rate decoding
    void setDataMode(CodeRate rate, bool connected);

    // Enable/disable interleaving
    void setInterleavingEnabled(bool enabled) { interleaving_enabled_ = enabled; }

    // ========================================================================
    // PROCESSING
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

    // ========================================================================
    // STATE
    // ========================================================================

    // Current accumulation state
    int getExpectedCodewords() const { return expected_codewords_; }
    int getAccumulatedCodewords() const;
    bool isAccumulating() const { return expected_codewords_ > 0; }

private:
    // Internal helpers

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

    // Configuration
    CodeRate data_code_rate_ = CodeRate::R1_4;
    bool connected_ = false;
    bool interleaving_enabled_ = true;

    // Callbacks
    FrameDeliveryCallback frame_callback_;
    PingReceivedCallback ping_callback_;
    StatusCallback status_callback_;
};

} // namespace gui
} // namespace ultra
