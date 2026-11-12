#pragma once

#include "frame_v2.hpp"
#include <functional>
#include <memory>

namespace ultra {
namespace protocol {

// ARQ mode selection
enum class ARQMode {
    STOP_AND_WAIT,      // Simple, low complexity, works everywhere
    SELECTIVE_REPEAT    // Higher throughput, needs more resources
};

const char* arqModeToString(ARQMode mode);

// ARQ configuration (shared across modes)
struct ARQConfig {
    uint32_t ack_timeout_ms = 8000;     // Time to wait for ACK (8s for faster retries on fading)
    uint32_t turnaround_ms = 500;       // TX to RX switch time
    int max_retries = 10;               // Max retransmission attempts (more retries for fading)

    // Selective Repeat specific
    size_t window_size = 4;             // TX window size (SR only)
    size_t rx_buffer_size = 8;          // RX reorder buffer size (SR only)
    uint32_t sack_delay_ms = 2000;      // Delay before sending SACK (wait for burst to complete)
    // ACK batch size: send SACK after this many in-order data frames received.
    // 0 = "track window_size" (preserves prior behavior). Must be <= window_size
    // when nonzero; setWindowSize() clamps it down on shrink.
    uint32_t ack_batch_size = 0;
};

// ARQ statistics (shared across modes)
struct ARQStats {
    int frames_sent = 0;
    int frames_received = 0;
    int acks_sent = 0;
    int acks_received = 0;
    int retransmissions = 0;
    int retransmissions_timeout = 0;
    int retransmissions_fast_hole = 0;
    int retransmissions_hole_probe = 0;
    int retransmissions_nack = 0;
    int timeouts = 0;
    int failed = 0;                     // Exceeded max retries
    int out_of_order = 0;               // Out-of-order frames (SR)
    int sacks_sent = 0;                 // Selective ACKs sent (SR)
    int sacks_received = 0;             // Sender-valid ACK/SACKs received, incl. valid duplicates (SR)
    int hole_events = 0;                // ACK hole indications observed
    int stale_acks_ignored = 0;         // ACK rejected as too old
    int future_acks_ignored = 0;        // ACK rejected as implausibly ahead
    int fabricated_acks_dropped = 0;    // ack referenced a seq NEVER SENT — corrupt/aliased/foreign, dropped
    int duplicate_acks_ignored = 0;     // ACK suppressed by dedup window
    // SACK trigger-reason counters (Phase 2 instrumentation, exclusive bumps —
    // each SACK send increments exactly one of these so the four sum equals
    // sacks_sent). Used to identify the dominant trigger before tuning.
    int sack_trigger_threshold = 0;     // frames_since_ack reached batch limit
    int sack_trigger_out_of_order = 0;  // hole detected (immediate, safety valve)
    int sack_trigger_timer = 0;         // sack_delay_ms timer flushed pending
    int sack_trigger_out_of_window = 0; // DATA arrived outside RX window
    int ack_repeat_jobs_coalesced = 0;  // Pending ACK repeat jobs replaced
    int ack_repeat_jobs_dropped = 0;    // Pending ACK repeat jobs dropped on overflow
    int partial_frames_received = 0;    // Partial MC-DPSK frames with CW0 decoded
    int partial_frames_completed = 0;   // Partial frames completed by later retransmission
    int partial_frame_crc_failed = 0;   // All CWs present but frame CRC rejected
    int partial_frame_expired = 0;      // Stale partial RX slots cleared
    int cw_nacks_sent = 0;              // Per-CW missing bitmap NACKs sent
    int cw_nacks_received = 0;          // Per-CW missing bitmap NACKs received
    int data_repairs_sent = 0;          // Compact DATA_REPAIR frames sent
    int data_repairs_received = 0;      // Compact DATA_REPAIR frames received
    int data_repair_cws_sent = 0;       // Original info CWs carried in repairs
    int data_repair_cws_merged = 0;     // Repair CWs accepted into partial RX slots
    int stale_epoch_acks_ignored = 0;   // ACK ignored: move-epoch != current TX epoch
                                        // (ULTRA_ARQ_MOVE_EPOCH, BUG-ARQ-SEQ-COLLISION)
    int descriptor_anchor_adoptions = 0;  // RX anchored to the peer's era/base from a burst
                                          // descriptor (INV-DESC-WINDOW-001)
    int descriptor_anchor_rejected = 0;   // descriptor anchor outside the peer's possible span
};

/**
 * Abstract ARQ Controller Interface
 *
 * Common interface for different ARQ strategies:
 * - Stop-and-Wait: Simple, one frame at a time
 * - Selective Repeat: Higher throughput with sliding window
 *
 * All implementations handle:
 * - Reliable data transfer with retransmission
 * - Sequence numbering and ACK/NAK handling
 * - Timeout management
 */
class IARQController {
public:
    // Callback types - uses Bytes for frame data (v2 serialized frames)
    using TransmitCallback = std::function<void(const Bytes&)>;
    using DataReceivedCallback = std::function<void(const Bytes& data)>;
    using SendCompleteCallback = std::function<void(bool success)>;

    virtual ~IARQController() = default;

    // Get the ARQ mode
    virtual ARQMode getMode() const = 0;

    // Set local and remote callsigns (must be set before use)
    virtual void setCallsigns(const std::string& local, const std::string& remote) = 0;

    // --- TX Side ---

    // Queue data for transmission
    // Returns false if cannot send (busy, window full, etc.)
    virtual bool sendData(const Bytes& data) = 0;
    virtual bool sendData(const std::string& text) = 0;

    // Send data with specific frame flags (e.g., MORE_DATA for file chunks)
    virtual bool sendDataWithFlags(const Bytes& data, uint8_t flags) = 0;

    // Check if ready to send (not busy/window not full)
    virtual bool isReadyToSend() const = 0;

    // Get number of available TX slots (1 for S&W, window_size - in_flight for SR)
    virtual size_t getAvailableSlots() const = 0;

    // --- RX Side ---

    // Check if MORE_DATA flag was set on last received frame
    virtual bool lastRxHadMoreData() const = 0;

    // Get flags from last received frame
    virtual uint8_t lastRxFlags() const = 0;

    // Process received frame data (v2 serialized bytes)
    virtual void onFrameReceived(const Bytes& frame_data) = 0;

    // --- Timing ---

    // Call periodically to handle timeouts
    virtual void tick(uint32_t elapsed_ms) = 0;

    // --- Callbacks ---

    virtual void setTransmitCallback(TransmitCallback cb) = 0;
    virtual void setDataReceivedCallback(DataReceivedCallback cb) = 0;
    virtual void setSendCompleteCallback(SendCompleteCallback cb) = 0;

    // --- State ---

    virtual ARQStats getStats() const = 0;
    virtual void resetStats() = 0;

    // Reset ARQ state
    virtual void reset() = 0;
};

// Factory function to create ARQ controller
std::unique_ptr<IARQController> createARQController(ARQMode mode, const ARQConfig& config = ARQConfig{});

} // namespace protocol
} // namespace ultra
