#pragma once

#include "frame.hpp"
#include <functional>
#include <optional>

namespace ultra {
namespace protocol {

// ARQ configuration
struct ARQConfig {
    uint32_t ack_timeout_ms = 5000;     // Time to wait for ACK
    uint32_t turnaround_ms = 100;       // TX to RX switch time
    int max_retries = 5;                // Max retransmission attempts
};

// ARQ statistics
struct ARQStats {
    int frames_sent = 0;
    int frames_received = 0;
    int acks_sent = 0;
    int acks_received = 0;
    int retransmissions = 0;
    int timeouts = 0;
    int failed = 0;                     // Exceeded max retries
};

/**
 * Simple Stop-and-Wait ARQ Controller
 *
 * Half-duplex operation suitable for HF:
 * 1. Send frame, wait for ACK
 * 2. If ACK received, send next frame
 * 3. If timeout, retransmit (up to max_retries)
 * 4. If max retries exceeded, report failure
 *
 * Also handles incoming frames and generates ACKs.
 */
class ARQController {
public:
    // Callback types
    using TransmitCallback = std::function<void(const Frame&)>;
    using DataReceivedCallback = std::function<void(const Bytes& data)>;
    using SendCompleteCallback = std::function<void(bool success)>;

    explicit ARQController(const ARQConfig& config = ARQConfig{});

    // Set local and remote callsigns (must be set before use)
    void setCallsigns(const std::string& local, const std::string& remote);

    // --- TX Side ---

    // Queue data for transmission (will be sent as DATA frame)
    // Returns false if already waiting for ACK (busy)
    bool sendData(const Bytes& data);
    bool sendData(const std::string& text);

    // Send data with specific frame flags (e.g., MORE_DATA for file chunks)
    bool sendDataWithFlags(const Bytes& data, uint8_t flags);

    // Check if ready to send (not waiting for ACK)
    bool isReadyToSend() const;

    // Check if MORE_DATA flag was set on last received frame
    bool lastRxHadMoreData() const { return last_rx_more_data_; }

    // Get flags from last received frame
    uint8_t lastRxFlags() const { return last_rx_flags_; }

    // --- RX Side ---

    // Process a received frame (could be DATA, ACK, NAK, etc.)
    // Returns true if an ACK should be transmitted
    void onFrameReceived(const Frame& frame);

    // --- Timing ---

    // Call periodically (from main loop) to handle timeouts
    // Pass elapsed milliseconds since last call
    void tick(uint32_t elapsed_ms);

    // --- Callbacks ---

    // Set callback for frames that need to be transmitted
    void setTransmitCallback(TransmitCallback cb);

    // Set callback for received data (after ACK is queued)
    void setDataReceivedCallback(DataReceivedCallback cb);

    // Set callback for send completion (success/failure)
    void setSendCompleteCallback(SendCompleteCallback cb);

    // --- State ---

    ARQStats getStats() const { return stats_; }
    void resetStats() { stats_ = ARQStats{}; }

    // Reset ARQ state (on disconnect, etc.)
    void reset();

private:
    enum class State {
        IDLE,           // Ready to send
        WAIT_ACK,       // Sent frame, waiting for ACK
        COOLDOWN        // Just received ACK, brief pause before next TX
    };

    ARQConfig config_;
    State state_ = State::IDLE;

    // Callsigns
    std::string local_call_;
    std::string remote_call_;

    // TX state
    uint8_t tx_seq_ = 0;
    Frame pending_frame_;
    int retry_count_ = 0;
    uint32_t timeout_remaining_ms_ = 0;

    // RX state
    uint8_t rx_expected_seq_ = 0;
    std::optional<Frame> pending_ack_;  // ACK to send after turnaround
    bool last_rx_more_data_ = false;    // MORE_DATA flag from last received frame
    uint8_t last_rx_flags_ = 0;         // All flags from last received frame

    // Statistics
    ARQStats stats_;

    // Callbacks
    TransmitCallback on_transmit_;
    DataReceivedCallback on_data_received_;
    SendCompleteCallback on_send_complete_;

    // Internal helpers
    void transmitFrame(const Frame& frame);
    void handleDataFrame(const Frame& frame);
    void handleAckFrame(const Frame& frame);
    void handleNakFrame(const Frame& frame);
    void retransmit();
    void sendFailed();
};

} // namespace protocol
} // namespace ultra
