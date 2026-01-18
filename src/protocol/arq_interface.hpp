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
    uint32_t ack_timeout_ms = 5000;     // Time to wait for ACK
    uint32_t turnaround_ms = 100;       // TX to RX switch time
    int max_retries = 5;                // Max retransmission attempts

    // Selective Repeat specific
    size_t window_size = 4;             // TX window size (SR only)
    size_t rx_buffer_size = 8;          // RX reorder buffer size (SR only)
};

// ARQ statistics (shared across modes)
struct ARQStats {
    int frames_sent = 0;
    int frames_received = 0;
    int acks_sent = 0;
    int acks_received = 0;
    int retransmissions = 0;
    int timeouts = 0;
    int failed = 0;                     // Exceeded max retries
    int out_of_order = 0;               // Out-of-order frames (SR)
    int sacks_sent = 0;                 // Selective ACKs sent (SR)
    int sacks_received = 0;             // Selective ACKs received (SR)
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
    // Callback types
    using TransmitCallback = std::function<void(const Frame&)>;
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

    // Process a received frame
    virtual void onFrameReceived(const Frame& frame) = 0;

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
