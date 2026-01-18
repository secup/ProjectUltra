#pragma once

#include "arq_interface.hpp"
#include "frame_v2.hpp"
#include <deque>
#include <optional>
#include <array>

namespace ultra {
namespace protocol {

/**
 * Selective Repeat ARQ Controller
 *
 * Sliding window ARQ for higher throughput on HF:
 * - Maintains window of N frames in flight simultaneously
 * - Retransmits only failed frames (not entire window)
 * - Uses SACK (Selective ACK) with bitmap for efficiency
 * - Reorders out-of-order frames at receiver
 *
 * Compared to Stop-and-Wait:
 * - 2-4x higher throughput for typical HF RTT
 * - More complex state management
 * - Requires more memory (TX/RX buffers)
 *
 * Window size should be chosen based on:
 * - Round-trip time (RTT = propagation + processing)
 * - Frame duration
 * - Target throughput
 */
class SelectiveRepeatARQ : public IARQController {
public:
    explicit SelectiveRepeatARQ(const ARQConfig& config = ARQConfig{});

    // --- IARQController Interface ---

    ARQMode getMode() const override { return ARQMode::SELECTIVE_REPEAT; }

    void setCallsigns(const std::string& local, const std::string& remote) override;

    bool sendData(const Bytes& data) override;
    bool sendData(const std::string& text) override;
    bool sendDataWithFlags(const Bytes& data, uint8_t flags) override;

    bool isReadyToSend() const override;
    size_t getAvailableSlots() const override;

    bool lastRxHadMoreData() const override { return last_rx_more_data_; }
    uint8_t lastRxFlags() const override { return last_rx_flags_; }

    void onFrameReceived(const Frame& frame) override;

    void tick(uint32_t elapsed_ms) override;

    void setTransmitCallback(TransmitCallback cb) override;
    void setDataReceivedCallback(DataReceivedCallback cb) override;
    void setSendCompleteCallback(SendCompleteCallback cb) override;

    ARQStats getStats() const override { return stats_; }
    void resetStats() override { stats_ = ARQStats{}; }

    void reset() override;

private:
    // TX state per frame in window
    struct TXSlot {
        bool active = false;        // Slot in use
        Frame frame;                // Frame to send/resend
        uint32_t timeout_ms = 0;    // Time until retransmit
        int retry_count = 0;        // Number of retransmits
        bool acked = false;         // ACK received (waiting for earlier frames)
    };

    // RX state per frame in receive window
    struct RXSlot {
        bool received = false;      // Frame received
        Frame frame;                // Received frame
    };

    // Maximum window size
    static constexpr size_t MAX_WINDOW = 8;

    ARQConfig config_;

    // Callsigns
    std::string local_call_;
    std::string remote_call_;

    // TX state
    std::array<TXSlot, MAX_WINDOW> tx_window_;
    uint8_t tx_base_seq_ = 0;       // First unACKed sequence number
    uint8_t tx_next_seq_ = 0;       // Next sequence to assign
    size_t tx_in_flight_ = 0;       // Number of frames in flight

    // RX state
    std::array<RXSlot, MAX_WINDOW> rx_window_;
    uint8_t rx_base_seq_ = 0;       // Next expected sequence
    bool last_rx_more_data_ = false;
    uint8_t last_rx_flags_ = 0;

    // Statistics
    ARQStats stats_;

    // Callbacks
    TransmitCallback on_transmit_;
    DataReceivedCallback on_data_received_;
    SendCompleteCallback on_send_complete_;

    // Internal helpers
    size_t seqToSlot(uint8_t seq) const;
    bool isInTXWindow(uint8_t seq) const;
    bool isInRXWindow(uint8_t seq) const;

    void transmitFrame(const Frame& frame);
    void handleDataFrame(const Frame& frame);
    void handleAckFrame(const Frame& frame);
    void handleNakFrame(const Frame& frame);
    void handleSackFrame(const Frame& frame);

    void retransmitFrame(size_t slot);
    void advanceTXWindow();
    void advanceRXWindow();
    void sendSack();

    uint8_t buildRXBitmap() const;
};

} // namespace protocol
} // namespace ultra
