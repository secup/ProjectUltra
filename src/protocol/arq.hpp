#pragma once

#include "arq_interface.hpp"
#include "frame_v2.hpp"
#include <functional>
#include <optional>

namespace ultra {
namespace protocol {

/**
 * Stop-and-Wait ARQ Controller
 *
 * Simple half-duplex ARQ suitable for HF:
 * 1. Send frame, wait for ACK
 * 2. If ACK received, send next frame
 * 3. If timeout, retransmit (up to max_retries)
 * 4. If max retries exceeded, report failure
 *
 * This is the simplest ARQ mode - one frame at a time.
 * Use Selective Repeat for higher throughput.
 */
class StopAndWaitARQ : public IARQController {
public:
    explicit StopAndWaitARQ(const ARQConfig& config = ARQConfig{});

    // --- IARQController Interface ---

    ARQMode getMode() const override { return ARQMode::STOP_AND_WAIT; }

    void setCallsigns(const std::string& local, const std::string& remote) override;

    bool sendData(const Bytes& data) override;
    bool sendData(const std::string& text) override;
    bool sendDataWithFlags(const Bytes& data, uint8_t flags) override;

    bool isReadyToSend() const override;
    size_t getAvailableSlots() const override { return isReadyToSend() ? 1 : 0; }

    bool lastRxHadMoreData() const override { return last_rx_more_data_; }
    uint8_t lastRxFlags() const override { return last_rx_flags_; }

    void onFrameReceived(const Bytes& frame_data) override;

    void tick(uint32_t elapsed_ms) override;

    void setTransmitCallback(TransmitCallback cb) override;
    void setDataReceivedCallback(DataReceivedCallback cb) override;
    void setSendCompleteCallback(SendCompleteCallback cb) override;

    ARQStats getStats() const override { return stats_; }
    void resetStats() override { stats_ = ARQStats{}; }

    void reset() override;

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
    uint16_t tx_seq_ = 0;
    Bytes pending_frame_data_;  // Serialized frame to send/resend
    int retry_count_ = 0;
    uint32_t timeout_remaining_ms_ = 0;

    // RX state
    uint16_t rx_expected_seq_ = 0;
    std::optional<Bytes> pending_ack_;  // ACK to send after turnaround
    bool last_rx_more_data_ = false;
    uint8_t last_rx_flags_ = 0;

    // Statistics
    ARQStats stats_;

    // Callbacks
    TransmitCallback on_transmit_;
    DataReceivedCallback on_data_received_;
    SendCompleteCallback on_send_complete_;

    // Internal helpers
    void transmitData(const Bytes& data);
    void handleDataFrame(const v2::DataFrame& frame);
    void handleAckFrame(const v2::ControlFrame& frame);
    void handleNackFrame(const v2::ControlFrame& frame);
    void retransmit();
    void sendFailed();
};

// Backward-compatible alias
using ARQController = StopAndWaitARQ;

} // namespace protocol
} // namespace ultra
