#pragma once

#include "types.hpp"
#include <functional>
#include <memory>
#include <chrono>

namespace ultra {

/**
 * ARQ Controller
 *
 * Automatic Repeat reQuest - ensures reliable delivery.
 *
 * Strategy:
 * - Selective repeat (only retransmit failed frames)
 * - Sequence numbers for ordering
 * - Sliding window for throughput
 * - Fast NACK for quicker recovery
 * - Channel quality feedback in ACKs for rate adaptation
 */
class ARQController {
public:
    using SendFrameCallback = std::function<void(ByteSpan frame)>;
    using DeliveryCallback = std::function<void(Bytes data)>;

    explicit ARQController(const ModemConfig& config);
    ~ARQController();

    // --- TX side ---

    // Queue data for reliable transmission
    void sendData(ByteSpan data);

    // Called when TX frame has been sent (starts timeout)
    void onFrameSent(uint16_t seq_num);

    // Process received ACK/NACK
    void onAckReceived(uint16_t ack_seq, bool is_nack,
                       const ChannelQuality& remote_quality);

    // Set callback for frames to transmit
    void setSendCallback(SendFrameCallback cb);

    // --- RX side ---

    // Process received data frame
    void onDataReceived(uint16_t seq_num, ByteSpan data);

    // Set callback for delivered (in-order) data
    void setDeliveryCallback(DeliveryCallback cb);

    // Generate ACK frame for received data
    Bytes generateAck();

    // Generate NACK for missing frame
    Bytes generateNack(uint16_t missing_seq);

    // --- Timing ---

    // Call periodically to handle retransmissions
    void tick(std::chrono::steady_clock::time_point now);

    // --- State ---

    // Get number of frames in flight
    size_t framesInFlight() const;

    // Get number of frames pending retransmission
    size_t framesPendingRetry() const;

    // Reset ARQ state (e.g., on new connection)
    void reset();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Frame Builder
 *
 * Constructs frames with headers, sequence numbers, CRC.
 */
class FrameBuilder {
public:
    explicit FrameBuilder(const ModemConfig& config);

    // Build a data frame
    Bytes buildDataFrame(uint16_t seq_num, ByteSpan data);

    // Build ACK frame
    Bytes buildAckFrame(uint16_t ack_seq, const ChannelQuality& quality);

    // Build NACK frame
    Bytes buildNackFrame(uint16_t nack_seq);

    // Build sync frame (preamble data)
    Bytes buildSyncFrame();

    // Build probe frame (for channel estimation)
    Bytes buildProbeFrame();

    // Build connect request
    Bytes buildConnectFrame();

    // Build disconnect
    Bytes buildDisconnectFrame();

    // Get max payload size per frame
    size_t maxPayloadSize() const;

private:
    ModemConfig config_;
};

/**
 * Frame Parser
 *
 * Parses received frames, validates CRC.
 */
class FrameParser {
public:
    struct ParsedFrame {
        FrameType type;
        uint16_t seq_num;
        Bytes payload;
        ChannelQuality remote_quality;  // For ACK frames
        bool valid;                      // CRC check passed
    };

    explicit FrameParser(const ModemConfig& config);

    // Parse a received frame
    ParsedFrame parse(ByteSpan frame_data);

private:
    ModemConfig config_;
};

} // namespace ultra
