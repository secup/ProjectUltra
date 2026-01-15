#pragma once

#include "ultra/types.hpp"
#include <string>
#include <optional>
#include <cstdint>

namespace ultra {
namespace protocol {

// Frame type identifiers
enum class FrameType : uint8_t {
    CONNECT     = 0x01,  // Request connection
    CONNECT_ACK = 0x02,  // Accept connection
    CONNECT_NAK = 0x03,  // Reject connection
    DISCONNECT  = 0x04,  // End connection
    DATA        = 0x05,  // Payload data
    ACK         = 0x06,  // Acknowledge received frame
    NAK         = 0x07,  // Request retransmission
    BEACON      = 0x08,  // CQ broadcast
    SACK        = 0x09,  // Selective ACK (for Selective Repeat ARQ)
};

// Convert frame type to string for debugging
const char* frameTypeToString(FrameType type);

// Frame flags
namespace FrameFlags {
    constexpr uint8_t NONE          = 0x00;
    constexpr uint8_t MORE_DATA     = 0x01;  // More frames follow
    constexpr uint8_t URGENT        = 0x02;  // High priority
    constexpr uint8_t COMPRESSED    = 0x04;  // Payload is compressed
    constexpr uint8_t ENCRYPTED     = 0x08;  // Payload is encrypted (future)
}

// Protocol frame structure
//
// Wire format (big-endian):
// ┌───────┬──────┬───────┬─────┬──────────┬──────────┬─────┬──────┬───────┐
// │ MAGIC │ TYPE │ FLAGS │ SEQ │ SRC_CALL │ DST_CALL │ LEN │ DATA │ CRC16 │
// │  1B   │  1B  │  1B   │ 1B  │    8B    │    8B    │ 2B  │  N   │  2B   │
// └───────┴──────┴───────┴─────┴──────────┴──────────┴─────┴──────┴───────┘
//
// Header: 22 bytes, CRC: 2 bytes, Max payload: 256 bytes
//
struct Frame {
    // Magic byte for frame detection
    static constexpr uint8_t MAGIC = 0x55;

    // Header size (before payload)
    static constexpr size_t HEADER_SIZE = 22;

    // CRC size
    static constexpr size_t CRC_SIZE = 2;

    // Minimum frame size (header + CRC, no payload)
    static constexpr size_t MIN_SIZE = HEADER_SIZE + CRC_SIZE;

    // Maximum payload size
    static constexpr size_t MAX_PAYLOAD = 256;

    // Maximum callsign length
    static constexpr size_t CALLSIGN_LEN = 8;

    // Frame fields
    FrameType type = FrameType::DATA;
    uint8_t flags = FrameFlags::NONE;
    uint8_t sequence = 0;
    std::string src_call;   // Source callsign (max 8 chars)
    std::string dst_call;   // Destination callsign (max 8 chars, "CQ" for broadcast)
    Bytes payload;          // Frame payload (max 256 bytes)

    // Constructors
    Frame() = default;

    // Create a data frame
    static Frame makeData(const std::string& src, const std::string& dst,
                          uint8_t seq, const Bytes& data);

    // Create a data frame from text
    static Frame makeData(const std::string& src, const std::string& dst,
                          uint8_t seq, const std::string& text);

    // Create an ACK frame
    static Frame makeAck(const std::string& src, const std::string& dst, uint8_t seq);

    // Create a NAK frame
    static Frame makeNak(const std::string& src, const std::string& dst, uint8_t seq);

    // Create a SACK frame (Selective ACK with bitmap)
    // base_seq: cumulative ACK point (all frames up to and including this are ACKed)
    // bitmap: bit i = (base_seq + 1 + i) received
    static Frame makeSack(const std::string& src, const std::string& dst,
                          uint8_t base_seq, uint8_t bitmap);

    // Create a CONNECT frame
    static Frame makeConnect(const std::string& src, const std::string& dst);

    // Create a CONNECT_ACK frame
    static Frame makeConnectAck(const std::string& src, const std::string& dst);

    // Create a CONNECT_NAK frame
    static Frame makeConnectNak(const std::string& src, const std::string& dst);

    // Create a DISCONNECT frame
    static Frame makeDisconnect(const std::string& src, const std::string& dst);

    // Create a BEACON frame (CQ)
    static Frame makeBeacon(const std::string& src, const std::string& info = "");

    // Serialize frame to bytes (includes CRC)
    Bytes serialize() const;

    // Deserialize frame from bytes
    // Returns nullopt if invalid (bad magic, CRC mismatch, truncated, etc.)
    static std::optional<Frame> deserialize(ByteSpan data);

    // Calculate CRC16-CCITT over data
    static uint16_t calculateCRC(const uint8_t* data, size_t len);

    // Get payload as text
    std::string payloadAsText() const;

    // Check if this is a control frame (no user payload)
    bool isControl() const;

    // Check if destination matches (exact or "CQ")
    bool isForCallsign(const std::string& call) const;

    // Validate and sanitize callsign (uppercase, valid chars only)
    static std::string sanitizeCallsign(const std::string& call);

    // Check if callsign is valid
    static bool isValidCallsign(const std::string& call);
};

// Debug output
std::string frameToString(const Frame& frame);

} // namespace protocol
} // namespace ultra
