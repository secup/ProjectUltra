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
    PROBE       = 0x0A,  // Channel probing request (link establishment)
    PROBE_ACK   = 0x0B,  // Channel report response
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

// Modulation modes for adaptive selection (negotiated in CONNECT)
enum class WaveformMode : uint8_t {
    OFDM     = 0x00,  // Standard OFDM (best for Moderate channels)
    OTFS_EQ  = 0x01,  // OTFS with TF equalization (best for Good channels)
    OTFS_RAW = 0x02,  // OTFS without TF eq (best for Poor channels)
    AUTO     = 0xFF,  // Automatic selection (let receiver decide)
};

// Mode capabilities bitmap (for CONNECT payload)
namespace ModeCapabilities {
    constexpr uint8_t OFDM     = 0x01;
    constexpr uint8_t OTFS_EQ  = 0x02;
    constexpr uint8_t OTFS_RAW = 0x04;
    constexpr uint8_t ALL      = OFDM | OTFS_EQ | OTFS_RAW;
}

const char* waveformModeToString(WaveformMode mode);

// Channel report from PROBE_ACK (link establishment)
// Contains measured channel parameters for mode selection
struct ChannelReport {
    float snr_db = 0.0f;           // Measured SNR (dB)
    float delay_spread_ms = 0.0f;  // RMS delay spread (ms)
    float doppler_spread_hz = 0.0f; // Doppler spread (Hz)
    WaveformMode recommended_mode = WaveformMode::OFDM;
    uint8_t capabilities = ModeCapabilities::ALL;

    // Encode to bytes for transmission (5 bytes)
    Bytes encode() const;

    // Decode from bytes
    static ChannelReport decode(const Bytes& data);

    // Get human-readable channel condition
    const char* getConditionName() const;
};

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

    // Create a CONNECT frame (with optional mode capabilities)
    // Payload format: [capabilities:1B, preferred_mode:1B]
    static Frame makeConnect(const std::string& src, const std::string& dst,
                             uint8_t capabilities = ModeCapabilities::ALL,
                             WaveformMode preferred = WaveformMode::AUTO);

    // Create a CONNECT_ACK frame (with negotiated mode)
    // Payload format: [negotiated_mode:1B]
    static Frame makeConnectAck(const std::string& src, const std::string& dst,
                                WaveformMode negotiated_mode = WaveformMode::OFDM);

    // Create a CONNECT_ACK frame with channel report (for CQ responses / implicit probing)
    // Payload format: [negotiated_mode:1B, channel_report:5B]
    // The channel report contains the responder's measured channel conditions
    static Frame makeConnectAckWithReport(const std::string& src, const std::string& dst,
                                          WaveformMode negotiated_mode,
                                          const ChannelReport& report);

    // Create a CONNECT_NAK frame
    static Frame makeConnectNak(const std::string& src, const std::string& dst);

    // Parse mode info from CONNECT payload
    static bool parseConnectPayload(const Bytes& payload,
                                    uint8_t& capabilities, WaveformMode& preferred);

    // Parse mode info from CONNECT_ACK payload
    static bool parseConnectAckPayload(const Bytes& payload, WaveformMode& mode);

    // Parse mode and channel report from CONNECT_ACK payload
    // Returns true if channel report was present
    static bool parseConnectAckPayload(const Bytes& payload, WaveformMode& mode,
                                       ChannelReport& report, bool& has_report);

    // Create a DISCONNECT frame
    static Frame makeDisconnect(const std::string& src, const std::string& dst);

    // Create a BEACON frame (CQ)
    static Frame makeBeacon(const std::string& src, const std::string& info = "");

    // Create a PROBE frame (channel sounding for link establishment)
    // Payload contains probe sequence identifier and capabilities
    static Frame makeProbe(const std::string& src, const std::string& dst,
                           uint8_t capabilities = ModeCapabilities::ALL);

    // Create a PROBE_ACK frame (channel report response)
    // Payload contains measured channel parameters
    static Frame makeProbeAck(const std::string& src, const std::string& dst,
                              const ChannelReport& report);

    // Parse channel report from PROBE_ACK payload
    static bool parseProbeAckPayload(const Bytes& payload, ChannelReport& report);

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
