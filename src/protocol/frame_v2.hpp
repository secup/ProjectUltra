#pragma once

#include "ultra/types.hpp"
#include <string>
#include <optional>
#include <cstdint>
#include <vector>

namespace ultra {
namespace protocol {
namespace v2 {

// ============================================================================
// ULTRA Protocol v2 - Optimized for per-codeword recovery
// ============================================================================
//
// Design goals:
// 1. Control frames fit in 1 LDPC codeword (≤20 bytes)
// 2. Data frames signal total codeword count in header
// 3. Per-codeword LDPC decode tracking enables selective retransmit
// 4. 24-bit callsign hashes for compact addressing
//
// Frame structure:
//
// Control frames (1 codeword = 20 bytes):
// ┌────────┬──────┬───────┬───────┬──────────┬──────────┬─────────┬───────┐
// │ MAGIC  │ TYPE │ FLAGS │ SEQ   │ SRC_HASH │ DST_HASH │ PAYLOAD │ CRC16 │
// │  2B    │  1B  │  1B   │  2B   │    3B    │    3B    │   6B    │  2B   │
// └────────┴──────┴───────┴───────┴──────────┴──────────┴─────────┴───────┘
//
// Data frames (N codewords):
// ┌────────┬──────┬───────┬───────┬──────────┬──────────┬─────────┬─────┬───────┐
// │ MAGIC  │ TYPE │ FLAGS │ SEQ   │ SRC_HASH │ DST_HASH │TOTAL_CW │ LEN │ HCRC  │
// │  2B    │  1B  │  1B   │  2B   │    3B    │    3B    │   1B    │ 2B  │  2B   │  = 17B
// └────────┴──────┴───────┴───────┴──────────┴──────────┴─────────┴─────┴───────┘
// │                              PAYLOAD (LEN bytes)                            │
// │                              + FRAME_CRC (2B) at end                        │
// └─────────────────────────────────────────────────────────────────────────────┘
//
// Codeword layout for data frames:
//   CW1: Header (17 bytes) + first 3 bytes of payload
//   CW2-N: 20 bytes payload each
//   Last CW: remaining payload + 2-byte frame CRC (may have padding)
// ============================================================================

// v2 Magic: "UL" (0x554C) - distinguishes from v1 "ULTR" (0x554C5452)
constexpr uint16_t MAGIC_V2 = 0x554C;

// Bytes per LDPC codeword (R1/4: k=162 bits = 20.25 bytes, use 20)
constexpr size_t BYTES_PER_CODEWORD = 20;

// Maximum codewords per frame (8 bits = 255 max)
constexpr size_t MAX_CODEWORDS = 255;

// Maximum payload size: 255 CWs * 20 bytes - overhead ≈ 5KB
// For larger transfers, use segmentation at higher layer
constexpr size_t MAX_PAYLOAD_V2 = 4096;

// Frame types
enum class FrameType : uint8_t {
    // Control frames (1 codeword)
    PROBE       = 0x10,  // Channel probe request
    PROBE_ACK   = 0x11,  // Channel probe response
    CONNECT     = 0x12,  // Connection request (includes full callsigns)
    CONNECT_ACK = 0x13,  // Connection accepted
    CONNECT_NAK = 0x14,  // Connection rejected
    DISCONNECT  = 0x15,  // End connection
    KEEPALIVE   = 0x16,  // Maintain connection
    ACK         = 0x20,  // Acknowledge frame
    NACK        = 0x21,  // Request retransmit (with codeword bitmap)
    BEACON      = 0x40,  // CQ broadcast

    // Data frames (variable codewords)
    DATA        = 0x30,  // Text/data payload
    DATA_START  = 0x31,  // First segment of large transfer
    DATA_CONT   = 0x32,  // Continuation segment
    DATA_END    = 0x33,  // Final segment
};

// Flags byte
namespace Flags {
    constexpr uint8_t NONE       = 0x00;
    constexpr uint8_t VERSION_V2 = 0x01;  // Always set for v2
    constexpr uint8_t URGENT     = 0x02;  // High priority
    constexpr uint8_t COMPRESSED = 0x04;  // Payload is compressed
    constexpr uint8_t ENCRYPTED  = 0x08;  // Payload is encrypted
    constexpr uint8_t MORE_FRAG  = 0x10;  // More fragments follow
    constexpr uint8_t FINAL      = 0x20;  // Final frame of transfer

    // Code rate (bits 6-7)
    constexpr uint8_t RATE_MASK  = 0xC0;
    constexpr uint8_t RATE_1_4   = 0x00;  // R1/4 (default)
    constexpr uint8_t RATE_1_2   = 0x40;  // R1/2
    constexpr uint8_t RATE_2_3   = 0x80;  // R2/3
    constexpr uint8_t RATE_3_4   = 0xC0;  // R3/4
}

// 24-bit callsign hash (DJB2 algorithm, truncated)
uint32_t hashCallsign(const std::string& callsign);

// Check if a type is a control frame (1 codeword)
inline bool isControlFrame(FrameType type) {
    uint8_t t = static_cast<uint8_t>(type);
    return (t >= 0x10 && t <= 0x21) || t == 0x40;
}

// Check if a type is a data frame (variable codewords)
inline bool isDataFrame(FrameType type) {
    uint8_t t = static_cast<uint8_t>(type);
    return t >= 0x30 && t <= 0x33;
}

// Frame type to string
const char* frameTypeToString(FrameType type);

// ============================================================================
// Control Frame (20 bytes - fits in 1 codeword)
// ============================================================================
struct ControlFrame {
    static constexpr size_t SIZE = 20;
    static constexpr size_t PAYLOAD_SIZE = 6;

    FrameType type = FrameType::PROBE;
    uint8_t flags = Flags::VERSION_V2;
    uint16_t seq = 0;
    uint32_t src_hash = 0;  // 24-bit (stored in lower 24 bits)
    uint32_t dst_hash = 0;  // 24-bit (0xFFFFFF = broadcast)
    uint8_t payload[PAYLOAD_SIZE] = {0};

    // Factory methods
    static ControlFrame makeProbe(const std::string& src, const std::string& dst);
    static ControlFrame makeProbeAck(const std::string& src, const std::string& dst,
                                      uint8_t snr_db, uint8_t recommended_rate);
    static ControlFrame makeAck(const std::string& src, const std::string& dst, uint16_t seq);
    static ControlFrame makeNack(const std::string& src, const std::string& dst,
                                  uint16_t seq, uint32_t cw_bitmap);
    static ControlFrame makeBeacon(const std::string& src);
    static ControlFrame makeKeepalive(const std::string& src, const std::string& dst);
    static ControlFrame makeDisconnect(const std::string& src, const std::string& dst);
    static ControlFrame makeConnect(const std::string& src, const std::string& dst,
                                     uint8_t mode_capabilities, uint8_t preferred_mode);
    static ControlFrame makeConnectAck(const std::string& src, const std::string& dst,
                                        uint8_t negotiated_mode);
    static ControlFrame makeConnectNak(const std::string& src, const std::string& dst);

    // Serialize to exactly 20 bytes
    Bytes serialize() const;

    // Deserialize from 20 bytes
    static std::optional<ControlFrame> deserialize(ByteSpan data);

    // Calculate CRC16
    static uint16_t calculateCRC(const uint8_t* data, size_t len);
};

// ============================================================================
// Data Frame (variable codewords)
// ============================================================================
struct DataFrame {
    static constexpr size_t HEADER_SIZE = 17;  // Before payload
    static constexpr size_t CRC_SIZE = 2;

    FrameType type = FrameType::DATA;
    uint8_t flags = Flags::VERSION_V2;
    uint16_t seq = 0;
    uint32_t src_hash = 0;  // 24-bit
    uint32_t dst_hash = 0;  // 24-bit
    uint8_t total_cw = 0;   // Total codewords for this frame
    uint16_t payload_len = 0;
    Bytes payload;

    // Factory methods
    static DataFrame makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const Bytes& data);
    static DataFrame makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const std::string& text);

    // Calculate number of codewords needed
    static uint8_t calculateCodewords(size_t payload_size);

    // Serialize to bytes (will be split into codewords by encoder)
    // Returns: header + payload + frame_crc
    Bytes serialize() const;

    // Deserialize from reassembled codewords
    static std::optional<DataFrame> deserialize(ByteSpan data);

    // Get payload as text
    std::string payloadAsText() const;
};

// ============================================================================
// NACK payload structure (for per-codeword recovery)
// ============================================================================
struct NackPayload {
    uint16_t frame_seq;      // Which frame had errors
    uint32_t cw_bitmap;      // Bit i = codeword i failed (up to 32 CWs)

    // Encode to 6 bytes (fits in control frame payload)
    void encode(uint8_t* out) const;

    // Decode from 6 bytes
    static NackPayload decode(const uint8_t* in);

    // Helper: count failed codewords
    int countFailed() const;

    // Helper: check if codeword i failed
    bool isFailed(int i) const { return (cw_bitmap >> i) & 1; }
};

// ============================================================================
// Codeword-aware encoder/decoder helpers
// ============================================================================

// Split serialized frame into codewords (20 bytes each, last may be padded)
std::vector<Bytes> splitIntoCodewords(const Bytes& frame_data);

// Reassemble codewords into frame data (strips padding from last)
Bytes reassembleCodewords(const std::vector<Bytes>& codewords, size_t expected_size);

// Track per-codeword decode status
struct CodewordStatus {
    std::vector<bool> decoded;  // true = LDPC succeeded for this CW
    std::vector<Bytes> data;    // Decoded data for each CW (20 bytes each)

    // Build NACK bitmap from decode status
    uint32_t getNackBitmap() const;

    // Check if all codewords decoded successfully
    bool allSuccess() const;

    // Count failures
    int countFailures() const;

    // Get total expected codewords (from first codeword header)
    // Returns 0 if first codeword not decoded or invalid
    uint8_t getExpectedCodewords() const;

    // Reassemble successfully decoded codewords into frame data
    // Returns empty if critical codewords missing
    Bytes reassemble() const;

    // Merge a retransmitted codeword into this status
    // Returns true if merge successful (codeword was previously failed)
    bool mergeCodeword(size_t index, const Bytes& cw_data);

    // Initialize for a frame with specified total codewords
    void initForFrame(uint8_t total_cw);
};

// ============================================================================
// LDPC Integration (codeword-aware encoding/decoding)
// ============================================================================

// LDPC parameters for R1/4
constexpr size_t LDPC_INFO_BITS = 162;      // k = info bits per codeword
constexpr size_t LDPC_CODEWORD_BITS = 648;  // n = total bits per codeword
constexpr size_t LDPC_CODEWORD_BYTES = 81;  // n/8 = bytes per encoded codeword

// Encode a v2 frame into LDPC codewords
// Input: serialized frame bytes
// Output: vector of LDPC-encoded codewords (81 bytes each)
std::vector<Bytes> encodeFrameWithLDPC(const Bytes& frame_data);

// Decode LDPC codewords into frame data with per-codeword status
// Input: vector of soft bit vectors (648 floats per codeword)
// Output: CodewordStatus with per-codeword decode results
// Note: Caller should first decode codeword 0, then read TOTAL_CW to know how many more
CodewordStatus decodeCodewordsWithLDPC(const std::vector<std::vector<float>>& soft_bits);

// Decode a single codeword from soft bits
// Returns: (success, decoded_data)
std::pair<bool, Bytes> decodeSingleCodeword(const std::vector<float>& soft_bits);

// Parse header from first decoded codeword (20 bytes)
// Returns: (is_valid, total_codewords, frame_type, payload_length)
struct HeaderInfo {
    bool valid = false;
    bool is_control = false;     // true = control frame (1 CW), false = data frame
    FrameType type = FrameType::PROBE;
    uint8_t total_cw = 1;
    uint16_t payload_len = 0;
    uint16_t seq = 0;
    uint32_t src_hash = 0;
    uint32_t dst_hash = 0;
};
HeaderInfo parseHeader(const Bytes& first_codeword_data);

} // namespace v2
} // namespace protocol
} // namespace ultra
