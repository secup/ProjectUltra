#pragma once

#include "ultra/types.hpp"
#include <string>
#include <optional>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cctype>

namespace ultra {
namespace protocol {

// ============================================================================
// Shared Protocol Types (used by both connection management and v2 frames)
// ============================================================================

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
    OFDM_NVIS  = 0x00,  // High-speed OFDM with Schmidl-Cox sync (17+ dB, good conditions)
    OTFS_EQ    = 0x01,  // OTFS with TF equalization (best for Good channels)
    OTFS_RAW   = 0x02,  // OTFS without TF eq (best for Poor channels)
    MFSK       = 0x03,  // MFSK for very low SNR (-17 dB to +3 dB)
    MC_DPSK    = 0x04,  // Multi-Carrier DPSK for low SNR with fading (0-10 dB)
    OFDM_CHIRP_PILOTS = 0x05,  // OFDM with chirp sync + coherent QPSK + pilots (fading channels)
    AUTO       = 0xFF,  // Automatic selection (let receiver decide)
};

// Mode capabilities bitmap (for CONNECT payload)
namespace ModeCapabilities {
    constexpr uint8_t OFDM_NVIS  = 0x01;
    constexpr uint8_t OTFS_EQ    = 0x02;
    constexpr uint8_t OTFS_RAW   = 0x04;
    constexpr uint8_t MFSK       = 0x08;  // MFSK for very low SNR (-17 to +3 dB)
    constexpr uint8_t MC_DPSK    = 0x10;  // Multi-Carrier DPSK for low SNR with fading (0-10 dB)
    constexpr uint8_t OFDM_CHIRP_PILOTS = 0x20;  // OFDM with chirp sync + pilots (fading)
    constexpr uint8_t ALL        = OFDM_NVIS | OTFS_EQ | OTFS_RAW | MFSK | MC_DPSK | OFDM_CHIRP_PILOTS;
}

const char* waveformModeToString(WaveformMode mode);

// Channel report from PROBE_ACK (link establishment)
// Contains measured channel parameters for mode selection
struct ChannelReport {
    float snr_db = 0.0f;           // Measured SNR (dB)
    float delay_spread_ms = 0.0f;  // RMS delay spread (ms)
    float doppler_spread_hz = 0.0f; // Doppler spread (Hz)
    WaveformMode recommended_mode = WaveformMode::OFDM_NVIS;
    uint8_t capabilities = ModeCapabilities::ALL;

    // Encode to bytes for transmission (5 bytes)
    Bytes encode() const;

    // Decode from bytes
    static ChannelReport decode(const Bytes& data);

    // Get human-readable channel condition
    const char* getConditionName() const;
};

// Callsign utilities
// Maximum callsign length
constexpr size_t CALLSIGN_LEN = 8;

// Validate and sanitize callsign (uppercase, valid chars only)
inline std::string sanitizeCallsign(const std::string& call) {
    std::string result;
    result.reserve(CALLSIGN_LEN);
    for (char c : call) {
        if (result.size() >= CALLSIGN_LEN) break;
        if (std::isalnum(static_cast<unsigned char>(c)) || c == '/' || c == '-') {
            result += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
    }
    return result;
}

// Check if callsign is valid
inline bool isValidCallsign(const std::string& call) {
    // Ham callsigns are 3-10 characters (e.g., W1AW, VA2MVR, VE3ABC/P)
    if (call.size() < 3 || call.size() > CALLSIGN_LEN) return false;
    for (char c : call) {
        if (!std::isalnum(static_cast<unsigned char>(c)) && c != '/' && c != '-') {
            return false;
        }
    }
    return true;
}

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
//   CW0: [0x55][0x4C][TYPE][FLAGS][SEQ 2B][SRC 3B][DST 3B][TOTAL_CW][LEN 2B][HCRC 2B][payload 3B]
//   CW1+: [0xD5][INDEX][payload 18B]  (marker identifies data CWs, index = 1-254)
//   Last CW: remaining payload + 2-byte frame CRC (may have padding)
//
// This design ensures every codeword is self-identifying:
//   - 0x554C magic = header codeword (CW0)
//   - 0xD5 marker = data codeword with index following
// ============================================================================

// v2 Magic: "UL" (0x554C) - distinguishes from v1 "ULTR" (0x554C5452)
constexpr uint16_t MAGIC_V2 = 0x554C;

// Data codeword marker (0xD5) - identifies continuation codewords (CW1+)
// Bit pattern 11010101 is balanced (good RF properties)
constexpr uint8_t DATA_CW_MARKER = 0xD5;

// Bytes per LDPC codeword (R1/4: k=162 bits = 20.25 bytes, use 20)
constexpr size_t BYTES_PER_CODEWORD = 20;

// Maximum codewords per frame (8 bits = 255 max, index 0-254)
constexpr size_t MAX_CODEWORDS = 255;

// Codeword payload sizes:
// - CW0 (Header): 17-byte header leaves 3 bytes for payload start
// - CW1+ (Data): 2-byte header (marker + index) leaves 18 bytes for payload
constexpr size_t HEADER_CW_PAYLOAD_SIZE = 3;   // Payload bytes in CW0
constexpr size_t DATA_CW_HEADER_SIZE = 2;      // Marker + Index in CW1+
constexpr size_t DATA_CW_PAYLOAD_SIZE = 18;    // Payload bytes per CW1+

// Maximum payload size: 255 CWs * 20 bytes - overhead ≈ 5KB
// For larger transfers, use segmentation at higher layer
constexpr size_t MAX_PAYLOAD_V2 = 4096;

// Frame types
enum class FrameType : uint8_t {
    // Minimal probe frames (no LDPC, just raw bytes via DPSK)
    // Used for fast "anyone home?" check before full CONNECT
    PING        = 0x01,  // Quick presence check (preamble + "ULTR")
    PONG        = 0x02,  // Response to PING (preamble + "ULTR")

    // Control frames (1 codeword)
    PROBE       = 0x10,  // Channel probe request
    PROBE_ACK   = 0x11,  // Channel probe response
    CONNECT     = 0x12,  // Connection request (includes full callsigns)
    CONNECT_ACK = 0x13,  // Connection accepted
    CONNECT_NAK = 0x14,  // Connection rejected
    DISCONNECT  = 0x15,  // End connection
    KEEPALIVE   = 0x16,  // Maintain connection
    MODE_CHANGE = 0x17,  // Request modulation/rate change
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

// Check if a type is a control frame (1 codeword = 20 bytes)
// NOTE: CONNECT/CONNECT_ACK/CONNECT_NAK/DISCONNECT are now ConnectFrames (larger)
inline bool isControlFrame(FrameType type) {
    return type == FrameType::PROBE || type == FrameType::PROBE_ACK ||
           type == FrameType::KEEPALIVE || type == FrameType::MODE_CHANGE ||
           type == FrameType::ACK || type == FrameType::NACK ||
           type == FrameType::BEACON;
}

// MODE_CHANGE reason codes
namespace ModeChangeReason {
    constexpr uint8_t CHANNEL_IMPROVED = 0;  // SNR increased, can use faster mode
    constexpr uint8_t CHANNEL_DEGRADED = 1;  // SNR decreased, need more robust mode
    constexpr uint8_t USER_REQUEST     = 2;  // Manual mode selection
    constexpr uint8_t INITIAL_SETUP    = 3;  // First mode negotiation after connect
}

// SNR encoding for MODE_CHANGE (maps 0-255 to -10 to +53.75 dB, 0.25 dB steps)
inline uint8_t encodeSNR(float snr_db) {
    float clamped = std::max(-10.0f, std::min(53.75f, snr_db));
    return static_cast<uint8_t>((clamped + 10.0f) * 4.0f);
}

inline float decodeSNR(uint8_t encoded) {
    return (encoded / 4.0f) - 10.0f;
}

// Check if a type is a connect frame (carries full callsigns, ~41 bytes = 3 codewords)
// Includes DISCONNECT for proper station identification at end of contact
inline bool isConnectFrame(FrameType type) {
    return type == FrameType::CONNECT || type == FrameType::CONNECT_ACK ||
           type == FrameType::CONNECT_NAK || type == FrameType::DISCONNECT;
}

// Check if a type is a data frame (variable codewords)
inline bool isDataFrame(FrameType type) {
    uint8_t t = static_cast<uint8_t>(type);
    return t >= 0x30 && t <= 0x33;
}

// Frame type to string
const char* frameTypeToString(FrameType type);

// ============================================================================
// Ping Frame (4 bytes - NO LDPC, raw DPSK modulation)
// ============================================================================
// Used for fast "anyone home?" presence check before full CONNECT.
// Just preamble + "ULTR" magic bytes (~1 sec total vs ~16 sec for CONNECT).
//
// Format: [0x55][0x4C][0x54][0x52] = "ULTR" (same as v1 magic)
// The frame type (PING vs PONG) is determined by context:
//   - Disconnected station sends PING when initiating
//   - Station receiving PING responds with PONG
//
// This frame is NOT LDPC-encoded - it's raw bytes modulated via DPSK.
// Detection: After DPSK preamble, look for "ULTR" magic in demodulated bits.
struct PingFrame {
    static constexpr size_t SIZE = 4;
    static constexpr uint8_t MAGIC[4] = {0x55, 0x4C, 0x54, 0x52};  // "ULTR"

    // Create ping/pong bytes (always returns the 4-byte magic)
    static Bytes serialize() {
        return Bytes(MAGIC, MAGIC + SIZE);
    }

    // Check if data starts with ping magic
    static bool isPing(const Bytes& data) {
        if (data.size() < SIZE) return false;
        return data[0] == MAGIC[0] && data[1] == MAGIC[1] &&
               data[2] == MAGIC[2] && data[3] == MAGIC[3];
    }

    // Check if data starts with ping magic (from raw bytes)
    static bool isPing(const uint8_t* data, size_t len) {
        if (len < SIZE) return false;
        return data[0] == MAGIC[0] && data[1] == MAGIC[1] &&
               data[2] == MAGIC[2] && data[3] == MAGIC[3];
    }
};

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

    // Factory methods (by callsign)
    static ControlFrame makeProbe(const std::string& src, const std::string& dst);
    static ControlFrame makeProbeAck(const std::string& src, const std::string& dst,
                                      uint8_t snr_db, uint8_t recommended_rate);
    static ControlFrame makeAck(const std::string& src, const std::string& dst, uint16_t seq);
    static ControlFrame makeNack(const std::string& src, const std::string& dst,
                                  uint16_t seq, uint32_t cw_bitmap);
    static ControlFrame makeBeacon(const std::string& src);
    static ControlFrame makeKeepalive(const std::string& src, const std::string& dst);
    static ControlFrame makeModeChange(const std::string& src, const std::string& dst,
                                        uint16_t seq, Modulation new_mod, CodeRate new_rate,
                                        float snr_db, uint8_t reason);
    static ControlFrame makeModeChangeByHash(const std::string& src, uint32_t dst_hash,
                                              uint16_t seq, Modulation new_mod, CodeRate new_rate,
                                              float snr_db, uint8_t reason);
    static ControlFrame makeConnect(const std::string& src, const std::string& dst,
                                     uint8_t mode_capabilities, uint8_t preferred_mode);
    static ControlFrame makeConnectAck(const std::string& src, const std::string& dst,
                                        uint8_t negotiated_mode);
    static ControlFrame makeConnectNak(const std::string& src, const std::string& dst);

    // Factory methods (by hash - for responding to frames when callsign is unknown)
    static ControlFrame makeProbeAckByHash(const std::string& src, uint32_t dst_hash,
                                            uint8_t snr_db, uint8_t recommended_rate);
    static ControlFrame makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                              uint8_t negotiated_mode);
    static ControlFrame makeConnectNakByHash(const std::string& src, uint32_t dst_hash);
    static ControlFrame makeAckByHash(const std::string& src, uint32_t dst_hash, uint16_t seq);
    static ControlFrame makeNackByHash(const std::string& src, uint32_t dst_hash,
                                        uint16_t seq, uint32_t cw_bitmap);

    // Serialize to exactly 20 bytes
    Bytes serialize() const;

    // Deserialize from 20 bytes
    static std::optional<ControlFrame> deserialize(ByteSpan data);

    // Calculate CRC16
    static uint16_t calculateCRC(const uint8_t* data, size_t len);

    // Helper to extract MODE_CHANGE payload
    struct ModeChangeInfo {
        Modulation modulation;
        CodeRate code_rate;
        float snr_db;
        uint8_t reason;
    };

    // Parse MODE_CHANGE payload from a ControlFrame
    ModeChangeInfo getModeChangeInfo() const {
        ModeChangeInfo info;
        info.modulation = static_cast<Modulation>(payload[0]);
        info.code_rate = static_cast<CodeRate>(payload[1]);
        info.snr_db = decodeSNR(payload[2]);
        info.reason = payload[3];
        return info;
    }
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
    // Factory with explicit code rate for CW1+ (CW0 always R1/4)
    static DataFrame makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const Bytes& data, CodeRate cw1_rate);
    static DataFrame makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const std::string& text, CodeRate cw1_rate);

    // Calculate number of codewords needed (assumes R1/4 for all)
    static uint8_t calculateCodewords(size_t payload_size);
    // Calculate with explicit rate for CW1+ (CW0 always R1/4 = 20 bytes)
    static uint8_t calculateCodewords(size_t payload_size, CodeRate cw1_rate);

    // Serialize to bytes (will be split into codewords by encoder)
    // Returns: header + payload + frame_crc
    Bytes serialize() const;

    // Deserialize from reassembled codewords
    static std::optional<DataFrame> deserialize(ByteSpan data);

    // Get payload as text
    std::string payloadAsText() const;
};

// ============================================================================
// Connect Frame (for ham-compliant callsign identification)
// ============================================================================
// Uses DATA frame structure for variable length, carrying full callsigns.
// This ensures proper callsign identification per ham radio regulations.
//
// Payload format:
// ┌────────────┬────────────┬──────┬──────┬─────┬──────┬─────┐
// │ SRC_CALL   │ DST_CALL   │ CAPS │ WFMOD│ MOD │ RATE │ SNR │
// │ 10B (null) │ 10B (null) │  1B  │  1B  │ 1B  │  1B  │ 1B  │  = 25 bytes payload
// └────────────┴────────────┴──────┴──────┴─────┴──────┴─────┘
//
// CONNECT frame fields (dual meaning based on frame type):
//   - mode_capabilities: our supported waveform modes (bitmap)
//   - negotiated_mode:   CONNECT = forced waveform (0xFF=AUTO)
//                        CONNECT_ACK = agreed waveform
//   - initial_modulation: CONNECT = forced modulation (0xFF=AUTO)
//                         CONNECT_ACK = agreed modulation
//   - initial_code_rate:  CONNECT = forced code rate (0xFF=AUTO)
//                         CONNECT_ACK = agreed code rate
//   - measured_snr:       CONNECT = unused (0)
//                         CONNECT_ACK = responder's measured SNR
//
// When forced values are set (not 0xFF), responder MUST use them.
// When AUTO (0xFF), responder chooses based on measured SNR.
//
// Total frame: 17B header + 25B payload + 2B CRC = 44 bytes (3 codewords)
struct ConnectFrame {
    static constexpr size_t MAX_CALLSIGN_LEN = 10;  // 9 chars + null terminator
    static constexpr size_t PAYLOAD_SIZE = 25;       // 10 + 10 + 1 + 1 + 1 + 1 + 1

    FrameType type = FrameType::CONNECT;
    uint8_t flags = Flags::VERSION_V2;
    uint16_t seq = 0;
    uint32_t src_hash = 0;  // For routing (24-bit)
    uint32_t dst_hash = 0;  // For routing (24-bit)

    char src_callsign[MAX_CALLSIGN_LEN] = {0};  // Full source callsign
    char dst_callsign[MAX_CALLSIGN_LEN] = {0};  // Full destination callsign
    uint8_t mode_capabilities = 0;               // Supported waveform modes (CONNECT)
    uint8_t negotiated_mode = 0;                 // Forced/negotiated waveform (0xFF=AUTO)

    // Data mode fields - dual meaning based on frame type
    uint8_t initial_modulation = 0;              // Forced/agreed Modulation (0xFF=AUTO)
    uint8_t initial_code_rate = 0;               // Forced/agreed CodeRate (0xFF=AUTO)
    uint8_t measured_snr = 0;                    // CONNECT_ACK: measured SNR

    // Factory methods
    static ConnectFrame makeConnect(const std::string& src, const std::string& dst,
                                     uint8_t mode_caps, uint8_t forced_waveform,
                                     uint8_t forced_modulation = 0xFF,
                                     uint8_t forced_code_rate = 0xFF);
    static ConnectFrame makeConnectAck(const std::string& src, const std::string& dst,
                                        uint8_t neg_mode, Modulation init_mod, CodeRate init_rate,
                                        float snr_db);
    static ConnectFrame makeConnectNak(const std::string& src, const std::string& dst);
    static ConnectFrame makeDisconnect(const std::string& src, const std::string& dst);

    // Hash-based factory (for responding when only hash is known, fills in our callsign)
    static ConnectFrame makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                              uint8_t neg_mode, Modulation init_mod, CodeRate init_rate,
                                              float snr_db);
    static ConnectFrame makeConnectNakByHash(const std::string& src, uint32_t dst_hash);

    // Serialize to bytes (uses DATA frame format)
    Bytes serialize() const;

    // Deserialize from bytes
    static std::optional<ConnectFrame> deserialize(ByteSpan data);

    // Get callsigns as strings
    std::string getSrcCallsign() const;
    std::string getDstCallsign() const;
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

// LDPC parameters
constexpr size_t LDPC_CODEWORD_BITS = 648;  // n = total bits per codeword (all rates)
constexpr size_t LDPC_CODEWORD_BYTES = 81;  // n/8 = bytes per encoded codeword

// Get info bits per codeword for a given rate
// R1/4=162, R1/2=324, R2/3=432, R3/4=486, R5/6=540
inline size_t getInfoBitsForRate(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 162;
        case CodeRate::R1_3: return 216;
        case CodeRate::R1_2: return 324;
        case CodeRate::R2_3: return 432;
        case CodeRate::R3_4: return 486;
        case CodeRate::R5_6: return 540;
        default: return 162;  // Default to R1/4
    }
}

// Get bytes per codeword for a given rate (info bits / 8, rounded down)
inline size_t getBytesPerCodeword(CodeRate rate) {
    return getInfoBitsForRate(rate) / 8;  // 162/8=20, 324/8=40, 432/8=54, etc.
}

// Legacy constant for R1/4 (used by control frames)
constexpr size_t LDPC_INFO_BITS = 162;      // k = info bits per codeword (R1/4)

// Encode a v2 frame into LDPC codewords (R1/4 - for control frames)
// Input: serialized frame bytes
// Output: vector of LDPC-encoded codewords (81 bytes each)
std::vector<Bytes> encodeFrameWithLDPC(const Bytes& frame_data);

// Encode a v2 frame into LDPC codewords with specified rate (for DATA frames)
std::vector<Bytes> encodeFrameWithLDPC(const Bytes& frame_data, CodeRate rate);

// Decode LDPC codewords into frame data with per-codeword status
// Input: vector of soft bit vectors (648 floats per codeword)
// Output: CodewordStatus with per-codeword decode results
// Note: Caller should first decode codeword 0, then read TOTAL_CW to know how many more
CodewordStatus decodeCodewordsWithLDPC(const std::vector<std::vector<float>>& soft_bits);

// Decode a single codeword from soft bits (R1/4 - for control frames)
// Returns: (success, decoded_data)
std::pair<bool, Bytes> decodeSingleCodeword(const std::vector<float>& soft_bits);

// Decode a single codeword from soft bits with specified rate (for DATA frames)
std::pair<bool, Bytes> decodeSingleCodeword(const std::vector<float>& soft_bits, CodeRate rate);

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

// ============================================================================
// Codeword Identification Helpers
// ============================================================================

// Codeword type enumeration
enum class CodewordType {
    UNKNOWN,      // Not identifiable (corrupted or invalid)
    HEADER,       // CW0: Header codeword (0x554C magic)
    DATA,         // CW1+: Data codeword (0xD5 marker)
};

// Identify codeword type from first two bytes
// Returns type and index (index is valid only for DATA type, 1-254)
struct CodewordInfo {
    CodewordType type = CodewordType::UNKNOWN;
    uint8_t index = 0;  // Only valid for DATA codewords
};

CodewordInfo identifyCodeword(const Bytes& cw_data);

// Check if bytes look like a header codeword (starts with 0x554C)
inline bool isHeaderCodeword(const Bytes& data) {
    return data.size() >= 2 &&
           data[0] == ((MAGIC_V2 >> 8) & 0xFF) &&
           data[1] == (MAGIC_V2 & 0xFF);
}

// Check if bytes look like a data codeword (starts with 0xD5)
inline bool isDataCodeword(const Bytes& data) {
    return data.size() >= 2 && data[0] == DATA_CW_MARKER;
}

// Get index from data codeword (assumes isDataCodeword returned true)
inline uint8_t getDataCodewordIndex(const Bytes& data) {
    return data.size() >= 2 ? data[1] : 0;
}

} // namespace v2

// Bring v2 types into protocol namespace for convenience
using FrameType = v2::FrameType;
using ControlFrame = v2::ControlFrame;
using DataFrame = v2::DataFrame;

} // namespace protocol
} // namespace ultra
