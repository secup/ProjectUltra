#pragma once

#include "ultra/types.hpp"
#include "fec/soft_combine.hpp"
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
    // 0x00 reserved (formerly OFDM_COX — removed; was a Schmidl-Cox experiment
    // never selected by the auto ladder. OFDM uses OFDM_CHIRP/OFDM_NARROW.)
    OTFS_EQ    = 0x01,  // Reserved legacy value; not advertised by production builds
    OTFS_RAW   = 0x02,  // Reserved legacy value; not advertised by production builds
    MFSK       = 0x03,  // Reserved legacy value; not implemented
    MC_DPSK    = 0x04,  // Multi-Carrier DPSK for low SNR with fading (0-10 dB)
    OFDM_CHIRP = 0x05,  // OFDM with chirp sync + DQPSK (low SNR fading channels)
    OFDM_NARROW = 0x06, // Narrowband OFDM (500 Hz, 21 carriers) for very low SNR (3-10 dB)
    AUTO       = 0xFF,  // Automatic selection (let receiver decide)
};

// Mode capabilities bitmap (for CONNECT payload)
namespace ModeCapabilities {
    // 0x01 reserved (formerly OFDM_COX capability bit — removed)
    constexpr uint8_t OTFS_EQ    = 0x02;  // Reserved; not in ALL
    constexpr uint8_t OTFS_RAW   = 0x04;  // Reserved; not in ALL
    constexpr uint8_t MFSK       = 0x08;  // Reserved; not in ALL
    constexpr uint8_t MC_DPSK    = 0x10;  // Multi-Carrier DPSK for low SNR with fading (0-10 dB)
    constexpr uint8_t OFDM_CHIRP = 0x20;  // OFDM with chirp sync, coherent QPSK (fading)
    constexpr uint8_t OFDM_NARROW = 0x40; // Narrowband OFDM (500 Hz, 3-10 dB)
    constexpr uint8_t PHY_MASK_V1 = 0x80; // Optional per-frame PHY carrier mask header
    // OFDM_NARROW DISABLED (thread A, 2026-05-31): dropped from ALL so it is never
    // advertised/negotiated/constructed. This makes the differential OFDM demod dead
    // (the only OFDM mode still on DQPSK) so it can be removed; OFDM_NARROW returns
    // later as COHERENT OFDM with a narrowband config (separate re-validation — see
    // docs/REMOVAL_BACKLOG.md R3). The bit value 0x40 stays reserved for that return.
    constexpr uint8_t ALL        = MC_DPSK | OFDM_CHIRP;
}

const char* waveformModeToString(WaveformMode mode);

enum class LadderRungId : uint8_t {
    UNKNOWN     = 0,
    ROBUST_LOW  = 1,
    ROBUST_MID  = 2,
    ROBUST      = 3,
    STANDARD    = 4,
    OFDM_CHIRP  = 5,
    OFDM_NARROW = 6,
};

inline const char* ladderRungIdToString(LadderRungId id) {
    switch (id) {
        case LadderRungId::ROBUST_LOW:  return "Robust-Low";
        case LadderRungId::ROBUST_MID:  return "Robust-Mid";
        case LadderRungId::ROBUST:      return "Robust";
        case LadderRungId::STANDARD:    return "Standard";
        case LadderRungId::OFDM_CHIRP:  return "OFDM_CHIRP";
        case LadderRungId::OFDM_NARROW: return "OFDM_NARROW";
        case LadderRungId::UNKNOWN:
        default:                        return "Unknown";
    }
}

inline uint8_t packCWCountAndRung(uint8_t cw_count,
                                  LadderRungId rung_id,
                                  bool phy_mask_v1_capability = false) {
    uint8_t packed = static_cast<uint8_t>(cw_count & 0x0F);
    packed |= static_cast<uint8_t>((static_cast<uint8_t>(rung_id) & 0x07) << 4);
    if (phy_mask_v1_capability) {
        packed |= ModeCapabilities::PHY_MASK_V1;
    }
    return packed;
}

inline uint8_t unpackCWCount(uint8_t packed) {
    return static_cast<uint8_t>(packed & 0x0F);
}

inline LadderRungId unpackLadderRungId(uint8_t packed) {
    return static_cast<LadderRungId>((packed >> 4) & 0x07);
}

inline bool hasPhyMaskV1Capability(uint8_t capabilities) {
    return (capabilities & ModeCapabilities::PHY_MASK_V1) != 0;
}

inline uint8_t setPhyMaskV1Capability(uint8_t capabilities) {
    return capabilities | ModeCapabilities::PHY_MASK_V1;
}

// Helper: check if a WaveformMode is any OFDM variant (CHIRP or NARROW)
inline bool isOFDMMode(WaveformMode mode) {
    return mode == WaveformMode::OFDM_CHIRP ||
           mode == WaveformMode::OFDM_NARROW;
}

// Channel report from PROBE_ACK (link establishment)
// Contains measured channel parameters for mode selection
struct ChannelReport {
    float snr_db = 0.0f;           // Measured SNR (dB)
    float delay_spread_ms = 0.0f;  // RMS delay spread (ms)
    float doppler_spread_hz = 0.0f; // Doppler spread (Hz)
    WaveformMode recommended_mode = WaveformMode::OFDM_CHIRP;
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

// Physical-only DATA padding used to keep an OFDM burst descriptor-bearing or
// to complete an interleave group. These frames are deliberately addressed
// away from the receiver and never participate in SR-ARQ identity.
inline constexpr const char* kOFDMBurstPadCallsign = "ULPAD";
inline constexpr uint16_t kOFDMBurstPadSeq = 0xFFFE;

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
    TURNOVER    = 0x22,  // Current ISS yields DATA turn to peer
    TURN_REQUEST = 0x23, // Current IRS requests the next DATA turn
    FILE_CANCEL = 0x24,  // Abort the active file transfer on both peers
    BURST_HEADER = 0x25, // One-way burst descriptor: declares the following group's decode
    GROUP_ACK   = 0x26,  // Whole-burst ACK: acks one interleaved group by group_seq (stop-and-wait)
                         // params (group size, CW/frame, mod/rate, interleave). Fixed-format
                         // 1-CW control frame, non-interleaved, sent before the data group so
                         // the receiver decodes the group from the sender's declaration (§14.17).
    GROUP_NACK  = 0x27,  // Whole-burst NACK: receiver could not decode group_seq (0/8) — resend
                         // NOW instead of waiting out the group-ACK timeout (fast loss recovery).
    BEACON      = 0x40,  // CQ broadcast

    // Data frames (variable codewords)
    DATA        = 0x30,  // Text/data payload
    DATA_START  = 0x31,  // First segment of large transfer
    DATA_CONT   = 0x32,  // Continuation segment
    DATA_END    = 0x33,  // Final segment
    DATA_REPAIR = 0x34,  // MC-DPSK partial-CW repair for an in-flight DATA seq
};

// Flags byte
namespace Flags {
    constexpr uint8_t NONE       = 0x00;
    constexpr uint8_t VERSION_V2 = 0x01;  // Always set for v2
    constexpr uint8_t URGENT     = 0x02;  // Legacy name; no DATA writer uses it
    // DATA-only physical-envelope marker. The streaming encoder owns this bit:
    // it clears stale copies and stamps only the last frame actually placed in a
    // non-interleaved OFDM burst. It is distinct from FINAL, which is a logical
    // transfer boundary and may still have physical group members after it.
    constexpr uint8_t PHYSICAL_BURST_END = URGENT;
    constexpr uint8_t TURN_REQUEST = URGENT;  // ACK flag: IRS has queued DATA
    constexpr uint8_t COMPRESSED = 0x04;  // Payload is compressed
    constexpr uint8_t ENCRYPTED  = 0x08;  // (reserved) encryption was never implemented
    constexpr uint8_t MORE_FRAG  = 0x10;  // More fragments follow
    constexpr uint8_t FINAL      = 0x20;  // Final frame of transfer

    // MOVE-EPOCH (2026-07-03, BUG-ARQ-SEQ-COLLISION structural fix, knob
    // ULTRA_ARQ_MOVE_EPOCH default-OFF = all these bits stay 0 = byte-identical):
    //
    // EPOCH_REBASE (repurposes the never-implemented ENCRYPTED bit on DATA
    // frames ONLY, and only while the knob is ON): stamped on a DATA frame
    // created while its seq == the sender's TX window base — i.e. "no un-retired
    // seq exists below this one in my current epoch". After a rate-change TX
    // abort (which rewinds tx_next_seq_ and re-uses seqs for DIFFERENT content)
    // this marks the exact era anchor the receiver may safely re-anchor
    // rx_base_seq_ to; anchoring to any OTHER first-heard frame of a new epoch
    // could fabricate cumulative ACKs for lost head frames (see
    // SelectiveRepeatARQ move-epoch notes).
    constexpr uint8_t EPOCH_REBASE = ENCRYPTED;
    // 2-bit TX move-epoch counter, mod 4, in bits 6-7. These bits were reserved
    // for a code-rate-in-flags scheme (RATE_1_4/1_2/2_3/3_4) that was NEVER
    // implemented — no code ever set or read them (rate travels in
    // MODE_CHANGE / CONNECT_ACK / BURST_HEADER instead), so they were free.
    constexpr uint8_t EPOCH_MASK  = 0xC0;
    constexpr uint8_t EPOCH_SHIFT = 6;

    // CONNECT_ACK-only profile provenance. Bit 6 is otherwise a DATA-frame move-epoch
    // bit, so this is wire-compatible with existing peers and does not consume payload
    // space. A new initiator uses it to retain the responder's operator-forced rung for
    // the whole QSO; old peers safely ignore unknown CONNECT_ACK flag bits.
    constexpr uint8_t CONNECT_FORCED_PROFILE = 0x40;
}

// MOVE-EPOCH helpers (DATA-frame flags bits 6-7). Pure bit accessors — the
// knob gating lives in SelectiveRepeatARQ.
inline constexpr uint8_t epochFromFlags(uint8_t flags) {
    return static_cast<uint8_t>((flags & Flags::EPOCH_MASK) >> Flags::EPOCH_SHIFT);
}
inline constexpr uint8_t epochToFlags(uint8_t epoch) {
    return static_cast<uint8_t>((epoch & 0x3u) << Flags::EPOCH_SHIFT);
}

// 24-bit callsign hash (DJB2 algorithm, truncated)
uint32_t hashCallsign(const std::string& callsign);

// Check if a type is a control frame (1 codeword = 20 bytes)
// DISCONNECT is treated as a control frame so it gets the hardened 1-CW
// control-path profile (DQPSK R1/4) like ACK/NACK.
inline bool isControlFrame(FrameType type) {
    return type == FrameType::PROBE || type == FrameType::PROBE_ACK ||
           type == FrameType::KEEPALIVE || type == FrameType::MODE_CHANGE ||
           type == FrameType::ACK || type == FrameType::NACK ||
           type == FrameType::TURNOVER || type == FrameType::TURN_REQUEST ||
           type == FrameType::FILE_CANCEL ||
           type == FrameType::DISCONNECT ||
           type == FrameType::BURST_HEADER ||
           type == FrameType::GROUP_ACK ||
           type == FrameType::GROUP_NACK ||
           type == FrameType::BEACON;
}

// MODE_CHANGE reason codes
namespace ModeChangeReason {
    constexpr uint8_t CHANNEL_IMPROVED = 0;  // SNR increased, can use faster mode
    constexpr uint8_t CHANNEL_DEGRADED = 1;  // SNR decreased, need more robust mode
    constexpr uint8_t USER_REQUEST     = 2;  // Manual mode selection
    constexpr uint8_t INITIAL_SETUP    = 3;  // First mode negotiation after connect
    constexpr uint8_t STARTUP_PROBE_TIMEOUT = 4;  // One-shot higher-rung group was unACKed
    constexpr uint8_t STARTUP_PROBE_BEGIN = 5;  // Legacy control-plane launch of one-shot UP
}

// SNR encoding for MODE_CHANGE (maps 0-255 to -10 to +53.75 dB, 0.25 dB steps)
inline uint8_t encodeSNR(float snr_db) {
    float clamped = std::max(-10.0f, std::min(53.75f, snr_db));
    return static_cast<uint8_t>((clamped + 10.0f) * 4.0f);
}

inline float decodeSNR(uint8_t encoded) {
    return (encoded / 4.0f) - 10.0f;
}

// Fading index encoding.
// 0 => unavailable (for backward compatibility with older peers)
// 1..255 => 0.00..2.54 in 0.01 steps.
// Typical values: AWGN~0.04, Good~0.62, Moderate~0.90, Poor~1.10+.
inline uint8_t encodeFadingIndex(float fading_index) {
    if (fading_index < 0.0f) {
        return 0;  // unknown / unavailable
    }
    float clamped = std::max(0.0f, std::min(2.54f, fading_index));
    return static_cast<uint8_t>(1 + clamped * 100.0f + 0.5f);
}

inline float decodeFadingIndex(uint8_t encoded) {
    if (encoded == 0) {
        return -1.0f;  // unknown / not provided
    }
    return (encoded - 1) / 100.0f;
}

// Check if a type is a connect frame (carries full callsigns, ~41 bytes = 3 codewords)
// DISCONNECT remains here for legacy ConnectFrame decode compatibility.
inline bool isConnectFrame(FrameType type) {
    return type == FrameType::CONNECT || type == FrameType::CONNECT_ACK ||
           type == FrameType::CONNECT_NAK || type == FrameType::DISCONNECT;
}

// Check if a type is a data frame (variable codewords)
inline bool isDataFrame(FrameType type) {
    uint8_t t = static_cast<uint8_t>(type);
    return t >= 0x30 && t <= 0x34;
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
// PHY Mask Header (20 bytes - phase-2 carrier-mask control word)
// ============================================================================
struct PHYMaskHeader {
    static constexpr size_t SIZE = 20;
    static constexpr uint8_t MAGIC0 = 0x50;  // 'P'
    static constexpr uint8_t MAGIC1 = 0x4D;  // 'M'
    static constexpr uint8_t VERSION_V1 = 1;
    static constexpr uint8_t SCHEME_BITMAP_INTERLEAVER_V1 = 1;
    static constexpr uint8_t INTERLEAVER_CARRIER_LDPC_V1 = 0;
    static constexpr uint8_t MIN_MASK_COUNT = 1;
    static constexpr uint8_t MAX_MASK_COUNT = 8;
    static constexpr uint8_t DATA_CARRIER_COUNT = 59;
    static constexpr uint64_t ACTIVE_CARRIER_MASK = (uint64_t{1} << DATA_CARRIER_COUNT) - 1;

    uint8_t version = VERSION_V1;
    uint8_t scheme = SCHEME_BITMAP_INTERLEAVER_V1;
    uint8_t flags = 0;
    uint8_t payload_profile = 0;
    uint8_t interleaver_id = INTERLEAVER_CARRIER_LDPC_V1;
    uint8_t mask_count = MIN_MASK_COUNT;
    uint8_t reserved = 0;
    uint64_t active_mask = ACTIVE_CARRIER_MASK & ~uint64_t{1};
    uint16_t crc16 = 0;
    uint16_t inverted_crc16 = 0;

    static uint8_t packVersionScheme(uint8_t header_version, uint8_t header_scheme) {
        return static_cast<uint8_t>(((header_version & 0x0F) << 4) | (header_scheme & 0x0F));
    }

    static uint8_t packPayloadProfile(uint8_t cw_count, uint8_t mod_id, uint8_t rate_id) {
        return static_cast<uint8_t>((((cw_count - 1) & 0x07) << 5) |
                                    ((mod_id & 0x07) << 2) |
                                    (rate_id & 0x03));
    }

    uint8_t payloadCWCount() const { return static_cast<uint8_t>((payload_profile >> 5) + 1); }
    uint8_t payloadModId() const { return static_cast<uint8_t>((payload_profile >> 2) & 0x07); }
    uint8_t payloadRateId() const { return static_cast<uint8_t>(payload_profile & 0x03); }

    Bytes serialize() const;
    bool validateFields() const;

    static bool validate(ByteSpan data);
    static std::optional<PHYMaskHeader> deserialize(ByteSpan data);
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
    static ControlFrame makeTurnover(const std::string& src, const std::string& dst);
    static ControlFrame makeTurnRequest(const std::string& src, const std::string& dst);
    static ControlFrame makeFileCancel(const std::string& src, const std::string& dst);
    // Burst descriptor (§14.17): declares the decode params of the data group that follows.
    // interleave_flags: bit0 = burst (cross-frame) interleave, bit1 = carrier-LDPC
    // interleave, bit2 = next descriptor is light, bit3 = following DATA group
    // starts with a full chirp+LTS anchor.
    // lifting_z (2026-05-28): LDPC lifting size for the data group. 27 -> n=648
    // (legacy, short LDPC); 81 -> n=1944 (long LDPC for OFDM data). 0 / default
    // is wire-encoded as legacy 27 for backward compatibility with older peers.
    static ControlFrame makeBurstHeader(const std::string& src, const std::string& dst,
                                        uint16_t seq, uint8_t group_size, uint8_t cw_per_frame,
                                        Modulation mod, CodeRate rate, uint8_t interleave_flags,
                                        uint8_t lifting_z = 27);
    // Whole-burst ACK (§14.26): acks one interleaved group as a unit. group_seq
    // identifies the group; the sender (stop-and-wait) advances on a matching ACK
    // and resends the whole group on timeout. No per-frame bitmap.
    // quality_q (§14.36, Phase 5c): the receiver's decode-headroom feedback for the
    // group it just decoded, quantized to a byte (round(quality*254); 0xFF = none).
    // The SENDER runs the rate controller on it (it knows its own current rate) and
    // picks the next burst's rate. Honored only when adaptation is on; 0xFF / lost
    // ACK -> sender holds rate.
    static ControlFrame makeGroupAck(const std::string& src, const std::string& dst,
                                     uint16_t group_seq, uint8_t quality_q = 0xFF);
    // Whole-burst NACK (§14.30): the receiver decoded the descriptor but the
    // interleaved group failed (0/8) — tell the sender to resend group_seq NOW
    // rather than waiting out the group-ACK timeout (fast fade recovery).
    static ControlFrame makeGroupNack(const std::string& src, const std::string& dst,
                                      uint16_t group_seq);
    static ControlFrame makeBeacon(const std::string& src);
    static ControlFrame makeKeepalive(const std::string& src, const std::string& dst);
    static ControlFrame makeDisconnect(const std::string& src, const std::string& dst);
    static ControlFrame makeModeChange(const std::string& src, const std::string& dst,
                                        uint16_t seq, Modulation new_mod, CodeRate new_rate,
                                        float snr_db, float fading_index, uint8_t reason,
                                        uint8_t cw_count = 0,
                                        LadderRungId rung_id = LadderRungId::UNKNOWN);
    static ControlFrame makeModeChangeByHash(const std::string& src, uint32_t dst_hash,
                                              uint16_t seq, Modulation new_mod, CodeRate new_rate,
                                              float snr_db, float fading_index, uint8_t reason,
                                              uint8_t cw_count = 0,
                                              LadderRungId rung_id = LadderRungId::UNKNOWN);
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
        float fading_index;
        uint8_t reason;
        // Negotiated fixed-frame CW count for the new rate. 0 means "old peer
        // / unspecified — receiver picks via recommendCWCount(rate)". Wire
        // byte: payload[5] low nibble. Bits 4..6 carry LadderRungId.
        uint8_t data_frame_cw_count;
        LadderRungId ladder_rung_id;
    };

    // Parse MODE_CHANGE payload from a ControlFrame
    ModeChangeInfo getModeChangeInfo() const {
        ModeChangeInfo info;
        info.modulation = static_cast<Modulation>(payload[0]);
        info.code_rate = static_cast<CodeRate>(payload[1]);
        info.snr_db = decodeSNR(payload[2]);
        info.reason = payload[3];
        info.fading_index = decodeFadingIndex(payload[4]);
        info.data_frame_cw_count = unpackCWCount(payload[5]);
        info.ladder_rung_id = unpackLadderRungId(payload[5]);
        return info;
    }

    // Burst-descriptor interleave flag bits (BURST_HEADER payload[4]).
    static constexpr uint8_t BURST_FLAG_INTERLEAVE = 0x01;    // cross-frame burst interleave
    static constexpr uint8_t BURST_FLAG_CARRIER_LDPC = 0x02;  // carrier-LDPC interleave
    // #69 anchor-skip: THIS group announces that the NEXT group's descriptor will be LIGHT
    // (chirp-less). Lets the RX full-search chirp groups and light-search skip groups IMMEDIATELY
    // (no grinding through light rejects). Only set when ULTRA_ANCHOR_SKIP_K>1; default-off byte 0.
    static constexpr uint8_t BURST_FLAG_NEXT_LIGHT_ANCHOR = 0x04;
    // The DATA group immediately following this descriptor starts with a full
    // chirp+LTS anchor rather than the normal warm light-LTS marker. The sender
    // uses this for reliability/mode-switch anchors and timeout repairs. Without an
    // explicit bit, a same-mode receiver predicts a light marker inside the
    // full chirp, performs a guaranteed-junk demodulation, then has to recover
    // by searching for the real LTS. Absent on legacy peers => normal light
    // handoff, preserving the historical wire behavior.
    static constexpr uint8_t BURST_FLAG_CURRENT_GROUP_FULL_ANCHOR = 0x08;

    struct BurstHeaderInfo {
        uint8_t group_size = 0;     // frames in the interleaved group
        uint8_t cw_per_frame = 0;   // codewords per data frame
        Modulation modulation = Modulation::DQPSK;
        CodeRate code_rate = CodeRate::R1_4;
        bool burst_interleave = false;  // cross-frame interleave applied
        bool carrier_ldpc = false;      // carrier-LDPC interleave applied
        bool next_light_anchor = false; // #69: the NEXT group's descriptor is light (chirp-less)
        bool current_group_full_anchor = false; // following DATA group starts chirp+LTS
        // LDPC lifting size Z for the data group's codewords (2026-05-28).
        //   27 -> n=648 (legacy short LDPC, fast handshake / ACK class)
        //   81 -> n=1944 (long LDPC for OFDM data, ~3 dB more FEC margin)
        // Wire byte: payload[5]. payload[5]==0 means "unspecified/legacy" and
        // the receiver MUST treat it as Z=27 for backward compatibility with
        // peers that predate the lifting_z field.
        uint8_t lifting_z = 27;
    };

    // Parse a GROUP_ACK payload (§14.26): the acked group sequence number.
    uint16_t getGroupAckSeq() const {
        return static_cast<uint16_t>(payload[0] | (payload[1] << 8));
    }

    // Parse the GROUP_ACK quality feedback (§14.36): receiver's decode headroom for
    // the acked group, in [0,1]; returns <0 when absent (0xFF = no feedback).
    float getGroupAckQuality() const {
        if (payload[2] == 0xFF) return -1.0f;
        return static_cast<float>(payload[2]) / 254.0f;
    }

    // Parse a GROUP_NACK payload (§14.30): the group sequence that failed to decode.
    uint16_t getGroupNackSeq() const {
        return static_cast<uint16_t>(payload[0] | (payload[1] << 8));
    }

    // Parse a BURST_HEADER payload (§14.17). The receiver decodes the data group
    // that follows from THESE declared params, not from local config.
    BurstHeaderInfo getBurstHeaderInfo() const {
        BurstHeaderInfo info;
        info.group_size = payload[0];
        info.cw_per_frame = payload[1];
        info.modulation = static_cast<Modulation>(payload[2]);
        info.code_rate = static_cast<CodeRate>(payload[3]);
        info.burst_interleave = (payload[4] & BURST_FLAG_INTERLEAVE) != 0;
        info.carrier_ldpc = (payload[4] & BURST_FLAG_CARRIER_LDPC) != 0;
        info.next_light_anchor = (payload[4] & BURST_FLAG_NEXT_LIGHT_ANCHOR) != 0;  // #69
        info.current_group_full_anchor =
            (payload[4] & BURST_FLAG_CURRENT_GROUP_FULL_ANCHOR) != 0;
        // payload[5] == 0 -> legacy/unspecified -> Z=27 (n=648).
        // payload[5] == 81 -> long LDPC (n=1944). Any other unexpected value
        // also falls back to 27 (defensive: we'd rather decode a control-size
        // codeword and fail than try N=1944 with the wrong base matrix).
        info.lifting_z = (payload[5] == 81) ? 81 : 27;
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
    // Factory with explicit code rate (all codewords use same rate)
    static DataFrame makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const Bytes& data, CodeRate rate);
    static DataFrame makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const std::string& text, CodeRate rate);

    // Calculate number of codewords needed (assumes R1/4 for all)
    static uint8_t calculateCodewords(size_t payload_size);
    // Calculate with explicit rate (all codewords use same rate)
    static uint8_t calculateCodewords(size_t payload_size, CodeRate rate);

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
//   - mode_capabilities: CONNECT = our supported waveform modes (bitmap)
//                        CONNECT_ACK = responder fading index (encoded, 0=unknown)
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
// Total frame: 17B header + 25B payload + 2B CRC = 44 bytes
// Always uses 4 codewords with frame-level interleaving for fading resistance.
struct ConnectFrame {
    static constexpr size_t MAX_CALLSIGN_LEN = 10;  // 9 chars + null terminator
    static constexpr size_t PAYLOAD_SIZE = 26;       // 10 + 10 + 1 + 1 + 1 + 1 + 1 + 1 (cw_count)

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
    // Negotiated fixed-frame CW count.
    //   CONNECT:     initiator's forced CW count (0=AUTO, else 1..8)
    //   CONNECT_ACK: responder's chosen CW count (final agreed value, 1..8)
    // CONNECT_ACK cannot reuse mode_capabilities for PHY_MASK_V1 because that
    // byte carries the responder fading index. Its wire flag is bit 7 here.
    uint8_t data_frame_cw_count = 0;
    LadderRungId ladder_rung_id = LadderRungId::UNKNOWN;
    bool phy_mask_v1_capability = false;

    // Factory methods
    static ConnectFrame makeConnect(const std::string& src, const std::string& dst,
                                     uint8_t mode_caps, uint8_t forced_waveform,
                                     uint8_t forced_modulation = 0xFF,
                                     uint8_t forced_code_rate = 0xFF,
                                     uint8_t forced_cw_count = 0);
    static ConnectFrame makeConnectAck(const std::string& src, const std::string& dst,
                                        uint8_t neg_mode, Modulation init_mod, CodeRate init_rate,
                                        float snr_db, float fading_index,
                                        uint8_t cw_count,
                                        LadderRungId rung_id = LadderRungId::UNKNOWN);
    static ConnectFrame makeConnectNak(const std::string& src, const std::string& dst);
    static ConnectFrame makeDisconnect(const std::string& src, const std::string& dst);

    // Hash-based factory (for responding when only hash is known, fills in our callsign)
    static ConnectFrame makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                              uint8_t neg_mode, Modulation init_mod, CodeRate init_rate,
                                              float snr_db, float fading_index,
                                              uint8_t cw_count,
                                              LadderRungId rung_id = LadderRungId::UNKNOWN);
    static ConnectFrame makeConnectNakByHash(const std::string& src, uint32_t dst_hash);

    // Serialize to bytes (uses DATA frame format)
    Bytes serialize() const;

    // Deserialize from bytes
    static std::optional<ConnectFrame> deserialize(ByteSpan data);

    // Get callsigns as strings
    std::string getSrcCallsign() const;
    std::string getDstCallsign() const;
};

bool hasPhyMaskV1Capability(const ConnectFrame& frame);
void setPhyMaskV1Capability(ConnectFrame& frame);

// ============================================================================
// NACK payload structure (for per-codeword recovery)
// ============================================================================
struct NackPayload {
    uint16_t frame_seq;      // Which frame had errors
    uint32_t cw_bitmap;      // Bit i = codeword i failed/missing (up to 32 CWs)

    // Encode to 6 bytes (fits in control frame payload)
    void encode(uint8_t* out) const;

    // Decode from 6 bytes
    static NackPayload decode(const uint8_t* in);

    // Helper: count failed codewords
    int countFailed() const;

    // Helper: check if codeword i failed
    bool isFailed(int i) const { return (cw_bitmap >> i) & 1; }
};

// MC-DPSK compact repair frame. The serialized form is a sequence of raw LDPC
// info codewords: CW0 is this 20-byte header, CW1..CWk are the original DATA
// frame info codewords named by repair_bitmap in ascending bit order.
struct DataRepairFrame {
    static constexpr size_t HEADER_BYTES = 20;
    static constexpr uint8_t MAX_REPAIR_CW = 16;

    uint8_t flags = Flags::VERSION_V2;
    uint16_t target_seq = 0;
    uint32_t src_hash = 0;
    uint32_t dst_hash = 0;
    uint8_t original_total_cw = 0;
    uint16_t repair_bitmap = 0;
    uint8_t repair_count = 0;
    CodeRate rate = CodeRate::R1_4;
    std::vector<Bytes> repair_codewords;

    static DataRepairFrame make(const std::string& src, const std::string& dst,
                                uint16_t target_seq, uint8_t original_total_cw,
                                uint32_t repair_bitmap, CodeRate rate,
                                const std::vector<Bytes>& repair_codewords);

    bool valid() const;
    std::vector<uint8_t> repairIndices() const;
    Bytes headerCodeword() const;
    std::vector<Bytes> infoCodewords() const;
    Bytes serialize() const;

    static std::optional<DataRepairFrame> parseHeader(ByteSpan first_codeword);
    static std::optional<DataRepairFrame> deserialize(ByteSpan data);
};

// Partial MC-DPSK data-frame decode. CW0 must have decoded successfully, so
// seq/addressing are known. ARQ uses this to slot good CWs across full-frame
// retransmissions; the wire data frame itself is unchanged.
struct PartialFrameCodewords {
    FrameType type = FrameType::DATA;
    uint8_t flags = Flags::VERSION_V2;
    uint16_t seq = 0;
    uint32_t src_hash = 0;
    uint32_t dst_hash = 0;
    uint8_t total_cw = 0;
    uint32_t decoded_bitmap = 0;  // Bit i = CW i decoded and data[i] is valid.
    std::vector<Bytes> data;      // Decoded info bytes, indexed by CW.
    bool from_repair = false;     // DATA_REPAIR may omit CW0 when only CW1+ failed.

    bool valid() const {
        return total_cw > 0 && total_cw <= 32 &&
               data.size() >= total_cw &&
               (from_repair ? decoded_bitmap != 0 : (decoded_bitmap & 0x1u) != 0);
    }

    uint32_t expectedBitmap() const {
        if (total_cw == 0) return 0;
        if (total_cw >= 32) return 0xFFFFFFFFu;
        return (1u << total_cw) - 1u;
    }

    uint32_t missingBitmap() const {
        return expectedBitmap() & ~decoded_bitmap;
    }
};

// ============================================================================
// Codeword-aware encoder/decoder helpers
// ============================================================================

// Split serialized frame into codewords (20 bytes each, last may be padded)
std::vector<Bytes> splitIntoCodewords(const Bytes& frame_data);
std::vector<Bytes> splitIntoCodewords(const Bytes& frame_data, CodeRate rate);

// Reassemble codewords into frame data (strips padding from last)
Bytes reassembleCodewords(const std::vector<Bytes>& codewords, size_t expected_size);

// Track per-codeword decode status
struct CodewordStatus {
    std::vector<bool> decoded;  // true = LDPC succeeded for this CW
    std::vector<Bytes> data;    // Decoded data for each CW (20 bytes each)
    std::vector<int> iterations;             // Final LDPC iteration count per CW
    std::vector<int> unsatisfied_checks;     // Final syndrome weight per CW
    std::vector<float> llr_abs_mean;         // Pre-FEC |LLR| mean per CW
    std::vector<float> llr_abs_min;          // Pre-FEC |LLR| minimum per CW
    std::vector<float> llr_abs_p10;          // Pre-FEC |LLR| 10th percentile per CW
    std::vector<float> llr_abs_p50;          // Pre-FEC |LLR| median per CW
    std::vector<float> llr_abs_p90;          // Pre-FEC |LLR| 90th percentile per CW
    std::vector<uint8_t> used_perturbation;  // Non-zero if perturbation retry decoded this CW
    std::vector<int> harq_attempts;          // Soft-combine attempts contributing to this CW
    bool fixed_frame = false;   // true for OFDM fixed-CW frames without CW1+ markers

    // Build NACK bitmap from decode status
    uint32_t getNackBitmap() const;

    // Check if all codewords decoded successfully
    bool allSuccess() const;

    // True if ANY codeword only decoded after the perturbation retry. The existing
    // false-positive block already treats perturbation as a false-positive marker
    // (it refuses CRC recovery when it fired); any consumer that needs "these bits
    // are certainly what was transmitted" must refuse the frame for the same reason.
    bool usedAnyPerturbation() const {
        for (uint8_t p : used_perturbation) {
            if (p != 0) return true;
        }
        return false;
    }

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

// Encode already-formed LDPC info codewords without adding DATA_CW markers.
// DATA_REPAIR uses this because CW1.. are original frame info CWs, not a
// reserialized payload stream.
std::vector<Bytes> encodeInfoCodewordsWithLDPC(const std::vector<Bytes>& info_codewords,
                                               CodeRate rate);

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

inline bool isOFDMBurstPadHeader(const HeaderInfo& header) {
    return header.valid && !header.is_control &&
           header.dst_hash == hashCallsign(kOFDMBurstPadCallsign);
}

inline bool isAddressedToCallsign(const HeaderInfo& header, const std::string& local_call) {
    if (local_call.empty()) {
        return true;
    }
    const uint32_t our_hash = hashCallsign(local_call);
    return header.dst_hash == our_hash || header.dst_hash == 0xFFFFFF;
}

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

// ============================================================================
// Fixed-Codeword Frame with Frame-Level Interleaving
// ============================================================================
//
// For OFDM data frames, use a fixed-size codeword structure with interleaving
// across all codewords. The default remains 4 codewords for compatibility, but
// the count is runtime-selectable for aggregation experiments. This provides:
//
// 1. FADING RESISTANCE: Burst errors spread across all CWs, so each sees only
//    a fraction of the errors instead of one CW being completely corrupted.
//
// 2. SIMPLE RECEIVER: Peers carry the configured fixed-frame CW count in
//    connection state while the frame header still advertises TOTAL_CW.
//
// 3. PREDICTABLE TIMING: Frame duration is derived from the configured count.
//
// Payload capacity varies by code rate and CW count. With the default 4 CWs:
//   R1/4: 4 × 20 bytes = 80 bytes - 19 overhead = 61 bytes usable
//   R1/2: 4 × 40 bytes = 160 bytes - 19 overhead = 141 bytes usable
//   R2/3: 4 × 54 bytes = 216 bytes - 19 overhead = 197 bytes usable
//   R3/4: 4 × 60 bytes = 240 bytes - 19 overhead = 221 bytes usable
//
// Frame structure (same as DataFrame but with a fixed configured CW count):
//   [HEADER 17B][PAYLOAD][CRC 2B] → LDPC encode → N CWs → Interleave → TX
//
// ============================================================================

// Fixed frame constants
inline constexpr int kMinFixedFrameCodewords = 1;
// 8 -> 16 (2026-07-05, cw16 build — fable_analysis/09 §5 item 5): 16 CWs/frame gives
// 16QAM the same ~1272 ms frame airtime as the proven QPSK cw8 geometry (same
// coherence exposure, HALF the LTS/header overhead per bit; 16QAM ceiling 3.3k ->
// ~3.9k). This cap is the wire/clamp LIMIT — actual selection stays per-mod/per-rate
// in recommendCWCount* (QAM16 baseline 16 behind ULTRA_QAM16_CW16), and the
// coherence walk in recommendCWCountForChannel still shrinks frames whenever frame
// airtime would exceed the measured coherence time (adaptivity rule). The cw16
// hardware gate (fable 09 §3b QAM16 acquisition collapse) was cleared 2026-07-05:
// the collapse was the tone-lock + anchor-wait-disarm bugs, not PAPR.
inline constexpr int kMaxFixedFrameCodewords = 16;
inline constexpr int kDefaultFixedFrameCodewords = 4;
inline constexpr int FIXED_FRAME_CODEWORDS = kDefaultFixedFrameCodewords;  // source compatibility
constexpr uint16_t DISCONNECT_SEQ = 0xFFFF;  // Unique seq for DISCONNECT (won't collide with ARQ 0-based seqs)
constexpr int FIXED_FRAME_OVERHEAD = DataFrame::HEADER_SIZE + DataFrame::CRC_SIZE;  // 17 + 2 = 19 bytes

inline int sanitizeFixedFrameCodewords(int cw_count) {
    return std::clamp(cw_count, kMinFixedFrameCodewords, kMaxFixedFrameCodewords);
}

/**
 * Get payload capacity for fixed frame at given code rate and CW count.
 *
 * @param rate Code rate (determines info bytes per codeword)
 * @param cw_count Codewords per frame, clamped to supported fixed-frame range
 * @return Maximum payload bytes that fit in a fixed frame
 */
inline size_t getFixedFramePayloadCapacity(CodeRate rate, int cw_count) {
    cw_count = sanitizeFixedFrameCodewords(cw_count);
    size_t total_info_bytes = static_cast<size_t>(cw_count) * getBytesPerCodeword(rate);
    return total_info_bytes - FIXED_FRAME_OVERHEAD;
}

// 2026-05-28: number of info-block columns in the 802.11n base matrix per rate.
//   k_per_lifting = infoBlocksForRate(rate) * Z.
// Mirrors the Codex-built helper at frame_v2.cpp::infoBlocksForRate (file-local).
// Exposed here so capacity callers in the connection layer can size chunks at
// the active lifting (Z=27 -> legacy k; Z=81 -> 3x k for the long-LDPC path).
inline int infoBlocksForRateBaseMatrix(CodeRate r) {
    switch (r) {
        case CodeRate::R1_4: return 6;
        case CodeRate::R1_2: return 12;
        case CodeRate::R2_3: return 16;
        case CodeRate::R3_4: return 18;
        case CodeRate::R5_6: return 20;
        default:             return 12;
    }
}

// Z-aware info bytes per codeword. Lifting Z=27 reproduces getBytesPerCodeword.
// Z=81 gives 3x as many info bytes per CW — the actual long-LDPC capacity.
inline size_t getBytesPerCodewordZ(CodeRate rate, int lifting_z) {
    return static_cast<size_t>(infoBlocksForRateBaseMatrix(rate))
           * static_cast<size_t>(lifting_z) / 8;
}

/**
 * Z-aware payload capacity for a fixed frame. The legacy getFixedFramePayloadCapacity
 * computes from getBytesPerCodeword() which uses hardcoded Z=27 info bit counts —
 * that's correct only on the legacy z=27 ladder. The long-LDPC z=81 path produces
 * frames with ~3x more info bytes per CW (e.g. R3/4 cw=2: 96 B legacy -> 345 B at z=81).
 * Pass lifting_z=27 for legacy; pass 81 for the long-LDPC data path so the chunker
 * fills the full frame instead of zero-padding 70% of every burst.
 */
inline size_t getFixedFramePayloadCapacityZ(CodeRate rate, int cw_count, int lifting_z) {
    cw_count = sanitizeFixedFrameCodewords(cw_count);
    size_t total_info_bytes = static_cast<size_t>(cw_count) * getBytesPerCodewordZ(rate, lifting_z);
    return total_info_bytes > FIXED_FRAME_OVERHEAD
        ? total_info_bytes - FIXED_FRAME_OVERHEAD
        : 0;
}

/**
 * Get payload capacity for a variable LDPC data frame at a target CW count.
 *
 * MC-DPSK DATA frames use the legacy variable-codeword split where CW0 carries
 * raw frame bytes and each continuation CW reserves DATA_CW_HEADER_SIZE bytes
 * for marker/index recovery. This is intentionally different from OFDM fixed
 * frames, which pack every CW as payload-bearing info bytes before frame-level
 * interleaving.
 */
inline size_t getVariableFramePayloadCapacity(CodeRate rate, int cw_count) {
    cw_count = sanitizeFixedFrameCodewords(cw_count);
    const size_t bytes_per_cw = getBytesPerCodeword(rate);
    const size_t continuation_payload =
        bytes_per_cw > DATA_CW_HEADER_SIZE ? bytes_per_cw - DATA_CW_HEADER_SIZE : 0;
    const size_t total_info_bytes =
        bytes_per_cw + static_cast<size_t>(cw_count - 1) * continuation_payload;
    return total_info_bytes > FIXED_FRAME_OVERHEAD
        ? total_info_bytes - FIXED_FRAME_OVERHEAD
        : 0;
}

/**
 * Backward-compatible default fixed-frame capacity (4 CWs).
 */
inline size_t getFixedFramePayloadCapacity(CodeRate rate) {
    return getFixedFramePayloadCapacity(rate, kDefaultFixedFrameCodewords);
}

/**
 * Encode a data frame with fixed-CW structure and frame-level interleaving.
 *
 * Steps:
 * 1. Serialize frame (header + payload + CRC)
 * 2. Pad to exactly N codewords worth of info bytes
 * 3. LDPC encode each codeword
 * 4. Optionally channel interleave each codeword (for fading resistance)
 * 5. Interleave coded bits across all N CWs
 *
 * @param frame_data Serialized frame data (from DataFrame::serialize())
 * @param rate Code rate for LDPC encoding
 * @param use_channel_interleave If true, apply channel interleaving within each CW
 * @param bits_per_symbol Bits per OFDM symbol (data_carriers × bits_per_carrier) for interleaver geometry
 * @return Interleaved coded bits (N × 648 bits)
 */
Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate, int cw_count,
                       bool use_channel_interleave, size_t bits_per_symbol = 106,
                       int lifting_z = 27);  // 27 -> n=648 (default), 81 -> n=1944 (file class)
Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate, bool use_channel_interleave, size_t bits_per_symbol = 106);

/**
 * OFDM DATA-frame FEC selection (2026-11-08, Stage 1 of the open-modem technique port).
 *
 * LDPC: today's N independent 648-bit (or 1944-bit) codewords per frame.
 * TURBO: ONE turbo codeword over the whole frame (fec/turbo_codec.hpp). The air
 *        geometry is identical (cw x 648 coded bits, same symbols/frame, same payload
 *        capacity); only the code changes. Applies to OFDM data frames ONLY — control
 *        frames, BURST_HEADER, ACK, DATA_REPAIR and the MC-DPSK fixed CONNECT stay LDPC.
 *
 * Lockstep knob ULTRA_DATA_FEC=ldpc|turbo (read once; BOTH stations must agree —
 * there is no negotiation by design decision).
 */
enum class DataFrameFec : uint8_t { LDPC = 0, TURBO = 1 };
DataFrameFec ofdmDataFrameFec();

/** Pads frame_data to total_info_bytes with the deterministic fixed-frame PRBS. */
Bytes padFixedFrameInfo(const Bytes& frame_data, size_t total_info_bytes);

/** Turbo encode of an OFDM data frame: returns cw x (24*Z) coded bits, air order. */
Bytes encodeTurboFixedFrame(const Bytes& frame_data, CodeRate rate, int cw_count, int lifting_z = 27);

/**
 * Turbo decode of an OFDM data frame. CodewordStatus contract (vectors have length
 * cw so every LDPC-era consumer keeps its indexing): success => decoded all true,
 * data[0] = the full de-whitened info bytes, data[1..] empty; failure => all false.
 * iterations[] carries the turbo iteration count, unsatisfied_checks[] = 0 on
 * success / 1 on failure.
 */
CodewordStatus decodeTurboFixedFrame(const std::vector<float>& air_soft, CodeRate rate,
                                     int cw_count, int lifting_z = 27,
                                     Bytes* hard_info_bytes = nullptr);

/**
 * Encode without channel interleaving for an explicit CW count.
 */
Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate, int cw_count);

/**
 * Encode without channel interleaving (backward compatible).
 */
Bytes encodeFixedFrame(const Bytes& frame_data, CodeRate rate);

/**
 * Decode a fixed-CW frame with frame-level deinterleaving.
 *
 * Steps:
 * 1. Deinterleave soft bits to restore original CW order (frame-level)
 * 2. Optionally channel deinterleave each CW (restore within-CW order)
 * 3. LDPC decode each codeword
 * 4. Reassemble into frame data
 *
 * @param interleaved_soft Soft bits from demodulator (2592 floats)
 * @param rate Code rate for LDPC decoding
 * @param use_channel_deinterleave If true, apply channel deinterleaving within each CW
 * @param bits_per_symbol Bits per OFDM symbol (data_carriers × bits_per_carrier) for interleaver geometry
 * @return CodewordStatus with decode results for all configured CWs
 */
CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate, int cw_count,
                                bool use_channel_deinterleave, size_t bits_per_symbol = 106,
                                fec::SoftCombineBuffer* harq_buffer = nullptr,
                                const fec::SoftCombineBuffer::Key* harq_key = nullptr,
                                int lifting_z = 27,  // 27 -> n=648 (default), 81 -> n=1944 (file class)
                                // harq_key was position-PREDICTED (CW0 undecodable), not
                                // header-verified: retains are tagged provisional, and a
                                // successful decode whose full header identity
                                // contradicts the key skips finalize entirely (never
                                // drop/pollute another destination or seq's accumulation).
                                bool harq_key_provisional = false);
CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate, bool use_channel_deinterleave, size_t bits_per_symbol = 106);

/**
 * Decode without channel deinterleaving for an explicit CW count.
 */
CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate, int cw_count);

/**
 * Decode without channel deinterleaving (backward compatible).
 */
CodewordStatus decodeFixedFrame(const std::vector<float>& interleaved_soft, CodeRate rate);

/**
 * Create a fixed-size data frame.
 *
 * If payload is smaller than capacity, it's zero-padded.
 * If payload is larger, it's truncated (caller should chunk).
 *
 * @param src Source callsign
 * @param dst Destination callsign
 * @param seq Sequence number
 * @param payload Payload bytes
 * @param rate Code rate (determines capacity)
 * @param cw_count Codewords per fixed frame (default 4)
 * @return DataFrame with total_cw set to the fixed-frame CW count
 */
DataFrame makeFixedDataFrame(const std::string& src, const std::string& dst,
                              uint16_t seq, const Bytes& payload, CodeRate rate,
                              int cw_count = kDefaultFixedFrameCodewords,
                              int lifting_z = 27);  // 27 -> n=648, 81 -> n=1944

} // namespace v2

// Bring v2 types into protocol namespace for convenience
using FrameType = v2::FrameType;
using ControlFrame = v2::ControlFrame;
using DataFrame = v2::DataFrame;

} // namespace protocol
} // namespace ultra
