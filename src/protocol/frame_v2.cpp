#include "frame_v2.hpp"
#include "ultra/fec.hpp"  // LDPC encoder/decoder
#include <cstring>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace ultra {
namespace protocol {

// ============================================================================
// Shared Protocol Types Implementation
// ============================================================================

const char* waveformModeToString(WaveformMode mode) {
    switch (mode) {
        case WaveformMode::OFDM:     return "OFDM";
        case WaveformMode::OTFS_EQ:  return "OTFS-EQ";
        case WaveformMode::OTFS_RAW: return "OTFS-RAW";
        case WaveformMode::AUTO:     return "AUTO";
        default:                     return "UNKNOWN";
    }
}

Bytes ChannelReport::encode() const {
    Bytes data(5);
    // SNR: 0-50 dB mapped to 0-250 (0.2 dB resolution)
    data[0] = static_cast<uint8_t>(std::min(250.0f, std::max(0.0f, snr_db * 5.0f)));
    // Delay spread: 0-25 ms mapped to 0-250 (0.1 ms resolution)
    data[1] = static_cast<uint8_t>(std::min(250.0f, std::max(0.0f, delay_spread_ms * 10.0f)));
    // Doppler: 0-25 Hz mapped to 0-250 (0.1 Hz resolution)
    data[2] = static_cast<uint8_t>(std::min(250.0f, std::max(0.0f, doppler_spread_hz * 10.0f)));
    // Recommended mode
    data[3] = static_cast<uint8_t>(recommended_mode);
    // Capabilities
    data[4] = capabilities;
    return data;
}

ChannelReport ChannelReport::decode(const Bytes& data) {
    ChannelReport report;
    if (data.size() >= 5) {
        report.snr_db = static_cast<float>(data[0]) / 5.0f;
        report.delay_spread_ms = static_cast<float>(data[1]) / 10.0f;
        report.doppler_spread_hz = static_cast<float>(data[2]) / 10.0f;
        report.recommended_mode = static_cast<WaveformMode>(data[3]);
        report.capabilities = data[4];
    }
    return report;
}

const char* ChannelReport::getConditionName() const {
    if (snr_db >= 25.0f && delay_spread_ms < 1.0f && doppler_spread_hz < 1.0f) {
        return "Excellent";
    } else if (snr_db >= 18.0f && delay_spread_ms < 2.0f && doppler_spread_hz < 2.0f) {
        return "Good";
    } else if (snr_db >= 10.0f) {
        return "Moderate";
    } else if (snr_db >= 3.0f) {
        return "Poor";
    } else {
        return "Flutter";
    }
}

namespace v2 {

// ============================================================================
// Callsign hashing (DJB2, 24-bit)
// ============================================================================
uint32_t hashCallsign(const std::string& callsign) {
    uint32_t hash = 5381;
    for (char c : callsign) {
        hash = ((hash << 5) + hash) ^ static_cast<uint8_t>(std::toupper(c));
    }
    return hash & 0xFFFFFF;  // 24 bits
}

// ============================================================================
// Frame type to string
// ============================================================================
const char* frameTypeToString(FrameType type) {
    switch (type) {
        case FrameType::PROBE:       return "PROBE";
        case FrameType::PROBE_ACK:   return "PROBE_ACK";
        case FrameType::CONNECT:     return "CONNECT";
        case FrameType::CONNECT_ACK: return "CONNECT_ACK";
        case FrameType::CONNECT_NAK: return "CONNECT_NAK";
        case FrameType::DISCONNECT:  return "DISCONNECT";
        case FrameType::KEEPALIVE:   return "KEEPALIVE";
        case FrameType::ACK:         return "ACK";
        case FrameType::NACK:        return "NACK";
        case FrameType::BEACON:      return "BEACON";
        case FrameType::DATA:        return "DATA";
        case FrameType::DATA_START:  return "DATA_START";
        case FrameType::DATA_CONT:   return "DATA_CONT";
        case FrameType::DATA_END:    return "DATA_END";
        default:                     return "UNKNOWN";
    }
}

// ============================================================================
// CRC-16 CCITT (same as v1 for compatibility)
// ============================================================================
uint16_t ControlFrame::calculateCRC(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// ControlFrame implementation
// ============================================================================

ControlFrame ControlFrame::makeProbe(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::PROBE;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeProbeAck(const std::string& src, const std::string& dst,
                                         uint8_t snr_db, uint8_t recommended_rate) {
    ControlFrame f;
    f.type = FrameType::PROBE_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = snr_db;
    f.payload[1] = recommended_rate;
    return f;
}

ControlFrame ControlFrame::makeAck(const std::string& src, const std::string& dst, uint16_t seq) {
    ControlFrame f;
    f.type = FrameType::ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeNack(const std::string& src, const std::string& dst,
                                     uint16_t seq, uint32_t cw_bitmap) {
    ControlFrame f;
    f.type = FrameType::NACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    NackPayload np;
    np.frame_seq = seq;
    np.cw_bitmap = cw_bitmap;
    np.encode(f.payload);

    return f;
}

ControlFrame ControlFrame::makeBeacon(const std::string& src) {
    ControlFrame f;
    f.type = FrameType::BEACON;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = 0xFFFFFF;  // Broadcast
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeKeepalive(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::KEEPALIVE;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeDisconnect(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::DISCONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeConnect(const std::string& src, const std::string& dst,
                                        uint8_t mode_capabilities, uint8_t preferred_mode) {
    ControlFrame f;
    f.type = FrameType::CONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = mode_capabilities;  // Our supported modes
    f.payload[1] = preferred_mode;     // Our preferred mode
    return f;
}

ControlFrame ControlFrame::makeConnectAck(const std::string& src, const std::string& dst,
                                           uint8_t negotiated_mode) {
    ControlFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = negotiated_mode;
    return f;
}

ControlFrame ControlFrame::makeConnectNak(const std::string& src, const std::string& dst) {
    ControlFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

// Hash-based factory methods (for responding when callsign is unknown)
ControlFrame ControlFrame::makeProbeAckByHash(const std::string& src, uint32_t dst_hash,
                                               uint8_t snr_db, uint8_t recommended_rate) {
    ControlFrame f;
    f.type = FrameType::PROBE_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;  // Ensure 24-bit
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = snr_db;
    f.payload[1] = recommended_rate;
    return f;
}

ControlFrame ControlFrame::makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                                 uint8_t negotiated_mode) {
    ControlFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    f.payload[0] = negotiated_mode;
    return f;
}

ControlFrame ControlFrame::makeConnectNakByHash(const std::string& src, uint32_t dst_hash) {
    ControlFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeAckByHash(const std::string& src, uint32_t dst_hash, uint16_t seq) {
    ControlFrame f;
    f.type = FrameType::ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;
    std::memset(f.payload, 0, PAYLOAD_SIZE);
    return f;
}

ControlFrame ControlFrame::makeNackByHash(const std::string& src, uint32_t dst_hash,
                                           uint16_t seq, uint32_t cw_bitmap) {
    ControlFrame f;
    f.type = FrameType::NACK;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;

    NackPayload np;
    np.frame_seq = seq;
    np.cw_bitmap = cw_bitmap;
    np.encode(f.payload);

    return f;
}

Bytes ControlFrame::serialize() const {
    Bytes out(SIZE);

    // Magic (2 bytes, big-endian)
    out[0] = (MAGIC_V2 >> 8) & 0xFF;
    out[1] = MAGIC_V2 & 0xFF;

    // Type (1 byte)
    out[2] = static_cast<uint8_t>(type);

    // Flags (1 byte)
    out[3] = flags;

    // Sequence (2 bytes, big-endian)
    out[4] = (seq >> 8) & 0xFF;
    out[5] = seq & 0xFF;

    // Source hash (3 bytes, big-endian)
    out[6] = (src_hash >> 16) & 0xFF;
    out[7] = (src_hash >> 8) & 0xFF;
    out[8] = src_hash & 0xFF;

    // Destination hash (3 bytes, big-endian)
    out[9] = (dst_hash >> 16) & 0xFF;
    out[10] = (dst_hash >> 8) & 0xFF;
    out[11] = dst_hash & 0xFF;

    // Payload (6 bytes)
    std::memcpy(out.data() + 12, payload, PAYLOAD_SIZE);

    // CRC16 (2 bytes, big-endian) - over bytes 0-17
    uint16_t crc = calculateCRC(out.data(), SIZE - 2);
    out[18] = (crc >> 8) & 0xFF;
    out[19] = crc & 0xFF;

    return out;
}

std::optional<ControlFrame> ControlFrame::deserialize(ByteSpan data) {
    if (data.size() < SIZE) {
        return std::nullopt;
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (magic != MAGIC_V2) {
        return std::nullopt;
    }

    // Verify CRC
    uint16_t received_crc = (static_cast<uint16_t>(data[18]) << 8) | data[19];
    uint16_t calculated_crc = calculateCRC(data.data(), SIZE - 2);
    if (received_crc != calculated_crc) {
        return std::nullopt;
    }

    ControlFrame f;
    f.type = static_cast<FrameType>(data[2]);
    f.flags = data[3];
    f.seq = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    f.src_hash = (static_cast<uint32_t>(data[6]) << 16) |
                 (static_cast<uint32_t>(data[7]) << 8) |
                 data[8];
    f.dst_hash = (static_cast<uint32_t>(data[9]) << 16) |
                 (static_cast<uint32_t>(data[10]) << 8) |
                 data[11];
    std::memcpy(f.payload, data.data() + 12, PAYLOAD_SIZE);

    return f;
}

// ============================================================================
// DataFrame implementation
// ============================================================================

uint8_t DataFrame::calculateCodewords(size_t payload_size) {
    // Total frame size = header (17) + payload + frame_CRC (2)
    size_t total = HEADER_SIZE + payload_size + CRC_SIZE;

    // CW0 carries 20 bytes of raw frame data
    if (total <= BYTES_PER_CODEWORD) {
        return 1;
    }

    // Remaining bytes after CW0
    size_t remaining = total - BYTES_PER_CODEWORD;

    // Each CW1+ carries DATA_CW_PAYLOAD_SIZE (18) bytes due to marker + index overhead
    size_t additional_cws = (remaining + DATA_CW_PAYLOAD_SIZE - 1) / DATA_CW_PAYLOAD_SIZE;

    return static_cast<uint8_t>(1 + additional_cws);
}

DataFrame DataFrame::makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const Bytes& data) {
    DataFrame f;
    f.type = FrameType::DATA;
    f.flags = Flags::VERSION_V2;
    f.seq = seq;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);
    f.payload = data;
    f.payload_len = static_cast<uint16_t>(data.size());
    f.total_cw = calculateCodewords(data.size());
    return f;
}

DataFrame DataFrame::makeData(const std::string& src, const std::string& dst,
                               uint16_t seq, const std::string& text) {
    Bytes data(text.begin(), text.end());
    return makeData(src, dst, seq, data);
}

Bytes DataFrame::serialize() const {
    // Total size = header + payload + CRC
    size_t total_size = HEADER_SIZE + payload.size() + CRC_SIZE;
    Bytes out(total_size);

    // Magic (2 bytes, big-endian)
    out[0] = (MAGIC_V2 >> 8) & 0xFF;
    out[1] = MAGIC_V2 & 0xFF;

    // Type (1 byte)
    out[2] = static_cast<uint8_t>(type);

    // Flags (1 byte)
    out[3] = flags;

    // Sequence (2 bytes, big-endian)
    out[4] = (seq >> 8) & 0xFF;
    out[5] = seq & 0xFF;

    // Source hash (3 bytes, big-endian)
    out[6] = (src_hash >> 16) & 0xFF;
    out[7] = (src_hash >> 8) & 0xFF;
    out[8] = src_hash & 0xFF;

    // Destination hash (3 bytes, big-endian)
    out[9] = (dst_hash >> 16) & 0xFF;
    out[10] = (dst_hash >> 8) & 0xFF;
    out[11] = dst_hash & 0xFF;

    // Total codewords (1 byte)
    out[12] = total_cw;

    // Payload length (2 bytes, big-endian)
    out[13] = (payload_len >> 8) & 0xFF;
    out[14] = payload_len & 0xFF;

    // Header CRC (2 bytes) - CRC of bytes 0-14
    uint16_t hcrc = ControlFrame::calculateCRC(out.data(), 15);
    out[15] = (hcrc >> 8) & 0xFF;
    out[16] = hcrc & 0xFF;

    // Payload
    if (!payload.empty()) {
        std::memcpy(out.data() + HEADER_SIZE, payload.data(), payload.size());
    }

    // Frame CRC (2 bytes) - CRC of entire frame except last 2 bytes
    uint16_t fcrc = ControlFrame::calculateCRC(out.data(), total_size - 2);
    out[total_size - 2] = (fcrc >> 8) & 0xFF;
    out[total_size - 1] = fcrc & 0xFF;

    return out;
}

std::optional<DataFrame> DataFrame::deserialize(ByteSpan data) {
    if (data.size() < HEADER_SIZE + CRC_SIZE) {
        return std::nullopt;
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (magic != MAGIC_V2) {
        return std::nullopt;
    }

    // Verify header CRC
    uint16_t received_hcrc = (static_cast<uint16_t>(data[15]) << 8) | data[16];
    uint16_t calculated_hcrc = ControlFrame::calculateCRC(data.data(), 15);
    if (received_hcrc != calculated_hcrc) {
        return std::nullopt;
    }

    // Parse header
    DataFrame f;
    f.type = static_cast<FrameType>(data[2]);
    f.flags = data[3];
    f.seq = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    f.src_hash = (static_cast<uint32_t>(data[6]) << 16) |
                 (static_cast<uint32_t>(data[7]) << 8) |
                 data[8];
    f.dst_hash = (static_cast<uint32_t>(data[9]) << 16) |
                 (static_cast<uint32_t>(data[10]) << 8) |
                 data[11];
    f.total_cw = data[12];
    f.payload_len = (static_cast<uint16_t>(data[13]) << 8) | data[14];

    // Check we have enough data
    size_t expected_size = HEADER_SIZE + f.payload_len + CRC_SIZE;
    if (data.size() < expected_size) {
        return std::nullopt;
    }

    // Verify frame CRC
    uint16_t received_fcrc = (static_cast<uint16_t>(data[expected_size - 2]) << 8) |
                              data[expected_size - 1];
    uint16_t calculated_fcrc = ControlFrame::calculateCRC(data.data(), expected_size - 2);
    if (received_fcrc != calculated_fcrc) {
        return std::nullopt;
    }

    // Extract payload
    if (f.payload_len > 0) {
        f.payload.assign(data.begin() + HEADER_SIZE, data.begin() + HEADER_SIZE + f.payload_len);
    }

    return f;
}

std::string DataFrame::payloadAsText() const {
    return std::string(payload.begin(), payload.end());
}

// ============================================================================
// ConnectFrame implementation (ham-compliant with full callsigns)
// ============================================================================

ConnectFrame ConnectFrame::makeConnect(const std::string& src, const std::string& dst,
                                        uint8_t mode_caps, uint8_t pref_mode) {
    ConnectFrame f;
    f.type = FrameType::CONNECT;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    // Copy callsigns (null-terminated, max 9 chars)
    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';
    std::strncpy(f.dst_callsign, dst.c_str(), MAX_CALLSIGN_LEN - 1);
    f.dst_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = mode_caps;
    f.negotiated_mode = pref_mode;
    return f;
}

ConnectFrame ConnectFrame::makeConnectAck(const std::string& src, const std::string& dst,
                                           uint8_t neg_mode) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';
    std::strncpy(f.dst_callsign, dst.c_str(), MAX_CALLSIGN_LEN - 1);
    f.dst_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = 0;  // Not used in ACK
    f.negotiated_mode = neg_mode;
    return f;
}

ConnectFrame ConnectFrame::makeConnectNak(const std::string& src, const std::string& dst) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = hashCallsign(dst);

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';
    std::strncpy(f.dst_callsign, dst.c_str(), MAX_CALLSIGN_LEN - 1);
    f.dst_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = 0;
    f.negotiated_mode = 0;
    return f;
}

ConnectFrame ConnectFrame::makeConnectAckByHash(const std::string& src, uint32_t dst_hash,
                                                 uint8_t neg_mode) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';
    // dst_callsign unknown - leave empty (will be filled from received CONNECT)
    f.dst_callsign[0] = '\0';

    f.mode_capabilities = 0;
    f.negotiated_mode = neg_mode;
    return f;
}

ConnectFrame ConnectFrame::makeConnectNakByHash(const std::string& src, uint32_t dst_hash) {
    ConnectFrame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = Flags::VERSION_V2;
    f.seq = 0;
    f.src_hash = hashCallsign(src);
    f.dst_hash = dst_hash & 0xFFFFFF;

    std::strncpy(f.src_callsign, src.c_str(), MAX_CALLSIGN_LEN - 1);
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';
    f.dst_callsign[0] = '\0';

    f.mode_capabilities = 0;
    f.negotiated_mode = 0;
    return f;
}

Bytes ConnectFrame::serialize() const {
    // Use DATA frame format: header (17B) + payload (22B) + CRC (2B) = 41 bytes
    Bytes out;
    out.reserve(DataFrame::HEADER_SIZE + PAYLOAD_SIZE + DataFrame::CRC_SIZE);

    // Magic (2 bytes)
    out.push_back((MAGIC_V2 >> 8) & 0xFF);
    out.push_back(MAGIC_V2 & 0xFF);

    // Type, flags, seq (4 bytes)
    out.push_back(static_cast<uint8_t>(type));
    out.push_back(flags);
    out.push_back((seq >> 8) & 0xFF);
    out.push_back(seq & 0xFF);

    // Hashes (6 bytes)
    out.push_back((src_hash >> 16) & 0xFF);
    out.push_back((src_hash >> 8) & 0xFF);
    out.push_back(src_hash & 0xFF);
    out.push_back((dst_hash >> 16) & 0xFF);
    out.push_back((dst_hash >> 8) & 0xFF);
    out.push_back(dst_hash & 0xFF);

    // Total codewords (1 byte) - 41 bytes = 3 codewords
    uint8_t total_cw = DataFrame::calculateCodewords(PAYLOAD_SIZE);
    out.push_back(total_cw);

    // Payload length (2 bytes)
    out.push_back((PAYLOAD_SIZE >> 8) & 0xFF);
    out.push_back(PAYLOAD_SIZE & 0xFF);

    // Header CRC (2 bytes)
    uint16_t hcrc = ControlFrame::calculateCRC(out.data(), out.size());
    out.push_back((hcrc >> 8) & 0xFF);
    out.push_back(hcrc & 0xFF);

    // Payload: src_callsign (10B) + dst_callsign (10B) + caps (1B) + mode (1B)
    for (int i = 0; i < MAX_CALLSIGN_LEN; i++) {
        out.push_back(static_cast<uint8_t>(src_callsign[i]));
    }
    for (int i = 0; i < MAX_CALLSIGN_LEN; i++) {
        out.push_back(static_cast<uint8_t>(dst_callsign[i]));
    }
    out.push_back(mode_capabilities);
    out.push_back(negotiated_mode);

    // Frame CRC (2 bytes)
    uint16_t fcrc = ControlFrame::calculateCRC(out.data(), out.size());
    out.push_back((fcrc >> 8) & 0xFF);
    out.push_back(fcrc & 0xFF);

    return out;
}

std::optional<ConnectFrame> ConnectFrame::deserialize(ByteSpan data) {
    constexpr size_t MIN_SIZE = DataFrame::HEADER_SIZE + PAYLOAD_SIZE + DataFrame::CRC_SIZE;
    if (data.size() < MIN_SIZE) {
        return std::nullopt;
    }

    // Verify magic
    uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    if (magic != MAGIC_V2) {
        return std::nullopt;
    }

    // Check type is CONNECT, CONNECT_ACK, or CONNECT_NAK
    FrameType ftype = static_cast<FrameType>(data[2]);
    if (ftype != FrameType::CONNECT && ftype != FrameType::CONNECT_ACK &&
        ftype != FrameType::CONNECT_NAK) {
        return std::nullopt;
    }

    // Verify header CRC
    uint16_t stored_hcrc = (static_cast<uint16_t>(data[15]) << 8) | data[16];
    uint16_t calc_hcrc = ControlFrame::calculateCRC(data.data(), 15);
    if (stored_hcrc != calc_hcrc) {
        return std::nullopt;
    }

    // Verify frame CRC
    size_t fcrc_offset = DataFrame::HEADER_SIZE + PAYLOAD_SIZE;
    uint16_t stored_fcrc = (static_cast<uint16_t>(data[fcrc_offset]) << 8) | data[fcrc_offset + 1];
    uint16_t calc_fcrc = ControlFrame::calculateCRC(data.data(), fcrc_offset);
    if (stored_fcrc != calc_fcrc) {
        return std::nullopt;
    }

    ConnectFrame f;
    f.type = ftype;
    f.flags = data[3];
    f.seq = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    f.src_hash = (static_cast<uint32_t>(data[6]) << 16) |
                 (static_cast<uint32_t>(data[7]) << 8) |
                 data[8];
    f.dst_hash = (static_cast<uint32_t>(data[9]) << 16) |
                 (static_cast<uint32_t>(data[10]) << 8) |
                 data[11];

    // Parse payload (starts at offset 17)
    size_t payload_offset = DataFrame::HEADER_SIZE;
    for (int i = 0; i < MAX_CALLSIGN_LEN; i++) {
        f.src_callsign[i] = static_cast<char>(data[payload_offset + i]);
    }
    f.src_callsign[MAX_CALLSIGN_LEN - 1] = '\0';  // Ensure null-terminated

    for (int i = 0; i < MAX_CALLSIGN_LEN; i++) {
        f.dst_callsign[i] = static_cast<char>(data[payload_offset + MAX_CALLSIGN_LEN + i]);
    }
    f.dst_callsign[MAX_CALLSIGN_LEN - 1] = '\0';

    f.mode_capabilities = data[payload_offset + 2 * MAX_CALLSIGN_LEN];
    f.negotiated_mode = data[payload_offset + 2 * MAX_CALLSIGN_LEN + 1];

    return f;
}

std::string ConnectFrame::getSrcCallsign() const {
    return std::string(src_callsign);
}

std::string ConnectFrame::getDstCallsign() const {
    return std::string(dst_callsign);
}

// ============================================================================
// NackPayload implementation
// ============================================================================

void NackPayload::encode(uint8_t* out) const {
    // Frame sequence (2 bytes)
    out[0] = (frame_seq >> 8) & 0xFF;
    out[1] = frame_seq & 0xFF;

    // Codeword bitmap (4 bytes)
    out[2] = (cw_bitmap >> 24) & 0xFF;
    out[3] = (cw_bitmap >> 16) & 0xFF;
    out[4] = (cw_bitmap >> 8) & 0xFF;
    out[5] = cw_bitmap & 0xFF;
}

NackPayload NackPayload::decode(const uint8_t* in) {
    NackPayload np;
    np.frame_seq = (static_cast<uint16_t>(in[0]) << 8) | in[1];
    np.cw_bitmap = (static_cast<uint32_t>(in[2]) << 24) |
                   (static_cast<uint32_t>(in[3]) << 16) |
                   (static_cast<uint32_t>(in[4]) << 8) |
                   in[5];
    return np;
}

int NackPayload::countFailed() const {
    int count = 0;
    uint32_t b = cw_bitmap;
    while (b) {
        count += b & 1;
        b >>= 1;
    }
    return count;
}

// ============================================================================
// Codeword helpers
// ============================================================================

std::vector<Bytes> splitIntoCodewords(const Bytes& frame_data) {
    std::vector<Bytes> codewords;

    // CW0: First 20 bytes of frame data (contains header with 0x554C magic)
    // No modification needed - the magic already identifies it
    {
        Bytes cw0(BYTES_PER_CODEWORD, 0);  // Zero-pad if needed
        size_t cw0_data = std::min(BYTES_PER_CODEWORD, frame_data.size());
        std::memcpy(cw0.data(), frame_data.data(), cw0_data);
        codewords.push_back(std::move(cw0));
    }

    // CW1+: Add marker + index header before payload data
    size_t offset = BYTES_PER_CODEWORD;  // Start after CW0's data
    uint8_t cw_index = 1;

    while (offset < frame_data.size()) {
        Bytes cw(BYTES_PER_CODEWORD, 0);  // Zero-pad if needed

        // Add marker and index
        cw[0] = DATA_CW_MARKER;
        cw[1] = cw_index;

        // Copy payload data (up to 18 bytes)
        size_t remaining = frame_data.size() - offset;
        size_t chunk_size = std::min(DATA_CW_PAYLOAD_SIZE, remaining);
        std::memcpy(cw.data() + DATA_CW_HEADER_SIZE, frame_data.data() + offset, chunk_size);

        codewords.push_back(std::move(cw));
        offset += DATA_CW_PAYLOAD_SIZE;  // Each CW1+ consumes 18 bytes of frame data
        cw_index++;
    }

    return codewords;
}

Bytes reassembleCodewords(const std::vector<Bytes>& codewords, size_t expected_size) {
    Bytes result;
    result.reserve(expected_size);

    for (size_t i = 0; i < codewords.size(); i++) {
        size_t remaining = expected_size - result.size();
        if (remaining == 0) break;

        if (i == 0) {
            // CW0: All 20 bytes are frame data (header + payload start)
            size_t to_copy = std::min(remaining, codewords[i].size());
            result.insert(result.end(), codewords[i].begin(), codewords[i].begin() + to_copy);
        } else {
            // CW1+: Skip marker (0xD5) and index, copy payload portion
            // Verify marker byte (optional - for robustness)
            if (codewords[i].size() >= DATA_CW_HEADER_SIZE && codewords[i][0] == DATA_CW_MARKER) {
                size_t payload_size = codewords[i].size() - DATA_CW_HEADER_SIZE;
                size_t to_copy = std::min(remaining, payload_size);
                result.insert(result.end(),
                              codewords[i].begin() + DATA_CW_HEADER_SIZE,
                              codewords[i].begin() + DATA_CW_HEADER_SIZE + to_copy);
            } else {
                // Fallback: old format without marker (backward compatibility during transition)
                size_t to_copy = std::min(remaining, codewords[i].size());
                result.insert(result.end(), codewords[i].begin(), codewords[i].begin() + to_copy);
            }
        }
    }

    return result;
}

uint32_t CodewordStatus::getNackBitmap() const {
    uint32_t bitmap = 0;
    for (size_t i = 0; i < decoded.size() && i < 32; i++) {
        if (!decoded[i]) {
            bitmap |= (1u << i);
        }
    }
    return bitmap;
}

bool CodewordStatus::allSuccess() const {
    for (bool d : decoded) {
        if (!d) return false;
    }
    return true;
}

int CodewordStatus::countFailures() const {
    int count = 0;
    for (bool d : decoded) {
        if (!d) count++;
    }
    return count;
}

uint8_t CodewordStatus::getExpectedCodewords() const {
    if (decoded.empty() || !decoded[0] || data.empty() || data[0].size() < 20) {
        return 0;
    }

    // Parse header from first codeword
    auto info = parseHeader(data[0]);
    if (!info.valid) {
        return 0;
    }

    return info.total_cw;
}

Bytes CodewordStatus::reassemble() const {
    if (decoded.empty() || !decoded[0] || data.empty()) {
        return {};
    }

    // Parse header to know total size
    auto info = parseHeader(data[0]);
    if (!info.valid) {
        return {};
    }

    // Calculate expected frame size
    size_t expected_size;
    if (info.is_control) {
        expected_size = ControlFrame::SIZE;  // 20 bytes
    } else {
        expected_size = DataFrame::HEADER_SIZE + info.payload_len + DataFrame::CRC_SIZE;
    }

    // Reassemble from decoded codewords
    return reassembleCodewords(data, expected_size);
}

bool CodewordStatus::mergeCodeword(size_t index, const Bytes& cw_data) {
    if (index >= decoded.size()) {
        return false;  // Index out of range
    }

    if (decoded[index]) {
        return false;  // Already decoded, no need to merge
    }

    // Merge the retransmitted codeword
    decoded[index] = true;
    data[index] = cw_data;
    return true;
}

void CodewordStatus::initForFrame(uint8_t total_cw) {
    decoded.resize(total_cw, false);
    data.resize(total_cw);
}

// ============================================================================
// LDPC Integration Implementation
// ============================================================================

std::vector<Bytes> encodeFrameWithLDPC(const Bytes& frame_data) {
    // Split frame into 20-byte chunks
    auto chunks = splitIntoCodewords(frame_data);

    // Create LDPC encoder (R1/4)
    LDPCEncoder encoder(CodeRate::R1_4);

    std::vector<Bytes> encoded_codewords;
    encoded_codewords.reserve(chunks.size());

    for (const auto& chunk : chunks) {
        // Each chunk is 20 bytes, but LDPC R1/4 expects exactly k/8 bytes
        // k=162 bits = 20.25 bytes, so we encode 20 bytes (160 bits) + 2 bits padding
        // The encoder handles this internally
        auto encoded = encoder.encode(chunk);
        encoded_codewords.push_back(std::move(encoded));
    }

    return encoded_codewords;
}

std::pair<bool, Bytes> decodeSingleCodeword(const std::vector<float>& soft_bits) {
    if (soft_bits.size() < LDPC_CODEWORD_BITS) {
        return {false, {}};
    }

    // Create LDPC decoder (R1/4)
    LDPCDecoder decoder(CodeRate::R1_4);

    // Decode - returns k/8 bytes (rounded up)
    auto decoded = decoder.decodeSoft(soft_bits);
    bool success = decoder.lastDecodeSuccess();

    if (!success || decoded.size() < BYTES_PER_CODEWORD) {
        return {false, {}};
    }

    // Return exactly 20 bytes (the info portion)
    Bytes result(decoded.begin(), decoded.begin() + BYTES_PER_CODEWORD);
    return {true, result};
}

CodewordStatus decodeCodewordsWithLDPC(const std::vector<std::vector<float>>& soft_bits) {
    CodewordStatus status;
    status.decoded.resize(soft_bits.size(), false);
    status.data.resize(soft_bits.size());

    for (size_t i = 0; i < soft_bits.size(); i++) {
        auto [success, data] = decodeSingleCodeword(soft_bits[i]);
        status.decoded[i] = success;
        if (success) {
            status.data[i] = std::move(data);
        }
    }

    return status;
}

HeaderInfo parseHeader(const Bytes& first_codeword_data) {
    HeaderInfo info;

    if (first_codeword_data.size() < BYTES_PER_CODEWORD) {
        return info;  // invalid
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(first_codeword_data[0]) << 8) |
                      first_codeword_data[1];
    if (magic != MAGIC_V2) {
        return info;  // invalid magic
    }

    // Parse type
    info.type = static_cast<FrameType>(first_codeword_data[2]);
    info.is_control = isControlFrame(info.type);

    // Parse sequence
    info.seq = (static_cast<uint16_t>(first_codeword_data[4]) << 8) |
                first_codeword_data[5];

    // Parse hashes
    info.src_hash = (static_cast<uint32_t>(first_codeword_data[6]) << 16) |
                    (static_cast<uint32_t>(first_codeword_data[7]) << 8) |
                    first_codeword_data[8];
    info.dst_hash = (static_cast<uint32_t>(first_codeword_data[9]) << 16) |
                    (static_cast<uint32_t>(first_codeword_data[10]) << 8) |
                    first_codeword_data[11];

    if (info.is_control) {
        // Control frame: always 1 codeword, verify CRC
        uint16_t received_crc = (static_cast<uint16_t>(first_codeword_data[18]) << 8) |
                                 first_codeword_data[19];
        uint16_t calculated_crc = ControlFrame::calculateCRC(first_codeword_data.data(), 18);
        if (received_crc != calculated_crc) {
            return info;  // CRC failed
        }
        info.total_cw = 1;
        info.payload_len = 0;
    } else {
        // Data frame: read TOTAL_CW and LEN, verify header CRC
        info.total_cw = first_codeword_data[12];
        info.payload_len = (static_cast<uint16_t>(first_codeword_data[13]) << 8) |
                            first_codeword_data[14];

        uint16_t received_hcrc = (static_cast<uint16_t>(first_codeword_data[15]) << 8) |
                                  first_codeword_data[16];
        uint16_t calculated_hcrc = ControlFrame::calculateCRC(first_codeword_data.data(), 15);
        if (received_hcrc != calculated_hcrc) {
            return info;  // Header CRC failed
        }
    }

    info.valid = true;
    return info;
}

CodewordInfo identifyCodeword(const Bytes& cw_data) {
    CodewordInfo info;

    if (cw_data.size() < 2) {
        return info;  // Too short to identify
    }

    // Check for header magic (0x554C = "UL")
    uint16_t first_two = (static_cast<uint16_t>(cw_data[0]) << 8) | cw_data[1];
    if (first_two == MAGIC_V2) {
        info.type = CodewordType::HEADER;
        info.index = 0;
        return info;
    }

    // Check for data codeword marker (0xD5)
    if (cw_data[0] == DATA_CW_MARKER) {
        info.type = CodewordType::DATA;
        info.index = cw_data[1];
        return info;
    }

    // Unknown codeword type
    return info;
}

} // namespace v2
} // namespace protocol
} // namespace ultra
