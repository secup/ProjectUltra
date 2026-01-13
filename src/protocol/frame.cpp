#include "frame.hpp"
#include <algorithm>
#include <cctype>
#include <cstdio>

namespace ultra {
namespace protocol {

const char* frameTypeToString(FrameType type) {
    switch (type) {
        case FrameType::CONNECT:     return "CONNECT";
        case FrameType::CONNECT_ACK: return "CONNECT_ACK";
        case FrameType::CONNECT_NAK: return "CONNECT_NAK";
        case FrameType::DISCONNECT:  return "DISCONNECT";
        case FrameType::DATA:        return "DATA";
        case FrameType::ACK:         return "ACK";
        case FrameType::NAK:         return "NAK";
        case FrameType::BEACON:      return "BEACON";
        default:                     return "UNKNOWN";
    }
}

// CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)
// Used in many radio protocols (AX.25, etc.)
uint16_t Frame::calculateCRC(const uint8_t* data, size_t len) {
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

std::string Frame::sanitizeCallsign(const std::string& call) {
    std::string result;
    result.reserve(CALLSIGN_LEN);

    for (char c : call) {
        if (result.size() >= CALLSIGN_LEN) break;

        // Convert to uppercase
        c = std::toupper(static_cast<unsigned char>(c));

        // Only allow A-Z, 0-9, /
        if ((c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') ||
            c == '/') {
            result += c;
        }
    }

    return result;
}

bool Frame::isValidCallsign(const std::string& call) {
    if (call.empty() || call.size() > CALLSIGN_LEN) {
        return false;
    }

    // Special case: "CQ" is always valid
    if (call == "CQ") return true;

    // Must have at least one letter and one digit for amateur callsigns
    bool has_letter = false;
    bool has_digit = false;

    for (char c : call) {
        if (c >= 'A' && c <= 'Z') has_letter = true;
        else if (c >= '0' && c <= '9') has_digit = true;
        else if (c != '/') return false;  // Invalid character
    }

    return has_letter && has_digit;
}

// Factory methods

Frame Frame::makeData(const std::string& src, const std::string& dst,
                      uint8_t seq, const Bytes& data) {
    Frame f;
    f.type = FrameType::DATA;
    f.flags = FrameFlags::NONE;
    f.sequence = seq;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    f.payload = data;

    // Truncate if too large
    if (f.payload.size() > MAX_PAYLOAD) {
        f.payload.resize(MAX_PAYLOAD);
    }

    return f;
}

Frame Frame::makeData(const std::string& src, const std::string& dst,
                      uint8_t seq, const std::string& text) {
    Bytes data(text.begin(), text.end());
    return makeData(src, dst, seq, data);
}

Frame Frame::makeAck(const std::string& src, const std::string& dst, uint8_t seq) {
    Frame f;
    f.type = FrameType::ACK;
    f.flags = FrameFlags::NONE;
    f.sequence = seq;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    return f;
}

Frame Frame::makeNak(const std::string& src, const std::string& dst, uint8_t seq) {
    Frame f;
    f.type = FrameType::NAK;
    f.flags = FrameFlags::NONE;
    f.sequence = seq;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    return f;
}

Frame Frame::makeConnect(const std::string& src, const std::string& dst) {
    Frame f;
    f.type = FrameType::CONNECT;
    f.flags = FrameFlags::NONE;
    f.sequence = 0;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    return f;
}

Frame Frame::makeConnectAck(const std::string& src, const std::string& dst) {
    Frame f;
    f.type = FrameType::CONNECT_ACK;
    f.flags = FrameFlags::NONE;
    f.sequence = 0;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    return f;
}

Frame Frame::makeConnectNak(const std::string& src, const std::string& dst) {
    Frame f;
    f.type = FrameType::CONNECT_NAK;
    f.flags = FrameFlags::NONE;
    f.sequence = 0;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    return f;
}

Frame Frame::makeDisconnect(const std::string& src, const std::string& dst) {
    Frame f;
    f.type = FrameType::DISCONNECT;
    f.flags = FrameFlags::NONE;
    f.sequence = 0;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = sanitizeCallsign(dst);
    return f;
}

Frame Frame::makeBeacon(const std::string& src, const std::string& info) {
    Frame f;
    f.type = FrameType::BEACON;
    f.flags = FrameFlags::NONE;
    f.sequence = 0;
    f.src_call = sanitizeCallsign(src);
    f.dst_call = "CQ";
    if (!info.empty()) {
        f.payload.assign(info.begin(), info.end());
        if (f.payload.size() > MAX_PAYLOAD) {
            f.payload.resize(MAX_PAYLOAD);
        }
    }
    return f;
}

Bytes Frame::serialize() const {
    Bytes result;
    result.reserve(HEADER_SIZE + payload.size() + CRC_SIZE);

    // MAGIC (1 byte)
    result.push_back(MAGIC);

    // TYPE (1 byte)
    result.push_back(static_cast<uint8_t>(type));

    // FLAGS (1 byte)
    result.push_back(flags);

    // SEQ (1 byte)
    result.push_back(sequence);

    // SRC_CALL (8 bytes, null-padded)
    for (size_t i = 0; i < CALLSIGN_LEN; i++) {
        result.push_back(i < src_call.size() ? src_call[i] : 0);
    }

    // DST_CALL (8 bytes, null-padded)
    for (size_t i = 0; i < CALLSIGN_LEN; i++) {
        result.push_back(i < dst_call.size() ? dst_call[i] : 0);
    }

    // LEN (2 bytes, big-endian)
    uint16_t len = static_cast<uint16_t>(std::min(payload.size(), MAX_PAYLOAD));
    result.push_back((len >> 8) & 0xFF);
    result.push_back(len & 0xFF);

    // DATA (N bytes)
    result.insert(result.end(), payload.begin(), payload.begin() + len);

    // CRC16 (2 bytes, big-endian) - calculated over everything before CRC
    uint16_t crc = calculateCRC(result.data(), result.size());
    result.push_back((crc >> 8) & 0xFF);
    result.push_back(crc & 0xFF);

    return result;
}

std::optional<Frame> Frame::deserialize(ByteSpan data) {
    // Check minimum size
    if (data.size() < MIN_SIZE) {
        return std::nullopt;
    }

    size_t pos = 0;

    // MAGIC
    if (data[pos++] != MAGIC) {
        return std::nullopt;
    }

    // TYPE
    uint8_t type_byte = data[pos++];
    if (type_byte < static_cast<uint8_t>(FrameType::CONNECT) ||
        type_byte > static_cast<uint8_t>(FrameType::BEACON)) {
        return std::nullopt;
    }

    Frame frame;
    frame.type = static_cast<FrameType>(type_byte);

    // FLAGS
    frame.flags = data[pos++];

    // SEQ
    frame.sequence = data[pos++];

    // SRC_CALL (8 bytes)
    frame.src_call.clear();
    for (size_t i = 0; i < CALLSIGN_LEN; i++) {
        char c = static_cast<char>(data[pos++]);
        if (c != 0) frame.src_call += c;
    }

    // DST_CALL (8 bytes)
    frame.dst_call.clear();
    for (size_t i = 0; i < CALLSIGN_LEN; i++) {
        char c = static_cast<char>(data[pos++]);
        if (c != 0) frame.dst_call += c;
    }

    // LEN (2 bytes, big-endian)
    uint16_t len = (static_cast<uint16_t>(data[pos]) << 8) |
                    static_cast<uint16_t>(data[pos + 1]);
    pos += 2;

    // Validate length
    if (len > MAX_PAYLOAD) {
        return std::nullopt;
    }

    // Check we have enough data for payload + CRC
    if (data.size() < HEADER_SIZE + len + CRC_SIZE) {
        return std::nullopt;
    }

    // DATA
    frame.payload.assign(data.begin() + pos, data.begin() + pos + len);
    pos += len;

    // CRC16 (2 bytes, big-endian)
    uint16_t received_crc = (static_cast<uint16_t>(data[pos]) << 8) |
                             static_cast<uint16_t>(data[pos + 1]);

    // Calculate CRC over header + payload (everything before CRC)
    uint16_t calculated_crc = calculateCRC(data.data(), pos);

    if (received_crc != calculated_crc) {
        return std::nullopt;  // CRC mismatch
    }

    return frame;
}

std::string Frame::payloadAsText() const {
    std::string text;
    text.reserve(payload.size());
    for (uint8_t b : payload) {
        if (b >= 32 && b < 127) {
            text += static_cast<char>(b);
        } else if (b == '\n' || b == '\r' || b == '\t') {
            text += static_cast<char>(b);
        }
        // Skip non-printable characters
    }
    return text;
}

bool Frame::isControl() const {
    switch (type) {
        case FrameType::CONNECT:
        case FrameType::CONNECT_ACK:
        case FrameType::CONNECT_NAK:
        case FrameType::DISCONNECT:
        case FrameType::ACK:
        case FrameType::NAK:
            return true;
        default:
            return false;
    }
}

bool Frame::isForCallsign(const std::string& call) const {
    // Broadcast frames are for everyone
    if (dst_call == "CQ") return true;

    // Exact match (case-insensitive since we sanitize to uppercase)
    return dst_call == sanitizeCallsign(call);
}

std::string frameToString(const Frame& frame) {
    char buf[256];
    snprintf(buf, sizeof(buf), "Frame{type=%s seq=%d src=%s dst=%s payload=%zu bytes}",
             frameTypeToString(frame.type),
             frame.sequence,
             frame.src_call.c_str(),
             frame.dst_call.c_str(),
             frame.payload.size());
    return buf;
}

} // namespace protocol
} // namespace ultra
