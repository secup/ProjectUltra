#include "ultra/arq.hpp"
#include "ultra/modem.hpp"
#include <cstring>
#include <cmath>

namespace ultra {

namespace {

// CRC-16-CCITT for frame validation
uint16_t crc16(ByteSpan data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Frame header structure (8 bytes):
// [0]: Frame type (1 byte)
// [1-2]: Sequence number (2 bytes, big-endian)
// [3-4]: Payload length (2 bytes, big-endian)
// [5]: Modulation/coding info (1 byte)
// [6-7]: Header CRC (2 bytes)
//
// Frame structure:
// [Header 8 bytes][Payload N bytes][Payload CRC 2 bytes]

constexpr size_t HEADER_SIZE = 8;
constexpr size_t CRC_SIZE = 2;

} // anonymous namespace

FrameBuilder::FrameBuilder(const ModemConfig& config) : config_(config) {}

size_t FrameBuilder::maxPayloadSize() const {
    return config_.frame_size - HEADER_SIZE - CRC_SIZE;
}

Bytes FrameBuilder::buildDataFrame(uint16_t seq_num, ByteSpan data) {
    Bytes frame;
    frame.reserve(HEADER_SIZE + data.size() + CRC_SIZE);

    // Frame type
    frame.push_back(static_cast<uint8_t>(FrameType::DATA));

    // Sequence number (big-endian)
    frame.push_back((seq_num >> 8) & 0xFF);
    frame.push_back(seq_num & 0xFF);

    // Payload length
    uint16_t len = static_cast<uint16_t>(data.size());
    frame.push_back((len >> 8) & 0xFF);
    frame.push_back(len & 0xFF);

    // Modulation/coding info
    uint8_t mod_code = (static_cast<uint8_t>(config_.modulation) << 4) |
                       static_cast<uint8_t>(config_.code_rate);
    frame.push_back(mod_code);

    // Header CRC (over first 6 bytes)
    uint16_t hdr_crc = crc16(ByteSpan(frame.data(), 6));
    frame.push_back((hdr_crc >> 8) & 0xFF);
    frame.push_back(hdr_crc & 0xFF);

    // Payload
    frame.insert(frame.end(), data.begin(), data.end());

    // Payload CRC
    uint16_t payload_crc = crc16(data);
    frame.push_back((payload_crc >> 8) & 0xFF);
    frame.push_back(payload_crc & 0xFF);

    return frame;
}

Bytes FrameBuilder::buildAckFrame(uint16_t ack_seq, const ChannelQuality& quality) {
    Bytes frame;
    frame.reserve(HEADER_SIZE + 8);

    frame.push_back(static_cast<uint8_t>(FrameType::ACK));
    frame.push_back((ack_seq >> 8) & 0xFF);
    frame.push_back(ack_seq & 0xFF);

    // Payload length (quality data)
    frame.push_back(0);
    frame.push_back(8);

    frame.push_back(0);  // Reserved

    // Header CRC
    uint16_t hdr_crc = crc16(ByteSpan(frame.data(), 6));
    frame.push_back((hdr_crc >> 8) & 0xFF);
    frame.push_back(hdr_crc & 0xFF);

    // Channel quality payload
    // SNR as fixed-point (8.8)
    int16_t snr_fp = static_cast<int16_t>(quality.snr_db * 256);
    frame.push_back((snr_fp >> 8) & 0xFF);
    frame.push_back(snr_fp & 0xFF);

    // Doppler as fixed-point
    int16_t doppler_fp = static_cast<int16_t>(quality.doppler_hz * 256);
    frame.push_back((doppler_fp >> 8) & 0xFF);
    frame.push_back(doppler_fp & 0xFF);

    // BER estimate (exponential encoding)
    uint8_t ber_exp = 0;
    float ber = quality.ber_estimate;
    while (ber < 1.0f && ber_exp < 16) {
        ber *= 10;
        ++ber_exp;
    }
    frame.push_back(ber_exp);

    // Recommended modulation/coding
    auto [rec_mod, rec_rate] = recommendMode(quality);
    uint8_t rec = (static_cast<uint8_t>(rec_mod) << 4) | static_cast<uint8_t>(rec_rate);
    frame.push_back(rec);

    // Padding
    frame.push_back(0);
    frame.push_back(0);

    // Payload CRC
    uint16_t payload_crc = crc16(ByteSpan(frame.data() + HEADER_SIZE, 8));
    frame.push_back((payload_crc >> 8) & 0xFF);
    frame.push_back(payload_crc & 0xFF);

    return frame;
}

Bytes FrameBuilder::buildNackFrame(uint16_t nack_seq) {
    Bytes frame;
    frame.reserve(HEADER_SIZE + CRC_SIZE);

    frame.push_back(static_cast<uint8_t>(FrameType::NACK));
    frame.push_back((nack_seq >> 8) & 0xFF);
    frame.push_back(nack_seq & 0xFF);
    frame.push_back(0);
    frame.push_back(0);
    frame.push_back(0);

    uint16_t hdr_crc = crc16(ByteSpan(frame.data(), 6));
    frame.push_back((hdr_crc >> 8) & 0xFF);
    frame.push_back(hdr_crc & 0xFF);

    // Empty payload, just CRC
    frame.push_back(0);
    frame.push_back(0);

    return frame;
}

Bytes FrameBuilder::buildSyncFrame() {
    Bytes frame;
    frame.push_back(static_cast<uint8_t>(FrameType::SYNC));
    // Sync frame is mostly handled by preamble, this is just a marker
    for (int i = 0; i < 7; ++i) frame.push_back(0);
    return frame;
}

Bytes FrameBuilder::buildProbeFrame() {
    Bytes frame;
    frame.push_back(static_cast<uint8_t>(FrameType::PROBE));
    for (int i = 0; i < 7; ++i) frame.push_back(0);
    return frame;
}

Bytes FrameBuilder::buildConnectFrame() {
    Bytes frame;
    frame.reserve(HEADER_SIZE + 16);

    frame.push_back(static_cast<uint8_t>(FrameType::CONNECT));
    frame.push_back(0);  // seq = 0
    frame.push_back(0);
    frame.push_back(0);  // payload len = 8
    frame.push_back(8);
    frame.push_back(0);

    uint16_t hdr_crc = crc16(ByteSpan(frame.data(), 6));
    frame.push_back((hdr_crc >> 8) & 0xFF);
    frame.push_back(hdr_crc & 0xFF);

    // Connect payload: protocol version, capabilities
    frame.push_back(0x01);  // Version 1
    frame.push_back(0x00);  // Flags
    frame.push_back(static_cast<uint8_t>(Modulation::QAM64));  // Max modulation
    frame.push_back(static_cast<uint8_t>(CodeRate::R5_6));     // Min coding overhead
    frame.push_back(0);
    frame.push_back(0);
    frame.push_back(0);
    frame.push_back(0);

    uint16_t payload_crc = crc16(ByteSpan(frame.data() + HEADER_SIZE, 8));
    frame.push_back((payload_crc >> 8) & 0xFF);
    frame.push_back(payload_crc & 0xFF);

    return frame;
}

Bytes FrameBuilder::buildDisconnectFrame() {
    Bytes frame;
    frame.push_back(static_cast<uint8_t>(FrameType::DISCONNECT));
    for (int i = 0; i < 7; ++i) frame.push_back(0);
    return frame;
}

// ============ Frame Parser ============

FrameParser::FrameParser(const ModemConfig& config) : config_(config) {}

FrameParser::ParsedFrame FrameParser::parse(ByteSpan frame_data) {
    ParsedFrame result;
    result.valid = false;

    if (frame_data.size() < HEADER_SIZE) {
        return result;
    }

    // Parse header
    result.type = static_cast<FrameType>(frame_data[0]);
    result.seq_num = (static_cast<uint16_t>(frame_data[1]) << 8) | frame_data[2];
    uint16_t payload_len = (static_cast<uint16_t>(frame_data[3]) << 8) | frame_data[4];

    // Verify header CRC
    uint16_t expected_hdr_crc = (static_cast<uint16_t>(frame_data[6]) << 8) | frame_data[7];
    uint16_t actual_hdr_crc = crc16(ByteSpan(frame_data.data(), 6));
    if (expected_hdr_crc != actual_hdr_crc) {
        return result;  // Header CRC failed
    }

    // Check we have full frame
    if (frame_data.size() < HEADER_SIZE + payload_len + CRC_SIZE) {
        return result;
    }

    // Verify payload CRC
    ByteSpan payload(frame_data.data() + HEADER_SIZE, payload_len);
    uint16_t expected_payload_crc =
        (static_cast<uint16_t>(frame_data[HEADER_SIZE + payload_len]) << 8) |
        frame_data[HEADER_SIZE + payload_len + 1];
    uint16_t actual_payload_crc = crc16(payload);
    if (expected_payload_crc != actual_payload_crc && payload_len > 0) {
        return result;  // Payload CRC failed
    }

    // Extract payload
    result.payload = Bytes(payload.begin(), payload.end());

    // For ACK frames, parse channel quality
    if (result.type == FrameType::ACK && payload_len >= 6) {
        int16_t snr_fp = (static_cast<int16_t>(payload[0]) << 8) | payload[1];
        result.remote_quality.snr_db = snr_fp / 256.0f;

        int16_t doppler_fp = (static_cast<int16_t>(payload[2]) << 8) | payload[3];
        result.remote_quality.doppler_hz = doppler_fp / 256.0f;

        uint8_t ber_exp = payload[4];
        result.remote_quality.ber_estimate = std::pow(10.0f, -static_cast<float>(ber_exp));
    }

    result.valid = true;
    return result;
}

} // namespace ultra
