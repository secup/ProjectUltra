#include "ota_simulator_service/audio_packet.hpp"

#include <cstring>
#include <utility>

namespace ultra::ota_simulator_service {
namespace {

void writeLe16(std::array<uint8_t, kOtaAudioHeaderBytes>& out, size_t offset, uint16_t value) {
    out[offset] = static_cast<uint8_t>(value & 0xffu);
    out[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
}

void writeLe32(std::array<uint8_t, kOtaAudioHeaderBytes>& out, size_t offset, uint32_t value) {
    out[offset] = static_cast<uint8_t>(value & 0xffu);
    out[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
    out[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xffu);
    out[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xffu);
}

void writeLe64(std::array<uint8_t, kOtaAudioHeaderBytes>& out, size_t offset, uint64_t value) {
    for (size_t i = 0; i < 8; ++i) {
        out[offset + i] = static_cast<uint8_t>((value >> (i * 8)) & 0xffu);
    }
}

uint16_t readLe16(std::span<const uint8_t> bytes, size_t offset) {
    return static_cast<uint16_t>(bytes[offset]) |
           (static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

uint32_t readLe32(std::span<const uint8_t> bytes, size_t offset) {
    return static_cast<uint32_t>(bytes[offset]) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

uint64_t readLe64(std::span<const uint8_t> bytes, size_t offset) {
    uint64_t value = 0;
    for (size_t i = 0; i < 8; ++i) {
        value |= static_cast<uint64_t>(bytes[offset + i]) << (i * 8);
    }
    return value;
}

void setError(AudioPacketError* error, std::string message) {
    if (error) {
        error->message = std::move(message);
    }
}

void appendLe32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xffu));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xffu));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xffu));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xffu));
}

}  // namespace

std::array<uint8_t, kOtaAudioHeaderBytes> serializeAudioHeader(
    const OtaAudioPacketHeader& header) {
    std::array<uint8_t, kOtaAudioHeaderBytes> out{};
    writeLe32(out, 0, header.magic);
    writeLe16(out, 4, header.version);
    writeLe16(out, 6, header.flags);
    writeLe64(out, 8, header.lease_id);
    writeLe64(out, 16, header.seq);
    writeLe64(out, 24, header.start_sample);
    writeLe32(out, 32, header.sample_count);
    writeLe16(out, 36, header.sample_format);
    writeLe16(out, 38, header.channel_count);
    return out;
}

std::optional<OtaAudioPacketHeader> parseAudioHeader(std::span<const uint8_t> bytes,
                                                     AudioPacketError* error) {
    if (bytes.size() < kOtaAudioHeaderBytes) {
        setError(error, "audio packet header too short");
        return std::nullopt;
    }

    OtaAudioPacketHeader header;
    header.magic = readLe32(bytes, 0);
    header.version = readLe16(bytes, 4);
    header.flags = readLe16(bytes, 6);
    header.lease_id = readLe64(bytes, 8);
    header.seq = readLe64(bytes, 16);
    header.start_sample = readLe64(bytes, 24);
    header.sample_count = readLe32(bytes, 32);
    header.sample_format = readLe16(bytes, 36);
    header.channel_count = readLe16(bytes, 38);

    if (header.magic != kOtaAudioMagic) {
        setError(error, "bad audio packet magic");
        return std::nullopt;
    }
    if (header.version != kOtaAudioVersion) {
        setError(error, "unsupported audio packet version");
        return std::nullopt;
    }
    if (header.sample_format != kOtaAudioFormatF32LE) {
        setError(error, "unsupported audio sample format");
        return std::nullopt;
    }
    if (header.channel_count != kOtaAudioChannelCountMono) {
        setError(error, "unsupported audio channel count");
        return std::nullopt;
    }
    return header;
}

std::vector<uint8_t> serializeAudioPacket(const OtaAudioPacket& packet) {
    OtaAudioPacketHeader header = packet.header;
    header.sample_count = static_cast<uint32_t>(packet.samples.size());
    auto header_bytes = serializeAudioHeader(header);

    std::vector<uint8_t> out;
    out.reserve(kOtaAudioHeaderBytes + packet.samples.size() * sizeof(float));
    out.insert(out.end(), header_bytes.begin(), header_bytes.end());
    for (float sample : packet.samples) {
        uint32_t raw = 0;
        std::memcpy(&raw, &sample, sizeof(raw));
        appendLe32(out, raw);
    }
    return out;
}

std::optional<OtaAudioPacket> parseAudioPacket(std::span<const uint8_t> bytes,
                                               AudioPacketError* error) {
    auto header = parseAudioHeader(bytes, error);
    if (!header) {
        return std::nullopt;
    }

    const size_t payload_bytes = bytes.size() - kOtaAudioHeaderBytes;
    const size_t expected_bytes = static_cast<size_t>(header->sample_count) * sizeof(float);
    if (payload_bytes != expected_bytes) {
        setError(error, "audio packet payload size mismatch");
        return std::nullopt;
    }

    OtaAudioPacket packet;
    packet.header = *header;
    packet.samples.resize(header->sample_count);
    for (size_t i = 0; i < packet.samples.size(); ++i) {
        const uint32_t raw = readLe32(bytes, kOtaAudioHeaderBytes + i * sizeof(float));
        std::memcpy(&packet.samples[i], &raw, sizeof(raw));
    }
    return packet;
}

}  // namespace ultra::ota_simulator_service
