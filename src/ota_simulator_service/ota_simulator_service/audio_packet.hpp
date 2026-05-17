#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ultra::ota_simulator_service {

inline constexpr uint32_t kOtaAudioMagic = 0x4d53414fu;
inline constexpr uint16_t kOtaAudioVersion = 1;
inline constexpr uint16_t kOtaAudioFormatF32LE = 1;
inline constexpr uint16_t kOtaAudioChannelCountMono = 1;
inline constexpr size_t kOtaAudioHeaderBytes = 40;

struct OtaAudioPacketHeader {
    uint32_t magic = kOtaAudioMagic;
    uint16_t version = kOtaAudioVersion;
    uint16_t flags = 0;
    uint64_t lease_id = 0;
    uint64_t seq = 0;
    uint64_t start_sample = 0;
    uint32_t sample_count = 0;
    uint16_t sample_format = kOtaAudioFormatF32LE;
    uint16_t channel_count = kOtaAudioChannelCountMono;
};

struct OtaAudioPacket {
    OtaAudioPacketHeader header;
    std::vector<float> samples;
};

struct AudioPacketError {
    std::string message;
};

std::array<uint8_t, kOtaAudioHeaderBytes> serializeAudioHeader(
    const OtaAudioPacketHeader& header);
std::optional<OtaAudioPacketHeader> parseAudioHeader(
    std::span<const uint8_t> bytes,
    AudioPacketError* error = nullptr);
std::vector<uint8_t> serializeAudioPacket(const OtaAudioPacket& packet);
std::optional<OtaAudioPacket> parseAudioPacket(std::span<const uint8_t> bytes,
                                               AudioPacketError* error = nullptr);

}  // namespace ultra::ota_simulator_service
