#pragma once

// CRC-32C (Castagnoli), reflected, init 0xFFFFFFFF, xorout 0xFFFFFFFF.
// Check value: crc32c("123456789") == 0xE3069283.
//
// Used as the turbo data-frame integrity check that also drives decoder early
// stopping. 32 bits rather than the frame's CRC-16 because the decoder checks it
// once per iteration (up to 8 times per failed frame): 8 x 2^-16 = 1.2e-4 undetected
// per failed frame is too weak; 8 x 2^-32 = 1.9e-9 is not.

#include <array>
#include <cstddef>
#include <cstdint>

namespace ultra {
namespace fec {

namespace detail {
constexpr std::array<uint32_t, 256> makeCrc32cTable() {
    std::array<uint32_t, 256> t{};
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1u) ? (0x82F63B78u ^ (c >> 1)) : (c >> 1);
        }
        t[i] = c;
    }
    return t;
}
inline constexpr std::array<uint32_t, 256> kCrc32cTable = makeCrc32cTable();
}  // namespace detail

inline uint32_t crc32c(const uint8_t* data, size_t len) {
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        c = detail::kCrc32cTable[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}

}  // namespace fec
}  // namespace ultra
