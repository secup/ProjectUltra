// tone_burst_payload.cpp — implementation of pack/unpack + CRC-12 + (15,11)
// Hamming + payload encode/decode for the tone-burst ACK.

#include "tone_burst_payload.hpp"

#include <array>
#include <cassert>
#include <cstdlib>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

namespace {

// Extract `bits` bits starting at position `offset` (LSB-first) from `value`.
// 64-bit since the 2026-07-02 frame_mask widen grew the packed payload to
// 40 bits (kPayloadBits) — a uint32_t container would silently drop the CRC
// top bits and the drive advisory.
inline uint64_t getBits(uint64_t value, uint32_t offset, uint32_t bits) {
    const uint64_t mask = (bits >= 64) ? ~0ull : ((1ull << bits) - 1ull);
    return (value >> offset) & mask;
}

// Insert `bits` bits of `field` starting at position `offset` (LSB-first)
// into `value` (sets, does not clear other bits — caller must ensure target
// bits are 0).
inline uint64_t putBits(uint64_t value, uint32_t offset, uint32_t bits, uint64_t field) {
    const uint64_t mask = (bits >= 64) ? ~0ull : ((1ull << bits) - 1ull);
    return value | ((field & mask) << offset);
}

}  // namespace

// ============================================================================
// ToneBurstAckPayload
// ============================================================================

bool ToneBurstAckPayload::clampToWireWidths() {
    const uint8_t group_seq_max = (1u << kPayloadGroupSeqBits) - 1u;
    // frame_mask is 16 wire bits (2026-07-02) — the max must be computed in a
    // 16-bit-capable type (a uint8_t here would truncate to 0xFF and clamp
    // away the widened high byte).
    const uint16_t frame_mask_max =
        static_cast<uint16_t>((1u << kPayloadFrameMaskBits) - 1u);
    const uint8_t rate_hint_max = (1u << kPayloadRateHintBits) - 1u;
    const uint8_t drive_advisory_max = (1u << kPayloadDriveAdvisoryBits) - 1u;
    const uint8_t move_epoch_max = (1u << kPayloadMoveEpochBits) - 1u;
    const uint8_t rung_cmd_max = (1u << kPayloadRungCmdBits) - 1u;

    bool clean = true;
    if (group_seq > group_seq_max) { group_seq &= group_seq_max; clean = false; }
    if (frame_mask > frame_mask_max) { frame_mask &= frame_mask_max; clean = false; }
    if (rate_hint > rate_hint_max) { rate_hint &= rate_hint_max; clean = false; }
    if (drive_advisory > drive_advisory_max) {
        drive_advisory &= drive_advisory_max;
        clean = false;
    }
    if (move_epoch > move_epoch_max) {
        move_epoch &= move_epoch_max;
        clean = false;
    }
    if (rung_cmd > rung_cmd_max) {
        rung_cmd &= rung_cmd_max;
        clean = false;
    }
    return clean;
}

namespace {

// Assemble the CRC message: 26 useful bits (0..25) + the 2 drive-advisory
// bits appended as message bits 26..27; with cover_rung_cmd (ULTRA_RX_RATE_CMD
// lockstep span) the 2 rung_cmd bits are further appended as message bits
// 28..29. The advisory/command ride ABOVE the CRC field on the wire (bits
// 38..39 / 42..43), so the message is built explicitly rather than masked.
// Fits a uint32_t (max span kPayloadCrcMessageBitsCmd = 30), matching
// crc12()'s input width.
inline uint32_t crcMessage(uint64_t raw, bool cover_rung_cmd) {
    const uint32_t useful =
        static_cast<uint32_t>(raw & ((1ull << kPayloadUsefulBits) - 1ull));
    const uint32_t advisory = static_cast<uint32_t>(
        getBits(raw, kBitOffsetDriveAdvisory, kPayloadDriveAdvisoryBits));
    uint32_t msg = useful | (advisory << kPayloadUsefulBits);
    if (cover_rung_cmd) {
        const uint32_t rung = static_cast<uint32_t>(
            getBits(raw, kBitOffsetRungCmd, kPayloadRungCmdBits));
        msg |= rung << kPayloadCrcMessageBits;  // message bits 28..29
    }
    return msg;
}

inline uint32_t crcMessageBits(bool cover_rung_cmd) {
    return cover_rung_cmd ? kPayloadCrcMessageBitsCmd : kPayloadCrcMessageBits;
}

}  // namespace

// The production codec: the receiver's absolute rung command (RX-AUTHORITY, permanent since
// 2026-11-25) reinterprets [rate_hint|rung_cmd] as one 5-bit word, so all five bits sit inside the
// CRC span (a corrupted command = a wrong-rate burst).
bool rungCmdCrcSpanEnabled() { return true; }

// ============================================================================
// Pack / unpack
// ============================================================================

uint64_t packPayload(const ToneBurstAckPayload& p, bool cover_rung_cmd) {
    ToneBurstAckPayload sanitized = p;
    sanitized.clampToWireWidths();

    uint64_t raw = 0;
    raw = putBits(raw, kBitOffsetGroupSeq, kPayloadGroupSeqBits, sanitized.group_seq);
    raw = putBits(raw, kBitOffsetFrameMask, kPayloadFrameMaskBits, sanitized.frame_mask);
    raw = putBits(raw, kBitOffsetRateHint, kPayloadRateHintBits, sanitized.rate_hint);
    raw = putBits(raw, kBitOffsetType, kPayloadTypeBits,
                  static_cast<uint64_t>(sanitized.type));
    raw = putBits(raw, kBitOffsetDriveAdvisory, kPayloadDriveAdvisoryBits,
                  sanitized.drive_advisory);
    // move_epoch (2026-07-03) rides the former zero-pad bits 40..41 and is NOT
    // part of the CRC message (crcMessage() above) — knob-OFF byte-identity.
    raw = putBits(raw, kBitOffsetMoveEpoch, kPayloadMoveEpochBits,
                  sanitized.move_epoch);
    // rung_cmd (2026-07-03 Phase 2) rides the last former zero-pad bits 42..43;
    // CRC-covered only under the widened span (cover_rung_cmd).
    raw = putBits(raw, kBitOffsetRungCmd, kPayloadRungCmdBits,
                  sanitized.rung_cmd);

    const uint16_t crc =
        crc12(crcMessage(raw, cover_rung_cmd), crcMessageBits(cover_rung_cmd));
    raw = putBits(raw, kBitOffsetCRC, kPayloadCRCBits, crc);
    return raw;
}

uint64_t packPayload(const ToneBurstAckPayload& p) {
    return packPayload(p, rungCmdCrcSpanEnabled());
}

ToneBurstAckPayload unpackPayload(uint64_t raw) {
    ToneBurstAckPayload p;
    p.group_seq = static_cast<uint8_t>(getBits(raw, kBitOffsetGroupSeq, kPayloadGroupSeqBits));
    p.frame_mask = static_cast<uint16_t>(getBits(raw, kBitOffsetFrameMask, kPayloadFrameMaskBits));
    p.rate_hint = static_cast<uint8_t>(getBits(raw, kBitOffsetRateHint, kPayloadRateHintBits));
    p.type = static_cast<AckType>(getBits(raw, kBitOffsetType, kPayloadTypeBits));
    p.drive_advisory = static_cast<uint8_t>(
        getBits(raw, kBitOffsetDriveAdvisory, kPayloadDriveAdvisoryBits));
    p.move_epoch = static_cast<uint8_t>(
        getBits(raw, kBitOffsetMoveEpoch, kPayloadMoveEpochBits));
    p.rung_cmd = static_cast<uint8_t>(
        getBits(raw, kBitOffsetRungCmd, kPayloadRungCmdBits));
    return p;
}

// ============================================================================
// CRC-12-CCITT (poly 0x80F, init 0xFFF)
// ============================================================================

uint16_t crc12(uint32_t value, uint32_t bits) {
    // Standard CRC-12-CCITT polynomial: x^12 + x^11 + x^3 + x^2 + x + 1
    // = 0x80F (without the implicit x^12 term).
    constexpr uint16_t kPoly = 0x80F;
    constexpr uint16_t kInit = 0xFFF;
    constexpr uint16_t kCrcMask = 0xFFF;  // 12 bits

    uint16_t crc = kInit;
    // Process MSB-first within the message bits.
    for (uint32_t i = bits; i > 0; --i) {
        const uint16_t input_bit = static_cast<uint16_t>((value >> (i - 1)) & 1u);
        const uint16_t top_bit = (crc >> 11) & 1u;
        crc = static_cast<uint16_t>((crc << 1) & kCrcMask);
        if (top_bit ^ input_bit) {
            crc ^= kPoly;
        }
    }
    return static_cast<uint16_t>(crc & kCrcMask);
}

bool verifyPayloadCRC(uint64_t raw, bool cover_rung_cmd) {
    const uint16_t expected =
        crc12(crcMessage(raw, cover_rung_cmd), crcMessageBits(cover_rung_cmd));
    const uint16_t observed = static_cast<uint16_t>(getBits(raw, kBitOffsetCRC, kPayloadCRCBits));
    return expected == observed;
}

bool verifyPayloadCRC(uint64_t raw) {
    return verifyPayloadCRC(raw, rungCmdCrcSpanEnabled());
}

// ============================================================================
// (15, 11) Hamming codec
// ============================================================================

namespace {

// 1-indexed position of each info bit in the 15-bit codeword.
// Positions 1, 2, 4, 8 are parity; everything else is info.
//   info 0 -> pos  3
//   info 1 -> pos  5
//   info 2 -> pos  6
//   info 3 -> pos  7
//   info 4 -> pos  9
//   info 5 -> pos 10
//   info 6 -> pos 11
//   info 7 -> pos 12
//   info 8 -> pos 13
//   info 9 -> pos 14
//   info 10-> pos 15
constexpr std::array<uint8_t, kHammingInfoBitsPerBlock> kInfoPos = {{
    3, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15
}};

}  // namespace

uint16_t hammingEncode15_11(uint16_t info_bits) {
    // Place info bits at their codeword positions (positions are 1-indexed;
    // we store the codeword in bits 0..14 corresponding to positions 1..15).
    uint16_t codeword = 0;
    for (uint32_t i = 0; i < kHammingInfoBitsPerBlock; ++i) {
        const uint32_t info_bit = (info_bits >> i) & 1u;
        const uint32_t pos = kInfoPos[i];  // 1..15
        codeword |= static_cast<uint16_t>(info_bit << (pos - 1));
    }
    // Compute the 4 parity bits.
    //   p1 at pos 1 covers positions with bit 0 set: 1,3,5,7,9,11,13,15
    //   p2 at pos 2 covers positions with bit 1 set: 2,3,6,7,10,11,14,15
    //   p4 at pos 4 covers positions with bit 2 set: 4,5,6,7,12,13,14,15
    //   p8 at pos 8 covers positions with bit 3 set: 8,9,10,11,12,13,14,15
    for (uint32_t p = 0; p < 4; ++p) {
        const uint32_t parity_pos_mask = 1u << p;  // 1, 2, 4, 8
        uint32_t parity = 0;
        for (uint32_t pos = 1; pos <= 15; ++pos) {
            if (pos == parity_pos_mask) continue;  // skip the parity bit itself
            if ((pos & parity_pos_mask) != 0) {
                parity ^= ((codeword >> (pos - 1)) & 1u);
            }
        }
        codeword |= static_cast<uint16_t>(parity << (parity_pos_mask - 1));
    }
    return static_cast<uint16_t>(codeword & 0x7FFF);  // 15 bits
}

uint16_t hammingDecode15_11(uint16_t coded_bits, int& corrected_errors) {
    corrected_errors = 0;
    uint16_t cw = static_cast<uint16_t>(coded_bits & 0x7FFF);

    // Compute syndrome (4 bits). Each parity bit's syndrome is XOR of all
    // positions covered by it (INCLUDING the parity bit itself, which is at
    // position == 1<<p — that XOR should give 0 if no error).
    uint32_t syndrome = 0;
    for (uint32_t p = 0; p < 4; ++p) {
        const uint32_t parity_pos_mask = 1u << p;
        uint32_t s = 0;
        for (uint32_t pos = 1; pos <= 15; ++pos) {
            if ((pos & parity_pos_mask) != 0) {
                s ^= ((cw >> (pos - 1)) & 1u);
            }
        }
        syndrome |= (s << p);
    }

    // Non-zero syndrome -> 1-indexed position of the bit error.
    if (syndrome != 0 && syndrome <= 15) {
        cw ^= static_cast<uint16_t>(1u << (syndrome - 1));
        corrected_errors = 1;
    }
    // Extract info bits from their codeword positions.
    uint16_t info = 0;
    for (uint32_t i = 0; i < kHammingInfoBitsPerBlock; ++i) {
        const uint32_t pos = kInfoPos[i];
        const uint32_t bit = (cw >> (pos - 1)) & 1u;
        info |= static_cast<uint16_t>(bit << i);
    }
    return info;
}

// ============================================================================
// Full payload encode/decode (44-bit packed payload <-> dibits)
// ============================================================================

namespace {

// Split a kHammingInfoBitsTotal-bit (zero-padded above kPayloadBits) info
// stream into kHammingNumBlocks blocks of 11 bits each (4 blocks / 44 bits
// since the 2026-07-02 40-bit payload; was 3 / 33).
std::array<uint16_t, kHammingNumBlocks> splitToHammingBlocks(uint64_t info) {
    std::array<uint16_t, kHammingNumBlocks> blocks{};
    for (uint32_t b = 0; b < kHammingNumBlocks; ++b) {
        blocks[b] = static_cast<uint16_t>(
            (info >> (b * kHammingInfoBitsPerBlock)) &
            ((1u << kHammingInfoBitsPerBlock) - 1u));
    }
    return blocks;
}

// Merge the 11-bit blocks back to a kHammingInfoBitsTotal-bit value (the top
// pad bits above kPayloadBits are dropped by the caller's payload mask).
uint64_t mergeFromHammingBlocks(const std::array<uint16_t, kHammingNumBlocks>& blocks) {
    uint64_t info = 0;
    for (uint32_t b = 0; b < kHammingNumBlocks; ++b) {
        info |= (static_cast<uint64_t>(blocks[b] &
                    ((1u << kHammingInfoBitsPerBlock) - 1u))
                    << (b * kHammingInfoBitsPerBlock));
    }
    return info;
}

// Convert the kHammingCodedBitsTotal (60) coded bits to kPayloadSymbols (30)
// dibits (4-FSK symbols, 2 bits each). 60 bits fill 30 dibits exactly; if the
// totals ever leave a remainder again, the last dibit's high bits are zero.
std::vector<uint8_t> codedBitsToDibits(uint64_t coded) {
    std::vector<uint8_t> dibits;
    dibits.reserve(kPayloadSymbols);
    for (uint32_t s = 0; s < kPayloadSymbols; ++s) {
        const uint32_t shift = s * kBitsPerSymbol;
        const uint8_t dibit = static_cast<uint8_t>((coded >> shift) & 0x3u);
        dibits.push_back(dibit);
    }
    return dibits;
}

uint64_t dibitsToCodedBits(const std::vector<uint8_t>& dibits) {
    uint64_t coded = 0;
    const size_t n = std::min<size_t>(dibits.size(), kPayloadSymbols);
    for (size_t s = 0; s < n; ++s) {
        coded |= (static_cast<uint64_t>(dibits[s] & 0x3u) << (s * kBitsPerSymbol));
    }
    return coded;
}

}  // namespace

std::vector<uint8_t> encodePayloadDibits(const ToneBurstAckPayload& p,
                                         bool cover_rung_cmd) {
    // 44-bit raw payload — exactly the 4-block Hamming info capacity
    // (kHammingInfoBitsTotal == kPayloadBits since the 2026-07-03 Phase-2
    // rung_cmd bits consumed the last pad; zero pad remains).
    const uint64_t info = packPayload(p, cover_rung_cmd);

    auto blocks = splitToHammingBlocks(info);
    uint64_t coded = 0;
    for (uint32_t b = 0; b < kHammingNumBlocks; ++b) {
        const uint16_t block_coded = hammingEncode15_11(blocks[b]);
        coded |= (static_cast<uint64_t>(block_coded &
                     ((1u << kHammingCodedBitsPerBlock) - 1u))
                     << (b * kHammingCodedBitsPerBlock));
    }
    return codedBitsToDibits(coded);
}

std::vector<uint8_t> encodePayloadDibits(const ToneBurstAckPayload& p) {
    return encodePayloadDibits(p, rungCmdCrcSpanEnabled());
}

std::optional<ToneBurstAckPayload> decodePayloadDibits(
    const std::vector<uint8_t>& dibits, PayloadDecodeStats& stats,
    bool cover_rung_cmd) {
    stats.hamming_corrected_blocks = 0;
    stats.crc_ok = false;

    if (dibits.size() < kPayloadSymbols) {
        return std::nullopt;
    }

    const uint64_t coded = dibitsToCodedBits(dibits);

    std::array<uint16_t, kHammingNumBlocks> info_blocks{};
    for (uint32_t b = 0; b < kHammingNumBlocks; ++b) {
        const uint16_t block_coded = static_cast<uint16_t>(
            (coded >> (b * kHammingCodedBitsPerBlock)) &
            ((1u << kHammingCodedBitsPerBlock) - 1u));
        int corrected = 0;
        info_blocks[b] = hammingDecode15_11(block_coded, corrected);
        if (corrected > 0) ++stats.hamming_corrected_blocks;
    }

    const uint64_t info = mergeFromHammingBlocks(info_blocks);
    // Mask to the 44 payload bits (== the 4-block info capacity exactly since
    // rung_cmd took the last former pad bits 42..43, 2026-07-03 Phase 2).
    const uint64_t raw = info & ((1ull << kPayloadBits) - 1ull);

    if (!verifyPayloadCRC(raw, cover_rung_cmd)) {
        return std::nullopt;
    }
    stats.crc_ok = true;
    return unpackPayload(raw);
}

std::optional<ToneBurstAckPayload> decodePayloadDibits(
    const std::vector<uint8_t>& dibits, PayloadDecodeStats& stats) {
    return decodePayloadDibits(dibits, stats, rungCmdCrcSpanEnabled());
}

std::vector<uint8_t> buildOnAirDibits(const ToneBurstAckPayload& p) {
    std::vector<uint8_t> on_air;
    on_air.reserve(kTotalSymbols);
    // Costas prefix.
    for (uint8_t d : kCostasPattern) on_air.push_back(d);
    // Payload.
    auto payload = encodePayloadDibits(p);
    on_air.insert(on_air.end(), payload.begin(), payload.end());
    assert(on_air.size() == kTotalSymbols);
    return on_air;
}

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
