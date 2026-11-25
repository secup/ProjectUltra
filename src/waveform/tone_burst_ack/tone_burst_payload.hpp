// tone_burst_payload.hpp — packed payload + CRC-12 + (15,11) Hamming for
// the tone-burst ACK (PHY_ADAPTATION_DESIGN §15).

#pragma once

#include "tone_burst_constants.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

// ============================================================================
// ToneBurstAckPayload — semantic view of one tone-burst ACK
// ============================================================================

enum class AckType : uint8_t {
    Ack = 0,
    Nack = 1,
};

struct ToneBurstAckPayload {
    // group_seq: 6-bit (mod-64) cumulative ARQ identity for ordinary ACKs.
    // For the reserved WAITING-REBASE NACK voice it is instead a dedicated
    // per-connection voice-event identity used only to dedup cached RF copies.
    uint8_t group_seq = 0;

    // frame_mask: 16 bits (widened 6->8 2026-06-17, 8->16 2026-07-02 for the wide
    // coherent ARQ window — WIRE-BREAKING, both stations must run the same build),
    // one per frame in the group.
    //   bit i = 1 -> frame i delivered OK
    //   bit i = 0 -> frame i FAILED (sender should resend just this frame)
    // For a NACK, mask is the "still missing" mask (sender resends each 0 bit).
    uint16_t frame_mask = 0;

    // rate_hint: 3-bit RateController feedback (§14.43).
    // SEMANTICS (comment corrected 2026-07-03 — the old text here was a stale
    // rate-index table, never what the code shipped): this is NOT a rate encoding.
    // The ACK emitter quantizes the group's decode-headroom QUALITY q∈[0,1] to 3
    // bits (round(q*7), the SACK-emit lambda in Connection's ctor) and the data
    // sender de-quantizes it (hint/7.0) in onToneBurstAck as the quality feed for
    // its EMA RateController. Phase 2 of docs/MODE_SWITCH_PIGGYBACK_DESIGN_
    // 2026_07_03.md (ULTRA_RX_RATE_CMD, IMPLEMENTED 2026-07-03) keeps this field's
    // quality semantics and adds the separate 2-bit rung_cmd field (bits 42-43,
    // below) for the receiver's demote command.
    uint8_t rate_hint = 0;

    AckType type = AckType::Ack;

    // drive_advisory: 2-bit software-ALC TX-drive feedback (2026-07-02, formerly
    // the reserved bits [30..31]): 0=hold, 1=up (+0.5 dB), 2=down (-2 dB fast),
    // 3=reserved (treat as hold). The receiver sets it from its per-burst RX level
    // verdict (LOW = chain-noise-limited for 2 consecutive bursts -> up; CLIPPED
    // crest-factor signature -> down immediately). CRC-covered on the wire.
    uint8_t drive_advisory = 0;

    // move_epoch: 2-bit ARQ move-epoch echo (2026-07-03, BUG-ARQ-SEQ-COLLISION
    // structural fix, knob ULTRA_ARQ_MOVE_EPOCH default-OFF): the receiver's
    // last-adopted epoch for the data direction being acked. The sender IGNORES
    // an ACK whose epoch != its current TX epoch (stale era — formed against a
    // pre-abort seq grid). Rides the former Hamming zero-pad bits 40..41, so a
    // knob-OFF build (always 0) is byte-identical on air. NOT CRC-covered (see
    // tone_burst_constants.hpp kBitOffsetMoveEpoch rationale); Hamming-protected.
    uint8_t move_epoch = 0;

    // rung_cmd: 2-bit receiver rung command (2026-07-03, MODE_SWITCH_PIGGYBACK
    // Phase 2, knob ULTRA_RX_RATE_CMD default-OFF): kRungCmdNone / kRungCmdDownOne /
    // kRungCmdDownHard / kRungCmdReserved. Demote-only by design (no UP — climbs
    // stay with the sender's EMA). Rides the LAST former Hamming zero-pad bits
    // 42..43, so a knob-OFF build (always 0) is byte-identical on air. CRC-covered
    // ONLY when the knob is ON (message widens 28 -> 30 bits — a corrupted command
    // fires a wrong demote, i.e. fails ACTIVE, so unlike move_epoch it must be
    // integrity-checked; the widened span is part of the knob's lockstep semantics).
    uint8_t rung_cmd = 0;

    // Sanitize fields to their bit-widths. Returns true if all fields are
    // already within range (no truncation occurred).
    bool clampToWireWidths();
};

// ============================================================================
// Bit-level pack / unpack
// ============================================================================
//
// Layout (LSB first within each field, fields in ascending bit-order):
//   bits  0..5   group_seq
//   bits  6..21  frame_mask  (16 bits; widened 6->8 2026-06-17, 8->16 2026-07-02)
//   bits 22..24  rate_hint
//   bit  25      type (0=ACK, 1=NACK)
//   bits 26..37  crc12
//   bits 38..39  drive_advisory (software-ALC; formerly reserved, 2026-07-02)
//   bits 40..41  move_epoch (ARQ move-epoch echo; formerly zero-pad, 2026-07-03,
//                NOT CRC-covered — knob-OFF byte-identity, see constants header)
//   bits 42..43  rung_cmd (receiver rung command; formerly the last zero-pad,
//                2026-07-03 Phase 2; CRC-covered ONLY when ULTRA_RX_RATE_CMD is ON)
//
// The CRC is computed over a 28-bit message: the 26 "useful" bits (bits 0..25)
// with the 2 drive-advisory bits appended above them, using CRC-12-CCITT
// (poly 0x80F, init 0xFFF); with ULTRA_RX_RATE_CMD ON the message is 30 bits
// (rung_cmd appended as message bits 28..29 — kPayloadCrcMessageBitsCmd).
// WIRE-BREAKING vs pre-2026-07-02 builds: (a) the
// advisory joined the CRC coverage; (b) the frame_mask widen 8->16 shifted
// every field above it and grew the payload 32->40 bits (now carried in a
// uint64_t). The 2026-07-03 move_epoch bits are byte-identical while
// ULTRA_ARQ_MOVE_EPOCH is OFF (default) and lockstep-only when ON; the
// 2026-07-03 Phase-2 rung_cmd bits are byte-identical while ULTRA_RX_RATE_CMD
// is OFF and DOUBLY lockstep when ON (live bits + widened CRC span). All
// offsets/widths come from tone_burst_constants.hpp (kBitOffset*/
// kPayload*Bits) — this comment is descriptive; the code reads the constants.
//
// STATELESSNESS: the CRC span is a PARAMETER of every pack/verify/codec entry
// point (`cover_rung_cmd`). The parameter-less overloads use the production
// span (the five rung-command bits covered, rungCmdCrcSpanEnabled); tests pass
// it explicitly.
//
// We use a 12-bit CRC (rather than 16) to keep the packet small: 12 bits at
// ~1 bit/symbol after 4-FSK + (15,11) Hamming means ~3-4 fewer symbols on
// air. CRC-12 still catches all 1-3 bit bursts and >99.9% of random errors
// for our 26-bit message — overkill for ACK semantics.

// Process-wide CRC-span binding for the parameter-less overloads below:
// true when either default-on receiver command path is enabled (read once on
// first use). The pure predicate is exposed so all four feature states can be
// tested without depending on the process-wide environment latch.
inline constexpr bool rungCmdCrcSpanRequired(bool rx_rate_cmd_enabled,
                                             bool rx_authority_enabled) {
    return rx_rate_cmd_enabled || rx_authority_enabled;
}
bool rungCmdCrcSpanEnabled();

// Pack the kPayloadBits (44) raw payload bits (excluding Hamming) into a
// uint64_t (LSB-first). cover_rung_cmd selects the CRC message span
// (28 vs 30 bits); the parameter-less overload binds it from the env knob.
uint64_t packPayload(const ToneBurstAckPayload& p, bool cover_rung_cmd);
uint64_t packPayload(const ToneBurstAckPayload& p);

// Unpack a raw 44-bit payload back to fields. Does NOT validate the CRC;
// use verifyPayloadCRC() for that.
ToneBurstAckPayload unpackPayload(uint64_t raw);

// CRC-12-CCITT (poly 0x80F, init 0xFFF, refin/refout false, xorout 0).
// Computed over `bits` lowest-order bits of `value` (MSB-first into the
// CRC engine).
uint16_t crc12(uint32_t value, uint32_t bits);

// Verify the embedded CRC against the rest of the payload. Returns true if
// the CRC matches. cover_rung_cmd must match the packer's span (lockstep).
bool verifyPayloadCRC(uint64_t raw, bool cover_rung_cmd);
bool verifyPayloadCRC(uint64_t raw);

// ============================================================================
// (15, 11) Hamming codec
// ============================================================================
//
// Per-block: take 11 info bits, append 4 parity bits, total 15 coded bits.
// We do NOT use the extended SECDED variant (which would add a 16th parity
// bit). For our packet size, the gain from SECDED is modest and costs an
// extra symbol per block.
//
// Encoding scheme (systematic, standard textbook):
//   Bit positions 1..15 (1-indexed):
//     pos 1, 2, 4, 8 = parity bits (cover overlapping bit subsets)
//     pos 3, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15 = info bits 0..10
//   Parity i covers all positions whose bit i is set:
//     p1 covers positions {1, 3, 5, 7, 9, 11, 13, 15}
//     p2 covers positions {2, 3, 6, 7, 10, 11, 14, 15}
//     p4 covers positions {4, 5, 6, 7, 12, 13, 14, 15}
//     p8 covers positions {8, 9, 10, 11, 12, 13, 14, 15}
//
// Decoder: compute syndrome (4 bits). If 0, no error. If non-zero, the
// syndrome (read as binary) is the 1-indexed bit position to flip.

// Encode 11 info bits into 15 coded bits. info_bits must be < (1<<11).
// Returns coded bits in bits 0..14 of the uint16_t.
uint16_t hammingEncode15_11(uint16_t info_bits);

// Decode 15 received bits to 11 info bits, correcting up to 1 bit error.
// Sets `corrected_errors` to the number of bits corrected (0 or 1).
// Two-bit errors mis-correct silently (this is the (15,11) limit; the
// outer CRC catches them).
uint16_t hammingDecode15_11(uint16_t coded_bits, int& corrected_errors);

// ============================================================================
// Full packet encode/decode (payload <-> dibits ready for 4-FSK modulation)
// ============================================================================

// Encode a full ToneBurstAckPayload to a sequence of dibits (2-bit values
// 0..3) representing 4-FSK tone indices for the PAYLOAD portion of the
// burst. Length = kPayloadSymbols. Does NOT include the Costas prefix
// (the encoder prepends that). The explicit-span overload exists for tests;
// production uses the env-bound one.
std::vector<uint8_t> encodePayloadDibits(const ToneBurstAckPayload& p,
                                         bool cover_rung_cmd);
std::vector<uint8_t> encodePayloadDibits(const ToneBurstAckPayload& p);

// Decode payload dibits back to a ToneBurstAckPayload. Returns std::nullopt
// if the CRC fails after Hamming correction.
struct PayloadDecodeStats {
    int hamming_corrected_blocks = 0;   // count of blocks where Hamming corrected 1 bit
    bool crc_ok = false;
};
std::optional<ToneBurstAckPayload> decodePayloadDibits(
    const std::vector<uint8_t>& dibits, PayloadDecodeStats& stats,
    bool cover_rung_cmd);
std::optional<ToneBurstAckPayload> decodePayloadDibits(
    const std::vector<uint8_t>& dibits, PayloadDecodeStats& stats);

// ============================================================================
// Costas pattern helpers
// ============================================================================

// Get the full Costas sync pattern as dibit indices.
inline std::array<uint8_t, kCostasSymbols> costasDibits() {
    return kCostasPattern;
}

// Build the full on-air dibit sequence: [COSTAS] [PAYLOAD]. Length =
// kTotalSymbols. This is what the modulator consumes.
std::vector<uint8_t> buildOnAirDibits(const ToneBurstAckPayload& p);

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
