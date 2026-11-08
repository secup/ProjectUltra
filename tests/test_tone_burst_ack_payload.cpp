// test_tone_burst_ack_payload.cpp — unit tests for tone-burst ACK payload,
// CRC-12, and (15,11) Hamming codec. PHY_ADAPTATION_DESIGN §15.7-8 step 1.

#include "waveform/tone_burst_ack/tone_burst_constants.hpp"
#include "waveform/tone_burst_ack/tone_burst_payload.hpp"

#include "env_compat.hpp"  // setenv/unsetenv on MSVC (knob-pinning below)

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

using namespace ultra::waveform::tone_burst_ack;

namespace {

int g_failures = 0;

#define EXPECT(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            ++g_failures;                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);\
        }                                                                       \
    } while (0)

#define EXPECT_EQ(a, b)                                                         \
    do {                                                                        \
        const auto _va = (a);                                                   \
        const auto _vb = (b);                                                   \
        if (!(_va == _vb)) {                                                    \
            ++g_failures;                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s != %s (%lld vs %lld)\n",       \
                         __FILE__, __LINE__, #a, #b,                            \
                         static_cast<long long>(_va),                           \
                         static_cast<long long>(_vb));                          \
        }                                                                       \
    } while (0)

// ============================================================================
// Constants sanity
// ============================================================================

void test_constants_sanity() {
    std::printf("[test] constants_sanity\n");
    // Costas pattern uses only valid dibits.
    for (uint8_t d : kCostasPattern) EXPECT(d < kNumTones);
    // Tone frequencies are inside the subband.
    for (float f : kToneFreqHz) {
        EXPECT(f >= static_cast<float>(kSubbandLowHz));
        EXPECT(f <= static_cast<float>(kSubbandHighHz));
    }
    // Total symbols == Costas + payload.
    EXPECT_EQ(kTotalSymbols, kCostasSymbols + kPayloadSymbols);
    // 23 payload symbols * 2 bits/sym = 46 bit-slots, holds 45 coded bits.
    EXPECT(kPayloadSymbols * kBitsPerSymbol >= kHammingCodedBitsTotal);
    // Tone frequencies are integer multiples of 25 Hz for clean alignment.
    for (float f : kToneFreqHz) {
        const float quotient = f / 25.0f;
        EXPECT_EQ(static_cast<int>(quotient * 100), static_cast<int>(quotient) * 100);
    }

    // Frequency-diversity subband (BUG-TONE-ACK-RETURN-LOSS-ON-FADING): same tone index,
    // kDiversityOffsetHz below, on the 25 Hz grid, inside a 300-2700 Hz SSB passband.
    constexpr float kSsbLowHz = 300.0f, kSsbHighHz = 2700.0f;
    for (uint32_t k = 0; k < kNumTones; ++k) {
        EXPECT(std::fabs(kToneFreqHzDiversity[k] -
                         (kToneFreqHz[k] - static_cast<float>(kDiversityOffsetHz))) < 1e-3f);
        const float q = kToneFreqHzDiversity[k] / 25.0f;
        EXPECT(std::fabs(q - std::round(q)) < 1e-4f);
        EXPECT(kToneFreqHzDiversity[k] >= kSsbLowHz);
    }
    // Disjoint from subband A by more than the shortest symbol's Goertzel main lobe (2 / T).
    const float main_lobe_hz = 2.0f * 1000.0f / static_cast<float>(kSymbolMsHighSNR);
    EXPECT(kToneFreqHz[0] - kToneFreqHzDiversity[kNumTones - 1] > main_lobe_hz);
    // Third-order intermod of each two-tone symbol (2b - a, 2a - b) lands OUTSIDE the passband:
    // a real PA's IMD3 cannot fall on the ACK's own tones or anywhere a peer listens.
    for (uint32_t k = 0; k < kNumTones; ++k) {
        const float a = kToneFreqHz[k], b = kToneFreqHzDiversity[k];
        EXPECT(2.0f * b - a < kSsbLowHz);
        EXPECT(2.0f * a - b > kSsbHighHz);
    }
    // The offset is DERIVED: the minimax over the ITU-R F.1487 delays (Good 0.5, Moderate 1,
    // Poor 2 ms) of the two-path branch power correlation cos^2(pi df tau), over every 25 Hz
    // offset that keeps subband B in the passband and clear of A. The ITU delays are all
    // multiples of 0.5 ms, so the objective is symmetric about df = 1000 Hz and 675 / 1325 Hz
    // tie exactly; the tie breaks toward the LARGER offset, which decorrelates faster at delay
    // spreads below 0.5 ms (tau 0.25 ms: 0.25 vs 0.75) and is the one whose IMD3 is out of band.
    {
        constexpr double kPi = 3.14159265358979323846;
        const double taus[] = {0.5e-3, 1.0e-3, 2.0e-3};
        auto worst = [&](double df) {
            double w = 0.0;
            for (double t : taus) w = std::max(w, std::pow(std::cos(kPi * df * t), 2.0));
            return w;
        };
        uint32_t best = 0;
        double best_w = 2.0;
        for (uint32_t df = 25; df <= 2400; df += 25) {
            const bool in_passband = (kToneFreqHz[0] - static_cast<float>(df)) >= kSsbLowHz;
            const bool clear_of_a = (kToneFreqHz[kNumTones - 1] - static_cast<float>(df)) <
                                    (kToneFreqHz[0] - main_lobe_hz);
            if (!in_passband || !clear_of_a) continue;
            const double w = worst(df);
            if (w < best_w - 1e-9 || std::fabs(w - best_w) <= 1e-9) {  // ties -> larger df
                best = df;
                best_w = std::min(best_w, w);
            }
        }
        EXPECT_EQ(best, kDiversityOffsetHz);
        EXPECT(best_w < 0.30);  // worst branch power correlation 0.273 (Moderate)
    }
}

// ============================================================================
// Pack/unpack round-trip
// ============================================================================

void test_pack_unpack_round_trip() {
    std::printf("[test] pack_unpack_round_trip\n");

    std::vector<ToneBurstAckPayload> cases;
    {
        ToneBurstAckPayload p;
        p.group_seq = 0; p.frame_mask = 0; p.rate_hint = 0; p.type = AckType::Ack;
        p.drive_advisory = kDriveAdvisoryHold;
        cases.push_back(p);
    }
    {
        ToneBurstAckPayload p;
        p.group_seq = 63; p.frame_mask = 63; p.rate_hint = 7; p.type = AckType::Nack;
        p.drive_advisory = 3;  // reserved value must round-trip intact
        cases.push_back(p);
    }
    {
        ToneBurstAckPayload p;
        p.group_seq = 42; p.frame_mask = 0b101010; p.rate_hint = 4; p.type = AckType::Ack;
        p.drive_advisory = kDriveAdvisoryUp;
        cases.push_back(p);
    }
    {
        ToneBurstAckPayload p;
        p.group_seq = 13; p.frame_mask = 0b010101; p.rate_hint = 2; p.type = AckType::Nack;
        p.drive_advisory = kDriveAdvisoryDown;
        cases.push_back(p);
    }
    {
        // Full 16-bit mask (widened 8->16 2026-07-02): a value with bits set in
        // BOTH bytes must round-trip intact — direct proof the high byte carries
        // data on the wire.
        ToneBurstAckPayload p;
        p.group_seq = 31; p.frame_mask = 0xABCD; p.rate_hint = 5; p.type = AckType::Ack;
        p.drive_advisory = kDriveAdvisoryHold;
        cases.push_back(p);
    }
    {
        ToneBurstAckPayload p;
        p.group_seq = 1; p.frame_mask = 0xFFFF; p.rate_hint = 7; p.type = AckType::Nack;
        p.drive_advisory = kDriveAdvisoryUp;
        cases.push_back(p);
    }
    {
        // MOVE-EPOCH (2026-07-03, bits 40-41, ULTRA_ARQ_MOVE_EPOCH): non-zero epoch
        // must round-trip intact. NOT CRC-covered by design (knob-OFF byte-identity)
        // — see the dedicated check below.
        ToneBurstAckPayload p;
        p.group_seq = 9; p.frame_mask = 0x00FF; p.rate_hint = 1; p.type = AckType::Ack;
        p.drive_advisory = kDriveAdvisoryHold;
        p.move_epoch = 3;
        cases.push_back(p);
    }
    {
        // RUNG-CMD (2026-07-03 Phase 2, bits 42-43, ULTRA_RX_RATE_CMD): a non-zero
        // command must round-trip intact even under the DEFAULT (knob-OFF) CRC span
        // — the bits ride the wire regardless; only the coverage is knob-gated.
        ToneBurstAckPayload p;
        p.group_seq = 11; p.frame_mask = 0x0F0F; p.rate_hint = 0; p.type = AckType::Ack;
        p.drive_advisory = kDriveAdvisoryHold;
        p.move_epoch = 1;
        p.rung_cmd = kRungCmdDownHard;
        cases.push_back(p);
    }

    for (const auto& orig : cases) {
        const uint64_t raw = packPayload(orig);
        EXPECT(verifyPayloadCRC(raw));
        const auto rt = unpackPayload(raw);
        EXPECT_EQ(rt.group_seq, orig.group_seq);
        EXPECT_EQ(rt.frame_mask, orig.frame_mask);
        EXPECT_EQ(rt.rate_hint, orig.rate_hint);
        EXPECT_EQ(static_cast<int>(rt.type), static_cast<int>(orig.type));
        EXPECT_EQ(rt.drive_advisory, orig.drive_advisory);
        EXPECT_EQ(rt.move_epoch, orig.move_epoch);
        EXPECT_EQ(rt.rung_cmd, orig.rung_cmd);
    }

    // MOVE-EPOCH knob-OFF byte-identity: two payloads differing ONLY in move_epoch
    // must differ ONLY in bits 40-41 — identical CRC field (the epoch is
    // deliberately OUTSIDE the CRC message) and identical everything else, so an
    // epoch-0 (knob-OFF) payload is bit-identical to a pre-2026-07-03 build's.
    {
        ToneBurstAckPayload a;
        a.group_seq = 5; a.frame_mask = 0x0F0F; a.rate_hint = 2; a.type = AckType::Ack;
        a.drive_advisory = kDriveAdvisoryHold;
        a.move_epoch = 0;
        ToneBurstAckPayload b = a;
        b.move_epoch = 2;
        const uint64_t raw_a = packPayload(a);
        const uint64_t raw_b = packPayload(b);
        EXPECT_EQ(raw_a ^ raw_b,
                  static_cast<uint64_t>(2) << kBitOffsetMoveEpoch);
        EXPECT(verifyPayloadCRC(raw_a));
        EXPECT(verifyPayloadCRC(raw_b));
    }

    // Software-ALC advisory is CRC-covered: two payloads differing ONLY in the
    // drive_advisory bits must produce different CRC fields (a flipped advisory
    // can't ride an unchanged CRC).
    {
        ToneBurstAckPayload a;
        a.group_seq = 7; a.frame_mask = 0b1100; a.rate_hint = 3; a.type = AckType::Ack;
        a.drive_advisory = kDriveAdvisoryHold;
        ToneBurstAckPayload b = a;
        b.drive_advisory = kDriveAdvisoryUp;
        const uint64_t raw_a = packPayload(a);
        const uint64_t raw_b = packPayload(b);
        const uint32_t crc_a = static_cast<uint32_t>(
            (raw_a >> kBitOffsetCRC) & ((1u << kPayloadCRCBits) - 1u));
        const uint32_t crc_b = static_cast<uint32_t>(
            (raw_b >> kBitOffsetCRC) & ((1u << kPayloadCRCBits) - 1u));
        EXPECT(crc_a != crc_b);
    }
}

void test_pack_clamps_out_of_range_fields() {
    std::printf("[test] pack_clamps_out_of_range_fields\n");
    ToneBurstAckPayload p;
    p.group_seq = 200;          // > 63 (6-bit) -> clamps
    p.frame_mask = 0xC5A3;      // bits set in both bytes: fits the WIDENED 16-bit frame_mask
                                // (the uint16_t field can't exceed the 16-bit wire width, so
                                // the frame_mask clamp is structurally a no-op since 2026-07-02)
    p.rate_hint = 15;           // > 7 (3-bit) -> clamps
    p.type = AckType::Ack;
    p.drive_advisory = 7;       // > 3 (2-bit) -> clamps
    p.move_epoch = 6;           // > 3 (2-bit) -> clamps
    p.rung_cmd = 5;             // > 3 (2-bit) -> clamps
    const uint64_t raw = packPayload(p);
    EXPECT(verifyPayloadCRC(raw));
    const auto rt = unpackPayload(raw);
    // group_seq (6-bit) and rate_hint (3-bit) still clamp; frame_mask is now 16-bit so
    // 0xC5A3 round-trips intact — direct proof the high byte carries data after the
    // 8->16 widen.
    EXPECT_EQ(rt.group_seq, static_cast<uint8_t>(200 & 0x3F));
    EXPECT_EQ(rt.frame_mask, static_cast<uint16_t>(0xC5A3));
    EXPECT_EQ(rt.rate_hint, static_cast<uint8_t>(15 & 0x07));
    EXPECT_EQ(rt.drive_advisory, static_cast<uint8_t>(7 & 0x03));
    EXPECT_EQ(rt.move_epoch, static_cast<uint8_t>(6 & 0x03));
    EXPECT_EQ(rt.rung_cmd, static_cast<uint8_t>(5 & 0x03));
}

// ============================================================================
// RUNG-CMD (Phase 2, ULTRA_RX_RATE_CMD) — wire bits 42-43 + knob-conditional
// CRC span. Uses the EXPLICIT-span overloads so the assertions are immune to
// env ordering (the parameter-less overloads latch ULTRA_RX_RATE_CMD once,
// process-wide).
// ============================================================================

void test_rung_cmd_knob_off_span_is_byte_identical() {
    std::printf("[test] rung_cmd_knob_off_span_is_byte_identical\n");
    // Knob-OFF span (28-bit CRC message): two payloads differing ONLY in rung_cmd
    // must differ ONLY in bits 42-43 — identical CRC field. With the production
    // knob-OFF emitter always writing rung_cmd=0, the packed payload is therefore
    // bit-identical to a pre-Phase-2 build's (bits 42-43 were the transmitted
    // Hamming zero-pad).
    ToneBurstAckPayload a;
    a.group_seq = 5; a.frame_mask = 0x0F0F; a.rate_hint = 2; a.type = AckType::Ack;
    a.drive_advisory = kDriveAdvisoryHold;
    a.move_epoch = 0;
    a.rung_cmd = kRungCmdNone;
    ToneBurstAckPayload b = a;
    b.rung_cmd = kRungCmdDownHard;
    const uint64_t raw_a = packPayload(a, /*cover_rung_cmd=*/false);
    const uint64_t raw_b = packPayload(b, /*cover_rung_cmd=*/false);
    EXPECT_EQ(raw_a ^ raw_b,
              static_cast<uint64_t>(kRungCmdDownHard) << kBitOffsetRungCmd);
    EXPECT(verifyPayloadCRC(raw_a, false));
    EXPECT(verifyPayloadCRC(raw_b, false));
    // Under the OFF span a flipped command bit is invisible to the CRC (Hamming-only,
    // like move_epoch) — this documents WHY the knob-ON span must cover it: the
    // knob-OFF consume side ignores the field entirely, so the exposure is nil OFF
    // and CRC-closed ON.
    EXPECT(verifyPayloadCRC(raw_a ^ (1ull << kBitOffsetRungCmd), false));
}

void test_rung_cmd_knob_on_span_covers_command() {
    std::printf("[test] rung_cmd_knob_on_span_covers_command\n");
    ToneBurstAckPayload p;
    p.group_seq = 33; p.frame_mask = 0x00FF; p.rate_hint = 0; p.type = AckType::Ack;
    p.drive_advisory = kDriveAdvisoryHold;
    p.move_epoch = 2;
    p.rung_cmd = kRungCmdDownHard;
    const uint64_t raw = packPayload(p, /*cover_rung_cmd=*/true);
    EXPECT(verifyPayloadCRC(raw, true));
    // Coverage proof: any flipped command bit must fail the widened-span CRC
    // (CRC linearity: a single flipped message bit always changes the CRC).
    for (uint32_t i = 0; i < kPayloadRungCmdBits; ++i) {
        EXPECT(!verifyPayloadCRC(raw ^ (1ull << (kBitOffsetRungCmd + i)), true));
    }
    // A forged command therefore cannot ride an unchanged CRC: differing commands
    // produce differing CRC fields under the ON span.
    ToneBurstAckPayload q = p;
    q.rung_cmd = kRungCmdNone;
    const uint64_t raw_q = packPayload(q, true);
    const uint32_t crc_p = static_cast<uint32_t>(
        (raw >> kBitOffsetCRC) & ((1u << kPayloadCRCBits) - 1u));
    const uint32_t crc_q = static_cast<uint32_t>(
        (raw_q >> kBitOffsetCRC) & ((1u << kPayloadCRCBits) - 1u));
    EXPECT(crc_p != crc_q);
    // move_epoch stays OUTSIDE the CRC in BOTH spans (its own knob's byte-identity
    // rationale is untouched by Phase 2): flipping it never fails either span.
    EXPECT(verifyPayloadCRC(raw ^ (1ull << kBitOffsetMoveEpoch), true));

    // Round-trip under the ON span, including the dibit codec (airtime-invariant:
    // still kPayloadSymbols/kTotalSymbols — the command consumed pad, not symbols).
    const auto dibits = encodePayloadDibits(p, /*cover_rung_cmd=*/true);
    EXPECT_EQ(dibits.size(), static_cast<size_t>(kPayloadSymbols));
    PayloadDecodeStats stats;
    const auto rt = decodePayloadDibits(dibits, stats, /*cover_rung_cmd=*/true);
    EXPECT(rt.has_value());
    if (rt) {
        EXPECT_EQ(rt->rung_cmd, p.rung_cmd);
        EXPECT_EQ(rt->move_epoch, p.move_epoch);
        EXPECT_EQ(rt->frame_mask, p.frame_mask);
    }
}

void test_rung_cmd_span_mismatch_is_rejected() {
    std::printf("[test] rung_cmd_span_mismatch_is_rejected\n");
    // Lockstep semantics: the two spans must be mutually unintelligible so a
    // mixed-version pair fails LOUDLY (CRC reject -> ACK-lost -> RTO) instead of
    // silently mis-steering. Deterministic, not probabilistic: by CRC affinity,
    // crc_30(m) == crc_28(m) XOR D where D = crc(init, 30 zeros) XOR
    // crc(init, 28 zeros) is a nonzero constant (the zero-feed register orbit from
    // init 0xFFF cannot have period 2 — x is invertible mod 0x80F and x^2 != 1) —
    // so EVERY payload's CRC differs between spans, even with rung_cmd = 0.
    ToneBurstAckPayload p;
    p.group_seq = 12; p.frame_mask = 0x3C3C; p.rate_hint = 6; p.type = AckType::Nack;
    p.drive_advisory = kDriveAdvisoryDown;
    p.rung_cmd = kRungCmdNone;
    const uint64_t raw_off = packPayload(p, /*cover_rung_cmd=*/false);
    const uint64_t raw_on = packPayload(p, /*cover_rung_cmd=*/true);
    EXPECT(!verifyPayloadCRC(raw_off, true));   // knob-ON RX rejects knob-OFF TX
    EXPECT(!verifyPayloadCRC(raw_on, false));   // knob-OFF RX rejects knob-ON TX
    // And the constants themselves: the payload now saturates the 4-block Hamming
    // info capacity — the free-bit budget is spent (a future widen changes airtime).
    EXPECT_EQ(kPayloadBits, kHammingInfoBitsTotal);
    EXPECT_EQ(kPayloadBits, kBitOffsetRungCmd + kPayloadRungCmdBits);
    EXPECT_EQ(kTotalSymbols, 34u);
}

// ============================================================================
// CRC-12 sanity
// ============================================================================

void test_crc12_detects_single_bit_flips() {
    std::printf("[test] crc12_detects_single_bit_flips\n");
    ToneBurstAckPayload p;
    p.group_seq = 17; p.frame_mask = 0b110001; p.rate_hint = 3; p.type = AckType::Ack;
    p.drive_advisory = kDriveAdvisoryUp;
    const uint64_t raw = packPayload(p);
    EXPECT(verifyPayloadCRC(raw));
    // Flip each of the useful bits (26 as of the 8->16 mask widen) and confirm
    // CRC catches it.
    int caught = 0;
    for (uint32_t i = 0; i < kPayloadUsefulBits; ++i) {
        const uint64_t flipped = raw ^ (1ull << i);
        if (!verifyPayloadCRC(flipped)) ++caught;
    }
    EXPECT_EQ(caught, static_cast<int>(kPayloadUsefulBits));
    // The drive-advisory bits [38..39] are inside the CRC coverage (widened
    // 2026-07-02): a flipped advisory bit must also fail the CRC. 1ull — the
    // advisory offset is past bit 31.
    int advisory_caught = 0;
    for (uint32_t i = 0; i < kPayloadDriveAdvisoryBits; ++i) {
        const uint64_t flipped = raw ^ (1ull << (kBitOffsetDriveAdvisory + i));
        if (!verifyPayloadCRC(flipped)) ++advisory_caught;
    }
    EXPECT_EQ(advisory_caught, static_cast<int>(kPayloadDriveAdvisoryBits));
}

void test_crc12_detects_two_bit_flips_in_useful_bits() {
    std::printf("[test] crc12_detects_two_bit_flips_in_useful_bits\n");
    ToneBurstAckPayload p;
    p.group_seq = 25; p.frame_mask = 0b011110; p.rate_hint = 5; p.type = AckType::Nack;
    const uint64_t raw = packPayload(p);
    int caught = 0;
    int total = 0;
    for (uint32_t i = 0; i < kPayloadUsefulBits; ++i) {
        for (uint32_t j = i + 1; j < kPayloadUsefulBits; ++j) {
            const uint64_t flipped = raw ^ (1ull << i) ^ (1ull << j);
            ++total;
            if (!verifyPayloadCRC(flipped)) ++caught;
        }
    }
    // CRC-12 detects all 1-2 bit errors over <= 12-bit windows; over the 26-bit
    // useful span it doesn't guarantee 100% but should be very close. Require >= 95%.
    const double rate = static_cast<double>(caught) / static_cast<double>(total);
    std::printf("  2-bit detection: %d/%d (%.1f%%)\n", caught, total, rate * 100.0);
    EXPECT(rate >= 0.95);
}

// ============================================================================
// (15,11) Hamming codec
// ============================================================================

void test_hamming_round_trip_all_codewords() {
    std::printf("[test] hamming_round_trip_all_codewords\n");
    for (uint16_t info = 0; info < (1u << kHammingInfoBitsPerBlock); ++info) {
        const uint16_t coded = hammingEncode15_11(info);
        int corrected = 0;
        const uint16_t decoded = hammingDecode15_11(coded, corrected);
        EXPECT_EQ(decoded, info);
        EXPECT_EQ(corrected, 0);
    }
}

void test_hamming_corrects_single_bit_errors() {
    std::printf("[test] hamming_corrects_single_bit_errors\n");
    int errors_recovered = 0;
    int total = 0;
    // Sample a few hundred random codewords for speed.
    std::mt19937 rng(0xC0DEBEEFu);
    std::uniform_int_distribution<uint32_t> info_dist(0, (1u << kHammingInfoBitsPerBlock) - 1u);
    for (int trial = 0; trial < 500; ++trial) {
        const uint16_t info = static_cast<uint16_t>(info_dist(rng));
        const uint16_t coded = hammingEncode15_11(info);
        for (uint32_t bit = 0; bit < kHammingCodedBitsPerBlock; ++bit) {
            const uint16_t corrupted = coded ^ static_cast<uint16_t>(1u << bit);
            int corrected = 0;
            const uint16_t decoded = hammingDecode15_11(corrupted, corrected);
            ++total;
            if (decoded == info && corrected == 1) ++errors_recovered;
        }
    }
    std::printf("  single-bit recovery: %d/%d\n", errors_recovered, total);
    EXPECT_EQ(errors_recovered, total);
}

// ============================================================================
// Full payload -> dibits -> payload (clean channel)
// ============================================================================

void test_payload_dibits_round_trip_clean() {
    std::printf("[test] payload_dibits_round_trip_clean\n");
    ToneBurstAckPayload p;
    p.group_seq = 21; p.frame_mask = 0b111100; p.rate_hint = 4; p.type = AckType::Ack;
    p.drive_advisory = kDriveAdvisoryDown;
    const auto dibits = encodePayloadDibits(p);
    EXPECT_EQ(dibits.size(), static_cast<size_t>(kPayloadSymbols));
    for (uint8_t d : dibits) EXPECT(d < kNumTones);

    PayloadDecodeStats stats;
    const auto recovered = decodePayloadDibits(dibits, stats);
    EXPECT(recovered.has_value());
    EXPECT(stats.crc_ok);
    EXPECT_EQ(stats.hamming_corrected_blocks, 0);
    if (recovered) {
        EXPECT_EQ(recovered->group_seq, p.group_seq);
        EXPECT_EQ(recovered->frame_mask, p.frame_mask);
        EXPECT_EQ(recovered->rate_hint, p.rate_hint);
        EXPECT_EQ(static_cast<int>(recovered->type), static_cast<int>(p.type));
        EXPECT_EQ(recovered->drive_advisory, p.drive_advisory);
    }
}

void test_on_air_includes_costas_prefix() {
    std::printf("[test] on_air_includes_costas_prefix\n");
    ToneBurstAckPayload p;
    p.group_seq = 7; p.frame_mask = 0b001110; p.rate_hint = 3; p.type = AckType::Nack;
    const auto on_air = buildOnAirDibits(p);
    EXPECT_EQ(on_air.size(), static_cast<size_t>(kTotalSymbols));
    for (uint32_t i = 0; i < kCostasSymbols; ++i) {
        EXPECT_EQ(on_air[i], kCostasPattern[i]);
    }
}

// ============================================================================
// Single-dibit error: one dibit flip = at most 2 coded-bit flips in ONE block
// -> at least one block sees a 2-bit error which (15,11) Hamming can't
// correct. Outer CRC must catch.
// ============================================================================

void test_single_dibit_error_recovery() {
    std::printf("[test] single_dibit_error_recovery\n");
    ToneBurstAckPayload p;
    p.group_seq = 34; p.frame_mask = 0b110011; p.rate_hint = 2; p.type = AckType::Ack;
    p.drive_advisory = kDriveAdvisoryUp;
    auto dibits = encodePayloadDibits(p);

    int decoded_ok = 0;
    int detected_corrupt = 0;
    int total = 0;
    for (size_t i = 0; i < dibits.size(); ++i) {
        for (uint8_t alt = 0; alt < kNumTones; ++alt) {
            if (alt == dibits[i]) continue;
            auto corrupted = dibits;
            corrupted[i] = alt;
            PayloadDecodeStats stats;
            const auto recovered = decodePayloadDibits(corrupted, stats);
            ++total;
            // Two possibilities, both acceptable: Hamming corrects + CRC ok
            // (with the original payload), OR CRC fails and we report no
            // decode. The unacceptable outcome is "CRC ok but wrong payload".
            if (recovered.has_value()) {
                const bool matches_original =
                    recovered->group_seq == p.group_seq &&
                    recovered->frame_mask == p.frame_mask &&
                    recovered->rate_hint == p.rate_hint &&
                    recovered->type == p.type &&
                    recovered->drive_advisory == p.drive_advisory;
                if (matches_original) ++decoded_ok;
                else {
                    // CRC ok but mis-decoded -> Hamming mis-correction that
                    // happened to produce a CRC collision. Should be vanishingly
                    // rare with CRC-12. Count as "undetected" — must be 0.
                    EXPECT(false);
                }
            } else {
                ++detected_corrupt;
            }
        }
    }
    std::printf("  single-dibit results: decoded_ok=%d, detected_corrupt=%d, total=%d\n",
                decoded_ok, detected_corrupt, total);
    // Either outcome is fine; the test gate is "no undetected mis-decode" (above).
    EXPECT_EQ(decoded_ok + detected_corrupt, total);
}

// ============================================================================
// Costas pattern autocorrelation properties
// ============================================================================

void test_costas_autocorrelation_peak() {
    std::printf("[test] costas_autocorrelation_peak\n");
    // For a Costas-style time-frequency pattern, the "thumbtack" auto-
    // ambiguity surface should have a strong peak at (0,0) and low side-
    // lobes elsewhere. We compute the discrete 2D autocorrelation over
    // (time_shift, freq_shift) where freq_shift is in tone-index units.

    const auto& p = kCostasPattern;
    const int N = static_cast<int>(p.size());
    const int F = static_cast<int>(kNumTones);

    // Compute peak (zero shift) and max side-lobe.
    auto matches_at = [&](int dt, int df) {
        int hits = 0;
        for (int i = 0; i < N; ++i) {
            const int j = i + dt;
            if (j < 0 || j >= N) continue;
            const int fi = p[i];
            const int fj = (p[j] + df + F) % F;
            // Pattern correlation: count time-frequency matches.
            if (fi == fj) ++hits;
        }
        return hits;
    };

    const int peak = matches_at(0, 0);
    int max_sidelobe = 0;
    for (int dt = -N + 1; dt <= N - 1; ++dt) {
        for (int df = -F + 1; df <= F - 1; ++df) {
            if (dt == 0 && df == 0) continue;
            const int h = matches_at(dt, df);
            if (h > max_sidelobe) max_sidelobe = h;
        }
    }
    std::printf("  peak=%d, max_sidelobe=%d, ratio=%.2f\n",
                peak, max_sidelobe,
                max_sidelobe ? static_cast<double>(peak) / max_sidelobe : 0.0);
    EXPECT_EQ(peak, N);
    EXPECT(max_sidelobe < N / 2);  // sidelobes must be substantially lower
}

void test_symbol_ms_staircase() {
    std::printf("[test] symbol_ms_staircase\n");
    // RE-MEASURED 2026-11-08 for the two-subband (frequency-diversity) ACK by
    // tools/measure_ack_staircase (production detectAndDecode, equal peak, 400-1000 trials per
    // short cell): 95 %-decode edges
    //
    //     symbol_ms |  AWGN  | Good | Moderate      fading column = worst fading class
    //        12 ms  |   0 dB |  +6  |    +8
    //        25 ms  |  -4 dB |  +2  |    +4
    //        50 ms  |  -6 dB |   0  |    +2
    //       100 ms  | -10 dB |  -2  |  (floor)
    //
    // (2026-08-01 single-tone table, 24 trials/cell: AWGN -4/-8/-10, Good +10/+2/-2, with a
    // 200 ms bottom rung the production monitor could never decode.) Edges below carry the
    // +2 dB kAckEdgeMarginDb (the sweep omits real-radio impairment).
    // AWGN column (fading_present defaults to false):
    EXPECT_EQ(symbolMsForSNR(25.0f), kSymbolMsHighSNR);
    EXPECT_EQ(symbolMsForSNR(2.0f), kSymbolMsHighSNR);    // measured 0 + 2 margin
    EXPECT_EQ(symbolMsForSNR(1.9f), kSymbolMsMidSNR);
    EXPECT_EQ(symbolMsForSNR(-2.0f), kSymbolMsMidSNR);    // measured -4 + 2
    EXPECT_EQ(symbolMsForSNR(-2.1f), kSymbolMsLowSNR);
    EXPECT_EQ(symbolMsForSNR(-4.0f), kSymbolMsLowSNR);    // measured -6 + 2
    EXPECT_EQ(symbolMsForSNR(-4.1f), kSymbolMsMargSNR);
    EXPECT_EQ(symbolMsForSNR(-20.0f), kSymbolMsMargSNR);  // floor, never 200 ms

    // FADING column — materially stricter: a deep null inside a short symbol is unrecoverable.
    EXPECT_EQ(symbolMsForSNR(25.0f, true), kSymbolMsHighSNR);
    EXPECT_EQ(symbolMsForSNR(10.0f, true), kSymbolMsHighSNR);  // Moderate +8 + 2
    EXPECT_EQ(symbolMsForSNR(9.9f,  true), kSymbolMsMidSNR);
    EXPECT_EQ(symbolMsForSNR(6.0f,  true), kSymbolMsMidSNR);   // Moderate +4 + 2
    EXPECT_EQ(symbolMsForSNR(5.9f,  true), kSymbolMsLowSNR);
    EXPECT_EQ(symbolMsForSNR(4.0f,  true), kSymbolMsLowSNR);   // Moderate +2 + 2
    EXPECT_EQ(symbolMsForSNR(3.9f,  true), kSymbolMsMargSNR);
    EXPECT_EQ(symbolMsForSNR(-20.0f, true), kSymbolMsMargSNR); // floor, never 200 ms

    // Every value the staircase can return is a rung the production monitor scans
    // (kStaircaseSymbolMs is its scan set): the 200 ms rung was a guaranteed miss.
    for (bool fading : {false, true}) {
        for (float s = -40.0f; s <= 40.0f; s += 0.25f) {
            const uint32_t ms = symbolMsForSNR(s, fading);
            bool scanned = false;
            for (uint32_t r : kStaircaseSymbolMs) scanned = scanned || (r == ms);
            EXPECT(scanned);
        }
    }

    // The fading edge must never be LOOSER than the AWGN one at the same duration —
    // fading strictly costs margin, and inverting that would key a short ACK into a null.
    for (float s = -20.0f; s <= 30.0f; s += 0.5f) {
        EXPECT(symbolMsForSNR(s, true) >= symbolMsForSNR(s, false));
    }

    // Monotonic non-increasing: a better channel never yields a LONGER ACK.
    for (bool fading : {false, true}) {
        uint32_t prev = symbolMsForSNR(-30.0f, fading);
        for (float s = -30.0f; s <= 30.0f; s += 0.5f) {
            const uint32_t cur = symbolMsForSNR(s, fading);
            EXPECT(cur <= prev);
            prev = cur;
        }
    }
    // The ~20 dB hardware operating point uses the short tier (675 -> 324 ms).
    EXPECT_EQ(symbolMsForSNR(20.0f), kSymbolMsHighSNR);
}

}  // namespace

int main() {
    EXPECT(!rungCmdCrcSpanRequired(false, false));
    EXPECT(rungCmdCrcSpanRequired(true, false));
    EXPECT(rungCmdCrcSpanRequired(false, true));
    EXPECT(rungCmdCrcSpanRequired(true, true));

    // Receiver authority is default-on, so the parameter-less production codec
    // must cover its command bits even when no environment knob is present.
    unsetenv("ULTRA_RX_RATE_CMD");
    unsetenv("ULTRA_RX_RATE_AUTHORITY");
    EXPECT(rungCmdCrcSpanEnabled());

    test_constants_sanity();
    test_pack_unpack_round_trip();
    test_pack_clamps_out_of_range_fields();
    test_rung_cmd_knob_off_span_is_byte_identical();
    test_rung_cmd_knob_on_span_covers_command();
    test_rung_cmd_span_mismatch_is_rejected();
    test_crc12_detects_single_bit_flips();
    test_crc12_detects_two_bit_flips_in_useful_bits();
    test_hamming_round_trip_all_codewords();
    test_hamming_corrects_single_bit_errors();
    test_payload_dibits_round_trip_clean();
    test_on_air_includes_costas_prefix();
    test_single_dibit_error_recovery();
    test_costas_autocorrelation_peak();
    test_symbol_ms_staircase();
    if (g_failures > 0) {
        std::fprintf(stderr, "\n%d test assertion(s) failed\n", g_failures);
        return 1;
    }
    std::printf("\nAll tone-burst ACK payload tests PASSED\n");
    return 0;
}
