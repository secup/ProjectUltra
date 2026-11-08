// tone_burst_constants.hpp — wire-format constants for the tone-burst ACK
//
// Tone-burst ACK is a narrowband 4-FSK control plane that replaces the
// OFDM 1-CW group ACK in burst transport (PHY_ADAPTATION_DESIGN §15).
//
// Design constraints (all four perspectives):
//
// PHY theorist
//   - 4-FSK gives 2 bits/symbol; 40-bit payload + (15,11) Hamming + 4-symbol
//     Costas sync = 34 symbols total = 850 ms at 40 baud (25 ms/sym).
//   - Tones spaced 75 Hz apart in a 300 Hz subband. 75 Hz > Δf for orthogonal
//     FSK at this baud (orthogonality threshold = 1/T_sym = 40 Hz).
//   - Costas pattern at start (5 tones across the 4 frequencies in a
//     specific order) lets the receiver verify "an ACK is arriving" with a
//     known-pattern matched filter — distinguishes the burst from noise that
//     happens to have energy in our band.
//
// DSP systems engineer
//   - All tone frequencies are integer multiples of 25 Hz (sub-band
//     anchored at 2400 Hz, sample rate 48000). 48000 / 25 = 1920 samples
//     per "tone cycle" — clean alignment with sample boundaries.
//   - Symbol duration 25 ms = 1200 samples at 48 kHz. FFT bin resolution
//     for a 1200-sample window is 40 Hz; tone separation 75 Hz puts each
//     tone in a distinct bin with ample margin.
//   - Receiver runs a sliding 1200-sample FFT focused on the four ACK
//     tone bins (4 complex multiplies + 4 magnitudes per sample = trivial).
//
// HF veteran operator
//   - 2400-2700 Hz is above the OFDM data band (300-2400 Hz for 51 carriers
//     centered ~1500 Hz). SSB radios pass 2400-2700 cleanly; rolloff steepens
//     past 2700.
//   - Voice operators use tone bursts at these frequencies routinely
//     (CTCSS, repeater identification). Real-radio proven path.
//
// First principles
//   - Processing gain for matched-filter detection on a known tone in a
//     narrow subband: 10·log₁₀(2·T·BW) = 10·log₁₀(2·0.025·40) = +3 dB per
//     symbol. Over 34 symbols → +18 dB cumulative integration. With 4-FSK
//     discrimination (~2 dB extra cost over 2-FSK), net detection floor
//     ~-2 dB in-band SNR for the 850 ms baseline duration. Longer
//     integration extends the floor lower (§15.5 staircase).

#pragma once

#include <array>
#include <cstdint>
#include <cstdlib>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

// ============================================================================
// Audio-band layout
// ============================================================================

inline constexpr uint32_t kSampleRate = 48000;
inline constexpr uint32_t kSubbandLowHz = 2400;   // bottom of the tone subband
inline constexpr uint32_t kSubbandHighHz = 2700;  // top of the tone subband

// 4-FSK tone frequencies, spaced 75 Hz apart, centered in the subband.
// Each tone is at an integer multiple of 25 Hz so 1200-sample symbol
// windows hit exact integer cycles (suppresses leakage in the FFT bin).
inline constexpr std::array<float, 4> kToneFreqHz = {{
    2400.0f,  // tone 0 -> dibit 0b00
    2475.0f,  // tone 1 -> dibit 0b01
    2550.0f,  // tone 2 -> dibit 0b10
    2625.0f,  // tone 3 -> dibit 0b11
}};

// FREQUENCY DIVERSITY (2026-11-08, BUG-TONE-ACK-RETURN-LOSS-ON-FADING). A 225 Hz-wide FSK
// burst fades as ONE Rayleigh branch on a two-path HF channel, so its loss probability falls
// only ~1/SNR: a deep fade during the 408 ms ACK costs a full RTO (measured in the GUI gate at
// 13.6-17 dB on ITU Good). The half-duplex ACK slot owns the whole audio band, so every symbol
// is sent SIMULTANEOUSLY on a second subband kDiversityOffsetHz below and the detector
// square-law combines the two (|X_A|^2 + |X_B|^2 per tone): loss ~P^2 instead of ~P.
// The offset is DERIVED: for a two-path channel with delay tau the branch power correlation is
// cos^2(pi * df * tau); 1325 Hz minimizes the WORST correlation over the ITU-R F.1487 delays
// (Good 0.5 ms 0.24, Moderate 1 ms 0.27, Poor 2 ms 0.21), on the 25 Hz grid, with both
// subbands inside a 300-2700 Hz SSB passband. Cost on AWGN: equal-peak two-tone ACK -> each
// branch -6 dB, combined -3 dB (PAPR 3 -> 6 dB). ULTRA_TONE_ACK_DIVERSITY=0 opts out; both
// directions degrade gracefully against an old peer (old RX sees branch A at -6 dB; new RX
// sums a noise-only branch B).
inline constexpr uint32_t kDiversityOffsetHz = 1325;
inline constexpr std::array<float, 4> kToneFreqHzDiversity = {{
    2400.0f - kDiversityOffsetHz,  // 1075 Hz
    2475.0f - kDiversityOffsetHz,  // 1150 Hz
    2550.0f - kDiversityOffsetHz,  // 1225 Hz
    2625.0f - kDiversityOffsetHz,  // 1300 Hz
}};

inline bool toneAckDiversityEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_TONE_ACK_DIVERSITY");
        return !(e && e[0] == '0');
    }();
    return on;
}

// Carrier-offset search when the receiver holds NO estimate of the reverse-link dial offset:
// the modem's specified acquisition range (+-50 Hz; the dual-chirp estimator itself spans
// +-60 Hz, chirp_sync.hpp estimateCFO). Below the 75 Hz tone spacing, so a hypothesis one
// tone away is resolved by the Costas pattern (<= 1 coincidence) and the CRC.
inline constexpr float kToneAckCfoAcquisitionHalfWidthHz = 50.0f;

inline constexpr uint32_t kToneSeparationHz = 75;
inline constexpr uint32_t kNumTones = 4;
inline constexpr uint32_t kBitsPerSymbol = 2;  // 4-FSK -> 2 bits/sym

// ============================================================================
// Timing
// ============================================================================

// Baseline (Good-channel) symbol duration. The §15.5 staircase scales
// duration with negotiated SNR; this is the default for SNR >= 12 dB.
inline constexpr uint32_t kBaselineSymbolMs = 25;
inline constexpr uint32_t kBaselineSymbolSamples =
    (kSampleRate * kBaselineSymbolMs) / 1000;  // 1200 samples @ 48 kHz
static_assert(kBaselineSymbolSamples * 1000 == kSampleRate * kBaselineSymbolMs,
              "tone-burst symbol duration must be sample-aligned");

// SNR-adaptive durations (§15.5 staircase).
inline constexpr uint32_t kSymbolMsHighSNR = 12;   // 408 ms ACK
inline constexpr uint32_t kSymbolMsMidSNR = 25;    // 850 ms ACK (baseline)
inline constexpr uint32_t kSymbolMsLowSNR = 50;    // 1.7 s ACK
inline constexpr uint32_t kSymbolMsMargSNR = 100;  // 3.4 s ACK, the staircase floor
// 200 ms (6.8 s ACK) is decodable by ToneBurstDetector but is NOT a staircase rung: a 200 ms
// burst is 326,400 samples, larger than the production monitor buffer (sized from
// kStaircaseSymbolMs), so the monitor never scans it. Kept for detector-level tests.
inline constexpr uint32_t kSymbolMsWeakSNR = 200;

// The staircase rungs — EXACTLY the durations the production ACK monitor scans
// (streaming_decoder.cpp builds its scan set and sizes its buffer from this array). A rung the
// monitor does not scan is a guaranteed miss and a full RTO: until 2026-11-08 symbolMsForSNR
// returned 200 ms below 0 dB on fading / -8 dB on AWGN while the monitor scanned only
// {12, 25, 50, 100} — the app.cpp claim "a conservative (longer) choice is always decodable"
// was false at the bottom of the table.
inline constexpr std::array<uint32_t, 4> kStaircaseSymbolMs = {{
    kSymbolMsHighSNR, kSymbolMsMidSNR, kSymbolMsLowSNR, kSymbolMsMargSNR}};

// §15.5 staircase selector: map the receiver's measured in-band SNR (of the forward link, the
// best available proxy for the reverse link the ACK rides) to the symbol duration. Shorter
// symbols at high SNR shrink the ACK airtime (lower half-duplex turnaround); longer symbols at
// low SNR add matched-filter integration. The sender's monitor scans every rung, so the ends
// need not pre-agree. Monotonic non-increasing in SNR.
//
// RE-MEASURED 2026-11-08 for the two-subband (frequency-diversity) ACK by
// tools/measure_ack_staircase: real encode -> Watterson -> production detectAndDecode, the
// burst at the SAME PEAK as the single-tone ACK (PEP-limited, each subband -6 dB), fixed noise
// referenced to the modem reference level. 400-1000 trials per 12/25/50 ms cell, 150 per
// 100 ms cell; edge = lowest 2 dB step clearing 95 % decode:
//
//     symbol_ms |  AWGN  | Good fading | Moderate fading
//        12 ms  |   0 dB |    +6 dB    |     +8 dB
//        25 ms  |  -4 dB |    +2 dB    |     +4 dB
//        50 ms  |  -6 dB |     0 dB    |     +2 dB
//       100 ms  | -10 dB |    -2 dB    |     (floor)
//
// The FADING column takes the WORST fading class: fading_present cannot tell Good from
// Moderate (the classifier can't make the distinction; see app.cpp). Diversity costs ~3.5-4 dB
// on AWGN (each subband -6 dB at equal peak, square-law combined), which moves every AWGN edge
// up but leaves them all below any wideband operating point; on fading it removes the
// single-branch loss FLOOR the old table could not see — single-tone 12 ms on Moderate stays at
// 1.0-1.5 % loss even at 18-22 dB (1000 trials), the two-subband ACK 0.1-0.2 %.
//
// Cross-check, expected airtime per ACK = T_ack + P_loss x C_loss with C_loss ~16 s (one RTO):
// with the margin below, the 12 <-> 25 ms switch the criterion picks (10 dB fading) is where
// the expected-cost optimum sits (8-10 dB); the lower edges are conservative by this measure
// in a regime the wideband modes do not operate in.
//
// MARGIN: +2 dB (one sweep step) over the measured edge. The sweep runs on the channel model
// alone and omits real-radio impairments (cheap-card distortion, level drift, ALC).
inline constexpr float kAckEdgeMarginDb = 2.0f;

inline constexpr uint32_t symbolMsForSNR(float snr_db, bool fading_present = false) {
    if (fading_present) {
        if (snr_db >= 8.0f + kAckEdgeMarginDb) return kSymbolMsHighSNR;  // 12 ms -> 408 ms
        if (snr_db >= 4.0f + kAckEdgeMarginDb) return kSymbolMsMidSNR;   // 25 ms -> 850 ms
        if (snr_db >= 2.0f + kAckEdgeMarginDb) return kSymbolMsLowSNR;   // 50 ms -> 1.7 s
        return kSymbolMsMargSNR;                                         // 100 ms floor
    }
    if (snr_db >=  0.0f + kAckEdgeMarginDb) return kSymbolMsHighSNR;
    if (snr_db >= -4.0f + kAckEdgeMarginDb) return kSymbolMsMidSNR;
    if (snr_db >= -6.0f + kAckEdgeMarginDb) return kSymbolMsLowSNR;
    return kSymbolMsMargSNR;
}

// ============================================================================
// Costas sync pattern (4 symbols, ORDER-4 COSTAS ARRAY)
// ============================================================================
//
// Used at the START of every tone-burst ACK so the receiver can verify "this
// is an ACK arriving" with a known-pattern matched filter, before trying to
// decode the payload.
//
// Mathematical property — a Costas array of order N has the defining property
// that all N*(N-1)/2 pairwise (time, frequency) displacement vectors are
// DISTINCT. This bounds the autocorrelation sidelobe height at ≤ 1 by
// construction: at any non-zero shift, at most one pair of dots aligns. So
// peak=N, max_sidelobe≤1, peak/sidelobe ratio = N (i.e. +10·log₁₀(N) dB).
//
// {0, 1, 3, 2} is a verified order-4 Costas array (displacement vectors
// (1,1), (2,3), (3,2), (1,2), (2,1), (1,-1) are all distinct).
//   - main-lobe energy = 4 (4 symbols match themselves at zero shift)
//   - max side-lobe energy = 1 (Costas property)
//   - peak / side-lobe ratio = 4x = +6 dB
//
// Same family as the JT9/JT65/FT8 Costas sync arrays (those use order-7
// arrays for their longer messages; ours is shorter so order-4 is enough).
inline constexpr std::array<uint8_t, 4> kCostasPattern = {{0, 1, 3, 2}};
inline constexpr uint32_t kCostasSymbols = kCostasPattern.size();

// ============================================================================
// Payload + FEC layout
// ============================================================================
//
// Payload (packed into 44 bits — the full 4-block Hamming info capacity):
//   bits  0..5   group_seq (6 bits, mod-64 sequence number)
//   bits  6..21  frame_mask (16 bits, 1 = frame OK, 0 = frame FAIL)  [widened 6->8 2026-06-17,
//                8->16 2026-07-02 so a coherent high-throughput window (16 frames) is fully
//                SACK-addressable — see connection_policy kToneBurstAckWindowCapFrames]
//   bits 22..24  rate_hint (3 bits, RateController feedback per §14.43)
//   bit  25      type (0 = ACK, 1 = NACK; receiver always emits a burst)
//   bits 26..37  crc12 (CRC-12 over the 26 useful bits AND the 2 drive-advisory bits
//                = 28 message bits; see kPayloadCrcMessageBits below)
//   bits 38..39  drive_advisory (software-ALC TX-drive feedback, formerly reserved:
//                0 = hold, 1 = up (+0.5 dB), 2 = down (-2 dB), 3 = reserved/treat-as-hold.
//                Receiver-derived from the per-burst RX level verdict — see
//                BUG-QAM16-RIG-LEVEL-BUDGET / ULTRA_SOFTWARE_ALC.)
//   bits 40..41  move_epoch (2 bits, 2026-07-03, BUG-ARQ-SEQ-COLLISION structural fix,
//                knob ULTRA_ARQ_MOVE_EPOCH default-OFF): the receiver's last-adopted
//                ARQ move-epoch echoed back so a stale-era ACK can never retire
//                re-gridded seqs. These two bits were previously part of the always-
//                transmitted Hamming ZERO-PAD (bits 40..43 of block 3), so a knob-OFF
//                build (epoch always 0) is BYTE-IDENTICAL on air to pre-change builds.
//                DELIBERATELY NOT CRC-COVERED: pulling them into the CRC message would
//                change every knob-OFF ACK's CRC (init 0xFFF, MSB-first) and break the
//                default-OFF byte-identity requirement; making CRC coverage knob-
//                conditional would push a protocol env knob into this stateless codec.
//                Protection: (15,11) Hamming block 3 corrects 1 bit; a (rare) 2-bit
//                miscorrect that lands on the epoch fails SAFE at the sender — an
//                epoch-mismatched ACK is IGNORED (= ACK-lost -> RTO resend), never
//                mis-credited.
//   bits 42..43  rung_cmd (2 bits, 2026-07-03, MODE_SWITCH_PIGGYBACK Phase 2, knob
//                ULTRA_RX_RATE_CMD default-OFF): receiver-commanded relative rung step
//                for the wideband-OFDM fade-riding ladder — 0 = no command (back-compat
//                zero), 1 = DOWN one rung, 2 = DOWN hard (crater: go to the sender's
//                escape target), 3 = reserved/treat-as-hold. NO UP command exists BY
//                DESIGN: climbs stay sender-side where the quality EMA lives (a
//                receiver UP would double-drive the control loop). These were the LAST
//                two Hamming zero-pad bits, so a knob-OFF build (cmd always 0) is
//                BYTE-IDENTICAL on air. CRC coverage is KNOB-CONDITIONAL (unlike
//                move_epoch): when ULTRA_RX_RATE_CMD is ON the CRC message widens
//                28 -> 30 bits to include these bits (kPayloadCrcMessageBitsCmd) —
//                a corrupted command fails ACTIVE (fires a wrong demote) rather than
//                SAFE (ignored) like a corrupted epoch, so unlike the epoch it MUST
//                be integrity-checked; the widened span is part of the knob's
//                lockstep semantics (a knob-OFF peer CRC-fails every knob-ON ACK,
//                same both-ends-or-neither class as the mask widen). The env knob is
//                still read OUTSIDE this header (once, in tone_burst_payload.cpp) and
//                every pack/verify entry point also takes the span as an explicit
//                parameter, keeping the codec functions themselves stateless.
//
// WIRE-BREAKING (2026-06-17, 2026-07-02 twice): no version field on the tone-burst
// payload — both stations MUST run the same build. 2026-06-17: the frame_mask widen
// (6->8) moved the CRC field. 2026-07-02 (a): the drive-advisory bits were pulled INTO
// the CRC coverage, so even an advisory=0 payload's CRC differs from pre-change builds.
// 2026-07-02 (b): frame_mask widened 8->16 (wide coherent ARQ window lever) — payload
// grows 32->40 bits, every field above the mask shifts, and the burst is 34 symbols
// (was 27). A mixed-version pair mis-parses every ACK -> CRC fail -> retx storm.
// 2026-07-03 (move_epoch): bits 40..41 (former zero-pad) — byte-identical while
// ULTRA_ARQ_MOVE_EPOCH is OFF (default). When ON the feature is SEMANTICS-BREAKING:
// both stations must run it in lockstep (a knob-OFF peer echoes epoch 0 forever, so
// the knob-ON sender ignores all its ACKs after the first rate-change abort).
// 2026-07-03 (rung_cmd, Phase 2): bits 42..43 (the LAST former zero-pad bits) —
// byte-identical while ULTRA_RX_RATE_CMD is OFF (default: cmd 0, CRC span unchanged).
// When ON the feature is SEMANTICS-BREAKING lockstep TWICE over: the command bits are
// live AND the CRC message widens 28 -> 30 bits, so a knob-OFF peer CRC-rejects every
// knob-ON ACK (deliberate: mixed-version pairs must fail loudly, not mis-steer).
//
// FEC: (15,11) Hamming code applied per nibble-block. 44 payload bits ->
// four 11-bit groups, each encoded to 15 coded bits = 60 coded
// bits = 30 symbols at 4-FSK. Hamming corrects 1 bit error per block,
// detects 2.

inline constexpr uint32_t kPayloadBits = 44;       // raw payload incl. CRC + move_epoch + rung_cmd
inline constexpr uint32_t kPayloadUsefulBits = 26; // bits before CRC (6+16+3+1)
inline constexpr uint32_t kPayloadGroupSeqBits = 6;
inline constexpr uint32_t kPayloadFrameMaskBits = 16;  // widened 6->8 (2026-06-17), 8->16 (2026-07-02): 16-frame SACK window
inline constexpr uint32_t kPayloadRateHintBits = 3;
inline constexpr uint32_t kPayloadTypeBits = 1;
inline constexpr uint32_t kPayloadCRCBits = 12;
inline constexpr uint32_t kPayloadDriveAdvisoryBits = 2;  // formerly reserved (2026-07-02)
inline constexpr uint32_t kPayloadMoveEpochBits = 2;      // formerly zero-pad (2026-07-03)
inline constexpr uint32_t kPayloadRungCmdBits = 2;        // formerly zero-pad (2026-07-03 Phase 2)
static_assert(kPayloadUsefulBits == kPayloadGroupSeqBits + kPayloadFrameMaskBits +
              kPayloadRateHintBits + kPayloadTypeBits, "payload bit layout mismatch");
static_assert(kPayloadBits == kPayloadUsefulBits + kPayloadCRCBits +
              kPayloadDriveAdvisoryBits + kPayloadMoveEpochBits + kPayloadRungCmdBits,
              "payload total mismatch");

// Bit-field offsets in the packed 44-bit payload (carried in a uint64_t).
inline constexpr uint32_t kBitOffsetGroupSeq = 0;
inline constexpr uint32_t kBitOffsetFrameMask = 6;
inline constexpr uint32_t kBitOffsetRateHint = 22;
inline constexpr uint32_t kBitOffsetType = 25;
inline constexpr uint32_t kBitOffsetCRC = 26;
inline constexpr uint32_t kBitOffsetDriveAdvisory = 38;
inline constexpr uint32_t kBitOffsetMoveEpoch = 40;
inline constexpr uint32_t kBitOffsetRungCmd = 42;
static_assert(kBitOffsetFrameMask == kBitOffsetGroupSeq + kPayloadGroupSeqBits &&
              kBitOffsetRateHint == kBitOffsetFrameMask + kPayloadFrameMaskBits &&
              kBitOffsetType == kBitOffsetRateHint + kPayloadRateHintBits &&
              kBitOffsetCRC == kBitOffsetType + kPayloadTypeBits &&
              kBitOffsetDriveAdvisory == kBitOffsetCRC + kPayloadCRCBits &&
              kBitOffsetMoveEpoch == kBitOffsetDriveAdvisory + kPayloadDriveAdvisoryBits &&
              kBitOffsetRungCmd == kBitOffsetMoveEpoch + kPayloadMoveEpochBits &&
              kPayloadBits == kBitOffsetRungCmd + kPayloadRungCmdBits,
              "payload bit offsets must tile the packed payload exactly");

// CRC message = the 26 useful bits with the 2 drive-advisory bits appended above
// them (LSB-first: message bit 26 = advisory bit 0, bit 27 = advisory bit 1). The
// advisory sits ABOVE the CRC field on the wire, so it cannot occupy contiguous
// message positions — pack/verify assemble this 28-bit message explicitly.
// NOTE: the move_epoch bits (40..41) are INTENTIONALLY excluded — see the layout
// comment above (knob-OFF byte-identity; Hamming-only protection fails safe).
// The rung_cmd bits (42..43) are appended as message bits 28..29 ONLY when
// ULTRA_RX_RATE_CMD is ON (kPayloadCrcMessageBitsCmd) — a corrupted command fires
// a wrong demote (fails ACTIVE), so unlike the epoch it must be CRC-checked; the
// widened span is part of the knob's lockstep semantics.
inline constexpr uint32_t kPayloadCrcMessageBits =
    kPayloadUsefulBits + kPayloadDriveAdvisoryBits;  // 28 (knob-OFF span)
inline constexpr uint32_t kPayloadCrcMessageBitsCmd =
    kPayloadCrcMessageBits + kPayloadRungCmdBits;    // 30 (ULTRA_RX_RATE_CMD span)

// Drive-advisory wire values (software-ALC, 2026-07-02).
inline constexpr uint8_t kDriveAdvisoryHold = 0;
inline constexpr uint8_t kDriveAdvisoryUp = 1;    // receiver chain-noise-limited: raise drive
inline constexpr uint8_t kDriveAdvisoryDown = 2;  // receiver sees clip signature: drop drive
inline constexpr uint8_t kDriveAdvisoryReserved = 3;  // treat as hold

// Rung-command wire values (MODE_SWITCH_PIGGYBACK Phase 2, ULTRA_RX_RATE_CMD,
// 2026-07-03). Demote-only BY DESIGN — no UP command (climbs stay sender-side
// where the quality EMA lives; a receiver UP would double-drive the loop).
inline constexpr uint8_t kRungCmdNone = 0;      // no command (back-compat zero)
inline constexpr uint8_t kRungCmdDownOne = 1;   // one rung more robust
inline constexpr uint8_t kRungCmdDownHard = 2;  // crater: sender's escape target
inline constexpr uint8_t kRungCmdReserved = 3;  // WAITING-REBASE voice (2026-07-04,
// BUG-UNANCHORED-SILENCE-ESCAPE, design §5.3): the unanchored receiver's only
// utterance during the move-epoch interregnum ack-silence — "alive, forward link
// works, the era-base frame keeps dying: resend it". NOT an ack: the sender
// consumes it whole in Connection::onToneBurstAck (no SACK parse, no rate-
// controller feed; zero-progress collapse evidence reset + standalone base
// resend via SelectiveRepeatARQ::expireBaseSlotTimerForRebase). Same
// ULTRA_RX_RATE_CMD lockstep as the demote commands; a knob-OFF build never
// emits it and never reaches the consume branch.

// (15,11) Hamming: 11 info bits -> 15 coded bits per block.
// ceil(44 / 11) = 4 blocks. The 2026-07-03 Phase-2 rung_cmd bits consumed the
// LAST two pad bits: the payload now fills the 4-block info capacity EXACTLY
// (44/44 — zero pad remains). Block count is DERIVED from kPayloadBits so a
// future payload widen can never silently truncate the top bits — but the next
// single bit added grows the burst to 5 blocks = 75 coded bits = 38 symbols,
// i.e. it CHANGES the ACK airtime for every knob state (wire-breaking AND
// airtime-breaking). The free-bit budget is spent; a future field must earn a
// symbol-count change. (History: the 2026-07-02 8->16 mask widen grew 3 -> 4
// blocks; move_epoch (40..41) and rung_cmd (42..43) both rode the block-4 pad,
// keeping the symbol count — and ACK airtime — unchanged at 34.)
inline constexpr uint32_t kHammingInfoBitsPerBlock = 11;
inline constexpr uint32_t kHammingCodedBitsPerBlock = 15;
inline constexpr uint32_t kHammingNumBlocks =
    (kPayloadBits + kHammingInfoBitsPerBlock - 1) / kHammingInfoBitsPerBlock;  // 4
inline constexpr uint32_t kHammingInfoBitsTotal =
    kHammingNumBlocks * kHammingInfoBitsPerBlock;  // 44 (== kPayloadBits: capacity saturated, zero pad left)
static_assert(kHammingInfoBitsTotal >= kPayloadBits,
              "Hamming blocks must cover the whole payload");
static_assert(kHammingInfoBitsTotal <= 64,
              "payload FEC pipeline carries info/coded bits in uint64_t");
inline constexpr uint32_t kHammingCodedBitsTotal =
    kHammingNumBlocks * kHammingCodedBitsPerBlock;  // 60
static_assert(kHammingCodedBitsTotal <= 64,
              "payload FEC pipeline carries info/coded bits in uint64_t");
inline constexpr uint32_t kPayloadSymbols =
    (kHammingCodedBitsTotal + kBitsPerSymbol - 1) / kBitsPerSymbol;  // 30

// ============================================================================
// Total burst structure
// ============================================================================
//
// On-air layout:  [COSTAS (4 sym)] [PAYLOAD (30 sym)]
// Total symbols:  34  (27 before the 2026-07-02 8->16 frame_mask widen)
// Baseline airtime (25 ms/sym): 850 ms
// High-SNR airtime (12 ms/sym): 408 ms
// Low-SNR airtime (50 ms/sym):  1700 ms
//
// Sender's "ACK detection window" after burst end = baseline_duration ± 200 ms.
// If no Costas pattern detected in window, sender treats as ACK-lost.

inline constexpr uint32_t kTotalSymbols = kCostasSymbols + kPayloadSymbols;  // 34
static_assert(kTotalSymbols == 34, "expected 34 total symbols");

inline constexpr uint32_t kBaselineTotalMs = kTotalSymbols * kBaselineSymbolMs;  // 850 ms
inline constexpr uint32_t kBaselineTotalSamples =
    kTotalSymbols * kBaselineSymbolSamples;  // 40800 samples

// Sender's no-ACK guard window around the expected ACK arrival.
inline constexpr uint32_t kAckDetectionGuardMs = 200;

// Number of consecutive ACK-lost events before escalating to OFDM ACK
// fallback (§15.7 question 5).
inline constexpr uint32_t kMissedAcksBeforeEscalation = 3;

// ============================================================================
// Tone amplitude (TX scaling)
// ============================================================================

// Single tone at the soundcard. PAPR is 0 dB (a sine has PAR = 3 dB amplitude
// to RMS, but only one tone is active at a time). Keep peak well below clipping.
inline constexpr float kToneAmplitude = 0.45f;  // peak; 0 dBFS = 1.0

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
