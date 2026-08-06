#pragma once

#include "frame_v2.hpp"
#include "waveform_selection.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cmath>

namespace ultra {
namespace protocol {
namespace connection_policy {

inline constexpr uint32_t kScriptedDisconnectLogGraceMs = 2000;

// Hardware scenario lifecycle policy: elapsed time since DISCONNECT TX is not
// completion evidence.  The connection-state callback must first observe the
// terminal transition; only then may the runner apply a short log-flush grace.
inline constexpr bool scriptedDisconnectQuitReady(
    bool completion_observed,
    uint64_t elapsed_since_completion_ms,
    uint32_t grace_ms = kScriptedDisconnectLogGraceMs) {
    return completion_observed && elapsed_since_completion_ms >= grace_ms;
}

// ═══════ OFDM LEGACY ANCHOR SCALE (2026-07-07 — RETIRE after anchor re-measure) ═══════
// The OFDM in-band SNR estimator was recalibrated 2026-07-07: a structural
// +2.758 dB conversion error (it credited the OFDM waveform with the PING-chirp
// reference power; the correct geometric conversion is per_carrier +
// 10log10(2*Ncar/N)) plus an SNR-dependent LS |h|^2 estimation-noise inflation
// were removed at the source (channel_equalizer_lts.cpp). Readings are now the
// honest in-band S:N. BUT three consumers were TUNED against the old (biased)
// scale: the kCoherentLadder rung anchors, the EESM gamma anchoring, and the
// tone-ACK staircase edges. Until those are re-measured on the honest scale,
// they consume reading + this offset — the exact structural constant, derived
// not fitted: 10log10(P_ref/(P_ofdm)) at 59 carriers/1024 FFT. The SNR-dependent
// debias intentionally passes through (that WAS the fading-optimism bug).
// Grep for this constant to find every compatibility site; delete them together.
//
// RE-MEASURED 2026-07-13 (rig F237-F245 vs the F208-F217 ledger, same dial
// MPG@20, same bench): per-ACK ofdm_broadband medians — old (biased) era
// 23.8 dB (n=539) vs honest era 15.1 dB (n=190) → the anchors were tuned
// 8.70 dB above the honest scale, NOT 2.758: the structural conversion bug
// was only part of the old inflation; the guard-bin noise under-read on this
// bench's band-limited noise (finding 1) and the fading/LS optimism made up
// the rest. With only +2.758 the authority sat 1-2 rungs low (fleet 1.07-1.69
// vs ledger 2.04; F246 parked at R1/4). NOTE this is a BENCH-MEASURED value
// (the guard-bin component depends on the noise shape); on OTASim (white
// noise) it over-boosts the authority — acceptable hotter-sim until the §3
// anchor re-measure replaces the whole quarantine.
inline constexpr float kOfdmLegacyAnchorScaleOffsetDb = 8.70f;

// ULTRA_OFDM_ANCHOR_OFFSET_DB — override the marking offset for the RATE-RELEVANT
// consumers (default = kOfdmLegacyAnchorScaleOffsetDb ⇒ byte-identical).
//
// WHY THIS IS OVERRIDABLE. The +8.70 above is a self-consistent pair with the anchor
// table: the new estimator reads 8.70 dB below the scale the anchors were measured on,
// so adding it back is correct BY CONSTRUCTION. But the anchors were measured where
// IMPLEMENTATION loss is small (OTASim/bench), and on real hardware there is more of it.
// Measured on the IONOS rig 2026-07-26 over 4 transfers: the ladder's marked-up snr_avg
// exceeds the decision-directed EVM USABLE estimate by a mean of 14.3 dB
// (16.4/15.2/13.7/12.0). Subtracting the deliberate 8.70 leaves
//     broadband(channel) − EVM(usable) ≈ 5.6 dB.
// Per the two-SNR model that residual is EXPECTED — channel SNR exceeds usable SNR by
// the implementation loss BY DESIGN — but it means the anchor table is optimistic on THIS
// hardware by roughly that much, i.e. the ladder over-commits by ~2 rungs. That
// over-commit is the disease behind the crater/churn cycle; the goodput-graded crater
// predicate (2026-07-26) treats a SYMPTOM of it.
//
// Correcting it on the INPUT keeps ONE decision-maker, which is why this is the right
// place: the EVM demote tried to correct the same thing on the OUTPUT and failed as a
// double driver (14.3 dB scale gap ⇒ its target sat 4-5 rungs below the ladder's; see
// CHANGELOG 2026-07-26). This knob exists to MEASURE the over-commit, not to become a
// second tuned constant — the durable fix is still the §3 anchor re-measure.
// ULTRA_LINEAR_SNR_RING (2026-07-26, default OFF ⇒ byte-identical): make the RX-authority
// SNR ring average LINEAR POWER instead of dB, and compensate the anchor offset in the SAME
// switch so the two can never be separated.
//
// THE DEFECT. Two SNR rings exist and disagree on domain: streaming_decoder.hpp's
// physicalSnrStats averages linear power (and says so — "mean is linear-domain
// (fade-averaged)"), while connection.cpp's rate-ladder ring sums dB. The IONOS dial and the
// anchor table are BOTH mean-power definitions (test_ofdm_snr_calibration gates on the linear
// mean), so by Jensen's inequality the ladder is fed a systematically LOW number on fading —
// worse the more fade variance there is.
//
// WHY THEY ARE COUPLED. Fixing the domain RAISES the ladder's input. The ladder is already
// measured to OVER-COMMIT by ~2 rungs on this bench (2026-07-26 offset A/B: 8.70 → 3.1 cut
// rung churn 65% and craters 60%). A lone domain fix therefore makes over-commit WORSE. So
// this knob applies both halves at once and the compensation is not independently settable.
//
// SIZING THE COMPENSATION — measured, not chosen. The gap must be computed on the RING's OWN
// samples, not on per-frame readings: per-frame in-band readings have dB sd 5.60 and a Jensen
// gap of 2.87 dB (n=2772, three epochs), but the ring is fed per-GROUP observations whose sd
// is 4.36, giving a gap of 1.65 dB (n=75). Compensating with the per-frame 2.87 would
// OVER-correct by >1 dB. An independent audit computing it a different way (ring value vs
// per-frame power-mean) obtained 1.74 dB. Both land on ~1.7.
inline constexpr float kLinearRingJensenCompensationDb = 1.70f;

inline bool linearSnrRingEnabled() {
    const char* e = std::getenv("ULTRA_LINEAR_SNR_RING");
    return e && e[0] == '1' && e[1] == '\0';
}

inline float ofdmAnchorScaleOffsetDb() {
    // An explicit override always wins, so the offset stays independently sweepable for the
    // anchor re-measure; otherwise the linear ring carries its own compensation with it.
    if (std::getenv("ULTRA_OFDM_ANCHOR_OFFSET_DB") == nullptr && linearSnrRingEnabled()) {
        return kOfdmLegacyAnchorScaleOffsetDb - kLinearRingJensenCompensationDb;
    }
    const char* e = std::getenv("ULTRA_OFDM_ANCHOR_OFFSET_DB");
    if (e && *e) {
        // strtod, NOT atof: atof("garbage") returns 0.0, which is a LEGITIMATE value here
        // (the honest scale), so a typo would silently un-calibrate the rate ladder rather
        // than fall back. Require the whole string to parse.
        char* end = nullptr;
        const double d = std::strtod(e, &end);
        if (end != e && *end == '\0') {
            const float v = static_cast<float>(d);
            // Sanity range: reject nonsense so a fat-fingered value cannot wreck the ladder.
            if (v >= -20.0f && v <= 20.0f) return v;
        }
    }
    return kOfdmLegacyAnchorScaleOffsetDb;
}


inline constexpr uint32_t kOFDMSampleRate = 48000;
// Hard production ceiling for one OFDM key-down. The configured base ceiling is
// lower (and may be streak-escalated), but ULTRA_MAX_BURST_AIRTIME_MS is clamped
// to this value. Decoder-side half-duplex guards use the same ceiling when the
// burst descriptor was lost and exact group geometry is therefore unavailable.
inline constexpr uint32_t kMaxBurstAirtimeCeilingMs = 12000;
// If an asynchronous ACK is requested after decoder evidence but without a
// descriptor-derived air-end, keep reverse TX silent for one entire maximum
// OFDM turn plus margin from the latest evidence. This is conservative by
// construction and bounded by the same production key-down ceiling.
inline constexpr int64_t kDescriptorLostReverseTxHoldMs =
    static_cast<int64_t>(kMaxBurstAirtimeCeilingMs) + 1000;
inline constexpr uint32_t kWideOFDMFFTSamples = 1024;
inline constexpr uint32_t kWideOFDMLongCPSamples = 128;
inline constexpr uint32_t kWideOFDMSymbolSamples = kWideOFDMFFTSamples + kWideOFDMLongCPSamples;
// The production ModemEngine installs presets::balanced() in StreamingEncoder:
// 1024 FFT + the live MEDIUM CP (96 samples).  kWideOFDMSymbolSamples above is
// retained for the legacy conservative airtime-ceiling policy; physical play-head
// accounting must use the samples the encoder actually returns.
inline constexpr uint32_t kWideOFDMWireCPSamples = 96;
inline constexpr uint32_t kWideOFDMWireSymbolSamples =
    kWideOFDMFFTSamples + kWideOFDMWireCPSamples;
inline constexpr uint32_t kWideOFDMCarriers = 59;
inline constexpr uint32_t kWideOFDMChirpDurationMs = 500;
inline constexpr uint32_t kWideOFDMChirpGapMs = 100;
inline constexpr uint32_t kWideOFDMFullAnchorExtraSamples =
    2 * ((kOFDMSampleRate * kWideOFDMChirpDurationMs) / 1000) +
    2 * ((kOFDMSampleRate * kWideOFDMChirpGapMs) / 1000);
inline constexpr uint32_t kWideOFDMFullAnchorExtraMs =
    (kWideOFDMFullAnchorExtraSamples * 1000 + kOFDMSampleRate - 1) / kOFDMSampleRate;
// ModemEngine::postProcessTx() wraps every ordinary DATA waveform in these
// configurable zero-sample guards.  Keep the defaults and the sample rounding in
// this protocol-neutral timing helper so the audio producer and Connection's
// half-duplex play-head cannot acquire two different definitions of key-down.
inline constexpr int kDefaultTxLeadInMs = 150;
inline constexpr int kDefaultTxTailMs = 50;

inline int configuredTxLeadInMs() {
    static const int value = [] {
        const char* e = std::getenv("ULTRA_TX_LEADIN_MS");
        return (e && *e) ? std::max(0, std::atoi(e)) : kDefaultTxLeadInMs;
    }();
    return value;
}

inline int configuredTxTailMs() {
    static const int value = [] {
        const char* e = std::getenv("ULTRA_TX_TAIL_MS");
        return (e && *e) ? std::max(0, std::atoi(e)) : kDefaultTxTailMs;
    }();
    return value;
}

inline uint64_t txGuardSamplesForMs(
        int duration_ms,
        uint32_t sample_rate = kOFDMSampleRate) {
    return static_cast<uint64_t>(sample_rate) *
           static_cast<uint64_t>(std::max(0, duration_ms)) / 1000u;
}

inline uint64_t txPostProcessGuardSamples(
        int lead_in_ms = -1,
        int tail_ms = -1,
        uint32_t sample_rate = kOFDMSampleRate) {
    const int lead = lead_in_ms >= 0 ? lead_in_ms : configuredTxLeadInMs();
    const int tail = tail_ms >= 0 ? tail_ms : configuredTxTailMs();
    // This deliberately matches ModemEngine's integer sample arithmetic exactly.
    return txGuardSamplesForMs(lead, sample_rate) +
           txGuardSamplesForMs(tail, sample_rate);
}

inline uint32_t sampleDurationCeilMs(
        uint64_t samples,
        uint32_t sample_rate = kOFDMSampleRate) {
    return static_cast<uint32_t>(std::min<uint64_t>(
        (samples * 1000u + sample_rate - 1u) / sample_rate,
        0xFFFFFFFFull));
}

inline uint64_t postProcessedTxSamples(
        uint64_t waveform_samples,
        int lead_in_ms = -1,
        int tail_ms = -1,
        uint32_t sample_rate = kOFDMSampleRate) {
    return waveform_samples +
           txPostProcessGuardSamples(lead_in_ms, tail_ms, sample_rate);
}

inline uint32_t postProcessedTxDurationFromSamplesMs(
        uint64_t waveform_samples,
        int lead_in_ms = -1,
        int tail_ms = -1,
        uint32_t sample_rate = kOFDMSampleRate) {
    return sampleDurationCeilMs(
        postProcessedTxSamples(
            waveform_samples, lead_in_ms, tail_ms, sample_rate),
        sample_rate);
}

inline uint32_t postProcessedTxDurationMs(
        uint32_t waveform_ms,
        int lead_in_ms = -1,
        int tail_ms = -1,
        uint32_t sample_rate = kOFDMSampleRate) {
    const uint64_t waveform_samples =
        static_cast<uint64_t>(sample_rate) * waveform_ms / 1000u;
    return postProcessedTxDurationFromSamplesMs(
        waveform_samples, lead_in_ms, tail_ms, sample_rate);
}
inline constexpr uint32_t kWideOFDMShortReanchorDefaultMs = 100;
inline constexpr uint32_t kWideOFDMShortReanchorMinMs = 100;
inline constexpr uint32_t kWideOFDMShortReanchorMaxMs = 300;
// ITU-R F.1487 design Doppler spreads, keyed to the channel class the link is
// actually operating on. a1c9c34 had set the "Good" constant to 0.5 Hz — but 0.5 Hz
// is the *Moderate* value (itu_r_f1487::moderate, models.cpp); the real Good channel
// (itu_r_f1487::good) is 0.1 Hz. That mislabel forced QPSK R2/3 cw=8→cw=4 on Good via
// a bogus 846ms coherence cap, when a cw=8 frame (1392ms) fits the TRUE Good coherence
// (Tc≈4230ms) easily. Verified 2026-05-26: cw=8 QPSK R2/3 Good@20 = 3/3 PASS, 0 CWFAIL,
// goodput ~1355 bps (+38% vs cw=4 980). The CW cap now derives its Doppler from the
// measured fading_index (designDopplerForFadingIndex) so Good gets cw=8 while Moderate
// keeps its protective cw=4 cap.
inline constexpr float kGoodHFDesignDopplerHz = 0.1f;
inline constexpr float kModerateHFDesignDopplerHz = 0.5f;
inline constexpr float kPoorHFDesignDopplerHz = 1.0f;

// Map the measured combined fading_index (same scale/boundaries as
// waveform_selection.hpp: kFading*Max — <0.15 AWGN, <0.76 Good, <1.10 Moderate,
// else Poor) to the channel class's design Doppler, so the coherence-time CW cap
// reflects the channel actually in use rather than a fixed worst-case constant.
inline float designDopplerForFadingIndex(float fading_index) {
    if (fading_index < kFadingGoodMax) return kGoodHFDesignDopplerHz;
    if (fading_index < kFadingModerateMax) return kModerateHFDesignDopplerHz;
    return kPoorHFDesignDopplerHz;
}
inline constexpr float kClarkeCoherenceNumerator = 0.423f;
inline constexpr uint32_t kNarrowOFDMSymbolSamples = 2240;
inline constexpr uint32_t kNarrowOFDMCarriers = 21;
inline constexpr uint32_t kNarrowOFDMPilotSpacing = 10;
inline constexpr uint32_t kLDPCBitsPerCodeword = 648;  // z=27 (n=648) short LDPC

// Coded bits per LDPC codeword for a given lifting size. The 802.11n base matrix has
// 24 columns, so N = z × 24: z=27 → 648 (short, default), z=81 → 1944 (long, ~3 dB more
// FEC margin for fading). Airtime scales with TOTAL coded bits (cw × N), so any airtime/
// timeout derived from codeword count MUST know z or it under-counts a z=81 frame ~3×.
inline constexpr uint32_t ldpcCodewordBits(int lifting_z) {
    return (lifting_z == 81) ? 1944u : kLDPCBitsPerCodeword;
}
inline constexpr uint32_t kFixedFrameCodewords = v2::kDefaultFixedFrameCodewords;
inline constexpr uint32_t kOFDMBurstAckBatchFrames = 4;
inline constexpr size_t kWideOFDMWindowFrames = 8;
inline constexpr size_t kHighThroughputOFDMWindowFrames = 16;
// Hard cap on the in-flight OFDM ARQ window when the interactive tone-burst ack is the
// ack mechanism: the cap exists because the tone-burst per-frame SACK `frame_mask` must
// cover the whole in-flight window — a window wider than the mask leaves the trailing
// frames outside it, un-ackable, hence falsely "lost" and resent forever. The wide/
// high-throughput windows above are therefore capped to this on the unified tone-burst
// path (an N-frame message streams as ≤cap-frame windows). SINGLE source of truth —
// code (configureArqForCurrentDataMode) and tests reference it.
// History: 2026-06-17 widened 6->8 (thin-frame cw5 bursts fill the ~8.6s PA-duty budget);
// 2026-07-02 widened 8->16 with the tone-burst frame_mask 8->16 (wide coherent ARQ window
// lever) — the cap now matches the 16-bit wire mask, so the high-throughput window (16)
// and the env-gated coherent override below are fully SACK-addressable. WIRE-BREAKING:
// both stations must run the same build (no version field on the tone-burst payload).
inline constexpr size_t kToneBurstAckWindowCapFrames = 16;
// Burst group size for the INTERLEAVE-ON (Moderate/Poor) path's whole-group ACK and the
// partial-burst padding. The SACK frame_mask is now 16 bits (2026-07-02; 8 on 2026-06-17),
// so the ceiling is 16; this default stays 6 deliberately — the interleave-ON group size
// is a fade-diversity-vs-loss tradeoff that has NOT been re-swept wider, and the measured
// Good-path win comes entirely from the WINDOW cap above (kToneBurstAckWindowCapFrames),
// not from this. (The Good interleave-OFF SR burst sizes itself by airtime/window — it does
// not use this constant.) 6 <= the 16-bit mask, so every group frame is still addressable.
// A group larger than the mask width leaves trailing frames un-ACKable; keep it <= 16.
// Overridable for the interleave-ON path via ULTRA_BURST_GROUP_FRAMES (clamped [2,16]).
inline constexpr size_t kBurstInterleaveGroupFrames = 6;

// Runtime burst group size. Read at the chunk/pad/config sites so TX file-chunking,
// padding, and the encoder's declared group_size all agree (the RX self-describes
// from the descriptor). Overridable via ULTRA_BURST_GROUP_FRAMES, clamped [2,16]
// (values past the 16-bit tone-ack mask leave trailing frames un-ACKable).
inline size_t burstInterleaveGroupFrames() {
    if (const char* env = std::getenv("ULTRA_BURST_GROUP_FRAMES")) {
        const int v = std::atoi(env);
        if (v >= 2) {
            return static_cast<size_t>(std::clamp(v, 2, 16));
        }
    }
    return kBurstInterleaveGroupFrames;
}

// 2026-05-30: SINGLE source of truth for the cross-frame burst-interleave profile.
// OFF by default. Rationale: SR-ARQ per-frame resend (interleave OFF) is the proven
// robustness lever on Good/AWGN, and deriving the TX encoder flag, the TX ARQ semantics,
// AND the on-wire BURST_HEADER descriptor bit all from THIS one function makes them
// impossible to disagree. They disagreed before (the encoder turned interleave OFF for
// QAM16 via a QPSK||QAM8 hardcode, but the ARQ stayed whole-group from the env default)
// -> ALPHA ignored BRAVO's per-frame masks and skipped partial-group holes (the QAM16
// offset-skip bug). Invariant: interleave OFF -> per-frame SR masks; ON -> whole-group
// ACK/NACK.
//
// DEFAULT OFF FOR ALL MODULATIONS (2026-07-21, rig-measured — flips the prior >=16QAM-ON
// default). Cross-frame TIME interleave spreads each LDPC codeword's bits across ALL N
// frames of the burst. The prior default turned it ON for dense mods (>=16QAM) on the
// theory that spreading a frozen FREQUENCY null into a ~1/N NICK is recoverable (GUI/offline
// +47% on 16QAM R2/3 Good@20, 2026-06-14). But the HF fade the rig actually delivers is
// TIME-localized (a Watterson deep fade hits one stretch of the burst): interleaving then
// puts ~1/N corruption into EVERY codeword at once, and when that exceeds the LDPC budget
// (it does on MPG@20) ALL codewords fail together -> a whole-group 0/N crater instead of one
// dead frame. Forced-16QAM-R1/2 interleaved A/B on the IONOS rig (F440-449, MPG@20,
// 2026-07-21): interleave OFF = +37% goodput (1.55 vs 1.13 kbps), 5/5 vs 4/5 delivery, and
// 5x FEWER full craters (12 vs 62) — because failures stay LOCAL (partial groups the
// per-frame SACK resends cheaply) instead of coupling to the burst's worst instant. MPG@20
// IS a "Good" channel, so this DIRECTLY CONTRADICTS the 2026-06-14 Good@20 +47% (a calmer
// epoch / R2/3 / possibly-sim realization) — the newer, faithful rig measurement wins, and
// AWGN has no nulls to smear so interleave buys nothing there either. Net: no channel where
// interleave clearly earns its keep on the real rig -> OFF everywhere, per-frame SACK.
// Re-enable to A/B the Good@20 claim: ULTRA_BURST_INTERLEAVE=1 (force ON), =0 (force OFF).
inline bool burstCrossFrameInterleaveOn(Modulation /*mod*/) {
    if (const char* env = std::getenv("ULTRA_BURST_INTERLEAVE")) {
        return env[0] == '1';  // explicit override forces ON/OFF for ALL modulations (testing)
    }
    return false;  // per-frame SR-ARQ SACK for every modulation (rig evidence above)
}
// ═════════════════════ Software-ALC (closed-loop TX-drive control, 2026-07-02) ═════════════════════
// BUG-QAM16-RIG-LEVEL-BUDGET: rig wire captures showed OFDM data arriving at only ~6-7 dB
// broadband SNR over the receiver's chain-noise floor with ~4-5 dB of unused TX level headroom
// — a chain-noise-dominated link the sender cannot see blindly. The RECEIVER measures, per
// decoded burst: (a) data-segment RMS over the group's kept data frames, (b) its idle
// chain-noise floor (IdleNoiseSNREstimator), (c) the burst crest factor CF = peak/RMS (clip
// signature). It derives a verdict — LOW (headroom available), CLIPPED (upstream clipping),
// OK — and feeds a 2-bit drive advisory back on the tone-burst ACK (bits [30..31]); the
// SENDER walks its per-burst peak target (tx_drive) within [configured baseline, 0.85].
//
// ULTRA_SOFTWARE_ALC=0 disables the LOOP on both ends (receiver always advises hold, sender
// ignores advisories); the receiver-side LEVEL ADVISORY log line stays active either way.
inline bool softwareAlcEnabled() {
    static const bool enabled = [] {
        const char* e = std::getenv("ULTRA_SOFTWARE_ALC");
        return !(e && e[0] == '0');  // default ON
    }();
    return enabled;
}

// Per-burst RX level verdict (receiver-side measurement, decoder-computed).
enum class RxLevelVerdict : int {
    OK = 0,       // healthy level, no advisory
    LOW = 1,      // data rides < kAlcLowHeadroomDb over the chain-noise floor
    CLIPPED = 2,  // crest factor collapsed below kAlcClipCfDb (upstream clipping)
};

// LOW threshold: burst data RMS over the idle chain-noise floor, in dB (env
// ULTRA_ALC_LOW_DB). Default 12 dB, derived from three anchors:
//  1. MEASURED separation: the level-limited rig state reads ~6-7 dB (wire captures,
//     BUG-QAM16-RIG-LEVEL-BUDGET), a healthy sim-reference link at Good@20 reads ~20 dB
//     — 12 splits them with >=5 dB margin each side.
//  2. FIRST PRINCIPLES: with data >=12 dB above the noise floor, the floor degrades a
//     channel-noise-limited link by <= 10*log10(1+10^-1.2) ~= 0.27 dB — the chain has
//     stopped being the bottleneck, so no drive-up is warranted; below it, the arriving
//     level itself caps the dense-constellation rungs (16QAM R1/2 needs ~11-13 dB eff.).
//  3. LOOP GEOMETRY: the full ALC walk (0.5 -> 0.85 peak target) is +4.6 dB — from the
//     measured 6-7 dB it reaches ~11-12 dB, i.e. the loop converges AT this threshold
//     (or at the digital ceiling), never chasing an unreachable target.
inline float alcLowHeadroomDb() {
    static const float v = [] {
        if (const char* e = std::getenv("ULTRA_ALC_LOW_DB")) {
            const float f = static_cast<float>(std::atof(e));
            if (f > 0.0f && f < 40.0f) return f;
        }
        return 12.0f;
    }();
    return v;
}

// CLIPPED threshold: burst data crest factor (peak/RMS) in dB below which upstream
// clipping is assumed (env ULTRA_ALC_CLIP_CF_DB). Default 6.5 dB:
//  - healthy OFDM data arrives at CF ~9-14 dB (near-Gaussian sum of ~59 carriers:
//    max|x| over an N-sample burst ~ sqrt(2 ln N)*sigma ~= 12-13 dB at N~3e4; wire
//    captures measured 9-14);
//  - a noise-dominated segment (the LOW case) is itself Gaussian -> CF ~10-12 dB, so
//    a buried burst can NOT false-trigger CLIPPED;
//  - hard upstream clipping collapses CF toward 0-6 dB (the 2026-06-15 IONOS
//    square-wave disaster measured CF=1.01 ~= 0.1 dB).
//  6.5 dB sits >=2.5 dB below the healthy floor and >=3 dB above the collapse signature.
inline float alcClipCrestFactorDb() {
    static const float v = [] {
        if (const char* e = std::getenv("ULTRA_ALC_CLIP_CF_DB")) {
            const float f = static_cast<float>(std::atof(e));
            if (f > 0.0f && f < 12.0f) return f;
        }
        return 6.5f;
    }();
    return v;
}

// Advisory hysteresis: consecutive LOW bursts required before advising "up". A single
// LOW can be a deep-fade artifact (the burst-average is fade-averaged over ~1-2 Tc,
// but a whole-burst trough remains possible); two consecutive multi-second bursts
// decorrelate across Tc. CLIPPED advises "down" IMMEDIATELY (clipping destroys frames
// now; asymmetric fast-attack/slow-release is classic ALC).
inline constexpr int kAlcLowStreakForUp = 2;

// Sender step sizes: +0.5 dB per advised-up ACK (slow release), -2 dB per advised-down
// (fast attack). One adjustment per ACKed group (repeat-ACK detections are deduped).
inline constexpr float kAlcUpStepFactor = 1.0593f;    // 10^(0.5/20)
inline constexpr float kAlcDownStepFactor = 0.7943f;  // 10^(-2/20)

inline constexpr uint32_t kResponderHandshakeFailSafeMs = 2200;
inline constexpr uint32_t kMCDPSKDualChirpPreambleMs = 1200;
inline constexpr uint32_t kMCDPSKInterFrameGuardMs = 100;
inline constexpr uint32_t kMCDPSKRobustLowAckTimeoutFloorMs = 36000;
inline constexpr uint32_t kCarrierSenseSackCoalesceMs = 30;
inline constexpr int kCarrierSenseAckRepeatCount = 1;

// A silent ACK repeat is an optional diversity copy, never progress-critical.  If the
// decoder has seen any recent RX signal, hold that copy in every waveform: descriptor-
// lost OFDM bursts have no declared air-end, and the live MPG@20 trace proved that CCA
// can call their faded tails idle.  False synchronizer accepts may postpone an opt-in
// repeat, which is strictly safer than keying it over a possible inbound burst.
inline bool shouldHoldSilentAckRepeatForBroadSignal(WaveformMode mode,
                                                     int64_t now_ms,
                                                     int64_t signal_ms,
                                                     int64_t armed_signal_ms,
                                                     int64_t hold_ms =
                                                         kDescriptorLostReverseTxHoldMs) {
    (void)mode;
    // The signal that produced the ACK is necessarily fresh when the repeat is
    // armed. It must not hold its own diversity copy for the full 13-second
    // descriptor-loss guard (which can extend beyond the sender's RTO). Only a
    // NEW post-arm broad stamp is evidence that another inbound waveform may be
    // in progress. This mirrors the existing arm-relative substantive gate.
    return signal_ms > armed_signal_ms && signal_ms > 0 &&
           now_ms >= signal_ms && now_ms - signal_ms < hold_ms;
}

// ULTRA_ACK_REPEAT_SILENT_MS parser.  The diversity repeat is safety-default OFF;
// only an explicit, wholly valid interval enables it.  Keep `0` as an explicit
// opt-out and fail closed on malformed/out-of-range values.
inline uint32_t silentAckRepeatDelayMs(const char* value) {
    if (!value || value[0] == '\0') {
        return 0u;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 10);
    if (end == value || *end != '\0') {
        return 0u;
    }
    if (parsed == 0) {
        return 0u;
    }
    return (parsed >= 300 && parsed <= 5000)
        ? static_cast<uint32_t>(parsed)
        : 0u;
}

// Primary tone-ACK listen-before-talk decision.  Decoder RX evidence is independent
// of the energy CCA and the descriptor-derived air-end: it is the only surviving
// signal in the descriptor/header-loss case where a valid classic DATA member arms
// the delayed SACK timer but no burst geometry was published.  Keep this as a pure
// policy primitive so GUI/headless front ends can share the same three-way gate.
inline bool shouldDeferToneBurstAck(bool cca_enabled,
                                    bool channel_busy,
                                    uint64_t burst_air_samples_remaining,
                                    bool recent_decoder_rx_evidence,
                                    bool inbound_group_complete) {
    return (cca_enabled && channel_busy) ||
           burst_air_samples_remaining > 0 ||
           (!inbound_group_complete && recent_decoder_rx_evidence);
}

// RX-evidence hold used while a primary ACK is pending on the GUI tick.  It is
// derived solely from physical-egress provenance, never from whether evidence
// happened to be present when the ACK was first deferred: a descriptor-loss
// sync/decode stamp may arrive later in the same inbound burst.
inline constexpr int64_t deferredToneAckRxHoldMs(bool inbound_group_complete) {
    return inbound_group_complete ? 0 : kDescriptorLostReverseTxHoldMs;
}
inline constexpr uint32_t kWideOFDMAckTimeoutFloorMs = 8000;

struct OFDMFrameTiming {
    uint32_t data_symbols = 0;
    uint32_t ack_symbols = 0;
    uint32_t data_ms = 0;
    uint32_t ack_ms = 0;
};

struct MCDPSKFrameTiming {
    uint32_t overhead_symbols = 0;
    uint32_t data_only_symbols = 0;
    uint32_t data_symbols = 0;
    uint32_t ack_symbols = 0;
    uint32_t overhead_ms = 0;
    uint32_t data_only_ms = 0;
    uint32_t data_ms = 0;
    uint32_t ack_ms = 0;
};

inline const char* fadingLabel(float fading) {
    if (fading < kFadingAwgnMax) return "AWGN";
    if (fading < kFadingGoodMax) return "Good";
    if (fading < kFadingModerateMax) return "Moderate";
    return "Poor";
}

enum class ChannelClassification {
    AWGN,
    GOOD,
    MODERATE,
    POOR,
};

inline ChannelClassification classifyChannel(float fading) {
    if (fading < kFadingAwgnMax) return ChannelClassification::AWGN;
    if (fading < kFadingGoodMax) return ChannelClassification::GOOD;
    if (fading < kFadingModerateMax) return ChannelClassification::MODERATE;
    return ChannelClassification::POOR;
}

// Good/Moderate discriminator constants (docs/CHANNEL_DISCRIMINATOR_DESIGN_2026_06_15.md).
// fading_index (fade DEPTH) cannot tell Good from Moderate; the Doppler coherence score
// (per-frame pilot |H|^2 autocorrelation @~1.5 s inter-frame cadence) can. It refines ONLY
// the Good<->Moderate boundary when valid; AWGN and Poor stay driven by fade depth.
//
// TWO-THRESHOLD DEAD ZONE (multi-seed GUI calibration, 2026-06-16). coherenceScore is the
// CUMULATIVE MEAN of the per-frame lag-1 autocorrelation (a single 40-snapshot read has ~0.16
// SE — Moderate single-reads scatter to ~0.45 — too noisy; the transfer mean separates cleanly).
// 12-seed×2-channel GUI sweep (cumulative-mean): confident-Good (>= kCoherenceGoodThreshold)
// = 11/12 Good (min 0.50), 0/12 Moderate; confident-Moderate (<= kCoherenceModerateThreshold)
// = 11/12 Moderate (max 0.30), 0/12 Good. The in-between dead zone DEFERS to the raw
// fading_index (conservative status quo) and absorbed the 2 marginal seeds (Good 0.421,
// Moderate 0.359). ZERO dangerous misreads: no Moderate reached the Good threshold (max 0.359
// < 0.45, margin 0.09), so "Moderate read as Good -> over-high rate" cannot happen on this data.
inline constexpr float kCoherenceGoodThreshold = 0.45f;      // confident Good (legacy lag-1; SIM-scale)
inline constexpr float kCoherenceModerateThreshold = 0.30f;  // confident Moderate (legacy lag-1; SIM-scale)
inline constexpr float kRepresentativeGoodFadingIndex = 0.40f;      // mid-Good (< kFadingGoodMax)
inline constexpr float kRepresentativeModerateFadingIndex = 0.85f;  // mid-Moderate (kFadingGoodMax..kFadingModerateMax)

// RADIO-AGNOSTIC coherence-AREA thresholds (2026-06-20, docs/SCALE_INVARIANT_COHERENCE_DISC_2026_06_20.md).
// The legacy lag-1 score above is SIM-calibrated and platform-broken (needs ~0.045 on the IONOS rig,
// ~0.30 on sim — a flat re-base would break sim). The coherence-AREA (cumulative-mean of the sliding-
// window Sum_{lag=1..5} normalized |H|^2 autocov, DopplerCoherenceEstimator::coherenceArea) separates
// Good from Moderate on a SINGLE dimensionless threshold across BOTH audio paths. Cross-platform
// validation (sim + IONOS rig, 7 transfers, faithful C++-algorithm replication): Good {rig +0.09,
// +0.11; sim +0.66} vs Moderate {rig −0.10, −0.10, −0.18; sim −0.12} -> worst-Good +0.091 vs best-Mod
// −0.100, gap 0.19, midpoint ~0. (The sliding-window LOCAL demean compresses Good toward 0 but rejects
// a Moderate transfer's transient Good-like patch — "Mod-b" reads −0.10 here vs +0.08 single-window.)
// Hysteresis (enter > exit): enter is above max-Mod by 0.15 (a Moderate misread needs a +0.15 jump =
// safe), and Good clearing enter by only ~0.04 fails SAFE (a benign Good->uncertain just keeps the
// conservative no-skip). The consumer ALSO has a REACTIVE override (escalations/resends/backlog/cold-
// start) so a momentarily-wrong label cannot strand a frame.
inline constexpr float kCoherenceAreaEnterGood = 0.05f;  // climb into confident-Good (clear of max Mod −0.10 by 0.15)
inline constexpr float kCoherenceAreaExitGood  = 0.00f;  // drop out of confident-Good (hysteresis floor; Mod sits <0)

// Returns a fading_index reflecting the coherence verdict on the Good<->Moderate axis when
// the coherence is valid AND confident; otherwise the raw fading_index (so CONNECT-time —
// before any OFDM data has pooled — and the uncertain dead zone are unchanged = status quo).
// AWGN/Poor are never overridden.
inline float coherenceAdjustedFadingIndex(float fading_index, float coherence_score,
                                          bool coherence_valid) {
    if (!coherence_valid) return fading_index;
    const ChannelClassification base = classifyChannel(fading_index);
    if (base != ChannelClassification::GOOD && base != ChannelClassification::MODERATE) {
        return fading_index;
    }
    if (coherence_score >= kCoherenceGoodThreshold) return kRepresentativeGoodFadingIndex;
    if (coherence_score <= kCoherenceModerateThreshold) return kRepresentativeModerateFadingIndex;
    return fading_index;  // uncertain dead zone -> defer to the blind metric (conservative)
}

struct LadderRung {
    LadderRungId id = LadderRungId::UNKNOWN;
    const char* name = "Unknown";
    WaveformMode waveform = WaveformMode::MC_DPSK;
    Modulation modulation = Modulation::DQPSK;
    CodeRate code_rate = CodeRate::R1_4;
    int num_carriers = 0;
    int samples_per_symbol = 0;
    int cw_count = v2::kDefaultFixedFrameCodewords;
};

inline LadderRung ladderRungForId(LadderRungId id) {
    switch (id) {
        case LadderRungId::ROBUST_LOW:
            // #72/#71 (2026-06-28): MC-DPSK standardized on ONE baud (sps=1024).
            // The CONTROL waveform is fixed at sps=1024 (the handshake profile); a
            // DATA rung at a DIFFERENT baud (the old 2048/512) re-cut the shared
            // waveform and shipped CONNECT_ACK at a baud the peer couldn't sync to
            // -> handshake strand. With every rung at 1024, control==data baud by
            // construction and the only thing that varies is the constellation
            // (DBPSK/DQPSK) + rate — handled by the control-profile path, exactly
            // like OFDM. The 512 "fast" gear's throughput is recovered at 1024 via
            // DQPSK (ROBUST) and code rate; its only real edge was fast-Doppler.
            return {id, "Robust-Low", WaveformMode::MC_DPSK,
                    Modulation::DBPSK, CodeRate::R1_4, 8, 1024, 3};
        case LadderRungId::ROBUST_MID:
            return {id, "Robust-Mid", WaveformMode::MC_DPSK,
                    Modulation::DBPSK, CodeRate::R1_4, 8, 1024, 3};
        case LadderRungId::ROBUST:
            return {id, "Robust", WaveformMode::MC_DPSK,
                    Modulation::DQPSK, CodeRate::R1_4, 8, 1024,
                    v2::kDefaultFixedFrameCodewords};
        case LadderRungId::STANDARD:
            // #72/#71: standardized to sps=1024 (see ROBUST_LOW). Same
            // constellation/rate as ROBUST now (DQPSK 1024 R1/4); the throughput
            // the old 512 baud gave is reachable here via DQPSK + a higher code
            // rate, without the variable-baud handshake hazard.
            return {id, "Standard", WaveformMode::MC_DPSK,
                    Modulation::DQPSK, CodeRate::R1_4, 8, 1024,
                    v2::kDefaultFixedFrameCodewords};
        case LadderRungId::OFDM_CHIRP:
            // Coherent-only wideband OFDM (thread A, 2026-05-31). Nominal mod is
            // QPSK; the real data mod still comes from recommendDataMode(). NARROW
            // below stays DQPSK — narrowband/low-SNR is a robust differential regime.
            return {id, "OFDM_CHIRP", WaveformMode::OFDM_CHIRP,
                    Modulation::QPSK, CodeRate::R1_4, 0, 0,
                    v2::kDefaultFixedFrameCodewords};
        case LadderRungId::OFDM_NARROW:
            return {id, "OFDM_NARROW", WaveformMode::OFDM_NARROW,
                    Modulation::DQPSK, CodeRate::R1_4, 0, 0,
                    v2::kDefaultFixedFrameCodewords};
        case LadderRungId::UNKNOWN:
        default:
            return {};
    }
}

inline LadderRung rungForMCDPSKConfig(Modulation modulation,
                                      int num_carriers,
                                      int samples_per_symbol,
                                      int cw_count) {
    if (num_carriers != 8) {
        return {};
    }
    if (modulation == Modulation::DBPSK && samples_per_symbol >= 2048 && cw_count == 3) {
        return ladderRungForId(LadderRungId::ROBUST_LOW);
    }
    if (modulation == Modulation::DBPSK && samples_per_symbol == 1024 && cw_count == 3) {
        return ladderRungForId(LadderRungId::ROBUST_MID);
    }
    if (modulation == Modulation::DQPSK && samples_per_symbol == 1024 &&
        cw_count == v2::kDefaultFixedFrameCodewords) {
        return ladderRungForId(LadderRungId::ROBUST);
    }
    if (modulation == Modulation::DQPSK && samples_per_symbol == 512 &&
        cw_count == v2::kDefaultFixedFrameCodewords) {
        return ladderRungForId(LadderRungId::STANDARD);
    }
    return {};
}

// ═══════════ #58 increment 3 — connect-SNR pool (BUG-CONNECT-SNR-VARIANCE) ═══════════
// The entry pick and the MODE_CHANGE wire byte were fed by the last-write-wins scalar
// measured_snr_db_ — a SINGLE fade realization (rig MPG@20: per-connect readings
// 3.9-17.9 dB, sigma 3.15; W3's lone 3.9 trough reading bought a ~20x mis-pick into
// 90 bps DBPSK on a dial-20 channel; W2 shipped a 3.2 dB reading ~31 s stale on the
// wire). The pool keeps the last few QUALIFYING readings and aggregates them as a
// decorrelation-clustered dB-domain MEAN:
//   - dB-mean, NOT linear mean (Jensen: E_linear >= E_dB — a linear mean systematically
//     exceeds the population the +5 connectSnrFadeBasisDb below was calibrated against,
//     i.e. it would double-count optimism), and NOT max (order statistics: E[max of N]
//     grows ~sigma*sqrt(2 ln N) above the mean — an N-dependent bias the fixed basis
//     cannot absorb; the one legitimate lower-bound argument already lives in the
//     Moderate saturation bound in connectSelectionSnrDb).
//   - Readings closer than Tc apart are ONE fade sample (Clarke/Jakes: the channel has
//     not decorrelated) — they merge into one cluster (dB-average) before the mean;
//     N_eff = cluster count. Tc derives from the same no-magic-constants chain as the
//     retx trough pacing: retxTroughDopplerHz -> coherenceTimeMsForDoppler (measured
//     Doppler when the coherence verdict is valid, else the ITU-R design Doppler of the
//     coherence-adjusted fading class). AWGN class => Tc = UINT32_MAX => everything
//     clusters to one sample, whose dB-mean is exactly the right stationary estimate.
// POPULATION CONTRACT (enforced HERE so it is unit-testable and cannot be bypassed):
//   - MCDPSK_IN_BAND readings enter ONLY when data_aided (the whole-frame fade-AVERAGED
//     estimate). The training snapshot is a single ~170 ms fade state with a documented
//     fade-crest over-read on Moderate — a DIFFERENT calibration basis; it must never
//     launder into the aggregate (or through it into the saturation bound).
//   - OFDM_BROADBAND readings enter tagged, and serve ONLY the wire-freshness fix
//     (handshake_only=false aggregations); the ENTRY pick filters them out.
//   - Everything else (IDLE_IN_BAND, SYNC_QUALITY, ...) is rejected.
// AGING: age_ms is advanced from Connection::tick(elapsed_ms) — modem-time, never
// Date/wall-clock (CPU-paced sims and the rig then share one clock basis).
struct ConnectSnrReading {
    float snr_db = 0.0f;
    // Fading index observed WITH this SNR reading (setChannelQuality feeds both from
    // the same decoded frame); NAN = absent (setMeasuredSNR-only feeds carry no fading).
    // Same single-realization scatter problem as the SNR (rig MPG@20 dial-Good,
    // 48-entry ledger docs/CONNECT_ENTRY_CALIBRATION_2026_07_03.md: single-frame
    // fading 0.24-0.74, sigma 0.129, straddling kFadingGoodMax=0.65 — one 0.66
    // reading classified a true-Good channel Moderate and bought a QPSK R1/4 entry).
    float fading_index = NAN;
    uint64_t age_ms = 0;     // advanced from Connection::tick elapsed_ms (modem-time)
    bool data_aided = false;
    SNRSource source = SNRSource::NONE;
};

class ConnectSnrPool {
public:
    static constexpr size_t kCapacity = 8;

    void clear() { count_ = 0; }
    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }

    // Adds a reading iff it satisfies the population contract (see block comment).
    // Returns whether the reading entered the pool. Oldest reading drops at capacity.
    // fading_index: the fading observed with this reading (NAN/absent for
    // setMeasuredSNR-only feeds); it NEVER gates admission — the SNR population
    // contract decides, and the fading aggregate simply skips absent values.
    bool addReading(float snr_db, SNRSource source, bool data_aided,
                    float fading_index = NAN) {
        if (!std::isfinite(snr_db)) {
            return false;
        }
        const bool handshake_reading =
            (source == SNRSource::MCDPSK_IN_BAND) && data_aided;
        const bool wire_reading = (source == SNRSource::OFDM_BROADBAND);
        if (!handshake_reading && !wire_reading) {
            return false;
        }
        if (count_ == kCapacity) {
            for (size_t i = 1; i < kCapacity; ++i) {
                readings_[i - 1] = readings_[i];
            }
            --count_;
        }
        readings_[count_++] =
            ConnectSnrReading{snr_db, fading_index, 0, data_aided, source};
        return true;
    }

    // Ages every reading (call from Connection::tick with its elapsed_ms).
    void tick(uint64_t elapsed_ms) {
        for (size_t i = 0; i < count_; ++i) {
            const uint64_t age = readings_[i].age_ms;
            readings_[i].age_ms =
                (age > UINT64_MAX - elapsed_ms) ? UINT64_MAX : age + elapsed_ms;
        }
    }

    // Clustered dB-mean over qualifying readings; NAN when none qualify.
    //   tc_ms          readings < tc_ms apart merge into one cluster (one fade sample)
    //   handshake_only restrict to the entry-pick population (data-aided MCDPSK_IN_BAND)
    //   max_age_ms     freshness gate (UINT64_MAX = no gate; entry pick uses no gate —
    //                  the pool is cleared at connection boundaries so its horizon IS
    //                  the handshake scope; the wire embed gates at 3*Tc)
    float clusteredDbMeanDb(uint64_t tc_ms, bool handshake_only,
                            uint64_t max_age_ms) const {
        int clusters = 0;
        float fading = NAN;
        return aggregate(tc_ms, handshake_only, max_age_ms, clusters, fading);
    }

    // Clustered fading aggregate over the SAME qualifying readings / Tc clusters as
    // the SNR mean: per-cluster MEAN of the finite fading values, then the MEAN of
    // cluster means; NAN when no qualifying reading carries a fading value. MEAN,
    // not median: fading_index is a bounded [0,2] dispersion statistic — no heavy
    // tail for a median to guard against (unlike an unbounded linear-power SNR),
    // at N_eff=2 the median IS the mean, and at the N_eff=2-3 the handshake yields,
    // the mean uses all samples (lowest-variance unbiased estimate) while a median
    // discards all but the middle one. Readings without fading (setMeasuredSNR-only
    // feeds) still delimit clusters (decorrelation is a property of the CHANNEL
    // timeline, not of which fields a feed recorded) but contribute nothing here.
    float clusteredFadingIndex(uint64_t tc_ms, bool handshake_only,
                               uint64_t max_age_ms) const {
        int clusters = 0;
        float fading = NAN;
        (void)aggregate(tc_ms, handshake_only, max_age_ms, clusters, fading);
        return fading;
    }

    // Number of decorrelated clusters among qualifying readings (N_eff).
    int effectiveCount(uint64_t tc_ms, bool handshake_only,
                       uint64_t max_age_ms) const {
        int clusters = 0;
        float fading = NAN;
        (void)aggregate(tc_ms, handshake_only, max_age_ms, clusters, fading);
        return clusters;
    }

    uint64_t freshestAgeMs() const {
        return count_ > 0 ? readings_[count_ - 1].age_ms : UINT64_MAX;
    }

private:
    // One pass computes both aggregates over the SAME cluster partition: the SNR
    // clustered dB-mean (returned) and the clustered fading mean (fading_out; NAN
    // when no qualifying reading in any cluster carries a finite fading value).
    float aggregate(uint64_t tc_ms, bool handshake_only, uint64_t max_age_ms,
                    int& clusters_out, float& fading_out) const {
        double cluster_sum = 0.0;
        int cluster_n = 0;
        double cluster_fading_sum = 0.0;
        int cluster_fading_n = 0;
        double cluster_mean_sum = 0.0;
        int clusters = 0;
        double fading_cluster_mean_sum = 0.0;
        int fading_clusters = 0;
        uint64_t prev_age = 0;
        bool have_prev = false;
        const auto close_cluster = [&]() {
            if (cluster_n > 0) {
                cluster_mean_sum += cluster_sum / cluster_n;
                ++clusters;
            }
            if (cluster_fading_n > 0) {
                fading_cluster_mean_sum += cluster_fading_sum / cluster_fading_n;
                ++fading_clusters;
            }
        };
        for (size_t i = 0; i < count_; ++i) {  // stored oldest -> newest
            const ConnectSnrReading& r = readings_[i];
            if (handshake_only &&
                !((r.source == SNRSource::MCDPSK_IN_BAND) && r.data_aided)) {
                continue;
            }
            if (r.age_ms > max_age_ms) {
                continue;
            }
            // Ages are monotonically non-increasing oldest->newest, so the gap to the
            // previous qualifying reading is prev_age - r.age_ms.
            if (have_prev && (prev_age - r.age_ms) < tc_ms) {
                cluster_sum += r.snr_db;
                ++cluster_n;
            } else {
                close_cluster();
                cluster_sum = r.snr_db;
                cluster_n = 1;
                cluster_fading_sum = 0.0;
                cluster_fading_n = 0;
            }
            if (std::isfinite(r.fading_index)) {
                cluster_fading_sum += r.fading_index;
                ++cluster_fading_n;
            }
            prev_age = r.age_ms;
            have_prev = true;
        }
        close_cluster();
        clusters_out = clusters;
        fading_out = (fading_clusters > 0)
            ? static_cast<float>(fading_cluster_mean_sum / fading_clusters)
            : NAN;
        if (clusters == 0) {
            return NAN;
        }
        return static_cast<float>(cluster_mean_sum / clusters);
    }

    ConnectSnrReading readings_[kCapacity] = {};
    size_t count_ = 0;
};

// Wire stale sentinel: encodeSNR(-10) == wire byte 0, which the receiver ALREADY
// renders as "peer SNR n/a" (app.cpp validity check snr_db >= 0) — zero receiver
// change. Physically collision-free: no control frame decodes at a true -10 dB
// effective SNR (all floors >= 5 dB), so byte 0 is unreachable as a measurement.
inline constexpr float kConnectSnrStaleSentinelDb = -10.0f;

// ...but "unreachable as a measurement" justifies the ENCODING, not the CONSUMPTION.
// The sentinel is a finite float, so every numeric consumer accepts it silently:
// isfinite() passes, comparisons work, and it reads as a catastrophically bad channel.
// IONOS rig 2026-08-05 caught this driving live decisions — `SNR=-10.0 dB
// (ofdm_broadband)` appeared in three consecutive MODE_CHANGE requests
// (R2/3 -> R1/2 -> R1/4) on a link that was transferring a file at the time.
// Any DECISION input must be screened through this; only the wire may carry it.
inline bool isStaleSnrSentinel(float snr_db) {
    // Same 0.5 dB tolerance the GUI already uses for its "n/a" render (app.cpp),
    // so display and decision agree on what counts as "no measurement".
    return !std::isfinite(snr_db) ||
           snr_db <= kConnectSnrStaleSentinelDb + 0.5f;
}

// Pure decision selector (knob- and env-free so it is unit-testable, same pattern as
// the defer predicate below). The WIRE may carry "unknown"; a DECISION may not.
//
// Cost asymmetry sets the direction: mistaking a genuine -10 dB channel for "unknown"
// costs nothing — no rung is usable there and every floor is >= 5 dB — while mistaking
// "unknown" for -10 dB collapses a healthy link to the most conservative geometry.
// So at the encodable floor we err toward "no measurement" and keep the last real one.
inline float decisionSnrFromWire(float wire_db, float last_real_db) {
    return isStaleSnrSentinel(wire_db) ? last_real_db : wire_db;
}
// A fade-state estimate decorrelates in Tc (Clarke/Jakes A(Tc)=0.5); nothing newer
// than 3*Tc means the pool no longer describes the CURRENT link -> send the sentinel
// instead of a frozen number (W2 shipped 16.5 for 40 s / 22.0 for 40+ s).
inline constexpr uint32_t kConnectWireSnrFreshTcMultiple = 3;

// ULTRA_CONNECT_SNR_POOL (default OFF => byte-identical): entry pick + CONNECT_ACK
// wire byte use the pool's clustered dB-mean instead of the last-write-wins scalar.
// The pool still ACCUMULATES silently when OFF (output-identical). Read ONCE (static).
inline bool connectSnrPoolEnabled() {
    static const bool v = [] {
        const char* e = std::getenv("ULTRA_CONNECT_SNR_POOL");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return v;
}

// ULTRA_CONNECT_PICK_DEFER (default OFF; requires ULTRA_CONNECT_SNR_POOL): auto-accept
// only — when the pool has ONE effective (decorrelated) reading on a fading channel and
// the pick would land sub-OFDM (the 15-40x cost-asymmetry zone), withhold CONNECT_ACK
// once and let the initiator's EXISTING CONNECT retransmit deliver a decorrelated
// second reading. Wire-compatible: a deferred CONNECT is indistinguishable from a
// decode failure to the initiator. Read ONCE (static).
inline bool connectPickDeferEnabled() {
    static const bool v = [] {
        const char* e = std::getenv("ULTRA_CONNECT_PICK_DEFER");
        return e != nullptr && std::atoi(e) != 0;
    }();
    return v;
}

// ULTRA_WIRE_SNR_FRESH (DEFAULT-ON since 2026-07-05 — the "default OFF => wire bytes
// unchanged" text here was stale and contradicted the code below; corrected 2026-08-05):
// MODE_CHANGE embeds use the pool mean over readings younger than 3*Tc, else the -10 dB
// stale sentinel (wire byte 0 = the receiver's existing "n/a" rendering). Read ONCE
// (static) — so a test binary that pins this env var latches it process-wide.
// Because it is DEFAULT-ON, the sentinel is reachable in production: see
// decisionSnrFromWire() above for why it must never reach a decision.
inline bool wireSnrFreshEnabled() {
    static const bool v = [] {
        const char* e = std::getenv("ULTRA_WIRE_SNR_FRESH");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return v;
}

// Pure defer predicate (knob- and env-free so it is unit-testable): defer exactly once,
// only when the aggregate rests on a single fade sample (N_eff==1), only on a fading
// channel (AWGN readings are stationary — one sample IS the mean), and only when the
// pick would land sub-OFDM (MC-DPSK; wrongly entering OFDM low still delivers ~450 bps,
// wrongly falling to MC-DPSK costs 15-40x — the asymmetry that justifies buying one
// more sample). N_eff==0 means the pool has no qualifying reading at all: the pick is
// already on the scalar fallback path — nothing to defer for.
inline bool shouldDeferConnectPick(int effective_count, float fading_index,
                                   WaveformMode selected_waveform,
                                   bool already_deferred) {
    return !already_deferred &&
           effective_count == 1 &&
           fading_index >= kFadingAwgnMax &&
           selected_waveform == WaveformMode::MC_DPSK;
}

// #58 connect-time SNR basis correction (2026-07-01, fable_analysis/09 §4).
// The OFDM entry floors and coherent-ladder anchors are DIAL-calibrated (forced-rung
// sim sweeps at --snr-db = the AWGN-equivalent dial), but the connect-time reading
// compared against them (#74 ratiometric MC-DPSK training SNR) is FADE-EFFECTIVE and
// INSTANT: on a fading channel it averages ~2 dB below the dial (Jensen penalty over
// the ~170 ms << Tc training window) and swings with the fade phase run-to-run.
// Comparing effective-instant readings against dial thresholds double-penalizes — a
// single fade dip at CONNECT dropped a dial-20 rig channel (sync SNR 21.8) to a 12.4
// reading -> below the Moderate entry floor 14 -> MC-DPSK DBPSK (~94 bps nominal, 0
// bytes delivered) on a channel that carries ~1.5 kbps at QPSK R2/3. Align the bases:
// on any FADING channel the SELECTION comparison adds back the measured penalty. The
// reported/wire SNR stays raw (honest measurement); MC-DPSK-internal rung floors were
// themselves calibrated against this same ratiometric reading on the rig (#71
// floor-finding, already effective-basis) — the +2 there equals lowering the DQPSK
// floor by 2, which the #71 rig data supports (DQPSK 3/3 across effective 2.4-9 dB)
// and whose comment anticipated pending #58. Cost asymmetry backs the sign: wrongly
// entering OFDM at true-effective ~10 still delivers ~450 bps (Good@10 R1/2 5/5,
// RATE_LADDER_ANCHORS), while wrongly falling to MC-DPSK costs 15-40x.
// Entry-time CLASSIFICATION shrinkage (2026-07-03, docs/CONNECT_ENTRY_CALIBRATION
// _2026_07_03.md): a single CONNECT frame's fading index at TRUE Watterson Good
// measures sigma = 0.129 (N=48 rig entries at a known dial), so a raw reading in
// (0.65, 0.78] is within 1 sigma of the Good/Moderate boundary — 18.8% of Good
// entries false-classified Moderate and 8/9 of those entered QPSK R1/4
// (~100-200 s of climb-back each). (BUG-MPG20-OVER-DEMOTE-R14, 2026-07-14: the
// class boundary kFadingGoodMax has since moved to the ML midpoint 0.76, which
// absorbs most of that band directly; this shrink now stacks as a small belt-and-
// suspenders at ENTRY, where N_eff is small — and it is the ONLY mitigation the raw
// per-group RX-authority verdict lacked before the boundary move.) The
// misclassification costs are ASYMMETRIC:
// false-Moderate = minutes of low-rung crawl (climbs need clean-group streaks);
// false-Good = one bad group (~16 s) before the prompt demote. So shrink the
// CLASSIFICATION input toward Good by the standard error of the estimate:
// sigma/sqrt(N_eff). Pooled estimates shrink less (they've earned confidence);
// callers apply this ONLY to class-keyed decisions at ENTRY and only when the
// pool knob is on — numeric consumers and the scalar path keep the raw value.
inline constexpr float kConnectFadingSigmaGood = 0.129f;
inline float entryClassificationFadingIndex(float pooled_fading, int n_eff) {
    const float shrink =
        kConnectFadingSigmaGood / std::sqrt(static_cast<float>(std::max(n_eff, 1)));
    return std::max(0.0f, pooled_fading - shrink);
}

// ULTRA_CONNECT_SNR_FADE_BASIS=0 disables; a value in (0,6] overrides the 2.0 default.
inline float connectSnrFadeBasisDb() {
    static const float v = [] {
        if (const char* e = std::getenv("ULTRA_CONNECT_SNR_FADE_BASIS")) {
            const float parsed = std::strtof(e, nullptr);
            if (parsed <= 0.0f) return 0.0f;
            if (parsed <= 6.0f) return parsed;
        }
        return 5.0f;  // 2026-07-02 recalibration: the connect reading is now the
                      // DATA-AIDED fade-AVERAGED estimate (#58 increment 2), whose
                      // offset vs the dial-calibrated floors is LARGER than the old
                      // snapshot's +2 (AWGN-only calibration misses the fading EVM
                      // inflation): measured across 7 rig+sim Good connects,
                      // data_aided reads 9.8-13.7 at dial/effective ~16-20 → ~5 dB.
                      // Consequences at +5: sim good@20 s43 (10.6→15.6) clears the
                      // Moderate floor; rig picks recover R2/3 (yesterday 2/3 fell
                      // to R1/2); true-low-SNR safety holds (MPM@8 effective 1-5
                      // → 6-10 < floors → MC-DPSK unchanged). AWGN untouched
                      // (fading gate). Revisit when the estimator itself gains a
                      // sim-Good calibration term (the cleaner long-term home).
    }();
    return v;
}
// ═══════════ Calibrated AFFINE entry-SNR basis (ULTRA_CONNECT_AFFINE_BASIS) ═══════════
// 2026-07-03, docs/CONNECT_ENTRY_CALIBRATION_2026_07_03.md §7. The flat +5 basis is the
// wrong MODEL at the tails: 48 rig entries at a KNOWN dial (MPG@20 Watterson Good) read
// 6.2-19.4 dB data-aided (mean 12.38, sigma 3.14), offset to dial mean -7.62 dB and
// reading-dependent (below-median readings -10.2, above-median -5.1) — a constant either
// under-corrects troughs (dial-20 connects with 9-11 readings entered QPSK R1/2 on a
// channel that carries R2/3) or over-corrects crests.
// THE FIT (least squares, dial on reading, all 48 targets = 20.0): EXACT and DEGENERATE —
//   a = 0, b = 20, in-sample residual sigma = 0 (equivalently offset-on-reading slope
//   -1, intercept 20: offset := 20 - reading is an exact affine function of the reading).
// What that degeneracy MEANS physically: within the calibrated population the reading
// carries NO dial information — the 6.2-19.4 spread is fade-phase sampling noise around
// ONE dial. The doc §2 "SNR-dependent offset" is, at a single dial, pure regression to
// the mean; a one-dial design cannot identify the dial-vs-reading slope (that needs a
// multi-dial sweep — see the extrapolation caveat below).
// DEPLOYED CONSTANTS:
//   - intercept: b shrunk by ONE standard error of the calibrated mean,
//     20 - sigma/sqrt(N) = 20 - 3.14/sqrt(48) = 19.55 dB (same one-sided-cost shrink as
//     entryClassificationFadingIndex above, and it keeps mid readings from landing
//     exactly ON the zero-margin Good QPSK R3/4 anchor at 20.0);
//   - correction = clamp(19.55 - reading, +2, +11) dB. The clamp IS the extrapolation
//     guard: outside the calibrated reading range the affine map degrades to a flat
//     basis (slope-1 in dial_equiv) at the nearer clamp edge, so an uncalibrated-low
//     reading (true low dial — e.g. genuine dial-10 readings ~2-6) gets at most +11,
//     and a crest reading gets at least the +2 the #58 Jensen argument supports.
// CALIBRATION SCOPE (both are A/B risks the default-OFF knob exists to measure):
//   - single channel condition (dial-20 Watterson Good, IONOS rig) — the map is
//     EXTRAPOLATED at any other dial: below ~reading 8.5 it asserts "a trough at the
//     calibration dial" where the truth could be a genuinely weak channel;
//   - DATA-AIDED estimator only ('local_measured' ledger lines). The selection path
//     applies it ONLY to data-aided readings (connectSelectionSnrDb below): the
//     training snapshot fade-crest OVER-reads (7.8 measured at true Moderate@8) and
//     +11 on an over-read crest would put a true-8dB channel deep into OFDM territory
//     — training readings keep the flat basis.
inline bool connectAffineBasisEnabled() {
    static const bool v = [] {
        const char* e = std::getenv("ULTRA_CONNECT_AFFINE_BASIS");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return v;
}
// Golden fit constants (pinned; derivation above + test_connection_policy.cpp).
inline constexpr float kConnectAffineFitSlope = 0.0f;    // LS slope a (exact, degenerate)
inline constexpr float kConnectAffineFitInterceptDb = 20.0f;  // LS intercept b = the dial
inline constexpr float kConnectEntryReadingSigmaDb = 3.14f;   // sample sigma, 48 readings
inline constexpr int kConnectEntryCalibCount = 48;
// Deployed intercept: b - sigma/sqrt(N) = 20 - 3.14/sqrt(48) = 19.547 -> 19.55.
inline constexpr float kConnectAffineDialEquivDb = 19.55f;
inline constexpr float kConnectAffineCorrMinDb = 2.0f;
inline constexpr float kConnectAffineCorrMaxDb = 11.0f;

inline float connectAffineCorrectionDb(float reading_db) {
    // Slope-0 fit => correction = (deployed intercept) - reading, clamped [+2, +11].
    const float corr = kConnectAffineDialEquivDb - reading_db;
    return std::min(kConnectAffineCorrMaxDb, std::max(kConnectAffineCorrMinDb, corr));
}

// ONE source of truth for reading -> dial-equivalent SNR: the entry SELECTION
// (connectSelectionSnrDb below) and BOTH GUI dial-equivalent displays (status bar +
// sidebar "dB eff" sites, app.cpp) call this, so the dial number the operator sees is
// the number the entry pick compares against the dial-calibrated floors/anchors.
// affine_enabled is passed explicitly (pure/unit-testable — the env read is latch-once);
// production callers pass connectAffineBasisEnabled() (displays) or compose it with
// data-aidedness (selection).
inline float dialEquivalentSnrDb(float reading_db, float fading_index,
                                 bool affine_enabled) {
    if (fading_index < kFadingAwgnMax) {
        return reading_db;  // AWGN: reading and dial thresholds share the basis already
    }
    if (affine_enabled) {
        return reading_db + connectAffineCorrectionDb(reading_db);
    }
    return reading_db + connectSnrFadeBasisDb();
}

// Pure 4-arg form (knob passed explicitly for tests); the 3-arg production wrapper
// below reads ULTRA_CONNECT_AFFINE_BASIS. The affine basis composes with data-
// aidedness here: it is calibrated on the data-aided estimator ONLY (see the block
// comment above), so a training-routed reading keeps the flat basis even when the
// knob is ON.
inline float connectSelectionSnrDb(float measured_snr_db, float fading_index,
                                   bool snr_is_data_aided,
                                   bool affine_basis_enabled,
                                   float physical_mean_db = 0.0f,
                                   uint32_t physical_n = 0) {
    if (fading_index >= kFadingAwgnMax) {
        float sel = dialEquivalentSnrDb(measured_snr_db, fading_index,
                                        affine_basis_enabled && snr_is_data_aided);
        // SATURATION BOUND (2026-07-02, Moderate-class fading): the data-aided
        // differential-EVM estimator saturates at the Doppler-EVM floor (~7-9 dB
        // at 0.5 Hz Doppler — the fade itself injects differential error), so on
        // fast fading NO additive basis can recover the true SNR: MPM@20 reads
        // 7.7 while MPM@8 reads 1-5 (measured). But EVM only ADDS error, so the
        // reading is a LOWER BOUND on true SNR: a reading AT/ABOVE the
        // saturation zone (>= 6.5) on Moderate-class fading implies the true
        // channel is >= ~10+ dB (a genuinely weak channel pulls the reading
        // BELOW saturation) -> OFDM entry is justified; the ladder's
        // conservative R1/4 Moderate entry + demote machinery makes a marginal
        // call recoverable. Below the zone the reading is trustworthy and the
        // MC-DPSK fallback logic applies unchanged (MPM@8 safety preserved).
        // The lower-bound argument holds ONLY for the data-aided estimator: the
        // training snapshot fade-crest OVER-reads (measured up to 7.8 at true
        // Moderate@8), so a training-routed reading must never trip this.
        // The 6.5 zone test stays keyed to the RAW reading under the affine basis
        // too (the saturation physics is about what the ESTIMATOR emitted, not the
        // corrected value); only the sel it maxes against changes.
        if (snr_is_data_aided &&
            fading_index >= kFadingGoodMax && measured_snr_db >= 6.5f) {
            sel = std::max(sel, kOFDMEntryFloorModerateDb + 0.5f);
        }
        // ═══ PHYSICAL ENTRY CAP (handoff §6, 2026-07-08) ═══
        // The dial-equivalent estimate can never exceed what the channel
        // PHYSICALLY measured: the per-frame physical readings (power ratio of
        // the training span vs the frame's burst-time noise, linear-mean over
        // the handshake — the dial convention by construction, and EVM-
        // unsaturated so it composes with the Moderate bound above: at MPM@20
        // physical reads ~20 and never clamps it). The affine boost bets a
        // snapshot was a trough; when the physically measured channel says
        // otherwise, the physics wins. CAP ONLY — entries can only get MORE
        // conservative (worst case: MC-DPSK entry, recoverable by the ladder).
        // Margin +2.0 dB = the payload-referenced definition's ~0.5 dB offset
        // from dial + ~1.5 dB single/few-frame measurement spread. AWGN is
        // outside this branch entirely (reading==dial basis there).
        if (physical_n >= 1 && physical_mean_db > 0.0f) {
            sel = std::min(sel, physical_mean_db + 2.0f);
        }
        return sel;
    }
    return measured_snr_db;  // AWGN: reading and thresholds share the basis already
}
inline float connectSelectionSnrDb(float measured_snr_db, float fading_index,
                                   bool snr_is_data_aided,
                                   float physical_mean_db, uint32_t physical_n) {
    return connectSelectionSnrDb(measured_snr_db, fading_index, snr_is_data_aided,
                                 connectAffineBasisEnabled(),
                                 physical_mean_db, physical_n);
}

inline float connectSelectionSnrDb(float measured_snr_db, float fading_index,
                                   bool snr_is_data_aided) {
    return connectSelectionSnrDb(measured_snr_db, fading_index, snr_is_data_aided,
                                 connectAffineBasisEnabled());
}

inline LadderRung selectLadderRung(float snr_db, ChannelClassification channel) {
    // DIAGNOSTIC force (ULTRA_FORCE_MCDPSK_RUNG=LOW|MID|ROBUST|STANDARD): pin the
    // MC-DPSK rung, bypassing the SNR thresholds, to MEASURE each rung's real floor
    // on the faithful gate (#71). The DQPSK rungs ROBUST (1024) / STANDARD (512) are
    // otherwise UNREACHABLE (robust_floor > ofdm_floor; STANDARD never returned), so
    // their floors are unknown. No-op unless set; not a production path.
    if (const char* e = std::getenv("ULTRA_FORCE_MCDPSK_RUNG")) {
        const std::string s(e);
        if (s == "LOW")      return ladderRungForId(LadderRungId::ROBUST_LOW);
        if (s == "MID")      return ladderRungForId(LadderRungId::ROBUST_MID);
        if (s == "ROBUST")   return ladderRungForId(LadderRungId::ROBUST);
        if (s == "STANDARD") return ladderRungForId(LadderRungId::STANDARD);
    }
    float ofdm_floor = 10.0f;
    float robust_mid_floor = 5.0f;  // DBPSK R1/4 floor (the robust MC-DPSK fallback)

    // OFDM entry floors are the SINGLE source in waveform_selection.hpp
    // (kOFDMEntryFloor*Db) so this enum-keyed path and the fading-index-keyed
    // recommendWaveformAndRate() can't drift. robust_mid_floor is the MC-DPSK
    // DBPSK floor, local to this ladder.
    switch (channel) {
        case ChannelClassification::AWGN:
            ofdm_floor = kOFDMEntryFloorAwgnDb;
            robust_mid_floor = 5.0f;
            break;
        case ChannelClassification::GOOD:
            ofdm_floor = kOFDMEntryFloorGoodDb;
            robust_mid_floor = 6.0f;
            break;
        case ChannelClassification::MODERATE:
            ofdm_floor = kOFDMEntryFloorModerateDb;
            robust_mid_floor = 7.0f;
            break;
        case ChannelClassification::POOR:
            ofdm_floor = kOFDMEntryFloorPoorDb;
            robust_mid_floor = 9.0f;
            break;
    }

    // #71: the DQPSK rung (ROBUST = DQPSK/1024/R1/4, ~2x DBPSK throughput) is now
    // REACHABLE in the MC-DPSK sub-band. The old robust_floor (13-17) sat ABOVE
    // ofdm_floor (10-14), so the DQPSK interval [robust_floor, ofdm_floor) was EMPTY
    // and MC-DPSK was pinned to DBPSK R1/4. Partition the sub-band by per-rung
    // GEOMETRY: DQPSK R1/4 needs ~+2.5 dB over DBPSK (the differential BPSK->QPSK gap).
    // MEASURED on BENIGN channels: forced DQPSK decodes CRC-clean at good@8/10/12 (0
    // cw_fail), matching the +2.3 dB prediction; the +2.5 dB floor sits with hysteresis
    // above that, clamped just below ofdm_floor.
    //
    // FAST-FADING (Moderate/Poor): the DIFFERENTIAL demod compounds the Doppler phase
    // error symbol-to-symbol, so DQPSK costs MORE than +2.5 dB there and its floor is
    // UNMEASURED. Keep DQPSK unreachable on fading channels (DBPSK only) until a fading
    // floor-probe sets a principled threshold — DBPSK is the robust differential pick
    // for fast fading anyway. DQPSK is already wired end-to-end; control stays fixed
    // DBPSK/1024 (#72), so this is a pure selector change.
    float robust_dqpsk_floor;
    switch (channel) {
        case ChannelClassification::AWGN:
        case ChannelClassification::GOOD:
            // #71: with the DQPSK window spiral fixed (round-trip-safe window=3, above),
            // rig A/B @ MPG@9 showed DQPSK-window=3 matches DBPSK reliability (3/3 vs 3/3)
            // at ~2x goodput across effective connect-SNR ~2.4-9 dB. The old +2.5 dB
            // differential BPSK->QPSK geometry margin (floor 8.5) is an AWGN bound and
            // over-penalizes DQPSK on FADING, where ARQ means only the good-fade frames
            // must decode. Lower to a validated-conservative +1.0 dB over the DBPSK floor
            // (Good 7.0, AWGN 6.0), clamped below the OFDM floor, keeping a thin DBPSK-mid
            // buffer above the deep-fade fallback. NOTE: going lower (toward the DBPSK
            // floor) is supported by the data but needs MPG@8/7 floor-finding + a
            // fade-AVERAGED connect SNR (task #58) — the single-snapshot reading is noisy,
            // so dial-9 auto-selection is not yet deterministic.
            robust_dqpsk_floor = std::min(robust_mid_floor + 1.0f, ofdm_floor - 0.5f);
            break;
        case ChannelClassification::MODERATE:
            robust_dqpsk_floor = 15.0f;  // UNCHANGED (old robust_floor): > ofdm 14 ->
                                         // DBPSK only; fading DQPSK floor unmeasured.
            break;
        case ChannelClassification::POOR:
        default:
            robust_dqpsk_floor = 17.0f;  // UNCHANGED (old robust_floor).
            break;
    }

    if (snr_db >= ofdm_floor) {
        return ladderRungForId(LadderRungId::OFDM_CHIRP);
    }
    if (snr_db >= robust_dqpsk_floor) {
        return ladderRungForId(LadderRungId::ROBUST);  // DQPSK R1/4 — ~2x DBPSK
    }
    if (snr_db >= robust_mid_floor) {
        return ladderRungForId(LadderRungId::ROBUST_MID);  // DBPSK R1/4
    }
    return ladderRungForId(LadderRungId::ROBUST_LOW);  // DBPSK R1/4 (deepest fallback)
}

inline LadderRung selectLadderRung(float snr_db, float fading_index) {
    return selectLadderRung(snr_db, classifyChannel(fading_index));
}

inline uint8_t modeToCapabilityBit(WaveformMode mode) {
    switch (mode) {
        case WaveformMode::OFDM_CHIRP:  return ModeCapabilities::OFDM_CHIRP;
        case WaveformMode::OFDM_NARROW: return ModeCapabilities::OFDM_NARROW;
        case WaveformMode::OTFS_EQ:     return ModeCapabilities::OTFS_EQ;
        case WaveformMode::OTFS_RAW:    return ModeCapabilities::OTFS_RAW;
        case WaveformMode::MFSK:        return ModeCapabilities::MFSK;
        case WaveformMode::MC_DPSK:     return ModeCapabilities::MC_DPSK;
        default: return 0;
    }
}

inline bool isNearAwgnOFDM(float fading_index, float snr_db) {
    return fading_index < 0.15f && snr_db >= 25.0f;
}

inline bool isHighThroughputOFDM(float fading_index, float snr_db) {
    return fading_index < 0.65f && snr_db >= 25.0f;
}

inline bool isHighThroughputOFDMMode(Modulation mod, CodeRate rate) {
    // High-throughput predicate gates window=16 selective-repeat
    // (vs window=8 default). DQPSK at R1/2+ uses bigger window because
    // fading correlation across an 8-frame burst is tolerable; D8PSK
    // gets the same treatment because the 2026-05-04 D8PSK gate only
    // fires when the channel is good enough to support it (in-band SNR>=20
    // fading<0.65 minimum), which is the same precondition the larger
    // window assumes.
    if (mod == Modulation::DQPSK || mod == Modulation::D8PSK) {
        const auto* descriptor = ofdmCodeRateDescriptor(rate);
        const auto* floor = ofdmCodeRateDescriptor(CodeRate::R1_2);
        return descriptor != nullptr && floor != nullptr &&
               descriptor->code_rate >= floor->code_rate;
    }
    return false;
}

inline bool isSpeculativeHighRateOFDM(Modulation mod, CodeRate rate) {
    // R2/3 and R3/4 are speculative (window=16 only on near-AWGN);
    // R1/2 is non-speculative (window=16 always when fading channel
    // is good). Both DQPSK and D8PSK follow the same logic.
    const auto* descriptor = ofdmCodeRateDescriptor(rate);
    const auto* floor = ofdmCodeRateDescriptor(CodeRate::R1_2);
    const bool risky_rate = descriptor != nullptr && floor != nullptr &&
                            descriptor->code_rate > floor->code_rate;
    return risky_rate && (mod == Modulation::DQPSK || mod == Modulation::D8PSK);
}

inline bool isBurstInterleavedOFDMMode(Modulation mod, CodeRate rate) {
    // Coherent 8PSK/16-QAM need the per-frame chirp/LTS channel reference while
    // the 2026-05 burst-accumulation path is still being reworked. Round-5 Good
    // fading A/B showed burst groups 2/4/8 lose to independent coherent frames
    // because continuation timing failures dominate the diversity gain.
    return isSpeculativeHighRateOFDM(mod, rate);
}

// Wide coherent ARQ window (ULTRA_COHERENT_WINDOW, 2026-07-02; DEFAULT-ON 16 since
// 2026-07-03). The coherent mods (QPSK/8PSK/16QAM) historically ran the default window
// (kWideOFDMWindowFrames=8) — the high-throughput window=16 predicate was
// differential-only (isHighThroughputOFDMMode), a fossil of the retired
// differential-OFDM wideband. With the tone-burst SACK frame_mask widened to 16 bits, a
// coherent >=R2/3 burst carries up to 16 selectively-ackable frames per key-down (fewer
// half-duplex turnarounds per delivered byte). A/B evidence (2026-07-03, 50KB faithful
// gate, all CRC-clean): Good@20 s42 1940->2280, s43 1210->1900, s7 1310->1650
// (mean +31%), AWGN@20 3370->3520 (record) — 4/4 cells up, so the override is now the
// DEFAULT. Env: unset -> 16; 8 restores the legacy window; 0 disables (legacy 8 via
// fall-through); clamp [0,16]. Read once (static). NOTE: the burst airtime budget
// (connection.cpp burstAirtimeBudgetFrames) still mins the real burst size against the
// PA-duty airtime ceiling; this widens the ARQ window, not the duty cycle.
inline int coherentOFDMWindowOverride() {
    static const int v = [] {
        if (const char* e = std::getenv("ULTRA_COHERENT_WINDOW")) {
            return std::clamp(std::atoi(e), 0,
                              static_cast<int>(kToneBurstAckWindowCapFrames));
        }
        return 16;
    }();
    return v;
}

inline size_t ofdmWindowSize(Modulation mod, CodeRate rate, bool near_awgn_ofdm) {
    // Coherent wide-window override (A/B, default-off). Coherent mods only — the
    // differential DQPSK/D8PSK path keeps its own high-throughput predicate below —
    // and only at rate >= R2/3 (the rungs the ladder gates to clean channels; the
    // robust low rates keep the conservative window). With the env unset this block
    // is a no-op and the selection is byte-identical to the pre-knob behavior.
    const int coherent_override = coherentOFDMWindowOverride();
    if (coherent_override > 0 && ofdm_link_adaptation::isCoherentModulation(mod)) {
        const auto* descriptor = ofdmCodeRateDescriptor(rate);
        const auto* floor = ofdmCodeRateDescriptor(CodeRate::R2_3);
        if (descriptor != nullptr && floor != nullptr &&
            descriptor->code_rate >= floor->code_rate) {
            return static_cast<size_t>(coherent_override);
        }
    }

    if (!isHighThroughputOFDMMode(mod, rate)) {
        return kWideOFDMWindowFrames;
    }

    if (isSpeculativeHighRateOFDM(mod, rate) && !near_awgn_ofdm) {
        return kWideOFDMWindowFrames;
    }

    return kHighThroughputOFDMWindowFrames;
}

inline size_t ofdmWindowSize(Modulation mod, CodeRate rate) {
    return ofdmWindowSize(mod, rate, true);
}

inline size_t ofdmWindowSizeForChannel(Modulation mod,
                                       CodeRate rate,
                                       float fading_index,
                                       float snr_db) {
    return ofdmWindowSize(mod, rate, isNearAwgnOFDM(fading_index, snr_db));
}

inline bool shouldPadHighRateFadingBurst(Modulation mod,
                                         CodeRate rate,
                                         bool near_awgn_ofdm,
                                         size_t burst_frames) {
    if (!isSpeculativeHighRateOFDM(mod, rate) || near_awgn_ofdm) {
        return false;
    }
    if (burst_frames <= 1) {
        return false;
    }

    return (burst_frames % burstInterleaveGroupFrames()) != 0;
}

inline bool shouldPadBurstInterleaveGroup(size_t burst_frames) {
    if (burst_frames <= 1) {
        return false;
    }
    return (burst_frames % burstInterleaveGroupFrames()) != 0;
}

inline uint32_t ofdmAckBatchSize(bool near_awgn_ofdm) {
    (void)near_awgn_ofdm;
    return 0;
}

inline uint32_t bitsPerOFDMSymbol(uint32_t carriers,
                                  bool include_pilots,
                                  int pilot_spacing,
                                  Modulation mod) {
    return static_cast<uint32_t>(ofdm_link_adaptation::bitsPerOFDMSymbol(
        static_cast<int>(carriers), include_pilots, pilot_spacing, mod));
}

inline uint32_t wideOFDMSymbolsForCodewords(Modulation mod, CodeRate rate, int codewords,
                                            int lifting_z = 27) {
    const int pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    const uint32_t bits_per_symbol = bitsPerOFDMSymbol(
        kWideOFDMCarriers, true, pilot_spacing, mod);
    // z-AWARE: coded bits per codeword is N = z×24 (648 at z=27, 1944 at z=81). A z=81
    // frame carries ~3× the coded bits per codeword, so it occupies ~3× the data symbols.
    const uint32_t frame_bits =
        static_cast<uint32_t>(codewords) * ldpcCodewordBits(lifting_z);
    const uint32_t data_symbols = (frame_bits + bits_per_symbol - 1) / bits_per_symbol;
    return 2 + data_symbols;
}

// Sample-exact counterpart to wideOFDMFrameTimingForCodewords() for the live
// ModemEngine/StreamingEncoder geometry.  The returned extent includes the two
// LTS symbols that connectedDataPreambleForFrame() emits, but not a chirp.
inline uint64_t wideOFDMWireFrameSamplesForCodewords(
        Modulation mod,
        CodeRate rate,
        int codewords,
        int lifting_z = 27) {
    return static_cast<uint64_t>(wideOFDMSymbolsForCodewords(
               mod, rate, std::clamp(codewords, 1, 255), lifting_z)) *
           kWideOFDMWireSymbolSamples;
}

// Exact default descriptor-bearing burst shape emitted by encodeBurstLight().
// A singleton has no descriptor and goes through encodeFrame().  Every
// multi-frame turn has one full QPSK-R1/4 descriptor; a repair/mode-switch can
// additionally replace the DATA group-start's light LTS with a full chirp.
inline uint64_t wideOFDMWireBurstSamples(
        Modulation data_mod,
        CodeRate data_rate,
        size_t frame_count,
        int cw_count = v2::kDefaultFixedFrameCodewords,
        int data_lifting_z = 27,
        bool force_full_group_start = false,
        Modulation control_mod = Modulation::QPSK,
        uint32_t continuation_reanchor_ms = 0) {
    if (frame_count == 0) return 0;

    uint64_t samples = kWideOFDMFullAnchorExtraSamples;
    if (frame_count > 1) {
        samples += wideOFDMWireFrameSamplesForCodewords(
            control_mod, CodeRate::R1_4, 1, 27);
    }
    samples += static_cast<uint64_t>(frame_count) *
               wideOFDMWireFrameSamplesForCodewords(
                   data_mod, data_rate, cw_count, data_lifting_z);
    if (frame_count > 1) {
        samples += static_cast<uint64_t>(frame_count - 1) *
                   txGuardSamplesForMs(
                       static_cast<int>(continuation_reanchor_ms));
        if (force_full_group_start) {
            samples += kWideOFDMFullAnchorExtraSamples;
        }
    }
    return samples;
}

inline uint32_t wideOFDMShortReanchorChirpDurationMs() {
    static const uint32_t duration_ms = [] {
        const char* value = std::getenv("ULTRA_SHORT_REANCHOR_CHIRP_MS");
        if (!value || value[0] == '\0') {
            return kWideOFDMShortReanchorDefaultMs;
        }

        char* end = nullptr;
        const float parsed = std::strtof(value, &end);
        if (end == value || !std::isfinite(parsed)) {
            return kWideOFDMShortReanchorDefaultMs;
        }
        const uint32_t rounded = static_cast<uint32_t>(parsed + 0.5f);
        return std::clamp<uint32_t>(rounded,
                                    kWideOFDMShortReanchorMinMs,
                                    kWideOFDMShortReanchorMaxMs);
    }();
    return duration_ms;
}

inline bool shouldUseWideOFDMShortReanchor(WaveformMode waveform,
                                           Modulation modulation,
                                           float fading_index) {
    // RETIRED 2026-07-07 (GROUP-SIZE LEVER, docs/GROUP_SIZE_LEVER_2026_07_07.md
    // §2.1): the encoder's adaptive short-chirp continuation re-anchor was
    // REMOVED in May (R4 — superseded by warm-handoff; streaming_encoder.cpp
    // "the adaptive short-chirp re-anchor was removed"), but this predicate
    // kept charging a PHANTOM 100 ms/frame in every airtime budget/timeout
    // model — the F163 sample-exact wire model (S(N) = N×59,360 + 67,680)
    // proves nothing rides between frames. The phantom pinned Good-era groups
    // at N=5 (5×1272+4×100+1200 = 7960 ≤ 8600; a 6th "cost" 9332) and made
    // N=8 unreachable even via the env ceiling. Always false now; the charge
    // sites all derive 0. Function kept so call sites need no churn; delete
    // with the R4 sweep (REMOVAL_BACKLOG).
    (void)waveform;
    (void)modulation;
    (void)fading_index;
    return false;
}

// Channel gate for the Phase 2a warm SHORT-DUAL descriptor anchor
// (ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS). The short anchor reclaims ~400-600 ms/burst of chirp
// airtime but trades ~1.5-3 dB of matched-filter margin vs the full 500 ms dual. GUI-measured
// (2026-06-13, gui_qso_scenario): on fading that margin loss produces a fat tail of
// descriptor-miss / fade-alignment storms whose worst-case seed RELOCATES with chirp duration
// but never disappears (250 ms cratered Moderate seed 2 @88 retx; 350 ms fixed it but cratered
// seed 43 @44 retx/96 CW-fail and dragged Good seed 2 to -18%). It is a clean win ONLY on benign
// Good/AWGN (250 ms = +7.2%, 3/3 seeds, 0 CW-fail, no crater).
//
// Gate on BENIGN-channel operating points (robust SENDER-SIDE proxies the ladder only sustains
// at high SNR + shallow fading, unlike the raw fading_index which suffers the Good/Moderate
// classifier blindness — the measured fading distributions overlap):
//   - QPSK R3/4: the ladder's top rung (entry floors + adaptive hysteresis), OR
//   - dense coherent mods (>=16QAM, 2026-06-14): the ladder only SELECTS these on benign Good
//     channels, so 16QAM at any rate IS a benign operating point. The reclaim is proportionally
//     BIGGER here — a denser payload packs the burst's data into fewer symbols, so the fixed
//     descriptor chirp is a larger fraction of the burst.
// When the channel degrades the ladder drops below R3/4 / off 16QAM and the short anchor
// auto-reverts to the full dual chirp, mid-session.
inline bool shouldUseWarmShortAnchorDescriptor(WaveformMode waveform,
                                               Modulation modulation,
                                               CodeRate rate) {
    if (waveform != WaveformMode::OFDM_CHIRP ||
        !ofdm_link_adaptation::isCoherentModulation(modulation)) {
        return false;
    }
    return rate == CodeRate::R3_4 || getBitsPerSymbol(modulation) >= 4;
}

// Exact physical timing for a serialized DATA frame. Unlike wideOFDMFrameTiming(),
// this deliberately accepts the full uint8_t total_cw wire range: the single-block
// file path uses variable-CW frames larger than the fixed/interleaved 16-CW ceiling.
inline OFDMFrameTiming wideOFDMFrameTimingForCodewords(Modulation mod,
                                                       CodeRate rate,
                                                       int cw_count,
                                                       int data_lifting_z = 27) {
    cw_count = std::clamp(cw_count, 1, 255);
    constexpr float symbol_ms =
        (1000.0f * static_cast<float>(kWideOFDMSymbolSamples)) /
        static_cast<float>(kOFDMSampleRate);

    OFDMFrameTiming timing;
    // DATA frames carry the negotiated lifting z (27 short / 81 long); the ACK/control
    // frame is ALWAYS a 1-CW short (z=27) frame, so its airtime is z-independent.
    timing.data_symbols = wideOFDMSymbolsForCodewords(mod, rate, cw_count, data_lifting_z);
    timing.ack_symbols = wideOFDMSymbolsForCodewords(mod, rate, 1, /*lifting_z=*/27);
    timing.data_ms = static_cast<uint32_t>(timing.data_symbols * symbol_ms + 0.5f);
    timing.ack_ms = static_cast<uint32_t>(timing.ack_symbols * symbol_ms + 0.5f);
    return timing;
}

inline OFDMFrameTiming wideOFDMFrameTiming(Modulation mod,
                                           CodeRate rate,
                                           int cw_count = v2::kDefaultFixedFrameCodewords,
                                           int data_lifting_z = 27) {
    return wideOFDMFrameTimingForCodewords(
        mod, rate, v2::sanitizeFixedFrameCodewords(cw_count), data_lifting_z);
}

// Conservative receiver-side extent after accepting the training position of
// an expected full OFDM anchor. In the longest wire shape that anchor belongs to
// the BURST_HEADER descriptor; the descriptor's remaining QPSK-R1/4 LTS+payload
// precedes a DATA burst that may consume the entire production airtime ceiling.
// A group-start anchor without a descriptor is shorter, so this remains safe.
inline uint32_t maxWideOFDMPhysicalTurnAfterAnchorTrainingMs() {
    const OFDMFrameTiming descriptor =
        wideOFDMFrameTiming(Modulation::QPSK, CodeRate::R1_4, 1, 27);
    return kMaxBurstAirtimeCeilingMs + descriptor.ack_ms;
}

inline uint32_t wideOFDMBurstAirtimeMs(Modulation mod,
                                       CodeRate rate,
                                       size_t frame_count,
                                       int cw_count = v2::kDefaultFixedFrameCodewords,
                                       uint32_t continuation_reanchor_ms = 0,
                                       int data_lifting_z = 27) {
    if (frame_count == 0) {
        return 0;
    }

    const OFDMFrameTiming timing =
        wideOFDMFrameTiming(mod, rate, cw_count, data_lifting_z);
    // Every nonempty physical OFDM turn has one full chirp+LTS anchor. In
    // particular, encodeBurstLight({one frame}) deliberately calls encodeFrame()
    // rather than encodeFrameLight(), so omitting this term for a singleton makes
    // its play-head and retry deadline 1.2 seconds early.
    uint64_t burst_ms = static_cast<uint64_t>(frame_count) * timing.data_ms +
                        kWideOFDMFullAnchorExtraMs;
    if (frame_count > 1) {
        // Continuations use light LTS-only preambles or an adaptive short reanchor.
        burst_ms += static_cast<uint64_t>(frame_count - 1) *
                    static_cast<uint64_t>(continuation_reanchor_ms);
    }
    return static_cast<uint32_t>(std::min<uint64_t>(burst_ms, 0xFFFFFFFFull));
}

// Derive the number of DATA frames that fit in one physical OFDM key-down.
// This is deliberately a pure geometry function: both the transmitter's burst
// builder and any rate selector comparing candidate rungs must use the same
// calculation.  In particular, an observed five-frame group says nothing about
// how many frames a different modulation/rate/CW candidate would fit.
inline size_t wideOFDMBurstFrameBudget(Modulation mod,
                                       CodeRate rate,
                                       int cw_count,
                                       size_t max_frames,
                                       uint32_t ceiling_ms,
                                       uint32_t continuation_reanchor_ms = 0,
                                       int data_lifting_z = 27,
                                       uint32_t group_start_extra_ms = 0) {
    if (max_frames <= 1) {
        return std::max<size_t>(1, max_frames);
    }
    size_t frames = 1;
    while (frames < max_frames) {
        const uint32_t airtime_ms = wideOFDMBurstAirtimeMs(
            mod, rate, frames + 1, cw_count,
            continuation_reanchor_ms, data_lifting_z);
        const uint64_t physical_signal_ms =
            static_cast<uint64_t>(airtime_ms) + group_start_extra_ms;
        if (physical_signal_ms > ceiling_ms) {
            break;
        }
        ++frames;
    }
    return frames;
}

inline uint32_t configuredBaseBurstAirtimeMs() {
    static const uint32_t value = [] {
        uint32_t configured = 8600;
        if (const char* env = std::getenv("ULTRA_MAX_BURST_AIRTIME_MS")) {
            const long parsed = std::strtol(env, nullptr, 10);
            if (parsed >= 5000 && parsed <= kMaxBurstAirtimeCeilingMs) {
                configured = static_cast<uint32_t>(parsed);
            }
        }
        return configured;
    }();
    return value;
}

// Production ceiling for a candidate rung after the clean-delivery evidence
// available at this decision.  Keeping this alongside the pure frame-budget
// function prevents the selector and sender from silently acquiring different
// definitions of a physical cycle.
inline uint32_t burstAirtimeCeilingMs(Modulation mod,
                                      CodeRate rate,
                                      int clean_group_streak) {
    constexpr uint32_t kEscalatedBurstAirtimeMs = 11500;
    static const bool kEscalationEnabled = [] {
        const char* e = std::getenv("ULTRA_BURST_ESCALATION");
        return !(e && e[0] == '0' && e[1] == '\0');
    }();
    const int non_dense_streak = [] {
        const char* e = std::getenv("ULTRA_BURST_ESC_STREAK");
        if (e == nullptr) return 0;
        const long parsed = std::strtol(e, nullptr, 10);
        return (parsed >= 2 && parsed <= 8) ? static_cast<int>(parsed) : 0;
    }();
    const bool dense_rung =
        coherentRungIndexFor(mod, rate) >= kRungIdxQam8R23;
    const bool streak_proven =
        dense_rung ? clean_group_streak >= 2
                   : (non_dense_streak > 0 && clean_group_streak >= non_dense_streak);
    const uint32_t base = configuredBaseBurstAirtimeMs();
    return (kEscalationEnabled && streak_proven)
               ? std::max(base, kEscalatedBurstAirtimeMs)
               : base;
}

inline uint32_t wideOFDMSackDelayMs(Modulation mod,
                                    CodeRate rate,
                                    size_t window_size,
                                    int cw_count = v2::kDefaultFixedFrameCodewords,
                                    uint32_t continuation_reanchor_ms = 0) {
    const uint32_t burst_ms = wideOFDMBurstAirtimeMs(
        mod, rate, std::max<size_t>(1, window_size), cw_count,
        continuation_reanchor_ms);
    return burst_ms + kCarrierSenseSackCoalesceMs;
}

inline uint32_t wideOFDMSackTailDelayMs() {
    return kCarrierSenseSackCoalesceMs;
}

inline uint32_t wideOFDMSlidingSackDelayMs(
        Modulation mod,
        CodeRate rate,
        int cw_count = v2::kDefaultFixedFrameCodewords) {
    const OFDMFrameTiming timing = wideOFDMFrameTiming(mod, rate, cw_count);
    // This timer is re-armed on every decoded DATA frame, so it is a quiet
    // interval after the observed burst tail, not a full-window hold. It covers
    // one selected-rate DATA frame cadence plus one selected-rate ACK/control
    // frame and the carrier-sense coalescing guard.
    const uint64_t quiet_interval_ms =
        static_cast<uint64_t>(timing.data_ms) +
        static_cast<uint64_t>(timing.ack_ms) +
        static_cast<uint64_t>(kCarrierSenseSackCoalesceMs);
    return static_cast<uint32_t>(std::min<uint64_t>(quiet_interval_ms, 0xFFFFFFFFull));
}

inline uint32_t coherenceTimeMsForDoppler(float doppler_hz) {
    if (!std::isfinite(doppler_hz) || doppler_hz <= 0.0f) {
        return UINT32_MAX;
    }
    // Clarke/Jakes 50%-correlation coherence-time approximation: Tc ~= 0.423/fD.
    const float tc_ms = (kClarkeCoherenceNumerator * 1000.0f) / doppler_hz;
    return static_cast<uint32_t>(std::max(1.0f, tc_ms) + 0.5f);
}

// ─── Retx trough-pacing deferral (docs/RETX_PACING_DESIGN_2026_07_03.md §1.2) ───
// f_D source for the deferral, in priority order: (1) the measured Doppler-coherence
// estimate when available (>0 — SECONDARY/approximate, fixed nominal cadence; acceptable
// because the deferral is order-of-magnitude machinery bounded by the clamps below, never
// a decode decision); (2) the ITU-R F.1487 design Doppler of the (coherence-adjusted)
// measured channel class — Good 0.1 Hz → Tc 4230 ms, Moderate 0.5 → 846 ms, Poor 1.0 →
// 423 ms. Never a tuned ms constant.
inline float retxTroughDopplerHz(float doppler_hz, float fading_index,
                                 float coherence_score, bool coherence_valid) {
    if (std::isfinite(doppler_hz) && doppler_hz > 0.0f) {
        return doppler_hz;
    }
    return designDopplerForFadingIndex(
        coherenceAdjustedFadingIndex(fading_index, coherence_score, coherence_valid));
}

// Absolute engineering clamp on any trough-pacing hold: no hold may exceed ~one burst-time
// (kMaxBurstAirtimeMs-scale) regardless of estimator garbage.
inline constexpr uint32_t kRetxTroughDeferAbsCapMs = 8000;

// After a ZERO-PROGRESS resend round (no TX base advance AND no new SACK bit — the whole
// key-down failed ⇒ trough-conditioned), defer the next resend round until the channel has
// decorrelated from the state that just killed it (Clarke/Jakes A(τ)=exp(−4π²σ²τ²) ⇒
// A(τ) ≤ 0.5 at τ ≥ Tc):
//
//   T_defer(n) = clamp( frac · Tc · 2^(n−1)  −  t_since_last_tx_end,   0,  T_cycle/2 )
//
// n = consecutive zero-progress rounds (≥1); Tc = coherenceTimeMsForDoppler(f_D);
// T_cycle = 1000/f_D ms (Clarke fade-cycle period). The elapsed-listening subtraction means
// the ~18 s RTO path adds ~nothing at Good (it already out-waited Tc); the ×2 escalation
// encodes "the era is longer than one Tc"; the T_cycle/2 cap is the trough-dwell bound
// (deferring past half a fade cycle overshoots the next crest — fast channels barely defer,
// which is right: their troughs pass on their own). Pure and unit-testable across the whole
// channel family. Partial-SACK rounds must NOT be fed here (§3: a partial round proves
// usable crests within the last burst-time — resend immediately, status quo).
inline uint32_t retxTroughDeferMs(float doppler_hz, float fading_index,
                                  float coherence_score, bool coherence_valid,
                                  int zero_rounds, uint32_t elapsed_since_tx_end_ms,
                                  float tc_frac = 1.0f) {
    const float fd = retxTroughDopplerHz(doppler_hz, fading_index,
                                         coherence_score, coherence_valid);
    if (!(fd > 0.0f)) {
        return 0;
    }
    const double frac = (std::isfinite(tc_frac) && tc_frac > 0.0f)
                            ? static_cast<double>(tc_frac)
                            : 1.0;
    const double tc_ms = static_cast<double>(coherenceTimeMsForDoppler(fd));
    const double half_cycle_ms = 500.0 / static_cast<double>(fd);  // (1000/f_D)/2
    const int n = std::max(1, zero_rounds);
    const double escalation = static_cast<double>(1u << std::min(n - 1, 16));
    double defer_ms = frac * tc_ms * escalation
                      - static_cast<double>(elapsed_since_tx_end_ms);
    defer_ms = std::min(defer_ms, half_cycle_ms);
    defer_ms = std::min(defer_ms, static_cast<double>(kRetxTroughDeferAbsCapMs));
    if (!(defer_ms > 0.0)) {
        return 0;
    }
    return static_cast<uint32_t>(defer_ms + 0.5);
}

// Recommend fixed-frame CW count for a given OFDM data rate + waveform.
// Inputs are deterministic and shared by both peers (rate is negotiated;
// waveform is negotiated too) so both peers compute the same CW count
// without risk of one reading a different SNR/fading and picking a
// different CW geometry.
//
// Wide OFDM (OFDM_CHIRP):
//   R1/2, R2/3, R3/4 → 8 (hardware A/B Mac↔Pi5 5KB DQPSK R1/2 SNR=15
//   good fading: CW=4 → 1077 bps 2 retx; CW=8 → 1615 bps 0 retx, +50%.
//   D8PSK R3/4 SNR=27 AWGN ceiling: CW=8 → 3127 bps.)
//   R1/4 stays at default 4 (low-SNR robustness, no measured win wider).
//
// Narrow OFDM (OFDM_NARROW): always 4. Narrow R1/2 frames are already
// ~6 s at CW=8 with the 21-carrier geometry; window=3 burst would be
// ~18 s — longer than typical narrow good-fading coherence (~10 s).
// 3-seed sim A/B at SNR=8 good fading R1/2 (2 KB): CW=8 1/3 FAIL with
// 240 s timeout, 2/3 PASS at 124-172 bps; CW=4 baseline 3/3 PASS at
// 116-149 bps. The bigger frame becomes a single fade event's victim.
inline int recommendCWCount(CodeRate rate, WaveformMode waveform) {
    if (waveform == WaveformMode::OFDM_NARROW) {
        return v2::kDefaultFixedFrameCodewords;  // 4 — fade-coherence cap
    }
    if (const auto* descriptor = ofdmCodeRateDescriptor(rate)) {
        return descriptor->wide_cw_count;
    }
    return v2::kDefaultFixedFrameCodewords;
}

// Modulation-aware data-frame CW policy for waveforms whose fade exposure is
// dominated by frame duration. OFDM keeps the fixed-frame policy above.
// Robust-Low MC-DPSK DBPSK uses variable LDPC frames; with R1/4, 3 CW carries
// a 37-byte ARQ payload, i.e. 32 file bytes after FILE_DATA overhead. 1 CW
// cannot carry file data and 2 CW is too slow to be operationally useful.
inline int recommendCWCount(Modulation mod, CodeRate rate, WaveformMode waveform) {
    if (waveform == WaveformMode::MC_DPSK && mod == Modulation::DBPSK) {
        (void)rate;
        return 3;
    }
    // cw16 for 16QAM (ULTRA_QAM16_CW16, default OFF; 2026-07-05 — fable 09 §5.5):
    // a 16QAM cw8 frame is only ~672 ms, so the dense rung pays the LTS/header
    // overhead TWICE as often per bit as QPSK cw8 (~1272 ms). Baseline 16 restores
    // the proven cw8-QPSK frame airtime at 16QAM (same coherence exposure, half
    // the per-bit overhead; rung ceiling 3.3k -> ~3.9k). BASELINE only:
    // recommendCWCountForChannel's coherence walk still shrinks the frame whenever
    // its airtime would exceed the measured coherence time — the selection stays
    // channel-adaptive by construction (adaptivity rule).
    if (waveform == WaveformMode::OFDM_CHIRP && mod == Modulation::QAM16) {
        static const bool kQam16Cw16 = [] {
            const char* e = std::getenv("ULTRA_QAM16_CW16");
            return !(e && e[0] == '0');  // DEFAULT-ON 2026-07-05
        }();
        if (kQam16Cw16) return 16;
    }
    // 8PSK revival (2026-07-05): cw12 normalizes coherent 8PSK (3 bits/sym) frames
    // to the same ~1272 ms airtime as QPSK cw8 / 16QAM cw16 — the per-modulation
    // frame-duration rule (12 × 648 coded bits / ~153 bits/sym ≈ 51 symbols).
    // Gated on the ladder knob (one knob for the whole revival); the coherence
    // walk in recommendCWCountForChannel still shrinks it past the measured Tc.
    if (waveform == WaveformMode::OFDM_CHIRP && mod == Modulation::QAM8 &&
        psk8LadderEnabled()) {
        return 12;
    }
    return recommendCWCount(rate, waveform);
}

// Counterfactual CW geometry for a receiver-issued rate command.  The tone-ACK
// command carries only the target ladder rung; it does not carry the receiver's
// local fading estimate or a requested CW count.  The sender chooses its actual
// channel-refined CW and announces that value in the following burst descriptor.
// Until that descriptor arrives, the only peer-independent geometry the receiver
// can price is the deterministic baseline (or the explicit operator override).
// Feeding receiver-local fading into this prediction can invent a wire shape the
// sender never uses (fixed04: predicted CW=5/N=9 while the Pi sent CW=8/N=5).
inline int receiverRateCommandCandidateCWCount(Modulation mod,
                                               CodeRate rate,
                                               WaveformMode waveform,
                                               int forced_cw_count = 0) {
    return forced_cw_count != 0
        ? v2::sanitizeFixedFrameCodewords(forced_cw_count)
        : recommendCWCount(mod, rate, waveform);
}

// Coherence-only CW refinement.  This is the shareable form for code paths
// (notably the outcome-fitted latent rate selector) that intentionally consume
// no SNR estimate.  The channel-facing wrapper below retains its near-AWGN
// bypass, then delegates here.
inline int recommendCWCountForFading(Modulation mod,
                                     CodeRate rate,
                                     WaveformMode waveform,
                                     float fading_index,
                                     float doppler_hz = -1.0f) {
    const int baseline = recommendCWCount(mod, rate, waveform);
    if (waveform != WaveformMode::OFDM_CHIRP ||
        !ofdm_link_adaptation::isCoherentModulation(mod)) {
        return baseline;
    }

    const auto* descriptor = ofdmCodeRateDescriptor(rate);
    const auto* coherence_floor = ofdmCodeRateDescriptor(CodeRate::R1_2);
    if (descriptor == nullptr || coherence_floor == nullptr ||
        descriptor->code_rate < coherence_floor->code_rate) {
        return baseline;
    }

    // Derive the design Doppler from the measured channel class (fading_index)
    // unless a caller overrides it explicitly (>0). Good→0.1 Hz keeps cw=8;
    // Moderate→0.5 Hz / Poor→1.0 Hz keep the protective shorter-frame cap.
    const float effective_doppler =
        (doppler_hz > 0.0f) ? doppler_hz : designDopplerForFadingIndex(fading_index);
    const uint32_t coherence_ms = coherenceTimeMsForDoppler(effective_doppler);
    int selected = v2::kDefaultFixedFrameCodewords;
    for (int candidate = baseline; candidate >= v2::kDefaultFixedFrameCodewords; --candidate) {
        const auto timing = wideOFDMFrameTiming(mod, rate, candidate);
        if (timing.data_ms <= coherence_ms) {
            selected = candidate;
            break;
        }
    }
    return v2::sanitizeFixedFrameCodewords(selected);
}

inline int recommendCWCountForChannel(Modulation mod,
                                      CodeRate rate,
                                      WaveformMode waveform,
                                      float fading_index,
                                      float snr_db,
                                      float doppler_hz = -1.0f) {
    if (isNearAwgnOFDM(fading_index, snr_db)) {
        return recommendCWCount(mod, rate, waveform);
    }
    return recommendCWCountForFading(
        mod, rate, waveform, fading_index, doppler_hz);
}

inline uint32_t computeWideOFDMAckTimeoutMs(Modulation mod,
                                            CodeRate rate,
                                            size_t window_size,
                                            uint32_t sack_delay_ms,
                                            int ack_repeat_count,
                                            int cw_count = v2::kDefaultFixedFrameCodewords,
                                            uint32_t continuation_reanchor_ms = 0) {
    const int sanitized_cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    const OFDMFrameTiming timing = wideOFDMFrameTiming(mod, rate, sanitized_cw_count);

    const uint32_t ack_copies = static_cast<uint32_t>(std::clamp(ack_repeat_count, 1, 3));
    const size_t window_frames = std::max<size_t>(1, window_size);
    const uint32_t tx_burst_ms = wideOFDMBurstAirtimeMs(
        mod, rate, window_frames, sanitized_cw_count,
        continuation_reanchor_ms);
    const uint32_t physical_sack_hold_ms = std::max<uint32_t>(
        sack_delay_ms,
        wideOFDMSackDelayMs(mod, rate, window_frames, sanitized_cw_count,
                            continuation_reanchor_ms));
    const uint32_t ack_path_ms = ack_copies * timing.ack_ms + physical_sack_hold_ms;

    constexpr uint32_t audio_chain_rtt_margin_ms = 700;
    const uint32_t decode_jitter_margin_ms = std::max<uint32_t>(700, timing.data_ms / 2)
                                             + audio_chain_rtt_margin_ms;

    // tx_burst_ms spans the actual multi-frame OFDM burst, including the full
    // chirp anchor on the first frame. The SACK path must also include that
    // receiver holdoff because a half-duplex peer cannot ACK until the burst
    // has physically cleared.
    const uint32_t timeout_ms = tx_burst_ms + ack_path_ms + decode_jitter_margin_ms;

    return std::max<uint32_t>(timeout_ms, kWideOFDMAckTimeoutFloorMs);
}

// Burst-transport (unified SR-ARQ) ACK timeout: the deadline the sender waits for the prompt
// tone-burst group-ACK before resending the WHOLE burst. A half-duplex peer cannot ACK until it
// has (1) physically received the multi-frame burst (real-time airtime), (2) held off for its
// deliberate SACK-coalesce delay, (3) decoded, then (4) keyed up to return the 1-CW tone-burst.
// EVERY term is mod/rate/cw/z-derived, so the deadline is correct for the whole modulation family
// by construction (QPSK..16QAM, R1/4..R3/4, any cw/frame-count, z=27/81).
//
// 2026-06-19 FIX: added physical_sack_hold_ms. The sibling computeWideOFDMAckTimeoutMs() budgets the
// receiver's SACK holdoff (line ~697) but THIS burst path did not — only a fixed 1500 ms round-trip
// slack, which is SMALLER than the receiver's mod/rate-scaled SACK-coalesce delay (~2000 ms+). So for
// a full cw8 burst the deadline fired ~1-2 s before the receiver's intentionally-delayed SACK could
// arrive, and the sender resent the entire group even though most frames had already decoded and the
// ACK was in flight (measured live: IONOS MPG@20, group seq63-67 — 4/5 frames OK at RX, SACK sent,
// yet a 5-frame timeout-resend cost ~8.8 s, the single largest retx in the run). Budgeting the same
// hold the receiver applies (wideOFDMSackDelayMs / setSackDelay) makes sender and receiver agree
// across every mod/rate. Conservative by design: an over-long deadline only delays a genuinely-lost
// ACK's resend slightly; an under-long one triggers spurious whole-burst resends (far worse).
// Finish the unified DATA-round timeout from exact physical geometry. This is shared
// by the homogeneous fixed-frame policy below and Connection's heterogeneous/variable-
// CW wire inspection, so both retain the same receiver, decode, ACK and turnaround
// margins. remaining_data_ms is the data airtime after the first frame; the receiver's
// group-timeout envelope is half that remainder plus 3 seconds. Multi-frame encoder
// reliability mode can add a second full group-start anchor beyond the descriptor's
// anchor, supplied as reliability_extra_anchor_ms. A singleton already uses encodeFrame()
// and has no second group-start preamble, so its reserve is zero.
inline uint32_t unifiedBurstAckTimeoutFromPhysicalGeometryMs(
        uint32_t burst_ms,
        uint32_t remaining_data_ms,
        uint32_t max_data_frame_ms,
        Modulation control_mod,
        uint32_t reliability_extra_anchor_ms,
        uint32_t reanchor_ms = 0) {
    const OFDMFrameTiming control_timing =
        wideOFDMFrameTiming(control_mod, CodeRate::R1_4);
    const uint32_t rx_response_ms = remaining_data_ms / 2u + 3000u;
    const uint32_t decode_margin_ms =
        std::max<uint32_t>(700, max_data_frame_ms / 2u) + 700u;
    const uint32_t ack_return_ms = control_timing.ack_ms + reanchor_ms;
    constexpr uint32_t kRoundTripSlackMs = 1500;
    const uint64_t total = static_cast<uint64_t>(burst_ms) + rx_response_ms +
                           decode_margin_ms + ack_return_ms +
                           kRoundTripSlackMs + reliability_extra_anchor_ms;
    return static_cast<uint32_t>(std::min<uint64_t>(total, 0xFFFFFFFFull));
}

inline uint32_t unifiedBurstAckTimeoutMs(Modulation data_mod,
                                         CodeRate data_rate,
                                         int cw_count,
                                         size_t burst_frames,
                                         int data_lifting_z,
                                         Modulation control_mod,
                                         uint32_t configured_sack_delay_ms,
                                         uint32_t reanchor_ms = 0) {
    const int sanitized_cw = v2::sanitizeFixedFrameCodewords(cw_count);
    const size_t frames = std::max<size_t>(1, burst_frames);
    const uint64_t frame_samples = wideOFDMWireFrameSamplesForCodewords(
        data_mod, data_rate, sanitized_cw, data_lifting_z);
    // (1) exact default StreamingEncoder output plus ModemEngine's configured
    // lead/tail samples. This includes the descriptor control frame for N>1.
    const uint64_t waveform_samples = wideOFDMWireBurstSamples(
        data_mod, data_rate, frames, sanitized_cw, data_lifting_z,
        /*force_full_group_start=*/false, control_mod, reanchor_ms);
    const uint32_t burst_ms =
        postProcessedTxDurationFromSamplesMs(waveform_samples);
    // (2) receiver response envelope — RE-DERIVED 2026-07-02 (closes
    // BUG-ACK-TIMEOUT-DOUBLECOUNT). Rig calibration (124 groups across 4 MPG@20
    // transfers) measured the CLEAN-path group-end->SACK hold at 0-1 ms — the
    // old max(configured window-hold, burstAirtime+30) term modeled a hold the
    // receiver never applies on the burst path (it double-counted the burst
    // airtime, ~+8-12 s of deadline). The REAL worst-case delayed response is
    // the receiver's group-timeout fast-NACK, whose airtime-derived budget
    // (accumulateBurstFrames: remaining x1.5 + 3000 ms from first-frame decode)
    // lands at most 0.5 x remaining-airtime + 3000 ms after burst end. Using
    // the SAME formula family keeps sender deadline and receiver timer coherent
    // by construction across every mod/rate/cw/z. configured_sack_delay_ms is
    // deliberately NOT consumed here anymore (it carries the sender-side
    // window-hold arming, which the burst-path receiver does not apply); the
    // parameter stays for call-site stability.
    (void)configured_sack_delay_ms;
    const uint64_t remaining_data_samples =
        static_cast<uint64_t>(frames - 1) * frame_samples;
    return unifiedBurstAckTimeoutFromPhysicalGeometryMs(
        burst_ms,
        sampleDurationCeilMs(remaining_data_samples),
        sampleDurationCeilMs(frame_samples), control_mod,
        frames > 1 ? kWideOFDMFullAnchorExtraMs : 0u,
        reanchor_ms);
}

inline uint32_t bitsPerMCDPSKCarrier(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK: return 1;
        case Modulation::D8PSK: return 3;
        case Modulation::DQPSK:
        default: return 2;
    }
}

inline uint32_t mcDpskSymbolsToMs(uint32_t symbols, int samples_per_symbol) {
    const int sps = std::clamp(samples_per_symbol, 1, 8192);
    return static_cast<uint32_t>(
        (static_cast<uint64_t>(symbols) * static_cast<uint64_t>(sps) * 1000ULL +
         kOFDMSampleRate / 2) / kOFDMSampleRate);
}

inline MCDPSKFrameTiming mcDpskFrameTiming(Modulation mod,
                                           int num_carriers,
                                           int samples_per_symbol,
                                           int data_cw_count = v2::kDefaultFixedFrameCodewords) {
    const int carriers = std::clamp(num_carriers, 1, 64);
    const int sps = std::clamp(samples_per_symbol, 1, 8192);
    const int cw_count = v2::sanitizeFixedFrameCodewords(data_cw_count);
    const uint32_t bits_per_symbol = static_cast<uint32_t>(carriers) * bitsPerMCDPSKCarrier(mod);
    const uint32_t data_symbols_per_cw =
        (kLDPCBitsPerCodeword + bits_per_symbol - 1) / bits_per_symbol;

    constexpr uint32_t kMCDPSKTrainingSymbols = 8;
    constexpr uint32_t kMCDPSKReferenceSymbols = 1;
    const uint32_t overhead_symbols = kMCDPSKTrainingSymbols + kMCDPSKReferenceSymbols;

    MCDPSKFrameTiming timing;
    timing.overhead_symbols = overhead_symbols;
    timing.data_only_symbols = static_cast<uint32_t>(cw_count) * data_symbols_per_cw;
    timing.data_symbols = overhead_symbols + timing.data_only_symbols;
    timing.ack_symbols = overhead_symbols + data_symbols_per_cw;
    timing.overhead_ms = mcDpskSymbolsToMs(timing.overhead_symbols, sps);
    timing.data_only_ms = mcDpskSymbolsToMs(timing.data_only_symbols, sps);
    timing.data_ms = mcDpskSymbolsToMs(timing.data_symbols, sps);
    timing.ack_ms = mcDpskSymbolsToMs(timing.ack_symbols, sps);
    return timing;
}

inline uint32_t mcDpskBurstAirtimeMs(const MCDPSKFrameTiming& timing,
                                     size_t window_size) {
    if (window_size == 0) return 0;
    const uint64_t burst_ms =
        static_cast<uint64_t>(kMCDPSKDualChirpPreambleMs) +
        static_cast<uint64_t>(timing.overhead_ms) +
        static_cast<uint64_t>(window_size) * timing.data_only_ms;
    return static_cast<uint32_t>(
        std::min<uint64_t>(burst_ms, static_cast<uint64_t>(UINT32_MAX)));
}

inline size_t mcDpskWindowSizeForTiming(const MCDPSKFrameTiming& timing) {
    if (timing.data_ms == 0 || timing.data_only_ms == 0) return 1;

    // #71: the window must be sized so the FULL half-duplex ACK round-trip fits under the
    // ACK RTO with margin — TX burst + the receiver's SERIAL decode of that burst (task
    // #56, and heavier/more variable for DQPSK's 2-bit soft demod) + SACK hold + ACK
    // airtime. The old sizing targeted a 19 s TX burst ONLY and was blind to the
    // decode+ACK half, so DQPSK's shorter frames let the window grow to 5; that burst's
    // round-trip intermittently exceeded the RTO -> the sender blind-resends the whole
    // window -> spiral. Rig-measured @ MPG@9 (paired, live fades): DQPSK window=5 delivered
    // 1/3 (2/3 spiraled, all cause=timeout, 0 cw_fail — decode was clean, the ACK just
    // didn't get back in time); DQPSK window=3 delivered 3/3 CRC-clean at ~2x DBPSK
    // goodput; DBPSK window=3 delivered 3/3. DBPSK was never affected — its longer frames
    // already cap it below 5 (test_connection_policy + the rig log confirm ROBUST_MID/sps=1024
    // -> 3, ROBUST_LOW/sps=2048 -> 1). So cap at the round-trip-safe, rig-validated 3.
    constexpr uint32_t kTargetContinuousBurstMs = 19000;
    constexpr size_t kMaxRoundTripSafeMCDPSKWindow = 3;
    size_t selected = 1;
    for (size_t candidate = 2; candidate <= kMaxRoundTripSafeMCDPSKWindow; ++candidate) {
        if (mcDpskBurstAirtimeMs(timing, candidate) > kTargetContinuousBurstMs) {
            break;
        }
        selected = candidate;
    }
    // Diagnostic override (ULTRA_MCDPSK_WINDOW_CAP=N): further cap the window for A/B
    // round-trip measurement per rung; unset = no-op.
    if (const char* cap = std::getenv("ULTRA_MCDPSK_WINDOW_CAP")) {
        const long v = std::strtol(cap, nullptr, 10);
        if (v >= 1) selected = std::min<size_t>(selected, static_cast<size_t>(v));
    }
    return selected;
}

inline size_t mcDpskWindowSizeForTiming(uint32_t data_frame_ms) {
    if (data_frame_ms == 0) return 1;

    // Round-trip-safe cap of 3 (see the MCDPSKFrameTiming overload above for the #71
    // rig-validated rationale). This coarse overload is not on the production window path
    // (connection.cpp uses the timing overload); kept consistent to avoid a stale bound.
    constexpr uint32_t kTargetBurstMs = 19000;
    constexpr size_t kMaxRoundTripSafeMCDPSKWindow = 3;
    const size_t by_burst = std::max<size_t>(1, kTargetBurstMs / data_frame_ms);
    return std::clamp<size_t>(by_burst, 1, kMaxRoundTripSafeMCDPSKWindow);
}

// Sender ACK RTO for an MC-DPSK selective-repeat window burst. It MUST exceed the real
// half-duplex ACK round-trip, else the sender blind-resends the whole window before the
// legitimate ACK lands -> doubled airtime -> the FINAL file chunk is never reached in a
// bounded session -> the transfer never finalizes (BUG-MCDPSK-FILE-COMPLETION), and on a
// hole the resend collides with the receiver's SACK (BUG-MCDPSK-ACK-COLLISION).
//
// The physical round-trip on MC-DPSK's long (~5.4 s) frames has FOUR terms the old formula
// under-budgeted (it summed only tx_burst + ack + a flat 12 s, and was passed the 30 ms
// carrier-sense coalesce instead of the real receiver hold):
//   (1) tx_burst  = window * data_ms         sender transmits the whole window
//   (2) rx_decode ~= window * data_ms         receiver SERIALLY decodes it before it can
//                                             build the SACK (streaming decoder / RXQ
//                                             backlog, task #56 — each frame ~= one data_ms;
//                                             this is the dominant term the old flat 12 s
//                                             margin missed, measured ~16 s on the rig)
//   (3) receiver_sack_hold_ms                 the tone-burst partial-SACK coalesce hold the
//                                             receiver actually applies (must be the SAME
//                                             value passed to setToneBurstPartialSackDelayMs)
//   (4) ack_copies * ack_ms                   ACK airtime (x repeat copies)
// plus a small T/R turnaround margin. Measured rig RTT (DBPSK R1/4, w=3) ~= 37.9 s; this
// budgets ~43.5 s. The lower clamp is lifted to the physical RTT so it can never truncate
// the round-trip and re-introduce the self-collision (mirrors the narrow-OFDM physical
// floor). See docs/CHANGELOG.md.
inline uint32_t computeMCDPSKAckTimeoutMs(const MCDPSKFrameTiming& timing,
                                          size_t window_size,
                                          uint32_t receiver_sack_hold_ms,
                                          int ack_repeat_count) {
    const uint32_t ack_copies = static_cast<uint32_t>(std::clamp(ack_repeat_count, 1, 3));
    const uint32_t tx_burst_ms  = static_cast<uint32_t>(window_size) * timing.data_ms;
    const uint32_t rx_decode_ms = static_cast<uint32_t>(window_size) * timing.data_ms;
    const uint32_t ack_path_ms  = ack_copies * timing.ack_ms + receiver_sack_hold_ms;
    constexpr uint32_t kTurnaroundMarginMs = 3000;

    const uint32_t physical_rtt_ms = tx_burst_ms + rx_decode_ms + ack_path_ms;
    const uint32_t timeout_ms = physical_rtt_ms + kTurnaroundMarginMs;
    // Never clamp below the physical RTT (else the sender self-collides with blind resends).
    return std::clamp(timeout_ms,
                      std::max<uint32_t>(18000u, physical_rtt_ms),
                      std::max<uint32_t>(72000u, physical_rtt_ms));
}

inline OFDMFrameTiming narrowOFDMFrameTiming(Modulation mod,
                                             int cw_count = v2::kDefaultFixedFrameCodewords) {
    cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    constexpr float symbol_ms =
        (1000.0f * static_cast<float>(kNarrowOFDMSymbolSamples)) /
        static_cast<float>(kOFDMSampleRate);

    const uint32_t bits_per_symbol = bitsPerOFDMSymbol(
        kNarrowOFDMCarriers, true, static_cast<int>(kNarrowOFDMPilotSpacing), mod);

    const uint32_t data_cw_symbols =
        (static_cast<uint32_t>(cw_count) * kLDPCBitsPerCodeword + bits_per_symbol - 1) / bits_per_symbol;
    const uint32_t ack_cw_symbols =
        (kLDPCBitsPerCodeword + bits_per_symbol - 1) / bits_per_symbol;

    OFDMFrameTiming timing;
    timing.data_symbols = 2 + data_cw_symbols;
    timing.ack_symbols = 2 + ack_cw_symbols;
    timing.data_ms = static_cast<uint32_t>(timing.data_symbols * symbol_ms + 0.5f);
    timing.ack_ms = static_cast<uint32_t>(timing.ack_symbols * symbol_ms + 0.5f);
    return timing;
}

// Receiver tone-burst partial-SACK coalesce hold, applied UNCONDITIONALLY for every mode that
// returns a tone-burst ack (selective_repeat_arq.cpp:622, kToneBurstPartialSackDelayMs). The narrow
// peer withholds its SACK this long before keying up, so the sender's deadline must budget it — the
// same omission the wide-OFDM burst path had (IONOS MPG E5).
inline constexpr uint32_t kToneBurstReceiverSackHoldMs = 1500;

// An anchored no-group timeout proves that the normal descriptor/group callback
// was lost, not that zero payload frames decoded. Classic fallback DATA is exact
// positive delivery evidence; without descriptor geometry its k/M is unknown, so
// withhold the selector observation while still allowing a cumulative async SACK.
inline constexpr bool shouldGradeAnchoredBackstopAsCrater(bool payload_seen) {
    return !payload_seen;
}

inline uint32_t computeNarrowOFDMAckTimeoutMs(
        Modulation mod,
        int cw_count = v2::kDefaultFixedFrameCodewords,
        size_t window_size = 1,
        uint32_t configured_sack_delay_ms = kToneBurstReceiverSackHoldMs) {
    const OFDMFrameTiming timing = narrowOFDMFrameTiming(mod, cw_count);
    // For window>1 selective-repeat, we hold the ARQ window full during
    // the burst, so the ACK timeout has to cover the full TX burst plus
    // ACK turnaround plus a generous decode margin. Without this scale
    // the timer fires while later frames are still on the wire.
    const uint32_t tx_burst_ms = timing.data_ms *
                                 static_cast<uint32_t>(std::max<size_t>(1, window_size));
    // 2026-06-19: budget the receiver's tone-burst SACK-coalesce hold (the term the formula
    // OMITTED — narrow QPSK/8PSK/QAM16 at window=3 went 0.3-1.5 s short of the 1.5 s hold, the
    // same premature-resend class as wide-OFDM E5). Match the larger of the configured delay and
    // the physical tone-burst hold so sender and receiver agree across every narrow mod/rate.
    const uint32_t physical_sack_hold_ms =
        std::max<uint32_t>(configured_sack_delay_ms, kToneBurstReceiverSackHoldMs);
    const uint32_t timeout_ms = tx_burst_ms + 2 * timing.ack_ms + 120 +
                                std::max<uint32_t>(700, timing.data_ms / 2) +
                                physical_sack_hold_ms;
    // The cap only bounds the NON-physical extras (decode-jitter slop). It must NEVER fall below
    // the physical minimum the peer needs to ACK — burst airtime + ack turnaround + receiver SACK
    // hold — else the clamp re-introduces the premature-timeout bug for large narrow frames.
    const uint32_t physical_floor_ms =
        tx_burst_ms + 2 * timing.ack_ms + physical_sack_hold_ms;
    const uint32_t upper = std::max<uint32_t>(
        physical_floor_ms,
        14000u + physical_sack_hold_ms +
            8000u * static_cast<uint32_t>(std::max<size_t>(1, window_size) - 1));
    return std::clamp(timeout_ms, 4500u, upper);
}

inline WaveformMode selectNegotiatedMode(uint8_t local_caps,
                                         uint8_t remote_caps,
                                         WaveformMode remote_pref,
                                         WaveformMode narrowband_override,
                                         WaveformMode local_pref,
                                         float snr_db,
                                         float fading_index) {
    const uint8_t common = local_caps & remote_caps;
    if (common == 0) {
        // No overlap in advertised capabilities: fall back to MC-DPSK, the
        // universal robust floor every station implements.
        return WaveformMode::MC_DPSK;
    }

    if (remote_pref != WaveformMode::AUTO &&
        (common & modeToCapabilityBit(remote_pref))) {
        return remote_pref;
    }

    if (narrowband_override != WaveformMode::AUTO &&
        (common & modeToCapabilityBit(narrowband_override))) {
        return narrowband_override;
    }

    if (local_pref != WaveformMode::AUTO &&
        (common & modeToCapabilityBit(local_pref))) {
        return local_pref;
    }

    const auto rec = recommendWaveformAndRate(snr_db, fading_index);
    if (common & modeToCapabilityBit(rec.waveform)) {
        return rec.waveform;
    }

    if (common & ModeCapabilities::OFDM_CHIRP) return WaveformMode::OFDM_CHIRP;
    if (common & ModeCapabilities::OFDM_NARROW) return WaveformMode::OFDM_NARROW;
    if (common & ModeCapabilities::MC_DPSK) return WaveformMode::MC_DPSK;

    // Last resort: MC-DPSK, the universal robust floor.
    return WaveformMode::MC_DPSK;
}

}  // namespace connection_policy
}  // namespace protocol
}  // namespace ultra
