// waveform_selection.hpp - Waveform and rate selection algorithm
//
// Centralized algorithm for selecting waveform mode and code rate
// based on SNR and fading index. Used by both protocol negotiation
// and ModemEngine.
//
// Based on testing with CFO=20Hz across AWGN/good/moderate channels (2026-01-29)

#pragma once

#include "protocol/frame_v2.hpp"  // WaveformMode
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"        // CodeRate

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include "ultra/fast_profile.hpp"

namespace ultra {
namespace protocol {

// OFDM "is the wideband waveform viable?" ENTRY floors (in-band SNR, dB), keyed by
// channel class. SINGLE source of truth: both recommendWaveformAndRate() below
// (fading-index-keyed) and connection_policy::selectLadderRung() (enum-keyed)
// reference these, so the OFDM-vs-MC-DPSK entry decision cannot drift between the
// two call sites. Classification thresholds: <0.15 AWGN, <0.65 Good, <1.10 Moderate.
inline constexpr float kOFDMEntryFloorAwgnDb = 8.0f;    // 2026-06-02: lowered 10->8 (R1/4 clean @ AWGN 8, 0% dmg; floor likely lower)
inline constexpr float kOFDMEntryFloorGoodDb = 8.0f;    // 2026-07-07: lowered 10->8 — measured floor sweep (measure_ack_fer data4_full, 2 seeds x 150):
                                                        // QPSK R1/4 Good@8 FER 17%/17% (ARQ-viable, ~8x MC-DPSK's delivered rate in this band);
                                                        // Good@6 29/35% = marginal, stays out. SAFE NOW vs the 2026-06 cliff: entry lands on the
                                                        // R1/4 floor rung and the PREDICTIVE CLIMB promotes only on measured per-carrier snapshots
                                                        // (no blind R1/2 pick off a stale connect snapshot). NOTE: standalone CONNECT at Good<=12
                                                        // still gated on the handshake floor (#70 ULTRA_ROBUST_IDLE_PING) — this band primarily
                                                        // rescues connected sessions that fade down. QAM8/8PSK R1/4 measured DOMINATED by QPSK
                                                        // R1/2 (equal FER, -33% capacity) — do not add low-rate dense rungs.
inline constexpr float kOFDMEntryFloorModerateDb = 14.0f;
// The AWGN OFDM entry floor consulted by the two entry decisions (recommendWaveformAndRate,
// selectLadderRung). Below the ladder's lowest anchor selectCoherentOFDM already returns QPSK
// R1/4, so lowering it only moves the OFDM-vs-MC-DPSK boundary.
//
// MEASURED FLOOR (2026-11-09): QPSK R1/4 on AWGN reaches 10 % FER at 4.0 dB (measure_ack_fer
// --geometry production, 7.3 Hz, 2026-11-08 sweep); the floor adds 2 dB for the connect
// estimate AND for the control plane, which is weaker than the data rung at the edge. GUI gate,
// 4 KB: OFDM entered at AWGN@6/7 passes 10/10 at 510-1170 bps (plain stack and ULTRA_FAST),
// against MC-DPSK's 45-90 bps there (8 dB floor: AWGN@7 90 bps, AWGN@6 too slow to finish 4 KB in
// 480 s). At AWGN@5 the data passes (4/5) but one ULTRA_FAST run lost every ACK to the async-ACK
// hold (BUG-ASYNC-ACK-ALWAYS-DROPPED) and a true 4 dB entry stalled — hence 6, not 5. Rig-dependent
// (a real radio's distortion moves the OFDM waterfall), so it is the ULTRA_FAST default, not the
// plain default. ULTRA_OFDM_FLOOR_AWGN_DB overrides. Read once.
inline constexpr float kOFDMEntryFloorAwgnMeasuredDb = 6.0f;
inline float ofdmEntryFloorAwgnDb() {
    static const float v = [] {
        if (const char* e = std::getenv("ULTRA_OFDM_FLOOR_AWGN_DB")) {
            const float f = std::strtof(e, nullptr);
            if (f >= 0.0f && f <= 20.0f) return f;
        }
        return fastProfileEnabled() ? kOFDMEntryFloorAwgnMeasuredDb : kOFDMEntryFloorAwgnDb;
    }();
    return v;
}
// Poor HF (fading >= 1.10: fast Doppler / heavy multipath) routes to MC-DPSK, NEVER
// OFDM (thread A, 2026-05-31). Coherent OFDM phase tracking breaks on fast fading and
// the OFDM band is now coherent-only; the decision doc + the long-standing "Poor HF:
// OFDM fails — use MC-DPSK" limitation both say Poor is MC-DPSK's. An unreachable floor
// expresses "no OFDM at Poor" in the single source both selection paths consult, so the
// retired Poor-OFDM corner can't reappear. (Was 18.0f.)
inline constexpr float kOFDMEntryFloorPoorDb = 1.0e9f;

// Fading-class boundaries (combined freq_cv + temporal_cv). Poor (>= kFadingModerateMax)
// routes to MC-DPSK upstream (kOFDMEntryFloorPoorDb), so the coherent OFDM ladder below
// only ever classifies AWGN / GOOD / MODERATE.
//
// Cluster centers, ground-truth-labeled by the OTASim ITU channel model in use
// (app.cpp fading calibration + docs/CONNECT_ENTRY_CALIBRATION_2026_07_03.md, the
// 48-entry MPG@20 dial-Good ledger): ITU Good clusters ~0.62, ITU Moderate ~0.90.
inline constexpr float kFadingCenterGood = 0.62f;
inline constexpr float kFadingCenterModerate = 0.90f;

inline constexpr float kFadingAwgnMax = 0.15f;
// Good<->Moderate boundary = the maximum-likelihood midpoint of the Good and Moderate
// cluster centers (equal-variance Gaussian => a reading joins the NEARER center) —
// DERIVED from the calibration, not hand-tuned. The old 0.65 sat essentially ON the
// Good center (0.62), giving a true-Good channel ~zero margin: its upper-tail readings
// (measured to 0.74, sigma 0.129) tipped into Moderate -> the ladder's sparse Moderate
// column -> QPSK R1/4 (BUG-MPG20-OVER-DEMOTE-R14: on a dial-20 Good channel, 18.8% of
// Good entries false-Moderate, each a ~100-200 s climb-back; the per-group RX-authority
// verdict pins R1/4 the whole transfer, connection_policy.hpp:708). The misclassification
// cost is ASYMMETRIC — false-Moderate = minutes of low-rung crawl, false-Good = one ~16 s
// group before the crater-demote — which justifies the boundary sitting AT LEAST at the
// ML midpoint. 0.76 clears the ledger's Good max (0.74) while keeping real Moderate
// (center 0.90) correctly classified. (isFading()/isHighThroughputOFDM keep their own
// 0.65 "significant-fading present"/conservative-window gate — a different question than
// the class label; not the rate-selection boundary this fixes.)
inline constexpr float kFadingGoodMax = (kFadingCenterGood + kFadingCenterModerate) * 0.5f;  // 0.76
inline constexpr float kFadingModerateMax = 1.10f;

enum class FadingClass { AWGN = 0, GOOD = 1, MODERATE = 2 };
inline FadingClass classifyFading(float fading_index) {
    if (fading_index < kFadingAwgnMax) return FadingClass::AWGN;
    if (fading_index < kFadingGoodMax) return FadingClass::GOOD;
    return FadingClass::MODERATE;
}

// A rung whose per-class auto-select SNR anchor is this value is "never auto-selected
// on that fading class" — still reachable via ULTRA_FORCE_DATA_MOD/RATE for measurement.
inline constexpr float kRungDisabledDb = 1.0e9f;

// Waveform + rate recommendation
struct WaveformRecommendation {
    WaveformMode waveform;
    CodeRate rate;
    float estimated_throughput_bps;
};

inline constexpr float kAnyFadingIndex = 1000.0f;
inline constexpr float kNoRateSnrDb = 1000.0f;
inline constexpr uint32_t kOFDMWideSampleRate = 48000;
inline constexpr uint32_t kOFDMWideSymbolSamples = 1024 + 128;
inline constexpr uint32_t kOFDMWideCarriers = 59;

// Per-rate METADATA only — selection is the coherent ladder below, not per-rate gates.
// Kept for frame sizing (info/coded bits, cw count) and the adaptive climb/drop
// (next/previousOFDMRateDescriptor walk these by code_rate).
struct OFDMCodeRateDescriptor {
    CodeRate rate = CodeRate::R1_4;
    float code_rate = 0.25f;
    uint32_t info_bits_per_codeword = 162;
    uint32_t coded_bits_per_codeword = v2::LDPC_CODEWORD_BITS;
    int wide_cw_count = v2::kDefaultFixedFrameCodewords;
};

inline constexpr std::array<OFDMCodeRateDescriptor, 5> kOFDMCodeRateDescriptors{{
    // rate           code_rate    K    coded                    cw
    {CodeRate::R1_4, 0.25f,       162, v2::LDPC_CODEWORD_BITS, v2::kDefaultFixedFrameCodewords},
    {CodeRate::R1_2, 0.50f,       324, v2::LDPC_CODEWORD_BITS, 8},
    {CodeRate::R2_3, 2.0f / 3.0f, 432, v2::LDPC_CODEWORD_BITS, 8},
    {CodeRate::R3_4, 0.75f,       486, v2::LDPC_CODEWORD_BITS, 8},
    {CodeRate::R5_6, 5.0f / 6.0f, 540, v2::LDPC_CODEWORD_BITS, 8},
}};

inline const OFDMCodeRateDescriptor* ofdmCodeRateDescriptor(CodeRate rate) {
    for (const auto& descriptor : kOFDMCodeRateDescriptors) {
        if (descriptor.rate == rate) {
            return &descriptor;
        }
    }
    return nullptr;
}

inline const OFDMCodeRateDescriptor& fallbackOFDMCodeRateDescriptor() {
    return kOFDMCodeRateDescriptors.front();
}

// ============================================================================
// COHERENT OFDM RATE LADDER — the single source of truth for auto (mod, rate)
// selection (replaces the old 4-gate-array × 3-pass machinery).
//
// The OFDM band is coherent-only (differential retired to MC-DPSK). One ordered
// ladder of (modulation, code-rate) rungs, each with a per-fading-class minimum
// in-band SNR "anchor" (dB). recommendDataMode() walks it HIGH throughput → LOW
// and picks the first rung the (snr, fading_class) clears; the floor rung
// (QPSK R1/4) anchors at the OFDM entry floors, so once OFDM is selected at all it
// always qualifies. Anchors come from MEASUREMENT, not hand-tuning — see
// docs/RATE_LADDER_ANCHORS.md. kRungDisabledDb = not auto-selected yet (forceable).
// ============================================================================
struct CoherentRung {
    Modulation mod;
    CodeRate rate;
    float min_snr_db[3];  // indexed by FadingClass: [AWGN, GOOD, MODERATE]
};

inline constexpr CoherentRung kCoherentLadder[] = {
    // mod              rate            AWGN             GOOD             MODERATE
    {Modulation::QAM16, CodeRate::R3_4, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R2_3, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R1_2, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QPSK,  CodeRate::R3_4, {15.0f,           20.0f,           kRungDisabledDb}},  // AWGN@15 measure_ack_fer 2026-06-06 (data4_full floor ~12 dB + ~3 margin); Good@20 measured 2026-06-02
    {Modulation::QPSK,  CodeRate::R2_3, {12.0f,           15.0f,           20.0f}},            // AWGN@12 measure_ack_fer 2026-06-06 (floor ~8 dB + ~4 margin); Good@15 measured 2026-06-02; MOD@20 measured 2026-06-09 (genuine moderate R2/3 9/9 PASS @20-24 dB, qso_sweep) — softens the Good/Moderate cliff: a misclassification now costs 1 rung (R3/4->R2/3) not 2 (R3/4->R1/2)
    {Modulation::QPSK,  CodeRate::R1_2, {10.0f,           10.0f,           18.0f}},            // AWGN/Good 10: Good@10 measured + AWGN≤Good monotonicity (was 12/14); MOD 18 unmeasured-lower
    {Modulation::QPSK,  CodeRate::R1_4, {kOFDMEntryFloorAwgnDb,            // FLOOR = OFDM entry floors
                                         kOFDMEntryFloorGoodDb,
                                         kOFDMEntryFloorModerateDb}},
};

// ── Phase 1 (2026-06-12): env-gated experimental QAM16 ladder ──────────────────
// ULTRA_ENABLE_QAM16_LADDER=1 lets the AUTO path SELECT 16QAM (vs ULTRA_FORCE_DATA_MOD,
// which bypasses the ladder entirely). Default OFF → the default kCoherentLadder above
// is used unchanged (byte-identical behavior). The experimental ladder enables ONLY the
// 16QAM R1/2 Good rung at its measured-clean floor (Good@18: Phase 0a 5/5 PASS, 0
// CW-fails — fable_analysis/07 + data_phase0a_sweep_2026-06-12.tsv). 16QAM AWGN/Moderate
// and the higher-rate 16QAM rungs stay DISABLED: R2/3+ are damage-bound on Good until the
// dense-rung margins work lands (fable_analysis Phase 2b). The CodeRate RateController WILL
// still probe up into 16QAM R2/3 within the fixed modulation and ssthresh-pin back — that
// is the behavior this gate exists to observe, not yet to ship. Lateral vs QPSK R3/4 today
// (16QAM R1/2 ≈ QPSK R3/4, Phase 0a); the value is exercising the real CONNECT-negotiation
// + RX-accept + adaptive path, which the forced knob cannot.
inline bool qam16LadderEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_ENABLE_QAM16_LADDER");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return on;
}

// ── 16QAM R3/4 crest rung (2026-07-03): env-gated A/B knob ─────────────────────
// ULTRA_QAM16_R34=1 lets the ADAPTIVE walk (applyAdaptiveRateFeedback, connection.cpp)
// step QAM16 R2/3 -> R3/4 after a clean-group streak, and lifts the QAM16 cap in
// maxValidatedCoherentRate() below to match. Default OFF -> byte-identical. The rung
// is NEVER connect-time selectable: BOTH ladders keep {QAM16, R3/4} at kRungDisabledDb,
// so the only way in is the mid-stream walk (which also owns the immediate demote).
inline bool qam16R34Enabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_QAM16_R34");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return on;
}

inline constexpr CoherentRung kCoherentLadderQAM16Exp[] = {
    // mod              rate            AWGN             GOOD             MODERATE
    {Modulation::QAM16, CodeRate::R3_4, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R2_3, {kRungDisabledDb, 26.0f,           kRungDisabledDb}},  // GOOD 20.0 -> 26.0 (2026-07-26, MEASURED): at 20.0 this rung measures **51.4% FER** on ITU Good and NEVER overtakes 8PSK R2/3 anywhere in 16-24 dB (delivered @20 1890 vs 2450; @24 2700 vs 2754). The ladder was confidently selecting a rung that fails HALF its frames, cratering, demoting and reaching again -- the dominant source of rate churn. 26.0 is EXTRAPOLATED, not measured: 16QAM R2/3 is still behind at the top swept point (24 dB) and the two curves converge, so it should overtake somewhere around 25-26. Set above the highest measured point where 8PSK still wins, which preserves the rung for a genuinely better channel rather than deleting it. Re-measure above 24 dB before trusting 26. Original note — MEASURED WRONG 2026-07-26: this Good anchor of 20.0 sits at 51.4% FER on ITU Good fading (6 seeds x 24 frames, tools/sweep_fading_anchors.sh) — the ladder selects a rung that fails HALF its frames, craters, demotes and reaches again, which is the dominant source of rate churn. It is the zero-margin anchor this comment already warns about. Correction pending a rig A/B (docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md §4). Original note — 2026-06-14: Good@20 + cross-frame TIME interleave (auto-on for QAM16) = ~1790 bps, 6/6 PASS — beats the R1/2 rung (~1550). EXPERIMENTAL zero-margin anchor (measured FLOOR, not floor+2); raise to ~22 for margin parity before default, beware Moderate-misclassified-as-Good. Below 20 -> R1/2@18.
    {Modulation::QAM16, CodeRate::R1_2, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},  // GOOD 18.0 -> DISABLED (2026-07-26): ⚠️ THE "STRICTLY DOMINATED" CLAIM BELOW IS MEASURABLY FALSE (2026-07-29). Full-family sweep, ITU Good @20, frame path, 4 seeds x n=100 (n=400/cell, sigma ~2 pts): 16QAM R1/2 passes **87.2%** vs 8PSK R2/3 **82.2%** — the 16QAM rung is ~5 points BETTER at identical eta=2.0, not dominated. The "prefer the lower order at equal eta" argument is a CONSTELLATION-GEOMETRY estimate that ignores the code rate: R1/2 buys more FEC margin than the denser constellation costs. Same class of error as the old 8PSK Good 19.0 anchor (derived as "QPSK R2/3 +3.6 constellation", never measured). **The rung nevertheless STAYS DISABLED**, because the SECOND reason in the note below is untouched and still real: the predictive climb jumped idx 3 -> 7 purely because 7 > 5 and parked the link there. Re-enabling requires fixing the climb, not just correcting this claim. Original note follows — STRICTLY DOMINATED by 8PSK R2/3 — identical eta (2.0) but denser constellation AND non-constant envelope, while 8PSK R2/3 now sits at Good 17. Leaving it enabled let the PREDICTIVE CLIMB jump idx 3 -> 7 (measured) purely because 7 > 5, parking the link on the worse of two equal-rate rungs; that run did not finish inside 420 s. At equal spectral efficiency, prefer the lower order.
    {Modulation::QPSK,  CodeRate::R3_4, {15.0f,           20.0f,           kRungDisabledDb}},
    {Modulation::QPSK,  CodeRate::R2_3, {12.0f,           15.0f,           20.0f}},
    {Modulation::QPSK,  CodeRate::R1_2, {10.0f,           10.0f,           18.0f}},
    {Modulation::QPSK,  CodeRate::R1_4, {kOFDMEntryFloorAwgnDb,
                                         kOFDMEntryFloorGoodDb,
                                         kOFDMEntryFloorModerateDb}},
};

// ── 8PSK revival (2026-07-05): env-gated coherent QAM8 ladder ──────────────────
// ULTRA_ENABLE_PSK8_LADDER=1 swaps in a ladder with the coherent 8PSK (QAM8) rungs
// enabled ALONGSIDE the experimental QAM16 rungs. Case (handoff §7.7): constant
// envelope (immune to the cheap-card compression that craters 16QAM above drive
// ~0.70), +3.6 dB-over-QPSK margin (vs 16QAM's ~+7) = the sweet spot between
// 16QAM R1/2 and R2/3; cw12 normalizes frames to the proven ~1272 ms.
// BUG-8PSK-001 (DD corrupting the 8PSK estimate on fading) was FIXED 2026-05-29
// (channel-adaptive DD gate) — re-probed 2026-07-05 on the modern stack: forced
// QAM8 R2/3 good@20 s42 PASS 2040 bps, 0 craters, 0 RTOs (the May "too marginal
// on Good" verdict is stale). Anchors: QAM8 = QPSK anchor + 3.6 dB (constellation
// distance), ordered consistently with the measured 16QAM anchors
// (QAM16 R2/3 G20 > QAM8 R2/3 G19 > QAM16 R1/2 G18); QAM8 R3/4 AWGN from the May
// AWGN validation (2330 bps clean). PROBE anchors — refine by measurement.
inline bool psk8LadderEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_ENABLE_PSK8_LADDER");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return on;
}

inline constexpr CoherentRung kCoherentLadderPsk8Exp[] = {
    // mod              rate            AWGN             GOOD             MODERATE
    {Modulation::QAM16, CodeRate::R3_4, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R2_3, {kRungDisabledDb, 26.0f,           kRungDisabledDb}},  // AWGN COLUMN — MEASURED 2026-07-28 AND DELIBERATELY LEFT DISABLED. Forced 16QAM R2/3 on OTASim AWGN passes at EVERY swept dial with ZERO craters: dial 20 -> 3250 bps (26/26 frames OK), 16 -> 3240, 14 -> 3110, 12 -> 3270 (27/28 frames). vs 8PSK R2/3 auto at AWGN@20 = 2430, i.e. +33.7% measured against +33.5% predicted from 2.67/2.00 bits-per-carrier. In-band reads dial-0.2 there, so the lowest working point is snr_avg ~20.5 and the AWGN floor is BELOW the swept range. THE RUNG WORKS. It stays disabled because the HAZARD IS THE CLASSIFIER, NOT THE RUNG: 8PSK R3/4 was auto-disabled 2026-07-06 for exactly this shape — "the A18 anchor was validated on TRUE AWGN only; on the Watterson bench a 60-90 s calm stretch legitimately classifies AWGN, unlocks this rung, and it craters when fading returns (F141/F143: 3 runs poisoned)". 16QAM R2/3 measures 51.4% FER on Good, and the Good/AWGN discriminator is a known-open bug (BUG-FADING-INDEX-BLIND; the coherence discriminator). A calm stretch misclassified as AWGN would unlock a rung that fails half its frames — the identical failure with a worse rung. UNLOCK CONDITION: a trustworthy sustained-AWGN classification (or the effective-SINR/PER selector, which makes the column moot because it predicts PER from the measured per-carrier grid instead of a channel-class label). Reproduce: ULTRA_FORCE_DATA_MOD=QAM16 ULTRA_FORCE_DATA_RATE=R2_3 ULTRA_LOCK_RATE=1 ULTRA_RX_RATE_AUTHORITY=0 tools/gui_qso_scenario.sh --channel awgn --snr-db 12 --expect-mod 16QAM --expect-rate R2/3. GOOD 20.0 -> 26.0 (2026-07-26, MEASURED, LIVE LADDER): at 20.0 this rung measures **51.4% FER** on ITU Good and NEVER overtakes 8PSK R2/3 anywhere in 16-24 dB (delivered @20 1890 vs 2450; @24 2700 vs 2754). The ladder was confidently selecting a rung that fails HALF its frames, cratering, demoting and reaching again -- the dominant source of rate churn. 26.0 is EXTRAPOLATED, not measured: 16QAM R2/3 is still behind at the top swept point (24 dB) and the curves converge, so it should overtake around 25-26. Set above the highest measured point where 8PSK still wins, preserving the rung for a genuinely better channel rather than deleting it. Re-measure above 24 dB before trusting 26.
    {Modulation::QAM8,  CodeRate::R3_4, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},  // KEEP DISABLED — the demanded FADING validation WAS RUN 2026-07-26 and FAILED: on ITU Good (= MPG) this rung measures 76.4/60.4/50.7/42.4/38.9% FER at 16/18/20/22/24 dB, i.e. it NEVER approaches the 10% floor in the swept range (tools/sweep_fading_anchors.sh; docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md). Attractive on paper (constant envelope, immune to the cheap-card compression that craters 16QAM above drive ~0.70, lower nominal requirement than 16QAM R2/3) but it does not survive fading. Original note — AUTO-DISABLED 2026-07-06: the A18 anchor was validated on TRUE AWGN only; on the Watterson bench a 60-90 s calm stretch legitimately classifies AWGN, unlocks this rung, and it craters when fading returns (F141/F143/F143-probe: 3 runs poisoned). Re-enable only after a FADING validation. Forceable for measurement.
    {Modulation::QAM8,  CodeRate::R2_3, {16.0f,           17.0f,           kRungDisabledDb}},  // GOOD 19.0 -> 17.0 (2026-07-26, MEASURED): the anchor belongs at the THROUGHPUT CROSSOVER, not the FER<=10% floor -- a denser rung carrying more retx still wins while rate x (1-FER) is higher. Swept on ITU Good (= MPG), delivered bps: @16 QPSK R3/4 1686 vs 8PSK R2/3 1499; @18 1914 vs 2106; @20 2066 vs 2450; @22 2096 vs 2673; @24 2111 vs 2754. Interpolated crossover 17.0 dB, and 8PSK R2/3 then wins at EVERY higher swept point. The old 19.0 was derived as "QPSK R2/3 anchor +3.6 (constellation)" -- a geometry estimate, never a fading measurement. Lowering it also makes QAM16 R1/2 (identical eta = 2.0, but denser constellation AND non-constant envelope) unreachable, which is correct: at equal spectral efficiency prefer the lower-order modulation. NOTE: only the GOOD column is measured; AWGN 16.0 and MODERATE are untouched. Original note — 2026-07-05 probe: good@20 s42 PASS 2040, 0 craters (forced). QPSK R2/3 anchor +3.6 -> A16/G19
    {Modulation::QAM16, CodeRate::R1_2, {kRungDisabledDb, kRungDisabledDb, kRungDisabledDb}},  // GOOD 18.0 -> DISABLED (2026-07-26): ⚠️ THE "STRICTLY DOMINATED" CLAIM BELOW IS MEASURABLY FALSE (2026-07-29). Full-family sweep, ITU Good @20, frame path, 4 seeds x n=100 (n=400/cell, sigma ~2 pts): 16QAM R1/2 passes **87.2%** vs 8PSK R2/3 **82.2%** — the 16QAM rung is ~5 points BETTER at identical eta=2.0, not dominated. The "prefer the lower order at equal eta" argument is a CONSTELLATION-GEOMETRY estimate that ignores the code rate: R1/2 buys more FEC margin than the denser constellation costs. Same class of error as the old 8PSK Good 19.0 anchor (derived as "QPSK R2/3 +3.6 constellation", never measured). **The rung nevertheless STAYS DISABLED**, because the SECOND reason in the note below is untouched and still real: the predictive climb jumped idx 3 -> 7 purely because 7 > 5 and parked the link there. Re-enabling requires fixing the climb, not just correcting this claim. Original note follows — STRICTLY DOMINATED by 8PSK R2/3 — identical eta (2.0) but denser constellation AND non-constant envelope, while 8PSK R2/3 now sits at Good 17. Leaving it enabled let the PREDICTIVE CLIMB jump idx 3 -> 7 (measured) purely because 7 > 5, parking the link on the worse of two equal-rate rungs; that run did not finish inside 420 s. At equal spectral efficiency, prefer the lower order.
    {Modulation::QPSK,  CodeRate::R3_4, {15.0f,           20.0f,           kRungDisabledDb}},
    {Modulation::QPSK,  CodeRate::R2_3, {12.0f,           15.0f,           20.0f}},
    {Modulation::QPSK,  CodeRate::R1_2, {10.0f,           10.0f,           18.0f}},
    {Modulation::QPSK,  CodeRate::R1_4, {kOFDMEntryFloorAwgnDb,
                                         kOFDMEntryFloorGoodDb,
                                         kOFDMEntryFloorModerateDb}},
};

struct CoherentPick {
    Modulation mod;
    CodeRate rate;
};

// Highest ladder rung whose per-class anchor the (snr, fading) clears. Falls back
// to the QPSK R1/4 floor (the last entry always qualifies once above the entry floor).
inline CoherentPick selectCoherentOFDM(float snr_db, float fading_index) {
    // Poor (>= kFadingModerateMax) routes to MC-DPSK upstream and OFDM is never
    // selected there; hold the QPSK R1/4 floor defensively if ever reached.
    if (fading_index >= kFadingModerateMax) {
        return {Modulation::QPSK, CodeRate::R1_4};
    }
    const int cls = static_cast<int>(classifyFading(fading_index));
    // Default = kCoherentLadder (QPSK-only auto). ULTRA_ENABLE_QAM16_LADDER swaps in the
    // experimental ladder with the measured 16QAM R1/2 Good rung enabled — default OFF.
    const CoherentRung* rungs = kCoherentLadder;
    size_t n = sizeof(kCoherentLadder) / sizeof(kCoherentLadder[0]);
    if (psk8LadderEnabled()) {
        // 8PSK revival ladder (includes the QAM16-exp rungs — see its comment).
        rungs = kCoherentLadderPsk8Exp;
        n = sizeof(kCoherentLadderPsk8Exp) / sizeof(kCoherentLadderPsk8Exp[0]);
    } else if (qam16LadderEnabled()) {
        rungs = kCoherentLadderQAM16Exp;
        n = sizeof(kCoherentLadderQAM16Exp) / sizeof(kCoherentLadderQAM16Exp[0]);
    }
    for (size_t i = 0; i < n; ++i) {
        if (snr_db >= rungs[i].min_snr_db[cls]) {
            return {rungs[i].mod, rungs[i].rate};
        }
    }
    return {Modulation::QPSK, CodeRate::R1_4};
}

// ═══════════ RX-AUTHORITY canonical rung index (2026-07-05) ═══════════
// A WIRE-STABLE absolute index for every coherent wideband (mod, rate) rung —
// knob-INDEPENDENT (unlike a ladder-array index, which reorders across the three
// ladder variants) and NOT LadderRungId (a 3-bit waveform-level id; the whole
// coherent family collapses to one value there). Carried in the tone-burst ACK's
// reinterpreted [rate_hint|rung_cmd] bits under ULTRA_RX_RATE_AUTHORITY: the
// RECEIVER measures the channel and commands the sender's next rung outright
// (single decision-maker, sitting where the information is). 0 = no command
// (keeps the all-zeros knob-OFF wire identity). Values fit 5 bits (1..31).
enum : uint8_t {
    kRungIdxNone = 0,
    kRungIdxQpskR14 = 1,
    kRungIdxQpskR12 = 2,
    kRungIdxQpskR23 = 3,
    kRungIdxQpskR34 = 4,
    kRungIdxQam8R23 = 5,
    kRungIdxQam8R34 = 6,
    kRungIdxQam16R12 = 7,
    kRungIdxQam16R23 = 8,
    kRungIdxQam16R34 = 9,
    kRungIdxCount = 10,  // first unassigned
};

// The authority field is five bits wide while canonical rung indices occupy only the
// low nibble. Bit 4 marks the bounded R1/2->R2/3 startup command as a one-shot physical
// probe. An upgraded sender rolls back before retransmitting if that probe receives no
// ACK; an older sender safely ignores the out-of-range value.
inline constexpr uint8_t kRungAuthorityStartupProbeFlag = 0x10;
inline constexpr uint8_t kRungAuthorityIndexMask = 0x0F;
static_assert(kRungIdxCount <= kRungAuthorityStartupProbeFlag,
              "canonical rung indices must fit below the startup-probe flag");

// (mod, rate) -> canonical index; kRungIdxNone when the pair is not a coherent rung.
inline uint8_t coherentRungIndexFor(Modulation mod, CodeRate rate) {
    if (mod == Modulation::QPSK) {
        switch (rate) {
            case CodeRate::R1_4: return kRungIdxQpskR14;
            case CodeRate::R1_2: return kRungIdxQpskR12;
            case CodeRate::R2_3: return kRungIdxQpskR23;
            case CodeRate::R3_4: return kRungIdxQpskR34;
            default: return kRungIdxNone;
        }
    }
    if (mod == Modulation::QAM8) {
        if (rate == CodeRate::R2_3) return kRungIdxQam8R23;
        if (rate == CodeRate::R3_4) return kRungIdxQam8R34;
        return kRungIdxNone;
    }
    if (mod == Modulation::QAM16) {
        if (rate == CodeRate::R1_2) return kRungIdxQam16R12;
        if (rate == CodeRate::R2_3) return kRungIdxQam16R23;
        if (rate == CodeRate::R3_4) return kRungIdxQam16R34;
        return kRungIdxNone;
    }
    return kRungIdxNone;
}

// canonical index -> (mod, rate); QPSK R1/4 floor for anything unknown.
inline CoherentPick coherentRungFromIndex(uint8_t idx) {
    switch (idx) {
        case kRungIdxQpskR14: return {Modulation::QPSK, CodeRate::R1_4};
        case kRungIdxQpskR12: return {Modulation::QPSK, CodeRate::R1_2};
        case kRungIdxQpskR23: return {Modulation::QPSK, CodeRate::R2_3};
        case kRungIdxQpskR34: return {Modulation::QPSK, CodeRate::R3_4};
        case kRungIdxQam8R23: return {Modulation::QAM8, CodeRate::R2_3};
        case kRungIdxQam8R34: return {Modulation::QAM8, CodeRate::R3_4};
        case kRungIdxQam16R12: return {Modulation::QAM16, CodeRate::R1_2};
        case kRungIdxQam16R23: return {Modulation::QAM16, CodeRate::R2_3};
        case kRungIdxQam16R34: return {Modulation::QAM16, CodeRate::R3_4};
        default: return {Modulation::QPSK, CodeRate::R1_4};
    }
}

// Parse the live measurement overrides atomically.  A present-but-invalid field makes
// the whole environment profile malformed: accepting the other half would silently turn
// a typo into a different physical experiment.  A valid field with the other field
// UNSET remains a supported partial override; its complement is selected automatically.
struct EnvironmentForcedDataProfile {
    Modulation modulation = Modulation::AUTO;
    CodeRate code_rate = CodeRate::AUTO;
    bool modulation_present = false;
    bool code_rate_present = false;
    bool malformed = false;

    bool forced() const {
        return !malformed &&
               (modulation != Modulation::AUTO || code_rate != CodeRate::AUTO);
    }
};

inline EnvironmentForcedDataProfile forcedDataProfileFromEnvironment() {
    EnvironmentForcedDataProfile profile;
    if (const char* env = std::getenv("ULTRA_FORCE_DATA_MOD")) {
        profile.modulation_present = true;
        const std::string s(env);
        if      (s == "DBPSK")                  profile.modulation = Modulation::DBPSK;
        else if (s == "BPSK")                   profile.modulation = Modulation::BPSK;
        else if (s == "DQPSK")                  profile.modulation = Modulation::DQPSK;
        else if (s == "QPSK")                   profile.modulation = Modulation::QPSK;
        else if (s == "D8PSK")                  profile.modulation = Modulation::D8PSK;
        else if (s == "QAM8" || s == "8PSK")   profile.modulation = Modulation::QAM8;
        else if (s == "QAM16" || s == "16QAM") profile.modulation = Modulation::QAM16;
        else if (s == "QAM32" || s == "32QAM") profile.modulation = Modulation::QAM32;
        else                                      profile.malformed = true;
    }
    if (const char* env = std::getenv("ULTRA_FORCE_DATA_RATE")) {
        profile.code_rate_present = true;
        const std::string s(env);
        if      (s == "R1_4" || s == "r1_4") profile.code_rate = CodeRate::R1_4;
        else if (s == "R1_3" || s == "r1_3") profile.code_rate = CodeRate::R1_3;
        else if (s == "R1_2" || s == "r1_2") profile.code_rate = CodeRate::R1_2;
        else if (s == "R2_3" || s == "r2_3") profile.code_rate = CodeRate::R2_3;
        else if (s == "R3_4" || s == "r3_4") profile.code_rate = CodeRate::R3_4;
        else if (s == "R5_6" || s == "r5_6") profile.code_rate = CodeRate::R5_6;
        else                                    profile.malformed = true;
    }
    if (profile.malformed) {
        // Fail the environment override as one unit; never preserve a valid half next
        // to a typo.  Callers that own logging can diagnose presence/malformed explicitly.
        profile.modulation = Modulation::AUTO;
        profile.code_rate = CodeRate::AUTO;
    }
    return profile;
}

inline Modulation forcedDataModulationFromEnvironment() {
    return forcedDataProfileFromEnvironment().modulation;
}

inline CodeRate forcedDataCodeRateFromEnvironment() {
    return forcedDataProfileFromEnvironment().code_rate;
}

inline bool environmentForcesDataProfile() {
    return forcedDataProfileFromEnvironment().forced();
}

// The CONNECT measurement is made on MC-DPSK and does not predict the first
// coherent-OFDM payload margin reliably on a fading channel.  RX authority has
// no coherent observations yet, so its automatic bootstrap is deliberately the
// proven QPSK R1/2 rung; the first real OFDM group then hands control to the
// measured-outcome ladder.  An already-resolved operator force bypasses this cap.
inline CoherentPick capRxAuthorityInitialRung(Modulation mod,
                                              CodeRate rate,
                                              bool authority_enabled,
                                              bool operator_forced = false) {
    if (!authority_enabled || operator_forced) return {mod, rate};
    const uint8_t idx = coherentRungIndexFor(mod, rate);
    if (idx == kRungIdxNone || idx <= kRungIdxQpskR12) return {mod, rate};
    return {Modulation::QPSK, CodeRate::R1_2};
}

// A commanded rung is obeyable only if the LOCAL ladder knows it (env knobs may
// differ across ends): clamp defensively rather than transmit a rung the local
// tables never validated.
inline bool coherentRungLocallyEnabled(Modulation mod, CodeRate rate) {
    const CoherentRung* rungs = kCoherentLadder;
    size_t n = sizeof(kCoherentLadder) / sizeof(kCoherentLadder[0]);
    if (psk8LadderEnabled()) {
        rungs = kCoherentLadderPsk8Exp;
        n = sizeof(kCoherentLadderPsk8Exp) / sizeof(kCoherentLadderPsk8Exp[0]);
    } else if (qam16LadderEnabled()) {
        rungs = kCoherentLadderQAM16Exp;
        n = sizeof(kCoherentLadderQAM16Exp) / sizeof(kCoherentLadderQAM16Exp[0]);
    }
    for (size_t i = 0; i < n; ++i) {
        if (rungs[i].mod == mod && rungs[i].rate == rate) {
            for (int c = 0; c < 3; ++c) {
                if (rungs[i].min_snr_db[c] < kRungDisabledDb) return true;
            }
            return false;  // fully-disabled row = placeholder, not selectable
        }
    }
    return false;
}

// ── RX-AUTHORITY PREDICTIVE (docs/RX_AUTHORITY_PREDICTIVE_2026_07_07.md) ──
// Calibration anchor for a rung: the MINIMUM enabled column, honoring the
// active ladder variant. The AWGN column is the flat-channel physics floor and
// wins when present; several rungs (16QAM R2/3, 16QAM R1/2) are enabled ONLY
// in fading columns — their column value carries scalar-fading margin, which
// makes the derived EESM alpha CONSERVATIVE (under-predicts capacity): the
// safe direction for a climb decision. kRungDisabledDb if fully disabled.
inline float calibrationAnchorDbFor(Modulation mod, CodeRate rate) {
    const CoherentRung* rungs = kCoherentLadder;
    size_t n = sizeof(kCoherentLadder) / sizeof(kCoherentLadder[0]);
    if (psk8LadderEnabled()) {
        rungs = kCoherentLadderPsk8Exp;
        n = sizeof(kCoherentLadderPsk8Exp) / sizeof(kCoherentLadderPsk8Exp[0]);
    } else if (qam16LadderEnabled()) {
        rungs = kCoherentLadderQAM16Exp;
        n = sizeof(kCoherentLadderQAM16Exp) / sizeof(kCoherentLadderQAM16Exp[0]);
    }
    for (size_t i = 0; i < n; ++i) {
        if (rungs[i].mod == mod && rungs[i].rate == rate) {
            float best = kRungDisabledDb;
            for (int c = 0; c < 3; ++c) {
                if (rungs[i].min_snr_db[c] < best) best = rungs[i].min_snr_db[c];
            }
            return best;
        }
    }
    return kRungDisabledDb;
}

// ── RADIO-AGNOSTIC EVM-USABLE RUNG FLOORS (Stage 2, ULTRA_EVM_DEMOTE) ──────────
// The decision-directed EVM SNR estimator (src/ofdm/demodulator_impl.hpp) measures
// the scatter of the equalized constellation around its hard decisions — it reads
// USABLE (post-equalizer, demod-domain) SNR, is CONSTANT-FREE, and CANNOT INFLATE
// (a gain-corrected error power can only grow as the channel worsens). That makes it
// radio-agnostic by construction: no per-soundcard reference level, no hardware-loss
// offset. It is exactly the honest quantity the +8.70 dB kOfdmLegacyAnchorScaleOffsetDb
// "quarantine" was faking — that offset marks the OFDM broadband estimator up to the
// dial/channel scale the legacy anchors assume, which manufactured the IONOS 16QAM
// over-commit (WGN@20 read 14.1 → 22.8 → selects 16QAM R2/3, a rung the real ~15 dB
// usable channel cannot hold). EVM reads the ~15 directly.
//
// These per-rung floors are the EVM-median at which each rung reached FULL goodput on
// the OTASim AWGN forced-rung sweep (2026-07-24; /private/tmp/evm_anchors/ANCHOR_TABLE.md):
//   QPSK R2/3 9.4 · QPSK R3/4 12.5 · 8PSK R2/3 13.7 · 16QAM R1/2 14.9 · 16QAM R2/3 16.7.
// Measured rungs are exact; the two unmeasured rungs (QAM8 R3/4, QAM16 R3/4 — both
// currently ladder holes) interpolate/extrapolate CONSERVATIVELY (a HIGHER floor
// demotes SOONER, the safe direction for a demote-only clamp). USABLE dB throughout —
// do NOT add any dial/hardware offset. Because EVM saturates at the demod ceiling
// (QPSK ~16, 16QAM ~20), this is a DEMOTE authority (clamp cmd DOWN), never a climb
// input. kRungDisabledDb for a non-rung index.
inline float evmUsableFloorDbForRung(uint8_t rung_idx) {
    switch (rung_idx) {
        case kRungIdxQpskR14:  return 6.5f;   // unmeasured; more robust than R1/2
        case kRungIdxQpskR12:  return 8.5f;   // fail@dial8, pass@dial10 (EVM 9.4)
        case kRungIdxQpskR23:  return 9.4f;   // measured
        case kRungIdxQpskR34:  return 12.5f;  // measured (dial13 11.9 partial→dial15 13.7 clean)
        case kRungIdxQam8R23:  return 13.7f;  // measured
        case kRungIdxQam8R34:  return 15.5f;  // interp 8PSK R2/3(13.7)↔16QAM R2/3(16.7)
        case kRungIdxQam16R12: return 14.9f;  // measured (13.3 partial)
        case kRungIdxQam16R23: return 16.7f;  // measured
        case kRungIdxQam16R34: return 19.5f;  // extrap above 16QAM R2/3
        default: return kRungDisabledDb;
    }
}

// Hold margin (dB) below a rung's EVM-usable floor before the demote fires — a single
// group's usable EVM must be clearly (≥ this) under the floor, damping boundary thrash.
// A modest fraction of the ~3 dB inter-rung EVM spacing measured on the sweep.
inline constexpr float kEvmDemoteHoldMarginDb = 1.0f;

// ULTRA_ENTRY_EVM_CAP (default-off): cap the CONNECT-time entry rung by the usable-domain
// EVM floors above, using the data-aided connect SNR (itself a usable-domain measurement).
// Rig evidence (MPM@20, 2026-07-25): an honest 8.3 dB usable reading entered at QPSK R3/4
// (needs ~12.5 usable) because the selection basis is a ~17 dB dial-equivalent -> five
// fully-cratered groups and ~136 s of dead air before the ladder walked down. Cap-only,
// data-aided-only: it can lower the entry rung, never raise it.
inline bool entryEvmCapEnabled() {
    const char* e = std::getenv("ULTRA_ENTRY_EVM_CAP");
    return e && e[0] == '1' && e[1] == '\0';
}

// Highest canonical rung whose EVM-usable floor the measured usable EVM clears (with
// a hold margin so a single noisy read does not thrash the boundary). Demote-only:
// the caller clamps cmd = min(cmd, this). Floors at QPSK R1/4 (the lowest rung — the
// clamp never strands below it). Walks top-down; the first index whose floor clears
// is the most permissive supportable rung.
inline uint8_t highestRungSupportedByEvm(float evm_db, float hold_margin_db) {
    for (uint8_t r = kRungIdxCount - 1; r > kRungIdxQpskR14; --r) {
        if (evm_db >= evmUsableFloorDbForRung(r) - hold_margin_db) return r;
    }
    return kRungIdxQpskR14;
}

// Anchor for one rung at the SPECIFIC fading class, honoring the active ladder
// variant. Unlike coherentLadderAnchorDb() (reads ONLY kCoherentLadder, so returns
// kRungDisabledDb for the experimental 16QAM/8PSK rungs) and calibrationAnchorDbFor()
// (ladder-aware but returns the class-BLIND minimum column), this gives the current
// rung's floor ON the current channel class. Used by the EMA-supported crater-hold:
// "does the fade-averaged SNR clear THIS rung's calibrated floor on THIS class?"
// If the class column is disabled but another is enabled (e.g. 16QAM enabled only in
// fading columns), fall back to the minimum enabled column (the rung's physics floor).
inline float rungClassAnchorDb(Modulation mod, CodeRate rate, float fading_index) {
    const int cls = static_cast<int>(classifyFading(fading_index));
    const CoherentRung* rungs = kCoherentLadder;
    size_t n = sizeof(kCoherentLadder) / sizeof(kCoherentLadder[0]);
    if (psk8LadderEnabled()) {
        rungs = kCoherentLadderPsk8Exp;
        n = sizeof(kCoherentLadderPsk8Exp) / sizeof(kCoherentLadderPsk8Exp[0]);
    } else if (qam16LadderEnabled()) {
        rungs = kCoherentLadderQAM16Exp;
        n = sizeof(kCoherentLadderQAM16Exp) / sizeof(kCoherentLadderQAM16Exp[0]);
    }
    for (size_t i = 0; i < n; ++i) {
        if (rungs[i].mod == mod && rungs[i].rate == rate) {
            if (rungs[i].min_snr_db[cls] < kRungDisabledDb) return rungs[i].min_snr_db[cls];
            float best = kRungDisabledDb;  // class column disabled → rung's physics floor
            for (int c = 0; c < 3; ++c) {
                if (rungs[i].min_snr_db[c] < best) best = rungs[i].min_snr_db[c];
            }
            return best;
        }
    }
    return kRungDisabledDb;
}

// EESM virtual rung evaluation: is rung (mod, rate) predicted decodable on the
// measured per-carrier SNR snapshot {gamma_k} (LINEAR, normalized to the
// in-band scale)? Per-carrier capacity fraction f(γ) = 1 − exp(−γ/α) with α
// SELF-CALIBRATED from the rung's own AWGN anchor A via the flat-channel
// identity 1 − exp(−A/α) = c  ⇒  α = −A/ln(1−c): on a flat channel this
// reproduces the anchor table EXACTLY (zero new tuned constants), on a
// selective channel strong carriers' surplus compensates weak ones up to the
// code budget — which is what LDPC + the channel interleaver physically do —
// and a parked notch exceeding the budget FAILS the prediction even when the
// mean SNR reads generous (the F149 crest trap, the F163/F165 16QAM craters).
// margin_db shifts every carrier down (climb hysteresis + crater penalty ride
// here). Modulation-adaptive by construction: no per-mod branches; geometry
// enters only through the rung's own measured anchor.
// ============================================================================
// MEASURED FER-FLOOR ANCHORS — a SECOND anchor table, deliberately separate.
// ============================================================================
//
// WHY A SECOND TABLE EXISTS. `min_snr_db` in kCoherentLadder is a LADDER-SELECTION
// threshold, and by explicit design it is NOT the decodability floor. The in-tree note
// on 8PSK R2/3 says so: "GOOD 19.0 -> 17.0 (2026-07-26, MEASURED): the anchor belongs at
// the THROUGHPUT CROSSOVER, not the FER<=10% floor" — placed where a denser rung
// carrying more retransmissions still wins on delivered bps.
//
// That is defensible for CHOOSING a rung. It is wrong as the capacity calibration inside
// rungPredictedSustainable(), whose alpha = -A/ln(1-c) answers "will this rung DECODE",
// not "will it win on goodput". Feeding a crossover constant to a decodability predictor
// makes it optimistic by exactly the gap. ONE CONSTANT WAS SERVING TWO INCOMPATIBLE
// PURPOSES; this separates them.
//
// EVIDENCE (2026-07-29/30). The Good column claims 8PSK R2/3 holds at 17.0 dB while QPSK
// R3/4 needs 20.0 — a denser constellation at HIGHER spectral efficiency (2.0 vs 1.5
// bits/carrier) requiring LESS SNR, which is not physical. Confirmed on the IONOS rig at
// MPG@20 with clean/crater frames grouped by codeword count (one log, one scale, no
// conversion): the 12-CW rung separates perfectly — every crater <=14.0 dB, every success
// >=16.7 dB — and the ladder attempted it 7 times, cratering 5 (71%), committing at ~14 dB.
//
// VALUES: the SNR at which each rung first reaches <=10% FER, measured with
// tools/measure_ack_fer over 9 rungs x {AWGN, Good, Moderate} x SNR 6-28 dB step 2 x
// seeds {7,11} x n=40 (n=80 per point). Provenance is the frame path at LONG CP.
//
// kRungDisabledDb means "no swept SNR in 6-28 dB reached 10% FER", which is the honest
// entry for every Moderate cell — Moderate has an SNR-INDEPENDENT FER FLOOR (QPSK R1/2
// bottoms at ~14% near 12 dB and plateaus ~19% out to 28 dB). A disabled entry makes the
// predictor REFUSE the rung, which is the correct conservative answer: on that class no
// rung is predicted to hold a 10% target at any SNR we can measure.
inline constexpr CoherentRung kMeasuredFerFloor[] = {
    // mod              rate             AWGN   GOOD             MODERATE
    {Modulation::QPSK,  CodeRate::R1_4, { 6.0f, 12.0f,           kRungDisabledDb}},
    {Modulation::QPSK,  CodeRate::R1_2, { 6.0f, 14.0f,           kRungDisabledDb}},
    {Modulation::QPSK,  CodeRate::R2_3, { 8.0f, 16.0f,           kRungDisabledDb}},
    {Modulation::QPSK,  CodeRate::R3_4, { 8.0f, 20.0f,           kRungDisabledDb}},
    {Modulation::QAM8,  CodeRate::R2_3, {14.0f, 22.0f,           kRungDisabledDb}},
    {Modulation::QAM8,  CodeRate::R3_4, {14.0f, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R1_2, {14.0f, 20.0f,           kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R2_3, {16.0f, kRungDisabledDb, kRungDisabledDb}},
    {Modulation::QAM16, CodeRate::R3_4, {16.0f, kRungDisabledDb, kRungDisabledDb}},
};

// KEEPER (docs/REMOVAL_BACKLOG.md R12 anti-footgun): the FER-floor lookup survives the
// removal of the predictor that used to call it. A never-reached cell returns
// kRungDisabledDb so a caller refuses the rung rather than borrowing a more permissive
// class's number — borrowing is precisely how the class-blind minimum became
// AWGN-calibrated on every channel.
inline float rungFerFloorDb(Modulation mod, CodeRate rate, float fading_index) {
    const int cls = static_cast<int>(classifyFading(fading_index));
    for (const auto& r : kMeasuredFerFloor) {
        if (r.mod == mod && r.rate == rate) {
            return r.min_snr_db[cls];
        }
    }
    return kRungDisabledDb;
}

// rungPredictedSustainable() — DELETED 2026-08-01 (docs/REMOVAL_BACKLOG.md R12), together
// with its ULTRA_RUNG_CLASS_ANCHOR / ULTRA_FER_FLOOR_ANCHOR anchor-selection knobs.
//
// It answered "will this rung DECODE", which is the wrong question for rate selection: the
// bar is LOWER for lower-efficiency rungs, so it handed back QPSK R3/4 (2066 bps @20) in
// place of 8PSK R2/3 (2450 incl. craters). Its only production caller was the RX-authority
// predictive climb, retired the same day; measured, when that climb disagreed with the map
// it downgraded 51x against 6 upgrades.
//
// KEPT deliberately (do not remove these while cleaning): kMeasuredFerFloor / rungFerFloorDb
// above — real data at n=80/point across 9 rungs x 3 classes, pinned by
// test_waveform_policy.cpp, and the right source to fit a PER-CLASS theta_r for the latent
// controller, whose table is currently ITU-Good-only. Also rungClassAnchorDb, which has a
// separate caller in connection.cpp.

// Nearest locally-enabled rung at or below idx (canonical order is monotone in
// speed), kRungIdxNone if nothing at/below is enabled. EVERY consumer of rung
// INDEX ARITHMETIC must snap through this: raw index steps (cur-2 strides,
// down-limits) are blind to anchor-table holes — F145 rig: with QAM8 R3/4
// auto-disabled, a confirmed crater at idx 8 commanded the hole (idx 6 = cur-2),
// the sender's enabled-guard refused it, and the link sat 50 s re-cratering
// 16QAM R2/3 (6 whole-burst resends, 0/5 each) with a correct DOWN verdict
// standing the whole time.
inline uint8_t snapRungIndexDownToEnabled(uint8_t idx) {
    if (idx >= kRungIdxCount) idx = kRungIdxCount - 1;
    for (int i = static_cast<int>(idx); i >= kRungIdxQpskR14; --i) {
        const CoherentPick p = coherentRungFromIndex(static_cast<uint8_t>(i));
        if (coherentRungLocallyEnabled(p.mod, p.rate)) return static_cast<uint8_t>(i);
    }
    return kRungIdxNone;
}

// Spectral efficiency of a rung, in information bits per data-carrier symbol:
// modulation bits x code rate. Pure constellation/code geometry — no bench
// constant, no channel assumption, defined for every rung in the family.
inline float rungSpectralEfficiency(uint8_t idx) {
    const CoherentPick p = coherentRungFromIndex(idx);
    return static_cast<float>(getBitsPerSymbol(p.mod)) * getCodeRateValue(p.rate);
}

// GOODPUT BREAK-EVEN DELIVERED FRACTION — the threshold that decides whether a
// lossy group is EVIDENCE THE RUNG IS TOO HIGH, or just the irreducible fading
// loss ARQ exists to absorb.
//
// Derivation (airtime to deliver one payload frame): on rung r with per-group
// delivered fraction f, a frame needs ~1/f transmissions and each transmission
// costs airtime ~1/eta_r, so airtime ~ 1/(f * eta_r). The rung with the larger
// (f * eta) therefore wins on goodput. Comparing the current rung against the
// first ENABLED rung below it, and crediting that lower rung the most optimistic
// f_below = 1 (deliberately conservative: it makes demoting EASIER, never
// harder), staying is right while
//
//     f_cur * eta_cur  >=  1 * eta_below     <=>     f_cur >= eta_below / eta_cur
//
// so the break-even IS the spectral-efficiency ratio of adjacent enabled rungs.
// It is derived from ladder geometry alone, so it is correct for the whole rung
// family by construction and adapts automatically when anchor-table holes move
// the demote target (that is why it snaps through snapRungIndexDownToEnabled —
// the threshold must be measured against the rung we would ACTUALLY land on,
// not idx-1, which may be a hole; F145).
//
// TWO APPROXIMATIONS, IN OPPOSITE DIRECTIONS — stated rather than fudged (a
// correction factor here would be exactly the tuned constant this derivation
// exists to avoid; the rig A/B is the arbiter):
//   (a) Airtime-only. A resend also consumes a share of a half-duplex turnaround,
//       which this ignores => it UNDER-prices staying high. Partly self-limiting:
//       SACK resends ride along in the next burst rather than buying their own
//       turnaround, so the share is amortised, not doubled.
//   (b) f_below = 1. The rung below will not really deliver perfectly, which
//       => OVER-prices demoting.
// (a) and (b) push the threshold opposite ways and partially cancel.
//
// Worked values on the current ladder: 16QAM R2/3 -> 16QAM R1/2 = 0.75 (a 7/8
// group is a WIN and must hold); QPSK R3/4 -> R2/3 = 0.889; QPSK R2/3 -> R1/2
// = 0.75; QPSK R1/2 -> R1/4 = 0.50. Returns 0 at the floor (nothing below to
// demote to, so no delivered fraction can be rung-failure evidence there).
inline float goodputBreakEvenDeliveredFraction(uint8_t cur) {
    if (cur <= kRungIdxQpskR14 || cur >= kRungIdxCount) return 0.0f;
    const uint8_t below = snapRungIndexDownToEnabled(static_cast<uint8_t>(cur - 1));
    if (below == kRungIdxNone || below >= cur) return 0.0f;
    const float eta_cur = rungSpectralEfficiency(cur);
    if (eta_cur <= 0.0f) return 0.0f;
    return rungSpectralEfficiency(below) / eta_cur;
}

// ULTRA_RX_RATE_AUTHORITY (2026-07-05, default ON): receiver-commanded absolute
// rung selection — the receiver maps ITS fresh per-group channel measurements
// through selectCoherentOFDM and commands the sender's next rung on every group
// ACK; the sender OBEYS (descriptor commit) and its own mid-transfer rate drivers
// (EMA walk, climb streaks, crest walks, cooldowns, amnesty) go inert. Sender-side
// ack-SILENCE escapes stay live (the receiver cannot command through a blackout).
// BOTH ends must set it: it reinterprets the ACK's [rate_hint|rung_cmd] bits and
// requires the widened CRC span (a knob-OFF peer CRC-rejects the ACKs — fails safe
// as ack loss). Supersedes ULTRA_RX_RATE_CMD's demote-only command when on.
inline bool rxRateAuthorityEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_RX_RATE_AUTHORITY");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05 (both ends lockstep: widened tone-ACK CRC span)
    }();
    return on;
}

// Highest GUI-validated code rate for a given coherent modulation — a per-modulation
// ceiling for the adaptive RateController, which is otherwise modulation-BLIND (it walks
// {R1/4..R3/4} at whatever modulation was fixed at CONNECT). Without this, a clean stretch
// promotes the connect-time 16QAM R1/2 up into the measured DAMAGE-BOUND 16QAM R2/3/R3/4
// (Phase 0a: 55-70% frame loss, 1-of-3 link-death — fable_analysis/07) BEFORE the reactive
// ssthresh can cap it, taking a frame into a fade at the over-climbed rung. QAM16 is capped
// at R2/3 (validated on the GUI gate 2026-06-14 with cross-frame interleave); the R3/4
// crest rung sits behind ULTRA_QAM16_R34 (A/B, default OFF). RAISE a cap per rung only as
// the GUI gate validates it. Non-QAM16 (QPSK) is capped at R3/4 — the top of the auto
// ladder since R5/6 was retired (2026-06-17, a measured-losing rung; see
// rate_controller.hpp). R5_6 is no longer a cap value.
inline CodeRate maxValidatedCoherentRate(Modulation mod) {
    switch (mod) {
        // 2026-06-14: lifted R1/2 -> R2/3. Cross-frame TIME interleave (then auto-on for QAM16 via
        // burstCrossFrameInterleaveOn) made 16QAM R2/3 Good@20 viable at ~1790 bps GUI-measured
        // (6/6 seeds), then ABOVE the R1/2 clean rung (~1550). NOTE (2026-07-21): interleave is now
        // DEFAULT-OFF (rig-measured harmful on MPG@20 fading — see burstCrossFrameInterleaveOn); the
        // R2/3 cap stands but its interleave-dependent margin is UNRE-MEASURED without it — re-A/B
        // 16QAM R2/3 vs R1/2 on the rig (per-frame SACK) before trusting R2/3 as the 16QAM cap. 2026-07-03: the R2/3 measured
        // ceiling is 3520 bps AWGN@20 with the wide window (the old "~2850 AWGN@30" figure is
        // stale); R3/4 raw = 9/8 of R2/3 -> projected ceiling ~3900. R3/4 measured damage-bound
        // PRE-interleave (55-70% frame loss — fable_analysis/07), so it ships knob-gated
        // (ULTRA_QAM16_R34): reachable ONLY via the adaptive walk, one-bad-group demote to R2/3.
        case Modulation::QAM16:
            return qam16R34Enabled() ? CodeRate::R3_4 : CodeRate::R2_3;
        // 8PSK revival (2026-07-05): R2/3 is the probe-validated ceiling (good@20
        // PASS 2040, 0 craters). R3/4 is AWGN-validated only (May, 2330 clean) —
        // its Good behavior (tight 45° boundaries on fading) is unmeasured on the
        // modern stack; raise only after a GUI-gate validation.
        case Modulation::QAM8:  return CodeRate::R2_3;
        default:                return CodeRate::R3_4;  // QPSK ceiling = top of the auto ladder
    }
}

inline float estimateWideOFDMRawBps(Modulation mod, CodeRate rate) {
    const auto* descriptor = ofdmCodeRateDescriptor(rate);
    if (descriptor == nullptr) {
        descriptor = &fallbackOFDMCodeRateDescriptor();
    }
    const int pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    const auto bits_per_symbol = static_cast<float>(
        ofdm_link_adaptation::bitsPerOFDMSymbol(
            static_cast<int>(kOFDMWideCarriers), true, pilot_spacing, mod));
    const float symbol_rate =
        static_cast<float>(kOFDMWideSampleRate) / static_cast<float>(kOFDMWideSymbolSamples);
    return bits_per_symbol * symbol_rate * descriptor->code_rate;
}

inline const OFDMCodeRateDescriptor* previousOFDMRateDescriptor(CodeRate rate) {
    const auto* current = ofdmCodeRateDescriptor(rate);
    if (current == nullptr) {
        return nullptr;
    }
    const OFDMCodeRateDescriptor* previous = nullptr;
    for (const auto& descriptor : kOFDMCodeRateDescriptors) {
        if (descriptor.code_rate < current->code_rate &&
            (previous == nullptr || descriptor.code_rate > previous->code_rate)) {
            previous = &descriptor;
        }
    }
    return previous;
}

inline const OFDMCodeRateDescriptor* nextOFDMRateDescriptorToward(CodeRate current_rate,
                                                                  CodeRate target_rate) {
    const auto* current = ofdmCodeRateDescriptor(current_rate);
    const auto* target = ofdmCodeRateDescriptor(target_rate);
    if (current == nullptr || target == nullptr || target->code_rate <= current->code_rate) {
        return current;
    }
    const OFDMCodeRateDescriptor* next = target;
    for (const auto& descriptor : kOFDMCodeRateDescriptors) {
        if (descriptor.code_rate > current->code_rate &&
            descriptor.code_rate < next->code_rate) {
            next = &descriptor;
        }
    }
    return next;
}

inline float ofdmCodeRateValue(CodeRate rate) {
    if (const auto* descriptor = ofdmCodeRateDescriptor(rate)) {
        return descriptor->code_rate;
    }
    return fallbackOFDMCodeRateDescriptor().code_rate;
}

// Shared helper: Select code rate for OFDM modes based on SNR and fading
// This is the SINGLE SOURCE OF TRUTH for rate selection thresholds.
// Both recommendWaveformAndRate() and recommendDataMode() use this.
//
// Fading index now combines freq_cv + temporal_cv (Doppler measurement).
// Thresholds (2026-05-21 floor recalibration sweep, PAPR-OFF baseline,
// in-band SNR convention; forced-waveform caveat below):
//   AWGN only (< 0.10):             R3/4 @ in-band SNR >= 25 (legacy)
//   Near-AWGN (< 0.15):             R2/3 @ in-band SNR >= 25 (legacy)
//   AWGN     (< 0.15):              R1/2 @ in-band SNR >= 12 (was 25)
//   Good     (< 0.65):              R1/2 @ in-band SNR >= 14 (was 25)
//   Moderate (< 1.10):              R1/2 @ in-band SNR >= 18 (was 25)
//   Heavy+   (>= 1.10):             R1/4 only
//
// 2026-05-21 floor recalibration measurements (PAPR-OFF, --waveform ofdm_chirp
// forced — see FLOOR_RECALIBRATION_2026_05_21.md). Each "reliable floor" is
// the lowest SNR where 3/3 seeds PASS with stable retx behavior:
//   OFDM R1/2 AWGN     reliable floor = 10 dB → gate ≥ 12 dB (+2 dB margin)
//   OFDM R1/2 Good     reliable floor = 12 dB → gate ≥ 14 dB (+2 dB margin)
//   OFDM R1/2 Moderate reliable floor = 16 dB → gate ≥ 18 dB (+2 dB margin)
//
// The +2 dB margin protects against seed variance and the
// IMD-vs-headroom asymmetry that PAPR ON introduces on simulator paths.
// Production hardware should see net +1.4 to +1.6 dB additional margin
// from PAPR-driven on-wire RMS gain (see PAPR_HARDWARE_VERIFICATION
// CHANGELOG entry).
//
// Caveat: today's floor measurements used `--waveform ofdm_chirp` which
// skips the MC-DPSK→OFDM handshake upgrade. Auto-negotiation
// re-verification is a deferred follow-up workstream
// (feedback_test_with_auto_negotiation.md). The relative ordering of
// R1/2 vs R1/4 should be unaffected, but absolute thresholds may shift
// ±1 dB.
//
// R3/4 verified (2026-02-10) — not re-measured:
//   DQPSK R3/4 AWGN SNR=20: 10/10 seeds PASS, 0 retransmissions
//   DQPSK R3/4 Good fading: FAILS (23 retx / 5 seeds) — AWGN only!
// R2/3 verified (2026-03-15) — not re-measured:
//   10KB Good fading SNR=15: 1485 bps, 33% retx
//   Demoted from Good: R1/2 gives similar throughput with half the retx.
// Rate for the coherent OFDM ladder at (snr, fading). SINGLE SOURCE OF TRUTH for
// rate selection — both recommendDataMode() and recommendWaveformAndRate() use it.
inline CodeRate selectOFDMCodeRate(float snr_db, float fading_index) {
    return selectCoherentOFDM(snr_db, fading_index).rate;
}

// ── Data-aided entry cap (2026-07-03): env-gated A/B knob ──────────────────────
// ULTRA_ENTRY_CAP_R34=1 lets the connect-time bootstrap cap admit a QPSK **R3/4 ENTRY**
// instead of the unconditional ULTRA_R23_BASIS R2/3 clamp (capInitialOFDMRate below).
// WHY: the R2/3 clamp's rationale was "chirp SNR can overestimate first OFDM frame
// quality" — but since #58 increment 2/3 the entry reading is the DATA-AIDED
// fade-AVERAGED MC-DPSK estimate (Connection::rateSelectionSnrDb/DataAided), a
// conservative LOWER-BOUND-leaning estimator (EVM only ADDS error), not the chirp
// snapshot. Rig evidence (12 connects at dial MPG@20): entries read 12.9-19.3
// data-aided, then every run spends ~60-90 s climbing R2/3→R3/4→16QAM through the
// EMA+streak arithmetic — the largest remaining goodput loss (~90-140 s below 16QAM
// per ~250 s transfer). Rate hop only — the modulation hop (→16QAM) stays
// adaptive-only. Default OFF → byte-identical. Read ONCE (static).
inline bool entryCapR34Enabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_ENTRY_CAP_R34");
        return e != nullptr && std::atoi(e) != 0;  // experimental; default OFF
    }();
    return on;
}

// Per-connect-reading dispersion (1 sigma, dB) of the connect-time SNR estimate.
// NOT a tuned constant — MEASURED: #58 rig campaign forensics, 12 connects at a
// constant dial (MPG@20) read 3.9-17.9 dB, sigma 3.15 (docs/CHANGELOG.md 2026-07-03
// "connect-SNR pool" entry; same population cited in connection_policy.hpp's
// ConnectSnrPool block). Used as the entry-cap margin: a single reading must clear a
// ladder threshold by >= 1 sigma before the entry pick may act on the excess.
inline constexpr float kConnectSnrReadingSigmaDb = 3.15f;

// Ladder-anchor lookup for one (mod, rate) rung at the given fading class. SINGLE
// source of truth = kCoherentLadder (the QPSK rows of the experimental QAM16 ladder
// are identical by construction, so this cannot drift between them). Returns
// kRungDisabledDb when the rung is not auto-selectable on that class.
inline float coherentLadderAnchorDb(Modulation mod, CodeRate rate, float fading_index) {
    const int cls = static_cast<int>(classifyFading(fading_index));
    for (const auto& rung : kCoherentLadder) {
        if (rung.mod == mod && rung.rate == rate) {
            return rung.min_snr_db[cls];
        }
    }
    return kRungDisabledDb;
}

// THE ULTRA_ENTRY_CAP_R34 gate (pure; knob state passed explicitly so boundary tests
// can exercise ON/OFF without racing the process-wide static env cache). R3/4 entry
// is allowed IFF ALL of:
//   1. the knob is ON;
//   2. the reading is DATA-AIDED (the fade-averaged estimator the argument rests on —
//      a training-snapshot reading keeps the R2/3 cap: it fade-crest OVER-reads);
//   3. fading is below Moderate-class (fading_index < kFadingGoodMax) — Moderate
//      entries ride the saturation bound and are deliberately marginal, so the R2/3
//      cap holds unconditionally there (the QPSK R3/4 Moderate rung is disabled
//      anyway: kRungDisabledDb);
//   4. the selection SNR clears the ladder's own QPSK R3/4 anchor for this fading
//      class (kCoherentLadder, e.g. Good 20.0) by >= kConnectSnrReadingSigmaDb —
//      i.e. R3/4 would still be the ladder pick even if this single reading
//      over-read by one measured per-reading sigma.
inline bool dataAidedEntryClearsR34(bool entry_cap_r34_on, float snr_db,
                                    float fading_index, bool data_aided) {
    if (!entry_cap_r34_on || !data_aided) {
        return false;
    }
    if (fading_index >= kFadingGoodMax) {
        return false;  // Moderate-class guard: keep the R2/3 cap unconditionally
    }
    const float r34_anchor_db =
        coherentLadderAnchorDb(Modulation::QPSK, CodeRate::R3_4, fading_index);
    return snr_db >= r34_anchor_db + kConnectSnrReadingSigmaDb;
}

// ULTRA_ENTRY_QAM16_SNR (default unset = OFF, byte-identical): EXPERIMENT — enter the
// coherent ladder directly AT 16QAM R2/3 when the data-aided fade-averaged connect reading
// clears the given threshold on a Good-class channel, INSTEAD of entering QPSK and climbing.
// Rationale (the "ride the fade cycle" strategy the commercial fade-riding modems use):
// start on the aggressive rung from frame 0 and let the closed-loop demote/re-climb machinery
// ride the troughs, rather than spending the calm windows crawling up the ladder. Entry-only:
// the adaptive ladder still demotes on a crater (receiver rung-command + sender escape) and
// re-climbs. Gates: promote only an AUTO QPSK entry (never a forced or MC-DPSK rung), Good-class
// fading only (fading_index < kFadingGoodMax — never Moderate/Poor, where 16QAM has no margin),
// and only the DATA-AIDED reading (the #58 fade-averaged estimate, not a raw chirp snapshot).
// NOTE the reading is EFFECTIVE (fade-compressed) SNR: at dial 20 Good it reads ~12 mean
// (CONNECT_ENTRY_CALIBRATION), so the threshold lives in READING-space, not dial-space —
// ~10-12 fires at dial 20; raise it to be selective (only the calmer snapshots start 16QAM).
inline bool entryQam16Promote(float snr_db, float fading_index, Modulation rec_mod,
                              bool data_aided) {
    const char* e = std::getenv("ULTRA_ENTRY_QAM16_SNR");
    if (!e || !e[0]) return false;                     // OFF -> byte-identical
    const float thresh = std::strtof(e, nullptr);
    if (thresh <= 0.0f) return false;
    if (rec_mod != Modulation::QPSK) return false;     // promote only an AUTO QPSK entry
    if (!data_aided) return false;                     // trust only the fade-averaged reading
    if (fading_index >= kFadingGoodMax) return false;  // Good-class only (never Moderate/Poor)
    return snr_db >= thresh;
}

// Initial OFDM rate at handshake bootstrap: start at the LADDER rate for the measured
// (snr, fading) — never above what the ladder supports, but no more conservative either.
// The per-class anchors (kCoherentLadder: AWGN measure_ack_fer 2026-06-06, GOOD measured
// 2026-06-02) ARE the measured connect-time reliability for that channel, so the
// connect-time SNR+fading already picks the right starting rung. (Removed the old "on
// fading, pin to R1/2 and climb" rule: with adaptive rate off it permanently froze
// fading channels at R1/2 even where the ladder has a measured R2/3+ — e.g. Good@15
// crawled at R1/2 instead of the measured R2/3.)
inline CodeRate capInitialOFDMRate(float snr_db, float fading_index, CodeRate candidate) {
    const CodeRate ladder_rate = selectOFDMCodeRate(snr_db, fading_index);
    return (ofdmCodeRateValue(candidate) < ofdmCodeRateValue(ladder_rate))
               ? candidate
               : ladder_rate;
}

// Implementation seam: entry_cap_r34_on is passed explicitly so the boundary tests can
// exercise the ULTRA_ENTRY_CAP_R34 ON path (the public wrapper below reads the knob).
inline CodeRate capInitialOFDMRateImpl(float snr_db,
                                       float fading_index,
                                       CodeRate candidate,
                                       Modulation modulation,
                                       bool data_aided,
                                       bool entry_cap_r34_on) {
    // ULTRA_FORCE_DATA_RATE: the operator is probing a specific rung — no demotion.
    if (forcedDataCodeRateFromEnvironment() != CodeRate::AUTO) {
        return candidate;
    }
    // ULTRA_R23_BASIS (2026-06-17, default ON; set =0 to disable): decouple the ENTRY rate
    // from the Good/Moderate classifier, which is unreliable on real hardware (it coin-flips
    // a genuinely-Good IONOS channel between Good and Moderate, dropping the entry from R3/4
    // to R1/2). On any FADING channel at usable SNR (>=18 dB) start every coherent-QPSK
    // connection at the robust R2/3 basis regardless of the (unreliable) Good-vs-Moderate
    // class, and let the closed-loop adaptive ladder climb to R3/4 on MEASURED headroom.
    //   GATED ON FADING PRESENT (fading_index >= kFadingAwgnMax): on a genuine AWGN channel
    //   there is no fade margin to recover and R3/4 is the measured-correct rung — a blanket
    //   R2/3 pin there is a pure ~11% throughput loss (OTASim AWGN@20: R3/4 2150 vs R2/3 1920
    //   bps). The fading_index cannot split Good from Moderate but it cleanly separates AWGN
    //   (~0) from any fading channel (~0.5), which is exactly the distinction this gate needs.
    // Entry-only: called only from the two initial-mode sites (connection.cpp acceptCall,
    // connection_handlers.cpp handleConnect); the rate_controller climb path is not gated,
    // so R2/3->R3/4 still works when adaptation is on.
    {
        const char* b = std::getenv("ULTRA_R23_BASIS");
        const bool r23_basis_on = !(b && b[0] == '0');  // default ON
        if (r23_basis_on && modulation == Modulation::QPSK && snr_db >= 18.0f &&
            fading_index >= kFadingAwgnMax) {
            // ULTRA_ENTRY_CAP_R34 exception (default OFF; gate documented at
            // dataAidedEntryClearsR34): a DATA-AIDED reading clearing the ladder's own
            // QPSK R3/4 anchor by >= 1 measured per-reading sigma, below Moderate-class
            // fading, may enter at R3/4. min(candidate, R3/4): the cap never RAISES the
            // candidate, and never admits anything above R3/4 — the 16QAM hop stays
            // adaptive-only (this branch is QPSK-gated and rate-only regardless).
            if (dataAidedEntryClearsR34(entry_cap_r34_on, snr_db, fading_index,
                                        data_aided)) {
                return (ofdmCodeRateValue(candidate) < ofdmCodeRateValue(CodeRate::R3_4))
                           ? candidate
                           : CodeRate::R3_4;
            }
            return CodeRate::R2_3;
        }
    }
    // Coherent-only band: modulation no longer changes the cap.
    return capInitialOFDMRate(snr_db, fading_index, candidate);
}

// data_aided: whether the (selection) snr_db rests on the data-aided fade-averaged
// MC-DPSK estimate — pass Connection::rateSelectionSnrDataAided() at the entry sites.
// Defaults to false so every unplumbed/legacy caller keeps the conservative cap.
inline CodeRate capInitialOFDMRate(float snr_db,
                                   float fading_index,
                                   CodeRate candidate,
                                   Modulation modulation,
                                   bool data_aided = false) {
    return capInitialOFDMRateImpl(snr_db, fading_index, candidate, modulation,
                                  data_aided, entryCapR34Enabled());
}

// Recommend waveform and rate based on SNR and fading index
//
// Fading index now combines freq_cv + temporal_cv (Doppler measurement).
// Raw-PHY estimates below use the strict definition
// `data_carriers × bits/sym × sym_rate × code_rate` against the
// production geometry (8-car MC-DPSK; 1024-FFT 59-car OFDM-CHIRP with
// CP=LONG, 1152 samples/symbol; pilots from
// `ofdm_link_adaptation::recommendedPilotSpacing()`).
//
// Calibrated reliability bands (2026-02-11):
// - In-band SNR < 10 dB: MC-DPSK is most robust (~375 bps raw at 8 car DQPSK R1/4)
// - In-band SNR >= 10 dB + AWGN: OFDM_CHIRP R1/4 (~1104 bps raw, warm LTS sync)
// - In-band SNR >= 12 dB + good fading: OFDM_CHIRP R1/4 with extra fading margin
// - In-band SNR >= 14 dB + moderate fading: OFDM_CHIRP R1/4 with extra fading margin
// - In-band SNR >= 18 dB + heavy fading: OFDM_CHIRP R1/4 with extra fading margin
// - In-band SNR >= 12 dB + AWGN: OFDM_CHIRP QPSK R2/3; >= 15 dB: R3/4 (measure_ack_fer
//   2026-06-06 data4_full floors R2/3 ~8 dB, R3/4 ~12 dB + margin; the OLD ">= 30 dB
//   for AWGN R3/4" was a stale, never-measured estimate — AWGN R3/4 closes far lower)
// - In-band SNR >= 30 dB + good fading: OFDM_CHIRP R2/3 (~2944 bps raw)
// - In-band SNR >= 25 dB + good/moderate fading: OFDM_CHIRP R1/2 (~2208 bps raw)
// - Heavy+ fading (>= 1.10): R1/4 only (~1104 bps raw)
//
// Calibrated fading thresholds (kFading*Max, single source above):
//   < 0.15: True AWGN, < 0.76: Good, >= 0.76: Moderate+
inline WaveformRecommendation recommendWaveformAndRate(float snr_db, float fading_index) {
    WaveformRecommendation rec;
    const float ofdm_floor =
        (fading_index < kFadingAwgnMax) ? ofdmEntryFloorAwgnDb() :
        (fading_index < kFadingGoodMax) ? kOFDMEntryFloorGoodDb :
        (fading_index < kFadingModerateMax) ? kOFDMEntryFloorModerateDb : kOFDMEntryFloorPoorDb;

    if (snr_db < ofdm_floor) {
        // Low SNR: MC-DPSK 8 carriers is most robust
        rec.waveform = WaveformMode::MC_DPSK;
        rec.rate = CodeRate::R1_4;
        rec.estimated_throughput_bps = 375.0f;
    }
    else if (fading_index < 0.15f) {
        // True AWGN (no fading)
        rec.waveform = WaveformMode::OFDM_CHIRP;
        rec.rate = selectOFDMCodeRate(snr_db, fading_index);
        rec.estimated_throughput_bps = estimateWideOFDMRawBps(Modulation::DQPSK, rec.rate);
    }
    else if (fading_index < 1.10f) {
        // Good-to-moderate fading: OFDM_CHIRP with extra floor margin vs AWGN.
        rec.waveform = WaveformMode::OFDM_CHIRP;
        rec.rate = selectOFDMCodeRate(snr_db, fading_index);
        rec.estimated_throughput_bps = estimateWideOFDMRawBps(Modulation::DQPSK, rec.rate);
    }
    else {
        // Heavy fading keeps extra margin but still uses the OFDM_CHIRP R1/4 floor.
        rec.waveform = WaveformMode::OFDM_CHIRP;
        rec.rate = selectOFDMCodeRate(snr_db, fading_index);
        rec.estimated_throughput_bps = estimateWideOFDMRawBps(Modulation::DQPSK, rec.rate);
    }

    return rec;
}

// Recommend modulation and code rate for data mode within an established connection
// This is used after waveform negotiation to set the data transmission parameters.
// Uses selectOFDMCodeRate() for rate selection to stay consistent with recommendWaveformAndRate().
//
// For OFDM modes: measured coherent rungs first, then DQPSK with rate from
// selectOFDMCodeRate().
// QPSK is the Good-fading workhorse rung at the measured Good@20 R2/3 floor.
// Admit one protocol SNR quantum below the nominal 20 dB floor: CONNECT and
// MODE_CHANGE carry SNR in 0.25 dB steps, and Phase-1 GUI seeds 4/6 landed
// at the 19.8 dB gray band where DQPSK R1/2 was selected and failed.
// D8PSK/QAM16 are clean-channel rungs; too sensitive for Good-fading nulls.
// For MC-DPSK: Always DQPSK R1/4
//
inline void recommendDataMode(float snr_db, WaveformMode waveform,
                               Modulation& mod, CodeRate& rate, float fading_index = 0.0f) {
    // 2026-05-28 test-only override: ULTRA_FORCE_DATA_MOD / ULTRA_FORCE_DATA_RATE
    // bypass the corresponding automatic choice so we can probe a specific rung on a
    // live channel.  A complete pair short-circuits the selector; a one-field force
    // resolves the unforced half normally instead of returning it uninitialized.
    const EnvironmentForcedDataProfile environment_force =
        forcedDataProfileFromEnvironment();
    const Modulation forced_mod = environment_force.modulation;
    const CodeRate forced_rate = environment_force.code_rate;
    const auto apply_environment_force = [&] {
        if (forced_mod != Modulation::AUTO) {
            mod = forced_mod;
        }
        if (forced_rate != CodeRate::AUTO) {
            rate = forced_rate;
        }
    };
    if (forced_mod != Modulation::AUTO && forced_rate != CodeRate::AUTO) {
        apply_environment_force();
        return;
    }

    // MC-DPSK always uses DQPSK R1/4 for robustness
    if (waveform == WaveformMode::MC_DPSK) {
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_4;
        apply_environment_force();
        return;
    }

    // OFDM_NARROW: DQPSK only, R1/4 default, R1/2 for clean-enough
    // conditions. Sweeps in cli_simulator (2026-05-03) verified R1/2
    // narrow + window=3 passes 7-message test at:
    //   legacy SNR=8  AWGN          (PASS)
    //   legacy SNR=10 good fading   (PASS)
    //   legacy SNR=12 good fading   (PASS)
    //   legacy SNR=15 good fading   (PASS)
    // and R1/4 narrow + window=3 passes:
    //   SNR=8  good fading   (PASS — documented baseline)
    //   SNR=8  moderate      (PASS, slow but recovers)
    // The hard floor (SNR=8 good fading R1/4) is preserved by the
    // in-band SNR>=20 gate on good fading. AWGN keeps the in-band SNR>=18 trigger
    // because near-AWGN is a much easier channel.
    if (waveform == WaveformMode::OFDM_NARROW) {
        mod = Modulation::DQPSK;
        const bool awgn_path = fading_index < kFadingAwgnMax && snr_db >= 18.0f;
        const bool good_path = fading_index < kFadingGoodMax && snr_db >= 20.0f;
        if (awgn_path || good_path) {
            rate = CodeRate::R1_2;
        } else {
            rate = CodeRate::R1_4;
        }
        apply_environment_force();
        return;
    }

    // OFDM modes: measured coherent rungs gated on conditions, otherwise DQPSK.
    //
    // D8PSK ladder (re-enabled 2026-05-04 after wide cli_simulator
    // sweeps showed it works in fading with the post-2026-03-15 CPE
    // correction + per-symbol pilot tracking already in the demod):
    //   sweep results for D8PSK on good fading (cli_simulator 7-msg):
    //     R1/2 SNR=8:   FAIL (cliff)
    //     R1/2 SNR=10:  PASS, 4 retx
    //     R1/2 SNR=12:  PASS, 2 retx
    //     R1/2 SNR=15:  PASS, 0 retx
    //     R2/3 SNR=10:  PASS, 28 retx (high)
    //     R2/3 SNR=12:  PASS, 45 retx (very high)
    //     R2/3 SNR=15:  PASS, 0 retx
    //     R2/3 SNR=20:  PASS, 1 retx
    //     R3/4 SNR=20:  PASS, 6 retx (border, AWGN-only)
    //   Moderate fading: R1/2 SNR>=15 also stable (3-6 retx).
    //
    // The throughput case: D8PSK R2/3 at SNR=15 good fading carries
    // 1.5× the bits/symbol of DQPSK R2/3 at the same conditions, so
    // the throughput jumps from ~3.4 kbps to ~5 kbps with zero retx.
    //
    // OFDM band is COHERENT-ONLY (differential DQPSK/D8PSK retired to MC-DPSK; Poor
    // routes to MC-DPSK upstream via kOFDMEntryFloorPoorDb, so OFDM never reaches
    // this with Poor fading). The single coherent ladder picks BOTH modulation and
    // rate from the measured per-fading-class anchors (kCoherentLadder) — no more
    // shouldSelectQPSK/QAM16 branches. See docs/RATE_LADDER_ANCHORS.md.
    const CoherentPick pick = selectCoherentOFDM(snr_db, fading_index);
    mod = pick.mod;
    rate = pick.rate;
    apply_environment_force();
}

// Conservative raw-PHY bitrate estimate per waveform mode. Used by
// TNC STATS and GUI wire-time estimators that need a single bps
// number without knowing the active rate/modulation pair. The numbers
// are the documented "raw PHY (theoretical maximum)" from
// README.md for the production geometry — they are deliberately
// pessimistic vs the per-rate ladder, so use selectOFDMCodeRate()
// when the caller wants per-rate precision.
inline int estimatedBitrateBpsForMode(WaveformMode mode) {
    switch (mode) {
    case WaveformMode::OFDM_NARROW:
        return 386;   // DQPSK R1/2, 18 data carriers @ 21.429 sym/s
    case WaveformMode::MC_DPSK:
        return 375;   // 8 carriers DQPSK R1/4 @ 93.75 sym/s
    case WaveformMode::OFDM_CHIRP:
        return 2208;  // DQPSK R1/2, 53 data carriers @ 41.667 sym/s
    case WaveformMode::OTFS_EQ:
    case WaveformMode::OTFS_RAW:
    case WaveformMode::MFSK:
    case WaveformMode::AUTO:
        return 0;
    }
    return 0;
}

} // namespace protocol
} // namespace ultra
