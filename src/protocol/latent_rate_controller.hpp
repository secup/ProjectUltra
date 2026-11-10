#pragma once
// ============================================================================
// LATENT-STATE RATE CONTROLLER  (ULTRA_LATENT_RATE, DEFAULT-ON since 2026-08-01)
// ============================================================================
//
// WHY THIS SHAPE — what every deployed rate controller does and we did not
// -----------------------------------------------------------------------
// Minstrel-HT (Linux mac80211 default), iwlwifi, LTE/NR OLLA and STANAG 5066's DRC all
// share one property we lacked:
//
//     THEY MAINTAIN A VALUE ESTIMATE FOR EVERY RATE AT ALL TIMES, AND THE DECISION IS A
//     STATELESS argmax OVER THAT VECTOR. Reliability enters as a clamped probability
//     INSIDE the value; it is never a penalty applied to the incumbent.
//
// Minstrel contains no dwell, no hysteresis, no ratchet, no penalty state and no
// crater rule. All of its stability lives in the ESTIMATOR; none in the ACTUATOR. Our
// RX-authority path is the exact inverse: a correct selector followed by ~10 actuator-side
// correctives which, measured over 135 rig verdicts, clamp 46% of decisions down a mean of
// 1.61 rungs (the ladder picks 8PSK R2/3 66% of the time; 26% survives to the wire).
//
// The predecessor, goodput_rate_controller.hpp, had the right OBJECTIVE and the wrong
// SHAPE: one ring buffer for the CURRENT rung, zeroed on every rung change, then 4-8 groups
// of "window-filling" before it may act. With 5-8 rung changes per transfer it is inert for
// most of a run — argmax over one entry is a thermostat. That, not statistics, is the most
// likely reason it measured a wash.
//
// WHY NOT JUST COPY MINSTREL
// --------------------------
// Minstrel's per-rate table is fed by probing 5-10% of packets, and its probe is nearly free
// (it rides a retry slot inside a TXOP that was going to be spent anyway). Neither holds here:
//   - Our feedback is ONE group ACK per ~9.5 s. A 10% probe budget is one probe per 95 s =
//     one refresh per 22 coherence times. Stale before use.
//   - A probe costs a WHOLE GROUP (~9.5 s of airtime), because we are half-duplex and
//     rung-committed for the burst.
//   - Frames inside a burst are CORRELATED: burst 8-10 s against Tc = 4.23 s gives
//     n_eff ~ 2.2 independent frames per group, not 5. A usable per-rung estimate then needs
//     ~20 groups ~ 190 s of EXCLUSIVE use per rung; ten rungs is 30-50 minutes. A transfer
//     is ~70 s. Per-rung independent statistics are structurally impossible here.
//
// So we take the cellular/HF route instead: populate all rungs from a SHARED low-dimensional
// model and spend ZERO dedicated probe groups.
//
// THE MODEL
// ---------
// One latent scalar x — the link's effective operating point in dB — with a per-rung
// threshold theta_r and a shared logistic slope:
//
//     P(frame decodes on rung r | x) = logistic(slope * (x - theta_r))
//
// Every group's SACK updates the posterior over x, and therefore updates the predicted
// success probability of ALL TEN RUNGS SIMULTANEOUSLY. Evidence SURVIVES a rung change:
// re-convergence after a switch is zero groups, against 4-8 groups of forced inertia before.
//
// THE PROPERTY THAT MATTERS MOST HERE
// -----------------------------------
// Because x is fitted from OUTCOMES rather than read from an SNR estimator, adding a
// constant to every theta_r is absorbed by x. COMMON-MODE CALIBRATION ERROR CANCELS EXACTLY.
// That retires, for rate selection, the entire class of defect that has cost this project
// the most: kOfdmLegacyAnchorScaleOffsetDb (+8.70 dB, itself a compatibility shim for an
// estimator bug that was already fixed), the guard-bin noise under-read, and the ~5.6 dB
// IONOS implementation loss. None of them can move the decision. Only the DIFFERENTIAL
// spacing theta_r - theta_s matters — which is precisely the quantity
// docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md measured directly.
//
// This controller therefore consumes NO SNR ESTIMATE AT ALL. It is the reason the offset can
// leave the rate path.
//
// SCOPE. Pure logic, no I/O, no clock, no RNG.

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <cstdlib>

#include "ultra/fast_profile.hpp"
#include "waveform_selection.hpp"

namespace ultra {
namespace protocol {

// DEFAULT-ON since 2026-08-01. Opt out with ULTRA_LATENT_RATE=0, which restores the legacy
// SNR-anchor ladder and its corrective stack.
//
// EVIDENCE FOR THE FLIP:
//   - Rig A/B, 8 interleaved pairs at IONOS MPG@20: +14.5% mean, median +18.4%, 7/8
//     positive, 1.64 vs 1.45 kbps. Paired t = 2.92, df=7 -> p = 0.022. The ONLY result in
//     this campaign to clear p<0.05 (the others: +8.0% goodput ctl, -13.8% dwell, +20.7%
//     trust-pick, +26.2% bundle, +5.2% forced rung — none significant).
//   - Faithful gate PASSES on BOTH classes: Good@20 and Moderate@20, CRC clean.
//   - Moderate is the one that mattered, because theta_r was measured on ITU Good ONLY. It
//     works anyway: on Moderate the posterior settled to x ~ 11-12 against 15-17 on Good and
//     held QPSK R1/2-R2/3 instead of 8PSK. A worse channel simply produces a lower x, so the
//     argmax lands correctly without a per-class table. That is the common-mode absorption
//     doing real work, not merely cancelling a calibration constant.
//
// HONEST CAVEATS carried into the release:
//   - Significant by the t-test, NOT by the sign test (7/8, p=0.070). Established, not
//     settled.
//   - The tie-break probe (kTieBreakMarginFrac / kTieBreakPeriod) raised the mean slightly
//     but DOUBLED the variance (+16.5% sd 27.3% over 7 pairs, losing significance). It is
//     therefore an explicit experiment, disabled by the production caller unless requested.
inline bool latentRateControllerEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_LATENT_RATE");
        return !(e != nullptr && e[0] == '0' && e[1] == '\0');  // DEFAULT-ON
    }();
    return on;
}

// Dedicated whole-group probes doubled the paired-run variance and erased the otherwise
// significant controller result. Keep the mechanism available for controlled experiments,
// but never spend an on-air group on it by default.
inline bool latentTieBreakProbeEnabled() {
    const char* e = std::getenv("ULTRA_LATENT_TIE_PROBE");
    return e != nullptr && e[0] == '1' && e[1] == '\0';
}

// Absolute canonical-rung ceiling promised by ULTRA_MAX_OFDM_RATE.  The legacy
// within-modulation controller parsed this at its actuator, but the latent controller
// returns before that path; passing this value into best() keeps its normal argmax, the
// experimental tie probe, and startup exploration under the same operator ceiling.
inline uint8_t latentConfiguredRungCeiling() {
    const char* env = std::getenv("ULTRA_MAX_OFDM_RATE");
    if (env == nullptr) return kRungIdxNone;
    if (std::strcmp(env, "R1_2") == 0 || std::strcmp(env, "r1_2") == 0) {
        return kRungIdxQpskR12;
    }
    if (std::strcmp(env, "R2_3") == 0 || std::strcmp(env, "r2_3") == 0) {
        return kRungIdxQpskR23;
    }
    if (std::strcmp(env, "R3_4") == 0 || std::strcmp(env, "r3_4") == 0) {
        return kRungIdxQpskR34;
    }
    return kRungIdxNone;
}

// Environment-forced profiles are resolved into the outbound CONNECT forced fields so both
// endpoints share the same rung.  They also form an operator measurement contract: the
// negotiated rung remains pinned for the whole QSO, including if the process environment is
// changed after CONNECT. ULTRA_LOCK_RATE is repeated here for the same explicit contract;
// Connection::rateAdaptationActive() supplies the instance-level and ULTRA_RATE_ADAPT checks
// at the call site.
inline bool latentStartupProbeAllowedByOperator() {
    // Merely exporting an empty or misspelled variable does not force the selector,
    // so it must not silently freeze the closed-loop controller either.
    if (environmentForcesDataProfile()) return false;
    if (const char* lock = std::getenv("ULTRA_LOCK_RATE")) {
        if (std::atoi(lock) != 0) return false;
    }
    return true;
}

// ── LINK MODEL ───────────────────────────────────────────────────────────────
//
// theta_r = the operating point at which rung r decodes half its frames, in the SAME
// arbitrary units as the latent x. The absolute scale is IRRELEVANT (it cancels); only the
// spacing carries information.
//
// RE-MEASURED 2026-11-08 FOR EVERY RUNG on the current PHY (measured-noise LLRs default ON;
// the CFO chain made correct for a real dial offset) at PRODUCTION geometry — measure_ack_fer
// --geometry production (QPSK cw8 / 8PSK cw12 / 16QAM cw16, MEDIUM CP) --preroll-ms 300
// --cfo-hz 7.3, ITU-R F.1487 Good, 6 seeds x 24 frames x 9 SNR points (2 dB) per rung;
// theta = the logit-interpolated 50 %-FER point (= the 20-70 % band logistic fit to 0.1-0.3 dB):
//
//     rung          theta  slope   (2026-07-26, 4-CW / LONG-CP harness)
//     QPSK R1/4      4.9   0.39    -2.0 derived
//     QPSK R1/2      7.3   0.49     3.0 derived
//     QPSK R2/3     10.3   0.42     8.0 derived
//     QPSK R3/4     12.3   0.37    13.0
//     8PSK R2/3     14.7   0.41    15.9
//     8PSK R3/4     17.1   0.39    20.5
//     16QAM R1/2    13.7   0.45    17.0 derived
//     16QAM R2/3    17.8   0.40    20.3
//     16QAM R3/4    22.0   0.13    26.0 derived   (20 % error floor: never usable on Good)
//
// TURBO RUNGS (2026-11-10). With the one-codeword turbo data frame (the ULTRA_FAST default) the
// 16QAM R3/4 floor is gone and 32QAM R2/3 decodes on Good. Their thetas are the 16QAM R2/3 theta
// plus the spacing measured on the turbo PHY in one sweep (measure_ack_fer, production geometry,
// ITU Good, 3 seeds x 40 frames x 2 dB; 50 %-FER points 16QAM R2/3 14.3, 16QAM R3/4 16.0,
// 32QAM R2/3 18.8 dB): +1.7 and +4.5 dB. The spacing, not the absolute point: this table's
// rungs were fitted on LDPC frames, and the latent x absorbs a common offset, not a spacing.
//     16QAM R3/4    19.5   (17.8 + 1.7)
//     32QAM R2/3    22.3   (17.8 + 4.5)
//
// The derived QPSK spacing (5 dB per code rate) was the large error: on fading the waterfall is
// shaped by fades as much as by coding, so adjacent code rates sit 2-3 dB apart. That error is
// also why the controller stalled on QPSK R2/3 at Good@20: R2/3 successes saturate the
// likelihood at theta + ln(0.965/0.035)/slope (old 17.5), at the old 8PSK crossover (17.0).
// The dense rungs gained 2.5-3.4 dB from the measured-noise LLRs (no more clipped LLRs).
//
// VALIDATION. With production geometry the argmax of this table switches QPSK R1/2 -> R2/3 at
// 11.5, -> 8PSK R2/3 at 16.0, -> 16QAM R2/3 at 19.5 dB; the MEASURED delivered-goodput argmax
// of the same sweep switches at ~11.9 / ~15.8 / ~19.8 dB (the old table: 10.0 / 17.0 / 22.5).
// QPSK R3/4 is never the argmax — and never the measured best either (R2/3 below 15 dB, 8PSK
// R2/3 above), so that is physics at production geometry, not a model defect.
//
// CHANNEL CLASSES. One latent x absorbs a COMMON offset, not a class-dependent SPACING. Relative
// to QPSK R1/2 the dense rungs sit further out on Moderate (8PSK R2/3 +9.5 vs +7.4 dB on Good,
// 16QAM R2/3 +14.0 vs +10.5) and carry 17-31 % error floors there (coherent tracking at 0.5 Hz
// Doppler, BUG-MODERATE-8PSK-COHERENT-FLOOR); AWGN compresses it. Offline controller Monte-Carlo
// with outcomes drawn from the measured per-rung success (60 transfers x 20 groups per cell),
// fraction of the best fixed rung, old -> new table: Good 18 dB 87 -> 98 %, 24 dB 94 -> 100 %;
// AWGN 12 dB 65 -> 74 %, 16 dB 55 -> 85 %; Moderate 15-24 dB 96-100 % -> 97-99 % (more rung
// switches there, which the rung-change anchor cost in the production geometry damps).
//
// SHIPPING STATE: DEFAULT via ULTRA_FAST since 2026-11-09 (ULTRA_FAST=0 or
// ULTRA_LATENT_ANCHORS=july restores the 2026-07 set; both stations). The measured set — this
// table, slope 0.41 and the anchor-consistent prior seed below — is one coherent calibration and
// switches as a unit. GUI gate (21 KB, 7.3 Hz CFO, 5 seeds per cell, all PASS) against the
// 2026-07 set: Good@20 +26.7 % (5/5 pairs up, paired t 3.7), AWGN@20 +54.7 %, Good@15 +8 % (n.s.),
// Moderate@20 -5.8 % (2/5 up, n.s.) with 5 craters vs 0 — the class-dependent spacing above:
// on Moderate the Good thetas over-state 8PSK R2/3, the posterior settles where it still looks
// as good as QPSK R3/4, and the GUI's failure cost (resends, anchors, craters) exceeds the frame
// model's. The table without the seed regressed every cell (-2 .. -9 %). Inside the ULTRA_FAST
// stack (structure acquisition + turbo + EVM climb) the Moderate deficit is gone (Moderate@20
// 2114 vs 1668 bps); a rig A/B of ULTRA_FAST vs ULTRA_FAST=0 is still pending (rate-controller
// A/Bs on the seeded simulator have reversed on the rig before).
inline bool latentMeasuredAnchorsEnabled();

inline constexpr float kLatentSlopePerDbJuly2026 = 0.35f;  // 2026-07 set: per-rung fits 0.15-0.45
inline constexpr float kLatentSlopePerDbMeasured = 0.41f;  // median of the 8 usable 2026-11 fits
                                                            // (0.37-0.49; 16QAM R3/4's 0.13 = floor)
inline float latentSlopePerDb() {
    return latentMeasuredAnchorsEnabled() ? kLatentSlopePerDbMeasured : kLatentSlopePerDbJuly2026;
}

inline float latentThetaForRung(uint8_t idx) {
    if (latentMeasuredAnchorsEnabled()) {
        switch (idx) {  // 2026-11-08, measured for every rung (table above)
            case kRungIdxQpskR14: return  4.9f;
            case kRungIdxQpskR12: return  7.3f;
            case kRungIdxQpskR23: return 10.3f;
            case kRungIdxQpskR34: return 12.3f;
            case kRungIdxQam8R23: return 14.7f;
            case kRungIdxQam8R34: return 17.1f;
            case kRungIdxQam16R12: return 13.7f;
            case kRungIdxQam16R23: return 17.8f;
            case kRungIdxQam16R34: return 19.5f;  // turbo: 16QAM R2/3 + 1.7 (was 22.0, LDPC floor)
            case kRungIdxQam32R23: return 22.3f;  // turbo: 16QAM R2/3 + 4.5
            default: return 99.0f;
        }
    }
    switch (idx) {  // 2026-07-26 set (default): 4 measured on the 4-CW harness, the rest derived
        case kRungIdxQpskR14: return -2.0f;   // derived: R1/2 - 5 dB (ladder spacing)
        case kRungIdxQpskR12: return  3.0f;   // derived: R3/4 - 10 dB
        case kRungIdxQpskR23: return  8.0f;   // derived: R3/4 - 5 dB
        case kRungIdxQpskR34: return 13.0f;   // measured
        case kRungIdxQam8R23: return 15.9f;   // measured
        case kRungIdxQam8R34: return 20.5f;   // measured
        case kRungIdxQam16R12: return 17.0f;  // derived: between 8PSK R2/3 and 8PSK R3/4
        case kRungIdxQam16R23: return 20.3f;  // measured
        case kRungIdxQam16R34: return 22.0f;  // derived: 16QAM R2/3 + the turbo spacing 1.7
        case kRungIdxQam32R23: return 24.8f;  // derived: 16QAM R2/3 + the turbo spacing 4.5
        default: return 99.0f;
    }
}

// PRIOR SEED. The connect-time automatic pick is a rung the ladder expects to WORK: its anchors
// sit at FER <= ~10 % (QPSK: floor + margin) or at the throughput crossover (8PSK Good). With
// theta measured as the 50 %-FER point, seeding x at theta(pick) — the pre-2026-11-08 rule,
// written when theta were relative ladder positions — starts the posterior ~5 dB below what the
// pick asserts, and the p25 decision a further ~2 dB below that: at Good@20 the controller then
// needs a whole 21 KB transfer to leave QPSK R2/3, and the slower exit from R1/2 arms the
// one-shot startup probe, whose failure on a fading channel (any lost frame) pins R1/2 for the
// QSO (GUI gate Moderate@20 s43: 15 of 16 groups at R1/2, 1280 vs 1690 bps). The consistent seed
// is the x at which the pick succeeds with the anchor's probability:
//     x0 = theta(pick) + logit(p) / slope.
// With the measured set the seed uses p = 0.9 (x0 = theta + 5.4 dB); the 2026-07 set keeps
// x0 = theta(pick), the rule its relative thetas were tuned with.
inline constexpr float kLatentSeedSuccessMeasured = 0.9f;

inline float latentSeedXForRung(uint8_t idx) {
    if (!latentMeasuredAnchorsEnabled()) return latentThetaForRung(idx);
    const float p = kLatentSeedSuccessMeasured;
    return latentThetaForRung(idx) + std::log(p / (1.0f - p)) / latentSlopePerDb();
}

// ULTRA_LATENT_ANCHORS=measured (or 1) selects the 2026-11-08 measured set; read once, with a
// test override. A WIRE-INVISIBLE but behaviour-defining choice: set it on both stations (the
// receiver holds rate authority).
inline std::atomic<int>& latentMeasuredAnchorsFlag() {
    static std::atomic<int> flag{-1};
    return flag;
}
inline bool latentMeasuredAnchorsEnabled() {
    int v = latentMeasuredAnchorsFlag().load(std::memory_order_relaxed);
    if (v < 0) {
        // Explicit ULTRA_LATENT_ANCHORS wins; unset, the ULTRA_FAST profile selects measured.
        const char* e = std::getenv("ULTRA_LATENT_ANCHORS");
        if (e != nullptr) {
            v = (std::strcmp(e, "measured") == 0 || std::strcmp(e, "1") == 0) ? 1 : 0;
        } else {
            v = fastProfileEnabled() ? 1 : 0;
        }
        latentMeasuredAnchorsFlag().store(v, std::memory_order_relaxed);
    }
    return v == 1;
}
inline void setLatentMeasuredAnchorsForTesting(bool on) {
    latentMeasuredAnchorsFlag().store(on ? 1 : 0, std::memory_order_relaxed);
}

// FADING ERROR FLOOR. On a Rayleigh channel no rung reaches P=1: deep nulls destroy frames
// regardless of margin (CLAUDE.md: "fading loss is irreducible"). The measured waterfall
// bottoms at 3.5% FER, so the model must cap success at 0.965 — a logistic that approaches
// 1.0 would let the posterior conclude "this rung never fails" from a clean run and then be
// unable to explain the first crater except by moving x a long way.
inline constexpr float kLatentMaxSuccess = 0.965f;
inline constexpr float kLatentMinSuccess = 0.035f;

// ── POSTERIOR ────────────────────────────────────────────────────────────────
class LatentRateController {
public:
    static constexpr int kBins = 61;          // 0.5 dB resolution over [-5, +25]
    static constexpr float kBinLo = -5.0f;
    static constexpr float kBinStep = 0.5f;

    // Minstrel's two decision constants, used verbatim and for its reasons:
    //   below kDead the rate is treated as having ZERO value, not merely low value —
    //     otherwise a 5%-success rung with a huge raw rate can win the argmax;
    //   above kCap the probability is clamped so a lucky clean streak cannot make a
    //     too-high rung look better than a rung that is genuinely reliable.
    static constexpr float kDeadProb = 0.10f;
    static constexpr float kCapProb = 0.90f;

    // TIE-BREAK UPWARD — the counterweight the pessimism above requires.
    //
    // Deciding from the 25th percentile is right (over-committing costs a cratered group
    // ~9.5 s; under-committing costs one group's rate delta) but it is NOT free: on the rig
    // it cost 1.2 dB (posterior mean 16.2 against x_p25 15.0), which was exactly the gap to
    // the QPSK R2/3 -> 8PSK R2/3 crossover at ~15.5. The controller therefore sat one rung
    // low for whole transfers and still beat the ladder by 14.5% on stability alone
    // (8PSK changes 0.4/run against baseline's 1.2 — it WON while using the good rung LESS).
    //
    // The second reason it sticks is structural: the likelihood SATURATES. QPSK R2/3
    // (theta 8.0) caps at p = kLatentMaxSuccess once x >= 17.5, after which a clean 5/5 group
    // carries no information at all — a rung that always succeeds tells you only "x is above
    // my threshold", never how far above. No amount of further evidence lifts x past that.
    //
    // So when the higher rung is predicted within kTieBreakMarginFrac of the best, take it
    // one decision in kTieBreakPeriod. This is Minstrel's INC bucket, and its expected cost
    // is bounded BY CONSTRUCTION: at most margin/period = 15%/4 = 3.75% of one group's
    // goodput, against unlocking a rung worth ~+20%. Crucially the probe is not wasted even
    // when it loses — the outcome updates the posterior, which is the only way to learn
    // about the saturated region at all.
    static constexpr float kTieBreakMarginFrac = 0.15f;
    static constexpr int kTieBreakPeriod = 4;

    // Pessimism. LTS uses min{sampled theta, posterior mean}; the asymmetry is real here too
    // — over-committing costs a cratered group (~9.5 s) while under-committing costs the
    // rate delta on one group. Decide from the 25th percentile of the posterior, not its
    // mean.
    static constexpr float kDecilePessimism = 0.25f;

    LatentRateController() { reset(); }

    void reset() {
        for (int i = 0; i < kBins; ++i) logp_[i] = 0.0f;
        normalise();
        have_prior_ = false;
        observations_ = 0;
        decisions_ = 0;
    }

    // Seed from the connect-time pick. Minstrel starts from ignorance because at 2000 pkt/s
    // it can afford to; we get one observation per 9.5 s and cannot. sigma is the measured
    // connect-snapshot spread (+-2-3 dB, a single fade snapshot).
    void seedPrior(float x0, float sigma_db = 3.0f) {
        const float s2 = std::max(0.25f, sigma_db * sigma_db);
        for (int i = 0; i < kBins; ++i) {
            const float x = binX(i);
            logp_[i] = -0.5f * (x - x0) * (x - x0) / s2;
        }
        normalise();
        have_prior_ = true;
    }

    // One group outcome: k of M frames delivered on rung r.
    //
    // TEMPERING is not optional. Frames inside a burst are correlated (burst 8-10 s vs
    // Tc 4.23 s), so a 5-frame group carries ~2.2 independent bits, not 5. Feeding
    // Binomial(5,p) would over-sharpen the posterior by ~2.3x per group and the controller
    // would converge to a confident wrong answer and then refuse to move.
    void observe(uint8_t rung, int k, int M, float n_eff_per_group = 2.2f) {
        if (M <= 0 || rung >= kRungIdxCount) return;
        k = std::clamp(k, 0, M);
        const float theta = latentThetaForRung(rung);
        const float w = std::clamp(n_eff_per_group / static_cast<float>(M), 0.05f, 1.0f);
        for (int i = 0; i < kBins; ++i) {
            const float p = successProb(binX(i), theta);
            logp_[i] += w * (static_cast<float>(k) * std::log(p) +
                             static_cast<float>(M - k) * std::log(1.0f - p));
        }
        normalise();
        ++observations_;
    }

    // Widen the posterior once per group: the ionosphere moves and old evidence must decay.
    // THIS IS THE ONE FREE PARAMETER AND IT IS GENUINELY UNKNOWN.
    //   too small -> the posterior collapses and the controller freezes on a rung that
    //     stopped being right. It will look EXCELLENT on OTASim, whose Watterson process is
    //     exactly stationary and models no ionospheric time-variation, and fail on the rig.
    //     That is the simulator-fidelity trap CLAUDE.md warns about, in its purest form.
    //   too large -> the posterior never sharpens, argmax returns the prior, and this whole
    //     controller reduces to the anchor table we already have.
    // Must be MEASURED from our own data, not assumed.
    void relax(float sigma_db) {
        if (!(sigma_db > 0.0f)) return;
        const float s = sigma_db / kBinStep;
        if (s < 0.05f) return;
        float p[kBins], out[kBins];
        float mx = logp_[0];
        for (int i = 1; i < kBins; ++i) mx = std::max(mx, logp_[i]);
        for (int i = 0; i < kBins; ++i) p[i] = std::exp(logp_[i] - mx);
        const int half = std::min(kBins - 1, static_cast<int>(std::ceil(3.0f * s)));
        for (int i = 0; i < kBins; ++i) {
            float acc = 0.0f, wsum = 0.0f;
            for (int d = -half; d <= half; ++d) {
                const int j = i + d;
                if (j < 0 || j >= kBins) continue;
                const float wk = std::exp(-0.5f * static_cast<float>(d) * d / (s * s));
                acc += wk * p[j];
                wsum += wk;
            }
            out[i] = (wsum > 0.0f) ? acc / wsum : p[i];
        }
        for (int i = 0; i < kBins; ++i) logp_[i] = std::log(std::max(out[i], 1e-30f));
        normalise();
    }

    // ── DECISION: stateless argmax over predicted delivered goodput ───────────
    //
    // The caller supplies physical geometry from the same helpers used by the transmitter.
    // This is deliberately bytes/cycle, not a spectral-efficiency proxy: production varies
    // codeword count by modulation and normalises most active rungs to approximately the
    // same frame duration. Multiplying by eta while also shrinking airtime by 1/eta rewards
    // a dense rung twice and was measured to change the selected rung.
    struct RungGeometry {
        float useful_bytes_per_frame = 0.0f;
        int frames_per_cycle = 0;
        float cycle_s = 0.0f;
        // Airtime of ONE data frame (s): a round carrying m < frames_per_cycle frames lasts
        // cycle_s - (frames_per_cycle - m) * frame_airtime_s. 0 = unknown (a partial round is
        // then priced as a full cycle). Used by the effective-SNR selector's time-to-finish.
        float frame_airtime_s = 0.0f;

        bool valid() const {
            return useful_bytes_per_frame > 0.0f &&
                   frames_per_cycle > 0 && cycle_s > 0.0f;
        }
    };
    using RungGeometryTable = std::array<RungGeometry, kRungIdxCount>;

    struct Pick {
        uint8_t rung = kRungIdxNone;
        float goodput = 0.0f;       // predicted useful bytes/s
        float x_used = 0.0f;
        bool tie_break_probe = false;   // this decision took the higher near-tied rung
    };

    Pick best(const RungGeometryTable& geometry,
              uint8_t ceiling = kRungIdxNone,
              bool allow_tie_break_probe = false) {
        Pick out;
        float g_by_rung[kRungIdxCount] = {};
        out.x_used = percentile(kDecilePessimism);
        // Most robust candidate (highest predicted success), for when every rung is dead.
        uint8_t robust = kRungIdxNone;
        float robust_p = -1.0f, robust_g = 0.0f;
        for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
            const CoherentPick cp = coherentRungFromIndex(r);
            if (!coherentRungLocallyEnabled(cp.mod, cp.rate)) continue;
            if (ceiling != kRungIdxNone && r > ceiling) continue;
            const auto& physical = geometry[r];
            if (!physical.valid()) continue;
            float p = successProb(out.x_used, latentThetaForRung(r));
            if (p > robust_p) {
                robust_p = p;
                robust = r;
                robust_g = physical.useful_bytes_per_frame *
                           static_cast<float>(physical.frames_per_cycle) * p / physical.cycle_s;
            }
            if (p < kDeadProb) continue;              // dead, not merely bad
            p = std::min(p, kCapProb);                // never reward a lucky streak
            const float useful_bytes = physical.useful_bytes_per_frame *
                                       static_cast<float>(physical.frames_per_cycle);
            const float g = useful_bytes * p / physical.cycle_s;
            if (g > out.goodput) { out.goodput = g; out.rung = r; }
            g_by_rung[r] = g;
        }
        // EVERY rung predicted dead (p < kDeadProb): command the most robust one rather than
        // returning none, which made the caller hold the incumbent — on a collapsing link that
        // froze a dense rung in place. With the 2026-11-08 measured thetas this happens below
        // x ~ -0.5 (QPSK R1/4's 10 % point); the old derived R1/4 theta (-2) kept it alive to
        // x ~ -8, outside the posterior range, which is why the case was never reached.
        if (out.rung == kRungIdxNone && robust != kRungIdxNone) {
            out.rung = robust;
            out.goodput = robust_g;
        }

        // TIE-BREAK UPWARD. Walk DOWN from the top so we find the HIGHEST rung that is
        // within the margin, not merely the runner-up: on a flat region several rungs can be
        // near-tied and the informative probe is the highest of them.
        ++decisions_;
        if (allow_tie_break_probe &&
            out.rung != kRungIdxNone && out.goodput > 0.0f &&
            (decisions_ % kTieBreakPeriod) == 0) {
            for (int r = static_cast<int>(kRungIdxCount) - 1;
                 r > static_cast<int>(out.rung); --r) {
                const float g = g_by_rung[r];
                if (g <= 0.0f) continue;
                if (g >= out.goodput * (1.0f - kTieBreakMarginFrac)) {
                    out.rung = static_cast<uint8_t>(r);
                    out.goodput = g;
                    out.tie_break_probe = true;
                    break;
                }
            }
        }
        return out;
    }

    float posteriorMean() const {
        float m = 0.0f;
        for (int i = 0; i < kBins; ++i) m += binX(i) * std::exp(logp_[i]);
        return m;
    }
    float percentile(float q) const {
        float acc = 0.0f;
        for (int i = 0; i < kBins; ++i) {
            acc += std::exp(logp_[i]);
            if (acc >= q) return binX(i);
        }
        return binX(kBins - 1);
    }
    float spreadDb() const {
        const float m = posteriorMean();
        float v = 0.0f;
        for (int i = 0; i < kBins; ++i) {
            const float d = binX(i) - m;
            v += d * d * std::exp(logp_[i]);
        }
        return std::sqrt(std::max(0.0f, v));
    }
    bool havePrior() const { return have_prior_; }
    int observations() const { return observations_; }
    int decisions() const { return decisions_; }

    static float successProb(float x, float theta) {
        const float z = latentSlopePerDb() * (x - theta);
        const float p = 1.0f / (1.0f + std::exp(-std::clamp(z, -40.0f, 40.0f)));
        return std::clamp(p, kLatentMinSuccess, kLatentMaxSuccess);
    }
    static float binX(int i) { return kBinLo + kBinStep * static_cast<float>(i); }

private:
    void normalise() {
        float mx = logp_[0];
        for (int i = 1; i < kBins; ++i) mx = std::max(mx, logp_[i]);
        float sum = 0.0f;
        for (int i = 0; i < kBins; ++i) sum += std::exp(logp_[i] - mx);
        const float lz = mx + std::log(std::max(sum, 1e-30f));
        for (int i = 0; i < kBins; ++i) logp_[i] -= lz;
    }

    float logp_[kBins] = {};
    bool have_prior_ = false;
    int observations_ = 0;
    int decisions_ = 0;
};

}  // namespace protocol
}  // namespace ultra
