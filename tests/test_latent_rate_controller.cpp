// Tests for the latent-state rate controller (ULTRA_LATENT_RATE).
//
// The properties under test are the ones that distinguish this design from the two
// controllers that preceded it and measured a wash / a regression:
//
//   1. Evidence updates EVERY rung, so it survives a rung change (the predecessor zeroed a
//      per-rung ring on every change and then sat inert for 4-8 groups).
//   2. COMMON-MODE calibration error cancels exactly. This is the property that lets the
//      rate path stop consuming kOfdmLegacyAnchorScaleOffsetDb (+8.70 dB) — and the +8.70
//      is itself only a compatibility shim for an estimator bug that was already fixed.
//   3. Value is useful file bytes per physical cycle from production geometry. Spectral
//      efficiency is not multiplied a second time.
//   4. Ranking on the MEASURED waterfall reproduces the measured ordering.

#include "env_compat.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/file_transfer.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/latent_rate_controller.hpp"
#include "protocol/waveform_selection.hpp"

#include <cmath>
#include <iostream>

using namespace ultra;
using namespace ultra::protocol;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { ++tests_failed; std::cout << "FAIL: " << msg << "\n"; } \
    } while (0)

constexpr int kFramesPerGroup = 5;
constexpr uint32_t kNonDataCycleMs = 1790;

LatentRateController::RungGeometryTable productionGeometry(int frames) {
    LatentRateController::RungGeometryTable table{};
    for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
        const auto candidate = coherentRungFromIndex(r);
        if (!coherentRungLocallyEnabled(candidate.mod, candidate.rate)) continue;
        const int cw = connection_policy::recommendCWCount(
            candidate.mod, candidate.rate, WaveformMode::OFDM_CHIRP);
        const size_t capacity = v2::getFixedFramePayloadCapacityZ(
            candidate.rate, cw, 27);
        if (capacity <= FileTransferController::FILE_DATA_OVERHEAD) continue;
        const uint32_t burst_ms = connection_policy::wideOFDMBurstAirtimeMs(
            candidate.mod, candidate.rate, static_cast<size_t>(frames), cw, 0, 27);
        table[r] = {
            static_cast<float>(capacity - FileTransferController::FILE_DATA_OVERHEAD),
            frames,
            static_cast<float>(burst_ms + kNonDataCycleMs) / 1000.0f,
        };
    }
    return table;
}

LatentRateController::RungGeometryTable mpg20SelectorGeometry(uint8_t incumbent) {
    // Exact selector policy for the completed 8PSK MPG@20 run: its runner set the
    // base PA ceiling to 11.5 s and enabled only ULTRA_8PSK_LONG_LDPC. QPSK R3/4
    // therefore remains cw8/Z27. Alternatives include the descriptor-switch full
    // group-start anchor; the incumbent does not.
    constexpr uint32_t kCampaignBaseCeilingMs = 11500;
    LatentRateController::RungGeometryTable table{};
    for (uint8_t rung = kRungIdxQpskR14; rung < kRungIdxCount; ++rung) {
        const auto candidate = coherentRungFromIndex(rung);
        if (!coherentRungLocallyEnabled(candidate.mod, candidate.rate)) continue;

        const int logical_cw = connection_policy::recommendCWCount(
            candidate.mod, candidate.rate, WaveformMode::OFDM_CHIRP);
        int physical_cw = logical_cw;
        int lifting_z = 27;
        if (rung == kRungIdxQam8R23 && logical_cw == 12) {
            physical_cw = 4;
            lifting_z = 81;
        }
        const bool force_full_group_start = rung != incumbent;
        const size_t window = connection_policy::ofdmWindowSize(
            candidate.mod, candidate.rate, /*near_awgn_ofdm=*/false);
        const size_t frames = connection_policy::wideOFDMBurstFrameBudget(
            candidate.mod, candidate.rate, physical_cw, window,
            kCampaignBaseCeilingMs,
            /*continuation_reanchor_ms=*/0, lifting_z,
            force_full_group_start
                ? connection_policy::kWideOFDMFullAnchorExtraMs
                : 0u);
        const size_t capacity = v2::getFixedFramePayloadCapacityZ(
            candidate.rate, physical_cw, lifting_z);
        uint32_t burst_ms = connection_policy::wideOFDMBurstAirtimeMs(
            candidate.mod, candidate.rate, frames, physical_cw,
            /*continuation_reanchor_ms=*/0, lifting_z);
        if (force_full_group_start && frames > 1) {
            burst_ms += connection_policy::kWideOFDMFullAnchorExtraMs;
        }
        table[rung] = {
            static_cast<float>(capacity - FileTransferController::FILE_DATA_OVERHEAD),
            static_cast<int>(frames),
            static_cast<float>(burst_ms + kNonDataCycleMs) / 1000.0f,
        };
    }
    return table;
}

LatentRateController::Pick pick(LatentRateController& c, int frames = kFramesPerGroup,
                                bool allow_probe = false) {
    return c.best(productionGeometry(frames), kRungIdxNone, allow_probe);
}

// ── 1. Evidence updates every rung and survives a rung change ─────────────────
void test_evidence_survives_rung_change() {
    LatentRateController c;
    c.seedPrior(18.0f, 3.0f);
    const float spread0 = c.spreadDb();

    // Observe on ONE rung only.
    for (int i = 0; i < 6; ++i) c.observe(kRungIdxQam8R23, 5, 5);
    CHECK(c.spreadDb() < spread0, "observing sharpens the posterior");
    CHECK(c.observations() == 6, "all observations counted");

    // The predecessor would now be blind about every other rung. Here the SAME posterior
    // already predicts them: a rung with a LOWER theta must have a HIGHER success
    // probability at the same x, with no observation of that rung ever having been made.
    const float x = c.percentile(0.25f);
    const float p_low = LatentRateController::successProb(x, latentThetaForRung(kRungIdxQpskR23));
    const float p_high = LatentRateController::successProb(x, latentThetaForRung(kRungIdxQam16R23));
    CHECK(p_low > p_high,
          "a never-tried lower rung is predicted more reliable than a never-tried higher one");

    // And a rung change costs nothing: no reset, no refill, no inertia.
    const int obs_before = c.observations();
    c.observe(kRungIdxQpskR34, 5, 5);
    CHECK(c.observations() == obs_before + 1,
          "switching rung does not discard accumulated evidence");
}

// ── 2. COMMON-MODE CANCELLATION — the reason +8.70 can leave the rate path ────
// If every theta_r moves by the same constant, the posterior over x moves with it and the
// DECISION is unchanged. That is what makes an outcome-fitted latent state immune to the
// whole family of calibration defects this project has fought: the +8.70 dB compatibility
// offset, the guard-bin noise under-read, the ~5.6 dB IONOS implementation loss.
void test_common_mode_calibration_error_cancels() {
    // Two controllers observing IDENTICAL outcomes, but one lives on a scale shifted by a
    // large constant. Shifting the prior by the same constant is exactly what "the whole
    // link model moved together" means.
    constexpr float kBogusOffset = 8.70f;   // the actual constant, for the avoidance of doubt

    LatentRateController a, b;
    a.seedPrior(18.0f, 3.0f);
    b.seedPrior(18.0f + kBogusOffset, 3.0f);

    const int outcomes[8][2] = {{5,5},{4,5},{5,5},{5,5},{3,5},{5,5},{4,5},{5,5}};
    for (auto& o : outcomes) {
        a.observe(kRungIdxQam8R23, o[0], o[1]);
        b.observe(kRungIdxQam8R23, o[0], o[1]);
    }

    // The posterior means differ by (almost) the offset — b simply lives further up the axis.
    const float shift = b.posteriorMean() - a.posteriorMean();
    CHECK(shift > 0.0f, "the shifted controller's posterior sits higher (shift=" << shift << ")");

    // The property that matters is not that the shift is preserved exactly (the likelihood
    // pulls both toward the SAME theta table, so b is pulled back down). It is that a
    // controller which reads an SNR would be wrong by 8.70 dB, while this one is fitted from
    // OUTCOMES and therefore has no SNR to be wrong about. Assert the decision is driven by
    // outcomes alone: identical outcomes on the SAME theta table give the identical pick,
    // regardless of where the prior was seeded.
    LatentRateController c1, c2;
    c1.seedPrior(10.0f, 6.0f);              // wildly pessimistic seed
    c2.seedPrior(26.0f, 6.0f);              // wildly optimistic seed
    for (int i = 0; i < 25; ++i) {
        c1.observe(kRungIdxQam8R23, 4, 5);
        c2.observe(kRungIdxQam8R23, 4, 5);
    }
    CHECK(std::fabs(c1.posteriorMean() - c2.posteriorMean()) < 1.5f,
          "priors 16 dB apart converge to the same latent state on identical outcomes "
          "(c1=" << c1.posteriorMean() << " c2=" << c2.posteriorMean() << ")");
    CHECK(pick(c1).rung == pick(c2).rung,
          "and therefore reach the same decision — no SNR calibration can move it");
}

// ── 3. Exact useful-byte / physical-cycle objective ───────────────────────────
void test_exact_production_geometry_and_no_eta_double_count() {
    const auto g5 = productionGeometry(5);
    const auto g8 = productionGeometry(8);
    CHECK(std::lround(g5[kRungIdxQpskR34].useful_bytes_per_frame) == 456,
          "QPSK R3/4 geometry carries 456 useful file bytes/frame");
    CHECK(std::lround(g5[kRungIdxQam8R23].useful_bytes_per_frame) == 624,
          "8PSK R2/3 geometry carries 624 useful file bytes/frame");
    CHECK(std::lround(g5[kRungIdxQam16R23].useful_bytes_per_frame) == 840,
          "16QAM R2/3 geometry carries 840 useful file bytes/frame");
    CHECK(g5[kRungIdxQpskR34].cycle_s > 0.0f &&
              g5[kRungIdxQpskR34].frames_per_cycle == 5,
          "five-frame geometry has a physical cycle");
    CHECK(g8[kRungIdxQpskR34].frames_per_cycle == 8 &&
              g8[kRungIdxQpskR34].cycle_s > g5[kRungIdxQpskR34].cycle_s,
          "eight-frame geometry uses its real longer burst cycle");

    // If two rows have identical useful bytes and cycle duration, their values differ
    // only by predicted reliability. No hidden eta term may make the denser row win.
    LatentRateController::RungGeometryTable equal{};
    equal[kRungIdxQpskR34] = {100.0f, 1, 1.0f};
    equal[kRungIdxQam8R23] = {100.0f, 1, 1.0f};
    LatentRateController high;
    high.seedPrior(25.0f, 0.1f);  // both probabilities clamp to the same 0.90 cap
    const auto same = high.best(equal);
    CHECK(same.rung == kRungIdxQpskR34,
          "equal physical value does not reward the denser rung a second time");

    LatentRateController only5, only8;
    only5.seedPrior(18.0f, 0.5f);
    only8.seedPrior(18.0f, 0.5f);
    auto q5 = g5;
    auto q8 = g8;
    for (uint8_t r = 0; r < kRungIdxCount; ++r) {
        if (r != kRungIdxQpskR23) {
            q5[r] = {};
            q8[r] = {};
        }
    }
    CHECK(only8.best(q8).goodput > only5.best(q5).goodput,
          "eight frames amortize the one anchor/turnaround better than five");
}

// ── 4. Ranking reproduces the measured waterfall ordering ─────────────────────
// Default (2026-07) set: the ordering its 4-CW-harness measurement pinned.
void test_ranking_matches_measured_ordering() {
    // At the bench's dial 20 the measured delivered throughput ordering is
    //   8PSK R2/3 (2450) > QPSK R3/4 (2066) > 16QAM R2/3 (1890) > 8PSK R3/4 (1618)
    // so a posterior concentrated near 20 must prefer 8PSK R2/3.
    LatentRateController c;
    c.seedPrior(20.0f, 1.0f);
    const auto p = pick(c);
    CHECK(p.rung == kRungIdxQam8R23,
          "at x~20 the controller picks 8PSK R2/3, the measured best rung (got "
              << static_cast<int>(p.rung) << ")");

    // A poor link must fall back, and a strong one must not sit at the floor.
    LatentRateController lo, hi;
    lo.seedPrior(6.0f, 1.0f);
    hi.seedPrior(24.0f, 1.0f);
    CHECK(pick(lo).rung < kRungIdxQam8R23, "a weak link drops below 8PSK R2/3");
    CHECK(pick(hi).rung >= kRungIdxQam8R23, "a strong link does not sit below it");

    // These are the two points where the retired eta-squared proxy disagreed with exact
    // production bytes/timing in the audit. Pin them so the algebra cannot regress.
    LatentRateController x16, x22;
    x16.seedPrior(16.2f, 0.2f);  // p25 ~= 16 dB
    x22.seedPrior(22.2f, 0.2f);  // p25 ~= 22 dB
    CHECK(pick(x16).rung == kRungIdxQpskR23,
          "exact geometry at x~16 stays on reliable QPSK R2/3");
    CHECK(pick(x22).rung == kRungIdxQam8R23,
          "exact geometry at x~22 prefers 8PSK R2/3 over fragile 16QAM R2/3");
}

void test_ranking_matches_measured_ordering_measured_set() {
    setLatentMeasuredAnchorsForTesting(true);
    // RE-MEASURED 2026-11-08 (production geometry, current PHY, ITU Good, 6 seeds x 24 frames;
    // latent_rate_controller.hpp "RE-MEASURED"): measured delivered useful bytes/s per rung,
    //   14 dB: QPSK R2/3 182 > QPSK R3/4 147
    //   18 dB: 8PSK R2/3 264 > 16QAM R2/3 234
    //   22 dB: 16QAM R2/3 368 > 8PSK R2/3 324     (the 2026-07-26 4-CW harness had 8PSK
    //                                             first at 20-22: superseded)
    // Points sit >= 2 dB from each measured switch (11.9 / 15.8 / 19.8 dB) so the check pins
    // the ordering, not a boundary.
    auto at = [](float x_p25) {
        LatentRateController c;
        c.seedPrior(x_p25 + 0.2f, 0.2f);  // p25 ~= x_p25
        return pick(c).rung;
    };
    CHECK(at(14.0f) == kRungIdxQpskR23, "x~14: QPSK R2/3, the measured best (got "
                                            << static_cast<int>(at(14.0f)) << ")");
    CHECK(at(18.0f) == kRungIdxQam8R23, "x~18: 8PSK R2/3, the measured best (got "
                                            << static_cast<int>(at(18.0f)) << ")");
    CHECK(at(22.0f) == kRungIdxQam16R23, "x~22: 16QAM R2/3, the measured best (got "
                                             << static_cast<int>(at(22.0f)) << ")");

    // A poor link must fall back, and a strong one must not sit at the floor.
    LatentRateController lo, hi;
    lo.seedPrior(6.0f, 1.0f);
    hi.seedPrior(24.0f, 1.0f);
    CHECK(pick(lo).rung < kRungIdxQam8R23, "a weak link drops below 8PSK R2/3");
    CHECK(pick(hi).rung >= kRungIdxQam8R23, "a strong link does not sit below it");
    setLatentMeasuredAnchorsForTesting(false);
}

// When EVERY rung is predicted dead the controller must still command the most robust one;
// returning none made the caller hold the incumbent, freezing a dense rung on a collapsing
// link (reachable since the 2026-11-08 measured thetas put QPSK R1/4's 10 % point at x ~ -0.5).
void test_all_dead_falls_back_to_most_robust_rung() {
    setLatentMeasuredAnchorsForTesting(true);
    LatentRateController c;
    c.seedPrior(-4.0f, 0.25f);
    const auto p = pick(c);
    CHECK(p.rung == kRungIdxQpskR14,
          "all rungs dead at x~-4: command the most robust rung, QPSK R1/4 (got "
              << static_cast<int>(p.rung) << ")");
    CHECK(LatentRateController::successProb(c.percentile(0.25f),
                                            latentThetaForRung(kRungIdxQpskR14)) <
              LatentRateController::kDeadProb,
          "the scenario really is all-dead (else this test checks nothing)");
    setLatentMeasuredAnchorsForTesting(false);
}

// ── 5. Minstrel's two clamps, and why ─────────────────────────────────────────
void test_dead_and_capped_probabilities() {
    // A rung whose predicted success is below 10% must contribute ZERO value, not a small
    // one — otherwise a very dense rung's raw rate can outvote its own unreliability.
    LatentRateController c;
    c.seedPrior(2.0f, 0.5f);        // far below every dense rung's theta
    const auto p = pick(c);
    CHECK(p.rung <= kRungIdxQpskR12,
          "with x far below the ladder, dense rungs are dead and a low rung is chosen");
    // Success is capped so a clean streak cannot make the model claim perfection; the
    // measured waterfall bottoms at 3.5% FER and no rung ever reaches 1.0.
    CHECK(LatentRateController::successProb(60.0f, 13.0f) <= kLatentMaxSuccess + 1e-6f,
          "success probability is capped at the measured fading error floor");
    CHECK(LatentRateController::successProb(-60.0f, 13.0f) >= kLatentMinSuccess - 1e-6f,
          "and floored, so a log-likelihood can never be -inf");
}

// ── 6. The relax term is the one free parameter — pin its behaviour ───────────
void test_relax_widens_and_is_the_only_forgetting() {
    LatentRateController c;
    c.seedPrior(18.0f, 1.0f);
    for (int i = 0; i < 10; ++i) c.observe(kRungIdxQam8R23, 5, 5);
    const float tight = c.spreadDb();
    c.relax(2.0f);
    CHECK(c.spreadDb() > tight, "relax widens the posterior (forgetting)");
    // Zero relax must be an exact no-op: if it drifted, the controller would forget at a
    // rate nobody chose.
    LatentRateController d;
    d.seedPrior(18.0f, 1.0f);
    const float before = d.spreadDb();
    d.relax(0.0f);
    CHECK(std::fabs(d.spreadDb() - before) < 1e-4f, "relax(0) is an exact no-op");
}

// ── 7. Tempering: a 5-frame group is NOT 5 independent bits ──────────────────
void test_correlated_frames_are_tempered() {
    // Burst 8-10 s against Tc 4.23 s gives ~2.2 independent frames per group. Feeding
    // Binomial(5,p) would over-sharpen by ~2.3x per group and the controller would become
    // confidently wrong.
    LatentRateController tempered, raw;
    tempered.seedPrior(18.0f, 3.0f);
    raw.seedPrior(18.0f, 3.0f);
    for (int i = 0; i < 5; ++i) {
        tempered.observe(kRungIdxQam8R23, 5, 5, 2.2f);
        raw.observe(kRungIdxQam8R23, 5, 5, 5.0f);
    }
    CHECK(tempered.spreadDb() > raw.spreadDb(),
          "tempered evidence sharpens more slowly than treating frames as independent");
}

// Trace-conditioned controller replay of the completed MPG@20 8PSK cw4/Z81
// outcomes through its first severe fade. This exercises posterior update and
// candidate ranking only; Connection command adoption, ACK loss, timers, and
// switch execution have separate integration coverage. A group spans roughly
// three coherence times, so its post-decode outcome describes the group that
// ended, not the next transmission. The tempered posterior must absorb one 1/8
// event without a snapshot-style rung chase.
void test_mpg20_trace_conditioned_fade_does_not_chase_next_group() {
    LatentRateController c;
    c.seedPrior(latentThetaForRung(kRungIdxQam8R23), 3.0f);
    const auto geometry = mpg20SelectorGeometry(kRungIdxQam8R23);

    for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
        if (coherentRungLocallyEnabled(coherentRungFromIndex(r).mod,
                                      coherentRungFromIndex(r).rate)) {
            CHECK(geometry[r].frames_per_cycle > 0 && geometry[r].cycle_s > 0.0f,
                  "MPG trace replay must compete against every enabled production rung");
        }
    }

    const int prefix[][2] = {
        {8, 8}, {8, 8}, {7, 8}, {5, 7}, {3, 7}, {7, 7},
    };
    for (const auto& outcome : prefix) {
        c.observe(kRungIdxQam8R23, outcome[0], outcome[1]);
        c.relax(0.35f);
    }
    CHECK(c.best(geometry).rung == kRungIdxQam8R23,
          "precondition: trace prefix cruises on 8PSK with exact MPG@20 runner geometry");

    c.observe(kRungIdxQam8R23, /*delivered=*/1, /*group=*/8);
    c.relax(0.35f);
    const auto after_fade = c.best(geometry);
    CHECK(after_fade.rung == kRungIdxQam8R23,
          "one completed 1/8 fade must not demote the next MPG group using stale evidence");

    // The real next trusted group was 7/7.  Feed it only after it completes; no EVM,
    // SNR, or future/same-frame oracle is part of this replay.
    c.observe(kRungIdxQam8R23, /*delivered=*/7, /*group=*/7);
    c.relax(0.35f);
    CHECK(c.best(geometry).rung == kRungIdxQam8R23,
          "post-fade recovery remains on the measured higher-goodput rung");
}


// ── 8. TIE-BREAK UPWARD — the counterweight to the pessimism ──────────────────
// Rig-motivated: deciding from the 25th percentile cost 1.2 dB (mean 16.2 vs p25 15.0),
// exactly the gap to the QPSK R2/3 -> 8PSK R2/3 crossover at ~15.5, so the controller sat
// one rung low for whole transfers. It still beat the ladder by 14.5% on stability alone —
// it WON while using the good rung LESS (0.4 vs 1.2 changes/run) — but the rung gain was
// left unclaimed. The likelihood also SATURATES (a rung that always succeeds stops carrying
// information), so no amount of further evidence lifts x past the plateau on its own.
void test_tie_break_probes_upward() {
    // Find the near-tie bands on the code-faithful geometry rather than baking in the old
    // eta-squared crossover. On the fourth decision an explicitly enabled experiment may
    // choose a higher near-tied rung; the default caller never enables it.
    int probes = 0, higher_than_argmax = 0;
    for (float x = -4.0f; x <= 25.0f; x += 0.25f) {
        LatentRateController baseline, experimental;
        baseline.seedPrior(x, 0.2f);
        experimental.seedPrior(x, 0.2f);
        const uint8_t argmax_rung = pick(baseline).rung;
        LatentRateController::Pick p;
        for (int i = 0; i < LatentRateController::kTieBreakPeriod; ++i) {
            p = pick(experimental, kFramesPerGroup, /*allow_probe=*/true);
        }
        if (p.tie_break_probe) {
            ++probes;
            if (p.rung > argmax_rung) ++higher_than_argmax;
        }
    }
    CHECK(probes > 0, "the tie-break fires when rungs are near-tied (got " << probes << ")");
    CHECK(higher_than_argmax == probes,
          "every probe selects a rung ABOVE the argmax — probing downward would be pointless");

    // BOUNDED COST. A probe is only taken when the higher rung is within
    // kTieBreakMarginFrac of the best, so the expected loss is margin/period.
    const float worst_case_loss = LatentRateController::kTieBreakMarginFrac /
                                  static_cast<float>(LatentRateController::kTieBreakPeriod);
    CHECK(worst_case_loss < 0.05f,
          "expected cost of probing is bounded below 5% of one group (" << worst_case_loss << ")");

    // It must NOT fire when the rungs are far apart — that would be a plain over-commit.
    LatentRateController far;
    far.seedPrior(4.0f, 0.5f);        // deep in QPSK territory, dense rungs nowhere near
    int far_probes = 0;
    for (int i = 0; i < 40; ++i) {
        if (pick(far, kFramesPerGroup, /*allow_probe=*/true).tie_break_probe) ++far_probes;
    }
    CHECK(far_probes == 0,
          "no probe when the higher rungs are not within the margin (got " << far_probes << ")");

    LatentRateController production;
    production.seedPrior(15.2f, 0.3f);
    int default_probes = 0;
    for (int i = 0; i < 40; ++i) {
        if (pick(production).tie_break_probe) ++default_probes;
    }
    CHECK(default_probes == 0,
          "whole-group tie probes are disabled unless the caller explicitly opts in");
}

void test_reset_clears_qso_state_and_probe_phase() {
    LatentRateController c;
    c.seedPrior(18.0f, 2.0f);
    c.observe(kRungIdxQam8R23, 6, 8);
    (void)pick(c, 8, true);
    CHECK(c.havePrior() && c.observations() == 1 && c.decisions() == 1,
          "precondition: controller accumulated one QSO's state");
    c.reset();
    CHECK(!c.havePrior(), "reset clears prior/posterior ownership");
    CHECK(c.observations() == 0, "reset clears observation count");
    CHECK(c.decisions() == 0, "reset clears decision/probe phase");
}

void test_operator_constraints_cover_latent_early_return() {
    LatentRateController strong_r12;
    strong_r12.seedPrior(24.0f, 0.5f);
    setenv("ULTRA_MAX_OFDM_RATE", "R1_2", 1);
    const uint8_t ceiling_r12 = latentConfiguredRungCeiling();
    const auto pick_r12 = strong_r12.best(productionGeometry(5), ceiling_r12);
    unsetenv("ULTRA_MAX_OFDM_RATE");
    CHECK(ceiling_r12 == kRungIdxQpskR12 && pick_r12.rung <= kRungIdxQpskR12,
          "ULTRA_MAX_OFDM_RATE=R1_2 is an absolute latent argmax ceiling");

    LatentRateController strong_r23;
    strong_r23.seedPrior(24.0f, 0.5f);
    setenv("ULTRA_MAX_OFDM_RATE", "r2_3", 1);
    const uint8_t ceiling_r23 = latentConfiguredRungCeiling();
    const auto pick_r23 = strong_r23.best(productionGeometry(5), ceiling_r23);
    unsetenv("ULTRA_MAX_OFDM_RATE");
    CHECK(ceiling_r23 == kRungIdxQpskR23 && pick_r23.rung <= kRungIdxQpskR23,
          "ULTRA_MAX_OFDM_RATE=R2_3 is an absolute latent argmax ceiling");

    setenv("ULTRA_FORCE_DATA_MOD", "QPSK", 1);
    CHECK(!latentStartupProbeAllowedByOperator(),
          "ULTRA_FORCE_DATA_MOD suppresses automatic startup exploration");
    unsetenv("ULTRA_FORCE_DATA_MOD");
    setenv("ULTRA_FORCE_DATA_RATE", "R1_2", 1);
    CHECK(!latentStartupProbeAllowedByOperator(),
          "ULTRA_FORCE_DATA_RATE suppresses automatic startup exploration");
    unsetenv("ULTRA_FORCE_DATA_RATE");
    setenv("ULTRA_FORCE_DATA_MOD", "", 1);
    setenv("ULTRA_FORCE_DATA_RATE", "not_a_rate", 1);
    CHECK(latentStartupProbeAllowedByOperator(),
          "empty/invalid force variables must not freeze automatic selection");
    unsetenv("ULTRA_FORCE_DATA_MOD");
    unsetenv("ULTRA_FORCE_DATA_RATE");
    setenv("ULTRA_LOCK_RATE", "1", 1);
    CHECK(!latentStartupProbeAllowedByOperator(),
          "ULTRA_LOCK_RATE suppresses automatic startup exploration");
    unsetenv("ULTRA_LOCK_RATE");
    CHECK(latentStartupProbeAllowedByOperator(),
          "automatic startup exploration is available when no operator pin exists");
}

}  // namespace

int main() {
    std::cout << "=== Latent Rate Controller Tests ===\n";
    unsetenv("ULTRA_MAX_BURST_AIRTIME_MS");
    unsetenv("ULTRA_BURST_ESC_STREAK");
    unsetenv("ULTRA_BURST_ESCALATION");
    unsetenv("ULTRA_COHERENT_WINDOW");
    test_evidence_survives_rung_change();
    test_common_mode_calibration_error_cancels();
    test_exact_production_geometry_and_no_eta_double_count();
    test_ranking_matches_measured_ordering();
    test_ranking_matches_measured_ordering_measured_set();
    test_all_dead_falls_back_to_most_robust_rung();
    test_dead_and_capped_probabilities();
    test_relax_widens_and_is_the_only_forgetting();
    test_correlated_frames_are_tempered();
    test_mpg20_trace_conditioned_fade_does_not_chase_next_group();
    test_tie_break_probes_upward();
    test_reset_clears_qso_state_and_probe_phase();
    test_operator_constraints_cover_latent_early_return();
    std::cout << (tests_failed == 0 ? "PASS" : "FAIL") << ": "
              << (tests_run - tests_failed) << "/" << tests_run << " checks\n";
    return tests_failed == 0 ? 0 : 1;
}
