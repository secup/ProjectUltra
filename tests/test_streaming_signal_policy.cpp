#include "sync/cfo_tracker.hpp"
#include "sync/signal_policy.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace ultra::sync::signal_policy;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
            return; \
        } \
    } while (0)

#define CHECK_CLOSE(actual, expected, tol, msg) \
    CHECK(std::abs((actual) - (expected)) <= (tol), msg)

void test_mean_abs_llr() {
    CHECK(meanAbsLLR(nullptr, 3) == 0.0f, "null LLR pointer should be safe");
    CHECK(meanAbsLLR(nullptr, 0) == 0.0f, "empty LLR span should be safe");

    const float values[] = {1.0f, -2.0f, 3.0f};
    CHECK_CLOSE(meanAbsLLR(values, 3), 2.0f, 0.0001f, "mean absolute LLR");
}

void test_future_training_start_is_deferred_not_clamped() {
    // Exact live IONOS incident (qpsk run, mac.log 41.791): the dual-chirp lock was
    // valid, but its training span started 1,341 samples after the current write head.
    // The former production branch replaced 673085 with 671744 and immediately decoded
    // unwritten samples.  The descriptor then failed despite a valid chirp acquisition.
    constexpr size_t kSearchStartAbs = 548285;
    constexpr int kTrainingOffset = 124800;
    constexpr size_t kFedAtDetection = 671744;
    constexpr size_t kTrueTrainingAbs = 673085;

    const auto future = classifyDetectedTrainingStart(
        kSearchStartAbs, kTrainingOffset, kFedAtDetection);
    CHECK(future.disposition == TrainingStartDisposition::DEFER,
          "future training must defer instead of decoding at the write head");
    CHECK(future.detected_abs == kTrueTrainingAbs,
          "deferral must preserve the detector's exact absolute training position");
    CHECK(future.samples_missing == 1341,
          "deferral must wait for exactly the missing training samples");
    CHECK(future.detected_abs != kFedAtDetection,
          "the future position must never be clamped to total_fed");

    const auto ready = classifyDetectedTrainingStart(
        kSearchStartAbs, kTrainingOffset, kTrueTrainingAbs);
    CHECK(ready.disposition == TrainingStartDisposition::READY,
          "the same preserved candidate becomes ready when training reaches the ring");
    CHECK(ready.detected_abs == kTrueTrainingAbs,
          "ready transition must not move the candidate");
    CHECK(ready.samples_missing == 0,
          "ready candidate has no missing samples");

    const auto invalid = classifyDetectedTrainingStart(
        kSearchStartAbs, -1, kFedAtDetection);
    CHECK(invalid.disposition == TrainingStartDisposition::INVALID,
          "a found result with a negative training offset is invalid");
}

void test_presync_llr_quality() {
    std::vector<float> strong(648, 2.0f);
    auto good = evaluatePreSyncLLR(strong.data(), strong.size(), 648);
    CHECK(good.count == 648, "presync LLR should cap at first CW");
    CHECK(good.near_zero_count == 0, "strong LLR should have no near-zero samples");
    CHECK_CLOSE(good.mean_abs, 2.0f, 0.0001f, "strong LLR mean");
    CHECK(!good.reject_as_false_lock, "strong LLR should not reject");

    // SHAPE gate DEFAULT-ON (2026-07-05): a low-mean but ZERO-FREE population is a
    // weak-but-REAL signal (the F92 16QAM head-frame class) — NOT a false lock.
    std::vector<float> weak(648, 1.0f);
    auto weak_quality = evaluatePreSyncLLR(weak.data(), weak.size(), 648);
    CHECK(!weak_quality.reject_as_false_lock,
          "weak-but-clean population must NOT reject (shape gate default)");
    // Noise IS zero-dominated — that still rejects.
    std::vector<float> noise(648, 0.05f);
    auto noise_quality = evaluatePreSyncLLR(noise.data(), noise.size(), 648);
    CHECK(noise_quality.reject_as_false_lock, "zero-dominated population must reject");

    std::vector<float> erased(648, 2.0f);
    for (size_t i = 0; i < 220; ++i) {
        erased[i] = 0.05f;
    }
    auto erasure_quality = evaluatePreSyncLLR(erased.data(), erased.size(), 648);
    CHECK(erasure_quality.reject_as_false_lock, "high near-zero fraction should reject");
    CHECK(erasure_quality.near_zero_fraction > kMaxErasureLikeFraction,
          "near-zero fraction should exceed policy limit");

    auto empty = evaluatePreSyncLLR(nullptr, 0, 648);
    CHECK(empty.reject_as_false_lock, "empty LLR quality should reject");
    CHECK(empty.near_zero_fraction == 1.0f, "empty LLR quality should be fully erasure-like");
}

void test_invalid_ofdm_lts_training() {
    CHECK(!invalidOFDMLTSTraining(false, true, 0.0f, 0.0f),
          "non-OFDM path should not apply LTS gate");
    CHECK(!invalidOFDMLTSTraining(true, false, 0.0f, 0.0f),
          "disconnected path should not apply LTS gate");
    CHECK(invalidOFDMLTSTraining(true, true,
                                 std::numeric_limits<float>::quiet_NaN(), 1.0f),
          "NaN LTS signal should reject");
    CHECK(invalidOFDMLTSTraining(true, true, 1.0f,
                                 std::numeric_limits<float>::infinity()),
          "infinite channel magnitude should reject");
    CHECK(invalidOFDMLTSTraining(true, true, 0.0005f, 0.01f),
          "both weak LTS metrics should reject");
    CHECK(!invalidOFDMLTSTraining(true, true, 0.0100f, 0.01f),
          "adequate LTS signal should not reject by itself");
    CHECK(!invalidOFDMLTSTraining(true, true, 0.0005f, 0.20f),
          "adequate channel magnitude should not reject by itself");
}

void test_light_sync_thresholds() {
    auto coherent = lightSyncThresholds(true, false, true, 99);
    CHECK_CLOSE(coherent.min_confidence, 0.90f, 0.0001f,
                "coherent threshold should stay strict");
    CHECK_CLOSE(coherent.weak_floor, 0.85f, 0.0001f, "coherent weak floor");

    auto narrow = lightSyncThresholds(false, true, true, 0);
    CHECK_CLOSE(narrow.min_confidence, 0.50f, 0.0001f, "narrowband threshold");
    CHECK_CLOSE(narrow.weak_floor, 0.40f, 0.0001f, "narrowband weak floor");

    auto connected = lightSyncThresholds(false, false, true, 0);
    CHECK_CLOSE(connected.min_confidence, 0.52f, 0.0001f, "connected threshold");
    CHECK_CLOSE(connected.weak_floor, 0.45f, 0.0001f, "connected weak floor");

    auto warm = lightSyncThresholds(false, false, true, 0, true, 9600, 2176);
    const float expected_warm_threshold =
        deriveNarrowWindowMagnitudeThreshold(0.52f, 9600, 2176);
    CHECK(warm.narrow_expected_window, "warm expected-window threshold should report narrow mode");
    CHECK_CLOSE(warm.false_positive_window_reduction, 9600.0f / 2176.0f, 0.001f,
                "warm threshold should expose window reduction factor");
    CHECK_CLOSE(warm.threshold_reduction_db,
                10.0f * std::log10(9600.0f / 2176.0f), 0.001f,
                "warm threshold should expose dB reduction");
    CHECK_CLOSE(warm.min_confidence, expected_warm_threshold, 0.0001f,
                "warm threshold should be derived from window-size reduction");
    CHECK(warm.min_confidence > 0.24f && warm.min_confidence < 0.26f,
          "warm threshold should land near 0.25 for the current narrow window");

    auto disconnected = lightSyncThresholds(false, false, false, 0);
    CHECK_CLOSE(disconnected.min_confidence, 0.65f, 0.0001f, "disconnected threshold");
    CHECK_CLOSE(disconnected.weak_floor, 0.55f, 0.0001f, "disconnected weak floor");

    auto relaxed = lightSyncThresholds(false, false, true, 5);
    CHECK_CLOSE(relaxed.min_confidence, 0.50f, 0.0001f, "reject streak should relax threshold");

    auto floor = lightSyncThresholds(false, false, true, 99);
    CHECK_CLOSE(floor.min_confidence, 0.40f, 0.0001f, "relaxation should have a hard floor");
    CHECK_CLOSE(floor.weak_floor, 0.35f, 0.0001f,
                "long reject streak should enable connected OFDM rescue floor");
}

void test_light_sync_decision() {
    LightSyncThresholds thresholds{0.52f, 0.45f};

    auto not_found = evaluateLightSyncCandidate(false, 0.0f, false, true, 7, thresholds);
    CHECK(!not_found.found, "not-found input should stay not found");
    CHECK(not_found.next_reject_streak == 7, "not-found input should preserve reject streak");

    auto strong = evaluateLightSyncCandidate(true, 0.80f, false, true, 7, thresholds);
    CHECK(strong.found, "strong candidate should be accepted");
    CHECK(!strong.weak_accept, "strong candidate should not be weak accept");
    CHECK(strong.next_reject_streak == 0, "strong candidate should clear reject streak");

    auto rejected = evaluateLightSyncCandidate(true, 0.43f, false, true, 0, thresholds);
    CHECK(!rejected.found, "low-correlation candidate should reject");
    CHECK(rejected.rejected, "low-correlation candidate should set rejected flag");
    CHECK(rejected.next_reject_streak == 1, "rejection should increment streak");

    auto weak = evaluateLightSyncCandidate(true, 0.46f, false, true, 3, thresholds);
    CHECK(weak.found, "weak candidate should be accepted after reject streak");
    CHECK(weak.weak_accept, "weak candidate should report weak accept");
    CHECK(weak.next_reject_streak == 1, "weak accept should decay reject streak");

    auto rescued = evaluateLightSyncCandidate(
        true, 0.36f, false, true, 8,
        lightSyncThresholds(false, false, true, 8));
    CHECK(rescued.found, "late connected OFDM rescue candidate should be accepted");
    CHECK(rescued.weak_accept, "late connected OFDM rescue should report weak accept");

    auto coherent = evaluateLightSyncCandidate(true, 0.86f, true, true, 10,
                                               LightSyncThresholds{0.90f, 0.85f});
    CHECK(!coherent.found, "coherent modes should not weak-accept below threshold");
    CHECK(coherent.next_reject_streak == 11, "coherent rejection should increment streak");
}

void test_cfo_drift_limit() {
    // High-correlation (trusted) chirps: behaviour is the original drift clamp.
    constexpr float kHi = 1.0f;
    auto disconnected = limitConnectedCFODrift(false, 10.0f, 1.0f, kHi, /*established=*/true);
    CHECK(!disconnected.clamped, "disconnected trusted CFO should not clamp");
    CHECK_CLOSE(disconnected.accepted_cfo, 10.0f, 0.0001f, "disconnected CFO accepted");

    auto unknown = limitConnectedCFODrift(true, 10.0f, 0.0f, kHi, /*established=*/false);
    CHECK(!unknown.clamped, "near-zero known + trusted chirp should not clamp");
    CHECK_CLOSE(unknown.accepted_cfo, 10.0f, 0.0001f, "unknown CFO accepted");

    auto small_drift = limitConnectedCFODrift(true, 10.9f, 10.0f, kHi, true);
    CHECK(!small_drift.clamped, "small CFO drift should not clamp");
    CHECK_CLOSE(small_drift.accepted_cfo, 10.9f, 0.0001f, "small drift accepted");

    auto large_drift = limitConnectedCFODrift(true, 12.0f, 10.0f, kHi, true);
    CHECK(large_drift.clamped, "large CFO drift should clamp");
    CHECK_CLOSE(large_drift.accepted_cfo, 10.0f, 0.0001f, "large drift should use known CFO");
    CHECK_CLOSE(large_drift.diff_hz, 2.0f, 0.0001f, "CFO diff telemetry");

    // Low-confidence (fade-jittered) chirp against an ESTABLISHED (measured) value: a large jump is
    // rejected to it at every stage, including the pre-connect PING.
    auto phantom_ping = limitConnectedCFODrift(false, 9.7f, 7.3f, 0.73f, /*established=*/true);
    CHECK(phantom_ping.clamped, "low-corr phantom must clamp to an established value");
    CHECK_CLOSE(phantom_ping.accepted_cfo, 7.3f, 0.0001f, "phantom rejected to the measured value");

    // 2026-11-08 (real dial offset): an UNESTABLISHED tracker holds the default 0 Hz, which is not
    // a measurement. A low-confidence chirp must NOT be replaced by it: that turned a correct
    // 7.3 Hz reading into a 7.3 Hz error on 27/48 Good@20 frames (62.5 % FER vs 4.2 %). A phantom
    // in an accepted chirp is corrected by the first OFDM frame's LTS (uncertified prior).
    auto unestablished = limitConnectedCFODrift(true, 7.3f, 0.0f, 0.80f, /*established=*/false);
    CHECK(!unestablished.clamped, "unestablished tracker accepts even a low-corr chirp");
    CHECK_CLOSE(unestablished.accepted_cfo, 7.3f, 0.0001f, "real dial offset kept");

    // Genuine first acquisition: a clean high-corr chirp with a large real dial offset is accepted
    // (no established CFO to clamp to, and the chirp is trusted).
    auto real_acq = limitConnectedCFODrift(false, 30.0f, 0.0f, 0.95f, /*established=*/false);
    CHECK(!real_acq.clamped, "trusted large real CFO at acquisition must be accepted");
    CHECK_CLOSE(real_acq.accepted_cfo, 30.0f, 0.0001f, "real dial offset acquired");
}

void test_connected_anchor_cfo_seed() {
    // BUG-ANCHOR-CFO-KILL: once pilot-refined, the connected full-anchor keeps the
    // warm CFO whatever the chirp reads.
    // (1) The sticky-G13 sub-clamp phantom: known +0.20, chirp -0.20, corr 0.94 —
    //     the drift clamp passes it (diff 0.4 < 1.0); the seed must reject it.
    auto g13 = connectedAnchorCFOSeed(-0.20f, 0.20f, 0.94f, /*pilot_seeded=*/true, true);
    CHECK(g13.used_warm, "seeded anchor must keep warm CFO");
    CHECK_CLOSE(g13.accepted_cfo, 0.20f, 0.0001f, "sub-clamp phantom rejected to warm");

    // (2) The >1 Hz high-corr blind spot: known ~0 (unestablished), chirp -1.15,
    //     corr 0.90 — the drift clamp accepts it (no established CFO); the seed must not.
    auto blind = connectedAnchorCFOSeed(-1.15f, 0.0f, 0.90f, true, true);
    CHECK(blind.used_warm, "seeded anchor rejects the high-corr unestablished phantom");
    CHECK_CLOSE(blind.accepted_cfo, 0.0f, 0.0001f, "phantom rejected to warm 0");

    // (3) NOT pilot-seeded: cold rules — a trusted large real CFO is accepted.
    auto cold = connectedAnchorCFOSeed(3.0f, 0.0f, 0.95f, /*pilot_seeded=*/false, false);
    CHECK(!cold.used_warm, "unseeded anchor keeps cold chirp trust");
    CHECK_CLOSE(cold.accepted_cfo, 3.0f, 0.0001f, "cold acquisition CFO accepted");

    // (4) NOT pilot-seeded + low-confidence phantom against a MEASURED value: the clamp bites.
    auto cold_phantom = connectedAnchorCFOSeed(9.7f, 7.3f, 0.73f, false, /*established=*/true);
    CHECK(!cold_phantom.used_warm, "unseeded path is the cold arm");
    CHECK_CLOSE(cold_phantom.accepted_cfo, 7.3f, 0.0001f,
                "low-corr phantom clamped to the measured value on the cold arm");
    // (5) NOT pilot-seeded, NOTHING measured yet (fresh tracker): the chirp is the only
    //     information and is accepted even at low correlation (the 2026-11-08 real-offset bug).
    auto cold_fresh = connectedAnchorCFOSeed(7.3f, 0.0f, 0.73f, false, /*established=*/false);
    CHECK_CLOSE(cold_fresh.accepted_cfo, 7.3f, 0.0001f,
                "fresh tracker keeps the low-corr chirp instead of the default 0 Hz");
}

void test_pilot_cfo_update() {
    auto residual_only = combinePilotCFO(0.0f, 0.25f, 0.0f, false);
    CHECK_CLOSE(residual_only.unclamped_cfo, 0.25f, 0.0001f,
                "pilot CFO should use residual when no pre-correction was applied");
    CHECK_CLOSE(residual_only.accepted_cfo, 0.25f, 0.0001f, "unclamped residual accepted");
    CHECK(!residual_only.clamped, "unclamped update should not clamp");

    auto corrected = combinePilotCFO(9.5f, 0.4f, 9.0f, true);
    CHECK_CLOSE(corrected.unclamped_cfo, 9.9f, 0.0001f,
                "pilot CFO should add pre-correction and residual");
    CHECK_CLOSE(corrected.accepted_cfo, 9.9f, 0.0001f, "small pilot CFO drift accepted");
    CHECK(!corrected.clamped, "small pilot CFO drift should not clamp");

    auto positive_clamp = combinePilotCFO(9.5f, 4.0f, 10.0f, true);
    CHECK(positive_clamp.clamped, "large positive pilot CFO drift should clamp");
    CHECK_CLOSE(positive_clamp.unclamped_cfo, 13.5f, 0.0001f,
                "pilot CFO unclamped telemetry");
    CHECK_CLOSE(positive_clamp.accepted_cfo, 12.0f, 0.0001f,
                "positive pilot CFO clamp should step from reference");

    auto negative_clamp = combinePilotCFO(9.5f, -4.0f, 10.0f, true);
    CHECK(negative_clamp.clamped, "large negative pilot CFO drift should clamp");
    CHECK_CLOSE(negative_clamp.accepted_cfo, 8.0f, 0.0001f,
                "negative pilot CFO clamp should step from reference");
}

void test_burst_outcome_cfo_reconciliation() {
    using ultra::sync::CFOTracker;
    using Action = CFOTracker::BurstOutcomeAction;

    // Exact MPG@20 failure: the descriptor/control frame proved +0.05 Hz, but
    // fade-corrupted pilot feedback late in a partially decoded group walked
    // the mutable tracker to -0.79 Hz. The partial verdict must restore the
    // proven value before the next anchor or synchronous retry callback.
    CFOTracker tracker;
    tracker.store(0.05f);
    CHECK(tracker.applyBurstDecodeOutcome(8, 8) == Action::CERTIFIED,
          "all-good burst should certify the current CFO");
    CHECK(tracker.pilotSeeded(), "all-good burst should arm warm CFO");

    tracker.store(-0.79f);
    CHECK(tracker.applyBurstDecodeOutcome(2, 8) == Action::ROLLED_BACK_WARM,
          "partial burst should roll mutable CFO back to the proven snapshot");
    CHECK_CLOSE(tracker.tracked(), 0.05f, 0.0001f,
                "partial burst should restore the exact certified CFO");
    CHECK(tracker.pilotSeeded(),
          "partial rollback should preserve an existing warm certificate");
    const auto next_anchor = connectedAnchorCFOSeed(
        0.20f, tracker.tracked(), 0.79f, tracker.pilotSeeded(), tracker.established());
    CHECK_CLOSE(next_anchor.accepted_cfo, 0.05f, 0.0001f,
                "next connected anchor should consume the restored warm CFO");

    tracker.store(-0.79f);
    CHECK(tracker.applyBurstDecodeOutcome(0, 7) == Action::ROLLED_BACK_COLD,
          "zero-good burst should restore the proven reference and revoke warm CFO");
    CHECK_CLOSE(tracker.tracked(), 0.05f, 0.0001f,
                "zero-good burst must not leave poison as the cold chirp reference");
    CHECK(!tracker.pilotSeeded(), "zero-good burst should force cold re-acquisition");

    // No prior proof: a partial group cannot create a certificate from its own
    // ambiguous residual history. It must also clear the numeric reference: a
    // >1 Hz pilot walk would otherwise clamp the next connected-cold chirp back
    // to poison.
    CFOTracker fresh;
    fresh.store(2.75f);
    CHECK(fresh.applyBurstDecodeOutcome(1, 2) == Action::REVOKED,
          "partial burst without a snapshot should remain cold");
    CHECK_CLOSE(fresh.tracked(), 0.0f, 0.0001f,
                "partial burst without proof should clear its unproven CFO reference");
    CHECK(!fresh.pilotSeeded(),
          "partial burst without proof must not manufacture a warm certificate");

    CFOTracker no_snapshot_crater;
    no_snapshot_crater.store(-2.75f);
    CHECK(no_snapshot_crater.applyBurstDecodeOutcome(0, 8) == Action::REVOKED,
          "zero-good burst without a snapshot should remain cold");
    CHECK_CLOSE(no_snapshot_crater.tracked(), 0.0f, 0.0001f,
                "zero-good burst without proof should clear the cold-clamp reference");

    CFOTracker no_snapshot_abort;
    no_snapshot_abort.store(2.75f);
    CHECK(no_snapshot_abort.reconcileUncertainBurstEnd() == Action::REVOKED,
          "uncertain burst without a snapshot should remain cold");
    CHECK_CLOSE(no_snapshot_abort.tracked(), 0.0f, 0.0001f,
                "uncertain burst without proof should clear the cold-clamp reference");

    // BUG-CFO-REVOKE-TO-ZERO: with no certificate yet, every revoke path falls back to the
    // session's handshake measurement (setSessionReference), uncertified and cold — never to
    // the 0 Hz default that left a 7.3 Hz link permanently outside the LTS correction bound.
    for (int path = 0; path < 3; ++path) {
        CFOTracker session;
        session.store(7.40f);
        session.setSessionReference(7.40f);
        session.store(11.9f);  // a pilot walk that must not survive
        const Action a = path == 0 ? session.reconcileUncertainBurstEnd()
                       : path == 1 ? session.applyBurstDecodeOutcome(0, 8)
                                   : session.applyBurstDecodeOutcome(1, 2);
        CHECK(a == Action::RESTORED_SESSION_REFERENCE,
              "no certificate: revoke restores the session reference");
        CHECK_CLOSE(session.tracked(), 7.40f, 0.0001f,
                    "the handshake CFO, not 0 Hz and not the pilot walk");
        CHECK(session.established() && !session.pilotSeeded(),
              "restored reference is information (established) but not warm proof");
    }
    {
        CFOTracker certified_wins;
        certified_wins.setSessionReference(7.40f);
        certified_wins.store(7.10f);
        certified_wins.applyBurstDecodeOutcome(8, 8);  // certifies 7.10
        certified_wins.store(3.0f);
        CHECK(certified_wins.reconcileUncertainBurstEnd() == Action::ROLLED_BACK_WARM,
              "a delivered-group certificate takes precedence over the session reference");
        CHECK_CLOSE(certified_wins.tracked(), 7.10f, 0.0001f, "certified value restored");
        certified_wins.reset();
        certified_wins.store(5.0f);
        CHECK(certified_wins.reconcileUncertainBurstEnd() == Action::REVOKED,
              "reset clears the session reference with the certificate");
        CHECK_CLOSE(certified_wins.tracked(), 0.0f, 0.0001f, "no reference left: 0 Hz default");
        CFOTracker nan_ref;
        nan_ref.setSessionReference(std::numeric_limits<float>::quiet_NaN());
        nan_ref.store(2.0f);
        CHECK(nan_ref.reconcileUncertainBurstEnd() == Action::REVOKED,
              "a non-finite reference is ignored");
    }

    // A historical snapshot may still be useful numerically after 0/N revoked
    // warm trust, but restoring it must not silently re-certify it.
    CFOTracker historical;
    historical.store(0.05f);
    historical.applyBurstDecodeOutcome(8, 8);
    historical.applyBurstDecodeOutcome(0, 8);
    historical.store(-0.79f);
    CHECK(historical.applyBurstDecodeOutcome(1, 2) == Action::ROLLED_BACK_COLD,
          "partial burst may restore the last historical proven value");
    CHECK_CLOSE(historical.tracked(), 0.05f, 0.0001f,
                "historical rollback should restore the proven numeric CFO");
    CHECK(!historical.pilotSeeded(),
          "historical rollback must preserve the revoked cold state");

    // Unknown terminal: restore the numeric snapshot, but preserve whether that
    // snapshot was still trusted before the ambiguous receive attempt.
    CFOTracker uncertain;
    uncertain.store(0.05f);
    uncertain.applyBurstDecodeOutcome(8, 8);
    uncertain.store(-0.79f);
    CHECK(uncertain.reconcileUncertainBurstEnd() == Action::ROLLED_BACK_WARM,
          "uncertain abort should restore and preserve prior warm proof");
    CHECK_CLOSE(uncertain.tracked(), 0.05f, 0.0001f,
                "uncertain abort should discard mutable pilot walk");
    CHECK(uncertain.pilotSeeded(),
          "uncertain abort must not revoke unrelated prior proof");

    // Malformed outcomes fail closed, and reset atomically clears both the warm
    // flag and historical snapshot.
    uncertain.store(-0.79f);
    CHECK(uncertain.applyBurstDecodeOutcome(9, 8) == Action::ROLLED_BACK_COLD,
          "invalid decoded count should restore the snapshot and fail cold");
    CHECK(!uncertain.pilotSeeded(), "invalid outcome must revoke warm trust");
    uncertain.reset();
    uncertain.store(-0.79f);
    CHECK(uncertain.applyBurstDecodeOutcome(1, 2) == Action::REVOKED,
          "reset must clear the historical certificate transactionally");

    // Non-finite estimates can never become a warm anchor or replace a finite
    // tracked reference.
    CFOTracker finite;
    finite.store(0.05f);
    finite.applyBurstDecodeOutcome(8, 8);
    finite.store(std::numeric_limits<float>::quiet_NaN());
    CHECK_CLOSE(finite.tracked(), 0.05f, 0.0001f,
                "non-finite store should retain the finite tracked CFO");
    CHECK(!finite.pilotSeeded(), "non-finite store should revoke warm trust");
}

}  // namespace

int main() {
    test_mean_abs_llr();
    test_future_training_start_is_deferred_not_clamped();
    test_presync_llr_quality();
    test_invalid_ofdm_lts_training();
    test_light_sync_thresholds();
    test_light_sync_decision();
    test_cfo_drift_limit();
    test_connected_anchor_cfo_seed();
    test_pilot_cfo_update();
    test_burst_outcome_cfo_reconciliation();

    if (tests_failed != 0) {
        std::cout << "StreamingSignalPolicy: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "StreamingSignalPolicy: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
