#include "gui/modem/streaming_frame_policy.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace ultra::gui::streaming_frame_policy;

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

void test_rms_and_ping_detection() {
    CHECK(rms(nullptr, 4) == 0.0f, "null RMS span should be safe");
    const float values[] = {1.0f, -1.0f, 1.0f, -1.0f};
    CHECK_CLOSE(rms(values, 4), 1.0f, 0.0001f, "RMS of unit samples");

    std::vector<float> ping(kPingTrainingSkipSamples + kPingRMSCheckSamples, 0.0f);
    for (size_t i = 0; i < kPingTrainingSkipSamples; ++i) {
        ping[i] = 1.0f;
    }
    for (size_t i = kPingTrainingSkipSamples; i < ping.size(); ++i) {
        ping[i] = 0.2f;
    }
    auto ping_decision = evaluatePingRMS(ping.data(), ping.size());
    CHECK(ping_decision.is_ping, "low data/training RMS ratio should be ping");
    CHECK_CLOSE(ping_decision.ratio, 0.2f, 0.0001f, "ping RMS ratio");

    std::vector<float> data(kPingTrainingSkipSamples + kPingRMSCheckSamples, 1.0f);
    auto data_decision = evaluatePingRMS(data.data(), data.size());
    CHECK(!data_decision.is_ping, "data energy after training should not be ping");
    CHECK_CLOSE(data_decision.ratio, 1.0f, 0.0001f, "data RMS ratio");

    // CHANGED 2026-07-30 (BUG-SYNC-CURSOR-AHEAD). This previously asserted that an EMPTY
    // buffer (nullptr, 0 samples) classifies as a PING — the same defect that deadlocked the
    // rig, in miniature: "no data at all" was the strongest possible ping evidence, because
    // rms()==0 forced ratio=0 which trivially clears kPingMaxDataToTrainingRMSRatio.
    //
    // Safe to change: evaluatePingRMS has NO production callers (verified by grep across
    // src/) — it is a test-only convenience wrapper. The single production entry point is
    // evaluatePingFrame at streaming_ofdm_decode.cpp:843.
    auto silent = evaluatePingRMS(nullptr, 0);
    CHECK(silent.buffer_invalid, "an empty buffer is invalid input, not a signal");
    CHECK(!silent.is_ping,
          "an empty buffer must NOT classify as a ping — nothing was measured");
}

void test_ping_chirp_lock_fallback() {
    std::vector<float> busy_band(kPingTrainingSkipSamples + kPingRMSCheckSamples, 1.0f);

    auto no_lock = evaluatePingFrame(
        busy_band.data(), busy_band.size(),
        kPingTrainingSkipSamples, kPingRMSCheckSamples,
        kPingCorrFloor - 0.001f, 83.0f,
        false, false);
    CHECK(!no_lock.is_ping, "chirp fallback should reject below correlation floor");

    auto wide_gap = evaluatePingFrame(
        busy_band.data(), busy_band.size(),
        kPingTrainingSkipSamples, kPingRMSCheckSamples,
        kPingCorrFloor, kPingMaxGapError + 1.0f,
        false, false);
    CHECK(!wide_gap.is_ping, "chirp fallback should reject loose dual-chirp gap");

    auto valid_frame = evaluatePingFrame(
        busy_band.data(), busy_band.size(),
        kPingTrainingSkipSamples, kPingRMSCheckSamples,
        kPingCorrFloor, 145.0f,
        true, true, true);
    CHECK(!valid_frame.is_ping, "valid LDPC frame should not classify as ping");

    auto full_ldpc_failure = evaluatePingFrame(
        busy_band.data(), busy_band.size(),
        kPingTrainingSkipSamples, kPingRMSCheckSamples,
        kPingCorrFloor, -145.0f,
        false, false, true);
    CHECK(!full_ldpc_failure.is_ping,
          "chirp-locked full-LDPC failure with data energy should stay a frame failure");
    CHECK(!full_ldpc_failure.ping_by_silence,
          "busy-band full-LDPC failure should not rely on RMS silence");
    CHECK(!full_ldpc_failure.ping_by_chirp_lock,
          "busy-band full-LDPC failure must not emit a false PING");

    auto low_snr_ping_after_llr_reject = evaluatePingFrame(
        busy_band.data(), busy_band.size(),
        kPingTrainingSkipSamples, kPingRMSCheckSamples,
        0.908f, 0.0f,
        false, false, false);
    CHECK(low_snr_ping_after_llr_reject.is_ping,
          "measured SNR15 chirp lock plus rejected data frame should classify as ping");
    CHECK(low_snr_ping_after_llr_reject.ping_by_chirp_lock,
          "measured SNR15 fallback should depend on chirp lock");
}

void test_false_lock_advance() {
    CHECK(falseOFDMLockAdvanceSamples(10000, 0) == kDefaultFalseLockAdvanceSamples,
          "missing preamble estimate should use default false-lock advance");
    CHECK(falseOFDMLockAdvanceSamples(10000, 800) == kMinFalseLockAdvanceSamples,
          "short preamble should respect minimum false-lock advance");
    CHECK(falseOFDMLockAdvanceSamples(10000, 3000) == 1500,
          "false-lock advance should use half data preamble when larger");
    CHECK(falseOFDMLockAdvanceSamples(700, 3000) == 700,
          "false-lock advance should not exceed copied frame length");
}

void test_control_first_peek_policy() {
    CHECK(shouldRunControlFirstOFDMPeek(0, true, true, 9000, 9000),
          "connected OFDM first pass within control size should peek");
    CHECK(!shouldRunControlFirstOFDMPeek(1, true, true, 9000, 9000),
          "pending codewords should not run control-first peek");
    CHECK(!shouldRunControlFirstOFDMPeek(0, false, true, 9000, 9000),
          "non-OFDM should not run OFDM control-first peek");
    CHECK(!shouldRunControlFirstOFDMPeek(0, true, false, 9000, 9000),
          "disconnected path should not run connected control-first peek");
    CHECK(!shouldRunControlFirstOFDMPeek(0, true, true, 9001, 9000),
          "full-frame sample count should not run control-first peek");
}

void test_sync_recovery_gate() {
    CHECK(!allowSyncRecovery(kMinSyncRecoveryCorrelation - 0.001f),
          "sync recovery should reject below threshold");
    CHECK(allowSyncRecovery(kMinSyncRecoveryCorrelation),
          "sync recovery should allow exact threshold");
    CHECK(allowSyncRecovery(kMinSyncRecoveryCorrelation + 0.1f),
          "sync recovery should allow strong correlation");
}

void test_reactive_anchor_announcement_threshold_crossing() {
    // Exact live IONOS failure: ordinal 12 is a FULL slot for K=2.  Crediting the
    // fourth clean delivery opens the reactive gate, so its descriptor must announce
    // that ordinal 13 will be LIGHT.  The old pre-credit calculation announced FULL.
    CHECK(shouldAnnounceNextLightAnchor(/*anchor_ordinal=*/12, /*anchor_skip_k=*/2,
                                        /*clean_streak_after_credit=*/4,
                                        /*clean_streak_threshold=*/4),
          "threshold-crossing descriptor must announce the next LIGHT anchor");
    CHECK(!shouldAnnounceNextLightAnchor(/*anchor_ordinal=*/12, /*anchor_skip_k=*/2,
                                         /*clean_streak_after_credit=*/3,
                                         /*clean_streak_threshold=*/4),
          "descriptor below the reactive threshold must continue announcing FULL");
    CHECK(!shouldAnnounceNextLightAnchor(/*anchor_ordinal=*/13, /*anchor_skip_k=*/2,
                                         /*clean_streak_after_credit=*/5,
                                         /*clean_streak_threshold=*/4),
          "LIGHT slot must announce that the next periodic slot is FULL");
    CHECK(!shouldAnnounceNextLightAnchor(/*anchor_ordinal=*/12, /*anchor_skip_k=*/1,
                                         /*clean_streak_after_credit=*/100,
                                         /*clean_streak_threshold=*/4),
          "K=1 opt-out must never announce a LIGHT anchor");
}

void test_burst_descriptor_anchor_flags_match_current_group() {
    namespace v2 = ultra::protocol::v2;

    const uint8_t light = burstDescriptorFlags(
        /*burst_interleave=*/false, /*carrier_ldpc=*/true,
        /*next_light_anchor=*/false, /*current_group_full_anchor=*/false);
    CHECK((light & v2::ControlFrame::BURST_FLAG_CARRIER_LDPC) != 0,
          "carrier-LDPC descriptor bit should remain independent");
    CHECK((light & v2::ControlFrame::BURST_FLAG_CURRENT_GROUP_FULL_ANCHOR) == 0,
          "steady warm group must announce a light current DATA anchor");

    const uint8_t repair = burstDescriptorFlags(
        /*burst_interleave=*/true, /*carrier_ldpc=*/false,
        /*next_light_anchor=*/true, /*current_group_full_anchor=*/true);
    CHECK((repair & v2::ControlFrame::BURST_FLAG_INTERLEAVE) != 0 &&
              (repair & v2::ControlFrame::BURST_FLAG_NEXT_LIGHT_ANCHOR) != 0,
          "existing descriptor flag meanings must be preserved");
    CHECK((repair & v2::ControlFrame::BURST_FLAG_CURRENT_GROUP_FULL_ANCHOR) != 0,
          "timeout repair must announce its full current DATA anchor");
}

void test_non_data_frame_detection() {
    std::vector<uint8_t> too_short = {0x55, 0x4c};
    CHECK(!isNonDataFrame(true, too_short.data(), too_short.size()),
          "short frame should not classify as non-data");
    CHECK(!isNonDataFrame(false, nullptr, 0),
          "failed decode should not classify as non-data");

    std::vector<uint8_t> ack = {0x55, 0x4c,
        static_cast<uint8_t>(ultra::protocol::v2::FrameType::ACK)};
    CHECK(isNonDataFrame(true, ack.data(), ack.size()), "ACK should classify as non-data");

    std::vector<uint8_t> connect = {0x55, 0x4c,
        static_cast<uint8_t>(ultra::protocol::v2::FrameType::CONNECT)};
    CHECK(isNonDataFrame(true, connect.data(), connect.size()),
          "CONNECT should classify as non-data");

    std::vector<uint8_t> data = {0x55, 0x4c,
        static_cast<uint8_t>(ultra::protocol::v2::FrameType::DATA)};
    CHECK(!isNonDataFrame(true, data.data(), data.size()), "DATA should not classify as non-data");
}

void test_consumed_samples_policy() {
    CHECK(consumedSamplesForDecodedFrame(true, true, true, 1, 12000, 4000) == 4000,
          "successful OFDM non-data frame should use exact CW sample count");
    CHECK(consumedSamplesForDecodedFrame(true, true, true, 1, 12000, 14000) == 12000,
          "exact count should not extend consumed samples");
    CHECK(consumedSamplesForDecodedFrame(true, true, true, 0, 12000, 4000) == 12000,
          "zero codewords should keep copied frame length");
    CHECK(consumedSamplesForDecodedFrame(true, false, true, 1, 12000, 4000) == 12000,
          "non-OFDM path should keep copied frame length");
    CHECK(consumedSamplesForDecodedFrame(true, true, false, 1, 12000, 4000) == 12000,
          "data frame should keep copied frame length");
    CHECK(consumedSamplesForDecodedFrame(false, true, true, 1, 12000, 4000) == 12000,
          "failed decode should keep copied frame length");
}

void test_provisional_burst_air_gate_policy() {
    constexpr uint64_t kAnchor = 48000;
    constexpr uint64_t kLaterLightFrame = kAnchor + 59360;

    CHECK(shouldClearProvisionalBurstAirGate(
              true, false, true, true, kAnchor, kAnchor),
          "non-FINAL singleton decoded at the accepted full anchor should clear");
    CHECK(!shouldClearProvisionalBurstAirGate(
               true, false, true, true, kAnchor, kLaterLightFrame),
          "later light DATA after a lost anchored head must retain the air gate");
    CHECK(!shouldClearProvisionalBurstAirGate(
               true, false, true, true, kAnchor, kLaterLightFrame),
          "logical FINAL on a later group member must retain the physical air gate");
    CHECK(!shouldClearProvisionalBurstAirGate(
               true, false, true, false, kAnchor, kAnchor),
          "DATA-sync fallback at its own provisional anchor is not singleton proof");
    CHECK(shouldClearProvisionalBurstAirGate(
              true, true, true, false, kAnchor, kLaterLightFrame),
          "standalone control should clear even on a later sync");
    CHECK(!shouldClearProvisionalBurstAirGate(
               false, false, true, true, kAnchor, kAnchor),
          "failed decode cannot prove a standalone tail");
    CHECK(!shouldClearProvisionalBurstAirGate(
               true, false, false, true, kAnchor, kAnchor),
          "an exact group gate must not be cleared by provisional policy");
}

void test_physical_burst_end_wire_policy() {
    namespace v2 = ultra::protocol::v2;

    CHECK(shouldStampPhysicalBurstEnd(
              2, ultra::protocol::WaveformMode::OFDM_CHIRP,
              /*supports_data_preamble=*/true, /*burst_interleave=*/false),
          "multi-frame non-interleaved OFDM should stamp a physical tail");
    CHECK(!shouldStampPhysicalBurstEnd(
               1, ultra::protocol::WaveformMode::OFDM_CHIRP, true, false),
          "singleton keeps its existing full-anchor physical proof");
    CHECK(!shouldStampPhysicalBurstEnd(
               2, ultra::protocol::WaveformMode::OFDM_CHIRP, true, true),
          "cross-frame interleave must keep the descriptor/group callback boundary");
    // BUG-MCDPSK-ACK-DROPPED: MC-DPSK marks EVERY physical DATA turn, singletons included.
    // Its only other end-of-turn signal (no continuation header one codeword later) comes
    // after the last frame's ACK was requested, which then could only be dropped.
    CHECK(shouldStampPhysicalBurstEnd(
              2, ultra::protocol::WaveformMode::MC_DPSK, true, false),
          "a multi-frame MC-DPSK turn stamps its last frame");
    CHECK(shouldStampPhysicalBurstEnd(
              1, ultra::protocol::WaveformMode::MC_DPSK, false, false),
          "an MC-DPSK singleton is a whole physical turn and stamps too");

    auto first = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 10, ultra::Bytes{0x10}, ultra::CodeRate::R1_2, 4);
    auto last = v2::makeFixedDataFrame(
        "ALPHA", "BRAVO", 11, ultra::Bytes{0x11}, ultra::CodeRate::R1_2, 4);
    // Simulate a stale upstream bit: encoder sanitation must own physical status.
    first.flags |= v2::Flags::PHYSICAL_BURST_END;
    const std::vector<ultra::Bytes> logical{first.serialize(), last.serialize()};
    const auto wire = preparePhysicalBurstFrames(logical, /*stamp_physical_end=*/true);

    CHECK(logical[0] == first.serialize(),
          "wire stamping must not mutate the ARQ-owned logical frame");
    auto wire_first = v2::DataFrame::deserialize(wire[0]);
    auto wire_last = v2::DataFrame::deserialize(wire[1]);
    CHECK(wire_first && wire_last, "stamped wire frames must retain valid CRCs");
    CHECK((wire_first->flags & v2::Flags::PHYSICAL_BURST_END) == 0,
          "stale marker must be cleared from every non-tail frame");
    CHECK((wire_last->flags & v2::Flags::PHYSICAL_BURST_END) != 0,
          "only the exact final wire frame carries physical-end proof");
    CHECK(hasPhysicalBurstEndMarker(
              true, true, true, true, wire[1]),
          "a valid connected OFDM burst tail should be recognized");
    CHECK(!hasPhysicalBurstEndMarker(
               true, true, true, true, wire[0]),
          "an earlier member must not be recognized as a tail");
    CHECK(!hasPhysicalBurstEndMarker(
               true, false, true, true, wire[1]),
          "disconnected traffic cannot authorize burst-tail reverse egress");
    // MC-DPSK receiver: the same CRC-valid marker proves the peer's turn ended.
    CHECK(hasMcDpskTurnEndMarker(true, true, /*is_mc_dpsk=*/true, wire[1]),
          "a marked MC-DPSK DATA frame proves the turn is complete");
    CHECK(!hasMcDpskTurnEndMarker(true, true, true, wire[0]),
          "an unmarked MC-DPSK member is not a turn end");
    CHECK(!hasMcDpskTurnEndMarker(false, true, true, wire[1]),
          "a failed decode proves nothing");
    CHECK(!hasMcDpskTurnEndMarker(true, true, /*is_mc_dpsk=*/false, wire[1]),
          "the MC-DPSK rule never applies to OFDM traffic");
    CHECK(!hasMcDpskTurnEndMarker(true, false, true, wire[1]),
          "disconnected traffic cannot authorize reverse egress");

    const auto sanitized =
        preparePhysicalBurstFrames(logical, /*stamp_physical_end=*/false);
    auto sanitized_first = v2::DataFrame::deserialize(sanitized[0]);
    auto sanitized_last = v2::DataFrame::deserialize(sanitized[1]);
    CHECK(sanitized_first && sanitized_last,
          "non-marker sanitation must retain valid DATA frames");
    CHECK(((sanitized_first->flags | sanitized_last->flags) &
           v2::Flags::PHYSICAL_BURST_END) == 0,
          "non-eligible transports must clear all stale physical markers");
}

}  // namespace


// ── BUG-SYNC-CURSOR-AHEAD: an unwritten slice must never classify as a PING ────
// Reproduces the exact rig failure of 2026-07-30 (Pi 5, IONOS). The sync cursor ran past
// the write head, the decoder sliced ring memory that reset() had zero-filled, and the
// classifier read the resulting digital silence as the STRONGEST POSSIBLE ping evidence:
//
//   [PI5] PING detected: path1=1 path2=0 ratio=0.000 chirp_corr=0.837 ldpc_ok=skipped
//
// That fabricated frame then set a search floor that walled off the genuine CONNECT_ACK
// for 2.5 s, and the connection deadlocked for 420 s.
//
// A live receiver cannot produce ~0.0 RMS: the measured floor on that radio was
// 0.009-0.011, an order of magnitude above kMinTrainingRMSForPingRatio (0.001), and even a
// disconnected input carries dither. Digital silence in the TRAINING window can only mean
// unwritten memory.
void test_unwritten_slice_is_not_a_ping() {
    const size_t n = kPingTrainingSkipSamples + kPingRMSCheckSamples;
    std::vector<float> zeros(n, 0.0f);

    // The rig's exact conditions: real chirp lock, no LDPC attempt, zero-filled slice.
    auto d = evaluatePingFrame(zeros.data(), zeros.size(),
                               kPingTrainingSkipSamples, kPingRMSCheckSamples,
                               /*chirp_corr=*/0.837f, /*gap_error_samples=*/-27.0f,
                               /*ldpc_decode_succeeded=*/false, /*ldpc_magic_valid=*/false,
                               /*ldpc_decode_attempted=*/false);
    CHECK(d.buffer_invalid, "a zero-filled training window must be flagged invalid");
    CHECK(!d.ping_by_silence, "an invalid buffer must not satisfy ping-by-silence");
    CHECK(!d.ping_by_chirp_lock,
          "an invalid buffer must not satisfy ping-by-chirp-lock either — the chirp was "
          "real (0.837) and zero data_rms trivially clears kPingChirpLockMaxDataRMS, so "
          "suppressing only the silence path would leave this route open");
    CHECK(!d.is_ping, "an unwritten slice must yield NO ping verdict");

    // A GENUINE ping at the measured live noise floor must still be detected. This is the
    // regression guard for the low-SNR PING path (#70): the reject keys on TRAINING rms
    // (physically impossible near zero), never on data_rms (a real silent gap reads ~0).
    std::vector<float> real(n, 0.0f);
    for (size_t i = 0; i < kPingTrainingSkipSamples; ++i) {
        real[i] = (i % 2 == 0) ? 0.05f : -0.05f;   // training present, well above the floor
    }
    for (size_t i = kPingTrainingSkipSamples; i < n; ++i) {
        real[i] = (i % 2 == 0) ? 0.010f : -0.010f; // payload area = ambient noise floor
    }
    auto g = evaluatePingFrame(real.data(), real.size(),
                               kPingTrainingSkipSamples, kPingRMSCheckSamples,
                               /*chirp_corr=*/0.837f, /*gap_error_samples=*/-27.0f,
                               false, false, false);
    CHECK(!g.buffer_invalid, "a real training window is not flagged invalid");
    CHECK(g.is_ping, "a genuine low-level PING must still be detected");

    // Boundary: exactly at the threshold counts as invalid (<=), just above does not.
    std::vector<float> at(n, kMinTrainingRMSForPingRatio);
    auto a = evaluatePingFrame(at.data(), at.size(), kPingTrainingSkipSamples,
                               kPingRMSCheckSamples, 0.837f, -27.0f, false, false, false);
    CHECK(a.buffer_invalid, "training rms exactly at the floor is invalid");
    std::vector<float> above(n, kMinTrainingRMSForPingRatio * 4.0f);
    auto b = evaluatePingFrame(above.data(), above.size(), kPingTrainingSkipSamples,
                               kPingRMSCheckSamples, 0.837f, -27.0f, false, false, false);
    CHECK(!b.buffer_invalid, "training rms above the floor is valid");
}

int main() {
    test_rms_and_ping_detection();
    test_ping_chirp_lock_fallback();
    test_false_lock_advance();
    test_control_first_peek_policy();
    test_sync_recovery_gate();
    test_reactive_anchor_announcement_threshold_crossing();
    test_burst_descriptor_anchor_flags_match_current_group();
    test_non_data_frame_detection();
    test_consumed_samples_policy();
    test_provisional_burst_air_gate_policy();
    test_physical_burst_end_wire_policy();
    test_unwritten_slice_is_not_a_ping();

    if (tests_failed != 0) {
        std::cout << "StreamingFramePolicy: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "StreamingFramePolicy: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
