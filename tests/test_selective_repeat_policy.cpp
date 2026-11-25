#include "protocol/selective_repeat_arq_policy.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

using namespace ultra::protocol::selective_repeat_arq_policy;

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

void test_window_and_batch_clamps() {
    CHECK(clampWindowSize(0, 16) == 1, "window should clamp to minimum 1");
    CHECK(clampWindowSize(4, 16) == 4, "window should preserve valid value");
    CHECK(clampWindowSize(99, 16) == 16, "window should clamp to maximum");

    CHECK(clampAckBatchSize(0, 4, 16) == 0, "ack batch sentinel should be preserved");
    CHECK(clampAckBatchSize(2, 4, 16) == 2, "valid ack batch should be preserved");
    CHECK(clampAckBatchSize(8, 4, 16) == 4, "ack batch should clamp to window");
    CHECK(clampAckBatchSize(99, 32, 16) == 16, "ack batch should clamp to max window");
    CHECK(effectiveAckBatchThreshold(0, 4) == 4, "sentinel batch should track window");
    CHECK(effectiveAckBatchThreshold(2, 4) == 2, "explicit batch should be used");
}

void test_sack_bitmap_decode() {
    uint8_t payload[6] = {};
    ultra::protocol::v2::NackPayload np;
    np.frame_seq = 123;
    np.cw_bitmap = 0x00000400u;
    np.encode(payload);
    CHECK(decodeSackBitmap(payload) == 0x00000400u,
          "wide SACK bitmap should preserve bits beyond legacy byte");

    uint8_t legacy_payload[6] = {};
    legacy_payload[2] = 0x05;
    legacy_payload[5] = 0x05;
    CHECK(decodeSackBitmap(legacy_payload) == 0x05u,
          "legacy repeated 8-bit SACK shape should decode to legacy byte");

    CHECK(decodeSackBitmap(nullptr) == 0u, "null SACK payload should be safe");
}

void test_sequence_window_policy() {
    CHECK(seqInWindow(10, 10, 4), "base sequence should be in window");
    CHECK(seqInWindow(13, 10, 4), "last sequence in window");
    CHECK(!seqInWindow(14, 10, 4), "first sequence outside window");
    CHECK(seqInWindow(0, 65534, 4), "sequence window should wrap across 65535");
    CHECK(!seqInWindow(2, 65534, 4), "wrapped sequence outside window");
}

void test_ack_freshness() {
    CHECK(classifyAckFreshness(9, 10, 4) == AckFreshness::Accept,
          "base-1 ACK should be accepted");
    CHECK(classifyAckFreshness(8, 10, 4) == AckFreshness::Stale,
          "ACK older than base-1 should be stale");
    CHECK(classifyAckFreshness(14, 10, 4) == AckFreshness::Accept,
          "ACK within window+1 should be accepted");
    CHECK(classifyAckFreshness(15, 10, 4) == AckFreshness::Future,
          "ACK too far ahead should be future");
    CHECK(classifyAckFreshness(65535, 0, 4) == AckFreshness::Accept,
          "wrapped base-1 ACK should be accepted");
    CHECK(classifyAckFreshness(65534, 0, 4) == AckFreshness::Stale,
          "wrapped stale ACK should be rejected");
    CHECK(classifyAckFreshness(1, 65534, 4) == AckFreshness::Accept,
          "near-wrap ACK through the wrapped TX window should be accepted");
    CHECK(classifyAckFreshness(3, 65534, 4) == AckFreshness::Future,
          "near-wrap ACK beyond window+1 should be future");
    CHECK(classifyAckFreshness(65532, 65534, 4) == AckFreshness::Stale,
          "near-wrap ACK older than base-1 should be stale");
}

void test_ack_dedup_policy() {
    CHECK(!shouldSuppressDuplicateAck(false, 100, 1, 0x10, 1, 0x10),
          "missing prior signature should not suppress");
    CHECK(!shouldSuppressDuplicateAck(true, 0, 1, 0x10, 1, 0x10),
          "expired dedup timer should not suppress");
    CHECK(shouldSuppressDuplicateAck(true, 100, 1, 0x10, 1, 0x10),
          "matching ACK signature within timer should suppress");
    CHECK(!shouldSuppressDuplicateAck(true, 100, 1, 0x10, 2, 0x10),
          "different ACK seq should not suppress");
    CHECK(!shouldSuppressDuplicateAck(true, 100, 1, 0x10, 1, 0x20),
          "different ACK bitmap should not suppress");

    CHECK(ackDedupWindowMs(1) == 80, "dedup window should clamp low");
    CHECK(ackDedupWindowMs(100) == 140, "dedup window should add guard time");
    CHECK(ackDedupWindowMs(1000) == 500, "dedup window should clamp high");
}

void test_sack_timer_policy() {
    CHECK(sackTimerForFrame(0, 500, 0, false) == 500,
          "zero short sentinel should use normal delay");
    CHECK(sackTimerForFrame(0, 500, 50, false) == 500,
          "regular frame should use long delay");
    CHECK(sackTimerForFrame(0, 500, 50, true) == 50,
          "explicit tail frame should use short delay");
    CHECK(sackTimerForFrame(440, 500, 50, true) == 50,
          "explicit tail frame should collapse existing long timer");
    CHECK(sackTimerForFrame(50, 500, 0, true) == 50,
          "new in-burst frame should not extend existing timer");
}

void test_hole_and_repeat_policy() {
    CHECK(isAlignedBaseHoleAck(9, 10, 0x02),
          "aligned SACK with higher bit and missing bit0 should indicate base hole");
    CHECK(!isAlignedBaseHoleAck(10, 10, 0x02),
          "unaligned SACK should not indicate base hole");
    CHECK(!isAlignedBaseHoleAck(9, 10, 0x01),
          "bit0 present should not indicate base hole");
    CHECK(!isAlignedBaseHoleAck(9, 10, 0x00),
          "empty bitmap should not indicate base hole");

    CHECK(holeProbeInitialTimerMs(1000) == 1200, "hole probe initial timer low clamp");
    CHECK(holeProbeInitialTimerMs(4000) == 2000, "hole probe initial timer midpoint");
    CHECK(holeProbeInitialTimerMs(8000) == 2500, "hole probe initial timer high clamp");
    CHECK(holeProbeNextTimerMs(2000) == 1800, "hole probe next timer low clamp");
    CHECK(holeProbeNextTimerMs(3900) == 2600, "hole probe next timer midpoint");
    CHECK(holeProbeNextTimerMs(6000) == 3000, "hole probe next timer high clamp");
    CHECK(fastRetransmitCooldownMs(1000) == 300, "fast retx cooldown low clamp");
    CHECK(fastRetransmitCooldownMs(3600) == 600, "fast retx cooldown midpoint");
    CHECK(fastRetransmitCooldownMs(9000) == 1200, "fast retx cooldown high clamp");

    CHECK(!shouldFastRetransmitHole(1, 0, 0),
          "single hole confirmation should not fast retransmit");
    CHECK(shouldFastRetransmitHole(2, 0, 0),
          "two hole confirmations should fast retransmit");
    CHECK(!shouldFastRetransmitHole(2, 1, 0),
          "fast retransmit count should cap repeats");
    CHECK(!shouldFastRetransmitHole(2, 0, 1),
          "cooldown should block fast retransmit");
}

void test_ack_repeat_policy() {
    CHECK(ackRepeatDelayForCopy(100, 2) == 100,
          "first delayed ACK repeat should use configured delay");
    CHECK(ackRepeatDelayForCopy(100, 3) == 300,
          "later ACK repeat should use wider spacing");
    CHECK(ackRepeatDelayWithHalfDuplexGuard(100, 5000, false) == 100,
          "unguarded ACK repeat should stay prompt");
    CHECK(ackRepeatDelayWithHalfDuplexGuard(100, 5000, true) == 5100,
          "guarded ACK repeat should wait past peer burst guard");

    const int jitter = ackRepeatJitterMs(7, 0x1234, 2);
    CHECK(jitter >= -30 && jitter <= 30, "ACK repeat jitter should stay bounded");
    CHECK(ackRepeatJitterMs(7, 0x1234, 2) == jitter,
          "ACK repeat jitter should be deterministic");
}

void test_rtt_rto_policy() {
    CHECK(rttSampleMs(1000, 900) == 100, "RTT sample should be elapsed time");
    CHECK(rttSampleMs(900, 1000) == 0, "RTT sample should reject time reversal");
    CHECK(rttSampleMs(70000, 0) == 60000, "RTT sample should clamp very large values");

    CHECK(!shouldUseRTTSample(false, 1000, 900),
          "ineligible slot should not update RTT estimator");
    CHECK(!shouldUseRTTSample(true, 900, 1000),
          "time reversal should not update RTT estimator");
    CHECK(!shouldUseRTTSample(true, 1049, 1000),
          "tiny RTT sample should not update estimator");
    CHECK(shouldUseRTTSample(true, 1050, 1000),
          "minimum RTT sample should update estimator");

    auto first = updateRTO(false, 0.0f, 0.0f, 600, 8000);
    CHECK(std::abs(first.srtt_ms - 600.0f) < 0.001f, "first SRTT sample");
    CHECK(std::abs(first.rttvar_ms - 300.0f) < 0.001f, "first RTTVAR sample");
    CHECK(first.floor_ms == 8000, "configured ACK timeout should floor RTO");
    CHECK(first.ceiling_ms == 12000, "RTO ceiling should use minimum ceiling");
    CHECK(first.rto_ms == 8000, "RTO should clamp up to configured floor");

    auto next = updateRTO(true, 600.0f, 300.0f, 1000, 8000);
    CHECK(std::abs(next.srtt_ms - 650.0f) < 0.001f, "subsequent SRTT EWMA");
    CHECK(std::abs(next.rttvar_ms - 325.0f) < 0.001f, "subsequent RTTVAR EWMA");
    CHECK(next.rto_ms == 8000, "configured ACK timeout should still floor low RTO");

    auto high_config = updateRTO(false, 0.0f, 0.0f, 2000, 15000);
    CHECK(high_config.floor_ms == 15000, "high configured timeout should floor RTO");
    CHECK(high_config.ceiling_ms == 15000, "high configured timeout should also raise ceiling");
    CHECK(high_config.rto_ms == 15000, "high configured timeout should dominate RTO");

    auto ceiling = updateRTO(true, 1000.0f, 10000.0f, 1000, 8000);
    CHECK(ceiling.rto_ms == 12000, "large RTTVAR should clamp to RTO ceiling");

}

}  // namespace

int main() {
    test_window_and_batch_clamps();
    test_sack_bitmap_decode();
    test_sequence_window_policy();
    test_ack_freshness();
    test_ack_dedup_policy();
    test_sack_timer_policy();
    test_hole_and_repeat_policy();
    test_ack_repeat_policy();
    test_rtt_rto_policy();

    if (tests_failed != 0) {
        std::cout << "SelectiveRepeatPolicy: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "SelectiveRepeatPolicy: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
