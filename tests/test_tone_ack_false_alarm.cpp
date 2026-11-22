// Tone-ACK candidate floor from the false-ACK budget (tone_ack_false_alarm.hpp, 2026-11-22).
// Pins: the rule reproduces the old single floor at the widest span, lowers it as the protocol
// predicate gets more selective, never leaves the measured range, keeps the old floor outside the
// measured conditions (wide carrier search, no diversity), and the monitor applies it per pass
// from the in-flight span provider.
#include "waveform/tone_burst_ack/tone_ack_false_alarm.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace ultra::waveform::tone_burst_ack;

static int failures = 0;
#define EXPECT(cond)                                                             \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

static void test_predicate_probability_geometry() {
    std::printf("[test] predicate pass probability from field widths\n");
    // span 0: ACK voice needs group_seq == base-1 and frame_mask == 0.
    const double rebase = 0.5 * 0.25 * 0.125 * 0.25 * std::ldexp(1.0, -16);
    EXPECT(std::fabs(toneAckPredicatePassProbability(0) - (0.5 / 64.0 * std::ldexp(1.0, -16) + rebase)) < 1e-15);
    // Monotone in span; the full 16-frame window passes ~1/64 of uniform payloads.
    for (size_t s = 1; s <= 20; ++s)
        EXPECT(toneAckPredicatePassProbability(s) > toneAckPredicatePassProbability(s - 1));
    EXPECT(std::fabs(toneAckPredicatePassProbability(16) - 0.5 * (std::ldexp(1.0, 17) - 1) / 64.0 / 65536.0 - rebase) < 1e-12);
}

static void test_floor_rule() {
    std::printf("[test] floor follows the in-flight span inside the budget\n");
    const float f1 = toneAckNoiseRatioFloorFor(1, 4.8f, true);
    const float f3 = toneAckNoiseRatioFloorFor(3, 4.8f, true);
    const float f5 = toneAckNoiseRatioFloorFor(5, 4.8f, true);
    const float f8 = toneAckNoiseRatioFloorFor(8, 4.8f, true);
    const float f16 = toneAckNoiseRatioFloorFor(16, 4.8f, true);
    std::printf("  span 1 %.3f  3 %.3f  5 %.3f  8 %.3f  16 %.3f\n", f1, f3, f5, f8, f16);
    EXPECT(f1 == kToneAckH0FloorRef);                 // never below the measured range
    EXPECT(f16 == kCostasNoiseRatioFloor);            // the widest window keeps the old floor
    EXPECT(std::fabs(f5 - 1.72f) < 0.02f);
    EXPECT(std::fabs(f8 - 1.90f) < 0.02f);
    EXPECT(f1 <= f3 && f3 <= f5 && f5 <= f8 && f8 <= f16);
    // SIZE_MAX (never armed) = widest.
    EXPECT(toneAckNoiseRatioFloorFor(SIZE_MAX, 4.8f, true) == kCostasNoiseRatioFloor);
    // Accepted false ACKs per armed hour at the returned floor stay inside the budget wherever
    // the floor is above the measured reference.
    for (size_t s = 0; s <= 16; ++s) {
        const float f = toneAckNoiseRatioFloorFor(s, 0.0f, true);
        const double rate = kToneAckH0CrcRatePerHourAtRef *
                            std::exp(-kToneAckH0RateSlopePerUnit * (f - kToneAckH0FloorRef)) *
                            toneAckPredicatePassProbability(s);
        if (f < kCostasNoiseRatioFloor) EXPECT(rate <= kToneAckFalseAckBudgetPerArmedHour * 1.0001);
    }
}

static void test_unmeasured_conditions_keep_old_floor() {
    std::printf("[test] wide search / no diversity keep kCostasNoiseRatioFloor\n");
    EXPECT(toneAckNoiseRatioFloorFor(1, 50.0f, true) == kCostasNoiseRatioFloor);
    EXPECT(toneAckNoiseRatioFloorFor(1, 4.8f, false) == kCostasNoiseRatioFloor);
    EXPECT(toneAckNoiseRatioFloorFor(1, 0.0f, true) == kToneAckH0FloorRef);
}

static void test_monitor_applies_provider() {
    std::printf("[test] monitor sets the floor per pass from the span provider\n");
    ToneBurstAckMonitor mon;
    size_t span = 1;
    mon.setInFlightSpanProvider([&]() { return span; });
    mon.setFrequencyPrior(0.0f, 4.8f);
    std::vector<float> silence(48000, 0.0f);
    mon.feedAudio(silence.data(), silence.size());
    EXPECT(mon.lastNoiseRatioFloor() == toneAckNoiseRatioFloorFor(1, 4.8f, mon.diversity()));
    span = 16;
    mon.feedAudio(silence.data(), silence.size());
    EXPECT(mon.lastNoiseRatioFloor() == kCostasNoiseRatioFloor);
    span = 1;
    mon.setFrequencyPrior(0.0f, 50.0f);
    mon.feedAudio(silence.data(), silence.size());
    EXPECT(mon.lastNoiseRatioFloor() == kCostasNoiseRatioFloor);
    ToneBurstAckMonitor plain;  // no provider: the old floor
    plain.feedAudio(silence.data(), silence.size());
    EXPECT(plain.lastNoiseRatioFloor() == kCostasNoiseRatioFloor);
}

int main() {
    test_predicate_probability_geometry();
    test_floor_rule();
    test_unmeasured_conditions_keep_old_floor();
    test_monitor_applies_provider();
    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
