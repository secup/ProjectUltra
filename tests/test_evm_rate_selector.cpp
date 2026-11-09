// Tests for the effective-SNR rate selector (ULTRA_RATE_EVM, protocol/evm_rate_selector.hpp).
//
// Properties under test — the ones the outcome-only latent controller lacks:
//   1. Margin is visible from ANY rung: frames received on QPSK R1/2 predict 16QAM R2/3.
//   2. A fade is rung-independent: faded frames lower every rung's predicted success, so a
//      deep fade does not make a lower rung look like an escape.
//   3. The outer-loop margin moves on calibration error only: a loss the frames' own EVMs
//      predicted leaves it unchanged; a loss they did not predict raises it.
//   4. Window and readiness: unmeasured frames are skipped, the window holds kWindowFrames.

#include "protocol/connection_policy.hpp"
#include "protocol/evm_rate_selector.hpp"
#include "protocol/file_transfer.hpp"
#include "protocol/frame_v2.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

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

LatentRateController::RungGeometryTable productionGeometry() {
    LatentRateController::RungGeometryTable table{};
    for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
        const auto candidate = coherentRungFromIndex(r);
        if (!coherentRungLocallyEnabled(candidate.mod, candidate.rate)) continue;
        const int cw = connection_policy::recommendCWCount(
            candidate.mod, candidate.rate, WaveformMode::OFDM_CHIRP);
        const size_t capacity = v2::getFixedFramePayloadCapacityZ(candidate.rate, cw, 27);
        if (capacity <= FileTransferController::FILE_DATA_OVERHEAD) continue;
        const uint32_t burst_ms = connection_policy::wideOFDMBurstAirtimeMs(
            candidate.mod, candidate.rate, 5, cw, 0, 27);
        table[r] = {static_cast<float>(capacity - FileTransferController::FILE_DATA_OVERHEAD),
                    5, static_cast<float>(burst_ms + 1790) / 1000.0f};
    }
    return table;
}

std::vector<float> frames(float evm_db, int n) { return std::vector<float>(n, evm_db); }

void test_margin_is_visible_from_any_rung() {
    // Clean frames received on QPSK R1/2: an outcome-only estimator learns nothing above
    // R1/2's saturation, but their EVM says 16QAM R2/3 would decode them.
    EvmRateSelector sel;
    sel.observeGroup(kRungIdxQpskR12, frames(20.0f, 5), 5, 5);
    CHECK(sel.ready(), "one five-frame group is enough to decide");
    CHECK(sel.successProb(kRungIdxQam16R23) > 0.99f,
          "20 dB EVM frames predict 16QAM R2/3 decodes them");
    const auto pick = sel.best(productionGeometry());
    CHECK(pick.rung == kRungIdxQam16R23,
          "AWGN-like 20 dB frames select 16QAM R2/3 directly from R1/2 (got "
              << static_cast<int>(pick.rung) << ")");

    // Frames at 8 dB: QPSK decodes them, 8PSK R2/3 (eta 9.5) does not.
    EvmRateSelector mid;
    mid.observeGroup(kRungIdxQpskR23, frames(8.0f, 5), 5, 5);
    CHECK(mid.successProb(kRungIdxQpskR23) > 0.95f, "8 dB frames decode on QPSK R2/3");
    CHECK(mid.successProb(kRungIdxQam8R23) < 0.05f, "8 dB frames fail on 8PSK R2/3");
    const auto p8 = mid.best(productionGeometry());
    CHECK(p8.rung >= kRungIdxQpskR23 && p8.rung <= kRungIdxQpskR34,
          "8 dB frames select a QPSK R2/3-R3/4 rung (got " << static_cast<int>(p8.rung) << ")");
}

void test_fade_is_rung_independent() {
    // Window: 15 frames at 12 dB and 5 frames in a deep fade (2 dB). The fade costs every
    // rung the same 5/20 — QPSK R2/3 is no escape from it.
    EvmRateSelector sel;
    for (int g = 0; g < 3; ++g) sel.observeGroup(kRungIdxQam8R23, frames(12.0f, 5), 5, 5);
    sel.observeGroup(kRungIdxQam8R23, frames(2.0f, 5), 0, 5);
    const float p23 = sel.successProb(kRungIdxQpskR23);
    const float p8 = sel.successProb(kRungIdxQam8R23);
    CHECK(std::fabs(p23 - 0.75f) < 0.02f && std::fabs(p8 - 0.75f) < 0.03f,
          "a deep fade costs QPSK R2/3 and 8PSK R2/3 the same fraction (" << p23 << ", " << p8
                                                                         << ")");
    CHECK(sel.best(productionGeometry()).rung == kRungIdxQam8R23,
          "so the selector keeps the denser rung through it");
}

void test_margin_moves_on_calibration_error_only() {
    EvmRateSelector fade;
    fade.observeGroup(kRungIdxQam8R23, frames(4.0f, 5), 0, 5);   // predicted 0, delivered 0
    CHECK(std::fabs(fade.marginDb()) < 0.05f,
          "a loss its own frames' EVM predicted leaves the margin unchanged ("
              << fade.marginDb() << ")");

    EvmRateSelector miscal;
    miscal.observeGroup(kRungIdxQam8R23, frames(13.0f, 5), 0, 5);  // predicted 5, delivered 0
    CHECK(miscal.marginDb() > 1.9f,
          "an unpredicted crater raises the margin by the full step (" << miscal.marginDb()
                                                                      << ")");
    // A raised margin lowers every rung's success at the same EVM.
    EvmRateSelector clean;
    clean.observeGroup(kRungIdxQam8R23, frames(13.0f, 5), 5, 5);
    CHECK(miscal.successProb(kRungIdxQam16R23) < clean.successProb(kRungIdxQam16R23),
          "the margin is common-mode: it makes the denser rungs harder to choose");

    // Bounded: no amount of evidence pushes it past its clamp.
    for (int i = 0; i < 50; ++i) miscal.observeGroup(kRungIdxQam8R23, frames(20.0f, 5), 0, 5);
    CHECK(miscal.marginDb() <= EvmRateSelector::kMarginMaxDb + 1e-6f, "margin is clamped");
}

void test_window_and_unmeasured_frames() {
    EvmRateSelector sel;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    sel.observeGroup(kRungIdxQpskR23, {nan, nan, 10.0f}, 3, 3);
    CHECK(sel.frames() == 1 && !sel.ready(), "unmeasured frames are skipped, not counted");
    CHECK(std::fabs(sel.marginDb()) < 1e-6f,
          "a partly unmeasured group does not calibrate the margin");
    for (int g = 0; g < 10; ++g) sel.observeGroup(kRungIdxQpskR23, frames(15.0f, 5), 5, 5);
    CHECK(sel.frames() == EvmRateSelector::kWindowFrames, "the window holds kWindowFrames");
    sel.reset();
    CHECK(sel.frames() == 0 && !sel.ready() && sel.marginDb() == 0.0f, "reset clears all");
}

void test_time_to_finish_objective() {
    // Exact when every frame decodes: ceil(n/N) rounds, the last one short.
    const float full = 8.0f, frame = 1.2f;
    const float t10 = EvmRateSelector::expectedTimeToDeliver(10, 4, 1.0f, full, frame);
    const float expect10 = 2.0f * full + (full - 2.0f * frame);   // 4 + 4 + 2 frames
    CHECK(std::fabs(t10 - expect10) < 1e-3f,
          "p=1: two full rounds and a two-frame round (" << t10 << " vs " << expect10 << ")");
    // Losses cost time, monotonically.
    float prev = t10;
    bool mono = true;
    for (float p = 0.95f; p > 0.2f; p -= 0.1f) {
        const float t = EvmRateSelector::expectedTimeToDeliver(10, 4, p, full, frame);
        mono = mono && t > prev;
        prev = t;
    }
    CHECK(mono, "lower frame success always takes longer");
    // One frame at p: a geometric number of one-frame rounds.
    const float t1 = EvmRateSelector::expectedTimeToDeliver(1, 4, 0.5f, full, frame);
    CHECK(std::fabs(t1 - 2.0f * (full - 3.0f * frame)) < 1e-3f,
          "a single frame at p=0.5 takes two one-frame rounds on average (" << t1 << ")");

    // The decision: frames that make 16QAM R2/3 ~80 % and 8PSK R2/3 ~100 %.
    auto geometry = productionGeometry();
    for (auto& g : geometry) {
        if (g.valid()) g.frame_airtime_s = 1.2f;
    }
    EvmRateSelector sel;
    for (int g = 0; g < 4; ++g) {
        std::vector<float> e = {13.5f, 13.5f, 13.5f, 13.5f, 11.0f};   // 1 in 5 below 16QAM's cliff
        sel.observeGroup(kRungIdxQam8R23, e, 5, 5);
    }
    CHECK(sel.successProb(kRungIdxQam16R23) > 0.7f && sel.successProb(kRungIdxQam16R23) < 0.85f &&
              sel.successProb(kRungIdxQam8R23) > 0.98f,
          "fixture: 16QAM ~80 %, 8PSK ~100 % (" << sel.successProb(kRungIdxQam16R23) << ", "
                                                 << sel.successProb(kRungIdxQam8R23) << ")");
    const auto long_xfer = sel.best(geometry, kRungIdxNone, 1.0e6f);
    CHECK(long_xfer.rung == kRungIdxQam16R23,
          "a 1 MB transfer takes the higher long-run rate (got "
              << static_cast<int>(long_xfer.rung) << ")");
    const auto last_frame = sel.best(geometry, kRungIdxNone, 500.0f);
    CHECK(last_frame.rung != kRungIdxQam16R23,
          "the last frame of a transfer does not gamble on an 80 % rung (got "
              << static_cast<int>(last_frame.rung) << ")");
}

void test_climb_mode_only_takes_reliable_rungs() {
    const auto geometry = productionGeometry();
    EvmRateSelector awgn;
    awgn.observeGroup(kRungIdxQpskR12, frames(18.0f, 5), 5, 5);
    CHECK(awgn.bestReliable(geometry, kRungIdxNone, LatentRateController::kCapProb) ==
              kRungIdxQam16R23,
          "AWGN-like frames: 16QAM R2/3 is reliable, so a climb may go straight to it");
    EvmRateSelector fading;
    for (int g = 0; g < 4; ++g) {
        fading.observeGroup(kRungIdxQam8R23, {13.5f, 13.5f, 13.5f, 13.5f, 11.0f}, 5, 5);
    }
    CHECK(fading.bestReliable(geometry, kRungIdxNone, LatentRateController::kCapProb) ==
              kRungIdxQam8R23,
          "16QAM R2/3 at ~80 % is not a reliable climb; 8PSK R2/3 is");
    EvmRateSelector weak;
    weak.observeGroup(kRungIdxQpskR12, frames(3.0f, 5), 0, 5);
    CHECK(weak.bestReliable(geometry, kRungIdxNone, LatentRateController::kCapProb) ==
              kRungIdxNone,
          "no rung qualifies on faded frames: the latent decision stands");
}

void test_thresholds_are_ordered() {
    // Within a modulation, a higher code rate needs more EVM; across the ladder the densest
    // enabled rung needs the most.
    CHECK(evmDecodeThresholdDb(kRungIdxQpskR14) < evmDecodeThresholdDb(kRungIdxQpskR12) &&
              evmDecodeThresholdDb(kRungIdxQpskR12) < evmDecodeThresholdDb(kRungIdxQpskR23) &&
              evmDecodeThresholdDb(kRungIdxQpskR23) < evmDecodeThresholdDb(kRungIdxQpskR34),
          "QPSK thresholds rise with code rate");
    CHECK(evmDecodeThresholdDb(kRungIdxQam8R23) < evmDecodeThresholdDb(kRungIdxQam8R34),
          "8PSK thresholds rise with code rate");
    CHECK(evmDecodeThresholdDb(kRungIdxQam16R12) < evmDecodeThresholdDb(kRungIdxQam16R23) &&
              evmDecodeThresholdDb(kRungIdxQam16R23) < evmDecodeThresholdDb(kRungIdxQam16R34),
          "16QAM thresholds rise with code rate");
    CHECK(evmDecodeThresholdDb(kRungIdxQpskR34) < evmDecodeThresholdDb(kRungIdxQam8R23) &&
              evmDecodeThresholdDb(kRungIdxQam8R23) < evmDecodeThresholdDb(kRungIdxQam16R23),
          "denser modulation at the same rate needs more EVM");
}

}  // namespace

int main() {
    std::cout << "=== EVM Rate Selector Tests ===\n";
    test_margin_is_visible_from_any_rung();
    test_fade_is_rung_independent();
    test_margin_moves_on_calibration_error_only();
    test_window_and_unmeasured_frames();
    test_time_to_finish_objective();
    test_climb_mode_only_takes_reliable_rungs();
    test_thresholds_are_ordered();
    std::cout << (tests_failed == 0 ? "PASS" : "FAIL") << ": "
              << (tests_run - tests_failed) << "/" << tests_run << " checks\n";
    return tests_failed == 0 ? 0 : 1;
}
