// Tests for the effective-SNR rate selector (ULTRA_RATE_EVM, protocol/evm_rate_selector.hpp).
//
// Properties under test — the ones the outcome-only latent controller lacks:
//   1. Margin is visible from ANY rung: frames received on QPSK R1/2 predict 16QAM R2/3.
//   2. A fade is rung-independent: faded frames lower every rung's predicted success, so a
//      deep fade does not make a lower rung look like an escape.
//   3. The outer-loop margin moves on calibration error only: a loss the frames' own EVMs
//      predicted leaves it unchanged; a loss they did not predict raises it.
//   4. Window and readiness: unmeasured frames are skipped, the window holds kWindowFrames.
//   5. Thresholds are carried between constellations (ofdm/dd_evm_model.hpp): a rung's own
//      threshold applies to its own constellation's frames, and frames of another
//      constellation are compared with the reading THEY give at the same true SNR.

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
    CHECK(sel.successProb(kRungIdxQam16R23) > 0.99f &&
              sel.successProb(kRungIdxQam32R23) > 0.99f,
          "20 dB EVM frames predict 16QAM R2/3 and 32QAM R2/3 decode them");
    const auto pick = sel.best(productionGeometry());
    CHECK(pick.rung == kRungIdxQam32R23,
          "AWGN-like 20 dB frames select the densest rung, 32QAM R2/3, directly from R1/2 (got "
              << static_cast<int>(pick.rung) << ")");

    // QPSK frames midway between QPSK R2/3's threshold and 8PSK R2/3's threshold as QPSK
    // frames read it: the first decodes them, the second does not.
    const float t_qpsk = evmThresholdOnFramesDb(kRungIdxQpskR23, Modulation::QPSK);
    const float t_8psk = evmThresholdOnFramesDb(kRungIdxQam8R23, Modulation::QPSK);
    CHECK(t_8psk - t_qpsk > 1.5f,
          "8PSK R2/3 sits >1.5 dB above QPSK R2/3 on QPSK frames (" << t_qpsk << ", " << t_8psk
                                                                     << ")");
    EvmRateSelector mid;
    mid.observeGroup(kRungIdxQpskR23, frames(0.5f * (t_qpsk + t_8psk), 5), 5, 5);
    CHECK(mid.successProb(kRungIdxQpskR23) > 0.9f, "the midway frames decode on QPSK R2/3");
    CHECK(mid.successProb(kRungIdxQam8R23) < 0.1f, "and fail on 8PSK R2/3");
    const auto p8 = mid.best(productionGeometry());
    CHECK(p8.rung >= kRungIdxQpskR23 && p8.rung <= kRungIdxQpskR34,
          "midway frames select a QPSK R2/3-R3/4 rung (got " << static_cast<int>(p8.rung) << ")");
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
    CHECK(sel.best(productionGeometry()).rung >= kRungIdxQam8R23,
          "so the selector does not escape to a lower rung through it");
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
    // On 8PSK frames: four well above 16QAM R2/3's threshold as 8PSK frames read it, one 1 dB
    // above 8PSK R2/3's own (1 in 5 below 16QAM's cliff).
    const float t16 = evmThresholdOnFramesDb(kRungIdxQam16R23, Modulation::QAM8);
    const float t8 = evmDecodeThresholdDb(kRungIdxQam8R23);
    EvmRateSelector sel;
    for (int g = 0; g < 4; ++g) {
        const float hi = t16 + 1.5f, lo = t8 + 1.0f;
        std::vector<float> e = {hi, hi, hi, hi, lo};
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
              kRungIdxQam32R23,
          "AWGN-like frames: 32QAM R2/3 is reliable, so a climb may go straight to it");
    // 8PSK frames: four well above 16QAM R2/3's threshold as 8PSK frames read it, one between
    // 8PSK R2/3's own threshold and that: 16QAM R2/3 decodes ~80 %, 8PSK R2/3 all but certainly.
    const float t16 = evmThresholdOnFramesDb(kRungIdxQam16R23, Modulation::QAM8);
    const float t8 = evmDecodeThresholdDb(kRungIdxQam8R23);
    CHECK(t16 - t8 > 1.2f, "16QAM R2/3 sits above 8PSK R2/3 on 8PSK frames");
    EvmRateSelector fading;
    for (int g = 0; g < 4; ++g) {
        const float hi = t16 + 1.5f, lo = t8 + 0.7f;
        fading.observeGroup(kRungIdxQam8R23, {hi, hi, hi, hi, lo}, 5, 5);
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
    CHECK(evmDecodeThresholdDb(kRungIdxQam16R34) < evmDecodeThresholdDb(kRungIdxQam32R23),
          "32QAM R2/3 needs more than 16QAM R3/4");
}

void test_thresholds_carried_between_constellations() {
    // Own constellation: identity, for every rung.
    for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
        const auto pick = coherentRungFromIndex(r);
        CHECK(evmThresholdOnFramesDb(r, pick.mod) == evmDecodeThresholdDb(r),
              "rung " << static_cast<int>(r) << " keeps its own threshold on its own frames");
    }
    // 32QAM frames read high near 32QAM's threshold, so on 16QAM frames the same channel reads
    // lower: the threshold a 16QAM link must clear to climb is below 32QAM's own.
    const float own32 = evmDecodeThresholdDb(kRungIdxQam32R23);
    const float on16 = evmThresholdOnFramesDb(kRungIdxQam32R23, Modulation::QAM16);
    CHECK(on16 < own32 - 1.0f && on16 > own32 - 3.0f,
          "32QAM R2/3 on 16QAM frames: " << on16 << " vs own " << own32);
    // Measured (measure_ack_fer AWGN, 2026-11-10): 16QAM frames read 15.2 dB at 18 dB, where
    // 32QAM R2/3 loses no frame, and 13.0 dB at 15 dB, where it loses every frame.
    EvmRateSelector at18;
    at18.observeGroup(kRungIdxQam16R23, frames(15.2f, 5), 5, 5);
    CHECK(at18.successProb(kRungIdxQam32R23) > LatentRateController::kCapProb,
          "16QAM frames at 18 dB AWGN predict 32QAM R2/3 reliably ("
              << at18.successProb(kRungIdxQam32R23) << ")");
    EvmRateSelector at15;
    at15.observeGroup(kRungIdxQam16R23, frames(13.0f, 5), 5, 5);
    CHECK(at15.successProb(kRungIdxQam32R23) < 0.5f,
          "16QAM frames at 15 dB AWGN do not (" << at15.successProb(kRungIdxQam32R23) << ")");
    CHECK(at15.bestReliable(productionGeometry(), kRungIdxNone,
                            LatentRateController::kCapProb) != kRungIdxQam32R23,
          "so no climb to 32QAM there");
    // A mixed window: each frame against its own constellation's reading.
    EvmRateSelector mixed;
    mixed.observeGroup(kRungIdxQam16R23, frames(15.2f, 5), 5, 5);
    mixed.observeGroup(kRungIdxQam32R23, frames(own32 + 2.0f, 5), 5, 5);
    CHECK(mixed.successProb(kRungIdxQam32R23) > 0.95f,
          "16QAM and 32QAM frames on one good channel agree");
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
    test_thresholds_carried_between_constellations();
    std::cout << (tests_failed == 0 ? "PASS" : "FAIL") << ": "
              << (tests_run - tests_failed) << "/" << tests_run << " checks\n";
    return tests_failed == 0 ? 0 : 1;
}
