#include "audio/channel_busy_detector.hpp"

#include <cassert>
#include <chrono>
#include <iostream>

using ultra::audio::ChannelBusyDetector;
using ultra::audio::ChannelBusyDetectorConfig;

namespace {

using Clock = ChannelBusyDetector::Clock;

ChannelBusyDetectorConfig testConfig() {
    ChannelBusyDetectorConfig config;
    config.quiet_rms_threshold = 0.005f;
    config.quiet_hold_ms = 30;
    config.rms_window_ms = 30;
    config.max_wait_for_idle_ms = 50;
    return config;
}

Clock::time_point syntheticBase() {
    static const Clock::time_point base = Clock::now() - std::chrono::seconds(10);
    return base;
}

Clock::time_point atMs(int ms) {
    return syntheticBase() + std::chrono::milliseconds(ms);
}

void observe(ChannelBusyDetector& detector,
             float rms,
             int ms,
             bool local_tx = false) {
    detector.observeRms(rms, local_tx, atMs(ms));
}

bool isIdleAtMs(const ChannelBusyDetector& detector, int ms) {
    return detector.isIdleFor(std::chrono::milliseconds(0), atMs(ms));
}

void testInitialStateIsIdle() {
    ChannelBusyDetector detector(testConfig());
    assert(detector.isIdle());
    assert(detector.isIdleFor(std::chrono::milliseconds(50)));
}

void testBusyThenQuietHold() {
    ChannelBusyDetector detector(testConfig());

    observe(detector, 0.100f, 0);
    assert(!detector.isIdle());

    observe(detector, 0.000f, 10);
    assert(!detector.isIdle());
    observe(detector, 0.000f, 20);
    assert(!detector.isIdle());
    observe(detector, 0.000f, 30);
    assert(!detector.isIdle());

    observe(detector, 0.000f, 40);
    assert(detector.timeSinceQuiet(Clock::now()).count() >= 0);
    assert(detector.isIdle());
}

void testMovingWindowAbsorbsShortDropout() {
    ChannelBusyDetector detector(testConfig());

    observe(detector, 0.080f, 0);
    observe(detector, 0.080f, 10);
    observe(detector, 0.000f, 20);
    observe(detector, 0.080f, 30);
    assert(!detector.isIdle());

    observe(detector, 0.000f, 70);
    observe(detector, 0.000f, 80);
    observe(detector, 0.000f, 90);
    observe(detector, 0.000f, 100);
    assert(detector.isIdle());
}

void testLocalTxBlackoutIsBusyRegardlessOfRms() {
    ChannelBusyDetector detector(testConfig());

    observe(detector, 0.000f, 0, true);
    assert(!isIdleAtMs(detector, 0));

    observe(detector, 0.000f, 10, false);
    observe(detector, 0.000f, 20, false);
    observe(detector, 0.000f, 30, false);
    assert(!isIdleAtMs(detector, 30));

    observe(detector, 0.000f, 40, false);
    assert(isIdleAtMs(detector, 40));
}

void testAdaptiveNoiseFloorTreatsAwgnFloorAsIdle() {
    ChannelBusyDetectorConfig config = testConfig();
    config.min_noise_floor_observations = 5;
    ChannelBusyDetector detector(config);

    for (int i = 0; i < 10; ++i) {
        observe(detector, 0.090f, i * 10);
    }
    assert(detector.isIdle());

    observe(detector, 0.360f, 200);
    observe(detector, 0.360f, 210);
    observe(detector, 0.360f, 220);
    assert(!detector.isIdle());

    for (int i = 0; i < 5; ++i) {
        observe(detector, 0.090f, 300 + i * 10);
    }
    assert(detector.isIdle());
}

void testAdaptiveThresholdRejectsFadedCarrierDips() {
    ChannelBusyDetectorConfig config = testConfig();
    config.min_noise_floor_observations = 5;
    ChannelBusyDetector detector(config);

    for (int i = 0; i < 20; ++i) {
        observe(detector, 0.050f, i * 10);
    }
    assert(detector.isIdle());

    observe(detector, 0.220f, 300);
    observe(detector, 0.180f, 310);
    observe(detector, 0.120f, 320);
    observe(detector, 0.095f, 330);
    observe(detector, 0.090f, 340);
    observe(detector, 0.105f, 350);
    observe(detector, 0.130f, 360);
    assert(!detector.isIdle());
    assert(!detector.isIdleFor(std::chrono::milliseconds(50)));
}

void testSignalDominatedHistoryDoesNotRaiseNoiseFloor() {
    ChannelBusyDetectorConfig config = testConfig();
    config.min_noise_floor_observations = 5;
    config.noise_floor_window_ms = 5000;
    ChannelBusyDetector detector(config);

    for (int i = 0; i < 20; ++i) {
        observe(detector, 0.060f, i * 10);
    }

    const float seeded_threshold = detector.quietThreshold();
    assert(seeded_threshold > 0.085f);
    assert(seeded_threshold < 0.100f);
    assert(detector.isIdle());

    for (int i = 0; i < 650; ++i) {
        const float signal_rms =
            (i % 3 == 0) ? 0.180f : ((i % 3 == 1) ? 0.260f : 0.300f);
        observe(detector, signal_rms, 500 + i * 10);
        assert(detector.quietThreshold() < 0.120f);
    }

    assert(!detector.isIdle());
    observe(detector, 0.181f, 7100);
    assert(!detector.isIdle());

    for (int i = 0; i < 10; ++i) {
        observe(detector, 0.060f, 8000 + i * 10);
    }

    const float recovered_threshold = detector.quietThreshold();
    assert(recovered_threshold > 0.085f);
    assert(recovered_threshold < 0.100f);
    assert(detector.isIdle());
}

void testSignalSampleCeilingRejectsFullPowerCarrier() {
    ChannelBusyDetectorConfig config = testConfig();
    config.min_noise_floor_observations = 5;
    config.noise_floor_window_ms = 5000;
    config.noise_floor_percentile = 0.50f;
    config.noise_floor_bootstrap_rms_ceiling = 0.305f;
    config.noise_floor_estimate_rms_ceiling = 0.229f;
    ChannelBusyDetector detector(config);

    for (int i = 0; i < 20; ++i) {
        observe(detector, 0.030f, i * 10);
    }

    assert(detector.isIdle());
    assert(detector.quietThreshold() > 0.040f);
    assert(detector.quietThreshold() < 0.050f);

    for (int i = 0; i < 650; ++i) {
        observe(detector, 0.260f, 500 + i * 10);
        if (i >= 4) {
            assert(!isIdleAtMs(detector, 500 + i * 10));
        }
        assert(detector.quietThreshold() < 0.050f);
    }

    assert(!detector.isIdle());
    observe(detector, 0.261f, 7100);
    assert(!detector.isIdle());
}

void testNoisyIdleCanSeedBelowSignalSampleCeiling() {
    ChannelBusyDetectorConfig config = testConfig();
    config.min_noise_floor_observations = 5;
    config.noise_floor_bootstrap_rms_ceiling = 0.305f;
    config.noise_floor_estimate_rms_ceiling = 0.229f;
    ChannelBusyDetector detector(config);

    for (int i = 0; i < 20; ++i) {
        observe(detector, 0.180f, i * 10);
    }

    assert(detector.isIdle());
    assert(detector.quietThreshold() > 0.265f);
    assert(detector.quietThreshold() < 0.275f);
}

// The production GUI/ModemEngine ratiometric calibration must (a) read idle on a
// bursty noise floor at ANY absolute level, (b) detect a signal that rises a few
// dB above that floor, (c) stay busy for ~the signal's length, and (d) release to
// idle once the signal stops — using the SAME shared config the GUI uses, so the
// calibration and its proof cannot drift apart.
void testRatiometricCarrierSenseAcrossLevelsAndSignal() {
    // ~60 dB of absolute range: a quiet sound-card input (-70 / -60 dBFS) must sense the peer
    // like a loud one (BUG-RX-ABSOLUTE-LEVEL: a 0.005 floor made 0.0003-0.001 read idle).
    const float levels[] = {0.0003f, 0.001f, 0.02f, 0.10f, 0.30f};
    for (float L : levels) {
        ChannelBusyDetectorConfig cfg = ultra::audio::ratiometricHfCarrierSenseConfig();
        cfg.quiet_hold_ms = 30;
        cfg.rms_window_ms = 30;
        cfg.min_noise_floor_observations = 5;
        cfg.max_wait_for_idle_ms = 50;
        ChannelBusyDetector detector(cfg);

        int t = 0;
        // (a) ~2 s of bursty noise around floor L (real-HF-like +/-15%).
        for (; t < 2000; t += 20) {
            observe(detector, (t / 20) % 2 ? L * 1.15f : L * 0.85f, t);
        }
        assert(isIdleAtMs(detector, t));  // idle on the noise floor at level L
        const float thr = detector.quietThreshold();
        assert(thr > 1.6f * L && thr < 2.4f * L);  // threshold tracks ~2x floor (ratiometric)

        // (b)+(c) inject a signal at 3x floor (+9.5 dB) for a known duration D.
        const int sig_start = t;
        const int D = 1000;
        int first_busy = -1;
        int last_busy = -1;
        for (; t < sig_start + D; t += 20) {
            observe(detector, 3.0f * L, t);
            if (!isIdleAtMs(detector, t)) {
                if (first_busy < 0) first_busy = t;
                last_busy = t;
            }
        }
        assert(first_busy >= 0);                       // signal was DETECTED
        assert(first_busy - sig_start <= 100);         // detected promptly (~window+hold)
        assert(detector.quietThreshold() < 2.6f * L);  // signal did NOT pollute the floor

        // (d) noise resumes -> must release back to idle.
        const int sig_end = t;
        for (; t < sig_end + 400; t += 20) {
            observe(detector, (t / 20) % 2 ? L * 1.15f : L * 0.85f, t);
        }
        assert(isIdleAtMs(detector, t));               // released after the signal stopped

        // busy interval should match the signal length within the detector latency.
        const int busy_dur = last_busy - first_busy + 20;
        const int err = busy_dur > D ? busy_dur - D : D - busy_dur;
        assert(err <= 150);                            // busy duration ~= signal length
    }
}

void testRatiometricFloorSurvivesLongBusySignal() {
    ChannelBusyDetectorConfig cfg = ultra::audio::ratiometricHfCarrierSenseConfig();
    cfg.quiet_hold_ms = 30;
    cfg.rms_window_ms = 30;
    cfg.noise_floor_window_ms = 1000;
    cfg.min_noise_floor_observations = 5;
    ChannelBusyDetector detector(cfg);

    int t = 0;
    for (; t < 1200; t += 20) {
        observe(detector, 0.030f, t);
    }
    assert(isIdleAtMs(detector, t));
    assert(detector.quietThreshold() > 0.055f);
    assert(detector.quietThreshold() < 0.070f);

    for (; t < 9000; t += 20) {
        observe(detector, 0.300f, t);
        assert(!isIdleAtMs(detector, t));
        assert(detector.quietThreshold() < 0.080f);
    }

    for (; t < 9400; t += 20) {
        observe(detector, 0.030f, t);
    }
    assert(isIdleAtMs(detector, t));
    assert(detector.quietThreshold() > 0.055f);
    assert(detector.quietThreshold() < 0.080f);
}

// Regression for the IONOS hardware lockup: the floor seeds LOW (the band noise
// is not yet flowing when audio opens / device warmup), then the real in-band
// WGN rises above floor x multiplier. The one-way admission gate rejects every
// real sample as "signal", starves the floor window, and latches the cached
// floor below the noise -> permanent false-busy (every pre-connection TX is
// CCA-deferred, collapsing the half-duplex handshake). The sustained-elevation
// relearn must detect that the elevation is a risen floor (not a time-bounded
// signal) and re-seed, returning the detector to IDLE on the steady noise.
void testRelearnsRisenNoiseFloorAfterLowSeed() {
    ChannelBusyDetectorConfig cfg = ultra::audio::ratiometricHfCarrierSenseConfig();
    cfg.quiet_hold_ms = 30;
    cfg.rms_window_ms = 30;
    cfg.noise_floor_window_ms = 500;
    cfg.min_noise_floor_observations = 5;
    cfg.max_wait_for_idle_ms = 50;
    cfg.noise_floor_relearn_after_ms = 1000;  // scaled-down for a fast test
    ChannelBusyDetector detector(cfg);

    int t = 0;
    // (a) seed the floor LOW on a quiet channel.
    for (; t < 600; t += 20) observe(detector, 0.004f, t);
    assert(isIdleAtMs(detector, t));
    const float low_thr = detector.quietThreshold();
    assert(low_thr < 0.012f);  // threshold tracked the low seed

    // (b) the band noise rises to 0.02 and STAYS there. Before relearn fires the
    //     latch keeps it busy (this is exactly the hardware false-busy).
    const int rise_start = t;
    for (; t < rise_start + 800; t += 20) observe(detector, 0.020f, t);
    assert(!isIdleAtMs(detector, t));  // latched busy on the noise (pre-relearn)

    // (c) once the elevation persists past relearn_after, the floor re-seeds and
    //     the detector returns to idle on the steady noise.
    for (; t < rise_start + 3000; t += 20) observe(detector, 0.020f, t);
    assert(isIdleAtMs(detector, t));  // unstuck
    const float new_thr = detector.quietThreshold();
    assert(new_thr > 1.6f * 0.020f && new_thr < 2.6f * 0.020f);  // re-tracked ~2x floor

    // (d) a genuine signal a few dB above the NEW floor is still detected busy.
    observe(detector, 0.060f, t); t += 20;
    observe(detector, 0.060f, t); t += 20;
    observe(detector, 0.060f, t);
    assert(!isIdleAtMs(detector, t));
}

// A half-duplex station hears digital silence while its own transmitter is keyed. Learned as noise,
// an 8 s burst filled the 5 s floor window with zeros: floor 0, every real noise sample refused by
// the 0 x 2 admission gate, busy until the 25 s relearn (GUI half-duplex gate, 2026-11-11).
void testDigitalSilenceIsNotNoise() {
    ChannelBusyDetectorConfig cfg = ultra::audio::ratiometricHfCarrierSenseConfig();
    cfg.quiet_hold_ms = 30;
    cfg.rms_window_ms = 30;
    cfg.max_wait_for_idle_ms = 50;
    ChannelBusyDetector detector(cfg);
    int t = 0;
    for (; t < 2000; t += 20) observe(detector, 0.030f, t);  // the band's noise
    assert(isIdleAtMs(detector, t));
    const float thr = detector.quietThreshold();
    assert(thr > 0.05f && thr < 0.07f);
    for (const int burst_end = t + 8000; t < burst_end; t += 20) {
        observe(detector, 0.0f, t);  // our own 8 s burst: the receive is muted
    }
    assert(detector.quietThreshold() == thr);  // the floor never learned the silence
    for (int k = 0; k < 5; ++k, t += 20) observe(detector, 0.030f, t);  // the band again
    assert(isIdleAtMs(detector, t));  // idle at once, not after the relearn
}

void testWaitUntilIdleTimeoutAndGuard() {
    ChannelBusyDetector detector(testConfig());
    observe(detector, 0.050f, 0);

    const auto start = Clock::now();
    const bool timed_out = detector.waitUntilIdle(
        std::chrono::milliseconds(10),
        std::chrono::milliseconds(5));
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - start);
    assert(!timed_out);
    assert(elapsed >= std::chrono::milliseconds(4));

    detector.reset();
    assert(detector.waitUntilIdle(std::chrono::milliseconds(10),
                                  std::chrono::milliseconds(20)));
}

}  // namespace

int main() {
    testInitialStateIsIdle();
    testBusyThenQuietHold();
    testMovingWindowAbsorbsShortDropout();
    testLocalTxBlackoutIsBusyRegardlessOfRms();
    testAdaptiveNoiseFloorTreatsAwgnFloorAsIdle();
    testAdaptiveThresholdRejectsFadedCarrierDips();
    testSignalDominatedHistoryDoesNotRaiseNoiseFloor();
    testSignalSampleCeilingRejectsFullPowerCarrier();
    testNoisyIdleCanSeedBelowSignalSampleCeiling();
    testRatiometricCarrierSenseAcrossLevelsAndSignal();
    testRatiometricFloorSurvivesLongBusySignal();
    testRelearnsRisenNoiseFloorAfterLowSeed();
    testDigitalSilenceIsNotNoise();
    testWaitUntilIdleTimeoutAndGuard();

    std::cout << "ChannelBusyDetector tests passed\n";
    return 0;
}
