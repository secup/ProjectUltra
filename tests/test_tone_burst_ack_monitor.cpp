// test_tone_burst_ack_monitor.cpp — unit tests for the sliding-window
// detector wrapper. PHY_ADAPTATION_DESIGN §15 step 4a.

#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"
#include "waveform/tone_burst_ack/tone_burst_constants.hpp"
#include "waveform/tone_burst_ack/tone_burst_encoder.hpp"

#include <algorithm>
#include <cstdio>
#include <random>
#include <vector>

using namespace ultra::waveform::tone_burst_ack;

namespace {

int g_failures = 0;

#define EXPECT(cond)                                                            \
    do {                                                                        \
        if (!(cond)) {                                                          \
            ++g_failures;                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);\
        }                                                                       \
    } while (0)

#define EXPECT_EQ(a, b)                                                         \
    do {                                                                        \
        const auto _va = (a);                                                   \
        const auto _vb = (b);                                                   \
        if (!(_va == _vb)) {                                                    \
            ++g_failures;                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s != %s (%lld vs %lld)\n",       \
                         __FILE__, __LINE__, #a, #b,                            \
                         static_cast<long long>(_va),                           \
                         static_cast<long long>(_vb));                          \
        }                                                                       \
    } while (0)

ToneBurstAckPayload payloadV(int v) {
    ToneBurstAckPayload p;
    switch (v % 4) {
        case 0: p.group_seq = 0;  p.frame_mask = 0;       p.rate_hint = 0; p.type = AckType::Ack;  break;
        case 1: p.group_seq = 63; p.frame_mask = 63;      p.rate_hint = 7; p.type = AckType::Nack; break;
        case 2: p.group_seq = 42; p.frame_mask = 0b101010;p.rate_hint = 4; p.type = AckType::Ack;  break;
        case 3: p.group_seq = 13; p.frame_mask = 0b010101;p.rate_hint = 2; p.type = AckType::Nack; break;
    }
    return p;
}
bool payloadsEqual(const ToneBurstAckPayload& a, const ToneBurstAckPayload& b) {
    return a.group_seq == b.group_seq && a.frame_mask == b.frame_mask &&
           a.rate_hint == b.rate_hint && a.type == b.type;
}

void appendScaled(std::vector<float>& out, const std::vector<float>& in,
                  float scale) {
    out.reserve(out.size() + in.size());
    for (float sample : in) out.push_back(sample * scale);
}

// ============================================================================
// Tests
// ============================================================================

void test_monitor_detects_single_burst_in_long_stream() {
    std::printf("[test] monitor_detects_single_burst_in_long_stream\n");
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    ToneBurstEncoder enc;
    const auto orig = payloadV(2);
    const auto burst = enc.encode(orig, kBaselineSymbolMs);

    // Feed 200 ms of leading silence in 1024-sample chunks, then the burst,
    // then 200 ms of trailing silence. This mimics the streaming-feed pattern
    // (small chunks, not the whole buffer at once).
    const size_t chunk = 1024;
    std::vector<float> silence(chunk, 0.0f);
    const size_t leading_silence_chunks = ((kSampleRate / 1000) * 200) / chunk;
    for (size_t i = 0; i < leading_silence_chunks; ++i) {
        mon.feedAudio(silence.data(), silence.size());
    }
    // Feed the burst in chunks.
    size_t off = 0;
    while (off < burst.size()) {
        const size_t n = std::min(chunk, burst.size() - off);
        mon.feedAudio(burst.data() + off, n);
        off += n;
    }
    for (size_t i = 0; i < leading_silence_chunks; ++i) {
        mon.feedAudio(silence.data(), silence.size());
    }

    EXPECT_EQ(events.size(), static_cast<size_t>(1));
    if (events.size() >= 1) {
        EXPECT(payloadsEqual(events[0].payload, orig));
        EXPECT_EQ(events[0].symbol_ms_used, kBaselineSymbolMs);
        std::printf("  detected@stream=%llu peak=%.1f symbol_ms=%u\n",
                    static_cast<unsigned long long>(events[0].detected_stream_offset),
                    events[0].correlation_peak, events[0].symbol_ms_used);
    }
    EXPECT_EQ(mon.detectionsEmitted(), static_cast<uint64_t>(1));
}

void test_monitor_handles_two_bursts_with_gap() {
    std::printf("[test] monitor_handles_two_bursts_with_gap\n");
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    ToneBurstEncoder enc;
    const auto orig0 = payloadV(0);
    const auto orig1 = payloadV(1);
    const auto burst0 = enc.encode(orig0, kBaselineSymbolMs);
    enc.resetPhase();
    const auto burst1 = enc.encode(orig1, kBaselineSymbolMs);

    // Lay out: [200 ms silence][burst0][1.5 s silence][burst1][200 ms silence]
    // The 1.5 s gap is longer than the suppress window (default 1 s).
    auto feed_silence_ms = [&](uint32_t ms) {
        const size_t n = (kSampleRate * ms) / 1000;
        std::vector<float> s(n, 0.0f);
        mon.feedAudio(s.data(), s.size());
    };
    feed_silence_ms(200);
    mon.feedAudio(burst0);
    feed_silence_ms(1500);
    mon.feedAudio(burst1);
    feed_silence_ms(200);

    EXPECT_EQ(events.size(), static_cast<size_t>(2));
    if (events.size() >= 2) {
        EXPECT(payloadsEqual(events[0].payload, orig0));
        EXPECT(payloadsEqual(events[1].payload, orig1));
    }
    EXPECT_EQ(mon.detectionsEmitted(), static_cast<uint64_t>(2));
    EXPECT_EQ(mon.suppressedAttempts(), static_cast<uint64_t>(0));
}

void test_monitor_suppresses_duplicate_within_window() {
    std::printf("[test] monitor_suppresses_duplicate_within_window\n");
    // Feed the SAME burst twice with a 100 ms gap (shorter than the 1 s
    // suppress window). Expect 1 detection (the first), the second is
    // suppressed.
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    ToneBurstEncoder enc;
    const auto orig = payloadV(3);
    const auto burst = enc.encode(orig, kBaselineSymbolMs);
    enc.resetPhase();
    const auto burst_again = enc.encode(orig, kBaselineSymbolMs);

    std::vector<float> short_gap((kSampleRate * 100u) / 1000u, 0.0f);
    mon.feedAudio(burst);
    mon.feedAudio(short_gap.data(), short_gap.size());
    mon.feedAudio(burst_again);

    EXPECT_EQ(events.size(), static_cast<size_t>(1));
    // The suppress count may be 0 or more depending on whether the second
    // burst's drop happens before any cadence pass; the important
    // guarantee is that we don't double-fire.
    EXPECT(mon.detectionsEmitted() == 1);
}

void test_monitor_no_detection_on_pure_silence() {
    std::printf("[test] monitor_no_detection_on_pure_silence\n");
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    // Feed 2 seconds of pure silence in 1024-sample chunks.
    std::vector<float> silence(1024, 0.0f);
    for (int i = 0; i < 100; ++i) mon.feedAudio(silence.data(), silence.size());

    EXPECT_EQ(events.size(), static_cast<size_t>(0));
    EXPECT_EQ(mon.detectionsEmitted(), static_cast<uint64_t>(0));
}

void test_monitor_no_false_positive_on_white_noise() {
    std::printf("[test] monitor_no_false_positive_on_white_noise\n");
    // Feed 1 second of mid-level white Gaussian noise. The Costas threshold
    // (3× noise floor) should prevent any false positive.
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    std::mt19937 rng(0xDEADBEEF);
    std::normal_distribution<float> dist(0.0f, 0.1f);
    const size_t chunk = 4800;  // 100 ms
    for (int i = 0; i < 10; ++i) {
        std::vector<float> noise(chunk);
        for (size_t k = 0; k < chunk; ++k) noise[k] = dist(rng);
        mon.feedAudio(noise.data(), noise.size());
    }
    std::printf("  detections=%llu (expected 0)\n",
                static_cast<unsigned long long>(mon.detectionsEmitted()));
    EXPECT_EQ(events.size(), static_cast<size_t>(0));
}

void test_monitor_reset_clears_state() {
    std::printf("[test] monitor_reset_clears_state\n");
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    ToneBurstEncoder enc;
    const auto orig = payloadV(1);
    const auto burst = enc.encode(orig, kBaselineSymbolMs);
    mon.feedAudio(burst);
    EXPECT_EQ(events.size(), static_cast<size_t>(1));

    mon.reset();
    // After reset, the suppression window is cleared — feeding the same
    // burst again should fire a fresh detection.
    enc.resetPhase();
    const auto burst2 = enc.encode(orig, kBaselineSymbolMs);
    mon.feedAudio(burst2);
    EXPECT_EQ(events.size(), static_cast<size_t>(2));
}

void test_monitor_handles_chunk_boundary_inside_burst() {
    std::printf("[test] monitor_handles_chunk_boundary_inside_burst\n");
    // Feed the burst split across many small chunks (16 samples each, which
    // is well below detect_interval_samples=240). The detector should
    // accumulate enough state to fire when the burst is complete.
    ToneBurstAckMonitor mon;
    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });

    ToneBurstEncoder enc;
    const auto orig = payloadV(2);
    const auto burst = enc.encode(orig, kBaselineSymbolMs);

    // 200 ms of leading silence first to seed the buffer.
    std::vector<float> silence((kSampleRate * 200u) / 1000u, 0.0f);
    mon.feedAudio(silence.data(), silence.size());

    const size_t tiny_chunk = 16;
    size_t off = 0;
    while (off < burst.size()) {
        const size_t n = std::min(tiny_chunk, burst.size() - off);
        mon.feedAudio(burst.data() + off, n);
        off += n;
    }
    // A bit of trailing audio so any final cadence pass fires.
    mon.feedAudio(silence.data(), silence.size());

    EXPECT_EQ(events.size(), static_cast<size_t>(1));
    if (events.size() >= 1) EXPECT(payloadsEqual(events[0].payload, orig));
}

void test_rearm_replaces_longer_prior_deadline() {
    std::printf("[test] rearm_replaces_longer_prior_deadline\n");
    ToneBurstAckMonitor::Config cfg;
    cfg.armed_only = true;
    cfg.detect_interval_samples_armed = 1000;
    ToneBurstAckMonitor mon(cfg);

    mon.arm(1000);
    EXPECT(mon.isArmed());
    mon.rearm(100);

    std::vector<float> silence(101, 0.0f);
    mon.feedAudio(silence);
    EXPECT(!mon.isArmed());
}

void test_protocol_gate_rejects_stronger_forgery_then_finds_valid_ack() {
    std::printf("[test] protocol_gate_rejects_stronger_forgery_then_finds_valid_ack\n");

    ToneBurstEncoder enc;
    ToneBurstAckPayload forged = payloadV(2);
    forged.group_seq = 10;  // outside the synthetic sender support below
    ToneBurstAckPayload valid = payloadV(2);
    valid.group_seq = 60;
    const auto forged_burst = enc.encode(forged, kBaselineSymbolMs);
    enc.resetPhase();
    const auto valid_burst = enc.encode(valid, kBaselineSymbolMs);

    // Put both wire-valid bursts inside ONE detector window. The impossible one is
    // deliberately stronger and earlier; the valid one is 0.9 dB down. Before the
    // pre-commit predicate, first-CRC-pass-wins would disarm on group_seq=10 and the
    // real group_seq=60 ACK would never reach the protocol.
    std::vector<float> stream((kSampleRate * 250u) / 1000u, 0.0f);
    appendScaled(stream, forged_burst, 1.0f);
    stream.insert(stream.end(), (kSampleRate * 80u) / 1000u, 0.0f);
    appendScaled(stream, valid_burst, 0.90f);
    stream.insert(stream.end(), (kSampleRate * 250u) / 1000u, 0.0f);

    ToneBurstAckMonitor::Config cfg;
    cfg.armed_only = true;
    cfg.symbol_durations_ms = {kBaselineSymbolMs};
    cfg.sweep_step_samples = 32;
    // A single pass sees the complete forged-before-valid window, making this a
    // regression of candidate continuation rather than a later cadence retry.
    cfg.detect_interval_samples_armed = stream.size();
    cfg.buffer_capacity_samples = stream.size() + 1024;
    ToneBurstAckMonitor mon(cfg);

    int predicate_calls = 0;
    int forged_rejections = 0;
    mon.setAcceptancePredicate([&](const ToneBurstAckPayload& candidate) {
        ++predicate_calls;
        // Acceptance runs before any monitor commit/disarm side effects.
        EXPECT(mon.isArmed());
        EXPECT_EQ(mon.detectionsEmitted(), static_cast<uint64_t>(0));
        if (candidate.group_seq == forged.group_seq) {
            ++forged_rejections;
            return false;
        }
        return candidate.group_seq == valid.group_seq;
    });

    std::vector<ToneBurstAckDetection> events;
    mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });
    mon.arm(stream.size() + 1);
    mon.feedAudio(stream);

    EXPECT(predicate_calls >= 2);
    EXPECT(forged_rejections >= 1);
    EXPECT(mon.semanticCandidatesRejected() >= 1);
    EXPECT_EQ(events.size(), static_cast<size_t>(1));
    EXPECT_EQ(mon.detectionsEmitted(), static_cast<uint64_t>(1));
    EXPECT(!mon.isArmed());
    if (!events.empty()) {
        EXPECT(payloadsEqual(events.front().payload, valid));
        EXPECT(events.front().min_symbol_confidence > 1.0f);
    }
}

// BUG-TONE-ACK-BEFORE-ARM: an ACK answers the burst that armed the window. A CRC-valid burst that
// was already in the buffer before arm() (pre-arm audio the scan window reaches back into) must not
// be accepted; the same burst after the arm must be.
void test_armed_monitor_ignores_bursts_before_the_arm() {
    std::printf("[test] armed_monitor_ignores_bursts_before_the_arm\n");
    ToneBurstEncoder enc;
    const auto burst = enc.encode(payloadV(2), kBaselineSymbolMs);
    const std::vector<float> gap((kSampleRate * 50u) / 1000u, 0.0f);
    const std::vector<float> tail(kSampleRate, 0.0f);
    for (const bool before_arm : {true, false}) {
        ToneBurstAckMonitor::Config cfg;
        cfg.armed_only = true;
        cfg.symbol_durations_ms = {kBaselineSymbolMs};
        ToneBurstAckMonitor mon(cfg);
        std::vector<ToneBurstAckDetection> events;
        mon.setCallback([&](const ToneBurstAckDetection& d) { events.push_back(d); });
        mon.feedAudio(gap.data(), gap.size());
        if (before_arm) {
            mon.feedAudio(burst.data(), burst.size());
            mon.feedAudio(gap.data(), gap.size());
            mon.arm(kSampleRate * 2);
        } else {
            mon.arm(kSampleRate * 4);
            mon.feedAudio(burst.data(), burst.size());
        }
        for (size_t off = 0; off < tail.size(); off += 1024) {
            mon.feedAudio(tail.data() + off, std::min<size_t>(1024, tail.size() - off));
        }
        EXPECT_EQ(events.size(), static_cast<size_t>(before_arm ? 0 : 1));
    }
}

}  // namespace

int main() {
    test_monitor_detects_single_burst_in_long_stream();
    test_monitor_handles_two_bursts_with_gap();
    test_monitor_suppresses_duplicate_within_window();
    test_monitor_no_detection_on_pure_silence();
    test_monitor_no_false_positive_on_white_noise();
    test_monitor_reset_clears_state();
    test_monitor_handles_chunk_boundary_inside_burst();
    test_rearm_replaces_longer_prior_deadline();
    test_protocol_gate_rejects_stronger_forgery_then_finds_valid_ack();
    test_armed_monitor_ignores_bursts_before_the_arm();

    if (g_failures > 0) {
        std::fprintf(stderr, "\n%d test assertion(s) failed\n", g_failures);
        return 1;
    }
    std::printf("\nAll tone-burst ACK monitor tests PASSED\n");
    return 0;
}
