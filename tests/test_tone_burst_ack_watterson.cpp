// test_tone_burst_ack_watterson.cpp — tone-burst ACK round-trip across
// Watterson Good + Moderate fading channels. PHY_ADAPTATION_DESIGN §15 step 3.
//
// The AWGN cells in step 2 are the easy case — additive noise spreads
// energy across all frequencies equally, so 4-FSK Goertzel-bin detection
// is bounded by SNR alone. Real HF adds two things AWGN doesn't have:
//   1. Multipath (delay-spread) → frequency-selective fading → an individual
//      tone can be nulled while its neighbors are fine.
//   2. Doppler spread → the channel's frequency response CHANGES over the
//      ~675 ms burst duration. Slow Doppler (Good=0.1 Hz, coherence
//      time ~10s) means the channel is ~constant within a burst; faster
//      Doppler (Moderate=0.5 Hz, coherence ~2s) means it can drift mid-burst.
//
// Question this test answers: does the tone-burst ACK survive HF fading,
// and at what SNR cells / duration choices? If a single deep null on one
// of the 4 tones reliably kills the burst, we need frequency diversity
// (multiple subbands) — that's the §15.7 question 4 fallback. If the
// (15,11) Hamming + 4-FSK density wins through, we're good as-is.

#include "ota_channel_core/models.hpp"
#include "waveform/tone_burst_ack/tone_burst_constants.hpp"
#include "waveform/tone_burst_ack/tone_burst_detector.hpp"
#include "waveform/tone_burst_ack/tone_burst_encoder.hpp"
#include "waveform/tone_burst_ack/tone_burst_payload.hpp"

#include <cstdint>
#include <cstdio>
#include <random>
#include <span>
#include <string>
#include <vector>

using namespace ultra::waveform::tone_burst_ack;
using ultra::ota_channel_core::WattersonChannel;
namespace itu = ultra::ota_channel_core::itu_r_f1487;

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

// ============================================================================
// Helpers
// ============================================================================

ToneBurstAckPayload samplePayload(int variant) {
    ToneBurstAckPayload p;
    switch (variant % 4) {
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

struct CellResult {
    int decoded_ok = 0;
    int decoded_wrong = 0;
    int crc_failed = 0;
    int trials = 0;
    float min_confidence_observed = 1e30f;
};

// Trials per cell. These cells are DETERMINISTIC (fixed per-trial seeds), so the
// pass rate is reproducible; the thresholds below carry generous margin for any
// cross-platform float drift. 20 trials keeps that margin while cutting this
// statistical test's CI cost ~2.5x vs the original 50 (it was the slowest test in
// the matrix at ~46 min on the Windows runner). Thresholds scale off kTrials.
constexpr int kTrials = 20;

CellResult runWattersonCell(const WattersonChannel::Config& base_config,
                             uint32_t symbol_ms, int trials, uint64_t base_seed,
                             float prior_hz = 0.0f, float prior_halfwidth_hz = 0.0f) {
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    det.setFrequencyPrior(prior_hz, prior_halfwidth_hz);
    CellResult r;
    r.trials = trials;

    // Use timing search for every trial. Real-world ACK arrival time is
    // always uncertain (T/R turnaround jitter + channel filter delay +
    // multipath spread), so this matches the production detection path.
    // The aligned-decode path is an optimization for when sync is already
    // locked; not what we want to test here.

    for (int t = 0; t < trials; ++t) {
        // Use a different seed per trial so each one sees independent fading.
        WattersonChannel ch(base_config, base_seed + static_cast<uint64_t>(t));
        // Warm-up: let the fading-oscillator state advance ~50 ms before
        // injecting the burst (so we start in a "live" channel state rather
        // than a constant-1 initial transient).
        std::vector<float> warmup_in(static_cast<size_t>(kSampleRate) * 50 / 1000, 0.0f);
        std::vector<float> warmup_out;
        ch.process(std::span<const float>(warmup_in), warmup_out);

        enc.resetPhase();
        const auto orig = samplePayload(t);
        const auto clean_burst = enc.encode(orig, symbol_ms);

        // Pad with silence so the detector has a search window. The
        // WattersonChannel adds Hilbert + multipath delay, so the burst
        // shows up at a non-zero offset; the timing search handles that.
        const uint32_t pad = (kSampleRate * 100u) / 1000u;
        std::vector<float> in;
        in.reserve(2 * pad + clean_burst.size());
        in.insert(in.end(), pad, 0.0f);
        in.insert(in.end(), clean_burst.begin(), clean_burst.end());
        in.insert(in.end(), pad, 0.0f);

        std::vector<float> noisy;
        ch.process(std::span<const float>(in), noisy);

        const auto dr = det.detectAndDecode(noisy.data(), noisy.size(), symbol_ms);
        const auto& recovered = dr.decode.payload;
        const float min_conf = dr.decode.min_confidence;

        if (recovered.has_value()) {
            if (payloadsEqual(*recovered, orig)) ++r.decoded_ok;
            else ++r.decoded_wrong;
        } else {
            ++r.crc_failed;
        }
        if (min_conf < r.min_confidence_observed) r.min_confidence_observed = min_conf;
    }
    return r;
}

void printCell(const std::string& label, const CellResult& r) {
    const double rate = r.trials > 0
        ? 100.0 * static_cast<double>(r.decoded_ok) / static_cast<double>(r.trials)
        : 0.0;
    std::printf("  %-50s ok=%3d wrong=%2d crc_fail=%2d (%.0f%%) min_conf=%.2f\n",
                label.c_str(), r.decoded_ok, r.decoded_wrong, r.crc_failed,
                rate, r.min_confidence_observed);
}

// ============================================================================
// Good fading cells (ITU-R F.1487: delay_spread=0.5 ms, Doppler=0.1 Hz)
// ============================================================================

void test_good_snr20_25ms_aligned() {
    std::printf("[test] good_snr20_25ms_aligned\n");
    auto cfg = itu::good(20.0f);
    const auto r = runWattersonCell(cfg, kBaselineSymbolMs, kTrials, 0xA001);
    printCell("Good@20 / 25ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 80 / 100);  // ~80% floor (Good@20, ~100% actual)
}

void test_good_snr10_25ms_aligned() {
    std::printf("[test] good_snr10_25ms_aligned\n");
    auto cfg = itu::good(10.0f);
    const auto r = runWattersonCell(cfg, kBaselineSymbolMs, kTrials, 0xA101);
    printCell("Good@10 / 25ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 80 / 100);  // ~80% floor (Good@10, ~100% actual)
}

void test_good_snr5_50ms_aligned() {
    std::printf("[test] good_snr5_50ms_aligned\n");
    // Longer symbol duration for marginal SNR (§15.5 staircase).
    auto cfg = itu::good(5.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsLowSNR, kTrials, 0xA201);
    printCell("Good@5  / 50ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 70 / 100);  // ~70% floor (Good@5, ~98% actual)
}

void test_good_snr20_25ms_timing_search() {
    std::printf("[test] good_snr20_25ms_timing_search\n");
    auto cfg = itu::good(20.0f);
    const auto r = runWattersonCell(cfg, kBaselineSymbolMs, kTrials, 0xA301);
    printCell("Good@20 / 25ms / timing search", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 80 / 100);  // ~80% floor (timing search, ~100% actual)
}

// ============================================================================
// Moderate fading cells (ITU-R F.1487: delay_spread=1.0 ms, Doppler=0.5 Hz)
// ============================================================================

void test_moderate_snr20_25ms_aligned() {
    std::printf("[test] moderate_snr20_25ms_aligned\n");
    auto cfg = itu::moderate(20.0f);
    const auto r = runWattersonCell(cfg, kBaselineSymbolMs, kTrials, 0xB001);
    printCell("Moderate@20 / 25ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 75 / 100);  // ~75% floor (Mod@20, ~100% actual)
}

void test_moderate_snr10_50ms_aligned() {
    std::printf("[test] moderate_snr10_50ms_aligned\n");
    auto cfg = itu::moderate(10.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsLowSNR, kTrials, 0xB101);
    printCell("Moderate@10 / 50ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 65 / 100);  // ~65% floor (Mod@10, ~96% actual)
}

void test_moderate_snr5_100ms_aligned() {
    std::printf("[test] moderate_snr5_100ms_aligned\n");
    auto cfg = itu::moderate(5.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsMargSNR, kTrials, 0xB201);
    printCell("Moderate@5  / 100ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 55 / 100);  // ~55% floor (Mod@5, ~98% actual)
}

// ============================================================================
// Reference: same SNR cells on AWGN-equivalent (no fading, just noise) to
// quantify the fading "cost" vs the AWGN baseline
// ============================================================================

void test_good_snr0_100ms_aligned() {
    std::printf("[test] good_snr0_100ms_aligned\n");
    auto cfg = itu::good(0.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsMargSNR, kTrials, 0xA401);
    printCell("Good@0   / 100ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 65 / 100);  // ~65% floor (Good@0, ~94% actual)
}

void test_good_snr_minus5_200ms_aligned() {
    std::printf("[test] good_snr_minus5_200ms_aligned\n");
    auto cfg = itu::good(-5.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsWeakSNR, kTrials, 0xA501);
    printCell("Good@-5  / 200ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 65 / 100);  // ~65% floor (Good@-5, ~94% actual)
}

void test_moderate_snr0_100ms_aligned() {
    std::printf("[test] moderate_snr0_100ms_aligned\n");
    auto cfg = itu::moderate(0.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsMargSNR, kTrials, 0xB301);
    printCell("Moderate@0  / 100ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 75 / 100);  // ~75% floor (Mod@0, ~100% actual)
}

void test_moderate_snr_minus5_200ms_aligned() {
    std::printf("[test] moderate_snr_minus5_200ms_aligned\n");
    auto cfg = itu::moderate(-5.0f);
    const auto r = runWattersonCell(cfg, kSymbolMsWeakSNR, kTrials, 0xB401);
    printCell("Moderate@-5 / 200ms / aligned", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 50 / 100);  // ~50% floor (Mod@-5, ~84% actual)
}

void test_good_no_fading_snr10_25ms_reference() {
    std::printf("[test] good_no_fading_snr10_25ms_reference\n");
    auto cfg = itu::good(10.0f);
    cfg.fading_enabled = false;
    cfg.multipath_enabled = false;
    const auto r = runWattersonCell(cfg, kBaselineSymbolMs, kTrials, 0xC001);
    printCell("NoFading@10 / 25ms / reference", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 85 / 100);  // ~85% floor (NoFading@10, ~100% actual)
}

// ============================================================================
// Carrier offset (2026-11-08). An SSB dial offset shifts every tone by the same delta; the
// Goertzel main lobe is 2 / T wide, so an uncompensated 7.3 Hz offset costs the 100 ms rung
// > 6 dB (measure_ack_staircase: -4 dB AWGN 15 % vs 100 % at 0 Hz). The receiver supplies a
// prior (its tracked reverse-link CFO) and the detector searches its stated uncertainty on a
// half-bin grid. AWGN cells (fading off) isolate the frequency effect.
// ============================================================================

WattersonChannel::Config awgnWithCfo(float snr_db, float cfo_hz) {
    auto cfg = itu::good(snr_db);
    cfg.fading_enabled = false;
    cfg.multipath_enabled = false;
    cfg.cfo_enabled = true;
    cfg.cfo_hz = cfo_hz;
    return cfg;
}

void test_cfo_without_prior_breaks_100ms_rung() {
    std::printf("[test] cfo_without_prior_breaks_100ms_rung\n");
    const auto r = runWattersonCell(awgnWithCfo(-4.0f, 7.3f), kSymbolMsMargSNR, kTrials, 0xCF01);
    printCell("AWGN@-4 / 100ms / CFO +7.3 / no prior", r);
    // The defect the prior exists for: if this ever passes, the search is not needed and the
    // plumbing can be revisited — it must not pass by accident of a weaker channel model.
    EXPECT(r.decoded_ok <= kTrials * 60 / 100);
}

void test_cfo_exact_prior_restores_100ms_rung() {
    std::printf("[test] cfo_exact_prior_restores_100ms_rung\n");
    const auto r = runWattersonCell(awgnWithCfo(-4.0f, 7.3f), kSymbolMsMargSNR, kTrials, 0xCF01,
                                    7.3f, 0.0f);
    printCell("AWGN@-4 / 100ms / CFO +7.3 / exact prior", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 90 / 100);
}

void test_cfo_uncertified_prior_restores_100ms_rung() {
    std::printf("[test] cfo_uncertified_prior_restores_100ms_rung\n");
    // A dual-chirp prior carrying a 3 Hz multipath phantom, searched over the 4.8 Hz phantom
    // bound (tau_max 2 ms x chirp rate 4800 Hz/s / 2): 3 hypotheses at 5 Hz spacing.
    const auto r = runWattersonCell(awgnWithCfo(-4.0f, 7.3f), kSymbolMsMargSNR, kTrials, 0xCF01,
                                    7.3f + 3.0f, 4.8f);
    printCell("AWGN@-4 / 100ms / CFO +7.3 / prior +3 Hz off, W 4.8", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 90 / 100);
}

void test_cfo_no_prior_wide_search_25ms() {
    std::printf("[test] cfo_no_prior_wide_search_25ms\n");
    const auto r = runWattersonCell(awgnWithCfo(-2.0f, -20.0f), kBaselineSymbolMs, kTrials,
                                    0xCF02, 0.0f, 50.0f);
    printCell("AWGN@-2 / 25ms / CFO -20 / no prior, W 50", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 90 / 100);
}

void test_cfo_no_prior_wide_search_12ms_large_offset() {
    std::printf("[test] cfo_no_prior_wide_search_12ms_large_offset\n");
    // +45 Hz is 60 % of the 75 Hz tone spacing: uncompensated, the energy lands nearer the
    // NEXT tone than its own.
    const auto r = runWattersonCell(awgnWithCfo(2.0f, 45.0f), kSymbolMsHighSNR, kTrials, 0xCF03,
                                    0.0f, 50.0f);
    printCell("AWGN@+2 / 12ms / CFO +45 / no prior, W 50", r);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= kTrials * 90 / 100);
}

}  // namespace

int main() {
    test_cfo_without_prior_breaks_100ms_rung();
    test_cfo_exact_prior_restores_100ms_rung();
    test_cfo_uncertified_prior_restores_100ms_rung();
    test_cfo_no_prior_wide_search_25ms();
    test_cfo_no_prior_wide_search_12ms_large_offset();
    test_good_snr20_25ms_aligned();
    test_good_snr10_25ms_aligned();
    test_good_snr5_50ms_aligned();
    test_good_snr20_25ms_timing_search();
    test_moderate_snr20_25ms_aligned();
    test_moderate_snr10_50ms_aligned();
    test_moderate_snr5_100ms_aligned();
    test_good_snr0_100ms_aligned();
    test_good_snr_minus5_200ms_aligned();
    test_moderate_snr0_100ms_aligned();
    test_moderate_snr_minus5_200ms_aligned();
    test_good_no_fading_snr10_25ms_reference();

    if (g_failures > 0) {
        std::fprintf(stderr, "\n%d test assertion(s) failed\n", g_failures);
        return 1;
    }
    std::printf("\nAll tone-burst ACK Watterson tests PASSED\n");
    return 0;
}
