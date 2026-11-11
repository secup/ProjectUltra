// test_tone_burst_ack_roundtrip.cpp — encode -> [optional AWGN] -> decode
// round-trip validation across SNR cells matching PHY_ADAPTATION_DESIGN §15.5.
//
// Channel model: in-band SNR per the project's convention (signal power
// vs noise power inside a 3 kHz reference band). For Fs=48 kHz, noise
// power in 3 kHz = sigma² × 3000/24000 = sigma²/8. For a tone of
// amplitude A, signal power = A²/2. So:
//   SNR_in_band_dB = 10·log10((A²/2) / (sigma²/8)) = 10·log10(4·A²/sigma²)
//   sigma = 2·A / sqrt(10^(SNR_dB/10))

#include "waveform/tone_burst_ack/tone_burst_constants.hpp"
#include "waveform/tone_burst_ack/tone_burst_detector.hpp"
#include "waveform/tone_burst_ack/tone_burst_encoder.hpp"
#include "waveform/tone_burst_ack/tone_burst_payload.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"

#include <cmath>
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

// ============================================================================
// Helpers
// ============================================================================

float sigmaForInBandSNR(float snr_db, float tone_amplitude) {
    return 2.0f * tone_amplitude / std::sqrt(std::pow(10.0f, snr_db / 10.0f));
}

void addAWGN(std::vector<float>& samples, float sigma, std::mt19937& rng) {
    std::normal_distribution<float> dist(0.0f, sigma);
    for (float& s : samples) s += dist(rng);
}

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

// ============================================================================
// Tests
// ============================================================================

void test_encode_output_size_and_amplitude() {
    std::printf("[test] encode_output_size_and_amplitude\n");
    ToneBurstEncoder enc;
    const auto burst = enc.encode(samplePayload(0));
    const uint32_t spp = (kSampleRate * kBaselineSymbolMs) / 1000;
    EXPECT_EQ(burst.size(), static_cast<size_t>(kTotalSymbols) * spp);
    // Peak amplitude bounded by kToneAmplitude (plus tiny numerical slop).
    float peak = 0.0f;
    for (float s : burst) {
        if (std::abs(s) > peak) peak = std::abs(s);
    }
    EXPECT(peak <= kToneAmplitude + 1e-6f);
    EXPECT(peak > 0.99f * kToneAmplitude);  // should actually hit the peak
    std::printf("  peak=%.4f (kToneAmplitude=%.4f)\n", peak, kToneAmplitude);
}

void test_phase_continuity_at_symbol_boundaries() {
    std::printf("[test] phase_continuity_at_symbol_boundaries\n");
    // Encode 2 distinct symbols and verify the audio is C0-continuous at
    // the boundary (no sample-to-sample step > 2·A·sin(2·pi·f_max/Fs)).
    ToneBurstEncoder enc;
    const std::vector<uint8_t> dibits = {0, 3};  // tones 2400 then 2625 Hz
    const auto burst = enc.encodeDibits(dibits);
    const uint32_t spp = (kSampleRate * kBaselineSymbolMs) / 1000;
    // The boundary is at sample index spp; check the step from spp-1 -> spp.
    const float step = std::abs(burst[spp] - burst[spp - 1]);
    // Max sample-to-sample step for a sine at frequency f sampled at Fs:
    //   ~ A · 2·pi·f / Fs
    const float max_step_expected = kToneAmplitude * 2.0f * 3.14159f *
                                     kToneFreqHz.back() /
                                     static_cast<float>(kSampleRate) * 1.05f;
    EXPECT(step < max_step_expected);
    std::printf("  boundary step=%.4f (max expected=%.4f)\n", step, max_step_expected);
}

void test_clean_round_trip_baseline_duration() {
    std::printf("[test] clean_round_trip_baseline_duration\n");
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    for (int v = 0; v < 4; ++v) {
        enc.resetPhase();
        const auto orig = samplePayload(v);
        const auto burst = enc.encode(orig);
        const auto r = det.decodeAligned(burst.data(), burst.size());
        EXPECT(r.payload.has_value());
        EXPECT(r.stats.crc_ok);
        EXPECT_EQ(r.stats.hamming_corrected_blocks, 0);
        if (r.payload) EXPECT(payloadsEqual(*r.payload, orig));
        std::printf("  variant %d: min_conf=%.2f\n", v, r.min_confidence);
    }
}

void test_clean_round_trip_high_snr_duration() {
    std::printf("[test] clean_round_trip_high_snr_duration\n");
    // Faster 12 ms/sym should still round-trip perfectly on a clean channel.
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    const auto orig = samplePayload(2);
    const auto burst = enc.encode(orig, kSymbolMsHighSNR);
    const auto r = det.decodeAligned(burst.data(), burst.size(), kSymbolMsHighSNR);
    EXPECT(r.payload.has_value());
    EXPECT(r.stats.crc_ok);
    if (r.payload) EXPECT(payloadsEqual(*r.payload, orig));
}

void test_costas_correlation_aligned() {
    std::printf("[test] costas_correlation_aligned\n");
    ToneBurstEncoder enc;
    const auto orig = samplePayload(1);
    const auto burst = enc.encode(orig);
    const uint32_t spp = (kSampleRate * kBaselineSymbolMs) / 1000;
    const float corr_at_zero =
        ToneBurstDetector::costasCorrelationAtOffset(burst.data(), burst.size(), 0, spp);
    const float corr_at_one_symbol =
        ToneBurstDetector::costasCorrelationAtOffset(burst.data(), burst.size(), spp, spp);
    std::printf("  corr at offset 0: %.1f\n", corr_at_zero);
    std::printf("  corr at offset 1 symbol: %.1f\n", corr_at_one_symbol);
    // At the correct offset, the Costas correlation should be very large.
    // Misaligned by 1 symbol, the correlation drops sharply because the
    // expected Costas tones at each position no longer match the actual
    // payload tones at those positions (random alignment).
    EXPECT(corr_at_zero > 2.0f * corr_at_one_symbol);
}

// ============================================================================
// AWGN cells — must match the §15.5 detection floor table
// ============================================================================
//
// Per §15.5, the realistic detection floor for 4-FSK + (15,11) Hamming:
//   SNR ≥ +12 dB in-band: should be 100% decode at 25 ms/sym (baseline).
//   SNR ≈  +5 dB        : should still decode reliably with Hamming margin.
//   SNR ≈   0 dB        : marginal at 25 ms; reliable at 50 ms.
//   SNR ≈  -5 dB        : reliable at 100 ms.
//
// We require:
//   - +20 dB: 100/100
//   - +10 dB: 100/100
//   - +5 dB : ≥ 95/100
//   - 0 dB  : at 50 ms/sym, ≥ 95/100  (proves staircase works)
//   - -5 dB : at 100 ms/sym, ≥ 90/100 (proves the lever exists)

struct AWGNTestResult {
    int decoded_ok = 0;
    int decoded_wrong = 0;  // CRC ok but mis-decoded — MUST be 0
    int crc_failed = 0;
    int trials = 0;
};

AWGNTestResult runAWGNCell(float snr_db, uint32_t symbol_ms, int trials,
                            uint32_t seed) {
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    std::mt19937 rng(seed);
    const float sigma = sigmaForInBandSNR(snr_db, kToneAmplitude);

    AWGNTestResult r;
    r.trials = trials;
    for (int t = 0; t < trials; ++t) {
        enc.resetPhase();
        const auto orig = samplePayload(t);
        auto burst = enc.encode(orig, symbol_ms);
        addAWGN(burst, sigma, rng);
        const auto dr = det.decodeAligned(burst.data(), burst.size(), symbol_ms);
        if (dr.payload.has_value()) {
            if (payloadsEqual(*dr.payload, orig)) ++r.decoded_ok;
            else ++r.decoded_wrong;
        } else {
            ++r.crc_failed;
        }
    }
    return r;
}

void test_awgn_plus20_db() {
    std::printf("[test] awgn_plus20_db\n");
    const auto r = runAWGNCell(20.0f, kBaselineSymbolMs, 100, 0x1A1B);
    std::printf("  decoded_ok=%d decoded_wrong=%d crc_failed=%d\n",
                r.decoded_ok, r.decoded_wrong, r.crc_failed);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= 100);
}

void test_awgn_plus10_db() {
    std::printf("[test] awgn_plus10_db\n");
    const auto r = runAWGNCell(10.0f, kBaselineSymbolMs, 100, 0x2A2B);
    std::printf("  decoded_ok=%d decoded_wrong=%d crc_failed=%d\n",
                r.decoded_ok, r.decoded_wrong, r.crc_failed);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= 100);
}

void test_awgn_plus5_db() {
    std::printf("[test] awgn_plus5_db\n");
    const auto r = runAWGNCell(5.0f, kBaselineSymbolMs, 100, 0x3A3B);
    std::printf("  decoded_ok=%d decoded_wrong=%d crc_failed=%d\n",
                r.decoded_ok, r.decoded_wrong, r.crc_failed);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= 95);
}

void test_awgn_0db_with_50ms_symbols() {
    std::printf("[test] awgn_0db_with_50ms_symbols (staircase lever)\n");
    const auto r = runAWGNCell(0.0f, kSymbolMsLowSNR, 100, 0x4A4B);
    std::printf("  decoded_ok=%d decoded_wrong=%d crc_failed=%d\n",
                r.decoded_ok, r.decoded_wrong, r.crc_failed);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= 95);
}

void test_awgn_minus5db_with_100ms_symbols() {
    std::printf("[test] awgn_minus5db_with_100ms_symbols (low-SNR lever)\n");
    const auto r = runAWGNCell(-5.0f, kSymbolMsMargSNR, 100, 0x5A5B);
    std::printf("  decoded_ok=%d decoded_wrong=%d crc_failed=%d\n",
                r.decoded_ok, r.decoded_wrong, r.crc_failed);
    EXPECT_EQ(r.decoded_wrong, 0);
    EXPECT(r.decoded_ok >= 90);
}

// ============================================================================
// detectAndDecode: timing offset recovery on a clean channel
// ============================================================================

void test_detect_and_decode_with_leading_silence() {
    std::printf("[test] detect_and_decode_with_leading_silence\n");
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    const auto orig = samplePayload(2);
    const auto burst = enc.encode(orig);
    // Prepend ~150 ms of silence and append ~150 ms more.
    const uint32_t pad_samples = (kSampleRate * 150u) / 1000u;
    std::vector<float> buf;
    buf.reserve(2 * pad_samples + burst.size());
    buf.insert(buf.end(), pad_samples, 0.0f);
    buf.insert(buf.end(), burst.begin(), burst.end());
    buf.insert(buf.end(), pad_samples, 0.0f);

    // Diagnostic: spot-check costas correlation at a few key offsets.
    const uint32_t spp = (kSampleRate * kBaselineSymbolMs) / 1000;
    std::printf("  diag corr@0    =%.1f\n",
                ToneBurstDetector::costasCorrelationAtOffset(buf.data(), buf.size(), 0, spp));
    std::printf("  diag corr@1216 =%.1f\n",
                ToneBurstDetector::costasCorrelationAtOffset(buf.data(), buf.size(), 1216, spp));
    std::printf("  diag corr@%u    =%.1f\n", pad_samples,
                ToneBurstDetector::costasCorrelationAtOffset(buf.data(), buf.size(), pad_samples, spp));
    std::printf("  diag corr@%u    =%.1f\n", pad_samples + 8,
                ToneBurstDetector::costasCorrelationAtOffset(buf.data(), buf.size(), pad_samples + 8, spp));

    const auto r = det.detectAndDecode(buf.data(), buf.size());
    EXPECT(r.sync_acquired);
    EXPECT(r.decode.payload.has_value());
    if (r.decode.payload) EXPECT(payloadsEqual(*r.decode.payload, orig));
    // Detected offset must be inside the burst region (within one symbol of
    // true Costas start). The candidate-driven scan may pick a slightly
    // sub-peak coarse offset if a payload alias outranks the exact peak,
    // but decode is robust to ~50-sample misalignment because Goertzel
    // still picks up the dominant tone in a 1200-sample window.
    const uint32_t spp_local = (kSampleRate * kBaselineSymbolMs) / 1000;
    const size_t err = (r.detected_offset_samples > pad_samples)
        ? r.detected_offset_samples - pad_samples
        : pad_samples - r.detected_offset_samples;
    std::printf("  detected_offset=%zu (true=%u) err=%zu peak=%.1f\n",
                r.detected_offset_samples, pad_samples, err, r.costas_correlation_peak);
    // < 1 symbol tolerance.
    EXPECT(err < spp_local);
}

void test_detect_and_decode_at_plus5_db_with_silence() {
    std::printf("[test] detect_and_decode_at_plus5_db_with_silence\n");
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    std::mt19937 rng(0x6A6B);
    const float sigma = sigmaForInBandSNR(5.0f, kToneAmplitude);
    int decoded_ok = 0;
    int total = 20;
    for (int t = 0; t < total; ++t) {
        enc.resetPhase();
        const auto orig = samplePayload(t);
        const auto burst = enc.encode(orig);
        const uint32_t pad_samples = (kSampleRate * 100u) / 1000u;
        std::vector<float> buf;
        buf.reserve(2 * pad_samples + burst.size());
        buf.insert(buf.end(), pad_samples, 0.0f);
        buf.insert(buf.end(), burst.begin(), burst.end());
        buf.insert(buf.end(), pad_samples, 0.0f);
        addAWGN(buf, sigma, rng);
        const auto r = det.detectAndDecode(buf.data(), buf.size());
        if (r.decode.payload && payloadsEqual(*r.decode.payload, orig)) ++decoded_ok;
    }
    std::printf("  decoded_ok=%d/%d at +5 dB with timing search\n", decoded_ok, total);
    EXPECT(decoded_ok >= 18);  // 90%+
}

// BUG-TONE-ACK-CLEAN-MISS (2026-11-08): a payload that CONTAINS the Costas sequence has an
// alias that scores like the preamble. With the candidate gate at 3x the window average, a
// window the ACK mostly fills (the monitor's 12 ms tail window: burst + one 100 ms cadence +
// one symbol) admitted only the global maximum, so when the alias outscored a slightly faded
// preamble the clean ACK was lost (GUI gate: group 42, alias at symbols 5-8, CRC fail every
// pass). Candidates are now tried strongest-first above the absolute floor.
void test_costas_alias_in_payload_does_not_hide_the_preamble() {
    std::printf("[test] costas_alias_in_payload_does_not_hide_the_preamble\n");
    // Find payloads whose dibits contain the Costas pattern EARLY enough that the alias is a
    // timing candidate of the monitor's tail window (the full burst must fit from it: alias
    // start <= the 100 ms cadence = burst symbol 8, i.e. payload index <= 4; the GUI-gate case
    // had it at payload index 1).
    std::vector<ToneBurstAckPayload> aliased;
    std::vector<size_t> alias_at;
    for (int g = 0; g < 64 && aliased.size() < 4; ++g) {
        for (int mask = 0; mask < 4 && aliased.size() < 4; ++mask) {
            for (int rate = 0; rate < 8 && aliased.size() < 4; ++rate) {
                ToneBurstAckPayload p;
                p.group_seq = static_cast<uint8_t>(g);
                p.frame_mask = static_cast<uint16_t>(mask);
                p.rate_hint = static_cast<uint8_t>(rate);
                p.type = AckType::Ack;
                const auto d = encodePayloadDibits(p);
                for (size_t i = 0; i <= 4; ++i) {
                    bool match = true;
                    for (uint32_t k = 0; k < kCostasSymbols; ++k) {
                        match = match && d[i + k] == kCostasPattern[k];
                    }
                    if (match) { aliased.push_back(p); alias_at.push_back(i); break; }
                }
            }
        }
    }
    EXPECT(aliased.size() >= 2);
    const uint32_t ms = kSymbolMsHighSNR;
    const uint32_t spp = (kSampleRate * ms) / 1000u;
    std::mt19937 rng(0xA11A5);
    int ok = 0;
    for (size_t n = 0; n < aliased.size(); ++n) {
        const auto& p = aliased[n];
        ToneBurstEncoder enc;
        auto burst = enc.encode(p, ms);
        // The preamble 1 dB down (a slow fade): the alias becomes the global maximum.
        for (size_t i = 0; i < static_cast<size_t>(kCostasSymbols) * spp; ++i) burst[i] *= 0.89f;
        std::vector<float> win(spp, 0.0f);  // monitor tail window geometry
        win.insert(win.end(), burst.begin(), burst.end());
        win.insert(win.end(), 4800, 0.0f);
        addAWGN(win, sigmaForInBandSNR(20.0f, kToneAmplitude), rng);
        ToneBurstDetector det;
        const auto r = det.detectAndDecode(win.data(), win.size(), ms, 32, {});  // production
        const bool good = r.decode.payload && payloadsEqual(*r.decode.payload, p);
        const size_t err = (r.detected_offset_samples > spp) ? r.detected_offset_samples - spp
                                                             : spp - r.detected_offset_samples;
        std::printf("  group %u mask %u rate %u (alias at payload %zu): decoded=%d offset_err=%zu\n",
                    static_cast<unsigned>(p.group_seq), static_cast<unsigned>(p.frame_mask),
                    static_cast<unsigned>(p.rate_hint), alias_at[n], good ? 1 : 0, err);
        if (good && err < spp / 4) ++ok;
    }
    EXPECT_EQ(ok, static_cast<int>(aliased.size()));
}

}  // namespace

// Narrowband band plan (2026-11-10): a narrowband session's ACK lies inside its 500 Hz channel
// (1250-1750 Hz), decodes on a narrowband detector with a dial offset reported as the dial offset,
// and is not mistaken for a wideband ACK (nor the reverse).
void test_narrowband_band_plan() {
    std::printf("[test] narrowband_band_plan\n");
    ToneBurstEncoder enc;
    enc.setBand(ToneAckBand::Narrowband);
    EXPECT(!enc.diversity());
    const auto orig = samplePayload(2);
    const auto burst = enc.encode(orig, 25);

    // Power inside 1250-1750 Hz, by direct DFT on a 5 Hz grid.
    double inside = 0.0, total = 0.0;
    for (int f = 0; f <= 4000; f += 5) {
        double re = 0.0, im = 0.0;
        for (size_t n = 0; n < burst.size(); ++n) {
            const double ph = 2.0 * M_PI * f * static_cast<double>(n) / kSampleRate;
            re += burst[n] * std::cos(ph);
            im -= burst[n] * std::sin(ph);
        }
        const double pw = re * re + im * im;
        total += pw;
        if (f >= 1250 && f <= 1750) inside += pw;
    }
    std::printf("  power inside 1250-1750 Hz: %.2f %%\n", 100.0 * inside / total);
    EXPECT(inside / total > 0.99);

    // +20 Hz dial offset, 150 ms of silence either side, 10 dB in-band SNR.
    std::vector<float> buf((kSampleRate * 150u) / 1000u, 0.0f);
    const size_t start = buf.size();
    for (size_t n = 0; n < burst.size(); ++n) buf.push_back(0.0f);
    buf.insert(buf.end(), (kSampleRate * 150u) / 1000u, 0.0f);
    // The same ACK with a +20 Hz dial offset, synthesized independently of the encoder: the
    // on-air dibits, each tone at its plan frequency + 20 Hz, phase-continuous.
    {
        const auto dibits = buildOnAirDibits(orig);
        const uint32_t spp = (kSampleRate * 25u) / 1000u;
        double phase = 0.0;
        size_t idx = start;
        for (uint8_t d : dibits) {
            const double f = kToneFreqHz[d] + kNarrowbandToneShiftHz + 20.0;
            for (uint32_t n = 0; n < spp; ++n) {
                buf[idx++] = kToneAmplitude * static_cast<float>(std::sin(phase));
                phase += 2.0 * M_PI * f / kSampleRate;
            }
        }
    }
    std::mt19937 rng(7);
    addAWGN(buf, sigmaForInBandSNR(10.0f, kToneAmplitude), rng);

    ToneBurstDetector narrow;
    narrow.setBand(ToneAckBand::Narrowband);
    narrow.setFrequencyPrior(0.0f, kToneAckCfoAcquisitionHalfWidthHz);
    const auto r = narrow.detectAndDecode(buf.data(), buf.size(), 25);
    EXPECT(r.sync_acquired);
    EXPECT(r.decode.payload.has_value());
    if (r.decode.payload) EXPECT(payloadsEqual(*r.decode.payload, orig));
    std::printf("  narrowband detector: acquired=%d offset=%.1f Hz (true +20)\n",
                r.sync_acquired ? 1 : 0, r.detected_frequency_offset_hz);
    EXPECT(std::fabs(r.detected_frequency_offset_hz - 20.0f) < 10.0f);

    ToneBurstDetector wide;  // default wideband plan
    wide.setFrequencyPrior(0.0f, kToneAckCfoAcquisitionHalfWidthHz);
    const auto w = wide.detectAndDecode(buf.data(), buf.size(), 25);
    EXPECT(!w.sync_acquired);

    // And a wideband ACK is not taken for a narrowband one.
    ToneBurstEncoder wide_enc;
    const auto wide_burst = wide_enc.encode(orig, 25);
    std::vector<float> wbuf((kSampleRate * 150u) / 1000u, 0.0f);
    wbuf.insert(wbuf.end(), wide_burst.begin(), wide_burst.end());
    wbuf.insert(wbuf.end(), (kSampleRate * 150u) / 1000u, 0.0f);
    const auto n2 = narrow.detectAndDecode(wbuf.data(), wbuf.size(), 25);
    EXPECT(!n2.sync_acquired);
}

// H0 bound behind kCostasNoiseRatioFloor (BUG-RX-ABSOLUTE-LEVEL): on noise alone no timing x
// frequency candidate reaches the floor, at any symbol duration or offset-search width, so noise
// is never handed to the CRC-12 through the floor.
void test_noise_only_never_reaches_the_floor() {
    std::printf("[test] noise_only_never_reaches_the_floor\n");
    std::mt19937 rng(0x40F0);
    std::normal_distribution<float> n01(0.0f, 1.0f);
    float worst = 0.0f;
    int crc = 0;
    for (uint32_t ms : {12u, 25u, 50u, 100u}) {
        for (float halfwidth : {0.0f, 50.0f}) {
            const uint32_t spp = (kSampleRate * ms) / 1000u;
            const size_t n = static_cast<size_t>(kTotalSymbols) * spp * 5 / 4 + 9600;
            for (int r = 0; r < 12; ++r) {
                std::vector<float> x(n);
                for (float& v : x) v = 0.05f * n01(rng);
                ToneBurstDetector det;
                det.setFrequencyPrior(0.0f, halfwidth);
                const auto res = det.detectAndDecode(x.data(), x.size(), ms, 32, {});
                worst = std::max(worst, res.strongest_candidate_noise_ratio);
                if (res.decode.stats.crc_ok) ++crc;
            }
        }
    }
    std::printf("  strongest noise candidate %.2f x its noise (floor %.2f), CRC passes %d\n", worst,
                kCostasNoiseRatioFloor, crc);
    EXPECT(worst > 1.0f);                      // the statistic is live
    EXPECT(worst < kCostasNoiseRatioFloor);    // and noise never reaches the floor
    EXPECT_EQ(crc, 0);
}

// INV-RX-LEVEL-001: the detector reads no absolute level. The same ACK in the same noise, 40 dB
// quieter or 20 dB louder (the receive level is the operator's AF / USB-audio / sound-card
// setting), decodes identically with the same noise ratio. The old floor (5 % of the TRANSMIT
// amplitude's ideal peak) dropped every ACK more than 26 dB under the transmit level.
void test_detection_is_level_invariant() {
    std::printf("[test] detection_is_level_invariant\n");
    ToneBurstAckPayload p;
    p.group_seq = 17;
    p.frame_mask = 0x0B;
    p.type = AckType::Ack;
    const uint32_t ms = kBaselineSymbolMs;
    ToneBurstEncoder enc;
    const auto burst = enc.encode(p, ms);
    std::vector<float> base(4800, 0.0f);
    base.insert(base.end(), burst.begin(), burst.end());
    base.insert(base.end(), 4800, 0.0f);
    std::mt19937 rng(0x1E7E1);
    addAWGN(base, sigmaForInBandSNR(6.0f, kToneAmplitude), rng);
    float ratio0 = 0.0f;
    for (float gain_db : {0.0f, -40.0f, 20.0f}) {
        const float g = std::pow(10.0f, gain_db / 20.0f);
        std::vector<float> x(base);
        for (float& v : x) v *= g;
        ToneBurstDetector det;
        const auto r = det.detectAndDecode(x.data(), x.size(), ms, 32, {});
        const bool ok = r.decode.payload && payloadsEqual(*r.decode.payload, p);
        std::printf("  gain %+5.1f dB: decoded=%d noise_ratio=%.3f\n", gain_db, ok ? 1 : 0,
                    r.costas_noise_ratio);
        EXPECT(ok);
        if (gain_db == 0.0f) ratio0 = r.costas_noise_ratio;
        else EXPECT(std::abs(r.costas_noise_ratio - ratio0) < 1e-3f * ratio0);
    }
}

int main() {
    test_noise_only_never_reaches_the_floor();
    test_detection_is_level_invariant();
    test_narrowband_band_plan();
    test_costas_alias_in_payload_does_not_hide_the_preamble();
    test_encode_output_size_and_amplitude();
    test_phase_continuity_at_symbol_boundaries();
    test_clean_round_trip_baseline_duration();
    test_clean_round_trip_high_snr_duration();
    test_costas_correlation_aligned();
    test_awgn_plus20_db();
    test_awgn_plus10_db();
    test_awgn_plus5_db();
    test_awgn_0db_with_50ms_symbols();
    test_awgn_minus5db_with_100ms_symbols();
    test_detect_and_decode_with_leading_silence();
    test_detect_and_decode_at_plus5_db_with_silence();
    if (g_failures > 0) {
        std::fprintf(stderr, "\n%d test assertion(s) failed\n", g_failures);
        return 1;
    }
    std::printf("\nAll tone-burst ACK round-trip tests PASSED\n");
    return 0;
}
