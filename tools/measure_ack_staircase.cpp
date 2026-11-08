// Measure the tone-burst ACK staircase edges ON THE HONEST SNR SCALE.
//
// WHY THIS EXISTS
// ---------------
// src/gui/app.cpp adds connection_policy::kOfdmLegacyAnchorScaleOffsetDb (+8.70 dB) to
// OFDM-sourced SNR before consulting symbolMsForSNR(). That offset is a COMPATIBILITY SHIM,
// not a physical quantity: the OFDM SNR estimator used to read ~8.7 dB high (a chirp-power
// conversion error, LS |h|^2 inflation, a guard-bin under-read and fading optimism), those
// bugs were fixed at source on 2026-07-07, and the staircase EDGES were left tuned to the
// old, inflated scale. Adding the bias back makes the old edges line up again.
//
// Folding +8.70 into the edge constants would be arithmetically identical and therefore
// NOT A FIX — it would bury the number in four magic thresholds where it is LESS visible
// than the named constant is today. The only honest way to delete it is to MEASURE what the
// edges actually are on the honest scale and replace the table.
//
// This tool measures exactly that.
//
// METHOD
// ------
// For each ACK symbol duration, sweep in-band SNR and measure the decode success rate of a
// real encode -> channel -> detect+decode round trip (the same path production uses,
// including the timing search: real ACK arrival time is always uncertain). The EDGE for a
// duration is the lowest SNR at which it clears the target success rate.
//
// symbolMsForSNR() should then return the SHORTEST duration whose measured edge is <= the
// reading. That table needs no offset, because it is measured in the same units the
// estimator now reports.
//
// The SNR here is the channel model's own snr_db, i.e. the generator's setting — an EXTERNAL
// reference, not anything the modem estimates. That is what makes the result honest and what
// makes it immune to whatever the estimator's calibration happens to be.
//
// Fading and AWGN are measured SEPARATELY because symbolMsForSNR already takes a
// fading_present flag and the two edges genuinely differ (a deep null inside a short symbol
// is unrecoverable; a longer symbol averages over it).
//
// Build:  cmake --build build --target measure_ack_staircase
// Run:    ./build/measure_ack_staircase [trials_per_cell] [fading_only=0|1] [snr_lo] [snr_hi]
//                                       [diversity=0|1] [short_only=0|1]
//         diversity toggles the two-subband ACK (kToneFreqHzDiversity) on BOTH encoder and
//         detector at EQUAL PEAK (the encoder keeps the single-tone peak), i.e. the physical,
//         PEP-limited comparison; rungs: 0 = all, 1 = 12/25 ms, 2 = 100/200 ms, 3 = 50 ms;
//         cfo_hz (arg 7) applies a constant carrier offset (a real SSB dial offset);
//         prior_err_hz (arg 8) / prior_halfwidth_hz (arg 9) give the detector the receiver's
//         CFO prior = cfo + err searched over +-halfwidth (production: tracked CFO, 4.8 Hz)
//         (fading_only=1 skips AWGN; snr_lo/hi narrow the sweep — use ~1000 trials to
//          resolve the 1-2 % loss rates that cost a full RTO each on fading)

#include "ota_channel_core/models.hpp"
#include "waveform/tone_burst_ack/tone_burst_constants.hpp"
#include "waveform/tone_burst_ack/tone_burst_encoder.hpp"
#include "waveform/tone_burst_ack/tone_burst_detector.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <span>
#include <vector>

using namespace ultra;
using namespace ultra::waveform::tone_burst_ack;
using ultra::ota_channel_core::WattersonChannel;

namespace {

constexpr uint32_t kFs = 48000;

// Target success rate defining an edge. An ACK is protected by the sender's ARQ, but a lost
// ACK costs a full retransmission cycle (~9.5 s), so the bar is high. 0.95 mirrors the
// reliability the existing edges were chosen for.
constexpr double kEdgeTargetSuccess = 0.95;

ToneBurstAckPayload samplePayload(int t) {
    ToneBurstAckPayload p{};
    p.group_seq = static_cast<uint8_t>(t & 0x3F);
    p.frame_mask = static_cast<uint16_t>(0xA5A5u ^ static_cast<uint16_t>(t));
    p.type = AckType::Ack;
    return p;
}

bool payloadsEqual(const ToneBurstAckPayload& a, const ToneBurstAckPayload& b) {
    return a.group_seq == b.group_seq && a.frame_mask == b.frame_mask && a.type == b.type;
}

// One (SNR, symbol_ms) cell: fraction of trials whose payload decoded correctly.
double runCell(const WattersonChannel::Config& cfg, uint32_t symbol_ms, int trials,
               uint64_t base_seed, bool diversity, float prior_hz, float halfwidth_hz) {
    ToneBurstEncoder enc;
    ToneBurstDetector det;
    enc.setDiversity(diversity);
    det.setDiversity(diversity);
    det.setFrequencyPrior(prior_hz, halfwidth_hz);
    int ok = 0;
    for (int t = 0; t < trials; ++t) {
        WattersonChannel ch(cfg, base_seed + static_cast<uint64_t>(t));
        // Warm-up so the fading oscillators start "live" rather than at their constant-1
        // initial transient, which would flatter short symbols.
        std::vector<float> warm_in(static_cast<size_t>(kFs) * 50 / 1000, 0.0f);
        std::vector<float> warm_out;
        ch.process(std::span<const float>(warm_in), warm_out);

        enc.resetPhase();
        const auto orig = samplePayload(t);
        const auto burst = enc.encode(orig, symbol_ms);

        const uint32_t pad = (kFs * 100u) / 1000u;
        std::vector<float> in;
        in.reserve(2 * pad + burst.size());
        in.insert(in.end(), pad, 0.0f);
        in.insert(in.end(), burst.begin(), burst.end());
        in.insert(in.end(), pad, 0.0f);

        std::vector<float> noisy;
        ch.process(std::span<const float>(in), noisy);

        const auto dr = det.detectAndDecode(noisy.data(), noisy.size(), symbol_ms);
        if (dr.decode.payload.has_value() && payloadsEqual(*dr.decode.payload, orig)) ++ok;
    }
    return static_cast<double>(ok) / static_cast<double>(trials);
}

struct Sweep {
    const char* label;
    bool fading;
    float doppler_hz;
    float delay_ms;
};

}  // namespace

// BENCH mode: CPU of one production monitor detection pass per rung (the production tail
// window: one burst + the 100 ms armed cadence + one symbol; 32-sample coarse sweep), on
// noise-only audio — the steady state of an armed monitor waiting for an ACK.
int runBench(bool diversity, float halfwidth_hz) {
    ToneBurstDetector det;
    det.setDiversity(diversity);
    det.setFrequencyPrior(0.0f, halfwidth_hz);
    std::mt19937 rng(7);
    std::normal_distribution<float> n01(0.0f, 0.1f);
    const uint32_t rungs[] = {12, 25, 50, 100};
    double pass_ms = 0.0;
    for (uint32_t ms : rungs) {
        const uint32_t spp = (kFs * ms) / 1000u;
        const size_t window = static_cast<size_t>(kTotalSymbols) * spp + 4800 + spp;
        std::vector<float> x(window);
        for (float& v : x) v = n01(rng);
        const int reps = (ms >= 50) ? 10 : 40;
        const auto t0 = std::chrono::steady_clock::now();
        for (int r = 0; r < reps; ++r) {
            (void)det.detectAndDecode(x.data(), x.size(), ms, 32, {});
        }
        const double per =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                .count() / reps;
        pass_ms += per;
        std::printf("  %3u ms rung: window=%zu  hypotheses=%d  %.2f ms/pass\n", ms, window,
                    2 * ToneBurstDetector::frequencyHypothesesPerSide(halfwidth_hz, ms) + 1, per);
    }
    std::printf("  one armed pass (all rungs): %.2f ms -> %.1f %% of one core at the 100 ms "
                "armed cadence (diversity=%d, prior half-width %.1f Hz)\n", pass_ms,
                pass_ms / 100.0 * 100.0, diversity ? 1 : 0, halfwidth_hz);
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "bench") == 0) {
        const float w = (argc > 2) ? static_cast<float>(std::atof(argv[2])) : 0.0f;
        std::printf("bench, diversity OFF\n"); runBench(false, w);
        std::printf("bench, diversity ON\n");  return runBench(true, w);
    }
    const int trials = (argc > 1) ? std::atoi(argv[1]) : 24;
    const bool fading_only = (argc > 2) && std::atoi(argv[2]) != 0;
    const float snr_lo = (argc > 3) ? static_cast<float>(std::atof(argv[3])) : -10.0f;
    const float snr_hi = (argc > 4) ? static_cast<float>(std::atof(argv[4])) : 26.0f;
    const bool diversity = (argc > 5) ? std::atoi(argv[5]) != 0 : toneAckDiversityEnabled();
    const int rung_sel = (argc > 6) ? std::atoi(argv[6]) : 0;
    const float cfo_hz = (argc > 7) ? static_cast<float>(std::atof(argv[7])) : 0.0f;
    const float prior_err_hz = (argc > 8) ? static_cast<float>(std::atof(argv[8])) : 0.0f;
    const float prior_w_hz = (argc > 9) ? static_cast<float>(std::atof(argv[9])) : 0.0f;
    const bool use_prior = argc > 8;
    const int first_d = (rung_sel == 2) ? 3 : (rung_sel == 3) ? 2 : 0;
    const int n_durations = (rung_sel == 1) ? 2 : (rung_sel == 3) ? 3 : 5;
    const uint32_t durations[] = {kSymbolMsHighSNR, kSymbolMsMidSNR, kSymbolMsLowSNR,
                                  kSymbolMsMargSNR, kSymbolMsWeakSNR};
    // ITU-R F.1487 "Good" is the class the staircase's fading edge is for.
    const Sweep sweeps[] = {
        {"AWGN  (fading_present=false)", false, 0.0f, 0.0f},
        {"GOOD  (fading_present=true)", true, 0.1f, 0.5f},
        {"MODERATE (fading_present=true)", true, 0.5f, 1.0f},
    };

    std::printf("Tone-burst ACK staircase — MEASURED on the honest in-band SNR scale\n");
    std::printf("trials/cell=%d  target success=%.0f%%  diversity=%s  cfo=%.1f Hz  prior=%s\n",
                trials, kEdgeTargetSuccess * 100, diversity ? "ON (two subbands)" : "OFF", cfo_hz,
                use_prior ? "cfo+err" : "none (0 Hz, no search)");
    if (use_prior) std::printf("prior error %+.1f Hz, search half-width %.1f Hz\n", prior_err_hz, prior_w_hz);
    std::printf("(SNR is the CHANNEL GENERATOR's setting — an external reference, not a\n"
                " modem estimate, so this result cannot inherit an estimator's calibration)\n\n");

    for (const auto& sw : sweeps) {
        if (fading_only && !sw.fading) continue;
        std::printf("=== %s ===\n", sw.label);
        std::printf("%8s", "SNR dB");
        for (int i = first_d; i < n_durations; ++i) std::printf("%9u", durations[i]);
        std::printf("   <- symbol_ms\n");

        // Track, per duration, the lowest SNR meeting the target.
        double edge[5];
        for (int i = 0; i < 5; ++i) edge[i] = 1e9;

        for (float snr = snr_lo; snr <= snr_hi; snr += 2.0f) {
            WattersonChannel::Config cfg;
            cfg.snr_db = snr;
            cfg.sample_rate = kFs;
            cfg.fading_enabled = sw.fading;
            cfg.multipath_enabled = sw.fading;
            cfg.doppler_spread_hz = sw.doppler_hz;
            cfg.delay_spread_ms = sw.delay_ms;
            cfg.noise_enabled = true;
            cfg.cfo_enabled = (cfo_hz != 0.0f);
            cfg.cfo_hz = cfo_hz;

            std::printf("%8.0f", snr);
            for (int i = first_d; i < n_durations; ++i) {
                const double p = runCell(cfg, durations[i], trials, 0xACE0u + i * 977u, diversity,
                                         use_prior ? cfo_hz + prior_err_hz : 0.0f,
                                         use_prior ? prior_w_hz : 0.0f);
                std::printf("%8.1f%%", p * 100.0);
                if (p >= kEdgeTargetSuccess && snr < edge[i]) edge[i] = snr;
            }
            std::printf("\n");
            std::fflush(stdout);
        }

        std::printf("\n  MEASURED EDGES (lowest SNR clearing %.0f%%):\n", kEdgeTargetSuccess * 100);
        for (int i = first_d; i < n_durations; ++i) {
            if (edge[i] > 1e8) std::printf("    %3u ms : never reached in the swept range\n", durations[i]);
            else               std::printf("    %3u ms : %+.0f dB\n", durations[i], edge[i]);
        }
        std::printf("\n  => symbolMsForSNR should return the SHORTEST duration whose edge "
                    "(+%.0f dB margin) <= snr.\n     Current table (%s):", kAckEdgeMarginDb,
                    sw.fading ? "fading" : "AWGN");
        for (float s = -20.0f; s <= 30.0f; s += 0.5f) {
            const uint32_t here = symbolMsForSNR(s, sw.fading);
            if (s == -20.0f || here != symbolMsForSNR(s - 0.5f, sw.fading)) {
                std::printf("  %u ms from %+.1f dB", here, s);
            }
        }
        std::printf("\n\n");
    }
    return 0;
}
