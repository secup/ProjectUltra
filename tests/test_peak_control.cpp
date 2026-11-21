// Peak control (include/ultra/peak_control.hpp, docs/PEAK_CONTROL_DESIGN_2026_11_21.md).
//
// 1. The headroom choice is derived from the rung's threshold: it never decreases as the threshold
//    rises (a denser constellation needs a cleaner waveform), and every rung gets a positive gain.
// 2. The cross-rung threshold carry and the EVM prediction are inverses, and the identity carry
//    (a == b) returns the threshold unchanged.
// 3. The clipper on a real production burst (ModemEngine::transmitBurst): the crest factor falls to
//    within 1 dB of the measured curve's level, the T/R silence stays exact zeros, and the energy
//    outside the pass band is gone.

#include "gui/modem/modem_engine.hpp"
#include "ofdm/pilot_pattern.hpp"
#include "protocol/evm_rate_selector.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"
#include "ultra/peak_control.hpp"

#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

using namespace ultra;
namespace pc = ultra::peak_control;

static int g_fail = 0;
static void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_fail;
}

int main() {
    // 1. headroom monotone in the threshold, gain positive for every rung
    float prev_h = -1.0f;
    bool monotone = true, positive = true;
    for (uint8_t r = protocol::kRungIdxQpskR14; r < protocol::kRungIdxCount; ++r) {
        const float eta = protocol::evmDecodeThresholdTurboDb(r);
        const auto c = pc::chooseHeadroom(eta);
        std::printf("  rung %u eta %.2f -> h %.0f dB, gain %+.2f dB, D %.1f dB\n", r, eta, c.h_db,
                    c.gain_db, c.sdr_db);
        if (c.h_db < prev_h) monotone = false;
        if (!(c.gain_db > 0.0f)) positive = false;
        prev_h = c.h_db;
    }
    check(monotone, "headroom never decreases with the rung's threshold");
    check(positive, "every wideband rung gets a positive predicted gain");

    // 2. carry and prediction are inverses; identity carry is exact
    bool inverse = true, identity = true;
    for (uint8_t a = protocol::kRungIdxQpskR14; a < protocol::kRungIdxCount; ++a) {
        for (uint8_t b = protocol::kRungIdxQpskR14; b < protocol::kRungIdxCount; ++b) {
            const auto ca = pc::chooseHeadroom(protocol::evmDecodeThresholdTurboDb(a));
            const auto cb = pc::chooseHeadroom(protocol::evmDecodeThresholdTurboDb(b));
            const float eta_b = protocol::evmDecodeThresholdTurboDb(b);
            const float on_a = pc::thresholdOnFramesOfRungDb(eta_b, ca, cb);
            if (on_a >= 99.0f) continue;
            const float back = pc::predictEvmAcrossRungsDb(on_a, ca, cb);
            if (std::fabs(back - eta_b) > 1e-3f) inverse = false;
            if (a == b && std::fabs(on_a - eta_b) > 1e-3f) identity = false;
        }
    }
    check(inverse, "predictEvmAcrossRungsDb inverts thresholdOnFramesOfRungDb (1e-3 dB)");
    check(identity, "carry onto the same rung returns the threshold unchanged");
    // A hard-clipped QPSK R1/2 frame (h=1, D=12.85) reads at most its own distortion floor, so the
    // 16QAM R2/3 threshold (11.75 on 16QAM's own frames, h=4: G 5.54, D 18.45) maps BELOW it on QPSK
    // frames: thermal needed at 16QAM 1/(1/11.75dB - 1/18.45dB) = 12.79 dB, +1.30 dB power
    // difference = 14.09 dB at QPSK, read through QPSK's floor = 10.42 dB (hand calculation).
    {
        const auto q = pc::chooseHeadroom(protocol::evmDecodeThresholdTurboDb(protocol::kRungIdxQpskR12));
        const auto m = pc::chooseHeadroom(protocol::evmDecodeThresholdTurboDb(protocol::kRungIdxQam16R23));
        const float eta = protocol::evmDecodeThresholdTurboDb(protocol::kRungIdxQam16R23);
        const float on_q = pc::thresholdOnFramesOfRungDb(eta, q, m);
        std::printf("  16QAM R2/3 threshold %.2f dB on its own frames, %.2f dB on QPSK R1/2 frames\n",
                    eta, on_q);
        check(std::fabs(on_q - 10.42f) < 0.05f && on_q < q.sdr_db,
              "16QAM R2/3 threshold on QPSK R1/2 frames = 10.42 dB, below QPSK's clip floor");
    }

    // MC-DPSK: thresholds moved into the tones' bands (8 x 2 x 46.875 Hz = 750 Hz of 3000: +6.02 dB),
    // and both constellations take the hardest clip (their thresholds sit ~10 dB under the floor D)
    {
        const float tb = pc::mcDpskToneBandThresholdDb(false, 8, 1024);
        const float tq = pc::mcDpskToneBandThresholdDb(true, 8, 1024);
        const auto cb = pc::chooseHeadroomOnCurve(pc::mcDpsk8x1024Curve(), tb);
        const auto cq = pc::chooseHeadroomOnCurve(pc::mcDpsk8x1024Curve(), tq);
        std::printf("  MC-DPSK DBPSK eta %.2f -> h %.0f (%+.2f dB), DQPSK eta %.2f -> h %.0f (%+.2f dB)\n",
                    tb, cb.h_db, cb.gain_db, tq, cq.h_db, cq.gain_db);
        check(std::fabs(tb - 0.02f) < 0.01f && std::fabs(tq - 2.52f) < 0.01f,
              "MC-DPSK tone-band thresholds 0.02 / 2.52 dB");
        check(cb.h_db == 0.0f && cq.h_db == 0.0f && cb.gain_db > 4.5f && cq.gain_db > 4.5f,
              "MC-DPSK DBPSK and DQPSK clip at h=0 for more than 4.5 dB");
    }

    // 3. a real burst through the clipper
    gui::ModemEngine tx;
    tx.joinConnectedSession(protocol::WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_2);
    tx.setMcDpskSessionAddresses(protocol::v2::hashCallsign("ALPHA"), protocol::v2::hashCallsign("BRAVO"));
    const int cw = tx.getFixedFrameCodewords();
    const size_t cap = protocol::v2::getFixedFramePayloadCapacity(CodeRate::R1_2, cw);
    std::mt19937 rng(7);
    std::vector<Bytes> frames;
    for (int f = 0; f < 4; ++f) {
        Bytes p(cap);
        for (auto& b : p) b = static_cast<uint8_t>(rng());
        frames.push_back(protocol::v2::makeFixedDataFrame("ALPHA", "BRAVO", uint16_t(f + 1), p,
                                                         CodeRate::R1_2, cw).serialize());
    }
    std::vector<float> burst = tx.transmitBurst(frames, 0);  // knob off in the test: unclipped
    check(!burst.empty(), "production burst built");
    size_t lead = 0;
    while (lead < burst.size() && burst[lead] == 0.0f) ++lead;
    const auto& cfg = tx.getConfig();
    const float df = float(cfg.sample_rate) / float(cfg.fft_size);
    const float lo = ofdm_pilots::carrierFrequencyHzForLogical(cfg, 0) - 3 * df;
    const float hi = ofdm_pilots::carrierFrequencyHzForLogical(cfg, cfg.num_carriers - 1) + 3 * df;
    const auto choice = pc::chooseHeadroom(protocol::evmDecodeThresholdTurboDb(protocol::kRungIdxQpskR12));
    const auto r = pc::apply(burst, choice.h_db, float(cfg.sample_rate), lo, hi);
    std::printf("  QPSK R1/2 burst: h %.0f dB, crest %.2f -> %.2f dB\n", choice.h_db, r.pre_crest_db,
                r.post_crest_db);
    check(r.applied, "clipper applied");
    check(r.pre_crest_db > 8.0f, "unclipped burst crest is the measured ~10 dB");
    check(r.post_crest_db < choice.h_db + 3.0f, "clipped crest within 3 dB of the headroom (regrowth)");
    check(r.pre_crest_db - r.post_crest_db > 5.0f, "clip removes more than 5 dB of crest at h=1");
    bool silent = true;
    for (size_t i = 0; i < lead; ++i) silent = silent && burst[i] == 0.0f;
    check(silent, "T/R lead-in silence stays exact zeros");
    // energy outside the band (DFT of the active span, one-sided)
    {
        size_t e = burst.size();
        while (e > lead && burst[e - 1] == 0.0f) --e;
        const size_t n = e - lead;
        double in = 0, out = 0;
        const size_t N = 1 << 14;  // a 16k-sample slice from the middle
        const size_t s0 = lead + n / 2 - N / 2;
        for (size_t k = 1; k < N / 2; k += 3) {
            std::complex<double> acc(0, 0);
            for (size_t i = 0; i < N; ++i)
                acc += double(burst[s0 + i]) * std::polar(1.0, -2.0 * M_PI * double(k * i) / double(N));
            const double f = double(cfg.sample_rate) * double(k) / double(N);
            ((f >= lo - 50 && f <= hi + 50) ? in : out) += std::norm(acc);
        }
        const double ratio_db = 10 * std::log10(out / in);
        std::printf("  out-of-band / in-band energy %.1f dB\n", ratio_db);
        check(ratio_db < -30.0, "clipped burst stays inside its band (< -30 dB outside)");
    }
    std::printf("%s\n", g_fail ? "FAILED" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
