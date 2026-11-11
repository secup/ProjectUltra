// test_mcdpsk_snr_calibration.cpp
//
// Characterizes the MC-DPSK training-SNR estimator
// (MultiCarrierDPSKDemodulator::updateTrainingSNREstimate, surfaced as
// MCDPSK_IN_BAND). It feeds an MC-DPSK CONNECT-geometry frame through the real
// StreamingEncoder -> AWGN(SimulatedChannel) -> StreamingDecoder path at a swept,
// KNOWN true SNR and reports the routed mcdpsk_in_band estimate vs the truth.
//
// WHY: the estimator computes 10*log10(signal_power/residual_power) where the
// residual comes from a TIME-DOMAIN reconstruction (fitted = 2*Re(sum_c H_c*train*
// carrier); residual = sample - fitted). Any reconstruction mismatch (timing,
// phase, the 2*Re passband form, carrier-basis non-orthogonality, in-sample LS
// fit) leaves an alpha*signal residual floor that does NOT vanish at high SNR, so
// the measured SNR SATURATES at ~1/alpha (rig: 16 dB @ true~20, 18.7 dB @ true~32).
// This harness makes that curve repeatable and is the gate for any fix: a proper
// estimator must track the true SNR (within a few dB) well past 18 dB.
//
// Report-only table today; PASS criterion lands with the residual-floor fix.
//
// #58 BUG-CONNECT-SNR-VARIANCE extension: also characterizes + gates the
// DATA-AIDED whole-frame estimator (fade-averaged sibling, routed as
// mcdpsk_in_band when the frame decodes). AWGN gate: within 1 dB, 0-25 dB.

#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "ota_channel_core/models.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <span>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

namespace {

using Bytes = std::vector<uint8_t>;

std::vector<float> withSilence(const std::vector<float>& frame) {
    std::vector<float> audio;
    audio.resize(48000, 0.0f);  // 1 s lead-in for chirp-search / noise-floor estimation
    audio.insert(audio.end(), frame.begin(), frame.end());
    audio.resize(audio.size() + 96000, 0.0f);  // trailing room for final decode
    return audio;
}

void feedInChunks(StreamingDecoder& decoder, const std::vector<float>& audio) {
    constexpr size_t kChunk = 4800;
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        decoder.feedAudio(audio.data() + pos, len);
        decoder.processBuffer();
    }
}

// CONNECT-class geometry: 8 carriers, 1024 samples/symbol (46.875 baud), DQPSK.
MultiCarrierDPSKConfig connectGeometry() {
    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::level8();
    cfg.samples_per_symbol = 1024;
    return cfg;
}

// Both MC-DPSK in-band estimates for a frame at true `snr_db` (NaN if absent):
// training = ~170 ms preamble snapshot; data_aided = whole-frame differential
// estimate (#58, only populated when the frame demodulated with enough symbols).
struct MeasuredSNR {
    float training = std::nanf("");
    float data_aided = std::nanf("");
    float cfo_hz = std::nanf("");  // the decoder's CFO estimate for the frame
};

MeasuredSNR measureEstimatedSNR(float snr_db, uint32_t seed, const Bytes& payload,
                                ultra::ota_channel_core::ChannelType chan =
                                    ultra::ota_channel_core::ChannelType::AWGN,
                                float cfo_hz = 0.0f) {
    MeasuredSNR out;
    auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", 7, payload, CodeRate::R1_4);
    const Bytes serialized = frame.serialize();

    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(connectGeometry());
    encoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
    auto samples = encoder.encodeFrame(serialized);
    if (samples.empty()) return out;

    auto audio = withSilence(samples);

    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(seed);
    channel.configure(snr_db, chan);
    channel.transmitFromA(audio);
    audio = channel.receiveForB(audio.size());
    if (cfo_hz != 0.0f) {
        // The receiver's dial offset, as OTASim applies it (streaming Hilbert shifter).
        ultra::ota_channel_core::ChannelConfig shift;
        shift.type = ultra::ota_channel_core::ChannelType::PASSTHROUGH;
        shift.cfo_hz = cfo_hz;
        ultra::ota_channel_core::RngRoot root(seed ^ 0xCF0u);
        auto model = ultra::ota_channel_core::createChannelModel(shift, root, "cal-cfo");
        audio = model->process(std::span<const float>(audio.data(), audio.size()));
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("CAL");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);  // connected -> routes MCDPSK_IN_BAND
    decoder.setMCDPSKConfig(connectGeometry());
    decoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);

    feedInChunks(decoder, audio);
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.has_mcdpsk_training_snr_db &&
            std::isfinite(result.mcdpsk_training_snr_db)) {
            out.training = result.mcdpsk_training_snr_db;
        }
        if (result.has_mcdpsk_data_aided_snr_db &&
            std::isfinite(result.mcdpsk_data_aided_snr_db)) {
            out.data_aided = result.mcdpsk_data_aided_snr_db;
        }
        out.cfo_hz = result.cfo_hz;
    }
    return out;
}

}  // namespace

int main() {
    Bytes payload(96);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>(0x40 + (i * 7) % 64);

    using CT = ultra::ota_channel_core::ChannelType;
    std::cout << "== MC-DPSK training-SNR estimator calibration: measured vs true ==\n";
    std::cout << "   AWGN proves the ESTIMATOR; GOOD (CCIR multipath, 0.1 Hz/0.5 ms)\n";
    std::cout << "   shows the FADING coherence limit over the 8-symbol (~170 ms) window.\n\n";
    std::cout << " true | AWGN  err  | GOOD  err   (mean of 5 noise seeds)\n";

    int awgn_sat = 0, awgn_hi = 0;
    for (float snr : {6.f, 9.f, 12.f, 15.f, 18.f, 21.f, 24.f, 27.f, 30.f, 35.f}) {
        auto meanFor = [&](CT ct) -> double {
            double sum = 0.0; int n = 0;
            for (uint32_t s = 0; s < 5; ++s) {
                float e = measureEstimatedSNR(snr, 0x1000u + s * 7919u, payload, ct).training;
                if (std::isfinite(e)) { sum += e; ++n; }
            }
            return n ? sum / n : std::nan("");
        };
        const double ma = meanFor(CT::AWGN);
        const double mg = meanFor(CT::GOOD);
        std::cout << std::setw(5) << (int)snr << " | " << std::fixed << std::setprecision(1);
        if (std::isfinite(ma)) std::cout << std::setw(4) << ma << " " << std::showpos
                                         << std::setw(5) << (ma - snr) << std::noshowpos;
        else std::cout << "  -- (no est)";
        std::cout << " | ";
        if (std::isfinite(mg)) std::cout << std::setw(4) << mg << " " << std::showpos
                                         << std::setw(5) << (mg - snr) << std::noshowpos;
        else std::cout << "  -- (no est)";
        std::cout << "\n";
        if (snr >= 18.f) { ++awgn_hi; if (std::isfinite(ma) && (ma - snr) < -3.0) ++awgn_sat; }
    }
    std::cout << "\nAWGN high-SNR (>=18) saturating (>3 dB low): " << awgn_sat << "/" << awgn_hi
              << "   <- estimator is sound iff this is 0\n";
    std::cout << "GOOD caps well below true at high SNR = the FADING coherence limit\n";
    std::cout << "(the estimator correctly reports the EFFECTIVE SNR through the fading channel)\n";

    // ========================================================================
    // #58 BUG-CONNECT-SNR-VARIANCE: data-aided whole-frame estimator (the
    // fade-averaged sibling routed at connect time when the frame decodes).
    // AWGN proves the ESTIMATOR CALIBRATION; the GOOD column is informational
    // (fade-averaged effective SNR sits below the AWGN-equivalent dial value
    // by the Jensen penalty -- that is the intended behavior, not a defect).
    // ========================================================================
    std::cout << "\n== #58 data-aided whole-frame estimator: measured vs true ==\n";
    std::cout << " true | AWGN  err  | GOOD  err   (mean of 5 noise seeds)\n";
    int da_fail = 0;
    for (float snr : {0.f, 5.f, 10.f, 15.f, 20.f, 25.f}) {
        auto meanFor = [&](CT ct) -> double {
            double sum = 0.0; int n = 0;
            for (uint32_t s = 0; s < 5; ++s) {
                float e = measureEstimatedSNR(snr, 0x2000u + s * 7919u, payload, ct).data_aided;
                if (std::isfinite(e)) { sum += e; ++n; }
            }
            return n ? sum / n : std::nan("");
        };
        const double ma = meanFor(CT::AWGN);
        const double mg = meanFor(CT::GOOD);
        std::cout << std::setw(5) << (int)snr << " | " << std::fixed << std::setprecision(1);
        if (std::isfinite(ma)) std::cout << std::setw(4) << ma << " " << std::showpos
                                         << std::setw(5) << (ma - snr) << std::noshowpos;
        else std::cout << "  -- (no est)";
        std::cout << " | ";
        if (std::isfinite(mg)) std::cout << std::setw(4) << mg << " " << std::showpos
                                         << std::setw(5) << (mg - snr) << std::noshowpos;
        else std::cout << "  -- (no est)";
        std::cout << "\n";
        // GATE (AWGN): |mean err| <= 1.0 dB at every point 0..25 dB. Measured
        // table at kDataAidedResidualCalDb = +0.5 (5 seeds/point, 2026-07-01):
        //   true   0    5    10   15   20   25
        //   err   +0.5 -0.1 -0.1 -0.0 +0.4 +0.1  (dB, estimate - true)
        // Calibration story (full derivation in multi_carrier_dpsk.hpp):
        //   * The naive differential "+3.01 dB" correction is NOT applied: the
        //     unit-phasor error metric discards the radial noise half, which
        //     cancels the differential doubling exactly.
        //   * Without the geometry-derived ICI subtraction the estimate
        //     saturated (measured -1.7 dB @15, -3.2 @20, -5.8 @25): the
        //     carriers are non-orthogonal on the symbol grid and the computed
        //     leakage (-29 dB/carrier for this geometry) matched the measured
        //     excess error, so it is subtracted analytically, not tuned.
        //   * kDataAidedResidualCalDb = +0.5 pins the remaining first-order
        //     drop (second-order differential noise term): measured errors at
        //     cal=0 were -0.4..-1.0 dB across 0..15 dB.
        // Hard-decision bias (wrong decisions shrink the error, over-reading
        // SNR) is the +0.5 at 0 dB true -- small even there because the
        // CONNECT geometry's per-carrier SNR is in-band + ~9 dB; it stays
        // well under the 1.5 dB acceptance above 3 dB true (consumers care
        // about 5-20 dB).
        if (!std::isfinite(ma) || std::abs(ma - snr) > 1.0) ++da_fail;
    }
    std::cout << "GOOD reads the fade-AVERAGED effective SNR (below dial = Jensen penalty, intended)\n";

    // REGRESSION GATE 1: the MC-DPSK training SNR estimator must remain LEVEL-/SNR-
    // accurate in AWGN (it backs the connect-time ratiometric rate decision). It must
    // NOT saturate at high SNR in AWGN -- any saturation there would be a real estimator
    // defect (vs the GOOD column, where the cap is the physical fading coherence limit,
    // which is correct and intentionally NOT gated).
    if (awgn_sat > 0) {
        std::cout << "\nFAIL: AWGN estimate saturated at high SNR (" << awgn_sat
                  << " point(s)) -- estimator regression.\n";
        return 1;
    }
    // REGRESSION GATE 2 (#58): the data-aided estimator must track true SNR in
    // AWGN within 1 dB across 0..25 dB (it is the value ROUTED at connect time
    // whenever the frame decodes).
    if (da_fail > 0) {
        std::cout << "\nFAIL: data-aided AWGN estimate off by > 1 dB at " << da_fail
                  << " point(s) -- #58 estimator regression.\n";
        return 1;
    }

    // REGRESSION GATE 3 (BUG-MCDPSK-TRAINING-SNR-CFO): both estimates are invariant to the dial
    // offset. The correlator removes the CFO, so the training fit must put it back; until
    // 2026-11-11 it did not, and at +7.3 Hz the training estimate read -2.4 dB at ANY SNR (the
    // fitted model kept its power but lost its phase). Gate: the mean estimate at each offset
    // within 1 dB of the same seeds at 0 Hz, over MC-DPSK's operating range (6, 12 dB: the
    // training value is routed only for a failed MC-DPSK frame, below the OFDM entry floor plus
    // the crossing margin). 18-30 dB are printed, not gated: there the training estimate under-
    // reads at large offsets, non-monotonically (BUG-MCDPSK-TRAINING-SNR-HIGH-CFO, open).
    // The 0.01 Hz column engages OTASim's shifter with no offset: it separates the shifter's band-
    // limiting (which alone lifts the data-aided estimate ~1 dB at 24 dB) from the offset.
    std::cout << "\n== CFO invariance (AWGN, mean of 5 seeds): training / data-aided ==\n";
    std::cout << " true |  0 Hz         | 0.01 Hz       | +7.3 Hz       | -23.5 Hz\n";
    int cfo_fail = 0;
    for (float snr : {6.f, 12.f, 18.f, 24.f, 30.f}) {
        const bool gated = snr <= 12.f;
        std::cout << std::setw(5) << (int)snr << " |" << std::fixed << std::setprecision(1);
        double ref_tr = 0.0, ref_da = 0.0;
        for (float cfo : {0.0f, 0.01f, 7.3f, -23.5f}) {
            double tr = 0.0, da = 0.0, cf = 0.0;
            int ntr = 0, nda = 0, ncf = 0;
            for (uint32_t s = 0; s < 5; ++s) {
                const MeasuredSNR m =
                    measureEstimatedSNR(snr, 0x3000u + s * 7919u, payload, CT::AWGN, cfo);
                if (std::isfinite(m.training)) { tr += m.training; ++ntr; }
                if (std::isfinite(m.data_aided)) { da += m.data_aided; ++nda; }
                if (std::isfinite(m.cfo_hz)) { cf += m.cfo_hz; ++ncf; }
            }
            tr = ntr ? tr / ntr : std::nan("");
            da = nda ? da / nda : std::nan("");
            cf = ncf ? cf / ncf : std::nan("");
            std::cout << " " << std::setw(5) << tr << " / " << std::setw(5) << da << " (cfo "
                      << std::setprecision(2) << cf << std::setprecision(1) << ") |";
            if (cfo == 0.0f) {
                ref_tr = tr;
                ref_da = da;
            } else if (gated && (!std::isfinite(tr) || !std::isfinite(da) ||
                                 std::abs(tr - ref_tr) > 1.0 || std::abs(da - ref_da) > 1.0)) {
                ++cfo_fail;
            }
        }
        std::cout << (gated ? "" : "   (not gated)") << "\n";
    }
    if (cfo_fail > 0) {
        std::cout << "\nFAIL: an SNR estimate moved > 1 dB with the dial offset at " << cfo_fail
                  << " point(s) -- BUG-MCDPSK-TRAINING-SNR-CFO regression.\n";
        return 1;
    }
    std::cout << "\nPASS: AWGN training estimate tracks true SNR (no saturation); "
                 "data-aided estimate within 1 dB (0-25 dB AWGN); both CFO-invariant over 6-12 dB.\n";
    return 0;
}
