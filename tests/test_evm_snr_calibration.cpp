// test_evm_snr_calibration.cpp
//
// Calibrates the RADIO-AGNOSTIC decision-directed EVM SNR estimator. The value under
// test is OFDMDemodulator::getEvmSnrDb(), surfaced via StreamingDecoder: the gain-
// corrected scatter of the equalized data constellation around its own hard decisions,
//     S_dd = Sum|decision|^2,  S_ee = Sum|eq|^2,  S_de = Sum Re(decision*conj(eq)),
//     SNR  = S_dd / (S_dd - S_de^2/S_ee)   (best-fit scalar gain removed).
//
// This measures the USABLE (effective) SNR — the SNR the demod's own decisions
// actually experience AFTER its channel estimation, unlike the LTS estimator which
// measures the CHANNEL (thermal) SNR from the pilot-noise residual. On AWGN the two
// agree while thermal noise dominates the error vector; ABOVE ~16-18 dB the usable SNR
// saturates at the demod's implementation ceiling (pilot-estimate residual, ICI,
// residual phase) — always <= thermal, never above. That is exactly the quantity rate
// selection needs, and its cardinal virtue is that it is CONSTANT-FREE and CANNOT
// INFLATE: signal power is measured (S_dd), not referenced, so the +8.70
// kOfdmLegacyAnchorScaleOffsetDb bench fudge (which pushed the rig 14.1 -> 22.8 and
// manufactured the 16QAM over-commit) is impossible by construction.
//
// Harness: identical known-truth injection to test_ofdm_snr_calibration.cpp — real
// StreamingEncoder -> SimulatedChannel(AWGN) -> StreamingDecoder, LTS/data section
// scaled to the channel's in-band reference so the AWGN dial IS the true receiver
// in-band SNR (3 kHz convention).
//
// GATES (per sweep row):
//   TRACK  (within a modulation's real operating/selection range): |mean - truth| <= tol
//           AND mean <= truth + 1.5  (honest, no offset).
//   HEADROOM (above the operating range, where the next modulation up is selected):
//           mean <= truth + 1.5 only (NO-INFLATION guard — proves the +8.70 cannot
//           return; under-reading toward the usable ceiling here is expected & safe).
//   MONOTONIC: readings non-decreasing in truth within each modulation.
// tol via env ULTRA_EVM_SNR_CAL_TOL_DB (default 2.0 dB).

#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "ota_channel_core/models.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
using Bytes = std::vector<uint8_t>;

namespace {

std::vector<float> withSilence(const std::vector<float>& frame) {
    std::vector<float> audio;
    audio.resize(48000, 0.0f);
    audio.insert(audio.end(), frame.begin(), frame.end());
    audio.resize(audio.size() + 96000, 0.0f);
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

ModemConfig ofdmGeometry(Modulation mod, CodeRate rate) {
    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500.0f;
    cfg.cp_mode = CyclicPrefixMode::LONG;
    cfg.modulation = mod;
    cfg.code_rate = rate;
    cfg.use_pilots = true;
    cfg.pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    return cfg;
}

double rmsOf(const std::vector<float>& x) {
    double sum = 0.0;
    for (float v : x) sum += static_cast<double>(v) * v;
    return x.empty() ? 0.0 : std::sqrt(sum / static_cast<double>(x.size()));
}

double ltsDataSectionRms(const Bytes& serialized, Modulation mod, CodeRate rate) {
    StreamingEncoder probe;
    probe.setMode(protocol::WaveformMode::OFDM_CHIRP);
    probe.setOFDMConfig(ofdmGeometry(mod, rate));
    probe.setDataMode(mod, rate);
    return rmsOf(probe.encodeFrameLight(serialized));
}

// One trial's EVM SNR reading (NaN if the decoder never produced one).
float measureEvmSNR(float snr_db, uint32_t seed, const Bytes& payload,
                    Modulation mod, CodeRate rate, bool fading = false) {
    auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", 7, payload, rate);
    const Bytes serialized = frame.serialize();

    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::OFDM_CHIRP);
    encoder.setOFDMConfig(ofdmGeometry(mod, rate));
    encoder.setDataMode(mod, rate);
    auto samples = encoder.encodeFrame(serialized);
    if (samples.empty()) return std::nanf("");

    // Known-truth injection: put the LTS/data section AT the channel's reference so the
    // AWGN dial IS the injected in-band SNR (identical to test_ofdm_snr_calibration).
    const double section_rms = ltsDataSectionRms(serialized, mod, rate);
    if (section_rms <= 0.0) return std::nanf("");
    const float scale = static_cast<float>(
        ultra::ota_channel_core::kModemReferenceInBandRms / section_rms);
    for (float& v : samples) v *= scale;

    auto audio = withSilence(samples);

    if (fading) {
        // ITU-R F.1487 Good = 0.1 Hz Doppler / 0.5 ms delay Watterson — the SAME channel
        // the IONOS bench calls MPG. Same known-truth noise convention as the AWGN arm, so
        // the only difference between the two sections is the fading itself: any bias delta
        // is attributable to fading and nothing else. One seed samples ONE fade state; the
        // ENSEMBLE across seeds is the fade average (ergodicity). Readings are recorded even
        // for frames that fail to decode, so there is no survivor bias (unlike the live ring).
        ultra::ota_channel_core::WattersonChannelModel model(
            ultra::ota_channel_core::itu_r_f1487::good(snr_db), seed);
        std::vector<float> faded;
        model.process(audio, 0, faded);
        audio = std::move(faded);
    } else {
        ultra::ota_channel_core::SimulatedChannel channel;
        channel.setSeed(seed);
        channel.configure(snr_db, ultra::ota_channel_core::ChannelType::AWGN);
        channel.setTxBurstNormalizationEnabled(false);
        channel.transmitFromA(audio);
        audio = channel.receiveForB(audio.size());
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("EVMCAL");
    decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                                 ofdmGeometry(mod, rate), mod, rate);
    // A session's receiver holds the handshake's CFO (setKnownCFO at connect); the truth here is
    // 0 Hz. Without it the decoder is cold and acquires on the burst's own CFO measurement
    // (sigma 0.1-0.4 Hz, BUG-STRUCTURE-ACQ-DROPS-CFO): that measures acquisition, not this
    // estimator.
    decoder.setKnownCFO(0.0f);
    feedInChunks(decoder, audio);
    // The EVM SNR is captured on each decoded frame (result.evm_snr_db) — average the
    // per-frame readings; fall back to the decoder's decode-time atomic otherwise.
    double sum = 0.0;
    int n = 0;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.is_ping) continue;
        if (result.has_evm_snr_db && std::isfinite(result.evm_snr_db)) {
            sum += result.evm_snr_db;
            ++n;
        }
    }
    if (n > 0) return static_cast<float>(sum / n);
    if (decoder.hasLastEvmSnr()) return decoder.getLastEvmSnrDb();
    return std::nanf("");
}

float toleranceDb() {
    if (const char* env = std::getenv("ULTRA_EVM_SNR_CAL_TOL_DB")) {
        char* end = nullptr;
        const float v = std::strtof(env, &end);
        if (end != env && std::isfinite(v) && v > 0.0f) return v;
    }
    return 2.0f;
}

struct Row {
    Modulation mod;
    CodeRate rate;
    const char* name;
    float snr;
    bool operating;  // true: TRACK gate (in selection range); false: HEADROOM (no-inflate only)
};

constexpr float kNoInflateMarginDb = 1.5f;  // usable SNR must never exceed thermal by more

}  // namespace

int main() {
    setLogLevel(LogLevel::ERROR);

    Bytes payload(32);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>(0x40 + (i * 7) % 64);

    constexpr uint32_t kSeeds = 5;
    const float tol = toleranceDb();

    std::cout << "== Radio-agnostic EVM (usable) SNR estimator calibration ==\n";
    std::cout << "   Gain-corrected decision-directed error vector; measures USABLE SNR.\n";
    std::cout << "   Constant-free -> cannot inflate (the +8.70 bench fudge is impossible).\n";
    std::cout << "   TRACK |bias|<=" << std::fixed << std::setprecision(1) << tol
              << " dB in operating range; HEADROOM: no-inflate only (<= truth+"
              << kNoInflateMarginDb << ").\n\n";

    // QPSK is selected at low SNR (R1/4 floor ~10); above ~15 dB higher-order takes over.
    // 16QAM is selected ~16-20 dB; 24 is headroom (32QAM/higher territory).
    const std::vector<Row> rows = {
        {Modulation::QPSK,  CodeRate::R1_4, "QPSK R1/4",  10.f, true},
        {Modulation::QPSK,  CodeRate::R1_4, "QPSK R1/4",  12.f, true},
        {Modulation::QPSK,  CodeRate::R1_4, "QPSK R1/4",  15.f, true},
        {Modulation::QPSK,  CodeRate::R1_4, "QPSK R1/4",  20.f, false},
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2", 16.f, true},
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2", 18.f, true},
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2", 20.f, true},
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2", 24.f, false},
    };

    std::cout << "  modulation | true | mean  bias | gate    result (of " << kSeeds << " seeds)\n";
    int failed = 0;
    const char* prev_name = nullptr;
    double prev_mean = -1e9;
    for (const auto& r : rows) {
        if (prev_name && std::string(prev_name) != r.name) prev_mean = -1e9;  // reset per mod
        double sum = 0.0;
        int n = 0;
        for (uint32_t s = 0; s < kSeeds; ++s) {
            const float v = measureEvmSNR(r.snr, 1000u + s * 37u, payload, r.mod, r.rate);
            if (std::isfinite(v)) { sum += v; ++n; }
        }
        if (n == 0) {
            std::cout << "  " << std::setw(10) << r.name << " | " << std::setw(4)
                      << std::setprecision(0) << r.snr << " | (no reading)  FAIL\n";
            ++failed;
            prev_name = r.name;
            continue;
        }
        const double mean = sum / n;
        const double bias = mean - r.snr;
        const bool no_inflate = mean <= r.snr + kNoInflateMarginDb;
        const bool monotonic = mean >= prev_mean - 0.5;  // non-decreasing (small slack)
        bool ok = no_inflate && monotonic;
        if (r.operating) ok = ok && std::abs(bias) <= tol;
        std::cout << "  " << std::setw(10) << r.name << " | " << std::setw(4)
                  << std::setprecision(0) << r.snr << " | " << std::setprecision(1)
                  << std::setw(5) << mean << " " << std::showpos << std::setw(5) << bias
                  << std::noshowpos << " | " << (r.operating ? "TRACK   " : "HEADROOM")
                  << (ok ? " OK" : " FAIL")
                  << (!no_inflate ? " (INFLATED!)" : "")
                  << (!monotonic ? " (non-monotonic)" : "") << "\n";
        if (!ok) ++failed;
        prev_mean = mean;
        prev_name = r.name;
    }

    std::cout << "\n";
    // ── FADING ENSEMBLE (2026-07-26) ──────────────────────────────────────────────
    // WHY: the whole table above is AWGN-only. The live rig reports EVM "usable" ~10 dB
    // while the IONOS dial says 20 on MPG (= ITU Good), and that 10 dB gap was being
    // attributed to hardware/implementation loss. A decision-directed estimator is
    // EXPECTED to read low on fading (wrong decisions feed back into the error vector, and
    // the error vector blows up in fades so the average is dominated by the worst moments),
    // so the gap must be decomposed before any of it is called real loss.
    // The linear (power) mean over the seed ensemble is the fade average; the dB-mean sits
    // below it by Jensen. Reported, and gated only loosely, because the point is the NUMBER.
    std::cout << "\n== FADING (ITU Good = 0.1 Hz / 0.5 ms = IONOS MPG) ==\n";
    std::cout << "  mod        | dial | lin-mean  bias | dB-mean  bias | n\n";
    std::vector<std::pair<std::string, double>> fading_bias;
    for (const auto& fr : std::vector<Row>{
             {Modulation::QPSK,  CodeRate::R1_4, "QPSK R1/4",  10.f, true},
             {Modulation::QPSK,  CodeRate::R1_4, "QPSK R1/4",  20.f, true},
             {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2", 20.f, true}}) {
        double lin = 0.0, dbs = 0.0;
        int n = 0;
        for (uint32_t s2 = 0; s2 < 16; ++s2) {
            const float v = measureEvmSNR(fr.snr, 0x9000u + s2 * 104729u, payload,
                                          fr.mod, fr.rate, /*fading=*/true);
            if (std::isfinite(v)) { lin += std::pow(10.0, v / 10.0); dbs += v; ++n; }
        }
        const double lin_db = n ? 10.0 * std::log10(lin / n) : std::nan("");
        const double db_mean = n ? dbs / n : std::nan("");
        std::cout << "  " << std::left << std::setw(10) << fr.name << std::right
                  << " | " << std::setw(4) << (int)fr.snr << " | " << std::fixed
                  << std::setprecision(1) << std::setw(8) << lin_db << " "
                  << std::showpos << std::setw(5) << (lin_db - fr.snr) << std::noshowpos
                  << " | " << std::setw(7) << db_mean << " " << std::showpos
                  << std::setw(5) << (db_mean - fr.snr) << std::noshowpos
                  << " | " << n << "\n";
        if (std::isfinite(lin_db))
            fading_bias.emplace_back(std::string(fr.name) + "@" + std::to_string((int)fr.snr),
                                     lin_db - fr.snr);
    }
    // GATE (2026-07-26): the MEASURED fading bias is -1.8 dB at dial 10 and -5.2..-6.8 dB
    // at dial 20 (16-seed ensembles). Two properties are pinned:
    //   (a) NEVER INFLATES on fading either -- the estimator's core safety claim must hold
    //       on a fading channel, not just AWGN;
    //   (b) the bias must not DEGRADE past -9 dB, which would mean the estimator has become
    //       useless on the channel class the rig actually runs.
    // This CHARACTERISES a known bias rather than validating accuracy: a decision-directed
    // estimator genuinely reads low on fading. It exists so the bias cannot silently grow,
    // and so nobody again reads a fading EVM number as if it were the true usable SNR.
    for (const auto& fb : fading_bias) {
        if (!(fb.second <= kNoInflateMarginDb)) {
            ++failed;
            std::cout << "  FAIL: fading bias " << fb.first << " = " << fb.second
                      << " dB INFLATES (must stay <= +" << kNoInflateMarginDb << ")\n";
        }
        if (!(fb.second >= -9.0)) {
            ++failed;
            std::cout << "  FAIL: fading bias " << fb.first << " = " << fb.second
                      << " dB degraded past the -9 dB floor\n";
        }
    }

    if (failed == 0) {
        std::cout << "PASS: usable-SNR EVM tracks thermal in range, never inflates, monotonic.\n";
        return 0;
    }
    std::cout << "FAIL: " << failed << " row(s) failed.\n";
    return 1;
}
