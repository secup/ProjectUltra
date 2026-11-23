// lowsnr_floor: decode rate of one MC-DPSK-family frame through the PRODUCTION encoder/decoder
// (StreamingEncoder / StreamingDecoder, MC_DPSK mode) on a peak-limited transmitter, versus the GUI
// gate's --snr-db (noise sized from the 0.3048 in-band reference; every frame scaled to the same
// 0.5 PEAK by normalizeTxBurstForHardware, as --tx-model hardware does) with a dial offset.
// Compares the payload modulations a weak link can use, at their own code rates and lengths.
//
// Usage: lowsnr_floor [--channel awgn|good|moderate|poor] [--trials N] [--cfo HZ]
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "ota_channel_core/models.hpp"
#include "protocol/frame_v2.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "sync/chirp_sync.hpp"
#include "ultra/logging.hpp"
#include "ultra/tx_burst_normalization.hpp"
#include "ultra/peak_control.hpp"

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace ultra;
namespace v2 = ultra::protocol::v2;

namespace {

struct Case {
    const char* name;
    Modulation mod;
    CodeRate rate;
    int mfsk_tones;
    size_t payload;
};

}  // namespace

// --connect: the handshake frame (a real ConnectFrame) as DBPSK R1/4 x2 and as the M-FSK handshake
// frame, both through the production encoder with ModemEngine's control clip (h = 6), decoded by a
// handshake-state (not connected) decoder.
static int runConnect(int trials, float cfo, ota_channel_core::ChannelType ch_type,
                      const std::vector<float>& snrs) {
    const Bytes ser = v2::ConnectFrame::makeConnect("ALPHA", "BRAVO", 0x0F, 0xFF).serialize();
    std::printf("CONNECT decode rate (production encoder + control clip h=6, 0.5 peak), "
                "CFO %.1f Hz, %d trials\n%-14s %6s |", cfo, trials, "frame", "air_s");
    for (float s : snrs) std::printf(" %4.0f", s);
    std::printf("\n");
    const int only_mfsk = std::getenv("LOW_MFSK") ? std::atoi(std::getenv("LOW_MFSK")) : -1;
    for (int mfsk = 0; mfsk <= 1; ++mfsk) {
        if (only_mfsk >= 0 && mfsk != only_mfsk) continue;
        auto cfg = ultra::mc_dpsk_presets::level8();
        cfg.samples_per_symbol = 1024;
        std::vector<int> ok(snrs.size(), 0), via_mfsk(snrs.size(), 0), nfade(snrs.size(), 0);
        std::vector<double> snr_sum(snrs.size(), 0.0), fade_sum(snrs.size(), 0.0);
        double air = 0;
        for (size_t si = 0; si < snrs.size(); ++si) {
            for (int t = 1; t <= trials; ++t) {
                gui::StreamingEncoder enc;
                enc.setMode(protocol::WaveformMode::MC_DPSK);
                enc.setMCDPSKConfig(cfg);
                enc.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
                enc.setHandshakeMfskOnce(mfsk != 0);
                auto frame = enc.encodeFrame(ser);
                air = double(frame.size()) / 48000.0;
                ultra::peak_control::apply(frame, 6.0f, 48000.0f, 250.0f, 2750.0f);
                (void)sim::normalizeTxBurstForHardware(frame, 0.5f);
                // The responder hears the initiator's PING first (it PONGs, then expects a
                // CONNECT): a cold decoder takes a first-contact frame whose first codeword fails
                // for a PING.
                std::vector<float> audio(48000, 0.0f);
                {
                    sync::ChirpSync ping(cfg.getChirpConfig());
                    auto p = ping.generate();
                    (void)sim::normalizeTxBurstForHardware(p, 0.5f);
                    audio.insert(audio.end(), p.begin(), p.end());
                    audio.resize(audio.size() + 96000, 0.0f);
                }
                audio.insert(audio.end(), frame.begin(), frame.end());
                audio.resize(audio.size() + 96000, 0.0f);
                ota_channel_core::SimulatedChannel ch;
                ch.setSeed(uint64_t(t) * 1000 + si);
                ch.configure(snrs[si], ch_type);
                ch.setTxCFO(cfo);
                ch.transmitFromA(audio);
                audio = ch.receiveForB(audio.size());
                {
                    static const auto taps = ota_channel_core::modemBandpassFirCoefficients();
                    std::vector<float> y(audio.size(), 0.0f);
                    const size_t L = taps.size(), d = L / 2;
                    for (size_t n = 0; n < audio.size(); ++n) {
                        double acc = 0.0;
                        for (size_t k = 0; k < L; ++k) {
                            const long idx = long(n) + long(d) - long(k);
                            if (idx >= 0 && idx < long(audio.size())) acc += double(taps[k]) * audio[size_t(idx)];
                        }
                        y[n] = float(acc);
                    }
                    audio.swap(y);
                }
                gui::StreamingDecoder dec;
                dec.setLogPrefix("LOW");
                dec.setMode(protocol::WaveformMode::MC_DPSK, false);
                dec.setMCDPSKConfig(cfg);
                dec.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
                for (size_t pos = 0; pos < audio.size(); pos += 4800) {
                    dec.feedAudio(audio.data() + pos, std::min<size_t>(4800, audio.size() - pos));
                    dec.processBuffer();
                }
                while (dec.hasFrame()) {
                    auto r = dec.getFrame();
                    if (r.success && v2::isConnectFrame(r.frame_type)) {
                        if (dec.hasLastFadingIndex()) { fade_sum[si] += dec.getLastFadingIndex(); ++nfade[si]; }
                        ++ok[si];
                        via_mfsk[si] += r.handshake_mfsk;
                        snr_sum[si] += r.has_mcdpsk_data_aided_snr_db ? r.mcdpsk_data_aided_snr_db : -99;
                        break;
                    }
                }
            }
        }
        std::printf("%-14s %6.2f |", mfsk ? "16FSK R1/2x1" : "DBPSK R1/4x2", air);
        for (int v : ok) std::printf(" %4d", v);
        std::printf("   (mfsk-decoded:");
        for (int v : via_mfsk) std::printf(" %d", v);
        std::printf(")  reading:");
        for (size_t i = 0; i < snrs.size(); ++i) std::printf(" %.1f", ok[i] ? snr_sum[i] / ok[i] : -99.0);
        std::printf("  fading:");
        for (size_t i = 0; i < snrs.size(); ++i) std::printf(" %.2f", nfade[i] ? fade_sum[i] / nfade[i] : -1.0);
        std::printf("\n");
        std::fflush(stdout);
    }
    return 0;
}

int main(int argc, char** argv) {
    int trials = 20;
    float cfo = 7.3f;
    auto ch_type = ota_channel_core::ChannelType::AWGN;
    std::string ch_name = "awgn";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--trials") && i + 1 < argc) trials = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--cfo") && i + 1 < argc) cfo = float(std::atof(argv[++i]));
        else if (!std::strcmp(argv[i], "--channel") && i + 1 < argc) {
            ch_name = argv[++i];
            if (ch_name == "good") ch_type = ota_channel_core::ChannelType::GOOD;
            else if (ch_name == "moderate") ch_type = ota_channel_core::ChannelType::MODERATE;
            else if (ch_name == "poor") ch_type = ota_channel_core::ChannelType::POOR;
        }
    }
    bool connect_mode = false;
    for (int i = 1; i < argc; ++i) if (!std::strcmp(argv[i], "--connect")) connect_mode = true;
    setLogLevel(std::getenv("LOW_DEBUG") ? LogLevel::DEBUG : LogLevel::ERROR);
    const char* only = std::getenv("LOW_ONLY");
    const std::vector<Case> cases = {
        {"DBPSK  R1/4", Modulation::DBPSK, CodeRate::R1_4, 0, 30},
        {"16FSK  R1/4", Modulation::MFSK16, CodeRate::R1_4, 0, 30},
        {"16FSK  R1/2", Modulation::MFSK16, CodeRate::R1_2, 0, 30},
        {"DQPSK  R1/4", Modulation::DQPSK, CodeRate::R1_4, 0, 30},
        {"16FSK-F", Modulation::MFSK16_FAST, CodeRate::R1_4, 0, 30},
    };
    // LOW_COMPACT=1: the session's compact DATA frame (one turbo codeword of 3 x 648 air bits, half
    // for 16-FSK-F; 52 payload bytes, the production MC-DPSK geometry) instead of the v2 LDPC chain:
    // what a connected session actually sends at each rung.
    const bool compact = std::getenv("LOW_COMPACT") != nullptr;
    std::vector<float> snrs = {-14, -12, -10, -8, -6, -4, -2, 0};
    if (const char* e = std::getenv("LOW_SNR")) snrs = {float(std::atof(e))};
    if (connect_mode) return runConnect(trials, cfo, ch_type, snrs);
    std::printf("Frame decode rate, production encoder/decoder, peak-limited TX (0.5 peak), "
                "channel %s, CFO %.1f Hz, %d trials\n%-14s %6s |", ch_name.c_str(), cfo, trials,
                "payload", "air_s");
    for (float s : snrs) std::printf(" %4.0f", s);
    std::printf("\n");
    for (const auto& c : cases) {
        if (only && std::string(c.name).find(only) == std::string::npos) continue;
        auto cfg = ultra::mc_dpsk_presets::level8();
        cfg.samples_per_symbol = 1024;
        cfg.mfsk_tones = c.mfsk_tones;
        const Bytes payload(compact ? 52 : c.payload, 0x5A);
        const Bytes ser =
            v2::DataFrame::makeData("ALPHA", "BRAVO", 7, payload, c.rate).serialize();
        std::vector<int> ok(snrs.size(), 0), nread(snrs.size(), 0);
        std::vector<double> reading(snrs.size(), 0.0);
        double air = 0;
        for (size_t si = 0; si < snrs.size(); ++si) {
            for (int t = 1; t <= trials; ++t) {
                gui::StreamingEncoder enc;
                enc.setMode(protocol::WaveformMode::MC_DPSK);
                enc.setMCDPSKConfig(cfg);
                enc.setDataMode(c.mod, c.rate);
                if (compact) {
                    enc.setFixedFrameCodewords(3);
                    enc.setMcDpskCompactFrames(true);
                }
                auto frame = enc.encodeFrame(ser);
                air = double(frame.size()) / 48000.0;
                // ModemEngine clips MC-DPSK data bursts at h = 0 (applyPeakControl, 8x1024 tones)
                if (std::getenv("LOW_CLIP")) ultra::peak_control::apply(frame, 0.0f, 48000.0f, 250.0f, 2750.0f);
                (void)sim::normalizeTxBurstForHardware(frame, 0.5f);
                std::vector<float> audio(48000, 0.0f);
                audio.insert(audio.end(), frame.begin(), frame.end());
                // A connected receiver peeks the longest frame any rung may send (~7 s): give it
                // that much air after the frame, as a live stream would.
                audio.resize(audio.size() + (compact ? 480000 : 96000), 0.0f);
                ota_channel_core::SimulatedChannel ch;
                ch.setSeed(uint64_t(t) * 1000 + si);
                ch.configure(snrs[si], ch_type);
                ch.setTxCFO(cfo);
                ch.transmitFromA(audio);
                audio = ch.receiveForB(audio.size());
                // SimulatedChannel's noise is white over the whole 24 kHz stream; a radio's IF
                // filter (and OTASim's AWGN, which adds its noise band-limited) leaves only the
                // ~3 kHz passband. The in-band SNR is the same either way, but the chirp detector's
                // normalized correlation reads the window's TOTAL noise: without the band limit it
                // sees ~9 dB more noise than any receiver behind an SSB filter.
                {
                    static const auto taps = ota_channel_core::modemBandpassFirCoefficients();
                    std::vector<float> y(audio.size(), 0.0f);
                    const size_t L = taps.size(), d = L / 2;
                    for (size_t n = 0; n < audio.size(); ++n) {
                        double acc = 0.0;
                        for (size_t k = 0; k < L; ++k) {
                            const long idx = long(n) + long(d) - long(k);
                            if (idx >= 0 && idx < long(audio.size())) acc += double(taps[k]) * audio[size_t(idx)];
                        }
                        y[n] = float(acc);
                    }
                    audio.swap(y);
                }
                gui::StreamingDecoder dec;
                dec.setLogPrefix("LOW");
                dec.setMode(protocol::WaveformMode::MC_DPSK, std::getenv("LOW_DISCONNECTED") == nullptr);
                dec.setMCDPSKConfig(cfg);
                dec.setDataMode(c.mod, c.rate);
                if (compact) {
                    dec.setFixedFrameCodewords(3);
                    dec.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
                }
                for (size_t pos = 0; pos < audio.size(); pos += 4800) {
                    dec.feedAudio(audio.data() + pos, std::min<size_t>(4800, audio.size() - pos));
                    dec.processBuffer();
                }
                bool got = false;
                while (dec.hasFrame()) {
                    auto r = dec.getFrame();
                    if (r.success && !got) {
                        got = true;
                        if (r.has_mcdpsk_data_aided_snr_db) { reading[si] += r.mcdpsk_data_aided_snr_db; ++nread[si]; }
                    }
                }
                ok[si] += got;
            }
        }
        std::printf("%-14s %6.2f |", c.name, air);
        for (int v : ok) std::printf(" %4d", v);
        std::printf("   reading:");
        for (size_t i = 0; i < snrs.size(); ++i) std::printf(" %.1f", nread[i] ? reading[i] / nread[i] : -99.0);
        std::printf("\n");
        std::fflush(stdout);
    }
    return 0;
}
