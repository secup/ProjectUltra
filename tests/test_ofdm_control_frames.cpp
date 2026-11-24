// Every connected-OFDM control frame type, through the production encoder and a connected
// StreamingDecoder over AWGN, must come out as that frame. Found 2026-11-24 (TNC pingpong, ITU Good
// 8 dB seed 49): DISCONNECT copies whose single codeword decoded (LDPC converged in 12 iterations at
// 2.8 dB LTS SNR) were still escalated as data and lost.
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

namespace {
int failures = 0;
#define CHECK(cond, msg)                                         \
    do {                                                         \
        if (!(cond)) {                                           \
            ++failures;                                          \
            std::cout << "FAIL: " << msg << "\n";                \
        }                                                        \
    } while (0)

ModemConfig ofdmGeometry(Modulation mod, CodeRate rate) {
    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500.0f;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;
    cfg.modulation = mod;
    cfg.code_rate = rate;
    cfg.use_pilots = true;
    cfg.pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    return cfg;
}

// Trials in which the decoder delivered a frame of `want` type.
int decodeTrials(const std::vector<float>& tx, v2::FrameType want, float snr_db, int trials,
                 std::string* got_types) {
    const Modulation mod = Modulation::QPSK;
    const CodeRate rate = CodeRate::R1_2;
    int ok = 0;
    for (int t = 0; t < trials; ++t) {
        std::vector<float> audio(28800, 0.0f);
        audio.insert(audio.end(), tx.begin(), tx.end());
        audio.resize(audio.size() + 48000, 0.0f);
        ultra::ota_channel_core::SimulatedChannel channel;
        channel.setSeed(500 + static_cast<uint64_t>(t));
        channel.configure(snr_db, ultra::ota_channel_core::ChannelType::AWGN);
        channel.transmitFromA(audio);
        const auto rx = channel.receiveForB(audio.size());

        StreamingDecoder decoder;
        decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP, ofdmGeometry(mod, rate),
                                     mod, rate);
        decoder.setKnownCFO(0.0f);
        decoder.expectFullOFDMAnchorOnce();
        bool found = false;
        for (size_t pos = 0; pos < rx.size(); pos += 4800) {
            const size_t len = std::min<size_t>(4800, rx.size() - pos);
            decoder.feedAudio(rx.data() + pos, len);
            decoder.processBuffer();
            while (decoder.hasFrame()) {
                const auto r = decoder.getFrame();
                if (!r.success) continue;
                const auto h = v2::parseHeader(r.frame_data);
                if (got_types && h.valid) *got_types += std::string(v2::frameTypeToString(h.type)) + " ";
                if (h.valid && h.type == want) found = true;
            }
        }
        ok += found ? 1 : 0;
    }
    return ok;
}

}  // namespace

int main() {
    ultra::setLogLevel(std::getenv("TEST_LOG") ? LogLevel::INFO : LogLevel::ERROR);
    const Modulation mod = Modulation::QPSK;
    const CodeRate rate = CodeRate::R1_2;
    struct Case { const char* name; Bytes frame; v2::FrameType type; };
    const Case cases[] = {
        {"DISCONNECT", v2::ControlFrame::makeDisconnect("BRAVO", "ALPHA").serialize(),
         v2::FrameType::DISCONNECT},
        {"TURN_REQUEST", v2::ControlFrame::makeTurnRequest("BRAVO", "ALPHA").serialize(),
         v2::FrameType::TURN_REQUEST},
        {"ACK", v2::ControlFrame::makeAck("BRAVO", "ALPHA", 0xFFFF).serialize(),
         v2::FrameType::ACK},
    };
    for (const auto& c : cases) {
        StreamingEncoder encoder;
        encoder.setMode(protocol::WaveformMode::OFDM_CHIRP);
        encoder.setOFDMConfig(ofdmGeometry(mod, rate));
        encoder.setDataMode(mod, rate);
        const auto tx = encoder.encodeFrame(c.frame);
        CHECK(!tx.empty(), c.name << ": the encoder produced it");
        if (tx.empty()) continue;
        for (const float snr : {20.0f, 10.0f}) {
            constexpr int kTrials = 8;
            std::string types;
            const int ok = decodeTrials(tx, c.type, snr, kTrials, &types);
            std::cout << c.name << " AWGN " << snr << " dB: " << ok << "/" << kTrials
                      << " decoded" << (ok < kTrials ? " (got: " + types + ")" : "") << "\n";
            CHECK(ok == kTrials, c.name << " must decode every time at " << snr << " dB");
        }
    }
    if (failures) {
        std::cout << "OFDMControlFrames: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "OFDMControlFrames: all checks passed\n";
    return 0;
}
