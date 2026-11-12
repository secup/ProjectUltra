// Dual-listen at the engine (docs/DUAL_LISTEN_DESIGN_2026_11_11.md): a receiver connected on one
// wideband waveform delivers what the sender sent on the other. The sender crosses the way a
// session's crossing callback does (ModemEngine::crossConnectedWaveform); the test stands in for the
// receiver's protocol: on the engine's waveform report it crosses too, as the binding's
// noteRxWaveform -> Connection::applyWaveformCrossing -> crossConnectedWaveform chain does. The
// report must come before the frame, and the frame must reach a receiver already on its waveform.

#include "gui/modem/modem_engine.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"

#include <atomic>
#include <cstdio>
#include <thread>
#include <cstdlib>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;
using protocol::WaveformMode;

namespace {

int failures = 0;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}

const std::string kMcText = "dual-listen: an MC-DPSK frame";
const std::string kOfdmText = "dual-listen: and an OFDM frame";

struct SenderAudio {
    std::vector<float> mcdpsk;
    std::vector<float> ofdm;
    int ofdm_cw = 0;
};

// ALPHA in a wideband session: down to MC-DPSK (the follow target, ROBUST_MID: DBPSK, 8 carriers,
// 1024 samples/symbol, 3 codewords, compact frames), one DATA frame; back up to OFDM, one frame.
SenderAudio senderAudio() {
    SenderAudio out;
    ModemEngine tx;
    tx.setLogPrefix("ALPHA");
    tx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    tx.setMcDpskSessionAddresses(v2::hashCallsign("ALPHA"), v2::hashCallsign("BRAVO"));
    tx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    out.mcdpsk = tx.transmit(v2::DataFrame::makeData("ALPHA", "BRAVO", 1, kMcText).serialize());
    const int cw = protocol::connection_policy::recommendCWCount(Modulation::QPSK, CodeRate::R1_4,
                                                                 WaveformMode::OFDM_CHIRP);
    tx.crossConnectedWaveform(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4, cw, 0, 0);
    out.ofdm_cw = tx.getFixedFrameCodewords();
    out.ofdm = tx.transmit(v2::makeFixedDataFrame("ALPHA", "BRAVO", 2,
                                                  Bytes(kOfdmText.begin(), kOfdmText.end()),
                                                  CodeRate::R1_4, out.ofdm_cw)
                               .serialize());
    return out;
}

struct Heard {
    std::string text;
    WaveformMode receiver_waveform;
    bool fading_measured;  // what the protocol is told about the frame's fading index
};

std::string payloadText(const Bytes& frame) {
    auto parsed = v2::DataFrame::deserialize(frame);
    if (!parsed) return {};
    return std::string(parsed->payload.begin(), parsed->payload.end());
}

// BRAVO connected on OFDM; the sender's MC-DPSK frame, then its OFDM frame, through feedAudio and
// processRxBuffer 20 ms at a time (the GUI's synchronous receive loop).
std::vector<Heard> receive(bool dual_listen, const SenderAudio& audio,
                           std::vector<WaveformMode>& reports, bool render_reads = false) {
    setenv("ULTRA_WAVEFORM_FOLLOW", dual_listen ? "2" : "0", 1);
    ModemEngine rx;  // no callsign, no label: it hears every station (the CLI's prx does the same)
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    rx.setSynchronousMode(true);
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    rx.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    std::vector<Heard> heard;
    rx.setRxWaveformObservedCallback([&](WaveformMode mode) {
        reports.push_back(mode);
        if (mode == WaveformMode::MC_DPSK) {
            rx.crossConnectedWaveform(mode, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
        } else {
            rx.crossConnectedWaveform(mode, Modulation::QPSK, CodeRate::R1_4, audio.ofdm_cw, 0, 0);
        }
    });
    rx.setRawDataCallback([&](const Bytes& data, bool) {
        heard.push_back({payloadText(data), rx.getWaveformMode(), rx.hasFadingIndex()});
    });
    constexpr size_t kChunk = 960;
    const std::vector<float> silence(kChunk, 0.0f);
    auto feed = [&](const float* p, size_t n) {
        rx.feedAudio(p, n);
        rx.processRxBuffer();
    };
    auto feedAll = [&](const std::vector<float>& a) {
        for (size_t i = 0; i < a.size(); i += kChunk) {
            feed(a.data() + i, std::min(kChunk, a.size() - i));
        }
    };
    auto feedSilence = [&](size_t samples) {
        for (size_t i = 0; i < samples; i += kChunk) feed(silence.data(), kChunk);
    };
    // The GUI's render loop reads the constellation on its own thread while the decoder runs and
    // the session crosses waveforms (a render-thread read of the waveform's demodulator crashed the
    // GUI, 2026-11-09 and 2026-11-11). Under the sanitizer build this is the race check.
    std::atomic<bool> stop{false};
    std::atomic<size_t> reads{0};
    std::thread render;
    if (render_reads) {
        render = std::thread([&] {
            while (!stop.load()) {
                const auto symbols = rx.getConstellationSymbols();
                (void)rx.getConstellationModulation();
                (void)symbols;
                reads.fetch_add(1);
            }
        });
    }
    feedSilence(24000);
    feedAll(audio.mcdpsk);
    feedSilence(48000);
    feedAll(audio.ofdm);
    feedSilence(3 * 48000);
    stop.store(true);
    if (render.joinable()) render.join();
    if (render_reads) {
        check(reads.load() > 0, "the render thread read the constellation throughout");
    }
    return heard;
}

}  // namespace

int main() {
    const SenderAudio audio = senderAudio();
    check(!audio.mcdpsk.empty() && !audio.ofdm.empty(), "the sender produced both bursts");

    std::vector<WaveformMode> reports;
    const auto heard = receive(/*dual_listen=*/true, audio, reports, /*render_reads=*/true);
    check(heard.size() == 2, "dual-listen: both frames delivered");
    if (heard.size() == 2) {
        check(heard[0].text == kMcText && heard[0].receiver_waveform == WaveformMode::MC_DPSK,
              "the MC-DPSK frame reached a receiver already on MC-DPSK");
        check(heard[1].text == kOfdmText && heard[1].receiver_waveform == WaveformMode::OFDM_CHIRP,
              "the OFDM frame reached a receiver back on OFDM");
    }
    if (heard.size() == 2) {
        // The spell's MC-DPSK decoder never measured a fading index: its 0.0 is "unmeasured",
        // and the protocol must not read it as AWGN; the OFDM frame's LTS measured one.
        check(!heard[0].fading_measured && heard[1].fading_measured,
              "fading is unmeasured on the MC-DPSK frame, measured on the OFDM frame");
    }
    check(reports.size() == 2 && reports[0] == WaveformMode::MC_DPSK &&
              reports[1] == WaveformMode::OFDM_CHIRP,
          "each waveform reported once, before its frame");

    std::vector<WaveformMode> no_reports;
    const auto single = receive(/*dual_listen=*/false, audio, no_reports);
    check(single.size() == 1 && single[0].text == kOfdmText && no_reports.empty(),
          "without dual-listen an OFDM receiver misses the MC-DPSK frame (the control)");

    std::printf("%s\n", failures == 0 ? "DualListenEngine: PASS" : "DualListenEngine: FAIL");
    return failures == 0 ? 0 : 1;
}
