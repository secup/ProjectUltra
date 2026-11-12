// Dual-listen at the engine (docs/DUAL_LISTEN_DESIGN_2026_11_11.md): a receiver connected on one
// wideband waveform delivers what the sender sent on the other. The sender crosses the way a
// session's crossing callback does (ModemEngine::crossConnectedWaveform); the test stands in for the
// receiver's protocol: on the engine's waveform report it crosses too, as the binding's
// noteRxWaveform -> Connection::applyWaveformCrossing -> crossConnectedWaveform chain does. The
// report must come before the frame, and the frame must reach a receiver already on its waveform.

#include "gui/modem/modem_engine.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"

#include <atomic>
#include <cstdio>
#include <thread>
#include <cmath>
#include <random>
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
// handshake_confirmed=false: a responder connected (it sent CONNECT_ACK) whose handshake is not yet
// confirmed by a frame from the initiator.
std::vector<Heard> receive(bool dual_listen, const SenderAudio& audio,
                           std::vector<WaveformMode>& reports, bool render_reads = false,
                           bool handshake_confirmed = true) {
    setenv("ULTRA_WAVEFORM_FOLLOW", dual_listen ? "2" : "0", 1);
    ModemEngine rx;  // no callsign, no label: it hears every station (the CLI's prx does the same)
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    rx.setSynchronousMode(true);
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    if (!handshake_confirmed) rx.setHandshakeComplete(false);
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

// ULTRA_MCDPSK_ACK_REPAIR, decoder side: a compact MC-DPSK burst whose first frame is lost (its
// data buried, its chirp and training intact) goes on at its stride, so frames 2 and 3 - which have
// no chirp of their own - still decode. Without the knob they were lost with it.
struct LossRun {
    std::vector<std::string> heard;
    std::vector<bool> turn_losses;  // turn_over flag of each turn-loss report
};

LossRun receiveBurstWithLostFrame(bool repair, int lost);

std::vector<std::string> receiveBurstWithLostFirstFrame(bool repair) {
    return receiveBurstWithLostFrame(repair, 1).heard;
}

LossRun receiveBurstWithLostFrame(bool repair, int lost) {
    ModemEngine tx;
    tx.setLogPrefix("ALPHA");
    tx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    tx.setMcDpskSessionAddresses(v2::hashCallsign("ALPHA"), v2::hashCallsign("BRAVO"));
    tx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    std::vector<Bytes> frames;
    for (int i = 0; i < 3; ++i) {
        auto f = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(i + 1),
                                         "burst frame " + std::to_string(i + 1));
        f.flags |= v2::Flags::MORE_FRAG;
        frames.push_back(f.serialize());
    }
    std::vector<float> burst = tx.transmitBurst(frames);
    // Frame 1 = chirp + training + data; bury its data under noise 10x the signal.
    MultiCarrierDPSKConfig cfg;
    cfg.num_carriers = 8;
    cfg.samples_per_symbol = 1024;
    MCDPSKWaveform geo(cfg);
    geo.configure(Modulation::DBPSK, CodeRate::R1_4);
    // Data frames have a fixed length; the preamble (dual chirp + training) is whatever the burst's
    // audible span holds beyond its three data frames.
    const size_t data_len = static_cast<size_t>(geo.getDataOnlySamplesForCWCount(3));
    size_t start = 0;
    while (start < burst.size() && std::abs(burst[start]) < 1e-6f) ++start;
    size_t end = burst.size();
    while (end > start && std::abs(burst[end - 1]) < 1e-6f) --end;
    const size_t first_len = (end - start) - 2 * data_len;
    double sum = 0.0;
    for (size_t i = start; i < std::min(burst.size(), start + first_len); ++i) sum += burst[i] * burst[i];
    const double rms = std::sqrt(sum / static_cast<double>(std::max<size_t>(1, first_len)));
    std::mt19937 rng(5);
    std::normal_distribution<double> nd(0.0, 10.0 * rms);
    // Frame k's data: frame 1's after the preamble, frame k > 1's at the stride after it.
    const size_t lost_begin = start + first_len - data_len + static_cast<size_t>(lost - 1) * data_len;
    for (size_t i = lost_begin; i < std::min(burst.size(), lost_begin + data_len); ++i) {
        burst[i] += static_cast<float>(nd(rng));
    }

    setenv("ULTRA_MCDPSK_ACK_REPAIR", repair ? "1" : "0", 1);
    ModemEngine rx;
    rx.setSynchronousMode(true);
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    rx.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    rx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    LossRun run;
    rx.setRawDataCallback([&](const Bytes& data, bool) { run.heard.push_back(payloadText(data)); });
    rx.setMcDpskTurnLossCallback([&](bool turn_over) { run.turn_losses.push_back(turn_over); });
    std::vector<float> air(24000, 0.0f);
    air.insert(air.end(), burst.begin(), burst.end());
    air.insert(air.end(), 4 * 48000, 0.0f);
    // A little background noise: the burst-noise reference and the tail test need a floor.
    std::normal_distribution<double> bg(0.0, 1e-3);
    for (auto& v : air) v += static_cast<float>(bg(rng));
    constexpr size_t kChunk = 960;
    for (size_t i = 0; i < air.size(); i += kChunk) {
        rx.feedAudio(air.data() + i, std::min(kChunk, air.size() - i));
        rx.processRxBuffer();
    }
    unsetenv("ULTRA_MCDPSK_ACK_REPAIR");
    return run;
}

// BUG-MCDPSK-FALSE-TAIL: a static crash (a QRN impulse) inside the silent inter-chirp gap - the
// burst's only noise reference - makes that reference read far above the noise floor. The in-band
// burst-tail test then read every continuation slot as noise and ended a 3-frame burst after frame
// 1 at any SNR (at -5 dB the reference's own 7.6 % spread did the same in one burst out of five).
// Frames 1 and 2 say a frame follows (MORE_FRAG without PHYSICAL_BURST_END), so 2 and 3 must decode.
std::vector<std::string> receiveBurstWithGapCrash() {
    ModemEngine tx;
    tx.setLogPrefix("ALPHA");
    tx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    tx.setMcDpskSessionAddresses(v2::hashCallsign("ALPHA"), v2::hashCallsign("BRAVO"));
    tx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    std::vector<Bytes> frames;
    for (int i = 0; i < 3; ++i) {
        auto f = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(i + 1),
                                         "burst frame " + std::to_string(i + 1));
        f.flags |= v2::Flags::MORE_FRAG;
        frames.push_back(f.serialize());
    }
    std::vector<float> burst = tx.transmitBurst(frames);
    size_t start = 0;
    while (start < burst.size() && std::abs(burst[start]) < 1e-6f) ++start;
    double sum = 0.0;
    size_t voiced = 0;
    for (size_t i = start; i < burst.size(); ++i) {
        if (std::abs(burst[i]) >= 1e-6f) {
            sum += burst[i] * burst[i];
            ++voiced;
        }
    }
    const double rms = std::sqrt(sum / static_cast<double>(std::max<size_t>(1, voiced)));
    // The inter-chirp gap is the burst's first silent run of 50 ms or more (it is 100 ms).
    size_t gap_begin = 0, gap_end = 0, run = 0;
    for (size_t i = start; i < burst.size(); ++i) {
        if (std::abs(burst[i]) < 1e-6f) {
            if (++run == 2400) gap_begin = i + 1 - run;
        } else {
            if (run >= 2400) {
                gap_end = i;
                break;
            }
            run = 0;
        }
    }
    check(gap_end > gap_begin + 2400, "the sender's burst has its silent inter-chirp gap");
    // White noise 5x the signal's RMS: ~12 % of it falls in the 50-2950 Hz test band, so the
    // reference reads ~3x the signal's in-band power.
    std::mt19937 rng(11);
    std::normal_distribution<double> crash(0.0, 5.0 * rms);
    for (size_t i = gap_begin; i < gap_end; ++i) burst[i] += static_cast<float>(crash(rng));

    ModemEngine rx;
    rx.setSynchronousMode(true);
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    rx.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    rx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    std::vector<std::string> heard;
    rx.setRawDataCallback([&](const Bytes& data, bool) { heard.push_back(payloadText(data)); });
    std::vector<float> air(24000, 0.0f);
    air.insert(air.end(), burst.begin(), burst.end());
    air.insert(air.end(), 4 * 48000, 0.0f);
    std::normal_distribution<double> bg(0.0, 1e-3);
    for (auto& v : air) v += static_cast<float>(bg(rng));
    constexpr size_t kChunk = 960;
    for (size_t i = 0; i < air.size(); i += kChunk) {
        rx.feedAudio(air.data() + i, std::min(kChunk, air.size() - i));
        rx.processRxBuffer();
    }
    return heard;
}

// BUG-MCDPSK-CONNECT-AFTER-DQPSK-PICK: a responder connected on a DQPSK session (4 codewords) has
// lost its CONNECT_ACK; the initiator repeats its CONNECT, which rides the handshake's DBPSK control
// profile (#72). The responder demodulated the repeat at its session constellation, never decoded
// it and never replayed the CONNECT_ACK (GUI ITU Good 0 dB seed 47: "Connect failed after 10
// attempts"). Returns whether the connected responder delivered the CONNECT.
bool connectedDqpskResponderHearsConnect() {
    ModemEngine initiator;  // not connected: the handshake profile
    initiator.setLogPrefix("ALPHA");
    const auto connect = v2::ConnectFrame::makeConnect("ALPHA", "BRAVO", 0xF0, 0xFF);
    std::vector<float> burst = initiator.transmit(connect.serialize());

    ModemEngine rx;
    rx.setLogPrefix("BRAVO");
    rx.setSynchronousMode(true);
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    rx.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    rx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DQPSK, CodeRate::R1_4, 4, 8, 1024);
    bool heard = false;
    rx.setRawDataCallback([&](const Bytes& data, bool) {
        if (data.size() >= 3 && static_cast<v2::FrameType>(data[2]) == v2::FrameType::CONNECT) {
            heard = true;
        }
    });
    std::vector<float> air(24000, 0.0f);
    air.insert(air.end(), burst.begin(), burst.end());
    air.insert(air.end(), 4 * 48000, 0.0f);
    std::mt19937 rng(23);
    std::normal_distribution<double> bg(0.0, 1e-3);
    for (auto& v : air) v += static_cast<float>(bg(rng));
    constexpr size_t kChunk = 960;
    for (size_t i = 0; i < air.size(); i += kChunk) {
        rx.feedAudio(air.data() + i, std::min(kChunk, air.size() - i));
        rx.processRxBuffer();
    }
    return heard;
}

// BUG-LISTENER-KEEPS-SESSION-PROFILE: a session that connected on MC-DPSK Robust (DQPSK, 4 codewords)
// follows the sender up to OFDM - the MC-DPSK decoder flips to listener - and the sender's OFDM
// attempt fails: it falls back to the down-crossing target, ROBUST_MID (DBPSK, 3 codewords). The
// listener kept the session's Robust profile, sliced every fallback burst as a 4-codeword DQPSK frame
// and never decoded one (GUI ITU Good 5 dB seed 48: 330 s of fallback bursts found at 20 dB sync and
// lost, then a terminal ARQ failure). Returns the payloads heard, in order.
std::vector<std::string> robustSessionHearsFallbackAfterUpCrossing() {
    const std::string up_text = "the up-crossing's OFDM frame";
    const std::string down_text = "the fallback's MC-DPSK frame";
    ModemEngine tx;
    tx.setLogPrefix("ALPHA");
    tx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    tx.setMcDpskSessionAddresses(v2::hashCallsign("ALPHA"), v2::hashCallsign("BRAVO"));
    const int cw = protocol::connection_policy::recommendCWCount(Modulation::QPSK, CodeRate::R1_4,
                                                                 WaveformMode::OFDM_CHIRP);
    tx.crossConnectedWaveform(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4, cw, 0, 0);
    const int ofdm_cw = tx.getFixedFrameCodewords();
    const auto up = tx.transmit(v2::makeFixedDataFrame("ALPHA", "BRAVO", 1,
                                                       Bytes(up_text.begin(), up_text.end()),
                                                       CodeRate::R1_4, ofdm_cw)
                                    .serialize());
    tx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    const auto down = tx.transmit(v2::DataFrame::makeData("ALPHA", "BRAVO", 2, down_text).serialize());

    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    ModemEngine rx;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    rx.setLogPrefix("BRAVO");
    rx.setSynchronousMode(true);
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    rx.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    // The connect pick: MC-DPSK Robust.
    rx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DQPSK, CodeRate::R1_4, 4, 8, 1024);
    rx.setRxWaveformObservedCallback([&](WaveformMode mode) {
        if (mode == WaveformMode::MC_DPSK) {
            rx.crossConnectedWaveform(mode, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
        } else {
            rx.crossConnectedWaveform(mode, Modulation::QPSK, CodeRate::R1_4, ofdm_cw, 0, 0);
        }
    });
    std::vector<std::string> heard;
    rx.setRawDataCallback([&](const Bytes& data, bool) { heard.push_back(payloadText(data)); });
    std::vector<float> air(24000, 0.0f);
    air.insert(air.end(), up.begin(), up.end());
    air.insert(air.end(), 48000, 0.0f);
    air.insert(air.end(), down.begin(), down.end());
    air.insert(air.end(), 4 * 48000, 0.0f);
    std::mt19937 rng(31);
    std::normal_distribution<double> bg(0.0, 1e-3);
    for (auto& v : air) v += static_cast<float>(bg(rng));
    constexpr size_t kChunk = 960;
    for (size_t i = 0; i < air.size(); i += kChunk) {
        rx.feedAudio(air.data() + i, std::min(kChunk, air.size() - i));
        rx.processRxBuffer();
    }
    const bool ok = heard.size() == 2 && heard[0] == up_text && heard[1] == down_text;
    std::printf("Robust session, OFDM then the ROBUST_MID fallback: %zu frame(s) heard%s\n",
                heard.size(), ok ? "" : " (expected both)");
    return heard;
}

void checkGapCrashBurst(const char* path) {
    const auto heard = receiveBurstWithGapCrash();
    std::printf("static crash in the inter-chirp gap (%s frames): %zu of 3 frames\n", path,
                heard.size());
    check(heard.size() == 3 && heard[0] == "burst frame 1" && heard[1] == "burst frame 2" &&
              heard[2] == "burst frame 3",
          "a static crash in the gap does not end a burst whose frames say more follow");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--gap-crash-only") {
        // The v2 (non-compact) continuation, run with ULTRA_MCDPSK_COMPACT_FRAME=0: the setting
        // is read once per process.
        checkGapCrashBurst(protocol::connection_policy::mcDpskCompactFrameEnabled() ? "compact"
                                                                                     : "v2");
        std::printf("%s\n", failures == 0 ? "DualListenEngine gap crash: PASS"
                                          : "DualListenEngine gap crash: FAIL");
        return failures == 0 ? 0 : 1;
    }
    checkGapCrashBurst("compact");
    {
        const auto heard = robustSessionHearsFallbackAfterUpCrossing();
        check(heard.size() == 2 && heard[1] == "the fallback's MC-DPSK frame",
              "a session that started on MC-DPSK Robust hears the sender's ROBUST_MID fallback "
              "after following it up to OFDM");
    }
    {
        const bool heard = connectedDqpskResponderHearsConnect();
        std::printf("connected DQPSK responder, repeated DBPSK CONNECT: %s\n",
                    heard ? "decoded" : "missed");
        check(heard, "a responder on a DQPSK session decodes the initiator's repeated CONNECT");
    }
    {
        const auto with_repair = receiveBurstWithLostFirstFrame(true);
        const auto without = receiveBurstWithLostFirstFrame(false);
        std::printf("lost first frame: %zu frame(s) with repair, %zu without\n",
                    with_repair.size(), without.size());
        check(with_repair.size() == 2 && with_repair[0] == "burst frame 2" &&
                  with_repair[1] == "burst frame 3",
              "repair: frames 2 and 3 of a burst whose first frame is lost still decode");
        check(without.empty(), "without repair they are lost with the first frame (the control)");
        // The burst's LAST frame lost: frames 1-2 decode, and the decoder reports one turn loss
        // at the burst's end (its tail or full window), flagged as the end of the turn.
        const auto last = receiveBurstWithLostFrame(true, 3);
        check(last.heard.size() == 2 && last.heard[0] == "burst frame 1" &&
                  last.heard[1] == "burst frame 2",
              "a burst whose last frame is lost delivers frames 1 and 2");
        check(last.turn_losses.size() == 1 && last.turn_losses[0],
              "and reports one turn loss at the end of the turn");
    }
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

    // BUG-LISTENER-WAITS-FOR-CONFIRMATION: a responder whose handshake is not confirmed yet (the
    // session started on OFDM and no OFDM frame got through) still hears the initiator's MC-DPSK
    // frame - the follow crossing that would confirm it. The listener used to wait for the
    // confirmation, so the crossing was never heard (half-open timeout after 240 s).
    std::vector<WaveformMode> pending_reports;
    const auto pending = receive(/*dual_listen=*/true, audio, pending_reports,
                                 /*render_reads=*/false, /*handshake_confirmed=*/false);
    std::printf("unconfirmed responder: %zu frame(s) heard\n", pending.size());
    check(!pending.empty() && pending[0].text == kMcText,
          "a responder awaiting handshake confirmation hears the initiator's MC-DPSK frame");

    std::vector<WaveformMode> no_reports;
    const auto single = receive(/*dual_listen=*/false, audio, no_reports);
    check(single.size() == 1 && single[0].text == kOfdmText && no_reports.empty(),
          "without dual-listen an OFDM receiver misses the MC-DPSK frame (the control)");

    std::printf("%s\n", failures == 0 ? "DualListenEngine: PASS" : "DualListenEngine: FAIL");
    return failures == 0 ? 0 : 1;
}
