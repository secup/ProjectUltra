// Engine concurrency: the GUI's main thread reconfigures a receiving engine while its decode thread
// runs. An MC-DPSK rate fallback calls setMCDPSKProfile / setDataMode / setFixedFrameCodewords from
// the protocol tick (App's data-mode callback); the render loop reads the constellation, the stats and
// carrier sense every frame. Two decode-thread crashes in the MC-DPSK chirp search coincided with that
// callback (BUG-CHIRP-SEARCH-SEGV), and the render-thread constellation read crashed the GUI
// (BUG-GUI-CONSTELLATION-RACE). Under ThreadSanitizer this is the race check; in a plain build it is
// a smoke test that must survive and still decode.

#include "gui/modem/modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "waveform/tone_burst_ack/tone_burst_payload.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;
using protocol::WaveformMode;

namespace {

// Under a sanitizer the DSP runs several times slower than real time: the decode thread cannot keep
// up with the audio and the decode counts below are not meaningful there (the races still are).
#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
constexpr bool kSanitized = true;
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer) || __has_feature(address_sanitizer)
constexpr bool kSanitized = true;
#else
constexpr bool kSanitized = false;
#endif
#else
constexpr bool kSanitized = false;
#endif

int failures = 0;
void check(bool ok, const char* what) {
    std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", what);
    if (!ok) ++failures;
}

// ALPHA in a wideband session that fell back to MC-DPSK (DBPSK, 8 carriers, 1024 samples/symbol,
// 3 codewords, compact frames): a run of DATA frames with gaps, as a fallback spell sends them.
std::vector<float> senderAudio(int frames) {
    ModemEngine tx;
    tx.setLogPrefix("ALPHA");
    tx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    tx.setMcDpskSessionAddresses(v2::hashCallsign("ALPHA"), v2::hashCallsign("BRAVO"));
    tx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    std::vector<float> audio(24000, 0.0f);
    for (int i = 0; i < frames; ++i) {
        const std::string text = "concurrency frame " + std::to_string(i);
        const auto frame = tx.transmit(
            v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(i + 1), text).serialize());
        audio.insert(audio.end(), frame.begin(), frame.end());
        audio.insert(audio.end(), 24000, 0.0f);
    }
    return audio;
}

// The same session on OFDM (QPSK R1/2): single fixed DATA frames with gaps; gap_starts marks where
// each frame's air ends (half-duplex: this station may key only there).
std::vector<float> ofdmSenderAudio(int frames, int& cw_out, std::vector<size_t>& gap_starts) {
    ModemEngine tx;
    tx.setLogPrefix("ALPHA");
    tx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_2);
    cw_out = tx.getFixedFrameCodewords();
    std::vector<float> audio(24000, 0.0f);
    for (int i = 0; i < frames; ++i) {
        const std::string text = "ofdm concurrency frame " + std::to_string(i);
        const auto frame = tx.transmit(v2::makeFixedDataFrame("ALPHA", "BRAVO",
                                                              static_cast<uint16_t>(i + 1),
                                                              Bytes(text.begin(), text.end()),
                                                              CodeRate::R1_2, cw_out)
                                           .serialize());
        audio.insert(audio.end(), frame.begin(), frame.end());
        gap_starts.push_back(audio.size());
        audio.insert(audio.end(), 48000, 0.0f);
    }
    return audio;
}

}  // namespace

int main() {
    const std::vector<float> audio = senderAudio(4);
    check(audio.size() > 48000, "the sender produced its frames");

    ModemEngine rx;  // asynchronous: its own decode thread, as in the GUI and the TNC
    rx.joinConnectedSession(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4);
    rx.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    rx.crossConnectedWaveform(WaveformMode::MC_DPSK, Modulation::DBPSK, CodeRate::R1_4, 3, 8, 1024);
    std::atomic<int> delivered{0};
    std::atomic<int> acks{0};
    // The protocol answers on the decode thread: a delivered frame's ACK is encoded inside the
    // delivery callback (Connection -> ARQ -> the frontend's tone-ACK transmit), while the main
    // thread's fallback rebuilds the same engine's encoder (setMCDPSKProfile).
    rx.setRawDataCallback([&](const Bytes&, bool) {
        delivered.fetch_add(1);
        ultra::waveform::tone_burst_ack::ToneBurstAckPayload ack;
        ack.group_seq = static_cast<uint8_t>(delivered.load() & 0x3F);
        const auto samples = rx.transmitToneBurstAck(ack);
        if (!samples.empty()) acks.fetch_add(1);
    });

    // The render loop: reads the decoder's view on its own thread at ~60 Hz.
    std::atomic<bool> stop{false};
    std::atomic<size_t> renders{0};
    std::thread render([&] {
        while (!stop.load()) {
            (void)rx.getConstellationSymbols();
            (void)rx.getConstellationModulation();
            (void)rx.getStats();
            (void)rx.channelBusyForTx();
            (void)rx.burstAirSamplesRemaining();
            renders.fetch_add(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    // The audio and protocol thread: 20 ms chunks at ~10x real time; every 100 ms of audio the
    // fallback's profile change (DBPSK <-> DQPSK), as App's data-mode callback makes it.
    constexpr size_t kChunk = 960;
    int profile_changes = 0;
    for (size_t i = 0, k = 0; i < audio.size(); i += kChunk, ++k) {
        rx.feedAudio(audio.data() + i, std::min(kChunk, audio.size() - i));
        if (k % 5 == 4) {
            const bool dbpsk = (profile_changes++ % 2) == 0;
            rx.setMCDPSKProfile(8, 1024, dbpsk ? 1 : 2);
            rx.setDataMode(dbpsk ? Modulation::DBPSK : Modulation::DQPSK, CodeRate::R1_4);
            rx.setFixedFrameCodewords(3);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // Let the decode thread drain the tail.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    // OFDM phase: the session back on OFDM; the decode thread deinterleaves and decodes while the
    // main thread transmits OFDM data (the encoder and the frame interleaver on its side) and
    // switches the data rung, as a sending station's protocol tick does.
    int ofdm_cw = 0;
    std::vector<size_t> gap_starts;
    const std::vector<float> ofdm_audio = ofdmSenderAudio(4, ofdm_cw, gap_starts);
    rx.crossConnectedWaveform(WaveformMode::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_2, ofdm_cw,
                              0, 0);
    const int delivered_before_ofdm = delivered.load();
    int ofdm_tx = 0;
    size_t next_gap = 0;
    for (size_t i = 0; i < ofdm_audio.size(); i += kChunk) {
        rx.feedAudio(ofdm_audio.data() + i, std::min(kChunk, ofdm_audio.size() - i));
        // In each gap, after the decode thread has had the frame: key up (encoder and frame
        // interleaver on this thread) while the decode thread is still busy with that frame.
        if (next_gap < gap_starts.size() && i >= gap_starts[next_gap] + 9600) {
            ++next_gap;
            // Half-duplex: key only once this frame has been heard (or given up on), or this
            // station's own transmission clears it from the receive buffer. Bounded wait.
            const int expect = delivered_before_ofdm + static_cast<int>(next_gap);
            for (int w = 0; w < 400 && delivered.load() < expect; ++w) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            const std::string text = "reverse " + std::to_string(ofdm_tx);
            (void)rx.transmit(v2::makeFixedDataFrame("BRAVO", "ALPHA",
                                                     static_cast<uint16_t>(++ofdm_tx),
                                                     Bytes(text.begin(), text.end()),
                                                     CodeRate::R1_2, ofdm_cw)
                                  .serialize());
            rx.setDataMode(Modulation::QPSK, (ofdm_tx % 2) ? CodeRate::R1_2 : CodeRate::R2_3);
            rx.setDataMode(Modulation::QPSK, CodeRate::R1_2);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    check(ofdm_tx == static_cast<int>(gap_starts.size()),
          "the main thread transmitted OFDM data in every gap while the engine decoded");
    const int ofdm_delivered = delivered.load() - delivered_before_ofdm;
    std::printf("OFDM phase: delivered %d frame(s)\n", ofdm_delivered);
    if (!kSanitized) {
        check(ofdm_delivered >= 1,
              "the OFDM phase decoded while this station transmitted in the gaps");
    }

    stop.store(true);
    render.join();
    check(profile_changes > 10, "the main thread reconfigured the decoding engine many times");
    check(renders.load() > 100, "the render thread read the decoder throughout");
    std::printf("delivered %d frame(s), encoded %d ACK(s) across %d profile changes\n",
                delivered.load(), acks.load(), profile_changes);
    std::printf("%s\n", failures == 0 ? "EngineConcurrency: PASS" : "EngineConcurrency: FAIL");
    return failures == 0 ? 0 : 1;
}
