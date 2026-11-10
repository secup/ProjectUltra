/**
 * ProjectUltra CLI - High-Speed HF Sound Modem
 *
 * ptx and prx run ModemEngine, the modem object the GUI and the TNC run. ptx builds each frame
 * with ModemEngine::transmit() in the state a session sends it from. prx plays the whole
 * recording into one ModemEngine receiver per station state that could have sent a frame of it
 * (idle MC-DPSK, connected MC-DPSK, connected OFDM at each rung), through feedAudio() and
 * processRxBuffer() like the GUI's synchronous receive loop, each set up with
 * joinConnectedSession() like the GUI's --monitor mode.
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "gui/modem/modem_engine.hpp"
#include "io/wav_io.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"
#include "sim/cli_enums.hpp"
#include "sync/acquisition_mode.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

// Signal handling for clean shutdown
static std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

void printUsage(const char* prog) {
    std::cerr << "ProjectUltra - High-Speed HF Sound Modem\n\n";
    std::cerr << "Usage: " << prog << " [options] <command>\n\n";
    std::cerr << "Commands:\n";
    std::cerr << "  ptx [msg]       Protocol TX - send v2 frame:\n";
    std::cerr << "                    ptx ping         -> PING probe (~1 sec, MC-DPSK)\n";
    std::cerr << "                    ptx connect      -> CONNECT (MC-DPSK)\n";
    std::cerr << "                    ptx disconnect   -> DISCONNECT (MC-DPSK)\n";
    std::cerr << "                    ptx \"Hello\"      -> DATA (text message, -w/-m/-r)\n";
    std::cerr << "  prx [file]      Protocol RX - decode v2 frames (from file or stdin)\n";
    std::cerr << "  info            Show modem capabilities\n";
    std::cerr << "\nOptions:\n";
    std::cerr << "  -s <call>       Source callsign (default: N0CALL)\n";
    std::cerr << "  -d <call>       Destination callsign (default: CQ)\n";
    std::cerr << "  -o <file>       Output to file instead of stdout; *.wav writes a 48 kHz\n";
    std::cerr << "                  16-bit WAV, anything else raw 32-bit float 48 kHz\n";
    std::cerr << "  -w <waveform>   ptx DATA: ofdm, narrow, mcdpsk (default: ofdm); with ping,\n";
    std::cerr << "                  connect, disconnect: narrow = narrowband control waveform\n";
    std::cerr << "                  prx: auto, ofdm, narrow, mcdpsk (default: auto = MC-DPSK\n";
    std::cerr << "                  handshake and data, wideband OFDM at every rung, and\n";
    std::cerr << "                  narrowband QPSK R1/4 and R1/2). One ModemEngine receiver\n";
    std::cerr << "                  per state hears the whole recording, like the GUI\n";
    std::cerr << "  -m <mod>        OFDM data modulation: qpsk, 8psk, 16qam, 32qam\n";
    std::cerr << "                  (ptx default: qpsk; prx default: every rung)\n";
    std::cerr << "  -r <rate>       Code rate: r1_4, r1_2, r2_3, r3_4\n";
    std::cerr << "                  (ptx default: r1_4; prx default: every rung)\n";
    std::cerr << "  --log-level <error|warn|info|debug|trace>\n";
    std::cerr << "                  Console verbosity (default: info for ptx, warn for prx)\n";
    std::cerr << "  --log-category <list>\n";
    std::cerr << "                  Comma list: operator,audio,tnc,modem,demod,sync,ldpc,channel,all\n";
    std::cerr << "  --log-file <path>\n";
    std::cerr << "                  Write logs to file instead of stderr\n";
    std::cerr << "\nprx reads raw 32-bit float at 48 kHz, or a WAV file at any sample rate\n";
    std::cerr << "(8/16/24/32-bit PCM or float; the first channel), resampled to 48 kHz.\n";
    std::cerr << "\nExamples:\n";
    std::cerr << "  # Send PING probe and play audio\n";
    std::cerr << "  " << prog << " ptx ping -s MYCALL | aplay -f FLOAT_LE -r 48000\n\n";
    std::cerr << "  # 16QAM R2/3 data frame to a WAV file\n";
    std::cerr << "  " << prog << " ptx \"Hello\" -s MYCALL -m 16qam -r r2_3 -o hello.wav\n\n";
    std::cerr << "  # Decode a recording (any frame, any rung)\n";
    std::cerr << "  " << prog << " prx recording.wav\n\n";
    std::cerr << "  # Loopback test\n";
    std::cerr << "  " << prog << " ptx ping | " << prog << " prx\n";
    std::cerr << "\n";
}

void printInfo() {
    std::cout << "=== ProjectUltra HF Modem ===\n\n";
    std::cout << "Signal Parameters:\n";
    std::cout << "  Sample rate:    48000 Hz\n";
    std::cout << "  Center freq:    1500 Hz\n";
    std::cout << "  Bandwidth:      ~2.8 kHz (wideband OFDM), ~0.5 kHz (narrowband OFDM)\n";
    std::cout << "  OFDM carriers:  59 (FFT 1024), narrowband 21 (FFT 2048)\n";
    std::cout << "  LDPC codeword:  648 bits\n\n";

    std::cout << "Waveforms:\n";
    std::cout << "  MC-DPSK   Multi-carrier DPSK: PING, connection setup, low-SNR data\n";
    std::cout << "  OFDM      Coherent OFDM data: QPSK, 8PSK, 16QAM, 32QAM\n\n";

    std::cout << "Code Rates:\n";
    std::cout << "  R1/4    20 info bytes per codeword, most robust\n";
    std::cout << "  R1/2    40 info bytes\n";
    std::cout << "  R2/3    54 info bytes\n";
    std::cout << "  R3/4    60 info bytes\n";
}

// Waveform type
enum class WaveformType { AUTO, OFDM_CHIRP, OFDM_NARROW, DPSK };

std::optional<WaveformType> parseWaveform(const char* s) {
    auto mode = tools::cli::parseWaveformMode(s, tools::cli::BareOFDMMode::Chirp,
                                              tools::cli::AllowAuto::Yes);
    if (!mode) return std::nullopt;
    switch (*mode) {
        case protocol::WaveformMode::AUTO: return WaveformType::AUTO;
        case protocol::WaveformMode::MC_DPSK: return WaveformType::DPSK;
        case protocol::WaveformMode::OFDM_CHIRP: return WaveformType::OFDM_CHIRP;
        case protocol::WaveformMode::OFDM_NARROW: return WaveformType::OFDM_NARROW;
        default: return std::nullopt;
    }
}

protocol::WaveformMode toWaveformMode(WaveformType w) {
    switch (w) {
        case WaveformType::DPSK: return protocol::WaveformMode::MC_DPSK;
        case WaveformType::OFDM_NARROW: return protocol::WaveformMode::OFDM_NARROW;
        case WaveformType::OFDM_CHIRP:
        case WaveformType::AUTO:
        default: return protocol::WaveformMode::OFDM_CHIRP;
    }
}

const char* waveformLabel(WaveformType w) {
    switch (w) {
        case WaveformType::DPSK: return "MC-DPSK";
        case WaveformType::OFDM_NARROW: return "OFDM narrow";
        default: return "OFDM";
    }
}

// The OFDM data constellations a coherent rung can carry.
bool isOfdmDataModulation(Modulation mod) {
    return mod == Modulation::QPSK || mod == Modulation::QAM8 || mod == Modulation::QAM16 ||
           mod == Modulation::QAM32;
}

bool hasWavExtension(const std::string& path) {
    if (path.size() < 4) return false;
    std::string ext = path.substr(path.size() - 4);
    for (char& c : ext) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext == ".wav";
}

// ============================================================================
// Protocol TX - Uses ModemEngine for clean audio generation
// ============================================================================
int runProtocolTx(const char* message, const char* output_file,
                  const std::string& src_call, const std::string& dst_call,
                  WaveformType waveform, std::optional<CodeRate> rate_opt,
                  std::optional<Modulation> mod_opt) {
    if (waveform == WaveformType::AUTO) waveform = WaveformType::OFDM_CHIRP;
    const CodeRate rate = rate_opt.value_or(CodeRate::R1_4);

    std::cerr << "Protocol TX: " << src_call << " -> " << dst_call << "\n";

    ModemEngine modem;
    modem.setLogPrefix("TX");

    // Data modulation: OFDM is coherent-only; MC-DPSK data carries its preset's modulation.
    Modulation mod = Modulation::QPSK;
    if (waveform == WaveformType::DPSK) {
        mod = mcDpskModulationForConfig(modem.getMCDPSKConfig());
        if (mod_opt && *mod_opt != mod) {
            std::cerr << "Error: MC-DPSK data uses the connection preset's "
                      << modulationToString(mod) << "; -m applies to OFDM data\n";
            return 1;
        }
    } else {
        mod = mod_opt.value_or(Modulation::QPSK);
        if (!isOfdmDataModulation(mod)) {
            std::cerr << "Error: OFDM data modulation must be qpsk, 8psk, 16qam or 32qam\n";
            return 1;
        }
    }

    // A narrowband station sends its PING and control frames on the narrowband control
    // waveform (500 Hz chirp and MC-DPSK), as the GUI does when OFDM_NARROW is forced.
    if (waveform == WaveformType::OFDM_NARROW) modem.setNarrowbandControl(true);

    std::vector<float> samples;
    std::string frame_type;

    // Determine frame type and generate audio
    if (!message || strlen(message) == 0 || strcmp(message, "ping") == 0) {
        // PING probe - raw DPSK, no LDPC
        frame_type = "PING";
        samples = modem.transmitPing();

    } else if (strcmp(message, "connect") == 0) {
        // CONNECT frame (MC-DPSK, as the connection sends it)
        frame_type = "CONNECT";
        auto frame = v2::ConnectFrame::makeConnect(src_call, dst_call, 0xFF, 0x00);
        samples = modem.transmit(frame.serialize());

    } else if (strcmp(message, "disconnect") == 0) {
        // DISCONNECT control frame (hardened 1-CW profile)
        frame_type = "DISCONNECT";
        auto frame = v2::ControlFrame::makeDisconnect(src_call, dst_call);
        samples = modem.transmit(frame.serialize());

    } else {
        // DATA frame with a text message, sent from the state a session is in after
        // MODE_CHANGE to this waveform and rung. OFDM data frames are fixed-size frames of the
        // session's codeword count, built the way the connection builds them; MC-DPSK data
        // frames are as long as their payload.
        frame_type = "DATA";
        const auto mode = toWaveformMode(waveform);
        modem.joinConnectedSession(mode, mod, rate);
        const std::string text(message);
        if (protocol::isOFDMMode(mode)) {
            const int cw = modem.getFixedFrameCodewords();
            const size_t capacity = v2::getFixedFramePayloadCapacity(rate, cw);
            if (text.size() > capacity) {
                std::cerr << "Error: " << text.size() << " bytes do not fit one " << cw
                          << "-codeword " << modulationToString(mod) << " "
                          << codeRateToString(rate) << " frame (" << capacity << " bytes)\n";
                return 1;
            }
            auto frame = v2::makeFixedDataFrame(src_call, dst_call, 1,
                                                Bytes(text.begin(), text.end()), rate, cw);
            samples = modem.transmit(frame.serialize());
        } else {
            auto frame = v2::DataFrame::makeData(src_call, dst_call, 1, text);
            samples = modem.transmit(frame.serialize());
        }
    }

    std::cerr << "  Frame: " << frame_type;
    if (frame_type == "DATA") {
        std::cerr << " (" << waveformLabel(waveform) << " " << modulationToString(mod) << " "
                  << codeRateToString(rate) << ")";
    } else {
        std::cerr << (waveform == WaveformType::OFDM_NARROW ? " (narrowband MC-DPSK)" : " (MC-DPSK)");
    }
    std::cerr << "\n";
    std::cerr << "  Samples: " << samples.size() << " ("
              << (samples.size() / 48000.0) << " sec)\n";

    // Write output
    if (output_file) {
        const std::string path(output_file);
        if (hasWavExtension(path)) {
            if (!tools::io::writeWavPCM16Mono(path, samples, 48000)) {
                std::cerr << "Error: Cannot write " << path << "\n";
                return 1;
            }
            std::cerr << "  Written to: " << path << " (48 kHz 16-bit WAV)\n";
        } else {
            std::ofstream out(path, std::ios::binary);
            if (!out) {
                std::cerr << "Error: Cannot open " << path << "\n";
                return 1;
            }
            out.write(reinterpret_cast<const char*>(samples.data()),
                      static_cast<std::streamsize>(samples.size() * sizeof(float)));
            std::cerr << "  Written to: " << path << " (raw float32, 48 kHz)\n";
        }
    } else {
        std::cout.write(reinterpret_cast<const char*>(samples.data()),
                        static_cast<std::streamsize>(samples.size() * sizeof(float)));
    }

    return 0;
}

// ============================================================================
// Protocol RX - ModemEngine receivers over the recording
// ============================================================================
// One station state that could have sent a frame of the recording.
struct ReceiverState {
    std::string label;                  // "MC-DPSK", "OFDM 16QAM R2/3", ...
    std::optional<protocol::WaveformMode> session;  // empty: idle (PING / CONNECT / control)
    Modulation mod = Modulation::QPSK;
    CodeRate rate = CodeRate::R1_4;
};

struct Heard {
    double time_s = 0.0;         // when the receiver delivered it (frame end + decode latency)
    std::string receiver;
    bool ping = false;
    Bytes data;
    float snr_db = 0.0f;
    float cfo_hz = 0.0f;
    bool has_evm = false;
    float evm_db = 0.0f;
};

// Feed `audio` to a fresh ModemEngine in `state` the way the GUI feeds its own receiver in
// synchronous mode: feedAudio() then processRxBuffer(), 20 ms at a time. Frames are what the
// engine delivers (its raw-data callback, the protocol's input); the observers only add the
// measurements of each one.
std::vector<Heard> listen(const ReceiverState& state, const std::vector<float>& audio) {
    // No callsign and no log label: ModemEngine delivers only frames addressed to its callsign,
    // and with none set it takes the log label for one. A recording decoder hears every station.
    ModemEngine modem;
    modem.setSynchronousMode(true);
    if (state.session) modem.joinConnectedSession(*state.session, state.mod, state.rate);

    std::vector<Heard> heard;
    constexpr size_t kChunk = 960;                 // 20 ms
    constexpr size_t kPreroll = 24000;             // 0.5 s: a chirp at sample 0 is still found
    constexpr size_t kTail = 3 * 48000;            // flush the decoder's last frame
    size_t fed = 0;                                // samples fed, preroll included
    auto now_s = [&]() {
        return (static_cast<double>(fed) - static_cast<double>(kPreroll)) / 48000.0;
    };
    Heard measured;                                // metrics of the frame being delivered
    modem.setDecodeObserver([&](const DecodeResult& r) {
        measured = Heard{};
        measured.snr_db = r.snr_db;
        measured.cfo_hz = r.cfo_hz;
        measured.has_evm = r.has_evm_snr_db;
        measured.evm_db = r.evm_snr_db;
    });
    modem.setRawDataCallback([&](const Bytes& data, bool /*physical_turn_complete*/) {
        Heard h = measured;
        h.time_s = now_s();
        h.receiver = state.label;
        h.data = data;
        heard.push_back(std::move(h));
    });
    modem.setPingObserver([&](float snr_db, float cfo_hz) {
        Heard h;
        h.time_s = now_s();
        h.receiver = state.label;
        h.ping = true;
        h.snr_db = snr_db;
        h.cfo_hz = cfo_hz;
        heard.push_back(std::move(h));
    });

    const std::vector<float> silence(kChunk, 0.0f);
    auto feed = [&](const float* p, size_t n) {
        modem.feedAudio(p, n);
        fed += n;
        modem.processRxBuffer();
    };
    for (size_t i = 0; i < kPreroll && g_running; i += kChunk) feed(silence.data(), kChunk);
    for (size_t i = 0; i < audio.size() && g_running; i += kChunk) {
        feed(audio.data() + i, std::min(kChunk, audio.size() - i));
    }
    for (size_t i = 0; i < kTail && g_running; i += kChunk) feed(silence.data(), kChunk);
    return heard;
}

// Every coherent wideband rung, the ones the auto ladder can command.
std::vector<std::pair<Modulation, CodeRate>> allOfdmRungs() {
    std::vector<std::pair<Modulation, CodeRate>> rungs;
    for (uint8_t idx = protocol::kRungIdxQpskR14; idx < protocol::kRungIdxCount; ++idx) {
        const auto pick = protocol::coherentRungFromIndex(idx);
        rungs.emplace_back(pick.mod, pick.rate);
    }
    return rungs;
}

std::string describeFrame(const Heard& h) {
    std::ostringstream os;
    if (h.ping) {
        os << "PING";
        return os.str();
    }
    const Bytes& data = h.data;
    if (auto connect = v2::ConnectFrame::deserialize(data)) {
        char dst_hex[16];
        std::snprintf(dst_hex, sizeof(dst_hex), "0x%06X",
                      static_cast<unsigned>(connect->dst_hash & 0xFFFFFFu));
        os << v2::frameTypeToString(connect->type) << " from " << connect->getSrcCallsign()
           << " to (hash " << dst_hex << ")";
        return os.str();
    }
    if (auto df = v2::DataFrame::deserialize(data)) {
        os << v2::frameTypeToString(df->type) << " from 0x" << std::hex << std::uppercase
           << std::setw(6) << std::setfill('0') << df->src_hash << std::dec
           << std::nouppercase << std::setfill(' ') << ", " << df->payload.size()
           << " bytes: \"" << df->payloadAsText() << "\"";
        return os.str();
    }
    if (auto ctrl = v2::ControlFrame::deserialize(data)) {
        os << v2::frameTypeToString(ctrl->type) << " seq=" << ctrl->seq;
        return os.str();
    }
    const auto header = v2::parseHeader(data);
    if (header.valid) {
        os << v2::frameTypeToString(header.type) << " (" << data.size() << " bytes)";
    } else {
        os << "frame (" << data.size() << " bytes)";
    }
    return os.str();
}

bool loadAudio(const char* input_file, std::vector<float>& audio) {
    if (input_file && hasWavExtension(input_file)) {
        tools::io::LoadedWav wav;
        try {
            wav = tools::io::loadWavMono48k(input_file);
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
            return false;
        }
        std::cerr << "  WAV: " << wav.source_rate << " Hz, " << wav.source_bits << "-bit, "
                  << wav.source_channels << " channel(s)"
                  << (wav.source_rate != 48000 ? ", resampled to 48 kHz" : "") << "\n";
        audio = std::move(wav.samples_48k);
        return true;
    }
    std::ifstream infile;
    std::istream* input = &std::cin;
    if (input_file) {
        infile.open(input_file, std::ios::binary);
        if (!infile) {
            std::cerr << "Error: Cannot open " << input_file << "\n";
            return false;
        }
        input = &infile;
    }
    std::vector<float> buffer(4800);
    while (g_running) {
        input->read(reinterpret_cast<char*>(buffer.data()),
                    static_cast<std::streamsize>(buffer.size() * sizeof(float)));
        const size_t got = static_cast<size_t>(input->gcount()) / sizeof(float);
        audio.insert(audio.end(), buffer.begin(), buffer.begin() + static_cast<long>(got));
        if (got < buffer.size()) break;
    }
    return true;
}

int runProtocolRx(const char* input_file, WaveformType waveform,
                  std::optional<CodeRate> rate_opt, std::optional<Modulation> mod_opt) {
    std::cerr << "Protocol RX";
    if (input_file) std::cerr << " from " << input_file;
    std::cerr << "\n";

    if (mod_opt && !isOfdmDataModulation(*mod_opt)) {
        std::cerr << "Error: -m selects the OFDM data modulation: qpsk, 8psk, 16qam or 32qam\n";
        return 1;
    }

    std::vector<float> audio;
    if (!loadAudio(input_file, audio)) return 1;
    std::cerr << "  Audio: " << audio.size() << " samples (" << (audio.size() / 48000.0)
              << " sec at 48 kHz)\n";

    // The station states a frame of the recording could come from. A session tells a receiver
    // its rung (MODE_CHANGE, burst descriptors); a recording does not, so every candidate rung
    // gets its own receiver. Each hears the whole recording, as a GUI in --monitor mode would.
    std::vector<ReceiverState> states;
    if (waveform == WaveformType::AUTO || waveform == WaveformType::DPSK) {
        states.push_back({"MC-DPSK", std::nullopt});                    // PING, CONNECT, control
        states.push_back({"MC-DPSK", protocol::WaveformMode::MC_DPSK});  // MC-DPSK data
    }
    struct OfdmPlan {
        protocol::WaveformMode mode;
        std::vector<std::pair<Modulation, CodeRate>> rungs;
    };
    std::vector<OfdmPlan> plans;
    auto pinned = [&](std::vector<std::pair<Modulation, CodeRate>> rungs) {
        std::vector<std::pair<Modulation, CodeRate>> kept;
        for (const auto& r : rungs) {
            if (mod_opt && r.first != *mod_opt) continue;
            if (rate_opt && r.second != *rate_opt) continue;
            kept.push_back(r);
        }
        if (kept.empty() && mod_opt && rate_opt) kept.emplace_back(*mod_opt, *rate_opt);
        return kept;
    };
    if (waveform == WaveformType::AUTO || waveform == WaveformType::OFDM_CHIRP) {
        plans.push_back({protocol::WaveformMode::OFDM_CHIRP, pinned(allOfdmRungs())});
    }
    if (waveform == WaveformType::OFDM_NARROW) {
        plans.push_back({protocol::WaveformMode::OFDM_NARROW, pinned(allOfdmRungs())});
    } else if (waveform == WaveformType::AUTO && !mod_opt && !rate_opt) {
        // Narrowband sessions run QPSK R1/4 and R1/2.
        plans.push_back({protocol::WaveformMode::OFDM_NARROW,
                         {{Modulation::QPSK, CodeRate::R1_4}, {Modulation::QPSK, CodeRate::R1_2}}});
    }
    for (const auto& plan : plans) {
        for (const auto& [mod, rate] : plan.rungs) {
            states.push_back({std::string(plan.mode == protocol::WaveformMode::OFDM_NARROW
                                              ? "OFDM narrow "
                                              : "OFDM ") +
                                  modulationToString(mod) + " " + codeRateToString(rate),
                              plan.mode, mod, rate});
        }
    }
    std::vector<Heard> all;
    for (const auto& state : states) {
        if (!g_running) break;
        auto heard = listen(state, audio);
        all.insert(all.end(), std::make_move_iterator(heard.begin()),
                   std::make_move_iterator(heard.end()));
    }

    // One line per frame: several receivers can decode the same one (an MC-DPSK frame on both
    // MC-DPSK receivers, an OFDM control frame on every rung). Keep the first, earliest report.
    std::stable_sort(all.begin(), all.end(),
                     [](const Heard& a, const Heard& b) { return a.time_s < b.time_s; });
    // A PING is a bare dual chirp, and every OFDM frame of a recording opens with one: an idle
    // MC-DPSK receiver reads the chirp of a short OFDM frame followed by silence as a PING. Such
    // a PING is that frame's preamble, not a frame of its own.
    auto opensOfdmFrame = [&](const Heard& ping) {
        return std::any_of(all.begin(), all.end(), [&](const Heard& o) {
            return !o.ping && o.receiver.rfind("OFDM", 0) == 0 &&
                   o.time_s >= ping.time_s - 0.5 && o.time_s - ping.time_s < 4.0;
        });
    };
    std::vector<Heard> frames;
    for (auto& h : all) {
        if (h.ping && opensOfdmFrame(h)) continue;
        const bool duplicate = std::any_of(frames.begin(), frames.end(), [&](const Heard& f) {
            return f.ping == h.ping && f.data == h.data && std::fabs(f.time_s - h.time_s) < 1.5;
        });
        if (!duplicate) frames.push_back(std::move(h));
    }

    int pings = 0;
    for (const auto& f : frames) {
        if (f.ping) ++pings;
        std::cout << std::fixed << std::setprecision(2) << std::setw(8) << f.time_s << " s  "
                  << std::left << std::setw(18) << f.receiver << std::right << "  "
                  << describeFrame(f) << "  SNR " << std::setprecision(1) << f.snr_db
                  << " dB, CFO " << std::showpos << f.cfo_hz << std::noshowpos << " Hz";
        if (f.has_evm) std::cout << ", EVM " << f.evm_db << " dB";
        std::cout << "\n";
    }

    std::cerr << "\n=== RX Statistics ===\n";
    std::cerr << "  Audio: " << audio.size() << " samples (" << (audio.size() / 48000.0)
              << " sec)\n";
    std::cerr << "  Receivers: " << states.size() << "\n";
    std::cerr << "  Frames: " << frames.size() << "\n";
    std::cerr << "  PINGs: " << pings << "\n";

    return 0;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    struct LogFileCloser {
        void operator()(std::FILE* file) const {
            if (file) std::fclose(file);
        }
    };
    std::unique_ptr<std::FILE, LogFileCloser> log_file(nullptr);
    LogLevel log_level = LogLevel::INFO;
    bool log_level_set = false;
    bool log_categories_set = false;
    std::string log_categories;
    std::string log_file_path;

    const char* output_file = nullptr;
    const char* command = nullptr;
    const char* input_file = nullptr;
    std::string src_call = "N0CALL";
    std::string dst_call = "CQ";
    WaveformType waveform = WaveformType::AUTO;
    std::optional<CodeRate> rate;
    std::optional<Modulation> mod;

    setOperatorLogProfile();

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            src_call = argv[++i];
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            dst_call = argv[++i];
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            const char* value = argv[++i];
            auto parsed = parseWaveform(value);
            if (!parsed) {
                std::cerr << "Unknown waveform: " << value
                          << " (use auto, ofdm, narrow, mcdpsk)\n";
                return 1;
            }
            waveform = *parsed;
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            const char* value = argv[++i];
            auto parsed = tools::cli::parseModulation(value);
            if (!parsed) {
                std::cerr << "Unknown modulation: " << value << " (use qpsk, 8psk, 16qam, 32qam)\n";
                return 1;
            }
            mod = *parsed;
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            const char* value = argv[++i];
            auto parsed = tools::cli::parseCodeRate(value);
            if (!parsed) {
                std::cerr << "Unknown code rate: " << value
                          << " (use " << tools::cli::codeRateChoices() << ")\n";
                return 1;
            }
            rate = *parsed;
        } else if (strcmp(argv[i], "--log-level") == 0 && i + 1 < argc) {
            const char* value = argv[++i];
            if (!parseLogLevel(value, log_level)) {
                std::cerr << "Invalid --log-level: " << value
                          << " (use error, warn, info, debug, or trace)\n";
                return 1;
            }
            log_level_set = true;
        } else if ((strcmp(argv[i], "--log-category") == 0 ||
                    strcmp(argv[i], "--log-categories") == 0) && i + 1 < argc) {
            log_categories = argv[++i];
            log_categories_set = true;
        } else if (strcmp(argv[i], "--log-file") == 0 && i + 1 < argc) {
            log_file_path = argv[++i];
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            if (!command) {
                command = argv[i];
            } else if (!input_file) {
                input_file = argv[i];
            }
        } else {
            std::cerr << "Unknown option: " << argv[i] << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    // prx runs a dozen decoders over the same audio; their per-frame chatter would bury the
    // result lines, so it defaults to warnings only.
    if (!log_level_set && command && strcmp(command, "prx") == 0) {
        log_level = LogLevel::WARN;
    }
    setLogLevel(log_level);
    if (log_level_set && log_level >= LogLevel::DEBUG && !log_categories_set) {
        setDeveloperLogProfile();
    }
    if (log_categories_set && !setLogCategories(log_categories)) {
        std::cerr << "Invalid --log-category list: " << log_categories << "\n";
        return 1;
    }
    if (!log_file_path.empty()) {
        log_file.reset(std::fopen(log_file_path.c_str(), "a"));
        if (!log_file) {
            std::cerr << "Failed to open --log-file " << log_file_path << "\n";
            return 1;
        }
        setLogFile(log_file.get());
    }

    if (!command) {
        printUsage(argv[0]);
        return 1;
    }

    if (strcmp(command, "info") == 0) {
        printInfo();
        return 0;
    } else if (strcmp(command, "ptx") == 0) {
        return runProtocolTx(input_file, output_file, src_call, dst_call, waveform, rate, mod);
    } else if (strcmp(command, "prx") == 0) {
        sync::preferChirpAcquisitionWithoutHandshake();
        return runProtocolRx(input_file, waveform, rate, mod);
    } else {
        std::cerr << "Unknown command: " << command << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
