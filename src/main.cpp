/**
 * ProjectUltra CLI - High-Speed HF Sound Modem
 *
 * Uses ModemEngine for clean TX/RX handling (same as GUI).
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "gui/modem/modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/types.hpp"

#include <iostream>
#include <fstream>
#include <cstring>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

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
    std::cerr << "                    ptx ping         -> PING probe (~1 sec, DPSK)\n";
    std::cerr << "                    ptx connect      -> CONNECT (with callsigns)\n";
    std::cerr << "                    ptx disconnect   -> DISCONNECT (end session)\n";
    std::cerr << "                    ptx \"Hello\"      -> DATA (text message)\n";
    std::cerr << "  prx [file]      Protocol RX - decode v2 frames (from file or stdin)\n";
    std::cerr << "  info            Show modem capabilities\n";
    std::cerr << "\nOptions:\n";
    std::cerr << "  -s <call>       Source callsign (default: N0CALL)\n";
    std::cerr << "  -d <call>       Destination callsign (default: CQ)\n";
    std::cerr << "  -o <file>       Output to file instead of stdout\n";
    std::cerr << "  -w <waveform>   Waveform: ofdm, dpsk (default: ofdm)\n";
    std::cerr << "\nExamples:\n";
    std::cerr << "  # Send PING probe and play audio\n";
    std::cerr << "  " << prog << " ptx ping -s MYCALL | aplay -f FLOAT_LE -r 48000\n\n";
    std::cerr << "  # Send CONNECT frame to file\n";
    std::cerr << "  " << prog << " ptx connect -s MYCALL -d THEIRCALL -o connect.f32\n\n";
    std::cerr << "  # Decode recording (auto-detect waveform)\n";
    std::cerr << "  " << prog << " prx recording.f32\n\n";
    std::cerr << "  # Decode DPSK recording\n";
    std::cerr << "  " << prog << " prx -w dpsk recording.f32\n\n";
    std::cerr << "  # Loopback test\n";
    std::cerr << "  " << prog << " ptx ping | " << prog << " prx -w dpsk\n";
    std::cerr << "\n";
}

void printInfo() {
    std::cout << "=== ProjectUltra HF Modem ===\n\n";
    std::cout << "Signal Parameters:\n";
    std::cout << "  Sample rate:    48000 Hz\n";
    std::cout << "  Center freq:    1500 Hz\n";
    std::cout << "  Bandwidth:      ~2.8 kHz\n";
    std::cout << "  OFDM carriers:  30\n";
    std::cout << "  LDPC codeword:  648 bits\n\n";

    std::cout << "Waveforms:\n";
    std::cout << "  OFDM    High throughput, good SNR (>17 dB)\n";
    std::cout << "  DPSK    Single-carrier, low SNR (-11 to 17 dB)\n\n";

    std::cout << "Code Rates:\n";
    std::cout << "  R1/4    20 info bytes, most robust\n";
    std::cout << "  R1/2    40 info bytes\n";
    std::cout << "  R2/3    54 info bytes\n";
    std::cout << "  R3/4    60 info bytes\n";
    std::cout << "  R5/6    67 info bytes, highest throughput\n";
}

// Waveform type
enum class WaveformType { OFDM, DPSK };

WaveformType parseWaveform(const char* s) {
    if (strcmp(s, "dpsk") == 0) return WaveformType::DPSK;
    return WaveformType::OFDM;
}

protocol::WaveformMode toWaveformMode(WaveformType w) {
    switch (w) {
        case WaveformType::DPSK: return protocol::WaveformMode::DPSK;
        default: return protocol::WaveformMode::OFDM_NVIS;
    }
}

// ============================================================================
// Protocol TX - Uses ModemEngine for clean audio generation
// ============================================================================
int runProtocolTx(const char* message, const char* output_file,
                  const std::string& src_call, const std::string& dst_call,
                  WaveformType waveform) {

    std::cerr << "Protocol TX: " << src_call << " -> " << dst_call << "\n";

    ModemEngine modem;
    modem.setLogPrefix("TX");
    modem.setWaveformMode(toWaveformMode(waveform));

    std::vector<float> samples;
    std::string frame_type;

    // Determine frame type and generate audio
    if (!message || strlen(message) == 0 || strcmp(message, "ping") == 0) {
        // PING probe - raw DPSK, no LDPC
        frame_type = "PING";
        samples = modem.transmitPing();

    } else if (strcmp(message, "connect") == 0) {
        // CONNECT frame with full callsigns
        frame_type = "CONNECT";
        auto frame = v2::ConnectFrame::makeConnect(src_call, dst_call, 0xFF, 0x00);
        samples = modem.transmit(frame.serialize());

    } else if (strcmp(message, "disconnect") == 0) {
        // DISCONNECT frame with full callsigns
        frame_type = "DISCONNECT";
        auto frame = v2::ConnectFrame::makeDisconnect(src_call, dst_call);
        samples = modem.transmit(frame.serialize());

    } else {
        // DATA frame with text message
        frame_type = "DATA";
        auto frame = v2::DataFrame::makeData(src_call, dst_call, 1, std::string(message));
        samples = modem.transmit(frame.serialize());
    }

    std::cerr << "  Frame: " << frame_type << "\n";
    std::cerr << "  Samples: " << samples.size() << " ("
              << (samples.size() / 48000.0) << " sec)\n";

    // Write output
    if (output_file) {
        std::ofstream out(output_file, std::ios::binary);
        if (!out) {
            std::cerr << "Error: Cannot open " << output_file << "\n";
            return 1;
        }
        out.write(reinterpret_cast<const char*>(samples.data()),
                  samples.size() * sizeof(float));
        std::cerr << "  Written to: " << output_file << "\n";
    } else {
        std::cout.write(reinterpret_cast<const char*>(samples.data()),
                        samples.size() * sizeof(float));
    }

    return 0;
}

// ============================================================================
// Protocol RX - Uses ModemEngine with callbacks for clean frame decoding
// ============================================================================
int runProtocolRx(const char* input_file, WaveformType waveform) {

    std::cerr << "Protocol RX";
    if (input_file) std::cerr << " from " << input_file;
    std::cerr << "\n";

    // Open input
    std::ifstream infile;
    std::istream* input = &std::cin;
    if (input_file) {
        infile.open(input_file, std::ios::binary);
        if (!infile) {
            std::cerr << "Error: Cannot open " << input_file << "\n";
            return 1;
        }
        input = &infile;
    }

    // Statistics
    std::atomic<int> frames_received{0};
    std::atomic<int> pings_received{0};
    std::atomic<bool> got_frame{false};

    // Create modem engine
    ModemEngine modem;
    modem.setLogPrefix("RX");
    modem.setWaveformMode(toWaveformMode(waveform));

    // Callback for PING detection
    modem.setPingReceivedCallback([&](float snr) {
        pings_received++;
        frames_received++;
        got_frame = true;
        std::cerr << "  [PING] Detected! (SNR=" << snr << " dB)\n";
    });

    // Callback for decoded frames (LDPC-decoded data)
    modem.setRawDataCallback([&](const Bytes& data) {
        if (data.empty()) return;

        // Try to identify and parse the frame
        auto cw_info = v2::identifyCodeword(data);

        if (cw_info.type == v2::CodewordType::HEADER) {
            auto header = v2::parseHeader(data);
            if (header.valid) {
                frames_received++;
                got_frame = true;
                std::cerr << "  [" << v2::frameTypeToString(header.type) << "] ";

                // For single-codeword frames (control frames)
                if (header.total_cw == 1) {
                    auto ctrl = v2::ControlFrame::deserialize(data);
                    if (ctrl) {
                        std::cerr << "seq=" << ctrl->seq << "\n";
                    } else {
                        std::cerr << "\n";
                    }
                } else {
                    std::cerr << "codewords=" << (int)header.total_cw << "\n";
                }
            }
        }

        // Try parsing as ConnectFrame (CONNECT, DISCONNECT, etc.)
        auto connect = v2::ConnectFrame::deserialize(data);
        if (connect) {
            std::cerr << "    " << connect->getSrcCallsign()
                      << " -> " << connect->getDstCallsign() << "\n";
            return;
        }

        // Try parsing as DataFrame
        auto df = v2::DataFrame::deserialize(data);
        if (df) {
            std::cerr << "    Message: \"" << df->payloadAsText() << "\"\n";
            return;
        }
    });

    // Read and process audio in chunks, with small delays to let threads process
    std::vector<float> buffer(960);  // 20ms chunks (smaller for better threading)
    size_t total_samples = 0;

    while (g_running && input->read(reinterpret_cast<char*>(buffer.data()),
                                     buffer.size() * sizeof(float))) {
        modem.feedAudio(buffer);
        total_samples += buffer.size();
    }

    // Handle partial read at end
    size_t bytes_read = input->gcount();
    if (bytes_read > 0) {
        size_t samples_read = bytes_read / sizeof(float);
        buffer.resize(samples_read);
        modem.feedAudio(buffer);
        total_samples += samples_read;
    }

    // Wait for modem to process buffered audio
    // ModemEngine has background threads - wait until frame received or timeout
    auto wait_start = std::chrono::steady_clock::now();
    while (!got_frame) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - wait_start).count();
        if (elapsed > 5000) break;  // 5 second timeout
    }

    // Small extra delay to catch any remaining callbacks
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cerr << "\n=== RX Statistics ===\n";
    std::cerr << "  Audio: " << total_samples << " samples ("
              << (total_samples / 48000.0) << " sec)\n";
    std::cerr << "  Frames: " << frames_received.load() << "\n";
    std::cerr << "  PINGs: " << pings_received.load() << "\n";

    return 0;
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    const char* output_file = nullptr;
    const char* command = nullptr;
    const char* input_file = nullptr;
    std::string src_call = "N0CALL";
    std::string dst_call = "CQ";
    WaveformType waveform = WaveformType::OFDM;

    // Parse arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            src_call = argv[++i];
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            dst_call = argv[++i];
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            waveform = parseWaveform(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            if (!command) {
                command = argv[i];
            } else if (!input_file) {
                input_file = argv[i];
            }
        }
    }

    if (!command) {
        printUsage(argv[0]);
        return 1;
    }

    if (strcmp(command, "info") == 0) {
        printInfo();
        return 0;
    } else if (strcmp(command, "ptx") == 0) {
        return runProtocolTx(input_file, output_file, src_call, dst_call, waveform);
    } else if (strcmp(command, "prx") == 0) {
        return runProtocolRx(input_file, waveform);
    } else {
        std::cerr << "Unknown command: " << command << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
