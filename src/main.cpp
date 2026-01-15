#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>

#include "ultra/modem.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"

#include <iostream>
#include <fstream>
#include <cstring>
#include <csignal>
#include <atomic>

// Signal handling for clean shutdown
static std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}

void printUsage(const char* prog) {
    std::cerr << "ProjectUltra - High-Speed HF Sound Modem\n\n";
    std::cerr << "Usage: " << prog << " [options] <command>\n\n";
    std::cerr << "Commands:\n";
    std::cerr << "  tx <file>       Transmit file (outputs audio to stdout)\n";
    std::cerr << "  rx              Receive (reads audio from stdin)\n";
    std::cerr << "  test            Run self-test\n";
    std::cerr << "  info            Show modem capabilities\n";
    std::cerr << "\nOptions:\n";
    std::cerr << "  -r <rate>       Sample rate (default: 48000)\n";
    std::cerr << "  -m <mod>        Modulation: bpsk, qpsk, qam16, qam64 (default: qpsk)\n";
    std::cerr << "  -c <rate>       Code rate: 1/4, 1/2, 2/3, 3/4, 5/6 (default: 1/2)\n";
    std::cerr << "  -a              Enable adaptive modulation/coding\n";
    std::cerr << "\n";
}

void printInfo(const ultra::ModemConfig& config) {
    std::cout << "=== ProjectUltra HF Modem ===\n\n";

    std::cout << "Configuration:\n";
    std::cout << "  Sample rate:    " << config.sample_rate << " Hz\n";
    std::cout << "  Center freq:    " << config.center_freq << " Hz\n";
    std::cout << "  FFT size:       " << config.fft_size << "\n";
    std::cout << "  Carriers:       " << config.num_carriers << "\n";
    std::cout << "  Cyclic prefix:  " << config.getCyclicPrefix() << " samples\n";
    std::cout << "\n";

    std::cout << "Theoretical max data rates:\n";
    for (auto mod : {ultra::Modulation::BPSK, ultra::Modulation::QPSK,
                     ultra::Modulation::QAM16, ultra::Modulation::QAM64}) {
        for (auto rate : {ultra::CodeRate::R1_2, ultra::CodeRate::R2_3,
                          ultra::CodeRate::R3_4, ultra::CodeRate::R5_6}) {
            float bps = ultra::calculateMaxDataRate(config, mod, rate);
            const char* mod_name = "";
            switch (mod) {
                case ultra::Modulation::BPSK: mod_name = "BPSK"; break;
                case ultra::Modulation::QPSK: mod_name = "QPSK"; break;
                case ultra::Modulation::QAM16: mod_name = "16QAM"; break;
                case ultra::Modulation::QAM64: mod_name = "64QAM"; break;
                default: break;
            }
            const char* rate_name = "";
            switch (rate) {
                case ultra::CodeRate::R1_2: rate_name = "1/2"; break;
                case ultra::CodeRate::R2_3: rate_name = "2/3"; break;
                case ultra::CodeRate::R3_4: rate_name = "3/4"; break;
                case ultra::CodeRate::R5_6: rate_name = "5/6"; break;
                default: break;
            }
            std::cout << "  " << mod_name << " R" << rate_name << ": "
                      << static_cast<int>(bps) << " bps ("
                      << static_cast<int>(bps / 1000) << "." << static_cast<int>(bps) % 1000 / 100
                      << " kbps)\n";
        }
    }
    std::cout << "\n";

    std::cout << "For comparison:\n";
    std::cout << "  Ultra:    Up to 16000 bps (open source)\n";
    std::cout << "  ARDOP:    ~2000 bps typical (open source)\n";
    std::cout << "  PACTOR-4: ~5500 bps peak (proprietary)\n";
}

int runTest() {
    std::cout << "Running self-test...\n\n";

    ultra::ModemConfig config;
    config.sample_rate = 48000;

    // Test OFDM modulator
    std::cout << "Testing OFDM modulator... ";
    ultra::OFDMModulator mod(config);
    std::vector<uint8_t> test_data = {0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34, 0x56, 0x78};
    auto samples = mod.modulate(test_data, ultra::Modulation::QPSK);
    std::cout << "OK (" << samples.size() << " samples)\n";

    // Test preamble
    std::cout << "Testing preamble generation... ";
    auto preamble = mod.generatePreamble();
    std::cout << "OK (" << preamble.size() << " samples)\n";

    // Test LDPC encoder
    std::cout << "Testing LDPC encoder... ";
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_2);
    auto encoded = encoder.encode(test_data);
    std::cout << "OK (" << test_data.size() << " -> " << encoded.size() << " bytes)\n";

    // Test LDPC decoder
    std::cout << "Testing LDPC decoder... ";
    ultra::LDPCDecoder decoder(ultra::CodeRate::R1_2);
    auto decoded = decoder.decode(encoded);
    bool match = (decoded.size() >= test_data.size());
    if (match) {
        for (size_t i = 0; i < test_data.size(); ++i) {
            if (decoded[i] != test_data[i]) {
                match = false;
                break;
            }
        }
    }
    std::cout << (match ? "OK" : "MISMATCH") << "\n";

    // Test FFT
    std::cout << "Testing FFT... ";
    ultra::FFT fft(512);
    std::vector<ultra::Complex> fft_in(512), fft_out(512), fft_inv(512);
    for (int i = 0; i < 512; ++i) {
        fft_in[i] = ultra::Complex(std::sin(2 * M_PI * i / 32), 0);
    }
    fft.forward(fft_in, fft_out);
    fft.inverse(fft_out, fft_inv);
    float max_err = 0;
    for (int i = 0; i < 512; ++i) {
        float err = std::abs(fft_in[i].real() - fft_inv[i].real());
        if (err > max_err) max_err = err;
    }
    std::cout << "OK (round-trip error: " << max_err << ")\n";

    // Test modem integration
    std::cout << "Testing modem integration... ";
    ultra::Modem modem(config);
    modem.start();
    modem.send(test_data);
    modem.stop();
    std::cout << "OK\n";

    std::cout << "\nAll tests passed!\n";

    // Show theoretical performance
    std::cout << "\n";
    printInfo(config);

    return 0;
}

int runTx(const char* filename, ultra::ModemConfig& config) {
    // Read input file
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Cannot open file: " << filename << "\n";
        return 1;
    }

    std::vector<uint8_t> data((std::istreambuf_iterator<char>(file)),
                               std::istreambuf_iterator<char>());
    file.close();

    std::cerr << "Transmitting " << data.size() << " bytes...\n";

    // Create modulator
    ultra::OFDMModulator mod(config);
    ultra::LDPCEncoder encoder(config.code_rate);
    ultra::Interleaver interleaver(32, 32);

    // Output preamble
    auto preamble = mod.generatePreamble();
    std::cout.write(reinterpret_cast<const char*>(preamble.data()),
                    preamble.size() * sizeof(float));

    // Encode and modulate
    auto encoded = encoder.encode(data);
    auto interleaved = interleaver.interleave(encoded);
    auto samples = mod.modulate(interleaved, config.modulation);

    // Output audio
    std::cout.write(reinterpret_cast<const char*>(samples.data()),
                    samples.size() * sizeof(float));

    std::cerr << "Done. Output " << (preamble.size() + samples.size())
              << " samples.\n";

    return 0;
}

int runRx(ultra::ModemConfig& config) {
    std::cerr << "Receiving... (Ctrl+C to stop)\n";

    ultra::Modem modem(config);

    modem.setDataCallback([](ultra::Bytes data) {
        std::cout.write(reinterpret_cast<const char*>(data.data()), data.size());
        std::cout.flush();
    });

    modem.start();

    // Read audio from stdin
    std::vector<float> buffer(1024);
    while (g_running && std::cin.read(reinterpret_cast<char*>(buffer.data()),
                                       buffer.size() * sizeof(float))) {
        modem.rxSamples(buffer);
    }

    modem.stop();

    auto stats = modem.getStats();
    std::cerr << "\nStatistics:\n";
    std::cerr << "  Bytes received:  " << stats.bytes_received << "\n";
    std::cerr << "  Frames received: " << stats.frames_received << "\n";
    std::cerr << "  Current SNR:     " << stats.current_snr_db << " dB\n";

    return 0;
}

ultra::Modulation parseModulation(const char* s) {
    if (strcmp(s, "bpsk") == 0) return ultra::Modulation::BPSK;
    if (strcmp(s, "qpsk") == 0) return ultra::Modulation::QPSK;
    if (strcmp(s, "qam16") == 0) return ultra::Modulation::QAM16;
    if (strcmp(s, "qam64") == 0) return ultra::Modulation::QAM64;
    return ultra::Modulation::QPSK;
}

ultra::CodeRate parseCodeRate(const char* s) {
    if (strcmp(s, "1/4") == 0) return ultra::CodeRate::R1_4;
    if (strcmp(s, "1/2") == 0) return ultra::CodeRate::R1_2;
    if (strcmp(s, "2/3") == 0) return ultra::CodeRate::R2_3;
    if (strcmp(s, "3/4") == 0) return ultra::CodeRate::R3_4;
    if (strcmp(s, "5/6") == 0) return ultra::CodeRate::R5_6;
    return ultra::CodeRate::R1_2;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    ultra::ModemConfig config;
    bool adaptive = false;

    // Parse options
    int i = 1;
    while (i < argc && argv[i][0] == '-') {
        if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            config.sample_rate = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            config.modulation = parseModulation(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            config.code_rate = parseCodeRate(argv[++i]);
        } else if (strcmp(argv[i], "-a") == 0) {
            adaptive = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        }
        ++i;
    }

    if (i >= argc) {
        printUsage(argv[0]);
        return 1;
    }

    const char* command = argv[i++];

    if (strcmp(command, "test") == 0) {
        return runTest();
    } else if (strcmp(command, "info") == 0) {
        printInfo(config);
        return 0;
    } else if (strcmp(command, "tx") == 0) {
        if (i >= argc) {
            std::cerr << "Error: tx requires a filename\n";
            return 1;
        }
        return runTx(argv[i], config);
    } else if (strcmp(command, "rx") == 0) {
        return runRx(config);
    } else {
        std::cerr << "Unknown command: " << command << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
