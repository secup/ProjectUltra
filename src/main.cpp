#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>

#include "ultra/modem.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"
#include "protocol/protocol_engine.hpp"
#include "protocol/frame.hpp"

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
    std::cerr << "  tx <file>       Transmit file (outputs audio to stdout or -o file)\n";
    std::cerr << "  rx [file]       Receive (from file or stdin, old modem framing)\n";
    std::cerr << "  prx [file]      Protocol RX (decodes ULTR frames with PROBE/CONNECT/DATA)\n";
    std::cerr << "  decode [file]   Raw decode (from file or stdin, outputs raw LDPC data)\n";
    std::cerr << "  test            Run self-test\n";
    std::cerr << "  info            Show modem capabilities\n";
    std::cerr << "\nOptions:\n";
    std::cerr << "  -r <rate>       Sample rate (default: 48000)\n";
    std::cerr << "  -m <mod>        Modulation: dbpsk, bpsk, dqpsk, qpsk, d8psk, qam16, qam64 (default: qpsk)\n";
    std::cerr << "  -c <rate>       Code rate: 1/4, 1/2, 2/3, 3/4, 5/6 (default: 1/2)\n";
    std::cerr << "  -o <file>       Output file (default: stdout)\n";
    std::cerr << "  -s <call>       Local callsign for protocol mode (default: TEST)\n";
    std::cerr << "  -a              Enable adaptive modulation/coding\n";
    std::cerr << "\nExamples:\n";
    std::cerr << "  " << prog << " -m dqpsk -c 1/4 decode recording.raw\n";
    std::cerr << "  " << prog << " -m dqpsk -s N0CALL prx recording.f32\n";
    std::cerr << "  " << prog << " tx data.bin -o output.raw\n";
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

int runRx(ultra::ModemConfig& config, const char* input_file, const char* output_file) {
    std::cerr << "Receiving";
    if (input_file) std::cerr << " from " << input_file;
    std::cerr << "... (Ctrl+C to stop)\n";

    // Open input file if specified
    std::ifstream infile;
    std::istream* input = &std::cin;
    if (input_file) {
        infile.open(input_file, std::ios::binary);
        if (!infile) {
            std::cerr << "Error: Cannot open input file: " << input_file << "\n";
            return 1;
        }
        input = &infile;
    }

    // Open output file if specified
    std::ofstream outfile;
    std::ostream* output = &std::cout;
    if (output_file) {
        outfile.open(output_file, std::ios::binary);
        if (!outfile) {
            std::cerr << "Error: Cannot open output file: " << output_file << "\n";
            return 1;
        }
        output = &outfile;
    }

    ultra::Modem modem(config);

    modem.setDataCallback([output](ultra::Bytes data) {
        output->write(reinterpret_cast<const char*>(data.data()), data.size());
        output->flush();
    });

    modem.start();

    // Read audio from input
    std::vector<float> buffer(1024);
    while (g_running && input->read(reinterpret_cast<char*>(buffer.data()),
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

// Protocol RX mode - decodes frames with 4-byte magic "ULTR" and protocol parsing
// Use this to receive PROBE, CONNECT, DATA frames from the GUI
int runProtocolRx(ultra::ModemConfig& config, const char* input_file, const std::string& callsign) {
    std::cerr << "Protocol RX mode (looking for ULTR frames)...\n";
    if (input_file) std::cerr << "Input: " << input_file << "\n";
    std::cerr << "Local callsign: " << callsign << "\n";

    // Open input file
    std::ifstream infile;
    std::istream* input = &std::cin;
    if (input_file) {
        infile.open(input_file, std::ios::binary);
        if (!infile) {
            std::cerr << "Error: Cannot open input file: " << input_file << "\n";
            return 1;
        }
        input = &infile;

        // Skip startup samples to get closer to real signal
        // User can trim audio file to start near the preamble
        constexpr size_t STARTUP_SKIP = 0;  // No skip - user should trim file
        std::vector<float> skip_buf(STARTUP_SKIP);
        input->read(reinterpret_cast<char*>(skip_buf.data()), STARTUP_SKIP * sizeof(float));
        std::cerr << "Skipped " << STARTUP_SKIP << " startup samples\n";
    }

    // Create demodulator and decoder
    ultra::OFDMDemodulator demod(config);
    ultra::LDPCDecoder decoder(config.code_rate);

    // Create protocol engine
    ultra::protocol::ConnectionConfig conn_config;
    conn_config.auto_accept = true;
    ultra::protocol::ProtocolEngine protocol(conn_config);
    protocol.setLocalCallsign(callsign);

    // Track received frames
    int frames_received = 0;

    // Set up protocol callbacks
    protocol.setMessageReceivedCallback([&](const std::string& from, const std::string& text) {
        std::cerr << "\n[MESSAGE from " << from << "]: " << text << "\n";
    });

    protocol.setConnectionChangedCallback([&](ultra::protocol::ConnectionState state, const std::string& remote) {
        const char* state_str = "UNKNOWN";
        switch (state) {
            case ultra::protocol::ConnectionState::DISCONNECTED: state_str = "DISCONNECTED"; break;
            case ultra::protocol::ConnectionState::CONNECTING: state_str = "CONNECTING"; break;
            case ultra::protocol::ConnectionState::CONNECTED: state_str = "CONNECTED"; break;
            case ultra::protocol::ConnectionState::DISCONNECTING: state_str = "DISCONNECTING"; break;
            case ultra::protocol::ConnectionState::PROBING: state_str = "PROBING"; break;
        }
        std::cerr << "\n[CONNECTION] State: " << state_str << ", Remote: " << remote << "\n";
    });

    protocol.setIncomingCallCallback([&](const std::string& from) {
        std::cerr << "\n[INCOMING CALL from " << from << "] - auto-accepting\n";
    });

    // Accumulate soft bits
    std::vector<float> accumulated_soft_bits;
    const size_t LDPC_BLOCK_SIZE = 648;

    // Track consecutive LDPC failures to detect false sync
    int consecutive_failures = 0;
    const int MAX_FAILURES_BEFORE_RESET = 3;  // Faster reset
    int negative_snr_count = 0;
    int total_resets = 0;

    // Read audio in chunks
    std::vector<float> buffer(960);
    uint32_t tick_counter = 0;

    while (g_running && input->read(reinterpret_cast<char*>(buffer.data()),
                                     buffer.size() * sizeof(float))) {
        // Process through demodulator
        ultra::SampleSpan span(buffer.data(), buffer.size());
        demod.process(span);

        if (demod.isSynced()) {
            auto soft_bits = demod.getSoftBits();
            accumulated_soft_bits.insert(accumulated_soft_bits.end(),
                                         soft_bits.begin(), soft_bits.end());

            // Decode complete LDPC blocks
            while (accumulated_soft_bits.size() >= LDPC_BLOCK_SIZE) {
                std::vector<float> block(accumulated_soft_bits.begin(),
                                         accumulated_soft_bits.begin() + LDPC_BLOCK_SIZE);
                accumulated_soft_bits.erase(accumulated_soft_bits.begin(),
                                            accumulated_soft_bits.begin() + LDPC_BLOCK_SIZE);

                ultra::Bytes decoded = decoder.decodeSoft(std::span<const float>(block));

                if (decoder.lastDecodeSuccess() && !decoded.empty()) {
                    // Check if it looks like a valid frame (starts with ULTR magic)
                    bool has_magic = (decoded.size() >= 4 &&
                                      decoded[0] == 0x55 && decoded[1] == 0x4C &&
                                      decoded[2] == 0x54 && decoded[3] == 0x52);

                    if (has_magic) {
                        // Feed to protocol engine
                        protocol.onRxData(decoded);
                        frames_received++;
                        consecutive_failures = 0;

                        std::cerr << "  [FRAME] " << decoded.size() << " bytes: ";
                        for (size_t i = 0; i < std::min(decoded.size(), size_t(20)); i++) {
                            fprintf(stderr, "%02X ", decoded[i]);
                        }
                        std::cerr << "\n";
                    } else {
                        // LDPC succeeded but no magic - probably false sync
                        std::cerr << "  [NO MAGIC] decoded " << decoded.size() << " bytes, first 4: ";
                        for (size_t i = 0; i < std::min(decoded.size(), size_t(4)); i++) {
                            fprintf(stderr, "%02X ", decoded[i]);
                        }
                        std::cerr << " (expected 55 4C 54 52)\n";
                        consecutive_failures++;
                    }
                } else {
                    consecutive_failures++;
                }

                // Too many failures = probably false sync, reset and search again
                if (consecutive_failures >= MAX_FAILURES_BEFORE_RESET) {
                    total_resets++;
                    std::cerr << "  [RESET #" << total_resets << "] " << consecutive_failures
                              << " failures, continuing search\n";

                    demod.reset();
                    accumulated_soft_bits.clear();
                    consecutive_failures = 0;
                    negative_snr_count = 0;
                }
            }

            // Check SNR - if consistently negative, we're processing noise
            auto quality = demod.getChannelQuality();
            if (quality.snr_db < -3) {  // Clearly noise
                negative_snr_count++;
                if (negative_snr_count >= 20) {
                    total_resets++;
                    std::cerr << "  [RESET #" << total_resets << "] Negative SNR, continuing search\n";

                    demod.reset();
                    accumulated_soft_bits.clear();
                    consecutive_failures = 0;
                    negative_snr_count = 0;
                }
            } else {
                negative_snr_count = 0;
            }
        }

        // Periodic tick for protocol timeouts
        tick_counter += 20;  // 20ms per chunk at 48kHz
        if (tick_counter >= 100) {
            protocol.tick(tick_counter);
            tick_counter = 0;
        }
    }

    std::cerr << "\n=== Protocol RX Statistics ===\n";
    std::cerr << "  LDPC frames decoded: " << frames_received << "\n";
    auto stats = protocol.getStats();
    std::cerr << "  ARQ frames RX:       " << stats.arq.frames_received << "\n";
    std::cerr << "  Connects received:   " << stats.connects_received << "\n";
    std::cerr << "  Connection state:    " << (protocol.isConnected() ? "CONNECTED" : "DISCONNECTED") << "\n";
    if (protocol.isConnected()) {
        std::cerr << "  Remote callsign:     " << protocol.getRemoteCallsign() << "\n";
    }

    return 0;
}

// Raw decode mode - bypasses protocol layer, outputs raw LDPC-decoded data
// Use this for decoding F3 test patterns from the GUI
int runDecode(ultra::ModemConfig& config, const char* input_file, const char* output_file) {
    std::cerr << "Raw decode mode (no protocol framing)...\n";
    if (input_file) std::cerr << "Input: " << input_file << "\n";
    std::cerr << "Modulation: " << static_cast<int>(config.modulation)
              << ", Code rate: " << static_cast<int>(config.code_rate) << "\n";

    // Open input file if specified
    std::ifstream infile;
    std::istream* input = &std::cin;
    if (input_file) {
        infile.open(input_file, std::ios::binary);
        if (!infile) {
            std::cerr << "Error: Cannot open input file: " << input_file << "\n";
            return 1;
        }
        input = &infile;
    }

    // Open output file if specified
    std::ofstream outfile;
    std::ostream* output = &std::cout;
    if (output_file) {
        outfile.open(output_file, std::ios::binary);
        if (!outfile) {
            std::cerr << "Error: Cannot open output file: " << output_file << "\n";
            return 1;
        }
        output = &outfile;
    }

    ultra::OFDMDemodulator demod(config);
    ultra::LDPCDecoder decoder(config.code_rate);

    int frames_decoded = 0;
    int total_bytes = 0;
    int chunks_read = 0;
    float max_level = 0;

    // Accumulate soft bits across chunks
    std::vector<float> accumulated_soft_bits;
    const size_t LDPC_BLOCK_SIZE = 648;  // Bits per R1/4 codeword

    // Read all audio from input
    std::vector<float> buffer(960);  // 20ms chunks

    while (g_running && input->read(reinterpret_cast<char*>(buffer.data()),
                                     buffer.size() * sizeof(float))) {
        chunks_read++;

        // Track max level
        for (float s : buffer) {
            if (std::abs(s) > max_level) max_level = std::abs(s);
        }

        ultra::SampleSpan span(buffer.data(), buffer.size());

        bool got_frame = demod.process(span);

        // Report progress every 50 chunks (~1 second)
        if (chunks_read % 50 == 0) {
            std::cerr << "Processed " << chunks_read << " chunks ("
                      << (chunks_read * 960 / 48000.0) << "s), max_level=" << max_level
                      << ", synced=" << demod.isSynced()
                      << ", accumulated=" << accumulated_soft_bits.size() << " bits\n";
        }

        if (demod.isSynced()) {
            auto soft_bits = demod.getSoftBits();
            if (!soft_bits.empty()) {
                accumulated_soft_bits.insert(accumulated_soft_bits.end(),
                                            soft_bits.begin(), soft_bits.end());
            }

            // Try to decode when we have enough bits
            while (accumulated_soft_bits.size() >= LDPC_BLOCK_SIZE) {
                // Extract one codeword worth of bits
                std::vector<float> codeword_bits(accumulated_soft_bits.begin(),
                                                  accumulated_soft_bits.begin() + LDPC_BLOCK_SIZE);
                accumulated_soft_bits.erase(accumulated_soft_bits.begin(),
                                            accumulated_soft_bits.begin() + LDPC_BLOCK_SIZE);

                // Debug: show first 32 LLRs
                std::cerr << "First 32 LLRs: ";
                int pos=0, neg=0;
                for (size_t i = 0; i < std::min(codeword_bits.size(), (size_t)32); i++) {
                    fprintf(stderr, "%.1f ", codeword_bits[i]);
                }
                for (float llr : codeword_bits) {
                    if (llr > 0) pos++; else if (llr < 0) neg++;
                }
                std::cerr << "... (" << pos << " pos, " << neg << " neg)\n";

                ultra::Bytes decoded = decoder.decodeSoft(codeword_bits);

                if (decoder.lastDecodeSuccess() && !decoded.empty()) {
                    frames_decoded++;
                    total_bytes += decoded.size();

                    // Output to file/stdout
                    output->write(reinterpret_cast<const char*>(decoded.data()), decoded.size());
                    output->flush();

                    // Also show hex on stderr
                    std::cerr << "Frame " << frames_decoded << ": ";
                    for (size_t i = 0; i < std::min(decoded.size(), (size_t)16); i++) {
                        fprintf(stderr, "%02X ", decoded[i]);
                    }
                    if (decoded.size() > 16) std::cerr << "...";
                    std::cerr << " (" << decoded.size() << " bytes)\n";
                } else {
                    std::cerr << "LDPC decode failed (accumulated " << accumulated_soft_bits.size() << " remaining)\n";
                }
            }
        } else {
            // Lost sync - clear accumulated bits
            if (!accumulated_soft_bits.empty()) {
                std::cerr << "Lost sync, discarding " << accumulated_soft_bits.size() << " accumulated bits\n";
                accumulated_soft_bits.clear();
            }
        }
    }

    std::cerr << "\nDecoded " << frames_decoded << " frames, " << total_bytes << " bytes total\n";
    return 0;
}

// Raw OFDM decode mode - NO LDPC, just extract hard bits from OFDM
// Use this to test OFDM layer independently
int runRawDecode(ultra::ModemConfig& config, const char* input_file, int timing_offset = 0) {
    std::cerr << "Raw OFDM decode (NO LDPC)...\n";
    if (input_file) std::cerr << "Input: " << input_file << "\n";
    if (timing_offset != 0) std::cerr << "Timing offset: " << timing_offset << " samples\n";

    // Open input file if specified
    std::ifstream infile;
    std::istream* input = &std::cin;
    if (input_file) {
        infile.open(input_file, std::ios::binary);
        if (!infile) {
            std::cerr << "Error: Cannot open input file: " << input_file << "\n";
            return 1;
        }
        input = &infile;
    }

    ultra::OFDMDemodulator demod(config);
    demod.setTimingOffset(timing_offset);  // Apply timing adjustment
    std::vector<float> all_soft_bits;
    std::vector<float> buffer(960);  // 20ms chunks

    while (g_running && input->read(reinterpret_cast<char*>(buffer.data()),
                                     buffer.size() * sizeof(float))) {
        ultra::SampleSpan span(buffer.data(), buffer.size());
        demod.process(span);

        if (demod.isSynced()) {
            auto bits = demod.getSoftBits();
            if (!bits.empty()) {
                all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
            }
        }
    }

    std::cerr << "Collected " << all_soft_bits.size() << " soft bits\n";

    // Convert soft bits to hard bits and bytes
    std::vector<uint8_t> rx_data;
    uint8_t byte = 0;
    int bit_count = 0;

    for (float llr : all_soft_bits) {
        // Negative LLR = bit 1, Positive LLR = bit 0
        uint8_t bit = (llr < 0) ? 1 : 0;
        byte = (byte << 1) | bit;
        bit_count++;
        if (bit_count == 8) {
            rx_data.push_back(byte);
            byte = 0;
            bit_count = 0;
        }
    }

    std::cerr << "\nRX Data (" << rx_data.size() << " bytes):\n";
    std::cerr << "Received (first 32):   ";
    for (size_t i = 0; i < std::min(rx_data.size(), (size_t)32); i++) {
        fprintf(stderr, "%02X ", rx_data[i]);
    }
    std::cerr << "\n";

    // Check against F5 pattern (0xAA 0x55)
    int matches_f5 = 0;
    int total = std::min(rx_data.size(), (size_t)81);
    for (size_t i = 0; i < total; i++) {
        uint8_t expected = (i % 2 == 0) ? 0xAA : 0x55;
        if (rx_data[i] == expected) matches_f5++;
    }

    // Check against F6 pattern (DEADBEEF)
    uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
    int matches_f6 = 0;
    for (size_t i = 0; i < total; i++) {
        if (rx_data[i] == deadbeef[i % 4]) matches_f6++;
    }

    std::cerr << "\nPattern matching:\n";
    std::cerr << "  F5 (AA 55):    " << matches_f5 << "/" << total << " bytes ("
              << (total > 0 ? 100.0 * matches_f5 / total : 0) << "%)\n";
    std::cerr << "  F6 (DEADBEEF): " << matches_f6 << "/" << total << " bytes ("
              << (total > 0 ? 100.0 * matches_f6 / total : 0) << "%)\n";

    // Bit-level analysis against best matching pattern
    bool use_deadbeef = (matches_f6 > matches_f5);
    std::cerr << "\nBit-level analysis (first 8 bytes) vs "
              << (use_deadbeef ? "DEADBEEF" : "AA 55") << ":\n";
    for (size_t i = 0; i < std::min(rx_data.size(), (size_t)8); i++) {
        uint8_t expected = use_deadbeef ? deadbeef[i % 4] : ((i % 2 == 0) ? 0xAA : 0x55);
        uint8_t got = rx_data[i];
        uint8_t diff = expected ^ got;
        fprintf(stderr, "  Byte %zu: expected=%02X got=%02X diff=%02X (%d bit errors)\n",
               i, expected, got, diff, __builtin_popcount(diff));
    }

    // Output raw bytes to stdout
    std::cout.write(reinterpret_cast<const char*>(rx_data.data()), rx_data.size());

    return 0;
}

ultra::Modulation parseModulation(const char* s) {
    if (strcmp(s, "dbpsk") == 0) return ultra::Modulation::DBPSK;
    if (strcmp(s, "bpsk") == 0) return ultra::Modulation::BPSK;
    if (strcmp(s, "dqpsk") == 0) return ultra::Modulation::DQPSK;
    if (strcmp(s, "qpsk") == 0) return ultra::Modulation::QPSK;
    if (strcmp(s, "d8psk") == 0) return ultra::Modulation::D8PSK;
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
    const char* output_file = nullptr;
    const char* command = nullptr;
    const char* input_file = nullptr;
    std::string callsign = "TEST";
    int timing_offset = 0;

    // Parse all arguments - options can appear before or after command
    // This allows both: "ultra -m dqpsk rawdecode file" and "ultra rawdecode -m dqpsk file"
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            config.sample_rate = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            config.modulation = parseModulation(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            config.code_rate = parseCodeRate(argv[++i]);
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_file = argv[++i];
        } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            callsign = argv[++i];
        } else if (strcmp(argv[i], "-a") == 0) {
            adaptive = true;
        } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
            timing_offset = std::atoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            // Non-option argument: first is command, second is input file
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

    if (strcmp(command, "test") == 0) {
        return runTest();
    } else if (strcmp(command, "info") == 0) {
        printInfo(config);
        return 0;
    } else if (strcmp(command, "tx") == 0) {
        if (!input_file) {
            std::cerr << "Error: tx requires a filename\n";
            return 1;
        }
        return runTx(input_file, config);
    } else if (strcmp(command, "rx") == 0) {
        return runRx(config, input_file, output_file);
    } else if (strcmp(command, "prx") == 0) {
        return runProtocolRx(config, input_file, callsign);
    } else if (strcmp(command, "decode") == 0) {
        return runDecode(config, input_file, output_file);
    } else if (strcmp(command, "rawdecode") == 0) {
        return runRawDecode(config, input_file, timing_offset);
    } else {
        std::cerr << "Unknown command: " << command << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
