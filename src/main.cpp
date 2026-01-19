#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>

#include "ultra/modem.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"
#include "protocol/protocol_engine.hpp"
#include "protocol/frame_v2.hpp"

#include <iostream>
#include <fstream>
#include <cstring>
#include <csignal>
#include <atomic>
#include <map>

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
    std::cerr << "                    ptx           -> PROBE (no message)\n";
    std::cerr << "                    ptx connect   -> CONNECT (with callsigns)\n";
    std::cerr << "                    ptx \"Hello\"   -> DATA (with message)\n";
    std::cerr << "  prx [file]      Protocol RX - decode v2 frames (from file or stdin)\n";
    std::cerr << "  tx <file>       Transmit file (raw modem, outputs audio)\n";
    std::cerr << "  rx [file]       Receive (raw modem, from file or stdin)\n";
    std::cerr << "  decode [file]   Raw LDPC decode (no protocol framing)\n";
    std::cerr << "  test            Run self-test\n";
    std::cerr << "  info            Show modem capabilities\n";
    std::cerr << "\nOptions:\n";
    std::cerr << "  -s <call>       Source callsign (default: TEST)\n";
    std::cerr << "  -d <call>       Destination callsign (default: CQ)\n";
    std::cerr << "  -o <file>       Output to file instead of stdout\n";
    std::cerr << "  -r <rate>       Sample rate (default: 48000)\n";
    std::cerr << "  -m <mod>        Modulation: dbpsk, dqpsk, qpsk, qam16, qam64\n";
    std::cerr << "  -c <rate>       Code rate: 1/4, 1/2, 2/3, 3/4, 5/6\n";
    std::cerr << "\nExamples:\n";
    std::cerr << "  # Send PROBE and play audio (Linux)\n";
    std::cerr << "  " << prog << " ptx -s MYCALL -d THEIRCALL | aplay -f FLOAT_LE -r 48000\n\n";
    std::cerr << "  # Send CONNECT frame to file\n";
    std::cerr << "  " << prog << " ptx connect -s MYCALL -d THEIRCALL -o connect.f32\n\n";
    std::cerr << "  # Send DATA message to file\n";
    std::cerr << "  " << prog << " ptx \"Hello!\" -s MYCALL -o msg.f32\n\n";
    std::cerr << "  # Listen for frames from audio recording\n";
    std::cerr << "  " << prog << " prx recording.f32 -s MYCALL\n\n";
    std::cerr << "  # Live RX from microphone (Linux)\n";
    std::cerr << "  arecord -f FLOAT_LE -r 48000 | " << prog << " prx -s MYCALL\n";
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

// Protocol TX - transmit v2 frame (PROBE, CONNECT, or DATA with text message)
// Usage: ultra ptx "Hello World" -o output.f32 -s MYCALL
//        ultra ptx connect -s MYCALL -d THEIRCALL -o connect.f32
int runProtocolTx(ultra::ModemConfig& config, const char* message, const char* output_file,
                  const std::string& src_call, const std::string& dst_call) {
    namespace v2 = ultra::protocol::v2;

    std::cerr << "Protocol TX v2 mode\n";
    std::cerr << "  From: " << src_call << " To: " << dst_call << "\n";

    // Create the frame
    ultra::Bytes frame_data;
    std::string frame_type_str;

    if (!message || strlen(message) == 0) {
        // No message = send PROBE
        auto probe = v2::ControlFrame::makeProbe(src_call, dst_call);
        frame_data = probe.serialize();
        frame_type_str = "PROBE";
    } else if (strcmp(message, "connect") == 0) {
        // "connect" keyword = send CONNECT frame with full callsigns
        // Mode caps: support all modes (0xFF), preferred: DQPSK R1/4 (0x00)
        auto connect = v2::ConnectFrame::makeConnect(src_call, dst_call, 0xFF, 0x00);
        frame_data = connect.serialize();
        frame_type_str = "CONNECT";
    } else {
        // Message = send DATA frame
        auto data = v2::DataFrame::makeData(src_call, dst_call, 1, std::string(message));
        frame_data = data.serialize();
        frame_type_str = "DATA";
    }

    std::cerr << "  Frame type: " << frame_type_str << "\n";
    std::cerr << "  Frame size: " << frame_data.size() << " bytes\n";

    // LDPC encode with v2 function (splits into codewords)
    auto encoded_cws = v2::encodeFrameWithLDPC(frame_data);
    std::cerr << "  LDPC codewords: " << encoded_cws.size() << "\n";

    // Concatenate all encoded codewords
    ultra::Bytes all_encoded;
    for (const auto& cw : encoded_cws) {
        all_encoded.insert(all_encoded.end(), cw.begin(), cw.end());
    }
    std::cerr << "  Total encoded: " << all_encoded.size() << " bytes\n";

    // Create modulator (DQPSK for robust link establishment)
    config.modulation = ultra::Modulation::DQPSK;
    config.code_rate = ultra::CodeRate::R1_4;
    ultra::OFDMModulator mod(config);

    // Generate preamble + modulated data
    auto preamble = mod.generatePreamble();
    auto modulated = mod.modulate(all_encoded, config.modulation);

    // Combine: preamble + data + tail guard
    // Tail guard ensures demodulator processes the last OFDM symbol
    const size_t TAIL_SAMPLES = 576 * 2;  // 2 OFDM symbols of silence
    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size() + TAIL_SAMPLES);
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());
    output.resize(output.size() + TAIL_SAMPLES, 0.0f);  // Add silence tail

    // Normalize
    float max_val = 0;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.8f / max_val;
        for (float& s : output) s *= scale;
    }

    std::cerr << "  Audio samples: " << output.size() << " ("
              << (output.size() / 48000.0) << " sec)\n";

    // Write to file
    if (output_file) {
        std::ofstream outfile(output_file, std::ios::binary);
        if (!outfile) {
            std::cerr << "Error: Cannot open output file: " << output_file << "\n";
            return 1;
        }
        outfile.write(reinterpret_cast<const char*>(output.data()),
                      output.size() * sizeof(float));
        std::cerr << "  Written to: " << output_file << "\n";
    } else {
        // Write to stdout
        std::cout.write(reinterpret_cast<const char*>(output.data()),
                        output.size() * sizeof(float));
    }

    return 0;
}

// Protocol RX mode - decodes v2 frames with 2-byte magic "UL"
int runProtocolRx(ultra::ModemConfig& config, const char* input_file, const std::string& callsign, int timing_offset = 0) {
    namespace v2 = ultra::protocol::v2;

    std::cerr << "Protocol RX v2 mode (looking for UL frames)...\n";
    if (input_file) std::cerr << "Input: " << input_file << "\n";
    if (timing_offset != 0) std::cerr << "Timing offset: " << timing_offset << " samples\n";
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
    }

    // Create demodulator and decoder (DQPSK R1/4 for link frames)
    config.modulation = ultra::Modulation::DQPSK;
    config.code_rate = ultra::CodeRate::R1_4;
    ultra::OFDMDemodulator demod(config);
    demod.setTimingOffset(timing_offset);  // Apply timing adjustment
    ultra::LDPCDecoder decoder(config.code_rate);

    // Track received frames
    int frames_received = 0;
    int control_frames = 0;
    int data_frames = 0;

    // Accumulate soft bits
    std::vector<float> accumulated_soft_bits;
    const size_t LDPC_BLOCK_SIZE = v2::LDPC_CODEWORD_BITS;  // 648 bits

    // For multi-codeword frames
    v2::CodewordStatus current_frame;
    int expected_codewords = 0;
    int current_cw_index = 0;

    // Buffer for data codewords that arrive before header
    // Key: codeword index, Value: decoded data
    std::map<uint8_t, ultra::Bytes> buffered_data_cws;

    // Track consecutive failures
    int consecutive_failures = 0;
    const int MAX_FAILURES = 5;

    // Decode-centric synchronization state
    // Instead of relying on perfect preamble timing, we hunt for valid codewords
    // by trying LDPC decode at various bit offsets until one succeeds with valid magic
    enum class CwSyncState { HUNTING, SYNCED };
    CwSyncState cw_sync_state = CwSyncState::HUNTING;
    const size_t HUNT_STEP = 30;  // Try every 30 bits (one OFDM symbol worth)
    const size_t MAX_HUNT_OFFSET = LDPC_BLOCK_SIZE * 2;  // Search up to two codeword lengths
    size_t hunt_attempts = 0;

    // Read audio in chunks
    std::vector<float> buffer(960);

    while (g_running && input->read(reinterpret_cast<char*>(buffer.data()),
                                     buffer.size() * sizeof(float))) {
        ultra::SampleSpan span(buffer.data(), buffer.size());
        demod.process(span);

        if (demod.isSynced()) {
            // Get soft bits produced by this chunk
            auto soft_bits = demod.getSoftBits();
            if (!soft_bits.empty()) {
                accumulated_soft_bits.insert(accumulated_soft_bits.end(),
                                             soft_bits.begin(), soft_bits.end());
                // Debug: track soft bit accumulation
                static size_t total_soft_bits = 0;
                total_soft_bits += soft_bits.size();
                if (soft_bits.size() > 100 || accumulated_soft_bits.size() > 500) {
                    std::cerr << "  [BITS] +" << soft_bits.size() << " total_accum=" << accumulated_soft_bits.size() << "\n";
                }
            }

            // === HUNTING STATE: Try to find codeword boundary ===
            if (cw_sync_state == CwSyncState::HUNTING) {
                // Try LDPC decode at various offsets to find a valid codeword
                size_t hunt_this_round = 0;
                for (size_t offset = 0;
                     offset < MAX_HUNT_OFFSET &&
                     accumulated_soft_bits.size() >= offset + LDPC_BLOCK_SIZE;
                     offset += HUNT_STEP) {

                    std::vector<float> block(accumulated_soft_bits.begin() + offset,
                                             accumulated_soft_bits.begin() + offset + LDPC_BLOCK_SIZE);

                    auto decoded = decoder.decodeSoft(std::span<const float>(block));
                    hunt_attempts++;
                    hunt_this_round++;

                    if (decoder.lastDecodeSuccess()) {
                        // LDPC decoded - check if it's a valid codeword (has magic)
                        ultra::Bytes cw_data(decoded.begin(),
                                             decoded.begin() + std::min(decoded.size(), size_t(v2::BYTES_PER_CODEWORD)));
                        auto cw_info = v2::identifyCodeword(cw_data);

                        if (cw_info.type != v2::CodewordType::UNKNOWN) {
                            // Found valid codeword! We're now synced to codeword boundaries
                            std::cerr << "  [HUNT SUCCESS] Found valid CW at offset " << offset
                                      << " after " << hunt_attempts << " attempts"
                                      << " (type=" << (cw_info.type == v2::CodewordType::HEADER ? "HEADER" : "DATA")
                                      << ")\n";

                            // Discard bits before this codeword
                            accumulated_soft_bits.erase(accumulated_soft_bits.begin(),
                                                        accumulated_soft_bits.begin() + offset);

                            cw_sync_state = CwSyncState::SYNCED;
                            consecutive_failures = 0;
                            hunt_attempts = 0;
                            break;  // Exit hunt loop, will process in SYNCED state
                        }
                    }
                }

                // Debug: show hunting activity
                if (hunt_this_round > 0 || accumulated_soft_bits.size() >= LDPC_BLOCK_SIZE) {
                    std::cerr << "  [HUNTING] tried " << hunt_this_round << " decodes, accum="
                              << accumulated_soft_bits.size() << " bits\n";
                }

                // Prevent unbounded buffer growth during hunting
                if (cw_sync_state == CwSyncState::HUNTING &&
                    accumulated_soft_bits.size() > LDPC_BLOCK_SIZE * 4) {
                    // Discard one step worth of bits and keep hunting
                    accumulated_soft_bits.erase(accumulated_soft_bits.begin(),
                                                accumulated_soft_bits.begin() + HUNT_STEP);
                }

                continue;  // Keep hunting or processing next chunk
            }

            // === SYNCED STATE: Decode at fixed codeword boundaries ===
            while (accumulated_soft_bits.size() >= LDPC_BLOCK_SIZE) {
                std::vector<float> block(accumulated_soft_bits.begin(),
                                         accumulated_soft_bits.begin() + LDPC_BLOCK_SIZE);
                accumulated_soft_bits.erase(accumulated_soft_bits.begin(),
                                            accumulated_soft_bits.begin() + LDPC_BLOCK_SIZE);

                // LDPC decode this codeword
                auto decoded = decoder.decodeSoft(std::span<const float>(block));

                if (!decoder.lastDecodeSuccess()) {
                    consecutive_failures++;
                    std::cerr << "  [CW] LDPC decode failed\n";

                    if (consecutive_failures >= MAX_FAILURES) {
                        std::cerr << "  [LOST SYNC] Too many failures, resetting for next preamble\n";
                        cw_sync_state = CwSyncState::HUNTING;
                        consecutive_failures = 0;
                        expected_codewords = 0;
                        current_cw_index = 0;
                        buffered_data_cws.clear();
                        accumulated_soft_bits.clear();  // Clear stale soft bits
                        demod.reset();  // Reset OFDM demod to look for new preamble
                    }
                    continue;
                }

                consecutive_failures = 0;

                // Trim to 20 bytes (info portion of codeword)
                ultra::Bytes cw_data(decoded.begin(),
                                     decoded.begin() + std::min(decoded.size(), size_t(v2::BYTES_PER_CODEWORD)));

                // Identify codeword type using marker bytes
                auto cw_info = v2::identifyCodeword(cw_data);

                // Is this the first codeword of a frame (header)?
                if (expected_codewords == 0) {
                    if (cw_info.type == v2::CodewordType::HEADER) {
                        // Parse header
                        auto header = v2::parseHeader(cw_data);
                        if (header.valid) {
                            expected_codewords = header.total_cw;
                            current_frame.initForFrame(expected_codewords);
                            current_frame.decoded[0] = true;
                            current_frame.data[0] = cw_data;
                            current_cw_index = 1;

                            std::cerr << "  [FRAME START] type=" << v2::frameTypeToString(header.type)
                                      << " codewords=" << (int)header.total_cw
                                      << " seq=" << header.seq << "\n";

                            // Merge any buffered data codewords that arrived before header
                            if (!buffered_data_cws.empty()) {
                                std::cerr << "  [MERGE] " << buffered_data_cws.size() << " buffered data CW(s)\n";
                                for (auto& [idx, data] : buffered_data_cws) {
                                    if (idx < expected_codewords) {
                                        current_frame.decoded[idx] = true;
                                        current_frame.data[idx] = data;
                                        std::cerr << "    CW" << (int)idx << " merged from buffer\n";
                                    }
                                }
                                buffered_data_cws.clear();
                            }

                            // Update current_cw_index to first missing codeword
                            while (current_cw_index < expected_codewords &&
                                   current_frame.decoded[current_cw_index]) {
                                current_cw_index++;
                            }

                            // Single codeword frame (control frame)?
                            if (expected_codewords == 1) {
                                // Parse and display control frame
                                auto ctrl = v2::ControlFrame::deserialize(cw_data);
                                if (ctrl) {
                                    frames_received++;
                                    control_frames++;
                                    std::cerr << "  [CONTROL] " << v2::frameTypeToString(ctrl->type)
                                              << " src=0x" << std::hex << ctrl->src_hash
                                              << " dst=0x" << ctrl->dst_hash << std::dec
                                              << " seq=" << ctrl->seq << "\n";
                                }
                                expected_codewords = 0;
                                current_cw_index = 0;
                            } else if (current_frame.allSuccess()) {
                                // All codewords already received (from buffer)
                                auto reassembled = current_frame.reassemble();
                                auto data_frame = v2::DataFrame::deserialize(reassembled);

                                if (data_frame) {
                                    frames_received++;
                                    data_frames++;
                                    std::cerr << "  [DATA] seq=" << data_frame->seq
                                              << " payload=" << data_frame->payload_len << " bytes\n";
                                    std::cerr << "  [MESSAGE] \"" << data_frame->payloadAsText() << "\"\n";
                                }
                                expected_codewords = 0;
                                current_cw_index = 0;
                            }
                        } else {
                            std::cerr << "  [INVALID HEADER] magic OK but header CRC failed\n";
                        }
                    } else if (cw_info.type == v2::CodewordType::DATA) {
                        // Data codeword arrived before header - buffer it
                        uint8_t idx = cw_info.index;
                        buffered_data_cws[idx] = cw_data;
                        std::cerr << "  [BUFFER] Data CW index=" << (int)idx
                                  << " (waiting for header)\n";
                    } else {
                        // Unknown codeword - not a frame start
                        std::cerr << "  [UNKNOWN CW] first bytes: ";
                        for (size_t i = 0; i < std::min(cw_data.size(), size_t(4)); i++) {
                            fprintf(stderr, "%02X ", cw_data[i]);
                        }
                        std::cerr << "(expected 55 4C or D5 xx)\n";
                    }
                } else {
                    // Waiting for continuation codewords
                    // With new format, data CWs have marker + index, so we can verify
                    if (cw_info.type == v2::CodewordType::DATA) {
                        uint8_t idx = cw_info.index;
                        if (idx > 0 && idx < expected_codewords) {
                            current_frame.decoded[idx] = true;
                            current_frame.data[idx] = cw_data;
                            std::cerr << "  [CW " << (int)idx << "/" << expected_codewords << "] OK\n";
                        } else {
                            std::cerr << "  [CW] Unexpected index=" << (int)idx
                                      << " (expected 1-" << (expected_codewords-1) << ")\n";
                        }
                    } else if (cw_info.type == v2::CodewordType::HEADER) {
                        // New header while waiting for data CWs - frame was lost
                        std::cerr << "  [FRAME ABORT] New header received, previous frame incomplete\n";
                        // Start processing the new header
                        expected_codewords = 0;
                        current_cw_index = 0;
                        buffered_data_cws.clear();
                        // Re-process this codeword as a new frame start
                        auto header = v2::parseHeader(cw_data);
                        if (header.valid) {
                            expected_codewords = header.total_cw;
                            current_frame.initForFrame(expected_codewords);
                            current_frame.decoded[0] = true;
                            current_frame.data[0] = cw_data;
                            current_cw_index = 1;
                            std::cerr << "  [FRAME START] type=" << v2::frameTypeToString(header.type)
                                      << " codewords=" << (int)header.total_cw
                                      << " seq=" << header.seq << "\n";
                        }
                    } else {
                        // Unknown codeword type - maybe corrupted
                        std::cerr << "  [CW] Unknown type, bytes: ";
                        for (size_t i = 0; i < std::min(cw_data.size(), size_t(4)); i++) {
                            fprintf(stderr, "%02X ", cw_data[i]);
                        }
                        std::cerr << "\n";
                    }

                    // Check if all codewords received
                    if (current_frame.allSuccess()) {
                        auto reassembled = current_frame.reassemble();
                        auto data_frame = v2::DataFrame::deserialize(reassembled);

                        if (data_frame) {
                            frames_received++;
                            data_frames++;
                            std::cerr << "  [DATA] seq=" << data_frame->seq
                                      << " payload=" << data_frame->payload_len << " bytes\n";
                            std::cerr << "  [MESSAGE] \"" << data_frame->payloadAsText() << "\"\n";
                        } else {
                            std::cerr << "  [ERROR] Failed to parse reassembled data frame\n";
                        }

                        expected_codewords = 0;
                        current_cw_index = 0;
                    }
                }
            }
        }
    }

    // End of input (file ended or Ctrl+C)
    // In real operation, this only happens on shutdown - just report stats
    if (!accumulated_soft_bits.empty()) {
        std::cerr << "  [INFO] " << accumulated_soft_bits.size() << " bits discarded (incomplete frame)\n";
    }

    std::cerr << "\n=== Protocol RX v2 Statistics ===\n";
    std::cerr << "  Frames received: " << frames_received << "\n";
    std::cerr << "  Control frames:  " << control_frames << "\n";
    std::cerr << "  Data frames:     " << data_frames << "\n";

    return 0;
}

// Raw decode mode - bypasses protocol layer, outputs raw LDPC-decoded data
// Use this for decoding F3 test patterns from the GUI
int runDecode(ultra::ModemConfig& config, const char* input_file, const char* output_file, int timing_offset = 0) {
    std::cerr << "Raw decode mode (no protocol framing)...\n";
    if (input_file) std::cerr << "Input: " << input_file << "\n";
    if (timing_offset != 0) std::cerr << "Timing offset: " << timing_offset << " samples\n";
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
    demod.setTimingOffset(timing_offset);  // Apply timing adjustment
    ultra::LDPCDecoder decoder(config.code_rate);
    ultra::Interleaver interleaver(32, 32);  // Match TX interleaver

    int frames_decoded = 0;
    int total_bytes = 0;
    int chunks_read = 0;
    float max_level = 0;

    // Accumulate soft bits across chunks
    // CRITICAL: Must accumulate ALL codewords before decoding to handle bit-level boundaries
    // correctly. k=162 bits doesn't align to bytes (20.25 bytes), so decoding codeword-by-codeword
    // and concatenating bytes causes corruption at boundaries.
    std::vector<float> accumulated_soft_bits;
    const size_t LDPC_BLOCK_SIZE = 648;  // Bits per R1/4 codeword
    bool was_synced = false;

    // Helper lambda to decode accumulated soft bits
    auto try_decode_accumulated = [&]() {
        if (accumulated_soft_bits.size() < LDPC_BLOCK_SIZE) {
            if (!accumulated_soft_bits.empty()) {
                std::cerr << "Insufficient bits for decode: " << accumulated_soft_bits.size()
                          << " (need at least " << LDPC_BLOCK_SIZE << ")\n";
                accumulated_soft_bits.clear();
            }
            return;
        }

        size_t num_codewords = accumulated_soft_bits.size() / LDPC_BLOCK_SIZE;
        size_t bits_to_decode = num_codewords * LDPC_BLOCK_SIZE;

        std::cerr << "Decoding " << num_codewords << " codewords ("
                  << bits_to_decode << " bits) as single multi-codeword block...\n";

        // Trim to exact multiple of codeword size
        std::vector<float> decode_bits(accumulated_soft_bits.begin(),
                                        accumulated_soft_bits.begin() + bits_to_decode);
        accumulated_soft_bits.erase(accumulated_soft_bits.begin(),
                                    accumulated_soft_bits.begin() + bits_to_decode);

        // LLR normalization: Scale LLRs to target mean magnitude
        float sum_abs = 0.0f;
        for (float llr : decode_bits) {
            sum_abs += std::abs(llr);
        }
        float mean_abs = sum_abs / decode_bits.size();
        const float target_mean_abs = 3.5f;
        float scale_factor = 1.0f;
        if (mean_abs > 0.01f) {
            scale_factor = target_mean_abs / mean_abs;
            scale_factor = std::max(0.1f, std::min(10.0f, scale_factor));
            for (float& llr : decode_bits) {
                llr *= scale_factor;
            }
        }

        // Debug: show first 32 LLRs
        std::cerr << "LLRs (scaled " << scale_factor << "x): ";
        int pos=0, neg=0;
        for (size_t i = 0; i < std::min(decode_bits.size(), (size_t)32); i++) {
            fprintf(stderr, "%.1f ", decode_bits[i]);
        }
        for (float llr : decode_bits) {
            if (llr > 0) pos++; else if (llr < 0) neg++;
        }
        std::cerr << "... (" << pos << " pos, " << neg << " neg)\n";

        // Decode ALL codewords at once - this handles bit-level boundaries correctly
        ultra::Bytes decoded = decoder.decodeSoft(decode_bits);

        if (decoder.lastDecodeSuccess() && !decoded.empty()) {
            frames_decoded++;
            total_bytes += decoded.size();

            output->write(reinterpret_cast<const char*>(decoded.data()), decoded.size());
            output->flush();

            std::cerr << "Frame " << frames_decoded << ": ";
            for (size_t i = 0; i < std::min(decoded.size(), (size_t)32); i++) {
                fprintf(stderr, "%02X ", decoded[i]);
            }
            if (decoded.size() > 32) std::cerr << "...";
            std::cerr << " (" << decoded.size() << " bytes)\n";
        } else {
            std::cerr << "LDPC decode failed for " << num_codewords << " codewords\n";
        }
    };

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

        bool is_synced = demod.isSynced();

        if (is_synced) {
            auto soft_bits = demod.getSoftBits();
            if (!soft_bits.empty()) {
                accumulated_soft_bits.insert(accumulated_soft_bits.end(),
                                            soft_bits.begin(), soft_bits.end());
            }
        } else if (was_synced) {
            // Just lost sync - decode all accumulated bits from this frame
            std::cerr << "Lost sync after accumulating " << accumulated_soft_bits.size() << " bits\n";
            try_decode_accumulated();
        }

        was_synced = is_synced;
    }

    // End of file - try to decode any remaining accumulated bits
    if (!accumulated_soft_bits.empty()) {
        std::cerr << "End of file with " << accumulated_soft_bits.size() << " accumulated bits\n";
        try_decode_accumulated();
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
    std::string dst_callsign = "CQ";
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
        } else if (strcmp(argv[i], "-d") == 0 && i + 1 < argc) {
            dst_callsign = argv[++i];
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
        return runProtocolRx(config, input_file, callsign, timing_offset);
    } else if (strcmp(command, "ptx") == 0) {
        // input_file is actually the message for ptx
        return runProtocolTx(config, input_file, output_file, callsign, dst_callsign);
    } else if (strcmp(command, "decode") == 0) {
        return runDecode(config, input_file, output_file, timing_offset);
    } else if (strcmp(command, "rawdecode") == 0) {
        return runRawDecode(config, input_file, timing_offset);
    } else {
        std::cerr << "Unknown command: " << command << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
