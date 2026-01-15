/**
 * ITU-R F.1487 HF Channel Benchmark Suite
 *
 * Measures SNR thresholds for BER=10^-3 and BER=10^-5 across all modes
 * under standardized ITU-R F.1487 Watterson channel conditions.
 *
 * CRITICAL: This benchmark uses the ACTUAL modem pipeline:
 *   TX: Data → LDPC Encode → Interleave → OFDM Modulate → Audio
 *   Channel: Audio → WattersonChannel → Audio
 *   RX: Audio → OFDM Demodulate → Deinterleave → LDPC Decode → Data
 *
 * NO simplified models. NO scaling factors. REAL system performance.
 *
 * Reference: ITU-R Recommendation F.1487-0 (2000)
 * "Testing of HF modems with bandwidths of up to about 12 kHz using
 * ionospheric channel simulators"
 */

#include "../src/sim/hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>

using namespace ultra;
using namespace ultra::sim;

// LDPC block size
static constexpr size_t LDPC_BLOCK_SIZE = 648;

// Get max data bytes for a code rate
static size_t getMaxDataBytes(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 20;
        case CodeRate::R1_2: return 40;
        case CodeRate::R2_3: return 54;
        case CodeRate::R3_4: return 60;
        case CodeRate::R5_6: return 67;
        default: return 40;
    }
}

// Count bit errors
static size_t countBitErrors(const Bytes& a, const Bytes& b) {
    size_t errors = 0;
    size_t len = std::min(a.size(), b.size());
    for (size_t i = 0; i < len; ++i) {
        uint8_t diff = a[i] ^ b[i];
        while (diff) {
            errors += diff & 1;
            diff >>= 1;
        }
    }
    if (a.size() > b.size()) errors += (a.size() - b.size()) * 8;
    else if (b.size() > a.size()) errors += (b.size() - a.size()) * 8;
    return errors;
}

// Modulation/CodeRate mode description
struct ModeConfig {
    Modulation mod;
    CodeRate rate;
    const char* name;
    float throughput_kbps;  // Theoretical throughput
};

// Result for a single benchmark run
struct BenchmarkResult {
    std::string mode_name;
    std::string channel_name;
    float snr_for_ber_1e3;    // SNR to achieve BER < 10^-3 (or -1 if failed)
    float snr_for_ber_1e5;    // SNR to achieve BER < 10^-5 (or -1 if failed)
    float throughput_kbps;
    size_t frames_tested;
};

/**
 * Measure BER at a specific SNR using the ACTUAL modem pipeline.
 *
 * This runs the REAL system:
 *   TX: Data → LDPC Encode → Interleave → OFDM Modulate → Audio
 *   Channel: Audio → WattersonChannel → Audio
 *   RX: Audio → OFDM Demodulate → Deinterleave → LDPC Decode → Data
 */
// Measure Frame Success Rate (more meaningful for ARQ systems)
float measureFSR(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    WattersonChannel::Config channel_cfg,
    size_t num_frames
) {
    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(24, 27);

    WattersonChannel channel(channel_cfg);

    size_t max_data_bytes = getMaxDataBytes(rate);
    size_t successful_frames = 0;

    for (size_t frame = 0; frame < num_frames; ++frame) {
        OFDMDemodulator demodulator(config);  // Fresh demod per frame

        Bytes tx_data(max_data_bytes);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }

        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, mod);

        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();
            if (soft_bits.size() >= LDPC_BLOCK_SIZE) {
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    rx_data.resize(tx_data.size());
                    if (rx_data == tx_data) {
                        successful_frames++;
                    }
                }
            }
        }
    }

    return static_cast<float>(successful_frames) / static_cast<float>(num_frames);
}

float measureBER(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    WattersonChannel::Config channel_cfg,
    size_t num_frames
) {
    // Create REAL modem components
    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(24, 27);  // 648 bits = 24 x 27

    // Create REAL channel
    WattersonChannel channel(channel_cfg);

    size_t max_data_bytes = getMaxDataBytes(rate);
    size_t total_bits = 0;
    size_t total_errors = 0;

    for (size_t frame = 0; frame < num_frames; ++frame) {
        // Fresh demodulator per frame (no state bleed between frames)
        OFDMDemodulator demodulator(config);

        // Generate test data
        Bytes tx_data(max_data_bytes);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }

        // === TX Chain (REAL) ===
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        // Generate preamble + modulated data
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, mod);

        // Combine preamble + data
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale to reasonable audio level
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // === Channel (REAL Watterson) ===
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // === RX Chain (REAL) ===
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();

            if (soft_bits.size() >= LDPC_BLOCK_SIZE) {
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    if (rx_data.size() > tx_data.size()) {
                        rx_data.resize(tx_data.size());
                    }
                    total_errors += countBitErrors(tx_data, rx_data);
                } else {
                    // Failed decode = all bits wrong
                    total_errors += tx_data.size() * 8;
                }
            } else {
                // Not enough soft bits = failed
                total_errors += tx_data.size() * 8;
            }
        } else {
            // No sync = failed
            total_errors += tx_data.size() * 8;
        }
        total_bits += tx_data.size() * 8;
    }

    return static_cast<float>(total_errors) / static_cast<float>(total_bits);
}

/**
 * Binary search to find SNR threshold for target BER.
 * Returns -1 if target BER cannot be achieved even at max SNR.
 */
float findSNRThreshold(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    WattersonChannel::Config channel_cfg,
    float target_ber,
    size_t frames_per_point = 50
) {
    float snr_low = 0.0f;
    float snr_high = 45.0f;

    // First check if achievable at max SNR
    channel_cfg.snr_db = snr_high;
    float ber_at_max = measureBER(config, mod, rate, channel_cfg, frames_per_point);
    if (ber_at_max > target_ber) {
        return -1.0f;  // Cannot achieve target
    }

    // Binary search
    while (snr_high - snr_low > 1.0f) {  // 1 dB resolution
        float snr_mid = (snr_low + snr_high) / 2.0f;
        channel_cfg.snr_db = snr_mid;
        float ber = measureBER(config, mod, rate, channel_cfg, frames_per_point);

        if (ber < target_ber) {
            snr_high = snr_mid;
        } else {
            snr_low = snr_mid;
        }
    }

    return snr_high;
}

// Get code rate as float
static float codeRateToFloat(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 0.25f;
        case CodeRate::R1_3: return 0.333f;
        case CodeRate::R1_2: return 0.5f;
        case CodeRate::R2_3: return 0.667f;
        case CodeRate::R3_4: return 0.75f;
        case CodeRate::R5_6: return 0.833f;
        case CodeRate::R7_8: return 0.875f;
        default: return 0.5f;
    }
}

// Calculate theoretical throughput
static float calcThroughput(const ModemConfig& config, Modulation mod, CodeRate rate) {
    size_t bits_per_carrier = static_cast<size_t>(mod);
    size_t data_carriers = config.num_carriers - config.num_carriers / config.pilot_spacing;
    float symbol_duration = static_cast<float>(config.getSymbolDuration());
    float symbol_rate = static_cast<float>(config.sample_rate) / symbol_duration;
    return data_carriers * bits_per_carrier * symbol_rate * codeRateToFloat(rate) / 1000.0f;
}

void printBanner() {
    std::cout << "\n";
    std::cout << "================================================================================\n";
    std::cout << "           ProjectUltra ITU-R F.1487 HF Channel Benchmark Suite\n";
    std::cout << "================================================================================\n";
    std::cout << "\n";
    std::cout << "CRITICAL: This benchmark uses the ACTUAL modem pipeline - not a simplified model.\n";
    std::cout << "  TX: Data -> LDPC Encode -> OFDM Modulate -> Audio\n";
    std::cout << "  Channel: Audio -> WattersonChannel -> Audio\n";
    std::cout << "  RX: Audio -> OFDM Demodulate -> LDPC Decode -> Data\n";
    std::cout << "\n";
    std::cout << "Reference: ITU-R Recommendation F.1487-0 (2000)\n";
    std::cout << "Channel Model: 2-path Rayleigh fading, Gaussian Doppler spectrum\n";
    std::cout << "\n";
    std::cout << "Channel Conditions (ITU-R F.1487 Standard):\n";
    std::cout << "  AWGN:     No fading, no multipath (baseline)\n";
    std::cout << "  Good:     0.5 ms delay, 0.1 Hz Doppler (quiet mid-latitude)\n";
    std::cout << "  Moderate: 1.0 ms delay, 0.5 Hz Doppler (typical daytime)\n";
    std::cout << "  Poor:     2.0 ms delay, 1.0 Hz Doppler (disturbed/high-latitude)\n";
    std::cout << "  Flutter:  0.5 ms delay, 10 Hz Doppler (auroral/polar)\n";
    std::cout << "\n";
    std::cout << "Metrics: SNR threshold for BER < 10^-3 and BER < 10^-5\n";
    std::cout << "================================================================================\n\n";
}

void runQuickBenchmark(ModemConfig config) {
    std::cout << "Quick benchmark (QPSK R1/2, 100 frames/channel at 25 dB SNR)...\n";
    std::cout << "Frame Success Rate (FSR) = correctly decoded frames / total frames\n";
    std::cout << "With ARQ, FSR>80% means reliable delivery (just needs occasional retries)\n\n";

    std::cout << std::setw(12) << "Channel"
              << std::setw(10) << "Pilots"
              << std::setw(12) << "FSR@25dB"
              << std::setw(14) << "Effective"
              << "\n";
    std::cout << std::string(48, '-') << "\n";

    // Channel conditions with appropriate pilot spacing
    // Pilot spacing must be < coherence_bandwidth / carrier_spacing
    // Carrier spacing = 93.75 Hz, coherence BW ≈ 1 / (2π × delay_spread)
    struct {
        const char* name;
        WattersonChannel::Config (*getConfig)(float);
        int pilot_spacing;  // Optimized for channel's coherence bandwidth
    } channels[] = {
        {"AWGN",     itu_r_f1487::awgn,     6},  // No frequency selectivity
        {"Good",     itu_r_f1487::good,     4},  // 0.5ms delay → ~320 Hz coherence BW
        {"Moderate", itu_r_f1487::moderate, 2},  // 1.0ms delay → ~160 Hz coherence BW
        {"Poor",     itu_r_f1487::poor,     2},  // 2.0ms delay → ~80 Hz - challenging!
        {"Flutter",  itu_r_f1487::flutter,  4},  // 0.5ms delay, fast Doppler - needs time tracking
    };

    for (const auto& ch : channels) {
        std::cout << std::setw(12) << ch.name << std::flush;

        // Set appropriate pilot spacing for this channel
        config.pilot_spacing = ch.pilot_spacing;
        auto cfg = ch.getConfig(25.0f);  // Test at 25 dB SNR

        // Measure Frame Success Rate
        float fsr = measureFSR(config, Modulation::QPSK, CodeRate::R1_2, cfg, 100);

        std::cout << std::setw(10) << ("1/" + std::to_string(ch.pilot_spacing));
        std::cout << std::setw(11) << std::fixed << std::setprecision(0) << (fsr * 100) << "%";

        // Rate effectiveness
        if (fsr >= 0.95f) {
            std::cout << std::setw(14) << "EXCELLENT";
        } else if (fsr >= 0.80f) {
            std::cout << std::setw(14) << "GOOD (ARQ)";
        } else if (fsr >= 0.50f) {
            std::cout << std::setw(14) << "MARGINAL";
        } else {
            std::cout << std::setw(14) << "POOR";
        }
        std::cout << "\n";
    }

    std::cout << "\nNote: This modem uses ARQ for reliable delivery. FSR>80% is usable.\n";
    std::cout << "For challenging channels (Poor/Flutter), lower data rates recommended.\n";
}

void runFullBenchmark(const ModemConfig& config, bool verbose, const std::string& csv_output) {
    // Define modes to test
    std::vector<ModeConfig> modes = {
        {Modulation::BPSK,   CodeRate::R1_4, "BPSK R1/4",   0},
        {Modulation::BPSK,   CodeRate::R1_2, "BPSK R1/2",   0},
        {Modulation::QPSK,   CodeRate::R1_2, "QPSK R1/2",   0},
        {Modulation::QPSK,   CodeRate::R3_4, "QPSK R3/4",   0},
        {Modulation::QAM16,  CodeRate::R1_2, "16QAM R1/2",  0},
        {Modulation::QAM16,  CodeRate::R3_4, "16QAM R3/4",  0},
        {Modulation::QAM64,  CodeRate::R1_2, "64QAM R1/2",  0},
        {Modulation::QAM64,  CodeRate::R3_4, "64QAM R3/4",  0},
    };

    // Calculate throughputs
    for (auto& m : modes) {
        m.throughput_kbps = calcThroughput(config, m.mod, m.rate);
    }

    // Define channel conditions with appropriate pilot spacing
    // Pilot spacing must be < coherence_bandwidth / carrier_spacing
    // Carrier spacing = 93.75 Hz, coherence BW ≈ 1 / (2π × delay_spread)
    struct ChannelCondition {
        const char* name;
        WattersonChannel::Config (*getConfig)(float);
        int pilot_spacing;  // Optimized for channel's coherence bandwidth
    };

    std::vector<ChannelCondition> channels = {
        {"AWGN",     itu_r_f1487::awgn,     6},  // No frequency selectivity
        {"Good",     itu_r_f1487::good,     4},  // 0.5ms delay → ~320 Hz coherence BW
        {"Moderate", itu_r_f1487::moderate, 2},  // 1.0ms delay → ~160 Hz coherence BW
        {"Poor",     itu_r_f1487::poor,     2},  // 2.0ms delay → ~80 Hz - challenging!
        {"Flutter",  itu_r_f1487::flutter,  4},  // 0.5ms delay, fast Doppler
    };

    // Results storage
    std::vector<BenchmarkResult> results;

    std::cout << "Running full benchmark with REAL modem pipeline...\n";
    std::cout << "This will take several minutes.\n\n";

    auto start_time = std::chrono::steady_clock::now();

    // Run benchmarks
    for (const auto& mode : modes) {
        std::cout << "Mode: " << mode.name << " (" << std::fixed << std::setprecision(1)
                  << mode.throughput_kbps << " kbps)\n";
        std::cout << std::setw(12) << "Channel"
                  << std::setw(16) << "SNR@BER<1e-3"
                  << std::setw(16) << "SNR@BER<1e-5"
                  << "\n";
        std::cout << std::string(44, '-') << "\n";

        for (const auto& channel : channels) {
            BenchmarkResult result;
            result.mode_name = mode.name;
            result.channel_name = channel.name;
            result.throughput_kbps = mode.throughput_kbps;

            // Use per-channel optimized pilot spacing
            ModemConfig channel_config = config;
            channel_config.pilot_spacing = channel.pilot_spacing;

            auto base_cfg = channel.getConfig(20.0f);

            std::cout << std::setw(12) << channel.name << std::flush;

            // Find SNR for BER < 10^-3
            result.snr_for_ber_1e3 = findSNRThreshold(
                channel_config, mode.mod, mode.rate, base_cfg, 1e-3f, 30
            );

            if (result.snr_for_ber_1e3 >= 0) {
                std::cout << std::setw(12) << std::fixed << std::setprecision(1)
                          << result.snr_for_ber_1e3 << " dB";
            } else {
                std::cout << std::setw(16) << "FAILED";
            }

            // Find SNR for BER < 10^-5
            result.snr_for_ber_1e5 = findSNRThreshold(
                channel_config, mode.mod, mode.rate, base_cfg, 1e-5f, 50
            );

            if (result.snr_for_ber_1e5 >= 0) {
                std::cout << std::setw(12) << std::fixed << std::setprecision(1)
                          << result.snr_for_ber_1e5 << " dB";
            } else {
                std::cout << std::setw(16) << "FAILED";
            }
            std::cout << "\n";

            results.push_back(result);
        }
        std::cout << "\n";
    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);

    std::cout << "================================================================================\n";
    std::cout << "Benchmark completed in " << duration.count() << " seconds.\n";
    std::cout << "================================================================================\n\n";

    // Print summary
    std::cout << "SUMMARY: SNR Thresholds for BER < 10^-5 (dB)\n";
    std::cout << "================================================================================\n";
    std::cout << std::setw(14) << "Mode"
              << std::setw(8) << "kbps"
              << std::setw(8) << "AWGN"
              << std::setw(8) << "Good"
              << std::setw(8) << "Mod"
              << std::setw(8) << "Poor"
              << std::setw(10) << "Flutter"
              << "\n";
    std::cout << std::string(64, '-') << "\n";

    for (const auto& mode : modes) {
        std::cout << std::setw(14) << mode.name
                  << std::setw(8) << std::fixed << std::setprecision(1) << mode.throughput_kbps;

        for (const auto& channel : channels) {
            for (const auto& r : results) {
                if (r.mode_name == mode.name && r.channel_name == channel.name) {
                    if (r.snr_for_ber_1e5 >= 0) {
                        std::cout << std::setw(8) << std::fixed << std::setprecision(1)
                                  << r.snr_for_ber_1e5;
                    } else {
                        std::cout << std::setw(8) << "-";
                    }
                    break;
                }
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";

    std::cout << "Note: These results are from the ACTUAL modem pipeline.\n";
    std::cout << "================================================================================\n";

    // Write CSV if requested
    if (!csv_output.empty()) {
        std::ofstream csv(csv_output);
        if (csv.is_open()) {
            csv << "Mode,Throughput_kbps,Channel,SNR_BER_1e3,SNR_BER_1e5\n";
            for (const auto& r : results) {
                csv << r.mode_name << ","
                    << r.throughput_kbps << ","
                    << r.channel_name << ","
                    << (r.snr_for_ber_1e3 >= 0 ? std::to_string(r.snr_for_ber_1e3) : "FAILED") << ","
                    << (r.snr_for_ber_1e5 >= 0 ? std::to_string(r.snr_for_ber_1e5) : "FAILED")
                    << "\n";
            }
            csv.close();
            std::cout << "\nResults written to: " << csv_output << "\n";
        }
    }
}

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n";
    std::cout << "\n";
    std::cout << "Options:\n";
    std::cout << "  --quick        Run quick benchmark (QPSK R1/2 only)\n";
    std::cout << "  --full         Run full benchmark (all modes) - takes several minutes\n";
    std::cout << "  --csv FILE     Write results to CSV file\n";
    std::cout << "  --verbose      Show progress during benchmark\n";
    std::cout << "  --help         Show this help\n";
    std::cout << "\n";
    std::cout << "Default: --quick\n";
    std::cout << "\n";
    std::cout << "IMPORTANT: This benchmark uses the ACTUAL modem pipeline.\n";
    std::cout << "Results are real system performance, not simplified simulations.\n";
}

int main(int argc, char* argv[]) {
    // Parse arguments
    bool quick_mode = true;
    bool verbose = false;
    std::string csv_output;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--quick") {
            quick_mode = true;
        } else if (arg == "--full") {
            quick_mode = false;
        } else if (arg == "--verbose") {
            verbose = true;
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_output = argv[++i];
        } else if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

    // Standard modem config
    ModemConfig config;
    config.sample_rate = 48000;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 6;
    config.cp_mode = CyclicPrefixMode::MEDIUM;  // 48 samples
    config.center_freq = 1500;

    // Enable adaptive equalizer for fading channels
    config.adaptive_eq_enabled = true;
    config.adaptive_eq_use_rls = false;  // Use LMS (more stable)
    config.lms_mu = 0.1f;  // Aggressive step size for fast tracking
    config.decision_directed = true;

    // Lower sync threshold for fading channels (multipath/fading can reduce
    // preamble correlation from 1.0 to ~0.75-0.85)
    config.sync_threshold = 0.70f;

    printBanner();

    if (quick_mode) {
        runQuickBenchmark(config);
    } else {
        runFullBenchmark(config, verbose, csv_output);
    }

    return 0;
}
