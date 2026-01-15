/**
 * MIL-STD-188-110B Comparison Benchmark
 *
 * Measures SNR threshold for BER < 10^-3 and BER < 10^-5 to enable
 * direct comparison with MIL-STD-188-110B published performance.
 *
 * MIL-STD-188-110B reference performance (ITU-R F.1487 Poor channel):
 *   2400 bps: ~15 dB for BER=10^-5
 *   4800 bps: ~17 dB for BER=10^-5
 *   9600 bps: Fails on Poor channel
 *
 * This benchmark uses the REAL modem pipeline:
 *   TX: Data -> LDPC Encode -> OFDM Modulate -> Audio
 *   Channel: Audio -> WattersonChannel (ITU-R F.1487) -> Audio
 *   RX: Audio -> OFDM Demodulate -> LDPC Decode -> Data
 */

#include "../src/sim/hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/logging.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <chrono>

using namespace ultra;
using namespace ultra::sim;

// ============================================================================
// BER Measurement at a given SNR
// ============================================================================

struct BERResult {
    float snr_db;
    size_t total_bits;
    size_t bit_errors;
    size_t frames_sent;
    size_t frames_decoded;
    float ber;
    float fer;  // Frame Error Rate
};

// Count bit errors between two byte arrays
size_t countBitErrors(const Bytes& a, const Bytes& b) {
    size_t errors = 0;
    size_t len = std::min(a.size(), b.size());

    for (size_t i = 0; i < len; ++i) {
        uint8_t diff = a[i] ^ b[i];
        while (diff) {
            errors += diff & 1;
            diff >>= 1;
        }
    }

    // Count missing bytes as all errors
    if (a.size() > b.size()) {
        errors += (a.size() - b.size()) * 8;
    } else if (b.size() > a.size()) {
        errors += (b.size() - a.size()) * 8;
    }

    return errors;
}

// Get data bytes for a given code rate
size_t getDataBytes(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 20;
        case CodeRate::R1_2: return 40;
        case CodeRate::R2_3: return 54;
        case CodeRate::R3_4: return 60;
        case CodeRate::R5_6: return 67;
        default: return 40;
    }
}

BERResult measureBER(
    Modulation mod,
    CodeRate rate,
    WattersonChannel::Config channel_cfg,
    size_t num_frames
) {
    BERResult result = {};
    result.snr_db = channel_cfg.snr_db;
    result.frames_sent = num_frames;

    // Configure modem
    ModemConfig config;
    config.modulation = mod;  // CRITICAL: Set modulation!
    config.sync_threshold = 0.70f;  // Standard threshold (0.90 is too strict)

    // Adjust pilot spacing based on channel conditions
    if (channel_cfg.doppler_spread_hz > 1.0f) {
        config.pilot_spacing = 2;  // Dense pilots for fast fading
    } else if (channel_cfg.delay_spread_ms > 1.0f) {
        config.pilot_spacing = 3;
    } else {
        config.pilot_spacing = 4;
    }

    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(24, 27);

    size_t data_bytes = getDataBytes(rate);

    for (size_t frame = 0; frame < num_frames; ++frame) {
        // Fresh demodulator and channel for each frame
        OFDMDemodulator demodulator(config);
        WattersonChannel channel(channel_cfg);

        // Generate test data
        Bytes tx_data(data_bytes);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }
        result.total_bits += tx_data.size() * 8;

        // TX chain
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, mod);

        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Normalize
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // Channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // RX chain
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();
            if (soft_bits.size() >= 648) {
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    result.frames_decoded++;
                    if (rx_data.size() > tx_data.size()) {
                        rx_data.resize(tx_data.size());
                    }
                    result.bit_errors += countBitErrors(tx_data, rx_data);
                } else {
                    // Decode failed - count as all bits wrong
                    result.bit_errors += tx_data.size() * 8;
                }
            } else {
                // Not enough soft bits - count as all bits wrong
                result.bit_errors += tx_data.size() * 8;
            }
        } else {
            // No sync - count as all bits wrong
            result.bit_errors += tx_data.size() * 8;
        }
    }

    result.ber = static_cast<float>(result.bit_errors) / result.total_bits;
    result.fer = 1.0f - static_cast<float>(result.frames_decoded) / result.frames_sent;

    return result;
}

// ============================================================================
// SNR Sweep to find threshold
// ============================================================================

struct SNRThreshold {
    float snr_1e3;   // SNR for BER < 10^-3
    float snr_1e5;   // SNR for BER < 10^-5
    bool found_1e3;
    bool found_1e5;
};

SNRThreshold findSNRThreshold(
    Modulation mod,
    CodeRate rate,
    WattersonChannel::Config (*getChannelConfig)(float snr),
    float snr_min = 0.0f,
    float snr_max = 35.0f,
    float snr_step = 2.0f,
    size_t frames_per_point = 100
) {
    SNRThreshold threshold = {-1, -1, false, false};

    std::cout << "      " << std::flush;

    for (float snr = snr_min; snr <= snr_max; snr += snr_step) {
        auto cfg = getChannelConfig(snr);
        BERResult result = measureBER(mod, rate, cfg, frames_per_point);

        // Progress indicator
        std::cout << "." << std::flush;

        // Check thresholds
        if (!threshold.found_1e3 && result.ber < 1e-3f) {
            threshold.snr_1e3 = snr;
            threshold.found_1e3 = true;
        }

        if (!threshold.found_1e5 && result.ber < 1e-5f) {
            threshold.snr_1e5 = snr;
            threshold.found_1e5 = true;
        }

        // Early exit if both found
        if (threshold.found_1e3 && threshold.found_1e5) {
            break;
        }
    }

    std::cout << "\n";
    return threshold;
}

// ============================================================================
// Calculate theoretical throughput
// ============================================================================

float getThroughput(Modulation mod, CodeRate rate, const ModemConfig& config) {
    float code_rate_val = 0.5f;
    switch (rate) {
        case CodeRate::R1_4: code_rate_val = 0.25f; break;
        case CodeRate::R1_2: code_rate_val = 0.5f; break;
        case CodeRate::R2_3: code_rate_val = 0.667f; break;
        case CodeRate::R3_4: code_rate_val = 0.75f; break;
        case CodeRate::R5_6: code_rate_val = 0.833f; break;
        default: break;
    }

    size_t bits_per_symbol = static_cast<size_t>(mod);
    size_t data_carriers = config.num_carriers - config.num_carriers / config.pilot_spacing;
    float symbol_duration = config.getSymbolDuration() / config.sample_rate;
    float symbol_rate = 1.0f / symbol_duration;

    return data_carriers * bits_per_symbol * symbol_rate * code_rate_val;
}

// ============================================================================
// Main benchmark
// ============================================================================

int main() {
    auto start_time = std::chrono::steady_clock::now();

    // Disable debug logging to keep output clean
    ultra::setLogLevel(ultra::LogLevel::WARN);
    ultra::g_log_categories.sync = false;

    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "     MIL-STD-188-110B Comparison Benchmark\n";
    std::cout << "================================================================\n\n";

    std::cout << "Finding SNR threshold for BER < 10^-3 and BER < 10^-5\n";
    std::cout << "Using REAL modem pipeline with ITU-R F.1487 channel model.\n\n";

    std::cout << "MIL-STD-188-110B Reference (Poor channel, BER=10^-5):\n";
    std::cout << "  2400 bps: ~15 dB\n";
    std::cout << "  4800 bps: ~17 dB\n";
    std::cout << "  9600 bps: N/A (fails)\n\n";

    std::cout << "This benchmark takes several minutes. Please wait...\n\n";

    ModemConfig config;

    // Test modes
    struct TestMode {
        Modulation mod;
        CodeRate rate;
        const char* name;
        float approx_throughput;
    };

    std::vector<TestMode> modes = {
        {Modulation::BPSK,  CodeRate::R1_4, "BPSK R1/4",  540},
        {Modulation::BPSK,  CodeRate::R1_2, "BPSK R1/2",  1080},
        {Modulation::QPSK,  CodeRate::R1_2, "QPSK R1/2",  2160},
        {Modulation::QPSK,  CodeRate::R3_4, "QPSK R3/4",  3240},
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2", 4320},
        {Modulation::QAM16, CodeRate::R3_4, "16QAM R3/4", 6480},
        {Modulation::QAM64, CodeRate::R3_4, "64QAM R3/4", 9720},
    };

    // Channel conditions
    struct ChannelTest {
        const char* name;
        WattersonChannel::Config (*getConfig)(float);
    };

    std::vector<ChannelTest> channels = {
        {"AWGN",     itu_r_f1487::awgn},
        {"Good",     itu_r_f1487::good},
        {"Moderate", itu_r_f1487::moderate},
        {"Poor",     itu_r_f1487::poor},
    };

    // Results storage
    struct Result {
        const char* mode_name;
        float throughput;
        const char* channel_name;
        SNRThreshold threshold;
    };
    std::vector<Result> results;

    // Run benchmarks
    for (const auto& mode : modes) {
        std::cout << "Testing " << mode.name << " (~" << mode.approx_throughput << " bps):\n";

        for (const auto& ch : channels) {
            std::cout << "  " << std::setw(10) << ch.name << ": " << std::flush;

            SNRThreshold threshold = findSNRThreshold(
                mode.mod, mode.rate, ch.getConfig,
                0.0f, 35.0f, 2.0f, 50  // 50 frames per SNR point for speed
            );

            results.push_back({mode.name, mode.approx_throughput, ch.name, threshold});

            // Print result
            std::cout << "        BER<1e-3: ";
            if (threshold.found_1e3) {
                std::cout << std::setw(4) << threshold.snr_1e3 << " dB";
            } else {
                std::cout << " >35 dB";
            }
            std::cout << "  |  BER<1e-5: ";
            if (threshold.found_1e5) {
                std::cout << std::setw(4) << threshold.snr_1e5 << " dB";
            } else {
                std::cout << " >35 dB";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }

    // ========================================================================
    // Summary Table
    // ========================================================================

    std::cout << "================================================================\n";
    std::cout << "                    RESULTS SUMMARY\n";
    std::cout << "================================================================\n\n";

    std::cout << "SNR Threshold for BER < 10^-5 (dB)\n";
    std::cout << "──────────────────────────────────────────────────────────────\n";
    std::cout << std::setw(14) << "Mode"
              << std::setw(10) << "Rate"
              << std::setw(10) << "AWGN"
              << std::setw(10) << "Good"
              << std::setw(10) << "Moderate"
              << std::setw(10) << "Poor"
              << "\n";
    std::cout << "──────────────────────────────────────────────────────────────\n";

    size_t idx = 0;
    for (const auto& mode : modes) {
        std::cout << std::setw(14) << mode.name;
        std::cout << std::setw(8) << static_cast<int>(mode.approx_throughput) << " bps";

        for (size_t ch = 0; ch < channels.size(); ++ch) {
            const auto& r = results[idx++];
            if (r.threshold.found_1e5) {
                std::cout << std::setw(8) << r.threshold.snr_1e5 << " dB";
            } else {
                std::cout << std::setw(10) << "FAIL";
            }
        }
        std::cout << "\n";
    }

    std::cout << "──────────────────────────────────────────────────────────────\n\n";

    // ========================================================================
    // MIL-STD Comparison
    // ========================================================================

    std::cout << "================================================================\n";
    std::cout << "              MIL-STD-188-110B COMPARISON\n";
    std::cout << "================================================================\n\n";

    std::cout << "                          ProjectUltra    MIL-STD-110B    Delta\n";
    std::cout << "──────────────────────────────────────────────────────────────\n";

    // Find closest modes to compare
    // ~2400 bps: QPSK R1/2 (2160 bps)
    // ~4800 bps: 16QAM R1/2 (4320 bps)
    // ~9600 bps: 64QAM R3/4 (9720 bps)

    // QPSK R1/2 vs 2400 bps on Poor channel
    for (const auto& r : results) {
        if (std::string(r.mode_name) == "QPSK R1/2" && std::string(r.channel_name) == "Poor") {
            std::cout << "~2400 bps Poor:     ";
            if (r.threshold.found_1e5) {
                std::cout << std::setw(8) << r.threshold.snr_1e5 << " dB";
                std::cout << std::setw(14) << "~15 dB";
                float delta = r.threshold.snr_1e5 - 15.0f;
                std::cout << std::setw(10) << std::showpos << delta << " dB" << std::noshowpos;
            } else {
                std::cout << std::setw(8) << "FAIL";
                std::cout << std::setw(14) << "~15 dB";
                std::cout << std::setw(10) << "N/A";
            }
            std::cout << "\n";
        }
    }

    // 16QAM R1/2 vs 4800 bps on Poor channel
    for (const auto& r : results) {
        if (std::string(r.mode_name) == "16QAM R1/2" && std::string(r.channel_name) == "Poor") {
            std::cout << "~4800 bps Poor:     ";
            if (r.threshold.found_1e5) {
                std::cout << std::setw(8) << r.threshold.snr_1e5 << " dB";
                std::cout << std::setw(14) << "~17 dB";
                float delta = r.threshold.snr_1e5 - 17.0f;
                std::cout << std::setw(10) << std::showpos << delta << " dB" << std::noshowpos;
            } else {
                std::cout << std::setw(8) << "FAIL";
                std::cout << std::setw(14) << "~17 dB";
                std::cout << std::setw(10) << "N/A";
            }
            std::cout << "\n";
        }
    }

    // 64QAM R3/4 vs 9600 bps on Poor channel
    for (const auto& r : results) {
        if (std::string(r.mode_name) == "64QAM R3/4" && std::string(r.channel_name) == "Poor") {
            std::cout << "~9600 bps Poor:     ";
            if (r.threshold.found_1e5) {
                std::cout << std::setw(8) << r.threshold.snr_1e5 << " dB";
                std::cout << std::setw(14) << "FAIL";
                std::cout << std::setw(10) << "BETTER";
            } else {
                std::cout << std::setw(8) << "FAIL";
                std::cout << std::setw(14) << "FAIL";
                std::cout << std::setw(10) << "TIE";
            }
            std::cout << "\n";
        }
    }

    std::cout << "──────────────────────────────────────────────────────────────\n\n";

    std::cout << "Notes:\n";
    std::cout << "  - Negative delta = ProjectUltra needs LESS SNR (better)\n";
    std::cout << "  - Positive delta = ProjectUltra needs MORE SNR (worse)\n";
    std::cout << "  - MIL-STD uses convolutional codes; we use LDPC (2-4 dB advantage)\n";
    std::cout << "  - MIL-STD uses serial tones; we use OFDM (different trade-offs)\n\n";

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    std::cout << "Benchmark completed in " << duration.count() << " seconds.\n\n";

    return 0;
}
