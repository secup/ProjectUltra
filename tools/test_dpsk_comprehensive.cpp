/**
 * Comprehensive DPSK Performance Test
 *
 * Tests all DPSK modes across SNR range to find optimal throughput
 */

#include "psk/dpsk.hpp"
#include "ultra/fec.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <vector>
#include <string>

using namespace ultra;
using namespace ultra::sim;

// Add AWGN to samples
void addAWGN(Samples& samples, float snr_db, std::mt19937& rng) {
    float signal_power = 0;
    for (float s : samples) signal_power += s * s;
    signal_power /= samples.size();

    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = signal_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) s += noise(rng);
}

// Test DPSK at given SNR with specified code rate
float testDPSK(const DPSKConfig& config, CodeRate rate, float snr_db,
               int trials = 100, bool use_channel = false,
               WattersonChannel::Config (*getChannel)(float) = nullptr) {
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    std::mt19937 rng(42 + (int)(snr_db * 100));

    Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                       0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                       0xDE, 0xAD, 0xBE, 0xEF};

    int success = 0;

    for (int t = 0; t < trials; t++) {
        DPSKModulator mod(config);
        DPSKDemodulator demod(config);

        Bytes encoded = encoder.encode(test_data);
        Samples tx = mod.modulate(encoded);

        float max_val = 0;
        for (float s : tx) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx) s *= 0.5f / max_val;

        Samples rx;
        if (use_channel && getChannel) {
            WattersonChannel channel(getChannel(snr_db));
            rx = channel.process(SampleSpan(tx.data(), tx.size()));
        } else {
            rx = tx;
            addAWGN(rx, snr_db, rng);
        }

        auto soft_bits = demod.demodulateSoft(SampleSpan(rx.data(), rx.size()));

        size_t expected_bits = 648;
        if (soft_bits.size() < expected_bits) continue;
        soft_bits.resize(expected_bits);

        float sum = 0;
        for (float f : soft_bits) sum += std::abs(f);
        float avg = sum / soft_bits.size();
        if (avg > 0.01f) {
            float scale = 3.5f / avg;
            for (float& f : soft_bits) f *= scale;
        }

        Bytes decoded = decoder.decodeSoft(soft_bits);
        if (!decoder.lastDecodeSuccess()) continue;

        decoded.resize(test_data.size());
        if (decoded == test_data) success++;
    }

    return 100.0f * success / trials;
}

// Calculate throughput
float calcThroughput(const DPSKConfig& config, CodeRate rate) {
    float raw_bps = config.raw_bps();
    float code_rate = 0.25f;
    switch (rate) {
        case CodeRate::R1_4: code_rate = 0.25f; break;
        case CodeRate::R1_2: code_rate = 0.5f; break;
        case CodeRate::R2_3: code_rate = 0.667f; break;
        case CodeRate::R3_4: code_rate = 0.75f; break;
        default: code_rate = 0.25f; break;
    }
    return raw_bps * code_rate;
}

int main() {
    std::cout << "=== Comprehensive DPSK Performance Test ===\n\n";

    // Define test configurations
    struct ModeConfig {
        std::string name;
        DPSKConfig (*getConfig)();
        CodeRate rate;
        float min_snr;
    };

    std::vector<ModeConfig> modes = {
        // Conservative modes for low SNR
        {"DBPSK 31b R1/4", dpsk_presets::robust, CodeRate::R1_4, -3},
        {"DBPSK 62b R1/4", dpsk_presets::low_snr, CodeRate::R1_4, 0},
        {"DQPSK 62b R1/4", dpsk_presets::medium, CodeRate::R1_4, 3},
        {"DQPSK 125b R1/4", dpsk_presets::fast, CodeRate::R1_4, 5},
        {"DQPSK 125b R1/2", dpsk_presets::fast, CodeRate::R1_2, 8},

        // High-throughput modes
        {"DQPSK 300b R1/2", dpsk_presets::speed1, CodeRate::R1_2, 5},
        {"DQPSK 375b R1/2", dpsk_presets::speed2, CodeRate::R1_2, 8},
        {"DQPSK 500b R1/2", dpsk_presets::speed3, CodeRate::R1_2, 10},
        {"D8PSK 375b R1/2", dpsk_presets::speed4, CodeRate::R1_2, 12},
        {"D8PSK 750b R1/2", dpsk_presets::max_speed, CodeRate::R1_2, 13},
    };

    std::vector<float> snr_levels = {0, 3, 5, 8, 10, 12, 15};

    // Test 1: AWGN Performance
    std::cout << "=== Test 1: AWGN Channel ===\n\n";

    std::cout << std::setw(20) << "Mode" << std::setw(10) << "Thru'put";
    for (float snr : snr_levels) {
        std::cout << std::setw(7) << (int)snr << "dB";
    }
    std::cout << "\n" << std::string(90, '-') << "\n";

    for (auto& mode : modes) {
        auto config = mode.getConfig();
        float throughput = calcThroughput(config, mode.rate);

        std::cout << std::setw(20) << mode.name;
        std::cout << std::setw(9) << std::fixed << std::setprecision(0) << throughput << "b";

        for (float snr : snr_levels) {
            if (snr < mode.min_snr) {
                std::cout << std::setw(7) << "-";
            } else {
                float r = testDPSK(config, mode.rate, snr, 100);
                if (r >= 90) std::cout << std::setw(6) << std::fixed << std::setprecision(0) << r << "%";
                else if (r >= 70) std::cout << std::setw(6) << r << "*";
                else if (r >= 50) std::cout << std::setw(6) << r << "~";
                else std::cout << std::setw(6) << r << "!";
            }
        }
        std::cout << "\n";
    }

    std::cout << "\n(% = >90%, * = 70-90%, ~ = 50-70%, ! = <50%)\n";

    // Test 2: Find optimal mode for each SNR
    std::cout << "\n=== Test 2: Optimal Mode at Each SNR ===\n\n";

    std::cout << std::setw(8) << "SNR" << std::setw(22) << "Best Mode"
              << std::setw(15) << "Throughput\n";
    std::cout << std::string(45, '-') << "\n";

    for (float snr : snr_levels) {
        std::string best_mode;
        float best_throughput = 0;

        for (auto& mode : modes) {
            if (snr < mode.min_snr) continue;

            auto config = mode.getConfig();
            float success = testDPSK(config, mode.rate, snr, 100);

            if (success >= 90) {
                float throughput = calcThroughput(config, mode.rate);
                if (throughput > best_throughput) {
                    best_throughput = throughput;
                    best_mode = mode.name;
                }
            }
        }

        std::cout << std::setw(6) << (int)snr << "dB";
        if (best_throughput > 0) {
            std::cout << std::setw(22) << best_mode;
            std::cout << std::setw(12) << std::fixed << std::setprecision(0) << best_throughput << " bps\n";
        } else {
            std::cout << std::setw(22) << "None >90%\n";
        }
    }

    // Test 3: HF Channel Performance
    std::cout << "\n=== Test 3: Good HF Channel ===\n\n";

    std::vector<ModeConfig> hf_modes = {
        {"DBPSK 62b R1/4", dpsk_presets::low_snr, CodeRate::R1_4, 0},
        {"DQPSK 125b R1/4", dpsk_presets::fast, CodeRate::R1_4, 5},
        {"DQPSK 300b R1/2", dpsk_presets::speed1, CodeRate::R1_2, 5},
        {"DQPSK 500b R1/2", dpsk_presets::speed3, CodeRate::R1_2, 10},
    };

    std::cout << std::setw(20) << "Mode" << std::setw(10) << "Thru'put";
    for (float snr : snr_levels) {
        std::cout << std::setw(7) << (int)snr << "dB";
    }
    std::cout << "\n" << std::string(90, '-') << "\n";

    for (auto& mode : hf_modes) {
        auto config = mode.getConfig();
        float throughput = calcThroughput(config, mode.rate);

        std::cout << std::setw(20) << mode.name;
        std::cout << std::setw(9) << std::fixed << std::setprecision(0) << throughput << "b";

        for (float snr : snr_levels) {
            if (snr < mode.min_snr) {
                std::cout << std::setw(7) << "-";
            } else {
                float r = testDPSK(config, mode.rate, snr, 50, true, itu_r_f1487::good);
                if (r >= 90) std::cout << std::setw(6) << std::fixed << std::setprecision(0) << r << "%";
                else if (r >= 70) std::cout << std::setw(6) << r << "*";
                else if (r >= 50) std::cout << std::setw(6) << r << "~";
                else std::cout << std::setw(6) << r << "!";
            }
        }
        std::cout << "\n";
    }

    // Summary
    std::cout << "\n=== Summary ===\n";
    std::cout << "DPSK provides reliable single-carrier communication from 0-15 dB SNR\n";
    std::cout << "filling the gap between MFSK (<0 dB) and OFDM (>15 dB).\n";

    return 0;
}
