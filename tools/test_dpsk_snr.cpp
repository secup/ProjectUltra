#include "psk/dpsk.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>

using namespace ultra;

struct TestResult {
    int success;
    float throughput_bps;
};

TestResult test_dpsk(float snr_db, DPSKModulation mod, int trials) {
    std::mt19937 rng(12345);

    // Configure DPSK - use medium symbol rate
    DPSKConfig config;
    config.sample_rate = 48000;
    config.carrier_freq = 1500.0f;
    config.modulation = mod;
    config.samples_per_symbol = 384;  // 125 baud

    DPSKModulator modulator(config);
    DPSKDemodulator demodulator(config);

    // Use R1/4 for robustness testing
    LDPCEncoder encoder(CodeRate::R1_4);
    LDPCDecoder decoder(CodeRate::R1_4);

    // Data size: 20 bytes fits in one R1/4 codeword
    const size_t data_bytes = 20;

    // Calculate effective throughput (raw * code_rate)
    TestResult result{0, config.raw_bps() * 0.25f};  // R1/4 = 0.25

    for (int t = 0; t < trials; t++) {
        Bytes data(data_bytes);
        for (auto& b : data) b = rng() & 0xFF;

        // Encode
        Bytes encoded = encoder.encode(data);

        // Modulate
        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(encoded);

        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        // Normalize
        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        for (float& s : signal) s *= 0.5f / max_val;

        // Add AWGN
        float sp = 0;
        for (float s : signal) sp += s * s;
        sp /= signal.size();
        float noise_std = std::sqrt(sp / std::pow(10.0f, snr_db / 10.0f));
        std::normal_distribution<float> noise(0.0f, noise_std);
        for (float& s : signal) s += noise(rng);

        // Demodulate - find preamble first, then demodulate data
        SampleSpan signal_span(signal.data(), signal.size());
        int data_start = demodulator.findPreamble(signal_span);

        if (data_start > 0 && data_start < (int)signal.size()) {
            SampleSpan data_span(signal.data() + data_start, signal.size() - data_start);
            auto soft = demodulator.demodulateSoft(data_span);

            if (soft.size() >= 648) {
                std::span<const float> llrs(soft.data(), 648);
                Bytes decoded = decoder.decodeSoft(llrs);

                bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
                if (match) {
                    for (size_t i = 0; i < data.size(); i++) {
                        if (decoded[i] != data[i]) { match = false; break; }
                    }
                }
                if (match) result.success++;
            }
        }
    }

    return result;
}

TestResult test_dpsk_rate(float snr_db, DPSKModulation mod, CodeRate rate, int trials) {
    std::mt19937 rng(12345);

    DPSKConfig config;
    config.sample_rate = 48000;
    config.carrier_freq = 1500.0f;
    config.modulation = mod;
    config.samples_per_symbol = 384;  // 125 baud

    DPSKModulator modulator(config);
    DPSKDemodulator demodulator(config);

    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    // Data sizes per rate (max bytes that fit in one codeword)
    size_t data_bytes = (rate == CodeRate::R1_4) ? 20 :
                        (rate == CodeRate::R1_2) ? 40 : 54;

    float rate_factor = (rate == CodeRate::R1_4) ? 0.25f :
                        (rate == CodeRate::R1_2) ? 0.5f : 0.667f;
    TestResult result{0, config.raw_bps() * rate_factor};

    for (int t = 0; t < trials; t++) {
        Bytes data(data_bytes);
        for (auto& b : data) b = rng() & 0xFF;

        Bytes encoded = encoder.encode(data);
        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(encoded);

        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        for (float& s : signal) s *= 0.5f / max_val;

        float sp = 0;
        for (float s : signal) sp += s * s;
        sp /= signal.size();
        float noise_std = std::sqrt(sp / std::pow(10.0f, snr_db / 10.0f));
        std::normal_distribution<float> noise(0.0f, noise_std);
        for (float& s : signal) s += noise(rng);

        SampleSpan signal_span(signal.data(), signal.size());
        int data_start = demodulator.findPreamble(signal_span);

        if (data_start > 0 && data_start < (int)signal.size()) {
            SampleSpan data_span(signal.data() + data_start, signal.size() - data_start);
            auto soft = demodulator.demodulateSoft(data_span);

            if (soft.size() >= 648) {
                std::span<const float> llrs(soft.data(), 648);
                Bytes decoded = decoder.decodeSoft(llrs);

                bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
                if (match) {
                    for (size_t i = 0; i < data.size(); i++) {
                        if (decoded[i] != data[i]) { match = false; break; }
                    }
                }
                if (match) result.success++;
            }
        }
    }
    return result;
}

int main() {
    setLogLevel(LogLevel::WARN);
    int trials = 20;

    std::cout << "=== Single-Carrier DPSK Performance vs SNR (AWGN) ===\n\n";

    // Test R1/4 at very low SNR
    std::cout << "--- R1/4 (most robust) ---\n";
    std::cout << "Mode       Throughput -10 dB  -8 dB  -5 dB  -3 dB   0 dB\n";
    for (auto mod : {DPSKModulation::DBPSK, DPSKModulation::DQPSK, DPSKModulation::D8PSK}) {
        const char* name = (mod == DPSKModulation::DBPSK) ? "DBPSK" :
                           (mod == DPSKModulation::DQPSK) ? "DQPSK" : "D8PSK";
        auto first = test_dpsk_rate(10.0f, mod, CodeRate::R1_4, 1);
        printf("%-10s %6.0f bps ", name, first.throughput_bps);
        for (float snr : {-10.0f, -8.0f, -5.0f, -3.0f, 0.0f}) {
            auto res = test_dpsk_rate(snr, mod, CodeRate::R1_4, trials);
            printf("%4d%%  ", res.success * 100 / trials);
        }
        std::cout << "\n";
    }

    // Test R1/2 at low SNR
    std::cout << "\n--- R1/2 (higher throughput) ---\n";
    std::cout << "Mode       Throughput   0 dB   3 dB   5 dB   8 dB  10 dB\n";
    for (auto mod : {DPSKModulation::DBPSK, DPSKModulation::DQPSK, DPSKModulation::D8PSK}) {
        const char* name = (mod == DPSKModulation::DBPSK) ? "DBPSK" :
                           (mod == DPSKModulation::DQPSK) ? "DQPSK" : "D8PSK";
        auto first = test_dpsk_rate(10.0f, mod, CodeRate::R1_2, 1);
        printf("%-10s %6.0f bps ", name, first.throughput_bps);
        for (float snr : {0.0f, 3.0f, 5.0f, 8.0f, 10.0f}) {
            auto res = test_dpsk_rate(snr, mod, CodeRate::R1_2, trials);
            printf("%4d%%  ", res.success * 100 / trials);
        }
        std::cout << "\n";
    }

    // Test R2/3 (max throughput)
    std::cout << "\n--- R2/3 (max throughput) ---\n";
    std::cout << "Mode       Throughput   5 dB   8 dB  10 dB  12 dB  15 dB\n";
    for (auto mod : {DPSKModulation::DBPSK, DPSKModulation::DQPSK, DPSKModulation::D8PSK}) {
        const char* name = (mod == DPSKModulation::DBPSK) ? "DBPSK" :
                           (mod == DPSKModulation::DQPSK) ? "DQPSK" : "D8PSK";
        auto first = test_dpsk_rate(10.0f, mod, CodeRate::R2_3, 1);
        printf("%-10s %6.0f bps ", name, first.throughput_bps);
        for (float snr : {5.0f, 8.0f, 10.0f, 12.0f, 15.0f}) {
            auto res = test_dpsk_rate(snr, mod, CodeRate::R2_3, trials);
            printf("%4d%%  ", res.success * 100 / trials);
        }
        std::cout << "\n";
    }

    std::cout << "\nDPSK gets full SNR (single-carrier). OFDM needs ~17 dB due to 30-carrier split.\n";

    return 0;
}
