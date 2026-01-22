#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

struct TestResult {
    int success;
    float measured_snr;  // Demodulator-measured per-carrier SNR
    bool synced;
};

TestResult test_ofdm(float snr_db, Modulation mod, CodeRate rate, size_t data_bytes, int trials) {
    std::mt19937 rng(12345);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = mod;
    config.code_rate = rate;
    // Differential modulation doesn't need pilots
    config.use_pilots = (mod != Modulation::DQPSK && mod != Modulation::D8PSK && mod != Modulation::DBPSK);

    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    TestResult result{0, 0.0f, false};
    float snr_sum = 0.0f;
    int snr_count = 0;

    for (int t = 0; t < trials; t++) {
        OFDMDemodulator demod(config);

        Bytes data(data_bytes);
        for (auto& b : data) b = rng() & 0xFF;

        Bytes encoded = encoder.encode(data);
        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(encoded, mod);

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

        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            demod.process(span);
        }

        float measured_snr = demod.getEstimatedSNR();
        auto soft = demod.getSoftBits();
        // Debug: print soft bits count for first trial
        if (t == 0 && trials == 1) {
            printf("  [soft=%zu, synced=%d] ", soft.size(), demod.isSynced() ? 1 : 0);
        }
        if (soft.size() >= 648) {
            result.synced = true;
            snr_sum += measured_snr;
            snr_count++;

            std::span<const float> llrs(soft.data(), 648);
            Bytes decoded = decoder.decodeSoft(llrs);
            // Debug
            if (t == 0 && trials == 1) {
                printf("decode_ok=%d iters=%d dec_sz=%zu\n",
                       decoder.lastDecodeSuccess() ? 1 : 0, decoder.lastIterations(), decoded.size());
                // Show first mismatch
                for (size_t i = 0; i < std::min(data.size(), decoded.size()); i++) {
                    if (data[i] != decoded[i]) {
                        printf("  mismatch at byte %zu: expected 0x%02X got 0x%02X\n",
                               i, data[i], decoded[i]);
                        break;
                    }
                }
            }
            // Compare only the original data bytes (decoded may have extra padding bytes)
            bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
            if (match) {
                for (size_t i = 0; i < data.size(); i++) {
                    if (decoded[i] != data[i]) { match = false; break; }
                }
            }
            if (match) result.success++;
        }
    }
    if (snr_count > 0) result.measured_snr = snr_sum / snr_count;
    return result;
}

// Wrapper for backward compatibility
TestResult test_exact_size(float snr_db, CodeRate rate, size_t data_bytes, int trials) {
    return test_ofdm(snr_db, Modulation::DQPSK, rate, data_bytes, trials);
}

int main() {
    setLogLevel(LogLevel::WARN);
    int trials = 20;

    std::cout << "=== OFDM D8PSK vs DQPSK - Full SNR Range (AWGN) ===\n\n";

    // Test D8PSK across wide SNR range
    std::cout << "--- D8PSK R1/2 (~3.4 kbps) ---\n";
    std::cout << "SNR:   15 dB  17 dB  20 dB  23 dB  25 dB  28 dB  30 dB\n";
    printf("Result:");
    for (float snr : {15.0f, 17.0f, 20.0f, 23.0f, 25.0f, 28.0f, 30.0f}) {
        auto res = test_ofdm(snr, Modulation::D8PSK, CodeRate::R1_2, 40, trials);
        printf("%4d%%  ", res.success * 100 / trials);
    }
    std::cout << "\n";

    std::cout << "\n--- D8PSK R2/3 (~4.8 kbps) ---\n";
    std::cout << "SNR:   20 dB  23 dB  25 dB  28 dB  30 dB  33 dB  35 dB\n";
    printf("Result:");
    for (float snr : {20.0f, 23.0f, 25.0f, 28.0f, 30.0f, 33.0f, 35.0f}) {
        auto res = test_ofdm(snr, Modulation::D8PSK, CodeRate::R2_3, 54, trials);
        printf("%4d%%  ", res.success * 100 / trials);
    }
    std::cout << "\n";

    std::cout << "\n--- DQPSK R1/2 (~2.3 kbps) for comparison ---\n";
    std::cout << "SNR:   15 dB  17 dB  20 dB  23 dB  25 dB  28 dB  30 dB\n";
    printf("Result:");
    for (float snr : {15.0f, 17.0f, 20.0f, 23.0f, 25.0f, 28.0f, 30.0f}) {
        auto res = test_ofdm(snr, Modulation::DQPSK, CodeRate::R1_2, 40, trials);
        printf("%4d%%  ", res.success * 100 / trials);
    }
    std::cout << "\n";

    std::cout << "\n--- DQPSK R2/3 (~3.2 kbps) for comparison ---\n";
    std::cout << "SNR:   20 dB  23 dB  25 dB  28 dB  30 dB  33 dB  35 dB\n";
    printf("Result:");
    for (float snr : {20.0f, 23.0f, 25.0f, 28.0f, 30.0f, 33.0f, 35.0f}) {
        auto res = test_ofdm(snr, Modulation::DQPSK, CodeRate::R2_3, 54, trials);
        printf("%4d%%  ", res.success * 100 / trials);
    }
    std::cout << "\n";

    std::cout << "\n=== Summary ===\n";
    std::cout << "Mode          Rate   Throughput   Min SNR for 90%+\n";
    std::cout << "DQPSK         R1/2   ~2.3 kbps    17 dB\n";
    std::cout << "D8PSK         R1/2   ~3.4 kbps    ?? dB (check above)\n";
    std::cout << "DQPSK         R2/3   ~3.2 kbps    ?? dB\n";
    std::cout << "D8PSK         R2/3   ~4.8 kbps    ?? dB\n";

    return 0;
}
