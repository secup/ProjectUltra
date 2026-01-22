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

TestResult test_exact_size(float snr_db, CodeRate rate, size_t data_bytes, int trials) {
    std::mt19937 rng(12345);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.code_rate = rate;

    OFDMModulator mod(config);
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
        auto preamble = mod.generatePreamble();
        auto modulated = mod.modulate(encoded, config.modulation);

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
        if (soft.size() >= 648) {
            result.synced = true;
            snr_sum += measured_snr;
            snr_count++;

            std::span<const float> llrs(soft.data(), 648);
            Bytes decoded = decoder.decodeSoft(llrs);
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

int main() {
    setLogLevel(LogLevel::WARN);
    int trials = 10;  // Reduced for faster testing

    // Use max bytes that fit in one codeword for each rate
    // Fine sweep to find threshold
    std::cout << "=== OFDM DQPSK Code Rate Performance vs SNR (AWGN) ===\n\n";
    std::cout << "           ";
    for (float snr : {14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 20.0f}) {
        printf("%4.0f dB  ", snr);
    }
    std::cout << "\n";

    struct RateInfo {
        CodeRate rate;
        const char* name;
        size_t bytes;
    };

    std::vector<RateInfo> rates = {
        {CodeRate::R1_4, "R1/4 (20B)", 20},
        {CodeRate::R1_2, "R1/2 (40B)", 40},
        {CodeRate::R2_3, "R2/3 (54B)", 54},
    };

    for (const auto& r : rates) {
        printf("%-11s", r.name);
        for (float snr : {14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 20.0f}) {
            auto res = test_exact_size(snr, r.rate, r.bytes, trials);
            printf("%4d%%    ", res.success * 100 / trials);
        }
        std::cout << "\n";
    }

    // Show what the demodulator actually measures
    std::cout << "\nMeasured (per-carrier) SNR at each AWGN level:\n";
    std::cout << "AWGN: ";
    for (float snr : {14.0f, 15.0f, 16.0f, 17.0f, 18.0f, 20.0f}) {
        auto res = test_exact_size(snr, CodeRate::R1_4, 20, 5);
        if (res.synced) {
            printf("%.0f→%.0f  ", snr, res.measured_snr);
        } else {
            printf("%.0f→N/S  ", snr);  // No sync
        }
    }
    std::cout << "\n";

    std::cout << "\n=== Throughput Reference ===\n";
    std::cout << "R1/4: ~1.1 kbps   R1/2: ~2.3 kbps   R2/3: ~3.2 kbps   R3/4: ~3.6 kbps\n";

    return 0;
}
