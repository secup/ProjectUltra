// Test OFDM + LDPC R2/3 integration (no HF channel, just AWGN)
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

// Add AWGN noise
void addAWGN(Samples& signal, float snr_db, std::mt19937& rng) {
    // Calculate signal RMS
    float sig_power = 0;
    for (float s : signal) sig_power += s * s;
    sig_power /= signal.size();

    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = sig_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : signal) s += noise(rng);
}

int main() {
    setLogLevel(LogLevel::WARN);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.use_pilots = false;

    std::mt19937 rng(42);
    int trials = 50;

    printf("=== OFDM + LDPC Integration Test ===\n\n");
    printf("DQPSK with 30 carriers, no pilots\n\n");

    // Test R2/3 at various SNR with NO NOISE first
    printf("=== R2/3 NO NOISE (perfect channel) ===\n");
    {
        int success = 0;
        int sync_failures = 0;

        for (int t = 0; t < trials; t++) {
            OFDMModulator modulator(config);
            OFDMDemodulator demod(config);
            LDPCEncoder encoder(CodeRate::R2_3);
            LDPCDecoder decoder(CodeRate::R2_3);

            // R2/3: 54 bytes
            Bytes data(54);
            for (auto& b : data) b = rng() & 0xFF;

            Bytes encoded = encoder.encode(data);

            auto preamble = modulator.generatePreamble();
            auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

            Samples signal;
            signal.insert(signal.end(), preamble.begin(), preamble.end());
            signal.insert(signal.end(), modulated.begin(), modulated.end());

            // Normalize
            float max_val = 0;
            for (float s : signal) max_val = std::max(max_val, std::abs(s));
            for (float& s : signal) s *= 0.5f / max_val;

            // NO NOISE - demod directly
            for (size_t i = 0; i < signal.size(); i += 960) {
                size_t len = std::min((size_t)960, signal.size() - i);
                SampleSpan span(signal.data() + i, len);
                demod.process(span);
            }

            auto soft = demod.getSoftBits();
            if (soft.size() < 648) {
                sync_failures++;
                continue;
            }

            std::span<const float> llrs(soft.data(), 648);
            Bytes decoded = decoder.decodeSoft(llrs);

            bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
            if (match) {
                for (size_t i = 0; i < data.size(); i++) {
                    if (decoded[i] != data[i]) { match = false; break; }
                }
            }
            if (match) success++;
        }
        printf("No noise: %3d%% success (%d/%d), sync_fail=%d\n",
               success * 100 / trials, success, trials, sync_failures);
    }

    // Test R1/2 for comparison
    printf("\n=== R1/2 NO NOISE (perfect channel) ===\n");
    {
        int success = 0;
        int sync_failures = 0;

        for (int t = 0; t < trials; t++) {
            OFDMModulator modulator(config);
            OFDMDemodulator demod(config);
            LDPCEncoder encoder(CodeRate::R1_2);
            LDPCDecoder decoder(CodeRate::R1_2);

            // R1/2: 40 bytes
            Bytes data(40);
            for (auto& b : data) b = rng() & 0xFF;

            Bytes encoded = encoder.encode(data);

            auto preamble = modulator.generatePreamble();
            auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

            Samples signal;
            signal.insert(signal.end(), preamble.begin(), preamble.end());
            signal.insert(signal.end(), modulated.begin(), modulated.end());

            float max_val = 0;
            for (float s : signal) max_val = std::max(max_val, std::abs(s));
            for (float& s : signal) s *= 0.5f / max_val;

            for (size_t i = 0; i < signal.size(); i += 960) {
                size_t len = std::min((size_t)960, signal.size() - i);
                SampleSpan span(signal.data() + i, len);
                demod.process(span);
            }

            auto soft = demod.getSoftBits();
            if (soft.size() < 648) {
                sync_failures++;
                continue;
            }

            std::span<const float> llrs(soft.data(), 648);
            Bytes decoded = decoder.decodeSoft(llrs);

            bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
            if (match) {
                for (size_t i = 0; i < data.size(); i++) {
                    if (decoded[i] != data[i]) { match = false; break; }
                }
            }
            if (match) success++;
        }
        printf("No noise: %3d%% success (%d/%d), sync_fail=%d\n",
               success * 100 / trials, success, trials, sync_failures);
    }

    // Now test with AWGN
    printf("\n=== OFDM + LDPC with AWGN ===\n");
    printf("%-8s %10s %10s\n", "SNR", "R1/2", "R2/3");
    printf("%-8s %10s %10s\n", "--------", "----------", "----------");

    float snr_levels[] = {10, 15, 20, 25, 30};
    for (float snr : snr_levels) {
        int r12_success = 0, r23_success = 0;

        // Test R1/2
        for (int t = 0; t < trials; t++) {
            OFDMModulator modulator(config);
            OFDMDemodulator demod(config);
            LDPCEncoder encoder(CodeRate::R1_2);
            LDPCDecoder decoder(CodeRate::R1_2);

            Bytes data(40);
            for (auto& b : data) b = rng() & 0xFF;

            Bytes encoded = encoder.encode(data);

            auto preamble = modulator.generatePreamble();
            auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

            Samples signal;
            signal.insert(signal.end(), preamble.begin(), preamble.end());
            signal.insert(signal.end(), modulated.begin(), modulated.end());

            float max_val = 0;
            for (float s : signal) max_val = std::max(max_val, std::abs(s));
            for (float& s : signal) s *= 0.5f / max_val;

            addAWGN(signal, snr, rng);

            for (size_t i = 0; i < signal.size(); i += 960) {
                size_t len = std::min((size_t)960, signal.size() - i);
                SampleSpan span(signal.data() + i, len);
                demod.process(span);
            }

            auto soft = demod.getSoftBits();
            if (soft.size() >= 648) {
                std::span<const float> llrs(soft.data(), 648);
                Bytes decoded = decoder.decodeSoft(llrs);

                bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
                if (match) {
                    for (size_t i = 0; i < data.size(); i++) {
                        if (decoded[i] != data[i]) { match = false; break; }
                    }
                }
                if (match) r12_success++;
            }
        }

        // Test R2/3
        for (int t = 0; t < trials; t++) {
            OFDMModulator modulator(config);
            OFDMDemodulator demod(config);
            LDPCEncoder encoder(CodeRate::R2_3);
            LDPCDecoder decoder(CodeRate::R2_3);

            Bytes data(54);
            for (auto& b : data) b = rng() & 0xFF;

            Bytes encoded = encoder.encode(data);

            auto preamble = modulator.generatePreamble();
            auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

            Samples signal;
            signal.insert(signal.end(), preamble.begin(), preamble.end());
            signal.insert(signal.end(), modulated.begin(), modulated.end());

            float max_val = 0;
            for (float s : signal) max_val = std::max(max_val, std::abs(s));
            for (float& s : signal) s *= 0.5f / max_val;

            addAWGN(signal, snr, rng);

            for (size_t i = 0; i < signal.size(); i += 960) {
                size_t len = std::min((size_t)960, signal.size() - i);
                SampleSpan span(signal.data() + i, len);
                demod.process(span);
            }

            auto soft = demod.getSoftBits();
            if (soft.size() >= 648) {
                std::span<const float> llrs(soft.data(), 648);
                Bytes decoded = decoder.decodeSoft(llrs);

                bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
                if (match) {
                    for (size_t i = 0; i < data.size(); i++) {
                        if (decoded[i] != data[i]) { match = false; break; }
                    }
                }
                if (match) r23_success++;
            }
        }

        printf("%4.0f dB  %8d%% %8d%%\n",
               snr, r12_success * 100 / trials, r23_success * 100 / trials);
    }

    // Debug: Look at soft bit distribution for one R2/3 trial (no noise)
    printf("\n=== Debug: Soft bit analysis for R2/3 (no noise) ===\n");
    {
        OFDMModulator modulator(config);
        OFDMDemodulator demod(config);
        LDPCEncoder encoder(CodeRate::R2_3);

        Bytes data(54);
        for (auto& b : data) b = rng() & 0xFF;

        Bytes encoded = encoder.encode(data);

        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        for (float& s : signal) s *= 0.5f / max_val;

        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            demod.process(span);
        }

        auto soft = demod.getSoftBits();
        printf("Got %zu soft bits\n", soft.size());

        // Count bit errors
        int errors = 0;
        int weak_llr = 0;  // |LLR| < 1
        for (size_t i = 0; i < std::min((size_t)648, soft.size()); i++) {
            int tx_bit = (encoded[i / 8] >> (7 - (i % 8))) & 1;
            int rx_bit = soft[i] > 0 ? 0 : 1;
            if (tx_bit != rx_bit) errors++;
            if (std::abs(soft[i]) < 1.0f) weak_llr++;
        }
        printf("Bit errors: %d / 648 (%.1f%%)\n", errors, errors * 100.0 / 648);
        printf("Weak LLRs (|LLR|<1): %d\n", weak_llr);

        // Show first 60 soft bits (first OFDM symbol)
        printf("\nFirst 60 soft bits (first OFDM symbol):\n");
        for (int i = 0; i < 60 && i < (int)soft.size(); i++) {
            int tx_bit = (encoded[i / 8] >> (7 - (i % 8))) & 1;
            int rx_bit = soft[i] > 0 ? 0 : 1;
            char mark = (tx_bit != rx_bit) ? '*' : ' ';
            printf("%c%6.1f", mark, soft[i]);
            if ((i + 1) % 10 == 0) printf("\n");
        }

        // Show errors by position
        printf("\nErrors by OFDM symbol:\n");
        for (int sym = 0; sym < 11 && sym * 60 < (int)soft.size(); sym++) {
            int sym_errors = 0;
            for (int i = sym * 60; i < (sym + 1) * 60 && i < (int)soft.size(); i++) {
                int tx_bit = (encoded[i / 8] >> (7 - (i % 8))) & 1;
                int rx_bit = soft[i] > 0 ? 0 : 1;
                if (tx_bit != rx_bit) sym_errors++;
            }
            printf("  Symbol %2d: %d errors\n", sym, sym_errors);
        }
    }

    return 0;
}
