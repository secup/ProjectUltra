/**
 * DPSK Performance Test
 *
 * Tests single-carrier DPSK at various SNR levels to verify it fills
 * the gap between MFSK (< 0 dB) and OFDM (> 15 dB)
 *
 * Target: 0-15 dB SNR range with good performance
 */

#include "psk/dpsk.hpp"
#include "ultra/fec.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>

using namespace ultra;
using namespace ultra::sim;

// Add AWGN to samples
void addAWGN(Samples& samples, float snr_db, std::mt19937& rng) {
    // Calculate signal power
    float signal_power = 0;
    for (float s : samples) signal_power += s * s;
    signal_power /= samples.size();

    // Calculate noise power for desired SNR
    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = signal_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) {
        s += noise(rng);
    }
}

// Test DPSK with LDPC at given SNR
float testDPSK(const DPSKConfig& config, float snr_db, int trials = 50) {
    LDPCEncoder encoder(CodeRate::R1_4);
    LDPCDecoder decoder(CodeRate::R1_4);

    std::mt19937 rng(42);

    int success = 0;
    Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                       0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                       0xDE, 0xAD, 0xBE, 0xEF};  // 20 bytes

    for (int t = 0; t < trials; t++) {
        DPSKModulator mod(config);
        DPSKDemodulator demod(config);

        // Encode with LDPC
        Bytes encoded = encoder.encode(test_data);

        // Modulate (no preamble for this test - direct timing)
        Samples tx = mod.modulate(encoded);

        // Normalize
        float max_val = 0;
        for (float s : tx) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx) s *= 0.5f / max_val;

        // Add noise
        addAWGN(tx, snr_db, rng);

        // Demodulate directly (perfect timing for this test)
        SampleSpan rx_span(tx.data(), tx.size());
        auto soft_bits = demod.demodulateSoft(rx_span);

        // Need 648 soft bits for LDPC R1/4
        if (soft_bits.size() < 648) continue;
        soft_bits.resize(648);

        // Scale soft bits for LDPC (target magnitude ~3.5)
        float sum = 0;
        for (float f : soft_bits) sum += std::abs(f);
        float avg = sum / soft_bits.size();
        if (avg > 0.01f) {
            float scale = 3.5f / avg;
            for (float& f : soft_bits) f *= scale;
        }

        // Decode LDPC
        Bytes decoded = decoder.decodeSoft(soft_bits);
        if (!decoder.lastDecodeSuccess()) continue;

        // Compare
        decoded.resize(test_data.size());
        if (decoded == test_data) success++;
    }

    return 100.0f * success / trials;
}

// Test DPSK with HF channel
float testDPSKChannel(const DPSKConfig& config,
                       WattersonChannel::Config (*getChannel)(float),
                       float snr_db, int trials = 30) {
    LDPCEncoder encoder(CodeRate::R1_4);
    LDPCDecoder decoder(CodeRate::R1_4);

    WattersonChannel channel(getChannel(snr_db));

    int success = 0;
    Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                       0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                       0xDE, 0xAD, 0xBE, 0xEF};

    for (int t = 0; t < trials; t++) {
        DPSKModulator mod(config);
        DPSKDemodulator demod(config);

        // Encode with LDPC
        Bytes encoded = encoder.encode(test_data);

        // Modulate (no preamble for this test)
        Samples tx = mod.modulate(encoded);

        // Normalize
        float max_val = 0;
        for (float s : tx) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx) s *= 0.5f / max_val;

        // Pass through HF channel
        Samples rx = channel.process(SampleSpan(tx.data(), tx.size()));

        // Demodulate directly
        SampleSpan rx_span(rx.data(), rx.size());
        auto soft_bits = demod.demodulateSoft(rx_span);

        if (soft_bits.size() < 648) continue;
        soft_bits.resize(648);

        // Scale soft bits
        float sum = 0;
        for (float f : soft_bits) sum += std::abs(f);
        float avg = sum / soft_bits.size();
        if (avg > 0.01f) {
            float scale = 3.5f / avg;
            for (float& f : soft_bits) f *= scale;
        }

        // Decode LDPC
        Bytes decoded = decoder.decodeSoft(soft_bits);
        if (!decoder.lastDecodeSuccess()) continue;

        decoded.resize(test_data.size());
        if (decoded == test_data) success++;
    }

    return 100.0f * success / trials;
}

int main() {
    std::cout << "=== DPSK Performance Test ===\n\n";

    // Test 1: Basic modulation/demodulation (no noise)
    std::cout << "1. Basic DPSK test (no noise):\n";
    {
        auto config = dpsk_presets::medium();
        DPSKModulator mod(config);
        DPSKDemodulator demod(config);

        Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF};
        auto preamble = mod.generatePreamble(32);
        auto audio = mod.modulate(test_data);

        Samples rx;
        rx.insert(rx.end(), preamble.begin(), preamble.end());
        rx.insert(rx.end(), audio.begin(), audio.end());

        int offset = demod.findPreamble(SampleSpan(rx.data(), rx.size()), 32);
        std::cout << "   Preamble found at offset: " << offset << "\n";

        int preamble_len = 32 * config.samples_per_symbol;
        SampleSpan data_span(rx.data() + offset + preamble_len, audio.size());
        Bytes decoded = demod.demodulate(data_span);

        bool match = (decoded.size() >= 4 &&
                     decoded[0] == 0xDE && decoded[1] == 0xAD &&
                     decoded[2] == 0xBE && decoded[3] == 0xEF);
        std::cout << "   Result: " << (match ? "PASS" : "FAIL") << "\n\n";
    }

    // Test 2: All DPSK presets
    std::cout << "2. DPSK presets:\n";
    struct { const char* name; DPSKConfig (*get)(); } presets[] = {
        {"robust",    dpsk_presets::robust},
        {"low_snr",   dpsk_presets::low_snr},
        {"medium",    dpsk_presets::medium},
        {"fast",      dpsk_presets::fast},
        {"turbo",     dpsk_presets::turbo},
        {"high_speed", dpsk_presets::high_speed},
    };

    for (auto& p : presets) {
        auto cfg = p.get();
        const char* mod_name = "?";
        switch (cfg.modulation) {
            case DPSKModulation::DBPSK: mod_name = "DBPSK"; break;
            case DPSKModulation::DQPSK: mod_name = "DQPSK"; break;
            case DPSKModulation::D8PSK: mod_name = "D8PSK"; break;
        }
        std::cout << "   " << std::setw(10) << p.name << ": "
                  << mod_name << " @ " << std::fixed << std::setprecision(1)
                  << cfg.symbol_rate() << " baud = "
                  << cfg.raw_bps() << " bps raw, ~"
                  << (cfg.raw_bps() * 0.25f) << " bps with R1/4\n";
    }
    std::cout << "\n";

    // Test 3: DPSK + LDPC with AWGN
    std::cout << "3. DPSK + LDPC R1/4 with AWGN:\n\n";

    struct { const char* name; DPSKConfig (*get)(); } modes[] = {
        {"DBPSK 31b", dpsk_presets::robust},
        {"DBPSK 62b", dpsk_presets::low_snr},
        {"DQPSK 62b", dpsk_presets::medium},
        {"DQPSK 125b", dpsk_presets::fast},
        {"D8PSK 125b", dpsk_presets::turbo},
    };

    int snrs[] = {0, 3, 5, 8, 10, 12, 15};

    std::cout << std::setw(12) << "Mode";
    for (int snr : snrs) std::cout << std::setw(7) << snr << "dB";
    std::cout << "\n" << std::string(70, '-') << "\n";

    for (auto& m : modes) {
        std::cout << std::setw(12) << m.name;
        auto cfg = m.get();

        for (int snr : snrs) {
            float r = testDPSK(cfg, (float)snr, 50);
            if (r >= 80) std::cout << std::setw(6) << std::fixed << std::setprecision(0) << r << "%";
            else if (r >= 50) std::cout << std::setw(6) << r << "*";
            else std::cout << std::setw(6) << r << "!";
        }
        std::cout << "\n";
    }
    std::cout << "\n(! = <50%, * = 50-80%, % = >80%)\n\n";

    // Test 4: DPSK with HF channels
    std::cout << "4. DPSK with HF channel simulation:\n\n";

    struct { const char* name; WattersonChannel::Config (*get)(float); } channels[] = {
        {"AWGN", itu_r_f1487::awgn},
        {"Good", itu_r_f1487::good},
        {"Moderate", itu_r_f1487::moderate},
    };

    // Use medium preset (DQPSK 62.5 baud) for HF channel test
    auto hf_config = dpsk_presets::medium();

    std::cout << "Using DQPSK @ 62.5 baud:\n\n";
    std::cout << std::setw(12) << "Channel";
    for (int snr : snrs) std::cout << std::setw(7) << snr << "dB";
    std::cout << "\n" << std::string(70, '-') << "\n";

    for (auto& ch : channels) {
        std::cout << std::setw(12) << ch.name;
        for (int snr : snrs) {
            float r = testDPSKChannel(hf_config, ch.get, (float)snr, 30);
            if (r >= 80) std::cout << std::setw(6) << std::fixed << std::setprecision(0) << r << "%";
            else if (r >= 50) std::cout << std::setw(6) << r << "*";
            else std::cout << std::setw(6) << r << "!";
        }
        std::cout << "\n";
    }

    std::cout << "\n";
    std::cout << "=== Summary ===\n";
    std::cout << "DPSK target range: 0-15 dB SNR\n";
    std::cout << "Expected: fills gap between MFSK (<0 dB) and OFDM (>15 dB)\n";
    std::cout << "\nThroughput with R1/4 LDPC:\n";
    std::cout << "  DBPSK 31.25 baud: ~8 bps\n";
    std::cout << "  DBPSK 62.5 baud:  ~16 bps\n";
    std::cout << "  DQPSK 62.5 baud:  ~31 bps\n";
    std::cout << "  DQPSK 125 baud:   ~62 bps\n";
    std::cout << "  D8PSK 125 baud:   ~94 bps\n";

    return 0;
}
