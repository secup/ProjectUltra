#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include <iostream>
#include <cmath>
#include <random>

int main() {
    std::cout << "Testing OFDM implementation...\n\n";

    ultra::ModemConfig config;
    config.sample_rate = 48000;
    config.fft_size = 512;
    config.num_carriers = 48;
    config.cp_mode = ultra::CyclicPrefixMode::LONG;  // 64 samples

    // Test modulator
    std::cout << "Testing OFDM modulator...\n";
    ultra::OFDMModulator modulator(config);

    // Generate test data
    std::vector<uint8_t> test_data(32);
    for (size_t i = 0; i < test_data.size(); ++i) {
        test_data[i] = static_cast<uint8_t>(i * 7 + 13);
    }

    // Test each modulation scheme
    for (auto mod : {ultra::Modulation::BPSK, ultra::Modulation::QPSK,
                     ultra::Modulation::QAM16}) {
        auto samples = modulator.modulate(test_data, mod);

        const char* mod_name = "";
        switch (mod) {
            case ultra::Modulation::BPSK: mod_name = "BPSK"; break;
            case ultra::Modulation::QPSK: mod_name = "QPSK"; break;
            case ultra::Modulation::QAM16: mod_name = "QAM16"; break;
            default: mod_name = "?"; break;
        }

        std::cout << "  " << mod_name << ": " << samples.size() << " samples";

        // Check signal properties
        float peak = ultra::dsp::peak(samples);
        float rms = ultra::dsp::rms(samples);

        std::cout << " (peak=" << peak << ", rms=" << rms << ")";

        if (samples.size() > 0 && peak < 10 && peak > 0.01) {
            std::cout << " OK\n";
        } else {
            std::cout << " FAILED\n";
            return 1;
        }
    }

    // Test preamble generation
    std::cout << "\nTesting preamble generation...\n";
    auto preamble = modulator.generatePreamble();
    std::cout << "  Preamble: " << preamble.size() << " samples";

    float preamble_duration_ms = preamble.size() * 1000.0f / config.sample_rate;
    std::cout << " (" << preamble_duration_ms << " ms)";

    if (preamble.size() > 0 && preamble_duration_ms > 10 && preamble_duration_ms < 500) {
        std::cout << " OK\n";
    } else {
        std::cout << " FAILED\n";
        return 1;
    }

    // Test AWGN channel simulation
    std::cout << "\nTesting with AWGN channel...\n";

    // Generate a frame
    std::vector<uint8_t> tx_data = {0x55, 0xAA, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    auto tx_samples = modulator.modulate(tx_data, ultra::Modulation::QPSK);

    // Add noise
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0, 0.1f);

    std::vector<float> rx_samples = tx_samples;
    for (auto& s : rx_samples) {
        s += noise(rng);
    }

    // Create demodulator and process
    ultra::OFDMDemodulator demodulator(config);

    // Feed samples (would need preamble for sync in real scenario)
    // For now just verify the demodulator doesn't crash
    bool frame_ready = demodulator.process(rx_samples);
    std::cout << "  Demodulator processed " << rx_samples.size() << " samples";
    std::cout << " (frame_ready=" << frame_ready << ") OK\n";

    // Test data rate calculations
    std::cout << "\nTheoretical data rates:\n";
    for (auto mod : {ultra::Modulation::BPSK, ultra::Modulation::QPSK,
                     ultra::Modulation::QAM16, ultra::Modulation::QAM64}) {
        size_t bits_per_symbol = modulator.bitsPerSymbol(mod);
        size_t samples_per_symbol = modulator.samplesPerSymbol();
        float symbol_rate = static_cast<float>(config.sample_rate) / samples_per_symbol;
        float raw_bps = bits_per_symbol * symbol_rate;

        const char* mod_name = "";
        switch (mod) {
            case ultra::Modulation::BPSK: mod_name = "BPSK"; break;
            case ultra::Modulation::QPSK: mod_name = "QPSK"; break;
            case ultra::Modulation::QAM16: mod_name = "QAM16"; break;
            case ultra::Modulation::QAM64: mod_name = "QAM64"; break;
            default: break;
        }

        std::cout << "  " << mod_name << ": " << bits_per_symbol << " bits/sym, "
                  << static_cast<int>(raw_bps) << " raw bps\n";
    }

    std::cout << "\nAll OFDM tests passed!\n";
    return 0;
}
