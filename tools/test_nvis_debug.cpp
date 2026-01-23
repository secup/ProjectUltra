/**
 * Debug NVIS vs Minimal Test - find the difference
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

bool testConfig(ModemConfig config, const char* name, std::mt19937& rng) {
    std::cout << "--- " << name << " ---\n";
    std::cout << "  FFT: " << config.fft_size
              << ", Carriers: " << config.num_carriers
              << ", CP: " << config.getCyclicPrefix()
              << ", use_pilots: " << config.use_pilots << "\n";

    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);

    // Create test data (40 bytes for R1/2)
    Bytes data(40);
    for (auto& b : data) b = rng() & 0xFF;
    std::cout << "  Data[0..3]: " << std::hex
              << (int)data[0] << " " << (int)data[1] << " "
              << (int)data[2] << " " << (int)data[3] << std::dec << "\n";

    // Encode and modulate
    Bytes encoded = encoder.encode(data);
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

    // Combine
    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    // Normalize
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    // Demodulate
    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    std::cout << "  Soft bits: " << soft.size() << "\n";

    if (soft.size() < 648) {
        std::cout << "  FAILED: Not enough soft bits\n\n";
        return false;
    }

    std::span<const float> llrs(soft.data(), 648);
    Bytes decoded = decoder.decodeSoft(llrs);

    bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
    if (match) {
        for (size_t i = 0; i < data.size(); i++) {
            if (data[i] != decoded[i]) { match = false; break; }
        }
    }

    std::cout << "  Result: " << (match ? "PASS" : "FAIL") << "\n";
    if (!match && decoded.size() >= 4) {
        std::cout << "  Expected[0..3]: " << std::hex
                  << (int)data[0] << " " << (int)data[1] << " "
                  << (int)data[2] << " " << (int)data[3] << std::dec << "\n";
        std::cout << "  Decoded [0..3]: " << std::hex
                  << (int)decoded[0] << " " << (int)decoded[1] << " "
                  << (int)decoded[2] << " " << (int)decoded[3] << std::dec << "\n";
    }
    std::cout << "\n";
    return match;
}

int main() {
    setLogLevel(LogLevel::WARN);

    std::cout << "=== Test 1: Minimal-style (512 then 1024) ===\n\n";
    {
        std::mt19937 rng(12345);

        // Test 1a: 512 FFT (like minimal test)
        ModemConfig cfg512;
        cfg512.fft_size = 512;
        cfg512.num_carriers = 30;
        cfg512.cp_mode = CyclicPrefixMode::MEDIUM;
        cfg512.symbol_guard = 0;
        cfg512.pilot_spacing = 2;
        cfg512.modulation = Modulation::DQPSK;
        cfg512.code_rate = CodeRate::R1_2;
        cfg512.use_pilots = false;

        testConfig(cfg512, "512 FFT (manual config)", rng);

        // Test 1b: 1024 FFT (like minimal test, RNG already warmed)
        ModemConfig cfg1024;
        cfg1024.fft_size = 1024;
        cfg1024.num_carriers = 59;
        cfg1024.cp_mode = CyclicPrefixMode::MEDIUM;
        cfg1024.symbol_guard = 0;
        cfg1024.pilot_spacing = 2;
        cfg1024.modulation = Modulation::DQPSK;
        cfg1024.code_rate = CodeRate::R1_2;
        cfg1024.use_pilots = false;

        testConfig(cfg1024, "1024 FFT (manual config, warmed RNG)", rng);
    }

    std::cout << "=== Test 2: Fresh 1024 FFT (like NVIS test) ===\n\n";
    {
        std::mt19937 rng(12345);  // Fresh RNG

        // Test 2: 1024 FFT from nvis_mode() preset
        ModemConfig cfg = presets::nvis_mode();
        cfg.modulation = Modulation::DQPSK;
        cfg.code_rate = CodeRate::R1_2;
        cfg.use_pilots = false;

        testConfig(cfg, "1024 FFT (nvis preset, fresh RNG)", rng);
    }

    std::cout << "=== Test 3: Fresh 1024 FFT with manual config ===\n\n";
    {
        std::mt19937 rng(12345);  // Fresh RNG

        // Test 3: 1024 FFT manual config, fresh RNG
        ModemConfig cfg;
        cfg.fft_size = 1024;
        cfg.num_carriers = 59;
        cfg.cp_mode = CyclicPrefixMode::MEDIUM;
        cfg.symbol_guard = 0;
        cfg.pilot_spacing = 2;
        cfg.modulation = Modulation::DQPSK;
        cfg.code_rate = CodeRate::R1_2;
        cfg.use_pilots = false;

        testConfig(cfg, "1024 FFT (manual config, fresh RNG)", rng);
    }

    return 0;
}
