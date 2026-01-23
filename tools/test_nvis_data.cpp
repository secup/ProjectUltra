/**
 * Test different data patterns on 1024 FFT
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

bool testData(ModemConfig config, const Bytes& data) {
    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);

    // Encode and modulate
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

    // Demodulate
    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    if (soft.size() < 648) return false;

    std::span<const float> llrs(soft.data(), 648);
    Bytes decoded = decoder.decodeSoft(llrs);

    bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
    if (match) {
        for (size_t i = 0; i < data.size(); i++) {
            if (data[i] != decoded[i]) { match = false; break; }
        }
    }
    return match;
}

int main() {
    setLogLevel(LogLevel::WARN);

    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;
    cfg.symbol_guard = 0;
    cfg.pilot_spacing = 2;
    cfg.modulation = Modulation::DQPSK;
    cfg.code_rate = CodeRate::R1_2;
    cfg.use_pilots = false;

    std::cout << "Testing 1024 FFT with different data patterns:\n\n";

    // Test known failing pattern
    Bytes fail_data = {0xe2, 0xe5, 0x1d, 0x81};
    fail_data.resize(40);
    std::mt19937 rng_fail(12345);
    for (size_t i = 0; i < 40; i++) fail_data[i] = rng_fail() & 0xFF;

    std::cout << "Pattern 1 (seed 12345, expected to fail): ";
    std::cout << (testData(cfg, fail_data) ? "PASS" : "FAIL") << "\n";

    // Test known passing pattern
    Bytes pass_data(40);
    std::mt19937 rng_pass(12345);
    // Skip 512 FFT data (40 bytes)
    for (int i = 0; i < 40; i++) rng_pass();
    for (size_t i = 0; i < 40; i++) pass_data[i] = rng_pass() & 0xFF;

    std::cout << "Pattern 2 (warmed seed, expected to pass): ";
    std::cout << (testData(cfg, pass_data) ? "PASS" : "FAIL") << "\n";

    // Test with all zeros
    Bytes zero_data(40, 0);
    std::cout << "Pattern 3 (all zeros): ";
    std::cout << (testData(cfg, zero_data) ? "PASS" : "FAIL") << "\n";

    // Test with all ones
    Bytes one_data(40, 0xFF);
    std::cout << "Pattern 4 (all 0xFF): ";
    std::cout << (testData(cfg, one_data) ? "PASS" : "FAIL") << "\n";

    // Test with different seeds
    std::cout << "\nTesting 20 random seeds:\n";
    int pass = 0, fail = 0;
    for (uint32_t seed = 0; seed < 20; seed++) {
        std::mt19937 rng(seed);
        Bytes data(40);
        for (auto& b : data) b = rng() & 0xFF;
        if (testData(cfg, data)) {
            pass++;
        } else {
            fail++;
            if (fail <= 3) {
                std::cout << "  Seed " << seed << " FAILED: ";
                for (int i = 0; i < 4; i++) std::cout << std::hex << (int)data[i] << " ";
                std::cout << std::dec << "\n";
            }
        }
    }
    std::cout << "Results: " << pass << "/20 pass (" << (100*pass/20) << "%)\n";

    return 0;
}
