/**
 * Compare test_nvis_data style vs test_nvis_mode style
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

// Style 1: Clean config (like test_nvis_data)
bool testClean(uint32_t seed) {
    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;
    cfg.symbol_guard = 0;
    cfg.pilot_spacing = 2;
    cfg.modulation = Modulation::DQPSK;
    cfg.code_rate = CodeRate::R1_2;
    cfg.use_pilots = false;

    std::mt19937 rng(seed);
    Bytes data(40);
    for (auto& b : data) b = rng() & 0xFF;

    OFDMModulator modulator(cfg);
    OFDMDemodulator demod(cfg);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);

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
    if (soft.size() < 648) return false;

    std::span<const float> llrs(soft.data(), 648);
    Bytes decoded = decoder.decodeSoft(llrs);

    if (!decoder.lastDecodeSuccess() || decoded.size() < data.size()) return false;
    for (size_t i = 0; i < data.size(); i++) {
        if (data[i] != decoded[i]) return false;
    }
    return true;
}

// Style 2: nvis_mode() preset (like test_nvis_mode)
bool testNvisPreset(uint32_t seed) {
    ModemConfig cfg = presets::nvis_mode();
    cfg.use_pilots = false;
    cfg.modulation = Modulation::DQPSK;
    cfg.code_rate = CodeRate::R1_2;

    std::mt19937 rng(seed);
    Bytes data(40);
    for (auto& b : data) b = rng() & 0xFF;

    OFDMModulator modulator(cfg);
    OFDMDemodulator demod(cfg);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);

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
    if (soft.size() < 648) return false;

    std::span<const float> llrs(soft.data(), 648);
    Bytes decoded = decoder.decodeSoft(llrs);

    if (!decoder.lastDecodeSuccess() || decoded.size() < data.size()) return false;
    for (size_t i = 0; i < data.size(); i++) {
        if (data[i] != decoded[i]) return false;
    }
    return true;
}

int main() {
    setLogLevel(LogLevel::WARN);

    std::cout << "Testing 20 seeds with both config styles:\n\n";

    int clean_pass = 0, preset_pass = 0;
    for (uint32_t seed = 0; seed < 20; seed++) {
        bool c = testClean(seed);
        bool p = testNvisPreset(seed);
        if (c) clean_pass++;
        if (p) preset_pass++;
        if (c != p) {
            std::cout << "  Seed " << seed << ": clean=" << c << ", preset=" << p << "\n";
        }
    }

    std::cout << "Clean config:  " << clean_pass << "/20\n";
    std::cout << "NVIS preset:   " << preset_pass << "/20\n";

    return 0;
}
