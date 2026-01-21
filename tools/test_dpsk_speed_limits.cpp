/**
 * DPSK Speed Limits: How fast can single-carrier DPSK go on poor/flutter?
 */

#include "psk/dpsk.hpp"
#include "ultra/fec.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <iomanip>

using namespace ultra;
using namespace ultra::sim;

Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                   0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                   0xDE, 0xAD, 0xBE, 0xEF};

float testDPSK(DPSKConfig cfg, CodeRate rate, WattersonChannel::Config chCfg, int trials = 30) {
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    int success = 0;

    for (int t = 0; t < trials; t++) {
        WattersonChannel channel(chCfg, 42 + t);
        DPSKModulator mod(cfg);
        DPSKDemodulator demod(cfg);

        Bytes encoded = encoder.encode(test_data);
        Samples tx = mod.modulate(encoded);

        float max_val = 0;
        for (float s : tx) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx) s *= 0.5f / max_val;

        Samples rx = channel.process(SampleSpan(tx.data(), tx.size()));
        auto soft = demod.demodulateSoft(SampleSpan(rx.data(), rx.size()));

        if (soft.size() < 648) continue;
        soft.resize(648);

        float sum = 0;
        for (float f : soft) sum += std::abs(f);
        float avg = sum / soft.size();
        if (avg > 0.01f) {
            float scale = 3.5f / avg;
            for (float& f : soft) f *= scale;
        }

        Bytes decoded = decoder.decodeSoft(soft);
        if (!decoder.lastDecodeSuccess()) continue;
        decoded.resize(test_data.size());
        if (decoded == test_data) success++;
    }
    return 100.0f * success / trials;
}

int main() {
    std::cout << "=== DPSK Speed Limits on Poor/Flutter ===\n\n";

    struct Mode {
        const char* name;
        DPSKConfig (*getConfig)();
        CodeRate rate;
        float throughput;
    };

    std::vector<Mode> modes = {
        // Slow modes (most robust)
        {"DBPSK 62b R1/4", dpsk_presets::low_snr, CodeRate::R1_4, 16},
        {"DQPSK 62b R1/4", dpsk_presets::medium, CodeRate::R1_4, 31},
        {"DQPSK 125b R1/4", dpsk_presets::fast, CodeRate::R1_4, 62},
        {"DQPSK 125b R1/2", dpsk_presets::fast, CodeRate::R1_2, 125},
        // Faster modes
        {"DQPSK 300b R1/4", dpsk_presets::speed1, CodeRate::R1_4, 150},
        {"DQPSK 300b R1/2", dpsk_presets::speed1, CodeRate::R1_2, 300},
        {"DQPSK 375b R1/2", dpsk_presets::speed2, CodeRate::R1_2, 375},
    };

    std::cout << "=== POOR CHANNEL (2ms delay, 1Hz Doppler) @ 15dB ===\n\n";
    std::cout << std::setw(20) << "Mode" << std::setw(12) << "Throughput" << std::setw(10) << "Success\n";
    std::cout << std::string(42, '-') << "\n";

    auto poor = itu_r_f1487::poor(15.0f);
    for (auto& m : modes) {
        float rate = testDPSK(m.getConfig(), m.rate, poor);
        std::cout << std::setw(20) << m.name
                  << std::setw(10) << std::fixed << std::setprecision(0) << m.throughput << " bps"
                  << std::setw(9) << rate << "%\n";
    }

    std::cout << "\n=== FLUTTER CHANNEL (0.5ms delay, 10Hz Doppler) @ 15dB ===\n\n";
    std::cout << std::setw(20) << "Mode" << std::setw(12) << "Throughput" << std::setw(10) << "Success\n";
    std::cout << std::string(42, '-') << "\n";

    auto flutter = itu_r_f1487::flutter(15.0f);
    for (auto& m : modes) {
        float rate = testDPSK(m.getConfig(), m.rate, flutter);
        std::cout << std::setw(20) << m.name
                  << std::setw(10) << std::fixed << std::setprecision(0) << m.throughput << " bps"
                  << std::setw(9) << rate << "%\n";
    }

    std::cout << "\n=== OPTIMAL SELECTION ===\n";
    std::cout << "Pick highest throughput mode with >90% success rate\n";

    return 0;
}
