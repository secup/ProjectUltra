#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <cmath>

using namespace ultra;

int main() {
    setLogLevel(LogLevel::DEBUG);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.use_pilots = false;

    printf("=== Testing varying patterns ===\n\n");

    // Test 1: All 0x00
    {
        Bytes data(81, 0x00);
        OFDMModulator mod(config);
        OFDMDemodulator demod(config);

        auto preamble = mod.generatePreamble();
        auto modulated = mod.modulate(data, Modulation::DQPSK);

        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        printf("  All0: max_val=%.4f, preamble=%zu, data=%zu, total=%zu\n",
               max_val, preamble.size(), modulated.size(), signal.size());
        for (float& s : signal) s *= 0.5f / max_val;

        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            demod.process(span);
        }

        auto soft = demod.getSoftBits();
        printf("All 0x00: soft_bits=%zu, first 8 LLRs: ", soft.size());
        for (int i = 0; i < 8 && i < (int)soft.size(); i++) printf("%.1f ", soft[i]);
        printf("\n");
    }

    // Test 2: Incrementing
    {
        Bytes data(81);
        for (size_t i = 0; i < 81; i++) data[i] = i;

        OFDMModulator mod(config);
        OFDMDemodulator demod(config);

        auto preamble = mod.generatePreamble();
        auto modulated = mod.modulate(data, Modulation::DQPSK);

        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        printf("  Incr: max_val=%.4f, preamble=%zu, data=%zu, total=%zu\n",
               max_val, preamble.size(), modulated.size(), signal.size());
        for (float& s : signal) s *= 0.5f / max_val;

        // Check guard samples
        float guard_max = 0, guard_sum = 0;
        for (size_t i = 0; i < 560; i++) {
            guard_sum += signal[i] * signal[i];
            guard_max = std::max(guard_max, std::abs(signal[i]));
        }
        float sts_energy = 0;
        for (size_t i = 560; i < 1120; i++) {
            sts_energy += signal[i] * signal[i];
        }
        printf("  Guard energy=%.9f, max=%.6f, STS_energy=%.6f\n",
               guard_sum / 560, guard_max, sts_energy / 560);

        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            demod.process(span);
        }

        auto soft = demod.getSoftBits();
        printf("Incr 0-80: soft_bits=%zu, first 8 LLRs: ", soft.size());
        for (int i = 0; i < 8 && i < (int)soft.size(); i++) printf("%.1f ", soft[i]);
        printf("\n");

        // Check first byte specifically
        printf("  First byte TX=0x00: ");
        for (int i = 0; i < 8; i++) {
            int tx_bit = 0;  // First byte is 0
            int rx_bit = soft[i] > 0 ? 0 : 1;
            printf("%d→%d%s ", tx_bit, rx_bit, tx_bit == rx_bit ? "" : "!");
        }
        printf("\n");
    }

    // Test 3: Incrementing but with first few bytes as 0
    {
        Bytes data(81);
        for (size_t i = 0; i < 81; i++) data[i] = (i < 8) ? 0 : i;  // First 8 bytes = 0

        OFDMModulator mod(config);
        OFDMDemodulator demod(config);

        auto preamble = mod.generatePreamble();
        auto modulated = mod.modulate(data, Modulation::DQPSK);

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
        printf("First8=0: soft_bits=%zu, first 8 LLRs: ", soft.size());
        for (int i = 0; i < 8 && i < (int)soft.size(); i++) printf("%.1f ", soft[i]);
        printf("\n");
    }

    return 0;
}
