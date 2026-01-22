// Deep debug of first-symbol DQPSK differential decoding
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

int main() {
    // Enable debug logging to see what's happening
    setLogLevel(LogLevel::DEBUG);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.use_pilots = false;

    printf("=== First Symbol Debug ===\n\n");

    // Use a simple test pattern instead of LDPC-encoded data
    // This makes it easier to trace TX vs RX bits

    // Test pattern: all zeros (should be 0° phase change on all carriers)
    printf("=== Test 1: All zeros ===\n");
    {
        OFDMModulator modulator(config);
        OFDMDemodulator demod(config);

        // 648 bits = 81 bytes, all zeros
        Bytes data(81, 0x00);

        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(data, Modulation::DQPSK);

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

        int errors = 0;
        for (size_t i = 0; i < std::min((size_t)648, soft.size()); i++) {
            int tx_bit = 0;  // All zeros
            int rx_bit = soft[i] > 0 ? 0 : 1;
            if (tx_bit != rx_bit) errors++;
        }
        printf("Bit errors: %d / 648\n\n", errors);
    }

    // Test pattern: incrementing bytes (0x00, 0x01, 0x02, ...)
    printf("=== Test 2: Incrementing bytes ===\n");
    {
        OFDMModulator modulator(config);
        OFDMDemodulator demod(config);

        Bytes data(81);
        for (size_t i = 0; i < 81; i++) data[i] = i & 0xFF;

        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(data, Modulation::DQPSK);

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

        int total_errors = 0;
        printf("First 60 bits (symbol 0) comparison:\n");
        for (int i = 0; i < 60 && i < (int)soft.size(); i++) {
            int tx_bit = (data[i / 8] >> (7 - (i % 8))) & 1;
            int rx_bit = soft[i] > 0 ? 0 : 1;
            if (i < 20) {
                printf("  bit %2d: TX=%d RX=%d LLR=%6.2f %s\n",
                       i, tx_bit, rx_bit, soft[i],
                       tx_bit != rx_bit ? "ERROR" : "");
            }
            if (tx_bit != rx_bit) total_errors++;
        }

        printf("\nTotal errors in first 60 bits: %d\n\n", total_errors);
    }

    // Analyze with logging enabled to see demod differential processing
    setLogLevel(LogLevel::DEBUG);

    printf("=== Test 3: Single byte 0x01 (rest zeros) ===\n");
    {
        OFDMModulator modulator(config);
        OFDMDemodulator demod(config);

        // 0x01 = 00000001 in binary
        // First 4 dibits: 00, 00, 00, 01
        // Expected phase changes: 0°, 0°, 0°, 90°
        Bytes data(81, 0x00);
        data[0] = 0x01;

        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(data, Modulation::DQPSK);

        printf("TX first byte = 0x%02X = ", data[0]);
        for (int i = 7; i >= 0; i--) printf("%d", (data[0] >> i) & 1);
        printf("\n");
        printf("Expected dibits: ");
        for (int i = 0; i < 4; i++) {
            int dibit = (data[0] >> (6 - 2*i)) & 3;
            printf("%d%d ", dibit >> 1, dibit & 1);
        }
        printf("\n\n");

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

        printf("First 16 bits:\n");
        for (int i = 0; i < 16 && i < (int)soft.size(); i++) {
            int tx_bit = (data[i / 8] >> (7 - (i % 8))) & 1;
            int rx_bit = soft[i] > 0 ? 0 : 1;
            printf("  bit %2d: TX=%d RX=%d LLR=%7.2f %s\n",
                   i, tx_bit, rx_bit, soft[i],
                   tx_bit != rx_bit ? "ERROR" : "");
        }
    }

    return 0;
}
