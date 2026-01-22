#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <cmath>

using namespace ultra;

int main() {
    setLogLevel(LogLevel::DEBUG);  // Enable debug logging

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.use_pilots = false;

    printf("=== DQPSK Detailed Trace ===\n\n");

    // Test with all zeros (phase 0° on all carriers)
    Bytes data(81, 0x00);
    printf("TX data: all 0x00 (DQPSK phase 0° = bits 00 for each carrier pair)\n\n");

    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);

    // Modulate
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(data, Modulation::DQPSK);

    printf("Preamble samples: %zu\n", preamble.size());
    printf("Modulated data samples: %zu\n", modulated.size());

    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    // Normalize
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    printf("Total signal samples: %zu\n\n", signal.size());

    // Process through demodulator
    printf("Processing through demodulator...\n\n");
    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    printf("\nGot %zu soft bits\n", soft.size());

    if (soft.size() >= 60) {
        printf("\nFirst symbol analysis (30 carriers × 2 bits = 60 bits):\n");
        printf("Carrier  TX_bits  RX_LLRs          RX_bits  Match\n");
        printf("-------  -------  ---------------  -------  -----\n");

        int errors = 0;
        for (int c = 0; c < 30; c++) {
            int bit_idx = c * 2;
            // TX: all zeros means bits 00 for each carrier
            int tx_bit0 = 0;
            int tx_bit1 = 0;

            float llr0 = soft[bit_idx];
            float llr1 = soft[bit_idx + 1];
            int rx_bit0 = llr0 > 0 ? 0 : 1;
            int rx_bit1 = llr1 > 0 ? 0 : 1;

            bool match = (tx_bit0 == rx_bit0) && (tx_bit1 == rx_bit1);
            if (!match) errors++;

            printf("  %2d     %d%d       %+6.2f %+6.2f    %d%d       %s\n",
                   c, tx_bit0, tx_bit1, llr0, llr1, rx_bit0, rx_bit1,
                   match ? "OK" : "ERR");
        }
        printf("\nTotal errors in first symbol: %d/60\n", errors);

        // Also show what phase the RX sees
        printf("\nDerived phase angles from LLRs (should all be ~0° for 0x00 input):\n");
        for (int c = 0; c < 10; c++) {
            int bit_idx = c * 2;
            float llr0 = soft[bit_idx];
            float llr1 = soft[bit_idx + 1];

            // Reverse the LLR formulas to estimate phase
            // llr0 = scale * sin(phase + π/4)
            // llr1 = scale * cos(2*phase)
            // For now just show the raw LLR pattern
            int rx_bits = (llr0 > 0 ? 0 : 1) * 2 + (llr1 > 0 ? 0 : 1);
            const char* phase_str = (rx_bits == 0) ? "0°" :
                                    (rx_bits == 1) ? "90°" :
                                    (rx_bits == 2) ? "180°" : "270°";
            printf("  Carrier %d: bits=%d%d → %s\n", c,
                   (llr0 > 0 ? 0 : 1), (llr1 > 0 ? 0 : 1), phase_str);
        }
    }

    return 0;
}
