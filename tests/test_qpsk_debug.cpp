/**
 * Debug test to trace QPSK bit mapping through modulator/demodulator
 */
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <bitset>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// QPSK constellation for reference
constexpr float QPSK_SCALE = 0.7071067811865476f;
const Complex QPSK_MAP[] = {
    Complex(-QPSK_SCALE, -QPSK_SCALE),  // 00
    Complex(-QPSK_SCALE,  QPSK_SCALE),  // 01
    Complex( QPSK_SCALE, -QPSK_SCALE),  // 10
    Complex( QPSK_SCALE,  QPSK_SCALE),  // 11
};

int main() {
    std::cout << "=== QPSK Bit Mapping Debug ===\n\n";

    // Show QPSK constellation
    std::cout << "QPSK Constellation:\n";
    for (int i = 0; i < 4; i++) {
        std::cout << "  bits=" << ((i>>1)&1) << (i&1) << " → ("
                  << std::fixed << std::setprecision(4) << QPSK_MAP[i].real()
                  << ", " << QPSK_MAP[i].imag() << ")\n";
    }
    std::cout << "\n";

    ModemConfig config;
    config.modulation = Modulation::QPSK;
    config.pilot_spacing = 2;
    // Disable adaptive EQ to use simpler ZF
    config.adaptive_eq_enabled = false;

    OFDMModulator tx(config);
    OFDMDemodulator rx(config);

    // Test with multiple bytes of 0xAA
    Bytes test_data(81, 0xAA);  // 81 bytes like the basic test

    std::cout << "Input byte: 0x" << std::hex << (int)test_data[0] << std::dec
              << " = " << std::bitset<8>(test_data[0]) << "\n";
    std::cout << "Expected QPSK symbols (2 bits each):\n";
    for (int i = 0; i < 4; i++) {
        int bits = (test_data[0] >> (6 - 2*i)) & 0x3;
        std::cout << "  Symbol " << i << ": bits=" << bits
                  << " (" << ((bits>>1)&1) << ((bits)&1) << ")\n";
    }
    std::cout << "\n";

    // Generate signal
    Samples preamble = tx.generatePreamble();
    Samples data = tx.modulate(test_data, Modulation::QPSK);

    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), data.begin(), data.end());

    // Normalize
    float maxv = 0;
    for (float s : signal) maxv = std::max(maxv, std::abs(s));
    if (maxv > 0) for (float& s : signal) s *= 0.5f / maxv;

    std::cout << "TX signal: " << signal.size() << " samples\n";

    // Feed to RX
    SampleSpan span(signal.data(), signal.size());
    rx.process(span);

    if (!rx.isSynced()) {
        std::cout << "[FAIL] No sync!\n";
        return 1;
    }
    std::cout << "[OK] Synced\n\n";

    // Get soft bits
    auto soft = rx.getSoftBits();
    std::cout << "Received " << soft.size() << " soft bits (LLRs)\n\n";

    std::cout << "Soft bit analysis (first 16 LLRs):\n";
    std::cout << "  LLR > 0 → bit 0, LLR < 0 → bit 1\n\n";

    for (size_t i = 0; i < std::min(soft.size(), (size_t)16); i++) {
        int hard_bit = (soft[i] < 0) ? 1 : 0;
        std::cout << "  LLR[" << std::setw(2) << i << "] = "
                  << std::setw(8) << std::fixed << std::setprecision(3) << soft[i]
                  << " → bit " << hard_bit;
        if (i % 2 == 1) {
            // End of QPSK symbol
            int prev_bit = (soft[i-1] < 0) ? 1 : 0;
            int sym_idx = i / 2;
            std::cout << "  | Symbol " << sym_idx << " = " << prev_bit << hard_bit;

            // What was expected?
            int expected_bits = (0xAA >> (6 - 2*sym_idx)) & 0x3;
            int expected_b1 = (expected_bits >> 1) & 1;
            int expected_b0 = expected_bits & 1;

            if (prev_bit == expected_b1 && hard_bit == expected_b0) {
                std::cout << " ✓";
            } else {
                std::cout << " ✗ (expected " << expected_b1 << expected_b0 << ")";
            }
        }
        std::cout << "\n";
    }

    // Convert all to bytes
    std::cout << "\nDecoded bytes:\n";
    uint8_t byte = 0;
    int bit_count = 0;
    for (size_t i = 0; i < soft.size(); i++) {
        uint8_t bit = (soft[i] < 0) ? 1 : 0;
        byte = (byte << 1) | bit;
        if (++bit_count == 8) {
            std::cout << "  Byte: 0x" << std::hex << (int)byte << std::dec
                      << " = " << std::bitset<8>(byte) << "\n";
            byte = 0;
            bit_count = 0;
        }
    }

    return 0;
}
