#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <cmath>

using namespace ultra;

// Test different bit patterns to find when first-symbol errors appear
void test_pattern(const char* name, const Bytes& data, ModemConfig& config) {
    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);

    // Modulate (data should be 81 bytes = 648 bits = 1 LDPC codeword)
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(data, Modulation::DQPSK);

    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    // Normalize signal (critical for sync detection!)
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : signal) s *= 0.5f / max_val;
    }

    // No noise - perfect channel
    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    if (soft.size() < 60) {
        printf("%-30s: NO SYNC (got %zu bits)\n", name, soft.size());
        return;
    }

    // Convert soft bits to hard bits and compare with TX
    // LLR convention: positive = bit 0, negative = bit 1
    int errors = 0;
    for (size_t i = 0; i < std::min((size_t)60, soft.size()); i++) {
        int rx_bit = soft[i] > 0 ? 0 : 1;  // Positive LLR = bit 0
        int tx_bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        if (rx_bit != tx_bit) errors++;
    }

    // Show first 30 bits TX vs RX
    printf("%-30s: %2d errors | TX:", name, errors);
    for (int i = 0; i < 30; i++) {
        int tx_bit = (data[i / 8] >> (7 - (i % 8))) & 1;
        printf("%d", tx_bit);
        if (i % 8 == 7) printf(" ");
    }
    printf(" | RX:");
    for (int i = 0; i < 30; i++) {
        int rx_bit = soft[i] > 0 ? 0 : 1;  // Positive LLR = bit 0
        printf("%d", rx_bit);
        if (i % 8 == 7) printf(" ");
    }
    printf("\n");
}

int main() {
    setLogLevel(LogLevel::WARN);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.use_pilots = false;

    printf("=== DQPSK First-Symbol Pattern Debug ===\n\n");
    printf("Testing which bit patterns cause first-symbol errors (no noise):\n\n");

    // Test patterns
    Bytes all_00(81, 0x00);
    test_pattern("All 0x00", all_00, config);

    Bytes all_FF(81, 0xFF);
    test_pattern("All 0xFF", all_FF, config);

    Bytes all_55(81, 0x55);
    test_pattern("All 0x55 (01010101)", all_55, config);

    Bytes all_AA(81, 0xAA);
    test_pattern("All 0xAA (10101010)", all_AA, config);

    // Incrementing pattern
    Bytes incr(81);
    for (size_t i = 0; i < 81; i++) incr[i] = i;
    test_pattern("Incrementing 0x00-0x50", incr, config);

    // Random-looking but fixed pattern
    Bytes pseudo(81);
    for (size_t i = 0; i < 81; i++) pseudo[i] = (i * 137 + 43) & 0xFF;
    test_pattern("Pseudo-random (i*137+43)", pseudo, config);

    // First byte different
    Bytes first_diff(81, 0x00);
    first_diff[0] = 0xFF;
    test_pattern("0xFF then all 0x00", first_diff, config);

    // First byte 0, rest random
    Bytes first_zero(81);
    for (size_t i = 0; i < 81; i++) first_zero[i] = (i == 0) ? 0x00 : ((i * 137 + 43) & 0xFF);
    test_pattern("0x00 then pseudo-random", first_zero, config);

    // Check if it's a per-carrier issue
    printf("\n=== Testing per-carrier patterns ===\n");

    // All carriers same phase change
    Bytes same_phase(81, 0x00);  // All 00 = no phase change
    test_pattern("Phase 0° all carriers", same_phase, config);

    Bytes phase_90(81, 0x55);  // All 01 = 90° change
    test_pattern("Phase 90° all carriers", phase_90, config);

    Bytes phase_180(81, 0xAA);  // All 10 = 180° change
    test_pattern("Phase 180° all carriers", phase_180, config);

    Bytes phase_270(81, 0xFF);  // All 11 = 270° change
    test_pattern("Phase 270° all carriers", phase_270, config);

    // Mixed phases in first symbol (first 8 bytes = first 64 bits = first 32 carriers = 1.07 symbols)
    printf("\n=== Testing mixed phase patterns ===\n");

    // First 4 carriers different, rest same
    Bytes mixed1(81, 0x00);
    mixed1[0] = 0x12;  // 00010010 = carrier 0: 00, carrier 1: 01, carrier 2: 00, carrier 3: 10
    test_pattern("Mixed first byte only", mixed1, config);

    // Alternating carrier phases
    Bytes alt_carrier(81);
    for (size_t i = 0; i < 81; i++) {
        // Each byte: 01 10 01 10 = 0x6A
        alt_carrier[i] = 0x6A;
    }
    test_pattern("Alt carrier phases (01 10)", alt_carrier, config);

    return 0;
}
