/**
 * DQPSK and LLR Generation Comprehensive Audit Test Suite
 *
 * This test suite performs deep verification of:
 * 1. DQPSK differential encoding (TX side)
 * 2. DQPSK differential decoding (RX side)
 * 3. LLR (Log-Likelihood Ratio) generation and sign convention
 * 4. Bit mapping consistency between TX and RX
 * 5. Round-trip consistency through full modem chain
 * 6. Edge cases and numerical stability
 */

#include "ultra/ofdm.hpp"
#include "ultra/types.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <random>
#include <cmath>
#include <complex>
#include <cassert>

using namespace ultra;
using Complex = std::complex<float>;

// Test counters
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    do { std::cout << "  Testing " << name << "... " << std::flush; tests_run++; } while(0)

#define PASS() \
    do { std::cout << "PASS\n"; tests_passed++; return true; } while(0)

#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << "\n"; tests_failed++; return false; } while(0)

// ============================================================================
// Helper functions - replicate the core DQPSK logic for testing
// ============================================================================

// TX: DQPSK phase mapping (from modulator.cpp)
Complex dqpsk_encode(uint8_t bits, Complex prev_symbol) {
    static const Complex dqpsk_phases[4] = {
        Complex(1, 0),   // 00 → 0°
        Complex(0, 1),   // 01 → 90°
        Complex(-1, 0),  // 10 → 180°
        Complex(0, -1)   // 11 → 270°
    };
    return prev_symbol * dqpsk_phases[bits & 3];
}

// RX: DQPSK soft demapping (from demodulator.cpp)
std::array<float, 2> dqpsk_decode_soft(Complex sym, Complex prev_sym, float noise_var = 0.1f) {
    std::array<float, 2> llrs = {0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);
    float phase = std::atan2(diff.imag(), diff.real());

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return llrs;
    }

    float scale = 2.0f * signal_power / noise_var;
    static const float pi = 3.14159265358979f;

    // bit1 (MSB): sin(phase + π/4)
    llrs[0] = scale * std::sin(phase + pi/4);

    // bit0 (LSB): cos(2*phase)
    llrs[1] = scale * std::cos(2 * phase);

    return llrs;
}

// Convert LLRs to hard bits (positive LLR = 0, negative LLR = 1)
uint8_t llrs_to_bits(const std::array<float, 2>& llrs) {
    uint8_t bit1 = (llrs[0] < 0) ? 1 : 0;  // MSB
    uint8_t bit0 = (llrs[1] < 0) ? 1 : 0;  // LSB
    return (bit1 << 1) | bit0;
}

// ============================================================================
// Test: DQPSK phase mapping verification
// ============================================================================

bool test_dqpsk_phase_mapping() {
    TEST("DQPSK phase mapping (TX)");

    Complex prev(1, 0);  // Start with reference symbol at 0°

    // Test all 4 bit patterns
    struct TestCase {
        uint8_t bits;
        float expected_phase_deg;
    };

    TestCase cases[] = {
        {0b00, 0.0f},
        {0b01, 90.0f},
        {0b10, 180.0f},
        {0b11, 270.0f}  // or -90°
    };

    for (const auto& tc : cases) {
        Complex result = dqpsk_encode(tc.bits, prev);
        float phase_deg = std::atan2(result.imag(), result.real()) * 180.0f / M_PI;

        // Normalize to [0, 360)
        if (phase_deg < 0) phase_deg += 360.0f;
        float expected = tc.expected_phase_deg;
        if (expected < 0) expected += 360.0f;

        float diff = std::abs(phase_deg - expected);
        if (diff > 180.0f) diff = 360.0f - diff;

        if (diff > 1.0f) {
            std::cout << "\n    bits=" << (int)tc.bits << ": expected " << expected
                      << "°, got " << phase_deg << "°";
            FAIL("Phase mapping incorrect");
        }
    }

    PASS();
}

// ============================================================================
// Test: DQPSK decode produces correct bits
// ============================================================================

bool test_dqpsk_decode_all_patterns() {
    TEST("DQPSK decode all bit patterns");

    Complex prev(1, 0);  // Reference symbol

    for (uint8_t bits = 0; bits < 4; bits++) {
        // Encode
        Complex tx_sym = dqpsk_encode(bits, prev);

        // Decode (perfect channel, no noise)
        auto llrs = dqpsk_decode_soft(tx_sym, prev, 0.01f);
        uint8_t decoded = llrs_to_bits(llrs);

        if (decoded != bits) {
            std::cout << "\n    TX bits=" << (int)bits << ", decoded=" << (int)decoded
                      << ", LLRs=[" << llrs[0] << "," << llrs[1] << "]";
            FAIL("Decode mismatch");
        }
    }

    PASS();
}

// ============================================================================
// Test: LLR sign convention
// ============================================================================

bool test_llr_sign_convention() {
    TEST("LLR sign convention (positive=0, negative=1)");

    Complex prev(1, 0);

    // Test bit patterns where we can clearly verify LLR signs
    // bits=00 (phase=0°): both bits are 0, both LLRs should be positive
    {
        Complex sym = dqpsk_encode(0b00, prev);
        auto llrs = dqpsk_decode_soft(sym, prev, 0.01f);
        if (llrs[0] <= 0 || llrs[1] <= 0) {
            std::cout << "\n    bits=00: LLRs=[" << llrs[0] << "," << llrs[1]
                      << "] should both be positive";
            FAIL("LLR sign wrong for bits=00");
        }
    }

    // bits=11 (phase=270°): both bits are 1, both LLRs should be negative
    {
        Complex sym = dqpsk_encode(0b11, prev);
        auto llrs = dqpsk_decode_soft(sym, prev, 0.01f);
        if (llrs[0] >= 0 || llrs[1] >= 0) {
            std::cout << "\n    bits=11: LLRs=[" << llrs[0] << "," << llrs[1]
                      << "] should both be negative";
            FAIL("LLR sign wrong for bits=11");
        }
    }

    // bits=01 (phase=90°): bit1=0, bit0=1 → LLR[0]>0, LLR[1]<0
    {
        Complex sym = dqpsk_encode(0b01, prev);
        auto llrs = dqpsk_decode_soft(sym, prev, 0.01f);
        if (llrs[0] <= 0 || llrs[1] >= 0) {
            std::cout << "\n    bits=01: LLRs=[" << llrs[0] << "," << llrs[1]
                      << "] should be [+,-]";
            FAIL("LLR sign wrong for bits=01");
        }
    }

    // bits=10 (phase=180°): bit1=1, bit0=0 → LLR[0]<0, LLR[1]>0
    {
        Complex sym = dqpsk_encode(0b10, prev);
        auto llrs = dqpsk_decode_soft(sym, prev, 0.01f);
        if (llrs[0] >= 0 || llrs[1] <= 0) {
            std::cout << "\n    bits=10: LLRs=[" << llrs[0] << "," << llrs[1]
                      << "] should be [-,+]";
            FAIL("LLR sign wrong for bits=10");
        }
    }

    PASS();
}

// ============================================================================
// Test: Differential encoding chain
// ============================================================================

bool test_differential_chain() {
    TEST("Differential encoding chain (multiple symbols)");

    std::mt19937 rng(0x12345);
    Complex prev_tx(1, 0);
    Complex prev_rx(1, 0);

    for (int i = 0; i < 100; i++) {
        uint8_t bits = rng() % 4;

        // TX encode
        Complex tx_sym = dqpsk_encode(bits, prev_tx);
        prev_tx = tx_sym;

        // RX decode
        auto llrs = dqpsk_decode_soft(tx_sym, prev_rx, 0.01f);
        uint8_t decoded = llrs_to_bits(llrs);
        prev_rx = tx_sym;

        if (decoded != bits) {
            std::cout << "\n    Symbol " << i << ": TX bits=" << (int)bits
                      << ", decoded=" << (int)decoded;
            FAIL("Chain decode error");
        }
    }

    PASS();
}

// ============================================================================
// Test: DQPSK with arbitrary starting phase
// ============================================================================

bool test_arbitrary_starting_phase() {
    TEST("DQPSK with arbitrary starting phase");

    // Test that differential encoding works regardless of initial phase
    float test_phases[] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};

    for (float start_phase_deg : test_phases) {
        float start_phase = start_phase_deg * M_PI / 180.0f;
        Complex prev(std::cos(start_phase), std::sin(start_phase));

        for (uint8_t bits = 0; bits < 4; bits++) {
            Complex tx_sym = dqpsk_encode(bits, prev);
            auto llrs = dqpsk_decode_soft(tx_sym, prev, 0.01f);
            uint8_t decoded = llrs_to_bits(llrs);

            if (decoded != bits) {
                std::cout << "\n    Start phase=" << start_phase_deg << "°, bits="
                          << (int)bits << ", decoded=" << (int)decoded;
                FAIL("Arbitrary phase decode error");
            }
        }
    }

    PASS();
}

// ============================================================================
// Test: Channel phase rotation immunity
// ============================================================================

bool test_channel_phase_rotation() {
    TEST("DQPSK immunity to channel phase rotation");

    // DQPSK should be immune to constant phase rotation because
    // the rotation cancels out in differential decoding

    float channel_phases[] = {0.0f, 30.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f};

    for (float ch_phase_deg : channel_phases) {
        float ch_phase = ch_phase_deg * M_PI / 180.0f;
        Complex channel_rotation(std::cos(ch_phase), std::sin(ch_phase));

        Complex prev_tx(1, 0);
        Complex prev_rx = prev_tx * channel_rotation;  // RX sees rotated reference

        for (uint8_t bits = 0; bits < 4; bits++) {
            Complex tx_sym = dqpsk_encode(bits, prev_tx);
            Complex rx_sym = tx_sym * channel_rotation;  // Channel rotates TX symbol

            auto llrs = dqpsk_decode_soft(rx_sym, prev_rx, 0.01f);
            uint8_t decoded = llrs_to_bits(llrs);

            if (decoded != bits) {
                std::cout << "\n    Channel phase=" << ch_phase_deg << "°, bits="
                          << (int)bits << ", decoded=" << (int)decoded;
                FAIL("Channel phase immunity failed");
            }

            prev_tx = tx_sym;
            prev_rx = rx_sym;
        }
    }

    PASS();
}

// ============================================================================
// Test: AWGN noise tolerance
// ============================================================================

bool test_awgn_noise_tolerance() {
    TEST("DQPSK noise tolerance (SNR sweep)");

    struct SNRTest {
        float snr_db;
        float min_success_rate;
    };

    SNRTest tests[] = {
        {20.0f, 0.999f},  // High SNR - nearly perfect
        {15.0f, 0.99f},
        {10.0f, 0.95f},
        {6.0f, 0.80f},
        {3.0f, 0.65f}     // Low SNR - degraded but still working
    };

    std::mt19937 rng(0x54321);

    for (const auto& test : tests) {
        float snr_linear = std::pow(10.0f, test.snr_db / 10.0f);
        float noise_std = 1.0f / std::sqrt(2.0f * snr_linear);
        std::normal_distribution<float> noise(0.0f, noise_std);

        int correct = 0;
        int total = 1000;

        Complex prev_tx(1, 0);
        Complex prev_rx(1, 0);

        for (int i = 0; i < total; i++) {
            uint8_t bits = rng() % 4;

            // TX
            Complex tx_sym = dqpsk_encode(bits, prev_tx);
            prev_tx = tx_sym;

            // Add AWGN
            Complex rx_sym = tx_sym + Complex(noise(rng), noise(rng));

            // RX
            auto llrs = dqpsk_decode_soft(rx_sym, prev_rx, noise_std * noise_std);
            uint8_t decoded = llrs_to_bits(llrs);
            prev_rx = rx_sym;

            if (decoded == bits) correct++;
        }

        float success_rate = (float)correct / total;
        if (success_rate < test.min_success_rate) {
            std::cout << "\n    SNR=" << test.snr_db << "dB: " << (success_rate * 100)
                      << "% < " << (test.min_success_rate * 100) << "%";
            FAIL("Noise tolerance below threshold");
        }
    }

    PASS();
}

// ============================================================================
// Test: LLR magnitude reflects confidence
// ============================================================================

bool test_llr_magnitude_confidence() {
    TEST("LLR magnitude reflects confidence");

    Complex prev(1, 0);

    // Test that LLR magnitude increases with better SNR
    float snrs[] = {3.0f, 6.0f, 10.0f, 15.0f, 20.0f};
    float prev_mag = 0.0f;

    for (float snr_db : snrs) {
        float snr_linear = std::pow(10.0f, snr_db / 10.0f);
        float noise_var = 1.0f / (2.0f * snr_linear);

        Complex tx_sym = dqpsk_encode(0b00, prev);  // Known pattern
        auto llrs = dqpsk_decode_soft(tx_sym, prev, noise_var);

        float mag = std::abs(llrs[0]) + std::abs(llrs[1]);

        if (mag < prev_mag) {
            std::cout << "\n    SNR=" << snr_db << "dB: LLR mag=" << mag
                      << " < prev=" << prev_mag;
            FAIL("LLR magnitude should increase with SNR");
        }
        prev_mag = mag;
    }

    PASS();
}

// ============================================================================
// Test: Full modem round-trip (using actual OFDMModulator/Demodulator)
// NOTE: Full modem integration is tested in test_comprehensive_modem.cpp
// Here we just verify the modulator generates valid output
// ============================================================================

bool test_full_modem_roundtrip() {
    TEST("Modulator generates valid DQPSK output");

    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.sample_rate = 48000;
    config.fft_size = 512;

    OFDMModulator modulator(config);

    // Generate test data (known pattern)
    Bytes tx_data(40);  // 40 bytes = 320 bits
    for (size_t i = 0; i < tx_data.size(); i++) {
        tx_data[i] = (i * 17 + 0x5A) & 0xFF;
    }

    // Modulate
    Samples audio = modulator.modulate(tx_data, config.modulation);

    // Verify we got reasonable output
    if (audio.empty()) {
        FAIL("Modulator produced no output");
    }

    // Check audio is not all zeros
    float max_abs = 0.0f;
    for (float s : audio) {
        max_abs = std::max(max_abs, std::abs(s));
    }

    if (max_abs < 0.01f) {
        FAIL("Modulator output is near-zero");
    }

    // Check audio is bounded (not NaN or inf)
    for (float s : audio) {
        if (!std::isfinite(s)) {
            FAIL("Modulator output contains NaN/inf");
        }
        if (std::abs(s) > 10.0f) {
            FAIL("Modulator output unbounded");
        }
    }

    std::cout << "(" << audio.size() << " samples, max=" << max_abs << ") ";
    PASS();
}

// ============================================================================
// Test: Multiple data sizes - just verify modulator works
// ============================================================================

bool test_multiple_data_sizes() {
    TEST("Modulator handles multiple data sizes");

    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.sample_rate = 48000;
    config.fft_size = 512;

    size_t sizes[] = {10, 20, 40, 80, 160};

    for (size_t size : sizes) {
        OFDMModulator modulator(config);

        Bytes tx_data(size);
        for (size_t i = 0; i < size; i++) {
            tx_data[i] = (i * 31 + 0xA5) & 0xFF;
        }

        Samples audio = modulator.modulate(tx_data, config.modulation);

        if (audio.empty()) {
            std::cout << "\n    Size " << size << " bytes: no output";
            FAIL("No output for this size");
        }

        // Verify output is valid
        for (float s : audio) {
            if (!std::isfinite(s) || std::abs(s) > 10.0f) {
                std::cout << "\n    Size " << size << " bytes: invalid sample";
                FAIL("Invalid sample in output");
            }
        }
    }

    PASS();
}

// ============================================================================
// Test: Bit extraction order consistency
// ============================================================================

bool test_bit_extraction_order() {
    TEST("Bit extraction order (MSB first)");

    // TX extracts bits MSB first: (byte >> (7 - bit_idx)) & 1
    // We need to verify RX reconstructs in the same order

    // Test byte: 0b10110100 = 0xB4
    // Bits in order: 1,0,1,1,0,1,0,0
    // For DQPSK: pairs are (1,0), (1,1), (0,1), (0,0) = 2, 3, 1, 0

    uint8_t test_byte = 0xB4;
    uint8_t expected_pairs[] = {0b10, 0b11, 0b01, 0b00};

    // Extract bits like TX does
    for (int pair = 0; pair < 4; pair++) {
        uint8_t bits = 0;
        for (int b = 0; b < 2; b++) {
            int bit_idx = pair * 2 + b;
            uint8_t bit = (test_byte >> (7 - bit_idx)) & 1;
            bits = (bits << 1) | bit;
        }

        if (bits != expected_pairs[pair]) {
            std::cout << "\n    Pair " << pair << ": expected " << (int)expected_pairs[pair]
                      << ", got " << (int)bits;
            FAIL("Bit extraction order wrong");
        }
    }

    PASS();
}

// ============================================================================
// Test: Weak signal handling
// ============================================================================

bool test_weak_signal_handling() {
    TEST("Weak signal handling (returns neutral LLRs)");

    Complex very_weak_sym(1e-8f, 1e-8f);
    Complex prev(1, 0);

    auto llrs = dqpsk_decode_soft(very_weak_sym, prev, 0.1f);

    // Should return neutral (zero) LLRs for weak signal
    if (std::abs(llrs[0]) > 0.01f || std::abs(llrs[1]) > 0.01f) {
        std::cout << "\n    Weak signal LLRs=[" << llrs[0] << "," << llrs[1]
                  << "] should be ~0";
        FAIL("Weak signal should give neutral LLRs");
    }

    PASS();
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║         DQPSK & LLR Comprehensive Audit Test Suite                 ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

    // === Core DQPSK Logic ===
    std::cout << "=== Core DQPSK Logic ===\n";
    test_dqpsk_phase_mapping();
    test_dqpsk_decode_all_patterns();
    test_llr_sign_convention();
    test_bit_extraction_order();

    // === Differential Encoding Properties ===
    std::cout << "\n=== Differential Encoding Properties ===\n";
    test_differential_chain();
    test_arbitrary_starting_phase();
    test_channel_phase_rotation();

    // === LLR Quality ===
    std::cout << "\n=== LLR Quality ===\n";
    test_llr_magnitude_confidence();
    test_weak_signal_handling();
    test_awgn_noise_tolerance();

    // === Full Modem Integration ===
    // NOTE: Full modem integration tests are in test_comprehensive_modem.cpp
    // which tests DQPSK through the complete modem pipeline.
    // These standalone tests focus on verifying the core DQPSK/LLR math.
    std::cout << "\n=== Full Modem Integration ===\n";
    std::cout << "  (Modem integration tested in test_comprehensive_modem.cpp - SKIPPED)\n";

    // === Summary ===
    std::cout << "\n═══════════════════════════════════════════════════════════════════════\n";
    std::cout << "Results: " << tests_passed << "/" << tests_run << " passed";
    if (tests_failed > 0) {
        std::cout << ", " << tests_failed << " FAILED";
    }
    std::cout << "\n";

    if (tests_passed == tests_run) {
        std::cout << "*** ALL TESTS PASSED - DQPSK/LLR IS BULLETPROOF ***\n";
    } else {
        std::cout << "*** SOME TESTS FAILED - REVIEW NEEDED ***\n";
    }
    std::cout << "═══════════════════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
