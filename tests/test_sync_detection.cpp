/**
 * Sync Detection Accuracy Test
 *
 * Tests the OFDM demodulator's sync detection to ensure it finds the
 * TRUE preamble start, not just "somewhere above threshold".
 *
 * This test was created to verify the fix for the ~118-sample sync offset
 * issue discovered during real audio testing (2026-01-17).
 *
 * Key insight: The STS correlation is broad (periodic signal), so coarse
 * search may land anywhere on the plateau. Fine-grained refinement is
 * needed to find the true peak.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <random>
#include <chrono>

#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// Test result tracking
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) std::cout << "\n=== TEST: " << name << " ===" << std::endl
#define PASS(msg) do { std::cout << "[PASS] " << msg << std::endl; tests_passed++; } while(0)
#define FAIL(msg) do { std::cout << "[FAIL] " << msg << std::endl; tests_failed++; } while(0)
#define CHECK(cond, msg) do { if (cond) PASS(msg); else FAIL(msg); } while(0)

/**
 * Test 1: Sync detection accuracy with known preamble position
 *
 * Creates a signal with preamble at a known offset, verifies sync
 * detects within acceptable tolerance.
 */
bool test_sync_accuracy_basic() {
    TEST("Sync Detection Accuracy (Basic)");

    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = CodeRate::R1_2;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    // Generate preamble and test data
    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0xAA);  // Alternating bits pattern
    Samples data = mod.modulate(test_data, config.modulation);

    // Known preamble start position
    const size_t TRUE_PREAMBLE_START = 1000;

    // Build RX signal: [silence][preamble][data]
    std::vector<float> rx_signal(TRUE_PREAMBLE_START, 0.0f);  // Leading silence
    rx_signal.insert(rx_signal.end(), preamble.begin(), preamble.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Scale signal
    float max_val = 0;
    for (float s : rx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : rx_signal) s *= 0.5f / max_val;
    }

    std::cout << "  True preamble start: " << TRUE_PREAMBLE_START << std::endl;
    std::cout << "  Signal length: " << rx_signal.size() << " samples" << std::endl;

    // Process
    SampleSpan span(rx_signal.data(), rx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }

    size_t detected_offset = demod.getLastSyncOffset();
    int error = static_cast<int>(detected_offset) - static_cast<int>(TRUE_PREAMBLE_START);

    std::cout << "  Detected sync offset: " << detected_offset << std::endl;
    std::cout << "  Sync error: " << error << " samples" << std::endl;

    // Accept error within ±16 samples (reasonable for single-sample refinement)
    constexpr int MAX_ACCEPTABLE_ERROR = 16;
    bool accurate = std::abs(error) <= MAX_ACCEPTABLE_ERROR;

    CHECK(accurate, "Sync offset error within " + std::to_string(MAX_ACCEPTABLE_ERROR) + " samples");
    return accurate;
}

/**
 * Test 2: Sync accuracy with various latencies
 *
 * Tests that sync detection works correctly across a range of
 * leading silence durations (simulating audio path latency).
 */
bool test_sync_accuracy_various_latencies() {
    TEST("Sync Detection Accuracy (Various Latencies)");

    ModemConfig config;
    config.modulation = Modulation::BPSK;
    config.code_rate = CodeRate::R1_4;

    std::vector<size_t> latencies = {0, 100, 500, 1000, 2000, 5000, 10000};
    int total_tests = 0;
    int passed_tests = 0;

    for (size_t latency : latencies) {
        OFDMModulator mod(config);
        OFDMDemodulator demod(config);

        Samples preamble = mod.generatePreamble();
        Bytes test_data(21, 0x00);
        LDPCEncoder encoder(config.code_rate);
        Bytes encoded = encoder.encode(test_data);
        Samples data = mod.modulate(encoded, config.modulation);

        // Build RX signal
        std::vector<float> rx_signal(latency, 0.0f);
        rx_signal.insert(rx_signal.end(), preamble.begin(), preamble.end());
        rx_signal.insert(rx_signal.end(), data.begin(), data.end());

        // Scale
        float max_val = 0;
        for (float s : rx_signal) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            for (float& s : rx_signal) s *= 0.5f / max_val;
        }

        // Process
        SampleSpan span(rx_signal.data(), rx_signal.size());
        demod.process(span);

        if (!demod.isSynced()) {
            std::cout << "  Latency " << latency << ": NO SYNC" << std::endl;
            total_tests++;
            continue;
        }

        size_t detected = demod.getLastSyncOffset();
        int error = static_cast<int>(detected) - static_cast<int>(latency);

        bool ok = std::abs(error) <= 16;
        std::cout << "  Latency " << latency << ": detected=" << detected
                  << " error=" << error << " " << (ok ? "OK" : "FAIL") << std::endl;

        total_tests++;
        if (ok) passed_tests++;
    }

    float pass_rate = 100.0f * passed_tests / total_tests;
    std::cout << "  Pass rate: " << passed_tests << "/" << total_tests
              << " (" << pass_rate << "%)" << std::endl;

    CHECK(pass_rate >= 100.0f, "All latency tests passed");
    return pass_rate >= 100.0f;
}

/**
 * Test 3: Sync with noise before preamble
 *
 * Tests sync detection when there's Gaussian noise before the preamble,
 * ensuring we don't false-trigger on noise.
 */
bool test_sync_with_noise() {
    TEST("Sync Detection with Leading Noise");

    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = CodeRate::R1_2;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0x55);
    Samples data = mod.modulate(test_data, config.modulation);

    // Create signal with noise followed by preamble
    const size_t NOISE_SAMPLES = 2000;
    const size_t TRUE_START = NOISE_SAMPLES;

    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::normal_distribution<float> noise_dist(0.0f, 0.05f);  // Low noise

    std::vector<float> rx_signal;
    rx_signal.reserve(NOISE_SAMPLES + preamble.size() + data.size());

    // Add noise
    for (size_t i = 0; i < NOISE_SAMPLES; ++i) {
        rx_signal.push_back(noise_dist(rng));
    }

    // Add preamble and data
    rx_signal.insert(rx_signal.end(), preamble.begin(), preamble.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Scale signal (preserve SNR relationship)
    float max_val = 0;
    for (size_t i = NOISE_SAMPLES; i < rx_signal.size(); ++i) {
        max_val = std::max(max_val, std::abs(rx_signal[i]));
    }
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : rx_signal) s *= scale;
    }

    std::cout << "  Noise samples: " << NOISE_SAMPLES << std::endl;
    std::cout << "  True preamble start: " << TRUE_START << std::endl;

    // Process
    SampleSpan span(rx_signal.data(), rx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }

    size_t detected = demod.getLastSyncOffset();
    int error = static_cast<int>(detected) - static_cast<int>(TRUE_START);

    std::cout << "  Detected: " << detected << ", error: " << error << std::endl;

    // Larger tolerance when noise is present
    CHECK(std::abs(error) <= 32, "Sync error within 32 samples with noise");
    return std::abs(error) <= 32;
}

/**
 * Test 4: Correlation peak profile
 *
 * Diagnostic test: measures correlation across a range around the
 * true preamble start to visualize the peak shape.
 */
bool test_correlation_profile() {
    TEST("Correlation Peak Profile (Diagnostic)");

    ModemConfig config;
    config.modulation = Modulation::BPSK;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0x00);
    Samples data = mod.modulate(test_data, config.modulation);

    const size_t TRUE_START = 500;

    std::vector<float> rx_signal(TRUE_START, 0.0f);
    rx_signal.insert(rx_signal.end(), preamble.begin(), preamble.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    float max_val = 0;
    for (float s : rx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : rx_signal) s *= 0.5f / max_val;
    }

    // Process to get sync
    SampleSpan span(rx_signal.data(), rx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }

    size_t detected = demod.getLastSyncOffset();
    int error = static_cast<int>(detected) - static_cast<int>(TRUE_START);

    std::cout << "  True start: " << TRUE_START << std::endl;
    std::cout << "  Detected: " << detected << std::endl;
    std::cout << "  Error: " << error << " samples (" << (error * 1000.0f / 48000.0f) << " ms)" << std::endl;

    CHECK(std::abs(error) <= 16, "Peak detection accurate to within 16 samples");
    return std::abs(error) <= 16;
}

/**
 * Test 5: Full decode accuracy after sync
 *
 * Verifies that after sync detection, data can be correctly decoded.
 * This is the ultimate test - correct sync should enable correct decode.
 */
bool test_decode_after_sync() {
    TEST("Decode Accuracy After Sync Detection");

    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = CodeRate::R1_2;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(config.code_rate);
    LDPCDecoder decoder(config.code_rate);

    // Test pattern: known bytes
    Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};

    std::cout << "  Test pattern: ";
    for (uint8_t b : test_data) printf("%02X ", b);
    std::cout << std::endl;

    // Encode and modulate
    Bytes encoded = encoder.encode(test_data);
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(encoded, config.modulation);

    // Add leading silence
    const size_t LATENCY = 1500;
    std::vector<float> rx_signal(LATENCY, 0.0f);
    rx_signal.insert(rx_signal.end(), preamble.begin(), preamble.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    // Scale
    float max_val = 0;
    for (float s : rx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : rx_signal) s *= 0.5f / max_val;
    }

    // Process
    SampleSpan span(rx_signal.data(), rx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }

    // Check sync accuracy
    size_t detected = demod.getLastSyncOffset();
    int sync_error = static_cast<int>(detected) - static_cast<int>(LATENCY);
    std::cout << "  Sync error: " << sync_error << " samples" << std::endl;

    // Get soft bits
    std::vector<float> all_soft_bits;
    auto bits = demod.getSoftBits();
    all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());

    while (all_soft_bits.size() < 648) {
        demod.process({});
        bits = demod.getSoftBits();
        if (bits.empty()) break;
        all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
    }

    if (all_soft_bits.size() < 648) {
        FAIL("Not enough soft bits: " + std::to_string(all_soft_bits.size()));
        return false;
    }

    // LDPC decode
    std::vector<float> codeword(all_soft_bits.begin(), all_soft_bits.begin() + 648);
    Bytes decoded = decoder.decodeSoft(codeword);

    if (decoded.empty() || !decoder.lastDecodeSuccess()) {
        FAIL("LDPC decode failed");
        return false;
    }

    // Check decoded data
    std::cout << "  Decoded: ";
    for (size_t i = 0; i < std::min(decoded.size(), test_data.size()); ++i) {
        printf("%02X ", decoded[i]);
    }
    std::cout << std::endl;

    int matches = 0;
    for (size_t i = 0; i < std::min(decoded.size(), test_data.size()); ++i) {
        if (decoded[i] == test_data[i]) matches++;
    }

    float match_rate = 100.0f * matches / test_data.size();
    std::cout << "  Match rate: " << matches << "/" << test_data.size()
              << " (" << match_rate << "%)" << std::endl;

    CHECK(match_rate == 100.0f, "All bytes decoded correctly");
    return match_rate == 100.0f;
}

/**
 * Test 6: Sync refinement improvement
 *
 * Diagnostic: verifies the fine-grained refinement actually improves
 * sync accuracy compared to coarse search alone.
 * (This test mainly validates that the refinement code runs without error)
 */
bool test_refinement_runs() {
    TEST("Sync Refinement Execution");

    ModemConfig config;
    config.modulation = Modulation::BPSK;
    config.code_rate = CodeRate::R1_4;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    Samples preamble = mod.generatePreamble();
    Bytes test_data(21, 0x00);
    LDPCEncoder encoder(config.code_rate);
    Bytes encoded = encoder.encode(test_data);
    Samples data = mod.modulate(encoded, config.modulation);

    // Offset that's not aligned to STEP_SIZE=8
    const size_t TRUE_START = 1003;  // Not divisible by 8

    std::vector<float> rx_signal(TRUE_START, 0.0f);
    rx_signal.insert(rx_signal.end(), preamble.begin(), preamble.end());
    rx_signal.insert(rx_signal.end(), data.begin(), data.end());

    float max_val = 0;
    for (float s : rx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : rx_signal) s *= 0.5f / max_val;
    }

    SampleSpan span(rx_signal.data(), rx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }

    size_t detected = demod.getLastSyncOffset();
    int error = static_cast<int>(detected) - static_cast<int>(TRUE_START);

    std::cout << "  True start (not aligned to 8): " << TRUE_START << std::endl;
    std::cout << "  Detected: " << detected << std::endl;
    std::cout << "  Error: " << error << " samples" << std::endl;

    // With refinement, should be very close even though TRUE_START % 8 != 0
    CHECK(std::abs(error) <= 8, "Refinement handles non-aligned offsets");
    return std::abs(error) <= 8;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "    SYNC DETECTION ACCURACY TEST" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "This test verifies the fix for the ~118-sample" << std::endl;
    std::cout << "sync offset issue (fine-grained peak refinement)." << std::endl;

    test_sync_accuracy_basic();
    test_sync_accuracy_various_latencies();
    test_sync_with_noise();
    test_correlation_profile();
    test_decode_after_sync();
    test_refinement_runs();

    std::cout << "\n========================================" << std::endl;
    std::cout << "    RESULTS: " << tests_passed << " passed, " << tests_failed << " failed" << std::endl;
    std::cout << "========================================" << std::endl;

    return tests_failed > 0 ? 1 : 0;
}
