#include "ultra/fec.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

using namespace ultra;

// Generate random bytes for testing
Bytes generateRandomBytes(size_t count, uint32_t seed = 42) {
    std::mt19937 rng(seed);
    Bytes data(count);
    for (size_t i = 0; i < count; ++i) {
        data[i] = static_cast<uint8_t>(rng() & 0xFF);
    }
    return data;
}

// Test 1: Round-trip - interleave then deinterleave should return original
bool testRoundTrip() {
    std::cout << "Test 1: Round-trip (bytes)..." << std::flush;

    Interleaver interleaver(32, 32);  // 32x32 = 1024 bits = 128 bytes

    // Test with exactly one block
    Bytes original = generateRandomBytes(128);

    Bytes interleaved = interleaver.interleave(original);

    // Interleaved should be different from original
    bool is_shuffled = false;
    for (size_t i = 0; i < original.size(); ++i) {
        if (interleaved[i] != original[i]) {
            is_shuffled = true;
            break;
        }
    }
    if (!is_shuffled) {
        std::cout << " FAILED (not shuffled)\n";
        return false;
    }

    Bytes recovered = interleaver.deinterleave(interleaved);

    // Recovered should match original
    for (size_t i = 0; i < original.size(); ++i) {
        if (recovered[i] != original[i]) {
            std::cout << " FAILED at byte " << i << "\n";
            return false;
        }
    }

    std::cout << " OK\n";
    return true;
}

// Test 2: Soft bits round-trip
bool testSoftBitsRoundTrip() {
    std::cout << "Test 2: Round-trip (soft bits)..." << std::flush;

    Interleaver interleaver(32, 32);  // 32x32 = 1024 soft bits

    // Generate soft bits (LLRs)
    std::vector<float> original(1024);
    for (int i = 0; i < 1024; ++i) {
        original[i] = (i - 512) * 0.01f;  // Range: -5.12 to +5.11
    }

    auto interleaved = interleaver.interleave(std::span<const float>(original));

    // Check it's shuffled
    bool is_shuffled = false;
    for (size_t i = 0; i < original.size(); ++i) {
        if (std::abs(interleaved[i] - original[i]) > 1e-6f) {
            is_shuffled = true;
            break;
        }
    }
    if (!is_shuffled) {
        std::cout << " FAILED (not shuffled)\n";
        return false;
    }

    auto recovered = interleaver.deinterleave(std::span<const float>(interleaved));

    // Check exact match
    for (size_t i = 0; i < original.size(); ++i) {
        if (std::abs(recovered[i] - original[i]) > 1e-6f) {
            std::cout << " FAILED at position " << i
                      << " (expected " << original[i] << ", got " << recovered[i] << ")\n";
            return false;
        }
    }

    std::cout << " OK\n";
    return true;
}

// Test 3: Verify burst error spreading
bool testBurstErrorSpreading() {
    std::cout << "Test 3: Burst error spreading..." << std::flush;

    Interleaver interleaver(32, 32);

    // Simulate soft bits with a burst error at the start
    // All "good" bits are +5.0, burst error bits are -5.0
    std::vector<float> with_burst(1024, 5.0f);

    // Introduce a 32-bit burst error (positions 0-31)
    for (int i = 0; i < 32; ++i) {
        with_burst[i] = -5.0f;
    }

    // Deinterleave (this is what RX does - errors come in, get spread out)
    auto spread = interleaver.deinterleave(std::span<const float>(with_burst));

    // Count gaps between errors
    // With 32x32 interleaver, errors at positions 0,1,2,...,31 (column 0)
    // After deinterleave, they should be at positions 0,32,64,96,... (every 32 positions)
    std::vector<int> error_positions;
    for (int i = 0; i < 1024; ++i) {
        if (spread[i] < 0) {
            error_positions.push_back(i);
        }
    }

    if (error_positions.size() != 32) {
        std::cout << " FAILED (expected 32 errors, got " << error_positions.size() << ")\n";
        return false;
    }

    // Check that errors are spread apart (minimum gap should be ~32)
    int min_gap = 1024;
    for (size_t i = 1; i < error_positions.size(); ++i) {
        int gap = error_positions[i] - error_positions[i-1];
        if (gap < min_gap) min_gap = gap;
    }

    // With proper interleaving, errors should be 32 positions apart
    if (min_gap < 30) {  // Allow some tolerance
        std::cout << " FAILED (min gap = " << min_gap << ", expected >= 30)\n";
        return false;
    }

    std::cout << " OK (min gap = " << min_gap << ")\n";
    return true;
}

// Test 4: Multiple block sizes
bool testDifferentSizes() {
    std::cout << "Test 4: Different interleaver sizes..." << std::flush;

    struct TestCase {
        int rows, cols;
    };

    TestCase cases[] = {
        {8, 8},    // 64 bits = 8 bytes
        {16, 16},  // 256 bits = 32 bytes
        {32, 32},  // 1024 bits = 128 bytes
        {16, 32},  // 512 bits = 64 bytes (non-square)
    };

    for (const auto& tc : cases) {
        Interleaver interleaver(tc.rows, tc.cols);
        int block_bytes = (tc.rows * tc.cols) / 8;

        Bytes original = generateRandomBytes(block_bytes);
        Bytes interleaved = interleaver.interleave(original);
        Bytes recovered = interleaver.deinterleave(interleaved);

        for (size_t i = 0; i < original.size(); ++i) {
            if (recovered[i] != original[i]) {
                std::cout << " FAILED (" << tc.rows << "x" << tc.cols << ")\n";
                return false;
            }
        }
    }

    std::cout << " OK\n";
    return true;
}

int main() {
    std::cout << "\nTesting Interleaver implementation...\n\n";

    int failures = 0;

    if (!testRoundTrip()) failures++;
    if (!testSoftBitsRoundTrip()) failures++;
    if (!testBurstErrorSpreading()) failures++;
    if (!testDifferentSizes()) failures++;

    std::cout << "\n";
    if (failures == 0) {
        std::cout << "All interleaver tests passed!\n";
        return 0;
    } else {
        std::cout << failures << " test(s) FAILED\n";
        return 1;
    }
}
