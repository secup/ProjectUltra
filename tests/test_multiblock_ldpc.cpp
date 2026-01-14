/**
 * Multi-Codeword LDPC Test Suite
 *
 * These tests verify that data requiring multiple LDPC codewords is encoded
 * and decoded correctly through the full modem pipeline.
 *
 * Protocol Context:
 * - ARQ protocol chunks files into 250-byte pieces
 * - Each chunk + headers = one DATA frame (~279 bytes max)
 * - Each DATA frame is a separate transmission with its own preamble
 * - Maximum realistic single-frame size: ~279 bytes = 5 LDPC codewords
 *
 * Key Technical Issue:
 * LDPC rate 3/4 has k=486 info bits per codeword = 60.75 bytes (NOT byte-aligned).
 * Multi-codeword data must be handled at the BIT level to avoid corruption
 * at codeword boundaries (e.g., byte 60's last 2 bits crossing into next codeword).
 */

#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/types.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <cassert>

using namespace ultra;

// Test counters
static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { std::cout << "  Testing " << name << "... " << std::flush; tests_run++; } while(0)

#define PASS() \
    do { std::cout << "PASS\n"; tests_passed++; } while(0)

#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << "\n"; return false; } while(0)

// LDPC parameters for each code rate
struct LDPCParams {
    CodeRate rate;
    int k;  // info bits
    int n;  // codeword bits
    int k_bytes;  // ceil(k/8)
    const char* name;
};

static const LDPCParams LDPC_RATES[] = {
    { CodeRate::R1_4, 162, 648, 21, "R1/4" },
    { CodeRate::R1_2, 324, 648, 41, "R1/2" },
    { CodeRate::R2_3, 432, 648, 54, "R2/3" },
    { CodeRate::R3_4, 486, 648, 61, "R3/4" },  // 486 bits = 60.75 bytes - problematic!
    { CodeRate::R5_6, 540, 648, 68, "R5/6" },
};

// Generate deterministic test data
Bytes generateTestData(size_t size, uint8_t seed = 0x42) {
    Bytes data(size);
    for (size_t i = 0; i < size; i++) {
        data[i] = static_cast<uint8_t>((i * 7 + seed + i / 256) & 0xFF);
    }
    return data;
}

// Compare two byte arrays and report first difference
bool compareData(const Bytes& expected, const Bytes& actual, size_t& first_diff_pos) {
    size_t min_len = std::min(expected.size(), actual.size());
    for (size_t i = 0; i < min_len; i++) {
        if (expected[i] != actual[i]) {
            first_diff_pos = i;
            return false;
        }
    }
    if (expected.size() != actual.size()) {
        first_diff_pos = min_len;
        return false;
    }
    return true;
}

// Print hex dump of bytes around a position
void printHexContext(const Bytes& data, size_t pos, const char* label) {
    size_t start = (pos > 5) ? pos - 5 : 0;
    size_t end = std::min(pos + 10, data.size());

    std::cout << "      " << label << " [" << start << "-" << end-1 << "]: ";
    for (size_t i = start; i < end; i++) {
        if (i == pos) std::cout << "[";
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i];
        if (i == pos) std::cout << "]";
        std::cout << " ";
    }
    std::cout << std::dec << "\n";
}

// ============================================================================
// LDPC Encoder/Decoder Direct Tests
// ============================================================================

bool test_ldpc_single_block(CodeRate rate, const char* rate_name) {
    std::string test_name = std::string("Single block LDPC ") + rate_name;
    TEST(test_name.c_str());

    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    // Get params for this rate
    int k = 0;
    for (const auto& p : LDPC_RATES) {
        if (p.rate == rate) { k = p.k; break; }
    }
    size_t data_bytes = (k + 7) / 8;  // Max bytes that fit in one block

    // Generate test data
    Bytes tx_data = generateTestData(data_bytes);

    // Encode
    Bytes encoded = encoder.encode(tx_data);

    // Convert to soft bits (simulate perfect reception)
    std::vector<float> soft_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) {
            uint8_t bit = (byte >> b) & 1;
            soft_bits.push_back(bit ? -6.0f : 6.0f);  // LLR: positive for 0, negative for 1
        }
    }

    // Decode
    Bytes rx_data = decoder.decodeSoft(soft_bits);

    if (!decoder.lastDecodeSuccess()) FAIL("Decode reported failure");

    // Truncate to original size (decoder may output padded bytes)
    if (rx_data.size() > tx_data.size()) {
        rx_data.resize(tx_data.size());
    }

    // Compare
    size_t diff_pos;
    if (!compareData(tx_data, rx_data, diff_pos)) {
        std::cout << "\n      First difference at byte " << diff_pos << "\n";
        printHexContext(tx_data, diff_pos, "TX");
        printHexContext(rx_data, diff_pos, "RX");
        FAIL("Data mismatch");
    }

    PASS();
    return true;
}

bool test_ldpc_multi_block(CodeRate rate, const char* rate_name, size_t num_blocks) {
    std::string test_name = std::string("Multi-block (") + std::to_string(num_blocks) +
                            " blocks) LDPC " + rate_name;
    TEST(test_name.c_str());

    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    // Get params for this rate
    int k = 0, n = 0;
    for (const auto& p : LDPC_RATES) {
        if (p.rate == rate) { k = p.k; n = p.n; break; }
    }

    // Calculate data size that requires exactly num_blocks codewords
    // Each block encodes k bits = k/8 bytes (with potential fractional byte)
    size_t data_bits = k * num_blocks;
    size_t data_bytes = (data_bits + 7) / 8;

    // Generate test data
    Bytes tx_data = generateTestData(data_bytes);

    // Encode
    Bytes encoded = encoder.encode(tx_data);

    // Verify encoded size
    size_t expected_encoded_bits = n * num_blocks;
    size_t expected_encoded_bytes = (expected_encoded_bits + 7) / 8;

    // Convert to soft bits
    std::vector<float> soft_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) {
            uint8_t bit = (byte >> b) & 1;
            soft_bits.push_back(bit ? -6.0f : 6.0f);
        }
    }

    // Decode
    Bytes rx_data = decoder.decodeSoft(soft_bits);

    if (!decoder.lastDecodeSuccess()) FAIL("Decode reported failure");

    // Truncate to original size
    if (rx_data.size() > tx_data.size()) {
        rx_data.resize(tx_data.size());
    }

    // Compare byte-by-byte
    size_t diff_pos;
    if (!compareData(tx_data, rx_data, diff_pos)) {
        std::cout << "\n      First difference at byte " << diff_pos << "\n";
        std::cout << "      TX size: " << tx_data.size() << ", RX size: " << rx_data.size() << "\n";
        std::cout << "      k=" << k << " bits, n=" << n << " bits, blocks=" << num_blocks << "\n";

        // Check if difference is at codeword boundary
        size_t bits_per_block = k;
        size_t byte_of_boundary = bits_per_block / 8;
        int fractional_bits = bits_per_block % 8;

        std::cout << "      Codeword boundary: byte " << byte_of_boundary
                  << " + " << fractional_bits << " bits\n";

        if (diff_pos == byte_of_boundary || diff_pos == byte_of_boundary + 1) {
            std::cout << "      *** CODEWORD BOUNDARY BUG DETECTED ***\n";
        }

        printHexContext(tx_data, diff_pos, "TX");
        printHexContext(rx_data, diff_pos, "RX");
        FAIL("Data mismatch");
    }

    PASS();
    return true;
}

// Test specific byte position - critical for finding boundary bugs
bool test_ldpc_boundary_bytes(CodeRate rate, const char* rate_name) {
    std::string test_name = std::string("Boundary byte test LDPC ") + rate_name;
    TEST(test_name.c_str());

    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    int k = 0;
    for (const auto& p : LDPC_RATES) {
        if (p.rate == rate) { k = p.k; break; }
    }

    // k bits per block - find the problematic boundary byte
    size_t boundary_byte = k / 8;  // First partial byte
    int partial_bits = k % 8;      // How many bits are used in partial byte

    if (partial_bits == 0) {
        std::cout << "(k=" << k << " is byte-aligned, skip) ";
        PASS();
        return true;
    }

    std::cout << "(k=" << k << ", boundary at byte " << boundary_byte
              << " with " << partial_bits << " bits) ";

    // Create data with specific pattern at boundary
    // Use 5 blocks to thoroughly test
    size_t num_blocks = 5;
    size_t data_size = (k * num_blocks + 7) / 8;
    Bytes tx_data = generateTestData(data_size);

    // Make boundary bytes distinctive
    for (size_t block = 0; block < num_blocks; block++) {
        size_t block_start_bit = block * k;
        size_t block_boundary_byte = block_start_bit / 8 + boundary_byte;
        if (block_boundary_byte < tx_data.size()) {
            tx_data[block_boundary_byte] = 0xAA;  // Alternating bits
        }
        if (block_boundary_byte + 1 < tx_data.size()) {
            tx_data[block_boundary_byte + 1] = 0x55;
        }
    }

    // Encode
    Bytes encoded = encoder.encode(tx_data);

    // Convert to soft bits
    std::vector<float> soft_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) {
            uint8_t bit = (byte >> b) & 1;
            soft_bits.push_back(bit ? -6.0f : 6.0f);
        }
    }

    // Decode
    Bytes rx_data = decoder.decodeSoft(soft_bits);

    if (rx_data.size() > tx_data.size()) {
        rx_data.resize(tx_data.size());
    }

    // Check specifically the boundary bytes
    bool all_ok = true;
    for (size_t block = 0; block < num_blocks; block++) {
        size_t block_start_bit = block * k;
        size_t block_boundary_byte = block_start_bit / 8 + boundary_byte;

        if (block_boundary_byte < tx_data.size() && block_boundary_byte < rx_data.size()) {
            if (tx_data[block_boundary_byte] != rx_data[block_boundary_byte]) {
                std::cout << "\n      Block " << block << " boundary byte " << block_boundary_byte
                          << ": TX=0x" << std::hex << (int)tx_data[block_boundary_byte]
                          << " RX=0x" << (int)rx_data[block_boundary_byte] << std::dec;
                all_ok = false;
            }
        }
    }

    if (!all_ok) {
        FAIL("Boundary byte corruption detected");
    }

    PASS();
    return true;
}

// ============================================================================
// Full Modem Pipeline Tests
// ============================================================================

bool test_modem_pipeline_multiblock(size_t data_size) {
    std::string test_name = "Full modem pipeline with " + std::to_string(data_size) + " bytes";
    TEST(test_name.c_str());

    // Use balanced config (QPSK R3/4 - the problematic rate)
    ModemConfig config;
    config.code_rate = CodeRate::R3_4;
    config.modulation = Modulation::QPSK;

    OFDMModulator modulator(config);
    OFDMDemodulator demodulator(config);
    LDPCEncoder encoder(config.code_rate);
    LDPCDecoder decoder(config.code_rate);
    Interleaver interleaver(24, 27);

    // Generate test data
    Bytes tx_data = generateTestData(data_size);

    // === TX Chain ===
    Bytes encoded = encoder.encode(tx_data);

    std::cout << "\n      TX: " << tx_data.size() << " bytes -> " << encoded.size()
              << " encoded bytes (" << encoded.size() * 8 << " bits)";

    // Don't interleave in this test - it's configured for single codeword
    // For multi-codeword, we'd need per-codeword interleaving (handled by modem_engine)
    Bytes& to_modulate = encoded;  // Skip interleaving for this test

    Samples preamble = modulator.generatePreamble();
    Samples data_audio = modulator.modulate(to_modulate, config.modulation);

    Samples tx_audio;
    tx_audio.reserve(preamble.size() + data_audio.size());
    tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
    tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

    // Scale
    float max_val = 0;
    for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : tx_audio) s *= scale;
    }

    // === RX Chain (perfect channel) ===
    // Debug: check audio length
    size_t symbol_samples = config.getSymbolDuration();
    std::cout << "\n      Config: fft=" << config.fft_size << " cp=" << config.getCyclicPrefix()
              << " carriers=" << config.num_carriers << " sym_dur=" << symbol_samples;
    std::cout << "\n      Audio: " << tx_audio.size() << " samples ("
              << tx_audio.size() / symbol_samples << " symbols, "
              << preamble.size() / symbol_samples << " preamble, "
              << data_audio.size() / symbol_samples << " data)";

    // Feed ALL samples to demodulator first
    SampleSpan full_span(tx_audio.data(), tx_audio.size());
    bool frame_ready = demodulator.process(full_span);

    std::cout << ", demod returned " << (frame_ready ? "true" : "false");

    // Now collect ALL soft bits from ALL codewords
    std::vector<float> all_soft_bits;
    int codewords_received = 0;

    while (frame_ready) {
        auto soft_bits = demodulator.getSoftBits();
        if (!soft_bits.empty()) {
            codewords_received++;
            // Skip deinterleaving (matches TX side)
            all_soft_bits.insert(all_soft_bits.end(),
                                 soft_bits.begin(), soft_bits.end());
        }
        // Continue processing to get next codeword
        SampleSpan empty;
        frame_ready = demodulator.process(empty);
    }

    if (all_soft_bits.empty()) FAIL("No soft bits received");

    // Verify we got expected number of codewords
    int k = 486;  // R3/4 info bits
    int n = 648;  // R3/4 codeword bits
    int expected_codewords = (data_size * 8 + k - 1) / k;

    if (codewords_received != expected_codewords) {
        std::cout << "\n      WARNING: Got " << codewords_received << " codewords, expected "
                  << expected_codewords << "\n";
        std::cout << "      Total soft bits: " << all_soft_bits.size()
                  << ", expected: " << (expected_codewords * n) << "\n";
    }

    // Decode all at once
    Bytes rx_data = decoder.decodeSoft(all_soft_bits);

    if (!decoder.lastDecodeSuccess()) FAIL("Decode reported failure");

    // Truncate to original size
    if (rx_data.size() > tx_data.size()) {
        rx_data.resize(tx_data.size());
    }

    // Compare
    size_t diff_pos;
    if (!compareData(tx_data, rx_data, diff_pos)) {
        std::cout << "\n      First difference at byte " << diff_pos << "\n";
        printHexContext(tx_data, diff_pos, "TX");
        printHexContext(rx_data, diff_pos, "RX");
        FAIL("Data mismatch through full pipeline");
    }

    PASS();
    return true;
}

// ============================================================================
// Protocol Frame Size Tests
// ============================================================================

bool test_protocol_frame_sizes() {
    TEST("Protocol frame sizes (24, 46, 279 bytes)");

    // These are actual protocol frame sizes from the logs
    size_t frame_sizes[] = { 24, 46, 279 };

    for (size_t size : frame_sizes) {
        LDPCEncoder encoder(CodeRate::R3_4);
        LDPCDecoder decoder(CodeRate::R3_4);

        Bytes tx_data = generateTestData(size);

        // Calculate expected codewords
        int k = 486;  // R3/4 info bits
        size_t num_codewords = (size * 8 + k - 1) / k;

        // Encode
        Bytes encoded = encoder.encode(tx_data);

        // Convert to soft bits
        std::vector<float> soft_bits;
        for (uint8_t byte : encoded) {
            for (int b = 7; b >= 0; --b) {
                uint8_t bit = (byte >> b) & 1;
                soft_bits.push_back(bit ? -6.0f : 6.0f);
            }
        }

        // Decode
        Bytes rx_data = decoder.decodeSoft(soft_bits);

        if (rx_data.size() > tx_data.size()) {
            rx_data.resize(tx_data.size());
        }

        size_t diff_pos;
        if (!compareData(tx_data, rx_data, diff_pos)) {
            std::cout << "\n      Frame size " << size << " bytes (" << num_codewords << " codewords)\n";
            std::cout << "      First difference at byte " << diff_pos << "\n";
            printHexContext(tx_data, diff_pos, "TX");
            printHexContext(rx_data, diff_pos, "RX");
            FAIL("Data mismatch for frame size");
        }
    }

    PASS();
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            Multi-Codeword LDPC Test Suite                          ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Single Block Tests (baseline):\n";
    for (const auto& p : LDPC_RATES) {
        test_ldpc_single_block(p.rate, p.name);
    }

    std::cout << "\nMulti-Block Tests (2 blocks):\n";
    for (const auto& p : LDPC_RATES) {
        test_ldpc_multi_block(p.rate, p.name, 2);
    }

    std::cout << "\nMulti-Block Tests (5 blocks):\n";
    for (const auto& p : LDPC_RATES) {
        test_ldpc_multi_block(p.rate, p.name, 5);
    }

    std::cout << "\nBoundary Byte Tests (codeword boundaries):\n";
    for (const auto& p : LDPC_RATES) {
        test_ldpc_boundary_bytes(p.rate, p.name);
    }

    std::cout << "\nProtocol Frame Size Tests:\n";
    test_protocol_frame_sizes();

    // Note: The ARQ protocol chunks files into 250-byte pieces, so the maximum
    // single frame through the modem is ~279 bytes (DATA frame with headers).
    // We test up to 279 bytes which covers all realistic scenarios.
    std::cout << "\nFull Modem Pipeline Tests:\n";
    test_modem_pipeline_multiblock(60);   // Single block (~1 codeword)
    test_modem_pipeline_multiblock(100);  // ~2 codewords
    test_modem_pipeline_multiblock(200);  // ~4 codewords
    test_modem_pipeline_multiblock(279);  // Max DATA frame size (~5 codewords)

    std::cout << "\n═══════════════════════════════════════════════════════════════════════\n";
    std::cout << "Results: " << tests_passed << "/" << tests_run << " passed";
    if (tests_passed == tests_run) {
        std::cout << " - ALL TESTS PASSED\n";
    } else {
        std::cout << " - " << (tests_run - tests_passed) << " TESTS FAILED\n";
    }
    std::cout << "═══════════════════════════════════════════════════════════════════════\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
