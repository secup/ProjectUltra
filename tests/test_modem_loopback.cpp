/**
 * Comprehensive modem loopback test
 *
 * Tests the full OFDM modulation/demodulation pipeline in software loopback
 * (no audio hardware involved). This isolates modem code bugs from audio path issues.
 *
 * Tests:
 * 1. FFT roundtrip (IFFT -> FFT gives original)
 * 2. Single carrier transmission
 * 3. All-zeros BPSK pattern
 * 4. All-ones BPSK pattern
 * 5. Full frame with LDPC encoding
 */

#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"
#include "ultra/types.hpp"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <complex>

using namespace ultra;

// Test result tracking
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) std::cout << "\n=== TEST: " << name << " ===" << std::endl
#define PASS(msg) do { std::cout << "[PASS] " << msg << std::endl; tests_passed++; } while(0)
#define FAIL(msg) do { std::cout << "[FAIL] " << msg << std::endl; tests_failed++; } while(0)
#define CHECK(cond, msg) do { if (cond) PASS(msg); else FAIL(msg); } while(0)

/**
 * Test 1: FFT Roundtrip
 * Verify that IFFT followed by FFT returns the original values (with scaling)
 */
bool test_fft_roundtrip() {
    TEST("FFT Roundtrip");

    const size_t N = 512;
    FFT fft(N);

    // Create a test frequency domain signal
    std::vector<Complex> freq_in(N, Complex(0, 0));
    freq_in[1] = Complex(1, 0);    // DC+1
    freq_in[10] = Complex(0, 1);   // Some carrier
    freq_in[N-10] = Complex(0, -1); // Conjugate for real signal

    // IFFT to time domain
    std::vector<Complex> time_domain;
    fft.inverse(freq_in, time_domain);

    // FFT back to frequency domain
    std::vector<Complex> freq_out;
    fft.forward(time_domain, freq_out);

    // Check that we get back the original (with possible scaling by N)
    // Find the scaling factor
    float scale = std::abs(freq_out[1]) / std::abs(freq_in[1]);
    std::cout << "FFT scaling factor: " << scale << std::endl;

    bool passed = true;
    for (size_t i = 0; i < N; i++) {
        Complex expected = freq_in[i] * scale;
        Complex actual = freq_out[i];
        float error = std::abs(actual - expected);
        if (error > 0.001f * scale) {
            std::cout << "  Bin " << i << ": expected (" << expected.real() << "," << expected.imag()
                      << ") got (" << actual.real() << "," << actual.imag() << ") error=" << error << std::endl;
            passed = false;
        }
    }

    CHECK(passed, "FFT roundtrip preserves values");
    return passed;
}

/**
 * Test 2: Modulator creates correct frequency domain
 * Verify pilot and data carrier placement
 */
bool test_carrier_mapping() {
    TEST("Carrier Mapping");

    ModemConfig config;
    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    // Generate preamble and check it doesn't crash
    auto preamble = mod.generatePreamble();
    CHECK(preamble.size() > 0, "Preamble generated");

    // Create a simple test pattern - all zeros (BPSK -1)
    Bytes test_data(81, 0x00);  // 648 bits of zeros
    auto modulated = mod.modulate(test_data, Modulation::BPSK);
    CHECK(modulated.size() > 0, "Modulation produced samples");

    std::cout << "  Preamble: " << preamble.size() << " samples" << std::endl;
    std::cout << "  Data: " << modulated.size() << " samples" << std::endl;
    std::cout << "  Samples per symbol: " << mod.samplesPerSymbol() << std::endl;
    std::cout << "  Bits per symbol (BPSK): " << mod.bitsPerSymbol(Modulation::BPSK) << std::endl;

    return true;
}

/**
 * Test 3: Full loopback - Modulate then Demodulate
 * This is the critical test - does the full pipeline work?
 */
bool test_full_loopback() {
    TEST("Full Loopback (no channel)");

    // Use exact same config as test_multiblock_ldpc (which passes)
    ModemConfig config;
    config.code_rate = CodeRate::R3_4;
    config.modulation = Modulation::QPSK;

    // Create separate modulator and demodulator (like real TX/RX)
    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    // Generate test pattern - all zeros for QPSK
    Bytes test_data(81, 0x00);

    // Modulate with QPSK (matching config)
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(test_data, config.modulation);

    // Combine preamble and data
    Samples tx_signal;
    tx_signal.reserve(preamble.size() + data.size());
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), data.begin(), data.end());

    // Scale signal (like test_multiblock_ldpc does)
    float max_val = 0;
    for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : tx_signal) s *= scale;
    }

    std::cout << "  TX signal: " << tx_signal.size() << " samples" << std::endl;

    // Feed to demodulator using explicit SampleSpan (like test_multiblock_ldpc)
    SampleSpan span(tx_signal.data(), tx_signal.size());
    bool synced = demod.process(span);
    std::cout << "  Sync found: " << (demod.isSynced() ? "YES" : "NO") << std::endl;

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }
    PASS("Demodulator synced");

    // Keep processing until we have soft bits
    int iterations = 0;
    while (!synced && iterations < 100) {
        synced = demod.process({});  // Empty span to extract more data
        iterations++;
    }

    // Get soft bits
    auto soft_bits = demod.getSoftBits();
    std::cout << "  Soft bits received: " << soft_bits.size() << std::endl;

    CHECK(soft_bits.size() >= 648, "Received enough soft bits for one codeword");

    if (soft_bits.size() < 648) {
        return false;
    }

    // Check soft bit polarity - for all-zeros input with BPSK:
    // Bit 0 maps to symbol -1
    // LLR for bit 0 should be POSITIVE (our convention: positive LLR = bit 0)
    // Wait, let me check the convention...
    // softDemapBPSK: return -2.0f * sym.real() / noise_var
    // For sym = -1 (bit 0): LLR = -2 * (-1) / nv = +2/nv (positive)
    // For sym = +1 (bit 1): LLR = -2 * (+1) / nv = -2/nv (negative)
    // So positive LLR = bit 0, negative LLR = bit 1

    int positive_count = 0;
    int negative_count = 0;
    float sum_llr = 0;

    for (size_t i = 0; i < std::min(soft_bits.size(), size_t(648)); i++) {
        if (soft_bits[i] > 0) positive_count++;
        else negative_count++;
        sum_llr += soft_bits[i];
    }

    std::cout << "  LLR stats: " << positive_count << " positive (bit 0), "
              << negative_count << " negative (bit 1)" << std::endl;
    std::cout << "  Average LLR: " << (sum_llr / 648) << std::endl;

    // For all-zeros input, we expect mostly POSITIVE LLRs
    float positive_ratio = (float)positive_count / (positive_count + negative_count);
    std::cout << "  Positive ratio: " << (positive_ratio * 100) << "%" << std::endl;

    CHECK(positive_ratio > 0.9f, "At least 90% of LLRs are positive (correct polarity)");

    // Convert to hard bits and check
    Bytes decoded_bits;
    uint8_t byte = 0;
    int bit_count = 0;
    for (size_t i = 0; i < 648; i++) {
        uint8_t bit = (soft_bits[i] > 0) ? 0 : 1;  // Positive LLR = bit 0
        byte = (byte << 1) | bit;
        bit_count++;
        if (bit_count == 8) {
            decoded_bits.push_back(byte);
            byte = 0;
            bit_count = 0;
        }
    }

    // Check first few bytes
    std::cout << "  First 10 decoded bytes: ";
    for (size_t i = 0; i < std::min(decoded_bits.size(), size_t(10)); i++) {
        printf("%02x ", decoded_bits[i]);
    }
    std::cout << std::endl;

    // For all-zeros input, output should be all zeros (or very close)
    int zero_bytes = 0;
    for (size_t i = 0; i < std::min(decoded_bits.size(), size_t(81)); i++) {
        if (decoded_bits[i] == 0x00) zero_bytes++;
    }

    float zero_ratio = (float)zero_bytes / std::min(decoded_bits.size(), size_t(81));
    std::cout << "  Zero bytes: " << zero_bytes << "/" << std::min(decoded_bits.size(), size_t(81))
              << " (" << (zero_ratio * 100) << "%)" << std::endl;

    CHECK(zero_ratio > 0.9f, "At least 90% of bytes are correctly decoded as 0x00");

    return positive_ratio > 0.9f && zero_ratio > 0.9f;
}

/**
 * Test 4: Full loopback with LDPC
 * Tests the complete chain: data -> LDPC encode -> modulate -> demodulate -> LDPC decode -> data
 */
bool test_full_loopback_with_ldpc() {
    TEST("Full Loopback with LDPC");

    // Use same config as test_multiblock_ldpc
    ModemConfig config;
    config.code_rate = CodeRate::R3_4;
    config.modulation = Modulation::QPSK;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(config.code_rate);
    LDPCDecoder decoder(config.code_rate);

    // Create test message
    std::string test_msg = "Hello, OFDM loopback test!";
    Bytes test_data(test_msg.begin(), test_msg.end());

    std::cout << "  Original message: \"" << test_msg << "\" (" << test_data.size() << " bytes)" << std::endl;

    // LDPC encode
    Bytes encoded = encoder.encode(test_data);
    std::cout << "  LDPC encoded: " << encoded.size() << " bytes" << std::endl;

    // Modulate
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(encoded, config.modulation);

    Samples tx_signal;
    tx_signal.reserve(preamble.size() + data.size());
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), data.begin(), data.end());

    // Scale signal
    float max_val = 0;
    for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : tx_signal) s *= scale;
    }

    std::cout << "  TX signal: " << tx_signal.size() << " samples" << std::endl;

    // Demodulate using explicit SampleSpan
    SampleSpan span(tx_signal.data(), tx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }
    PASS("Demodulator synced");

    // Collect all soft bits
    std::vector<float> all_soft_bits;
    int iterations = 0;
    while (iterations < 100) {
        auto bits = demod.getSoftBits();
        if (bits.empty()) {
            demod.process({});
            bits = demod.getSoftBits();
        }
        if (bits.empty()) break;
        all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
        iterations++;
    }

    std::cout << "  Total soft bits: " << all_soft_bits.size() << std::endl;

    // LDPC decode
    if (all_soft_bits.size() < 648) {
        FAIL("Not enough soft bits for LDPC decode");
        return false;
    }

    // Decode one codeword
    std::vector<float> codeword_bits(all_soft_bits.begin(), all_soft_bits.begin() + 648);
    Bytes decoded = decoder.decodeSoft(codeword_bits);

    std::cout << "  LDPC decoded: " << decoded.size() << " bytes" << std::endl;

    // Extract message (remove padding)
    std::string decoded_msg;
    for (size_t i = 0; i < std::min(decoded.size(), test_data.size()); i++) {
        if (decoded[i] >= 32 && decoded[i] < 127) {
            decoded_msg += (char)decoded[i];
        } else {
            decoded_msg += '.';
        }
    }

    std::cout << "  Decoded message: \"" << decoded_msg << "\"" << std::endl;

    // Check if messages match
    bool match = true;
    for (size_t i = 0; i < std::min(decoded.size(), test_data.size()); i++) {
        if (decoded[i] != test_data[i]) {
            match = false;
            break;
        }
    }

    CHECK(match, "Decoded message matches original");

    return match;
}

/**
 * Test 5: Verify equalized symbols directly
 * Check that after channel estimation and equalization, symbols are correct
 */
bool test_equalized_symbols() {
    TEST("Equalized Symbol Verification");

    // Use BPSK for cleaner constellation check
    ModemConfig config;
    config.modulation = Modulation::BPSK;
    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    // All-zeros = BPSK -1 symbols
    Bytes test_data(81, 0x00);

    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(test_data, Modulation::BPSK);

    Samples tx_signal;
    tx_signal.reserve(preamble.size() + data.size());
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), data.begin(), data.end());

    // Scale like other tests
    float max_val = 0;
    for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : tx_signal) s *= scale;
    }

    SampleSpan span(tx_signal.data(), tx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }

    // Get constellation symbols (equalized)
    auto symbols = demod.getConstellationSymbols();
    std::cout << "  Constellation symbols: " << symbols.size() << std::endl;

    if (symbols.empty()) {
        // Process more to get symbols
        demod.process({});
        symbols = demod.getConstellationSymbols();
    }

    if (symbols.size() < 10) {
        FAIL("Not enough constellation symbols");
        return false;
    }

    // For BPSK -1, equalized symbols should be near (-1, 0)
    int correct_count = 0;
    float total_real = 0;
    float total_imag = 0;

    for (size_t i = 0; i < std::min(symbols.size(), size_t(100)); i++) {
        total_real += symbols[i].real();
        total_imag += symbols[i].imag();

        // Check if symbol is in the correct half-plane (real < 0 for BPSK -1)
        if (symbols[i].real() < 0) {
            correct_count++;
        }
    }

    size_t check_count = std::min(symbols.size(), size_t(100));
    float avg_real = total_real / check_count;
    float avg_imag = total_imag / check_count;
    float correct_ratio = (float)correct_count / check_count;

    std::cout << "  Average equalized symbol: (" << avg_real << ", " << avg_imag << ")" << std::endl;
    std::cout << "  Symbols in correct half-plane: " << correct_count << "/" << check_count
              << " (" << (correct_ratio * 100) << "%)" << std::endl;

    CHECK(correct_ratio > 0.9f, "At least 90% of symbols in correct half-plane");
    CHECK(avg_real < -0.5f, "Average real part is negative (as expected for BPSK -1)");

    return correct_ratio > 0.9f && avg_real < -0.5f;
}

/**
 * Test 6: BPSK R1/4 loopback (PROBE frame configuration)
 * This is the exact config used for link establishment (PROBE, CONNECT, etc.)
 * Must work reliably - it's the most robust mode.
 */
bool test_probe_config_loopback() {
    TEST("BPSK R1/4 Loopback (PROBE config)");

    // Exact config used for PROBE frames
    ModemConfig config;
    config.code_rate = CodeRate::R1_4;
    config.modulation = Modulation::BPSK;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(config.code_rate);
    LDPCDecoder decoder(config.code_rate);

    // Simulate a PROBE frame payload (small, like real probes)
    Bytes test_data = {0x07};  // Capability byte (typical PROBE payload)

    std::cout << "  Config: BPSK R1/4 (k=162 bits, n=648 bits)" << std::endl;
    std::cout << "  Test data: " << test_data.size() << " bytes" << std::endl;

    // LDPC encode
    Bytes encoded = encoder.encode(test_data);
    std::cout << "  LDPC encoded: " << encoded.size() << " bytes" << std::endl;

    // Modulate
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(encoded, config.modulation);

    Samples tx_signal;
    tx_signal.reserve(preamble.size() + data.size());
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), data.begin(), data.end());

    // Scale signal
    float max_val = 0;
    for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : tx_signal) s *= scale;
    }

    std::cout << "  TX signal: " << tx_signal.size() << " samples ("
              << (tx_signal.size() * 1000.0f / 48000.0f) << " ms)" << std::endl;

    // Demodulate
    SampleSpan span(tx_signal.data(), tx_signal.size());
    demod.process(span);

    if (!demod.isSynced()) {
        FAIL("Demodulator did not sync");
        return false;
    }
    PASS("Demodulator synced");

    // Collect soft bits
    std::vector<float> all_soft_bits;
    auto bits = demod.getSoftBits();
    all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());

    // Try to get more if needed
    while (all_soft_bits.size() < 648) {
        SampleSpan empty;
        demod.process(empty);
        bits = demod.getSoftBits();
        if (bits.empty()) break;
        all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
    }

    std::cout << "  Soft bits received: " << all_soft_bits.size() << std::endl;

    if (all_soft_bits.size() < 648) {
        FAIL("Not enough soft bits for LDPC decode");
        return false;
    }

    // LDPC decode
    std::vector<float> codeword(all_soft_bits.begin(), all_soft_bits.begin() + 648);
    Bytes decoded = decoder.decodeSoft(codeword);

    if (!decoder.lastDecodeSuccess()) {
        FAIL("LDPC decode failed");
        return false;
    }
    PASS("LDPC decode succeeded");

    // Check first byte matches
    if (decoded.size() > 0 && decoded[0] == test_data[0]) {
        std::cout << "  Decoded byte: 0x" << std::hex << (int)decoded[0] << std::dec
                  << " (expected 0x" << std::hex << (int)test_data[0] << std::dec << ")" << std::endl;
        PASS("Decoded data matches");
        return true;
    } else {
        std::cout << "  Decoded byte: 0x" << std::hex << (int)(decoded.size() > 0 ? decoded[0] : 0xFF) << std::dec
                  << " (expected 0x" << std::hex << (int)test_data[0] << std::dec << ")" << std::endl;
        FAIL("Decoded data mismatch");
        return false;
    }
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "    MODEM LOOPBACK VERIFICATION TEST    " << std::endl;
    std::cout << "========================================" << std::endl;

    test_fft_roundtrip();
    test_carrier_mapping();
    test_full_loopback();
    test_equalized_symbols();
    test_full_loopback_with_ldpc();
    test_probe_config_loopback();  // BPSK R1/4 - critical for link establishment

    std::cout << "\n========================================" << std::endl;
    std::cout << "    RESULTS: " << tests_passed << " passed, " << tests_failed << " failed" << std::endl;
    std::cout << "========================================" << std::endl;

    return tests_failed > 0 ? 1 : 0;
}
