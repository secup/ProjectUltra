/**
 * OTFS Unit Tests
 *
 * Tests the OTFS modulation/demodulation through the full modem pipeline.
 * Per CLAUDE.md: "EVERY test that validates data integrity MUST go through this pipeline."
 */

#include "ultra/otfs.hpp"
#include "ultra/fec.hpp"
#include "ultra/types.hpp"
#include <iostream>
#include <cmath>

using namespace ultra;

// Test helper: check if two complex values are approximately equal
bool approxEqual(Complex a, Complex b, float tolerance = 0.01f) {
    return std::abs(a - b) < tolerance;
}

// Test 1: ISFFT/SFFT roundtrip (identity transform)
bool testTransformRoundtrip() {
    std::cout << "Test 1: ISFFT/SFFT roundtrip... ";

    const uint32_t M = 16, N = 8;
    std::vector<Complex> dd_input(M * N);

    // Create test pattern: known QPSK symbols
    for (size_t i = 0; i < dd_input.size(); ++i) {
        float phase = static_cast<float>(i) * 0.5f;
        dd_input[i] = Complex(std::cos(phase) * 0.707f, std::sin(phase) * 0.707f);
    }

    // ISFFT → SFFT should give back original (within tolerance)
    std::vector<Complex> tf_grid, dd_output;
    isfft(dd_input, tf_grid, M, N);
    sfft(tf_grid, dd_output, M, N);

    float max_error = 0;
    for (size_t i = 0; i < dd_input.size(); ++i) {
        float error = std::abs(dd_input[i] - dd_output[i]);
        max_error = std::max(max_error, error);
    }

    if (max_error > 0.001f) {
        std::cout << "FAILED (max_error=" << max_error << ")\n";
        return false;
    }

    std::cout << "PASSED (max_error=" << max_error << ")\n";
    return true;
}

// Test 2: Full modulator/demodulator roundtrip (no channel)
bool testModemRoundtrip() {
    std::cout << "Test 2: Full modem roundtrip (no channel)... ";

    OTFSConfig config;
    config.M = 32;
    config.N = 16;
    config.fft_size = 512;
    config.cp_length = 64;
    config.sample_rate = 48000;
    config.center_freq = 1500.0f;

    OTFSModulator mod(config);
    OTFSDemodulator demod(config);

    // Create known DD symbols
    std::vector<Complex> tx_dd(config.M * config.N);
    for (size_t i = 0; i < tx_dd.size(); ++i) {
        // QPSK constellation points
        int pattern = i % 4;
        float scale = 0.707f;
        switch (pattern) {
            case 0: tx_dd[i] = Complex(-scale, -scale); break;
            case 1: tx_dd[i] = Complex(-scale,  scale); break;
            case 2: tx_dd[i] = Complex( scale, -scale); break;
            case 3: tx_dd[i] = Complex( scale,  scale); break;
        }
    }

    // Modulate
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(tx_dd, Modulation::QPSK);

    Samples tx_audio;
    tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
    tx_audio.insert(tx_audio.end(), data.begin(), data.end());

    // Normalize
    float max_val = 0;
    for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : tx_audio) s *= 0.5f / max_val;
    }

    // Demodulate (no channel - direct loopback)
    bool ready = false;
    size_t chunk = 1024;
    for (size_t off = 0; off < tx_audio.size() && !ready; off += chunk) {
        size_t len = std::min(chunk, tx_audio.size() - off);
        ready = demod.process(SampleSpan(tx_audio.data() + off, len));
    }

    if (!ready) {
        std::cout << "FAILED (demod not ready)\n";
        return false;
    }

    auto rx_dd = demod.getDDSymbols();
    if (rx_dd.size() != tx_dd.size()) {
        std::cout << "FAILED (size mismatch: " << rx_dd.size() << " vs " << tx_dd.size() << ")\n";
        return false;
    }

    // Check symbol recovery (allow for amplitude scaling)
    int errors = 0;
    for (size_t i = 0; i < tx_dd.size(); ++i) {
        // Hard decision on received symbol
        int tx_bits = 0;
        if (tx_dd[i].real() > 0) tx_bits |= 2;
        if (tx_dd[i].imag() > 0) tx_bits |= 1;

        int rx_bits = 0;
        if (rx_dd[i].real() > 0) rx_bits |= 2;
        if (rx_dd[i].imag() > 0) rx_bits |= 1;

        if (tx_bits != rx_bits) errors++;
    }

    if (errors > 0) {
        std::cout << "FAILED (" << errors << " symbol errors)\n";
        return false;
    }

    std::cout << "PASSED (0 symbol errors)\n";
    return true;
}

// Test 3: Full pipeline with LDPC (rigorous test per CLAUDE.md)
bool testFullPipelineWithLDPC() {
    std::cout << "Test 3: Full OTFS + LDPC pipeline... ";

    OTFSConfig config;
    config.M = 32;
    config.N = 16;
    config.fft_size = 512;
    config.cp_length = 64;
    config.sample_rate = 48000;
    config.center_freq = 1500.0f;

    OTFSModulator mod(config);
    OTFSDemodulator demod(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);
    Interleaver interleaver(24, 27);

    // Test data
    Bytes tx_data(40);
    for (size_t i = 0; i < tx_data.size(); ++i) {
        tx_data[i] = static_cast<uint8_t>((i * 17 + 7) & 0xFF);
    }

    // Encode
    Bytes encoded = encoder.encode(tx_data);
    Bytes interleaved = interleaver.interleave(encoded);

    // Map to DD symbols
    auto dd_symbols = mod.mapToDD(ByteSpan(interleaved.data(), interleaved.size()), Modulation::QPSK);

    // Modulate
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(dd_symbols, Modulation::QPSK);

    Samples tx_audio;
    tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
    tx_audio.insert(tx_audio.end(), data.begin(), data.end());

    // Normalize
    float max_val = 0;
    for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : tx_audio) s *= 0.5f / max_val;
    }

    // Demodulate (no channel)
    bool ready = false;
    size_t chunk = 1024;
    for (size_t off = 0; off < tx_audio.size() && !ready; off += chunk) {
        size_t len = std::min(chunk, tx_audio.size() - off);
        ready = demod.process(SampleSpan(tx_audio.data() + off, len));
    }

    if (!ready) {
        std::cout << "FAILED (demod not ready)\n";
        return false;
    }

    // Get soft bits and decode
    auto soft_bits = demod.getSoftBits();
    if (soft_bits.size() < 648) {
        std::cout << "FAILED (insufficient soft bits: " << soft_bits.size() << ")\n";
        return false;
    }
    soft_bits.resize(648);

    auto deinterleaved = interleaver.deinterleave(soft_bits);
    Bytes rx_data = decoder.decodeSoft(deinterleaved);

    if (!decoder.lastDecodeSuccess()) {
        std::cout << "FAILED (LDPC decode failed)\n";
        return false;
    }

    rx_data.resize(tx_data.size());
    if (rx_data != tx_data) {
        std::cout << "FAILED (data mismatch)\n";
        return false;
    }

    std::cout << "PASSED\n";
    return true;
}

// Test 4: Verify bit mapping consistency
bool testBitMapping() {
    std::cout << "Test 4: Bit mapping consistency... ";

    OTFSConfig config;
    config.M = 32;
    config.N = 16;
    OTFSModulator mod(config);

    // Test all possible 2-bit patterns for QPSK
    Bytes test_data = {0b00011011};  // All 4 patterns: 00, 01, 10, 11

    auto dd_symbols = mod.mapToDD(ByteSpan(test_data.data(), test_data.size()), Modulation::QPSK);

    // Verify first 4 symbols have correct constellation points
    float scale = 0.707f;
    Complex expected[] = {
        Complex(-scale, -scale),  // 00
        Complex(-scale,  scale),  // 01
        Complex( scale, -scale),  // 10
        Complex( scale,  scale),  // 11
    };

    for (int i = 0; i < 4; ++i) {
        if (!approxEqual(dd_symbols[i], expected[i])) {
            std::cout << "FAILED (symbol " << i << " mismatch)\n";
            std::cout << "  Expected: (" << expected[i].real() << ", " << expected[i].imag() << ")\n";
            std::cout << "  Got: (" << dd_symbols[i].real() << ", " << dd_symbols[i].imag() << ")\n";
            return false;
        }
    }

    std::cout << "PASSED\n";
    return true;
}

int main() {
    std::cout << "\n=== OTFS Unit Tests ===\n\n";

    int passed = 0, failed = 0;

    if (testTransformRoundtrip()) passed++; else failed++;
    if (testModemRoundtrip()) passed++; else failed++;
    if (testFullPipelineWithLDPC()) passed++; else failed++;
    if (testBitMapping()) passed++; else failed++;

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n\n";

    return failed > 0 ? 1 : 0;
}
