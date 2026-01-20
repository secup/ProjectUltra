/**
 * Test DQPSK with all code rates through full OFDM chain
 *
 * Compile: g++ -O2 -std=c++20 -I../src -I../include -o test_dqpsk_rates ../tools/test_dqpsk_rates.cpp -L. -lultra_core -lfftw3f -lm
 * Run: ./test_dqpsk_rates
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"  // Includes Interleaver
#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <cstring>

using namespace ultra;

// Get info bits for each code rate (must match ldpc_encoder.cpp)
size_t getInfoBits(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 162;
        case CodeRate::R1_2: return 324;
        case CodeRate::R2_3: return 432;
        case CodeRate::R3_4: return 486;
        case CodeRate::R5_6: return 540;
        default: return 324;  // Default to R1/2
    }
}

// Test one code rate with DQPSK through full OFDM chain
bool testCodeRate(CodeRate rate, float snr_db = 20.0f) {
    const char* rate_str = codeRateToString(rate);

    // Get bytes per codeword for this rate
    size_t k_bits = getInfoBits(rate);
    size_t k_bytes = (k_bits + 7) / 8;  // Round up to bytes

    // Calculate actual bytes to compare (full bytes only, ignore padding)
    size_t full_bytes = k_bits / 8;

    // Create test data (fill to k_bytes for encoder, but only compare full_bytes)
    std::vector<uint8_t> tx_data(k_bytes);
    std::mt19937 rng(42);
    for (size_t i = 0; i < full_bytes; i++) {
        tx_data[i] = rng() & 0xFF;
    }
    // Zero padding bits
    for (size_t i = full_bytes; i < k_bytes; i++) {
        tx_data[i] = 0;
    }

    // Setup modem config
    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = rate;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.fft_size = 512;
    config.cp_mode = CyclicPrefixMode::MEDIUM;

    // Create encoder/decoder
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    // Create modulator/demodulator
    OFDMModulator modulator(config);
    OFDMDemodulator demodulator(config);

    // Create interleaver (6x108 for frequency diversity)
    Interleaver interleaver(6, 108);

    // Encode
    Bytes encoded = encoder.encode(tx_data);

    // Interleave
    Bytes interleaved = interleaver.interleave(encoded);

    // Generate preamble + modulated data
    Samples preamble = modulator.generatePreamble();
    Samples modulated = modulator.modulate(interleaved, config.modulation);

    // Combine preamble + data
    Samples audio;
    audio.reserve(preamble.size() + modulated.size());
    audio.insert(audio.end(), preamble.begin(), preamble.end());
    audio.insert(audio.end(), modulated.begin(), modulated.end());

    // Normalize
    float max_val = 0;
    for (float s : audio) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : audio) s *= scale;
    }

    // Add AWGN noise
    float signal_power = 0.0f;
    for (float s : audio) signal_power += s * s;
    signal_power /= audio.size();

    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : audio) {
        s += noise(rng);
    }

    // Demodulate in chunks (like real-time processing)
    std::vector<float> soft_bits;
    size_t chunk_size = 960;

    for (size_t i = 0; i < audio.size(); i += chunk_size) {
        size_t len = std::min(chunk_size, audio.size() - i);
        SampleSpan span(audio.data() + i, len);
        demodulator.process(span);

        if (demodulator.isSynced()) {
            auto bits = demodulator.getSoftBits();
            soft_bits.insert(soft_bits.end(), bits.begin(), bits.end());
        }
    }

    if (soft_bits.empty()) {
        std::cout << "  " << rate_str << " @ " << snr_db << " dB: FAIL (no sync)\n";
        return false;
    }

    // Check we have enough bits (648 per codeword)
    const size_t LDPC_BLOCK = 648;
    if (soft_bits.size() < LDPC_BLOCK) {
        std::cout << "  " << rate_str << " @ " << snr_db << " dB: FAIL (only "
                  << soft_bits.size() << "/" << LDPC_BLOCK << " bits)\n";
        return false;
    }

    // Truncate to one codeword for this test
    soft_bits.resize(LDPC_BLOCK);

    // Deinterleave
    auto deinterleaved = interleaver.deinterleave(soft_bits);

    // Decode
    Bytes rx_data = decoder.decodeSoft(deinterleaved);
    bool ldpc_ok = decoder.lastDecodeSuccess();

    if (!ldpc_ok) {
        std::cout << "  " << rate_str << " @ " << snr_db << " dB: FAIL (LDPC decode failed)\n";
        return false;
    }

    // Compare
    // Compare only full bytes (ignore padding bits)
    bool match = (rx_data.size() >= full_bytes) &&
                 (std::memcmp(tx_data.data(), rx_data.data(), full_bytes) == 0);

    if (match) {
        std::cout << "  " << rate_str << " @ " << snr_db << " dB: PASS ("
                  << full_bytes << " bytes, " << k_bits << " bits)\n";
    } else {
        // Find first mismatch
        size_t mismatch_pos = 0;
        size_t compare_len = std::min(tx_data.size(), rx_data.size());
        for (size_t i = 0; i < compare_len; i++) {
            if (tx_data[i] != rx_data[i]) {
                mismatch_pos = i;
                break;
            }
        }

        std::cout << "  " << rate_str << " @ " << snr_db << " dB: FAIL (mismatch at byte "
                  << mismatch_pos << ", tx_size=" << tx_data.size()
                  << ", rx_size=" << rx_data.size() << ")\n";

        // Show bytes around mismatch
        size_t start = (mismatch_pos > 4) ? mismatch_pos - 4 : 0;
        size_t end = std::min(mismatch_pos + 8, compare_len);
        std::cout << "    TX[" << start << "-" << end << "]: ";
        for (size_t i = start; i < end; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)tx_data[i] << " ";
        }
        std::cout << "\n    RX[" << start << "-" << end << "]: ";
        for (size_t i = start; i < end; i++) {
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << (int)(i < rx_data.size() ? rx_data[i] : 0) << " ";
        }
        std::cout << std::dec << "\n";
    }

    return match;
}

int main() {
    std::cout << "=== DQPSK Code Rate Test ===\n\n";

    // Test at clean SNR first (20 dB)
    std::cout << "Clean channel (20 dB SNR):\n";
    std::vector<CodeRate> rates = {
        CodeRate::R1_4,
        CodeRate::R1_2,
        CodeRate::R2_3,
        CodeRate::R3_4,
        CodeRate::R5_6,
    };

    int pass_count = 0;
    for (auto rate : rates) {
        if (testCodeRate(rate, 20.0f)) {
            pass_count++;
        }
    }

    std::cout << "\nResults: " << pass_count << "/" << rates.size() << " passed\n";

    // Test marginal conditions (raised SNR to account for sync detection needs)
    std::cout << "\n\nMarginal conditions (lower SNR):\n";
    std::cout << "  R1/4 @ 14 dB: " << (testCodeRate(CodeRate::R1_4, 14.0f) ? "PASS" : "FAIL") << "\n";
    std::cout << "  R1/2 @ 14 dB: " << (testCodeRate(CodeRate::R1_2, 14.0f) ? "PASS" : "FAIL") << "\n";
    std::cout << "  R2/3 @ 14 dB: " << (testCodeRate(CodeRate::R2_3, 14.0f) ? "PASS" : "FAIL") << "\n";
    std::cout << "  R3/4 @ 16 dB: " << (testCodeRate(CodeRate::R3_4, 16.0f) ? "PASS" : "FAIL") << "\n";
    std::cout << "  R5/6 @ 18 dB: " << (testCodeRate(CodeRate::R5_6, 18.0f) ? "PASS" : "FAIL") << "\n";

    // Throughput comparison
    std::cout << "\n\nThroughput comparison (per codeword):\n";
    std::cout << "  R1/4: " << 162 << " bits / 648 coded = 25% efficiency\n";
    std::cout << "  R1/2: " << 324 << " bits / 648 coded = 50% efficiency (2x R1/4)\n";
    std::cout << "  R2/3: " << 432 << " bits / 648 coded = 67% efficiency (2.7x R1/4)\n";
    std::cout << "  R3/4: " << 486 << " bits / 648 coded = 75% efficiency (3x R1/4)\n";
    std::cout << "  R5/6: " << 540 << " bits / 648 coded = 83% efficiency (3.3x R1/4)\n";

    return (pass_count == rates.size()) ? 0 : 1;
}
