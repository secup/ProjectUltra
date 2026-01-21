/**
 * Throughput Test - Measures actual bytes per second for each mode
 *
 * This measures REAL throughput by:
 *   1. Creating a payload of known size
 *   2. Encoding through full TX chain (LDPC → Interleave → OFDM)
 *   3. Measuring audio duration
 *   4. Calculating: payload_bytes / audio_duration
 *
 * Compile: g++ -O2 -std=c++20 -I../src -I../include -o test_throughput ../tools/test_throughput.cpp -L. -lultra_core -lfftw3f -lm
 * Run: ./test_throughput
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>

using namespace ultra;

// Get info bits per LDPC codeword for each code rate
size_t getInfoBits(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 162;
        case CodeRate::R1_2: return 324;
        case CodeRate::R2_3: return 432;
        case CodeRate::R3_4: return 486;
        case CodeRate::R5_6: return 540;
        default: return 324;
    }
}

// Get usable bytes per codeword (info bits / 8, minus header overhead)
size_t getPayloadBytes(CodeRate rate) {
    // v2 frame uses 2 bytes marker+index per data codeword
    // So usable payload = info_bytes - 2
    size_t info_bytes = getInfoBits(rate) / 8;
    return info_bytes > 2 ? info_bytes - 2 : 0;
}

// Measure throughput for a specific mode
struct ThroughputResult {
    Modulation mod;
    CodeRate rate;
    size_t payload_bytes;
    size_t audio_samples;
    float audio_duration_ms;
    float throughput_bps;
    float throughput_kbps;
};

ThroughputResult measureThroughput(Modulation mod, CodeRate rate, size_t num_codewords = 10) {
    ThroughputResult result;
    result.mod = mod;
    result.rate = rate;

    // Setup modem config
    ModemConfig config;
    config.modulation = mod;
    config.code_rate = rate;
    config.sample_rate = 48000;
    config.num_carriers = 30;
    config.fft_size = 512;
    config.cp_mode = CyclicPrefixMode::MEDIUM;

    // DQPSK doesn't need pilots - use all 30 carriers for data!
    if (mod == Modulation::DQPSK) {
        config.use_pilots = false;  // All 30 carriers are data
    } else {
        config.use_pilots = true;
        config.pilot_spacing = 4;   // Sparser pilots for QAM (25% overhead instead of 50%)
    }

    // Create encoder and modulator
    LDPCEncoder encoder(rate);
    OFDMModulator modulator(config);
    Interleaver interleaver(6, 108);

    // Calculate payload per codeword
    size_t payload_per_cw = getPayloadBytes(rate);
    result.payload_bytes = payload_per_cw * num_codewords;

    // Create test data
    std::vector<uint8_t> payload(result.payload_bytes);
    for (size_t i = 0; i < payload.size(); i++) {
        payload[i] = static_cast<uint8_t>(i & 0xFF);
    }

    // Generate preamble (sent once per frame)
    Samples preamble = modulator.generatePreamble();

    // Encode all codewords
    Samples all_modulated;
    size_t info_bytes = getInfoBits(rate) / 8;

    for (size_t cw = 0; cw < num_codewords; cw++) {
        // Create codeword data (header + payload)
        std::vector<uint8_t> cw_data(info_bytes, 0);

        // Add marker + index (2 bytes overhead per codeword)
        cw_data[0] = 0xD5;  // Data marker
        cw_data[1] = static_cast<uint8_t>(cw);

        // Copy payload
        size_t payload_offset = cw * payload_per_cw;
        size_t copy_len = std::min(payload_per_cw, result.payload_bytes - payload_offset);
        if (copy_len > 0) {
            std::memcpy(cw_data.data() + 2, payload.data() + payload_offset, copy_len);
        }

        // LDPC encode
        Bytes encoded = encoder.encode(cw_data);

        // Interleave
        Bytes interleaved = interleaver.interleave(encoded);

        // Modulate
        Samples modulated = modulator.modulate(interleaved, mod);
        all_modulated.insert(all_modulated.end(), modulated.begin(), modulated.end());
    }

    // Total audio = preamble + all modulated data
    result.audio_samples = preamble.size() + all_modulated.size();
    result.audio_duration_ms = (result.audio_samples * 1000.0f) / config.sample_rate;

    // Calculate throughput
    float duration_sec = result.audio_duration_ms / 1000.0f;
    result.throughput_bps = (result.payload_bytes * 8.0f) / duration_sec;
    result.throughput_kbps = result.throughput_bps / 1000.0f;

    return result;
}

const char* modToStr(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK:   return "BPSK";
        case Modulation::QPSK:   return "QPSK";
        case Modulation::DQPSK:  return "DQPSK";
        case Modulation::QAM16:  return "16-QAM";
        case Modulation::QAM64:  return "64-QAM";
        case Modulation::QAM256: return "256-QAM";
        default: return "???";
    }
}

// Calculate theoretical throughput (GUI formula - no overhead)
float theoreticalThroughput(Modulation mod, CodeRate rate, bool use_pilots = true, int pilot_spacing = 4) {
    // Symbol parameters
    float symbol_duration = 512.0f + 48.0f;  // FFT + CP
    float symbol_rate = 48000.0f / symbol_duration;

    // Data carriers
    int num_carriers = 30;
    int data_carriers;

    // DQPSK doesn't need pilots - use all 30 carriers!
    if (mod == Modulation::DQPSK || !use_pilots) {
        data_carriers = num_carriers;  // All 30 carriers for DQPSK
    } else {
        // QAM modes need pilots for channel estimation
        int pilots = (num_carriers + pilot_spacing - 1) / pilot_spacing;
        data_carriers = num_carriers - pilots;
    }

    // Bits per symbol
    int bits_per_carrier;
    switch (mod) {
        case Modulation::BPSK:   bits_per_carrier = 1; break;
        case Modulation::QPSK:
        case Modulation::DQPSK:  bits_per_carrier = 2; break;
        case Modulation::QAM16:  bits_per_carrier = 4; break;
        case Modulation::QAM64:  bits_per_carrier = 6; break;
        case Modulation::QAM256: bits_per_carrier = 8; break;
        default: bits_per_carrier = 2;
    }

    // Code rate value
    float code_rate_val;
    switch (rate) {
        case CodeRate::R1_4: code_rate_val = 0.25f; break;
        case CodeRate::R1_2: code_rate_val = 0.5f; break;
        case CodeRate::R2_3: code_rate_val = 0.667f; break;
        case CodeRate::R3_4: code_rate_val = 0.75f; break;
        case CodeRate::R5_6: code_rate_val = 0.833f; break;
        default: code_rate_val = 0.5f;
    }

    return data_carriers * bits_per_carrier * code_rate_val * symbol_rate;
}

int main() {
    std::cout << "=============================================================================\n";
    std::cout << "               ProjectUltra Throughput Test\n";
    std::cout << "=============================================================================\n\n";

    std::cout << "System Parameters:\n";
    std::cout << "  Sample rate:     48000 Hz\n";
    std::cout << "  FFT size:        512\n";
    std::cout << "  Cyclic prefix:   48 samples (MEDIUM)\n";
    std::cout << "  Total carriers:  30\n";
    std::cout << "  LDPC block:      648 bits\n";
    std::cout << "  Interleaver:     6x108\n";
    std::cout << "\n";

    // Calculate symbol rate
    float symbol_duration = 512.0f + 48.0f;  // FFT + CP
    float symbol_rate = 48000.0f / symbol_duration;
    std::cout << "  Symbol duration: " << symbol_duration << " samples ("
              << std::fixed << std::setprecision(2) << (symbol_duration * 1000.0f / 48000.0f) << " ms)\n";
    std::cout << "  Symbol rate:     " << std::setprecision(1)
              << symbol_rate << " symbols/sec\n";
    std::cout << "  Data carriers:   30 for DQPSK (no pilots), 22-23 for QAM (pilot_spacing=4)\n\n";

    // Test configurations (modes we actually use)
    struct TestMode {
        Modulation mod;
        CodeRate rate;
        const char* description;
    };

    std::vector<TestMode> modes = {
        // Low SNR modes (reliable)
        {Modulation::DQPSK, CodeRate::R1_4, "Low SNR (<12 dB)"},
        {Modulation::DQPSK, CodeRate::R1_2, "Medium SNR (12-18 dB)"},
        {Modulation::DQPSK, CodeRate::R2_3, "Good SNR (18-22 dB)"},
        {Modulation::DQPSK, CodeRate::R3_4, "High SNR (22-26 dB)"},

        // Higher order modulation (high SNR)
        {Modulation::QAM16, CodeRate::R1_2, "Very good SNR (25-30 dB)"},
        {Modulation::QAM16, CodeRate::R2_3, "Excellent SNR (28-32 dB)"},
        {Modulation::QAM16, CodeRate::R3_4, "Excellent SNR (30+ dB)"},

        // Maximum throughput modes
        {Modulation::QAM64, CodeRate::R1_2, "High SNR (32+ dB)"},
        {Modulation::QAM64, CodeRate::R3_4, "Very high SNR (35+ dB)"},
    };

    std::cout << "=============================================================================\n";
    std::cout << std::setw(10) << "Mode"
              << std::setw(8) << "Rate"
              << std::setw(12) << "Theoretical"
              << std::setw(12) << "Measured"
              << std::setw(10) << "Effic"
              << "  Description\n";
    std::cout << std::setw(10) << ""
              << std::setw(8) << ""
              << std::setw(12) << "(kbps)"
              << std::setw(12) << "(kbps)"
              << std::setw(10) << "(%)"
              << "\n";
    std::cout << "-----------------------------------------------------------------------------\n";

    for (const auto& mode : modes) {
        auto result = measureThroughput(mode.mod, mode.rate, 10);  // 10 codewords
        // DQPSK: no pilots (all 30 carriers), QAM: pilots with spacing=4
        bool use_pilots = (mode.mod != Modulation::DQPSK);
        float theoretical = theoreticalThroughput(mode.mod, mode.rate, use_pilots, 4) / 1000.0f;
        float efficiency = (result.throughput_kbps / theoretical) * 100.0f;

        std::cout << std::setw(10) << modToStr(mode.mod)
                  << std::setw(8) << codeRateToString(mode.rate)
                  << std::setw(12) << std::fixed << std::setprecision(2) << theoretical
                  << std::setw(12) << std::setprecision(2) << result.throughput_kbps
                  << std::setw(10) << std::setprecision(0) << efficiency
                  << "  " << mode.description << "\n";
    }

    std::cout << "=============================================================================\n\n";

    // Summary with adaptive modes
    std::cout << "\nADAPTIVE MODE THROUGHPUT (actual modes used by protocol):\n";
    std::cout << "=============================================================================\n";
    std::cout << std::setw(12) << "SNR Range"
              << std::setw(14) << "Mode"
              << std::setw(12) << "Measured"
              << std::setw(12) << "Effective*"
              << "\n";
    std::cout << std::setw(12) << "(dB)"
              << std::setw(14) << ""
              << std::setw(12) << "(kbps)"
              << std::setw(12) << "(kbps)"
              << "\n";
    std::cout << "-----------------------------------------------------------------------------\n";

    // Adaptive mode thresholds from CLAUDE.md
    std::vector<TestMode> adaptive_modes = {
        {Modulation::DQPSK, CodeRate::R1_4, "< 16 dB"},
        {Modulation::DQPSK, CodeRate::R1_2, "16-20 dB"},
        {Modulation::DQPSK, CodeRate::R2_3, "20-25 dB"},
        {Modulation::QAM16, CodeRate::R2_3, "25-30 dB"},
        {Modulation::QAM16, CodeRate::R3_4, "> 30 dB"},
    };

    for (const auto& mode : adaptive_modes) {
        auto result = measureThroughput(mode.mod, mode.rate, 50);
        float effective = result.throughput_kbps * 0.80f;  // ~80% with protocol overhead

        std::string mode_str = std::string(modToStr(mode.mod)) + " " + codeRateToString(mode.rate);
        std::cout << std::setw(12) << mode.description
                  << std::setw(14) << mode_str
                  << std::setw(12) << std::fixed << std::setprecision(2) << result.throughput_kbps
                  << std::setw(12) << std::setprecision(2) << effective
                  << "\n";
    }

    std::cout << "-----------------------------------------------------------------------------\n";
    std::cout << "* Effective = measured × 0.80 (accounts for ACKs, retransmits, negotiation)\n";
    std::cout << "=============================================================================\n";

    // Time to send 1KB
    std::cout << "\nTIME TO SEND 1 KB MESSAGE:\n";
    std::cout << "-----------------------------------------------------------------------------\n";
    for (const auto& mode : adaptive_modes) {
        auto result = measureThroughput(mode.mod, mode.rate, 50);
        float effective_bps = result.throughput_bps * 0.80f;
        float time_for_1kb = (1024.0f * 8.0f) / effective_bps;

        std::string mode_str = std::string(modToStr(mode.mod)) + " " + codeRateToString(mode.rate);
        std::cout << "  " << std::setw(14) << std::left << mode_str
                  << std::setw(10) << std::right << std::fixed << std::setprecision(1)
                  << time_for_1kb << " seconds\n";
    }
    std::cout << "=============================================================================\n";

    return 0;
}
