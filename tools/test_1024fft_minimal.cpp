/**
 * Minimal 1024 FFT Test - Debug why NVIS mode fails
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

int main() {
    setLogLevel(LogLevel::DEBUG);  // Full debug output
    std::mt19937 rng(12345);

    std::cout << "=== Minimal 1024 FFT Test ===\n\n";

    // Test both 512 and 1024 FFT
    for (int fft_size : {512, 1024}) {
        std::cout << "--- Testing FFT size " << fft_size << " ---\n";

        ModemConfig config;
        config.sample_rate = 48000;
        config.center_freq = 1500;
        config.fft_size = fft_size;
        config.num_carriers = (fft_size == 512) ? 30 : 59;
        config.cp_mode = CyclicPrefixMode::MEDIUM;  // 48 or 96 samples
        config.symbol_guard = 0;
        config.pilot_spacing = 2;
        config.modulation = Modulation::DQPSK;
        config.code_rate = CodeRate::R1_2;
        config.use_pilots = false;  // DQPSK doesn't need pilots

        std::cout << "  FFT: " << config.fft_size
                  << ", Carriers: " << config.num_carriers
                  << ", CP: " << config.getCyclicPrefix()
                  << ", Symbol: " << config.getSymbolDuration() << " samples\n";

        OFDMModulator modulator(config);
        OFDMDemodulator demod(config);
        LDPCEncoder encoder(CodeRate::R1_2);
        LDPCDecoder decoder(CodeRate::R1_2);

        // Create test data (40 bytes for R1/2)
        Bytes data(40);
        for (auto& b : data) b = rng() & 0xFF;
        std::cout << "  Original data[0..3]: " << std::hex
                  << (int)data[0] << " " << (int)data[1] << " "
                  << (int)data[2] << " " << (int)data[3] << std::dec << "\n";

        // Encode
        Bytes encoded = encoder.encode(data);
        std::cout << "  Encoded size: " << encoded.size() << " bytes\n";

        // Modulate
        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(encoded, Modulation::DQPSK);
        std::cout << "  Preamble: " << preamble.size() << " samples\n";
        std::cout << "  Modulated: " << modulated.size() << " samples\n";

        // Combine
        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());
        std::cout << "  Total signal: " << signal.size() << " samples\n";

        // Normalize (like a real audio path)
        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        for (float& s : signal) s *= 0.5f / max_val;

        // Feed to demodulator
        std::cout << "  Feeding to demodulator...\n";
        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            demod.process(span);
        }

        // Get soft bits
        auto soft = demod.getSoftBits();
        std::cout << "  Soft bits: " << soft.size() << " (need 648)\n";

        if (soft.size() < 648) {
            std::cout << "  FAILED: Not enough soft bits\n\n";
            continue;
        }

        // Decode
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);

        bool success = decoder.lastDecodeSuccess();
        std::cout << "  LDPC decode: " << (success ? "OK" : "FAILED")
                  << ", decoded size: " << decoded.size() << "\n";

        if (success && decoded.size() >= 4) {
            std::cout << "  Decoded data[0..3]: " << std::hex
                      << (int)decoded[0] << " " << (int)decoded[1] << " "
                      << (int)decoded[2] << " " << (int)decoded[3] << std::dec << "\n";

            bool match = true;
            for (size_t i = 0; i < data.size() && i < decoded.size(); i++) {
                if (data[i] != decoded[i]) {
                    match = false;
                    std::cout << "  MISMATCH at byte " << i << ": expected "
                              << std::hex << (int)data[i] << " got " << (int)decoded[i] << std::dec << "\n";
                    break;
                }
            }
            if (match) {
                std::cout << "  Data MATCHES!\n";
            }
        }

        std::cout << "\n";
    }

    return 0;
}
