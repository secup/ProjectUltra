/**
 * Test bit-level debugging for 1024 FFT
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
    setLogLevel(LogLevel::WARN);

    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;
    cfg.symbol_guard = 0;
    cfg.pilot_spacing = 2;
    cfg.modulation = Modulation::DQPSK;
    cfg.code_rate = CodeRate::R1_2;
    cfg.use_pilots = false;

    // Use failing data pattern
    std::mt19937 rng(12345);
    Bytes data(40);
    for (auto& b : data) b = rng() & 0xFF;

    std::cout << "Testing 1024 FFT bit-level debug\n";
    std::cout << "Input data[0..3]: " << std::hex
              << (int)data[0] << " " << (int)data[1] << " "
              << (int)data[2] << " " << (int)data[3] << std::dec << "\n\n";

    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);
    Bytes encoded = encoder.encode(data);

    std::cout << "Encoded: " << encoded.size() << " bytes (" << (encoded.size()*8) << " bits)\n";

    // Convert encoded bytes to bits
    std::vector<int> tx_bits;
    for (uint8_t byte : encoded) {
        for (int i = 7; i >= 0; i--) {
            tx_bits.push_back((byte >> i) & 1);
        }
    }
    std::cout << "TX bits (first 20): ";
    for (int i = 0; i < 20 && i < tx_bits.size(); i++) std::cout << tx_bits[i];
    std::cout << "\n\n";

    // Modulate and demodulate
    OFDMModulator modulator(cfg);
    OFDMDemodulator demod(cfg);

    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    std::cout << "Soft bits: " << soft.size() << "\n";

    // Convert soft bits to hard bits
    std::vector<int> rx_bits;
    for (float s : soft) {
        rx_bits.push_back(s < 0 ? 1 : 0);  // Negative LLR = bit 1
    }

    std::cout << "RX bits (first 20): ";
    for (int i = 0; i < 20 && i < rx_bits.size(); i++) std::cout << rx_bits[i];
    std::cout << "\n\n";

    // Count bit errors (compare first 648 bits)
    int errors = 0;
    std::cout << "First bit differences:\n";
    for (int i = 0; i < 648 && i < tx_bits.size() && i < rx_bits.size(); i++) {
        if (tx_bits[i] != rx_bits[i]) {
            errors++;
            if (errors <= 20) {
                std::cout << "  Bit " << i << ": TX=" << tx_bits[i] << " RX=" << rx_bits[i]
                          << " (sym=" << (i/2/59) << ", car=" << ((i/2)%59) << ", bit=" << (i%2) << ")\n";
            }
        }
    }
    std::cout << "\nTotal bit errors: " << errors << " / 648\n";

    // Try to decode
    if (soft.size() >= 648) {
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);
        std::cout << "\nLDPC decode: " << (decoder.lastDecodeSuccess() ? "OK" : "FAIL") << "\n";
    }

    return 0;
}
