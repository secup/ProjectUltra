#include <iostream>
#include <vector>
#include <cmath>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main() {
    std::vector<uint8_t> test_data(21);
    uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
    for (size_t i = 0; i < test_data.size(); i++) test_data[i] = deadbeef[i % 4];

    ultra::ModemConfig config;
    config.modulation = ultra::Modulation::DQPSK;

    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(test_data);

    ultra::OFDMModulator modulator(config);
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, ultra::Modulation::DQPSK);

    std::vector<float> tx_signal;
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), modulated.begin(), modulated.end());

    float max_val = 0;
    for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : tx_signal) s *= 0.8f / max_val;

    ultra::OFDMDemodulator demodulator(config);
    std::vector<float> all_soft_bits;

    size_t chunk_size = 960;
    for (size_t offset = 0; offset < tx_signal.size(); offset += chunk_size) {
        size_t end = std::min(offset + chunk_size, tx_signal.size());
        ultra::SampleSpan chunk(tx_signal.data() + offset, end - offset);
        demodulator.process(chunk);

        if (demodulator.isSynced()) {
            auto bits = demodulator.getSoftBits();
            all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
        }
    }

    std::cout << "Input: " << test_data.size() << " bytes\n";
    std::cout << "Encoded: " << encoded.size() << " bytes\n";
    std::cout << "Received: " << all_soft_bits.size() << " soft bits\n\n";

    // Test 1: Multi-block with ALL bits (fails per earlier test)
    ultra::LDPCDecoder dec1(ultra::CodeRate::R1_4);
    auto res1 = dec1.decodeSoft(all_soft_bits);
    std::cout << "Multi-block ALL " << all_soft_bits.size() << " bits: "
              << (dec1.lastDecodeSuccess() ? "SUCCESS" : "FAILED")
              << " → " << res1.size() << " bytes\n";

    // Test 2: Multi-block with TRIMMED bits
    size_t n = 648;  // bits per codeword
    size_t num_cw = all_soft_bits.size() / n;
    size_t trimmed_size = num_cw * n;
    std::vector<float> trimmed(all_soft_bits.begin(), all_soft_bits.begin() + trimmed_size);

    ultra::LDPCDecoder dec2(ultra::CodeRate::R1_4);
    auto res2 = dec2.decodeSoft(trimmed);
    std::cout << "Multi-block TRIMMED " << trimmed_size << " bits: "
              << (dec2.lastDecodeSuccess() ? "SUCCESS" : "FAILED")
              << " → " << res2.size() << " bytes\n";

    if (dec2.lastDecodeSuccess()) {
        std::cout << "Decoded: ";
        for (size_t i = 0; i < std::min(res2.size(), (size_t)24); i++) {
            printf("%02X ", res2[i]);
        }
        std::cout << "\n";

        // Check match for original data length
        bool match = true;
        for (size_t i = 0; i < test_data.size() && i < res2.size(); i++) {
            if (res2[i] != test_data[i]) { match = false; break; }
        }
        std::cout << "First " << test_data.size() << " bytes: "
                  << (match ? "MATCH" : "MISMATCH") << "\n";
    }

    return 0;
}
