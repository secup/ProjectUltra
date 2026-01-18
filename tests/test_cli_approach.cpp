#include <iostream>
#include <vector>
#include <cmath>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main() {
    std::vector<uint8_t> test_data(21);
    uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
    for (size_t i = 0; i < test_data.size(); i++) test_data[i] = deadbeef[i % 4];

    std::cout << "Input: ";
    for (int i = 0; i < 8; i++) printf("%02X ", test_data[i]);
    std::cout << "\n";

    ultra::ModemConfig config;
    config.modulation = ultra::Modulation::DQPSK;

    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(test_data);

    std::cout << "Encoded: " << encoded.size() << " bytes = " << encoded.size() * 8 << " bits\n";

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

    std::cout << "Received: " << all_soft_bits.size() << " soft bits\n";

    // CLI approach: Extract 648 bits at a time, decode individually
    std::cout << "\n=== CLI approach (per-codeword decode) ===\n";
    const size_t LDPC_BLOCK_SIZE = 648;
    std::vector<uint8_t> all_decoded;
    size_t offset = 0;
    int codeword_num = 0;

    while (offset + LDPC_BLOCK_SIZE <= all_soft_bits.size()) {
        codeword_num++;
        std::vector<float> codeword_bits(all_soft_bits.begin() + offset,
                                          all_soft_bits.begin() + offset + LDPC_BLOCK_SIZE);
        offset += LDPC_BLOCK_SIZE;

        ultra::LDPCDecoder decoder(ultra::CodeRate::R1_4);
        auto decoded = decoder.decodeSoft(codeword_bits);

        std::cout << "Codeword " << codeword_num << " (" << LDPC_BLOCK_SIZE << " bits): "
                  << (decoder.lastDecodeSuccess() ? "SUCCESS" : "FAILED");

        if (decoder.lastDecodeSuccess()) {
            std::cout << " → ";
            for (int i = 0; i < 8 && i < (int)decoded.size(); i++) {
                printf("%02X ", decoded[i]);
            }
            all_decoded.insert(all_decoded.end(), decoded.begin(), decoded.end());
        }
        std::cout << "\n";
    }

    std::cout << "\nRemaining bits: " << (all_soft_bits.size() - offset) << "\n";
    std::cout << "Total decoded: " << all_decoded.size() << " bytes\n";

    if (!all_decoded.empty()) {
        std::cout << "Decoded data: ";
        for (int i = 0; i < 8 && i < (int)all_decoded.size(); i++) {
            printf("%02X ", all_decoded[i]);
        }

        bool match = true;
        for (size_t i = 0; i < test_data.size() && i < all_decoded.size(); i++) {
            if (all_decoded[i] != test_data[i]) { match = false; break; }
        }
        std::cout << " → " << (match ? "PERFECT MATCH!" : "MISMATCH") << "\n";
    }

    return 0;
}
