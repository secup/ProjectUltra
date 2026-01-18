#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main(int argc, char** argv) {
    if (argc < 2) return 1;
    
    // Read recording and demodulate ALL soft bits
    std::ifstream file(argv[1], std::ios::binary);
    file.seekg(0, std::ios::end);
    size_t n = file.tellg() / sizeof(float);
    file.seekg(0);
    std::vector<float> samples(n);
    file.read(reinterpret_cast<char*>(samples.data()), n * sizeof(float));
    
    ultra::ModemConfig config;
    config.modulation = ultra::Modulation::DQPSK;
    ultra::OFDMDemodulator demod(config);
    demod.setTimingOffset(215);
    
    std::vector<float> rx_soft_bits;
    for (size_t offset = 0; offset < samples.size(); offset += 960) {
        size_t end = std::min(offset + (size_t)960, samples.size());
        ultra::SampleSpan chunk(samples.data() + offset, end - offset);
        demod.process(chunk);
        if (demod.isSynced()) {
            auto bits = demod.getSoftBits();
            rx_soft_bits.insert(rx_soft_bits.end(), bits.begin(), bits.end());
        }
    }
    std::cout << "Total RX soft bits: " << rx_soft_bits.size() << "\n";
    
    // Encode expected frame locally
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x32, 0x00, 0x00, 0x00
    };
    
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    std::cout << "Encoded frame: " << encoded.size() << " bytes = " << encoded.size() * 8 << " bits\n";
    
    // Convert to expected bits
    std::vector<int> expected_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) {
            expected_bits.push_back((byte >> b) & 1);
        }
    }
    
    // Hard decode RX bits
    std::vector<int> rx_bits;
    for (float llr : rx_soft_bits) {
        rx_bits.push_back(llr < 0 ? 1 : 0);
    }
    
    // Search for best match at each offset
    int best_offset = -1;
    int min_errors = expected_bits.size();
    
    std::cout << "\nSearching for encoded pattern in received bits...\n";
    for (size_t offset = 0; offset + expected_bits.size() <= rx_bits.size(); offset++) {
        int errors = 0;
        for (size_t i = 0; i < expected_bits.size(); i++) {
            if (rx_bits[offset + i] != expected_bits[i]) errors++;
        }
        if (errors < min_errors) {
            min_errors = errors;
            best_offset = offset;
            if (errors < 50) {
                printf("  Offset %5zu: %d errors (%.1f%%)\n", offset, errors, 100.0 * errors / expected_bits.size());
            }
        }
    }
    
    std::cout << "\nBest match at offset " << best_offset << ": " << min_errors << " errors ("
              << (100.0 * min_errors / expected_bits.size()) << "%)\n";
    
    // If good match found, try LDPC decode from that offset
    if (best_offset >= 0 && min_errors < 100) {
        std::cout << "\nTrying LDPC decode from offset " << best_offset << "...\n";
        std::vector<float> aligned(rx_soft_bits.begin() + best_offset,
                                   rx_soft_bits.begin() + best_offset + 648);
        ultra::LDPCDecoder decoder(ultra::CodeRate::R1_4);
        auto decoded = decoder.decodeSoft(aligned);
        std::cout << "LDPC decode: " << (decoder.lastDecodeSuccess() ? "SUCCESS!" : "FAILED") << "\n";
        if (decoder.lastDecodeSuccess()) {
            std::cout << "Decoded: ";
            for (int i = 0; i < 8 && i < (int)decoded.size(); i++) printf("%02X ", decoded[i]);
            std::cout << "\n";
        }
    }
    
    return 0;
}
