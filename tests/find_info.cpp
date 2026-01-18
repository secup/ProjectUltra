#include <iostream>
#include <vector>
#include <fstream>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main(int argc, char** argv) {
    if (argc < 2) return 1;
    
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
    
    std::vector<int> rx_bits;
    for (size_t offset = 0; offset < samples.size(); offset += 960) {
        size_t end = std::min(offset + (size_t)960, samples.size());
        ultra::SampleSpan chunk(samples.data() + offset, end - offset);
        demod.process(chunk);
        if (demod.isSynced()) {
            auto bits = demod.getSoftBits();
            for (float llr : bits) rx_bits.push_back(llr < 0 ? 1 : 0);
        }
    }
    std::cout << "RX bits: " << rx_bits.size() << "\n";
    
    // Expected info bits (first 162 bits = 20.25 bytes of frame)
    // The frame we expect
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x32, 0x00, 0x00, 0x00
    };
    
    // Convert to bits (first 162)
    std::vector<int> info_bits;
    for (uint8_t byte : frame) {
        for (int b = 7; b >= 0; --b) {
            info_bits.push_back((byte >> b) & 1);
            if (info_bits.size() >= 162) break;
        }
        if (info_bits.size() >= 162) break;
    }
    std::cout << "Info bits to search: " << info_bits.size() << "\n";
    
    // Find best match for info bits only
    int best_offset = -1;
    int min_errors = info_bits.size();
    
    for (size_t offset = 0; offset + info_bits.size() <= rx_bits.size(); offset++) {
        int errors = 0;
        for (size_t i = 0; i < info_bits.size(); i++) {
            if (rx_bits[offset + i] != info_bits[i]) errors++;
        }
        if (errors < min_errors) {
            min_errors = errors;
            best_offset = offset;
        }
        if (errors == 0) {
            printf("PERFECT match at offset %zu!\n", offset);
        } else if (errors <= 5) {
            printf("Near-perfect match at offset %zu: %d errors\n", offset, errors);
        }
    }
    
    std::cout << "\nBest info match at offset " << best_offset << ": " << min_errors << " errors\n";
    
    // Now check parity at that offset
    if (best_offset >= 0 && min_errors <= 5) {
        std::cout << "\nChecking parity at offset " << best_offset << "...\n";
        
        // Get expected full codeword
        ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
        auto encoded = encoder.encode(frame);
        std::vector<int> expected_cw;
        for (uint8_t byte : encoded) {
            for (int b = 7; b >= 0; --b) {
                expected_cw.push_back((byte >> b) & 1);
            }
        }
        
        // Compare parity portion (bits 162-647)
        int parity_errors = 0;
        for (int i = 162; i < 648 && best_offset + i < (int)rx_bits.size(); i++) {
            if (rx_bits[best_offset + i] != expected_cw[i]) parity_errors++;
        }
        std::cout << "Parity errors: " << parity_errors << " / 486\n";
        
        // Show what parity we received vs expected
        std::cout << "\nReceived parity (first 40 bits): ";
        for (int i = 162; i < 202 && best_offset + i < (int)rx_bits.size(); i++) {
            std::cout << rx_bits[best_offset + i];
        }
        std::cout << "\nExpected parity (first 40 bits): ";
        for (int i = 162; i < 202; i++) {
            std::cout << expected_cw[i];
        }
        std::cout << "\n";
    }
    
    return 0;
}
