#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main(int argc, char** argv) {
    if (argc < 2) return 1;
    
    // Read recording and demodulate
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
            if (rx_soft_bits.size() >= 648) break;
        }
    }
    
    // Expected frame - encode locally
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x32, 0x00, 0x00, 0x00
    };
    
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    
    // Convert encoded to bits
    std::vector<uint8_t> expected_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) {
            expected_bits.push_back((byte >> b) & 1);
        }
    }
    
    // Hard decode received bits
    std::vector<uint8_t> rx_bits;
    for (float llr : rx_soft_bits) {
        rx_bits.push_back(llr < 0 ? 1 : 0);
    }
    
    // Compare: info bits (0-161) and parity bits (162-647)
    int info_errors = 0, parity_errors = 0;
    for (int i = 0; i < 162 && i < (int)rx_bits.size(); i++) {
        if (rx_bits[i] != expected_bits[i]) info_errors++;
    }
    for (int i = 162; i < 648 && i < (int)rx_bits.size() && i < (int)expected_bits.size(); i++) {
        if (rx_bits[i] != expected_bits[i]) parity_errors++;
    }
    
    std::cout << "Info bits (0-161):    " << info_errors << " errors\n";
    std::cout << "Parity bits (162-647): " << parity_errors << " errors\n";
    
    // Show first few parity bytes
    std::cout << "\nFirst 8 parity bytes (expected vs received):\n";
    for (int byte_idx = 0; byte_idx < 8; byte_idx++) {
        int bit_start = 162 + byte_idx * 8;  // Parity starts at bit 162
        if (bit_start + 8 > (int)rx_bits.size()) break;
        
        uint8_t exp_byte = 0, rx_byte = 0;
        for (int b = 0; b < 8; b++) {
            exp_byte = (exp_byte << 1) | expected_bits[bit_start + b];
            rx_byte = (rx_byte << 1) | rx_bits[bit_start + b];
        }
        printf("  Byte %d: expected=%02X received=%02X %s\n", 
               byte_idx, exp_byte, rx_byte, (exp_byte == rx_byte) ? "" : "MISMATCH");
    }
    
    return 0;
}
