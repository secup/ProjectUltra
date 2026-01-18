#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <recording.f32>\n";
        return 1;
    }
    
    // Read recording
    std::ifstream file(argv[1], std::ios::binary);
    file.seekg(0, std::ios::end);
    size_t n = file.tellg() / sizeof(float);
    file.seekg(0);
    std::vector<float> samples(n);
    file.read(reinterpret_cast<char*>(samples.data()), n * sizeof(float));
    
    // Demodulate
    ultra::ModemConfig config;
    config.modulation = ultra::Modulation::DQPSK;
    ultra::OFDMDemodulator demod(config);
    demod.setTimingOffset(215);
    
    std::vector<float> all_soft_bits;
    size_t chunk_size = 960;
    for (size_t offset = 0; offset < samples.size(); offset += chunk_size) {
        size_t end = std::min(offset + chunk_size, samples.size());
        ultra::SampleSpan chunk(samples.data() + offset, end - offset);
        demod.process(chunk);
        
        if (demod.isSynced()) {
            auto bits = demod.getSoftBits();
            all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
            if (all_soft_bits.size() >= 1296) break;  // 2 codewords
        }
    }
    
    std::cout << "Got " << all_soft_bits.size() << " soft bits\n\n";
    
    if (all_soft_bits.size() < 648) {
        std::cerr << "Not enough bits!\n";
        return 1;
    }
    
    // Expected frame (first 23 bytes)
    std::vector<uint8_t> expected = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x32, 0x00, 0x00, 0x00
    };
    
    // Hard decode first 20 bytes from LLRs
    std::cout << "First 20 bytes - Expected vs Hard Decoded:\n";
    int total_bit_errors = 0;
    for (int byte_idx = 0; byte_idx < 20 && byte_idx * 8 < (int)all_soft_bits.size(); byte_idx++) {
        uint8_t decoded_byte = 0;
        int bit_errors = 0;
        for (int b = 0; b < 8; b++) {
            int bit_idx = byte_idx * 8 + b;
            uint8_t expected_bit = (expected[byte_idx] >> (7 - b)) & 1;
            uint8_t decoded_bit = (all_soft_bits[bit_idx] < 0) ? 1 : 0;
            decoded_byte = (decoded_byte << 1) | decoded_bit;
            if (expected_bit != decoded_bit) bit_errors++;
        }
        total_bit_errors += bit_errors;
        printf("  Byte %2d: expected=%02X decoded=%02X errors=%d\n", 
               byte_idx, expected[byte_idx], decoded_byte, bit_errors);
    }
    std::cout << "\nTotal bit errors in first 160 bits: " << total_bit_errors << "\n";
    
    // Try LDPC decode on first codeword
    std::vector<float> cw1(all_soft_bits.begin(), all_soft_bits.begin() + 648);
    ultra::LDPCDecoder decoder(ultra::CodeRate::R1_4);
    auto decoded = decoder.decodeSoft(cw1);
    std::cout << "\nLDPC decode (first 648 bits): " 
              << (decoder.lastDecodeSuccess() ? "SUCCESS" : "FAILED") << "\n";
    
    return 0;
}
