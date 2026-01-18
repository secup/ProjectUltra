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
    
    // Try multiple timing offsets
    for (int t_off = 0; t_off <= 300; t_off += 25) {
        ultra::OFDMDemodulator demod(config);
        demod.setTimingOffset(t_off);
        
        std::vector<int> rx_bits;
        for (size_t offset = 0; offset < samples.size(); offset += 960) {
            size_t end = std::min(offset + (size_t)960, samples.size());
            ultra::SampleSpan chunk(samples.data() + offset, end - offset);
            demod.process(chunk);
            if (demod.isSynced()) {
                auto bits = demod.getSoftBits();
                for (float llr : bits) rx_bits.push_back(llr < 0 ? 1 : 0);
                if (rx_bits.size() >= 2000) break;
            }
        }
        
        if (rx_bits.size() < 162) continue;
        
        // Expected frame with TEST3
        std::vector<uint8_t> frame = {
            0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
            0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
            0x45, 0x53, 0x54, 0x33, 0x00, 0x00, 0x00
        };
        
        // Convert to bits
        std::vector<int> expected;
        for (uint8_t byte : frame) {
            for (int b = 7; b >= 0; --b) expected.push_back((byte >> b) & 1);
        }
        
        // Check first 184 bits (23 bytes frame)
        int errors = 0;
        for (size_t i = 0; i < expected.size() && i < rx_bits.size(); i++) {
            if (rx_bits[i] != expected[i]) errors++;
        }
        
        if (errors < 30) {
            printf("t=%3d: %d errors in first %zu bits\n", t_off, errors, expected.size());
        }
    }
    
    return 0;
}
