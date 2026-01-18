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
    
    // Expected frame with TEST3
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x33, 0x00, 0x00, 0x00
    };
    std::vector<int> expected;
    for (uint8_t byte : frame) {
        for (int b = 7; b >= 0; --b) expected.push_back((byte >> b) & 1);
    }
    
    // Encode to get expected parity
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    std::vector<int> expected_cw;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) expected_cw.push_back((byte >> b) & 1);
    }
    
    // Process with fresh demodulator, collecting each frame separately
    int frame_num = 0;
    for (int t_off = 175; t_off <= 225; t_off += 25) {
        ultra::OFDMDemodulator demod(config);
        demod.setTimingOffset(t_off);
        
        std::vector<float> all_soft;
        bool was_synced = false;
        
        for (size_t offset = 0; offset < samples.size(); offset += 960) {
            size_t end = std::min(offset + (size_t)960, samples.size());
            ultra::SampleSpan chunk(samples.data() + offset, end - offset);
            demod.process(chunk);
            
            bool synced = demod.isSynced();
            
            // Detect frame boundaries
            if (synced && !was_synced) {
                frame_num++;
                all_soft.clear();
            }
            
            if (synced) {
                auto bits = demod.getSoftBits();
                all_soft.insert(all_soft.end(), bits.begin(), bits.end());
                
                // When we have enough for analysis
                if (all_soft.size() >= 648 && all_soft.size() < 700) {
                    // Convert to hard bits
                    std::vector<int> rx_bits;
                    for (float llr : all_soft) rx_bits.push_back(llr < 0 ? 1 : 0);
                    
                    // Count errors
                    int info_errors = 0, parity_errors = 0;
                    for (int i = 0; i < 162 && i < (int)rx_bits.size(); i++) {
                        if (rx_bits[i] != expected_cw[i]) info_errors++;
                    }
                    for (int i = 162; i < 648 && i < (int)rx_bits.size(); i++) {
                        if (rx_bits[i] != expected_cw[i]) parity_errors++;
                    }
                    
                    printf("Frame %d (t=%d): info_errors=%d, parity_errors=%d\n", 
                           frame_num, t_off, info_errors, parity_errors);
                    
                    // Try LDPC decode
                    std::vector<float> cw(all_soft.begin(), all_soft.begin() + 648);
                    ultra::LDPCDecoder decoder(ultra::CodeRate::R1_4);
                    auto decoded = decoder.decodeSoft(cw);
                    printf("  LDPC: %s\n", decoder.lastDecodeSuccess() ? "SUCCESS!" : "failed");
                    
                    if (decoder.lastDecodeSuccess()) {
                        printf("  Decoded: ");
                        for (int i = 0; i < 16 && i < (int)decoded.size(); i++) 
                            printf("%02X ", decoded[i]);
                        printf("\n");
                    }
                }
            }
            was_synced = synced;
        }
    }
    
    return 0;
}
