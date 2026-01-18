#include <iostream>
#include <vector>
#include "ultra/fec.hpp"

int main() {
    // Simulate a PROBE frame header (similar to what we saw in recording)
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52,  // MAGIC "ULTR"
        0x0A,                     // PROBE type
        0x00, 0x00,              // flags, seq
        'V','A','2','M','V','R',0,0,  // src callsign
        'T','E','S','T','2',0,0,0,    // dst callsign
        0x00, 0x02,              // length
        0x00, 0x00               // dummy payload
    };
    
    std::cout << "Input: " << frame.size() << " bytes\n";
    std::cout << "First 8 bytes: ";
    for (int i = 0; i < 8; i++) printf("%02X ", frame[i]);
    std::cout << "\n";
    
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    
    std::cout << "Encoded: " << encoded.size() << " bytes (expected ~" 
              << encoder.getCodedSize(frame.size()) << ")\n";
    std::cout << "First 8 encoded bytes: ";
    for (int i = 0; i < 8 && i < (int)encoded.size(); i++) printf("%02X ", encoded[i]);
    std::cout << "\n";
    
    // Check if it's systematic (first bytes should match input for R1/4)
    bool systematic = true;
    for (size_t i = 0; i < std::min(frame.size(), (size_t)20); i++) {
        if (encoded[i] != frame[i]) { systematic = false; break; }
    }
    std::cout << "Systematic (first 20 bytes match): " << (systematic ? "YES" : "NO") << "\n";
    
    return 0;
}
