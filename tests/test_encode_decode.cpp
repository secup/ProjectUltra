#include <iostream>
#include <vector>
#include "ultra/fec.hpp"

int main() {
    // Simulate exact frame from recording
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52,  // MAGIC "ULTR"
        0x0A, 0x00, 0x00,        // type, flags, seq
        'V','A','2','M','V','R',0,0,  // src
        'T','E','S','T','2',0,0,0     // dst
    };
    
    std::cout << "Original: " << frame.size() << " bytes: ";
    for (int i = 0; i < 8; i++) printf("%02X ", frame[i]);
    std::cout << "\n";
    
    // Encode
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    std::cout << "Encoded: " << encoded.size() << " bytes\n";
    
    // Convert to soft bits (perfect channel)
    std::vector<float> soft_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) {
            float bit = (byte >> b) & 1;
            soft_bits.push_back(bit ? -5.0f : 5.0f);  // LLR: positive=0, negative=1
        }
    }
    std::cout << "Soft bits: " << soft_bits.size() << "\n";
    
    // Decode
    ultra::LDPCDecoder decoder(ultra::CodeRate::R1_4);
    auto decoded = decoder.decodeSoft(soft_bits);
    
    std::cout << "Decode: " << (decoder.lastDecodeSuccess() ? "SUCCESS" : "FAILED") 
              << " → " << decoded.size() << " bytes\n";
    
    if (decoder.lastDecodeSuccess()) {
        std::cout << "Decoded: ";
        for (int i = 0; i < 8 && i < (int)decoded.size(); i++) printf("%02X ", decoded[i]);
        
        bool match = true;
        for (size_t i = 0; i < frame.size() && i < decoded.size(); i++) {
            if (decoded[i] != frame[i]) { match = false; break; }
        }
        std::cout << "→ " << (match ? "MATCH" : "MISMATCH") << "\n";
    }
    
    return 0;
}
