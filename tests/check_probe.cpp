#include <iostream>
#include <vector>
#include "ultra/fec.hpp"
#include "../src/protocol/frame.hpp"

int main() {
    // Create same PROBE frame as GUI would send
    ultra::protocol::Frame probe = ultra::protocol::Frame::makeProbe("VA2MVR", "TEST5", 0x0F);
    auto serialized = probe.serialize();
    
    std::cout << "PROBE frame: " << serialized.size() << " bytes\n";
    std::cout << "Raw frame bytes:\n";
    for (size_t i = 0; i < serialized.size(); i++) {
        printf("%02X ", serialized[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n\n");
    
    // LDPC encode with R1/4
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(serialized);
    
    std::cout << "LDPC R1/4 encoded: " << encoded.size() << " bytes\n";
    std::cout << "Encoded bytes (COMPARE THIS WITH MAC):\n";
    for (size_t i = 0; i < encoded.size(); i++) {
        printf("%02X ", encoded[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
    
    return 0;
}
