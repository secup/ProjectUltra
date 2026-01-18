#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include "ultra/fec.hpp"

int main() {
    // Test that encoder produces expected output
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x33, 0x00, 0x00, 0x00
    };
    
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    
    std::cout << "Input: " << frame.size() << " bytes\n";
    std::cout << "Encoded: " << encoded.size() << " bytes\n";
    
    // For R1/4: k=162 info bits, n=648 total bits per codeword
    // 23 bytes = 184 bits -> ceil(184/162) = 2 codewords
    // Output: 2 * 648 = 1296 bits = 162 bytes
    
    std::cout << "\nFirst codeword (bytes 0-80):\n";
    std::cout << "  Info part (bytes 0-19, 162 bits):\n    ";
    for (int i = 0; i < 20; i++) printf("%02X ", encoded[i]);
    std::cout << "\n  Parity part (bytes 20-80):\n    ";
    for (int i = 20; i < 40; i++) printf("%02X ", encoded[i]);
    std::cout << "...\n";
    
    // The key test: what are the first 8 parity bytes?
    // Info bits occupy bytes 0-19 (160 bits) + 2 bits of byte 20
    // Parity bits start at bit 162
    std::cout << "\nExpected parity (from encoder):\n";
    std::cout << "  Bytes 20-27: ";
    for (int i = 20; i < 28; i++) printf("%02X ", encoded[i]);
    std::cout << "\n";
    
    // What we received from Mac (at t=210):
    // 55 4C 54 52 0A 00 00 56 41 32 4D 56 52 00 00 54 45 53 54 33 2D D8 04 35 12 DC CA ED BC AC 5B 52
    std::cout << "\nReceived from Mac:\n";
    std::cout << "  Bytes 20-27: 2D D8 04 35 12 DC CA ED\n";
    
    return 0;
}
