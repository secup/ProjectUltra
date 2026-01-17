/**
 * Basic OFDM test - no LDPC, no differential encoding
 * Just preamble + QPSK symbols with DEADBEEF pattern
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// Simple WAV writer
bool writeWav(const std::string& filename, const std::vector<float>& samples, int rate = 48000) {
    std::ofstream f(filename, std::ios::binary);
    if (!f) return false;
    
    uint32_t data_size = samples.size() * 2;
    uint32_t file_size = 36 + data_size;
    uint16_t channels = 1, bits = 16;
    uint32_t byte_rate = rate * 2;
    
    f.write("RIFF", 4);
    f.write((char*)&file_size, 4);
    f.write("WAVE", 4);
    f.write("fmt ", 4);
    uint32_t fmt_size = 16;
    f.write((char*)&fmt_size, 4);
    uint16_t audio_fmt = 1;
    f.write((char*)&audio_fmt, 2);
    f.write((char*)&channels, 2);
    f.write((char*)&rate, 4);
    f.write((char*)&byte_rate, 4);
    uint16_t block_align = 2;
    f.write((char*)&block_align, 2);
    f.write((char*)&bits, 2);
    f.write("data", 4);
    f.write((char*)&data_size, 4);
    
    for (float s : samples) {
        int16_t v = std::max(-32767, std::min(32767, (int)(s * 32767)));
        f.write((char*)&v, 2);
    }
    return true;
}

// Simple WAV reader
bool readWav(const std::string& filename, std::vector<float>& samples) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) return false;
    f.seekg(44);
    std::vector<int16_t> raw(500000);
    f.read((char*)raw.data(), raw.size() * 2);
    size_t n = f.gcount() / 2;
    samples.resize(n);
    for (size_t i = 0; i < n; i++) samples[i] = raw[i] / 32768.0f;
    return true;
}

int main(int argc, char* argv[]) {
    std::cout << "=== Basic OFDM Test (No LDPC, QPSK) ===\n\n";
    
    ModemConfig config;
    config.modulation = Modulation::QPSK;  // Coherent QPSK, not differential
    config.pilot_spacing = 2;
    
    // Test pattern: DEADBEEF repeated
    std::vector<uint8_t> pattern = {0xDE, 0xAD, 0xBE, 0xEF};
    Bytes test_data;
    for (int i = 0; i < 10; i++) {  // 40 bytes = 320 bits
        test_data.insert(test_data.end(), pattern.begin(), pattern.end());
    }
    
    std::cout << "Test data: " << test_data.size() << " bytes (";
    for (int i = 0; i < 8; i++) printf("%02X ", test_data[i]);
    std::cout << "...)\n";
    
    if (argc >= 2 && strcmp(argv[1], "--generate") == 0) {
        // === TX: Generate signal ===
        OFDMModulator mod(config);
        
        Samples preamble = mod.generatePreamble();
        Samples data = mod.modulate(test_data, Modulation::QPSK);
        
        std::vector<float> tx;
        tx.insert(tx.end(), 12000, 0.0f);  // 0.25s silence
        tx.insert(tx.end(), preamble.begin(), preamble.end());
        tx.insert(tx.end(), data.begin(), data.end());
        tx.insert(tx.end(), 12000, 0.0f);  // 0.25s silence
        
        // Normalize
        float maxv = 0;
        for (float s : tx) maxv = std::max(maxv, std::abs(s));
        for (float& s : tx) s *= 0.5f / maxv;
        
        std::string outfile = (argc >= 3) ? argv[2] : "/tmp/basic_ofdm_tx.wav";
        writeWav(outfile, tx);
        std::cout << "Generated: " << outfile << " (" << tx.size() << " samples)\n";
        return 0;
    }
    
    if (argc >= 2 && strcmp(argv[1], "--decode") == 0) {
        // === RX: Decode signal ===
        std::string infile = (argc >= 3) ? argv[2] : "/tmp/basic_ofdm_tx.wav";
        
        std::vector<float> samples;
        if (!readWav(infile, samples)) {
            std::cerr << "Failed to read " << infile << "\n";
            return 1;
        }
        std::cout << "Read: " << samples.size() << " samples\n";
        
        OFDMDemodulator demod(config);
        
        // Process
        SampleSpan span(samples.data(), samples.size());
        demod.process(span);
        
        if (!demod.isSynced()) {
            // Try pumping more
            for (int i = 0; i < 100 && !demod.isSynced(); i++) {
                SampleSpan empty;
                demod.process(empty);
            }
        }
        
        if (!demod.isSynced()) {
            std::cout << "[FAIL] No sync!\n";
            return 1;
        }
        std::cout << "[OK] Synced\n";
        
        // Get soft bits
        std::vector<float> all_soft;
        for (int i = 0; i < 200; i++) {
            SampleSpan empty;
            demod.process(empty);
            auto bits = demod.getSoftBits();
            if (!bits.empty()) {
                all_soft.insert(all_soft.end(), bits.begin(), bits.end());
                std::cout << "Got " << bits.size() << " soft bits (total: " << all_soft.size() << ")\n";
            }
        }
        
        std::cout << "\nTotal soft bits: " << all_soft.size() << "\n";
        
        // Convert to hard bits and bytes
        std::vector<uint8_t> decoded;
        uint8_t byte = 0;
        int bit_count = 0;
        for (float llr : all_soft) {
            uint8_t bit = (llr < 0) ? 1 : 0;
            byte = (byte << 1) | bit;
            if (++bit_count == 8) {
                decoded.push_back(byte);
                byte = 0;
                bit_count = 0;
            }
        }
        
        std::cout << "Decoded " << decoded.size() << " bytes\n";
        std::cout << "First 20 bytes: ";
        for (int i = 0; i < 20 && i < (int)decoded.size(); i++) {
            printf("%02X ", decoded[i]);
        }
        std::cout << "\n";
        std::cout << "Expected:       ";
        for (int i = 0; i < 20; i++) {
            printf("%02X ", test_data[i % test_data.size()]);
        }
        std::cout << "\n";
        
        // Check match
        int matches = 0;
        for (size_t i = 0; i < std::min(decoded.size(), test_data.size()); i++) {
            if (decoded[i] == test_data[i]) matches++;
        }
        float match_pct = 100.0f * matches / std::min(decoded.size(), test_data.size());
        std::cout << "\nMatch: " << matches << "/" << std::min(decoded.size(), test_data.size()) 
                  << " (" << match_pct << "%)\n";
        
        if (match_pct > 90) {
            std::cout << "[PASS] Basic OFDM works!\n";
            return 0;
        } else {
            std::cout << "[FAIL] Data mismatch\n";
            return 1;
        }
    }
    
    std::cout << "Usage:\n";
    std::cout << "  " << argv[0] << " --generate [output.wav]  Generate test signal\n";
    std::cout << "  " << argv[0] << " --decode [input.wav]     Decode and verify\n";
    return 1;
}
