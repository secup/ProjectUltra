#include <iostream>
#include <vector>
#include <fstream>
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

int main() {
    std::ifstream file("/tmp/recording3_frame2.f32", std::ios::binary);
    file.seekg(0, std::ios::end);
    size_t n = file.tellg() / sizeof(float);
    file.seekg(0);
    std::vector<float> samples(n);
    file.read(reinterpret_cast<char*>(samples.data()), n * sizeof(float));
    
    ultra::ModemConfig config;
    config.modulation = ultra::Modulation::DQPSK;
    ultra::OFDMDemodulator demod(config);
    demod.setTimingOffset(210);
    
    std::vector<float> rx_soft;
    for (size_t offset = 0; offset < samples.size(); offset += 960) {
        size_t end = std::min(offset + (size_t)960, samples.size());
        ultra::SampleSpan chunk(samples.data() + offset, end - offset);
        demod.process(chunk);
        if (demod.isSynced()) {
            auto bits = demod.getSoftBits();
            rx_soft.insert(rx_soft.end(), bits.begin(), bits.end());
            if (rx_soft.size() >= 1296) break;
        }
    }
    
    // Expected frame with TEST3
    std::vector<uint8_t> frame = {
        0x55, 0x4C, 0x54, 0x52, 0x0A, 0x00, 0x00, 0x56,
        0x41, 0x32, 0x4D, 0x56, 0x52, 0x00, 0x00, 0x54,
        0x45, 0x53, 0x54, 0x33, 0x00, 0x00, 0x00
    };
    
    ultra::LDPCEncoder encoder(ultra::CodeRate::R1_4);
    auto encoded = encoder.encode(frame);
    
    std::vector<int> expected_cw, rx_bits;
    for (uint8_t byte : encoded) {
        for (int b = 7; b >= 0; --b) expected_cw.push_back((byte >> b) & 1);
    }
    for (float llr : rx_soft) rx_bits.push_back(llr < 0 ? 1 : 0);
    
    int info_err = 0, parity_err = 0;
    for (int i = 0; i < 162; i++) if (rx_bits[i] != expected_cw[i]) info_err++;
    for (int i = 162; i < 648; i++) if (rx_bits[i] != expected_cw[i]) parity_err++;
    
    printf("Info errors: %d / 162\n", info_err);
    printf("Parity errors: %d / 486\n", parity_err);
    
    if (info_err == 0 && parity_err < 50) {
        printf("\nTrying LDPC decode...\n");
        std::vector<float> cw(rx_soft.begin(), rx_soft.begin() + 648);
        ultra::LDPCDecoder decoder(ultra::CodeRate::R1_4);
        auto decoded = decoder.decodeSoft(cw);
        printf("LDPC: %s\n", decoder.lastDecodeSuccess() ? "SUCCESS!" : "failed");
        if (decoder.lastDecodeSuccess()) {
            printf("Decoded: ");
            for (int i = 0; i < 20 && i < (int)decoded.size(); i++) printf("%02X ", decoded[i]);
            printf("\n");
        }
    }
    
    return 0;
}
