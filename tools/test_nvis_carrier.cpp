/**
 * Debug carrier-level TX/RX for 1024 FFT
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

int main() {
    setLogLevel(LogLevel::DEBUG);

    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;
    cfg.symbol_guard = 0;
    cfg.pilot_spacing = 2;
    cfg.modulation = Modulation::DQPSK;
    cfg.code_rate = CodeRate::R1_2;
    cfg.use_pilots = false;

    std::cout << "Testing 1024 FFT carrier-level debug\n";
    std::cout << "FFT=" << cfg.fft_size << ", carriers=" << cfg.num_carriers << ", CP=" << cfg.getCyclicPrefix() << "\n\n";

    // Use failing data pattern
    std::mt19937 rng(12345);
    Bytes data(40);
    for (auto& b : data) b = rng() & 0xFF;

    LDPCEncoder encoder(CodeRate::R1_2);
    Bytes encoded = encoder.encode(data);
    std::cout << "Encoded " << encoded.size() << " bytes (" << (encoded.size()*8) << " bits)\n";

    // Show first few encoded bytes
    std::cout << "Encoded[0..7]: ";
    for (int i = 0; i < 8 && i < encoded.size(); i++) {
        std::cout << std::hex << (int)encoded[i] << " ";
    }
    std::cout << std::dec << "\n\n";

    OFDMModulator modulator(cfg);
    OFDMDemodulator demod(cfg);

    // Generate preamble and modulated signal
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, Modulation::DQPSK);

    std::cout << "Preamble: " << preamble.size() << " samples\n";
    std::cout << "Modulated: " << modulated.size() << " samples\n";

    // Combine
    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());
    std::cout << "Total signal: " << signal.size() << " samples\n\n";

    // Normalize
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    std::cout << "Signal max before normalize: " << max_val << "\n";
    for (float& s : signal) s *= 0.5f / max_val;

    // Show signal samples around first data symbol
    size_t data_start = preamble.size();
    std::cout << "Data starts at sample " << data_start << "\n";
    std::cout << "Samples around data start:\n";
    for (int i = -5; i <= 5; i++) {
        size_t idx = data_start + i;
        if (idx < signal.size()) {
            std::cout << "  [" << idx << "]: " << signal[idx] << "\n";
        }
    }
    std::cout << "\n";

    // Demodulate
    std::cout << "=== Demodulating ===\n";
    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    std::cout << "\nSoft bits: " << soft.size() << "\n";

    // Show first few soft bits
    std::cout << "Soft bits[0..11]: ";
    for (int i = 0; i < 12 && i < soft.size(); i++) {
        std::cout << (soft[i] < 0 ? "1" : "0");
        if ((i+1) % 2 == 0) std::cout << " ";
    }
    std::cout << "\n";

    // Decode and compare
    LDPCDecoder decoder(CodeRate::R1_2);
    if (soft.size() >= 648) {
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);
        std::cout << "LDPC: " << (decoder.lastDecodeSuccess() ? "OK" : "FAIL") << "\n";
    }

    return 0;
}
