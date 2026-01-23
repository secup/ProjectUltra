// Single frame test matching test_hf_reality
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>

using namespace ultra;

int main() {
    setLogLevel(LogLevel::DEBUG);
    std::mt19937 rng(42);
    
    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::DQPSK;
    config.code_rate = CodeRate::R1_2;
    config.use_pilots = false;
    
    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);
    
    // Same payload as test_hf_reality
    std::string msg = "MSG0_HF_TEST";
    Bytes payload(msg.begin(), msg.end());
    
    printf("Payload: %zu bytes\n", payload.size());
    
    Bytes encoded = encoder.encode(payload);
    printf("Encoded: %zu bytes = %zu bits\n", encoded.size(), encoded.size() * 8);
    
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, Modulation::DQPSK);
    
    printf("Preamble: %zu samples\n", preamble.size());
    printf("Modulated: %zu samples\n", modulated.size());
    
    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());
    
    // Normalize (like test_hf_reality)
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;
    
    // Add leading silence + noise (like test_hf_reality)
    size_t leading = 48000;  // 1 second
    Samples full_audio(leading + signal.size() + 48000);  // Add trailing silence too
    
    for (size_t i = 0; i < signal.size(); ++i) {
        full_audio[leading + i] = signal[i];
    }
    
    // Add noise at 25 dB SNR
    float sp = 0;
    for (float s : signal) sp += s * s;
    sp /= signal.size();
    float noise_std = std::sqrt(sp / std::pow(10.0f, 25.0f / 10.0f));
    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : full_audio) s += noise(rng);
    
    printf("\nTotal audio: %zu samples (%.1f sec)\n", full_audio.size(), full_audio.size() / 48000.0f);
    printf("Signal at: %zu - %zu\n", leading, leading + signal.size());
    
    // Process
    printf("\nProcessing...\n");
    for (size_t i = 0; i < full_audio.size(); i += 960) {
        size_t len = std::min((size_t)960, full_audio.size() - i);
        SampleSpan span(full_audio.data() + i, len);
        demod.process(span);
    }
    
    auto soft = demod.getSoftBits();
    printf("\nGot %zu soft bits\n", soft.size());
    
    if (soft.size() >= 648) {
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);
        if (decoder.lastDecodeSuccess()) {
            printf("SUCCESS: ");
            for (size_t i = 0; i < std::min(payload.size(), decoded.size()); ++i)
                printf("%c", decoded[i] >= 32 && decoded[i] < 127 ? decoded[i] : '?');
            printf("\n");
        } else {
            printf("LDPC FAILED\n");
        }
    } else {
        printf("NOT ENOUGH BITS\n");
    }
    
    return 0;
}
