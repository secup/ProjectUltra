// Simple test: one DQPSK frame with leading noise
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
    config.use_pilots = false;  // DQPSK
    
    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);
    
    // Create test payload
    Bytes data = {'H', 'E', 'L', 'L', 'O', '!', 0, 0, 0, 0};  // 10 bytes
    Bytes encoded = encoder.encode(data);
    
    // Generate frame
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, Modulation::DQPSK);
    
    Samples frame;
    frame.insert(frame.end(), preamble.begin(), preamble.end());
    frame.insert(frame.end(), modulated.begin(), modulated.end());

    // Test WITHOUT normalization to verify amplitude-independent sync
    float max_val = 0;
    for (float s : frame) max_val = std::max(max_val, std::abs(s));
    // Normalization DISABLED - testing amplitude independence
    // for (float& s : frame) s *= 0.5f / max_val;

    printf("Frame size: %zu samples (%.1f ms)\n", frame.size(), frame.size() / 48.0f);
    printf("Preamble: %zu, Data: %zu\n", preamble.size(), modulated.size());
    printf("Max val before norm: %.6f, after: 0.5\n", max_val);
    
    // Create buffer with leading noise then frame
    size_t leading_silence = 48000;  // 1 second before frame
    Samples full_audio(leading_silence + frame.size());

    // Place frame after leading region
    for (size_t i = 0; i < frame.size(); ++i) {
        full_audio[leading_silence + i] = frame[i];
    }

    printf("Leading silence: %zu samples\n", leading_silence);
    
    // Calculate signal power
    float signal_power = 0;
    for (float s : frame) signal_power += s * s;
    signal_power /= frame.size();
    
    // Add noise at 25 dB SNR
    float snr_db = 25.0f;
    float noise_std = std::sqrt(signal_power / std::pow(10.0f, snr_db / 10.0f));
    std::normal_distribution<float> noise(0.0f, noise_std);
    
    printf("Signal power: %.6f, Noise std: %.6f (SNR=%.1f dB)\n", 
           signal_power, noise_std, snr_db);
    
    // Add noise (testing amplitude-independent sync)
    for (float& s : full_audio) {
        s += noise(rng);
    }
    
    // Process through demodulator
    printf("\nProcessing %zu samples...\n", full_audio.size());
    size_t chunk_size = 960;
    for (size_t i = 0; i < full_audio.size(); i += chunk_size) {
        size_t len = std::min(chunk_size, full_audio.size() - i);
        SampleSpan span(full_audio.data() + i, len);
        demod.process(span);
    }
    
    // Try to decode
    auto soft = demod.getSoftBits();
    printf("\nGot %zu soft bits\n", soft.size());
    
    if (soft.size() >= 648) {
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);
        
        if (decoder.lastDecodeSuccess()) {
            printf("DECODE SUCCESS! Got: ");
            for (size_t i = 0; i < std::min((size_t)10, decoded.size()); ++i) {
                printf("%c", decoded[i] >= 32 && decoded[i] < 127 ? decoded[i] : '?');
            }
            printf("\n");
        } else {
            printf("LDPC decode FAILED (iters=%d)\n", decoder.lastIterations());
        }
    } else {
        printf("NOT ENOUGH SOFT BITS for decode\n");
    }
    
    return 0;
}
