// Test MFSK sync with PTT noise (replicates GUI simulator behavior)
// Usage: ./test_mfsk_sync

#include "fsk/mfsk.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/fec.hpp"
#include <iostream>
#include <random>
#include <cmath>

using namespace ultra;
namespace v2 = protocol::v2;

// Add channel noise to signal
void addChannelNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
    // Calculate signal RMS
    float sum_sq = 0;
    for (float s : samples) sum_sq += s * s;
    float signal_rms = std::sqrt(sum_sq / samples.size());

    if (signal_rms < 1e-6f) return;

    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float signal_power = signal_rms * signal_rms;
    float noise_power = signal_power / snr_linear;
    float noise_stddev = std::sqrt(noise_power);

    std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

    for (float& sample : samples) {
        sample += noise_dist(rng);
    }
}

int main() {
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    float snr_db = 15.0f;  // Typical simulation SNR

    printf("=== MFSK Sync Test with PTT Noise ===\n\n");

    // Create MFSK config (8-tone, similar to what GUI uses)
    MFSKConfig config;
    config.num_tones = 8;
    config.samples_per_symbol = 1536;  // ~31.25 baud
    config.repetition = 2;

    printf("MFSK Config: %d-FSK, %d samples/symbol (%.1f baud), rep=%d\n",
           config.num_tones, config.samples_per_symbol, config.symbol_rate(), config.repetition);
    printf("Effective BPS: %.1f\n", config.effective_bps());

    // Create modulator and demodulator
    MFSKModulator mod(config);
    MFSKDemodulator demod(config);

    // Create a test frame (similar to CONNECT - 3 codewords)
    printf("\n--- Generating test signal ---\n");

    // Create frame data
    Bytes frame_data = {0x55, 0x4C, 0x12, 0x01};  // UL magic + header
    frame_data.resize(41, 0xAA);  // Pad to 41 bytes (3 codewords)

    // Encode with LDPC
    auto encoded_cws = v2::encodeFrameWithLDPC(frame_data);
    printf("Frame: %zu bytes -> %zu codewords\n", frame_data.size(), encoded_cws.size());

    // Concatenate encoded codewords
    Bytes to_modulate;
    for (const auto& cw : encoded_cws) {
        to_modulate.insert(to_modulate.end(), cw.begin(), cw.end());
    }
    printf("Encoded: %zu bytes (%zu bits)\n", to_modulate.size(), to_modulate.size() * 8);

    // Generate preamble and modulate
    auto preamble = mod.generatePreamble(2);  // 2 cycles of tone sweep
    auto modulated = mod.modulate(to_modulate);

    int preamble_symbols = 2 * config.num_tones;  // cycles * num_tones
    printf("Preamble: %zu samples (%d symbols, %.1f ms)\n",
           preamble.size(), preamble_symbols, preamble.size() * 1000.0f / 48000.0f);
    printf("Data: %zu samples (%.1f ms)\n",
           modulated.size(), modulated.size() * 1000.0f / 48000.0f);

    // Combine preamble + data
    std::vector<float> signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    // Add tail guard
    signal.resize(signal.size() + config.samples_per_symbol * 2, 0.0f);

    printf("Total signal: %zu samples (%.1f ms)\n",
           signal.size(), signal.size() * 1000.0f / 48000.0f);

    // Scale signal
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : signal) s *= 0.8f / max_val;
    }

    // Test 1: Clean signal (no PTT noise)
    printf("\n--- Test 1: Clean signal (no PTT noise) ---\n");
    {
        SampleSpan span(signal.data(), signal.size());
        int preamble_start = demod.findPreamble(span);

        // findPreamble returns start of preamble, data starts after preamble
        int data_start = (preamble_start >= 0) ? preamble_start + preamble.size() : -1;

        int expected = preamble.size();  // Data should start right after preamble
        printf("Expected data_start: %d\n", expected);
        printf("Preamble detected at: %d\n", preamble_start);
        printf("Computed data_start: %d\n", data_start);
        printf("Error: %d samples (%.2f symbols)\n",
               data_start - expected, (data_start - expected) / (float)config.samples_per_symbol);

        if (preamble_start >= 0 && std::abs(data_start - expected) <= config.samples_per_symbol) {
            printf("Result: PASS\n");
        } else {
            printf("Result: FAIL\n");
        }
    }

    // Test 2: With PTT noise (like GUI simulator)
    printf("\n--- Test 2: With PTT noise (%.0f dB SNR) ---\n", snr_db);
    {
        std::vector<float> noisy_signal = signal;

        // Prepend PTT noise (fixed 250ms for reproducibility)
        size_t ptt_samples = 12000;  // 250ms at 48kHz
        float noise_stddev = 0.1f / std::sqrt(std::pow(10.0f, snr_db / 10.0f));
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        std::vector<float> ptt_noise(ptt_samples);
        for (size_t i = 0; i < ptt_samples; ++i) {
            ptt_noise[i] = noise_dist(rng);
        }
        noisy_signal.insert(noisy_signal.begin(), ptt_noise.begin(), ptt_noise.end());

        printf("Added %zu samples (%.0f ms) PTT noise\n",
               ptt_samples, ptt_samples * 1000.0f / 48000.0f);
        printf("Total buffer: %zu samples\n", noisy_signal.size());

        // Add channel noise
        addChannelNoise(noisy_signal, snr_db, rng);

        SampleSpan span(noisy_signal.data(), noisy_signal.size());
        int preamble_start = demod.findPreamble(span);
        int data_start = (preamble_start >= 0) ? preamble_start + preamble.size() : -1;

        int expected = ptt_samples + preamble.size();
        printf("Expected data_start: %d (PTT %zu + preamble %zu)\n",
               expected, ptt_samples, preamble.size());
        printf("Preamble detected at: %d (expected %zu)\n", preamble_start, ptt_samples);
        printf("Computed data_start: %d\n", data_start);
        printf("Error: %d samples (%.2f symbols)\n",
               data_start - expected, (data_start - expected) / (float)config.samples_per_symbol);

        // Check available samples
        int available = noisy_signal.size() - data_start;
        int needed = modulated.size();
        printf("Available after data_start: %d samples\n", available);
        printf("Needed for data: %d samples\n", needed);
        printf("Margin: %d samples (%.1f symbols)\n",
               available - needed, (available - needed) / (float)config.samples_per_symbol);

        if (data_start > 0 && available >= needed) {
            printf("Result: PASS (enough samples)\n");

            // Try to decode
            SampleSpan data_span(noisy_signal.data() + data_start, needed);
            auto soft_bits = demod.demodulateSoft(data_span);
            printf("Demodulated: %zu soft bits\n", soft_bits.size());

            // Decode first codeword
            if (soft_bits.size() >= v2::LDPC_CODEWORD_BITS) {
                std::vector<float> cw0(soft_bits.begin(), soft_bits.begin() + v2::LDPC_CODEWORD_BITS);
                LDPCDecoder decoder(CodeRate::R1_4);
                auto decoded = decoder.decodeSoft(cw0);
                if (decoder.lastDecodeSuccess()) {
                    printf("CW0 decode: SUCCESS\n");
                } else {
                    printf("CW0 decode: FAILED\n");
                }
            }
        } else {
            printf("Result: FAIL (not enough samples or no sync)\n");
        }
    }

    // Test 3: Multiple trials with random PTT noise
    printf("\n--- Test 3: 10 trials with random PTT noise ---\n");
    int pass_count = 0;
    int decode_count = 0;

    for (int trial = 0; trial < 10; trial++) {
        std::vector<float> noisy_signal = signal;

        // Random PTT noise (100-500ms)
        std::uniform_int_distribution<size_t> delay_dist(4800, 24000);
        size_t ptt_samples = delay_dist(rng);
        float noise_stddev = 0.1f / std::sqrt(std::pow(10.0f, snr_db / 10.0f));
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        std::vector<float> ptt_noise(ptt_samples);
        for (size_t i = 0; i < ptt_samples; ++i) {
            ptt_noise[i] = noise_dist(rng);
        }
        noisy_signal.insert(noisy_signal.begin(), ptt_noise.begin(), ptt_noise.end());
        addChannelNoise(noisy_signal, snr_db, rng);

        SampleSpan span(noisy_signal.data(), noisy_signal.size());
        int preamble_start = demod.findPreamble(span);
        int data_start = (preamble_start >= 0) ? preamble_start + preamble.size() : -1;

        int expected = ptt_samples + preamble.size();
        int available = (data_start >= 0) ? noisy_signal.size() - data_start : 0;
        int needed = modulated.size();

        bool sync_ok = (data_start > 0 && available >= needed);
        bool decode_ok = false;

        if (sync_ok) {
            pass_count++;

            SampleSpan data_span(noisy_signal.data() + data_start, needed);
            auto soft_bits = demod.demodulateSoft(data_span);

            if (soft_bits.size() >= v2::LDPC_CODEWORD_BITS) {
                std::vector<float> cw0(soft_bits.begin(), soft_bits.begin() + v2::LDPC_CODEWORD_BITS);
                LDPCDecoder decoder(CodeRate::R1_4);
                auto decoded = decoder.decodeSoft(cw0);
                decode_ok = decoder.lastDecodeSuccess();
                if (decode_ok) decode_count++;
            }
        }

        printf("  Trial %d: PTT=%5zu expected=%5d detected=%5d error=%+5d sync=%s decode=%s\n",
               trial + 1, ptt_samples, expected, data_start,
               data_start - expected,
               sync_ok ? "OK" : "FAIL",
               decode_ok ? "OK" : "FAIL");
    }

    printf("\nSync success: %d/10\n", pass_count);
    printf("Decode success: %d/10\n", decode_count);

    return (decode_count >= 8) ? 0 : 1;
}
