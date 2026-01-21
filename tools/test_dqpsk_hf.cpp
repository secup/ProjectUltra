/**
 * DQPSK HF Channel Test - Verifies DQPSK without pilots works on HF
 *
 * Tests the optimized DQPSK mode (all 30 carriers, no pilots) through
 * ITU-R F.1487 Watterson HF channel model.
 *
 * Compile: In build/, run: make test_dqpsk_hf
 * Run: ./tests/test_dqpsk_hf
 */

#include "sim/hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>

using namespace ultra;
using namespace ultra::sim;

// Test DQPSK at a specific SNR through HF channel
struct TestResult {
    int frames_sent;
    int frames_ok;
    float success_rate;
};

TestResult testDQPSK(CodeRate rate, WattersonChannel::Config channel_cfg, int num_frames = 50) {
    // Setup modem config - DQPSK without pilots!
    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = rate;
    config.sample_rate = 48000;
    config.num_carriers = 30;
    config.fft_size = 512;
    config.cp_mode = CyclicPrefixMode::MEDIUM;
    config.use_pilots = false;  // KEY: All 30 carriers for data!
    config.sync_threshold = 0.70f;  // Lower threshold for fading

    // Create modem components
    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(6, 108);

    // Create channel
    WattersonChannel channel(channel_cfg);

    // Get data size for this rate
    size_t k_bits;
    switch (rate) {
        case CodeRate::R1_4: k_bits = 162; break;
        case CodeRate::R1_2: k_bits = 324; break;
        case CodeRate::R2_3: k_bits = 432; break;
        case CodeRate::R3_4: k_bits = 486; break;
        default: k_bits = 324;
    }
    size_t k_bytes = k_bits / 8;

    int success_count = 0;

    for (int frame = 0; frame < num_frames; frame++) {
        // Fresh demodulator per frame
        OFDMDemodulator demodulator(config);

        // Generate test data
        Bytes tx_data(k_bytes);
        for (size_t i = 0; i < k_bytes; i++) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13) & 0xFF);
        }

        // TX: encode -> interleave -> modulate
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, Modulation::DQPSK);

        // Combine preamble + data
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Normalize
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // Pass through HF channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // RX: demodulate -> deinterleave -> decode
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();
            if (soft_bits.size() >= 648) {
                soft_bits.resize(648);  // One codeword
                auto deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    rx_data.resize(k_bytes);
                    if (rx_data == tx_data) {
                        success_count++;
                    }
                }
            }
        }
    }

    TestResult result;
    result.frames_sent = num_frames;
    result.frames_ok = success_count;
    result.success_rate = 100.0f * success_count / num_frames;
    return result;
}

int main() {
    std::cout << "=============================================================================\n";
    std::cout << "        DQPSK HF Channel Test (No Pilots - All 30 Carriers)\n";
    std::cout << "=============================================================================\n\n";

    std::cout << "Configuration:\n";
    std::cout << "  Modulation:    DQPSK (differential - no pilots needed)\n";
    std::cout << "  Data carriers: 30 (ALL carriers used for data)\n";
    std::cout << "  Interleaver:   6x108\n";
    std::cout << "  Frames/test:   50\n\n";

    std::cout << "ITU-R F.1487 Channel Conditions:\n";
    std::cout << "  AWGN:     No fading (baseline)\n";
    std::cout << "  Good:     0.5ms delay, 0.1Hz Doppler\n";
    std::cout << "  Moderate: 1.0ms delay, 0.5Hz Doppler\n";
    std::cout << "  Poor:     2.0ms delay, 1.0Hz Doppler\n\n";

    // Test conditions
    struct ChannelTest {
        const char* name;
        WattersonChannel::Config (*getConfig)(float snr);
    };

    std::vector<ChannelTest> channels = {
        {"AWGN", itu_r_f1487::awgn},
        {"Good", itu_r_f1487::good},
        {"Moderate", itu_r_f1487::moderate},
        {"Poor", itu_r_f1487::poor},
    };

    std::vector<std::pair<CodeRate, const char*>> rates = {
        {CodeRate::R1_4, "R1/4"},
        {CodeRate::R1_2, "R1/2"},
        {CodeRate::R2_3, "R2/3"},
    };

    std::vector<float> snr_levels = {10.0f, 15.0f, 20.0f, 25.0f};

    // Print header
    std::cout << "=============================================================================\n";
    std::cout << "Frame Success Rate (%) - Need >80% for reliable ARQ\n";
    std::cout << "=============================================================================\n";

    for (auto& rate_info : rates) {
        std::cout << "\n" << rate_info.second << ":\n";
        std::cout << std::setw(12) << "Channel";
        for (float snr : snr_levels) {
            std::cout << std::setw(8) << (int)snr << "dB";
        }
        std::cout << "\n";
        std::cout << "-----------------------------------------------------------------------------\n";

        for (auto& ch : channels) {
            std::cout << std::setw(12) << ch.name;

            for (float snr : snr_levels) {
                auto cfg = ch.getConfig(snr);
                auto result = testDQPSK(rate_info.first, cfg, 50);

                // Color code: green >80%, yellow 50-80%, red <50%
                if (result.success_rate >= 80) {
                    std::cout << std::setw(7) << std::fixed << std::setprecision(0)
                              << result.success_rate << "%";
                } else if (result.success_rate >= 50) {
                    std::cout << std::setw(7) << std::fixed << std::setprecision(0)
                              << result.success_rate << "*";
                } else {
                    std::cout << std::setw(7) << std::fixed << std::setprecision(0)
                              << result.success_rate << "!";
                }
            }
            std::cout << "\n";
        }
    }

    std::cout << "\n=============================================================================\n";
    std::cout << "Legend: XX% = good (>80%), XX* = marginal (50-80%), XX! = poor (<50%)\n";
    std::cout << "With ARQ, >80% success rate means reliable delivery.\n";
    std::cout << "=============================================================================\n";

    return 0;
}
