/**
 * DQPSK Comparison Test - With vs Without Pilots on HF
 *
 * Compares performance to see if removing pilots actually hurts HF performance
 */

#include "sim/hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

#include <iostream>
#include <iomanip>

using namespace ultra;
using namespace ultra::sim;

float testConfig(bool use_pilots, int pilot_spacing, CodeRate rate,
                 WattersonChannel::Config channel_cfg, int num_frames = 30) {
    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = rate;
    config.sample_rate = 48000;
    config.num_carriers = 30;
    config.fft_size = 512;
    config.cp_mode = CyclicPrefixMode::MEDIUM;
    config.use_pilots = use_pilots;
    config.pilot_spacing = pilot_spacing;
    config.sync_threshold = 0.70f;

    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(6, 108);
    WattersonChannel channel(channel_cfg);

    size_t k_bits = (rate == CodeRate::R1_4) ? 162 : 324;
    size_t k_bytes = k_bits / 8;

    int success = 0;
    for (int frame = 0; frame < num_frames; frame++) {
        OFDMDemodulator demodulator(config);

        Bytes tx_data(k_bytes);
        for (size_t i = 0; i < k_bytes; i++)
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13) & 0xFF);

        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, Modulation::DQPSK);

        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx_audio) s *= 0.5f / max_val;

        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        bool frame_ready = false;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += 1024) {
            size_t remaining = std::min((size_t)1024, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            auto soft_bits = demodulator.getSoftBits();
            if (soft_bits.size() >= 648) {
                soft_bits.resize(648);
                auto deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);
                if (decoder.lastDecodeSuccess()) {
                    rx_data.resize(k_bytes);
                    if (rx_data == tx_data) success++;
                }
            }
        }
    }
    return 100.0f * success / num_frames;
}

int main() {
    std::cout << "=============================================================================\n";
    std::cout << "        DQPSK Comparison: With Pilots vs Without Pilots\n";
    std::cout << "=============================================================================\n\n";

    auto good_20 = itu_r_f1487::good(20.0f);
    auto good_25 = itu_r_f1487::good(25.0f);
    auto mod_20 = itu_r_f1487::moderate(20.0f);
    auto mod_25 = itu_r_f1487::moderate(25.0f);

    std::cout << "Testing DQPSK R1/4 on Good channel (0.5ms delay, 0.1Hz Doppler):\n";
    std::cout << "-----------------------------------------------------------------------------\n";
    std::cout << std::setw(30) << "Configuration" << std::setw(15) << "20dB" << std::setw(15) << "25dB\n";
    std::cout << "-----------------------------------------------------------------------------\n";

    // No pilots (30 data carriers)
    float np_20 = testConfig(false, 2, CodeRate::R1_4, good_20);
    float np_25 = testConfig(false, 2, CodeRate::R1_4, good_25);
    std::cout << std::setw(30) << "No pilots (30 data carriers)"
              << std::setw(14) << std::fixed << std::setprecision(0) << np_20 << "%"
              << std::setw(14) << np_25 << "%\n";

    // Sparse pilots (22 data carriers)
    float sp_20 = testConfig(true, 4, CodeRate::R1_4, good_20);
    float sp_25 = testConfig(true, 4, CodeRate::R1_4, good_25);
    std::cout << std::setw(30) << "Pilots 1/4 (22 data carriers)"
              << std::setw(14) << sp_20 << "%"
              << std::setw(14) << sp_25 << "%\n";

    // Dense pilots (15 data carriers) - OLD config
    float dp_20 = testConfig(true, 2, CodeRate::R1_4, good_20);
    float dp_25 = testConfig(true, 2, CodeRate::R1_4, good_25);
    std::cout << std::setw(30) << "Pilots 1/2 (15 data carriers)"
              << std::setw(14) << dp_20 << "%"
              << std::setw(14) << dp_25 << "%\n";

    std::cout << "\nTesting DQPSK R1/4 on Moderate channel (1.0ms delay, 0.5Hz Doppler):\n";
    std::cout << "-----------------------------------------------------------------------------\n";

    np_20 = testConfig(false, 2, CodeRate::R1_4, mod_20);
    np_25 = testConfig(false, 2, CodeRate::R1_4, mod_25);
    std::cout << std::setw(30) << "No pilots (30 data carriers)"
              << std::setw(14) << np_20 << "%"
              << std::setw(14) << np_25 << "%\n";

    sp_20 = testConfig(true, 4, CodeRate::R1_4, mod_20);
    sp_25 = testConfig(true, 4, CodeRate::R1_4, mod_25);
    std::cout << std::setw(30) << "Pilots 1/4 (22 data carriers)"
              << std::setw(14) << sp_20 << "%"
              << std::setw(14) << sp_25 << "%\n";

    dp_20 = testConfig(true, 2, CodeRate::R1_4, mod_20);
    dp_25 = testConfig(true, 2, CodeRate::R1_4, mod_25);
    std::cout << std::setw(30) << "Pilots 1/2 (15 data carriers)"
              << std::setw(14) << dp_20 << "%"
              << std::setw(14) << dp_25 << "%\n";

    std::cout << "\n=============================================================================\n";
    std::cout << "Conclusion: Compare success rates to see if pilots help on HF fading.\n";
    std::cout << "=============================================================================\n";

    return 0;
}
