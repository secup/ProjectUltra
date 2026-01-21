/**
 * Symbol Rate Investigation - Comparing fast vs slow symbol rates on HF
 *
 * Current system: 512 FFT, 48 CP = 85.7 sym/s
 * VARA-like:      1024 FFT, 96 CP = 42.9 sym/s
 */

#include "sim/hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"

#include <iostream>
#include <iomanip>

using namespace ultra;
using namespace ultra::sim;

struct TestConfig {
    const char* name;
    uint32_t fft_size;
    CyclicPrefixMode cp_mode;
    uint32_t num_carriers;
    bool use_pilots;
};

float testSymbolRate(const TestConfig& cfg, CodeRate rate,
                     WattersonChannel::Config channel_cfg, int num_frames = 30) {
    ModemConfig config;
    config.modulation = Modulation::DQPSK;
    config.code_rate = rate;
    config.sample_rate = 48000;
    config.fft_size = cfg.fft_size;
    config.num_carriers = cfg.num_carriers;
    config.cp_mode = cfg.cp_mode;
    config.use_pilots = cfg.use_pilots;
    config.sync_threshold = 0.65f;

    // Calculate actual parameters
    uint32_t cp_samples = config.getCyclicPrefix();
    uint32_t symbol_samples = cfg.fft_size + cp_samples;
    float symbol_rate = 48000.0f / symbol_samples;
    float carrier_spacing = 48000.0f / cfg.fft_size;

    static bool printed = false;
    if (!printed) {
        std::cout << "\nConfig: " << cfg.name << "\n";
        std::cout << "  FFT: " << cfg.fft_size << ", CP: " << cp_samples << " samples\n";
        std::cout << "  Symbol duration: " << (symbol_samples * 1000.0f / 48000.0f) << " ms\n";
        std::cout << "  Symbol rate: " << symbol_rate << " sym/s\n";
        std::cout << "  Carrier spacing: " << carrier_spacing << " Hz\n";
        std::cout << "  Data carriers: " << cfg.num_carriers << "\n";
        std::cout << "  Bits/symbol: " << (cfg.num_carriers * 2) << " (DQPSK)\n";
        printed = true;
    }

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
    std::cout << "        Symbol Rate Investigation: Fast vs Slow on HF\n";
    std::cout << "=============================================================================\n";

    // Test configurations
    TestConfig configs[] = {
        // Current system
        {"Current (512 FFT, 85.7 sym/s)", 512, CyclicPrefixMode::MEDIUM, 30, false},

        // VARA-like slower symbol rate
        {"VARA-like (1024 FFT, 42.9 sym/s)", 1024, CyclicPrefixMode::MEDIUM, 30, false},

        // VARA-like with more carriers
        {"VARA-like + 60 carriers", 1024, CyclicPrefixMode::MEDIUM, 60, false},
    };

    std::vector<float> snr_levels = {15.0f, 20.0f, 25.0f};

    std::cout << "\n=============================================================================\n";
    std::cout << "DQPSK R1/4 Frame Success Rate (%) on ITU-R HF Channels\n";
    std::cout << "=============================================================================\n";

    for (auto& cfg : configs) {
        std::cout << "\n" << cfg.name << ":\n";
        std::cout << std::setw(12) << "Channel";
        for (float snr : snr_levels) {
            std::cout << std::setw(10) << (int)snr << "dB";
        }
        std::cout << "\n";
        std::cout << "-----------------------------------------------\n";

        // Reset printed flag for each config
        static bool first = true;
        if (!first) {
            // Force reprint of config details
        }
        first = false;

        struct {
            const char* name;
            WattersonChannel::Config (*getConfig)(float);
        } channels[] = {
            {"AWGN", itu_r_f1487::awgn},
            {"Good", itu_r_f1487::good},
            {"Moderate", itu_r_f1487::moderate},
            {"Poor", itu_r_f1487::poor},
        };

        for (auto& ch : channels) {
            std::cout << std::setw(12) << ch.name;
            for (float snr : snr_levels) {
                auto channel_cfg = ch.getConfig(snr);
                float success = testSymbolRate(cfg, CodeRate::R1_4, channel_cfg, 30);

                if (success >= 80) {
                    std::cout << std::setw(9) << std::fixed << std::setprecision(0) << success << "%";
                } else if (success >= 50) {
                    std::cout << std::setw(9) << success << "*";
                } else {
                    std::cout << std::setw(9) << success << "!";
                }
            }
            std::cout << "\n";
        }
    }

    // Throughput comparison
    std::cout << "\n=============================================================================\n";
    std::cout << "Throughput Comparison (theoretical max with DQPSK R1/2)\n";
    std::cout << "=============================================================================\n";

    std::cout << "\nCurrent (512 FFT, 30 carriers, 85.7 sym/s):\n";
    std::cout << "  Bits/symbol: 30 × 2 = 60 bits\n";
    std::cout << "  Raw rate: 60 × 85.7 = 5142 bps\n";
    std::cout << "  With R1/2: 5142 × 0.5 = 2571 bps = 2.57 kbps\n";

    std::cout << "\nVARA-like (1024 FFT, 30 carriers, 42.9 sym/s):\n";
    std::cout << "  Bits/symbol: 30 × 2 = 60 bits\n";
    std::cout << "  Raw rate: 60 × 42.9 = 2574 bps\n";
    std::cout << "  With R1/2: 2574 × 0.5 = 1287 bps = 1.29 kbps\n";

    std::cout << "\nVARA-like (1024 FFT, 60 carriers, 42.9 sym/s):\n";
    std::cout << "  Bits/symbol: 60 × 2 = 120 bits\n";
    std::cout << "  Raw rate: 120 × 42.9 = 5148 bps\n";
    std::cout << "  With R1/2: 5148 × 0.5 = 2574 bps = 2.57 kbps\n";

    std::cout << "\n=============================================================================\n";
    std::cout << "Key insight: Slower symbols with more carriers can maintain throughput\n";
    std::cout << "while potentially improving HF robustness.\n";
    std::cout << "=============================================================================\n";

    return 0;
}
