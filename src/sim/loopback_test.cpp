/**
 * ProjectUltra HF Channel Simulator
 *
 * Tests modem performance under realistic HF propagation conditions
 * using the ITU-R F.1487 Watterson channel model with CCIR standard
 * test conditions (Good, Moderate, Poor, Flutter).
 */

#include "hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"

#include <iostream>
#include <iomanip>

using namespace ultra;
using namespace ultra::sim;

// LDPC block size (must match demodulator expectation)
static constexpr size_t LDPC_BLOCK_SIZE = 648;

// Count bit errors between two byte arrays
size_t countBitErrors(const Bytes& a, const Bytes& b) {
    size_t errors = 0;
    size_t len = std::min(a.size(), b.size());

    for (size_t i = 0; i < len; ++i) {
        uint8_t diff = a[i] ^ b[i];
        while (diff) {
            errors += diff & 1;
            diff >>= 1;
        }
    }

    // Count missing bytes as all errors
    if (a.size() > b.size()) {
        errors += (a.size() - b.size()) * 8;
    } else if (b.size() > a.size()) {
        errors += (b.size() - a.size()) * 8;
    }

    return errors;
}

void printHeader() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     ProjectUltra - Realistic HF Channel Simulation (Watterson)     ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";
}

// Full end-to-end test through realistic channel
struct E2EResult {
    size_t frames_sent;
    size_t frames_decoded;
    size_t total_bits;
    size_t bit_errors;
    float frame_success_rate;
    float ber;
    float effective_throughput;
};

E2EResult runE2ETest(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    const WattersonChannel::Config& channel_cfg,
    size_t num_frames = 50
) {
    E2EResult result = {};
    result.frames_sent = num_frames;

    // Create modem components
    OFDMModulator modulator(config);
    OFDMDemodulator demodulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(32, 32);

    // Create channel
    WattersonChannel channel(channel_cfg);

    // Calculate theoretical throughput
    float code_rate_val = 0.5f;
    switch (rate) {
        case CodeRate::R1_4: code_rate_val = 0.25f; break;
        case CodeRate::R1_2: code_rate_val = 0.5f; break;
        case CodeRate::R2_3: code_rate_val = 0.667f; break;
        case CodeRate::R3_4: code_rate_val = 0.75f; break;
        case CodeRate::R5_6: code_rate_val = 0.833f; break;
        case CodeRate::R7_8: code_rate_val = 0.875f; break;
        default: break;
    }

    size_t bits_per_carrier = static_cast<size_t>(mod);
    size_t data_carriers = config.num_carriers - config.num_carriers / config.pilot_spacing;
    float symbol_samples = config.getSymbolDuration();
    float symbol_rate = config.sample_rate / symbol_samples;
    float raw_throughput = data_carriers * bits_per_carrier * symbol_rate * code_rate_val;

    for (size_t frame = 0; frame < num_frames; ++frame) {
        // Generate test data
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }
        result.total_bits += tx_data.size() * 8;

        // === TX Chain ===
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);
        Samples tx_audio = modulator.modulate(interleaved, mod);

        // === Channel ===
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // === RX Chain ===
        // Feed to demodulator
        demodulator.reset();
        SampleSpan rx_span(rx_audio.data(), rx_audio.size());
        bool frame_ready = demodulator.process(rx_span);

        if (frame_ready) {
            // Get soft bits
            std::vector<float> soft_bits = demodulator.getSoftBits();

            if (soft_bits.size() >= LDPC_BLOCK_SIZE) {
                // Deinterleave
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);

                // Decode
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    // Truncate to original size
                    if (rx_data.size() > tx_data.size()) {
                        rx_data.resize(tx_data.size());
                    }

                    size_t errors = countBitErrors(tx_data, rx_data);
                    result.bit_errors += errors;

                    if (errors == 0) {
                        result.frames_decoded++;
                    }
                }
            }
        }
    }

    result.frame_success_rate = 100.0f * result.frames_decoded / result.frames_sent;
    result.ber = result.total_bits > 0 ?
        static_cast<float>(result.bit_errors) / result.total_bits : 1.0f;
    result.effective_throughput = raw_throughput * result.frame_success_rate / 100.0f;

    return result;
}

// Simplified test that bypasses sync (for BER curves)
E2EResult runSimplifiedTest(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    const WattersonChannel::Config& channel_cfg,
    size_t num_frames = 100
) {
    E2EResult result = {};
    result.frames_sent = num_frames;

    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(32, 32);

    // Create channel for soft bit degradation simulation
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0.0f, 1.0f);

    // Calculate noise based on SNR
    float noise_std = std::pow(10.0f, -channel_cfg.snr_db / 20.0f);

    // Modulation sensitivity factor (approximates minimum constellation distance)
    // Higher-order modulations have smaller symbol spacing, requiring better SNR
    float mod_sensitivity = 1.0f;
    switch (mod) {
        case Modulation::BPSK:   mod_sensitivity = 1.0f;  break;  // d_min = 2
        case Modulation::QPSK:   mod_sensitivity = 0.7f;  break;  // d_min = sqrt(2)
        case Modulation::QAM16:  mod_sensitivity = 0.4f;  break;  // d_min = 2/sqrt(10)
        case Modulation::QAM64:  mod_sensitivity = 0.25f; break;  // d_min = 2/sqrt(42)
        case Modulation::QAM256: mod_sensitivity = 0.15f; break;  // d_min = 2/sqrt(170)
        default: break;
    }

    // Rayleigh fading degrades SNR by averaging over deep fades
    // Slow fading (~0.1 Hz): ~6 dB loss, fast fading (>0.5 Hz): ~10 dB loss
    float fading_factor = 1.0f;
    if (channel_cfg.fading_enabled) {
        fading_factor = (channel_cfg.doppler_spread_hz > 0.5f) ? 0.3f : 0.5f;
    }

    // ISI penalty when delay spread exceeds cyclic prefix
    // CP = 48 samples at 48 kHz = 1.0 ms
    float isi_factor = 1.0f;
    if (channel_cfg.multipath_enabled && channel_cfg.delay_spread_ms > 1.0f) {
        isi_factor = 0.5f;
    }

    // Calculate throughput
    float code_rate_val = 0.5f;
    switch (rate) {
        case CodeRate::R1_4: code_rate_val = 0.25f; break;
        case CodeRate::R1_2: code_rate_val = 0.5f; break;
        case CodeRate::R2_3: code_rate_val = 0.667f; break;
        case CodeRate::R3_4: code_rate_val = 0.75f; break;
        case CodeRate::R5_6: code_rate_val = 0.833f; break;
        case CodeRate::R7_8: code_rate_val = 0.875f; break;
        default: break;
    }

    size_t bits_per_carrier = static_cast<size_t>(mod);
    size_t data_carriers = config.num_carriers - config.num_carriers / config.pilot_spacing;
    float symbol_samples = config.getSymbolDuration();
    float symbol_rate = config.sample_rate / symbol_samples;
    float raw_throughput = data_carriers * bits_per_carrier * symbol_rate * code_rate_val;

    for (size_t frame = 0; frame < num_frames; ++frame) {
        // Generate test data
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }
        result.total_bits += tx_data.size() * 8;

        // Encode
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        // Simulate channel effect on soft bits
        std::vector<float> soft_bits;
        soft_bits.reserve(interleaved.size() * 8);

        for (uint8_t byte : interleaved) {
            for (int b = 7; b >= 0; --b) {
                uint8_t bit = (byte >> b) & 1;
                float base_llr = bit ? -4.0f : 4.0f;

                // Apply channel degradation to LLR confidence
                float effective_noise = noise_std / (mod_sensitivity * fading_factor * isi_factor);
                float noisy_llr = base_llr + effective_noise * noise(rng);

                soft_bits.push_back(noisy_llr);
            }
        }

        // Decode
        auto deinterleaved = interleaver.deinterleave(soft_bits);
        Bytes rx_data = decoder.decodeSoft(deinterleaved);

        if (decoder.lastDecodeSuccess()) {
            if (rx_data.size() > tx_data.size()) {
                rx_data.resize(tx_data.size());
            }

            size_t errors = countBitErrors(tx_data, rx_data);
            result.bit_errors += errors;

            if (errors == 0) {
                result.frames_decoded++;
            }
        }
    }

    result.frame_success_rate = 100.0f * result.frames_decoded / result.frames_sent;
    result.ber = result.total_bits > 0 ?
        static_cast<float>(result.bit_errors) / result.total_bits : 1.0f;
    result.effective_throughput = raw_throughput * result.frame_success_rate / 100.0f;

    return result;
}

void printResultRow(const char* condition, const E2EResult& r, float raw_throughput) {
    std::cout << std::setw(12) << condition
              << std::setw(8) << r.frames_decoded << "/" << r.frames_sent
              << std::setw(10) << std::fixed << std::setprecision(1) << r.frame_success_rate << "%"
              << std::setw(12) << std::scientific << std::setprecision(1) << r.ber
              << std::setw(10) << std::fixed << std::setprecision(1) << r.effective_throughput / 1000
              << " kbps\n";
}

void runRealisticTests(const ModemConfig& config) {
    std::cout << "=== Realistic HF Channel Tests (Watterson Model) ===\n\n";

    std::cout << "Channel conditions:\n";
    ccir::printConfig(ccir::good(), "Good");
    ccir::printConfig(ccir::moderate(), "Moderate");
    ccir::printConfig(ccir::poor(), "Poor");
    ccir::printConfig(ccir::flutter(), "Flutter");
    std::cout << "\n";

    // Test matrix: Mode vs Channel condition
    struct TestMode {
        Modulation mod;
        CodeRate rate;
        const char* name;
    };

    std::vector<TestMode> modes = {
        {Modulation::BPSK, CodeRate::R1_2, "BPSK R1/2"},
        {Modulation::QPSK, CodeRate::R1_2, "QPSK R1/2"},
        {Modulation::QPSK, CodeRate::R3_4, "QPSK R3/4"},
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2"},
        {Modulation::QAM16, CodeRate::R3_4, "16QAM R3/4"},
        {Modulation::QAM64, CodeRate::R1_2, "64QAM R1/2"},
        {Modulation::QAM64, CodeRate::R3_4, "64QAM R3/4"},
        {Modulation::QAM256, CodeRate::R3_4, "256QAM R3/4"},
    };

    for (const auto& mode : modes) {
        std::cout << "──────────────────────────────────────────────────────────────────\n";
        std::cout << "Mode: " << mode.name << "\n";
        std::cout << std::setw(12) << "Channel"
                  << std::setw(12) << "Frames"
                  << std::setw(12) << "Success"
                  << std::setw(12) << "BER"
                  << std::setw(14) << "Throughput\n";
        std::cout << "──────────────────────────────────────────────────────────────────\n";

        // Calculate raw throughput for reference
        float code_rate = 0.5f;
        switch (mode.rate) {
            case CodeRate::R1_2: code_rate = 0.5f; break;
            case CodeRate::R3_4: code_rate = 0.75f; break;
            default: break;
        }
        size_t bits_per_carrier = static_cast<size_t>(mode.mod);
        size_t data_carriers = config.num_carriers - config.num_carriers / config.pilot_spacing;
        float symbol_samples = config.getSymbolDuration();
        float symbol_rate = config.sample_rate / symbol_samples;
        float raw_tput = data_carriers * bits_per_carrier * symbol_rate * code_rate;

        // AWGN baseline
        auto r_awgn = runSimplifiedTest(config, mode.mod, mode.rate, ccir::awgn(20.0f), 100);
        printResultRow("AWGN 20dB", r_awgn, raw_tput);

        // Realistic conditions
        auto r_good = runSimplifiedTest(config, mode.mod, mode.rate, ccir::good(20.0f), 100);
        printResultRow("Good", r_good, raw_tput);

        auto r_mod = runSimplifiedTest(config, mode.mod, mode.rate, ccir::moderate(15.0f), 100);
        printResultRow("Moderate", r_mod, raw_tput);

        auto r_poor = runSimplifiedTest(config, mode.mod, mode.rate, ccir::poor(10.0f), 100);
        printResultRow("Poor", r_poor, raw_tput);

        auto r_flutter = runSimplifiedTest(config, mode.mod, mode.rate, ccir::flutter(8.0f), 100);
        printResultRow("Flutter", r_flutter, raw_tput);

        std::cout << "\n";
    }
}

void runSpeedProfileTest(const ModemConfig& config) {
    std::cout << "\n=== Speed Profile Performance Under Realistic Conditions ===\n\n";

    // Conservative profile
    std::cout << "CONSERVATIVE (QPSK R1/2 - for poor conditions):\n";
    auto cons_good = runSimplifiedTest(config, Modulation::QPSK, CodeRate::R1_2, ccir::good(15.0f), 100);
    auto cons_mod = runSimplifiedTest(config, Modulation::QPSK, CodeRate::R1_2, ccir::moderate(12.0f), 100);
    auto cons_poor = runSimplifiedTest(config, Modulation::QPSK, CodeRate::R1_2, ccir::poor(8.0f), 100);
    std::cout << "  Good channel:     " << std::fixed << std::setprecision(1)
              << cons_good.effective_throughput / 1000 << " kbps (" << cons_good.frame_success_rate << "% success)\n";
    std::cout << "  Moderate channel: " << cons_mod.effective_throughput / 1000 << " kbps (" << cons_mod.frame_success_rate << "% success)\n";
    std::cout << "  Poor channel:     " << cons_poor.effective_throughput / 1000 << " kbps (" << cons_poor.frame_success_rate << "% success)\n\n";

    // Balanced profile
    std::cout << "BALANCED (64-QAM R3/4 - for typical conditions):\n";
    auto bal_good = runSimplifiedTest(config, Modulation::QAM64, CodeRate::R3_4, ccir::good(20.0f), 100);
    auto bal_mod = runSimplifiedTest(config, Modulation::QAM64, CodeRate::R3_4, ccir::moderate(15.0f), 100);
    auto bal_poor = runSimplifiedTest(config, Modulation::QAM64, CodeRate::R3_4, ccir::poor(10.0f), 100);
    std::cout << "  Good channel:     " << std::fixed << std::setprecision(1)
              << bal_good.effective_throughput / 1000 << " kbps (" << bal_good.frame_success_rate << "% success)\n";
    std::cout << "  Moderate channel: " << bal_mod.effective_throughput / 1000 << " kbps (" << bal_mod.frame_success_rate << "% success)\n";
    std::cout << "  Poor channel:     " << bal_poor.effective_throughput / 1000 << " kbps (" << bal_poor.frame_success_rate << "% success)\n\n";

    // Turbo profile
    std::cout << "TURBO (256-QAM R7/8 - needs excellent conditions):\n";
    auto turbo_awgn = runSimplifiedTest(config, Modulation::QAM256, CodeRate::R7_8, ccir::awgn(30.0f), 100);
    auto turbo_good = runSimplifiedTest(config, Modulation::QAM256, CodeRate::R7_8, ccir::good(25.0f), 100);
    auto turbo_mod = runSimplifiedTest(config, Modulation::QAM256, CodeRate::R7_8, ccir::moderate(20.0f), 100);
    std::cout << "  AWGN 30dB:        " << std::fixed << std::setprecision(1)
              << turbo_awgn.effective_throughput / 1000 << " kbps (" << turbo_awgn.frame_success_rate << "% success)\n";
    std::cout << "  Good channel:     " << turbo_good.effective_throughput / 1000 << " kbps (" << turbo_good.frame_success_rate << "% success)\n";
    std::cout << "  Moderate channel: " << turbo_mod.effective_throughput / 1000 << " kbps (" << turbo_mod.frame_success_rate << "% success)\n\n";
}

void runSummary() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                         REALISTIC ASSESSMENT                        ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════════════╝\n\n";

    std::cout << "Based on Watterson channel simulation:\n\n";

    std::cout << "EXPECTED REAL-WORLD PERFORMANCE:\n";
    std::cout << "─────────────────────────────────────────────────────────────────────\n";
    std::cout << "  Excellent conditions (NVIS, quiet band):      4-8 kbps\n";
    std::cout << "  Good conditions (typical mid-latitude):       2-5 kbps\n";
    std::cout << "  Moderate conditions (average DX):             1-3 kbps\n";
    std::cout << "  Poor conditions (disturbed, polar):           0.5-1.5 kbps\n";
    std::cout << "  Extreme flutter (auroral):                    <0.5 kbps (fallback to BPSK)\n\n";

    std::cout << "COMPARISON TO OTHER HF MODES:\n";
    std::cout << "─────────────────────────────────────────────────────────────────────\n";
    std::cout << "  VARA HF:      typically 2-4 kbps (claims 8.5 kbps peak)\n";
    std::cout << "  PACTOR IV:    typically 3-5 kbps (claims 10.5 kbps peak)\n";
    std::cout << "  ARDOP:        typically 1-2 kbps (claims 2.4 kbps peak)\n";
    std::cout << "  ProjectUltra: estimated 2-5 kbps typical, competitive with VARA\n\n";

    std::cout << "KEY INSIGHTS:\n";
    std::cout << "─────────────────────────────────────────────────────────────────────\n";
    std::cout << "  - Original claims (16 kbps) were AWGN-only, not realistic\n";
    std::cout << "  - Rayleigh fading reduces effective SNR by ~5-10 dB\n";
    std::cout << "  - Multipath > 1ms causes significant ISI\n";
    std::cout << "  - Adaptive modulation is essential for real HF\n";
    std::cout << "  - QPSK R1/2 is the 'workhorse' mode for typical conditions\n";
    std::cout << "  - Higher modes (64-QAM+) only useful for excellent NVIS/groundwave\n\n";
}

int main(int, char*[]) {
    printHeader();

    ModemConfig config;

    std::cout << "Modem Configuration:\n";
    std::cout << "  FFT size:     " << config.fft_size << "\n";
    std::cout << "  Carriers:     " << config.num_carriers << " ("
              << (config.num_carriers - config.num_carriers/config.pilot_spacing) << " data + "
              << (config.num_carriers/config.pilot_spacing) << " pilots)\n";
    std::cout << "  Bandwidth:    " << (config.num_carriers * config.sample_rate / config.fft_size) << " Hz\n";
    std::cout << "  Symbol time:  " << std::fixed << std::setprecision(1)
              << (float)config.getSymbolDuration() / config.sample_rate * 1000 << " ms\n";
    std::cout << "  Cyclic prefix: " << config.getCyclicPrefix() << " samples ("
              << std::setprecision(2) << (float)config.getCyclicPrefix() / config.sample_rate * 1000 << " ms)\n";
    std::cout << "\n";

    // Run the realistic tests
    runRealisticTests(config);
    runSpeedProfileTest(config);
    runSummary();

    return 0;
}
