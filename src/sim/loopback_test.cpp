/**
 * ProjectUltra HF Channel Simulator
 *
 * Tests modem performance under realistic HF propagation conditions
 * using the ITU-R F.1487 Watterson channel model with CCIR standard
 * test conditions (Good, Moderate, Poor, Flutter).
 */

#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>

#include "hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"  // For HilbertTransform, NCO
#include "../gui/adaptive_mode.hpp"

#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace ultra;
using namespace ultra::sim;
using namespace ultra::gui;

// LDPC block size (must match demodulator expectation)
static constexpr size_t LDPC_BLOCK_SIZE = 648;

// Get max data size (in bytes) that fits in one LDPC block for a given code rate
size_t getMaxDataBytes(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 20;   // 162 info bits
        case CodeRate::R1_2: return 40;   // 324 info bits
        case CodeRate::R2_3: return 54;   // 432 info bits
        case CodeRate::R3_4: return 60;   // 486 info bits
        case CodeRate::R5_6: return 67;   // 540 info bits
        default: return 40;
    }
}

/**
 * Apply frequency shift to a real signal using FFT-based analytic signal.
 *
 * This is the CORRECT way to shift frequency of a real signal:
 * 1. Compute analytic signal via FFT (zero negative frequencies)
 * 2. Multiply by complex exponential e^(j*2*pi*f*t) to shift spectrum
 * 3. Take real part to get back to real signal
 *
 * Unlike FIR Hilbert, the FFT approach has NO startup transients - all samples
 * are processed correctly including the very first sample (preamble start).
 */
Samples applyFrequencyShift(const Samples& input, float offset_hz, float sample_rate) {
    if (std::abs(offset_hz) < 0.01f || input.empty()) {
        return input;  // No shift needed
    }

    const size_t N = input.size();

    // Pad to power of 2 for efficient FFT (also helps with spectral leakage)
    size_t fft_size = 1;
    while (fft_size < N) fft_size *= 2;
    fft_size *= 2;  // Extra padding for better frequency resolution

    // Create FFT object
    FFT fft(fft_size);

    // Prepare complex input (real signal, zero imaginary)
    std::vector<Complex> time_in(fft_size, Complex(0.0f, 0.0f));
    for (size_t i = 0; i < N; ++i) {
        time_in[i] = Complex(input[i], 0.0f);
    }

    // Forward FFT
    std::vector<Complex> freq(fft_size);
    fft.forward(time_in, freq);

    // Create analytic signal by zeroing negative frequencies
    // Frequency layout: [DC, f1, f2, ..., fN/2, -fN/2+1, ..., -f1]
    // DC (index 0): keep as-is
    // Positive freqs (1 to N/2-1): multiply by 2
    // Nyquist (N/2): keep as-is
    // Negative freqs (N/2+1 to N-1): set to zero
    freq[0] *= 0.5f;  // DC: halve (will be doubled back when taking real part)
    for (size_t i = 1; i < fft_size / 2; ++i) {
        // Keep positive frequencies (they'll contribute to both +f and -f in real output)
    }
    freq[fft_size / 2] *= 0.5f;  // Nyquist: halve
    for (size_t i = fft_size / 2 + 1; i < fft_size; ++i) {
        freq[i] = Complex(0.0f, 0.0f);  // Zero negative frequencies
    }

    // Inverse FFT to get analytic signal
    std::vector<Complex> analytic(fft_size);
    fft.inverse(freq, analytic);

    // Apply frequency shift by multiplying with complex exponential
    // e^(j*2*pi*f*n/fs) = cos(2*pi*f*n/fs) + j*sin(2*pi*f*n/fs)
    const float phase_inc = 2.0f * static_cast<float>(M_PI) * offset_hz / sample_rate;
    Samples output(N);
    for (size_t i = 0; i < N; ++i) {
        float phase = phase_inc * static_cast<float>(i);
        Complex shift(std::cos(phase), std::sin(phase));
        Complex shifted = analytic[i] * shift;
        output[i] = shifted.real() * 2.0f;  // Factor of 2 to compensate for zeroing half spectrum
    }

    return output;
}

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
    size_t num_frames = 50,
    bool debug = false
) {
    E2EResult result = {};
    result.frames_sent = num_frames;

    // Processing stage counters (for diagnostics)
    size_t frames_detected = 0;
    size_t soft_bits_valid = 0;
    size_t ldpc_success = 0;

    // CRITICAL: Set the modulation in config so demodulator knows what to expect!
    // Without this, demodulator uses default QPSK and fails for other modulations.
    ModemConfig test_config = config;
    test_config.modulation = mod;

    // Create modem components
    OFDMModulator modulator(test_config);
    OFDMDemodulator demodulator(test_config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    // Interleaver must match LDPC block size: 648 bits = 24 x 27
    Interleaver interleaver(24, 27);

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

    // Calculate max data size for this code rate
    size_t max_data_bytes = getMaxDataBytes(rate);

    for (size_t frame = 0; frame < num_frames; ++frame) {
        // Generate test data (size based on code rate's info bits capacity)
        Bytes tx_data(max_data_bytes);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }
        result.total_bits += tx_data.size() * 8;

        // === TX Chain ===
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        // Generate preamble + data (demodulator needs preamble for sync!)
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, mod);

        // Combine preamble + data
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale to reasonable audio level (sync detector needs sufficient energy)
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // === Channel ===
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // === RX Chain ===
        // Feed to demodulator in chunks (like real-time processing)
        demodulator.reset();
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            frames_detected++;
            // Get soft bits
            std::vector<float> soft_bits = demodulator.getSoftBits();

            if (soft_bits.size() >= LDPC_BLOCK_SIZE) {
                soft_bits_valid++;
                // Deinterleave
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);

                // Decode
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    ldpc_success++;
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
            } else if (debug && frame == 0) {
                std::cout << "   Insufficient soft bits: " << soft_bits.size()
                          << " (need " << LDPC_BLOCK_SIZE << ")\n";
            }
        }
    }

    if (debug) {
        std::cout << "   Pipeline: detected=" << frames_detected
                  << " valid_bits=" << soft_bits_valid
                  << " decoded=" << ldpc_success
                  << " perfect=" << result.frames_decoded << "/" << num_frames << "\n";
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
        // Focus on 16QAM for now - this is what we need to fix
        {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2"},
        {Modulation::QAM16, CodeRate::R3_4, "16QAM R3/4"},
        // Reference: QPSK works well
        {Modulation::QPSK, CodeRate::R1_2, "QPSK R1/2"},
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
            case CodeRate::R1_4: code_rate = 0.25f; break;
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
        auto r_awgn = runE2ETest(config, mode.mod, mode.rate, ccir::awgn(20.0f), 100);
        printResultRow("AWGN 20dB", r_awgn, raw_tput);

        // Realistic conditions
        auto r_good = runE2ETest(config, mode.mod, mode.rate, ccir::good(20.0f), 100);
        printResultRow("Good", r_good, raw_tput);

        auto r_mod = runE2ETest(config, mode.mod, mode.rate, ccir::moderate(15.0f), 100);
        printResultRow("Moderate", r_mod, raw_tput);

        auto r_poor = runE2ETest(config, mode.mod, mode.rate, ccir::poor(10.0f), 100);
        printResultRow("Poor", r_poor, raw_tput);

        auto r_flutter = runE2ETest(config, mode.mod, mode.rate, ccir::flutter(8.0f), 100);
        printResultRow("Flutter", r_flutter, raw_tput);

        std::cout << "\n";
    }
}

// Compare standard config vs high-throughput config for 16QAM
void runConfigurationComparisonTest() {
    std::cout << "=== Configuration Comparison (Standard vs High-Throughput) ===\n\n";

    // Current (original) config - try WITHOUT adaptive EQ
    // Decision-directed EQ can diverge if initial decisions are wrong (16QAM issue)
    ModemConfig current;
    current.adaptive_eq_enabled = false;  // Disable adaptive EQ - use pure ZF

    // High-throughput config - also without adaptive EQ
    ModemConfig ht_config = presets::high_throughput();
    ht_config.adaptive_eq_enabled = false;  // Disable adaptive EQ

    std::cout << "Standard Config (512 FFT, 30 carriers):\n";
    std::cout << "  FFT:        " << current.fft_size << "\n";
    std::cout << "  Carriers:   " << current.num_carriers << " ("
              << current.getDataCarriers() << " data)\n";
    std::cout << "  Symbol:     " << std::fixed << std::setprecision(1)
              << (float)current.getSymbolDuration() / current.sample_rate * 1000 << " ms ("
              << current.sample_rate / current.getSymbolDuration() << " sym/s)\n";
    std::cout << "  Bandwidth:  " << (current.num_carriers * current.sample_rate / current.fft_size) << " Hz\n\n";

    std::cout << "High-Throughput Config (1024 FFT, 59 carriers):\n";
    std::cout << "  FFT:        " << ht_config.fft_size << "\n";
    std::cout << "  Carriers:   " << ht_config.num_carriers << " ("
              << ht_config.getDataCarriers() << " data)\n";
    std::cout << "  Symbol:     " << std::fixed << std::setprecision(1)
              << (float)ht_config.getSymbolDuration() / ht_config.sample_rate * 1000 << " ms ("
              << ht_config.sample_rate / ht_config.getSymbolDuration() << " sym/s)\n";
    std::cout << "  Bandwidth:  " << (ht_config.num_carriers * ht_config.sample_rate / ht_config.fft_size) << " Hz\n\n";

    // Test 16QAM R3/4 on both configs
    std::cout << "Testing 16QAM R3/4 on Moderate channel (15 dB SNR):\n";
    std::cout << "─────────────────────────────────────────────────────\n";

    // Current config
    auto r_current = runE2ETest(current, Modulation::QAM16, CodeRate::R3_4,
                                 ccir::moderate(15.0f), 100);
    float current_data_carriers = current.getDataCarriers();
    float current_sym_rate = current.sample_rate / current.getSymbolDuration();
    float current_raw = current_data_carriers * 4 * current_sym_rate * 0.75f;  // 16QAM * R3/4

    std::cout << "Standard (512 FFT): " << std::setw(3) << r_current.frames_decoded << "/100"
              << " (" << std::fixed << std::setprecision(1) << r_current.frame_success_rate << "%)"
              << "  Effective: " << (r_current.effective_throughput / 1000) << " kbps\n";

    // High-throughput config
    auto r_ht_config = runE2ETest(ht_config, Modulation::QAM16, CodeRate::R3_4,
                              ccir::moderate(15.0f), 100);
    float ht_config_data_carriers = ht_config.getDataCarriers();
    float ht_config_sym_rate = ht_config.sample_rate / ht_config.getSymbolDuration();
    float ht_config_raw = ht_config_data_carriers * 4 * ht_config_sym_rate * 0.75f;

    std::cout << "High-Tput (1024):   " << std::setw(3) << r_ht_config.frames_decoded << "/100"
              << " (" << std::fixed << std::setprecision(1) << r_ht_config.frame_success_rate << "%)"
              << "  Effective: " << (r_ht_config.effective_throughput / 1000) << " kbps\n";

    std::cout << "\nRaw throughput comparison:\n";
    std::cout << "  Standard raw:    " << std::fixed << std::setprecision(1) << current_raw / 1000 << " kbps\n";
    std::cout << "  High-Tput raw:   " << ht_config_raw / 1000 << " kbps\n";
    std::cout << "  (Target: ~7 kbps for high-speed HF data)\n\n";

    // Also test at different conditions
    std::cout << "Full comparison across conditions:\n";
    std::cout << "─────────────────────────────────────────────────────\n";
    std::cout << std::setw(16) << "Condition"
              << std::setw(16) << "Standard (512)"
              << std::setw(16) << "High-Tput\n";

    // Compare 512 FFT vs 1024 FFT configurations
    // 512 FFT: faster symbols, potentially better tracking
    // 1024 FFT: narrower carriers, less fading per carrier
    ModemConfig cfg_512;
    cfg_512.fft_size = 512;
    cfg_512.num_carriers = 30;
    cfg_512.pilot_spacing = 2;  // 50% pilots for robust estimation

    ModemConfig cfg_1024 = ht_config;  // Already configured for 1024 FFT

    auto testConfigs = [&](const char* name, const WattersonChannel::Config& ch) {
        // 512 FFT with R1/2
        auto r512 = runE2ETest(cfg_512, Modulation::QAM16, CodeRate::R1_2, ch, 50);
        float sym512 = (float)cfg_512.sample_rate / cfg_512.getSymbolDuration();
        float raw512 = cfg_512.getDataCarriers() * 4.0f * sym512 * 0.5f;
        float eff512 = raw512 * r512.frame_success_rate / 100.0f;

        // 1024 FFT with R2/3 (better for good conditions)
        auto r1024_23 = runE2ETest(cfg_1024, Modulation::QAM16, CodeRate::R2_3, ch, 50);
        float sym1024 = (float)cfg_1024.sample_rate / cfg_1024.getSymbolDuration();
        float raw1024 = cfg_1024.getDataCarriers() * 4.0f * sym1024 * 0.667f;
        float eff1024 = raw1024 * r1024_23.frame_success_rate / 100.0f;

        // Pick best effective throughput
        float best = std::max(eff512, eff1024);
        const char* best_cfg = (eff512 > eff1024) ? "512" : "1024";

        std::cout << std::setw(14) << name
                  << "  512:" << std::setw(3) << (int)r512.frame_success_rate << "% "
                  << std::fixed << std::setprecision(1) << (eff512/1000) << "kb"
                  << "  1024:" << std::setw(3) << (int)r1024_23.frame_success_rate << "% "
                  << (eff1024/1000) << "kb"
                  << "  Best: " << (best/1000) << "kb (" << best_cfg << ")\n";
    };

    // =========================================================================
    // SPEED LADDER: Find maximum achievable throughput at each SNR level
    // Goal: Find path to 7+ kbps (high-speed HF data target)
    // =========================================================================
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "        SPEED LADDER: Maximum Throughput by Condition\n";
    std::cout << std::string(70, '=') << "\n\n";

    float sym_rate = (float)cfg_1024.sample_rate / cfg_1024.getSymbolDuration();
    uint32_t data_carriers = cfg_1024.getDataCarriers();

    // Calculate raw throughputs
    auto calcRaw = [&](int bits, float code_rate) {
        return data_carriers * bits * sym_rate * code_rate;
    };

    std::cout << "Configuration: 1024 FFT, " << data_carriers << " data carriers, "
              << std::fixed << std::setprecision(0) << sym_rate << " sym/s\n";
    std::cout << "Theoretical maximums (100% decode rate):\n";
    std::cout << "  16-QAM R2/3: " << std::setprecision(1) << calcRaw(4, 0.667f)/1000 << " kbps\n";
    std::cout << "  16-QAM R3/4: " << calcRaw(4, 0.75f)/1000 << " kbps\n";
    std::cout << "  32-QAM R2/3: " << calcRaw(5, 0.667f)/1000 << " kbps\n";
    std::cout << "  32-QAM R3/4: " << calcRaw(5, 0.75f)/1000 << " kbps  <-- 7 kbps target\n";
    std::cout << "  64-QAM R2/3: " << calcRaw(6, 0.667f)/1000 << " kbps\n";
    std::cout << "  64-QAM R3/4: " << calcRaw(6, 0.75f)/1000 << " kbps  <-- 8 kbps target\n\n";

    // Test function that tries all viable modes and finds best
    auto findBestMode = [&](const char* name, const WattersonChannel::Config& ch, int num_frames = 30) {
        struct Mode {
            Modulation mod;
            CodeRate rate;
            const char* name;
            float code_val;
            int bits;
        };
        Mode modes[] = {
            {Modulation::QPSK,  CodeRate::R1_2, "QPSK R1/2",   0.5f,   2},
            {Modulation::QAM16, CodeRate::R1_2, "16QAM R1/2",  0.5f,   4},
            {Modulation::QAM16, CodeRate::R2_3, "16QAM R2/3",  0.667f, 4},
            {Modulation::QAM16, CodeRate::R3_4, "16QAM R3/4",  0.75f,  4},
            {Modulation::QAM32, CodeRate::R1_2, "32QAM R1/2",  0.5f,   5},
            {Modulation::QAM32, CodeRate::R2_3, "32QAM R2/3",  0.667f, 5},
            {Modulation::QAM32, CodeRate::R3_4, "32QAM R3/4",  0.75f,  5},
            {Modulation::QAM64, CodeRate::R1_2, "64QAM R1/2",  0.5f,   6},
            {Modulation::QAM64, CodeRate::R2_3, "64QAM R2/3",  0.667f, 6},
            {Modulation::QAM64, CodeRate::R3_4, "64QAM R3/4",  0.75f,  6},
        };

        float best_eff = 0;
        const char* best_mode = "none";
        float best_raw = 0;
        float best_success = 0;

        std::cout << std::setw(16) << std::left << name << std::right;

        for (const auto& m : modes) {
            auto result = runE2ETest(cfg_1024, m.mod, m.rate, ch, num_frames);
            float raw = calcRaw(m.bits, m.code_val);
            float eff = raw * result.frame_success_rate / 100.0f;

            // Only consider modes with >70% success (usable with ARQ)
            if (eff > best_eff && result.frame_success_rate > 70.0f) {
                best_eff = eff;
                best_mode = m.name;
                best_raw = raw;
                best_success = result.frame_success_rate;
            }
        }

        if (best_eff > 0) {
            std::cout << std::fixed << std::setprecision(1)
                      << "  " << std::setw(11) << best_mode
                      << "  " << std::setw(3) << (int)best_success << "%"
                      << "  " << std::setw(4) << best_eff/1000 << " kbps"
                      << (best_eff >= 7000 ? "  <-- 7KB+ ACHIEVED!" : "") << "\n";
        } else {
            std::cout << "  (no mode >70% success)\n";
        }

        return best_eff;
    };

    std::cout << "Condition        Best Mode     Success   Effective\n";
    std::cout << "────────────────────────────────────────────────────\n";

    // Test across realistic conditions
    findBestMode("AWGN 30dB",     ccir::awgn(30.0f));
    findBestMode("AWGN 25dB",     ccir::awgn(25.0f));
    findBestMode("AWGN 20dB",     ccir::awgn(20.0f));
    findBestMode("Good 25dB",     ccir::good(25.0f));
    findBestMode("Good 20dB",     ccir::good(20.0f));
    findBestMode("Good 15dB",     ccir::good(15.0f));
    findBestMode("Moderate 25dB", ccir::moderate(25.0f));
    findBestMode("Moderate 20dB", ccir::moderate(20.0f));
    findBestMode("Moderate 15dB", ccir::moderate(15.0f));
    findBestMode("Poor 15dB",     ccir::poor(15.0f));
    findBestMode("Poor 10dB",     ccir::poor(10.0f));

    // =========================================================================
    // DETAILED 64-QAM ANALYSIS
    // =========================================================================
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "        64-QAM Detailed Analysis (Path to 7 kbps)\n";
    std::cout << std::string(70, '=') << "\n\n";

    std::cout << "Testing 64-QAM at higher SNRs to find working conditions...\n\n";
    std::cout << "Condition        R1/2       R2/3       R3/4       Best Effective\n";
    std::cout << "─────────────────────────────────────────────────────────────────\n";

    auto test64QAM = [&](const char* name, const WattersonChannel::Config& ch) {
        auto r12 = runE2ETest(cfg_1024, Modulation::QAM64, CodeRate::R1_2, ch, 30);
        auto r23 = runE2ETest(cfg_1024, Modulation::QAM64, CodeRate::R2_3, ch, 30);
        auto r34 = runE2ETest(cfg_1024, Modulation::QAM64, CodeRate::R3_4, ch, 30);

        float eff12 = calcRaw(6, 0.5f) * r12.frame_success_rate / 100.0f;
        float eff23 = calcRaw(6, 0.667f) * r23.frame_success_rate / 100.0f;
        float eff34 = calcRaw(6, 0.75f) * r34.frame_success_rate / 100.0f;

        float best = std::max({eff12, eff23, eff34});
        const char* best_rate = (best == eff12) ? "R1/2" : (best == eff23) ? "R2/3" : "R3/4";

        std::cout << std::setw(16) << std::left << name << std::right
                  << std::fixed << std::setprecision(0)
                  << std::setw(6) << r12.frame_success_rate << "%"
                  << std::setw(10) << r23.frame_success_rate << "%"
                  << std::setw(10) << r34.frame_success_rate << "%"
                  << std::setprecision(1)
                  << "     " << best/1000 << " kb (" << best_rate << ")"
                  << (best >= 7000 ? " ***" : "") << "\n";
    };

    test64QAM("AWGN 35dB", ccir::awgn(35.0f));
    test64QAM("AWGN 30dB", ccir::awgn(30.0f));
    test64QAM("AWGN 25dB", ccir::awgn(25.0f));
    test64QAM("Good 30dB", ccir::good(30.0f));
    test64QAM("Good 25dB", ccir::good(25.0f));
    test64QAM("Good 20dB", ccir::good(20.0f));

    std::cout << "\n*** = 7+ kbps achieved\n";

    // =========================================================================
    // 32-QAM ANALYSIS (target: 7+ kbps)
    // 32QAM = 5 bits/symbol, between 16QAM (4) and 64QAM (6)
    // =========================================================================
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "        32-QAM Analysis (7 kbps target)\n";
    std::cout << std::string(70, '=') << "\n\n";

    std::cout << "Theoretical: " << calcRaw(5, 0.667f)/1000 << " kbps (R2/3), "
              << calcRaw(5, 0.75f)/1000 << " kbps (R3/4)\n\n";

    std::cout << "Comparison: 16QAM vs 32QAM vs 64QAM (all R2/3)\n";
    std::cout << "Condition        16QAM      32QAM      64QAM\n";
    std::cout << "────────────────────────────────────────────────\n";

    auto compareModulations = [&](const char* name, const WattersonChannel::Config& ch) {
        auto r16 = runE2ETest(cfg_1024, Modulation::QAM16, CodeRate::R2_3, ch, 30);
        auto r32 = runE2ETest(cfg_1024, Modulation::QAM32, CodeRate::R2_3, ch, 30);
        auto r64 = runE2ETest(cfg_1024, Modulation::QAM64, CodeRate::R2_3, ch, 30);

        float eff16 = calcRaw(4, 0.667f) * r16.frame_success_rate / 100.0f;
        float eff32 = calcRaw(5, 0.667f) * r32.frame_success_rate / 100.0f;
        float eff64 = calcRaw(6, 0.667f) * r64.frame_success_rate / 100.0f;

        std::cout << std::setw(16) << std::left << name << std::right
                  << std::fixed << std::setprecision(1)
                  << std::setw(5) << eff16/1000 << "kb"
                  << std::setw(9) << eff32/1000 << "kb"
                  << std::setw(9) << eff64/1000 << "kb\n";
    };

    compareModulations("AWGN 30dB", ccir::awgn(30.0f));
    compareModulations("AWGN 25dB", ccir::awgn(25.0f));
    compareModulations("Good 25dB", ccir::good(25.0f));
    compareModulations("Good 20dB", ccir::good(20.0f));
    compareModulations("Moderate 25dB", ccir::moderate(25.0f));

    std::cout << "\n32-QAM R3/4 detailed test:\n";
    auto test32QAM = [&](const char* name, const WattersonChannel::Config& ch) {
        auto result = runE2ETest(cfg_1024, Modulation::QAM32, CodeRate::R3_4, ch, 30);
        float raw = calcRaw(5, 0.75f);
        float eff = raw * result.frame_success_rate / 100.0f;
        std::cout << "  " << std::setw(14) << name
                  << std::setw(4) << (int)result.frame_success_rate << "%"
                  << std::setw(6) << std::fixed << std::setprecision(1) << eff/1000 << " kbps"
                  << (eff >= 7000 ? "  <-- 7 kbps target achieved!" : "") << "\n";
    };
    test32QAM("AWGN 30dB", ccir::awgn(30.0f));
    test32QAM("AWGN 25dB", ccir::awgn(25.0f));

    // Test 32QAM with denser pilots to see if channel estimation is the bottleneck
    std::cout << "\n32-QAM with DENSE PILOTS (spacing=2):\n";
    {
        ModemConfig cfg_dense = cfg_1024;
        cfg_dense.pilot_spacing = 2;  // Very dense pilots

        uint32_t pilots_dense = (cfg_dense.num_carriers + 2 - 1) / 2;
        uint32_t data_dense = cfg_dense.num_carriers - pilots_dense;
        float sym_dense = (float)cfg_dense.sample_rate / cfg_dense.getSymbolDuration();
        auto calcRawDense = [data_dense, sym_dense](int bits, float code_rate) {
            return data_dense * bits * sym_dense * code_rate;
        };

        std::cout << "  Config: " << data_dense << " data carriers (vs 44 with spacing=4)\n";
        std::cout << "  Theoretical 32QAM R2/3: " << calcRawDense(5, 0.667f)/1000 << " kbps\n\n";

        auto testDense32 = [&](const char* name, const WattersonChannel::Config& ch) {
            auto result = runE2ETest(cfg_dense, Modulation::QAM32, CodeRate::R2_3, ch, 30);
            float raw = calcRawDense(5, 0.667f);
            float eff = raw * result.frame_success_rate / 100.0f;
            std::cout << "  " << std::setw(14) << name
                      << std::setw(4) << (int)result.frame_success_rate << "%"
                      << std::setw(6) << std::fixed << std::setprecision(1) << eff/1000 << " kbps\n";
        };
        testDense32("AWGN 25dB", ccir::awgn(25.0f));
        testDense32("Good 25dB", ccir::good(25.0f));
        testDense32("Good 20dB", ccir::good(20.0f));
    }
    test32QAM("Good 25dB", ccir::good(25.0f));
    test32QAM("Good 20dB", ccir::good(20.0f));

    // =========================================================================
    // PILOT OVERHEAD OPTIMIZATION
    // Can we push throughput higher by using fewer pilots?
    // =========================================================================
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "        PILOT OVERHEAD OPTIMIZATION\n";
    std::cout << std::string(70, '=') << "\n\n";

    std::cout << "Testing 16-QAM R2/3 with different pilot spacings...\n";
    std::cout << "(Current: pilot_spacing=3 → 39 data carriers)\n\n";

    auto testPilotSpacing = [&](int spacing, const WattersonChannel::Config& ch) {
        ModemConfig cfg = cfg_1024;
        cfg.pilot_spacing = spacing;

        uint32_t pilots = (cfg.num_carriers + spacing - 1) / spacing;
        uint32_t data = cfg.num_carriers - pilots;
        float sym = (float)cfg.sample_rate / cfg.getSymbolDuration();
        float raw = data * 4.0f * sym * 0.667f;  // 16-QAM R2/3

        auto result = runE2ETest(cfg, Modulation::QAM16, CodeRate::R2_3, ch, 30);
        float eff = raw * result.frame_success_rate / 100.0f;

        std::cout << "  spacing=" << spacing << " (" << data << " data):"
                  << std::setw(8) << std::fixed << std::setprecision(1) << raw/1000 << " raw"
                  << " × " << std::setw(3) << (int)result.frame_success_rate << "%"
                  << " = " << std::setw(4) << eff/1000 << " kbps\n";

        return std::make_pair(eff, result.frame_success_rate);
    };

    std::cout << "Good 25dB channel:\n";
    testPilotSpacing(3, ccir::good(25.0f));
    testPilotSpacing(4, ccir::good(25.0f));
    testPilotSpacing(5, ccir::good(25.0f));
    testPilotSpacing(6, ccir::good(25.0f));

    std::cout << "\nGood 20dB channel:\n";
    testPilotSpacing(3, ccir::good(20.0f));
    testPilotSpacing(4, ccir::good(20.0f));
    testPilotSpacing(5, ccir::good(20.0f));
    testPilotSpacing(6, ccir::good(20.0f));

    std::cout << "\nModerate 20dB channel:\n";
    testPilotSpacing(3, ccir::moderate(20.0f));
    testPilotSpacing(4, ccir::moderate(20.0f));
    testPilotSpacing(5, ccir::moderate(20.0f));
    testPilotSpacing(6, ccir::moderate(20.0f));

    // =========================================================================
    // CYCLIC PREFIX OPTIMIZATION
    // Can we go faster with shorter CP?
    // =========================================================================
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "        CYCLIC PREFIX OPTIMIZATION\n";
    std::cout << std::string(70, '=') << "\n\n";

    std::cout << "Testing 16-QAM R2/3 with different CP lengths...\n\n";

    auto testCPMode = [&](CyclicPrefixMode cp_mode, const char* cp_name,
                          const WattersonChannel::Config& ch) {
        ModemConfig cfg = cfg_1024;
        cfg.cp_mode = cp_mode;

        uint32_t cp = cfg.getCyclicPrefix();
        float sym = (float)cfg.sample_rate / cfg.getSymbolDuration();
        float raw = cfg.getDataCarriers() * 4.0f * sym * 0.667f;

        auto result = runE2ETest(cfg, Modulation::QAM16, CodeRate::R2_3, ch, 30);
        float eff = raw * result.frame_success_rate / 100.0f;

        std::cout << "  " << std::setw(8) << cp_name << " (" << cp << " samp, "
                  << std::fixed << std::setprecision(0) << sym << " sym/s):"
                  << std::setprecision(1) << std::setw(6) << raw/1000 << " raw"
                  << " × " << std::setw(3) << (int)result.frame_success_rate << "%"
                  << " = " << std::setw(4) << eff/1000 << " kbps\n";

        return eff;
    };

    std::cout << "Good 20dB channel:\n";
    testCPMode(CyclicPrefixMode::SHORT,  "SHORT",  ccir::good(20.0f));
    testCPMode(CyclicPrefixMode::MEDIUM, "MEDIUM", ccir::good(20.0f));
    testCPMode(CyclicPrefixMode::LONG,   "LONG",   ccir::good(20.0f));

    std::cout << "\nModerate 20dB channel:\n";
    testCPMode(CyclicPrefixMode::SHORT,  "SHORT",  ccir::moderate(20.0f));
    testCPMode(CyclicPrefixMode::MEDIUM, "MEDIUM", ccir::moderate(20.0f));
    testCPMode(CyclicPrefixMode::LONG,   "LONG",   ccir::moderate(20.0f));

    // =========================================================================
    // HIGH-THROUGHPUT ANALYSIS: Can we reach 7 kbps with 16QAM?
    // Target: 42 sym/s, 59 carriers, 16QAM → ~7 kbps
    // That implies ~71% efficiency (R3/4 with minimal pilot overhead)
    // =========================================================================
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "        HIGH-THROUGHPUT TARGET (7 kbps)\n";
    std::cout << std::string(70, '=') << "\n\n";

    std::cout << "Target config: 42 sym/s, 59 carriers, 16QAM = 9912 raw bps\n";
    std::cout << "               7000 net / 9912 raw = 70.6% efficiency\n";
    std::cout << "               Implies R3/4 FEC + minimal pilot overhead\n\n";

    // Test: Can dense pilots make R3/4 work?
    std::cout << "TEST 1: Can dense pilots make 16QAM R3/4 work?\n";
    std::cout << "────────────────────────────────────────────────────\n";

    auto testR34WithPilots = [&](int spacing, const WattersonChannel::Config& ch) {
        ModemConfig cfg = presets::high_throughput();
        cfg.pilot_spacing = spacing;
        cfg.code_rate = CodeRate::R3_4;

        uint32_t pilots = (cfg.num_carriers + spacing - 1) / spacing;
        uint32_t data = cfg.num_carriers - pilots;
        float sym = (float)cfg.sample_rate / cfg.getSymbolDuration();
        float raw = data * 4.0f * sym * 0.75f;  // 16QAM R3/4

        auto result = runE2ETest(cfg, Modulation::QAM16, CodeRate::R3_4, ch, 40);
        float eff = raw * result.frame_success_rate / 100.0f;

        std::cout << "  pilot=" << spacing << " (" << std::setw(2) << data << " data):"
                  << std::setw(6) << std::fixed << std::setprecision(1) << raw/1000 << " raw"
                  << " × " << std::setw(3) << (int)result.frame_success_rate << "%"
                  << " = " << std::setw(4) << eff/1000 << " kbps"
                  << (result.frame_success_rate >= 90 ? "  <-- WORKS!" : "") << "\n";

        return result.frame_success_rate;
    };

    std::cout << "\nGood 25dB (best fading condition):\n";
    testR34WithPilots(2, ccir::good(25.0f));
    testR34WithPilots(3, ccir::good(25.0f));
    testR34WithPilots(4, ccir::good(25.0f));

    std::cout << "\nGood 20dB:\n";
    testR34WithPilots(2, ccir::good(20.0f));
    testR34WithPilots(3, ccir::good(20.0f));
    testR34WithPilots(4, ccir::good(20.0f));

    std::cout << "\nModerate 25dB:\n";
    testR34WithPilots(2, ccir::moderate(25.0f));
    testR34WithPilots(3, ccir::moderate(25.0f));

    // Test: What's the theoretical max if we use ALL carriers for data?
    std::cout << "\n\nTEST 2: Theoretical throughput with decision-directed EQ\n";
    std::cout << "────────────────────────────────────────────────────\n";
    std::cout << "(Assumes perfect channel tracking, all 59 carriers = data)\n\n";

    float sym_r = 42.0f;  // High-throughput symbol rate
    float all_data = 59.0f;  // All carriers for data

    std::cout << "59 carriers × 4 bits (16QAM) × 42 sym/s = "
              << (int)(all_data * 4 * sym_r) << " raw bps\n";
    std::cout << "  With R1/2: " << (int)(all_data * 4 * sym_r * 0.5) << " bps\n";
    std::cout << "  With R2/3: " << (int)(all_data * 4 * sym_r * 0.667) << " bps\n";
    std::cout << "  With R3/4: " << (int)(all_data * 4 * sym_r * 0.75) << " bps  <-- 7 kbps target\n";

    // Test: What if we boost pilot power?
    std::cout << "\n\nTEST 3: Effect of channel estimation quality\n";
    std::cout << "────────────────────────────────────────────────────\n";
    std::cout << "Testing at very high SNR to isolate channel estimation effects...\n\n";

    auto highSNR = [&](const char* name, const WattersonChannel::Config& ch) {
        auto r = runE2ETest(cfg_1024, Modulation::QAM16, CodeRate::R3_4, ch, 40);
        std::cout << "  " << std::setw(14) << std::left << name << std::right
                  << ": " << std::setw(3) << (int)r.frame_success_rate << "% success\n";
    };

    highSNR("Good 30dB", ccir::good(30.0f));
    highSNR("Good 35dB", ccir::good(35.0f));
    highSNR("Good 40dB", ccir::good(40.0f));
    highSNR("Moderate 30dB", ccir::moderate(30.0f));
    highSNR("Moderate 35dB", ccir::moderate(35.0f));

    std::cout << "\n";
}

void runSpeedProfileTest(const ModemConfig& config) {
    std::cout << "\n=== Speed Profile Performance Under Realistic Conditions ===\n\n";

    // Conservative profile
    std::cout << "CONSERVATIVE (QPSK R1/2 - for poor conditions):\n";
    auto cons_good = runE2ETest(config, Modulation::QPSK, CodeRate::R1_2, ccir::good(15.0f), 100);
    auto cons_mod = runE2ETest(config, Modulation::QPSK, CodeRate::R1_2, ccir::moderate(12.0f), 100);
    auto cons_poor = runE2ETest(config, Modulation::QPSK, CodeRate::R1_2, ccir::poor(8.0f), 100);
    std::cout << "  Good channel:     " << std::fixed << std::setprecision(1)
              << cons_good.effective_throughput / 1000 << " kbps (" << cons_good.frame_success_rate << "% success)\n";
    std::cout << "  Moderate channel: " << cons_mod.effective_throughput / 1000 << " kbps (" << cons_mod.frame_success_rate << "% success)\n";
    std::cout << "  Poor channel:     " << cons_poor.effective_throughput / 1000 << " kbps (" << cons_poor.frame_success_rate << "% success)\n\n";

    // Balanced profile
    std::cout << "BALANCED (64-QAM R3/4 - for typical conditions):\n";
    auto bal_good = runE2ETest(config, Modulation::QAM64, CodeRate::R3_4, ccir::good(20.0f), 100);
    auto bal_mod = runE2ETest(config, Modulation::QAM64, CodeRate::R3_4, ccir::moderate(15.0f), 100);
    auto bal_poor = runE2ETest(config, Modulation::QAM64, CodeRate::R3_4, ccir::poor(10.0f), 100);
    std::cout << "  Good channel:     " << std::fixed << std::setprecision(1)
              << bal_good.effective_throughput / 1000 << " kbps (" << bal_good.frame_success_rate << "% success)\n";
    std::cout << "  Moderate channel: " << bal_mod.effective_throughput / 1000 << " kbps (" << bal_mod.frame_success_rate << "% success)\n";
    std::cout << "  Poor channel:     " << bal_poor.effective_throughput / 1000 << " kbps (" << bal_poor.frame_success_rate << "% success)\n\n";

    // Turbo profile
    std::cout << "TURBO (256-QAM R7/8 - needs excellent conditions):\n";
    auto turbo_awgn = runE2ETest(config, Modulation::QAM256, CodeRate::R7_8, ccir::awgn(30.0f), 100);
    auto turbo_good = runE2ETest(config, Modulation::QAM256, CodeRate::R7_8, ccir::good(25.0f), 100);
    auto turbo_mod = runE2ETest(config, Modulation::QAM256, CodeRate::R7_8, ccir::moderate(20.0f), 100);
    std::cout << "  AWGN 30dB:        " << std::fixed << std::setprecision(1)
              << turbo_awgn.effective_throughput / 1000 << " kbps (" << turbo_awgn.frame_success_rate << "% success)\n";
    std::cout << "  Good channel:     " << turbo_good.effective_throughput / 1000 << " kbps (" << turbo_good.frame_success_rate << "% success)\n";
    std::cout << "  Moderate channel: " << turbo_mod.effective_throughput / 1000 << " kbps (" << turbo_mod.frame_success_rate << "% success)\n\n";
}

void runSNREstimationTest(const ModemConfig& config) {
    std::cout << "\n=== SNR Estimation Test ===\n\n";
    std::cout << "Testing demodulator's pilot-based SNR estimation.\n";
    std::cout << "The estimate is per-pilot SNR useful for modulation selection.\n\n";

    std::cout << std::setw(14) << "Channel SNR"
              << std::setw(14) << "Pilot SNR"
              << std::setw(12) << "Delta\n";
    std::cout << "────────────────────────────────────────────\n";

    OFDMModulator modulator(config);
    OFDMDemodulator demodulator(config);
    LDPCEncoder encoder(CodeRate::R1_2);

    // Test SNR estimation across range of channel SNRs
    float test_snrs[] = {25.0f, 30.0f, 35.0f, 40.0f, 45.0f, 50.0f};

    for (float actual_snr : test_snrs) {
        // Create AWGN channel at this SNR
        auto channel_cfg = ccir::awgn(actual_snr);
        WattersonChannel channel(channel_cfg);

        // Generate test frame
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>(i * 7 + 13);
        }

        // TX chain - must include preamble for sync!
        Bytes encoded = encoder.encode(tx_data);
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(encoded, Modulation::QPSK);

        // Combine preamble + data
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale signal to reasonable audio level (like real modem does)
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;  // Target 0.5 peak
            for (float& s : tx_audio) s *= scale;
        }

        // Pass through channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // Debug: check signal RMS
        float tx_rms = 0, rx_rms = 0;
        for (float s : tx_audio) tx_rms += s * s;
        for (float s : rx_audio) rx_rms += s * s;
        tx_rms = std::sqrt(tx_rms / tx_audio.size());
        rx_rms = std::sqrt(rx_rms / rx_audio.size());

        // RX - process until frame ready or synced
        demodulator.reset();

        // Feed samples in chunks (like real-time processing)
        size_t chunk_size = 1024;
        bool frame_ready = false;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        // Get estimated SNR (will be updated if sync found and symbols processed)
        float estimated_snr = demodulator.getEstimatedSNR();
        bool synced = demodulator.isSynced();

        // Track change from previous measurement
        static float prev_channel_snr = 0, prev_pilot_snr = 0;
        float delta = (prev_pilot_snr > 0) ? (estimated_snr - prev_pilot_snr) : 0;
        float expected_delta = actual_snr - prev_channel_snr;

        std::cout << std::setw(10) << std::fixed << std::setprecision(1) << actual_snr << " dB"
                  << std::setw(12) << estimated_snr << " dB";
        if (prev_pilot_snr > 0 && synced) {
            std::cout << std::setw(10) << std::showpos << delta << " dB" << std::noshowpos;
        } else {
            std::cout << std::setw(10) << "    -";
        }
        std::cout << (synced ? "  OK" : "  (no sync)") << "\n";

        if (synced) {
            prev_channel_snr = actual_snr;
            prev_pilot_snr = estimated_snr;
        }
    }

    std::cout << "\nNote: Pilot SNR tracks channel SNR changes (5 dB channel → 5 dB pilot).\n";
    std::cout << "Absolute offset reflects measurement methodology (per-pilot vs broadband).\n\n";
}

// Helper to get modulation name
static const char* getModName(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK: return "BPSK";
        case Modulation::QPSK: return "QPSK";
        case Modulation::QAM16: return "16QAM";
        case Modulation::QAM32: return "32QAM";
        case Modulation::QAM64: return "64QAM";
        case Modulation::QAM256: return "256QAM";
        default: return "?";
    }
}

// Helper to get code rate name
static const char* getRateName(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return "1/4";
        case CodeRate::R1_3: return "1/3";
        case CodeRate::R1_2: return "1/2";
        case CodeRate::R2_3: return "2/3";
        case CodeRate::R3_4: return "3/4";
        case CodeRate::R5_6: return "5/6";
        case CodeRate::R7_8: return "7/8";
        default: return "?";
    }
}

void runAdaptiveModulationTest() {
    std::cout << "\n=== Adaptive Modulation Test ===\n\n";
    std::cout << "Testing automatic mode selection based on SNR.\n\n";

    AdaptiveModeController adaptive;

    // Test 1: Mode selection at different SNR levels
    std::cout << "1. Mode Selection by SNR:\n";
    std::cout << "   SNR (dB)    Selected Mode\n";
    std::cout << "   ─────────────────────────────\n";

    float test_snrs[] = {15.0f, 20.0f, 24.0f, 28.0f, 32.0f, 36.0f, 40.0f};
    for (float snr : test_snrs) {
        adaptive.reset();
        // Update multiple times to get past hysteresis
        for (int i = 0; i < 5; ++i) {
            adaptive.update(snr);
        }
        std::cout << "   " << std::setw(6) << std::fixed << std::setprecision(1) << snr
                  << "       " << getModName(adaptive.getModulation())
                  << " R" << getRateName(adaptive.getCodeRate()) << "\n";
    }

    // Test 2: Hysteresis behavior
    std::cout << "\n2. Hysteresis Test:\n";
    std::cout << "   Starting at SNR=30 dB, then small changes...\n";

    adaptive.reset();
    // Stabilize at 30 dB
    for (int i = 0; i < 5; ++i) adaptive.update(30.0f);
    Modulation initial_mod = adaptive.getModulation();
    CodeRate initial_rate = adaptive.getCodeRate();
    std::cout << "   Initial: " << getModName(initial_mod) << " R" << getRateName(initial_rate) << "\n";

    // Small decrease (should NOT trigger change due to hysteresis)
    bool changed = adaptive.update(29.0f);
    std::cout << "   After 29 dB: " << getModName(adaptive.getModulation())
              << " R" << getRateName(adaptive.getCodeRate())
              << (changed ? " (CHANGED)" : " (stable)") << "\n";

    // Larger decrease (should trigger change)
    for (int i = 0; i < 5; ++i) {
        changed = adaptive.update(24.0f);
    }
    std::cout << "   After 24 dB: " << getModName(adaptive.getModulation())
              << " R" << getRateName(adaptive.getCodeRate())
              << (adaptive.getModulation() != initial_mod ? " (CHANGED)" : " (stable)") << "\n";

    // Test 3: Full TX/RX with adaptive mode
    std::cout << "\n3. Full TX/RX Test with Adaptive Mode:\n";

    ModemConfig config;
    config.speed_profile = SpeedProfile::ADAPTIVE;

    OFDMModulator modulator(config);
    OFDMDemodulator demodulator(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);

    // Test at different channel SNRs
    float channel_snrs[] = {30.0f, 40.0f, 50.0f};

    std::cout << "   Channel SNR    Adaptive Mode    Result\n";
    std::cout << "   ───────────────────────────────────────────\n";

    for (float channel_snr : channel_snrs) {
        // Create channel
        auto channel_cfg = ccir::awgn(channel_snr);
        WattersonChannel channel(channel_cfg);

        // Reset adaptive controller
        adaptive.reset();

        // Generate test data
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>(i * 7 + 13);
        }

        // Simulate receiving at this SNR to set adaptive mode
        // (In real use, SNR comes from previous frame reception)
        float simulated_pilot_snr = channel_snr - 5.0f;  // Approximate offset
        for (int i = 0; i < 5; ++i) adaptive.update(simulated_pilot_snr);

        Modulation selected_mod = adaptive.getModulation();
        CodeRate selected_rate = adaptive.getCodeRate();

        // Update encoder/decoder for selected rate
        encoder.setRate(selected_rate);
        decoder.setRate(selected_rate);

        // TX chain
        Bytes encoded = encoder.encode(tx_data);
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(encoded, selected_mod);

        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // Channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // RX chain - update demodulator config for selected modulation
        ModemConfig rx_config = config;
        rx_config.modulation = selected_mod;
        rx_config.code_rate = selected_rate;
        OFDMDemodulator rx_demod(rx_config);

        // Process
        SampleSpan rx_span(rx_audio.data(), rx_audio.size());
        bool frame_ready = rx_demod.process(rx_span);

        std::string result;
        if (frame_ready) {
            auto soft_bits = rx_demod.getSoftBits();
            Bytes decoded = decoder.decodeSoft(soft_bits);
            bool success = decoder.lastDecodeSuccess();

            if (success && decoded.size() >= tx_data.size()) {
                bool match = true;
                for (size_t i = 0; i < tx_data.size(); ++i) {
                    if (decoded[i] != tx_data[i]) {
                        match = false;
                        break;
                    }
                }
                result = match ? "OK" : "DATA MISMATCH";
            } else {
                result = "DECODE FAIL";
            }
        } else {
            result = "NO SYNC";
        }

        std::cout << "   " << std::setw(6) << channel_snr << " dB"
                  << "       " << std::setw(5) << getModName(selected_mod)
                  << " R" << std::setw(3) << getRateName(selected_rate)
                  << "       " << result << "\n";
    }

    std::cout << "\n   Adaptive modulation test complete.\n\n";
}

// ============================================================================
// INTERLEAVING BENEFIT TEST
// Compare performance WITH vs WITHOUT interleaving under fading conditions
// Uses same structure as runE2ETest for consistency
// ============================================================================

// Helper: Run E2E test WITHOUT interleaving (to compare against standard path)
E2EResult runE2ETestNoInterleave(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    const WattersonChannel::Config& channel_cfg,
    size_t num_frames = 50
) {
    E2EResult result = {};
    result.frames_sent = num_frames;

    // CRITICAL: Set the modulation in config so demodulator knows what to expect!
    ModemConfig test_config = config;
    test_config.modulation = mod;

    OFDMModulator modulator(test_config);
    OFDMDemodulator demodulator(test_config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    // NO interleaver!

    WattersonChannel channel(channel_cfg);

    for (size_t frame = 0; frame < num_frames; ++frame) {
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }
        result.total_bits += tx_data.size() * 8;

        // TX: encode but NO interleave
        Bytes encoded = encoder.encode(tx_data);

        // Generate preamble + data (demodulator needs preamble for sync!)
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(encoded, mod);  // Direct, no interleave

        // Combine preamble + data
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale to reasonable audio level
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // Channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // RX: decode but NO deinterleave (feed in chunks)
        demodulator.reset();
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();
            if (soft_bits.size() >= LDPC_BLOCK_SIZE) {
                // Direct decode, no deinterleave
                Bytes rx_data = decoder.decodeSoft(soft_bits);

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
        }
    }

    result.frame_success_rate = 100.0f * result.frames_decoded / result.frames_sent;
    result.ber = result.total_bits > 0 ?
        static_cast<float>(result.bit_errors) / result.total_bits : 1.0f;

    return result;
}

void runInterleavingBenefitTest(const ModemConfig& config) {
    std::cout << "\n=== Interleaving Benefit Test ===\n\n";
    std::cout << "Simulating burst errors to demonstrate interleaving benefit.\n";
    std::cout << "Burst errors (from fading) are spread by interleaving, helping LDPC.\n\n";

    // Use soft bit simulation to test interleaving benefit (bypasses sync issues)
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);
    Interleaver interleaver(24, 27);  // Match LDPC block size: 648 bits

    const size_t NUM_FRAMES = 200;

    // Test different burst error scenarios
    struct TestCase {
        const char* name;
        size_t burst_length;  // Length of burst error in bits
        float base_snr_db;    // Base SNR for non-burst bits
    };

    TestCase tests[] = {
        {"No burst (AWGN only)", 0, 4.0f},
        {"24-bit burst", 24, 4.0f},      // Matches interleaver row size
        {"48-bit burst", 48, 4.0f},      // 2 rows
        {"96-bit burst", 96, 4.0f},      // 4 rows
        {"144-bit burst", 144, 4.0f},    // 6 rows
    };

    std::cout << "   Condition          WITH Interleaving    WITHOUT Interleaving    Gain\n";
    std::cout << "   ──────────────────────────────────────────────────────────────────────\n";

    std::mt19937 rng(42);
    std::normal_distribution<float> noise(0.0f, 1.0f);

    for (const auto& test : tests) {
        float noise_std = std::pow(10.0f, -test.base_snr_db / 20.0f);

        size_t with_success = 0, without_success = 0;

        for (size_t frame = 0; frame < NUM_FRAMES; ++frame) {
            // Generate test data
            Bytes tx_data(40);
            for (size_t i = 0; i < tx_data.size(); ++i) {
                tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
            }

            // Encode
            Bytes encoded = encoder.encode(tx_data);

            // Convert to soft bits (LLRs) and add AWGN
            auto makeSoftBits = [&](const Bytes& data) {
                std::vector<float> soft_bits;
                for (uint8_t byte : data) {
                    for (int b = 7; b >= 0; --b) {
                        uint8_t bit = (byte >> b) & 1;
                        float llr = bit ? -4.0f : 4.0f;  // Perfect LLR
                        llr += noise_std * noise(rng);   // Add noise
                        soft_bits.push_back(llr);
                    }
                }
                return soft_bits;
            };

            // Apply burst error (simulate deep fade - bits are uncertain, not flipped)
            auto applyBurst = [&](std::vector<float>& soft_bits, size_t burst_start) {
                for (size_t i = 0; i < test.burst_length && burst_start + i < soft_bits.size(); ++i) {
                    // Deep fade: LLR goes to 0 (erasure-like) plus some noise
                    soft_bits[burst_start + i] = 0.3f * noise(rng);
                }
            };

            // Test WITH interleaving
            {
                Bytes interleaved = interleaver.interleave(encoded);
                auto soft_bits = makeSoftBits(interleaved);

                // Apply burst at random position (simulates fade)
                if (test.burst_length > 0) {
                    size_t burst_pos = (frame * 37) % (soft_bits.size() - test.burst_length);
                    applyBurst(soft_bits, burst_pos);
                }

                // Deinterleave (spreads burst errors)
                auto deinterleaved = interleaver.deinterleave(std::span<const float>(soft_bits));
                Bytes decoded = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    if (decoded.size() >= tx_data.size()) {
                        bool match = true;
                        for (size_t i = 0; i < tx_data.size(); ++i) {
                            if (decoded[i] != tx_data[i]) { match = false; break; }
                        }
                        if (match) with_success++;
                    }
                }
            }

            // Test WITHOUT interleaving
            {
                auto soft_bits = makeSoftBits(encoded);

                // Apply burst at random position
                if (test.burst_length > 0) {
                    size_t burst_pos = (frame * 37) % (soft_bits.size() - test.burst_length);
                    applyBurst(soft_bits, burst_pos);
                }

                // Direct decode (no deinterleaving)
                Bytes decoded = decoder.decodeSoft(soft_bits);

                if (decoder.lastDecodeSuccess()) {
                    if (decoded.size() >= tx_data.size()) {
                        bool match = true;
                        for (size_t i = 0; i < tx_data.size(); ++i) {
                            if (decoded[i] != tx_data[i]) { match = false; break; }
                        }
                        if (match) without_success++;
                    }
                }
            }
        }

        float with_rate = 100.0f * with_success / NUM_FRAMES;
        float without_rate = 100.0f * without_success / NUM_FRAMES;
        float gain = with_rate - without_rate;

        std::cout << "   " << std::setw(20) << std::left << test.name
                  << std::setw(6) << std::right << with_success << "/" << NUM_FRAMES
                  << " (" << std::setw(5) << std::fixed << std::setprecision(1) << with_rate << "%)"
                  << std::setw(10) << without_success << "/" << NUM_FRAMES
                  << " (" << std::setw(5) << without_rate << "%)"
                  << std::setw(10) << std::showpos << gain << "%" << std::noshowpos << "\n";
    }

    std::cout << "\n   Analysis: Block interleaver with this LDPC code shows minimal benefit.\n";
    std::cout << "             LDPC's random parity structure already handles burst errors.\n";
    std::cout << "             Consider random interleaver or different block size.\n\n";
}

// ============================================================================
// FREQUENCY OFFSET CORRECTION TEST
// Test demodulator's ability to estimate and correct frequency offset
// Uses runE2ETest structure with frequency offset applied to channel
// ============================================================================

// Helper: Run E2E test with frequency offset applied
E2EResult runE2ETestWithOffset(
    const ModemConfig& config,
    Modulation mod,
    CodeRate rate,
    const WattersonChannel::Config& channel_cfg,
    float freq_offset_hz,
    size_t num_frames = 50
) {
    E2EResult result = {};
    result.frames_sent = num_frames;

    // CRITICAL: Set the modulation in config so demodulator knows what to expect!
    ModemConfig test_config = config;
    test_config.modulation = mod;

    OFDMModulator modulator(test_config);
    OFDMDemodulator demodulator(test_config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    // Interleaver must match LDPC block size: 648 bits = 24 x 27
    Interleaver interleaver(24, 27);

    WattersonChannel channel(channel_cfg);

    for (size_t frame = 0; frame < num_frames; ++frame) {
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }
        result.total_bits += tx_data.size() * 8;

        // TX chain
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        // Generate preamble + data
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, mod);

        // Combine preamble + data
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale to reasonable audio level
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        // Apply frequency offset using proper analytic signal approach
        // This correctly shifts the entire spectrum without creating images
        tx_audio = applyFrequencyShift(tx_audio, freq_offset_hz, config.sample_rate);

        // Channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // RX chain (feed in chunks)
        demodulator.reset();
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();
            if (soft_bits.size() >= LDPC_BLOCK_SIZE) {
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);
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
        }
    }

    result.frame_success_rate = 100.0f * result.frames_decoded / result.frames_sent;
    result.ber = result.total_bits > 0 ?
        static_cast<float>(result.bit_errors) / result.total_bits : 1.0f;

    return result;
}

void runFrequencyOffsetTest(const ModemConfig& config) {
    std::cout << "\n=== Frequency Offset Correction Test ===\n\n";
    std::cout << "Using analytic signal (Hilbert transform) for proper frequency shifting.\n";
    std::cout << "Testing estimation accuracy and decode tolerance.\n\n";

    auto channel_cfg = ccir::awgn(40.0f);  // High SNR to isolate frequency effects

    // =========================================================================
    // PART 1: Frequency Offset Estimation Accuracy
    // =========================================================================
    std::cout << "1. FREQUENCY OFFSET ESTIMATION ACCURACY\n";
    std::cout << "   Testing if demodulator can accurately estimate applied offset.\n\n";

    std::cout << "   Applied (Hz)   Estimated (Hz)   Error (Hz)   Status\n";
    std::cout << "   ────────────────────────────────────────────────────────\n";

    OFDMModulator modulator(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    Interleaver interleaver(24, 27);

    float test_offsets[] = {0.0f, 2.0f, 5.0f, -5.0f, 10.0f, -10.0f, 15.0f};
    bool estimation_ok = true;

    for (float applied_offset : test_offsets) {
        OFDMDemodulator demodulator(config);

        // Generate test frame
        Bytes tx_data(40);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>(i * 7 + 13);
        }

        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, Modulation::QPSK);

        Samples tx_audio;
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        // Scale signal
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            for (float& s : tx_audio) s *= 0.5f / max_val;
        }

        // Apply frequency offset using proper analytic signal method
        Samples shifted = applyFrequencyShift(tx_audio, applied_offset, config.sample_rate);

        // Pass through AWGN channel
        WattersonChannel channel(channel_cfg);
        SampleSpan tx_span(shifted.data(), shifted.size());
        Samples rx_audio = channel.process(tx_span);

        // Process through demodulator
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size(); offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            demodulator.process(rx_span);
        }

        float estimated = demodulator.getFrequencyOffset();
        float error = estimated - applied_offset;
        bool ok = std::abs(error) < 3.0f;  // Allow 3 Hz error margin
        if (!ok) estimation_ok = false;

        std::cout << "   " << std::setw(10) << std::fixed << std::setprecision(1) << applied_offset
                  << std::setw(16) << estimated
                  << std::setw(13) << std::showpos << error << std::noshowpos
                  << "        " << (ok ? "OK" : "FAIL") << "\n";
    }

    std::cout << "\n   Estimation accuracy: " << (estimation_ok ? "PASS" : "NEEDS WORK") << "\n\n";

    // =========================================================================
    // PART 2: Decode Tolerance vs Frequency Offset
    // =========================================================================
    std::cout << "2. DECODE TOLERANCE VS FREQUENCY OFFSET\n";
    std::cout << "   Testing how much offset the modem can tolerate and still decode.\n\n";

    std::cout << "   Offset (Hz)    Sync    Decode    Success Rate\n";
    std::cout << "   ────────────────────────────────────────────────\n";

    const size_t NUM_FRAMES = 30;  // Enough for statistical significance
    float tolerance_offsets[] = {0.0f, 2.0f, 5.0f, 8.0f, 10.0f, 12.0f, 15.0f, 20.0f};

    float max_tolerable_offset = 0.0f;

    for (float offset : tolerance_offsets) {
        E2EResult result = runE2ETestWithOffset(config, Modulation::QPSK, CodeRate::R1_2,
                                                 channel_cfg, offset, NUM_FRAMES);

        // Count sync successes (frames_decoded > 0 implies sync worked at some point)
        std::string sync_status = result.frames_decoded > 0 ? "YES" : "NO ";
        std::string decode_status = result.frame_success_rate > 80.0f ? "GOOD" :
                                    result.frame_success_rate > 50.0f ? "FAIR" :
                                    result.frame_success_rate > 0.0f  ? "POOR" : "FAIL";

        if (result.frame_success_rate > 80.0f) {
            max_tolerable_offset = offset;
        }

        std::cout << "   " << std::setw(8) << std::fixed << std::setprecision(1) << offset
                  << std::setw(10) << sync_status
                  << std::setw(10) << decode_status
                  << std::setw(12) << result.frame_success_rate << "%\n";
    }

    std::cout << "\n   Maximum tolerable offset (>80% decode): ±"
              << max_tolerable_offset << " Hz\n\n";

    // =========================================================================
    // PART 3: Summary and Assessment
    // =========================================================================
    std::cout << "3. SUMMARY\n";
    std::cout << "   ─────────────────────────────────────────────────────────────\n";
    std::cout << "   Estimation accuracy:     " << (estimation_ok ? "PASS" : "NEEDS WORK") << "\n";
    std::cout << "   Frequency tolerance:     ±" << max_tolerable_offset << " Hz\n";
    std::cout << "   Target tolerance:        ±20 Hz (typical HF radio drift)\n";
    std::cout << "\n";

    if (max_tolerable_offset >= 15.0f) {
        std::cout << "   Assessment: EXCELLENT - Meets HF requirements\n";
    } else if (max_tolerable_offset >= 10.0f) {
        std::cout << "   Assessment: GOOD - Adequate for most HF applications\n";
    } else if (max_tolerable_offset >= 5.0f) {
        std::cout << "   Assessment: FAIR - May need radio AFC or better correction\n";
    } else {
        std::cout << "   Assessment: POOR - Frequency correction needs improvement\n";
    }
    std::cout << "\n";
}

int main(int, char*[]) {
    printHeader();

    ModemConfig config;
    // Enable adaptive RLS equalizer for better fading channel performance
    config.adaptive_eq_enabled = true;
    config.adaptive_eq_use_rls = true;  // RLS tracks faster than LMS
    config.rls_lambda = 0.95f;          // Faster adaptation (lower = faster)

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
    std::cout << "  Adaptive EQ:  " << (config.adaptive_eq_enabled ? (config.adaptive_eq_use_rls ? "ENABLED (RLS)" : "ENABLED (LMS)") : "disabled") << "\n";
    std::cout << "\n";

    // Run the full test suite
    runRealisticTests(config);
    runConfigurationComparisonTest();
    runAdaptiveModulationTest();
    runSNREstimationTest(config);
    runInterleavingBenefitTest(config);
    runFrequencyOffsetTest(config);
    runSpeedProfileTest(config);

    return 0;
}
