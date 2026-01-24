/**
 * OTFS vs OFDM Performance Comparison
 *
 * Tests both waveforms under realistic HF channel conditions:
 * - AWGN (baseline)
 * - ITU-R F.1487 Good (0.5ms delay, 0.1 Hz Doppler)
 * - ITU-R F.1487 Moderate (1.0ms delay, 0.5 Hz Doppler)
 * - ITU-R F.1487 Poor (2.0ms delay, 1.0 Hz Doppler)
 * - ITU-R F.1487 Flutter (0.5ms delay, 10 Hz Doppler - auroral/polar)
 *
 * OTFS is designed for doubly-selective channels (time + frequency varying)
 * and should outperform OFDM in Poor and Flutter conditions.
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/otfs.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <cstring>
#include <span>

using namespace ultra;
using namespace ultra::sim;

// Get info bytes for each code rate (from LDPC 648-bit codeword)
size_t getInfoBytes(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 162 / 8;   // 20 bytes
        case CodeRate::R1_2: return 324 / 8;   // 40 bytes
        case CodeRate::R2_3: return 432 / 8;   // 54 bytes
        case CodeRate::R3_4: return 486 / 8;   // 60 bytes
        case CodeRate::R5_6: return 540 / 8;   // 67 bytes
        default: return 40;
    }
}

struct TestResult {
    int success;
    int trials;
    float ber;  // Bit error rate before FEC
};

// Add AWGN noise to signal (for baseline comparison)
void addNoise(Samples& signal, float snr_db, std::mt19937& rng) {
    float signal_power = 0;
    for (float s : signal) signal_power += s * s;
    signal_power /= signal.size();

    float noise_std = std::sqrt(signal_power / std::pow(10.0f, snr_db / 10.0f));
    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : signal) s += noise(rng);
}

// Channel condition enum for cleaner API
enum class ChannelCondition { AWGN, Good, Moderate, Poor, Flutter };

const char* getConditionName(ChannelCondition cond) {
    switch (cond) {
        case ChannelCondition::AWGN: return "AWGN";
        case ChannelCondition::Good: return "Good";
        case ChannelCondition::Moderate: return "Moderate";
        case ChannelCondition::Poor: return "Poor";
        case ChannelCondition::Flutter: return "Flutter";
        default: return "?";
    }
}

WattersonChannel::Config getChannelConfig(ChannelCondition cond, float snr_db) {
    switch (cond) {
        case ChannelCondition::Good: return itu_r_f1487::good(snr_db);
        case ChannelCondition::Moderate: return itu_r_f1487::moderate(snr_db);
        case ChannelCondition::Poor: return itu_r_f1487::poor(snr_db);
        case ChannelCondition::Flutter: return itu_r_f1487::flutter(snr_db);
        default: return itu_r_f1487::awgn(snr_db);
    }
}

// Normalize signal to 0.5 peak
void normalize(Samples& signal) {
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : signal) s *= 0.5f / max_val;
    }
}

// Test OFDM at given SNR with optional HF channel
TestResult test_ofdm(float snr_db, Modulation mod, CodeRate rate, int trials,
                     ChannelCondition channel = ChannelCondition::AWGN) {
    std::mt19937 rng(12345);

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = mod;
    config.code_rate = rate;
    config.use_pilots = (mod != Modulation::DQPSK && mod != Modulation::D8PSK);

    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    size_t data_bytes = getInfoBytes(rate);
    TestResult result{0, trials, 0.0f};

    // Create HF channel if not AWGN
    auto ch_config = getChannelConfig(channel, snr_db);
    WattersonChannel hf_channel(ch_config, 12345);

    for (int t = 0; t < trials; t++) {
        OFDMDemodulator demod(config);
        hf_channel.reset();

        // Generate random data
        Bytes data(data_bytes);
        for (auto& b : data) b = rng() & 0xFF;

        // Encode and modulate
        Bytes encoded = encoder.encode(data);
        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(encoded, mod);

        // Build signal
        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        normalize(signal);

        // Apply channel
        if (channel == ChannelCondition::AWGN) {
            addNoise(signal, snr_db, rng);
        } else {
            SampleSpan span(signal.data(), signal.size());
            signal = hf_channel.process(span);
        }

        // Demodulate
        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            demod.process(span);
        }

        auto soft = demod.getSoftBits();
        if (soft.size() >= 648) {
            std::span<const float> llrs(soft.data(), 648);
            Bytes decoded = decoder.decodeSoft(llrs);

            bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
            if (match) {
                for (size_t i = 0; i < data.size(); i++) {
                    if (decoded[i] != data[i]) { match = false; break; }
                }
            }
            if (match) result.success++;
        }
    }

    return result;
}

// Test OTFS at given SNR with optional HF channel
TestResult test_otfs(float snr_db, Modulation mod, CodeRate rate, int trials,
                     ChannelCondition channel = ChannelCondition::AWGN) {
    std::mt19937 rng(12345);

    OTFSConfig config;
    config.M = 32;              // Delay bins
    config.N = 16;              // Doppler bins
    config.fft_size = 512;
    config.cp_length = 64;
    config.sample_rate = 48000;
    config.center_freq = 1500.0f;
    config.modulation = mod;
    config.tf_equalization = true;

    OTFSModulator modulator(config);
    OTFSDemodulator demodulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    size_t data_bytes = getInfoBytes(rate);
    TestResult result{0, trials, 0.0f};

    // Create HF channel if not AWGN
    auto ch_config = getChannelConfig(channel, snr_db);
    WattersonChannel hf_channel(ch_config, 12345);

    for (int t = 0; t < trials; t++) {
        demodulator.reset();
        hf_channel.reset();

        // Generate random data
        Bytes data(data_bytes);
        for (auto& b : data) b = rng() & 0xFF;

        // Encode
        Bytes encoded = encoder.encode(data);

        // Map to DD grid and modulate
        auto dd_symbols = modulator.mapToDD(encoded, mod);
        auto preamble = modulator.generatePreamble();
        auto modulated = modulator.modulate(dd_symbols, mod);

        // Build signal
        Samples signal;
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), modulated.begin(), modulated.end());

        normalize(signal);

        // Apply channel
        if (channel == ChannelCondition::AWGN) {
            addNoise(signal, snr_db, rng);
        } else {
            SampleSpan span(signal.data(), signal.size());
            signal = hf_channel.process(span);
        }

        // Demodulate
        bool frame_ready = false;
        for (size_t i = 0; i < signal.size(); i += 960) {
            size_t len = std::min((size_t)960, signal.size() - i);
            SampleSpan span(signal.data() + i, len);
            if (demodulator.process(span)) {
                frame_ready = true;
            }
        }

        if (frame_ready) {
            auto soft = demodulator.getSoftBits();
            if (soft.size() >= 648) {
                std::span<const float> llrs(soft.data(), 648);
                Bytes decoded = decoder.decodeSoft(llrs);

                bool match = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
                if (match) {
                    for (size_t i = 0; i < data.size(); i++) {
                        if (decoded[i] != data[i]) { match = false; break; }
                    }
                }
                if (match) result.success++;
            }
        }
    }

    return result;
}

void print_result(const char* name, TestResult r) {
    int pct = r.trials > 0 ? (r.success * 100 / r.trials) : 0;
    printf("%s: %3d%% (%d/%d)", name, pct, r.success, r.trials);
}

int main(int argc, char* argv[]) {
    setLogLevel(LogLevel::WARN);

    int trials = 20;
    float snr = 20.0f;
    bool awgn_only = false;
    bool verbose = false;

    // Parse args
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--trials") == 0 && i+1 < argc) {
            trials = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--snr") == 0 && i+1 < argc) {
            snr = atof(argv[++i]);
        } else if (strcmp(argv[i], "--awgn") == 0) {
            awgn_only = true;
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("OTFS vs OFDM HF Channel Comparison\n\n");
            printf("Tests both waveforms on ITU-R F.1487 Watterson channel model\n\n");
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --snr <dB>     SNR level (default: 20)\n");
            printf("  --trials <n>   Trials per test (default: 20)\n");
            printf("  --awgn         AWGN only (skip HF channel tests)\n");
            printf("  -v             Verbose output\n");
            return 0;
        }
    }

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║       OTFS vs OFDM - ITU-R F.1487 HF Channel Comparison      ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    printf("Configuration:\n");
    printf("  SNR:     %.0f dB\n", snr);
    printf("  Trials:  %d per test\n\n", trials);

    // Channel conditions to test
    std::vector<ChannelCondition> conditions;
    if (awgn_only) {
        conditions = {ChannelCondition::AWGN};
    } else {
        conditions = {
            ChannelCondition::AWGN,
            ChannelCondition::Good,
            ChannelCondition::Moderate,
            ChannelCondition::Poor,
            ChannelCondition::Flutter
        };
    }

    // Print channel condition details
    if (!awgn_only) {
        printf("ITU-R F.1487 Channel Conditions:\n");
        printf("  AWGN:     No fading, no multipath (baseline)\n");
        printf("  Good:     0.5ms delay, 0.1 Hz Doppler (quiet mid-latitude)\n");
        printf("  Moderate: 1.0ms delay, 0.5 Hz Doppler (typical)\n");
        printf("  Poor:     2.0ms delay, 1.0 Hz Doppler (disturbed)\n");
        printf("  Flutter:  0.5ms delay, 10 Hz Doppler (auroral/polar)\n\n");
    }

    // Test QPSK R1/2 across all channel conditions
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("                    QPSK R1/2 (2 bits/symbol)\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    printf("Channel      OFDM         OTFS         Winner\n");
    printf("───────      ────         ────         ──────\n");

    int ofdm_wins = 0, otfs_wins = 0, ties = 0;

    for (auto cond : conditions) {
        auto ofdm_res = test_ofdm(snr, Modulation::QPSK, CodeRate::R1_2, trials, cond);
        auto otfs_res = test_otfs(snr, Modulation::QPSK, CodeRate::R1_2, trials, cond);

        int ofdm_pct = ofdm_res.success * 100 / trials;
        int otfs_pct = otfs_res.success * 100 / trials;

        const char* winner = "TIE";
        if (otfs_pct > ofdm_pct + 5) { winner = "OTFS"; otfs_wins++; }
        else if (ofdm_pct > otfs_pct + 5) { winner = "OFDM"; ofdm_wins++; }
        else { ties++; }

        printf("%-10s   %3d%%         %3d%%         %s\n",
               getConditionName(cond), ofdm_pct, otfs_pct, winner);
    }

    // Test DQPSK (OFDM) vs QPSK (OTFS) - differential vs coherent
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("          DQPSK R1/2 (OFDM) vs QPSK R1/2 (OTFS)\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    printf("Channel      OFDM-DQPSK   OTFS-QPSK    Winner\n");
    printf("───────      ──────────   ─────────    ──────\n");

    for (auto cond : conditions) {
        auto ofdm_res = test_ofdm(snr, Modulation::DQPSK, CodeRate::R1_2, trials, cond);
        auto otfs_res = test_otfs(snr, Modulation::QPSK, CodeRate::R1_2, trials, cond);

        int ofdm_pct = ofdm_res.success * 100 / trials;
        int otfs_pct = otfs_res.success * 100 / trials;

        const char* winner = "TIE";
        if (otfs_pct > ofdm_pct + 5) { winner = "OTFS"; otfs_wins++; }
        else if (ofdm_pct > otfs_pct + 5) { winner = "OFDM"; ofdm_wins++; }
        else { ties++; }

        printf("%-10s   %3d%%         %3d%%         %s\n",
               getConditionName(cond), ofdm_pct, otfs_pct, winner);
    }

    // Test 16QAM R1/2 (high throughput)
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("                   16QAM R1/2 (4 bits/symbol)\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");
    printf("Channel      OFDM         OTFS         Winner\n");
    printf("───────      ────         ────         ──────\n");

    for (auto cond : conditions) {
        auto ofdm_res = test_ofdm(snr, Modulation::QAM16, CodeRate::R1_2, trials, cond);
        auto otfs_res = test_otfs(snr, Modulation::QAM16, CodeRate::R1_2, trials, cond);

        int ofdm_pct = ofdm_res.success * 100 / trials;
        int otfs_pct = otfs_res.success * 100 / trials;

        const char* winner = "TIE";
        if (otfs_pct > ofdm_pct + 5) { winner = "OTFS"; otfs_wins++; }
        else if (ofdm_pct > otfs_pct + 5) { winner = "OFDM"; ofdm_wins++; }
        else { ties++; }

        printf("%-10s   %3d%%         %3d%%         %s\n",
               getConditionName(cond), ofdm_pct, otfs_pct, winner);
    }

    // Summary
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("                          SUMMARY\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    printf("  OTFS wins:  %d\n", otfs_wins);
    printf("  OFDM wins:  %d\n", ofdm_wins);
    printf("  Ties:       %d\n\n", ties);

    if (otfs_wins > ofdm_wins) {
        printf("  \033[32m>>> OTFS outperforms OFDM on HF channels! <<<\033[0m\n");
    } else if (ofdm_wins > otfs_wins) {
        printf("  \033[33m>>> OFDM performs better (unexpected) <<<\033[0m\n");
    } else {
        printf("  \033[33m>>> Mixed results - similar performance <<<\033[0m\n");
    }

    printf("\n");
    printf("Expected behavior:\n");
    printf("  - AWGN: Similar performance (both near-optimal)\n");
    printf("  - Good/Moderate: OTFS slightly better (diversity gain)\n");
    printf("  - Poor/Flutter: OTFS significantly better (time-freq diversity)\n");
    printf("\n");

    return 0;
}
