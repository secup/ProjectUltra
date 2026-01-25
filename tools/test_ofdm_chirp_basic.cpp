// test_ofdm_chirp_basic.cpp - Direct OFDM_CHIRP loopback test
// Tests: chirp sync + OFDM DQPSK without ModemEngine complexity

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "sync/chirp_sync.hpp"
#include "protocol/frame_v2.hpp"
#include <cstdio>
#include <cmath>
#include <random>

using namespace ultra;
namespace v2 = protocol::v2;

// Add AWGN noise - standard version
void addNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
    float signal_power = 0.0f;
    for (float s : samples) signal_power += s * s;
    signal_power /= samples.size();

    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) s += noise(rng);
}

// Add AWGN noise to a subsection of a larger buffer, calibrated to that section's power
void addNoiseToSection(std::vector<float>& buffer, size_t start, size_t len, float snr_db, std::mt19937& rng) {
    // Compute signal power over the section
    float signal_power = 0.0f;
    for (size_t i = 0; i < len; i++) {
        float s = buffer[start + i];
        signal_power += s * s;
    }
    signal_power /= len;

    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    // Add noise to entire buffer with this calibrated noise level
    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : buffer) s += noise(rng);
}

int main(int argc, char** argv) {
    float snr_db = 20.0f;
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--snr" && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (std::string(argv[i]) == "-v") {
            verbose = true;
        }
    }

    printf("=== OFDM_CHIRP Basic Loopback Test ===\n");
    printf("SNR: %.1f dB\n\n", snr_db);

    // Config: DQPSK, no pilots (30 data carriers)
    ModemConfig config;
    config.sample_rate = 48000;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.center_freq = 1500;
    config.modulation = Modulation::DQPSK;
    config.code_rate = CodeRate::R1_4;
    config.use_pilots = false;  // DQPSK doesn't need pilots

    // Create components
    OFDMModulator modulator(config);
    OFDMDemodulator demodulator(config);
    LDPCEncoder encoder(CodeRate::R1_4);
    LDPCDecoder decoder(CodeRate::R1_4);

    sync::ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = 48000;
    chirp_cfg.duration_ms = 500;
    chirp_cfg.f_start = 300.0f;
    chirp_cfg.f_end = 2700.0f;
    chirp_cfg.repetitions = 1;  // Single chirp for OFDM_CHIRP
    chirp_cfg.gap_ms = 0;       // No gap after chirp - training follows immediately
    sync::ChirpSync chirp_sync(chirp_cfg);

    std::mt19937 rng(42);

    // Create test data for R1/4: info_bits=162, so 20 bytes (160 bits < 162)
    // This fits in 1 codeword with padding. Output: 648 bits = 81 bytes
    Bytes test_data(20);
    for (size_t i = 0; i < test_data.size(); i++) {
        test_data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    // === TX ===
    printf("TX: Encoding %zu bytes...\n", test_data.size());
    Bytes encoded = encoder.encode(test_data);
    printf("TX: Encoded to %zu bytes (648 bits)\n", encoded.size());

    // Generate chirp preamble
    auto chirp = chirp_sync.generate();
    printf("TX: Chirp preamble: %zu samples (%.1f ms)\n",
           chirp.size(), chirp.size() * 1000.0f / 48000);

    // Generate training symbols (resets mixer, sets DBPSK state to (1,0))
    auto training = modulator.generateTrainingSymbols(2);
    printf("TX: Training symbols: %zu samples (%d symbols)\n", training.size(), 2);

    // Modulate with DQPSK
    auto data_samples = modulator.modulate(encoded, Modulation::DQPSK);
    printf("TX: Data samples: %zu samples\n", data_samples.size());

    // Combine: [lead-in][chirp][training][data][tail]
    bool include_chirp = true;  // Enable chirp for testing
    size_t lead_in = include_chirp ? 4800 : 0;  // 100ms lead-in only with chirp
    size_t chirp_len = include_chirp ? chirp.size() : 0;
    size_t tail = 0;

    std::vector<float> tx_signal(lead_in + chirp_len + training.size() + data_samples.size() + tail, 0.0f);

    size_t pos = lead_in;
    if (include_chirp) {
        std::copy(chirp.begin(), chirp.end(), tx_signal.begin() + pos);
        pos += chirp.size();
    }
    std::copy(training.begin(), training.end(), tx_signal.begin() + pos);
    pos += training.size();
    std::copy(data_samples.begin(), data_samples.end(), tx_signal.begin() + pos);

    printf("TX: Total signal: %zu samples (%.1f ms)\n",
           tx_signal.size(), tx_signal.size() * 1000.0f / 48000);

    // === Channel ===
    // Add noise calibrated to OFDM signal power (not chirp power)
    // This gives correct SNR for the data portion we care about
    size_t ofdm_start = lead_in + chirp_len;
    size_t ofdm_len = training.size() + data_samples.size();
    addNoiseToSection(tx_signal, ofdm_start, ofdm_len, snr_db, rng);
    printf("Channel: Added AWGN at %.1f dB SNR\n\n", snr_db);

    // === RX ===
    size_t training_start;

    if (include_chirp) {
        printf("RX: Searching for chirp...\n");

        float chirp_corr = 0.0f;
        SampleSpan search_span(tx_signal.data(), tx_signal.size());
        int chirp_start = chirp_sync.detect(search_span, chirp_corr, 0.35f);

        if (chirp_start < 0) {
            printf("RX: FAILED - Chirp not detected!\n");
            return 1;
        }

        size_t chirp_end = chirp_start + chirp_sync.getTotalSamples();
        printf("RX: Chirp found at %d (corr=%.3f), training starts at %zu\n",
               chirp_start, chirp_corr, chirp_end);

        // Expected positions
        size_t expected_chirp_start = lead_in;
        size_t expected_training_start = lead_in + chirp.size();
        printf("RX: Expected chirp at %zu, training at %zu\n",
               expected_chirp_start, expected_training_start);

        training_start = expected_training_start;  // Use known position
    } else {
        printf("RX: No chirp mode - training at sample 0\n");
        training_start = 0;  // No chirp, training starts at beginning
    }

    // Pass samples starting at training symbols to processPresynced
    size_t remaining = tx_signal.size() - training_start;
    printf("RX: Passing %zu samples to processPresynced (training + data)\n", remaining);

    SampleSpan rx_span(tx_signal.data() + training_start, remaining);
    bool frame_ready = demodulator.processPresynced(rx_span, 2);

    printf("RX: processPresynced returned frame_ready=%d\n", frame_ready);

    if (!frame_ready) {
        printf("RX: FAILED - Not enough soft bits!\n");
        return 1;
    }

    // Get all soft bits
    std::vector<float> all_soft_bits;
    while (true) {
        auto bits = demodulator.getSoftBits();
        if (bits.empty()) break;
        all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
    }

    printf("RX: Got %zu soft bits (%.1f codewords)\n",
           all_soft_bits.size(), all_soft_bits.size() / 648.0f);

    if (all_soft_bits.size() < 648) {
        printf("RX: FAILED - Need at least 648 bits!\n");
        return 1;
    }

    // Show first few soft bits
    if (verbose) {
        printf("RX: First 16 soft bits: ");
        for (int i = 0; i < 16 && i < (int)all_soft_bits.size(); i++) {
            printf("%.1f ", all_soft_bits[i]);
        }
        printf("\n");
    }

    // Decode
    std::vector<float> cw_bits(all_soft_bits.begin(), all_soft_bits.begin() + 648);
    Bytes decoded = decoder.decodeSoft(cw_bits);

    if (!decoder.lastDecodeSuccess()) {
        printf("RX: LDPC FAILED!\n");

        // Show soft bit statistics
        float sum = 0, min_val = cw_bits[0], max_val = cw_bits[0];
        int positive = 0, negative = 0;
        for (float v : cw_bits) {
            sum += std::abs(v);
            if (v < min_val) min_val = v;
            if (v > max_val) max_val = v;
            if (v > 0) positive++; else negative++;
        }
        printf("RX: Soft bit stats: avg_abs=%.2f, min=%.2f, max=%.2f, +:%d -:%d\n",
               sum / cw_bits.size(), min_val, max_val, positive, negative);
        return 1;
    }

    printf("RX: LDPC decode SUCCESS!\n");

    // Verify data
    bool match = (decoded.size() >= test_data.size());
    for (size_t i = 0; i < test_data.size() && match; i++) {
        if (decoded[i] != test_data[i]) match = false;
    }

    if (match) {
        printf("\n=== SUCCESS: Data matches! ===\n");
        return 0;
    } else {
        printf("\nRX: Data MISMATCH!\n");
        printf("Expected: ");
        for (int i = 0; i < 8; i++) printf("%02x ", test_data[i]);
        printf("...\n");
        printf("Got:      ");
        for (int i = 0; i < 8 && i < (int)decoded.size(); i++) printf("%02x ", decoded[i]);
        printf("...\n");
        return 1;
    }
}
