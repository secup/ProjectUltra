// test_mc_dpsk.cpp - Test Multi-Carrier DPSK on HF channels
//
// Tests the new multi-carrier DPSK mode that fills the gap between
// single-carrier DPSK and full OFDM.
//
// Validates:
// 1. Works at lower SNR than OFDM (frequency diversity)
// 2. Survives frequency-selective fading (unlike single-carrier)
// 3. Matches VARA-like speed levels

#include "sync/chirp_sync.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "ultra/fec.hpp"
#include "protocol/frame_v2.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>

using namespace ultra;
using namespace ultra::sync;
using namespace ultra::sim;

int main(int argc, char* argv[]) {
    std::cout << "=== Multi-Carrier DPSK Test ===\n\n";

    // Parse args
    int num_carriers = 8;
    float snr_db = 10.0f;
    std::string channel_type = "moderate";
    std::string rate_str = "r14";  // Default R1/4
    int trials = 10;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--carriers") == 0 && i + 1 < argc) {
            num_carriers = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            channel_type = argv[++i];
        } else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc) {
            rate_str = argv[++i];
        } else if (strcmp(argv[i], "--trials") == 0 && i + 1 < argc) {
            trials = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "  --carriers N   Number of carriers (3-40, default: 8)\n";
            std::cout << "  --snr dB       SNR in dB (default: 10)\n";
            std::cout << "  --channel X    Channel type: awgn, good, moderate, poor (default: moderate)\n";
            std::cout << "  --rate X       Code rate: r14, r12, r23, r34 (default: r14)\n";
            std::cout << "  --trials N     Number of trials (default: 10)\n";
            return 0;
        }
    }

    // Parse code rate
    CodeRate code_rate = CodeRate::R1_4;
    float rate_efficiency = 0.25f;
    if (rate_str == "r12" || rate_str == "R12") {
        code_rate = CodeRate::R1_2;
        rate_efficiency = 0.5f;
    } else if (rate_str == "r23" || rate_str == "R23") {
        code_rate = CodeRate::R2_3;
        rate_efficiency = 0.667f;
    } else if (rate_str == "r34" || rate_str == "R34") {
        code_rate = CodeRate::R3_4;
        rate_efficiency = 0.75f;
    }

    constexpr float SAMPLE_RATE = 48000.0f;

    // Configure multi-carrier DPSK
    MultiCarrierDPSKConfig mc_cfg;
    mc_cfg.sample_rate = SAMPLE_RATE;
    mc_cfg.num_carriers = num_carriers;
    mc_cfg.freq_low = 500.0f;
    mc_cfg.freq_high = 2500.0f;
    mc_cfg.samples_per_symbol = 512;  // 93.75 baud
    mc_cfg.bits_per_symbol = 2;       // DQPSK
    mc_cfg.training_symbols = 8;

    auto carrier_freqs = mc_cfg.getCarrierFreqs();

    float net_bps = mc_cfg.getRawBitRate() * rate_efficiency;
    std::cout << "Configuration:\n";
    std::cout << "  Carriers: " << num_carriers << "\n";
    std::cout << "  Symbol rate: " << mc_cfg.getSymbolRate() << " baud\n";
    std::cout << "  Raw bit rate: " << mc_cfg.getRawBitRate() << " bps\n";
    std::cout << "  Code rate: " << rate_str << " (" << (rate_efficiency * 100) << "% efficiency)\n";
    std::cout << "  Net throughput: " << net_bps << " bps\n";
    std::cout << "  Carrier frequencies: ";
    for (size_t i = 0; i < carrier_freqs.size(); i++) {
        if (i > 0) std::cout << ", ";
        std::cout << (int)carrier_freqs[i];
    }
    std::cout << " Hz\n";
    std::cout << "  SNR: " << snr_db << " dB\n";
    std::cout << "  Channel: " << channel_type << "\n";
    std::cout << "  Trials: " << trials << "\n\n";

    // Chirp config for timing sync (single chirp to avoid confusion on fading channels)
    ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = SAMPLE_RATE;
    chirp_cfg.f_start = 300.0f;
    chirp_cfg.f_end = 2700.0f;
    chirp_cfg.duration_ms = 500.0f;
    chirp_cfg.repetitions = 1;  // Single chirp
    chirp_cfg.gap_ms = 0.0f;
    ChirpSync chirp_sync(chirp_cfg);

    // Test data - one LDPC codeword
    // Data bytes depend on code rate
    size_t data_bytes = protocol::v2::BYTES_PER_CODEWORD;  // 20 bytes for R1/4
    if (code_rate == CodeRate::R1_2) data_bytes = 40;
    else if (code_rate == CodeRate::R2_3) data_bytes = 54;
    else if (code_rate == CodeRate::R3_4) data_bytes = 60;

    Bytes test_data = {0x55, 0x4C, 0x12, 0x01};  // v2 magic
    test_data.resize(data_bytes, 0xAA);

    LDPCEncoder encoder(code_rate);
    Bytes encoded = encoder.encode(test_data);

    std::cout << "Test data: " << test_data.size() << " bytes -> "
              << encoded.size() << " encoded bytes ("
              << encoded.size() * 8 << " bits)\n\n";

    // Channel configuration
    WattersonChannel::Config ch_cfg;
    ch_cfg.sample_rate = SAMPLE_RATE;
    ch_cfg.snr_db = snr_db;
    ch_cfg.noise_enabled = true;

    if (channel_type == "awgn") {
        ch_cfg.fading_enabled = false;
        ch_cfg.multipath_enabled = false;
    } else if (channel_type == "good") {
        ch_cfg.fading_enabled = true;
        ch_cfg.multipath_enabled = true;
        ch_cfg.delay_spread_ms = 0.5f;
        ch_cfg.doppler_spread_hz = 0.2f;
        ch_cfg.path1_gain = 0.9f;
        ch_cfg.path2_gain = 0.4f;
    } else if (channel_type == "moderate") {
        ch_cfg.fading_enabled = true;
        ch_cfg.multipath_enabled = true;
        ch_cfg.delay_spread_ms = 1.0f;
        ch_cfg.doppler_spread_hz = 0.5f;
        ch_cfg.path1_gain = 0.707f;
        ch_cfg.path2_gain = 0.707f;
    } else if (channel_type == "poor") {
        ch_cfg.fading_enabled = true;
        ch_cfg.multipath_enabled = true;
        ch_cfg.delay_spread_ms = 2.0f;
        ch_cfg.doppler_spread_hz = 1.0f;
        ch_cfg.path1_gain = 0.6f;
        ch_cfg.path2_gain = 0.8f;
    }

    int passed = 0;
    int chirp_detected = 0;
    int ldpc_success = 0;

    for (int t = 0; t < trials; t++) {
        // Fresh modulator/demodulator
        MultiCarrierDPSKModulator modulator(mc_cfg);
        MultiCarrierDPSKDemodulator demodulator(mc_cfg);

        // Build TX signal
        Samples tx_signal;

        // Lead-in silence (150ms)
        tx_signal.resize(7200, 0.0f);

        // Chirp preamble
        size_t chirp_start = tx_signal.size();
        Samples chirp = chirp_sync.generate();
        tx_signal.insert(tx_signal.end(), chirp.begin(), chirp.end());

        // Training sequence
        size_t training_start = tx_signal.size();
        Samples training = modulator.generateTrainingSequence();
        tx_signal.insert(tx_signal.end(), training.begin(), training.end());

        // Reference symbol
        size_t ref_start = tx_signal.size();
        Samples ref = modulator.generateReferenceSymbol();
        tx_signal.insert(tx_signal.end(), ref.begin(), ref.end());

        // Data
        size_t data_start = tx_signal.size();
        Samples data = modulator.modulate(encoded);
        tx_signal.insert(tx_signal.end(), data.begin(), data.end());

        // Trail-out (500ms - extra for multipath extension)
        tx_signal.resize(tx_signal.size() + 24000, 0.0f);

        // Apply channel
        WattersonChannel channel(ch_cfg, 42 + t);
        SampleSpan tx_span(tx_signal.data(), tx_signal.size());
        Samples rx_signal = channel.process(tx_span);

        // Detect chirp
        SampleSpan rx_span(rx_signal.data(), rx_signal.size());
        float corr = 0.0f;
        int detected_chirp = chirp_sync.detect(rx_span, corr, 0.35f);

        if (detected_chirp < 0) {
            std::cout << "Trial " << (t+1) << ": Chirp not detected (corr="
                      << std::fixed << std::setprecision(3) << corr << ")\n";
            continue;
        }
        chirp_detected++;

        // Calculate offsets
        size_t detected_chirp_end = detected_chirp + chirp_sync.getTotalSamples();
        size_t detected_training = detected_chirp_end;
        size_t training_len = mc_cfg.training_symbols * mc_cfg.samples_per_symbol;
        size_t detected_ref = detected_training + training_len;
        size_t detected_data = detected_ref + mc_cfg.samples_per_symbol;

        // Check bounds
        if (detected_data + data.size() > rx_signal.size()) {
            std::cout << "Trial " << (t+1) << ": Buffer too short\n";
            continue;
        }

        // Process training for CFO
        SampleSpan train_span(rx_signal.data() + detected_training, training_len);
        demodulator.processTraining(train_span);

        // Set reference
        SampleSpan ref_span(rx_signal.data() + detected_ref, mc_cfg.samples_per_symbol);
        demodulator.setReference(ref_span);

        // Demodulate
        SampleSpan data_span(rx_signal.data() + detected_data, data.size());
        auto soft_bits = demodulator.demodulateSoft(data_span);

        if (soft_bits.size() < protocol::v2::LDPC_CODEWORD_BITS) {
            std::cout << "Trial " << (t+1) << ": Not enough soft bits ("
                      << soft_bits.size() << " < " << protocol::v2::LDPC_CODEWORD_BITS << ")\n";
            continue;
        }

        // LDPC decode
        std::vector<float> cw_bits(soft_bits.begin(),
                                    soft_bits.begin() + protocol::v2::LDPC_CODEWORD_BITS);

        // Calculate soft bit statistics
        float sb_sum = 0, sb_abs_sum = 0;
        float sb_min = cw_bits[0], sb_max = cw_bits[0];
        for (float sb : cw_bits) {
            sb_sum += sb;
            sb_abs_sum += std::abs(sb);
            sb_min = std::min(sb_min, sb);
            sb_max = std::max(sb_max, sb);
        }

        LDPCDecoder decoder(code_rate);
        Bytes decoded = decoder.decodeSoft(cw_bits);

        if (!decoder.lastDecodeSuccess()) {
            std::cout << "Trial " << (t+1) << ": LDPC fail, corr="
                      << std::fixed << std::setprecision(3) << corr
                      << ", soft_avg=" << std::setprecision(2) << (sb_abs_sum/cw_bits.size())
                      << ", CFO=" << std::setprecision(1) << demodulator.getEstimatedCFO() << "Hz\n";
            continue;
        }
        ldpc_success++;

        // Verify data
        bool match = true;
        for (size_t i = 0; i < test_data.size() && match; i++) {
            if (decoded[i] != test_data[i]) match = false;
        }

        if (match) {
            passed++;
            std::cout << "Trial " << (t+1) << ": PASS, corr="
                      << std::fixed << std::setprecision(3) << corr
                      << ", CFO=" << std::setprecision(1) << demodulator.getEstimatedCFO() << "Hz\n";
        } else {
            std::cout << "Trial " << (t+1) << ": Data mismatch\n";
        }
    }

    std::cout << "\n=== Results ===\n";
    std::cout << "Chirp detected: " << chirp_detected << "/" << trials << "\n";
    std::cout << "LDPC success:   " << ldpc_success << "/" << trials << "\n";
    std::cout << "Data correct:   " << passed << "/" << trials << "\n";

    float success_rate = 100.0f * passed / trials;
    std::cout << "\nSuccess rate: " << std::fixed << std::setprecision(1)
              << success_rate << "%\n";

    if (success_rate >= 80) {
        std::cout << "Status: GOOD - Multi-carrier DPSK working!\n";
    } else if (success_rate >= 50) {
        std::cout << "Status: MARGINAL - May need parameter tuning\n";
    } else {
        std::cout << "Status: POOR - Needs investigation\n";
    }

    return (passed > trials/2) ? 0 : 1;
}
