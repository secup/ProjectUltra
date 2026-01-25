// test_ofdm_chirp_pilots.cpp - Test OFDM with chirp sync + pilots on fading channels
//
// Tests coherent QPSK with pilots for channel tracking on fading channels.
// Goal: Exceed MC-DPSK speeds (>938 bps) at 10-15 dB on fading.

#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "sync/chirp_sync.hpp"
#include "sim/hf_channel.hpp"
#include "protocol/frame_v2.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <random>

using namespace ultra;
using namespace ultra::sync;
using namespace ultra::sim;

int main(int argc, char* argv[]) {
    std::cout << "=== OFDM Chirp + Pilots Test (Fading Channels) ===\n\n";

    // Parse args
    int num_carriers = 30;
    int pilot_spacing = 4;
    float snr_db = 15.0f;
    std::string channel_type = "moderate";
    std::string rate_str = "r12";
    int trials = 10;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--carriers") == 0 && i + 1 < argc) {
            num_carriers = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--pilots") == 0 && i + 1 < argc) {
            pilot_spacing = std::stoi(argv[++i]);
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
            std::cout << "  --carriers N   Number of carriers (default: 30)\n";
            std::cout << "  --pilots N     Pilot spacing (default: 4 = every 4th carrier)\n";
            std::cout << "  --snr dB       SNR in dB (default: 15)\n";
            std::cout << "  --channel X    Channel type: awgn, good, moderate, poor (default: moderate)\n";
            std::cout << "  --rate X       Code rate: r14, r12, r23, r34 (default: r12)\n";
            std::cout << "  --trials N     Number of trials (default: 10)\n";
            return 0;
        }
    }

    // Parse code rate
    CodeRate code_rate = CodeRate::R1_2;
    float rate_efficiency = 0.5f;
    if (rate_str == "r14" || rate_str == "R14") {
        code_rate = CodeRate::R1_4;
        rate_efficiency = 0.25f;
    } else if (rate_str == "r23" || rate_str == "R23") {
        code_rate = CodeRate::R2_3;
        rate_efficiency = 0.667f;
    } else if (rate_str == "r34" || rate_str == "R34") {
        code_rate = CodeRate::R3_4;
        rate_efficiency = 0.75f;
    }

    constexpr float SAMPLE_RATE = 48000.0f;

    // Configure OFDM with pilots
    ModemConfig ofdm_cfg;
    ofdm_cfg.sample_rate = SAMPLE_RATE;
    ofdm_cfg.fft_size = 512;
    ofdm_cfg.num_carriers = num_carriers;
    ofdm_cfg.modulation = Modulation::QPSK;  // Coherent
    ofdm_cfg.code_rate = code_rate;
    ofdm_cfg.use_pilots = true;
    ofdm_cfg.pilot_spacing = pilot_spacing;
    ofdm_cfg.scattered_pilots = true;
    ofdm_cfg.cp_mode = CyclicPrefixMode::MEDIUM;

    uint32_t data_carriers = ofdm_cfg.getDataCarriers();
    uint32_t pilot_count = num_carriers - data_carriers;
    float symbol_rate = ofdm_cfg.getSymbolRate();
    float raw_bps = data_carriers * 2 * symbol_rate;  // QPSK = 2 bits/symbol
    float net_bps = raw_bps * rate_efficiency;

    std::cout << "Configuration:\n";
    std::cout << "  Carriers: " << num_carriers << " (" << data_carriers << " data, " << pilot_count << " pilots)\n";
    std::cout << "  Pilot spacing: " << pilot_spacing << "\n";
    std::cout << "  Modulation: QPSK (coherent)\n";
    std::cout << "  Symbol rate: " << std::fixed << std::setprecision(1) << symbol_rate << " baud\n";
    std::cout << "  Raw bit rate: " << raw_bps << " bps\n";
    std::cout << "  Code rate: " << rate_str << " (" << (rate_efficiency * 100) << "% efficiency)\n";
    std::cout << "  Net throughput: " << net_bps << " bps\n";
    std::cout << "  SNR: " << snr_db << " dB\n";
    std::cout << "  Channel: " << channel_type << "\n";
    std::cout << "  Trials: " << trials << "\n\n";

    // Chirp sync config
    // IMPORTANT: Chirp amplitude must match OFDM signal levels for correct SNR!
    // OFDM has avg_power ~0.0001 (RMS ~0.01), so chirp amplitude should be ~0.02
    // Default chirp amplitude of 0.8 is ~500x higher, causing terrible OFDM SNR
    // when noise is calibrated to the combined signal.
    ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = SAMPLE_RATE;
    chirp_cfg.f_start = 300.0f;
    chirp_cfg.f_end = 2700.0f;
    chirp_cfg.duration_ms = 500.0f;
    chirp_cfg.repetitions = 1;
    chirp_cfg.gap_ms = 0.0f;
    chirp_cfg.amplitude = 0.02f;  // Match OFDM power level
    ChirpSync chirp_sync(chirp_cfg);

    // Test data - depends on code rate
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
        // Fresh modulator/demodulator each trial
        OFDMModulator modulator(ofdm_cfg);
        OFDMDemodulator demodulator(ofdm_cfg);

        // Build TX signal
        Samples tx_signal;

        // Lead-in silence (150ms)
        tx_signal.resize(7200, 0.0f);

        // Chirp preamble
        size_t chirp_start = tx_signal.size();
        Samples chirp = chirp_sync.generate();
        tx_signal.insert(tx_signal.end(), chirp.begin(), chirp.end());

        // Training symbols (2 LTS for channel estimation)
        size_t training_start = tx_signal.size();
        Samples training = modulator.generateTrainingSymbols(2);
        tx_signal.insert(tx_signal.end(), training.begin(), training.end());

        // Data symbols
        size_t data_start = tx_signal.size();
        Samples data = modulator.modulate(encoded, Modulation::QPSK);
        tx_signal.insert(tx_signal.end(), data.begin(), data.end());

        // Trail-out (500ms)
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
        size_t chirp_samples = chirp_sync.getTotalSamples();
        size_t detected_training = detected_chirp + chirp_samples;

        // Check bounds
        if (detected_training + training.size() + data.size() > rx_signal.size()) {
            std::cout << "Trial " << (t+1) << ": Buffer too short\n";
            continue;
        }

        // Process with presynced (bypass Schmidl-Cox, use chirp timing)
        // Use KNOWN training start position (since we built the signal ourselves)
        // This matches test_ofdm_chirp_basic.cpp which works with DQPSK
        size_t known_training_start = training_start;  // From TX signal construction
        size_t total_ofdm_samples = training.size() + data.size();
        SampleSpan ofdm_span(rx_signal.data() + known_training_start, total_ofdm_samples);

        demodulator.reset();
        bool ready = demodulator.processPresynced(ofdm_span, 2);  // 2 training symbols

        if (!ready) {
            std::cout << "Trial " << (t+1) << ": Demodulator not ready\n";
            continue;
        }

        auto soft_bits = demodulator.getSoftBits();

        if (soft_bits.size() < protocol::v2::LDPC_CODEWORD_BITS) {
            std::cout << "Trial " << (t+1) << ": Not enough soft bits ("
                      << soft_bits.size() << " < " << protocol::v2::LDPC_CODEWORD_BITS << ")\n";
            continue;
        }

        // LDPC decode
        std::vector<float> cw_bits(soft_bits.begin(),
                                    soft_bits.begin() + protocol::v2::LDPC_CODEWORD_BITS);

        // Calculate soft bit statistics
        float sb_abs_sum = 0;
        for (float sb : cw_bits) {
            sb_abs_sum += std::abs(sb);
        }

        LDPCDecoder decoder(code_rate);
        Bytes decoded = decoder.decodeSoft(cw_bits);

        if (!decoder.lastDecodeSuccess()) {
            std::cout << "Trial " << (t+1) << ": LDPC fail, corr="
                      << std::fixed << std::setprecision(3) << corr
                      << ", soft_avg=" << std::setprecision(2) << (sb_abs_sum/cw_bits.size())
                      << ", SNR_est=" << std::setprecision(1) << demodulator.getEstimatedSNR() << "dB\n";
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
                      << ", SNR_est=" << std::setprecision(1) << demodulator.getEstimatedSNR() << "dB\n";
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
    std::cout << "Net throughput: " << std::setprecision(0) << net_bps << " bps\n";

    if (success_rate >= 90) {
        std::cout << "Status: EXCELLENT - Ready for production!\n";
    } else if (success_rate >= 70) {
        std::cout << "Status: GOOD - Usable with retries\n";
    } else if (success_rate >= 50) {
        std::cout << "Status: MARGINAL - Needs tuning\n";
    } else {
        std::cout << "Status: POOR - Needs investigation\n";
    }

    return (passed > trials/2) ? 0 : 1;
}
