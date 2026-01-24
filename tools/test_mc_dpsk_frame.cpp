// test_mc_dpsk_frame.cpp - Fast MC-DPSK frame test (no threading)
//
// Tests full v2 CONNECT frames (3 codewords) through MC-DPSK
// Same audio generation as test_hf_modem but direct decode for speed

#include "sync/chirp_sync.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "ultra/fec.hpp"
#include "protocol/frame_v2.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>

using namespace ultra;
using namespace ultra::sync;
using namespace ultra::sim;
namespace v2 = protocol::v2;

int main(int argc, char* argv[]) {
    float snr_db = 15.0f;
    int trials = 10;
    bool verbose = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (strcmp(argv[i], "--trials") == 0 && i + 1 < argc) {
            trials = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [--snr dB] [--trials N] [-v]\n";
            return 0;
        }
    }

    constexpr float SAMPLE_RATE = 48000.0f;

    // MC-DPSK config (same as ModemEngine)
    MultiCarrierDPSKConfig mc_cfg;
    mc_cfg.sample_rate = SAMPLE_RATE;
    mc_cfg.num_carriers = 8;
    mc_cfg.freq_low = 500.0f;
    mc_cfg.freq_high = 2500.0f;
    mc_cfg.samples_per_symbol = 512;
    mc_cfg.bits_per_symbol = 2;
    mc_cfg.training_symbols = 8;

    // Chirp config (single chirp like test_mc_dpsk)
    ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = SAMPLE_RATE;
    chirp_cfg.f_start = 300.0f;
    chirp_cfg.f_end = 2700.0f;
    chirp_cfg.duration_ms = 500.0f;
    chirp_cfg.repetitions = 1;  // Single chirp for fading channels
    chirp_cfg.gap_ms = 0.0f;
    ChirpSync chirp_sync(chirp_cfg);

    // Create v2 CONNECT frame (same as test_hf_modem)
    v2::ConnectFrame frame = v2::ConnectFrame::makeConnect(
        "TEST0", "DEST", protocol::ModeCapabilities::ALL,
        static_cast<uint8_t>(protocol::WaveformMode::OFDM));
    frame.seq = 1;
    Bytes frame_data = frame.serialize();

    // Encode with LDPC R1/4 (3 codewords for CONNECT)
    auto encoded_cws = v2::encodeFrameWithLDPC(frame_data, CodeRate::R1_4);
    Bytes encoded;
    for (const auto& cw : encoded_cws) {
        encoded.insert(encoded.end(), cw.begin(), cw.end());
    }

    std::cout << "=== MC-DPSK Full Frame Test ===\n\n";
    std::cout << "Config: " << mc_cfg.num_carriers << " carriers, "
              << mc_cfg.samples_per_symbol << " samples/sym\n";
    std::cout << "Frame: " << frame_data.size() << " bytes -> "
              << encoded_cws.size() << " codewords -> "
              << encoded.size() << " encoded bytes\n";
    std::cout << "SNR: " << snr_db << " dB, Trials: " << trials << "\n\n";

    // Channel configs
    struct ChannelTest {
        const char* name;
        float delay_ms;
        float doppler_hz;
        float path1_gain;
        float path2_gain;
        bool fading;
        bool multipath;
    };

    ChannelTest channels[] = {
        {"AWGN",     0.0f, 0.0f, 1.0f,   0.0f,   false, false},
        {"Good",     0.5f, 0.2f, 0.9f,   0.4f,   true,  true},
        {"Moderate", 1.0f, 0.5f, 0.707f, 0.707f, true,  true},
        {"Poor",     2.0f, 1.0f, 0.6f,   0.8f,   true,  true},
    };

    for (const auto& ch : channels) {
        int passed = 0;
        int chirp_ok = 0;
        int cfo_reject = 0;
        int ldpc_fail = 0;

        for (int t = 0; t < trials; t++) {
            // Fresh modulator/demodulator each trial
            MultiCarrierDPSKModulator modulator(mc_cfg);
            MultiCarrierDPSKDemodulator demodulator(mc_cfg);

            // Build TX signal
            Samples tx_signal;
            tx_signal.resize(7200, 0.0f);  // 150ms lead-in

            // Chirp preamble
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

            // Trail-out (extended for multipath tail)
            tx_signal.resize(tx_signal.size() + 48000, 0.0f);

            // Apply channel
            WattersonChannel::Config ch_cfg;
            ch_cfg.sample_rate = SAMPLE_RATE;
            ch_cfg.snr_db = snr_db;
            ch_cfg.delay_spread_ms = ch.delay_ms;
            ch_cfg.doppler_spread_hz = ch.doppler_hz;
            ch_cfg.path1_gain = ch.path1_gain;
            ch_cfg.path2_gain = ch.path2_gain;
            ch_cfg.fading_enabled = ch.fading;
            ch_cfg.multipath_enabled = ch.multipath;
            ch_cfg.noise_enabled = true;

            WattersonChannel channel(ch_cfg, 42 + t);
            SampleSpan tx_span(tx_signal.data(), tx_signal.size());
            Samples rx_signal = channel.process(tx_span);

            // Detect chirp
            SampleSpan rx_span(rx_signal.data(), rx_signal.size());
            float corr = 0.0f;
            int chirp_pos = chirp_sync.detect(rx_span, corr, 0.35f);

            if (chirp_pos < 0) {
                if (verbose) std::cout << "  Trial " << (t+1) << ": Chirp not found\n";
                continue;
            }
            chirp_ok++;

            // Calculate offsets
            size_t chirp_end = chirp_pos + chirp_sync.getTotalSamples();
            size_t det_training = chirp_end;
            size_t training_len = mc_cfg.training_symbols * mc_cfg.samples_per_symbol;
            size_t det_ref = det_training + training_len;
            size_t det_data = det_ref + mc_cfg.samples_per_symbol;

            if (det_data + data.size() > rx_signal.size()) {
                if (verbose) std::cout << "  Trial " << (t+1) << ": Buffer too short\n";
                continue;
            }

            // Process training for CFO
            SampleSpan train_span(rx_signal.data() + det_training, training_len);
            demodulator.processTraining(train_span);

            // CFO sanity check (relaxed for fading channels)
            float cfo = demodulator.getEstimatedCFO();
            if (std::abs(cfo) > 5.0f) {
                cfo_reject++;
                if (verbose) std::cout << "  Trial " << (t+1) << ": CFO reject (" << cfo << " Hz)\n";
                continue;
            }

            // Set reference
            SampleSpan ref_span(rx_signal.data() + det_ref, mc_cfg.samples_per_symbol);
            demodulator.setReference(ref_span);

            // Demodulate
            SampleSpan data_span(rx_signal.data() + det_data, data.size());
            auto soft_bits = demodulator.demodulateSoft(data_span);

            // Decode all 3 codewords
            bool all_ok = true;
            for (size_t cw = 0; cw < encoded_cws.size() && all_ok; cw++) {
                if (soft_bits.size() < (cw + 1) * v2::LDPC_CODEWORD_BITS) {
                    all_ok = false;
                    break;
                }

                std::vector<float> cw_bits(
                    soft_bits.begin() + cw * v2::LDPC_CODEWORD_BITS,
                    soft_bits.begin() + (cw + 1) * v2::LDPC_CODEWORD_BITS);

                LDPCDecoder decoder(CodeRate::R1_4);
                Bytes decoded = decoder.decodeSoft(cw_bits);

                if (!decoder.lastDecodeSuccess()) {
                    all_ok = false;
                    ldpc_fail++;
                }
            }

            if (all_ok) {
                passed++;
                if (verbose) std::cout << "  Trial " << (t+1) << ": PASS (CFO=" << cfo << ")\n";
            } else {
                if (verbose) std::cout << "  Trial " << (t+1) << ": LDPC fail\n";
            }
        }

        float rate = 100.0f * passed / trials;
        std::cout << std::left << std::setw(10) << ch.name
                  << ": " << std::right << std::setw(3) << passed << "/" << trials
                  << " (" << std::fixed << std::setprecision(0) << std::setw(3) << rate << "%)";

        if (chirp_ok < trials || cfo_reject > 0 || ldpc_fail > 0) {
            std::cout << "  [chirp:" << chirp_ok << " cfo_rej:" << cfo_reject
                      << " ldpc_fail:" << ldpc_fail << "]";
        }
        std::cout << "\n";
    }

    return 0;
}
