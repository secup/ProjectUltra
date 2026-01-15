/**
 * OFDM vs OTFS Performance Comparison
 *
 * Compares Frame Success Rate (FSR) of OFDM and OTFS on ITU-R F.1487
 * channel conditions, particularly focusing on doubly-selective channels
 * (Poor: 2ms delay, Flutter: 10Hz Doppler) where OTFS should excel.
 *
 * OTFS advantage: Works in delay-Doppler domain where channel is sparse
 * and quasi-static, even for severely time/frequency selective channels.
 */

#include "../src/sim/hf_channel.hpp"
#include "ultra/types.hpp"
#include "ultra/fec.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/otfs.hpp"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>

using namespace ultra;
using namespace ultra::sim;

// ============================================================================
// OFDM Frame Success Rate measurement (existing code)
// ============================================================================

float measureOFDM_FSR(
    const ModemConfig& config,
    CodeRate rate,
    WattersonChannel::Config channel_cfg,
    size_t num_frames
) {
    OFDMModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(24, 27);
    WattersonChannel channel(channel_cfg);

    size_t max_data_bytes = (rate == CodeRate::R1_2) ? 40 : 20;
    size_t successful_frames = 0;

    for (size_t frame = 0; frame < num_frames; ++frame) {
        OFDMDemodulator demodulator(config);

        Bytes tx_data(max_data_bytes);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }

        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(interleaved, Modulation::QPSK);

        Samples tx_audio;
        tx_audio.reserve(preamble.size() + data_audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), data_audio.begin(), data_audio.end());

        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

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
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    rx_data.resize(tx_data.size());
                    if (rx_data == tx_data) {
                        successful_frames++;
                    }
                }
            }
        }
    }

    return static_cast<float>(successful_frames) / static_cast<float>(num_frames);
}

// ============================================================================
// OTFS Frame Success Rate measurement
// ============================================================================

float measureOTFS_FSR(
    const OTFSConfig& config,
    CodeRate rate,
    WattersonChannel::Config channel_cfg,
    size_t num_frames
) {
    OTFSModulator modulator(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    Interleaver interleaver(24, 27);
    WattersonChannel channel(channel_cfg);

    size_t max_data_bytes = (rate == CodeRate::R1_2) ? 40 : 20;
    size_t successful_frames = 0;

    for (size_t frame = 0; frame < num_frames; ++frame) {
        OTFSDemodulator demodulator(config);

        Bytes tx_data(max_data_bytes);
        for (size_t i = 0; i < tx_data.size(); ++i) {
            tx_data[i] = static_cast<uint8_t>((frame * 17 + i * 13 + 7) & 0xFF);
        }

        // Encode
        Bytes encoded = encoder.encode(tx_data);
        Bytes interleaved = interleaver.interleave(encoded);

        // Map to DD symbols
        auto dd_symbols = modulator.mapToDD(
            ByteSpan(interleaved.data(), interleaved.size()),
            Modulation::QPSK
        );

        // Generate preamble and modulate
        Samples preamble = modulator.generatePreamble();
        Samples data_audio = modulator.modulate(dd_symbols, Modulation::QPSK);

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

        // Pass through channel
        SampleSpan tx_span(tx_audio.data(), tx_audio.size());
        Samples rx_audio = channel.process(tx_span);

        // Demodulate
        bool frame_ready = false;
        size_t chunk_size = 1024;
        for (size_t offset = 0; offset < rx_audio.size() && !frame_ready; offset += chunk_size) {
            size_t remaining = std::min(chunk_size, rx_audio.size() - offset);
            SampleSpan rx_span(rx_audio.data() + offset, remaining);
            frame_ready = demodulator.process(rx_span);
        }

        if (frame_ready) {
            std::vector<float> soft_bits = demodulator.getSoftBits();
            // Truncate to exactly 648 bits (LDPC codeword size)
            if (soft_bits.size() > 648) {
                soft_bits.resize(648);
            }
            if (soft_bits.size() >= 648) {
                std::vector<float> deinterleaved = interleaver.deinterleave(soft_bits);
                Bytes rx_data = decoder.decodeSoft(deinterleaved);

                if (decoder.lastDecodeSuccess()) {
                    rx_data.resize(tx_data.size());
                    if (rx_data == tx_data) {
                        successful_frames++;
                    }
                }
            }
        }
    }

    return static_cast<float>(successful_frames) / static_cast<float>(num_frames);
}

// ============================================================================
// Main benchmark
// ============================================================================

int main() {
    std::cout << "\n";
    std::cout << "================================================================\n";
    std::cout << "         OFDM vs OTFS Performance Comparison\n";
    std::cout << "================================================================\n\n";

    std::cout << "OTFS (Orthogonal Time Frequency Space) is a 2D modulation\n";
    std::cout << "designed for doubly-selective (delay + Doppler) channels.\n";
    std::cout << "It works in the delay-Doppler domain where the channel is\n";
    std::cout << "SPARSE and QUASI-STATIC, even for severe fading.\n\n";

    std::cout << "Test: QPSK R1/2 at 25 dB SNR, 50 frames per condition\n";
    std::cout << "================================================================\n\n";

    // OFDM configuration
    ModemConfig ofdm_config;
    ofdm_config.sample_rate = 48000;
    ofdm_config.fft_size = 512;
    ofdm_config.num_carriers = 30;
    ofdm_config.cp_mode = CyclicPrefixMode::MEDIUM;
    ofdm_config.center_freq = 1500;
    ofdm_config.sync_threshold = 0.70f;
    ofdm_config.adaptive_eq_enabled = true;
    ofdm_config.lms_mu = 0.1f;

    // OTFS configuration
    // M and N must be powers of 2 for FFT
    // M=32, N=16 = 512 symbols = 1024 bits (QPSK)
    // LDPC R1/2 needs 648 bits, so we truncate soft bits
    OTFSConfig otfs_config;
    otfs_config.M = 32;           // 32 delay bins (subcarriers)
    otfs_config.N = 16;           // 16 Doppler bins (OFDM symbols)
    otfs_config.fft_size = 512;
    otfs_config.cp_length = 64;
    otfs_config.sample_rate = 48000;
    otfs_config.center_freq = 1500.0f;

    // Channel conditions
    struct ChannelTest {
        const char* name;
        WattersonChannel::Config (*getConfig)(float);
        int ofdm_pilot_spacing;
        const char* description;
    } channels[] = {
        {"AWGN",     itu_r_f1487::awgn,     6, "No fading (baseline)"},
        {"Good",     itu_r_f1487::good,     4, "0.5ms delay, 0.1Hz Doppler"},
        {"Moderate", itu_r_f1487::moderate, 2, "1.0ms delay, 0.5Hz Doppler"},
        {"Poor",     itu_r_f1487::poor,     2, "2.0ms delay, 1.0Hz Doppler"},
        {"Flutter",  itu_r_f1487::flutter,  4, "0.5ms delay, 10Hz Doppler"},
    };

    std::cout << std::setw(12) << "Channel"
              << std::setw(10) << "OFDM"
              << std::setw(10) << "OTFS"
              << std::setw(12) << "Winner"
              << "  Description\n";
    std::cout << std::string(70, '-') << "\n";

    int ofdm_wins = 0;
    int otfs_wins = 0;
    int ties = 0;

    for (const auto& ch : channels) {
        std::cout << std::setw(12) << ch.name << std::flush;

        // Configure OFDM pilot spacing
        ofdm_config.pilot_spacing = ch.ofdm_pilot_spacing;

        // Get channel config at 25 dB SNR
        auto cfg = ch.getConfig(25.0f);

        // Measure OFDM FSR
        float ofdm_fsr = measureOFDM_FSR(ofdm_config, CodeRate::R1_2, cfg, 50);
        std::cout << std::setw(9) << std::fixed << std::setprecision(0)
                  << (ofdm_fsr * 100) << "%" << std::flush;

        // Measure OTFS FSR
        float otfs_fsr = measureOTFS_FSR(otfs_config, CodeRate::R1_2, cfg, 50);
        std::cout << std::setw(9) << std::fixed << std::setprecision(0)
                  << (otfs_fsr * 100) << "%";

        // Determine winner
        if (otfs_fsr > ofdm_fsr + 0.05f) {
            std::cout << std::setw(12) << "OTFS";
            otfs_wins++;
        } else if (ofdm_fsr > otfs_fsr + 0.05f) {
            std::cout << std::setw(12) << "OFDM";
            ofdm_wins++;
        } else {
            std::cout << std::setw(12) << "TIE";
            ties++;
        }

        std::cout << "  " << ch.description << "\n";
    }

    std::cout << std::string(70, '-') << "\n\n";

    std::cout << "Summary:\n";
    std::cout << "  OFDM wins: " << ofdm_wins << "\n";
    std::cout << "  OTFS wins: " << otfs_wins << "\n";
    std::cout << "  Ties: " << ties << "\n\n";

    if (otfs_wins > ofdm_wins) {
        std::cout << "OTFS shows improvement on challenging channels!\n";
    } else if (ofdm_wins > otfs_wins) {
        std::cout << "OFDM performs better - OTFS implementation may need tuning.\n";
    } else {
        std::cout << "Both perform similarly - may need different test conditions.\n";
    }

    std::cout << "\n================================================================\n";

    return 0;
}
