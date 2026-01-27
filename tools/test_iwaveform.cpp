// test_iwaveform.cpp - Test IWaveform interface via ModemEngine
//
// This tests that ModemEngine correctly uses the IWaveform interface for
// MC-DPSK and OFDM_CHIRP waveforms. Uses the full ModemEngine pipeline
// with acquisition thread, feedAudio, etc.
//
// Similar to test_hf_modem but focused on verifying IWaveform integration.
//
// Usage:
//   ./test_iwaveform [--snr N] [--cfo N] [--channel TYPE] [--frames N] [-w TYPE]

#include "gui/modem/modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "sim/hf_channel.hpp"
#include "ultra/logging.hpp"
#include "ultra/dsp.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <cstring>

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::sim;
namespace v2 = protocol::v2;

// ============================================================================
// Channel Simulation (from test_hf_modem)
// ============================================================================

void addNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
    float signal_power = 0.0f;
    size_t signal_samples = 0;
    for (float s : samples) {
        if (std::abs(s) > 1e-6f) {
            signal_power += s * s;
            signal_samples++;
        }
    }
    if (signal_samples == 0) return;
    signal_power /= signal_samples;

    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = signal_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) {
        s += noise(rng);
    }
}

// Apply CFO using Hilbert transform (simulates real radio frequency offset)
// This shifts the ENTIRE signal uniformly, like a real radio with tuning error
void applyCFO(std::vector<float>& samples, float cfo_hz, float sample_rate = 48000.0f) {
    if (samples.size() < 128 || std::abs(cfo_hz) < 0.001f) return;

    size_t N = samples.size();
    printf("[CFO] Applying %.1f Hz offset to %zu samples (%.1f sec)\n", cfo_hz, N, N/sample_rate);

    // Use FIR Hilbert transform for accurate analytic signal
    HilbertTransform hilbert(127);  // 127-tap filter

    float phase = 0.0f;
    float phase_inc = 2.0f * static_cast<float>(M_PI) * cfo_hz / sample_rate;

    // Process in chunks to avoid memory issues
    constexpr size_t chunk_size = 32768;

    for (size_t offset = 0; offset < N; offset += chunk_size) {
        size_t len = std::min(chunk_size, N - offset);

        SampleSpan span(samples.data() + offset, len);
        auto analytic = hilbert.process(span);

        // Frequency shift by multiplying analytic signal with complex exponential
        for (size_t i = 0; i < len; i++) {
            Complex rot(std::cos(phase), std::sin(phase));
            samples[offset + i] = std::real(analytic[i] * rot);
            phase += phase_inc;
            // Keep phase bounded
            if (phase > M_PI) phase -= 2.0f * M_PI;
            else if (phase < -M_PI) phase += 2.0f * M_PI;
        }
    }

    printf("[CFO] Done\n");
}

// ============================================================================
// Main Test
// ============================================================================

const char* channelName(const std::string& type) {
    if (type == "awgn") return "AWGN";
    if (type == "good") return "Good HF";
    if (type == "moderate") return "Moderate HF";
    if (type == "poor") return "Poor HF";
    return "Unknown";
}

int main(int argc, char** argv) {
    // Parse arguments
    float snr_db = 15.0f;
    float cfo_hz = 0.0f;
    std::string channel_type = "awgn";
    int num_frames = 10;
    std::string waveform_str = "mc_dpsk";
    bool verbose = false;
    uint32_t seed = 42;
    int num_carriers = 8;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--snr") == 0 && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (strcmp(argv[i], "--cfo") == 0 && i + 1 < argc) {
            cfo_hz = std::stof(argv[++i]);
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            channel_type = argv[++i];
        } else if (strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            num_frames = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            waveform_str = argv[++i];
        } else if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
            verbose = true;
        } else if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
            seed = std::stoul(argv[++i]);
        } else if (strcmp(argv[i], "--carriers") == 0 && i + 1 < argc) {
            num_carriers = std::stoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --snr N       SNR in dB (default: 15)\n");
            printf("  --cfo N       CFO in Hz (default: 0)\n");
            printf("  --channel T   Channel: awgn, good, moderate, poor (default: awgn)\n");
            printf("  --frames N    Number of frames (default: 10)\n");
            printf("  -w TYPE       Waveform: mc_dpsk, ofdm_chirp (default: mc_dpsk)\n");
            printf("  --carriers N  Number of carriers for MC-DPSK (default: 8)\n");
            printf("  --seed N      Random seed (default: 42)\n");
            printf("  -v            Verbose output\n");
            return 0;
        }
    }

    // Determine waveform mode
    protocol::WaveformMode waveform_mode;
    const char* waveform_name;
    if (waveform_str == "mc_dpsk" || waveform_str == "dpsk") {
        waveform_mode = protocol::WaveformMode::MC_DPSK;
        waveform_name = "MC-DPSK";
    } else if (waveform_str == "ofdm_chirp") {
        waveform_mode = protocol::WaveformMode::OFDM_CHIRP;
        waveform_name = "OFDM_CHIRP";
    } else if (waveform_str == "ofdm_cox" || waveform_str == "ofdm") {
        waveform_mode = protocol::WaveformMode::OFDM_COX;
        waveform_name = "OFDM_COX";
    } else {
        fprintf(stderr, "Unknown waveform: %s\n", waveform_str.c_str());
        return 1;
    }

    printf("=== IWaveform Interface Test (via ModemEngine) ===\n");
    printf("Waveform: %s", waveform_name);
    if (waveform_mode == protocol::WaveformMode::MC_DPSK) {
        printf(" (%d carriers)", num_carriers);
    }
    printf("\n");
    printf("Channel: %s, SNR: %.1f dB, CFO: %.1f Hz\n",
           channelName(channel_type), snr_db, cfo_hz);
    printf("Frames: %d, Seed: %u\n\n", num_frames, seed);

    std::mt19937 rng(seed);

    // ========================================
    // Create TX ModemEngine
    // ========================================
    ModemEngine tx_modem;
    tx_modem.setLogPrefix("TX");
    tx_modem.setWaveformMode(waveform_mode);
    tx_modem.setInterleavingEnabled(false);
    tx_modem.setFilterEnabled(false);

    // Configure TX (CFO applied as channel effect, not at modulator)
    if (waveform_mode == protocol::WaveformMode::MC_DPSK) {
        tx_modem.setMCDPSKCarriers(num_carriers);
    }

    // Generate test frame to determine size
    v2::ConnectFrame test_frame = v2::ConnectFrame::makeConnect(
        "TEST0", "DEST", protocol::ModeCapabilities::ALL,
        static_cast<uint8_t>(waveform_mode));
    auto test_audio = tx_modem.transmit(test_frame.serialize());
    float frame_duration_sec = test_audio.size() / 48000.0f;

    printf("Frame duration: %.1fs (%zu samples)\n", frame_duration_sec, test_audio.size());

    // Calculate total duration
    float min_gap_sec = 2.0f;
    float duration_sec = (frame_duration_sec + min_gap_sec) * num_frames + 5.0f;

    // ========================================
    // Generate frames
    // ========================================
    struct TestFrame {
        uint16_t seq;
        std::string src;
        Bytes data;
        size_t audio_start;
        size_t audio_len;
        bool decoded = false;
    };

    std::vector<TestFrame> frames(num_frames);
    size_t total_samples = static_cast<size_t>(duration_sec * 48000);
    std::vector<float> full_audio(total_samples, 0.0f);

    float avg_gap = (duration_sec - frame_duration_sec * num_frames) / (num_frames + 1);
    std::uniform_real_distribution<float> first_delay(1.0f, 3.0f);
    float current_time = first_delay(rng);

    printf("Generating %d %s frames...\n", num_frames, waveform_name);

    for (int i = 0; i < num_frames; i++) {
        v2::ConnectFrame frame = v2::ConnectFrame::makeConnect(
            "TEST" + std::to_string(i), "DEST",
            protocol::ModeCapabilities::ALL,
            static_cast<uint8_t>(waveform_mode));
        frame.seq = static_cast<uint16_t>(i + 1);

        frames[i].seq = frame.seq;
        frames[i].src = frame.getSrcCallsign();
        frames[i].data = frame.serialize();
        frames[i].audio_start = static_cast<size_t>(current_time * 48000);

        auto tx_audio = tx_modem.transmit(frames[i].data);
        frames[i].audio_len = tx_audio.size();

        if (verbose) {
            printf("  Frame %2d at %5.1fs: seq=%d src=%s\n",
                   i + 1, current_time, frame.seq, frames[i].src.c_str());
        }

        // Place in audio buffer
        size_t pos = frames[i].audio_start;
        for (size_t j = 0; j < tx_audio.size() && pos + j < full_audio.size(); j++) {
            full_audio[pos + j] = tx_audio[j];
        }

        std::uniform_real_distribution<float> gap_var(0.8f, 1.2f);
        current_time += frame_duration_sec + avg_gap * gap_var(rng);
    }

    // ========================================
    // Apply CFO (simulates radio tuning error - shifts entire signal)
    // ========================================
    if (std::abs(cfo_hz) > 0.001f) {
        applyCFO(full_audio, cfo_hz);
    }

    // ========================================
    // Apply channel
    // ========================================
    printf("Applying %s channel (SNR=%.1f dB)...\n", channelName(channel_type), snr_db);

    if (channel_type == "awgn") {
        addNoise(full_audio, snr_db, rng);
    } else {
        WattersonChannel::Config ch_cfg;
        ch_cfg.sample_rate = 48000.0f;
        ch_cfg.snr_db = snr_db;
        ch_cfg.noise_enabled = true;
        ch_cfg.cfo_enabled = false;

        if (channel_type == "good") {
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

        WattersonChannel channel(ch_cfg, seed);
        SampleSpan input_span(full_audio.data(), full_audio.size());
        full_audio = channel.process(input_span);
    }

    // ========================================
    // RX - Decode each frame region separately (like test_hf_modem)
    // ========================================
    printf("Decoding via ModemEngine.feedAudio()...\n\n");

    constexpr size_t chunk_size = 960;  // 20ms at 48kHz
    int decoded_count = 0;

    for (int i = 0; i < num_frames; i++) {
        // Create fresh RX modem for each frame
        ModemEngine rx_modem;
        rx_modem.setLogPrefix("RX");
        rx_modem.setWaveformMode(waveform_mode);
        rx_modem.setInterleavingEnabled(false);

        if (waveform_mode == protocol::WaveformMode::MC_DPSK) {
            rx_modem.setMCDPSKCarriers(num_carriers);
        }

        // Give RX threads time to start before feeding audio
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Define window around expected frame position
        size_t margin_before = 9600;   // 200ms
        size_t margin_after = 24000;   // 500ms
        size_t window_start = (frames[i].audio_start > margin_before) ?
                              frames[i].audio_start - margin_before : 0;
        size_t window_end = std::min(frames[i].audio_start + frames[i].audio_len + margin_after,
                                     full_audio.size());

        // Set up callback
        std::atomic<bool> got_frame{false};
        uint16_t received_seq = 0;

        rx_modem.setRawDataCallback([&](const Bytes& data) {
            if (data.size() >= 2 && data[0] == 0x55 && data[1] == 0x4C) {
                auto parsed = v2::ConnectFrame::deserialize(data);
                if (parsed) {
                    received_seq = parsed->seq;
                    got_frame = true;
                    if (verbose) {
                        printf("  [RX] Decoded seq=%d src=%s\n",
                               parsed->seq, parsed->getSrcCallsign().c_str());
                    }
                }
            }
        });

        // Feed audio
        for (size_t j = window_start; j < window_end; j += chunk_size) {
            size_t len = std::min(chunk_size, window_end - j);
            rx_modem.feedAudio(full_audio.data() + j, len);
        }

        // Wait for RX threads to process (chirp modes need longer)
        int max_wait_iters = 1000;  // 20 seconds max
        for (int wait = 0; wait < max_wait_iters && !got_frame; wait++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        // Check result
        if (got_frame && received_seq == frames[i].seq) {
            frames[i].decoded = true;
            decoded_count++;
            printf("  Frame %2d: OK\n", i + 1);
        } else if (got_frame) {
            printf("  Frame %2d: WRONG SEQ (got %d, expected %d)\n",
                   i + 1, received_seq, frames[i].seq);
        } else {
            printf("  Frame %2d: MISSED\n", i + 1);
        }
    }

    // ========================================
    // Results
    // ========================================
    printf("\n=== RESULTS ===\n");
    printf("TX frames: %d\n", num_frames);
    printf("Decoded: %d/%d (%.0f%%)\n", decoded_count, num_frames,
           100.0f * decoded_count / num_frames);

    return (decoded_count == num_frames) ? 0 : 1;
}
