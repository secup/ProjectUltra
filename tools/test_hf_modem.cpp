/**
 * HF Modem Pipeline Test
 *
 * Tests the full ModemEngine audio pipeline (same path as real HF rig):
 * 1. Generate v2 protocol frames
 * 2. TX via ModemEngine.transmit() -> audio samples
 * 3. Add HF channel noise
 * 4. RX via ModemEngine.feedAudio() (same as plugging in HF rig)
 * 5. Verify frames via setRawDataCallback()
 *
 * This validates the complete audio pipeline that would be used
 * when connecting a real HF transceiver.
 */

#include "gui/modem/modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;
using ultra::protocol::WaveformMode;

// Apply AWGN channel - calculate signal power only from non-silent samples
void addNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
    // Calculate signal power only from active (non-silent) portions
    float signal_energy = 0;
    size_t active_samples = 0;
    for (float s : samples) {
        if (std::abs(s) > 1e-6f) {
            signal_energy += s * s;
            active_samples++;
        }
    }
    if (active_samples == 0) return;

    float signal_power = signal_energy / active_samples;
    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) s += noise(rng);
}

// Save audio for debugging
bool saveAudio(const std::vector<float>& audio, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;
    file.write(reinterpret_cast<const char*>(audio.data()), audio.size() * sizeof(float));
    return file.good();
}

int main(int argc, char* argv[]) {
    setLogLevel(LogLevel::WARN);

    float snr_db = 25.0f;
    int num_frames = 10;
    float duration_sec = 30.0f;
    bool verbose = false;
    bool save_audio = false;
    bool play_audio = false;
    std::string output_file = "hf_modem_test.f32";
    WaveformMode waveform_mode = WaveformMode::OFDM;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--snr" && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            num_frames = std::stoi(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stof(argv[++i]);
        } else if (arg == "--save") {
            save_audio = true;
        } else if (arg == "-o" && i + 1 < argc) {
            output_file = argv[++i];
            save_audio = true;
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            setLogLevel(LogLevel::DEBUG);
        } else if (arg == "--play" || arg == "-p") {
            play_audio = true;
        } else if ((arg == "-w" || arg == "--waveform") && i + 1 < argc) {
            std::string mode = argv[++i];
            if (mode == "ofdm") {
                waveform_mode = WaveformMode::OFDM;
            } else if (mode == "dpsk") {
                waveform_mode = WaveformMode::DPSK;
            } else {
                fprintf(stderr, "Unknown waveform mode: %s\n", mode.c_str());
                return 1;
            }
        } else if (arg == "-h" || arg == "--help") {
            printf("HF Modem Pipeline Test\n\n");
            printf("Tests full ModemEngine audio pipeline (same as real HF rig)\n\n");
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --snr <dB>       SNR level (default: 25)\n");
            printf("  --frames <n>     Number of frames (default: 10)\n");
            printf("  --duration <s>   Audio duration in seconds (default: 30)\n");
            printf("  -w, --waveform   Waveform mode: ofdm, dpsk (default: ofdm)\n");
            printf("  -p, --play       Play audio through speakers\n");
            printf("  --save           Save audio file for playback\n");
            printf("  -o <file>        Output audio file (implies --save)\n");
            printf("  -v, --verbose    Enable verbose output\n");
            printf("\nPlayback: aplay -f FLOAT_LE -r 48000 -c 1 %s\n", output_file.c_str());
            return 0;
        }
    }

    std::mt19937 rng(42);

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║        HF MODEM PIPELINE TEST - Full Audio Path              ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    const char* waveform_name = "?";
    if (waveform_mode == WaveformMode::OFDM) waveform_name = "OFDM";
    else if (waveform_mode == WaveformMode::DPSK) waveform_name = "DPSK";

    printf("Configuration:\n");
    printf("  SNR:        %.1f dB\n", snr_db);
    printf("  Frames:     %d\n", num_frames);
    printf("  Duration:   %.0f seconds\n", duration_sec);
    printf("  Waveform:   %s\n\n", waveform_name);

    // ========================================
    // TX Setup - Generate v2 frames
    // ========================================
    ModemEngine tx_modem;
    tx_modem.setLogPrefix("TX");
    // Set connected state so TX uses the selected waveform (not DPSK connect waveform)
    tx_modem.setConnected(true);
    tx_modem.setHandshakeComplete(true);
    tx_modem.setWaveformMode(waveform_mode);

    // Generate frames with random timing
    struct TestFrame {
        uint16_t seq;
        std::string src;
        std::string dst;
        size_t audio_start;
        size_t audio_len;  // Length of TX audio (varies by waveform)
        std::atomic<bool> decoded{false};
    };

    // Generate a test frame to determine frame length
    v2::ConnectFrame test_frame = v2::ConnectFrame::makeConnect(
        "TEST0", "DEST", protocol::ModeCapabilities::ALL,
        static_cast<uint8_t>(WaveformMode::OFDM));
    auto test_audio = tx_modem.transmit(test_frame.serialize());
    float frame_duration_sec = test_audio.size() / 48000.0f;

    // Calculate minimum duration needed: (frame_len + gap) * num_frames + margins
    float min_gap_sec = 2.0f;  // Minimum gap between frames
    float min_duration = (frame_duration_sec + min_gap_sec) * num_frames + 5.0f;
    if (duration_sec < min_duration) {
        duration_sec = min_duration;
        printf("Adjusted duration to %.0fs for %s frames (%.1fs each)\n",
               duration_sec, waveform_name, frame_duration_sec);
    }

    std::vector<TestFrame> frames(num_frames);
    size_t total_samples = (size_t)(duration_sec * 48000);
    std::vector<float> full_audio(total_samples, 0.0f);

    // Spread frames across duration with proper spacing
    float avg_gap = (duration_sec - frame_duration_sec * num_frames) / (num_frames + 1);
    std::uniform_real_distribution<float> first_delay(1.0f, 3.0f);
    float current_time = first_delay(rng);

    printf("Generating %d v2 CONNECT frames via %s (%.1fs each)...\n",
           num_frames, waveform_name, frame_duration_sec);

    for (int i = 0; i < num_frames; i++) {
        // Create v2 CONNECT frame
        v2::ConnectFrame frame = v2::ConnectFrame::makeConnect(
            "TEST" + std::to_string(i),  // Unique source callsign
            "DEST",
            protocol::ModeCapabilities::ALL,
            static_cast<uint8_t>(WaveformMode::OFDM)
        );
        frame.seq = static_cast<uint16_t>(i + 1);

        Bytes frame_data = frame.serialize();

        frames[i].seq = frame.seq;
        frames[i].src = frame.getSrcCallsign();
        frames[i].dst = frame.getDstCallsign();
        frames[i].audio_start = (size_t)(current_time * 48000);

        // Transmit frame
        auto tx_audio = tx_modem.transmit(frame_data);
        frames[i].audio_len = tx_audio.size();

        if (verbose) {
            printf("  Frame %2d at %5.1fs: CONNECT seq=%d src=%s (%zu samples)\n",
                   i + 1, current_time, frame.seq, frames[i].src.c_str(), tx_audio.size());
        }

        // Place in full audio buffer
        size_t pos = frames[i].audio_start;
        for (size_t j = 0; j < tx_audio.size() && pos + j < full_audio.size(); j++) {
            full_audio[pos + j] = tx_audio[j];
        }

        // Next frame: frame_duration + random gap
        std::uniform_real_distribution<float> gap_var(0.8f, 1.2f);
        current_time += frame_duration_sec + avg_gap * gap_var(rng);
    }

    // ========================================
    // Add HF channel noise
    // ========================================
    printf("Adding channel noise (SNR=%.1f dB)...\n", snr_db);
    addNoise(full_audio, snr_db, rng);

    // Save audio if requested
    if (save_audio) {
        printf("Saving audio to %s...\n", output_file.c_str());
        if (saveAudio(full_audio, output_file)) {
            printf("  Play with: aplay -f FLOAT_LE -r 48000 -c 1 %s\n", output_file.c_str());
        }
    }

    // Play audio through speakers if requested
    if (play_audio) {
        printf("\n🔊 Playing audio through speakers (%.1f seconds)...\n", full_audio.size() / 48000.0f);
        FILE* aplay = popen("aplay -f FLOAT_LE -r 48000 -c 1 -q 2>/dev/null", "w");
        if (aplay) {
            fwrite(full_audio.data(), sizeof(float), full_audio.size(), aplay);
            pclose(aplay);
            printf("🔊 Playback complete.\n");
        } else {
            fprintf(stderr, "Warning: Could not open aplay for playback\n");
        }
    }

    // ========================================
    // RX - Decode each frame region separately
    // ========================================
    printf("\nDecoding via ModemEngine.feedAudio()...\n\n");

    // Process each frame region separately (like real HF - fresh sync per burst)
    for (int i = 0; i < num_frames; i++) {
        // Create fresh RX modem for each frame
        ModemEngine rx_modem;
        rx_modem.setLogPrefix("RX");
        rx_modem.setConnected(true);
        rx_modem.setHandshakeComplete(true);
        rx_modem.setWaveformMode(waveform_mode);

        // Setup callback
        std::atomic<bool> got_frame{false};
        uint16_t received_seq = 0;

        rx_modem.setRawDataCallback([&](const Bytes& data) {
            if (data.size() >= 2 && data[0] == 0x55 && data[1] == 0x4C) {
                auto parsed = v2::ConnectFrame::deserialize(data);
                if (parsed) {
                    received_seq = parsed->seq;
                    got_frame = true;
                    if (verbose) {
                        printf("  [RX] CONNECT seq=%d src=%s\n",
                               parsed->seq, parsed->getSrcCallsign().c_str());
                    }
                }
            }
        });

        // Extract window around frame (margin before + full audio + margin after)
        size_t margin_before = 9600;   // 200ms
        size_t margin_after = 24000;   // 500ms
        size_t window_start = (frames[i].audio_start > margin_before) ?
                              frames[i].audio_start - margin_before : 0;
        size_t window_end = std::min(frames[i].audio_start + frames[i].audio_len + margin_after,
                                     full_audio.size());

        // Feed audio in chunks
        size_t chunk_size = 960;
        for (size_t j = window_start; j < window_end; j += chunk_size) {
            size_t len = std::min(chunk_size, window_end - j);
            rx_modem.feedAudio(full_audio.data() + j, len);
        }

        // Wait for RX threads to process
        for (int wait = 0; wait < 100 && !got_frame; wait++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        if (got_frame && received_seq == frames[i].seq) {
            frames[i].decoded = true;
        }
    }

    // ========================================
    // Results
    // ========================================
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("                         RESULTS\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    int success = 0;
    for (int i = 0; i < num_frames; i++) {
        const char* status = frames[i].decoded ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m";
        printf("  %s Frame %2d at %5.1fs: CONNECT seq=%d src=%s\n",
               status, i + 1, frames[i].audio_start / 48000.0f,
               frames[i].seq, frames[i].src.c_str());
        if (frames[i].decoded) success++;
    }

    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("                        SUMMARY\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    printf("  Decoded:  %d/%d (%d%%)\n", success, num_frames,
           num_frames > 0 ? success * 100 / num_frames : 0);

    printf("\n");
    if (success == num_frames) {
        printf("  \033[32m★★★ PERFECT! Full audio pipeline validated! ★★★\033[0m\n");
    } else if (success >= num_frames * 9 / 10) {
        printf("  \033[33m★★ Excellent - Minor losses ★★\033[0m\n");
    } else if (success >= num_frames * 7 / 10) {
        printf("  \033[33m★ Good - Some frames lost ★\033[0m\n");
    } else {
        printf("  \033[31m⚠ Pipeline needs debugging ⚠\033[0m\n");
    }

    if (save_audio) {
        printf("\n  Audio saved to: %s\n", output_file.c_str());
    }

    printf("\n");
    return (success == num_frames) ? 0 : 1;
}
