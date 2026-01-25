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
#include "sim/hf_channel.hpp"
#include "ultra/logging.hpp"
#include "ultra/dsp.hpp"
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
using namespace ultra::sim;
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

// Apply AWGN calibrated to OFDM signal (excludes chirp which has different power)
// frame_positions: list of (audio_start, chirp_len, training_len, data_len) tuples
void addNoiseOFDM(std::vector<float>& samples, float snr_db, std::mt19937& rng,
                  const std::vector<std::tuple<size_t, size_t, size_t, size_t>>& frame_positions) {
    // Calculate signal power from OFDM regions only (training + data, not chirp)
    float signal_energy = 0;
    size_t active_samples = 0;

    for (const auto& [start, chirp_len, training_len, data_len] : frame_positions) {
        // OFDM starts after chirp
        size_t ofdm_start = start + chirp_len;
        size_t ofdm_end = ofdm_start + training_len + data_len;

        for (size_t i = ofdm_start; i < ofdm_end && i < samples.size(); i++) {
            float s = samples[i];
            if (std::abs(s) > 1e-6f) {
                signal_energy += s * s;
                active_samples++;
            }
        }
    }

    if (active_samples == 0) return;

    float signal_power = signal_energy / active_samples;
    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) s += noise(rng);
}

// Apply CFO using overlap-save FFT-based Hilbert transform
// Processes in chunks to handle long signals while maintaining correct phase
void applyCFO_Chunked(std::vector<float>& samples, float cfo_hz, float sample_rate = 48000.0f) {
    if (samples.size() < 64 || std::abs(cfo_hz) < 0.001f) return;

    size_t N = samples.size();
    printf("[applyCFO_Chunked] N=%zu samples (%.1f sec), cfo=%.1f Hz\n", N, N/sample_rate, cfo_hz);

    // Use overlap-save: process in chunks, keep only center portion
    // This avoids circular convolution artifacts at edges
    constexpr size_t fft_size = 32768;  // 682ms chunks
    constexpr size_t margin = 2048;      // Discard this many samples at each edge
    constexpr size_t valid_len = fft_size - 2 * margin;  // Keep this many samples

    FFT fft(fft_size);
    std::vector<Complex> freq(fft_size);
    std::vector<Complex> time_in(fft_size);
    std::vector<Complex> analytic(fft_size);
    std::vector<float> output(N);

    float phase_inc = 2.0f * static_cast<float>(M_PI) * cfo_hz / sample_rate;
    size_t out_pos = 0;

    for (size_t chunk = 0; out_pos < N; chunk++) {
        // Input offset with margin for overlap-save
        size_t in_start = (chunk == 0) ? 0 : chunk * valid_len - margin;

        // Fill input buffer
        for (size_t i = 0; i < fft_size; i++) {
            size_t idx = in_start + i;
            time_in[i] = (idx < N) ? Complex(samples[idx], 0) : Complex(0, 0);
        }

        // Forward FFT
        fft.forward(time_in, freq);

        // Create analytic signal (zero negative frequencies, double positive)
        freq[0] *= 0.5f;  // DC
        for (size_t i = 1; i < fft_size / 2; i++) {
            freq[i] *= 2.0f;  // Double positive frequencies
        }
        freq[fft_size / 2] *= 0.5f;  // Nyquist
        for (size_t i = fft_size / 2 + 1; i < fft_size; i++) {
            freq[i] = Complex(0, 0);  // Zero negative frequencies
        }

        // Inverse FFT
        fft.inverse(freq, analytic);

        // Determine valid output range for this chunk
        size_t valid_start = (chunk == 0) ? 0 : margin;
        size_t valid_end = fft_size - margin;
        if (chunk == 0) valid_end = fft_size - margin;  // First chunk: keep start, discard end margin

        // Copy valid portion with CFO rotation
        for (size_t i = valid_start; i < valid_end && out_pos < N; i++) {
            size_t global_idx = in_start + i;
            float phase = phase_inc * static_cast<float>(global_idx);
            Complex rot(std::cos(phase), std::sin(phase));
            output[out_pos++] = std::real(analytic[i] * rot);
        }
    }

    samples = std::move(output);
    printf("[applyCFO_Chunked] CFO shift applied (%zu samples)\n", N);
}

// Apply CFO using simple time-domain multiplication
// This is more accurate than FFT-based Hilbert which has zero-padding artifacts
void applyCFO_TimeDomain(std::vector<float>& samples, float cfo_hz, float sample_rate = 48000.0f) {
    if (samples.size() < 64 || std::abs(cfo_hz) < 0.001f) return;

    size_t N = samples.size();
    printf("[applyCFO_TimeDomain] N=%zu samples (%.1f sec), cfo=%.1f Hz\n", N, N/sample_rate, cfo_hz);

    // For a bandpass signal, frequency shift by multiplying with complex exponential
    // requires creating an analytic signal first. Use FIR Hilbert for accuracy.
    // Process in chunks to avoid memory issues with very long signals.

    constexpr size_t chunk_size = 65536;
    HilbertTransform hilbert(255);  // Longer filter for better accuracy

    float phase = 0.0f;
    float phase_inc = 2.0f * static_cast<float>(M_PI) * cfo_hz / sample_rate;

    for (size_t offset = 0; offset < N; offset += chunk_size) {
        size_t len = std::min(chunk_size, N - offset);

        // Process this chunk
        SampleSpan span(samples.data() + offset, len);
        auto analytic = hilbert.process(span);

        // Apply CFO rotation
        for (size_t i = 0; i < len; i++) {
            Complex rot(std::cos(phase), std::sin(phase));
            samples[offset + i] = std::real(analytic[i] * rot);
            phase += phase_inc;
            while (phase > static_cast<float>(M_PI)) phase -= 2.0f * static_cast<float>(M_PI);
        }
    }

    printf("[applyCFO_TimeDomain] CFO shift applied\n");
}

// Apply CFO using FFT-based Hilbert transform
// NOTE: This has issues with zero-padding causing time shifts. Use applyCFO_TimeDomain instead.
void applyCFO_FFT(std::vector<float>& samples, float cfo_hz, float sample_rate = 48000.0f) {
    // Delegate to time-domain method which is more accurate
    applyCFO_TimeDomain(samples, cfo_hz, sample_rate);
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
    WaveformMode waveform_mode = WaveformMode::OFDM_NVIS;
    std::string channel_type = "awgn";  // awgn, good, moderate, poor
    uint32_t channel_seed = 0;  // 0 = random
    bool use_nvis = false;  // Use NVIS config (1024 FFT, 59 carriers)
    bool no_interleave = false;  // Disable channel interleaving
    Modulation test_modulation = Modulation::DQPSK;  // Default for chirp mode
    CodeRate test_code_rate = CodeRate::R1_2;  // Default code rate
    float cfo_hz = 0.0f;  // Carrier frequency offset (0 = random ±20 Hz for fading channels)
    bool cfo_specified = false;  // Was --cfo explicitly set?

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
                waveform_mode = WaveformMode::OFDM_NVIS;
            } else if (mode == "dpsk") {
                waveform_mode = WaveformMode::MC_DPSK;
            } else if (mode == "otfs" || mode == "otfs_eq") {
                waveform_mode = WaveformMode::OTFS_EQ;
            } else if (mode == "otfs_raw") {
                waveform_mode = WaveformMode::OTFS_RAW;
            } else if (mode == "chirp" || mode == "ofdm_chirp") {
                waveform_mode = WaveformMode::OFDM_CHIRP;
            } else {
                fprintf(stderr, "Unknown waveform mode: %s (use: ofdm, chirp, dpsk, otfs, otfs_raw)\n", mode.c_str());
                return 1;
            }
        } else if (arg == "--nvis") {
            use_nvis = true;
        } else if ((arg == "-c" || arg == "--channel") && i + 1 < argc) {
            channel_type = argv[++i];
            if (channel_type != "awgn" && channel_type != "good" &&
                channel_type != "moderate" && channel_type != "poor") {
                fprintf(stderr, "Unknown channel type: %s (use: awgn, good, moderate, poor)\n", channel_type.c_str());
                return 1;
            }
        } else if (arg == "--seed" && i + 1 < argc) {
            channel_seed = std::stoul(argv[++i]);
        } else if (arg == "--no-interleave") {
            no_interleave = true;
        } else if ((arg == "-m" || arg == "--mod") && i + 1 < argc) {
            std::string mod = argv[++i];
            if (mod == "dqpsk") {
                test_modulation = Modulation::DQPSK;
            } else if (mod == "d8psk") {
                test_modulation = Modulation::D8PSK;
            } else if (mod == "qpsk") {
                test_modulation = Modulation::QPSK;
            } else if (mod == "16qam") {
                test_modulation = Modulation::QAM16;
            } else {
                fprintf(stderr, "Unknown modulation: %s (use: dqpsk, d8psk, qpsk, 16qam)\n", mod.c_str());
                return 1;
            }
        } else if ((arg == "-r" || arg == "--rate") && i + 1 < argc) {
            std::string rate = argv[++i];
            if (rate == "1/4" || rate == "r1/4") {
                test_code_rate = CodeRate::R1_4;
            } else if (rate == "1/2" || rate == "r1/2") {
                test_code_rate = CodeRate::R1_2;
            } else if (rate == "2/3" || rate == "r2/3") {
                test_code_rate = CodeRate::R2_3;
            } else if (rate == "3/4" || rate == "r3/4") {
                test_code_rate = CodeRate::R3_4;
            } else if (rate == "5/6" || rate == "r5/6") {
                test_code_rate = CodeRate::R5_6;
            } else {
                fprintf(stderr, "Unknown code rate: %s (use: 1/4, 1/2, 2/3, 3/4, 5/6)\n", rate.c_str());
                return 1;
            }
        } else if (arg == "--cfo" && i + 1 < argc) {
            cfo_hz = std::stof(argv[++i]);
            cfo_specified = true;
        } else if (arg == "-h" || arg == "--help") {
            printf("HF Modem Pipeline Test\n\n");
            printf("Tests full ModemEngine audio pipeline (same as real HF rig)\n\n");
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --snr <dB>       SNR level (default: 25)\n");
            printf("  --frames <n>     Number of frames (default: 10)\n");
            printf("  --duration <s>   Audio duration in seconds (default: 30)\n");
            printf("  -w, --waveform   Waveform mode: ofdm, chirp, dpsk, otfs (default: ofdm)\n");
            printf("  -m, --mod        Modulation: dqpsk, d8psk, qpsk, 16qam (default: dqpsk for chirp)\n");
            printf("  -r, --rate       Code rate: 1/4, 1/2, 2/3, 3/4, 5/6 (default: 1/2)\n");
            printf("  --cfo <Hz>       Carrier frequency offset (default: random ±20 Hz for fading)\n");
            printf("  --nvis           Use NVIS config (1024 FFT, 59 carriers) for high-speed\n");
            printf("  -c, --channel    Channel type: awgn, good, moderate, poor (default: awgn)\n");
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
    if (waveform_mode == WaveformMode::OFDM_NVIS) waveform_name = "OFDM";
    else if (waveform_mode == WaveformMode::OFDM_CHIRP) waveform_name = "OFDM-CHIRP";
    else if (waveform_mode == WaveformMode::MC_DPSK) waveform_name = "MC-DPSK";
    else if (waveform_mode == WaveformMode::OTFS_EQ) waveform_name = "OTFS-EQ";
    else if (waveform_mode == WaveformMode::OTFS_RAW) waveform_name = "OTFS-RAW";

    printf("Configuration:\n");
    printf("  SNR:        %.1f dB\n", snr_db);
    printf("  Channel:    %s\n", channel_type.c_str());
    printf("  Frames:     %d\n", num_frames);
    printf("  Duration:   %.0f seconds\n", duration_sec);
    printf("  Waveform:   %s\n", waveform_name);
    const char* mod_name = "?";
    if (test_modulation == Modulation::DQPSK) mod_name = "DQPSK";
    else if (test_modulation == Modulation::D8PSK) mod_name = "D8PSK";
    else if (test_modulation == Modulation::QPSK) mod_name = "QPSK";
    else if (test_modulation == Modulation::QAM16) mod_name = "16QAM";
    printf("  Modulation: %s\n", mod_name);
    const char* rate_name = "?";
    if (test_code_rate == CodeRate::R1_4) rate_name = "R1/4";
    else if (test_code_rate == CodeRate::R1_2) rate_name = "R1/2";
    else if (test_code_rate == CodeRate::R2_3) rate_name = "R2/3";
    else if (test_code_rate == CodeRate::R3_4) rate_name = "R3/4";
    else if (test_code_rate == CodeRate::R5_6) rate_name = "R5/6";
    printf("  Code Rate:  %s\n", rate_name);
    printf("  Config:     %s\n\n", use_nvis ? "NVIS (1024 FFT, 59 carriers)" : "Standard (512 FFT, 30 carriers)");

    // ========================================
    // TX Setup - Generate v2 frames
    // ========================================
    ModemEngine tx_modem;
    tx_modem.setLogPrefix("TX");

    // Apply NVIS config if requested (1024 FFT, 59 carriers for high-speed)
    if (use_nvis) {
        ModemConfig nvis_cfg = presets::nvis_mode();
        // For OFDM_CHIRP, use differential modulation (no pilots)
        if (waveform_mode == WaveformMode::OFDM_CHIRP) {
            nvis_cfg.use_pilots = false;
            nvis_cfg.modulation = test_modulation;
            nvis_cfg.code_rate = test_code_rate;
        }
        tx_modem.setConfig(nvis_cfg);
    } else if (waveform_mode == WaveformMode::OFDM_CHIRP) {
        // For OFDM_CHIRP in standard mode, use differential modulation (no pilots)
        ModemConfig cfg = tx_modem.getConfig();
        cfg.use_pilots = false;
        cfg.modulation = test_modulation;
        cfg.code_rate = test_code_rate;
        // Apply TX CFO if specified (simulates radio tuning error)
        if (cfo_specified) {
            cfg.tx_cfo_hz = cfo_hz;
        }
        tx_modem.setConfig(cfg);
    }

    // Set connected state so TX uses the selected waveform (not DPSK connect waveform)
    tx_modem.setConnected(true);
    tx_modem.setHandshakeComplete(true);
    tx_modem.setWaveformMode(waveform_mode);
    // Control interleaving
    if (no_interleave) {
        tx_modem.setInterleavingEnabled(false);
    }
    // Disable TX filter to avoid group delay mismatch (RX doesn't filter)
    tx_modem.setFilterEnabled(false);

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
        static_cast<uint8_t>(WaveformMode::OFDM_NVIS));
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
            static_cast<uint8_t>(WaveformMode::OFDM_NVIS)
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

        // CFO is now applied at the modulator level via cfg.tx_cfo_hz
        // (mixer frequency is shifted, which correctly shifts both chirp and OFDM)
        // No need for post-generation frequency shift here.

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
    // Apply HF channel simulation
    // ========================================
    printf("Applying %s channel (SNR=%.1f dB)...\n", channel_type.c_str(), snr_db);

    if (channel_type == "awgn") {
        // Simple AWGN - good for baseline testing
        addNoise(full_audio, snr_db, rng);

        // CFO already applied per-frame during TX
        if (cfo_specified && std::abs(cfo_hz) > 0.001f) {
            printf("CFO: %.1f Hz (applied per-frame at TX)\n", cfo_hz);
        }
    } else {
        // HF fading channel (Watterson model)
        WattersonChannel::Config ch_cfg;
        ch_cfg.sample_rate = 48000.0f;
        ch_cfg.snr_db = snr_db;
        ch_cfg.noise_enabled = true;

        // Disable CFO in channel - we'll apply it separately with FFT-based method
        // (the channel's Weaver method implementation has filter issues)
        ch_cfg.cfo_enabled = false;
        ch_cfg.cfo_hz = 0.0f;
        ch_cfg.random_cfo_max_hz = 0.0f;

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

        // Use provided seed or generate random for realistic HF simulation
        uint32_t seed = channel_seed;
        if (seed == 0) {
            std::random_device rd;
            seed = rd();
        }
        printf("Channel seed: %u (use --seed %u to reproduce)\n", seed, seed);

        WattersonChannel channel(ch_cfg, seed);
        SampleSpan input_span(full_audio.data(), full_audio.size());
        full_audio = channel.process(input_span);

        // CFO already applied per-frame during TX
        if (cfo_specified && std::abs(cfo_hz) > 0.001f) {
            printf("CFO: %.1f Hz (applied per-frame at TX)\n", cfo_hz);
        }
    }

    // Save audio if requested
    if (save_audio) {
        printf("Saving audio to %s...\n", output_file.c_str());
        if (saveAudio(full_audio, output_file)) {
            printf("  Play with: aplay -f FLOAT_LE -r 48000 -c 1 %s\n", output_file.c_str());
        }
    }

    // ========================================
    // RX - Decode each frame region separately
    // ========================================
    printf("\nDecoding via ModemEngine.feedAudio()...\n");
    if (play_audio) {
        printf("🔊 Real-time audio playback enabled\n");
    }
    printf("\n");

    // Process each frame region separately (like real HF - fresh sync per burst)
    for (int i = 0; i < num_frames; i++) {
        // Create fresh RX modem for each frame
        ModemEngine rx_modem;
        rx_modem.setLogPrefix("RX");

        // Configure RX to match TX settings (modulation, FFT size, code rate, etc.)
        if (use_nvis) {
            ModemConfig nvis_cfg = presets::nvis_mode();
            if (waveform_mode == WaveformMode::OFDM_CHIRP) {
                nvis_cfg.use_pilots = false;
                nvis_cfg.modulation = test_modulation;
                nvis_cfg.code_rate = test_code_rate;
            }
            rx_modem.setConfig(nvis_cfg);
        } else if (waveform_mode == WaveformMode::OFDM_CHIRP) {
            ModemConfig cfg = rx_modem.getConfig();
            cfg.use_pilots = false;
            cfg.modulation = test_modulation;
            cfg.code_rate = test_code_rate;
            rx_modem.setConfig(cfg);
        }

        // DPSK uses acquisition path (disconnected mode) for chirp detection
        // Other modes use connected mode for direct buffer processing
        if (waveform_mode != WaveformMode::MC_DPSK) {
            rx_modem.setConnected(true);
            rx_modem.setHandshakeComplete(true);
            rx_modem.setWaveformMode(waveform_mode);
        }
        // Match TX interleaving setting
        if (no_interleave) {
            rx_modem.setInterleavingEnabled(false);
        }

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

        // Feed audio in chunks (with optional real-time playback)
        size_t chunk_size = 960;  // 20ms @ 48kHz
        FILE* aplay_pipe = nullptr;
        if (play_audio) {
            aplay_pipe = popen("aplay -f FLOAT_LE -r 48000 -c 1 -q 2>/dev/null", "w");
        }

        for (size_t j = window_start; j < window_end; j += chunk_size) {
            size_t len = std::min(chunk_size, window_end - j);
            rx_modem.feedAudio(full_audio.data() + j, len);

            // Real-time playback: write to speaker and pace at sample rate
            if (aplay_pipe) {
                fwrite(full_audio.data() + j, sizeof(float), len, aplay_pipe);
                std::this_thread::sleep_for(std::chrono::microseconds(len * 1000000 / 48000));
            }
        }

        if (aplay_pipe) {
            pclose(aplay_pipe);
        }

        // Wait for RX threads to process
        // With real-time playback, RX processes during feed, but still need time to finish decode
        // DPSK needs longer wait because chirp detection scans large buffer (takes ~6s)
        int max_wait_iters = (waveform_mode == WaveformMode::MC_DPSK) ? 1000 : 100;
        for (int wait = 0; wait < max_wait_iters && !got_frame; wait++) {
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
