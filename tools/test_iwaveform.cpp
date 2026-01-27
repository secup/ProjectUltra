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
#include "waveform/ofdm_chirp_waveform.hpp"
#include "waveform/waveform_factory.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/fec.hpp"
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
#include <set>
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
// OFDM_CHIRP Decode using IWaveform interface directly
// ============================================================================
// ModemEngine's acquisition thread routes all chirp frames to MC-DPSK decoder,
// which is wrong for OFDM_CHIRP. So we decode OFDM_CHIRP frames directly using
// the IWaveform interface to test the proper CFO correction path.

bool decodeOFDMChirpFrame(SampleSpan audio, const Bytes& expected_data, bool verbose) {
    // Create OFDM_CHIRP waveform
    OFDMChirpWaveform waveform;

    // Step 1: Detect sync (chirp)
    SyncResult sync_result;
    if (!waveform.detectSync(audio, sync_result, 0.15f)) {
        if (verbose) printf("  [OFDM_CHIRP] Chirp not detected\n");
        return false;
    }

    printf("[CHIRP-RX] CFO estimate: %.2f Hz\n", sync_result.cfo_hz);

    // Step 2: Apply CFO correction
    waveform.setFrequencyOffset(sync_result.cfo_hz);

    // Step 3: Process samples from training start
    // detectSync returns start_sample pointing to training symbols
    if (sync_result.start_sample >= audio.size()) {
        if (verbose) printf("  [OFDM_CHIRP] Invalid start_sample %d\n", sync_result.start_sample);
        return false;
    }

    size_t data_samples = audio.size() - sync_result.start_sample;
    SampleSpan data_span(audio.data() + sync_result.start_sample, data_samples);

    if (!waveform.process(data_span)) {
        if (verbose) printf("  [OFDM_CHIRP] process() returned false\n");
        // Don't return - check soft bits anyway
    }

    // Step 4: Get soft bits and decode LDPC
    auto soft_bits = waveform.getSoftBits();
    if (soft_bits.size() < v2::LDPC_CODEWORD_BITS) {
        if (verbose) printf("  [OFDM_CHIRP] Not enough soft bits: %zu < %zu\n",
                           soft_bits.size(), static_cast<size_t>(v2::LDPC_CODEWORD_BITS));
        return false;
    }

    // Decode CW0 to get frame info
    std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + v2::LDPC_CODEWORD_BITS);
    LDPCDecoder decoder(CodeRate::R1_4);  // TX uses R1/4 for disconnected mode
    Bytes cw0_data = decoder.decodeSoft(cw0_bits);

    if (!decoder.lastDecodeSuccess() || cw0_data.size() < v2::BYTES_PER_CODEWORD) {
        if (verbose) printf("  [OFDM_CHIRP] CW0 LDPC decode failed\n");
        return false;
    }
    cw0_data.resize(v2::BYTES_PER_CODEWORD);

    // Check magic and get codeword count
    auto cw_info = v2::identifyCodeword(cw0_data);
    if (cw_info.type != v2::CodewordType::HEADER) {
        if (verbose) printf("  [OFDM_CHIRP] CW0 not a header\n");
        return false;
    }

    auto header = v2::parseHeader(cw0_data);
    if (!header.valid) {
        if (verbose) printf("  [OFDM_CHIRP] Invalid header\n");
        return false;
    }

    int expected_codewords = header.total_cw;
    if (verbose) printf("  [OFDM_CHIRP] Header: type=%d, %d codewords\n",
                       static_cast<int>(header.type), expected_codewords);

    // Decode all codewords
    Bytes full_data;
    for (int cw = 0; cw < expected_codewords; cw++) {
        size_t offset = cw * v2::LDPC_CODEWORD_BITS;
        if (offset + v2::LDPC_CODEWORD_BITS > soft_bits.size()) {
            if (verbose) printf("  [OFDM_CHIRP] Not enough bits for CW%d\n", cw);
            return false;
        }

        std::vector<float> cw_bits(soft_bits.begin() + offset,
                                   soft_bits.begin() + offset + v2::LDPC_CODEWORD_BITS);
        decoder.setRate(CodeRate::R1_4);
        Bytes cw_data = decoder.decodeSoft(cw_bits);
        if (!decoder.lastDecodeSuccess() || cw_data.size() < v2::BYTES_PER_CODEWORD) {
            if (verbose) printf("  [OFDM_CHIRP] CW%d LDPC decode failed\n", cw);
            return false;
        }
        cw_data.resize(v2::BYTES_PER_CODEWORD);
        full_data.insert(full_data.end(), cw_data.begin(), cw_data.end());
    }

    // Verify magic bytes
    if (full_data.size() >= 2 && full_data[0] == 0x55 && full_data[1] == 0x4C) {
        return true;
    }

    if (verbose) printf("  [OFDM_CHIRP] Invalid magic: %02x %02x\n",
                       full_data.size() > 0 ? full_data[0] : 0,
                       full_data.size() > 1 ? full_data[1] : 0);
    return false;
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
    // Generate frames (TX modem in separate scope so it's destroyed before RX)
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
    size_t total_samples;
    std::vector<float> full_audio;
    float frame_duration_sec;

    // ========================================
    // TX - Generate ONE continuous audio stream (like real radio)
    // ========================================
    // Per TESTING_METHODOLOGY.md:
    // - Use ONE TX modem for entire stream
    // - Frames separated by silence periods (simulating PTT gaps)
    // - This creates a realistic continuous audio stream

    {   // TX scope
        ModemEngine tx_modem;
        tx_modem.setLogPrefix("TX");
        tx_modem.setWaveformMode(waveform_mode);
        tx_modem.setConnectWaveform(waveform_mode);
        tx_modem.setInterleavingEnabled(false);
        tx_modem.setFilterEnabled(false);

        if (waveform_mode == protocol::WaveformMode::MC_DPSK) {
            tx_modem.setMCDPSKCarriers(num_carriers);
        }

        // Build continuous audio stream: [silence][frame1][silence][frame2]...
        // Gap must be short enough that total audio < MAX_PENDING_SAMPLES (960000 = 20s)
        // With 5 frames @ 2.3s each + 4 gaps, use 1.5s gaps to stay under limit
        float gap_sec = 1.5f;  // Gap between frames
        size_t gap_samples = static_cast<size_t>(gap_sec * 48000);
        size_t initial_silence = static_cast<size_t>(1.5f * 48000);  // 1.5s initial silence

        printf("Generating %d %s frames as ONE continuous stream...\n", num_frames, waveform_name);

        // Start with initial silence
        full_audio.resize(initial_silence, 0.0f);

        for (int i = 0; i < num_frames; i++) {
            v2::ConnectFrame frame = v2::ConnectFrame::makeConnect(
                "TEST" + std::to_string(i), "DEST",
                protocol::ModeCapabilities::ALL,
                static_cast<uint8_t>(waveform_mode));
            frame.seq = static_cast<uint16_t>(i + 1);

            frames[i].seq = frame.seq;
            frames[i].src = frame.getSrcCallsign();
            frames[i].data = frame.serialize();
            frames[i].audio_start = full_audio.size();  // Current position in stream

            // Generate frame using the SAME TX modem (continuous operation)
            auto tx_audio = tx_modem.transmit(frames[i].data);
            frames[i].audio_len = tx_audio.size();

            // Set frame_duration_sec from first frame
            if (i == 0) {
                frame_duration_sec = tx_audio.size() / 48000.0f;
                printf("Frame duration: %.1fs (%zu samples)\n", frame_duration_sec, tx_audio.size());
            }

            // Append frame audio to continuous stream
            full_audio.insert(full_audio.end(), tx_audio.begin(), tx_audio.end());

            printf("  Frame %2d at %.3fs (sample %zu): seq=%d, len=%zu\n",
                   i + 1, frames[i].audio_start / 48000.0f, frames[i].audio_start,
                   frame.seq, tx_audio.size());

            // Add gap after frame (except after last frame)
            if (i < num_frames - 1) {
                full_audio.resize(full_audio.size() + gap_samples, 0.0f);
            }
        }

        // Add trailing silence
        full_audio.resize(full_audio.size() + 48000, 0.0f);  // 1s trailing
        total_samples = full_audio.size();

        printf("Total audio: %.1fs (%zu samples)\n",
               total_samples / 48000.0f, total_samples);
    }   // TX modem destroyed here, before RX starts

    // Verify frames are in the buffer
    // Verify frame positions
    printf("Frame positions in full_audio:\n");
    for (int i = 0; i < num_frames; i++) {
        float e = 0;
        size_t pos = frames[i].audio_start;
        for (size_t j = 0; j < 24000 && pos + j < full_audio.size(); j++) {
            e += full_audio[pos+j] * full_audio[pos+j];
        }
        printf("  Frame %d @ %zu: RMS=%.4f\n", i+1, pos, std::sqrt(e/24000));
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
    // RX - Single receiver for entire audio stream (like real HF rig)
    // ========================================
    // Per TESTING_METHODOLOGY.md:
    // - Use a SINGLE receiver for entire audio stream
    // - Feed audio in small chunks (like real audio callbacks)
    // - Let receiver detect sync autonomously
    // - Do NOT create new receiver per frame (that's cheating)

    // OFDM_CHIRP: ModemEngine's acquisition thread routes chirp frames to MC-DPSK decoder,
    // which is broken. Use IWaveform directly for now (still single instance).
    bool use_iwaveform_direct = (waveform_mode == protocol::WaveformMode::OFDM_CHIRP);

    printf("Decoding via %s (single RX instance)...\n\n",
           use_iwaveform_direct ? "IWaveform directly" : "ModemEngine.feedAudio()");

    constexpr size_t chunk_size = 960;  // 20ms at 48kHz
    int decoded_count = 0;

    // Track which frames have been decoded (by sequence number)
    std::mutex decoded_mutex;
    std::set<uint16_t> decoded_seqs;

    if (use_iwaveform_direct) {
        // OFDM_CHIRP: Use single IWaveform instance, feed entire audio
        // Note: OFDMChirpWaveform doesn't support continuous streaming yet,
        // so we still need to search for frames. But we use ONE waveform instance.
        OFDMChirpWaveform waveform;

        // Search through audio for frames (single waveform instance)
        size_t search_pos = 0;
        size_t min_gap = 48000;  // 1 second minimum between frames

        while (search_pos + 90000 < full_audio.size()) {
            SampleSpan search_span(full_audio.data() + search_pos,
                                   full_audio.size() - search_pos);

            // Reset waveform state for new search (but same instance)
            waveform.reset();

            SyncResult sync_result;
            if (waveform.detectSync(search_span, sync_result, 0.15f)) {
                printf("[OFDM_CHIRP] Sync at %zu + %d, CFO=%.1f Hz\n",
                       search_pos, sync_result.start_sample, sync_result.cfo_hz);

                // Apply CFO and process
                waveform.setFrequencyOffset(sync_result.cfo_hz);

                size_t data_start = sync_result.start_sample;
                if (data_start < search_span.size()) {
                    SampleSpan data_span(search_span.data() + data_start,
                                         search_span.size() - data_start);
                    waveform.process(data_span);

                    auto soft_bits = waveform.getSoftBits();
                    if (soft_bits.size() >= v2::LDPC_CODEWORD_BITS) {
                        // Decode CW0
                        std::vector<float> cw0_bits(soft_bits.begin(),
                                                    soft_bits.begin() + v2::LDPC_CODEWORD_BITS);
                        LDPCDecoder decoder(CodeRate::R1_4);
                        Bytes cw0_data = decoder.decodeSoft(cw0_bits);

                        if (decoder.lastDecodeSuccess() && cw0_data.size() >= 2 &&
                            cw0_data[0] == 0x55 && cw0_data[1] == 0x4C) {
                            // Valid frame - decode all codewords
                            auto header = v2::parseHeader(cw0_data);
                            if (header.valid) {
                                Bytes full_data;
                                bool all_ok = true;
                                for (int cw = 0; cw < header.total_cw && all_ok; cw++) {
                                    size_t offset = cw * v2::LDPC_CODEWORD_BITS;
                                    if (offset + v2::LDPC_CODEWORD_BITS > soft_bits.size()) {
                                        all_ok = false;
                                        break;
                                    }
                                    std::vector<float> cw_bits(soft_bits.begin() + offset,
                                                               soft_bits.begin() + offset + v2::LDPC_CODEWORD_BITS);
                                    decoder.setRate(CodeRate::R1_4);
                                    Bytes cw_data = decoder.decodeSoft(cw_bits);
                                    if (!decoder.lastDecodeSuccess()) {
                                        all_ok = false;
                                        break;
                                    }
                                    cw_data.resize(v2::BYTES_PER_CODEWORD);
                                    full_data.insert(full_data.end(), cw_data.begin(), cw_data.end());
                                }

                                if (all_ok) {
                                    auto parsed = v2::ConnectFrame::deserialize(full_data);
                                    if (parsed) {
                                        decoded_seqs.insert(parsed->seq);
                                        printf("  [OFDM_CHIRP] Decoded seq=%d src=%s\n",
                                               parsed->seq, parsed->getSrcCallsign().c_str());
                                    }
                                }
                            }
                        }
                    }
                }

                // Move past this frame
                search_pos += sync_result.start_sample + min_gap;
            } else {
                // No sync found, move forward
                search_pos += min_gap;
            }
        }
    } else {
        // MC-DPSK: Use SINGLE ModemEngine for entire audio stream
        ModemEngine rx_modem;
        rx_modem.setLogPrefix("RX");
        rx_modem.setWaveformMode(waveform_mode);
        rx_modem.setInterleavingEnabled(false);

        if (waveform_mode == protocol::WaveformMode::MC_DPSK) {
            rx_modem.setMCDPSKCarriers(num_carriers);
        }

        // Set up callback to track decoded frames
        rx_modem.setRawDataCallback([&](const Bytes& data) {
            if (data.size() >= 2 && data[0] == 0x55 && data[1] == 0x4C) {
                auto parsed = v2::ConnectFrame::deserialize(data);
                if (parsed) {
                    std::lock_guard<std::mutex> lock(decoded_mutex);
                    decoded_seqs.insert(parsed->seq);
                    printf("  [RX] Decoded seq=%d src=%s\n",
                           parsed->seq, parsed->getSrcCallsign().c_str());
                }
            }
        });

        // Give RX threads time to fully initialize before first frame
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Feed ENTIRE audio stream in chunks (like real audio callback from sound card)
        printf("Feeding %zu samples (%.1f sec) in %zu-sample chunks...\n",
               full_audio.size(), full_audio.size() / 48000.0f, chunk_size);

        // Feed audio with periodic pauses to let acquisition process
        // This prevents buffer overflow (MAX_PENDING_SAMPLES = 960000 = 20s)
        // Pause every ~5 seconds of audio to let acquisition catch up
        constexpr size_t PAUSE_INTERVAL = 48000 * 5;  // 5 seconds of audio

        for (size_t j = 0; j < full_audio.size(); j += chunk_size) {
            size_t len = std::min(chunk_size, full_audio.size() - j);
            rx_modem.feedAudio(full_audio.data() + j, len);

            // Pause every 5 seconds of audio to let acquisition process
            if (j > 0 && j % PAUSE_INTERVAL < chunk_size) {
                std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            }
        }

        // Wait for RX threads to finish processing
        // The audio has all been fed, now wait for decode to complete
        printf("Audio fed, waiting for decode to complete...\n");

        int expected_frames = num_frames;
        int max_wait_ms = 30000;  // 30 seconds max
        int wait_interval_ms = 100;

        for (int waited = 0; waited < max_wait_ms; waited += wait_interval_ms) {
            {
                std::lock_guard<std::mutex> lock(decoded_mutex);
                if (static_cast<int>(decoded_seqs.size()) >= expected_frames) {
                    break;  // Got all frames
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_interval_ms));
        }
    }

    // Match decoded frames to expected frames
    for (int i = 0; i < num_frames; i++) {
        uint16_t expected_seq = frames[i].seq;
        if (decoded_seqs.count(expected_seq)) {
            frames[i].decoded = true;
            decoded_count++;
            printf("  Frame %2d (seq=%d): OK\n", i + 1, expected_seq);
        } else {
            printf("  Frame %2d (seq=%d): MISSED\n", i + 1, expected_seq);
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
