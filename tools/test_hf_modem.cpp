/**
 * ═══════════════════════════════════════════════════════════════════════════════
 *                    HF MODEM PIPELINE TEST - IWaveform + RxPipeline
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * PURPOSE:
 *   Tests the full audio pipeline using the new IWaveform interface and
 *   RxPipeline streaming decoder, simulating real HF rig conditions.
 *
 * INTERFACE ARCHITECTURE:
 *   ┌─────────────────────────────────────────────────────────────────────────┐
 *   │  IWaveform (Abstract Interface)                                         │
 *   │  - Common API for all waveform types                                    │
 *   │  - TX: generatePreamble() + modulate(encoded_data)                      │
 *   │  - RX: detectSync() + process() + getSoftBits()                         │
 *   ├─────────────────────────────────────────────────────────────────────────┤
 *   │  Implementations:                                                       │
 *   │  - OFDMChirpWaveform: Chirp sync + OFDM (10-17 dB)                      │
 *   │  - MCDPSKWaveform:    Chirp sync + Multi-Carrier DPSK (-3 to 10 dB)    │
 *   │  - OFDMCoxWaveform:   Schmidl-Cox + OFDM (17+ dB)                       │
 *   └─────────────────────────────────────────────────────────────────────────┘
 *
 *   ┌─────────────────────────────────────────────────────────────────────────┐
 *   │  RxPipeline (Streaming Decoder)                                         │
 *   │  - feedAudio(): Accept continuous audio stream                          │
 *   │  - Buffer management with automatic sync detection                      │
 *   │  - Calls IWaveform::detectSync() → process() → getSoftBits()           │
 *   │  - LDPC decode + frame parsing                                          │
 *   │  - hasFrame() / getFrame(): Retrieve decoded frames                     │
 *   └─────────────────────────────────────────────────────────────────────────┘
 *
 * SIGNAL GENERATION CHAIN:
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  1. FRAME CREATION                                                    │
 *   │     - Create v2::ConnectFrame with callsigns, capabilities            │
 *   │     - Serialize to bytes (header + payload)                           │
 *   │     - Frame format: [MAGIC][TYPE][LEN][SEQ][SRC][DST][CAPS][CRC]     │
 *   └──────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                                    ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  2. LDPC ENCODING                                                     │
 *   │     - Rate 1/4, 1/2, 2/3, 3/4, or 5/6                                │
 *   │     - 648-bit codewords                                               │
 *   │     - Channel interleaving (spreads burst errors)                     │
 *   └──────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                                    ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  3. TX MODULATION (via IWaveform)                                     │
 *   │                                                                       │
 *   │     A. generatePreamble() - Waveform-specific:                        │
 *   │        * OFDM_CHIRP: [DUAL_CHIRP 1.2s][OFDM_TRAINING 2 symbols]      │
 *   │        * MC_DPSK:    [DUAL_CHIRP 1.2s][DPSK_TRAINING][REF_SYMBOL]    │
 *   │        * OFDM_COX:   [STS][LTS] (Schmidl-Cox training)               │
 *   │                                                                       │
 *   │     B. modulate(encoded_bits) - Data symbols:                         │
 *   │        * OFDM: IFFT-based, 30 carriers at 86 baud                    │
 *   │        * DPSK: Multi-carrier differential PSK                        │
 *   │        * Real-valued output at 1500 Hz center (300-2700 Hz band)     │
 *   └──────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                                    ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  4. CHANNEL SIMULATION                                                │
 *   │                                                                       │
 *   │     A. CFO SIMULATION (Carrier Frequency Offset):                    │
 *   │        Method: Hilbert transform + complex rotation                   │
 *   │        1. Create analytic signal: x_a(t) = x(t) + j*H{x(t)}          │
 *   │        2. Multiply by e^{j*2π*CFO*t}                                 │
 *   │        3. Take real part → frequency-shifted signal                   │
 *   │        Why Hilbert: Real multiplication with cos() creates images    │
 *   │        at ±CFO. Analytic signal avoids this.                         │
 *   │                                                                       │
 *   │     B. HF FADING (Watterson model):                                   │
 *   │        - Two independent Rayleigh fading paths                       │
 *   │        - Path 1: Direct (gain g1)                                    │
 *   │        - Path 2: Delayed by τ ms (gain g2)                           │
 *   │        - Each path has Doppler spread fd Hz                          │
 *   │        - Transfer: H(f,t) = g1*h1(t) + g2*h2(t)*exp(-j2πfτ)         │
 *   │        Good HF:     τ=0.5ms, fd=0.2Hz, g1=0.9, g2=0.4                │
 *   │        Moderate HF: τ=1.0ms, fd=0.5Hz, g1=g2=0.707                   │
 *   │        Poor HF:     τ=2.0ms, fd=1.0Hz, g1=0.6, g2=0.8                │
 *   │                                                                       │
 *   │     C. AWGN:                                                          │
 *   │        - SNR calibrated from SIGNAL power (not total audio power)    │
 *   │        - Signal power from active samples (excludes silence)          │
 *   │        - Noise σ² = P_signal / 10^(SNR_dB/10)                        │
 *   └──────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                                    ▼
 *   ┌──────────────────────────────────────────────────────────────────────┐
 *   │  5. RX DEMODULATION (via RxPipeline + IWaveform)                      │
 *   │                                                                       │
 *   │     A. RxPipeline.feedAudio() - Streaming input:                      │
 *   │        - Accumulates samples in internal buffer                       │
 *   │        - Triggers sync detection when enough samples                  │
 *   │                                                                       │
 *   │     B. IWaveform.detectSync() - Find preamble:                        │
 *   │        - Complex correlation for CFO tolerance                        │
 *   │        - Dual chirp → CFO estimate ±1 Hz accuracy                    │
 *   │        - Returns: SyncResult{detected, start_sample, cfo_hz}         │
 *   │                                                                       │
 *   │     C. IWaveform.process(samples) - Demodulate:                       │
 *   │        - OFDM: FFT, channel equalization, symbol demapping           │
 *   │        - DPSK: Multi-carrier differential decode                     │
 *   │        - Extracts soft bits for FEC decoder                           │
 *   │                                                                       │
 *   │     D. RxPipeline.decodeFrame() - FEC + Parse:                        │
 *   │        - Soft bits → LDPC decoder → frame bytes                       │
 *   │        - Parse v2 frame structure                                     │
 *   │        - Queue result for retrieval via getFrame()                    │
 *   └──────────────────────────────────────────────────────────────────────┘
 *
 * USAGE:
 *   ./test_hf_modem --snr 20 -w chirp --channel moderate --cfo 30
 *   ./test_hf_modem --snr 10 -w dpsk --channel poor
 *   ./test_hf_modem --snr 25 -w ofdm --channel awgn
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 */

#include "waveform/waveform_interface.hpp"
#include "waveform/waveform_factory.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "gui/modem/rx_pipeline.hpp"
#include "protocol/frame_v2.hpp"
#include "sim/hf_channel.hpp"
#include "ultra/logging.hpp"
#include "ultra/dsp.hpp"
#include "ultra/fec.hpp"
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
#include <memory>
#include <algorithm>

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::sim;
namespace v2 = ultra::protocol::v2;
using ultra::protocol::WaveformMode;

// ═══════════════════════════════════════════════════════════════════════════════
//                          CHANNEL SIMULATION FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Add AWGN noise calibrated to target SNR.
 *
 * Signal power is calculated ONLY from active (non-silent) samples to ensure
 * accurate SNR even when the audio has silence gaps between frames.
 */
void addNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
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

/**
 * Apply CFO using Hilbert transform for proper frequency shifting.
 * Avoids image frequencies that would result from simple sinusoid multiplication.
 */
void applyCFO_TimeDomain(std::vector<float>& samples, float cfo_hz, float sample_rate = 48000.0f) {
    if (samples.size() < 64 || std::abs(cfo_hz) < 0.001f) return;

    size_t N = samples.size();
    printf("[CFO] Applying %.1f Hz shift to %zu samples (%.1f sec)\n", cfo_hz, N, N/sample_rate);

    constexpr size_t chunk_size = 65536;
    HilbertTransform hilbert(255);

    float phase = 0.0f;
    float phase_inc = 2.0f * static_cast<float>(M_PI) * cfo_hz / sample_rate;

    for (size_t offset = 0; offset < N; offset += chunk_size) {
        size_t len = std::min(chunk_size, N - offset);

        SampleSpan span(samples.data() + offset, len);
        auto analytic = hilbert.process(span);

        for (size_t i = 0; i < len; i++) {
            Complex rot(std::cos(phase), std::sin(phase));
            samples[offset + i] = std::real(analytic[i] * rot);
            phase += phase_inc;
            while (phase > static_cast<float>(M_PI)) phase -= 2.0f * static_cast<float>(M_PI);
        }
    }
}

/**
 * Save audio samples to raw float32 file.
 * Format: Little-endian 32-bit float, mono, 48 kHz
 * Play with: aplay -f FLOAT_LE -r 48000 -c 1 <file>
 */
bool saveAudio(const std::vector<float>& audio, const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;
    file.write(reinterpret_cast<const char*>(audio.data()), audio.size() * sizeof(float));
    return file.good();
}

// ═══════════════════════════════════════════════════════════════════════════════
//                     TX FRAME GENERATION VIA IWaveform
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * Generate a complete TX frame using IWaveform interface.
 *
 * Signal structure (OFDM_CHIRP example):
 *   [DUAL_CHIRP ~1.2s][OFDM_TRAINING 2 sym][OFDM_DATA N sym]
 *
 * @param waveform      IWaveform instance for TX
 * @param encoder       LDPC encoder
 * @param interleaver   Channel interleaver (nullptr to disable)
 * @param frame_bytes   Raw frame data to transmit
 * @return              Audio samples (float, 48 kHz)
 */
Samples generateTxFrame(IWaveform* waveform,
                        LDPCEncoder& encoder,
                        ChannelInterleaver* interleaver,
                        const Bytes& frame_bytes) {
    // 1. LDPC encode (returns packed bytes, not bits)
    ByteSpan input_span(frame_bytes.data(), frame_bytes.size());
    Bytes encoded_bytes = encoder.encode(input_span);

    // 2. Interleave if enabled
    if (interleaver) {
        encoded_bytes = interleaver->interleave(encoded_bytes);
    }

    // 3. Generate preamble + data
    Samples preamble = waveform->generatePreamble();
    Samples data = waveform->modulate(encoded_bytes);

    // 4. Combine
    Samples output;
    output.reserve(preamble.size() + data.size());
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), data.begin(), data.end());

    return output;
}

// ═══════════════════════════════════════════════════════════════════════════════
//                                    MAIN
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    setLogLevel(LogLevel::WARN);

    // Configuration defaults
    float snr_db = 20.0f;
    int num_frames = 10;
    float duration_sec = 30.0f;
    bool verbose = false;
    bool save_audio_file = false;
    bool play_audio = false;
    std::string output_file = "hf_modem_test.f32";
    WaveformMode waveform_mode = WaveformMode::OFDM_CHIRP;
    std::string channel_type = "awgn";
    uint32_t channel_seed = 0;
    bool no_interleave = false;
    Modulation test_modulation = Modulation::DQPSK;
    CodeRate test_code_rate = CodeRate::R1_2;
    float cfo_hz = 0.0f;
    bool cfo_specified = false;

    // Argument parsing
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--snr" && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (arg == "--frames" && i + 1 < argc) {
            num_frames = std::stoi(argv[++i]);
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stof(argv[++i]);
        } else if (arg == "--save") {
            save_audio_file = true;
        } else if (arg == "-o" && i + 1 < argc) {
            output_file = argv[++i];
            save_audio_file = true;
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
            setLogLevel(LogLevel::DEBUG);
        } else if (arg == "--play" || arg == "-p") {
            play_audio = true;
        } else if ((arg == "-w" || arg == "--waveform") && i + 1 < argc) {
            std::string mode = argv[++i];
            if (mode == "ofdm" || mode == "cox" || mode == "ofdm_cox") {
                waveform_mode = WaveformMode::OFDM_COX;
            } else if (mode == "dpsk" || mode == "mc_dpsk") {
                waveform_mode = WaveformMode::MC_DPSK;
            } else if (mode == "chirp" || mode == "ofdm_chirp") {
                waveform_mode = WaveformMode::OFDM_CHIRP;
            } else {
                fprintf(stderr, "Unknown waveform: %s (use: ofdm, chirp, dpsk)\n", mode.c_str());
                return 1;
            }
        } else if ((arg == "-c" || arg == "--channel") && i + 1 < argc) {
            channel_type = argv[++i];
            if (channel_type != "awgn" && channel_type != "good" &&
                channel_type != "moderate" && channel_type != "poor") {
                fprintf(stderr, "Unknown channel: %s (use: awgn, good, moderate, poor)\n", channel_type.c_str());
                return 1;
            }
        } else if (arg == "--seed" && i + 1 < argc) {
            channel_seed = std::stoul(argv[++i]);
        } else if (arg == "--no-interleave") {
            no_interleave = true;
        } else if ((arg == "-m" || arg == "--mod") && i + 1 < argc) {
            std::string mod = argv[++i];
            if (mod == "dqpsk") test_modulation = Modulation::DQPSK;
            else if (mod == "d8psk") test_modulation = Modulation::D8PSK;
            else if (mod == "qpsk") test_modulation = Modulation::QPSK;
            else if (mod == "16qam") test_modulation = Modulation::QAM16;
            else {
                fprintf(stderr, "Unknown modulation: %s\n", mod.c_str());
                return 1;
            }
        } else if ((arg == "-r" || arg == "--rate") && i + 1 < argc) {
            std::string rate = argv[++i];
            if (rate == "1/4" || rate == "r1/4") test_code_rate = CodeRate::R1_4;
            else if (rate == "1/2" || rate == "r1/2") test_code_rate = CodeRate::R1_2;
            else if (rate == "2/3" || rate == "r2/3") test_code_rate = CodeRate::R2_3;
            else if (rate == "3/4" || rate == "r3/4") test_code_rate = CodeRate::R3_4;
            else {
                fprintf(stderr, "Unknown code rate: %s\n", rate.c_str());
                return 1;
            }
        } else if (arg == "--cfo" && i + 1 < argc) {
            cfo_hz = std::stof(argv[++i]);
            cfo_specified = true;
        } else if (arg == "-h" || arg == "--help") {
            printf("HF Modem Pipeline Test (IWaveform + RxPipeline)\n\n");
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --snr <dB>       SNR level (default: 20)\n");
            printf("  --frames <n>     Number of frames (default: 10)\n");
            printf("  -w, --waveform   Waveform: ofdm, chirp, dpsk (default: chirp)\n");
            printf("  -m, --mod        Modulation: dqpsk, d8psk, qpsk, 16qam\n");
            printf("  -r, --rate       Code rate: 1/4, 1/2, 2/3, 3/4\n");
            printf("  --cfo <Hz>       Carrier frequency offset\n");
            printf("  -c, --channel    Channel: awgn, good, moderate, poor\n");
            printf("  -p, --play       Play audio through speakers\n");
            printf("  --save           Save audio file\n");
            printf("  -o <file>        Output file (implies --save)\n");
            printf("  -v, --verbose    Verbose output\n");
            return 0;
        }
    }

    std::mt19937 rng(42);

    // ═══════════════════════════════════════════════════════════════════════════
    // PRINT CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║       HF MODEM PIPELINE TEST - IWaveform + RxPipeline        ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    const char* waveform_name = "?";
    if (waveform_mode == WaveformMode::OFDM_COX) waveform_name = "OFDM-COX";
    else if (waveform_mode == WaveformMode::OFDM_CHIRP) waveform_name = "OFDM-CHIRP";
    else if (waveform_mode == WaveformMode::MC_DPSK) waveform_name = "MC-DPSK";

    printf("Configuration:\n");
    printf("  Waveform:   %s\n", waveform_name);
    printf("  Modulation: %s\n", modulationToString(test_modulation));
    printf("  Code Rate:  %s\n", codeRateToString(test_code_rate));
    printf("  SNR:        %.1f dB\n", snr_db);
    printf("  Channel:    %s\n", channel_type.c_str());
    if (cfo_specified) printf("  CFO:        %.1f Hz\n", cfo_hz);
    printf("  Frames:     %d\n", num_frames);
    printf("  Interleave: %s\n\n", no_interleave ? "disabled" : "enabled");

    // ═══════════════════════════════════════════════════════════════════════════
    // CREATE TX WAVEFORM + ENCODER
    // ═══════════════════════════════════════════════════════════════════════════
    ModemConfig config = presets::balanced();
    config.modulation = test_modulation;
    config.code_rate = test_code_rate;
    config.use_pilots = false;  // Differential modes don't need pilots
    // NOTE: CFO is applied to the FULL signal via Hilbert transform after generation
    // (not via setTxFrequencyOffset which only affects chirp preamble)

    // Create TX waveform via factory
    auto tx_waveform = WaveformFactory::create(waveform_mode, config);
    if (!tx_waveform) {
        fprintf(stderr, "Failed to create TX waveform\n");
        return 1;
    }
    tx_waveform->configure(test_modulation, test_code_rate);

    // Create encoder and interleaver
    LDPCEncoder encoder(test_code_rate);

    // Bits per symbol for interleaver: carriers × bits_per_carrier
    int bits_per_carrier = 2;  // DQPSK default
    if (test_modulation == Modulation::D8PSK) bits_per_carrier = 3;
    else if (test_modulation == Modulation::QAM16) bits_per_carrier = 4;
    size_t bits_per_symbol = tx_waveform->getCarrierCount() * bits_per_carrier;

    std::unique_ptr<ChannelInterleaver> tx_interleaver;
    if (!no_interleave) {
        tx_interleaver = std::make_unique<ChannelInterleaver>(bits_per_symbol);
    }

    // Generate test frame to measure duration
    v2::ConnectFrame test_frame = v2::ConnectFrame::makeConnect(
        "TEST0", "DEST", protocol::ModeCapabilities::ALL,
        static_cast<uint8_t>(waveform_mode));
    auto test_audio = generateTxFrame(tx_waveform.get(), encoder,
                                       tx_interleaver.get(), test_frame.serialize());
    float frame_duration_sec = test_audio.size() / 48000.0f;

    // Calculate minimum duration
    float min_gap_sec = 2.0f;
    float min_duration = (frame_duration_sec + min_gap_sec) * num_frames + 5.0f;
    if (duration_sec < min_duration) {
        duration_sec = min_duration;
    }

    fprintf(stderr, "Frame duration: %.1f s (preamble: %d samples, data: %zu samples)\n",
           frame_duration_sec, tx_waveform->getPreambleSamples(),
           test_audio.size() - tx_waveform->getPreambleSamples());
    fprintf(stderr, "Total duration: %.0f s\n\n", duration_sec);

    // ═══════════════════════════════════════════════════════════════════════════
    // GENERATE ALL FRAMES INTO AUDIO BUFFER
    // ═══════════════════════════════════════════════════════════════════════════
    struct TestFrame {
        uint16_t seq;
        std::string src;
        size_t audio_start;
        size_t audio_len;
        std::atomic<bool> decoded{false};
    };
    std::vector<TestFrame> frames(num_frames);

    size_t total_samples = static_cast<size_t>(duration_sec * 48000);
    std::vector<float> full_audio(total_samples, 0.0f);

    float avg_gap = (duration_sec - frame_duration_sec * num_frames) / (num_frames + 1);
    std::uniform_real_distribution<float> first_delay(1.0f, 3.0f);
    float current_time = first_delay(rng);

    fprintf(stderr, "Generating %d frames via IWaveform...\n", num_frames);

    for (int i = 0; i < num_frames; i++) {
        v2::ConnectFrame frame = v2::ConnectFrame::makeConnect(
            "TEST" + std::to_string(i), "DEST",
            protocol::ModeCapabilities::ALL,
            static_cast<uint8_t>(waveform_mode));
        frame.seq = static_cast<uint16_t>(i + 1);

        frames[i].seq = frame.seq;
        frames[i].src = frame.getSrcCallsign();
        frames[i].audio_start = static_cast<size_t>(current_time * 48000);

        auto tx_audio = generateTxFrame(tx_waveform.get(), encoder,
                                         tx_interleaver.get(), frame.serialize());
        frames[i].audio_len = tx_audio.size();

        fprintf(stderr, "  Frame %2d at %.1fs (pos=%zu): seq=%d src=%s (%zu samples)\n",
               i + 1, current_time, frames[i].audio_start, frame.seq, frames[i].src.c_str(), tx_audio.size());

        size_t pos = frames[i].audio_start;
        for (size_t j = 0; j < tx_audio.size() && pos + j < full_audio.size(); j++) {
            full_audio[pos + j] = tx_audio[j];
        }

        std::uniform_real_distribution<float> gap_var(0.8f, 1.2f);
        current_time += frame_duration_sec + avg_gap * gap_var(rng);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // APPLY CFO (must be done BEFORE noise/fading, affects entire signal)
    // ═══════════════════════════════════════════════════════════════════════════
    if (cfo_specified && std::abs(cfo_hz) > 0.1f) {
        printf("Applying CFO=%.1f Hz to full signal...\n", cfo_hz);
        applyCFO_TimeDomain(full_audio, cfo_hz);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // APPLY CHANNEL (noise and fading)
    // ═══════════════════════════════════════════════════════════════════════════
    printf("Applying %s channel (SNR=%.1f dB)...\n", channel_type.c_str(), snr_db);

    if (channel_type == "awgn") {
        addNoise(full_audio, snr_db, rng);
    } else {
        WattersonChannel::Config ch_cfg;
        ch_cfg.sample_rate = 48000.0f;
        ch_cfg.snr_db = snr_db;
        ch_cfg.noise_enabled = true;
        ch_cfg.cfo_enabled = false;  // CFO applied at TX level

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

        uint32_t seed = channel_seed ? channel_seed : std::random_device{}();
        printf("Channel seed: %u\n", seed);

        WattersonChannel channel(ch_cfg, seed);
        SampleSpan input_span(full_audio.data(), full_audio.size());
        full_audio = channel.process(input_span);
    }

    if (save_audio_file) {
        printf("Saving audio to %s...\n", output_file.c_str());
        saveAudio(full_audio, output_file);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // RX DECODE VIA RxPipeline + IWaveform
    // ═══════════════════════════════════════════════════════════════════════════
    // IMPORTANT: Use a SINGLE RxPipeline for the entire audio stream!
    // This simulates a real receiver that doesn't know where frames are.
    // The pipeline must detect sync, decode, and find all frames autonomously.
    // ═══════════════════════════════════════════════════════════════════════════
    printf("\nDecoding via RxPipeline + IWaveform (single receiver)...\n");

    // Create ONE RX waveform + pipeline for the entire audio stream
    auto rx_waveform = WaveformFactory::create(waveform_mode, config);
    if (!rx_waveform) {
        fprintf(stderr, "Failed to create RX waveform\n");
        return 1;
    }
    rx_waveform->configure(test_modulation, test_code_rate);

    RxPipeline rx_pipeline("RX");
    rx_pipeline.setWaveform(rx_waveform.get());
    rx_pipeline.setInterleavingEnabled(!no_interleave);
    rx_pipeline.setInterleaverConfig(bits_per_symbol);
    rx_pipeline.setDataMode(test_code_rate, true);

    // Track received frames by sequence number
    std::mutex rx_mutex;
    std::vector<uint16_t> received_seqs;

    rx_pipeline.setFrameCallback([&](const Bytes& data, v2::FrameType type) {
        if (data.size() >= 2 && data[0] == 0x55 && data[1] == 0x4C) {
            auto parsed = v2::ConnectFrame::deserialize(data);
            if (parsed) {
                std::lock_guard<std::mutex> lock(rx_mutex);
                received_seqs.push_back(parsed->seq);
                if (verbose) {
                    printf("    [RX] Frame decoded: seq=%d src=%s\n",
                           parsed->seq, parsed->getSrcCallsign().c_str());
                }
            }
        }
    });

    // Feed ENTIRE audio stream through the pipeline in chunks
    // This is how a real receiver works - continuous audio input
    size_t chunk_size = 960;  // 20ms chunks (typical audio callback size)
    FILE* aplay_pipe = nullptr;
    if (play_audio) {
        aplay_pipe = popen("aplay -f FLOAT_LE -r 48000 -c 1 -q 2>/dev/null", "w");
    }

    printf("Feeding %.1f seconds of audio through RxPipeline...\n", full_audio.size() / 48000.0f);

    for (size_t j = 0; j < full_audio.size(); j += chunk_size) {
        size_t len = std::min(chunk_size, full_audio.size() - j);
        rx_pipeline.feedAudio(full_audio.data() + j, len);

        if (aplay_pipe) {
            fwrite(full_audio.data() + j, sizeof(float), len, aplay_pipe);
            // Real-time playback pacing
            std::this_thread::sleep_for(std::chrono::microseconds(len * 1000000 / 48000));
        }

        // Also poll for frames (in addition to callback)
        while (rx_pipeline.hasFrame()) {
            auto result = rx_pipeline.getFrame();
            if (result.success && result.frame_data.size() >= 2) {
                if (result.frame_data[0] == 0x55 && result.frame_data[1] == 0x4C) {
                    auto parsed = v2::ConnectFrame::deserialize(result.frame_data);
                    if (parsed) {
                        std::lock_guard<std::mutex> lock(rx_mutex);
                        // Avoid duplicates (callback might have already added it)
                        if (std::find(received_seqs.begin(), received_seqs.end(), parsed->seq) == received_seqs.end()) {
                            received_seqs.push_back(parsed->seq);
                            if (verbose) {
                                printf("    [RX] Frame via poll: seq=%d SNR=%.1f dB CFO=%.1f Hz\n",
                                       parsed->seq, result.snr_estimate, result.cfo_estimate);
                            }
                        }
                    }
                }
            }
        }
    }

    if (aplay_pipe) pclose(aplay_pipe);

    // Give pipeline time to finish processing any pending data
    for (int wait = 0; wait < 100; wait++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        while (rx_pipeline.hasFrame()) {
            auto result = rx_pipeline.getFrame();
            if (result.success && result.frame_data.size() >= 2) {
                if (result.frame_data[0] == 0x55 && result.frame_data[1] == 0x4C) {
                    auto parsed = v2::ConnectFrame::deserialize(result.frame_data);
                    if (parsed) {
                        std::lock_guard<std::mutex> lock(rx_mutex);
                        if (std::find(received_seqs.begin(), received_seqs.end(), parsed->seq) == received_seqs.end()) {
                            received_seqs.push_back(parsed->seq);
                        }
                    }
                }
            }
        }
    }

    // Match received frames to expected frames
    for (int i = 0; i < num_frames; i++) {
        std::lock_guard<std::mutex> lock(rx_mutex);
        if (std::find(received_seqs.begin(), received_seqs.end(), frames[i].seq) != received_seqs.end()) {
            frames[i].decoded = true;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // RESULTS
    // ═══════════════════════════════════════════════════════════════════════════
    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("                         RESULTS\n");
    printf("═══════════════════════════════════════════════════════════════\n\n");

    int success = 0;
    for (int i = 0; i < num_frames; i++) {
        const char* status = frames[i].decoded ? "\033[32m✓\033[0m" : "\033[31m✗\033[0m";
        printf("  %s Frame %2d at %.1fs: seq=%d src=%s\n",
               status, i + 1, frames[i].audio_start / 48000.0f,
               frames[i].seq, frames[i].src.c_str());
        if (frames[i].decoded) success++;
    }

    printf("\n═══════════════════════════════════════════════════════════════\n");
    printf("  Waveform: %s @ %s %s\n", waveform_name,
           modulationToString(test_modulation), codeRateToString(test_code_rate));
    printf("  Channel:  %s @ %.1f dB\n", channel_type.c_str(), snr_db);
    if (cfo_specified) printf("  CFO:      %.1f Hz\n", cfo_hz);
    printf("  Decoded:  %d/%d (%d%%)\n", success, num_frames,
           num_frames > 0 ? success * 100 / num_frames : 0);

    if (success == num_frames) {
        printf("\n  \033[32m★★★ PERFECT! ★★★\033[0m\n");
    } else if (success >= num_frames * 9 / 10) {
        printf("\n  \033[33m★★ Excellent ★★\033[0m\n");
    } else if (success >= num_frames * 7 / 10) {
        printf("\n  \033[33m★ Good ★\033[0m\n");
    } else {
        printf("\n  \033[31m⚠ Needs debugging ⚠\033[0m\n");
    }

    printf("\n");
    return (success == num_frames) ? 0 : 1;
}
