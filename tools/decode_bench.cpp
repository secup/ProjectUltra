// Decode benchmark tool — generates and decodes deterministic audio
// fixtures so AI agents and humans can measure the impact of decoder
// changes without running the full cli_simulator handshake/ARQ stack.
//
// Two modes:
//   gen   — synthesize an OFDM/MC-DPSK frame burst with optional AWGN,
//           save as 32-bit float WAV (listenable in Audacity/VLC).
//   bench — load a WAV, decode at faster-than-realtime, print the
//           DecoderProfile breakdown plus frame validation.
//
// Fixtures are committed to fixtures/ so every bench run measures
// bit-identical audio. Encoder is deterministic given a seed.

#include "io/wav_io.hpp"
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "sim/channel_calibration.hpp"
#include "sim/cli_enums.hpp"
#include "sim/hf_channel.hpp"
#include "ultra/dsp.hpp"
#include "ultra/timing_profiler.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace v2 = ultra::protocol::v2;
using ultra::Bytes;
using ultra::CodeRate;
using ultra::Modulation;
using ultra::gui::DecodeResult;
using ultra::gui::StreamingDecoder;
using ultra::gui::StreamingEncoder;
using ultra::protocol::WaveformMode;

// -------- Argument parsing -------------------------------------------------

struct Args {
    std::string mode;          // "gen" or "bench"
    std::string wav_path;
    std::string waveform = "ofdm_chirp";
    std::string code_rate = "r1_4";
    std::string modulation = "dqpsk";
    std::string channel = "awgn";
    std::string wav_format = "f32";
    uint32_t output_sample_rate = 48000;
    float snr_db = 100.0f;     // 100 = effectively no noise
    uint32_t seed = 1;
    int payload_bytes = 256;   // info payload per frame
    int num_frames = 4;        // burst size
    std::string text;          // optional readable payload (gen mode)
    std::string preamble = "chirp";  // "chirp" (full chirp+LTS) or "light" (LTS only)
    bool connected = false;    // bench mode: start decoder post-CONNECT_ACK
    bool burst_interleave = true;
    int burst_group_size = 8;
    int cw_count = 0;          // 0 = connected policy default, otherwise explicit
    bool expert_phy = false;
};

void printUsage() {
    std::cout <<
        "decode_bench --mode gen|bench --wav <path> [options]\n"
        "\n"
        "Common:\n"
        "  --waveform <ofdm_chirp|ofdm_cox|ofdm_narrow|mc_dpsk>  default: ofdm_chirp\n"
        "  --rate <r1_4|r1_2|r2_3|r3_4>                          default: r1_4\n"
        "  --mod <dqpsk>                                           default: dqpsk\n"
        "  --expert                                                Allow lab-only forced PHY modes in --mod\n"
        "  --connected           Bench post-CONNECT_ACK OFDM audio: light preamble,\n"
        "                        negotiated --rate/--mod, no startup chirp required.\n"
        "  --cw-count <N>        Fixed OFDM frame codewords (connected default follows policy)\n"
        "  --no-burst-interleave Disable connected OFDM_CHIRP burst deinterleaving.\n"
        "  --burst-group-size <N> Connected burst group size (default: 8)\n"
        "\n"
        "gen options:\n"
        "  --channel <awgn|good|moderate|poor|flutter>  default: awgn\n"
        "  --snr <db>            Channel SNR (default: 100 = no noise)\n"
        "  --wav-format <f32|pcm16>                      default: f32\n"
        "  --sample-rate <Hz>    Output WAV rate; resampled from 48 kHz (default: 48000)\n"
        "  --seed <N>            RNG seed for payload + channel (default: 1)\n"
        "  --payload <N>         Bytes of info per frame (default: 256)\n"
        "  --frames <N>          Number of frames in the burst (default: 4)\n"
        "  --text <string>       Use string as payload (repeats to fill); makes\n"
        "                        the decoded bytes human-readable. Overrides RNG payload.\n"
        "\n"
        "Examples:\n"
        "  decode_bench --mode gen --wav fixtures/ofdm_chirp_r14_snr15_awgn.wav \\\n"
        "               --rate r1_4 --snr 15 --frames 4 --seed 1\n"
        "  decode_bench --mode bench --wav fixtures/ofdm_chirp_r14_snr15_awgn.wav \\\n"
        "               --rate r1_4\n"
        "  decode_bench --mode bench --connected --wav /tmp/post_connect_r1_2.wav \\\n"
        "               --waveform ofdm_chirp --rate r1_2 --mod dqpsk\n";
}

bool envFlagEnabled(const char* name) {
    if (const char* value = std::getenv(name)) {
        const std::string v = ultra::tools::cli::normalizedToken(value);
        return v == "1" || v == "true" || v == "yes" || v == "on";
    }
    return false;
}

std::optional<Modulation> parseFixtureModulation(const Args& a) {
    const auto parsed_any = ultra::tools::cli::parseModulation(
        a.modulation,
        ultra::tools::cli::AllowAuto::No,
        ultra::tools::cli::AllowExperimentalModulation::Yes);
    if (!parsed_any) {
        std::cerr << "Unknown modulation: " << a.modulation
                  << " (use "
                  << ultra::tools::cli::modulationChoices(
                         ultra::tools::cli::AllowAuto::No,
                         a.expert_phy ? ultra::tools::cli::AllowExperimentalModulation::Yes
                                      : ultra::tools::cli::AllowExperimentalModulation::No)
                  << ")\n";
        return std::nullopt;
    }
    if (ultra::tools::cli::isExpertOnlyModulation(*parsed_any)) {
        if (!a.expert_phy) {
            std::cerr << "EXPERT PHY MODE BLOCKED: --mod " << a.modulation
                      << " is outside the replay ladder. Re-run with --expert "
                         "only for controlled lab fixtures.\n";
            return std::nullopt;
        }
        std::cerr << "EXPERT PHY MODE: forcing --mod " << a.modulation
                  << " outside the replay ladder; fixture results are lab-only.\n";
    }
    return parsed_any;
}

bool parseArgs(int argc, char** argv, Args& a) {
    a.expert_phy = envFlagEnabled("ULTRA_EXPERT_PHY");
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--expert") {
            a.expert_phy = true;
            break;
        }
    }
    auto need = [&](int& i) -> std::optional<std::string> {
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << argv[i] << "\n";
            return std::nullopt;
        }
        return std::string(argv[++i]);
    };
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { printUsage(); std::exit(0); }
        else if (arg == "--mode")     { auto v = need(i); if (!v) return false; a.mode = *v; }
        else if (arg == "--wav")      { auto v = need(i); if (!v) return false; a.wav_path = *v; }
        else if (arg == "--waveform") { auto v = need(i); if (!v) return false; a.waveform = *v; }
        else if (arg == "--rate")     { auto v = need(i); if (!v) return false; a.code_rate = *v; }
        else if (arg == "--mod")      { auto v = need(i); if (!v) return false; a.modulation = *v; }
        else if (arg == "--channel")  { auto v = need(i); if (!v) return false; a.channel = *v; }
        else if (arg == "--wav-format") { auto v = need(i); if (!v) return false; a.wav_format = *v; }
        else if (arg == "--sample-rate") { auto v = need(i); if (!v) return false; a.output_sample_rate = static_cast<uint32_t>(std::stoul(*v)); }
        else if (arg == "--snr")      { auto v = need(i); if (!v) return false; a.snr_db = std::stof(*v); }
        else if (arg == "--seed")     { auto v = need(i); if (!v) return false; a.seed = static_cast<uint32_t>(std::stoul(*v)); }
        else if (arg == "--payload")  { auto v = need(i); if (!v) return false; a.payload_bytes = std::stoi(*v); }
        else if (arg == "--frames")   { auto v = need(i); if (!v) return false; a.num_frames = std::stoi(*v); }
        else if (arg == "--text")     { auto v = need(i); if (!v) return false; a.text = *v; }
        else if (arg == "--preamble") { auto v = need(i); if (!v) return false; a.preamble = *v; }
        else if (arg == "--connected") { a.connected = true; }
        else if (arg == "--cw-count") { auto v = need(i); if (!v) return false; a.cw_count = std::stoi(*v); }
        else if (arg == "--burst-interleave") { a.burst_interleave = true; }
        else if (arg == "--no-burst-interleave") { a.burst_interleave = false; }
        else if (arg == "--burst-group-size") { auto v = need(i); if (!v) return false; a.burst_group_size = std::stoi(*v); }
        else if (arg == "--expert") { a.expert_phy = true; }
        else { std::cerr << "Unknown option: " << arg << "\n"; return false; }
    }
    if (a.mode != "gen" && a.mode != "bench") {
        std::cerr << "Need --mode gen|bench\n"; return false;
    }
    if (a.wav_path.empty()) {
        std::cerr << "Need --wav <path>\n"; return false;
    }
    return true;
}

std::string printableBytes(const Bytes& bytes, size_t start = 0) {
    std::string out;
    for (size_t i = start; i < bytes.size(); ++i) {
        char c = static_cast<char>(bytes[i]);
        out.push_back((c >= 0x20 && c < 0x7F) ? c : '.');
    }
    return out;
}

// -------- Channel models at target SNR -------------------------------------

void applyAWGN(std::vector<float>& samples, float snr_db, uint32_t seed) {
    // Continuous AWGN sized to the calibrated modem-reference RMS so the
    // configured snr_db matches the SimulatedChannel / GUI sim semantics.
    std::mt19937 rng(seed);
    const float sigma = ultra::sim::modemReferenceNoiseStddev(snr_db);
    std::normal_distribution<float> noise(0.0f, sigma);
    for (float& s : samples) {
        s += noise(rng);
    }
}

ultra::sim::WattersonChannel::Config channelConfig(ChannelType type, float snr_db) {
    switch (type) {
        case ChannelType::PASSTHROUGH: return ultra::sim::itu_r_f1487::awgn(80.0f);
        case ChannelType::AWGN:     return ultra::sim::itu_r_f1487::awgn(snr_db);
        case ChannelType::GOOD:     return ultra::sim::itu_r_f1487::good(snr_db);
        case ChannelType::MODERATE: return ultra::sim::itu_r_f1487::moderate(snr_db);
        case ChannelType::POOR:     return ultra::sim::itu_r_f1487::poor(snr_db);
        case ChannelType::FLUTTER:  return ultra::sim::itu_r_f1487::flutter(snr_db);
    }
    return ultra::sim::itu_r_f1487::awgn(snr_db);
}

std::vector<float> applyFadingChannel(const std::vector<float>& samples,
                                      ChannelType type,
                                      float snr_db,
                                      uint32_t seed) {
    ultra::sim::WattersonChannel channel(channelConfig(type, snr_db), seed);
    return channel.process(ultra::SampleSpan(samples.data(), samples.size()));
}

std::string channelName(ChannelType type) {
    switch (type) {
        case ChannelType::PASSTHROUGH: return "passthrough";
        case ChannelType::AWGN:     return "awgn";
        case ChannelType::GOOD:     return "good";
        case ChannelType::MODERATE: return "moderate";
        case ChannelType::POOR:     return "poor";
        case ChannelType::FLUTTER:  return "flutter";
    }
    return "unknown";
}

bool writeFixtureWav(const Args& a, const std::vector<float>& samples) {
    std::vector<float> output;
    const std::vector<float>* to_write = &samples;
    if (a.output_sample_rate != ultra::tools::io::kWavTargetSampleRate) {
        ultra::Resampler resampler(ultra::tools::io::kWavTargetSampleRate,
                                   a.output_sample_rate);
        output = resampler.process(ultra::SampleSpan(samples.data(), samples.size()));
        to_write = &output;
    }

    const std::string format = ultra::tools::cli::normalizedToken(a.wav_format);
    if (format == "f32" || format == "float32") {
        return ultra::tools::io::writeWavF32Mono(a.wav_path, *to_write,
                                                 a.output_sample_rate);
    }
    if (format == "pcm16" || format == "s16") {
        return ultra::tools::io::writeWavPCM16Mono(a.wav_path, *to_write,
                                                   a.output_sample_rate);
    }
    std::cerr << "Unknown WAV format: " << a.wav_format << " (use f32 or pcm16)\n";
    return false;
}

// -------- gen mode --------------------------------------------------------

// Build the OFDM config used by both bench encoder and decoder.
// MUST match what production code uses post-handshake — otherwise
// fixtures generated by the bench can't be decoded by ultra_gui /
// ultra_tnc and vice-versa. Production path:
//   ModemEngine has a default ModemConfig (cp=MEDIUM, use_pilots=false,
//   spacing=2). On setConnected it sets use_pilots=true and
//   pilot_spacing=recommended (10 for DQPSK R1/4). cp_mode stays
//   MEDIUM. setWaveformMode propagates the same config to the
//   encoder via setOFDMConfig(), so encoder and decoder agree.
//
// We mirror that here. cp_mode=MEDIUM is the critical one — using
// LONG (the StreamingEncoder constructor default, which gets
// overridden in production) yields fixtures that decode in the
// bench but NOT in the GUI's monitor mode or in any production
// receiver.
ultra::ModemConfig benchOFDMConfig() {
    ultra::ModemConfig c;
    c.fft_size = 1024;
    c.num_carriers = 59;
    c.sample_rate = 48000;
    c.center_freq = 1500;
    c.cp_mode = ultra::CyclicPrefixMode::MEDIUM;
    c.use_pilots = true;
    c.pilot_spacing = 10;
    return c;
}

int runGen(const Args& a) {
    const auto modulation = parseFixtureModulation(a);
    if (!modulation) return 1;
    const CodeRate code_rate = ultra::tools::cli::requireCodeRate(a.code_rate);
    const ChannelType channel_type = ultra::tools::cli::requireChannelType(a.channel);
    if (a.output_sample_rate == 0) {
        std::cerr << "--sample-rate must be > 0\n";
        return 1;
    }

    StreamingEncoder enc;
    enc.setMode(ultra::tools::cli::requireWaveformMode(a.waveform));
    enc.setOFDMConfig(benchOFDMConfig());
    enc.setDataMode(*modulation, code_rate);
    const int fixed_cw = (a.cw_count > 0)
        ? v2::sanitizeFixedFrameCodewords(a.cw_count)
        : v2::kDefaultFixedFrameCodewords;
    // Bench targets the connected-mode fixed-frame data path.
    enc.setFixedFrameCodewords(fixed_cw);
    // Channel interleave defaults to true on both encoder and decoder.
    // Match the default so fixtures are decodable by anything that
    // hasn't explicitly overridden — including the GUI in monitor mode
    // and any real-radio receiver running the production defaults.

    // Cap payload to fixed-frame capacity so the encoder doesn't spill
    // into multi-frame fragmentation. We want a deterministic single-
    // frame burst per iteration.
    const size_t cap = v2::getFixedFramePayloadCapacity(
        code_rate, fixed_cw);
    const size_t payload_bytes = std::min(static_cast<size_t>(a.payload_bytes), cap);

    std::cout << "[gen] waveform=" << a.waveform
              << " rate=" << a.code_rate
              << " mod=" << a.modulation
              << " channel=" << channelName(channel_type)
              << " snr=" << a.snr_db
              << " wav_format=" << a.wav_format
              << " sample_rate=" << a.output_sample_rate
              << " frames=" << a.num_frames
              << " fixed_cw=" << fixed_cw
              << " payload=" << payload_bytes << " bytes/frame (capacity=" << cap << ")"
              << " seed=" << a.seed << "\n";

    // Deterministic payload: derive bytes from a seeded LCG so the same
    // (seed, payload_bytes) always yields identical input.
    std::mt19937 rng(a.seed);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    std::vector<float> all_samples;
    all_samples.reserve(static_cast<size_t>(a.num_frames) * 24000);  // rough preallocate

    for (int f = 0; f < a.num_frames; ++f) {
        Bytes payload(payload_bytes);
        if (!a.text.empty()) {
            // Repeat the supplied text across the payload so the decoded
            // bytes spell out a human-readable message. Easier to verify
            // an OTA test by eye than diffing pseudo-random bytes.
            for (size_t i = 0; i < payload.size(); ++i) {
                payload[i] = static_cast<uint8_t>(a.text[i % a.text.size()]);
            }
        } else {
            for (auto& b : payload) b = static_cast<uint8_t>(byte_dist(rng));
        }

        // Use v2::makeFixedDataFrame so total_cw is explicitly set to
        // the requested fixed-CW geometry. DataFrame::makeData() calls
        // calculateCodewords() which for a 60-byte payload at R1/4 returns 5 CWs
        // (continuation CWs
        // reserve DATA_CW_HEADER_SIZE bytes). The OFDM encoder trusts
        // byte 12 of the serialized frame and frame-interleaves over
        // that count — if it disagrees with the decoder, the
        // de-interleave permutation is wrong and LDPC fails on every
        // CW with saturated-but-wrong-position bits. (Codex review.)
        auto frame = v2::makeFixedDataFrame(
            "BENCH1", "BENCH2", static_cast<uint16_t>(f), payload,
            code_rate, fixed_cw);
        Bytes serialized = frame.serialize();

        // Preamble selection:
        //   "chirp" — full chirp + LTS, decodable by an idle receiver
        //   in disconnected mode (what the GUI does by default, what
        //   any real-radio OTA listener expects when not in session).
        //   This is the right default for OTA fixtures.
        //   "light" — LTS-only, smaller per-frame overhead. Decodable
        //   only by a receiver already in connected mode (e.g. our
        //   bench harness).
        std::vector<float> frame_samples;
        if (a.preamble == "light") {
            frame_samples = enc.encodeFrameLight(serialized);
        } else {
            frame_samples = enc.encodeFrame(serialized);
        }
        all_samples.insert(all_samples.end(), frame_samples.begin(), frame_samples.end());

        // ~50 ms gap between frames so the decoder sees clean boundaries.
        all_samples.insert(all_samples.end(), 2400, 0.0f);
    }

    // Decoder needs ≥2.5 s of buffered audio before it begins LTS
    // correlation search. Pad with silence so a small payload doesn't
    // get stuck in "not enough samples" forever.
    constexpr size_t kMinTrailingSilenceSamples = 48000 * 3;
    if (all_samples.size() < kMinTrailingSilenceSamples) {
        all_samples.insert(all_samples.end(),
                           kMinTrailingSilenceSamples - all_samples.size(),
                           0.0f);
    } else {
        all_samples.insert(all_samples.end(), 24000, 0.0f);  // 0.5 s tail
    }

    std::cout << "[gen] synthesized " << all_samples.size() << " samples ("
              << std::fixed << static_cast<double>(all_samples.size()) / 48000.0
              << " s)\n";

    if (a.snr_db < 80.0f) {
        // Apply the channel with a deterministic offset of the seed so two
        // fixtures at different SNR but same seed don't share noise/fades.
        if (channel_type == ChannelType::AWGN) {
            applyAWGN(all_samples, a.snr_db, a.seed ^ 0xA5A5A5A5u);
        } else {
            all_samples = applyFadingChannel(all_samples, channel_type, a.snr_db,
                                             a.seed ^ 0xA5A5A5A5u);
        }
        std::cout << "[gen] applied " << channelName(channel_type)
                  << " channel at " << a.snr_db << " dB\n";
    } else {
        std::cout << "[gen] noiseless (SNR>=80)\n";
    }

    if (!writeFixtureWav(a, all_samples)) {
        std::cerr << "Failed to write " << a.wav_path << "\n";
        return 1;
    }
    std::cout << "[gen] wrote " << a.wav_path << "\n";
    return 0;
}

// -------- bench mode ------------------------------------------------------

int runBench(const Args& a) {
    ultra::tools::io::LoadedWav wav;
    try {
        wav = ultra::tools::io::loadWavMono48k(a.wav_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to read " << a.wav_path << ": " << e.what() << "\n";
        return 1;
    }
    std::vector<float>& samples = wav.samples_48k;
    std::cout << "[bench] loaded " << samples.size() << " samples ("
              << static_cast<double>(samples.size()) / 48000.0 << " s) from "
              << a.wav_path << " [source=" << wav.source_rate << " Hz, "
              << wav.source_channels << " ch, format=" << wav.source_format
              << ", bits=" << wav.source_bits << "]\n";

    const WaveformMode waveform = ultra::tools::cli::requireWaveformMode(a.waveform);
    if (a.connected && !ultra::protocol::isOFDMMode(waveform)) {
        std::cerr << "--connected requires an OFDM waveform\n";
        return 1;
    }

    const CodeRate code_rate = ultra::tools::cli::requireCodeRate(a.code_rate);
    const auto modulation = parseFixtureModulation(a);
    if (!modulation) return 1;
    const int fixed_cw = (a.cw_count > 0)
        ? v2::sanitizeFixedFrameCodewords(a.cw_count)
        : (a.connected ? ultra::protocol::connection_policy::recommendCWCount(code_rate, waveform)
                       : v2::kDefaultFixedFrameCodewords);

    std::cout << "[bench] connected=" << (a.connected ? "yes" : "no")
              << " waveform=" << a.waveform
              << " rate=" << a.code_rate
              << " mod=" << a.modulation
              << " fixed_cw=" << fixed_cw << "\n";

    StreamingDecoder dec;
    if (a.connected) {
        // Post-CONNECT_ACK capture: receiver is already in negotiated OFDM data
        // mode and must search for LTS/light preambles, not startup chirps.
        dec.setConnectedOFDMMode(waveform, benchOFDMConfig(),
                                 *modulation, code_rate);
        dec.setBurstInterleave(a.burst_interleave && waveform == WaveformMode::OFDM_CHIRP);
        dec.setBurstInterleaveGroupSize(a.burst_group_size);
    } else {
        // Standalone fixture mode: acquire a full preamble from idle.
        dec.setMode(waveform, false);
        dec.setOFDMConfig(benchOFDMConfig());
        dec.setDataMode(*modulation, code_rate);
    }
    dec.setFixedFrameCodewords(fixed_cw);
    // Match encoder default (channel_interleave=true) so the bench
    // self-test agrees with how production receivers interpret the
    // fixture. Forcing false here would diverge from the GUI/TNC
    // defaults and turn decoded output into bit-permuted garbage.
    dec.setKnownCFO(0.0f);
    dec.clearShutdown();

    std::mutex results_mutex;
    int frames_decoded = 0;
    int frames_failed = 0;
    int data_frames_decoded = 0;
    int ack_frames_decoded = 0;
    int control_frames_decoded = 0;
    int other_frames_decoded = 0;
    int byte_exact_ok = 0;
    int byte_exact_bad = 0;
    std::vector<std::string> decoded_payloads;
    std::vector<std::string> reassembled_texts;
    Bytes reassembly;
    dec.setFrameCallback([&](const DecodeResult& r) {
        std::lock_guard<std::mutex> lock(results_mutex);
        if (r.success) {
            ++frames_decoded;
            auto hdr = v2::parseHeader(r.frame_data);
            const char* type_name = hdr.valid ? v2::frameTypeToString(hdr.type) : "UNKNOWN";
            bool exact = hdr.valid;

            if (hdr.valid && v2::isDataFrame(hdr.type)) {
                ++data_frames_decoded;
                auto df = v2::DataFrame::deserialize(r.frame_data);
                exact = df.has_value();
                if (df) {
                    decoded_payloads.push_back(printableBytes(df->payload));
                    reassembly.insert(reassembly.end(), df->payload.begin(), df->payload.end());
                    if ((df->flags & v2::Flags::MORE_FRAG) == 0) {
                        size_t start = (!reassembly.empty() && reassembly[0] == 0x00) ? 1 : 0;
                        reassembled_texts.push_back(printableBytes(reassembly, start));
                        reassembly.clear();
                    }
                }
            } else if (hdr.valid && hdr.type == v2::FrameType::ACK) {
                ++ack_frames_decoded;
            } else if (hdr.valid && hdr.is_control) {
                ++control_frames_decoded;
            } else {
                ++other_frames_decoded;
            }
            if (exact) ++byte_exact_ok;
            else ++byte_exact_bad;

            std::cout << "[frame] idx=" << frames_decoded
                      << " type=" << type_name
                      << " seq=" << (hdr.valid ? hdr.seq : 0)
                      << " bytes=" << r.frame_data.size()
                      << " byte_exact=" << (exact ? "OK" : "BAD") << "\n";
        } else {
            ++frames_failed;
        }
    });

    // Reset profiling counters so we measure only this decode.
    ultra::timing::globalDecoderProfile().reset();

    // Spawn a worker thread that drives processBuffer() repeatedly,
    // matching how production code uses the decoder. We then feed all
    // samples and wait until either: (a) all expected frames decoded,
    // or (b) a quiet period elapses with no new frame deliveries.
    std::atomic<bool> stop_worker{false};
    std::thread worker([&]() {
        while (!stop_worker.load(std::memory_order_acquire)) {
            dec.processBuffer();
        }
    });

    const auto wall_start = std::chrono::steady_clock::now();

    // Feed in small chunks so each chunk re-arms new_data_available_,
    // matching how the audio thread feeds in production. processBuffer
    // is a single-step state machine that consumes one wakeup per call;
    // dumping all audio at once would only fire SEARCHING, then time
    // out. We pace at faster-than-realtime (~10× audio rate) so the
    // decoder thread has room to advance through SEARCHING→SYNC_FOUND→
    // DECODING between chunks but the bench still finishes quickly.
    constexpr size_t kChunkSamples = 4096;     // ~85 ms at 48 kHz
    constexpr int kInterChunkSleepMs = 8;      // ~10× realtime feed
    for (size_t off = 0; off < samples.size(); off += kChunkSamples) {
        const size_t chunk = std::min(kChunkSamples, samples.size() - off);
        dec.feedAudio(samples.data() + off, chunk);
        std::this_thread::sleep_for(std::chrono::milliseconds(kInterChunkSleepMs));
    }
    // Drip silence after the real audio so the decoder's state machine
    // gets new_data_available_ wakeups to advance from SYNC_FOUND →
    // DECODING after the last frame arrives. Without this the worker
    // sits in its 50 ms timeout loop and never finishes the last frame.
    std::vector<float> tail_silence(kChunkSamples, 0.0f);
    for (int i = 0; i < 25; ++i) {  // ~2 s of silence wakeups
        dec.feedAudio(tail_silence);
        std::this_thread::sleep_for(std::chrono::milliseconds(kInterChunkSleepMs));
    }

    // Wait until decode quiesces. "Quiet" = no new frame for 200 ms
    // AND no samples remaining in the input buffer. Cap total wait at
    // 5 s so a busted fixture doesn't hang the bench.
    const auto poll_start = std::chrono::steady_clock::now();
    int last_total = -1;
    auto last_change = poll_start;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        int total = 0;
        {
            std::lock_guard<std::mutex> lock(results_mutex);
            total = frames_decoded + frames_failed;
        }
        const auto now = std::chrono::steady_clock::now();
        if (total != last_total) {
            last_total = total;
            last_change = now;
        }
        const auto since_change = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_change).count();
        const auto since_start = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - poll_start).count();
        const bool buffer_empty = dec.samplesInBuffer() == 0;
        if (buffer_empty && since_change > 200) break;
        if (since_start > 5000) break;
    }

    stop_worker.store(true, std::memory_order_release);
    dec.stop();          // wakes processBuffer() out of any blocking wait
    worker.join();

    const auto wall_end = std::chrono::steady_clock::now();
    const double wall_ms =
        std::chrono::duration<double, std::milli>(wall_end - wall_start).count();

    // ----- Print profile -----
    auto& dp = ultra::timing::globalDecoderProfile();
    auto fmt = [](const ultra::timing::PhaseStats& s) -> std::string {
        const uint64_t cnt = s.count.load();
        const uint64_t tot = s.total_us.load();
        const uint64_t mx  = s.max_us.load();
        if (cnt == 0) return "(0 calls)";
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                 "n=%llu  total=%.2fms  mean=%.1fus  max=%lluus",
                 static_cast<unsigned long long>(cnt),
                 tot / 1000.0,
                 static_cast<double>(tot) / static_cast<double>(cnt),
                 static_cast<unsigned long long>(mx));
        return std::string(buf);
    };

    std::cout << "\n=== decode_bench results ===\n";
    std::cout << "wall_clock_ms             " << wall_ms << "\n";
    std::cout << "frames_decoded            " << frames_decoded << "\n";
    std::cout << "frames_failed             " << frames_failed << "\n";
    std::cout << "data_frames_decoded       " << data_frames_decoded << "\n";
    std::cout << "ack_frames_decoded        " << ack_frames_decoded << "\n";
    std::cout << "control_frames_decoded    " << control_frames_decoded << "\n";
    std::cout << "other_frames_decoded      " << other_frames_decoded << "\n";
    std::cout << "byte_exact_ok             " << byte_exact_ok << "\n";
    std::cout << "byte_exact_bad            " << byte_exact_bad << "\n";

    if (!decoded_payloads.empty()) {
        // Print the first decoded payload verbatim — eyeball check
        // for text fixtures. Drop non-printable bytes so a mangled
        // payload doesn't garble the terminal.
        std::cout << "first_decoded_payload     \"";
        for (char c : decoded_payloads.front()) {
            if (c >= 0x20 && c < 0x7F) std::cout << c;
            else std::cout << '.';
        }
        std::cout << "\"\n";
    }
    if (!reassembled_texts.empty()) {
        std::cout << "reassembled_text_count    " << reassembled_texts.size() << "\n";
        std::cout << "first_reassembled_text    \"" << reassembled_texts.front() << "\"\n";
    }
    std::cout << "\n--- DecoderProfile (this decode) ---\n";
    std::cout << "  detect_data_sync          " << fmt(dp.detect_data_sync) << "\n";
    std::cout << "  ofdm_process_total        " << fmt(dp.ofdm_process_total) << "\n";
    std::cout << "  lts_channel_estimate      " << fmt(dp.lts_channel_estimate) << "\n";
    std::cout << "  data_symbol_loop          " << fmt(dp.data_symbol_loop) << "\n";
    std::cout << "  decode_fixed_frame_total  " << fmt(dp.decode_fixed_frame_total) << "\n";
    std::cout << "  ldpc_cw_total             " << fmt(dp.ldpc_cw_total) << "\n";
    std::cout << "  single_cw_decode_total    " << fmt(dp.single_cw_decode_total) << "\n";
    std::cout << "  control_first_1cw         " << fmt(dp.control_first_1cw) << "\n";
    std::cout << "  cw0_peek_1cw              " << fmt(dp.cw0_peek_1cw) << "\n";
    std::cout << "  ofdm_cw0_probe_decode     " << fmt(dp.ofdm_cw0_probe_decode) << "\n";
    std::cout << "  failed_4cw_after_peek     " << fmt(dp.failed_4cw_after_peek) << "\n";
    std::cout << "  low_llr_escalation_skipped count=" << dp.low_llr_escalation_skipped.load() << "\n";
    std::cout << "  raw_cw0_probe_skipped     count=" << dp.raw_cw0_probe_skipped.load() << "\n";
    std::cout << "  harq_key_build            success=" << dp.harq_key_build_success.load()
              << "  failed=" << dp.harq_key_build_failed.load() << "\n";

    return frames_failed > 0 ? 2 : 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Args a;
        if (!parseArgs(argc, argv, a)) {
            printUsage();
            return 1;
        }
        if (a.mode == "gen") return runGen(a);
        if (a.mode == "bench") return runBench(a);
        return 1;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
