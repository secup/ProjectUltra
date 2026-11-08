#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "gui/modem/streaming_frame_policy.hpp"
#include "ota_channel_core/channel.hpp"
#include "ofdm/genie_tx_capture.hpp"
#include "ofdm/genie_true_h.hpp"
#include "ofdm/pilot_pattern.hpp"
#include "protocol/link_abstraction.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using ultra::Bytes;
namespace gui = ultra::gui;
namespace protocol = ultra::protocol;
namespace v2 = ultra::protocol::v2;

constexpr int kSampleRate = 48000;
constexpr size_t kPumpChunkSamples = 4800;
constexpr size_t kLightSearchSamples = 9600;
constexpr size_t kFullSearchSamples = 120000;

enum class MeasureConfig {
    AckLight,
    Data4Light,
    Data4Full,
    AckFull,
    WarmSyncLight,
    BurstChunk,
};

struct Args {
    float snr_db = 0.0f;
    MeasureConfig config = MeasureConfig::AckLight;
    std::string config_name;
    uint64_t seed = 0;
    int n = 0;
    // Diversity-sweep parameters. Default modulation is QPSK (2026-11-08): the old
    // DQPSK default was silently measured as coherent QPSK (OFDMChirpWaveform falls
    // back) and DQPSK will become a real differential waveform, so the default must
    // name what is actually transmitted.
    ultra::Modulation mod = ultra::Modulation::QPSK;
    ultra::CodeRate rate = ultra::CodeRate::R1_4;
    ultra::ota_channel_core::ChannelType channel =
        ultra::ota_channel_core::ChannelType::AWGN;
    bool carrier_interleave = false;
    // Production geometry (2026-11-08): frame CW count from recommendCWCount(mod, rate)
    // and MEDIUM CP, i.e. what the GUI actually transmits. Off by default so historical
    // invocations stay comparable.
    bool geometry_production = false;
    // Channel-noise pre-roll fed to the decoder before EACH burst. A real receiver always
    // hears band noise before a burst (INV-SIM-AWGN-001); without it the first trial on a
    // fresh decoder arms the anchor wait mid-burst and misses its chirp (the constant
    // 1-per-run first-frame failure). 0 = legacy behaviour.
    int preroll_ms = 0;
    // Pump the full sample budget every trial so two arms of an A/B stay in lockstep
    // (identical channel samples per frame => paired comparison).
    bool fixed_budget = false;
    std::string mod_name = "qpsk";
    std::string rate_name = "r1_4";
    std::string channel_name = "awgn";
    // Burst-chunk (file-class keystone) parameters.
    int group = 0;               // frames per burst group (chunk length); clamps to [2,8]
    int frame_cw = static_cast<int>(v2::kDefaultFixedFrameCodewords);  // codewords per frame
    bool burst_interleave = true; // cross-frame deep interleave ON/OFF (the A/B)
    bool burst_descriptor = false; // §14.17: emit BURST_HEADER descriptor + mis-set
                                   // decoder group size to prove the descriptor fixes it
    // 2026-07-29 diag: perfect-CSI bound. Runs every frame TWICE -- once through a
    // shadow channel with the same seed at very high SNR (so its LTS estimate is
    // the true H), then through the real channel with that H injected. Both passes
    // are pumped by an identical, fixed sample budget so the two fade trajectories
    // stay in lockstep for the whole run. See src/ofdm/genie_true_h.hpp.
    bool genie_true_h = false;
    // Shadow-channel SNR. Setting this EQUAL to --snr is the oracle's own null
    // control: same seed + same SNR => bit-identical signal => both passes sync
    // identically => injection must be a no-op and reproduce baseline exactly.
    // Any deviation there is a defect in the oracle, not a channel finding.
    float genie_clean_snr_db = 60.0f;
    // Cyclic prefix. DEFAULT LONG deliberately: every measurement ever made with this
    // tool used LONG, so defaulting to production MEDIUM would silently invalidate
    // comparison against all of them. PRODUCTION IS MEDIUM (presets::balanced(), pushed
    // via setOFDMConfig at modem_engine.cpp:250) -- pass --cp medium to measure the
    // shipped geometry. See docs/CHANGELOG.md 2026-07-29.
    std::string cp_name = "long";
    // Phase 5: dump the per-frame per-carrier gamma grid paired with the decode
    // outcome. This is the training data for the EESM/PER link abstraction.
    std::string gamma_csv;
    // Phase 6 shadow selector: load a calibration table, score EVERY candidate against
    // the measured grid, and report predicted-vs-actual plus what the selector would
    // have chosen. Scores only -- it never changes the transmitted rung.
    std::string calib_path;
    double shadow_max_per = 0.10;
    // 2026-07-29 Phase 0 payoff: measure the LTS channel estimate against
    // SIMULATOR TRUTH and report NMSE. Requires ULTRA_CHANNEL_DOPPLER_HZ=0 so the
    // taps are frozen and truth H is time-invariant -- otherwise the tap state
    // read after the frame is not the state the LTS saw.
    bool chest_nmse = false;
    int ldpc_z = 27;               // LDPC lifting size: 27 (N=648) or 81 (N=1944).
                                   // The encoder uses its production setter. For
                                   // non-burst frame measurements, the decoder uses
                                   // an explicit test-only override because there is
                                   // no BURST_HEADER on that isolated path.
};

struct Counts {
    int sync_fail = 0;
    int decode_fail = 0;
    int crc_fail = 0;
    int pass = 0;
};

struct FrameTrial {
    Bytes frame_bytes;
    std::vector<float> tx_samples;
};

struct TrialOutcome {
    bool sync_seen = false;
    bool decoder_failed = false;
    bool saw_result = false;
    gui::DecodeResult result;
};

bool bytesMatch(const Bytes& a, const Bytes& b);
void classify(const TrialOutcome& outcome, const Bytes& expected, Counts& counts);

void usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " --snr <db> --config <ack_light|data4_light|data4_full|ack_full|warm_sync_light|burst_chunk>"
        << " --seed <u64> --n <frames|chunks>"
        << " [--channel awgn|good|moderate|poor]"
        << " [--mod dqpsk|qpsk|d8psk|qam8|qam16]"
        << " [--rate r1_4|r1_2|r2_3|r3_4]"
        << " [--carrier-interleave 0|1]"
        << " [--group <2..8>] [--burst-interleave 0|1]\n"
        << "CSV (frame): snr,config,channel,mod,rate,ci,seed,n,sync_fail,decode_fail,crc_fail,pass\n"
        << "CSV (burst): snr,config,channel,mod,rate,bi,group,seed,chunks,frames_total,"
           "frames_recovered,chunks_complete,chunk_sync_fail\n";
}

MeasureConfig parseConfig(const std::string& name) {
    if (name == "ack_light") {
        return MeasureConfig::AckLight;
    }
    if (name == "data4_light") {
        return MeasureConfig::Data4Light;
    }
    if (name == "data4_full") {
        return MeasureConfig::Data4Full;
    }
    if (name == "burst_chunk") {
        return MeasureConfig::BurstChunk;
    }
    if (name == "ack_full") {
        return MeasureConfig::AckFull;
    }
    if (name == "warm_sync_light") {
        return MeasureConfig::WarmSyncLight;
    }
    throw std::runtime_error("unknown --config: " + name);
}

ultra::Modulation parseMod(const std::string& s) {
    if (s == "dqpsk") return ultra::Modulation::DQPSK;
    if (s == "qpsk") return ultra::Modulation::QPSK;  // coherent
    if (s == "d8psk") return ultra::Modulation::D8PSK;
    if (s == "qam8") return ultra::Modulation::QAM8;
    if (s == "qam16") return ultra::Modulation::QAM16;
    throw std::runtime_error("unknown --mod: " + s);
}

ultra::CodeRate parseRate(const std::string& s) {
    if (s == "r1_4") return ultra::CodeRate::R1_4;
    if (s == "r1_2") return ultra::CodeRate::R1_2;
    if (s == "r2_3") return ultra::CodeRate::R2_3;
    if (s == "r3_4") return ultra::CodeRate::R3_4;
    throw std::runtime_error("unknown --rate: " + s);
}

ultra::ota_channel_core::ChannelType parseChannel(const std::string& s) {
    using CT = ultra::ota_channel_core::ChannelType;
    if (s == "awgn") return CT::AWGN;
    if (s == "good") return CT::GOOD;
    if (s == "moderate") return CT::MODERATE;
    if (s == "poor") return CT::POOR;
    throw std::runtime_error("unknown --channel: " + s);
}

Args parseArgs(int argc, char** argv) {
    Args args;
    bool have_snr = false;
    bool have_config = false;
    bool have_seed = false;
    bool have_n = false;

    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto requireValue = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + opt);
            }
            return argv[++i];
        };

        if (key == "--snr") {
            args.snr_db = std::stof(requireValue("--snr"));
            have_snr = true;
        } else if (key == "--config") {
            args.config_name = requireValue("--config");
            args.config = parseConfig(args.config_name);
            have_config = true;
        } else if (key == "--seed") {
            args.seed = std::stoull(requireValue("--seed"));
            have_seed = true;
        } else if (key == "--n") {
            args.n = std::stoi(requireValue("--n"));
            have_n = true;
        } else if (key == "--mod") {
            args.mod_name = requireValue("--mod");
            args.mod = parseMod(args.mod_name);
        } else if (key == "--rate") {
            args.rate_name = requireValue("--rate");
            args.rate = parseRate(args.rate_name);
        } else if (key == "--channel") {
            args.channel_name = requireValue("--channel");
            args.channel = parseChannel(args.channel_name);
        } else if (key == "--carrier-interleave") {
            args.carrier_interleave = std::stoi(requireValue("--carrier-interleave")) != 0;
        } else if (key == "--group") {
            args.group = std::stoi(requireValue("--group"));
        } else if (key == "--frame-cw") {
            args.frame_cw = std::stoi(requireValue("--frame-cw"));
        } else if (key == "--burst-interleave") {
            args.burst_interleave = std::stoi(requireValue("--burst-interleave")) != 0;
        } else if (key == "--burst-descriptor") {
            args.burst_descriptor = std::stoi(requireValue("--burst-descriptor")) != 0;
        } else if (key == "--genie-true-h") {
            args.genie_true_h = std::stoi(requireValue("--genie-true-h")) != 0;
        } else if (key == "--calib") {
            args.calib_path = requireValue("--calib");
        } else if (key == "--shadow-max-per") {
            args.shadow_max_per = std::stod(requireValue("--shadow-max-per"));
        } else if (key == "--gamma-csv") {
            args.gamma_csv = requireValue("--gamma-csv");
        } else if (key == "--cp") {
            args.cp_name = requireValue("--cp");
            if (args.cp_name != "medium" && args.cp_name != "long" &&
                args.cp_name != "short") {
                throw std::runtime_error("unknown --cp: " + args.cp_name);
            }
        } else if (key == "--chest-nmse") {
            args.chest_nmse = std::stoi(requireValue("--chest-nmse")) != 0;
        } else if (key == "--genie-clean-snr") {
            args.genie_clean_snr_db = std::stof(requireValue("--genie-clean-snr"));
        } else if (key == "--ldpc-z") {
            args.ldpc_z = std::stoi(requireValue("--ldpc-z"));
        } else if (key == "--geometry") {
            const std::string g = requireValue("--geometry");
            if (g != "production" && g != "legacy") {
                throw std::runtime_error("unknown --geometry: " + g);
            }
            args.geometry_production = (g == "production");
        } else if (key == "--preroll-ms") {
            args.preroll_ms = std::max(0, std::stoi(requireValue("--preroll-ms")));
        } else if (key == "--fixed-budget") {
            args.fixed_budget = std::stoi(requireValue("--fixed-budget")) != 0;
        } else if (key == "--help" || key == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }

    if (!have_snr || !have_config || !have_seed || !have_n || args.n <= 0) {
        usage(argv[0]);
        throw std::runtime_error("missing required argument");
    }
    if (args.geometry_production) {
        // What the GUI transmits on the connected wideband path: per-rung CW count
        // (connection_policy::recommendCWCount) and the MEDIUM cyclic prefix installed
        // by presets::balanced(). Explicit flags given AFTER --geometry do not override
        // this; give the explicit flags instead of --geometry to measure other shapes.
        args.frame_cw = ultra::protocol::connection_policy::recommendCWCount(
            args.mod, args.rate, ultra::protocol::WaveformMode::OFDM_CHIRP);
        args.cp_name = "medium";
    }
    return args;
}

ultra::ModemConfig makeOFDMConfig(ultra::Modulation mod, ultra::CodeRate rate,
                                  const std::string& cp_name = "long") {
    ultra::ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.sample_rate = kSampleRate;
    cfg.center_freq = 1500.0f;
    cfg.cp_mode = cp_name == "medium" ? ultra::CyclicPrefixMode::MEDIUM
                : cp_name == "short"  ? ultra::CyclicPrefixMode::SHORT
                                      : ultra::CyclicPrefixMode::LONG;
    cfg.modulation = mod;
    cfg.code_rate = rate;
    cfg.use_pilots = true;
    // 2026-05-29: use the PRODUCTION adaptive pilot spacing (was hardcoded 10,
    // which under-piloted coherent high-order mods by ~2x vs production's 5/8 and
    // made 16QAM look structurally broken on fading — a harness-fidelity bug).
    cfg.pilot_spacing = ultra::ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    return cfg;
}

bool usesFullPreamble(MeasureConfig config) {
    return config == MeasureConfig::AckFull || config == MeasureConfig::Data4Full;
}

void configureEncoder(gui::StreamingEncoder& encoder, const Args& args) {
    const auto ofdm = makeOFDMConfig(args.mod, args.rate, args.cp_name);
    encoder.setMode(protocol::WaveformMode::OFDM_CHIRP);
    encoder.setOFDMConfig(ofdm);
    encoder.setDataMode(args.mod, args.rate);
    encoder.setFixedFrameCodewords(args.frame_cw);
    encoder.setCarrierLdpcInterleaver(args.carrier_interleave);
    // Sets the member so the actual codeword size AND the BURST_HEADER descriptor's
    // announced lifting_z agree (the production API the connection layer uses
    // per-burst). Without this, Z=81 codewords get announced/decoded as Z=27 → 0%.
    encoder.setLDPCLiftingZ(static_cast<uint8_t>(args.ldpc_z == 81 ? 81 : 27));
}

void configureDecoder(gui::StreamingDecoder& decoder, const Args& args) {
    const auto ofdm = makeOFDMConfig(args.mod, args.rate, args.cp_name);
    // Only 1-CW control frames decode on the cold setMode() path. 4-CW DATA
    // frames are a connected-mode construct, so Data4Full uses the connected
    // decoder too — its full chirp preamble is admitted via
    // expectFullOFDMAnchorOnce() (see measure()), giving reliable sync at any
    // fade depth WITHOUT the cold-light-sync selection bias that hides the
    // deep-fade decode failures we are trying to measure.
    if (args.config == MeasureConfig::AckFull) {
        decoder.setMode(protocol::WaveformMode::OFDM_CHIRP, false);
        decoder.setOFDMConfig(ofdm);
        decoder.setDataMode(args.mod, args.rate);
    } else {
        decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                                     ofdm,
                                     args.mod,
                                     args.rate);
    }
    decoder.setFixedFrameCodewords(args.frame_cw);
    decoder.setCarrierLdpcInterleaver(args.carrier_interleave);
    if (args.ldpc_z == 81 && args.config != MeasureConfig::BurstChunk) {
        // Connected-mode configuration is deferred until processBuffer(). Apply it
        // now so the test-only Z hook reaches the newly-created OFDM waveform, then
        // keep that override for every reset() in this measurement session.
        decoder.applyPendingConfigForTesting();
        decoder.setLDPCLiftingZForTesting(81);
    }
}

Bytes randomPayload(std::mt19937_64& rng, size_t size) {
    Bytes payload(size);
    for (uint8_t& b : payload) {
        b = static_cast<uint8_t>(rng() & 0xFFu);
    }
    return payload;
}

FrameTrial makeFrame(gui::StreamingEncoder& encoder,
                     const Args& args,
                     std::mt19937_64& rng,
                     int frame_index) {
    const MeasureConfig config = args.config;
    const uint16_t seq = static_cast<uint16_t>((rng() + frame_index) & 0xFFFFu);
    const bool is_data = config == MeasureConfig::Data4Light ||
                         config == MeasureConfig::Data4Full;

    FrameTrial trial;
    if (is_data) {
        const Bytes payload = randomPayload(rng, 20);
        auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", seq, payload,
                                            args.rate,
                                            args.frame_cw);
        trial.frame_bytes = frame.serialize();
        trial.tx_samples = config == MeasureConfig::Data4Full
            ? encoder.encodeFrame(trial.frame_bytes)
            : encoder.encodeFrameLight(trial.frame_bytes);
    } else {
        const auto ack = v2::ControlFrame::makeAck("BRAVO", "ALPHA", seq);
        trial.frame_bytes = ack.serialize();
        trial.tx_samples = config == MeasureConfig::AckFull
            ? encoder.encodeFrame(trial.frame_bytes)
            : encoder.encodeFrameLight(trial.frame_bytes);
    }

    if (trial.tx_samples.empty()) {
        throw std::runtime_error("encoder produced no samples");
    }
    return trial;
}

FrameTrial makeAckFrame(gui::StreamingEncoder& encoder,
                        uint16_t seq,
                        bool full_preamble) {
    FrameTrial trial;
    const auto ack = v2::ControlFrame::makeAck("BRAVO", "ALPHA", seq);
    trial.frame_bytes = ack.serialize();
    trial.tx_samples = full_preamble
        ? encoder.encodeFrame(trial.frame_bytes)
        : encoder.encodeFrameLight(trial.frame_bytes);

    if (trial.tx_samples.empty()) {
        throw std::runtime_error("encoder produced no samples");
    }
    return trial;
}

void drainDecoder(gui::StreamingDecoder& decoder,
                  TrialOutcome& outcome,
                  uint64_t initial_frames_failed) {
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.is_ping) {
            continue;
        }
        outcome.saw_result = true;
        outcome.result = std::move(result);
    }

    const auto stats = decoder.getStats();
    if (stats.frames_failed > initial_frames_failed) {
        outcome.decoder_failed = true;
    }
}

TrialOutcome runFrame(gui::StreamingDecoder& decoder,
                      ultra::ota_channel_core::SimulatedChannel& channel,
                      const std::vector<float>& tx_samples,
                      bool full_preamble,
                      bool reset_decoder,
                      bool expect_full_anchor,
                      // 2026-07-29 diag: when true, pump the FULL sample budget
                      // regardless of early success. The genie mode runs two
                      // channels with the same seed and they only stay in
                      // lockstep if both are advanced by identical sample counts;
                      // the normal early-break would desynchronize their fades.
                      bool fixed_budget = false,
                      size_t preroll_samples = 0) {
    if (reset_decoder) {
        decoder.reset();
    }
    if (expect_full_anchor) {
        decoder.expectFullOFDMAnchorOnce();
    }
    // Band noise before the burst, as a real receiver hears it (see Args::preroll_ms).
    for (size_t fed = 0; fed < preroll_samples;) {
        const size_t take = std::min(kPumpChunkSamples, preroll_samples - fed);
        auto rx = channel.receiveForB(take);
        decoder.feedAudio(rx.data(), rx.size());
        decoder.processBuffer();
        fed += take;
    }

    TrialOutcome outcome;
    decoder.setDataSyncAcceptedCallback([&outcome](float) {
        outcome.sync_seen = true;
    });

    const auto initial_stats = decoder.getStats();
    const uint64_t initial_frames_failed = initial_stats.frames_failed;

    channel.transmitFromA(tx_samples);

    const size_t min_search = full_preamble ? kFullSearchSamples : kLightSearchSamples;
    const size_t target_samples = std::max(tx_samples.size() + 4 * kPumpChunkSamples,
                                           min_search + 4 * kPumpChunkSamples);

    size_t received = 0;
    while (received < target_samples) {
        const size_t take = std::min(kPumpChunkSamples, target_samples - received);
        auto rx = channel.receiveForB(take);
        decoder.feedAudio(rx.data(), rx.size());
        decoder.processBuffer();
        outcome.sync_seen = outcome.sync_seen || decoder.isSynced();
        drainDecoder(decoder, outcome, initial_frames_failed);

        if (!fixed_budget && outcome.saw_result && outcome.result.success) {
            break;
        }
        received += take;
    }

    for (int i = 0;
         i < 8 && (fixed_budget || !(outcome.saw_result && outcome.result.success));
         ++i) {
        auto rx = channel.receiveForB(kPumpChunkSamples);
        decoder.feedAudio(rx.data(), rx.size());
        decoder.processBuffer();
        outcome.sync_seen = outcome.sync_seen || decoder.isSynced();
        drainDecoder(decoder, outcome, initial_frames_failed);
    }

    return outcome;
}

bool isExpectedSuccess(const TrialOutcome& outcome, const Bytes& expected) {
    return outcome.saw_result &&
           outcome.result.success &&
           bytesMatch(outcome.result.frame_data, expected);
}

void runWarmSyncTrial(gui::StreamingEncoder& encoder,
                      gui::StreamingDecoder& decoder,
                      ultra::ota_channel_core::SimulatedChannel& channel,
                      const FrameTrial& measured_light,
                      uint16_t anchor_seq,
                      Counts& counts) {
    const FrameTrial anchor = makeAckFrame(encoder, anchor_seq, /*full_preamble=*/true);
    auto anchor_outcome = runFrame(decoder, channel, anchor.tx_samples,
                                   /*full_preamble=*/true,
                                   /*reset_decoder=*/true,
                                   /*expect_full_anchor=*/true);
    if (!isExpectedSuccess(anchor_outcome, anchor.frame_bytes)) {
        classify(anchor_outcome, anchor.frame_bytes, counts);
        return;
    }

    auto measured_outcome = runFrame(decoder, channel, measured_light.tx_samples,
                                     /*full_preamble=*/false,
                                     /*reset_decoder=*/false,
                                     /*expect_full_anchor=*/false);
    classify(measured_outcome, measured_light.frame_bytes, counts);
}

bool bytesMatch(const Bytes& a, const Bytes& b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

void classify(const TrialOutcome& outcome, const Bytes& expected, Counts& counts) {
    if (outcome.saw_result) {
        if (outcome.result.success && bytesMatch(outcome.result.frame_data, expected)) {
            ++counts.pass;
            return;
        }

        if (outcome.result.success ||
            (outcome.result.codewords_ok > 0 && outcome.result.codewords_failed == 0)) {
            ++counts.crc_fail;
            return;
        }

        ++counts.decode_fail;
        return;
    }

    if (!outcome.sync_seen) {
        ++counts.sync_fail;
    } else {
        ++counts.decode_fail;
    }
}



// === Phase 6: shadow selector evaluation ====================================
//
// Scores every calibrated candidate against the measured per-carrier grid, then
// compares (a) the predicted PER of the rung that was ACTUALLY transmitted against
// what that rung actually did -- the calibration check -- and (b) what a
// reliability-constrained selector WOULD have chosen. It never changes the
// transmitted rung; Phase 7 handles actuation, and only after this evidence exists.
namespace la = ultra::protocol::link_abstraction;

la::CalibrationTable loadCalibration(const std::string& path) {
    la::CalibrationTable t;
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open calibration table: " + path);
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::vector<std::string> f;
        std::string cur;
        for (char c : line) {
            if (c == ',') { f.push_back(cur); cur.clear(); }
            else { cur.push_back(c); }
        }
        f.push_back(cur);
        if (f.size() == 2 && f[0] == "version") {
            t.version = std::stoi(f[1]);
            continue;
        }
        if (f.size() == 2 && f[0] == "provenance") {
            t.provenance = f[1];
            continue;
        }
        if (f.size() < 8) {
            continue;
        }
        la::RungCalibration c;
        if (!la::parseModulation(f[0], c.mod) || !la::parseCodeRate(f[1], c.rate)) {
            continue;
        }
        c.beta = std::stod(f[2]);
        c.midpoint_db = std::stod(f[3]);
        c.slope = std::stod(f[4]);
        c.valid = (f[5] == "1" || f[5] == "true");
        c.n_frames = std::stoi(f[6]);
        c.fit_rms_error = std::stod(f[7]);
        t.rungs.push_back(c);
    }
    return t;
}

struct ShadowAccum {
    long frames = 0;
    double pred_per_sum = 0.0;   // predicted PER of the TRANSMITTED rung
    long actual_fail = 0;        // what it actually did
    long shadow_same = 0;        // selector agreed with the transmitted rung
    long shadow_lower = 0;       // selector would have chosen a slower rung
    long shadow_higher = 0;      // selector would have chosen a faster rung
    long shadow_none = 0;        // nothing cleared the reliability ceiling
    double shadow_eff_sum = 0.0; // efficiency the selector would have commanded
    double actual_eff = 0.0;
    // Catastrophic overcommit: predicted <=10% PER but the frame failed. The plan's
    // Phase 5 acceptance bounds this ("no candidate predicted at <=10% FER that
    // measures >=30% FER"), and it is the failure mode that actually costs goodput.
    long confident_but_failed = 0;
    long confident_total = 0;
};

// === Estimator NMSE against simulator truth -- NOT YET TRUSTWORTHY ==========
//
// STATUS 2026-07-29 (updated): the tool's PROFILE provenance bug is FIXED; its
// TEMPORAL provenance is still unsolved, so it is usable ONLY with frozen taps.
//
// FIXED -- profile provenance. The receiver runs a control-first hypothesis (QPSK
// R1/4, pilot spacing 5) before the data profile (spacing 8), so a "first capture
// per frame" rule stored H_channel * X_spacing8 / X_spacing5: |H| clean, adjacent
// phases jumping 50-140 deg. The capture site now accepts only a pass whose geometry
// matches genie_true_h::expect(). Identity control went 4.401 (+6.44 dB) ->
// 0.0202 (-16.95 dB), with rejected_profile_captures == frames.
//
// NOT FIXED -- temporal provenance. Truth taps are read from the channel AFTER
// runFrame(), i.e. the END-OF-BURST state, while the estimate is captured at the LTS
// near the START of the burst. transmitFromA() advances the Watterson taps across the
// whole burst before any decode happens. On a fading channel the comparison therefore
// spans that interval and measures channel EVOLUTION plus estimator error. Measured
// on ITU Good @20, n=30: 0.0685 with Doppler live vs 0.0268 frozen for QPSK R3/4, and
// the ranking tracks FRAME LENGTH (QPSK 89856 samples worst, QAM16 R3/4 74880 best),
// which is the signature of a time gap rather than of estimator quality.
//
// Enforced: --chest-nmse now REFUSES to run on GOOD/MODERATE/POOR unless
// ULTRA_CHANNEL_DOPPLER_HZ=0. To lift that restriction, record Watterson tap truth
// indexed by ABSOLUTE SAMPLE and query it at the exact LTS FFT window -- the same
// machinery then serves every post-Wiener data symbol.
//
// STILL ONLY AN INITIALIZATION METRIC. Even time-aligned, this is the LTS estimate.
// Production refines per-symbol via pilots + Wiener immediately before equalize()
// (ofdm_stream_processor.cpp:1146). The Phase 3 question needs H_used at the equalizer
// input, and FER -- not NMSE -- as the stopping criterion.
//
// === Phase 0 payoff: estimator NMSE against simulator truth =================
//
// THE question the whole estimator thread turns on: how far is our channel
// estimate from the true channel? Two oracles tried to answer it indirectly and
// both were invalid. This measures it directly.
//
// DOMAIN CONTRACT (pinned by tests/test_channel_truth_h.cpp): remove a global
// complex scale and a linear phase ramp, and NOTHING more. Those two are what an
// OFDM receiver absorbs (phase reference, FFT-window timing). Removing less
// reports a wrong answer; removing more hides real estimator error.
struct ChestNmseAccum {
    double sum_nmse = 0.0;
    double sum_delay = 0.0;
    long frames = 0;
};

const ultra::ota_channel_core::WattersonChannel* wattersonOf(
    const ultra::ota_channel_core::SimulatedChannel& channel) {
    const auto* model = channel.modelAToBForDiagnostics();
    const auto* watterson =
        dynamic_cast<const ultra::ota_channel_core::WattersonChannelModel*>(model);
    return watterson ? &watterson->channelForDiagnostics() : nullptr;
}

void accumulateChestNmse(const ultra::ota_channel_core::SimulatedChannel& channel,
                         const ultra::ModemConfig& cfg,
                         ChestNmseAccum& acc) {
    namespace gtrue = ultra::ofdm::genie_true_h;
    const auto& est = gtrue::buffer();
    if (est.empty()) {
        return;
    }
    const auto* w = wattersonOf(channel);
    if (w == nullptr) {
        return;  // AWGN path is not a Watterson model; no taps to read.
    }

    std::vector<int> all, data, pilot;
    std::vector<size_t> dlog, plog;
    std::vector<bool> ispilot;
    std::vector<ultra::Complex> pseq;
    ultra::ofdm_pilots::buildCarrierPattern(cfg, 0, all, data, pilot, dlog, plog,
                                            ispilot, pseq);

    std::vector<double> freqs;
    std::vector<std::complex<double>> e, t;
    for (int idx : all) {
        if (static_cast<size_t>(idx) >= est.size()) {
            return;  // geometry mismatch; refuse to report a number
        }
        const float f = ultra::ofdm_pilots::carrierFrequencyHz(cfg, idx);
        const auto th = w->trueFrequencyResponse(f);
        if (std::abs(th) < 1e-9f || std::abs(est[idx]) < 1e-12f) {
            continue;
        }
        freqs.push_back(static_cast<double>(f));
        e.emplace_back(est[idx].real(), est[idx].imag());
        t.emplace_back(th.real(), th.imag());
    }
    if (freqs.size() < 8) {
        return;
    }
    if (acc.frames == 0 && std::getenv("ULTRA_CHEST_NMSE_DUMP")) {
        std::cerr << "# chest dump: est.size=" << est.size()
                  << " cfg.fft=" << cfg.fft_size << " Nc=" << cfg.num_carriers
                  << " carriers_used=" << freqs.size() << "\n";
        // Unoccupied bins first: if these hold data too, the array is not what we
        // think it is. DC(0) is the skipped centre; 100/500/900 are outside the
        // occupied sets (1..30, 995..1023).
        for (int probe : {0, 100, 500, 900}) {
            std::cerr << "#   UNOCCUPIED bin " << probe << " |est|="
                      << std::abs(est[probe]) << "\n";
        }
        // ADJACENT carriers -- sampling every 6th aliases a phase ramp into
        // apparent scatter, which is how this looked like non-ramp noise.
        for (size_t i = 0; i < freqs.size() && i < 12; ++i) {
            std::cerr << "#   f=" << freqs[i]
                      << "  |est|=" << std::abs(e[i])
                      << "  |truth|=" << std::abs(t[i])
                      << "  arg(est/truth)=" 
                      << std::arg(e[i] / t[i]) * 180.0 / M_PI << " deg\n";
        }
    }

    // Remove the linear phase ramp by SEARCHING over candidate delays.
    //
    // An unwrap-based fit CANNOT work here and that is not a tuning detail: the RX
    // sync convention sits up to ~0.7 symbol off the TX grid (measured +796 samples
    // on ITU Good, channel_equalizer_equalize.cpp:462-465). At 46.875 Hz carrier
    // spacing, 800 samples is 2*pi*800/1024 = 4.9 rad of phase step PER CARRIER --
    // more than a full turn -- so the ramp aliases and no unwrapping can recover it.
    // Measured that way this returned NMSE +15.1 dB (estimate and truth unrelated).
    // A search over delay is immune to the aliasing.
    auto residualForDelay = [&](double d_samples, std::complex<double>* alpha_out) {
        std::complex<double> snum(0, 0);
        double sden = 0.0;
        std::vector<std::complex<double>> aligned(freqs.size());
        for (size_t i = 0; i < freqs.size(); ++i) {
            const double a = -2.0 * M_PI * freqs[i] * d_samples / 48000.0;
            aligned[i] = e[i] * std::complex<double>(std::cos(-a), std::sin(-a));
            snum += std::conj(t[i]) * aligned[i];
            sden += std::norm(t[i]);
        }
        if (sden <= 0.0) {
            return 1e30;
        }
        const std::complex<double> alpha = snum / sden;
        if (std::abs(alpha) < 1e-12) {
            return 1e30;
        }
        double num = 0.0, dnm = 0.0;
        for (size_t i = 0; i < freqs.size(); ++i) {
            const std::complex<double> scaled = aligned[i] / alpha;
            num += std::norm(scaled - t[i]);
            dnm += std::norm(t[i]);
        }
        if (alpha_out) {
            *alpha_out = alpha;
        }
        return num / dnm;
    };

    // Coarse sweep over +/- one symbol (fft + CP), then refine.
    double best_d = 0.0;
    double best_r = 1e30;
    if (std::getenv("ULTRA_CHEST_DELAY_SCAN")) {
        std::cerr << "# delay scan (residual vs candidate delay):\n";
        for (double d = -2560.0; d <= 2560.0; d += 64.0) {
            std::cerr << "#   d=" << d << "  resid=" << residualForDelay(d, nullptr)
                      << "\n";
        }
    }
    for (double d = -1200.0; d <= 1200.0; d += 0.5) {
        const double r = residualForDelay(d, nullptr);
        if (r < best_r) {
            best_r = r;
            best_d = d;
        }
    }
    for (double d = best_d - 1.0; d <= best_d + 1.0; d += 0.02) {
        const double r = residualForDelay(d, nullptr);
        if (r < best_r) {
            best_r = r;
            best_d = d;
        }
    }

    acc.sum_nmse += best_r;
    acc.sum_delay += best_d;
    ++acc.frames;
}

Counts measure(const Args& args) {
    gui::StreamingEncoder encoder;
    configureEncoder(encoder, args);

    gui::StreamingDecoder decoder;
    configureDecoder(decoder, args);

    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(args.seed);
    channel.configure(args.snr_db, args.channel);

    // 2026-07-29 diag: shadow channel + decoder for the perfect-CSI bound. Same
    // seed and channel type => same fade realization; kGenieCleanSnrDb is high
    // enough that the LTS estimate taken from it is the true H to well below any
    // level that could matter. Noise is left ENABLED rather than switched off so
    // the model draws the same number of RNG samples per call as the real
    // channel, which is what keeps the two fade trajectories aligned.
    std::unique_ptr<gui::StreamingDecoder> clean_decoder;
    std::unique_ptr<ultra::ota_channel_core::SimulatedChannel> clean_channel;
    if (args.genie_true_h) {
        clean_decoder = std::make_unique<gui::StreamingDecoder>();
        configureDecoder(*clean_decoder, args);
        clean_channel = std::make_unique<ultra::ota_channel_core::SimulatedChannel>();
        clean_channel->setSeed(args.seed);
        clean_channel->configure(args.genie_clean_snr_db, args.channel);
    }

    std::mt19937_64 payload_rng(args.seed ^ 0xA6E22C15D9B3F1A5ull);
    Counts counts;
    ChestNmseAccum chest_acc;
    la::CalibrationTable shadow_table;
    ShadowAccum shadow;
    if (!args.calib_path.empty()) {
        shadow_table = loadCalibration(args.calib_path);
        if (!shadow_table.usable()) {
            throw std::runtime_error(
                "calibration table unusable (version mismatch or empty): " +
                args.calib_path + " -- refusing to score against a stale table");
        }
    }
    std::ofstream gamma_out;
    if (!args.gamma_csv.empty()) {
        gamma_out.open(args.gamma_csv, std::ios::app);
    }
    if (args.chest_nmse) {
        // FAIL FAST on a moving channel. Truth taps are read AFTER runFrame() and so
        // report the END-OF-BURST state, while the estimate is captured at the LTS near
        // the START. On a fading channel that difference is TEMPORAL PROVENANCE ERROR,
        // not estimator error -- it silently inflated a published table on 2026-07-29
        // (see the CHANGELOG retraction) despite this file already documenting the
        // precondition. A comment was not enough; this is now enforced.
        const bool fading_channel =
            args.channel == ultra::ota_channel_core::ChannelType::GOOD ||
            args.channel == ultra::ota_channel_core::ChannelType::MODERATE ||
            args.channel == ultra::ota_channel_core::ChannelType::POOR;
        const char* dop = std::getenv("ULTRA_CHANNEL_DOPPLER_HZ");
        const bool doppler_frozen = dop != nullptr && std::atof(dop) == 0.0;
        if (fading_channel && !doppler_frozen) {
            throw std::runtime_error(
                "--chest-nmse on a fading channel requires ULTRA_CHANNEL_DOPPLER_HZ=0. "
                "Truth taps are sampled after the burst; with Doppler active the "
                "comparison spans LTS-time to end-of-burst and measures channel "
                "EVOLUTION, not estimator error. Time-stamped truth is not implemented "
                "yet -- see docs/CHANGELOG.md 2026-07-29 retraction.");
        }
    }

    for (int i = 0; i < args.n; ++i) {
        if (ultra::genie::txCapture().enabled) {
            ultra::genie::txCapture().reportIfDebug("prev frame");
            ultra::genie::txCapture().reset();
        }
        const auto trial = makeFrame(encoder, args, payload_rng, i);
        if (args.config == MeasureConfig::WarmSyncLight) {
            const uint16_t anchor_seq =
                static_cast<uint16_t>((0x8000u + static_cast<unsigned>(i)) & 0xFFFFu);
            runWarmSyncTrial(encoder, decoder, channel, trial, anchor_seq, counts);
            continue;
        }

        const bool full_preamble = usesFullPreamble(args.config);
        // Data4Full decodes on the connected path; its full chirp preamble must
        // be admitted as an anchor so sync succeeds at any fade depth (isolating
        // decode-diversity from light-sync acquisition). AckFull uses the cold
        // setMode path and does not need anchor admission.
        const bool expect_anchor = args.config == MeasureConfig::Data4Full;

        // 2026-07-29 diag: clean pass first -- capture the noiseless H over this
        // frame's fade realization -- then the real pass with that H injected.
        if (args.genie_true_h) {
            // NB: alias must not be named `genie` -- ultra::genie already exists
            // (genie_tx_capture.hpp) and would shadow-collide here.
            namespace gtrue = ultra::ofdm::genie_true_h;
            gtrue::buffer().clear();
            gtrue::mode() = gtrue::Mode::Capture;
            runFrame(*clean_decoder, *clean_channel, trial.tx_samples, full_preamble,
                     /*reset_decoder=*/true, /*expect_full_anchor=*/expect_anchor,
                     /*fixed_budget=*/true);
            gtrue::mode() = gtrue::Mode::Inject;
            auto outcome = runFrame(decoder, channel, trial.tx_samples, full_preamble,
                                    /*reset_decoder=*/true,
                                    /*expect_full_anchor=*/expect_anchor,
                                    /*fixed_budget=*/true);
            gtrue::mode() = gtrue::Mode::Off;
            classify(outcome, trial.frame_bytes, counts);
            continue;
        }

        if (!args.gamma_csv.empty()) {
            namespace gtrue = ultra::ofdm::genie_true_h;
            gtrue::buffer().clear();
            {   // same provenance gate as --chest-nmse: only the pass whose geometry
                // matches the transmitted profile, never the control-first peek.
                const auto want = makeOFDMConfig(args.mod, args.rate, args.cp_name);
                auto& x = gtrue::expect();
                x.active = true;
                x.pilot_spacing = static_cast<uint32_t>(want.pilot_spacing);
                x.modulation = static_cast<int>(want.modulation);
                x.num_carriers = static_cast<uint32_t>(want.num_carriers);
                x.fft_size = static_cast<uint32_t>(want.fft_size);
            }
            gtrue::mode() = gtrue::Mode::Capture;
            auto outcome = runFrame(decoder, channel, trial.tx_samples, full_preamble,
                                    /*reset_decoder=*/true,
                                    /*expect_full_anchor=*/expect_anchor);
            gtrue::mode() = gtrue::Mode::Off;

            const bool ok = isExpectedSuccess(outcome, trial.frame_bytes);
            const auto& est = gtrue::buffer();
            const float nv = std::max(gtrue::capturedNoiseVar(), 1e-12f);
            if (!est.empty() && gamma_out.is_open()) {
                const auto cfg = makeOFDMConfig(args.mod, args.rate, args.cp_name);
                std::vector<int> all, data, pilot;
                std::vector<size_t> dlog, plog;
                std::vector<bool> isp;
                std::vector<ultra::Complex> pseq;
                ultra::ofdm_pilots::buildCarrierPattern(cfg, 0, all, data, pilot,
                                                        dlog, plog, isp, pseq);
                gamma_out << args.snr_db << ',' << args.channel_name << ','
                          << args.mod_name << ',' << args.rate_name << ','
                          << args.seed << ',' << i << ',' << (ok ? 1 : 0)
                          << ',' << gtrue::capturedSymbolMeanCv();
                for (int idx : data) {
                    if (static_cast<size_t>(idx) < est.size()) {
                        gamma_out << ',' << (std::norm(est[idx]) / nv);
                    }
                }
                gamma_out << '\n';
            }

            // Phase 6 shadow scoring on the SAME grid, raw (un-renormalized) gamma.
            if (shadow_table.usable() && !est.empty()) {
                const auto cfg2 = makeOFDMConfig(args.mod, args.rate, args.cp_name);
                std::vector<int> a2, d2, p2;
                std::vector<size_t> dl2, pl2;
                std::vector<bool> ip2;
                std::vector<ultra::Complex> ps2;
                ultra::ofdm_pilots::buildCarrierPattern(cfg2, 0, a2, d2, p2, dl2, pl2,
                                                        ip2, ps2);
                std::vector<float> grid;
                grid.reserve(d2.size());
                for (int idx : d2) {
                    if (static_cast<size_t>(idx) < est.size()) {
                        grid.push_back(std::norm(est[idx]) / nv);
                    }
                }
                if (!grid.empty()) {
                    const auto scored = la::scoreAll(shadow_table, grid);
                    const auto* txc = shadow_table.find(args.mod, args.rate);
                    if (txc != nullptr && txc->valid) {
                        const double geff =
                            la::toDb(la::effectiveSnrEesm(grid, txc->beta));
                        const double pper = la::predictPer(*txc, geff);
                        shadow.pred_per_sum += pper;
                        shadow.actual_fail += ok ? 0 : 1;
                        ++shadow.frames;
                        shadow.actual_eff = la::bitsPerCarrier(args.mod, args.rate);
                        if (pper <= 0.10) {
                            ++shadow.confident_total;
                            if (!ok) ++shadow.confident_but_failed;
                        }
                        const auto* pick =
                            la::selectUnderReliability(scored, args.shadow_max_per);
                        if (pick == nullptr) {
                            ++shadow.shadow_none;
                        } else {
                            shadow.shadow_eff_sum += pick->efficiency;
                            const double txe = shadow.actual_eff;
                            if (std::abs(pick->efficiency - txe) < 1e-6) ++shadow.shadow_same;
                            else if (pick->efficiency < txe) ++shadow.shadow_lower;
                            else ++shadow.shadow_higher;
                        }
                    }
                }
            }
            classify(outcome, trial.frame_bytes, counts);
            continue;
        }

        if (args.chest_nmse) {
            namespace gtrue = ultra::ofdm::genie_true_h;
            gtrue::buffer().clear();
            {   // Accept only the pass matching the TRANSMITTED profile. Without this
                // the control-first QPSK R1/4 peek is captured instead of the data pass.
                const auto want = makeOFDMConfig(args.mod, args.rate, args.cp_name);
                auto& x = gtrue::expect();
                x.active = true;
                x.pilot_spacing = static_cast<uint32_t>(want.pilot_spacing);
                x.modulation = static_cast<int>(want.modulation);
                x.num_carriers = static_cast<uint32_t>(want.num_carriers);
                x.fft_size = static_cast<uint32_t>(want.fft_size);
            }
            gtrue::mode() = gtrue::Mode::Capture;   // capture only; never inject
            auto outcome = runFrame(decoder, channel, trial.tx_samples, full_preamble,
                                    /*reset_decoder=*/true,
                                    /*expect_full_anchor=*/expect_anchor);
            gtrue::mode() = gtrue::Mode::Off;
            accumulateChestNmse(channel, makeOFDMConfig(args.mod, args.rate, args.cp_name), chest_acc);
            classify(outcome, trial.frame_bytes, counts);
            continue;
        }

        auto outcome = runFrame(decoder, channel, trial.tx_samples, full_preamble,
                                /*reset_decoder=*/true,
                                /*expect_full_anchor=*/expect_anchor,
                                /*fixed_budget=*/args.fixed_budget,
                                /*preroll_samples=*/static_cast<size_t>(args.preroll_ms) * 48);
        classify(outcome, trial.frame_bytes, counts);
    }

    // 2026-07-29 diag: null control. If captures or injections is 0 the oracle
    // never engaged and any "no effect" reading is meaningless.
    if (shadow.frames > 0) {
        const double pred = shadow.pred_per_sum / static_cast<double>(shadow.frames);
        const double act = static_cast<double>(shadow.actual_fail) /
                           static_cast<double>(shadow.frames);
        std::cerr << "# shadow frames=" << shadow.frames
                  << " pred_per=" << pred << " actual_per=" << act
                  << " calib_err=" << (pred - act)
                  << " agree=" << shadow.shadow_same
                  << " would_demote=" << shadow.shadow_lower
                  << " would_promote=" << shadow.shadow_higher
                  << " no_pick=" << shadow.shadow_none
                  << " confident_but_failed=" << shadow.confident_but_failed
                  << "/" << shadow.confident_total
                  << "\n";
    }
    if (args.chest_nmse) {
        const double mean = chest_acc.frames
            ? chest_acc.sum_nmse / static_cast<double>(chest_acc.frames) : -1.0;
        std::cerr << "# chest_nmse rejected_profile_captures="
                  << ultra::ofdm::genie_true_h::stats().rejected_profile
                  << " frames=" << chest_acc.frames
                  << " mean_nmse=" << mean
                  << " mean_nmse_db=" << (mean > 0 ? 10.0 * std::log10(mean) : 0.0)
                  << " mean_fitted_delay_samples="
                  << (chest_acc.frames
                          ? chest_acc.sum_delay / static_cast<double>(chest_acc.frames)
                          : 0.0)
                  << "\n";
    }
    if (args.genie_true_h) {
        const auto& s = ultra::ofdm::genie_true_h::stats();
        std::cerr << "# genie_true_h: captures=" << s.captures
                  << " injections=" << s.injections
                  << " misses=" << s.misses
                  << " mean_nmse_vs_replaced="
                  << (s.nmse_count ? s.nmse_sum / static_cast<double>(s.nmse_count) : -1.0)
                  << "\n";
    }

    return counts;
}

// === Burst-chunk (file-class keystone) measurement ===========================
// THE keystone validation: send a CHUNK of N file frames as ONE burst (full
// chirp anchor + N-1 light preambles, per encodeBurstLight), with cross-frame
// deep interleave ON vs OFF, through a fading channel, and measure how many of
// the N frames RECOVER. The premise of the file-class architecture is that a
// deep fade null that kills an ISOLATED frame only NICKS each codeword when the
// codeword is spread across a chunk longer than the fade, so FEC recovers it.
// That is a block-level / cross-frame property; single-frame FER cannot show it.
struct BurstCounts {
    int chunks = 0;
    int chunk_sync_fail = 0;  // anchor never synced → whole chunk lost
    int frames_total = 0;
    int frames_recovered = 0;
    int chunks_complete = 0;  // all N frames in the chunk recovered
};

BurstCounts measureBurst(const Args& args) {
    int group = args.group > 0 ? args.group : 8;
    // Persistent channel: the Watterson fade advances continuously across chunks
    // so successive chunks sample different fade phases (a real distribution).
    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(args.seed);
    channel.configure(args.snr_db, args.channel);

    std::mt19937_64 payload_rng(args.seed ^ 0xA6E22C15D9B3F1A5ull);
    BurstCounts bc;
    const std::vector<float> preburst_idle(kPumpChunkSamples, 0.0f);

    for (int c = 0; c < args.n; ++c) {
        // Fresh capture per chunk: encoder pushes during encode, decoder reads in the
        // same data-symbol order during decode.
        if (ultra::genie::txCapture().enabled) {
            ultra::genie::txCapture().reportIfDebug("prev chunk");
            ultra::genie::txCapture().reset();
        }
        // Fresh encoder + decoder per chunk → each chunk starts from idle, so
        // encodeBurstLight emits a FULL chirp anchor (reliable sync at any fade
        // depth); the channel persists (fade keeps advancing).
        gui::StreamingEncoder encoder;
        configureEncoder(encoder, args);
        encoder.setBurstInterleave(args.burst_interleave);
        encoder.setBurstInterleaveGroupSize(group);

        gui::StreamingDecoder decoder;
        configureDecoder(decoder, args);  // connected path (BurstChunk != AckFull)
        decoder.setBurstInterleave(args.burst_interleave);
        decoder.setBurstInterleaveGroupSize(group);
        // ULTRA_MEASURE_BURST_NO_ANCHOR_WAIT=1 (default OFF, diagnostic): skip the
        // full-anchor demand so the LIGHT-LTS marked group start (data-sync corr
        // ~0.26, below the 0.52 full-anchor threshold) is accepted and burst
        // accumulation can arm on the production warm-handoff preamble.
        {
            const char* e = std::getenv("ULTRA_MEASURE_BURST_NO_ANCHOR_WAIT");
            if (!(e != nullptr && e[0] == '1')) {
                decoder.expectFullOFDMAnchorOnce();
            }
        }

        // §14.17 self-describing burst: when enabled, the encoder emits a full-anchor
        // BURST_HEADER descriptor declaring the group params. To prove the descriptor
        // actually drives the decoder (the cross-station 0/8 fix), deliberately
        // mis-configure the decoder here: wrong group size + interleave OFF. If the
        // descriptor works, the RX reconfigures itself and still recovers the group.
        // The BURST_HEADER descriptor is REQUIRED for Z=81: the RX learns the lifting
        // size only from the wire descriptor (decodeFixedFrame ldpc_z <- last_burst_
        // descriptor_.lifting_z). With Z=81 and no descriptor the RX decodes N=1944
        // soft bits with a Z=27 matrix → 0% (BUG-HARNESS-002 Defect 3). So force the
        // descriptor on whenever Z=81 (production always sends it for burst), and also
        // when the §14.17 descriptor-proof test is requested.
        const bool emit_descriptor = args.burst_descriptor || args.ldpc_z == 81;
        if (emit_descriptor) {
            encoder.setBurstDescriptorEnabled(true);
            encoder.setBurstDescriptorIdentity("ALPHA", "BRAVO");
        }
        if (args.burst_descriptor) {
            // §14.17 proof: deliberately mis-config the RX; if the descriptor works
            // the RX reconfigures itself and still recovers the group.
            const int wrong_group = (group >= 4) ? 2 : 8;
            decoder.setBurstInterleaveGroupSize(wrong_group);
            decoder.setBurstInterleave(false);
        }

        const int clamped = encoder.getBurstInterleaveGroupSize();  // [2,8]
        std::vector<Bytes> frames;
        std::vector<Bytes> expected;
        frames.reserve(static_cast<size_t>(clamped));
        for (int f = 0; f < clamped; ++f) {
            Bytes payload = randomPayload(payload_rng, 20);
            payload[0] = static_cast<uint8_t>(c & 0xFF);
            payload[1] = static_cast<uint8_t>(f & 0xFF);  // make each frame unique
            const uint16_t seq = static_cast<uint16_t>((c * clamped + f) & 0xFFFFu);
            auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", seq, payload,
                                                args.rate,
                                                args.frame_cw);
            Bytes ser = frame.serialize();
            frames.push_back(ser);
            expected.push_back(std::move(ser));
        }

        // encodeBurstLight() does not transmit the ARQ-owned bytes verbatim: for a
        // multi-frame, non-interleaved OFDM turn it clears stale physical-tail bits
        // and stamps PHYSICAL_BURST_END into the final private wire copy.  Compare
        // against that actual wire image.  Otherwise a perfect N/N decode is reported
        // as (N-1)/N solely because the final frame's CRC-protected flags byte differs.
        // This used to make every group=2 AWGN Z=81 control read 1/2 and zero complete
        // chunks even though the decoder log showed both frames successful.
        expected = gui::streaming_frame_policy::preparePhysicalBurstFrames(
            expected,
            gui::streaming_frame_policy::shouldStampPhysicalBurstEnd(
                expected.size(), protocol::WaveformMode::OFDM_CHIRP,
                /*supports_data_preamble=*/true, args.burst_interleave));

        // HARNESS FIDELITY (2026-07-29): FULL chirp+LTS anchor at the group start.
        //
        // Without this the harness could never exercise the burst-transport RX path
        // at all. encodeBurstLight's default group-start preamble is the §16.8
        // warm-handoff LIGHT LTS, whose data-sync correlation reads ~0.26 — below the
        // 0.52 full-anchor threshold that decoder.expectFullOFDMAnchorOnce() (set
        // above, per chunk) enforces. So the marked group-start frame was REJECTED
        // three times and the decoder instead accepted a LATER, unmarked member frame
        // (corr 0.95). wasBurstInterleaved() is therefore false at the burst_marker
        // test (streaming_ofdm_decode.cpp:1568), "entering accumulation" never fires,
        // and every frame is decoded as an isolated frame — measured: 0 "Burst frame
        // N/M demodulated" lines in a full DEBUG trace of a burst_chunk run.
        //
        // Production does not have this problem: it forces the full anchor on the
        // session's first burst (forceNextFrameFullPreamble on entering connected
        // OFDM) and on every resend. Each chunk here builds a FRESH encoder, i.e. a
        // fresh session, so the production-equivalent per-request full-anchor option
        // must be supplied for each chunk. It reaches only the DATA group-start; the
        // BURST_HEADER descriptor cannot consume or detach it from this encode.
        //
        // Env-gated (ULTRA_MEASURE_BURST_FULL_ANCHOR=1, default OFF) so the historical
        // burst_chunk numbers stay reproducible and the two regimes can be A/B'd.
        gui::BurstAnchorOptions burst_anchor_options;
        {
            const char* e = std::getenv("ULTRA_MEASURE_BURST_FULL_ANCHOR");
            if (e != nullptr && e[0] == '1') {
                burst_anchor_options.force_full_group_start = true;
            }
        }

        const auto tx = encoder.encodeBurstLight(frames, burst_anchor_options);
        if (tx.empty()) {
            throw std::runtime_error("burst encoder produced no samples");
        }

        std::vector<Bytes> recovered;
        bool any_sync = false;

        // HARNESS FIDELITY (2026-07-29): COLLECT THE BURST-TRANSPORT DELIVERY.
        //
        // finalizeBurstGroup routes decoded frames two different ways
        // (streaming_burst_interleave.cpp:1244): with burst_transport_rx_ set — the
        // production default, and the ONLY regime in which the burst-transport RX
        // path runs at all — successful logical frames are collected into
        // burst_group_frames and delivered SOLELY through burst_group_callback_; the
        // per-frame frame_queue_ push is deliberately suppressed so the file group is
        // not double-processed through onRxData. This harness never set that callback,
        // so every burst-delivered frame was dropped on the floor and
        // frames_recovered read 0 no matter how well the group decoded. Measured:
        // AWGN@60 16QAM R2/3, --burst-descriptor 1 → 60 "decode SUCCESS" / 0 "decode
        // FAILED" in the DEBUG trace, and frames_recovered = 0/32. The decode was
        // perfect; the harness was not listening.
        decoder.setBurstGroupCallback(
            [&recovered](uint16_t /*group_seq*/, const std::vector<Bytes>& group_frames,
                         bool /*all_ok*/, float /*quality*/, uint16_t /*frame_mask*/,
                         bool /*interleaved*/, uint8_t /*group_size*/,
                         bool /*geometry_proven*/) {
                for (const auto& f : group_frames) {
                    recovered.push_back(f);
                }
            });

        // A production receiver is already consuming idle audio before PTT opens.
        // A fresh harness decoder was instead born on the descriptor's very first
        // sample. Its first 4800-sample process call initializes correlation_pos at
        // the write head, permanently skipping a chirp that starts at absolute zero.
        // Pump one real channel interval first: this advances the fading/noise model
        // and initializes the decoder cursor while the medium is idle, then leaves
        // the descriptor chirp wholly searchable from its first sample.
        channel.transmitFromA(preburst_idle);
        auto idle_rx = channel.receiveForB(preburst_idle.size());
        decoder.feedAudio(idle_rx.data(), idle_rx.size());
        decoder.processBuffer();

        channel.transmitFromA(tx);

        // HARNESS FIDELITY (2026-07-28): two floors on how long we pump.
        //
        // (1) kFullSearchSamples. The full-anchor chirp search refuses to run until it
        //     has 2.5 s of unsearched audio ("searchForSync: SKIP unsearched<min"), so a
        //     short burst (e.g. --frame-cw 1, 121344 samples) never got searched at all
        //     and reported chunk_sync_fail on EVERY chunk. Measured 5/5 -> 0/5.
        //
        // (2) The trailing allowance. The burst decoder re-decodes each frame several
        //     times (control-first peek, header peek, authoritative pass, sync-recovery
        //     retries) and runs behind live, so it still emits frames well after the
        //     last transmitted sample. The old 8 chunks (0.8 s) truncated that backlog
        //     and UNDER-REPORTED recovery. Measured, QPSK R3/4 Good@60 seed 7 n=10:
        //     8 -> 60/80, 16 -> 70/80, and 32/64/128/256 -> 70/80 (saturated). 16 it is
        //     — a measurement floor, not a tuning knob.
        constexpr size_t kTrailingPumpChunks = 16;
        const size_t target = std::max(tx.size() + kTrailingPumpChunks * kPumpChunkSamples,
                                       kFullSearchSamples + kTrailingPumpChunks * kPumpChunkSamples);
        size_t received = 0;
        while (received < target) {
            const size_t take = std::min(kPumpChunkSamples, target - received);
            auto rx = channel.receiveForB(take);
            decoder.feedAudio(rx.data(), rx.size());
            decoder.processBuffer();
            any_sync = any_sync || decoder.isSynced();
            while (decoder.hasFrame()) {
                auto r = decoder.getFrame();
                if (r.is_ping) {
                    continue;
                }
                if (r.success) {
                    recovered.push_back(r.frame_data);
                }
            }
            received += take;
        }

        int rec = 0;
        for (const auto& e : expected) {
            for (const auto& g : recovered) {
                if (bytesMatch(e, g)) {
                    ++rec;
                    break;
                }
            }
        }

        ++bc.chunks;
        bc.frames_total += clamped;
        bc.frames_recovered += rec;
        if (rec == clamped) {
            ++bc.chunks_complete;
        }
        if (rec == 0 && !any_sync) {
            ++bc.chunk_sync_fail;
        }
    }

    return bc;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        ultra::LogLevel log_level = ultra::LogLevel::ERROR;
        if (const char* env = std::getenv("ULTRA_LOG_LEVEL")) {
            ultra::parseLogLevel(env, log_level);
        }
        ultra::setLogLevel(log_level);
        const Args args = parseArgs(argc, argv);

        // 2026-05-29 diag: enable the true per-symbol data-aided channel genie
        // (H=Y/X) process-wide. Off unless ULTRA_GENIE_DATA_AIDED=1. Both the
        // frame path (measure) and burst path (measureBurst) reset per iteration.
        {
            const char* env = std::getenv("ULTRA_GENIE_DATA_AIDED");
            ultra::genie::txCapture().enabled = (env && std::atoi(env) == 1);
        }

        if (args.config == MeasureConfig::BurstChunk) {
            const BurstCounts bc = measureBurst(args);
            // CSV: snr,config,channel,mod,rate,bi,group,seed,chunks,frames_total,
            //      frames_recovered,chunks_complete,chunk_sync_fail
            std::cout << args.snr_db << ","
                      << args.config_name << ","
                      << args.channel_name << ","
                      << args.mod_name << ","
                      << args.rate_name << ","
                      << (args.burst_interleave ? 1 : 0) << ","
                      << (args.group > 0 ? args.group : 8) << ","
                      << args.seed << ","
                      << bc.chunks << ","
                      << bc.frames_total << ","
                      << bc.frames_recovered << ","
                      << bc.chunks_complete << ","
                      << bc.chunk_sync_fail << "\n";
            return 0;
        }

        const Counts counts = measure(args);
        const int total = counts.sync_fail + counts.decode_fail +
                          counts.crc_fail + counts.pass;
        if (total != args.n) {
            std::cerr << "internal count mismatch: got " << total
                      << " expected " << args.n << "\n";
            return 2;
        }

        std::cout << args.snr_db << ","
                  << args.config_name << ","
                  << args.channel_name << ","
                  << args.mod_name << ","
                  << args.rate_name << ","
                  << (args.carrier_interleave ? 1 : 0) << ","
                  << args.seed << ","
                  << args.n << ","
                  << counts.sync_fail << ","
                  << counts.decode_fail << ","
                  << counts.crc_fail << ","
                  << counts.pass << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "measure_ack_fer: " << e.what() << "\n";
        return 1;
    }
}
