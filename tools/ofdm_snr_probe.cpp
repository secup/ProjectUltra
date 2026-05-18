#include "fec/frame_interleaver.hpp"
#include "protocol/frame_v2.hpp"
#include "sim/cli_enums.hpp"
#include "sim/simulated_station.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

using ultra::Bytes;
using ultra::CodeRate;
using ultra::ModemConfig;
using ultra::Modulation;
using ultra::OFDMChirpWaveform;
using ultra::SampleSpan;
using ultra::Samples;
namespace cli = ultra::tools::cli;
namespace v2 = ultra::protocol::v2;

struct Args {
    float snr_db = 15.0f;
    ::ChannelType channel = ::ChannelType::AWGN;
    CodeRate rate = CodeRate::R1_2;
    Modulation mod = Modulation::DQPSK;
    int cw_count = 4;
    uint32_t seed = 42;
    size_t payload_bytes = 32;
    bool header = true;
};

void usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [--snr DB] [--channel awgn|good|moderate|poor|flutter]\n"
        << "       [--rate r1_4|r1_2|r2_3|r3_4] [--mod dqpsk|d8psk]\n"
        << "       [--cw-count N] [--seed N] [--payload BYTES]\n";
}

const char* channelName(::ChannelType channel) {
    switch (channel) {
        case ::ChannelType::AWGN: return "AWGN";
        case ::ChannelType::GOOD: return "GOOD";
        case ::ChannelType::MODERATE: return "MODERATE";
        case ::ChannelType::POOR: return "POOR";
        case ::ChannelType::FLUTTER: return "FLUTTER";
        default: return "UNKNOWN";
    }
}

bool parseArgs(int argc, char** argv, Args& args) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--snr") {
            const char* v = need("--snr");
            if (!v) return false;
            args.snr_db = std::stof(v);
        } else if (arg == "--channel") {
            const char* v = need("--channel");
            if (!v) return false;
            auto parsed = cli::parseChannelType(v);
            if (!parsed) {
                std::cerr << "Unknown channel: " << v << "\n";
                return false;
            }
            args.channel = *parsed;
        } else if (arg == "--rate") {
            const char* v = need("--rate");
            if (!v) return false;
            auto parsed = cli::parseCodeRate(v);
            if (!parsed || *parsed == CodeRate::AUTO) {
                std::cerr << "Unknown rate: " << v << "\n";
                return false;
            }
            args.rate = *parsed;
        } else if (arg == "--mod") {
            const char* v = need("--mod");
            if (!v) return false;
            auto parsed = cli::parseModulation(
                v, cli::AllowAuto::No, cli::AllowExperimentalModulation::Yes);
            if (!parsed) {
                std::cerr << "Unknown modulation: " << v << "\n";
                return false;
            }
            args.mod = *parsed;
        } else if (arg == "--cw-count") {
            const char* v = need("--cw-count");
            if (!v) return false;
            args.cw_count = v2::sanitizeFixedFrameCodewords(std::stoi(v));
        } else if (arg == "--seed") {
            const char* v = need("--seed");
            if (!v) return false;
            args.seed = static_cast<uint32_t>(std::stoul(v));
        } else if (arg == "--payload") {
            const char* v = need("--payload");
            if (!v) return false;
            args.payload_bytes = std::max<size_t>(1, std::stoul(v));
        } else if (arg == "--no-header") {
            args.header = false;
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return false;
        }
    }
    return true;
}

ModemConfig makeConfig(const Args& args) {
    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500;
    cfg.cp_mode = ultra::CyclicPrefixMode::MEDIUM;
    cfg.modulation = args.mod;
    cfg.code_rate = args.rate;
    cfg.use_pilots = true;
    cfg.pilot_spacing =
        ultra::ofdm_link_adaptation::recommendedPilotSpacing(args.mod, args.rate);
    return cfg;
}

int bitsPerOFDMSymbol(const ModemConfig& cfg) {
    return ultra::ofdm_link_adaptation::bitsPerOFDMSymbol(
        static_cast<int>(cfg.num_carriers),
        cfg.use_pilots,
        static_cast<int>(cfg.pilot_spacing),
        cfg.modulation);
}

struct TxFrame {
    Samples samples;
    size_t signal_start = 0;
    size_t frame_samples = 0;
    Bytes serialized_frame;
};

TxFrame buildTxFrame(const Args& args, const ModemConfig& cfg) {
    OFDMChirpWaveform waveform(cfg);
    waveform.configure(args.mod, args.rate);

    Bytes payload(args.payload_bytes);
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>((i * 37u + 11u) & 0xffu);
    }

    const auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", 1, payload,
                                              args.rate, args.cw_count);
    const Bytes frame_data = frame.serialize();
    const Bytes encoded = v2::encodeFixedFrame(frame_data, args.rate,
                                               args.cw_count, true,
                                               static_cast<size_t>(bitsPerOFDMSymbol(cfg)));

    TxFrame tx;
    tx.serialized_frame = frame_data;
    tx.signal_start = 48000;
    tx.samples.reserve(48000 + waveform.getDataPreambleSamples() +
                       waveform.getMinSamplesForCWCount(args.cw_count) + 48000);
    tx.samples.resize(tx.signal_start, 0.0f);

    Samples preamble = waveform.generateDataPreamble();
    Samples data = waveform.modulate(encoded);
    tx.frame_samples = preamble.size() + data.size();
    tx.samples.insert(tx.samples.end(), preamble.begin(), preamble.end());
    tx.samples.insert(tx.samples.end(), data.begin(), data.end());
    tx.samples.resize(tx.samples.size() + 48000, 0.0f);
    return tx;
}

struct ProbeResult {
    bool got_result = false;
    bool success = false;
    int cw_ok = 0;
    int cw_failed = 0;
    float sync_snr_db = 0.0f;
    bool has_pilot_snr = false;
    float pilot_snr_db = 0.0f;
    float lts_snr_db = 0.0f;
    float fading_index = 0.0f;
};

ProbeResult decodeProbe(const Args& args, const ModemConfig& cfg,
                        const TxFrame& tx, const Samples& rx) {
    ProbeResult result;

    OFDMChirpWaveform sync_waveform(cfg);
    sync_waveform.configure(args.mod, args.rate);
    ultra::SyncResult sync;
    if (sync_waveform.detectDataSync(SampleSpan(rx.data(), rx.size()), sync, 0.0f, 0.3f)) {
        result.sync_snr_db = std::clamp((sync.correlation - 0.15f) / 0.03f,
                                        -5.0f, 30.0f);
    }

    if (tx.signal_start + tx.frame_samples > rx.size()) {
        return result;
    }

    OFDMChirpWaveform rx_waveform(cfg);
    rx_waveform.configure(args.mod, args.rate);
    rx_waveform.setFrequencyOffset(0.0f);
    rx_waveform.setAbsoluteTrainingPosition(tx.signal_start);

    const bool ready = rx_waveform.process(
        SampleSpan(rx.data() + tx.signal_start, tx.frame_samples));
    result.got_result = ready;
    result.lts_snr_db = rx_waveform.estimatedSNR();
    result.has_pilot_snr = rx_waveform.hasLastSNREstimate();
    result.pilot_snr_db = rx_waveform.getLastSNREstimate();
    result.fading_index = rx_waveform.getFadingIndex();

    std::vector<float> soft_bits = rx_waveform.getSoftBits();
    if (soft_bits.size() < static_cast<size_t>(args.cw_count) * v2::LDPC_CODEWORD_BITS) {
        result.got_result = false;
        return result;
    }

    auto status = v2::decodeFixedFrame(
        soft_bits, args.rate, args.cw_count, true,
        static_cast<size_t>(bitsPerOFDMSymbol(cfg)));
    result.cw_failed = status.countFailures();
    result.cw_ok = static_cast<int>(status.decoded.size()) - result.cw_failed;
    result.success = status.allSuccess() && status.reassemble() == tx.serialized_frame;
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    if (!parseArgs(argc, argv, args)) {
        return 1;
    }

    ultra::setLogLevel(ultra::LogLevel::ERROR);

    const ModemConfig cfg = makeConfig(args);
    const TxFrame tx = buildTxFrame(args, cfg);

    ::SimulatedChannel channel;
    channel.setSeed(args.seed);
    channel.configure(args.snr_db, args.channel);
    channel.transmitFromA(tx.samples);
    const std::vector<float> rx = channel.receiveForB(tx.samples.size());

    const ProbeResult r = decodeProbe(args, cfg, tx, rx);
    if (args.header) {
        std::cout << "channel,configured_snr,mod,rate,cw_count,success,cw_ok,cw_failed,"
                  << "sync_snr_db,pilot_snr_db,lts_snr_db,fading_index\n";
    }
    std::cout << channelName(args.channel) << ","
              << std::fixed << std::setprecision(2)
              << args.snr_db << ","
              << ultra::modulationToString(args.mod) << ","
              << ultra::codeRateToString(args.rate) << ","
              << args.cw_count << ","
              << (r.success ? 1 : 0) << ","
              << r.cw_ok << ","
              << r.cw_failed << ","
              << r.sync_snr_db << ","
              << (r.has_pilot_snr ? r.pilot_snr_db : 0.0f) << ","
              << r.lts_snr_db << ","
              << r.fading_index << "\n";

    return r.got_result ? 0 : 2;
}
