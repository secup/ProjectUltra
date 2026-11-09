#include "ota_simulator/clip_gen.hpp"
#include "ota_simulator/scenario.hpp"
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/session_manager.hpp"
#include "ota_simulator_service/auth_allowlist.hpp"
#include "ota_simulator_service/ota_simulator_service.hpp"
#include "ultra/version.hpp"

#include <map>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <grpcpp/grpcpp.h>

namespace {

namespace channel = ultra::ota_channel_core;
namespace service = ultra::ota_simulator_service;

volatile std::sig_atomic_t g_stop_signal = 0;

struct BindAddress {
    std::string host = "127.0.0.1";
    uint16_t port = 0;
};

struct ServeOptions {
    BindAddress grpc_bind{.host = "127.0.0.1", .port = 47000};
    BindAddress udp_bind{.host = "127.0.0.1", .port = 0};
    std::filesystem::path tokens_path;
    std::filesystem::path captures_root = "captures";
    channel::ChannelType lobby_channel = channel::ChannelType::AWGN;
    float lobby_snr_db = 15.0f;
    uint64_t lobby_seed = 42;
    uint32_t lobby_station_cap = 16;
    std::map<std::string, float> lobby_rx_cfo_hz;  // --lobby-rx-cfo-hz "BRAVO:2.5,ALPHA:-2.5"
    std::map<std::string, float> lobby_rx_clock_ppm;  // --lobby-rx-clock-ppm "BRAVO:50,ALPHA:-50"
    channel::TxPaConfig lobby_tx_pa;  // --lobby-tx-pa "pep=0.5,drive_db=3" (the senders' ALC/PA)
    uint32_t shutdown_deadline_seconds = 60;
    std::filesystem::path noise_bed_wav;
};

void usage() {
    std::cerr
        << "Usage:\n"
        << "  ota_simulator gen --frame FRAME --callsign CALL [--peer-callsign CALL] --out FILE.wav\n"
        << "  ota_simulator serve --bind HOST:PORT --tokens FILE [--udp-bind HOST:PORT]\n"
        << "      [--noise-bed-wav FILE.wav]\n"
        << "      [--lobby-rx-cfo-hz RX:HZ,...] [--lobby-rx-clock-ppm RX:PPM,...]\n"
        << "      [--lobby-tx-pa on|pep=0.5,drive_db=3,p=2,sat_db=3,attack_ms=2,release_ms=300]\n";
}

void handleStopSignal(int signal) {
    g_stop_signal = signal;
}


std::string requireValue(int& i, int argc, char** argv, const std::string& opt) {
    if (i + 1 >= argc) {
        throw std::runtime_error(opt + " requires a value");
    }
    return argv[++i];
}

uint32_t parseUint32(const std::string& value, uint32_t max_value, const std::string& opt) {
    size_t pos = 0;
    const unsigned long parsed = std::stoul(value, &pos, 10);
    if (pos != value.size() || parsed > max_value) {
        throw std::runtime_error(opt + " is out of range");
    }
    return static_cast<uint32_t>(parsed);
}

uint64_t parseUint64(const std::string& value, const std::string& opt) {
    size_t pos = 0;
    const unsigned long long parsed = std::stoull(value, &pos, 10);
    if (pos != value.size()) {
        throw std::runtime_error(opt + " must be an integer");
    }
    return static_cast<uint64_t>(parsed);
}

BindAddress parseBindAddress(const std::string& value, const std::string& opt) {
    const auto pos = value.rfind(':');
    if (pos == std::string::npos || pos == 0 || pos + 1 == value.size()) {
        throw std::runtime_error(opt + " must be HOST:PORT");
    }
    const uint32_t port = parseUint32(
        value.substr(pos + 1), std::numeric_limits<uint16_t>::max(), opt);
    return {.host = value.substr(0, pos), .port = static_cast<uint16_t>(port)};
}

std::string toTarget(const BindAddress& bind) {
    return bind.host + ":" + std::to_string(bind.port);
}

int runGen(int argc, char** argv) {
    ultra::tools::ota::ClipGenOptions options;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--frame") {
            options.frame = requireValue(i, argc, argv, arg);
        } else if (arg == "--callsign") {
            options.callsign = requireValue(i, argc, argv, arg);
        } else if (arg == "--peer-callsign" || arg == "--peer") {
            options.peer_callsign = requireValue(i, argc, argv, arg);
        } else if (arg == "--out") {
            options.out_path = requireValue(i, argc, argv, arg);
        } else {
            throw std::runtime_error("unknown gen argument: " + arg);
        }
    }
    return ultra::tools::ota::generateClip(options);
}


ServeOptions parseServeOptions(int argc, char** argv) {
    ServeOptions options;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--bind") {
            options.grpc_bind = parseBindAddress(requireValue(i, argc, argv, arg), arg);
        } else if (arg == "--udp-bind") {
            options.udp_bind = parseBindAddress(requireValue(i, argc, argv, arg), arg);
        } else if (arg == "--tokens") {
            options.tokens_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--captures-root") {
            options.captures_root = requireValue(i, argc, argv, arg);
        } else if (arg == "--lobby-channel") {
            auto channel_type = channel::parseChannelType(requireValue(i, argc, argv, arg));
            if (!channel_type) {
                throw std::runtime_error("unknown lobby channel");
            }
            options.lobby_channel = *channel_type;
        } else if (arg == "--lobby-snr-db") {
            options.lobby_snr_db = std::stof(requireValue(i, argc, argv, arg));
        } else if (arg == "--lobby-seed") {
            options.lobby_seed = parseUint64(requireValue(i, argc, argv, arg), arg);
        } else if (arg == "--lobby-rx-cfo-hz") {
            // Comma-separated RECEIVER:HZ pairs (receiver = station callsign).
            std::string spec = requireValue(i, argc, argv, arg);
            size_t pos = 0;
            while (pos < spec.size()) {
                const size_t comma = spec.find(',', pos);
                const std::string item =
                    spec.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                const size_t colon = item.find(':');
                if (colon == std::string::npos || colon == 0) {
                    throw std::runtime_error("--lobby-rx-cfo-hz expects RECEIVER:HZ[,RECEIVER:HZ]");
                }
                options.lobby_rx_cfo_hz[item.substr(0, colon)] = std::stof(item.substr(colon + 1));
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        } else if (arg == "--lobby-rx-clock-ppm") {
            // Comma-separated RECEIVER:HZ pairs (receiver = station callsign).
            std::string spec = requireValue(i, argc, argv, arg);
            size_t pos = 0;
            while (pos < spec.size()) {
                const size_t comma = spec.find(',', pos);
                const std::string item =
                    spec.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                const size_t colon = item.find(':');
                if (colon == std::string::npos || colon == 0) {
                    throw std::runtime_error("--lobby-rx-clock-ppm expects RECEIVER:PPM[,RECEIVER:PPM]");
                }
                options.lobby_rx_clock_ppm[item.substr(0, colon)] = std::stof(item.substr(colon + 1));
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        } else if (arg == "--lobby-tx-pa") {
            const std::string spec = requireValue(i, argc, argv, arg);
            const auto parsed = channel::parseTxPaSpec(spec);
            if (!parsed) {
                throw std::runtime_error(
                    "--lobby-tx-pa expects on | key=value[,...] with pep, drive_db, p, sat_db, "
                    "attack_ms, release_ms");
            }
            options.lobby_tx_pa = *parsed;
        } else if (arg == "--lobby-station-cap") {
            options.lobby_station_cap = parseUint32(
                requireValue(i, argc, argv, arg), 1024, arg);
        } else if (arg == "--shutdown-deadline-sec") {
            options.shutdown_deadline_seconds = parseUint32(
                requireValue(i, argc, argv, arg), 3600, arg);
        } else if (arg == "--noise-bed-wav") {
            options.noise_bed_wav = requireValue(i, argc, argv, arg);
        } else {
            throw std::runtime_error("unknown serve argument: " + arg);
        }
    }
    if (options.tokens_path.empty()) {
        throw std::runtime_error("serve requires --tokens");
    }
    if (options.lobby_station_cap == 0) {
        throw std::runtime_error("--lobby-station-cap must be non-zero");
    }
    if (options.lobby_channel == channel::ChannelType::REAL_HF_LOOP &&
        options.noise_bed_wav.empty()) {
        throw std::runtime_error("real_hf_loop requires --noise-bed-wav");
    }
    return options;
}

int runServe(int argc, char** argv) {
    const ServeOptions options = parseServeOptions(argc, argv);

    service::AuthAllowlist auth;
    std::string error;
    if (!auth.loadFromFile(options.tokens_path, &error)) {
        throw std::runtime_error(error);
    }
    if (auth.empty()) {
        throw std::runtime_error("token allowlist is empty");
    }

    std::shared_ptr<const std::vector<float>> real_hf_loop_noise;
    if (!options.noise_bed_wav.empty()) {
        auto samples = channel::loadRealHfLoopNoiseBedWav(options.noise_bed_wav.string());
        real_hf_loop_noise =
            std::make_shared<const std::vector<float>>(std::move(samples));
    }

    auto lobby = channel::SessionManager::defaultLobbyConfig();
    lobby.default_channel_model = options.lobby_channel;
    lobby.default_snr_db = options.lobby_snr_db;
    lobby.seed = options.lobby_seed;
    lobby.station_cap = options.lobby_station_cap;
    lobby.rx_cfo_hz = options.lobby_rx_cfo_hz;
    lobby.rx_clock_ppm = options.lobby_rx_clock_ppm;
    lobby.tx_pa = options.lobby_tx_pa;
    lobby.real_hf_loop_noise = real_hf_loop_noise;

    service::OtaSimulatorServiceConfig config;
    config.udp_bind_host = options.udp_bind.host;
    config.udp_bind_port = options.udp_bind.port;
    config.capture_root = options.captures_root;
    config.real_hf_loop_noise = real_hf_loop_noise;
    config.lobby_config = lobby;

    service::OtaSimulatorService control(std::move(auth), std::move(config));
    if (!control.start(&error)) {
        throw std::runtime_error(error);
    }

    grpc::ServerBuilder builder;
    int selected_port = 0;
    builder.AddListeningPort(
        toTarget(options.grpc_bind),
        grpc::InsecureServerCredentials(),
        &selected_port);
    builder.RegisterService(&control);
    std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
    if (!server) {
        control.shutdown();
        throw std::runtime_error("failed to start gRPC server");
    }

    std::cout << "OTASIM_SERVE_READY"
              << " project_version=" << ultra::kProjectUltraVersion
              << " grpc_version=" << grpc::Version()
              << " grpc=" << options.grpc_bind.host << ":" << selected_port
              << " udp=" << control.audioHost() << ":" << control.audioPort()
              << " lobby_session=" << channel::kLobbySessionId
              << " lobby_channel=" << channel::channelTypeName(options.lobby_channel)
              << " lobby_snr_db=" << options.lobby_snr_db
              << " lobby_rx_cfo_hz=" << [&] {
                     std::string out;
                     for (const auto& [rx, hz] : options.lobby_rx_cfo_hz) {
                         if (!out.empty()) out += ',';
                         out += rx + ":" + std::to_string(hz);
                     }
                     return out.empty() ? std::string("none") : out;
                 }()
              << " lobby_rx_clock_ppm=" << [&] {
                     std::string out;
                     for (const auto& [rx, ppm] : options.lobby_rx_clock_ppm) {
                         if (!out.empty()) out += ',';
                         out += rx + ":" + std::to_string(ppm);
                     }
                     return out.empty() ? std::string("none") : out;
                 }()
              << " lobby_tx_pa=" << [&] {
                     if (!options.lobby_tx_pa.enabled) return std::string("none");
                     const auto& t = options.lobby_tx_pa;
                     return "pep:" + std::to_string(t.pep) + ",drive_db:" + std::to_string(t.drive_db) +
                            ",p:" + std::to_string(t.rapp_p) + ",sat_db:" + std::to_string(t.sat_db) +
                            ",attack_ms:" + std::to_string(t.attack_ms) +
                            ",release_ms:" + std::to_string(t.release_ms);
                 }()
              << " lobby_seed=" << options.lobby_seed
              << " lobby_station_cap=" << options.lobby_station_cap
              << " captures_root=" << options.captures_root.string()
              << "\n" << std::flush;

    g_stop_signal = 0;
    std::signal(SIGINT, handleStopSignal);
    std::signal(SIGTERM, handleStopSignal);
    while (g_stop_signal == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const int signal = g_stop_signal;
    std::cerr << "ota_simulator serve: draining after signal " << signal << "\n";
    control.beginDraining();
    const auto deadline = std::chrono::system_clock::now() +
                          std::chrono::seconds(options.shutdown_deadline_seconds);
    server->Shutdown(deadline);
    server->Wait();
    control.shutdown();
    std::cerr << "ota_simulator serve: stopped cleanly\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 2;
    }

    try {
        const std::string command = argv[1];
        if (command == "gen") {
            return runGen(argc, argv);
        }
        if (command == "serve") {
            return runServe(argc, argv);
        }
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "ota_simulator: " << e.what() << "\n";
        return 1;
    }
}
