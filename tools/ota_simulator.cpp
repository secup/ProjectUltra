#include "ota_simulator/clip_gen.hpp"
#include "ota_simulator/runner.hpp"
#include "ota_simulator/scenario.hpp"
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/session_manager.hpp"
#include "ota_simulator_service/auth_allowlist.hpp"
#include "ota_simulator_service/ota_simulator_service.hpp"
#include "ultra/version.hpp"

#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

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
    uint32_t shutdown_deadline_seconds = 60;
};

void usage() {
    std::cerr
        << "Usage:\n"
        << "  ota_simulator gen --frame FRAME --callsign CALL [--peer-callsign CALL] --out FILE.wav\n"
        << "  ota_simulator run --scenario FILE.json [--save-rx-audio]\n"
        << "  ota_simulator serve --bind HOST:PORT --tokens FILE [--udp-bind HOST:PORT]\n";
}

void handleStopSignal(int signal) {
    g_stop_signal = signal;
}

std::string lower(std::string value) {
    for (char& c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

std::optional<channel::ChannelType> parseChannelType(std::string value) {
    value = lower(std::move(value));
    if (value.empty() || value == "passthrough" || value == "null") {
        return channel::ChannelType::PASSTHROUGH;
    }
    if (value == "awgn") {
        return channel::ChannelType::AWGN;
    }
    if (value == "good" || value == "watterson_good") {
        return channel::ChannelType::GOOD;
    }
    if (value == "moderate" || value == "watterson_moderate") {
        return channel::ChannelType::MODERATE;
    }
    if (value == "poor" || value == "watterson_poor") {
        return channel::ChannelType::POOR;
    }
    if (value == "flutter" || value == "watterson_flutter") {
        return channel::ChannelType::FLUTTER;
    }
    return std::nullopt;
}

std::string deriveRxPath(const std::string& tx_path) {
    const std::string needle = "_tx";
    const auto pos = tx_path.rfind(needle);
    if (pos != std::string::npos) {
        return tx_path.substr(0, pos) + "_rx" + tx_path.substr(pos + needle.size());
    }
    const auto dot = tx_path.rfind('.');
    if (dot != std::string::npos) {
        return tx_path.substr(0, dot) + "_rx" + tx_path.substr(dot);
    }
    return tx_path + "_rx.wav";
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

int runScenario(int argc, char** argv) {
    std::string scenario_path;
    bool save_rx_audio = false;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario") {
            scenario_path = requireValue(i, argc, argv, arg);
        } else if (arg == "--save-rx-audio") {
            save_rx_audio = true;
        } else {
            throw std::runtime_error("unknown run argument: " + arg);
        }
    }
    if (scenario_path.empty()) {
        throw std::runtime_error("run requires --scenario");
    }
    auto scenario = ultra::tools::ota::loadScenario(scenario_path);
    if (save_rx_audio) {
        if (scenario.output.alice_rx_capture.empty()) {
            scenario.output.alice_rx_capture =
                deriveRxPath(scenario.output.alice_tx_capture);
        }
        if (scenario.output.bob_rx_capture.empty()) {
            scenario.output.bob_rx_capture =
                deriveRxPath(scenario.output.bob_tx_capture);
        }
    }
    return ultra::tools::ota::runScenario(scenario);
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
            auto channel_type = parseChannelType(requireValue(i, argc, argv, arg));
            if (!channel_type) {
                throw std::runtime_error("unknown lobby channel");
            }
            options.lobby_channel = *channel_type;
        } else if (arg == "--lobby-snr-db") {
            options.lobby_snr_db = std::stof(requireValue(i, argc, argv, arg));
        } else if (arg == "--lobby-seed") {
            options.lobby_seed = parseUint64(requireValue(i, argc, argv, arg), arg);
        } else if (arg == "--lobby-station-cap") {
            options.lobby_station_cap = parseUint32(
                requireValue(i, argc, argv, arg), 1024, arg);
        } else if (arg == "--shutdown-deadline-sec") {
            options.shutdown_deadline_seconds = parseUint32(
                requireValue(i, argc, argv, arg), 3600, arg);
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

    auto lobby = channel::SessionManager::defaultLobbyConfig();
    lobby.default_channel_model = options.lobby_channel;
    lobby.default_snr_db = options.lobby_snr_db;
    lobby.seed = options.lobby_seed;
    lobby.station_cap = options.lobby_station_cap;

    service::OtaSimulatorServiceConfig config;
    config.udp_bind_host = options.udp_bind.host;
    config.udp_bind_port = options.udp_bind.port;
    config.capture_root = options.captures_root;
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
        if (command == "run") {
            return runScenario(argc, argv);
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
