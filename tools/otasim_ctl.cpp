// otasim_ctl — small admin CLI for the OTASim server.
//
// Talks gRPC to ota_simulator serve to inspect or mutate channel state
// live, without restarting the daemon. Useful for demos, ops debugging,
// and scripted scenarios.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <grpcpp/grpcpp.h>

#include "ota_simulator.grpc.pb.h"

namespace otasim = projectultra::otasim::v1;

namespace {

constexpr auto kRpcDeadline = std::chrono::milliseconds(2000);

struct CommonOpts {
    std::string server = "127.0.0.1:50051";
    std::string token;
    std::string session = "lobby";
};

void addToken(grpc::ClientContext& ctx, const std::string& token) {
    if (!token.empty()) {
        ctx.AddMetadata("authorization", "Bearer " + token);
    }
}

void setDeadline(grpc::ClientContext& ctx) {
    ctx.set_deadline(std::chrono::system_clock::now() + kRpcDeadline);
}

std::unique_ptr<otasim::OtaSimulatorControl::Stub> connect(const std::string& target,
                                                       std::string* error) {
    auto channel = grpc::CreateChannel(target, grpc::InsecureChannelCredentials());
    if (!channel->WaitForConnected(std::chrono::system_clock::now() + kRpcDeadline)) {
        *error = "gRPC channel did not connect to " + target;
        return nullptr;
    }
    return otasim::OtaSimulatorControl::NewStub(channel);
}

int runHealth(const CommonOpts& opts) {
    std::string error;
    auto stub = connect(opts.server, &error);
    if (!stub) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    otasim::HealthRequest req;
    otasim::HealthResponse resp;
    grpc::ClientContext ctx;
    addToken(ctx, opts.token);
    setDeadline(ctx);
    auto status = stub->Health(&ctx, req, &resp);
    if (!status.ok()) {
        std::fprintf(stderr, "Health failed: %s\n", status.error_message().c_str());
        return 1;
    }
    std::printf("ok=%s message=%s\n",
                resp.ok() ? "true" : "false",
                resp.message().c_str());
    return 0;
}

int runListSessions(const CommonOpts& opts) {
    std::string error;
    auto stub = connect(opts.server, &error);
    if (!stub) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    otasim::ListSessionsRequest req;
    otasim::ListSessionsResponse resp;
    grpc::ClientContext ctx;
    addToken(ctx, opts.token);
    setDeadline(ctx);
    auto status = stub->ListSessions(&ctx, req, &resp);
    if (!status.ok()) {
        std::fprintf(stderr, "ListSessions failed: %s\n", status.error_message().c_str());
        return 1;
    }
    if (resp.sessions().empty()) {
        std::printf("(no sessions)\n");
        return 0;
    }
    std::printf("%-24s %-8s %-9s %-12s %s\n",
                "SESSION", "LOBBY", "STATIONS", "CHANNEL", "SNR");
    for (const auto& s : resp.sessions()) {
        std::printf("%-24s %-8s %u/%-7u %-12s %.1f\n",
                    s.session_id().c_str(),
                    s.is_lobby() ? "yes" : "no",
                    s.station_count(),
                    s.station_cap(),
                    s.channel().model().c_str(),
                    s.channel().snr_db());
    }
    return 0;
}

int runGetChannel(const CommonOpts& opts) {
    std::string error;
    auto stub = connect(opts.server, &error);
    if (!stub) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    otasim::GetChannelRequest req;
    req.set_session_id(opts.session);
    otasim::ChannelState resp;
    grpc::ClientContext ctx;
    addToken(ctx, opts.token);
    setDeadline(ctx);
    auto status = stub->GetChannel(&ctx, req, &resp);
    if (!status.ok()) {
        std::fprintf(stderr, "GetChannel failed: %s\n", status.error_message().c_str());
        return 1;
    }
    std::printf("session=%s model=%s snr_db=%.2f seed=%llu applied_at_sample=%llu effects=%d\n",
                resp.session_id().c_str(),
                resp.model().c_str(),
                resp.snr_db(),
                static_cast<unsigned long long>(resp.channel_seed()),
                static_cast<unsigned long long>(resp.applied_at_sample()),
                resp.active_effects_size());
    for (const auto& e : resp.active_effects()) {
        std::printf("  effect id=%s type=%s start=%llu duration=%llu\n",
                    e.effect_id().c_str(),
                    e.effect_type().c_str(),
                    static_cast<unsigned long long>(e.start_sample()),
                    static_cast<unsigned long long>(e.duration_samples()));
    }
    return 0;
}

int runSetChannel(const CommonOpts& opts,
                  const std::string& model,
                  double snr_db,
                  uint64_t seed,
                  bool seed_provided) {
    std::string error;
    auto stub = connect(opts.server, &error);
    if (!stub) {
        std::fprintf(stderr, "%s\n", error.c_str());
        return 2;
    }
    otasim::SetChannelRequest req;
    req.set_session_id(opts.session);
    req.set_model(model);
    req.set_snr_db(snr_db);
    if (seed_provided) {
        req.set_seed(seed);
    }
    otasim::CommandAck ack;
    grpc::ClientContext ctx;
    addToken(ctx, opts.token);
    setDeadline(ctx);
    auto status = stub->SetChannel(&ctx, req, &ack);
    if (!status.ok()) {
        std::fprintf(stderr, "SetChannel failed: %s\n", status.error_message().c_str());
        return 1;
    }
    if (!ack.accepted()) {
        std::fprintf(stderr, "SetChannel rejected: %s\n", ack.message().c_str());
        return 1;
    }
    std::printf("ok session=%s model=%s snr_db=%.2f%s\n",
                opts.session.c_str(), model.c_str(), snr_db,
                seed_provided ? (" seed=" + std::to_string(seed)).c_str() : "");
    return 0;
}

void printUsage(const char* prog) {
    std::printf("Usage:\n"
                "  %s [--server HOST:PORT] [--token TOKEN] [--session ID] <subcommand>\n"
                "\n"
                "Subcommands:\n"
                "  health                              Server health, versions, counts\n"
                "  list-sessions                       List active sessions\n"
                "  get-channel                         Show current channel for --session\n"
                "  set-channel --model M --snr DB [--seed N]\n"
                "                                      Update channel for --session\n"
                "\n"
                "Channel models:\n"
                "  passthrough, awgn,\n"
                "  watterson_good, watterson_moderate, watterson_poor, watterson_flutter\n"
                "\n"
                "Examples:\n"
                "  %s --token alpha_tok health\n"
                "  %s --token alpha_tok set-channel --model awgn --snr 20\n"
                "  %s --token alpha_tok --session lobby set-channel \\\n"
                "      --model watterson_moderate --snr 12\n"
                "\n"
                "Token may also be set via OTASIM_TOKEN environment variable.\n",
                prog, prog, prog, prog);
}

bool readArgValue(int argc, char** argv, int& i, const char* flag, std::string& out) {
    if (i + 1 >= argc) {
        std::fprintf(stderr, "Missing value for %s\n", flag);
        return false;
    }
    out = argv[++i];
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    CommonOpts opts;
    if (const char* env_token = std::getenv("OTASIM_TOKEN")) {
        opts.token = env_token;
    }

    std::string subcommand;
    std::string model;
    double snr_db = 15.0;
    uint64_t seed = 0;
    bool seed_provided = false;
    bool snr_provided = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--server") {
            if (!readArgValue(argc, argv, i, "--server", opts.server)) return 2;
        } else if (arg == "--token") {
            if (!readArgValue(argc, argv, i, "--token", opts.token)) return 2;
        } else if (arg == "--session") {
            if (!readArgValue(argc, argv, i, "--session", opts.session)) return 2;
        } else if (arg == "--model") {
            if (!readArgValue(argc, argv, i, "--model", model)) return 2;
        } else if (arg == "--snr") {
            std::string v;
            if (!readArgValue(argc, argv, i, "--snr", v)) return 2;
            snr_db = std::atof(v.c_str());
            snr_provided = true;
        } else if (arg == "--seed") {
            std::string v;
            if (!readArgValue(argc, argv, i, "--seed", v)) return 2;
            seed = std::strtoull(v.c_str(), nullptr, 10);
            seed_provided = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else if (subcommand.empty()) {
            subcommand = arg;
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            printUsage(argv[0]);
            return 2;
        }
    }

    if (subcommand.empty()) {
        printUsage(argv[0]);
        return 2;
    }

    if (subcommand == "health") {
        return runHealth(opts);
    } else if (subcommand == "list-sessions") {
        return runListSessions(opts);
    } else if (subcommand == "get-channel") {
        return runGetChannel(opts);
    } else if (subcommand == "set-channel") {
        if (model.empty()) {
            std::fprintf(stderr, "set-channel requires --model\n");
            return 2;
        }
        if (!snr_provided) {
            std::fprintf(stderr, "set-channel requires --snr\n");
            return 2;
        }
        return runSetChannel(opts, model, snr_db, seed, seed_provided);
    }

    std::fprintf(stderr, "Unknown subcommand: %s\n", subcommand.c_str());
    printUsage(argv[0]);
    return 2;
}
