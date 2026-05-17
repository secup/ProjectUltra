#if defined(_WIN32)

#include <iostream>

int main() {
    std::cout << "ota_simulator serve smoke skipped on Windows\n";
    return 0;
}

#else

#include "helpers/temp_dir.hpp"
#include "ota_channel_core/session_manager.hpp"
#include "ota_simulator.grpc.pb.h"
#include "ota_simulator_service/audio_packet.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <grpcpp/grpcpp.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

namespace otasim = projectultra::otasim::v1;
namespace service = ultra::ota_simulator_service;

struct UniqueFd {
    int fd = -1;

    UniqueFd() = default;
    explicit UniqueFd(int value) : fd(value) {}
    ~UniqueFd() { reset(); }

    UniqueFd(const UniqueFd&) = delete;
    UniqueFd& operator=(const UniqueFd&) = delete;

    UniqueFd(UniqueFd&& other) noexcept : fd(other.fd) { other.fd = -1; }
    UniqueFd& operator=(UniqueFd&& other) noexcept {
        if (this != &other) {
            reset();
            fd = other.fd;
            other.fd = -1;
        }
        return *this;
    }

    void reset(int value = -1) {
        if (fd >= 0) {
            ::close(fd);
        }
        fd = value;
    }
};

struct ChildProcess {
    pid_t pid = -1;
    UniqueFd output;
    bool reaped = false;
    int status = 0;

    void markReaped(int child_status) {
        status = child_status;
        reaped = true;
        pid = -1;
    }

    void killNow() {
        if (pid > 0 && !reaped) {
            (void)::kill(pid, SIGKILL);
            int child_status = 0;
            (void)::waitpid(pid, &child_status, 0);
            markReaped(child_status);
        }
    }

    void terminateCleanly() {
        if (pid <= 0 || reaped) {
            throw std::runtime_error("daemon exited before SIGTERM");
        }
        if (::kill(pid, SIGTERM) != 0) {
            throw std::runtime_error("failed to send SIGTERM");
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (std::chrono::steady_clock::now() < deadline) {
            int child_status = 0;
            const pid_t result = ::waitpid(pid, &child_status, WNOHANG);
            if (result == pid) {
                markReaped(child_status);
                if (WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0) {
                    return;
                }
                throw std::runtime_error("daemon did not exit 0 after SIGTERM");
            }
            if (result < 0 && errno != EINTR) {
                throw std::runtime_error("waitpid failed during SIGTERM shutdown");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        killNow();
        throw std::runtime_error("daemon did not exit before SIGTERM timeout");
    }
};

struct ReadyBanner {
    std::string grpc_target;
    std::string udp_host;
    uint16_t udp_port = 0;
    std::string output;
};

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void addToken(grpc::ClientContext& context, const std::string& token) {
    context.AddMetadata("authorization", "Bearer " + token);
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream in(path);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::string valueFor(const std::string& line, const std::string& key) {
    const std::string needle = key + "=";
    const auto begin = line.find(needle);
    if (begin == std::string::npos) {
        return {};
    }
    const auto value_begin = begin + needle.size();
    const auto end = line.find(' ', value_begin);
    return line.substr(value_begin, end == std::string::npos ? end : end - value_begin);
}

std::pair<std::string, uint16_t> splitHostPort(const std::string& value) {
    const auto pos = value.rfind(':');
    check(pos != std::string::npos && pos + 1 < value.size(), "missing host:port");
    const auto port = std::stoul(value.substr(pos + 1));
    check(port <= 65535, "port out of range");
    return {value.substr(0, pos), static_cast<uint16_t>(port)};
}

ChildProcess startDaemon(const std::string& binary,
                         const std::filesystem::path& token_path,
                         const std::filesystem::path& capture_root) {
    int pipe_fds[2] = {-1, -1};
    check(::pipe(pipe_fds) == 0, "pipe failed");

    const pid_t pid = ::fork();
    check(pid >= 0, "fork failed");
    if (pid == 0) {
        ::dup2(pipe_fds[1], STDOUT_FILENO);
        ::dup2(pipe_fds[1], STDERR_FILENO);
        ::close(pipe_fds[0]);
        ::close(pipe_fds[1]);
        ::execl(binary.c_str(),
                binary.c_str(),
                "serve",
                "--bind", "127.0.0.1:0",
                "--udp-bind", "127.0.0.1:0",
                "--tokens", token_path.c_str(),
                "--captures-root", capture_root.c_str(),
                "--lobby-channel", "passthrough",
                "--shutdown-deadline-sec", "2",
                nullptr);
        ::_exit(127);
    }

    ::close(pipe_fds[1]);
    const int flags = ::fcntl(pipe_fds[0], F_GETFL, 0);
    check(flags >= 0, "fcntl get failed");
    check(::fcntl(pipe_fds[0], F_SETFL, flags | O_NONBLOCK) == 0, "fcntl set failed");

    ChildProcess child;
    child.pid = pid;
    child.output = UniqueFd(pipe_fds[0]);
    return child;
}

ReadyBanner waitForReady(ChildProcess& child) {
    std::string output;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        std::array<char, 1024> buffer{};
        const ssize_t n = ::read(child.output.fd, buffer.data(), buffer.size());
        if (n > 0) {
            output.append(buffer.data(), static_cast<size_t>(n));
            const auto ready_pos = output.find("OTASIM_SERVE_READY");
            if (ready_pos != std::string::npos) {
                const auto line_end = output.find('\n', ready_pos);
                if (line_end != std::string::npos) {
                    const std::string line = output.substr(ready_pos, line_end - ready_pos);
                    const auto grpc_target = valueFor(line, "grpc");
                    const auto udp_value = valueFor(line, "udp");
                    const auto [udp_host, udp_port] = splitHostPort(udp_value);
                    check(!grpc_target.empty(), "ready banner missing grpc target");
                    check(!udp_host.empty() && udp_port != 0, "ready banner missing UDP target");
                    return {.grpc_target = grpc_target,
                            .udp_host = udp_host,
                            .udp_port = udp_port,
                            .output = output};
                }
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            throw std::runtime_error("failed to read daemon output");
        }

        int child_status = 0;
        const pid_t result = ::waitpid(child.pid, &child_status, WNOHANG);
        if (result == child.pid) {
            child.markReaped(child_status);
            throw std::runtime_error("daemon exited before ready: " + output);
        }
        if (result < 0 && errno != EINTR) {
            throw std::runtime_error("waitpid failed while waiting for ready");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("timed out waiting for daemon ready: " + output);
}

UniqueFd makeUdpSocket() {
    UniqueFd fd(::socket(AF_INET, SOCK_DGRAM, 0));
    check(fd.fd >= 0, "UDP socket failed");
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(0);
    local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    check(::bind(fd.fd, reinterpret_cast<sockaddr*>(&local), sizeof(local)) == 0,
          "UDP bind failed");
    return fd;
}

sockaddr_in udpAddress(const std::string& host, uint16_t port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    check(::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) == 1, "bad UDP host");
    return addr;
}

void sendPacket(int fd,
                const sockaddr_in& server,
                uint64_t lease_id,
                uint64_t seq,
                uint64_t start_sample,
                const std::vector<float>& samples) {
    service::OtaAudioPacket packet;
    packet.header.lease_id = lease_id;
    packet.header.seq = seq;
    packet.header.start_sample = start_sample;
    packet.samples = samples;
    const auto bytes = service::serializeAudioPacket(packet);
    const ssize_t sent = ::sendto(fd,
                                  bytes.data(),
                                  bytes.size(),
                                  0,
                                  reinterpret_cast<const sockaddr*>(&server),
                                  sizeof(server));
    check(sent == static_cast<ssize_t>(bytes.size()), "UDP send failed");
}

bool sameSamples(const std::vector<float>& actual, const std::vector<float>& expected) {
    return actual.size() == expected.size() &&
           std::equal(actual.begin(), actual.end(), expected.begin());
}

std::vector<float> makeSamples(size_t count, float sign) {
    std::vector<float> out(count);
    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = sign * static_cast<float>((i % 31) + 1) / 32.0f;
    }
    return out;
}

uint64_t waitForSamples(int fd,
                        uint64_t lease_id,
                        const std::vector<float>& expected) {
    std::array<uint8_t, 8192> buffer{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);
        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        const int selected = ::select(fd + 1, &read_fds, nullptr, nullptr, &timeout);
        if (selected < 0 && errno == EINTR) {
            continue;
        }
        check(selected >= 0, "UDP select failed");
        if (selected == 0) {
            continue;
        }
        const ssize_t n = ::recv(fd, buffer.data(), buffer.size(), 0);
        if (n <= 0) {
            continue;
        }
        auto packet = service::parseAudioPacket(
            std::span<const uint8_t>(buffer.data(), static_cast<size_t>(n)));
        if (!packet) {
            continue;
        }
        if (packet->header.lease_id == lease_id &&
            sameSamples(packet->samples, expected)) {
            return packet->header.start_sample;
        }
    }
    throw std::runtime_error("timed out waiting for exact passthrough audio");
}

void verifyCaptureArtifacts(const std::filesystem::path& capture_root) {
    const auto capture_dir = capture_root / ultra::ota_channel_core::kLobbySessionId;
    check(std::filesystem::is_directory(capture_dir), "capture directory missing");

    const auto manifest_path = capture_dir / "manifest.json";
    const auto events_path = capture_dir / "events.jsonl";
    check(std::filesystem::is_regular_file(manifest_path), "manifest missing");
    check(std::filesystem::is_regular_file(events_path), "events jsonl missing");
    check(std::filesystem::file_size(events_path) > 0, "events jsonl empty");

    const std::string manifest = readFile(manifest_path);
    check(manifest.find("\"session_id\":\"lobby\"") != std::string::npos,
          "manifest missing lobby session");
    check(manifest.find("\"model\":\"passthrough\"") != std::string::npos,
          "manifest missing passthrough channel");
    check(!manifest.empty() && manifest.front() == '{' && manifest.find("\n}") != std::string::npos,
          "manifest is not a JSON object");

    size_t wav_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(capture_dir)) {
        if (entry.path().extension() == ".wav") {
            check(std::filesystem::file_size(entry.path()) > 44, "WAV capture is empty");
            ++wav_count;
        }
    }
    check(wav_count >= 4, "expected TX and RX WAV captures for both stations");
}

}  // namespace

int main(int argc, char** argv) {
    ChildProcess child;
    try {
        check(argc == 2, "usage: test_otasim_serve_smoke <ota_simulator>");
        ultra::test::TempDir temp("otasim_serve_smoke");
        check(temp.valid(), "failed to create temp dir");

        const auto token_path = temp.child("tokens.conf");
        {
            std::ofstream out(token_path);
            out << "alice_token:ALPHA:Alpha station\n";
            out << "bob_token:BRAVO:Bravo station\n";
        }
        const auto capture_root = temp.child("captures");

        child = startDaemon(argv[1], token_path, capture_root);
        const ReadyBanner ready = waitForReady(child);

        auto channel = grpc::CreateChannel(ready.grpc_target, grpc::InsecureChannelCredentials());
        check(channel->WaitForConnected(std::chrono::system_clock::now() +
                                        std::chrono::seconds(5)),
              "gRPC channel did not connect");
        auto stub = otasim::OtaSimulatorControl::NewStub(channel);

        otasim::RegisterStationRequest register_request;
        otasim::StationLease alice_lease;
        grpc::ClientContext alice_register_context;
        addToken(alice_register_context, "alice_token");
        auto status = stub->RegisterStation(
            &alice_register_context, register_request, &alice_lease);
        check(status.ok(), "alice register failed: " + status.error_message());

        otasim::StationLease bob_lease;
        grpc::ClientContext bob_register_context;
        addToken(bob_register_context, "bob_token");
        status = stub->RegisterStation(&bob_register_context, register_request, &bob_lease);
        check(status.ok(), "bob register failed: " + status.error_message());

        otasim::JoinSessionRequest alice_join;
        alice_join.set_session_id(ultra::ota_channel_core::kLobbySessionId);
        alice_join.set_station_id(alice_lease.station_id());
        otasim::JoinSessionResponse alice_join_response;
        grpc::ClientContext alice_join_context;
        addToken(alice_join_context, "alice_token");
        status = stub->JoinSession(&alice_join_context, alice_join, &alice_join_response);
        check(status.ok(), "alice join failed: " + status.error_message());

        otasim::JoinSessionRequest bob_join;
        bob_join.set_session_id(ultra::ota_channel_core::kLobbySessionId);
        bob_join.set_station_id(bob_lease.station_id());
        otasim::JoinSessionResponse bob_join_response;
        grpc::ClientContext bob_join_context;
        addToken(bob_join_context, "bob_token");
        status = stub->JoinSession(&bob_join_context, bob_join, &bob_join_response);
        check(status.ok(), "bob join failed: " + status.error_message());

        otasim::NegotiateAudioRequest alice_audio_request;
        alice_audio_request.set_session_id(ultra::ota_channel_core::kLobbySessionId);
        alice_audio_request.set_station_id(alice_lease.station_id());
        otasim::AudioLease alice_audio;
        grpc::ClientContext alice_audio_context;
        addToken(alice_audio_context, "alice_token");
        status = stub->NegotiateAudio(&alice_audio_context, alice_audio_request, &alice_audio);
        check(status.ok(), "alice audio negotiate failed: " + status.error_message());

        otasim::NegotiateAudioRequest bob_audio_request;
        bob_audio_request.set_session_id(ultra::ota_channel_core::kLobbySessionId);
        bob_audio_request.set_station_id(bob_lease.station_id());
        otasim::AudioLease bob_audio;
        grpc::ClientContext bob_audio_context;
        addToken(bob_audio_context, "bob_token");
        status = stub->NegotiateAudio(&bob_audio_context, bob_audio_request, &bob_audio);
        check(status.ok(), "bob audio negotiate failed: " + status.error_message());

        UniqueFd alice_udp = makeUdpSocket();
        UniqueFd bob_udp = makeUdpSocket();
        const sockaddr_in udp_server = udpAddress(ready.udp_host, ready.udp_port);

        const std::vector<float> warmup(480, 0.0f);
        sendPacket(alice_udp.fd, udp_server, alice_audio.lease_id(), 0, 0, warmup);
        sendPacket(bob_udp.fd, udp_server, bob_audio.lease_id(), 0, 0, warmup);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        otasim::StartCaptureRequest start_capture;
        start_capture.set_session_id(ultra::ota_channel_core::kLobbySessionId);
        start_capture.set_start_sample(0);
        otasim::CaptureInfo started;
        grpc::ClientContext start_capture_context;
        addToken(start_capture_context, "alice_token");
        status = stub->StartCapture(&start_capture_context, start_capture, &started);
        check(status.ok(), "start capture failed: " + status.error_message());
        check(started.active(), "capture did not become active");

        const std::vector<float> alice_samples = makeSamples(480, 1.0f);
        const std::vector<float> bob_samples = makeSamples(480, -1.0f);
        sendPacket(alice_udp.fd, udp_server, alice_audio.lease_id(), 1, 480, alice_samples);
        sendPacket(bob_udp.fd, udp_server, bob_audio.lease_id(), 1, 480, bob_samples);
        const uint64_t bob_start =
            waitForSamples(bob_udp.fd, bob_audio.lease_id(), alice_samples);
        const uint64_t alice_start =
            waitForSamples(alice_udp.fd, alice_audio.lease_id(), bob_samples);
        check(bob_start % 480 == 0, "bob RX start sample is not session tick aligned");
        check(alice_start % 480 == 0, "alice RX start sample is not session tick aligned");

        otasim::StopCaptureRequest stop_capture;
        stop_capture.set_session_id(ultra::ota_channel_core::kLobbySessionId);
        otasim::CaptureInfo stopped;
        grpc::ClientContext stop_capture_context;
        addToken(stop_capture_context, "alice_token");
        status = stub->StopCapture(&stop_capture_context, stop_capture, &stopped);
        check(status.ok(), "stop capture failed: " + status.error_message());
        check(!stopped.active(), "capture stayed active after stop");

        verifyCaptureArtifacts(capture_root);
        child.terminateCleanly();

        std::cout << "ota_simulator serve smoke covered gRPC, UDP, captures, SIGTERM\n";
        return 0;
    } catch (const std::exception& e) {
        child.killNow();
        std::cerr << "test_otasim_serve_smoke: " << e.what() << "\n";
        return 1;
    }
}

#endif
