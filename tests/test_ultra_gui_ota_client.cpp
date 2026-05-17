#if defined(_WIN32)

#include <iostream>

int main() {
    std::cout << "ultra_gui OTASim client test skipped on Windows\n";
    return 0;
}

#else

#include "gui/ota_audio_backend.hpp"
#include "helpers/temp_dir.hpp"

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
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {

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
    std::string output;
};

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
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
                    check(!grpc_target.empty(), "ready banner missing grpc target");
                    return {.grpc_target = grpc_target, .output = output};
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

bool containsSamples(const std::vector<float>& haystack, const std::vector<float>& needle) {
    if (needle.empty() || haystack.size() < needle.size()) {
        return false;
    }
    return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end()) != haystack.end();
}

void waitForSamples(ultra::gui::OtaAudioBackend& backend,
                    const std::vector<float>& expected) {
    std::vector<float> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
        auto chunk = backend.getRxSamples(256);
        received.insert(received.end(), chunk.begin(), chunk.end());
        if (containsSamples(received, expected)) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("timed out waiting for passthrough audio");
}

}  // namespace

int main(int argc, char** argv) {
    ChildProcess child;
    try {
        check(argc == 2, "usage: test_ultra_gui_ota_client <ota_simulator>");
        ultra::test::TempDir temp("ultra_gui_ota_client");
        check(temp.valid(), "failed to create temp dir");

        const auto token_path = temp.child("tokens.conf");
        {
            std::ofstream out(token_path);
            out << "alice_token:ALICE:Alice station\n";
            out << "bob_token:BOB:Bob station\n";
        }

        child = startDaemon(argv[1], token_path, temp.child("captures"));
        const ReadyBanner ready = waitForReady(child);

        ultra::gui::OtaAudioBackend alice;
        ultra::gui::OtaAudioBackend bob;
        std::string error;
        check(alice.start({.grpc_target = ready.grpc_target,
                           .token = "alice_token",
                           .station_id = "alice",
                           .session_id = "lobby"},
                          &error),
              "alice backend start failed: " + error);
        check(bob.start({.grpc_target = ready.grpc_target,
                         .token = "bob_token",
                         .station_id = "bob",
                         .session_id = "lobby"},
                        &error),
              "bob backend start failed: " + error);
        check(alice.waitForConnected(std::chrono::seconds(5)),
              "alice backend did not connect: " + alice.status().text);
        check(bob.waitForConnected(std::chrono::seconds(5)),
              "bob backend did not connect: " + bob.status().text);

        const std::vector<float> tx{0.125f, -0.25f, 0.5f, -0.75f,
                                    0.875f, -1.0f, 0.375f, -0.625f};
        check(alice.queueTxSamples(tx, &error), "alice TX failed: " + error);
        waitForSamples(bob, tx);

        alice.close();
        bob.close();
        check(alice.status().state == ultra::gui::OtaAudioConnectionState::Disconnected,
              "alice did not close cleanly");
        check(bob.status().state == ultra::gui::OtaAudioConnectionState::Disconnected,
              "bob did not close cleanly");

        child.terminateCleanly();
        std::cout << "ultra_gui OTASim audio backend passed passthrough and close checks\n";
        return 0;
    } catch (const std::exception& e) {
        child.killNow();
        std::cerr << "test_ultra_gui_ota_client: " << e.what() << "\n";
        return 1;
    }
}

#endif
