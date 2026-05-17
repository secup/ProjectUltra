#if defined(_WIN32)

#include <iostream>

int main() {
    std::cout << "ultra_tnc OTASim client test skipped on Windows\n";
    return 0;
}

#else

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
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
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
    std::string name;
    std::string captured;
    bool reaped = false;
    int status = 0;

    void markReaped(int child_status) {
        status = child_status;
        reaped = true;
        pid = -1;
    }

    void readAvailable() {
        if (output.fd < 0) {
            return;
        }
        std::array<char, 4096> buffer{};
        while (true) {
            const ssize_t n = ::read(output.fd, buffer.data(), buffer.size());
            if (n > 0) {
                captured.append(buffer.data(), static_cast<size_t>(n));
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                return;
            }
            return;
        }
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
            throw std::runtime_error(name + " exited before SIGTERM");
        }
        if (::kill(pid, SIGTERM) != 0) {
            throw std::runtime_error("failed to send SIGTERM to " + name);
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (std::chrono::steady_clock::now() < deadline) {
            readAvailable();
            int child_status = 0;
            const pid_t result = ::waitpid(pid, &child_status, WNOHANG);
            if (result == pid) {
                markReaped(child_status);
                if (WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0) {
                    return;
                }
                throw std::runtime_error(name + " did not exit 0 after SIGTERM:\n" + captured);
            }
            if (result < 0 && errno != EINTR) {
                throw std::runtime_error("waitpid failed during " + name + " shutdown");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        killNow();
        throw std::runtime_error(name + " did not exit before SIGTERM timeout:\n" + captured);
    }
};

struct ReadyBanner {
    std::string grpc_target;
};

void check(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void makeNonblocking(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    check(flags >= 0, "fcntl get failed");
    check(::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0, "fcntl set failed");
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
    makeNonblocking(pipe_fds[0]);

    ChildProcess child;
    child.pid = pid;
    child.output = UniqueFd(pipe_fds[0]);
    child.name = "ota_simulator";
    return child;
}

ChildProcess startTnc(const std::string& binary,
                      const std::string& grpc_target,
                      const std::string& token,
                      const std::string& station_id,
                      const std::string& callsign,
                      uint16_t port) {
    int pipe_fds[2] = {-1, -1};
    check(::pipe(pipe_fds) == 0, "pipe failed");

    const std::string port_text = std::to_string(port);
    const pid_t pid = ::fork();
    check(pid >= 0, "fork failed");
    if (pid == 0) {
        ::dup2(pipe_fds[1], STDOUT_FILENO);
        ::dup2(pipe_fds[1], STDERR_FILENO);
        ::close(pipe_fds[0]);
        ::close(pipe_fds[1]);
        ::execl(binary.c_str(),
                binary.c_str(),
                "--sim-audio",
                "--ota-host", grpc_target.c_str(),
                "--token", token.c_str(),
                "--station-id", station_id.c_str(),
                "--session-id", "lobby",
                "--callsign", callsign.c_str(),
                "--bind", "127.0.0.1",
                "--port", port_text.c_str(),
                "--log-level", "info",
                "--log-category", "operator,audio,modem",
                nullptr);
        ::_exit(127);
    }

    ::close(pipe_fds[1]);
    makeNonblocking(pipe_fds[0]);

    ChildProcess child;
    child.pid = pid;
    child.output = UniqueFd(pipe_fds[0]);
    child.name = "ultra_tnc " + callsign;
    return child;
}

ReadyBanner waitForReady(ChildProcess& child) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
        child.readAvailable();
        const auto ready_pos = child.captured.find("OTASIM_SERVE_READY");
        if (ready_pos != std::string::npos) {
            const auto line_end = child.captured.find('\n', ready_pos);
            if (line_end != std::string::npos) {
                const std::string line = child.captured.substr(ready_pos, line_end - ready_pos);
                const auto grpc_target = valueFor(line, "grpc");
                check(!grpc_target.empty(), "ready banner missing grpc target");
                return {.grpc_target = grpc_target};
            }
        }

        int child_status = 0;
        const pid_t result = ::waitpid(child.pid, &child_status, WNOHANG);
        if (result == child.pid) {
            child.markReaped(child_status);
            throw std::runtime_error("daemon exited before ready:\n" + child.captured);
        }
        if (result < 0 && errno != EINTR) {
            throw std::runtime_error("waitpid failed while waiting for daemon ready");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("timed out waiting for daemon ready:\n" + child.captured);
}

void pumpChildren(std::vector<ChildProcess*>& children) {
    for (ChildProcess* child : children) {
        if (child) {
            child->readAvailable();
        }
    }
}

void waitForOutput(ChildProcess& child,
                   const std::string& needle,
                   std::vector<ChildProcess*>& children,
                   std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        pumpChildren(children);
        if (child.captured.find(needle) != std::string::npos) {
            return;
        }

        int child_status = 0;
        const pid_t result = ::waitpid(child.pid, &child_status, WNOHANG);
        if (result == child.pid) {
            child.markReaped(child_status);
            throw std::runtime_error(child.name + " exited while waiting for '" +
                                     needle + "':\n" + child.captured);
        }
        if (result < 0 && errno != EINTR) {
            throw std::runtime_error("waitpid failed for " + child.name);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("timed out waiting for " + child.name + " output '" +
                             needle + "':\n" + child.captured);
}

uint16_t reservePortPair() {
    for (int attempt = 0; attempt < 200; ++attempt) {
        UniqueFd first(::socket(AF_INET, SOCK_STREAM, 0));
        UniqueFd second(::socket(AF_INET, SOCK_STREAM, 0));
        check(first.fd >= 0 && second.fd >= 0, "socket failed while reserving ports");

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(0);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(first.fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            continue;
        }

        sockaddr_in bound{};
        socklen_t bound_len = sizeof(bound);
        if (::getsockname(first.fd, reinterpret_cast<sockaddr*>(&bound), &bound_len) != 0) {
            continue;
        }
        const uint16_t port = ntohs(bound.sin_port);
        if (port == 0 || port >= 65535) {
            continue;
        }

        sockaddr_in next{};
        next.sin_family = AF_INET;
        next.sin_port = htons(static_cast<uint16_t>(port + 1));
        next.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (::bind(second.fd, reinterpret_cast<sockaddr*>(&next), sizeof(next)) == 0) {
            return port;
        }
    }
    throw std::runtime_error("failed to reserve adjacent localhost TCP ports");
}

class TcpClient {
public:
    TcpClient() = default;
    ~TcpClient() { close(); }

    TcpClient(const TcpClient&) = delete;
    TcpClient& operator=(const TcpClient&) = delete;

    void connectTo(uint16_t port,
                   std::vector<ChildProcess*>& children,
                   std::chrono::seconds timeout = std::chrono::seconds(10)) {
        close();
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        check(fd_ >= 0, "TCP socket failed");
        makeNonblocking(fd_);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const int rc = ::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
            if (rc == 0 || errno == EISCONN) {
                return;
            }
            if (errno == EINPROGRESS || errno == EALREADY || errno == ECONNREFUSED) {
                pollfd pfd{};
                pfd.fd = fd_;
                pfd.events = POLLOUT;
                (void)::poll(&pfd, 1, 50);
                int error = 0;
                socklen_t len = sizeof(error);
                if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &len) == 0) {
                    if (error == 0) {
                        return;
                    }
                    if (error != EINPROGRESS && error != EALREADY && error != ECONNREFUSED) {
                        close();
                        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
                        check(fd_ >= 0, "TCP socket retry failed");
                        makeNonblocking(fd_);
                    }
                }
                pumpChildren(children);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            throw std::runtime_error("connect failed: " + std::string(std::strerror(errno)));
        }
        throw std::runtime_error("timed out connecting to TNC port " + std::to_string(port));
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        rx_.clear();
    }

    void writeBytes(const std::vector<uint8_t>& bytes) {
        size_t offset = 0;
        while (offset < bytes.size()) {
            const ssize_t n = ::send(fd_,
                                     reinterpret_cast<const char*>(bytes.data() + offset),
                                     bytes.size() - offset,
                                     0);
            if (n > 0) {
                offset += static_cast<size_t>(n);
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            throw std::runtime_error("socket write failed");
        }
    }

    void writeString(const std::string& text) {
        writeBytes(std::vector<uint8_t>(text.begin(), text.end()));
    }

    std::string readUntil(char delimiter,
                          std::vector<ChildProcess*>& children,
                          std::chrono::seconds timeout = std::chrono::seconds(20)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto pos = rx_.find(delimiter);
            if (pos != std::string::npos) {
                std::string line = rx_.substr(0, pos + 1);
                rx_.erase(0, pos + 1);
                return line;
            }
            readSome(children, deadline);
        }
        throw std::runtime_error("timed out waiting for command line");
    }

    std::vector<uint8_t> readBytes(size_t count,
                                   std::vector<ChildProcess*>& children,
                                   std::chrono::seconds timeout = std::chrono::seconds(60)) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (rx_.size() >= count) {
                std::vector<uint8_t> out(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(count));
                rx_.erase(0, count);
                return out;
            }
            readSome(children, deadline);
        }
        throw std::runtime_error("timed out waiting for data payload");
    }

private:
    void readSome(std::vector<ChildProcess*>& children,
                  std::chrono::steady_clock::time_point deadline) {
        pumpChildren(children);
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return;
        }
        const int wait_ms = std::min<int>(
            50,
            static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count()));
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN | POLLERR | POLLHUP | POLLNVAL;
        const int rc = ::poll(&pfd, 1, wait_ms);
        if (rc <= 0) {
            return;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            throw std::runtime_error("socket closed while reading");
        }
        std::array<char, 4096> buffer{};
        const ssize_t n = ::recv(fd_, buffer.data(), buffer.size(), 0);
        if (n > 0) {
            rx_.append(buffer.data(), static_cast<size_t>(n));
            return;
        }
        if (n == 0) {
            throw std::runtime_error("socket closed");
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            throw std::runtime_error("socket read failed");
        }
    }

    int fd_ = -1;
    std::string rx_;
};

std::string command(TcpClient& client,
                    const std::string& line,
                    std::vector<ChildProcess*>& children) {
    client.writeString(line + "\r");
    return client.readUntil('\r', children);
}

void expectCommand(TcpClient& client,
                   const std::string& line,
                   const std::string& expected,
                   std::vector<ChildProcess*>& children) {
    const std::string actual = command(client, line, children);
    check(actual == expected,
          "command '" + line + "' got [" + actual + "] expected [" + expected + "]");
}

std::string waitForLineStarting(TcpClient& client,
                                const std::string& prefix,
                                std::vector<ChildProcess*>& children,
                                std::chrono::seconds timeout = std::chrono::seconds(45)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::seconds>(
            deadline - std::chrono::steady_clock::now());
        const std::string line = client.readUntil(
            '\r',
            children,
            std::max(std::chrono::seconds(1), remaining));
        if (line.rfind(prefix, 0) == 0) {
            return line;
        }
    }
    throw std::runtime_error("timed out waiting for command line starting with " + prefix);
}

}  // namespace

int main(int argc, char** argv) {
    ChildProcess daemon;
    ChildProcess alice;
    ChildProcess bob;

    try {
        check(argc == 3, "usage: test_ultra_tnc_sim_audio <ota_simulator> <ultra_tnc>");
        ::signal(SIGPIPE, SIG_IGN);

        ultra::test::TempDir temp("ultra_tnc_sim_audio");
        check(temp.valid(), "failed to create temp dir");

        const auto token_path = temp.child("tokens.conf");
        {
            std::ofstream out(token_path);
            out << "alice_token:ALICE:Alice station\n";
            out << "bob_token:BOB:Bob station\n";
        }

        daemon = startDaemon(argv[1], token_path, temp.child("captures"));
        const ReadyBanner ready = waitForReady(daemon);

        const uint16_t alice_port = reservePortPair();
        uint16_t bob_port = reservePortPair();
        while (bob_port == alice_port || bob_port == alice_port + 1 ||
               alice_port == bob_port + 1) {
            bob_port = reservePortPair();
        }

        alice = startTnc(argv[2], ready.grpc_target, "alice_token", "alice", "ALICE", alice_port);
        bob = startTnc(argv[2], ready.grpc_target, "bob_token", "bob", "BOB", bob_port);

        std::vector<ChildProcess*> children{&daemon, &alice, &bob};
        waitForOutput(alice, "ultra_tnc listening", children, std::chrono::seconds(10));
        waitForOutput(bob, "ultra_tnc listening", children, std::chrono::seconds(10));
        waitForOutput(alice, "[otasim] Connected", children, std::chrono::seconds(10));
        waitForOutput(bob, "[otasim] Connected", children, std::chrono::seconds(10));

        TcpClient alice_cmd;
        TcpClient bob_cmd;
        TcpClient alice_data;
        TcpClient bob_data;
        alice_cmd.connectTo(alice_port, children);
        bob_cmd.connectTo(bob_port, children);
        alice_data.connectTo(static_cast<uint16_t>(alice_port + 1), children);
        bob_data.connectTo(static_cast<uint16_t>(bob_port + 1), children);

        expectCommand(bob_cmd, "MYCALL BOB", "OK\r", children);
        expectCommand(bob_cmd, "LISTEN ON", "OK\r", children);
        expectCommand(alice_cmd, "MYCALL ALICE", "OK\r", children);
        expectCommand(alice_cmd, "CONNECT ALICE BOB", "OK\r", children);

        const std::string alice_connected = waitForLineStarting(alice_cmd, "CONNECTED ", children);
        const std::string bob_connected = waitForLineStarting(bob_cmd, "CONNECTED ", children);
        check(alice_connected.find("ALICE BOB") != std::string::npos,
              "alice CONNECTED line mismatch: " + alice_connected);
        check(bob_connected.find("ALICE BOB") != std::string::npos,
              "bob CONNECTED line mismatch: " + bob_connected);

        const std::string text = "ProjectUltra ultra_tnc OTASim payload";
        const std::vector<uint8_t> payload(text.begin(), text.end());
        alice_data.writeBytes(payload);

        const std::vector<uint8_t> received = bob_data.readBytes(payload.size(), children);
        check(received == payload, "payload mismatch over ultra_tnc --sim-audio");

        alice_data.close();
        bob_data.close();
        alice_cmd.close();
        bob_cmd.close();

        alice.terminateCleanly();
        bob.terminateCleanly();
        daemon.terminateCleanly();

        std::cout << "ultra_tnc OTASim client delivered payload and shut down cleanly\n";
        return 0;
    } catch (const std::exception& e) {
        alice.killNow();
        bob.killNow();
        daemon.killNow();
        std::cerr << "test_ultra_tnc_sim_audio: " << e.what() << "\n";
        if (!alice.captured.empty()) {
            std::cerr << "\n[alice]\n" << alice.captured << "\n";
        }
        if (!bob.captured.empty()) {
            std::cerr << "\n[bob]\n" << bob.captured << "\n";
        }
        if (!daemon.captured.empty()) {
            std::cerr << "\n[daemon]\n" << daemon.captured << "\n";
        }
        return 1;
    }
}

#endif
