/**
 * CLI Simulator - Single Audio I/O Thread Model (like real sound card)
 *
 * Each station has ONE audio thread that handles both TX and RX,
 * exactly like a real sound card callback:
 *   - Every 10ms, read RX samples from channel
 *   - Feed RX to StreamingDecoder
 *   - Check if we have TX samples pending
 *   - Send TX samples to channel
 *
 * REFACTORED: Uses IWaveform + StreamingDecoder directly (not ModemEngine)
 * This ensures consistent configuration between TX and RX.
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <memory>
#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <csignal>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#endif
#include "ultra/timing_profiler.hpp"
#include <queue>
#include <algorithm>
#include <cmath>
#include <functional>
#include <sstream>
#include <array>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <utility>

#include <grpcpp/grpcpp.h>

#include "ota_simulator.grpc.pb.h"
#include "otasim_client/ota_audio_backend.hpp"
#include "waveform/waveform_factory.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "waveform/ofdm_cox_waveform.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"  // TX encoding (mirrors StreamingDecoder)
#include "protocol/protocol_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"
#include "sim/cli_enums.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/fec.hpp"  // ChannelInterleaver, LDPCEncoder
#include "ultra/dsp.hpp"  // FFT for analytic-signal CFO injection
#include "fec/frame_interleaver.hpp"  // FrameInterleaver
#include "sim/channel_snr_probe.hpp"
#include "sim/hf_channel.hpp"
#include "sim/simulated_station.hpp"

#ifdef ULTRA_HAVE_SDL2
#include "gui/audio_engine.hpp"  // Real SDL2 audio I/O for --role A|B
#endif

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::protocol;
using namespace ultra::sim;
namespace cli = ultra::tools::cli;
namespace otasim = projectultra::otasim::v1;
namespace otasim_client = ultra::otasim_client;

namespace {

std::string audioDeviceLabel(const std::string& device) {
    return (device.empty() || cli::normalizedToken(device) == "default") ? "Default" : device;
}

void printCliAudioDeviceHint() {
    std::cerr << "Next step: run: cli_simulator --list-audio-devices --role A\n"
              << "Then pass the exact names with --audio-output and --audio-input.\n";
}

bool envFlagEnabled(const char* name) {
    if (const char* value = std::getenv(name)) {
        const std::string v = cli::normalizedToken(value);
        return v == "1" || v == "true" || v == "yes" || v == "on";
    }
    return false;
}

const char* mcDpskPresetChoices() {
    return "standard, robust_low, robust_mid, robust";
}

bool parseMCDPSKPreset(const std::string& value,
                       std::string& preset_name,
                       MultiCarrierDPSKConfig& config) {
    const std::string v = cli::normalizedToken(value);
    if (v == "standard") {
        preset_name = "standard";
        config = mc_dpsk_presets::level8();
        return true;
    }
    if (v == "robust_low" || v == "robustlow") {
        preset_name = "robust_low";
        config = mc_dpsk_presets::robust_low();
        return true;
    }
    if (v == "robust_mid" || v == "robustmid") {
        preset_name = "robust_mid";
        config = mc_dpsk_presets::robust_mid();
        return true;
    }
    if (v == "robust") {
        preset_name = "robust";
        config = mc_dpsk_presets::robust();
        return true;
    }
    return false;
}

std::string mcDpskPresetSummary(const std::string& preset_name,
                                const MultiCarrierDPSKConfig& config) {
    std::ostringstream oss;
    oss << preset_name;
    if (preset_name == "adaptive") {
        oss << " listen=robust_mid";
    }
    oss << " (" << config.num_carriers << " carriers, "
        << config.samples_per_symbol << " sps, "
        << (config.bits_per_symbol == 1 ? "DBPSK" : "DQPSK")
        << ", raw=" << std::fixed << std::setprecision(1)
        << config.getRawBitRate() << " bps)";
    return oss.str();
}

bool parseForcedModulation(const std::string& value, bool expert_phy, Modulation& out) {
    const auto parsed_any = cli::parseModulation(
        value, cli::AllowAuto::No, cli::AllowExperimentalModulation::Yes);
    if (!parsed_any) {
        std::cerr << "Unknown modulation: " << value
                  << " (use "
                  << cli::modulationChoices(
                         cli::AllowAuto::No,
                         expert_phy ? cli::AllowExperimentalModulation::Yes
                                    : cli::AllowExperimentalModulation::No)
                  << ")\n";
        return false;
    }
    if (cli::isExpertOnlyModulation(*parsed_any)) {
        if (!expert_phy) {
            std::cerr << "EXPERT PHY MODE BLOCKED: --mod " << value
                      << " is outside the operator ladder. Re-run with --expert "
                         "only for controlled lab testing.\n";
            return false;
        }
        std::cerr << "EXPERT PHY MODE: forcing --mod " << value
                  << " outside the operator ladder; results are lab-only.\n";
    }
    out = *parsed_any;
    return true;
}

constexpr const char* kOtaDefaultSession = "lobby";
constexpr const char* kOtaAlphaToken = "alpha_token";
constexpr const char* kOtaBravoToken = "bravo_token";

void addOtaToken(grpc::ClientContext& context, const std::string& token) {
    context.AddMetadata("authorization", "Bearer " + token);
}

void setRpcDeadline(grpc::ClientContext& context, std::chrono::milliseconds duration) {
    context.set_deadline(std::chrono::system_clock::now() + duration);
}

std::string channelModelForOta(ChannelType type) {
    switch (type) {
        case ChannelType::PASSTHROUGH: return "passthrough";
        case ChannelType::AWGN:        return "awgn";
        case ChannelType::GOOD:        return "watterson_good";
        case ChannelType::MODERATE:    return "watterson_moderate";
        case ChannelType::POOR:        return "watterson_poor";
        case ChannelType::FLUTTER:     return "watterson_flutter";
        default:                       return "awgn";
    }
}

std::optional<ChannelType> parseFadingType(const std::string& value) {
    const std::string v = cli::normalizedToken(value);
    if (v == "none" || v == "awgn") {
        return ChannelType::AWGN;
    }
    return cli::parseChannelType(value, cli::AllowAwgn::No);
}

std::string defaultOtaSimulatorPath(const char* argv0) {
    std::filesystem::path exe = argv0 ? std::filesystem::absolute(argv0) : std::filesystem::path{};
    std::filesystem::path sibling = exe.parent_path() / "ota_simulator";
#ifdef _WIN32
    sibling += ".exe";
#endif
    if (std::filesystem::exists(sibling)) {
        return sibling.string();
    }
    return "ota_simulator";
}

bool setOtaChannel(const std::string& grpc_target,
                   const std::string& token,
                   const std::string& session_id,
                   ChannelType channel_type,
                   float snr_db,
                   uint64_t seed,
                   std::string* error) {
    auto channel = grpc::CreateChannel(grpc_target, grpc::InsecureChannelCredentials());
    if (!channel->WaitForConnected(std::chrono::system_clock::now() +
                                   std::chrono::seconds(5))) {
        if (error) *error = "gRPC channel did not connect for SetChannel";
        return false;
    }
    auto stub = otasim::OtaSimulatorControl::NewStub(channel);
    otasim::SetChannelRequest request;
    request.set_session_id(session_id);
    request.set_model(channelModelForOta(channel_type));
    request.set_snr_db(snr_db);
    request.set_seed(seed);
    otasim::CommandAck ack;
    grpc::ClientContext context;
    addOtaToken(context, token);
    setRpcDeadline(context, std::chrono::milliseconds(1500));
    const auto status = stub->SetChannel(&context, request, &ack);
    if (!status.ok()) {
        if (error) *error = "SetChannel failed: " + status.error_message();
        return false;
    }
    if (!ack.accepted()) {
        if (error) *error = "SetChannel rejected: " + ack.message();
        return false;
    }
    return true;
}

bool startOtaCapture(const std::string& grpc_target,
                     const std::string& token,
                     const std::string& session_id,
                     std::string* capture_path,
                     std::string* error) {
    auto channel = grpc::CreateChannel(grpc_target, grpc::InsecureChannelCredentials());
    if (!channel->WaitForConnected(std::chrono::system_clock::now() +
                                   std::chrono::seconds(5))) {
        if (error) *error = "gRPC channel did not connect for StartCapture";
        return false;
    }
    auto stub = otasim::OtaSimulatorControl::NewStub(channel);
    otasim::StartCaptureRequest request;
    request.set_session_id(session_id);
    request.set_start_sample(8);
    otasim::CaptureInfo info;
    grpc::ClientContext context;
    addOtaToken(context, token);
    setRpcDeadline(context, std::chrono::milliseconds(1500));
    const auto status = stub->StartCapture(&context, request, &info);
    if (!status.ok()) {
        if (error) *error = "StartCapture failed: " + status.error_message();
        return false;
    }
    if (capture_path) {
        *capture_path = info.capture_path();
    }
    return true;
}

bool stopOtaCapture(const std::string& grpc_target,
                    const std::string& token,
                    const std::string& session_id,
                    std::string* capture_path,
                    std::string* error) {
    auto channel = grpc::CreateChannel(grpc_target, grpc::InsecureChannelCredentials());
    if (!channel->WaitForConnected(std::chrono::system_clock::now() +
                                   std::chrono::seconds(5))) {
        if (error) *error = "gRPC channel did not connect for StopCapture";
        return false;
    }
    auto stub = otasim::OtaSimulatorControl::NewStub(channel);
    otasim::StopCaptureRequest request;
    request.set_session_id(session_id);
    otasim::CaptureInfo info;
    grpc::ClientContext context;
    addOtaToken(context, token);
    setRpcDeadline(context, std::chrono::milliseconds(1500));
    const auto status = stub->StopCapture(&context, request, &info);
    if (!status.ok()) {
        if (error) *error = "StopCapture failed: " + status.error_message();
        return false;
    }
    if (capture_path) {
        *capture_path = info.capture_path();
    }
    return true;
}

class OtaAudioPort : public AudioPort {
public:
    OtaAudioPort(otasim_client::OtaAudioBackendConfig config, std::string label)
        : config_(std::move(config)), label_(std::move(label)) {}

    bool start() override {
        std::string error;
        if (!backend_.start(config_, &error)) {
            std::cerr << label_ << " OTASim audio start failed: " << error << "\n";
            return false;
        }
        if (!backend_.waitForConnected(std::chrono::seconds(10))) {
            const auto status = backend_.status();
            std::cerr << label_ << " OTASim audio connect timeout: " << status.text << "\n";
            backend_.close();
            return false;
        }
        return true;
    }

    void stop() override {
        backend_.close();
    }

    std::vector<float> pullRx(size_t count) override {
        return backend_.getRxSamples(count);
    }

    void queueTx(const std::vector<float>& samples) override {
        if (samples.empty()) {
            return;
        }
        std::string error;
        if (!backend_.queueTxSamples(samples, &error)) {
            tx_errors_++;
            if (tx_errors_ <= 4 || (tx_errors_ % 32) == 0) {
                LOG_MODEM(WARN, "%s OTASim TX failed: %s", label_.c_str(), error.c_str());
            }
        }
    }

private:
    otasim_client::OtaAudioBackendConfig config_;
    std::string label_;
    otasim_client::OtaAudioBackend backend_;
    uint64_t tx_errors_ = 0;
};

#ifndef _WIN32
class LocalOtaServer {
public:
    ~LocalOtaServer() {
        killNow();
    }

    bool start(const std::string& binary,
               ChannelType channel_type,
               float snr_db,
               uint64_t seed,
               std::string* error) {
        if (binary.empty()) {
            if (error) *error = "ota_simulator binary path is empty";
            return false;
        }
        const std::filesystem::path bin_path(binary);
        if (!std::filesystem::exists(bin_path)) {
            if (error) *error = "ota_simulator binary not found: " + binary;
            return false;
        }

        const uint16_t port = findFreeTcpPort(error);
        if (port == 0) {
            return false;
        }

        const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        temp_dir_ = std::filesystem::temp_directory_path() /
                    ("cli_otasim_" + std::to_string(::getpid()) + "_" +
                     std::to_string(stamp));
        capture_root_ = temp_dir_ / "captures";
        token_path_ = temp_dir_ / "tokens.conf";
        log_path_ = temp_dir_ / "ota_simulator.log";

        std::error_code ec;
        std::filesystem::create_directories(capture_root_, ec);
        if (ec) {
            if (error) *error = "failed to create OTASim temp dir: " + ec.message();
            return false;
        }
        {
            std::ofstream out(token_path_);
            if (!out) {
                if (error) *error = "failed to write OTASim token file";
                return false;
            }
            out << kOtaAlphaToken << ":ALPHA:Alpha station\n";
            out << kOtaBravoToken << ":BRAVO:Bravo station\n";
        }

        const int log_fd = ::open(log_path_.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0600);
        if (log_fd < 0) {
            if (error) *error = "failed to open OTASim log file";
            return false;
        }

        const std::string bind = "127.0.0.1:" + std::to_string(port);
        const std::string snr = std::to_string(snr_db);
        const std::string seed_arg = std::to_string(seed);
        const std::string model = channelModelForOta(channel_type);

        const pid_t pid = ::fork();
        if (pid < 0) {
            ::close(log_fd);
            if (error) *error = "fork failed";
            return false;
        }
        if (pid == 0) {
            ::dup2(log_fd, STDOUT_FILENO);
            ::dup2(log_fd, STDERR_FILENO);
            ::close(log_fd);
            ::execl(bin_path.c_str(),
                    bin_path.c_str(),
                    "serve",
                    "--bind", bind.c_str(),
                    "--udp-bind", "127.0.0.1:0",
                    "--tokens", token_path_.c_str(),
                    "--captures-root", capture_root_.c_str(),
                    "--lobby-channel", model.c_str(),
                    "--lobby-snr-db", snr.c_str(),
                    "--lobby-seed", seed_arg.c_str(),
                    "--shutdown-deadline-sec", "2",
                    nullptr);
            ::_exit(127);
        }

        ::close(log_fd);
        pid_ = pid;
        grpc_target_ = bind;
        if (!waitForReady(error)) {
            killNow();
            return false;
        }
        return true;
    }

    bool terminateCleanly(std::string* error) {
        if (pid_ <= 0 || reaped_) {
            return true;
        }
        if (::kill(pid_, SIGTERM) != 0) {
            if (error) *error = "failed to send SIGTERM to ota_simulator";
            killNow();
            return false;
        }
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
        while (std::chrono::steady_clock::now() < deadline) {
            int status = 0;
            const pid_t result = ::waitpid(pid_, &status, WNOHANG);
            if (result == pid_) {
                markReaped(status);
                if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
                    return true;
                }
                if (error) *error = "ota_simulator did not exit cleanly after SIGTERM";
                return false;
            }
            if (result < 0 && errno != EINTR) {
                if (error) *error = "waitpid failed during OTASim shutdown";
                killNow();
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (error) *error = "ota_simulator did not exit before SIGTERM timeout";
        killNow();
        return false;
    }

    void killNow() {
        if (pid_ > 0 && !reaped_) {
            (void)::kill(pid_, SIGKILL);
            int status = 0;
            (void)::waitpid(pid_, &status, 0);
            markReaped(status);
        }
    }

    const std::string& grpcTarget() const { return grpc_target_; }
    const std::filesystem::path& captureRoot() const { return capture_root_; }
    const std::filesystem::path& logPath() const { return log_path_; }

private:
    static uint16_t findFreeTcpPort(std::string* error) {
        const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) {
            if (error) *error = "failed to create TCP socket";
            return 0;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(0);
        if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(fd);
            if (error) *error = "failed to bind ephemeral TCP port";
            return 0;
        }
        sockaddr_in actual{};
        socklen_t len = sizeof(actual);
        if (::getsockname(fd, reinterpret_cast<sockaddr*>(&actual), &len) != 0) {
            ::close(fd);
            if (error) *error = "failed to inspect ephemeral TCP port";
            return 0;
        }
        const uint16_t port = ntohs(actual.sin_port);
        ::close(fd);
        return port;
    }

    bool waitForReady(std::string* error) {
        auto channel = grpc::CreateChannel(grpc_target_, grpc::InsecureChannelCredentials());
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (std::chrono::steady_clock::now() < deadline) {
            if (channel->WaitForConnected(std::chrono::system_clock::now() +
                                          std::chrono::milliseconds(200))) {
                return true;
            }
            int status = 0;
            const pid_t result = ::waitpid(pid_, &status, WNOHANG);
            if (result == pid_) {
                markReaped(status);
                if (error) {
                    *error = "ota_simulator exited before ready; log=" + log_path_.string();
                }
                return false;
            }
            if (result < 0 && errno != EINTR) {
                if (error) *error = "waitpid failed while waiting for OTASim";
                return false;
            }
        }
        if (error) {
            *error = "timed out waiting for ota_simulator; log=" + log_path_.string();
        }
        return false;
    }

    void markReaped(int status) {
        status_ = status;
        reaped_ = true;
        pid_ = -1;
    }

    pid_t pid_ = -1;
    bool reaped_ = false;
    int status_ = 0;
    std::string grpc_target_;
    std::filesystem::path temp_dir_;
    std::filesystem::path token_path_;
    std::filesystem::path capture_root_;
    std::filesystem::path log_path_;
};
#else
class LocalOtaServer {
public:
    bool start(const std::string&, ChannelType, float, uint64_t, std::string* error) {
        if (error) *error = "cli_simulator self-spawn is not implemented on Windows; pass --ota-host";
        return false;
    }
    bool terminateCleanly(std::string*) { return true; }
    void killNow() {}
    const std::string& grpcTarget() const { return grpc_target_; }
    const std::filesystem::path& captureRoot() const { return empty_path_; }
    const std::filesystem::path& logPath() const { return empty_path_; }
private:
    std::string grpc_target_;
    std::filesystem::path empty_path_;
};
#endif

}  // namespace

// Channel condition types (ITU-R F.1487)
#ifdef ULTRA_HAVE_SDL2
/**
     * ChannelInjector - Streaming TX-side channel emulator. Applies CFO +
     * Watterson fading + AWGN to each transmitted audio buffer before it
     * reaches the soundcard.
 *
 * Wraps SimulatedChannel and uses only its A→B direction:
 *   transmitFromA(tx)     -> processed samples land in buffer_b_rx_
 *   receiveForB(tx.size())-> drains them back as the channel-degraded TX
 *
 * Buffer never underflows because we push and drain in lockstep, so the
 * "noise on underflow" path in receiveForB doesn't fire.
 */
class ChannelInjector {
public:
    ChannelInjector(float snr_db, ChannelType ch_type,
                    uint32_t seed, float tx_cfo_hz,
                    float output_gain = 0.70f)
        : output_gain_(std::clamp(output_gain, 0.05f, 1.0f)) {
        sim_channel_.setSeed(seed);
        sim_channel_.setTxCFO(tx_cfo_hz);
        sim_channel_.configure(snr_db, ch_type);
    }

    std::vector<float> process(const std::vector<float>& tx) {
        sim_channel_.transmitFromA(tx);
        auto out = sim_channel_.receiveForB(tx.size());
        const float rms_before_gain = sampleRms(out);
        const float peak_before_gain = samplePeak(out);
        const size_t clipped_before_gain = countFullScaleSamples(out);
        if (output_gain_ != 1.0f) {
            for (float& sample : out) {
                sample *= output_gain_;
            }
        }
        process_count_++;
        if (process_count_ <= 8 || (process_count_ % 32) == 0) {
            LOG_MODEM(INFO,
                      "ChannelInjector: process #%llu samples=%zu "
                      "gain=%.3f rms_in=%.4f peak_in=%.4f "
                      "rms_ch=%.4f peak_ch=%.4f clip_ch=%zu "
                      "rms_out=%.4f peak_out=%.4f clip_out=%zu",
                      static_cast<unsigned long long>(process_count_),
                      tx.size(), output_gain_, sampleRms(tx), samplePeak(tx),
                      rms_before_gain, peak_before_gain, clipped_before_gain,
                      sampleRms(out), samplePeak(out), countFullScaleSamples(out));
        }
        return out;
    }

private:
    SimulatedChannel sim_channel_;
    float output_gain_ = 0.70f;
    uint64_t process_count_ = 0;
};

/**
 * HardwareAudioPort - Real soundcard I/O via SDL2 (gui::AudioEngine).
 * Used for --role A|B mode when running across two physical machines
 * connected by an audio cable (or speaker/mic).
 *
     * On RX short read, waits for the SDL capture period to finish before
     * padding. Padding immediately would synthesize callback-sized sample gaps.
 *
 * If a ChannelInjector is supplied, TX samples pass through it before
 * hitting the soundcard, so the receiving station sees a realistic
 * channel-degraded signal even on a clean audio cable.
 */
class HardwareAudioPort : public AudioPort {
public:
    HardwareAudioPort(const std::string& output_device,
                      const std::string& input_device,
                      std::unique_ptr<ChannelInjector> injector = nullptr,
                      int buffer_size = 0)
        : output_device_(output_device),
          input_device_(input_device),
          injector_(std::move(injector)),
          buffer_size_(buffer_size) {}

    bool start() override {
        if (!engine_.initialize()) {
            std::cerr << "AudioEngine init failed\n";
            printCliAudioDeviceHint();
            return false;
        }
        if (buffer_size_ > 0) engine_.setBufferSize(buffer_size_);
        if (!engine_.openOutput(output_device_)) {
            std::cerr << "Failed to open output device '"
                      << audioDeviceLabel(output_device_) << "'\n";
            printCliAudioDeviceHint();
            return false;
        }
        if (!engine_.openInput(input_device_)) {
            std::cerr << "Failed to open input device '"
                      << audioDeviceLabel(input_device_) << "'\n";
            printCliAudioDeviceHint();
            return false;
        }
        engine_.startPlayback();
        engine_.startCapture();
        return true;
    }

    void stop() override {
        engine_.stopCapture();
        engine_.stopPlayback();
        engine_.closeInput();
        engine_.closeOutput();
        engine_.shutdown();
    }

    std::vector<float> pullRx(size_t count) override {
        const int sample_rate = std::max(1, engine_.getSampleRate());
        const int period_ms = std::max(1, (engine_.getBufferSize() * 1000 + sample_rate - 1) / sample_rate);
        const int wait_ms = std::clamp(period_ms * 2 + 20, 50, 1000);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);

        while (engine_.getRxBufferSize() < count &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        auto got = engine_.getRxSamples(count);
        if (got.size() < count) {
            rx_short_reads_++;
            rx_padded_samples_ += (count - got.size());
            if (rx_short_reads_ <= 4 || (rx_short_reads_ % 16) == 0) {
                LOG_MODEM(WARN,
                          "HardwareAudioPort: RX short read after %dms wait "
                          "(got=%zu need=%zu, padded_total=%llu, events=%llu)",
                          wait_ms, got.size(), count,
                          static_cast<unsigned long long>(rx_padded_samples_),
                          static_cast<unsigned long long>(rx_short_reads_));
            }
            // Only pad after waiting for real captured samples. Padding before
            // the SDL capture period completes creates artificial 400-500 sample
            // discontinuities when the hardware callback size is not 480.
            got.resize(count, 0.0f);
        }
        return got;
    }

    void queueTx(const std::vector<float>& samples) override {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        std::vector<float> queued_samples;
        if (injector_) {
            queued_samples = injector_->process(samples);
        } else {
            queued_samples = samples;
        }
        engine_.queueTxSamples(queued_samples);

        tx_queue_events_++;
        if (tx_queue_events_ <= 8 || (tx_queue_events_ % 32) == 0) {
            LOG_MODEM(INFO,
                      "HardwareAudioPort: TX queue #%llu inject=%s "
                      "in=%zu rms=%.4f peak=%.4f out=%zu rms=%.4f peak=%.4f "
                      "device='%s'",
                      static_cast<unsigned long long>(tx_queue_events_),
                      injector_ ? "yes" : "no",
                      samples.size(), sampleRms(samples), samplePeak(samples),
                      queued_samples.size(), sampleRms(queued_samples),
                      samplePeak(queued_samples), output_device_.c_str());
        }
    }

    bool shouldPaceTxInStationLoop() const override { return false; }

private:
    gui::AudioEngine engine_;
    std::string output_device_;
    std::string input_device_;
    std::unique_ptr<ChannelInjector> injector_;
    int buffer_size_ = 0;  // 0 = engine default
    std::mutex tx_mutex_;
    uint64_t rx_short_reads_ = 0;
    uint64_t rx_padded_samples_ = 0;
    uint64_t tx_queue_events_ = 0;
};
#endif  // ULTRA_HAVE_SDL2

/**
 * SimulatedStation - One station with single audio I/O thread
 *
 * Uses IWaveform for TX and StreamingDecoder for RX (NOT ModemEngine).
 * This ensures consistent configuration between TX and RX paths.
 */
// =============================================================================
// MAIN SIMULATOR
// =============================================================================

class CLISimulator {
public:
    void setSNR(float snr) { snr_db_ = snr; }
    void setVerbose(bool v) { verbose_ = v; }
    void setFading(bool f) { use_fading_ = f; }
    void setChannelType(ChannelType t) { channel_type_ = t; use_fading_ = (t != ChannelType::AWGN); }
    void setForcedModulation(Modulation mod) { forced_mod_ = mod; }
    void setForcedCodeRate(CodeRate rate) { forced_rate_ = rate; }
    void setOFDMConfigPreset(OFDMConfigPreset preset) { ofdm_config_preset_ = preset; }
    void setMCDPSKPreset(const std::string& name, const MultiCarrierDPSKConfig& config) {
        mc_dpsk_preset_name_ = name;
        mc_dpsk_config_ = config;
        forced_waveform_ = WaveformMode::MC_DPSK;
        forced_mod_ = mc_dpsk_config_.bits_per_symbol == 1
            ? Modulation::DBPSK
            : Modulation::DQPSK;
    }
    // Marks an operator-forced override; remembered so initStation()
    // pushes it into the protocol layer with forced=true (which makes
    // the initiator embed it in CONNECT and the responder honor it).
    void setFixedFrameCodewords(int cw_count) {
        fixed_frame_codewords_ = v2::sanitizeFixedFrameCodewords(cw_count);
        cw_count_forced_ = true;
    }
    void setCarrierMask(uint64_t active_mask) { carrier_mask_ = active_mask; }
    void setPreferredWaveform(WaveformMode mode) { forced_waveform_ = mode; }
    void setTestFileTransfer(bool v) { test_file_transfer_ = v; }
    void setTestFileSize(size_t bytes) { test_file_size_ = bytes; }
    void setChannelInterleave(bool enable) { use_channel_interleave_ = enable; }
    void setNoBurstInterleave(bool v) { no_burst_interleave_ = v; }
    void setBurstInterleaveGroupSize(int n) {
        burst_group_size_ = ofdm_link_adaptation::sanitizeBurstGroupSize(n);
    }
    void setTestBurst(bool v) { test_burst_ = v; }
    void setSeed(uint32_t seed) { seed_ = seed; }
    void setTxCFO(float cfo_hz) { tx_cfo_hz_ = cfo_hz; }
    void setSaveSignals(bool enable, int message_limit = 0) {
        save_signals_ = enable;
        save_signals_message_limit_ = std::max(0, message_limit);
    }
    void setSaveSignalsPrefix(const std::string& prefix) { save_signals_prefix_ = prefix; }
    void setSaveSignalsMaxSamples(size_t max_samples) { save_signals_max_samples_ = max_samples; }
    void setAdaptiveTest(bool enable) { adaptive_test_ = enable; }
    void setAdaptiveHopSNR(float snr) { adaptive_hop_snr_db_ = snr; }
    void setAdaptiveHopChannel(ChannelType t) { adaptive_hop_channel_ = t; }
    void setRxOverfeedFactor(int factor) { rx_overfeed_factor_ = std::clamp(factor, 1, 200); }
    void setDecodeDelayMs(int ms) { decode_delay_ms_ = std::clamp(ms, 0, 500); }
    void setRxBatchCallbacks(int n) { rx_batch_callbacks_ = std::clamp(n, 1, 1000); }
    void setSoftCombiningHARQ(bool enable) { soft_combining_harq_ = enable; }

    // Hardware-audio mode (real soundcard I/O across two physical machines)
    void setRoleBoth() { role_ = Role::Both; }
    void setRoleA() { role_ = Role::A; }
    void setRoleB() { role_ = Role::B; }
    void setSelfCallsign(const std::string& c) { self_callsign_ = c; }
    void setPeerCallsign(const std::string& c) { peer_callsign_ = c; }
    void setAudioOutputDevice(const std::string& d) { audio_output_device_ = d; }
    void setAudioInputDevice(const std::string& d) { audio_input_device_ = d; }
    void setListAudioDevices(bool v) { list_audio_devices_ = v; }
    void setRoleBIdleSeconds(int s) { role_b_idle_seconds_ = std::max(0, s); }
    void setInjectChannel(bool v) { inject_channel_ = v; }
    void setInjectGain(float gain) { inject_gain_ = std::clamp(gain, 0.05f, 1.0f); }
    void setAudioBufferSize(int n) { audio_buffer_size_ = n; }
    void setVerifySNR(bool v) { verify_snr_ = v; }
    void setOtaHost(const std::string& host) { ota_host_ = host; }
    void setOtaSessionId(const std::string& session_id) { ota_session_id_ = session_id.empty() ? kOtaDefaultSession : session_id; }
    void setOtaAlphaToken(const std::string& token) { ota_alpha_token_ = token; }
    void setOtaBravoToken(const std::string& token) { ota_bravo_token_ = token; }
    void setOtaSimulatorPath(const std::string& path) { ota_server_binary_ = path; }

    bool runTest() {
        if (verify_snr_ && !runSNRVerification()) {
            return false;
        }

        // Hardware-audio mode (real soundcard, single station per process).
        // Dispatched here so we don't spin up the in-process SimulatedChannel
        // or two-station orchestration.
        if (role_ != Role::Both) {
            return runHardwareTest();
        }

        printHeader();

        LocalOtaServer local_server;
        active_ota_grpc_target_ = ota_host_;
        const bool spawned_local = active_ota_grpc_target_.empty();
        if (active_ota_grpc_target_.empty()) {
            std::string error;
            if (!local_server.start(ota_server_binary_, channel_type_, snr_db_, seed_, &error)) {
                std::cerr << "Failed to start local ota_simulator serve: " << error << "\n";
                return false;
            }
            active_ota_grpc_target_ = local_server.grpcTarget();
            std::cout << "  OTASim: spawned " << active_ota_grpc_target_
                      << " (captures " << local_server.captureRoot().string() << ")\n";
        } else {
            std::cout << "  OTASim: using " << active_ota_grpc_target_ << "\n";
        }

        {
            std::string error;
            if (!setOtaChannel(active_ota_grpc_target_, ota_alpha_token_,
                               ota_session_id_, channel_type_, snr_db_, seed_, &error)) {
                std::cerr << "Failed to configure OTASim channel: " << error << "\n";
                return false;
            }
            std::cout << "  OTASim channel: " << channelModelForOta(channel_type_)
                      << " @ " << snr_db_ << " dB, seed=" << seed_ << "\n";
        }

        ota_capture_active_ = false;
        ota_capture_path_.clear();
        if (save_signals_) {
            capture_limit_hit_.store(false);
            std::string error;
            if (startOtaCapture(active_ota_grpc_target_, ota_alpha_token_,
                                ota_session_id_, &ota_capture_path_, &error)) {
                ota_capture_active_ = true;
                std::cout << "  [capture] OTASim capture enabled";
                if (!ota_capture_path_.empty()) {
                    std::cout << " at " << ota_capture_path_;
                }
                std::cout << "\n";
            } else {
                std::cout << "  \033[33m[capture] warning: " << error << "\033[0m\n";
            }
        }

        otasim_client::OtaAudioBackendConfig alpha_ota;
        alpha_ota.grpc_target = active_ota_grpc_target_;
        alpha_ota.token = ota_alpha_token_;
        alpha_ota.station_id = "ALPHA";
        alpha_ota.session_id = ota_session_id_;

        otasim_client::OtaAudioBackendConfig bravo_ota;
        bravo_ota.grpc_target = active_ota_grpc_target_;
        bravo_ota.token = ota_bravo_token_;
        bravo_ota.station_id = "BRAVO";
        bravo_ota.session_id = ota_session_id_;

        // Simulator stations have zero PTT-off settling, so the
        // 500 ms connection-level holds added for real-radio safety
        // become pure latency that drives spurious ARQ retx. Override
        // to 0 here (same pattern as tools/ota_simulator/runner_v2.cpp).
        protocol::ConnectionConfig connection_config;
        connection_config.pong_tx_delay_ms = 0;
        connection_config.post_connect_data_delay_ms = 0;
        connection_config.ack_tx_delay_ms = 0;

        alpha_ = std::make_unique<SimulatedStation>(
            "ALPHA", std::make_unique<OtaAudioPort>(std::move(alpha_ota), "ALPHA"),
            ofdm_config_preset_, mc_dpsk_config_, connection_config);
        bravo_ = std::make_unique<SimulatedStation>(
            "BRAVO", std::make_unique<OtaAudioPort>(std::move(bravo_ota), "BRAVO"),
            ofdm_config_preset_, mc_dpsk_config_, connection_config);
        alpha_->setRxOverfeedFactor(rx_overfeed_factor_);
        bravo_->setRxOverfeedFactor(rx_overfeed_factor_);
        alpha_->setDecodeDelayMs(decode_delay_ms_);
        bravo_->setDecodeDelayMs(decode_delay_ms_);
        alpha_->setRxBatchCallbacks(rx_batch_callbacks_);
        bravo_->setRxBatchCallbacks(rx_batch_callbacks_);
        alpha_->setFixedFrameCodewords(fixed_frame_codewords_, cw_count_forced_);
        bravo_->setFixedFrameCodewords(fixed_frame_codewords_, cw_count_forced_);
        alpha_->setCarrierMask(carrier_mask_);
        bravo_->setCarrierMask(carrier_mask_);
        alpha_->setSoftCombiningHARQ(soft_combining_harq_);
        bravo_->setSoftCombiningHARQ(soft_combining_harq_);

        // Set channel SNR for mode negotiation
        alpha_->setSNR(snr_db_);
        bravo_->setSNR(snr_db_);

        // Set forced settings on INITIATOR only (alpha)
        // Responder (bravo) reads these from the CONNECT frame and honors them
        if (forced_mod_ != Modulation::AUTO) {
            alpha_->setForcedModulation(forced_mod_);
        }
        if (forced_rate_ != CodeRate::AUTO) {
            alpha_->setForcedCodeRate(forced_rate_);
        }
        if (forced_waveform_ != WaveformMode::AUTO) {
            alpha_->setPreferredWaveform(forced_waveform_);
        }

        // Apply channel interleaving setting to both stations
        alpha_->setChannelInterleave(use_channel_interleave_);
        bravo_->setChannelInterleave(use_channel_interleave_);
        if (!use_channel_interleave_) {
            std::cout << "  \033[33mChannel interleaving DISABLED\033[0m\n";
        }

        // Apply burst interleave setting to both stations
        alpha_->setNoBurstInterleave(no_burst_interleave_);
        bravo_->setNoBurstInterleave(no_burst_interleave_);
        alpha_->setBurstInterleaveGroupSize(burst_group_size_);
        bravo_->setBurstInterleaveGroupSize(burst_group_size_);
        if (no_burst_interleave_) {
            std::cout << "  \033[33mBurst interleaving DISABLED\033[0m\n";
        }
        if (burst_group_size_ != 8) {
            std::cout << "  \033[36mBurst interleave group size = " << burst_group_size_ << "\033[0m\n";
        }
        if (fixed_frame_codewords_ != v2::kDefaultFixedFrameCodewords) {
            std::cout << "  \033[36mFixed frame CW count = " << fixed_frame_codewords_ << "\033[0m\n";
        }
        if (carrier_mask_ != UINT64_MAX) {
            std::cout << "  \033[36mCarrier mask = 0x" << std::hex << carrier_mask_
                      << std::dec << "\033[0m\n";
        }
        if (soft_combining_harq_) {
            std::cout << "  \033[36mRX soft-combining HARQ ENABLED\033[0m\n";
        }

        // Setup message callback on BRAVO
        bravo_->setMessageCallback([this](const std::string& msg) {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            received_message_ = msg;
            message_received_ = true;
            received_messages_.push_back(msg);
            int count = static_cast<int>(received_messages_.size());
            messages_received_count_.store(count);

            if (save_signals_ &&
                save_signals_message_limit_ > 0 &&
                count >= save_signals_message_limit_ &&
                !capture_limit_hit_.exchange(true)) {
                LOG_MODEM(INFO, "[capture] reached message limit (%d), capture will finish at teardown",
                          save_signals_message_limit_);
            }
        });

        // Setup file received callback on BRAVO
        bravo_->setReceiveDirectory("/tmp");
        bravo_->setFileReceivedCallback([this](const std::string& path, bool success) {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            received_file_path_ = path;
            file_transfer_success_ = success;
            file_received_ = true;
        });

        // Start audio threads
        alpha_->start();
        bravo_->start();

        // Run protocol test (message, file, or burst)
        bool success;
        if (adaptive_test_) {
            success = runAdaptiveTest();
        } else if (test_burst_) {
            success = runBurstTest();
        } else if (test_file_transfer_) {
            success = runFileTransferTest();
        } else {
            success = runProtocolTest();
        }

        // Stop
        alpha_->stop();
        bravo_->stop();
        if (ota_capture_active_) {
            std::string error;
            std::string stopped_path;
            if (stopOtaCapture(active_ota_grpc_target_, ota_alpha_token_,
                               ota_session_id_, &stopped_path, &error)) {
                ota_capture_active_ = false;
                if (!stopped_path.empty()) {
                    ota_capture_path_ = stopped_path;
                }
            } else {
                std::cout << "  \033[33m[capture] warning: " << error << "\033[0m\n";
            }
        }
        if (save_signals_) {
            saveCapturedSignals(success);
        }
        if (spawned_local) {
            std::string error;
            if (!local_server.terminateCleanly(&error)) {
                std::cout << "  \033[31m✗ OTASim shutdown failed: "
                          << error << "\033[0m\n";
                if (!local_server.logPath().empty()) {
                    std::cout << "  OTASim log: " << local_server.logPath().string() << "\n";
                }
                success = false;
            }
        }

        if (success) {
            printSummary();
        } else {
            std::cout << "\n";
            std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
            std::cout << "║                     TEST FAILED                              ║\n";
            std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
            printStationStats("ALPHA (TX)", alpha_.get());
            printStationStats("BRAVO (RX)", bravo_.get());
            printDecoderPhaseBreakdown();
            std::cout << "\n";
        }
        return success;
    }

private:
    float snr_db_ = 20.0f;
    bool verbose_ = false;
    bool use_fading_ = false;
    ChannelType channel_type_ = ChannelType::AWGN;
    bool test_file_transfer_ = false;
    bool test_burst_ = false;              // --burst-test mode: send large messages for burst interleaving
    bool use_channel_interleave_ = true;   // Enabled by default for OFDM fading resistance
    bool no_burst_interleave_ = false;     // --no-burst-interleave for A/B testing
    int burst_group_size_ = 8;             // --burst-group-size N (experimental)
    int rx_overfeed_factor_ = 1;           // --rx-overfeed-factor N (decoder overload stress)
    int decode_delay_ms_ = 0;              // --decode-delay-ms N (simulated slow decoder)
    int rx_batch_callbacks_ = 1;           // --rx-batch-callbacks N (batched decoder feed)
    int fixed_frame_codewords_ = v2::kDefaultFixedFrameCodewords;
    bool cw_count_forced_ = false;  // true iff --cw-count was passed
    uint64_t carrier_mask_ = UINT64_MAX;
    bool soft_combining_harq_ = false;
    bool save_signals_ = false;
    int save_signals_message_limit_ = 0;   // 0 = full run
    size_t save_signals_max_samples_ = 0;  // 0 = unlimited
    std::string save_signals_prefix_ = "/tmp/cli_signals";
    std::atomic<bool> capture_limit_hit_{false};
    bool adaptive_test_ = false;
    float adaptive_hop_snr_db_ = 12.0f;
    ChannelType adaptive_hop_channel_ = ChannelType::MODERATE;
    size_t test_file_size_ = 256;  // Default 256 bytes test file
    uint32_t seed_ = 42;
    float tx_cfo_hz_ = 0.0f;
    Modulation forced_mod_ = Modulation::AUTO;
    CodeRate forced_rate_ = CodeRate::AUTO;
    WaveformMode forced_waveform_ = WaveformMode::AUTO;
    OFDMConfigPreset ofdm_config_preset_ = OFDMConfigPreset::Default;
    std::string mc_dpsk_preset_name_ = "adaptive";
    MultiCarrierDPSKConfig mc_dpsk_config_ = mc_dpsk_presets::robust_mid();

    // Hardware-audio role (--role A|B|both, default both = current sim behavior)
    enum class Role { Both, A, B };
    Role role_ = Role::Both;
    std::string self_callsign_;        // empty -> default per role
    std::string peer_callsign_;        // empty -> default per role (only used by role A)
    std::string audio_output_device_;  // empty -> SDL default device
    std::string audio_input_device_;   // empty -> SDL default device
    bool list_audio_devices_ = false;
    int role_b_idle_seconds_ = 0;      // 0 = run until peer disconnects (no idle cap)
    bool inject_channel_ = false;       // --inject-channel: apply TX-side channel sim
                                        // to real-audio output (uses snr_db_/channel_type_)
    float inject_gain_ = 0.70f;         // Post-injection headroom before DAC full scale
    int audio_buffer_size_ = 0;         // 0 = AudioEngine default (4096)
    bool verify_snr_ = false;
    std::string ota_host_;
    std::string ota_session_id_ = kOtaDefaultSession;
    std::string ota_alpha_token_ = kOtaAlphaToken;
    std::string ota_bravo_token_ = kOtaBravoToken;
    std::string ota_server_binary_;
    std::string active_ota_grpc_target_;
    bool ota_capture_active_ = false;
    std::string ota_capture_path_;

    std::unique_ptr<SimulatedStation> alpha_;
    std::unique_ptr<SimulatedStation> bravo_;

    std::mutex msg_mutex_;
    std::string received_message_;
    std::atomic<bool> message_received_{false};
    std::vector<std::string> received_messages_;  // For batch receive
    std::atomic<int> messages_received_count_{0};

    // File transfer state
    std::string received_file_path_;
    bool file_transfer_success_ = false;
    std::atomic<bool> file_received_{false};

    static bool writeF32File(const std::string& path, const std::vector<float>& data) {
        std::ofstream out(path, std::ios::binary);
        if (!out) return false;
        out.write(reinterpret_cast<const char*>(data.data()),
                  static_cast<std::streamsize>(data.size() * sizeof(float)));
        return static_cast<bool>(out);
    }

    std::string capturePrefixForRun() const {
        std::ostringstream oss;
        oss << save_signals_prefix_ << "_seed" << seed_;
        return oss.str();
    }

    void saveCapturedSignals(bool test_success) {
        (void)test_success;
        if (!ota_capture_path_.empty()) {
            std::cout << "\n  [capture] OTASim capture path: "
                      << ota_capture_path_ << "\n";
        }
    }

    bool runSNRVerification() const {
        ChannelSNRProbeConfig cfg;
        cfg.snr_db = snr_db_;
        cfg.channel_type = channel_type_;
        cfg.seed = seed_;
        cfg.tx_cfo_hz = tx_cfo_hz_;

        const ChannelSNRProbeResult r = runChannelSNRProbe(cfg);
        std::cout << std::fixed << std::setprecision(2)
                  << "SNR verify: channel=" << channelSNRProbeName(channel_type_)
                  << " configured=" << r.configured_snr_db
                  << " measured=" << r.measured_snr_db
                  << " delta=" << r.delta_db
                  << " signal=" << r.measured_signal_rms
                  << " noise=" << r.measured_noise_rms
                  << std::defaultfloat << std::endl;

        if (!std::isfinite(r.measured_snr_db) || std::abs(r.delta_db) > 2.0f) {
            std::cerr << "SNR verification failed: configured="
                      << r.configured_snr_db
                      << " measured=" << r.measured_snr_db
                      << " delta=" << r.delta_db
                      << " exceeds +/-2 dB\n";
            return false;
        }
        return true;
    }

    bool sendAndVerifyMessage(const std::string& msg, int timeout_seconds = 90) {
        if (!waitFor([this]{ return alpha_->isReadyToSend(); }, 30)) {
            std::cout << "  \033[31m✗ ARQ not ready!\033[0m\n";
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            received_message_.clear();
        }
        message_received_.store(false);

        std::cout << "  TX (" << msg.size() << " bytes): " << msg << "\n";
        alpha_->sendMessage(msg);

        if (!waitFor([this]{ return message_received_.load(); }, timeout_seconds)) {
            std::cout << "  \033[31m✗ Message receive timeout!\033[0m\n";
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            if (received_message_ != msg) {
                std::cout << "  \033[31m✗ Message mismatch!\033[0m\n";
                return false;
            }
        }

        std::cout << "  \033[32m✓ Message delivered\033[0m\n";
        return true;
    }

    bool runAdaptiveTest() {
        std::cout << "\n=== PHASE 1: CONNECTION ===\n";
        std::cout << "  ALPHA connecting to BRAVO...\n";
        alpha_->connect("BRAVO");

        if (!waitFor([this]{ return alpha_->isConnected() && bravo_->isConnected(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Connection timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Both stations connected!\033[0m\n";

        std::cout << "\n=== PHASE 2: HANDSHAKE ===\n";
        if (!waitFor([this]{ return alpha_->isHandshakeComplete(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Handshake timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Handshake complete!\033[0m\n";
        printNegotiatedProfile();

        std::cout << "\n=== PHASE 3: ADAPTIVE SMOKE (2 conditions) ===\n";
        std::cout << "  Condition A: SNR=" << snr_db_ << " dB, channel=" << channelTypeName() << "\n";
        std::string msg_a = "[ADPT_TEST] Phase A baseline ";
        msg_a += std::string(900, 'A');  // Force fragmentation for richer advisory samples
        if (!sendAndVerifyMessage(msg_a)) {
            return false;
        }

        std::cout << "  Switching to Condition B: SNR=" << adaptive_hop_snr_db_
                  << " dB, channel=" << channelTypeToString(adaptive_hop_channel_) << "\n";
        std::string error;
        if (!setOtaChannel(active_ota_grpc_target_, ota_alpha_token_,
                           ota_session_id_, adaptive_hop_channel_,
                           adaptive_hop_snr_db_, seed_, &error)) {
            std::cout << "  \033[31m✗ OTASim channel update failed: "
                      << error << "\033[0m\n";
            return false;
        }
        alpha_->setSNR(adaptive_hop_snr_db_);
        bravo_->setSNR(adaptive_hop_snr_db_);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::string msg_b = "[ADPT_TEST] Phase B changed condition ";
        msg_b += std::string(900, 'B');  // Force fragmentation under changed channel
        if (!sendAndVerifyMessage(msg_b)) {
            return false;
        }

        std::cout << "  \033[32m✓ Adaptive smoke sequence complete\033[0m\n";

        std::cout << "\n=== PHASE 4: DISCONNECT ===\n";
        alpha_->disconnect();
        if (!waitFor([this]{ return !alpha_->isConnected() && !bravo_->isConnected(); }, 15)) {
            std::cout << "  \033[33m! Disconnect timeout (non-fatal)\033[0m\n";
        } else {
            std::cout << "  \033[32m✓ Disconnected!\033[0m\n";
        }

        return true;
    }

    bool runProtocolTest() {
        // Phase 1: Connect
        std::cout << "\n=== PHASE 1: CONNECTION ===\n";
        std::cout << "  ALPHA connecting to BRAVO...\n";
        alpha_->connect("BRAVO");

        if (!waitFor([this]{ return alpha_->isConnected() && bravo_->isConnected(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Connection timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Both stations connected!\033[0m\n";

        // Phase 2: Mode negotiation
        std::cout << "\n=== PHASE 2: MODE NEGOTIATION ===\n";
        if (!waitFor([this]{ return alpha_->isHandshakeComplete(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Mode negotiation timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Handshake complete!\033[0m\n";
        printNegotiatedProfile();

        // Phase 3: Send 5 short + 2 long messages as a burst
        std::cout << "\n=== PHASE 3: DATA TRANSFER (7 messages) ===\n";

        std::vector<std::string> test_messages;
        for (int i = 1; i <= 5; i++) {
            test_messages.push_back("Message " + std::to_string(i) + " from ALPHA");
        }
        // Long messages that exceed single-frame capacity (61 bytes at R1/4)
        test_messages.push_back(
            "This is a long test message that exceeds the 61-byte frame capacity "
            "and must be fragmented across multiple OFDM frames for delivery.");
        test_messages.push_back(
            "CQ CQ CQ de ALPHA. Testing long message fragmentation over HF radio. "
            "The quick brown fox jumps over the lazy dog. 73 de ALPHA.");

        int total = static_cast<int>(test_messages.size());

        if (!waitFor([this]{ return alpha_->isReadyToSend(); }, 30)) {
            std::cout << "  \033[31m✗ ARQ not ready!\033[0m\n";
            return false;
        }

        // Clear received state
        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            received_messages_.clear();
            messages_received_count_.store(0);
        }

        // Batch-send all messages (burst-interleaved)
        for (int i = 0; i < total; i++) {
            std::cout << "  [" << (i+1) << "/" << total << "] Queuing (" << test_messages[i].size() << "b): \"" << test_messages[i] << "\"\n";
        }
        ultra::timing::globalDecoderProfile().reset();
        alpha_->sendMessages(test_messages);
        std::cout << "  Sent " << total << " messages as burst\n";

        // Wait for all messages to arrive
        // Narrowband needs much longer: ~4.4s/frame RTT with window=1, plus retransmissions
        int burst_timeout = isRobustMCDPSKPreset()
            ? 360
            : ((forced_waveform_ == WaveformMode::OFDM_NARROW) ? 300 : 120);
        if (!waitFor([this, total]{ return messages_received_count_.load() >= total; }, burst_timeout)) {
            int got = messages_received_count_.load();
            std::cout << "  \033[31m✗ Only received " << got << "/" << total << " messages!\033[0m\n";
            return false;
        }

        // Verify all messages
        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            bool all_ok = true;
            for (int i = 0; i < total; i++) {
                const auto idx = static_cast<size_t>(i);
                if (idx < received_messages_.size() && received_messages_[idx] == test_messages[idx]) {
                    std::cout << "  \033[32m✓ [" << (i+1) << "/" << total << "] Received (" << received_messages_[idx].size() << "b): \"" << received_messages_[idx] << "\"\033[0m\n";
                } else {
                    std::string got = (idx < received_messages_.size()) ? received_messages_[idx] : "(missing)";
                    std::cout << "  \033[31m✗ Message " << (i+1) << " mismatch! Got: \"" << got << "\"\033[0m\n";
                    all_ok = false;
                }
            }
            if (!all_ok) return false;
        }

        std::cout << "  \033[32m✓ All " << total << " messages transferred successfully!\033[0m\n";

        // Phase 4: Disconnect (non-fatal if timeout - data transfer already proved)
        std::cout << "\n=== PHASE 4: DISCONNECT ===\n";
        alpha_->disconnect();

        if (!waitFor([this]{ return !alpha_->isConnected() && !bravo_->isConnected(); }, 15)) {
            std::cout << "  \033[33m! Disconnect timeout (non-fatal)\033[0m\n";
        } else {
            std::cout << "  \033[32m✓ Disconnected!\033[0m\n";
        }

        return true;  // Data transfer succeeded, disconnect is best-effort
    }

    bool runFileTransferTest() {
        // Create test file
        std::string test_file = "/tmp/cli_sim_test_file_" + std::to_string(::getpid()) + ".bin";
        {
            std::ofstream ofs(test_file, std::ios::binary);
            if (!ofs) {
                std::cout << "  \033[31m✗ Failed to create test file!\033[0m\n";
                return false;
            }
            // Write test pattern
            for (size_t i = 0; i < test_file_size_; i++) {
                ofs.put(static_cast<char>(i & 0xFF));
            }
        }
        std::cout << "  Created test file: " << test_file << " (" << test_file_size_ << " bytes)\n";

        // Phase 1: Connect
        std::cout << "\n=== PHASE 1: CONNECTION ===\n";
        std::cout << "  ALPHA connecting to BRAVO...\n";
        alpha_->connect("BRAVO");

        if (!waitFor([this]{ return alpha_->isConnected() && bravo_->isConnected(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Connection timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Both stations connected!\033[0m\n";

        // Phase 2: Mode negotiation
        std::cout << "\n=== PHASE 2: MODE NEGOTIATION ===\n";
        if (!waitFor([this]{ return alpha_->isHandshakeComplete(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Mode negotiation timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Handshake complete!\033[0m\n";
        printNegotiatedProfile();

        // Phase 3: File transfer
        std::cout << "\n=== PHASE 3: FILE TRANSFER ===\n";

        if (!waitFor([this]{ return alpha_->isReadyToSend(); }, 10)) {
            std::cout << "  \033[31m✗ ARQ not ready!\033[0m\n";
            return false;
        }

        file_received_.store(false);
        ultra::timing::globalDecoderProfile().reset();
        std::cout << "  Sending file: " << test_file << " (" << test_file_size_ << " bytes)\n";

        if (!alpha_->sendFile(test_file)) {
            std::cout << "  \033[31m✗ Failed to start file transfer!\033[0m\n";
            return false;
        }

        // Wait for file transfer with progress updates
        auto start = std::chrono::steady_clock::now();
        int last_progress = -1;
        while (!file_received_.load()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

            // Channel-aware file-transfer budget; see fileTransferTimeoutSeconds().
            const long timeout_s = fileTransferTimeoutSeconds(test_file_size_, false);
            if (elapsed >= timeout_s) {
                std::cout << "  \033[31m✗ File transfer timeout (budget=" << timeout_s
                          << "s, channel=" << channelTypeName() << ")!\033[0m\n";
                return false;
            }

            alpha_->tick();
            bravo_->tick();

            // Show progress
            auto progress = alpha_->getFileProgress();
            int pct = static_cast<int>(progress.percentage());
            if (pct != last_progress && pct % 10 == 0) {
                std::cout << "  Progress: " << pct << "% (" << progress.transferred_bytes << "/" << progress.total_bytes << " bytes)\n";
                last_progress = pct;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Measure transfer time (from sendFile to file_received)
        auto transfer_end = std::chrono::steady_clock::now();
        float transfer_sec = std::chrono::duration<float>(transfer_end - start).count();
        float throughput_bps = (transfer_sec > 0.01f)
            ? (test_file_size_ * 8.0f / transfer_sec) : 0.0f;

        // Verify received file
        {
            std::lock_guard<std::mutex> lock(msg_mutex_);
            if (!file_transfer_success_) {
                std::cout << "  \033[31m✗ File transfer reported failure!\033[0m\n";
                return false;
            }
            std::cout << "  \033[32m✓ File received: " << received_file_path_ << "\033[0m\n";
            std::cout << "  Transfer: " << test_file_size_ << " bytes in "
                      << std::fixed << std::setprecision(1) << transfer_sec << "s = "
                      << std::setprecision(0) << throughput_bps << " bps\n";

            // Verify contents
            std::ifstream ifs(received_file_path_, std::ios::binary);
            if (!ifs) {
                std::cout << "  \033[31m✗ Cannot open received file!\033[0m\n";
                return false;
            }

            bool content_ok = true;
            for (size_t i = 0; i < test_file_size_; i++) {
                char c;
                if (!ifs.get(c) || static_cast<uint8_t>(c) != (i & 0xFF)) {
                    content_ok = false;
                    break;
                }
            }

            if (content_ok) {
                std::cout << "  \033[32m✓ File contents verified!\033[0m\n";
            } else {
                std::cout << "  \033[31m✗ File contents corrupted!\033[0m\n";
                return false;
            }
        }

        if (!waitFor([this]{ return !alpha_->isFileTransferInProgress() && alpha_->isReadyToSend(); }, 60)) {
            std::cout << "  \033[33m! Sender ACK drain timeout before disconnect (non-fatal)\033[0m\n";
        }

        // Phase 4: Disconnect (non-fatal if timeout - file transfer already proved)
        std::cout << "\n=== PHASE 4: DISCONNECT ===\n";
        alpha_->disconnect();

        if (!waitFor([this]{ return !alpha_->isConnected() && !bravo_->isConnected(); }, 15)) {
            std::cout << "  \033[33m! Disconnect timeout (non-fatal)\033[0m\n";
        } else {
            std::cout << "  \033[32m✓ Disconnected!\033[0m\n";
        }

        // Cleanup
        std::remove(test_file.c_str());
        std::remove(received_file_path_.c_str());

        return true;
    }

    bool runBurstTest() {
        // Phase 1: Connect
        std::cout << "\n=== PHASE 1: CONNECTION ===\n";
        std::cout << "  ALPHA connecting to BRAVO...\n";
        alpha_->connect("BRAVO");

        if (!waitFor([this]{ return alpha_->isConnected() && bravo_->isConnected(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Connection timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Both stations connected!\033[0m\n";

        // Phase 2: Mode negotiation
        std::cout << "\n=== PHASE 2: MODE NEGOTIATION ===\n";
        if (!waitFor([this]{ return alpha_->isHandshakeComplete(); }, mcDpskHandshakeTimeoutSeconds())) {
            std::cout << "  \033[31m✗ Mode negotiation timeout!\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Handshake complete!\033[0m\n";
        printNegotiatedProfile();

        // Phase 3: Send 3 large messages that fragment into 5+ frames each
        // At R1/2: payload capacity = 141 bytes, so 600 bytes → ceil(600/141) = 5 frames
        // At R1/4: payload capacity = 61 bytes, so 600 bytes → ceil(600/61) = 10 frames
        // With N-frame grouping: at least one burst-interleaved group per large message
        std::cout << "\n=== PHASE 3: BURST DATA TRANSFER (3 large messages) ===\n";
        std::cout << "  Burst interleaving: " << (no_burst_interleave_ ? "DISABLED" : "ENABLED") << "\n";
        std::cout << "  Burst group size: " << burst_group_size_ << "\n";

        std::vector<std::string> test_messages;
        // Generate 3 large messages (~600 bytes each)
        for (int i = 0; i < 3; i++) {
            std::string msg;
            msg.reserve(600);
            for (int j = 0; j < 60; j++) {
                char buf[16];
                snprintf(buf, sizeof(buf), "BLK%d_%02d ", i + 1, j);
                msg += buf;
            }
            // Trim to exactly 600 bytes
            msg.resize(600, 'X');
            test_messages.push_back(msg);
        }

        int total = static_cast<int>(test_messages.size());
        for (int msg_num = 0; msg_num < total; msg_num++) {
            const std::string& test_msg = test_messages[msg_num];

            if (!waitFor([this]{ return alpha_->isReadyToSend(); }, 60)) {
                std::cout << "  \033[31m✗ ARQ not ready for message " << (msg_num+1) << "!\033[0m\n";
                return false;
            }

            message_received_.store(false);

            std::cout << "  [" << (msg_num+1) << "/" << total << "] Sending (" << test_msg.size() << " bytes)...\n";
            alpha_->sendMessage(test_msg);

            if (!waitFor([this]{ return message_received_.load(); }, 120)) {
                std::cout << "  \033[31m✗ Message " << (msg_num+1) << " not received (timeout)!\033[0m\n";
                return false;
            }

            {
                std::lock_guard<std::mutex> lock(msg_mutex_);
                if (received_message_ == test_msg) {
                    std::cout << "  \033[32m✓ [" << (msg_num+1) << "/" << total << "] Received (" << received_message_.size() << " bytes) OK\033[0m\n";
                } else {
                    std::cout << "  \033[31m✗ Message " << (msg_num+1) << " corrupted!\033[0m\n";
                    return false;
                }
            }
        }

        // Print decoder stats
        auto stats = bravo_->getDecoderStats();
        std::cout << "\n=== RESULTS ===\n";
        std::cout << "  Frames decoded: " << stats.frames_decoded << "\n";
        std::cout << "  Frames failed:  " << stats.frames_failed << "\n";
        if (stats.frames_decoded + stats.frames_failed > 0) {
            float success_rate = 100.0f * stats.frames_decoded / (stats.frames_decoded + stats.frames_failed);
            std::cout << "  Success rate:   " << std::fixed << std::setprecision(1) << success_rate << "%\n";
        }
        std::cout << "  \033[32m✓ All " << total << " large messages transferred successfully!\033[0m\n";

        // Phase 4: Disconnect
        std::cout << "\n=== PHASE 4: DISCONNECT ===\n";
        alpha_->disconnect();
        if (!waitFor([this]{ return !alpha_->isConnected() && !bravo_->isConnected(); }, 15)) {
            std::cout << "  \033[33m! Disconnect timeout (non-fatal)\033[0m\n";
        } else {
            std::cout << "  \033[32m✓ Disconnected!\033[0m\n";
        }

        return true;
    }

    bool waitFor(std::function<bool()> condition, int timeout_seconds) {
        auto start = std::chrono::steady_clock::now();
        int last_print = -1;

        while (!condition()) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

            if (elapsed >= timeout_seconds) {
                return false;
            }

            // Tick protocols
            alpha_->tick();
            bravo_->tick();

            // Progress indicator
            if (elapsed != last_print && elapsed % 2 == 0) {
                std::cout << "  [" << alpha_->getSimTime() << "s sim]\n";
                last_print = elapsed;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return true;
    }

    int mcDpskHandshakeTimeoutSeconds() const {
        return (mc_dpsk_config_.samples_per_symbol > 512 ||
                mc_dpsk_config_.bits_per_symbol < 2) ? 120 : 30;
    }

    bool isRobustMCDPSKPreset() const {
        return mc_dpsk_config_.samples_per_symbol > 512 ||
               mc_dpsk_config_.bits_per_symbol < 2;
    }

    const char* channelTypeName() const {
        switch (channel_type_) {
            case ChannelType::PASSTHROUGH: return "Passthrough";
            case ChannelType::AWGN:     return "AWGN (no fading)";
            case ChannelType::GOOD:     return "Good (0.5ms, 0.1Hz)";
            case ChannelType::MODERATE: return "Moderate (1ms, 0.5Hz)";
            case ChannelType::POOR:     return "Poor (2ms, 1Hz)";
            case ChannelType::FLUTTER:  return "Flutter (0.5ms, 10Hz)";
            default:                    return "Unknown";
        }
    }

    // Channel-aware file-transfer timeout. Sizes for worst-case sustained
    // throughput at R1/4 OFDM (the slowest rate), then adds base-overhead
    // seconds for handshake + disconnect. Values are floors observed
    // empirically — measured numbers are typically 1.5-3x faster.
    //   AWGN:     ~60 B/s in-process sim, ~30 B/s on real-audio hardware
    //   Good:     ~20 B/s sim, ~12 B/s hardware
    //   Moderate: ~12 B/s sim, ~8 B/s hardware
    //   Poor/Flutter: ~8 B/s sim, ~6 B/s hardware
    // Robust DBPSK MC-DPSK is much slower: use 1-2 B/s floors so hardware
    // validation budgets reflect long DBPSK frame airtime.
    // Hardware mode is slower: soundcard jitter, ACK turnaround latency,
    // and ~5-15% retx overhead from USB-1.1 audio devices. Hardware also
    // gets 90s base (vs 60s sim) for two-machine handshake setup time.
    long fileTransferTimeoutSeconds(size_t bytes, bool hardware_mode = false) const {
        if (isRobustMCDPSKPreset()) {
            const long bytes_per_second_floor = hardware_mode ? 1 : 2;
            const long base_overhead_s = hardware_mode ? 180 : 120;
            return base_overhead_s + static_cast<long>(bytes) / bytes_per_second_floor;
        }

        long bps_floor;
        if (hardware_mode) {
            switch (channel_type_) {
                case ChannelType::AWGN:     bps_floor = 30; break;
                case ChannelType::GOOD:     bps_floor = 12; break;
                case ChannelType::MODERATE: bps_floor = 8;  break;
                case ChannelType::POOR:     // fall through
                case ChannelType::FLUTTER:  bps_floor = 6;  break;
                default:                    bps_floor = 30; break;
            }
        } else {
            switch (channel_type_) {
                case ChannelType::AWGN:     bps_floor = 60; break;
                case ChannelType::GOOD:     bps_floor = 20; break;
                case ChannelType::MODERATE: bps_floor = 12; break;
                case ChannelType::POOR:     // fall through
                case ChannelType::FLUTTER:  bps_floor = 8;  break;
                default:                    bps_floor = 60; break;
            }
        }
        const long base_overhead_s = hardware_mode ? 90 : 60;
        return base_overhead_s + static_cast<long>(bytes) / bps_floor;
    }

    // ======================================================================
    // Hardware-audio mode (--role A|B): single station per process, real
    // soundcard I/O, peer is on another machine connected by audio cable.
    // ======================================================================

    bool runHardwareTest() {
#ifndef ULTRA_HAVE_SDL2
        std::cerr << "Hardware audio (--role A|B) requires SDL2. "
                     "This build was compiled without SDL2 support.\n";
        return false;
#else
        // Pick callsigns based on role unless overridden on the CLI.
        const std::string self = !self_callsign_.empty()
            ? self_callsign_
            : (role_ == Role::A ? std::string("ALPHA") : std::string("BRAVO"));
        const std::string peer = !peer_callsign_.empty()
            ? peer_callsign_
            : (role_ == Role::A ? std::string("BRAVO") : std::string("ALPHA"));

        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║   CLI Simulator - HARDWARE AUDIO MODE                        ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "  Role:     " << (role_ == Role::A ? "A (initiator)" : "B (responder)") << "\n";
        std::cout << "  Self:     " << self << "\n";
        if (role_ == Role::A) std::cout << "  Peer:     " << peer << "\n";
        std::cout << "  Output:   " << (audio_output_device_.empty() ? "(default)" : audio_output_device_) << "\n";
        std::cout << "  Input:    " << (audio_input_device_.empty() ? "(default)" : audio_input_device_) << "\n";
        std::cout << "  OFDM cfg: " << ofdmConfigPresetToString(ofdm_config_preset_) << "\n";
        std::cout << "  MC-DPSK:  "
                  << mcDpskPresetSummary(mc_dpsk_preset_name_, mc_dpsk_config_) << "\n";

        if (list_audio_devices_) {
            gui::AudioEngine probe;
            if (!probe.initialize()) {
                std::cerr << "Failed to init SDL audio for device listing\n"
                          << "Next step: confirm OS audio permissions and that no other "
                             "process has exclusive control of the sound device.\n";
                return false;
            }
            std::cout << "\n  Output devices:\n";
            for (const auto& d : probe.getOutputDevices()) std::cout << "    " << d << "\n";
            std::cout << "\n  Input devices:\n";
            for (const auto& d : probe.getInputDevices()) std::cout << "    " << d << "\n";
            probe.shutdown();
            return true;
        }

        // Optional TX-side channel injection (CFO + Watterson + AWGN
        // applied to outgoing audio before the soundcard). Each side runs
        // its own injector so both directions get realistic channel.
        std::unique_ptr<ChannelInjector> injector;
        if (inject_channel_) {
            const uint32_t injector_seed = (role_ == Role::A)
                ? seed_                  // ALPHA's TX uses base seed
                : seed_ + 0x9E3779B9u;   // BRAVO's TX uses a decorrelated seed
            injector = std::make_unique<ChannelInjector>(
                snr_db_, channel_type_, injector_seed, tx_cfo_hz_, inject_gain_);
            std::cout << "  Inject:   " << channelTypeName()
                      << " @ " << snr_db_ << " dB SNR (TX-side), gain="
                      << inject_gain_ << "\n";
        }

        // Build the single station with hardware I/O
        auto port = std::make_unique<HardwareAudioPort>(
            audio_output_device_, audio_input_device_, std::move(injector),
            audio_buffer_size_);
        auto station = std::make_unique<SimulatedStation>(
            self, std::move(port), ofdm_config_preset_, mc_dpsk_config_);

        // Forced settings (only meaningful on initiator A — responder B picks
        // them up from the CONNECT frame):
        if (role_ == Role::A) {
            if (forced_mod_ != Modulation::AUTO) station->setForcedModulation(forced_mod_);
            if (forced_rate_ != CodeRate::AUTO) station->setForcedCodeRate(forced_rate_);
            if (forced_waveform_ != WaveformMode::AUTO) station->setPreferredWaveform(forced_waveform_);
        }

        // Pretend SNR for adaptive-mode logic. With real audio this is just
        // a hint to the link-adaptation layer; the real channel is whatever
        // the soundcard cable + --inject-channel produces.
        station->setSNR(snr_db_);
        station->setChannelInterleave(use_channel_interleave_);
        station->setNoBurstInterleave(no_burst_interleave_);
        station->setBurstInterleaveGroupSize(burst_group_size_);
        station->setCarrierMask(carrier_mask_);
        // forced=true only if user passed --cw-count; otherwise this is
        // boot-time init that should leave protocol-level forced_cw_count=0
        // so the responder gets to auto-pick via recommendCWCount(rate).
        station->setFixedFrameCodewords(fixed_frame_codewords_, cw_count_forced_);
        station->setSoftCombiningHARQ(soft_combining_harq_);
        if (soft_combining_harq_) {
            std::cout << "  HARQ:     RX soft-combining enabled\n";
        }

        // Role-B receive callbacks
        std::atomic<bool> peer_connected{false};
        std::atomic<bool> peer_disconnected{false};
        std::atomic<int> rx_message_count{0};
        std::atomic<bool> rx_file_done{false};

        if (role_ == Role::B) {
            station->setReceiveDirectory("/tmp");
            station->setMessageCallback([&](const std::string& msg) {
                int n = rx_message_count.fetch_add(1) + 1;
                std::cout << "  [RX MSG #" << n << " (" << msg.size() << "b)]: " << msg << "\n";
            });
            station->setFileReceivedCallback([&](const std::string& path, bool ok) {
                std::cout << "  [RX FILE] " << path
                          << (ok ? "  ✓" : "  ✗") << "\n";
                rx_file_done.store(true);
            });
        }

        std::cout << "\n  Starting station...\n";
        station->start();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let audio open

        bool ok = false;
        if (role_ == Role::A) {
            ok = runRoleA_initiator(*station, peer);
        } else {
            ok = runRoleB_responder(*station, peer_connected, peer_disconnected,
                                    rx_message_count, rx_file_done);
        }

        std::cout << "\n  Stopping station...\n";
        station->stop();

        // Print stats from the local station (peer's stats live in its own log)
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << (ok ? "║                     TEST PASSED                              ║\n"
                         : "║                     TEST FAILED                              ║\n");
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        printStationStats(self.c_str(), station.get());
        printDecoderPhaseBreakdown();

        return ok;
#endif
    }

#ifdef ULTRA_HAVE_SDL2
    // Block while ticking only one station (no peer in this process).
    bool waitForRole(SimulatedStation& s,
                     std::function<bool()> cond,
                     int timeout_seconds,
                     const char* label) {
        auto start = std::chrono::steady_clock::now();
        int last_print = -1;
        while (!cond()) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed >= timeout_seconds) return false;
            s.tick();
            if (elapsed != last_print && elapsed % 5 == 0) {
                std::cout << "  [" << s.getSimTime() << "s] " << label << " ...\n";
                last_print = static_cast<int>(elapsed);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return true;
    }

    bool runRoleA_initiator(SimulatedStation& station, const std::string& peer) {
        // 1. Connect
        std::cout << "\n=== PHASE 1: CONNECT ===\n";
        std::cout << "  Connecting to " << peer << "...\n";
        station.connect(peer);
        const int connect_timeout_s = std::max(60, mcDpskHandshakeTimeoutSeconds());
        if (!waitForRole(station, [&]{ return station.isConnected(); }, connect_timeout_s, "waiting for connect")) {
            std::cout << "  \033[31m✗ Connect timeout (" << connect_timeout_s << "s)\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Connected\033[0m\n";

        // 2. Mode negotiation / handshake
        std::cout << "\n=== PHASE 2: MODE NEGOTIATION ===\n";
        if (!waitForRole(station, [&]{ return station.isHandshakeComplete(); },
                         mcDpskHandshakeTimeoutSeconds(), "handshake")) {
            std::cout << "  \033[31m✗ Handshake timeout\033[0m\n";
            return false;
        }
        std::cout << "  \033[32m✓ Handshake complete\033[0m\n";

        if (!waitForRole(station, [&]{ return station.isReadyToSend(); }, 10, "ARQ ready")) {
            std::cout << "  \033[31m✗ ARQ not ready\033[0m\n";
            return false;
        }

        // 3. Send file or message
        bool data_ok = false;
        if (test_file_transfer_) {
            std::atomic<bool> file_sent_done{false};
            std::atomic<bool> file_sent_success{false};
            std::mutex file_sent_mutex;
            std::string file_sent_error;
            station.setFileSentCallback([&](bool success, const std::string& error) {
                {
                    std::lock_guard<std::mutex> lock(file_sent_mutex);
                    file_sent_error = error;
                }
                file_sent_success.store(success);
                file_sent_done.store(true);
            });

            std::string test_file = "/tmp/cli_sim_role_a_" + std::to_string(::getpid()) + ".bin";
            {
                std::ofstream ofs(test_file, std::ios::binary);
                if (!ofs) { std::cout << "  ✗ create file failed\n"; return false; }
                for (size_t i = 0; i < test_file_size_; i++) ofs.put(static_cast<char>(i & 0xFF));
            }
            std::cout << "\n=== PHASE 3: FILE TRANSFER ===\n";
            std::cout << "  Sending " << test_file << " (" << test_file_size_ << " bytes)\n";
            ultra::timing::globalDecoderProfile().reset();
            if (!station.sendFile(test_file)) {
                std::cout << "  ✗ sendFile failed\n";
                return false;
            }
            // Hardware-mode budget: channel-aware floor + 90s base for
            // soundcard jitter. See fileTransferTimeoutSeconds().
            const long timeout_s = fileTransferTimeoutSeconds(test_file_size_, true);
            std::cout << "  Budget: " << timeout_s << "s (channel=" << channelTypeName() << ")\n";
            data_ok = waitForRole(station,
                [&]{ return file_sent_done.load() || !station.isFileTransferInProgress(); },
                static_cast<int>(timeout_s), "file transfer");
            if (!data_ok) {
                std::cout << "  \033[31m✗ File transfer timeout (budget=" << timeout_s
                          << "s, channel=" << channelTypeName() << ")\033[0m\n";
            } else if (!file_sent_done.load()) {
                std::cout << "  \033[31m✗ File transfer ended without completion callback\033[0m\n";
                data_ok = false;
            } else if (!file_sent_success.load()) {
                std::lock_guard<std::mutex> lock(file_sent_mutex);
                std::cout << "  \033[31m✗ File transfer failed";
                if (!file_sent_error.empty()) {
                    std::cout << ": " << file_sent_error;
                }
                std::cout << "\033[0m\n";
                data_ok = false;
            } else {
                std::cout << "  Transferred " << test_file_size_ << "/" << test_file_size_
                          << " bytes (100%)\n";
                std::cout << "  \033[32m✓ File transfer complete\033[0m\n";
                data_ok = true;
            }
            station.setFileSentCallback({});
        } else {
            std::cout << "\n=== PHASE 3: MESSAGE TEST ===\n";
            const std::string msg = "Hello from station A (hw test, pid=" +
                                    std::to_string(::getpid()) + ")";
            station.sendMessage(msg);
            // Wait for ARQ idle (no pending TX) as a proxy for ack received.
            data_ok = waitForRole(station, [&]{ return station.isReadyToSend(); },
                                  90, "message ack");
            std::cout << (data_ok ? "  \033[32m✓ Message sent\033[0m\n"
                                  : "  \033[31m✗ Message timeout\033[0m\n");
        }

        // 4. Disconnect (best-effort)
        std::cout << "\n=== PHASE 4: DISCONNECT ===\n";
        station.disconnect();
        if (!waitForRole(station, [&]{ return !station.isConnected(); }, 15, "disconnect")) {
            std::cout << "  \033[33m! Disconnect timeout (non-fatal)\033[0m\n";
        } else {
            std::cout << "  \033[32m✓ Disconnected\033[0m\n";
        }
        return data_ok;
    }

    bool runRoleB_responder(SimulatedStation& station,
                            std::atomic<bool>& peer_connected,
                            std::atomic<bool>& peer_disconnected,
                            std::atomic<int>& rx_message_count,
                            std::atomic<bool>& rx_file_done) {
        std::cout << "\n  Listening for incoming connections...\n";
        std::cout << "  (Ctrl-C or peer disconnect to exit";
        if (role_b_idle_seconds_ > 0) std::cout << ", idle cap=" << role_b_idle_seconds_ << "s";
        std::cout << ")\n";

        auto start = std::chrono::steady_clock::now();
        bool was_connected = false;
        int last_print = -1;
        ultra::timing::globalDecoderProfile().reset();

        while (true) {
            station.tick();

            const bool connected = station.isConnected();
            if (connected && !was_connected) {
                std::cout << "  \033[32m✓ Peer connected\033[0m\n";
                peer_connected.store(true);
                was_connected = true;
            } else if (!connected && was_connected) {
                std::cout << "  Peer disconnected — exiting\n";
                peer_disconnected.store(true);
                return true;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start).count();

            if (role_b_idle_seconds_ > 0 && elapsed >= role_b_idle_seconds_) {
                std::cout << "  Idle cap reached (" << role_b_idle_seconds_
                          << "s), exiting\n";
                return rx_message_count.load() > 0 || rx_file_done.load();
            }

            if (elapsed != last_print && elapsed % 5 == 0) {
                std::cout << "  [" << station.getSimTime() << "s] connected="
                          << (connected ? "yes" : "no")
                          << "  rx_msgs=" << rx_message_count.load()
                          << "  rx_file=" << (rx_file_done.load() ? "yes" : "no")
                          << "\n";
                last_print = static_cast<int>(elapsed);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
#endif  // ULTRA_HAVE_SDL2

    void printHeader() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║   CLI Simulator - IWaveform + StreamingDecoder               ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "  SNR:     " << snr_db_ << " dB\n";
        std::cout << "  TX CFO:  " << tx_cfo_hz_ << " Hz\n";
        std::cout << "  Channel: " << channelTypeName() << "\n";
        std::cout << "  OFDM cfg: " << ofdmConfigPresetToString(ofdm_config_preset_) << "\n";
        std::cout << "  MC-DPSK:  "
                  << mcDpskPresetSummary(mc_dpsk_preset_name_, mc_dpsk_config_) << "\n";
        if (adaptive_test_) {
            std::cout << "  ADPT:    enabled (hop -> "
                      << channelTypeToString(adaptive_hop_channel_)
                      << " @ " << adaptive_hop_snr_db_ << " dB)\n";
        }
        std::cout << "  Model:   OTASim server (48kHz, 10ms callbacks)\n";
        std::cout << "  Session: " << ota_session_id_ << "\n";
        if (rx_overfeed_factor_ > 1) {
            std::cout << "  Stress:  RX overfeed x" << rx_overfeed_factor_ << "\n";
        }
        if (decode_delay_ms_ > 0) {
            std::cout << "  Stress:  Decode delay " << decode_delay_ms_ << " ms\n";
        }
        if (rx_batch_callbacks_ > 1) {
            std::cout << "  Stress:  RX batch " << rx_batch_callbacks_ << " callbacks/feed\n";
        }
        if (fixed_frame_codewords_ != v2::kDefaultFixedFrameCodewords) {
            std::cout << "  CW/frame: " << fixed_frame_codewords_ << "\n";
        }
        std::cout << "\n";
    }

    void printNegotiatedProfile() const {
        if (!alpha_) return;
        const WaveformMode waveform = alpha_->getNegotiatedWaveform();
        std::cout << "  Negotiated data: waveform=" << waveformModeToString(waveform)
                  << " mod=" << modulationToString(alpha_->getDataModulation())
                  << " rate=" << codeRateToString(alpha_->getDataCodeRate())
                  << " cw=" << alpha_->getFixedFrameCodewords();
        if (waveform == WaveformMode::MC_DPSK) {
            const auto cfg = alpha_->getMCDPSKConfig();
            std::cout << " mc_carriers=" << cfg.num_carriers
                      << " mc_sps=" << cfg.samples_per_symbol
                      << " mc_bits=" << cfg.bits_per_symbol;
        }
        std::cout << "\n";
    }

    void printSummary() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                     TEST PASSED                              ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";

        // Print detailed stats from both stations
        printStationStats("ALPHA (TX)", alpha_.get());
        printStationStats("BRAVO (RX)", bravo_.get());
        printDecoderPhaseBreakdown();

        std::cout << "\n";
    }

    void printDecoderPhaseBreakdown() {
        auto& dp = ultra::timing::globalDecoderProfile();
        auto fmt = [](const ultra::timing::PhaseStats& s) -> std::string {
            const uint64_t cnt = s.count.load();
            const uint64_t tot = s.total_us.load();
            const uint64_t mx  = s.max_us.load();
            if (cnt == 0) return "(0 calls)";
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "n=%llu  total=%.1fms  mean=%.1fus  max=%lluus",
                     static_cast<unsigned long long>(cnt),
                     tot / 1000.0,
                     static_cast<double>(tot) / static_cast<double>(cnt),
                     static_cast<unsigned long long>(mx));
            return std::string(buf);
        };

        std::cout << "\n  --- Decoder phase breakdown (decode thread, this transfer) ---\n";
        std::cout << "  detect_data_sync          " << fmt(dp.detect_data_sync) << "\n";
        std::cout << "  ofdm_process_total        " << fmt(dp.ofdm_process_total) << "\n";
        std::cout << "  lts_channel_estimate      " << fmt(dp.lts_channel_estimate) << "\n";
        std::cout << "  data_symbol_loop          " << fmt(dp.data_symbol_loop)
                  << "  (full per-symbol loop incl. erase/updateQuality)\n";
        std::cout << "  decode_fixed_frame_total  " << fmt(dp.decode_fixed_frame_total) << "\n";
        std::cout << "  ldpc_cw_total             " << fmt(dp.ldpc_cw_total)
                  << "  (subset of decode_fixed_frame_total)\n";
        std::cout << "  single_cw_decode_total    " << fmt(dp.single_cw_decode_total) << "\n";
        std::cout << "  control_first_1cw         " << fmt(dp.control_first_1cw)
                  << "  (subset of single_cw_decode_total - ACK decode path)\n";
        std::cout << "  cw0_peek_1cw              " << fmt(dp.cw0_peek_1cw)
                  << "  (subset of single_cw_decode_total)\n";
        std::cout << "  ofdm_cw0_probe_decode     " << fmt(dp.ofdm_cw0_probe_decode)
                  << "  (codec_->decode probes in decodeFrame)\n";
        std::cout << "  failed_4cw_after_peek     " << fmt(dp.failed_4cw_after_peek)
                  << "  (subset of decode_fixed_frame_total - incl. real-channel failures)\n";
        std::cout << "  low_llr_escalation_skipped count="
                  << dp.low_llr_escalation_skipped.load()
                  << "  (counter only)\n";
        std::cout << "  raw_cw0_probe_skipped     count="
                  << dp.raw_cw0_probe_skipped.load()
                  << "  (gated when known-4-CW data frame)\n";
        std::cout << "  low_llr_1cw_skipped       control_first="
                  << dp.low_llr_1cw_skipped_control_first.load()
                  << "  cw0_peek="
                  << dp.low_llr_1cw_skipped_cw0_peek.load()
                  << "  (LLR pre-screen avoided the ~85ms decode-and-fail)\n";
        // Codex review #17 instrumentation: HARQ requires CW0 to peek-
        // decode for the chase key. Failures here mean the failed-frame
        // LLRs are NOT retained for combining; HARQ silently provides
        // no benefit on first-attempt header damage.
        {
            const auto succ = dp.harq_key_build_success.load();
            const auto fail = dp.harq_key_build_failed.load();
            const auto total = succ + fail;
            std::cout << "  harq_key_build            success=" << succ
                      << "  failed=" << fail;
            if (total > 0) {
                const double pct = 100.0 * static_cast<double>(fail) /
                                   static_cast<double>(total);
                char buf[64];
                snprintf(buf, sizeof(buf), "  (%.1f%% miss rate)", pct);
                std::cout << buf;
            }
            std::cout << "\n";
        }

        auto fmt_hist = [](const ultra::timing::SingleCWHistogram& h) -> std::string {
            char buf[160];
            snprintf(buf, sizeof(buf),
                     "first=%llu  retry1=%llu  retry2=%llu  retry3=%llu  retry4=%llu  exhausted=%llu",
                     static_cast<unsigned long long>(h.first_try.load()),
                     static_cast<unsigned long long>(h.retry[0].load()),
                     static_cast<unsigned long long>(h.retry[1].load()),
                     static_cast<unsigned long long>(h.retry[2].load()),
                     static_cast<unsigned long long>(h.retry[3].load()),
                     static_cast<unsigned long long>(h.exhausted.load()));
            return std::string(buf);
        };
        std::cout << "  robustDecodeSingleCW retry histogram (per call site):\n";
        std::cout << "    control_first  " << fmt_hist(dp.robust_cw_control_first) << "\n";
        std::cout << "    cw0_peek       " << fmt_hist(dp.robust_cw_cw0_peek) << "\n";
        std::cout << "    default        " << fmt_hist(dp.robust_cw_default) << "\n";

        // |LLR|_avg distribution split by decode outcome — for picking the
        // pre-screen threshold from data. Bins of 0.5 width; last bin is 6.0+.
        auto print_llr_dist = [](const char* label,
                                 const ultra::timing::LLRHistogram& h) {
            using LH = ultra::timing::LLRHistogram;
            uint64_t s_total = 0, f_total = 0;
            for (size_t i = 0; i < LH::kBins; ++i) {
                s_total += h.success[i].load();
                f_total += h.fail[i].load();
            }
            std::cout << "    " << label << "  (n=" << (s_total + f_total)
                      << ", success=" << s_total << ", fail=" << f_total << ")\n";
            if (s_total + f_total == 0) return;
            for (size_t i = 0; i < LH::kBins; ++i) {
                const uint64_t s = h.success[i].load();
                const uint64_t f = h.fail[i].load();
                if (s + f == 0) continue;
                char range[32];
                if (i + 1 == LH::kBins) {
                    snprintf(range, sizeof(range), "[%.1f,inf)",
                             i * LH::kBinWidth);
                } else {
                    snprintf(range, sizeof(range), "[%.1f,%.1f)",
                             i * LH::kBinWidth,
                             (i + 1) * LH::kBinWidth);
                }
                const double p_succ = static_cast<double>(s)
                                    / static_cast<double>(s + f);
                char buf[160];
                snprintf(buf, sizeof(buf),
                         "      %-12s success=%-5llu fail=%-5llu  P(ok)=%.2f",
                         range,
                         static_cast<unsigned long long>(s),
                         static_cast<unsigned long long>(f),
                         p_succ);
                std::cout << buf << "\n";
            }
        };
        std::cout << "  |LLR|_avg distribution (1-CW pre-screen tuning):\n";
        print_llr_dist("control_first", dp.llr_dist_control_first);
        print_llr_dist("cw0_peek     ", dp.llr_dist_cw0_peek);
    }

    void printStationStats(const char* label, SimulatedStation* station) {
        if (!station) return;

        auto cs = station->getConnectionStats();
        auto ds = station->getDecoderStats();

        std::cout << "\n  --- " << label << " ---\n";

        // ARQ stats
        std::cout << "  ARQ:  frames_sent=" << cs.arq.frames_sent
                  << "  frames_rcvd=" << cs.arq.frames_received
                  << "  retransmissions=" << cs.arq.retransmissions
                  << "  timeouts=" << cs.arq.timeouts
                  << "  failed=" << cs.arq.failed << "\n";
        std::cout << "  RETX: timeout=" << cs.arq.retransmissions_timeout
                  << "  fast_hole=" << cs.arq.retransmissions_fast_hole
                  << "  hole_probe=" << cs.arq.retransmissions_hole_probe
                  << "  nack=" << cs.arq.retransmissions_nack
                  << "  hole_events=" << cs.arq.hole_events << "\n";
        std::cout << "  ACK:  acks_sent=" << cs.arq.acks_sent
                  << "  acks_rcvd=" << cs.arq.acks_received
                  << "  sacks_sent=" << cs.arq.sacks_sent
                  << "  sacks_rcvd=" << cs.arq.sacks_received << "\n";
        std::cout << "  ACKf: stale_ignored=" << cs.arq.stale_acks_ignored
                  << "  future_ignored=" << cs.arq.future_acks_ignored
                  << "  dup_ignored=" << cs.arq.duplicate_acks_ignored
                  << "  repeat_coalesced=" << cs.arq.ack_repeat_jobs_coalesced
                  << "  repeat_dropped=" << cs.arq.ack_repeat_jobs_dropped << "\n";
        std::cout << "  PCW:  partial_rcvd=" << cs.arq.partial_frames_received
                  << "  partial_done=" << cs.arq.partial_frames_completed
                  << "  partial_crc_fail=" << cs.arq.partial_frame_crc_failed
                  << "  partial_expired=" << cs.arq.partial_frame_expired
                  << "  cw_nack_sent=" << cs.arq.cw_nacks_sent
                  << "  cw_nack_rcvd=" << cs.arq.cw_nacks_received
                  << "  repair_tx=" << cs.arq.data_repairs_sent
                  << "  repair_rx=" << cs.arq.data_repairs_received
                  << "  repair_cw_tx=" << cs.arq.data_repair_cws_sent
                  << "  repair_cw_merge=" << cs.arq.data_repair_cws_merged << "\n";

        // Effective ACK rate: how many ACK frames BRAVO sent per data frame
        // received. Baseline reference today on 50 KB OFDM ~1.1.
        if (cs.arq.frames_received > 0) {
            float ack_ratio = static_cast<float>(cs.arq.acks_sent) /
                              static_cast<float>(cs.arq.frames_received);
            std::cout << "  AckR: acks_sent/frames_received=" << std::fixed
                      << std::setprecision(2) << ack_ratio
                      << std::defaultfloat << "\n";

            // SACK trigger-reason breakdown (Phase 2). Each SACK send bumps
            // exactly one counter, so these four sum to sacks_sent.
            std::cout << "  SACKw: threshold=" << cs.arq.sack_trigger_threshold
                      << "  out_of_order=" << cs.arq.sack_trigger_out_of_order
                      << "  timer=" << cs.arq.sack_trigger_timer
                      << "  out_of_window=" << cs.arq.sack_trigger_out_of_window
                      << "\n";
        }

        // Decoder stats
        std::cout << "  RX:   frames_decoded=" << ds.frames_decoded
                  << "  frames_failed=" << ds.frames_failed
                  << "  pings=" << ds.pings_received
                  << "  overflows=" << ds.buffer_overflows
                  << "  drop_samples=" << ds.overflow_samples_dropped
                  << "  resets=" << ds.overflow_state_resets
                  << "  unsearched=" << ds.current_unsearched_samples
                  << "  backlog_ms=" << std::fixed << std::setprecision(1) << ds.backlog_ms
                  << "  peak_backlog_ms=" << ds.peak_backlog_ms
                  << std::defaultfloat << "\n";
        std::cout << "  SyncR: attempts=" << ds.sync_recovery_attempts
                  << "  success=" << ds.sync_recovery_successes
                  << "  d(+8/-8/+16/-16/+24/-24/+32/-32)="
                  << ds.sync_recovery_delta_p8 << "/"
                  << ds.sync_recovery_delta_m8 << "/"
                  << ds.sync_recovery_delta_p16 << "/"
                  << ds.sync_recovery_delta_m16 << "/"
                  << ds.sync_recovery_delta_p24 << "/"
                  << ds.sync_recovery_delta_m24 << "/"
                  << ds.sync_recovery_delta_p32 << "/"
                  << ds.sync_recovery_delta_m32 << "\n";

        // CW success rate (from log grep is imprecise, this is the real number)
        uint64_t total_frames = ds.frames_decoded + ds.frames_failed;
        if (total_frames > 0) {
            float success_pct = 100.0f * ds.frames_decoded / total_frames;
            std::cout << "  Rate: frame_success=" << std::fixed << std::setprecision(1)
                      << success_pct << "%" << std::defaultfloat << "\n";
        }
    }
};

int main(int argc, char* argv[]) {
    try {
        struct LogFileCloser {
            void operator()(std::FILE* file) const {
                if (file) std::fclose(file);
            }
        };
        std::unique_ptr<std::FILE, LogFileCloser> log_file(nullptr);
        LogLevel log_level = LogLevel::INFO;
        bool log_level_set = false;
        bool log_categories_set = false;
        std::string log_categories;
        std::string log_file_path;
        bool expert_phy = envFlagEnabled("ULTRA_EXPERT_PHY");
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]) == "--expert") {
                expert_phy = true;
                break;
            }
        }

        setOperatorLogProfile();
        CLISimulator sim;
        sim.setOtaSimulatorPath(defaultOtaSimulatorPath(argv[0]));

        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if ((arg == "--snr" || arg == "-s") && i + 1 < argc) {
                sim.setSNR(std::stof(argv[++i]));
            } else if (arg == "--verbose" || arg == "-v") {
                sim.setVerbose(true);
            } else if (arg == "--fading" || arg == "-f") {
                // --fading alone = moderate, --fading <type> = specified type
                if (i + 1 < argc && argv[i + 1][0] != '-') {
                    std::string ftype = argv[++i];
                    auto channel = parseFadingType(ftype);
                    if (!channel) {
                        std::cerr << "Unknown fading type: " << ftype
                                  << " (use none, awgn, good, moderate, poor, flutter)\n";
                        return 1;
                    }
                    sim.setChannelType(*channel);
                } else {
                    sim.setChannelType(ChannelType::MODERATE);  // Default fading = moderate
                }
            } else if (arg == "--channel" || arg == "-c") {
                if (i + 1 < argc) {
                    std::string ch_str = argv[++i];
                    auto channel = cli::parseChannelType(ch_str);
                    if (!channel) {
                        std::cerr << "Unknown channel: " << ch_str
                                  << " (use " << cli::channelChoices() << ")\n";
                        return 1;
                    }
                    sim.setChannelType(*channel);
                }
            } else if (arg == "--hop-channel") {
                if (i + 1 < argc) {
                    std::string ch_str = argv[++i];
                    auto channel = cli::parseChannelType(ch_str);
                    if (!channel) {
                        std::cerr << "Unknown hop channel: " << ch_str
                                  << " (use " << cli::channelChoices() << ")\n";
                        return 1;
                    }
                    sim.setAdaptiveHopChannel(*channel);
                }
            } else if (arg == "--mod" || arg == "-m") {
                if (i + 1 < argc) {
                    std::string mod_str = argv[++i];
                    Modulation mod = Modulation::AUTO;
                    if (!parseForcedModulation(mod_str, expert_phy, mod)) {
                        return 1;
                    }
                    sim.setForcedModulation(mod);
                }
            } else if (arg == "--expert") {
                expert_phy = true;
            } else if (arg == "--rate" || arg == "-r") {
                if (i + 1 < argc) {
                    std::string rate_str = argv[++i];
                    auto rate = cli::parseCodeRate(rate_str, cli::AllowAuto::Yes);
                    if (!rate) {
                        std::cerr << "Unknown code rate: " << rate_str
                                  << " (use " << cli::codeRateChoices(cli::AllowAuto::Yes) << ")\n";
                        return 1;
                    }
                    sim.setForcedCodeRate(*rate);
                }
            } else if (arg == "--cw-count" && i + 1 < argc) {
                int cw_count = std::stoi(argv[++i]);
                if (cw_count < v2::kMinFixedFrameCodewords ||
                    cw_count > v2::kMaxFixedFrameCodewords) {
                    std::cerr << "Invalid --cw-count: " << cw_count
                              << " (use " << v2::kMinFixedFrameCodewords
                              << ".." << v2::kMaxFixedFrameCodewords << ")\n";
                    return 1;
                }
                sim.setFixedFrameCodewords(cw_count);
            } else if (arg == "--carrier-mask" && i + 1 < argc) {
                sim.setCarrierMask(static_cast<uint64_t>(std::stoull(argv[++i], nullptr, 0)));
            } else if (arg == "--mask-clear-carrier" && i + 1 < argc) {
                const int carrier = std::stoi(argv[++i]);
                if (carrier < 0 || carrier >= 59) {
                    std::cerr << "Invalid --mask-clear-carrier: " << carrier
                              << " (use 0..58)\n";
                    return 1;
                }
                sim.setCarrierMask(UINT64_MAX & ~(uint64_t{1} << carrier));
            } else if (arg == "--waveform" || arg == "-w") {
                if (i + 1 < argc) {
                    std::string wf_str = argv[++i];
                    auto waveform = cli::parseWaveformMode(wf_str, cli::BareOFDMMode::Cox);
                    if (!waveform) {
                        std::cerr << "Unknown waveform: " << wf_str
                                  << " (use " << cli::waveformChoices() << ")\n";
                        return 1;
                    }
                    sim.setPreferredWaveform(*waveform);
                }
            } else if (arg == "--ofdm-config") {
                if (i + 1 >= argc) {
                    std::cerr << "Missing value for --ofdm-config (use default or nvis)\n";
                    return 1;
                }
                std::string cfg_str = argv[++i];
                if (cfg_str == "default" || cfg_str == "DEFAULT") {
                    sim.setOFDMConfigPreset(OFDMConfigPreset::Default);
                } else if (cfg_str == "nvis" || cfg_str == "NVIS") {
                    sim.setOFDMConfigPreset(OFDMConfigPreset::Nvis);
                } else {
                    std::cerr << "Unknown OFDM config: " << cfg_str
                              << " (use default or nvis)\n";
                    return 1;
                }
            } else if (arg == "--mc-dpsk-preset") {
                if (i + 1 >= argc) {
                    std::cerr << "Missing value for --mc-dpsk-preset (use "
                              << mcDpskPresetChoices() << ")\n";
                    return 1;
                }
                std::string preset_name;
                MultiCarrierDPSKConfig preset_config;
                const std::string preset_arg = argv[++i];
                if (!parseMCDPSKPreset(preset_arg, preset_name, preset_config)) {
                    std::cerr << "Unknown MC-DPSK preset: " << preset_arg
                              << " (use " << mcDpskPresetChoices() << ")\n";
                    return 1;
                }
                sim.setMCDPSKPreset(preset_name, preset_config);
            } else if (arg == "--file" || arg == "--test-file") {
                sim.setTestFileTransfer(true);
                // Optional file size argument
                if (i + 1 < argc && argv[i + 1][0] != '-') {
                    sim.setTestFileSize(std::stoul(argv[++i]));
                }
            } else if (arg == "--no-channel-interleave" || arg == "--nci") {
                sim.setChannelInterleave(false);
            } else if (arg == "--channel-interleave" || arg == "-ci") {
                sim.setChannelInterleave(true);
            } else if (arg == "--no-burst-interleave" || arg == "--nbi") {
                sim.setNoBurstInterleave(true);
            } else if (arg == "--burst-group-size" && i + 1 < argc) {
                sim.setBurstInterleaveGroupSize(std::stoi(argv[++i]));
            } else if (arg == "--harq") {
                sim.setSoftCombiningHARQ(true);
            } else if (arg == "--rx-overfeed-factor" && i + 1 < argc) {
                sim.setRxOverfeedFactor(std::stoi(argv[++i]));
            } else if (arg == "--decode-delay-ms" && i + 1 < argc) {
                sim.setDecodeDelayMs(std::stoi(argv[++i]));
            } else if (arg == "--rx-batch-callbacks" && i + 1 < argc) {
                sim.setRxBatchCallbacks(std::stoi(argv[++i]));
            } else if (arg == "--burst-test") {
                sim.setTestBurst(true);
            } else if (arg == "--seed" && i + 1 < argc) {
                sim.setSeed(static_cast<uint32_t>(std::stoul(argv[++i])));
            } else if ((arg == "--tx-cfo" || arg == "--cfo") && i + 1 < argc) {
                sim.setTxCFO(std::stof(argv[++i]));
            } else if (arg == "--save-signals") {
                int message_limit = 0;
                if (i + 1 < argc && argv[i + 1][0] != '-') {
                    message_limit = std::stoi(argv[++i]);
                }
                sim.setSaveSignals(true, message_limit);
            } else if (arg == "--adpt-test") {
                sim.setAdaptiveTest(true);
            } else if (arg == "--hop-snr" && i + 1 < argc) {
                sim.setAdaptiveHopSNR(std::stof(argv[++i]));
            } else if (arg == "--save-prefix" && i + 1 < argc) {
                sim.setSaveSignalsPrefix(argv[++i]);
            } else if (arg == "--save-max-samples" && i + 1 < argc) {
                sim.setSaveSignalsMaxSamples(static_cast<size_t>(std::stoull(argv[++i])));
            } else if (arg == "--role" && i + 1 < argc) {
                std::string r = argv[++i];
                if (r == "A" || r == "a")          sim.setRoleA();
                else if (r == "B" || r == "b")     sim.setRoleB();
                else if (r == "both" || r == "BOTH") sim.setRoleBoth();
                else {
                    std::cerr << "Unknown --role: " << r << " (use A, B, or both)\n";
                    return 1;
                }
            } else if (arg == "--peer" && i + 1 < argc) {
                sim.setPeerCallsign(argv[++i]);
            } else if (arg == "--callsign" && i + 1 < argc) {
                sim.setSelfCallsign(argv[++i]);
            } else if (arg == "--audio-output" && i + 1 < argc) {
                sim.setAudioOutputDevice(argv[++i]);
            } else if (arg == "--audio-input" && i + 1 < argc) {
                sim.setAudioInputDevice(argv[++i]);
            } else if (arg == "--list-audio-devices") {
                sim.setListAudioDevices(true);
            } else if (arg == "--idle-seconds" && i + 1 < argc) {
                sim.setRoleBIdleSeconds(std::stoi(argv[++i]));
            } else if (arg == "--inject-channel" || arg == "--inject") {
                sim.setInjectChannel(true);
            } else if (arg == "--verify-snr") {
                sim.setVerifySNR(true);
            } else if (arg == "--inject-gain" && i + 1 < argc) {
                sim.setInjectGain(std::stof(argv[++i]));
            } else if (arg == "--audio-buffer-size" && i + 1 < argc) {
                sim.setAudioBufferSize(std::stoi(argv[++i]));
            } else if (arg == "--ota-host" && i + 1 < argc) {
                sim.setOtaHost(argv[++i]);
            } else if (arg == "--ota-session" && i + 1 < argc) {
                sim.setOtaSessionId(argv[++i]);
            } else if (arg == "--ota-alpha-token" && i + 1 < argc) {
                sim.setOtaAlphaToken(argv[++i]);
            } else if (arg == "--ota-bravo-token" && i + 1 < argc) {
                sim.setOtaBravoToken(argv[++i]);
            } else if (arg == "--ota-server-bin" && i + 1 < argc) {
                sim.setOtaSimulatorPath(argv[++i]);
            } else if (arg == "--log-level" && i + 1 < argc) {
                const std::string value = argv[++i];
                if (!parseLogLevel(value, log_level)) {
                    std::cerr << "Invalid --log-level: " << value
                              << " (use error, warn, info, debug, or trace)\n";
                    return 1;
                }
                log_level_set = true;
            } else if ((arg == "--log-category" || arg == "--log-categories") && i + 1 < argc) {
                log_categories = argv[++i];
                log_categories_set = true;
            } else if (arg == "--log-file" && i + 1 < argc) {
                log_file_path = argv[++i];
            } else if (arg == "--help" || arg == "-h") {
                std::cout << "CLI Simulator - IWaveform + StreamingDecoder Model\n\n";
                std::cout << "Uses IWaveform for TX and StreamingDecoder for RX directly.\n";
                std::cout << "Every 10ms: read RX, feed decoder, get TX, send to channel.\n\n";
                std::cout << "Options:\n";
                std::cout << "  --snr, -s <dB>      SNR (default: 20)\n";
                std::cout << "  --channel, -c <CH>  Channel type: passthrough, awgn, good, moderate, poor, flutter\n";
                std::cout << "                        passthrough - Null OTASim channel\n";
                std::cout << "                        awgn     - No fading, no multipath\n";
                std::cout << "                        good     - 0.5ms delay, 0.1Hz Doppler (quiet)\n";
                std::cout << "                        moderate - 1.0ms delay, 0.5Hz Doppler (typical)\n";
                std::cout << "                        poor     - 2.0ms delay, 1.0Hz Doppler (disturbed)\n";
                std::cout << "                        flutter  - 0.5ms delay, 10Hz Doppler (auroral)\n";
                std::cout << "  --fading, -f        Alias for --channel moderate; none/awgn selects AWGN\n";
                std::cout << "  --mod, -m <MOD>     Force modulation: dqpsk (DBPSK via robust presets)\n";
                std::cout << "  --expert            Allow lab-only forced PHY modes in --mod\n";
                std::cout << "  --rate, -r <RATE>   Force code rate: auto, r1_4, r1_2, r2_3, r3_4\n";
                std::cout << "  --cw-count <N>      Fixed OFDM data-frame codewords (1-8, default: 4)\n";
                std::cout << "  --carrier-mask <M>  OFDM_CHIRP active-carrier mask (default: all-on)\n";
                std::cout << "  --mask-clear-carrier <N>  Clear one carrier bit (0-58)\n";
                std::cout << "  --waveform, -w <WF> Force waveform: mc_dpsk, ofdm_chirp, ofdm_cox, ofdm_narrow\n";
                std::cout << "  --ofdm-config <CFG> OFDM_COX config: default (512/30) or nvis (1024/59)\n";
                std::cout << "  --mc-dpsk-preset <P> Force MC-DPSK preset: standard, robust_low, robust_mid, robust\n";
                std::cout << "                        default: adaptive, with Robust-Mid as cold-call/listen PHY\n";
                std::cout << "  --seed <N>          Random seed (default: 42)\n";
                std::cout << "  --tx-cfo <Hz>       Inject TX CFO in channel model (default: 0)\n";
                std::cout << "  --cfo <Hz>          Alias for --tx-cfo\n";
                std::cout << "  --file [SIZE]       Test file transfer (default: 256 bytes)\n";
                std::cout << "  --adpt-test         Two-message adaptive advisory smoke test\n";
                std::cout << "  --hop-snr <dB>      Condition-B SNR for --adpt-test (default: 12)\n";
                std::cout << "  --hop-channel <CH>  Condition-B channel for --adpt-test\n";
                std::cout << "  --channel-interleave, -ci  Enable channel interleaving\n";
                std::cout << "  --no-burst-interleave     Disable burst-level long interleaving\n";
                std::cout << "  --burst-group-size <N>    Burst interleave group size (2-8, default: 8)\n";
                std::cout << "  --harq                    Enable RX soft-combining HARQ (default: off)\n";
                std::cout << "  --rx-overfeed-factor <N>  Run audio callbacks N× faster wall-clock (stress, default: 1)\n";
                std::cout << "  --decode-delay-ms <N>     Add decode-thread delay (0-500 ms, stress)\n";
                std::cout << "  --rx-batch-callbacks <N>  Batch N callbacks per decoder feed (stress)\n";
                std::cout << "  --burst-test              Send large messages to test burst interleaving\n";
                std::cout << "  --save-signals [N]        Save TX/RX raw float traces (.f32)\n";
                std::cout << "                           N = stop capture after BRAVO receives N app messages\n";
                std::cout << "                           (default: 0 = capture full run)\n";
                std::cout << "  --save-prefix <PATH>      Capture file prefix (default: /tmp/cli_signals)\n";
                std::cout << "  --save-max-samples <N>    Per-stream capture cap (0 = unlimited)\n";
                std::cout << "  --verbose, -v       Verbose output\n";
                std::cout << "  --log-level <error|warn|info|debug|trace>\n";
                std::cout << "                      Console verbosity (default: info)\n";
                std::cout << "  --log-category <list>  Comma list: operator,audio,tnc,modem,demod,sync,ldpc,channel,all\n";
                std::cout << "  --log-file <PATH>      Write logs to file instead of stderr\n";
                std::cout << "  --verify-snr           Probe configured broadband SNR and fail if error exceeds +/-2 dB\n";
                std::cout << "  --ota-host HOST:PORT   Use existing ota_simulator serve instead of self-spawn\n";
                std::cout << "  --ota-session ID       OTASim session id (default: lobby)\n";
                std::cout << "  --ota-alpha-token T    Token for ALPHA (default: alpha_token)\n";
                std::cout << "  --ota-bravo-token T    Token for BRAVO (default: bravo_token)\n";
                std::cout << "  --ota-server-bin PATH  ota_simulator binary for self-spawn\n";
                std::cout << "\nHardware audio mode (real soundcard, two-machine setup):\n";
                std::cout << "  --role A|B|both     A=initiator, B=responder, both=in-process sim (default)\n";
                std::cout << "  --callsign <NAME>   Local callsign (default: ALPHA for A, BRAVO for B)\n";
                std::cout << "  --peer <NAME>       Peer callsign for --role A (default: BRAVO)\n";
                std::cout << "  --audio-output <D>  SDL2 output device name (empty = default)\n";
                std::cout << "  --audio-input <D>   SDL2 input device name (empty = default)\n";
                std::cout << "  --list-audio-devices  List available audio devices and exit\n";
                std::cout << "  --idle-seconds <N>  Role B: max idle seconds before giving up (0 = forever)\n";
                std::cout << "  --inject-channel, --inject\n";
                std::cout << "                      Apply --snr/--channel/--cfo to outgoing audio\n";
                std::cout << "                      (lets Mac-to-Pi cable carry a synthetic HF channel)\n";
                std::cout << "  --inject-gain <G>   Post-injection output gain/headroom (0.05-1.0, default 0.70)\n";
                std::cout << "  --audio-buffer-size <N>  SDL2 period size, samples (default 8192)\n";
                std::cout << "                      Smaller = lower latency. Larger = more XRUN headroom.\n";
                return 0;
            }
        }
        setLogLevel(log_level);
        if (log_level_set && log_level >= LogLevel::DEBUG && !log_categories_set) {
            setDeveloperLogProfile();
        }
        if (log_categories_set && !setLogCategories(log_categories)) {
            std::cerr << "Invalid --log-category list: " << log_categories << "\n";
            return 1;
        }
        if (!log_file_path.empty()) {
            errno = 0;
            log_file.reset(std::fopen(log_file_path.c_str(), "a"));
            if (!log_file) {
                std::cerr << "Failed to open --log-file '" << log_file_path << "'";
                if (errno != 0) {
                    std::cerr << ": " << std::strerror(errno);
                }
                std::cerr << "\nNext step: choose a writable path or fix directory permissions.\n";
                return 1;
            }
            setLogFile(log_file.get());
        }
        return sim.runTest() ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Fatal exception in cli_simulator: " << e.what() << "\n";
        return 2;
    } catch (...) {
        std::cerr << "Fatal unknown exception in cli_simulator\n";
        return 3;
    }
}
