#include "ota_simulator/runner_v2.hpp"

#include "io/wav_io.hpp"
#include "ota_simulator/session_log.hpp"
#include "protocol/frame_v2.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "replay/json_util.hpp"
#include "sim/channel_calibration.hpp"
#include "sim/simulated_station.hpp"
#include "ultra/dsp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace ultra::tools::ota {
namespace {

namespace fs = std::filesystem;
namespace json = ultra::replay::json;
namespace v2 = ultra::protocol::v2;

struct TxFrameRecord {
    double t_s = 0.0;
    std::string frame_type;
    int seq = -1;
};

struct SentFileRecord {
    std::string path;
    std::vector<uint8_t> bytes;
    double t_s = 0.0;
    std::string sender;
};

struct ReceivedFileRecord {
    std::string path;
    bool success = false;
    std::vector<uint8_t> bytes;
    double t_s = 0.0;
};

struct EndpointRuntime {
    std::string name;
    EndpointConfig config;
    std::unique_ptr<SimulatedStation> station;
    std::vector<std::string> received_messages;
    std::vector<SentFileRecord> sent_files;
    std::vector<ReceivedFileRecord> received_files;
    std::vector<TxFrameRecord> tx_frames;
    std::mutex mutex;
    protocol::ConnectionState last_state = protocol::ConnectionState::DISCONNECTED;
};

std::string resolveInputPath(const std::string& scenario_path,
                             const std::string& path) {
    fs::path p(path);
    if (p.is_absolute() || fs::exists(p)) {
        return p.string();
    }
    fs::path base = fs::path(scenario_path).parent_path() / p;
    if (fs::exists(base)) {
        return base.string();
    }
    return path;
}

std::vector<uint8_t> deterministicFilePayload(size_t size_bytes) {
    std::vector<uint8_t> out(size_bytes);
    uint32_t state = 0x9E3779B9u;
    for (size_t i = 0; i < out.size(); ++i) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        out[i] = static_cast<uint8_t>((state >> 24) ^ (i * 37u));
    }
    return out;
}

std::vector<uint8_t> readBinaryFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) {
        throw std::runtime_error("failed to open file payload: " + path);
    }
    const std::streampos end = in.tellg();
    if (end < 0) {
        throw std::runtime_error("failed to size file payload: " + path);
    }
    std::vector<uint8_t> bytes(static_cast<size_t>(end));
    in.seekg(0, std::ios::beg);
    if (!bytes.empty()) {
        in.read(reinterpret_cast<char*>(bytes.data()),
                static_cast<std::streamsize>(bytes.size()));
        if (!in) {
            throw std::runtime_error("failed to read file payload: " + path);
        }
    }
    return bytes;
}

void writeBinaryFile(const std::string& path, const std::vector<uint8_t>& bytes) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("failed to create file payload: " + path);
    }
    if (!bytes.empty()) {
        out.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
        if (!out) {
            throw std::runtime_error("failed to write file payload: " + path);
        }
    }
}

std::string generatedFilePath(const EndpointRuntime& runtime,
                              const ScenarioEvent& event) {
    fs::path dir = fs::temp_directory_path() / "projectultra_ota_sim";
    fs::create_directories(dir);
    std::ostringstream name;
    name << "send_file_" << runtime.name << "_"
         << static_cast<long long>(event.t_s * 1000.0 + 0.5) << "_"
         << runtime.sent_files.size() << ".bin";
    return (dir / name.str()).string();
}

std::string receiveDirectoryPath(const std::string& endpoint) {
    fs::path dir = fs::temp_directory_path() / "projectultra_ota_sim" /
                   "received" / endpoint;
    fs::create_directories(dir);
    return dir.string();
}

float rmsOf(const std::vector<float>& samples) {
    if (samples.empty()) {
        return 0.0f;
    }
    double sum_sq = 0.0;
    for (float sample : samples) {
        sum_sq += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(samples.size())));
}

void scaleToTargetRms(std::vector<float>& samples, double target_rms) {
    if (samples.empty() || target_rms <= 0.0) {
        return;
    }
    const float current = rmsOf(samples);
    if (current <= 0.0f) {
        return;
    }
    const float gain = static_cast<float>(target_rms) / current;
    for (float& sample : samples) {
        sample *= gain;
    }
}

double coefficientEnergy(const FIRFilter& filter) {
    const auto& coeffs = filter.coefficients();
    return std::accumulate(coeffs.begin(), coeffs.end(), 0.0,
                           [](double sum, float h) {
                               return sum + static_cast<double>(h) *
                                            static_cast<double>(h);
                           });
}

std::optional<float> estimateNoiseBedStationSNRDb(const std::vector<float>& scaled_noise) {
    if (scaled_noise.empty()) {
        return std::nullopt;
    }

    FIRFilter filter = FIRFilter::bandpass(/*num_taps=*/101,
                                           /*low_freq=*/50.0f,
                                           /*high_freq=*/2950.0f,
                                           /*sample_rate=*/48000.0f);
    const double fir_energy = coefficientEnergy(filter);
    if (fir_energy <= 0.0 || !std::isfinite(fir_energy)) {
        return std::nullopt;
    }

    double filtered_sum_sq = 0.0;
    for (float sample : scaled_noise) {
        const float filtered = filter.process(sample);
        filtered_sum_sq += static_cast<double>(filtered) * static_cast<double>(filtered);
    }
    const double filtered_power =
        filtered_sum_sq / static_cast<double>(scaled_noise.size());
    const double noise_power = filtered_power / fir_energy;
    if (noise_power <= 0.0 || !std::isfinite(noise_power)) {
        return std::nullopt;
    }

    return static_cast<float>(
        10.0 * std::log10(sim::kModemReferencePower / noise_power));
}

MultiCarrierDPSKConfig mcDpskConfigFor(Modulation modulation) {
    if (modulation == Modulation::DQPSK) {
        return mc_dpsk_presets::level8();
    }
    if (modulation == Modulation::DBPSK) {
        return mc_dpsk_presets::robust_mid();
    }
    throw std::runtime_error(
        "ota_simulator v2 supports only DQPSK or DBPSK MC-DPSK initial_mode");
}

std::string frameTypeFromResult(const gui::DecodeResult& result,
                                const std::string& ping_like_tx_type) {
    if (result.is_ping) {
        return ping_like_tx_type;
    }
    if (!result.frame_data.empty()) {
        auto header = v2::parseHeader(result.frame_data);
        if (header.valid) {
            return v2::frameTypeToString(header.type);
        }
    }
    return result.success ? v2::frameTypeToString(result.frame_type) : "DECODE_FAIL";
}

int frameSeqFromResult(const gui::DecodeResult& result) {
    if (!result.frame_data.empty()) {
        auto header = v2::parseHeader(result.frame_data);
        if (header.valid) {
            return header.seq;
        }
    }
    return -1;
}

class TxMonitor {
public:
    TxMonitor(const InitialMode& mode, std::string ping_like_tx_type)
        : ping_like_tx_type_(std::move(ping_like_tx_type)) {
        decoder_.setLogPrefix("ota_simulator_v2_tx");
        decoder_.setMCDPSKConfig(mcDpskConfigFor(mode.modulation));
        decoder_.setMode(protocol::WaveformMode::MC_DPSK, false);
        decoder_.setDataMode(mode.modulation, mode.code_rate);
    }

    ~TxMonitor() {
        decoder_.stop();
    }

    void feed(const std::string& endpoint,
              const std::vector<float>& samples,
              SessionLog& log,
              std::vector<TxFrameRecord>& frames) {
        if (samples.empty()) {
            return;
        }
        trackActivity(samples);
        decoder_.feedAudio(samples.data(), samples.size());
        samples_fed_ += samples.size();
        decoder_.processBuffer();
        while (decoder_.hasFrame()) {
            auto result = decoder_.getFrame();
            const std::string frame_type = frameTypeFromResult(result, ping_like_tx_type_);
            const int seq = frameSeqFromResult(result);
            uint64_t event_sample = samples_fed_;
            if (!activity_starts_.empty()) {
                event_sample = activity_starts_.front();
                activity_starts_.pop_front();
            }
            const double t_s = static_cast<double>(event_sample) /
                               static_cast<double>(SimulatedStation::SAMPLE_RATE);
            frames.push_back(TxFrameRecord{t_s, frame_type, seq});
            log.writeTxFrame(endpoint, t_s, frame_type, seq, result);
        }
    }

private:
    void trackActivity(const std::vector<float>& samples) {
        constexpr float kActivityRms = 0.006f;
        double sum_sq = 0.0;
        for (float s : samples) {
            sum_sq += static_cast<double>(s) * static_cast<double>(s);
        }
        const float rms = samples.empty()
            ? 0.0f
            : static_cast<float>(std::sqrt(sum_sq / static_cast<double>(samples.size())));
        if (rms >= kActivityRms && !tx_active_) {
            activity_starts_.push_back(samples_fed_);
            tx_active_ = true;
        } else if (rms < kActivityRms) {
            tx_active_ = false;
        }
    }

    gui::StreamingDecoder decoder_;
    std::string ping_like_tx_type_;
    uint64_t samples_fed_ = 0;
    bool tx_active_ = false;
    std::deque<uint64_t> activity_starts_;
};

void validateEndpointConfig(const std::string& name, const EndpointConfig& endpoint) {
    if (endpoint.initial_state != protocol::ConnectionState::DISCONNECTED) {
        throw std::runtime_error(
            "ota_simulator v2 endpoint '" + name +
            "' supports only initial_state DISCONNECTED");
    }
    if (endpoint.initial_mode.waveform != protocol::WaveformMode::MC_DPSK) {
        throw std::runtime_error(
            "ota_simulator v2 endpoint '" + name +
            "' supports only MC_DPSK initial_mode");
    }
    (void)mcDpskConfigFor(endpoint.initial_mode.modulation);
}

EndpointRuntime& endpointFor(std::map<std::string, std::unique_ptr<EndpointRuntime>>& endpoints,
                             const std::string& name) {
    auto it = endpoints.find(name);
    if (it == endpoints.end()) {
        throw std::runtime_error("unknown endpoint '" + name + "'");
    }
    return *it->second;
}

bool frameMatches(const TxFrameRecord& frame, const TxFrameWithinAssert& assertion) {
    if (frame.frame_type != assertion.frame_type) {
        return false;
    }
    if (assertion.seq && frame.seq != *assertion.seq) {
        return false;
    }
    const double end_t = assertion.since_t_s + assertion.max_age_s;
    return frame.t_s >= assertion.since_t_s && frame.t_s <= end_t;
}

std::string describeTxAssert(const TxFrameWithinAssert& assertion) {
    std::ostringstream oss;
    oss << "tx_frame_within frame_type=" << assertion.frame_type
        << " window=[" << assertion.since_t_s << ","
        << (assertion.since_t_s + assertion.max_age_s) << "]";
    if (assertion.seq) {
        oss << " seq=" << *assertion.seq;
    }
    return oss.str();
}

bool hasReceivedMessageContaining(EndpointRuntime& runtime, const std::string& needle) {
    std::lock_guard<std::mutex> lock(runtime.mutex);
    return std::any_of(runtime.received_messages.begin(), runtime.received_messages.end(),
                       [&](const std::string& msg) {
                           return msg.find(needle) != std::string::npos;
                       });
}

size_t maxSuccessfulReceivedFileSize(EndpointRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.mutex);
    size_t max_size = 0;
    for (const auto& file : runtime.received_files) {
        if (file.success) {
            max_size = std::max(max_size, file.bytes.size());
        }
    }
    return max_size;
}

std::vector<ReceivedFileRecord> snapshotReceivedFiles(EndpointRuntime& runtime) {
    std::lock_guard<std::mutex> lock(runtime.mutex);
    return runtime.received_files;
}

std::vector<SentFileRecord> snapshotSentFiles(
    std::map<std::string, std::unique_ptr<EndpointRuntime>>& endpoints) {
    std::vector<SentFileRecord> out;
    for (auto& [name, runtime] : endpoints) {
        (void)name;
        std::lock_guard<std::mutex> lock(runtime->mutex);
        out.insert(out.end(), runtime->sent_files.begin(), runtime->sent_files.end());
    }
    return out;
}

bool hasByteExactReceivedFile(EndpointRuntime& runtime,
                              const std::vector<SentFileRecord>& sent_files) {
    const auto received_files = snapshotReceivedFiles(runtime);
    for (const auto& received : received_files) {
        if (!received.success) {
            continue;
        }
        for (const auto& sent : sent_files) {
            if (received.bytes == sent.bytes) {
                return true;
            }
        }
    }
    return false;
}

bool evaluateAssert(const ScenarioEvent& event,
                    EndpointRuntime& runtime,
                    const std::vector<SentFileRecord>& sent_files,
                    SessionLog& log,
                    bool check_state_and_messages,
                    bool check_tx) {
    bool ok = true;
    if (check_state_and_messages && event.assert_state) {
        const auto actual = runtime.station->getConnectionState();
        const bool pass = actual == *event.assert_state;
        ok = ok && pass;
        std::ostringstream desc;
        desc << "state expected=" << connectionStateName(*event.assert_state)
             << " actual=" << connectionStateName(actual);
        log.writeAssert(runtime.name, event.t_s, pass, desc.str());
    }

    if (check_state_and_messages && event.assert_received_message_contains) {
        const bool found = hasReceivedMessageContaining(
            runtime, *event.assert_received_message_contains);
        ok = ok && found;
        log.writeAssert(runtime.name, event.t_s, found,
                        "received_message_contains=\"" +
                            *event.assert_received_message_contains + "\"");
    }

    if (check_state_and_messages && event.assert_received_file_size_at_least) {
        const size_t actual = maxSuccessfulReceivedFileSize(runtime);
        const bool pass = actual >= *event.assert_received_file_size_at_least;
        ok = ok && pass;
        std::ostringstream desc;
        desc << "received_file_size_at_least expected="
             << *event.assert_received_file_size_at_least
             << " actual_max=" << actual;
        log.writeAssert(runtime.name, event.t_s, pass, desc.str());
    }

    if (check_state_and_messages && event.assert_received_file_byte_exact) {
        const bool pass = hasByteExactReceivedFile(runtime, sent_files);
        ok = ok && pass;
        std::ostringstream desc;
        desc << "received_file_byte_exact sent_files=" << sent_files.size()
             << " received_files=" << snapshotReceivedFiles(runtime).size();
        log.writeAssert(runtime.name, event.t_s, pass, desc.str());
    }

    if (check_tx && event.assert_tx_frame_within) {
        const auto& assertion = *event.assert_tx_frame_within;
        const bool found = std::any_of(runtime.tx_frames.begin(), runtime.tx_frames.end(),
                                       [&](const TxFrameRecord& frame) {
                                           return frameMatches(frame, assertion);
                                       });
        ok = ok && found;
        log.writeAssert(runtime.name, event.t_s, found, describeTxAssert(assertion));
    }
    return ok;
}

void executeCommand(const ScenarioEvent& event, EndpointRuntime& runtime,
                    const Scenario& scenario,
                    SessionLog& log) {
    if (event.action == "connect_to") {
        runtime.station->connect(event.peer_callsign);
        log.writeNote(runtime.name, event.t_s, "command",
                      "{\"action\":\"connect_to\",\"peer_callsign\":\"" +
                          json::escape(event.peer_callsign) + "\"}");
    } else if (event.action == "send_message") {
        runtime.station->sendMessage(event.text);
        log.writeNote(runtime.name, event.t_s, "command",
                      "{\"action\":\"send_message\",\"text\":\"" +
                          json::escape(event.text) + "\"}");
    } else if (event.action == "send_file") {
        if (!event.file_size_bytes) {
            throw std::runtime_error("send_file missing size_bytes");
        }

        std::string path;
        std::vector<uint8_t> bytes;
        if (event.filename) {
            path = resolveInputPath(scenario.source_path, *event.filename);
            bytes = readBinaryFile(path);
            if (bytes.size() != *event.file_size_bytes) {
                std::ostringstream err;
                err << "send_file filename size mismatch: " << path
                    << " has " << bytes.size() << " bytes, expected "
                    << *event.file_size_bytes;
                throw std::runtime_error(err.str());
            }
        } else {
            bytes = deterministicFilePayload(*event.file_size_bytes);
            path = generatedFilePath(runtime, event);
            writeBinaryFile(path, bytes);
        }

        if (!runtime.station->sendFile(path)) {
            log.writeNote(runtime.name, event.t_s, "command",
                          "{\"action\":\"send_file\",\"size_bytes\":" +
                              std::to_string(bytes.size()) +
                              ",\"started\":false}");
            throw std::runtime_error("send_file failed to start for endpoint '" +
                                     runtime.name + "'");
        }

        const double sent_t_s = runtime.station->getSimTime();
        {
            std::lock_guard<std::mutex> lock(runtime.mutex);
            runtime.sent_files.push_back(SentFileRecord{
                path, std::move(bytes), sent_t_s, runtime.name});
        }
        log.writeNote(runtime.name, event.t_s, "command",
                      "{\"action\":\"send_file\",\"size_bytes\":" +
                          std::to_string(*event.file_size_bytes) +
                          ",\"path\":\"" + json::escape(path) + "\"}");
    } else if (event.action == "disconnect") {
        runtime.station->disconnect();
        log.writeNote(runtime.name, event.t_s, "command",
                      "{\"action\":\"disconnect\"}");
    } else {
        throw std::runtime_error("unsupported command action '" + event.action + "'");
    }
}

std::optional<float> configureChannel(const Scenario& scenario, SimulatedChannel& channel) {
    channel.setSeed(42);
    std::optional<float> station_snr_db;
    if (scenario.channel && scenario.channel->snr_db) {
        station_snr_db = static_cast<float>(*scenario.channel->snr_db);
        const ChannelType channel_type =
            scenario.channel->channel_type.value_or(ChannelType::AWGN);
        channel.configure(*station_snr_db, channel_type);
    } else if (scenario.channel && scenario.channel->channel_type) {
        throw std::runtime_error("channel.fading/type requires channel.snr_db");
    }
    if (scenario.channel && scenario.channel->noise_bed) {
        const auto& bed = *scenario.channel->noise_bed;
        const std::string path = resolveInputPath(scenario.source_path, bed.file);
        auto wav = io::loadWavMono48k(path);
        scaleToTargetRms(wav.samples_48k, bed.target_rms);
        if (!station_snr_db) {
            station_snr_db = estimateNoiseBedStationSNRDb(wav.samples_48k);
            if (!station_snr_db) {
                throw std::runtime_error(
                    "noise_bed station SNR estimate failed; set channel.snr_db explicitly");
            }
            std::cout << "ota_simulator: noise_bed station_snr_db="
                      << *station_snr_db << " from " << path << "\n";
        }
        channel.setNoiseOverlay(std::move(wav.samples_48k), bed.loop, 0.0f);
    }
    return station_snr_db;
}

void emitFileTransferSummaries(
    std::map<std::string, std::unique_ptr<EndpointRuntime>>& endpoints,
    SessionLog& log) {
    const auto sent_files = snapshotSentFiles(endpoints);
    for (const auto& sent : sent_files) {
        for (const auto& [name, runtime] : endpoints) {
            if (name == sent.sender) continue;
            const auto received_files = snapshotReceivedFiles(*runtime);
            for (const auto& received : received_files) {
                if (!received.success) continue;
                if (received.bytes != sent.bytes) continue;
                const double transfer_sec = received.t_s - sent.t_s;
                const double bps = transfer_sec > 0.01
                    ? static_cast<double>(sent.bytes.size()) * 8.0 / transfer_sec
                    : 0.0;
                std::cout << "[" << sent.sender << " -> " << name << "] file "
                          << sent.bytes.size() << " B in "
                          << std::fixed << std::setprecision(1) << transfer_sec
                          << "s = " << std::fixed << std::setprecision(0)
                          << bps << " bps\n";
                std::ostringstream fields;
                fields << "{\"sender\":\"" << json::escape(sent.sender) << "\""
                       << ",\"receiver\":\"" << json::escape(name) << "\""
                       << ",\"size_bytes\":" << sent.bytes.size()
                       << ",\"transfer_sec\":" << std::fixed
                       << std::setprecision(3) << transfer_sec
                       << ",\"throughput_bps\":" << std::fixed
                       << std::setprecision(1) << bps << "}";
                log.writeNote(received.t_s, "file_transfer.complete",
                              fields.str());
                goto next_sent;
            }
        }
    next_sent:;
    }
}

void writeCaptures(const Scenario& scenario,
                   const std::vector<std::string>& endpoint_names,
                   const SimulatedChannel::CapturedSignals& captured) {
    (void)endpoint_names;
    if (!io::writeWavF32Mono(scenario.output.alice_tx_capture, captured.a_tx_raw)) {
        throw std::runtime_error("failed to write TX capture: " +
                                 scenario.output.alice_tx_capture);
    }
    if (!io::writeWavF32Mono(scenario.output.bob_tx_capture, captured.b_tx_raw)) {
        throw std::runtime_error("failed to write TX capture: " +
                                 scenario.output.bob_tx_capture);
    }
    if (!scenario.output.alice_rx_capture.empty()) {
        if (!io::writeWavF32Mono(scenario.output.alice_rx_capture,
                                 captured.a_rx_raw)) {
            throw std::runtime_error("failed to write RX capture: " +
                                     scenario.output.alice_rx_capture);
        }
    }
    if (!scenario.output.bob_rx_capture.empty()) {
        if (!io::writeWavF32Mono(scenario.output.bob_rx_capture,
                                 captured.b_rx_raw)) {
            throw std::runtime_error("failed to write RX capture: " +
                                     scenario.output.bob_rx_capture);
        }
    }
}

}  // namespace

int runScenarioV2(const Scenario& scenario) {
    if (scenario.endpoints.size() != 2) {
        throw std::runtime_error("ota_simulator v2 requires exactly two endpoints");
    }

    std::vector<std::string> endpoint_names;
    endpoint_names.reserve(2);
    for (const auto& [name, endpoint] : scenario.endpoints) {
        validateEndpointConfig(name, endpoint);
        endpoint_names.push_back(name);
    }

    SimulatedChannel channel;
    const auto station_snr_db = configureChannel(scenario, channel);
    const size_t capture_samples = static_cast<size_t>(
        std::ceil(scenario.duration_s * SimulatedStation::SAMPLE_RATE)) +
        static_cast<size_t>(10 * SimulatedStation::SAMPLE_RATE);
    channel.setSignalCaptureMaxSamples(capture_samples);
    channel.clearCapturedSignals();
    channel.setSignalCaptureEnabled(true);

    std::map<std::string, std::unique_ptr<EndpointRuntime>> endpoints;
    for (size_t i = 0; i < endpoint_names.size(); ++i) {
        const std::string& name = endpoint_names[i];
        const auto& cfg = scenario.endpoints.at(name);
        auto runtime = std::make_unique<EndpointRuntime>();
        runtime->name = name;
        runtime->config = cfg;
        runtime->last_state = cfg.initial_state;
        auto port = std::make_unique<VirtualAudioPort>(channel, i == 0);
        // Simulator has no PTT-off settling (rx_settling_ms defaults to 0),
        // so connection-level PTT-dodge holds add pure latency with no PHY
        // benefit. Real-radio code paths keep the safe 500 ms defaults.
        protocol::ConnectionConfig connection_config;
        connection_config.pong_tx_delay_ms = 0;
        connection_config.post_connect_data_delay_ms = 0;
        connection_config.ack_tx_delay_ms = 0;
        runtime->station = std::make_unique<SimulatedStation>(
            cfg.callsign, std::move(port),
            OFDMConfigPreset::Default,
            mcDpskConfigFor(cfg.initial_mode.modulation),
            connection_config);
        runtime->station->setReceiveDirectory(receiveDirectoryPath(name));
        runtime->station->setAutoAccept(cfg.auto_accept);
        if (cfg.force_data_mode) {
            runtime->station->setForcedModulation(cfg.initial_mode.modulation);
            runtime->station->setForcedCodeRate(cfg.initial_mode.code_rate);
        }
        if (station_snr_db) {
            runtime->station->setSNR(*station_snr_db);
        }
        if (scenario.channel && scenario.channel->force_connected_waveform) {
            runtime->station->setPreferredWaveform(
                *scenario.channel->force_connected_waveform);
        }
        endpoints.emplace(name, std::move(runtime));
    }

    SessionLog log(scenario.output.session_log);
    for (auto& [name, runtime] : endpoints) {
        runtime->station->setRxDecodeResultCallback(
            [&log, runtime = runtime.get()](const gui::DecodeResult& result) {
                log.writeRxFrame(runtime->name,
                                 runtime->station->getSimTime(),
                                 result);
            });
        runtime->station->setMessageCallback([&log, runtime = runtime.get()](const std::string& msg) {
            {
                std::lock_guard<std::mutex> lock(runtime->mutex);
                runtime->received_messages.push_back(msg);
            }
            log.writeNote(runtime->name, runtime->station->getSimTime(), "message.rx",
                          "{\"text\":\"" + json::escape(msg) + "\"}");
        });
        runtime->station->setFileReceivedCallback(
            [&log, runtime = runtime.get()](const std::string& path, bool success) {
                std::vector<uint8_t> bytes;
                bool captured = success;
                std::string error;
                if (success && !path.empty()) {
                    try {
                        bytes = readBinaryFile(path);
                    } catch (const std::exception& e) {
                        captured = false;
                        error = e.what();
                    }
                }
                const size_t captured_size = bytes.size();
                const double recv_t_s = runtime->station->getSimTime();
                {
                    std::lock_guard<std::mutex> lock(runtime->mutex);
                    runtime->received_files.push_back(
                        ReceivedFileRecord{path, captured, std::move(bytes), recv_t_s});
                }

                std::ostringstream fields;
                fields << "{\"success\":" << (captured ? "true" : "false")
                       << ",\"path\":\"" << json::escape(path) << "\""
                       << ",\"size_bytes\":" << captured_size;
                if (!error.empty()) {
                    fields << ",\"error\":\"" << json::escape(error) << "\"";
                }
                fields << "}";
                log.writeNote(runtime->name, runtime->station->getSimTime(),
                              "file.rx", fields.str());
            });
    }

    for (auto& [name, runtime] : endpoints) {
        runtime->station->start();
    }
    for (auto& [name, runtime] : endpoints) {
        log.writeState(name, 0.0, runtime->last_state);
    }

    const auto start = std::chrono::steady_clock::now();
    size_t event_index = 0;
    int assertion_failures = 0;
    std::vector<ScenarioEvent> pending_tx_asserts;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= scenario.duration_s) {
            break;
        }

        for (auto& [name, runtime] : endpoints) {
            runtime->station->tick();
        }

        for (auto& [name, runtime] : endpoints) {
            const auto state = runtime->station->getConnectionState();
            if (state != runtime->last_state) {
                log.writeState(name, elapsed, state);
                runtime->last_state = state;
            }
        }

        while (event_index < scenario.events.size() &&
               scenario.events[event_index].t_s <= elapsed) {
            const auto& event = scenario.events[event_index];
            if (event.type == ScenarioEvent::Type::Command) {
                executeCommand(event, endpointFor(endpoints, event.endpoint),
                               scenario, log);
            } else if (event.type == ScenarioEvent::Type::Assert) {
                auto& runtime = endpointFor(endpoints, event.endpoint);
                if (!evaluateAssert(event, runtime, snapshotSentFiles(endpoints),
                                    log, true, false)) {
                    ++assertion_failures;
                }
                if (event.assert_tx_frame_within) {
                    pending_tx_asserts.push_back(event);
                }
            } else if (event.type == ScenarioEvent::Type::Wait) {
                log.writeNote(event.t_s, "wait", "{}");
            } else {
                throw std::runtime_error("ota_simulator v2 does not support event type '" +
                                         std::string(eventTypeName(event.type)) + "'");
            }
            ++event_index;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for (auto& [name, runtime] : endpoints) {
        runtime->station->stop();
    }
    channel.setSignalCaptureEnabled(false);

    for (auto& [name, runtime] : endpoints) {
        const auto state = runtime->station->getConnectionState();
        if (state != runtime->last_state) {
            log.writeState(name, scenario.duration_s, state);
            runtime->last_state = state;
        }
    }

    while (event_index < scenario.events.size()) {
        const auto& event = scenario.events[event_index];
        if (event.type == ScenarioEvent::Type::Assert) {
            auto& runtime = endpointFor(endpoints, event.endpoint);
            if (!evaluateAssert(event, runtime, snapshotSentFiles(endpoints),
                                log, true, false)) {
                ++assertion_failures;
            }
            if (event.assert_tx_frame_within) {
                pending_tx_asserts.push_back(event);
            }
        } else if (event.type == ScenarioEvent::Type::Wait) {
            log.writeNote(event.t_s, "wait", "{}");
        }
        ++event_index;
    }

    auto captured = channel.getCapturedSignals();
    {
        TxMonitor monitor_a(scenario.endpoints.at(endpoint_names[0]).initial_mode,
                            "PING_OR_PONG");
        auto& runtime = endpointFor(endpoints, endpoint_names[0]);
        monitor_a.feed(endpoint_names[0], captured.a_tx_raw, log, runtime.tx_frames);
    }
    {
        TxMonitor monitor_b(scenario.endpoints.at(endpoint_names[1]).initial_mode,
                            "PING_OR_PONG");
        auto& runtime = endpointFor(endpoints, endpoint_names[1]);
        monitor_b.feed(endpoint_names[1], captured.b_tx_raw, log, runtime.tx_frames);
    }

    for (const auto& event : pending_tx_asserts) {
        auto& runtime = endpointFor(endpoints, event.endpoint);
        if (!evaluateAssert(event, runtime, snapshotSentFiles(endpoints),
                            log, false, true)) {
            ++assertion_failures;
        }
    }

    writeCaptures(scenario, endpoint_names, captured);

    emitFileTransferSummaries(endpoints, log);

    if (assertion_failures != 0) {
        std::cerr << "ota_simulator: " << assertion_failures
                  << " assertion(s) failed; see " << scenario.output.session_log << "\n";
        return 1;
    }

    std::cout << "ota_simulator: scenario passed; wrote "
              << scenario.output.alice_tx_capture << ", "
              << scenario.output.bob_tx_capture << ", and "
              << scenario.output.session_log << "\n";
    return 0;
}

}  // namespace ultra::tools::ota
