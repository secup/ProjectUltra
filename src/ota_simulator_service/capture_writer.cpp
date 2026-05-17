#include "ota_simulator_service/capture_writer.hpp"

#include "ota_channel_core/models.hpp"
#include "ultra/version.hpp"

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <sstream>
#include <utility>

namespace ultra::ota_simulator_service {
namespace {

void writeLe16(std::ostream& out, uint16_t value) {
    const uint8_t bytes[2] = {
        static_cast<uint8_t>(value & 0xffu),
        static_cast<uint8_t>((value >> 8) & 0xffu),
    };
    out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

void writeLe32(std::ostream& out, uint32_t value) {
    const uint8_t bytes[4] = {
        static_cast<uint8_t>(value & 0xffu),
        static_cast<uint8_t>((value >> 8) & 0xffu),
        static_cast<uint8_t>((value >> 16) & 0xffu),
        static_cast<uint8_t>((value >> 24) & 0xffu),
    };
    out.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

std::string jsonEscape(std::string_view value) {
    std::ostringstream out;
    for (unsigned char c : value) {
        switch (c) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\b': out << "\\b"; break;
            case '\f': out << "\\f"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (c < 0x20) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(c) << std::dec;
                } else {
                    out << static_cast<char>(c);
                }
        }
    }
    return out.str();
}

std::string pathComponent(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (unsigned char c : value) {
        if (std::isalnum(c) || c == '-' || c == '_') {
            out.push_back(static_cast<char>(c));
        } else {
            out.push_back('_');
        }
    }
    return out.empty() ? "station" : out;
}

}  // namespace

bool SessionCaptureWriter::WavWriter::open(const std::filesystem::path& path) {
    close();
    out_.open(path, std::ios::binary);
    if (!out_) {
        return false;
    }
    sample_count_ = 0;
    out_.write("RIFF", 4);
    writeLe32(out_, 36);
    out_.write("WAVE", 4);
    out_.write("fmt ", 4);
    writeLe32(out_, 16);
    writeLe16(out_, 3);
    writeLe16(out_, 1);
    writeLe32(out_, ultra::ota_channel_core::kDefaultSampleRate);
    writeLe32(out_, ultra::ota_channel_core::kDefaultSampleRate * sizeof(float));
    writeLe16(out_, sizeof(float));
    writeLe16(out_, 32);
    out_.write("data", 4);
    writeLe32(out_, 0);
    return out_.good();
}

void SessionCaptureWriter::WavWriter::write(std::span<const float> samples) {
    if (!out_ || samples.empty()) {
        return;
    }
    out_.write(reinterpret_cast<const char*>(samples.data()),
               static_cast<std::streamsize>(samples.size() * sizeof(float)));
    sample_count_ += samples.size();
}

void SessionCaptureWriter::WavWriter::close() {
    if (!out_) {
        return;
    }
    const uint64_t data_bytes_u64 = sample_count_ * sizeof(float);
    const uint32_t data_bytes = static_cast<uint32_t>(data_bytes_u64);
    const uint32_t riff_size = 36u + data_bytes;
    out_.seekp(4, std::ios::beg);
    writeLe32(out_, riff_size);
    out_.seekp(40, std::ios::beg);
    writeLe32(out_, data_bytes);
    out_.close();
}

bool SessionCaptureWriter::start(const std::filesystem::path& capture_root,
                                 std::string session_id,
                                 uint64_t start_sample,
                                 const ultra::ota_channel_core::SessionConfig& config,
                                 const std::vector<std::string>& stations,
                                 std::string* error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) {
        return true;
    }

    session_id_ = std::move(session_id);
    capture_path_ = capture_root / pathComponent(session_id_);
    manifest_path_ = capture_path_ / "manifest.json";
    std::error_code ec;
    std::filesystem::create_directories(capture_path_, ec);
    if (ec) {
        if (error) {
            *error = "failed to create capture directory: " + ec.message();
        }
        return false;
    }

    events_.open(capture_path_ / "events.jsonl", std::ios::app);
    if (!events_) {
        if (error) {
            *error = "failed to open capture event log";
        }
        return false;
    }

    writers_.clear();
    started_at_sample_ = start_sample;
    stopped_at_sample_ = start_sample;
    tx_sample_count_ = 0;
    rx_sample_count_ = 0;
    active_ = true;
    writeManifest(config, stations);
    events_ << "{\"event_id\":\"capture_start\",\"session_id\":\""
            << jsonEscape(session_id_) << "\",\"sample_index\":" << started_at_sample_
            << ",\"type\":\"capture_started\",\"payload\":{}}\n";
    events_.flush();
    return true;
}

CaptureSummary SessionCaptureWriter::stop(
    const ultra::ota_channel_core::SessionConfig& config,
    const std::vector<std::string>& stations,
    std::string* error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
        return {
            .session_id = session_id_,
            .active = active_,
            .capture_path = capture_path_,
            .manifest_path = manifest_path_,
            .started_at_sample = started_at_sample_,
            .stopped_at_sample = stopped_at_sample_,
            .tx_sample_count = tx_sample_count_,
            .rx_sample_count = rx_sample_count_,
        };
    }

    active_ = false;
    for (auto& [_, writer] : writers_) {
        writer.close();
    }
    writeManifest(config, stations);
    if (events_) {
        events_ << "{\"event_id\":\"capture_stop\",\"session_id\":\""
                << jsonEscape(session_id_) << "\",\"sample_index\":" << stopped_at_sample_
                << ",\"type\":\"capture_stopped\",\"payload\":{}}\n";
        events_.close();
    }
    if (error && !manifest_path_.empty() && !std::filesystem::exists(manifest_path_)) {
        *error = "capture manifest was not written";
    }
    return {
        .session_id = session_id_,
        .active = active_,
        .capture_path = capture_path_,
        .manifest_path = manifest_path_,
        .started_at_sample = started_at_sample_,
        .stopped_at_sample = stopped_at_sample_,
        .tx_sample_count = tx_sample_count_,
        .rx_sample_count = rx_sample_count_,
    };
}

bool SessionCaptureWriter::active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
}

CaptureSummary SessionCaptureWriter::summary() const {
    return {
        .session_id = session_id_,
        .active = active_,
        .capture_path = capture_path_,
        .manifest_path = manifest_path_,
        .started_at_sample = started_at_sample_,
        .stopped_at_sample = stopped_at_sample_,
        .tx_sample_count = tx_sample_count_,
        .rx_sample_count = rx_sample_count_,
    };
}

void SessionCaptureWriter::recordTx(std::string_view station_id,
                                    uint64_t start_sample,
                                    std::span<const float> samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
        return;
    }
    writerFor(station_id, "tx").write(samples);
    tx_sample_count_ += samples.size();
    stopped_at_sample_ = std::max(stopped_at_sample_,
                                  start_sample + static_cast<uint64_t>(samples.size()));
}

void SessionCaptureWriter::recordRx(std::string_view station_id,
                                    uint64_t start_sample,
                                    std::span<const float> samples) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) {
        return;
    }
    writerFor(station_id, "rx").write(samples);
    rx_sample_count_ += samples.size();
    stopped_at_sample_ = std::max(stopped_at_sample_,
                                  start_sample + static_cast<uint64_t>(samples.size()));
}

void SessionCaptureWriter::recordEvent(const CaptureEvent& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !events_) {
        return;
    }
    events_ << "{\"event_id\":\"" << jsonEscape(event.event_id)
            << "\",\"session_id\":\"" << jsonEscape(event.session_id)
            << "\",\"sample_index\":" << event.sample_index
            << ",\"type\":\"" << jsonEscape(event.type)
            << "\",\"payload\":" << (event.payload_json.empty() ? "{}" : event.payload_json)
            << "}\n";
    events_.flush();
}

SessionCaptureWriter::WavWriter& SessionCaptureWriter::writerFor(
    std::string_view station_id,
    std::string_view direction) {
    const std::string key = std::string(station_id) + ":" + std::string(direction);
    auto [it, inserted] = writers_.try_emplace(key);
    if (inserted) {
        const auto filename = pathComponent(station_id) + "_" +
                              std::string(direction) + "_48k_f32.wav";
        (void)it->second.open(capture_path_ / filename);
    }
    return it->second;
}

void SessionCaptureWriter::writeManifest(
    const ultra::ota_channel_core::SessionConfig& config,
    const std::vector<std::string>& stations) {
    if (manifest_path_.empty()) {
        return;
    }
    std::ofstream out(manifest_path_);
    if (!out) {
        return;
    }

    out << "{\n";
    out << "  \"server_version\":\"" << jsonEscape(ultra::kProjectUltraVersion) << "\",\n";
    out << "  \"session_id\":\"" << jsonEscape(session_id_) << "\",\n";
    out << "  \"channel\":{\"model\":\""
        << ultra::ota_channel_core::channelTypeName(config.default_channel_model)
        << "\",\"snr_db\":" << config.default_snr_db
        << ",\"seed\":" << config.seed << "},\n";
    out << "  \"sample_rate\":" << config.sample_rate << ",\n";
    out << "  \"sample_range\":{\"start\":" << started_at_sample_
        << ",\"stop\":" << stopped_at_sample_ << "},\n";
    out << "  \"stations\":[";
    for (size_t i = 0; i < stations.size(); ++i) {
        if (i) {
            out << ",";
        }
        out << "\"" << jsonEscape(stations[i]) << "\"";
    }
    out << "]\n";
    out << "}\n";
}

}  // namespace ultra::ota_simulator_service
