#pragma once

#include "ota_channel_core/session_config.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <span>
#include <string>
#include <vector>

namespace ultra::ota_simulator_service {

struct CaptureEvent {
    std::string event_id;
    std::string session_id;
    uint64_t sample_index = 0;
    std::string type;
    std::string payload_json;
};

struct CaptureSummary {
    std::string session_id;
    bool active = false;
    std::filesystem::path capture_path;
    std::filesystem::path manifest_path;
    uint64_t started_at_sample = 0;
    uint64_t stopped_at_sample = 0;
    uint64_t tx_sample_count = 0;
    uint64_t rx_sample_count = 0;
};

class SessionCaptureWriter {
public:
    bool start(const std::filesystem::path& capture_root,
               std::string session_id,
               uint64_t start_sample,
               const ultra::ota_channel_core::SessionConfig& config,
               const std::vector<std::string>& stations,
               std::string* error = nullptr);
    CaptureSummary stop(const ultra::ota_channel_core::SessionConfig& config,
                        const std::vector<std::string>& stations,
                        std::string* error = nullptr);

    bool active() const;
    CaptureSummary summary() const;
    void recordTx(std::string_view station_id,
                  uint64_t start_sample,
                  std::span<const float> samples);
    void recordRx(std::string_view station_id,
                  uint64_t start_sample,
                  std::span<const float> samples);
    void recordEvent(const CaptureEvent& event);

private:
    class WavWriter {
    public:
        bool open(const std::filesystem::path& path);
        void write(std::span<const float> samples);
        void close();
        uint64_t sampleCount() const { return sample_count_; }

    private:
        std::ofstream out_;
        uint64_t sample_count_ = 0;
    };

    WavWriter& writerFor(std::string_view station_id, std::string_view direction);
    void writeManifest(const ultra::ota_channel_core::SessionConfig& config,
                       const std::vector<std::string>& stations);

    mutable std::mutex mutex_;
    bool active_ = false;
    std::string session_id_;
    std::filesystem::path capture_path_;
    std::filesystem::path manifest_path_;
    uint64_t started_at_sample_ = 0;
    uint64_t stopped_at_sample_ = 0;
    uint64_t tx_sample_count_ = 0;
    uint64_t rx_sample_count_ = 0;
    std::map<std::string, WavWriter> writers_;
    std::ofstream events_;
};

}  // namespace ultra::ota_simulator_service
