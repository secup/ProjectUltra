#pragma once

#include "ota_channel_core/mixer.hpp"
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/rng.hpp"
#include "ota_channel_core/session_config.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace ultra::ota_channel_core {

struct SessionEvent {
    uint64_t sequence = 0;
    uint64_t sample_index = 0;
    std::string type;
    std::string station_id;
};

struct CaptureState {
    bool enabled = false;
    uint64_t tx_samples = 0;
    uint64_t rx_samples = 0;
};

class SessionContext {
public:
    explicit SessionContext(SessionConfig config);

    const SessionConfig& config() const { return config_; }
    const std::string& id() const { return config_.session_id; }
    ChannelType channelType() const { return config_.default_channel_model; }
    float snrDb() const { return config_.default_snr_db; }
    uint64_t seed() const { return config_.seed; }

    bool registerStation(std::string station_id);
    bool leaveStation(std::string_view station_id);
    bool hasStation(std::string_view station_id) const;
    size_t stationCount() const;
    std::vector<std::string> listStations() const;

    bool submitTransmit(std::string_view station_id,
                        uint64_t start_sample,
                        std::span<const float> samples);
    bool receiveForStation(std::string_view station_id,
                           uint64_t start_sample,
                           size_t count,
                           std::vector<float>& output);
    std::vector<float> receiveForStation(std::string_view station_id,
                                         uint64_t start_sample,
                                         size_t count);
    void discardBefore(uint64_t sample_index);
    size_t pendingAudioBlocks() const;

    RngStream rngStream(std::string_view name, uint64_t index = 0) const;
    uint32_t rngChildSeed(std::string_view name, uint64_t index = 0) const;

    void setCaptureEnabled(bool enabled);
    CaptureState captureState() const;

    void appendEvent(std::string type,
                     std::string station_id = {},
                     uint64_t sample_index = 0);
    std::vector<SessionEvent> eventLog() const;

private:
    void appendEventLocked(std::string type,
                           std::string station_id,
                           uint64_t sample_index);

    SessionConfig config_;
    RngRoot rng_root_;
    std::unique_ptr<IChannelModel> channel_;
    SampleIndexedMixer mixer_;
    std::set<std::string> stations_;
    CaptureState capture_;
    std::vector<SessionEvent> events_;
    uint64_t next_event_sequence_ = 0;
    mutable std::mutex mutex_;
};

}  // namespace ultra::ota_channel_core
