#pragma once

#include "ota_channel_core/mixer.hpp"
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/rng.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <vector>

namespace ultra::ota_channel_core {

class SimulatedChannel {
public:
    struct CapturedSignals {
        std::vector<float> a_tx_raw;
        std::vector<float> b_tx_raw;
        std::vector<float> a_rx_raw;
        std::vector<float> b_rx_raw;
        bool truncated = false;
        size_t max_samples = 0;
    };

    SimulatedChannel();

    void setSeed(uint64_t seed);
    uint64_t seed() const { return seed_; }
    void setTxCFO(float cfo_hz);
    float getTxCFO() const { return tx_cfo_hz_; }

    void configure(float snr_db, ChannelType channel_type = ChannelType::AWGN);
    ChannelType channelType() const { return channel_type_; }
    float snrDb() const { return snr_db_; }

    void setSignalCaptureEnabled(bool enabled);
    void setSignalCaptureMaxSamples(size_t max_samples);
    void clearCapturedSignals();
    CapturedSignals getCapturedSignals() const;

    void setNoiseOverlay(std::vector<float> bed, bool loop, float target_rms);
    void setRxBlackoutCallback(bool is_station_a, std::function<bool()> callback);

    void transmitFromA(const std::vector<float>& samples);
    void transmitFromB(const std::vector<float>& samples);
    std::vector<float> receiveForA(size_t count);
    std::vector<float> receiveForB(size_t count);

private:
    void rebuildModels();
    std::vector<float> applyChannel(std::span<const float> samples,
                                    IChannelModel& model);
    void addReceiveNoise(std::vector<float>& samples, bool for_a);
    void applyOverlay(std::vector<float>& out, uint64_t& cursor);
    bool isStationAInRxBlackout() const;
    bool isStationBInRxBlackout() const;
    void captureTxIfEnabled(std::span<const float> tx_raw, bool from_a);
    void captureRxIfEnabled(std::span<const float> rx_raw, bool for_a);
    void appendLimited(std::vector<float>& dst, std::span<const float> src);
    std::vector<float> applyTxCFO(std::span<const float> samples, float& phase_acc);

    uint64_t seed_ = 42;
    float snr_db_ = 20.0f;
    ChannelType channel_type_ = ChannelType::PASSTHROUGH;
    float noise_stddev_ = 0.0f;
    float tx_cfo_hz_ = 0.0f;
    float cfo_phase_a_to_b_ = 0.0f;
    float cfo_phase_b_to_a_ = 0.0f;

    std::mt19937 awgn_rng_{42};
    std::normal_distribution<float> noise_dist_{0.0f, 1.0f};
    std::mutex rng_mutex_;
    std::unique_ptr<IChannelModel> channel_a_to_b_;
    std::unique_ptr<IChannelModel> channel_b_to_a_;

    std::mutex mutex_a_rx_;
    std::mutex mutex_b_rx_;
    std::queue<float> buffer_a_rx_;
    std::queue<float> buffer_b_rx_;

    std::vector<float> noise_overlay_;
    bool noise_overlay_loop_ = false;
    bool has_noise_overlay_ = false;
    uint64_t noise_overlay_cursor_a_ = 0;
    uint64_t noise_overlay_cursor_b_ = 0;

    mutable std::mutex rx_blackout_mutex_;
    std::function<bool()> station_a_rx_blackout_;
    std::function<bool()> station_b_rx_blackout_;

    std::atomic<bool> capture_enabled_{false};
    mutable std::mutex capture_mutex_;
    CapturedSignals captured_;
    size_t capture_max_samples_ = 0;
};

}  // namespace ultra::ota_channel_core
