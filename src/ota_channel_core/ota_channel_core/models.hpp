#pragma once

#include "ota_channel_core/rng.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <random>
#include <span>
#include <vector>

namespace ultra::ota_channel_core {

inline constexpr uint32_t kDefaultSampleRate = 48000;
inline constexpr float kModemReferenceRms = 0.3180724f;
inline constexpr double kModemReferencePower =
    static_cast<double>(kModemReferenceRms) *
    static_cast<double>(kModemReferenceRms);

enum class ChannelType {
    PASSTHROUGH,
    AWGN,
    GOOD,
    MODERATE,
    POOR,
    FLUTTER
};

struct ChannelConfig {
    ChannelType type = ChannelType::PASSTHROUGH;
    float snr_db = 20.0f;
    uint64_t seed = 42;
    uint32_t sample_rate = kDefaultSampleRate;
};

float modemReferenceNoiseStddev(float snr_db);
const char* channelTypeName(ChannelType type);

class IChannelModel {
public:
    virtual ~IChannelModel() = default;
    virtual void reset() = 0;
    virtual void process(std::span<const float> input, std::vector<float>& output) = 0;

    std::vector<float> process(std::span<const float> input) {
        std::vector<float> output;
        process(input, output);
        return output;
    }
};

class PassthroughChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    void reset() override {}
    void process(std::span<const float> input, std::vector<float>& output) override;
};

class AWGNChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    AWGNChannelModel(float snr_db, RngStream rng);

    void reset() override {}
    void setSNR(float snr_db);
    void process(std::span<const float> input, std::vector<float>& output) override;

private:
    float noise_stddev_ = 0.0f;
    RngStream rng_;
};

class WattersonChannel {
public:
    struct Config {
        float snr_db = 15.0f;
        float delay_spread_ms = 2.0f;
        float doppler_spread_hz = 1.0f;
        float cfo_hz = 0.0f;
        float random_cfo_max_hz = 0.0f;
        float path1_gain = 0.707f;
        float path2_gain = 0.707f;
        uint32_t sample_rate = kDefaultSampleRate;
        bool fading_enabled = true;
        bool multipath_enabled = true;
        bool noise_enabled = true;
        bool cfo_enabled = true;
    };

    explicit WattersonChannel(const Config& config, uint64_t seed = 42);

    void reset();
    void setSNR(float snr_db);
    void process(std::span<const float> input, std::vector<float>& output);
    std::vector<float> process(std::span<const float> input);

    float actualCFO() const { return actual_cfo_hz_; }
    float fadingMagnitude() const;
    const Config& config() const { return config_; }

private:
    void updateFading();
    void applyCFO(std::vector<float>& samples);

    Config config_;
    std::mt19937 rng_;
    std::normal_distribution<float> gaussian_{0.0f, 1.0f};
    std::deque<float> delay_line_;
    size_t delay_samples_ = 0;
    float noise_stddev_ = 0.0f;
    float fading_alpha_ = 0.0f;
    std::complex<float> fading1_{1.0f, 0.0f};
    std::complex<float> fading2_{1.0f, 0.0f};
    float cfo_phase_ = 0.0f;
    float cfo_phase_inc_ = 0.0f;
    float actual_cfo_hz_ = 0.0f;
};

class WattersonChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    WattersonChannelModel(const WattersonChannel::Config& config, uint64_t seed);

    void reset() override;
    void setSNR(float snr_db);
    void process(std::span<const float> input, std::vector<float>& output) override;

    const WattersonChannel::Config& config() const;

private:
    WattersonChannel channel_;
};

namespace itu_r_f1487 {
WattersonChannel::Config good(float snr_db = 20.0f);
WattersonChannel::Config moderate(float snr_db = 20.0f);
WattersonChannel::Config poor(float snr_db = 20.0f);
WattersonChannel::Config flutter(float snr_db = 20.0f);
WattersonChannel::Config awgn(float snr_db = 20.0f);
}  // namespace itu_r_f1487

WattersonChannel::Config configForWatterson(ChannelType type,
                                            float snr_db,
                                            uint32_t sample_rate = kDefaultSampleRate);
std::unique_ptr<IChannelModel> createChannelModel(const ChannelConfig& config,
                                                  const RngRoot& rng_root,
                                                  std::string_view stream_name);

}  // namespace ultra::ota_channel_core
