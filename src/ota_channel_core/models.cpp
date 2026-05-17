#include "ota_channel_core/models.hpp"

#define POCKETFFT_CACHE_SIZE 64
#if defined(__APPLE__) || defined(__unix__)
#define POCKETFFT_USE_POSIX_MEMALIGN
#endif

#include "pocketfft/pocketfft_hdronly.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ultra::ota_channel_core {
namespace {

using Complex = std::complex<float>;

constexpr float kPi = 3.14159265358979323846f;

void analyticFrequencyShift(std::vector<float>& samples,
                            float cfo_hz,
                            uint32_t sample_rate,
                            float& phase_acc) {
    if (samples.empty() || std::abs(cfo_hz) < 0.001f) {
        return;
    }

    size_t fft_size = 1;
    while (fft_size < samples.size()) {
        fft_size <<= 1;
    }

    std::vector<Complex> time(fft_size, Complex(0.0f, 0.0f));
    std::vector<Complex> freq(fft_size, Complex(0.0f, 0.0f));
    std::vector<Complex> analytic(fft_size, Complex(0.0f, 0.0f));
    for (size_t i = 0; i < samples.size(); ++i) {
        time[i] = Complex(samples[i], 0.0f);
    }

    const pocketfft::shape_t shape{fft_size};
    const pocketfft::stride_t stride{static_cast<ptrdiff_t>(sizeof(Complex))};
    const pocketfft::shape_t axes{0};
    pocketfft::c2c<float>(shape, stride, stride, axes,
                          pocketfft::FORWARD, time.data(), freq.data(), 1.0f);

    if (fft_size >= 2) {
        for (size_t i = 1; i < fft_size / 2; ++i) {
            freq[i] *= 2.0f;
        }
        for (size_t i = fft_size / 2 + 1; i < fft_size; ++i) {
            freq[i] = Complex(0.0f, 0.0f);
        }
    }

    pocketfft::c2c<float>(shape, stride, stride, axes,
                          pocketfft::BACKWARD, freq.data(), analytic.data(),
                          1.0f / static_cast<float>(fft_size));

    const float phase_inc = 2.0f * kPi * cfo_hz /
                            static_cast<float>(sample_rate);
    float phase = phase_acc;
    for (size_t i = 0; i < samples.size(); ++i) {
        const Complex rot(std::cos(phase), std::sin(phase));
        samples[i] = std::real(analytic[i] * rot);
        phase += phase_inc;
        if (phase > kPi) {
            phase -= 2.0f * kPi;
        } else if (phase < -kPi) {
            phase += 2.0f * kPi;
        }
    }
    phase_acc = phase;
}

}  // namespace

float modemReferenceNoiseStddev(float snr_db) {
    return kModemReferenceRms * std::pow(10.0f, -snr_db / 20.0f);
}

const char* channelTypeName(ChannelType type) {
    switch (type) {
        case ChannelType::PASSTHROUGH: return "passthrough";
        case ChannelType::AWGN:        return "awgn";
        case ChannelType::GOOD:        return "good";
        case ChannelType::MODERATE:    return "moderate";
        case ChannelType::POOR:        return "poor";
        case ChannelType::FLUTTER:     return "flutter";
        default:                       return "unknown";
    }
}

void PassthroughChannelModel::process(std::span<const float> input,
                                      std::vector<float>& output) {
    output.assign(input.begin(), input.end());
}

AWGNChannelModel::AWGNChannelModel(float snr_db, RngStream rng)
    : rng_(std::move(rng)) {
    setSNR(snr_db);
}

void AWGNChannelModel::setSNR(float snr_db) {
    noise_stddev_ = modemReferenceNoiseStddev(snr_db);
}

void AWGNChannelModel::process(std::span<const float> input,
                               std::vector<float>& output) {
    output.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        output[i] = input[i] + noise_stddev_ * rng_.normal();
    }
}

WattersonChannel::WattersonChannel(const Config& config, uint64_t seed)
    : config_(config),
      rng_(static_cast<uint32_t>(seed)) {
    delay_samples_ = static_cast<size_t>(
        config_.delay_spread_ms * static_cast<float>(config_.sample_rate) / 1000.0f);
    delay_line_.assign(delay_samples_, 0.0f);
    noise_stddev_ = modemReferenceNoiseStddev(config_.snr_db);

    const float normalized_doppler =
        config_.doppler_spread_hz / static_cast<float>(config_.sample_rate);
    fading_alpha_ = 1.0f - std::exp(-2.0f * kPi * normalized_doppler);
    if (fading_alpha_ <= 0.0f) {
        fading_alpha_ = 1.0f;
    }

    actual_cfo_hz_ = config_.cfo_hz;
    if (config_.random_cfo_max_hz > 0.0f) {
        std::uniform_real_distribution<float> cfo_dist(
            -config_.random_cfo_max_hz, config_.random_cfo_max_hz);
        actual_cfo_hz_ = cfo_dist(rng_);
    }
    cfo_phase_inc_ = 2.0f * kPi * actual_cfo_hz_ /
                     static_cast<float>(config_.sample_rate);
}

void WattersonChannel::reset() {
    std::fill(delay_line_.begin(), delay_line_.end(), 0.0f);
    fading1_ = Complex(1.0f, 0.0f);
    fading2_ = Complex(1.0f, 0.0f);
    cfo_phase_ = 0.0f;
}

void WattersonChannel::setSNR(float snr_db) {
    config_.snr_db = snr_db;
    noise_stddev_ = modemReferenceNoiseStddev(snr_db);
}

void WattersonChannel::process(std::span<const float> input,
                               std::vector<float>& output) {
    output.resize(input.size());

    for (size_t i = 0; i < input.size(); ++i) {
        const float sample = input[i];
        if (config_.fading_enabled) {
            updateFading();
        }

        float out = 0.0f;
        if (config_.multipath_enabled && delay_samples_ > 0) {
            const float h1_mag = config_.fading_enabled ? std::abs(fading1_) : 1.0f;
            const float h2_mag = config_.fading_enabled ? std::abs(fading2_) : 1.0f;
            out += sample * config_.path1_gain * h1_mag;

            const float delayed = delay_line_.front();
            delay_line_.pop_front();
            delay_line_.push_back(sample);
            out += delayed * config_.path2_gain * h2_mag;
        } else {
            const float h_mag = config_.fading_enabled ? std::abs(fading1_) : 1.0f;
            out = sample * h_mag;
        }

        if (config_.noise_enabled) {
            out += noise_stddev_ * gaussian_(rng_);
        }
        output[i] = out;
    }

    if (config_.cfo_enabled && std::abs(actual_cfo_hz_) > 0.001f) {
        applyCFO(output);
    }
}

std::vector<float> WattersonChannel::process(std::span<const float> input) {
    std::vector<float> output;
    process(input, output);
    return output;
}

float WattersonChannel::fadingMagnitude() const {
    return std::abs(fading1_);
}

void WattersonChannel::updateFading() {
    const float noise_scale = std::sqrt(1.0f / fading_alpha_);
    const Complex noise1(noise_scale * gaussian_(rng_),
                         noise_scale * gaussian_(rng_));
    const Complex noise2(noise_scale * gaussian_(rng_),
                         noise_scale * gaussian_(rng_));
    fading1_ = (1.0f - fading_alpha_) * fading1_ + fading_alpha_ * noise1;
    fading2_ = (1.0f - fading_alpha_) * fading2_ + fading_alpha_ * noise2;
}

void WattersonChannel::applyCFO(std::vector<float>& samples) {
    if (samples.size() < 256) {
        return;
    }

    constexpr float fc = 1500.0f;
    const float fs = static_cast<float>(config_.sample_rate);
    std::vector<float> i_bb(samples.size(), 0.0f);
    std::vector<float> q_bb(samples.size(), 0.0f);

    for (size_t i = 0; i < samples.size(); ++i) {
        const float t = static_cast<float>(i) / fs;
        const float phase = 2.0f * kPi * fc * t;
        i_bb[i] = samples[i] * std::cos(phase);
        q_bb[i] = samples[i] * std::sin(phase);
    }

    constexpr size_t win = 48;
    std::vector<float> i_filt(samples.size(), 0.0f);
    std::vector<float> q_filt(samples.size(), 0.0f);
    float i_sum = 0.0f;
    float q_sum = 0.0f;
    for (size_t i = 0; i < samples.size(); ++i) {
        i_sum += i_bb[i];
        q_sum += q_bb[i];
        if (i >= win) {
            i_sum -= i_bb[i - win];
            q_sum -= q_bb[i - win];
        }
        const size_t n = std::min(i + 1, win);
        i_filt[i] = i_sum / static_cast<float>(n);
        q_filt[i] = q_sum / static_cast<float>(n);
    }

    float phase = cfo_phase_;
    for (size_t i = 0; i < samples.size(); ++i) {
        const float t = static_cast<float>(i) / fs;
        const float mix_phase = 2.0f * kPi * fc * t;
        const float cfo_cos = std::cos(phase);
        const float cfo_sin = std::sin(phase);
        const float i_cfo = i_filt[i] * cfo_cos - q_filt[i] * cfo_sin;
        const float q_cfo = i_filt[i] * cfo_sin + q_filt[i] * cfo_cos;
        samples[i] =
            2.0f * (i_cfo * std::cos(mix_phase) - q_cfo * std::sin(mix_phase));

        phase += cfo_phase_inc_;
        if (phase > 2.0f * kPi) {
            phase -= 2.0f * kPi;
        }
    }
    cfo_phase_ = phase;
}

WattersonChannelModel::WattersonChannelModel(const WattersonChannel::Config& config,
                                             uint64_t seed)
    : channel_(config, seed) {}

void WattersonChannelModel::reset() {
    channel_.reset();
}

void WattersonChannelModel::setSNR(float snr_db) {
    channel_.setSNR(snr_db);
}

void WattersonChannelModel::process(std::span<const float> input,
                                    std::vector<float>& output) {
    channel_.process(input, output);
}

const WattersonChannel::Config& WattersonChannelModel::config() const {
    return channel_.config();
}

namespace itu_r_f1487 {

WattersonChannel::Config good(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 0.5f,
            .doppler_spread_hz = 0.1f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config moderate(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 1.0f,
            .doppler_spread_hz = 0.5f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config poor(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 2.0f,
            .doppler_spread_hz = 1.0f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config flutter(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 0.5f,
            .doppler_spread_hz = 10.0f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config awgn(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 0.0f,
            .doppler_spread_hz = 0.0f,
            .path1_gain = 1.0f,
            .path2_gain = 0.0f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = false,
            .multipath_enabled = false,
            .noise_enabled = true};
}

}  // namespace itu_r_f1487

WattersonChannel::Config configForWatterson(ChannelType type,
                                            float snr_db,
                                            uint32_t sample_rate) {
    WattersonChannel::Config cfg;
    switch (type) {
        case ChannelType::GOOD:
            cfg = itu_r_f1487::good(snr_db);
            break;
        case ChannelType::MODERATE:
            cfg = itu_r_f1487::moderate(snr_db);
            break;
        case ChannelType::POOR:
            cfg = itu_r_f1487::poor(snr_db);
            break;
        case ChannelType::FLUTTER:
            cfg = itu_r_f1487::flutter(snr_db);
            break;
        case ChannelType::AWGN:
            cfg = itu_r_f1487::awgn(snr_db);
            break;
        case ChannelType::PASSTHROUGH:
            cfg = itu_r_f1487::awgn(snr_db);
            cfg.noise_enabled = false;
            break;
        default:
            throw std::invalid_argument("unknown channel type");
    }
    cfg.sample_rate = sample_rate;
    return cfg;
}

std::unique_ptr<IChannelModel> createChannelModel(const ChannelConfig& config,
                                                  const RngRoot& rng_root,
                                                  std::string_view stream_name) {
    switch (config.type) {
        case ChannelType::PASSTHROUGH:
            return std::make_unique<PassthroughChannelModel>();
        case ChannelType::AWGN:
            return std::make_unique<AWGNChannelModel>(
                config.snr_db, rng_root.stream(stream_name));
        case ChannelType::GOOD:
        case ChannelType::MODERATE:
        case ChannelType::POOR:
        case ChannelType::FLUTTER: {
            auto cfg = configForWatterson(config.type, config.snr_db, config.sample_rate);
            cfg.cfo_hz = 0.0f;
            return std::make_unique<WattersonChannelModel>(
                cfg, rng_root.childSeed(stream_name));
        }
        default:
            throw std::invalid_argument("unknown channel type");
    }
}

}  // namespace ultra::ota_channel_core
