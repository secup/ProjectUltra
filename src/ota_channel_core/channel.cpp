#include "ota_channel_core/channel.hpp"

#define POCKETFFT_CACHE_SIZE 64
#if defined(__APPLE__) || defined(__unix__)
#define POCKETFFT_USE_POSIX_MEMALIGN
#endif

#include "pocketfft/pocketfft_hdronly.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <utility>

namespace ultra::ota_channel_core {
namespace {

using Complex = std::complex<float>;

constexpr float kPi = 3.14159265358979323846f;

float rms(std::span<const float> samples) {
    if (samples.empty()) {
        return 0.0f;
    }
    double sum_sq = 0.0;
    for (float sample : samples) {
        sum_sq += static_cast<double>(sample) * static_cast<double>(sample);
    }
    return static_cast<float>(std::sqrt(sum_sq / static_cast<double>(samples.size())));
}

}  // namespace

SimulatedChannel::SimulatedChannel() {
    rebuildModels();
}

void SimulatedChannel::setSeed(uint64_t seed) {
    seed_ = seed;
}

void SimulatedChannel::setTxCFO(float cfo_hz) {
    tx_cfo_hz_ = cfo_hz;
}

void SimulatedChannel::configure(float snr_db, ChannelType channel_type) {
    snr_db_ = snr_db;
    channel_type_ = channel_type;
    noise_stddev_ = channel_type == ChannelType::AWGN
        ? modemReferenceNoiseStddev(snr_db_)
        : 0.0f;
    cfo_phase_a_to_b_ = 0.0f;
    cfo_phase_b_to_a_ = 0.0f;
    {
        std::lock_guard<std::mutex> lock(rng_mutex_);
        awgn_rng_.seed(static_cast<uint32_t>(seed_));
    }
    rebuildModels();
}

void SimulatedChannel::setSignalCaptureEnabled(bool enabled) {
    capture_enabled_.store(enabled);
}

void SimulatedChannel::setSignalCaptureMaxSamples(size_t max_samples) {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    capture_max_samples_ = max_samples;
}

void SimulatedChannel::clearCapturedSignals() {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    captured_ = CapturedSignals{};
    captured_.max_samples = capture_max_samples_;
}

SimulatedChannel::CapturedSignals SimulatedChannel::getCapturedSignals() const {
    std::lock_guard<std::mutex> lock(capture_mutex_);
    return captured_;
}

void SimulatedChannel::setNoiseOverlay(std::vector<float> bed,
                                       bool loop,
                                       float target_rms) {
    if (!bed.empty() && target_rms > 0.0f) {
        const float current = rms(bed);
        if (current > 0.0f) {
            const float gain = target_rms / current;
            for (float& sample : bed) {
                sample *= gain;
            }
        }
    }

    noise_overlay_ = std::move(bed);
    noise_overlay_loop_ = loop;
    has_noise_overlay_ = !noise_overlay_.empty();
    noise_overlay_cursor_a_ = 0;
    noise_overlay_cursor_b_ = 0;
}

void SimulatedChannel::setRxBlackoutCallback(bool is_station_a,
                                             std::function<bool()> callback) {
    std::lock_guard<std::mutex> lock(rx_blackout_mutex_);
    if (is_station_a) {
        station_a_rx_blackout_ = std::move(callback);
    } else {
        station_b_rx_blackout_ = std::move(callback);
    }
}

void SimulatedChannel::transmitFromA(const std::vector<float>& samples) {
    auto with_cfo = applyTxCFO(samples, cfo_phase_a_to_b_);
    auto processed = applyChannel(with_cfo, *channel_a_to_b_);
    captureTxIfEnabled(samples, true);
    if (isStationBInRxBlackout()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_b_rx_);
    for (float sample : processed) {
        buffer_b_rx_.push(sample);
    }
}

void SimulatedChannel::transmitFromB(const std::vector<float>& samples) {
    auto with_cfo = applyTxCFO(samples, cfo_phase_b_to_a_);
    auto processed = applyChannel(with_cfo, *channel_b_to_a_);
    captureTxIfEnabled(samples, false);
    if (isStationAInRxBlackout()) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_a_rx_);
    for (float sample : processed) {
        buffer_a_rx_.push(sample);
    }
}

std::vector<float> SimulatedChannel::receiveForA(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_a_rx_);
    std::vector<float> result(count, 0.0f);
    for (size_t i = 0; i < count; ++i) {
        if (!buffer_a_rx_.empty()) {
            result[i] = buffer_a_rx_.front();
            buffer_a_rx_.pop();
        }
    }
    addReceiveNoise(result, true);
    applyOverlay(result, noise_overlay_cursor_a_);
    captureRxIfEnabled(result, true);
    return result;
}

std::vector<float> SimulatedChannel::receiveForB(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_b_rx_);
    std::vector<float> result(count, 0.0f);
    for (size_t i = 0; i < count; ++i) {
        if (!buffer_b_rx_.empty()) {
            result[i] = buffer_b_rx_.front();
            buffer_b_rx_.pop();
        }
    }
    addReceiveNoise(result, false);
    applyOverlay(result, noise_overlay_cursor_b_);
    captureRxIfEnabled(result, false);
    return result;
}

void SimulatedChannel::rebuildModels() {
    RngRoot root(seed_);

    ChannelConfig path_config{
        .type = channel_type_ == ChannelType::AWGN ? ChannelType::PASSTHROUGH : channel_type_,
        .snr_db = snr_db_,
        .seed = seed_,
        .sample_rate = kDefaultSampleRate};
    if (path_config.type == ChannelType::GOOD ||
        path_config.type == ChannelType::MODERATE ||
        path_config.type == ChannelType::POOR ||
        path_config.type == ChannelType::FLUTTER) {
        auto cfg = configForWatterson(path_config.type, path_config.snr_db,
                                      path_config.sample_rate);
        cfg.cfo_hz = 0.0f;
        channel_a_to_b_ = std::make_unique<WattersonChannelModel>(cfg, seed_);
        channel_b_to_a_ = std::make_unique<WattersonChannelModel>(cfg, seed_ + 1);
    } else {
        channel_a_to_b_ = createChannelModel(path_config, root, "path:a_to_b");
        channel_b_to_a_ = createChannelModel(path_config, root, "path:b_to_a");
    }
}

std::vector<float> SimulatedChannel::applyChannel(std::span<const float> samples,
                                                  IChannelModel& model) {
    std::vector<float> processed;
    model.process(samples, processed);
    return processed;
}

void SimulatedChannel::addReceiveNoise(std::vector<float>& samples, bool for_a) {
    (void)for_a;
    if (channel_type_ != ChannelType::AWGN || noise_stddev_ <= 0.0f) {
        return;
    }

    std::lock_guard<std::mutex> lock(rng_mutex_);
    for (float& sample : samples) {
        sample += noise_stddev_ * noise_dist_(awgn_rng_);
    }
}

void SimulatedChannel::applyOverlay(std::vector<float>& out, uint64_t& cursor) {
    if (!has_noise_overlay_ || noise_overlay_.empty()) {
        return;
    }

    for (float& sample : out) {
        if (cursor < noise_overlay_.size()) {
            sample += noise_overlay_[static_cast<size_t>(cursor)];
        } else if (noise_overlay_loop_) {
            sample += noise_overlay_[
                static_cast<size_t>(cursor % noise_overlay_.size())];
        }
        ++cursor;
    }
}

bool SimulatedChannel::isStationAInRxBlackout() const {
    std::function<bool()> callback;
    {
        std::lock_guard<std::mutex> lock(rx_blackout_mutex_);
        callback = station_a_rx_blackout_;
    }
    return callback && callback();
}

bool SimulatedChannel::isStationBInRxBlackout() const {
    std::function<bool()> callback;
    {
        std::lock_guard<std::mutex> lock(rx_blackout_mutex_);
        callback = station_b_rx_blackout_;
    }
    return callback && callback();
}

void SimulatedChannel::captureTxIfEnabled(std::span<const float> tx_raw, bool from_a) {
    if (!capture_enabled_.load()) {
        return;
    }

    std::lock_guard<std::mutex> lock(capture_mutex_);
    captured_.max_samples = capture_max_samples_;
    appendLimited(from_a ? captured_.a_tx_raw : captured_.b_tx_raw, tx_raw);
}

void SimulatedChannel::captureRxIfEnabled(std::span<const float> rx_raw, bool for_a) {
    if (!capture_enabled_.load()) {
        return;
    }

    std::lock_guard<std::mutex> lock(capture_mutex_);
    captured_.max_samples = capture_max_samples_;
    appendLimited(for_a ? captured_.a_rx_raw : captured_.b_rx_raw, rx_raw);
}

void SimulatedChannel::appendLimited(std::vector<float>& dst, std::span<const float> src) {
    if (src.empty()) {
        return;
    }

    if (capture_max_samples_ == 0) {
        dst.insert(dst.end(), src.begin(), src.end());
        return;
    }

    if (dst.size() >= capture_max_samples_) {
        captured_.truncated = true;
        return;
    }

    const size_t room = capture_max_samples_ - dst.size();
    const size_t take = std::min(room, src.size());
    dst.insert(dst.end(), src.begin(), src.begin() + static_cast<std::ptrdiff_t>(take));
    if (take < src.size()) {
        captured_.truncated = true;
    }
}

std::vector<float> SimulatedChannel::applyTxCFO(std::span<const float> samples,
                                                float& phase_acc) {
    std::vector<float> out(samples.begin(), samples.end());
    if (std::abs(tx_cfo_hz_) < 0.001f || out.empty()) {
        return out;
    }

    size_t fft_size = 1;
    while (fft_size < out.size()) {
        fft_size <<= 1;
    }

    std::vector<Complex> time(fft_size, Complex(0.0f, 0.0f));
    std::vector<Complex> freq(fft_size, Complex(0.0f, 0.0f));
    std::vector<Complex> analytic(fft_size, Complex(0.0f, 0.0f));
    for (size_t i = 0; i < out.size(); ++i) {
        time[i] = Complex(out[i], 0.0f);
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

    const float phase_inc = 2.0f * kPi * tx_cfo_hz_ /
                            static_cast<float>(kDefaultSampleRate);
    float phase = phase_acc;
    for (size_t i = 0; i < out.size(); ++i) {
        const Complex rot(std::cos(phase), std::sin(phase));
        out[i] = std::real(analytic[i] * rot);
        phase += phase_inc;
        if (phase > kPi) {
            phase -= 2.0f * kPi;
        } else if (phase < -kPi) {
            phase += 2.0f * kPi;
        }
    }
    phase_acc = phase;
    return out;
}

}  // namespace ultra::ota_channel_core
