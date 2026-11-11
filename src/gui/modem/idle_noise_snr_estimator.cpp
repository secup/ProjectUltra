#include "idle_noise_snr_estimator.hpp"

#include "sim/channel_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace ultra {
namespace gui {

namespace {

double safePositive(double value, double fallback) {
    return value > 0.0 && std::isfinite(value) ? value : fallback;
}

}  // namespace

IdleNoiseSNREstimator::IdleNoiseSNREstimator()
    : IdleNoiseSNREstimator(Config{}) {}

IdleNoiseSNREstimator::IdleNoiseSNREstimator(const Config& config)
    : config_(config),
      filter_(FIRFilter::bandpass(
          static_cast<size_t>(std::max(3, config.filter_taps)),
          config.band_low_hz,
          config.band_high_hz,
          config.sample_rate)) {
    fir_energy_ = safePositive(coefficientEnergy(filter_), 1.0);
    equivalent_noise_bandwidth_hz_ = 0.5 * static_cast<double>(config_.sample_rate) *
                                     fir_energy_;
}

void IdleNoiseSNREstimator::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_.reset();
    window_sum_sq_ = 0.0;
    window_fill_ = 0;
    valid_ = false;
    smoothed_snr_db_ = 0.0f;
    latest_instant_snr_db_ = 0.0f;
    filtered_noise_rms_ = 0.0f;
    normalized_noise_rms_ = 0.0f;
    window_rms_ring_.fill(0.0f);
    windows_observed_ = 0;
}

void IdleNoiseSNREstimator::observeIdleAudio(const float* samples, size_t count) {
    if (!samples || count == 0 || config_.window_samples == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < count; ++i) {
        const float filtered = filter_.process(samples[i]);
        window_sum_sq_ += static_cast<double>(filtered) * static_cast<double>(filtered);
        ++window_fill_;

        if (window_fill_ < config_.window_samples) {
            continue;
        }

        const double filtered_power =
            window_sum_sq_ / static_cast<double>(config_.window_samples);

        // In-band SNR convention:
        //
        // The decoder and operator meter care about signal/noise inside the
        // receiver's actual modem bandwidth. The FIR output is therefore the
        // reference noise measurement:
        //
        //     P_noise_in_band = E{y^2}
        //
        // Earlier code divided filtered_power by sum_k h[k]^2 to extrapolate a
        // broadband-equivalent variance. That is only valid for white noise. On
        // real-HF noise with nearly all energy already in the modem passband,
        // the divide inflated the measured noise by about 1/sum(h^2), causing
        // the meter to under-read by roughly 9.6 dB. Keeping filtered_power here
        // reports the physically relevant in-band SNR for both white and
        // colored idle noise.
        //
        // Snapshot::normalized_noise_rms keeps its legacy field name for local
        // API stability, but now carries the in-band FIR-output RMS.
        const double normalized_noise_power = filtered_power;
        const double bounded_noise_power =
            std::max(normalized_noise_power, std::numeric_limits<double>::min());
        const float snr_db = static_cast<float>(
            10.0 * std::log10(sim::kModemReferencePower / bounded_noise_power));

        latest_instant_snr_db_ = snr_db;
        filtered_noise_rms_ = static_cast<float>(std::sqrt(std::max(0.0, filtered_power)));
        window_rms_ring_[windows_observed_ % kFloorWindowCount] = filtered_noise_rms_;
        normalized_noise_rms_ = static_cast<float>(std::sqrt(bounded_noise_power));
        if (!valid_) {
            smoothed_snr_db_ = snr_db;
            valid_ = true;
        } else {
            const float alpha = std::clamp(config_.ema_alpha, 0.0f, 1.0f);
            smoothed_snr_db_ = alpha * snr_db + (1.0f - alpha) * smoothed_snr_db_;
        }
        ++windows_observed_;

        window_sum_sq_ = 0.0;
        window_fill_ = 0;
    }
}

bool IdleNoiseSNREstimator::hasEstimate() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return valid_;
}

void IdleNoiseSNREstimator::setSignalReferencePower(double in_band_power) {
    if (!(in_band_power > 0.0) || !std::isfinite(in_band_power)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    signal_reference_power_ = in_band_power;
}

// The windows are scored against the simulator's reference power (a fixed convention, so the
// smoothing runs on a stable scale); the measured peer power then shifts the whole SNR by a
// constant, which commutes with the EMA.
static float referenceOffsetDb(double signal_reference_power) {
    return static_cast<float>(10.0 * std::log10(signal_reference_power / sim::kModemReferencePower));
}

float IdleNoiseSNREstimator::snrDb() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!(signal_reference_power_ > 0.0)) return std::numeric_limits<float>::quiet_NaN();
    return smoothed_snr_db_ + referenceOffsetDb(signal_reference_power_);
}

IdleNoiseSNREstimator::Snapshot IdleNoiseSNREstimator::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot out;
    out.valid = valid_;
    out.snr_valid = valid_ && signal_reference_power_ > 0.0;
    const float offset_db =
        signal_reference_power_ > 0.0 ? referenceOffsetDb(signal_reference_power_) : 0.0f;
    out.idle_in_band_snr_db = smoothed_snr_db_ + offset_db;
    out.latest_instant_idle_in_band_snr_db = latest_instant_snr_db_ + offset_db;
    out.filtered_noise_rms = filtered_noise_rms_;
    {
        // Min over the populated part of the ring (ignore empty slots).
        float floor = 0.0f;
        const size_t filled = std::min(windows_observed_, kFloorWindowCount);
        for (size_t i = 0; i < filled; ++i) {
            const float v = window_rms_ring_[i];
            if (v > 0.0f && (floor == 0.0f || v < floor)) floor = v;
        }
        out.floor_noise_rms = floor;
    }
    out.normalized_noise_rms = normalized_noise_rms_;
    out.fir_energy = fir_energy_;
    out.equivalent_noise_bandwidth_hz = equivalent_noise_bandwidth_hz_;
    out.windows_observed = windows_observed_;
    return out;
}

double IdleNoiseSNREstimator::coefficientEnergy(const FIRFilter& filter) {
    const auto& coeffs = filter.coefficients();
    return std::accumulate(coeffs.begin(), coeffs.end(), 0.0,
                           [](double sum, float h) {
                               return sum + static_cast<double>(h) *
                                            static_cast<double>(h);
                           });
}

}  // namespace gui
}  // namespace ultra
