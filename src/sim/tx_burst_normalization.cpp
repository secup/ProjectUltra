#include "ultra/tx_burst_normalization.hpp"

#include "ota_channel_core/models.hpp"
#include "ultra/logging.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ultra::sim {
namespace {

constexpr float kPi = 3.14159265358979323846f;

struct ActiveRegion {
    size_t begin = 0;
    size_t end = 0;
    size_t samples = 0;
    bool fragment_warning = false;
};

ActiveRegion detectActiveRegion(std::span<const float> samples) {
    ActiveRegion active;
    if (samples.empty()) {
        return active;
    }

    size_t begin = 0;
    while (begin < samples.size() &&
           std::abs(samples[begin]) <= kTxBurstActiveThreshold) {
        ++begin;
    }

    size_t end = samples.size();
    while (end > begin &&
           std::abs(samples[end - 1]) <= kTxBurstActiveThreshold) {
        --end;
    }

    active.begin = begin;
    active.end = end;
    active.samples = end > begin ? end - begin : 0;
    active.fragment_warning =
        active.samples > 0 && active.samples < kTxBurstMinimumActiveSamples;
    return active;
}

float maxAbsSample(std::span<const float> samples) {
    float peak = 0.0f;
    for (float sample : samples) {
        const float abs_sample = std::abs(sample);
        if (abs_sample > peak) {
            peak = abs_sample;
        }
    }
    return peak;
}

std::vector<float> makeReferenceBandFirCoefficients() {
    std::vector<float> coeffs(kReferenceBandFirTaps);
    const float fc_low = kReferenceBandLowHz / kReferenceSampleRateHz;
    const float fc_high = kReferenceBandHighHz / kReferenceSampleRateHz;
    const int midpoint = static_cast<int>((kReferenceBandFirTaps - 1) / 2);

    for (int n = 0; n < static_cast<int>(kReferenceBandFirTaps); ++n) {
        if (n == midpoint) {
            coeffs[static_cast<size_t>(n)] = 2.0f * (fc_high - fc_low);
        } else {
            const float x = kPi * static_cast<float>(n - midpoint);
            coeffs[static_cast<size_t>(n)] =
                (std::sin(2.0f * fc_high * x) -
                 std::sin(2.0f * fc_low * x)) / x;
        }
        const float w = 2.0f * kPi * static_cast<float>(n) /
                        static_cast<float>(kReferenceBandFirTaps - 1);
        coeffs[static_cast<size_t>(n)] *=
            0.42f - 0.5f * std::cos(w) + 0.08f * std::cos(2.0f * w);
    }
    return coeffs;
}

float processFirSample(float sample,
                       const std::vector<float>& coeffs,
                       std::vector<float>& delay_line,
                       size_t& delay_idx) {
    delay_line[delay_idx] = sample;

    float out = 0.0f;
    size_t j = delay_idx;
    for (size_t i = 0; i < coeffs.size(); ++i) {
        out += coeffs[i] * delay_line[j];
        if (j == 0) {
            j = coeffs.size();
        }
        --j;
    }

    delay_idx = (delay_idx + 1) % coeffs.size();
    return out;
}

void updatePeakStats(TxBurstRmsMeasurement& result,
                     std::span<const float> samples) {
    result.peak_after_gain = 0.0f;
    result.peak_warning_samples = 0;
    result.peak_clip_samples = 0;
    for (float sample : samples) {
        const float after_gain = std::abs(sample * result.gain_to_reference);
        result.peak_after_gain =
            std::max(result.peak_after_gain, after_gain);
        if (after_gain > kTxBurstPeakWarningThreshold) {
            ++result.peak_warning_samples;
        }
        if (after_gain > kTxBurstPeakClipThreshold) {
            ++result.peak_clip_samples;
        }
    }
    result.peak_warning = result.peak_warning_samples > 0;
    result.peak_clip_error = result.peak_clip_samples > 0;
}

}  // namespace

const std::vector<float>& referenceBandFirCoefficients() {
    static const std::vector<float> coeffs = makeReferenceBandFirCoefficients();
    return coeffs;
}

TxBurstRmsMeasurement measureTxBurstInBandRms(std::span<const float> samples) {
    TxBurstRmsMeasurement result;
    if (samples.empty()) {
        return result;
    }

    const ActiveRegion active = detectActiveRegion(samples);
    const size_t begin = active.begin;
    const size_t end = active.end;
    result.active_begin = active.begin;
    result.active_end = active.end;
    result.active_samples = active.samples;
    result.burst_fragment_warning = active.fragment_warning;

    if (result.active_samples == 0) {
        updatePeakStats(result, samples);
        return result;
    }

    const auto& coeffs = referenceBandFirCoefficients();
    std::vector<float> delay_line(coeffs.size(), 0.0f);
    size_t delay_idx = 0;
    double broadband_sum_sq = 0.0;
    double in_band_sum_sq = 0.0;
    constexpr size_t kTypicalWindow = 2400;  // 50 ms at 48 kHz
    std::vector<double> window_power;
    double window_sum = 0.0;
    size_t window_n = 0;

    for (size_t i = 0; i < end; ++i) {
        const float sample = samples[i];
        const float filtered =
            processFirSample(sample, coeffs, delay_line, delay_idx);
        if (i >= begin) {
            broadband_sum_sq += static_cast<double>(sample) *
                                static_cast<double>(sample);
            const double f2 = static_cast<double>(filtered) * static_cast<double>(filtered);
            in_band_sum_sq += f2;
            window_sum += f2;
            if (++window_n == kTypicalWindow) {
                window_power.push_back(window_sum / static_cast<double>(kTypicalWindow));
                window_sum = 0.0;
                window_n = 0;
            }
        }
    }
    if (!window_power.empty()) {
        // Near-silent windows (inter-chirp gap, lead-in/tail inside the active span) are not level.
        const double max_p = *std::max_element(window_power.begin(), window_power.end());
        std::vector<double> live;
        live.reserve(window_power.size());
        for (double p : window_power) {
            if (p > 1e-3 * max_p) live.push_back(p);
        }
        if (!live.empty()) {
            const auto mid = live.begin() + static_cast<std::ptrdiff_t>(live.size() / 2);
            std::nth_element(live.begin(), mid, live.end());
            result.in_band_typical_rms = static_cast<float>(std::sqrt(*mid));
        }
    }

    result.broadband_rms = static_cast<float>(
        std::sqrt(broadband_sum_sq / static_cast<double>(result.active_samples)));
    result.in_band_rms = static_cast<float>(
        std::sqrt(in_band_sum_sq / static_cast<double>(result.active_samples)));

    if (result.in_band_rms > std::numeric_limits<float>::min()) {
        result.gain_to_reference =
            ota_channel_core::kModemReferenceInBandRms / result.in_band_rms;
    }

    updatePeakStats(result, samples);
    return result;
}

TxBurstHardwareMeasurement normalizeTxBurstForHardware(std::vector<float>& samples,
                                                       float target_peak) {
    TxBurstHardwareMeasurement result;
    if (!std::isfinite(target_peak)) {
        target_peak = kHardwareTxDefaultPeakTarget;
    }
    // Upper clamp is the software-ALC ceiling (0.85), not the 0.7 operator ceiling:
    // the closed loop may legitimately request targets in (0.7, 0.85] — it is
    // guarded by the receiver's crest-factor clip detector (see
    // kSoftwareAlcMaxPeakTarget). Operator SETTINGS remain clamped to 0.7 upstream
    // (settings.cpp), so nothing but the ALC can reach this range.
    result.target_peak = std::clamp(target_peak,
                                    kHardwareTxMinPeakTarget,
                                    kSoftwareAlcMaxPeakTarget);

    const ActiveRegion active = detectActiveRegion(samples);
    result.active_begin = active.begin;
    result.active_end = active.end;
    result.active_samples = active.samples;
    result.burst_fragment_warning = active.fragment_warning;
    result.peak_before_gain = maxAbsSample(samples);
    result.peak_after_gain = result.peak_before_gain;

    if (samples.empty() || result.active_samples == 0) {
        return result;
    }

    if (result.burst_fragment_warning) {
        LOG_WARN("AUDIO",
                 "Hardware TX peak normalization bypassed fragment: active=%zu "
                 "minimum=%zu samples=%zu",
                 result.active_samples,
                 kTxBurstMinimumActiveSamples,
                 samples.size());
        return result;
    }

    if (!(result.peak_before_gain > 0.0f) ||
        !std::isfinite(result.peak_before_gain)) {
        return result;
    }

    result.gain_to_target = result.target_peak / result.peak_before_gain;
    if (!(result.gain_to_target > 0.0f) ||
        !std::isfinite(result.gain_to_target)) {
        result.gain_to_target = 1.0f;
        return result;
    }

    for (float& sample : samples) {
        sample *= result.gain_to_target;
    }
    result.peak_after_gain = maxAbsSample(samples);
    return result;
}

TxBurstRmsMeasurement normalizeTxBurstToReference(std::vector<float>& samples) {
    const TxBurstRmsMeasurement result = measureTxBurstInBandRms(samples);
    if (result.burst_fragment_warning) {
        LOG_WARN("SIM",
                 "TX burst normalization called on fragment: active=%zu "
                 "minimum=%zu samples=%zu",
                 result.active_samples,
                 kTxBurstMinimumActiveSamples,
                 samples.size());
    }
    if (samples.empty() || result.active_samples == 0 ||
        !(result.gain_to_reference > 0.0f) ||
        !std::isfinite(result.gain_to_reference)) {
        return result;
    }

    for (float& sample : samples) {
        sample *= result.gain_to_reference;
    }
    return result;
}

}  // namespace ultra::sim
