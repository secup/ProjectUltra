#pragma once

#include "ultra/types.hpp"
#include <random>
#include <cmath>

namespace ultra {
namespace sim {

/**
 * HF Channel Simulator
 *
 * Models realistic HF propagation effects:
 * - AWGN (Additive White Gaussian Noise)
 * - Multipath (multiple delayed copies)
 * - Rayleigh/Rician fading
 * - Doppler spread
 *
 * Based on ITU-R F.1487 and Watterson model
 */
class HFChannel {
public:
    struct Config {
        // Noise
        float snr_db = 20.0f;           // Signal-to-noise ratio

        // Multipath (Watterson model)
        bool multipath_enabled = true;
        float path1_delay_ms = 0.0f;    // Direct path
        float path1_gain = 1.0f;
        float path2_delay_ms = 2.0f;    // Reflected path
        float path2_gain = 0.5f;

        // Fading
        bool fading_enabled = true;
        float doppler_spread_hz = 1.0f; // Typical HF: 0.1 - 2 Hz

        // Frequency offset (oscillator drift)
        float freq_offset_hz = 0.0f;

        // Sample rate
        uint32_t sample_rate = 48000;
    };

    explicit HFChannel(const Config& config, uint32_t seed = 42)
        : config_(config)
        , rng_(seed)
        , noise_dist_(0.0f, 1.0f)
        , phase_(0.0f)
        , fading_phase1_(0.0f)
        , fading_phase2_(0.0f)
    {
        // Initialize delay line for multipath
        size_t max_delay_samples = static_cast<size_t>(
            std::max(config.path1_delay_ms, config.path2_delay_ms)
            * config.sample_rate / 1000.0f + 10
        );
        delay_line_.resize(max_delay_samples, 0.0f);

        // Calculate noise standard deviation from SNR
        // SNR = 10 * log10(signal_power / noise_power)
        // Assuming signal power = 1
        noise_std_ = std::pow(10.0f, -config.snr_db / 20.0f);

        // Doppler increment per sample
        doppler_inc_ = 2.0f * M_PI * config.doppler_spread_hz / config.sample_rate;

        // Frequency offset increment
        freq_offset_inc_ = 2.0f * M_PI * config.freq_offset_hz / config.sample_rate;
    }

    // Process samples through the channel
    Samples process(SampleSpan input) {
        Samples output(input.size());

        for (size_t i = 0; i < input.size(); ++i) {
            float sample = input[i];

            // Store in delay line
            delay_line_[delay_idx_] = sample;

            float out = 0.0f;

            if (config_.multipath_enabled) {
                // Path 1 (direct)
                size_t delay1 = static_cast<size_t>(
                    config_.path1_delay_ms * config_.sample_rate / 1000.0f
                );
                size_t idx1 = (delay_idx_ + delay_line_.size() - delay1) % delay_line_.size();
                float gain1 = config_.path1_gain;

                if (config_.fading_enabled) {
                    // Rayleigh fading on each path
                    gain1 *= 0.5f + 0.5f * std::sin(fading_phase1_);
                    fading_phase1_ += doppler_inc_ * (1.0f + 0.1f * noise_dist_(rng_));
                }

                out += delay_line_[idx1] * gain1;

                // Path 2 (reflected)
                size_t delay2 = static_cast<size_t>(
                    config_.path2_delay_ms * config_.sample_rate / 1000.0f
                );
                size_t idx2 = (delay_idx_ + delay_line_.size() - delay2) % delay_line_.size();
                float gain2 = config_.path2_gain;

                if (config_.fading_enabled) {
                    gain2 *= 0.5f + 0.5f * std::sin(fading_phase2_);
                    fading_phase2_ += doppler_inc_ * 1.3f * (1.0f + 0.1f * noise_dist_(rng_));
                }

                out += delay_line_[idx2] * gain2;
            } else {
                out = sample;
            }

            // Apply frequency offset
            if (config_.freq_offset_hz != 0.0f) {
                float cos_p = std::cos(phase_);
                float sin_p = std::sin(phase_);
                // This is simplified - proper SSB frequency shift would need Hilbert transform
                out *= cos_p;
                phase_ += freq_offset_inc_;
                if (phase_ > 2 * M_PI) phase_ -= 2 * M_PI;
            }

            // Add AWGN
            out += noise_std_ * noise_dist_(rng_);

            output[i] = out;
            delay_idx_ = (delay_idx_ + 1) % delay_line_.size();
        }

        return output;
    }

    // Reset channel state
    void reset() {
        std::fill(delay_line_.begin(), delay_line_.end(), 0.0f);
        delay_idx_ = 0;
        phase_ = 0;
        fading_phase1_ = 0;
        fading_phase2_ = 0;
    }

    // Change SNR dynamically
    void setSNR(float snr_db) {
        config_.snr_db = snr_db;
        noise_std_ = std::pow(10.0f, -snr_db / 20.0f);
    }

private:
    Config config_;
    std::mt19937 rng_;
    std::normal_distribution<float> noise_dist_;

    std::vector<float> delay_line_;
    size_t delay_idx_ = 0;

    float noise_std_;
    float phase_;
    float doppler_inc_;
    float freq_offset_inc_;
    float fading_phase1_;
    float fading_phase2_;
};

// Preset channel conditions
namespace presets {

inline HFChannel::Config good_conditions() {
    return {
        .snr_db = 25.0f,
        .multipath_enabled = true,
        .path1_delay_ms = 0.0f,
        .path1_gain = 1.0f,
        .path2_delay_ms = 1.0f,
        .path2_gain = 0.3f,
        .fading_enabled = true,
        .doppler_spread_hz = 0.5f,
        .freq_offset_hz = 0.0f,
        .sample_rate = 48000
    };
}

inline HFChannel::Config moderate_conditions() {
    return {
        .snr_db = 15.0f,
        .multipath_enabled = true,
        .path1_delay_ms = 0.0f,
        .path1_gain = 1.0f,
        .path2_delay_ms = 2.0f,
        .path2_gain = 0.5f,
        .fading_enabled = true,
        .doppler_spread_hz = 1.0f,
        .freq_offset_hz = 0.5f,
        .sample_rate = 48000
    };
}

inline HFChannel::Config poor_conditions() {
    return {
        .snr_db = 8.0f,
        .multipath_enabled = true,
        .path1_delay_ms = 0.0f,
        .path1_gain = 1.0f,
        .path2_delay_ms = 4.0f,
        .path2_gain = 0.7f,
        .fading_enabled = true,
        .doppler_spread_hz = 2.0f,
        .freq_offset_hz = 1.0f,
        .sample_rate = 48000
    };
}

inline HFChannel::Config awgn_only(float snr_db) {
    return {
        .snr_db = snr_db,
        .multipath_enabled = false,
        .path1_delay_ms = 0.0f,
        .path1_gain = 1.0f,
        .path2_delay_ms = 0.0f,
        .path2_gain = 0.0f,
        .fading_enabled = false,
        .doppler_spread_hz = 0.0f,
        .freq_offset_hz = 0.0f,
        .sample_rate = 48000
    };
}

} // namespace presets

} // namespace sim
} // namespace ultra
