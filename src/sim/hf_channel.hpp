#pragma once

#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>

// M_PI fallback for MSVC (in case cmath was included earlier without _USE_MATH_DEFINES)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ultra/types.hpp"
#include <random>
#include <complex>
#include <deque>

namespace ultra {
namespace sim {

// Complex type for fading calculations
using Complex = std::complex<float>;

/**
 * Watterson HF Channel Model (ITU-R F.1487)
 *
 * Models realistic HF ionospheric propagation:
 * - Two independent Rayleigh-fading taps (multipath)
 * - Gaussian Doppler spectrum on each tap
 * - Configurable delay spread and Doppler spread
 * - AWGN noise
 *
 * This is the standard model used for HF modem testing.
 */
class WattersonChannel {
public:
    struct Config {
        // SNR in dB
        float snr_db = 15.0f;

        // Multipath delay spread (second path delay in ms)
        float delay_spread_ms = 2.0f;

        // Doppler spread in Hz (fading rate)
        // Typical HF: 0.1 Hz (quiet) to 2 Hz (disturbed)
        float doppler_spread_hz = 1.0f;

        // Carrier Frequency Offset in Hz (radio tuning error)
        // Typical HF: ±10-50 Hz from imperfect tuning
        // Set to 0 for no CFO, or use random_cfo_hz for random offset
        float cfo_hz = 0.0f;
        float random_cfo_max_hz = 0.0f;  // If >0, randomize CFO in ±this range

        // Path gains (usually equal for worst-case)
        float path1_gain = 0.707f;  // -3dB each for equal power
        float path2_gain = 0.707f;

        // Sample rate
        uint32_t sample_rate = 48000;

        // Enable/disable effects for testing
        bool fading_enabled = true;
        bool multipath_enabled = true;
        bool noise_enabled = true;
        bool cfo_enabled = true;
    };

    explicit WattersonChannel(const Config& config, uint32_t seed = 42)
        : config_(config)
        , rng_(seed)
        , gaussian_(0.0f, 1.0f)
    {
        // Calculate delay in samples
        delay_samples_ = static_cast<size_t>(
            config.delay_spread_ms * config.sample_rate / 1000.0f
        );

        // Initialize delay line
        delay_line_.resize(delay_samples_ + 1, 0.0f);

        // Noise standard deviation from SNR
        // SNR = 10*log10(Ps/Pn), assuming Ps = 1
        noise_std_ = std::pow(10.0f, -config.snr_db / 20.0f);

        // Fading filter coefficient for Gaussian Doppler spectrum
        // Using simple IIR lowpass to approximate Gaussian spectrum
        // Cutoff at Doppler spread frequency
        float normalized_doppler = config.doppler_spread_hz / config.sample_rate;
        fading_alpha_ = 1.0f - std::exp(-2.0f * M_PI * normalized_doppler);

        // Initialize fading states (complex for Rayleigh)
        fading1_ = std::complex<float>(1.0f, 0.0f);
        fading2_ = std::complex<float>(1.0f, 0.0f);

        // Initialize CFO
        cfo_phase_ = 0.0f;
        actual_cfo_hz_ = config.cfo_hz;
        if (config.random_cfo_max_hz > 0.0f) {
            std::uniform_real_distribution<float> cfo_dist(-config.random_cfo_max_hz, config.random_cfo_max_hz);
            actual_cfo_hz_ = cfo_dist(rng_);
        }
        cfo_phase_inc_ = 2.0f * M_PI * actual_cfo_hz_ / config.sample_rate;
    }

    // Process samples through the channel
    // CRITICAL: Input signal should be normalized to unit RMS for correct SNR
    Samples process(SampleSpan input) {
        Samples output(input.size());

        // Calculate input RMS for SNR normalization
        float input_power = 0.0f;
        for (size_t i = 0; i < input.size(); ++i) {
            input_power += input[i] * input[i];
        }
        float input_rms = std::sqrt(input_power / input.size());

        // Normalize noise to achieve correct SNR relative to actual signal power
        // SNR = 10*log10(Ps/Pn), so Pn = Ps * 10^(-SNR/10)
        // noise_std = sqrt(Pn) = input_rms * 10^(-SNR/20)
        float effective_noise_std = input_rms * std::pow(10.0f, -config_.snr_db / 20.0f);

        for (size_t i = 0; i < input.size(); ++i) {
            float sample = input[i];

            // Update fading coefficients (Rayleigh fading)
            if (config_.fading_enabled) {
                updateFading();
            }

            float out = 0.0f;

            if (config_.multipath_enabled && delay_samples_ > 0) {
                // Two-tap multipath with independent Rayleigh fading
                // For real-valued signal, apply fading MAGNITUDE (Rayleigh envelope)
                // Phase effects are captured by OFDM pilot-based channel estimation
                float h1_mag = config_.fading_enabled ? std::abs(fading1_) : 1.0f;
                float h2_mag = config_.fading_enabled ? std::abs(fading2_) : 1.0f;

                // Path 1: direct path (no delay)
                out += sample * config_.path1_gain * h1_mag;

                // Path 2: delayed path
                float delayed = delay_line_.front();
                delay_line_.pop_front();
                delay_line_.push_back(sample);

                out += delayed * config_.path2_gain * h2_mag;
            } else {
                // No multipath - just fading on single path
                float h_mag = config_.fading_enabled ? std::abs(fading1_) : 1.0f;
                out = sample * h_mag;
            }

            // Add AWGN with SNR relative to actual signal power
            if (config_.noise_enabled) {
                out += effective_noise_std * gaussian_(rng_);
            }

            // Apply Carrier Frequency Offset (frequency shift)
            // For real signal: multiply by cos(2*pi*cfo*t) shifts spectrum
            // This simulates radio tuning error
            if (config_.cfo_enabled && std::abs(actual_cfo_hz_) > 0.001f) {
                out *= std::cos(cfo_phase_);
                cfo_phase_ += cfo_phase_inc_;
                // Keep phase in reasonable range to avoid precision loss
                if (cfo_phase_ > 2.0f * M_PI) {
                    cfo_phase_ -= 2.0f * M_PI;
                }
            }

            output[i] = out;
        }

        return output;
    }

    // Get the actual CFO being applied (useful when random)
    float getActualCFO() const { return actual_cfo_hz_; }

    // Get current fading magnitude (for monitoring)
    float getFadingMagnitude() const {
        return std::abs(fading1_);
    }

    // Reset channel state
    void reset() {
        std::fill(delay_line_.begin(), delay_line_.end(), 0.0f);
        fading1_ = std::complex<float>(1.0f, 0.0f);
        fading2_ = std::complex<float>(1.0f, 0.0f);
    }

    // Change SNR dynamically
    void setSNR(float snr_db) {
        config_.snr_db = snr_db;
        noise_std_ = std::pow(10.0f, -snr_db / 20.0f);
    }

    const Config& getConfig() const { return config_; }

private:
    void updateFading() {
        // Generate complex Gaussian noise (for Rayleigh fading)
        // For IIR lowpass: y[n] = (1-α)·y[n-1] + α·x[n]
        // Output power ≈ (α/2) × input_power for α << 1
        // For unit output power, we need input_variance = 2/α
        // Each complex component needs variance = 1/α, so std = sqrt(1/α)
        float noise_scale = std::sqrt(1.0f / fading_alpha_);
        std::complex<float> noise1(noise_scale * gaussian_(rng_), noise_scale * gaussian_(rng_));
        std::complex<float> noise2(noise_scale * gaussian_(rng_), noise_scale * gaussian_(rng_));

        // IIR lowpass filter to shape Doppler spectrum
        // This gives approximately Gaussian-shaped spectrum
        // Output is complex Gaussian with Rayleigh magnitude distribution
        fading1_ = (1.0f - fading_alpha_) * fading1_ + fading_alpha_ * noise1;
        fading2_ = (1.0f - fading_alpha_) * fading2_ + fading_alpha_ * noise2;

        // No per-sample normalization! The noise scaling above ensures proper power
    }

    Config config_;
    std::mt19937 rng_;
    std::normal_distribution<float> gaussian_;

    std::deque<float> delay_line_;
    size_t delay_samples_;
    float noise_std_;
    float fading_alpha_;

    std::complex<float> fading1_;
    std::complex<float> fading2_;

    // CFO state
    float cfo_phase_;
    float cfo_phase_inc_;
    float actual_cfo_hz_;
};

/**
 * ITU-R / CCIR Standard HF Channel Conditions
 *
 * Based on ITU-R F.1487 and MIL-STD-188-110
 */
namespace ccir {

// CCIR Good (benign mid-latitude, quiet conditions)
// - Low delay spread, slow fading
inline WattersonChannel::Config good(float snr_db = 25.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 0.5f,       // 0.5 ms multipath
        .doppler_spread_hz = 0.1f,     // Very slow fading
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// CCIR Moderate (typical mid-latitude)
// - Moderate delay spread, moderate fading
inline WattersonChannel::Config moderate(float snr_db = 15.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 1.0f,       // 1 ms multipath
        .doppler_spread_hz = 0.5f,     // Moderate fading
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// CCIR Poor (disturbed conditions, high-latitude)
// - Large delay spread, fast fading
inline WattersonChannel::Config poor(float snr_db = 10.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 2.0f,       // 2 ms multipath
        .doppler_spread_hz = 1.0f,     // Fast fading
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// CCIR Flutter (extreme flutter fading, auroral/polar paths)
// - Low delay spread, VERY fast fading (ITU-R F.1487)
// Note: Flutter is characterized by rapid fading, not large delay spread
inline WattersonChannel::Config flutter(float snr_db = 8.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 0.5f,       // 0.5 ms multipath (same as Good)
        .doppler_spread_hz = 10.0f,    // 10 Hz - very fast fading (auroral)
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// AWGN only (for baseline comparison)
inline WattersonChannel::Config awgn(float snr_db = 15.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 0.0f,
        .doppler_spread_hz = 0.0f,
        .path1_gain = 1.0f,
        .path2_gain = 0.0f,
        .sample_rate = 48000,
        .fading_enabled = false,
        .multipath_enabled = false,
        .noise_enabled = true
    };
}

// Print channel condition summary
inline void printConfig(const WattersonChannel::Config& cfg, const char* name) {
    printf("  %s: SNR=%.0fdB, delay=%.1fms, doppler=%.1fHz\n",
           name, cfg.snr_db, cfg.delay_spread_ms, cfg.doppler_spread_hz);
}

} // namespace ccir

/**
 * ITU-R F.1487 Standard Channel Conditions
 *
 * These are the exact parameters specified in ITU-R Recommendation F.1487
 * "Testing of HF modems with bandwidths of up to about 12 kHz using
 * ionospheric channel simulators"
 *
 * Used by MIL-STD-188-110 and STANAG for standardized modem testing.
 *
 * Reference: ITU-R F.1487-0 (2000)
 */
namespace itu_r_f1487 {

// ITU-R F.1487 Good (mid-latitude, quiet conditions)
// 2-path Rayleigh, equal power, Gaussian Doppler spectrum
inline WattersonChannel::Config good(float snr_db = 20.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 0.5f,       // 0.5 ms differential delay
        .doppler_spread_hz = 0.1f,     // 0.1 Hz Doppler spread (2σ)
        .path1_gain = 0.707f,          // Equal power paths (-3dB each)
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// ITU-R F.1487 Moderate (mid-latitude, normal conditions)
inline WattersonChannel::Config moderate(float snr_db = 20.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 1.0f,       // 1.0 ms differential delay
        .doppler_spread_hz = 0.5f,     // 0.5 Hz Doppler spread
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// ITU-R F.1487 Poor (disturbed, high-latitude)
inline WattersonChannel::Config poor(float snr_db = 20.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 2.0f,       // 2.0 ms differential delay
        .doppler_spread_hz = 1.0f,     // 1.0 Hz Doppler spread
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// ITU-R F.1487 Flutter (auroral/polar paths)
// Characterized by rapid fading, NOT large delay spread
inline WattersonChannel::Config flutter(float snr_db = 20.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 0.5f,       // 0.5 ms (same as Good)
        .doppler_spread_hz = 10.0f,    // 10 Hz - very rapid fading
        .path1_gain = 0.707f,
        .path2_gain = 0.707f,
        .sample_rate = 48000,
        .fading_enabled = true,
        .multipath_enabled = true,
        .noise_enabled = true
    };
}

// AWGN only (baseline reference)
inline WattersonChannel::Config awgn(float snr_db = 20.0f) {
    return {
        .snr_db = snr_db,
        .delay_spread_ms = 0.0f,
        .doppler_spread_hz = 0.0f,
        .path1_gain = 1.0f,
        .path2_gain = 0.0f,
        .sample_rate = 48000,
        .fading_enabled = false,
        .multipath_enabled = false,
        .noise_enabled = true
    };
}

// Channel condition names for reporting
inline const char* getConditionName(int idx) {
    static const char* names[] = {"AWGN", "Good", "Moderate", "Poor", "Flutter"};
    return (idx >= 0 && idx < 5) ? names[idx] : "Unknown";
}

} // namespace itu_r_f1487

// Convenience alias
using HFChannel = WattersonChannel;

} // namespace sim
} // namespace ultra
