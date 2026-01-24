#pragma once

// ChirpSync - Linear FM chirp for robust HF presence detection
//
// Why chirp instead of Barker-13:
// - Spreads energy across entire bandwidth (300-2700 Hz)
// - Frequency-selective fades only affect part of the chirp
// - Matched filter integrates across time-frequency for robust detection
// - Proven technique used by industry-leading commercial HF modems
//
// Design:
// - Linear up-chirp: 300 Hz -> 2700 Hz over 500ms
// - Repeated 2-3 times with short gaps
// - Total duration: ~1.5 seconds
// - Detection via matched filter correlation

#include "ultra/types.hpp"
#include "ultra/dsp.hpp"
#include <vector>
#include <cmath>
#include <complex>

namespace ultra {
namespace sync {

struct ChirpConfig {
    float sample_rate = 48000.0f;
    float f_start = 300.0f;      // Start frequency (Hz)
    float f_end = 2700.0f;       // End frequency (Hz)
    float duration_ms = 500.0f;  // Chirp duration (ms)
    int repetitions = 2;         // Number of chirp repetitions
    float gap_ms = 100.0f;       // Gap between repetitions (ms)
    float amplitude = 0.8f;      // Output amplitude
};

class ChirpSync {
public:
    explicit ChirpSync(const ChirpConfig& config = ChirpConfig())
        : config_(config)
    {
        generateTemplate();
    }

    // Generate chirp signal for transmission
    // Returns: samples for complete chirp sequence (with repetitions)
    // NOTE: No lead-in silence - caller can add if needed
    Samples generate() const {
        size_t chirp_samples = static_cast<size_t>(config_.sample_rate * config_.duration_ms / 1000.0f);
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);

        // Total: [chirp1][gap][chirp2][gap]...
        size_t total = config_.repetitions * chirp_samples +
                       (config_.repetitions - 1) * gap_samples + gap_samples;

        Samples output(total, 0.0f);
        size_t offset = 0;

        for (int rep = 0; rep < config_.repetitions; rep++) {
            // Generate one chirp
            for (size_t i = 0; i < chirp_samples; i++) {
                float t = static_cast<float>(i) / config_.sample_rate;
                float phase = generateChirpPhase(t);
                output[offset + i] = config_.amplitude * std::sin(phase);
            }
            offset += chirp_samples + gap_samples;
        }

        return output;
    }

    // Detect chirp in received signal
    // Returns: sample offset of chirp start, or -1 if not found
    // Also returns peak correlation value via out parameter
    int detect(SampleSpan samples, float& peak_correlation, float threshold = 0.3f) const {
        if (samples.size() < chirp_template_.size()) {
            peak_correlation = 0.0f;
            return -1;
        }

        // Matched filter correlation
        size_t search_len = samples.size() - chirp_template_.size();
        float best_corr = 0.0f;
        int best_offset = -1;

        // Coarse search - step of 24 samples (0.5ms) for reliable detection
        // Chirp correlation is very narrow - even 48 samples offset drops to ~0.01
        constexpr size_t step = 24;

        for (size_t start = 0; start < search_len; start += step) {
            float corr = computeCorrelation(samples, start);
            if (corr > best_corr) {
                best_corr = corr;
                best_offset = static_cast<int>(start);
            }
        }

        // Fine search around best coarse position
        if (best_offset >= 0) {
            int fine_start = std::max(0, best_offset - static_cast<int>(step));
            int fine_end = std::min(static_cast<int>(search_len), best_offset + static_cast<int>(step));

            for (int start = fine_start; start < fine_end; start++) {
                float corr = computeCorrelation(samples, start);
                if (corr > best_corr) {
                    best_corr = corr;
                    best_offset = start;
                }
            }
        }

        // If we found a peak, search backward for an earlier repetition
        // (chirp signal may have multiple repetitions, we want the first)
        if (best_offset >= 0 && best_corr >= threshold) {
            size_t chirp_plus_gap = getChirpSamples() +
                static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);

            // Check one repetition earlier
            if (best_offset >= (int)chirp_plus_gap) {
                int earlier_center = best_offset - static_cast<int>(chirp_plus_gap);
                int search_start = std::max(0, earlier_center - static_cast<int>(step));
                int search_end = std::min(static_cast<int>(search_len),
                                          earlier_center + static_cast<int>(step));

                for (int start = search_start; start < search_end; start++) {
                    float corr = computeCorrelation(samples, start);
                    // Accept earlier peak if it's at least 90% as good
                    if (corr >= threshold && corr >= best_corr * 0.9f) {
                        best_corr = corr;
                        best_offset = start;
                        break;  // Found earlier peak, use it
                    }
                }
            }
        }

        peak_correlation = best_corr;

        if (best_corr >= threshold) {
            return best_offset;
        }
        return -1;
    }

    // Get the chirp template (for debugging/visualization)
    const Samples& getTemplate() const { return chirp_template_; }

    // Get expected chirp duration in samples
    size_t getChirpSamples() const {
        return static_cast<size_t>(config_.sample_rate * config_.duration_ms / 1000.0f);
    }

    // Get total signal duration in samples (including repetitions and gaps)
    // This matches the output of generate() exactly
    size_t getTotalSamples() const {
        size_t chirp_samples = getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.gap_ms / 1000.0f);
        return config_.repetitions * chirp_samples +
               (config_.repetitions - 1) * gap_samples + gap_samples;
    }

    const ChirpConfig& getConfig() const { return config_; }

private:
    ChirpConfig config_;
    Samples chirp_template_;
    float template_energy_ = 0.0f;  // Precomputed for normalization

    // Generate phase for linear chirp at time t
    // Linear FM: f(t) = f_start + (f_end - f_start) * t / T
    // Phase: φ(t) = 2π * integral(f(t)) = 2π * (f_start * t + (f_end - f_start) * t^2 / (2*T))
    float generateChirpPhase(float t) const {
        float T = config_.duration_ms / 1000.0f;
        float k = (config_.f_end - config_.f_start) / T;  // Chirp rate
        return 2.0f * M_PI * (config_.f_start * t + 0.5f * k * t * t);
    }

    void generateTemplate() {
        size_t chirp_samples = getChirpSamples();
        chirp_template_.resize(chirp_samples);

        template_energy_ = 0.0f;
        for (size_t i = 0; i < chirp_samples; i++) {
            float t = static_cast<float>(i) / config_.sample_rate;
            float phase = generateChirpPhase(t);
            chirp_template_[i] = std::sin(phase);
            template_energy_ += chirp_template_[i] * chirp_template_[i];
        }
    }

    // Compute normalized correlation at given offset
    float computeCorrelation(SampleSpan samples, size_t offset) const {
        float corr = 0.0f;
        float sig_energy = 0.0f;

        for (size_t i = 0; i < chirp_template_.size(); i++) {
            float s = samples[offset + i];
            float t = chirp_template_[i];
            corr += s * t;
            sig_energy += s * s;
        }

        float denom = std::sqrt(sig_energy * template_energy_);
        if (denom < 1e-10f) return 0.0f;

        return std::abs(corr) / denom;  // abs() because phase might be inverted
    }
};  // class ChirpSync

} // namespace sync
} // namespace ultra
