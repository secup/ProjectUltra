#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "ultra/adaptive_modem.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/otfs.hpp"
#include "ultra/dsp.hpp"
#include <algorithm>
#include <numeric>

namespace ultra {

// ============================================================================
// Mode selection logic
// ============================================================================

ModulationMode selectMode(float delay_spread_ms, float doppler_spread_hz) {
    // Based on ITU-R F.1487 benchmark results at 15 dB SNR:
    //   Good (0.5ms, 0.1Hz): OTFS-EQ 90% vs OFDM 66%
    //   Moderate (1.0ms, 0.5Hz): OFDM 82% vs OTFS 42%
    //   Poor (2.0ms, 1.0Hz): OTFS-RAW 20% vs OFDM 10%
    //   Flutter (0.5ms, 10Hz): All fail

    if (doppler_spread_hz >= 5.0f) {
        // Flutter: Nothing works well, OFDM is most robust fallback
        return ModulationMode::OFDM;
    }

    if (delay_spread_ms >= 1.5f) {
        // Poor channel: High delay spread
        if (doppler_spread_hz >= 0.5f) {
            // High Doppler too - OTFS diversity without stale estimates
            return ModulationMode::OTFS_RAW;
        } else {
            // High delay but slow fading - OTFS-EQ can work
            return ModulationMode::OTFS_EQ;
        }
    }

    if (doppler_spread_hz >= 0.3f) {
        // Moderate Doppler: OFDM's sweet spot (per-symbol tracking)
        return ModulationMode::OFDM;
    }

    // Good channel: Stable, OTFS-EQ wins
    return ModulationMode::OTFS_EQ;
}

const char* modeToString(ModulationMode mode) {
    switch (mode) {
        case ModulationMode::OFDM: return "OFDM";
        case ModulationMode::OTFS_EQ: return "OTFS-EQ";
        case ModulationMode::OTFS_RAW: return "OTFS-RAW";
        case ModulationMode::AUTO: return "AUTO";
        default: return "Unknown";
    }
}

// ============================================================================
// Channel Characterizer Implementation
// ============================================================================

struct ChannelCharacterizer::Impl {
    Config config;
    FFT fft;
    NCO mixer;

    Impl(const Config& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {}

    std::vector<Complex> toBaseband(const float* samples, size_t count, size_t offset = 0) {
        std::vector<Complex> baseband(count);
        NCO temp_mixer(config.center_freq, config.sample_rate);
        for (size_t i = 0; i < offset; ++i) temp_mixer.next();

        for (size_t i = 0; i < count; ++i) {
            Complex carrier = temp_mixer.next();
            baseband[i] = Complex(samples[i], 0) * std::conj(carrier);
        }
        return baseband;
    }
};

ChannelCharacterizer::ChannelCharacterizer(const Config& config)
    : impl_(std::make_unique<Impl>(config)) {}

ChannelCharacterizer::~ChannelCharacterizer() = default;

PreambleChannelEstimate ChannelCharacterizer::characterize(
    const float* preamble_samples,
    size_t num_samples,
    const std::vector<Complex>& known_sequence
) {
    PreambleChannelEstimate result;
    const auto& cfg = impl_->config;

    size_t sym_len = cfg.fft_size + cfg.cp_length;
    size_t num_symbols = std::min(cfg.preamble_symbols, static_cast<uint32_t>(num_samples / sym_len));

    if (num_symbols < 2) {
        // Not enough preamble for estimation
        return result;
    }

    // Extract channel estimates from each preamble symbol
    std::vector<std::vector<Complex>> H_snapshots;
    float total_signal_power = 0;
    float total_noise_power = 0;
    int noise_samples = 0;

    for (size_t sym = 0; sym < num_symbols; ++sym) {
        size_t sym_offset = sym * sym_len;
        auto baseband = impl_->toBaseband(preamble_samples + sym_offset, sym_len, sym_offset);

        // FFT (skip CP)
        std::vector<Complex> time_domain(baseband.begin() + cfg.cp_length,
                                          baseband.begin() + cfg.cp_length + cfg.fft_size);
        std::vector<Complex> freq_domain;
        impl_->fft.forward(time_domain, freq_domain);

        // Extract channel estimate H(f) = Y(f) / X(f)
        std::vector<Complex> H_est(cfg.num_subcarriers);
        for (size_t m = 0; m < cfg.num_subcarriers && m < known_sequence.size(); ++m) {
            size_t idx = m + 1;  // Skip DC
            if (idx < cfg.fft_size / 2) {
                // SSB scaling: compensate for Hilbert transform gain in upper sideband
                // Factor of ~2.4 accounts for: 2x from analytic signal + filter rolloff
                Complex received = freq_domain[idx] * 2.4f;
                Complex expected = known_sequence[m];
                float expected_mag_sq = std::norm(expected);

                if (expected_mag_sq > 0.01f) {
                    H_est[m] = received * std::conj(expected) / expected_mag_sq;
                    total_signal_power += std::norm(received);

                    // Estimate noise from residual
                    Complex error = received - H_est[m] * expected;
                    total_noise_power += std::norm(error);
                    noise_samples++;
                } else {
                    H_est[m] = Complex(1, 0);
                }
            }
        }
        H_snapshots.push_back(H_est);
    }

    // Estimate SNR
    if (noise_samples > 0 && total_noise_power > 0) {
        float signal_power_avg = total_signal_power / noise_samples;
        float noise_power_avg = total_noise_power / noise_samples;
        result.snr_db = 10.0f * std::log10(signal_power_avg / noise_power_avg);
        result.snr_db = std::max(0.0f, std::min(50.0f, result.snr_db));
    }

    // Estimate delay spread from averaged H(f)
    std::vector<Complex> H_avg(cfg.num_subcarriers, Complex(0, 0));
    for (const auto& H : H_snapshots) {
        for (size_t m = 0; m < cfg.num_subcarriers && m < H.size(); ++m) {
            H_avg[m] += H[m];
        }
    }
    for (auto& h : H_avg) h /= static_cast<float>(H_snapshots.size());

    result.delay_spread_ms = estimateDelaySpread(H_avg);

    // Estimate Doppler from symbol-to-symbol variation
    float symbol_duration_ms = static_cast<float>(sym_len) / cfg.sample_rate * 1000.0f;
    result.doppler_spread_hz = estimateDopplerSpread(H_snapshots, symbol_duration_ms);

    // Coherence time (approximately 1 / (2 * Doppler))
    if (result.doppler_spread_hz > 0.01f) {
        result.coherence_time_ms = 500.0f / result.doppler_spread_hz;
    } else {
        result.coherence_time_ms = 10000.0f;  // Very long for static channel
    }

    return result;
}

float ChannelCharacterizer::estimateDelaySpread(const std::vector<Complex>& H_freq) {
    const auto& cfg = impl_->config;

    if (H_freq.size() < 4) return 0.0f;

    // Pad H(f) to FFT size for better resolution
    std::vector<Complex> H_padded(cfg.fft_size, Complex(0, 0));
    for (size_t i = 0; i < H_freq.size() && i < cfg.fft_size / 2; ++i) {
        H_padded[i] = H_freq[i];
    }

    // IFFT to get impulse response h(τ)
    std::vector<Complex> h_time;
    impl_->fft.inverse(H_padded, h_time);

    // Calculate RMS delay spread
    // τ_rms = sqrt(E[τ²] - E[τ]²) where expectation is weighted by |h(τ)|²
    float total_power = 0;
    float weighted_delay = 0;
    float weighted_delay_sq = 0;

    float sample_period_ms = 1000.0f / cfg.sample_rate;

    // Only consider first half (positive delays) and first few ms
    size_t max_delay_samples = static_cast<size_t>(5.0f / sample_period_ms);  // 5ms max
    max_delay_samples = std::min(max_delay_samples, h_time.size() / 2);

    for (size_t i = 0; i < max_delay_samples; ++i) {
        float power = std::norm(h_time[i]);
        float delay_ms = i * sample_period_ms;

        total_power += power;
        weighted_delay += power * delay_ms;
        weighted_delay_sq += power * delay_ms * delay_ms;
    }

    if (total_power < 1e-10f) return 0.0f;

    float mean_delay = weighted_delay / total_power;
    float rms_delay_spread = std::sqrt(
        std::max(0.0f, weighted_delay_sq / total_power - mean_delay * mean_delay)
    );

    return rms_delay_spread;
}

float ChannelCharacterizer::estimateDopplerSpread(
    const std::vector<std::vector<Complex>>& H_snapshots,
    float symbol_duration_ms
) {
    if (H_snapshots.size() < 2) return 0.0f;

    size_t num_subcarriers = H_snapshots[0].size();
    if (num_subcarriers == 0) return 0.0f;

    // Calculate average channel variation between consecutive symbols
    float total_variation = 0;
    float total_power = 0;
    int count = 0;

    for (size_t sym = 0; sym < H_snapshots.size() - 1; ++sym) {
        for (size_t m = 0; m < num_subcarriers; ++m) {
            if (m < H_snapshots[sym].size() && m < H_snapshots[sym + 1].size()) {
                Complex h1 = H_snapshots[sym][m];
                Complex h2 = H_snapshots[sym + 1][m];

                // Normalized variation: |h2 - h1| / |h1|
                float h1_mag = std::abs(h1);
                if (h1_mag > 0.1f) {
                    Complex diff = h2 - h1;
                    total_variation += std::norm(diff);
                    total_power += std::norm(h1);
                    count++;
                }
            }
        }
    }

    if (count == 0 || total_power < 1e-10f) return 0.0f;

    // Normalized RMS variation per symbol
    float rms_variation = std::sqrt(total_variation / total_power);

    // Convert to Doppler spread estimate
    // For Rayleigh fading with Gaussian Doppler spectrum:
    //   E[|ΔH|²] ≈ 2 * (1 - J₀(2π * fD * T))
    // For small fD*T: E[|ΔH|²] ≈ (2π * fD * T)²
    // So: fD ≈ sqrt(E[|ΔH|²]) / (2π * T)

    float T_seconds = symbol_duration_ms / 1000.0f;
    float doppler_hz = rms_variation / (2.0f * M_PI * T_seconds);

    // Clamp to reasonable range
    return std::max(0.0f, std::min(20.0f, doppler_hz));
}

// ============================================================================
// Adaptive Modem Implementation
// ============================================================================

struct AdaptiveModem::Impl {
    Config config;
    ModulationMode requested_mode = ModulationMode::AUTO;
    ModulationMode active_mode = ModulationMode::OFDM;
    PreambleChannelEstimate last_estimate;
    ModeChangeCallback mode_callback;

    // Underlying modems
    std::unique_ptr<OFDMModulator> ofdm_mod;
    std::unique_ptr<OFDMDemodulator> ofdm_demod;
    std::unique_ptr<OTFSModulator> otfs_mod;
    std::unique_ptr<OTFSDemodulator> otfs_demod;

    // Channel characterizer
    std::unique_ptr<ChannelCharacterizer> channel_characterizer;

    // Preamble sync sequence (Zadoff-Chu)
    std::vector<Complex> sync_sequence;

    // State
    bool synced = false;
    std::vector<float> last_soft_bits;

    Impl(const Config& cfg) : config(cfg) {
        // Initialize OFDM modems
        ModemConfig ofdm_cfg;
        ofdm_cfg.sample_rate = cfg.sample_rate;
        ofdm_cfg.fft_size = cfg.fft_size;
        ofdm_cfg.center_freq = cfg.center_freq;
        ofdm_cfg.num_carriers = cfg.num_carriers;
        ofdm_cfg.pilot_spacing = cfg.pilot_spacing;
        ofdm_cfg.cp_mode = cfg.cp_mode;
        ofdm_cfg.adaptive_eq_enabled = cfg.adaptive_eq_enabled;
        ofdm_cfg.lms_mu = cfg.lms_mu;

        ofdm_mod = std::make_unique<OFDMModulator>(ofdm_cfg);
        ofdm_demod = std::make_unique<OFDMDemodulator>(ofdm_cfg);

        // Initialize OTFS modems
        OTFSConfig otfs_cfg;
        otfs_cfg.M = cfg.otfs_M;
        otfs_cfg.N = cfg.otfs_N;
        otfs_cfg.fft_size = cfg.fft_size;
        otfs_cfg.cp_length = cfg.otfs_cp;
        otfs_cfg.sample_rate = cfg.sample_rate;
        otfs_cfg.center_freq = cfg.center_freq;
        otfs_cfg.tf_equalization = true;  // Will be set per-frame

        otfs_mod = std::make_unique<OTFSModulator>(otfs_cfg);

        // Initialize channel characterizer
        ChannelCharacterizer::Config est_cfg;
        est_cfg.fft_size = cfg.fft_size;
        est_cfg.cp_length = cfg.otfs_cp;
        est_cfg.sample_rate = cfg.sample_rate;
        est_cfg.center_freq = cfg.center_freq;
        est_cfg.num_subcarriers = cfg.otfs_M;
        est_cfg.preamble_symbols = 4;

        channel_characterizer = std::make_unique<ChannelCharacterizer>(est_cfg);

        // Generate sync sequence (Zadoff-Chu)
        generateSyncSequence();

        requested_mode = cfg.default_mode;
    }

    void generateSyncSequence() {
        size_t N = config.otfs_M;
        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }
    }

    ModulationMode selectActiveMode(const PreambleChannelEstimate& estimate) {
        if (requested_mode != ModulationMode::AUTO) {
            return requested_mode;
        }
        return estimate.getRecommendedMode();
    }

    void switchMode(ModulationMode new_mode, const PreambleChannelEstimate& estimate) {
        if (new_mode != active_mode) {
            ModulationMode old_mode = active_mode;
            active_mode = new_mode;

            if (mode_callback) {
                mode_callback(old_mode, new_mode, estimate);
            }
        }
    }

    OTFSDemodulator* createOTFSDemod(bool tf_eq) {
        OTFSConfig otfs_cfg;
        otfs_cfg.M = config.otfs_M;
        otfs_cfg.N = config.otfs_N;
        otfs_cfg.fft_size = config.fft_size;
        otfs_cfg.cp_length = config.otfs_cp;
        otfs_cfg.sample_rate = config.sample_rate;
        otfs_cfg.center_freq = config.center_freq;
        otfs_cfg.tf_equalization = tf_eq;

        return new OTFSDemodulator(otfs_cfg);
    }
};

AdaptiveModem::AdaptiveModem(const Config& config)
    : impl_(std::make_unique<Impl>(config)) {}

AdaptiveModem::~AdaptiveModem() = default;

void AdaptiveModem::setMode(ModulationMode mode) {
    impl_->requested_mode = mode;
}

ModulationMode AdaptiveModem::getMode() const {
    return impl_->requested_mode;
}

ModulationMode AdaptiveModem::getActiveMode() const {
    return impl_->active_mode;
}

Samples AdaptiveModem::generatePreamble() {
    // Use OTFS preamble (works for both modes)
    return impl_->otfs_mod->generatePreamble();
}

Samples AdaptiveModem::modulate(const Bytes& data, Modulation mod) {
    // TX mode is determined by requested_mode (not adaptive)
    // The receiver will adaptively select demodulation mode

    if (impl_->requested_mode == ModulationMode::OFDM ||
        impl_->requested_mode == ModulationMode::AUTO) {
        // For AUTO, default to OFDM on TX (most compatible)
        return impl_->ofdm_mod->modulate(ByteSpan(data.data(), data.size()), mod);
    } else {
        // OTFS modulation
        auto dd_symbols = impl_->otfs_mod->mapToDD(ByteSpan(data.data(), data.size()), mod);
        return impl_->otfs_mod->modulate(dd_symbols, mod);
    }
}

bool AdaptiveModem::process(SampleSpan samples) {
    // For adaptive mode, we need to:
    // 1. Detect preamble
    // 2. Estimate channel from preamble
    // 3. Select mode based on channel
    // 4. Demodulate with selected mode

    // Use the active mode's demodulator (set via setMode() before receiving)
    // Future: could auto-detect mode from preamble characteristics

    bool ready = false;

    switch (impl_->active_mode) {
        case ModulationMode::OFDM:
            ready = impl_->ofdm_demod->process(samples);
            if (ready) {
                impl_->last_soft_bits = impl_->ofdm_demod->getSoftBits();
                impl_->synced = true;
            }
            break;

        case ModulationMode::OTFS_EQ:
        case ModulationMode::OTFS_RAW: {
            bool tf_eq = (impl_->active_mode == ModulationMode::OTFS_EQ);
            impl_->otfs_demod.reset(impl_->createOTFSDemod(tf_eq));
            ready = impl_->otfs_demod->process(samples);
            if (ready) {
                impl_->last_soft_bits = impl_->otfs_demod->getSoftBits();
                impl_->synced = true;
            }
            break;
        }

        default:
            break;
    }

    return ready;
}

std::vector<float> AdaptiveModem::getSoftBits() {
    return impl_->last_soft_bits;
}

void AdaptiveModem::reset() {
    impl_->ofdm_demod->reset();
    if (impl_->otfs_demod) {
        impl_->otfs_demod->reset();
    }
    impl_->synced = false;
    impl_->last_soft_bits.clear();
}

PreambleChannelEstimate AdaptiveModem::getLastChannelEstimate() const {
    return impl_->last_estimate;
}

bool AdaptiveModem::isSynced() const {
    return impl_->synced;
}

void AdaptiveModem::setModeChangeCallback(ModeChangeCallback callback) {
    impl_->mode_callback = std::move(callback);
}

} // namespace ultra
