#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include <atomic>
#include <mutex>

// LDPC codeword size (same for all code rates in this implementation)
static constexpr size_t LDPC_BLOCK_SIZE = 648;

namespace ultra {

// Soft demapping - returns LLRs (log-likelihood ratios)
namespace {

float softDemapBPSK(Complex sym, float noise_var) {
    // BPSK: -1 maps to bit 0, +1 maps to bit 1
    // LLR convention: negative = bit 1, positive = bit 0
    // So: positive symbol → bit 1 → need negative LLR
    return -2.0f * sym.real() / noise_var;
}

std::vector<float> softDemapQPSK(Complex sym, float noise_var) {
    // QPSK: I and Q are independent BPSK channels
    // Modulator maps: bits[1,0] where bit1→I, bit0→Q
    // QPSK_MAP[0]=(-,-), [1]=(-,+), [2]=(+,-), [3]=(+,+)
    // So positive I/Q → bit 1 → need negative LLR
    float scale = -2.0f * 0.7071067811865476f / noise_var;
    return {sym.real() * scale, sym.imag() * scale};
}

std::vector<float> softDemapQAM16(Complex sym, float noise_var) {
    // 16-QAM soft demapping with correct LLR signs
    // Modulator Gray code: levels[] = {-3, -1, 3, 1} for bits 00,01,10,11
    //   MSB: negative → 0, positive → 1
    //   LSB: outer (|val|=3) → 0, inner (|val|=1) → 1
    // LLR convention: negative = bit 1, positive = bit 0
    constexpr float d = 0.6324555320336759f;  // 2/sqrt(10) - threshold between 1 and 3
    std::vector<float> llrs(4);

    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 3,2 of 4-bit word)
    llrs[0] = -scale * I;                      // bit3 (MSB): sign
    llrs[1] = scale * (std::abs(I) - d);       // bit2: outer→0, inner→1

    // Q bits (bits 1,0 of 4-bit word)
    llrs[2] = -scale * Q;                      // bit1 (MSB): sign
    llrs[3] = scale * (std::abs(Q) - d);       // bit0: outer→0, inner→1

    return llrs;
}

std::vector<float> softDemapQAM64(Complex sym, float noise_var) {
    // 64-QAM soft demapping with correct LLR signs
    // Modulator uses upper 3 bits for I, lower 3 bits for Q
    // Gray-coded levels: {-7,-5,-1,-3,7,5,1,3} / sqrt(42)
    // For this mapping:
    //   bit5/bit2 (MSB): negative I/Q → 0, positive I/Q → 1
    //   bit4/bit1: outer (|val| > d4) → 0, inner (|val| < d4) → 1
    //   bit3/bit0: depends on region, threshold at d2 from midpoint
    //
    // LLR convention: negative → bit 1, positive → bit 0
    constexpr float d2 = 0.3086067f;  // 2/sqrt(42)
    constexpr float d4 = 0.6172134f;  // 4/sqrt(42)
    std::vector<float> llrs(6);

    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 5,4,3 of 6-bit word)
    // bit5: I<0 → 0, I>0 → 1. LLR: I>0 should give negative LLR
    llrs[0] = -scale * I;
    // bit4: |I|>d4 → 0 (outer), |I|<d4 → 1 (inner). LLR: outer should give positive
    llrs[1] = scale * (std::abs(I) - d4);
    // bit3: distance from midpoint. Pattern gives 0 at extremes, 1 in middle of each region
    llrs[2] = scale * (std::abs(std::abs(I) - d4) - d2);

    // Q bits (bits 2,1,0 of 6-bit word) - same pattern
    llrs[3] = -scale * Q;
    llrs[4] = scale * (std::abs(Q) - d4);
    llrs[5] = scale * (std::abs(std::abs(Q) - d4) - d2);

    return llrs;
}

std::vector<float> softDemapQAM256(Complex sym, float noise_var) {
    // 256-QAM soft demapping with correct LLR signs
    // Same Gray code pattern as QAM64 but with 4 bits per axis
    // LLR convention: negative = bit 1, positive = bit 0
    constexpr float d2 = 0.1290994f;  // 2/sqrt(170)
    constexpr float d4 = 0.2581989f;  // 4/sqrt(170)
    constexpr float d8 = 0.5163978f;  // 8/sqrt(170)
    std::vector<float> llrs(8);

    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 7,6,5,4 of 8-bit word)
    llrs[0] = -scale * I;                                               // bit 7: sign
    llrs[1] = scale * (std::abs(I) - d8);                               // bit 6: outer/inner
    llrs[2] = scale * (std::abs(std::abs(I) - d8) - d4);                // bit 5
    llrs[3] = scale * (std::abs(std::abs(std::abs(I) - d8) - d4) - d2); // bit 4

    // Q bits (bits 3,2,1,0 of 8-bit word)
    llrs[4] = -scale * Q;                                               // bit 3: sign
    llrs[5] = scale * (std::abs(Q) - d8);                               // bit 2: outer/inner
    llrs[6] = scale * (std::abs(std::abs(Q) - d8) - d4);                // bit 1
    llrs[7] = scale * (std::abs(std::abs(std::abs(Q) - d8) - d4) - d2); // bit 0

    return llrs;
}

} // anonymous namespace

struct OFDMDemodulator::Impl {
    ModemConfig config;
    FFT fft;
    NCO mixer;

    // Carrier indices (must match modulator)
    std::vector<int> data_carrier_indices;
    std::vector<int> pilot_carrier_indices;
    std::vector<Complex> pilot_sequence;

    // Synchronization state (atomic for thread-safe UI access)
    enum class State { SEARCHING, SYNCED };
    std::atomic<State> state{State::SEARCHING};
    std::atomic<int> synced_symbol_count{0};  // Count symbols since sync, for timeout
    static constexpr int MAX_SYMBOLS_BEFORE_TIMEOUT = 30;  // ~300ms - faster timeout

    // Sample buffer
    Samples rx_buffer;
    size_t symbol_samples;

    // Channel estimate (per carrier)
    std::vector<Complex> channel_estimate;
    float noise_variance = 0.1f;

    // SNR estimation (from pilots)
    float estimated_snr_linear = 1.0f;  // Linear SNR (not dB)
    float snr_alpha = 0.3f;             // Smoothing factor for running average
    int snr_symbol_count = 0;           // Counts symbols for SNR warm-up

    // Output data
    Bytes demod_data;
    std::vector<float> soft_bits;
    ChannelQuality quality;

    // Constellation display (latest equalized symbols)
    std::vector<Complex> constellation_symbols;
    mutable std::mutex constellation_mutex;

    // Sync detection
    std::vector<Complex> sync_sequence;
    float sync_threshold = 0.90f;  // Threshold for valid sync region
    size_t search_offset = 0;      // Track where we left off searching (optimization)

    // Pre-computed interpolation lookup (avoids O(n²) per symbol)
    struct InterpInfo {
        int fft_idx;        // FFT bin index of this data carrier
        int lower_pilot;    // FFT bin index of pilot below (-1 if none)
        int upper_pilot;    // FFT bin index of pilot above (-1 if none)
        float alpha;        // Interpolation weight: estimate = (1-alpha)*lower + alpha*upper
    };
    std::vector<InterpInfo> interp_table;

    // Frequency offset estimation and correction
    float freq_offset_hz = 0.0f;           // Estimated frequency offset
    float freq_offset_filtered = 0.0f;     // Low-pass filtered estimate
    std::vector<Complex> prev_pilot_phases; // Previous symbol's pilot values
    static constexpr float FREQ_OFFSET_ALPHA = 0.3f;  // Smoothing factor
    float freq_correction_phase = 0.0f;    // Running phase for correction

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        symbol_samples = cfg.getSymbolDuration();
        channel_estimate.resize(cfg.fft_size, Complex(1, 0));

        setupCarriers();
        generateSequences();
        buildInterpTable();
    }

    void setupCarriers() {
        // Must match modulator exactly
        int half_carriers = config.num_carriers / 2;

        int pilot_count = 0;
        for (int i = -half_carriers; i <= half_carriers; ++i) {
            if (i == 0) continue;

            int fft_idx = (i + config.fft_size) % config.fft_size;

            if (pilot_count % config.pilot_spacing == 0) {
                pilot_carrier_indices.push_back(fft_idx);
            } else {
                data_carrier_indices.push_back(fft_idx);
            }
            ++pilot_count;
        }
    }

    void generateSequences() {
        // Same Zadoff-Chu sequence as modulator
        size_t N = config.num_carriers;
        size_t u = 1;

        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * u * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }

        // Same pilot sequence
        pilot_sequence.resize(pilot_carrier_indices.size());
        std::mt19937 rng(0xDEADBEEF);
        for (size_t i = 0; i < pilot_sequence.size(); ++i) {
            pilot_sequence[i] = (rng() & 1) ? Complex(1, 0) : Complex(-1, 0);
        }
    }

    void buildInterpTable() {
        // Pre-compute interpolation weights for each data carrier
        // This replaces the O(n²) computation that was done every symbol

        // Build ordered carrier list: -half to -1, then +1 to +half (skipping DC)
        struct CarrierInfo {
            int fft_idx;
            bool is_pilot;
        };
        std::vector<CarrierInfo> carriers;
        int half = config.num_carriers / 2;
        int pilot_count = 0;

        for (int i = -half; i <= half; ++i) {
            if (i == 0) continue;
            int fft_idx = (i + config.fft_size) % config.fft_size;
            bool is_pilot = (pilot_count % config.pilot_spacing == 0);
            carriers.push_back({fft_idx, is_pilot});
            ++pilot_count;
        }

        // For each data carrier, find surrounding pilots and compute weight
        interp_table.clear();
        interp_table.reserve(data_carrier_indices.size());

        for (size_t ci = 0; ci < carriers.size(); ++ci) {
            if (carriers[ci].is_pilot) continue;

            InterpInfo info;
            info.fft_idx = carriers[ci].fft_idx;
            info.lower_pilot = -1;
            info.upper_pilot = -1;
            info.alpha = 0.5f;

            // Find nearest pilot before (in carrier order)
            int lower_ci = -1;
            for (int j = (int)ci - 1; j >= 0; --j) {
                if (carriers[j].is_pilot) {
                    info.lower_pilot = carriers[j].fft_idx;
                    lower_ci = j;
                    break;
                }
            }

            // Find nearest pilot after (in carrier order)
            int upper_ci = -1;
            for (size_t j = ci + 1; j < carriers.size(); ++j) {
                if (carriers[j].is_pilot) {
                    info.upper_pilot = carriers[j].fft_idx;
                    upper_ci = (int)j;
                    break;
                }
            }

            // Compute interpolation weight
            if (lower_ci >= 0 && upper_ci >= 0) {
                float total_dist = (float)(upper_ci - lower_ci);
                info.alpha = (total_dist > 0) ? (float)((int)ci - lower_ci) / total_dist : 0.5f;
            }

            interp_table.push_back(info);
        }
    }

    // Quick energy check - sample a few points to see if there's signal
    // Returns true if energy is above minimum threshold for valid signal
    bool hasMinimumEnergy(size_t offset, size_t window_len) const {
        if (offset + window_len > rx_buffer.size()) return false;

        // Sample every 16th point for speed (64 samples for 1024 window)
        constexpr size_t SAMPLE_STEP = 16;
        constexpr float MIN_RMS = 0.05f;  // Minimum RMS to consider as potential signal
        float sum_sq = 0;
        size_t count = 0;

        for (size_t i = 0; i < window_len; i += SAMPLE_STEP) {
            float s = rx_buffer[offset + i];
            sum_sq += s * s;
            ++count;
        }

        float rms = std::sqrt(sum_sq / count);
        return rms >= MIN_RMS;
    }

    // Convert real signal to analytic (complex) signal using FFT-based Hilbert transform
    // This is essential for CFO-tolerant correlation of real passband signals
    std::vector<Complex> toAnalytic(const float* samples, size_t len) {
        // Pad to next power of 2 for FFT efficiency
        size_t fft_len = 1;
        while (fft_len < len) fft_len *= 2;

        // Create temporary FFT
        FFT hilbert_fft(fft_len);

        // Copy input to complex buffer (real signal, zero imag)
        std::vector<Complex> time_in(fft_len, Complex(0, 0));
        for (size_t i = 0; i < len; ++i) {
            time_in[i] = Complex(samples[i], 0);
        }

        // Forward FFT
        std::vector<Complex> freq(fft_len);
        hilbert_fft.forward(time_in, freq);

        // Create analytic signal by zeroing negative frequencies
        // DC: keep as-is
        // Positive frequencies (1 to N/2-1): multiply by 2
        // Nyquist (N/2): keep as-is
        // Negative frequencies (N/2+1 to N-1): zero
        for (size_t i = 1; i < fft_len / 2; ++i) {
            freq[i] *= 2.0f;
        }
        for (size_t i = fft_len / 2 + 1; i < fft_len; ++i) {
            freq[i] = Complex(0, 0);
        }

        // Inverse FFT to get analytic signal
        std::vector<Complex> analytic(fft_len);
        hilbert_fft.inverse(freq, analytic);

        // Return only the original length
        analytic.resize(len);
        return analytic;
    }

    float measureCorrelation(size_t offset, float* out_energy = nullptr) {
        // CFO-tolerant sync detection using analytic signal correlation
        // For real passband signal s(t), we create analytic signal z(t) = s(t) + j*HT(s(t))
        // Then correlate z(t) with z(t-L) - the magnitude is independent of carrier frequency
        if (offset + symbol_samples * 2 > rx_buffer.size()) return 0.0f;

        size_t len = config.fft_size + config.getCyclicPrefix();

        // IMPORTANT: Compute analytic signal on the COMBINED window (both symbols together)
        // This preserves the phase relationship between the two symbols
        auto analytic = toAnalytic(&rx_buffer[offset], len * 2);

        // Complex correlation between first half and second half of the analytic signal
        // This maintains the phase relationship (delay of L samples = phase rotation)
        Complex P(0.0f, 0.0f);
        float R = 0.0f;

        for (size_t i = 0; i < len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + len];
            R += std::norm(analytic[i + len]);
        }

        if (out_energy) *out_energy = R;

        // Return magnitude of normalized correlation
        // For identical symbols, |P|/R ≈ 1.0 regardless of carrier frequency or CFO
        return std::abs(P) / (R + 1e-10f);
    }

    bool detectSync(size_t offset) {
        float energy = 0;
        float normalized = measureCorrelation(offset, &energy);

        // Require minimum energy to avoid false triggers on transients/noise
        // Energy is sum of squares over ~560 samples * 2 windows
        // For 0.3 RMS signal: energy = 560 * 0.3^2 * 2 = 100.8
        // Use high threshold to reject finger snaps and other transients
        constexpr float MIN_ENERGY = 80.0f;  // Require substantial sustained signal
        if (energy < MIN_ENERGY) {
            return false;
        }

        // Debug: print correlation at key offsets
        if (offset % 500 == 0 || normalized > 0.5f) {
            LOG_DEMOD(TRACE, "detectSync offset=%zu normalized=%.3f energy=%.1f threshold=%.2f",
                    offset, normalized, energy, sync_threshold);
        }

        return normalized > sync_threshold;
    }

    // Coarse CFO estimation using analytic signal correlation
    // Uses adjacent STS symbols to estimate frequency offset from phase rotation
    // Phase difference between symbols = 2π × (fc + δf) × L / fs
    // Returns estimated frequency offset in Hz
    float estimateCoarseCFO(size_t sync_offset) {
        size_t sym_len = config.fft_size + config.getCyclicPrefix();

        // Need at least 2 symbols for estimate
        if (sync_offset + sym_len * 2 > rx_buffer.size()) {
            return 0.0f;
        }

        // Compute analytic signal on the COMBINED window (both symbols together)
        // This preserves the phase relationship between the two symbols
        auto analytic = toAnalytic(&rx_buffer[sync_offset], sym_len * 2);

        // Complex correlation between first symbol and second symbol
        Complex P(0.0f, 0.0f);
        float R = 0.0f;

        for (size_t i = 0; i < sym_len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + sym_len];
            R += std::norm(analytic[i + sym_len]);
        }

        if (R < 1e-10f) {
            return 0.0f;
        }

        // Extract phase angle - this is the phase rotation between symbols
        // The Hilbert transform creates an analytic signal effectively at baseband,
        // so the phase directly represents the frequency offset (no carrier subtraction needed)
        float phase = std::atan2(P.imag(), P.real());

        // Convert phase to frequency: CFO = phase / (2π × L / fs)
        // The phase represents the rotation over sym_len samples
        float T_sym = static_cast<float>(sym_len) / config.sample_rate;
        float cfo_hz = phase / (2.0f * M_PI * T_sym);

        // Clamp to reasonable range
        // Maximum unambiguous range: ±fs/(2*L) = ±42.9 Hz for our params
        cfo_hz = std::max(-40.0f, std::min(40.0f, cfo_hz));

        LOG_SYNC(INFO, "Coarse CFO estimate: %.2f Hz (phase=%.3f rad)", cfo_hz, phase);

        return cfo_hz;
    }

    std::vector<Complex> toBaseband(SampleSpan samples) {
        // Mix down from center frequency, with frequency offset correction
        std::vector<Complex> baseband(samples.size());

        // Phase increment per sample for frequency correction
        // Negative because we're correcting (removing) the offset
        float phase_increment = -2.0f * M_PI * freq_offset_hz / config.sample_rate;

        for (size_t i = 0; i < samples.size(); ++i) {
            // Mix down from center frequency
            Complex osc = mixer.next();
            Complex mixed = samples[i] * std::conj(osc);

            // Apply frequency offset correction
            if (std::abs(freq_offset_hz) > 0.1f) {
                Complex correction(std::cos(freq_correction_phase),
                                   std::sin(freq_correction_phase));
                mixed *= correction;
                freq_correction_phase += phase_increment;

                // Wrap phase to prevent numerical overflow
                if (freq_correction_phase > M_PI) {
                    freq_correction_phase -= 2.0f * M_PI;
                } else if (freq_correction_phase < -M_PI) {
                    freq_correction_phase += 2.0f * M_PI;
                }
            }

            baseband[i] = mixed;
        }
        return baseband;
    }

    std::vector<Complex> extractSymbol(const std::vector<Complex>& baseband, size_t offset) {
        // Remove cyclic prefix
        size_t start = offset + config.getCyclicPrefix();

        // Copy FFT-sized chunk
        std::vector<Complex> symbol(config.fft_size);
        for (size_t i = 0; i < config.fft_size && (start + i) < baseband.size(); ++i) {
            symbol[i] = baseband[start + i];
        }

        // FFT to frequency domain
        std::vector<Complex> freq;
        fft.forward(symbol, freq);

        return freq;
    }

    void updateChannelEstimate(const std::vector<Complex>& freq_domain) {
        // Use pilots to estimate channel
        // H = Rx_pilot / Tx_pilot

        // For the FIRST symbol after sync, use the pilot estimate directly (no smoothing)
        // to avoid the initial (1,0) estimate corrupting equalization.
        // For subsequent symbols, smooth to track channel changes.
        float alpha = soft_bits.empty() ? 1.0f : 0.9f;

        // First pass: compute all LS estimates and their average
        std::vector<Complex> h_ls_all(pilot_carrier_indices.size());
        Complex h_sum(0, 0);

        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            Complex rx = freq_domain[idx];
            Complex tx = pilot_sequence[i];

            // LS estimate: H = Rx / Tx
            h_ls_all[i] = rx / tx;
            h_sum += h_ls_all[i];
        }

        Complex h_avg = h_sum / static_cast<float>(pilot_carrier_indices.size());
        float signal_power = std::norm(h_avg);

        // Second pass: update channel estimates and measure noise
        // Noise = deviation of each pilot's estimate from the average
        // For flat channel: all pilots see same H, deviation = noise
        float noise_power_sum = 0.0f;

        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];

            // Measure deviation from average (this is the noise)
            Complex deviation = h_ls_all[i] - h_avg;
            noise_power_sum += std::norm(deviation);

            // Update smoothed channel estimate
            Complex h_old = channel_estimate[idx];
            channel_estimate[idx] = alpha * h_ls_all[i] + (1.0f - alpha) * h_old;
        }

        // === Frequency offset estimation from pilot phase differences ===
        // Compare pilot phases between consecutive symbols
        if (!prev_pilot_phases.empty() && prev_pilot_phases.size() == h_ls_all.size()) {
            float phase_diff_sum = 0.0f;
            int valid_count = 0;

            for (size_t i = 0; i < h_ls_all.size(); ++i) {
                // Phase difference = arg(current * conj(previous))
                Complex diff = h_ls_all[i] * std::conj(prev_pilot_phases[i]);
                float phase_diff = std::atan2(diff.imag(), diff.real());

                // Skip pilots with very low magnitude (unreliable)
                if (std::norm(prev_pilot_phases[i]) > 1e-6f &&
                    std::norm(h_ls_all[i]) > 1e-6f) {
                    phase_diff_sum += phase_diff;
                    valid_count++;
                }
            }

            if (valid_count > 0) {
                float avg_phase_diff = phase_diff_sum / valid_count;

                // Convert phase difference to frequency offset
                // Δf = Δφ / (2π * T_symbol)
                float symbol_duration = static_cast<float>(config.getSymbolDuration()) /
                                       static_cast<float>(config.sample_rate);
                float instantaneous_offset = avg_phase_diff / (2.0f * M_PI * symbol_duration);

                // Low-pass filter to reduce noise
                freq_offset_filtered = FREQ_OFFSET_ALPHA * instantaneous_offset +
                                      (1.0f - FREQ_OFFSET_ALPHA) * freq_offset_filtered;

                // Clamp to reasonable range (±50 Hz for HF)
                freq_offset_hz = std::max(-50.0f, std::min(50.0f, freq_offset_filtered));

                LOG_DEMOD(TRACE, "Freq offset: instant=%.2f Hz, filtered=%.2f Hz",
                         instantaneous_offset, freq_offset_hz);
            }
        }

        // Store current pilots for next symbol
        prev_pilot_phases = h_ls_all;

        // Interpolate between pilots for data carriers
        interpolateChannel();

        // Update noise variance and SNR
        // noise_power_sum = sum of |h[i] - h_avg|² over N pilots
        // For N pilots, this measures sample variance with (N-1) degrees of freedom
        // So average noise per pilot = noise_power_sum / (N-1)
        size_t N = pilot_carrier_indices.size();
        if (N > 1 && noise_power_sum > 0.0f) {
            noise_variance = noise_power_sum / (N - 1);  // Unbiased variance estimator
            if (noise_variance < 1e-6f) noise_variance = 1e-6f;

            // SNR = signal_power / noise_variance
            float instantaneous_snr = signal_power / noise_variance;

            // Clamp to reasonable range (0.1 to 10000 linear = -10 to +40 dB)
            instantaneous_snr = std::max(0.1f, std::min(10000.0f, instantaneous_snr));

            // Smooth SNR estimate
            estimated_snr_linear = snr_alpha * instantaneous_snr + (1.0f - snr_alpha) * estimated_snr_linear;
        }
    }

    void interpolateChannel() {
        // Interpolate channel estimate from pilots to data carriers
        // Uses pre-computed lookup table for O(n) instead of O(n²)
        for (size_t dc = 0; dc < interp_table.size(); ++dc) {
            const auto& info = interp_table[dc];
            if (info.lower_pilot >= 0 && info.upper_pilot >= 0) {
                // Linear interpolate between surrounding pilots
                channel_estimate[info.fft_idx] =
                    (1.0f - info.alpha) * channel_estimate[info.lower_pilot] +
                    info.alpha * channel_estimate[info.upper_pilot];
            } else if (info.lower_pilot >= 0) {
                channel_estimate[info.fft_idx] = channel_estimate[info.lower_pilot];
            } else if (info.upper_pilot >= 0) {
                channel_estimate[info.fft_idx] = channel_estimate[info.upper_pilot];
            }
        }
    }

    std::vector<Complex> equalize(const std::vector<Complex>& freq_domain) {
        // Zero-forcing equalization: divide by channel estimate
        std::vector<Complex> equalized(data_carrier_indices.size());

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            Complex h = channel_estimate[idx];

            // Avoid division by zero
            if (std::norm(h) < 1e-10f) {
                equalized[i] = Complex(0, 0);
            } else {
                equalized[i] = freq_domain[idx] / h;
            }

        }

        return equalized;
    }

    void demodulateSymbol(const std::vector<Complex>& equalized, Modulation mod) {
        // Save symbols for constellation display
        {
            std::lock_guard<std::mutex> lock(constellation_mutex);
            // Keep last ~200 symbols for display (rolling buffer)
            constellation_symbols.insert(constellation_symbols.end(), equalized.begin(), equalized.end());
            if (constellation_symbols.size() > 500) {
                constellation_symbols.erase(constellation_symbols.begin(),
                                           constellation_symbols.begin() + (constellation_symbols.size() - 500));
            }
        }

        // Log first few equalized symbols when starting a new frame
        if (soft_bits.empty() && !equalized.empty()) {
            LOG_DEMOD(DEBUG, "First equalized symbols: (%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f)",
                    equalized[0].real(), equalized[0].imag(),
                    equalized.size() > 1 ? equalized[1].real() : 0.0f,
                    equalized.size() > 1 ? equalized[1].imag() : 0.0f,
                    equalized.size() > 2 ? equalized[2].real() : 0.0f,
                    equalized.size() > 2 ? equalized[2].imag() : 0.0f);
        }

        // Convert symbols to soft bits (LLRs)
        for (const auto& sym : equalized) {
            switch (mod) {
                case Modulation::BPSK:
                    soft_bits.push_back(softDemapBPSK(sym, noise_variance));
                    break;
                case Modulation::QPSK: {
                    auto llrs = softDemapQPSK(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM16: {
                    auto llrs = softDemapQAM16(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM64: {
                    auto llrs = softDemapQAM64(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM256: {
                    auto llrs = softDemapQAM256(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                default: {
                    auto llrs = softDemapQPSK(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                }
            }
        }
    }

    void updateQuality() {
        // SNR from pilot-based estimation (already computed in updateChannelEstimate)
        quality.snr_db = 10.0f * std::log10(estimated_snr_linear);

        // Doppler estimation would require tracking phase changes across symbols
        // For now, estimate from noise variance rate of change (placeholder)
        quality.doppler_hz = 0;

        // Delay spread estimation would require analyzing channel impulse response
        // For now, estimate from channel frequency selectivity (placeholder)
        quality.delay_spread_ms = 0;

        // BER estimate based on SNR and modulation
        // These thresholds are approximate for QPSK with LDPC
        if (quality.snr_db > 15) {
            quality.ber_estimate = 1e-6f;
        } else if (quality.snr_db > 10) {
            quality.ber_estimate = 1e-5f;
        } else if (quality.snr_db > 5) {
            quality.ber_estimate = 1e-3f;
        } else {
            quality.ber_estimate = 1e-1f;
        }
    }
};

OFDMDemodulator::OFDMDemodulator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OFDMDemodulator::~OFDMDemodulator() = default;

bool OFDMDemodulator::process(SampleSpan samples) {
    // Add to buffer
    impl_->rx_buffer.insert(impl_->rx_buffer.end(), samples.begin(), samples.end());

    // State machine
    if (impl_->state.load() == Impl::State::SEARCHING) {
        // Look for sync preamble
        // Preamble symbols have NO guard interval (only fft_size + cp)
        size_t preamble_symbol_len = impl_->config.fft_size + impl_->config.getCyclicPrefix();
        size_t preamble_total_len = preamble_symbol_len * 6;  // 4 STS + 2 LTS
        size_t correlation_window = preamble_symbol_len * 2;  // Window needed for correlation

        // OPTIMIZATION: Trim samples we've already searched (they're stale)
        // Keep a margin of preamble_total_len in case preamble spans the boundary
        if (impl_->search_offset > preamble_total_len * 2) {
            size_t trim = impl_->search_offset - preamble_total_len;
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + trim);
            impl_->search_offset -= trim;
        }

        // Hard limit on buffer size to prevent runaway memory usage
        constexpr size_t ABSOLUTE_MAX_BUFFER = 24000;  // ~500ms at 48kHz
        if (impl_->rx_buffer.size() > ABSOLUTE_MAX_BUFFER) {
            size_t trim = impl_->rx_buffer.size() - ABSOLUTE_MAX_BUFFER;
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + trim);
            impl_->search_offset = (impl_->search_offset > trim) ? impl_->search_offset - trim : 0;
        }

        // Sync detection: Resume from where we left off (don't re-search old samples)
        // Step by 8 samples - plenty accurate for preamble detection
        // Energy pre-filter: skip silent regions to save correlation work
        bool found_sync = false;
        size_t sync_offset = 0;
        float sync_corr = 0;

        constexpr size_t STEP_SIZE = 8;  // Check every 8th sample
        constexpr size_t MAX_ITERATIONS = 300;  // Limit work per frame (~2400 samples)
        size_t iterations = 0;
        size_t i = impl_->search_offset;

        // Track max correlation for debugging
        float max_corr_found = 0.0f;
        size_t max_corr_offset = 0;

        while (i + preamble_total_len < impl_->rx_buffer.size() && iterations < MAX_ITERATIONS) {
            // Quick energy check - skip silent regions entirely
            if (!impl_->hasMinimumEnergy(i, correlation_window)) {
                i += correlation_window / 2;  // Skip ahead by half window
                ++iterations;
                continue;
            }

            // Full correlation check
            float corr = impl_->measureCorrelation(i);

            // Track maximum for debugging
            if (corr > max_corr_found) {
                max_corr_found = corr;
                max_corr_offset = i;
            }

            if (corr > impl_->sync_threshold) {
                // Found first point exceeding threshold - this is preamble start
                found_sync = true;
                sync_offset = i;
                sync_corr = corr;
                break;
            }

            i += STEP_SIZE;
            ++iterations;
        }

        // Log max correlation when sync fails to help debug
        if (!found_sync && max_corr_found > 0.5f) {
            LOG_SYNC(DEBUG, "Sync failed: max_corr=%.3f at offset=%zu (threshold=%.2f)",
                    max_corr_found, max_corr_offset, impl_->sync_threshold);
        }

        // Update search position for next frame (resume from here)
        impl_->search_offset = i;

        if (found_sync) {
            LOG_SYNC(INFO, "SYNC FOUND: offset=%zu (corr=%.3f)",
                    sync_offset, sync_corr);

            // Estimate coarse CFO from preamble BEFORE discarding it
            float coarse_cfo = impl_->estimateCoarseCFO(sync_offset);

            // Initialize frequency offset with coarse estimate
            // This gives us a head start on tracking
            impl_->freq_offset_hz = coarse_cfo;
            impl_->freq_offset_filtered = coarse_cfo;
            impl_->freq_correction_phase = 0.0f;  // Reset correction phase

            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + sync_offset + preamble_total_len);
            impl_->search_offset = 0;  // Reset for next search
            impl_->state.store(Impl::State::SYNCED);
            impl_->synced_symbol_count.store(0);  // Reset timeout counter
            impl_->mixer.reset();
        }
    }

    if (impl_->state.load() == Impl::State::SYNCED) {
        // Process complete symbols (stop when we have enough for one LDPC block)
        while (impl_->rx_buffer.size() >= impl_->symbol_samples &&
               impl_->soft_bits.size() < LDPC_BLOCK_SIZE) {
            // Convert to baseband
            SampleSpan sym_samples(impl_->rx_buffer.data(), impl_->symbol_samples);
            auto baseband = impl_->toBaseband(sym_samples);

            // Extract and FFT
            auto freq_domain = impl_->extractSymbol(baseband, 0);

            // Update channel estimate from pilots
            impl_->updateChannelEstimate(freq_domain);

            // Equalize
            auto equalized = impl_->equalize(freq_domain);

            // Demodulate (using config's modulation setting)
            impl_->demodulateSymbol(equalized, impl_->config.modulation);

            // Remove processed samples
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + impl_->symbol_samples);

            impl_->updateQuality();

            // Timeout check: if we've processed too many symbols without completing,
            // this was likely a false sync - reset to searching
            int sym_count = ++impl_->synced_symbol_count;
            if (sym_count > Impl::MAX_SYMBOLS_BEFORE_TIMEOUT) {
                LOG_SYNC(WARN, "Sync timeout after %d symbols, resetting to SEARCHING", sym_count);
                impl_->state.store(Impl::State::SEARCHING);
                impl_->soft_bits.clear();
                impl_->synced_symbol_count.store(0);
                return false;
            }
        }

        // Return true if we have enough soft bits for an LDPC codeword
        return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
    }

    return false;
}

Bytes OFDMDemodulator::getData() {
    // Convert soft bits to hard bits, then to bytes
    Bytes data;
    uint8_t byte = 0;
    int bit_count = 0;

    for (float llr : impl_->soft_bits) {
        uint8_t bit = (llr > 0) ? 1 : 0;
        byte = (byte << 1) | bit;
        ++bit_count;

        if (bit_count == 8) {
            data.push_back(byte);
            byte = 0;
            bit_count = 0;
        }
    }

    impl_->soft_bits.clear();
    return data;
}

std::vector<float> OFDMDemodulator::getSoftBits() {
    // Debug: print first 24 LLRs and their hard decisions
    if (g_log_level >= LogLevel::DEBUG && g_log_categories.demod && impl_->soft_bits.size() >= 24) {
        char buf[256];
        int pos = 0;
        for (size_t i = 0; i < 24 && pos < 240; ++i) {
            int bit = (impl_->soft_bits[i] < 0) ? 1 : 0;  // LDPC convention
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%+.1f(%d) ", impl_->soft_bits[i], bit);
            if ((i + 1) % 6 == 0) pos += snprintf(buf + pos, sizeof(buf) - pos, "| ");
        }
        LOG_DEMOD(DEBUG, "First 24 LLRs: %s", buf);
    }

    // Return exactly one LDPC block worth of soft bits
    if (impl_->soft_bits.size() <= LDPC_BLOCK_SIZE) {
        auto bits = std::move(impl_->soft_bits);
        impl_->soft_bits.clear();
        return bits;
    } else {
        // Return first block, keep remainder
        std::vector<float> bits(impl_->soft_bits.begin(),
                                impl_->soft_bits.begin() + LDPC_BLOCK_SIZE);
        impl_->soft_bits.erase(impl_->soft_bits.begin(),
                               impl_->soft_bits.begin() + LDPC_BLOCK_SIZE);
        return bits;
    }
}

ChannelQuality OFDMDemodulator::getChannelQuality() const {
    return impl_->quality;
}

float OFDMDemodulator::getEstimatedSNR() const {
    return 10.0f * std::log10(impl_->estimated_snr_linear);
}

float OFDMDemodulator::getFrequencyOffset() const {
    return impl_->freq_offset_hz;
}

Symbol OFDMDemodulator::getConstellationSymbols() const {
    std::lock_guard<std::mutex> lock(impl_->constellation_mutex);
    return impl_->constellation_symbols;
}

bool OFDMDemodulator::isSynced() const {
    return impl_->state.load() == Impl::State::SYNCED;
}

void OFDMDemodulator::reset() {
    impl_->state.store(Impl::State::SEARCHING);
    impl_->synced_symbol_count.store(0);
    impl_->search_offset = 0;
    impl_->rx_buffer.clear();
    impl_->soft_bits.clear();
    impl_->demod_data.clear();
    std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
    impl_->snr_symbol_count = 0;
    impl_->estimated_snr_linear = 1.0f;
    impl_->noise_variance = 0.1f;  // Reset to default (prevents stale state from prior frames)

    // Reset frequency offset tracking
    impl_->freq_offset_hz = 0.0f;
    impl_->freq_offset_filtered = 0.0f;
    impl_->freq_correction_phase = 0.0f;
    impl_->prev_pilot_phases.clear();
}

// ============ Channel Estimator ============

struct ChannelEstimator::Impl {
    ModemConfig config;
    std::vector<Complex> h_estimate;
    ChannelQuality quality;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , h_estimate(cfg.fft_size, Complex(1, 0))
    {}
};

ChannelEstimator::ChannelEstimator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

ChannelEstimator::~ChannelEstimator() = default;

void ChannelEstimator::updateFromPilots(const Symbol& received, const Symbol& expected) {
    for (size_t i = 0; i < received.size() && i < expected.size(); ++i) {
        if (std::norm(expected[i]) > 1e-10f) {
            Complex h = received[i] / expected[i];
            impl_->h_estimate[i] = 0.5f * h + 0.5f * impl_->h_estimate[i];
        }
    }
}

Symbol ChannelEstimator::equalize(const Symbol& received) {
    Symbol output(received.size());
    for (size_t i = 0; i < received.size(); ++i) {
        if (std::norm(impl_->h_estimate[i]) > 1e-10f) {
            output[i] = received[i] / impl_->h_estimate[i];
        } else {
            output[i] = received[i];
        }
    }
    return output;
}

ChannelQuality ChannelEstimator::getQuality() const {
    return impl_->quality;
}

void ChannelEstimator::interpolate() {
    // Simple linear interpolation - would be enhanced with MMSE
}

} // namespace ultra
