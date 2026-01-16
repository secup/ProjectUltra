#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
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

// LLR clipping to prevent overconfident decisions
// On fading channels, channel estimation errors can produce huge LLRs
// that are confidently WRONG. Clipping allows LDPC to correct them.
constexpr float MAX_LLR = 10.0f;

// Minimum LLR magnitude to prevent complete erasures
// When symbol is near decision boundary, don't tell LDPC "I have no idea"
// Instead, give it a weak opinion that LDPC can use or override
// This helps with dense constellations where many symbols land on boundaries
constexpr float MIN_LLR_MAG = 0.5f;

// Pilot sequence RNG seed - must match modulator for channel estimation
constexpr uint32_t PILOT_RNG_SEED = 0x50494C54;  // "PILT" in ASCII

inline float clipLLR(float llr) {
    float clipped = std::max(-MAX_LLR, std::min(MAX_LLR, llr));
    // Apply minimum magnitude while preserving sign
    if (std::abs(clipped) < MIN_LLR_MAG) {
        clipped = (clipped >= 0) ? MIN_LLR_MAG : -MIN_LLR_MAG;
    }
    return clipped;
}

float softDemapBPSK(Complex sym, float noise_var) {
    // BPSK: -1 maps to bit 0, +1 maps to bit 1
    // LLR convention: negative = bit 1, positive = bit 0
    // So: positive symbol → bit 1 → need negative LLR
    return clipLLR(-2.0f * sym.real() / noise_var);
}

std::vector<float> softDemapQPSK(Complex sym, float noise_var) {
    // QPSK: I and Q are independent BPSK channels
    // Modulator maps: bits[1,0] where bit1→I, bit0→Q
    // QPSK_MAP[0]=(-,-), [1]=(-,+), [2]=(+,-), [3]=(+,+)
    // So positive I/Q → bit 1 → need negative LLR
    float scale = -2.0f * 0.7071067811865476f / noise_var;
    return {clipLLR(sym.real() * scale), clipLLR(sym.imag() * scale)};
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
    llrs[0] = clipLLR(-scale * I);                      // bit3 (MSB): sign
    llrs[1] = clipLLR(scale * (std::abs(I) - d));       // bit2: outer→0, inner→1

    // Q bits (bits 1,0 of 4-bit word)
    llrs[2] = clipLLR(-scale * Q);                      // bit1 (MSB): sign
    llrs[3] = clipLLR(scale * (std::abs(Q) - d));       // bit0: outer→0, inner→1

    return llrs;
}

std::vector<float> softDemapQAM32(Complex sym, float noise_var) {
    // 32-QAM BRUTE-FORCE soft demapping (max-log-MAP)
    // 8×4 grid: 8 Q levels × 4 I levels = 32 points
    //
    // For each bit position, compute:
    //   LLR(b) = min_dist(bit=0) - min_dist(bit=1)
    // where min_dist is the minimum squared distance to any constellation
    // point with that bit value.
    //
    // This is more expensive than analytical but guaranteed correct.

    constexpr float scale = 0.1961161351381840f;  // 1/sqrt(26)

    // I levels (2 bits): Gray code 00→-3, 01→-1, 11→+1, 10→+3
    static const float I_LEVELS[4] = {-3, -1, 1, 3};
    static const int I_GRAY[4] = {0, 1, 3, 2};  // Bit pattern for each level index

    // Q levels (3 bits): Gray code
    static const float Q_LEVELS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    static const int Q_GRAY[8] = {0, 1, 3, 2, 6, 7, 5, 4};  // Bit pattern for each level index

    // Build constellation with bit mappings
    // bits = (q_bits << 2) | i_bits, where q_bits uses Q_GRAY and i_bits uses I_GRAY
    struct Point {
        Complex pos;
        int bits;
    };
    static Point constellation[32];
    static bool initialized = false;

    if (!initialized) {
        for (int qi = 0; qi < 8; ++qi) {
            for (int ii = 0; ii < 4; ++ii) {
                int idx = qi * 4 + ii;
                constellation[idx].pos = Complex(I_LEVELS[ii] * scale, Q_LEVELS[qi] * scale);
                constellation[idx].bits = (Q_GRAY[qi] << 2) | I_GRAY[ii];
            }
        }
        initialized = true;
    }

    std::vector<float> llrs(5);
    float scale_factor = 2.0f / noise_var;  // Must match other demappers (2/σ² for proper LLR)

    // For each bit position, find minimum distance to points with bit=0 and bit=1
    for (int b = 0; b < 5; ++b) {
        int bit_mask = 1 << (4 - b);  // bit 4 is llrs[0], bit 0 is llrs[4]
        float min_dist_0 = 1e10f;
        float min_dist_1 = 1e10f;

        for (int i = 0; i < 32; ++i) {
            Complex diff = sym - constellation[i].pos;
            float dist_sq = diff.real() * diff.real() + diff.imag() * diff.imag();

            if (constellation[i].bits & bit_mask) {
                if (dist_sq < min_dist_1) min_dist_1 = dist_sq;
            } else {
                if (dist_sq < min_dist_0) min_dist_0 = dist_sq;
            }
        }

        // LLR = scale * (min_dist_1 - min_dist_0)
        // Standard max-log-MAP: positive LLR = bit more likely 0, negative = bit more likely 1
        // If symbol is near bit-0 point: min_dist_0 small, min_dist_1 large → LLR positive
        llrs[b] = clipLLR(scale_factor * (min_dist_1 - min_dist_0));
    }

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
    llrs[0] = clipLLR(-scale * I);
    // bit4: |I|>d4 → 0 (outer), |I|<d4 → 1 (inner). LLR: outer should give positive
    llrs[1] = clipLLR(scale * (std::abs(I) - d4));
    // bit3: distance from midpoint. Pattern gives 0 at extremes, 1 in middle of each region
    llrs[2] = clipLLR(scale * (std::abs(std::abs(I) - d4) - d2));

    // Q bits (bits 2,1,0 of 6-bit word) - same pattern
    llrs[3] = clipLLR(-scale * Q);
    llrs[4] = clipLLR(scale * (std::abs(Q) - d4));
    llrs[5] = clipLLR(scale * (std::abs(std::abs(Q) - d4) - d2));

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
    llrs[0] = clipLLR(-scale * I);                                               // bit 7: sign
    llrs[1] = clipLLR(scale * (std::abs(I) - d8));                               // bit 6: outer/inner
    llrs[2] = clipLLR(scale * (std::abs(std::abs(I) - d8) - d4));                // bit 5
    llrs[3] = clipLLR(scale * (std::abs(std::abs(std::abs(I) - d8) - d4) - d2)); // bit 4

    // Q bits (bits 3,2,1,0 of 8-bit word)
    llrs[4] = clipLLR(-scale * Q);                                               // bit 3: sign
    llrs[5] = clipLLR(scale * (std::abs(Q) - d8));                               // bit 2: outer/inner
    llrs[6] = clipLLR(scale * (std::abs(std::abs(Q) - d8) - d4));                // bit 1
    llrs[7] = clipLLR(scale * (std::abs(std::abs(std::abs(Q) - d8) - d4) - d2)); // bit 0

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
    std::atomic<int> idle_call_count{0};      // Count process() calls with no progress
    static constexpr int MAX_SYMBOLS_BEFORE_TIMEOUT = 250;  // Support larger frames (up to ~1KB)
    static constexpr int MAX_IDLE_CALLS_BEFORE_RESET = 10; // Reset after 10 calls with no new soft bits

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
    float sync_threshold;  // Threshold for valid sync region (from config)
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

    // LMS/RLS Adaptive equalizer state
    std::vector<Complex> lms_weights;      // Per-carrier LMS weights
    std::vector<Complex> last_decisions;   // Last equalized symbols (for decision-directed)
    std::vector<float> rls_P;              // RLS inverse correlation (diagonal approx)

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
        , sync_threshold(cfg.sync_threshold)
    {
        symbol_samples = cfg.getSymbolDuration();
        channel_estimate.resize(cfg.fft_size, Complex(1, 0));

        // Initialize adaptive equalizer state
        lms_weights.resize(cfg.fft_size, Complex(1, 0));
        last_decisions.resize(cfg.fft_size, Complex(0, 0));
        rls_P.resize(cfg.fft_size, 1.0f);

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

        // Same pilot sequence as modulator (must match for channel estimation)
        pilot_sequence.resize(pilot_carrier_indices.size());
        std::mt19937 rng(PILOT_RNG_SEED);
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

        // Compute average signal power from pilots
        float signal_power_sum = 0.0f;
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            signal_power_sum += std::norm(h_ls_all[i]);
        }
        float signal_power = signal_power_sum / pilot_carrier_indices.size();

        // Measure noise using TEMPORAL comparison (not frequency average!)
        // On frequency-selective channels, H varies across frequency legitimately.
        // But H should be nearly constant across TIME for slow fading.
        // So we compare each pilot to its previous value - the difference is noise.
        float noise_power_sum = 0.0f;
        size_t noise_count = 0;

        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];

            // If we have previous pilot values, use temporal noise estimation
            if (!prev_pilot_phases.empty() && i < prev_pilot_phases.size()) {
                // Noise = change in pilot estimate from previous symbol
                // For slow fading (< 1 Hz Doppler), this is mostly noise
                Complex prev_h = prev_pilot_phases[i];
                Complex curr_h = h_ls_all[i];

                // Normalize by magnitude to measure phase/amplitude noise
                // relative to signal strength
                if (std::norm(prev_h) > 1e-6f && std::norm(curr_h) > 1e-6f) {
                    Complex diff = curr_h - prev_h;
                    noise_power_sum += std::norm(diff);
                    noise_count++;
                }
            }

            // Update smoothed channel estimate
            Complex h_old = channel_estimate[idx];
            channel_estimate[idx] = alpha * h_ls_all[i] + (1.0f - alpha) * h_old;
        }

        // Fallback: if no previous pilots, use frequency-domain estimation
        // (less accurate on frequency-selective channels but better than nothing)
        if (noise_count == 0) {
            Complex h_avg = h_sum / static_cast<float>(pilot_carrier_indices.size());
            for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                Complex deviation = h_ls_all[i] - h_avg;
                noise_power_sum += std::norm(deviation);
            }
            noise_count = pilot_carrier_indices.size();
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

        // Initialize adaptive equalizer weights from pilot-based channel estimate
        // This gives the adaptive filter a good starting point for fading channels
        if (config.adaptive_eq_enabled) {
            for (int idx : data_carrier_indices) {
                // Only initialize if weights are at default (1,0) or significantly different
                // This allows LMS/RLS to track once initialized
                if (snr_symbol_count < 3) {  // First few symbols - seed from pilots
                    lms_weights[idx] = channel_estimate[idx];
                }
            }
            // Also initialize pilot carriers
            for (int idx : pilot_carrier_indices) {
                if (snr_symbol_count < 3) {
                    lms_weights[idx] = channel_estimate[idx];
                }
            }
        }

        // Update noise variance and SNR
        // noise_power_sum = sum of |h[i] - h_avg|² over N pilots
        // For N pilots, this measures sample variance with (N-1) degrees of freedom
        // So average noise per pilot = noise_power_sum / (N-1)
        if (noise_count > 1 && noise_power_sum > 0.0f) {
            noise_variance = noise_power_sum / (noise_count - 1);  // Unbiased variance estimator
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
        // Uses linear interpolation between adjacent pilots
        //
        // NOTE: Use complex interpolation, NOT magnitude/phase interpolation!
        // While mag/phase interpolation seems intuitive, it assumes smooth phase
        // variation across frequency - which is WRONG for multipath HF channels.
        // In Watterson/multipath channels, phase can jump rapidly between carriers
        // due to frequency-selective fading. Complex interpolation handles this
        // correctly by not forcing artificial phase continuity.
        for (size_t dc = 0; dc < interp_table.size(); ++dc) {
            const auto& info = interp_table[dc];
            if (info.lower_pilot >= 0 && info.upper_pilot >= 0) {
                Complex H1 = channel_estimate[info.lower_pilot];
                Complex H2 = channel_estimate[info.upper_pilot];

                // Linear interpolation of complex channel estimate
                channel_estimate[info.fft_idx] = (1.0f - info.alpha) * H1 + info.alpha * H2;
            } else if (info.lower_pilot >= 0) {
                channel_estimate[info.fft_idx] = channel_estimate[info.lower_pilot];
            } else if (info.upper_pilot >= 0) {
                channel_estimate[info.fft_idx] = channel_estimate[info.upper_pilot];
            }
        }
    }

    // Hard decision slicer for decision-directed mode
    // Returns nearest constellation point for the given modulation
    Complex hardDecision(Complex sym, Modulation mod) const {
        switch (mod) {
            case Modulation::BPSK:
                return Complex(sym.real() > 0 ? 1.0f : -1.0f, 0);

            case Modulation::QPSK: {
                float I = sym.real() > 0 ? 0.7071f : -0.7071f;
                float Q = sym.imag() > 0 ? 0.7071f : -0.7071f;
                return Complex(I, Q);
            }

            case Modulation::QAM16: {
                // Levels: -3, -1, 1, 3 normalized by sqrt(10)
                auto slice = [](float x) -> float {
                    if (x < -0.4f) return -0.9487f;       // -3/sqrt(10)
                    if (x < 0.0f) return -0.3162f;        // -1/sqrt(10)
                    if (x < 0.4f) return 0.3162f;         //  1/sqrt(10)
                    return 0.9487f;                        //  3/sqrt(10)
                };
                return Complex(slice(sym.real()), slice(sym.imag()));
            }

            case Modulation::QAM32: {
                // 32-QAM rectangular 8×4 constellation
                // I: 4 levels {-3,-1,+1,+3}, Q: 8 levels {-7,-5,-3,-1,+1,+3,+5,+7}
                constexpr float d = 0.1961161351381840f;  // 1/sqrt(26)
                auto slice_i = [](float x) -> float {
                    // 4 levels: -3,-1,+1,+3
                    constexpr float d = 0.1961161351381840f;
                    if (x < -2*d) return -3*d;
                    if (x < 0) return -d;
                    if (x < 2*d) return d;
                    return 3*d;
                };
                auto slice_q = [](float x) -> float {
                    // 8 levels: -7,-5,-3,-1,+1,+3,+5,+7
                    constexpr float d = 0.1961161351381840f;
                    if (x < -6*d) return -7*d;
                    if (x < -4*d) return -5*d;
                    if (x < -2*d) return -3*d;
                    if (x < 0) return -d;
                    if (x < 2*d) return d;
                    if (x < 4*d) return 3*d;
                    if (x < 6*d) return 5*d;
                    return 7*d;
                };
                return Complex(slice_i(sym.real()), slice_q(sym.imag()));
            }

            case Modulation::QAM64: {
                // Levels: -7,-5,-3,-1,1,3,5,7 normalized by sqrt(42)
                auto slice = [](float x) -> float {
                    constexpr float d = 0.1543f;  // 1/sqrt(42)
                    if (x < -6*d) return -7*d;
                    if (x < -4*d) return -5*d;
                    if (x < -2*d) return -3*d;
                    if (x < 0) return -d;
                    if (x < 2*d) return d;
                    if (x < 4*d) return 3*d;
                    if (x < 6*d) return 5*d;
                    return 7*d;
                };
                return Complex(slice(sym.real()), slice(sym.imag()));
            }

            default:
                return Complex(sym.real() > 0 ? 0.7071f : -0.7071f,
                              sym.imag() > 0 ? 0.7071f : -0.7071f);
        }
    }

    // LMS adaptive equalizer update
    void lmsUpdate(int idx, Complex received, Complex reference) {
        float mu = config.lms_mu;

        // Error = received - weight * reference (assumes weight ≈ channel)
        Complex error = received - lms_weights[idx] * reference;

        // Weight update: w = w + mu * conj(reference) * error
        lms_weights[idx] += mu * std::conj(reference) * error;
    }

    // RLS adaptive equalizer update
    void rlsUpdate(int idx, Complex received, Complex reference) {
        float lambda = config.rls_lambda;

        // Simplified diagonal RLS (scalar P per carrier)
        float P = rls_P[idx];
        float ref_norm = std::norm(reference);

        // Gain: k = P * ref / (lambda + P * |ref|^2)
        float k = P / (lambda + P * ref_norm);

        // Error
        Complex error = received - lms_weights[idx] * reference;

        // Weight update
        lms_weights[idx] += k * std::conj(reference) * error;

        // P update: P = (P - k * |ref|^2 * P) / lambda
        rls_P[idx] = (P - k * ref_norm * P) / lambda;

        // Prevent P from getting too small or too large
        rls_P[idx] = std::max(0.001f, std::min(1000.0f, rls_P[idx]));
    }

    // Per-carrier noise variance after equalization (for soft demapping)
    std::vector<float> carrier_noise_var;

    std::vector<Complex> equalize(const std::vector<Complex>& freq_domain, Modulation mod) {
        std::vector<Complex> equalized(data_carrier_indices.size());
        carrier_noise_var.resize(data_carrier_indices.size());

        bool use_adaptive = config.adaptive_eq_enabled;

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            Complex received = freq_domain[idx];

            if (use_adaptive) {
                // Adaptive equalization using LMS/RLS weights
                Complex h = lms_weights[idx];
                float h_power = std::norm(h);

                // Avoid division by zero
                if (h_power < 1e-10f) {
                    equalized[i] = Complex(0, 0);
                    carrier_noise_var[i] = 1.0f;  // High uncertainty
                } else {
                    equalized[i] = received / h;
                    // After ZF: noise variance = σ² / |H|²
                    carrier_noise_var[i] = noise_variance / h_power;
                }

                // Update weights using decision-directed mode
                if (config.decision_directed) {
                    Complex decision = hardDecision(equalized[i], mod);

                    if (config.adaptive_eq_use_rls) {
                        rlsUpdate(idx, received, decision);
                    } else {
                        lmsUpdate(idx, received, decision);
                    }

                    last_decisions[idx] = decision;
                }
            } else {
                // Zero-Forcing equalization with per-carrier noise tracking
                // ZF gives unbiased estimate: y_eq = y/H = x + n/H
                // The noise variance after ZF is: σ²_eq = σ² / |H|²
                // This per-carrier variance is CRITICAL for soft demapping!
                Complex h = channel_estimate[idx];
                float h_power = std::norm(h);

                if (h_power < 1e-10f) {
                    // Deep fade - output zero with high uncertainty
                    equalized[i] = Complex(0, 0);
                    carrier_noise_var[i] = 100.0f;  // Erasure: LLRs ≈ 0
                } else {
                    // Standard ZF equalization
                    equalized[i] = received / h;
                    // Per-carrier noise variance after ZF
                    // This makes deeply faded carriers give low-confidence LLRs
                    carrier_noise_var[i] = noise_variance / h_power;
                    // Clamp - but allow high variance for soft erasure
                    carrier_noise_var[i] = std::max(1e-6f, std::min(100.0f, carrier_noise_var[i]));
                }
            }
        }

        // Second pass: detect deep fades relative to average and apply soft erasure
        // On frequency-selective channels, some carriers can be 10-20 dB below average
        // These should be erased (LLR ≈ 0) rather than trusted
        float avg_h_power = 0.0f;
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            avg_h_power += std::norm(channel_estimate[idx]);
        }
        avg_h_power /= data_carrier_indices.size();

        // Threshold: 10 dB below average = 0.1 * average
        float fade_threshold = 0.1f * avg_h_power;
        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            float h_power = std::norm(channel_estimate[idx]);
            if (h_power < fade_threshold) {
                // Soft erasure: very high noise variance → LLRs ≈ 0
                // This tells LDPC "no information here, use other bits"
                carrier_noise_var[i] = 100.0f;
            }
        }

        return equalized;
    }

    // Legacy interface for backwards compatibility
    std::vector<Complex> equalize(const std::vector<Complex>& freq_domain) {
        return equalize(freq_domain, config.modulation);
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
            // For BPSK, symbols should be near +1 or -1 on real axis
            // Count how many are positive vs negative to detect phase inversion
            int pos_count = 0, neg_count = 0;
            for (const auto& s : equalized) {
                if (s.real() > 0) pos_count++;
                else neg_count++;
            }
            LOG_DEMOD(INFO, "First symbol stats: %zu carriers, %d positive, %d negative, first 3: (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)",
                    equalized.size(), pos_count, neg_count,
                    equalized[0].real(), equalized[0].imag(),
                    equalized.size() > 1 ? equalized[1].real() : 0.0f,
                    equalized.size() > 1 ? equalized[1].imag() : 0.0f,
                    equalized.size() > 2 ? equalized[2].real() : 0.0f,
                    equalized.size() > 2 ? equalized[2].imag() : 0.0f);
        }

        // Convert symbols to soft bits (LLRs)
        // CRITICAL: Use per-carrier noise variance for accurate LLRs!
        // On frequency-selective channels, deeply faded carriers have high noise variance
        // after ZF equalization, so their LLRs should have low confidence.
        //
        // For higher-order modulations, add channel estimation error margin.
        // Residual channel estimation error acts like additional noise, causing
        // constellation rotation/scaling. This error is proportional to the
        // channel estimation MSE which depends on pilot density and SNR.
        float ce_error_margin = 1.0f;
        switch (mod) {
            case Modulation::BPSK:
            case Modulation::QPSK:
                ce_error_margin = 1.0f;  // Robust to small errors
                break;
            case Modulation::QAM16:
                ce_error_margin = 1.2f;  // 20% margin
                break;
            case Modulation::QAM32:
                ce_error_margin = 1.5f;  // Same as 16QAM - let LDPC handle errors
                break;
            case Modulation::QAM64:
                ce_error_margin = 1.8f;  // 80% margin
                break;
            case Modulation::QAM256:
                ce_error_margin = 2.5f;  // Very tight spacing
                break;
            default:
                ce_error_margin = 1.0f;
        }

        for (size_t i = 0; i < equalized.size(); ++i) {
            const auto& sym = equalized[i];
            // Use per-carrier noise variance with CE error margin
            float base_nv = (i < carrier_noise_var.size()) ? carrier_noise_var[i] : noise_variance;
            float nv = base_nv * ce_error_margin;

            switch (mod) {
                case Modulation::BPSK:
                    soft_bits.push_back(softDemapBPSK(sym, nv));
                    break;
                case Modulation::QPSK: {
                    auto llrs = softDemapQPSK(sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM16: {
                    auto llrs = softDemapQAM16(sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM32: {
                    auto llrs = softDemapQAM32(sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM64: {
                    auto llrs = softDemapQAM64(sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM256: {
                    auto llrs = softDemapQAM256(sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                default: {
                    auto llrs = softDemapQPSK(sym, nv);
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
        // Must be large enough for biggest expected frame + preamble (~5s for large files)
        constexpr size_t ABSOLUTE_MAX_BUFFER = 240000;  // ~5s at 48kHz
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
        size_t energy_skips = 0;
        size_t i = impl_->search_offset;

        // Track max correlation for debugging
        float max_corr_found = 0.0f;
        size_t max_corr_offset = 0;

        while (i + preamble_total_len < impl_->rx_buffer.size() && iterations < MAX_ITERATIONS) {
            // Quick energy check - skip silent regions entirely
            if (!impl_->hasMinimumEnergy(i, correlation_window)) {
                i += correlation_window / 2;  // Skip ahead by half window
                ++iterations;
                ++energy_skips;
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
                // Found sync point - use this offset
                // NOTE: We previously had "fine timing refinement" that searched ±16
                // samples for the peak, but this caused problems: in clean channels,
                // tiny numerical noise (0.999 vs 1.001) led to choosing wrong offsets.
                // The first-above-threshold approach is more robust.
                found_sync = true;
                sync_offset = i;
                sync_corr = corr;
                break;
            }

            i += STEP_SIZE;
            ++iterations;
        }

        // Update search position for next frame (resume from here)
        impl_->search_offset = i;

        if (found_sync) {
            // Estimate coarse CFO from preamble BEFORE discarding it
            float coarse_cfo = impl_->estimateCoarseCFO(sync_offset);

            // Initialize frequency offset with coarse estimate
            // This gives us a head start on tracking
            impl_->freq_offset_hz = coarse_cfo;
            impl_->freq_offset_filtered = coarse_cfo;
            impl_->freq_correction_phase = 0.0f;  // Reset correction phase

            LOG_SYNC(INFO, "SYNC FOUND: corr=%.3f at offset %zu, CFO=%.1f Hz, buffer=%zu samples",
                     sync_corr, sync_offset, coarse_cfo, impl_->rx_buffer.size());

            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + sync_offset + preamble_total_len);
            impl_->search_offset = 0;  // Reset for next search
            impl_->state.store(Impl::State::SYNCED);
            impl_->synced_symbol_count.store(0);  // Reset timeout counter
            impl_->mixer.reset();
        }
    }

    if (impl_->state.load() == Impl::State::SYNCED) {
        // === Preamble detection while synced ===
        // Check if a new transmission's preamble has arrived. This happens when:
        // 1. Previous transmission ended (we finished decoding or timed out)
        // 2. New transmission started with fresh preamble
        //
        // If we detect a preamble, sync directly to it (don't defer to SEARCHING).
        // This is critical for test mode where we can't wait for another process() call.
        //
        // Note: The preamble might not be at offset 0 - there could be trailing
        // garbage from the previous transmission. Search a reasonable window.
        size_t preamble_symbol_len = impl_->config.fft_size + impl_->config.getCyclicPrefix();
        size_t preamble_total_len = preamble_symbol_len * 6;  // 4 STS + 2 LTS

        // Only check for new preamble when we're IDLE (not actively receiving a frame)
        // This prevents false preamble detection mid-frame when data correlates with preamble
        // "Idle" means: we've processed at least one symbol AND been idle for multiple calls
        // Note: Don't check right after initial sync (synced_symbol_count == 0) - that causes
        // false detection when data correlates with preamble sequence
        bool should_check_preamble = impl_->synced_symbol_count.load() > 0 &&
                                      impl_->idle_call_count.load() >= 2;

        if (should_check_preamble && impl_->rx_buffer.size() >= preamble_total_len) {
            // Search for preamble in the first portion of buffer
            // Limit search to avoid wasting CPU on every call
            size_t search_limit = std::min(impl_->rx_buffer.size() - preamble_total_len,
                                           impl_->symbol_samples * 2);  // Max 2 symbols worth
            constexpr size_t STEP = 8;

            for (size_t offset = 0; offset <= search_limit; offset += STEP) {
                float corr = impl_->measureCorrelation(offset);
                if (corr > impl_->sync_threshold) {
                    // Found new preamble - use this offset directly
                    // (No fine timing refinement - see note in SEARCHING state)
                    size_t best_offset = offset;

                    // Sync directly to the new preamble (same as SEARCHING state does)
                    // Estimate coarse CFO from preamble
                    float coarse_cfo = impl_->estimateCoarseCFO(best_offset);
                    impl_->freq_offset_hz = coarse_cfo;
                    impl_->freq_offset_filtered = coarse_cfo;
                    impl_->freq_correction_phase = 0.0f;

                    // Remove preamble and any garbage before it
                    impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                           impl_->rx_buffer.begin() + best_offset + preamble_total_len);

                    // Reset ALL state for new transmission (critical for correct decode!)
                    LOG_SYNC(WARN, "Mid-frame preamble detected at offset %zu (had %zu soft bits)! Clearing state.",
                             best_offset, impl_->soft_bits.size());
                    impl_->soft_bits.clear();
                    impl_->synced_symbol_count.store(0);
                    impl_->idle_call_count.store(0);
                    impl_->mixer.reset();

                    // Reset channel estimation state (must start fresh for new transmission)
                    std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
                    impl_->snr_symbol_count = 0;
                    impl_->estimated_snr_linear = 1.0f;
                    impl_->noise_variance = 0.1f;
                    impl_->prev_pilot_phases.clear();

                    // Reset adaptive equalizer state
                    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
                    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
                    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);

                    // Continue processing in SYNCED state (don't return!)
                    break;
                }
            }
        }

        size_t soft_bits_before = impl_->soft_bits.size();
        size_t symbols_to_process = impl_->rx_buffer.size() / impl_->symbol_samples;
        LOG_DEMOD(DEBUG, "SYNCED: buffer=%zu samples, symbol_samples=%zu, can process %zu symbols",
                  impl_->rx_buffer.size(), impl_->symbol_samples, symbols_to_process);

        // Process ALL complete symbols in the buffer (multi-codeword support)
        // Don't stop at LDPC_BLOCK_SIZE - keep processing to handle large frames
        size_t symbols_processed = 0;
        while (impl_->rx_buffer.size() >= impl_->symbol_samples) {
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
            ++symbols_processed;

            impl_->updateQuality();

            // Timeout check: if we've processed too many symbols without completing,
            // this was likely a false sync - reset to searching
            int sym_count = ++impl_->synced_symbol_count;
            if (sym_count > Impl::MAX_SYMBOLS_BEFORE_TIMEOUT) {
                LOG_SYNC(WARN, "Sync timeout after %d symbols (%zu soft bits accumulated), resetting to SEARCHING",
                         sym_count, impl_->soft_bits.size());
                impl_->state.store(Impl::State::SEARCHING);
                // DON'T clear soft bits here - let caller extract whatever we have
                // The caller can try to decode partial data
                impl_->synced_symbol_count.store(0);
                impl_->idle_call_count.store(0);
                // Return true if we have any codewords worth of data - give caller a chance to decode
                return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
            }
        }

        if (symbols_processed > 0) {
            float snr_db = (impl_->estimated_snr_linear > 0)
                ? 10.0f * std::log10(impl_->estimated_snr_linear) : 0.0f;
            LOG_DEMOD(INFO, "Processed %zu symbols, soft_bits=%zu (+%zu), SNR=%.1f dB, CFO=%.1f Hz",
                      symbols_processed, impl_->soft_bits.size(),
                      impl_->soft_bits.size() - soft_bits_before,
                      snr_db, impl_->freq_offset_hz);
        }

        // Track idle calls: if we're synced but not accumulating soft bits,
        // the transmission might have ended
        if (impl_->soft_bits.size() == soft_bits_before) {
            int idle = ++impl_->idle_call_count;
            if (idle > Impl::MAX_IDLE_CALLS_BEFORE_RESET) {
                LOG_SYNC(DEBUG, "Idle timeout after %d calls (%zu soft bits), resetting to SEARCHING",
                         idle, impl_->soft_bits.size());
                impl_->state.store(Impl::State::SEARCHING);
                // DON'T clear soft bits - let caller extract whatever we have
                impl_->synced_symbol_count.store(0);
                impl_->idle_call_count.store(0);
                // Return true if we have data worth decoding
                return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
            }
        } else {
            // Made progress - reset idle counter
            impl_->idle_call_count.store(0);
        }

        // Return true if we have enough soft bits for an LDPC codeword
        bool has_codeword = impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;

        // Frame completion detection: Reset to SEARCHING when frame is truly done.
        //
        // CRITICAL: We must only reset when we're TRULY IDLE (empty span passed), not
        // just when symbols_processed == 0. There are two cases where symbols_processed == 0:
        // 1. Empty span passed (checking for more codewords) - truly idle
        // 2. Samples added but not enough for a symbol (partial buffered) - NOT idle
        //
        // We distinguish by checking if samples were actually passed to us.
        // If samples.empty() && symbols_processed == 0 → truly idle, OK to reset.
        // If !samples.empty() && symbols_processed == 0 → partial symbol buffered, don't reset.
        //
        // Conditions for frame complete:
        // 1. Don't have enough bits for a codeword
        // 2. Have processed symbols in the past (synced_symbol_count > 0)
        // 3. Didn't process any new symbols THIS CALL (symbols_processed == 0)
        // 4. No new samples were passed (samples.empty()) - truly idle, not buffering
        //
        // Note: Back-to-back frames are also handled by preamble detection while
        // synced, which triggers when idle_call_count >= 2.
        bool truly_idle = samples.empty() && symbols_processed == 0;
        if (!has_codeword && impl_->synced_symbol_count.load() > 0 && truly_idle) {
            LOG_DEMOD(INFO, "Frame complete (only %zu leftover bits, truly idle), resetting to SEARCHING",
                      impl_->soft_bits.size());
            impl_->state.store(Impl::State::SEARCHING);
            impl_->synced_symbol_count.store(0);
            impl_->idle_call_count.store(0);
            impl_->soft_bits.clear();  // Discard padding bits
        }

        LOG_DEMOD(DEBUG, "process: soft_bits=%zu, returning %s",
                  impl_->soft_bits.size(), has_codeword ? "true" : "false");
        return has_codeword;
    }

    LOG_DEMOD(DEBUG, "process: not synced, returning false");
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
    LOG_DEMOD(DEBUG, "getSoftBits: buffer has %zu soft bits", impl_->soft_bits.size());

    // Trace: print first 24 LLRs and their hard decisions
    if (g_log_level >= LogLevel::TRACE && g_log_categories.demod && impl_->soft_bits.size() >= 24) {
        char buf[256];
        int pos = 0;
        for (size_t i = 0; i < 24 && pos < 240; ++i) {
            int bit = (impl_->soft_bits[i] < 0) ? 1 : 0;  // LDPC convention
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%+.1f(%d) ", impl_->soft_bits[i], bit);
            if ((i + 1) % 6 == 0) pos += snprintf(buf + pos, sizeof(buf) - pos, "| ");
        }
        LOG_DEMOD(TRACE, "First 24 LLRs: %s", buf);
    }

    // Return exactly LDPC_BLOCK_SIZE soft bits for one codeword
    // Keep any extras in the buffer (can happen with higher-order modulations
    // like QAM16/QAM64 where we might overshoot by a few bits)
    if (impl_->soft_bits.size() <= LDPC_BLOCK_SIZE) {
        auto bits = std::move(impl_->soft_bits);
        impl_->soft_bits.clear();
        return bits;
    } else {
        // Extract exactly one codeword, preserve extras
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

bool OFDMDemodulator::hasPendingData() const {
    // Has pending data if:
    // 1. We're synced and have accumulated soft bits (partial codeword), OR
    // 2. We're synced and have samples in buffer (more symbols to process)
    if (impl_->state.load() != Impl::State::SYNCED) {
        return false;
    }
    // Check for accumulated soft bits
    if (!impl_->soft_bits.empty()) {
        return true;
    }
    // Check for enough samples for at least one more symbol
    return impl_->rx_buffer.size() >= impl_->symbol_samples;
}

void OFDMDemodulator::reset() {
    impl_->state.store(Impl::State::SEARCHING);
    impl_->synced_symbol_count.store(0);
    impl_->idle_call_count.store(0);
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

    // Reset adaptive equalizer state
    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);
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
