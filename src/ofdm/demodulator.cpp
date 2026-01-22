#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/fec.hpp"  // For LDPC decoder in hunting mode
#include "ultra/logging.hpp"
#include <algorithm>
#include <numeric>
#include <random>
#include <atomic>
#include <mutex>
#include <array>

// LDPC codeword size (same for all code rates in this implementation)
static constexpr size_t LDPC_BLOCK_SIZE = 648;

namespace ultra {

// Soft demapping - returns LLRs (log-likelihood ratios)
namespace {

// LLR clipping to prevent overconfident decisions
// On fading channels, channel estimation errors can produce huge LLRs
// that are confidently WRONG. Clipping allows LDPC to correct them.
// LLR clipping: lower values help LDPC converge when first-symbol errors exist
// LLR clipping to prevent numerical issues in LDPC decoder
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

// DBPSK soft demapping - compares current symbol to previous symbol
// Returns LLR based on phase difference:
//   phase_diff ≈ 0   → bit 0 → positive LLR
//   phase_diff ≈ π   → bit 1 → negative LLR
float softDemapDBPSK(Complex sym, Complex prev_sym, float noise_var) {
    // Compute phase difference: angle(sym * conj(prev))
    // This is equivalent to angle(sym) - angle(prev) but handles wrap-around
    Complex diff = sym * std::conj(prev_sym);
    float phase_diff = std::atan2(diff.imag(), diff.real());

    // LLR based on distance from decision boundaries (0 and π)
    // phase_diff near 0 → bit 0, phase_diff near ±π → bit 1
    //
    // For DBPSK with AWGN, the LLR is approximately:
    //   LLR ≈ 2 * |sym| * |prev_sym| * cos(phase_diff) / noise_var
    //
    // Simplified: use phase_diff directly scaled by SNR
    // phase_diff ∈ [-π, π], decision boundary at ±π/2
    // Map to: LLR positive when |phase_diff| < π/2, negative when > π/2

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        // Very weak signal - return neutral LLR
        return 0.0f;
    }

    // cos(phase_diff) gives distance from boundaries:
    // +1 when phase_diff=0 (bit 0), -1 when phase_diff=π (bit 1)
    float cos_diff = std::cos(phase_diff);

    // Scale by signal power and inverse noise
    // Factor of 2 for BPSK-like scaling, factor of 2 again because differential
    // uses product of two symbols (doubles the noise contribution)
    float llr = 2.0f * signal_power * cos_diff / noise_var;

    return clipLLR(llr);
}

// DQPSK soft demapping - compares current symbol to previous symbol
// Returns 2 LLRs based on phase difference
// TX encoding: 00→0°, 01→90°, 10→180°, 11→270°
std::array<float, 2> softDemapDQPSK(Complex sym, Complex prev_sym, float noise_var) {
    std::array<float, 2> llrs = {0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);
    float phase = std::atan2(diff.imag(), diff.real());  // [-π, π]

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return llrs;  // Neutral LLRs for weak signal
    }

    float scale = 2.0f * signal_power / noise_var;
    static const float pi = 3.14159265358979f;

    // Bit mapping for 00→0°, 01→90°, 10→180°, 11→270°:
    // - bit1 (MSB): 0 for 0°,90°; 1 for 180°,270° → boundaries at 135° and -45°
    // - bit0 (LSB): 0 for 0°,180°; 1 for 90°,270° → boundaries at 45°,135°,225°,315°
    //
    // LLR positive means bit=0, negative means bit=1

    // bit1: use sin(phase + π/4) as soft metric
    // sin(45°)=+0.707 (bit1=0), sin(135°)=+0.707 (bit1=0)
    // sin(225°)=-0.707 (bit1=1), sin(315°)=-0.707 (bit1=1)
    llrs[0] = clipLLR(scale * std::sin(phase + pi/4));

    // bit0: use cos(2*phase) as soft metric
    // cos(0°)=+1 (bit0=0), cos(180°)=-1 (bit0=1)
    // cos(360°)=+1 (bit0=0), cos(540°)=-1 (bit0=1)
    llrs[1] = clipLLR(scale * std::cos(2 * phase));

    return llrs;
}

// D8PSK soft demapping - compares current symbol to previous symbol
// Returns 3 LLRs based on phase difference (8 phases at 45° increments)
// Uses same formula as working single-carrier DPSK (sin-based)
std::array<float, 3> softDemapD8PSK(Complex sym, Complex prev_sym, float noise_var) {
    std::array<float, 3> llrs = {0.0f, 0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);
    float phase_diff = std::atan2(diff.imag(), diff.real());  // [-π, π]

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return llrs;  // Neutral LLRs for weak signal
    }

    // D8PSK: 8 phases at 45° increments
    // 000→0°, 001→45°, 010→90°, 011→135°, 100→180°, 101→225°, 110→270°, 111→315°
    // Natural binary mapping (no Gray code)

    float confidence = signal_power / noise_var;

    // Use sin-based formulas (matches working single-carrier DPSK implementation)
    // bit 2 (MSB): sin(phase) - positive for 0-180°, negative for 180-360°
    // bit 1: sin(2*phase) - period of 180°
    // bit 0 (LSB): sin(4*phase) - period of 90°
    llrs[0] = clipLLR(confidence * std::sin(phase_diff));
    llrs[1] = clipLLR(confidence * std::sin(2.0f * phase_diff));
    llrs[2] = clipLLR(confidence * std::sin(4.0f * phase_diff));

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
    size_t last_sync_offset = 0;   // Last detected sync offset (for testing/debugging)

    // Buffer management constants
    // Preamble = 6 symbols × 560 samples = 3360 samples
    // We need at least this much data to detect a preamble
    static constexpr size_t MIN_SEARCH_SAMPLES = 4000;   // Minimum buffer before searching
    static constexpr size_t MAX_BUFFER_SAMPLES = 240000; // ~5 seconds at 48kHz
    static constexpr size_t OVERLAP_SAMPLES = 20000;     // Keep for boundary-spanning preambles (~400ms PTT)

    // Pre-computed interpolation lookup (avoids O(n²) per symbol)
    struct InterpInfo {
        int fft_idx;        // FFT bin index of this data carrier
        int lower_pilot;    // FFT bin index of pilot below (-1 if none)
        int upper_pilot;    // FFT bin index of pilot above (-1 if none)
        float alpha;        // Interpolation weight: estimate = (1-alpha)*lower + alpha*upper
    };
    std::vector<InterpInfo> interp_table;

    // Frequency offset estimation and correction
    // NOTE: freq_offset_hz is the TOTAL estimated CFO (what we report and correct for)
    // The pilot phase tracking measures the RESIDUAL after correction is applied
    float freq_offset_hz = 0.0f;           // Total estimated frequency offset (applied + residual)
    float freq_offset_filtered = 0.0f;     // Low-pass filtered residual measurement
    std::vector<Complex> prev_pilot_phases; // Previous symbol's pilot values
    static constexpr float FREQ_OFFSET_ALPHA = 0.3f;  // Smoothing factor for tracking (after lock)
    static constexpr int CFO_ACQUISITION_SYMBOLS = 10;  // Use fast alpha for first N symbols
    int symbols_since_sync = 0;            // Counter for adaptive alpha
    float freq_correction_phase = 0.0f;    // Running phase for correction

    // Pilot-based phase tracking for differential modulation (DQPSK/D8PSK)
    Complex pilot_phase_correction = Complex(1, 0);  // Average phase correction

    // Symbol timing recovery - corrects for sample rate offset (SRO)
    // SRO causes FFT window to drift, creating linear phase slope across carriers
    float timing_offset_samples = 0.0f;  // Estimated timing offset in fractional samples
    static constexpr float TIMING_ALPHA = 0.3f;  // Smoothing factor for timing estimate

    // Carrier phase recovery - corrects arbitrary phase offset after sync
    Complex carrier_phase_correction = Complex(1, 0);  // Rotation to apply
    bool carrier_phase_initialized = false;

    // Per-carrier phase from LTS (for DQPSK without pilots)
    // Each carrier may have different phase due to timing offset
    std::vector<Complex> lts_carrier_phases;

    // Phase inversion detection (for audio paths with inverted polarity)
    bool llr_sign_flip = false;  // Flip LLR signs if inverted

    // Manual timing offset adjustment (for debugging/testing)
    int manual_timing_offset = 0;  // Samples to skip (positive) or include (negative) after preamble

    // DBPSK state: previous equalized symbols for differential decoding
    std::vector<Complex> dbpsk_prev_equalized;
    bool dbpsk_first_symbol = true;  // First symbol after sync needs special handling
    bool dqpsk_skip_first_symbol = false;  // Skip first symbol data for DQPSK (use as reference only)

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

            // If use_pilots=false (e.g., DQPSK), all carriers are data
            if (!config.use_pilots) {
                data_carrier_indices.push_back(fft_idx);
            } else {
                if (pilot_count % config.pilot_spacing == 0) {
                    pilot_carrier_indices.push_back(fft_idx);
                } else {
                    data_carrier_indices.push_back(fft_idx);
                }
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

        // DEBUG: Log pilot configuration for comparison with TX
        LOG_DEMOD(DEBUG, "Demod pilot config: %zu pilots, %zu data carriers",
                  pilot_carrier_indices.size(), data_carrier_indices.size());
        if (pilot_carrier_indices.size() >= 3) {
            LOG_DEMOD(DEBUG, "Demod pilot indices[0-2]: %d, %d, %d",
                      pilot_carrier_indices[0], pilot_carrier_indices[1], pilot_carrier_indices[2]);
        }
        if (pilot_sequence.size() >= 3) {
            LOG_DEMOD(DEBUG, "Demod pilot seq[0-2]: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)",
                      pilot_sequence[0].real(), pilot_sequence[0].imag(),
                      pilot_sequence[1].real(), pilot_sequence[1].imag(),
                      pilot_sequence[2].real(), pilot_sequence[2].imag());
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
        // Preamble has RMS ~0.05, silence has RMS ~0.00004
        // Use 0.03 to reliably catch preamble start while rejecting silence
        constexpr float MIN_RMS = 0.03f;
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

    // Real-valued correlation for TIMING detection (no Hilbert edge effects)
    // This gives accurate timing but is sensitive to CFO
    float measureRealCorrelation(size_t offset, float* out_energy = nullptr) {
        if (offset + symbol_samples * 2 > rx_buffer.size()) return 0.0f;

        size_t len = config.fft_size + config.getCyclicPrefix();

        float P = 0.0f;   // Real correlation
        float E1 = 0.0f;  // Energy of first window
        float E2 = 0.0f;  // Energy of second window

        for (size_t i = 0; i < len; ++i) {
            float s1 = rx_buffer[offset + i];
            float s2 = rx_buffer[offset + i + len];
            P += s1 * s2;
            E1 += s1 * s1;
            E2 += s2 * s2;
        }

        if (out_energy) *out_energy = E2;

        // Minimum energy threshold to avoid division by near-zero
        // For 560 samples, 0.001 RMS signal gives energy = 560 * 0.001^2 = 0.00056
        // Use threshold of 0.1 to reject very weak signals where correlation is meaningless
        constexpr float MIN_ENERGY_THRESHOLD = 0.1f;
        if (E1 < MIN_ENERGY_THRESHOLD || E2 < MIN_ENERGY_THRESHOLD) {
            return 0.0f;  // Reject - signal too weak for reliable correlation
        }

        float normalization = std::sqrt(E1 * E2);
        return P / normalization;  // Safe now that we've checked energy
    }

    // ========================================================================
    // SCHMIDL-COX SYNCHRONIZATION
    // ========================================================================
    //
    // The Schmidl-Cox preamble uses only EVEN subcarriers, creating a time-domain
    // symbol where the second half equals the first half: x[n] = x[n + N/2]
    //
    // This enables HALF-SYMBOL correlation:
    //   P(d) = Σ r*(d+n) × r(d+n+N/2)  for n in [0, N/2-1]
    //   R(d) = Σ |r(d+n+N/2)|²          for n in [0, N/2-1]
    //
    // Benefits:
    // 1. CFO range DOUBLES: ±fs/N = ±93.75 Hz (vs ±42.9 Hz with full-symbol)
    // 2. More robust timing metric with plateau shape
    // 3. CFO doesn't degrade correlation magnitude (phase-invariant)

    // Schmidl-Cox correlation: correlate first half with second half of FFT portion
    // Returns normalized correlation magnitude (amplitude-invariant)
    float measureSchmidlCoxCorrelation(size_t offset, Complex* out_P = nullptr, float* out_energy = nullptr) {
        size_t cp_len = config.getCyclicPrefix();
        size_t fft_len = config.fft_size;
        size_t half_len = fft_len / 2;

        // We need: CP + FFT samples
        if (offset + cp_len + fft_len > rx_buffer.size()) {
            if (out_energy) *out_energy = 0.0f;
            return 0.0f;
        }

        // Skip CP, correlate within FFT portion (which has two identical halves)
        size_t data_start = offset + cp_len;

        // Remove DC offset before correlation (handles receiver DC bias)
        float dc_sum = 0.0f;
        for (size_t i = 0; i < fft_len; ++i) {
            dc_sum += rx_buffer[data_start + i];
        }
        float dc_offset = dc_sum / fft_len;

        std::vector<float> dc_removed(fft_len);
        for (size_t i = 0; i < fft_len; ++i) {
            dc_removed[i] = rx_buffer[data_start + i] - dc_offset;
        }

        // Use analytic signal for CFO-invariant correlation magnitude
        auto analytic = toAnalytic(dc_removed.data(), fft_len);

        Complex P(0.0f, 0.0f);
        float R1 = 0.0f;  // Energy of first half
        float R2 = 0.0f;  // Energy of second half

        for (size_t i = 0; i < half_len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + half_len];
            R1 += std::norm(analytic[i]);
            R2 += std::norm(analytic[i + half_len]);
        }

        if (out_P) *out_P = P;
        if (out_energy) *out_energy = R2;  // Return second half energy for backward compat

        // Symmetric normalization: |P| / sqrt(R1 * R2)
        // This is amplitude-invariant - works even if two halves have different amplitudes
        // (e.g., during AGC ramp-up)
        float normalization = std::sqrt(R1 * R2);
        if (normalization < 1e-10f) {
            return 0.0f;
        }

        return std::abs(P) / normalization;
    }

    // Legacy full-symbol correlation for comparison/fallback
    float measureAnalyticCorrelation(size_t offset, Complex* out_P = nullptr, float* out_energy = nullptr) {
        if (offset + symbol_samples * 2 > rx_buffer.size()) return 0.0f;

        size_t len = config.fft_size + config.getCyclicPrefix();
        auto analytic = toAnalytic(&rx_buffer[offset], len * 2);

        Complex P(0.0f, 0.0f);
        float E1 = 0.0f;
        float E2 = 0.0f;

        for (size_t i = 0; i < len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + len];
            E1 += std::norm(analytic[i]);
            E2 += std::norm(analytic[i + len]);
        }

        if (out_P) *out_P = P;
        if (out_energy) *out_energy = E2;

        float normalization = std::sqrt(E1 * E2);
        return std::abs(P) / (normalization + 1e-10f);
    }

    float measureCorrelation(size_t offset, float* out_energy = nullptr) {
        // Use Schmidl-Cox half-symbol correlation for sync detection
        // This works because our STS uses only even subcarriers,
        // creating two identical halves within each symbol.
        return measureSchmidlCoxCorrelation(offset, nullptr, out_energy);
    }

    bool detectSync(size_t offset) {
        float energy = 0;
        float normalized = measureCorrelation(offset, &energy);

        // Require minimum energy to avoid false triggers on transients/noise
        // Energy is sum of squares over N/2 = 256 samples
        // For 0.3 RMS signal: energy = 256 * 0.3^2 = 23
        constexpr float MIN_ENERGY = 20.0f;
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

    // Schmidl-Cox CFO estimation
    // Uses half-symbol correlation within a single STS symbol
    // CFO range: ±fs/N = ±93.75 Hz (for N=512)
    float estimateCoarseCFO(size_t sync_offset) {
        size_t cp_len = config.getCyclicPrefix();
        size_t fft_len = config.fft_size;
        size_t half_len = fft_len / 2;

        // Skip CP, work with FFT portion
        size_t data_start = sync_offset + cp_len;

        if (data_start + fft_len > rx_buffer.size()) {
            return 0.0f;
        }

        // Compute analytic signal for phase-preserving correlation
        auto analytic = toAnalytic(&rx_buffer[data_start], fft_len);

        // Schmidl-Cox correlation: first half with second half
        Complex P(0.0f, 0.0f);
        for (size_t i = 0; i < half_len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + half_len];
        }

        // Extract phase angle
        float phase = std::atan2(P.imag(), P.real());

        // Schmidl-Cox CFO formula:
        // Phase rotation over N/2 samples = 2π × CFO × (N/2) / fs
        // Therefore: CFO = phase × fs / (π × N)
        float cfo_hz = phase * config.sample_rate / (M_PI * fft_len);

        // Clamp to unambiguous range: ±fs/N = ±93.75 Hz
        float max_cfo = config.sample_rate / fft_len;
        cfo_hz = std::max(-max_cfo, std::min(max_cfo, cfo_hz));

        LOG_DEMOD(DEBUG, "Schmidl-Cox CFO: phase=%.3f rad, cfo=%.1f Hz (max ±%.1f Hz)",
                  phase, cfo_hz, max_cfo);

        return cfo_hz;
    }

    // ========================================================================
    // DECODE HUNTING: Try multiple timing offsets and validate with LDPC
    // ========================================================================
    //
    // Instead of trying to nail exact timing from correlation alone, we:
    // 1. Use correlation to find candidate regions (coarse detection)
    // 2. Try demodulating at multiple timing offsets around the candidate
    // 3. Use LDPC decode success as the ultimate validator
    //
    // This is more robust because LDPC R1/4 can tolerate some timing error,
    // and a successful decode proves we have the correct timing.

    // Trial demodulate: demodulate symbols from buffer WITHOUT modifying state
    // Returns soft bits for LDPC validation
    std::vector<float> trialDemodulate(size_t data_start_offset, size_t num_symbols) {
        std::vector<float> trial_soft_bits;

        // Need enough samples for the requested symbols
        size_t samples_needed = num_symbols * symbol_samples;
        if (data_start_offset + samples_needed > rx_buffer.size()) {
            return trial_soft_bits;  // Empty = failed
        }

        // Create a temporary NCO for mixing (don't use main mixer state)
        NCO trial_mixer(config.center_freq, config.sample_rate);

        // Temporary channel estimate (will be initialized from first symbol)
        std::vector<Complex> trial_channel_estimate(config.fft_size, Complex(1, 0));
        bool channel_initialized = false;

        // Temporary DQPSK state
        std::vector<Complex> trial_prev_equalized;

        // Process symbols
        for (size_t sym = 0; sym < num_symbols; ++sym) {
            size_t sym_offset = data_start_offset + sym * symbol_samples;

            // Mix to baseband (no CFO correction in trial - assume small)
            std::vector<Complex> baseband(symbol_samples);
            for (size_t i = 0; i < symbol_samples; ++i) {
                Complex osc = trial_mixer.next();
                baseband[i] = rx_buffer[sym_offset + i] * std::conj(osc);
            }

            // Extract symbol (skip CP, do FFT)
            size_t cp_len = config.getCyclicPrefix();
            std::vector<Complex> symbol(config.fft_size);
            for (size_t i = 0; i < config.fft_size; ++i) {
                symbol[i] = baseband[cp_len + i];
            }
            std::vector<Complex> freq_domain;
            fft.forward(symbol, freq_domain);

            // Update channel estimate from pilots (simplified version)
            if (!channel_initialized) {
                // First symbol: compute channel estimate from pilots
                for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                    int idx = pilot_carrier_indices[i];
                    Complex rx_pilot = freq_domain[idx];
                    Complex tx_pilot = pilot_sequence[i];
                    if (std::abs(tx_pilot) > 1e-6f) {
                        trial_channel_estimate[idx] = rx_pilot / tx_pilot;
                    }
                }
                // Interpolate to data carriers (simplified: use nearest pilot)
                for (int idx : data_carrier_indices) {
                    // Find nearest pilot
                    int nearest = pilot_carrier_indices[0];
                    int min_dist = std::abs(idx - nearest);
                    for (int pidx : pilot_carrier_indices) {
                        int dist = std::abs(idx - pidx);
                        if (dist < min_dist) {
                            min_dist = dist;
                            nearest = pidx;
                        }
                    }
                    trial_channel_estimate[idx] = trial_channel_estimate[nearest];
                }
                channel_initialized = true;
            }

            // Equalize data carriers
            std::vector<Complex> equalized;
            equalized.reserve(data_carrier_indices.size());
            for (int idx : data_carrier_indices) {
                Complex H = trial_channel_estimate[idx];
                float H_mag_sq = std::norm(H);
                if (H_mag_sq < 1e-6f) H_mag_sq = 1e-6f;
                Complex eq = freq_domain[idx] * std::conj(H) / H_mag_sq;
                equalized.push_back(eq);
            }

            // DQPSK demodulation
            if (config.modulation == Modulation::DQPSK) {
                if (!trial_prev_equalized.empty()) {
                    // Differential decode using previous symbol
                    for (size_t i = 0; i < equalized.size() && i < trial_prev_equalized.size(); ++i) {
                        Complex diff = equalized[i] * std::conj(trial_prev_equalized[i]);
                        float phase = std::atan2(diff.imag(), diff.real());
                        float signal_power = std::abs(equalized[i]) * std::abs(trial_prev_equalized[i]);

                        if (signal_power < 1e-6f) {
                            trial_soft_bits.push_back(0.0f);
                            trial_soft_bits.push_back(0.0f);
                        } else {
                            float scale = 2.0f * signal_power / noise_variance;
                            static const float pi = 3.14159265358979f;
                            // bit1 (MSB)
                            trial_soft_bits.push_back(std::max(-10.0f, std::min(10.0f,
                                scale * std::sin(phase + pi/4))));
                            // bit0 (LSB)
                            trial_soft_bits.push_back(std::max(-10.0f, std::min(10.0f,
                                scale * std::cos(2 * phase))));
                        }
                    }
                }
                trial_prev_equalized = equalized;
            } else {
                // BPSK fallback (simplified)
                for (const auto& eq : equalized) {
                    float llr = -2.0f * eq.real() / noise_variance;
                    trial_soft_bits.push_back(std::max(-10.0f, std::min(10.0f, llr)));
                }
            }
        }

        return trial_soft_bits;
    }

    // Hunt result codes
    static constexpr int HUNT_NEED_MORE_SAMPLES = -9999;  // Sentinel: retry when more samples available

    // Hunt for valid codeword at multiple timing offsets around candidate position
    // Returns: (success, best_offset) where offset is relative to candidate
    // If success, the data can be decoded starting at (candidate + preamble_len + best_offset)
    // NOTE: This function is currently unused - kept for potential future use
    std::pair<bool, int> huntForCodeword(size_t candidate_sync_pos) {
        size_t preamble_len = symbol_samples * 6;  // 4 STS + 2 LTS symbols

        // Timing offsets to try (samples relative to nominal data start)
        // Negative = data starts earlier, positive = data starts later
        static const int offsets[] = {0, -50, 50, -100, 100, -150, 150};
        static const size_t num_offsets = sizeof(offsets) / sizeof(offsets[0]);

        // Symbols needed for one codeword (648 bits at 30 bits/symbol for DQPSK)
        // Add +1 because DQPSK needs a reference symbol
        size_t bits_per_symbol = (config.modulation == Modulation::DQPSK) ?
            (data_carrier_indices.size() * 2) : data_carrier_indices.size();
        size_t symbols_needed = (LDPC_BLOCK_SIZE + bits_per_symbol - 1) / bits_per_symbol + 1;

        // Check if we have enough samples for ANY offset attempt
        // If not, signal caller to wait for more samples
        size_t samples_for_hunt = candidate_sync_pos + preamble_len + 150 + symbols_needed * symbol_samples;
        if (samples_for_hunt > rx_buffer.size()) {
            LOG_SYNC(INFO, "Hunt at %zu: need %zu samples, have %zu - waiting for more",
                     candidate_sync_pos, samples_for_hunt, rx_buffer.size());
            return {false, HUNT_NEED_MORE_SAMPLES};
        }

        bool any_ldpc_attempted = false;

        // Try each offset
        for (size_t i = 0; i < num_offsets; ++i) {
            int offset = offsets[i];

            // Calculate data start position
            // Check for underflow when offset is negative
            size_t nominal_data_start = candidate_sync_pos + preamble_len;
            size_t data_start;
            if (offset < 0 && static_cast<size_t>(-offset) > nominal_data_start) {
                continue;  // Would go negative, skip
            }
            data_start = nominal_data_start + offset;

            // Trial demodulate
            auto soft_bits = trialDemodulate(data_start, symbols_needed);

            if (soft_bits.size() < LDPC_BLOCK_SIZE) {
                LOG_SYNC(INFO, "Hunt offset %+d: only %zu soft bits (need %zu)",
                         offset, soft_bits.size(), LDPC_BLOCK_SIZE);
                continue;
            }

            any_ldpc_attempted = true;

            // Try LDPC decode (R1/4)
            LDPCDecoder decoder(CodeRate::R1_4);
            std::vector<float> codeword_bits(soft_bits.begin(),
                                              soft_bits.begin() + LDPC_BLOCK_SIZE);
            auto decoded = decoder.decodeSoft(codeword_bits);

            if (decoded.empty()) {
                LOG_SYNC(INFO, "Hunt offset %+d: LDPC decode failed", offset);
                continue;
            }

            // Check for valid frame magic (v2 magic = 0x55 0x4C or "UL")
            // First 2 bytes of decoded data should be magic
            if (decoded.size() >= 2) {
                uint16_t magic = (static_cast<uint16_t>(decoded[0]) << 8) | decoded[1];
                LOG_SYNC(INFO, "Hunt offset %+d: LDPC OK, magic=0x%04X (bytes: %02X %02X %02X %02X)",
                         offset, magic, decoded[0], decoded[1],
                         decoded.size() > 2 ? decoded[2] : 0,
                         decoded.size() > 3 ? decoded[3] : 0);
                if (magic == 0x554C) {  // "UL" - v2 frame magic
                    LOG_SYNC(INFO, "Hunt SUCCESS at offset %+d: valid v2 frame magic found!", offset);
                    return {true, offset};
                }
            }
        }

        if (!any_ldpc_attempted) {
            LOG_SYNC(DEBUG, "Hunt at %zu: no LDPC attempts made (insufficient samples at all offsets)", candidate_sync_pos);
            return {false, HUNT_NEED_MORE_SAMPLES};
        }

        LOG_SYNC(DEBUG, "Hunt FAILED: no valid codeword found at any offset");
        return {false, 0};
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

            // Apply frequency offset correction (always for CFO > 0.01 Hz)
            // Lower threshold than before (0.1) to ensure CFO correction is applied
            // even for small offsets that would otherwise accumulate phase errors
            if (std::abs(freq_offset_hz) > 0.01f) {
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

        // Carrier phase recovery: on first symbol, compute average phase offset
        // and store correction to apply to all H estimates
        if (!carrier_phase_initialized && !pilot_carrier_indices.empty()) {
            Complex h_avg = h_sum / float(pilot_carrier_indices.size());
            float avg_mag = std::abs(h_avg);
            if (avg_mag > 0.01f) {
                // Correction = conjugate of unit vector in direction of average phase
                // This rotates all H values so average phase becomes 0
                carrier_phase_correction = std::conj(h_avg) / avg_mag;
                carrier_phase_initialized = true;
                LOG_DEMOD(DEBUG, "Carrier phase recovery: avg_phase=%.1f°, applying correction",
                          std::arg(h_avg) * 180.0f / M_PI);
            }
        }

        // Apply carrier phase correction to all H estimates
        for (size_t i = 0; i < h_ls_all.size(); ++i) {
            h_ls_all[i] *= carrier_phase_correction;
        }
        h_sum *= carrier_phase_correction;

        // DEBUG: Log first symbol's pilot analysis - ALL pilots (verbose, use DEBUG level)
        if (soft_bits.empty()) {
            LOG_DEMOD(DEBUG, "=== First symbol pilot analysis ===");
            for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                int idx = pilot_carrier_indices[i];
                LOG_DEMOD(DEBUG, "Pilot[%zu] idx=%d: tx=(%.1f,%.1f) rx=(%.2f,%.2f) H=(%.2f,%.2f) |H|=%.2f phase=%.1f°",
                          i, idx,
                          pilot_sequence[i].real(), pilot_sequence[i].imag(),
                          freq_domain[idx].real(), freq_domain[idx].imag(),
                          h_ls_all[i].real(), h_ls_all[i].imag(),
                          std::abs(h_ls_all[i]),
                          std::arg(h_ls_all[i]) * 180.0f / M_PI);
            }
            LOG_DEMOD(DEBUG, "H avg: (%.2f,%.2f), |H|=%.2f, phase=%.1f deg",
                      (h_sum / float(pilot_carrier_indices.size())).real(),
                      (h_sum / float(pilot_carrier_indices.size())).imag(),
                      std::abs(h_sum / float(pilot_carrier_indices.size())),
                      std::arg(h_sum / float(pilot_carrier_indices.size())) * 180.0f / M_PI);
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

        // Fallback: if no previous pilots (first symbol), use a conservative default
        // The old method (comparing pilots to average) confuses frequency-selective
        // channel response with noise, giving falsely low SNR on non-flat channels.
        // Instead, use the signal power with an assumed moderate SNR (15 dB).
        if (noise_count == 0) {
            // First symbol: assume 15 dB SNR as default until we have inter-symbol data
            // SNR = signal_power / noise_power => noise_power = signal_power / SNR
            // 15 dB = 31.6 linear
            noise_power_sum = signal_power / 31.6f;
            noise_count = 1;
        }

        // === Frequency offset estimation from pilot phase differences ===
        // Compare pilot phases between consecutive symbols
        if (!prev_pilot_phases.empty() && prev_pilot_phases.size() == h_ls_all.size()) {
            Complex phase_diff_sum(0, 0);  // Use complex sum for better averaging
            int valid_count = 0;

            for (size_t i = 0; i < h_ls_all.size(); ++i) {
                // Phase difference = current * conj(previous)
                Complex diff = h_ls_all[i] * std::conj(prev_pilot_phases[i]);

                // Skip pilots with very low magnitude (unreliable)
                if (std::norm(prev_pilot_phases[i]) > 1e-6f &&
                    std::norm(h_ls_all[i]) > 1e-6f) {
                    // Normalize to unit vector before averaging
                    float mag = std::abs(diff);
                    if (mag > 1e-6f) {
                        phase_diff_sum += diff / mag;
                        valid_count++;
                    }
                }
            }

            if (valid_count > 0) {
                // Average unit vectors → gives average phase rotation as complex number
                Complex avg_diff = phase_diff_sum / static_cast<float>(valid_count);
                float avg_phase_diff = std::atan2(avg_diff.imag(), avg_diff.real());

                // Store the INVERSE rotation for correction
                // This will be applied to data carriers before differential decoding
                pilot_phase_correction = Complex(std::cos(-avg_phase_diff), std::sin(-avg_phase_diff));

                // Convert phase difference to frequency offset
                // Δf = Δφ / (2π * T_symbol)
                // NOTE: This measures the RESIDUAL CFO after our correction is applied
                float symbol_duration = static_cast<float>(config.getSymbolDuration()) /
                                       static_cast<float>(config.sample_rate);
                float residual_cfo = avg_phase_diff / (2.0f * M_PI * symbol_duration);

                // TOTAL CFO = currently applied correction + measured residual
                // This fixes the feedback loop that was causing 50% underestimation
                float total_cfo = freq_offset_hz + residual_cfo;

                // Use adaptive alpha: fast acquisition for first N symbols, then slow tracking
                // This prevents oscillation that occurs if alpha drops too quickly
                float adaptive_alpha = FREQ_OFFSET_ALPHA;
                if (symbols_since_sync < CFO_ACQUISITION_SYMBOLS) {
                    // During acquisition, use high alpha for fast convergence
                    // Decay from 0.9 to FREQ_OFFSET_ALPHA over the acquisition period
                    float progress = static_cast<float>(symbols_since_sync) / CFO_ACQUISITION_SYMBOLS;
                    adaptive_alpha = 0.9f * (1.0f - progress) + FREQ_OFFSET_ALPHA * progress;
                }
                // Also boost alpha if residual is very large (re-acquisition)
                if (std::abs(residual_cfo) > 10.0f) {
                    adaptive_alpha = std::max(adaptive_alpha, 0.9f);
                }
                symbols_since_sync++;

                // Low-pass filter the TOTAL estimate
                freq_offset_filtered = adaptive_alpha * total_cfo +
                                      (1.0f - adaptive_alpha) * freq_offset_filtered;

                // Clamp to Schmidl-Cox unambiguous range (±fs/N ≈ ±93 Hz for N=512)
                freq_offset_hz = std::max(-90.0f, std::min(90.0f, freq_offset_filtered));

                LOG_DEMOD(TRACE, "Freq offset: residual=%.2f Hz, total=%.2f Hz, filtered=%.2f Hz",
                         residual_cfo, total_cfo, freq_offset_hz);
            }
        } else {
            // First symbol - no phase correction available yet
            pilot_phase_correction = Complex(1, 0);
        }

        // === Symbol timing recovery from pilot phase slope ===
        // Timing offset τ causes linear phase slope: phase(k) = 2π × k × τ / N
        // Use linear regression on pilot phases to estimate τ
        {
            // Collect (fft_index, phase) pairs for linear regression
            float sum_k = 0, sum_k2 = 0, sum_phase = 0, sum_k_phase = 0;
            int timing_valid_count = 0;

            for (size_t i = 0; i < h_ls_all.size(); ++i) {
                if (std::norm(h_ls_all[i]) < 1e-6f) continue;

                int k = pilot_carrier_indices[i];
                // Unwrap to centered FFT index for correct slope
                if (k > (int)config.fft_size / 2) k -= config.fft_size;

                float phase = std::arg(h_ls_all[i]);

                sum_k += k;
                sum_k2 += k * k;
                sum_phase += phase;
                sum_k_phase += k * phase;
                timing_valid_count++;
            }

            if (timing_valid_count >= 3) {  // Need at least 3 pilots for reliable slope
                // Linear regression: slope = (n*sum_xy - sum_x*sum_y) / (n*sum_x2 - sum_x^2)
                float n = timing_valid_count;
                float denom = n * sum_k2 - sum_k * sum_k;
                if (std::abs(denom) > 1e-6f) {
                    float slope = (n * sum_k_phase - sum_k * sum_phase) / denom;

                    // slope = 2π × τ / N  →  τ = slope × N / (2π)
                    float instantaneous_timing = slope * config.fft_size / (2.0f * M_PI);

                    // Low-pass filter to smooth timing estimate
                    timing_offset_samples = TIMING_ALPHA * instantaneous_timing +
                                           (1.0f - TIMING_ALPHA) * timing_offset_samples;

                    // Clamp to reasonable range (±half symbol shouldn't happen)
                    timing_offset_samples = std::max(-50.0f, std::min(50.0f, timing_offset_samples));

                    LOG_DEMOD(DEBUG, "Timing recovery: instant=%.2f samp, filtered=%.2f samp (slope=%.4f rad/bin)",
                             instantaneous_timing, timing_offset_samples, slope);
                }
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
        // Uses linear interpolation between adjacent pilots, BUT:
        // If phase difference between pilots is >90°, use nearest pilot instead.
        // Large phase jumps indicate frequency-selective nulls where interpolation
        // would give completely wrong estimates.
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

                // Check phase difference between pilots
                // phase_diff = angle(H2 * conj(H1)) gives the phase difference
                Complex phase_diff_complex = H2 * std::conj(H1);
                float phase_diff = std::abs(std::atan2(phase_diff_complex.imag(), phase_diff_complex.real()));

                // If phase difference > 90°, interpolation will be wrong
                // Use nearest pilot instead
                constexpr float PHASE_THRESHOLD = 1.5708f;  // π/2 = 90°
                if (phase_diff > PHASE_THRESHOLD) {
                    // Use nearest pilot based on alpha (interpolation weight)
                    if (info.alpha < 0.5f) {
                        channel_estimate[info.fft_idx] = H1;  // Closer to lower pilot
                    } else {
                        channel_estimate[info.fft_idx] = H2;  // Closer to upper pilot
                    }
                    LOG_DEMOD(DEBUG, "Nearest-pilot for bin %d: phase_diff=%.1f°",
                              info.fft_idx, phase_diff * 180.0f / 3.14159f);
                } else {
                    // Normal linear interpolation of complex channel estimate
                    channel_estimate[info.fft_idx] = (1.0f - info.alpha) * H1 + info.alpha * H2;
                }
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

        // For differential modulation (DBPSK, DQPSK, D8PSK), skip equalization entirely!
        // The channel phase cancels out in differential decoding: H_n * conj(H_{n-1}) ≈ |H|²
        // Equalizing introduces phase errors from poor channel estimation that DON'T cancel.
        bool is_differential = (mod == Modulation::DBPSK || mod == Modulation::DQPSK || mod == Modulation::D8PSK);

        if (is_differential) {
            // Return raw received symbols for differential decoding
            // Apply two corrections:
            // 1. Pilot-based phase correction: removes common-mode phase rotation (CFO, channel drift)
            // 2. Timing correction: removes per-carrier phase slope from sample rate offset (SRO)
            //
            // Timing offset τ samples causes phase rotation: exp(-j × 2π × k × τ / N) per carrier k
            // We correct by multiplying by exp(+j × 2π × k × τ / N)
            for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
                int idx = data_carrier_indices[i];

                // Calculate per-carrier timing correction
                // Use centered FFT index for correct sign
                int k = idx;
                if (k > (int)config.fft_size / 2) k -= config.fft_size;

                // Timing correction: exp(+j × 2π × k × τ / N)
                float timing_phase = 2.0f * M_PI * k * timing_offset_samples / config.fft_size;
                Complex timing_correction = std::exp(Complex(0, timing_phase));

                // Apply both corrections: pilot phase (common-mode) + timing (per-carrier)
                equalized[i] = freq_domain[idx] * pilot_phase_correction * timing_correction;

                // Estimate noise variance from channel estimate amplitude
                // (use channel estimate just for noise scaling, not phase correction)
                float h_power = std::norm(channel_estimate[idx]);
                if (h_power < 1e-6f) {
                    carrier_noise_var[i] = 100.0f;  // Weak carrier
                } else {
                    carrier_noise_var[i] = noise_variance / h_power;
                    carrier_noise_var[i] = std::max(1e-6f, std::min(100.0f, carrier_noise_var[i]));
                }
            }
            return equalized;
        }

        bool use_adaptive = config.adaptive_eq_enabled;

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            Complex received = freq_domain[idx];

            if (use_adaptive) {
                // Adaptive equalization using LMS/RLS weights with MMSE
                Complex h = lms_weights[idx];
                float h_power = std::norm(h);

                // MMSE equalization: y_eq = H* × y / (|H|² + σ²)
                // Unlike ZF (y/H), MMSE doesn't amplify noise on weak carriers
                // This is critical for soundcards with poor frequency response
                float mmse_denom = h_power + noise_variance;
                if (mmse_denom < 1e-10f) {
                    equalized[i] = Complex(0, 0);
                    carrier_noise_var[i] = 100.0f;  // High uncertainty
                } else {
                    equalized[i] = std::conj(h) * received / mmse_denom;
                    // MMSE noise variance (approximation - conservative estimate)
                    carrier_noise_var[i] = noise_variance / (h_power + 1e-6f);
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
                // MMSE equalization with per-carrier noise tracking
                // MMSE: y_eq = H* × y / (|H|² + σ²)
                // Unlike ZF (y/H), MMSE gracefully handles weak carriers by not
                // amplifying noise excessively. This is critical for soundcards
                // with non-flat frequency response (e.g., 20+ dB variation).
                Complex h = channel_estimate[idx];
                float h_power = std::norm(h);

                // MMSE denominator includes noise variance to prevent blow-up
                float mmse_denom = h_power + noise_variance;
                if (mmse_denom < 1e-10f) {
                    // Deep fade - output zero with high uncertainty
                    equalized[i] = Complex(0, 0);
                    carrier_noise_var[i] = 100.0f;  // Erasure: LLRs ≈ 0
                } else {
                    // MMSE equalization
                    equalized[i] = std::conj(h) * received / mmse_denom;
                    // Per-carrier noise variance (conservative estimate)
                    // This makes deeply faded carriers give low-confidence LLRs
                    carrier_noise_var[i] = noise_variance / (h_power + 1e-6f);
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
        // Also detect phase inversion based on symbol sign distribution
        if (soft_bits.empty() && !equalized.empty()) {
            // For BPSK, symbols should be near +1 or -1 on real axis
            // Count how many are positive vs negative to detect phase inversion
            int pos_count = 0, neg_count = 0;
            for (const auto& s : equalized) {
                if (s.real() > 0) pos_count++;
                else neg_count++;
            }
            LOG_DEMOD(DEBUG, "First symbol stats: %zu carriers, %d positive, %d negative, first 3: (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)",
                    equalized.size(), pos_count, neg_count,
                    equalized[0].real(), equalized[0].imag(),
                    equalized.size() > 1 ? equalized[1].real() : 0.0f,
                    equalized.size() > 1 ? equalized[1].imag() : 0.0f,
                    equalized.size() > 2 ? equalized[2].real() : 0.0f,
                    equalized.size() > 2 ? equalized[2].imag() : 0.0f);


            // Phase inversion detection DISABLED
            // This heuristic causes false positives with legitimate data patterns:
            // - All-ones (0xFF) has 100% positive symbols
            // - High-bit-density data like DEADBEEF has 80%+ positive symbols
            // Without LDPC scrambling, any raw data can have extreme bias.
            // The correct approach is to handle phase ambiguity at a higher layer
            // (e.g., LDPC decoder, or explicit sync word).
            //
            // TODO: Re-enable only for LDPC-coded data where scrambling ensures ~50/50 distribution
            llr_sign_flip = false;
            float neg_ratio = float(neg_count) / equalized.size();
            LOG_DEMOD(DEBUG, "Symbol polarity: %.0f%% negative, %.0f%% positive (no flip)",
                      neg_ratio * 100, (1.0f - neg_ratio) * 100);
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
            case Modulation::DBPSK:  // Differential modes are robust to phase errors
            case Modulation::DQPSK:
            case Modulation::BPSK:
            case Modulation::QPSK:
                ce_error_margin = 1.0f;  // Robust to small errors
                break;
            case Modulation::D8PSK:  // 8PSK has tighter spacing (45° vs 90°)
            case Modulation::QAM8:
                ce_error_margin = 1.1f;  // Small margin for phase noise
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

        // Sign flip multiplier for phase inversion correction
        float llr_sign = llr_sign_flip ? -1.0f : 1.0f;

        // DQPSK/D8PSK: Initialize reference for differential decoding
        // DQPSK/D8PSK: Initialize reference for differential decoding
        // TX starts with reference (1,0), so RX should too
        if ((mod == Modulation::DQPSK || mod == Modulation::D8PSK) && dbpsk_prev_equalized.empty()) {
            dbpsk_prev_equalized.assign(equalized.size(), Complex(1, 0));
            LOG_DEMOD(DEBUG, "DQPSK: Initialized reference with (1,0)");
        }

        for (size_t i = 0; i < equalized.size(); ++i) {
            const auto& sym = equalized[i];
            // Use per-carrier noise variance with CE error margin
            float base_nv = (i < carrier_noise_var.size()) ? carrier_noise_var[i] : noise_variance;
            float nv = base_nv * ce_error_margin;

            switch (mod) {
                case Modulation::DBPSK: {
                    // DBPSK: compare to previous symbol for differential decoding
                    // Initialize reference on first use
                    if (dbpsk_prev_equalized.empty()) {
                        dbpsk_prev_equalized.assign(equalized.size(), Complex(1, 0));
                    }
                    Complex prev_sym = dbpsk_prev_equalized[i];
                    float llr = softDemapDBPSK(sym, prev_sym, nv);
                    soft_bits.push_back(llr);  // No sign flip for DBPSK - it's inherently phase-invariant
                    dbpsk_prev_equalized[i] = sym;  // Update for next symbol
                    break;
                }
                case Modulation::DQPSK: {
                    // DQPSK: compare to previous symbol, outputs 2 bits
                    // Note: first symbol handled above (establishes reference)
                    Complex prev_sym = dbpsk_prev_equalized[i];
                    auto llrs = softDemapDQPSK(sym, prev_sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());

                    // Debug: log differential phase for first few carriers on early symbols
                    if (snr_symbol_count < 5 && i < 3) {
                        Complex diff = sym * std::conj(prev_sym);
                        float diff_phase = std::atan2(diff.imag(), diff.real()) * 180.0f / M_PI;
                        LOG_DEMOD(DEBUG, "DQPSK sym=%d car=%zu: phase=%.1f° LLRs=[%.2f,%.2f]",
                                 snr_symbol_count, i, diff_phase, llrs[0], llrs[1]);
                    }

                    dbpsk_prev_equalized[i] = sym;
                    break;
                }
                case Modulation::D8PSK: {
                    // D8PSK: compare to previous symbol, outputs 3 bits
                    // Note: first symbol handled above (establishes reference)
                    Complex prev_sym = dbpsk_prev_equalized[i];
                    auto llrs = softDemapD8PSK(sym, prev_sym, nv);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    dbpsk_prev_equalized[i] = sym;
                    break;
                }
                case Modulation::BPSK:
                    soft_bits.push_back(softDemapBPSK(sym, nv) * llr_sign);
                    break;
                case Modulation::QPSK: {
                    auto llrs = softDemapQPSK(sym, nv);
                    for (auto& llr : llrs) llr *= llr_sign;
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM16: {
                    auto llrs = softDemapQAM16(sym, nv);
                    for (auto& llr : llrs) llr *= llr_sign;
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM32: {
                    auto llrs = softDemapQAM32(sym, nv);
                    for (auto& llr : llrs) llr *= llr_sign;
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM64: {
                    auto llrs = softDemapQAM64(sym, nv);
                    for (auto& llr : llrs) llr *= llr_sign;
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM256: {
                    auto llrs = softDemapQAM256(sym, nv);
                    for (auto& llr : llrs) llr *= llr_sign;
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                default: {
                    auto llrs = softDemapQPSK(sym, nv);
                    for (auto& llr : llrs) llr *= llr_sign;
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
    // ============================================================
    // SIMPLE BUFFER STRATEGY: Accumulate → Search → Consume
    //
    // 1. Append incoming samples to buffer
    // 2. If buffer >= minimum, search for preamble from index 0
    // 3. If preamble found → sync → consume processed samples
    // 4. If buffer too large → keep only overlap for boundary cases
    //
    // Always search from index 0 - no offset tracking needed
    // ============================================================

    // Add to buffer
    impl_->rx_buffer.insert(impl_->rx_buffer.end(), samples.begin(), samples.end());

    // Preamble size constants
    size_t preamble_symbol_len = impl_->config.fft_size + impl_->config.getCyclicPrefix();
    size_t preamble_total_len = preamble_symbol_len * 6;  // 4 STS + 2 LTS = 3360 samples
    size_t correlation_window = preamble_symbol_len * 2;  // Window for autocorrelation

    // State machine
    if (impl_->state.load() == Impl::State::SEARCHING) {
        // Need minimum data before searching
        if (impl_->rx_buffer.size() < impl_->MIN_SEARCH_SAMPLES) {
            return false;  // Wait for more samples
        }

        // Buffer overflow protection - keep only recent data + overlap
        if (impl_->rx_buffer.size() > impl_->MAX_BUFFER_SAMPLES) {
            size_t keep = impl_->OVERLAP_SAMPLES;
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.end() - keep);
            LOG_SYNC(WARN, "Buffer overflow, trimmed to %zu samples", keep);
        }

        // Search for preamble from beginning of buffer
        bool found_sync = false;
        size_t sync_offset = 0;
        float sync_corr = 0;

        constexpr size_t STEP_SIZE = 8;  // Check every 8th sample

        // Search until we run out of buffer
        size_t search_end = (impl_->rx_buffer.size() > preamble_total_len + correlation_window)
                          ? impl_->rx_buffer.size() - preamble_total_len - correlation_window
                          : 0;

        for (size_t i = 0; i < search_end; i += STEP_SIZE) {
            // Quick energy check - skip silent regions
            if (!impl_->hasMinimumEnergy(i, correlation_window)) {
                i += correlation_window / 2 - STEP_SIZE;  // Skip ahead (will add STEP_SIZE in loop)
                continue;
            }

            // Autocorrelation check
            float corr = impl_->measureCorrelation(i);

            if (corr > impl_->sync_threshold) {
                // Timing acquisition using CP-based offset.
                //
                // The STS autocorrelation produces a plateau, not a sharp peak. The optimal
                // timing position is 4.5 × CP_length samples after the coarse detection point.
                // This relationship is derived from preamble geometry:
                //   - Preamble = 2 identical symbols, each of length (FFT + CP)
                //   - Coarse detection (correlation > 0.80) occurs during correlation ramp-up
                //   - Optimal timing (correlation ≈ 1.0) is at the plateau center
                //   - Empirically verified: offset = 4.5 × CP = 216 samples (for CP=48)
                //
                // To distinguish real recordings (with ramp-up) from synthetic signals (no ramp-up):
                // - Check if correlation increases significantly from coarse to coarse+offset
                // - If increase > 0.05: real signal with ramp-up, use offset
                // - If increase <= 0.05: already at plateau, use offset=0
                constexpr float PLATEAU_THRESHOLD = 0.90f;
                constexpr float RAMP_UP_THRESHOLD = 0.01f;  // Min increase to indicate ramp-up
                constexpr size_t SEARCH_WINDOW = 300;
                constexpr size_t MIN_PLATEAU_SAMPLES = 15;

                // CP-based timing offset: 4.5 × CP = (9 × CP) / 2
                const size_t cp_len = impl_->config.getCyclicPrefix();
                const size_t CP_TIMING_OFFSET = (9 * cp_len) / 2;  // 216 for CP=48

                size_t plateau_count = 0;
                float peak_corr = corr;
                size_t peak_pos = i;  // Track WHERE the peak occurs

                for (size_t j = 0; j <= SEARCH_WINDOW && i + j + preamble_total_len < impl_->rx_buffer.size(); j += 8) {
                    float ref_corr = impl_->measureCorrelation(i + j);
                    if (ref_corr >= PLATEAU_THRESHOLD) {
                        plateau_count++;
                    }
                    if (ref_corr > peak_corr) {
                        peak_corr = ref_corr;
                        peak_pos = i + j;  // Remember peak position
                    }
                }

                if (plateau_count >= MIN_PLATEAU_SAMPLES) {
                    found_sync = true;

                    // Schmidl-Cox timing: use peak position directly
                    // The half-symbol correlation peaks at the optimal symbol boundary
                    // (where the CP starts). No additional offset needed.
                    float improvement = peak_corr - corr;
                    sync_offset = peak_pos;
                    sync_corr = peak_corr;

                    LOG_SYNC(INFO, "Preamble: coarse=%zu, peak=%zu, sync=%zu, corr=%.3f/%.3f, impr=%.3f",
                             i, peak_pos, sync_offset, corr, peak_corr, improvement);
                    break;
                }
            }
        }

        if (found_sync) {
            // Estimate coarse CFO from preamble
            float coarse_cfo = impl_->estimateCoarseCFO(sync_offset);
            impl_->freq_offset_hz = coarse_cfo;
            impl_->freq_offset_filtered = coarse_cfo;
            impl_->freq_correction_phase = 0.0f;
            impl_->symbols_since_sync = 0;

            LOG_SYNC(INFO, "SYNC: offset=%zu, corr=%.3f, CFO=%.1f Hz, buffer=%zu",
                     sync_offset, sync_corr, coarse_cfo, impl_->rx_buffer.size());

            impl_->last_sync_offset = sync_offset;

            // Find where STS actually starts by detecting the guard-to-STS transition.
            // The guard is silence (or noise-only), STS has signal energy.
            //
            // Strategy: Scan forward looking for a sharp energy transition.
            // This works for both clean signals (0→signal) and noisy signals (noise→signal+noise).
            constexpr size_t ENERGY_WINDOW = 64;  // Window for energy measurement
            constexpr size_t SEARCH_STEP = 8;  // Step size for scanning
            constexpr float TRANSITION_RATIO = 3.0f;  // Signal should be 3x higher than guard/noise

            // Measure signal energy at sync_offset (we know signal is there)
            float signal_energy = 0;
            for (size_t j = 0; j < ENERGY_WINDOW && sync_offset + j < impl_->rx_buffer.size(); j++) {
                float s = impl_->rx_buffer[sync_offset + j];
                signal_energy += s * s;
            }
            signal_energy /= ENERGY_WINDOW;

            // The guard is BEFORE sync_offset. Search backward to find the guard-to-STS transition.
            // We expect: [noise/silence] [guard=560] [STS=2240] <- sync_offset is somewhere here
            // The guard should be approximately 560-2800 samples before sync_offset.

            // Start search from well before sync_offset (account for STS length + margin)
            size_t search_start = (sync_offset > preamble_symbol_len * 6) ?
                                  sync_offset - preamble_symbol_len * 6 : 0;

            // Measure baseline energy from the start of search region (likely in guard or pre-signal noise)
            float baseline_energy = 0;
            size_t baseline_samples = std::min((size_t)256, sync_offset - search_start);
            for (size_t j = 0; j < baseline_samples && search_start + j < impl_->rx_buffer.size(); j++) {
                float s = impl_->rx_buffer[search_start + j];
                baseline_energy += s * s;
            }
            if (baseline_samples > 0) baseline_energy /= baseline_samples;

            LOG_SYNC(DEBUG, "Guard search: sync_offset=%zu, search_start=%zu, signal_energy=%.6f, baseline_energy=%.6f, ratio=%.1f",
                    sync_offset, search_start, signal_energy, baseline_energy,
                    baseline_energy > 1e-12f ? signal_energy / baseline_energy : 999.0f);

            size_t guard_end = 0;
            bool found_guard = false;

            // If baseline is much lower than signal, there's likely a guard region - find the transition
            // The guard (silence) + noise should be significantly lower than STS (signal) + noise
            if (baseline_energy < signal_energy * 0.3f) {
                // Special case: if baseline is VERY low (long silence before preamble),
                // the Schmidl-Cox peak detection should give us the STS start position.
                // However, the peak position can vary by up to a symbol length depending on
                // signal content (the correlation search has inherent uncertainty).
                // Round to the nearest symbol boundary to ensure consistent timing.
                if (baseline_energy < 1e-5f && signal_energy > 1e-3f) {
                    // Long silence detected - round sync_offset to nearest symbol boundary
                    size_t symbol_size = preamble_symbol_len;
                    size_t rounded = ((sync_offset + symbol_size / 2) / symbol_size) * symbol_size;
                    guard_end = rounded;
                    found_guard = true;
                    LOG_SYNC(DEBUG, "Long silence detected, sync_offset=%zu rounded to %zu",
                            sync_offset, rounded);
                } else {
                    // Normal case: search for energy transition
                    float threshold = std::max(baseline_energy * TRANSITION_RATIO, signal_energy * 0.1f);

                    // Search limit: we need to search PAST sync_offset because:
                    // 1. sync_offset is in the middle of STS plateau
                    // 2. With some noise, the transition might be AT or NEAR sync_offset
                    size_t search_limit = std::min(sync_offset + preamble_symbol_len * 2,
                                                   impl_->rx_buffer.size() - ENERGY_WINDOW);

                    if (search_limit <= search_start + ENERGY_WINDOW) {
                        LOG_SYNC(DEBUG, "search_limit too small, using fallback");
                    } else {
                        for (size_t i = search_start; i + ENERGY_WINDOW < search_limit; i += SEARCH_STEP) {
                            float energy = 0;
                            for (size_t j = 0; j < ENERGY_WINDOW; j++) {
                                float s = impl_->rx_buffer[i + j];
                                energy += s * s;
                            }
                            energy /= ENERGY_WINDOW;

                            if (energy > threshold) {
                                // Found transition - use this position as STS start
                                // Round to NEAREST symbol boundary (not always UP)
                                size_t symbol_size = preamble_symbol_len;
                                size_t rounded_down = (i / symbol_size) * symbol_size;
                                size_t rounded_up = rounded_down + symbol_size;
                                // Pick whichever is closer to detected position
                                if (i - rounded_down <= rounded_up - i) {
                                    guard_end = rounded_down;
                                } else {
                                    guard_end = rounded_up;
                                }
                                found_guard = true;
                                LOG_SYNC(DEBUG, "Found guard→STS transition at %zu (rounded to %zu), energy=%.6f, threshold=%.6f",
                                        i, guard_end, energy, threshold);
                                break;
                            }
                        }
                    }
                }
            }

            if (!found_guard) {
                // No clear guard region found - estimate STS start from sync_offset
                // sync_offset points somewhere in the middle of the STS plateau (M metric window)
                // The Schmidl-Cox M metric peaks at approximately STS_start + 2*symbol_len
                // So: STS_start ≈ sync_offset - 2*symbol_len
                // Round UP to be conservative - better to consume slightly more than leave preamble bits
                size_t estimated_sts_start = (sync_offset > preamble_symbol_len * 2) ?
                                             sync_offset - preamble_symbol_len * 2 : 0;
                // Round UP to nearest symbol boundary to ensure we don't leave any preamble
                guard_end = ((estimated_sts_start + preamble_symbol_len - 1) / preamble_symbol_len) * preamble_symbol_len;
                LOG_SYNC(DEBUG, "No guard detected, estimating STS start at %zu (from sync_offset=%zu)",
                        guard_end, sync_offset);
            }

            size_t sts_start = guard_end;

            // === Extract carrier phase from LTS (for DQPSK without pilots) ===
            // LTS starts after 4 STS symbols: sts_start + 4*symbol_samples
            // We use the first LTS symbol to estimate carrier phase
            bool is_differential = (impl_->config.modulation == Modulation::DQPSK ||
                                    impl_->config.modulation == Modulation::D8PSK ||
                                    impl_->config.modulation == Modulation::DBPSK);
            LOG_SYNC(DEBUG, "LTS phase check: is_differential=%d, use_pilots=%d",
                    is_differential, impl_->config.use_pilots);
            // DISABLED: LTS phase extraction doesn't improve first symbol accuracy
            // The timing offset between LTS and data causes phase mismatch
            if (false && is_differential && !impl_->config.use_pilots) {
                size_t lts_start = sts_start + 4 * impl_->symbol_samples;
                size_t cp_len = impl_->config.getCyclicPrefix();
                size_t symbol_size = impl_->config.fft_size + cp_len;
                LOG_SYNC(DEBUG, "LTS extraction: lts_start=%zu, symbol_size=%zu, buffer_size=%zu",
                        lts_start, symbol_size, impl_->rx_buffer.size());

                if (lts_start + symbol_size <= impl_->rx_buffer.size()) {
                    // Mix LTS to baseband (same as data symbols)
                    impl_->mixer.reset();  // Start from known phase
                    std::vector<Complex> lts_mixed(impl_->config.fft_size);
                    for (size_t i = 0; i < impl_->config.fft_size; ++i) {
                        size_t idx = lts_start + cp_len + i;
                        float sample = impl_->rx_buffer[idx];
                        Complex osc = impl_->mixer.next();
                        lts_mixed[i] = sample * std::conj(osc);
                    }

                    // FFT to frequency domain
                    std::vector<Complex> lts_freq;
                    impl_->fft.forward(lts_mixed, lts_freq);

                    // Extract per-carrier phases from LTS
                    // With timing offset, each carrier has different phase
                    impl_->lts_carrier_phases.clear();
                    impl_->lts_carrier_phases.resize(impl_->data_carrier_indices.size(), Complex(1, 0));

                    int count = 0;
                    for (size_t i = 0; i < impl_->data_carrier_indices.size(); ++i) {
                        int idx = impl_->data_carrier_indices[i];
                        Complex rx = lts_freq[idx];
                        Complex tx = impl_->sync_sequence[i % impl_->sync_sequence.size()];

                        if (std::abs(rx) > 0.1f && std::abs(tx) > 0.1f) {
                            Complex h = rx / tx;  // Channel estimate
                            // Store normalized phase for this carrier
                            impl_->lts_carrier_phases[i] = h / std::abs(h);
                            count++;
                        }
                    }

                    // Debug: show first few phases
                    if (count >= 5) {
                        LOG_SYNC(DEBUG, "LTS carrier phases[0-4]: %.1f° %.1f° %.1f° %.1f° %.1f°",
                                std::arg(impl_->lts_carrier_phases[0]) * 180.0f / M_PI,
                                std::arg(impl_->lts_carrier_phases[1]) * 180.0f / M_PI,
                                std::arg(impl_->lts_carrier_phases[2]) * 180.0f / M_PI,
                                std::arg(impl_->lts_carrier_phases[3]) * 180.0f / M_PI,
                                std::arg(impl_->lts_carrier_phases[4]) * 180.0f / M_PI);
                    }

                    LOG_SYNC(DEBUG, "LTS phase extraction: %d carriers processed", count);
                    if (count >= 10) {  // Need enough carriers for reliable estimate
                        impl_->carrier_phase_initialized = true;
                        LOG_SYNC(DEBUG, "LTS per-carrier phases stored for DQPSK reference");
                    } else {
                        LOG_SYNC(DEBUG, "LTS: Not enough carriers for phase estimation (%d)", count);
                        impl_->lts_carrier_phases.clear();
                    }
                }
            }

            // Consume samples: from STS start + full preamble length (4 STS + 2 LTS)
            // Note: preamble_total_len = 6 * symbol_samples covers STS + LTS but NOT guard
            size_t consume = sts_start + preamble_total_len + impl_->manual_timing_offset;
            LOG_SYNC(DEBUG, "Consume calc: sts_start=%zu + preamble=%zu + offset=%d = %zu (buffer was %zu)",
                    sts_start, preamble_total_len, impl_->manual_timing_offset, consume, impl_->rx_buffer.size());
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + consume);

            // Transition to SYNCED state
            impl_->state.store(Impl::State::SYNCED);
            impl_->synced_symbol_count.store(0);
            impl_->mixer.reset();
            impl_->dbpsk_prev_equalized.clear();
            // Note: carrier_phase_correction may have been set above from LTS
            // Don't clear it here if already initialized
            if (!is_differential || impl_->config.use_pilots) {
                impl_->carrier_phase_initialized = false;
                impl_->carrier_phase_correction = Complex(1, 0);
            }
            impl_->dqpsk_skip_first_symbol = false;
        } else {
            // No preamble found - if buffer is large, trim old data but keep overlap
            if (impl_->rx_buffer.size() > impl_->OVERLAP_SAMPLES * 2) {
                size_t trim = impl_->rx_buffer.size() - impl_->OVERLAP_SAMPLES;
                impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                       impl_->rx_buffer.begin() + trim);
            }
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
                    impl_->symbols_since_sync = 0;

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

                    // Reset DBPSK differential state for new frame
                    impl_->dbpsk_prev_equalized.clear();

                    // Reset carrier phase recovery for new frame
                    impl_->carrier_phase_initialized = false;
                    impl_->carrier_phase_correction = Complex(1, 0);

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

size_t OFDMDemodulator::getLastSyncOffset() const {
    return impl_->last_sync_offset;
}

void OFDMDemodulator::setTimingOffset(int offset) {
    impl_->manual_timing_offset = offset;
}

void OFDMDemodulator::reset() {
    impl_->state.store(Impl::State::SEARCHING);
    impl_->synced_symbol_count.store(0);
    impl_->idle_call_count.store(0);
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
    impl_->symbols_since_sync = 0;
    impl_->prev_pilot_phases.clear();
    impl_->pilot_phase_correction = Complex(1, 0);  // No correction until pilots received

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
