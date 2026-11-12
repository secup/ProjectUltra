#pragma once

// Soft demapping functions for OFDM demodulator
// Computes LLRs (log-likelihood ratios) for various modulation schemes

#include "ultra/types.hpp"
#include "demodulator_constants.hpp"
#include <cmath>
#include <cstdlib>
#include <vector>
#include <array>
#include <algorithm>

namespace ultra {
namespace soft_demap {

using namespace demod_constants;

// =============================================================================
// LLR CLIPPING
// =============================================================================

inline float clipLLR(float llr) {
    float clipped = std::max(-MAX_LLR, std::min(MAX_LLR, llr));
    // Apply minimum magnitude while preserving sign
    if (std::abs(clipped) < MIN_LLR_MAG) {
        clipped = (clipped >= 0) ? MIN_LLR_MAG : -MIN_LLR_MAG;
    }
    return clipped;
}


// =============================================================================
// COHERENT MODULATION DEMAPPERS
// =============================================================================

// BPSK: -1 maps to bit 0, +1 maps to bit 1
// LLR convention: negative = bit 1, positive = bit 0
inline float demapBPSK(Complex sym, float noise_var) {
    return clipLLR(-2.0f * sym.real() / noise_var);
}

// QPSK: I and Q are independent BPSK channels
inline std::vector<float> demapQPSK(Complex sym, float noise_var) {
    float scale = -2.0f * QPSK_SCALE / noise_var;
    return {clipLLR(sym.real() * scale), clipLLR(sym.imag() * scale)};
}

// Coherent 8PSK max-log metrics. Keep this calculation in one place: the same
// posterior metric is used both by the LDPC demapper and by the coherent
// decision-directed channel-update gate. A private copy of this routine used to
// let the two paths silently drift apart.
inline std::array<float, 3> psk8MaxLogDistanceDeltas(Complex sym) {
    static constexpr int gray_bits[8] = {0, 1, 3, 2, 6, 7, 5, 4};
    static const float pi = 3.14159265358979f;

    float dist_sq[8];
    for (int i = 0; i < 8; ++i) {
        const float angle = i * (pi / 4.0f) + pi / 8.0f;
        const Complex ref(std::cos(angle), std::sin(angle));
        const Complex diff = sym - ref;
        dist_sq[i] = std::norm(diff);
    }

    std::array<float, 3> deltas{};
    for (int bit = 0; bit < 3; ++bit) {
        const int mask = 1 << (2 - bit);
        float min_d0 = 1.0e30f;
        float min_d1 = 1.0e30f;
        for (int i = 0; i < 8; ++i) {
            if (gray_bits[i] & mask) {
                min_d1 = std::min(min_d1, dist_sq[i]);
            } else {
                min_d0 = std::min(min_d0, dist_sq[i]);
            }
        }
        deltas[bit] = min_d1 - min_d0;
    }
    return deltas;
}

// soft_demap's coherent scalar demappers receive the post-equalizer variance of
// ONE real component. For circular complex noise with per-real variance sigma^2,
// max-log is
//
//   log P(b=0|y)/P(b=1|y) ~= (d1^2 - d0^2) / (2 sigma^2).
//
// The legacy 8PSK-only coefficient is 2/sigma^2, while the dimensionally correct
// per-real-variance coefficient is 0.5.  Keep the legacy default until a measured
// decoding win exists: an 800-frame Good@20 A/B was exactly tied because normalized
// min-sum is largely invariant to uniform LLR scaling.  The environment override
// keeps the calibrated 0.5 arm available without silently changing production.
inline float psk8MaxLogCoefficient() {
    static const float coeff = []() {
        if (const char* env = std::getenv("ULTRA_8PSK_MAXLOG_COEFF")) {
            const float v = static_cast<float>(std::atof(env));
            if (std::isfinite(v) && v > 0.0f && v <= 4.0f) {
                return v;
            }
        }
        return 2.0f;
    }();
    return coeff;
}

inline std::array<float, 3> demap8PSKUnclipped(
    Complex sym, float noise_var, float max_log_coeff = psk8MaxLogCoefficient()) {
    const float scale = max_log_coeff /
                        std::max(noise_var, MIN_CARRIER_NOISE_VAR);
    const auto deltas = psk8MaxLogDistanceDeltas(sym);
    return {scale * deltas[0], scale * deltas[1], scale * deltas[2]};
}

inline float psk8MinAbsLLRNoClip(Complex sym, float noise_var) {
    const auto llrs = demap8PSKUnclipped(sym, noise_var);
    return std::min(std::abs(llrs[0]),
                    std::min(std::abs(llrs[1]), std::abs(llrs[2])));
}

// Coherent 8PSK soft demapping. This is the absolute-phase version of the
// differential D8PSK Gray constellation: same 22.5 degree offset and same
// bit mapping, but no previous-symbol multiplication.
inline std::vector<float> demap8PSK(Complex sym, float noise_var) {
    const auto raw = demap8PSKUnclipped(sym, noise_var);
    std::vector<float> llrs;
    llrs.reserve(raw.size());
    for (float llr : raw) {
        llrs.push_back(clipLLR(llr));
    }

    return llrs;
}

// 16-QAM soft demapping with correct LLR signs
// Gray code: levels[] = {-3, -1, 3, 1} for bits 00,01,10,11
inline std::vector<float> demapQAM16(Complex sym, float noise_var) {
    std::vector<float> llrs(4);
    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 3,2 of 4-bit word)
    llrs[0] = clipLLR(-scale * I);                      // bit3 (MSB): sign
    llrs[1] = clipLLR(scale * (std::abs(I) - QAM16_THRESHOLD));  // bit2: outer/inner

    // Q bits (bits 1,0 of 4-bit word)
    llrs[2] = clipLLR(-scale * Q);                      // bit1 (MSB): sign
    llrs[3] = clipLLR(scale * (std::abs(Q) - QAM16_THRESHOLD));  // bit0: outer/inner

    return llrs;
}

// 32-QAM BRUTE-FORCE soft demapping (max-log-MAP)
// 8×4 grid: 8 Q levels × 4 I levels = 32 points
inline std::vector<float> demapQAM32(Complex sym, float noise_var) {
    // I levels (2 bits): Gray code 00→-3, 01→-1, 11→+1, 10→+3
    static const float I_LEVELS[4] = {-3, -1, 1, 3};
    static const int I_GRAY[4] = {0, 1, 3, 2};

    // Q levels (3 bits): Gray code
    static const float Q_LEVELS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    static const int Q_GRAY[8] = {0, 1, 3, 2, 6, 7, 5, 4};

    // Build constellation with bit mappings (static initialization)
    struct Point {
        Complex pos;
        int bits;
    };
    // Built once, thread-safely (a function-local static's initializer runs exactly once).
    struct Table {
        Point p[32];
    };
    static const Table table = [&] {
        Table built{};
        for (int qi = 0; qi < 8; ++qi) {
            for (int ii = 0; ii < 4; ++ii) {
                int idx = qi * 4 + ii;
                built.p[idx].pos = Complex(I_LEVELS[ii] * QAM32_SCALE,
                                           Q_LEVELS[qi] * QAM32_SCALE);
                built.p[idx].bits = (Q_GRAY[qi] << 2) | I_GRAY[ii];
            }
        }
        return built;
    }();
    const Point* constellation = table.p;

    std::vector<float> llrs(5);
    float scale_factor = 2.0f / noise_var;

    // For each bit position, find minimum distance to points with bit=0 and bit=1
    for (int b = 0; b < 5; ++b) {
        int bit_mask = 1 << (4 - b);
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

        llrs[b] = clipLLR(scale_factor * (min_dist_1 - min_dist_0));
    }

    return llrs;
}

// 64-QAM soft demapping
inline std::vector<float> demapQAM64(Complex sym, float noise_var) {
    std::vector<float> llrs(6);
    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 5,4,3 of 6-bit word)
    llrs[0] = clipLLR(-scale * I);
    llrs[1] = clipLLR(scale * (std::abs(I) - QAM64_D4));
    llrs[2] = clipLLR(scale * (std::abs(std::abs(I) - QAM64_D4) - QAM64_D2));

    // Q bits (bits 2,1,0 of 6-bit word)
    llrs[3] = clipLLR(-scale * Q);
    llrs[4] = clipLLR(scale * (std::abs(Q) - QAM64_D4));
    llrs[5] = clipLLR(scale * (std::abs(std::abs(Q) - QAM64_D4) - QAM64_D2));

    return llrs;
}

// 256-QAM soft demapping
inline std::vector<float> demapQAM256(Complex sym, float noise_var) {
    std::vector<float> llrs(8);
    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 7,6,5,4 of 8-bit word)
    llrs[0] = clipLLR(-scale * I);
    llrs[1] = clipLLR(scale * (std::abs(I) - QAM256_D8));
    llrs[2] = clipLLR(scale * (std::abs(std::abs(I) - QAM256_D8) - QAM256_D4));
    llrs[3] = clipLLR(scale * (std::abs(std::abs(std::abs(I) - QAM256_D8) - QAM256_D4) - QAM256_D2));

    // Q bits (bits 3,2,1,0 of 8-bit word)
    llrs[4] = clipLLR(-scale * Q);
    llrs[5] = clipLLR(scale * (std::abs(Q) - QAM256_D8));
    llrs[6] = clipLLR(scale * (std::abs(std::abs(Q) - QAM256_D8) - QAM256_D4));
    llrs[7] = clipLLR(scale * (std::abs(std::abs(std::abs(Q) - QAM256_D8) - QAM256_D4) - QAM256_D2));

    return llrs;
}

// =============================================================================
// DIFFERENTIAL MODULATION DEMAPPERS
// =============================================================================

// DBPSK soft demapping - compares current symbol to previous symbol
// Returns LLR based on phase difference:
//   phase_diff ≈ 0   → bit 0 → positive LLR
//   phase_diff ≈ π   → bit 1 → negative LLR
inline float demapDBPSK(Complex sym, Complex prev_sym, float noise_var) {
    Complex diff = sym * std::conj(prev_sym);
    float phase_diff = std::atan2(diff.imag(), diff.real());

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return 0.0f;  // Very weak signal - neutral LLR
    }

    // Differential detection doubles noise: diff = sym*conj(prev) combines noise from both symbols
    float diff_noise_var = 2.0f * noise_var;

    // cos(phase_diff) gives distance from boundaries
    // No fixed cap here — noise_var already includes per-carrier quality and fading scaling
    // from ce_error_margin. clipLLR() caps final LLR at ±MAX_LLR.
    float cos_diff = std::cos(phase_diff);
    float confidence = 2.0f * signal_power / diff_noise_var;
    float llr = confidence * cos_diff;

    return clipLLR(llr);
}

// DQPSK soft demapping - compares current symbol to previous symbol
// Returns 2 LLRs based on phase difference
// TX encoding: 00→0°, 01→90°, 10→180°, 11→270°
inline std::array<float, 2> demapDQPSK(Complex sym, Complex prev_sym, float noise_var) {
    std::array<float, 2> llrs = {0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);
    float I = diff.real();
    float Q = diff.imag();
    float diff_mag = std::abs(diff);

    if (diff_mag < 1e-6f) {
        return llrs;  // Neutral LLRs for weak signal
    }

    // Differential detection doubles noise: diff = sym*conj(prev) combines noise from both symbols
    float diff_noise_var = 2.0f * noise_var;

    // LLR scaling: 2 * sqrt(SNR). No fixed cap — noise_var already includes
    // per-carrier quality and fading scaling. clipLLR() caps final LLR at ±MAX_LLR.
    float signal_power = std::abs(sym) * std::abs(prev_sym);
    float snr_linear = signal_power / diff_noise_var;
    float scale = 2.0f * std::sqrt(snr_linear);

    // Max-log-MAP demapping for DQPSK constellation at (1,0),(0,1),(-1,0),(0,-1):
    //
    // bit0 (MSB): sin(phase + π/4) = (I+Q)/(√2 * |diff|)
    //   Positive → near 0° or 90° (bits 00,01), negative → near 180° or 270° (bits 10,11)
    static const float pi = 3.14159265358979f;
    float phase = std::atan2(Q, I);
    llrs[0] = clipLLR(scale * std::sin(phase + pi/4));

    // bit1 (LSB): max-log-MAP uses (|I|-|Q|)/|diff| instead of cos(2*phase)
    //   cos(2*phase) = (I²-Q²)/|diff|² overweights decisions near constellation points
    //   (|I|-|Q|)/|diff| gives properly calibrated LLRs near 45° decision boundary
    //   Positive → near 0° or 180° (bits 00,10), negative → near 90° or 270° (bits 01,11)
    llrs[1] = clipLLR(scale * (std::abs(I) - std::abs(Q)) / diff_mag);

    return llrs;
}

// D8PSK soft demapping with Gray code - max-log-MAP approach
// Compares differential detection output to 8 Gray-coded constellation points
// Returns 3 LLRs (MSB first) with proper bit-to-phase mapping
//
// Gray code mapping (phase_index → data_bits = index ^ (index >> 1)):
//   idx 0 (22.5°)→000  idx 1 (67.5°)→001  idx 2 (112.5°)→011  idx 3 (157.5°)→010
//   idx 4 (202.5°)→110  idx 5 (247.5°)→111  idx 6 (292.5°)→101  idx 7 (337.5°)→100
// Adjacent phases differ by exactly 1 bit → single phase error flips only 1 bit
inline std::array<float, 3> demapD8PSK(Complex sym, Complex prev_sym, float noise_var) {
    std::array<float, 3> llrs = {0.0f, 0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return llrs;  // Neutral LLRs for weak signal
    }

    // Differential detection doubles noise
    float diff_noise_var = 2.0f * noise_var;

    // Scale: converts squared Euclidean distance difference to LLR
    // Factor 1/(2σ²_diff) where σ²_diff is per-real-dimension noise of diff output
    float scale = 1.0f / (2.0f * diff_noise_var);

    // Gray code: phase_index → data_bits
    static constexpr int gray_bits[8] = {0, 1, 3, 2, 6, 7, 5, 4};

    // Compute squared distance to each constellation point
    // Constellation at amplitude = signal_power (since diff ≈ |sym|*|prev| * e^{jΔφ})
    static const float pi = 3.14159265358979f;
    float dist_sq[8];
    for (int i = 0; i < 8; i++) {
        float angle = i * (pi / 4.0f) + pi / 8.0f;  // 22.5° offset
        float ref_r = signal_power * std::cos(angle);
        float ref_i = signal_power * std::sin(angle);
        float dr = diff.real() - ref_r;
        float di = diff.imag() - ref_i;
        dist_sq[i] = dr * dr + di * di;
    }

    // Max-log-MAP: for each bit, find closest point with bit=0 vs bit=1
    for (int bit = 0; bit < 3; bit++) {
        int mask = 1 << (2 - bit);  // bit 0 → MSB (mask=4), bit 2 → LSB (mask=1)
        float min_d0 = 1e30f;
        float min_d1 = 1e30f;

        for (int i = 0; i < 8; i++) {
            if (gray_bits[i] & mask) {
                if (dist_sq[i] < min_d1) min_d1 = dist_sq[i];
            } else {
                if (dist_sq[i] < min_d0) min_d0 = dist_sq[i];
            }
        }

        llrs[bit] = clipLLR(scale * (min_d1 - min_d0));
    }

    return llrs;
}

// =============================================================================
// TWO-PASS DQPSK HELPERS
// =============================================================================

// Extract nearest DQPSK phase (0°, 90°, 180°, 270°) from differential symbol
// Returns the ideal phase that the hard decision would produce
inline float extractDQPSKIdealPhase(float actual_phase) {
    // Quantize to nearest 90° (DQPSK grid: 0°, 90°, 180°, 270°)
    static const float pi = 3.14159265358979f;
    static const float half_pi = pi / 2.0f;
    int quadrant = static_cast<int>(std::round(actual_phase / half_pi));
    // Wrap to [-2, 1] → maps to -180°, -90°, 0°, 90°
    quadrant = ((quadrant + 2) % 4) - 2;  // Results in -2, -1, 0, 1
    return quadrant * half_pi;
}

// Compute phase error: difference between actual differential phase and ideal DQPSK phase
// Positive error means actual phase is ahead of ideal
inline float computeDQPSKPhaseError(Complex sym, Complex prev_sym) {
    Complex diff = sym * std::conj(prev_sym);
    float actual_phase = std::atan2(diff.imag(), diff.real());
    float ideal_phase = extractDQPSKIdealPhase(actual_phase);
    float error = actual_phase - ideal_phase;
    // Wrap to [-π, π]
    static const float pi = 3.14159265358979f;
    if (error > pi) error -= 2 * pi;
    if (error < -pi) error += 2 * pi;
    return error;
}

// DQPSK soft demapping with phase correction applied
// Same as demapDQPSK but applies a phase correction before computing LLRs
inline std::array<float, 2> demapDQPSKCorrected(Complex sym, Complex prev_sym,
                                                  float noise_var, float phase_correction) {
    // Apply phase correction to the symbol before differential detection
    Complex correction(std::cos(phase_correction), std::sin(phase_correction));
    Complex corrected_sym = sym * correction;
    return demapDQPSK(corrected_sym, prev_sym, noise_var);
}

// =============================================================================
// CHANNEL ESTIMATION ERROR MARGIN HELPER
// =============================================================================

inline float getCEErrorMargin(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK:
        case Modulation::DQPSK:
            return CE_MARGIN_BPSK_QPSK;
        case Modulation::BPSK:
        case Modulation::QPSK:
            return CE_MARGIN_BPSK_QPSK;
        case Modulation::D8PSK:
            return CE_MARGIN_8PSK;
        case Modulation::QAM8:
            return CE_MARGIN_8PSK;
        case Modulation::QAM16:
            return CE_MARGIN_QAM16;
        case Modulation::QAM32:
            return CE_MARGIN_QAM32;
        case Modulation::QAM64:
            return CE_MARGIN_QAM64;
        case Modulation::QAM256:
            return CE_MARGIN_QAM256;
        default:
            return CE_MARGIN_BPSK_QPSK;
    }
}

} // namespace soft_demap
} // namespace ultra
