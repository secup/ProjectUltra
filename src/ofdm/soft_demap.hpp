#pragma once

// Soft demapping functions for OFDM demodulator
// Computes LLRs (log-likelihood ratios) for various modulation schemes

#include "ultra/types.hpp"
#include "demodulator_constants.hpp"
#include <cmath>
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
    static Point constellation[32];
    static bool initialized = false;

    if (!initialized) {
        for (int qi = 0; qi < 8; ++qi) {
            for (int ii = 0; ii < 4; ++ii) {
                int idx = qi * 4 + ii;
                constellation[idx].pos = Complex(I_LEVELS[ii] * QAM32_SCALE,
                                                  Q_LEVELS[qi] * QAM32_SCALE);
                constellation[idx].bits = (Q_GRAY[qi] << 2) | I_GRAY[ii];
            }
        }
        initialized = true;
    }

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

    // cos(phase_diff) gives distance from boundaries
    float cos_diff = std::cos(phase_diff);
    float llr = 2.0f * signal_power * cos_diff / noise_var;

    return clipLLR(llr);
}

// DQPSK soft demapping - compares current symbol to previous symbol
// Returns 2 LLRs based on phase difference
// TX encoding: 00→0°, 01→90°, 10→180°, 11→270°
inline std::array<float, 2> demapDQPSK(Complex sym, Complex prev_sym, float noise_var) {
    std::array<float, 2> llrs = {0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);
    float phase = std::atan2(diff.imag(), diff.real());

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return llrs;  // Neutral LLRs for weak signal
    }

    float scale = 2.0f * signal_power / noise_var;
    static const float pi = 3.14159265358979f;

    // bit1 (MSB): use sin(phase + π/4) as soft metric
    llrs[0] = clipLLR(scale * std::sin(phase + pi/4));

    // bit0: use cos(2*phase) as soft metric
    llrs[1] = clipLLR(scale * std::cos(2 * phase));

    return llrs;
}

// D8PSK soft demapping - compares current symbol to previous symbol
// Returns 3 LLRs based on phase difference (8 phases at 45° increments)
inline std::array<float, 3> demapD8PSK(Complex sym, Complex prev_sym, float noise_var) {
    std::array<float, 3> llrs = {0.0f, 0.0f, 0.0f};

    Complex diff = sym * std::conj(prev_sym);
    float phase_diff = std::atan2(diff.imag(), diff.real());

    float signal_power = std::abs(sym) * std::abs(prev_sym);
    if (signal_power < 1e-6f) {
        return llrs;  // Neutral LLRs for weak signal
    }

    // D8PSK: 8 phases at 45° increments (natural binary, no Gray code)
    float confidence = signal_power / noise_var;

    // Use sin-based formulas (matches working single-carrier DPSK)
    llrs[0] = clipLLR(confidence * std::sin(phase_diff));
    llrs[1] = clipLLR(confidence * std::sin(2.0f * phase_diff));
    llrs[2] = clipLLR(confidence * std::sin(4.0f * phase_diff));

    return llrs;
}

// =============================================================================
// CHANNEL ESTIMATION ERROR MARGIN HELPER
// =============================================================================

inline float getCEErrorMargin(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK:
        case Modulation::DQPSK:
        case Modulation::BPSK:
        case Modulation::QPSK:
            return CE_MARGIN_BPSK_QPSK;
        case Modulation::D8PSK:
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
