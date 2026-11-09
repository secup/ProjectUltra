#pragma once

// Frame-level turbo code for OFDM data frames: ONE codeword per frame.
//
// One long PCCC codeword per data frame instead of several short LDPC codewords.
// Physics: a frame of 8-16 independent 648-bit LDPC codewords fails if ANY codeword
// fails, and each short code sits ~2 dB from capacity. One codeword spanning the whole
// frame (K = 2.6-7.7 kbit) removes the N-fold multiplier and gains ~1-1.5 dB of coding.
//
// Code: PCCC of two 8-state recursive systematic convolutional encoders, feedback
// 1+D^2+D^3 (13 octal), parity 1+D+D^3 (15 octal), each terminated with 3 tail steps
// (12 tail bits, never punctured). Internal interleaver: deterministic S-random.
// Exact rate matching to the frame's air-bit count N (Bresenham parity selection; low
// rates repeat systematic bits) folded with a frame-wide random channel permutation
// into one air-position -> decoder-slot table. Info block = whitened info bytes +
// CRC-32C; the CRC drives early stopping.
//
// Decoder defaults (16 iterations, scale 0.75) are justified at DecodeOptions.
//
// Determinism: every table comes from integer-only SplitMix64 (never std::shuffle /
// std::uniform_int_distribution, whose output differs between libc++ and libstdc++ —
// a Mac and a Pi would build different codes and decode 0%). Digests are pinned in
// tests/test_turbo_codec.cpp.
//
// LLR convention: positive = bit 0 (INV-LDPC-001). Decoder: scaled max-log-MAP
// (extrinsic scale 0.75) — invariant to a uniform LLR scale, so only the demapper clip
// (MAX_LLR) matters, and only if LLRs are mis-calibrated.

#include "ultra/types.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace ultra {
namespace fec {
namespace turbo {

// Decoder slot layout: [0,K) systematic, [K,2K) parity-1, [2K,3K) parity-2,
// [3K, 3K+12) tails in the order x1t0 z1t0 x1t1 z1t1 x1t2 z1t2 x2t0 z2t0 ... z2t2.
constexpr int kTailSlots = 12;
constexpr size_t kCrcBytes = 4;

struct Tables {
    int K = 0;       // info bits including the 32-bit CRC
    int N = 0;       // air bits
    int spread = 0;  // achieved S-random spread S
    int E1 = 0;      // parity-1 bits kept
    int E2 = 0;      // parity-2 bits kept
    int R = 0;       // repeated bits (low rates)
    std::vector<uint32_t> pi;        // encoder-2 input at step t is info bit pi[t]
    std::vector<uint32_t> air_slot;  // air position -> decoder slot
    uint64_t digest = 0;             // FNV-1a over pi and air_slot
};

// Thread-safe cached tables for (K, N). Throws std::invalid_argument when N < K + 12,
// N is not a multiple of 8, K is not a multiple of 8, or the rate needs more
// repetition than 3K bits can source.
const Tables& tables(int K, int N);

// K for a given info-byte count: 8 * (info_bytes + 4).
inline int infoBitsForBytes(size_t info_bytes) {
    return static_cast<int>(8 * (info_bytes + kCrcBytes));
}

// Encode `info_bytes` (whitened and CRC-32C protected internally) into `air_bits`
// coded bits, packed MSB-first (air_bits / 8 bytes, air order).
Bytes encode(const Bytes& info_bytes, int air_bits);

enum class Stop : uint8_t { Crc, MaxIterations, Stuck, NoEnergy };

struct DecodeOptions {
    // 16, not 8: near the waterfall a turbo decoder is still converging at 8 iterations.
    // Measured (BPSK/AWGN, 300 frames): R1/2 K=2592 @ Eb/N0 1.0 dB FER 25.0 % -> 6.3 %,
    // R2/3 K=3488 @ 1.9 dB 23.7 % -> 7.7 %; saturates by 16 (24/32 within noise).
    // CRC early stopping keeps clean frames at 1-3 iterations, so only frames that
    // are genuinely hard pay for the extra iterations.
    int max_iterations = 16;
    // Max-log-MAP over-estimates extrinsic reliability; the standard correction is a
    // scale in 0.7-0.75 (Vogt & Finger). 0.75 measured better at 16 iterations
    // (R2/3 @1.9 dB 7.7 -> 5.7 %, R3/4 @2.6 dB 3.0 -> 1.7 %); same role as the
    // LDPC normalized min-sum factor 0.75.
    float extrinsic_scale = 0.75f;
    bool early_fail = true;        // stop after 2 iterations with no decision flips
};

struct DecodeResult {
    bool crc_ok = false;
    int iterations = 0;
    Stop stop = Stop::MaxIterations;
    Bytes info_bytes;            // de-whitened hard decisions, always info_bytes long
    float mean_abs_llr = 0.0f;   // over the air LLRs
    float clip_fraction = 0.0f;  // fraction of air LLRs at or above the clip level
    // Constituent-decoder sign agreement of the two extrinsic streams on the final
    // iteration (fraction of info bits). Evidence that the window holds a REAL codeword
    // even when the CRC fails (a fade) versus noise / a misaligned window (~0.5).
    float extrinsic_agreement = 0.5f;
};

DecodeResult decode(std::span<const float> air_llrs, size_t info_bytes,
                    const DecodeOptions& opts = {}, float clip_level = 20.0f);

// Exposed for tests / tools.
uint64_t splitmix64Next(uint64_t& state);
std::vector<uint8_t> whitening(size_t nbytes);
// Encoder-only helper: the K info bits (after whitening + CRC) for a payload.
std::vector<uint8_t> infoBits(const Bytes& info_bytes);

}  // namespace turbo
}  // namespace fec
}  // namespace ultra
