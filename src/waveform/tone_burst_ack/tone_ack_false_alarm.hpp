#pragma once
// Tone-ACK candidate floor from a false-ACK budget (2026-11-22).
//
// A false ACK retires frames the receiver never got. Noise reaches the sender's ARQ only through
// three gates in series: the detector's candidate floor F (Costas statistic over its noise-only
// expectation), the CRC-12, and the protocol predicate (Connection::isToneBurstAckCandidatePlausible:
// type ACK, group_seq inside [base-1, base-1+span], frame_mask only over frames still sent). The
// floor was one number, 2.2, sized for the worst sender state. The predicate's selectivity depends
// on the span in flight by orders of magnitude, so the floor follows it: the lowest F whose
// accepted-false-ACK rate stays inside the budget.
//
// H0 model, MEASURED (tools/ack_noise_h0, 2.3 M noise windows; all five staircase rungs; carrier
// search 0 and +-4.8 Hz, the connected monitor's widths; diversity band plan): CRC-passing noise
// decodes per ARMED HOUR, every rung scanned, at floor F:
//     F    1.5   1.6   1.7   1.8   1.9      (+-4.8 Hz;  search 0: 14, 4.3, 0.99, 0.20, 0.023)
//     /h   19    5.5   1.2   0.26  0.081
// modelled R(F) = R0 exp(-k (F - F0)), F0 = 1.5, R0 = 19 /h, k = 12 = the SHALLOWEST local slope
// measured (1.5 -> 1.6; the tail steepens above), so every floor it returns is at least as high
// as the data requires. Below F0 there is no data: the floor never goes under it.
// CRC-passing noise payloads are uniform (4242 measured: type, epoch, group_seq flat; frame_mask
// popcount binomial(16, 1/2)), so the predicate's pass probability follows from field widths.
// The move-epoch echo is not counted (it is off unless enabled), nor the 2.2 cap's own margin.
//
// Budget: 1e-5 accepted false ACKs per armed hour (one per ~11 years of continuous armed
// listening). It returns 2.36 for a full 16-frame window, i.e. the old 2.2 floor (the cap) is
// this rule at the widest span; 8 in flight 1.90, 5 1.72, 3 1.60, 1-2 1.5.
// Outside the measured conditions the floor stays at kCostasNoiseRatioFloor: a wider carrier
// search (the +-50 Hz acquisition case: more hypotheses, higher noise maxima) or the narrowband
// band plan (no diversity: a wider noise statistic).
#include "tone_burst_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

inline constexpr double kToneAckFalseAckBudgetPerArmedHour = 1e-5;
inline constexpr float kToneAckH0FloorRef = 1.5f;             // F0: lowest measured floor
inline constexpr double kToneAckH0CrcRatePerHourAtRef = 19.0;  // R0
inline constexpr double kToneAckH0RateSlopePerUnit = 12.0;     // k
// Widest carrier search the model covers: the dual-chirp phantom bound the connected monitor
// uses (toneAckChirpPhantomBoundHz, 4.8 Hz), with rounding headroom.
inline constexpr float kToneAckH0MaxHalfWidthHz = 5.0f;
inline constexpr int kToneAckFrameMaskBits = 16;
inline constexpr int kToneAckGroupSeqBits = 6;

// Probability that a uniformly random CRC-passing payload passes the protocol predicate while
// `span` frames are in flight. ACK voice: type (1/2) x group_seq delta d in [0, span] (1/64
// each) x frame_mask inside the (span - d) frames still sent (2^(rem - 16)). The reserved
// WAITING-REBASE NACK voice passes regardless of span: type, rung_cmd (2 bits), frame_mask 0,
// rate_hint 0 (3 bits), drive hold (2 bits).
inline double toneAckPredicatePassProbability(size_t span) {
    const double seq_cells = std::ldexp(1.0, kToneAckGroupSeqBits);
    // group_seq is 6 bits: its delta from base-1 is at most 63, so no span reaches further. Also
    // bounds the loop: Connection publishes SIZE_MAX before its first arm, and an unbounded loop
    // here ran forever on gcc/MSVC (clang deleted it as side-effect free; CI 2026-11-22 hung).
    span = std::min<size_t>(span, (size_t{1} << kToneAckGroupSeqBits) - 1);
    double mask_sum = 0.0;
    for (size_t d = 0; d <= span; ++d) {
        const int rem = static_cast<int>(std::min<size_t>(span - d, kToneAckFrameMaskBits));
        mask_sum += std::ldexp(1.0, rem - kToneAckFrameMaskBits);
    }
    const double ack_voice = 0.5 * mask_sum / seq_cells;
    const double rebase_voice = 0.5 * 0.25 * 0.125 * 0.25 * std::ldexp(1.0, -kToneAckFrameMaskBits);
    return ack_voice + rebase_voice;
}

// The candidate floor for `span` frames in flight at this carrier search and band plan.
inline float toneAckNoiseRatioFloorFor(size_t span, float halfwidth_hz, bool diversity) {
    if (!diversity || halfwidth_hz > kToneAckH0MaxHalfWidthHz) return kCostasNoiseRatioFloor;
    const double excess = kToneAckH0CrcRatePerHourAtRef * toneAckPredicatePassProbability(span) /
                          kToneAckFalseAckBudgetPerArmedHour;
    const double f = kToneAckH0FloorRef + std::log(std::max(excess, 1e-300)) / kToneAckH0RateSlopePerUnit;
    return static_cast<float>(std::clamp(f, static_cast<double>(kToneAckH0FloorRef),
                                         static_cast<double>(kCostasNoiseRatioFloor)));
}

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
