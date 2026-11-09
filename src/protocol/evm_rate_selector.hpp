#pragma once
// ============================================================================
// EFFECTIVE-SNR RATE SELECTION  (2026-11-08; ULTRA_RATE_EVM=climb is the default via ULTRA_FAST
// since 2026-11-09, =1 FULL is an experiment, =0 off)
// ============================================================================
//
// WHAT WAS MEASURED. Every burst data frame carries the demodulator's decision-directed,
// gain-corrected EVM SNR (demodulator_impl.hpp "Radio-agnostic decision-directed EVM SNR"):
// the scatter of the equalized constellation around its own decisions, pooled over the
// frame's data cells. Pairing it with the frame's decode verdict over 7131 GUI-gate frames
// (OTASim, ITU Good / Moderate / AWGN, 7.3 Hz dial offset) shows that a frame decodes iff its
// EVM clears a per-rung threshold, and that the threshold DOES NOT DEPEND ON THE CHANNEL:
//
//     rung        Good     Moderate   (EVM SNR at 50 % frame success; 10-90 % in ~1-1.5 dB)
//     QPSK R2/3   6.5 dB   6.3 dB
//     8PSK R2/3   9.55     9.45
//     QPSK R1/2   5.85     (no failures above 4 dB)
//
// That is the effective-SNR link abstraction: fading, Doppler-induced estimation error and
// implementation loss are all already inside the EVM, so what remains is the code's own
// waterfall: per-frame measured SNR against per-level thresholds plus an outer-loop margin
// (standard outer-loop link adaptation), with no channel classifier.
//
// WHY IT REPLACES THE OUTCOME-ONLY DECISION. The latent controller sees only k-of-M per group
// on the rung in use. A rung that always decodes saturates its likelihood, so the controller
// cannot see how much margin it has (it needed a startup probe to leave QPSK R1/2, never
// climbed to 16QAM R2/3 on AWGN@20 where every frame would decode, and after one fade crater
// on Good@20 it sat on QPSK R2/3 for the rest of the transfer). And a channel-SNR model
// predicts a lower rung far more robust than it is inside a fade, because deep fades kill
// every rung (the 2026-11-08 renewal-pricing experiment, reverted). The EVM sees both: a frame
// in a deep fade has a low EVM and is predicted to fail on every rung.
//
// THE SELECTOR. For every rung, the counterfactual success over the last kWindowFrames frames:
//     P_r = mean_f logistic(s * (e_f - eta_r - m))
// i.e. the fraction of what was just received that rung r WOULD have decoded. The window
// (~4 groups ~ 35 s) spans several coherence times on Good (Tc 4.2 s) while tracking
// propagation change over half a minute. The value is the same delivered-bytes-per-cycle
// pricing, dead/cap clamps and operator ceiling as the latent controller; only the success
// probability changes.
//
// OUTER LOOP (calibration, not fade tracking). m is a common-mode margin on every threshold,
// moved by the incumbent's residual between delivered frames and the success its own frames'
// EVMs predicted. A fade does not move it — the faded frames' EVMs predicted the losses — only
// a mismatch between this radio's EVM and the measured thresholds does (non-Gaussian
// impairments, an estimator scale shift). On the simulator it stays near zero.
//
// SCOPE. Pure logic, no I/O, no clock, no RNG. Decides only when it holds kMinFrames measured
// frames; otherwise the caller keeps the latent decision.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "frame_v2.hpp"
#include "ultra/fast_profile.hpp"
#include "latent_rate_controller.hpp"
#include "waveform_selection.hpp"

namespace ultra {
namespace protocol {

inline std::atomic<int>& evmRateSelectionFlag() {
    static std::atomic<int> flag{-1};
    return flag;
}
// ULTRA_RATE_EVM: read once, with a test override. Wire-invisible: it runs on the receiver
// (rate authority) only, but set it on both stations so either can lead.
//   =1      FULL: the effective-SNR selector decides every rung.
//   =climb  CLIMB: the latent controller decides; the selector may only lift its pick to a
//           higher rung whose predicted success is at least LatentRateController::kCapProb
//           (the most the controller ever credits any rung) — an upward move that costs no
//           reliability. Demotions stay with the outcome-fitted posterior.
enum class EvmRateMode : int { Off = 0, Full = 1, Climb = 2 };
inline EvmRateMode evmRateSelectionMode() {
    int v = evmRateSelectionFlag().load(std::memory_order_relaxed);
    if (v < 0) {
        // Explicit ULTRA_RATE_EVM wins; unset, the ULTRA_FAST profile selects CLIMB.
        const char* e = std::getenv("ULTRA_RATE_EVM");
        v = 0;
        if (e != nullptr) {
            if (std::strcmp(e, "1") == 0) v = 1;
            if (std::strcmp(e, "climb") == 0) v = 2;
        } else if (fastProfileEnabled()) {
            v = 2;
        }
        evmRateSelectionFlag().store(v, std::memory_order_relaxed);
    }
    return static_cast<EvmRateMode>(v);
}
inline bool evmRateSelectionEnabled() { return evmRateSelectionMode() != EvmRateMode::Off; }
inline void setEvmRateSelectionModeForTesting(EvmRateMode mode) {
    evmRateSelectionFlag().store(static_cast<int>(mode), std::memory_order_relaxed);
}
inline void setEvmRateSelectionForTesting(bool on) {
    setEvmRateSelectionModeForTesting(on ? EvmRateMode::Full : EvmRateMode::Off);
}

// eta_r: the EVM SNR (dB) at which a frame of rung r decodes with probability 1/2. Measured per
// frame: tools/measure_ack_fer --frame-log (production geometry, 7.3 Hz dial offset, ITU Good,
// 3 SNR points x 3 seeds x 24 frames around each waterfall) and the GUI gate's own frames for
// the rungs the controller visits (docs/EVM_RATE_SELECTION_2026_11_08.md):
//
//     rung        harness Good   GUI Good / Moderate
//     QPSK R1/4      4.8            -
//     QPSK R1/2      5.5            5.85 / -
//     QPSK R2/3      6.5            6.5 / 6.3
//     QPSK R3/4      7.1            -
//     8PSK R2/3      9.6            9.55 / 9.45
//     8PSK R3/4     10.5            -
//     16QAM R1/2    11.4            -
//     16QAM R2/3    12.55          12.1 (10-12 dB: 5 %, 12-14 dB: 96 %)
//     16QAM R3/4    13.9            -
//
// CROSS-RUNG USE. The EVM a frame reads also depends on the rung it was sent on: each rung's
// pilot/codeword geometry sets its own channel-estimation ceiling (AWGN fits: QPSK ~21.7 dB,
// 16QAM ~24.7 dB, QPSK R1/4 ~17.7 dB), so on a strong channel 16QAM frames read up to ~1.3 dB
// higher than QPSK frames. Below ~15 dB thermal noise dominates and the rungs agree to within
// 0.1-0.3 dB — and every threshold above sits there — so a frame measured on one rung predicts
// another's waterfall without a per-rung correction where the decision is made.
inline float evmDecodeThresholdLdpcDb(uint8_t rung) {
    switch (rung) {
        case kRungIdxQpskR14: return 4.8f;
        case kRungIdxQpskR12: return 5.6f;
        case kRungIdxQpskR23: return 6.5f;
        case kRungIdxQpskR34: return 7.1f;
        case kRungIdxQam8R23: return 9.6f;
        case kRungIdxQam8R34: return 10.5f;
        case kRungIdxQam16R12: return 11.4f;
        case kRungIdxQam16R23: return 12.3f;   // harness 12.55, GUI 12.1
        case kRungIdxQam16R34: return 13.9f;
        default: return 99.0f;
    }
}
// TURBO data frames (ULTRA_DATA_FEC=turbo): the same waterfalls shifted by the code's measured
// gain. GUI gate frames give 8PSK R2/3 -0.44 dB and 16QAM R2/3 -0.52 dB in the EVM domain,
// matching the 2026-11-08 AWGN FER-10 % gains (8PSK R2/3 0.69, 16QAM R2/3 0.55, QPSK R1/2 0.88,
// R2/3 0.45, R1/4 > 1 dB); the unmeasured R3/4 rungs take their modulation's R2/3 shift. (The
// classic-path harness emits no result for a failed turbo frame, so it cannot fit these.)
inline float evmDecodeThresholdTurboDb(uint8_t rung) {
    switch (rung) {
        case kRungIdxQpskR14: return 3.8f;
        case kRungIdxQpskR12: return 4.7f;
        case kRungIdxQpskR23: return 6.05f;
        case kRungIdxQpskR34: return 6.65f;
        case kRungIdxQam8R23: return 9.1f;
        case kRungIdxQam8R34: return 10.0f;
        case kRungIdxQam16R12: return 10.85f;
        case kRungIdxQam16R23: return 11.75f;
        case kRungIdxQam16R34: return 13.4f;
        default: return 99.0f;
    }
}
inline float evmDecodeThresholdDb(uint8_t rung) {
    return v2::ofdmDataFrameFec() == v2::DataFrameFec::TURBO ? evmDecodeThresholdTurboDb(rung)
                                                             : evmDecodeThresholdLdpcDb(rung);
}
// Logistic slope around eta: the measured 10-90 % widths are 1-1.5 dB (fits 3-4 /dB); 3 /dB
// keeps a little width for the estimator's own per-frame noise.
inline constexpr float kEvmDecodeSlopePerDb = 3.0f;

class EvmRateSelector {
public:
    static constexpr size_t kWindowFrames = 20;
    static constexpr size_t kMinFrames = 4;
    // Outer-loop step: dB of margin per unit of per-frame residual (predicted - delivered)/M.
    // A group whose EVMs predicted 5/5 that delivers 0/5 moves the margin by this much; a
    // perfectly calibrated link moves it by zero on average.
    static constexpr float kMarginStepDb = 2.0f;
    static constexpr float kMarginMinDb = -3.0f;
    static constexpr float kMarginMaxDb = 6.0f;

    EvmRateSelector() { reset(); }

    void reset() {
        ring_.fill(0.0f);
        next_ = 0;
        count_ = 0;
        margin_db_ = 0.0f;
        groups_ = 0;
    }

    static float frameSuccess(float evm_db, uint8_t rung, float margin_db) {
        const float z = kEvmDecodeSlopePerDb * (evm_db - evmDecodeThresholdDb(rung) - margin_db);
        return 1.0f / (1.0f + std::exp(-std::clamp(z, -40.0f, 40.0f)));
    }

    // One received group: its rung, each data frame's EVM (non-finite = not measured), and how
    // many of its `frames` decoded. Unmeasured frames are skipped, not counted as fades.
    void observeGroup(uint8_t rung, const std::vector<float>& evm_db, int delivered, int frames) {
        int measured = 0;
        float predicted = 0.0f;
        for (float e : evm_db) {
            if (!std::isfinite(e)) continue;
            ++measured;
            if (rung < kRungIdxCount) predicted += frameSuccess(e, rung, margin_db_);
            ring_[next_] = e;
            next_ = (next_ + 1) % kWindowFrames;
            if (count_ < kWindowFrames) ++count_;
        }
        // Calibrate only on groups whose every frame was measured, so delivered and predicted
        // count the same frames.
        if (rung < kRungIdxCount && frames > 0 && measured == frames) {
            const float residual =
                (predicted - static_cast<float>(std::clamp(delivered, 0, frames))) /
                static_cast<float>(frames);
            margin_db_ = std::clamp(margin_db_ + kMarginStepDb * residual,
                                    kMarginMinDb, kMarginMaxDb);
        }
        ++groups_;
    }

    bool ready() const { return count_ >= kMinFrames; }
    size_t frames() const { return count_; }
    float marginDb() const { return margin_db_; }

    // Counterfactual success of rung r over the window.
    float successProb(uint8_t rung) const {
        if (count_ == 0) return 0.0f;
        float s = 0.0f;
        for (size_t i = 0; i < count_; ++i) s += frameSuccess(ring_[i], rung, margin_db_);
        return s / static_cast<float>(count_);
    }

    // CLIMB mode: the best-value rung among those predicted to decode at least `min_success`
    // of the recent frames (none if no rung qualifies).
    uint8_t bestReliable(const LatentRateController::RungGeometryTable& geometry,
                         uint8_t ceiling, float min_success) const {
        uint8_t pick = kRungIdxNone;
        float best_g = 0.0f;
        for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
            const CoherentPick cp = coherentRungFromIndex(r);
            if (!coherentRungLocallyEnabled(cp.mod, cp.rate)) continue;
            if (ceiling != kRungIdxNone && r > ceiling) continue;
            const auto& physical = geometry[r];
            if (!physical.valid()) continue;
            const float p = successProb(r);
            if (p < min_success) continue;
            const float g = physical.useful_bytes_per_frame *
                            static_cast<float>(physical.frames_per_cycle) *
                            std::min(p, LatentRateController::kCapProb) / physical.cycle_s;
            if (g > best_g) {
                best_g = g;
                pick = r;
            }
        }
        return pick;
    }

    // EXPECTED TIME TO DELIVER n FRAMES at success p, `per_round` frames per round, each round
    // lasting round_s(m) for its m frames. Exact for independent frame outcomes:
    //     T(n) = [ round_s(m) + sum_{k=1..m} Binom(k; m, p) T(n-k) ] / (1 - (1-p)^m),
    //     m = min(n, per_round).
    // The tail is in it: the last frames of a transfer go in short rounds that still pay the
    // round's fixed overhead, and every frame lost near the end costs another round.
    static float expectedTimeToDeliver(int n_frames, int per_round, float p, float full_round_s,
                                       float frame_airtime_s) {
        if (n_frames <= 0 || per_round <= 0 || !(full_round_s > 0.0f)) return 0.0f;
        const float fixed_s =
            std::max(0.0f, full_round_s - static_cast<float>(per_round) * frame_airtime_s);
        auto round_s = [&](int m) {
            return frame_airtime_s > 0.0f ? fixed_s + static_cast<float>(m) * frame_airtime_s
                                          : full_round_s;
        };
        const double q = 1.0 - static_cast<double>(std::clamp(p, 1.0e-3f, 1.0f));
        std::vector<double> t(static_cast<size_t>(n_frames) + 1, 0.0);
        for (int n = 1; n <= n_frames; ++n) {
            const int m = std::min(n, per_round);
            double acc = round_s(m);
            double b = std::pow(q, m);          // Binom(0; m, p)
            const double b0 = b;
            for (int k = 1; k <= m; ++k) {
                // Binom(k) = Binom(k-1) * (m-k+1)/k * p/q, written to stay finite as q -> 0.
                b = std::exp(std::lgamma(m + 1.0) - std::lgamma(k + 1.0) -
                             std::lgamma(m - k + 1.0) + k * std::log(1.0 - q) +
                             (m - k) * std::log(std::max(q, 1e-300)));
                acc += b * t[static_cast<size_t>(n - k)];
            }
            t[static_cast<size_t>(n)] = acc / std::max(1.0 - b0, 1e-12);
        }
        return static_cast<float>(t[static_cast<size_t>(n_frames)]);
    }

    // remaining_bytes > 0 (the receiver knows what is left of the transfer): choose the rung
    // that delivers it soonest. Otherwise the long-run delivered rate, with Minstrel's cap.
    LatentRateController::Pick best(const LatentRateController::RungGeometryTable& geometry,
                                    uint8_t ceiling = kRungIdxNone,
                                    float remaining_bytes = 0.0f) const {
        LatentRateController::Pick out;
        uint8_t robust = kRungIdxNone;
        float robust_p = -1.0f, robust_g = 0.0f;
        float best_time = 0.0f;
        for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
            const CoherentPick cp = coherentRungFromIndex(r);
            if (!coherentRungLocallyEnabled(cp.mod, cp.rate)) continue;
            if (ceiling != kRungIdxNone && r > ceiling) continue;
            const auto& physical = geometry[r];
            if (!physical.valid()) continue;
            float p = successProb(r);
            const float bytes = physical.useful_bytes_per_frame *
                                static_cast<float>(physical.frames_per_cycle);
            if (p > robust_p) {
                robust_p = p;
                robust = r;
                robust_g = bytes * p / physical.cycle_s;
            }
            if (p < LatentRateController::kDeadProb) continue;
            if (remaining_bytes > 0.0f) {
                const int frames_needed = static_cast<int>(
                    std::ceil(remaining_bytes / physical.useful_bytes_per_frame));
                const float t = expectedTimeToDeliver(
                    std::min(frames_needed, 4096), physical.frames_per_cycle, p,
                    physical.cycle_s, physical.frame_airtime_s);
                if (t > 0.0f && (out.rung == kRungIdxNone || t < best_time)) {
                    best_time = t;
                    out.rung = r;
                    out.goodput = remaining_bytes / t;
                }
                continue;
            }
            p = std::min(p, LatentRateController::kCapProb);
            const float g = bytes * p / physical.cycle_s;
            if (g > out.goodput) {
                out.goodput = g;
                out.rung = r;
            }
        }
        if (out.rung == kRungIdxNone && robust != kRungIdxNone) {
            out.rung = robust;
            out.goodput = robust_g;
        }
        out.x_used = margin_db_;
        return out;
    }

private:
    std::array<float, kWindowFrames> ring_{};
    size_t next_ = 0;
    size_t count_ = 0;
    float margin_db_ = 0.0f;
    int groups_ = 0;
};

}  // namespace protocol
}  // namespace ultra
