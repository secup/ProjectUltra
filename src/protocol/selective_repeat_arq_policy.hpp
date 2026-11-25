#pragma once

#include <cstdlib>
#include "frame_v2.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace ultra {
namespace protocol {
namespace selective_repeat_arq_policy {

inline constexpr size_t kMaxWindow = 16;
inline constexpr int kMaxFastRetransmitsPerHole = 1;
inline constexpr int kMinHoleConfirmations = 2;
inline constexpr int kMaxHoleProbeRetransmits = 1;
inline constexpr uint32_t kAckRepeatMaxJitterMs = 30;

inline size_t clampWindowSize(size_t size, size_t max_window = kMaxWindow) {
    return std::clamp<size_t>(size, 1, max_window);
}

inline uint32_t clampAckBatchSize(uint32_t ack_batch_size,
                                  size_t window_size,
                                  size_t max_window = kMaxWindow) {
    ack_batch_size = std::min<uint32_t>(ack_batch_size, static_cast<uint32_t>(max_window));
    if (ack_batch_size > 0 && ack_batch_size > static_cast<uint32_t>(window_size)) {
        return static_cast<uint32_t>(window_size);
    }
    return ack_batch_size;
}

inline uint32_t effectiveAckBatchThreshold(uint32_t ack_batch_size, size_t window_size) {
    return ack_batch_size > 0 ? ack_batch_size : static_cast<uint32_t>(window_size);
}

inline uint32_t decodeSackBitmap(const uint8_t* payload) {
    if (!payload) {
        return 0;
    }

    const uint32_t bitmap = v2::NackPayload::decode(payload).cw_bitmap;

    // Compatibility with the legacy 8-bit SACK encoding used before the
    // wider-window path: makeNack() placed the bitmap in payload[5], then the
    // ARQ code also copied it into payload[2] and read only payload[2].
    const uint8_t legacy = payload[2];
    const uint32_t legacy_shape = (static_cast<uint32_t>(legacy) << 24) | legacy;
    if (legacy != 0 && bitmap == legacy_shape) {
        return legacy;
    }

    return bitmap;
}

inline bool seqInWindow(uint16_t seq, uint16_t base_seq, size_t window_size) {
    const uint16_t diff = (seq - base_seq) & 0xFFFF;
    return diff < window_size;
}

enum class AckFreshness {
    Accept,
    Stale,
    Future,
};

inline AckFreshness classifyAckFreshness(uint16_t ack_seq,
                                         uint16_t tx_base_seq,
                                         size_t window_size) {
    const uint16_t ack_base = (tx_base_seq - 1) & 0xFFFF;
    const uint16_t back = (ack_base - ack_seq) & 0xFFFF;
    if (back > 0 && back < 0x8000) {
        return AckFreshness::Stale;
    }

    const uint16_t forward = (ack_seq - ack_base) & 0xFFFF;
    if (forward > window_size + 1 && forward < 0x8000) {
        return AckFreshness::Future;
    }

    return AckFreshness::Accept;
}

inline bool shouldSuppressDuplicateAck(bool last_signature_valid,
                                       uint32_t dedup_timer_ms,
                                       uint16_t last_seq,
                                       uint32_t last_bitmap,
                                       uint16_t seq,
                                       uint32_t bitmap) {
    return last_signature_valid &&
           dedup_timer_ms > 0 &&
           last_seq == seq &&
           last_bitmap == bitmap;
}

inline uint32_t ackDedupWindowMs(uint32_t ack_repeat_delay_ms) {
    return std::clamp(ack_repeat_delay_ms + 40u, 80u, 500u);
}

inline uint32_t sackDelayForFrame(uint32_t sack_delay_ms,
                                  uint32_t sack_delay_short_ms,
                                  bool use_short_delay) {
    uint32_t pick_ms = sack_delay_ms;
    if (sack_delay_short_ms != 0 && use_short_delay) {
        pick_ms = sack_delay_short_ms;
    }
    return pick_ms;
}

inline uint32_t sackTimerForFrame(uint32_t current_timer_ms,
                                  uint32_t sack_delay_ms,
                                  uint32_t sack_delay_short_ms,
                                  bool use_short_delay) {
    const uint32_t pick_ms =
        sackDelayForFrame(sack_delay_ms, sack_delay_short_ms, use_short_delay);

    if (current_timer_ms == 0) {
        return pick_ms;
    }
    return std::min(current_timer_ms, pick_ms);
}

inline bool isAlignedBaseHoleAck(uint16_t ack_seq,
                                 uint16_t tx_base_seq,
                                 uint32_t bitmap) {
    const bool is_aligned = (ack_seq == ((tx_base_seq - 1) & 0xFFFF));
    const bool has_hole = (bitmap & 0x01u) == 0 && (bitmap & ~0x01u) != 0;
    return is_aligned && has_hole;
}

inline uint32_t holeProbeInitialTimerMs(uint32_t ack_timeout_ms) {
    return std::clamp(ack_timeout_ms / 2, 1200u, 2500u);
}

inline uint32_t holeProbeNextTimerMs(uint32_t ack_timeout_ms) {
    return std::clamp((ack_timeout_ms * 2) / 3, 1800u, 3000u);
}

inline uint32_t fastRetransmitCooldownMs(uint32_t ack_timeout_ms) {
    return std::clamp(ack_timeout_ms / 6, 300u, 1200u);
}

inline bool shouldFastRetransmitHole(int hole_ack_count,
                                     int fast_retx_count,
                                     uint32_t fast_retx_cooldown_ms) {
    return hole_ack_count >= kMinHoleConfirmations &&
           fast_retx_count < kMaxFastRetransmitsPerHole &&
           fast_retx_cooldown_ms == 0;
}

inline bool shouldSendImmediateFrameNackForGap(bool out_of_order,
                                               bool more_frag,
                                               bool final_frame) {
    return out_of_order && final_frame && !more_frag;
}

inline uint32_t ackRepeatDelayForCopy(uint32_t ack_repeat_delay_ms, int copy_index) {
    if (copy_index <= 2) {
        return ack_repeat_delay_ms;
    }
    return ack_repeat_delay_ms * 3;
}

inline uint32_t ackRepeatDelayWithHalfDuplexGuard(uint32_t repeat_delay_ms,
                                                  uint32_t peer_burst_guard_ms,
                                                  bool guard_half_duplex) {
    if (!guard_half_duplex) {
        return repeat_delay_ms;
    }

    const uint64_t guarded_delay =
        static_cast<uint64_t>(peer_burst_guard_ms) + repeat_delay_ms;
    return static_cast<uint32_t>(std::min<uint64_t>(guarded_delay, 0xFFFFFFFFull));
}

inline uint32_t ackRepeatTailGuardMs(uint32_t ack_airtime_ms,
                                     uint32_t sack_delay_ms,
                                     uint32_t ack_repeat_delay_ms,
                                     int ack_repeat_count,
                                     bool guard_half_duplex_repeat) {
    if (ack_repeat_count <= 1) {
        return 0;
    }

    const uint32_t base_delay =
        ackRepeatDelayForCopy(ack_repeat_delay_ms, ack_repeat_count);
    const uint32_t repeat_delay =
        ackRepeatDelayWithHalfDuplexGuard(base_delay, sack_delay_ms,
                                          guard_half_duplex_repeat);
    const uint64_t tail_ms = static_cast<uint64_t>(repeat_delay) +
                             kAckRepeatMaxJitterMs +
                             static_cast<uint64_t>(ack_airtime_ms);
    return static_cast<uint32_t>(std::min<uint64_t>(tail_ms, 0xFFFFFFFFull));
}

inline int ackRepeatJitterMs(uint16_t base_seq, uint32_t bitmap, int copy_index) {
    uint32_t h = static_cast<uint32_t>(base_seq);
    h = (h * 1103515245u + 12345u) ^ (bitmap << 8) ^ (bitmap >> 16)
        ^ static_cast<uint32_t>(copy_index * 7919);
    return static_cast<int>(h % (2u * kAckRepeatMaxJitterMs + 1u)) -
           static_cast<int>(kAckRepeatMaxJitterMs);
}

inline uint32_t rttSampleMs(uint64_t now_ms, uint64_t first_tx_ms) {
    if (now_ms < first_tx_ms) {
        return 0;
    }
    return static_cast<uint32_t>(std::min<uint64_t>(now_ms - first_tx_ms, 60000ULL));
}

inline bool shouldUseRTTSample(bool rtt_sample_eligible,
                               uint64_t now_ms,
                               uint64_t first_tx_ms,
                               uint32_t min_sample_ms = 50) {
    return rtt_sample_eligible &&
           now_ms >= first_tx_ms &&
           rttSampleMs(now_ms, first_tx_ms) >= min_sample_ms;
}

struct RTOUpdate {
    float srtt_ms = 0.0f;
    float rttvar_ms = 0.0f;
    uint32_t rto_ms = 0;
    uint32_t floor_ms = 0;
    uint32_t ceiling_ms = 0;
};

// The configured worst-case timeout floors the RFC6298 estimate (so the RTO sits at it on the
// wideband window). An adaptive floor (ULTRA_ADAPTIVE_RTO) was tried and retracted: measured srtt
// already sat at the configured value, so it moved the RTO 0-20 %, not 7.5x (REMOVAL_BACKLOG R12).

// ULTRA_INFLIGHT_RTO (default OFF) — size the retransmission timeout on the frames
// ACTUALLY OUTSTANDING, not on the window maximum.
//
// `configured_ack_timeout_ms` is computed once per mode configuration as
// computeWideOFDMAckTimeoutMs(mod, rate, arq_.getWindowSize(), ...) — the airtime of a
// FULL window plus SACK hold, ACK-repeat tail and decode jitter. That is the correct
// bound while a full burst is in flight: the sender must wait out the whole burst before
// concluding loss. It is set ONCE and never shrinks.
//
// At the END of a transfer one frame is outstanding — ~1.27 s of airtime — and the sender
// still waits the full-window timeout. Operator observation that prompted this: "would
// have been a lot faster if it didn't hit a bunch of timeout and retx at the end". One
// rig transfer took 57 s to deliver its last ~1.4 KB of a 50 KB file, with the sender
// logging CCA idle=1 through the gap.
//
// This is a DIFFERENT defect from ULTRA_ADAPTIVE_RTO, which only stopped the configured
// value from being used as the estimator's floor. That did not help the tail, and the
// measurement says why: srtt is averaged over ALL round trips, which are mostly full
// bursts, so the estimator never learns that a 1-frame tail is cheap. The timeout has to
// be a FUNCTION of the outstanding count, not a scalar.
//
// The Connection supplies a table computed with the SAME policy function at each frame
// count, so no timing knowledge is duplicated into the ARQ and index [window] reproduces
// the legacy value exactly.
inline bool inflightRtoEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_INFLIGHT_RTO");
        return e != nullptr && e[0] == '1' && e[1] == '\0';
    }();
    return on;
}

// RFC 6298 smoothing (alpha 1/8, beta 1/4) of one latency sample; the first sample seeds
// srtt = L, rttvar = L/2.
struct LatencyEstimate {
    float srtt_ms = 0.0f;
    float rttvar_ms = 0.0f;
};
inline LatencyEstimate smoothLatency(bool have_estimate, float srtt_ms, float rttvar_ms,
                                     float sample_ms) {
    LatencyEstimate out;
    if (!have_estimate) {
        out.srtt_ms = sample_ms;
        out.rttvar_ms = 0.5f * sample_ms;
        return out;
    }
    out.rttvar_ms = 0.75f * rttvar_ms + 0.25f * std::fabs(srtt_ms - sample_ms);
    out.srtt_ms = 0.875f * srtt_ms + 0.125f * sample_ms;
    return out;
}

inline RTOUpdate updateRTO(bool have_estimator,
                           float previous_srtt_ms,
                           float previous_rttvar_ms,
                           uint32_t sample_ms,
                           uint32_t configured_ack_timeout_ms) {
    RTOUpdate update;
    if (!have_estimator) {
        update.srtt_ms = static_cast<float>(sample_ms);
        update.rttvar_ms = static_cast<float>(sample_ms) * 0.5f;
    } else {
        const float sample_f = static_cast<float>(sample_ms);
        const float err = std::fabs(previous_srtt_ms - sample_f);
        update.rttvar_ms = 0.75f * previous_rttvar_ms + 0.25f * err;
        update.srtt_ms = 0.875f * previous_srtt_ms + 0.125f * sample_f;
    }

    const float rto_f = update.srtt_ms + 4.0f * update.rttvar_ms;

    const uint32_t srtt_floor = static_cast<uint32_t>(update.srtt_ms * 1.5f + 0.5f);
    update.floor_ms = std::clamp(srtt_floor, 600u, 2500u);
    // The configured worst case also floors the estimate.
    update.floor_ms = std::max(update.floor_ms, configured_ack_timeout_ms);
    update.ceiling_ms = std::max<uint32_t>(12000u, configured_ack_timeout_ms);
    update.rto_ms = std::clamp(static_cast<uint32_t>(rto_f + 0.5f),
                               update.floor_ms, update.ceiling_ms);
    return update;
}

}  // namespace selective_repeat_arq_policy
}  // namespace protocol
}  // namespace ultra
