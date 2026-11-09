#pragma once

#include "protocol/frame_v2.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace ultra {
namespace gui {
namespace streaming_frame_policy {

inline constexpr size_t kPingTrainingSkipSamples = 4608;
inline constexpr size_t kPingRMSCheckSamples = 5000;
inline constexpr float kMinTrainingRMSForPingRatio = 0.001f;
inline constexpr float kPingMaxDataToTrainingRMSRatio = 0.5f;
inline constexpr float kPingChirpLockMaxDataRMS = 0.16f;
// Noise-relative payload-absence gate (#70 stage 1.5): a bare PING's gap is
// ambient noise, so gap_rms/idle_noise_rms ≈ 1 at every RX level and SNR. The
// 1.5x margin admits fade/estimator slop while a real 4-CW payload rides
// sqrt(1+SNR_lin) above the floor — 3.3x at a 10 dB in-band SNR, 2.2x at 6 dB.
// BOTH sides MUST be measured IN-BAND (same 50-2950 Hz FIR): in the raw
// domain, out-of-band ambient noise adds equally to both measurements and
// compresses the discriminant to sqrt(1 + SNR_lin*B_band/B_total) — measured
// 1.49x at sim good@10, straddling this factor, which misclassified a real
// CONNECT as a PING. Level-invariant, unlike the absolute
// kPingChirpLockMaxDataRMS floor (#74 lesson) — the absolute floor is kept as
// the fallback when the caller has no valid idle-noise estimate.
inline constexpr float kPingNoiseGapFactor = 1.5f;
inline constexpr float kMinIdleNoiseRMSForPingGap = 1e-4f;
inline constexpr float kPingCorrFloor = 0.30f;
inline constexpr float kPingMaxGapError = 1000.0f;
inline constexpr size_t kDefaultFalseLockAdvanceSamples = 1024;
inline constexpr size_t kMinFalseLockAdvanceSamples = 512;
inline constexpr float kMinSyncRecoveryCorrelation = 0.80f;

inline float rms(const float* samples, size_t count) {
    if (!samples || count == 0) {
        return 0.0f;
    }

    float sum = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        sum += samples[i] * samples[i];
    }
    return std::sqrt(sum / static_cast<float>(count));
}

struct PingFrameDecision {
    float training_rms = 0.0f;
    float data_rms = 0.0f;
    float ratio = 0.0f;
    float chirp_corr = 0.0f;
    float gap_error_samples = std::numeric_limits<float>::infinity();
    bool ldpc_decode_attempted = false;
    bool ldpc_decode_succeeded = false;
    bool ldpc_magic_valid = false;
    bool ping_by_silence = false;
    bool ping_by_chirp_lock = false;
    bool is_ping = true;
    float idle_noise_rms = 0.0f;   // 0 = caller had no valid idle-noise estimate
    float data_inband_rms = 0.0f;  // gap RMS through the in-band FIR (0 = not measured)
    bool gap_is_noise = false;     // noise-relative payload-absence verdict
    // BUG-SYNC-CURSOR-AHEAD: the training window read digital silence, which a live RX
    // path cannot produce (measured floor 0.009-0.011). Means the slice covered unwritten
    // ring memory — the decode is invalid, not a PING.
    bool buffer_invalid = false;
};

using PingRMSDecision = PingFrameDecision;

inline PingFrameDecision evaluatePingFrame(
    const float* samples, size_t count,
    size_t training_skip_samples = kPingTrainingSkipSamples,
    size_t rms_check_samples = kPingRMSCheckSamples,
    float chirp_corr = 0.0f,
    float gap_error_samples = std::numeric_limits<float>::infinity(),
    bool ldpc_decode_succeeded = false,
    bool ldpc_magic_valid = false,
    bool ldpc_decode_attempted = false,
    float idle_noise_rms = 0.0f,
    float data_inband_rms = 0.0f) {
    PingFrameDecision decision;

    const size_t train_len = std::min(training_skip_samples, count);
    decision.training_rms = rms(samples, train_len);

    const size_t check_start = std::min(training_skip_samples, count);
    const size_t check_len = std::min(count - check_start, rms_check_samples);
    decision.data_rms = rms(samples ? samples + check_start : nullptr, check_len);

    // BUG-SYNC-CURSOR-AHEAD (2026-07-30): DIGITAL SILENCE IS UNWRITTEN MEMORY, NOT A PING.
    //
    // A training window whose RMS is at or below kMinTrainingRMSForPingRatio (0.001) cannot
    // come from a live receiver. The measured noise floor on real hardware is 0.009-0.011
    // (Pi 5 rig, CCA and RMS-skip lines), an order of magnitude above it; even a disconnected
    // input carries dither. The only way to read ~0.0 is to slice ring memory that was never
    // written — reset() zero-fills the whole 50 s buffer (streaming_decoder.cpp:1393).
    //
    // Before this change that condition forced ratio = 0.0, which is the STRONGEST POSSIBLE
    // PING EVIDENCE (ping_by_silence = ratio < threshold). So the one observation that proves
    // the buffer is invalid was being read as proof of a valid PING. On the rig a real chirp
    // locked at corr=0.837, the decoder sliced past the write head into zeroes, and the log
    // printed "PING detected ... ratio=0.000" — a fabricated frame that then set a search
    // floor walling off the genuine CONNECT_ACK for 2.5 s.
    //
    // It must be the strongest possible REJECT instead. Keyed on training_rms specifically:
    // the low-SNR PING path (#70) deliberately accepts weak/flooded payloads via data_rms and
    // PATH2, and near-zero DATA rms is physically ordinary (a genuine silent gap). Near-zero
    // TRAINING rms is not — the training symbols are what the transmitter just sent.
    decision.buffer_invalid = (decision.training_rms <= kMinTrainingRMSForPingRatio);
    decision.ratio = (decision.training_rms > kMinTrainingRMSForPingRatio)
        ? decision.data_rms / decision.training_rms
        : 0.0f;
    decision.chirp_corr = chirp_corr;
    decision.gap_error_samples = gap_error_samples;
    decision.ldpc_decode_attempted = ldpc_decode_attempted;
    decision.ldpc_decode_succeeded = ldpc_decode_succeeded;
    decision.ldpc_magic_valid = ldpc_magic_valid;

    // An invalid buffer can never be evidence FOR a ping. Suppress both ping paths and
    // the final verdict; the caller then treats this as "no frame here" and keeps searching
    // real audio instead of fabricating one and walling off the region behind a search floor.
    decision.ping_by_silence =
        !decision.buffer_invalid && decision.ratio < kPingMaxDataToTrainingRMSRatio;
    const bool chirp_signature_real =
        chirp_corr >= kPingCorrFloor &&
        std::abs(gap_error_samples) <= kPingMaxGapError;
    const bool no_valid_frame = !ldpc_decode_succeeded || !ldpc_magic_valid;
    // PATH2 is for chirp-only probes whose payload area was too weak/noisy to
    // justify an LDPC decode. Once a full LDPC attempt has been made on a
    // data-bearing MC-DPSK frame, a failure is a frame failure, not evidence
    // that the peer sent a PING. This keeps faded CONNECT frames from eliciting
    // false PONGs while preserving low-SNR PING acquisition.
    //
    // NOTE: this is the POST-4-CW-decode ping classifier (streaming_ofdm_decode
    // PATH2, after the full fixed CONNECT decode has already FAILED). At that
    // point an absolute data_rms floor is acceptable as a last-resort tie-break
    // for a low-level noisy ping whose 4-CW decode produced nothing. The
    // *pre*-decode WAIT decision must NOT use this absolute floor (it would skip
    // the 4-CW CONNECT decode for a low-level-but-real CONNECT) — it gates on
    // ping_by_silence (ratiometric) only. See streaming_ofdm_decode.cpp ~1290.
    // Payload absence, most-principled test first: (a) ratiometric silence,
    // (b) noise-relative — the gap reads as the receiver's own idle noise
    // floor (level-invariant; the discriminator that works when broadband
    // noise floods the gap at low SNR, where (a) and (c) both fail),
    // (c) absolute-floor fallback for callers without a noise estimate.
    decision.idle_noise_rms = idle_noise_rms;
    decision.data_inband_rms = data_inband_rms;
    decision.gap_is_noise =
        idle_noise_rms > kMinIdleNoiseRMSForPingGap &&
        data_inband_rms > 0.0f &&
        data_inband_rms <= idle_noise_rms * kPingNoiseGapFactor;
    const bool payload_energy_absent =
        decision.ping_by_silence || decision.gap_is_noise ||
        decision.data_rms <= kPingChirpLockMaxDataRMS;
    decision.ping_by_chirp_lock =
        chirp_signature_real && no_valid_frame &&
        (!ldpc_decode_attempted || payload_energy_absent);
    // BUG-SYNC-CURSOR-AHEAD: suppressing ping_by_silence alone is NOT enough. The chirp-lock
    // path reaches the same verdict by a different route: on the rig the chirp signature was
    // genuinely real (corr=0.837 >= kPingCorrFloor), no LDPC frame validated, and
    // payload_energy_absent is satisfied by data_rms <= kPingChirpLockMaxDataRMS (0.16) —
    // which zero-filled memory trivially meets. So an unwritten slice would still have been
    // classified as a PING through ping_by_chirp_lock.
    //
    // An invalid slice must yield NO verdict in either direction. The chirp detection itself
    // remains real information (it is what put us here), but nothing can be concluded about a
    // payload that was never read; the caller must keep searching real audio rather than
    // fabricate a frame and wall the region off behind a search floor.
    if (decision.buffer_invalid) {
        decision.ping_by_chirp_lock = false;
        decision.is_ping = false;
        return decision;
    }
    decision.is_ping = decision.ping_by_silence || decision.ping_by_chirp_lock;
    return decision;
}

inline PingRMSDecision evaluatePingRMS(
    const float* samples, size_t count,
    size_t training_skip_samples = kPingTrainingSkipSamples,
    size_t rms_check_samples = kPingRMSCheckSamples) {
    return evaluatePingFrame(samples, count, training_skip_samples, rms_check_samples,
                             0.0f, std::numeric_limits<float>::infinity(),
                             true, true);
}

inline size_t falseOFDMLockAdvanceSamples(size_t frame_len, int data_preamble_samples) {
    size_t advance = kDefaultFalseLockAdvanceSamples;
    if (data_preamble_samples > 0) {
        advance = std::max(kMinFalseLockAdvanceSamples,
                           static_cast<size_t>(data_preamble_samples) / 2);
    }
    return std::min(advance, frame_len);
}

inline bool shouldRunControlFirstOFDMPeek(int pending_total_cw,
                                          bool is_ofdm,
                                          bool connected,
                                          size_t frame_len,
                                          size_t control_frame_samples) {
    return pending_total_cw == 0 &&
           is_ofdm &&
           connected &&
           frame_len <= control_frame_samples;
}

inline bool allowSyncRecovery(float sync_correlation) {
    return sync_correlation >= kMinSyncRecoveryCorrelation;
}

// The descriptor for group N announces whether group N+1 will use a chirp-less
// LIGHT anchor.  The reactive streak is credited from the just-observed prior
// delivery while group N is being encoded, so the announcement must use the
// post-credit streak.  Using the pre-credit streak creates a one-group wire
// contradiction exactly when the gate opens: N announces FULL, then N+1 sends
// LIGHT and the receiver rejects the entire burst while waiting for a chirp.
inline bool shouldAnnounceNextLightAnchor(uint32_t anchor_ordinal,
                                          int anchor_skip_k,
                                          uint32_t clean_streak_after_credit,
                                          uint32_t clean_streak_threshold) {
    return anchor_skip_k > 1 &&
           clean_streak_after_credit >= clean_streak_threshold &&
           static_cast<int>((anchor_ordinal + 1u) %
                            static_cast<uint32_t>(anchor_skip_k)) != 0;
}

// One source of truth for BURST_HEADER payload[4]. In particular, the
// current-group anchor bit must be derived from the same one-shot that chooses
// the physical preamble; otherwise sender and receiver silently disagree at
// the exact timeout-repair boundary where robust acquisition matters most.
inline constexpr uint8_t burstDescriptorFlags(bool burst_interleave,
                                               bool carrier_ldpc,
                                               bool next_light_anchor,
                                               bool current_group_full_anchor) {
    uint8_t flags = 0;
    if (burst_interleave) {
        flags |= protocol::v2::ControlFrame::BURST_FLAG_INTERLEAVE;
    }
    if (carrier_ldpc) {
        flags |= protocol::v2::ControlFrame::BURST_FLAG_CARRIER_LDPC;
    }
    if (next_light_anchor) {
        flags |= protocol::v2::ControlFrame::BURST_FLAG_NEXT_LIGHT_ANCHOR;
    }
    if (current_group_full_anchor) {
        flags |=
            protocol::v2::ControlFrame::BURST_FLAG_CURRENT_GROUP_FULL_ANCHOR;
    }
    return flags;
}

inline bool isNonDataFrame(bool success, const uint8_t* frame_data, size_t frame_size) {
    if (!success || !frame_data || frame_size < 3) {
        return false;
    }

    const auto type = static_cast<protocol::v2::FrameType>(frame_data[2]);
    return protocol::v2::isControlFrame(type) || protocol::v2::isConnectFrame(type);
}

inline bool shouldStampPhysicalBurstEnd(size_t frame_count,
                                        protocol::WaveformMode mode,
                                        bool supports_data_preamble,
                                        bool burst_interleave) {
    // MC-DPSK: every physical DATA turn, singletons included. Its only other burst-end
    // signal is the receiver failing to find a continuation header ~one codeword after the
    // last frame, which arrives AFTER that frame's ACK was requested; without the marker
    // the ACK was classified asynchronous, held for a whole maximum OFDM turn of decoder
    // evidence and dropped at its 2.5 s deadline every time (BUG-MCDPSK-ACK-DROPPED).
    if (mode == protocol::WaveformMode::MC_DPSK) return frame_count >= 1;
    return frame_count > 1 && protocol::isOFDMMode(mode) &&
           supports_data_preamble && !burst_interleave;
}

// Produce wire-owned DATA copies. The ARQ's serialized frames remain logical and
// byte-stable; a retry may regroup a seq at a different physical position, so every
// emission first clears the marker before stamping the exact final wire frame.
inline std::vector<Bytes> preparePhysicalBurstFrames(
    const std::vector<Bytes>& frames, bool stamp_physical_end) {
    std::vector<Bytes> wire_frames = frames;
    for (size_t i = 0; i < wire_frames.size(); ++i) {
        auto frame = protocol::v2::DataFrame::deserialize(wire_frames[i]);
        if (!frame || frame->type == protocol::v2::FrameType::DATA_REPAIR ||
            !protocol::v2::isDataFrame(frame->type)) {
            continue;
        }
        frame->flags = static_cast<uint8_t>(
            frame->flags & ~protocol::v2::Flags::PHYSICAL_BURST_END);
        if (stamp_physical_end && i + 1 == wire_frames.size()) {
            frame->flags = static_cast<uint8_t>(
                frame->flags | protocol::v2::Flags::PHYSICAL_BURST_END);
        }
        wire_frames[i] = frame->serialize();
    }
    return wire_frames;
}

inline bool hasPhysicalBurstEndMarker(bool decode_success,
                                      bool connected,
                                      bool is_ofdm,
                                      bool burst_transport,
                                      const Bytes& frame_data) {
    if (!decode_success || !connected || !is_ofdm || !burst_transport ||
        frame_data.size() < 4) {
        return false;
    }
    const auto header = protocol::v2::parseHeader(frame_data);
    return header.valid && !header.is_control &&
           protocol::v2::isDataFrame(header.type) &&
           header.type != protocol::v2::FrameType::DATA_REPAIR &&
           (frame_data[3] & protocol::v2::Flags::PHYSICAL_BURST_END) != 0;
}

// MC-DPSK DATA turns carry PHYSICAL_BURST_END on their last frame (every turn, singletons
// included — shouldStampPhysicalBurstEnd). A CRC-valid marked frame proves the peer's turn is
// over, so its ACK is a completed-turn ACK, not an asynchronous one.
inline bool hasMcDpskTurnEndMarker(bool decode_success,
                                   bool connected,
                                   bool is_mc_dpsk,
                                   const Bytes& frame_data) {
    if (!decode_success || !connected || !is_mc_dpsk || frame_data.size() < 4) {
        return false;
    }
    const auto header = protocol::v2::parseHeader(frame_data);
    return header.valid && !header.is_control &&
           protocol::v2::isDataFrame(header.type) &&
           header.type != protocol::v2::FrameType::DATA_REPAIR &&
           (frame_data[3] & protocol::v2::Flags::PHYSICAL_BURST_END) != 0;
}

inline bool shouldClearProvisionalBurstAirGate(bool decode_success,
                                               bool non_data_frame,
                                               bool gate_is_provisional,
                                               bool anchor_is_proven_full_chirp,
                                               uint64_t provisional_anchor_abs,
                                               uint64_t frame_sync_abs) {
    if (!decode_success || !gate_is_provisional) {
        return false;
    }
    // A classic frame decoded at the exact accepted anchor is a standalone
    // physical turn, even when its protocol flags are non-FINAL. A later light
    // frame belongs to the descriptor/marked-head-loss fallback and must retain
    // the original physical-air deadline. A protocol FINAL is only a logical
    // payload boundary: interleaver padding or other physical group members may
    // still follow it, so FINAL cannot authorize reverse egress. Control frames
    // are standalone on this classic decode route.
    return non_data_frame ||
           (anchor_is_proven_full_chirp &&
            frame_sync_abs == provisional_anchor_abs);
}

inline size_t consumedSamplesForDecodedFrame(bool success,
                                             bool is_ofdm,
                                             bool is_non_data_frame,
                                             int actual_codewords,
                                             size_t frame_len,
                                             size_t exact_codeword_samples) {
    if (success &&
        is_ofdm &&
        is_non_data_frame &&
        actual_codewords > 0 &&
        exact_codeword_samples < frame_len) {
        return exact_codeword_samples;
    }
    return frame_len;
}

}  // namespace streaming_frame_policy
}  // namespace gui
}  // namespace ultra
