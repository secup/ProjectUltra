#pragma once

#include "streaming_control_profile.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace ultra {
namespace gui {
namespace streaming_decode_policy {

// ============================================================================
// BUG-BURST-STALE-GEOMETRY (2026-07-28) — ULTRA_COMMANDED_GEOMETRY, DEFAULT-OFF
// ============================================================================
// Controls only the commanded-geometry experiment:
//   (B) on a MISSED burst descriptor, slice with the rung THIS receiver
//       COMMANDED instead of the latched one (resolveCommandedGeometry).
// The independent truncation/alignment repair (A) is an unconditional correctness
// guard and MUST NOT be coupled to this default-off experiment.
//
// DELIBERATELY NOT a function-local `static const` lambda (the house pattern for
// most knobs): a function-local static latches the env on FIRST call, so one
// process can only ever observe one value — and tests/test_burst_stale_geometry.cpp
// runs the pre-fix (=0) and fixed (=1) arms IN ONE PROCESS, which is what keeps the
// fail-before evidence live on every CI run instead of rotting in a comment. This
// runs at most a few times per burst group, never on a per-sample hot path.
// DEFAULT-OFF as of 2026-07-29 — flipped on RIG EVIDENCE, opt in with =1.
//
// WHY IT IS OFF. Measured on the IONOS rig at MPG@20, 8 interleaved transfers (4 pairs,
// Mac = receiver = where this decoder knob lives): the guard armed **once** — and in that same
// batch there were **7 groups that arrived with no BURST_HEADER descriptor at all**, i.e. 7
// opportunities. cmd_arms read 0 in 7 of the 8 runs. The arm-mean goodput difference (ON 1.83
// vs OFF 1.50 kbps) is therefore a NULL CONTROL, not an effect: a knob that provably did
// nothing cannot have caused a 23% swing, and reading it as one would be the exact scoring
// error this project has been bitten by before.
//
// The suppression was PREDICTED before it was measured: three `have_burst_descriptor_`
// stale-TRUE leaks (MODEM_INFRASTRUCTURE_MAP §7b — the accumulateBurstFrames hard-failure
// abort, the two deinterleave-throw early returns, and the group-timeout clear gated on
// burst_transport_rx_ && burst_group_callback_) leave the latch TRUE, so the resolver's
// `have_burst_descriptor_` early-out declines to arm. Shipping this DEFAULT-ON would ship
// code that does not engage.
//
// RE-ENABLE ONLY AFTER: (1) those three leaks are fixed so the guard can actually arm, and
// (2) a fresh interleaved rig A/B shows cmd_arms > 0 in the ON arm. Until then the honest
// state is "implemented, reviewed, not yet exercised".
inline bool commandedGeometryEnabled() {
    const char* e = std::getenv("ULTRA_COMMANDED_GEOMETRY");
    return e != nullptr && e[0] != '\0' && e[0] != '0';  // DEFAULT-OFF (rig: does not engage)
}

// Sample-clock gap that separates a STEADY-STATE burst group from a post-ACK-RTO
// resend. Already in-tree and justified in-comment at the descriptor-consume site
// (gate D2): steady-state groups arrive every burst+turnaround (<= ~11.5 s worst
// case) while a timeout resend can only follow the sender's ACK RTO (>= ~19 s of
// silence). Measured on the 2026-07-28 rig capture: steady-state 7.00-13.33 s
// (n=24) vs post-RTO 19.43/20.01 s (n=2) — disjoint, 15 s sits between.
// ONE definition, two consumers (D2 provisional-HARQ keys + the commanded-geometry
// cadence guard) so they cannot drift apart.
inline constexpr uint64_t kBurstCadenceRtoGapSamples = 15ull * 48000ull;

// Keep the timeout discriminator independent from anchor choice. A selective
// repair is intentionally announced with a FULL current-group anchor, but it is
// still a steady-cadence ARQ repair and remains safe for a descriptor-proven
// provisional HARQ key. A real sample-clock gap blocks that key path; unknown
// or non-forward cadence also fails closed.
inline constexpr bool burstCadenceBlocksProvisionalHarq(
    uint64_t previous_descriptor_abs, uint64_t current_descriptor_abs) {
    // Unknown or non-forward time is not evidence of steady cadence. The first
    // descriptor seeds the reference; the following prompt repair may qualify.
    return previous_descriptor_abs == 0 ||
           current_descriptor_abs <= previous_descriptor_abs ||
           (current_descriptor_abs - previous_descriptor_abs) >
               kBurstCadenceRtoGapSamples;
}

// A decoded descriptor normally hands its trusted timing directly to the
// following light-LTS DATA marker. A sender-announced full current-group anchor
// is an explicit exception: retaining the light prediction would point inside
// the chirp prefix. Mode hops remain the other exception because their carrier
// geometry invalidates the warm channel state.
inline constexpr bool keepDescriptorWarmHandoff(bool phase_is_warm,
                                                float confidence,
                                                bool current_group_full_anchor,
                                                bool descriptor_mode_switch_enabled,
                                                bool descriptor_mode_hop) {
    return phase_is_warm && confidence > 0.0f &&
           !current_group_full_anchor &&
           !(descriptor_mode_switch_enabled && descriptor_mode_hop);
}

// ULTRA_HARQ_PROVISIONAL remains a deliberately narrow, default-off experiment.
// A failed CW0 hides both the ARQ identity and PHYSICAL_BURST_END.  The receiver
// may predict the identity only for the currently-authorized profiles. A tail
// member may be observed to validate the mirror, but may never use a provisional
// key: a retry can regroup the same ARQ seq at another physical position, and
// tail/non-tail DATA frames have different protected bits.
//
// This helper contains every cheap gate that is known before pulling the ARQ
// mirror callback.  The caller separately verifies that the returned context is
// valid and contains this logical position.
inline constexpr bool allowProvisionalHarqContext(
    bool opt_in,
    bool burst_transport,
    bool exact_descriptor_proven,
    bool burst_interleave,
    bool cadence_blocked,
    bool prediction_invalid,
    bool context_callback_available,
    Modulation modulation,
    CodeRate rate,
    int frame_cw_count,
    int lifting_z,
    int logical_index,
    int declared_group_size) {
    const bool qpsk_r34_profile =
        modulation == Modulation::QPSK && rate == CodeRate::R3_4;
    // The new 8PSK authorization deliberately matches only the live profile
    // that exposed the repeated CW0 holes. Keep cw12/Z27 and any future 8PSK
    // geometry outside this experiment until measured independently.
    const bool psk8_long_profile =
        modulation == Modulation::QAM8 && rate == CodeRate::R2_3 &&
        frame_cw_count == 4 && lifting_z == 81;
    const bool authorized_profile = qpsk_r34_profile || psk8_long_profile;
    return opt_in && burst_transport && exact_descriptor_proven &&
           !burst_interleave && !cadence_blocked && !prediction_invalid &&
           context_callback_available && authorized_profile && logical_index >= 0 &&
           declared_group_size >= 2 &&
           logical_index < declared_group_size;
}

// A profile/cadence-eligible context is still only a prediction. Do not use it
// as a soft-combine key until at least one real CW0 header in this physical group
// has matched the mirror. This closes the all-unknown-prefix contamination path.
inline constexpr bool allowProvisionalHarqKey(bool context_allowed,
                                              bool prediction_validated,
                                              int logical_index,
                                              int declared_group_size) {
    return context_allowed && prediction_validated && logical_index >= 0 &&
           declared_group_size >= 2 &&
           logical_index < declared_group_size - 1;
}

inline constexpr bool provisionalHarqPredictionMatchesHeader(
    uint32_t predicted_sender_hash, uint32_t predicted_dst_hash,
    uint16_t predicted_seq, uint32_t actual_sender_hash,
    uint32_t actual_dst_hash, uint16_t actual_seq) {
    return predicted_sender_hash == actual_sender_hash &&
           predicted_dst_hash == actual_dst_hash &&
           predicted_seq == actual_seq;
}

inline constexpr bool provisionalHarqContextCoversPosition(
    bool context_valid, int logical_index, size_t predicted_seq_count) {
    return context_valid && logical_index >= 0 &&
           static_cast<size_t>(logical_index) < predicted_seq_count;
}

inline size_t estimateRobustOFDMControlSamples(size_t default_control_samples,
                                               Modulation data_mod,
                                               CodeRate data_rate,
                                               int carriers,
                                               int samples_per_symbol) {
    const auto control_profile =
        streaming_control_profile::profileForDataMode(data_mod);
    if (control_profile.modulation == data_mod && data_rate == control_profile.rate) {
        return default_control_samples;
    }

    if (carriers <= 0 || samples_per_symbol <= 0) {
        return default_control_samples;
    }

    constexpr int kLDPCCodewordBits = 648;
    const int control_pilot_spacing =
        streaming_control_profile::pilotSpacingForProfile(control_profile);
    const int bits_per_symbol = ofdm_link_adaptation::bitsPerOFDMSymbol(
        carriers, true, control_pilot_spacing, control_profile.modulation);
    if (bits_per_symbol <= 0) {
        return default_control_samples;
    }

    const int data_symbols = (kLDPCCodewordBits + bits_per_symbol - 1) / bits_per_symbol;
    const size_t robust_samples = static_cast<size_t>(2 + data_symbols) *
                                  static_cast<size_t>(samples_per_symbol);
    return std::max(default_control_samples, robust_samples);
}

enum class DecodeSampleMode {
    PendingCodewords,
    ConnectedOFDMPeek,
    ConnectedOFDMBurst,
    ControlPeek,
};

struct DecodeSampleRequirement {
    size_t samples = 0;
    DecodeSampleMode mode = DecodeSampleMode::ControlPeek;
};

// A frame wait gives up only on audio that is late: when the frame's samples are already in the
// ring it is decoded, whatever the clock says. The clock runs from the sync, so a requirement that
// shrinks after a long wait - a connected MC-DPSK receiver's first-frame slice (7.1 s at 4
// codewords), then the 3.6 s fixed CONNECT it turned out to be - read as a timeout with every
// sample present, and a responder whose CONNECT_ACK was lost never decoded the initiator's repeats
// ("Frame timeout after 7072 ms (need=175104 samples, timeout=5648 ms)", GUI ITU Good 0 dB).
inline constexpr bool frameWaitTimedOut(size_t available_samples, size_t required_samples,
                                        long long elapsed_ms, int timeout_ms) {
    return available_samples < required_samples && elapsed_ms > timeout_ms;
}

inline bool hasSubFixedFrameSoftBits(size_t soft_bits,
                                     int fixed_frame_codewords,
                                     size_t ldpc_block_bits) {
    if (fixed_frame_codewords <= 0 || ldpc_block_bits == 0) {
        return false;
    }

    const size_t fixed_frame_bits =
        static_cast<size_t>(fixed_frame_codewords) * ldpc_block_bits;
    return soft_bits >= ldpc_block_bits && soft_bits < fixed_frame_bits;
}

// 2026-05-29 channel-adaptive interleaver: burst_regime_active means "in a burst
// file-transfer" (interleave-INDEPENDENT — use_burst_interleave_ || burst_transport_rx_),
// NOT "byte interleave enabled". burst_latched means "the negated-LTS group-start
// marker was detected". A group-start data frame is sized as a FULL data frame (not a
// control peek) whenever a marker is latched in the burst regime — this holds whether
// or not the group's bytes are interleaved, because the encoder emits the marker for
// every group regardless. (Pre-decouple this branch was gated on the interleave flag,
// which silently mis-sized interleave-off groups as control peeks → they never decoded.)
inline DecodeSampleRequirement selectDecodeSampleRequirement(int pending_total_cw,
                                                             bool is_ofdm,
                                                             bool connected,
                                                             bool burst_regime_active,
                                                             bool burst_latched,
                                                             size_t pending_cw_samples,
                                                             size_t ofdm_control_samples,
                                                             size_t full_frame_samples,
                                                             size_t control_frame_samples) {
    if (pending_total_cw > 0) {
        return {pending_cw_samples, DecodeSampleMode::PendingCodewords};
    }

    if (is_ofdm && connected) {
        if (burst_regime_active && burst_latched) {
            return {full_frame_samples, DecodeSampleMode::ConnectedOFDMBurst};
        }
        return {ofdm_control_samples, DecodeSampleMode::ConnectedOFDMPeek};
    }

    return {control_frame_samples, DecodeSampleMode::ControlPeek};
}

}  // namespace streaming_decode_policy
}  // namespace gui
}  // namespace ultra
