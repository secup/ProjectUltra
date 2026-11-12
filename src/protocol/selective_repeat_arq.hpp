#pragma once

#include "arq_interface.hpp"
#include "frame_v2.hpp"
#include "selective_repeat_arq_policy.hpp"
#include <algorithm>
#include <deque>
#include <functional>
#include <optional>
#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace ultra {
namespace protocol {

/**
 * Selective Repeat ARQ Controller
 *
 * Sliding window ARQ for higher throughput on HF:
 * - Maintains window of N frames in flight simultaneously
 * - Retransmits only failed frames (not entire window)
 * - Uses SACK (Selective ACK) with bitmap for efficiency
 * - Reorders out-of-order frames at receiver
 *
 * Compared to Stop-and-Wait:
 * - 2-4x higher throughput for typical HF RTT
 * - More complex state management
 * - Requires more memory (TX/RX buffers)
 *
 * Window size should be chosen based on:
 * - Round-trip time (RTT = propagation + processing)
 * - Frame duration
 * - Target throughput
 */
class SelectiveRepeatARQ : public IARQController {
public:
    struct DeferredTimeoutCommitResult {
        std::vector<Bytes> frames;
        bool terminal_failure = false;
    };

    explicit SelectiveRepeatARQ(const ARQConfig& config = ARQConfig{});

    // --- IARQController Interface ---

    ARQMode getMode() const override { return ARQMode::SELECTIVE_REPEAT; }

    void setCallsigns(const std::string& local, const std::string& remote) override;

    bool sendData(const Bytes& data) override;
    bool sendData(const std::string& text) override;
    bool sendDataWithFlags(const Bytes& data, uint8_t flags) override;
    bool sendDataWithTypeAndFlags(const Bytes& data, v2::FrameType frame_type, uint8_t flags);
    bool sendFixedDataWithFlags(const Bytes& data, uint8_t flags);
    bool sendFixedDataWithTypeAndFlags(const Bytes& data, v2::FrameType frame_type, uint8_t flags);
    bool sendVariableDataWithFlags(const Bytes& data, uint8_t flags);

    bool isReadyToSend() const override;
    size_t getAvailableSlots() const override;
    size_t getTxInFlightBytes() const;
    // True when tick(elapsed_ms) will retransmit at least one frame on TIMEOUT (same conditions as
    // tick: an active unacked slot whose timer expires, not suspended, no DATA_REPAIR guard). Lets
    // the caller change the PHY profile BEFORE the resend is rendered (MC-DPSK timeout fallback).
    bool timeoutRetransmitDue(uint32_t elapsed_ms) const;
    // Highest retransmit count among active un-acked TX slots (0 if nothing in flight). The
    // rate controller polls this to detect a frame STUCK at a too-aggressive rate (the fade
    // troughs keep killing it) so it can escape-drop to a more robust rung before max_retries.
    int maxInFlightRetryCount() const;
    // The most ACK rounds any in-flight frame has been missing from (heard_miss_count).
    int maxInFlightHeardMissCount() const;

    bool lastRxHadMoreData() const override { return last_rx_more_data_; }
    uint8_t lastRxFlags() const override { return last_rx_flags_; }
    v2::FrameType lastRxFrameType() const { return last_rx_frame_type_; }

    // §RETX-PACING (docs/RETX_PACING_DESIGN_2026_07_03.md §1.1): forward progress of the
    // most recent FRESH ack processed by handleAckFrame = (frames retired by the cumulative
    // base advance) + (newly-set SACK bits). −1 = no fresh ack since the last
    // consumeAckProgress() — stale/future/DUPLICATE acks (the existing ack-signature dedup)
    // return early and never touch this, so re-heard SACK copies cannot fabricate a phantom
    // zero-progress round. This ARQ-window state is the identity-agnostic ground truth for
    // round accounting (never FileTransfer chunk counters — BUG-FILE-ACK-IDENTITY).
    int lastAckProgressFrames() const { return last_ack_progress_frames_; }
    // Round consumption: the Connection reads the outcome exactly once per round boundary,
    // then re-arms the "no ack this round" sentinel.
    void consumeAckProgress() { last_ack_progress_frames_ = -1; }

    // §RETX-PACING §1.3 trigger #2: push every pending (active, un-acked) TX slot's
    // retransmit timer out by `ms`, so the per-slot RTO cannot blind-fire around a
    // trough-pacing hold armed at a round boundary (one hold state must gate BOTH the
    // turn refill and the slot RTO, or the RTO leaks around the hold). Deliberately NOT
    // a global freeze: acked/inactive slots, ACK-repeat jobs, SACK timers, hole-probe
    // state and the receiver role are untouched.
    void deferPendingRetransmits(uint32_t ms);

    void onFrameReceived(const Bytes& frame_data) override;
    void onPartialFrame(const v2::PartialFrameCodewords& partial);

    void tick(uint32_t elapsed_ms) override;

    void setTransmitCallback(TransmitCallback cb) override;
    void setDataReceivedCallback(DataReceivedCallback cb) override;
    void setSendCompleteCallback(SendCompleteCallback cb) override;

    using TransmitBatchCallback = std::function<void(const std::vector<Bytes>&)>;
    void setTransmitBatchCallback(TransmitBatchCallback cb);

    // Connection's collapse escape needs to inspect a timeout round after tick() has
    // unwound but before any old-geometry DATA reaches the physical transmitter.  This
    // opt-in changes only timeout batches: tick publishes exact serialized identities as
    // intents, and the owner must either commit or cancel them synchronously after tick.
    // Ordinary SelectiveRepeatARQ users retain the historical callback-is-egress contract.
    void setDeferredTimeoutCommitEnabled(bool enabled) {
        deferred_timeout_commit_enabled_ = enabled;
    }
    DeferredTimeoutCommitResult commitDeferredTimeoutRetransmits(
        const std::vector<Bytes>& intents);
    size_t cancelDeferredTimeoutRetransmits(const std::vector<Bytes>& intents,
                                            bool suspend_until_mode_resolution);
    size_t resumeDeferredTimeoutRetransmits(uint32_t timeout_ms = 1);

    // TRANSPORT MERGE (step 1): tone-burst ACK on the interactive path. When this
    // callback is installed, sendSack() emits the ack as a fast tone-burst —
    // base_seq + the (low 6 bits of the) RX bitmap — via the callback instead of a
    // SACK control frame. Installed only when the feature is enabled; otherwise the
    // legacy SACK-frame path is used unchanged. has_final mirrors the FINAL flag the
    // SACK would have carried. move_epoch (2026-07-03, ULTRA_ARQ_MOVE_EPOCH) is the
    // receiver's adopted 2-bit move-epoch to echo in the tone-burst payload bits
    // 40-41; always 0 while the knob is OFF.
    using ToneBurstSackCallback =
        std::function<void(uint16_t base_seq, uint32_t bitmap, bool has_final,
                           uint8_t move_epoch)>;
    void setEmitToneBurstSackCallback(ToneBurstSackCallback cb) {
        on_emit_tone_burst_sack_ = std::move(cb);
    }
    // A callback can be wired before the host has installed a physical tone
    // transmitter.  Keep the legacy control-frame plane alive until that transport is
    // genuinely ready; otherwise a tail gap can suppress its only usable feedback.
    void setToneBurstSackTransportReady(bool ready) {
        tone_burst_sack_transport_ready_ = ready;
    }
    // Sender side: consume an incoming tone-burst ACK. Reconstructs the full 16-bit
    // ack base from the 6-bit group_seq (nearest to tx_base-1) and drives the standard
    // ack path (handleAckFrame), so selective-repeat behaves identically to a SACK.
    // move_epoch is the payload's epoch echo (bits 40-41); ignored while
    // ULTRA_ARQ_MOVE_EPOCH is OFF.
    // Returns false only when the payload is outside the sent-window/epoch support and
    // therefore must not drive Connection-level rate, ALC, or refill side effects. A valid
    // same-epoch duplicate returns true: it may be a fresh physical keepalive after a lost
    // repair ACK even though it contributes no new ARQ progress.
    bool onToneBurstAck(uint8_t group_seq6, uint32_t bitmap, uint8_t move_epoch);

    // Side-effect-free exact-identity query for Connection's immediately preceding
    // physical round.  A frame is counted only while the same serialized DATA
    // identity is still active and unACKed; SACKed or cumulatively retired frames
    // therefore count as delivered.  Full identity matching is intentional: a bare
    // 16-bit seq can be reused after wrap and must not authorize a light repair.
    size_t countUnackedFrameIdentities(const std::vector<Bytes>& frames) const;

    // Side-effect-free counterpart used by the PHY monitor before it commits a
    // CRC-valid timing candidate. Checks sequence/epoch support and rejects bitmap
    // claims for frames this sender has not emitted. The stateful onToneBurstAck()
    // repeats the same gates as defense in depth.
    bool isToneBurstAckPlausible(uint8_t group_seq6, uint32_t bitmap,
                                 uint8_t move_epoch) const;

    // Receiver side, BURST-AWARE ACK (transport merge): the unified path delivers a
    // whole decoded burst (group) at once. The receiver knows the group boundary, so
    // instead of the per-frame coalescing heuristic (which can't see "the group ended"
    // and stalls a sub-window burst), bracket the group's frames with these:
    //   beginGroupReceive() — suppress the per-frame ack decision while feeding frames;
    //   endGroupReceiveAndAck() — emit EXACTLY ONE tone-burst ack (cumulative base +
    //     hole bitmap) for the whole burst. Always emits, even for an all-duplicate
    //     group, so a sender retransmit can never get stuck waiting for an ack.
    void beginGroupReceive() { group_ack_deferred_ = true; }
    void endGroupReceiveAndAck();

    // STOP-AND-WAIT burst sender: resend the in-flight UNACKED frames (the holes the
    // receiver is missing) on demand, in seq order from the window base, up to
    // max_frames. The caller invokes this with burst buffering open so the holes land in
    // the SAME group as the new frames that fill the rest of the budget ([holes]+[new]),
    // keeping the pipe full instead of sending a lonely 1-frame resend. Returns how many
    // were actually resent (a frame that hits max_retries is dropped, not counted).
    size_t retransmitInFlightUnacked(size_t max_frames);

    // Frames received OUT OF ORDER and buffered in the RX window past the current base
    // (a hole below them blocks in-order delivery). The unified path delivers in order,
    // so these aren't visible to file_transfer_ yet; surface the count so the GUI's
    // "received" progress advances on ANY frame, not just contiguous ones.
    size_t bufferedRxFrameCount() const;

    ARQStats getStats() const override { return stats_; }
    void resetStats() override { stats_ = ARQStats{}; }

    void reset() override;
    // Drop all pending TX slots and advance to a fresh sequence boundary. A terminal
    // frame failure has already removed the failed slot before Connection can unwind,
    // so callers handling that event must force the move-epoch bump even when no other
    // live slot remains; otherwise the peer can stay pinned behind the abandoned seq.
    void abortPendingTx(bool force_move_epoch_bump = false);
    void clearPendingAckRepeats();

    // Set the code rate for DATA frame total_cw calculation
    void setCodeRate(CodeRate rate);
    CodeRate getCodeRate() const { return code_rate_; }

    // DESC-SWITCH telemetry (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.1):
    // read-only view of the TX move-epoch for the sender's commit log. Always 0 while
    // ULTRA_ARQ_MOVE_EPOCH is OFF; a CLEAN-boundary commit never bumps it (an empty
    // window has nothing to abort — see setCodeRate's bump condition).
    uint8_t txMoveEpoch() const { return tx_epoch_; }
    // RX-RATE-CMD Phase 2 gate (design §5.2/§7): a receiver-commanded MID-WINDOW
    // descriptor commit regrids live seqs — only era-safe when the move-epoch
    // machinery is active. Exposes the ctor-latched ULTRA_ARQ_MOVE_EPOCH state so
    // Connection can fall back to the legacy MODE_CHANGE exchange when it is OFF.
    bool moveEpochEnabled() const { return move_epoch_enabled_; }
    // WAITING-REBASE voice (BUG-UNANCHORED-SILENCE-ESCAPE, design §5.3): receiver
    // side — true while the unanchored interregnum holds (epoch adopted, era base
    // not yet seen, total ack silence). Connection reads it per burst-group event
    // to emit the rung_cmd=3 "waiting-rebase" tone signal (ULTRA_RX_RATE_CMD).
    bool rxWaitingRebase() const { return rx_epoch_wait_rebase_; }
    // WAITING-REBASE voice, sender side: the peer told us it is unanchored — the
    // era-base frame (head-of-burst, the most fade-exposed acquisition slot) keeps
    // dying. Force the BASE slot's retransmit timer due so the existing resend
    // machinery re-sends it promptly — typically as a STANDALONE single-frame
    // burst with its own preamble (a different acquisition shape than the
    // head-of-burst slot that kept failing). All standard pacing/dedup applies.
    void expireBaseSlotTimerForRebase();

    // Set the complete fixed-OFDM DATA geometry. Lifting Z is part of the
    // serializer contract, not only the PHY contract: cw4/Z81 and cw4/Z27 have
    // different payload capacities even though their frame headers carry the
    // same total_cw value. Changing either coordinate aborts pending TX so one
    // ARQ window can never mix serialized geometries.
    void setFixedFrameGeometry(int cw_count, int lifting_z);

    // Compatibility setters for callers that change one coordinate at a time.
    // New connection-layer code should prefer setFixedFrameGeometry().
    void setFixedFrameCodewords(int cw_count);
    int getFixedFrameCodewords() const { return fixed_frame_codewords_; }
    void setFixedFrameLiftingZ(int lifting_z);
    int getFixedFrameLiftingZ() const { return fixed_frame_lifting_z_; }

    // Set window size (1 = stop-and-wait behavior for MC-DPSK)
    void setWindowSize(size_t size) {
        size = selective_repeat_arq_policy::clampWindowSize(size, MAX_WINDOW);
        config_.window_size = size;
        // Clamp ack_batch_size if it would exceed the new window — protects
        // against setter ordering where Connection::enterConnected() shrinks
        // the window after a wider batch was previously configured.
        config_.ack_batch_size = selective_repeat_arq_policy::clampAckBatchSize(
            config_.ack_batch_size, config_.window_size, MAX_WINDOW);
    }
    size_t getWindowSize() const { return config_.window_size; }
    // Frames this end puts in flight at most (0 = the window). Transmit side only: the receive
    // window stays the window, because the peer's burst length is the peer's choice and this end
    // acknowledges at the burst's marked last frame whatever its length (PHYSICAL_BURST_END).
    void setTxInFlightCap(size_t cap) { tx_in_flight_cap_ = cap; }
    size_t getTxInFlightCap() const { return tx_in_flight_cap_; }
    // Advances whenever in-flight frames are abandoned (a rate or frame-geometry change, an
    // explicit abort). A caller that keeps a ledger of what those frames carried - the file
    // layer - rewinds it exactly when this moves, never on a prediction of it.
    uint32_t txAbortGeneration() const { return tx_abort_generation_; }
    uint16_t getRxBaseSeq() const { return rx_base_seq_; }

    // Keepalive ACK (BUG-ANCHOR-WAIT-NO-ACK-STALL, 2026-07-14): when we are
    // actively receiving a file (more frames expected) but our ACK side has
    // gone silent for threshold_ms — bursts arriving yet rejected at sync
    // (marginal corr) or failing before decode, so no SACK was emitted — we
    // re-emit the current cumulative ACK. The sender treats ANY tone-burst ACK
    // as a turn boundary and resends the holes immediately, converting a full
    // 44 s RTO stall into a fast turnaround. Routes through the normal
    // listen-before-ACK channel gating. Returns true if a keepalive was sent.
    bool keepaliveAckIfStalled(uint32_t threshold_ms);
    uint16_t getTxBaseSeq() const { return tx_base_seq_; }
    // HARQ provisional keys (2026-07-01): the receiver's mirror of the
    // sender's next-burst seq fill = ascending !received seqs in the rx
    // window. The sender fills bursts [unacked holes in window order][new
    // sequential seqs] (sendNextFileChunk), both window-bound, so the
    // concatenation is globally ascending and equals this complement
    // EXACTLY whenever the sender acted on our last SACK. Indexed by burst
    // logical position; divergence cases (lost SACK, timeout batch) are
    // handled by the caller's gates + the frame-CRC guard.
    std::vector<uint16_t> predictedIncomingSeqs(size_t max_n) const;

    // ACK batch size: send SACK after this many in-order data frames received.
    // 0 (default) means "track window_size" — preserves prior behavior bit
    // for bit. Nonzero values must be <= window_size; otherwise ALPHA can
    // saturate the window before BRAVO ACKs and stall on the sack_delay
    // fallback timer.
    void setAckBatchSize(uint32_t n) {
        config_.ack_batch_size = selective_repeat_arq_policy::clampAckBatchSize(
            n, config_.window_size, MAX_WINDOW);
    }
    uint32_t getAckBatchSize() const { return config_.ack_batch_size; }

    // Set ACK timeout (adaptive based on waveform frame duration)
    // In-flight-aware timeout table. Index = frames outstanding, 1..kMaxWindow; entry [window]
    // must equal the scalar. Supplied by the Connection from the same policy function, so the
    // ARQ holds no timing knowledge; (nullptr, 0) clears it and the scalar applies again.
    void setAckTimeoutTable(const uint32_t* table, size_t count) {
        ack_timeout_table_n_ = std::min(count, sizeof(ack_timeout_table_) / sizeof(uint32_t));
        for (size_t i = 0; i < ack_timeout_table_n_; ++i) ack_timeout_table_[i] = table[i];
    }

    // Measured burst timeout (ULTRA_MCDPSK_MEASURED_RTO; the Connection supplies the burst's
    // airtime model: a burst of n frames lasts burst_overhead_ms + n x frame_airtime_ms). Each
    // first-transmission burst whose ACK comes back yields one sample of the post-burst ACK
    // latency - the ACK's arrival minus the modelled end of the burst (the peer's decode, T/R,
    // ACK airtime, detection), RFC 6298 smoothed, Karn-safe - and a burst's timeout becomes its
    // airtime + latency + max(4 x variation, guard_ms) + extra_ms, doubled after each consecutive
    // timeout round that heard no ACK (RFC 6298 back-off, which keeps the outage horizon of
    // max_retries rounds), and never above the in-flight table (setAckTimeoutTable), which
    // governs alone until the first sample. Works only with that table.
    void setMeasuredBurstTimeouts(uint32_t burst_overhead_ms, uint32_t frame_airtime_ms,
                                  uint32_t guard_ms, uint32_t extra_ms) {
        measured_burst_rto_ = frame_airtime_ms > 0;
        burst_overhead_ms_ = burst_overhead_ms;
        frame_airtime_ms_ = frame_airtime_ms;
        latency_guard_ms_ = guard_ms;
        latency_extra_ms_ = extra_ms;
    }
    void clearMeasuredBurstTimeouts() { measured_burst_rto_ = false; }
    bool measuredBurstTimeoutsEnabled() const { return measured_burst_rto_; }
    bool haveMeasuredPostBurstLatency() const { return have_post_burst_latency_; }
    float postBurstLatencySrttMs() const { return post_burst_latency_srtt_ms_; }
    float postBurstLatencyRttvarMs() const { return post_burst_latency_rttvar_ms_; }
    uint32_t silentTimeoutRounds() const { return silent_timeout_rounds_; }

    void setAckTimeout(uint32_t timeout_ms) {
        config_.ack_timeout_ms = timeout_ms;
        if (!have_rtt_estimator_) {
            adaptive_ack_timeout_ms_ = timeout_ms;
        }
    }
    uint32_t getAckTimeout() const { return config_.ack_timeout_ms; }
    // The retransmission timeout for a burst of `frames` in flight (the INFLIGHT-RTO table): both
    // ends build it for the rung, so a receiver can predict when the sender resends a group.
    uint32_t ackTimeoutForFrameCount(size_t frames) const { return ackTimeoutForFrames(frames); }
    // Forget a pending (delayed) SACK without sending it: an ACK that could not reach the sender
    // before its resend is not keyed at all. What it would have reported rides the next ACK, which
    // is cumulative (base + hole bitmap).
    void dropPendingSack() {
        sack_pending_ = false;
        sack_timer_ms_ = 0;
        frames_since_ack_ = 0;
        group_ack_deferred_ = false;
    }

    // Commit an explicit deadline to the exact DATA frame identities that physically
    // went on air in one half-duplex round. Padding contributes to the Connection's
    // timeout calculation but never matches a TX slot. Other live holes are suspended
    // until a later physical round actually emits them, preventing a near-expiry timer
    // from launching a second half-duplex burst under the current one. This bypasses
    // (without mutating) the mode-wide/adaptive estimator and can move a transmitted
    // slot's timer both up and down.
    size_t rearmTransmittedDataFrames(const std::vector<Bytes>& frames,
                                      uint32_t timeout_ms);

    // Set delayed SACK coalescing timer
    void setSackDelay(uint32_t ms) { config_.sack_delay_ms = std::max(1u, ms); }
    uint32_t getSackDelay() const { return config_.sack_delay_ms; }

    // Stream-aware tail-of-burst override: when set to nonzero, frames
    // explicitly marked FINAL arm the SACK timer at this shorter delay instead
    // of sack_delay_ms. Ordinary MORE_FRAG=0 message boundaries are not enough:
    // a multi-message physical burst can contain several such frames.
    // Sentinel 0 (default) preserves prior behavior bit-for-bit.
    void setSackDelayShort(uint32_t ms) { sack_delay_short_ms_ = ms; }
    uint32_t getSackDelayShort() const { return sack_delay_short_ms_; }

    // OFDM burst streams need a quiet-interval SACK timer: each decoded DATA
    // frame proves the physical burst is still arriving, so the delayed SACK
    // timer should slide forward until an airtime-derived burst-tail interval
    // has passed.
    // Disabled by default to preserve legacy "earliest timer wins" behavior.
    void setSackDelaySlidesOnData(bool enabled) { sack_delay_slides_on_data_ = enabled; }
    bool getSackDelaySlidesOnData() const { return sack_delay_slides_on_data_; }

    // MC-DPSK continuous bursts decode several DATA frames from one physical
    // waveform. Once the ACK batch threshold is reached, transmitting a SACK
    // no longer risks colliding with a per-frame preamble still in flight.
    // Leave disabled for OFDM streams, where MORE_FRAG is still the guard.
    void setAckBatchThroughMoreFrag(bool enabled) { ack_batch_through_more_frag_ = enabled; }
    bool getAckBatchThroughMoreFrag() const { return ack_batch_through_more_frag_; }

    // Immediate out-of-order SACKs are the default recovery safety valve. OFDM
    // physical-hold profiles can defer in-burst hole reports until the sender's
    // burst airtime has cleared, avoiding a control turn inside DATA.
    void setImmediateOutOfOrderSackEnabled(bool enabled) {
        immediate_out_of_order_sack_enabled_ = enabled;
    }
    bool getImmediateOutOfOrderSackEnabled() const {
        return immediate_out_of_order_sack_enabled_;
    }

    // Set max retries before giving up on a frame
    void setMaxRetries(int retries) { config_.max_retries = std::max(1, retries); }
    int getMaxRetries() const { return config_.max_retries; }

    // ACK repeat: send multiple copies with delay for fading reliability
    void setAckRepeatCount(int count) { ack_repeat_count_ = std::clamp(count, 1, 3); }
    void setAckRepeatDelay(uint32_t ms) { ack_repeat_delay_ms_ = std::max(1u, ms); }
    void setAckRepeatPeerBurstGuardMs(uint32_t ms) { ack_repeat_peer_burst_guard_ms_ = ms; }
    int getAckRepeatCount() const { return ack_repeat_count_; }
    uint32_t getAckRepeatDelay() const { return ack_repeat_delay_ms_; }
    uint32_t getAckRepeatPeerBurstGuardMs() const {
        return ack_repeat_peer_burst_guard_ms_.value_or(config_.sack_delay_ms);
    }

    // Tone-burst partial (hole-bearing) SACK sliding delay: fires this long after the
    // LAST out-of-order frame decoded in a burst, coalescing the burst's holes into one
    // SACK that the sender hears in the inter-burst gap. MUST be >= one sender frame
    // airtime, else on a long-frame waveform (MC-DPSK, ~3691 ms/frame) the SACK fires
    // while the sender is still transmitting a trailing (failed) frame -> half-duplex
    // collision -> the sender never hears the NACK -> RTO whole-window resend ->
    // phase-locked livelock (BUG-MCDPSK-ACK-COLLISION). Default 1500 ms is correct for
    // OFDM (short frames); the Connection scales it to the frame airtime for MC-DPSK.
    void setToneBurstPartialSackDelayMs(uint32_t ms) {
        tone_burst_partial_sack_delay_ms_ = std::max(1u, ms);
    }
    uint32_t getToneBurstPartialSackDelayMs() const {
        return tone_burst_partial_sack_delay_ms_;
    }

    using ReceiveWindowAdvancedCallback = std::function<void(uint16_t base_seq, size_t window_size)>;
    void setReceiveWindowAdvancedCallback(ReceiveWindowAdvancedCallback cb) {
        on_rx_window_advanced_ = std::move(cb);
    }
    using TxFrameSubmittedCallback = std::function<void(uint16_t seq)>;
    void setTxFrameSubmittedCallback(TxFrameSubmittedCallback cb) {
        on_tx_frame_submitted_ = std::move(cb);
    }
    using TxBaseAdvancedCallback = std::function<void(uint16_t base_seq)>;
    void setTxBaseAdvancedCallback(TxBaseAdvancedCallback cb) {
        on_tx_base_advanced_ = std::move(cb);
    }
    using TxFrameFailedCallback = std::function<void(uint16_t seq)>;

    // F163 FIX-4: fired for each SACKED (receiver-confirmed) TX slot that a
    // rate/CW-change abort is about to discard — the peer HAS these bytes; the
    // file layer must not re-send them. Carries the slot's serialized frame.
    using SackedFrameDiscardedCallback = std::function<void(const Bytes& frame_data)>;
    void setSackedFrameDiscardedCallback(SackedFrameDiscardedCallback cb) {
        on_sacked_frame_discarded_ = std::move(cb);
    }
    void setTxFrameFailedCallback(TxFrameFailedCallback cb) {
        on_tx_frame_failed_ = std::move(cb);
    }
    using TurnRequestCallback = std::function<bool()>;
    void setTurnRequestCallback(TurnRequestCallback cb) {
        should_request_turn_ = std::move(cb);
    }

private:
    friend struct SelectiveRepeatARQTestAccess;

    enum class RetransmitCause : uint8_t {
        TIMEOUT,
        FAST_HOLE,
        HOLE_PROBE,
        NACK
    };

    // TX state per frame in window
    struct TXSlot {
        bool active = false;        // Slot in use
        Bytes frame_data;           // Serialized v2 frame to send/resend
        std::vector<Bytes> info_codewords; // Original variable-frame info CWs for DATA_REPAIR
        uint16_t seq = 0;           // Sequence number
        int fixed_frame_codewords = v2::kDefaultFixedFrameCodewords;
        uint32_t timeout_ms = 0;    // Time until retransmit
        // Unified stop-and-wait may have more live holes than fit in the current
        // physical group. Only identities in the group on air may run an RTO; later
        // holes stay suspended until a subsequent group actually transmits them.
        bool timeout_suspended = false;
        // A deferred timeout intent was canceled because a synchronized MODE_CHANGE now
        // owns egress. Kept separate from ordinary physical-round suspension so mode
        // resolution resumes only the slots it froze, never unrelated later holes.
        bool timeout_transition_suspended = false;
        uint64_t first_tx_ms = 0;   // ARQ monotonic clock when first sent
        bool rtt_sample_eligible = false; // Karn-safe RTT sampling guard
        int retry_count = 0;        // Number of retransmits
        // ACK rounds that came back without this frame. A retransmission timeout is not one: a
        // round that drew nothing says nothing about the frame (INV-RATE-EVIDENCE-001).
        int heard_miss_count = 0;
        bool acked = false;         // ACK received (waiting for earlier frames)
        int hole_ack_count = 0;     // Consecutive ACKs showing this frame as gap
        int fast_retx_count = 0;    // Number of fast retransmits for current hole context
        uint32_t fast_retx_cooldown_ms = 0; // Prevent ACK-repeat storms from immediate re-retransmit
        bool hole_probe_armed = false;      // Timer armed for progress-based probe retx
        uint32_t hole_probe_timer_ms = 0;   // Countdown to hole probe retx
        int hole_probe_count = 0;           // Number of hole-probe retransmits in current epoch
        uint32_t last_repair_bitmap = 0;    // Dedup repeated CW_NACK copies
        uint32_t repair_cooldown_ms = 0;
        bool repair_in_flight = false;      // Compact DATA_REPAIR is awaiting ACK/SACK result
        uint32_t repair_guard_ms = 0;       // Suppresses full-frame retx while repair can still land
    };

    // RX state per frame in receive window
    struct RXSlot {
        bool received = false;      // Frame received
        bool partial = false;       // Some CWs received, frame not yet complete
        uint16_t seq = 0;           // Sequence number
        Bytes payload;              // Received payload
        uint8_t flags = 0;          // Frame flags
        v2::FrameType type = v2::FrameType::DATA;
        uint8_t total_cw = 0;
        uint32_t cw_bitmap = 0;     // Bit i = decoded CW stored in cw_data[i]
        uint32_t partial_age_ms = 0;
        uint32_t last_cw_nack_bitmap = 0;
        uint32_t cw_nack_cooldown_ms = 0;
        std::vector<Bytes> cw_data;
    };

    // Maximum window size. Control frames already carry a 32-bit SACK bitmap;
    // keep the production cap at 16 for bounded memory/latency while allowing
    // clean OFDM audio chains to cover real soundcard buffering.
    static constexpr size_t MAX_WINDOW = 16;
    static constexpr uint32_t PARTIAL_RX_TTL_MS = 120000;

    ARQConfig config_;
    CodeRate code_rate_ = CodeRate::R1_4;  // Default R1/4, updated when connected
    int fixed_frame_codewords_ = v2::kDefaultFixedFrameCodewords;
    int fixed_frame_lifting_z_ = 27;

    // Callsigns
    std::string local_call_;
    std::string remote_call_;

    // TX state
    std::array<TXSlot, MAX_WINDOW> tx_window_;
    uint16_t tx_base_seq_ = 0;      // First unACKed sequence number
    uint16_t tx_next_seq_ = 0;      // Next sequence to assign
    size_t tx_in_flight_ = 0;       // Number of frames in flight
    size_t tx_in_flight_cap_ = 0;   // setTxInFlightCap (0 = the window)
    uint32_t tx_abort_generation_ = 0;  // txAbortGeneration

    // RX state
    std::array<RXSlot, MAX_WINDOW> rx_window_;
    uint16_t rx_base_seq_ = 0;      // Next expected sequence
    uint32_t keepalive_silent_ms_ = 0;  // ms since last SACK emit (stall detect)
    bool last_rx_more_data_ = false;
    uint8_t last_rx_flags_ = 0;
    v2::FrameType last_rx_frame_type_ = v2::FrameType::DATA;
    bool rx_final_delivered_since_sack_ = false;
    // Burst-aware ack (transport merge): while true, handleDataFrame suppresses the
    // per-frame ack decision; endGroupReceiveAndAck() emits one ack for the group.
    bool group_ack_deferred_ = false;

    // Delayed SACK for half-duplex (wait for burst to complete)
    bool sack_pending_ = false;     // SACK waiting to be sent
    uint32_t sack_timer_ms_ = 0;    // Time until SACK is sent
    // Stream-aware tail override (0 = "use sack_delay_ms for both legs").
    uint32_t sack_delay_short_ms_ = 0;
    bool sack_delay_slides_on_data_ = false;
    bool ack_batch_through_more_frag_ = false;
    bool immediate_out_of_order_sack_enabled_ = true;
    // BUG-ARQ-SEQ-COLLISION interim salvage (env ULTRA_BELOW_WINDOW_FILE_SALVAGE, read
    // once in the ctor, default OFF = byte-identical): when ON, a BELOW-window DATA frame
    // whose payload is FILE_START/FILE_DATA is handed up the normal delivery callback
    // before the (unchanged) out-of-window SACK, instead of being dropped. The file layer
    // is offset-keyed and idempotent by construction (dedup + straddle-merge), so double
    // delivery is safe there; message payloads are seq-deduped ONLY and are never salvaged.
    bool below_window_file_salvage_ = false;

    // ==================== MOVE-EPOCH (BUG-ARQ-SEQ-COLLISION structural fix) =========
    // Env ULTRA_ARQ_MOVE_EPOCH (read once in the ctor, default OFF = byte-identical;
    // SEMANTICS/WIRE-BREAKING when ON — both stations must run it in LOCKSTEP, no
    // capability negotiation in this increment). Root cure for the W16 seq collision:
    // a rate-change TX abort rewinds tx_next_seq_ to the sender's (possibly STALE,
    // under one-way ACK loss) tx_base_seq_ and re-chunks DIFFERENT file bytes under
    // seqs the receiver already retired. Two independent per-direction 2-bit (mod-4)
    // counters:
    //
    //   tx_epoch_ — MY data direction. BUMP: (a) in setCodeRate(), when the TX-abort
    //     branch fires (rewinds tx_next_seq_ — the seq-REUSE collision arm); (b) in
    //     abortPendingTx() when it drops live unacked payload (2026-07-04, Phase-2
    //     review): forward abandonment (tx_base -> tx_next) never re-uses a seq, but
    //     a mid-window regrid through the CW-change abort alone (same-rate mod change,
    //     e.g. QAM16 R3/4 -> QPSK R3/4, where setCodeRate early-returns) leaves the
    //     receiver's in-order rx_base below the abandoned seqs with no rebase-anchor —
    //     an unfillable hole, the SAME bug by the other door. Any abort dropping live
    //     payload is a seq<->payload remap = new era. The two bumps are mutually
    //     exclusive per move (whichever abort runs first empties the window, so the
    //     other's condition sees nothing live). STAMP: every DATA-frame send path ORs
    //     epochToFlags(tx_epoch_)
    //     into flags; additionally EPOCH_REBASE is stamped iff the frame is created
    //     while seq == tx_base_seq_ ("nothing un-retired below me in this era" — an
    //     invariant that stays true for the frame's whole life, since only a
    //     setCodeRate rewind could put an older seq back on air and that clears the
    //     slot + bumps the epoch). Retransmits re-send the serialized bytes, so both
    //     stamps ride along. GATE: handleAckFrame extracts the ACK's epoch echo
    //     (SACK bitmap bits 16-17, or the tone-burst payload epoch folded in by
    //     onToneBurstAck) and IGNORES any ACK whose epoch != tx_epoch_ (stale era —
    //     formed against a pre-abort grid; retiring anything on it is the W16
    //     phantom-retire). Ignoring an ACK is always protocol-safe (= ACK lost).
    //     Side cure: the late stale ACK can no longer advance tx_base past the
    //     rewound tx_next (the "below-base zombie transmissions" wart).
    //
    //   rx_epoch_ — the PEER's data direction, adopted from the wire. On a DATA frame
    //     whose epoch != rx_epoch_ (serial half-duplex channel => any change is a
    //     NEWER era; old-era frames cannot arrive after new-era ones): adopt the
    //     epoch, discard ALL buffered rx slots + pending SACK/ack-repeat state
    //     (old-era numbering is unusable; the sender's requeue re-covers those bytes
    //     on the new grid), then anchor:
    //       * frame has EPOCH_REBASE  -> rx_base_seq_ = frame.seq (exact era base —
    //         cumulative claims below it only name seqs the sender already retired,
    //         so no fabrication is possible);
    //       * frame lacks EPOCH_REBASE (era head was lost) -> enter the UNANCHORED
    //         interregnum (rx_epoch_wait_rebase_): window bookkeeping suspended, ALL
    //         acks suppressed (any cumulative ack built from the old rx_base would
    //         fabricate delivery of new-era seqs = the disease), FILE payloads are
    //         salvage-delivered (offset-idempotent), everything else dropped. The
    //         sender hears silence -> RTO -> resends its window base-first; the
    //         EPOCH_REBASE frame re-arrives and anchors us. A rebase frame that
    //         fails max_retries kills the transfer exactly as an undecodable frame
    //         does today (no NEW failure mode).
    //     NOT re-anchor-to-any-incoming-seq (the naive rule): if the first new-era
    //     frame heard is NOT the sender's base (head loss), anchoring to it makes the
    //     next cumulative ACK claim the lost head frames -> sender retires them ->
    //     their bytes become permanently unresendable — recreating the hole this fix
    //     cures. ECHO: sendSack stamps rx_epoch_ into SACK bitmap bits 16-17 (window
    //     <= 16 occupies bits 0-15; bits 24-31 must stay 0 for the decodeSackBitmap
    //     legacy-8-bit shim) / the tone-burst callback epoch argument.
    //
    // Residual (documented, accepted): mod-4 wrap — 4 TX aborts with ZERO frames
    // decoded in between return to the same epoch value (below-window frames then
    // fall back to today's salvage path); repairs/partials of a stale era are
    // dropped or frame-CRC-rejected, never merged. GROUP_ACK/GROUP_NACK (legacy
    // burst control) and NACK-type frames carry no epoch: they trigger retransmits,
    // never retirement (a stale NACK causes at worst a duplicate resend).
    bool move_epoch_enabled_ = false;
    uint8_t tx_epoch_ = 0;             // stamps outgoing DATA; gates incoming ACKs
    uint8_t rx_epoch_ = 0;             // adopted from incoming DATA; echoed in ACKs
    bool rx_epoch_wait_rebase_ = false; // unanchored interregnum (see block comment)
    uint32_t frames_since_ack_ = 0; // Frames received since last ACK sent

    // ACK repeat config (time-diversity for fading channels)
    int ack_repeat_count_ = 1;         // Total copies (1=single, 2=double, 3=triple)
    uint32_t ack_repeat_delay_ms_ = 80; // Delay between copies
    std::optional<uint32_t> ack_repeat_peer_burst_guard_ms_;
    // Tone-burst partial-SACK sliding delay (>= one sender frame airtime). Default is
    // the legacy 1500 ms (OFDM-correct); the Connection scales it up for MC-DPSK.
    uint32_t tone_burst_partial_sack_delay_ms_ = 1500;

    // Pending repeat state (queue avoids overwriting repeats during ACK bursts)
    struct AckRepeatJob {
        Bytes frame_data;
        uint16_t base_seq = 0;
        uint32_t bitmap = 0;
        uint32_t timer_ms = 0;
        int copy_index = 0;  // 2 = first repeat copy, 3 = second repeat copy
    };
    std::deque<AckRepeatJob> ack_repeat_jobs_;

    // Track cumulative ACK base progress (for critical immediate duplicate)
    bool last_sack_base_valid_ = false;
    uint16_t last_sack_base_ = 0;
    bool last_ack_signature_valid_ = false;
    uint16_t last_ack_seq_ = 0;
    uint32_t last_ack_bitmap_ = 0;
    uint32_t ack_dedup_timer_ms_ = 0;
    // §RETX-PACING §1.1 round-progress accessor state (see lastAckProgressFrames()).
    int last_ack_progress_frames_ = -1;

    // Monotonic ARQ time and adaptive RTO estimator (Karn-safe)
    uint64_t arq_time_ms_ = 0;
    bool have_rtt_estimator_ = false;
    float srtt_ms_ = 0.0f;
    float rttvar_ms_ = 0.0f;
    uint32_t adaptive_ack_timeout_ms_ = 0;
    uint32_t ack_timeout_table_[selective_repeat_arq_policy::kMaxWindow + 1] = {};
    size_t ack_timeout_table_n_ = 0;
    uint32_t last_logged_rto_ms_ = 0;  // null-control dedup (see maybeSampleRTT)
    // Measured burst timeout (setMeasuredBurstTimeouts). The egress group is the DATA frames
    // that left in one ARQ tick (a physical burst: the Connection hands a burst over at once),
    // with the airtime they were sent at; a latency sample needs the acknowledged frame to be in
    // the last group, so a constellation change between send and ACK cannot skew it.
    void noteEgress();
    bool measured_burst_rto_ = false;
    uint32_t burst_overhead_ms_ = 0;
    uint32_t frame_airtime_ms_ = 0;
    uint32_t latency_guard_ms_ = 0;
    uint32_t latency_extra_ms_ = 0;
    uint64_t egress_group_tx_ms_ = UINT64_MAX;
    uint32_t egress_group_airtime_ms_ = 0;
    uint64_t last_latency_sample_tx_ms_ = UINT64_MAX;
    bool have_post_burst_latency_ = false;
    float post_burst_latency_srtt_ms_ = 0.0f;
    float post_burst_latency_rttvar_ms_ = 0.0f;
    // Consecutive timeout rounds without an ACK in between (one per tick that resends on timeout).
    uint32_t silent_timeout_rounds_ = 0;
    uint64_t last_timeout_round_ms_ = UINT64_MAX;

    // Statistics
    ARQStats stats_;

    // Callbacks
    TransmitCallback on_transmit_;
    TransmitBatchCallback on_transmit_batch_;
    bool deferred_timeout_commit_enabled_ = false;
    ToneBurstSackCallback on_emit_tone_burst_sack_;
    bool tone_burst_sack_transport_ready_ = true;
    DataReceivedCallback on_data_received_;
    SendCompleteCallback on_send_complete_;
    ReceiveWindowAdvancedCallback on_rx_window_advanced_;
    TxFrameSubmittedCallback on_tx_frame_submitted_;
    TxBaseAdvancedCallback on_tx_base_advanced_;
    TxFrameFailedCallback on_tx_frame_failed_;
    TurnRequestCallback should_request_turn_;
    SackedFrameDiscardedCallback on_sacked_frame_discarded_;

    // Internal helpers
    bool toneBurstSackTransportReady() const {
        return static_cast<bool>(on_emit_tone_burst_sack_) &&
               tone_burst_sack_transport_ready_;
    }
    size_t seqToSlot(uint16_t seq) const;
    bool isInTXWindow(uint16_t seq) const;
    bool isInRXWindow(uint16_t seq) const;

    // MOVE-EPOCH helpers (no-ops / identity while ULTRA_ARQ_MOVE_EPOCH is OFF).
    // Stamp epoch bits (+ EPOCH_REBASE when seq == tx_base_seq_) into DATA flags.
    uint8_t stampMoveEpochFlags(uint8_t flags, uint16_t seq) const;
    // Discard all rx-window slots + pending SACK/ack-repeat state on era adoption
    // (mirrors setCodeRate's receiver-side discard).
    void discardRxStateForEpochAdoption(const char* reason);
    // Returns true when the frame was fully consumed by the move-epoch layer
    // (unanchored interregnum: salvage/drop + ack silence).
    bool handleMoveEpochOnData(const v2::DataFrame& frame);

    void transmitData(const Bytes& data);
    void transmitDataBatch(const std::vector<Bytes>& frames);
    void handleDataFrame(const v2::DataFrame& frame);
    void handlePartialFrame(const v2::PartialFrameCodewords& partial);
    void handleDataRepairFrame(const v2::DataRepairFrame& repair);
    bool handleAckFrame(const v2::ControlFrame& frame);
    void handleNackFrame(const v2::ControlFrame& frame);

    void retransmitFrame(size_t slot,
                         RetransmitCause cause,
                         std::vector<Bytes>* deferred_timeout_batch = nullptr);
    void performRetransmitFrame(size_t slot,
                                RetransmitCause cause,
                                std::vector<Bytes>* deferred_timeout_batch = nullptr);
    std::optional<size_t> matchingLiveTXSlot(const Bytes& frame_data) const;
    bool sendDataRepair(size_t slot, uint32_t missing_bitmap);
    bool suppressFullRetransmitForRepair(size_t slot, RetransmitCause cause);
    uint32_t computeRepairGuardMs(const TXSlot& slot, size_t repair_frame_codewords) const;
    void advanceTXWindow();
    void notifyTXBaseAdvanced(uint16_t base_before);
    void advanceRXWindow();
    void sendSack();
    void sendFrameNack(uint16_t seq);
    void sendCwNack(uint16_t seq, uint32_t missing_bitmap);
    void maybeSendCwNack(size_t slot_index, uint32_t missing_bitmap);
    void clearPartialRXSlot(RXSlot& slot);
    void clearTXSlotRepairState(TXSlot& slot);
    bool tryCompletePartialRXSlot(size_t slot_index);
    void maybeSampleRTT(TXSlot& slot);
    uint32_t currentAckTimeoutMs() const;
    uint32_t ackTimeoutForFrames(size_t frames) const;
    void rearmOutstandingTimeouts();
    uint32_t ackRepeatDelayForCopy(int copy_index) const;
    int ackRepeatJitterMs(uint16_t base_seq, uint32_t bitmap, int copy_index) const;

    uint32_t buildRXBitmap() const;
};

} // namespace protocol
} // namespace ultra
