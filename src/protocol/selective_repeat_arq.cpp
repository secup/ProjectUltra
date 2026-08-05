#include "selective_repeat_arq.hpp"
#include "selective_repeat_arq_policy.hpp"
#include "file_transfer.hpp"  // PayloadType — below-window FILE salvage gate only
#include "ultra/logging.hpp"
#include "ultra/phy_diagnostics.hpp"
#include <cstdlib>
#include <sstream>

namespace ultra {
namespace protocol {

namespace arq_policy = selective_repeat_arq_policy;

namespace {

uint8_t dataFrameFlags(uint8_t flags) {
    return static_cast<uint8_t>(v2::Flags::VERSION_V2 | flags);
}

const char* boolDigit(bool value) {
    return value ? "1" : "0";
}

}  // namespace

SelectiveRepeatARQ::SelectiveRepeatARQ(const ARQConfig& config)
    : config_(config)
{
    config_.window_size = arq_policy::clampWindowSize(config_.window_size, MAX_WINDOW);
    config_.ack_batch_size = arq_policy::clampAckBatchSize(
        config_.ack_batch_size, config_.window_size, MAX_WINDOW);
    adaptive_ack_timeout_ms_ = config_.ack_timeout_ms;

    // BUG-ARQ-SEQ-COLLISION interim salvage — DEFAULT-ON since 2026-07-03 evening:
    // 9/9 rig field engagements positive (W20 x9, W23 x3, W27 x14, W29 x2, W31 x5,
    // W32 x8, W35 x8, W36 x9, W37 x15 — ~60 frames rescued, every touched transfer
    // delivered byte-exact, zero anomalies), unit-tested (test_selective_repeat
    // 44/44). Salvage is receiver-side and idempotent by construction (offset-keyed
    // FILE payloads only). ULTRA_BELOW_WINDOW_FILE_SALVAGE=0 opts out.
    below_window_file_salvage_ = true;
    if (const char* e = std::getenv("ULTRA_BELOW_WINDOW_FILE_SALVAGE"); e && e[0] == '0') {
        below_window_file_salvage_ = false;
    }

    // BUG-ARQ-SEQ-COLLISION structural fix (2026-07-03): move-epoch carried on the
    // wire so a stale-era ACK can never retire re-gridded seqs and the receiver
    // never destroys a re-gridded resend. Default OFF = byte-identical;
    // SEMANTICS/WIRE-BREAKING when ON — BOTH stations must run it (lockstep, no
    // capability negotiation this increment). Full state machine: the MOVE-EPOCH
    // block comment in selective_repeat_arq.hpp. Rig-validation-pending.
    if (const char* e = std::getenv("ULTRA_ARQ_MOVE_EPOCH"); !(e && e[0] == '0')) {  // DEFAULT-ON 2026-07-05
        move_epoch_enabled_ = true;
    }
}

void SelectiveRepeatARQ::setCallsigns(const std::string& local, const std::string& remote) {
    local_call_ = sanitizeCallsign(local);
    remote_call_ = sanitizeCallsign(remote);
}

uint8_t SelectiveRepeatARQ::stampMoveEpochFlags(uint8_t flags, uint16_t seq) const {
    if (!move_epoch_enabled_) {
        return flags;  // knob OFF: bits 6-7 and EPOCH_REBASE stay 0 — byte-identical
    }
    flags = static_cast<uint8_t>(flags | v2::epochToFlags(tx_epoch_));
    // EPOCH_REBASE iff created at the window base: "no un-retired seq below me in
    // this era" — the safe rx_base re-anchor point. The invariant holds for the
    // frame's whole life: seqs below it are retired (base only passes acked/failed
    // frames) and only a setCodeRate rewind could re-use one, which clears this
    // slot and bumps the epoch. Baked into the serialized bytes, so every
    // retransmit re-carries it (RTO recovery of a lost era head).
    if (seq == tx_base_seq_) {
        flags = static_cast<uint8_t>(flags | v2::Flags::EPOCH_REBASE);
    }
    return flags;
}

void SelectiveRepeatARQ::discardRxStateForEpochAdoption(const char* reason) {
    size_t discarded = 0;
    size_t file_salvaged = 0;
    for (auto& slot : rx_window_) {
        if (slot.received || slot.partial) {
            ++discarded;
        }
        // F168 SACKED-BYTES DURABILITY: a buffered (SACKed) slot holds bytes
        // the peer may consider DELIVERED — the sender-side salvage (F163
        // FIX-4) skips re-sending SACKed FILE ranges across a rate abort.
        // Discarding the buffer WITHOUT delivering it makes that a PERMANENT
        // hole (F168: sender completed at 1.54 kbps believing 51200/51200;
        // receiver stranded at ~50%, bytes 31064-32744 gone forever, 640 s
        // wedge). FILE payloads are offset-idempotent at the file layer (the
        // unanchored-interregnum salvage uses the same contract) — deliver
        // them BEFORE the discard. TEXT stays dropped (seq-deduped only; the
        // sender resends it normally post-rebase).
        if (slot.received && !slot.payload.empty() &&
            slot.type == v2::FrameType::DATA &&
            (slot.payload[0] == static_cast<uint8_t>(PayloadType::FILE_START) ||
             slot.payload[0] == static_cast<uint8_t>(PayloadType::FILE_DATA))) {
            ++file_salvaged;
            last_rx_frame_type_ = slot.type;
            if (on_data_received_) {
                on_data_received_(slot.payload);
            }
        }
        slot.received = false;
        clearPartialRXSlot(slot);
        slot.payload.clear();
        slot.flags = 0;
        slot.type = v2::FrameType::DATA;
    }
    if (file_salvaged > 0) {
        LOG_MODEM(WARN,
                  "SR-ARQ: MOVE-EPOCH %s — salvaged %zu buffered FILE frame(s) "
                  "to the file layer before discard (SACKed bytes stay durable)",
                  reason, file_salvaged);
    }
    // Old-era ack state must not fire post-adoption (mirrors setCodeRate's
    // receiver-side discard): queued repeats hold old-era serialized SACKs, and a
    // pending delayed SACK would be rebuilt from the discarded window anyway.
    sack_pending_ = false;
    sack_timer_ms_ = 0;
    frames_since_ack_ = 0;
    ack_repeat_jobs_.clear();
    last_sack_base_valid_ = false;
    last_sack_base_ = 0;
    LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH %s — discarded %zu buffered RX slot(s) at base %u",
              reason, discarded, rx_base_seq_);
}

bool SelectiveRepeatARQ::handleMoveEpochOnData(const v2::DataFrame& frame) {
    if (!move_epoch_enabled_) {
        return false;
    }

    const uint8_t frame_epoch = v2::epochFromFlags(frame.flags);
    const bool rebase = (frame.flags & v2::Flags::EPOCH_REBASE) != 0;

    if (frame_epoch != rx_epoch_) {
        // NEW ERA. The channel is a serial half-duplex audio stream: frames cannot
        // reorder across the epoch bump, so ANY change is a newer era (mod-4
        // distance comparison would add nothing and mis-handle multi-bump jumps).
        rx_epoch_ = frame_epoch;
        discardRxStateForEpochAdoption("adoption");
        if (rebase) {
            // Exact era base: everything below frame.seq in this era is already
            // retired at the sender, so our next cumulative claim (frame.seq-1)
            // cannot fabricate anything.
            rx_base_seq_ = frame.seq;
            rx_epoch_wait_rebase_ = false;
            LOG_MODEM(WARN,
                      "SR-ARQ: MOVE-EPOCH adopted epoch %u, re-anchored rx_base to %u (rebase frame)",
                      rx_epoch_, rx_base_seq_);
            if (on_rx_window_advanced_) {
                // HARQ soft-combine hygiene: prune retained keys to the new window.
                on_rx_window_advanced_(rx_base_seq_, config_.window_size);
            }
        } else {
            // Era head lost: we cannot know the sender's rewound base, and anchoring
            // to THIS seq would make our cumulative ACK claim the lost head frames
            // (sender retires them -> permanent hole — the disease itself). Go
            // ack-silent and harvest until the EPOCH_REBASE frame arrives via the
            // sender's RTO resend (base-first).
            rx_epoch_wait_rebase_ = true;
            LOG_MODEM(WARN,
                      "SR-ARQ: MOVE-EPOCH adopted epoch %u UNANCHORED (seq=%u lacks rebase) — ack-silent until era base arrives",
                      rx_epoch_, frame.seq);
        }
    } else if (rx_epoch_wait_rebase_ && rebase) {
        // The era base arrived (RTO resend): anchor now. Slots are empty by
        // construction (nothing is stored while unanchored); discard defensively.
        discardRxStateForEpochAdoption("late rebase anchor");
        rx_base_seq_ = frame.seq;
        rx_epoch_wait_rebase_ = false;
        LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH late rebase — re-anchored rx_base to %u",
                  rx_base_seq_);
        if (on_rx_window_advanced_) {
            on_rx_window_advanced_(rx_base_seq_, config_.window_size);
        }
    }

    if (!rx_epoch_wait_rebase_) {
        return false;  // anchored: normal window processing handles the frame
    }

    // UNANCHORED interregnum: no window bookkeeping (slot keys are era-relative and
    // rx_base is still an old-era number), no acks (a cumulative ack from the old
    // rx_base would fabricate delivery of new-era seqs — the W16 phantom-retire).
    // Salvage FILE payloads: provably fresh content (new era) and offset-idempotent
    // at the file layer; TEXT stays un-acked so the sender resends it post-anchor —
    // in-order delivery, no duplicates.
    const bool file_payload =
        frame.type == v2::FrameType::DATA && !frame.payload.empty() &&
        (frame.payload[0] == static_cast<uint8_t>(PayloadType::FILE_START) ||
         frame.payload[0] == static_cast<uint8_t>(PayloadType::FILE_DATA));
    if (file_payload) {
        last_rx_frame_type_ = frame.type;
        LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH unanchored SALVAGE of FILE frame seq=%u", frame.seq);
        if (on_data_received_) {
            on_data_received_(frame.payload);
        }
    } else {
        LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH unanchored — dropped %s seq=%u (awaiting rebase)",
                  v2::frameTypeToString(frame.type), frame.seq);
    }
    return true;  // consumed: no window state touched, no SACK emitted
}

void SelectiveRepeatARQ::setCodeRate(CodeRate rate) {
    if (rate == code_rate_) {
        return;
    }

    code_rate_ = rate;

    size_t aborted_unacked = 0;
    size_t cleared_acked = 0;
    for (auto& slot : tx_window_) {
        if (!slot.active) {
            continue;
        }

        if (slot.acked) {
            cleared_acked++;
            // F163 FIX-4: the peer confirmed this frame (SACK) — salvage its
            // payload identity so the file layer skips those bytes on requeue
            // (13 receiver-confirmed frames were re-sent across F163).
            if (on_sacked_frame_discarded_ && !slot.frame_data.empty()) {
                on_sacked_frame_discarded_(slot.frame_data);
            }
        } else {
            aborted_unacked++;
        }

        slot.active = false;
        slot.acked = false;
        slot.frame_data.clear();
        slot.fixed_frame_codewords = fixed_frame_codewords_;
        slot.timeout_ms = 0;
        slot.timeout_suspended = false;
        slot.timeout_transition_suspended = false;
        slot.first_tx_ms = 0;
        slot.rtt_sample_eligible = false;
        slot.retry_count = 0;
        slot.hole_ack_count = 0;
        slot.fast_retx_count = 0;
        slot.fast_retx_cooldown_ms = 0;
        slot.hole_probe_armed = false;
        slot.hole_probe_timer_ms = 0;
        slot.hole_probe_count = 0;
        clearTXSlotRepairState(slot);
    }

    if (aborted_unacked > 0 || cleared_acked > 0 || tx_in_flight_ > 0) {
        tx_next_seq_ = tx_base_seq_;
        tx_in_flight_ = 0;
        last_ack_signature_valid_ = false;
        last_ack_seq_ = 0;
        last_ack_bitmap_ = 0;
        ack_dedup_timer_ms_ = 0;

        // MOVE-EPOCH bump (ULTRA_ARQ_MOVE_EPOCH): this rewind re-uses seq numbers
        // for DIFFERENT content — the BUG-ARQ-SEQ-COLLISION precondition. Enter a
        // new era so (a) ACKs formed against the old grid are ignored and (b) the
        // receiver recognizes the re-gridded resends as fresh content. The first
        // frame sent after the rewind (seq == tx_base_seq_) carries EPOCH_REBASE.
        if (move_epoch_enabled_) {
            tx_epoch_ = static_cast<uint8_t>((tx_epoch_ + 1) & 0x3);
            LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH bumped to %u on rate-change TX abort",
                      tx_epoch_);
        }

        LOG_MODEM(WARN,
                  "SR-ARQ: Code rate changed, aborted %zu unACKed in-flight TX slots "
                  "(cleared %zu SACKed); rewound TX seq to %u",
                  aborted_unacked, cleared_acked, tx_next_seq_);
    }

    size_t discarded_rx = 0;
    size_t rate_change_file_salvaged = 0;
    for (auto& slot : rx_window_) {
        if (!slot.received && !slot.partial) {
            continue;
        }
        // F168 SACKED-BYTES DURABILITY — same salvage as
        // discardRxStateForEpochAdoption (see that comment): buffered FILE
        // bytes may be sender-retired; deliver before discarding.
        if (slot.received && !slot.payload.empty() &&
            slot.type == v2::FrameType::DATA &&
            (slot.payload[0] == static_cast<uint8_t>(PayloadType::FILE_START) ||
             slot.payload[0] == static_cast<uint8_t>(PayloadType::FILE_DATA))) {
            ++rate_change_file_salvaged;  // F181: this site salvaged SILENTLY
            last_rx_frame_type_ = slot.type;
            if (on_data_received_) {
                on_data_received_(slot.payload);
            }
        }
        slot.received = false;
        clearPartialRXSlot(slot);
        slot.payload.clear();
        slot.flags = 0;
        slot.type = v2::FrameType::DATA;
        discarded_rx++;
    }
    if (rate_change_file_salvaged > 0) {
        LOG_MODEM(WARN,
                  "SR-ARQ: rate-change RX discard — salvaged %zu buffered FILE "
                  "frame(s) to the file layer (F181 forensic parity)",
                  rate_change_file_salvaged);
    }

    if (discarded_rx > 0) {
        sack_pending_ = false;
        sack_timer_ms_ = 0;
        frames_since_ack_ = 0;
        ack_repeat_jobs_.clear();
        last_sack_base_valid_ = false;
        last_sack_base_ = 0;

        LOG_MODEM(WARN,
                  "SR-ARQ: Code rate changed, discarded %zu buffered RX slots at seq base %u",
                  discarded_rx, rx_base_seq_);
    }
}

void SelectiveRepeatARQ::setFixedFrameGeometry(int cw_count, int lifting_z) {
    cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    lifting_z = (lifting_z == 81) ? 81 : 27;
    if (cw_count == fixed_frame_codewords_ &&
        lifting_z == fixed_frame_lifting_z_) {
        return;
    }

    fixed_frame_codewords_ = cw_count;
    fixed_frame_lifting_z_ = lifting_z;
    abortPendingTx();
    LOG_MODEM(INFO, "SR-ARQ: Fixed frame geometry set to cw=%d z=%d",
              fixed_frame_codewords_, fixed_frame_lifting_z_);
}

void SelectiveRepeatARQ::setFixedFrameCodewords(int cw_count) {
    setFixedFrameGeometry(cw_count, fixed_frame_lifting_z_);
}

void SelectiveRepeatARQ::setFixedFrameLiftingZ(int lifting_z) {
    setFixedFrameGeometry(fixed_frame_codewords_, lifting_z);
}

bool SelectiveRepeatARQ::sendData(const Bytes& data) {
    return sendDataWithFlags(data, v2::Flags::NONE);
}

bool SelectiveRepeatARQ::sendData(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return sendData(data);
}

bool SelectiveRepeatARQ::sendDataWithFlags(const Bytes& data, uint8_t flags) {
    return sendDataWithTypeAndFlags(data, v2::FrameType::DATA, flags);
}

bool SelectiveRepeatARQ::sendDataWithTypeAndFlags(const Bytes& data,
                                                  v2::FrameType frame_type,
                                                  uint8_t flags) {
    if (!isReadyToSend()) {
        LOG_MODEM(WARN, "SR-ARQ: Window full, cannot send");
        return false;
    }

    if (local_call_.empty() || remote_call_.empty()) {
        LOG_MODEM(ERROR, "SR-ARQ: Callsigns not set");
        return false;
    }

    const uint16_t seq = tx_next_seq_;
    size_t slot = seqToSlot(seq);

    auto frame = v2::DataFrame::makeData(local_call_, remote_call_, seq, data, code_rate_);
    frame.type = frame_type;
    frame.flags = stampMoveEpochFlags(dataFrameFlags(flags), seq);

    tx_window_[slot].active = true;
    tx_window_[slot].frame_data = frame.serialize();
    tx_window_[slot].info_codewords =
        v2::splitIntoCodewords(tx_window_[slot].frame_data, code_rate_);
    tx_window_[slot].seq = seq;
    tx_window_[slot].fixed_frame_codewords = fixed_frame_codewords_;
    tx_window_[slot].timeout_ms = ackTimeoutForFrames(tx_in_flight_ + 1);
    tx_window_[slot].timeout_suspended = false;
    tx_window_[slot].timeout_transition_suspended = false;
    tx_window_[slot].first_tx_ms = arq_time_ms_;
    tx_window_[slot].rtt_sample_eligible = true;
    tx_window_[slot].retry_count = 0;
    tx_window_[slot].acked = false;
    tx_window_[slot].hole_ack_count = 0;
    tx_window_[slot].fast_retx_count = 0;
    tx_window_[slot].fast_retx_cooldown_ms = 0;
    tx_window_[slot].hole_probe_armed = false;
    tx_window_[slot].hole_probe_timer_ms = 0;
    tx_window_[slot].hole_probe_count = 0;
    tx_window_[slot].last_repair_bitmap = 0;
    tx_window_[slot].repair_cooldown_ms = 0;
    tx_window_[slot].repair_in_flight = false;
    tx_window_[slot].repair_guard_ms = 0;

    // Publish TX state before invoking the callback. Unit tests and future
    // low-latency transports may synchronously deliver the DATA and its ACK
    // before transmitData() returns.
    stats_.frames_sent++;
    tx_next_seq_ = (tx_next_seq_ + 1) & 0xFFFF;
    tx_in_flight_++;
    rearmOutstandingTimeouts();

    LOG_MODEM(DEBUG, "SR-ARQ: Sent %s seq=%d slot=%zu, window=[%d,%d)",
              v2::frameTypeToString(frame_type), seq, slot, tx_base_seq_, tx_next_seq_);
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_data_tx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " seq=" << seq
            << " slot=" << slot
            << " frame_type=" << v2::frameTypeToString(frame_type)
            << " fixed=0"
            << " payload_bytes=" << data.size()
            << " frame_bytes=" << tx_window_[slot].frame_data.size()
            << " in_flight=" << tx_in_flight_
            << " window=" << config_.window_size
            << " timeout_ms=" << tx_window_[slot].timeout_ms;
        ultra::phyDiagLine(oss.str());
    }

    if (on_tx_frame_submitted_) {
        on_tx_frame_submitted_(seq);
    }
    transmitData(tx_window_[slot].frame_data);

    return true;
}

bool SelectiveRepeatARQ::sendFixedDataWithFlags(const Bytes& data, uint8_t flags) {
    return sendFixedDataWithTypeAndFlags(data, v2::FrameType::DATA, flags);
}

bool SelectiveRepeatARQ::sendFixedDataWithTypeAndFlags(const Bytes& data,
                                                       v2::FrameType frame_type,
                                                       uint8_t flags) {
    if (!isReadyToSend()) {
        LOG_MODEM(WARN, "SR-ARQ: Window full, cannot send fixed frame");
        return false;
    }

    if (local_call_.empty() || remote_call_.empty()) {
        LOG_MODEM(ERROR, "SR-ARQ: Callsigns not set");
        return false;
    }

    const size_t capacity = fixed_frame_lifting_z_ == 81
        ? v2::getFixedFramePayloadCapacityZ(
              code_rate_, fixed_frame_codewords_, 81)
        : v2::getFixedFramePayloadCapacity(
              code_rate_, fixed_frame_codewords_);
    if (data.size() > capacity) {
        // makeFixedDataFrame() is a general helper with legacy truncation
        // semantics. Production ARQ must never inherit that behavior: accepting
        // an old-geometry chunk after a rate/CW shrink would report delivery for
        // bytes that were never put on wire.
        LOG_MODEM(ERROR,
                  "SR-ARQ: Refusing oversized fixed payload (%zu > %zu bytes, %s cw=%d z=%d)",
                  data.size(), capacity, codeRateToString(code_rate_),
                  fixed_frame_codewords_, fixed_frame_lifting_z_);
        return false;
    }

    const uint16_t seq = tx_next_seq_;
    size_t slot = seqToSlot(seq);

    auto frame = v2::makeFixedDataFrame(local_call_, remote_call_, seq, data,
                                        code_rate_, fixed_frame_codewords_,
                                        fixed_frame_lifting_z_);
    frame.type = frame_type;
    frame.flags = stampMoveEpochFlags(dataFrameFlags(flags), seq);

    tx_window_[slot].active = true;
    tx_window_[slot].frame_data = frame.serialize();
    tx_window_[slot].info_codewords.clear();
    tx_window_[slot].seq = seq;
    tx_window_[slot].fixed_frame_codewords = fixed_frame_codewords_;
    tx_window_[slot].timeout_ms = ackTimeoutForFrames(tx_in_flight_ + 1);
    tx_window_[slot].timeout_suspended = false;
    tx_window_[slot].timeout_transition_suspended = false;
    tx_window_[slot].first_tx_ms = arq_time_ms_;
    tx_window_[slot].rtt_sample_eligible = true;
    tx_window_[slot].retry_count = 0;
    tx_window_[slot].acked = false;
    tx_window_[slot].hole_ack_count = 0;
    tx_window_[slot].fast_retx_count = 0;
    tx_window_[slot].fast_retx_cooldown_ms = 0;
    tx_window_[slot].hole_probe_armed = false;
    tx_window_[slot].hole_probe_timer_ms = 0;
    tx_window_[slot].hole_probe_count = 0;
    tx_window_[slot].last_repair_bitmap = 0;
    tx_window_[slot].repair_cooldown_ms = 0;
    tx_window_[slot].repair_in_flight = false;
    tx_window_[slot].repair_guard_ms = 0;

    stats_.frames_sent++;
    tx_next_seq_ = (tx_next_seq_ + 1) & 0xFFFF;
    tx_in_flight_++;
    rearmOutstandingTimeouts();

    LOG_MODEM(DEBUG, "SR-ARQ: Sent fixed %s seq=%d slot=%zu cw=%d z=%d, window=[%d,%d)",
              v2::frameTypeToString(frame_type), seq, slot, fixed_frame_codewords_,
              fixed_frame_lifting_z_,
              tx_base_seq_, tx_next_seq_);
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_data_tx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " seq=" << seq
            << " slot=" << slot
            << " frame_type=" << v2::frameTypeToString(frame_type)
            << " fixed=1"
            << " payload_bytes=" << data.size()
            << " frame_bytes=" << tx_window_[slot].frame_data.size()
            << " cw=" << fixed_frame_codewords_
            << " z=" << fixed_frame_lifting_z_
            << " in_flight=" << tx_in_flight_
            << " window=" << config_.window_size
            << " timeout_ms=" << tx_window_[slot].timeout_ms;
        ultra::phyDiagLine(oss.str());
    }

    if (on_tx_frame_submitted_) {
        on_tx_frame_submitted_(seq);
    }
    transmitData(tx_window_[slot].frame_data);
    return true;
}

bool SelectiveRepeatARQ::sendVariableDataWithFlags(const Bytes& data, uint8_t flags) {
    if (!isReadyToSend()) {
        LOG_MODEM(WARN, "SR-ARQ: Window full, cannot send variable frame");
        return false;
    }

    if (local_call_.empty() || remote_call_.empty()) {
        LOG_MODEM(ERROR, "SR-ARQ: Callsigns not set");
        return false;
    }

    const uint16_t seq = tx_next_seq_;
    size_t slot = seqToSlot(seq);

    auto frame = v2::DataFrame::makeData(local_call_, remote_call_, seq, data, code_rate_);
    frame.flags = stampMoveEpochFlags(dataFrameFlags(flags), seq);

    tx_window_[slot].active = true;
    tx_window_[slot].frame_data = frame.serialize();
    tx_window_[slot].info_codewords =
        v2::splitIntoCodewords(tx_window_[slot].frame_data, code_rate_);
    tx_window_[slot].seq = seq;
    tx_window_[slot].timeout_ms = ackTimeoutForFrames(tx_in_flight_ + 1);
    tx_window_[slot].timeout_suspended = false;
    tx_window_[slot].timeout_transition_suspended = false;
    tx_window_[slot].first_tx_ms = arq_time_ms_;
    tx_window_[slot].rtt_sample_eligible = true;
    tx_window_[slot].retry_count = 0;
    tx_window_[slot].acked = false;
    tx_window_[slot].hole_ack_count = 0;
    tx_window_[slot].fast_retx_count = 0;
    tx_window_[slot].fast_retx_cooldown_ms = 0;
    tx_window_[slot].hole_probe_armed = false;
    tx_window_[slot].hole_probe_timer_ms = 0;
    tx_window_[slot].hole_probe_count = 0;
    tx_window_[slot].last_repair_bitmap = 0;
    tx_window_[slot].repair_cooldown_ms = 0;
    tx_window_[slot].repair_in_flight = false;
    tx_window_[slot].repair_guard_ms = 0;

    stats_.frames_sent++;
    tx_next_seq_ = (tx_next_seq_ + 1) & 0xFFFF;
    tx_in_flight_++;
    rearmOutstandingTimeouts();

    LOG_MODEM(INFO, "SR-ARQ: Sent variable DATA seq=%d slot=%zu total_cw=%d",
              seq, slot, static_cast<int>(frame.total_cw));
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_data_tx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " seq=" << seq
            << " slot=" << slot
            << " frame_type=DATA"
            << " fixed=0"
            << " variable=1"
            << " payload_bytes=" << data.size()
            << " frame_bytes=" << tx_window_[slot].frame_data.size()
            << " total_cw=" << static_cast<int>(frame.total_cw)
            << " in_flight=" << tx_in_flight_
            << " window=" << config_.window_size
            << " timeout_ms=" << tx_window_[slot].timeout_ms;
        ultra::phyDiagLine(oss.str());
    }

    if (on_tx_frame_submitted_) {
        on_tx_frame_submitted_(seq);
    }
    transmitData(tx_window_[slot].frame_data);
    return true;
}

bool SelectiveRepeatARQ::isReadyToSend() const {
    return getAvailableSlots() > 0;
}

size_t SelectiveRepeatARQ::getAvailableSlots() const {
    size_t window = config_.window_size;
    return (tx_in_flight_ < window) ? (window - tx_in_flight_) : 0;
}

size_t SelectiveRepeatARQ::getTxInFlightBytes() const {
    size_t bytes = 0;
    for (const auto& slot : tx_window_) {
        if (!slot.active || slot.acked) {
            continue;
        }

        auto frame = v2::DataFrame::deserialize(slot.frame_data);
        if (frame) {
            bytes += frame->payload.size();
        }
    }
    return bytes;
}

int SelectiveRepeatARQ::maxInFlightRetryCount() const {
    int max_retry = 0;
    for (const auto& slot : tx_window_) {
        if (slot.active && !slot.acked && slot.retry_count > max_retry) {
            max_retry = slot.retry_count;
        }
    }
    return max_retry;
}

void SelectiveRepeatARQ::onFrameReceived(const Bytes& frame_data) {
    if (frame_data.size() < 2) {
        return;
    }

    // Drop gates below log at WARN (were TRACE/silent): on a healthy link they fire
    // ~never, and when they DO fire they eat an LDPC-clean frame invisibly — F144
    // lost the last frame of a 5/5 group with zero log evidence at any gate.
    uint16_t magic = (static_cast<uint16_t>(frame_data[0]) << 8) | frame_data[1];
    if (magic != v2::MAGIC_V2) {
        LOG_MODEM(WARN, "SR-ARQ: DROP frame with wrong magic 0x%04X (len=%zu)", magic,
                  frame_data.size());
        return;
    }

    auto header = v2::parseHeader(frame_data);
    if (!header.valid) {
        LOG_MODEM(WARN, "SR-ARQ: DROP frame with invalid header (len=%zu)", frame_data.size());
        return;
    }

    uint32_t our_hash = v2::hashCallsign(local_call_);
    if (header.dst_hash != our_hash && header.dst_hash != 0xFFFFFF) {
        LOG_MODEM(WARN, "SR-ARQ: DROP %s seq=%u for different station (dst=%06X us=%06X)",
                  v2::frameTypeToString(header.type), header.seq, header.dst_hash, our_hash);
        return;
    }

    LOG_MODEM(DEBUG, "SR-ARQ: Received %s seq=%d",
              v2::frameTypeToString(header.type), header.seq);

    if (header.is_control) {
        auto ctrl = v2::ControlFrame::deserialize(frame_data);
        if (ctrl) {
            switch (ctrl->type) {
                case v2::FrameType::ACK:
                    handleAckFrame(*ctrl);
                    break;
                case v2::FrameType::NACK:
                    handleNackFrame(*ctrl);
                    break;
                default:
                    break;
            }
        }
    } else {
        if (header.type == v2::FrameType::DATA_REPAIR) {
            auto repair = v2::DataRepairFrame::deserialize(frame_data);
            if (repair) {
                handleDataRepairFrame(*repair);
            } else {
                LOG_MODEM(WARN, "SR-ARQ: DROP DATA_REPAIR seq=%u — deserialize failed (len=%zu)",
                          header.seq, frame_data.size());
            }
        } else {
            auto data_frame = v2::DataFrame::deserialize(frame_data);
            if (data_frame) {
                handleDataFrame(*data_frame);
            } else {
                LOG_MODEM(WARN, "SR-ARQ: DROP %s seq=%u — deserialize failed (len=%zu)",
                          v2::frameTypeToString(header.type), header.seq, frame_data.size());
            }
        }
    }
}

void SelectiveRepeatARQ::onPartialFrame(const v2::PartialFrameCodewords& partial) {
    handlePartialFrame(partial);
}

void SelectiveRepeatARQ::handleDataFrame(const v2::DataFrame& frame) {
    last_rx_flags_ = frame.flags;
    last_rx_more_data_ = (frame.flags & v2::Flags::MORE_FRAG) != 0;

    // Capture the just-arrived frame's MORE_FRAG locally — the member
    // last_rx_more_data_ above will be overwritten by advanceRXWindow()
    // when it delivers buffered frames (line 552 area). The timer-arm
    // logic below needs the value of THIS frame, not whichever frame
    // advanceRXWindow happened to deliver last.
    const bool frame_more_frag = (frame.flags & v2::Flags::MORE_FRAG) != 0;
    const bool frame_final = (frame.flags & v2::Flags::FINAL) != 0;

    uint16_t seq = frame.seq;

    // MOVE-EPOCH (ULTRA_ARQ_MOVE_EPOCH, no-op when OFF): adopt a newer era /
    // re-anchor on its EPOCH_REBASE base / consume frames during the unanchored
    // interregnum. Runs BEFORE window classification so a re-gridded below-window
    // resend is re-based into the window instead of dying at the seq-keyed dedup.
    if (handleMoveEpochOnData(frame)) {
        return;
    }

    if (isInRXWindow(seq)) {
        uint16_t expected_seq = rx_base_seq_;
        size_t slot = seqToSlot(seq);
        bool new_frame = false;
        const uint16_t rx_base_before_frame = rx_base_seq_;
        bool out_of_order = false;

        if (!rx_window_[slot].received) {
            rx_window_[slot].received = true;
            clearPartialRXSlot(rx_window_[slot]);
            rx_window_[slot].seq = seq;
            rx_window_[slot].payload = frame.payload;
            rx_window_[slot].flags = frame.flags;
            rx_window_[slot].type = frame.type;
            stats_.frames_received++;
            new_frame = true;

            LOG_MODEM(DEBUG, "SR-ARQ: DATA seq=%d stored in slot %zu", seq, slot);

            if (seq == expected_seq) {
                advanceRXWindow();
            } else {
                out_of_order = true;
                stats_.out_of_order++;
                LOG_MODEM(DEBUG, "SR-ARQ: Out-of-order seq=%d (expected %d)",
                          seq, expected_seq);
            }
        } else {
            LOG_MODEM(DEBUG, "SR-ARQ: Duplicate DATA seq=%d", seq);
        }
        // How far the receive window jumped forward on this frame: 1 for an ordinary
        // in-order frame, >1 when an in-order frame filled a hole and caught up to
        // already-buffered out-of-order frames (a repair completing).
        const int rx_base_advance =
            static_cast<int>(static_cast<uint16_t>(rx_base_seq_ - rx_base_before_frame));

        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_data_rx"
                << " local=" << local_call_
                << " remote=" << remote_call_
                << " seq=" << seq
                << " expected=" << expected_seq
                << " slot=" << slot
                << " frame_type=" << v2::frameTypeToString(frame.type)
                << " new=" << boolDigit(new_frame)
                << " duplicate=" << boolDigit(!new_frame)
                << " out_of_order=" << boolDigit(out_of_order)
                << " in_window=1"
                << " more_frag=" << boolDigit(frame_more_frag)
                << " final=" << boolDigit(frame_final)
                << " rx_base=" << rx_base_seq_
                << " frames_since_ack=" << frames_since_ack_;
            ultra::phyDiagLine(oss.str());
        }

        // ACK strategy for burst traffic:
        // - Immediate ACK on hole detection (out-of-order) — safety valve, MUST
        //   stay first in the condition.
        // - While MORE_FRAG is set, threshold ACKs stay on the delayed path so
        //   a receiver does not transmit a control frame into the sender's
        //   still-arriving physical burst. MC-DPSK continuous-burst mode can
        //   opt out because decoded frames share one physical preamble and
        //   arrive only as the sample cursor reaches them.
        // - At message/stream tail, threshold ACKs fire immediately; otherwise
        //   only an explicit FINAL marker can use the short delayed timer. A
        //   plain MORE_FRAG=0 boundary may be just one message inside a still
        //   arriving physical burst.
        if (new_frame) {
            frames_since_ack_++;
        }

        const uint32_t batch_threshold = arq_policy::effectiveAckBatchThreshold(
            config_.ack_batch_size, config_.window_size);
        const bool batch_threshold_reached = frames_since_ack_ >= batch_threshold;
        const bool batch_ack_allowed = !frame_more_frag || ack_batch_through_more_frag_;
        const bool final_gap_needs_fast_feedback =
            arq_policy::shouldSendImmediateFrameNackForGap(
                out_of_order, frame_more_frag, frame_final);
        if (final_gap_needs_fast_feedback) {
            // The explicit frame NACK predates the unified tone-SACK transport.  On
            // the legacy control-frame path it is still the fast tail-gap repair: the
            // sender receives the NACK and retransmits before the long RTO.
            //
            // On the tone-SACK path the cumulative base + bitmap already identifies
            // this exact hole and the Connection refills the next stop-and-wait turn
            // with [holes]+[new].  Emitting the legacy NACK as well queues a second,
            // long OFDM control transmission behind the tone ACK.  The sender hears
            // the short tone first and starts its repair while this receiver is still
            // keyed on the OFDM NACK, destroying the repair preamble.  One receive
            // event must therefore select exactly one feedback plane.
            if (toneBurstSackTransportReady()) {
                LOG_MODEM(INFO,
                          "SR-ARQ: Tail gap seq=%d carried by tone SACK only "
                          "(legacy frame NACK suppressed)",
                          expected_seq);
            } else {
                sendFrameNack(expected_seq);
            }
        }

        const bool out_of_order_sack_allowed =
            out_of_order &&
            (immediate_out_of_order_sack_enabled_ || !frame_more_frag || frame_final);

        // HALF-DUPLEX tone-burst ack: each ack is a full keyup + T/R turnaround, so we
        // must NOT ack per frame (the "3 bursts in a row" the operator saw). Coalesce to
        // ONE tone-burst per received burst — emit only at the message tail (no more
        // fragments / FINAL), or when an in-order frame fills a hole and the window jumps
        // forward (a repair completed). Mid-burst in-order frames and standalone
        // out-of-order frames are silent; the bitmap is a cumulative snapshot so one
        // burst conveys the whole window. A dropped TAIL is recovered by the sender's
        // RTO (it resends, the tail re-arrives, and that tail-ack fires) — so we
        // deliberately do NOT arm the legacy delayed-SACK timer here, which would fire
        // mid-burst between the widely-spaced physical frames.
        bool immediate_ack;
        bool arm_delayed_timer;
        if (toneBurstSackTransportReady()) {
            // One tone-burst per received WINDOW/turn: the message tail, an in-order
            // frame that fills a hole, OR a full window-worth of frames since the last
            // ack — so a MULTI-WINDOW transfer (a file) acks each window instead of
            // stalling waiting for a tail that is many windows away. No per-out-of-order
            // acks (half-duplex: one keyup per turn).
            const bool tail = !frame_more_frag || frame_final;
            const bool hole_filled = new_frame && !out_of_order && rx_base_advance > 1;
            const bool window_worth =
                new_frame && frames_since_ack_ >= config_.window_size;
            // Burst-aware path: while feeding a decoded group, SUPPRESS the per-frame
            // ack entirely — the caller emits exactly one ack at the group boundary
            // (endGroupReceiveAndAck), which is the real "the burst ended" signal a
            // sub-window burst can't convey through frames_since_ack_.
            immediate_ack = !group_ack_deferred_ && new_frame &&
                            (tail || hole_filled || window_worth);
            // SELECTIVE REPEAT on a PARTIAL burst: an out-of-order new frame means we now
            // hold frames the sender thinks are missing. On bi=0 (no cross-frame
            // interleave) a fade can drop frame N while N+1/N+2 decode independently — so
            // the surviving frames arrive out-of-order past the hole. Without telling the
            // sender, it times out and resends the WHOLE burst (re-sending frames we
            // already have). Arm a short, sliding delayed SACK so ONE tone-burst
            // (cumulative base + out-of-order bitmap) goes back after the burst settles,
            // and the sender resends ONLY the hole. Suppressed inside an explicit group
            // (endGroupReceiveAndAck carries it) and when an immediate ack already fired.
            arm_delayed_timer =
                !group_ack_deferred_ && new_frame && out_of_order && !immediate_ack;
        } else {
            immediate_ack =
                out_of_order_sack_allowed || (batch_threshold_reached && batch_ack_allowed);
            arm_delayed_timer = new_frame;
        }
        if (immediate_ack) {
            // Bump the trigger-reason counter BEFORE sendSack — out_of_order
            // takes priority because it's the immediate safety valve. Each
            // SACK send increments exactly one trigger counter.
            if (out_of_order_sack_allowed) {
                stats_.sack_trigger_out_of_order++;
            } else {
                stats_.sack_trigger_threshold++;
            }
            sendSack();
            sack_pending_ = false;
            sack_timer_ms_ = 0;
            frames_since_ack_ = 0;
        } else if (arm_delayed_timer) {
            sack_pending_ = true;
            if (toneBurstSackTransportReady()) {
                // Tone-burst partial-burst SACK: SLIDING delay (re-armed on each
                // out-of-order frame, so it fires ~this long after the LAST hole frame —
                // coalescing the burst's holes into ONE tone-burst). Kept well under the
                // sender's retransmit timeout so the selective SACK reaches it before it
                // would blindly resend the whole burst. MUST exceed one sender frame
                // airtime so it clears a trailing (failed) frame still on air — else
                // half-duplex collision livelock on long-frame MC-DPSK
                // (BUG-MCDPSK-ACK-COLLISION). Default 1500 ms (OFDM); the Connection
                // scales it to the MC-DPSK frame airtime via setToneBurstPartialSackDelayMs.
                sack_timer_ms_ = tone_burst_partial_sack_delay_ms_;
            } else if (sack_delay_slides_on_data_) {
                // OFDM physical bursts decode frames in cadence. Re-arming on
                // each decoded frame turns the SACK timer into a burst-tail
                // quiet detector while the delay itself still comes from the
                // selected waveform's DATA/ACK airtime.
                sack_timer_ms_ = arq_policy::sackDelayForFrame(
                    config_.sack_delay_ms, sack_delay_short_ms_, frame_final);
            } else {
                // Legacy stream-aware timer: regular frames get the long
                // physical burst delay; explicit FINAL frames can collapse to
                // the short tail delay. Sentinel sack_delay_short_ms_ = 0
                // preserves legacy long-delay behavior for every frame.
                sack_timer_ms_ = arq_policy::sackTimerForFrame(
                    sack_timer_ms_, config_.sack_delay_ms, sack_delay_short_ms_,
                    frame_final);
            }
        }

    } else {
        // BUG-ARQ-SEQ-COLLISION interim salvage (ULTRA_BELOW_WINDOW_FILE_SALVAGE=1,
        // default OFF): a rate-change abort under ONE-WAY ack loss rewinds the sender to
        // its STALE tx_base and re-chunks DIFFERENT file bytes on the new grid under seq
        // numbers this receiver already retired (rx_base ran ahead). Those frames arrive
        // here as below-window "dupes" and die at the seq-keyed dedup BEFORE the
        // offset-idempotent file layer — whose straddle-merge (file_transfer.cpp,
        // processFileData) was built exactly for the regrid-resend case — can see them
        // (rig W16: 9×(456−384)=648 bytes permanently unresendable, receiver stranded).
        // With the knob ON, hand FILE_START/FILE_DATA payloads up the SAME delivery
        // callback advanceRXWindow uses, then fall through to the UNCHANGED out-of-window
        // SACK. NEVER salvage other payload types: messages are seq-deduped only, so
        // re-delivery would duplicate them. Half-space test = strictly-behind-base only
        // (below-window), never far-future.
        if (below_window_file_salvage_) {
            const uint16_t behind = static_cast<uint16_t>(rx_base_seq_ - seq);
            const bool below_window = behind != 0 && behind < 0x8000;
            const bool file_payload =
                frame.type == v2::FrameType::DATA && !frame.payload.empty() &&
                (frame.payload[0] == static_cast<uint8_t>(PayloadType::FILE_START) ||
                 frame.payload[0] == static_cast<uint8_t>(PayloadType::FILE_DATA));
            if (below_window && file_payload) {
                LOG_MODEM(WARN, "SR-ARQ: SALVAGE below-window FILE frame seq=%u", seq);
                // Mirror advanceRXWindow's delivery contract: the Connection reads
                // lastRxFlags()/lastRxHadMoreData()/lastRxFrameType() from the callback.
                // Flags/more_data were already latched from THIS frame at function entry;
                // latch the type too (it is otherwise only set on in-order delivery).
                last_rx_frame_type_ = frame.type;
                if (on_data_received_) {
                    on_data_received_(frame.payload);
                }
            }
        }
        LOG_MODEM(WARN, "SR-ARQ: DATA seq=%d outside window [%d, %d)",
                  seq, rx_base_seq_, (rx_base_seq_ + config_.window_size) & 0xFFFF);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_data_rx"
                << " local=" << local_call_
                << " remote=" << remote_call_
                << " seq=" << seq
                << " frame_type=" << v2::frameTypeToString(frame.type)
                << " in_window=0"
                << " rx_base=" << rx_base_seq_
                << " window=" << config_.window_size;
            ultra::phyDiagLine(oss.str());
        }
        // Out-of-window: normally send a SACK immediately to help the sender recover.
        // BUT while feeding a decoded BURST GROUP (group_ack_deferred_), suppress it —
        // a single group commonly contains frames that became duplicates mid-group (an
        // earlier frame filled a hole and advanced rx_base past them: the "delivered 3,4,5
        // then seq=4,5 outside window" cascade). Each such frame would otherwise emit its
        // own tone-burst → 3 acks for one group (the multi-ack regression on fading). The
        // single endGroupReceiveAndAck() at the group boundary carries the correct
        // cumulative state, so defer to it: ONE tone-burst per group.
        stats_.sack_trigger_out_of_window++;
        if (!group_ack_deferred_) {
            sendSack();
            sack_pending_ = false;
            sack_timer_ms_ = 0;
            frames_since_ack_ = 0;
        }
    }
}

void SelectiveRepeatARQ::handlePartialFrame(const v2::PartialFrameCodewords& partial) {
    if (!partial.valid()) {
        return;
    }
    // MOVE-EPOCH (no-op when OFF): never merge CWs across eras (a seq re-used on a
    // different grid would mix old/new-era codewords) and stay silent while
    // unanchored. DATA_REPAIR-sourced partials are EXEMPT from the epoch check: the
    // repair header's flags are synthesized (epoch-blind, handleDataRepairFrame) and
    // a cross-era merge is frame-CRC-rejected in tryCompletePartialRXSlot anyway;
    // adoption itself only happens on COMPLETE DATA frames.
    if (move_epoch_enabled_) {
        if (rx_epoch_wait_rebase_) {
            LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH unanchored — dropped partial seq=%d",
                      partial.seq);
            return;
        }
        if (!partial.from_repair && v2::epochFromFlags(partial.flags) != rx_epoch_) {
            LOG_MODEM(WARN,
                      "SR-ARQ: MOVE-EPOCH dropped stale/foreign-era partial seq=%d (epoch=%u rx_epoch=%u)",
                      partial.seq, v2::epochFromFlags(partial.flags), rx_epoch_);
            return;
        }
    }
    if (!isInRXWindow(partial.seq)) {
        LOG_MODEM(WARN, "SR-ARQ: Partial DATA seq=%d outside window [%d, %d)",
                  partial.seq, rx_base_seq_, (rx_base_seq_ + config_.window_size) & 0xFFFF);
        stats_.sack_trigger_out_of_window++;
        sendSack();
        sack_pending_ = false;
        sack_timer_ms_ = 0;
        frames_since_ack_ = 0;
        return;
    }

    size_t slot_index = seqToSlot(partial.seq);
    RXSlot& slot = rx_window_[slot_index];
    if (slot.received) {
        LOG_MODEM(DEBUG, "SR-ARQ: Partial DATA seq=%d ignored; frame already complete", partial.seq);
        sendSack();
        return;
    }

    if (!slot.partial || slot.seq != partial.seq) {
        if (partial.from_repair && (partial.decoded_bitmap & 0x1u) == 0) {
            LOG_MODEM(DEBUG,
                      "SR-ARQ: DATA_REPAIR seq=%d ignored; no partial slot and CW0 absent",
                      partial.seq);
            return;
        }
        clearPartialRXSlot(slot);
        slot.partial = true;
        slot.seq = partial.seq;
        slot.flags = partial.flags;
        slot.type = partial.type;
        slot.total_cw = partial.total_cw;
        slot.cw_data.assign(partial.total_cw, Bytes{});
        slot.partial_age_ms = 0;
    }

    bool merged = false;
    const uint32_t expected = partial.expectedBitmap();
    for (uint8_t cw = 0; cw < partial.total_cw && cw < 32; ++cw) {
        const uint32_t bit = 1u << cw;
        if ((partial.decoded_bitmap & bit) == 0) {
            continue;
        }
        if (cw >= partial.data.size() || partial.data[cw].empty()) {
            continue;
        }
        if ((slot.cw_bitmap & bit) == 0) {
            merged = true;
            if (partial.from_repair) {
                stats_.data_repair_cws_merged++;
            }
        }
        slot.cw_bitmap |= bit;
        slot.cw_data[cw] = partial.data[cw];
    }

    if (merged) {
        stats_.partial_frames_received++;
    }

    LOG_MODEM(INFO, "SR-ARQ: Partial DATA seq=%d cw=0x%08X/%08X missing=0x%08X",
              partial.seq, slot.cw_bitmap, expected, expected & ~slot.cw_bitmap);

    if (tryCompletePartialRXSlot(slot_index)) {
        return;
    }

    maybeSendCwNack(slot_index, expected & ~slot.cw_bitmap);
}

void SelectiveRepeatARQ::handleDataRepairFrame(const v2::DataRepairFrame& repair) {
    v2::PartialFrameCodewords partial;
    partial.type = v2::FrameType::DATA;
    partial.flags = v2::Flags::VERSION_V2;
    partial.seq = repair.target_seq;
    partial.src_hash = repair.src_hash;
    partial.dst_hash = repair.dst_hash;
    partial.total_cw = repair.original_total_cw;
    partial.decoded_bitmap = repair.repair_bitmap;
    partial.from_repair = true;
    partial.data.assign(repair.original_total_cw, Bytes{});

    auto indices = repair.repairIndices();
    for (size_t i = 0; i < indices.size() && i < repair.repair_codewords.size(); ++i) {
        partial.data[indices[i]] = repair.repair_codewords[i];
    }

    stats_.data_repairs_received++;
    LOG_MODEM(INFO, "SR-ARQ: DATA_REPAIR seq=%d bitmap=0x%04X repair_cw=%d",
              repair.target_seq, repair.repair_bitmap, repair.repair_count);
    handlePartialFrame(partial);
}

bool SelectiveRepeatARQ::handleAckFrame(const v2::ControlFrame& frame) {
    uint16_t seq = frame.seq;
    uint32_t bitmap = arq_policy::decodeSackBitmap(frame.payload);

    // MOVE-EPOCH (ULTRA_ARQ_MOVE_EPOCH, no-op when OFF): the ACK's era echo rides
    // SACK bitmap bits 16-17 (the window bitmap occupies bits 0-15 — MAX_WINDOW=16;
    // bits 24-31 must stay clear of the decodeSackBitmap legacy-8-bit shim). Strip
    // it before ANY bitmap use.
    uint8_t ack_epoch = 0;
    if (move_epoch_enabled_) {
        ack_epoch = static_cast<uint8_t>((bitmap >> 16) & 0x3u);
        bitmap &= 0xFFFFu;
    }

    LOG_MODEM(INFO, "SR-ARQ: ACK seq=%d bitmap=0x%08X (base=%d, in_flight=%zu)",
              seq, bitmap, tx_base_seq_, tx_in_flight_);
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_ack_rx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " ack_seq=" << seq
            << " bitmap=0x" << std::hex << bitmap << std::dec
            << " tx_base=" << tx_base_seq_
            << " in_flight=" << tx_in_flight_;
        ultra::phyDiagLine(oss.str());
    }
    // MOVE-EPOCH gate (before the seq-space guards: seq comparisons are meaningless
    // across eras). An epoch-mismatched ACK was formed against a pre-abort seq grid —
    // crediting it against the re-gridded frames is exactly the W16 phantom-retire
    // (648 B permanently unresendable). IGNORING an ACK is always protocol-safe
    // (identical to ACK loss: the RTO resends). Returns BEFORE the dedup-signature
    // update and before last_ack_progress_frames_ is touched (§RETX-PACING: non-fresh
    // acks never count as a round).
    if (move_epoch_enabled_ && ack_epoch != tx_epoch_) {
        stats_.stale_epoch_acks_ignored++;
        LOG_MODEM(WARN,
                  "SR-ARQ: stale-epoch ACK ignored (ack_epoch=%u tx_epoch=%u seq=%d bitmap=0x%08X)",
                  ack_epoch, tx_epoch_, seq, bitmap);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_ack_ignore"
                << " local=" << local_call_
                << " ack_seq=" << seq
                << " reason=stale_epoch"
                << " ack_epoch=" << static_cast<int>(ack_epoch)
                << " tx_epoch=" << static_cast<int>(tx_epoch_)
                << " tx_base=" << tx_base_seq_
                << " bitmap=0x" << std::hex << bitmap << std::dec;
            ultra::phyDiagLine(oss.str());
        }
        return false;
    }

    // Stale-ACK guard: reject ACKs strictly older than (tx_base_seq_ - 1).
    // seq == (tx_base_seq_ - 1) is valid — it's the common "no new cumulative progress"
    // ACK that still carries a fresh SACK bitmap for the current window.
    const auto ack_freshness = arq_policy::classifyAckFreshness(
        seq, tx_base_seq_, config_.window_size);
    const uint16_t ack_base = (tx_base_seq_ - 1) & 0xFFFF;
    if (ack_freshness == arq_policy::AckFreshness::Stale) {
        stats_.stale_acks_ignored++;
        LOG_MODEM(INFO, "SR-ARQ: Stale ACK seq=%d < base-1=%d, ignoring", seq, ack_base);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_ack_ignore"
                << " local=" << local_call_
                << " ack_seq=" << seq
                << " reason=stale"
                << " tx_base=" << tx_base_seq_
                << " bitmap=0x" << std::hex << bitmap << std::dec;
            ultra::phyDiagLine(oss.str());
        }
        return false;
    }

    // Far-future guard: reject ACKs implausibly ahead (> window_size + 1 past base)
    if (ack_freshness == arq_policy::AckFreshness::Future) {
        stats_.future_acks_ignored++;
        LOG_MODEM(INFO, "SR-ARQ: Future ACK seq=%d too far ahead of base=%d, ignoring", seq, tx_base_seq_);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_ack_ignore"
                << " local=" << local_call_
                << " ack_seq=" << seq
                << " reason=future"
                << " tx_base=" << tx_base_seq_
                << " bitmap=0x" << std::hex << bitmap << std::dec;
            ultra::phyDiagLine(oss.str());
        }
        return false;
    }

    // NEVER-SENT guard (BUG-TONEACK-FABRICATION defense-in-depth): the window-based
    // Future guard above admits seqs up to window+1 past base, but the receiver can
    // only ever ack seqs we actually TRANSMITTED — [base-1, tx_next_seq_-1]. An ack
    // beyond tx_next_seq_-1 is fabricated (corrupt frame that survived its CRC, or a
    // mis-expanded tone ack) and cumulative-retiring on it destroys payload bytes
    // irreversibly (fires on_send_complete for undelivered frames). Drop = ack loss,
    // always recoverable.
    {
        const uint16_t fwd = static_cast<uint16_t>((seq - ack_base) & 0xFFFF);
        const uint16_t sent_span =
            static_cast<uint16_t>((tx_next_seq_ - tx_base_seq_) & 0xFFFF);
        if (fwd < 0x8000 && fwd > sent_span) {
            stats_.fabricated_acks_dropped++;
            LOG_MODEM(WARN,
                      "SR-ARQ: ACK seq=%d beyond highest sent seq %u (base=%u) — "
                      "dropped as fabricated",
                      seq, static_cast<unsigned>((tx_next_seq_ - 1) & 0xFFFF),
                      tx_base_seq_);
            return false;
        }
    }

    // Count protocol-valid SACKs only. Rejected stale/future/epoch/fabricated ACKs
    // already have an exclusive reason counter above and must not inflate the accepted
    // SACK total. A support-valid duplicate still counts as received; the dedup counter
    // below records that its protocol effects were suppressed.
    stats_.sacks_received++;

    // ACK-repeat dedup guard: suppress clustered duplicate ACKs carrying
    // identical cumulative+bitmap information.
    if (arq_policy::shouldSuppressDuplicateAck(
            last_ack_signature_valid_, ack_dedup_timer_ms_, last_ack_seq_,
            last_ack_bitmap_, seq, bitmap)) {
        stats_.duplicate_acks_ignored++;
        LOG_MODEM(INFO, "SR-ARQ: Duplicate ACK seq=%d bitmap=0x%08X suppressed", seq, bitmap);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_ack_ignore"
                << " local=" << local_call_
                << " ack_seq=" << seq
                << " reason=duplicate"
                << " tx_base=" << tx_base_seq_
                << " bitmap=0x" << std::hex << bitmap << std::dec;
            ultra::phyDiagLine(oss.str());
        }
        return true;
    }
    last_ack_signature_valid_ = true;
    last_ack_seq_ = seq;
    last_ack_bitmap_ = bitmap;
    ack_dedup_timer_ms_ = arq_policy::ackDedupWindowMs(ack_repeat_delay_ms_);

    // §RETX-PACING §1.1: forward progress of THIS (fresh — the dedup/stale/future guards
    // above returned already) ack = frames retired by base advance + newly-set SACK bits.
    int ack_progress_frames = 0;

    // --- Cumulative ACK: advance base past all frames up to seq ---
    uint16_t base_before_ack = tx_base_seq_;
    // tx_base_seq_ != tx_next_seq_: structural invariant — the cumulative walk can
    // never advance base PAST the highest seq ever sent, whatever the ack claims
    // (F116: a fabricated base=69 walked base 57->63 with tx_next=63, then the split
    // window was unhealable because every real ack read as stale).
    while (tx_in_flight_ > 0 && tx_base_seq_ != tx_next_seq_ &&
           tx_base_seq_ != ((seq + 1) & 0xFFFF)) {
        size_t slot = seqToSlot(tx_base_seq_);
        // SLOT-IDENTITY guard (defense-in-depth, 2026-07-24): retire the slot only if
        // it actually HOLDS the seq being retired. The SACK-bitmap loop below has always
        // verified `.seq == sack_seq`; this cumulative walk checked only `.active`, so a
        // slot occupied by a DIFFERENT seq (aliased through seqToSlot when a hole is
        // punched inside [base,next) — reachable once the live span reaches the window
        // size) would be retired here, firing on_send_complete_(true) and irreversibly
        // popping the FileTransfer tx ledger for a frame the receiver never confirmed.
        // Free to add, and it makes the two ack paths agree on what "this frame" means.
        if (tx_window_[slot].active && tx_window_[slot].seq == tx_base_seq_) {
            maybeSampleRTT(tx_window_[slot]);
            tx_window_[slot].active = false;
            tx_window_[slot].acked = true;
            tx_window_[slot].hole_ack_count = 0;
            tx_window_[slot].fast_retx_count = 0;
            tx_window_[slot].fast_retx_cooldown_ms = 0;
            tx_window_[slot].hole_probe_armed = false;
            tx_window_[slot].hole_probe_timer_ms = 0;
            tx_window_[slot].hole_probe_count = 0;
            clearTXSlotRepairState(tx_window_[slot]);
            tx_in_flight_--;
            stats_.acks_received++;
            ack_progress_frames++;

            if (on_send_complete_) {
                on_send_complete_(true);
            }
        }
        tx_base_seq_ = (tx_base_seq_ + 1) & 0xFFFF;
    }
    notifyTXBaseAdvanced(base_before_ack);

    // --- Positive-only SACK bitmap: mark frames the receiver confirms it HAS ---
    if (bitmap != 0) {
        bool any_sacked = false;
        for (int i = 0; i < 32 && i < static_cast<int>(config_.window_size); i++) {
            if (!(bitmap & (1u << i))) continue;

            uint16_t sack_seq = (tx_base_seq_ + i) & 0xFFFF;
            size_t slot = seqToSlot(sack_seq);

            if (tx_window_[slot].active && !tx_window_[slot].acked && tx_window_[slot].seq == sack_seq) {
                tx_window_[slot].acked = true;
                any_sacked = true;
                ack_progress_frames++;
                LOG_MODEM(INFO, "SR-ARQ: SACK seq=%d confirmed received (bitmap=0x%08X)", sack_seq, bitmap);
            }
        }

        if (any_sacked) {
            advanceTXWindow();
        }
    }

    // Publish the round outcome (consumed once per round boundary by the Connection).
    last_ack_progress_frames_ = ack_progress_frames;

    // --- Hole-based fast retransmit for base gap frame ---
    // Trigger: ACK aligned to base (seq == tx_base-1), bit0=0, any higher bit set.
    // This means the receiver is missing the base frame but has later frames.
    //
    // DISABLED on the ready tone-burst / stop-and-wait burst path.
    // Fast-retx + hole-probe are PIPELINING mechanisms: they resend a lost frame
    // mid-stream while later frames are still flowing, before a timeout. Half-duplex
    // burst is stop-and-wait — you key down a whole group, turn around, and get ONE
    // tone-burst back, then form the next burst ([remaining holes] + [new]). There is
    // nothing to pipeline; fast-retx only adds cooldowns, 2-confirmation latency, and
    // spurious duplicate resends to an inherently one-turn loop. Resends here are
    // timeout-only (the single burst-level ack timeout), driven by forming the next
    // coalesced burst on the one ack/timeout per turn.
    if (!toneBurstSackTransportReady() &&
        arq_policy::isAlignedBaseHoleAck(seq, tx_base_seq_, bitmap)) {
        size_t base_slot = seqToSlot(tx_base_seq_);
        TXSlot& s = tx_window_[base_slot];

        if (s.active && !s.acked && s.seq == tx_base_seq_) {
            s.hole_ack_count++;
            stats_.hole_events++;
            LOG_MODEM(INFO, "SR-ARQ: Hole detected for base seq=%d (hole_count=%d, bitmap=0x%08X)",
                      tx_base_seq_, s.hole_ack_count, bitmap);

            // Do not (re)arm the hole-probe while a fast-hole retransmit for this
            // base seq is still within its in-flight cooldown window. The fast-hole
            // handler disarms the probe when it fires (see below), but subsequent
            // hole-confirmation SACKs still show the gap (the repair hasn't landed +
            // been ACKed across the half-duplex turnaround yet), so without this guard
            // the probe re-arms and then fires a redundant retx of a frame the
            // fast-hole repair already recovered. fast_retx_cooldown_ms is RTT-scaled
            // (ack_timeout/6) and self-clears in tick(); once it expires with the hole
            // still open (fast-hole genuinely failed), the probe re-arms for legitimate
            // escalation. Mirrors the "fast_hole already serves that purpose" disarm.
            if (!s.hole_probe_armed && s.fast_retx_cooldown_ms == 0) {
                s.hole_probe_armed = true;
                s.hole_probe_count = 0;
                s.hole_probe_timer_ms = arq_policy::holeProbeInitialTimerMs(
                    currentAckTimeoutMs());
                LOG_MODEM(INFO, "SR-ARQ: Armed hole-probe timer for seq=%d (%ums)",
                          s.seq, s.hole_probe_timer_ms);
            }

            // Send one fast repair for a base hole. Additional SACK bitmap updates
            // often arrive before the repair ACK catches up, so repeated fast
            // retransmits mostly create stale out-of-window duplicates.
            //
            // Require TWO hole-confirmation SACKs before retx — protects against
            // false-loss inferences when the original ACK is just delayed in the
            // audio buffer chain. Real-hardware tests showed fast_hole firing
            // before the original ACK had time to traverse the 340ms-each-way
            // soundcard buffers, creating ~25% wasted duplicate retx. With
            // window=4 and ACKs arriving every ~700ms, requiring 2 SACKs adds
            // ~700ms of latency to genuine-loss recovery — acceptable cost for
            // eliminating the spurious retx storm.
            uint32_t fast_retx_cooldown_ms = arq_policy::fastRetransmitCooldownMs(
                config_.ack_timeout_ms);
            if (arq_policy::shouldFastRetransmitHole(
                    s.hole_ack_count, s.fast_retx_count, s.fast_retx_cooldown_ms)) {
                s.fast_retx_count++;
                s.fast_retx_cooldown_ms = fast_retx_cooldown_ms;
                // Reset the timeout timer too — we just retx'd, give the new
                // ACK a fresh round-trip window before timer-firing again.
                // Without this, fast_hole + timeout both fire for the same
                // seq, doubling the duplicate count. Likewise disarm the
                // hole_probe — fast_hole already serves that purpose.
                s.timeout_ms = currentAckTimeoutMs();
                s.hole_probe_armed = false;
                s.hole_probe_timer_ms = 0;
                LOG_MODEM(INFO,
                          "SR-ARQ: Fast retransmit base seq=%d (bitmap=0x%08X, fast=%d/%d, cooldown=%ums, confirms=%d)",
                          tx_base_seq_, bitmap, s.fast_retx_count,
                          arq_policy::kMaxFastRetransmitsPerHole,
                          fast_retx_cooldown_ms, s.hole_ack_count);
                retransmitFrame(base_slot, RetransmitCause::FAST_HOLE);
            }
        }
    }

    // Reset hole and fast-retransmit guards when base advances (new gap context).
    if (tx_base_seq_ != base_before_ack) {
        for (size_t i = 0; i < config_.window_size; i++) {
            size_t slot = seqToSlot((tx_base_seq_ + i) & 0xFFFF);
            tx_window_[slot].hole_ack_count = 0;
            tx_window_[slot].fast_retx_count = 0;
            tx_window_[slot].fast_retx_cooldown_ms = 0;
            tx_window_[slot].hole_probe_armed = false;
            tx_window_[slot].hole_probe_timer_ms = 0;
            tx_window_[slot].hole_probe_count = 0;
        }
    }
    return true;
}

void SelectiveRepeatARQ::handleNackFrame(const v2::ControlFrame& frame) {
    v2::NackPayload np = v2::NackPayload::decode(frame.payload);
    uint16_t seq = np.frame_seq;
    if (seq != frame.seq) {
        seq = frame.seq;
    }

    LOG_MODEM(INFO, "SR-ARQ: NACK seq=%d missing_cw=0x%08X", seq, np.cw_bitmap);
    if (np.cw_bitmap != 0) {
        stats_.cw_nacks_received++;
    }

    if (isInTXWindow(seq)) {
        size_t slot = seqToSlot(seq);
        if (tx_window_[slot].active && !tx_window_[slot].acked) {
            if (np.cw_bitmap != 0 && sendDataRepair(slot, np.cw_bitmap)) {
                return;
            }
            retransmitFrame(slot, RetransmitCause::NACK);
        }
    }
}

void SelectiveRepeatARQ::tick(uint32_t elapsed_ms) {
    arq_time_ms_ += elapsed_ms;
    keepalive_silent_ms_ += elapsed_ms;  // reset on each sendSack() emit

    if (ack_dedup_timer_ms_ > 0) {
        if (elapsed_ms >= ack_dedup_timer_ms_) {
            ack_dedup_timer_ms_ = 0;
        } else {
            ack_dedup_timer_ms_ -= elapsed_ms;
        }
    }

    // Delayed ACK repeats (time diversity for fading channels).
    // Jobs are one-shot; each queued copy is sent once when its timer expires.
    for (auto it = ack_repeat_jobs_.begin(); it != ack_repeat_jobs_.end();) {
        AckRepeatJob& job = *it;
        if (elapsed_ms >= job.timer_ms) {
            transmitData(job.frame_data);
            stats_.acks_sent++;
            LOG_MODEM(INFO, "SR-ARQ: ACK_REPEAT_SENT copy=%d", job.copy_index);
            if (ultra::phyDiagnosticsEnabled()) {
                std::ostringstream oss;
                oss << "event=arq_ack_repeat_tx"
                    << " local=" << local_call_
                    << " remote=" << remote_call_
                    << " ack_seq=" << job.base_seq
                    << " bitmap=0x" << std::hex << job.bitmap << std::dec
                    << " copy=" << job.copy_index;
                ultra::phyDiagLine(oss.str());
            }
            it = ack_repeat_jobs_.erase(it);
            continue;
        }

        job.timer_ms -= elapsed_ms;
        ++it;
    }

    // TX side: check for timeouts and retransmit. When several slots expire
    // together, defer their physical transmission so OFDM can send one repair
    // burst instead of N independent full-preamble waveforms.
    std::vector<Bytes> timeout_retx_batch;
    std::vector<Bytes>* timeout_retx_batch_ptr =
        on_transmit_batch_ ? &timeout_retx_batch : nullptr;
    for (size_t i = 0; i < config_.window_size; i++) {
        size_t slot = seqToSlot((tx_base_seq_ + i) & 0xFFFF);
        TXSlot& s = tx_window_[slot];

        if (s.active && !s.acked) {
            if (s.repair_cooldown_ms > 0) {
                if (elapsed_ms >= s.repair_cooldown_ms) {
                    s.repair_cooldown_ms = 0;
                } else {
                    s.repair_cooldown_ms -= elapsed_ms;
                }
            }
            if (s.repair_guard_ms > 0) {
                if (elapsed_ms >= s.repair_guard_ms) {
                    s.repair_guard_ms = 0;
                    s.repair_in_flight = false;
                } else {
                    s.repair_guard_ms -= elapsed_ms;
                }
            }

            if (s.fast_retx_cooldown_ms > 0) {
                if (elapsed_ms >= s.fast_retx_cooldown_ms) {
                    s.fast_retx_cooldown_ms = 0;
                } else {
                    s.fast_retx_cooldown_ms -= elapsed_ms;
                }
            }

            if (!s.timeout_suspended && s.hole_probe_armed) {
                if (elapsed_ms >= s.hole_probe_timer_ms) {
                    if (s.repair_in_flight && s.repair_guard_ms > 0) {
                        LOG_MODEM(INFO,
                                  "SR-ARQ: Suppressed hole-probe full retx seq=%d while DATA_REPAIR in flight (%ums)",
                                  s.seq, s.repair_guard_ms);
                        s.hole_probe_timer_ms = s.repair_guard_ms;
                    } else if (s.hole_probe_count < arq_policy::kMaxHoleProbeRetransmits) {
                        s.hole_probe_count++;
                        s.hole_probe_timer_ms = arq_policy::holeProbeNextTimerMs(
                            currentAckTimeoutMs());
                        LOG_MODEM(INFO,
                                  "SR-ARQ: Hole-probe retransmit seq=%d (%d/%d)",
                                  s.seq, s.hole_probe_count,
                                  arq_policy::kMaxHoleProbeRetransmits);
                        const int failures_before = stats_.failed;
                        retransmitFrame(slot, RetransmitCause::HOLE_PROBE);
                        if (stats_.failed != failures_before) {
                            // The slot is inactive and the callback requested a
                            // transfer-level unwind. Stop before touching the stale
                            // reference again or failing another slot in this scan.
                            break;
                        }
                    } else {
                        s.hole_probe_armed = false;
                        s.hole_probe_timer_ms = 0;
                    }
                } else {
                    s.hole_probe_timer_ms -= elapsed_ms;
                }
            }

            if (s.timeout_suspended) {
                // A different subset is the current half-duplex physical round. This
                // slot has not gone on air in that round, so it cannot legitimately
                // time out and start a colliding second burst.
                continue;
            }
            if (elapsed_ms >= s.timeout_ms) {
                if (s.repair_in_flight && s.repair_guard_ms > 0) {
                    LOG_MODEM(INFO,
                              "SR-ARQ: Suppressed timeout full retx seq=%d while DATA_REPAIR in flight (%ums)",
                              s.seq, s.repair_guard_ms);
                    s.timeout_ms = s.repair_guard_ms;
                } else {
                    if (ultra::phyDiagnosticsEnabled()) {
                        std::ostringstream oss;
                        oss << "event=arq_timeout"
                            << " local=" << local_call_
                            << " remote=" << remote_call_
                            << " seq=" << s.seq
                            << " slot=" << slot
                            << " retry_count=" << s.retry_count
                            << " in_flight=" << tx_in_flight_
                            << " ack_timeout_ms=" << currentAckTimeoutMs();
                        ultra::phyDiagLine(oss.str());
                    }
                    stats_.timeouts++;
                    const int failures_before = stats_.failed;
                    retransmitFrame(slot, RetransmitCause::TIMEOUT,
                                    timeout_retx_batch_ptr);
                    if (stats_.failed != failures_before) {
                        // One terminal event owns this tick. Connection suppresses
                        // any earlier deferred batch and atomically aborts the rest
                        // of the logical window after tick() unwinds.
                        break;
                    }
                }
            } else {
                s.timeout_ms -= elapsed_ms;
            }
        }
    }
    transmitDataBatch(timeout_retx_batch);

    // RX side: delayed SACK for half-duplex burst handling
    if (sack_pending_) {
        if (elapsed_ms >= sack_timer_ms_) {
            LOG_MODEM(DEBUG, "SR-ARQ: SACK timer expired, sending SACK");
            stats_.sack_trigger_timer++;
            sendSack();
            sack_pending_ = false;
            sack_timer_ms_ = 0;
            frames_since_ack_ = 0;
        } else {
            sack_timer_ms_ -= elapsed_ms;
        }
    }

    for (auto& slot : rx_window_) {
        if (!slot.partial || slot.received) {
            continue;
        }
        if (slot.cw_nack_cooldown_ms > 0) {
            slot.cw_nack_cooldown_ms = elapsed_ms >= slot.cw_nack_cooldown_ms
                ? 0
                : slot.cw_nack_cooldown_ms - elapsed_ms;
        }
        if (elapsed_ms >= PARTIAL_RX_TTL_MS - std::min(slot.partial_age_ms, PARTIAL_RX_TTL_MS)) {
            LOG_MODEM(WARN, "SR-ARQ: Partial DATA seq=%d expired (cw=0x%08X)",
                      slot.seq, slot.cw_bitmap);
            clearPartialRXSlot(slot);
            stats_.partial_frame_expired++;
        } else {
            slot.partial_age_ms += elapsed_ms;
        }
    }
}

uint32_t SelectiveRepeatARQ::computeRepairGuardMs(const TXSlot& slot,
                                                  size_t repair_frame_codewords) const {
    const uint32_t full_timeout_ms = std::max<uint32_t>(currentAckTimeoutMs(), 1u);
    const size_t original_cw = std::max<size_t>(
        1, slot.info_codewords.empty()
               ? static_cast<size_t>(std::max(slot.fixed_frame_codewords, 1))
               : slot.info_codewords.size());
    const size_t repair_cw = std::max<size_t>(1, repair_frame_codewords);

    const uint64_t scaled_airtime_ms =
        (static_cast<uint64_t>(full_timeout_ms) * repair_cw + original_cw - 1) / original_cw;
    const uint32_t repeat_margin_ms =
        ack_repeat_delay_ms_ * static_cast<uint32_t>(std::max(0, ack_repeat_count_ - 1));
    const uint32_t ack_margin_ms =
        std::max<uint32_t>(1000u, config_.sack_delay_ms + repeat_margin_ms);
    const uint32_t max_guard_ms = std::max(full_timeout_ms, ack_margin_ms);
    const uint64_t guard_ms = scaled_airtime_ms + ack_margin_ms;
    return static_cast<uint32_t>(std::min<uint64_t>(guard_ms, max_guard_ms));
}

bool SelectiveRepeatARQ::suppressFullRetransmitForRepair(size_t slot,
                                                         RetransmitCause cause) {
    TXSlot& s = tx_window_[slot];
    if (!s.repair_in_flight || s.repair_guard_ms == 0) {
        return false;
    }

    const char* cause_str = "unknown";
    switch (cause) {
        case RetransmitCause::TIMEOUT: cause_str = "timeout"; break;
        case RetransmitCause::FAST_HOLE: cause_str = "fast-hole"; break;
        case RetransmitCause::HOLE_PROBE: cause_str = "hole-probe"; break;
        case RetransmitCause::NACK: cause_str = "nack"; break;
    }

    LOG_MODEM(INFO,
              "SR-ARQ: Suppressed %s full retx seq=%d while DATA_REPAIR in flight (%ums)",
              cause_str, s.seq, s.repair_guard_ms);
    if (s.timeout_ms == 0 || s.timeout_ms > s.repair_guard_ms) {
        s.timeout_ms = s.repair_guard_ms;
    }
    return true;
}

void SelectiveRepeatARQ::retransmitFrame(size_t slot,
                                         RetransmitCause cause,
                                         std::vector<Bytes>* deferred_timeout_batch) {
    TXSlot& s = tx_window_[slot];
    if (suppressFullRetransmitForRepair(slot, cause)) {
        return;
    }

    if (cause == RetransmitCause::TIMEOUT && deferred_timeout_batch != nullptr &&
        deferred_timeout_commit_enabled_) {
        // Two-phase Connection path: this is an egress INTENT, not a retransmission
        // yet. In particular, do not consume retry budget, Karn eligibility, repair/
        // hole state, retransmission counters, or the terminal-failure callback. The
        // exact serialized identity lets the post-tick owner reject an obsolete rung
        // without ever mutating its live slot.
        deferred_timeout_batch->push_back(s.frame_data);
        return;
    }

    performRetransmitFrame(slot, cause, deferred_timeout_batch);
}

void SelectiveRepeatARQ::performRetransmitFrame(
    size_t slot, RetransmitCause cause,
    std::vector<Bytes>* deferred_timeout_batch) {
    TXSlot& s = tx_window_[slot];

    s.repair_in_flight = false;
    s.repair_guard_ms = 0;
    s.last_repair_bitmap = 0;
    s.repair_cooldown_ms = 0;

    if (cause == RetransmitCause::TIMEOUT) {
        // New timeout epoch: permit another cycle of hole-based fast retransmits.
        s.fast_retx_count = 0;
        s.fast_retx_cooldown_ms = 0;
        s.hole_ack_count = 0;
        s.hole_probe_armed = false;
        s.hole_probe_timer_ms = 0;
        s.hole_probe_count = 0;
    }

    // Karn's algorithm: once retransmitted, do not use this frame for RTT sampling.
    s.rtt_sample_eligible = false;

    s.retry_count++;
    if (s.retry_count >= config_.max_retries) {
        LOG_MODEM(ERROR, "SR-ARQ: Frame seq=%d failed after %d retries",
                  s.seq, config_.max_retries);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_frame_fail"
                << " local=" << local_call_
                << " remote=" << remote_call_
                << " seq=" << s.seq
                << " retries=" << s.retry_count
                << " max_retries=" << config_.max_retries;
            ultra::phyDiagLine(oss.str());
        }
        stats_.failed++;

        if (on_tx_frame_failed_) {
            on_tx_frame_failed_(s.seq);
        }
        s.active = false;
        tx_in_flight_--;

        if (on_send_complete_) {
            on_send_complete_(false);
        }

        advanceTXWindow();
        return;
    }

    const char* cause_str = "unknown";
    switch (cause) {
        case RetransmitCause::TIMEOUT: cause_str = "timeout"; break;
        case RetransmitCause::FAST_HOLE: cause_str = "fast-hole"; break;
        case RetransmitCause::HOLE_PROBE: cause_str = "hole-probe"; break;
        case RetransmitCause::NACK: cause_str = "nack"; break;
    }

    LOG_MODEM(INFO, "SR-ARQ: Retransmitting seq=%d (attempt %d/%d, cause=%s, cw=%d)",
              s.seq, s.retry_count + 1, config_.max_retries, cause_str,
              s.fixed_frame_codewords);
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_retx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " seq=" << s.seq
            << " slot=" << slot
            << " attempt=" << (s.retry_count + 1)
            << " max_retries=" << config_.max_retries
            << " cause=" << cause_str
            << " cw=" << s.fixed_frame_codewords
            << " in_flight=" << tx_in_flight_
            << " timeout_ms=" << currentAckTimeoutMs();
        ultra::phyDiagLine(oss.str());
    }

    stats_.retransmissions++;
    switch (cause) {
        case RetransmitCause::TIMEOUT: stats_.retransmissions_timeout++; break;
        case RetransmitCause::FAST_HOLE: stats_.retransmissions_fast_hole++; break;
        case RetransmitCause::HOLE_PROBE: stats_.retransmissions_hole_probe++; break;
        case RetransmitCause::NACK: stats_.retransmissions_nack++; break;
    }
    s.timeout_ms = currentAckTimeoutMs();
    s.timeout_suspended = false;
    s.timeout_transition_suspended = false;
    if (cause == RetransmitCause::TIMEOUT && deferred_timeout_batch != nullptr) {
        deferred_timeout_batch->push_back(s.frame_data);
    } else {
        transmitData(s.frame_data);
    }
}

bool SelectiveRepeatARQ::sendDataRepair(size_t slot, uint32_t missing_bitmap) {
    TXSlot& s = tx_window_[slot];
    if (s.info_codewords.empty()) {
        return false;
    }
    if (s.info_codewords.size() > v2::DataRepairFrame::MAX_REPAIR_CW) {
        return false;
    }

    uint32_t available_mask = 0;
    for (size_t i = 0; i < s.info_codewords.size() && i < 32; ++i) {
        available_mask |= (1u << i);
    }
    const uint32_t repair_bitmap = missing_bitmap & available_mask & 0xFFFFu;
    if (repair_bitmap == 0) {
        return false;
    }
    if (s.last_repair_bitmap == repair_bitmap && s.repair_cooldown_ms > 0) {
        LOG_MODEM(DEBUG, "SR-ARQ: Suppressed duplicate DATA_REPAIR seq=%d bitmap=0x%04X",
                  s.seq, static_cast<unsigned>(repair_bitmap));
        return true;
    }

    std::vector<Bytes> repair_codewords;
    repair_codewords.reserve(8);
    for (size_t i = 0; i < s.info_codewords.size() && i < 16; ++i) {
        if ((repair_bitmap & (1u << i)) != 0) {
            repair_codewords.push_back(s.info_codewords[i]);
        }
    }
    if (repair_codewords.empty()) {
        return false;
    }

    // Karn's algorithm: repair frames are retransmissions of the original seq.
    s.rtt_sample_eligible = false;
    s.retry_count++;
    if (s.retry_count >= config_.max_retries) {
        LOG_MODEM(ERROR, "SR-ARQ: Frame seq=%d failed after %d repairs/retries",
                  s.seq, config_.max_retries);
        stats_.failed++;
        if (on_tx_frame_failed_) {
            on_tx_frame_failed_(s.seq);
        }
        s.active = false;
        tx_in_flight_--;
        if (on_send_complete_) {
            on_send_complete_(false);
        }
        advanceTXWindow();
        return true;
    }

    auto repair = v2::DataRepairFrame::make(local_call_, remote_call_, s.seq,
                                            static_cast<uint8_t>(s.info_codewords.size()),
                                            repair_bitmap, code_rate_, repair_codewords);
    Bytes repair_data = repair.serialize();
    if (repair_data.empty()) {
        return false;
    }

    stats_.retransmissions++;
    stats_.retransmissions_nack++;
    stats_.data_repairs_sent++;
    stats_.data_repair_cws_sent += static_cast<int>(repair_codewords.size());

    const uint32_t repair_guard_ms = computeRepairGuardMs(s, repair_codewords.size() + 1);
    s.timeout_ms = repair_guard_ms;
    s.timeout_suspended = false;
    s.timeout_transition_suspended = false;
    s.last_repair_bitmap = repair_bitmap;
    s.repair_cooldown_ms = repair_guard_ms;
    s.repair_in_flight = true;
    s.repair_guard_ms = repair_guard_ms;

    LOG_MODEM(INFO,
              "SR-ARQ: DATA_REPAIR seq=%d bitmap=0x%04X repair_cw=%zu guard=%ums (attempt %d/%d)",
              s.seq, static_cast<unsigned>(repair_bitmap), repair_codewords.size(),
              repair_guard_ms, s.retry_count + 1, config_.max_retries);
    transmitData(repair_data);
    return true;
}

void SelectiveRepeatARQ::advanceTXWindow() {
    const uint16_t base_before = tx_base_seq_;
    while (tx_in_flight_ > 0) {
        size_t slot = seqToSlot(tx_base_seq_);
        if (tx_window_[slot].active && !tx_window_[slot].acked) {
            break;
        }
        if (tx_window_[slot].active) {
            maybeSampleRTT(tx_window_[slot]);
            tx_window_[slot].active = false;
            tx_window_[slot].hole_ack_count = 0;
            tx_window_[slot].fast_retx_count = 0;
            tx_window_[slot].fast_retx_cooldown_ms = 0;
            tx_window_[slot].hole_probe_armed = false;
            tx_window_[slot].hole_probe_timer_ms = 0;
            tx_window_[slot].hole_probe_count = 0;
            clearTXSlotRepairState(tx_window_[slot]);
            tx_in_flight_--;

            if (on_send_complete_) {
                on_send_complete_(true);
            }
        }
        tx_base_seq_ = (tx_base_seq_ + 1) & 0xFFFF;
    }
    notifyTXBaseAdvanced(base_before);
}

void SelectiveRepeatARQ::notifyTXBaseAdvanced(uint16_t base_before) {
    if (tx_base_seq_ != base_before && on_tx_base_advanced_) {
        on_tx_base_advanced_(tx_base_seq_);
    }
}

void SelectiveRepeatARQ::advanceRXWindow() {
    const uint16_t base_before = rx_base_seq_;
    while (true) {
        size_t slot = seqToSlot(rx_base_seq_);
        if (!rx_window_[slot].received) {
            break;
        }

        LOG_MODEM(DEBUG, "SR-ARQ: Delivering seq=%d", rx_base_seq_);

        // Update flags from the delivered frame's stored flags (not from the
        // last arrived frame). When advanceRXWindow delivers multiple buffered
        // frames in sequence (e.g., after retransmission fills a gap), the
        // Connection layer calls lastRxHadMoreData() to check MORE_FRAG.
        // Without this, it would see the flags from handleDataFrame's last
        // call, which is the gap-filling frame — not the frame being delivered.
        last_rx_flags_ = rx_window_[slot].flags;
        last_rx_more_data_ = (rx_window_[slot].flags & v2::Flags::MORE_FRAG) != 0;
        last_rx_frame_type_ = rx_window_[slot].type;
        if ((rx_window_[slot].flags & v2::Flags::FINAL) != 0) {
            rx_final_delivered_since_sack_ = true;
        }

        if (on_data_received_) {
            on_data_received_(rx_window_[slot].payload);
        }

        rx_window_[slot].received = false;
        clearPartialRXSlot(rx_window_[slot]);
        rx_window_[slot].payload.clear();
        rx_window_[slot].flags = 0;
        rx_window_[slot].type = v2::FrameType::DATA;
        rx_base_seq_ = (rx_base_seq_ + 1) & 0xFFFF;
    }

    if (rx_base_seq_ != base_before && on_rx_window_advanced_) {
        on_rx_window_advanced_(rx_base_seq_, config_.window_size);
    }
}

void SelectiveRepeatARQ::clearPartialRXSlot(RXSlot& slot) {
    slot.partial = false;
    slot.total_cw = 0;
    slot.cw_bitmap = 0;
    slot.partial_age_ms = 0;
    slot.last_cw_nack_bitmap = 0;
    slot.cw_nack_cooldown_ms = 0;
    slot.cw_data.clear();
}

void SelectiveRepeatARQ::clearTXSlotRepairState(TXSlot& slot) {
    slot.info_codewords.clear();
    slot.last_repair_bitmap = 0;
    slot.repair_cooldown_ms = 0;
    slot.repair_in_flight = false;
    slot.repair_guard_ms = 0;
}

bool SelectiveRepeatARQ::tryCompletePartialRXSlot(size_t slot_index) {
    RXSlot& slot = rx_window_[slot_index];
    if (!slot.partial || slot.total_cw == 0 || slot.total_cw > 32) {
        return false;
    }

    const uint32_t expected =
        slot.total_cw >= 32 ? 0xFFFFFFFFu : ((1u << slot.total_cw) - 1u);
    if ((slot.cw_bitmap & expected) != expected) {
        return false;
    }

    v2::CodewordStatus status;
    status.initForFrame(slot.total_cw);
    for (uint8_t i = 0; i < slot.total_cw; ++i) {
        status.decoded[i] = true;
        status.data[i] = slot.cw_data[i];
    }

    Bytes assembled = status.reassemble();
    auto frame = v2::DataFrame::deserialize(assembled);
    if (!frame || frame->seq != slot.seq) {
        LOG_MODEM(WARN, "SR-ARQ: Partial DATA seq=%d had all CWs but frame CRC rejected",
                  slot.seq);
        stats_.partial_frame_crc_failed++;
        const uint16_t seq = slot.seq;
        clearPartialRXSlot(slot);
        sendCwNack(seq, expected);
        return false;
    }

    stats_.partial_frames_completed++;
    LOG_MODEM(INFO, "SR-ARQ: Partial DATA seq=%d completed from CW slots", slot.seq);
    clearPartialRXSlot(slot);
    handleDataFrame(*frame);
    return true;
}

void SelectiveRepeatARQ::sendCwNack(uint16_t seq, uint32_t missing_bitmap) {
    if (missing_bitmap == 0) {
        return;
    }

    auto nack = v2::ControlFrame::makeNack(local_call_, remote_call_, seq, missing_bitmap);
    stats_.cw_nacks_sent++;
    auto data = nack.serialize();
    LOG_MODEM(INFO, "SR-ARQ: Sent CW_NACK seq=%d missing=0x%08X", seq, missing_bitmap);
    transmitData(data);
}

void SelectiveRepeatARQ::maybeSendCwNack(size_t slot_index, uint32_t missing_bitmap) {
    if (missing_bitmap == 0) {
        return;
    }
    RXSlot& slot = rx_window_[slot_index];
    if (slot.last_cw_nack_bitmap == missing_bitmap && slot.cw_nack_cooldown_ms > 0) {
        LOG_MODEM(DEBUG, "SR-ARQ: Suppressed duplicate CW_NACK seq=%d missing=0x%08X",
                  slot.seq, missing_bitmap);
        return;
    }

    sendCwNack(slot.seq, missing_bitmap);
    slot.last_cw_nack_bitmap = missing_bitmap;
    slot.cw_nack_cooldown_ms = std::max<uint32_t>(config_.sack_delay_ms, 1000u);
}

bool SelectiveRepeatARQ::isToneBurstAckPlausible(uint8_t group_seq6,
                                                 uint32_t bitmap,
                                                 uint8_t move_epoch) const {
    if (tx_in_flight_ == 0) return false;
    if (move_epoch_enabled_ && (move_epoch & 0x3u) != tx_epoch_) return false;

    const uint16_t ref = static_cast<uint16_t>((tx_base_seq_ - 1) & 0xFFFF);
    const uint16_t span =
        static_cast<uint16_t>((tx_next_seq_ - tx_base_seq_) & 0xFFFF);
    const uint16_t delta =
        static_cast<uint16_t>((group_seq6 - (ref & 0x3F)) & 0x3F);
    if (delta > span) return false;

    // After cumulative progress `delta`, bitmap bit i describes
    // tx_base+delta+i. A receiver cannot positively SACK a sequence at/after
    // tx_next_seq_, so any set bit beyond the remaining sent span is impossible.
    const uint16_t remaining = static_cast<uint16_t>(span - delta);
    const uint32_t allowed_bitmap =
        remaining >= 32 ? 0xFFFFFFFFu
                        : (remaining == 0 ? 0u : ((1u << remaining) - 1u));
    return (bitmap & ~allowed_bitmap) == 0;
}

bool SelectiveRepeatARQ::onToneBurstAck(uint8_t group_seq6, uint32_t bitmap,
                                        uint8_t move_epoch) {
    // SUPPORT-CONSTRAINED decode (BUG-TONEACK-FABRICATION, F116 2026-07-05): a
    // cumulative ack can only reference the receiver's in-order base, which lives in
    // [tx_base_seq_-1, tx_next_seq_-1] — behind base-1 is stale (our base only
    // advances on the receiver's own acks), ahead of the highest seq EVER SENT is
    // impossible (the receiver cannot ack what never left this radio). That support
    // spans <= window+1 < 64 seqs, so the 6-bit decode is UNIQUE inside it. A decode
    // OUTSIDE the support is a corrupt/aliased/foreign detection and must be DROPPED
    // — equivalent to ack loss, which half-duplex ARQ already survives via RTO. It
    // must never be nearest-mapped onto the window: a fabricated cumulative ack fires
    // on_send_complete(true) for undelivered frames, which irreversibly pops the
    // FileTransfer tx ledger (F116 silently lost bytes 34944..38688 to a stale-audio
    // 50 ms re-decode that CRC-12-fluked into group_seq6=5 and nearest-mapped to
    // base=69 across a 6-frame in-flight window).
    const uint16_t ref = static_cast<uint16_t>((tx_base_seq_ - 1) & 0xFFFF);
    const uint16_t span =
        static_cast<uint16_t>((tx_next_seq_ - tx_base_seq_) & 0xFFFF);  // seqs outstanding
    const uint16_t delta =
        static_cast<uint16_t>((group_seq6 - (ref & 0x3F)) & 0x3F);  // forward distance from ref
    if (delta > span) {
        stats_.fabricated_acks_dropped++;
        LOG_MODEM(WARN,
                  "SR-ARQ: TONE-BURST ack group_seq6=%u decodes OUTSIDE the sent window "
                  "(base-1=%u, span=%u, delta=%u) — dropped as corrupt/aliased/foreign",
                  group_seq6, ref, span, delta);
        return false;
    }

    if (move_epoch_enabled_ && (move_epoch & 0x3u) != tx_epoch_) {
        stats_.stale_epoch_acks_ignored++;
        LOG_MODEM(WARN,
                  "SR-ARQ: TONE-BURST ack epoch=%u does not match tx_epoch=%u "
                  "— dropped before synthetic SACK",
                  static_cast<unsigned>(move_epoch & 0x3u),
                  static_cast<unsigned>(tx_epoch_));
        return false;
    }
    const uint16_t remaining = static_cast<uint16_t>(span - delta);
    const uint32_t allowed_bitmap =
        remaining >= 32 ? 0xFFFFFFFFu
                        : (remaining == 0 ? 0u : ((1u << remaining) - 1u));
    if ((bitmap & ~allowed_bitmap) != 0) {
        stats_.fabricated_acks_dropped++;
        LOG_MODEM(WARN,
                  "SR-ARQ: TONE-BURST ack bitmap=0x%08X claims unsent frames "
                  "(group_seq6=%u remaining_sent=%u allowed=0x%08X) — dropped",
                  bitmap, static_cast<unsigned>(group_seq6),
                  static_cast<unsigned>(remaining), allowed_bitmap);
        return false;
    }
    const uint16_t base = static_cast<uint16_t>((ref + delta) & 0xFFFF);

    // MOVE-EPOCH: fold the tone-burst payload's epoch echo into bitmap bits 16-17
    // (the tone frame_mask is 16 bits, so those bits are free) so handleAckFrame
    // extracts/gates it uniformly with SACK control frames. No-op when OFF.
    if (move_epoch_enabled_) {
        bitmap = (bitmap & 0xFFFFu) |
                 (static_cast<uint32_t>(move_epoch & 0x3u) << 16);
    }

    // Drive the standard ack path so selective-repeat (cumulative advance + hole
    // detection + retransmit + send-complete) behaves identically to a SACK frame.
    auto ack = v2::ControlFrame::makeNack(local_call_, remote_call_, base, bitmap);
    ack.type = v2::FrameType::ACK;
    LOG_MODEM(INFO,
              "SR-ARQ: TONE-BURST ack RX group_seq6=%u -> base=%d (retires thru %d) bitmap=0x%08X",
              group_seq6, base, base, bitmap);
    return handleAckFrame(ack);
}

size_t SelectiveRepeatARQ::countUnackedFrameIdentities(
    const std::vector<Bytes>& frames) const {
    return static_cast<size_t>(std::count_if(
        frames.begin(), frames.end(), [this](const Bytes& frame) {
            return matchingLiveTXSlot(frame).has_value();
        }));
}

void SelectiveRepeatARQ::endGroupReceiveAndAck() {
    group_ack_deferred_ = false;
    // MOVE-EPOCH unanchored interregnum: total ack silence (see sendSack) — skip the
    // trigger counter too so the "trigger counters sum to sacks_sent" invariant holds.
    if (move_epoch_enabled_ && rx_epoch_wait_rebase_) {
        LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH unanchored — group ack suppressed");
        return;
    }
    // EXACTLY ONE tone-burst ack for the whole received burst: cumulative base + hole
    // bitmap (sendSack snapshots rx_base_seq_/buildRXBitmap). Emit UNCONDITIONALLY —
    // even an all-duplicate group re-confirms the receiver's window state so a sender
    // retransmit can never deadlock waiting for an ack it will never get. One keyup per
    // received burst = the half-duplex-correct cadence.
    stats_.sack_trigger_threshold++;
    sendSack();
    sack_pending_ = false;
    sack_timer_ms_ = 0;
    frames_since_ack_ = 0;
}

void SelectiveRepeatARQ::deferPendingRetransmits(uint32_t ms) {
    // §RETX-PACING §1.3 trigger #2 (docs/RETX_PACING_DESIGN_2026_07_03.md): extend every
    // pending slot's RTO by the trough-pacing hold, so tick() cannot blind-fire a timeout
    // batch around the Connection-level hold. Resending LATER than RTO is always
    // protocol-legal (the RTO is a lower bound on when a resend is permitted, not a
    // deadline owed to the peer); the dangerous direction — resending while the ACK is
    // still in flight — is the one this moves strictly away from.
    if (ms == 0) {
        return;
    }
    size_t deferred = 0;
    for (size_t i = 0; i < config_.window_size; i++) {
        size_t slot = seqToSlot((tx_base_seq_ + i) & 0xFFFF);
        TXSlot& s = tx_window_[slot];
        if (!s.active || s.acked) {
            continue;
        }
        s.timeout_ms = (s.timeout_ms > UINT32_MAX - ms) ? UINT32_MAX : s.timeout_ms + ms;
        ++deferred;
    }
    if (deferred > 0) {
        LOG_MODEM(WARN,
                  "SR-ARQ: TROUGH-PACING deferred %zu pending retransmit timer(s) by %ums",
                  deferred, ms);
    }
}

size_t SelectiveRepeatARQ::retransmitInFlightUnacked(size_t max_frames) {
    if (max_frames == 0) {
        return 0;
    }
    // Snapshot the holes first (retransmitFrame can drop a frame on max_retries, which
    // shifts the window — don't iterate the window while mutating it).
    std::vector<size_t> holes;
    for (size_t i = 0; i < config_.window_size && holes.size() < max_frames; i++) {
        size_t slot = seqToSlot((tx_base_seq_ + i) & 0xFFFF);
        const TXSlot& s = tx_window_[slot];
        if (s.active && !s.acked) {
            holes.push_back(slot);
        }
    }
    size_t resent = 0;
    for (size_t slot : holes) {
        // Re-check: a prior fail-out could have cleared/advanced this slot.
        if (tx_window_[slot].active && !tx_window_[slot].acked) {
            // NACK cause: this is a turn-driven (ack-revealed) hole resend, not a blind
            // timeout. retransmitFrame resets the slot's timeout and routes the frame
            // through transmitData → the connection buffers it into the open burst group.
            const int failures_before = stats_.failed;
            retransmitFrame(slot, RetransmitCause::NACK);
            if (stats_.failed != failures_before) {
                // A terminal failure is a logical-transfer boundary for Connection.
                // Stop this turn-driven batch immediately so later snapshotted holes
                // do not consume retries or enter the open physical burst after the
                // failure callback has requested an unwind.
                break;
            }
            if (tx_window_[slot].active) {  // still active => actually resent (not failed out)
                ++resent;
            }
        }
    }
    return resent;
}

size_t SelectiveRepeatARQ::bufferedRxFrameCount() const {
    size_t count = 0;
    for (size_t i = 0; i < config_.window_size; i++) {
        size_t slot = seqToSlot((rx_base_seq_ + i) & 0xFFFF);
        if (rx_window_[slot].received) {
            ++count;  // out-of-order buffered (a hole below blocks in-order delivery)
        }
    }
    return count;
}

bool SelectiveRepeatARQ::keepaliveAckIfStalled(uint32_t threshold_ms) {
    if (!(rx_base_seq_ > 0 && last_rx_more_data_ &&
          keepalive_silent_ms_ >= threshold_ms)) {
        return false;
    }
    LOG_MODEM(INFO,
              "SR-ARQ: keepalive ACK — receiver silent %u ms during active RX "
              "(base=%u, stall recovery)",
              keepalive_silent_ms_, rx_base_seq_);
    sendSack();  // re-emit current cumulative state; resets keepalive_silent_ms_
    return true;
}

void SelectiveRepeatARQ::sendSack() {
    // MOVE-EPOCH: TOTAL ack silence while unanchored — rx_base_seq_ is still an
    // old-era number, so ANY cumulative claim built from it would fabricate
    // delivery of new-era seqs (the W16 phantom-retire). The sender's RTO brings
    // the EPOCH_REBASE era base, which anchors us and lifts the silence. Single
    // choke point: covers the tone-burst branch, the SACK-frame branch, delayed/
    // out-of-window/partial-triggered sends and endGroupReceiveAndAck alike.
    if (move_epoch_enabled_ && rx_epoch_wait_rebase_) {
        LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH unanchored — SACK suppressed (awaiting rebase)");
        return;
    }

    keepalive_silent_ms_ = 0;  // we are emitting an ACK now → not silent

    uint32_t bitmap = buildRXBitmap();
    uint16_t base_seq = (rx_base_seq_ - 1) & 0xFFFF;
    const bool sack_has_final = rx_final_delivered_since_sack_;

    // TRANSPORT MERGE (step 1): emit the ack as a fast tone-burst instead of a SACK
    // control frame when the feature is wired. The tone-burst carries base_seq (low 6
    // bits) + the low bits of the RX bitmap up to the wire mask width (16 as of the
    // 2026-07-02 widen; the Connection truncates to kPayloadFrameMaskBits) — enough to
    // selectively ack the full capped in-flight window (kToneBurstAckWindowCapFrames).
    if (toneBurstSackTransportReady()) {
        stats_.sacks_sent++;
        stats_.acks_sent++;
        rx_final_delivered_since_sack_ = false;
        last_sack_base_valid_ = true;
        last_sack_base_ = base_seq;
        // base= is the WIRE value = highest-received (rx_base-1); next= is the
        // window's next-expected. Print BOTH — the mixed convention (adoption
        // logs print next-expected) read as a lost frame in F144/F145 forensics.
        LOG_MODEM(INFO,
                  "SR-ARQ: Sent TONE-BURST ack base=%d (next=%u) bitmap=0x%08X final=%d",
                  base_seq, rx_base_seq_, bitmap, sack_has_final ? 1 : 0);
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_ack_tx_toneburst"
                << " local=" << local_call_ << " remote=" << remote_call_
                << " ack_seq=" << base_seq
                << " bitmap=0x" << std::hex << bitmap << std::dec
                << " final=" << boolDigit(sack_has_final) << " rx_base=" << rx_base_seq_;
            ultra::phyDiagLine(oss.str());
        }
        // MOVE-EPOCH echo rides a dedicated callback argument (the Connection
        // truncates `bitmap` to the 16-bit wire frame_mask, so in-band bits would
        // be lost); always 0 while the knob is OFF.
        on_emit_tone_burst_sack_(base_seq, bitmap, sack_has_final,
                                 move_epoch_enabled_ ? rx_epoch_ : 0);
        return;
    }

    const bool turn_requested = should_request_turn_ && should_request_turn_();

    // MOVE-EPOCH echo on the SACK control frame: bits 16-17 of the bitmap word
    // (window bitmap occupies bits 0-15; bits 24-31 must stay 0 for the
    // decodeSackBitmap legacy-8-bit shim). Zero when the knob is OFF —
    // byte-identical.
    const uint32_t wire_bitmap =
        move_epoch_enabled_
            ? (bitmap | (static_cast<uint32_t>(rx_epoch_ & 0x3u) << 16))
            : bitmap;

    // Use NACK with bitmap as SACK
    auto sack = v2::ControlFrame::makeNack(local_call_, remote_call_,
                                            base_seq,
                                            wire_bitmap);
    // Override type to ACK for cumulative ack behavior
    sack.type = v2::FrameType::ACK;
    if (sack_has_final) {
        sack.flags |= v2::Flags::FINAL;
    }
    if (turn_requested) {
        sack.flags |= v2::Flags::TURN_REQUEST;
    }

    stats_.sacks_sent++;
    stats_.acks_sent++;

    auto data = sack.serialize();
    rx_final_delivered_since_sack_ = false;

    LOG_MODEM(INFO, "SR-ARQ: Sent SACK base=%d bitmap=0x%08X turn_request=%d",
              base_seq, bitmap, turn_requested ? 1 : 0);
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_ack_tx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " ack_seq=" << base_seq
            << " bitmap=0x" << std::hex << bitmap << std::dec
            << " final=" << boolDigit(sack_has_final)
            << " turn_request=" << boolDigit(turn_requested)
            << " rx_base=" << rx_base_seq_
            << " frames_since_ack=" << frames_since_ack_
            << " repeat_count=" << ack_repeat_count_;
        ultra::phyDiagLine(oss.str());
    }

    transmitData(data);

    last_sack_base_valid_ = true;
    last_sack_base_ = base_seq;
    bool repeat_ack = ack_repeat_count_ > 1;

    // Coalesce pending repeats:
    // - Keep queued repeats matching current ACK state (base+bitmap).
    // - Drop queued repeats for superseded ACK states.
    bool have_copy_queued[4] = {false, false, false, false};
    size_t removed_jobs = 0;
    for (auto it = ack_repeat_jobs_.begin(); it != ack_repeat_jobs_.end();) {
        if (it->base_seq == base_seq && it->bitmap == bitmap) {
            if (it->copy_index >= 0 && it->copy_index < 4) {
                have_copy_queued[it->copy_index] = true;
            }
            ++it;
        } else {
            it = ack_repeat_jobs_.erase(it);
            removed_jobs++;
        }
    }
    if (removed_jobs > 0) {
        stats_.ack_repeat_jobs_coalesced += static_cast<int>(removed_jobs);
        LOG_MODEM(INFO, "SR-ARQ: ACK_REPEAT coalesced %zu queued jobs", removed_jobs);
    }

    // Schedule delayed repeats for any ACK state when the connection profile
    // requests ACK diversity. Fading tests showed the dominant loss is often a
    // plain cumulative ACK (bitmap=0), especially tail ACKs; repeating only
    // selective SACKs leaves those losses unprotected. Superseded ACK states
    // were coalesced above, and the sender-side stale/duplicate guards make
    // late repeats benign.
    for (int copy_index = 2; repeat_ack && copy_index <= ack_repeat_count_; ++copy_index) {
        if (copy_index < 4 && have_copy_queued[copy_index]) {
            continue;
        }

        const uint32_t base_delay_ms = ackRepeatDelayForCopy(copy_index);
        // Clean non-final cumulative ACK repeats are diversity copies for a
        // state after which the peer may immediately start its next data burst.
        // In half-duplex audio/radio paths, sending those copies immediately can
        // deafen the receiver to that burst. Hole-bearing SACK repeats remain
        // prompt because they are repair feedback, not just ACK diversity.
        const bool guard_half_duplex_repeat = (bitmap == 0) && !sack_has_final;
        const uint32_t peer_burst_guard_ms = getAckRepeatPeerBurstGuardMs();
        uint32_t delay_ms = arq_policy::ackRepeatDelayWithHalfDuplexGuard(
            base_delay_ms, peer_burst_guard_ms, guard_half_duplex_repeat);
        int jitter_ms = ackRepeatJitterMs(base_seq, bitmap, copy_index);

        int64_t scheduled = static_cast<int64_t>(delay_ms) + jitter_ms;
        if (scheduled < 1) {
            scheduled = 1;
        }

        if (ack_repeat_jobs_.size() >= 16) {
            LOG_MODEM(WARN, "SR-ARQ: ACK_REPEAT queue full, dropping oldest pending repeat");
            ack_repeat_jobs_.pop_front();
            stats_.ack_repeat_jobs_dropped++;
        }

        AckRepeatJob job;
        job.frame_data = data;
        job.base_seq = base_seq;
        job.bitmap = bitmap;
        job.timer_ms = static_cast<uint32_t>(scheduled);
        job.copy_index = copy_index;
        ack_repeat_jobs_.push_back(std::move(job));

        LOG_MODEM(INFO, "SR-ARQ: ACK_REPEAT scheduled copy=%d delay=%ums jitter=%dms peer_guard=%ums enabled=%d queue=%zu",
                  copy_index, static_cast<uint32_t>(scheduled), jitter_ms,
                  peer_burst_guard_ms, repeat_ack ? 1 : 0, ack_repeat_jobs_.size());
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "event=arq_ack_repeat_schedule"
                << " local=" << local_call_
                << " remote=" << remote_call_
                << " ack_seq=" << base_seq
                << " bitmap=0x" << std::hex << bitmap << std::dec
                << " copy=" << copy_index
                << " delay_ms=" << static_cast<uint32_t>(scheduled)
                << " queue=" << ack_repeat_jobs_.size()
                << " final=" << boolDigit(sack_has_final);
            ultra::phyDiagLine(oss.str());
        }
    }
}

uint32_t SelectiveRepeatARQ::currentAckTimeoutMs() const {
    if (adaptive_ack_timeout_ms_ > 0) {
        return adaptive_ack_timeout_ms_;
    }
    return config_.ack_timeout_ms;
}

std::optional<size_t> SelectiveRepeatARQ::matchingLiveTXSlot(
    const Bytes& frame_data) const {
    const auto header = v2::parseHeader(frame_data);
    if (!header.valid || header.is_control) {
        return std::nullopt;
    }
    const size_t slot_index = seqToSlot(header.seq);
    const auto& slot = tx_window_[slot_index];
    // Full serialized identity protects against a wrapped/reused seq and against
    // interleave padding that happens to carry an addressable sequence number.
    if (!slot.active || slot.acked || slot.seq != header.seq ||
        slot.frame_data != frame_data) {
        return std::nullopt;
    }
    return slot_index;
}

SelectiveRepeatARQ::DeferredTimeoutCommitResult
SelectiveRepeatARQ::commitDeferredTimeoutRetransmits(
    const std::vector<Bytes>& intents) {
    DeferredTimeoutCommitResult result;
    std::vector<size_t> slots;
    slots.reserve(intents.size());
    std::array<bool, MAX_WINDOW> seen{};
    for (const auto& frame_data : intents) {
        const auto slot_index = matchingLiveTXSlot(frame_data);
        if (!slot_index || seen[*slot_index]) {
            continue;
        }
        seen[*slot_index] = true;
        slots.push_back(*slot_index);
    }

    // Preflight the WHOLE physical batch before mutating any member. The old path
    // could consume retries on early slots, discover a terminal slot later in the
    // scan, then suppress the whole batch. More importantly, terminal callbacks are
    // irreversible and cannot be rolled back if a same-tick mode transition wins.
    for (size_t slot_index : slots) {
        const auto& slot = tx_window_[slot_index];
        if (slot.retry_count + 1 >= config_.max_retries) {
            const int failures_before = stats_.failed;
            performRetransmitFrame(slot_index, RetransmitCause::TIMEOUT, nullptr);
            result.terminal_failure = stats_.failed != failures_before;
            return result;
        }
    }

    result.frames.reserve(slots.size());
    for (size_t slot_index : slots) {
        performRetransmitFrame(slot_index, RetransmitCause::TIMEOUT,
                               &result.frames);
    }
    return result;
}

size_t SelectiveRepeatARQ::cancelDeferredTimeoutRetransmits(
    const std::vector<Bytes>& intents, bool suspend_until_mode_resolution) {
    size_t canceled = 0;
    std::array<bool, MAX_WINDOW> seen{};
    for (const auto& frame_data : intents) {
        const auto slot_index = matchingLiveTXSlot(frame_data);
        if (!slot_index || seen[*slot_index]) {
            continue;
        }
        seen[*slot_index] = true;
        auto& slot = tx_window_[*slot_index];
        if (suspend_until_mode_resolution) {
            slot.timeout_suspended = true;
            slot.timeout_transition_suspended = true;
        } else {
            slot.timeout_ms = std::max<uint32_t>(currentAckTimeoutMs(), 1u);
            slot.timeout_suspended = false;
            slot.timeout_transition_suspended = false;
        }
        ++canceled;
    }
    return canceled;
}

size_t SelectiveRepeatARQ::resumeDeferredTimeoutRetransmits(uint32_t timeout_ms) {
    const uint32_t resume_ms = std::max<uint32_t>(timeout_ms, 1u);
    size_t resumed = 0;
    for (auto& slot : tx_window_) {
        if (!slot.active || slot.acked || !slot.timeout_transition_suspended) {
            continue;
        }
        slot.timeout_ms = resume_ms;
        slot.timeout_suspended = false;
        slot.timeout_transition_suspended = false;
        ++resumed;
    }
    if (resumed > 0) {
        LOG_MODEM(INFO,
                  "SR-ARQ: Resumed %zu mode-transition timeout intent(s) at %ums",
                  resumed, resume_ms);
    }
    return resumed;
}

size_t SelectiveRepeatARQ::rearmTransmittedDataFrames(
    const std::vector<Bytes>& frames, uint32_t timeout_ms) {
    const uint32_t explicit_timeout_ms = std::max<uint32_t>(timeout_ms, 1u);
    size_t rearmed = 0;
    std::array<bool, MAX_WINDOW> transmitted_slots{};
    for (const auto& frame_data : frames) {
        const auto header = v2::parseHeader(frame_data);
        if (!header.valid || header.is_control) {
            continue;
        }

        const size_t slot_index = seqToSlot(header.seq);
        auto& slot = tx_window_[slot_index];
        // Match the complete serialized identity as well as seq. This prevents an
        // interleave pad or a wrapped/stale seq alias from extending a different slot.
        if (slot.active && !slot.acked && slot.seq == header.seq &&
            slot.frame_data == frame_data) {
            slot.timeout_ms = explicit_timeout_ms;
            slot.timeout_suspended = false;
            slot.timeout_transition_suspended = false;
            transmitted_slots[slot_index] = true;
            ++rearmed;
        }
    }

    // A unified physical round is stop-and-wait. If a changed group cap leaves other
    // live holes outside this transmission, freeze their old near-expiry timers: they
    // have not been retransmitted yet and must not launch a second burst underneath the
    // current round. The next [holes]+[new] refill unsuspends each identity as it is
    // actually emitted. Do nothing when no identity matched (defensive: padding or a
    // non-slot repair must not accidentally suspend the whole sender).
    size_t suspended = 0;
    if (rearmed > 0) {
        // Scan physical storage, not [0, configured_window): seqToSlot uses the
        // fixed MAX_WINDOW ring, so a nonzero/wrapped base (or a recent window shrink)
        // can leave valid live slots at any physical index.
        for (size_t i = 0; i < MAX_WINDOW; ++i) {
            auto& slot = tx_window_[i];
            if (!slot.active || slot.acked || transmitted_slots[i]) {
                continue;
            }
            slot.timeout_suspended = true;
            ++suspended;
        }
    }

    LOG_MODEM(DEBUG,
              "SR-ARQ: committed transmitted-frame timeout=%ums to %zu/%zu frame(s); "
              "suspended=%zu later hole(s)",
              explicit_timeout_ms, rearmed, frames.size(), suspended);
    return rearmed;
}

// ULTRA_INFLIGHT_RTO: the timeout for the burst CURRENTLY outstanding. See
// selective_repeat_arq_policy.hpp::inflightRtoEnabled for why a scalar is wrong at the
// tail of a transfer. Falls through to the scalar when the knob is off, when no table has
// been supplied (MC-DPSK / narrow / control paths), or for counts outside the table.
uint32_t SelectiveRepeatARQ::ackTimeoutForFrames(size_t frames) const {
    if (!arq_policy::inflightRtoEnabled() || ack_timeout_table_n_ == 0) {
        return currentAckTimeoutMs();
    }
    const size_t idx = std::clamp<size_t>(frames, 1, ack_timeout_table_n_ - 1);
    const uint32_t t = ack_timeout_table_[idx];
    if (t == 0) return currentAckTimeoutMs();
    // Never exceed the scalar: the table is a REDUCTION for small bursts, never a licence
    // to wait longer than the configured bound.
    return std::min(t, currentAckTimeoutMs());
}

// Re-arm every outstanding slot to the timeout for the burst as it now stands. Called as
// frames are queued: slot 1 of a 5-frame burst is seeded when in_flight==1, but it is
// waiting for the SAME group ACK as slot 5, so it must be re-armed upward as the burst
// grows. Without this, early frames in a full burst would time out mid-burst — the
// spurious-retransmission failure this knob must not introduce.
void SelectiveRepeatARQ::rearmOutstandingTimeouts() {
    if (!arq_policy::inflightRtoEnabled() || ack_timeout_table_n_ == 0) return;
    const uint32_t t = ackTimeoutForFrames(tx_in_flight_);
    for (auto& s : tx_window_) {
        if (s.active && !s.acked && s.timeout_ms > 0 && s.timeout_ms < t) {
            s.timeout_ms = t;
        }
    }
}

void SelectiveRepeatARQ::maybeSampleRTT(TXSlot& slot) {
    if (!slot.rtt_sample_eligible) {
        return;
    }
    if (arq_time_ms_ < slot.first_tx_ms) {
        return;
    }

    uint32_t sample_ms = arq_policy::rttSampleMs(arq_time_ms_, slot.first_tx_ms);
    slot.rtt_sample_eligible = false;
    if (!arq_policy::shouldUseRTTSample(
            true, arq_time_ms_, slot.first_tx_ms)) {
        return;
    }

    // RFC6298-style estimator (Karn-safe: retransmitted slots are marked ineligible).
    const auto rto = arq_policy::updateRTO(
        have_rtt_estimator_, srtt_ms_, rttvar_ms_, sample_ms, config_.ack_timeout_ms,
        arq_policy::adaptiveRtoEnabled());
    srtt_ms_ = rto.srtt_ms;
    rttvar_ms_ = rto.rttvar_ms;
    adaptive_ack_timeout_ms_ = rto.rto_ms;
    have_rtt_estimator_ = true;

    LOG_MODEM(DEBUG, "SR-ARQ: RTT sample=%ums srtt=%.1f rttvar=%.1f rto=%ums",
              sample_ms, srtt_ms_, rttvar_ms_, adaptive_ack_timeout_ms_);
    // NULL CONTROL (2026-07-30). A knob that provably did not engage proves nothing, and
    // this one is easy to leave inert: the burst/file path must actually reach
    // maybeSampleRTT for the adaptive floor to matter at all. Log at INFO, once per
    // change, so a rig run can be checked for engagement instead of assumed. Prints the
    // legacy value alongside so the delta is visible without a second run.
    if (arq_policy::adaptiveRtoEnabled() && adaptive_ack_timeout_ms_ != last_logged_rto_ms_) {
        last_logged_rto_ms_ = adaptive_ack_timeout_ms_;
        const auto legacy = arq_policy::updateRTO(
            have_rtt_estimator_, srtt_ms_, rttvar_ms_, sample_ms, config_.ack_timeout_ms,
            /*adaptive_floor=*/false);
        LOG_MODEM(INFO,
                  "SR-ARQ: ADAPTIVE-RTO ENGAGED rto=%ums (legacy would be %ums, "
                  "configured=%ums) srtt=%.0f rttvar=%.0f sample=%ums",
                  adaptive_ack_timeout_ms_, legacy.rto_ms, config_.ack_timeout_ms,
                  srtt_ms_, rttvar_ms_, sample_ms);
    }
}

void SelectiveRepeatARQ::sendFrameNack(uint16_t seq) {
    auto nack = v2::ControlFrame::makeNack(local_call_, remote_call_, seq, 0);
    auto data = nack.serialize();
    LOG_MODEM(INFO, "SR-ARQ: Sent frame NACK seq=%d", seq);
    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "event=arq_nack_tx"
            << " local=" << local_call_
            << " remote=" << remote_call_
            << " seq=" << seq
            << " missing_cw=0x00000000";
        ultra::phyDiagLine(oss.str());
    }
    transmitData(data);
}

uint32_t SelectiveRepeatARQ::ackRepeatDelayForCopy(int copy_index) const {
    return arq_policy::ackRepeatDelayForCopy(ack_repeat_delay_ms_, copy_index);
}

int SelectiveRepeatARQ::ackRepeatJitterMs(uint16_t base_seq, uint32_t bitmap, int copy_index) const {
    return arq_policy::ackRepeatJitterMs(base_seq, bitmap, copy_index);
}

uint32_t SelectiveRepeatARQ::buildRXBitmap() const {
    uint32_t bitmap = 0;

    for (int i = 0; i < 32 && i < static_cast<int>(config_.window_size); i++) {
        size_t slot = seqToSlot((rx_base_seq_ + i) & 0xFFFF);
        if (rx_window_[slot].received) {
            bitmap |= (1u << i);
        }
    }

    return bitmap;
}

std::vector<uint16_t> SelectiveRepeatARQ::predictedIncomingSeqs(size_t max_n) const {
    std::vector<uint16_t> out;
    out.reserve(std::min(max_n, config_.window_size));
    for (size_t i = 0; i < config_.window_size && out.size() < max_n; i++) {
        const uint16_t seq = (rx_base_seq_ + static_cast<uint16_t>(i)) & 0xFFFF;
        if (!rx_window_[seqToSlot(seq)].received) {
            out.push_back(seq);
        }
    }
    return out;
}

size_t SelectiveRepeatARQ::seqToSlot(uint16_t seq) const {
    return seq % MAX_WINDOW;
}

bool SelectiveRepeatARQ::isInTXWindow(uint16_t seq) const {
    return arq_policy::seqInWindow(seq, tx_base_seq_, config_.window_size);
}

bool SelectiveRepeatARQ::isInRXWindow(uint16_t seq) const {
    return arq_policy::seqInWindow(seq, rx_base_seq_, config_.window_size);
}

void SelectiveRepeatARQ::transmitData(const Bytes& data) {
    if (on_transmit_) {
        on_transmit_(data);
    }
}

void SelectiveRepeatARQ::transmitDataBatch(const std::vector<Bytes>& frames) {
    if (frames.empty()) {
        return;
    }
    // This function is the timeout-round commit boundary. Let Connection stage even a
    // singleton so a same-tick collapse escape cannot queue one obsolete full-anchor
    // resend before switching geometry.
    if (on_transmit_batch_) {
        on_transmit_batch_(frames);
        return;
    }
    for (const auto& frame : frames) {
        transmitData(frame);
    }
}

void SelectiveRepeatARQ::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void SelectiveRepeatARQ::setTransmitBatchCallback(TransmitBatchCallback cb) {
    on_transmit_batch_ = std::move(cb);
}

void SelectiveRepeatARQ::setDataReceivedCallback(DataReceivedCallback cb) {
    on_data_received_ = std::move(cb);
}

void SelectiveRepeatARQ::setSendCompleteCallback(SendCompleteCallback cb) {
    on_send_complete_ = std::move(cb);
}

void SelectiveRepeatARQ::expireBaseSlotTimerForRebase() {
    const size_t idx = seqToSlot(tx_base_seq_);
    auto& slot = tx_window_[idx];
    if (!slot.active || slot.acked || slot.seq != tx_base_seq_) {
        return;  // no live base frame — nothing to re-anchor with
    }
    // Only pull the timer in when it is not already imminent: the receiver's
    // fading-aware ACK repeats deliver up to 3 voice copies ~seconds apart, and
    // each group event re-voices — without this floor every copy would re-fire
    // a resend. 2 s ≈ the shortest control turnaround; an already-due timer
    // means the machinery is about to act anyway.
    if (slot.timeout_suspended || slot.timeout_ms > 2000) {
        LOG_MODEM(WARN,
                  "SR-ARQ: WAITING-REBASE voice — expiring base slot seq=%u timer "
                  "(was %u ms) for a standalone era-base resend",
                  slot.seq, slot.timeout_ms);
        slot.timeout_ms = 1;
        slot.timeout_suspended = false;
        slot.timeout_transition_suspended = false;
    }
}

void SelectiveRepeatARQ::abortPendingTx(bool force_move_epoch_bump) {
    // MOVE-EPOCH (2026-07-04 fix, Phase-2 review finding): detect whether this abort
    // actually drops live payload BEFORE clearing the slots. The 2026-07-03 design
    // assumed forward abandonment (tx_base_seq_ = tx_next_seq_ below) needs no epoch
    // because seqs are never RE-USED — true, but blind to the second collision arm:
    // with a mid-window regrid (Phase-2 receiver-commanded demote at the SAME code
    // rate, where setCodeRate early-returns and only the CW-change abort fires) the
    // RECEIVER's in-order rx_base is left below the abandoned seqs with no
    // rebase-anchor — an unfillable hole = the BUG-ARQ-SEQ-COLLISION stall by another
    // door. Any abort that drops live payload is a seq↔payload remap and must enter
    // a new era; the first post-abort frame (at the advanced base) then carries the
    // bumped epoch + EPOCH_REBASE and the receiver re-anchors.
    bool dropped_payload = force_move_epoch_bump || tx_in_flight_ > 0;
    for (const auto& slot : tx_window_) {
        if (slot.active && !slot.acked) {
            dropped_payload = true;
            break;
        }
    }
    for (auto& slot : tx_window_) {
        // F163 FIX-4 (same salvage as setCodeRate): SACKed = peer-confirmed.
        if (slot.active && slot.acked && on_sacked_frame_discarded_ &&
            !slot.frame_data.empty()) {
            on_sacked_frame_discarded_(slot.frame_data);
        }
        slot.active = false;
        slot.acked = false;
        slot.frame_data.clear();
        slot.fixed_frame_codewords = fixed_frame_codewords_;
        slot.timeout_ms = 0;
        slot.timeout_suspended = false;
        slot.timeout_transition_suspended = false;
        slot.first_tx_ms = 0;
        slot.rtt_sample_eligible = false;
        slot.retry_count = 0;
        slot.hole_ack_count = 0;
        slot.fast_retx_count = 0;
        slot.fast_retx_cooldown_ms = 0;
        slot.hole_probe_armed = false;
        slot.hole_probe_timer_ms = 0;
        slot.hole_probe_count = 0;
        clearTXSlotRepairState(slot);
    }

    tx_base_seq_ = tx_next_seq_;
    tx_in_flight_ = 0;

    if (move_epoch_enabled_ && dropped_payload) {
        tx_epoch_ = static_cast<uint8_t>((tx_epoch_ + 1) & 0x3);
        LOG_MODEM(WARN, "SR-ARQ: MOVE-EPOCH bumped to %u on pending-TX abort%s",
                  tx_epoch_, force_move_epoch_bump ? " (terminal failure)" : " (regrid)");
    }

    // Cancel pending control TX from ARQ side as well.
    sack_pending_ = false;
    sack_timer_ms_ = 0;
    frames_since_ack_ = 0;
    ack_repeat_jobs_.clear();
    ack_dedup_timer_ms_ = 0;

    LOG_MODEM(INFO, "SR-ARQ: Aborted pending TX state");
}

void SelectiveRepeatARQ::clearPendingAckRepeats() {
    if (!ack_repeat_jobs_.empty()) {
        LOG_MODEM(INFO, "SR-ARQ: Cleared %zu turn-scoped ACK repeat(s)", ack_repeat_jobs_.size());
    }
    ack_repeat_jobs_.clear();
}

void SelectiveRepeatARQ::reset() {
    for (auto& slot : tx_window_) {
        slot.active = false;
        slot.acked = false;
        slot.frame_data.clear();
        slot.fixed_frame_codewords = fixed_frame_codewords_;
        slot.first_tx_ms = 0;
        slot.timeout_suspended = false;
        slot.timeout_transition_suspended = false;
        slot.rtt_sample_eligible = false;
        slot.hole_ack_count = 0;
        slot.fast_retx_count = 0;
        slot.fast_retx_cooldown_ms = 0;
        slot.hole_probe_armed = false;
        slot.hole_probe_timer_ms = 0;
        slot.hole_probe_count = 0;
        clearTXSlotRepairState(slot);
    }
    tx_base_seq_ = 0;
    tx_next_seq_ = 0;
    tx_in_flight_ = 0;

    for (auto& slot : rx_window_) {
        slot.received = false;
        clearPartialRXSlot(slot);
        slot.payload.clear();
        slot.flags = 0;
        slot.type = v2::FrameType::DATA;
    }
    rx_base_seq_ = 0;

    last_rx_more_data_ = false;
    last_rx_flags_ = 0;
    last_rx_frame_type_ = v2::FrameType::DATA;
    rx_final_delivered_since_sack_ = false;

    sack_pending_ = false;
    sack_timer_ms_ = 0;
    frames_since_ack_ = 0;

    ack_repeat_jobs_.clear();
    last_sack_base_valid_ = false;
    last_sack_base_ = 0;
    last_ack_signature_valid_ = false;
    last_ack_seq_ = 0;
    last_ack_bitmap_ = 0;
    ack_dedup_timer_ms_ = 0;
    last_ack_progress_frames_ = -1;
    arq_time_ms_ = 0;
    have_rtt_estimator_ = false;
    srtt_ms_ = 0.0f;
    rttvar_ms_ = 0.0f;
    adaptive_ack_timeout_ms_ = config_.ack_timeout_ms;

    // MOVE-EPOCH: a reset is a new session — both stations restart at era 0,
    // anchored (matches seq counters restarting at 0).
    tx_epoch_ = 0;
    rx_epoch_ = 0;
    rx_epoch_wait_rebase_ = false;

    LOG_MODEM(DEBUG, "SR-ARQ: Reset");
}

} // namespace protocol
} // namespace ultra
