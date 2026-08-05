// Connection state machine - core logic
// Frame handlers are in connection_handlers.cpp

#include <chrono>
#include "connection.hpp"
#include "connection_policy.hpp"
#include "waveform_selection.hpp"
#include "ultra/logging.hpp"
#include <algorithm>
#include <bit>
#include <cctype>
#include <limits>
#include <filesystem>

namespace ultra {
namespace protocol {

namespace {
constexpr size_t kOFDMFileBlockPayloadLimit = 2300;
constexpr size_t kMaxQueuedPayloads = 32;

// TRANSPORT MERGE (step 1): opt-in tone-burst ACK on the interactive SR-ARQ path.
constexpr uint32_t kInteractiveToneAckWindowMs = 8000;  // floor: monitor arm window for the ack
// TRANSPORT MERGE (2026-06-06): the unified arq_ path is now THE OFDM file/message
// transport — one 16-bit seq space, one tone-burst ack, one retransmit window. The
// file still bursts+interleaves via sendNextFileChunk()->flushBurstBuffer(); arq_ owns
// sequencing/dedup/retransmit and the RX delivers through processArqFrame. The legacy
// burst_transport_ group controller (BurstStopAndWaitController) is removed — there is
// exactly ONE group-generation path. These two helpers are now unconditionally true
// (kept as named call sites during the cleanup; inline + delete in a follow-up).
// Proof: /tmp/unified_multiseed.sh — AWGN R3/4+R2/3, Good R2/3 CRC-clean, legacy_calls=0.
bool kInteractiveToneAckEnabled() { return true; }
bool kUnifiedSeqEnabled() { return true; }

// Experimental long-code profile for the coherent 8PSK R2/3 file rung.  Keep
// this read fresh so a test (or a new transfer in the same process) can compare
// the two wire geometries without restarting the application.  The profile is
// deliberately exact-"1" and default-off.
bool psk8LongLdpcExperimentEnabled() {
    const char* e = std::getenv("ULTRA_8PSK_LONG_LDPC");
    return e && e[0] == '1' && e[1] == '\0';
}

// Experimental long-code profile for the production QPSK R3/4 file rung.
// The normal logical geometry remains cw8/Z27; an eligible file transfer is
// represented physically as cw3/Z81 and announced by BURST_HEADER. Unlike the
// 8PSK profile this is not an equal-airtime substitution (5832 versus 5184 coded
// bits), so it is a measurement-only, exact-"1", default-off experiment.
bool qpskR34LongLdpcExperimentEnabled() {
    const char* e = std::getenv("ULTRA_QPSK_R34_LONG_LDPC");
    return e && e[0] == '1' && e[1] == '\0';
}

// ULTRA_RX_EMA_HOLD (default-OFF; =1 opts in): lever #1 of the throughput-ceiling audit.
// REVERTED to default-off 2026-07-23 — the honest all-levers-ON vs baseline reckoning
// (F580-591, MPG@20) showed the four throughput levers as an AGGREGATE WASH (delivered
// mean +11% but ALL-ON failed 2/6 vs baseline 1/6 → effective throughput ~tied/slightly
// negative), and this "hold" lever contributes to over-holding a failing rung (the 88s
// crater stalls). Was briefly default-ON 2026-07-21 on the F500-511 A/B (which read +8.5%,
// mostly a floor win); that gain did not survive the aggregate reckoning. Kept behind the
// knob for measurement. The REAL cap is the rung controller over-committing to 16QAM +
// demoting too slowly — a rung-SELECTION fix, not more holding. Two coupled corrections to
// the RX-authority rung controller's crater response, targeting the fast-vs-slow variance
// (F344 2.61 vs F372 1.16 kbps on the SAME MPG@20 channel — a rate-controller limit cycle):
// (1) EMA-SUPPORTED HOLD — a confirmed crater does NOT demote while
// the fade-averaged broadband SNR still clears the CURRENT rung's calibrated floor
// (rungClassAnchorDb); consecutive craters at a healthy average are deep-null fade
// brushes the ARQ absorbs, not rung failure. (2) CENSORED failed-group SNR — a
// cratered group feeds a sample right-censored at the rung floor into the obs ring,
// killing the survivor bias (a fully-cratered group otherwise re-feeds a STALE CREST
// read; a partial crater's fresh read is itself crest-biased) that let the ring read
// 26-32 dB on a 20 dB channel and re-clear the climb bar after every demote. (2)
// makes (1) honest: the hold gate cannot latch on a crest-biased average.
bool emaHoldEnabled() {
    const char* e = std::getenv("ULTRA_RX_EMA_HOLD");
    return e && e[0] == '1' && e[1] == '\0';  // default-OFF (reverted 2026-07-23); =1 opts in
}

// ULTRA_DENSE_FAST_DEMOTE (default-off; =1 opts in): the RX-authority over-commit fix
// (2026-07-23). The predictive climb makes AGGRESSIVE direct multi-rung jumps INTO 16QAM
// (idx 3→8 on 2.5 dB EESM margin over crest-biased snapshots), but the demote is
// deliberately SLOW (F122 two-crater rule + one-rung steps, anti-oscillation). Aggressive-up
// + slow-down = the measured 82s crater stall when the jump is wrong (channel can't hold
// 16QAM). The two-crater grace is CALIBRATED FOR ROBUST RUNGS: a single crater on QPSK's
// WIDE margin is an ARQ-absorbed deep null, not rung failure. But 16QAM's TIGHT rings make a
// FULL crater (0/N) much more likely genuine over-commit — and the cost is asymmetric (82s of
// craters vs one rung down, quickly re-climbable when the EESM re-proves it). So on a
// dense-mod rung (bits/symbol ≥ 4), a FULL crater demotes IMMEDIATELY (streak ≥ 1) instead of
// waiting for two. Modulation-adaptive by construction; QPSK/8PSK keep the grace. Targets
// FULL craters only (frame_mask==0) — partial 16QAM now resends cheaply via per-frame SACK
// (interleave-off), so it keeps the two-crater grace. The demote's penalty ratchet still
// gates re-climb (no oscillation). Leaves the CLIMB untouched.
bool denseFastDemoteEnabled() {
    const char* e = std::getenv("ULTRA_DENSE_FAST_DEMOTE");
    return e && e[0] == '1' && e[1] == '\0';
}

// ULTRA_EVM_DEMOTE (default-off; =1 opts in): radio-agnostic EVM demote authority
// (Stage 2, 2026-07-24). The RX-authority ladder is steered by the OFDM broadband
// SNR estimator, which is marked up by the +8.70 dB kOfdmLegacyAnchorScaleOffsetDb
// quarantine to the dial/channel scale the legacy anchors assume — on a real radio
// that offset MASKS the hardware loss (~5 dB on the IONOS bench) and over-commits the
// rung. The decision-directed EVM SNR (demodulator_impl.hpp) reads USABLE dB directly,
// constant-free and non-inflating. When this group's measured usable EVM cannot support
// the CURRENT rung's EVM-usable floor (evmUsableFloorDbForRung), clamp the command DOWN
// to the highest rung the EVM CAN support. Demote-only: it can never raise the command
// (EVM saturates at the demod ceiling, so it is not a climb input) — it only strips an
// over-commit the broadband estimator's optimism let through. Orthogonal to the crater/
// penalty machinery: a pure output clamp so the A/B reads its effect cleanly.
bool evmDemoteEnabled() {
    const char* e = std::getenv("ULTRA_EVM_DEMOTE");
    return e && e[0] == '1' && e[1] == '\0';
}

// ULTRA_CRATER_GOODPUT_GRADE (2026-07-25; DEFAULT ON since 2026-07-26, opt out with =0):
// grade the crater predicate by DELIVERED FRACTION against the goodput break-even instead
// of the binary quality byte.
//
// The decoder assigns quality EXACTLY 0.0 to any group that is not perfect
// (streaming_burst_interleave), so `quality <= 0` means "at least one frame of N was
// lost" — NOT "the rung failed". Measured on the MPG@20 rig (3 transfers, 64 groups):
// 21 groups fed crater evidence, but only 6 were true 0/N craters; the other 15 (71%)
// delivered a MEAN of 67% of their frames, including 7/8 (88%) and 4/5 (80%) groups
// counted identically to total loss. Two such groups in a row ratchet the rung down one
// step, and since climbing needs clean groups plus cooldowns, the fast-down/slow-up
// asymmetry walks a healthy link to the floor: run 2 spent t=48..127 s ratcheting
// idx 4 -> 1 while snr_avg ROSE to 22 dB and the ladder's own pick was idx 8, then
// delivered 3111/4842 bps once it climbed back. That first quartile ran at 54% of the
// rest of the transfer = 19% of wall clock.
//
// Treating "not perfect" as rung failure also contradicts the project's own physics
// doctrine: fading loss is irreducible and ARQ is mandatory, so a group whose losses
// the ARQ resends more cheaply than a whole rung demote is the system WORKING.
// goodputBreakEvenDeliveredFraction() is the geometry-derived line between the two.
//
// DEFAULT-ON RATIONALE (rig A/B, 16 interleaved runs, 8/arm, 2026-07-26). This is a
// CORRECTNESS fix before it is a throughput lever: "7/8 frames delivered" is simply not
// evidence that a rung failed, so the binary predicate is wrong independent of any measured
// delta. The A/B supports it on six independent metrics, with the honest numbers:
//   completion            8/8 vs 5/8      (one-sided p = 0.100)
//   completed-run goodput 1689 vs 1554 bps = +8.7%
//   pathological demotes  1.57 vs 2.88 /run = -45%   <- the DIRECT causal metric
//   craters               1.0 vs 2.1 /run  (the pre-committed falsifier passed INVERTED)
//   rung changes          5.4 vs 8.0 /run
//   sign test on 8 pairs  6 pos / 1 neg / 1 tie, p = 0.0625, median +29.8%
// No single metric reaches p < 0.05 on a rig with documented +/-25% epoch noise, so the case
// rests on the convergence plus the derivation. Two earlier headline figures were ARTIFACTS
// and are not the effect size: +56% came from timeout draws (a timed-out run scores ~830 bps
// vs 1380-2170 completed, so any pair drawing one timeout reads +70..110%), and an ON run with
// regrades == 0 is byte-identical to OFF -- such a NULL CONTROL landed at 1590 bps, +2.3% vs
// the OFF mean, which is the harness's internal-validity check.
// "Pathological demote" = q == 0 && cmd < cur && raw > cmd && fading <= kFadingGoodMax: a
// demote BELOW what the ladder itself picked, on a channel the fading index calls GOOD.
bool craterGoodputGradeEnabled() {
    const char* e = std::getenv("ULTRA_CRATER_GOODPUT_GRADE");
    return !(e && e[0] == '0' && e[1] == '\0');
}

Modulation wideOFDMControlModulationForData(Modulation data_modulation) {
    return ofdm_link_adaptation::isCoherentModulation(data_modulation)
        ? Modulation::QPSK
        : Modulation::DQPSK;
}

uint32_t ackRepeatDelayForControlAirtimeMs(uint32_t control_airtime_ms) {
    return control_airtime_ms +
           selective_repeat_arq_policy::kAckRepeatMaxJitterMs +
           connection_policy::kCarrierSenseSackCoalesceMs;
}

Bytes makeOFDMBurstPadPayload(CodeRate rate, int cw_count, size_t pad_index,
                              int lifting_z = 27) {
    const size_t capacity = (lifting_z == 81)
        ? v2::getFixedFramePayloadCapacityZ(rate, cw_count, 81)
        : v2::getFixedFramePayloadCapacity(rate, cw_count);
    Bytes payload(capacity);
    if (payload.empty()) {
        return payload;
    }

    // Fill the dummy frame instead of sending an empty DATA payload. Empty pad
    // frames leave most fixed-frame info bits as all-zero LDPC codewords, which
    // are prone to ugly "4/4 CWs OK but frame invalid" tail artifacts in fading.
    uint32_t x = 0xA5C35A7Du ^
                 (static_cast<uint32_t>(rate) << 24) ^
                 static_cast<uint32_t>(pad_index * 0x9E3779B1u);
    for (size_t i = 0; i < payload.size(); ++i) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        payload[i] = static_cast<uint8_t>(x & 0xFF);
    }
    payload[0] = 0x7F;  // reserved dummy discriminator if a hash collision ever delivers it
    return payload;
}

bool shouldUseSingleOFDMFileBlock(float fading_index, float snr_db, CodeRate rate) {
    const auto* descriptor = ofdmCodeRateDescriptor(rate);
    const auto* single_block_floor = ofdmCodeRateDescriptor(CodeRate::R2_3);
    return connection_policy::isNearAwgnOFDM(fading_index, snr_db) &&
           descriptor != nullptr && single_block_floor != nullptr &&
           descriptor->code_rate >= single_block_floor->code_rate;
}

bool shouldPadPartialOFDMBurst(WaveformMode mode,
                               Modulation modulation,
                               CodeRate rate,
                               float fading_index,
                               float snr_db,
                               FileTransferState file_state,
                               size_t burst_frames) {
    if (!isOFDMMode(mode)) {
        return false;
    }
    if (file_state != FileTransferState::SENDING) {
        return connection_policy::isHighThroughputOFDMMode(modulation, rate) &&
               connection_policy::shouldPadBurstInterleaveGroup(burst_frames);
    }

    return connection_policy::shouldPadHighRateFadingBurst(
        modulation,
        rate,
        connection_policy::isNearAwgnOFDM(fading_index, snr_db),
        burst_frames);
}

bool isFinalDataFrame(const Bytes& frame_data) {
    auto hdr = v2::parseHeader(frame_data);
    return hdr.valid &&
           hdr.type == v2::FrameType::DATA &&
           frame_data.size() > 3 &&
           ((frame_data[3] & v2::Flags::FINAL) != 0);
}

bool seqBefore(uint16_t a, uint16_t b) {
    return a != b && static_cast<uint16_t>(b - a) < 0x8000;
}

float modeEfficiency(Modulation mod, CodeRate rate) {
    return estimateWideOFDMRawBps(mod, rate);
}

bool isFasterMode(Modulation candidate_mod, CodeRate candidate_rate,
                  Modulation current_mod, CodeRate current_rate) {
    return modeEfficiency(candidate_mod, candidate_rate) >
           modeEfficiency(current_mod, current_rate) + 0.05f;
}

bool isNormalArqAckFrame(const Bytes& frame_data) {
    if (frame_data.size() < v2::ControlFrame::SIZE ||
        static_cast<v2::FrameType>(frame_data[2]) != v2::FrameType::ACK) {
        return false;
    }

    const uint16_t seq =
        (static_cast<uint16_t>(frame_data[4]) << 8) | frame_data[5];
    return seq != 0xFFFF;
}

bool expectsFullOFDMAnchorAfterTx(const Bytes& frame_data) {
    const auto header = v2::parseHeader(frame_data);
    if (header.valid && header.seq == v2::DISCONNECT_SEQ &&
        (header.type == v2::FrameType::DISCONNECT ||
         header.type == v2::FrameType::ACK)) {
        // Teardown is a cold, control-only half-duplex transaction.  Both the
        // request and every sentinel ACK are full-anchored, so explicitly arm
        // the matching acquisition contract instead of inheriting whatever
        // expectation happened to survive the final DATA turn.
        return true;
    }
    if (isNormalArqAckFrame(frame_data)) {
        return true;
    }
    if (frame_data.size() < v2::ControlFrame::SIZE) {
        return false;
    }

    const auto type = static_cast<v2::FrameType>(frame_data[2]);
    return type == v2::FrameType::TURNOVER ||
           type == v2::FrameType::TURN_REQUEST ||
           type == v2::FrameType::FILE_CANCEL;
}

}

const char* connectionStateToString(ConnectionState state) {
    switch (state) {
        case ConnectionState::DISCONNECTED:  return "DISCONNECTED";
        case ConnectionState::PROBING:       return "PROBING";
        case ConnectionState::CONNECTING:    return "CONNECTING";
        case ConnectionState::CONNECTED:     return "CONNECTED";
        case ConnectionState::DISCONNECTING: return "DISCONNECTING";
        default: return "UNKNOWN";
    }
}

// =============================================================================
// CONSTRUCTOR
// =============================================================================

Connection::Connection(const ConnectionConfig& config)
    : config_(config)
    , arq_(config.arq)
{
    data_frame_cw_count_ = v2::sanitizeFixedFrameCodewords(config_.fixed_frame_codewords);
    config_.fixed_frame_codewords = data_frame_cw_count_;
    arq_.setFixedFrameGeometry(data_frame_cw_count_, /*lifting_z=*/27);

    // §14.27: burst transport is THE OFDM-wideband file path — UNCONDITIONAL, no env
    // gate (2026-06-02; the ULTRA_BURST_TRANSPORT opt-out was removed — burst is the
    // only valid file method now). `use_burst_transport_` stays initialized true; the
    // legacy windowed-file `!use_burst_transport_` branches are now dead code (R1
    // deletion follow-up). NOTE: burst is itself selective-repeat (GROUP_ACK carries
    // the 16-bit SACK frame_mask) — SelectiveRepeatARQ (`arq_`) still serves MC-DPSK/
    // narrow/control; this is NOT "remove SR-ARQ".
    // §14.36 Phase 5c: per-block decode-headroom quality feedback. Default ON (drives the GUI
    // "Adapt:" bar + diagnostics on sim AND hardware); opt OUT with ULTRA_ADAPTIVE_RATE=0
    // (which also disables the rate-change path). The actual rate CHANGE is separately gated by
    // rateAdaptationActive() — default-ON for connected wideband OFDM since 2026-07-02
    // (fade-riding ladder); ULTRA_RATE_ADAPT=0 / ULTRA_LOCK_RATE=1 opt out.
    if (const char* ar = std::getenv("ULTRA_ADAPTIVE_RATE"); ar && ar[0] == '0') {
        adaptive_rate_enabled_ = false;
    }

    // DESC-SWITCH Phase 1 (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.1, knob
    // ULTRA_DESCRIPTOR_MODE_SWITCH, read once per Connection like ULTRA_ARQ_MOVE_EPOCH;
    // default OFF = byte-identical): clean-boundary wideband-OFDM ladder moves commit
    // LOCALLY and ride the next BURST_HEADER descriptor instead of the MODE_CHANGE
    // stop-and-wait exchange. SEMANTICS-BREAKING lockstep when ON (both ends must be
    // built + enabled; same increment policy as move-epoch/tone-payload).
    if (const char* ds = std::getenv("ULTRA_DESCRIPTOR_MODE_SWITCH"); !(ds && ds[0] == '0')) {  // DEFAULT-ON 2026-07-05
        descriptor_mode_switch_enabled_ = true;
    }

    // RX-RATE-CMD Phase 2 (design §5.2, knob ULTRA_RX_RATE_CMD, read once per
    // Connection; default ON since 2026-07-05). The tone-ACK codec must use the
    // widened CRC span whenever this command path or RX authority is enabled.
    // SEMANTICS-BREAKING lockstep when ON; the descriptor-committed consume path
    // additionally needs ULTRA_DESCRIPTOR_MODE_SWITCH (+ ULTRA_ARQ_MOVE_EPOCH for
    // mid-window) — it falls back to the legacy MODE_CHANGE exchange without them.
    if (const char* rc = std::getenv("ULTRA_RX_RATE_CMD"); !(rc && rc[0] == '0')) {  // DEFAULT-ON 2026-07-05 (voice; demote-cmd inert under authority)
        rx_rate_cmd_enabled_ = true;
    }

    // Progress-bearing partial-SACK descriptor-only repair experiment.  Strict
    // default OFF and read per Connection so two local test peers can independently
    // prove the mixed-version behavior.  Enabling only the receiver is wire-safe:
    // reserved drive-advisory value 3 is explicitly HOLD to existing senders.
    if (const char* pr = std::getenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");
        pr && pr[0] == '1' && pr[1] == '\0') {
        partial_sack_descriptor_repair_enabled_ = true;
        LOG_MODEM(WARN,
                  "Connection: EXPERIMENT partial-SACK descriptor-only repair enabled");
    }

    // Wire up ARQ callbacks
    arq_.setTransmitCallback([this](const Bytes& data) {
        if (disconnect_teardown_active_) {
            LOG_MODEM(DEBUG,
                      "Connection: Suppressing ARQ egress during disconnect teardown");
            return;
        }
        const auto header = v2::parseHeader(data);
        const bool expects_data_ack = header.valid && !header.is_control;
        if (deferred_arq_failure_abort_ && expects_data_ack) {
            LOG_MODEM(WARN,
                      "Connection: Dropping DATA egress requested after terminal ARQ failure");
            return;
        }
        // A buffered physical group is committed and armed once at flush.  This covers
        // wide/narrow OFDM and MC-DPSK; restricting it to OFDM made MC arm once per
        // logical frame and then again for the same key-down at flush.
        const bool unified_group_is_buffering =
            burst_mode_active_ && static_cast<bool>(on_transmit_burst_) &&
            kUnifiedSeqEnabled();
        uint32_t explicit_timeout_ms = 0;
        const bool is_slot_data =
            expects_data_ack && header.type != v2::FrameType::DATA_REPAIR;
        if (!unified_group_is_buffering && is_slot_data) {
            // A descriptor-less singleton cannot carry exact group k/M provenance
            // for the descriptor-only repair experiment.
            partial_sack_last_round_ = {};
            const uint32_t queue_delay_ms = noteDataBurstKeydown({data});
            explicit_timeout_ms =
                finalizeUnifiedBurstWindow({data}, queue_delay_ms);
        }

        // Arm before handing the frame to the host: transport callbacks are allowed to
        // synchronously loop an ACK back into the protocol. The modeled timeout already
        // includes the small encode/queue latency.
        if (!unified_group_is_buffering && expects_data_ack) {
            armToneBurstAckListenWindow(explicit_timeout_ms);
        }
        transmitFrame(data);
        // A buffered group is finalized once, after its exact frame+padding count is
        // known. A standalone new send/NACK repair/RTO resend is finalized above and
        // armed once at the transport handoff boundary. DATA_REPAIR keeps its existing
        // repair-guard/scalar timer but still arms; control frames expect no DATA ACK.
    });

    arq_.setDataReceivedCallback([this](const Bytes& data) {
        handleDataPayload(data, arq_.lastRxHadMoreData(), arq_.lastRxFrameType(),
                          arq_.lastRxFlags());
    });

    arq_.setReceiveWindowAdvancedCallback([this](uint16_t base_seq, size_t window_size) {
        soft_combine_harq_.retainOnlySeqWindow(base_seq, window_size);
    });
    arq_.setTxFrameSubmittedCallback([this](uint16_t seq) {
        handleArqFrameSubmitted(seq);
        // ACK-monitor arming belongs to the physical egress callbacks above/below, not
        // logical submission: only egress knows the exact group size and queue edge.
    });
    arq_.setTxBaseAdvancedCallback([this](uint16_t base_seq) {
        handleArqTxBaseAdvanced(base_seq);
    });
    arq_.setTxFrameFailedCallback([this](uint16_t seq) {
        handleArqFrameFailed(seq);
    });
    arq_.setTurnRequestCallback([this]() {
        return noteTurnRequestOnAckIfNeeded();
    });
    // F163 FIX-4: a rate/CW-change abort discards SACKed (peer-confirmed) TX
    // slots — salvage their FILE byte-ranges so the requeue does not re-send
    // bytes the receiver already holds (13 confirmed frames re-sent in F163).
    // F218 COMPLETION GATE: the ARQ is the completion ground truth.
    file_transfer_.setCompletionGate(
        [this]() { return arq_.getTxInFlightBytes() == 0; });
    // Keep the transfer-scoped PHY re-grid internal and unconditional. The
    // application callback is optional, but the next message/file must always
    // return to the negotiated logical/Z27 geometry after an experiment ends.
    file_transfer_.setSentCallback(
        [this](bool success, const std::string& error) {
            ladderTelemetryFinish(success);
            setExperimentalLongLDPCTransferProfilesActive(false, false);
            if (on_file_sent_) {
                on_file_sent_(success, error);
            }
        });
    arq_.setSackedFrameDiscardedCallback([this](const Bytes& frame_data) {
        // F181 (BUG-SACK-DURABILITY-RESIDUAL): the skip trusts slot.acked, and
        // F181 proved a SACK mark can be era-corrupted — the sender skipped
        // re-sending [0,456) (the file's FIRST chunk) on a mark for a frame
        // the receiver PROVABLY never had (prefix pinned at 0, receiver was
        // ack-silent/UNANCHORED that whole era) — permanent hole, stranded
        // file. The optimization is worth ~1 s/transfer; the failure costs the
        // run. Default OFF until the phantom-mark chain is root-caused
        // (ULTRA_SACK_SALVAGE=1 re-enables for measurement).
        static const bool kSackSalvage = []() {
            const char* e = std::getenv("ULTRA_SACK_SALVAGE");
            return e && e[0] == '1' && e[1] == '\0';
        }();
        if (!kSackSalvage) {
            return;  // re-send SACKed ranges; receiver dedups by offset
        }
        auto frame = v2::DataFrame::deserialize(frame_data);
        if (!frame || frame->payload.size() <= FileTransferController::FILE_DATA_OVERHEAD) {
            return;
        }
        if (frame->payload[0] != static_cast<uint8_t>(PayloadType::FILE_DATA)) {
            return;  // FILE_START/TEXT: metadata or seq-deduped — never salvage
        }
        const uint32_t offset = (static_cast<uint32_t>(frame->payload[1]) << 24) |
                                (static_cast<uint32_t>(frame->payload[2]) << 16) |
                                (static_cast<uint32_t>(frame->payload[3]) << 8) |
                                static_cast<uint32_t>(frame->payload[4]);
        const uint32_t len = static_cast<uint32_t>(
            frame->payload.size() - FileTransferController::FILE_DATA_OVERHEAD);
        file_transfer_.noteRangeDelivered(offset, len);
    });

    // TRANSPORT MERGE (step 1, env ULTRA_TONE_ACK_INTERACTIVE): route the interactive
    // SACK through the same tone-burst transport the burst path uses. When enabled, the
    // receiver emits the ack as a tone-burst (low-6 base_seq + the RX bitmap truncated
    // to the 16-bit wire mask) instead of a SACK control frame; the sender arms its
    // monitor (above) and consumes it in onToneBurstAck(). Default off — the legacy
    // SACK-frame path is unchanged.
    if (kInteractiveToneAckEnabled()) {
        arq_.setEmitToneBurstSackCallback(
            [this](uint16_t base_seq, uint32_t bitmap, bool has_final,
                   uint8_t move_epoch) {
                if (disconnect_teardown_active_ || !on_transmit_tone_burst_ack_) {
                    return;
                }
                // The startup UP command is valid only on the synchronous ACK of a
                // wire-proven completed group.  Any delayed/keepalive/timer SACK has no
                // outcome provenance and must fail closed before its bits are serialized.
                // This single stamp-site invariant also covers marker abandonment paths
                // that intentionally produce no immediate group callback.
                if (latent_startup_probe_waiting_ &&
                    !tone_ack_group_complete_context_) {
                    failClosedLatentStartupProbeUnknown(
                        "asynchronous tone-SACK without completed-group provenance");
                }
                ultra::waveform::tone_burst_ack::ToneBurstAckPayload tba;
                // frame_mask width tracks the wire layout (16 bits as of 2026-07-02) so the
                // SACK can address a 16-frame in-flight window — see kToneBurstAckWindowCapFrames.
                constexpr uint32_t kFrameMaskWire =
                    (1u << ultra::waveform::tone_burst_ack::kPayloadFrameMaskBits) - 1u;
                tba.group_seq = static_cast<uint8_t>(base_seq & 0x3F);
                tba.frame_mask = static_cast<uint16_t>(bitmap & kFrameMaskWire);
                tba.type = ultra::waveform::tone_burst_ack::AckType::Ack;
                // MOVE-EPOCH echo (ULTRA_ARQ_MOVE_EPOCH, BUG-ARQ-SEQ-COLLISION):
                // payload bits 40-41; always 0 while the knob is OFF (byte-identical).
                tba.move_epoch = move_epoch;
                // RX-RATE-CMD Phase 2 (ULTRA_RX_RATE_CMD): payload bits 42-43 — the
                // standing receiver rung command computed per group event in
                // updateRxRateCommandFromGroup (crater-only DOWN-hard). Only ever
                // non-zero when the knob is ON (byte-identical OFF); re-emitted ACK
                // copies re-carry the same command, the sender dedups by group_seq.
                tba.rung_cmd = rx_rate_cmd_pending_;
                // §14.43: carry the receiver's last measured group decode headroom [0,1] back to
                // the sender, quantized into the 3-bit rate_hint (0..7). The sender de-quantizes it
                // in onToneBurstAck and feeds its RateController. -1 (no sample yet) -> 0.
                tba.rate_hint = (last_group_quality_ >= 0.0f)
                    ? static_cast<uint8_t>(std::lround(
                          std::clamp(last_group_quality_, 0.0f, 1.0f) * 7.0f))
                    : 0;
                // RX-AUTHORITY (ULTRA_RX_RATE_AUTHORITY): reinterpret the SAME five
                // bits [rate_hint(3)|rung_cmd(2)] as the receiver's ABSOLUTE canonical
                // rung command (waveform_selection.hpp kRungIdx*; 0 = no command).
                // Overrides the hint/relative-cmd stamps above — under authority the
                // sender's EMA has no consumer for the hint, and the demote-only
                // relative command is superseded by the absolute one. The WAITING-
                // REBASE voice is untouched (type=NACK path, emitted elsewhere).
                if (rxRateAuthorityEnabled()) {
                    // A logical FINAL retires the peer's transfer.  Advertising a NEW
                    // rung in this ACK cannot accelerate any remaining payload, but it
                    // can make the sender pay a standalone MODE_CHANGE after its file
                    // has already drained.  Preserve the proven current rung; a future
                    // transfer will produce fresh evidence before changing it.
                    if (has_final) {
                        const uint8_t current = coherentRungIndexFor(
                            data_modulation_, data_code_rate_);
                        if (current != kRungIdxNone && rx_authority_cmd_ != current) {
                            LOG_MODEM(INFO,
                                      "Connection: RX-AUTHORITY FINAL hold idx %u "
                                      "(discarding tail-only idx %u command)",
                                      current, rx_authority_cmd_);
                            rx_authority_cmd_ = current;
                        }
                        if (latent_startup_probe_waiting_) {
                            latent_startup_probe_waiting_ = false;
                            latent_startup_probe_spent_ = true;
                            latent_startup_probe_clean_groups_ = 0;
                            latent_startup_probe_pending_base_groups_ = 0;
                            latent_startup_probe_rollback_pending_ = false;
                            LOG_MODEM(INFO,
                                      "Connection: LATENT startup probe suppressed at "
                                      "logical FINAL (no next physical group)");
                        }
                    }
                    uint8_t wire_authority_cmd = rx_authority_cmd_;
                    if (!has_final && tone_ack_group_complete_context_ &&
                        latent_startup_probe_waiting_ &&
                        rx_authority_cmd_ == kRungIdxQpskR23) {
                        wire_authority_cmd |= kRungAuthorityStartupProbeFlag;
                    }
                    tba.rate_hint = static_cast<uint8_t>(wire_authority_cmd & 0x7);
                    tba.rung_cmd = static_cast<uint8_t>((wire_authority_cmd >> 3) & 0x3);
                }
                // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET): stamp the drive advisory
                // from the per-burst RX level verdict fed via setRxLevelVerdict just
                // before this group's delivery. Down IMMEDIATELY on a clip signature
                // (fast attack); up only after kAlcLowStreakForUp consecutive fresh
                // LOW verdicts (fade hysteresis, slow release). ULTRA_SOFTWARE_ALC=0
                // pins the advisory to hold (the receiver advisory LOG still runs in
                // the decoder). Cached RF repeats retain the already-rendered
                // advisory; later timer/classic/backstop SACKs have no current-group
                // level provenance and therefore stay HOLD.
                if (connection_policy::softwareAlcEnabled() &&
                    tone_ack_alc_group_context_) {
                    if (rx_level_clipped_) {
                        tba.drive_advisory =
                            ultra::waveform::tone_burst_ack::kDriveAdvisoryDown;
                    } else if (rx_level_low_streak_ >=
                                   connection_policy::kAlcLowStreakForUp &&
                               tone_ack_group_has_decoded_data_context_) {
                        // ALC RUNAWAY GUARD (2026-07-04, F18 forensics): a LOW level
                        // reading is drive evidence ONLY while at least one CRC-valid,
                        // locally-addressed frame from THIS physical group decoded at
                        // that level (genuinely level-starved but workable chain).
                        // Aggregate `quality` cannot provide that provenance: the burst
                        // decoder deliberately assigns exactly 0.0 to every partial
                        // group, including useful 2/8..7/8 deliveries. A LOW reading on
                        // a zero-delivery group is a FADE TROUGH -- the ladder owns
                        // fades; drive must not chase them. Without this gate every
                        // trough ratcheted the peer's tx_drive up (0.63 -> the 0.85 cap
                        // by t=171), TX compression on the cheap card then cratered the
                        // thin-margin rungs at 30+ dB readings, and no RX clip signature
                        // ever brought the drive back down.
                        tba.drive_advisory =
                            ultra::waveform::tone_burst_ack::kDriveAdvisoryUp;
                    }
                }
                // The 2-bit reserved advisory is CRC-covered and already specified as
                // HOLD at old senders. Reuse it only when this synchronous callback is
                // causally bracketed by a decoder-proven complete physical group, and
                // only when ALC has no real UP/DOWN command to carry. No-group timer
                // backstops, classic-tail recovery, clipping, and low-level ALC all
                // therefore fail closed to the established full repair anchor.
                if (partial_sack_descriptor_repair_enabled_ &&
                    tone_ack_group_complete_context_ &&
                    tone_ack_exact_group_geometry_context_ &&
                    tba.drive_advisory ==
                        ultra::waveform::tone_burst_ack::kDriveAdvisoryHold) {
                    tba.drive_advisory =
                        ultra::waveform::tone_burst_ack::kDriveAdvisoryReserved;
                    LOG_MODEM(INFO,
                              "Connection: PARTIAL-SACK exact-group provenance "
                              "stamped in hold-compatible advisory=3");
                }
                on_transmit_tone_burst_ack_(
                    tba, tone_ack_group_complete_context_);
            });
        // The protocol callback is installed during construction, before the GUI/TNC
        // supplies its physical tone transmitter. Until then, retain the legacy ACK/NACK
        // plane instead of silently suppressing feedback into an unwired callback.
        arq_.setToneBurstSackTransportReady(false);
    }

    arq_.setSendCompleteCallback([this](bool success) {
        if (deferred_arq_failure_abort_) {
            // A terminal slot failure owns the whole logical transfer. ARQ may now
            // advance across a contiguous SACKed suffix, which normally emits one
            // success completion per retired slot. Suppress every completion until
            // the post-unwind drain publishes exactly one terminal failure.
            LOG_MODEM(DEBUG,
                      "Connection: Suppressed ARQ completion success=%d during "
                      "terminal-failure unwind",
                      success ? 1 : 0);
            return;
        }
        if (file_transfer_.getState() == FileTransferState::SENDING) {
            if (success) {
                file_transfer_.onChunkAcked();
                if (arq_callback_defer_refill_) {
                    deferred_file_refill_ = true;
                } else {
                    sendNextFileChunk();
                }
            } else {
                file_transfer_.onSendFailed();
            }
        } else if (!pending_tx_fragments_.empty()) {
            if (!success) {
                // Fragment send failed - abort remaining fragments
                LOG_MODEM(WARN, "Connection: Fragment send failed, aborting remaining %zu fragments",
                          pending_tx_fragments_.size() - next_fragment_idx_);
                pending_tx_fragments_.clear();
                pending_tx_fragment_flags_.clear();
                pending_tx_fragment_types_.clear();
                pending_tx_fragment_message_tokens_.clear();
                next_fragment_idx_ = 0;
                acked_fragment_count_ = 0;
                if (on_message_sent_) {
                    on_message_sent_(false);
                }
            } else {
                acked_fragment_count_++;

                if (next_fragment_idx_ < pending_tx_fragments_.size()) {
                    // More fragments to submit to ARQ
                    if (arq_callback_defer_refill_) {
                        deferred_fragment_refill_ = true;
                    } else {
                        sendNextFragment();
                    }
                }

                if (acked_fragment_count_ >= pending_tx_fragments_.size()) {
                    // ALL fragments truly ACKed
                    LOG_MODEM(INFO, "Connection: All %zu fragments sent and ACKed",
                              pending_tx_fragments_.size());
                    pending_tx_fragments_.clear();
                    pending_tx_fragment_flags_.clear();
                    pending_tx_fragment_types_.clear();
                    pending_tx_fragment_message_tokens_.clear();
                    next_fragment_idx_ = 0;
                    acked_fragment_count_ = 0;
                    if (on_message_sent_) {
                        on_message_sent_(true);
                    }
                }
            }
        } else {
            if (on_message_sent_) {
                on_message_sent_(success);
            }
        }
    });
}

Connection::~Connection() {
    // FileTransferController::~FileTransferController() calls cancel(). Its
    // controller-owned callbacks capture this Connection, but callback members
    // declared after file_transfer_ are destroyed before file_transfer_ during
    // implicit member teardown. Clear those callbacks while every Connection
    // member is still alive, then retire transfer state silently; otherwise a
    // Connection destroyed mid-transfer can call through already-destroyed state.
    file_transfer_.setSentCallback({});
    file_transfer_.setReceivedCallback({});
    file_transfer_.cancel("Connection destroyed");
}

// =============================================================================
// CONFIGURATION
// =============================================================================

void Connection::setLocalCallsign(const std::string& call) {
    local_call_ = sanitizeCallsign(call);
}

// =============================================================================
// CONNECTION CONTROL
// =============================================================================

bool Connection::connect(const std::string& remote_call) {
    if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot connect, state=%s",
                  connectionStateToString(state_));
        return false;
    }

    if (local_call_.empty()) {
        LOG_MODEM(ERROR, "Connection: Local callsign not set");
        return false;
    }

    remote_call_ = sanitizeCallsign(remote_call);
    if (remote_call_.empty() || !isValidCallsign(remote_call_)) {
        LOG_MODEM(ERROR, "Connection: Invalid remote callsign: %s", remote_call.c_str());
        return false;
    }

    outbound_forced_modulation_ = Modulation::AUTO;
    outbound_forced_code_rate_ = CodeRate::AUTO;

    LOG_MODEM(INFO, "Connection: Connecting to %s (starting with PING probe)", remote_call_.c_str());

    // Use current connect_waveform_ (can be pre-set via setInitialConnectWaveform)
    // Notify the modem of the waveform to use
    if (on_connect_waveform_changed_) {
        on_connect_waveform_changed_(connect_waveform_);
    }

    // Start with PROBING state - send PING for fast presence check
    state_ = ConnectionState::PROBING;
    ping_retry_count_ = 0;
    timeout_remaining_ms_ = pingTimeoutMsForCurrentProfile();
    stats_.connects_initiated++;

    // Send PING (modem will generate preamble + "ULTR")
    if (on_ping_tx_) {
        LOG_MODEM(INFO, "Connection: Sending PING via %s",
                  waveformModeToString(connect_waveform_));
        on_ping_tx_();
    } else {
        // Fallback: no ping callback, send CONNECT directly
        LOG_MODEM(WARN, "Connection: No ping callback, sending CONNECT directly");
        sendFullConnect();
    }

    return true;
}

void Connection::acceptCall() {
    if (state_ != ConnectionState::DISCONNECTED || pending_remote_call_.empty()) {
        LOG_MODEM(WARN, "Connection: No pending call to accept");
        return;
    }

    remote_call_ = pending_remote_call_;
    pending_remote_call_.clear();

    negotiated_mode_ = negotiateMode(remote_capabilities_, remote_preferred_);

    // Check if initiator forced specific modes (0xFF = AUTO, else forced)
    Modulation rec_mod;
    CodeRate rec_rate;

    // Use centralized algorithm from waveform_selection.hpp. #58: the selection value
    // is basis-corrected (fade-effective reading vs dial-calibrated anchors). Increment
    // 3: snr_db is the connect-SNR-pool aggregate when ULTRA_CONNECT_SNR_POOL is set
    // (clustered dB-mean of the handshake's data-aided readings — same population the
    // +5 basis was calibrated on, so the basis composes unchanged and is applied ONCE,
    // downstream); knob-off it is exactly the raw measured_snr_db_ scalar.
    const float snr_db = rateSelectionSnrDb();
    // Increment 4: entry-pick fading is pooled like the SNR (single-frame fading
    // scatters 0.24-0.74 across the 0.65 boundary at Watterson Good); knob-off
    // this IS fading_index_, byte-identical.
    const float entry_fading_raw = rateSelectionFadingIndex();
    // Entry classification shrinkage — same rationale as handleConnect (see the
    // helper's provenance comment); knob-off keeps the raw scalar path.
    const float entry_fading =
        connection_policy::connectSnrPoolEnabled()
            ? connection_policy::entryClassificationFadingIndex(
                  entry_fading_raw,
                  connect_snr_pool_.effectiveCount(
                      connectSnrPoolTcMs(), /*handshake_only=*/true,
                      /*max_age_ms=*/UINT64_MAX))
            : entry_fading_raw;
    const bool accept_snr_data_aided = rateSelectionSnrDataAided();
    const float accept_selection_snr_db =
        connection_policy::connectSelectionSnrDb(snr_db, entry_fading,
                                                 accept_snr_data_aided,
                                                 physical_channel_mean_db_,
                                                 physical_channel_n_);
    recommendDataMode(accept_selection_snr_db, negotiated_mode_, rec_mod, rec_rate, entry_fading);

    // Bootstrap safety: the connect-time reading can overestimate first OFDM frame
    // quality (historically the chirp snapshot; since #58 it is the data-aided
    // fade-averaged estimate). ULTRA_ENTRY_CAP_R34 (default OFF) lets a data-aided
    // reading clearing the R3/4 anchor by >= 1 sigma enter at R3/4.
    if (isOFDMMode(negotiated_mode_)) {
        CodeRate capped = capInitialOFDMRate(accept_selection_snr_db, entry_fading, rec_rate, rec_mod,
                                             accept_snr_data_aided);
        if (capped != rec_rate) {
            LOG_MODEM(INFO, "Connection: Bootstrap cap %s -> %s for initial OFDM setup (SNR=%.1f (%s), fading=%.2f)",
                      codeRateToString(rec_rate), codeRateToString(capped), snr_db,
                      snrSourceToString(measured_snr_source_), entry_fading);
            rec_rate = capped;
        }
    }

    // ULTRA_ENTRY_QAM16_SNR (experiment): start AT 16QAM R2/3 on a strong Good-class
    // connect instead of QPSK-and-climb (the fade-riding strategy). AFTER the bootstrap
    // cap, BEFORE forced overrides. Mirrors the responder site in connection_handlers.cpp.
    if (isOFDMMode(negotiated_mode_) &&
        entryQam16Promote(accept_selection_snr_db, entry_fading, rec_mod,
                          accept_snr_data_aided)) {
        LOG_MODEM(INFO,
                  "Connection: ENTRY-QAM16 promote %s %s -> 16QAM R2/3 (data-aided SNR=%.1f, fading=%.2f)",
                  modulationToString(rec_mod), codeRateToString(rec_rate),
                  accept_selection_snr_db, entry_fading);
        rec_mod = Modulation::QAM16;
        rec_rate = CodeRate::R2_3;
    }

    const EnvironmentForcedDataProfile environment_force =
        forcedDataProfileFromEnvironment();
    if (environment_force.malformed) {
        LOG_MODEM(ERROR,
                  "Connection: malformed ULTRA_FORCE_DATA_MOD/RATE profile on "
                  "manual accept; ignoring the environment override as one unit");
    }
    const Modulation environment_forced_mod = environment_force.modulation;
    const CodeRate environment_forced_rate = environment_force.code_rate;
    const bool environment_forced_rung = environment_force.forced();

    // 2026-05-28 experiment (env-gated): the industry leader's tactical ladder
    // picks QPSK R2/3 (~3230 bps net) as its 3000 bps speed slot, not R3/4
    // or R5/6. Stronger FEC -> fewer drop-on-timeout cascades -> higher
    // *effective* e2e throughput. ULTRA_MAX_OFDM_RATE=R2_3 caps both initial
    // selection AND adaptive climb at R2/3 to test this hypothesis. No-op when
    // the env is unset (default ladder unchanged).
    if (const char* env = std::getenv("ULTRA_MAX_OFDM_RATE")) {
        const std::string s(env);
        const CodeRate cap = (s == "R1_2" || s == "r1_2") ? CodeRate::R1_2
                           : (s == "R2_3" || s == "r2_3") ? CodeRate::R2_3
                           : (s == "R3_4" || s == "r3_4") ? CodeRate::R3_4
                           : CodeRate::AUTO;  // anything else = no cap (AUTO sentinel)
        if (cap != CodeRate::AUTO && rec_rate > cap) {
            LOG_MODEM(INFO, "Connection: ULTRA_MAX_OFDM_RATE cap %s -> %s",
                      codeRateToString(rec_rate), codeRateToString(cap));
            rec_rate = cap;
        }
    }

    // ULTRA_FORCE_DATA_* is an exact operator profile, while MAX is only an
    // automatic-selector ceiling.  The exact profile has the same final precedence as
    // serialized CONNECT force fields below.  Re-apply only the valid forced halves so
    // a partial force retains the automatically resolved complementary field.
    if (environment_forced_mod != Modulation::AUTO) {
        rec_mod = environment_forced_mod;
    }
    if (environment_forced_rate != CodeRate::AUTO) {
        rec_rate = environment_forced_rate;
    }

    // Preserve the automatic selector's belief separately from the conservative
    // physical first-group cap.  A forced operator rung below replaces this seed.
    const uint8_t automatic_latent_seed_rung =
        (isOFDMMode(negotiated_mode_) && rxRateAuthorityEnabled())
            ? coherentRungIndexFor(rec_mod, rec_rate)
            : kRungIdxNone;

    // Keep manual-accept and auto-accept bootstrap behavior identical.  This is
    // intentionally before the forced-mode overrides below: an operator probing
    // an exact rung still gets the requested modulation/rate unchanged.
    const auto authority_entry = capRxAuthorityInitialRung(
        rec_mod, rec_rate, rxRateAuthorityEnabled(), environment_forced_rung);
    if (authority_entry.mod != rec_mod || authority_entry.rate != rec_rate) {
        LOG_MODEM(INFO,
                  "Connection: RX-AUTHORITY measured-entry probe %s %s -> QPSK R1/2 "
                  "(first coherent group owns the next decision)",
                  modulationToString(rec_mod), codeRateToString(rec_rate));
        rec_mod = authority_entry.mod;
        rec_rate = authority_entry.rate;
    }

    if (pending_forced_modulation_ != Modulation::AUTO) {
        // Initiator forced a specific modulation - honor it
        rec_mod = pending_forced_modulation_;
        LOG_MODEM(INFO, "Connection: Using FORCED modulation %s from initiator",
                  modulationToString(rec_mod));
    }

    if (pending_forced_code_rate_ != CodeRate::AUTO) {
        // Initiator forced a specific code rate - honor it
        rec_rate = pending_forced_code_rate_;
        LOG_MODEM(INFO, "Connection: Using FORCED code rate %s from initiator",
                  codeRateToString(rec_rate));
    }

    const bool forced_rung =
        environment_forced_rung ||
        pending_forced_modulation_ != Modulation::AUTO ||
        pending_forced_code_rate_ != CodeRate::AUTO;
    latent_bootstrap_rung_ =
        (isOFDMMode(negotiated_mode_) && rxRateAuthorityEnabled())
            ? (forced_rung
                   ? coherentRungIndexFor(rec_mod, rec_rate)
                   : automatic_latent_seed_rung)
            : kRungIdxNone;

    // Pick negotiated CW count (honor initiator's forced value, else auto).
    // Computed BEFORE building CONNECT_ACK so the embedded byte and the
    // initiator's view match what we'll actually use locally.
    int negotiated_cw = (pending_forced_cw_count_ != 0)
        ? v2::sanitizeFixedFrameCodewords(pending_forced_cw_count_)
        : connection_policy::recommendCWCountForChannel(
              rec_mod, rec_rate, negotiated_mode_, entry_fading, snr_db);

    // Clear pending forced modes
    pending_forced_modulation_ = Modulation::AUTO;
    pending_forced_code_rate_ = CodeRate::AUTO;
    pending_forced_cw_count_ = 0;

    LadderRungId rung_id = LadderRungId::UNKNOWN;
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        rung_id = connection_policy::rungForMCDPSKConfig(
            rec_mod, config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol, negotiated_cw).id;
    } else if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        rung_id = LadderRungId::OFDM_CHIRP;
    } else if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        rung_id = LadderRungId::OFDM_NARROW;
    }

    // 2026-05-28: ULTRA_FRAME_CW env override for reliability sweep. Allows
    // pinning cw count per frame in [1, kMaxFixedFrameCodewords]. Default unset
    // = use the negotiated value as before.
    if (const char* env = std::getenv("ULTRA_FRAME_CW")) {
        const int v = std::atoi(env);
        if (v >= v2::kMinFixedFrameCodewords && v <= v2::kMaxFixedFrameCodewords) {
            LOG_MODEM(INFO, "Connection: ULTRA_FRAME_CW override %d -> %d",
                      negotiated_cw, v);
            negotiated_cw = v;
        }
    }

    // Set our local data mode immediately.
    applyDataMode(rec_mod, rec_rate, negotiated_cw, rung_id);

    LOG_MODEM(INFO, "Connection: Accepting call from %s (waveform=%s, data=%s %s, cw=%d)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_),
              modulationToString(data_modulation_), codeRateToString(data_code_rate_),
              data_frame_cw_count_);

    // CONNECT_ACK wire bytes: the pool aggregates under ULTRA_CONNECT_SNR_POOL (always
    // fresh at this instant by construction — never the stale sentinel), else the raw
    // scalars exactly as before. SNR and fading get the SAME treatment so the
    // initiator's display shows the values that actually drove this pick.
    auto ack = v2::ConnectFrame::makeConnectAck(local_call_, remote_call_,
                                                 static_cast<uint8_t>(negotiated_mode_),
                                                 data_modulation_, data_code_rate_,
                                                 snr_db, entry_fading,
                                                 static_cast<uint8_t>(data_frame_cw_count_),
                                                 rung_id);
    if (forced_rung) {
        ack.flags |= v2::Flags::CONNECT_FORCED_PROFILE;
    }
    Bytes ack_data = ack.serialize();
    connect_ack_frame_ = ack_data;
    const uint32_t responder_handshake_failsafe_ms = responderHandshakeFailSafeMs();

    LOG_MODEM(INFO, "Connection: Sending CONNECT_ACK (%zu bytes, SNR=%.1f dB (%s))",
              ack_data.size(), snr_db, snrSourceToString(measured_snr_source_));
    transmitFrame(ack_data);

    // We are the responder - we received CONNECT and are sending CONNECT_ACK
    is_initiator_ = false;
    handshake_confirmed_ = false;  // Responder waits for first frame to confirm
    responder_handshake_wait_ms_ = responder_handshake_failsafe_ms;

    // A manual modulation/rate request is an operator experiment, not permission for
    // the automatic controller to move away from it.  Pass the gate through connected
    // entry so it is established before the application's on_connected callback.
    enterConnected(/*automatic_rate_allowed=*/!forced_rung);

    // Notify application of initial data mode (LOCAL reading — responder pick;
    // entry_fading = the pooled value that drove the pick, raw scalar knob-off)
    notifyDataModeChanged(snr_db, entry_fading, /*snr_is_wire=*/false);
}

void Connection::rejectCall() {
    if (pending_remote_call_.empty()) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Rejecting call from %s", pending_remote_call_.c_str());

    auto nak = v2::ConnectFrame::makeConnectNak(local_call_, pending_remote_call_);
    Bytes nak_data = nak.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT_NAK (%zu bytes)", nak_data.size());
    transmitFrame(nak_data);

    pending_remote_call_.clear();
}

bool Connection::isDisconnectTeardownWireFrame(const Bytes& frame_data) {
    const auto header = v2::parseHeader(frame_data);
    return header.valid && header.seq == v2::DISCONNECT_SEQ &&
           (header.type == v2::FrameType::DISCONNECT ||
            header.type == v2::FrameType::ACK);
}

bool Connection::isAllowedDisconnectTeardownRx(
    const v2::HeaderInfo& header) const {
    if (!header.valid || header.seq != v2::DISCONNECT_SEQ) {
        return false;
    }
    if (header.type == v2::FrameType::DISCONNECT) {
        return true;  // duplicate request or crossed close
    }
    return state_ == ConnectionState::DISCONNECTING &&
           header.type == v2::FrameType::ACK;
}

void Connection::setDisconnectTeardownActive(bool active) {
    if (disconnect_teardown_active_ == active) {
        return;
    }

    disconnect_teardown_active_ = active;
    if (active) {
        // Connection::tick is egress-exclusive during grace, but application and
        // decoder callbacks can arrive between ticks.  Detach any half-built DATA
        // request now; the wire gate below then makes the quarantine exhaustive.
        burst_mode_active_ = false;
        burst_tx_buffer_.clear();
        staged_timeout_batch_.clear();
        tone_ack_group_complete_context_ = false;
        rx_level_verdict_pending_for_group_ = false;
        tone_ack_alc_group_context_ = false;
        tone_ack_group_has_decoded_data_context_ = false;
        tone_ack_exact_group_geometry_context_ = false;
        deferred_file_refill_ = false;
        deferred_fragment_refill_ = false;
        partial_sack_descriptor_repair_scope_ = false;
        file_cancel_confirm_pending_ = false;
        mode_change_ack_repeat_jobs_.clear();
        LOG_MODEM(INFO,
                  "Connection: disconnect teardown ACTIVE — control-only RX and "
                  "close-only egress required");
    } else {
        LOG_MODEM(DEBUG, "Connection: disconnect teardown cleared");
    }

    if (on_disconnect_teardown_) {
        on_disconnect_teardown_(active);
    }
}

void Connection::disconnect() {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    if (state_ == ConnectionState::CONNECTING) {
        enterDisconnected("Cancelled");
        return;
    }

    if (state_ == ConnectionState::CONNECTED) {
        LOG_MODEM(INFO, "Connection: Disconnecting from %s", remote_call_.c_str());

        auto disc = v2::ControlFrame::makeDisconnect(local_call_, remote_call_);
        disconnect_frame_ = disc.serialize();

        // Publish/quarantine the close phase before handing bytes to a host callback.
        // Host transports are allowed to loop a decoded ACK back synchronously; the
        // old queue-first ordering would then see CONNECTED and misroute the sentinel
        // ACK through the DATA ARQ before DISCONNECTING was assigned.
        state_ = ConnectionState::DISCONNECTING;
        timeout_remaining_ms_ = config_.disconnect_timeout_ms;
        disconnect_retry_count_ = 0;
        disconnect_retransmit_ms_ = disconnectRetryIntervalMs();
        setDisconnectTeardownActive(true);
        stats_.disconnects++;
        LOG_MODEM(INFO, "Connection: Sending DISCONNECT (%zu bytes)", disconnect_frame_.size());
        transmitFrame(disconnect_frame_);
        if (state_ != ConnectionState::DISCONNECTING) {
            return;  // A synchronous host loopback already completed the close.
        }
        LOG_MODEM(INFO,
                  "Connection: DISCONNECT retry armed at %ums from queue "
                  "(control_keydown=%ums, timeout=%ums)",
                  disconnect_retransmit_ms_, disconnectControlKeydownMs(),
                  timeout_remaining_ms_);
    }
}

void Connection::abortTxNow() {
    const bool aborting_disconnect_teardown = disconnect_teardown_active_;
    const bool had_pending_payload =
        !queued_payloads_.empty() || !pending_tx_fragments_.empty() ||
        !outbound_message_tx_records_.empty() || arq_.getTxInFlightBytes() > 0;
    auto failed_message_records = detachOutboundMessageRecords();

    // Cancel all outbound ARQ activity (data retransmit timers, delayed ACK repeats,
    // delayed SACK, in-flight TX slots) while preserving RX reassembly state.
    arq_.abortPendingTx();

    // Cancel pending local TX assembly/burst state.
    queued_payloads_.clear();
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    pending_tx_fragment_message_tokens_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    arq_callback_defer_refill_ = false;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;
    deferred_arq_failure_abort_ = false;
    deferred_arq_failure_seq_valid_ = false;
    staged_timeout_batch_.clear();
    arq_tick_in_progress_ = false;

    // Cancel file TX if active. Keep RX file state untouched.
    queued_file_path_.reset();
    queued_file_order_ = 0;
    next_queued_operation_order_ = 1;
    if (file_transfer_.getState() == FileTransferState::SENDING) {
        file_transfer_.cancel();
    }

    // Cancel pending control-path retries/timeouts.
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    desc_switch_full_anchor_pending_ = false;
    disconnect_pending_ = false;
    disconnect_pending_ms_ = 0;
    disconnect_ack_retransmit_ms_ = 0;
    disconnect_ack_epoch_elapsed_ms_ = 0;
    disconnect_ack_repeat_count_ = 0;
    disconnect_ack_frame_.clear();
    disconnect_frame_.clear();
    disconnect_retry_count_ = 0;
    disconnect_retransmit_ms_ = 0;
    timeout_remaining_ms_ = 0;
    connect_retry_count_ = 0;
    ping_retry_count_ = 0;
    responder_handshake_wait_ms_ = 0;
    connect_ack_frame_.clear();

    // Stop transient connection attempts immediately.
    if (aborting_disconnect_teardown ||
        state_ == ConnectionState::PROBING ||
        state_ == ConnectionState::CONNECTING ||
        state_ == ConnectionState::DISCONNECTING) {
        emitFailedMessageRecords(failed_message_records);
        if (had_pending_payload && on_message_sent_) {
            on_message_sent_(false);
        }
        enterDisconnected("TX aborted");
        return;
    }

    // Connected state remains established; only outbound transfer is aborted.
    if (state_ == ConnectionState::CONNECTED) {
        emitFailedMessageRecords(failed_message_records);
        if (had_pending_payload && on_message_sent_) {
            on_message_sent_(false);
        }
    }

    LOG_MODEM(INFO, "Connection: TX abort applied (state=%s)",
              connectionStateToString(state_));
}

void Connection::setForcedFrameCodewords(int cw_count, bool forced) {
    cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    const uint8_t previous_forced_cw = config_.forced_cw_count;
    const bool logical_cw_changed = cw_count != data_frame_cw_count_;
    const bool force_policy_changed =
        forced && static_cast<uint8_t>(cw_count) != previous_forced_cw;
    if (!logical_cw_changed && !force_policy_changed) {
        return;
    }

    // This public operator/configuration API used to bypass applyDataMode() and
    // re-grid the ARQ window underneath a fragmented message. Reject the runtime
    // change while any DATA operation owns the current geometry. Startup calls
    // and clean file-group boundaries remain admissible.
    if (hasGeometryBoundDataOperation()) {
        LOG_MODEM(WARN,
                  "Connection: Refusing fixed-CW override %d while a DATA operation owns the current geometry",
                  cw_count);
        return;
    }

    if (forced) {
        // Operator override: initiator embeds in CONNECT.data_frame_cw_count
        // so responder honors and echoes via CONNECT_ACK. One-sided
        // propagation — caller only needs to set this on one peer.
        config_.forced_cw_count = static_cast<uint8_t>(cw_count);
    }
    // Note: !forced is the boot-time default path (host wiring up encoder/
    // decoder before connection). It MUST NOT touch config_.forced_cw_count
    // or every connect would advertise the default as a forced override
    // and bypass auto-pick on the responder.

    if (logical_cw_changed) {
        data_frame_cw_count_ = cw_count;
        config_.fixed_frame_codewords = cw_count;
    }

    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    if (isOFDMMode(negotiated_mode_) || bounded_variable_mc_dpsk) {
        // Re-evaluate the complete physical tuple even when the logical CW is
        // unchanged. A newly forced logical count disables either experimental
        // Z81 substitution; returning early here could leave ARQ on the old long
        // tuple while the PHY reverts to the descriptor's logical/Z27 policy.
        configureArqForCurrentDataMode();
    }

    LOG_MODEM(INFO, "Connection: Fixed data frame CW count set to %d", data_frame_cw_count_);
}

// =============================================================================
// DATA TRANSFER
// =============================================================================

bool Connection::sendMessage(const std::string& text) {
    if (text.empty()) {
        LOG_MODEM(WARN, "Connection: Refusing empty operator message");
        return false;
    }
    Bytes data(text.begin(), text.end());
    return sendPayload(data, false);
}

bool Connection::sendBinary(const Bytes& data) {
    return sendPayload(data, true);
}

uint64_t Connection::createOutboundMessageRecord(const Bytes& data) {
    if (data.empty()) {
        return 0;
    }

    OutboundMessageTxRecord record;
    record.token = next_outbound_message_token_++;
    if (next_outbound_message_token_ == 0) {
        next_outbound_message_token_ = 1;
    }
    record.text.assign(data.begin(), data.end());
    record.remote_call = remote_call_;
    outbound_message_tx_records_.push_back(std::move(record));
    return outbound_message_tx_records_.back().token;
}

void Connection::setOutboundMessageExpectedFragments(uint64_t token, size_t fragments) {
    if (token == 0) {
        return;
    }
    for (auto& record : outbound_message_tx_records_) {
        if (record.token == token) {
            record.expected_fragments = fragments;
            return;
        }
    }
}

void Connection::dropOutboundMessageRecord(uint64_t token) {
    if (token == 0) {
        return;
    }
    outbound_message_tx_records_.erase(
        std::remove_if(outbound_message_tx_records_.begin(),
                       outbound_message_tx_records_.end(),
                       [token](const OutboundMessageTxRecord& record) {
                           return record.token == token;
                       }),
        outbound_message_tx_records_.end());
}

void Connection::failOutboundMessageRecord(uint64_t token) {
    if (token == 0) {
        return;
    }
    std::optional<OutboundMessageTxRecord> failed;
    for (auto it = outbound_message_tx_records_.begin();
         it != outbound_message_tx_records_.end(); ++it) {
        if (it->token != token) {
            continue;
        }
        it->terminal_reported = true;
        failed = *it;
        outbound_message_tx_records_.erase(it);
        break;
    }
    if (failed) {
        emitMessageTxStatus(*failed, MessageTxStatus::FAILED);
    }
}

std::vector<Connection::OutboundMessageTxRecord>
Connection::detachOutboundMessageRecords() {
    std::vector<OutboundMessageTxRecord> records;
    records.reserve(outbound_message_tx_records_.size());
    for (auto& record : outbound_message_tx_records_) {
        if (!record.terminal_reported) {
            record.terminal_reported = true;
            records.push_back(record);
        }
    }
    clearOutboundMessageTracking();
    return records;
}

void Connection::emitFailedMessageRecords(
    const std::vector<OutboundMessageTxRecord>& records) {
    MessageTxStatusDeferralGuard status_guard(*this);
    for (const auto& record : records) {
        emitMessageTxStatus(record, MessageTxStatus::FAILED);
    }
}

void Connection::clearOutboundMessageTracking() {
    outbound_message_tx_records_.clear();
    pending_tx_fragment_message_tokens_.clear();
    arq_submit_message_token_ = 0;
}

uint64_t Connection::allocateQueuedOperationOrder() {
    const uint64_t order = next_queued_operation_order_++;
    if (next_queued_operation_order_ == 0) {
        next_queued_operation_order_ = 1;
    }
    return order;
}

bool Connection::sendArqPayloadFrame(const Bytes& chunk,
                                     v2::FrameType frame_type,
                                     uint8_t flags,
                                     bool fixed_frame,
                                     uint64_t message_token) {
    arq_submit_message_token_ = message_token;
    const bool sent = fixed_frame
        ? arq_.sendFixedDataWithTypeAndFlags(chunk, frame_type, flags)
        : arq_.sendDataWithTypeAndFlags(chunk, frame_type, flags);
    arq_submit_message_token_ = 0;
    return sent;
}

void Connection::emitMessageTxStatus(const OutboundMessageTxRecord& record,
                                     MessageTxStatus status) {
    if (!on_message_tx_status_) {
        return;
    }

    MessageTxStatusEvent event;
    event.status = status;
    event.first_seq = record.first_seq;
    event.last_seq = record.last_seq;
    event.sequence_valid = record.first_seq_valid;
    event.remote_call = record.remote_call;
    event.text = record.text;
    if (record.first_seq_valid) {
        const auto now = std::chrono::steady_clock::now();
        event.elapsed_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - record.submitted_at).count());
    }
    pending_message_tx_status_events_.push_back(std::move(event));
    drainMessageTxStatusEvents();
}

void Connection::drainMessageTxStatusEvents() {
    if (message_tx_status_deferral_depth_ != 0 ||
        message_tx_status_dispatch_active_ || !on_message_tx_status_) {
        return;
    }

    // A callback may synchronously submit/reset/abort. Those operations append
    // to this same FIFO and the active dispatcher consumes them only after the
    // current callback returns, preserving SUBMITTED-before-terminal ordering.
    message_tx_status_dispatch_active_ = true;
    while (!pending_message_tx_status_events_.empty()) {
        MessageTxStatusEvent event =
            std::move(pending_message_tx_status_events_.front());
        pending_message_tx_status_events_.pop_front();
        auto callback = on_message_tx_status_;
        if (callback) {
            callback(event);
        }
    }
    message_tx_status_dispatch_active_ = false;
}

void Connection::handleArqFrameSubmitted(uint16_t seq) {
    const uint64_t token = arq_submit_message_token_;
    if (token == 0) {
        return;
    }

    std::optional<OutboundMessageTxRecord> submitted_record;
    for (auto& record : outbound_message_tx_records_) {
        if (record.token != token || record.terminal_reported) {
            continue;
        }
        if (!record.first_seq_valid) {
            record.first_seq = seq;
            record.first_seq_valid = true;
            record.submitted_at = std::chrono::steady_clock::now();
        }
        record.last_seq = seq;
        record.assigned_fragments++;
        if (!record.submitted_reported) {
            record.submitted_reported = true;
            submitted_record = record;
        }
        if (record.expected_fragments != 0 &&
            record.assigned_fragments >= record.expected_fragments) {
            LOG_MODEM(INFO,
                      "Connection: Message TX #%u spans seq=%u..%u (%zu frame%s, %zu bytes)",
                      record.first_seq,
                      record.first_seq,
                      record.last_seq,
                      record.assigned_fragments,
                      record.assigned_fragments == 1 ? "" : "s",
                      record.text.size());
        }
        break;
    }
    // Never enter application code with an iterator/reference into the live deque.
    // A status callback may synchronously send, reset, or disconnect.
    if (submitted_record) {
        emitMessageTxStatus(*submitted_record, MessageTxStatus::SUBMITTED);
    }
}

void Connection::handleArqTxBaseAdvanced(uint16_t base_seq) {
    if (deferred_arq_failure_abort_) {
        // advanceTXWindow() may cross SACKed suffix slots after the base frame has
        // terminal-failed. They are not a successful logical completion; the deferred
        // drain will fail the active record(s) after the ARQ call stack unwinds.
        return;
    }
    // F218: every retiring ack re-checks the deferred completion — the gate
    // (ARQ idle) opens exactly when the last outstanding frame retires.
    file_transfer_.maybeCompleteSend();
    std::vector<OutboundMessageTxRecord> delivered_records;
    for (auto it = outbound_message_tx_records_.begin();
         it != outbound_message_tx_records_.end();) {
        const bool delivered =
            !it->terminal_reported && it->first_seq_valid &&
            it->assigned_fragments >= it->expected_fragments &&
            seqBefore(it->last_seq, base_seq);
        if (!delivered) {
            ++it;
            continue;
        }
        it->terminal_reported = true;
        delivered_records.push_back(*it);
        it = outbound_message_tx_records_.erase(it);
    }

    // Publish only after every delivered record has been detached. Reentrant sends
    // append fresh records without invalidating an outer iterator or being erased as
    // part of the just-completed batch.
    for (const auto& record : delivered_records) {
        emitMessageTxStatus(record, MessageTxStatus::DELIVERED);
    }
}

void Connection::handleArqFrameFailed(uint16_t seq) {
    // The callback runs inside SelectiveRepeatARQ::retransmitFrame(), before that
    // method retires the failed slot and decrements its counters. Never mutate the
    // ARQ window here; request one post-unwind logical-transfer abort instead.
    if (!deferred_arq_failure_abort_) {
        deferred_arq_failure_seq_ = seq;
        deferred_arq_failure_seq_valid_ = true;
    }
    deferred_arq_failure_abort_ = true;
}

bool Connection::sendPayload(const Bytes& data, bool binary_payload) {
    if (state_ != ConnectionState::CONNECTED || disconnect_teardown_active_) {
        LOG_MODEM(WARN, "Connection: Cannot send, link is not data-ready");
        return false;
    }
    if (data.empty()) {
        LOG_MODEM(WARN, "Connection: Refusing empty %s payload",
                  binary_payload ? "binary" : "text");
        return false;
    }

    const uint64_t message_token = binary_payload ? 0 : createOutboundMessageRecord(data);
    LOG_MODEM(DEBUG, "Connection: B2F-DBG sendPayload %zuB queue=%d (local_turn=%d is_init=%d hs_conf=%d yield_pend=%d peer_req=%d guard=%u)",
              data.size(), shouldQueuePayloadForLinkTurn() ? 1 : 0,
              local_data_turn_ ? 1 : 0, is_initiator_ ? 1 : 0, handshake_confirmed_ ? 1 : 0,
              data_turn_yield_pending_ ? 1 : 0, peer_data_turn_requested_ ? 1 : 0, data_turn_tx_guard_ms_);
    if (shouldQueuePayloadForLinkTurn()) {
        if (queued_payloads_.size() >= kMaxQueuedPayloads) {
            dropOutboundMessageRecord(message_token);
            LOG_MODEM(WARN,
                      "Connection: Queued payload limit reached; refusing newest payload");
            return false;
        }
        queued_payloads_.push_back(QueuedPayload{
            data, binary_payload, message_token, allocateQueuedOperationOrder()});
        LOG_MODEM(INFO,
                  "Connection: Queued %zu byte %s until local ISS DATA turn (depth=%zu, local_turn=%d, peer_request=%d)",
                  data.size(), binary_payload ? "binary payload" : "message",
                  queued_payloads_.size(), local_data_turn_ ? 1 : 0,
                  peer_data_turn_requested_ ? 1 : 0);
        sendTurnRequestIfNeeded();
        return true;
    }

    const bool started = startPayloadNow(data, binary_payload, message_token);
    if (!started) {
        dropOutboundMessageRecord(message_token);
    }
    return started;
}

bool Connection::hasLocalOutboundDataTurn() const {
    return deferred_arq_failure_abort_ ||
           file_transfer_.getState() == FileTransferState::SENDING ||
           mode_change_pending_ ||
           !pending_tx_fragments_.empty() ||
           arq_.getTxInFlightBytes() > 0;
}

bool Connection::hasGeometryBoundDataOperation() const {
    // File chunks are rebuilt at a clean ARQ-window boundary, so a file may move
    // between groups. A message/binary operation is different: its complete flat
    // fragment vector was cut using one capacity and must own that geometry until
    // the whole logical object retires. Otherwise a clean first group could demote
    // before the remaining chunks and silently resize their wire representation.
    const bool file_window_busy =
        file_transfer_.getState() == FileTransferState::SENDING &&
        (file_transfer_.hasPendingChunks() || arq_.getTxInFlightBytes() > 0);
    const bool nonfile_operation_busy =
        file_transfer_.getState() != FileTransferState::SENDING &&
        (!pending_tx_fragments_.empty() || arq_.getTxInFlightBytes() > 0 ||
         std::any_of(outbound_message_tx_records_.begin(),
                     outbound_message_tx_records_.end(),
                     [](const OutboundMessageTxRecord& record) {
                         return record.first_seq_valid && !record.terminal_reported;
                     }));
    return file_window_busy || nonfile_operation_busy;
}

bool Connection::hasLocalInFlightDataTurn() const {
    return mode_change_pending_ ||
           !pending_tx_fragments_.empty() ||
           arq_.getTxInFlightBytes() > 0 ||
           (file_transfer_.getState() == FileTransferState::SENDING &&
            file_transfer_.hasPendingChunks());
}

bool Connection::hasLocalDataWaitingForTurn() const {
    const bool file_waiting =
        file_transfer_.getState() == FileTransferState::SENDING &&
        (file_transfer_.hasMoreChunks() || file_transfer_.hasPendingChunks());

    const bool fragments_waiting =
        !pending_tx_fragments_.empty() &&
        (next_fragment_idx_ < pending_tx_fragments_.size() ||
         acked_fragment_count_ < pending_tx_fragments_.size());

    return queued_file_path_.has_value() ||
           !queued_payloads_.empty() ||
           file_waiting ||
           fragments_waiting ||
           mode_change_pending_;
}

bool Connection::dataTurnFairBudgetMet() const {
    return data_turn_payload_bytes_sent_ >= DATA_TURN_FAIR_BURST_BYTES ||
           (data_turn_contended_ms_ >= DATA_TURN_FAIR_BURST_MS &&
            data_turn_payload_bytes_sent_ >= DATA_TURN_FAIR_MIN_BYTES_FOR_TIME_YIELD);
}

bool Connection::shouldPauseLocalDataForPeerRequest() const {
    if (state_ != ConnectionState::CONNECTED ||
        !local_data_turn_ ||
        !peer_data_turn_requested_ ||
        file_cancel_confirm_pending_ ||
        file_transfer_.getState() == FileTransferState::SENDING ||
        !dataTurnFairBudgetMet()) {
        return false;
    }

    // Do not split an already-started fragmented operator payload. An active
    // file transfer owns the DATA turn until completion or cancel, so file
    // chunks are intentionally not paused for chat turn requests.
    return pending_tx_fragments_.empty();
}

bool Connection::shouldQueuePayloadForLinkTurn() const {
    if (state_ != ConnectionState::CONNECTED) {
        return false;
    }

    // HF ARQ is a half-duplex link. Only the current ISS may originate DATA.
    // Operator payloads from the IRS are queued and announced through a
    // turn-request ACK/control frame; they do not race the peer on the channel.
    const bool responder_handshake_turn =
        !is_initiator_ && !handshake_confirmed_;

    return responder_handshake_turn ||
           data_turn_yield_pending_ ||
           !local_data_turn_ ||
           peer_data_turn_requested_ ||
           file_cancel_confirm_pending_ ||
           data_turn_tx_guard_ms_ > 0 ||
           queued_file_path_.has_value() ||
           !queued_payloads_.empty() ||
           hasLocalOutboundDataTurn();
}

bool Connection::shouldRequestDataTurnOnAck() const {
    return state_ == ConnectionState::CONNECTED &&
           !local_data_turn_ &&
           !file_transfer_.isBusy() &&
           !file_cancel_confirm_pending_ &&
           hasLocalDataWaitingForTurn() &&
           (is_initiator_ || handshake_confirmed_);
}

bool Connection::noteTurnRequestOnAckIfNeeded() {
    if (!shouldRequestDataTurnOnAck()) {
        return false;
    }

    // A TURN_REQUEST bit riding on an ACK is already an on-air request. The
    // first standalone retransmission must wait long enough for the peer to
    // receive that ACK, finish any ACK-diversity guard, and send TURNOVER back
    // across the half-duplex channel. Otherwise both sides can transmit control
    // bursts into each other at the ownership change.
    local_turn_request_pending_ = true;
    turn_request_retransmit_ms_ = std::max(
        turn_request_retransmit_ms_,
        turnRequestAckEmbeddedRetransmitMs());
    return true;
}

void Connection::resetDataTurnFairness() {
    data_turn_payload_bytes_sent_ = 0;
    data_turn_contended_ms_ = 0;
}

void Connection::noteDataTurnPayloadStarted(size_t payload_bytes) {
    if (local_data_turn_) {
        data_turn_payload_bytes_sent_ += payload_bytes;
    }

    // The one-shot interactive TURNOVER is only bootstrap choreography for an
    // initiator that has sent no DATA (the B2F responder speaks first).  Once the
    // initiator has actually started DATA, firing that bootstrap rule at the next
    // momentary empty-window boundary can split one continuous host stream: the TNC
    // may still have the next accumulated block outside Connection's queue.  Leave
    // the turn with the active sender; a responder with real outbound data retains
    // the normal TURN_REQUEST/fair-budget path.
    if (half_duplex_interactive_ && is_initiator_) {
        interactive_initiator_yield_done_ = true;
    }
}

void Connection::sendTurnRequestIfNeeded() {
    if (state_ != ConnectionState::CONNECTED ||
        local_data_turn_ ||
        file_transfer_.isBusy() ||
        file_cancel_confirm_pending_ ||
        yielded_data_turn_waiting_for_peer_data_ ||
        !hasLocalDataWaitingForTurn() ||
        local_turn_request_pending_ ||
        turn_request_holdoff_ms_ > 0 ||
        (!is_initiator_ && !handshake_confirmed_)) {
        return;
    }

    auto request = v2::ControlFrame::makeTurnRequest(local_call_, remote_call_);
    LOG_MODEM(INFO, "Connection: TX TURN_REQUEST (queued=%zu, backlog=%zu bytes)",
              queued_payloads_.size(), getTxBacklogBytes());
    transmitFrame(request.serialize());
    local_turn_request_pending_ = true;
    turn_request_retransmit_ms_ = turnRequestRetransmitMs();
}

void Connection::armDataTurnTxGuard(uint32_t guard_ms) {
    data_turn_tx_guard_ms_ = std::max(data_turn_tx_guard_ms_, guard_ms);
}

bool Connection::maybeYieldDataTurn() {
    if (state_ != ConnectionState::CONNECTED ||
        !local_data_turn_ ||
        !peer_data_turn_requested_ ||
        file_transfer_.getState() == FileTransferState::SENDING ||
        hasLocalInFlightDataTurn()) {
        return false;
    }

    const bool only_unstarted_file_waiting =
        queued_file_path_.has_value() &&
        queued_payloads_.empty() &&
        file_transfer_.getState() != FileTransferState::SENDING &&
        pending_tx_fragments_.empty() &&
        arq_.getTxInFlightBytes() == 0 &&
        !mode_change_pending_;

    if (!data_turn_yield_pending_ &&
        data_turn_payload_bytes_sent_ > 0 &&
        hasLocalDataWaitingForTurn() &&
        !only_unstarted_file_waiting &&
        !dataTurnFairBudgetMet()) {
        return false;
    }

    if (data_turn_tx_guard_ms_ > 0) {
        data_turn_yield_pending_ = true;
        return false;
    }

    auto turnover = v2::ControlFrame::makeTurnover(local_call_, remote_call_);
    LOG_MODEM(INFO,
              "Connection: TX TURNOVER to %s (peer requested DATA turn, turn_bytes=%llu, contended_ms=%u, backlog=%zu)",
              remote_call_.c_str(),
              static_cast<unsigned long long>(data_turn_payload_bytes_sent_),
              data_turn_contended_ms_, getTxBacklogBytes());
    transmitFrame(turnover.serialize());
    local_data_turn_ = false;
    peer_data_turn_requested_ = false;
    local_turn_request_pending_ = false;
    yielded_data_turn_waiting_for_peer_data_ = true;
    data_turn_yield_pending_ = false;
    turn_request_retransmit_ms_ = 0;
    turn_request_holdoff_ms_ = turnRequestHoldoffAfterDataMs();
    received_peer_data_since_connect_ = false;
    resetDataTurnFairness();
    armDataTurnTxGuard(dataTurnControlGuardMs());
    return true;
}

bool Connection::startPayloadNow(const Bytes& data, bool binary_payload, uint64_t message_token) {
    MessageTxStatusDeferralGuard status_guard(*this);
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send, not connected");
        return false;
    }

    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    const size_t capacity = currentDataPayloadCapacity();
    auto markPayloadStarted = [this, &data](bool started) {
        if (started) {
            noteDataTurnPayloadStarted(data.size());
        }
        return started;
    };

    if (!is_ofdm && !bounded_variable_mc_dpsk) {
        setOutboundMessageExpectedFragments(message_token, 1);
        if (binary_payload) {
            return markPayloadStarted(
                sendArqPayloadFrame(data, v2::FrameType::DATA_END, v2::Flags::NONE,
                                    false, message_token));
        }
        return markPayloadStarted(
            sendArqPayloadFrame(data, v2::FrameType::DATA, v2::Flags::NONE,
                                false, message_token));
    }

    if (capacity == 0) {
        LOG_MODEM(ERROR, "Connection: Data frame payload capacity is zero for current mode");
        return false;
    }

    if (data.size() <= capacity) {
        setOutboundMessageExpectedFragments(message_token, 1);
        if (binary_payload) {
            return markPayloadStarted(
                is_ofdm
                    ? sendArqPayloadFrame(data, v2::FrameType::DATA_END, v2::Flags::FINAL,
                                          true, message_token)
                    : sendArqPayloadFrame(data, v2::FrameType::DATA_END, v2::Flags::FINAL,
                                          false, message_token));
        }
        return markPayloadStarted(
            sendArqPayloadFrame(data, v2::FrameType::DATA, v2::Flags::FINAL,
                                is_ofdm, message_token));
    }

    // Fragment the message into chunks that fit in one frame each
    LOG_MODEM(INFO, "Connection: Fragmenting %zu byte %s into %zu-byte chunks",
              data.size(), binary_payload ? "binary payload" : "message", capacity);

    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    pending_tx_fragment_message_tokens_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;

    for (size_t offset = 0; offset < data.size(); offset += capacity) {
        size_t chunk_size = std::min(capacity, data.size() - offset);
        pending_tx_fragments_.emplace_back(data.begin() + offset, data.begin() + offset + chunk_size);
        if (binary_payload) {
            const bool first = (offset == 0);
            const bool last = (offset + chunk_size >= data.size());
            pending_tx_fragment_types_.push_back(
                first ? v2::FrameType::DATA_START :
                (last ? v2::FrameType::DATA_END : v2::FrameType::DATA_CONT));
        }
        const bool last = (offset + chunk_size >= data.size());
        pending_tx_fragment_flags_.push_back(
            last ? v2::Flags::FINAL : v2::Flags::MORE_FRAG);
        pending_tx_fragment_message_tokens_.push_back(message_token);
    }
    setOutboundMessageExpectedFragments(message_token, pending_tx_fragments_.size());

    LOG_MODEM(INFO, "Connection: Split into %zu fragments", pending_tx_fragments_.size());

    if (sendNextFragment() == 0) {
        pending_tx_fragments_.clear();
        pending_tx_fragment_flags_.clear();
        pending_tx_fragment_types_.clear();
        pending_tx_fragment_message_tokens_.clear();
        acked_fragment_count_ = 0;
        return false;
    }
    return true;
}

void Connection::transmitFileCancelControl(const char* reason) {
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }

    auto cancel = v2::ControlFrame::makeFileCancel(local_call_, remote_call_);
    LOG_MODEM(INFO, "Connection: TX FILE_CANCEL to %s%s",
              remote_call_.c_str(), reason ? reason : "");
    transmitFrame(cancel.serialize());
}

void Connection::armFileCancelReassertion() {
    file_cancel_reassert_ms_ = FILE_CANCEL_REASSERT_WINDOW_MS;
    file_cancel_reassert_cooldown_ms_ = 0;
}

void Connection::clearFileCancelReassertion() {
    file_cancel_reassert_ms_ = 0;
    file_cancel_reassert_cooldown_ms_ = 0;
}

void Connection::maybeReassertFileCancelForStaleData() {
    if (file_cancel_reassert_ms_ == 0 ||
        file_cancel_reassert_cooldown_ms_ > 0 ||
        state_ != ConnectionState::CONNECTED) {
        return;
    }

    transmitFileCancelControl(" (reassert stale DATA)");
    file_cancel_reassert_cooldown_ms_ = FILE_CANCEL_REASSERT_COOLDOWN_MS;
}

bool Connection::tryStartQueuedFileIfReady() {
    if (!queued_file_path_) {
        return false;
    }
    if (state_ != ConnectionState::CONNECTED) {
        return false;
    }
    if (!queued_payloads_.empty() &&
        queued_payloads_.front().enqueue_order < queued_file_order_) {
        return false;
    }
    if (!local_data_turn_) {
        sendTurnRequestIfNeeded();
        return false;
    }
    if (file_cancel_confirm_pending_) {
        return false;
    }
    if (peer_data_turn_requested_ || data_turn_yield_pending_) {
        maybeYieldDataTurn();
        return false;
    }
    if (data_turn_tx_guard_ms_ > 0 ||
        hasLocalOutboundDataTurn() ||
        !pending_tx_fragments_.empty() ||
        !arq_.isReadyToSend()) {
        return false;
    }

    const std::string path = *queued_file_path_;
    queued_file_path_.reset();
    queued_file_order_ = 0;
    LOG_MODEM(INFO, "Connection: Starting queued file transfer on local ISS DATA turn: %s",
              path.c_str());
    if (!startFileTransferNow(path)) {
        // sendFile() already returned true when this request entered the queue.
        // Geometry can legitimately change while waiting for the DATA turn, so a
        // later FILE_START-capacity rejection must be an explicit terminal result,
        // not a silently discarded accepted request. The queue was cleared before
        // start to keep a re-entrant callback free to submit the next file.
        const std::string error = "Queued file transfer failed to start";
        LOG_MODEM(ERROR, "Connection: %s: %s", error.c_str(), path.c_str());
        if (on_file_sent_) {
            on_file_sent_(false, error);
        }
        return false;
    }
    return true;
}

void Connection::sendNextQueuedPayloadIfReady() {
    const bool file_is_next =
        queued_file_path_ &&
        (queued_payloads_.empty() ||
         queued_file_order_ < queued_payloads_.front().enqueue_order);
    if (file_is_next) {
        tryStartQueuedFileIfReady();
        return;
    }
    if (queued_payloads_.empty()) {
        return;
    }
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }
    if (!is_initiator_ && !handshake_confirmed_) {
        return;
    }
    if (!local_data_turn_) {
        sendTurnRequestIfNeeded();
        return;
    }
    if (file_cancel_confirm_pending_) {
        return;
    }
    if (data_turn_tx_guard_ms_ > 0) {
        return;
    }
    if (data_turn_yield_pending_) {
        maybeYieldDataTurn();
        return;
    }
    if (shouldPauseLocalDataForPeerRequest()) {
        maybeYieldDataTurn();
        return;
    }
    if (hasLocalOutboundDataTurn() || !arq_.isReadyToSend()) {
        return;
    }

    QueuedPayload payload = std::move(queued_payloads_.front());
    queued_payloads_.pop_front();
    LOG_MODEM(INFO,
              "Connection: Sending deferred half-duplex payload (%zu bytes, remaining=%zu)",
              payload.data.size(), queued_payloads_.size());
    if (!startPayloadNow(payload.data, payload.binary_payload,
                         payload.message_token)) {
        LOG_MODEM(ERROR,
                  "Connection: Accepted queued %s failed to start; reporting terminal failure",
                  payload.binary_payload ? "binary payload" : "message");
        failOutboundMessageRecord(payload.message_token);
        if (on_message_sent_) {
            on_message_sent_(false);
        }
    }
}

bool Connection::sendMessages(const std::vector<std::string>& texts) {
    MessageTxStatusDeferralGuard status_guard(*this);
    if (state_ != ConnectionState::CONNECTED || disconnect_teardown_active_) {
        LOG_MODEM(WARN, "Connection: Cannot send, link is not data-ready");
        return false;
    }
    if (texts.empty() ||
        std::any_of(texts.begin(), texts.end(),
                    [](const std::string& text) { return text.empty(); })) {
        LOG_MODEM(WARN,
                  "Connection: Refusing empty message or empty message batch");
        return false;
    }
    if (shouldQueuePayloadForLinkTurn()) {
        if (texts.size() > kMaxQueuedPayloads - queued_payloads_.size()) {
            LOG_MODEM(WARN,
                      "Connection: Queued payload limit reached; refusing %zu-message batch",
                      texts.size());
            return false;
        }
        for (const auto& text : texts) {
            Bytes data(text.begin(), text.end());
            const uint64_t message_token = createOutboundMessageRecord(data);
            queued_payloads_.push_back(QueuedPayload{
                data, false, message_token, allocateQueuedOperationOrder()});
        }
        LOG_MODEM(INFO,
                  "Connection: Queued %zu-message batch until local ISS DATA turn (depth=%zu)",
                  texts.size(), queued_payloads_.size());
        sendTurnRequestIfNeeded();
        return !texts.empty();
    }

    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    size_t capacity = (is_ofdm || bounded_variable_mc_dpsk) ? currentDataPayloadCapacity() : SIZE_MAX;
    if (capacity == 0) {
        LOG_MODEM(ERROR,
                  "Connection: Cannot send message batch with zero payload capacity");
        return false;
    }

    // Pre-fragment all messages into a flat list of frame payloads with flags
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    pending_tx_fragment_message_tokens_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;

    for (const auto& text : texts) {
        Bytes data(text.begin(), text.end());
        const uint64_t message_token = createOutboundMessageRecord(data);
        size_t frames_for_message = 0;

        if (data.size() <= capacity) {
            // Single frame — no MORE_FRAG
            pending_tx_fragments_.push_back(data);
            pending_tx_fragment_flags_.push_back(v2::Flags::NONE);
            pending_tx_fragment_message_tokens_.push_back(message_token);
            frames_for_message = 1;
        } else {
            // Fragment this message
            for (size_t offset = 0; offset < data.size(); offset += capacity) {
                size_t chunk_size = std::min(capacity, data.size() - offset);
                Bytes chunk(data.begin() + offset, data.begin() + offset + chunk_size);
                bool is_last = (offset + chunk_size >= data.size());
                pending_tx_fragments_.push_back(chunk);
                pending_tx_fragment_flags_.push_back(
                    is_last ? v2::Flags::NONE : v2::Flags::MORE_FRAG);
                pending_tx_fragment_message_tokens_.push_back(message_token);
                frames_for_message++;
            }
        }
        setOutboundMessageExpectedFragments(message_token, frames_for_message);
    }

    if (!pending_tx_fragment_flags_.empty()) {
        pending_tx_fragment_flags_.back() |= v2::Flags::FINAL;
    }

    LOG_MODEM(INFO, "Connection: Batch queued %zu messages as %zu frames",
              texts.size(), pending_tx_fragments_.size());

    // Send first window-worth via sendNextFragment() (handles burst buffering)
    if (sendNextFragment() == 0) {
        for (const uint64_t token : pending_tx_fragment_message_tokens_) {
            dropOutboundMessageRecord(token);
        }
        pending_tx_fragments_.clear();
        pending_tx_fragment_flags_.clear();
        pending_tx_fragment_types_.clear();
        pending_tx_fragment_message_tokens_.clear();
        return false;
    }
    return true;
}

bool Connection::isReadyToSend() const {
    return state_ == ConnectionState::CONNECTED && !disconnect_teardown_active_ &&
           arq_.isReadyToSend() &&
           local_data_turn_ && !peer_data_turn_requested_ &&
           !file_cancel_confirm_pending_ &&
           data_turn_tx_guard_ms_ == 0 && !file_transfer_.isBusy() &&
           !queued_file_path_.has_value() && queued_payloads_.empty();
}

size_t Connection::getTxBacklogBytes() const {
    size_t bytes = arq_.getTxInFlightBytes();

    if (next_fragment_idx_ < pending_tx_fragments_.size()) {
        for (size_t i = next_fragment_idx_; i < pending_tx_fragments_.size(); ++i) {
            bytes += pending_tx_fragments_[i].size();
        }
    }

    for (const auto& payload : queued_payloads_) {
        bytes += payload.data.size();
    }

    if (queued_file_path_) {
        std::error_code ec;
        const auto size = std::filesystem::file_size(*queued_file_path_, ec);
        bytes += ec ? 1 : static_cast<size_t>(size);
    }

    if (file_transfer_.getState() == FileTransferState::SENDING) {
        bytes += file_transfer_.remainingTxBytes();
    }

    return bytes;
}

// =============================================================================
// FILE TRANSFER
// =============================================================================

bool Connection::sendFile(const std::string& filepath) {
    LOG_MODEM(WARN, "Connection::sendFile() called path=%s state=%d use_burst=%d",
              filepath.c_str(), static_cast<int>(state_), use_burst_transport_ ? 1 : 0);
    if (state_ != ConnectionState::CONNECTED || disconnect_teardown_active_) {
        LOG_MODEM(WARN, "Connection: Cannot send file, link is not data-ready");
        return false;
    }

    if (file_transfer_.isBusy() || queued_file_path_) {
        LOG_MODEM(WARN, "Connection: File transfer already in progress");
        return false;
    }

    // Validate before either the immediate-start or queued-acceptance branch.
    // A true return means the caller owns a live request and may wait for its
    // completion callback; accepting an impossible FILE_START into the ISS queue
    // would otherwise lose it later when the turn finally becomes available.
    const bool bounded_file_frames =
        isOFDMMode(negotiated_mode_) || usesBoundedVariableMCDPSKFrames();
    if (bounded_file_frames) {
        const size_t capacity = currentDataPayloadCapacity();
        if (capacity < FileTransferController::MIN_FILE_START_PAYLOAD) {
            LOG_MODEM(ERROR,
                      "Connection: File payload capacity %zu is too small for FILE_START (minimum %zu); request not accepted",
                      capacity, FileTransferController::MIN_FILE_START_PAYLOAD);
            return false;
        }
    }

    // Non-interactive burst files retain their sender-driven ISS bypass, but only
    // at a genuinely empty logical-operation boundary. The ARQ completion callback
    // is state-dispatched, so allowing a file to enter SENDING over message/binary
    // slots makes those older ACKs pop the file ledger (BUG-FILE-ACK-IDENTITY).
    const bool noninteractive_burst_bypass =
        use_burst_transport_ && isOFDMMode(negotiated_mode_) &&
        !half_duplex_interactive_;
    const bool logical_operation_active =
        hasLocalOutboundDataTurn() || !pending_tx_fragments_.empty() ||
        !queued_payloads_.empty() || !outbound_message_tx_records_.empty() ||
        !arq_.isReadyToSend();
    const bool owns_clear_data_turn =
        local_data_turn_ && !peer_data_turn_requested_ &&
        !file_cancel_confirm_pending_ && data_turn_tx_guard_ms_ == 0 &&
        !data_turn_yield_pending_;
    if (noninteractive_burst_bypass && owns_clear_data_turn &&
        !logical_operation_active) {
        return startFileTransferNow(filepath);
    }

    if (!local_data_turn_ ||
        peer_data_turn_requested_ ||
        file_cancel_confirm_pending_ ||
        data_turn_tx_guard_ms_ > 0 ||
        hasLocalOutboundDataTurn() ||
        !pending_tx_fragments_.empty() ||
        !queued_payloads_.empty() ||
        !arq_.isReadyToSend()) {
        queued_file_path_ = filepath;
        queued_file_order_ = allocateQueuedOperationOrder();
        LOG_MODEM(INFO,
                  "Connection: Queued file transfer until local ISS DATA turn is clear (path=%s, local_turn=%d, peer_request=%d, guard_ms=%u)",
                  filepath.c_str(), local_data_turn_ ? 1 : 0,
                  peer_data_turn_requested_ ? 1 : 0, data_turn_tx_guard_ms_);
        sendTurnRequestIfNeeded();
        maybeYieldDataTurn();
        return true;
    }

    return startFileTransferNow(filepath);
}

bool Connection::startFileTransferNow(const std::string& filepath) {
    if (!arq_.isReadyToSend()) {
        LOG_MODEM(INFO, "Connection: ARQ busy, cannot start file transfer");
        return false;
    }

    // Arm BEFORE capacity selection/startSend. FileTransferController is still
    // IDLE here, so deriving this policy from getState()==SENDING would size the
    // chunks at Z27 and only switch the encoder afterward. The transfer arms
    // survive rung moves; the exact mod/rate/logical-CW predicates decide whether
    // a given burst uses a physical Z81 profile.
    setExperimentalLongLDPCTransferProfilesActive(
        psk8LongLdpcExperimentEnabled() &&
            negotiated_mode_ == WaveformMode::OFDM_CHIRP,
        qpskR34LongLdpcExperimentEnabled() &&
            negotiated_mode_ == WaveformMode::OFDM_CHIRP);

    // Set chunk size to match frame capacity for bounded frame geometries.
    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    if (is_ofdm || bounded_variable_mc_dpsk) {
        size_t capacity = currentDataPayloadCapacity();
        if (capacity < FileTransferController::MIN_FILE_START_PAYLOAD) {
            LOG_MODEM(ERROR,
                      "Connection: File payload capacity %zu is too small for FILE_START (minimum %zu)",
                      capacity, FileTransferController::MIN_FILE_START_PAYLOAD);
            setExperimentalLongLDPCTransferProfilesActive(false, false);
            return false;
        }
        file_transfer_.setMaxChunkPayload(capacity);
        LOG_MODEM(INFO, "Connection: File chunk payload limited to %zu bytes (%s %s, cw=%d)",
                  capacity,
                  is_ofdm ? "OFDM fixed-frame" : "MC-DPSK variable-frame",
                  codeRateToString(data_code_rate_),
                  is_ofdm ? physicalDataFrameCodewords()
                          : data_frame_cw_count_);
    }

    LOG_MODEM(INFO, "Connection: Starting file transfer: %s", filepath.c_str());

    if (!file_transfer_.startSend(filepath)) {
        LOG_MODEM(ERROR, "Connection: Failed to start file transfer");
        setExperimentalLongLDPCTransferProfilesActive(false, false);
        return false;
    }

    // [LADDER] per-transfer telemetry (time-in-rung + move count, logged at completion).
    ladderTelemetryStart();

    // TRANSPORT MERGE (2026-06-06): the unified arq_ path is the only OFDM file transport —
    // it bursts + interleaves via sendNextFileChunk() -> flushBurstBuffer() over ONE 16-bit
    // seq space, one tone-burst ack, one retransmit window (legacy burst_transport_ removed).
    // #58 increment 3: selection-flavored consumer — the pool aggregate (knob-gated)
    // replaces the single-snapshot scalar so one trough reading can't flip the block
    // strategy for the whole transfer. Knob-off: exactly measured_snr_db_.
    if (is_ofdm && !usesExperimentalLongLDPC() &&
        shouldUseSingleOFDMFileBlock(
            fading_index_, rateSelectionSnrDb(), data_code_rate_)) {
        Bytes block = file_transfer_.getSingleBlockPayload(kOFDMFileBlockPayloadLimit);
        if (!block.empty()) {
            LOG_MODEM(INFO, "Connection: Sending file as single OFDM block (%zu bytes payload)",
                      block.size());
            if (!arq_.sendVariableDataWithFlags(block, v2::Flags::FINAL)) {
                file_transfer_.onSendFailed();
                return false;
            }
            noteDataTurnPayloadStarted(block.size());
            return true;
        }
    } else if (is_ofdm) {
        LOG_MODEM(INFO, "Connection: Using interleaved OFDM chunks for file (SNR=%.1f (%s), fading=%.2f, rate=%s)",
                  measured_snr_db_, snrSourceToString(measured_snr_source_),
                  fading_index_, codeRateToString(data_code_rate_));
    }

    sendNextFileChunk();
    return true;
}

void Connection::setReceiveDirectory(const std::string& dir) {
    file_transfer_.setReceiveDirectory(dir);
}

void Connection::clearFileTransferArqState() {
    deferred_file_refill_ = false;
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    arq_.reset();
    soft_combine_harq_.clear();
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    desc_switch_full_anchor_pending_ = false;
    partial_sack_last_round_ = {};
    partial_sack_descriptor_repair_scope_ = false;
}

void Connection::cancelFileTransfer() {
    const bool had_active_transfer = file_transfer_.isBusy();
    const bool was_local_iss = local_data_turn_;
    if (state_ == ConnectionState::CONNECTED && had_active_transfer) {
        transmitFileCancelControl(" (local cancel)");
        armFileCancelReassertion();
    }

    queued_file_path_.reset();
    queued_file_order_ = 0;
    if (had_active_transfer) {
        file_transfer_.cancel("Transfer cancelled");
        clearFileTransferArqState();
        file_cancel_rx_drain_ms_ = FILE_CANCEL_RX_DRAIN_MS;
        armDataTurnTxGuard(fileCancelTxGuardMs());
        file_cancel_confirm_pending_ = false;
    }

    if (state_ == ConnectionState::CONNECTED) {
        data_turn_yield_pending_ = false;
        resetDataTurnFairness();
        if (!was_local_iss) {
            local_turn_request_pending_ = false;
            turn_request_retransmit_ms_ = 0;
            sendTurnRequestIfNeeded();
        }
        maybeYieldDataTurn();
        sendNextQueuedPayloadIfReady();
    }
}

bool Connection::isFileTransferInProgress() const {
    return file_transfer_.isBusy() || queued_file_path_.has_value();
}

FileTransferProgress Connection::getFileProgress() const {
    FileTransferProgress p = file_transfer_.getProgress();
    // UNIFIED PATH: out-of-order frames are buffered inside the ARQ rx window and not yet
    // handed to file_transfer_ (which assembles in order). Add them to received_bytes so
    // the GUI's "received" count + progress bar advance on ANY frame, not just contiguous
    // ones — matching the old offset-assembler feedback (received_bytes >= transferred).
    // Estimate by frame count × the data-chunk payload size (uniform except the tail).
    if (kUnifiedSeqEnabled() && p.is_sending == false && p.total_bytes > 0) {
        const size_t buffered_frames = arq_.bufferedRxFrameCount();
        if (buffered_frames > 0) {
            const size_t cap = currentDataPayloadCapacity();
            const size_t chunk_bytes =
                (cap > FileTransferController::FILE_DATA_OVERHEAD)
                    ? cap - FileTransferController::FILE_DATA_OVERHEAD
                    : 0;
            const uint32_t buffered_bytes =
                static_cast<uint32_t>(buffered_frames * chunk_bytes);
            p.received_bytes = std::min(p.total_bytes,
                                        p.transferred_bytes + buffered_bytes);
        }
    }
    return p;
}

void Connection::sendNextFileChunk() {
    if (disconnect_teardown_active_ ||
        file_transfer_.getState() != FileTransferState::SENDING) {
        return;
    }

    bool is_ofdm = isOFDMMode(negotiated_mode_);

    const bool is_mc_dpsk = negotiated_mode_ == WaveformMode::MC_DPSK;

    // Enable burst buffering for OFDM and MC-DPSK data-window mode.
    if ((is_ofdm || is_mc_dpsk) && on_transmit_burst_) {
        burst_mode_active_ = true;
        burst_tx_buffer_.clear();
    }

    // Bound ONE burst (key-down) to the half-duplex airtime ceiling: cap the frames
    // submitted this turn to burstAirtimeBudgetFrames(window) so a single transmission
    // can't run too long (PA duty, ack latency, and — the bug this fixes — a burst
    // outliving its own tone-burst ack window). DERIVED per rung (mod/rate/cw/fading),
    // not a fixed count. OFDM only (the airtime model is OFDM); MC-DPSK keeps its own
    // timing-derived window. Gated to the unified path for now (default build unchanged).
    const bool repair_turn =
        is_ofdm && kUnifiedSeqEnabled() && burst_mode_active_ &&
        arq_.getTxInFlightBytes() > 0;
    const bool descriptor_only_partial_repair =
        repair_turn && partial_sack_descriptor_repair_scope_;
    // Consume before any host/CCA callback can run.  The resulting anchor reason is
    // carried on the exact DeferredTx request by App; no encoder-global latch exists.
    partial_sack_descriptor_repair_scope_ = false;
    const size_t burst_frame_cap = prepareUnifiedBurstWindow(
        repair_turn && !descriptor_only_partial_repair);

    // STOP-AND-WAIT, keep-the-pipe-full: this burst = [in-flight holes] + [new chunks],
    // filled to the budget, as ONE group. First RESEND the frames the receiver is still
    // missing (holes), then top up with new chunks. A partial burst's surviving frames
    // were SACKed, so the holes are few — they ride the next group instead of going out
    // as a lonely 1-frame resend, and the rest of the key-down carries new data. (Unified
    // OFDM only; the resend buffers into the open burst via transmitFrame.)
    size_t retransmitted_frames = 0;
    if (is_ofdm && kUnifiedSeqEnabled() && burst_mode_active_) {
        retransmitted_frames = arq_.retransmitInFlightUnacked(burst_frame_cap);
        if (deferred_arq_failure_abort_) {
            // retransmitInFlightUnacked can reach max_retries synchronously. Do not
            // top up or flush a physical group after the logical transfer has failed;
            // runDeferredArqRefill drains the ARQ window once this call unwinds.
            burst_mode_active_ = false;
            burst_tx_buffer_.clear();
            return;
        }
    }

    // Fill the remainder of the budget with new chunks. The budget counts the whole
    // group, so burst_tx_buffer_.size() (holes already buffered) is the running total.
    while (arq_.isReadyToSend() && file_transfer_.hasMoreChunks()) {
        const size_t in_burst =
            burst_mode_active_ ? burst_tx_buffer_.size() : 0;
        if (in_burst >= burst_frame_cap) {
            break;
        }
        Bytes chunk = file_transfer_.getNextChunk();
        if (chunk.empty()) {
            break;
        }

        // MORE_FRAG indicates more data remaining in file (not burst). FINAL
        // marks the actual stream tail for the short SACK timer.
        const bool has_more = file_transfer_.hasMoreChunks();
        uint8_t flags = has_more ? v2::Flags::MORE_FRAG : v2::Flags::FINAL;
        bool sent = false;
        if (is_ofdm) {
            sent = arq_.sendFixedDataWithFlags(chunk, flags);
        } else {
            sent = arq_.sendDataWithFlags(chunk, flags);
        }
        if (sent) {
            noteDataTurnPayloadStarted(chunk.size());
        }
    }

    // Flush burst buffer
    if ((is_ofdm || is_mc_dpsk) && on_transmit_burst_) {
        burst_mode_active_ = false;
        flushBurstBuffer(
            retransmitted_frames > 0,
            descriptor_only_partial_repair && retransmitted_frames > 0);
    }
}

// §15 step 4d-ii: receiver-side tone-burst ACK handoff. Mirrors the OFDM
// GROUP_ACK arrival path at connection.cpp:2516, but takes a ToneBurstAckDetection
// (already decoded by the StreamingDecoder's monitor — no frame parse needed).
//
// Quality mapping for the rate controller:
//   - ACK  -> quality 1.0 (clean decode; the receiver couldn't have CRC-verified
//             a tone-burst payload otherwise). The 3-bit rate_hint field in the
//             payload is NOT consumed yet — that's a future refinement; for v1
//             the ACK/NACK semantic is all we use.
//   - NACK -> quality 0.0 (receiver couldn't decode -> step rate down).
//
// group_seq is 6 bits on the wire (mod 64). For files with <= 64 groups (~1 MB
// at QPSK R3/4) this matches the in-flight group exactly. Beyond that, the
// caller must extend the wire format; not gated for v1.
bool Connection::isToneBurstAckCandidatePlausible(
    const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload) const {
    if (state_ != ConnectionState::CONNECTED || disconnect_teardown_active_) {
        return false;
    }

    // The reserved WAITING-REBASE voice is intentionally outside the sender's
    // ordinary SACK support. It is the one legitimate NACK-typed tone accepted by
    // onToneBurstAck(), and must remain audible while the ARQ grid is unanchored.
    if (rx_rate_cmd_enabled_ &&
        payload.type == ultra::waveform::tone_burst_ack::AckType::Nack &&
        payload.rung_cmd ==
            ultra::waveform::tone_burst_ack::kRungCmdReserved &&
        payload.frame_mask == 0 && payload.rate_hint == 0 &&
        payload.drive_advisory ==
            ultra::waveform::tone_burst_ack::kDriveAdvisoryHold) {
        return true;
    }
    if (payload.type != ultra::waveform::tone_burst_ack::AckType::Ack) {
        return false;
    }
    if (!kInteractiveToneAckEnabled()) return false;
    return arq_.isToneBurstAckPlausible(
        payload.group_seq, payload.frame_mask, payload.move_epoch);
}

bool Connection::onToneBurstAck(
    const ultra::waveform::tone_burst_ack::ToneBurstAckDetection& detection) {
    MessageTxStatusDeferralGuard status_guard(*this);
    if (disconnect_teardown_active_) {
        return false;
    }
    // TXLAT (ULTRA_TXLAT_DIAG): timestamp the two boundaries that bracket a half-duplex
    // turnaround on the SENDER's own clock, so the interval can be decomposed without
    // mixing clocks across stations. This measurement retired two wrong figures on
    // 2026-07-30 (see CHANGELOG): encode+ARQ is 62-89 ms, not 2.5 s, and measured airtime
    // is 81-84% of wall clock rather than the 68% a modelled-airtime budget implied.
    // WARN level so it survives the default log level; env-gated so it costs nothing off.
    if (std::getenv("ULTRA_TXLAT_DIAG")) {
        const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        LOG_MODEM(WARN, "TXLAT ack_heard us=%lld", static_cast<long long>(now_us));
    }

    if (state_ != ConnectionState::CONNECTED) return false;

    // WAITING-REBASE voice (BUG-UNANCHORED-SILENCE-ESCAPE, design §5.3, gated on
    // ULTRA_RX_RATE_CMD): rung_cmd==3 is NOT an ack — it is the unanchored peer's
    // only utterance ("I am alive, forward data is arriving, but the era-base frame
    // keeps dying — resend it"). Consume it FIRST and consume it WHOLE: its mask is
    // meaningless (must not reach the ARQ as a SACK) and it must not feed the rate
    // controller a 0 (the voice is proof the forward link WORKS — treating it as
    // demote evidence would re-manufacture the exact collapse it exists to prevent).
    if (rx_rate_cmd_enabled_ &&
        detection.payload.type ==
            ultra::waveform::tone_burst_ack::AckType::Nack &&
        detection.payload.rung_cmd ==
            ultra::waveform::tone_burst_ack::kRungCmdReserved &&
        detection.payload.frame_mask == 0 &&
        detection.payload.rate_hint == 0 &&
        detection.payload.drive_advisory ==
            ultra::waveform::tone_burst_ack::kDriveAdvisoryHold) {
        const int seen = static_cast<int>(detection.payload.group_seq);
        // Reset the collapse evidence every voice copy: silence-while-unanchored is
        // by design, not a forward crater (the E1/D1/D3 manufactured demotes).
        zero_progress_rounds_ = 0;
        if (seen != rx_rebase_voice_seq_seen_) {
            rx_rebase_voice_seq_seen_ = seen;
            LOG_MODEM(WARN,
                      "Connection: WAITING-REBASE voice from peer (seq=%d) — "
                      "re-sending era base standalone; zero-progress evidence reset",
                      seen);
            arq_.expireBaseSlotTimerForRebase();
        }
        return true;  // consumed whole — no ARQ sack, no controller feed, no advisory
    }

    // FOREIGN-SEMANTICS gate (BUG-TONEACK-FABRICATION, F116 2026-07-05): nothing on
    // the unified path emits a Nack-TYPED tone burst — a crater'd group still acks as
    // an Ack-typed no-progress SACK (sendSack: base = rx_base-1, holes in the mask),
    // and the only real Nack producer is the WAITING-REBASE voice, whose group_seq is
    // a BURST-GROUP ordinal in a DIFFERENT sequence space (consumed above when
    // ULTRA_RX_RATE_CMD is on). A Nack-typed detection reaching this point is foreign
    // or corrupt (F116: a stale-audio re-decode at the wrong symbol_ms rung that
    // fluked Costas+Hamming+CRC-12) — its fields must NEVER be indexed into the ARQ
    // window, the rate controller, or the drive advisory. Consume it whole; worst
    // case is one lost real ack, which the RTO covers.
    if (detection.payload.type == ultra::waveform::tone_burst_ack::AckType::Nack) {
        LOG_MODEM(WARN,
                  "Connection: Nack-typed tone-burst detection dropped whole "
                  "(group_seq=%u mask=0x%X) — foreign or corrupt",
                  detection.payload.group_seq,
                  detection.payload.frame_mask);
        return true;
    }

    // The flagged ACK which LAUNCHED a startup probe may be re-detected while the
    // resulting target group is already on air/waiting for its own verdict.  It is
    // ARQ-support-valid as a base-1/no-progress copy, so letting it reach the generic
    // tone-SACK path can schedule another file refill and put a second R2/3 group on
    // air before the first probe reaches RTO.  Once the one physical probe is airborne,
    // every flagged command is necessarily launch-ACK diversity, not a target outcome
    // (target success/rollback ACKs are deliberately unflagged). Consume it whole.
    if (tx_latent_startup_probe_active_ &&
        tx_latent_startup_probe_airborne_) {
        const uint8_t authority_word = static_cast<uint8_t>(
            (detection.payload.rate_hint & 0x7) |
            ((detection.payload.rung_cmd & 0x3) << 3));
        if ((authority_word & kRungAuthorityStartupProbeFlag) != 0) {
            LOG_MODEM(INFO,
                      "Connection: duplicate LATENT startup launch ACK consumed while "
                      "one-shot target group awaits verdict");
            return true;
        }
    }

    // TRANSPORT MERGE (step 1): when interactive tone-burst acks are enabled and we have
    // interactive SR-ARQ frames in flight (no active burst owns this ack), route it to
    // the ARQ window. The ARQ reconstructs the seq and drives its normal ack path.
    //
    // COALESCE THE REFILL: a cumulative ack of N frames fires on_send_complete_ N times,
    // each of which would otherwise call sendNextFileChunk() immediately — fragmenting
    // the next window into N tiny bursts (the "3+1+1+1" / shorter-burst chains observed).
    // Bracket the ack with the SAME arq_callback_defer_refill_ guard processArqFrame uses
    // so all N completions defer to ONE runDeferredArqRefill() → ONE budget-sized burst.
    if (kInteractiveToneAckEnabled() && arq_.getTxInFlightBytes() > 0) {
        const bool outermost = !arq_callback_defer_refill_;
        if (outermost) arq_callback_defer_refill_ = true;
        // Snapshot exact latest-round ownership BEFORE this ACK mutates the ARQ.
        // PSDR must prove that this accepted ACK, not an earlier classic/control
        // SACK, changed a member of the immediately preceding physical group.
        const size_t partial_sack_unacked_before_ack =
            arq_.countUnackedFrameIdentities(
                partial_sack_last_round_.arq_frame_identities);
        // §RETX-PACING §1.1: re-arm the progress sentinel FIRST so the reading below is
        // exactly what THIS ack produced — a leftover value from any non-round ack path
        // (e.g. a control-frame SACK through processArqFrame) must not leak into round
        // accounting when this ack gets dedup/stale-dropped inside handleAckFrame.
        arq_.consumeAckProgress();
        // MOVE-EPOCH: hand the payload's era echo (bits 40-41) to the ARQ, which
        // folds it into the synthetic SACK and gates retirement on it (no-op OFF).
        const bool accepted = arq_.onToneBurstAck(
            detection.payload.group_seq, detection.payload.frame_mask,
            detection.payload.move_epoch);
        if (!accepted) {
            // ARQ rejected an impossible/stale-era payload. Treat it exactly like ACK
            // loss: it must not steer the rate, ALC, retry pacing, or file refill. A
            // same-era identical keepalive is accepted by ARQ and still reaches the
            // file-tail pre-RTO escape below.
            if (outermost) arq_callback_defer_refill_ = false;
            return false;
        }
        // §14.43: feed the RateController the RECEIVER's GRADED decode headroom carried in
        // rate_hint (0..7 -> [0,1]), not a binary ack/nack — restoring the closed loop the
        // unification cut. A NACK (group lost) still feeds 0. (Replaces the legacy GROUP_ACK
        // quality byte; same controller, now on the unified tone-burst path.)
        // RX-RATE-CMD Phase 2: snapshot the mode BEFORE the controller runs — one ACK is
        // ONE piece of channel evidence, so if the EMA/QAM16 machinery already moved the
        // rung on this ACK, the piggybacked command (same evidence, measured receiver-side)
        // must not fire a second move (maybeApplyRxRateCommand compares against these).
        const Modulation mod_at_ack = data_modulation_;
        const CodeRate rate_at_ack = data_code_rate_;
        const WaveformMode mode_at_ack = negotiated_mode_;
        const int logical_cw_at_ack = data_frame_cw_count_;
        const int physical_cw_at_ack = physicalDataFrameCodewords();
        const int lifting_z_at_ack = selectBurstLiftingZ();
        // RX-AUTHORITY (ULTRA_RX_RATE_AUTHORITY): the receiver commands the rung
        // outright — the ACK's [rate_hint|rung_cmd] bits are its ABSOLUTE canonical
        // rung index, and the sender's own mid-transfer drivers (the EMA walk, the
        // dense demote/crest walks, the climb hop, trough amnesty, the relative
        // rung command) are INERT: one decision-maker, sitting where the channel is
        // actually measured. Ack-SILENCE safety rails (collapse escape, stuck-frame
        // escape, RTO machinery) stay live — no command crosses a blackout.
        const bool rx_authority = rxRateAuthorityEnabled();
        if (rx_authority) {
            const uint8_t cmd_word = static_cast<uint8_t>(
                (detection.payload.rate_hint & 0x7) |
                ((detection.payload.rung_cmd & 0x3) << 3));
            if (acceptAuthorityCommandRevision(
                    cmd_word, detection.payload.group_seq,
                    detection.payload.frame_mask, detection.payload.move_epoch)) {
                // The receiver updates its clean-group streak before pricing this
                // command.  Sender round accounting runs later in this callback so
                // refill remains coalesced, but target-burst economics must include
                // the same just-accepted clean round.  Otherwise the selector can
                // price the escalated N=8 geometry while this guard sees stale N=5.
                const bool accepted_clean_round =
                    arq_.lastAckProgressFrames() > 0 &&
                    arq_.getTxInFlightBytes() == 0;
                maybeObeyAuthorityCommand(cmd_word, accepted_clean_round);
            }
        } else {
            const float fed_quality =
                detection.payload.type ==
                        ultra::waveform::tone_burst_ack::AckType::Nack
                    ? 0.0f
                    : static_cast<float>(detection.payload.rate_hint) / 7.0f;
            applyAdaptiveRateFeedback(fed_quality);
        }
        // Software-ALC sender side (BUG-QAM16-RIG-LEVEL-BUDGET): the receiver's
        // drive advisory rides bits [30..31] of this ACK. Hand a non-hold advisory
        // to the host (the host owns tx_drive), which applies at most ONE step per
        // ACKed group (dedup by group_seq), clamped to [configured baseline, 0.85].
        // Advisory 3 (reserved) is treated as hold. ACK loss is stateless-safe:
        // the advisory simply doesn't arrive and the next group's ACK re-derives it.
        {
            const uint8_t advisory = detection.payload.drive_advisory;
            if (connection_policy::softwareAlcEnabled() && on_drive_advisory_ &&
                (advisory == ultra::waveform::tone_burst_ack::kDriveAdvisoryUp ||
                 advisory == ultra::waveform::tone_burst_ack::kDriveAdvisoryDown)) {
                on_drive_advisory_(advisory, detection.payload.group_seq);
            }
        }
        // RX-RATE-CMD Phase 2 (ULTRA_RX_RATE_CMD): consume the receiver's rung command
        // (bits 42-43) — ADVISORY input through the sender's own guards, never a blind
        // obey. Runs INSIDE the defer-refill bracket so a committed demote's refill
        // coalesces into the single outermost runDeferredArqRefill below (which then
        // sends the [holes]+[new] burst at the NEW rung). Hard no-op while OFF.
        // RX-AUTHORITY supersedes it: bits 42-43 are then command bits already
        // consumed above, not a relative demote.
        if (!rx_authority) {
            maybeApplyRxRateCommand(detection.payload.rung_cmd,
                                    detection.payload.group_seq,
                                    mod_at_ack, rate_at_ack);
        }
        if (outermost) {
            arq_callback_defer_refill_ = false;
            // §RETX-PACING §1.1 round boundary: every tone-burst ack ends a resend round.
            // Read the ARQ's identity-agnostic progress (base advance + new SACK bits;
            // −1 = the ack was dedup/stale-dropped ⇒ NOT a round) exactly once. Progress
            // resets the streak + releases any hold; a zero-progress round counts toward
            // the collapse escape and (knob-gated) arms the trough deferral BEFORE the
            // refill below, so the refill latches instead of re-blasting into the trough.
            const int round_progress = arq_.lastAckProgressFrames();
            arq_.consumeAckProgress();
            noteArqRoundOutcome(round_progress, "toneburst-ack");
            // TROUGH AMNESTY: progress after a zero-progress episode = the null ended;
            // restore the pre-episode rung (see maybeTroughAmnesty). Inside the same
            // defer-refill bracket, so the restored rung rides the very next refill.
            // RX-AUTHORITY: inert — the receiver re-commands the right rung on the
            // first post-trough ACK; a second restorer would double-drive.
            if (!rx_authority) {
                maybeTroughAmnesty(round_progress, detection.payload.rung_cmd);
            }
            // Default-OFF descriptor-only repair: arm exactly one synchronous refill
            // only when this ACK proves partial forward progress for the immediately
            // preceding descriptor-bearing physical group.  Reserved advisory=3 is
            // the receiver's CRC-covered exact-k/M provenance; every missing or
            // ambiguous input fails closed to the established double/full anchor.
            const bool exact_group_provenance =
                detection.payload.drive_advisory ==
                ultra::waveform::tone_burst_ack::kDriveAdvisoryReserved;
            const bool holes_remain = arq_.getTxInFlightBytes() > 0;
            const size_t prior_round_unacked_after_ack =
                arq_.countUnackedFrameIdentities(
                    partial_sack_last_round_.arq_frame_identities);
            const size_t prior_round_delivered =
                partial_sack_last_round_.arq_frame_identities.size() -
                prior_round_unacked_after_ack;
            const bool fresh_partial_progress =
                partial_sack_last_round_.arq_frames > 1 &&
                partial_sack_unacked_before_ack ==
                    partial_sack_last_round_.arq_frame_identities.size() &&
                prior_round_unacked_after_ack > 0 &&
                prior_round_unacked_after_ack < partial_sack_unacked_before_ack;
            const bool same_ack_geometry =
                partial_sack_last_round_.mode == mode_at_ack &&
                partial_sack_last_round_.modulation == mod_at_ack &&
                partial_sack_last_round_.code_rate == rate_at_ack &&
                partial_sack_last_round_.logical_cw == logical_cw_at_ack &&
                partial_sack_last_round_.physical_cw == physical_cw_at_ack &&
                partial_sack_last_round_.lifting_z == lifting_z_at_ack;
            const bool geometry_still_current =
                negotiated_mode_ == mode_at_ack &&
                data_modulation_ == mod_at_ack &&
                data_code_rate_ == rate_at_ack &&
                data_frame_cw_count_ == logical_cw_at_ack &&
                physicalDataFrameCodewords() == physical_cw_at_ack &&
                selectBurstLiftingZ() == lifting_z_at_ack;
            const bool transition_clear =
                !mode_change_pending_ && !desc_switch_full_anchor_pending_ &&
                staged_timeout_batch_.empty() &&
                !tx_latent_startup_probe_active_;
            partial_sack_descriptor_repair_scope_ =
                partial_sack_descriptor_repair_enabled_ &&
                partial_sack_last_round_.descriptor_light_repair_eligible &&
                exact_group_provenance && holes_remain &&
                fresh_partial_progress && same_ack_geometry &&
                geometry_still_current && transition_clear;
            if (partial_sack_descriptor_repair_enabled_ && holes_remain) {
                LOG_MODEM(INFO,
                          "Connection: PARTIAL-SACK descriptor-only decision "
                          "eligible=%d provenance=%d arq_progress=%d "
                          "physical_delivery=%zu/%zu unacked_before=%zu prior_ok=%d "
                          "same_ack_geometry=%d current_geometry=%d transition_clear=%d",
                          partial_sack_descriptor_repair_scope_ ? 1 : 0,
                          exact_group_provenance ? 1 : 0, round_progress,
                          prior_round_delivered,
                          partial_sack_last_round_.arq_frame_identities.size(),
                          partial_sack_unacked_before_ack,
                          partial_sack_last_round_.descriptor_light_repair_eligible ? 1 : 0,
                          same_ack_geometry ? 1 : 0,
                          geometry_still_current ? 1 : 0,
                          transition_clear ? 1 : 0);
            }
            // STOP-AND-WAIT: every tone-burst ack is a TURN boundary — it's now our turn
            // to send the next burst (resend remaining holes + new frames). Trigger the
            // refill even when the cumulative base did NOT advance: a SACK with a hole at
            // base (e.g. "I have 4,5, missing 3") doesn't fire on_send_complete_, but it's
            // still our turn to re-send the hole. sendNextFileChunk() coalesces
            // [holes]+[new] into one budget burst; an empty turn (nothing to send) no-ops.
            // FILE is deliberately different from fragmented messages here.  A receiver
            // keepalive re-emits the same cumulative SACK after a repair/ACK was lost, so
            // ARQ correctly reports -1 even though the waveform is a fresh physical turn.
            // Refill files on *any* detected tone ACK to preserve that pre-RTO stall
            // escape.  The physical tone monitor consumes at most one detection per arm;
            // the wire format has no round nonce with which Connection could distinguish
            // a fresh identical keepalive from a delayed copy by payload alone.
            if (file_transfer_.getState() == FileTransferState::SENDING) {
                deferred_file_refill_ = true;
            }
            // Fragmented messages use the same stop-and-wait tone-SACK turns as files.
            // A FINAL message frame can expose a base hole after every original
            // fragment has already been submitted; no completion callback fires for
            // that hole, so explicitly schedule the [holes]+[new] refill here. A
            // duplicate/stale ACK reports -1 and must not launch the same repair twice.
            if (round_progress >= 0 && isOFDMMode(negotiated_mode_) &&
                !pending_tx_fragments_.empty() &&
                arq_.getTxInFlightBytes() > 0) {
                deferred_fragment_refill_ = true;
            }
            runDeferredArqRefill();
            // runDeferredArqRefill may legitimately latch behind pacing, turn, or
            // mode-control gates. Exact provenance is not transferable to that future
            // physical request, so a no-op/deferred refill always discards the token.
            partial_sack_descriptor_repair_scope_ = false;
        }
        return true;
    }

    // Ack arrived with no in-flight ARQ bytes (nothing to advance) → nothing to do.
    return false;
}

// Retransmit depth at which a stuck in-flight frame triggers a one-rung ESCAPE-drop. 5 is well
// above the 1-3 retx a frame needs at a SUSTAINABLE rate on a fading channel, and well below
// max_retries (15) — so a frame the current (over-climbed) rate genuinely cannot push through
// drops to a robust rung before it dies, but ordinary Moderate retx don't trip it.
// ULTRA_STUCK_ESCAPE_RETX [2..10] (2026-07-02 campaign A/B knob): the Phase-0 forensics measured
// the 5-retx trigger costing ~84 s of frozen-base blind re-blast per 16QAM collapse (sim g43 and
// live rig MPG@20 both) — each retx round at 672 ms/frame x8 + RTO is a whole group-time spent
// re-sending into the same trough. Lower = earlier demote at the cost of occasionally fleeing a
// rung a lucky retx would have salvaged.
static int stuckRetransmitEscape() {
    static const int v = [] {
        if (const char* e = std::getenv("ULTRA_STUCK_ESCAPE_RETX")) {
            const int n = std::atoi(e);
            if (n >= 2 && n <= 10) return n;
        }
        return 5;
    }();
    return v;
}

// ═══════ Retx trough pacing + collapse-conditioned escape (docs/RETX_PACING_DESIGN_2026_07_03.md) ═══════
// ULTRA_RETX_TROUGH_PACING (default OFF ⇒ byte-identical): master switch for the §1
// trough-aware resend deferral — after a ZERO-progress round (whole key-down failed ⇒
// trough-conditioned), hold the next resend ~Tc so the channel decorrelates from the state
// that just killed it, instead of re-blasting the same frames into the same trough (the
// measured 6.3-retx/delivered collapse waste). Read ONCE (static).
static bool retxTroughPacingEnabled() {
    static const bool v = [] {
        const char* e = std::getenv("ULTRA_RETX_TROUGH_PACING");
        return e == nullptr || std::atoi(e) != 0;  // DEFAULT-ON 2026-07-05
    }();
    return v;
}

// ULTRA_TROUGH_DEFER_TC_FRAC [0.25..4.0] (default 1.0): `frac` in
// T_defer(n) = clamp(frac·Tc·2^(n−1) − elapsed, 0, T_cycle/2) — §1.2. Out-of-range/garbage
// values fall back to 1.0. Read ONCE (static).
static float troughDeferTcFrac() {
    static const float v = [] {
        if (const char* e = std::getenv("ULTRA_TROUGH_DEFER_TC_FRAC")) {
            const float f = static_cast<float>(std::atof(e));
            if (std::isfinite(f) && f >= 0.25f && f <= 4.0f) return f;
        }
        return 1.0f;
    }();
    return v;
}

// ULTRA_COLLAPSE_ESCAPE_ROUNDS 0 (OFF, default ⇒ byte-identical) or [2..8]: N consecutive
// zero-progress rounds at the current rung (with ≥ half the burst budget in flight) ⇒
// escape-drop the rung (§2). Zero-DELIVERED evidence, not retry depth — the rejected
// ULTRA_STUCK_ESCAPE_RETX=3 hair-trigger fled rungs that were delivering (g42 −28%); the
// round condition cannot trip on a lone straggler amid deliveries (§2.3). The existing
// 5-retx per-frame backstop (stuckRetransmitEscape above) stays untouched. Read ONCE (static).
static int collapseEscapeRounds() {
    static const int v = [] {
        if (const char* e = std::getenv("ULTRA_COLLAPSE_ESCAPE_ROUNDS")) {
            const int n = std::atoi(e);
            if (n >= 2 && n <= 8) return n;
        }
        return 2;  // DEFAULT 2026-07-05 (campaign standing value; 0 opts out)
    }();
    return v;
}

// QAM16 R2/3 cross-modulation climb (ULTRA_QAM16_CLIMB). Climb to QAM16 only after this many
// CONSECUTIVE clean groups (quality >= climb_above) while pinned at the QPSK R3/4 top rung — a
// modulation hop costs a full MODE_CHANGE (T/R + re-anchor) and QAM16 is fragile. Default 2
// (history 8 -> 4 on 2026-06-17, 4 -> 2 on 2026-07-02 fade-riding ladder: the crests of a
// Good ~10-20 s fade cycle only last a few ~8 s groups, so a 4-group streak forfeited most of
// each crest; 2 clean groups is the fastest gate that still requires a SUSTAINED reading, and
// the immediate demote + re-climb cooldown are the safety net). Env-tunable for rig sweeps via
// ULTRA_QAM16_CLIMB_STREAK [1..64]; lower = faster switch but a weaker Good-vs-Moderate proxy.
// Demote off QAM16 IMMEDIATELY on a bad group (kQam16DemoteBadStreak=1, was 2 — fade-riding
// needs the trough exit to be as prompt as the cliff NACK exit) or on a NACK.
// The ULTRA_QAM16_R34 crest-rung walk (QAM16 R2/3 -> R3/4, default-OFF) reuses this same
// streak length as its climb gate — one clean-evidence constant for both upward moves.
// MID-RUNG demote landing (ULTRA_QAM16_DEMOTE_MIDRUNG, default OFF; F102 finding):
// with the 16QAM R1/2 rung ladder-enabled, a crater/escape at 16QAM R2/3+ lands at
// 16QAM R1/2 (raw ~2.45k, ~2x the FEC margin) instead of skipping to QPSK R3/4
// (~2.05k raw). The channel that craters R2/3 often still carries R1/2 — that is
// the margin argument the epoch-stats exposed (broadband ~24 dB, damage-limited).
// If R1/2 craters too, the NEXT command lands QPSK R3/4 (one extra ~12 s step).
// Requires ULTRA_ENABLE_QAM16_LADDER (the rung must be selectable at all).
static bool qam16DemoteMidrungEnabled() {
    static const bool v = [] {
        const char* a = std::getenv("ULTRA_QAM16_DEMOTE_MIDRUNG");
        const char* b = std::getenv("ULTRA_ENABLE_QAM16_LADDER");
        return !(a && a[0] == '0') && !(b && b[0] == '0');  // DEFAULT-ON 2026-07-05
    }();
    return v;
}

static int qam16ClimbStreak() {
    static const int v = [] {
        if (const char* e = std::getenv("ULTRA_QAM16_CLIMB_STREAK")) {
            const int n = std::atoi(e);
            if (n >= 1 && n <= 64) return n;
        }
        return 1;  // DEFAULT 2026-07-05 (campaign standing value, was 2)
    }();
    return v;
}
static constexpr int kQam16DemoteBadStreak = 1;

// Re-climb cooldown BASE after a QAM16 demote, in CLEAN groups (quality >= climb_above).
// ULTRA_QAM16_RECLIMB_COOLDOWN [0..64]; 0 = no cooldown. Doubles per demote this connection
// (cap x4) via noteQam16Demoted — see the member comment in connection.hpp and the move-
// overhead arithmetic in CHANGELOG 2026-07-02.
static int qam16ReclimbCooldownBase() {
    static const int v = [] {
        if (const char* e = std::getenv("ULTRA_QAM16_RECLIMB_COOLDOWN")) {
            const int n = std::atoi(e);
            if (n >= 0 && n <= 64) return n;
        }
        return 1;  // DEFAULT 2026-07-05 (campaign standing value, was 3)
    }();
    return v;
}

// Register a QAM16 demote and arm the re-climb cooldown. weight=1 for the ack-driven soft
// demote; weight=2 for the escape-drop (a frame nearly DIED at QAM16 — stronger evidence, so
// the backoff advances two steps). Cooldown = base << min(demotes-1, 2): 3, 6, 12, 12...
void Connection::noteQam16Demoted(int weight) {
    qam16_demote_count_ = std::min(qam16_demote_count_ + weight, 8);
    const int scale = 1 << std::min(qam16_demote_count_ - 1, 2);
    qam16_reclimb_cooldown_ = qam16ReclimbCooldownBase() * scale;
    qam16_clean_streak_ = 0;
    qam16_r34_clean_streak_ = 0;  // leaving QAM16 — any pending crest-rung walk dies with it
}

bool Connection::rateAdaptationActive() const {
    if (!adaptive_rate_enabled_) return false;
    // enterConnected(false) is the QSO-scoped operator-profile pin established by
    // either serialized CONNECT fields or a valid environment force.  It must gate
    // every actuator (receiver authority, escapes and the legacy controller), not just
    // the latent startup probe.  Keep the live env check too so a force applied while
    // connected fails closed; an invalid value is AUTO and does not freeze anything.
    if (!latent_startup_probe_allowed_ || environmentForcesDataProfile()) return false;
    const char* l = std::getenv("ULTRA_LOCK_RATE");
    if (l != nullptr && std::atoi(l) != 0) return false;  // operator pin always wins
    const char* e = std::getenv("ULTRA_RATE_ADAPT");
    if (e != nullptr) return std::atoi(e) != 0;  // explicit override (any OFDM mode, as before)
    // DEFAULT (2026-07-02, fade-riding ladder): ON for connected wideband OFDM — the
    // burst-transport file path whose clean-boundary gate (06-10), synchronized
    // MODE_CHANGE (06-09), ssthresh ceiling (06-11) and QAM16 climb/demote machinery
    // (06-17) are GUI-proven. MC-DPSK and OFDM_NARROW keep their fixed negotiated rate
    // unless explicitly enabled (their adaptation is unvalidated on the faithful gate).
    return negotiated_mode_ == WaveformMode::OFDM_CHIRP;
}

// Shared escape ACTION (§RETX_PACING_DESIGN_2026_07_03 §2.2 — refactored out of
// maybeEscapeStuckFrame so the collapse-conditioned round escape REUSES it, not forks it).
// All guards (rateAdaptationActive, CONNECTED, mode_change_pending_, floor, in-flight)
// stay with the CALLERS — they keep first refusal exactly as before.
//
// DESC-SWITCH Phase-1 scope gate (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §7):
// the escape drops fire MID-WINDOW (frames in flight, bypassing the clean-boundary gate
// BY DESIGN) — a descriptor-committed regrid here is only era-safe under the move-epoch
// machinery (ULTRA_ARQ_MOVE_EPOCH). These sender-initiated escapes stay on the legacy
// requestModeChange exchange in EVERY knob state — DELIBERATELY, Phase 2 included: they
// fire on zero-ACK evidence, i.e. precisely when the tone-burst control plane from the
// peer has gone silent, so a descriptor-only (announce-and-hope) commit would be aimed
// at a receiver that is demonstrably not confirming reception — the synchronized
// exchange doubles as the deaf-peer escalation (§6.5/§6 row 7). The RECEIVER-commanded
// demote (RX-RATE-CMD Phase 2, maybeApplyRxRateCommand below) is the mid-window case
// that DOES ride the descriptor commit: a command in hand proves the reverse control
// channel is alive, and the ARQ abort inside commitLocalModeSwitch bumps the epoch
// (setCodeRate rate-abort, or the abortPendingTx payload-drop bump for same-rate regrids — 2026-07-04 fix) for era safety.
void Connection::executeEscapeDrop(const char* trigger) {
    if (data_modulation_ == Modulation::QAM16 ||
        data_modulation_ == Modulation::QAM8) {
        // 8PSK revival (2026-07-05): QAM8 takes the same dense-constellation escape
        // EXIT as QAM16 (never the rate walk — the probe measured the walk sliding
        // QAM8 to the strictly-dominated R1/4). Differences handled below: no
        // noteQam16Demoted (QAM8 has no climb-in cooldown to meter) and no midrung
        // landing (that lever targets 16QAM R1/2).
        const bool esc_is_qam16 = data_modulation_ == Modulation::QAM16;
        // QAM16 top-gear stuck on a fade. When QAM16 craters off the decodability cliff it may emit
        // NO tone-burst ack at all, so the ack-driven demote in applyAdaptiveRateFeedback never sees
        // it — this escape path is the only one that fires. Demote STRAIGHT to the robust QPSK R3/4
        // home gear (not a QAM16 code-rate step — this applies from EITHER QAM16 rate, so the
        // ULTRA_QAM16_R34 crest rung takes the same straight exit) and arm a DOUBLE-weight re-climb
        // cooldown (a frame nearly died — stronger evidence than a soft demote, but not a permanent
        // forfeit of the next fade crest). requestModeChange re-anchors both stations at a clean
        // boundary.
        // MID-RUNG landing (ULTRA_QAM16_DEMOTE_MIDRUNG): the escape lands at 16QAM
        // R1/2 (2x margin, stays on QAM16 ⇒ no reclimb cooldown) unless already
        // there — a second escape then takes the QPSK R3/4 exit below.
        const bool midrung_exit = esc_is_qam16 &&
            qam16DemoteMidrungEnabled() && data_code_rate_ != CodeRate::R1_2;
        Modulation esc_mod =
            midrung_exit ? Modulation::QAM16 : Modulation::QPSK;
        CodeRate esc_rate =
            midrung_exit ? CodeRate::R1_2 : CodeRate::R3_4;
        // F145/F147 ENABLED-LADDER: escape landings are fixed rungs — snap them
        // through the enabled table like every other rung-index consumer.
        {
            const uint8_t esc_snapped = snapRungIndexDownToEnabled(
                coherentRungIndexFor(esc_mod, esc_rate));
            if (esc_snapped != kRungIdxNone) {
                const CoherentPick p = coherentRungFromIndex(esc_snapped);
                esc_mod = p.mod;
                esc_rate = p.rate;
            }
        }
        // F147 log-truth + no-op guard: snapshot the CURRENT rung BEFORE the
        // descriptor commit mutates data_modulation_/data_code_rate_ — the old
        // log read the post-commit state and printed "16QAM R1/2 -> 16QAM R1/2"
        // for a real R2/3 -> R1/2 drop. And if the escape target IS the current
        // rung (e.g. it raced an authority obey that already landed there),
        // skip: re-committing burns an epoch bump + re-encode for nothing.
        const Modulation esc_from_mod = data_modulation_;
        const CodeRate esc_from_rate = data_code_rate_;
        if (esc_mod == esc_from_mod && esc_rate == esc_from_rate) {
            LOG_MODEM(WARN,
                      "Connection: ESCAPE-drop target %s %s == current rung (%s) — "
                      "already landed (authority/escape race), skipping",
                      modulationToString(esc_mod), codeRateToString(esc_rate), trigger);
            return;
        }
        if (!midrung_exit && esc_is_qam16) noteQam16Demoted(2);
        // Silence is negative evidence: unlike a receiver-carried rung command,
        // it does not prove the peer can hear a descriptor.  A unilateral regrid
        // here left the receiver slicing R1/2 as stale R2/3 for 96 s in the live
        // IONOS transfer.  Always use the synchronized MODE_CHANGE exchange; the
        // peer keeps the old geometry until it decodes and ACKs the request.
        ++consecutive_escape_drops_;
        LOG_MODEM(WARN, "Connection: ESCAPE-drop %s %s -> %s %s (%s) via MODE_CHANGE",
                  modulationToString(esc_from_mod),
                  codeRateToString(esc_from_rate), modulationToString(esc_mod),
                  codeRateToString(esc_rate), trigger);
        // Wire SNR embed: freshness-gated pool aggregate under ULTRA_WIRE_SNR_FRESH
        // (stale sentinel -10 when nothing < 3*Tc), else the raw scalar (unchanged).
        requestModeChange(esc_mod, esc_rate, wireSnrDb(),
                          v2::ModeChangeReason::CHANNEL_DEGRADED);
        return;
    }

    // A frame/window the current (over-climbed) rate genuinely cannot push through: the fade
    // troughs are killing it. It produces NO group ACK, so the ack-driven RateController never sees
    // it, and the clean-boundary gate defers any change because the stuck frame keeps the window
    // busy — so without this it grinds to max_retries and fails the whole transfer (Moderate@18:
    // adapt climbed R1/4->R3/4, a frame stuck 248 retx and died; locked R1/4 completed). Force a
    // ONE-rung drop to a more robust rate so the frame can punch through. requestModeChange
    // re-anchors both stations; setCodeRate re-sends the in-flight frames at the new rate KEEPING
    // their seqs (tx_next=tx_base) so there is no receiver seq-hole; ssthresh (noteRungFailed) stops
    // the controller from immediately climbing back into the rung that just failed.
    const CodeRate robust = rate_controller_.moreRobustRung(data_code_rate_);
    if (robust == data_code_rate_) return;
    rate_controller_.noteRungFailed(data_code_rate_);
    const Modulation esc_from_mod = data_modulation_;
    const CodeRate esc_from_rate = data_code_rate_;
    ++consecutive_escape_drops_;
    LOG_MODEM(WARN, "Connection: ESCAPE-drop %s -> %s (%s) via MODE_CHANGE",
              codeRateToString(esc_from_rate), codeRateToString(robust), trigger);
    requestModeChange(esc_from_mod, robust, wireSnrDb(),
                      v2::ModeChangeReason::CHANNEL_DEGRADED);
}

// ═══════ RX-RATE-CMD Phase 2 — receiver rung command in the tone-burst ACK ═══════
// docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.2 as amended (knob
// ULTRA_RX_RATE_CMD, default OFF = byte-identical; SEMANTICS-BREAKING lockstep when
// ON: live payload bits 42-43 + the widened tone-ACK CRC span). Motivation (the
// Phase-1 D3 rig finding): with climbs ~free under the descriptor commit, the ESCAPE
// side became the measured bottleneck — the RECEIVER sees a 16QAM crater IMMEDIATELY
// (failed group decode) while the sender only learns after ~2 zero-progress rounds
// (~2×RTO); 4 climb/escape cycles in 90 s on a Moderate epoch. The command closes
// that gap: the verdict rides the cratered group's own tone-burst ACK — the 4-FSK
// control plane whose detection floor out-survives every OFDM waveform by ~15-20 dB
// on fading (§1.2), exactly the channel a trough verdict must cross.
//
// DELIBERATE deviations from the design doc's §5.2 sketch (stated in-doc too):
//   - NO UP command (the sketch's value 1 = STEP-UP): climbs stay sender-side where
//     the quality EMA + ssthresh + climb streaks live — a second upward driver would
//     double-drive the control loop (two integrators, one plant) and re-open the
//     2026-06-09 churn arm. Demote-only keeps the command channel single-purpose:
//     trough-escape latency. Wire encoding: 0 none / 1 DOWN-one / 2 DOWN-hard /
//     3 reserved-hold (tone_burst_constants.hpp kRungCmd*).
//   - Emit policy is CRATER-ONLY DOWN-hard: the receiver has NO quality EMA/streak
//     machinery (the RateController runs sender-side only) and building a parallel
//     estimator is an explicit anti-goal. DOWN-one is defined on the wire and
//     consumed (forward-compatible), but nothing emits it yet.

// RECEIVER side (called from onBurstGroupReceived, before the group's ACK emits).
// Crater predicate — both arms derived from EXISTING policy quantizations, no new
// numeric constants:
//   frame_mask == 0  ⇔ ZERO frames delivered — the decoder's whole-fail/fast-NACK
//                      signature and the same zero-progress evidence class the
//                      sender's collapse escape counts. No finer grade exists on
//                      this axis: the decoder assigns quality EXACTLY 0.0 to any
//                      !all_ok group (streaming_burst_interleave.cpp), so the
//                      3-bit rate_hint quantization already saturates at 0 for
//                      every partial-fail — mask==0 is the only receiver-side
//                      distinction between "cliff" and "ordinary fade losses".
//   QAM16 only       — the modulation whose demote-on-a-single-bad-group policy is
//                      already codified sender-side (kQam16DemoteBadStreak = 1: the
//                      decodability-cliff asymmetry). At QPSK rungs a zero group is
//                      indistinguishable from an irreducible deep null (fading loss
//                      is irreducible at ANY rate); commanding DOWN there would
//                      bypass the deliberately EMA-smoothed policy and re-introduce
//                      the 2026-06-09 single-NACK ratchet-to-R1/4.
void Connection::updateRxRateCommandFromGroup(bool all_ok, uint16_t frame_mask) {
    using ultra::waveform::tone_burst_ack::kRungCmdDownHard;
    using ultra::waveform::tone_burst_ack::kRungCmdDownOne;
    using ultra::waveform::tone_burst_ack::kRungCmdNone;
    if (all_ok) {
        // Clean group — the bad stretch (if any) ended; nothing may keep riding.
        qam16_rx_bad_streak_ = 0;
        rx_rate_cmd_pending_ = kRungCmdNone;
        return;
    }
    if (data_modulation_ != Modulation::QAM16) {
        qam16_rx_bad_streak_ = 0;
        return;  // deep null at a robust rung: irreducible fading, not a rate signal
    }
    ++qam16_rx_bad_streak_;
    if (frame_mask != 0) {
        // PARTIAL failure (frames delivered, group not clean). A single partial is a
        // fade brush — clear any standing command (the base advances, fresh seqs, the
        // dedup would no longer swallow it). But TWO consecutive failed groups at
        // 16QAM = the rung is under water even without a total crater — and the
        // sender's own detection of that state costs ~40 s of zero-progress rounds
        // (F27 paid it twice; partials never froze the base so no crater command
        // fired). Command DOWN-ONE once per bad pair; the next group event either
        // clears it (delivered) or re-arms it (still failing — a new seq, and the
        // sender's per-ACK mode-snapshot guard keeps it to one rung per ACK).
        rx_rate_cmd_pending_ =
            (qam16_rx_bad_streak_ >= 2) ? kRungCmdDownOne : kRungCmdNone;
        return;
    }
    // Total crater at the high-order mode → command a demote. Idempotent across
    // consecutive craters: the SAME command keeps riding every re-emitted ACK until
    // the sender's move is observed (applyDataMode clears the latch on a real
    // mod/rate change — the once-per-committed-move rule). That re-carry is
    // ACK-loss diversity for free and storm-safe: a crater freezes the ARQ base,
    // so every copy carries ONE group_seq and the sender acts once.
    //
    // CREST-RUNG graded landing (2026-07-04, F10 finding): a crater AT 16QAM R3/4
    // commands DOWN-ONE (→ 16QAM R2/3, the sender's DownOne mapping) — R2/3's
    // requirement sits ~1.5 dB below R3/4's, so a trough that kills the crest rung
    // by inches usually leaves R2/3 alive (F10: the pre-hop R2/3 groups ran 0.85-0.98
    // in the same epoch). One rung per evidence quantum; if R2/3 craters too, the
    // NEXT command is DOWN-hard to the QPSK home gear. R2/3 craters keep the
    // straight-to-QPSK exit (the decodability-cliff asymmetry).
    rx_rate_cmd_pending_ =
        (data_code_rate_ == CodeRate::R3_4)
            ? ultra::waveform::tone_burst_ack::kRungCmdDownOne
            : kRungCmdDownHard;
}

void Connection::failClosedLatentStartupProbeUnknown(const char* origin) {
    if (!latent_startup_probe_waiting_) return;

    latent_startup_probe_waiting_ = false;
    latent_startup_probe_spent_ = true;
    latent_startup_probe_clean_groups_ = 0;
    latent_startup_probe_pending_base_groups_ = 0;
    latent_startup_probe_rollback_pending_ = true;
    latent_startup_probe_failed_ = true;

    const uint8_t cur = coherentRungIndexFor(data_modulation_, data_code_rate_);
    rx_authority_cmd_ =
        (cur != kRungIdxNone && cur < kRungIdxQpskR12)
            ? cur
            : kRungIdxQpskR12;
    LOG_MODEM(WARN,
              "Connection: LATENT startup probe fail-closed (%s) — command idx %u; "
              "hold through one trusted clean base group",
              origin ? origin : "unknown physical outcome", rx_authority_cmd_);
}

// ═══════════ RX-AUTHORITY receiver verdict (ULTRA_RX_RATE_AUTHORITY) ═══════════
// Called per delivered/failed burst group, BEFORE the group's ACK emits. Maps the
// receiver's FRESH channel observation (broadband SNR EMA + coherence-adjusted
// fading, fed via setBurstChannelObservation from the decoder's per-frame atomics)
// through selectCoherentOFDM — the SAME anchor tables used at connect — into an
// absolute canonical rung command. Multi-rung moves in both directions are the
// point: the map's input is already alpha-smoothed, so stability lives in the
// measurement, not in streak counters. Two decode-evidence overrides keep the
// verdict honest when the meter and the decoder disagree:
//   - CRATER (quality <= 0): never command AT or ABOVE the rung that just failed —
//     clamp to one canonical step below the current rung, whatever the SNR map
//     says (the map's fading class lags a fresh trough).
//   - CLEAN group: never command BELOW the current rung (the rung is proven
//     working THIS group; a stale-low SNR reading must not thrash it downward).
void Connection::updateRxAuthorityCommand(bool all_ok, float quality, bool full_crater,
                                          float delivered_fraction,
                                          uint8_t delivered_frames,
                                          uint8_t group_size,
                                          bool geometry_proven) {
    if (state_ != ConnectionState::CONNECTED ||
        negotiated_mode_ != WaveformMode::OFDM_CHIRP) {
        rx_authority_cmd_ = kRungIdxNone;
        return;
    }
    if (burst_obs_snr_db_ < 0.0f) {
        // No fresh observation yet this connection — command nothing rather than
        // steer on handshake-stale state.
        rx_authority_cmd_ = kRungIdxNone;
        return;
    }
    // FADE-AVERAGED verdict SNR (2026-07-05, first-probe finding): the per-frame
    // broadband EMA swung 16.9..26.8 dB on a dial-20 Good channel — a fade
    // snapshot. Commanding on it aliases the fade cycle (crest reading -> climb ->
    // trough crater -> demote -> repeat; 12 moves/300 s measured). The rung anchors
    // are calibrated on dial-equivalent SNR, so the verdict input is the dB mean of
    // the last few group observations (~30 s ≈ many Tc). Decode-evidence overrides
    // below keep the fast reactions (crater = instant clamp).
    float inst_fading = connection_policy::coherenceAdjustedFadingIndex(
        (burst_obs_fading_ >= 0.0f) ? burst_obs_fading_ : fading_index_,
        burst_obs_coh_score_, burst_obs_coh_valid_);
    // ESTIMATOR HYGIENE (F142): a fading index > 1.5 is not a channel — it is the
    // across-carrier CV of a tone/noise snapshot (measured 2.9-3.1 on our own ACK
    // echo). One such sample in a 6-ring lifts the average by ~+0.45 and flips
    // the class. Reject absurd readings; reuse the last sane value.
    if (inst_fading > 1.5f) inst_fading = rx_auth_fading_passed_;
    // CENSORED failed-group SNR (ULTRA_RX_EMA_HOLD): a group that did not fully
    // deliver is right-censored at the current rung's calibrated floor before it
    // enters the obs ring. A fully-cratered group feeds NO fresh broadband SNR, so
    // burst_obs_snr_db_ is a STALE CREST read from an earlier good group; a partial
    // crater's fresh read is itself crest-biased (only the frames that DECODED
    // contributed). Either way, capping the sample at the rung anchor stops a failure
    // from pulling the ring average UP — the survivor bias that let the ring read
    // 26-32 dB on a 20 dB channel and re-clear the climb bar ~60 s after every demote.
    float obs_sample = burst_obs_snr_db_;
    if (emaHoldEnabled() && !all_ok) {
        const float a = calibrationAnchorDbFor(data_modulation_, data_code_rate_);
        if (a < kRungDisabledDb) obs_sample = std::min(obs_sample, a);
    }
    rx_auth_obs_db_[rx_auth_obs_next_] = obs_sample;
    rx_auth_fading_ring_[rx_auth_obs_next_] = inst_fading;
    rx_auth_obs_age_ms_[rx_auth_obs_next_] = 0;
    rx_auth_obs_next_ = (rx_auth_obs_next_ + 1) % kRxAuthObsRing;
    if (rx_auth_obs_count_ < kRxAuthObsRing) ++rx_auth_obs_count_;
    float snr_sum = 0.0f;
    float fading_sum = 0.0f;
    int snr_n = 0;
    // ULTRA_LINEAR_SNR_RING: average LINEAR POWER, not dB. The dial and the anchor table are
    // mean-power definitions, so a dB-domain mean reads systematically LOW on fading (Jensen)
    // — measured 1.65 dB on this ring's own samples. streaming_decoder.hpp::physicalSnrStats
    // already does it this way and documents the intent; this ring was the odd one out.
    // The knob also carries the compensating anchor-offset reduction (see
    // connection_policy.hpp::ofdmAnchorScaleOffsetDb) because fixing the domain ALONE raises
    // the ladder's input on a path already measured to over-commit ~2 rungs.
    const bool linear_ring = connection_policy::linearSnrRingEnabled();
    double snr_lin_sum = 0.0;
    for (size_t i = 0; i < rx_auth_obs_count_; ++i) {
        if (rx_auth_obs_age_ms_[i] <= kRxAuthObsMaxAgeMs) {
            snr_sum += rx_auth_obs_db_[i];
            snr_lin_sum += std::pow(10.0, static_cast<double>(rx_auth_obs_db_[i]) / 10.0);
            fading_sum += rx_auth_fading_ring_[i];
            ++snr_n;
        }
    }
    const float snr_avg =
        (snr_n > 0)
            ? (linear_ring ? static_cast<float>(
                                 10.0 * std::log10(snr_lin_sum / static_cast<double>(snr_n)))
                           : (snr_sum / static_cast<float>(snr_n)))
            : burst_obs_snr_db_;
    const float fading_avg = (snr_n > 0)
        ? (fading_sum / static_cast<float>(snr_n)) : inst_fading;
    // STICKY CLASS (F123): the anchor table's three fading columns are cliffs and
    // the class boundary is intrinsically fuzzy — a column switch must be an epoch
    // verdict, not a snapshot. Adopt a new class only when the SMOOTHED fading's
    // class persists 2 consecutive group verdicts; until then keep feeding the map
    // the last in-class fading value.
    const int raw_class = static_cast<int>(classifyFading(fading_avg));
    float eff_fading = fading_avg;
    if (raw_class == rx_auth_class_sticky_) {
        rx_auth_class_streak_ = 0;
        rx_auth_fading_passed_ = fading_avg;
    } else if (raw_class < rx_auth_class_sticky_ && rx_auth_class_streak_ < 4) {
        // ASYMMETRIC PERSISTENCE (F143): class IMPROVEMENTS (e.g. Good -> AWGN)
        // adopt only after ~4 consecutive verdicts (vs 2 for degradations) — a
        // calm dip in the fading average unlocked the AWGN-only 8PSK R3/4 rung
        // on a fading channel TWICE in one run, cratering on arrival each time.
        // Optimism is earned at half the speed of caution. While unconfirmed,
        // the map keeps seeing the last IN-CLASS fading value (passing the
        // improved reading would classify itself and bypass this hold).
        ++rx_auth_class_streak_;
        eff_fading = rx_auth_fading_passed_;
    } else if (raw_class > rx_auth_class_sticky_ && rx_auth_clean_streak_ >= 2) {
        // DECODE-EVIDENCE VETO (F125): the classifier says the channel got WORSE
        // while the last 2+ groups decoded clean — the decode record refutes it
        // (measured: fading walked 0.26->0.77 into MODERATE while 16QAM R2/3 ran
        // q=0.9-0.98 at 24-25 dB; the Moderate column has no dense rungs, so the
        // flip commanded a delivering link into the QPSK basement). Hold the
        // class; a genuine degradation craters within a couple of groups, the
        // clean streak dies, and the switch proceeds.
        rx_auth_class_streak_ = 0;
    } else if (++rx_auth_class_streak_ >= 2) {
        rx_auth_class_sticky_ = raw_class;
        rx_auth_class_streak_ = 0;
        rx_auth_fading_passed_ = fading_avg;
        LOG_MODEM(INFO, "Connection: RX-AUTHORITY fading class -> %d (fading=%.2f)",
                  raw_class, fading_avg);
    } else {
        eff_fading = rx_auth_fading_passed_;  // unconfirmed flap: hold the column
    }
    const CoherentPick mapped = selectCoherentOFDM(snr_avg, eff_fading);
    uint8_t cmd = coherentRungIndexFor(mapped.mod, mapped.rate);
    const uint8_t cur = coherentRungIndexFor(data_modulation_, data_code_rate_);

    // QSO-scoped clean-delivery evidence is shared by the legacy authority gates,
    // burst-ceiling prediction, and the latent selector.  Update it before the
    // selector's early return; otherwise the default latent path permanently sees
    // zero clean groups even while the transmitter has already earned escalation.
    if (all_ok) ++rx_auth_clean_streak_;
    else rx_auth_clean_streak_ = 0;

    // ── LATENT-STATE PATH (ULTRA_LATENT_RATE, DEFAULT-ON; =0 restores the legacy ladder)
    //
    // Consumes NO SNR ESTIMATE. It fits a latent operating point from OUTCOMES (frames
    // SACKed per group) against a link model measured directly as FER-vs-SNR
    // (docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md §1), so every group updates the
    // predicted success of ALL rungs at once and the evidence survives a rung change.
    //
    // THE POINT OF THE SHAPE: with P_r = f(x - theta_r) and x fitted from outcomes, adding a
    // constant to every theta_r is absorbed by x. COMMON-MODE CALIBRATION ERROR CANCELS
    // EXACTLY. That is what lets the rate path stop consuming
    // connection_policy::kOfdmLegacyAnchorScaleOffsetDb (+8.70 dB) — a compatibility shim
    // for an estimator bug fixed on 2026-07-07, which the code itself documents as leaving
    // the ladder ~5.6 dB optimistic on this hardware and over-committing by ~2 rungs. On
    // THIS path snr_avg is never read, so the offset cannot influence the decision.
    //
    // No dwell, no ratchet, no penalty array, no crater trigger: Minstrel's discipline, that
    // stability belongs in the estimator and not in the actuator.
    if (latentRateControllerEnabled()) {
        const auto now_tp = std::chrono::steady_clock::now();
        float group_s = 10.0f;
        if (goodput_last_verdict_valid_) {
            const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now_tp - goodput_last_verdict_).count();
            if (dt_ms > 1000 && dt_ms < 60000) group_s = static_cast<float>(dt_ms) / 1000.0f;
        }
        goodput_last_verdict_ = now_tp;
        goodput_last_verdict_valid_ = true;

        if (!latent_ctl_.havePrior()) {
            // Seed from the connect-time pick expressed as a LADDER POSITION, not as an SNR:
            // use the theta of the automatic pick BEFORE the conservative first-group
            // cap.  The cap is an acquisition-safety probe, not evidence that the link
            // lives at R1/2.  Forced modes replace this bootstrap at the handshake sites.
            // This keeps even the prior free of the calibration chain while avoiding an
            // artificial multi-group slow-start after a clean probe.
            const uint8_t seed_rung =
                latent_bootstrap_rung_ != kRungIdxNone
                    ? latent_bootstrap_rung_
                    : cur;
            latent_ctl_.seedPrior(latentThetaForRung(seed_rung), 3.0f);
            latent_bootstrap_rung_ = kRungIdxNone;
        }

        // Consume the descriptor's REAL group geometry. Production dynamically switches
        // between five- and eight-frame turns (and emits shorter file tails); reducing 6/8
        // to round(0.75*5)=4/5 changes both the posterior and the predicted cycle economics.
        // Anchored-no-descriptor backstops have no group_size and retain the conservative
        // five-frame fallback.
        const int M = group_size > 0 ? static_cast<int>(group_size) : 5;
        const int k = group_size > 0
                          ? std::clamp(static_cast<int>(delivered_frames), 0, M)
                          : ((delivered_fraction >= 0.0f)
                                 ? static_cast<int>(std::lround(delivered_fraction * M))
                                 : (all_ok ? M : 0));
        constexpr int kStartupMinTrustedGroupFrames = 4;
        const bool geometry_known = geometry_proven && group_size > 0;
        const bool trusted_group =
            geometry_known && group_size >= kStartupMinTrustedGroupFrames;
        const bool exact_clean_group = trusted_group && all_ok && k == M;
        const bool fail_unknown_probe = latent_startup_probe_waiting_ && !trusted_group;
        if (fail_unknown_probe) {
            // Do this BEFORE any posterior update or ACK emission.  A descriptorless
            // callback may carry a plausible stale k/M; it is neither success nor failure
            // evidence for the one-shot probe and must not leave the standing UP command.
            failClosedLatentStartupProbeUnknown("group geometry not wire-proven");
        }
        // Never fabricate a denominator from stale fallback geometry.  Proven short
        // physical tails remain valid general posterior evidence, but cannot qualify
        // the startup probe's two full-group gate.
        if (geometry_known) {
            latent_ctl_.observe(cur, k, M);
        }
        // Forget at the rate the ionosphere actually moves. THE ONE FREE PARAMETER; see the
        // header. Too small freezes the controller and looks perfect on OTASim (stationary
        // Watterson, no ITV) while failing on the rig.
        static const float kRelaxDb = [] {
            if (const char* e = std::getenv("ULTRA_LATENT_RELAX_DB")) {
                const float v = std::strtof(e, nullptr);
                if (v >= 0.0f && v <= 5.0f) return v;
            }
            return 0.35f;
        }();
        if (geometry_known) {
            latent_ctl_.relax(kRelaxDb);
        }

        // Code-faithful bulk-transfer geometry. The old objective both compressed airtime
        // by 1/eta AND multiplied value by eta, rewarding dense rungs twice. It also ignored
        // modulation-aware CW normalisation (QPSK=8, 8PSK=12, 16QAM=16), which makes the
        // active rungs nearly equal-duration frames. Build useful file bytes and burst
        // airtime from the same production helpers used by TX, then add the rig-measured
        // median non-data turnaround.  IMPORTANT: M above belongs only to the OBSERVATION.
        // Every candidate gets its own CW/window/duty-ceiling-derived frame count; reusing
        // observed M here overvalues whichever rungs happen to fit fewer frames per turn.
        // The full 1.2 s anchor is already in burst airtime.
        LatentRateController::RungGeometryTable geometry{};
        std::array<int, kRungIdxCount> candidate_logical_cw{};
        std::array<int, kRungIdxCount> candidate_physical_cw{};
        std::array<int, kRungIdxCount> candidate_z{};
        std::array<bool, kRungIdxCount> candidate_full_anchor{};
        for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
            const CoherentPick candidate = coherentRungFromIndex(r);
            if (!coherentRungLocallyEnabled(candidate.mod, candidate.rate)) continue;
            // The receiver commands only a rung.  Its local fading estimate is not
            // transmitted and therefore cannot determine the sender's next CW shape;
            // the following descriptor will announce the sender's actual choice.  Price
            // the logical CW from the peer-independent baseline and resolve any explicitly
            // configured long-file representation through the same CW/Z contract as TX.
            // Every descriptor-committed rung move arms one extra full group-start
            // anchor.  It consumes both airtime and enough of the first cycle's PA
            // ceiling to reduce N on several profiles, so a non-incumbent must be
            // priced as the first burst the sender will actually emit.
            const bool force_full_group_start = r != cur;
            const auto physical = latentRateCandidateGeometryFor(
                candidate.mod, candidate.rate, force_full_group_start);
            candidate_logical_cw[r] = physical.logical_cw;
            candidate_physical_cw[r] = physical.physical_cw;
            candidate_z[r] = physical.lifting_z;
            candidate_full_anchor[r] = physical.force_full_group_start;
            geometry[r] = physical.value;
        }
        const uint8_t latent_ceiling = latentConfiguredRungCeiling();
        const auto best = latent_ctl_.best(
            geometry, latent_ceiling, latentTieBreakProbeEnabled());
        uint8_t lat_cmd = (best.rung != kRungIdxNone) ? best.rung : cur;
        if (lat_cmd != cur) {
            const uint8_t snapped = snapRungIndexDownToEnabled(lat_cmd);
            lat_cmd = (snapped != kRungIdxNone) ? snapped : cur;
        }
        if (latent_startup_probe_failed_ && lat_cmd > kRungIdxQpskR12) {
            // A failed/unknown one-shot establishes a QSO-scoped safety ceiling.  The
            // posterior may recover after clean base groups, but it may not silently
            // recreate the exploration that was promised to happen only once.
            lat_cmd = kRungIdxQpskR12;
        }

        // BOUNDED STARTUP EXPLORATION.  R1/2 successes saturate the likelihood, so the
        // estimator cannot learn whether R2/3 works by observing R1/2 forever (fixed04:
        // clean physical N=5 groups until the natural climb at observation 16).  Spend at
        // most ONE higher-rung group per unforced QSO, and only after two consecutive
        // descriptor/tail-proven clean groups.  This is deliberately separate from the
        // periodic tie-probe experiment, which remains default-off after doubling rig-run
        // variance.  Unknown geometry is not positive evidence and a lossy probe rolls
        // back in the ACK for that very group.
        constexpr int kStartupCleanGroupsRequired = 2;
        constexpr int kStartupArmObservationLimit = 4;
        constexpr int kStartupMaxPendingBaseGroups = 2;
        const uint8_t startup_base = kRungIdxQpskR12;
        const uint8_t startup_target = kRungIdxQpskR23;
        const bool target_enabled =
            snapRungIndexDownToEnabled(startup_target) == startup_target &&
            (latent_ceiling == kRungIdxNone || startup_target <= latent_ceiling);
        const float target_p_at_decision = LatentRateController::successProb(
            best.x_used, latentThetaForRung(startup_target));
        const char* startup_note = "";

        const bool automatic_rate_allowed =
            latent_startup_probe_allowed_ && rateAdaptationActive() &&
            latentStartupProbeAllowedByOperator();
        if (!automatic_rate_allowed) {
            // Config-forced CONNECT profiles and live force/lock knobs freeze the WHOLE
            // latent actuator.  Merely suppressing the special probe still let the normal
            // argmax escape an operator's fixed-rung measurement on a later verdict.
            lat_cmd = cur;
            latent_startup_probe_spent_ = true;
            latent_startup_probe_waiting_ = false;
            latent_startup_probe_clean_groups_ = 0;
            latent_startup_probe_pending_base_groups_ = 0;
            latent_startup_probe_rollback_pending_ = false;
            latent_startup_probe_failed_ = false;
            startup_note = " operator-rate-pin";
        } else if (latent_startup_probe_rollback_pending_) {
            // Re-carry the rollback until the sender's descriptor proves it adopted a
            // base-or-lower rung, then keep that rung for one trusted clean group.  This
            // prevents the posterior update from the failed probe undoing the rollback on
            // the very next ACK.
            const uint8_t rollback =
                (cur != kRungIdxNone && cur < startup_base) ? cur : startup_base;
            lat_cmd = rollback;
            if (cur <= startup_base && exact_clean_group) {
                latent_startup_probe_rollback_pending_ = false;
                startup_note = " startup-probe-rollback-settled";
            } else {
                startup_note = " startup-probe-rollback-pending";
            }
        } else if (!geometry_known) {
            // No fresh selector evidence: hold the physical rung.  The rollback branch
            // above deliberately has priority so an unknown result cannot erase a safety
            // landing that is already in progress.
            lat_cmd = cur;
            latent_startup_probe_clean_groups_ = 0;
            startup_note = " geometry-unproven-hold";
        } else if (latent_startup_probe_waiting_) {
            if (cur == startup_target) {
                // This is the one physical probe group.  A clean result is direct
                // evidence that R2/3 works, so retain it for the next group while the
                // posterior catches up.  Anything else reverts immediately to R1/2.
                if (exact_clean_group) {
                    latent_startup_probe_waiting_ = false;
                    latent_startup_probe_spent_ = true;
                    latent_startup_probe_pending_base_groups_ = 0;
                    lat_cmd = startup_target;
                    startup_note = " startup-probe-pass";
                } else {
                    failClosedLatentStartupProbeUnknown(
                        trusted_group ? "probe group imperfect" : "probe result unknown");
                    lat_cmd = startup_base;
                    startup_note = " startup-probe-rollback";
                }
            } else if (cur == startup_base && exact_clean_group) {
                // The absolute command may need ACK diversity before the sender adopts it.
                // Clean intervening base-rung groups are not extra probes: keep requesting
                // the same target until its descriptor arrives.
                ++latent_startup_probe_pending_base_groups_;
                if (latent_startup_probe_pending_base_groups_ <=
                        kStartupMaxPendingBaseGroups) {
                    lat_cmd = startup_target;
                    startup_note = " startup-probe-pending";
                } else {
                    failClosedLatentStartupProbeUnknown("target descriptor not adopted");
                    lat_cmd = startup_base;
                    startup_note = " startup-probe-expired";
                }
            } else {
                // Adoption did not occur and the intervening evidence is imperfect or has
                // no trusted geometry.  Cancel without ever risking the higher rung.
                failClosedLatentStartupProbeUnknown(
                    trusted_group ? "unexpected rung before adoption"
                                  : "pre-adoption result unknown");
                lat_cmd = (cur != kRungIdxNone && cur < startup_base)
                    ? cur
                    : startup_base;
                startup_note = " startup-probe-rollback";
            }
        } else if (!latent_startup_probe_spent_) {
            if (cur != startup_base ||
                latent_ctl_.observations() > kStartupArmObservationLimit) {
                // This is a startup-only escape from R1/2 identifiability, not a general
                // periodic probe.  Once the normal selector moves, or four verdicts pass,
                // it is permanently out of the decision path for this QSO.
                latent_startup_probe_spent_ = true;
                latent_startup_probe_clean_groups_ = 0;
            } else {
                latent_startup_probe_clean_groups_ = exact_clean_group
                    ? latent_startup_probe_clean_groups_ + 1
                    : 0;
                if (latent_startup_probe_clean_groups_ >=
                        kStartupCleanGroupsRequired) {
                    // Never override a natural upward argmax, and never probe a target the
                    // model itself declares dead.  The clean-pair is the exploration
                    // evidence; these gates prevent blind forcing.
                    if (lat_cmd != startup_base) {
                        latent_startup_probe_spent_ = true;
                    } else if (target_enabled &&
                               target_p_at_decision >=
                                   LatentRateController::kDeadProb) {
                        latent_startup_probe_waiting_ = true;
                        latent_startup_probe_pending_base_groups_ = 0;
                        lat_cmd = startup_target;
                        startup_note = " startup-probe-start";
                    } else {
                        latent_startup_probe_spent_ = true;
                    }
                }
            }
        }
        rx_authority_cmd_ = lat_cmd;
        LOG_MODEM(INFO,
                  "Connection: LATENT-RATE idx %u -> %u  x_p25=%.1f mean=%.1f sd=%.1f "
                  "obs=%d  k=%d/%d  group=%.1fs  candidate_LCW=%d "
                  "candidate_CW=%d candidate_Z=%d candidate_N=%d candidate_FULL=%d  "
                  "predicted=%.0fB/s%s%s  "
                  "(NO SNR CONSUMED)",
                  cur, lat_cmd, best.x_used, latent_ctl_.posteriorMean(),
                  latent_ctl_.spreadDb(), latent_ctl_.observations(), k, M, group_s,
                  candidate_logical_cw[lat_cmd],
                  candidate_physical_cw[lat_cmd], candidate_z[lat_cmd],
                  geometry[lat_cmd].frames_per_cycle,
                  candidate_full_anchor[lat_cmd] ? 1 : 0,
                  best.goodput, best.tie_break_probe ? " probe" : "", startup_note);
        return;
    }

    // ── GOODPUT-MAXIMIZING PATH (ULTRA_GOODPUT_RATE, default OFF) ────────────────
    // REPLACES everything below, deliberately. The SNR-anchor pick plus its ~10
    // correctives measured 10.6% WORSE than pinning 8PSK R2/3 on the rig (2026-07-30,
    // both pairs of an interleaved MPG@20 A/B; the pinned arm never demoted even on the
    // rough epoch). Layering a goodput rule ON TOP of that stack would keep the churn it
    // exists to remove, so this returns early rather than composing.
    //
    // It reads NO SNR. That is the point: the anchor/scale chain has been the largest
    // single source of wrong rate decisions here, and a controller that never consumes
    // an SNR cannot be wrong about one. See goodput_rate_controller.hpp for the
    // derivation and the hold guarantee.
    if (goodputRateControllerEnabled()) {
        // Window sized from the channel, not chosen: Tc = 0.423/f_d with f_d derived
        // from the SAME fading index the rest of the controller uses, and the group
        // cadence MEASURED from the interval between verdicts (this function runs once
        // per burst group). First verdict has no interval yet and falls back to 10 s.
        const auto now_tp = std::chrono::steady_clock::now();
        float group_s = 10.0f;
        if (goodput_last_verdict_valid_) {
            const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now_tp - goodput_last_verdict_)
                                   .count();
            if (dt_ms > 1000 && dt_ms < 60000) group_s = static_cast<float>(dt_ms) / 1000.0f;
        }
        goodput_last_verdict_ = now_tp;
        goodput_last_verdict_valid_ = true;
        const float doppler = connection_policy::designDopplerForFadingIndex(eff_fading);
        goodput_ctl_.configure(windowGroupsForCoherence(doppler, group_s), group_s);
        // The ladder's own pick is a CEILING, not a command: it bounds which rungs are
        // plausible at this SNR/class, while the measured goodput chooses within that.
        // Without it the climb is an unbounded staircase that walks up until a rung
        // breaks (rig: 2->3->4->5->8 into a 51.4%-FER rung in four windows).
        goodput_ctl_.setCeiling(coherentRungIndexFor(mapped.mod, mapped.rate));

        // delivered_fraction < 0 means the frame count was not measurable for this
        // group; fall back to the binary outcome rather than inventing a number.
        const float f = (delivered_fraction >= 0.0f)
                            ? delivered_fraction
                            : (all_ok ? 1.0f : 0.0f);
        goodput_ctl_.observe(cur, f);
        const auto decision = goodput_ctl_.decide(cur);
        uint8_t gp_cmd = decision.rung;
        if (gp_cmd != kRungIdxNone && gp_cmd != cur) {
            // Same enabled-ladder discipline as the legacy path (F145): a command
            // naming a disabled rung is unobeyable and pins a cratering rung forever.
            const uint8_t snapped = snapRungIndexDownToEnabled(gp_cmd);
            gp_cmd = (snapped != kRungIdxNone) ? snapped : cur;
        }
        rx_authority_cmd_ = gp_cmd;
        if (gp_cmd != cur) {
            const CoherentPick commanded = coherentRungFromIndex(gp_cmd);
            LOG_MODEM(INFO,
                      "Connection: GOODPUT-RATE verdict %s %s (idx %u -> %u) reason=%s "
                      "f_window=%.3f break_even=%.3f window=%dg/%.1fs doppler=%.2f q=%.2f",
                      modulationToString(commanded.mod), codeRateToString(commanded.rate),
                      cur, gp_cmd, decision.reason, goodput_ctl_.windowedDeliveredFraction(),
                      goodputBreakEvenDeliveredFraction(cur), goodput_ctl_.windowGroups(),
                      group_s, doppler, quality);
        } else {
            LOG_MODEM(INFO,
                      "Connection: GOODPUT-RATE hold idx %u reason=%s f_window=%.3f "
                      "break_even=%.3f n=%d/%d q=%.2f",
                      cur, decision.reason, goodput_ctl_.windowedDeliveredFraction(),
                      goodputBreakEvenDeliveredFraction(cur), goodput_ctl_.observations(),
                      goodput_ctl_.windowGroups(), quality);
        }
        return;
    }
    // TWO-CRATER rule + CRATER-MARGIN memory (F122: 10 moves/283 s — a single
    // crater at a ~10 s decision quantum vs Tc 2-4 s is an irreducible deep null
    // the ARQ absorbs, NOT rate evidence; chasing singles paid the full-anchor +
    // requeue-rewind tax every ~28 s). Only CONSECUTIVE craters demote and charge
    // the rung's margin memory (+2 dB, cap 6; posterior beats prior). Every clean
    // group decays all penalties (0.25 dB) — the fade epoch ends, evidence expires.
    bool crater = (quality <= 0.0f && !all_ok);
    // GOODPUT-GRADED CRATER (ULTRA_CRATER_GOODPUT_GRADE): the binary quality byte
    // cannot tell 7/8 from 0/8. Re-grade against the geometry-derived break-even —
    // a lossy group is rung-failure evidence only when this rung's delivered
    // fraction no longer beats a PERFECT run at the rung below. A true 0/N crater
    // is always evidence (fraction 0 < any positive break-even), so the cliff case
    // is unchanged; only the "ARQ is absorbing it cheaply" middle is reclassified.
    // Requires a real measurement: with no frame count (delivered_fraction < 0) the
    // predicate falls through to the legacy binary form untouched.
    if (crater && craterGoodputGradeEnabled() && delivered_fraction >= 0.0f) {
        const float break_even = goodputBreakEvenDeliveredFraction(cur);
        if (break_even > 0.0f && delivered_fraction >= break_even) {
            LOG_MODEM(INFO,
                      "Connection: RX-AUTHORITY crater REGRADED to hold (idx %u "
                      "delivered=%.2f >= break-even=%.2f vs rung below) — ARQ "
                      "absorbs this cheaper than a demote",
                      cur, delivered_fraction, break_even);
            crater = false;
        }
    }
    if (crater) ++rx_auth_crater_streak_;
    else if (all_ok) rx_auth_crater_streak_ = 0;
    // DENSE-MOD FAST DEMOTE (ULTRA_DENSE_FAST_DEMOTE): the two-crater grace is calibrated
    // for robust rungs (a single QPSK crater is an ARQ-absorbed null). A dense-mod rung's
    // tight rings make a FULL crater (0/N) genuine over-commit, and the stall cost is
    // asymmetric — so leave after ONE full crater instead of two. Robust mods and partial
    // craters keep the grace (partial 16QAM resends cheaply via per-frame SACK).
    const bool dense_mod = getBitsPerSymbol(data_modulation_) >= 4;
    const int crater_threshold =
        (denseFastDemoteEnabled() && dense_mod && full_crater) ? 1 : 2;
    const bool crater_confirmed = crater && rx_auth_crater_streak_ >= crater_threshold;
    // EMA-SUPPORTED HOLD (ULTRA_RX_EMA_HOLD, lever #1): a confirmed crater is only
    // rung-failure EVIDENCE if the fade-averaged SNR no longer clears the current
    // rung's floor ON the current class. While snr_avg strictly exceeds that class
    // anchor, consecutive craters are deep-null fade brushes the ARQ resends through
    // (F344: 5/5 craters were partial, 24/31 frames decoded, avg 18-26 dB) — demoting
    // there is the limit cycle that stranded F372 at QPSK (1.16 vs 2.61 kbps, same
    // channel). The censored ring feed above keeps snr_avg honest, so sustained real
    // failure (samples driven toward the physics floor) crosses below and DOES demote.
    // Strict '>' avoids a latch when censor floor == class anchor (AWGN class).
    const float cur_class_anchor =
        rungClassAnchorDb(data_modulation_, data_code_rate_, eff_fading);
    const bool ema_supports_cur =
        emaHoldEnabled() && cur_class_anchor < kRungDisabledDb && snr_avg > cur_class_anchor;
    const bool demote_on_crater = crater_confirmed && !ema_supports_cur;
    if (cur != kRungIdxNone && cur < kRungIdxCount) {
        if (demote_on_crater) {
            // RATCHET (F126): the penalty DOUBLES per confirmed failure episode of
            // the rung (2 -> 4 -> 8, cap 8) and decays 40x slower than before —
            // "this rung does not work TODAY" must survive minutes, not one clean
            // stretch. F125/F126 oscillated 16QAM<->QPSK every ~10 groups: +2dB
            // penalties decayed within ~8 clean groups, the 24-25 dB average
            // re-cleared anchor+margin, and every re-climb bought two dead groups
            // and two anchors. Episode 2 now demands avg >= anchor+2.5+4 (~26.5),
            // episode 3 pins the rung out until the channel genuinely improves.
            // F160: first confirmed episode prices at 4 dB (was 2) — with
            // one-rung climbs a toxic rung's second visit already costs a full
            // ladder walk, so the first episode must push the re-try bar to
            // anchor+6.5 (locks the rung out for the transfer at the ~24 dB
            // ring averages this bench produces; a genuinely improved channel
            // still clears it).
            const float p = rx_auth_rung_penalty_db_[cur];
            rx_auth_rung_penalty_db_[cur] =
                std::min(8.0f, (p < 4.0f) ? 4.0f : p * 2.0f);
            // F149 CLIMB DWELL, F160-rebalanced: a confirmed episode arms a
            // dwell — no up-command until post-episode reality displaces the
            // survivor-biased crest reads (failed groups contribute NO SNR
            // observation; the ring read 26-32 dB on a 20 dB channel and
            // re-cleared any fixed haircut ~60 s after every demote). THREE
            // clean groups, not a full ring turnover: F160 parked 61 s at
            // QPSK R1/4 on a 22 dB channel waiting out dwell=6 — the dwell
            // gates RE-BETTING, it must not tax the recovery.
            rx_auth_climb_dwell_ = kRxAuthClimbDwellGroups;
        } else if (all_ok) {
            for (size_t i = 0; i < kRungIdxCount; ++i) {
                rx_auth_rung_penalty_db_[i] =
                    std::max(0.0f, rx_auth_rung_penalty_db_[i] - 0.05f);
            }
            if (rx_auth_climb_dwell_ > 0) --rx_auth_climb_dwell_;
        }
    }
    if (cur != kRungIdxNone) {
        if (cmd > cur && rx_auth_climb_dwell_ > 0) {
            // F149: post-episode dwell — the ring is still crest-biased from
            // the last confirmed crater; hold until it turns over.
            cmd = cur;
        }
        // RX-AUTHORITY PREDICTIVE CLIMB — RETIRED 2026-08-01.
        //
        // It gated every up-command on an EESM prediction over a ring of per-carrier gamma
        // snapshots, and it was the ONLY reader of the gamma vector that
        // streaming_burst_interleave.cpp marked up with kOfdmLegacyAnchorScaleOffsetDb.
        // Removing it removes that offset consumer.
        //
        // Why it went rather than being fixed (25-agent corrective-stack audit, 2026-07-31):
        //  - It answers the WRONG QUESTION. rungPredictedSustainable asks "will this rung
        //    decode", not "which rung delivers more bps", so its bar is LOWER for lower-eta
        //    rungs. It handed back QPSK R3/4 (2066 bps @20) in place of 8PSK R2/3 (2450 incl.
        //    craters), undoing the 2026-07-26 anchor re-measure whose entire point was to put
        //    8PSK R2/3's Good anchor at the throughput crossover.
        //  - Measured asymmetry: when it disagreed with the map it downgraded 51x and
        //    upgraded 6x.
        //  - Its hold (cmd = cur) is INVISIBLE in the clamp statistics, because the verdict
        //    log only prints when cmd != cur — so the measured "46% of decisions clamped"
        //    UNDERSTATES the real rate.
        //  - The conjunction "all fresh snapshots must pass" over a 4-deep ring with a 180 s
        //    max age means one weak snapshot blocks every climb until four newer ones
        //    displace it.
        //
        // The latent-state controller supersedes it properly: it predicts EVERY rung from a
        // posterior fitted to outcomes, which is the job this was attempting with a single
        // pass/fail test on a stale ring.
        if (cmd > cur) {
            // CLIMB HYSTERESIS + crater margin: an up-command must survive a
            // haircut — the base 2.5 dB guards against slow swells; the target
            // rung's crater penalty demands the channel PROVE headroom the anchors
            // only assumed. Descents take the map directly (down is safe).
            constexpr float kClimbMarginDb = 2.5f;
            const float haircut = kClimbMarginDb +
                ((cmd < kRungIdxCount) ? rx_auth_rung_penalty_db_[cmd] : 0.0f);
            const CoherentPick guarded =
                selectCoherentOFDM(snr_avg - haircut, eff_fading);
            const uint8_t guarded_idx =
                coherentRungIndexFor(guarded.mod, guarded.rate);
            if (guarded_idx <= cur) cmd = cur;  // not a margin-proof climb: hold
        }
        if (cmd > cur) {
            // F149 ONE-RUNG CLIMB (FALLBACK path — the predictive climb above
            // replaces it whenever measurements exist): a climb is a bet on
            // headroom no delivered group has proven — bet the minimum stake.
            // Walk UP to the first enabled rung above cur (holes skipped
            // upward), never past the map's own pick.
            uint8_t step = kRungIdxNone;
            for (uint8_t i = static_cast<uint8_t>(cur + 1);
                 i <= cmd && i < kRungIdxCount; ++i) {
                const CoherentPick p = coherentRungFromIndex(i);
                if (coherentRungLocallyEnabled(p.mod, p.rate)) {
                    step = i;
                    break;
                }
            }
            cmd = (step != kRungIdxNone) ? step : cur;
        }
        if (demote_on_crater) {
            // Confirmed-crater override: two in a row is the rung failing, not a
            // null — command the FIRST ENABLED rung below it. F160 rebalance: the
            // old cur-2 stride, snapped down THROUGH the QAM8 R3/4 hole, turned
            // every 16QAM R2/3 episode into a 3-rung collapse (8 -> 5) that the
            // one-rung climb then repaid across 3 switches — fast-down/slow-up
            // parked the run below the sustainable rung (F160: 0.89 kbps, 17
            // switches). One enabled rung down lands exactly on the rung the
            // ladder just PROVED on the way up (16QAM R1/2 here); the crater
            // penalty + dwell — not collapse depth — do the recidivism work.
            // (F129's ~70 s grind predates both; honest sustained failure still
            // descends verdict by verdict.)
            uint8_t below = kRungIdxQpskR14;
            if (cur > kRungIdxQpskR14) {
                const uint8_t b =
                    snapRungIndexDownToEnabled(static_cast<uint8_t>(cur - 1));
                if (b != kRungIdxNone) below = b;
            }
            if (cmd >= cur) cmd = below;
        } else if (crater && cmd > cur) {
            // Single crater — OR a confirmed crater the EMA still supports
            // (ULTRA_RX_EMA_HOLD): hold the rung (the ARQ resends through the null),
            // never climb ON a crater. The EMA-supported confirmed crater lands here
            // because demote_on_crater is false, so the demote branch is skipped.
            cmd = cur;
        } else if (crater && ema_supports_cur && cmd < cur) {
            // EMA-supported confirmed crater where the map independently wants down
            // (fading-class flap): the fade-averaged SNR still clears the rung floor,
            // so hold rather than let the map drop us — the whole point of the hold.
            cmd = cur;
        } else if (all_ok && cmd < cur) {
            // Clean-group override: this rung just WORKED end to end.
            cmd = cur;
        }
        if (cmd < cur) {
            // DOWN RATE-LIMIT (F123, made UNCONDITIONAL after F125): any single
            // verdict steps at most 2 canonical rungs down — confirmed craters
            // included (F125: a confirmed pair let the class-cliff map pick
            // through unclamped, 16QAM R2/3 -> QPSK R2/3 in ONE command on a
            // 23 dB channel). The confirmed-crater clamp above already guarantees
            // at least one step below the failed rung; repeated honest verdicts
            // still reach any depth, bounded step by bounded step.
            const uint8_t floor_step = static_cast<uint8_t>(cur > 2 ? cur - 2 : 1);
            if (cmd < floor_step) cmd = floor_step;
        }
        if (cmd != cur && cmd != kRungIdxNone) {
            // ENABLED-LADDER ARITHMETIC (F145 deadlock): the stride/limit math
            // above is raw index arithmetic and can land on an anchor-table hole
            // (QAM8 R3/4 disabled => idx 6). A command naming a disabled rung is
            // unobeyable — the sender holds and a cratering rung pins forever.
            // Snap DOWN to the nearest enabled rung; the walk meets cur (enabled,
            // we are running it) before ever inverting a climb into a descent.
            const uint8_t snapped = snapRungIndexDownToEnabled(cmd);
            cmd = (snapped != kRungIdxNone) ? snapped : cur;
        }
        // ── RADIO-AGNOSTIC EVM DEMOTE (ULTRA_EVM_DEMOTE, Stage 2) ── the DEFAULT
        // latent-rate path returned above and never reaches this block; this clamp
        // belongs to the legacy selector selected by ULTRA_LATENT_RATE=0. The broadband
        // estimator that drove cmd is dial/channel-scaled (+kOfdmLegacyAnchorScaleOffsetDb)
        // and on a real radio over-reads the usable channel by the hardware loss. The
        // decision-directed EVM reads USABLE dB directly and cannot inflate. If this
        // group's usable EVM cannot support the CURRENT rung's EVM-usable floor, strip
        // the over-commit: clamp DOWN to the highest rung the EVM can carry (never up —
        // EVM saturates at the demod ceiling), then re-snap to the enabled ladder. A
        // pure output clamp, orthogonal to the crater/penalty machinery.
        if (evmDemoteEnabled() && burst_obs_evm_snr_db_ >= 0.0f) {
            // Feed the ring first so the confident form always sees THIS group.
            rx_auth_evm_ring_[rx_auth_evm_next_] = burst_obs_evm_snr_db_;
            rx_auth_evm_next_ = (rx_auth_evm_next_ + 1) % kRxAuthObsRing;
            if (rx_auth_evm_count_ < kRxAuthObsRing) ++rx_auth_evm_count_;

            // CONFIDENCE-GATED EVM DEMOTE (ULTRA_EVM_DEMOTE_CONFIDENT, default OFF).
            //
            // The raw clamp compares ONE per-group EVM sample against a fixed rung floor.
            // Measured on the rig 2026-07-26: that sample has sd 3.1-4.4 dB, while adjacent
            // rung EVM floors are only 1.2-3.1 dB apart. The noise EXCEEDS the rung
            // spacing, so a single sample cannot resolve which rung is supported — and a
            // noisy fast signal hard-clamping the output of the ladder's SMOOTHED snr_avg
            // (ring of 6) saws: rung commands 15 vs 7 per run, EVM firings scattering
            // across idx 5->4, 3->2, 7->5, 7->3, 3->1 in ONE transfer. This is an
            // SNR-RESOLUTION error, not an estimator defect (the estimator is calibrated).
            //
            // Fix: demote only when the rung is UNAMBIGUOUSLY unsupported — when even the
            // OPTIMISTIC end of the confidence interval on usable SNR still misses the
            // floor. With mean m and standard error se = s/sqrt(N):
            //     demote iff  m + z*se  <  floor - margin
            // z = 1.645 (one-sided 95%) is a PROBABILITY constant, not a bench constant, so
            // the gate stays radio-agnostic. It also self-sharpens: se shrinks as evidence
            // accumulates, so a genuinely unsupported rung is still caught, just later and
            // correctly. The TARGET stays the point estimate (mean), never the pessimistic
            // bound, so a confirmed demote cannot overshoot downward on noise.
            // Deliberately NOT a function-local static: a latched knob cannot be A/B'd
            // within one process, which made the confidence gate untestable (the first
            // arm's value stuck for every later arm). This runs once per burst group
            // (~10 s), so a getenv here is free.
            const char* kEvmConfidentEnv = std::getenv("ULTRA_EVM_DEMOTE_CONFIDENT");
            const bool kEvmConfident =
                kEvmConfidentEnv && kEvmConfidentEnv[0] == '1' && kEvmConfidentEnv[1] == '\0';
            float evm_for_clamp = burst_obs_evm_snr_db_;
            float evm_upper = burst_obs_evm_snr_db_;  // raw form: the sample IS the bound
            if (kEvmConfident) {
                const size_t n = rx_auth_evm_count_;
                float sum = 0.0f;
                for (size_t i = 0; i < n; ++i) sum += rx_auth_evm_ring_[i];
                const float mean = sum / static_cast<float>(n);
                float var = 0.0f;
                for (size_t i = 0; i < n; ++i) {
                    const float d = rx_auth_evm_ring_[i] - mean;
                    var += d * d;
                }
                // Sample sd (n-1); with n<2 there is no spread estimate yet, so fall back
                // to the measured per-group sd so a single sample can never fire the clamp.
                constexpr float kMeasuredPerGroupSdDb = 3.1f;
                const float sd = (n >= 2) ? std::sqrt(var / static_cast<float>(n - 1))
                                          : kMeasuredPerGroupSdDb;
                constexpr float kOneSidedZ95 = 1.645f;
                const float se = sd / std::sqrt(static_cast<float>(n));
                evm_for_clamp = mean;
                evm_upper = mean + kOneSidedZ95 * se;
            }
            const float cur_floor = evmUsableFloorDbForRung(cur);
            if (cur_floor < kRungDisabledDb &&
                evm_upper < cur_floor - kEvmDemoteHoldMarginDb) {
                uint8_t evm_cap =
                    highestRungSupportedByEvm(evm_for_clamp, kEvmDemoteHoldMarginDb);
                const uint8_t snapped_evm = snapRungIndexDownToEnabled(evm_cap);
                evm_cap = (snapped_evm != kRungIdxNone) ? snapped_evm : kRungIdxQpskR14;
                if (evm_cap < cmd) {
                    LOG_MODEM(INFO,
                              "Connection: RX-AUTHORITY EVM-DEMOTE idx %u -> %u "
                              "(usable EVM=%.1f upper=%.1f n=%zu < rung floor=%.1f-%.1f margin)",
                              cmd, evm_cap, evm_for_clamp, evm_upper, rx_auth_evm_count_,
                              cur_floor, kEvmDemoteHoldMarginDb);
                    cmd = evm_cap;
                }
            }
        }
    }
    // ── TRUST THE LADDER'S PICK (ULTRA_TRUST_LADDER_PICK, default OFF) ──────────
    //
    // MEASURED 2026-07-31, IONOS rig, 135 RX-AUTHORITY verdicts:
    //
    //     ladder picked (raw)          actually commanded
    //       8PSK R2/3   66%              QPSK R2/3   45%
    //       QPSK R2/3   28%              QPSK R3/4   24%
    //       16QAM R2/3   3%              8PSK R2/3   26%
    //
    //     46% of verdicts clamped away from the pick, mean drop 1.61 rungs (max 4),
    //     with fading correctly classified Good in 72% and snr_avg typically 21-24 dB.
    //
    // THE SELECTOR IS NOT THE DEFECT. selectCoherentOFDM picks 8PSK R2/3 two-thirds of the
    // time — the correct answer, since FADING_ANCHOR_MEASUREMENT_2026_07_26 measures it as
    // the best rung anywhere on ITU Good (2450 bps @20 INCLUDING its crater rate). The
    // corrective stack applied AFTER the pick then drags the command down 1.61 rungs.
    //
    // This also explains two failed experiments this session. Forcing that rung won (+26%,
    // 2.55 kbps record) not because "holding is good" but because forcing BYPASSES the
    // stack. And a minimum-dwell knob measured -13.8%, because the changes are not the
    // ladder re-deciding — they are clamps firing — so throttling change frequency merely
    // froze whatever a clamp had already dragged us down to.
    //
    // ON: command the ladder's pick, retaining only what is structurally required:
    //   - the ENABLED-LADDER SNAP. A command naming a disabled rung is UNOBEYABLE: the
    //     sender holds and a cratering rung pins forever (F145, a real 50 s deadlock).
    //     Disabled anchor rows are HOLES and raw index arithmetic walks into them.
    //   - a CATASTROPHIC-CRATER ESCAPE. Repeated 0/N groups mean the rung delivers literally
    //     nothing; that is a stall, not a rung to hold, and unlike a partial crater it is
    //     unambiguous. Everything else — the two-crater rule, ratcheting penalties, climb
    //     dwell, EMA hold, the one-rung walk, the 2-rung down limit — is a throughput
    //     heuristic and is skipped.
    //
    // Default OFF: each corrective was added after a real incident (F122/F125/F126/F149/
    // F160 oscillation and stall episodes), and the honest position is that this trades a
    // known-measured 46% clamping rate against those recurrence risks. Rig A/B before any
    // default flip; the regression signature to watch for is rung oscillation between
    // adjacent rungs every few groups with craters following each up-switch.
    if (cur != kRungIdxNone && cur < kRungIdxCount) {
        static const bool kTrustLadderPick = [] {
            const char* e = std::getenv("ULTRA_TRUST_LADDER_PICK");
            return e != nullptr && e[0] == '1' && e[1] == '\0';
        }();
        if (kTrustLadderPick) {
            const bool catastrophic =
                full_crater && rx_auth_crater_streak_ >= kCatastrophicDwellBypassCraters;
            const uint8_t raw_pick = coherentRungIndexFor(mapped.mod, mapped.rate);
            if (!catastrophic && raw_pick != kRungIdxNone && raw_pick < kRungIdxCount) {
                const uint8_t snapped = snapRungIndexDownToEnabled(raw_pick);
                if (snapped != kRungIdxNone && snapped != cmd) {
                    LOG_MODEM(INFO,
                              "Connection: TRUST-LADDER-PICK overriding clamps: cmd %u -> %u "
                              "(ladder raw=%u, snapped=%u) snr_avg=%.1f fading=%.2f q=%.2f",
                              cmd, snapped, raw_pick, snapped, snr_avg, eff_fading, quality);
                    cmd = snapped;
                }
            } else if (catastrophic) {
                LOG_MODEM(WARN,
                          "Connection: TRUST-LADDER-PICK yielding to %d consecutive TOTAL "
                          "craters (keeping clamped cmd %u, ladder wanted %u)",
                          rx_auth_crater_streak_, cmd, raw_pick);
            }
        }
    }

    // ── MINIMUM RUNG DWELL (ULTRA_RUNG_DWELL_MS, default 0 = off) ────────────────
    //
    // MEASURED 2026-07-30/31, IONOS MPG@20, 15 completed runs. Bucketing every run by how
    // many times its rung changed:
    //
    //     changes | runs | mean kbps | best
    //           1 |    7 |    1.93   | 2.55   <- session record, ~= this rung's physical
    //           3 |    5 |    1.68   | 1.89      ceiling on ITU Good @20 (2450 bps)
    //           4 |    1 |    1.45   |
    //           5 |    1 |    1.57   |
    //           6 |    1 |    1.27   |
    //
    // correlation(changes, goodput) = -0.58. Held (<=1) 1.93 vs churned (>=3) 1.59 = +22%.
    //
    // WHY CHURN IS THE DEFECT, not a symptom of a bad estimator:
    //  - The steering variable does not predict the outcome. Across 1292 rig frames,
    //    instantaneous SNR and frame success OVERLAP COMPLETELY (craters at 18.6 dB, clean
    //    frames at 13.1 dB). Sampling that faster cannot help.
    //  - The loop runs inside the coherence time. At MPG, f_d = 0.1 Hz so Tc ~ 4.2 s while a
    //    burst is 8-10 s — LONGER than the channel stays correlated. Every decision is acted
    //    on after its evidence has decorrelated.
    //  - Craters are irreducible on Rayleigh, so "crater => rung too high" is a category
    //    error. docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md measures 8PSK R2/3 as the best
    //    rung on Good@20 (2450 bps) INCLUDING its crater rate — it wins BECAUSE of the FER
    //    trade. Demoting away from it to stop craters is backwards.
    //  - Each change costs a turnaround, a re-acquisition, and the frames cratered while
    //    switching. That is the -0.58.
    //
    // So the fix is not a better estimator or a faster one: it is to STOP RE-DECIDING.
    // Adaptation must track propagation (minutes), not fading (seconds). The dwell is
    // therefore expressed in COHERENCE TIMES and derived from the live fading estimate —
    // Tc = kClarkeCoherenceNumerator / f_d — not as a wall-clock constant, so it stays
    // correct across channel classes by construction (MPG 0.1 Hz -> ~63 s; a rougher
    // channel decorrelates sooner and the dwell shortens with it).
    //
    // ESCAPE: sustained TOTAL craters bypass the dwell. Holding a rung that delivers
    // literally nothing is not adaptivity, it is a stall — and a 0/N group is unambiguous
    // in a way a partial crater is not.
    if (cmd != cur && cur != kRungIdxNone) {
        static const uint32_t kDwellMsOverride = [] {
            if (const char* e = std::getenv("ULTRA_RUNG_DWELL_MS")) {
                const long v = std::atol(e);
                if (v >= 0 && v <= 600000) return static_cast<uint32_t>(v);
            }
            return 0u;  // default OFF
        }();
        if (kDwellMsOverride > 0) {
            const float doppler =
                connection_policy::designDopplerForFadingIndex(eff_fading);
            const float tc_s = (doppler > 0.0f)
                ? connection_policy::kClarkeCoherenceNumerator / doppler
                : 4.23f;
            // The knob sets the dwell directly; the coherence figure is logged beside it so
            // a wrong-timescale setting is visible rather than silent.
            const uint32_t dwell_ms = kDwellMsOverride;
            const auto now_tp = std::chrono::steady_clock::now();
            const bool have_prev = rx_auth_last_change_valid_;
            const auto held_ms =
                have_prev
                    ? std::chrono::duration_cast<std::chrono::milliseconds>(
                          now_tp - rx_auth_last_change_).count()
                    : static_cast<long long>(dwell_ms);
            const bool catastrophic =
                full_crater && rx_auth_crater_streak_ >= kCatastrophicDwellBypassCraters;
            if (!catastrophic && have_prev && held_ms < static_cast<long long>(dwell_ms)) {
                LOG_MODEM(INFO,
                          "Connection: RUNG-DWELL suppressing %u -> %u (held %llds of "
                          "%ums = %.1f x Tc %.1fs); adaptation tracks propagation, not fading",
                          cur, cmd, static_cast<long long>(held_ms / 1000), dwell_ms,
                          dwell_ms / 1000.0f / tc_s, tc_s);
                cmd = cur;
            } else if (catastrophic && have_prev &&
                       held_ms < static_cast<long long>(dwell_ms)) {
                LOG_MODEM(WARN,
                          "Connection: RUNG-DWELL BYPASSED by %d consecutive TOTAL craters "
                          "(%u -> %u after only %llds) — a rung delivering nothing is a "
                          "stall, not a rung to hold",
                          rx_auth_crater_streak_, cur, cmd,
                          static_cast<long long>(held_ms / 1000));
            }
        }
    }
    if (cmd != cur) {
        rx_auth_last_change_ = std::chrono::steady_clock::now();
        rx_auth_last_change_valid_ = true;
    }

    // Canonical indices stay < 24 so bits [rung_cmd] never equal kRungCmdReserved
    // (3) — the WAITING-REBASE voice encoding stays unambiguous (it is also
    // type=NACK, but keep the value space disjoint regardless).
    static_assert(kRungIdxCount <= 24, "rung index would alias the rebase voice");
    rx_authority_cmd_ = cmd;
    if (cmd != cur) {
        // Print the rung actually COMMANDED, not the raw ladder pick: `mapped` is
        // the pre-clamp selection and `cmd` is rewritten by the crater/hold/floor/
        // snap/EVM clamps below it, so the two disagree whenever a clamp fires.
        // Logging `mapped` here made the printed modulation contradict the printed
        // index and corrupted offline rung-trajectory analysis. `raw` is kept so a
        // clamped decision stays diagnosable (raw != idx means a clamp moved it).
        const CoherentPick commanded = coherentRungFromIndex(cmd);
        const uint8_t raw_idx = coherentRungIndexFor(mapped.mod, mapped.rate);
        LOG_MODEM(INFO,
                  "Connection: RX-AUTHORITY verdict %s %s (idx %u -> %u raw=%u) "
                  "snr_avg=%.1f (inst=%.1f n=%d) fading=%.2f coh=%.2f q=%.2f",
                  modulationToString(commanded.mod), codeRateToString(commanded.rate),
                  cur, cmd, raw_idx, snr_avg, burst_obs_snr_db_, snr_n, eff_fading,
                  burst_obs_coh_score_, quality);
    } else if (crater_confirmed && ema_supports_cur) {
        // Made-visible: a confirmed crater that would have demoted but the EMA still
        // clears the rung floor — the crux of ULTRA_RX_EMA_HOLD's A/B signature.
        LOG_MODEM(INFO,
                  "Connection: RX-AUTHORITY EMA-HOLD idx %u (snr_avg=%.1f > anchor=%.1f) "
                  "confirmed crater absorbed (streak=%d fading=%.2f q=%.2f)",
                  cur, snr_avg, cur_class_anchor, rx_auth_crater_streak_, eff_fading,
                  quality);
    }
}

// SENDER side of RX-AUTHORITY: obey a non-zero absolute rung command from the
// receiver's ACK. Dedup by target (repeat ACK copies re-carry the same command);
// clamp to the locally-enabled ladder (env knobs may differ across ends); commit
// via the descriptor (mid-window era-safe) with the legacy MODE_CHANGE fallback.
bool Connection::acceptAuthorityCommandRevision(uint8_t cmd_word, uint8_t group_seq,
                                                uint16_t frame_mask,
                                                uint8_t move_epoch) {
    const uint8_t cmd_idx = static_cast<uint8_t>(
        cmd_word & kRungAuthorityIndexMask);
    const bool same_episode =
        tx_authority_ack_identity_valid_ &&
        tx_authority_ack_group_seq_ == group_seq &&
        tx_authority_ack_move_epoch_ == move_epoch;
    if (same_episode) {
        // An ACK re-emission may revise a startup probe UP into a timeout rollback
        // DOWN while its cumulative SACK bitmap grows.  The bitmap is a revision of
        // the same sequence/epoch episode, not a new authority decision: a delayed
        // older mask must never reverse the safety revision after applyDataMode resets
        // the target dedup latch.
        tx_authority_ack_frame_mask_ = static_cast<uint16_t>(
            tx_authority_ack_frame_mask_ | frame_mask);
        if (cmd_idx != kRungIdxNone &&
            tx_authority_ack_lowest_cmd_ != kRungIdxNone &&
            cmd_idx > tx_authority_ack_lowest_cmd_) {
            LOG_MODEM(WARN,
                      "Connection: stale upward authority revision dropped for "
                      "the same seq/epoch ACK episode (idx=%u floor=%u)",
                      cmd_idx, tx_authority_ack_lowest_cmd_);
            return false;
        }
        if (cmd_idx != kRungIdxNone &&
            (tx_authority_ack_lowest_cmd_ == kRungIdxNone ||
             cmd_idx < tx_authority_ack_lowest_cmd_)) {
            tx_authority_ack_lowest_cmd_ = cmd_idx;
        }
        return true;
    }

    tx_authority_ack_identity_valid_ = true;
    tx_authority_ack_group_seq_ = group_seq;
    tx_authority_ack_frame_mask_ = frame_mask;
    tx_authority_ack_move_epoch_ = move_epoch;
    tx_authority_ack_lowest_cmd_ = cmd_idx;
    return true;
}

bool Connection::startupProbeHasSufficientPayload() const {
    constexpr size_t kMinTrustedProbeFrames = 4;
    if (file_transfer_.getState() != FileTransferState::SENDING) return false;
    const auto progress = file_transfer_.getProgress();
    if (progress.total_bytes <= progress.transferred_bytes) return false;

    const int cw = (config_.forced_cw_count != 0)
        ? v2::sanitizeFixedFrameCodewords(config_.forced_cw_count)
        : connection_policy::recommendCWCountForChannel(
              Modulation::QPSK, CodeRate::R2_3, negotiated_mode_,
              connection_policy::coherenceAdjustedFadingIndex(
                  fading_index_, coherence_score_, coherence_valid_),
              wireSnrDb());
    const int physical_cw = physicalDataFrameCodewordsFor(
        Modulation::QPSK, CodeRate::R2_3, cw);
    const int lifting_z = selectBurstLiftingZFor(
        Modulation::QPSK, CodeRate::R2_3, cw);
    const size_t frame_payload = (lifting_z == 81)
        ? v2::getFixedFramePayloadCapacityZ(CodeRate::R2_3, physical_cw, 81)
        : v2::getFixedFramePayloadCapacity(CodeRate::R2_3, physical_cw);
    if (frame_payload <= FileTransferController::FILE_DATA_OVERHEAD) return false;

    // Four bytes-worth of prospective frames is not enough: all four must fit in
    // ONE physical target-rung key-down, because the receiver deliberately refuses
    // to qualify a startup probe with M<4.  Forced long CWs and unusual airtime
    // ceilings can otherwise pass the byte test below but produce only N=2/3, wasting
    // the QSO-scoped experiment on an outcome we are required to distrust.  Mirror the
    // exact post-switch window/ceiling/re-anchor geometry used by configureArqForCurrentDataMode()
    // and burstAirtimeBudgetFrames().
    size_t target_window = connection_policy::ofdmWindowSizeForChannel(
        Modulation::QPSK, CodeRate::R2_3, fading_index_, rateSelectionSnrDb());
    if (kInteractiveToneAckEnabled()) {
        target_window = std::min<size_t>(
            target_window, connection_policy::kToneBurstAckWindowCapFrames);
    }
    const uint32_t target_ceiling_ms = connection_policy::burstAirtimeCeilingMs(
        Modulation::QPSK, CodeRate::R2_3, burst_clean_group_streak_);
    const uint32_t target_reanchor_ms =
        connection_policy::shouldUseWideOFDMShortReanchor(
            negotiated_mode_, Modulation::QPSK, fading_index_)
            ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
            : 0;
    const size_t target_group_frames =
        connection_policy::wideOFDMBurstFrameBudget(
            Modulation::QPSK, CodeRate::R2_3, physical_cw, target_window,
            target_ceiling_ms, target_reanchor_ms, lifting_z);
    if (target_group_frames < kMinTrustedProbeFrames) return false;

    const size_t chunk = frame_payload - FileTransferController::FILE_DATA_OVERHEAD;
    const size_t remaining = progress.total_bytes - progress.transferred_bytes;
    const size_t prospective_frames = (remaining + chunk - 1) / chunk;
    return prospective_frames >= kMinTrustedProbeFrames;
}

bool Connection::authorityClimbHasSufficientPayload(
    Modulation target_mod, CodeRate target_rate,
    int post_ack_clean_streak) const {
    // This is an economic guard for an ACTIVE file only.  Between payloads there is
    // no short tail to strand, and reliability DOWN commands never call this helper.
    if (file_transfer_.getState() != FileTransferState::SENDING) return true;

    // maybeObeyAuthorityCommand calls this only at a clean send boundary, where
    // remainingTxBytes() is exact: all submitted chunks have been ACKed and the value
    // therefore represents the complete unsent tail.  Calling it while the window is
    // busy would under-count already-submitted bytes, so keep the busy guard at the
    // caller immediately ahead of this one.
    const size_t remaining = file_transfer_.remainingTxBytes();
    const int cw = (config_.forced_cw_count != 0)
        ? v2::sanitizeFixedFrameCodewords(config_.forced_cw_count)
        : connection_policy::recommendCWCountForChannel(
              target_mod, target_rate, negotiated_mode_,
              connection_policy::coherenceAdjustedFadingIndex(
                  fading_index_, coherence_score_, coherence_valid_),
              wireSnrDb());
    const int physical_cw = physicalDataFrameCodewordsFor(
        target_mod, target_rate, cw);
    const int lifting_z = selectBurstLiftingZFor(
        target_mod, target_rate, cw);
    const size_t frame_payload = (lifting_z == 81)
        ? v2::getFixedFramePayloadCapacityZ(target_rate, physical_cw, 81)
        : v2::getFixedFramePayloadCapacity(target_rate, physical_cw);
    if (frame_payload <= FileTransferController::FILE_DATA_OVERHEAD) return false;

    size_t target_window = connection_policy::ofdmWindowSizeForChannel(
        target_mod, target_rate, fading_index_, rateSelectionSnrDb());
    if (kInteractiveToneAckEnabled()) {
        target_window = std::min<size_t>(
            target_window, connection_policy::kToneBurstAckWindowCapFrames);
    }
    const uint32_t target_ceiling_ms = connection_policy::burstAirtimeCeilingMs(
        target_mod, target_rate, post_ack_clean_streak);
    const uint32_t target_reanchor_ms =
        connection_policy::shouldUseWideOFDMShortReanchor(
            negotiated_mode_, target_mod, fading_index_)
            ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
            : 0;
    const size_t target_group_frames =
        connection_policy::wideOFDMBurstFrameBudget(
            target_mod, target_rate, physical_cw, target_window, target_ceiling_ms,
            target_reanchor_ms, lifting_z,
            // A descriptor-committed climb's first target group is deliberately
            // full-anchored. Price the group that can actually be emitted, not its
            // later warm steady-state shape.
            connection_policy::kWideOFDMFullAnchorExtraMs);
    const size_t chunk = frame_payload - FileTransferController::FILE_DATA_OVERHEAD;
    const size_t prospective_frames =
        (remaining == 0) ? 0 : (remaining + chunk - 1) / chunk;

    // The latent argmax prices a steady-state candidate using a full target group.
    // Executing that result as a shorter tail is a different counterfactual.  The
    // final_default_07 trace priced N=8, switched for N=3, decoded 1/3, then paid a
    // synchronized demote and two-chunk rewind.  Refuse only the optimistic move;
    // current-rung refill continues and the logical FINAL ACK neutralizes the command.
    if (target_group_frames == 0 || prospective_frames < target_group_frames) {
        LOG_MODEM(INFO,
                  "Connection: RX-AUTHORITY tail hold %s %s -> %s %s "
                  "(%zu prospective target frame(s) < post-ACK target group %zu; "
                  "%zu byte(s) remain)",
                  modulationToString(data_modulation_),
                  codeRateToString(data_code_rate_),
                  modulationToString(target_mod), codeRateToString(target_rate),
                  prospective_frames, target_group_frames, remaining);
        return false;
    }
    return true;
}

void Connection::maybeObeyAuthorityCommand(uint8_t cmd_idx,
                                           bool accepted_clean_round) {
    const bool startup_probe_command =
        (cmd_idx & kRungAuthorityStartupProbeFlag) != 0;
    cmd_idx &= kRungAuthorityIndexMask;
    if (cmd_idx == kRungIdxNone || cmd_idx >= kRungIdxCount) return;
    if (startup_probe_command && cmd_idx != kRungIdxQpskR23) {
        LOG_MODEM(WARN,
                  "Connection: ignoring malformed startup-probe authority idx=%u",
                  cmd_idx);
        return;
    }
    if (startup_probe_command && !on_transmit_burst_) {
        // The one-physical-group guarantee depends on staged timeout batches.  A
        // transport without burst staging cannot safely run this experiment.
        LOG_MODEM(WARN,
                  "Connection: ignoring startup probe without staged burst egress");
        return;
    }
    if (startup_probe_command && !startupProbeHasSufficientPayload()) {
        LOG_MODEM(INFO,
                  "Connection: ignoring startup probe — remaining payload/physical "
                  "geometry cannot form one trusted four-frame target group");
        return;
    }
    if (state_ != ConnectionState::CONNECTED ||
        negotiated_mode_ != WaveformMode::OFDM_CHIRP) return;
    // The operator pin is the final authority. Receiver authority replaces the
    // adaptive controller, but it must not bypass ULTRA_LOCK_RATE or an explicit
    // ULTRA_RATE_ADAPT=0 fixed-rung measurement.
    if (!rateAdaptationActive()) return;
    if (mode_change_pending_) return;  // a move is already in flight — obey later copies
    // The endpoints may have asymmetric environments.  Re-apply the sender-local
    // absolute ceiling before resolving the peer's command; receiver authority is not
    // permission to violate an operator cap on this transmitter.
    const uint8_t ceiling = latentConfiguredRungCeiling();
    if (ceiling != kRungIdxNone && cmd_idx > ceiling) {
        LOG_MODEM(INFO,
                  "Connection: RX-AUTHORITY command idx=%u capped locally to idx=%u",
                  cmd_idx, ceiling);
        cmd_idx = ceiling;
    }
    CoherentPick pick = coherentRungFromIndex(cmd_idx);
    Modulation mod = pick.mod;
    CodeRate rate = pick.rate;
    if (!coherentRungLocallyEnabled(mod, rate)) {
        // Command names a rung the local ladder disabled (knob/build mismatch, or
        // a peer whose stride math predates the enabled-ladder snap). HOLDING here
        // deadlocked F145: a DOWN command into the QAM8 R3/4 hole was refused 4×
        // while 16QAM R2/3 re-cratered for 50 s. Obey the nearest enabled rung at
        // or below the command instead — a DOWN command must produce a DOWN move.
        const uint8_t snapped = snapRungIndexDownToEnabled(cmd_idx);
        if (snapped == kRungIdxNone) {
            LOG_MODEM(WARN,
                      "Connection: RX-AUTHORITY command idx=%u (%s %s) has no "
                      "enabled rung at or below it — holding current rung",
                      cmd_idx, modulationToString(mod), codeRateToString(rate));
            return;
        }
        pick = coherentRungFromIndex(snapped);
        LOG_MODEM(WARN,
                  "Connection: RX-AUTHORITY command idx=%u (%s %s) not locally "
                  "enabled — snapping to idx=%u (%s %s)",
                  cmd_idx, modulationToString(mod), codeRateToString(rate),
                  snapped, modulationToString(pick.mod), codeRateToString(pick.rate));
        cmd_idx = snapped;
        mod = pick.mod;
        rate = pick.rate;
    }
    if (mod == data_modulation_ && rate == data_code_rate_) {
        if (tx_latent_startup_probe_active_ && !startup_probe_command &&
            cmd_idx == kRungIdxQpskR23) {
            // The receiver decoded the one physical target group and retained the
            // target without the probe bit: direct success confirmation.
            tx_latent_startup_probe_active_ = false;
            tx_latent_startup_probe_airborne_ = false;
            tx_latent_startup_probe_timeout_rollback_pending_ = false;
            tx_latent_startup_probe_base_rung_ = kRungIdxNone;
            LOG_MODEM(INFO,
                      "Connection: LATENT startup probe confirmed by target-group ACK");
        }
        tx_authority_last_obeyed_ = cmd_idx;  // already there — arm dedup anyway
        return;
    }
    if (cmd_idx == tx_authority_last_obeyed_) {
        // Same target as last obeyed but we're not there yet (descriptor commit
        // still propagating / legacy exchange in flight) — don't re-fire on every
        // repeated ACK copy carrying the same command.
        return;
    }
    const uint8_t current_before_move = coherentRungIndexFor(
        data_modulation_, data_code_rate_);
    const bool valid_startup_probe =
        startup_probe_command && current_before_move == kRungIdxQpskR12 &&
        cmd_idx == kRungIdxQpskR23;
    if (startup_probe_command && !valid_startup_probe) {
        LOG_MODEM(WARN,
                  "Connection: ignoring startup-probe command outside R1/2->R2/3 "
                  "transition (cur=%u target=%u)",
                  current_before_move, cmd_idx);
        return;
    }
    if (tx_latent_startup_probe_active_ && !startup_probe_command) {
        // A non-probe command is the receiver's success/rollback verdict. Let the
        // ordinary authority move below obey it, but disarm RTO interception.
        tx_latent_startup_probe_active_ = false;
        tx_latent_startup_probe_airborne_ = false;
        tx_latent_startup_probe_timeout_rollback_pending_ = false;
        tx_latent_startup_probe_base_rung_ = kRungIdxNone;
    }
    const bool faster = isFasterMode(mod, rate, data_modulation_, data_code_rate_);
    const bool busy = hasGeometryBoundDataOperation();
    if (faster) {
        // UP-commands defer to a CLEAN send boundary (F122): a mid-window switch
        // discards the receiver's buffered frames and rewinds the file cursor by
        // the whole in-flight window — pure gain-chasing must never pay that
        // redundant-airtime tax. The receiver re-stamps the command on every ACK,
        // so the deferred climb re-asserts free at the next full-ack tick.
        // DOWN-commands obey immediately: a failing window never drains (waiting
        // would deadlock), and its frames need resending anyway — the rewind is
        // free information-wise; move-epoch makes it era-safe.
        if (busy) {
            LOG_MODEM(DEBUG,
                      "Connection: RX-AUTHORITY hold climb idx=%u for clean boundary",
                      cmd_idx);
            return;  // do NOT arm dedup — the re-carried command must retry
        }
        // A clean boundary makes remainingTxBytes() exact.  Keep the automatic
        // steady-state argmax from spending a descriptor/MODE_CHANGE on a file tail
        // shorter than the complete target burst it priced.  This return is a real
        // HOLD, not tryDescriptorModeSwitch(false): never fall through to the legacy
        // control exchange, and do not arm the dedup latch.
        // A flagged startup probe has its own deliberately smaller M>=4 evidence
        // contract, checked above.  The ordinary latent argmax instead assumes a
        // complete steady-state target burst.  Account for this accepted clean ACK
        // now, even though noteArqRoundOutcome() commits the streak later in order to
        // preserve the callback's single coalesced refill.
        if (!valid_startup_probe) {
            const int post_ack_clean_streak = burst_clean_group_streak_ +
                (accepted_clean_round ? 1 : 0);
            if (!authorityClimbHasSufficientPayload(
                    mod, rate, post_ack_clean_streak)) {
                return;
            }
        }
    }
    const uint8_t reason = faster ? v2::ModeChangeReason::CHANNEL_IMPROVED
                                  : v2::ModeChangeReason::CHANNEL_DEGRADED;
    const char* old_mod = modulationToString(data_modulation_);
    const char* old_rate = codeRateToString(data_code_rate_);
    if (valid_startup_probe) {
        // Arm BEFORE the commit: descriptor switching synchronously refills the file,
        // so the first target group can reach noteDataBurstKeydown inside
        // tryDescriptorModeSwitch().
        tx_latent_startup_probe_active_ = true;
        tx_latent_startup_probe_airborne_ = false;
        tx_latent_startup_probe_timeout_rollback_pending_ = false;
        tx_latent_startup_probe_base_rung_ = current_before_move;
        LOG_MODEM(INFO,
                  "Connection: LATENT sender armed one-shot R1/2->R2/3 probe");
    }
    // A DOWN command may arrive mid-window.  Descriptor regrid is era-safe there only
    // with move-epoch enabled; otherwise use the synchronized MODE_CHANGE handshake.
    bool desc_committed = false;
    if (!busy || arq_.moveEpochEnabled()) {
        desc_committed = tryDescriptorModeSwitch(mod, rate, wireSnrDb(), reason);
    }
    if (!desc_committed) {
        // A legacy synchronized launch needs its own give-up semantics. If the peer
        // applies R2/3 but every control ACK is lost, resuming base-rung DATA would
        // create a split-geometry session. The timeout path recognizes this reason
        // and disconnects instead of resuming ambiguous file intents.
        requestModeChange(
            mod, rate, wireSnrDb(),
            valid_startup_probe ? v2::ModeChangeReason::STARTUP_PROBE_BEGIN
                                : reason);
    }
    tx_authority_last_obeyed_ = cmd_idx;
    LOG_MODEM(INFO, "Connection: RX-AUTHORITY obey %s %s -> %s %s via %s",
              old_mod, old_rate, modulationToString(mod), codeRateToString(rate),
              desc_committed ? "DESC-SWITCH" : "MODE_CHANGE");
}

// SENDER side (called from onToneBurstAck, inside the defer-refill bracket, AFTER
// applyAdaptiveRateFeedback). The command is ADVISORY: it routes through the sender's
// own guards, ladder tables, caps and cooldowns — the receiver is authoritative about
// what it could not decode; the sender stays authoritative about what it transmits
// (design §4.1 arbitration).
void Connection::maybeApplyRxRateCommand(uint8_t cmd, uint8_t group_seq,
                                         Modulation mod_at_ack, CodeRate rate_at_ack) {
    using ultra::waveform::tone_burst_ack::kRungCmdDownHard;
    using ultra::waveform::tone_burst_ack::kRungCmdDownOne;
    if (!rx_rate_cmd_enabled_) return;  // knob OFF: byte-identical (bits ignored)
    if (cmd != kRungCmdDownOne && cmd != kRungCmdDownHard) {
        return;  // 0 = no command; 3 = reserved, treat as hold (forward compat)
    }
    if (state_ != ConnectionState::CONNECTED) return;
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) return;  // wideband ladder only
    // One action attempt per command episode: a crater freezes the ARQ base, so every
    // re-emitted copy of the command carries the same group_seq (the drive_advisory
    // dedup pattern, connection.cpp:1718-1731 class). Recorded BEFORE the policy
    // guards below — a transiently-blocked command is dropped, not retried later
    // under the same seq (fail-soft to the sender's own escape backstops, which
    // remain armed and unchanged).
    if (static_cast<int>(group_seq) == rx_rate_cmd_seq_seen_) return;
    rx_rate_cmd_seq_seen_ = group_seq;
    if (!rateAdaptationActive()) return;  // ULTRA_LOCK_RATE / rate-adapt-off wins
    if (mode_change_pending_) return;     // a legacy exchange is already re-anchoring
    if (mod_at_ack != data_modulation_ || rate_at_ack != data_code_rate_) {
        // The EMA/QAM16 machinery already moved the rung on THIS ack (the command and
        // the quality byte are the same channel evidence measured two ways) — one ACK
        // may move the ladder at most once.
        return;
    }

    // Target selection — the sender's OWN ladder tables, never a receiver-named rung:
    // DOWN-hard mirrors executeEscapeDrop exactly (crater semantics: QAM16 → straight
    // to the robust QPSK R3/4 home gear + double-weight re-climb cooldown; below QAM16
    // → one rung more robust + noteRungFailed so ssthresh remembers the cratered
    // rung). DOWN-one mirrors the soft ack-driven demote ladder (single-weight).
    Modulation target_mod = data_modulation_;
    CodeRate target_rate = data_code_rate_;
    if (data_modulation_ == Modulation::QAM16) {
        if (cmd == kRungCmdDownOne && qam16R34Enabled() &&
            data_code_rate_ == CodeRate::R3_4) {
            target_rate = CodeRate::R2_3;  // crest-rung step-down: stays on QAM16
        } else if (qam16DemoteMidrungEnabled() &&
                   data_code_rate_ != CodeRate::R1_2) {
            // MID-RUNG landing: stay on 16QAM at R1/2 (2x margin) instead of the
            // QPSK R3/4 exit. Stays-on-QAM16 ⇒ no noteQam16Demoted (mirrors the
            // crest step-down above); a further crater lands QPSK via the else.
            target_rate = CodeRate::R1_2;
        } else {
            target_mod = Modulation::QPSK;
            target_rate = CodeRate::R3_4;
            noteQam16Demoted(cmd == kRungCmdDownHard ? 2 : 1);
        }
    } else {
        const CodeRate robust = rate_controller_.moreRobustRung(data_code_rate_);
        if (robust == data_code_rate_) {
            return;  // already the most robust rung — irreducible (floor guard)
        }
        if (cmd == kRungCmdDownHard) {
            rate_controller_.noteRungFailed(data_code_rate_);
        }
        target_rate = robust;
    }

    // COMMIT. Mid-window is the whole point of the command (the crater keeps the
    // window busy, so the clean boundary the Phase-1 sites wait for never comes):
    // a descriptor commit there regrids live seqs → only era-safe under the
    // move-epoch machinery, so gate on ULTRA_ARQ_MOVE_EPOCH and fall back to the
    // legacy synchronized exchange without it (exactly executeEscapeDrop's commit).
    // At a clean boundary no epoch is needed (Phase-1 invariant: empty window ⇒
    // nothing to abort). tryDescriptorModeSwitch re-checks the Phase-1 knob and
    // scope (incl. the descriptor-bearing >1-frame-remaining guard) and returns
    // false out of scope — the guards compose, never duplicate.
    const bool busy = hasGeometryBoundDataOperation();
    bool desc_committed = false;
    if (!busy || arq_.moveEpochEnabled()) {
        desc_committed = tryDescriptorModeSwitch(
            target_mod, target_rate, wireSnrDb(),
            v2::ModeChangeReason::CHANNEL_DEGRADED);
    }
    if (!desc_committed) {
        requestModeChange(target_mod, target_rate, wireSnrDb(),
                          v2::ModeChangeReason::CHANNEL_DEGRADED);
    }
    char buf[112];
    std::snprintf(buf, sizeof(buf), "RX-RATE-CMD %s: %s %s -> %s %s via %s (seq=%u)",
                  cmd == kRungCmdDownHard ? "down-hard" : "down-one",
                  modulationToString(mod_at_ack), codeRateToString(rate_at_ack),
                  modulationToString(target_mod), codeRateToString(target_rate),
                  desc_committed ? "DESC-SWITCH" : "MODE_CHANGE",
                  static_cast<unsigned>(group_seq));
    LOG_MODEM(WARN, "Connection: %s", buf);
    last_adaptive_action_ = buf;
}

void Connection::maybeEscapeStuckFrame() {
    if (!rateAdaptationActive()) return;
    if (state_ != ConnectionState::CONNECTED) return;
    if (mode_change_pending_) return;             // a rate change is already in flight
    if (!isOFDMMode(negotiated_mode_)) return;    // burst-transport rate ladder only
    if (arq_.getTxInFlightBytes() == 0) return;   // nothing in flight to be stuck
    if (rate_controller_.isAtFloor(data_code_rate_)) return;  // already most robust — irreducible
    if (arq_.maxInFlightRetryCount() < stuckRetransmitEscape()) return;

    // The pathological-single-frame 5-retx backstop (ULTRA_STUCK_ESCAPE_RETX), kept verbatim
    // (§RETX_PACING_DESIGN_2026_07_03 §2.3). During genuine collapses the round-conditioned
    // maybeCollapseEscape typically preempts this (2 rounds ≈ retry 2-3 < 5); on healthy
    // windows only this backstop can fire — exactly the pre-pacing behavior.
    char trigger[64];
    std::snprintf(trigger, sizeof(trigger), "frame stuck, %d retx at current rate",
                  arq_.maxInFlightRetryCount());
    executeEscapeDrop(trigger);
}

// Collapse-conditioned escape (§RETX_PACING_DESIGN_2026_07_03 §2, behind
// ULTRA_COLLAPSE_ESCAPE_ROUNDS, default OFF). Polled from the CONNECTED tick beside
// maybeEscapeStuckFrame — NEVER fired from inside an ARQ callback (an RTO-batch round
// increments the counter while its physical batch is staged; the escape lands after
// arq_.tick unwinds and before that batch is committed). Zero-DELIVERED evidence: only a whole-window zero
// streak (the g43/rig frozen-base signature) trips it; any progress reset the counter
// (noteArqRoundOutcome), so a lone straggler amid deliveries cannot (g42-protective).
void Connection::maybeCollapseEscape() {
    const int rounds_needed = collapseEscapeRounds();
    if (rounds_needed <= 0) return;               // knob OFF (default) — byte-identical
    if (zero_progress_rounds_ < rounds_needed) return;
    if (!rateAdaptationActive()) return;
    if (state_ != ConnectionState::CONNECTED) return;
    if (mode_change_pending_) return;             // a rate change is already in flight
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) return;  // scope gate (§1.3/§6.2)
    if (arq_.getTxInFlightBytes() == 0) return;   // nothing in flight to be collapsing
    if (rate_controller_.isAtFloor(data_code_rate_)) return;  // already most robust — irreducible

    // ESCAPE EPISODE CAP (ULTRA_ESCAPE_EPISODE_CAP, default 0 = OFF = unlimited/legacy).
    // A fade null is TIME-bounded (~Tc-scale), not rate-bounded: during the null NO rung
    // delivers (SNR is -inf in the null regardless of code rate), so zero-progress
    // evidence measures time-stuck, not rate error. ONE drop hedges the post-null
    // decode; further drops buy zero delivery during the null and cost minutes of
    // under-rated cruise after (F89: one trough cascaded R3/4->R2/3->R1/2->R1/4 while
    // the deliveries BETWEEN stalls ran q=0.94-0.99). Saturate at the cap per silent
    // episode (consecutive_escape_drops_ resets on ANY ack progress — the episode
    // boundary); the trough-pacing hold owns the time axis, and a genuinely over-
    // climbed rung post-null still demotes via the ack-driven RateController (partial
    // deliveries feed it) and the stuck-frame escape (frame-death evidence, uncapped).
    static const int kEscapeEpisodeCap = [] {
        if (const char* e = std::getenv("ULTRA_ESCAPE_EPISODE_CAP")) {
            const int n = std::atoi(e);
            if (n >= 1 && n <= 8) return n;
        }
        return 1;  // DEFAULT 2026-07-05 (campaign standing value; 0 = unlimited cascade)
    }();
    if (kEscapeEpisodeCap > 0 && consecutive_escape_drops_ >= kEscapeEpisodeCap) {
        LOG_MODEM(INFO,
                  "Connection: COLLAPSE-escape SATURATED (episode cap %d, drops %d) — "
                  "holding rung through the null",
                  kEscapeEpisodeCap, consecutive_escape_drops_);
        zero_progress_rounds_ = 0;  // consume the evidence; re-accumulates before the next poll
        return;
    }

    // §2.1 window-collapse evidence: ≥⌈burst_cap/2⌉ frames pending at the current rung.
    // in_flight ≥ ceil(cap/2) ⇔ 2·in_flight ≥ cap (integer).
    const size_t burst_cap = burstAirtimeBudgetFrames(arq_.getWindowSize());
    const size_t in_flight_frames = arq_.getWindowSize() - arq_.getAvailableSlots();
    if (2 * in_flight_frames < burst_cap) return;

    LOG_MODEM(WARN, "Connection: COLLAPSE-escape (%d zero rounds)", zero_progress_rounds_);
    char trigger[64];
    std::snprintf(trigger, sizeof(trigger), "collapse, %d zero-progress rounds",
                  zero_progress_rounds_);
    // Reset the era before the drop — requestModeChange sets mode_change_pending_ and the
    // commit path (applyDataMode) starts a new era anyway; this keeps the counter from
    // double-firing if the MODE_CHANGE round-trip is slow.
    zero_progress_rounds_ = 0;
    retx_pace_hold_ms_ = 0;
    executeEscapeDrop(trigger);
}

// §RETX_PACING_DESIGN_2026_07_03 §1.3 scope gate: CONNECTED **wideband** OFDM
// (OFDM_CHIRP only) on the unified tone-burst burst path, file SENDING with bytes in
// flight — the mirror of the escape's guards. MC-DPSK and OFDM_NARROW are explicitly OUT
// of scope (§6.2): their ACK timers were just re-derived (BUG-MCDPSK-ACK-COLLISION /
// BUG-MCDPSK-FILE-COMPLETION) and their RTT already dwarfs any Tc.
bool Connection::retxPacingScopeActive() const {
    return state_ == ConnectionState::CONNECTED &&
           negotiated_mode_ == WaveformMode::OFDM_CHIRP &&
           use_burst_transport_ && kUnifiedSeqEnabled() &&
           file_transfer_.getState() == FileTransferState::SENDING &&
           arq_.getTxInFlightBytes() > 0;
}

// §RETX-PACING: record the modeled END of an OFDM data-burst key-down (flush time +
// exact emitted samples, including the descriptor, repair anchor, and TX guards). This is
// the reference point for T_defer's t_since_last_tx_end subtraction (§1.2): by the time
// the sender LEARNS a round was zero-progress it has already spent part of Tc listening —
// ~3-4 s on the fast-NACK path (deferral bites), ~10+ s on the RTO path (deferral ≈ 0 at
// Good, correct: the RTO already over-paces that path). Recording is unconditional and
// behavior-free (a clock read + member store); every DECISION stays knob-gated.
Connection::PhysicalDataRoundTiming Connection::physicalDataRoundTiming(
    const std::vector<Bytes>& transmitted_frames,
    bool force_full_group_start) const {
    PhysicalDataRoundTiming result;
    // This model is deliberately the 48 kHz wide OFDM encoder geometry.  Narrow OFDM
    // and MC-DPSK have independent, already-derived RTT policies and must never be
    // re-timed from wide symbols/anchors.
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP ||
        transmitted_frames.empty()) {
        return result;
    }

    const uint32_t reanchor_ms =
        connection_policy::shouldUseWideOFDMShortReanchor(
            negotiated_mode_, data_modulation_, fading_index_)
            ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
            : 0;
    // Every one-frame Connection egress uses the standalone on_transmit_ path;
    // ModemEngine::transmit() resets that encoder path to z=27 because there is
    // no BURST_HEADER descriptor that could announce z=81 to the receiver.
    const int fixed_lifting_z =
        transmitted_frames.size() == 1 ? 27 : selectBurstLiftingZ();
    uint64_t data_samples = 0;
    uint64_t remaining_data_samples = 0;
    uint32_t max_frame_ms = 0;

    for (size_t i = 0; i < transmitted_frames.size(); ++i) {
        const auto header = v2::parseHeader(transmitted_frames[i]);
        int codewords = physicalDataFrameCodewords();
        int lifting_z = fixed_lifting_z;
        if (header.valid && !header.is_control && header.total_cw > 0) {
            codewords = header.total_cw;
            // StreamingEncoder::encodeFrameBytes routes large variable DATA and
            // DATA_REPAIR through encodeFrameWithLDPC/encodeInfoCodewordsWithLDPC,
            // whose wire codewords are always the short z=27 LDPC geometry.
            if (codewords > v2::kMaxFixedFrameCodewords ||
                header.type == v2::FrameType::DATA_REPAIR) {
                lifting_z = 27;
            }
        }

        const uint64_t frame_samples =
            connection_policy::wideOFDMWireFrameSamplesForCodewords(
                data_modulation_, data_code_rate_, codewords, lifting_z);
        const uint32_t frame_ms =
            connection_policy::sampleDurationCeilMs(frame_samples);
        data_samples += frame_samples;
        if (i > 0) {
            remaining_data_samples += frame_samples;
        }
        max_frame_ms = std::max(max_frame_ms, frame_ms);
        result.total_codewords += static_cast<uint32_t>(std::clamp(codewords, 1, 255));
        result.max_codewords = std::max<uint8_t>(
            result.max_codewords, static_cast<uint8_t>(std::clamp(codewords, 1, 255)));
        if (codewords > v2::kMaxFixedFrameCodewords) {
            ++result.variable_frames;
        }
    }

    // Sample-exact default StreamingEncoder shape. A singleton is
    // [full chirp + its LTS/data]. A multi-frame turn is
    // [full descriptor chirp + QPSK-R1/4 descriptor LTS/data] followed by each
    // DATA frame's light-LTS/data block.
    uint64_t burst_samples =
        data_samples + connection_policy::kWideOFDMFullAnchorExtraSamples;
    if (transmitted_frames.size() > 1) {
        burst_samples +=
            connection_policy::wideOFDMWireFrameSamplesForCodewords(
                wideOFDMControlModulationForData(data_modulation_),
                CodeRate::R1_4, 1, 27);
        burst_samples +=
            static_cast<uint64_t>(transmitted_frames.size() - 1) *
            connection_policy::txGuardSamplesForMs(
                static_cast<int>(reanchor_ms));
        if (force_full_group_start) {
            // encodeBurstLight(..., force_full_preamble=true) emits a reliability
            // group-start chirp+LTS in addition to the descriptor anchor already
            // present in every multi-frame burst.
            burst_samples += connection_policy::kWideOFDMFullAnchorExtraSamples;
        }
    }
    result.waveform_samples = burst_samples;
    result.keyed_samples =
        connection_policy::postProcessedTxSamples(result.waveform_samples);
    result.waveform_airtime_ms =
        connection_policy::sampleDurationCeilMs(result.waveform_samples);
    result.airtime_ms =
        connection_policy::sampleDurationCeilMs(result.keyed_samples);
    result.ack_timeout_ms =
        connection_policy::unifiedBurstAckTimeoutFromPhysicalGeometryMs(
            result.airtime_ms,
            connection_policy::sampleDurationCeilMs(remaining_data_samples),
            max_frame_ms,
            wideOFDMControlModulationForData(data_modulation_),
            transmitted_frames.size() > 1 && !force_full_group_start
                ? connection_policy::kWideOFDMFullAnchorExtraMs
                : 0u,
            reanchor_ms);
    return result;
}

uint32_t Connection::noteDataBurstKeydown(
    const std::vector<Bytes>& transmitted_frames,
    bool force_full_group_start) {
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP || transmitted_frames.empty()) {
        return 0;
    }
    if (tx_latent_startup_probe_active_ &&
        coherentRungIndexFor(data_modulation_, data_code_rate_) == kRungIdxQpskR23) {
        if (tx_latent_startup_probe_airborne_) {
            // Defense in depth: the staged RTO path below should intercept before a
            // second target egress. Never silently weaken the one-shot contract.
            LOG_MODEM(ERROR,
                      "Connection: invariant violation — second LATENT startup-probe "
                      "egress reached physical accounting");
        } else {
            tx_latent_startup_probe_airborne_ = true;
            LOG_MODEM(INFO,
                      "Connection: LATENT startup probe physical group committed");
        }
    }
    const uint32_t airtime_ms =
        physicalDataRoundTiming(transmitted_frames, force_full_group_start).airtime_ms;
    // F163 PLAY-HEAD CARRY: the audio device serializes — a burst submitted while
    // the previous one is still (modeled as) airing QUEUES BEHIND it and starts
    // at the previous modeled end, not now. Without the carry, back-to-back
    // flushes (RTO resend + escape re-encode 66 ms apart = 16.4 s continuous
    // keydown in F163) under-model the second burst's end and the ARQ slot RTO
    // — armed from SUBMIT time — can fire while its own burst is still ON AIR
    // (observed at t=346: a whole doomed duplicate burst).
    const auto now = std::chrono::steady_clock::now();
    const auto start = (last_data_burst_end_valid_ && last_data_burst_end_ > now)
                           ? last_data_burst_end_
                           : now;
    const uint32_t queue_delay_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(start - now).count());
    last_data_burst_end_ = start + std::chrono::milliseconds(airtime_ms);
    last_data_burst_end_valid_ = true;
    if (queue_delay_ms > 0) {
        LOG_MODEM(INFO,
                  "Connection: TX queue-behind %u ms — transmitted-frame deadline "
                  "will follow the air schedule (burst airtime %u ms)",
                  queue_delay_ms, airtime_ms);
    }
    return queue_delay_ms;
}

uint32_t Connection::elapsedSinceLastDataBurstEndMs() const {
    if (!last_data_burst_end_valid_) {
        return 0;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now <= last_data_burst_end_) {
        return 0;  // still (modeled as) keyed down — no listening time elapsed yet
    }
    const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now - last_data_burst_end_).count();
    return static_cast<uint32_t>(std::min<long long>(ms, 0xFFFFFFFFll));
}

// §RETX_PACING_DESIGN_2026_07_03 §1.1/§1.2: one call per ROUND boundary.
//   progress_frames > 0  → the channel just proved it delivers: reset the zero-round
//                          streak and EARLY-RELEASE any armed hold (never wait out a hold
//                          when a late/duplicate SACK finally decodes) — §2.3
//                          g42-protective property.
//   progress_frames == 0 → a fully-failed round (fresh ack with no base advance and no
//                          new SACK bit, or a slot-RTO batch — a timeout IS the absence
//                          of an ack): count it; knob-gated, arm the channel-derived
//                          deferral on BOTH resend triggers (§1.3).
//   progress_frames < 0  → no fresh ack was processed (duplicate/stale/future — the ARQ
//                          ack-signature dedup): NOT a round, touch nothing.
// Partial-SACK rounds land here with progress > 0 and therefore resend immediately
// (status quo) — only zero-progress rounds ever defer (§3).
// TROUGH AMNESTY (ULTRA_TROUGH_AMNESTY, default OFF = byte-identical). A fade null is
// TIME-bounded: a rung proven clean seconds before it is not invalidated by it. Yet the
// down-side (collapse escape + quality-EMA demote) and the up-side (ssthresh pins +
// per-rung climb streaks) both treat trough evidence as RATE evidence — F89/F91 measured
// a ~20 s bidirectional/forward null demoting three rungs and then sitting at R1/4 for
// minutes while delivering 8/8 q=0.9+. When the episode ends (the first progress-bearing
// ack), restore the pre-episode rung directly: if the channel genuinely worsened, one
// crater at the restored rung re-demotes in ~3.5 s (receiver rung command) — bounded
// downside, minutes of upside. Ordering with the receiver's rung command: the command
// applies BEFORE the round outcome in onToneBurstAck, so (a) an episode that starts on a
// crater ack snapshots the POST-crater rung (crater evidence kept), and (b) amnesty
// SKIPS when this ack carries a fresh DOWN command (receiver evidence wins).
static int coherentRungOrdinal(Modulation m, CodeRate r) {
    const int mod_rank = (m == Modulation::QAM16) ? 1 : 0;
    return mod_rank * 16 + static_cast<int>(ofdmCodeRateValue(r) * 12.0f + 0.5f);
}
void Connection::maybeTroughAmnesty(int progress_frames, uint8_t rung_cmd) {
    if (!trough_episode_active_ || progress_frames <= 0) return;
    trough_episode_active_ = false;  // the episode is over either way
    static const bool kAmnestyOn = [] {
        const char* e = std::getenv("ULTRA_TROUGH_AMNESTY");
        return !(e && e[0] == '0');  // DEFAULT-ON 2026-07-05 (inert under authority)
    }();
    if (!kAmnestyOn) return;
    if (rung_cmd != ultra::waveform::tone_burst_ack::kRungCmdNone) return;
    if (state_ != ConnectionState::CONNECTED) return;
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) return;
    if (!rateAdaptationActive() || mode_change_pending_) return;
    if (file_transfer_.getState() != FileTransferState::SENDING &&
        hasGeometryBoundDataOperation()) {
        // Amnesty is an opportunistic up-move. A message/binary object owns one
        // geometry for its complete lifetime; never fail it merely to restore a
        // pre-trough rung. The next transfer can re-evaluate normally.
        return;
    }
    if (coherentRungOrdinal(data_modulation_, data_code_rate_) >=
        coherentRungOrdinal(pre_episode_mod_, pre_episode_rate_)) {
        return;  // nothing was lost to the trough
    }
    LOG_MODEM(WARN,
              "Connection: TROUGH-AMNESTY restore %s %s -> %s %s (null ended; "
              "trough demotes are not rate evidence)",
              modulationToString(data_modulation_), codeRateToString(data_code_rate_),
              modulationToString(pre_episode_mod_), codeRateToString(pre_episode_rate_));
    // Same commit envelope as maybeApplyRxRateCommand (we are inside the ack's
    // defer-refill bracket): descriptor commit when era-safe, legacy exchange fallback.
    bool desc_committed = false;
    const bool busy = hasGeometryBoundDataOperation();
    if (!busy || arq_.moveEpochEnabled()) {
        desc_committed = tryDescriptorModeSwitch(
            pre_episode_mod_, pre_episode_rate_, wireSnrDb(),
            v2::ModeChangeReason::CHANNEL_IMPROVED);
    }
    if (!desc_committed) {
        requestModeChange(pre_episode_mod_, pre_episode_rate_, wireSnrDb(),
                          v2::ModeChangeReason::CHANNEL_IMPROVED);
    }
}

void Connection::noteArqRoundOutcome(int progress_frames, const char* origin) {
    if (progress_frames > 0) {
        if (retx_pace_hold_ms_ > 0) {
            LOG_MODEM(INFO,
                      "Connection: TROUGH-PACING early release (%d frames progressed, %s)",
                      progress_frames, origin);
        }
        zero_progress_rounds_ = 0;
        retx_pace_hold_ms_ = 0;
        consecutive_escape_drops_ = 0;  // channel proved alive — end the silent escape episode
        // GROUP-SIZE LEVER: a CLEAN round = full retire, nothing left in flight
        // (an ack with holes leaves retransmits in flight). Two in a row unlock
        // the escalated 11.5 s ceiling (burstAirtimeBudgetFrames); anything
        // less resets to the 8.6 s base.
        if (arq_.getTxInFlightBytes() == 0) {
            ++burst_clean_group_streak_;
            if (burst_clean_group_streak_ == 2) {
                LOG_MODEM(INFO,
                          "Connection: GROUP-SIZE streak proven (2 clean rounds) — "
                          "11.5 s ceiling eligibility now depends on the active-rung/"
                          "ULTRA_BURST_ESC_STREAK gate");
            }
        } else {
            burst_clean_group_streak_ = 0;  // holey round: keep short key-downs
        }
        return;
    }
    if (progress_frames < 0) {
        return;  // duplicate/stale ack — never a phantom round (§1.1 dedup)
    }
    burst_clean_group_streak_ = 0;  // zero-progress round: back to the base ceiling
    if (!retxPacingScopeActive()) {
        // Out of scope (MC-DPSK / OFDM_NARROW / no file in flight): never accumulate
        // rounds or holds here — their timers must never see this machinery (§6.2).
        zero_progress_rounds_ = 0;
        retx_pace_hold_ms_ = 0;
        return;
    }

    // RX-AUTHORITY double-driver fix (F128): under receiver authority, a RECEIVED
    // ack is the receiver's living verdict — a zero-progress dup-ack during crater
    // recovery is the receiver SPEAKING ("resend those"), not evidence it can't
    // reach us. Counting it toward the collapse escape let the sender unilaterally
    // demote mid-recovery (F128 t=151: ESCAPE-drop fired on '2 zero-progress
    // rounds' that were both decoded dup-acks — while the receiver's verdict was
    // simultaneously commanding UP; the two drivers fought and the ladder sawed).
    // The escape's charter is ack-SILENCE self-rescue: only RTO rounds (no ack
    // decoded at all) count while authority is on. Without authority the old
    // accounting stands (the sender is the only driver there).
    if (rxRateAuthorityEnabled() && std::strcmp(origin, "rto") != 0) {
        return;
    }

    ++zero_progress_rounds_;
    // TROUGH AMNESTY: snapshot the rung active as the episode BEGINS. Runs after
    // maybeApplyRxRateCommand in the ack path, so a crater rung-command landing on
    // this same ack has already applied — the snapshot is the post-crater rung
    // (legitimate receiver evidence is kept; only trough-driven drops are amnestied).
    if (zero_progress_rounds_ == 1) {
        trough_episode_active_ = true;
        pre_episode_mod_ = data_modulation_;
        pre_episode_rate_ = data_code_rate_;
    }
    LOG_MODEM(INFO, "Connection: zero-progress ARQ round %d (%s)",
              zero_progress_rounds_, origin);
    // The §2 collapse escape is POLLED from the CONNECTED tick (maybeCollapseEscape), not
    // fired here — this function runs inside ack processing AND inside the ARQ transmit
    // callback (RTO batch), and requestModeChange must not re-enter the ARQ from the latter.
    if (!retxTroughPacingEnabled()) {
        return;  // §5.1 master knob OFF (default) — counting above is inert bookkeeping
    }
    if (mode_change_pending_) {
        return;  // a rate change is already re-anchoring the era; don't stack a hold on it
    }
    const uint32_t elapsed_ms = elapsedSinceLastDataBurstEndMs();
    const float doppler_hz = coherence_valid_ ? coherence_doppler_hz_ : 0.0f;
    const uint32_t hold_ms = connection_policy::retxTroughDeferMs(
        doppler_hz, fading_index_, coherence_score_, coherence_valid_,
        zero_progress_rounds_, elapsed_ms, troughDeferTcFrac());
    if (hold_ms == 0) {
        return;  // e.g. the ~18 s RTO path already out-waited Tc (§1.2) — add nothing
    }
    retx_pace_hold_ms_ = hold_ms;                 // trigger #1: turn refill (runDeferredArqRefill)
    arq_.deferPendingRetransmits(hold_ms);        // trigger #2: per-slot RTO (one state, both)
    const float tc_s = static_cast<float>(connection_policy::coherenceTimeMsForDoppler(
        connection_policy::retxTroughDopplerHz(doppler_hz, fading_index_,
                                               coherence_score_, coherence_valid_))) /
        1000.0f;
    LOG_MODEM(WARN,
              "Connection: TROUGH-PACING defer %ums (round %d, Tc=%.2fs, elapsed=%ums, %s)",
              hold_ms, zero_progress_rounds_, tc_s, elapsed_ms, origin);
    // Label the pause for the operator (2 AM waterfall rule, §4): a visible "Adapt:" text,
    // not a silent hang. Also the A/B grep hook.
    char buf[96];
    std::snprintf(buf, sizeof(buf), "pace-hold %ums (zero-progress round %d)",
                  hold_ms, zero_progress_rounds_);
    last_adaptive_action_ = buf;
}

void Connection::applyAdaptiveRateFeedback(float quality) {
    // §14.36 Phase 5c: the SENDER runs the rate controller on the receiver's
    // decode-headroom feedback (carried on the GROUP_ACK; a GROUP_NACK feeds 0).
    // It owns the rate, so data_code_rate_ is the rate the just-acked group used.
    if (!adaptive_rate_enabled_ || !use_burst_transport_) {
        return;
    }
    if (quality < 0.0f) {
        return;  // no feedback byte (old peer / adaptation off on the far end)
    }
    // GUI "Adapt:" headroom bar reads lastGroupQuality(). Record EVERY valid sample here —
    // before the lock-rate / rate-change branches below each return — or the bar stays at the
    // init -1.0 forever ("waiting for first group..."). The unification dropped this assignment
    // (only last_adaptive_action_ text was kept), so the bar never populated mid-transfer.
    // NOTE: on the unified tone-burst path `quality` is currently the binary ack(1.0)/nack(0.0)
    // signal; a graded decode-headroom would need the tone-burst rate_hint field wired through
    // (TODO) — but binary already drives the green/red bar instead of "waiting" all run.
    last_group_quality_ = quality;
    // Rate ADAPTATION is DEFAULT-ON for connected wideband OFDM (2026-07-02, fade-riding
    // ladder; was default-OFF 2026-06-07..07-01). The §14.43 closed-loop quality feedback is
    // wired end-to-end (receiver LDPC headroom -> rate_hint -> here). Opt out with
    // ULTRA_RATE_ADAPT=0 or pin with ULTRA_LOCK_RATE=1 — then the graded quality still drives
    // the GUI "Adapt:" bar (last_group_quality_, set above) + the action text, so we can SEE
    // what the controller WOULD do without it moving the rate.
    if (!rateAdaptationActive()) {
        const bool rate_locked = [] {
            const char* l = std::getenv("ULTRA_LOCK_RATE");
            return l != nullptr && std::atoi(l) != 0;
        }();
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s %s (q=%.2f)",
                      rate_locked ? "lock" : "off", codeRateToString(data_code_rate_), quality);
        last_adaptive_action_ = buf;
        return;
    }

    // ───────── QAM16 R2/3 cross-modulation climb (ULTRA_QAM16_CLIMB, default-ON) ─────────
    // Default-ON since 2026-07-02 (fade-riding ladder): above QPSK R3/4 the next throughput
    // step is the QAM16 R2/3 modulation hop, and riding the Good fade crests requires taking
    // it. "0" opts out.
    const bool qam16_climb_enabled = [] {
        const char* e = std::getenv("ULTRA_QAM16_CLIMB");
        if (e == nullptr || e[0] == '\0') return true;
        return std::atoi(e) != 0;
    }();
    if (data_modulation_ == Modulation::QAM16 ||
        data_modulation_ == Modulation::QAM8) {
        // While on a DENSE-constellation gear we do NOT walk the QPSK code-rate ladder
        // (the RateController is QPSK-blind — its R2/3 index is a QPSK rung). QAM16 is
        // fragile on frequency-selective fading (the decodability cliff: 55-70% loss /
        // link-death — fable_07); QAM8's 45° boundaries fail the same way, just later
        // (+3.6 dB margin). 8PSK revival probe (2026-07-05): letting the EMA walk QAM8
        // down its own rate ladder slid R2/3 -> R1/2 -> R1/4 (a strictly-dominated
        // rung: tight boundaries AND low rate) and finished at 990 vs the pinned-rate
        // 2040 — dense mods need the EXIT semantics, not the walk. So: HOLD or a
        // prompt asymmetric demote to the robust QPSK R3/4 home gear (QAM16's
        // re-climb cooldown machinery applies to QAM16 only; QAM8 has no mid-stream
        // climb-in yet — it is entry-selected — so no cooldown to meter).
        // ULTRA_QAM16_R34 crest semantics generalize: a bad group at the mod's R3/4
        // rung steps down ONE rung to its validated R2/3 and STAYS on the modulation;
        // a further bad group takes the QPSK exit.
        // RAW quality (not the EMA): a cliff demands a prompt reaction.
        const bool is_qam16 = data_modulation_ == Modulation::QAM16;
        const float drop_below = rate_controller_.config().drop_below;
        const bool nack = quality <= 0.0f;  // group fully lost — the cliff signature
        if (quality < drop_below) ++qam16_bad_streak_; else qam16_bad_streak_ = 0;
        char buf[96];
        if (nack || qam16_bad_streak_ >= kQam16DemoteBadStreak) {
            qam16_bad_streak_ = 0;
            qam16_r34_clean_streak_ = 0;  // a bad group also aborts any pending R3/4 walk
            // Crest-rung exit is one step at a time: R3/4 demotes to the validated R2/3, and
            // STAYS on the dense mod — noteQam16Demoted meters the QPSK->QAM16 re-entry
            // cooldown, which this is not. A further bad group at R2/3 exits to QPSK below.
            // (QAM8 R3/4 is a ladder rung when the psk8 ladder is on — same one-step-down.)
            const bool r34_step_down =
                (is_qam16 ? qam16R34Enabled() : true) &&
                data_code_rate_ == CodeRate::R3_4;
            const bool busy = hasGeometryBoundDataOperation();
            if (busy) {
                std::snprintf(buf, sizeof(buf),
                              "hold %s %s (demote->%s at clean boundary, q=%.2f)",
                              modulationToString(data_modulation_),
                              codeRateToString(data_code_rate_),
                              r34_step_down ? "R2/3 (same mod)" : "QPSK R3/4", quality);
            } else if (r34_step_down) {
                const Modulation step_mod = data_modulation_;  // stays on the dense mod
                const bool desc_committed = tryDescriptorModeSwitch(
                    step_mod, CodeRate::R2_3, wireSnrDb(),
                    v2::ModeChangeReason::CHANNEL_DEGRADED);
                if (!desc_committed) {
                    requestModeChange(step_mod, CodeRate::R2_3, wireSnrDb(),
                                      v2::ModeChangeReason::CHANNEL_DEGRADED);
                }
                std::snprintf(buf, sizeof(buf),
                              "%s R3/4 -> R2/3 demote via %s (q=%.2f)",
                              modulationToString(step_mod),
                              desc_committed ? "DESC-SWITCH" : "MODE_CHANGE", quality);
                LOG_MODEM(INFO, "Connection: adaptive %s", buf);
            } else {
                // Count the demote + arm the re-climb cooldown only when the MODE_CHANGE actually
                // fires (a busy-held decision re-asserts on the next bad group; counting the hold
                // would double-charge the backoff for one logical demote). QAM16 only: QAM8 has
                // no mid-stream climb-in, so there is no re-entry cooldown to meter.
                if (is_qam16) noteQam16Demoted(1);
                // Old-mode strings captured BEFORE the commit: a descriptor commit applies
                // the new mode immediately (data_code_rate_ mutates), unlike the pending
                // MODE_CHANGE path which holds it until the ACK.
                const char* old_mod_str = modulationToString(data_modulation_);
                const char* old_rate_str = codeRateToString(data_code_rate_);
                const bool desc_committed = tryDescriptorModeSwitch(
                    Modulation::QPSK, CodeRate::R3_4, wireSnrDb(),
                    v2::ModeChangeReason::CHANNEL_DEGRADED);
                if (!desc_committed) {
                    requestModeChange(Modulation::QPSK, CodeRate::R3_4, wireSnrDb(),
                                      v2::ModeChangeReason::CHANNEL_DEGRADED);
                }
                std::snprintf(buf, sizeof(buf),
                              "%s %s -> QPSK R3/4 demote via %s (q=%.2f)",
                              old_mod_str, old_rate_str,
                              desc_committed ? "DESC-SWITCH" : "MODE_CHANGE", quality);
                LOG_MODEM(INFO, "Connection: adaptive %s", buf);
            }
        } else if (is_qam16 && qam16R34Enabled() && data_code_rate_ == CodeRate::R2_3) {
            // Crest-rung walk: QAM16 R2/3 -> R3/4 after qam16ClimbStreak() CONSECUTIVE clean
            // groups (quality >= climb_above; a sub-threshold group resets the streak — same
            // gate as the QPSK->QAM16 hop). Fires only at a clean send boundary; when the
            // window is busy the streak is KEPT so the walk re-asserts on a later
            // clean-boundary ack, mirroring the modulation hop's deferred re-assert.
            const float climb_above = rate_controller_.config().climb_above;
            if (quality >= climb_above) ++qam16_r34_clean_streak_;
            else qam16_r34_clean_streak_ = 0;
            // FAST-CREST (2026-07-04, F9 finding, knob ULTRA_R34_FAST_CREST default-OFF):
            // the 2-group streak TRAILS the crest — F9's hop confirmed at fading 0.17
            // but fired at 0.51 (the window had closed; the excursion caught the tail,
            // 4/9 + 0/9, ~40 s lost). With descriptor commits + the ~4 s receiver-command
            // demote, a wrong hop is now cheap — so when the receiver's quality hint
            // SATURATES (>= 0.99 = the 3-bit quantizer's top bin: decode headroom far
            // beyond the R2/3 requirement), ONE such group arms the walk. Exit speed
            // funds entry speed.
            static const bool kFastCrest = [] {
                const char* e = std::getenv("ULTRA_R34_FAST_CREST");
                return !(e && e[0] == '0');  // DEFAULT-ON 2026-07-05
            }();
            const int walk_streak_needed =
                (kFastCrest && quality >= 0.99f) ? 1 : qam16ClimbStreak();
            // CALM-GATE (2026-07-04, F3-replication + crest A/B, knob
            // ULTRA_R34_CALM_FADING default-OFF byte-identical): the R3/4 walk fired
            // on `quality` alone — a BACKWARD-looking signal (the group that just
            // decoded had headroom). But R3/4 must survive the NEXT ~9 s, which is a
            // channel-COHERENCE property, not a decode-headroom one. In a chop
            // realization a clean group is routinely followed by a crater, so blind
            // probing pays 1 cratered group + a re-climb per probe (afternoon A/B:
            // crest median 1.25 vs crest-OFF 1.43/1.62 in the SAME chop; F3's 2.50
            // was a zero-crater 16QAM cruise that NEVER probed). Gate the walk on the
            // coherence-adjusted fading index: probe only in AWGN/low-Doppler calm —
            // exactly F3's cruise condition — otherwise stay at 16QAM R2/3 and cruise.
            // Set the knob to a threshold (e.g. 0.30) to enable; absent = legacy.
            static const float kR34CalmFading = [] {
                const char* e = std::getenv("ULTRA_R34_CALM_FADING");
                return (e && e[0]) ? std::strtof(e, nullptr) : 0.30f;  // DEFAULT 0.30 2026-07-05; <0 = off
            }();
            const bool calm_gate_ok =
                kR34CalmFading < 0.0f ||
                connection_policy::coherenceAdjustedFadingIndex(
                    fading_index_, coherence_score_, coherence_valid_) <= kR34CalmFading;
            if (calm_gate_ok && qam16_r34_clean_streak_ >= walk_streak_needed) {
                const bool busy = hasGeometryBoundDataOperation();
                if (busy) {
                    std::snprintf(buf, sizeof(buf),
                                  "hold QAM16 R2/3 for clean boundary (want QAM16 R3/4, q=%.2f)",
                                  quality);
                } else {
                    qam16_r34_clean_streak_ = 0;  // walk FIRED at a clean boundary -> reset
                    const bool desc_committed = tryDescriptorModeSwitch(
                        Modulation::QAM16, CodeRate::R3_4, wireSnrDb(),
                        v2::ModeChangeReason::CHANNEL_IMPROVED);
                    if (!desc_committed) {
                        requestModeChange(Modulation::QAM16, CodeRate::R3_4, wireSnrDb(),
                                          v2::ModeChangeReason::CHANNEL_IMPROVED);
                    }
                    std::snprintf(buf, sizeof(buf),
                                  "QAM16 R2/3 -> QAM16 R3/4 climb via %s (q=%.2f)",
                                  desc_committed ? "DESC-SWITCH" : "MODE_CHANGE", quality);
                    LOG_MODEM(INFO, "Connection: adaptive %s", buf);
                }
            } else {
                std::snprintf(buf, sizeof(buf), "hold QAM16 R2/3 (q=%.2f)", quality);
            }
        } else if (!is_qam16 && psk8LadderEnabled() &&
                   data_code_rate_ == CodeRate::R2_3) {
            // QAM8 R2/3 -> QAM16 R2/3 modulation crest-step (2026-07-05, F121 finding:
            // QAM8 was entry-only — no upward walk existed). Same gates as the QAM16
            // R3/4 crest walk above: qam16ClimbStreak() CONSECUTIVE clean groups
            // (>= climb_above; one sub-threshold group resets), fire only at a clean
            // send boundary, streak KEPT on a busy hold so the step re-asserts.
            // Reuses qam16_r34_clean_streak_ (identical lifecycle, mutually exclusive
            // with the QAM16 walk via the is_qam16 arm; rename tracked in the cleanup
            // register). +1 bit/symbol at -3.6 dB margin — strictly a crest move, the
            // streak is the calm gate.
            const float climb_above = rate_controller_.config().climb_above;
            if (quality >= climb_above) ++qam16_r34_clean_streak_;
            else qam16_r34_clean_streak_ = 0;
            if (qam16_r34_clean_streak_ >= qam16ClimbStreak() &&
                qam16_reclimb_cooldown_ == 0) {
                const bool busy = hasGeometryBoundDataOperation();
                if (busy) {
                    std::snprintf(buf, sizeof(buf),
                                  "hold QAM8 R2/3 for clean boundary (want QAM16 R2/3, q=%.2f)",
                                  quality);
                } else {
                    qam16_r34_clean_streak_ = 0;  // step FIRED at a clean boundary -> reset
                    const bool desc_committed = tryDescriptorModeSwitch(
                        Modulation::QAM16, CodeRate::R2_3, wireSnrDb(),
                        v2::ModeChangeReason::CHANNEL_IMPROVED);
                    if (!desc_committed) {
                        requestModeChange(Modulation::QAM16, CodeRate::R2_3, wireSnrDb(),
                                          v2::ModeChangeReason::CHANNEL_IMPROVED);
                    }
                    std::snprintf(buf, sizeof(buf),
                                  "QAM8 R2/3 -> QAM16 R2/3 climb via %s (q=%.2f)",
                                  desc_committed ? "DESC-SWITCH" : "MODE_CHANGE", quality);
                    LOG_MODEM(INFO, "Connection: adaptive %s", buf);
                }
            } else {
                std::snprintf(buf, sizeof(buf), "hold QAM8 R2/3 (q=%.2f)", quality);
            }
        } else {
            std::snprintf(buf, sizeof(buf), "hold %s %s (q=%.2f)",
                          modulationToString(data_modulation_),
                          codeRateToString(data_code_rate_), quality);
        }
        last_adaptive_action_ = buf;
        return;
    }

    const CodeRate prev = data_code_rate_;
    CodeRate next = rate_controller_.update(prev, quality);
    // 2026-05-28 experiment: ULTRA_MAX_OFDM_RATE caps climbs at a chosen rung.
    if (const char* env = std::getenv("ULTRA_MAX_OFDM_RATE")) {
        const std::string s(env);
        const CodeRate cap = (s == "R1_2" || s == "r1_2") ? CodeRate::R1_2
                           : (s == "R2_3" || s == "r2_3") ? CodeRate::R2_3
                           : (s == "R3_4" || s == "r3_4") ? CodeRate::R3_4
                           : CodeRate::AUTO;  // anything else = no cap (AUTO sentinel)
        if (cap != CodeRate::AUTO && next > cap) {
            next = cap;
        }
    }
    // Per-modulation validated-rate cap (2026-06-12, Phase 1). The RateController is
    // modulation-blind: it would climb the connect-time 16QAM R1/2 up into the measured
    // damage-bound 16QAM R2/3/R3/4 (Phase 0a) before the REACTIVE ssthresh can cap it —
    // taking a frame into a fade at the over-climbed rung (the seed-7 R3/4 FAIL mode).
    // Cap the CLIMB at the highest GUI-validated rate for the active modulation. Drops
    // (escape-drop / ssthresh) go DOWN and are unaffected. No-op for QPSK (cap = R3/4, ladder top).
    const CodeRate mod_cap = maxValidatedCoherentRate(data_modulation_);
    if (next > mod_cap) {
        next = mod_cap;
    }

    // QPSK R3/4 -> QAM16 R2/3 cross-modulation CLIMB. Above the QPSK top rung (R3/4) the next
    // throughput step is a MODULATION step, not a thinner code (R5/6 retired — a measured loser).
    // Climb ONLY after qam16ClimbStreak() CONSECUTIVE clean groups (quality >= climb_above) while
    // pinned at QPSK R3/4: a single sub-threshold group breaks the streak, so the streak doubles
    // as a low-variance Good-vs-Moderate gate (a Moderate channel's fades keep resetting it) — the
    // sender-side proxy for "confirmed-Good" that needs no wire change. The LTS coherence disc (the
    // sharper gate) lives on the RECEIVER, which is deaf to its own channel while sending, and its
    // HW threshold is not yet validated, so disc-gating is a deliberate later add. After a demote,
    // the re-climb COOLDOWN (noteQam16Demoted) must first be spent in CLEAN groups before the
    // streak may accrue again — the fade-riding replacement for the 06-17 sticky no-reclimb.
    Modulation target_mod = data_modulation_;
    CodeRate target_rate = next;
    bool qam16_hop = false;
    // LADDER-AWARE dense climb (2026-07-05, F121 finding): with the 8PSK ladder on,
    // the first dense rung above QPSK R3/4 is QAM8 R2/3 (constant-envelope, ~3.6 dB
    // more constellation margin than 16QAM). Without this the hop was hardcoded
    // QPSK->QAM16, making QAM8 ENTRY-ONLY: any exit mid-transfer stranded the run on
    // the QPSK<->QAM16 loop (F121 never revisited 8PSK after an 18 dB connect
    // snapshot). The streak / cooldown / calm-gate machinery below is unchanged —
    // only the hop TARGET generalizes. The QAM8 R2/3 -> QAM16 R2/3 upward step lives
    // in the dense-constellation branch above (this code is unreachable while on a
    // dense mod — that branch returns). (The qam16_* names now cover the generic
    // dense-climb machinery; rename tracked in the cleanup register.)
    const Modulation climb_to_mod =
        psk8LadderEnabled() ? Modulation::QAM8 : Modulation::QAM16;
    if (qam16_climb_enabled &&
        data_modulation_ == Modulation::QPSK && prev == CodeRate::R3_4 &&
        next == CodeRate::R3_4) {
        const float climb_above = rate_controller_.config().climb_above;
        if (quality >= climb_above) {
            if (qam16_reclimb_cooldown_ > 0) {
                --qam16_reclimb_cooldown_;  // a clean group spends cooldown; streak stays 0
                qam16_clean_streak_ = 0;
            } else {
                ++qam16_clean_streak_;
            }
        } else {
            qam16_clean_streak_ = 0;
        }
        // 16QAM CALM-GATE (2026-07-04, crater-frequency lever, knob
        // ULTRA_QAM16_CALM_FADING default-OFF byte-identical): the climb fires on
        // backward-looking `quality`; 16QAM R2/3 sits at ZERO fade-headroom, so in a
        // choppy realization it climbs into a trough and craters (afternoon 8/29 vs
        // F3's calm 1/18). Now that crater RECOVERY is fast (self-echo fix, ~3.5 s),
        // the residual tax is crater FREQUENCY. Gate the hop on the coherence-adjusted
        // fading index — climb to 16QAM only when the channel is calm (fade-riding:
        // 16QAM on crests), else hold the robust QPSK R3/4 and cruise. The streak is
        // preserved when the gate blocks (same deferral semantics as a busy window),
        // so the hop re-asserts the moment calm returns. Mirror of the R3/4 calm-gate.
        static const float kQam16CalmFading = [] {
            const char* e = std::getenv("ULTRA_QAM16_CALM_FADING");
            return (e && e[0]) ? std::strtof(e, nullptr) : -1.0f;  // <0 = gate off
        }();
        const bool qam16_calm_ok =
            kQam16CalmFading < 0.0f ||
            connection_policy::coherenceAdjustedFadingIndex(
                fading_index_, coherence_score_, coherence_valid_) <= kQam16CalmFading;
        if (qam16_clean_streak_ >= qam16ClimbStreak() && qam16_calm_ok) {
            target_mod = climb_to_mod;
            target_rate = CodeRate::R2_3;
            qam16_hop = true;
            // NOTE: do NOT reset the streak here. If the change is DEFERRED below (busy send
            // window — the common case mid-transfer), keeping the streak >= qam16ClimbStreak() lets
            // the hop RE-ASSERT on the next clean-boundary ack (mirroring how the QPSK rate change
            // re-asserts via rate_controller_.update). The streak resets only when the hop actually
            // FIRES, or when a sub-climb_above group breaks it (the channel degraded — abort).
        }
    } else {
        qam16_clean_streak_ = 0;  // not pinned at the QPSK top rung / disabled -> reset the streak
    }

    char buf[96];
    const bool changed = (target_mod != data_modulation_) || (target_rate != prev);
    if (changed) {
        // OPTION A (2026-06-10): a mid-FILE rate/mod change must land at a CLEAN send boundary —
        // one with no in-flight/pending file chunks. applyDataMode() re-encodes pending chunks
        // via requeuePendingChunks(), which REWINDS the file send cursor by the whole in-flight
        // window and re-sends already-delivered data. On a clean channel that's just wasted
        // airtime (survivable), but coincident with a fade the redundant re-sends starve the
        // real remaining chunks and the transfer strands (~75%, Good@20 seeds 2/99, 2026-06-09).
        // requestModeChange() holds the rate until BRAVO ACKs and runDeferredArqRefill() is gated
        // on mode_change_pending_, so NO new chunks submit between issue and apply — a change
        // ISSUED at a clean boundary is also APPLIED at one (the window stays drained until the
        // ACK -> requeuePendingChunks() is a no-op -> nothing re-sent). When the window is busy,
        // HOLD: applyAdaptiveRateFeedback runs on every group ack, and the EMA controller
        // re-asserts the decision on a later ack that lands at a clean boundary (a full-ack tick).
        // This also correctly THROTTLES rate churn during a fade (partial acks keep us busy).
        const bool geometry_owner_busy = hasGeometryBoundDataOperation();
        if (geometry_owner_busy) {
            std::snprintf(buf, sizeof(buf), "hold %s %s for clean boundary (want %s %s, q=%.2f)",
                          modulationToString(data_modulation_), codeRateToString(prev),
                          modulationToString(target_mod), codeRateToString(target_rate), quality);
        } else {
            // Route the adaptive rate/mod change through the SYNCHRONIZED MODE_CHANGE handshake:
            // requestModeChange() holds the local rate until BRAVO ACKs, so both stations switch
            // TOGETHER and re-anchor cleanly. The former unilateral `data_code_rate_ = next` flip
            // (in-band BURST_HEADER descriptor) desynced the pilot/carrier geometry between sender
            // and receiver — stale warm-sync -> |H| garbage -> 0/8 CWs forever -> churn to R1/4
            // (2026-06-09). The EMA RateController owns WHEN to change; the MODE_CHANGE owns HOW.
            const uint8_t reason =
                isFasterMode(target_mod, target_rate, data_modulation_, prev)
                    ? v2::ModeChangeReason::CHANNEL_IMPROVED
                    : v2::ModeChangeReason::CHANNEL_DEGRADED;
            // Old-mod string captured BEFORE the commit: a descriptor commit applies the
            // new mode immediately (data_modulation_ mutates), unlike the pending
            // MODE_CHANGE path which holds it until the ACK.
            const char* old_mod_str = modulationToString(data_modulation_);
            // DESC-SWITCH Phase 1 (knob-ON): skip the MODE_CHANGE round-trip — commit
            // locally and let the next burst's BURST_HEADER descriptor announce the move
            // (§5.1; the pilot/carrier-geometry desync arm of the 2026-06-09 failure is
            // closed by the mandatory full-anchor one-shot + RX warm-handoff demotion).
            // Falls back to the synchronized exchange when out of scope.
            const bool desc_committed =
                tryDescriptorModeSwitch(target_mod, target_rate, wireSnrDb(), reason);
            if (!desc_committed) {
                requestModeChange(target_mod, target_rate, wireSnrDb(), reason);
            }
            if (qam16_hop) qam16_clean_streak_ = 0;  // hop FIRED at a clean boundary -> reset
            std::snprintf(buf, sizeof(buf), "%s %s -> %s %s via %s (q=%.2f)",
                          old_mod_str, codeRateToString(prev),
                          modulationToString(target_mod), codeRateToString(target_rate),
                          desc_committed ? "DESC-SWITCH" : "MODE_CHANGE", quality);
            LOG_MODEM(INFO, "Connection: adaptive %s", buf);
        }
    } else {
        std::snprintf(buf, sizeof(buf), "hold %s (q=%.2f)", codeRateToString(prev), quality);
    }
    last_adaptive_action_ = buf;
}

void Connection::setRxLevelVerdict(int verdict, uint32_t seq) {
    if (seq == rx_level_verdict_seq_seen_) {
        return;  // stale re-feed — no fresh per-burst measurement since last time
    }
    rx_level_verdict_seq_seen_ = seq;
    rx_level_verdict_pending_for_group_ = true;
    using connection_policy::RxLevelVerdict;
    switch (static_cast<RxLevelVerdict>(verdict)) {
        case RxLevelVerdict::CLIPPED:
            rx_level_clipped_ = true;
            rx_level_low_streak_ = 0;
            break;
        case RxLevelVerdict::LOW:
            rx_level_clipped_ = false;
            ++rx_level_low_streak_;
            break;
        default:
            rx_level_clipped_ = false;
            rx_level_low_streak_ = 0;
            break;
    }
}

// Responder handshake confirmation (BUG-RESPONDER-HANDSHAKE-NEVER-CONFIRMS,
// 2026-07-04): the single site that flips a responder's handshake_confirmed_ and
// fires on_handshake_confirmed_() (→ the modem switches TX onto the negotiated OFDM
// data waveform instead of the handshake last-RX-waveform mirror). Historically it
// ran only in onFrameReceived — the CLASSIC frame path — because in the legacy
// control plane the initiator's first MODE_CHANGE frame always arrived there. The
// descriptor-committed control plane (Phase 1/2) eliminated classic frames BY
// DESIGN, so a burst-only session never confirmed: the responder's modem sat in
// handshake TX-routing all session and its rare classic control TXs (frame NACKs)
// went out as 3.1 s MC-DPSK DBPSK full-preamble frames (last_rx_waveform_ = the
// CONNECT-phase MC-DPSK) — the "MC-DPSK at the end of the run" the operator saw.
// A CRC-valid post-CONNECT frame addressed to us and sourced by the established
// peer is equally hard evidence the initiator heard our CONNECT_ACK. A PHY sync or
// an empty/all-failed group is not: the IONOS rig accepted a duplicate MC-DPSK
// CONNECT as weak OFDM sync and later delivered 0/6, while the initiator was still
// waiting for the ACK.
bool Connection::isAuthoritativeResponderHandshakeFrame(
    const v2::HeaderInfo& header) const {
    if (!header.valid || local_call_.empty() || header.src_hash == 0 ||
        header.dst_hash != v2::hashCallsign(local_call_)) {
        return false;
    }

    // Handshake/probe traffic is not evidence that the initiator advanced into
    // the negotiated session. In particular, duplicate CONNECT proves the
    // opposite and must retain the cached ACK for the reactive replay path.
    switch (header.type) {
        case v2::FrameType::CONNECT:
        case v2::FrameType::CONNECT_ACK:
        case v2::FrameType::CONNECT_NAK:
        case v2::FrameType::PROBE:
        case v2::FrameType::PROBE_ACK:
            return false;
        default:
            break;
    }

    const uint32_t remote_call_hash =
        remote_call_.empty() ? 0 : v2::hashCallsign(remote_call_);
    const uint32_t pending_call_hash =
        pending_remote_call_.empty() ? 0 : v2::hashCallsign(pending_remote_call_);
    return (remote_hash_ != 0 && header.src_hash == remote_hash_) ||
           (pending_remote_hash_ != 0 && header.src_hash == pending_remote_hash_) ||
           (remote_call_hash != 0 && header.src_hash == remote_call_hash) ||
           (pending_call_hash != 0 && header.src_hash == pending_call_hash);
}

void Connection::maybeConfirmResponderHandshake(const char* evidence) {
    if (state_ != ConnectionState::CONNECTED || is_initiator_ || handshake_confirmed_) {
        return;
    }
    LOG_MODEM(INFO, "Connection: Handshake confirmed (%s)", evidence);
    // Authoritative peer traffic makes cached reactive CONNECT_ACK recovery
    // obsolete. Clear it before the callback so a re-entrant application cannot
    // observe or replay stale handshake bytes.
    connect_ack_frame_.clear();
    handshake_confirmed_ = true;
    responder_handshake_wait_ms_ = 0;
    if (on_handshake_confirmed_) {
        on_handshake_confirmed_();
    }
    // Initial data mode is already carried in CONNECT_ACK.
}

void Connection::onBurstGroupReceived(uint16_t group_seq, const std::vector<Bytes>& frames,
                                      bool all_ok, float quality, uint16_t frame_mask,
                                      bool interleaved, uint8_t group_size,
                                      bool geometry_proven) {
    if (disconnect_teardown_active_ || !use_burst_transport_) {
        return;
    }
    // TRANSPORT MERGE (increment 1): one seq space END-TO-END. The sender formed these
    // frames through arq_ (unified TX), so feed each decoded REAL frame back through the
    // ARQ window (processArqFrame) instead of the burst-group delivery + group_seq ack.
    // arq_ dedups/reorders by the frames' own seqs, delivers in order, and emits the
    // tone-burst ack itself — which the sender's arq_ consumes (seqs match), driving
    // selective repeat over the unified space. Skips the burst controller entirely.
    if (kUnifiedSeqEnabled()) {
        (void)all_ok; (void)frame_mask; (void)interleaved; (void)group_seq;
        // §14.43 closed-loop rate feedback + RECEIVER GUI "Adapt:" bar. THIS callback is where the
        // graded per-group decode headroom [0,1] is MEASURED (streaming_burst_interleave:
        // 1 - worst_CW_LDPC_iters/80). Record it so (a) BRAVO's Adapt bar shows it — the sender's
        // bar is fed via applyAdaptiveRateFeedback(), which the RECEIVER never runs — and (b) the
        // next tone-burst ack carries it back to ALPHA in rate_hint (the loop the unification cut).
        if (quality >= 0.0f) {
            last_group_quality_ = quality;
        }
        // RX-RATE-CMD Phase 2 (ULTRA_RX_RATE_CMD): refresh the receiver's standing rung
        // command BEFORE the ARQ emits this group's tone-burst ACK (endGroupReceiveAndAck
        // below) so the verdict rides THIS ACK — the crater is visible to the sender one
        // whole escape-detection cycle (~2×RTO) earlier than its own zero-progress
        // evidence. Hard no-op while the knob is OFF.
        if (rx_rate_cmd_enabled_) {
            updateRxRateCommandFromGroup(all_ok, frame_mask);
        }
        // RX-AUTHORITY (ULTRA_RX_RATE_AUTHORITY): compute the receiver's ABSOLUTE
        // rung verdict for this group BEFORE its ACK emits — the command rides THIS
        // ACK. Hard no-op while the knob is OFF.
        if (rxRateAuthorityEnabled()) {
            // Delivered fraction for the goodput-graded crater predicate: frame_mask
            // bits are the DELIVERED frames of this group (the full-crater arm below
            // reads mask==0 as zero delivered), so popcount/group_size is the measured
            // per-group delivery. Only meaningful with a known group size — pass -1
            // otherwise so the predicate keeps its legacy binary form.
            const uint8_t delivered_frames = static_cast<uint8_t>(
                std::min<unsigned>(std::popcount(frame_mask), group_size));
            const float delivered_fraction =
                (group_size > 0)
                    ? static_cast<float>(delivered_frames) / static_cast<float>(group_size)
                    : -1.0f;
            updateRxAuthorityCommand(all_ok, quality, /*full_crater=*/!all_ok && frame_mask == 0,
                                     delivered_fraction, delivered_frames, group_size,
                                     geometry_proven);
        }
        // ALC RUNAWAY GUARD (2026-07-04, F18): a fully-failed group invalidates any
        // accumulated LOW-level streak — those readings were fade-trough artifacts,
        // not drive-starvation evidence (see the SACK-emit advisory gate).
        if (!all_ok && frame_mask == 0) {
            rx_level_low_streak_ = 0;
        }
        // BURST-AWARE ACK: this callback IS the group boundary the operator pointed to —
        // "whatever ALPHA sends as a burst, BRAVO must ack, it knows the group ended."
        // Bracket the group's frames so arq_ suppresses its per-frame ack heuristic
        // (which can't ack a sub-window burst) and emits EXACTLY ONE tone-burst ack for
        // the whole burst at the end — cumulative base + hole bitmap, every burst.
        // Live "incoming burst" status for the GUI — the flashing partial-group
        // indicator (group #, X decoded / Y group size from frame_mask). Set this BEFORE
        // processing the frames: if this group completes the file, the file-received
        // callback (setFileReceivedCallback -> burst_activity_ = {}) fires DURING
        // processArqFrame and must WIN. Previously this block ran after the loop and
        // re-activated the indicator post-completion, so it kept flashing "received
        // group X/Y" after the "File Received" toast (looked like an extra/late group).
        {
            unsigned decoded = 0;
            for (uint16_t m = frame_mask; m; m &= (m - 1)) ++decoded;      // popcount = X
            // Y = the REAL group size from the descriptor (modem passes burst_group_size).
            // The old bit-length-of-frame_mask trick UNDERCOUNTS when the group's TRAILING
            // frame(s) fail (a failed frame is a 0 bit, so it's invisible to bit-length) —
            // e.g. a 3-frame group with seq 2 faded showed "2/2" instead of "2/3". Fall
            // back to that heuristic only if group_size wasn't supplied (group_size==0).
            unsigned group_bits = 0;
            for (uint16_t m = frame_mask; m; m >>= 1) ++group_bits;        // bit-length
            unsigned Y = group_size > 0
                             ? group_size
                             : (group_bits > decoded ? group_bits : decoded);
            if (Y < decoded) Y = decoded;
            burst_activity_.active = true;
            burst_activity_.group_seq = group_seq;
            burst_activity_.frames_decoded = static_cast<uint8_t>(decoded);
            burst_activity_.frames_in_group = static_cast<uint8_t>(Y);
            ++burst_activity_.groups_seen;
        }
        // Keep physical-egress provenance true for the *whole* logical processing
        // transaction, not just endGroupReceiveAndAck(): a FINAL DATA frame may
        // synchronously emit its completion SACK from inside processArqFrame().
        // However, a configured/stale fallback N is accounting only.  It does not
        // prove that the peer's physical turn ended, and must not bypass the
        // frontend's recent-RX/CCA gates.  This is defense-in-depth behind the
        // decoder's unproven-marker suppression: if another untrusted outcome ever
        // reaches here, its ACK remains asynchronous and cannot key mid-burst.
        tone_ack_group_complete_context_ = geometry_proven;
        // Software-ALC needs a different, narrower provenance contract than the
        // cumulative SACK: did THIS decoder callback contain any CRC-valid DATA for
        // us? Compute it before processArqFrame(), which can synchronously emit the
        // FINAL ACK. Addressed-away ULPAD frames do not qualify.
        tone_ack_alc_group_context_ = rx_level_verdict_pending_for_group_;
        rx_level_verdict_pending_for_group_ = false;
        tone_ack_group_has_decoded_data_context_ = std::any_of(
            frames.begin(), frames.end(), [this](const Bytes& frame) {
                const auto hdr = v2::parseHeader(frame);
                return hdr.valid && !hdr.is_control &&
                       v2::isAddressedToCallsign(hdr, local_call_);
            });
        // PSDR's advisory=3 is a per-frame SACK provenance claim.  BI1 couples
        // the physical members before decode, so do not let its group callback
        // authorize the BI0-only descriptor/light repair policy.
        tone_ack_exact_group_geometry_context_ = geometry_proven && !interleaved;
        arq_.beginGroupReceive();
        for (const auto& frame : frames) {
            auto hdr = v2::parseHeader(frame);
            // F144 forensics: a clean 5/5 group acked one frame short with ZERO drop
            // evidence — every gate between here and the ARQ window was silent. Name
            // each frame on its way in (bounded: <= group_size lines per group).
            LOG_MODEM(INFO, "Connection: burst->arq %s seq=%u hdr_valid=%d len=%zu",
                      hdr.valid ? v2::frameTypeToString(hdr.type) : "?", hdr.valid ? hdr.seq : 0,
                      hdr.valid ? 1 : 0, frame.size());
            if (hdr.valid && !v2::isAddressedToCallsign(hdr, local_call_)) {
                LOG_MODEM(WARN, "Connection: burst frame seq=%u SKIPPED by pad/callsign filter (dst_hash=%06X)",
                          hdr.seq, hdr.dst_hash);
                continue;  // burst pad — addressed to the pad callsign
            }
            if (isAuthoritativeResponderHandshakeFrame(hdr)) {
                maybeConfirmResponderHandshake(
                    "first CRC-valid burst frame from established initiator");
            }
            processArqFrame(frame);  // a file-completing frame clears burst_activity_ (wins)
        }
        // The ARQ emit callback is synchronous.  The provenance bracket began
        // before frame processing so both FINAL-immediate and boundary SACKs are
        // identified as causally safe.
        arq_.endGroupReceiveAndAck();
        tone_ack_group_complete_context_ = false;
        tone_ack_alc_group_context_ = false;
        tone_ack_group_has_decoded_data_context_ = false;
        tone_ack_exact_group_geometry_context_ = false;
        // WAITING-REBASE voice (BUG-UNANCHORED-SILENCE-ESCAPE, design §5.3, gated on
        // ULTRA_RX_RATE_CMD): checked AFTER the frames processed — if THIS group carried
        // the era base, the interregnum just ended and no voice is needed. While it
        // holds, the ARQ suppressed the group ack above (total ack silence by design),
        // so this is the receiver's ONLY utterance: a tone burst with rung_cmd=3 whose
        // mask/type the sender never parses as an ack (consumed whole in
        // onToneBurstAck). It tells the sender "alive + forward link works + resend
        // the era base" — the anti-manufactured-collapse signal, on the 4-FSK plane
        // that out-survives every OFDM waveform in the trough.
        if (rx_rate_cmd_enabled_ && arq_.rxWaitingRebase() &&
            kInteractiveToneAckEnabled() && on_transmit_tone_burst_ack_) {
            ultra::waveform::tone_burst_ack::ToneBurstAckPayload voice;
            // This is an event identity, not the descriptor's transport group_seq.
            // Unified BURST_HEADER currently carries group_seq=0 for every physical
            // group, which made the sender dedup every voice after the first one.
            // Allocate once at the distinct group-outcome boundary; the front end's
            // cached RF repeat reuses this already-stamped payload unchanged.
            voice.group_seq = rebase_voice_event_seq_;
            rebase_voice_event_seq_ = static_cast<uint8_t>(
                (rebase_voice_event_seq_ + 1u) & 0x3Fu);
            voice.frame_mask = 0;  // meaningless — consumed whole, never as a SACK
            voice.type = ultra::waveform::tone_burst_ack::AckType::Nack;
            voice.rung_cmd = ultra::waveform::tone_burst_ack::kRungCmdReserved;  // 3
            LOG_MODEM(WARN,
                      "Connection: WAITING-REBASE — voicing unanchored state to the "
                      "sender (event_seq=%u, descriptor_group_seq=%u)",
                      static_cast<unsigned>(voice.group_seq),
                      static_cast<unsigned>(group_seq));
            on_transmit_tone_burst_ack_(voice, /*inbound_group_complete=*/true);
        }
        return;
    }
}

size_t Connection::sendNextFragment() {
    MessageTxStatusDeferralGuard status_guard(*this);
    if (disconnect_teardown_active_) {
        burst_mode_active_ = false;
        burst_tx_buffer_.clear();
        return 0;
    }
    // A host transport is allowed to loop an ACK back synchronously from the
    // transmit callback. Defer that ACK's refill until this submission loop has
    // advanced its own indices; otherwise the nested refill can clear/reuse the
    // fragment vectors under the outer frame's references.
    const bool owns_refill_defer = !arq_callback_defer_refill_;
    if (owns_refill_defer) {
        arq_callback_defer_refill_ = true;
    }
    size_t new_fragments_submitted = 0;
    auto finish = [this, owns_refill_defer, &new_fragments_submitted]() {
        if (owns_refill_defer) {
            arq_callback_defer_refill_ = false;
            runDeferredArqRefill();
        }
        return new_fragments_submitted;
    };
    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool pipeline_fragments = is_ofdm;

    // Enable burst buffering for OFDM mode
    if (is_ofdm && on_transmit_burst_) {
        burst_mode_active_ = true;
        burst_tx_buffer_.clear();
    }

    // UNIFIED PATH: bound a message burst to the SAME airtime budget as a file burst
    // (one budget-sized group per key-down, ack timeout sized to it) — so a large
    // message and a file transfer key down identically. SIZE_MAX (no cap) off the
    // unified OFDM path, preserving legacy fill-the-window message behavior.
    const bool repair_turn =
        is_ofdm && kUnifiedSeqEnabled() && burst_mode_active_ &&
        arq_.getTxInFlightBytes() > 0;
    const bool descriptor_only_partial_repair =
        repair_turn && partial_sack_descriptor_repair_scope_;
    partial_sack_descriptor_repair_scope_ = false;
    const size_t burst_frame_cap = prepareUnifiedBurstWindow(
        repair_turn && !descriptor_only_partial_repair);
    size_t submitted_this_call = 0;
    size_t retransmitted_frames = 0;

    // STOP-AND-WAIT parity with the file path: a tone SACK ends the previous physical
    // turn and identifies any holes. Put those repairs at the front of this turn, then
    // use the remaining airtime budget for new fragments. This also handles the tail
    // case where next_fragment_idx_ already reached the end and only one hole remains.
    if (is_ofdm && kUnifiedSeqEnabled() && burst_mode_active_) {
        retransmitted_frames =
            arq_.retransmitInFlightUnacked(burst_frame_cap);
        submitted_this_call = retransmitted_frames;
        if (deferred_arq_failure_abort_) {
            burst_mode_active_ = false;
            burst_tx_buffer_.clear();
            return finish();
        }
    }
    while (arq_.isReadyToSend() && next_fragment_idx_ < pending_tx_fragments_.size()) {
        if (submitted_this_call >= burst_frame_cap) {
            break;  // one budget-sized group per burst (unified path)
        }
        const size_t fragment_index = next_fragment_idx_;
        const Bytes& chunk = pending_tx_fragments_[fragment_index];
        const size_t chunk_size = chunk.size();

        // Use pre-computed flags if available (from sendMessages batch),
        // otherwise derive from position (single-message fragmentation)
        uint8_t flags;
        if (fragment_index < pending_tx_fragment_flags_.size()) {
            flags = pending_tx_fragment_flags_[fragment_index];
        } else {
            bool is_last = (fragment_index + 1 == pending_tx_fragments_.size());
            flags = is_last ? v2::Flags::NONE : v2::Flags::MORE_FRAG;
        }
        v2::FrameType frame_type = v2::FrameType::DATA;
        if (fragment_index < pending_tx_fragment_types_.size()) {
            frame_type = pending_tx_fragment_types_[fragment_index];
        }
        uint64_t message_token = 0;
        if (fragment_index < pending_tx_fragment_message_tokens_.size()) {
            message_token = pending_tx_fragment_message_tokens_[fragment_index];
        }

        LOG_MODEM(DEBUG, "Connection: Sending fragment %zu/%zu (%zu bytes, type=%s, flags=0x%02X)",
                  fragment_index + 1, pending_tx_fragments_.size(), chunk_size,
                  v2::frameTypeToString(frame_type), flags);

        // Advance before entering the transport callback. A synchronous ACK can
        // legitimately complete and clear the whole batch before this call returns.
        ++next_fragment_idx_;
        ++submitted_this_call;
        ++new_fragments_submitted;
        const bool sent =
            sendArqPayloadFrame(chunk, frame_type, flags, is_ofdm, message_token);
        if (!sent) {
            // No transport callback ran on refusal, so rolling the local indices
            // back is safe and leaves this exact fragment for a later refill.
            --next_fragment_idx_;
            --submitted_this_call;
            --new_fragments_submitted;
            LOG_MODEM(WARN,
                      "Connection: ARQ refused fragment %zu/%zu; retaining it for a later refill",
                      fragment_index + 1, pending_tx_fragments_.size());
            break;
        }
        noteDataTurnPayloadStarted(chunk_size);

        // MC-DPSK bulk-file transfer uses sendNextFileChunk() and is safe to
        // pipeline. The message-fragment path still shares OFDM-oriented
        // MORE_FRAG/SACK semantics, so keep it stop-and-wait until it has its
        // own hardware-validated burst pacing.
        if (!pipeline_fragments && submitted_this_call >= 1) {
            break;
        }
    }

    // Flush burst buffer
    if (is_ofdm && on_transmit_burst_) {
        burst_mode_active_ = false;
        flushBurstBuffer(
            retransmitted_frames > 0,
            descriptor_only_partial_repair && retransmitted_frames > 0);
    }
    return finish();
}

// =============================================================================
// FRAME DISPATCHING
// =============================================================================

void Connection::onFrameReceived(const Bytes& frame_data,
                                 bool physical_turn_complete) {
    MessageTxStatusDeferralGuard status_guard(*this);
    if (frame_data.size() < 2) {
        return;
    }

    // Check v2 magic
    uint16_t magic = (static_cast<uint16_t>(frame_data[0]) << 8) | frame_data[1];
    if (magic != v2::MAGIC_V2) {
        LOG_MODEM(TRACE, "Connection: Ignoring frame with wrong magic");
        return;
    }

    auto header = v2::parseHeader(frame_data);
    if (!header.valid) {
        LOG_MODEM(TRACE, "Connection: Ignoring frame with invalid header");
        return;
    }

    // Check if frame is for us
    uint32_t our_hash = v2::hashCallsign(local_call_);
    if (header.dst_hash != our_hash && header.dst_hash != 0xFFFFFF) {
        LOG_MODEM(TRACE, "Connection: Ignoring frame for different station");
        return;
    }

    // A responder remains logically CONNECTED during its close-recovery grace,
    // so ConnectionState alone cannot protect this boundary.  Once teardown is
    // active, only a duplicate/crossed DISCONNECT and (for a local/crossed close)
    // its sentinel ACK are authoritative.  Stale DATA, MODE_CHANGE, turnover,
    // file control, and ordinary ACKs must not mutate ARQ state or trigger egress.
    if (disconnect_teardown_active_ &&
        !isAllowedDisconnectTeardownRx(header)) {
        LOG_MODEM(DEBUG,
                  "Connection: Dropping %s seq=%u during disconnect teardown",
                  v2::frameTypeToString(header.type), header.seq);
        return;
    }

    // Only a CRC-valid, locally-addressed post-CONNECT header from the established
    // peer proves that peer advanced past CONNECT_ACK. This deliberately excludes
    // duplicate CONNECT, probe traffic, and valid traffic from another station.
    if (isAuthoritativeResponderHandshakeFrame(header)) {
        maybeConfirmResponderHandshake(
            "first CRC-valid classic frame from established initiator");
    }

    // Resolve source callsign from hash if possible
    std::string src_call;
    if (!remote_call_.empty() && v2::hashCallsign(remote_call_) == header.src_hash) {
        src_call = remote_call_;
    } else if (!pending_remote_call_.empty() && v2::hashCallsign(pending_remote_call_) == header.src_hash) {
        src_call = pending_remote_call_;
    }

    LOG_MODEM(DEBUG, "Connection: Received %s seq=%d from hash 0x%06X",
              v2::frameTypeToString(header.type), header.seq, header.src_hash);

    // DISCONNECT now uses control-frame encoding (20 bytes) for hardened
    // 1-CW handling. Keep ConnectFrame fallback for legacy peers/log replay.
    if (header.type == v2::FrameType::DISCONNECT) {
        if (auto ctrl = v2::ControlFrame::deserialize(frame_data)) {
            handleDisconnect(*ctrl, src_call);
            return;
        }

        if (auto conn = v2::ConnectFrame::deserialize(frame_data)) {
            std::string frame_src_call = conn->getSrcCallsign();
            if (!frame_src_call.empty()) {
                src_call = frame_src_call;
            }
            handleDisconnectFrame(*conn, src_call);
            return;
        }

        LOG_MODEM(WARN, "Connection: Failed to parse DISCONNECT frame");
        return;
    }

    // Check frame type category and dispatch accordingly
    if (v2::isConnectFrame(header.type)) {
        // CONNECT/CONNECT_ACK/CONNECT_NAK - parse as ConnectFrame (carries full callsigns)
        auto conn = v2::ConnectFrame::deserialize(frame_data);
        if (conn) {
            // Extract callsign from frame if available
            std::string frame_src_call = conn->getSrcCallsign();
            if (!frame_src_call.empty()) {
                src_call = frame_src_call;  // Use verified callsign from frame
            }

            switch (conn->type) {
                case v2::FrameType::CONNECT:
                    handleConnect(*conn, src_call);
                    break;
                case v2::FrameType::CONNECT_ACK:
                    handleConnectAck(*conn, src_call);
                    break;
                case v2::FrameType::CONNECT_NAK:
                    handleConnectNak(*conn, src_call);
                    break;
                default:
                    break;
            }
        }
    } else if (v2::isControlFrame(header.type)) {
        // Control frames (DISCONNECT, ACK, NACK, etc) - parse as ControlFrame
        auto ctrl = v2::ControlFrame::deserialize(frame_data);
        if (ctrl) {
            switch (ctrl->type) {
                case v2::FrameType::ACK:
                    if (state_ == ConnectionState::DISCONNECTING) {
                        if (ctrl->seq == v2::DISCONNECT_SEQ) {
                            LOG_MODEM(INFO, "Connection: Disconnect acknowledged (seq=0x%04X)", ctrl->seq);
                            enterDisconnected("Disconnect complete");
                        } else {
                            LOG_MODEM(DEBUG, "Connection: Ignoring stale data ACK seq=%d while disconnecting", ctrl->seq);
                        }
                    } else if (state_ == ConnectionState::CONNECTED) {
                        // Check if this ACK is for our pending MODE_CHANGE
                        if (mode_change_pending_ && ctrl->seq == mode_change_seq_) {
                            commitPendingModeChange("ACKed");
                        } else {
                            // Regular data ACK
                            if (local_data_turn_) {
                                armDataTurnTxGuard(dataTurnAckDiversityGuardMs(*ctrl));
                                if ((ctrl->flags & v2::Flags::TURN_REQUEST) != 0) {
                                    peer_data_turn_requested_ = true;
                                    LOG_MODEM(INFO,
                                              "Connection: Peer requested DATA turn on ACK seq=%u",
                                              ctrl->seq);
                                }
                            }
                            processArqFrame(frame_data);
                            maybeYieldDataTurn();
                        }
                    }
                    break;
                case v2::FrameType::NACK:
                    if (state_ == ConnectionState::CONNECTED) {
                        processArqFrame(frame_data);
                    }
                    break;
                case v2::FrameType::MODE_CHANGE:
                    handleModeChange(*ctrl, src_call);
                    break;
                case v2::FrameType::TURNOVER:
                    handleTurnover(*ctrl, src_call);
                    break;
                case v2::FrameType::TURN_REQUEST:
                    handleTurnRequest(*ctrl, src_call);
                    break;
                case v2::FrameType::FILE_CANCEL:
                    handleFileCancel(*ctrl, src_call);
                    break;
                case v2::FrameType::PROBE:
                case v2::FrameType::PROBE_ACK:
                    // PROBE not used - ignore (or could respond with CONNECT_NAK)
                    LOG_MODEM(DEBUG, "Connection: Ignoring PROBE (not supported)");
                    break;
                default:
                    break;
            }
        }
    } else {
        // Data frame - pass to ARQ
        if (state_ == ConnectionState::CONNECTED) {
            if (file_cancel_rx_drain_ms_ > 0 || file_cancel_reassert_ms_ > 0) {
                LOG_MODEM(INFO,
                          "Connection: Dropping stale DATA seq=%u during FILE_CANCEL drain/reassert (drain=%ums reassert=%ums)",
                          header.seq, file_cancel_rx_drain_ms_, file_cancel_reassert_ms_);
                if (file_cancel_rx_drain_ms_ == 0) {
                    file_cancel_rx_drain_ms_ = FILE_CANCEL_RX_DRAIN_MS;
                }
                maybeReassertFileCancelForStaleData();
                return;
            }
            if (physical_turn_complete) {
                // The classic path can carry either a full-anchored singleton or
                // the CRC-protected final member recovered after descriptor/head
                // loss. The decoder proved the physical turn is over in both
                // cases. Recreate the same logical group transaction used by
                // onBurstGroupReceived so the cumulative window (including prior
                // orphan DATA), duplicates, and FINAL each emit exactly one SACK
                // with safe reverse-egress provenance.
                // No exact group k/M reaches this classic tail path.  Never let an
                // outstanding startup UP command hitch a ride on its ACK.
                failClosedLatentStartupProbeUnknown("classic physical tail");
                rx_level_verdict_pending_for_group_ = false;
                tone_ack_alc_group_context_ = false;
                tone_ack_group_has_decoded_data_context_ = false;
                tone_ack_exact_group_geometry_context_ = false;
                tone_ack_group_complete_context_ = true;
                arq_.beginGroupReceive();
                processArqFrame(frame_data);
                arq_.endGroupReceiveAndAck();
                tone_ack_group_complete_context_ = false;
                tone_ack_exact_group_geometry_context_ = false;
            } else {
                processArqFrame(frame_data);
            }
        }
    }
}

void Connection::noteAnchoredBurstNoGroup(bool payload_seen) {
    if (disconnect_teardown_active_ ||
        state_ != ConnectionState::CONNECTED || !use_burst_transport_ ||
        !kUnifiedSeqEnabled()) {
        return;
    }
    LOG_MODEM(WARN,
              "Connection: ANCHORED-BURST BACKSTOP — re-confirm ack; "
              "payload_seen=%d group geometry unavailable",
              payload_seen ? 1 : 0);
    // This timer/backstop has no wire-proven group geometry.  Clear a standing startup
    // probe command before every ACK variant, including payload_seen=true where the
    // selector observation itself is intentionally withheld.
    failClosedLatentStartupProbeUnknown("anchored no-group backstop");
    // Crater verdict BEFORE the ack so the command rides it (same ordering as
    // onBurstGroupReceived). A decoded fallback payload makes k=0 false, while
    // the lost descriptor leaves M unknowable; skip the selector observation
    // rather than inventing the latent controller's fallback 0/5 sample.
    if (rxRateAuthorityEnabled() &&
        connection_policy::shouldGradeAnchoredBackstopAsCrater(payload_seen)) {
        updateRxAuthorityCommand(/*all_ok=*/false, /*quality=*/0.0f, /*full_crater=*/true);
    } else if (payload_seen) {
        LOG_MODEM(INFO,
                  "Connection: ANCHORED-BURST selector observation withheld — "
                  "fallback DATA decoded but exact group k/M is unknown");
    }
    // This is a TIMER backstop, not a synchronous physical group boundary.  Its
    // 14.6 s deadline can expire after the peer has already started a new burst;
    // live IONOS logs caught two such ACKs erasing the head descriptors of the
    // recovery turns.  Keep asynchronous provenance so the frontend's CCA,
    // recent-RX, and decoder-air gates can defer/drop it safely.  A real group or
    // standalone physical completion uses the explicitly-safe paths instead.
    tone_ack_group_complete_context_ = false;
    rx_level_verdict_pending_for_group_ = false;
    tone_ack_alc_group_context_ = false;
    tone_ack_group_has_decoded_data_context_ = false;
    tone_ack_exact_group_geometry_context_ = false;
    arq_.endGroupReceiveAndAck();
}

void Connection::noteBurstOutcomeUnknown() {
    if (disconnect_teardown_active_) {
        return;
    }
    rx_level_verdict_pending_for_group_ = false;
    tone_ack_alc_group_context_ = false;
    tone_ack_group_has_decoded_data_context_ = false;
    failClosedLatentStartupProbeUnknown("decoder abandoned unproven marker");
}

void Connection::processArqFrame(const Bytes& frame_data) {
    // TEST HOOK (env ULTRA_DROP_RX_SEQ=N): drop the FIRST receipt of the DATA frame with
    // seq=N to prove SELECTIVE repeat — the ack bitmap must then show only that hole and
    // the sender must resend ONLY that frame. One-shot: the resend is accepted.
    static const long kDropRxSeq = [] {
        const char* e = std::getenv("ULTRA_DROP_RX_SEQ");
        return e ? std::atol(e) : -1L;
    }();
    if (kDropRxSeq >= 0) {
        static bool dropped_once = false;
        auto hdr = v2::parseHeader(frame_data);
        if (!dropped_once && hdr.valid && !hdr.is_control &&
            hdr.seq == static_cast<uint16_t>(kDropRxSeq)) {
            dropped_once = true;
            LOG_MODEM(WARN, "Connection: TEST-DROP RX DATA seq=%u (one-shot, proving selective repeat)",
                      hdr.seq);
            return;
        }
    }

    const bool outermost = !arq_callback_defer_refill_;
    if (outermost) {
        arq_callback_defer_refill_ = true;
    }

    arq_.onFrameReceived(frame_data);

    if (outermost) {
        arq_callback_defer_refill_ = false;
        runDeferredArqRefill();
    }
}

void Connection::onMCDPSKPartialFrame(const v2::PartialFrameCodewords& partial) {
    if (disconnect_teardown_active_ || state_ != ConnectionState::CONNECTED ||
        negotiated_mode_ != WaveformMode::MC_DPSK) {
        return;
    }
    if (!partial.valid()) {
        return;
    }
    const uint32_t our_hash = v2::hashCallsign(local_call_);
    if (partial.dst_hash != our_hash && partial.dst_hash != 0xFFFFFF) {
        return;
    }

    const bool outermost = !arq_callback_defer_refill_;
    if (outermost) {
        arq_callback_defer_refill_ = true;
    }

    arq_.onPartialFrame(partial);

    if (outermost) {
        arq_callback_defer_refill_ = false;
        runDeferredArqRefill();
    }
}

void Connection::onAcceptedOFDMDataSync(float sync_correlation) {
    if (disconnect_teardown_active_ || state_ != ConnectionState::CONNECTED || is_initiator_ ||
        !isOFDMMode(negotiated_mode_)) {
        return;
    }
    if (connect_ack_frame_.empty()) {
        return;
    }

    // A sync correlation is not authoritative peer state: live IONOS accepted a
    // duplicate MC-DPSK CONNECT as weak OFDM sync, then decoded an empty 0/6 group.
    // Destroying the cache made that half-open permanent. Keeping the old timer,
    // however, was also rig-falsified (3/3 deadlocks): an 8.3 s proactive ACK
    // collided with the initiator's CONNECT retry on the half-duplex channel.
    //
    // There is no scheduled retransmission: retain the immutable ACK bytes so a
    // fully decoded duplicate CONNECT can replay them reactively after the peer's
    // key-down has ended. A CRC-valid post-CONNECT frame from the established peer
    // clears the cache and confirms through maybeConfirmResponderHandshake().
    LOG_MODEM(INFO,
              "Connection: Accepted OFDM DATA sync (corr=%.2f) is not handshake proof; "
              "cached ACK retained for reactive duplicate-CONNECT replay",
              sync_correlation);
}

bool Connection::drainDeferredArqFailureAbort() {
    if (!deferred_arq_failure_abort_) {
        return false;
    }

    const bool failed_file =
        file_transfer_.getState() == FileTransferState::SENDING;
    const bool failed_fragment_batch = !pending_tx_fragments_.empty();
    const uint16_t failed_seq = deferred_arq_failure_seq_;

    // Resolve every status record owned by the active logical fragment batch, but do
    // not invoke application code while iterating the live deque. A FAILED callback is
    // allowed to synchronously send/reset/disconnect; publishing from local copies only
    // after ARQ retirement and epoch bump makes that reentrancy safe.
    std::vector<OutboundMessageTxRecord> failed_message_records;
    for (auto it = outbound_message_tx_records_.begin();
         it != outbound_message_tx_records_.end();) {
        const bool failed_seq_belongs_to_record =
            deferred_arq_failure_seq_valid_ && it->first_seq_valid &&
            (failed_seq == it->first_seq ||
             failed_seq == it->last_seq ||
             (seqBefore(it->first_seq, failed_seq) &&
              seqBefore(failed_seq, it->last_seq)));
        const bool belongs_to_active_fragment_batch =
            std::find(pending_tx_fragment_message_tokens_.begin(),
                      pending_tx_fragment_message_tokens_.end(),
                      it->token) != pending_tx_fragment_message_tokens_.end();
        if (!it->terminal_reported &&
            (failed_seq_belongs_to_record || belongs_to_active_fragment_batch)) {
            it->terminal_reported = true;
            failed_message_records.push_back(*it);
            it = outbound_message_tx_records_.erase(it);
        } else {
            ++it;
        }
    }

    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();

    // The failed slot was retired before this post-callback drain. Force a fresh
    // move epoch even if it was the only live slot, so the next DATA frame carries
    // EPOCH_REBASE and the peer can skip the deliberately abandoned sequence.
    arq_.abortPendingTx(/*force_move_epoch_bump=*/true);

    const bool must_disconnect = failed_file || !arq_.moveEpochEnabled();

    // Clear logical-transfer state before publishing any external callback. A callback
    // can now enqueue a new message safely: the old slots are gone, the epoch has
    // already advanced, and no outer iterator references application-owned state.
    if (failed_fragment_batch) {
        const size_t remaining =
            next_fragment_idx_ < pending_tx_fragments_.size()
                ? pending_tx_fragments_.size() - next_fragment_idx_
                : 0;
        LOG_MODEM(WARN,
                  "Connection: Fragment send failed, aborting remaining %zu fragments",
                  remaining);
    }
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    pending_tx_fragment_message_tokens_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;

    deferred_arq_failure_abort_ = false;
    deferred_arq_failure_seq_valid_ = false;

    if (must_disconnect && state_ == ConnectionState::CONNECTED) {
        // Without the negotiated move-epoch wire semantics there is no safe way to
        // continue DATA after abandoning a sequence the peer never received. A failed
        // FILE also requires a disconnect: move-epoch resets ARQ, but the peer's file
        // assembler remains RECEIVING and would reject the next FILE_START.
        LOG_MODEM(ERROR, "Connection: Terminal ARQ %s failure%s; disconnecting to "
                         "reset peer transfer and sequence state",
                  failed_file ? "FILE" : "DATA",
                  arq_.moveEpochEnabled() ? "" : " with MOVE-EPOCH disabled");
        disconnect();
    } else {
        LOG_MODEM(ERROR,
                  "Connection: Terminal ARQ failure aborted the remaining logical "
                  "TX window; next DATA will rebase at move-epoch %u",
                  static_cast<unsigned>(arq_.txMoveEpoch()));
    }

    emitFailedMessageRecords(failed_message_records);

    if (failed_file) {
        file_transfer_.onSendFailed();
    } else if (on_message_sent_) {
        on_message_sent_(false);
    }
    return true;
}

void Connection::runDeferredArqRefill() {
    if (arq_callback_defer_refill_) {
        return;
    }

    drainDeferredArqFailureAbort();

    if (mode_change_pending_) {
        return;
    }

    const bool refill_file = deferred_file_refill_;
    const bool refill_fragments = deferred_fragment_refill_;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;

    if (state_ != ConnectionState::CONNECTED) {
        return;
    }
    // §RETX-PACING §1.3 trigger #1: retx_pace_hold_ms_ > 0 blocks the turn refill exactly
    // like the existing guards — the deferred-refill flags RE-LATCH below, so the refill
    // fires automatically (same [holes]+[new] coalescing, untouched) when the hold expires
    // in the CONNECTED tick. Default-off knob ⇒ the hold is never armed ⇒ byte-identical.
    if (!local_data_turn_ || file_cancel_confirm_pending_ || data_turn_tx_guard_ms_ > 0 ||
        retx_pace_hold_ms_ > 0) {
        deferred_file_refill_ = refill_file || deferred_file_refill_;
        deferred_fragment_refill_ = refill_fragments || deferred_fragment_refill_;
        return;
    }
    if (shouldPauseLocalDataForPeerRequest()) {
        deferred_file_refill_ = refill_file || deferred_file_refill_;
        deferred_fragment_refill_ = refill_fragments || deferred_fragment_refill_;
        maybeYieldDataTurn();
        return;
    }

    if (refill_file && file_transfer_.getState() == FileTransferState::SENDING) {
        sendNextFileChunk();
        if (drainDeferredArqFailureAbort()) {
            sendNextQueuedPayloadIfReady();
            return;
        }
    }

    const bool fragment_hole_remains =
        !pending_tx_fragments_.empty() && arq_.getTxInFlightBytes() > 0;
    if (refill_fragments &&
        !pending_tx_fragments_.empty() &&
        (next_fragment_idx_ < pending_tx_fragments_.size() ||
        fragment_hole_remains)) {
        sendNextFragment();
        if (drainDeferredArqFailureAbort()) {
            sendNextQueuedPayloadIfReady();
            return;
        }
    }

    sendNextQueuedPayloadIfReady();
}

// =============================================================================
// TIMER / TICK
// =============================================================================

bool Connection::tickDisconnectResponderGrace(uint32_t elapsed_ms) {
    if (!disconnect_pending_) {
        return false;
    }

    if (elapsed_ms >= disconnect_pending_ms_) {
        const bool crossed_close = state_ == ConnectionState::DISCONNECTING;
        disconnect_pending_ = false;
        disconnect_pending_ms_ = 0;
        disconnect_ack_retransmit_ms_ = 0;
        disconnect_ack_epoch_elapsed_ms_ = 0;
        disconnect_ack_repeat_count_ = 0;
        disconnect_ack_frame_.clear();
        enterDisconnected(crossed_close ? "Mutual disconnect complete"
                                        : "Remote disconnected");
        return true;
    }
    disconnect_pending_ms_ -= elapsed_ms;
    disconnect_ack_epoch_elapsed_ms_ =
        (disconnect_ack_epoch_elapsed_ms_ > 0xFFFFFFFFu - elapsed_ms)
            ? 0xFFFFFFFFu
            : disconnect_ack_epoch_elapsed_ms_ + elapsed_ms;

    // Proactively re-send ACK periodically because the first copy may fade.
    // A crossed close is already conclusive on both sides; timed symmetric ACK
    // trains would only create another phase lock, so that case stays reactive.
    const int repeat_limit = state_ == ConnectionState::DISCONNECTING
        ? 0
        : disconnectAckMaxProactiveRepeats();
    if (!disconnect_ack_frame_.empty() &&
        disconnect_ack_repeat_count_ < repeat_limit) {
        if (elapsed_ms >= disconnect_ack_retransmit_ms_) {
            if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
                const uint64_t safe_finish_ms =
                    static_cast<uint64_t>(disconnect_ack_epoch_elapsed_ms_) +
                    2ULL * static_cast<uint64_t>(disconnectControlKeydownMs()) +
                    static_cast<uint64_t>(
                        connection_policy::kCarrierSenseSackCoalesceMs);
                if (safe_finish_ms >= disconnectRetryIntervalMs()) {
                    disconnect_ack_repeat_count_ = repeat_limit;
                    disconnect_ack_retransmit_ms_ = 0;
                    LOG_MODEM(DEBUG,
                              "Connection: Skipping late disconnect ACK copy "
                              "at %ums; preserving quiet retry window",
                              disconnect_ack_epoch_elapsed_ms_);
                    return false;
                }
            }

            if ((tx_active_provider_ && tx_active_provider_()) ||
                (channel_busy_query_ && channel_busy_query_())) {
                disconnect_ack_retransmit_ms_ =
                    connection_policy::kCarrierSenseSackCoalesceMs;
                LOG_MODEM(DEBUG,
                          "Connection: Disconnect ACK repeat due but channel/TX "
                          "busy; polling in %ums without spending repeat budget",
                          disconnect_ack_retransmit_ms_);
                return false;
            }

            disconnect_ack_repeat_count_++;
            disconnect_ack_retransmit_ms_ = disconnectAckRetransmitMs();
            LOG_MODEM(INFO,
                      "Connection: Re-sending disconnect ACK (proactive %d/%d, "
                      "%dms grace remaining)",
                      disconnect_ack_repeat_count_,
                      repeat_limit,
                      disconnect_pending_ms_);
            transmitFrame(disconnect_ack_frame_);
        } else {
            disconnect_ack_retransmit_ms_ -= elapsed_ms;
        }
    }
    return false;
}

void Connection::tick(uint32_t elapsed_ms) {
    soft_combine_harq_.tick(elapsed_ms);
    // #58 increment 3: age the connect-SNR pool on the modem-time tick (elapsed_ms is
    // the same clock every timer here runs on — never Date/wall-clock). Ticks in ALL
    // states: handshake readings accumulate age while DISCONNECTED/CONNECTING too.
    connect_snr_pool_.tick(elapsed_ms);
    // The fading-freshness clock ages on the same tick (saturating).
    ms_since_fading_update_ =
        (ms_since_fading_update_ > 0x7FFFFFFF - elapsed_ms)
            ? 0x7FFFFFFF
            : ms_since_fading_update_ + elapsed_ms;

    switch (state_) {
        case ConnectionState::PROBING:
            // Fast presence check via PING/PONG
            if (elapsed_ms >= timeout_remaining_ms_) {
                ping_retry_count_++;
                if (ping_retry_count_ >= MAX_PING_RETRIES) {
                    // No response after all PINGs - give up
                    LOG_MODEM(INFO, "Connection: No response after %d PINGs, giving up",
                              MAX_PING_RETRIES);
                    stats_.connects_failed++;
                    enterDisconnected("No response");
                } else {
                    LOG_MODEM(INFO, "Connection: PING timeout, retrying (%d/%d)",
                              ping_retry_count_, MAX_PING_RETRIES);
                    if (on_ping_tx_) {
                        on_ping_tx_();
                    }
                    timeout_remaining_ms_ = pingTimeoutMsForCurrentProfile();
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        case ConnectionState::CONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                connect_retry_count_++;
                if (connect_retry_count_ >= config_.connect_retries) {
                    LOG_MODEM(ERROR, "Connection: Connect failed after %d attempts",
                              config_.connect_retries);
                    stats_.connects_failed++;
                    char reason[64];
                    snprintf(reason, sizeof(reason), "Connection timeout after %d attempts", config_.connect_retries);
                    enterDisconnected(reason);
                } else {
                    LOG_MODEM(WARN, "Connection: Connect timeout, retrying via %s (%d/%d)",
                              waveformModeToString(connect_waveform_),
                              connect_retry_count_ + 1, config_.connect_retries);
                    auto connect_frame = v2::ConnectFrame::makeConnect(local_call_, remote_call_,
                                                                        config_.mode_capabilities,
                                                                        static_cast<uint8_t>(config_.preferred_mode),
                                                                        static_cast<uint8_t>(outbound_forced_modulation_),
                                                                        static_cast<uint8_t>(outbound_forced_code_rate_),
                                                                        config_.forced_cw_count);
                    transmitFrame(connect_frame.serialize());
                    timeout_remaining_ms_ = connectRetryIntervalMs();
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        case ConnectionState::CONNECTED:
            connected_time_ms_ += elapsed_ms;
            stats_.connected_time_ms = connected_time_ms_;
            // Teardown is egress-exclusive. Once peer close intent is accepted,
            // only its ACK/grace machinery may run while the responder remains
            // logically CONNECTED for duplicate recovery.
            if (disconnect_pending_) {
                (void)tickDisconnectResponderGrace(elapsed_ms);
                break;
            }
            if (data_turn_tx_guard_ms_ > 0) {
                data_turn_tx_guard_ms_ =
                    elapsed_ms >= data_turn_tx_guard_ms_ ? 0 : data_turn_tx_guard_ms_ - elapsed_ms;
            }
            // §RETX-PACING §1.3: tick the trough-pacing hold down BEFORE the
            // runDeferredArqRefill() below, so an expiring hold releases the latched
            // deferred refill in the SAME tick (the deferred-refill flags stay latched
            // while the hold runs — no resubmission logic changes).
            if (retx_pace_hold_ms_ > 0) {
                retx_pace_hold_ms_ =
                    elapsed_ms >= retx_pace_hold_ms_ ? 0 : retx_pace_hold_ms_ - elapsed_ms;
            }
            if (turn_request_holdoff_ms_ > 0) {
                turn_request_holdoff_ms_ =
                    elapsed_ms >= turn_request_holdoff_ms_ ? 0 : turn_request_holdoff_ms_ - elapsed_ms;
            }
            if (file_cancel_rx_drain_ms_ > 0) {
                file_cancel_rx_drain_ms_ =
                    elapsed_ms >= file_cancel_rx_drain_ms_ ? 0 : file_cancel_rx_drain_ms_ - elapsed_ms;
            }
            if (file_cancel_reassert_ms_ > 0) {
                file_cancel_reassert_ms_ =
                    elapsed_ms >= file_cancel_reassert_ms_ ? 0 : file_cancel_reassert_ms_ - elapsed_ms;
            }
            if (file_cancel_reassert_cooldown_ms_ > 0) {
                file_cancel_reassert_cooldown_ms_ =
                    elapsed_ms >= file_cancel_reassert_cooldown_ms_
                        ? 0
                        : file_cancel_reassert_cooldown_ms_ - elapsed_ms;
            }
            if (local_data_turn_ && peer_data_turn_requested_) {
                data_turn_contended_ms_ += elapsed_ms;
            }
            // While we've yielded and are waiting for the peer's first DATA burst, keep
            // the decoder armed to COLD-acquire its full chirp+LTS anchor. Ordinary GUI
            // message replies reverse the DATA turn too; this is therefore driven by the
            // actual turn state, not the stronger half-duplex-interactive B2F policy.
            // The one-shot armed on the yield-TURNOVER gets consumed in the gap (stray
            // detect / warm-sync DEGRADED), so re-arm periodically until the peer's data
            // actually arrives (yielded_data_turn_waiting_for_peer_data_ clears). The peer
            // force-fulls its first frame on turn-acquire, so this meets it (BUG-TNC-B2F-001).
            if (yielded_data_turn_waiting_for_peer_data_ &&
                !local_data_turn_ && on_full_ofdm_anchor_expected_) {
                if (elapsed_ms >= interactive_anchor_rearm_ms_) {
                    interactive_anchor_rearm_ms_ = 400;
                    on_full_ofdm_anchor_expected_();
                } else {
                    interactive_anchor_rearm_ms_ -= elapsed_ms;
                }
            }
            // VARA-HF turnaround: the interactive ISS with an empty TX buffer yields to the
            // IRS so the B2F responder can send its SID. Fire once, ~1.5 s after connect.
            if (half_duplex_interactive_ && is_initiator_ && !interactive_initiator_yield_done_) {
                const bool ready = local_data_turn_ && connected_time_ms_ >= 1500 &&
                                   data_turn_tx_guard_ms_ == 0 &&
                                   !hasLocalDataWaitingForTurn() && !file_transfer_.isBusy() &&
                                   arq_.isReadyToSend();
                if (ready) {
                    // Force a full chirp+LTS anchor on this TURNOVER — it is our first OFDM
                    // frame after the MC-DPSK handshake, so the peer hasn't tracked our OFDM
                    // timing and a light preamble won't decode (BUG-TNC-B2F-001).
                    if (on_data_turn_acquired_) {
                        on_data_turn_acquired_();
                    }
                    auto turnover = v2::ControlFrame::makeTurnover(local_call_, remote_call_);
                    transmitFrame(turnover.serialize());
                    local_data_turn_ = false;
                    yielded_data_turn_waiting_for_peer_data_ = true;
                    received_peer_data_since_connect_ = false;
                    resetDataTurnFairness();
                    armDataTurnTxGuard(dataTurnControlGuardMs());
                    interactive_initiator_yield_done_ = true;
                    LOG_MODEM(INFO, "Connection: interactive ISS yielded first DATA turn to %s (B2F responder speaks first)",
                              remote_call_.c_str());
                } else {
                    interactive_yield_log_throttle_ms_ += elapsed_ms;
                    if (interactive_yield_log_throttle_ms_ >= 2000) {
                        interactive_yield_log_throttle_ms_ = 0;
                        LOG_MODEM(DEBUG, "Connection: interactive yield WAIT: turn=%d conn_ms=%u guard=%u data_waiting=%d file_busy=%d arq_ready=%d",
                                  local_data_turn_ ? 1 : 0, connected_time_ms_, data_turn_tx_guard_ms_,
                                  hasLocalDataWaitingForTurn() ? 1 : 0, file_transfer_.isBusy() ? 1 : 0,
                                  arq_.isReadyToSend() ? 1 : 0);
                    }
                }
            }
            if (file_cancel_confirm_pending_ &&
                data_turn_tx_guard_ms_ == 0 &&
                arq_.isReadyToSend()) {
                transmitFileCancelControl(" (confirm)");
                file_cancel_confirm_pending_ = false;
                armDataTurnTxGuard(fileCancelConfirmDataGuardMs());
            }
            if (!local_data_turn_ && hasLocalDataWaitingForTurn() && !local_turn_request_pending_) {
                sendTurnRequestIfNeeded();
            }
            if (!local_data_turn_ && hasLocalDataWaitingForTurn() && local_turn_request_pending_) {
                if (elapsed_ms >= turn_request_retransmit_ms_) {
                    local_turn_request_pending_ = false;
                    sendTurnRequestIfNeeded();
                } else {
                    turn_request_retransmit_ms_ -= elapsed_ms;
                }
            }

            // Responder fail-safe: after the initiator's CONNECT retry window, keep the
            // responder quiet until a real post-CONNECT frame arrives. A duplicate
            // CONNECT means the initiator is still in MC-DPSK setup, so do not mark
            // the protocol handshake confirmed on a timer.
            if (!is_initiator_ && !handshake_confirmed_ &&
                responder_handshake_wait_ms_ > 0) {
                if (elapsed_ms >= responder_handshake_wait_ms_) {
                    responder_handshake_wait_ms_ = 0;
                    LOG_MODEM(WARN,
                              "Connection: Responder handshake still unconfirmed after initiator retry window; waiting for initiator frame");
                } else {
                    responder_handshake_wait_ms_ -= elapsed_ms;
                }
            }
            // HALF-OPEN TIMEOUT (2026-07-04, F29: a one-way CONNECT_ACK loss left the
            // responder "Connected" for 31 minutes while the initiator had cleanly
            // given up ~356 s in after its 10 connect retries). If the handshake NEVER
            // confirms — no classic frame, no burst group, nothing — the initiator is
            // provably gone once its whole retry ladder (~340 s) has elapsed: release
            // the session and return to listening. 240,000 ms > 6x the worst normal
            // confirm time observed (~38 s) and inside the initiator give-up bound.
            if (!is_initiator_ && !handshake_confirmed_) {
                responder_half_open_ms_ += elapsed_ms;
                if (responder_half_open_ms_ >= 240000) {
                    LOG_MODEM(WARN,
                              "Connection: handshake never confirmed %u ms after "
                              "CONNECT_ACK — initiator presumed gone (one-way ACK "
                              "loss); releasing the half-open session",
                              responder_half_open_ms_);
                    enterDisconnected("handshake never confirmed (half-open timeout)");
                    return;
                }
            } else {
                responder_half_open_ms_ = 0;
            }

            tickModeChangeAckRepeats(elapsed_ms);

            // Handle MODE_CHANGE timeout.
            // BUG-MC-RETRY-SPURIOUS (2026-07-04, E1 forensics): the deadline HOLDS
            // while our own TX is keyed — half-duplex means we cannot have decoded
            // the peer's MC-ACK during our key-down, so wall time spent transmitting
            // is not evidence of ACK loss. The MODE_CHANGE rides the TAIL of a
            // bundled data key-down (~10.6 s observed), which alone ate most of the
            // old request-anchored 18.2 s deadline; all three observed spurious-retry
            // cycles (21.1/21.3/30.4 s pipelines) become retry-free with the hold.
            // Unwired provider (tests/headless) => legacy behavior.
            if (mode_change_pending_ && tx_active_provider_ && tx_active_provider_()) {
                // keyed: hold the deadline (neither decrement nor fire)
            } else if (mode_change_pending_) {
                if (elapsed_ms >= mode_change_timeout_ms_) {
                    // A MODE_CHANGE retry is a new half-duplex key-down.  Even after
                    // its control-geometry deadline expires, never launch it over a
                    // reverse ACK (or any other inbound signal).  Poll again after the
                    // existing carrier-sense coalesce interval without consuming retry
                    // budget; an unwired provider keeps the headless/test behavior.
                    if (channel_busy_query_ && channel_busy_query_()) {
                        mode_change_timeout_ms_ =
                            connection_policy::kCarrierSenseSackCoalesceMs;
                        LOG_MODEM(DEBUG,
                                  "Connection: MODE_CHANGE retry due but channel busy; "
                                  "holding retry budget for %ums",
                                  mode_change_timeout_ms_);
                    } else {
                        mode_change_retry_count_++;
                        if (mode_change_retry_count_ > MODE_CHANGE_MAX_RETRIES) {
                            const bool startup_probe_control =
                                pending_reason_ ==
                                    v2::ModeChangeReason::STARTUP_PROBE_TIMEOUT ||
                                pending_reason_ ==
                                    v2::ModeChangeReason::STARTUP_PROBE_BEGIN;
                            LOG_MODEM(WARN,
                                      "Connection: MODE_CHANGE ACK unresolved after %d retries; keeping current %s %s because peer ACK was not proven",
                                      MODE_CHANGE_MAX_RETRIES,
                                      modulationToString(data_modulation_),
                                      codeRateToString(data_code_rate_));
                            mode_change_pending_ = false;
                            mode_change_timeout_ms_ = 0;
                            mode_change_retry_count_ = 0;
                            pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
                            // Old-geometry timeout intents were suspended while the
                            // synchronized exchange owned egress. The peer never confirmed
                            // the switch, so make those exact live slots promptly eligible
                            // again at the retained geometry.
                            if (startup_probe_control) {
                                // For BEGIN, the peer may already have applied R2/3 even
                                // though every ACK was lost; for TIMEOUT, resuming these
                                // intents would put a second unACKed target group on air.
                                // In both cases peer geometry is ambiguous. If robust
                                // control diversity cannot synchronize the episode, reset
                                // the session instead of resuming split-rung DATA.
                                LOG_MODEM(ERROR,
                                          "Connection: startup-probe control could not "
                                          "be confirmed; refusing ambiguous DATA intents "
                                          "and disconnecting (reason=%u)",
                                          pending_reason_);
                                disconnect();
                            } else {
                                arq_.resumeDeferredTimeoutRetransmits(/*timeout_ms=*/1);
                                runDeferredArqRefill();
                            }
                        } else {
                            LOG_MODEM(WARN, "Connection: MODE_CHANGE timeout, retrying (%d/%d)",
                                      mode_change_retry_count_, MODE_CHANGE_MAX_RETRIES);
                            // Resend MODE_CHANGE with same parameters (incl. CW)
                            auto frame = v2::ControlFrame::makeModeChange(local_call_, remote_call_,
                                                                           mode_change_seq_, pending_modulation_,
                                                                           pending_code_rate_, pending_snr_db_,
                                                                           pending_fading_index_,
                                                                           pending_reason_,
                                                                           pending_cw_count_,
                                                                           pending_ladder_rung_id_);
                            transmitFrame(frame.serialize());
                            mode_change_timeout_ms_ = modeChangeRetryMs();
                        }
                    }
                } else {
                    mode_change_timeout_ms_ -= elapsed_ms;
                }
            }

            arq_tick_in_progress_ = true;
            arq_.tick(elapsed_ms);
            arq_tick_in_progress_ = false;
            if (drainDeferredArqFailureAbort() &&
                state_ != ConnectionState::CONNECTED) {
                break;
            }
            handleLatentStartupProbeTimeoutRollback();

            // KEEPALIVE ACK (BUG-ANCHOR-WAIT-NO-ACK-STALL, ULTRA_KEEPALIVE_ACK,
            // default off): break the 44 s RTO stall when marginal bursts get
            // rejected at sync (no ACK emitted) during an active file receive.
            // Threshold 25 s > max burst airtime (window-16 ≈ 21.5 s) so it
            // only fires on a genuine stall (no burst completed in the window),
            // not mid-legit-burst — no channel-busy gating needed for v1. Fires
            // before the 44.68 s RTO, saving ~19 s/stall. The re-emitted ACK
            // routes through listen-before-ACK; the sender resends holes on any
            // tone-burst ACK (turn boundary). v2 (filed): channel-gated shorter
            // threshold for a bigger saving.
            // KEEPALIVE ACK, DEFAULT-ON 2026-07-14 (opt-out ULTRA_KEEPALIVE_ACK=0).
            // 8 s keepalive: rig-proven (F290/F291 max ACK-silence gap capped at
            // exactly 8.0 s vs OFF's 36.7 s, reproducible, cwfails normal).
            // UNIVERSAL collision-safety via channel_busy_query_ (below) — the
            // keepalive checks "is a burst arriving?" itself and skips if busy,
            // so it no longer depends on the GUI-only listen-before-ACK (which
            // had blocked default-on: the headless TNC lacked it). Both the GUI
            // (app.cpp) and the TNC (ultra_tnc.cpp) wire the query from the modem.
            // DEFAULT-OFF (reverted 2026-07-14, rig F293-F298 MPG@20): the
            // keepalive re-emits the FULL tone-burst ACK, which carries the
            // current rung command (rate_hint). During a stall the receiver's
            // reading is momentarily low (usable 8.8, fading misread Moderate),
            // so the re-emits RE-ASSERT the low rung (rate_hint=1 → R1/4) 7×,
            // PINNING R1/4 through recovery + adding ACK-vs-burst collision
            // traffic (18 fires/transfer). A keepalive must trigger a RESEND,
            // NOT re-command the rung. BLOCKER for default-on: emit a
            // rung-NEUTRAL/HOLD keepalive (base+bitmap only, no rate_hint
            // update). Until then opt-in.
            static const bool keepalive_ack_enabled = [] {
                const char* e = std::getenv("ULTRA_KEEPALIVE_ACK");
                return e != nullptr && e[0] != '\0' && e[0] != '0';
            }();
            // Threshold env-tunable (ULTRA_KEEPALIVE_ACK_MS, default 25000).
            // The keepalive routes through listen-before-ACK (defers on channel
            // busy), so a mid-legit-burst fire is deferred, not collided —
            // meaning a SHORTER threshold (v2, ~8-10 s) is safe and catches the
            // stall ~15 s earlier. Left at 25 s by default pending the v2 rig
            // A/B (F284-F289 at 25 s: fires 1-2×/transfer on the real stall,
            // benefit epoch-noise-dominated).
            static const uint32_t keepalive_ack_ms = [] {
                if (const char* e = std::getenv("ULTRA_KEEPALIVE_ACK_MS")) {
                    const long v = std::atol(e);
                    if (v >= 3000 && v <= 60000) return static_cast<uint32_t>(v);
                }
                return 8000u;  // v2 (rig-proven): caps stall at 8 s, no collision
            }();
            // Emit only when the channel is NOT busy (no inbound burst) — the
            // universal collision guard. If unwired (tests), fall back to
            // threshold-only (safe when threshold > max burst airtime).
            if (keepalive_ack_enabled &&
                (!channel_busy_query_ || !channel_busy_query_())) {
                arq_.keepaliveAckIfStalled(keepalive_ack_ms);
            }

            // RX-AUTHORITY: age the verdict-SNR ring (stale readings must not
            // steer the rung after a quiet stretch).
            for (size_t i = 0; i < kRxAuthObsRing; ++i) {
                if (rx_auth_obs_age_ms_[i] <= kRxAuthObsMaxAgeMs) {
                    rx_auth_obs_age_ms_[i] += elapsed_ms;
                }
            }
            maybeEscapeStuckFrame();
            // §RETX-PACING §2: the collapse-conditioned escape is POLLED here (proven-safe
            // context, same as maybeEscapeStuckFrame) — rounds counted inside the ARQ
            // transmit callback (RTO batch) and act after ARQ unwinds, never re-entrantly.
            maybeCollapseEscape();
            // The timeout batch was serialized by ARQ before these escape decisions, but
            // has not gone on air. Commit it only if the rung/geometry is still current;
            // otherwise the mode-change refill owns the next physical turn.
            flushStagedArqTimeoutBatch();
            maybeYieldDataTurn();
            runDeferredArqRefill();
            sendNextQueuedPayloadIfReady();
            break;

        case ConnectionState::DISCONNECTING:
            // Mutual close: once the peer's own DISCONNECT is decoded, its close
            // intent is conclusive. Keep its ACK alive for one responder grace
            // window and suppress our phase-locked local DISCONNECT retries.
            if (disconnect_pending_) {
                (void)tickDisconnectResponderGrace(elapsed_ms);
                break;
            }
            if (elapsed_ms >= timeout_remaining_ms_) {
                LOG_MODEM(INFO, "Connection: Disconnect timeout, forcing disconnect");
                enterDisconnected("Disconnect timeout");
            } else {
                timeout_remaining_ms_ -= elapsed_ms;

                // Retransmit DISCONNECT periodically (fading can lose the frame)
                if (elapsed_ms >= disconnect_retransmit_ms_) {
                    const uint32_t response_window_ms =
                        disconnectRetryIntervalMs();
                    if (disconnect_retry_count_ >= DISCONNECT_MAX_RETRIES ||
                        disconnect_frame_.empty()) {
                        disconnect_retransmit_ms_ = timeout_remaining_ms_;
                    } else if (timeout_remaining_ms_ <= response_window_ms) {
                        // The hard timeout wins ties. Never launch a request that
                        // cannot retain one complete physical response window.
                        LOG_MODEM(INFO,
                                  "Connection: Suppressing late DISCONNECT retry "
                                  "(%ums remain, %ums response window required)",
                                  timeout_remaining_ms_, response_window_ms);
                        disconnect_retry_count_ = DISCONNECT_MAX_RETRIES;
                        disconnect_retransmit_ms_ = timeout_remaining_ms_;
                    } else if ((tx_active_provider_ && tx_active_provider_()) ||
                               (channel_busy_query_ && channel_busy_query_())) {
                        // A due retry is a new half-duplex key-down. Poll without
                        // consuming retry budget rather than transmitting over an
                        // ACK or an already-running local waveform.
                        disconnect_retransmit_ms_ =
                            connection_policy::kCarrierSenseSackCoalesceMs;
                        LOG_MODEM(DEBUG,
                                  "Connection: DISCONNECT retry due but channel/TX "
                                  "busy; polling in %ums without spending budget",
                                  disconnect_retransmit_ms_);
                    } else {
                        disconnect_retry_count_++;
                        LOG_MODEM(INFO, "Connection: Retransmitting DISCONNECT (%d/%d)",
                                  disconnect_retry_count_, DISCONNECT_MAX_RETRIES);
                        transmitFrame(disconnect_frame_);
                        disconnect_retransmit_ms_ = response_window_ms;
                    }
                } else {
                    disconnect_retransmit_ms_ -= elapsed_ms;
                }
            }
            break;

        default:
            break;
    }
}

// =============================================================================
// STATE TRANSITIONS
// =============================================================================

void Connection::transmitFrame(const Bytes& frame_data) {
    if (disconnect_teardown_active_ &&
        !isDisconnectTeardownWireFrame(frame_data)) {
        const auto header = v2::parseHeader(frame_data);
        LOG_MODEM(DEBUG,
                  "Connection: Suppressing %s egress during disconnect teardown",
                  header.valid ? v2::frameTypeToString(header.type) : "invalid frame");
        return;
    }
    LOG_MODEM(DEBUG, "Connection: TX %zu bytes", frame_data.size());
    const bool teardown_control = isDisconnectTeardownWireFrame(frame_data);
    const bool can_expect_peer_reply =
        state_ == ConnectionState::CONNECTED ||
        (state_ == ConnectionState::DISCONNECTING && teardown_control);
    const bool expect_full_anchor_after_tx =
        negotiated_mode_ == WaveformMode::OFDM_CHIRP &&
        can_expect_peer_reply &&
        expectsFullOFDMAnchorAfterTx(frame_data);

    // If burst mode is active, buffer instead of transmitting immediately
    if (burst_mode_active_ && on_transmit_burst_) {
        burst_tx_buffer_.push_back(frame_data);
        return;
    }

    if (on_transmit_info_) {
        on_transmit_info_(frame_data, expect_full_anchor_after_tx);
        return;
    }

    if (on_transmit_) {
        on_transmit_(frame_data);
    }

    if (expect_full_anchor_after_tx && on_full_ofdm_anchor_expected_) {
        on_full_ofdm_anchor_expected_();
    }
}

uint32_t Connection::currentDataFrameAirtimeMs() const {
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        return connection_policy::wideOFDMFrameTiming(
            data_modulation_, data_code_rate_, data_frame_cw_count_).data_ms;
    }
    if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        return connection_policy::narrowOFDMFrameTiming(
            data_modulation_, data_frame_cw_count_).data_ms;
    }
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        return connection_policy::mcDpskFrameTiming(
            data_modulation_,
            config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol,
            data_frame_cw_count_).data_ms;
    }
    return 1000;
}

uint32_t Connection::currentControlFrameAirtimeMs() const {
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        uint32_t control_ms = connection_policy::wideOFDMFrameTiming(
            wideOFDMControlModulationForData(data_modulation_), CodeRate::R1_4).ack_ms;
        if (connection_policy::shouldUseWideOFDMShortReanchor(
                negotiated_mode_, data_modulation_, fading_index_)) {
            control_ms += connection_policy::wideOFDMShortReanchorChirpDurationMs();
        }
        return control_ms;
    }
    if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        return connection_policy::narrowOFDMFrameTiming(
            data_modulation_, data_frame_cw_count_).ack_ms;
    }
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        return connection_policy::mcDpskFrameTiming(
            data_modulation_,
            config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol,
            data_frame_cw_count_).ack_ms;
    }
    return 500;
}

uint32_t Connection::currentBurstAnchorAirtimeMs() const {
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        return connection_policy::kWideOFDMFullAnchorExtraMs;
    }
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        return connection_policy::kMCDPSKDualChirpPreambleMs;
    }
    return 0;
}

uint32_t Connection::connectControlFrameAirtimeMs() const {
    // The GUI/OTASim cold-call MC-DPSK profile is Robust-Mid DBPSK. Forced
    // MC-DPSK profiles can override this, but AUTO should match the waveform
    // engine's real handshake modulation rather than the connected data default.
    Modulation control_mod = Modulation::DBPSK;
    if (config_.forced_modulation == Modulation::DBPSK ||
        config_.forced_modulation == Modulation::DQPSK ||
        config_.forced_modulation == Modulation::D8PSK) {
        control_mod = config_.forced_modulation;
    }

    const int connect_cw_count = v2::kDefaultFixedFrameCodewords;
    const auto timing = connection_policy::mcDpskFrameTiming(
        control_mod,
        config_.mc_dpsk_num_carriers,
        config_.mc_dpsk_samples_per_symbol,
        connect_cw_count);

    const uint64_t airtime_ms =
        static_cast<uint64_t>(connection_policy::mcDpskBurstAirtimeMs(timing, 1)) +
        2ULL * static_cast<uint64_t>(connection_policy::kMCDPSKInterFrameGuardMs);
    return static_cast<uint32_t>(std::min<uint64_t>(airtime_ms, 0xFFFFFFFFull));
}

uint32_t Connection::connectRetryIntervalMs() const {
    const uint64_t control_ms = std::max<uint32_t>(1, connectControlFrameAirtimeMs());
    const uint64_t interval_ms =
        4ULL * control_ms +
        static_cast<uint64_t>(connection_policy::kCarrierSenseSackCoalesceMs);
    return static_cast<uint32_t>(std::min<uint64_t>(interval_ms, 0xFFFFFFFFull));
}

uint32_t Connection::responderHandshakeFailSafeMs() const {
    const uint64_t attempts = static_cast<uint64_t>(
        std::max(1, config_.connect_retries));
    const uint64_t initiator_retry_window_ms =
        attempts * static_cast<uint64_t>(connectRetryIntervalMs()) +
        static_cast<uint64_t>(connectControlFrameAirtimeMs());
    const uint64_t failsafe_ms = std::max<uint64_t>(
        RESPONDER_HANDSHAKE_FAILSAFE_MS,
        initiator_retry_window_ms);
    return static_cast<uint32_t>(std::min<uint64_t>(failsafe_ms, 0xFFFFFFFFull));
}

uint32_t Connection::dataTurnAckDiversityGuardMs(const v2::ControlFrame& ack) const {
    const uint32_t control_airtime_ms = currentControlFrameAirtimeMs();
    uint64_t guard_ms = static_cast<uint64_t>(control_airtime_ms / 2) +
                        connection_policy::kCarrierSenseSackCoalesceMs;

    const auto ack_payload = v2::NackPayload::decode(ack.payload);
    const bool ack_has_final = (ack.flags & v2::Flags::FINAL) != 0;
    const bool guard_half_duplex_repeat =
        ack_payload.cw_bitmap == 0 && !ack_has_final;
    const uint32_t repeat_tail_ms =
        selective_repeat_arq_policy::ackRepeatTailGuardMs(
            control_airtime_ms,
            arq_.getAckRepeatPeerBurstGuardMs(),
            arq_.getAckRepeatDelay(),
            arq_.getAckRepeatCount(),
            guard_half_duplex_repeat);
    if (repeat_tail_ms > 0) {
        guard_ms = std::max<uint64_t>(
            guard_ms,
            static_cast<uint64_t>(repeat_tail_ms) +
                connection_policy::kCarrierSenseSackCoalesceMs);
    }

    return static_cast<uint32_t>(std::min<uint64_t>(
        std::max<uint64_t>(DATA_TURN_ACK_DIVERSITY_GUARD_FLOOR_MS, guard_ms),
        0xFFFFFFFFull));
}

uint32_t Connection::dataTurnConnectGuardMs() const {
    const uint64_t guard_ms =
        static_cast<uint64_t>(currentBurstAnchorAirtimeMs()) +
        static_cast<uint64_t>(currentControlFrameAirtimeMs()) +
        static_cast<uint64_t>(currentDataFrameAirtimeMs() / 2);
    return static_cast<uint32_t>(
        std::max<uint64_t>(DATA_TURN_CONNECT_GUARD_FLOOR_MS, guard_ms));
}

uint32_t Connection::dataTurnControlGuardMs() const {
    const uint64_t guard_ms =
        static_cast<uint64_t>(currentBurstAnchorAirtimeMs()) +
        static_cast<uint64_t>(currentControlFrameAirtimeMs());
    return static_cast<uint32_t>(
        std::max<uint64_t>(DATA_TURN_CONTROL_GUARD_FLOOR_MS, guard_ms));
}

uint32_t Connection::turnRequestHoldoffAfterDataMs() const {
    const uint64_t guard_ms =
        2ULL * static_cast<uint64_t>(dataTurnControlGuardMs()) +
        static_cast<uint64_t>(currentDataFrameAirtimeMs());
    return static_cast<uint32_t>(
        std::max<uint64_t>(TURN_REQUEST_HOLDOFF_FLOOR_MS, guard_ms));
}

uint32_t Connection::turnRequestRetransmitMs() const {
    const uint64_t guard_ms =
        static_cast<uint64_t>(turnRequestHoldoffAfterDataMs()) +
        static_cast<uint64_t>(currentControlFrameAirtimeMs());
    return static_cast<uint32_t>(
        std::max<uint64_t>(TURN_REQUEST_RETRANSMIT_FLOOR_MS, guard_ms));
}

uint32_t Connection::turnRequestAckEmbeddedRetransmitMs() const {
    const uint64_t guard_ms =
        static_cast<uint64_t>(turnRequestRetransmitMs()) +
        static_cast<uint64_t>(dataTurnControlGuardMs());
    return static_cast<uint32_t>(
        std::min<uint64_t>(guard_ms, 0xFFFFFFFFull));
}

uint32_t Connection::fileCancelTxGuardMs() const {
    const uint64_t guard_ms =
        static_cast<uint64_t>(dataTurnControlGuardMs()) +
        2ULL * static_cast<uint64_t>(currentDataFrameAirtimeMs());
    return static_cast<uint32_t>(
        std::max<uint64_t>(FILE_CANCEL_TX_GUARD_FLOOR_MS, guard_ms));
}

uint32_t Connection::fileCancelConfirmDataGuardMs() const {
    const uint64_t guard_ms =
        static_cast<uint64_t>(dataTurnControlGuardMs()) +
        static_cast<uint64_t>(currentDataFrameAirtimeMs());
    return static_cast<uint32_t>(
        std::max<uint64_t>(FILE_CANCEL_CONFIRM_DATA_GUARD_FLOOR_MS, guard_ms));
}

uint32_t Connection::modeChangeRetryMs() const {
    // RATIOMETRIC control-exchange round trip at the CURRENT waveform (2026-07-03):
    // MODE_CHANGE and its ACK are single control frames — anchor + ctl airtime each
    // way + SACK coalesce — NOT a data-burst object. The old code preferred
    // arq_.getAckTimeout(), the unified multi-frame BURST deadline: 3-4x larger and
    // scaling with the ARQ window (worse at window 16). Rig W4 (IONOS MPG@20):
    // retries spaced 18.5 s for a ~5 s exchange — ~74 s of one 328 s transfer burned
    // waiting on MODE_CHANGE alone. dataTurnControlGuardMs derives from the current
    // mode's anchor/control airtimes, so this scales with waveform automatically
    // (MC-DPSK control is slower -> longer timer, by construction).
    // ULTRA_MODE_CHANGE_RETRY_MS [1000..60000] pins it for A/B.
    static const uint32_t env_pin = [] {
        if (const char* e = std::getenv("ULTRA_MODE_CHANGE_RETRY_MS")) {
            const long n = std::atol(e);
            if (n >= 1000 && n <= 60000) return static_cast<uint32_t>(n);
        }
        return 0u;
    }();
    if (env_pin > 0) {
        return env_pin;
    }

    // Airtime-only RTT (2x anchor+ctl + coalesce) under-budgets the real exchange:
    // USB/CoreAudio buffering and decode scheduling are material on a half-duplex
    // control round.  But borrowing arq_.getAckTimeout() is the opposite error: that
    // scalar is sized for the ENTIRE current DATA window (44.75 s at QPSK R2/3,
    // window 16), although MODE_CHANGE and ACK are each one hardened 1-CW control
    // frame. fixed_default_03 measured exactly that coupling: key-up at 317.337 s,
    // retry at 362.098 s, while the peer had sent all ACK copies by 324.931 s.
    const uint64_t control_round_trip_ms =
        2ULL * static_cast<uint64_t>(dataTurnControlGuardMs()) +
        static_cast<uint64_t>(connection_policy::kCarrierSenseSackCoalesceMs);

    // Wide OFDM already has a field-proven one-physical-frame timing policy with
    // the required audio/decode margin and an 8 s floor.  Evaluate that policy at
    // the actual control profile (QPSK/DQPSK R1/4, one frame, one CW), rather than
    // at the current DATA modulation/rate/window.  At the production QPSK control
    // geometry this is 8,000 ms: safely above the observed 3.85-5.06 s key-up->ACK
    // decode paths, yet 5.6x shorter than the unrelated 44.75 s DATA-window scalar.
    uint64_t control_decode_floor_ms = control_round_trip_ms;
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        const Modulation control_mod =
            wideOFDMControlModulationForData(data_modulation_);
        control_decode_floor_ms = connection_policy::computeWideOFDMAckTimeoutMs(
            control_mod,
            CodeRate::R1_4,
            /*window_size=*/1,
            connection_policy::kCarrierSenseSackCoalesceMs,
            /*ack_repeat_count=*/1,
            /*cw_count=*/1);
    } else {
        // Narrow OFDM and MC-DPSK retain their separately-derived DATA deadline;
        // this fix is deliberately scoped to the measured wide-OFDM transaction.
        control_decode_floor_ms =
            std::max<uint64_t>(control_decode_floor_ms, arq_.getAckTimeout());
    }

    // A lost transaction is fade-conditioned.  Space the sender's next REQUEST
    // by at least one coherence interval, capped by the existing 8 s trough-pacing
    // engineering bound so an implausibly slow/noisy estimate cannot recreate the
    // old multi-tens-of-seconds freeze.  Standard Good/Moderate/Poor design Dopplers
    // yield Tc=4230/846/423 ms, all already covered by the 8 s control floor.
    const float retry_doppler_hz = connection_policy::retxTroughDopplerHz(
        coherence_doppler_hz_, fading_index_, coherence_score_, coherence_valid_);
    const uint32_t coherence_ms = connection_policy::coherenceTimeMsForDoppler(
        retry_doppler_hz);
    const uint64_t decorrelation_floor_ms = std::min<uint32_t>(
        coherence_ms, connection_policy::kRetxTroughDeferAbsCapMs);

    return static_cast<uint32_t>(std::min<uint64_t>(
        std::max({control_round_trip_ms,
                  control_decode_floor_ms,
                  decorrelation_floor_ms}),
        0xFFFFFFFFull));
}

uint32_t Connection::disconnectControlKeydownMs() const {
    // Conservative full-control key-down: negotiated full anchor + hardened
    // control waveform, plus the sample-exact ModemEngine lead/tail wrapper.
    // dataTurnControlGuardMs() is millisecond-rounded, so the combined value is a
    // safe upper bound rather than a claim about the final sample count.
    const uint64_t guard_samples = connection_policy::txPostProcessGuardSamples();
    const uint64_t keyed_ms =
        static_cast<uint64_t>(dataTurnControlGuardMs()) +
        connection_policy::sampleDurationCeilMs(guard_samples);
    return static_cast<uint32_t>(std::min<uint64_t>(keyed_ms, 0xFFFFFFFFull));
}

uint32_t Connection::disconnectRetryIntervalMs() const {
    // Scope the measured repair to wide OFDM.  computeWideOFDMAckTimeoutMs()
    // already includes the complete anchored request from queue-time, the ACK
    // path, and decode/audio margin; adding disconnectControlKeydownMs() again
    // would double-count our request airtime.  Do not inherit the MODE_CHANGE env
    // pin: that knob is an A/B override for a different transaction.
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) {
        return DISCONNECT_RETRANSMIT_FLOOR_MS;
    }

    const Modulation control_mod =
        wideOFDMControlModulationForData(data_modulation_);
    const uint64_t physical_deadline_ms =
        connection_policy::computeWideOFDMAckTimeoutMs(
            control_mod, CodeRate::R1_4, /*window_size=*/1,
            connection_policy::kCarrierSenseSackCoalesceMs,
            /*ack_repeat_count=*/1, /*cw_count=*/1);
    const uint64_t control_round_trip_ms =
        2ULL * static_cast<uint64_t>(disconnectControlKeydownMs()) +
        static_cast<uint64_t>(connection_policy::kCarrierSenseSackCoalesceMs);
    const float retry_doppler_hz = connection_policy::retxTroughDopplerHz(
        coherence_doppler_hz_, fading_index_, coherence_score_, coherence_valid_);
    const uint64_t decorrelation_floor_ms = std::min<uint32_t>(
        connection_policy::coherenceTimeMsForDoppler(retry_doppler_hz),
        connection_policy::kRetxTroughDeferAbsCapMs);

    return static_cast<uint32_t>(std::min<uint64_t>(
        std::max({physical_deadline_ms, control_round_trip_ms,
                  decorrelation_floor_ms,
                  static_cast<uint64_t>(DISCONNECT_RETRANSMIT_FLOOR_MS)}),
        0xFFFFFFFFull));
}

uint32_t Connection::disconnectResponderGraceMs() const {
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) {
        return DISCONNECT_GRACE_LEGACY_MS;
    }

    // Stay receptive through every retry that can still retain a complete response
    // window under the initiator's hard timeout.  A due retry may be held by CCA/TX
    // activity beyond its nominal +8/+16 s slot without spending retry budget.  The
    // latest legal queue point is therefore just before (hard_timeout - R), not N*R.
    // Size to the larger boundary, then retain one complete control key-down margin.
    const uint64_t retry_ms = disconnectRetryIntervalMs();
    const uint64_t nominal_last_retry_ms =
        static_cast<uint64_t>(disconnectUsefulRetryCount()) * retry_ms;
    const uint64_t latest_deferred_retry_ms =
        config_.disconnect_timeout_ms > retry_ms
            ? static_cast<uint64_t>(config_.disconnect_timeout_ms) - retry_ms
            : 0ULL;
    const uint64_t grace_ms =
        std::max(nominal_last_retry_ms, latest_deferred_retry_ms) +
        static_cast<uint64_t>(disconnectControlKeydownMs()) +
        static_cast<uint64_t>(connection_policy::kCarrierSenseSackCoalesceMs);
    return static_cast<uint32_t>(std::min<uint64_t>(
        std::max<uint64_t>(grace_ms, DISCONNECT_GRACE_LEGACY_MS),
        0xFFFFFFFFull));
}

uint32_t Connection::disconnectAckRetransmitMs() const {
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) {
        return DISCONNECT_ACK_RETRANSMIT_FLOOR_MS;
    }

    // Never queue a proactive copy before the preceding full ACK can clear the
    // audio play-head. Only two proactive copies are sent; after that the
    // responder stays silently receptive for a duplicate DISCONNECT, avoiding an
    // ACK train phase-locked with the initiator's retry.
    const uint64_t repeat_ms =
        static_cast<uint64_t>(disconnectControlKeydownMs()) +
        static_cast<uint64_t>(connection_policy::kCarrierSenseSackCoalesceMs);
    return static_cast<uint32_t>(std::min<uint64_t>(
        std::max<uint64_t>(repeat_ms, DISCONNECT_ACK_RETRANSMIT_FLOOR_MS),
        0xFFFFFFFFull));
}

int Connection::disconnectUsefulRetryCount() const {
    const uint64_t retry_ms = disconnectRetryIntervalMs();
    uint64_t remaining_ms = config_.disconnect_timeout_ms;
    int useful = 0;
    for (int i = 0; i < DISCONNECT_MAX_RETRIES; ++i) {
        if (remaining_ms <= retry_ms) {
            break;  // timeout wins a tie at the retry boundary
        }
        remaining_ms -= retry_ms;
        if (remaining_ms <= retry_ms) {
            break;  // no complete response window would remain
        }
        ++useful;
    }
    return useful;
}

int Connection::disconnectAckMaxProactiveRepeats() const {
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) {
        return DISCONNECT_ACK_MAX_PROACTIVE_REPEATS_CAP;
    }

    // A delayed proactive copy must finish before the peer can begin its next
    // request.  Budget full anchoring on both sides: a retained full-preamble
    // latch can make the first nominally-light ACK a full control waveform.
    const uint64_t retry_ms = disconnectRetryIntervalMs();
    const uint64_t repeat_ms = disconnectAckRetransmitMs();
    const uint64_t full_keydown_ms = disconnectControlKeydownMs();
    const uint64_t coalesce_ms =
        connection_policy::kCarrierSenseSackCoalesceMs;
    int allowed = 0;
    for (int copies = 1;
         copies <= DISCONNECT_ACK_MAX_PROACTIVE_REPEATS_CAP;
         ++copies) {
        const uint64_t latest_safe_finish_ms =
            static_cast<uint64_t>(copies) * repeat_ms +
            2ULL * full_keydown_ms + coalesce_ms;
        if (latest_safe_finish_ms >= retry_ms) {
            break;
        }
        allowed = copies;
    }
    return allowed;
}

void Connection::scheduleModeChangeAckRepeats(const Bytes& ack_data, uint16_t ack_seq) {
    if (!isOFDMMode(negotiated_mode_)) {
        return;
    }

    // ULTRA_MC_ACK_REPEATS [1..3] pins the full-control ACK-copy count for A/B.
    static const int env_pin = [] {
        if (const char* e = std::getenv("ULTRA_MC_ACK_REPEATS")) {
            const int n = std::atoi(e);
            if (n >= 1 && n <= 3) return n;
        }
        return 0;
    }();
    // Default to one immediate ACK.  The old fading default sent copies 2/3 at
    // roughly 0.25/0.83 s; fixed_default_03 measured fD=0.181 Hz (Tc=2.34 s),
    // so all three occupied the same fade and all three were lost together.  The
    // pending sender's 8 s CCA-gated REQUEST retry now supplies real time diversity,
    // and the existing duplicate path returns exactly one fresh ACK.  Keeping late
    // receiver-side copies would also risk keying over DATA after copy 1 succeeded.
    // ULTRA_MC_ACK_REPEATS remains available for explicit A/B experiments.
    const int repeat_count = env_pin > 0 ? env_pin : 1;
    if (repeat_count <= 1) {
        return;
    }

    for (auto it = mode_change_ack_repeat_jobs_.begin();
         it != mode_change_ack_repeat_jobs_.end();) {
        if (it->seq == ack_seq) {
            it = mode_change_ack_repeat_jobs_.erase(it);
        } else {
            ++it;
        }
    }

    constexpr uint32_t kAckPayloadBitmap = 0;
    for (int copy_index = 2; copy_index <= repeat_count; ++copy_index) {
        const uint32_t base_delay_ms =
            selective_repeat_arq_policy::ackRepeatDelayForCopy(
                arq_.getAckRepeatDelay(), copy_index);
        const int jitter_ms = selective_repeat_arq_policy::ackRepeatJitterMs(
            ack_seq, kAckPayloadBitmap, copy_index);
        int64_t scheduled_ms = static_cast<int64_t>(base_delay_ms) + jitter_ms;
        if (scheduled_ms < 1) {
            scheduled_ms = 1;
        }

        ModeChangeAckRepeatJob job;
        job.frame_data = ack_data;
        job.seq = ack_seq;
        job.timer_ms = static_cast<uint32_t>(scheduled_ms);
        job.copy_index = copy_index;
        mode_change_ack_repeat_jobs_.push_back(std::move(job));

        LOG_MODEM(INFO,
                  "Connection: MODE_CHANGE ACK_REPEAT scheduled seq=%u copy=%d delay=%ums jitter=%dms",
                  static_cast<unsigned>(ack_seq),
                  copy_index,
                  static_cast<unsigned>(scheduled_ms),
                  jitter_ms);
    }
}

void Connection::tickModeChangeAckRepeats(uint32_t elapsed_ms) {
    for (auto it = mode_change_ack_repeat_jobs_.begin();
         it != mode_change_ack_repeat_jobs_.end();) {
        ModeChangeAckRepeatJob& job = *it;
        if (elapsed_ms >= job.timer_ms) {
            LOG_MODEM(INFO,
                      "Connection: MODE_CHANGE ACK_REPEAT sent seq=%u copy=%d",
                      static_cast<unsigned>(job.seq),
                      job.copy_index);
            transmitFrame(job.frame_data);
            it = mode_change_ack_repeat_jobs_.erase(it);
            continue;
        }

        job.timer_ms -= elapsed_ms;
        ++it;
    }
}

void Connection::configureArqForCurrentDataMode() {
    arq_.setCodeRate(data_code_rate_);
    const int physical_data_cw = physicalDataFrameCodewords();
    const int physical_data_z = selectBurstLiftingZ();
    // Fixed DATA serialization and the PHY descriptor must consume the same
    // (cw, Z) tuple. Passing only cw here used makeFixedDataFrame's default
    // Z=27, silently truncating cw4/Z81 file chunks from 629 to 216 bytes while
    // BURST_HEADER correctly announced Z=81 to the receiver.
    arq_.setFixedFrameGeometry(physical_data_cw, physical_data_z);
    arq_.setAckBatchThroughMoreFrag(false);
    arq_.setSackDelaySlidesOnData(false);

    if (isOFDMMode(negotiated_mode_) || usesBoundedVariableMCDPSKFrames()) {
        file_transfer_.setMaxChunkPayload(currentDataPayloadCapacity());
    }

    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        const auto timing = connection_policy::mcDpskFrameTiming(
            data_modulation_,
            config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol,
            data_frame_cw_count_);
        const size_t window_size = connection_policy::mcDpskWindowSizeForTiming(timing);
        arq_.setWindowSize(window_size);
        arq_.setSackDelay(connection_policy::kCarrierSenseSackCoalesceMs);
        arq_.setSackDelayShort(0);
        arq_.setAckBatchThroughMoreFrag(true);
        arq_.setImmediateOutOfOrderSackEnabled(true);
        arq_.setAckRepeatCount(connection_policy::kCarrierSenseAckRepeatCount);
        arq_.setAckRepeatPeerBurstGuardMs(arq_.getSackDelay());
        // BUG-MCDPSK-ACK-COLLISION: the tone-burst partial (hole-bearing) SACK fires this
        // long after the last decoded out-of-order frame. It MUST exceed one MC-DPSK frame
        // airtime (~timing.data_ms), else it lands while the sender is still transmitting a
        // trailing failed frame of the same ~18.7 s window burst -> half-duplex collision ->
        // the sender never hears the NACK -> RTO whole-window resend -> phase-locked
        // livelock -> disconnect. One frame airtime + a T/R/decode margin lands the SACK in
        // the inter-burst gap. Floored at the 1500 ms OFDM default; stays well under the
        // ~31.6 s ACK RTO. (Carrier-sense — defer until the channel is heard idle — is the
        // fully radio-correct generalization; this airtime-scaled guard is the targeted fix.)
        // Compute the receiver tone-burst partial-SACK hold ONCE and feed it to BOTH the
        // receiver (setToneBurstPartialSackDelayMs) AND the sender's ACK RTO budget
        // (computeMCDPSKAckTimeoutMs) — otherwise the sender's deadline omits the hold the
        // receiver actually applies, times out before the ACK round-trip completes, and
        // blind-resends the whole window (BUG-MCDPSK-FILE-COMPLETION: the resulting doubled
        // airtime means the FINAL file chunk is never reached in-session -> never finalizes).
        // Previously the RTO was passed arq_.getSackDelay() = 30 ms carrier-sense coalesce,
        // NOT this ~6.4 s hold.
        const uint32_t sack_hold_ms =
            std::max<uint32_t>(1500u, timing.data_ms + 1000u);
        arq_.setToneBurstPartialSackDelayMs(sack_hold_ms);
        uint32_t ack_timeout_ms = connection_policy::computeMCDPSKAckTimeoutMs(
            timing, window_size, sack_hold_ms,
            connection_policy::kCarrierSenseAckRepeatCount);
        if (data_modulation_ == Modulation::DBPSK &&
            config_.mc_dpsk_samples_per_symbol >= 2048) {
            ack_timeout_ms = std::max<uint32_t>(
                ack_timeout_ms, connection_policy::kMCDPSKRobustLowAckTimeoutFloorMs);
        }
        arq_.setAckTimeout(ack_timeout_ms);
        LOG_MODEM(INFO, "Connection: ARQ window=%zu, timeout=%.1fs (data=%ums, ack=%ums x%d), carrier_sense_sack_coalesce=%ums, cw=%d (MC-DPSK %s %s, carriers=%d, sps=%d)",
                  window_size,
                  ack_timeout_ms / 1000.0f,
                  timing.data_ms,
                  timing.ack_ms,
                  connection_policy::kCarrierSenseAckRepeatCount,
                  arq_.getSackDelay(),
                  data_frame_cw_count_,
                  modulationToString(data_modulation_),
                  codeRateToString(data_code_rate_),
                  config_.mc_dpsk_num_carriers,
                  config_.mc_dpsk_samples_per_symbol);
    } else if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        // Selective-repeat window=3 — chosen after A/B in cli_simulator
        // SNR=8 good fading R1/4 7-message test:
        //   window=1 (was): 180 s wall-clock
        //   window=2:        116 s (-36 %)
        //   window=3:         92 s (-49 %, +96 % throughput)
        //   window=4:         69 s (-62 %)
        //   window=8:         87 s (diminishing returns)
        // Settled on 3 because Codex audit explicitly capped at "2, maybe
        // 3 after tests"; window=4 hasn't been audited even though it
        // also passes every documented baseline. 30 m fading coherence
        // on real channels could correlate across a 4-frame burst (each
        // frame ~3.4 s) and turn one fade into 4 retransmits. If real-
        // OTA testing later shows we have headroom, bump to 4 after a
        // fresh audit. If correlated fades chew throughput, drop to 2.
        constexpr size_t kNarrowWindow = 3;
        arq_.setWindowSize(kNarrowWindow);
        arq_.setMaxRetries(15);
        arq_.setSackDelay(connection_policy::kCarrierSenseSackCoalesceMs);
        arq_.setSackDelayShort(0);
        arq_.setImmediateOutOfOrderSackEnabled(true);
        arq_.setAckRepeatCount(connection_policy::kCarrierSenseAckRepeatCount);
        arq_.setAckRepeatPeerBurstGuardMs(arq_.getSackDelay());

        const auto timing = connection_policy::narrowOFDMFrameTiming(
            data_modulation_, data_frame_cw_count_);
        uint32_t timeout_ms = connection_policy::computeNarrowOFDMAckTimeoutMs(
            data_modulation_, data_frame_cw_count_, kNarrowWindow, arq_.getSackDelay());
        arq_.setAckTimeout(timeout_ms);

        LOG_MODEM(INFO, "Connection: ARQ window=%zu, timeout=%.2fs (data=%ums, ack=%ums), carrier_sense_sack_coalesce=%ums, ack_repeat=%d, cw=%d (OFDM_NARROW %s %s)",
                  kNarrowWindow, timeout_ms / 1000.0f, timing.data_ms, timing.ack_ms,
                  arq_.getSackDelay(),
                  connection_policy::kCarrierSenseAckRepeatCount,
                  data_frame_cw_count_,
                  modulationToString(data_modulation_), codeRateToString(data_code_rate_));
    } else {
        // #58 increment 3: selection-flavored consumers (knob-gated pool aggregate) —
        // a lone trough snapshot could deny the window-16 gate for the whole session.
        const float window_snr_db = rateSelectionSnrDb();
        const bool near_awgn_ofdm =
            connection_policy::isNearAwgnOFDM(fading_index_, window_snr_db);
        arq_.setWindowSize(connection_policy::ofdmWindowSizeForChannel(
            data_modulation_, data_code_rate_, fading_index_, window_snr_db));
        // TRANSPORT MERGE (step 1): the tone-burst ack carries a 16-bit frame_mask (widened
        // 6->8 2026-06-17, 8->16 2026-07-02), so cap the in-flight window to 16. An N-frame
        // message then streams as ≤16-frame windows, each fully covered by one tone-burst
        // snapshot — no mask truncation, no spurious resend of frames past the mask. (MC-DPSK
        // 1-5 and OFDM_NARROW 3 are already within it.) The timing math below then sizes
        // timeouts for the capped window.
        if (kInteractiveToneAckEnabled() &&
            arq_.getWindowSize() > connection_policy::kToneBurstAckWindowCapFrames) {
            LOG_MODEM(INFO, "Connection: capped ARQ window %zu -> %zu (tone-burst 16-bit mask)",
                      arq_.getWindowSize(),
                      connection_policy::kToneBurstAckWindowCapFrames);
            arq_.setWindowSize(connection_policy::kToneBurstAckWindowCapFrames);
        }
        arq_.setMaxRetries(15);
        arq_.setAckBatchSize(connection_policy::ofdmAckBatchSize(near_awgn_ofdm));

        const auto timing = connection_policy::wideOFDMFrameTiming(
            data_modulation_, data_code_rate_, physical_data_cw,
            physical_data_z);
        const auto control_timing = connection_policy::wideOFDMFrameTiming(
            wideOFDMControlModulationForData(data_modulation_), CodeRate::R1_4);
        const bool adaptive_short_reanchor =
            connection_policy::shouldUseWideOFDMShortReanchor(
                negotiated_mode_, data_modulation_, fading_index_);
        const uint32_t continuation_reanchor_ms =
            adaptive_short_reanchor
                ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
                : 0;
        const uint32_t control_ack_airtime_ms =
            control_timing.ack_ms + continuation_reanchor_ms;
        const uint32_t burst_airtime_ms = connection_policy::wideOFDMBurstAirtimeMs(
            data_modulation_, data_code_rate_, arq_.getWindowSize(),
            physical_data_cw, continuation_reanchor_ms, physical_data_z);
        // ACK diversity (repeat the ack N times) protects a single fade-lost ack on a
        // SACK-frame path. The tone-burst group-ack is different: it fires ONE prompt ack
        // per received burst, and a lost ack is already backstopped by the sender's ARQ
        // retransmit (which re-sends the group → the receiver re-acks). Repeating it just
        // keys the receiver down for ~5 s of redundant acks (deaf to the sender that whole
        // time) — the "4-5 ack chain" the operator saw. So: ONE ack on the tone-burst path.
        const int kWideOFDMAckRepeatCount = kInteractiveToneAckEnabled() ? 1 : 3;
        const uint32_t physical_sack_hold_ms = burst_airtime_ms +
            connection_policy::kCarrierSenseSackCoalesceMs;
        const uint64_t sliding_sack_delay_ms =
            static_cast<uint64_t>(timing.data_ms) + timing.ack_ms +
            connection_policy::kCarrierSenseSackCoalesceMs;
        const uint32_t sack_delay_ms = static_cast<uint32_t>(
            std::min<uint64_t>(sliding_sack_delay_ms, UINT32_MAX));
        arq_.setSackDelay(adaptive_short_reanchor ? physical_sack_hold_ms : sack_delay_ms);
        arq_.setSackDelayShort(connection_policy::wideOFDMSackTailDelayMs());
        arq_.setSackDelaySlidesOnData(!adaptive_short_reanchor);
        arq_.setImmediateOutOfOrderSackEnabled(!adaptive_short_reanchor);
        arq_.setAckRepeatCount(kWideOFDMAckRepeatCount);
        arq_.setAckRepeatPeerBurstGuardMs(
            adaptive_short_reanchor ? 0 : arq_.getSackDelay());
        arq_.setAckRepeatDelay(ackRepeatDelayForControlAirtimeMs(control_ack_airtime_ms));

        uint32_t ack_timeout_ms = physical_data_z == 81
            ? connection_policy::unifiedBurstAckTimeoutMs(
                  data_modulation_, data_code_rate_, physical_data_cw,
                  arq_.getWindowSize(), physical_data_z,
                  wideOFDMControlModulationForData(data_modulation_),
                  arq_.getSackDelay(), continuation_reanchor_ms)
            : connection_policy::computeWideOFDMAckTimeoutMs(
                  data_modulation_, data_code_rate_, arq_.getWindowSize(),
                  arq_.getSackDelay(), kWideOFDMAckRepeatCount,
                  physical_data_cw, continuation_reanchor_ms);
        const uint32_t ack_repeat_tail_ms =
            selective_repeat_arq_policy::ackRepeatTailGuardMs(
                control_ack_airtime_ms,
                arq_.getAckRepeatPeerBurstGuardMs(),
                arq_.getAckRepeatDelay(),
                kWideOFDMAckRepeatCount,
                true);
        const uint32_t decode_jitter_margin_ms =
            std::max<uint32_t>(700, timing.data_ms / 2) + 700;
        const uint64_t repeat_covered_timeout_ms =
            static_cast<uint64_t>(burst_airtime_ms) +
            static_cast<uint64_t>(physical_sack_hold_ms) +
            static_cast<uint64_t>(ack_repeat_tail_ms) +
            static_cast<uint64_t>(decode_jitter_margin_ms);
        if (adaptive_short_reanchor) {
            ack_timeout_ms = std::max<uint32_t>(
                ack_timeout_ms,
                static_cast<uint32_t>(
                    std::min<uint64_t>(repeat_covered_timeout_ms, 0xFFFFFFFFull)));
        }
        arq_.setAckTimeout(ack_timeout_ms);

        // ULTRA_INFLIGHT_RTO: the SAME timeout evaluated at every frame count 1..window,
        // so the ARQ can size a retransmit on what is ACTUALLY outstanding instead of on
        // the window maximum. Built from the identical policy calls used for the scalar
        // above, including the adaptive_short_reanchor repeat-covered floor, so entry
        // [window] reproduces ack_timeout_ms exactly — logged as an identity check.
        {
            const size_t win = std::min<size_t>(
                arq_.getWindowSize(), selective_repeat_arq_policy::kMaxWindow);
            uint32_t table[selective_repeat_arq_policy::kMaxWindow + 1] = {};
            for (size_t n = 1; n <= win; ++n) {
                const uint32_t burst_n = connection_policy::wideOFDMBurstAirtimeMs(
                    data_modulation_, data_code_rate_, n, physical_data_cw,
                    continuation_reanchor_ms, physical_data_z);
                const uint32_t sack_hold_n = burst_n +
                    connection_policy::kCarrierSenseSackCoalesceMs;
                uint32_t t = physical_data_z == 81
                    ? connection_policy::unifiedBurstAckTimeoutMs(
                          data_modulation_, data_code_rate_, physical_data_cw, n,
                          physical_data_z,
                          wideOFDMControlModulationForData(data_modulation_),
                          adaptive_short_reanchor ? sack_hold_n : sack_delay_ms,
                          continuation_reanchor_ms)
                    : connection_policy::computeWideOFDMAckTimeoutMs(
                          data_modulation_, data_code_rate_, n,
                          adaptive_short_reanchor ? sack_hold_n : sack_delay_ms,
                          kWideOFDMAckRepeatCount, physical_data_cw,
                          continuation_reanchor_ms);
                if (adaptive_short_reanchor) {
                    const uint64_t covered_n =
                        static_cast<uint64_t>(burst_n) +
                        static_cast<uint64_t>(sack_hold_n) +
                        static_cast<uint64_t>(ack_repeat_tail_ms) +
                        static_cast<uint64_t>(decode_jitter_margin_ms);
                    t = std::max<uint32_t>(
                        t, static_cast<uint32_t>(
                               std::min<uint64_t>(covered_n, 0xFFFFFFFFull)));
                }
                table[n] = t;
            }
            arq_.setAckTimeoutTable(table, win + 1);
            LOG_MODEM(INFO,
                      "Connection: INFLIGHT-RTO table n=1..%zu -> %ums..%ums "
                      "(scalar=%ums; identity at n=window %s)",
                      win, table[1], table[win], ack_timeout_ms,
                      (table[win] == ack_timeout_ms) ? "OK" : "MISMATCH");
        }

        LOG_MODEM(INFO,
                  "Connection: ARQ window=%zu, timeout=%.2fs (data=%ums, burst=%ums, ack=%ums/control=%ums x%d), max_retries=%d, ack_batch=%u, sack_delay=%ums, sack_slides=%d, physical_sack_hold=%ums, tail_sack=%ums, ack_repeat=%d, ack_repeat_delay=%ums, ack_repeat_guard=%ums, cw=%d, z=%d, continuation_reanchor=%ums (OFDM %s %s)",
                  arq_.getWindowSize(),
                  ack_timeout_ms / 1000.0f,
                  timing.data_ms,
                  burst_airtime_ms,
                  timing.ack_ms,
                  control_ack_airtime_ms,
                  kWideOFDMAckRepeatCount,
                  arq_.getMaxRetries(),
                  arq_.getAckBatchSize(),
                  arq_.getSackDelay(),
                  arq_.getSackDelaySlidesOnData() ? 1 : 0,
                  physical_sack_hold_ms,
                  arq_.getSackDelayShort(),
                  kWideOFDMAckRepeatCount,
                  arq_.getAckRepeatDelay(),
                  arq_.getAckRepeatPeerBurstGuardMs(),
                  physical_data_cw,
                  physical_data_z,
                  continuation_reanchor_ms,
                  modulationToString(data_modulation_),
                  codeRateToString(data_code_rate_));
    }

    configureSoftCombineHARQBounds();
}

void Connection::configureSoftCombineHARQBounds() {
    const size_t max_entries = arq_.getWindowSize() *
        static_cast<size_t>(v2::sanitizeFixedFrameCodewords(
            physicalDataFrameCodewords()));
    soft_combine_harq_.setMaxEntries(max_entries);
}

uint32_t Connection::pingTimeoutMsForCurrentProfile() const {
    // DBPSK MC-DPSK PING/PONG detection has to wait through the same slower
    // training/ref energy check as CONNECT detection. Keeping the
    // standard timer unchanged avoids slowing normal retries while preventing
    // robust DBPSK probe retries from overlapping the following CONNECT.
    const bool robust_dbpsk_probe =
        connect_waveform_ == WaveformMode::MC_DPSK &&
        (config_.forced_modulation == Modulation::DBPSK ||
         config_.mc_dpsk_samples_per_symbol >= 1024);
    return robust_dbpsk_probe ? ROBUST_LOW_PING_TIMEOUT_MS : PING_TIMEOUT_MS;
}

bool Connection::usesBoundedVariableMCDPSKFrames() const {
    return negotiated_mode_ == WaveformMode::MC_DPSK;
}

OFDMDataWireProfile Connection::resolveExperimentalLongLDPCWireProfileFor(
    Modulation mod, CodeRate rate, int logical_cw,
    bool psk8_long_enabled, bool qpsk_r34_long_enabled) const {
    OFDMDataWireProfile profile{logical_cw, logical_cw, 27};

    // This is the shared sender/receiver contract.  The sender passes the
    // transfer-scoped arms captured by startFileTransferNow(); the receiver has no
    // access to those peer-owned booleans and passes its matching local experiment
    // policy while pricing the next rate command.  BURST_HEADER remains authoritative
    // once the peer actually emits the next group.
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP ||
        !use_burst_transport_ || config_.forced_cw_count != 0) {
        return profile;
    }
    if (psk8_long_enabled && mod == Modulation::QAM8 &&
        rate == CodeRate::R2_3 && logical_cw == 12) {
        profile.physical_cw = 4;
        profile.lifting_z = 81;
    } else if (qpsk_r34_long_enabled && mod == Modulation::QPSK &&
               rate == CodeRate::R3_4 && logical_cw == 8) {
        profile.physical_cw = 3;
        profile.lifting_z = 81;
    }
    return profile;
}

bool Connection::usesExperimental8PSKLongLDPCFor(
    Modulation mod, CodeRate rate, int logical_cw) const {
    // Exact capacity/airtime substitution only: 12 short codewords and four
    // long codewords both carry 7776 coded bits, and at R2/3 both expose the
    // same 648 information bytes before the fixed-frame overhead.  Do not
    // silently reinterpret an operator-forced or coherence-shortened CW count.
    return resolveExperimentalLongLDPCWireProfileFor(
               mod, rate, logical_cw,
               experimental_8psk_long_ldpc_transfer_active_,
               /*qpsk_r34_long_enabled=*/false).lifting_z == 81;
}

bool Connection::usesExperimental8PSKLongLDPC() const {
    return usesExperimental8PSKLongLDPCFor(
        data_modulation_, data_code_rate_, data_frame_cw_count_);
}

bool Connection::usesExperimentalQPSKR34LongLDPCFor(
    Modulation mod, CodeRate rate, int logical_cw) const {
    // The production QPSK R3/4 logical frame is cw8/Z27 (5184 coded bits,
    // 480 information bytes before fixed-frame overhead). The experimental
    // physical representation is cw3/Z81 (5832 coded bits, 546 information
    // bytes). This deliberately buys a 12.5% longer coded frame; it is not the
    // equal-airtime cw9/Z27 laboratory comparison. Never reinterpret an
    // operator-forced or coherence-shortened logical geometry.
    return resolveExperimentalLongLDPCWireProfileFor(
               mod, rate, logical_cw,
               /*psk8_long_enabled=*/false,
               experimental_qpsk_r34_long_ldpc_transfer_active_).lifting_z == 81;
}

bool Connection::usesExperimentalQPSKR34LongLDPC() const {
    return usesExperimentalQPSKR34LongLDPCFor(
        data_modulation_, data_code_rate_, data_frame_cw_count_);
}

bool Connection::usesExperimentalLongLDPCFor(
    Modulation mod, CodeRate rate, int logical_cw) const {
    return resolveExperimentalLongLDPCWireProfileFor(
               mod, rate, logical_cw,
               experimental_8psk_long_ldpc_transfer_active_,
               experimental_qpsk_r34_long_ldpc_transfer_active_).lifting_z == 81;
}

bool Connection::usesExperimentalLongLDPC() const {
    return usesExperimentalLongLDPCFor(
        data_modulation_, data_code_rate_, data_frame_cw_count_);
}

int Connection::physicalDataFrameCodewordsFor(
    Modulation mod, CodeRate rate, int logical_cw) const {
    return resolveExperimentalLongLDPCWireProfileFor(
               mod, rate, logical_cw,
               experimental_8psk_long_ldpc_transfer_active_,
               experimental_qpsk_r34_long_ldpc_transfer_active_).physical_cw;
}

int Connection::physicalDataFrameCodewords() const {
    return physicalDataFrameCodewordsFor(
        data_modulation_, data_code_rate_, data_frame_cw_count_);
}

LatentRateCandidateGeometry Connection::latentRateCandidateGeometryFor(
    Modulation mod, CodeRate rate, bool force_full_group_start) const {
    LatentRateCandidateGeometry out;
    out.force_full_group_start = force_full_group_start;
    out.logical_cw = connection_policy::receiverRateCommandCandidateCWCount(
        mod, rate, WaveformMode::OFDM_CHIRP, config_.forced_cw_count);

    // The latent selector runs on the receiver.  The peer's transfer-scoped arms
    // exist only in its Connection, so using experimental_*_transfer_active_ here
    // would silently price Z27 forever.  The long-profile campaign requires matching
    // endpoint policy (and the runner verifies env parity); use that exact local,
    // default-off policy for the counterfactual.  The following descriptor still
    // proves the physical tuple before its outcome is consumed.
    const OFDMDataWireProfile wire = resolveExperimentalLongLDPCWireProfileFor(
        mod, rate, out.logical_cw,
        psk8LongLdpcExperimentEnabled(),
        qpskR34LongLdpcExperimentEnabled());
    out.physical_cw = wire.physical_cw;
    out.lifting_z = wire.lifting_z;

    const size_t arq_payload = v2::getFixedFramePayloadCapacityZ(
        rate, out.physical_cw, out.lifting_z);
    if (arq_payload <= FileTransferController::FILE_DATA_OVERHEAD) {
        return out;
    }
    const size_t candidate_window = connection_policy::ofdmWindowSize(
        mod, rate, /*near_awgn_ofdm=*/false);
    // The sender's clean-ACK streak is private state.  In particular, a receiver
    // may decode the same clean group repeatedly while every ACK is lost, so its
    // local clean-group count is not a lower bound on the sender's count.  Price
    // the guaranteed base ceiling; the sender remains free to exploit its own
    // delivery-proven escalation when it has actually heard enough clean ACKs.
    const uint32_t candidate_ceiling = connection_policy::burstAirtimeCeilingMs(
        mod, rate, /*sender_clean_group_streak=*/0);
    out.frames_per_cycle = connection_policy::wideOFDMBurstFrameBudget(
        mod, rate, out.physical_cw, candidate_window, candidate_ceiling,
        /*continuation_reanchor_ms=*/0, out.lifting_z,
        force_full_group_start
            ? connection_policy::kWideOFDMFullAnchorExtraMs
            : 0u);
    out.burst_airtime_ms = connection_policy::wideOFDMBurstAirtimeMs(
        mod, rate, out.frames_per_cycle, out.physical_cw,
        /*continuation_reanchor_ms=*/0, out.lifting_z);
    if (force_full_group_start && out.frames_per_cycle > 1) {
        out.burst_airtime_ms += connection_policy::kWideOFDMFullAnchorExtraMs;
    }

    constexpr uint32_t kMedianNonDataCycleMs = 1790;
    out.value = {
        static_cast<float>(arq_payload - FileTransferController::FILE_DATA_OVERHEAD),
        static_cast<int>(out.frames_per_cycle),
        static_cast<float>(out.burst_airtime_ms + kMedianNonDataCycleMs) / 1000.0f,
    };
    return out;
}

void Connection::setExperimentalLongLDPCTransferProfilesActive(
    bool psk8_active, bool qpsk_r34_active) {
    if (experimental_8psk_long_ldpc_transfer_active_ == psk8_active &&
        experimental_qpsk_r34_long_ldpc_transfer_active_ == qpsk_r34_active) {
        return;
    }
    experimental_8psk_long_ldpc_transfer_active_ = psk8_active;
    experimental_qpsk_r34_long_ldpc_transfer_active_ = qpsk_r34_active;

    // The negotiated logical geometry remains unchanged. Re-grid only the
    // physical ARQ/chunker/timer view while the file owns the link, then put it
    // back before the application observes transfer completion/failure/cancel.
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        configureArqForCurrentDataMode();
    }
    LOG_MODEM(INFO,
              "Connection: experimental long-LDPC transfer profiles "
              "8PSK=%s QPSK-R3/4=%s active_matching_rung=%s "
              "(logical cw=%d, physical cw=%d, z=%d)",
              psk8_active ? "armed" : "off",
              qpsk_r34_active ? "armed" : "off",
              usesExperimentalLongLDPC() ? "yes" : "no",
              data_frame_cw_count_, physicalDataFrameCodewords(),
              selectBurstLiftingZ());
}

void Connection::appendExperimentalLongLDPCTailPad(
    std::vector<Bytes>& frames) const {
    if (!usesExperimentalLongLDPC() || !on_transmit_burst_ ||
        frames.size() != 1) {
        return;
    }

    // A singleton normally takes on_transmit_ and has no BURST_HEADER, so the
    // receiver cannot learn Z=81. Reuse the existing addressed-away ULPAD
    // frame to make a two-frame physical group. encodeBurstLight then stamps
    // PHYSICAL_BURST_END on this pad (not on the logical FINAL DATA), emits the
    // descriptor, and the receiver's callsign filter keeps the pad out of ARQ.
    const int cw = physicalDataFrameCodewords();
    auto pad = v2::makeFixedDataFrame(
        local_call_, v2::kOFDMBurstPadCallsign, v2::kOFDMBurstPadSeq,
        makeOFDMBurstPadPayload(data_code_rate_, cw, /*pad_index=*/0, /*z=*/81),
        data_code_rate_, cw, /*lifting_z=*/81).serialize();
    frames.push_back(std::move(pad));
    LOG_MODEM(INFO,
              "Connection: padded experimental Z=81 singleton 1 -> 2 frames "
              "so BURST_HEADER can announce cw=%d/z=81",
              cw);
}

int Connection::selectBurstLiftingZFor(
    Modulation mod, CodeRate rate, int logical_cw) const {
    if (usesExperimentalLongLDPCFor(mod, rate, logical_cw)) {
        return 81;
    }
    // Traffic-class policy (C-policy): long LDPC (Z=81, n=1944) for bulk/file OFDM
    // bursts — a 1944-bit codeword spans ~1.8x the coherence interval at 0.1 Hz
    // Doppler, buying fade diversity where latency is free. Short (Z=27, n=648) for
    // control, interactive messages, and MC-DPSK (fast ACK turnaround).
    //
    // GATED ON use_burst_transport_: long LDPC needs the burst-group machinery —
    // the BURST_HEADER descriptor announces BOTH Z and the independently-selected
    // physical CW count. A descriptor-less SelectiveRepeatARQ file path must stay
    // Z=27 because the RX otherwise cannot infer either physical geometry. The
    // app/cli pushes this Z to the TX encoder so its Z matches the chunker.
    // See docs/LDPC_Z_DERIVATION_DESIGN_2026_05_30.md.
    //
    // TRANSPORT MERGE (increment 1, ULTRA_UNIFIED_SEQ): the unified path sends the
    // file as REGULAR arq_ DATA frames (sendNextFileChunk → flushBurstBuffer), NOT
    // through burst_transport_'s long-LDPC group machinery. Those frames are
    // serialized declaring cw=data_frame_cw_count_ at the connection's data geometry;
    // forcing z=81 without rewriting the fixed DATA header and capacity together
    // would make the announced physical geometry disagree with the serialized frame.
    // Keeping Z at the DEFAULT 27 makes z/cw consistent end-to-end AND makes the
    // descriptor's z=27 itself the on-wire "regular frames, not a file" signal the RX
    // routes on (operator's keystone — no extra descriptor bit needed).
    //
    // The former ULTRA_LDPC_Z=81 override deliberately does NOT live above this
    // gate anymore. It could select long LDPC for descriptor-less singleton DATA:
    // the connection would size/serialize at Z=81 while ModemEngine correctly
    // forced that unannounced transmission back to Z=27. Long LDPC experiments
    // must use a scoped profile such as ULTRA_8PSK_LONG_LDPC or
    // ULTRA_QPSK_R34_LONG_LDPC, which guarantees a BURST_HEADER and changes
    // serializer, timing, and PHY from one policy tuple.
    if (isOFDMMode(negotiated_mode_) &&
        use_burst_transport_ &&
        !kUnifiedSeqEnabled() &&
        file_transfer_.getState() == FileTransferState::SENDING) {
        return 81;
    }
    return 27;
}

int Connection::selectBurstLiftingZ() const {
    return selectBurstLiftingZFor(
        data_modulation_, data_code_rate_, data_frame_cw_count_);
}

size_t Connection::burstAirtimeBudgetFrames(
    size_t max_frames,
    bool force_full_group_start) const {
    if (max_frames <= 1) {
        return std::max<size_t>(1, max_frames);
    }
    // Soft half-duplex airtime ceiling for ONE key-down. A real 100 W PA derates on
    // long key-downs and the T/R turnaround must catch air, so a single burst can't run
    // arbitrarily long; it also must not outlive its own tone-burst ack window. SOFT:
    // we just want "not ~10 s straight" — a burst that lands a touch over the nominal
    // 6 s (e.g. one more frame at ~6.8 s) is fine. This is the ONLY fixed number — the
    // frame count is DERIVED from the live per-frame airtime below, so it adapts across
    // modulation (bits/carrier), code rate (pilot spacing — N=648 coded bits is
    // rate-invariant, only pilots move), cw count, and fading (re-anchor) by construction.
    //
    // GROUP SIZE is this airtime ceiling for ONE key-down; the frame count is DERIVED from the
    // live per-frame airtime above. Default 8600 ms = a 5-frame group at the nominal z=27
    // QPSK-R2/3-cw8 rung (1392 ms/frame + 1200 ms dual-chirp anchor: 5 frames = 8560 ms <= 8600;
    // a 6th would be 10052 ms). Codified from a 20-seed Good@16 sweep (2026-06-07): groups 5 and
    // 6 TIE on goodput (~1400 bps, within run-to-run noise) and both deliver reliably with the
    // full-chirp-on-resend fix (maxretry=0; the two genuine failures recovered) — so the smaller
    // group wins on NON-speed grounds: shorter 8.6 s key-down (easier on a real PA than group 6's
    // ~10 s), fewer frames lost per fade, and well below the 16-bit SACK frame_mask ceiling (raised
    // 6->8 on 2026-06-17, 8->16 on 2026-07-02, so thin-frame cw5 bursts can fill the budget — a cw8 burst stays
    // airtime-bound at 5 frames regardless). Replaces the 3-frame 7000 ms default that re-paid the 1.2 s anchor +
    // turnaround every 3 frames. Still env-overridable for sweeps (clamped [5000, 12000]).
    // GROUP-SIZE LEVER (2026-07-07, docs/GROUP_SIZE_LEVER_2026_07_07.md §2.3):
    // STREAK-GATED ceiling escalation. After 2 consecutive CLEAN groups at the
    // current rung (full retire, no holes — burst_clean_group_streak_), the
    // ceiling rises to 11,500 ms → N=8 at the duration-normalized rungs
    // (1200 + 8×1272 = 11,376 ≤ 11,500 < 12,648 for the 9th). Economics
    // (measured constants): cycle efficiency 64.7 % → 74.6 %, fade-taxed
    // ≈ +11.6 %; N=8 never loses across the observed 5-20 % group-loss range.
    // Any crater / holey round resets to the 8,600 base — rough epochs (knee
    // ≈ N 6.7) keep short key-downs. Duty: 11.10 s key-down ≈ 77 % inside the
    // deliberate [5000,12000] PA clamp; the ~2 s turnaround stays the cooling
    // gap. Same evidence doctrine as the predictive climb: prove it, then
    // spend it.
    // RUNG GATE (gate A/B 2026-07-07): the streak alone over-escalates at
    // Moderate — clean pairs happen between fades there, and 11 s key-downs
    // then eat the next fade (moderate@16 s42: escalation ON = FAIL 690 bps
    // with twelve 8-frame groups; OFF = PASS 1030, 0 craters). The ACTIVE RUNG
    // is the receiver's measured channel verdict (authority-commanded, anchor
    // columns encode fading class): Moderate-class operation lives at QPSK
    // rungs by construction, so requiring a dense rung (>= QAM8 R2/3) makes
    // long key-downs reachable exactly when the receiver's own measurement
    // rates the channel calm — no new estimator, no new threshold.
    // ── DENSE-RUNG PROXY REPAIR (ULTRA_BURST_ESC_STREAK, default-off) ──────────
    // `dense_rung` above is a PROXY for "the receiver rates this channel calm": the
    // anchor columns encode fading class, so Moderate-class operation lives at QPSK
    // rungs by construction. The radio-agnostic EVM demote (2026-07-24) BREAKS that
    // proxy — it correctly holds the rung at QPSK when HARDWARE loss, not channel
    // roughness, depresses usable SNR, so a QPSK rung no longer implies a rough
    // channel. Measured on the IONOS rig (run A6, MPG@20): 100% of the receiver's
    // fading verdicts were GOOD class (0.28-0.40, boundary kFadingGoodMax=0.76), yet
    // 14 of 17 groups were pinned to the 8.6 s / N=5 ceiling by this gate alone.
    //
    // The sender cannot substitute a fading index here: its own fading_index_ FREEZES
    // between sparse control decodes during a burst transfer (see wireFadingIndex() —
    // rig W5b/W8 measured peer_fading pinned for 300+ s), so it is not a live channel
    // reading. The honest, FRESH, sender-side evidence that long key-downs survive on
    // this channel right now is the CLEAN-GROUP STREAK itself (full retire, no holes).
    // So at a non-dense rung, escalate on a LONGER proven streak: the rung no longer
    // vouches for the channel, so demand more delivery evidence in its place.
    //
    // Safe by construction: any holey or zero-progress round resets the streak to 0
    // (base ceiling) exactly as before, so a genuinely Moderate channel — where craters
    // keep the streak short — still never escalates. That honors the 2026-07-07
    // moderate@16 A/B (escalation ON = FAIL 690 bps) which motivated the dense-rung
    // gate; what CHANGED since is cross-frame interleave going default-OFF (2026-07-21),
    // so a fade inside a long burst now costs only the frames it touches (per-frame SACK
    // resend) instead of cratering the whole group — A6: 4 partial groups, 0 full craters.
    // Simulated against A6's real per-group outcomes: +6.5% at streak>=2, +4.9% at >=3,
    // +3.3% at >=4 (the ceiling if EVERY group ran N=8 would be +13.9% — the streak
    // requirement legitimately caps it). Value = required streak; unset/0 = legacy.
    // Read per call (not latched): this runs once per burst-group formation (~10 s
    // apart), never on the audio thread, so the getenv is free — and it keeps the knob
    // togglable mid-session for A/B and unit-testable in a single process.
    // STREAK = 2 with rung gate + climb-carry. Measured trail: streak-2 alone
    // won +15 % paired in calm epochs but re-escalated between craters in rough
    // ones (F198-F207 cross-epoch mean 1.42 vs 1.57 — confounded by a 23 %
    // rougher epoch); streak-6 fired too late to pay on normal transfers.
    // The 2026-07-07 interleaved same-epoch A/B is the deciding measurement —
    // see docs/GROUP_SIZE_LEVER_2026_07_07.md addendum.
    // A forced/operator-pinned dense profile is not evidence that the channel
    // earned a longer key-down.  Only an adaptive, delivery-proven link may spend
    // the two-clean-group escalation; a pin stays at the base ceiling.
    const int trusted_clean_streak =
        rateAdaptationActive() ? burst_clean_group_streak_ : 0;
    const uint32_t ceiling_ms = connection_policy::burstAirtimeCeilingMs(
        data_modulation_, data_code_rate_, trusted_clean_streak);
    const uint32_t reanchor_ms =
        connection_policy::shouldUseWideOFDMShortReanchor(
            negotiated_mode_, data_modulation_, fading_index_)
            ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
            : 0;
    return connection_policy::wideOFDMBurstFrameBudget(
        data_modulation_, data_code_rate_, physicalDataFrameCodewords(), max_frames,
        ceiling_ms, reanchor_ms, selectBurstLiftingZ(),
        force_full_group_start
            ? connection_policy::kWideOFDMFullAnchorExtraMs
            : 0u);
}

uint32_t Connection::unifiedBurstAckTimeoutMs(size_t burst_frames) const {
    const uint32_t reanchor_ms =
        connection_policy::shouldUseWideOFDMShortReanchor(
            negotiated_mode_, data_modulation_, fading_index_)
            ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
            : 0;
    // DATA frames carry the negotiated lifting z (z=81 long-LDPC ≈ 3× the coded bits/
    // codeword → ~3× the per-frame airtime); the control/ack frame is always short (z=27).
    const int data_z = selectBurstLiftingZ();
    // Single source of truth (testable across the whole mod/rate/cw/z matrix):
    // connection_policy::unifiedBurstAckTimeoutMs. It budgets burst airtime + the receiver's
    // SACK-coalesce holdoff (the term the old inline formula OMITTED — see the IONOS MPG E5
    // premature-timeout fix 2026-06-19) + decode jitter + ack-return + turnaround + the §16.4
    // reliability full-anchor reserve. arq_.getSackDelay() is only a FLOOR here — the free function
    // takes max(it, the mod/rate-modeled wideOFDMSackDelayMs), and that model floor is what
    // guarantees coverage of the receiver's full physical hold (the sender's own configured value is
    // its shorter sliding delay, not necessarily the peer's full-burst coalesce hold).
    return connection_policy::unifiedBurstAckTimeoutMs(
        data_modulation_, data_code_rate_, physicalDataFrameCodewords(), burst_frames, data_z,
        wideOFDMControlModulationForData(data_modulation_), arq_.getSackDelay(), reanchor_ms);
}

size_t Connection::prepareUnifiedBurstWindow(bool repair_turn) {
    if (!(negotiated_mode_ == WaveformMode::OFDM_CHIRP &&
          kUnifiedSeqEnabled())) {
        return std::numeric_limits<size_t>::max();  // legacy: fill the whole window
    }
    // Logical submission keeps the conservative mode-wide slot timeout. The physical
    // egress commits an exact deadline only to frames that really go on air.
    return burstAirtimeBudgetFrames(
        arq_.getWindowSize(), repair_turn || desc_switch_full_anchor_pending_);
}

uint32_t Connection::finalizeUnifiedBurstWindow(
    const std::vector<Bytes>& transmitted_frames,
    uint32_t queue_delay_ms,
    bool force_full_group_start) {
    if (!(negotiated_mode_ == WaveformMode::OFDM_CHIRP &&
          kUnifiedSeqEnabled()) ||
        transmitted_frames.empty()) {
        return 0;
    }

    const size_t physical_frame_count = transmitted_frames.size();
    const auto timing = physicalDataRoundTiming(
        transmitted_frames, force_full_group_start);
    const uint64_t timeout_with_queue =
        static_cast<uint64_t>(timing.ack_timeout_ms) + queue_delay_ms;
    const uint32_t timeout_ms = static_cast<uint32_t>(
        std::min<uint64_t>(timeout_with_queue, UINT32_MAX));
    const size_t rearmed =
        arq_.rearmTransmittedDataFrames(transmitted_frames, timeout_ms);
    LOG_MODEM(INFO,
              "Connection: Finalized DATA round: physical=%zu, ARQ=%zu, "
              "cw_total=%u, cw_max=%u, variable=%zu, waveform=%ums, keyed=%ums, "
              "queue=%ums, timeout=%ums",
              physical_frame_count, rearmed, timing.total_codewords,
              static_cast<unsigned>(timing.max_codewords), timing.variable_frames,
              timing.waveform_airtime_ms, timing.airtime_ms, queue_delay_ms,
              timeout_ms);
    return timeout_ms;
}

void Connection::armToneBurstAckListenWindow(uint32_t explicit_timeout_ms) {
    if (!(kInteractiveToneAckEnabled() && on_arm_tone_burst_ack_monitor_)) {
        return;
    }
    // Listen for the ack as long as we'd wait before resending — the ARQ ack timeout is
    // burst-aware (it covers the burst airtime + decode jitter + ack return), so the
    // monitor can never expire mid-round-trip. Floor at the interactive value for short
    // MC-DPSK/interactive sends. Each physical DATA round replaces the prior round's
    // deadline and the monitor auto-disarms the instant an ACK decodes.
    uint32_t window_ms = kInteractiveToneAckWindowMs;
    if (isOFDMMode(negotiated_mode_) ||
        negotiated_mode_ == WaveformMode::MC_DPSK) {
        const uint32_t timeout_ms = explicit_timeout_ms > 0
            ? explicit_timeout_ms
            : arq_.getAckTimeout();
        window_ms = std::max(window_ms, timeout_ms);
    }
    on_arm_tone_burst_ack_monitor_(window_ms);
}

size_t Connection::currentDataPayloadCapacity() const {
    if (isOFDMMode(negotiated_mode_)) {
        const int physical_cw = physicalDataFrameCodewords();
        // Z-aware capacity: at z=81 info bytes per codeword scale 3x. Sizing the
        // chunker at the active z (selectBurstLiftingZ) keeps the encoder from
        // zero-padding 70%+ of every burst (the real-world throughput killer).
        if (selectBurstLiftingZ() == 81) {
            return v2::getFixedFramePayloadCapacityZ(
                data_code_rate_, physical_cw, 81);
        }
        return v2::getFixedFramePayloadCapacity(data_code_rate_, physical_cw);
    }
    if (usesBoundedVariableMCDPSKFrames()) {
        return v2::getVariableFramePayloadCapacity(data_code_rate_, data_frame_cw_count_);
    }
    return SIZE_MAX;
}

LadderRungId Connection::currentLadderRungId() const {
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        return LadderRungId::OFDM_CHIRP;
    }
    if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        return LadderRungId::OFDM_NARROW;
    }
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        return connection_policy::rungForMCDPSKConfig(
            data_modulation_, config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol, data_frame_cw_count_).id;
    }
    return LadderRungId::UNKNOWN;
}

void Connection::notifyDataModeChanged(float snr_db, float peer_fading_index,
                                       bool snr_is_wire) {
    if (!on_data_mode_changed_) {
        return;
    }
    const bool mc_dpsk = negotiated_mode_ == WaveformMode::MC_DPSK;
    on_data_mode_changed_(data_modulation_, data_code_rate_, data_frame_cw_count_,
                          snr_db, peer_fading_index,
                          mc_dpsk ? config_.mc_dpsk_num_carriers : 0,
                          mc_dpsk ? config_.mc_dpsk_samples_per_symbol : 0,
                          snr_is_wire);
}

// ─────────────────────── [LADDER] per-transfer telemetry ────────────────────────
// Sender-side observability for the fade-riding ladder: accumulates wall time per
// (modulation, rate) rung between startFileTransferNow and transfer completion, and
// counts mid-stream moves (applyDataMode mod/rate changes). Logged ONCE at completion:
//   [LADDER] qpsk_r34=54% qam16_r23=38% moves=9 (52s, ok)
// Pure telemetry — no control effect; wall clock is fine (the faithful gate and the
// rig both run wall==sample time).

// "QPSK"+"R3/4" -> "qpsk_r34" (lowercase, slash dropped) — grep-stable rung labels.
static std::string ladderRungLabel(Modulation mod, CodeRate rate) {
    std::string label = modulationToString(mod);
    label += '_';
    label += codeRateToString(rate);
    std::string out;
    out.reserve(label.size());
    for (char c : label) {
        if (c == '/') continue;
        out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

void Connection::ladderTelemetryStart() {
    ladder_telemetry_active_ = true;
    ladder_moves_ = 0;
    ladder_rung_stats_.clear();
    ladder_transfer_start_ = ladder_rung_start_ = std::chrono::steady_clock::now();
    ladder_cur_mod_ = data_modulation_;
    ladder_cur_rate_ = data_code_rate_;
}

void Connection::ladderTelemetryNoteRung(Modulation mod, CodeRate rate, bool count_move) {
    if (!ladder_telemetry_active_) return;
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now - ladder_rung_start_).count();
    bool found = false;
    for (auto& s : ladder_rung_stats_) {
        if (s.mod == ladder_cur_mod_ && s.rate == ladder_cur_rate_) {
            s.seconds += seconds;
            found = true;
            break;
        }
    }
    if (!found) {
        ladder_rung_stats_.push_back({ladder_cur_mod_, ladder_cur_rate_, seconds});
    }
    ladder_rung_start_ = now;
    ladder_cur_mod_ = mod;
    ladder_cur_rate_ = rate;
    if (count_move) ++ladder_moves_;
}

void Connection::ladderTelemetryFinish(bool success) {
    if (!ladder_telemetry_active_) return;
    ladderTelemetryNoteRung(data_modulation_, data_code_rate_, /*count_move=*/false);
    ladder_telemetry_active_ = false;
    double total_s = 0.0;
    for (const auto& s : ladder_rung_stats_) total_s += s.seconds;
    if (total_s <= 0.0) return;
    std::string line;
    char buf[64];
    for (const auto& s : ladder_rung_stats_) {
        std::snprintf(buf, sizeof(buf), "%s%s=%.0f%%", line.empty() ? "" : " ",
                      ladderRungLabel(s.mod, s.rate).c_str(), 100.0 * s.seconds / total_s);
        line += buf;
    }
    LOG_MODEM(INFO, "Connection: [LADDER] %s moves=%d (%.0fs, %s)",
              line.c_str(), ladder_moves_, total_s, success ? "ok" : "fail");
}

void Connection::applyDataMode(Modulation mod, CodeRate rate, int cw_count,
                               LadderRungId rung_id) {
    // Any data-mode application is an acquisition-era boundary for the experimental
    // partial-SACK shortcut.  Fail closed even when a caller eventually resolves to
    // the same tuple; a stale exact-group ACK must never bridge a control transaction.
    partial_sack_last_round_ = {};
    partial_sack_descriptor_repair_scope_ = false;
    if (rung_id != LadderRungId::UNKNOWN) {
        const auto rung = connection_policy::ladderRungForId(rung_id);
        if (rung.id != LadderRungId::UNKNOWN) {
            negotiated_mode_ = rung.waveform;
            if (rung.waveform == WaveformMode::MC_DPSK) {
                config_.mc_dpsk_num_carriers = rung.num_carriers;
                config_.mc_dpsk_samples_per_symbol = rung.samples_per_symbol;
                mod = rung.modulation;
                rate = rung.code_rate;
                if (cw_count == 0) {
                    cw_count = rung.cw_count;
                }
            }
        }
    }

    // Resolve final CW count: explicit value if specified (e.g. from
    // MODE_CHANGE wire byte), else auto-pick from rate.
    const int new_cw = (cw_count > 0)
        ? v2::sanitizeFixedFrameCodewords(cw_count)
        : connection_policy::recommendCWCount(mod, rate, negotiated_mode_);
    const bool rate_changed = rate != data_code_rate_;
    const bool cw_changed = new_cw != data_frame_cw_count_;
    // A MODULATION change (e.g. the QPSK-R3/4 -> QAM16-R2/3 climb) changes the constellation
    // geometry (bits/symbol -> frame airtime; per-CW BYTE capacity is rate/CW-derived,
    // getFixedFramePayloadCapacity, and does NOT change with modulation) — so like a rate/CW
    // change it must trigger the re-encode + HARQ flush. Computed BEFORE data_modulation_ is overwritten
    // below. NOTE: this drives requeuePendingChunks() + the soft_combine_harq_.clear() below (the
    // load-bearing part — stale old-constellation LLRs would corrupt HARQ). The deeper ARQ-window
    // byte-capacity rewind lives in setCodeRate()/setFixedFrameCodewords() (via
    // configureArqForCurrentDataMode), which early-return on an unchanged rate/CW — so a PURE
    // modulation-only transition (same rate AND same CW) is NOT yet fully ARQ-safe. That case does
    // not occur on the DEFAULT path: the QAM16 climb (R3/4->R2/3) and demote (R2/3->R3/4) ALWAYS
    // co-change the rate, so setCodeRate fires and the window rewinds correctly. EXCEPTION
    // (ULTRA_QAM16_R34, default-OFF A/B): the stuck-frame escape QAM16 R3/4 -> QPSK R3/4 is
    // mod-only at the SAME rate (and typically the same CW=8) — the setCodeRate rewind is skipped
    // there. The per-CW BYTE capacity is rate/CW-derived (LDPC K x cw_count), not
    // modulation-derived, so in-flight frame bytes stay geometry-valid and the requeue+HARQ flush
    // above still fire; validate that transition on the faithful gate before the knob graduates.
    const Modulation old_mod_for_streak = data_modulation_;
    const CodeRate old_rate_for_streak = data_code_rate_;
    const bool mod_changed = mod != data_modulation_;
    const bool geometry_changed = rate_changed || cw_changed || mod_changed;

    // A fragmented message/binary object is cut once using the geometry active
    // at admission. Ordinary adaptive moves are held by
    // hasGeometryBoundDataOperation(), but mandatory receiver-commanded or
    // stuck-frame demotions must still be able to escape a window that will not
    // drain. Fail that logical object explicitly before regridding: abandoning
    // the old slots with a forced move-epoch bump makes the next object an
    // unambiguous rebase, so the receiver discards any partial old prefix instead
    // of stitching or accepting truncated bytes.
    std::vector<OutboundMessageTxRecord> geometry_failed_message_records;
    bool geometry_failed_nonfile_operation = false;
    if (geometry_changed &&
        file_transfer_.getState() != FileTransferState::SENDING) {
        const bool fragment_batch_active = !pending_tx_fragments_.empty();
        const bool arq_payload_active = arq_.getTxInFlightBytes() > 0;
        const bool submitted_message_active =
            std::any_of(outbound_message_tx_records_.begin(),
                        outbound_message_tx_records_.end(),
                        [](const OutboundMessageTxRecord& record) {
                            return record.first_seq_valid &&
                                   !record.terminal_reported;
                        });
        geometry_failed_nonfile_operation =
            fragment_batch_active || arq_payload_active ||
            submitted_message_active;
        if (geometry_failed_nonfile_operation) {
            for (auto it = outbound_message_tx_records_.begin();
                 it != outbound_message_tx_records_.end();) {
                const bool token_in_fragment_batch =
                    std::find(pending_tx_fragment_message_tokens_.begin(),
                              pending_tx_fragment_message_tokens_.end(),
                              it->token) !=
                    pending_tx_fragment_message_tokens_.end();
                const bool active_record =
                    !it->terminal_reported &&
                    (it->first_seq_valid || token_in_fragment_batch);
                if (!active_record) {
                    ++it;
                    continue;
                }
                it->terminal_reported = true;
                geometry_failed_message_records.push_back(*it);
                it = outbound_message_tx_records_.erase(it);
            }

            LOG_MODEM(WARN,
                      "Connection: Abandoning active message/binary operation before geometry change %s %s -> %s %s",
                      modulationToString(data_modulation_),
                      codeRateToString(data_code_rate_),
                      modulationToString(mod), codeRateToString(rate));
            deferred_fragment_refill_ = false;
            burst_mode_active_ = false;
            burst_tx_buffer_.clear();
            arq_.abortPendingTx(/*force_move_epoch_bump=*/true);
            pending_tx_fragments_.clear();
            pending_tx_fragment_flags_.clear();
            pending_tx_fragment_types_.clear();
            pending_tx_fragment_message_tokens_.clear();
            next_fragment_idx_ = 0;
            acked_fragment_count_ = 0;
            arq_submit_message_token_ = 0;
        }
    }
    // Pending chunks must be re-encoded if rate OR CW OR modulation changed: the ARQ payload
    // capacity depends on all three, and chunks queued under the old geometry will
    // overflow / mis-align under the new one.
    const bool requeue_file =
        geometry_changed &&
        file_transfer_.getState() == FileTransferState::SENDING &&
        file_transfer_.hasPendingChunks();
    const bool refill_file =
        geometry_changed &&
        file_transfer_.getState() == FileTransferState::SENDING;
    if (requeue_file) {
        file_transfer_.requeuePendingChunks();
    }

    // [LADDER] telemetry: close the current rung segment on a real mid-transfer move.
    if ((rate_changed || mod_changed) && ladder_telemetry_active_) {
        ladderTelemetryNoteRung(mod, rate, /*count_move=*/true);
    }

    data_modulation_ = mod;
    data_code_rate_ = rate;
    data_frame_cw_count_ = new_cw;
    config_.fixed_frame_codewords = new_cw;
    data_ladder_rung_id_ = (rung_id != LadderRungId::UNKNOWN)
        ? rung_id
        : currentLadderRungId();
    configureArqForCurrentDataMode();
    // §RETX-PACING: a mode/rate change starts a NEW era — the zero-round evidence and any
    // armed hold belong to the rung we just left (§7 checklist: reset on applyDataMode).
    zero_progress_rounds_ = 0;
    trough_episode_active_ = false;  // trough-amnesty episode dies with the era
    retx_pace_hold_ms_ = 0;
    // GROUP-SIZE: the clean streak is CHANNEL evidence (fade cadence), not rung
    // evidence — an UP-move was itself earned by clean rounds, so it carries the
    // streak (resetting on climbs made escalation unreachable on short
    // transfers: the ladder's own ramp consumed the window). DOWN/lateral moves
    // mean the channel got worse — reset; a crater at the new rung resets via
    // the round accounting regardless.
    {
        const uint8_t old_idx = coherentRungIndexFor(old_mod_for_streak, old_rate_for_streak);
        const uint8_t new_idx = coherentRungIndexFor(mod, rate);
        if (new_idx <= old_idx || old_idx == kRungIdxNone) {
            burst_clean_group_streak_ = 0;
        }
    }
    // RX-RATE-CMD Phase 2: an APPLIED mod/rate change is exactly the adoption the
    // receiver's standing rung command was waiting for (descriptor adopt and legacy
    // MODE_CHANGE both funnel through here) — clear the once-per-committed-move latch
    // so the next ACK stops carrying the consumed command. An idempotent re-apply
    // (nothing changed) is NOT an adoption and keeps the latch.
    if (rate_changed || mod_changed) {
        rx_rate_cmd_pending_ = 0;
        // RX-AUTHORITY: any real mode move (an obey, an ack-silence escape, a
        // legacy exchange) starts a new era for the obey-dedup — without this, a
        // safety escape that moved us OFF a previously-obeyed target would block
        // re-obeying that same target when the receiver re-commands it.
        tx_authority_last_obeyed_ = 0;
    }
    if (geometry_changed) {
        soft_combine_harq_.clear();  // mod change => old-constellation LLRs would corrupt HARQ
        // 2026-05-28: recompute burst ack_timeout for the new mode (same
        // formula as startup / applyAdaptiveRateFeedback). MODE_CHANGE
        // negotiations can land on a slower rate where the original
        // timeout no longer covers the burst+ack round-trip; without
        // this, the next burst at the new rate will fire premature
        // resends every group.
    }

    if (refill_file) {
        deferred_file_refill_ = true;
    }

    // Publish only after the old ARQ identities are gone and the new geometry is
    // fully installed. Reentrant application code can now submit its retry; while
    // a MODE_CHANGE commit is still pending it will be queued and released by the
    // caller's normal post-commit refill.
    if (geometry_failed_nonfile_operation && !arq_.moveEpochEnabled() &&
        state_ == ConnectionState::CONNECTED) {
        // Without the negotiated epoch/rebase contract, abandoning any old DATA
        // sequence leaves the peer waiting on an unfillable RX hole. Match the
        // terminal-slot-failure policy: close the session before application code
        // can enqueue a "next" message onto a structurally unusable sequence grid.
        LOG_MODEM(ERROR,
                  "Connection: Geometry change abandoned active DATA with MOVE-EPOCH disabled; disconnecting to reset peer sequence state");
        disconnect();
    }
    emitFailedMessageRecords(geometry_failed_message_records);
    if (geometry_failed_nonfile_operation && on_message_sent_) {
        on_message_sent_(false);
    }
}

void Connection::commitPendingModeChange(const char* outcome) {
    if (!mode_change_pending_) {
        return;
    }

    LOG_MODEM(INFO, "Connection: MODE_CHANGE %s, applying %s %s",
              outcome,
              modulationToString(pending_modulation_),
              codeRateToString(pending_code_rate_));
    applyDataMode(pending_modulation_, pending_code_rate_,
                  pending_cw_count_, pending_ladder_rung_id_);
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    // A normal rate/CW regrid has already invalidated old serialized identities. This
    // matters for the same-rate/same-CW modulation edge, where live bytes remain valid
    // and only the physical constellation changes.
    arq_.resumeDeferredTimeoutRetransmits(/*timeout_ms=*/1);

    // pending_snr_db_ is what WE embedded in our MODE_CHANGE request — a LOCAL value.
    notifyDataModeChanged(pending_snr_db_, pending_fading_index_, /*snr_is_wire=*/false);
    runDeferredArqRefill();
}

// ═══════════════ DESC-SWITCH — descriptor-committed rate/mod move ═══════════════
// docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.1, knob ULTRA_DESCRIPTOR_MODE_SWITCH
// (default OFF = byte-identical). The DECISION machinery (RateController EMA + ssthresh +
// QAM16 climb/demote + clean-boundary gate + escape drops) is UNTOUCHED — only the COMMIT
// changes: instead of the MODE_CHANGE stop-and-wait round-trip (2.4-4 s clean, 18.5 s per
// retry × 2-9 receptions in troughs, TX frozen throughout), the sender applies the mode
// locally and the next burst's BURST_HEADER descriptor — control-profile QPSK R1/4,
// already trusted by the RX demod for exactly this reconfiguration — IS the announcement.
//
// Phase-1 scope gate: CLEAN-BOUNDARY wideband-OFDM ladder moves. This function itself
// carries NO in-flight guard — the boundary property comes from its callers. Phase 2
// (RX-RATE-CMD, maybeApplyRxRateCommand) adds the ONE sanctioned mid-window caller: a
// receiver-commanded demote, pre-gated on arq_.moveEpochEnabled() so the resulting
// commitLocalModeSwitch regrid is era-safe (the ARQ abort bumps the move-epoch).
// Excluded BY DESIGN (all keep the legacy MODE_CHANGE exchange, in every knob state):
//   - connect-time INITIAL_SETUP and USER_REQUEST moves,
//   - MC-DPSK rung moves (carriers/sps need ladder_rung_id on the wire) and OFDM_NARROW,
//   - sender-initiated stuck/collapse escapes (silence does not prove descriptor receipt).
bool Connection::tryDescriptorModeSwitch(Modulation mod, CodeRate rate,
                                         float measured_snr, uint8_t reason) {
    if (!descriptor_mode_switch_enabled_) return false;
    if (state_ != ConnectionState::CONNECTED) return false;
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) return false;  // wideband ladder only
    if (mode_change_pending_) return false;  // a legacy exchange is already in flight

    // CW pick: IDENTICAL to requestModeChange (operator --cw-count override preserved,
    // else the Doppler-coherence-refined channel recommendation) so the descriptor
    // announces the same frame geometry the old exchange would have negotiated.
    const int cw = (config_.forced_cw_count != 0)
        ? v2::sanitizeFixedFrameCodewords(config_.forced_cw_count)
        : connection_policy::recommendCWCountForChannel(
              mod, rate, negotiated_mode_,
              connection_policy::coherenceAdjustedFadingIndex(
                  fading_index_, coherence_score_, coherence_valid_),
              measured_snr);

    // Descriptor-bearing-burst guard: encodeBurstLight emits NO BURST_HEADER for a
    // single-frame burst (streaming_encoder.cpp:476-489) — the announcement could not
    // ride, and the lone post-switch frame would be undecodable at the peer's old
    // geometry (an RTO grind at the file tail, NOT the §6 row-1 one-lost-group case).
    // Commit via descriptor only when the remaining file payload guarantees a >=2-frame
    // (descriptor-bearing) group at the NEW geometry; the file tail and the non-file
    // (message) path fall back to the legacy exchange, which handles them correctly
    // today. This is also where the mid-transfer dead-air lives, so the fallback costs
    // nothing the design targets.
    if (file_transfer_.getState() != FileTransferState::SENDING) return false;
    const auto progress = file_transfer_.getProgress();
    if (progress.total_bytes <= progress.transferred_bytes) return false;
    const size_t remaining_bytes = progress.total_bytes - progress.transferred_bytes;
    const int physical_cw = physicalDataFrameCodewordsFor(mod, rate, cw);
    const int lifting_z = selectBurstLiftingZFor(mod, rate, cw);
    const size_t frame_payload = (lifting_z == 81)
        ? v2::getFixedFramePayloadCapacityZ(rate, physical_cw, 81)
        : v2::getFixedFramePayloadCapacity(rate, physical_cw);
    const size_t chunk_bytes =
        (frame_payload > FileTransferController::FILE_DATA_OVERHEAD)
            ? frame_payload - FileTransferController::FILE_DATA_OVERHEAD
            : 0;
    if (chunk_bytes == 0 || remaining_bytes <= chunk_bytes) return false;

    commitLocalModeSwitch(mod, rate, cw, measured_snr, reason);
    return true;
}

// The COMMIT half (§5.1 steps 1-3): applyDataMode NOW — no mode_change_pending_, no
// retry timer, no TX freeze; the next burst's descriptor (stamped with the new
// mod/rate/cw/z by transmitBurst → setDataMode) announces the move. Boundary cases:
//   - CLEAN boundary (all Phase-1 callers): the send window is drained, so
//     requeuePendingChunks() and arq_.setCodeRate's abort/rewind are no-ops — an
//     EMPTY-window regrid is collision-free by construction and needs NO epoch bump
//     (when ULTRA_ARQ_MOVE_EPOCH is ON the existing machinery still stamps
//     EPOCH_REBASE on the first frame at the window base and echoes the epoch on the
//     tone-ACK — belt-and-braces, zero extra work here).
//   - MID-WINDOW (Phase 2's receiver-commanded demote, the ONLY such caller, itself
//     gated on arq_.moveEpochEnabled()): frames ARE in flight — applyDataMode
//     requeues the pending chunks and the arq_.setCodeRate abort/rewind fires,
//     bumping the TX move-epoch (setCodeRate rate-abort; same-rate mod/CW regrids bump via the abortPendingTx payload-drop site, 2026-07-04) so the regrid is
//     a recognized new era, and the log line below shows the bumped value.
void Connection::commitLocalModeSwitch(Modulation mod, CodeRate rate, int cw_count,
                                       float measured_snr, uint8_t reason) {
    (void)reason;  // decision telemetry only — nothing rides a control frame on this path
    // FIXED-GRID BAND (2026-07-06): capture the pilot geometry BEFORE the switch.
    const int old_spacing = ultra::ofdm_link_adaptation::recommendedPilotSpacing(
        data_modulation_, data_code_rate_);
    applyDataMode(mod, rate, cw_count, currentLadderRungId());
    const int new_spacing =
        ultra::ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);

    // §2.6-arm-3 mitigation, RE-REFINED (F163 2026-07-06): the fixed-grid theory
    // ("within-band switch changes only the constellation — nothing to desync,
    // the descriptor alone re-labels the next group") was HALF right: warm
    // sync/CFO/|H| do carry, but the DESCRIPTOR still has to be FOUND, and on
    // fading a light-preamble descriptor at a MODE BOUNDARY is exactly the one
    // that cannot afford to be missed. F163: three up-switches rode light
    // descriptors into fades, the receiver decoded whole groups at the STALE
    // config (cw_fail=8 vs a 12-cw burst), and adoption waited 25 s for a
    // full-anchor RTO resend — ~130 s of the 424 s transfer. A missed descriptor
    // between SAME-mode groups is harmless (warm frame path, same geometry);
    // across a switch it is total loss. So: FULL anchor on EVERY commit — the
    // +1.2 s of chirp+LTS is the cheapest insurance in the whole budget.
    desc_switch_full_anchor_pending_ = true;
    if (new_spacing != old_spacing) {
        LOG_MODEM(INFO,
                  "Connection: DESC-SWITCH grid change (pilot sp %d -> %d) — full "
                  "anchor armed",
                  old_spacing, new_spacing);
    } else {
        LOG_MODEM(INFO,
                  "Connection: DESC-SWITCH within fixed grid (sp %d) — warm state "
                  "carries; full anchor armed for the switch descriptor (F163)",
                  new_spacing);
    }

    ++stats_.descriptor_mode_switches;
    // A/B grep line (§7.2 metric): the epoch is the ARQ TX move-epoch — 0 while
    // ULTRA_ARQ_MOVE_EPOCH is OFF, and a clean-boundary commit never bumps it.
    LOG_MODEM(INFO, "Connection: DESC-SWITCH commit %s %s (epoch %u)",
              modulationToString(mod), codeRateToString(rate),
              static_cast<unsigned>(arq_.txMoveEpoch()));

    // GUI/modem follow-through — the encoder picks up the new mode before the next
    // transmitBurst. Same notify commitPendingModeChange fires; LOCAL reading.
    notifyDataModeChanged(measured_snr, wireFadingIndex(), /*snr_is_wire=*/false);
    // applyDataMode deferred the file refill (rate/CW/mod changed while SENDING);
    // release it now — the next burst goes out at the new rung with NO idle round-trip
    // (the whole point: the mode_change_pending_ TX freeze is gone).
    runDeferredArqRefill();
}

// RX side (§5.1 step 4b): the decoder consumed a mode-hop BURST_HEADER descriptor
// (wired decoder → ModemEngine → frontend binding → ProtocolEngine::onDescriptorModeChange,
// mirroring onBurstGroupReceived — same thread/locking class, §6 row 11). Run the
// RX-relevant subset of a mode change: applyDataMode sets data_modulation_/
// data_code_rate_/data_frame_cw_count_ and configureArqForCurrentDataMode refreshes
// window/timers/chunk capacity — ofdmWindowSize is mod/rate-dependent, so without this a
// receiver holding the old window would below-window-drop the tail of a wider post-hop
// burst. NO MODE_CHANGE ACK machinery fires: confirmation is the switched group's
// tone-burst ACK (implicit and free).
void Connection::onDescriptorModeChange(Modulation mod, CodeRate rate, int cw_per_frame) {
    if (!descriptor_mode_switch_enabled_) return;  // knob-OFF: byte-identical no-op
    if (disconnect_teardown_active_) return;
    if (state_ != ConnectionState::CONNECTED) return;
    if (negotiated_mode_ != WaveformMode::OFDM_CHIRP) return;  // wideband-OFDM scope
    if (mod == data_modulation_ && rate == data_code_rate_ &&
        (cw_per_frame <= 0 || cw_per_frame == data_frame_cw_count_)) {
        return;  // already adopted (re-announced descriptor on a resend) — idempotent
    }
    if (arq_.getTxInFlightBytes() > 0) {
        // Receiver-ISS asymmetry (§6 row 12): WE have our own DATA in flight
        // (half-duplex interactive role overlap). Adopting the peer's TX geometry now
        // would abort OUR send window; per-direction rungs are independent and the
        // peer's next descriptor re-announces. Skip.
        LOG_MODEM(WARN,
                  "Connection: DESC-SWITCH adopt skipped (%s %s) — local DATA in flight",
                  modulationToString(mod), codeRateToString(rate));
        return;
    }
    // §8 checklist 4: clean-boundary invariant — at a boundary switch the RX slots are
    // provably empty (the sender's drained window ⟹ we delivered in-order), so
    // arq_.setCodeRate's RX discard below is a no-op. Non-empty slots + move-epoch OFF
    // mean the sender committed mid-window without era safety — log loudly; the discard
    // still runs (existing setCodeRate semantics) and the sender's ARQ resends cover
    // the loss. With move-epoch ON this is the EXPECTED Phase-2 escape-adopt shape
    // (RX-RATE-CMD mid-window commit: the descriptor arrives ahead of the EPOCH_REBASE
    // frames whose adoption performs the discard) — INFO, not a violation.
    if (arq_.bufferedRxFrameCount() > 0) {
        if (arq_.moveEpochEnabled()) {
            LOG_MODEM(INFO,
                      "Connection: DESC-SWITCH mid-window adopt with %zu buffered RX "
                      "frames — era safety via move-epoch (Phase-2 escape path)",
                      arq_.bufferedRxFrameCount());
        } else {
            LOG_MODEM(WARN,
                      "Connection: DESC-SWITCH adopt with %zu buffered RX frames — "
                      "clean-boundary invariant violated (mid-window regrids belong to "
                      "the move-epoch machinery)",
                      arq_.bufferedRxFrameCount());
        }
    }
    // A/B grep line (§7.2 metric).
    LOG_MODEM(INFO, "Connection: DESC-SWITCH adopt %s %s",
              modulationToString(mod), codeRateToString(rate));
    applyDataMode(mod, rate, cw_per_frame, LadderRungId::UNKNOWN);
    ++stats_.descriptor_mode_switches;
    // GUI/modem follow-through: LOCAL snr reading (no SNR rides a descriptor); peer
    // fading unknown on this path → -1.0 = the existing "n/a" render.
    notifyDataModeChanged(measured_snr_db_, /*peer_fading_index=*/-1.0f,
                          /*snr_is_wire=*/false);
    runDeferredArqRefill();
}

void Connection::resetAdaptiveDecisionState() {
    // All outcome estimators and burst-policy evidence are scoped to one QSO. A persistent
    // GUI/TNC process must not seed the next peer/channel from the preceding transfer, nor
    // inherit the every-Nth probe phase, an already-escalated eight-frame burst streak,
    // a saturated collapse-escape episode, or the preceding QSO's rung-dwell clock.
    latent_ctl_.reset();
    latent_startup_probe_allowed_ = true;
    latent_startup_probe_waiting_ = false;
    latent_startup_probe_spent_ = false;
    latent_startup_probe_clean_groups_ = 0;
    latent_startup_probe_pending_base_groups_ = 0;
    latent_startup_probe_rollback_pending_ = false;
    latent_startup_probe_failed_ = false;
    tx_latent_startup_probe_active_ = false;
    tx_latent_startup_probe_airborne_ = false;
    tx_latent_startup_probe_timeout_rollback_pending_ = false;
    tx_latent_startup_probe_base_rung_ = kRungIdxNone;
    tx_authority_ack_identity_valid_ = false;
    tx_authority_ack_group_seq_ = 0;
    tx_authority_ack_frame_mask_ = 0;
    tx_authority_ack_move_epoch_ = 0;
    tx_authority_ack_lowest_cmd_ = kRungIdxNone;
    goodput_ctl_ = GoodputRateController{};
    goodput_last_verdict_ = {};
    goodput_last_verdict_valid_ = false;
    burst_clean_group_streak_ = 0;
    consecutive_escape_drops_ = 0;
    rx_auth_last_change_ = {};
    rx_auth_last_change_valid_ = false;
}

void Connection::enterConnected(bool automatic_rate_allowed) {
    state_ = ConnectionState::CONNECTED;
    connected_time_ms_ = 0;
    resetAdaptiveDecisionState();
    latent_startup_probe_allowed_ = automatic_rate_allowed;
    staged_timeout_batch_.clear();
    arq_tick_in_progress_ = false;
    // #58 increment 3: the handshake spent (or never needed) its one pick-defer;
    // re-arm for the next handshake. Consulted only pre-CONNECT, so this is safe here.
    connect_pick_deferred_once_ = false;
    // Half-duplex INTERACTIVE (TNC/Winlink-B2F): the RESPONDER speaks first (the SID banner).
    // It does NOT need a pre-confirmed handshake to do so — the initiator proactively yields a
    // TURNOVER ~1.5 s after connect (see tick()), and receiving that TURNOVER is the responder's
    // "first valid frame" → it flips handshake_confirmed_ AND fires the modem-waveform switch
    // (onFrameReceived), both at the correct time, then handleTurnover makes it ISS. So the
    // responder enters as a normal IRS and the turn falls to it naturally.
    interactive_initiator_yield_done_ = false;
    interactive_yield_log_throttle_ms_ = 0;
    local_data_turn_ = is_initiator_;
    peer_data_turn_requested_ = false;
    local_turn_request_pending_ = false;
    received_peer_data_since_connect_ = false;
    yielded_data_turn_waiting_for_peer_data_ = false;
    data_turn_yield_pending_ = false;
    resetDataTurnFairness();
    // QAM16 climb state is per-connection (the re-climb cooldown/backoff resets here).
    qam16_clean_streak_ = 0;
    qam16_bad_streak_ = 0;
    qam16_r34_clean_streak_ = 0;
    qam16_reclimb_cooldown_ = 0;
    qam16_demote_count_ = 0;
    // Doppler-coherence verdict is per-connection: setChannelCoherence holds the last VALID
    // verdict while CONNECTED (BUG-DOPPLER-COHERENCE-MODECHANGE-WIPE), so it must start
    // invalid here or a previous connection's channel class would leak into this one.
    coherence_score_ = 0.0f;
    coherence_doppler_hz_ = 0.0f;
    coherence_valid_ = false;
    // §RETX-PACING: trough-pacing / collapse-escape round state is per-connection.
    zero_progress_rounds_ = 0;
    trough_episode_active_ = false;  // trough-amnesty episode dies with the era
    retx_pace_hold_ms_ = 0;
    last_data_burst_end_valid_ = false;
    partial_sack_last_round_ = {};
    partial_sack_descriptor_repair_scope_ = false;
    // Software-ALC receiver-side state is per-connection.
    rx_level_low_streak_ = 0;
    rx_level_clipped_ = false;
    rx_level_verdict_pending_for_group_ = false;
    tone_ack_alc_group_context_ = false;
    tone_ack_group_has_decoded_data_context_ = false;
    // RX-RATE-CMD Phase 2 state is per-connection (the seq dedup space restarts with
    // the ARQ reset below; a stale standing command must never leak across sessions).
    rx_rate_cmd_pending_ = 0;
    rx_rate_cmd_seq_seen_ = -1;
    rebase_voice_event_seq_ = 0;
    rx_rebase_voice_seq_seen_ = -1;
    rx_authority_cmd_ = 0;
    tx_authority_last_obeyed_ = 0;
    rx_auth_obs_count_ = 0;
    rx_auth_obs_next_ = 0;
    rx_auth_crater_streak_ = 0;
    rx_auth_clean_streak_ = 0;
    rx_auth_class_sticky_ = 1;
    rx_auth_class_streak_ = 0;
    rx_auth_fading_passed_ = 0.3f;
    rx_auth_climb_dwell_ = 0;
    rx_auth_gamma_count_ = 0;
    rx_auth_gamma_next_ = 0;
    for (auto& v : rx_auth_gamma_ring_) v.clear();
    for (size_t i = 0; i < kRungIdxCount; ++i) rx_auth_rung_penalty_db_[i] = 0.0f;
    burst_obs_snr_db_ = -1.0f;
    burst_obs_fading_ = -1.0f;
    burst_obs_evm_snr_db_ = -1.0f;
    burst_obs_coh_valid_ = false;
    qam16_rx_bad_streak_ = 0;
    last_applied_mode_change_valid_ = false;  // MC dedup (fix 3) is session-scoped
    data_turn_tx_guard_ms_ = 0;
    turn_request_retransmit_ms_ = 0;
    turn_request_holdoff_ms_ = 0;
    file_cancel_rx_drain_ms_ = 0;
    clearFileCancelReassertion();
    file_cancel_confirm_pending_ = false;
    if (local_data_turn_) {
        armDataTurnTxGuard(dataTurnConnectGuardMs());
    }

    if (is_initiator_ || handshake_confirmed_) {
        responder_handshake_wait_ms_ = 0;
    } else if (responder_handshake_wait_ms_ == 0) {
        responder_handshake_wait_ms_ = responderHandshakeFailSafeMs();
    }

    arq_.setCallsigns(local_call_, remote_call_);
    deferred_arq_failure_abort_ = false;
    deferred_arq_failure_seq_valid_ = false;
    arq_.reset();
    configureArqForCurrentDataMode();
    mode_change_ack_repeat_jobs_.clear();

    LOG_MODEM(INFO, "Connection: Now CONNECTED to %s (mode=%s, data_turn=%s)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_),
              local_data_turn_ ? "ISS" : "IRS");
    LOG_MODEM(DEBUG, "Connection: B2F-DBG connect state: is_initiator=%d local_data_turn=%d handshake_confirmed=%d interactive=%d",
              is_initiator_ ? 1 : 0, local_data_turn_ ? 1 : 0,
              handshake_confirmed_ ? 1 : 0, half_duplex_interactive_ ? 1 : 0);

    if (on_mode_negotiated_) {
        on_mode_negotiated_(negotiated_mode_);
    }

    if (on_connected_) {
        on_connected_();
    }

}

void Connection::enterDisconnected(const std::string& reason) {
    auto failed_message_records = detachOutboundMessageRecords();
    state_ = ConnectionState::DISCONNECTED;
    experimental_8psk_long_ldpc_transfer_active_ = false;
    experimental_qpsk_r34_long_ldpc_transfer_active_ = false;
    resetAdaptiveDecisionState();
    latent_bootstrap_rung_ = kRungIdxNone;  // no connect verdict may cross a QSO boundary
    staged_timeout_batch_.clear();
    arq_tick_in_progress_ = false;
    is_initiator_ = false;
    handshake_confirmed_ = false;
    burst_activity_ = BurstActivity{};  // clear the "incoming burst" GUI indicator
    setPhyMaskV1Negotiated(false);
    narrowband_override_ = WaveformMode::AUTO;  // Clear session-scoped narrowband override
    std::string old_remote = remote_call_;
    remote_call_.clear();
    pending_remote_call_.clear();
    outbound_forced_modulation_ = Modulation::AUTO;
    outbound_forced_code_rate_ = CodeRate::AUTO;
    mode_change_pending_ = false;
    desc_switch_full_anchor_pending_ = false;
    partial_sack_last_round_ = {};
    partial_sack_descriptor_repair_scope_ = false;
    rx_rate_cmd_pending_ = 0;       // RX-RATE-CMD: session-scoped
    rx_rate_cmd_seq_seen_ = -1;
    rebase_voice_event_seq_ = 0;
    rx_rebase_voice_seq_seen_ = -1;
    rx_authority_cmd_ = 0;
    tx_authority_last_obeyed_ = 0;
    rx_auth_obs_count_ = 0;
    rx_auth_obs_next_ = 0;
    rx_auth_crater_streak_ = 0;
    rx_auth_clean_streak_ = 0;
    rx_auth_class_sticky_ = 1;
    rx_auth_class_streak_ = 0;
    rx_auth_fading_passed_ = 0.3f;
    rx_auth_climb_dwell_ = 0;
    rx_auth_gamma_count_ = 0;
    rx_auth_gamma_next_ = 0;
    for (auto& v : rx_auth_gamma_ring_) v.clear();
    for (size_t i = 0; i < kRungIdxCount; ++i) rx_auth_rung_penalty_db_[i] = 0.0f;
    burst_obs_snr_db_ = -1.0f;
    burst_obs_fading_ = -1.0f;
    burst_obs_evm_snr_db_ = -1.0f;
    burst_obs_coh_valid_ = false;
    qam16_rx_bad_streak_ = 0;
    last_applied_mode_change_valid_ = false;  // MC dedup (fix 3) is session-scoped
    mode_change_ack_repeat_jobs_.clear();
    disconnect_frame_.clear();
    disconnect_retry_count_ = 0;
    disconnect_retransmit_ms_ = 0;
    timeout_remaining_ms_ = 0;
    disconnect_pending_ = false;
    disconnect_pending_ms_ = 0;
    disconnect_ack_retransmit_ms_ = 0;
    disconnect_ack_epoch_elapsed_ms_ = 0;
    disconnect_ack_repeat_count_ = 0;
    disconnect_ack_frame_.clear();
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    rx_level_low_streak_ = 0;   // software-ALC: per-connection
    rx_level_clipped_ = false;
    rx_level_verdict_pending_for_group_ = false;
    tone_ack_alc_group_context_ = false;
    tone_ack_group_has_decoded_data_context_ = false;
    responder_handshake_wait_ms_ = 0;
    connect_ack_frame_.clear();
    local_data_turn_ = false;
    peer_data_turn_requested_ = false;
    local_turn_request_pending_ = false;
    received_peer_data_since_connect_ = false;
    yielded_data_turn_waiting_for_peer_data_ = false;
    data_turn_yield_pending_ = false;
    resetDataTurnFairness();
    data_turn_tx_guard_ms_ = 0;
    turn_request_retransmit_ms_ = 0;
    turn_request_holdoff_ms_ = 0;
    file_cancel_rx_drain_ms_ = 0;
    clearFileCancelReassertion();
    file_cancel_confirm_pending_ = false;
    data_ladder_rung_id_ = LadderRungId::UNKNOWN;
    pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    arq_callback_defer_refill_ = false;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;
    deferred_arq_failure_abort_ = false;
    deferred_arq_failure_seq_valid_ = false;
    arq_.reset();
    // SelectiveRepeatARQ::reset intentionally preserves negotiated geometry.
    // A session boundary does not: no transfer-scoped Z=81 state may leak into
    // descriptor-less traffic in the next connection attempt.
    arq_.setFixedFrameGeometry(data_frame_cw_count_, /*lifting_z=*/27);
    soft_combine_harq_.clear();
    file_transfer_.cancel();
    queued_file_path_.reset();
    queued_file_order_ = 0;
    next_queued_operation_order_ = 1;
    queued_payloads_.clear();
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    pending_tx_fragment_message_tokens_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;
    rx_reassembly_buffer_.clear();
    rx_reassembly_active_ = false;
    rx_reassembly_binary_ = false;
    rx_reassembly_epoch_ = 0;
    // #58 increment 3: session boundary — nothing in the connect-SNR pool may leak
    // into the next handshake's entry pick.
    connect_snr_pool_.clear();
    connect_pick_deferred_once_ = false;

    // Reset connect waveform to DPSK for next connection attempt
    connect_waveform_ = WaveformMode::MC_DPSK;

    setDisconnectTeardownActive(false);

    LOG_MODEM(INFO, "Connection: Disconnected from %s (%s)",
              old_remote.c_str(), reason.c_str());

    emitFailedMessageRecords(failed_message_records);
    if (!failed_message_records.empty() && on_message_sent_) {
        on_message_sent_(false);
    }
    if (on_disconnected_) {
        on_disconnected_(reason);
    }
}

// =============================================================================
// CALLBACKS
// =============================================================================

void Connection::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void Connection::setTransmitInfoCallback(TransmitInfoCallback cb) {
    on_transmit_info_ = std::move(cb);
}

void Connection::setTransmitBurstCallback(TransmitBurstCallback cb) {
    on_transmit_burst_ = std::move(cb);
    if (on_transmit_burst_) {
        arq_.setDeferredTimeoutCommitEnabled(true);
        arq_.setTransmitBatchCallback([this](const std::vector<Bytes>& frames) {
            handleArqTimeoutBatch(frames);
        });
    } else {
        arq_.setTransmitBatchCallback(SelectiveRepeatARQ::TransmitBatchCallback{});
        arq_.setDeferredTimeoutCommitEnabled(false);
    }
}

void Connection::setMCDPSKConfig(int num_carriers, int samples_per_symbol) {
    config_.mc_dpsk_num_carriers = std::clamp(num_carriers, 1, 64);
    config_.mc_dpsk_samples_per_symbol = std::clamp(samples_per_symbol, 1, 8192);
}

void Connection::setPhyMaskV1Negotiated(bool enabled) {
    if (phy_mask_v1_negotiated_ == enabled) {
        return;
    }
    phy_mask_v1_negotiated_ = enabled;
    LOG_MODEM(INFO, "Connection: PHY_MASK_V1 %s",
              enabled ? "negotiated" : "disabled");
    if (on_phy_mask_v1_negotiated_) {
        on_phy_mask_v1_negotiated_(enabled);
    }
}

void Connection::flushBurstBuffer(bool retransmission_required,
                                  bool descriptor_only_partial_repair) {
    if (disconnect_teardown_active_) {
        burst_tx_buffer_.clear();
        return;
    }
    if (deferred_arq_failure_abort_) {
        LOG_MODEM(WARN,
                  "Connection: Discarding buffered DATA after terminal ARQ failure");
        burst_tx_buffer_.clear();
        return;
    }
    if (burst_tx_buffer_.empty()) return;

    const size_t real_frame_count = burst_tx_buffer_.size();
    const bool final_file_tail =
        file_transfer_.getState() == FileTransferState::SENDING &&
        isFinalDataFrame(burst_tx_buffer_.back());
    if (!final_file_tail &&
        shouldPadPartialOFDMBurst(negotiated_mode_,
                                  data_modulation_,
                                  data_code_rate_,
                                  fading_index_,
                                  measured_snr_db_,
                                  file_transfer_.getState(),
                                  real_frame_count)) {
        const size_t remainder =
            real_frame_count % connection_policy::burstInterleaveGroupFrames();
        const size_t pad_count =
            connection_policy::burstInterleaveGroupFrames() - remainder;
        const int pad_cw = physicalDataFrameCodewords();
        const int pad_z = selectBurstLiftingZ();
        for (size_t i = 0; i < pad_count; ++i) {
            auto pad_frame = v2::makeFixedDataFrame(
                local_call_,
                v2::kOFDMBurstPadCallsign,
                static_cast<uint16_t>(v2::kOFDMBurstPadSeq - i),
                makeOFDMBurstPadPayload(data_code_rate_, pad_cw, i, pad_z),
                data_code_rate_,
                pad_cw,
                pad_z).serialize();
            burst_tx_buffer_.push_back(pad_frame);
        }
        LOG_MODEM(INFO,
                  "Connection: Padded OFDM burst %zu -> %zu frames for burst interleaver",
                  real_frame_count,
                  burst_tx_buffer_.size());
    }

    // Long Z cannot ride a descriptor-less singleton. This applies to the
    // logical FINAL/tail as well as a one-hole selective retry; the added pad
    // is addressed away and exists only to preserve the physical wire group.
    appendExperimentalLongLDPCTailPad(burst_tx_buffer_);

    // This is the first point where the physical round geometry is exact: optional
    // interleave padding has been appended, and nothing else below changes the frame
    // count. Re-time only the matching live ARQ slots and arm once before handing the
    // group to a transport callback (which is allowed to synchronously loop back ACKs).
    const bool expects_data_ack = std::any_of(
        burst_tx_buffer_.begin(), burst_tx_buffer_.end(), [](const Bytes& frame) {
            const auto header = v2::parseHeader(frame);
            return header.valid && !header.is_control;
        });
    const bool descriptor_bearing_burst =
        burst_tx_buffer_.size() > 1 && on_transmit_burst_;
    const bool desc_switch_anchor =
        descriptor_bearing_burst && desc_switch_full_anchor_pending_;
    const bool use_descriptor_only_partial_repair =
        partial_sack_descriptor_repair_enabled_ &&
        descriptor_bearing_burst && retransmission_required &&
        descriptor_only_partial_repair && !desc_switch_anchor;
    const bool force_full_group_start =
        descriptor_bearing_burst &&
        ((retransmission_required && !use_descriptor_only_partial_repair) ||
         desc_switch_anchor);
    const uint8_t anchor_reason = use_descriptor_only_partial_repair
        ? kAnchorReasonProvenPartialRepair
        : (retransmission_required
               ? kAnchorReasonResend
               : (desc_switch_anchor ? kAnchorReasonModeSwitch
                                     : kAnchorReasonNone));
    const uint32_t queue_delay_ms = noteDataBurstKeydown(
        burst_tx_buffer_, force_full_group_start);
    const uint32_t round_timeout_ms = finalizeUnifiedBurstWindow(
        burst_tx_buffer_, queue_delay_ms, force_full_group_start);

    // Capture the exact, already-padded physical geometry before any host callback
    // can synchronously return its ACK.  Only normal/light or proven descriptor-only
    // rounds may seed another descriptor-only decision. A full resend, mode switch,
    // singleton, fallback, or non-wide mode invalidates the provenance chain.
    partial_sack_last_round_ = {};
    if (expects_data_ack && descriptor_bearing_burst &&
        negotiated_mode_ == WaveformMode::OFDM_CHIRP && kUnifiedSeqEnabled()) {
        partial_sack_last_round_.descriptor_light_repair_eligible =
            anchor_reason == kAnchorReasonNone ||
            anchor_reason == kAnchorReasonProvenPartialRepair;
        partial_sack_last_round_.physical_frames = burst_tx_buffer_.size();
        partial_sack_last_round_.arq_frames = static_cast<size_t>(std::count_if(
            burst_tx_buffer_.begin(), burst_tx_buffer_.end(), [](const Bytes& frame) {
                const auto header = v2::parseHeader(frame);
                return header.valid && !header.is_control &&
                       !v2::isOFDMBurstPadHeader(header);
            }));
        partial_sack_last_round_.arq_frame_identities.reserve(
            partial_sack_last_round_.arq_frames);
        for (const auto& frame : burst_tx_buffer_) {
            const auto header = v2::parseHeader(frame);
            if (header.valid && !header.is_control &&
                !v2::isOFDMBurstPadHeader(header)) {
                partial_sack_last_round_.arq_frame_identities.push_back(frame);
            }
        }
        partial_sack_last_round_.mode = negotiated_mode_;
        partial_sack_last_round_.modulation = data_modulation_;
        partial_sack_last_round_.code_rate = data_code_rate_;
        partial_sack_last_round_.logical_cw = data_frame_cw_count_;
        partial_sack_last_round_.physical_cw = physicalDataFrameCodewords();
        partial_sack_last_round_.lifting_z = selectBurstLiftingZ();
    }
    if (expects_data_ack) {
        // Wide OFDM supplies its exact serialized-round deadline.  Narrow OFDM and
        // MC-DPSK deliberately return zero and retain their waveform-specific scalar
        // ARQ/monitor timeout.  Buffering suppresses the per-frame arm in the ARQ
        // transmit callback, so arm exactly once here for every physical group.
        armToneBurstAckListenWindow(round_timeout_ms);
    }

    // Detach the committed physical round before entering a host callback. A test
    // transport may synchronously return an ACK, and that ACK is allowed to build the
    // next burst recursively; keeping the outer round in the shared member would let
    // the nested refill clear/overwrite it (or let the outer cleanup erase the refill).
    std::vector<Bytes> committed_frames;
    committed_frames.swap(burst_tx_buffer_);

    if (committed_frames.size() == 1 && on_transmit_) {
        // Single frame, no burst needed. NOTE: desc_switch_full_anchor_pending_ is
        // deliberately NOT consumed here — a single-frame send carries no BURST_HEADER
        // descriptor (encodeBurstLight:476-489), so the one-shot stays armed for the
        // next descriptor-bearing (multi-frame) burst.
        on_transmit_(committed_frames[0]);
    } else if (on_transmit_burst_) {
        LOG_MODEM(INFO, "Connection: Flushing burst of %zu frames", committed_frames.size());
        // DESC-SWITCH §5.1 step 2: the first burst group after a descriptor-committed
        // mode switch carries the full chirp+LTS anchor (one-shot; the §2.6-arm-3
        // geometry-change mitigation). The anchor reason stays bound to this physical
        // request through the frontend and resets the encoder's anchor-skip clean streak
        // at encode time (warm_descriptor=false path).
        desc_switch_full_anchor_pending_ = false;
        // An ACK-revealed hole is just as much negative delivery evidence as an
        // RTO.  The receiver already closed the failed group and deliberately
        // overrides that descriptor's stale NEXT_LIGHT bit.  Mark this refill as
        // a resend so both the descriptor and group start carry a full anchor and
        // the encoder's delivery-earned skip streak is recooled.  A resend takes
        // precedence over the weaker mode-switch reason when both coincide.
        if (use_descriptor_only_partial_repair) {
            LOG_MODEM(WARN,
                      "Connection: PARTIAL-SACK descriptor-only repair ENGAGED: "
                      "%zu frames, robust descriptor + light DATA group",
                      committed_frames.size());
        }
        on_transmit_burst_(committed_frames, /*group_seq=*/0,
                           anchor_reason);
    } else if (on_transmit_) {
        // Fallback: send individually
        for (const auto& frame : committed_frames) {
            on_transmit_(frame);
        }
    }
}

void Connection::handleArqTimeoutBatch(const std::vector<Bytes>& frame_data_list) {
    if (disconnect_teardown_active_ || frame_data_list.empty()) return;
    if (!arq_tick_in_progress_) {
        transmitFrameBatch(frame_data_list);
        return;
    }

    // One ARQ tick produces at most one timeout batch. Account the missing ACK now so
    // maybeCollapseEscape() can consume this very round, but delay every irreversible
    // physical side effect (play-head, timeout commit, monitor arm, encoder queue).
    noteArqRoundOutcome(0, "rto");
    staged_timeout_batch_ = frame_data_list;
    staged_timeout_mod_ = data_modulation_;
    staged_timeout_rate_ = data_code_rate_;
    staged_timeout_cw_ = data_frame_cw_count_;
    if (tx_latent_startup_probe_active_ && tx_latent_startup_probe_airborne_ &&
        coherentRungIndexFor(data_modulation_, data_code_rate_) == kRungIdxQpskR23) {
        // Do not commit this higher-rung retransmission. The post-ARQ-tick handler
        // starts a robust synchronized rollback; the normal staged-batch flush then
        // cancels/suspends these old-geometry intents under mode_change_pending_.
        tx_latent_startup_probe_timeout_rollback_pending_ = true;
    }
    LOG_MODEM(DEBUG,
              "Connection: Staged ARQ timeout batch of %zu frame(s) pending escape check",
              staged_timeout_batch_.size());
}

void Connection::handleLatentStartupProbeTimeoutRollback() {
    if (!tx_latent_startup_probe_timeout_rollback_pending_) return;

    tx_latent_startup_probe_timeout_rollback_pending_ = false;
    const uint8_t base = tx_latent_startup_probe_base_rung_;
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }
    if (mode_change_pending_) {
        // Never erase the probe identity and silently return here. The peer may have
        // applied either this target descriptor or the unrelated pending control; there
        // is no safe way to overwrite/queue a second MODE_CHANGE while preserving order.
        // Disconnect leaves the staged target timeout obsolete and prevents any refill.
        LOG_MODEM(ERROR,
                  "Connection: LATENT startup-probe RTO collided with pending "
                  "MODE_CHANGE; refusing ambiguous geometry and disconnecting");
        disconnect();
        return;
    }
    if (base == kRungIdxNone || base >= kRungIdxCount) {
        LOG_MODEM(ERROR,
                  "Connection: LATENT startup-probe RTO has no valid base rung; "
                  "disconnecting instead of retransmitting target DATA");
        disconnect();
        return;
    }
    const CoherentPick rollback = coherentRungFromIndex(base);
    if (rollback.mod == data_modulation_ && rollback.rate == data_code_rate_) {
        tx_latent_startup_probe_active_ = false;
        tx_latent_startup_probe_airborne_ = false;
        tx_latent_startup_probe_base_rung_ = kRungIdxNone;
        return;
    }

    LOG_MODEM(WARN,
              "Connection: LATENT startup probe ACK timeout — suppressing target "
              "retransmit and synchronizing rollback to idx %u",
              base);
    // ACK silence does not prove whether the receiver adopted the target descriptor.
    // Always use the robust control-plane handshake, even when move-epoch is enabled.
    requestModeChange(rollback.mod, rollback.rate, wireSnrDb(),
                      v2::ModeChangeReason::STARTUP_PROBE_TIMEOUT);
    if (!mode_change_pending_ ||
        pending_reason_ != v2::ModeChangeReason::STARTUP_PROBE_TIMEOUT) {
        LOG_MODEM(ERROR,
                  "Connection: LATENT startup-probe rollback could not be armed; "
                  "disconnecting before target DATA can refill");
        disconnect();
        return;
    }
    // The synchronized rollback now owns egress and its special retry-exhaustion path
    // disconnects instead of resuming target intents. It is safe to disarm the physical
    // probe interceptor only after that ownership is established.
    tx_latent_startup_probe_active_ = false;
    tx_latent_startup_probe_airborne_ = false;
    tx_latent_startup_probe_base_rung_ = kRungIdxNone;
}

void Connection::flushStagedArqTimeoutBatch() {
    if (staged_timeout_batch_.empty()) return;

    const bool obsolete =
        state_ != ConnectionState::CONNECTED || deferred_arq_failure_abort_ ||
        mode_change_pending_ || data_modulation_ != staged_timeout_mod_ ||
        data_code_rate_ != staged_timeout_rate_ ||
        data_frame_cw_count_ != staged_timeout_cw_;
    if (obsolete) {
        std::vector<Bytes> canceled;
        canceled.swap(staged_timeout_batch_);
        // A legacy synchronized MODE_CHANGE keeps the old ARQ identities live while
        // control-plane agreement is pending. Suspend exactly these expired intents so
        // repeated ticks cannot burn retries or launch DATA under the control exchange.
        // Successful regrid invalidates them by exact identity; give-up resumes them.
        const bool suspend_for_mode_change =
            state_ == ConnectionState::CONNECTED && mode_change_pending_ &&
            !deferred_arq_failure_abort_;
        const size_t matched = arq_.cancelDeferredTimeoutRetransmits(
            canceled, suspend_for_mode_change);
        LOG_MODEM(WARN,
                  "Connection: Discarding staged old-geometry timeout batch (%zu frame(s)); "
                  "matched=%zu suspended=%d; mode/escape boundary owns next egress",
                  canceled.size(), matched, suspend_for_mode_change ? 1 : 0);
        return;
    }

    std::vector<Bytes> intents;
    intents.swap(staged_timeout_batch_);
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP && kUnifiedSeqEnabled()) {
        const size_t repair_cap =
            burstAirtimeBudgetFrames(arq_.getWindowSize(),
                                     /*force_full_group_start=*/true);
        if (intents.size() > repair_cap) {
            LOG_MODEM(INFO,
                      "Connection: Capping full-anchor timeout repair %zu -> %zu "
                      "frame(s) to the physical burst ceiling",
                      intents.size(), repair_cap);
            // Omitted expired intents remain live without spending retry budget.
            // finalizeUnifiedBurstWindow() suspends them behind this stop-and-wait
            // physical round; ACK-driven refill emits them on the next turn.
            intents.resize(repair_cap);
        }
    }
    auto committed = arq_.commitDeferredTimeoutRetransmits(intents);
    if (committed.terminal_failure) {
        // The terminal callback is deliberately deferred to this post-escape commit
        // boundary. Drain it NOW; the earlier post-tick drain ran before commit and
        // therefore could not observe it.
        drainDeferredArqFailureAbort();
        return;
    }
    if (!committed.frames.empty()) {
        transmitFrameBatch(committed.frames, /*account_rto_round=*/false);
    }
}

void Connection::transmitFrameBatch(const std::vector<Bytes>& frame_data_list,
                                    bool account_rto_round) {
    if (disconnect_teardown_active_ || frame_data_list.empty()) {
        return;
    }
    if (deferred_arq_failure_abort_) {
        // ARQ tick may assemble a timeout batch after one slot in that same scan
        // terminal-fails. The failure callback already owns the logical outcome;
        // never key residual retries before the post-tick abort drains the window.
        LOG_MODEM(WARN,
                  "Connection: Suppressing timeout batch after terminal ARQ failure");
        return;
    }

    std::vector<Bytes> physical_frames = frame_data_list;
    appendExperimentalLongLDPCTailPad(physical_frames);
    // Lost-ACK/RTO egress is never eligible for the descriptor-only shortcut,
    // even if a delayed progress-bearing ACK arrives while this full repair is live.
    partial_sack_last_round_ = {};
    partial_sack_descriptor_repair_scope_ = false;

    // §RETX-PACING §1.1: this callback fires ONLY as the ARQ slot-RTO batch (arq_.tick →
    // transmitDataBatch), and an RTO round is zero-progress BY DEFINITION — a timeout IS
    // the absence of an ack. Account the round BEFORE stamping this resend's own key-down
    // end time (the elapsed-listening subtraction must reference the PREVIOUS burst). The
    // collapse escape itself is polled from the CONNECTED tick, never fired from inside
    // this ARQ transmit callback.
    if (account_rto_round) {
        noteArqRoundOutcome(0, "rto");
    }
    const bool burst_capable_mode =
        isOFDMMode(negotiated_mode_) || negotiated_mode_ == WaveformMode::MC_DPSK;
    const bool force_full_group_start =
        physical_frames.size() > 1 && on_transmit_burst_ && burst_capable_mode;
    const uint32_t queue_delay_ms = noteDataBurstKeydown(
        physical_frames, force_full_group_start);

    // A timeout batch is a new physical round and may contain only a subset of the
    // still-open window. Commit the exact n-frame deadline to those identities only;
    // unsent holes retain their previous countdown.
    const uint32_t round_timeout_ms = finalizeUnifiedBurstWindow(
        physical_frames, queue_delay_ms, force_full_group_start);
    const bool expects_data_ack = std::any_of(
        physical_frames.begin(), physical_frames.end(), [](const Bytes& frame) {
            const auto header = v2::parseHeader(frame);
            return header.valid && !header.is_control;
        });
    if (expects_data_ack) {
        armToneBurstAckListenWindow(round_timeout_ms);
    }

    if (physical_frames.size() == 1 || !on_transmit_burst_ || !burst_capable_mode) {
        for (const auto& frame_data : physical_frames) {
            transmitFrame(frame_data);
        }
        return;
    }

    // A timed-out burst is resent as a WHOLE RE-INTERLEAVED BURST on one anchor —
    // NOT as N standalone full-anchor frames (the old SR-ARQ "standalone repair
    // anchors" path, §14.20 throw). Standalone repair frames threw away the
    // cross-frame burst interleaving (so the resend had no fade diversity and was
    // just as likely to fail on the next fade) AND paid N chirp anchors instead of
    // one — the dominant goodput sink on Good fading. encodeBurstLight re-interleaves
    // the group, so the resend gets fresh fade diversity at ~1/N the preamble cost.
    //
    // RESEND USES A FULL CHIRP+LTS ANCHOR (force_full_preamble=true), not warm light-LTS.
    // A timeout almost always means the receiver MISSED the group's light-LTS acquisition at
    // the warm-handoff boundary — it never completed the group, so it never acked. On that
    // miss the RX side ALREADY arms expect_full_anchor=1 ("waiting for a full chirp on the
    // resend", §16.4 escalation). Resending with light-LTS again just re-misses the same
    // preamble; a full chirp re-acquires deterministically. Costs ~1.4 s extra airtime ONLY
    // on resends (rare) — first-attempt groups keep warm light-LTS (+goodput). Restores the
    // pre-unification reliability coupling the merge dropped when it hardcoded this to false.
    LOG_MODEM(INFO, "Connection: Resending ARQ timeout-repair as re-interleaved burst of %zu frames (full anchor)",
              physical_frames.size());
    on_transmit_burst_(physical_frames, /*group_seq=*/0,
                       /*anchor_reason=*/kAnchorReasonResend);  // full chirp re-anchor +
                                                              // recool the skip streak
}

void Connection::setConnectedCallback(ConnectedCallback cb) {
    on_connected_ = std::move(cb);
}

void Connection::setDisconnectedCallback(DisconnectedCallback cb) {
    on_disconnected_ = std::move(cb);
}

void Connection::setDisconnectTeardownCallback(DisconnectTeardownCallback cb) {
    on_disconnect_teardown_ = std::move(cb);
    // Registration can occur while a session is already closing (notably a TNC
    // rebind).  Publish the current level immediately instead of requiring an edge.
    if (on_disconnect_teardown_) {
        on_disconnect_teardown_(disconnect_teardown_active_);
    }
}

void Connection::setMessageReceivedCallback(MessageReceivedCallback cb) {
    on_message_received_ = std::move(cb);
}

void Connection::setMessageSentCallback(MessageSentCallback cb) {
    on_message_sent_ = std::move(cb);
}

void Connection::setMessageTxStatusCallback(MessageTxStatusCallback cb) {
    on_message_tx_status_ = std::move(cb);
}

void Connection::setIncomingCallCallback(IncomingCallCallback cb) {
    on_incoming_call_ = std::move(cb);
}

void Connection::setDataReceivedCallback(DataReceivedCallback cb) {
    on_data_received_ = std::move(cb);
}

void Connection::setFileProgressCallback(FileProgressCallback cb) {
    file_transfer_.setProgressCallback(std::move(cb));
}

void Connection::setFileReceivedCallback(FileReceivedCallback cb) {
    // Wrap so the "incoming burst" GUI indicator clears the moment the file finishes
    // (success or fail), not only on disconnect.
    file_transfer_.setReceivedCallback(
        [this, cb = std::move(cb)](const std::string& path, bool success,
                                   const std::string& error) {
            burst_activity_ = BurstActivity{};
            if (cb) cb(path, success, error);
        });
}

void Connection::setFileSentCallback(FileSentCallback cb) {
    // The controller-owned wrapper is installed in the constructor so internal
    // transfer cleanup still runs when the application does not register a
    // callback. Store only the outward notification here.
    on_file_sent_ = std::move(cb);
}

// =============================================================================
// STATS & RESET
// =============================================================================

ConnectionStats Connection::getStats() const {
    ConnectionStats s = stats_;
    s.arq = arq_.getStats();
    return s;
}

void Connection::resetStats() {
    stats_ = ConnectionStats{};
    arq_.resetStats();
}

void Connection::setSoftCombiningHARQ(bool enable) {
    soft_combine_harq_.setEnabled(enable);
    LOG_MODEM(INFO, "Connection: soft-combining HARQ %s",
              enable ? "ENABLED" : "disabled");
}

std::optional<fec::SoftCombineBuffer::ProvisionalContext>
Connection::harqProvisionalContext() const {
    if (state_ != ConnectionState::CONNECTED ||
        remote_call_.empty() ||
        !soft_combine_harq_.enabled()) {
        return std::nullopt;
    }

    fec::SoftCombineBuffer::ProvisionalContext ctx;
    ctx.sender_hash = v2::hashCallsign(remote_call_);
    ctx.dst_hash = v2::hashCallsign(local_call_);
    ctx.seq = arq_.getRxBaseSeq();
    ctx.window_size = arq_.getWindowSize();
    // The receiver's mirror of the sender's next-burst fill (see
    // predictedIncomingSeqs) — indexed by burst logical position.
    ctx.predicted_seqs = arq_.predictedIncomingSeqs(arq_.getWindowSize());
    if (!ctx.valid()) {
        return std::nullopt;
    }
    return ctx;
}

void Connection::reset() {
    auto failed_message_records = detachOutboundMessageRecords();
    state_ = ConnectionState::DISCONNECTED;
    experimental_8psk_long_ldpc_transfer_active_ = false;
    experimental_qpsk_r34_long_ldpc_transfer_active_ = false;
    resetAdaptiveDecisionState();
    latent_bootstrap_rung_ = kRungIdxNone;
    staged_timeout_batch_.clear();
    arq_tick_in_progress_ = false;
    is_initiator_ = false;
    handshake_confirmed_ = false;
    setPhyMaskV1Negotiated(false);
    remote_call_.clear();
    pending_remote_call_.clear();
    timeout_remaining_ms_ = 0;
    connect_retry_count_ = 0;
    connected_time_ms_ = 0;
    narrowband_override_ = WaveformMode::AUTO;  // Clear session-scoped narrowband override
    negotiated_mode_ = WaveformMode::OFDM_CHIRP;
    remote_capabilities_ = ModeCapabilities::OFDM_CHIRP;
    remote_preferred_ = WaveformMode::OFDM_CHIRP;
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    desc_switch_full_anchor_pending_ = false;
    partial_sack_last_round_ = {};
    partial_sack_descriptor_repair_scope_ = false;
    mode_change_ack_repeat_jobs_.clear();
    data_modulation_ = Modulation::DQPSK;
    data_code_rate_ = CodeRate::R1_4;
    data_ladder_rung_id_ = LadderRungId::UNKNOWN;
    connect_waveform_ = WaveformMode::MC_DPSK;  // Reset to DPSK for next connect attempt
    responder_handshake_wait_ms_ = 0;
    connect_ack_frame_.clear();
    disconnect_frame_.clear();
    disconnect_retry_count_ = 0;
    disconnect_retransmit_ms_ = 0;
    disconnect_pending_ = false;
    disconnect_pending_ms_ = 0;
    disconnect_ack_retransmit_ms_ = 0;
    disconnect_ack_epoch_elapsed_ms_ = 0;
    disconnect_ack_repeat_count_ = 0;
    disconnect_ack_frame_.clear();
    local_data_turn_ = false;
    peer_data_turn_requested_ = false;
    local_turn_request_pending_ = false;
    received_peer_data_since_connect_ = false;
    yielded_data_turn_waiting_for_peer_data_ = false;
    data_turn_yield_pending_ = false;
    resetDataTurnFairness();
    data_turn_tx_guard_ms_ = 0;
    turn_request_retransmit_ms_ = 0;
    turn_request_holdoff_ms_ = 0;
    file_cancel_rx_drain_ms_ = 0;
    clearFileCancelReassertion();
    file_cancel_confirm_pending_ = false;
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    arq_callback_defer_refill_ = false;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;
    deferred_arq_failure_abort_ = false;
    deferred_arq_failure_seq_valid_ = false;
    arq_.reset();
    arq_.setFixedFrameGeometry(data_frame_cw_count_, /*lifting_z=*/27);
    soft_combine_harq_.clear();
    file_transfer_.cancel();
    queued_file_path_.reset();
    queued_file_order_ = 0;
    next_queued_operation_order_ = 1;
    queued_payloads_.clear();
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    pending_tx_fragment_message_tokens_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;
    rx_reassembly_buffer_.clear();
    rx_reassembly_active_ = false;
    rx_reassembly_binary_ = false;
    rx_reassembly_epoch_ = 0;
    // Per-connection channel-coherence verdict (see setChannelCoherence hold-last-valid).
    coherence_score_ = 0.0f;
    coherence_doppler_hz_ = 0.0f;
    coherence_valid_ = false;
    // §RETX-PACING: trough-pacing / collapse-escape round state is per-connection.
    zero_progress_rounds_ = 0;
    trough_episode_active_ = false;  // trough-amnesty episode dies with the era
    retx_pace_hold_ms_ = 0;
    last_data_burst_end_valid_ = false;
    rebase_voice_event_seq_ = 0;
    rx_rebase_voice_seq_seen_ = -1;
    ladder_telemetry_active_ = false;
    // #58 increment 3: the connect-SNR pool is per-session (its entry horizon IS the
    // handshake scope); the defer one-shot re-arms for the next handshake.
    connect_snr_pool_.clear();
    connect_pick_deferred_once_ = false;
    setDisconnectTeardownActive(false);
    emitFailedMessageRecords(failed_message_records);
    if (!failed_message_records.empty() && on_message_sent_) {
        on_message_sent_(false);
    }
    LOG_MODEM(DEBUG, "Connection: Full reset");
}

} // namespace protocol
} // namespace ultra
