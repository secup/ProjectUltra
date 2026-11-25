/**
 * RX-AUTHORITY regression suite (ULTRA_RX_RATE_AUTHORITY, 2026-07-05)
 *
 * The receiver measures, the receiver decides: per burst group it maps its fresh
 * channel observation through selectCoherentOFDM and commands the sender's next
 * rung (absolute canonical index in the ACK's [rate_hint|rung_cmd] bits). The
 * sender obeys; its own mid-transfer drivers are inert under the knob.
 *
 * Own binary: rxRateAuthorityEnabled()/psk8LadderEnabled() latch env at first
 * call, so setenv() runs first in main().
 */

#include "env_compat.hpp"
#include "protocol/connection.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::protocol;

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { std::cout << "  Testing " << name << "... " << std::flush; tests_run++; } while (0)
#define PASS() \
    do { std::cout << "PASS\n"; tests_passed++; } while (0)
#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << "\n"; return false; } while (0)

namespace ultra {
namespace protocol {

struct ConnectionAdaptiveTestAccess {
    static void makeLocalIss(Connection& c) {
        c.turn_.setLocalHolds(true);
        c.turn_.setPeerRequested(false);
        c.turn_.setRequestPending(false);
        c.turn_.setPeerDataSinceConnect(false);
        c.turn_.setYieldPending(false);
        c.data_turn_payload_bytes_sent_ = 0;
        c.data_turn_contended_ms_ = 0;
        c.data_turn_tx_guard_ms_ = 0;
        c.turn_request_retransmit_ms_ = 0;
        c.turn_request_holdoff_ms_ = 0;
    }
    static void makeConnectedOFDM(Connection& c, CodeRate rate, float snr,
                                  float fading, Modulation modulation) {
        c.local_call_ = "W1ABC";
        c.remote_call_ = "K2DEF";
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = true;
        c.handshake_confirmed_ = true;
        c.negotiated_mode_ = WaveformMode::OFDM_CHIRP;
        c.data_modulation_ = modulation;
        c.data_code_rate_ = rate;
        c.measured_snr_db_ = snr;
        c.fading_index_ = fading;
        makeLocalIss(c);
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
        c.configureArqForCurrentDataMode();
    }
    static void verdict(Connection& c, bool all_ok, float quality) {
        c.updateRxAuthorityCommand(all_ok, quality);
    }
    static void verdictFull(Connection& c, bool all_ok, float quality, bool full_crater) {
        c.updateRxAuthorityCommand(all_ok, quality, full_crater);
    }
    static void verdictGraded(Connection& c, bool all_ok, float quality, bool full_crater,
                              float delivered_fraction) {
        c.updateRxAuthorityCommand(all_ok, quality, full_crater, delivered_fraction);
    }
    static uint8_t rxCmd(const Connection& c) { return c.rx_authority_cmd_; }
    // GROUP-SIZE gate (ULTRA_BURST_ESC_STREAK): drive the clean-round streak and read
    // back the frame budget the airtime ceiling allows.
    static void setCleanStreak(Connection& c, int n) { c.burst_clean_group_streak_ = n; }
    static size_t burstFrames(const Connection& c, size_t want) {
        return c.burstAirtimeBudgetFrames(want);
    }
    static void obey(Connection& c, uint8_t idx) { c.maybeObeyAuthorityCommand(idx); }
    static void setStartupWireState(Connection& c, bool waiting, uint8_t cmd) {
        c.latent_startup_probe_waiting_ = waiting;
        c.latent_startup_probe_spent_ = false;
        c.latent_startup_probe_failed_ = false;
        c.rx_authority_cmd_ = cmd;
    }
    static void emitToneSack(Connection& c, bool completed_group) {
        c.tone_ack_group_complete_context_ = completed_group;
        c.arq_.beginGroupReceive();
        c.arq_.endGroupReceiveAndAck();
        c.tone_ack_group_complete_context_ = false;
    }
    static bool startupWaiting(const Connection& c) {
        return c.latent_startup_probe_waiting_;
    }
    static bool startupFailed(const Connection& c) {
        return c.latent_startup_probe_failed_;
    }
    static bool acceptAuthorityRevision(Connection& c, uint8_t cmd_word,
                                        uint8_t group_seq, uint16_t mask,
                                        uint8_t epoch) {
        return c.acceptAuthorityCommandRevision(
            cmd_word, group_seq, mask, epoch);
    }
    static bool modeChangePending(const Connection& c) { return c.mode_change_pending_; }
    static Modulation pendingModulation(const Connection& c) { return c.pending_modulation_; }
    static CodeRate pendingCodeRate(const Connection& c) { return c.pending_code_rate_; }
    static void fillArq(Connection& c) {
        c.arq_.sendData(Bytes{0x01, 0x02, 0x03});  // one frame in flight = busy
    }
    // Simulate the sender obeying + the receiver adopting the standing command
    // (the real loop moves `cur`; without this every verdict re-climbs from the
    // same rung and dwell/one-rung assertions measure the wrong thing).
    static void adoptCmd(Connection& c) {
        if (c.rx_authority_cmd_ == kRungIdxNone) return;
        const CoherentPick p = coherentRungFromIndex(c.rx_authority_cmd_);
        c.data_modulation_ = p.mod;
        c.data_code_rate_ = p.rate;
    }
};

}  // namespace protocol
}  // namespace ultra

using TA = ConnectionAdaptiveTestAccess;

// ── RX-AUTHORITY PREDICTIVE (docs/RX_AUTHORITY_PREDICTIVE_2026_07_07.md) ──

// EESM self-calibration identity: on a FLAT channel the predictor must
// reproduce the anchor table exactly — pass at the anchor, fail below it.
// test_prediction_flat_identity / test_prediction_rejects_notched_channel — DELETED
// 2026-08-01 with rungPredictedSustainable(). Its only production caller was the
// RX-authority predictive climb (retired), so the predictor had no reader left outside
// these tests. See docs/REMOVAL_BACKLOG.md R12.

// Boundary asymmetry: with data in flight, an UP command defers (no mode change);
// a DOWN command obeys immediately.
static bool test_busy_defers_up_not_down() {
    TEST("busy window: climb defers, demote obeys");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM8);
    // Put a frame in flight directly through the friend'd ARQ.
    TA::fillArq(c);
    TA::obey(c, kRungIdxQam16R23);  // UP from QAM8 R2/3
    if (TA::modeChangePending(c)) FAIL("climb fired mid-window");
    TA::obey(c, kRungIdxQpskR23);   // DOWN
    if (!TA::modeChangePending(c)) FAIL("demote deferred mid-window");
    if (TA::pendingModulation(c) != Modulation::QPSK ||
        TA::pendingCodeRate(c) != CodeRate::R2_3)
        FAIL("demote target wrong");
    PASS();
    return true;
}

// Clean override: a clean group must never command below the rung that just worked.
static bool test_clean_never_commands_down() {
    TEST("clean group holds the working rung against a stale-low reading");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM8);
    c.setBurstChannelObservation(11.0f, 0.30f, 0.8f, true, 0.2f);  // stale-low meter
    TA::verdict(c, /*all_ok=*/true, /*quality=*/0.95f);
    const uint8_t cur = coherentRungIndexFor(Modulation::QAM8, CodeRate::R2_3);
    if (TA::rxCmd(c) < cur)
        FAIL("clean-group verdict commanded below the working rung");
    PASS();
    return true;
}

// No observation yet -> no command (never steer on handshake-stale state).
static bool test_no_observation_no_command() {
    TEST("no fresh observation = no command");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
    TA::verdict(c, true, 0.9f);
    if (TA::rxCmd(c) != kRungIdxNone) FAIL("commanded without an observation");
    PASS();
    return true;
}

// Sender obey: a commanded rung asserts the mode move; repeats dedup; arrival at
// the target re-arms.
static bool test_sender_obeys_and_dedups() {
    TEST("sender obeys the command once and dedups repeats");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
    TA::obey(c, kRungIdxQam8R23);
    if (!TA::modeChangePending(c)) FAIL("command did not assert a mode change");
    if (TA::pendingModulation(c) != Modulation::QAM8 ||
        TA::pendingCodeRate(c) != CodeRate::R2_3)
        FAIL("obeyed target was not QAM8 R2/3");
    // A repeated copy while the exchange is pending must be a no-op (guard is the
    // mode_change_pending_ early-return + the last-obeyed dedup).
    TA::obey(c, kRungIdxQam8R23);
    if (!TA::modeChangePending(c)) FAIL("repeat handling clobbered the pending move");
    PASS();
    return true;
}

// Sender obey: command 0 and out-of-range indices are inert.
static bool test_sender_ignores_none_and_garbage() {
    TEST("cmd 0 / out-of-range are inert");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
    TA::obey(c, kRungIdxNone);
    TA::obey(c, 31);
    TA::obey(c, kRungIdxCount);
    if (TA::modeChangePending(c)) FAIL("inert command asserted a mode change");
    PASS();
    return true;
}

// A fixed-rung probe is an operator command, not an advisory. Receiver authority
// must not move the sender while ULTRA_LOCK_RATE is active.
static bool test_operator_lock_overrides_authority() {
    TEST("operator adaptation controls override receiver authority");

    {
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
        setenv("ULTRA_LOCK_RATE", "1", 1);
        TA::obey(c, kRungIdxQam8R23);
        unsetenv("ULTRA_LOCK_RATE");
        if (TA::modeChangePending(c)) FAIL("authority bypassed ULTRA_LOCK_RATE");
    }
    {
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
        setenv("ULTRA_RATE_ADAPT", "0", 1);
        TA::obey(c, kRungIdxQam8R23);
        unsetenv("ULTRA_RATE_ADAPT");
        if (TA::modeChangePending(c)) FAIL("authority bypassed ULTRA_RATE_ADAPT=0");
    }
    {
        setenv("ULTRA_ADAPTIVE_RATE", "0", 1);
        Connection c;
        unsetenv("ULTRA_ADAPTIVE_RATE");
        TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
        TA::obey(c, kRungIdxQam8R23);
        if (TA::modeChangePending(c)) FAIL("authority bypassed ULTRA_ADAPTIVE_RATE=0");
    }
    PASS();
    return true;
}

// ── RADIO-AGNOSTIC EVM DEMOTE (ULTRA_EVM_DEMOTE, Stage 2) ───────────────────────
// The measured OTASim AWGN anchor floors (2026-07-24): QPSK R2/3 9.4, R3/4 12.5,
// 8PSK R2/3 13.7, 16QAM R1/2 14.9, 16QAM R2/3 16.7 dB usable EVM. The table must
// stay monotonic across the measured rungs, and highestRungSupportedByEvm must map a
// usable EVM to the densest rung it clears (with the 1 dB hold margin).
static bool test_evm_floor_table_and_mapping() {
    TEST("EVM floor table monotonic; supported-rung mapping tracks the anchors");
    // Measured rungs increase monotonically in usable-EVM floor.
    const float f_qpsk23 = evmUsableFloorDbForRung(kRungIdxQpskR23);
    const float f_qpsk34 = evmUsableFloorDbForRung(kRungIdxQpskR34);
    const float f_8psk23 = evmUsableFloorDbForRung(kRungIdxQam8R23);
    const float f_16q12 = evmUsableFloorDbForRung(kRungIdxQam16R12);
    const float f_16q23 = evmUsableFloorDbForRung(kRungIdxQam16R23);
    if (!(f_qpsk23 < f_qpsk34 && f_qpsk34 < f_8psk23 &&
          f_8psk23 < f_16q12 && f_16q12 < f_16q23))
        FAIL("measured EVM floors are not monotonic across the rung ladder");
    // A usable EVM of 20 dB clears 16QAM R2/3 (16.7); 14 dB does not (needs 15.7 with
    // margin) but clears 16QAM R1/2 (14.9-1=13.9); 10 dB only QPSK R2/3 (9.4-1=8.4);
    // 5 dB floors at QPSK R1/4.
    if (highestRungSupportedByEvm(20.0f, kEvmDemoteHoldMarginDb) < kRungIdxQam16R23)
        FAIL("20 dB usable EVM must support 16QAM R2/3");
    if (highestRungSupportedByEvm(14.0f, kEvmDemoteHoldMarginDb) != kRungIdxQam16R12)
        FAIL("14 dB usable EVM must support 16QAM R1/2 (not R2/3)");
    if (highestRungSupportedByEvm(10.0f, kEvmDemoteHoldMarginDb) != kRungIdxQpskR23)
        FAIL("10 dB usable EVM must support QPSK R2/3 only");
    if (highestRungSupportedByEvm(5.0f, kEvmDemoteHoldMarginDb) != kRungIdxQpskR14)
        FAIL("5 dB usable EVM must floor at QPSK R1/4");
    PASS();
    return true;
}

// ── GOODPUT-GRADED CRATER (ULTRA_CRATER_GOODPUT_GRADE) ─────────────────────────
// The break-even delivered fraction is the spectral-efficiency RATIO of adjacent
// enabled rungs, derived from constellation/code geometry alone. Verifies the
// derivation (not a memorised table), the floor case, and that a true 0/N crater
// is still evidence at every rung.
static bool test_goodput_break_even_is_rate_ratio() {
    TEST("goodput break-even == spectral-efficiency ratio of adjacent enabled rungs");
    // eta = modulation bits x code rate, computed independently of the helper.
    auto eta = [](Modulation m, CodeRate r) {
        return static_cast<float>(getBitsPerSymbol(m)) * getCodeRateValue(r);
    };
    if (std::fabs(rungSpectralEfficiency(kRungIdxQam16R23) - eta(Modulation::QAM16, CodeRate::R2_3)) > 1e-4f)
        FAIL("rungSpectralEfficiency disagrees with bits x rate");
    // 16QAM R2/3 (eta 8/3) over 16QAM R1/2 (eta 2) = 0.75: a 7/8 group (0.875) is a
    // WIN and must not read as rung failure; 5/8 (0.625) is a genuine loss.
    const float be16 = goodputBreakEvenDeliveredFraction(kRungIdxQam16R23);
    if (std::fabs(be16 - 0.75f) > 1e-3f)
        FAIL("16QAM R2/3 break-even must be 0.75 (= eta(16QAM R1/2)/eta(16QAM R2/3))");
    if (!(0.875f >= be16)) FAIL("7/8 delivery must clear the 16QAM R2/3 break-even");
    if (!(0.625f < be16)) FAIL("5/8 delivery must fall below the 16QAM R2/3 break-even");
    // QPSK R3/4 over R2/3 = (2*2/3)/(2*3/4) = 0.889 — a stricter line than the dense
    // rung, exactly as geometry demands (the step down is smaller, so holding needs
    // a higher delivered fraction to pay).
    const float beq = goodputBreakEvenDeliveredFraction(kRungIdxQpskR34);
    if (std::fabs(beq - (eta(Modulation::QPSK, CodeRate::R2_3) /
                         eta(Modulation::QPSK, CodeRate::R3_4))) > 1e-3f)
        FAIL("QPSK R3/4 break-even must equal its adjacent-rung rate ratio");
    if (!(beq > be16)) FAIL("a smaller rung step must demand a HIGHER delivered fraction");
    // Floor: nothing below QPSK R1/4, so no delivered fraction is demote evidence.
    if (goodputBreakEvenDeliveredFraction(kRungIdxQpskR14) != 0.0f)
        FAIL("the floor rung must have no break-even (nothing below to demote to)");
    // A true crater is evidence at every rung: 0 delivered < every positive break-even.
    for (uint8_t i = kRungIdxQpskR12; i < kRungIdxCount; ++i) {
        const float be = goodputBreakEvenDeliveredFraction(i);
        if (be > 0.0f && !(0.0f < be))
            FAIL("a 0/N crater must remain rung-failure evidence at every rung");
    }
    PASS();
    return true;
}

// ── GROUP-SIZE GATE: dense-rung proxy repair (ULTRA_BURST_ESC_STREAK) ───────────
// The burst airtime ceiling escalates 8600 -> 11500 ms (N=5 -> N=8) only on proven
// clean delivery. Legacy gate: dense rung (>= QAM8 R2/3) + 2 clean rounds. The EVM
// demote breaks that proxy by correctly holding a calm-channel link at QPSK, so the
// knob lets a NON-dense rung earn the same escalation with a longer clean streak.
// Safety property under test: the streak still gates it, so a cratering (Moderate)
// channel — whose streak keeps resetting to 0 — never escalates.
static bool test_group_size_gate_streak() {
    TEST("group-size gate: non-dense rung earns escalation via clean streak");
    auto frames = [](Modulation mod, CodeRate rate, int streak, const char* knob) -> size_t {
        if (knob) setenv("ULTRA_BURST_ESC_STREAK", knob, 1);
        else unsetenv("ULTRA_BURST_ESC_STREAK");
        Connection c;
        TA::makeConnectedOFDM(c, rate, 20.0f, 0.30f, mod);
        TA::setCleanStreak(c, streak);
        // Ask for far more than any ceiling allows so the AIRTIME ceiling — not the
        // request — is what binds (the unit-test frame geometry is finer than the
        // rig's, so a small ask fits under both ceilings and would prove nothing).
        const size_t n = TA::burstFrames(c, 64);
        unsetenv("ULTRA_BURST_ESC_STREAK");
        return n;
    };
    // Frame AIRTIME differs per modulation at a fixed cw count, so frame counts are
    // only comparable WITHIN one modulation. Compare each rung against itself.
    //
    // Legacy path intact: the DENSE rung escalates on 2 clean rounds, not on 1.
    const size_t dense_hi = frames(Modulation::QAM8, CodeRate::R2_3, 2, nullptr);
    const size_t dense_lo = frames(Modulation::QAM8, CodeRate::R2_3, 1, nullptr);
    if (!(dense_hi > dense_lo))
        FAIL("legacy dense-rung escalation broken: streak2=" + std::to_string(dense_hi) +
             " streak1=" + std::to_string(dense_lo));
    // The QPSK (non-dense) baseline: with the knob OFF the streak is irrelevant —
    // byte-identical to legacy, which never escalated a non-dense rung.
    const size_t qpsk_base = frames(Modulation::QPSK, CodeRate::R2_3, 1, nullptr);
    if (frames(Modulation::QPSK, CodeRate::R2_3, 6, nullptr) != qpsk_base)
        FAIL("knob OFF: a QPSK rung must stay at the BASE ceiling at ANY streak");
    // KNOB ON (=2): the same QPSK rung with a proven streak now earns the escalation.
    const size_t qpsk_esc = frames(Modulation::QPSK, CodeRate::R2_3, 2, "2");
    if (!(qpsk_esc > qpsk_base))
        FAIL("knob ON: QPSK rung with streak 2 must escalate (got " +
             std::to_string(qpsk_esc) + ", base=" + std::to_string(qpsk_base) + ")");
    // ...but an UNPROVEN streak still does not — the safety property that keeps a
    // cratering Moderate channel (streak perpetually reset to 0) on short key-downs.
    if (frames(Modulation::QPSK, CodeRate::R2_3, 1, "2") != qpsk_base)
        FAIL("knob ON: streak below the threshold must NOT escalate");
    if (frames(Modulation::QPSK, CodeRate::R2_3, 0, "2") != qpsk_base)
        FAIL("knob ON: a cratering channel (streak 0) must NEVER escalate");
    // Threshold is honored: =4 rejects a streak of 3, accepts 4.
    if (frames(Modulation::QPSK, CodeRate::R2_3, 3, "4") != qpsk_base)
        FAIL("knob=4: streak 3 must not clear the bar");
    if (frames(Modulation::QPSK, CodeRate::R2_3, 4, "4") != qpsk_esc)
        FAIL("knob=4: streak 4 must clear the bar");
    // Out-of-range values fall back to OFF (no accidental escalation from a typo).
    if (frames(Modulation::QPSK, CodeRate::R2_3, 6, "99") != qpsk_base)
        FAIL("out-of-range knob must fall back to default-OFF");
    // The dense rung is UNAFFECTED by the knob (its own 2-streak rule still governs).
    if (frames(Modulation::QAM8, CodeRate::R2_3, 1, "2") != dense_lo)
        FAIL("knob must not relax the DENSE rung's own 2-clean-round requirement");
    PASS();
    return true;
}

static bool test_startup_probe_wire_tail_and_async_safety() {
    TEST("startup probe wire flag; async and FINAL ACKs fail/hold safely");
    using ultra::waveform::tone_burst_ack::ToneBurstAckPayload;
    auto word = [](const ToneBurstAckPayload& ack) {
        return static_cast<uint8_t>((ack.rate_hint & 0x7) |
                                    ((ack.rung_cmd & 0x3) << 3));
    };

    Connection flagged;
    std::vector<ToneBurstAckPayload> flagged_acks;
    flagged.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { flagged_acks.push_back(ack); });
    TA::makeConnectedOFDM(
        flagged, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TA::setStartupWireState(flagged, true, kRungIdxQpskR23);
    TA::emitToneSack(flagged, /*completed_group=*/true);
    if (flagged_acks.empty() ||
        word(flagged_acks.back()) !=
            static_cast<uint8_t>(kRungAuthorityStartupProbeFlag |
                                 kRungIdxQpskR23)) {
        FAIL("trusted group ACK did not carry sender-visible one-shot flag");
    }

    Connection async;
    std::vector<ToneBurstAckPayload> async_acks;
    async.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { async_acks.push_back(ack); });
    TA::makeConnectedOFDM(
        async, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TA::setStartupWireState(async, true, kRungIdxQpskR23);
    TA::emitToneSack(async, /*completed_group=*/false);
    if (async_acks.empty() || word(async_acks.back()) != kRungIdxQpskR12 ||
        TA::startupWaiting(async) || !TA::startupFailed(async)) {
        FAIL("asynchronous SACK emitted an unprotected UP or failed to spend probe");
    }

    Connection final_hold;
    std::vector<ToneBurstAckPayload> final_acks;
    final_hold.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { final_acks.push_back(ack); });
    TA::makeConnectedOFDM(
        final_hold, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TA::setStartupWireState(final_hold, false, kRungIdxQam8R23);
    auto final = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 0, Bytes{0x42}, CodeRate::R1_2);
    final.flags |= v2::Flags::FINAL;
    final_hold.onFrameReceived(final.serialize(), /*physical_turn_complete=*/true);
    if (final_acks.empty() || word(final_acks.back()) != kRungIdxQpskR12 ||
        TA::rxCmd(final_hold) != kRungIdxQpskR12) {
        FAIL("logical FINAL ACK advertised a pointless new rung");
    }

    PASS();
    return true;
}

static bool test_identical_ack_authority_revisions_are_downward_only() {
    TEST("same seq/epoch ACK episode accepts rollback across mask growth and rejects stale UP");
    Connection c;
    const uint8_t up = static_cast<uint8_t>(
        kRungAuthorityStartupProbeFlag | kRungIdxQpskR23);
    if (!TA::acceptAuthorityRevision(c, up, 7, 0x0000, 0))
        FAIL("first flagged UP command was rejected");
    if (!TA::acceptAuthorityRevision(c, kRungIdxQpskR12, 7, 0x0002, 0))
        FAIL("same-episode safety rollback with a newer SACK mask was rejected");
    if (TA::acceptAuthorityRevision(c, up, 7, 0x0000, 0))
        FAIL("delayed old-mask UP reversed an accepted same-episode rollback");
    if (!TA::acceptAuthorityRevision(c, up, 8, 0x001F, 0))
        FAIL("a genuinely new ACK identity did not reset the revision floor");
    PASS();
    return true;
}

int main() {
    // MUST precede any Connection construction (env-latched statics). The suite runs the
    // production latent controller; the legacy ladder it once tested (ULTRA_LATENT_RATE=0)
    // was deleted 2026-11-25.
    setenv("ULTRA_ENABLE_PSK8_LADDER", "1", 1);
    unsetenv("ULTRA_DESCRIPTOR_MODE_SWITCH");  // legacy path -> pending_* observable

    std::cout << "RX-AUTHORITY suite\n";
    bool ok = true;
    ok &= test_busy_defers_up_not_down();
    ok &= test_clean_never_commands_down();
    ok &= test_no_observation_no_command();
    ok &= test_sender_obeys_and_dedups();
    ok &= test_sender_ignores_none_and_garbage();
    ok &= test_operator_lock_overrides_authority();
    ok &= test_evm_floor_table_and_mapping();
    ok &= test_goodput_break_even_is_rate_ratio();
    ok &= test_group_size_gate_streak();
    ok &= test_startup_probe_wire_tail_and_async_safety();
    ok &= test_identical_ack_authority_revisions_are_downward_only();
    std::cout << tests_passed << "/" << tests_run << " passed\n";
    return (ok && tests_passed == tests_run) ? 0 : 1;
}
