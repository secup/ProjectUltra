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
        c.local_data_turn_ = true;
        c.peer_data_turn_requested_ = false;
        c.local_turn_request_pending_ = false;
        c.received_peer_data_since_connect_ = false;
        c.data_turn_yield_pending_ = false;
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
    static constexpr size_t obsRing() { return Connection::kRxAuthObsRing; }
    static constexpr int climbDwell() { return Connection::kRxAuthClimbDwellGroups; }
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

// Receiver verdict: a strong calm channel climbs ONE enabled rung per clean
// verdict (F149: multi-rung crest jumps fueled the climb-crater-demote cycle —
// 11 switches / 5.5 min; each rung must now prove itself with a delivered
// group), and repeated clean verdicts still reach the map's top pick. Descents
// keep taking the map directly (bounded by the 2-rung down-limit).
static bool test_verdict_maps_snr_to_rung() {
    TEST("verdict climbs one rung per clean verdict; 13 dB maps straight down");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);
    c.setBurstChannelObservation(21.0f, 0.30f, 0.8f, true, 0.2f);
    TA::verdict(c, /*all_ok=*/true, /*quality=*/0.9f);
    if (TA::rxCmd(c) != kRungIdxQam8R23)
        FAIL("first clean 21 dB verdict from QPSK R3/4 was idx " +
             std::to_string(TA::rxCmd(c)) +
             " (want ONE enabled rung up = QAM8 R2/3 = " +
             std::to_string(kRungIdxQam8R23) + ")");
    // Each further step must ITSELF clear anchor + 2.5 dB margin (the old code
    // proved "some climb" then jumped to the raw map target — the crest-jump hole).
    // 2026-07-26 ANCHORS RE-MEASURED ON FADING: at 21 dB Good the ladder now TOPS OUT at
    // 8PSK R2/3. 16QAM R2/3's Good anchor moved 20 -> 26 (it measured 51.4% FER at 20 on
    // ITU Good and never beats 8PSK R2/3 in 16-24 dB), and 16QAM R1/2 is now unreachable
    // because it is STRICTLY DOMINATED: identical spectral efficiency (eta = 2.0) but a
    // denser constellation AND non-constant envelope, while 8PSK R2/3 sits at a LOWER
    // anchor (17) and precedes it in the table. At equal eta, prefer the lower order.
    // The property under test is unchanged: the climb must not over-run the ladder top.
    TA::adoptCmd(c);
    TA::verdict(c, true, 0.9f);
    if (TA::rxCmd(c) != kRungIdxQam8R23)
        FAIL("second step was idx " + std::to_string(TA::rxCmd(c)) +
             " (want the ladder top on Good = QAM8 R2/3 = " +
             std::to_string(kRungIdxQam8R23) + ")");
    TA::adoptCmd(c);
    TA::verdict(c, true, 0.9f);
    if (TA::rxCmd(c) != kRungIdxQam8R23)
        FAIL("21 dB over-climbed past the measured ladder top to idx " +
             std::to_string(TA::rxCmd(c)) + " (margin-proof ladder must hold)");

    Connection d;
    TA::makeConnectedOFDM(d, CodeRate::R1_2, 13.0f, 0.05f, Modulation::QPSK);
    d.setBurstChannelObservation(13.0f, 0.30f, 0.8f, true, 0.2f);
    TA::verdict(d, true, 0.9f);
    if (TA::rxCmd(d) != kRungIdxQpskR12)
        FAIL("13 dB Good verdict was idx " + std::to_string(TA::rxCmd(d)) +
             " (want QPSK R1/2)");
    PASS();
    return true;
}

// F149/F160 CLIMB DWELL: after a confirmed crater episode, post-episode reality
// must displace the crest-biased reads (kRxAuthClimbDwellGroups clean groups)
// before ANY up-command — a fade
// crest read right after the demote must not re-arm the loop.
static bool test_confirmed_crater_arms_climb_dwell() {
    TEST("confirmed crater blocks re-climb until the ring turns over");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
    c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
    TA::verdict(c, false, 0.0f);  // crater #1 — hold
    TA::verdict(c, false, 0.0f);  // crater #2 — confirmed: demote + dwell armed
    const uint8_t demoted = TA::rxCmd(c);
    if (demoted >= kRungIdxQam16R23)
        FAIL("confirmed crater did not demote");
    TA::adoptCmd(c);  // sender obeys the demote

    // Fade-crest reads right after the demote: climbs must stay blocked for a
    // full ring turnover even though the map screams UP.
    for (int i = 0; i + 1 < TA::climbDwell(); ++i) {
        c.setBurstChannelObservation(36.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdict(c, true, 0.95f);
        if (TA::rxCmd(c) > demoted)
            FAIL("climb re-armed after only " + std::to_string(i + 1) +
                 " clean groups (dwell must hold " +
                 std::to_string(TA::climbDwell()) + ")");
    }
    // Ring turned over: the next clean verdict may climb (one rung).
    // 36 dB, not 32: the re-climb must clear anchor + crater penalty + margin, and 16QAM
    // R2/3's Good anchor moved 20 -> 26 (2026-07-26 fading re-measure). 26 + 4 (confirmed-
    // crater penalty) + 2.5 (climb margin) = 32.5, so 32 dB is now CORRECTLY refused. The
    // dwell property under test is unchanged; only the SNR that clears the new requirement.
    c.setBurstChannelObservation(36.0f, 0.20f, 0.9f, true, 0.1f);
    TA::verdict(c, true, 0.95f);
    if (TA::rxCmd(c) <= demoted)
        FAIL("climb still blocked after the dwell expired");
    // Assert against the next ENABLED rung, not a raw index delta: the ladder has holes
    // (QAM8 R3/4 auto-disabled, and 16QAM R1/2 disabled 2026-07-26 as dominated), so a
    // single ENABLED step can span several indices. A fixed "+2" silently encoded the old
    // hole layout and broke when the layout changed.
    uint8_t next_enabled = demoted;
    for (uint8_t i = static_cast<uint8_t>(demoted + 1); i < kRungIdxCount; ++i) {
        const CoherentPick p = coherentRungFromIndex(i);
        if (coherentRungLocallyEnabled(p.mod, p.rate)) { next_enabled = i; break; }
    }
    if (TA::rxCmd(c) > next_enabled)
        FAIL("post-dwell climb jumped past the next ENABLED rung (got idx " +
             std::to_string(TA::rxCmd(c)) + ", next enabled above " +
             std::to_string(demoted) + " is " + std::to_string(next_enabled) + ")");
    PASS();
    return true;
}

// ── RX-AUTHORITY PREDICTIVE (docs/RX_AUTHORITY_PREDICTIVE_2026_07_07.md) ──

// EESM self-calibration identity: on a FLAT channel the predictor must
// reproduce the anchor table exactly — pass at the anchor, fail below it.
// test_prediction_flat_identity / test_prediction_rejects_notched_channel — DELETED
// 2026-08-01 with rungPredictedSustainable(). Its only production caller was the
// RX-authority predictive climb (retired), so the predictor had no reader left outside
// these tests. See docs/REMOVAL_BACKLOG.md R12.

static bool test_two_crater_rule() {
    TEST("single crater holds; second consecutive crater clamps below");

    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
    c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);  // map says stay high
    const uint8_t cur = coherentRungIndexFor(Modulation::QAM16, CodeRate::R2_3);

    TA::verdict(c, /*all_ok=*/false, /*quality=*/0.0f);  // crater #1
    if (TA::rxCmd(c) != cur)
        FAIL("single crater moved the command (idx " +
             std::to_string(TA::rxCmd(c)) + ", want hold " + std::to_string(cur) + ")");

    TA::verdict(c, false, 0.0f);  // crater #2 — confirmed
    if (TA::rxCmd(c) >= cur)
        FAIL("confirmed crater verdict idx " + std::to_string(TA::rxCmd(c)) +
             " not below current " + std::to_string(cur));
    // F160: demote lands on the FIRST ENABLED rung below — 16QAM R1/2 (idx 7),
    // the rung the ladder proved on the way up. The old cur-2 stride snapped
    // through the QAM8 R3/4 hole to idx 5 = a 3-rung collapse the one-rung
    // climb repaid across 3 switches.
    if (TA::rxCmd(c) != kRungIdxQam8R23)
        FAIL("confirmed crater demoted to idx " + std::to_string(TA::rxCmd(c)) +
             " (want first enabled below = QAM8 R2/3 = " +
             std::to_string(kRungIdxQam8R23) + "; 16QAM R1/2 is disabled as dominated, and "
             "idx 5 carries the SAME eta = 2.0)");
    // ENABLED-LADDER pin (F145 deadlock): the raw cur-2 stride from QAM16 R2/3
    // (idx 8) lands on QAM8 R3/4 (idx 6) — a fully-disabled anchor row. The
    // command must snap to an ENABLED rung (idx 5, QAM8 R2/3) or the sender's
    // guard refuses it and the cratering rung pins forever (F145: 50 s, six
    // 0/5 whole-burst resends, four refused DOWN commands).
    {
        const CoherentPick p = coherentRungFromIndex(TA::rxCmd(c));
        if (!coherentRungLocallyEnabled(p.mod, p.rate))
            FAIL("confirmed crater commanded a DISABLED rung (idx " +
                 std::to_string(TA::rxCmd(c)) + ") — unobeyable, deadlocks the link");
    }

    // A clean group resets the streak: the next single crater holds again.
    c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
    TA::verdict(c, true, 0.9f);
    TA::verdict(c, false, 0.0f);
    const uint8_t cur2 = coherentRungIndexFor(c.getDataModulation(), c.getDataCodeRate());
    if (TA::rxCmd(c) < cur2 && cur2 != kRungIdxNone)
        FAIL("post-clean single crater demoted (streak not reset)");
    PASS();
    return true;
}

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

// ── ULTRA_RX_EMA_HOLD (throughput-ceiling audit lever #1, 2026-07-21) ──
// A confirmed crater must HOLD the rung while the fade-averaged SNR still clears the
// rung's class floor (deep-null brush the ARQ absorbs), but sustained real failure —
// censored samples driving the average down — must still eventually demote. Same
// scenario, knob ON vs OFF, is the A/B: OFF demotes on the 2nd crater, ON holds.
static bool test_ema_hold_absorbs_supported_crater() {
    TEST("EMA-HOLD holds a confirmed crater the fade-averaged SNR still supports");
    // 16QAM R2/3 on Good reads 24 dB (> the 20 dB Good anchor). Seed clean history
    // so the ring average sits above the anchor, THEN two craters (a fade brush).
    auto run = [](bool knob_on) -> uint8_t {
        setenv("ULTRA_RX_EMA_HOLD", knob_on ? "1" : "0", 1);  // explicit (default is ON)
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.20f, Modulation::QAM16);
        for (int i = 0; i < 4; ++i) {  // clean history keeps the ring average high
            c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
            TA::verdict(c, true, 0.95f);
        }
        c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdict(c, false, 0.0f);  // crater #1
        c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdict(c, false, 0.0f);  // crater #2 — confirmed
        const uint8_t cmd = TA::rxCmd(c);
        setenv("ULTRA_RX_EMA_HOLD", "0", 1);  // restore the suite baseline (off)
        return cmd;
    };
    const uint8_t off_cmd = run(false);
    const uint8_t on_cmd = run(true);
    if (off_cmd >= kRungIdxQam16R23)
        FAIL("knob OFF: confirmed crater must still demote (regression guard)");
    if (on_cmd != kRungIdxQam16R23)
        FAIL("knob ON: EMA-supported confirmed crater must HOLD 16QAM R2/3, got idx " +
             std::to_string(on_cmd));
    PASS();
    return true;
}

static bool test_ema_hold_still_demotes_sustained_failure() {
    TEST("EMA-HOLD does NOT latch: sustained failure demotes once the average drops");
    setenv("ULTRA_RX_EMA_HOLD", "1", 1);
    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.20f, Modulation::QAM16);
    for (int i = 0; i < 3; ++i) {  // brief clean history
        c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdict(c, true, 0.95f);
    }
    // Sustained craters: each failed group is censored toward the rung floor, so the
    // ring average is dragged down and MUST cross below the anchor within a bounded
    // number of groups (strict '>' breaks the censor==anchor equality latch).
    int demoted_at = -1;
    for (int k = 1; k <= 10; ++k) {
        c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdict(c, false, 0.0f);
        if (TA::rxCmd(c) < kRungIdxQam16R23) { demoted_at = k; break; }
    }
    setenv("ULTRA_RX_EMA_HOLD", "0", 1);  // restore the suite baseline (off)
    if (demoted_at < 0)
        FAIL("sustained crater never demoted — EMA-HOLD latched (bug)");
    if (demoted_at < 2)
        FAIL("demoted on the very first confirmed crater — the hold did not engage");
    PASS();
    return true;
}

// ── ULTRA_DENSE_FAST_DEMOTE (RX-authority over-commit fix, 2026-07-23) ──
// On a dense-mod rung (16QAM), a FULL crater (0/N) demotes on the FIRST crater instead of
// waiting for two — the two-crater grace is for robust rungs. Robust mods and partial
// craters keep the grace. Knob-gated (default-off).
static bool test_dense_fast_demote_full_crater() {
    TEST("dense-mod full crater demotes on crater #1 (fast); robust/partial keep the grace");
    auto run_16qam = [](bool knob_on, bool full_crater) -> uint8_t {
        setenv("ULTRA_DENSE_FAST_DEMOTE", knob_on ? "1" : "0", 1);
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.20f, Modulation::QAM16);
        for (int i = 0; i < 3; ++i) {  // clean history at 16QAM R2/3
            c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
            TA::verdict(c, true, 0.95f);
        }
        c.setBurstChannelObservation(24.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdictFull(c, false, 0.0f, full_crater);  // ONE crater
        const uint8_t cmd = TA::rxCmd(c);
        setenv("ULTRA_DENSE_FAST_DEMOTE", "0", 1);
        return cmd;
    };
    // knob ON, dense mod, FULL crater → single crater demotes below 16QAM R2/3.
    if (run_16qam(true, true) >= kRungIdxQam16R23)
        FAIL("knob ON: a full 16QAM crater must demote on crater #1");
    // knob ON but PARTIAL crater → keeps the two-crater grace (no demote on #1).
    if (run_16qam(true, false) != kRungIdxQam16R23)
        FAIL("knob ON but PARTIAL crater must keep the grace (hold on #1)");
    // knob OFF → legacy two-crater rule (no demote on #1 even for a full crater).
    if (run_16qam(false, true) != kRungIdxQam16R23)
        FAIL("knob OFF: single crater must NOT demote (legacy two-crater rule)");
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

// End-to-end: the broadband ladder holds 16QAM R2/3 on a clean, high-SNR group, but
// the honest usable EVM says the rung is over-committed. With the knob ON the command
// is clamped DOWN to the EVM-supported rung; with the knob OFF the clamp is inert
// (byte-identical to legacy). This is the IONOS over-commit the +8.70 offset caused.
static bool test_evm_demote_strips_overcommit() {
    TEST("EVM demote clamps an over-committed 16QAM R2/3 down (knob-gated)");
    auto run = [](bool knob_on, float evm_db) -> uint8_t {
        if (knob_on) setenv("ULTRA_EVM_DEMOTE", "1", 1);
        else unsetenv("ULTRA_EVM_DEMOTE");
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
        // Broadband meter says stay high (survivor-crest read); EVM tells the truth.
        c.setBurstChannelObservation(24.0f, 0.20f, 0.9f, true, 0.1f);
        c.setBurstEvmObservation(evm_db);
        TA::verdict(c, /*all_ok=*/true, /*quality=*/0.9f);
        unsetenv("ULTRA_EVM_DEMOTE");
        return TA::rxCmd(c);
    };
    // Knob ON, usable EVM 13 dB: 16QAM R2/3 floor 16.7 is not cleared → demote to the
    // highest supported rung (8PSK R2/3, floor 13.7 -1 margin). Measurement-backed, so
    // it lands directly (not bounded by the broadband 2-rung down-limit).
    if (run(true, 13.0f) != kRungIdxQam8R23)
        FAIL("knob ON, EVM 13 dB: must demote 16QAM R2/3 -> 8PSK R2/3");
    // Knob ON, usable EVM 14 dB: clears 16QAM R1/2 (13.9) but not R2/3 → one rung down.
    // 16QAM R1/2 is disabled as dominated (2026-07-26), so the clamp snaps to the first
    // ENABLED rung at or below the EVM-supported one: 8PSK R2/3, which carries the SAME
    // eta = 2.0, so no rate is given up by the snap.
    if (run(true, 14.0f) != kRungIdxQam8R23)
        FAIL("knob ON, EVM 14 dB: must demote 16QAM R2/3 -> QAM8 R2/3 (16QAM R1/2 disabled)");
    // Knob OFF: the clamp is inert — the clean high-SNR group holds the top rung.
    if (run(false, 13.0f) < kRungIdxQam16R23)
        FAIL("knob OFF: EVM demote must be inert (no clamp below 16QAM R2/3)");
    PASS();
    return true;
}

// The demote is a floor, not a ceiling: a usable EVM that CLEARS the current rung's
// floor never demotes (and EVM can never RAISE the command — it saturates at the demod
// ceiling and is only ever a min-clamp).
static bool test_evm_demote_inert_when_supported() {
    TEST("EVM demote never fires when the rung floor is cleared");
    setenv("ULTRA_EVM_DEMOTE", "1", 1);
    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
    c.setBurstChannelObservation(24.0f, 0.20f, 0.9f, true, 0.1f);
    c.setBurstEvmObservation(20.0f);  // clears 16QAM R2/3 (16.7) comfortably
    TA::verdict(c, /*all_ok=*/true, /*quality=*/0.9f);
    const uint8_t got = TA::rxCmd(c);
    unsetenv("ULTRA_EVM_DEMOTE");
    if (got < kRungIdxQam16R23)
        FAIL("EVM 20 dB clears the rung floor — must NOT demote (idx " +
             std::to_string(got) + ")");
    PASS();
    return true;
}

static bool test_missing_group_evm_clears_prior_observation() {
    TEST("missing group EVM explicitly clears a prior low observation");
    setenv("ULTRA_EVM_DEMOTE", "1", 1);
    Connection c;
    TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
    c.setBurstChannelObservation(24.0f, 0.20f, 0.9f, true, 0.1f);
    c.setBurstEvmObservation(5.0f);  // would force a deep demote if reused
    c.clearBurstEvmObservation();    // this group has incomplete/no-process coverage
    TA::verdict(c, /*all_ok=*/true, /*quality=*/0.9f);
    const uint8_t got = TA::rxCmd(c);
    unsetenv("ULTRA_EVM_DEMOTE");
    if (got < kRungIdxQam16R23)
        FAIL("invalid current-group EVM must not reuse the prior low scalar (idx " +
             std::to_string(got) + ")");
    PASS();
    return true;
}

// CONFIDENCE-GATED EVM DEMOTE (ULTRA_EVM_DEMOTE_CONFIDENT). The per-group EVM estimate
// has measured sd 3.1-4.4 dB while adjacent rung floors are 1.2-3.1 dB apart, so ONE
// sample cannot resolve a rung. Property under test: a single low sample must NOT fire
// the clamp (that is the sawing the raw form produced -- 15 vs 7 rung commands/run),
// while SUSTAINED low evidence still must (the clamp's whole purpose survives).
static bool test_evm_demote_confidence_gate() {
    TEST("confident EVM demote: a MARGINAL sample holds, sustained evidence still demotes");
    const uint8_t cur = coherentRungIndexFor(Modulation::QAM16, CodeRate::R2_3);
    // 16QAM R2/3 floor is 16.7 dB, hold margin 1.0 => the clamp fires below 15.7.
    // A MARGINAL reading is one inside the estimator's own noise band of that line:
    // 13.0 dB is 2.7 dB low, well within the measured 3.1 dB per-group sd, so a single
    // sample cannot distinguish "rung unsupported" from "noise-low sample".
    auto run = [&](const char* confident, const std::vector<float>& samples) {
        setenv("ULTRA_EVM_DEMOTE", "1", 1);
        if (confident) setenv("ULTRA_EVM_DEMOTE_CONFIDENT", confident, 1);
        else unsetenv("ULTRA_EVM_DEMOTE_CONFIDENT");
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
        for (float db : samples) {
            c.setBurstChannelObservation(24.0f, 0.20f, 0.9f, true, 0.1f);
            c.setBurstEvmObservation(db);
            TA::verdict(c, /*all_ok=*/true, /*quality=*/0.9f);
        }
        return TA::rxCmd(c);
    };
    // RAW form (knob off): one marginal sample clamps immediately -- the documented saw.
    if (!(run(nullptr, {13.0f}) < cur))
        FAIL("raw EVM demote should clamp on a single marginal sample (baseline behaviour)");
    // CONFIDENT form: one marginal sample carries no spread estimate; the optimistic
    // bound (13.0 + 1.645*3.1 = 18.1) still clears 15.7, so it must HOLD.
    if (run("1", {13.0f}) < cur)
        FAIL("confident gate must NOT demote on a single marginal sample");
    // SUSTAINED marginal evidence with realistic spread: mean ~13, se shrinks with n, the
    // optimistic bound falls under the floor, so the clamp must still fire.
    if (!(run("1", {11.5f, 13.0f, 14.5f, 12.0f, 14.0f, 13.0f}) < cur))
        FAIL("confident gate must still demote once sustained evidence is unambiguous");
    // An UNAMBIGUOUS single sample (8.0 dB = 8.7 below the floor, far outside the noise
    // band) SHOULD still fire even at n=1 -- the gate must not be a blanket delay.
    if (!(run("1", {8.0f}) < cur))
        FAIL("confident gate must fire at n=1 when the sample is unambiguous");
    unsetenv("ULTRA_EVM_DEMOTE_CONFIDENT");
    unsetenv("ULTRA_EVM_DEMOTE");
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

// DEFAULT-ON PIN (2026-07-26): with the knob UNSET, two consecutive partial groups that
// each clear the goodput break-even must NOT demote — the whole point of the fix. With
// ULTRA_CRATER_GOODPUT_GRADE=0 the legacy binary predicate returns and the same evidence
// demotes. This is what fails if anyone silently flips the default back.
static bool test_crater_grading_is_default_on() {
    TEST("goodput-graded crater is DEFAULT ON; =0 restores the legacy binary demote");
    const uint8_t cur = coherentRungIndexFor(Modulation::QAM16, CodeRate::R2_3);
    // 7/8 delivered = 0.875, comfortably above the 0.75 break-even for 16QAM R2/3.
    auto run = [&](const char* knob) {
        if (knob) setenv("ULTRA_CRATER_GOODPUT_GRADE", knob, 1);
        else unsetenv("ULTRA_CRATER_GOODPUT_GRADE");
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
        c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdictGraded(c, /*all_ok=*/false, /*quality=*/0.0f, /*full_crater=*/false, 0.875f);
        TA::verdictGraded(c, false, 0.0f, false, 0.875f);
        return TA::rxCmd(c);
    };
    // Not lower: since 2026-11-10 16QAM R2/3 is no longer the ladder top (16QAM R3/4 and 32QAM
    // R2/3 sit above it with turbo data frames), so the same evidence may also climb.
    if (run(nullptr) < cur)
        FAIL("DEFAULT (knob unset): two 7/8 groups must NOT demote from idx " +
             std::to_string(cur) + ", got " + std::to_string(run(nullptr)));
    if (run("0") >= cur)
        FAIL("knob=0 must restore the legacy binary demote (got idx " +
             std::to_string(run("0")) + ", want < " + std::to_string(cur) + ")");
    // A TRUE crater still demotes with the default ON — the cliff case is untouched.
    unsetenv("ULTRA_CRATER_GOODPUT_GRADE");
    {
        Connection c;
        TA::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM16);
        c.setBurstChannelObservation(28.0f, 0.20f, 0.9f, true, 0.1f);
        TA::verdictGraded(c, false, 0.0f, /*full_crater=*/true, 0.0f);
        TA::verdictGraded(c, false, 0.0f, true, 0.0f);
        if (TA::rxCmd(c) >= cur)
            FAIL("a true 0/N crater must STILL demote under the default-on grading");
    }
    unsetenv("ULTRA_CRATER_GOODPUT_GRADE");
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
    // MUST precede any Connection construction (env-latched statics).
    setenv("ULTRA_RX_RATE_AUTHORITY", "1", 1);
    // ULTRA_LATENT_RATE went DEFAULT-ON 2026-08-01 and returns EARLY from
    // updateRxAuthorityCommand, bypassing every legacy clamp. This whole suite exercises
    // that legacy ladder — the two-crater rule, climb dwell, EMA hold, rung penalties — so
    // it must select the path it is written for. This is not a workaround: the legacy path
    // is a supported opt-out for 0.5.1 and it needs coverage as long as it ships.
    // The latent controller has its own suite in test_latent_rate_controller.cpp.
    setenv("ULTRA_LATENT_RATE", "0", 1);
    setenv("ULTRA_DENSE_FAST_DEMOTE", "0", 1);  // explicit baseline; the test toggles it
    setenv("ULTRA_ENABLE_PSK8_LADDER", "1", 1);
    unsetenv("ULTRA_DESCRIPTOR_MODE_SWITCH");  // legacy path -> pending_* observable
    unsetenv("ULTRA_RX_RATE_CMD");
    // ULTRA_RX_EMA_HOLD is default-OFF (reverted 2026-07-23) — matches the legacy
    // demote-machinery tests (two-crater rule, climb-dwell), which exercise the path that
    // runs when the hold does NOT engage. Set explicitly so the baseline is unambiguous
    // regardless of the default. The two test_ema_hold_* cases toggle it on and restore "0".
    setenv("ULTRA_RX_EMA_HOLD", "0", 1);

    std::cout << "RX-AUTHORITY suite\n";
    bool ok = true;
    ok &= test_verdict_maps_snr_to_rung();
    ok &= test_two_crater_rule();
    ok &= test_confirmed_crater_arms_climb_dwell();
    ok &= test_ema_hold_absorbs_supported_crater();
    ok &= test_ema_hold_still_demotes_sustained_failure();
    ok &= test_dense_fast_demote_full_crater();
    ok &= test_busy_defers_up_not_down();
    ok &= test_clean_never_commands_down();
    ok &= test_no_observation_no_command();
    ok &= test_sender_obeys_and_dedups();
    ok &= test_sender_ignores_none_and_garbage();
    ok &= test_operator_lock_overrides_authority();
    ok &= test_evm_floor_table_and_mapping();
    ok &= test_evm_demote_strips_overcommit();
    ok &= test_evm_demote_inert_when_supported();
    ok &= test_missing_group_evm_clears_prior_observation();
    ok &= test_evm_demote_confidence_gate();
    ok &= test_goodput_break_even_is_rate_ratio();
    ok &= test_crater_grading_is_default_on();
    ok &= test_group_size_gate_streak();
    ok &= test_startup_probe_wire_tail_and_async_safety();
    ok &= test_identical_ack_authority_revisions_are_downward_only();
    std::cout << tests_passed << "/" << tests_run << " passed\n";
    return (ok && tests_passed == tests_run) ? 0 : 1;
}
