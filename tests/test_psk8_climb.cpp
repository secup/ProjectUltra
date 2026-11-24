/**
 * 8PSK ladder-aware climb regression (F121 finding, 2026-07-05)
 *
 * With ULTRA_ENABLE_PSK8_LADDER=1 the dense climb must walk THROUGH the QAM8
 * rung instead of hopping straight to 16QAM:
 *   QPSK R3/4  --streak-->  QAM8 R2/3  --streak-->  QAM16 R2/3
 * Before this fix QAM8 was entry-only: the hardcoded QPSK->QAM16 hop stranded
 * any run that exited QAM8 on the QPSK<->QAM16 loop for its remainder.
 *
 * NOTE: psk8LadderEnabled() latches its env at first call (process-lifetime
 * static), so this suite lives in its OWN binary and setenv() runs first in
 * main() before any Connection is constructed.
 */

#include "env_compat.hpp"
#include "protocol/connection.hpp"
#include "protocol/frame_v2.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

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

// Same friend-struct backdoor test_connection_adaptive.cpp uses.
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
    static void applyFeedback(Connection& c, float quality) {
        c.applyAdaptiveRateFeedback(quality);
    }
    static bool modeChangePending(const Connection& c) { return c.mode_change_pending_; }
    static Modulation pendingModulation(const Connection& c) { return c.pending_modulation_; }
    static CodeRate pendingCodeRate(const Connection& c) { return c.pending_code_rate_; }
};

}  // namespace protocol
}  // namespace ultra

// Feed clean groups until a mode change asserts (or the budget runs out) and
// return whether one is pending.
static bool feedCleanUntilPending(Connection& c, int budget = 32) {
    for (int i = 0; i < budget; i++) {
        ConnectionAdaptiveTestAccess::applyFeedback(c, 1.0f);
        if (ConnectionAdaptiveTestAccess::modeChangePending(c)) return true;
    }
    return false;
}

// QPSK R3/4 pinned + clean streak must climb to QAM8 R2/3 (not QAM16) when the
// psk8 ladder is enabled.
static bool test_qpsk_climbs_to_qam8() {
    TEST("QPSK R3/4 clean streak climbs to QAM8 R2/3 (psk8 ladder on)");

    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R3_4, 20.0f, 0.05f, Modulation::QPSK);

    if (!feedCleanUntilPending(c))
        FAIL("clean streak at QPSK R3/4 never asserted a climb");
    if (ConnectionAdaptiveTestAccess::pendingModulation(c) != Modulation::QAM8)
        FAIL("climb target was not QAM8 (got mod=" +
             std::to_string(static_cast<int>(
                 ConnectionAdaptiveTestAccess::pendingModulation(c))) + ")");
    if (ConnectionAdaptiveTestAccess::pendingCodeRate(c) != CodeRate::R2_3)
        FAIL("climb target rate was not R2/3");
    PASS();
    return true;
}

// QAM8 R2/3 pinned + clean streak must step up to QAM16 R2/3 (the dense-branch
// upward walk — this exact path was dead before the fix).
static bool test_qam8_climbs_to_qam16() {
    TEST("QAM8 R2/3 clean streak steps up to QAM16 R2/3");

    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM8);

    if (!feedCleanUntilPending(c))
        FAIL("clean streak at QAM8 R2/3 never asserted the upward step");
    if (ConnectionAdaptiveTestAccess::pendingModulation(c) != Modulation::QAM16)
        FAIL("upward step target was not QAM16");
    if (ConnectionAdaptiveTestAccess::pendingCodeRate(c) != CodeRate::R2_3)
        FAIL("upward step rate was not R2/3");
    PASS();
    return true;
}

// A bad group at QAM8 R2/3 must still take the dense EXIT to QPSK R3/4 (the
// escape semantics must survive the new upward walk sharing the branch).
static bool test_qam8_bad_group_still_exits_to_qpsk() {
    TEST("QAM8 R2/3 crater still exits to QPSK R3/4");

    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.05f, Modulation::QAM8);

    ConnectionAdaptiveTestAccess::applyFeedback(c, 0.0f);  // NACK-grade group
    if (!ConnectionAdaptiveTestAccess::modeChangePending(c))
        FAIL("crater at QAM8 R2/3 did not assert an exit");
    if (ConnectionAdaptiveTestAccess::pendingModulation(c) != Modulation::QPSK ||
        ConnectionAdaptiveTestAccess::pendingCodeRate(c) != CodeRate::R3_4)
        FAIL("exit target was not QPSK R3/4");
    PASS();
    return true;
}

int main() {
    // MUST precede any Connection construction: these statics latch on first read.
    setenv("ULTRA_ENABLE_PSK8_LADDER", "1", 1);
    setenv("ULTRA_QAM16_CLIMB", "1", 1);
    unsetenv("ULTRA_DESCRIPTOR_MODE_SWITCH");  // legacy MODE_CHANGE path -> pending_* observable
    unsetenv("ULTRA_QAM16_CALM_FADING");

    std::cout << "8PSK ladder-aware climb suite\n";
    bool ok = true;
    ok &= test_qpsk_climbs_to_qam8();
    ok &= test_qam8_climbs_to_qam16();
    ok &= test_qam8_bad_group_still_exits_to_qpsk();
    std::cout << tests_passed << "/" << tests_run << " passed\n";
    return (ok && tests_passed == tests_run) ? 0 : 1;
}
