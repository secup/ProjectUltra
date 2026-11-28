// planControlTx (protocol/link_control_plan.hpp): every link-control rule, its positive case and the
// conditions that must keep it off. Slice 1 of docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md.
#include "protocol/link_control_plan.hpp"

#include <iostream>
#include <string>

using namespace ultra;
using namespace ultra::protocol;

namespace {
int failures = 0;
int checks = 0;
#define CHECK(cond, msg)                              \
    do {                                              \
        ++checks;                                     \
        if (!(cond)) {                                \
            ++failures;                               \
            std::cout << "FAIL: " << msg << "\n";     \
        }                                             \
    } while (0)

LinkFacts ofdmSession() {
    LinkFacts f;
    f.session = WaveformMode::OFDM_CHIRP;
    f.session_mod = Modulation::QPSK;
    f.peer_decodes_robust = true;
    return f;
}
const Modulation kRobustMid = connection_policy::ladderRungForId(LadderRungId::ROBUST_MID).modulation;

void rule1_reply_on_arrival_waveform() {
    for (const ControlKind k : {ControlKind::DisconnectAck, ControlKind::Turnover}) {
        ControlEvent e;
        e.kind = k;
        e.answering_mcdpsk = true;
        const auto p = planControlTx(ofdmSession(), e);
        CHECK(p.robust_once && p.robust_mod == kRobustMid, "R1: an MC-DPSK request is answered on MC-DPSK");
        e.answering_mcdpsk = false;
        CHECK(!planControlTx(ofdmSession(), e).robust_once, "R1: an OFDM request is answered on OFDM");
        e.answering_mcdpsk = true;
        LinkFacts no_listener = ofdmSession();
        no_listener.peer_decodes_robust = false;
        CHECK(!planControlTx(no_listener, e).robust_once, "R1: never without the peer's MC-DPSK decoder");
        LinkFacts mc = ofdmSession();
        mc.session = WaveformMode::MC_DPSK;
        CHECK(!planControlTx(mc, e).robust_once, "R1: an MC-DPSK session already replies on MC-DPSK");
    }
}

void rule2_close_during_unconfirmed_upcross() {
    LinkFacts f = ofdmSession();
    f.up_unconfirmed_tx = true;
    f.up_return_mod = Modulation::DQPSK;
    ControlEvent e;
    e.kind = ControlKind::DisconnectStart;
    CHECK(planControlTx(f, e).revert_up_crossing, "R2: a close during an unconfirmed up-crossing reverts it");
    f.up_unconfirmed_tx = false;
    CHECK(!planControlTx(f, e).revert_up_crossing, "R2: a confirmed OFDM session closes on OFDM");
}

void rule3_turn_request_into_silence() {
    ControlEvent e;
    e.kind = ControlKind::TurnRequestRetry;
    e.retry_interval_ms = 16000;
    e.peer_silent_ms = 16000;
    CHECK(planControlTx(ofdmSession(), e).robust_once, "R3: a retry into a silent peer rides MC-DPSK");
    e.peer_silent_ms = 3000;
    CHECK(!planControlTx(ofdmSession(), e).robust_once,
          "R3: never while the peer is heard (a healthy peer holding the turn is sending)");
    e.kind = ControlKind::TurnRequest;
    e.peer_silent_ms = 60000;
    CHECK(!planControlTx(ofdmSession(), e).robust_once, "R3: the first request rides the session waveform");
}

// R7 BUG-TURN-REQUEST-OFDM-AT-FLOOR: the first request rides MC-DPSK only when this session's record
// says it reaches the turn sooner there.
void rule7_turn_request_on_faster_waveform() {
    ControlEvent e;
    e.kind = ControlKind::TurnRequest;
    e.retry_interval_ms = 15800;
    e.ofdm_round_trip_ms = 4000;
    e.robust_round_trip_ms = 0;
    e.ofdm_lost = 5.0f;
    CHECK(!planControlTx(ofdmSession(), e).robust_once,
          "R7: no measured MC-DPSK round trip, nothing to compare: OFDM");
    e.robust_round_trip_ms = 17500;
    e.ofdm_answered = 0.0f;
    e.ofdm_lost = 0.0f;
    CHECK(!planControlTx(ofdmSession(), e).robust_once,
          "R7: no losses yet - the optimistic prior keeps OFDM");
    e.ofdm_lost = 2.0f;
    CHECK(planControlTx(ofdmSession(), e).robust_once,
          "R7: OFDM requests lost (Good 8 dB) - the first request rides MC-DPSK");
    e.ofdm_answered = 5.0f;
    e.ofdm_lost = 1.0f;
    CHECK(!planControlTx(ofdmSession(), e).robust_once,
          "R7: OFDM requests mostly answered (Good 15 dB): OFDM");
    e.ofdm_answered = 0.0f;
    e.ofdm_lost = 0.5f;
    CHECK(!planControlTx(ofdmSession(), e).robust_once,
          "R7: decayed losses fade - the session tries OFDM again");
    e.ofdm_lost = 2.0f;
    LinkFacts f = ofdmSession();
    f.peer_decodes_robust = false;
    CHECK(!planControlTx(f, e).robust_once, "R7: never to a peer without an MC-DPSK decoder");
    f = ofdmSession();
    f.session = WaveformMode::MC_DPSK;
    f.session_mod = Modulation::MFSK16;
    CHECK(!planControlTx(f, e).robust_once,
          "R7: an MC-DPSK session already rides MC-DPSK (R9 only steps a rung, none below 16-FSK)");
}

void rule4_last_disconnect_retry() {
    ControlEvent e;
    e.kind = ControlKind::DisconnectRetry;
    e.useful_retry_count = 2;
    e.retry_count = 1;
    CHECK(!planControlTx(ofdmSession(), e).robust_once, "R4: an earlier retry stays on OFDM");
    e.retry_count = 2;
    CHECK(planControlTx(ofdmSession(), e).robust_once, "R4: the last useful retry rides MC-DPSK");
}

void rule5_mcdpsk_close_steps_down() {
    LinkFacts f;
    f.session = WaveformMode::MC_DPSK;
    f.session_mod = Modulation::DQPSK;
    ControlEvent e;
    e.kind = ControlKind::DisconnectRetry;
    e.retry_count = 1;
    CHECK(planControlTx(f, e).step_down_to == Modulation::MFSK16_FAST, "R5: DQPSK steps to 16-FSK-F");
    f.session_mod = Modulation::MFSK16_FAST;
    CHECK(planControlTx(f, e).step_down_to == Modulation::MFSK16, "R5: 16-FSK-F steps to 16-FSK");
    f.session_mod = Modulation::MFSK16;
    CHECK(planControlTx(f, e).step_down_to == Modulation::AUTO, "R5: the bottom rung has nowhere to go");
}

void rule8_mcdpsk_turn_request_retry_steps_down() {
    LinkFacts f;
    f.session = WaveformMode::MC_DPSK;
    f.session_mod = Modulation::DQPSK;
    ControlEvent e;
    e.kind = ControlKind::TurnRequestRetry;
    e.retry_interval_ms = 18000;
    e.peer_silent_ms = 17000;
    CHECK(planControlTx(f, e).step_down_to == Modulation::AUTO,
          "R8: a peer heard within the retry interval keeps the rung");
    e.peer_silent_ms = 18000;
    CHECK(planControlTx(f, e).step_down_to == Modulation::MFSK16_FAST, "R8: DQPSK steps to 16-FSK-F");
    CHECK(!planControlTx(f, e).robust_once, "R8: no waveform crossing in an MC-DPSK session");
    f.session_mod = Modulation::MFSK16;
    CHECK(planControlTx(f, e).step_down_to == Modulation::AUTO, "R8: the bottom rung has nowhere to go");
    e.kind = ControlKind::TurnRequest;
    f.session_mod = Modulation::DQPSK;
    CHECK(planControlTx(f, e).step_down_to == Modulation::AUTO, "R8: a first request keeps the rung");
}

void rule9_mcdpsk_turn_request_one_rung_down() {
    LinkFacts f;
    f.session = WaveformMode::MC_DPSK;
    f.session_mod = Modulation::DQPSK;
    ControlEvent e;
    e.kind = ControlKind::TurnRequest;
    e.retry_interval_ms = 18000;
    e.ofdm_round_trip_ms = 7000;
    e.robust_round_trip_ms = 10000;
    e.ofdm_answered = 10.0f;
    CHECK(!planControlTx(f, e).robust_once,
          "R9: the session rung answers (p 0.92 > 18 / (18 + 10 - 7)): it stays");
    e.ofdm_answered = 0.0f;
    e.ofdm_lost = 1.0f;
    const auto p = planControlTx(f, e);
    CHECK(p.robust_once && p.robust_mod == Modulation::MFSK16_FAST,
          "R9: a request lost at DQPSK - the next goes at 16-FSK-F");
    e.robust_round_trip_ms = 0;
    CHECK(!planControlTx(f, e).robust_once, "R9: no measured round trip below, nothing to compare");
    e.robust_round_trip_ms = 10000;
    f.session_mod = Modulation::MFSK16;
    CHECK(!planControlTx(f, e).robust_once, "R9: the bottom rung has nowhere to go");
    f.session_mod = Modulation::DQPSK;
    e.ofdm_answered = 10.0f;
    e.ofdm_lost = 0.5f;
    CHECK(!planControlTx(f, e).robust_once, "R9: requests mostly answered at the session rung");
    e.ofdm_answered = 0.0f;
    e.ofdm_lost = 0.0f;
    CHECK(planControlTx(f, e).robust_once,
          "R9: the prior alone (p 2/3) loses to an 18 s retry against a 3 s longer round trip");
}

void rule6_mcdpsk_reply_at_request_rung() {
    LinkFacts f;
    f.session = WaveformMode::MC_DPSK;
    f.session_mod = Modulation::DQPSK;
    for (const ControlKind k : {ControlKind::DisconnectAck, ControlKind::Turnover}) {
        ControlEvent e;
        e.kind = k;
        e.answering_mcdpsk = true;
        e.answering_mod = Modulation::MFSK16_FAST;
        const auto p = planControlTx(f, e);
        CHECK(p.robust_once && p.robust_mod == Modulation::MFSK16_FAST,
              "R6: a DQPSK station answers a 16-FSK-F request at 16-FSK-F");
        e.answering_mod = Modulation::DQPSK;
        CHECK(!planControlTx(f, e).robust_once, "R6: a request at its own rung needs no change");
        LinkFacts slow = f;
        slow.session_mod = Modulation::MFSK16;
        e.answering_mod = Modulation::DQPSK;
        CHECK(!planControlTx(slow, e).robust_once, "R6: never speed a reply up past its own rung");
        e.answering_mod = Modulation::AUTO;
        CHECK(!planControlTx(f, e).robust_once, "R6: an unknown request rung changes nothing");
    }
}

void robust_targets_the_peer_decoder() {
    LinkFacts f = ofdmSession();
    CHECK(peerMcDpskDecoderModulation(f) == kRobustMid, "a settled session: the listener's ROBUST_MID");
    f.up_unconfirmed_tx = true;
    f.up_return_mod = Modulation::DQPSK;
    CHECK(peerMcDpskDecoderModulation(f) == Modulation::DQPSK,
          "an unconfirmed up-crossing: the peer primary's rung");
}
}  // namespace

int main() {
    rule1_reply_on_arrival_waveform();
    rule2_close_during_unconfirmed_upcross();
    rule3_turn_request_into_silence();
    rule7_turn_request_on_faster_waveform();
    rule4_last_disconnect_retry();
    rule5_mcdpsk_close_steps_down();
    rule8_mcdpsk_turn_request_retry_steps_down();
    rule9_mcdpsk_turn_request_one_rung_down();
    rule6_mcdpsk_reply_at_request_rung();
    robust_targets_the_peer_decoder();
    if (failures) {
        std::cout << "LinkControlPlan: " << failures << "/" << checks << " failed\n";
        return 1;
    }
    std::cout << "LinkControlPlan: " << checks << "/" << checks << " passed\n";
    return 0;
}
