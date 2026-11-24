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
    rule4_last_disconnect_retry();
    rule5_mcdpsk_close_steps_down();
    robust_targets_the_peer_decoder();
    if (failures) {
        std::cout << "LinkControlPlan: " << failures << "/" << checks << " failed\n";
        return 1;
    }
    std::cout << "LinkControlPlan: " << checks << "/" << checks << " passed\n";
    return 0;
}
