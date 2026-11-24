// Waveform agreement state (protocol/waveform_agreement.hpp), slice 3 of
// docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md: the transitions and the invariants the loose
// fields they replaced could not enforce.
#include "protocol/waveform_agreement.hpp"

#include <iostream>

using namespace ultra;
using namespace ultra::protocol;

namespace {
int failures = 0;
int checks = 0;
#define CHECK(cond, msg)                          \
    do {                                          \
        ++checks;                                 \
        if (!(cond)) {                            \
            ++failures;                           \
            std::cout << "FAIL: " << msg << "\n"; \
        }                                         \
    } while (0)

void probation_sides_are_exclusive() {
    CrossingProbation p;
    CHECK(!p.active() && !p.sender() && !p.receiver(), "starts with no crossing on probation");
    p.beginSender(Modulation::DQPSK);
    CHECK(p.sender() && !p.receiver() && p.returnMod() == Modulation::DQPSK,
          "a sender probation remembers the rung it left");
    p.beginReceiver(Modulation::MFSK16_FAST, 9000);
    CHECK(p.receiver() && !p.sender(), "one side at a time: a receiver probation replaces it");
    CHECK(p.revertMs() == 9000, "the receiver waits its revert window");
}

void ending_clears_what_belongs_to_the_crossing() {
    CrossingProbation p;
    p.beginSender(Modulation::DQPSK);
    p.markLaunchAck(7, 0x00F0);
    CHECK(p.launchAckValid() && p.launchAckSeq() == 7 && p.launchAckMask() == 0x00F0,
          "the launching ACK is recorded");
    p.end();
    CHECK(!p.active() && !p.launchAckValid() && p.revertMs() == 0,
          "ending clears the side, the launch ACK and the countdown");
    CHECK(p.returnMod() == Modulation::DQPSK, "the rung left is kept after the probation ends");
    p.reset();
    CHECK(p.launchAckSeq() == 0 && p.launchAckMask() == 0, "a session reset clears the launch identity");
}

void revert_countdown() {
    CrossingProbation p;
    p.beginReceiver(Modulation::MFSK16_FAST, 1000);
    CHECK(p.consumeRevert(400) && p.revertMs() == 600, "time remains after 400 ms");
    CHECK(!p.consumeRevert(600), "the window is spent at its end");
}

void rx_command_transitions() {
    RxCrossingCommand c;
    c.stampPending(13);
    CHECK(c.pending() && c.word() == 13 && !c.upCommanded(), "a pending command carries its word");
    c.clearPending();
    CHECK(!c.pending() && c.word() == 0, "applied or withdrawn: nothing pending");
    c.setWord(5);
    c.markUpCommanded();
    CHECK(c.upCommanded() && !c.pending() && c.word() == 5,
          "a dual-listen up-command rides the ACKs without being pending");
    c.clearPending();
    CHECK(c.upCommanded(), "clearing the pending command leaves the up-command flag");
    c.reset();
    CHECK(!c.upCommanded() && !c.pending() && c.word() == 0, "a reset clears everything");
}
}  // namespace

int main() {
    probation_sides_are_exclusive();
    ending_clears_what_belongs_to_the_crossing();
    revert_countdown();
    rx_command_transitions();
    if (failures) {
        std::cout << "WaveformAgreement: " << failures << "/" << checks << " failed\n";
        return 1;
    }
    std::cout << "WaveformAgreement: " << checks << "/" << checks << " passed\n";
    return 0;
}
