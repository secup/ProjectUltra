// Half-duplex data-turn state (protocol/data_turn.hpp), slice 4 of
// docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md: the grouped transitions.
#include "protocol/data_turn.hpp"

#include <iostream>

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

void session_start() {
    DataTurn t;
    t.reset(/*local_holds=*/true);
    CHECK(t.localHolds() && !t.peerRequested() && !t.requestPending() && !t.yieldPending() &&
              !t.yieldedWaitingForPeerData() && !t.peerDataSinceConnect(),
          "the initiator starts holding the turn, nothing asked or pending");
    t.reset(false);
    CHECK(!t.localHolds(), "the responder starts without it");
}

void yield_then_acquire() {
    DataTurn t;
    t.reset(true);
    t.setPeerRequested(true);
    t.setYieldPending(true);
    t.setPeerDataSinceConnect(true);
    t.yieldToPeer();
    CHECK(!t.localHolds() && t.yieldedWaitingForPeerData() && !t.peerRequested() &&
              !t.yieldPending() && !t.peerDataSinceConnect(),
          "yielding hands the turn over and waits for the peer's data");
    t.setRequestPending(true);
    t.acquire();
    CHECK(t.localHolds() && !t.requestPending() && !t.yieldedWaitingForPeerData(),
          "a TURNOVER gives the turn back and clears our request");
}

void initiator_hand_over_is_partial() {
    DataTurn t;
    t.reset(true);
    t.setPeerRequested(true);
    t.initiatorHandOver();
    CHECK(!t.localHolds() && t.yieldedWaitingForPeerData() && t.peerRequested(),
          "the post-handshake hand-over leaves the request flags as they were");
}
}  // namespace

int main() {
    session_start();
    yield_then_acquire();
    initiator_hand_over_is_partial();
    if (failures) {
        std::cout << "DataTurn: " << failures << "/" << checks << " failed\n";
        return 1;
    }
    std::cout << "DataTurn: " << checks << "/" << checks << " passed\n";
    return 0;
}
