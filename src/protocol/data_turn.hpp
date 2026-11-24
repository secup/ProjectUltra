#pragma once
// Half-duplex data-turn state (docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md, slice 4).
//
// Who may send data, and what each side has asked for. Until 2026-11-24 these were six loose
// Connection booleans written at ~50 sites; the grouped writes (taking the turn, yielding it, the
// initiator's hand-over, a session reset) are named transitions here, the rest setters. Pure state,
// behaviour-identical to the fields it replaces. The turn timers (request retransmit, holdoff, TX
// guard) stay in Connection: they are countdowns the tick drives.
#include <cstdint>

namespace ultra {
namespace protocol {

class DataTurn {
public:
    // This station holds the data turn (it is the ISS).
    bool localHolds() const { return local_; }
    // Holding the turn: the peer asked for it (TURN_REQUEST, or the flag on an ACK).
    bool peerRequested() const { return peer_requested_; }
    // Not holding it: our TURN_REQUEST is out, unanswered.
    bool requestPending() const { return request_pending_; }
    // We sent TURNOVER and wait for the peer's first data to confirm it took the turn.
    bool yieldedWaitingForPeerData() const { return yielded_waiting_; }
    // A yield is decided but waits for the TX guard to clear.
    bool yieldPending() const { return yield_pending_; }
    // The peer's data has been received since this session (or this turn) began.
    bool peerDataSinceConnect() const { return peer_data_seen_; }

    // Transitions.
    // The peer handed us the turn (TURNOVER received).
    void acquire() {
        local_ = true;
        peer_requested_ = false;
        request_pending_ = false;
        yielded_waiting_ = false;
        yield_pending_ = false;
    }
    // We sent TURNOVER: the peer holds the turn until its data confirms it.
    void yieldToPeer() {
        local_ = false;
        peer_requested_ = false;
        request_pending_ = false;
        yielded_waiting_ = true;
        yield_pending_ = false;
        peer_data_seen_ = false;
    }
    // The initiator's post-handshake hand-over (a partial yield: request/yield flags untouched).
    void initiatorHandOver() {
        local_ = false;
        yielded_waiting_ = true;
        peer_data_seen_ = false;
    }
    // A session boundary: `local_holds` says who starts with the turn.
    void reset(bool local_holds) {
        local_ = local_holds;
        peer_requested_ = false;
        request_pending_ = false;
        peer_data_seen_ = false;
        yielded_waiting_ = false;
        yield_pending_ = false;
    }

    // Single-flag updates (each was one assignment at its site).
    void setLocalHolds(bool v) { local_ = v; }
    void setPeerRequested(bool v) { peer_requested_ = v; }
    void setRequestPending(bool v) { request_pending_ = v; }
    void setYieldedWaiting(bool v) { yielded_waiting_ = v; }
    void setYieldPending(bool v) { yield_pending_ = v; }
    void setPeerDataSinceConnect(bool v) { peer_data_seen_ = v; }

private:
    bool local_ = false;
    bool peer_requested_ = false;
    bool request_pending_ = false;
    bool yielded_waiting_ = false;
    bool yield_pending_ = false;
    bool peer_data_seen_ = false;
};

}  // namespace protocol
}  // namespace ultra
