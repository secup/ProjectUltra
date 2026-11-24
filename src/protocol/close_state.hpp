#pragma once
// Session close state (docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md, slice 5).
//
// The two sides of a close. Until 2026-11-24 these were ten loose Connection fields, reset by hand
// in five places (three full blocks, two answering-side ones). A plain aggregate: the countdowns
// and counters are driven by Connection::tick and the DISCONNECT handlers exactly as before.
#include <cstdint>

#include "ultra/types.hpp"

namespace ultra {
namespace protocol {

struct CloseState {
    // CLOSER side: this station sent DISCONNECT and waits for its ACK.
    Bytes disconnect_frame;          // cached DISCONNECT, resent on each retry
    int retry_count = 0;             // DISCONNECT retries sent so far
    uint32_t retransmit_ms = 0;      // time until the next retry

    // ANSWERING side: the peer closed; this station acknowledged and keeps a grace window in which
    // a repeated DISCONNECT is re-acknowledged (its first ACK may have been lost).
    bool grace = false;              // in the peer-close grace window
    uint32_t grace_ms = 0;           // time left in it
    uint32_t ack_retransmit_ms = 0;  // time until the next proactive ACK repeat
    uint32_t ack_epoch_elapsed_ms = 0;
    int ack_repeat_count = 0;
    Bytes ack_frame;                 // cached DISCONNECT ACK
    bool ack_mcdpsk = false;         // the DISCONNECT being acknowledged arrived on MC-DPSK

    void resetCloser() {
        disconnect_frame.clear();
        retry_count = 0;
        retransmit_ms = 0;
    }
    void resetAnswering() {
        grace = false;
        grace_ms = 0;
        ack_retransmit_ms = 0;
        ack_epoch_elapsed_ms = 0;
        ack_repeat_count = 0;
        ack_frame.clear();
    }
    void reset() {
        resetCloser();
        resetAnswering();
    }
};

}  // namespace protocol
}  // namespace ultra
