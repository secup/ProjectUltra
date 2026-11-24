# Link control state machine — design (2026-11-24)

Status: DESIGN, not implemented. Owner of the migration: Claude (Codex not used).

## Why

`Connection` is 15.3 k lines (`connection.cpp` 11.6 k, `connection_handlers.cpp` 1.5 k, `connection.hpp`
2.2 k) with 208 state fields, ~80 of them booleans. The on-air control decisions — who may transmit,
on which waveform, what a reply rides, when a session is over — are spread across ~25 flags and ~20
countdowns that change in dozens of places. The 2026-11-23/24 TNC sweeps found 25+ defects; nearly every
one was two of these flags disagreeing, not a PHY error:

| Defect (KNOWN_BUGS id) | Flags that disagreed |
|---|---|
| REPLY-ON-WRONG-WAVEFORM | session waveform vs the waveform a request arrived on (not tracked) |
| CLOSE-DURING-UNCONFIRMED-UPCROSS | `crossing_probe_active_` vs `disconnect()` (never consulted) |
| TURN-CONTROL-LOST-AT-OFDM-FLOOR | turn retry vs `ms_since_peer_heard_` (not consulted) |
| MESSAGE-ABANDONED-ON-FOLLOW-FLAP | regrid allowance vs who decided the geometry change |
| RESPONDER-NEVER-ASKS-TURN | `handshake_confirmed_` vs `responder_quiet_ms_` vs turn request gate |
| REDIAL-WAITS-OUT-GRACE | `disconnect_pending_` vs CONNECT handling |
| ROBUST-CLOSE targeting | robust-once rung vs the peer's decoder role (not modelled) |

Each was fixed locally and correctly, and each fix added one more flag or one more special case. The
goal is to make the questions those bugs asked answerable in ONE place.

## Scope

IN — on-air control:
- A. Session lifecycle (connect, half-open responder, connected, closing, peer-closed grace).
- B. Turn ownership (half-duplex token: holder, requests, yields, orphan recovery).
- C. Waveform agreement (what each end transmits, what it believes the peer decodes, confirmation).
- D. The control-TX plan: for any control frame, the waveform/rung it rides and when it is retried.

OUT (unchanged, consumed as inputs): rate selection (ladder, EVM, latent anchors), SR-ARQ internals,
burst transport, file transfer, tone-ACK staircase, PHY. They keep their own state; they READ the link
state where they need it (e.g. ARQ timing reads "which waveform", not raw flags).

## Model

Three orthogonal machines plus one pure planning function. "Orthogonal" is the point: today the turn
code reads crossing flags and the close code reads turn flags ad hoc; in the new model each machine
owns its fields and the others see only its enum + a few read-only facts.

### A. Session

```
DISCONNECTED -> PROBING -> CONNECTING -> CONNECTED_UNCONFIRMED -> CONNECTED -> CLOSING -> DISCONNECTED
                                (responder: CONNECT_ACK sent, initiator not yet heard)
CONNECTED/CONNECTED_UNCONFIRMED --peer DISCONNECT--> PEER_CLOSED_GRACE -> DISCONNECTED
any --link lost (peerSilentLinkLostMs)--> DISCONNECTED
```
Absorbs: `state_` (ConnectionState), `handshake_confirmed_`, `disconnect_pending_`,
`disconnect_teardown_active_`, `responder_half_open_ms_`, `responder_quiet_ms_`,
`responder_handshake_wait_ms_`, `disconnect_*` countdowns. `ConnectionState` stays the public enum
(GUI/TNC/tests use it); the two new sub-states map onto CONNECTED / DISCONNECTING publicly.

### B. Turn

```
         TURNOVER rx                    yield decided            TURNOVER tx
IRS  ------------------> ISS  ---------------------> ISS_YIELDING ---------> YIELDED_WAITING
 |  ^                                                                          |
 |  +------------------------------- peer DATA rx -----------------------------+
 +--local data--> IRS_REQUESTING (retry timer) --TURNOVER rx--> ISS
ORPHANED (neither holds: lost TURNOVER) -> resolved by the BUG-ORPHANED-DATA-TURN rule
```
Absorbs: `local_data_turn_`, `peer_data_turn_requested_`, `local_turn_request_pending_`,
`yielded_data_turn_waiting_for_peer_data_`, `data_turn_yield_pending_`,
`received_peer_data_since_connect_`, `turn_request_retransmit_ms_`, `turn_request_holdoff_ms_`,
`data_turn_tx_guard_ms_` (guard stays a timer, owned here), `peer_turn_request_mcdpsk_` (becomes the
arrival waveform recorded on the IRS_REQUESTING->ISS edge's triggering event).

### C. Waveform agreement

Per end, one value:
```
SETTLED(w, rung)                          both ends decode w as primary
UP_UNCONFIRMED(from_rung -> to_rung)      sender crossed up on the receiver's command; peer's PRIMARY
                                          is still MC-DPSK at from_rung until our OFDM is heard
DOWN_COMMANDED(target)                    receiver stamped a crossing command; not yet obeyed
FALLBACK_SPELL(w)                         sender-initiated down-crossing (rounds without progress)
```
plus the per-event fact `arrival_waveform` (MC-DPSK or OFDM) carried on every RX event.
Absorbs: `crossing_probe_active_`, `crossing_probe_rx_side_`, `crossing_return_mod_`,
`crossing_probe_revert_ms_`, `rx_crossing_apply_pending_`, `rx_crossing_word_`, `rx_dual_up_commanded_`,
`follow_fallback_spell_`, `in_waveform_crossing_`, `receiver_commanded_geometry_change_`,
`last_rx_frame_mcdpsk_`, `disconnect_ack_mcdpsk_`. The follow PICK logic (when to command a crossing)
stays in the follow/rate code: it produces commands, this machine records their consequences.

Derived, read-only: `peerPrimary()` = the waveform+rung the peer's primary decoder runs (OFDM, the old
MC-DPSK rung during UP_UNCONFIRMED, ROBUST_MID listener otherwise). Today this knowledge exists only
implicitly; three bugs came from not having it.

### D. Control-TX plan (pure function)

```
ControlTxPlan planControlTx(ControlKind kind, const LinkState& s, const RxEvent* answering)
  -> { waveform, mc_rung, retry_interval_ms, robust }
```
Every control frame (DISCONNECT, DISCONNECT ack, TURN_REQUEST, TURNOVER, MODE_CHANGE, CONNECT retry,
file cancel) asks it. The rules that today live in five places become its table:

| # | Rule | Source bug |
|---|---|---|
| R1 | A reply rides the waveform its request arrived on | REPLY-ON-WRONG-WAVEFORM |
| R2 | During UP_UNCONFIRMED every control frame rides the peer's primary (old MC-DPSK rung) | CLOSE-DURING-UNCONFIRMED-UPCROSS |
| R3 | A retry into a peer silent for one full retry interval rides MC-DPSK (`peerPrimary` rung) | TURN-CONTROL-LOST-AT-OFDM-FLOOR |
| R4 | The last useful DISCONNECT retry of an OFDM session rides MC-DPSK | DISCONNECT-LOST-IN-FADE |
| R5 | The retry interval is computed for the waveform the frame actually rides | DISCONNECT-TIMING-NON-OFDM |
| R6 | A receiver-commanded geometry change never spends a message's regrid allowance | MESSAGE-ABANDONED-ON-FOLLOW-FLAP |

R5 is the one the current code cannot express: retry timing reads the SESSION waveform even when a copy
rides the other one (today avoided by reverting the crossing on close, R2).

## Events

`onFrame(type, arrival_waveform)`, `onToneAck(...)`, `onRoundTimeout`, `onCrossingCommand(word, by)`
(by = receiver | sender-fallback), `onHostConnect/Disconnect/Send`, `tick(ms)`. Each handler: compute
next state for A/B/C, then ask D for any frame to send. No handler sets another machine's fields.

## Invariants (become CTests, one per row; most already exist as regression tests from 2026-11-23/24)

1. A control reply never rides a waveform the peer was not just heard on (R1).
2. No control frame rides OFDM while the peer's primary is MC-DPSK (R2).
3. Exactly one station holds the turn, or the ORPHANED resolution is running.
4. A session in CLOSING always ends (ack or hard timeout) — never waits on a turn or a crossing.
5. A message fails only on delivery evidence (retry budget, link loss), never on protocol decisions.
6. Retry intervals exceed the round trip of the waveform actually used.

## Migration (each slice ships alone; gate = unit suite + 2-seed x 5-condition TNC smoke; full sweep at the end)

1. Introduce `LinkState` + `planControlTx` as a pure header with its own CTest, called nowhere.
   Encode R1-R6; unit-test it exhaustively (small state space).
2. Route every control-frame send through `planControlTx` (behaviour-identical: the current special
   cases become table rows). Delete `answerOnArrivalWaveform`, robust-once call sites, the close revert.
3. Waveform agreement machine (C): replace the crossing/probe/arrival flags. `peerPrimary()` replaces
   `robustControlModulation()`.
4. Turn machine (B).
5. Session machine (A).
6. Remove the chirp hint unless proven (BUG-CHIRP-HINT-UNVERIFIED) and every flag the slices made dead;
   log removals in REMOVAL_BACKLOG.md; update MODEM_INFRASTRUCTURE_MAP.md per slice.

Order rationale: D first because it is pure and testable and it alone removes the category of bug the
sweeps keep finding; C before B/A because waveform disagreement caused most incidents.

## Non-goals / risks

- No wire-format change. Behaviour must be identical after slice 2 (byte-identical TX per scenario
  where the timing allows; the sweep is the oracle).
- Not a rewrite of `Connection`: rate selection, ARQ, burst and file code are untouched.
- Risk: hidden coupling (a rate-path read of `crossing_probe_active_`). Mitigation: slice 3 starts by
  listing every reader (grep) and routes them through C's read-only accessors.
- The 0-5 dB throughput work (BUG-OFDM-FLOOR-STAYS-MCDPSK, follow pick near the floor) comes AFTER
  this: it tunes decisions that slice 3 moves.
