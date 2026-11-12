# Waveform follow: one session moves between MC-DPSK and OFDM (2026-11-11)

> **Status (2026-11-11 evening): default OFF, superseded in direction.** Crossing by agreement works
> (step-ups 3x faster; a dropped channel completes instead of failing) but costs on dips: GUI, 4 seeds,
> a two-minute 20 -> 0 -> 20 dB dip +7.6 % mean; a 40 s QSB dip once left the stations on different
> waveforms for 78 s (receiver after 37.6 s of silence, sender after 113 s). The fix is structural:
> dual-listen - the receiver decodes either waveform at any time (they share the dual chirp; the
> training after it tells them apart), so the sender picks the waveform per burst and nothing has to be
> agreed over a failing link. This document describes the agreement design it replaces.

Knob `ULTRA_WAVEFORM_FOLLOW` (same setting on both stations; read once per Connection). Invariant
INV-FOLLOW-001.

## Problem

A session's waveform was chosen once, at connect (`handleConnect` -> `selectLadderRungPaprCorrected`),
and kept until disconnect. OFDM moved between OFDM rates and MC-DPSK between DBPSK and DQPSK, but
nothing crossed between them.

- A session that connected weak stayed in MC-DPSK when the channel came up: GUI gate, AWGN -3 -> 20 dB
  at t = 70 s, 4 KB: 364 s at ~100 bps (DQPSK), where OFDM carries 1.2-2.6 kbps.
- A session that connected strong stayed in OFDM when the channel went down. At 0 dB the OFDM sender
  sat at 16QAM with an unacknowledged MODE_CHANGE (OFDM control frames do not get through either)
  and moved nothing until the channel came back.

## Design

### One ladder, one command word
The tone ACK's 5-bit rung word names one rung of the session ladder, whatever the waveform:
0 = no command, 1..10 = the canonical OFDM rungs (`kRungIdx*`), 11/12 = MC-DPSK DBPSK/DQPSK
(`kSessionRungWordMcDpskBase`). An MC-DPSK session reads an OFDM value as an up-crossing, an OFDM
session reads an MC-DPSK value as a down-crossing. The word is CRC-covered (the authority span).

### Crossing = the MC-DPSK constellation change, one level up
The receiver (it measures the forward channel) puts the target rung in its tone ACK and crosses
itself; the sender crosses when it reads the command. No MODE_CHANGE exchange, no extra airtime.
`Connection::applyWaveformCrossing` does both ends:
- protocol: `applyDataMode` with the target rung (ARQ re-cuts unacked frames under a new move-epoch;
  the file layer re-queues from the first unacked byte), fresh OFDM controllers on the way up
  (`resetAdaptiveDecisionState` + `resetOfdmLadderState`, as at connect);
- modem: the crossing callback (`setWaveformCrossingCallback` -> binding ->
  `ModemEngine::crossConnectedWaveform`) switches the waveform FIRST, then the data mode. A queued
  connected-OFDM reconfiguration must not outlive a switch to MC-DPSK; `StreamingDecoder::setMode`
  now cancels one when it switches to a non-OFDM mode. The first OFDM burst carries a chirp anchor
  and the receiver expects one (`setWaveformMode`), as after connect.
- The receiver sends its ACK from inside the ARQ, so it applies its own crossing on the next tick.

### Going up (MC-DPSK -> OFDM): receiver decision
After each MC-DPSK burst the receiver runs the connect-time pick on the session's pooled readings
(`sessionLadderPick`: `rateSelectionSnrDb` / fading pool, `connectSelectionSnrDb`, PAPR correction,
`selectLadderRungPaprCorrected`, then `recommendDataMode` + `capInitialOFDMRate` for the OFDM rung):
the connect thresholds, with more samples. When it names OFDM on `run` consecutive clean bursts
(`UpCrossingGate`), the ACK commands that rung. The run starts at ONE burst because the two errors
cost very differently: crossing one burst late forgoes (OFDM - MC-DPSK) x one MC-DPSK cycle, ~(1000 -
50) bps x 18 s ~ 2 KB, while a crossing that fails costs its probation at MC-DPSK's rate, ~70-190
bytes; and one burst is no snapshot (three frames, ~15 s, four coherence times of ITU Good). A burst
is clean when the ACK that closes its turn reports no hole; ACKs raised in the middle of a burst
(re-ACKs of duplicates, out-of-order SACKs) are no verdict (BUG-MCDPSK-MID-BURST-ACK-SCORED).

The evidence is the current MC-DPSK spell's. A down-crossing clears the SNR pool and the physical
channel measurement on both ends, and the modem restarts the decoder's physical ring
(BUG-FOLLOW-STALE-EVIDENCE): first GUI runs kept the 20 dB handshake reading in the pool and a
physical cap made of pre-fade frames, and the fading basis (+11 dB at low readings, bounded only by
that cap) lifted a fade read at -5..+1 dB to a 9.5-13 dB selection - an up-crossing into the fade, a
failed probation and a doubled run. Until the spell has a physical measurement the reading stands
unboosted (`boundByPhysicalEvidenceDb`).

Probation until the first OFDM burst:
- receiver: no OFDM burst by `upCrossingRevertMs` (its ACK at the slowest staircase rung + TX lead-in
  + anchor/descriptor/first frame + two chirp-search windows, ~11 s) -> back to MC-DPSK (the sender
  missed the command and resends on MC-DPSK only after its RTO, later than that);
- receiver: first OFDM group decodes something -> probation ends; delivers nothing -> commands
  MC-DPSK in that group's ACK;
- sender: a retransmission timeout before any ACK for OFDM data -> back to MC-DPSK.
A failed probation doubles the run (up to 8x). The run resets only when the crossing HOLDS: two
delivering OFDM groups (`UpCrossingGate::kHeldGroups`; otherwise a crossing that delivers one group and
fails the next would cycle at the first run on a marginal channel).

### Going down (OFDM -> MC-DPSK)
When to leave a link that stopped delivering is an economic question first. MC-DPSK carries a few
percent of OFDM (DBPSK R1/4 x 3 codewords: 3.6 B/s against 43-278 B/s from QPSK R1/4 to 16QAM R2/3),
and coming back costs: the up-crossing command rides the ACK that ends the first MC-DPSK round after
the channel returns, while OFDM would have resumed at its own next resend (tau ~27 s at 8PSK R2/3,
`followReturnCost`). Ski rental gives T* = tau x r_o / r_m - minutes for any link that was delivering
(`followSkiRentalMs`), past what the air and an operator allow. So a link that was delivering waits
min(T*, `kFollowPatienceMs` = 60 s) of silence before any rule below may cross (fading deep enough to
take OFDM out lasts about a second on ITU Good; slow QSB troughs return within tens of seconds; a band
silent for a minute has usually changed), never past the point where the ARQ would abandon a frame;
one that delivered nothing in this OFDM spell crosses on the evidence alone. r_o is measured: bytes
delivered in the spell over the busy time to the last delivery (idle is neither rate nor silence).
Measured before this rule (GUI, crossing after 25-40 s): ITU Good 40 s QSB dips +17..+30 % time,
20 -> 0 -> 20 dB two-minute fade +6..+17 %.
- Receiver hears bursts that deliver nothing (decoded groups or anchored bursts with no group): two
  empty groups in a row and the ladder has nowhere left to go - the second at the OFDM floor (QPSK
  R1/4), or at the same rung or higher than the first (the sender did not step down) -> commands
  MC-DPSK DBPSK in that ACK. While the ladder is still stepping down it keeps its chance. The floor
  delivering even one frame in eight roughly matches MC-DPSK's best, so only empty groups count.
- Sender: two retransmission timeouts in a row in which the peer was not heard
  (`maybeCrossDownOnCollapse`, `kFollowRoundsWithoutProgressToCross`, follow's own count
  `follow_silent_rounds_`; the collapse escapes consume theirs). Any ACK - an empty one is the peer
  answering, the receiver's to judge - and any CRC-valid frame from the peer end the silence
  (2026-11-12: a MODE_CHANGE ACK arrived after a 40 s QSB trough and the sender crossed 0.2 s later
  on the trough's rounds). Polled before the escapes, so it replaces them: their MODE_CHANGE rides
  OFDM control frames, which do not get through a link this dead (fade run: four MODE_CHANGE
  retries, 40 s, at 0 dB).
- Receiver hears nothing while an object is incoming - a file in progress, or a payload partly
  reassembled (the TNC's messages travel as payloads, not files): 1.5 sender rounds of silence
  (`followReceiverSilenceMs`) -> crosses by itself, before the sender's rule, so the sender's first
  MC-DPSK burst finds it listening. An idle peer holding the turn leaves both empty: its silence
  moves nothing.
- A down-crossing doubles the up-crossing run only when it ends an up-crossing that never held (the
  pick was too hopeful for this channel). Losing an OFDM link that held is the channel moving: no
  backoff, so the session comes back up as soon as the readings say so.
The target is DBPSK at the standard geometry (ROBUST_MID: 8 carriers, 1024 samples/symbol, 3
codewords); the in-session constellation change lifts it to DQPSK when the channel allows.

### Scope
Wideband only (MC-DPSK <-> OFDM_CHIRP); OFDM_NARROW keeps its waveform. An operator-pinned QSO
(forced profile, `ULTRA_LOCK_RATE`) never crosses. Follow needs the receiver authority
(`ULTRA_RX_RATE_AUTHORITY`, default on): the commands are its rung bits, and only an authority sender
reads them and ends its probation (an ACK with progress). Each end checks only its own capabilities: the
initiator never learns the responder's (not in CONNECT_ACK), and a peer that cannot follow does not
obey - probation returns the end that commanded.

## Verification

Simulator: `gui_qso_scenario.sh --snr-profile T:DB,...` (signal scaled along a dB curve over a fixed
noise floor, both directions). Results in docs/CHANGELOG.md (2026-11-11 waveform follow entry).
Unit: `WaveformFollow` (own binary at the production defaults; ConnectionAdaptive pins the
authority off) (crossings switch the modem first; knob off inert;
receiver commands up after a clean run and reverts; first OFDM group settles probation; empty floor
commands down; a down-crossing forgets the pre-fade evidence; receiver silence needs an incoming
object), `ConnectionPolicy` (word codec, gate,
revert window), `ChannelCoreModels` (`checkSnrProfile`).
