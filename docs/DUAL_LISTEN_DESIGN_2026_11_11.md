# Dual-listen: the receiver hears both waveforms (2026-11-11)

Knob `ULTRA_WAVEFORM_FOLLOW`: dual-listen (`2`) is the default since 2026-11-11; `0` turns follow off, `1` keeps crossing by agreement for A/B. Same setting on both stations.
Builds on waveform follow (docs/WAVEFORM_FOLLOW_DESIGN_2026_11_11.md, commit 1d695ab).

## Why

Waveform follow switches a session between MC-DPSK and OFDM by agreement: the receiver commands, both
ends switch, probation checks that the other side followed. The agreement travels over the link that
is failing, so every switch needs a command that gets through, a timeout when it does not, a probation
and a fallback, each a guess about time. Measured (GUI gate, ITU Good): a two-minute dip cost +7.6 %
over 4 seeds; a 40 s QSB dip left the stations on different waveforms for 78 s (receiver switched after
37.6 s of silence, sender after 113 s). MC-DPSK carries a few percent of OFDM, so each such loss shows.

## Principle

The receiver decodes either waveform at any time. Nothing about the receiver has to be agreed: the
sender picks the waveform of each burst (from the receiver's last ACK, or from its own silence rule),
and whatever arrives is decoded. A wrong choice costs one burst; there is no state to get out of step.

## How

- **Two decoders, one per waveform, for the whole connected session.** `ModemEngine` owns two
  `StreamingDecoder` objects; one runs the session's current waveform (the primary, which every
  existing engine call addresses), the other the other waveform (the listener). Both are fed the same
  audio, each on its own decode thread. A waveform change flips which one is primary (an atomic index):
  neither decoder is rebuilt, so neither loses its sync, CFO, clock-drift or warm state. With the knob
  off there is one decoder and the engine behaves exactly as before.
- **The chirps are identical** (MC-DPSK and the OFDM anchor: 300-2700 Hz, 500 ms, 100 ms gap, dual), so
  each decoder will sometimes lock on the other's burst and fail it. A listener's failures therefore
  never reach the protocol (no frame-lost, no anchored-burst backstop, no burst-outcome-unknown, no
  descriptor callbacks): only positive evidence - a frame or group that decoded and passed its CRC.
- **The receiver follows what arrives.** Before a listener's decoded frame or group is delivered, the
  engine reports the waveform it arrived on; the connection switches its own state to it (data mode,
  ARQ geometry, ACK semantics) and the engine flips the primary. Then the frame is processed.
- **The sender decides.** It obeys the session rung word in the receiver's ACK (1..10 OFDM, 11/12
  MC-DPSK) and keeps its own silence rule (`maybeCrossDownOnCollapse`). A burst after a waveform change
  starts with the full dual chirp (MC-DPSK always does; OFDM is forced to on a switch), so the
  listener can acquire it cold.
- **Shared across the two decoders:** the tone-ACK monitor runs on the primary only (ACKs are the same
  tones for both waveforms); the listen-before-ACK air-end gate and carrier sense take the larger /
  either; the TX echo clear applies to both; per-frame measurements (SNR, fading, physical SNR) are
  answered by the decoder that delivered the frame.

## What it replaces (dual-listen on)

The receiver no longer switches waveform on its own decisions: no receiver probation or revert
window, no receiver silence crossing, no empty-groups crossing - it only commands. The stale-evidence
reset, the one-burst up-crossing gate and the sender's economics (min(T*, 60 s) patience) stay.

## Verification

Unit: two StreamingDecoders fed the same mixed audio (OFDM bursts, then MC-DPSK frames, then OFDM)
deliver every frame with no mode change; a listener's failure callbacks are dropped; the connection
switches on a listener delivery before processing it. GUI: the follow matrix (step-up, dip, QSB,
drop, outages, steady) with dual-listen on vs follow-only vs off, multi-seed.
