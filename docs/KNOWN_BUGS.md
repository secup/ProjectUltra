# Known Bugs

Last updated: 2026-08-04

## Purpose
Track only currently relevant issues that can affect reliability, throughput, or release quality.
Fixed/obsolete historical deep dives belong in `docs/CHANGELOG.md`.

## Active Issues

### BUG-TURNOVER-GRANT-LOST-IN-REQUESTER-ECHO (open, 2026-08-05) — P2, LATENCY (IONOS rig)

**Symptom.** The DATA-turn handover stalls for ~46 s on a healthy link. The requester
re-sends `TURN_REQUEST` up to a dozen times while the grantor answers every one — the grants
simply do not land.

**Measured (IONOS Mac↔Pi5, bidirectional 10 KB file run, 2026-08-05, both ends on `1fdd15c`):**

| Direction | Sent | Received | Loss |
|---|---|---|---|
| Mac→Pi5 `TURN_REQUEST` | 11 | 8 | 27% |
| Pi5→Mac `TURNOVER` | 8 | **1** | **87.5%** |

This is NOT a weak path: Pi5→Mac *file data* completed CRC-clean at 0.91 kbps in the same
session, and Pi5→Mac control frames (MODE_CHANGE, DISCONNECT) decoded fine. Only the grant
is being lost, and only in this exchange.

**Mechanism (well-supported hypothesis, not yet proven by a controlled run).** The grant is
transmitted with ZERO turnaround delay — `handleTurnRequest()`
(`connection_handlers.cpp:1007-1018`) calls `transmitFrame(turnover)` synchronously on
receipt. Meanwhile the requester decodes its OWN `TURN_REQUEST` **2.00–2.14 s** after
transmitting it (measured, 5 consecutive samples: 2.14/2.10/2.00/2.14/2.03 s), i.e. its own
emission is still coming back off the channel. The grant therefore arrives inside the window
where the requester's own transmission occupies the medium and the decoder. The protocol
layer correctly ignores the self-addressed echo (dst=peer), so this is not a framing bug —
it is a half-duplex TIMING bug: **we answer into the other station's own echo tail.**

Note the asymmetry is self-consistent: `TURN_REQUEST` is sent from an idle channel (73%
success) while `TURNOVER` is sent into the tail of the request that triggered it (12.5%).

**Why it self-heals eventually.** `TURNOVER` is fire-and-forget — there is no grant ACK and
no grant retransmit timer. Recovery relies entirely on the requester's `TURN_REQUEST` retry
(~4.3 s apart), so each lost grant costs a full T/R cycle. With 87.5% grant loss that is
~46 s of dead air before the roles swap.

**Fix direction (do NOT patch blind — one rig run only).** Delay the grant by at least the
requester's echo/turnaround window before transmitting, or give `TURNOVER` its own
retransmit/ack discipline instead of leaning on the requester's retry. The guard machinery
already exists (`armDataTurnTxGuard`/`dataTurnControlGuardMs`) but currently gates the
grantor's DATA, not the grant frame itself. Any change here affects EVERY QSO's turnaround,
so it needs its own paired rig measurement (n≥8 per the rig A/B discipline), not a one-shot.

**Not a regression.** Independent of BUG-MESSAGE-LOST-ON-FORCED-DEMOTE; the turn path was
untouched by that fix. Evidence: `/tmp/mac_run2_full.log`, `/tmp/pi5_run2_full.log`.

### SCOPE CORRECTION (same session, 2026-08-05) — DOES NOT REPRODUCE in the clean case

The 87.5% figure above is ONE run, and a follow-up run did not reproduce it. In a clean
message+reply scenario on the same pair, the handover completed in **~7 s after 2 requests**:

```
[45.117] TX TURN_REQUEST      [49.417] TX TURN_REQUEST
[51.921] RX << TURNOVER seq=0        -> reply delivered, rx_messages=1
```

So this is NOT a general handover failure, and the entry as first written overstated it. The
distinguishing condition in the failing run was CONTENTION: both stations held a queued file,
the ladder was thrashing (MODE_CHANGE R1/2<->R2/3, and a `peer_snr=-10.0` decision), and the
channel was busy — versus a 14-byte reply on an otherwise idle link here. Any retest must
reproduce the CONTENDED case (both ends `--half-duplex --auto-send-file`), not the idle one.

Also corrected: two earlier runs where "the reply never arrived" were a TEST-CONFIG artifact,
not evidence for this bug — `--auto-disconnect-after` was short enough that the initiator tore
the link down before the responder could take its turn. Bidirectional messaging is confirmed
WORKING on the rig (message out 4.8 s, reply back by 55 s).

The `Full-anchor wait rejected DATA fallback` streak (300 rejections, streak 294) seen
alongside is the IDLE receiver spinning with nothing to decode after the exchange completed —
not a stalled transfer. Do not read it as a stall signature without checking whether work was
actually outstanding.

### BUG-MESSAGE-LOST-ON-FORCED-DEMOTE — FIXED 2026-08-05 (see Fixed Bugs) — kept here for the two dead ends

**Symptom.** An application message in flight is destroyed when a MANDATORY geometry
escape (receiver-commanded or stuck-frame demote) changes the data geometry. The operator
sees `FAIL #N failed`; the payload is never delivered.

```
[103.868][WARN] Connection: Abandoning active message/binary operation before geometry
                change QPSK R1/2 -> QPSK R1/4
[103.868][INFO] SR-ARQ: Aborted pending TX state
[103.930][INFO] [GUI] FAIL #1 failed
```

**Reproduce.** `tools/gui_qso_scenario.sh --channel good --snr-db 20 --seed 23
--message-only --message ProjectUltra --message-count 3 --message-vary-len
--reply-message "roger" --exit-after 400` (and `--channel poor --snr-db 20 --seed 3
--message Relay --message-count 3`). NOTE: these are NOT deterministic — a code change
shifts the timeline enough that the same seed may not meet the same fade, so absence of
the failure in one run proves nothing.

**Frequency.** 2 of 10 GUI scenarios in the 2026-08-05 matrix, including on **Good@20 —
a clean channel**. Adaptive demotes are routine on HF, so this is not an edge case.

**Why it is deliberate, and why it is still safe.** `applyDataMode()` fails the object
rather than serializing an old-sized fragment at the new capacity, because the fixed-frame
helper would silently TRUNCATE it and the sender would report delivery for bytes never
placed on wire. Explicit failure is strictly better than that. Ordinary adaptive moves are
already held by `hasGeometryBoundDataOperation()`; only mandatory escapes reach this path.

### DO NOT re-attempt these two fixes — both were tried and measured 2026-08-05

1. **Sender-side resume (re-fragment at the new geometry) DUPLICATES MESSAGES.** The record
   is still "active" in two different situations — mid-flight, AND fully on the wire awaiting
   ACK. In the second case the peer has already reassembled and delivered the object, so a
   resend puts the same report on the operator screen twice. MEASURED: matrix rows m3 and m4
   produced `msg_rx > msg_tx`, with message `#2` delivered twice, both copies complete:
   `rx = #1 / #2 / #2 / #3`. A duplicate delivered message is a corrupt message stream —
   worse than the loss this was meant to fix.
2. **Narrowing the resume to "fragments still unsent" does not work either.** That gate stops
   the duplicate but also stops firing in the case that matters, because *submitted-but-
   unacked* IS the ambiguous state and it is the common one. Unacked does not mean unreceived:
   in m4 the ACK was simply still in flight.

**The real constraint.** This is the classic ARQ ambiguity — the sender cannot determine
whether the receiver holds the object. No sender-side heuristic resolves it.

**Correct fix (not yet implemented).** Receiver-side duplicate suppression keyed on logical
object identity, so a resend is idempotent. `Connection` RX reassembly currently tracks only
`rx_reassembly_buffer_ / _active_ / _binary_ / _epoch_` — there is no object identity and no
delivered-set, so this needs an identity on the wire. That also closes the residual
"submitted-but-unacked" ambiguity generally, not just for this escape.

**Interim behaviour: keep the explicit failure.** It loses a message but does not corrupt the
stream. Do not trade it for a duplicate.

### Fix design (2026-08-05, ready to implement — no deployed peers, so the wire is free)

Make the RESEND IDEMPOTENT so the sender no longer has to guess:

1. Stamp each logical message object with a rolling **object ID**.
2. Receiver keeps the last ~8 delivered IDs.
3. On reassembly completion, drop an object whose ID was already delivered.
4. Re-enable the sender-side resume (written and reverted 2026-08-05; the diff shape is in
   that session's transcript). With dedup in place a wrong guess costs nothing, so the
   ambiguity that blocks approach 1 above stops mattering.

**Two design facts already established — do not re-derive:**

- **Do NOT grow `DataFrame::HEADER_SIZE`.** It feeds `FIXED_FRAME_OVERHEAD`, CONNECT frame
  sizing (`protocol_engine.cpp:530,561`) and the RX framer (`:577,581`), so a header byte
  ripples into control frames and payload-capacity math for no benefit.
- **A 1-bit generation toggle is NOT sufficient**, tempting as it looks. Rule "new object
  toggles, resume keeps" handles resumes, but fails when an object is never delivered:
  last delivered gen=0, message B (gen=1) is lost entirely, message C (gen=1->0) arrives and
  is dropped as a duplicate. That is silent message loss — the original bug, reintroduced.

**Therefore:** put a multi-bit ID (a byte is ample) in the **message-object payload prefix**,
which only the message/binary reassembly path parses. Control frames, file transfer, and the
fixed-frame capacity math stay untouched.

**Verification required:** `ctest`; `tools/message_gui_matrix.sh` green with
`msg_tx == msg_rx == msg_delivered` on every row (watch for `msg_rx > msg_tx`, the duplicate
signature); plus a deterministic forced-demote regression, since the matrix's m3/m4 rows do
not reproduce reliably.

**IMPLEMENTED 2026-08-05 exactly as designed above** — `PayloadType::TEXT_MESSAGE_OBJECT`
(`0x04`), a per-session rolling object id, a depth-8 receiver delivered-set, and the
re-enabled sender re-grid bounded at 2 attempts. Details and verification in Fixed Bugs.
The two dead ends above are retained deliberately: they are the reason the fix is
receiver-side, and re-deriving them costs a measured GUI matrix each.


### BUG-GUI-RX-CONSUMER-STALL-OVERRUN (open, 2026-08-04) — P2, APPARATUS/REAL-TIME

**Observed.** During immutable IONOS A/B pair 1 OFF, the Pi SDL capture callback kept
producing across a 3.145-second consumer-service gap coincident with the two-second FIFO
overrun. About 1.1 seconds of old CONNECT_ACK-tail/silence audio was discarded.
The event happened before the first DATA burst and the exact 51,200-byte transfer still
completed, but the predetermined runtime marker correctly voided the arm.

**Attribution.** All 16 DATA groups in that arm were physically matched. A later missed
ACK occurred roughly 100 seconds afterward with no second overrun, so it is unrelated.
This is a consumer scheduling/ownership defect, not evidence that the FIFO should simply
be enlarged. The render loop drains only a bounded number of chunks per frame and can be
stalled by GUI/protocol mutex work or OS scheduling while the audio callback continues.

**Fix direction.** Drain raw capture audio on a dedicated real-time-safe pump into a
bounded SPSC handoff. Add FIFO high-water, longest consumer-stall, and per-event dropped
sample telemetry; the current warning can make an accumulated drop count look like one
callback's loss. Keep the total jitter cushion independent from the device callback
period. Do not paper over this with a larger latency buffer.

### EXPERIMENT-PSDR-ACQUISITION-DIVERSITY (open, 2026-08-04) — DEFAULT-OFF, REDESIGN

`ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR=1` removes the second full DATA chirp after a
robust BURST_HEADER. Its exact-identity ARQ gate is correct, but a missed descriptor then
leaves a light-only DATA group with no independent acquisition opportunity. Pair 9 ON
lost the complete callback for such an engaged repair after a 1/8 partial and paid seven
timeout retransmissions; the otherwise void pair 1 ON repeated the shape after 3/6.

The completed eight-pair MPG@20 result was a wash (+1.468%, `p=0.8289`; log effect
-0.119%) despite 61 engagements mechanically removing 73.2 seconds. One valid OFF arm
also lost a full-anchored callback, so the two enabled events are not a causal failure-rate
estimate. They are sufficient to reject the claim that descriptor-only repair is an
unconditional acquisition-safe optimization.

Keep the knob strict default-OFF. A successor must either preserve a second acquisition
opportunity when the descriptor is lost, or fail closed after severe partial delivery.
Do not promote or rerun this exact design; see `IONOS_MPG20_CAMPAIGN_2026_08_04.md`.

### BUG-QAM16-MMSE-SLICER-BIAS (open, inherited 2026-06-12) — P3, UNVERIFIED ON CURRENT CODE

**Rescued from `fable_analysis/02_LLR_CALIBRATION_THE_MISSED_FIX.md` §4.4 when that folder was
deleted 2026-08-03.** Filed so the finding is not lost; the line references are from June 2026
and `soft_demap.hpp` has changed since, so **verify before acting**.

**Claimed defect.** MMSE equalisation outputs a SHRUNKEN estimate: mean is `beta * x` with
`beta = |H|^2 / (|H|^2 + sigma^2)`. The 16QAM demapper was said to compare that shrunken
magnitude against an UNBIASED fixed slicer grid (`2/sqrt(10)` = 0.6325), with no bias
compensation. On any low-gamma carrier the outer constellation points then read as inner ones,
giving deterministic wrong-sign ring-bit LLRs.

QPSK is immune by construction: its sign-bit LLR is proportional to `eq/nv` and therefore
invariant to the shrinkage. This is a 16QAM-structural asymmetry that fires even with an
ACCURATE channel estimate.

**Proposed fix (unverified).** Scale the slicer grid by `beta`, or divide the equaliser output
by `beta` before slicing.

**Scope.** Vanishes at high SNR, so it would explain fading-carrier fragility, NOT a broad
decline. Relevant only while the 16QAM rungs are enabled — all of them currently ship
DISABLED (measured 51.4% FER on Good at 20 dB for R2/3), so this is not on any live path today.

**Also inherited from the same document, unverified:** a CARRIER_ADAPTIVE_K constant-modulus
bias (the |eq|-EMA instability detector reading 16QAM's three rings as channel instability),
and LLR scale conventions reported as ~4x (8PSK) and ~3.16x (16QAM) overconfident versus the
QPSK-exact convention, which advances the +/-20 clip and erases CSI dynamic range.

### BUG-DISCONNECT-WAVEFORM-SWAP-SIGSEGV (open, 2026-07-29) — P2, ATTRIBUTION UNRESOLVED

**Symptom.** `ultra_gui` segfaults in the RX decode thread during DISCONNECT teardown.
Crash report `~/Library/Logs/DiagnosticReports/ultra_gui-2026-07-28-220448.ips`:

```
EXC_BAD_ACCESS / SIGSEGV   KERN_INVALID_ADDRESS at 0x0
  ultra::HilbertTransform::process              <- null dereference
  ultra::OFDMChirpWaveform::detectDataSync
  ultra::sync::SyncController::detectFullAnchorFallback
  ultra::gui::StreamingDecoder::searchForSync
  ultra::gui::StreamingDecoder::processBuffer
  ultra::gui::ModemEngine::rxDecodeLoop         <- RX decode thread
```

**Mechanism.** Same class as the documented §14.36 race (`streaming_ofdm_decode.cpp`:
*"swapped modulator_/demodulator_/chirp_sync_ while other paths still held references —
SIGSEGV in HilbertTransform::process"*) but a **different trigger**: the disconnect-time mode
switch, not a descriptor switch. The rig log ends:

```
[365.935] Disconnect timeout, forcing disconnect
[365.935] Disconnected from PI5
[365.956] [StreamingEncoder] Mode changed to MC-DPSK     <- waveform swapped
   ... crash 23 s later, decode thread still inside searchForSync()
```

**Scope, stated accurately — do not overstate this:**
- The transfer had **COMPLETED** (1.42 kbps, 26 groups, file received). **No data was lost.**
- It occurred in the `ULTRA_COMMANDED_GEOMETRY=0` arm, so that feature is not implicated.
- 1 occurrence in 8 rig transfers.
- **BUT** in the real GUI a disconnect does not quit the app, so this would crash a live
  station on disconnect. User-facing, which is why it is filed rather than ignored.

### ✅ ROOT CAUSE FOUND 2026-07-29 (by code audit, after the statistics said "unresolved")

**`StreamingDecoder::setMode()` swaps the waveform INLINE while the RX decode thread is using
it.** `src/gui/modem/streaming_decoder.cpp:729-758`:

```cpp
void StreamingDecoder::setMode(protocol::WaveformMode mode, bool connected) {
    std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
    ...
    if (waveform_mode_changed) {
        waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);   // <-- destroys the
    }                                                                //     object in use
```

The disconnect path calls it from the protocol/GUI thread (`modem_mode.cpp:222-231`:
`reset()` → `setMode(MC_DPSK,false)` → `setDataMode(DQPSK,R1_4)`), while:

```
protocol/GUI thread                   RX decode thread (rxDecodeLoop)
setMode(MC_DPSK,false)                searchForSync()
  lock(ring_.buffer_mutex_)             detectFullAnchorFallback(waveform_, ...)
  waveform_ = createMCDPSK(...)           waveform->detectDataSync(...)   <-- NO LOCK HELD
  ^ unique_ptr assign DESTROYS it           HilbertTransform::process()   <-- freed memory
```

`detectFullAnchorFallback` (`sync_controller.cpp:793`) calls through a raw `IWaveform*`
**without holding `buffer_mutex_`**. That mutex guards the ring buffer, not the waveform
pointer — so it provides no protection here at all. Classic use-after-free; the null deref at
`0x0` is the destroyed object's vtable/member.

**⇒ This is PRE-EXISTING.** Both `setMode` and the disconnect path that calls it are
long-standing. Tonight's work is NOT implicated, which supersedes the statistical
"unresolved" below (kept for the record, and as a reminder that 1-in-8 could not have
settled it either way — the code audit did).

**FIX DIRECTION.** The codebase already has the correct pattern: defer the swap to the decode
thread, exactly like the connected-OFDM path does
(`setConnectedOFDMMode` → `pending_connected_ofdm_change_` → `applyPendingConnectedOFDMMode()`
at the top of the next `processBuffer`, `streaming_decoder.cpp:604/1000`). The §14.36 comment
exists because doing it inline caused this same crash before. Do NOT instead take
`buffer_mutex_` across `detectDataSync` — that would serialise the decode thread against audio
ingest and risk stalls. **Also audit `setDataMode` and `reset()` on the same path for the same
inline-swap hazard.**

---

**Original statistical attribution (superseded by the root cause above):**

| arm | runs | crashes |
|---|---|---|
| clean HEAD `808942a`, RIG (both ends clean) | 8 | **0** |
| clean HEAD `808942a`, SIM (`gui_qso` good@20) | 8 | **0** |
| dirty tree, RIG | 8 | **1** |

0/8 vs 1/8 is **one event — below the resolving power of this sample.** The decision rule was
written down before the data was collected, specifically so it could not be rationalised after
the fact. At a ~12% rate, separating the arms needs roughly **24 runs each** (~2 h rig time per
arm). Worth spending only if it recurs.

Note the only prior crash report the code cites (`ultra_gui-2026-05-28-000112.ips`) has been
rotated away, so "only one report on this machine" does NOT prove the signature is new.

**Next diagnostic step (cheap, do this before more rig time):** the disconnect path calls
`setDataMode`/mode-switch from the protocol thread while `rxDecodeLoop` is inside
`searchForSync()`. Audit whether that switch honours the same deferral discipline the descriptor
path uses (`pending_descriptor_*` applied at the top of the next `processBuffer`) — §14.36 exists
precisely because doing it inline swaps the waveform under live references. If the disconnect
path swaps inline, that is the bug and it needs no statistics.

### BUG-GUI-GATE-EARLY-EXIT-FLAKE (open, 2026-07-29) — P2, MEASUREMENT BLOCKER

**Symptom.** `tools/gui_qso_scenario.sh --channel good --snr-db 20 --file-kb 21` intermittently
ends with `RESULT=FAIL REASON=process_exit_before_pass`. Both `ultra_gui` processes stop within
~0.5 s of each other, their log files truncated MID-LINE (unflushed), and **no macOS crash report
is written**. Two observed timings: during handshake (~14.6 s, `ELAPSED_SEC=21`) and mid-transfer
(~70 s). stderr is discarded by the harness (`>/dev/null 2>&1` at
`tools/gui_qso_scenario.sh:497/507`), so the termination cause is currently invisible.

**NOT caused by `ULTRA_ITERATIVE_CHEST`.** Measured on the SAME tree, interleaved, ITU Good @20,
21 KB:

| seed | knob ON | knob OFF (baseline) |
|---|---|---|
| 42 | PASS 2100 bps | PASS 1840 bps |
| 7 | **FAIL** (died ~70 s) | PASS 2110 bps |
| 11 | PASS 1890 bps | **FAIL** (died ~14.6 s) |
| 23 | PASS 1540 bps | PASS 1630 bps |

The baseline fails with the identical signature, so this is a property of the working tree /
harness, not of the feature.

**Impact.** It makes the faithful gate unusable as a *reliability* instrument until fixed: a
one-seed FAIL cannot be attributed. Any A/B run on this gate right now must report per-seed
results and must NOT quote a pass rate.

**First step when picking this up:** re-run with stderr captured (a copy of the harness with the
two `>/dev/null 2>&1` redirections pointed at files) and read the termination message. Do not
theorise first — the harness is hiding the answer, not lacking one.


### BUG-BURST-STALE-GEOMETRY: a missed switch-descriptor makes the receiver slice the whole group with the PREVIOUS rung's geometry — group destroyed (0 CWs on every frame) AND the ack keyed into the sender's own transmission (~28 s dead air)

- **STATUS: ✅ DEFECT (A) FIXED unconditionally; broad defect (B) recovery is
  DEFAULT-OFF since 2026-07-29. A narrow same-rung Z81 recovery is default-active
  for BI0 only, covering the two wire-learned long file profiles as of
  2026-08-04. It covers both a surviving marked head and a later strong classic
  member after the descriptor plus marked head were lost, but never invents `N`:
  only a CRC-valid physical tail closes the turn. BI1 fails closed when its
  descriptor—and therefore `N`—is lost. Fresh IONOS validation of the later-member
  arm is pending.**
  See CHANGELOG 2026-07-28. Two things this entry got WRONG, both corrected below in place:
  1. **It is TWO independent defects, not one.** (A) an ALIGNMENT/TRUNCATION defect produces
     the TIMEOUT and is NOT fixed by the commanded-geometry work; (B) the stale geometry
     destroys the group. (A) is fixed; (B) has a safe narrow BI0 recovery while
     broad reconstruction remains opt-in. §"Mechanism" below only described (B).
  2. **Line refs had drifted** (corrected in the text): the arm site is
     `streaming_ofdm_decode.cpp:1536` not 1533 (1533 is `descriptor_group_size_locked_`), the
     descriptor set-site is **878** not 762, and the mid-burst gate is **1204** not 1013.
  A live interleaved MPG@20 run exposed the remaining structural boundary: without
  the descriptor, BI1 cannot know physical `N` before deinterleaving. The N7 sender /
  stale-N8 receiver case declared air-end 2.382 s early and forced seven RTO
  retransmissions. A pre-deployment review then caught the inverse BI0 hazard:
  stale N7 could stop before a real N8 tail. Descriptorless BI0 now treats configured
  `N` as no boundary, searches to the sender-policy ceiling, and authorizes a callback
  only at a CRC-valid tail. The conservative half-duplex interlock is published in
  the arm transaction itself (not after member 2). `tests/test_burst_stale_geometry.cpp`
  pins both N directions, the tail-corrupt fail-closed case, and the arm-time span.
- **DEFECT (A) — ALIGNMENT/TRUNCATION, the one that causes the TIMEOUT (added 2026-07-28).**
  `checkIfReadyToDecode` (`streaming_sync_acquisition.cpp`) releases the group-start frame once
  `available` clears the requirement IT computed. `available` only grows, so reaching
  `frame_len = std::min(frame_len, available)` in `decodeCurrentFrame` with a short `frame_len`
  means the REQUIREMENT GREW between the two calls — silent truncation. Live causes: the
  deferred `applyPendingConnectedOFDMMode`/`applyPendingDescriptorDataMode` run at the TOP of
  the processBuffer iteration that decodes, i.e. after the SYNC_FOUND iteration that gated on
  the old profile; and `setFixedFrameCodewords` is a plain int written from the GUI thread.
  `burst_next_pos_` AND `burst_data_start_abs_` both derive from `frame_len`, so the whole
  group lands early and `refreshBurstAirEnd()` inherits it. On the capture the head window was
  processed TWICE 9 ms apart (t=137.634/137.643, identical `training_start=21313`), the group
  counted 5 frames while consuming ~4 frame-times of air and finished 1.24 s early.
  **`burst_min_block_` itself was CORRECT (59360 both sides — cw is airtime-normalizing by
  design, `connection_policy.hpp`), so (A) is NOT a geometry error.**
  FIX: never arm a burst group from a truncated frame — defer to SYNC_FOUND and retry.
- **Root-caused 2026-07-28 from a two-station DEBUG capture** (operator-caught
  "full receiver timeout, not acking anything, 2-3 in a row"). Rig MPG@20,
  `/tmp/dbg/{mac,pi5}.log`. Clocks aligned sender+4.75 s = receiver, pinned by
  three independent ACK arrivals at the sender (4.78/4.78/4.71) — **not inferred
  from timing**.
- **Mechanism.** A group's frame geometry arrives in its BURST_HEADER
  descriptor, carried by a control frame behind the full dual-chirp anchor
  (the sender arms one for every DESC-SWITCH, F163). If the receiver misses
  that anchor and takes the connected DATA-sync fallback
  (`sync_controller.cpp:830`), it never decodes the descriptor, and
  `streaming_ofdm_decode.cpp:1536` (was cited as 1533; 1533 is
  `descriptor_group_size_locked_ = false`) arms accumulation from the **latched**
  count:
  `burst_min_block_ = getMinSamplesForCWCount(fixed_frame_codewords_)`.
  Nothing checks that a descriptor for THIS group was ever received.
- **Measured instance.** Receiver (rate authority) commanded 8PSK R2/3 →
  QPSK R3/4 at t=134.25; sender obeyed at 129.87 (`Fixed frame CW count set
  to 8`) and keyed 5 frames / 422080 samples = 8.79 s. Receiver missed the
  anchor (`Down chirp NOT found`, up-chirp WAS at corr=0.671), took the data
  fallback at corr=0.95, sliced 8-CW frames as **12-CW** → `FAILED (0/12 CWs)`
  ×5 with LTS in-band SNR **14.1 dB** (not a fade). Short slices exhausted the
  group **1.9 s before the sender stopped transmitting**; the no-progress ack
  was keyed at sender-clock 137.02 inside key-down [130.1, 138.9].
  **The sender's own audio-activity log has a 27 s hole (129.5→156.5)** —
  direct proof the ack never arrived. Sender ate the full **17.5 s RTO**
  (`zero-progress ARQ round 1 (rto)`), resent, and the SAME frames decoded
  **5/5 q=0.96** once the descriptor was finally read. **28.2 s between useful
  acks; 17.6 s of airtime delivered zero bytes.**
- **Why all three half-duplex guards failed at once** (this is the important
  part — no single check is missing):
  1. **F176 geometry air-gate** — `refreshBurstAirEnd()` computes
     `burst_data_start_abs_ + N*burst_min_block_` from the SAME stale
     `fixed_frame_codewords_`, so it published a short air-end; and it is
     **zeroed at finalize** (`streaming_burst_interleave.cpp:1236`), so a
     *premature* finalize disarms the very gate meant to catch it.
  2. **CCA `channelBusyForTx()`** — read `idle=1` at t=141.57 mid-burst: the
     channel was in a genuine ~7 dB fade trough (rms 0.077 vs thresh 0.152).
     Energy carrier-sense cannot be the interlock here (already documented at
     app.cpp:745).
  3. **`rxSignalActive(1600)`** — anchored on SYNC events, and a burst group is
     sliced from one sync, so it was 4.3 s stale by ack time.
  The deferred-ack path fired **once in the whole 287 s run**, and not here.
- **Frequency / cost.** Missed-descriptor groups vs zero-decode groups across
  4 captured runs: debug 2/2, xfer_1 0/0, xfer_2 0/0, xfer_3 3/2. Both debug
  failures land immediately after a `DESC-SWITCH`. ~35 s of a 287 s transfer
  ≈ **12%**. NOT strictly 1:1 — xfer_3 burst#26 missed its descriptor and still
  decoded 5/5, which fits: a missed descriptor is harmless when the geometry
  did not actually change.
- **FIX DIRECTION — and two traps that make the obvious patches WRONG:**
  - ❌ *"Refuse to arm without a descriptor."* Strictly removes the corrupted
    slice, but **does not fix the stall** (sender still RTOs), and it **kills a
    load-bearing behaviour**: a failed 0/N group deliberately still emits a
    no-progress tone-ack = the fast NACK that
    `BUG-ANCHOR-WAIT-NO-ACK-STALL` was fixed to provide. Burst#21 in the same
    run was also geometry-broken but its ack landed AFTER key-down (receiver was
    running behind) → sender resent immediately, ~9 s not ~28 s. Refusing to arm
    would convert those cheap cases into full RTOs.
  - ❌ *"Publish a maximally-conservative air-end when geometry is unknown."*
    The deferred-ack deadline **DROPS** the ack if air still appears to be
    arriving (F127, app.cpp:2966) — an over-large air-end manufactures dropped
    acks, i.e. worse.
  - ❌ *"Single-chirp anchor fallback"* (the up-chirp WAS detected at corr=0.671
    when the down-chirp match failed) — **WITHDRAWN 2026-07-28, operator-caught:
    this breaks low-SNR sync.** The dual chirp does two irreplaceable jobs.
    (a) *CFO/timing decoupling*: slope = 2400 Hz / 500 ms = 4800 Hz/s →
    **10 samples per Hz** (matches the log's `cfo_to_samples=10.0`). The SUM of
    up/down positions gives timing with CFO cancelled; the DIFFERENCE gives CFO.
    One chirp collapses both into one unknown — ±3 samples at connected warm CFO
    (±0.3 Hz), but ±500 samples at acquisition (±50 Hz). (b) *False-alarm
    rejection — the decisive one.* Measured on this run: accepted dual-verified
    syncs n=108 **mean corr 0.894, min 0.587**; up-found/down-MISSING n=27
    **mean corr 0.848, max 0.965**. The distributions overlap almost entirely —
    an up-chirp at 0.965 had NO valid down-chirp. **Correlation magnitude is not
    a discriminator; the gap test is the whole discriminator.** Any single-chirp
    threshold rejecting the 0.965 false peaks also throws away real 0.587
    anchors. The burst#10 up-chirp at 0.671 sits BELOW the failed-test mean and
    was most likely a fade false peak, not the real anchor.
  - ❌ *"Announce the next group's geometry one group early"* (extend the
    `BURST_FLAG_NEXT_LIGHT_ANCHOR` next-group pattern). Two blockers:
    `ControlFrame::PAYLOAD_SIZE = 6` is fully consumed by the descriptor
    (`payload[0..5]`), and more fundamentally **the sender cannot know the next
    rung** — the switch is TRIGGERED by the receiver's ACK rate_hint
    (`detected group_seq=36` → `Code rate changed` → `RX-AUTHORITY obey`, all
    within 10 ms). There is no lookahead to carry.
  - ✅ **USE THE COMMANDED GEOMETRY (recommended; operator-simplified).** The
    receiver IS the rate authority — it issued `rate_hint=4` at t=134.25 and so
    knew the sender's next geometry BEFORE the sender did. On a missed
    descriptor it should slice with **its own commanded rung**, not the latched
    one. The single ambiguity is whether the sender HEARD the command (the
    command rides in the tone-burst ack; if that ack was lost the sender never
    switched and is resending the OLD geometry — applying the commanded rung
    there would mirror the bug). That is directly observable from arrival
    cadence, and **the discriminator + threshold already exist in the code**
    (`kTimeoutBatchGapSamples = 15 s`, streaming_ofdm_decode.cpp, justified
    in-comment as steady-state ≤ ~11.5 s vs ack-RTO ≥ ~19 s):
      * missed descriptor + steady-state gap → **use the commanded rung**
      * missed descriptor + post-RTO gap → sender never heard it → keep latched
    **Measured separation on this run: steady-state groups 7.00-13.33 s (n=24)
    vs post-RTO resends 19.43/20.01 s (n=2) — disjoint, 15 s sits between.**
    Both failing groups (#10 at 8.53 s, #21 at 10.75 s) were STEADY-STATE, i.e.
    the sender had switched and the commanded geometry was correct — **both
    would have been fixed.** No wire change, no second decode, no new constant,
    and it touches neither the chirp anchor nor the ack gate.
    (Superseded a two-hypothesis decode-both-and-pick-by-LDPC-parity design —
    unnecessary hedging against an ambiguity that is already observable.)
- **✅ IMPLEMENTED 2026-07-28, then scoped 2026-07-29/2026-08-04.** The separate
  defect (A) guard is unconditional. Broad commanded-rung reconstruction is
  default-OFF after its live engagement/null-control findings. The exact
  same-rung 8PSK R2/3 cw4/Z81 and QPSK R3/4 cw3/Z81 restoration is default-active
  only for BI0 and only after that geometry has already been learned from a decoded
  descriptor; it retains the authority, declined, identity, and cadence gates.
  BI0 proves exact physical `N` from the CRC-protected tail, never from stale
  configured count; its search and half-duplex air-end interlock use a conservative
  sender-policy ceiling. BI1 fails closed until an independently decodable outer
  `N`/tail signal exists. The three ❌
  routes each have a
  demonstrated regression or a broken premise and stay rejected. Sites:
  `ofdm_link_adaptation.hpp` (pure geometry query), `ofdm_chirp_waveform.cpp`
  (delegates to it + a latent `initComponents` z-reassert fix),
  `streaming_decode_policy.hpp` (knob + the ONE 15 s cadence constant),
  `streaming_decoder.hpp` (`commanded_rung_idx_`, `learned_rung_geometry_`,
  `last_group_start_abs_`), `streaming_ofdm_decode.cpp` (resolver + truncation guard
  + marker arm + descriptor learn), `streaming_sync_acquisition.cpp` (readiness
  mirror), `streaming_burst_interleave.cpp` (late-join cadence stamp),
  `modem_engine.cpp` (ack snoop). Gate: `tests/test_burst_stale_geometry.cpp`.
  **Rig validation still owed** (interleaved MPG@20 A/B).
- **⚠ THE PREMISE NEEDED TWO MORE GATES — "we commanded X" is NOT "the sender is at
  X" (adversarial review, 2026-07-28).** The ✅ route above states the single
  ambiguity is whether the sender HEARD the command. That is wrong: it may hear it and
  still not obey. `Connection::maybeObeyAuthorityCommand` declines a standing command
  on FIVE paths — `!rateAdaptationActive()` (`ULTRA_LOCK_RATE`); `mode_change_pending_`;
  a locally-disabled rung SNAPPED to a different index; an **UP command deferred to a
  clean send boundary** while its TX window drains, which explicitly does *not* arm
  dedup ("the re-carried command must retry") so it can stand un-adopted for many
  groups while the receiver re-stamps it on every ack; and `tryDescriptorModeSwitch`
  falling back to the legacy MODE_CHANGE round trip. Arming an un-adopted command
  measured **5/5 → 0/5** on an otherwise healthy group in-tree, i.e. the fix
  destroying exactly the case this entry documents as harmless (xfer_3 burst#26).
  Two gates, both derived from the sender's own obey logic, not tuned:
  **DEMOTE-ONLY** (the sender defers only `isFasterMode`; a demote obeys immediately,
  and the measured failure 8PSK R2/3 → QPSK R3/4 is a demote) and **DECLINED** (a
  decoded descriptor announcing a rung ≠ the standing command is direct wire evidence
  of non-adoption). Pinned by trials (C) and (D) of the gate test.
- **Review findings REJECTED, with reasons** (do not re-file):
  * *"`burst_min_block_` from the learned cw can be wrong because the sender's
    coherence walk moves cw within a rung."* Real mechanism, but a wrong cw degrades
    to exactly today's failure (wrong stride, group destroyed) and self-corrects at the
    next decoded descriptor; the reviewer's demonstration of it was actually the
    un-adopted-rung case, now gated. Deriving cw instead of learning it needs the
    sender's fading-index input, which the receiver does not have.
  * *"Publish the commanded rung at RF emission, not at ack encode."* Real (the ack can
    be parked ≤12 s in a single lossy slot). Not worth the plumbing: a dropped ack ⇒
    sender RTO ⇒ the cadence guard fires; an overwritten ack ⇒ the newer one carries the
    same-or-newer command and we store latest-wins; any residue is caught by DECLINED.
  * *"Narrow guard (A) away from `PendingCodewords`."* Rejected — that IS the
    descriptor-declared group start, i.e. where the guard is most needed. The stall
    concern is answered instead by bounding the hold on PROGRESS.
  * *"The `initComponents` z re-assert is not inert at z=81."* Kept. It is a strict
    no-op at the default z=27, and at z=81 the OLD behaviour was incoherent by
    construction (the waveform said 81 while its own demodulator said 648).
- **IMPLEMENTATION NOTES (2026-07-28 — path mapped; the "blocker" DISSOLVED, see
  the ⚠ item).** Do not re-derive these:
  - *Getting the commanded rung to the decoder needs NO protocol-layer change.*
    The tone-burst ack carries it and is built by the StreamingEncoder
    (`ToneBurstAck encoded: ... rate_hint=4`); encoder and decoder are both owned
    by ModemEngine. Full index = `rate_hint | (rung_cmd << 3)`
    (connection.cpp:445-446), → `coherentRungFromIndex()` → mod/rate.
  - *Do NOT try to derive cw/frame from the rung.* It is not rate-derived
    (8PSK R2/3→12, QPSK R3/4→8, QPSK R2/3→8) and the real derivation is
    `applyDataMode`'s `new_cw` parameter deep in the connection layer. Instead
    **learn the table from descriptors already received** — the decoder sees
    `cw/frame`+`lifting_z` per rung on the wire. Verified sufficient for both
    failures: QPSK R3/4→8 CW was learned at t=53.7 (failure at 137.6) and
    QPSK R2/3→8 CW at t=73.8 (failure at 260.2). No entry → fall back to today.
  - *Existing RX-side adoption hook*: `Connection::onDescriptorModeChange(mod,
    rate, cw_per_frame)` (connection.cpp:6011) already runs the RX subset of a
    mode change (window/timers/chunk capacity) — a synthesized geometry should
    call it too, or the receiver keeps the old ARQ window.
  - ⚠ **"OPEN BLOCKER" — DISSOLVED 2026-07-28, it was not real.**
    `getMinSamplesForCWCount` is closed-form in scalars and its ONLY live-object
    read (`getSamplesPerSymbol()`) is INVARIANT across the profile change being
    predicted: `configure()` rewrites only modulation / code_rate / use_pilots /
    pilot_spacing, and `initComponents()` rebuilds the modulator from the
    unchanged fft_size / cp_mode / symbol_guard. So an explicit-params query
    (`ofdm_link_adaptation::minSamplesForCWCountExplicit`) needs no live object,
    no deferred `configure()`, and cannot reach `HilbertTransform::process`.
    **The REAL blocker was different and bigger:** the stale call has THREE
    consumers per group (the readiness mirror in `checkIfReadyToDecode`, the
    slice length `full_frame_samples`/`frame_len` in `decodeCurrentFrame`, and
    `burst_min_block_` at the marker) and the first two run BEFORE the marker
    block — so fixing only `burst_min_block_` (the line this entry cites)
    misaligns the whole group. All three now resolve one tuple. Also: frame 1 is
    necessarily demodulated with the OLD profile, so its soft bits must be pushed
    as a zero-LLR ERASURE of the correct width or the burst deinterleave throws
    and the whole group is dropped. Original text kept below for the record:
  - ⚠ (original) **reconfigure ordering.** The descriptor path cannot apply a
    mod/rate change inline: it sets `pending_descriptor_mod_/rate_` and DEFERS
    `configure()` to the top of the next `processBuffer` because doing it inline
    swapped `modulator_/demodulator_/chirp_sync_` under live references →
    **SIGSEGV in HilbertTransform::process** (ultra_gui-2026-05-28-000112.ips,
    streaming_ofdm_decode.cpp:~838). That deferral is safe for a real descriptor
    (it is its own frame, so the switch lands before the marker frame), but the
    synthesized case IS the marker frame: `burst_min_block_ =
    getMinSamplesForCWCount(fixed_frame_codewords_)` (line 1533) needs the NEW
    profile at arm time, and `getMinSamplesForCWCount(int)` reads the configured
    profile internally — there is no explicit-params overload. Resolve by adding
    a pure geometry query (cw + mod + rate + z → samples) that does not touch the
    live waveform objects, THEN arm. Partially applying the geometry here — new
    cw with the old profile — would reproduce this very bug from the other side.

### BUG-PHYSICAL-SNR-RIG-REF: the new physical in-band SNR readout is wrong on the IONOS bench (read 4.4 dB below effective 6.7 — physically impossible) and stale after handshake
- Status: **OPEN (filed 2026-07-07; display-only — nothing consumes it).** The two-SNR model
  (CHANGELOG 2026-07-07) computes physical = (P_train − N)/N with N = the burst-time
  inter-chirp-gap RMS. On sim it reads 9.0 at a true 10.0 (known −1 dB training-window
  geometry bias). On the rig (WGN@10) it read 4.4 vs effective 6.7 — impossible (physical ≥
  effective always), so the reference is polluted: the IONOS gap noise measured ~4-5 dB hotter
  than the noise present during the training span (S:N-machine tracker dynamics inside the
  100 ms gap, and/or gap-window geometry on the corrected chirp positions). Also STALE: only
  computed on ping-check paths (handshake), then the atomic latches — data-frame logs reprint
  the connect-era value.
- Fix path: (1) move the measurement into `updateTrainingSNREstimate` where the training
  geometry is exact (per-symbol span, settled FIRs); (2) characterize the IONOS gap-noise
  level vs during-signal noise (one bench experiment: long chirp train, compare gap RMS vs
  post-burst decay curve); (3) recompute per decoded frame, not per ping-check.
- The EFFECTIVE (routed) SNR is unaffected and remains the rate-selection input (#74).
- Context: rig effective 6.5 vs wire-truth 9.3 at WGN@10 = real hardware implementation loss
  (~1.2 Hz clock/jitter wander over the 144 ms training window explains it exactly; sim reads
  10.1 dead-on). The operator display should eventually show BOTH numbers.

### BUG-ANCHOR-WAIT-NO-ACK-STALL: marginal-SNR bursts rejected in full-anchor-wait emit NO ACK → sender 44 s RTO stall (THE marginal-SNR throughput lever, quantified ~+30%)

- **Discovered/quantified 2026-07-14 (operator-caught "we didn't even ACK at
  SNR 25"; MPG@25 natural batch F279-F283).** The receiver, while in
  full-anchor-wait, runs a light DATA sync on each incoming burst. At marginal
  SNR the burst-head correlations arrive at **0.15-0.32**, below the
  weak_accept floor (~0.44-0.45, `signal_policy.hpp` evaluateLightSyncCandidate
  :262-278 requires corr ≥ max(weak_floor 0.45, min_confidence-0.08)). So the
  burst is REJECTED (`Full-anchor wait rejected DATA fallback`,
  sync_controller.cpp:817), in streaks of 2-5. A rejected burst never reaches
  group-decode → **NO ACK is emitted** → the sender waits its full RTO
  (**44.68 s** at window-16, burst airtime 21.5 s × 2) before resending as a
  full-anchor burst the receiver finally accepts.
- **Cost (measured):** each stall ≈ 44 s dead air. MPG@25 batch: goodput
  inversely tracks CW-fails/stalls — F283 clean (4 fails) = **2.81 kbps** vs
  rough F281 (1015 fails) = 1.38; mean 2.16. Eliminating the stall lifts the
  mean toward the ~2.8 clean-epoch ceiling ≈ **+30% at this SNR** — bigger than
  any PAPR/QAM lever. Forensics: ~/Documents/ultra_forensics/{F283_mac,nat25_pi5}.log.
- **KEY (the elegant fix): even a FAILED (0/N) decoded group still emits a
  tone-burst ACK with the cumulative base UNCHANGED** ("no progress, resend") —
  which breaks the 44 s silence and gets an immediate resend. So the receiver
  does NOT need to decode the weak burst; it only needs to NOT be silent.
- **Status (2026-07-14): v1 SHIPPED (keepalive ACK, ULTRA_KEEPALIVE_ACK default
  off, fc75b5b+). Mechanism confirmed on rig (fires 1-2×/transfer on the
  stall), safe (ctest clean, cannot cause fails), but the 25 s threshold
  catches the stall late and the A/B was epoch-noise-dominated (inconclusive,
  faint positive in the one clean pair). NEXT = v2: ULTRA_KEEPALIVE_ACK_MS
  ≈ 8-10 s (safe — routes through listen-before-ACK, mid-burst fire is
  deferred; catches the stall ~15 s earlier) + many-pair rig A/B; then flip
  default-on. See CHANGELOG 2026-07-14.**
- **Fix approaches (protocol/sync path, rig-validate; ACK
  contract has regressed before, build carefully):**
  1. ACK-LIVENESS ACCEPT: when a burst is rejected in full-anchor-wait but
     light_found with clear energy (corr above a low floor, e.g. 0.15 — a real
     burst, weak sync), still let it into the decode path so a group-ACK (even
     0/N) is emitted. Risk: false-lock on noise / misaligning the next real
     sync — bound with an energy floor + the correct-base 0/N ACK is harmless.
  2. IMMEDIATE ANCHOR-NACK: on reject-with-energy, send a bare "resend full
     anchor" tone NACK immediately (needs a group-less NACK tone type). Cleaner
     semantically, more protocol surface.
  3. RTO CAP for the anchor-reject case: the receiver knows instantly it can't
     sync; the sender shouldn't wait 44 s. (Blunt; approach 1 is better.)
- Related: [[project_turnaround_rxq_defeats_warmsync]] (warm-sync defeat /
  echo-clear), the ACK-contract family.

### BUG-FILE-CRC-MISMATCH: complete 51200/51200 file assembled with WRONG CONTENT (P0)
- Status: **FIXED 2026-07-05 late evening.** Root-caused by 3-agent forensics (write-map + code audit + adversarial verify) over the preserved run; rig gate LIFTED.
- Mechanism (confirmed, single event in all logs): the LDPC false-positive BIT-FLIP SALVAGE (frame_v2.cpp Case 2). Under status.allSuccess() every CW is a VALID codeword, so the true error is a codeword DIFFERENCE (>= d_min, tens of bits) — a 1-bit "repair" can never be genuine there. The salvage searched ~5k bit positions for a 16-bit CRC syndrome match (~7.8% collision odds vs garbage), hit one at t=38.4 ("FALSE POSITIVE RECOVERED (1-bit flip frame byte 584 bit 3)"), and delivered a corrupted 616-byte chunk (file[616:1232), seq=2) that was ACKed and never resent. Receiver assembly was proven flawless (78 chunks, perfect tiling, all era boundaries exact ACK-edge requeues).
- Fix: ALL bit-flip salvage removed (1-bit, CRC-bit, 2-bit-suspect, Case-1 header search) — completes the 2026-03-15 removal of the 3/4-bit searches that had already caused file corruption (the 1-bit search was wrongly exempted as "exact"). The min-sum re-decode fallback KEPT (it can converge to the TRUE codeword; full frame CRC gates it). Detected false positives now fail the frame; ARQ resends.
- BONUS FIND, also fixed: the vendored CRC32 table had TWO corrupted entries (111: 0xDD0D7A9B->0xDD0D7CC9, 245: 0xCDD706B3->0xCDD70693 — independently re-derived from the polynomial). The file "CRC32" was non-polynomial (linearity broken, burst-detection guarantees void) but self-consistent between stations, so transfers verified — and it still caught this corruption. Wire note: both ends rebuild together.
- Validation: gate run PASS CRC-clean, 0 false-positive events; full ctest green. Prior PASSes confirmed genuinely clean (the gate CRC-checks every run; this was the only false-positive event in any preserved log).
- Evidence preserved: `/tmp/campaign_3000/PRESERVED_crc_mismatch_run/` (gate run good@20 s42, RX-AUTHORITY + anchor-CFO-fix v1 build). `FileTransfer: CRC mismatch (got=CC1983F9 expected=BFD6400B, size=51200/51200)` at t=268.7 — full size, wrong bytes, every fabrication guard silent.
- Run context (suspect factors, unproven): 5 authority mode moves + 8 crater'd groups + requeue-rewinds; heterogeneous chunk sizes across rate changes writing overlapping offsets is the prime suspect class (BUG-FILE-REQUEUE-OFFSET's sibling: content, not resume-point). Frame-CRC-passing constellation corruption is considered implausible (would need many simultaneous CRC collisions). HARQ cross-era combining is second suspect (soft_combine_harq_.clear() coverage on descriptor-committed moves).
- Investigation entry point: reconstruct the file's wrong byte ranges (diff against qam16_50KB.bin), map to chunk offsets/eras, find which transmission wrote them.

### BUG-ANCHOR-CFO-KILL: connected full-chirp re-anchor CFO seeding killed 25% of full-anchor groups at 16QAM (0/N, all frames)
- Status: **FIX v2 IN TREE 2026-07-05 (uncommitted), gate validation in progress.** Root-caused by 3-agent forensics over 4 gate runs (61 FULL groups: 15 fail vs 19 LIGHT: 0 fail; causal A/B sticky-G13 vs climb-G13 same trough; the one LTS-refine firing flipped a failing group to 5/5 iters=1).
- Mechanism: the full-anchor path seeds the whole group's CFO from the chirp gap estimate (sigma 0.3-1.15 Hz phantoms under fading) instead of the warm pilot-tracked value (<0.1 Hz); the drift clamp has a sub-1 Hz blind spot; the LTS residual refine is structurally gated off on fading (cv>=0.20); the poisoned burst_cfo_ rotates every frame -> 16QAM (~10 deg margin) dies group-wide with confident-but-rotated LLRs.
- Fix v1 (warm-keep alone) FAILED its gate run: the noisy chirp was accidentally load-bearing as the tracker's re-center — burst-frame pilot ingest runs BEFORE any LDPC verdict, so crater stretches walk the tracker (measured -0.10 -> +0.29) and v1 removed the only correction. Fix v2 adds outcome-owned certification: a delivered group certifies the warm value (certifyWarm), a 0/N group revokes it (revokeWarm) -> next full anchor re-centers from the chirp. Cold/idle/PING/MC-DPSK/narrow unchanged.
- Residual (separate lever): ~13% of full-anchor groups die to genuine deep nulls under cross-frame interleave (class A) — no CFO fix touches those.

### BUG-TONEACK-FABRICATION: phantom tone-ACK detection fabricated cumulative delivery of 6 undelivered frames — silent 3.7KB file hole + 9.5-min zombie stall (F116)
- Status: **FIXED 2026-07-05 (4-layer, unconditional — data integrity, no knob).** Root-caused from F116 rig forensics + 5-agent adversarial verification (workflow wf_1e79fe9e).
- Failure shape: rig 50KB transfer died with receiver FileTransfer stuck at `expected=34944` while BOTH ARQ ends stayed "consistent" — sender fully ACKed → "payload drained" → 900s-grace auto-disconnect; receiver kept ACKing everything it saw. Bytes 34944..38688 (exactly the crater'd 8PSK group) were never retransmitted by anyone. No error surfaced anywhere.
- Trigger: ToneBurstAckMonitor false positive on STALE AUDIO. The post-detection `consume_until` omitted `tail_base` (window-relative offset used as buffer-relative) → the decoded burst stayed re-scannable; a later cadence tick re-decoded the peer's old 12ms ACK at the 50ms rung (duration aliasing) and fluked Costas+Hamming+CRC-12 (1/4096/candidate) into `group_seq=5/type=NACK/mask=0x7E02` — 60k samples BEHIND the previous detection. The {12,25,50,100}ms multi-rung scan (2026-06-15) is the false-positive surface.
- Amplification: `SelectiveRepeatARQ::onToneBurstAck` nearest-mapped the 6-bit value onto the seq space → base=69, a seq NEVER SENT (nothing consulted `tx_next_seq_`); freshness guard passed (13 ≤ window+1); the cumulative walk retired all 6 in-flight slots firing `on_send_complete(true)` each → FileTransfer ledger popped irreversibly → every later `requeuePendingChunks()` resumed at 38688, past the hole. `tx_base` also advanced PAST `tx_next` (split window) so every real ack afterwards read as stale.
- Fix (all unconditional):
  1. `tone_burst_ack_monitor.cpp` — consume coordinates fixed (`tail_base + offset + needed`) + stream-MONOTONICITY guard (a detection at/behind the previous one is a stale-audio re-decode → dropped, WARN).
  2. `selective_repeat_arq.cpp onToneBurstAck` — SUPPORT-CONSTRAINED decode: an ack can only reference `[tx_base-1, tx_next-1]`; span ≤ window+1 < 64 ⇒ the 6-bit decode is unique inside the support; outside = prior probability zero → DROP as ack-loss (RTO recovers). Never nearest-map.
  3. `connection.cpp onToneBurstAck` — Nack-TYPED detections consumed whole before the ARQ/rate-controller/drive-advisory (nothing on the unified path emits type=Nack; the WAITING-REBASE voice's group_seq is a different sequence space).
  4. `handleAckFrame` — never-sent guard on the shared ack path (covers corrupt control-frame SACKs too) + structural invariant `tx_base != tx_next` in the cumulative walk. New stat `fabricated_acks_dropped`.
- Regression: `tests/test_arq_toneburst_fabrication.cpp` (4/4: exact F116 repro, 64-value property sweep, control-path fabrication, legit-ack preservation).
- Residual: a corrupt control-frame SACK could still phantom-retire WITHIN the sent window (needs an LDPC+frame-CRC fluke — astronomically rarer than the tone path).

### BUG-FALSE-COMPLETION-FAMILY (STRUCTURALLY CLOSED 2026-07-07 — F218 completion gate): chunk-count completion drifted three separate ways; the ARQ is now the ground truth
- Status: **BELT LANDED (F218, third family member).** F218: sender declared
  "Transfer complete (1.43 kbps)" on an ack that retired only thru seq 89 with
  **10 frames in flight** (salvage OFF — a different count leak than F181),
  went idle, receiver stranded at 93 % rcvd / 88 % assembled. Evidence:
  ~/Documents/ultra_forensics/F218_{mac,pi5}.log.
- **Structural fix:** `FileTransferController::maybeCompleteSend()` +
  completion gate injected by the Connection (`arq_.getTxInFlightBytes()==0`).
  The chunk ledger can say done, but completion DEFERS while any frame is in
  flight — state stays SENDING, RTOs keep retransmitting the holes, and the
  gate re-checks on every retiring ack (handleArqTxBaseAdvanced). Every count
  drift in this family is now non-fatal by construction.
- **Open (low prio):** the specific F218 count leak (which path over-counted
  chunks_acked_ / under-counted sent across the 16QAM re-encodes) — forensic
  from the preserved logs; the gate contains it regardless.

### BUG-SACK-DURABILITY-RESIDUAL (DEFUSED 2026-07-07; root-cause narrowed): F181 reproduced the sender-complete/receiver-stranded wedge WITH the F168 deliver-before-discard fix active — a third loss path exists
- Status: **OPEN — observed 2026-07-07 F181 (final batch, MPG@20).** Sender
  "Transfer complete 1.37 kbps" (salvaged ranges [1824,3048) at t=82); receiver
  logged ZERO file-progress marks in 900+ s (contiguous prefix stuck < 25 %)
  and died on the scenario timeout. The two patched discard sites (epoch
  adoption + RX setCodeRate) log salvages — 13 epoch/salvage-class lines
  present — yet the low-offset bytes never reached the receiver's file prefix.
- **Suspects (unverified):** (a) a THIRD RX-slot discard path (full arq reset?
  partial-slot clear?) that drops buffered FILE payloads unlogged; (b) sender
  slots marked `acked` by a STALE-EPOCH SACK (epoch-echo gating hole) so the
  salvage skipped ranges the receiver NEVER confirmed in the new era; (c) the
  salvage delivered but FILE_START/offset bookkeeping rejected the write.
- **Evidence preserved:** ~/Documents/ultra_forensics/F181_{mac,pi5}.log.
- **DEFUSED (same night):** forensics narrowed it — the sender skipped
  [0,456) (the file's FIRST chunk) on a SACK mark for a frame the receiver
  PROVABLY never had (prefix pinned at 0; receiver spent that era ack-silent/
  UNANCHORED, so it could not have SACKed anything → the mark was
  era-corrupted/phantom). The sender-side skip now defaults OFF
  (`ULTRA_SACK_SALVAGE=1` re-enables for measurement); SACKed ranges are
  re-sent and the receiver dedups by offset — the stranded-file class is
  structurally closed. Receiver-side deliver-before-discard stays (pure
  benefit; the rate-change site now logs its salvages for forensic parity).
- **ROOT-CAUSED + FIXED 2026-07-24 — it was NOT a phantom mark.** Forensics over
  F181_{mac,pi5}.log + code settled all three suspects:
  - **(b) phantom/era-aliased SACK mark — REFUTED for F181.** Only 3 epoch bumps
    (0→1→2→3), no mod-4 revisit; the deciding ACK carried epoch 0 == `tx_epoch_` 0
    nine seconds BEFORE the first bump (so it passed the era gate legitimately), and
    it came from the seq-verified bitmap loop, not the cumulative walk (which
    executed zero iterations). The mark was CORRECT.
  - **(a) receiver destroyed legitimately-SACKed bytes — CONFIRMED, this is the
    destroyer.** `file_transfer.cpp` wrote both receive maps with a bare
    `map[offset] = Bytes(...)`. `chunk_size_` changes on every mid-stream rate/mod
    move, so the same byte offset re-arrives SHORTER on the new grid and the
    assignment destroyed the longer copy's tail: a 456 B chunk at offset 0 was
    replaced by era-1's re-gridded 408 B chunk ⇒ `[408,456)` ceased to exist ⇒ 103
    chunks stranded behind a 48-byte hole. **This was LIVE on the default path** (no
    knob) — the salvage only made it permanent by skipping the re-send.
  - **(c) FILE_START/offset bookkeeping — REFUTED as cause; it is the exposure.** The
    missing FILE_START only ROUTED the bytes into the staging map; the contiguous-edge
    rule then stranded everything behind the hole.
  - **FIX:** NEVER-SHRINK insert in both maps (offsets are absolute into one immutable
    `tx_data_`, so the longest entry at an offset is a strict superset — keep it);
    receiver byte coverage is now MONOTONE. Plus the missing slot-identity guard in the
    ARQ cumulative walk. Regressions
    `test_receiver_coverage_never_shrinks_on_regrid_out_of_order` +
    `test_prestart_staging_coverage_never_shrinks_on_regrid` reproduce the stranding
    (fail-before/pass-after proven). See docs/CHANGELOG.md 2026-07-24.
- **`ULTRA_SACK_SALVAGE` stays DEFAULT-OFF — structural, not caution.** A sender-local
  decision to PERMANENTLY never send a byte needs evidence that is byte-domain,
  monotone and era-independent. `slot.acked` is frame-domain, era-relative (2-bit echo)
  and retractable by the receiver's own discard paths; the only monotone byte-domain
  evidence the sender holds locally is the cumulative retirement point, which IS the
  requeue resume offset — so gating the salvage on trustworthy local evidence makes it a
  no-op. Safety requires receiver-authored byte-domain evidence on the wire, and the
  tone-burst payload is SATURATED at 44/44 bits (+1 bit ⇒ 5 Hamming blocks ⇒ 30→38
  symbols ⇒ +27% ACK airtime, lockstep break) for a ~1 s/transfer prize. Not worth it;
  the stranded-file class is closed on the receiver side instead.

### BUG-ALC-NOISE-REF-CONTAMINATED (FIXED 2026-07-25): the RX level meter measured SIGNAL as noise and pinned a false LOW, ramping TX drive into compression
- Status: **FIXED 2026-07-25.** Two independent defects; the meter was wrong, the rig levels were fine.
- **Symptom (rig, MPG@20, xfer_1):** `[ALC-RX] headroom_db` sat at **-4.4 dB** (median) — signal
  apparently BELOW the noise floor — for 18 of 25 groups, producing a permanent `verdict=LOW`.
- **The signal was never the problem.** `data_rms` held rock-steady at 0.073-0.085, `cf_db`
  11.0-11.3 (healthy crest factor, no clipping), `peak` ~0.30. Only the NOISE reference moved:
  it ratcheted `0.0295 -> 0.0826 -> 0.0699 -> 0.1380` and LATCHED at 0.1380.
- **The arithmetic that settles it:** at `data_rms = 0.0824` on a 20 dB S:N channel, true noise
  is `0.0824/10 = 0.0082`. The meter used **0.1380 — +24.5 dB above anything the dial can
  physically produce.** That is not a channel; it is signal. Cross-check on the same transfer:
  LTS usable SNR 10.6-18.5 dB and decision-directed EVM median 9.9 dB, both healthy. The ALC was
  the only meter claiming a fault.
- **Defect 1 — wrong statistic.** `streaming_burst_interleave.cpp` used
  `IdleNoiseSNREstimator::Snapshot::normalized_noise_rms`, which is literally THE MOST RECENT
  window: one contaminated window becomes the reference instantly and persists. The estimator
  already computes `floor_noise_rms` (min over the last `kFloorWindowCount`=15 windows) — the
  correct statistic, because contamination can only ADD power and so can never pull a minimum
  down. Now consumed; the latest-window value is still logged alongside so contamination stays
  visible in a rig trace.
- **Defect 2 — contaminated input.** `streaming_sync_acquisition.cpp:891` fed the raw
  `search_buffer` into `observeIdleNoiseCandidate()` on the branch taken when the sync search
  found NO frame. Mid-transfer that does NOT mean the air is quiet: the buffer can hold this
  station's own tone-burst ACK tail, the multipath echo of the burst just received, or a burst
  the search failed to lock. Now gated on `!rxSignalActiveWithin(12 s)` — decoder EVIDENCE of
  quiet, covering a full burst cycle. (The sibling call site was already guarded in spirit: its
  comment warns "a prefix can contain an undetected earlier transmission's tail, which would
  over-read the floor".)
- **Why it mattered on real hardware:** the false LOW drives the sender's software-ALC to ramp
  `tx_drive` 0.500 -> 0.700 (**+2.9 dB**). On IONOS that is merely useless (an S:N machine — the
  noise tracks the drive, so measured EVM moved 11.04 -> 11.00 dB, i.e. ZERO gain), but on a real
  radio it pushes the PA into ALC compression and actively degrades the link. Latent hardware bug.
- **VERIFIED live on the rig after the fix:** noise reference **0.0302** (stable, was 0.1380
  latched), headroom **+8.4..+9.0 dB** (was -4.4), `data_rms`/`cf_db`/`peak` unchanged —
  confirming the signal was always healthy. Transfer 51200 B CRC-clean, 1.67 kbps, md5 match.
  Full ctest 88/88.
- **Residual (not fixed here):** the verdict still prints LOW because `alcLowHeadroomDb()` sits
  above ~9 dB. That is now an HONEST reading of genuinely modest headroom rather than a
  fabricated one; whether that threshold is correctly placed is a separate open question.

### BUG-COHERENCE-ESTIMATOR-STARVED (FIXED 2026-07-25): the Good/Moderate discriminator fed ZERO snapshots on the burst path since 2026-07-06
- Status: **FIXED 2026-07-25.** Root cause of the modem having no working channel-class
  discriminator — the estimator was never broken, it was STARVED.
- **Regression:** commit `1820987` (2026-07-06, "F142 census fixes") changed the single
  `addSnapshot` call site (`streaming_sync_acquisition.cpp:158`) from
  `if (std::isfinite(lts_mag) ...)` to `if (result.success && std::isfinite(lts_mag) ...)`.
  The F142 intent was correct (tone/false-lock snapshots with across-carrier CV ~3 had
  poisoned the score and pinned the ladder at R1/4 on a 24 dB link). But on the BURST path —
  the wideband file path — `populateDecodeMetrics` is handed a freshly default-constructed
  `DecodeResult` as a metrics TEMPLATE, built BEFORE the LDPC verdict exists, so
  `result.success` is **false by construction**. Net: ZERO snapshots on the only path that
  carries file data ⇒ `score_n_ == 0` ⇒ `coherenceScore()` returned a literal `0.0f` and
  `valid()` stayed false ⇒ `coherenceAdjustedFadingIndex` silently fell back to the blind CV
  for EVERY rate decision. Rig confirmation: **157/157** RX-AUTHORITY verdicts read exactly
  `coh=0.00` on BOTH MPG and MPM.
- **Measurement caveat worth remembering:** an earlier read of "coh mean 0.243, max 1.00" was a
  LOG-FIELD COLLISION — `channel_equalizer_lts.cpp:441` logs an unrelated CFO correlation as
  `coh=%.2f`. Always filter to the `RX-AUTHORITY verdict` line when reading the class input.
- **FIX:** `populateDecodeMetrics` takes an explicit `channel_evidence_ok` (defaults false, so
  unchanged call sites keep today's behaviour); the burst path passes its REAL per-frame
  verdict. The gate became `(result.success || channel_evidence_ok)` — F142's contract is
  intact (evidence still only from a successful decode), the burst path can now say so.
- **VERIFIED end-to-end on the faithful gate** (`gui_qso_scenario.sh`, 21 KB, seed 42):
  | channel | score | verdict | doppler | result |
  |---|---|---|---|---|
  | `--channel good` | **+0.606** | **[GOOD]** ✓ | 0.070 Hz (true Good ≈0.05 RMS) | PASS 2360 bps |
  | `--channel moderate` | **−0.285** | **[MODERATE/POOR]** ✓ | 0.000 Hz | PASS 830 bps |
  A 0.89 separation with the correct label on both, snapshots accumulating (24, 25 — was 0).
- **Note on `coherenceArea`:** a Stage-B change to route the CLASS decision through
  `coherenceArea()` was built and then REVERTED the same day. Independent re-simulation put it
  at d′≈3.0 with a **12.5% Moderate-read-as-Good** rate (the over-commit direction) vs the
  lag-1 score's d′≈8.5 — and the live Good-channel run above logged `area=-0.461 [MOD/POOR]`
  on a genuinely Good channel, confirming the misread empirically. The estimator header's claim
  that the area is "the radio-agnostic discriminator" is NOT supported; do not re-wire it
  without new evidence. Its ground-truth unit test feeds synthetic snapshots DIRECTLY, so it
  passes regardless of the production feed — it proves the maths, not the plumbing.
- Related and still open: [[BUG-FADING-INDEX-BLIND]] — the blind CV remains the FALLBACK
  whenever coherence is not yet valid (first ~31 frames), so a faster first-frame discriminator
  (demeaned |H|² frequency autocorrelation, d′≈11.4 at SNR 20 and available from frame 1) is
  still the right upgrade.

### BUG-FADING-INDEX-BLIND: the fading index cannot distinguish Good from Moderate — it measures the wrong physical quantity, so the Moderate/Poor anchor columns are effectively unreachable
- Status: **OPEN — measured 2026-07-25 on the IONOS rig against GROUND TRUTH.** Operator
  confirmed the dial setting; the IONOS manual (`docs/references/teensy_ionos_hf_manual_rev_2.03.pdf`,
  Fig 1/2 channel table) gives the physical parameters:
  WGN 0 Hz/0 ms · **MPG 0.1 Hz/0.5 ms (Good)** · **MPM 0.5 Hz/1 ms (Moderate)** ·
  MPP 1 Hz/2 ms · MPD 2 Hz/4 ms (spread is 2σ Doppler).
  So MPG→MPM is **5× Doppler and 2× delay spread** — the two channels the class boundary exists to separate.
- **MEASURED — the estimator does not separate them, and the sign is BACKWARDS:**

  | basis | MPG@20 (Good) | MPM@20 (Moderate) |
  |---|---|---|
  | fade-averaged (used by the RX-authority verdict) | mean 0.354, max 0.48, n=91 | mean ~0.35, n=88 |
  | single-frame (`LTS fading index`) | mean 0.686, **sd 0.875**, max 3.38, n=1309 | mean 0.526, sd 0.636, max 3.15, n=281 |

- **Root cause (`src/ofdm/channel_equalizer_lts.cpp:838-851`):** the index is the
  **across-carrier coefficient of variation of |H|** within one LTS observation
  (`raw_cv2 = var/mean²`, noise-corrected, `fading_index = sqrt(corrected_cv2)`).
  That is a *variance* statistic, and for a Rayleigh (Watterson) channel it converges to a
  CONSTANT — `CV = √(2−π/2)/√(π/2) = 0.5227` — **independent of delay spread and Doppler**.
  It answers "is this Rayleigh?", not "how dispersive / how fast". Every multipath mode is
  Rayleigh, so every multipath mode returns the same number. Single-frame values reaching
  3.38 (6× the physical ceiling) are estimator NOISE, not channel — sd is 1.27× the mean,
  consistent with the existing hygiene note that ">1.5 is a tone/noise snapshot, not a channel".
- **Consequence (this is the expensive part):** `selectCoherentOFDM` picks its anchor
  COLUMN by fading class. With the class pinned to GOOD on every multipath channel, the
  Moderate and Poor columns are effectively dead and the ladder uses Good-column rungs on a
  genuinely Moderate channel = systematic over-commitment. This is a standing crater source,
  independent of the +8.70 SNR offset. It also explains the anchor-skip gate's recorded
  puzzle ("the IONOS rig DISPROVED a PREDICTED coherence label: a Moderate channel read
  clean-Good for a whole ~60 s transfer") — that was NOT non-stationarity, it was this.
- **Note on `kFadingGoodMax` = 0.76** (raised from 0.65 on 2026-07-14 as the Good/Moderate
  ML midpoint): that change correctly killed the FALSE-Moderate flips coming from the noisy
  single-frame tail, and should stay. But the class centers it derives from
  (`kFadingCenterGood` 0.62, `kFadingCenterModerate` 0.90) both sit ABOVE the 0.523 Rayleigh
  asymptote, so on the fade-averaged basis the boundary can never be crossed. Raising the
  boundary was right; the underlying metric is the problem.
- **FIX DIRECTION — measure correlation STRUCTURE, not variance** (both are constant-free and
  radio-agnostic; we already hold per-frame `H` estimates from LTS + pilots):
  1. **Doppler / coherence TIME** — correlate frame *t*'s `H` vector with frame *t+1*'s.
     Frames are 1.27 s apart; MPG's Tc ≈ 3-4 s stays correlated while MPM's ≈ 0.6-0.9 s
     decorrelates. This is the **5×** axis — the cleanest separation and nearly free.
  2. **Delay spread / coherence BANDWIDTH** — correlate `H(f)` against `H(f+Δf)` and find the
     Δf where |corr| falls to 0.5. MPG ≈ 318 Hz vs MPM ≈ 159 Hz (6.8 vs 3.4 carriers per Bc
     at 46.9 Hz spacing) — a clean **2×**.
- **Also validates a design choice:** the group-size gate (`ULTRA_BURST_ESC_STREAK`,
  2026-07-24) was deliberately keyed on the delivery-driven clean-group streak rather than
  the fading index. Had it used fading, it would have been a compile-time TRUE. Any future
  channel-class gate must not trust this metric until it is replaced.

### BUG-TONEACK-EPOCH-UNPROTECTED: the tone-burst era echo is the only payload field outside CRC coverage
- Status: **OPEN — found 2026-07-24 during the F181 root-cause (not yet observed in the wild).**
- `tone_burst_constants.hpp` CRC-12 covers payload bits 0-25 + 38-39 (+42-43 under
  `ULTRA_RX_RATE_CMD`); bits **40-41 (`move_epoch`) are NOT covered**. They sit in Hamming
  block 3, so a double-bit error mis-corrects and can rewrite the era echo onto the
  sender's current `tx_epoch_` **with a valid CRC** — a stale ACK then passes the era gate.
- Related: `SelectiveRepeatARQ::reset()` zeroes `tx_epoch_` and restarts seqs at 0 under a
  DEBUG-only log, and `Connection::clearFileTransferArqState()` calls it MID-CONNECTION —
  a silent wrap-equivalent of the documented mod-4 residual.
- **Cheap fixes (neither needs new wire bits):** widen the CRC message to cover bits 40-41
  (lockstep both ends); and/or add monotone sample-index ACK freshness — `ToneBurstAckMonitor`
  already tracks `total_samples_fed_` and per-detection sample positions, so recording
  `epoch_bump_sample_` at every bump/reset and ignoring ACKs detected before it gives a
  PHYSICAL ordering that cannot alias, dominating any modulo counter (~10 lines, zero wire cost).
- Not urgent while the salvage is OFF (a stale ACK then costs at most a duplicate resend).

### BUG-SYNC-CURSOR-AHEAD: the sync cursor can run PAST live audio; the distance calc aliases that into a phantom ~50 s backlog, and the decoder fabricates a PING from zero-filled memory — FIXED 2026-07-30

**Symptom.** On the Pi 5 rig a handshake deadlocked for 420 s. The receiver's log claimed
the sync search had fallen far behind the antenna:

    [27.285] searchForSync: LOAD-SHED 2187129 samples (45.6 s) - search fell behind live

...when only 2.6 s of audio had EVER been fed to that decoder.

**The search had NOT fallen behind. It was 1.8 s AHEAD.** The 45.6 s figure is
`capacity - lead`, not a measurement. Forced reconstruction, no fitted parameters:
shed 2,187,129 + backlog_cap 124,800 = 2,311,929 = 2,400,000 - 88,071, i.e. a correlation
cursor 88,071 samples (1.835 s) past the write head.

**Three defects, each independently sufficient to keep the bug alive.**

1. **Unvalidated modular distance.** `sync_controller.cpp` computed
   `unsearched = write_pos_ >= correlation_pos_ ? diff : capacity - correlation_pos_ + write_pos_`.
   The else-branch is correct ONLY if the ring wrapped. When the cursor is instead merely
   AHEAD of live it returns a near-full buffer. The same open-coded distance appears in the
   readiness gate (`streaming_sync_acquisition.cpp`), which is how the decoder was allowed to
   slice memory it had never written.

2. **Two cursors from one expression, one clamped.** `streaming_ofdm_decode.cpp:915-916`:
   `correlation_pos_ = wrapRingIndexLocked(sync_position_ + min_frame)` (wrap only) sits one
   line above `setSearchFloorLocked(frame_sync_abs + min_frame)`, which has ALWAYS clamped to
   `total_fed_` (`sync_ring_buffer.cpp:68-73`). Only one of the pair was guarded.
   `sync_position_` itself can legitimately point past the searched window: it is a
   TRAINING-START offset (`mc_dpsk_waveform.cpp:136-141` adds chirp + gap to a peak bounded
   only by `window_len - chirp_len`, `chirp_sync.hpp:642-643`), giving a maximum of 124,800
   for a 120,000-sample window. The SEARCH_BACKTRACK cushion normally absorbs it; on a COLD
   ring (every pre-TX `reset()` rewinds and zero-fills, `streaming_decoder.cpp:1343-1393`) the
   first search runs with `search_start = 0` and no cushion at all.

3. **Digital silence read as PING evidence.** `evaluatePingFrame` forced `ratio = 0.0` when
   `training_rms <= kMinTrainingRMSForPingRatio` (0.001), and `ping_by_silence = ratio <
   0.5` — so the ONE observation that proves the buffer is invalid was the STRONGEST possible
   evidence of a valid PING. A live RX path cannot produce ~0.0 (measured floor 0.009-0.011;
   the gate's genuine PING reads train_RMS=0.3346). The rig logged
   `PING detected ... ratio=0.000 chirp_corr=0.837` — a real chirp lock with a fabricated
   payload verdict — which then set a search floor walling off the genuine CONNECT_ACK for
   2.5 s.

**Causal chain to the deadlock.** cold ring -> no backtrack cushion -> detector offset past
live -> readiness gate defeated by the same modular bug -> decode reads zeroes -> phantom
PING -> unclamped cursor parks in the future -> alias detonates the shed -> search blind 2.5 s
-> CONNECT_ACK missed -> responder's rescue retry disarmed by an unrelated false
`Accepted OFDM DATA sync (corr=0.59)` -> permanent half-open, 420 s.

**Fixes (all default-ON; correctness, not tuning).**
- `sync_controller.cpp` — invariant `unsearched <= total_fed_`. You cannot have more
  unsearched data than you have ever received; the invariant holds under every wrap state and
  needs no knowledge of which writer misbehaved, so it catches the whole family. Violation
  logs ERROR and re-anchors the cursor to the write head.
- `streaming_sync_acquisition.cpp` — clamp `sync_position_` at its SOURCE. 15+ downstream
  `correlation_pos_` writers are all of the form `sync_position_ + <len>`, so a source that
  can never exceed live audio cannot produce a cursor in the future. One edit, not 26.
- `streaming_frame_policy.hpp` — `buffer_invalid` when `training_rms <= 0.001`; suppresses
  `ping_by_silence`, `ping_by_chirp_lock` AND `is_ping`. Suppressing the silence path alone is
  insufficient: the chirp-lock route reaches the same verdict because zero `data_rms`
  trivially clears `kPingChirpLockMaxDataRMS` (0.16). Keyed on TRAINING rms specifically so
  the low-SNR PING path (#70), which accepts weak payloads via `data_rms`/PATH2, is unaffected.

**Verification.** `test_streaming_frame_policy` 47/47 — replays the rig's exact inputs
(zero-filled slice, chirp_corr 0.837, no LDPC attempt) and requires no ping verdict by EITHER
route, plus a genuine low-level PING that must still detect, plus the threshold boundary.
One pre-existing assertion was CORRECTED: it required `evaluatePingRMS(nullptr, 0)` to
classify as a PING — the same defect in miniature. Safe to change: that wrapper has no
production callers (verified by grep); the only production entry is
`streaming_ofdm_decode.cpp:843`. Full suite 99/99. Faithful gate
`gui_qso_scenario.sh --channel good --snr-db 20` PASS, 2350 bps, CRC ok, and neither new
guard fired spuriously.

**Handshake-chain closure (2026-08-03).** The final half-open link in that causal
chain is now repaired without reviving the collision-prone timer. Accepted OFDM
sync no longer confirms or clears the cached CONNECT_ACK; empty/all-failed groups
are non-authoritative. The responder keeps only a cache for reactive replay after
a decoded duplicate CONNECT. A connected wide-OFDM responder also makes one
bounded fixed-DBPSK MC-DPSK decode from the retained raw ring when a true expected
full anchor arrives, then silently tears down the false OFDM group and restores
the pre-half-open SNR/EVM/fading baseline. Focused regressions cover two successive
lost replay ACKs; fresh IONOS transfer validation remains pending.

**Audit provenance.** 65-agent adversarial audit, 15 findings confirmed / 44 refuted; the
load-bearing file:line claims were re-verified by hand before acting.

---

### BUG-DECODE-BACKLOG-COLLISIONS: under deep-fade search thrash the decoder falls 10-20 s behind LIVE audio — every receiver response (ACK, backstop, adopt) leaves stale, colliding with the sender's already-airing recovery bursts
- Status: **OPEN — pinned 2026-07-07 (F176 rig, MPG@20).** Hard evidence: a burst
  AIRED at t≈218, its anchor was ACCEPTED by the decoder at t≈239 (**~20 s of
  processing lag**), the anchored-burst backstop (sample-clock, correct by its own
  basis) therefore fired at t=251 — exactly as the sender's next recovery burst
  keyed. Operator waterfall shows the ACK tones over the burst head.
- **Mechanism:** when fades deepen (F176 post-80%: CFO drift to −0.49, 2/5
  groups), the sync search THRASHES — full-anchor-wait reject streaks with
  decaying thresholds, weak-DATA fallbacks, repeated correlation over the same
  audio — and per-buffer processing cost exceeds real time. The ring keeps
  filling; the decoder's "now" detaches from the antenna's "now".
- **Why no receiver-side gate can fix collisions while this holds:** the
  geometric ACK gate (F176 fix), CCA, and decoder-evidence checks all consume
  DECODED state; a burst the decoder hasn't reached yet is invisible to all of
  them. With the decoder 20 s behind, the receiver is answering questions from
  20 s ago. This is the same #56/RXQ class that defeated warm-sync in June
  (rig turnaround forensics) — now shown to also manufacture TX collisions.
- **MITIGATED 2026-07-07 (same night):** search LOAD-SHED landed — real-time
  decoders (production engine opts in; batch tools/tests never shed) cap the
  search backlog at 2 s: the search floor jumps forward and eats the loss
  (WARN-logged with shed seconds). Receiver staleness is now bounded at ~2 s —
  the collision class and 20 s-stale responses are structurally gone. The
  original fix direction below remains for the underlying thrash cost:
- **Fix direction (deeper, next session):**
  bound the search cost per fed buffer (correlation work budget per real-time
  interval; skip-ahead instead of re-correlating overlapping windows on reject
  streaks), and/or a load-shedding rule: when unsearched backlog exceeds ~2 s,
  jump the search floor to (write_pos − backlog_cap) and eat the loss — a
  real radio that falls behind MUST drop audio, not time-travel. Measure
  backlog_ms (already in DecoderStats: current/peak_unsearched_samples) around
  fade episodes to size the budget.
- **Impact:** the top remaining source of ACK/burst collisions and late-response
  stalls; also inflates every ack round-trip during fade episodes (the F163
  budget's "RTO dead-air" rows are partly this).

### BUG-POSTTX-ACK-MISS: tone monitor tail-window sweep never scans audio deeper than ~520ms into a single feedAudio append — the first ACK after our own key-down (capture-resume backlog) is captured, fed, and never scanned (~19s RTO each, 2/run F76-F77)
- Status: **FIXED 2026-07-05 (ULTRA_ACK_MONITOR_GAPLESS, DEFAULT-ON since 6340f51+flip; =0 opts out).** Root cause: all cadence passes triggered inside one feedAudio(count) call scan the SAME end-anchored window (tail_base = buffer end - window); a chunk larger than a bin's tail window leaves a permanent blind hole (12ms bin window ≈ 25k samples). Fade and staircase-bin mismatch REFUTED by capture ledger (tone arrived rms 0.033-0.091, all ACKs symbol_ms=12).
- Fix: gapless armed sweep — per-bin window extends to cover everything since the scan high-water mark + one burst of context (gapless by induction; hw jumps to buffer end per pass; end-straddling tones re-covered next pass). Plus permanent forensics: arm log INFO, armed-window-EXPIRED-undetected INFO (fed_in_window/max_chunk/passes classifies any future miss), monitor-level detection INFO chained in front of the production callback.
- Validation: 10-run rig batch F78-F87 — ~280 ack exchanges, 0 misses, 0 expired-undetected (was 2/run); every RTO classified genuine fade. Sim parity (1960 PASS), tone-burst tests 5/5.

### BUG-ACKLISTEN-TONE-FALSELOCK: the sender's warm data-sync detector S&C-false-locks on the PEER'S TONE-BURST ACK during ACK-listen — garbage decode + blind re-search races the tone monitor for the same samples; the rig loses the race (missed ACK -> ~28s RTO stall / demote spirals)
- Status: **FIXED 2026-07-05 (knob ULTRA_ACKLISTEN_SUPPRESS_OFDM, default-OFF, in the standing campaign config; 7752d60 — sync_controller.cpp guards in detectConnectedLightSync + detectFullAnchorFallback).**
- **CORRECTS the BUG-SELF-ECHO-REANCHOR-STALL attribution below** (2026-07-05 forensics, F73/F74 + three-way proof): the corr 0.9x locks during ACK-listen are NOT self-echo — (a) the OTASim server EXCLUDES self-audio by construction (`ota_channel_core/mixer.cpp:40-42` skips blocks whose station_id == receiver); (b) a solo-station control (one GUI alone on OTASim, PING into the void) heard NOTHING; (c) on the rig, capture is STOPPED during our own TX (app.cpp:3360-3364 setRxMuted + stopCapture) — recording our own burst is physically impossible. The locks are the peer's 4-FSK tone ACK: its periodic carrier lead-in + repeated FSK symbols score ~0.9+ on the Schmidl-Cox metric while the LTS matched filter stays ~0.1 — the sc-high/mf-low signature on EVERY such lock, sim and rig; in sim the locks land exactly on the ack-repeat copy (+376ms). A threshold cannot gate this (0.94 > every decayed threshold, incl. the full-anchor-wait 0.52 gate it slipped in F74) — only unconditional suppress-while-armed works. The dual-chirp path stays live (a tone cannot fake the up+down pair at the 28800-sample gap), so full-anchor control frames still acquire during the window; reject streaks / §16.4 untouched.
- Validation: OTASim A/B seed42 good@20 tone-locks 50->0, 0 resends, PASS 1990 (baseline 1730). Rig F75: first ACK 9.9s (F74: 28.5s + wasted resend + demote-to-R1/2), ACK cadence 9.56s metronomic, 0 timeout-resends, 0 craters, **2.62 kbps all-time rig record** (single cell ±25-30%; the mechanism metrics are the proof). Multi-seed validation queued.
- The earlier ULTRA_ECHO_REANCHOR_GATE (bfe5676) suppressed one downstream *reaction* (the 0-CW re-anchor) to the same tone-locks — mechanism mislabeled, suppression directionally right (+17% F65-F68). Both guards stack; bfe5676's log line still says "SELF-ECHO" (rename queued).
- Related F73 lesson (ULTRA_ENTRY_QAM16_SNR, 20f6006, default-OFF): cold 16QAM connect entry decodes marginal (quality 0.35, no warm estimate) — its slow decode widened this race and collapsed the ladder to R1/4. Keep entry at QPSK; the climb warms the equalizer.

### BUG-SELF-ECHO-REANCHOR-STALL: sender decodes its own burst echo during ACK-listen, re-anchors, and MISSES the receiver's crater-demote tone-ACK -> 2x-RTO collapse (~36s) instead of a ~3.5s demote
- Status: **FIXED 2026-07-04 (knob ULTRA_ECHO_REANCHOR_GATE, default-ON; streaming_ofdm_decode.cpp self-echo guard).** Root-caused by forensic wrp84o66o (high confidence, clocks aligned Mac=Pi5+22.48s). **2026-07-05 MECHANISM CORRECTION: the "own burst echo" attribution was WRONG — see BUG-ACKLISTEN-TONE-FALSELOCK above (the signal was the peer's tone ACK; self-echo is physically excluded on both benches). The fix keyed on the right state (monitor armed) and remains valid.**
- **THE 2.50->1.44 GAP.** F3 (morning 2.50) and afternoon runs deliver the SAME clean groups/craters/signal-level; the only difference is per-group cadence (10.2 vs 15.8 s/group). Steady state is 9.4s (fine); the gap is occasional 30-36s STALLS. Mechanism: a 16QAM R2/3 group genuinely craters on a deep fade (thin-margin payload dies, QPSK R1/4 header survives corr 0.99). The receiver immediately sends the crater-demote command (kRungCmdDownHard) on its tone-ACK — the fast path that resolves craters in ~3.5s (F4/F6/F7/F10/F11). BUT the sender never hears it: its RX path is decoding its OWN transmitted burst echoing back (self-echo, corr 0.96-0.98 at sample positions inside its just-transmitted span), each 0-CW fail re-arms expect_full_ofdm_anchor_ -> a 120000-sample blind search that consumes the RX path -> the tone-ACK is missed -> 2x18s RTO + collapse-escape = 36s.
- **Fix:** while tone_burst_monitor_.isArmed() (= we are the data-sender in post-burst ACK-listen; the peer emits ONLY a tone-ACK then), suppress the destructive full-chirp re-anchor on a 0-CW OFDM "decode". A genuine receiver decoding real peer OFDM never arms the monitor, so its legitimate crater re-anchoring is untouched. Sender-local, no wire change. Expected: 36s -> ~4-10s; most afternoon runs jump from ~1.4 toward the 16QAM ceiling.
- Not to be confused with BUG-BURST-HEADNULL-DROP (a fade nulls the group HEAD -> clean frames 2..N dropped; separate, still open).


### BUG-RESPONDER-HANDSHAKE-NEVER-CONFIRMS: responder handshake_confirmed_ only flipped in the CLASSIC frame path — a descriptor-era burst-only session never confirms, so the modem TX-routes every classic control frame via the handshake last-RX-waveform mirror = 3.1 s MC-DPSK DBPSK (operator saw "MC-DPSK at the end of the run")
- Status: **FIXED 2026-07-04** — confirm extracted to maybeConfirmResponderHandshake, now also fired on the first delivered burst group (equally hard evidence the initiator heard our CONNECT_ACK). Legacy runs confirmed via the initiator first MODE_CHANGE side effect (OFF-arm log: confirm@77s); full-descriptor runs confirmed only at DISCONNECT (@294s). Exposed by Phase 1/2 eliminating classic control frames by design.


### BUG-STAIRCASE-SNAPSHOT-INPUT: the tone-ACK staircase reads a single last-frame SNR cache write — no validity gate (a total-erasure slot at noise-floor RMS with 100% zero soft bits still lands in the cache), no fade averaging, no rung clamp
- Status: **ACTIVE, deferred (2026-07-04 forensics link 2).** The detonator (phantom frame from the DESC-SWITCH group-size clobber) and the trap (monitor buffer < 100ms rung) are both FIXED 2026-07-04, which de-fangs this for the observed incident class; the input remains unprincipled. Fix direction: gate cache writes on frame validity (zero-soft-bit/noise-RMS slots are not measurements) + fade-averaged/median statistic over kept frames (aligns with task #58) + clamp emitted rungs to the sender-decodable set.


### BUG-UNANCHORED-SILENCE-ESCAPE: a lost EPOCH_REBASE head frame makes the receiver by-design ACK-SILENT while it keeps DELIVERING data → the sender's zero-progress collapse-escape reads the silence as a forward-link crater → MANUFACTURED demote of a working rate + forced re-delivery
- Status: **FIX IMPLEMENTED 2026-07-04 (WAITING-REBASE voice: rung_cmd=3 under ULTRA_RX_RATE_CMD — unanchored receiver voices per group; sender resets zero-progress evidence + standalone era-base resend; design \x{a7}5.3). Validation: build+ctest+sim, then rig batch.**
  forensics, HIGH confidence, no rerun needed). Fix in progress.**
- Mechanism (E1, IONOS MPG@20, clocks aligned Mac=Pi5+16.93s): after the demote-1 epoch
  rewind, the epoch-1 EPOCH_REBASE frame (seq 33, head-of-burst = the most fade-exposed
  slot: acquisition marker-timing retries logged) never decoded across 3 rounds. The
  receiver adopted epoch 1 UNANCHORED → deliberate ack-silence
  (`selective_repeat_arq.cpp` interregnum rules) while still SALVAGING frames 34-37 every
  round — receiver rx_base=38 (everything delivered) vs sender acked=33. Two ACK-less RTO
  rounds → collapse-escape demoted working R2/3 → R1/2 (+ another full stop-and-wait
  exchange + re-delivery). ~83 s of a 600 s window burned. Same signature in D1 (base 15
  vs rewind 10) and D3 (four cycles, two 4-retry exhaustions).
- Fix direction (ranked by the adjudicator): give the unanchored receiver a VOICE on the
  4-FSK tone plane (rung_cmd reserved value 3 = "waiting-rebase" under `ULTRA_RX_RATE_CMD`)
  → sender resends the era-base frame instead of counting zero rounds; escape policy
  becomes reverse-path-aware (silence-while-unanchored ≠ forward crater).

### BUG-MC-RETRY-SPURIOUS: MODE_CHANGE retry timer is anchored at REQUEST time, but the frame rides the TAIL of the sender's own bundled key-down (~10.6 s) → the 18.2 s deadline structurally loses to the real 21-30 s pipeline → EVERY trough exchange retries even though copy #1 was ACKed
- Status: **ACTIVE (proven on E1 demotes 1+2, D1, D3 — all observed cycles 21.07/21.27/30.37 s
  vs 18.2 s timer; the winning MC-ACK had fully ARRIVED 1.9-3.0 s before each retry fired —
  the sender's own RX decode pipeline surfaces control decodes only after key-up).
  ALL FOUR FIXES IMPLEMENTED 2026-07-04:** (1) TX-hold retry clock (setTxActiveProvider — deadline holds while keyed), (3) receiver same-(seq,mod,rate) dedup single-re-ACK, (4) stale CCA-deferred data-TX purge on mode commit, plus (2) the waiting-rebase voice above. Validation: build+ctest+sim, then rig batch.
  carried the frame (evidence-driven), not at request time. Related receiver-side wart:
  `handleModeChange` has NO same-seq dedup — every duplicate copy re-applies + re-notifies
  + emits a fresh fading-aware 3-copy ACK set (the operator-visible dup [MODE] lines).
- Also caught in the same forensics: a stale CCA-deferred TX burst rendered at the OLD
  rate/epoch pre-commit was flushed post-commit → 9.0 s of undecodable airtime (fix:
  drop/re-render deferred audio on mode/epoch commit).

### BUG-ARQ-SEQ-COLLISION: rate-change abort under ONE-WAY ACK loss re-chunks DIFFERENT file bytes under seq numbers the receiver already retired → receiver's seq-keyed dedup destroys them before the offset-idempotent file layer can see them → permanent byte hole + sender false-complete + receiver stranded
- Status: **STRUCTURAL FIX IMPLEMENTED 2026-07-03 (knob-gated `ULTRA_ARQ_MOVE_EPOCH`,
  default OFF = byte-identical; edits-only/UNVALIDATED — no build/ctest/gate run yet;
  rig-validation-pending with BOTH stations knob-ON — WIRE/SEMANTICS-BREAKING when ON,
  lockstep, no capability negotiation this increment).** Interim receiver-side salvage
  (`ULTRA_BELOW_WINDOW_FILE_SALVAGE`, now DEFAULT-ON, 9/9 rig field engagements) stays as
  belt-and-braces. Root-caused from rig W16 (IONOS MPG@20, Pi5 sender → Mac receiver,
  50 KB) by multi-agent forensics + adversarial verify, 2026-07-03.
- **Downstream dependency (2026-07-03):** the descriptor-committed mode switch
  (`ULTRA_DESCRIPTOR_MODE_SWITCH` Phase 1, docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md)
  HARD-DEPENDS on this epoch machinery for its Phase-2 ESCAPE (mid-window) commit path —
  until `ULTRA_ARQ_MOVE_EPOCH` rig-validates, escape drops stay on the legacy MODE_CHANGE
  exchange even with the descriptor-switch knob ON (Phase 1 = clean-boundary commits only,
  which need no epoch: an empty window has nothing to abort).
- **Structural fix (move-epoch) — what landed (unit-test-edited, unvalidated):**
  - **Epoch state:** two independent per-direction 2-bit (mod-4) counters in
    `SelectiveRepeatARQ`. `tx_epoch_` bumps exactly when `setCodeRate`'s TX-abort branch
    rewinds `tx_next_seq_` (the collision precondition; `setFixedFrameCodewords` →
    `abortPendingTx` abandons seqs FORWARD — no re-use, no bump). `rx_epoch_` is adopted
    from the wire (a receiver with nothing in flight never bumps locally).
  - **Wire (all bits are 0 when OFF = byte-identical):** DATA flags bits 6-7 carry the
    epoch (formerly the never-implemented rate-in-flags bits) and `EPOCH_REBASE` = the
    formerly never-implemented ENCRYPTED bit 0x08, stamped on any DATA frame created
    while its seq == the sender's window base ("nothing un-retired below me in this
    era" — invariant holds for the frame's life; baked into the serialized bytes so RTO
    resends re-carry it). ACK echo: v2 SACK bitmap bits 16-17 (window bitmap occupies
    bits 0-15 with MAX_WINDOW=16; bits 24-31 must stay clear of the `decodeSackBitmap`
    legacy-8-bit shim — that's why NOT bits 30-31); tone-burst ACK payload bits 40-41
    (the former Hamming zero-pad — still transmitted, so airtime is unchanged at 34
    symbols; deliberately NOT CRC-covered, else the knob-OFF CRC would change — a
    Hamming-miscorrected epoch fails SAFE: the ACK is ignored = ACK-lost = RTO resend).
  - **Sender gate:** `handleAckFrame` extracts+strips the echo and IGNORES any ACK whose
    epoch != `tx_epoch_` (log `stale-epoch ACK ignored`, stat `stale_epoch_acks_ignored`,
    phy-diag `reason=stale_epoch`; returns before the dedup signature and §RETX-PACING
    progress sentinel). This kills W16 kill-arm 2 (the out-of-window SACK crediting
    phantom chunks) AND structurally cures the review-flagged below-base zombie-TX wart
    (a late stale ACK can no longer advance tx_base past the rewound tx_next).
  - **Receiver adoption (the risky part — NOT naive re-anchor-to-incoming-seq):** on a
    DATA frame with epoch != `rx_epoch_` (serial half-duplex channel ⇒ any change is a
    newer era), adopt + discard all rx slots/pending-ack state, then anchor ONLY on an
    `EPOCH_REBASE` frame (`rx_base_seq_` = its seq — cumulative claims below it name
    only sender-retired seqs, zero fabrication). A rebase-less adoption (era head lost)
    enters an ACK-SILENT unanchored interregnum: no window bookkeeping, ALL acks
    suppressed (any cumulative claim from the old rx_base would fabricate delivery of
    new-era seqs = the phantom-retire disease), FILE payloads salvage-delivered
    (offset-idempotent), TEXT dropped (resent after anchor — in order, no dupes); the
    sender's RTO resends base-first and the rebase frame anchors us. Naive
    re-anchor-to-first-heard-seq was rejected: under head loss it makes the next
    cumulative ACK claim the lost head frames → sender retires them → their bytes
    permanently unresendable — recreating the hole. This kills W16 kill-arm 1 for ALL
    payload types (the salvage only covered FILE).
  - **Files:** `selective_repeat_arq.{hpp,cpp}` (state machine — full spec in the hpp
    MOVE-EPOCH block comment), `frame_v2.hpp` (`Flags::EPOCH_MASK/EPOCH_SHIFT/
    EPOCH_REBASE`, `epochFrom/ToFlags`), `arq_interface.hpp` (stat),
    `tone_burst_constants.hpp`/`tone_burst_payload.{hpp,cpp}` (bits 40-41),
    `connection.cpp` (tone-ACK emit/consume plumb). Unit tests:
    `test_selective_repeat` (`test_move_epoch_*`: bump+stamp on abort; the W16
    below-window regrid delivered as TEXT — proving epoch, not salvage; stale-epoch ACK
    ignored + fresh retires; unanchored ack-silence → late-rebase anchor; knob-off
    byte-identical) + `test_tone_burst_ack_payload` (epoch roundtrip/clamp + CRC-field
    invariance proof).
  - **Residuals (documented, accepted):** mod-4 wrap — 4 TX aborts with ZERO frames
    decoded in between return to the same epoch (falls back to today's salvage
    behavior); adoption only from COMPLETE DATA frames (stale/foreign-era partials and
    unanchored partials are dropped; DATA_REPAIR-sourced partials are epoch-exempt —
    synthesized flags — and cross-era merges are frame-CRC-rejected); GROUP_ACK/
    GROUP_NACK/NACK frames carry no epoch (they trigger retransmits, never retirement —
    a stale NACK costs at worst one duplicate resend); a rebase frame that exhausts
    max_retries kills the transfer exactly as an undecodable frame does today (no new
    failure mode); MODE_CHANGE ACKs are intercepted by the Connection before the ARQ
    and carry no epoch.
- **Mechanism (the full chain):**
  1. During a 16QAM R2/3 epoch the receiver DELIVERED seqs 69-77 (384 B/chunk grid),
     advancing its `rx_base_seq_` to 78 — but the sender heard NONE of the ACKs (one-way
     ACK loss: data direction alive, ACK direction dead). Sender base stayed 69.
  2. The collapse-escape's rate-change abort rewound the sender to its OWN stale base:
     `SelectiveRepeatARQ::setCodeRate` does `tx_next_seq_ = tx_base_seq_`
     (`selective_repeat_arq.cpp:~76`), and `FileTransferController::requeuePendingChunks`
     resumed at the (now exact, post-BUG-FILE-REQUEUE-OFFSET) sender-ledger offset 30576 —
     re-chunking bytes 30576+ on the NEW QPSK R3/4 grid (456 B/chunk) under the SAME
     seqs 69-77.
  3. New-69..77 covered bytes [30576, 34680) but the receiver's ARQ dedups BY SEQ, not by
     byte: the frames died at the below-window drop (`selective_repeat_arq.cpp` `handleDataFrame`
     out-of-window branch, ~:645-675) BEFORE reaching the offset-idempotent file layer —
     whose straddle-merge (`file_transfer.cpp` `processFileData`, ~:612-621) was built
     exactly for regrid resends and never ran. Old-69..77 covered only [30576, 34032)
     (9×384), so bytes [34032, 34680) — exactly 9×(456−384) = **648 B** — existed ONLY in
     the destroyed frames.
  4. Each below-window frame triggered an out-of-window SACK carrying cumulative base 78;
     the sender (hearing ACKs again by then) interpreted it as delivery of new-69..77 →
     the phantom chunks retired forever (`onChunkAcked`), never resendable.
  5. Ghost transfer: everything from seq 78 (offset 34680+) was received AND ACKed but
     buffered out-of-order behind the 648 B hole (~16.5 KB); contiguous edge frozen at
     34032 (66.5%). Sender declared "Transfer complete" (identity-blind counter equality,
     `file_transfer.cpp` `onChunkAcked` — no receiver confirmation; sibling:
     BUG-FILE-ACK-IDENTITY), idled out the 600 s grace, disconnected; receiver stranded
     237 s then "Transfer cancelled".
- **Precondition:** `rx_base > tx_base` at a rate-change abort — i.e. the data direction
  working while the ACK direction is dead, exactly the asymmetric failure the rig exhibits
  (W15 shows the same one-way signature without a rate change → RTO grind instead of a hole).
- **W16 evidence** (`/tmp/campaign_3000/pi5_W16_gui.log` (P) /
  `rig_W16_failed_mac_gui.log` (M), Pi5 clock = Mac − ~6 s; full forensics in the
  2026-07-03 W16 workflow output): P304.097 "Re-queued 9 pending chunks after ARQ abort
  (acked=69, resume_offset=30576)" + "SR-ARQ: Code rate changed, aborted 9 unACKed
  in-flight TX slots (cleared 0 SACKed); rewound TX seq to 69"; M320.205 "SR-ARQ: DATA
  seq=69..73 outside window [78, 94)" and M332.767 "seq=74..77 outside window [78, 94)";
  P397.334 "[FILE] Transfer complete (362.8s, 1.13 kbps)" (false complete); M648.694
  "[FILE] Transfer cancelled". Chunk-grid arithmetic exact: 22×456 + 6×384 = 12336 and
  (30576−12336)/40 = 456.
- **Why the earlier W16 escape (t=132) was benign:** that 16QAM epoch had failed BOTH
  directions (Mac logged 0/8-CW decode failures at 116-162), so `tx_base == rx_base == 29`
  at the abort (P161.263 "acked=29, resume_offset=12336") — same seqs re-covered the same
  bytes, no collision. The bug arms ONLY on one-way ACK loss.
- **Relation to prior work:**
  - BUG-FILE-REQUEUE-OFFSET (FIXED 2026-07-02): fixed the SENDER-side resume arithmetic
    (offset ledger). W16 proves the requeue is still destructive when the RECEIVER's base
    is ahead — the ledger resume is exact w.r.t. the sender's knowledge, but the sender's
    knowledge is stale by construction under one-way ACK loss. Same gate-bypassing
    escape-drop path (the clean-boundary gate cannot help: the escape fires with frames in
    flight BY DESIGN).
  - The review-flagged **stale-ACK epoch hazard** (2026-07-02-late adversarial review of
    the requeue fix): ACKs are epoch-blind — an ACK formed against the OLD chunk grid (or
    an out-of-window SACK carrying an advanced base) is credited by the post-abort sender
    against NEW-grid chunks carrying different bytes. W16 is the live confirmation of that
    hazard. The verify also found the abort leaves `tx_next_seq_` below an ack-advanced
    `tx_base_seq_` (`handleAckFrame`'s cumulative advance, `selective_repeat_arq.cpp:~863-885`,
    never re-clamps it) → 4 below-base zombie transmissions observed on air (the M332
    reject group).
  - 2026-06-09 verdict vindicated: gate-less escape-drop (abort-coordinated requeue) is
    unsafe; this is the rig proof.
- **STRUCTURAL fix direction (IMPLEMENTED 2026-07-03 as `ULTRA_ARQ_MOVE_EPOCH` — see the
  Status block above; kept for the record):** a **move-epoch** counter
  carried on DATA frames and echoed in ACKs, bumped at every rate/mod-change abort, so a
  stale-epoch ACK can never retire new-content seqs (the sender ignores ACKs whose epoch
  predates its current grid; the receiver's out-of-window SACK for old-epoch frames is
  then harmless). Complementary candidates from the forensics (still OPEN): carry the receiver's
  `rx_base_seq_` + contiguous byte offset in the MODE_CHANGE_ACK (which demonstrably DOES
  arrive — it triggers the abort) and resync the sender to the RECEIVER's state; and the
  receiver-confirmed completion handshake (DATA_END, CLAUDE.md Known Limitation 4) so a
  false sender-complete is detected and hole-repaired by byte offset while airtime remains
  (W16 had 244 s of idle grace and needed ~120-150 s).
- **Interim salvage (landed 2026-07-03, default OFF, rig-validation-pending):**
  `ULTRA_BELOW_WINDOW_FILE_SALVAGE=1` — receiver-side only: in the ARQ's below-window drop
  path, a frame whose payload decodes to FILE_START/FILE_DATA (`PayloadType`,
  `file_transfer.hpp`) is handed up the SAME delivery callback in-order frames use
  (Connection `handleDataPayload` → `file_transfer_.processPayload`) BEFORE the (unchanged)
  out-of-window SACK; the file layer's offset dedup + straddle-merge make double delivery
  safe by construction. NEVER salvages other payload types (messages are seq-deduped only —
  re-delivery would duplicate them) and never far-future seqs (strict half-space
  below-window test). Converts W16's 648 B hole into a delivered straddle-merge chunk: the
  contiguous edge advances, the ghost 16.5 KB drains, the receiver finalizes. Log grep:
  `SALVAGE below-window FILE frame`. Unit-tested in `test_selective_repeat`
  (`test_below_window_file_salvage`: knob-on FILE_DATA/FILE_START salvaged + SACK
  unchanged, TEXT never, far-future never, knob-off byte-identical drop).

### BUG-BURST-HEADNULL-DROP: group-head fade null → clean mid-group frames silently dropped (no decode attempt, no counter, no ACK credit) → whole-group RTO saga
- Status: **OPEN — root-caused in code + reproduced in logs (2026-07-01 audit, adversarially verified).** Found via `fable_analysis/09_WHY_STUCK_AT_2000_2026_07_01.md` §3.2.
- **What:** burst accumulation is marker-gated — only the group-start frame (negated-LTS marker) can enter accumulation (`streaming_ofdm_decode.cpp:~1090` marker check, `:1271` entry). When a fade null covers the group head (LTS + frame 1), frames 2..N arrive clean (observed corr 0.95-0.96, LTS SNR 26.7-27.0 dB), are sync-accepted (`accepted connected DATA sync fallback`), fail the 1-CW control-first peek, and are consumed by the **log-less** mid-burst re-search (`streaming_ofdm_decode.cpp:1042-1052`): no 8-CW decode attempt, no counter, no partial-ACK credit. Each occurrence costs a full sender RTO (~24.5 s) + a whole-group resend (~8 s). Observed: Good@20 seed-42 stock run lost 83 s on one group (3 resend attempts; ~20 s of clean 27 dB frames received and discarded) = ~23% of the transfer.
- **Why the silent path exists (do NOT naively fall through):** the legacy control→data profile fall-through double-demod **poisoned the burst's shared coherent channel estimate** (§14.24/§14.25, comment at `streaming_ofdm_decode.cpp:1021-1029`); the gate was also narrowed for BUG-TNC-B2F-001. A fix must re-demodulate from the ring at the data profile (fresh estimate), not fall through.
- **Fix path:** (1) FREE: add a counter/log for "sync-accepted data-profile frame consumed without decode attempt" (today it is invisible). (2) The descriptor already gives the receiver the group geometry — enter accumulation from ANY group-member sync with the head erasure-marked, converting the saga into a 1-frame nack. (3) Interim mitigation is the RX group-timeout fast-NACK, which requires the BURST_HEADER *and* ≥1 decoded data frame — the head-null case defeats it.

### BUG-ACK-STAIRCASE-FADE-BIN: SNR-adaptive fast tone-ACK never engages on fading channels — **FIXED (b85c0e1, 2026-07-01) — this entry was STALE; occupancy verified 2026-07-03**
- Status: **FIXED + GATE-VERIFIED.** The fix landed in b85c0e1 ("revive the SNR-adaptive ACK
  staircase (feed + edge)"): per-group broadband SNR feed (the cache had been FROZEN at the
  handshake MC-DPSK reading for whole transfers) + the fading-conditioned fast edge
  (`kFastAckEdgeFadingDb=16.0` vs AWGN 18.0, `tone_burst_constants.hpp` `symbolMsForSNR`).
  The staleness was found by the 2026-07-03 campaign Phase-0 audit. Occupancy measured on the
  2026-07-02 5-cell gate: g42 = 28/30 fast (12 ms), g43 = 35/38, g7 = 24/33, AWGN = 17/17 —
  the fast rung engages on fading. Residual: the 100 ms rung fires in trough sagas at RTO
  cadence (g7: 5×2700 ms), which is detection-safety-correct behavior, not this bug.
- History (kept for the record): the §15.5 staircase picked 12 ms at ≥18 dB but was fed
  fade-effective SNR (~16-17 at Good@20) AND a frozen cache → 0% fast occupancy everywhere
  incl. 30/30 rig ACKs at 675 ms. Same disease family as #74/#58 (fade-aware measurement vs
  AWGN-calibrated threshold) plus a dead feed. NOTE 2026-07-03: the tone-burst payload widened
  32→40 bits (16-bit SACK mask) — ACK airtimes are now 408/850/1700/3400/6800 ms per rung.

### BUG-CONNECT-SNR-VARIANCE (#58 completion): the connect-time SNR is a single 170 ms snapshot on a ~4 s-fade channel → ~10 dB pick-to-pick spread; the +2 dB basis correction fixes the BIAS, not the VARIANCE
- Status: **FIX IMPLEMENTED (increment 2, 2026-07-02, default-ON) — pending rig
  connect-spread re-measure.** The designed data-aided estimator LANDED
  (`updateDataAidedSNREstimate`, `multi_carrier_dpsk.hpp`; routed in
  `populateDecodeMetrics` via `ULTRA_CONNECT_DATA_AIDED_SNR`, default-ON, `=0` opts out;
  decode-then-measure — routed only when the frame's LDPC decode succeeded, else the
  training snapshot). Differential-level (unit-phasor chord error vs the config-driven
  DBPSK/DQPSK constellation, ~0.2 s block averaging ≪ Tc), so it needs NO static channel
  reconstruction and is drift-immune across the multi-second frame. Calibration derived,
  not tuned: magnitude normalization cancels the differential +3.01 dB; geometry-computed
  non-orthogonal-carrier ICI (−29 dB/carrier @8×1024, matches measured excess) subtracted;
  (k−2)/k inverse-chi-square block correction; +0.5 dB measured residual. GATED in
  test_mcdpsk_snr_calibration: AWGN within 1 dB at 0/5/10/15/20/25 dB (measured
  +0.5/−0.1/−0.1/−0.0/+0.4/+0.1). Both values logged per frame
  (`MC-DPSK SNR: training=X data_aided=Y (routed=...)`) for the rig spread comparison.
- Rig MPG@20 evidence (one evening, 6 connects): snapshots
  18.2 / 17.0 / 14.6 / 14.5 / 12.4 / **8.4** while the chirp-quality proxy read 22-27
  throughout. The 8.4 connect survived to OFDM only by 0.4 dB (sel=10.4 vs Good floor 10) and
  picked **QPSK R1/2** — half throughput on a channel that carries R2/3. Deeper dips than the
  +2 average penalty WILL occasionally cross into the MC-DPSK branch (the residual coin-flip).
- **Root cause was architectural, not the estimator** (AWGN-accurate to 30 dB,
  test_mcdpsk_snr_calibration): `updateTrainingSNREstimate` measures ONLY the ~170 ms training
  preamble = one fade state (Tc ≈ 4.2 s at Good); the CONNECT frame's 4 CWs span ~7.1 s ≈
  1.7 Tc and are fully decoded before the pick → the whole-frame estimate is fade-AVERAGED
  by construction at zero handshake latency.
- **Increment 3 IMPLEMENTED (2026-07-03, UNVALIDATED — edits-only session, all three knobs
  default-OFF/byte-identical):** the VARIANCE fix proper. Rig campaign data (12 connects at
  dial MPG@20) showed the increment-2 estimator still yields per-connect readings 3.9-17.9
  (σ 3.15) — one fade sample per pick; W3's lone 3.9 trough reading bought a ~90 bps DBPSK
  session (~20× mis-pick). Landed: `ConnectSnrPool` (pure-header ring, cap 8, population =
  data-aided MCDPSK + tagged OFDM_BROADBAND; decorrelation-clustered dB-mean, Tc from the
  trough-pacing derivation chain) behind `ULTRA_CONNECT_SNR_POOL` (entry pick + CONNECT_ACK
  byte + window-16/file-block gates); `ULTRA_CONNECT_PICK_DEFER` (N_eff==1 fading sub-OFDM
  pick withholds CONNECT_ACK once → the initiator's CONNECT retry supplies a decorrelated
  second reading); `ULTRA_WIRE_SNR_FRESH` (MODE_CHANGE embeds gate at 3·Tc, else the −10
  sentinel = wire byte 0 = the receiver's existing "n/a" rendering — fixes the W2 stale-wire
  signature: 3.2 dB shipped 31 s stale, 16.5/22.0 frozen 40-300 s). Companion knob-free GUI
  fix: `MODE_CHANGE:` line labels `wire_peer` vs `local_measured` (responder connect line was
  a mislabeled LOCAL reading). Composition unchanged: +5 basis and the Moderate saturation
  bound apply ONCE, downstream of the aggregation (`test_connect_snr_pool_*` gates this).
- **Increment 4 IMPLEMENTED (2026-07-03, BUG-CONNECT-FADING-VARIANCE — the FADING side of
  the same disease; rides `ULTRA_CONNECT_SNR_POOL`, no new knob):** the entry pick classified
  the channel from a SINGLE CONNECT frame's `fading_index` while the SNR beside it was pooled.
  Screenshot bug (dial-20 Good): one 0.66 reading → Moderate (boundary 0.65) → QPSK R1/4 on an
  R2/3 channel. Rig ledger (48 dial-MPG@20 entries, docs/CONNECT_ENTRY_CALIBRATION_2026_07_03.md):
  single-frame fading 0.24-0.74 (σ 0.129), **false-Moderate 18.8%** (8/9 of those entered R1/4)
  → projected 7.8%/4.1% at N_eff=2/3. Landed: fading rides each pooled reading
  (`ConnectSnrReading.fading_index`, fed from `setChannelQuality`), `clusteredFadingIndex`
  (same Tc clusters, mean of cluster means — bounded statistic, mean not median),
  `rateSelectionFadingIndex()` feeds all entry-pick fading consumers at
  handleConnect/acceptCall/negotiateMode incl. the defer predicate and the CONNECT_ACK
  fading byte; non-entry `fading_index_` uses untouched. ctest green
  (`test_connect_fading_pool_aggregate`), build clean; same rig validation gate as inc 3.
- **Remaining (do NOT close yet):** (1) knob-off byte-identical gui_qso gate,
  then the knobs-ON 5-cell sim gate and low-SNR safety cells (good@8, MPM@8); (2) the rig
  MPG@20 ≥10-connect bench — pass criteria: 0 sub-OFDM entries, effective spread ≤ ~6 dB,
  per-connect σ ≤ 2.2 (√2 tightening), **0 false-Moderate entries at Good**, W2 staleness
  signature absent, ≤1 extra CONNECT cycle on deferred picks; (3) re-evaluate the +5
  `connectSelectionSnrDb` basis after the rig re-measure (the 48-entry ledger says mean
  offset −7.6 dB, SNR-dependent — deliberately untouched in increments 2, 3 AND 4).

### BUG-QAM16-RIG-LEVEL-BUDGET — CLOSED 2026-07-02 (final): there is NO hidden level deficit anywhere on the bench; the IONOS dial is the only SNR lever. 16QAM at MPG@20 is trough-limited physics; it opens at dial ~22+ (matches the sim crossover)
- **Every gain lever measured, all dead:** (1) TX drive walk +4.6 dB digital (software-ALC, live)
  arrived as −0.8 dB at the Mac — the IONOS normalizes its input; (2) Mac input volume 60→90:
  the noise FLOOR scaled +9.0 dB with the gain — the floor is the IONOS's own calibrated output
  noise, not ADC self-noise, so RX gain moves signal and noise together. The bench delivers
  exactly what the dial says. The earlier "raise CH-OUT / +4-5 dB level" advice is RETRACTED —
  measurably wrong. The ladder's QPSK R2/3 pick at MPG@20 is the bench's true optimum (~1.7 kbps).
- **To test 16QAM on this bench: turn the SNR dial** to MPG@22-24 (sim crossover data: Good@22 =
  2710/2150/2050 for 16QAM R2/3). The software-ALC remains correct and valuable for REAL radios
  (no box normalizes your level there); it is proven mechanically end-to-end on the rig.
- **FIX IN VALIDATION (2026-07-02, software-ALC — CHANGELOG entry of same date):** the "level
  lever" is now CLOSED-LOOP instead of operator-manual. Receiver measures per-burst data-RMS
  over chain noise + crest factor (`[ALC-RX]` / `LEVEL ADVISORY:` log lines, thresholds
  `ULTRA_ALC_LOW_DB`=12 / `ULTRA_ALC_CLIP_CF_DB`=6.5); a 2-bit drive advisory rides the
  tone-burst ACK (bits [30..31], **WIRE-BREAKING — lockstep builds only**); sender walks
  tx_drive +0.5 dB/−2 dB within [baseline, 0.85] on connected OFDM data bursts only
  (`ULTRA_SOFTWARE_ALC=0` disables the loop). Sim-proven no-op at reference levels (gate PASS,
  zero `ALC:` moves). **PENDING: rig A/B** — expect the loop to walk 0.5→~0.85 (+4.6 dB) at
  MPG@20, lifting arriving data SNR from ~6-7 dB toward the ~11-12 dB the 16QAM rungs need;
  then re-run the 16QAM ladder. Grep `ALC: tx_drive` (sender) + `\[ALC-RX\]` (receiver).
- Status: **DIAGNOSED via wire capture + rung falsification (overnight 07-02).** The Mac-input
  ffmpeg captures (paired QPSK vs 16QAM forced runs, MPG@20) show IDENTICAL level structure for
  both mods: data segments ~0.077-0.079 RMS, anchors ~0.16-0.18, noise floor ~0.037 → the data
  arrives at only **~6-7 dB broadband wire SNR**, while anchors ride 6-7 dB hotter (per-burst
  PEAK normalization: OFDM crest ~14.3 dB eats average power). No 16QAM-specific TX defect.
- **Rung falsification:** 16QAM R1/2 (4-5 dB more margin than R2/3, sim-clean 5/5 at Good@18)
  ALSO fails to complete on the rig — decode is **bimodal** (13 groups 8/8 flawless, 13 groups
  0/8 dead at median 22.7 dB effective): the fade TROUGHS at this wire level kill any dense
  constellation whole-group; the crests pass 16QAM perfectly. QPSK's phase-only margins bridge
  the troughs. 16QAM transfers PROGRESS (~26 KB of clean groups in 480 s) but can't finish at
  ~50% group loss.
- **Dead end tested + reverted same night:** coherent-OFDM PAPR soft-clip (recover average power
  under peak normalization). Sim A/B: EVM cost >> benefit for 16QAM at every depth (9 dB target:
  181 vs 30 deint-fails, hard FAIL; 12 dB: 78 fails, 2210->1320). `ULTRA_COHERENT_PAPR_DB`
  ships default-0 (off) as an experiment knob only.
- **The fix is a LEVEL lever, not code:** ~+4-5 dB of arriving data SNR moves the troughs above
  16QAM R1/2's floor (tx_drive 0.5->~0.8 and/or IONOS CH-gain re-staging — needs the operator at
  the IONOS CF/level panel per the 2026-06-15 calibration method; peaks must stay under the
  1800 mVpp input clip). Alternates if level can't move: cw16 (raises per-bit efficiency, same
  trough problem), fade-phase-aware scheduling (research). The prior ANCHOR-COLLAPSE observation
  (07-01 afternoon: corr 0.95->0.2) did NOT reproduce in any of 6 subsequent 16QAM runs — kept
  below as historical until seen again.

### BUG-QAM16-RIG-ANCHOR-COLLAPSE: 16QAM bursts stop being ACQUIRED on the real rig (sync corr 0.95 → 0.2) while their PHY decodes clean — root cause NOT isolated
- Status: **OPEN — observed 2026-07-01 on IONOS MPG@20 (Mac↔Pi5, HEAD a81725d), forced 16QAM R2/3.** The transfer decoded 9+ groups cleanly (**77/77 deinterleave SUCCESS, 0 FAILED — zero fade-damage tax**), then fell into a persistent no-delivery saga: 117 nack + 74 timeout retx, no completion in 480 s. Sender escalates full-anchor resends; receiver pinned in `Full-anchor wait rejected DATA fallback (corr=0.34 < 0.50)`.
- **The discriminating signature:** same channel, minutes apart — QPSK R3/4 run sync-corr modes 0.92-0.99 (delivered 1.62 kbps clean); 16QAM run corr modes **0.16-0.29**. The constellation doesn't fail; the *acquisition* of its bursts does. Burst-erasure gate hits: 0.
- **Candidate mechanisms (untested):** (a) TX-side PAPR — 16QAM+cross-frame-interleave raises burst crest factor → per-burst hardware normalization lowers average power → the in-burst anchor chirp sinks toward the RX noise floor (the 2026-06-16 IONOS "high-PAPR data below the anchor" mechanism; `ULTRA_SIM_PAPR_PENALTY` exists because OTASim cannot exhibit this); (b) RX-side — decode/search backlog during the saga (#56 class) + the full-anchor-wait rejection threshold locking the pair into mutual starvation.
- **Discriminating experiment:** paired QPSK-vs-16QAM rig runs logging the Pi5 per-burst normalization factor (AUDIO category) and the Mac chirp-segment RMS/corr per burst; if 16QAM's normalization factor is materially lower, it's (a) — mitigations: anchor power boost within the normalization budget, PAPR reduction (clip/tone-reserve) on the data symbols, or descriptor-profile robustness. If not, instrument the full-anchor-wait state residency.
- **Impact:** blocks the entire 16QAM-on-hardware path (the only route to 3000 bps) regardless of the sim-side damage work. Sim CANNOT reproduce (fidelity gap — document per SIMULATOR FIDELITY rules). See `fable_analysis/09_WHY_STUCK_AT_2000_2026_07_01.md` §4.

### BUG-ACK-TIMEOUT-DOUBLECOUNT: unified burst ACK deadline counted burst airtime ~twice (+ phantom reanchor term) → every timeout saga paid ~8-10 s extra dead air — **CLOSED IN CODE by the 2026-07-02 RTO re-derivation (register reconciled 2026-07-03)**
- Status: **CLOSED in code — register was stale.** The 2026-07-02 `unifiedBurstAckTimeoutMs`
  re-derivation (`connection_policy.hpp` ~1002-1053, comment: "RE-DERIVED 2026-07-02 (closes
  BUG-ACK-TIMEOUT-DOUBLECOUNT)") replaced the double-counting `physical_sack_hold_ms =
  max(configured window-hold, burstAirtime+30)` term with a receiver-response envelope derived
  from the SAME formula family as the receiver's group-timeout fast-NACK (rig-calibrated: 124
  groups across 4 MPG@20 transfers measured the clean-path group-end→SACK hold at 0-1 ms);
  `configured_sack_delay_ms` is deliberately no longer consumed on the burst path. This entry
  had stayed OPEN with pre-07-02 line numbers while the code claimed the fix — reconciled while
  landing the retx trough-pacing design (its §6.4), which deliberately adds ZERO deferral on
  the RTO path at Good so nothing in pacing depends on the RTO's exact length (a later RTO
  tightening makes pacing MORE valuable, not less).
- **Floor rule still binds (do not cut below):** the deadline back-stops the receiver's
  wall-clock group-timeout fast-NACK (`streaming_decoder.hpp:877` `BURST_TIMEOUT_MS_BASE` —
  its 8000 ms wall-clock floor is itself a tracked adaptivity item, NOT changed by pacing)
  and rig worst-case turnaround (~5.8 s post-burst latency; #56 RXQ backlog). The 2026-06-19
  premature-resend incident is the regression class to avoid. **Move both timers together.**

### BUG-MCDPSK-ACK-COLLISION: tone-burst partial-SACK fires < one MC-DPSK frame airtime → half-duplex collision livelock — **FIX IMPLEMENTED (2026-06-30), pending lossy-rig validation**
- Status: **ROOT-CAUSED + REPRODUCED on the live IONOS rig + FIX IMPLEMENTED + ctest-clean (OFDM byte-identical). Pending: lossy-channel rig A/B (the faithful gate runs clean → no holes → can't exercise it; also confounded by BUG-MCDPSK-FILE-COMPLETION which blocks *completion* regardless).** Surfaced once #74 let MC-DPSK connect + transfer at low SNR (rig MPG@8, DQPSK R1/4, window=5).
- **Fix (implemented):** the real mechanism is the tone-burst PARTIAL (hole-bearing) SACK sliding timer (`selective_repeat_arq.cpp` ~622), hardcoded to `kToneBurstPartialSackDelayMs = 1500 ms` — correct for OFDM (short frames) but < one MC-DPSK frame airtime (3691 ms), so it fired while the sender was still transmitting a trailing failed frame. Made it configurable (`setToneBurstPartialSackDelayMs`, default 1500 → OFDM byte-identical) and the Connection scales it to `max(1500, timing.data_ms + 1000)` ≈ 4.7 s for MC-DPSK so the SACK lands in the inter-burst gap and the sender does a FAST retransmit instead of a timeout whole-window resend. Carrier-sense (defer until the channel is heard idle) is the fully radio-correct generalization + would also cover the rare multi-trailing-hole case; this airtime-scaled guard is the targeted fix.
- **What:** on a hole (a frame in the window fails to decode), the transfer livelocks: the SENDER retransmits the whole window on its 31.6 s RTO (`cause=timeout`, NEVER on a NACK), the RECEIVER keeps re-sending the same SACK (`group_seq=35 frame_mask=0x02`), they collide forever → 10/10 retries → DISCONNECT. The clean path (no holes) works fine — groups advance every ~21 s. Only the REPAIR path collides.
- **Trace (rig, MPG@8):** Pi5 (sender) TX bursts at 408.3 / 439.8 / 471.4 s (each 896256 samples ≈ **18.7 s**, period ~31.5 s); Mac (receiver) ACKs at 420.6 / 452.0 / 483.6 s (period ~31.5 s). Every ACK lands ~12 s INSIDE a sender burst → sender mid-TX (deaf) → never hears the NACK → times out → resends → next ACK lands in that burst too. Both ends on the same ~31.5 s period, phase-offset to collide.
- **Root cause:** `selective_repeat_arq.cpp:1747` — `guard_half_duplex_repeat = (bitmap == 0) && !sack_has_final`. The half-duplex peer-burst guard is DELIBERATELY disabled for hole-bearing SACKs (`bitmap != 0`) so repair feedback is "prompt." Correct for OFDM (short bursts → sender stops & listens fast), WRONG for MC-DPSK: the window burst is ~18.7 s so the "prompt" NACK fires straight into the sender's transmission. The RTO was made rate-agnostic (`computeMCDPSKAckTimeoutMs` scales with the 3691 ms frame) but the ACK-repeat guard was NOT — `setAckRepeatPeerBurstGuardMs(arq_.getSackDelay())` = **30 ms**, vs an 18.7 s burst.
- **Fix (next session):** peer-burst-guard the hole-SACK repeat ALSO on the long-burst path — delay it past the sender's window-burst airtime (`connection_policy::mcDpskBurstAirtimeMs ≈ 18.7 s`, already computed) so the NACK lands in the inter-burst gap. Rate-agnostic = guard derived from the actual burst airtime, not the 30 ms sack_delay. The fully radio-correct version is **carrier-sense** (don't key up while the peer is heard); the burst-airtime-scaled guard is the minimal targeted fix. Needs rig A/B on a lossy channel (collision only appears on a hole). Likely interacts with BUG-MCDPSK-FILE-COMPLETION (a persistent hole that never repairs).

### BUG-MCDPSK-FILE-COMPLETION: MC-DPSK file never finalizes — sender ACK RTO under-budgets the RTT → FINAL chunk never reached — **FIXED + GATE-PROVEN + HW-PROVEN on IONOS (2026-06-30)**
- Status: **ROOT-CAUSED (workflow + adversarial verification) + FIXED + ctest-clean + FAITHFUL-GATE PROVEN + LIVE-IONOS PROVEN.** gui_qso good@7 1KB MC-DPSK (ULTRA_ROBUST_IDLE_PING=1 to clear the handshake floor, DBPSK R1/4): **FILE_CRC_OK_COUNT=2, ALPHA_FILE_DONE_COUNT=1, RESULT=PASS** (was FILE_CRC_OK=0 / resend-forever / FAIL). GOODPUT=10 bps (~11 min/1KB — the #71 SPEED lever, not this bug). Pre-existing (#73); surfaced once the handshake fixes let MC-DPSK connect + the ACK-collision fix (33ccade) widened the receiver hold.
- **HW proof (2026-06-30, IONOS MPG@8 Good, Pi5→Mac, commit 9579a1a both ends, ULTRA_ROBUST_IDLE_PING=1 + ULTRA_CONNECT_RATIOMETRIC_SNR=1):** ladder correctly dropped to **MC-DPSK DBPSK R1/4** (mcdpsk_in_band effective SNR 0.9–5.1 dB read < 10 → not OFDM), **ARQ window=3 timeout=43.6 s** (the fixed RTO; old ~30 s < 37.9 s RTT), both tone-burst ACKs `bitmap=0x0` (**0 retx, 0 holes**), sender `[FILE] Transfer complete (37.4 s)`, receiver `Received OK (568 bytes, CRC ok)` → **md5 65f0ced2…b0de byte-identical**. The old build blind-resent forever here; now it finalizes cleanly. Goodput 0.12–0.20 kbps = #71.
- **NOT an assembly/flag/routing bug** (the earlier hypothesis): the receiver path is byte-correct and identical to OFDM (every delivered seq → "Processed file transfer payload"; FINAL/MORE_FRAG plumbing sound; file chunks are FrameType::DATA → `file_transfer_.processPayload`, NOT the binary DATA_START/CONT/END path). The real cause is a **sender-side ACK-RTO defect**: `computeMCDPSKAckTimeoutMs` was passed `arq_.getSackDelay()=30 ms` instead of the **6376 ms receiver tone-burst SACK hold** (that 33ccade added), AND omitted the **~16 s receiver serial-decode latency** (#56). So the RTO (30.1 s) < the measured half-duplex RTT (**37.9 s**) → the sender blind-resends the whole window before the ACK lands (all 21 retx `cause=timeout`) → doubled airtime → the FINAL chunk (~seq 32 of 33) is never reached in a bounded session → `checkAndFinalizeReceive` never fires → `file-recv=0`. This bug and BUG-MCDPSK-ACK-COLLISION are the SAME bug (33ccade widened the hold without updating the sender deadline).
- **Fix (CHANGELOG 2026-06-30):** `computeMCDPSKAckTimeoutMs` now budgets the full physical RTT (`2·tx_burst + receiver_sack_hold + ack + turnaround` ≈ 43.5 s > 37.9 s), lower clamp lifted to the physical floor; Connection computes the hold ONCE and feeds both the receiver setter and the sender RTO. Completion now reliable in an unbounded session.
- **Remaining (separate lever, NOT this bug):** SPEED. RTT ~38 s × ~11 windows ≈ **~7 min for 1 KB** (the #56 decode latency + 32-byte chunks / window=3 = ~33 seqs). Practical MC-DPSK files need #71 (DQPSK rung / bigger chunk / window). Repro: `/tmp/v72_nat7`; completion proof: `/tmp/v73_complete`.

### BUG-MCDPSK-CONTROL-BAUD: CONNECT_ACK shipped at the data rung's mod/baud → handshake strand on sps≠1024 rungs — **FIXED (#72, 2026-06-29, CHANGELOG)**
- Standardized MC-DPSK on sps=1024 + routed handshake-negotiation frames through the DBPSK control profile. Forced rungs now CONNECT; no regression. See CHANGELOG.

### BUG-FILE-ACK-IDENTITY: ARQ send-complete dispatch is identity-blind — a non-file frame retiring while a file is SENDING pops a file-chunk ledger entry / inflates chunks_acked_
- **Status: CONTAINED by strict logical-operation serialization (2026-08-04); ideal ARQ
  origin tagging remains deferred.** Found 2026-07-02 by adversarial review of the
  requeue-ledger fix; pre-existing in kind (the same blindness inflated the old count
  arithmetic). `Connection::setSendCompleteCallback` still dispatches a successful DATA
  retirement by current `FileTransferState`, not by the retired slot's application origin.
  If a non-file slot could retire while the controller was `SENDING`, it could still pop the
  file ledger, resume a later requeue one chunk forward, and over-count completion.
- **Public trigger closed:** message, binary, and file requests now share one enqueue order and
  one logical application operation owns ARQ retirement at a time. A queued file starts only
  after older payloads retire and the real local DATA turn is held. The non-interactive burst
  bypass starts a file only at a genuinely empty logical boundary; it no longer skips live or
  queued message/binary work. A newer payload cannot bypass queued survivors after a failure,
  and queue overflow refuses the newest operation instead of evicting accepted work. GUI
  messaging is restored, but its manual and automated paths consume this same serialization.
- **Why this is containment, not closure:** the callback's type contract is unchanged. Future
  work must not permit mixed logical operations in one ARQ window until each TX slot carries an
  origin such as `FILE_CHUNK` / `MESSAGE_FRAGMENT` / `BINARY_FRAGMENT` and completion reports
  that origin. The longer-term structural fix is still `on_send_complete_(success, origin)`
  dispatch rather than state inference. The current one-operation boundary deliberately trades
  cross-class pipelining for integrity and half-duplex predictability.
- **Regression evidence:** focused `ConnectionAdaptive` cases require binary-before-file FIFO,
  queued files to wait for the actual DATA turn, queue-survivor order after a head failure, and
  exactly one terminal result per accepted message. See the 2026-08-04 CHANGELOG entry for the
  complete message/turn/geometry verification boundary.

### BUG-HANDSHAKE-PING-FLOOR: low-SNR PING/CONNECT classifier starves → handshake never connects below ~15 dB Good (caps ALL operation) — **FIXED, DEFAULT-ON (2026-07-01) — RE-OPENED for the mid-SNR SIM window (see re-open note)**
- **RE-OPEN NOTE (2026-07-01 evening, found by the #58 Good@12 boundary probe):** at SIM
  reference levels good@12 the PING's noise-flooded gap reads data_rms **0.23** — ABOVE the
  a81725d high-SNR-churn gate `kPingChirpLockMaxDataRMS=0.16` — so the robust chirp-lock emit
  is SUPPRESSED (chirp corr 0.66-0.75 solid, ratio 0.78 "data-bearing", path1=0 path2=0, no
  PONG, 5/5 PING timeouts, 2/2 seeds). This is the documented "0.16 is absolute → level-fragile"
  caveat biting: the gate separates rig PING gaps (0.04-0.09) from good@20 false-syncs
  (0.28-0.33), but the good@10-15 SIM window falls in between. The a81725d "PROVEN good@10/12"
  claim predates the 0.16 gate and no longer holds. Rig low-SNR (MPM@8 etc.) unaffected. Fix as
  already noted: noise-floor-RELATIVE emit gate. Until then, sim work at good@10-15 cannot
  connect out-of-box.
- **ctest reproducer (2026-07-02):** `UltraTncSimAudio` (OTASim lobby awgn@15) now FAILS on
  this window deterministically — PING gap reads data_rms 0.1653 (> the 0.16 emit gate),
  ratio 0.547 (≥ 0.5 "data-bearing"), robust emit suppressed → no PONG → "timed out waiting
  for command line". Verified pre-existing on main (fails at main HEAD with a clean tree,
  2026-07-02); NOT a live-ladder or connect-policy regression. awgn@15 sits in the same
  in-between band as good@10-15. The red ctest is THIS bug; fix = the noise-floor-relative
  emit gate above. **FIXED 2026-07-07 by STAGE 1.5 (below): PASS ×3, handshake 16 s (was
  never-completes). NOTE: it can still fail under `ctest -j4` when a GUI sim runs
  CONCURRENTLY on the same machine (real-time CPU starvation, harness artifact).**
- **STAGE 1.5 (2026-07-07, LANDED — the mid-SNR window CLOSED):** the noise-floor-RELATIVE
  emit gate the entries above called for. `gap_is_noise` = IN-BAND gap RMS (same 101-tap FIR
  as `IdleNoiseSNREstimator`) ≤ the receiver's own measured idle floor × 1.5 — level-invariant
  (#74-safe) and high-SNR-safe by construction (a real payload rides sqrt(1+SNR_lin) above the
  floor; the good@20 false-sync class reads ~10×). TWO WRONG CUTS documented for posterity:
  (a) comparing the RAW-domain gap vs the in-band floor never fires (broadband ambient reads
  ~2.9× the in-band floor); (b) comparing raw-vs-raw fires but COMPRESSES the payload
  discriminant to sqrt(1+SNR·B_band/B_tot) ≈ 1.49× at good@10 → misclassified a real CONNECT
  as a PING (caught in sim). In-band-vs-in-band is the only correct domain. Also: the CW0-peek
  wait gate honors gap_is_noise (no more multi-second 4-CW waits per flooded probe), and
  sync-found feeds the estimator the pre-chirp ambient prefix (SyncResult.preamble_start_sample,
  boot-only) so a first-pass responder has a floor reference. This RETIRES the never-implemented
  STAGE2 `bare_chirp_expected_` plan as the gate for the robust emit (the noise-relative test
  subsumes it). PROVEN: sim good@10 out-of-box CONNECT+PASS (QPSK R1/4, CRC ×2), good@20 PASS
  2070 bps 0 spurious, UltraTncSimAudio PASS ×3.
- Status: **FIXED + DEFAULT-ON (rig path); mid-SNR sim window re-opened by the emit gate.** `ULTRA_ROBUST_IDLE_PING` promoted DEFAULT-ON (opt-out `=0`) after all three sub-issues were closed: (1) initiator #27 (bare_chirp_expected_=FALSE during CONNECTING), (2) responder starvation (STAGE2 expects-CONNECT window), (3) high-SNR churn (data_rms≤0.16 emit gate). VERIFIED: good@20 sim PASS with the emit active (2/2 PING/PONG, 1860 bps); rig **MPM@8 pure-default (zero env vars) connects + delivers CRC-clean** (2/2). With #74 (ratiometric) also default-on, the low-SNR path is now the shipped default on a real radio. See CHANGELOG 2026-07-01.
- **What:** a PING is a bare chirp with no data (`encodePing`→`generatePreamble`). The receiver tells a PING from a CONNECT with a LEVEL test (`data/training RMS ratio < 0.5`, abs floor 0.16). At low SNR broadband noise floods the PING's silent gap (ratio 0.68–0.88 > 0.5) → real PING reads as a faded CONNECT → waits for a 4-CW frame that never comes → no PONG → never connects. The chirp itself locks solid (corr 0.6–0.75). Floor map (faithful gate): never connects awgn@6/8 good@8/10/12; marginal good@15; reliable good@20. The published 5 dB AWGN *data* floor never caught this (`measure_ack_fer` skips the live handshake).
- **Fix (env-gated):** when the knob is ON, a solid chirp-lock with a low-LLR (false-lock-rejected) frame emits the PING on chirp signature alone; gated on `bare_chirp_expected_` (FALSE during CONNECTING) so a faded CONNECT_ACK isn't mis-PONGed (#27-safe on the initiator). PROVEN: good@10/12 never-connect→PASS, good@20 no-regress.
- **STAGE2 (2026-07-01, LANDED — responder hole CLOSED):** on PONG-TX the responder now sets `bare_chirp_expected_`=FALSE + arms a ~20 s expects-CONNECT window (app.cpp), mirroring the initiator's PROBING→CONNECTING disarm, then the tick re-arms. Sound for any window: while CONNECTING the initiator re-SENDS CONNECT (never re-PINGs, connection.cpp:2421-2442) and a stray PONG to a CONNECTING initiator is a no-op + half-duplex-inaudible, so worst case is occasional harmless waste, never permanent starvation.
- **Remaining before default-ON (RESOLVED 2026-07-07 by STAGE 1.5 — kept for history):** flipping `ULTRA_ROBUST_IDLE_PING` default-ON REGRESSES the faithful sim gate at good@20 — the near-silent HIGH-SNR PONG's residual pushes the data/train ratio just above 0.5 (data_bearing) with low LLR + a real chirp, so the robust emit fires SPURIOUSLY → PING/PONG churn (11/9) → ~30 s connect → gate overrun. Isolation-proven: same build with `=0` → good@20 PASS (1860 bps); STAGE2 alone is clean. FIX NEEDED: a high-SNR-safe emit gate (e.g. a higher data_bearing floor separating a noise-FLOODED low-SNR PING (ratio 0.68–0.88) from high-SNR PONG residual (~0.5–0.6)). Also: throughput below ~8 dB is a separate lever (#71), not this bug.

### BUG-IONOS-PI5-CHEAP-DAC: Pi5→Mac handshake one-way — cheap-card carrier JITTER (root cause REFINED 2026-06-15) — partial software mitigation landed
> **⚠ SUPERSEDED IN PART — 2026-07-26. The card described below is NO LONGER IN THE PATH and its
> measured figures MUST NOT be cited as current.** The Pi5 now runs an **Fe-Pi Audio / sgtl5000**
> codec HAT (confirmed via `aplay -l` and `wpctl`). That path was measured directly with
> `tools/measure_analog_path.py` (16-tone multitone, Pi5 → IONOS WGN@40 → Mac, at the operational
> RX level — capture RMS 0.0924 vs 0.082 in real transfers):
>
> | metric | this entry's cheap USB dongle | **Fe-Pi, measured 2026-07-26** |
> |---|---|---|
> | band tilt | 14.8 dB | **0.7 dB** |
> | analog SNDR ceiling | (implied by −17.8 dB distortion) | **30.7 dB** |
> | cost at dial 20 | — | **0.36 dB** |
>
> A 30.7 dB ceiling costs 0.36 dB at a 20 dB dial, so **the analog chain is clean and cannot
> explain the ~5.3 dB sim-vs-rig SNR discrepancy** that was briefly (and wrongly) attributed to
> it. See CHANGELOG 2026-07-26. The BURST_ERASURE level-relative fix below remains valid on its
> own merits — a level-relative gate is correct regardless of which card is attached — but the
> "~6× lower RX operating level" premise should be re-measured before being relied on again.
> What is still genuinely open from this entry: nothing hardware-side has been re-verified for
> JITTER; only tilt, distortion and SNDR were measured.

- Status: **OPEN (hardware), root cause REFINED + software mitigation landed 2026-06-15.** First
  Mac↔IONOS↔Pi5 bringup. After the CONNECT-decode code fix (CHANGELOG 2026-06-14), Mac→Pi5 completes
  (CONNECT decodes 4/4, Pi5 CONNECTED) but Pi5→Mac CONNECT_ACK never decodes → initiator never
  CONNECTED → no file transfer.
- **UPDATE 2026-06-16 — when the handshake DOES complete (Mac→Pi5, MPG), the FILE TRANSFER stalls,
  and the proximate mechanism is now isolated from the real receiver log (`/tmp/pi5_full.log`):**
  the chirp anchor decodes but burst data frames 2–6 are ERASED every group at broadband RMS
  0.0038–0.0145 (all under the absolute `BURST_ERASURE_RMS_THRESHOLD=0.015`) → all-zero-LLR group →
  `header invalid` → ARQ retransmit → stall. The 0.015 gate was implicitly "~25 dB below the SIM
  anchor (0.27)"; at the cheap card's ~6× lower RX operating level it became only ~5 dB below the
  anchor and erased recoverable frames. **FIX A landed (UNCOMMITTED, see CHANGELOG 2026-06-16):** the
  gate is now operating-level-RELATIVE (`max(0.055*anchor, 0.005)`), zero sim regression, keeps 16/20
  of the erased frames on IONOS. **Hardware note (user, 2026-06-16): the MPG logs are the CLEAN Fe-Pi
  HAT, not the old USB card** — so the deficit is codec-INDEPENDENT (high-PAPR data below the anchor +
  lower RX operating level + Good fade nulls), and on a clean codec the KEPT frames should DECODE, so
  FIX A is likely the actual fix (the cheap-card per-carrier-LLR / `CHEAP_CARD_ROBUSTNESS_PLAN.md` work
  applies only to the old USB card). Re-test Mac→Pi5 MPG with `ULTRA_BURST_RMS_DIAG=1` to confirm the
  anchor/level + that frames are KEPT and decode; if marginal, raise RX gain and/or use R2/3 (the
  morning run negotiated the fragile R3/4). Decisive proof is rig-only.
- **UPDATE 2026-06-15 — the "bad clock, swap the card" diagnosis was RE-EXAMINED and largely REFUTED;
  the real lever is carrier JITTER, and a software mitigation now recovers the slow-jitter regime.**
  In-sim impairment discrimination through the real encode→AWGN→decode path (`test_mcdpsk_clock_offset`,
  CHANGELOG 2026-06-15) showed: (1) sample-CLOCK offset is NOT the bottleneck — R1/4 LDPC decodes CRC-
  clean to ±1000 ppm with or without correction, and the prior "−1800..−3000 ppm wandering" figure was a
  measurement artifact (a crystal cannot wander ~1200 ppm run-to-run; the values match an integer ALSA
  buffer-period drop = USB starvation on the headless Pi); (2) band TILT is NOT the bottleneck (frequency
  diversity + LDPC ride through 20 dB); (3) the cheap DAC's ±7 Hz carrier JITTER IS the bottleneck (it
  exceeds the DQPSK ±45°/symbol decision margin → payload garbage, chirp survives). The MC-DPSK demod now
  has clock-offset + carrier-jitter tracking (default on) that recovers SLOW jitter and is a proven no-op
  / no-harm otherwise. So this is no longer purely an equipment fault — re-test with the current build
  before concluding a card swap is needed.
- **LIVE IONOS BRINGUP 2026-06-15 — DEFINITIVE ISOLATION (the re-test above, run on real hardware).**
  Worked the full Mac↔IONOS↔Pi5 path knob-by-knob and eliminated everything except the card:
  - **Wiring:** one direction (Pi5→Mac) was dead (Mac RX rms 0.0004 = digital silence) until a CABLE
    SWAP — then both directions carried signal. (So part of the original failure was a bad cable, not
    the modem at all.)
  - **IONOS input CLIPPING (the big one):** the Pi5's transmit was overdriving the IONOS input — the
    panel read `Lvl=2000 mvp-p` RED with `CF=1.01 [0.05 dB] (PEP/Pavg)` = the multi-carrier signal
    arriving as a HARD-CLIPPED SQUARE WAVE (a clean MC-DPSK signal is CF≈10 dB). That clipping, not the
    DAC, was garbling the CONNECT_ACK at high level. Gain staging: the IONOS CH-IN gain (factory default
    1; had been run at 5–20) and the **Pi5 ALSA `Speaker` volume are the real level knobs** — see tx_drive
    note below. Nominal IONOS input max is 1800 mvp-p; the modem's ~10–12 dB crest factor means target the
    *average* well under that (peaks clip first).
  - **CONNECT_ACK→PING misclassification (software, FIXED):** once clipping was removed, the Mac decoded
    the low-level CONNECT_ACK to garbage AND then mis-classified it as a PING via the
    `pre_ldpc_llr_reject` path (ldpc_attempted=false short-circuits `ping_by_chirp_lock`), firing BEFORE
    the 06-14 ratiometric 4-CW wait gate. FIXED (CHANGELOG 2026-06-15): a ratiometric guard lets a
    data-bearing (ratio≈1) MC-DPSK frame fall through to the full 4-CW decode instead of being pinged.
    HW-confirmed the Mac now attempts the 4-CW decode (`got 2592 soft bits`); PingDetector + 16/16
    regression still pass.
  - **Level matched, decode STILL fails → THE CARD.** With wiring, clipping, and classification all
    fixed, swept the Mac's RX: at 0.08 (low), **0.16 (matched to the Pi5's working 0.17), and 0.20**, the
    4-CW CONNECT_ACK decodes to GARBAGE every time — while the SoundBlaster→Pi5 CONNECT decodes fine at
    0.17. Same level, clean, green, no clipping; only the transmitting card differs. **Definitive: the
    cheap Pi5 USB dongle's TX is the limit** (its measured tilt+distortion+jitter), beyond what the
    clock/slow-jitter trackers recover.
  - **tx_drive is NOT a software bug (correction of an in-flight claim).** On the Pi5, changing tx_drive
    (0.10/0.20/0.70) barely moved the level — but the AUDIO-category log shows the per-burst hardware
    normalization is APPLIED, not bypassed (no `normalization bypassed fragment` warning), and tx_drive
    DID work on the Mac (0.7 clipped to 2000). So the software is correct; the Pi5 cheap card's analog
    output just doesn't respond linearly to the digital level (saturation/compression — same hardware
    trait as the decode distortion).
  - **VARA paradox / forward path:** the user runs this exact dongle with a commercial HF modem, so the
    card is usable by a *robust-enough* handshake. Our 4-CW DQPSK R1/4 CONNECT_ACK simply isn't as
    robust. Two paths: (a) swap to the SGTL5000 I2S HAT (on hand) for an immediate working QSO;
    (b) earn cheap-card tolerance in software — DBPSK control frames (double the phase margin vs jitter)
    + per-carrier SNR weighting (handle tilt) + more control-frame FEC. See docs/CHEAP_CARD_ROBUSTNESS_PLAN.
- **Root cause (tone-test measured, /tmp/measure_pi5_tx.sh + measure_mac_tx.sh + analyze_tone.py):**
  the Pi5's cheap "USB Audio Device" **DAC/playback** is the problem. Same IONOS, same two cards,
  TX/RX roles swapped — only the transmitting card differs:
  | metric | Pi5 cheap card TX (fails) | Mac SoundBlaster TX (clean) |
  | band flatness across 0.5-2.6 kHz | **14.8 dB tilt** | 5.7 dB |
  | 2nd-harmonic distortion | **-17.8 dB** | -36.7 dB |
  | freq accuracy (jitter) | **±7 Hz** | ±0.5 Hz |
  The cheap DAC's 15 dB band tilt starves the edge MC-DPSK carriers + the distortion/jitter smear the
  DPSK phase → the multi-carrier 4-CW payload fails to decode, while the robust band-sweeping chirp
  shrugs it off (CFO≈0, chirp decodes) — hence PING works, CONNECT payload doesn't. The cheap card's
  **ADC/record is fine** (reverse dir clean), so it's specifically the TX path.
- Of the three measured TX impairments, the discrimination (above) shows tilt is absorbed by diversity
  and a stable clock offset is absorbed by LDPC; the **jitter** is what crosses the DQPSK margin. The
  measurement was also taken on the Pi5 host, so part of the apparent jitter/tilt may be Pi-side USB
  starvation rather than the DAC itself (re-measure driving the cheap card from an unloaded host to
  separate DAC-intrinsic defects from host starvation — recommended experiment).
- **Fix order (revised 2026-06-15):** (1) re-test Pi5→Mac with the current build — the new MC-DPSK
  clock+jitter tracking may already close it if the real jitter is slow; (2) if it still fails, capture
  a Pi5-TX CONNECT recording and decode it offline to confirm jitter rate vs an ALSA-drop discontinuity;
  (3) swap the Pi5's cheap USB soundcard as a BACKSTOP only if the residual jitter is fast/large enough
  to be below the trackable regime. Earlier "balance the L/R levels" framing was wrong/incomplete. HW
  knobs the bringup harness sets: Pi5 `callsign=PI5`, `tx_drive=0.7`, ALSA Speaker −3 dBFS; Mac
  `tx_drive=0.7`, input vol 60. Run the gui_qso gate LID-OPEN on an unloaded machine (lid/throttle →
  ALPHA audio starvation `SKIP unsearched < min` → false FAIL).

### BUG-TNC-B2F-002: post-burst non-burst FF frame not delivered (blocks bulk-accum-to-burst B2F) — LAYERED
- Status: **FIXED 2026-06-05.** ROOT CAUSE (LAYER 2, finally isolated): the ENCODER did not revert
  its LDPC lifting-z after a burst. A burst lifts the encoder to z=81 (`setLDPCLiftingZ(81)` in
  transmitBurst); nothing reverted it, so the next non-burst frame on the `transmit()` path (the FF
  terminator, a chat line, an SR-ARQ repair) was still encoded at **z=81 (~106 880 samples)** while
  the receiver — which DOES revert at group-end — decoded it as **z=27 (~17 920 samples)**. BRAVO
  read the first ~17 % of a z=81 frame as a z=27 frame → saturated-magnitude/random-sign LLRs
  (`|llr|=20`, `llr_avg≈0`) → LDPC 0/CW → never ACKed → stall. Fix: revert the encoder to z=27 for
  every non-burst frame (`modem_engine.cpp` transmit() dispatch). Plus the TNC accumulation was made
  spec-correct (honest VARA `BUFFER` — see CHANGELOG 2026-06-05) so the body no longer striped across
  burst+interactive transports (the out-of-order corruption). 20 KB JPEG now delivers byte-identical
  with a CLEAN teardown on the plain (no-bulk) path, GUI-verified. Layers, peeled in order:
  - LAYER 0 (sync acquisition): FIXED 2026-06-04 (catch-up drain + full-anchor buffer, CHANGELOG).
  - LAYER 1 (§14.24 gate drop): **FIXED 2026-06-05** — `have_burst_descriptor_` now drops at
    group-end (`finalizeBurstGroup`, streaming_burst_interleave.cpp) instead of persisting the whole
    connection, so the trailing FF is no longer gated as mid-burst. The FF now REACHES the decoder
    (`Chirp detected … escalating to N CWs`). Verified no multi-group regression: gui_bidir AWGN@20
    7/7 each way, 0 CWfail.
  - LAYER 2 (post-burst mode-revert — **REMAINING**): the FF demodulates to garbage — saturated
    magnitude, random sign (`CW FAIL, llr_avg≈0.3, |llr|=20`). RULED OUT as root (traced 2026-06-05):
    NOT a buffer/coordinate bug — `process()` calls `processPresynced(samples, 2)` which skips 2
    training symbols from the buffer START, and `sync_position_ = search_start + start_sample` points
    AT the training (the `training_start=63839` in the log is only the CFO phase ref, and cfo=0 here);
    NOT the CW-count (2 vs sent 1) — that fallback is a SYMPTOM: the CW0 header peek can't read a
    valid header BECAUSE the demod is already garbage. ROOT: post-burst, BRAVO does not fully revert
    to NON-burst interactive mode. The FF is a small non-burst interactive frame that on the normal
    (no-preceding-burst) path delivers fine via LIGHT-LTS + the proven non-burst decode; but here it
    is sent with a FULL preamble (post-burst `forceNextFrameFullPreamble` / `expect_full_ofdm_anchor_`
    re-anchor) and decoded through the full-anchor burst-data path, which mis-estimates/mis-structures
    it → garbage soft bits. LAYER 1's latch-clear reverted ONE piece of burst state
    (`have_burst_descriptor_`); the remaining burst-data-mode state (the full-anchor expectation, the
    data-decode profile, `fixed_frame_codewords_`) is not reverted, so the trailing FF is treated as
    burst data. Fix = complete the post-burst revert so the trailing interactive frame decodes as a
    normal non-burst frame — a COORDINATED encoder+decoder change (both ends must agree the burst is
    over and drop back to the light-LTS interactive PHY). This is the production form of the
    declared-but-unwired `traffic_profile.hpp` TrafficClass system (File vs Chat/Control). Deliberate
    work, must not regress the burst path; do not land blind against hanging-run verification.
  - LAYER 2 SIDESTEPPED (2026-06-05): rather than fix the broken non-burst full-anchor demod, the
    TNC now ROUTES the trailing FF through the BURST path too (`bulk_burst_started_`,
    tnc_session.cpp): once a body has bursted, trailing flushes stay burst, reusing the PROVEN
    burst descriptor+group decode (same path that delivers the body 10/10). This made the image
    deliver byte-intact over burst.
  - LAYER 3 (clean teardown — **REMAINING**): the FF is a tiny (3-byte) 1-frame burst. It DELIVERS
    (BRAVO `Received OK (3 bytes)`), but its tone-burst GROUP-ACK does not get back to ALPHA
    (`group-ACK timeout`), so ALPHA's `file_transfer_` stays SENDING/busy. The next trailing flush
    then fails `sendFile` "already in progress" and the bulk-accum retries every quiet period (~65
    wasted re-stages of the same 3 bytes) → the sender's `connect` never exits cleanly (the message
    IS delivered; only the sender-side close hangs). Root: tiny / turn-flip-boundary 1-frame burst
    groups don't ARQ-complete on the sender side (related to the old "Issue 2 turn-flip" ACK
    reliability). Fix candidates: (a) make tiny/last-group tone-burst ACKs reliable at the turn
    boundary; (b) coalesce body+FF into ONE burst (under-report BUFFER 0 early so PAT writes the FF
    before the body transmits, then burst both — risk: premature Flush close); (c) the LAYER-2 PHY
    fix so the FF can go non-burst. An idle-gated flush (only flush when `getTxBackloggBytes()==0`)
    was added but does NOT cover this (a tiny burst reports ~0 remaining bytes while file_transfer_
    is still busy on the ARQ).
- Symptom: with `ULTRA_TNC_BULK_ACCUM=1`, a PAT B2F body now bursts (z=81) and decodes CRC-clean,
  but the trailing non-burst `FF` terminator never reaches PAT (`Receiving … offset 0`, mailbox
  empty). PAT retries the whole message (`100%→0%→100%`) → the run HANGS instead of disconnecting.
- Root cause: after acquiring the post-burst full-anchor chirp (now succeeds, `up_pos` small),
  BRAVO does a control-first peek (1 CW) then processes the frame with the **burst-data geometry**
  it's still holding — `samples=10080` (burst frame size), reconfigures to the QPSK R1/2 *data*
  profile — but ALPHA's `FF` is a NON-burst frame (29 B → 106880 samples, full preamble, ~2 CW at
  R1/4). Geometry/profile mismatch → the FF never completes → not routed to the data port. The
  plain non-burst path WORKS when NOT preceded by a burst (the pre-bulk-accum image delivered),
  so the burst-left decoder state (`have_burst_descriptor_` set per group at
  `streaming_ofdm_decode.cpp:762`, the burst frame geometry, warm-sync next-group expectation) is
  what mis-decodes the trailing non-burst frame. There is no end-of-burst signal, so the receiver
  stays in burst-decode mode.
- SURGICAL root cause (fully traced 2026-06-05, with debug logging since reverted):
  1. The BODY delivers fine. Confirmed via temporary logs in `onFileReceived`/`onModemDataReceived`:
     the burst body decodes CRC-clean and BRAVO-TNC delivers all 12215 bytes out its data port to
     PAT-BRAVO (`state=CONNECTED, marker=raw, data_out=1`). The POLLOUT flush is correct. In B2F the
     sender writes body → `Flush()` → THEN the `FF` terminator, so PAT-BRAVO HAS the body and is
     waiting only for the FF (its `offset 0` line is just stale; VARA `Read` has no deadline so it
     waits forever, no timeout).
  2. The FF (a 1-CW non-burst frame after the burst) is DROPPED at the §14.24 gate
     `streaming_ofdm_decode.cpp:1013-1022`: `if ((use_burst_interleave_ || burst_transport_rx_) &&
     connected_ && have_burst_descriptor_) { re-search; return; }`. The control-first peek decodes
     the FF's CW0 at R1/4-control profile → garbage (the FF is DATA, not control) → reaches this
     gate → re-searched away.
  3. THE LATCH: `have_burst_descriptor_` is set on the first BURST_HEADER
     (`streaming_ofdm_decode.cpp:762`) and PERSISTS for the WHOLE connection — by deliberate design
     it is cleared ONLY on disconnect (`streaming_decoder.cpp:537-551`; it is the RX source-of-truth
     for the descriptor-declared z=81, must survive across groups+ACKs within a transfer). So after
     the body burst it stays set forever and the gate drops EVERY post-burst non-burst frame.
- Fix direction (NOT done — deliberate cross-engine change, must not regress §14.24 / z-lifecycle):
  clear `have_burst_descriptor_` when the burst FILE TRANSFER COMPLETES (the clean "burst is over"
  signal — preserves in-burst z-sizing). The completion signal lives in the ProtocolEngine
  (`FileTransferController::on_received_` → `Connection::setFileReceivedCallback`), but the latch
  lives in the ModemEngine's `StreamingDecoder`; there is no existing ProtocolEngine→ModemEngine
  path, so this needs new wiring (e.g. a `StreamingDecoder::clearBurstDescriptor()` invoked from the
  file-received callback at the app/TNC layer that holds both engines). Do NOT shortcut with
  `&& waveform_->wasBurstInterleaved()` on the gate — mid-burst noise has a ~50/50 marker and would
  fall through to the data decode, re-introducing the §14.24 shared-estimate poisoning.
- BONUS bug found: bulk-accum (BRAVO side) also hoards PAT-BRAVO's small CONTROL sends (BUFFER 50 on
  its SID/FS+) — the cap should apply only to bulk bodies, not tiny control writes.
- Repro: `ULTRA_TNC_BULK_ACCUM=1 /tmp/b2f_pat/run_image.sh` (BOUND with a timeout — it HANGS, PAT
  retries 100%→0%→100% forever). Diagnostics: VARA_DEBUG=1 on PAT for its data RX;
  `ULTRA_S16_TRACE_WARM_WINDOW=1` + BRAVO `--log-category …,sync`; mailbox
  `/tmp/pat_bravo_mbox/BRAVO/in/*.b2f`.

### BUG-TNC-B2F-001: bidirectional B2F stalls — the NON-BURST short path (the actual message path) was dead at RX + ACK
- Status: **FIXED for the message path** (2026-06-03). Issues 1, 3, 4 fixed; Issue 2 (burst
  turn-flip) remains but is **off the B2F message path** (Winlink messages are small → non-burst).
- Symptom: PAT↔PAT Winlink message over `ultra_tnc`/OTASim connects + completes the B2F
  handshake, then never delivers. **Reproduces single-machine** (shared clock) → NOT a
  cross-machine timing problem. (Earlier framing chased the burst path because an A/B test
  forced `kInteractiveMaxBytes=0`; the DESIGN routes small messages to the non-burst path.)
- Root cause (Issue 1, fixed `c27aa45`): the one-way burst bypass skipped the ISS/IRS turn
  gate → both B2F stations keyed up uncoordinated and collided. Fixed via `half_duplex_interactive_`
  turn-gating. Serialization verified correct.
- **Root cause (Issue 3, fixed 2026-06-03): the receiver synced cleanly (corr=1.0, SNR 28 dB)
  yet DROPPED every non-burst DATA frame.** When `burst_transport_rx_` became the unconditional
  default (ULTRA_BURST_TRANSPORT removed 2026-06-02), the §14.24 control-peek-fail re-search at
  `streaming_ofdm_decode.cpp:~1004` started discarding ALL non-burst multi-CW DATA as
  "burst-regime noise": a non-burst `sendBinary` frame carries no BURST_HEADER descriptor →
  `pending_total_cw_=0` → it enters the 1-CW control peek, fails it, and was re-searched away.
  Fix: gate the re-search on `sync_controller_.have_burst_descriptor_` (only mid-burst, where a
  real burst's SHARED channel estimate is at risk); a standalone non-burst frame falls through
  to the legacy data decode → delivered via SR-ARQ.
- **Root cause (Issue 4, fixed 2026-06-03): the responder ACK'd in the wrong waveform.** The B2F
  responder pre-confirms `handshake_confirmed_` in `enterConnected` (so it can speak first),
  which skipped the only site that fires `on_handshake_confirmed_()` → the modem's
  `handshake_complete_` stayed false (reset by `setConnected()`, with `setWaveformMode`→OFDM
  landing afterwards), so BOB keyed its SR-ARQ ACK in **MC-DPSK** while ALICE listened in OFDM
  → ACK never landed → ALICE retransmitted to the retry cap. Fix: re-fire `on_handshake_confirmed_()`
  once on the responder's first decoded frame (guaranteed past `setConnected`+`setWaveformMode`),
  switching TX onto the negotiated OFDM data waveform.
- Verified: `tests/test_ultra_tnc_sim_audio` (two real `ultra_tnc --sim-audio` over OTASim).
  Default = bulk burst file (8192 B, CRC-clean — no regression). `ULTRA_TNC_TEST_NONBURST=1` =
  bidirectional 300 B short message, **CRC-clean BOTH directions** (was: forward dropped entirely).
- **End-to-end VERIFIED (2026-06-03):** real PAT↔PAT Winlink B2F, single machine
  (OTASim → 2×`ultra_tnc` → 2×PAT). `pat connect varahf:///BRAVO` ran the FULL exchange clean —
  banner → proposal → `FS +` → body 0%→100% → `FF`/`>FQ` → disconnect — and the message landed in
  BRAVO's inbox. The prior session stalled at the proposal; now it delivers.
- Root cause (Issue 2, still open): the BURST path needs full chirp+LTS **anchor re-acquisition
  on every turn-flip**; a burst transfer whose turn flips mid-stream re-acquires at corr~0.27.
  Not on the message path; relevant only to bidirectional *bulk* over one connection.
- Full diagnosis + repro: `docs/TNC_B2F_HALFDUPLEX_FINDINGS_2026_06_03.md`.

### BUG-FINACK-001: Final-group ACK loss → sender infinite-resends the last group; no clean transfer close
- Status: **FIX LANDED, UNVALIDATED** (2026-06-02). Decode-independent re-ACK implemented
  in `connection.cpp::onBurstGroupReceived` (the `!all_ok` else branch now routes a resent
  ALREADY-DELIVERED group into the controller's existing `onGroupReceived`→`seqLess`→re-ACK
  path instead of dropping it). Builds; burst/ARQ unit tests pass (StopWaitARQ,
  SelectiveRepeatARQ, ConnectionAdaptive, ToneBurstAckMonitor/Watterson). **NOT yet validated
  on a triggering run** — needs a fade-timed seed where the final GROUP_ACK is genuinely lost
  so the re-ACK actually fires and the transfer closes cleanly on-air (re-ACK fired 0× on the
  post-fix runs because their final ACK happened to land). PRODUCTION-VISIBLE: the GUI/sweep
  quick-kill (scenario_passed now PASSes on delivery + pkills) MASKS this in the harness, but
  real hardware will still show the wasted-airtime infinite last-group resend until validated.
  Was OPEN since 2026-05-31, folded into thread C (`docs/OFDM_COHERENT_ONLY_DECISION_2026_05_31.md §5`).
  Pre-existing; exposed by a fade landing on the completion ACK.
- **Still TODO for the robust fix:** a FILE_END / completion handshake (FILE_END → FILE_END_ACK
  → DISCONNECT) so the close doesn't depend on a single per-group ACK landing at all. The re-ACK
  breaks the infinite loop; the handshake is the belt-and-suspenders close.
- Area: burst completion handshake — `Connection` GROUP_ACK path + the "payload drained →
  auto-disconnect" trigger; receiver duplicate-group handling in `onGroupReceived` / burst assembler.
- Reported by operator (live GUI), coherent QPSK R1/4 Moderate@14 seed 777:
  "it received the file and alpha is still sending."
- Repro / evidence (`/tmp/zmode_mod14_qpsk_s777b`): Bravo logs `Received …10240 bytes, CRC ok`
  at 282 s (file COMPLETE). Alpha sits at `cursor=10240/10240, resent=1, resend_left=0` and
  re-emits `TX Burst: 6 frames` for `group_seq=18` every ~10 s (ack_timeout) indefinitely; the
  scenario only ends via `exit_after`. seed 42 escaped it solely because its final ACK happened
  to land and `auto-disconnect (payload drained)` fired.
- Root cause: the completion handshake has no robust close. The last group's **GROUP_ACK was lost
  in a fade**, so the sender never learns the receiver has it and resends forever; "payload drained
  → disconnect" never triggers because the sender still considers the final group unacked. When the
  sender resends an **already-delivered** group, the receiver does **not re-ACK** — it tries to
  re-decode the duplicate (which here also hits the fade-lost-descriptor z=27 path →
  `0/6 max_iters=0`, a red herring) and stays silent. The file delivered fine; only the close fails.
- Impact: on real hardware, wasted airtime (sender keeps keying the last group) and a transfer that
  never cleanly terminates without an external timeout. Half-duplex final-ACK fragility.
- Fix (thread C): group-level **duplicate detection + re-ACK** — a resend of an already-delivered
  group re-emits the GROUP_ACK **without** re-decoding. Plus a proper **FILE_END / completion
  handshake** so the transfer terminates on "receiver has the whole file" (FILE_END → FILE_END_ACK →
  DISCONNECT), not on a single per-group ACK happening to land. (Note: §7.6 z-consolidation would
  *partly* help IF the re-ACK were decode-gated — but the robust fix is decode-independent.)
- Verification: a fade-timed seed where the last GROUP_ACK is lost (seed 777 Moderate@14) must still
  reach a clean DISCONNECT (no infinite group-18 resend); `ALPHA_DISCONNECTED_COUNT>0`, RESULT=PASS.

### BUG-CTRL-001: Control path is still the bottleneck in aggressive fading profiles
- Status: IN_PROGRESS (handshake leg fixed 2026-04-26)
- Area: OFDM connected mode (ACK/SACK/control reliability)
- Symptoms:
  - Data codewords decode but ACK reception misses trigger avoidable retransmits/timeouts.
  - Most visible with aggressive profiles (for example D8PSK R1/2), but can still appear on weaker seeds in other OFDM rates.
- Impact:
  - Throughput tail collapse on bad seeds.
  - File transfer latency variance much larger than message transfer variance.
- Current mitigations already in tree:
  - R1/4 control profile for OFDM control frames.
  - ACK repeat/coalescing and improved ARQ observability.
  - `DISCONNECT_SEQ` protection against stale data ACK being mistaken as disconnect ACK.
  - **Proactive CONNECT_ACK retransmission (responder side)** — covers handshake-leg
    losses at OFDM data modes. Auto-mode baseline at SNR=15 moderate (DQPSK R1/2)
    went from 4/5 → 5/5 message tests and 2/3 → 3/3 file 2048 tests on 5/3-seed
    samples. See CHANGELOG 2026-04-26.
  - **Airtime-derived ACK/retransmit RTO (commit `d182751`, 2026-05-23, backlog #119).**
    Replaced the `[8000,12000]` magic timeout clamp with an RTO derived from burst
    airtime + carrier-sense/SACK coalesce + ACK airtime, so the sender no longer
    times out before a half-duplex-deferred ACK can physically return. Eliminated
    premature timeout-retransmission of already-delivered frames on a clean channel
    (AWGN SNR20 16QAM R1/2: 5/10 seeds → 0/10). See CHANGELOG 2026-05-23. NOTE: a
    longer honest RTO may add idle wait at window boundaries on clean channels —
    quantify under #121 (recoverable dead air) before further RTO tuning.
- Remaining work:
  - Connected-mode tail variance under aggressive forced profiles (D8PSK R1/2,
    DQPSK R3/4) when channel falls outside auto-selector envelope — these aren't
    auto-selected, so they bite only operators forcing modes manually.
  - BRAVO missing the initiator's CONNECT (opposite leg of the same race) is rare
    on the production envelope but lacks a comparable fast retry — initiator's
    `connect_timeout_ms = 60000` is far longer than the cli_simulator harness's
    30s PHASE 1 budget, so harness exposure of this case looks like seed noise.

### BUG-NACK-001: Burst GROUP_NACK sent over MC-DPSK handshake waveform, not the tone-burst
- Status: FIXED (2026-05-29, branch feat/oneway-arch-2026-05-27, commit pending). Verified
  seed 2 Good@20 warm-ON: tone-burst NACK emits=1, MC-DPSK NACK=0, MC-DPSK TX post-connect=0,
  file PASS 11/11 CRC-clean, run ~34 s faster (171 s vs 205 s).
- Area: burst transport failure path (`Connection::onGroupReceived`, the `!all_ok` branch)
- Reported by operator (waterfall observation), seed 2 Good@20 file transfer:
  - "Bravo issues a NACK using the old NACK system, not the tone-burst."
  - "After group 0's initial failure there is a weird MC-DPSK signal repassing on the waterfall."
- Root cause (BOTH symptoms are one bug): on a failed group the receiver sent
  `transmitFrame(makeGroupNack(...))`, a 20-byte control frame over the CURRENT
  waveform. For group 0 `handshake_complete_` is still false (it is set later, in
  the `all_ok` path the NACK branch `return`s before reaching), so the frame went
  out as the **MC-DPSK handshake waveform = 149760 samples ≈ 3.1 s** on the air —
  the "weird MC-DPSK signal" — instead of the §15 tone-burst (675 ms). The §15
  work routed the success GROUP_ACK to the tone-burst but left the failure
  GROUP_NACK on the legacy control-frame path.
- Impact:
  - ~4.6× slower NACK (3.1 s vs 675 ms) → slower deep-fade resend recovery (a chunk
    of seed 2's latency); off-waveform MC-DPSK energy mid-OFDM transfer.
- Fix: emit a NACK-type tone-burst (`AckType::Nack`, whole-group missing mask) from
  the `!all_ok` branch, mirroring `setSendGroupAck`. The sender's `onToneBurstAck`
  already maps a NACK-type tone-burst to `burst_transport_.onGroupNack` (resend
  now), so the entire receive path already existed — only the emit was wrong.
  Falls back to the OFDM `makeGroupNack` frame if the tone-burst callback is absent.
- Verification: seed 2 warm-ON GUI re-run — expect 0 MC-DPSK TX bursts after
  connect and tone-burst NACK emits > 0, file still CRC-clean.

### BUG-8PSK-001: Decision-directed tracking corrupts the 8PSK channel estimate on fading
- Status: FIXED (2026-05-29, channel-adaptive DD gate). DD cascade removed; see
  `docs/8PSK_GOOD_FADING_DIAGNOSIS_2026_05_29.md` and
  `docs/SYSTEM_PICTURE_FADE_SURVIVABILITY_2026_05_29.md`.
- Fix: `use_coherent_dd` now also requires `last_fading_index < 0.15`
  (`channel_equalizer_pilot.cpp`) — DD off on frequency-selective/faded frames
  (where its wrong decisions poison H), on for AWGN/flat frames (where it's
  safe). Threshold from measured data (AWGN ≤0.07, Good ~0.34 median) and equal
  to the existing LLR-scaling "faded" boundary. Env override `ULTRA_DD_FADING_MAX`
  (default 0.15); `ULTRA_COHERENT_DD_OFF=1` force-off. A per-symbol pilot-anchor
  innovation gate was tried first and removed — ineffective (a wrong-decision
  rotation and a legit between-pilot interpolation error are indistinguishable
  per-symbol; flat across a 4× tightness sweep).
- Verification: GUI 8PSK AWGN30 PASS 2330 bps 0 CW fail (DD stays on, no
  regression); Good@20 cascade gone (DD-on 83–125 CW fails/no delivery →
  adaptive: seed 42 PASS, seed 43 delivered CRC-clean). Offline (measure_ack_fer
  qam8 Good@20): adaptive == DD-off (46/120 chunks) vs DD-on 41/120.
- RESIDUAL (separate, NOT this bug): 8PSK is too marginal a rung for Good@20 —
  delivers slowly with heavy resends and fails on hard fades (seed 44: 0
  delivery). QPSK is ~2× more survivable there (offline 106/150 vs 52/150
  chunks). This is a rung-selection design item (adaptive mod choice per
  channel), tracked in `docs/SYSTEM_PICTURE_FADE_SURVIVABILITY_2026_05_29.md`,
  not a DD bug.
- Original diagnosis (root-cause history):
- Area: `src/ofdm/channel_equalizer_pilot.cpp` (`use_coherent_dd`, ON for QAM8/QAM16)
- Symptom: forced 8PSK (QAM8) R3/4 delivers perfectly on clean AWGN (2330 bps) but
  fails on Good@20 fading — heavy resends, frequently no delivery, confident-WRONG
  bits (strong |llr|≈15-20, LDPC parity fails 50-99 unsat). Bimodal per group (6/6
  or 0/6, never partial).
- Root cause (experimentally confirmed by five-way elimination — chain/fade/demap/
  static-H all ruled out): decision-directed channel tracking feeds 8PSK's occasional
  wrong hard-decisions (its 22.5° boundaries are tight on a fading channel) back into
  the H estimate, poisoning it → cascade of confident-wrong bits. `ULTRA_COHERENT_DD_OFF=1`
  flips 8PSK Good@20 from FAIL → PASS. The base LTS+pilot H is fine (DD-off uses it and
  delivers); a genie perfect-H with DD *on* still failed (DD corrupts even a perfect H).
  Matches the code's own comment warning DD "poisons H on bad decisions during fades."
- Impact: 8PSK is the promotion lever toward 3000 bps; this blocks it on fading.
- Fix direction: reliability-gate DD (only DD a symbol whose EVM is well inside the
  95% noise radius — the code comment's own proposal), or gate DD off for 8PSK on
  fading; pair with channel-adaptive promotion (8PSK on clean/mild, QPSK on deep fade).
- Residual after DD-off: still marginal (heavy resends) — irreducible
  frequency-selective phase near spectral nulls; ARQ/adaptive-rate handles the tail.

### BUG-HARNESS-001: gui_qso_scenario.sh hard-aborts on auto-negotiated mode != --expect-mod
- Status: KNOWN (2026-05-29; harness limitation, not a modem bug).
- Symptom: on a channel where the auto rate-ladder legitimately promotes (e.g. AWGN30
  → 16QAM), running with `--expect-mod QPSK` makes `hard_failure_reason` abort the run
  at the MODE_CHANGE (`REASON=unexpected_data_mode`), ~25 s in, 0 file-transfer
  attempts. Falsely looks like a QPSK/AWGN modem failure (it is not — forced QPSK on
  AWGN30 delivers fine).
- Fix direction: when rate is not locked/forced, treat `--expect-mod` as advisory
  (accept the auto-selected mode) instead of hard-aborting.

### BUG-HARNESS-002: measure_ack_fer fidelity defects (under-piloting + broken AWGN burst path)
- Status: PARTIALLY FIXED (2026-05-29). The pilot-spacing defect is fixed; the AWGN
  burst_chunk defect is documented, not yet fixed.
- Defect 1 (FIXED): `makeOFDMConfig` hardcoded `cfg.pilot_spacing = 10`, while
  production uses `ofdm_link_adaptation::recommendedPilotSpacing(mod,rate)` = 5 for
  R1/2 & R2/3, 8 for R3/4. This under-piloted coherent high-order mods by ~2× and
  made 16QAM look structurally worse than production would. Fixed to call
  `recommendedPilotSpacing` (`tools/measure_ack_fer.cpp`). NOTE: re-test showed the
  under-piloting was NOT the cause of 16QAM folding (folds at spacing 5 too) — but
  the harness should still match production.
- Defect 2 (KNOWN): `--config burst_chunk --channel awgn` returns 0 frames recovered
  for all mods (qpsk/qam8/qam16, all DD modes) — the offline AWGN burst path does not
  match the GUI/OTASim AWGN path (where these mods decode fine). So measure_ack_fer
  AWGN burst_chunk numbers are not usable; use the GUI for AWGN.
- Defect 3 (KNOWN, 2026-05-29): the **Z=81 / N=1944 (long-LDPC burst keystone) path
  decodes to 0% in measure_ack_fer — even noiseless QPSK** — on BOTH the frame path
  (`--config data4_full` + `ULTRA_LDPC_Z=81`) and the burst path
  (`--config burst_chunk` + `ULTRA_LDPC_Z=81`). Verified Good@60: qpsk/qam16 = 0/100
  at Z=81 vs 91/84 at Z=27; confirmed pre-existing (reproduces with all genie edits
  git-stashed). ROOT CAUSE: `ULTRA_LDPC_Z=81` changes the ENCODER codeword size and
  the decoder's *block size* (→1944, `streaming_decoder.cpp:641`), but
  `decodeFixedFrame`'s `ldpc_z` argument is sourced ONLY from the burst descriptor
  (`last_burst_descriptor_.lifting_z`, `streaming_ofdm_decode.cpp:2876-2889`), NOT
  from the env. With no BURST_HEADER on the wire (frame path never sends one; the
  burst_chunk harness only enables the descriptor in its `wrong_group` branch), the
  decoder accumulates 1944 soft bits but decodes them with a **Z=27 matrix** →
  guaranteed failure. Production (GUI burst transport) IS unaffected: it transmits the
  BURST_HEADER with `lifting_z=81`, so the decoder learns Z and decodes correctly
  (GUI-proven CRC-clean ~3.1 kbps Good@20). CONSEQUENCE: **measure_ack_fer cannot
  validly screen the production burst config (Z=81); all offline Z=81 numbers are
  garbage.** The valid offline data is Z=27 only.
- DEEPER ROOT CAUSE (2026-05-29, attempted fix found it is MULTI-LAYER — NOT a quick
  fix): even with the encoder emitting Z=81 (via `setLDPCLiftingZ`, added as
  `--ldpc-z 81`) AND the decoder env-forced to Z=81 (block-size getter +
  `decodeFixedFrame` ldpc_z both read `ULTRA_LDPC_Z`), Z=81 STILL decodes 0% — even a
  1-CW frame, noiseless. Two more layers:
  (1) the waveform's soft-bit-grab size (`active_ldpc_block_size`, used by
      `getSoftBits`) is bumped to 1944 ONLY by `setActiveLDPCLiftingZ()`, which fires
      ONLY on descriptor-consume (streaming_ofdm_decode.cpp:763). With no descriptor
      the decoder grabs 648 bits of a 1944-bit codeword ("Got 648 soft bits,
      proceeding to decode") → garbage.
  (2) the descriptor-consume itself does not fire in the harness: the BURST_HEADER (a
      DQPSK-R1/4 control frame) is routed to the DATA decode path (line 1124), not the
      CONTROL path (line 661, which recognizes `FrameType::BURST_HEADER`), so it is
      mis-decoded as a failed data frame and dropped — never consumed (verified:
      `--burst-descriptor 1` gives 0% at BOTH Z=27 and Z=81 on Good@60).
  KEY INSIGHT (user): the BURST_HEADER is the SINGLE self-describing conveyor of
  lifting_z AND cw_per_frame AND group_size AND interleave flags together — the
  decoder is designed to learn the whole burst geometry from the wire in one shot, so
  piecemeal patching (env for Z, setFixedFrameCodewords for cw) cannot reproduce
  production. The ONLY faithful fix is to make descriptor-consume fire in the harness
  (route the BURST_HEADER to the control decode path). That is real plumbing across
  the control-detection + block-size + descriptor-consume layers — deferred. The
  `--ldpc-z 81` scaffolding (encoder emits + announces Z=81; env wires the decoder
  decode matrix + block-size getter) is committed as a correct PARTIAL toward that fix.
- Impact (updated): characterize ANY 16QAM-on-production-burst (Z=81) claim on the
  real-time GUI; the offline harness cannot do it until descriptor-consume is fixed.
- Impact: measure_ack_fer is a fast *screen* for relative offline comparisons on the
  Good path at **Z=27 only**; its AWGN path and its entire Z=81 path are not faithful.
  Confirm any fade/throughput conclusion — and ANY 16QAM-on-production-burst claim —
  on the real-time GUI per the project's standing rule.

### BUG-CFO-001: OFDM two-stage CFO refinement remains incomplete
- Status: OPEN
- Area: `src/ofdm/demodulator.cpp`
- Evidence:
  - TODO at `src/ofdm/demodulator.cpp:1307` for proper two-stage CFO (CP/frequency-domain refinement).
- Impact:
  - CFO handling works for current tested profiles but remains less robust than desired for broader OTA variation.
- Next steps:
  - Implement and validate two-stage CFO refinement with seeded regression + OTA logs.

### BUG-TEARDOWN-001: RX decode thread UAF crash on disconnect/shutdown (SIGSEGV in HilbertTransform::process)
- Status: OPEN (surfaced 2026-05-31; PRE-EXISTING — identical faulting stack in crash
  reports from 2026-05-28, predating the SyncController work).
- Symptom: `ultra_gui` (receiver side) SIGSEGVs at end-of-transfer / disconnect / shutdown.
  EXC_BAD_ACCESS / KERN_INVALID_ADDRESS. Faulting thread = the RX decode thread:
  `HilbertTransform::process` ← `OFDMChirpWaveform::detectDataSync` ← `StreamingDecoder::
  searchForSync` ← `processBuffer` ← `ModemEngine::rxDecodeLoop`. The receiver's log ends
  abruptly right after `Disconnected ... (Remote disconnected)` + `SR-ARQ: Reset`, with NO
  `RX decode thread stopped` line (cf. the sender, which tears down cleanly).
- Does NOT corrupt results: the file delivers CRC-clean BEFORE teardown (e.g. QPSK R1/4
  AWGN@10 seed42 → RESULT=PASS, GOODPUT=380, 0 retx, then crash on teardown).
- Root cause (CONFIRMED from both racer stacks 2026-05-31): on disconnect, the MAIN thread
  synchronously rebuilds the waveform while the RX decode thread is mid-demod on the old one:
  - WRITER (main thread, frees/rebuilds): `Connection::enterDisconnected` → `ModemEngine::
    setConnected(false)` → `StreamingEncoder::setDataMode` → `MCDPSKWaveform::initComponents`
    → `new MultiCarrierDPSKModulator` → `new ChirpSync` → `ChirpSync::generateTemplate`.
  - READER (Thread 22, faults): `rxDecodeLoop` → `StreamingDecoder::searchForSync` →
    `OFDMChirpWaveform::detectDataSync` → `HilbertTransform::process` → UAF (`KERN_INVALID_
    ADDRESS`, small offset = deref into freed/rebuilt waveform memory).
  The §14.36 "crash fix v3" (`applyPendingConnectedOFDMMode`) deferred only CONNECTED-mode
  reconstruction to a safe boundary in the RX loop; the DISCONNECT reconfig
  (`setConnected(false)`→`setDataMode`→waveform rebuild) is NOT synchronized with the RX
  thread at all. `stopRxDecodeThread()` joins, but `enterDisconnected` runs the rebuild on the
  protocol/main thread WITHOUT first quiescing the still-running RX `detectDataSync`.
- Why it matters for the SyncController refactor: this is exactly the §7.7#4 lifecycle-
  discipline scope the refactor must own. It also POLLUTES the GUI gate's crash signal —
  every gate run crashes at teardown, so a refactor-introduced crash can't be cleanly
  distinguished from this one. Fixing it first gives the behavioral phase a clean crash
  baseline.
- Fix direction: synchronize disconnect/shutdown waveform reconfig with the RX thread —
  either stop+join (or quiesce under `reset_generation_` + `buffer_mutex_`) BEFORE the
  waveform/HilbertTransform is reconfigured on disconnect, or route the disconnect reconfig
  through the same deferred safe-boundary the connected path uses. Fold into the
  SyncController lifecycle ownership (`reset()` discipline).

## Release Blockers

An issue is release-blocking if it causes any of:
- reproducible data loss,
- deterministic disconnect deadlock,
- non-deterministic gate failure in default mode ladder.

Current blockers:
- None identified for `0.2.1-alpha` default ladder.

## Fixed Bugs

- 2026-08-05: BUG-STALE-SNR-SENTINEL-DRIVES-DECISIONS fixed — the "no measurement" marker
  was steering live rate decisions. IONOS rig logged three consecutive demotes
  (**R2/3 → R1/2 → R1/4**), each stamped `SNR=-10.0 dB (ofdm_broadband)`, on a link that was
  successfully transferring a file (`local_fading=0.12 AWGN`). **Root cause:** `-10.0` carries
  two meanings no consumer can separate. `kConnectSnrStaleSentinelDb` (−10.0) means "no
  measurement"; the existing comment argued it is collision-free because no frame decodes at a
  true −10 dB effective SNR. That argument justifies the ENCODING, not the CONSUMPTION — the
  sentinel is a finite float, so `isfinite()` passes and a decision consumer reads it as a
  catastrophic channel. `wireSnrDb()` returns it whenever the connect-SNR pool holds nothing
  younger than 3·Tc, and it flowed straight into `recommendCWCountForChannel()`, so a healthy
  link with a merely STALE pool got the most conservative geometry. **Fix:** separate what we
  REPORT from what we DECIDE with — `isStaleSnrSentinel()` (0.5 dB tolerance, matching the
  GUI's existing n/a render) plus the pure, env-free selector `decisionSnrFromWire()`;
  `Connection::decisionSnrDb()` delegates and falls back to `measured_snr_db_`, which is only
  ever assigned from a finite reading that passed `acceptsRateSelectionSNR()`. The wire is
  deliberately unchanged — it still sends the honest "unknown" (byte 0 → peer renders n/a),
  because reporting a frozen number is the dishonesty the freshness gate exists to prevent.
  Direction set by cost asymmetry: mistaking a real −10 dB channel for "unknown" costs nothing
  (no rung is usable there, every floor ≥ 5 dB); mistaking "unknown" for −10 dB collapses a
  healthy link. **Scope note — the first fix was 1 of 4.** A sweep of every `wireSnrDb()`
  consumer found three more decision sites (`connection.cpp:4272`, `:4336`, and
  `tryDescriptorModeSwitch()`, whose `measured_snr` parameter is fed `wireSnrDb()` by both its
  call sites). The descriptor path is the one that runs mid-file-transfer — exactly where the
  rig saw it — so stopping at the first site would have left the observed defect alive. Also
  corrected a stale comment claiming `ULTRA_WIRE_SNR_FRESH` is "default OFF" while the code
  beneath returns DEFAULT-ON since 2026-07-05; that disagreement is why the path is reachable
  in production at all. Tests: the seam is covered by unit-testing the pure selector — the test
  binaries pin `ULTRA_WIRE_SNR_FRESH` process-wide via a function-local static latch, so a
  knob-dependent Connection test cannot exercise it (that variant was written, found
  un-runnable, and removed rather than left passing vacuously). ctest 101/101.

- 2026-08-05: BUILD-PROVENANCE-STALE fixed — `ultra_gui --version` reported the commit the
  build tree was CONFIGURED on, not the code in the binary. The Mac reported
  `commit=923ce70 dirty=false` while running feature-branch code, during a rig campaign where
  binary provenance is the entire basis for attributing a measurement; caught only because
  `strings` disagreed with `--version`. `execute_process(git rev-parse)` + `configure_file`
  run once at configure time and incremental builds never re-run configure. Fixed by
  regenerating `build_info.hpp` at BUILD time (`cmake/UltraBuildInfo.cmake` via an ALL custom
  target), writing a temp and `copy_if_different` so the 6 TUs including it rebuild only when
  git state actually moved (verified: no-op rebuild recompiles 0 TUs). `kBuildTimeUtc` is now
  HEAD's COMMIT date, deliberately not wall-clock — a wall-clock stamp differs every
  invocation, defeating copy-if-different and forcing a rebuild every time. Verified: reports
  the true commit and flips `dirty` correctly across a commit.

- 2026-08-05: BUG-MESSAGE-LOST-ON-FORCED-DEMOTE fixed — an in-flight operator message was
  destroyed by a MANDATORY geometry escape (receiver-commanded / stuck-frame demote), on
  **Good@20, a clean channel**, 2 of 10 GUI matrix rows. **Root cause:** `applyDataMode()`
  failed the logical object rather than serializing old-sized fragments through the smaller
  frame builder — correct as far as it went (silent truncation is worse than loss), but it
  treated a FRAGMENTATION problem as an OBJECT problem. A geometry change invalidates the cut,
  not the content; the payload is still in hand and nothing physical requires it to die.
  **Why the obvious fix fails:** simply re-sending duplicates the message, because "record
  still active" covers both mid-flight AND fully-sent-awaiting-ACK — the classic ARQ ambiguity.
  Both sender-side variants were built and MEASURED unsound (duplicate delivery `rx = #1/#2/#2/#3`;
  then a narrowed gate that stopped firing). No sender-side heuristic resolves it: the sender
  cannot know what the receiver holds. **Fix — make the resend IDEMPOTENT so the sender no
  longer has to guess:** (1) new `PayloadType::TEXT_MESSAGE_OBJECT` = `0x04`, wire form
  `{0x04, object_id, text…}` in the message-object payload prefix — NOT in `DataFrame::HEADER_SIZE`,
  which feeds `FIXED_FRAME_OVERHEAD`/CONNECT sizing/the RX framer; (2) a rolling per-session
  `object_id` allocated at admission and held CONSTANT across re-grids; (3) receiver keeps the
  last 8 delivered ids (`delivered_message_object_ids_`) and drops a repeat in
  `handleDataPayload` before any application callback; (4) the sender re-grid is re-enabled in
  `applyDataMode()`, bounded at `kMaxMessageGeometryRegridAttempts` = 2, re-admitted at the
  FRONT of `queued_payloads_` by admission rank so the cross-class FIFO holds, gated on
  `arq_.moveEpochEnabled()` (the forced epoch bump is what makes the peer drop the stale
  partial prefix — without it the path still fails closed and disconnects). A 1-bit generation
  toggle was rejected with a counterexample: a fully lost object desynchronizes the toggle and
  the NEXT message is dropped as a duplicate = silent loss, the original bug reintroduced.
  **Also hardened:** a reassembled text object whose first byte is neither `0x04` nor the legacy
  `0x00` is now discarded with a WARN instead of being handed to the operator as content —
  that is exactly what a stitched stale-prefix object looks like. Status truthfulness holds:
  one SUBMITTED and one terminal result per object across any number of re-grids, `submitted_at`
  stamped only on the FIRST wire submission so elapsed_ms reports the operator's real wait, and
  the legacy `on_message_sent_(false)` fires only when something actually died. Binary/TNC
  payloads carry no identity and are NOT re-gridded — they still fail explicitly, and the TNC
  session layer owns their retry. Per-session scope: id counter and delivered-set cleared in
  `enterConnected()`, `enterDisconnected()` and `reset()`. Tests: 4 new deterministic
  regressions in test_connection_adaptive (re-grid delivers exactly once end-to-end; re-cut
  honours the NEW capacity on every resumed frame; the budget is bounded then fails closed with
  one attributed FAILED; receiver suppresses a duplicate by identity, delivers a distinct id
  with identical text, and handles the fragmented case) — the escape no longer depends on a
  seeded channel happening to demote. **Verification:** ctest 101/101; `tools/message_gui_matrix.sh`
  **10/10 PASS** (binary md5 `ccc7b4ed…`), every message row `msg_tx == msg_rx == msg_delivered`
  with `MESSAGE_EXACT_MATCH=1`, both file regressions `FILE_CRC_OK_COUNT=2`, and **zero**
  `msg_rx > msg_tx` (the attempt-1 duplicate signature). Crucially the matrix did not merely
  fail to reproduce the bug — **rows m2 and m3 exercised the whole path**: `Regridding 1 /
  abandoning 0 … QPSK R1/2 -> QPSK R1/4` → `Re-queued 1 message object(s)` → receiver
  `Dropping duplicate message object id=3 (97 bytes) — already delivered` (m3: `id=4, 50 bytes`),
  with `cmp` byte-identical text. That dropped duplicate IS the ambiguous fully-sent-awaiting-ACK
  case that made attempt 1 deliver twice. Zero `Abandoning active message` and zero `FAIL #`
  anywhere in the matrix. NOTE: m4/m8 — the two rows that failed originally — did NOT hit an
  escape this run (the documented non-determinism), so they are no-regression evidence only;
  the fix evidence is m2/m3 plus the deterministic unit regressions.

- 2026-07-14: BUG-MPG20-OVER-DEMOTE-R14 fixed — Good@20 read as Moderate → RX authority
  pinned QPSK R1/4 (operator: "R1/4 doesn't make sense"). **Root cause:** the Good/Moderate
  class boundary `kFadingGoodMax` sat at **0.65 — essentially ON the measured Good cluster
  center (0.62)**, giving a true-Good channel ~zero margin. On a dial-20 Good channel
  single-frame fading readings scatter to 0.74 (σ 0.129, 48-entry ledger), so the upper tail
  false-classified Moderate → the ladder's sparse Moderate column (R2/3 needs 20, R1/2 needs
  18 dB) → QPSK R1/4. The per-group RX-authority verdict (`updateRxAuthorityCommand`) used the
  RAW boundary and re-issued R1/4 every group (38× rate_hint=1 in one transfer); connect-time
  had a partial mitigation (`entryClassificationFadingIndex` σ/√N shrink) but the running
  verdict did not. **Fix:** `kFadingGoodMax` now = the maximum-likelihood midpoint of the Good
  (0.62) and Moderate (0.90) cluster centers = **0.76**, DERIVED not hand-tuned
  (`waveform_selection.hpp`). At the verdict SNR of a Good@20 channel (~17.5 dB after the +8.70
  legacy-anchor offset) the class flips Good→R2/3 instead of Moderate→R1/4. All duplicated 0.65
  class boundaries consolidated onto the single constant (connection_policy classifyChannel/
  fadingLabel/designDoppler, app.cpp fadingToQuality); the `isFading()`/`isHighThroughputOFDM`
  "significant-fading present"/conservative-window gates keep their own 0.65 (a different
  question than the class label). Cost-asymmetry backs the move up (false-Moderate = minutes of
  low-rung crawl; false-Good = one ~16 s group before the crater-demote self-corrects). Tests:
  test_connection_policy (classifyChannel(0.74)==GOOD, (0.78)==MODERATE + regression comment),
  test_waveform_policy — full ctest 87/87 green. Was independent of the keepalive, which only
  AMPLIFIED it by re-asserting the low rung (see 2026-07-14 keepalive revert).

- 2026-07-02: BUG-FILE-REQUEUE-OFFSET fixed — the live-ladder Moderate@20 data-loss root cause.
  `FileTransferController::requeuePendingChunks()` reconstructed the resume offset as
  `(chunks_acked_-1) * chunk_size_` — wrong the moment chunk_size_ has EVER changed mid-file
  (every mid-stream rate/mod move re-derives it). On the Moderate@20 gate cell the stuck-frame
  ESCAPE-drop (gate-bypassing by design) aborted 8 in-flight 16QAM chunks; the formula computed
  108×408=44064 where the true acked bytes were 34048, jumping the send cursor FORWARD 10016
  bytes (8 aborted chunks + 6.7 KB never sent). Reused seqs kept the receiver's ARQ space
  contiguous, the receiver-blind completion (count parity + cursor-at-EOF) declared done, and
  the auto-disconnect cancelled the receive — sender "Transfer complete", receiver stuck at
  expected=34048 with 16 buffered chunks. Fix: send-order ledger `tx_pending_ledger_`
  ({offset, metadata} per chunk handed to the ARQ, popped on retirement — retirement is strictly
  TX-base-order); requeue resumes exactly at `front().offset` for ANY chunk-size history;
  forward jumps impossible by construction. Companion receiver hardening (adversarial-review
  findings): `processFileData` tail-merges chunks that straddle the contiguous edge (a requeue
  resend on a CHANGED grid can start below the edge — the old whole-chunk drop lost the unseen
  tail forever), and the buffered-chunk drain is overlap-aware (covered entries drop, straddlers
  tail-append; the old exact-offset drain stranded covered entries and permanently blocked
  compressed finalization). `startSend` clears the ledger (SENDING→RECEIVING trample). Deferred
  structural sibling: BUG-FILE-ACK-IDENTITY (mixed-retirement trigger contained by
  one-logical-operation serialization; origin-tagged ARQ slots remain deferred, above). Proof: new
  test_file_transfer_controller cases (requeue-across-size-change reproduces 44064-vs-40 exactly;
  straddle-merge + covered-drain byte-exact CRC) + full 5-cell sequential gate PASS incl. the
  first-ever Moderate@20 pass (CRC-clean ×2, 1150 bps, 4 moves). See CHANGELOG 2026-07-02.

- 2026-07-02: BUG-DOPPLER-COHERENCE-MODECHANGE-WIPE fixed — the `ULTRA_RATE_ADAPT` precondition
  (2026-06-16 four-tier review): a mid-stream MODE_CHANGE could revert the Good/Moderate coherence
  verdict to the blind `fading_index` during the ~30 s re-pooling window. As-verified mechanics
  (2026-07-02 code read — the original "pool wiped every rate move" claim was PARTLY STALE): the
  decoder-hosted estimator pool + its atomics already survive `applyPendingConnectedOFDMMode`
  (waveform rebuild) and the pre-TX echo-clears (post 06-17/#67 `reset_doppler_coherence=false`
  paths); the remaining hole was that ANY modem-layer reset that does wipe the pool
  (`ModemEngine::reset`, future rebuild paths) immediately overwrote the Connection's valid verdict
  with invalid via the per-frame binding refresh. Fix per the named option: CARRY at the Connection
  layer — `Connection::setChannelCoherence` now holds the last VALID verdict while CONNECTED (the
  estimator is a cumulative mean and never un-validates on its own, so an invalid feed while
  connected can only mean "pool reset"), made safe by NEW per-connection clearing in
  `enterConnected()`/`reset()` (also fixes a latent cross-connection verdict leak). The mid-stream
  `requestModeChange` CW-count pick now routes through `coherenceAdjustedFadingIndex` (the
  CONNECT-time sites already did; remaining CONNECT-time raw-`fading_index` sites are provably
  identical there since coherence is invalid at CONNECT). Shipped with the fade-riding ladder
  default-ON (CHANGELOG 2026-07-02).
- 2026-05-13: BUG-PING-DETECTOR-001 fixed - real-HF PINGs now classify via additive chirp-lock plus LDPC-invalid PATH 2 while preserving the clean-cable/AWGN RMS-silence PATH 1.
- 2026-05-13: BUG-TNC-SESSION-001 fixed — R1 added the full RX decoder/session reset on disconnect; R2 added audio-producer quiesce/drain plus a reset-generation guard for in-flight decode callbacks, so a persistent `ultra_tnc` starts the next back-to-back PAT CONNECT from a fresh modem session boundary without stale capture backlog or stale cursor commits.
- 2026-05-08: BUG-CARRIER-LDPC-001 fixed — CarrierLDPC v1 was a new OFDM coded-bit wire image, but SP4 enabled the runtime as a local modem default instead of a negotiated peer capability. A partially upgraded Mac↔Pi pair applied the TX permutation on one endpoint while the peer decoded legacy ordering, producing the AWGN R1/2 1KB 0-ACK/15-timeout failure. The repair uses the existing `PHY_MASK_V1` capability: modern-modern CONNECT/CONNECT_ACK enables CarrierLDPC on both TX/RX; modern-legacy leaves the legacy ordering active. Synchronized upgraded hardware now passes AWGN / Good / Moderate at SNR=15 R1/2 1KB with DATA `Ncw=8` active and ACK/control `Ncw=1` inactive.
- 2026-05-08: BUG-BENCH-001 resolved — committed `fixtures/*.wav` are valid
  post-CONNECT DATA fixtures. They decode cleanly with
  `./build/decode_bench --mode bench --connected ...`; the earlier `0`-frame
  result came from running the bench in disconnected control-search mode.
- 2026-05-05: BUG-RATE-001 fixed — adaptive MODE_CHANGE panic-downshift on short Watterson-Good transfers. Hysteresis (`ADAPTIVE_PRESSURE_WINDOWS_FOR_DOWNGRADE = 2`) + lockout reduction (`ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS` 15 s → 5 s). 5-seed reproducer now 5/5 PASS with worst-case throughput improved 444 → 684 bps (no panic downgrade). See CHANGELOG 2026-05-05.
- 2026-02-12: GUI immediate TX abort control (`STOP TX`) added.
- 2026-02-12: GUI telemetry split into PHY vs effective goodput, plus ARQ health view.
- 2026-02-11: OTA control-path hardening and bootstrap safety updates.
- 2026-02-06 to 2026-02-10: ACK/control-frame decoding and ARQ robustness fixes.

For full details and commit-level history, use:
- `docs/CHANGELOG.md`

## BUG-COHERENCE-THRESHOLD-PLATFORM-BROKEN (open, 2026-07-25) — P1

**Symptom.** On the IONOS rig the RX-authority raw ladder pick oscillates between idx 8
(16QAM R2/3) and idx 3/2 (QPSK R2/3 / R1/2) on a **22-26 dB** channel — a 5-rung swing.
Measured across 3 A/B runs (both arms, so it is NOT the crater knob):

```
raw seq: 8 8 3 8 3        (OFF_2)
raw seq: 8 8 8 8 8 3 3 3 3 (OFF_3)
raw seq: 8 8 2 8 3 8      (ON_1)
```

**Every** low pick carries `fading=0.85` **exactly**; every idx-8 pick has fading <= 0.73.
The class boundary is `kFadingGoodMax = 0.76`.

**Root cause.** 0.85 is not a measurement — it is the literal constant
`kRepresentativeModerateFadingIndex` (`connection_policy.hpp:318`), returned by
`coherenceAdjustedFadingIndex()` when `coherence_score <= kCoherenceModerateThreshold`
(0.30). That threshold is **SIM-scale**, and the code comment directly above it already
says so:

> "The legacy lag-1 score above is SIM-calibrated and platform-broken (needs ~0.045 on
> the IONOS rig, ~0.30 on sim — a flat re-base would break sim)."

Every coherence score observed on the rig is <= 0.11 (measured: 0.00, 0.08, 0.09, 0.11,
-0.11, -0.15, -0.18, -0.23, -0.27, -0.29). So on this platform
`score <= 0.30` is **always true** once the estimator goes valid, and the channel is
pinned to the representative-Moderate fading index for the rest of the transfer. The
Good column of the anchor table becomes unreachable and the ladder drops ~5 rungs on a
channel whose own SNR estimate says 22-26 dB.

This is the same family as BUG-MPG20-OVER-DEMOTE-R14 (a class-boundary defect starving
the rate ladder), but the mechanism is different: there the boundary constant was
mis-placed; here a sim-calibrated THRESHOLD is applied to rig-scale data, so the override
fires unconditionally.

**Why it was invisible.** The verdict log printed the PRE-clamp pick's modulation next to
the POST-clamp index, so the trajectory looked incoherent rather than bimodal. The `raw=`
field added 2026-07-25 (same-day CHANGELOG entry) is what exposed it.

**Do NOT fix by re-tuning the threshold.** The comment is right that a flat re-base breaks
sim; a per-platform constant is exactly the bench constant the ADAPTIVITY rule forbids.
The fix must be a statistic that is scale-invariant BY CONSTRUCTION. Candidate already
built and rig-validated: the demeaned |H|^2 frequency-autocorrelation discriminator
(`src/ofdm/frequency_selectivity.hpp`, `test_frequency_selectivity.cpp`), whose decision
threshold is 0 with a null sigma of 1/sqrt(N-L) — a probability constant, not a bench
constant. Rig validation: MPM 18/20 MODERATE and 0/20 GOOD; MPG 16/19 Good-ish with
per-frame Good->MODERATE 0/706.

**Also note** `coherenceArea` is NOT a valid substitute here (d' ~3.0, 12.5%
Moderate-read-as-Good; a passing Good run logged area=-0.461). See
project_channel_class_discriminator_2026_07_25.

**Cost NOT yet established — an initial claim here was overstated and is retracted.**
Whenever it fires the ladder loses ~5 rungs (in OFF_3 it held for the last 4 of 9
verdicts; in OFF_2, 2 of 5), so the per-event cost is real. But a dose-response test over
8 interleaved runs does NOT support it being the dominant throughput driver:

| pins (fading=0.85 verdicts) | mean bps | n |
|---|---|---|
| <= 1 | 1567 | 5 |
| >= 2 | 1197 | 3 |

Pearson r(pins, bps) = **-0.40** (n=8, not significant, p ~0.32), and the WORST run of the
batch (OFF_6, 823 bps) had **zero** pins — which alone refutes "pin count drives
throughput". The naive <=1 vs >=2 split suggests ~24% but is confounded by exactly that
run. So: the defect is proven (0.85 is a literal constant, not a measurement, and the
threshold is documented as platform-broken), the per-event rung loss is proven, and the
aggregate throughput cost is UNMEASURED. Establish it with a proper interleaved A/B once a
scale-invariant discriminator exists to A/B against — do not budget a gain from it before
then.

## BUG-LADDER-SNR-DB-MEAN: the rate ladder's SNR ring averages dB where the dial and anchors are mean-power (open, 2026-07-26) — P1

**STATUS 2026-07-26: FIX IMPLEMENTED behind `ULTRA_LINEAR_SNR_RING` (default-OFF) — rig A/B
measured a WASH** (4 pairs: paired −2.5% mean, sign test p=0.688; churn 6.2 vs 6.8/run;
completion 3/4 vs 4/4). The compensation sizing WAS confirmed correct (matched pair: ladder
input snr_avg 23.7 ON vs 23.6 OFF, i.e. net neutral as designed). The bug below is therefore
REAL and FIXED-IN-CODE but NOT ENABLED, because a correct change with no measurable benefit
does not get switched on. Its remaining value is that the **anchor re-measure must be run in
the correct domain** — calibrating per-rung floors against a dB-domain mean would bake this
same Jensen error into the anchor table. See CHANGELOG 2026-07-26.

**Defect.** Two SNR rings exist in this codebase and they disagree on domain:

| site | code | domain |
|---|---|---|
| `connection.cpp:2500` (**feeds the rate ladder**) | `snr_sum += rx_auth_obs_db_[i]` … `/ snr_n` | **dB-domain mean** |
| `streaming_decoder.hpp:465-470` (`physicalSnrStats`) | `lin_sum += pow(10, db/10)` … `10*log10(lin_sum/n)` | **linear-power mean** |

The decoder is right and says so in its own comment ("mean is linear-domain (fade-averaged)"). The
ladder input is the one that averages dB. Both the IONOS dial and the anchor table are **mean-power**
definitions (`tests/test_ofdm_snr_calibration.cpp` gates on the linear/power mean), so by Jensen's
inequality the ladder is fed a systematically LOW number on any fading channel — the more fade
variance, the larger the error.

**Measured cost: 1.74 dB** on the 2026-07-26 MPG@20 OFF arm (per-frame dB-mean 13.89 vs
power-mean 16.42 over 1023 readings; the ring itself read 14.68). On the 2026-07-24 epoch the same
comparison gives **2.76 dB** (13 vs 1309 readings). This is the single largest identified term in the
~5.4 dB sim-vs-rig SNR discrepancy — see `docs/ANALOG_CHAIN_VERIFIED_2026_07_26.md` §4 for the full
decomposition, in which measured ANALOG loss is only 0.36 dB.

**⚠ THE FIX IS COUPLED — do not land it alone.** Making the ring linear RAISES the ladder's input by
~1.7-2.8 dB. The ladder is already measured to OVER-COMMIT by ~2 rungs on this bench (2026-07-26
offset A/B: dropping `kOfdmLegacyAnchorScaleOffsetDb` from 8.70 to 3.1 cut rung churn 65% and craters
60%). So a lone domain fix makes over-commit WORSE. It must land together with a ~2.5 dB reduction of
`kOfdmLegacyAnchorScaleOffsetDb` (`connection_policy.hpp:43`), or preferably as part of the §3 anchor
re-measure that deletes that constant outright.

**This also reinterprets last night's result.** The (3.1, 8.70) bracket was described as bounding
"~5.6 dB of implementation loss". Part of that is now identified as a DOMAIN ERROR in our own
averaging, not implementation loss at all.

**Secondary defects found alongside it (same investigation):**
- **Stale ring re-feed:** 3 of 32 verdicts carry an `inst=` identical to the previous verdict
  (`OFF_6_mac.log` ends with `inst=29.4` four times while `snr_avg` walks 25.1 → 28.0). The ring is
  re-fed without a fresh observation. Fix: gate on a fresh-observation sequence number — the
  machinery already exists for the level verdict at `modem_protocol_binding.hpp:118-121`.
- **Calibration grades the wrong branch:** the fading gate covers **QPSK R1/4 only**
  (`test_ofdm_snr_calibration.cpp:154-155`), while the rig runs QPSK R1/2·R2/3·R3/4, 8PSK R2/3 and
  16QAM R1/2·R2/3 — different pilot spacing. Worth 0.5-1.1 dB of the discrepancy purely as a
  reference mismatch. Add a rung sweep.
- **Doc bug (not code):** `rx_filter_` is constructed at `modem_engine.cpp:325` and reset at `:332`
  and referenced NOWHERE else — only `tx_filter_` is applied (`:763`). CLAUDE.md's "in-band RMS
  0.3048 (after 101-tap 50-2950 Hz RX FIR)" therefore describes a **noise-referencing convention**,
  not a filter in the RX signal path. Corrected in CLAUDE.md.
