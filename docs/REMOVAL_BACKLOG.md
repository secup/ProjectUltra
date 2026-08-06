# Removal Backlog (the demolition list)

A single tracked list of code / features / experiments **slated for deletion** —
superseded models, failed experiments, dead paths. This is the *action* list for
removals; it complements (does not duplicate) the broader cleanup register in
`docs/MODEM_INFRASTRUCTURE_MAP.md §7`, which also covers *consolidate / rename /
codify* work that is **not** deletion.

**How to use:** add an entry the moment something is decided-dead (don't let it rot
silently). Each entry states **what**, **why dead**, **scope** (exactly what gets
deleted), **KEEP** (what must NOT be touched — the anti-footgun), **blocker**, and
**status**. Verify file:line against the infra map before cutting — a wrong "dead"
here causes a wrong deletion later. Move finished items to *Completed* with the commit.

**Status:** `QUEUED` (decided, not started) · `BLOCKED` (needs X first) · `IN-PROGRESS` · `DONE`.

---

## Decided removals (architecture direction — confirmed)

### R1. Legacy OFDM-wideband **file routing** (NOT "SR-ARQ") — `SUPERSEDED 2026-06-06 by the Transport Merge — see R1b`

> **2026-06-06 — REVERSED by the Transport Merge (R1b).** This entry wanted to KEEP
> `burst_transport_` (the `BurstStopAndWaitController` group controller) and DELETE the
> wideband-file routing through `arq_`. The Transport Merge inverts that decision: the
> **unified `arq_` path is now THE keeper** (one 16-bit seq space, one tone-burst ack, one
> retransmit window — it still bursts+interleaves via `encodeBurstLight`), and the
> **`burst_transport_` controller is the legacy to delete.** So R1's "scope (delete)" below is
> VOID (those `arq_`-file branches survive as the unified path). The live deletion is **R1b**.
> R1 kept here only for history.

> **2026-06-02 — env gate REMOVED (user directive).** Per the user ("burst transport is the
> only valid way to transfer files now … we shouldn't gate it"), the `ULTRA_BURST_TRANSPORT`
> opt-out was deleted (all 3 env reads: `connection.cpp:353`, `app.cpp:593`,
> `modem_engine.hpp`) and the inconsistent RX default `burst_transport_rx_` flipped
> `false→true`. **This SUPERSEDES the BLOCKED rationale below** — the "keep the `=0` fallback
> until burst is throughput-proven post-ladder-rework" caution is overridden by the product
> decision that burst is the only file method. Motivating bug fixed: `ultra_tnc` owns a raw
> `StreamingDecoder` and never enabled burst-RX, so it transmitted bursts (TX default ON) it
> couldn't decode (RX default was OFF) → TNC file transfer failed; data-burst sync went
> reject@0.25 → accept@0.99 after the flip. **Remaining R1 scope:** delete the now-DEAD
> `!use_burst_transport_` windowed-file branches + the `use_burst_transport_` member (kept as
> dead code for now so the low-SNR fallback stays in-tree until burst is floor-proven
> post-ladder; delete in a focused follow-up). KEEP section below still holds.

- **What:** the legacy path that sends a wideband OFDM **file** through the continuous
  windowed `SelectiveRepeatARQ arq_` instead of the burst transport.
- **Why dead:** burst transport (`BurstStopAndWaitController burst_transport_`) is THE
  OFDM-wideband file method going forward (default ON since 2026-05-30, commit `c40a0b5`).
  We are not going back to the windowed-file model.
- **Scope (delete):** the `!use_burst_transport_` branches in `connection.cpp`
  (file-routing sites ~`1632`/`1707`/`1855`/`2327`/`2366`/`2394`/`2759`), the
  `use_burst_transport_` member + its `false` default, and the `ULTRA_BURST_TRANSPORT`
  opt-out knob (`connection.cpp:353`, `app.cpp:593`, `modem_engine.hpp:432`). Make burst
  unconditional for OFDM-wideband file transfer.
- **⚠ KEEP (do not over-cut):**
  - **`SelectiveRepeatARQ arq_` stays** — it still serves **MC-DPSK data, OFDM_NARROW
    data, and ALL control ACKs**. Only the *wideband-file routing through it* is removed.
  - **Burst is itself selective-repeat** (GROUP_ACK 6-bit SACK `frame_mask`,
    resend-failed-only + refill). This is NOT a "remove SR-ARQ" task — SR semantics are
    shared by both controllers. Do not rip out SACK / selective-repeat machinery.
- **Blocker:** the auto rate ladder is mid-rework (floors not re-established), so the
  burst default path is not yet throughput-proven end-to-end on the GUI gate. Keep the
  `=0` fallback until burst is proven post-ladder-rework, THEN delete the legacy routing
  + knob. (Tracking: the burst-default flip itself already shipped regression-free.)

### R1b. Legacy `burst_transport_` group-burst controller (Transport Merge) — `DONE 2026-06-06` (unified is the default; controller deleted)

> **DONE 2026-06-06 (uncommitted, NOT pushed).** Flipped `kUnifiedSeqEnabled()`/
> `kInteractiveToneAckEnabled()` (connection.cpp) + the two modem-side reads (`modem_engine.cpp`,
> `streaming_burst_interleave.cpp`) to unconditional → unified is the default, no env needed.
> Deleted (~1000 lines from connection.cpp): `startBurstFileTransfer`, `formAndSendBurstGroup`,
> `formAndSendBurstGroupSR`, `formOneNewBurstFrame`, `onBurstGroupReceivedSR`, `collectBurstGroupFrame`,
> the 4 `burst_transport_` callbacks, the legacy `onBurstGroupReceived`/`onToneBurstAck` branches, the
> GROUP_ACK/NACK switch cases + their `setAckTimeoutMs`/`tick` plumbing, the route fork in
> `startFileTransferNow`, the `BurstStopAndWaitController burst_transport_` member, and the whole
> `src/protocol/burst_transport.hpp` + `tests/test_burst_transport.cpp` (+ CMake). Wired
> `applyAdaptiveRateFeedback` into the unified `onToneBurstAck` branch so rate adaptation survives.
> Removed the 4 in-process `SimulatedChannel` file/binary-send tests from `test_protocol.cpp` (they
> can't carry the modem burst path; file transfer is gated on the GUI/OTASim path). KEPT `arq_`,
> `encodeBurstLight`/BURST_HEADER, `burst_transport_rx_`, `flushBurstBuffer`, `transmitFrameBatch`,
> tone-burst ack, and `onAcceptedOFDMDataSync` (made unconditional). **FOLLOW-UPS:** (1) `use_burst_transport_`
> bool is now always-true — collapse to a constant / inline its gates; (2) orphaned TX members
> (`burst_resend_frames_`, `burst_inflight_frames_`/`_is_pad_`, `burst_file_payload_`,
> `burst_rx_group_frames_`, etc.) are now unused — sweep them.

> **Supersedes R1 (reversed).** The Transport Merge unifies the 3 SR-ARQ-over-OFDM transports
> (interactive SR-ARQ / SR-on-burst / burst-file) onto ONE path: the unified `arq_` selective-repeat
> window, where "burst" is just TX framing (`encodeBurstLight` + BURST_HEADER descriptor) + RX group
> assembly that `arq_` drives. The separate `BurstStopAndWaitController burst_transport_` becomes a
> SECOND, redundant way to form/sequence/ack a group → delete it so there is exactly ONE group-gen path.

- **What:** the legacy OFDM-wideband file/group transport built on `BurstStopAndWaitController
  burst_transport_` — a separate group seq space, group-level stop-and-wait, GROUP_ACK/GROUP_NACK
  control frames, and its own TX group-formation (`formAndSendBurstGroup` z=81 / `formAndSendBurstGroupSR`).
- **Why dead:** the unified `arq_` path (`sendNextFileChunk`/`sendNextFragment` → `flushBurstBuffer` →
  `transmitFrameBatch` → `encodeBurstLight`, RX `onBurstGroupReceived` unified branch → `processArqFrame`
  → `endGroupReceiveAndAck` → tone-burst ack) does the same bursting+interleaving with ONE seq space,
  one ack, one retransmit window. Half-duplex stop-and-wait + coalesced `[holes]+[new]` resends live in
  `arq_` now (`retransmitInFlightUnacked` + `prepareUnifiedBurstWindow`). Validated CRC-clean AWGN
  (R1/2,R3/4) + Good@15 (R1/2,R2/3); multi-seed proof = `/tmp/unified_multiseed.sh` (gate for the flip).
- **Scope (delete, VERIFY file:line first — connection.cpp/.hpp unless noted):**
  - TX group-formation: `startBurstFileTransfer()`, `formAndSendBurstGroup()`, `formAndSendBurstGroupSR()`.
  - RX: `onBurstGroupReceivedSR()`; the LEGACY (`!kUnifiedSeqEnabled()`) branch of `onBurstGroupReceived()`
    (the `interleaved`/`all_ok`/fast-NACK-control-frame block at ~2540–2682) — KEEP the unified branch.
  - The `BurstStopAndWaitController burst_transport_` member + `use_burst_transport_` member/default + all
    its callback wiring (`setTransmitGroup`/`setSendGroupAck`/`setGroupDelivered`/`setTransferDone`/
    `setFormAndSendGroup`/`setAckTimeoutMs`/`startTransfer`/`onGroupAck`/`onGroupNack`/`onGroupReceived`/
    `rxExpectedGroupSeq`/`groupsAcked`/`ackTimeoutMs`/`tick`) in connection.cpp.
  - The `!use_burst_transport_` / `use_burst_transport_` route forks (incl. `startFileTransferNow`
    `&& !kUnifiedSeqEnabled()` branch at ~1755, and the GROUP_ACK/GROUP_NACK handlers ~3158–3182, ~3298).
  - GROUP_ACK / GROUP_NACK control frames (`ControlFrame::makeGroupNack`/`makeGroupAck`) IF no other caller.
  - `src/protocol/burst_transport.hpp` (the header-only `BurstStopAndWaitController` class) once unreferenced.
  - The `kUnifiedSeqEnabled()` / `kInteractiveToneAckEnabled()` env gates themselves (make unconditional)
    + the 3 env reads (`connection.cpp:34`, `modem_engine.cpp:537`, `streaming_burst_interleave.cpp:195`).
- **⚠ KEEP (do not over-cut — anti-footgun):**
  - **`SelectiveRepeatARQ arq_` stays** — the unified keeper; also still serves MC-DPSK data, OFDM_NARROW
    data, and ALL control ACKs. This is NOT "remove SR-ARQ".
  - **`encodeBurstLight` + BURST_HEADER descriptor** (self-describing group_size/cw/mod/rate/z) — the ONE
    group TX framing. **`burst_transport_rx_`** (modem RX group collector) + `flushBurstBuffer` +
    `transmitFrameBatch` + `onBurstGroupReceived` **unified branch** + tone-burst ack — all stay.
  - **`onAcceptedOFDMDataSync()` rescue-disarm is LOAD-BEARING for the unified path** (verified 2026-06-06:
    bravo log line 452 "Accepted OFDM DATA sync … disarming CONNECT_ACK rescue (burst transport)"). It is
    gated `if (use_burst_transport_)` (connection.cpp:3300). When `use_burst_transport_` is deleted, make
    this disarm **UNCONDITIONAL** — do NOT delete it. Without it the responder keeps blasting an 8.3 s
    CONNECT_ACK rescue INTO the initiator's in-flight burst (half-duplex collision → group-0 ACK latency).
  - The responder handshake-confirm rides `Connection::onFrameReceived` (connection.cpp:3049, "received
    first valid frame from initiator"), NOT the legacy `onBurstGroupReceived` branches — so deleting those
    branches does not lose it. Keep `onFrameReceived` on the RX path (re-verify on the post-delete gate).
  - Burst is itself selective-repeat — SACK/`frame_mask`/resend-failed-only semantics are shared; don't
    rip them out.
- **Blocker:** unified must be the DEFAULT (gate flip) and pass the multi-seed proof first. Sequence:
  (1) prove `/tmp/unified_multiseed.sh` green → (2) make unified unconditional → (3) delete legacy above
  → re-gate. Doing the delete before (1) is the "single-cell pass ≠ proof" trap.
- **Design:** `docs/TRANSPORT_MERGE_DESIGN_2026_06_06.md`.

### R2. Operator chat-message (free-text) feature — `RESCINDED / RESTORED 2026-08-04`
- **What:** interactive operator free-text messaging remains a supported application path.
  The GUI compose row, exact TX lifecycle reporting, fragmentation/reassembly regressions,
  and faithful `gui_qso_scenario.sh --message-only` automation were restored on 2026-08-04.
  `ProtocolEngine::sendMessage` / `sendMessages` and the message callbacks are KEEP, not
  removal targets; the TNC also depends on the shared non-file payload machinery.
- **Why the 2026-05-30 removal decision was reversed:** long-LDPC bulk framing made the old
  chat UX unattractive, but that did not make short application messages dead. Low-throughput
  profiles that are impractical for a file can still carry a useful message, and the protocol
  must fragment longer text safely instead of deleting the use case. The restored design
  serializes message/binary/file logical operations and keeps the GUI compose limit at 255
  bytes, so message integrity no longer depends on fitting one LDPC/frame geometry.
- **Historical action (2026-06-02, commits `95982c0` + `b3835a9`):** the old chat tests were
  removed while R2 was considered dead. That history remains valid, but it is no longer a
  forward plan. Current message/turn/geometry coverage is documented in the 2026-08-04
  CHANGELOG entry and `tests/test_protocol.cpp` / `tests/test_connection_adaptive.cpp`.
- **⚠ KEEP:** application-message APIs and callbacks, text and binary fragmentation/
  reassembly, message TX status, GUI/TNC wiring, and the separate file-transfer path. A future
  cleanup may consolidate shared fragment code only with byte-exact text/binary/file and
  half-duplex turn regressions in place; it must not revive this removal item.

### R3. Differential on the OFDM band — `DONE` (selection + RX demod/control code removal); TX-modulator differential is the remaining follow-up
- **What:** retire differential modulation from the **wideband OFDM_CHIRP** band. SNR ≥ 10
  (AWGN/Good/Moderate) → coherent QPSK; Poor → MC-DPSK.
- **Why:** mode selection routes SNR<10 / Poor → MC-DPSK (differential, its real home) and SNR≥10 →
  OFDM. Coherent beats differential across the OFDM band — GUI clean-rate **81% Good@10 / 89-90%
  Moderate@14 (2 seeds) vs differential 32%**, `max_iters` flat 1–6 as fading sped up. Full rationale:
  `docs/OFDM_COHERENT_ONLY_DECISION_2026_05_31.md`.
- **DONE (commit `4c72a51`, thread A2):** the *selection* of differential on OFDM_CHIRP is removed —
  `recommendDataMode` OFDM default DQPSK→QPSK, D8PSK rungs deleted, Poor routes to MC-DPSK
  (`kOFDMEntryFloorPoorDb` unreachable), OFDM_CHIRP ladder rung mod→QPSK, policy tests updated. The
  bug-causing coherent-vs-differential *ambiguity* on OFDM_CHIRP is GONE (no path forks on it).
- **✅ DONE — RX demod/control code removed (2026-05-31, commits `9b20d91` + `469ee8b` + `19f3df8`):**
  the disable-narrow-then-remove plan executed in full.
  1. ✅ **Disabled** OFDM_NARROW (`d490524`, dropped from `ModeCapabilities::ALL`) so no OFDM path is DQPSK.
  2. ✅ **Made differential impossible:** `isSupportedChirpModulation` rejects DBPSK/DQPSK/D8PSK and
     `configure()` falls back to QPSK (was DQPSK) → `config.modulation` for any OFDM waveform is never
     differential → `is_differential` provably false on every demod/channel-est path.
  3. ✅ **Removed** the now-dead differential demod/control: the DBPSK/DQPSK/D8PSK demap cases + DD phase
     tracking + `demodulateD8PSKTwoPass`/`demodulateDQPSKTwoPass` + callerless `computeFadingIndex`
     (`ofdm_symbol_demap.cpp`); the `is_differential` MMSE early-return (`channel_equalizer_equalize.cpp`);
     all `is_differential` branches in `updateChannelEstimate` (`channel_equalizer_pilot.cpp`); the LTS
     check + `dbpsk_prev_equalized`/`differential_prev_erased_` clear sites (`ofdm_stream_processor.cpp`);
     the differential members (`demodulator_impl.hpp`/`ofdm_demodulator_setup.cpp`); and the
     `profileForDataMode(DQPSK)→DQPSK` control switch + `coherent_ofdm_control_profile_enabled_` flag.
     **KEPT** carrier-LDPC, the coherent `dd_qam16` tracker, MC-DPSK, the `Modulation` enum, and
     `soft_demap`'s differential inline helpers.
- **✅ DONE — TX + RX-LTS differential removed (2026-05-31, commits `65b27b6` + `2f3c2ce`):** the OFDM
  **TX** differential encoder (`modulator.cpp`, DBPSK/DQPSK/D8PSK branches + `dbpsk_prev_symbols`) was
  deleted (the only caller passes `config_.modulation`, never differential; MC-DPSK uses its own
  modulator). The RX twin — the dead `lts_carrier_phases` / `lts_phase_offset` / `phase_advance` DQPSK
  reference computation in `estimateChannelFromLTS` (write-only after A3) — was deleted too. The OFDM
  TX+RX path is now differential-free end-to-end.
- **▶ LATER — revamp OFDM_NARROW as COHERENT** (reuses the coherent OFDM machinery with a narrowband
  config; no differential code needed). **⚠ real PHY re-validation, NOT a config flip:** narrowband
  ~17 dB / 500 Hz is where differential's no-phase-reference robustness is the point, so coherent narrow
  may land at a HIGHER SNR floor (fewer carriers to track phase across). Don't assume it reaches the
  differential floor.
- **⚠ KEEP regardless:** MC-DPSK differential (`multi_carrier_dpsk.hpp`); the `Modulation` enum
  `DQPSK/DBPSK/D8PSK`; the **COHERENT** DD tracker `dd_qam16_*` (`channel_equalizer_equalize.cpp:646`).

### R4. Adaptive 100 ms short data re-anchor (§16.2/§16.4 superseded) — `DONE` (gui/modem subsystem)
- **DONE 2026-05-31:** deleted the whole gui/modem short-reanchor subsystem (−379 lines, 14 files):
  the search path (`streaming_sync_acquisition`), `SyncController::planWarmSearch` short-lead params,
  the encoder light/short preamble branch + `adaptive_short_data_preamble_`, the decoder member, the
  engine `syncAdaptiveShortDataPreamble`/`setAdaptivePreamblePeerFading`/members, app.cpp call, the 3
  waveform virtuals (`generateShortDataPreamble`/`detectShortDataSync`/`getShortDataPreambleSamples`)
  + ofdm impls + `shortReanchorSync`/`getShortReanchorChirpConfig`/members, `ultra_tnc.cpp`'s mirror,
  and the `adaptive_reanchor_policy.hpp` file. Build clean; ctest red-set byte-identical to baseline.
- **⚠ FOLLOW-UP (separate subsystem, NOT done):** `connection.cpp` still has
  `connection_policy::shouldUseWideOFDMShortReanchor` driving ARQ SACK/continuation-reanchor TIMING
  (`connection.cpp:4238`). It is independent of the deleted preamble machinery (still builds), but
  since the modem no longer EMITS a short re-anchor preamble, its airtime budget may now be stale —
  review whether `continuation_reanchor_ms` should go to 0 post-warm-handoff. Load-bearing ARQ timing;
  needs its own analysis + GUI gate, NOT a mechanical delete.

### R4-historical. (original scope, now done — kept for reference)
- **What:** the adaptive short-chirp re-anchor group-boundary strategy (the "§16.2 short re-anchor
  that broke frame-stride timing").
- **Why dead:** mutually exclusive with the warm-sync hand-off, which is now the PRODUCTION DEFAULT
  (§16.8, flag removed 2026-05-31, commit `5535fd2`). `ModemEngine::syncAdaptiveShortDataPreamble`
  (`modem_mode.cpp`) now FORCES `enable = false` unconditionally — warm-handoff owns the group
  boundary via the descriptor chirp + warm-light path — so `adaptive_short_data_preamble_` is always
  false and the entire short-reanchor search/preamble path is unreachable.
- **Scope (delete, VERIFY file:line first):** `adaptive_reanchor_policy::shouldUseShortReanchor` +
  `shortReanchorChirpDurationMs` (if no other live caller); the `adaptive_short_data_preamble_` /
  `adaptive_short_reanchor_active_` / `adaptive_preamble_peer_fading_` members + `setAdaptiveShort
  DataPreamble` / `setAdaptivePreamblePeerFading` plumbing (encoder + decoder); the
  `use_short_reanchor_search` / `detectShortDataSync` / `getShortDataPreambleSamples` /
  `short_reanchor_lead_samples` paths in `streaming_sync_acquisition.cpp` + the waveform; the
  now-dead-store `shouldUseShortReanchor` call in `modem_mode.cpp` (replace with `bool enable=false`).
- **⚠ KEEP (do not over-cut):** the FULL chirp+LTS preamble (`generatePreamble`); the warm-handoff
  LIGHT group-start preamble; the BURST_HEADER descriptor chirp anchor — these REPLACE the short
  re-anchor, they are not part of the deletion.
- **Blocker:** none — warm-handoff is default + GUI-proven. Mechanical dead-path removal.
- **Status:** QUEUED. Gate on ctest byte-identical + GUI floor/Good@12.

---

### R12. Retired rate-control experiments + the dead predictive-climb predictor — `QUEUED 2026-08-01`

Six knob-guarded experiments whose target code is gone or whose result is closed. All
default-off; none is reachable on the v0.5.1 default path.

**Scope (delete):**
- `rungPredictedSustainable()` (`waveform_selection.hpp`) and its two anchor-selection knobs
  `ULTRA_RUNG_CLASS_ANCHOR` / `ULTRA_FER_FLOOR_ANCHOR`. Its ONLY production caller was the
  RX-authority predictive climb, retired 2026-08-01; the sole remaining references are in
  `tests/test_rx_authority.cpp`. Delete those tests with it.
- `goodput_rate_controller.hpp` + `tests/test_goodput_rate_controller.cpp` +
  `ULTRA_GOODPUT_RATE` and its block in `updateRxAuthorityCommand`. Superseded by
  `ULTRA_LATENT_RATE`, which shipped the same thesis (value estimate for EVERY rung,
  stateless argmax, stability in the estimator) and measured +14.5% p=0.022 against this
  one's +8.0% n=3 wash.
- `ULTRA_TRUST_LADDER_PICK` block. Superseded: its hypothesis WON but the latent controller
  is the better expression of it.
- `ULTRA_RUNG_DWELL_MS` block + `rx_auth_last_change_*` members. FALSIFIED on the rig
  (−13.8%, 1/4 positive); its premise was reverse causation.
- `ULTRA_ADAPTIVE_RTO` + the `adaptive_floor` parameter on `updateRTO`. Hypothesis RETRACTED:
  the adaptive floor moves the RTO 0-20%, not 7.5x, because measured srtt (~11-14 s) already
  sits at the configured value.
- `ULTRA_CONNECT_ACK_RESCUE_DEFER` block. **Removed 2026-08-03.** Its proactive full-ACK
  timer was falsified (3/3 handshake deadlocks vs ~1-in-20); authoritative confirmation plus
  cache-only reactive duplicate-CONNECT replay now closes the underlying defect without a knob.

**KEEP — the anti-footgun list. Do NOT over-cut:**
- `snapRungIndexDownToEnabled` and every call site. F145 is a REAL 50 s deadlock (commit
  `63358e6`): disabled anchor rows are HOLES and raw index arithmetic walks into them.
- `kMeasuredFerFloor` / `rungFerFloorDb` (`waveform_selection.hpp`). Real data — 9 rungs x 3
  classes x SNR 6-28 x seeds {7,11}, n=80/point — pinned by `test_waveform_policy.cpp`. It is
  the measurement a per-class `theta_r` should be fitted from, which is the latent
  controller's known next gap (its table is ITU-Good-only).
- `rungClassAnchorDb` — it has a SECOND caller in `connection.cpp` beyond the deleted knob.
- `updateRTO` itself and the RFC6298 estimator: only the `adaptive_floor` parameter goes.
- Keep the CONNECT_ACK half-open incident record. The defect is code-closed by the 2026-08-03
  authoritative-confirmation and cross-mode reactive replay design, but its fresh IONOS
  end-to-end validation is still pending.

**Blocker:** none. All six are unreachable on the default path.

**Explicitly NOT in scope:** the legacy SNR-anchor ladder and its corrective stack. That is
the `ULTRA_LATENT_RATE=0` escape hatch, deliberately retained for 0.5.1 so the default flip
has a fallback; it carries the last `kOfdmLegacyAnchorScaleOffsetDb` reference and dies in
0.5.2 after field exposure.

### R13. DATA-turn grant machinery, once the handover is fixed — `BLOCKED 2026-08-06` (written BEFORE the fix, deliberately)

**Why this exists now.** BUG-TURNOVER-GRANT-LOST-IN-REQUESTER-ECHO will be fixed in one of two
shapes, and BOTH leave debris. Filing the demolition plan before writing the fix so the
interim scaffolding cannot quietly become permanent — the failure mode is N default-off knobs
and two mechanisms doing one job.

**Measured baseline the fix must beat** (IONOS n=8, 2026-08-06): 121 grants transmitted / 7
decoded; handover 41–176 s; direction needing no handover 8/8 delivered vs 6/8 for the
direction requiring one. Keep this table until the replacement is proven against it.

**Scope (delete) — IF the interim "grant retransmit timer" ships first:**
- the whole retransmit scaffolding (timer field, retry counter, its max-retries constant) and
  its default-off knob, the MOMENT the piggyback version below lands. Two mechanisms granting
  the turn is strictly worse than either alone.
- the reactive re-grant in `handleTurnRequest()` (`connection_handlers.cpp` ~1007-1018,
  "RX TURN_REQUEST … reasserting TURNOVER"): with a sender-side timer OR a piggybacked grant,
  re-granting on a repeated request is a second recovery path for the same loss. Keep exactly
  one.

**Scope (delete) — IF/WHEN the grant is piggybacked on the last DATA frame:**
- the dedicated grant transmission on the *common* path (ISS yields while it still has a final
  frame to stamp), i.e. the `makeTurnover()` emit inside `maybeYieldDataTurn()` for that case.
- any grant-retransmit machinery from the interim step (see above).
- whichever request-retry slack becomes unnecessary once grants stop being independently
  loseable — re-derive `turn_request_retransmit_ms_` at that point rather than assuming.

**KEEP — the anti-footgun. Do NOT delete `TURNOVER` as a frame type or its handler.**
- `ControlFrame::makeTurnover()` / `handleTurnover()` must survive: there is a real case with
  **no data frame to piggyback on** — the ISS yielding when it has nothing queued (see
  `only_unstarted_file_waiting` in `maybeYieldDataTurn`) and the initiator's proactive
  post-connect yield (~1.5 s after CONNECT, the half-duplex INTERACTIVE/TNC path). A
  piggyback-only design would strand both.
- `Flags::TURN_REQUEST` on the SACK (`selective_repeat_arq.cpp:2315`,
  `connection.cpp:6013`) — this is the REQUEST half, it already piggybacks, and it MEASURES
  FINE. It is the model the grant should copy, not something to touch.
- `PHYSICAL_BURST_END` and `FINAL` keep their current distinct meanings (physical burst
  envelope vs logical transfer boundary). A grant bit is a THIRD meaning — do not overload
  either of them to save a bit.

**Blocker.** The mechanism is not yet isolated: on-air collision vs the requester's receiver
discarding audio while it catches up (`LOAD-SHED … fell behind live`, and it decodes its own
TURN_REQUEST in 60–76% of transmissions). If the cause is receiver-side, NEITHER fix above
helps and this entry's scope changes entirely. Settle that first (contended scenario,
`ULTRA_WARM_TURNAROUND_OFF=1` vs default, interleaved).

**UPDATE 2026-08-06 — the interim fix EXISTS and MEASURED NET-NEGATIVE.** `ULTRA_TURNOVER_REPEAT`
(`7432650`) is implemented, unit-tested and default-OFF. Its A/B improved every mechanism
metric (handover 43.5→19.1 s, landed 3/3 vs 2/3, 4× fewer requests) but **reduced completed
return transfers 2/3 → 1/3**, because re-asserts land after the peer has taken the turn and
interfere with its data (`grants_rx=3` observed). See KNOWN_BUGS for the full table.

So this scope entry is now LIVE, not hypothetical:
- If the repeat is not repaired (energy-gated stop condition) within a couple of sessions,
  **delete it** — knob, timer, counter, the three arm sites and the tick hook. A default-off
  knob that measured net-negative is exactly the dead path this list exists to prevent.
- If it IS repaired and ships, it still gets deleted when a piggybacked grant supersedes it,
  per the scope above.

**Status:** `QUEUED` — either repair with an energy-gated stop condition, or delete. Do not
leave it sitting default-off indefinitely.

---

## Dead code — audit-confirmed (verify no test-tool dependency, then cut)

Sourced from `MODEM_INFRASTRUCTURE_MAP.md §7` (file:line authoritative there):

- **R3. Schmidl-Cox TX preamble generators** — `modulator.cpp` `generatePreamble():551`,
  `createSchmidlCoxSTS():332`, `g_logged_tx_pilots:122`, `generateProbe():662` (verify).
  Test-tool-only; OFDM_COX is gone. **KEEP the S-C *correlation primitives*** in
  `ofdm_sync.cpp` — reused by warm-LTS sync.
- **R4. Reserved enum stubs** — `CodecType::{LDPC_5G,CONVOLUTIONAL,TURBO,POLAR}`;
  waveform `OTFS_EQ/OTFS_RAW/MFSK`; FrameType `DATA_START/CONT/END` (or finish the
  file-segment impl). Unreachable in production. Mind wire-compat (reserved values).
- **R5. Dead constellation mappers** — QAM32/64/256 in `modulator.cpp:85` (`mapBits`):
  no auto rung reaches them. Confirm no forced-mod test path first.
- **R6. `carrier_ldpc_interleaver`** — default-off diversity layer, auto-on only on masked
  carriers; likely removable. Confirm the masked-carrier path is truly unused first.
- **R9. `src/gui/adaptive_mode.cpp` legacy SNR-threshold controller** — the old
  `>48 dB → QAM64 R5/6` ladder (`:28-40`). NOT the production auto path (`recommendDataMode` /
  `selectCoherentOFDM` is), and it is the last live picker of `CodeRate::R5_6` after R5/6 was
  retired from the real ladder (2026-06-17). KEEP-check: confirm no GUI control still calls it
  before deleting; the `R5_6` enum value itself stays (valid LDPC rate + forcible probe).
- **R10. Sender-side mid-transfer rate drivers (superseded by RX-AUTHORITY)** — logged
  2026-07-05, deletion HARD-GATED on `ULTRA_RX_RATE_AUTHORITY` graduating default-ON
  (multi-seed faithful gate + rig campaign). Once the receiver commands the rung per group
  ACK, these `connection.cpp` senders-side drivers are dead weight (already inert under the
  knob): the `applyAdaptiveRateFeedback` EMA walk + `RateController` mid-transfer use
  (entry-time use STAYS), `qam16_clean_streak_`/`qam16ClimbStreak` climb hop, the dense-branch
  demote/crest walks (`qam16_bad_streak_`, `qam16_r34_clean_streak_`, the QAM8 upward step),
  `noteQam16Demoted` re-climb cooldowns, trough amnesty (`maybeTroughAmnesty` +
  `trough_episode_*`), `maybeApplyRxRateCommand` (the relative RX-RATE-CMD Phase 2 command —
  superseded by the absolute command), ssthresh/`noteRungFailed` mid-transfer arm.
  **KEEP (anti-footgun):** the ack-SILENCE safety rails — `executeEscapeDrop`,
  `maybeCollapseEscape`, `maybeEscapeStuckFrame` (the receiver cannot command through a
  blackout, these are the sender's only self-rescue); retx pacing; entry-time
  `selectCoherentOFDM`/`selectLadderRung`; the whole MC-DPSK/narrow rate machinery (RX-AUTHORITY
  is wideband-coherent-only); `RateController` itself if entry/fallback still consults it.

- **R13. `Impl::interpolateChannel()` + its N×N phasor tables** (found 2026-07-28) — an earlier
  DFT-channel-interpolation attempt that is **declared and defined but called from nowhere**.
  **Scope:** `demodulator_impl.hpp:415` (decl), `channel_equalizer_pilot.cpp:1204` (defn),
  `buildInterpolationPhasors()` `demodulator_impl.hpp:371` / `ofdm_demodulator_setup.cpp:70,193`,
  the members `interp_idft_phasors` / `interp_dft_phasors` (`demodulator_impl.hpp:53-54`, two
  N×N complex tables built unconditionally at setup for a function nobody calls) and the three
  scratch buffers `interp_h_full_scratch` / `interp_h_cir_scratch` / `interp_h_clean_scratch`
  (`:49-51`) plus `interp_pilot_logical_pos_scratch` (`:52`). It is also **wrong** where it is
  not dead: it IDFTs over the LOGICAL carrier index, which warps the delay axis because the
  carriers skip DC; it seeds the transform with linear interpolation (injecting the error the
  transform is supposed to remove); and it hardcodes `L = 5` taps. The correct delay-domain
  reconstruction now lives in `src/ofdm/delay_domain_interpolator.hpp` (closed form, physical
  carrier index, no FFT — see CHANGELOG 2026-07-28).
  **KEEP (anti-footgun):** `interp_table` / `buildInterpTable()` / the `InterpInfo` struct are
  the LIVE linear-interpolation table used every symbol in `updateChannelEstimate` — do NOT cut
  those with the phasors. Same for `all_carrier_fft_indices` and `is_pilot_logical`.

## Conditional removal — `ULTRA_ITERATIVE_CHEST` (added 2026-07-29)

Not decided-dead; logged here so the decision point is not lost.

**Scope if it loses its A/B** (it is default-OFF and unproven for reliability — see
BUG-GUI-GATE-EARLY-EXIT-FLAKE): `src/ofdm/iterative_chest.hpp`; the `da_*` /
`wiener_symbol_base_` / `wiener_history_flat_` / `wiener_carry_armed_` members and their
guards; `ingestDataAidedGrid`; `rereferenceCarriedHistoryPhase`;
`OFDMChirpWaveform::remodulateDataCarrierSymbols` / `ingestDataAidedFrame`; the five
`IWaveform` virtuals; `DecodeResult::data_aided_air_bytes`; the arm/origin/ingest block in
`streaming_burst_interleave.cpp`.

**KEEP even then (anti-footgun):**
- `CodewordStatus::usedAnyPerturbation()` — a general "these bits are certainly what was
  transmitted" predicate; the existing false-positive block already relies on the same fact.
- `signedBinForLogicalCarrier()` — trivial, and it de-duplicates a bin-signing expression
  that is otherwise open-coded in three places.
- The `test_iterative_chest_remod.cpp` ROUND-TRIP itself. It pins
  `encode → decode → re-encode → re-modulate == X` across the whole family, which is a
  standing guard on the TX/RX mapping agreement (interleavers, PRBS pad, CarrierLDPC
  eligibility, pilot rotation) — valuable regardless of whether the estimator lever ships.
  Only the last two knob-specific cases would go.

## Deprecate (divergent, not yet removed)

_(none currently — R7 done, see Completed removals)_

## Doc-only stale (not a code deletion)

- **R8. SC-DPSK** is listed in the CLAUDE.md waveform table but is not an `IWaveform` and
  not in the factory — fix the doc, no code to remove.

---

## Completed removals (record / momentum)

| Done | What | Commit |
|------|------|--------|
| 2026-05-30 | `decode_bench` tool + `DecodeBenchReplay` CTest + 6 replay fixtures | `bdee556` |
| 2026-05-30 | `agents/` autonomous system + Mac↔Pi5 hardware-cable rig + CI adaptation | `833725d` |
| 2026-05-30 | `cli_simulator` + `test_waveform_simple` + `SimulatedStation` (~14k lines) | `207a0af` |
| 2026-06-02 | **R7** `ultra_tnc` in-process AWGN: `applyAwgn`, `rng_`, `inject_channel*` config + `--inject-channel`/`--no-inject-channel` flags. OTASim is now the TNC's only channel (covered by the re-enabled `UltraTncSimAudio` 8 KB file test). | _pending_ |
| (earlier) | OFDM_COX as a selectable mode (enum `0x00` now reserved; S-C primitive kept) | — |

## R11 (2026-07-07): wideOFDM short-reanchor charge machinery
- **Scope:** `shouldUseWideOFDMShortReanchor` (now constant false), `wideOFDMShortReanchorChirpDurationMs`, `kWideOFDMShortReanchor*Ms` constants, the `reanchor_ms` parameters threaded through `wideOFDMBurstAirtimeMs`/`wideOFDMSackDelayMs` + 5 connection.cpp call sites + test_connection_policy reanchor rows. The encoder feature was removed in May (R4); the charge was a phantom that mis-priced every airtime budget (GROUP_SIZE_LEVER brief §1).
- **KEEP:** the airtime FORMULAS themselves (budget/timeout derivations are live); `#69 anchor` streak machinery (unrelated); warm-handoff (the thing that superseded it).

## R12 — tools/snr_meter_validation.sh + orphaned ofdm_snr_probe (REMOVED 2026-07-10)

**What:** the shell-based SNR meter validation probe. External review (finding
2) proved it FALSE-PASSING: its `ofdm_snr_probe` CMake target had been deleted
(a stale orphaned binary lingered in build/), the script parsed the stale
binary's columns incorrectly, dropped every row, and treated the empty summary
as PASS (exit 0). Its header also asserted the pre-recalibration
"+9.642 dB" expectation — wrong since the 2026-07-07 estimator fix.

**Scope:** tools/snr_meter_validation.sh (deleted), build/ofdm_snr_probe
(untracked artifact, removed). No CMake changes needed (target was already
gone).

**Superseded by:** tests/test_ofdm_snr_calibration.cpp (OFDMSnrCalibration
CTest) — hard assertions, AWGN ±1.5 dB + colored-noise + 16-seed fading
ensemble sections; plus MCDPSKSnrCalibration and
ChannelIdleNoiseSNRCalibration.

**KEEP (anti-footgun):** the CTest calibration suite and
`docs/PERFORMANCE_HISTORY.md` records — this removal is the SCRIPT only.

## R13 — `fable_analysis/` audit folder (REMOVED 2026-08-03)

**Scope (deleted).** All 17 tracked files in `fable_analysis/`: the June 2026 Fable-audit
record (`00_EXECUTIVE_SUMMARY` … `08_STALE_DOCS_AND_BUGS_REGISTER`), the July follow-up
`09_WHY_STUCK_AT_2000_2026_07_01.md`, `NEXT_SESSION_BRIEF.md`, `README.md`, and the five
`data_phase*.tsv` sweep files.

**Why dead.** It was a point-in-time campaign workspace, not durable documentation. Its own
kickoff brief routed future sessions to branch `wip/live-ladder-unvalidated`, which **no
longer exists** — so the brief could not be acted on. Its central subject (the 16QAM wall and
the "stuck at 2000" question) has since been superseded twice: the 16QAM rungs were MEASURED
on fading and are now all default-DISABLED, and the throughput ceiling it was reasoning about
turned out to rest on a raw-rate error corrected 2026-08-02 (pilots counted as payload).

**KEEP — do NOT treat as also-deleted:**
- The finding in `02_LLR_CALIBRATION` §4.4 was RESCUED first, as
  `BUG-QAM16-MMSE-SLICER-BIAS` in `docs/KNOWN_BUGS.md`. It documents a 16QAM MMSE
  slicer-bias defect that exists nowhere else. Do not delete that bug entry.
- `docs/LLR_NOTCH_CALIBRATION_2026_07_06.md` cites `fable_analysis/02` as its source for the
  same defect. That citation is now a dangling path; the content lives in KNOWN_BUGS.
- `docs/CHANGELOG.md` cites `fable_analysis/02`, `/07`, `/09` and the `data_phase2b_epsH_*.tsv`
  files as evidence in ~9 places. Those are HISTORICAL entries and were deliberately left
  unedited — the paths resolve in git history (`git show <rev>:fable_analysis/...`).

**Recovery.** Nothing is lost. The folder is in git history up to `a65d50b`; restore any file
with `git show a65d50b:fable_analysis/<file>`.
