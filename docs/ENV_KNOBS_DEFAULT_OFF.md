# Default-off environment knobs

Status: written 2026-08-01 and refreshed 2026-08-04, against `v0.5.1-pre-alpha`.
Scope: every `ULTRA_*` knob that is **off by default**, plus the handful of default-ON
knobs whose *opt-out* is the interesting half. Companion to
`docs/MODEM_INFRASTRUCTURE_MAP.md` (the live stage/knob map) and
`docs/REMOVAL_BACKLOG.md` (the deletion tracker).

Two halves:

* **[THE SHORTLIST](#the-shortlist--built-plausible-never-properly-tested)** — knobs that
  were built, look plausible, and were never properly tested, or were abandoned for
  reasons that no longer hold. This is the valuable half.
* **[Full reference](#full-reference)** — what every default-off knob actually does.

---

## How to read this

### Determining the true default

The default is in the *predicate*, not the name. Five patterns are in use, and three of
them are traps:

| Pattern | Example | True default |
|---|---|---|
| `e && e[0]=='1' && e[1]=='\0'` | `selective_repeat_arq_policy.hpp:287` | **OFF**, strict — only the exact string `1` enables |
| `e && e[0]!='\0' && e[0]!='0'` | `streaming_decode_policy.hpp:53` | **OFF**, loose — `false` / `no` / `off` all *enable* (anything not starting with `0`) |
| `e == nullptr \|\| atoi(e) != 0` | `waveform_selection.hpp:712` | **ON** — an opt-out; `=0` disables |
| `!(e && e[0]=='0')` | `modem_engine.cpp:567` | **ON** — an opt-out |
| `getenv(...) != nullptr` | `streaming_burst_interleave.cpp:512` | **OFF**, presence-only — **`=0` ENABLES it** |
| numeric with a fallback | `connection.cpp:5104` | the **fallback value is the default**; out-of-range input silently falls back |

Three live footguns, verified in the tree today:
* **`ULTRA_TNC_ACCUM_DISABLE` is inverted**: `bulk_accum_ = getenv(...) == nullptr`
  (`src/tnc/tnc_session.cpp:177`), so setting it to *any* value including `0` disables
  accumulation.
* **Presence-only knobs enable on `=0`**: `ULTRA_GAMMA_DOMAIN_LOG`,
  `ULTRA_CHEST_DELAY_SCAN`, `ULTRA_CHEST_NMSE_DUMP`, `ULTRA_TNC_ACCUM_PROBE`,
  `ULTRA_TNC_FLUSH_LOG`, `ULTRA_CHANNEL_DELAY_MS` (the `=0` case additionally flips
  `multipath_enabled=false`).
* **An invalid `ULTRA_MAX_OFDM_RATE` means no ceiling.** Only `R1_2`, `R2_3` and
  `R3_4` (or their lowercase forms) are recognized; any other text maps silently to the
  no-cap sentinel. This is deliberately different from the atomic `ULTRA_FORCE_DATA_*`
  parser described below, which diagnoses and rejects a malformed force as one unit.

Also: a knob wrapped in a function-local `static const` **latches on first call** and
cannot be toggled within one process. `ULTRA_ITERATIVE_CHEST`, `ULTRA_CHEST_NOISE_SCALE`
and `ULTRA_EVM_DEMOTE_CONFIDENT` were deliberately written *non*-latching so both arms can
run in one binary; `ULTRA_ACE_PAPR` latches, so each arm needs its own process.

### What "properly tested" means here

The IONOS rig has a **paired A/B standard deviation of 14–36 %**. Consequently:

| n (interleaved pairs) | resolves roughly |
|---|---|
| 3 | nothing |
| 8 | ~15 % |
| 12 | ~12 % |
| 20–30 | ~8 % |

Anything under ~5 % is structurally unmeasurable on this bench — the 16QAM R2/3 Good@20
gate has been observed swinging **0–2060 bps on identical config**
(`MEMORY 2026-06-12`), which is what retired the `eps_H` "wall-mover" as noise.

This is why several entries below say **UNDER-TESTED** rather than "rejected". Two
recorded examples of why n≤3 is not evidence:

* a forced-rung result of "+10.6 % unanimous over 2 pairs" became **+5.2 %, p=0.73 at n=8**;
* `ULTRA_RUNG_DWELL_MS` was built on a **−0.58 correlation that was reverse causation**
  (a rough channel causes *both* the churn and the low goodput).

**A knob is closed** if it was measured over ≥8 pairs, **or** if a *mechanism-level null
control* fired — i.e. the knob demonstrably did what it was designed to do and the outcome
did not improve. That second form legitimately closes a case at n=2
(`ULTRA_ANCHOR_SKIP_KEEP_STREAK_ON_SWITCH`, `ULTRA_EVM_DEMOTE_CONFIDENT`).

### Status vocabulary

| Status | Meaning |
|---|---|
| `DIAGNOSTIC` | logging/instrumentation only, no behaviour change |
| `MEASURED-HARMFUL` | tested adequately and it lost — closed |
| `MEASURED-WASH` | tested adequately, no effect — closed |
| `UNDER-TESTED` | tested at n≤3 or never A/B'd — **candidate** |
| `NEVER-TESTED` | built, plumbed, zero measurement — **candidate** |
| `SUPERSEDED` | the thing it tunes is bypassed or deleted — deletion candidate |
| `FORCE/DEBUG` | a measurement override; a tool, not a shipping option |

### The v0.5.1 rate-control cliff — read before trusting any rate knob

As of `v0.5.1-pre-alpha` the latent-state controller (`ULTRA_LATENT_RATE`) is
**DEFAULT-ON** and returns early from `updateRxAuthorityCommand` at
`src/protocol/connection.cpp:2596`. Everything downstream of that line in that function is
**unreachable on the default path** — the two-crater rule, ratcheting penalties, climb
dwell, EMA hold, the one-rung walk, the predictive climb, the EVM demote clamp,
trust-pick. Their `getenv` sites still exist; they no longer influence anything unless you
set `ULTRA_LATENT_RATE=0`.

A second, older bypass catches people out even more often: `rxRateAuthorityEnabled()` has
been **DEFAULT-ON since 2026-07-05** (`src/protocol/waveform_selection.hpp:712`), and
`applyAdaptiveRateFeedback` — the entire *sender-side* climb/demote machinery — is in the
`else` arm at `src/protocol/connection.cpp:2000-2013`. It has been dead by default for a
month longer than the latent controller.

Those knobs are listed under
**[Superseded / deletion candidates](#superseded--deletion-candidates)**, not as options.

---

## THE SHORTLIST — built, plausible, never properly tested

### Tier 0 — free repairs. Do these before any rig time.

These are not experiments. They are defects found while auditing, each with a one-line or
one-grep fix, and two of them **invalidate future measurements** if left alone.

| # | Item | file:line | What is wrong | Fix |
|---|---|---|---|---|
| R1 | ~~`ULTRA_COMMANDED_GEOMETRY` gated two things~~ | `src/gui/modem/streaming_ofdm_decode.cpp`; `streaming_burst_interleave.cpp` | **CLOSED 2026-08-03/04:** the truncation/alignment guard (A) is unconditional; only broad commanded-rung reconstruction (B) remains behind the default-OFF knob. Exact same-rung, wire-learned Z81 restoration is default-active for BI0 8PSK R2/3 cw4 and QPSK R3/4 cw3. BI1 fails closed because descriptor loss also loses non-rung-derivable `N`. BI0 ignores stale configured `N`, searches under the sender-policy ceiling, and finalizes only on a CRC-valid tail. | The first BI1 immutable run found N7-wire/stale-N8; review caught inverse N8-wire/stale-N7 before deployment. Fresh revalidation remains owed. |
| R2 | ~~`ULTRA_ACK_REPEAT_SILENT_MS` opt-out and arm baseline~~ | `src/gui/app.cpp`; `connection_policy.hpp` | **CLOSED 2026-08-04:** unset, empty, malformed, out-of-range, and `0` now remain OFF; only 300–5000 enables. A second audit found the broad-signal hold counted the just-decoded group that armed the ACK, delaying a nominal 4000-ms repeat for 13 seconds and past the sender RTO. | The arm now snapshots the broad signal stamp and only newer evidence holds. Policy regressions pin equal/older/newer/stale/future stamps. The knob remains default-OFF pending a fresh immutable OFF-vs-4000 IONOS A/B. |
| R3 | LTS residual-CFO log lies about *why* it skipped | `src/ofdm/channel_equalizer_lts.cpp:554-555` | A 2.71 Hz residual against a 0.3 Hz threshold prints `(below correction threshold)` from an `else-if` catch-all when it actually failed the seed/CV gate. The informative branch at `:548-553` is behind `cfoDebugLogEnabled()`. This is recorded as the reason the root cause took so long to find, and it nearly caused an A/B null control to be read **backwards** (178 vs 38 "LTS residual CFO" lines, of which 176 were *rejections*). | Make the catch-all state the actual rejection reason. |
| R4 | 44 knobs missing from the live map | see [Not in the infrastructure map](#not-in-the-infrastructure-map) | `ULTRA_LATENT_RATE` — the **default rate controller since 2026-08-01** — has zero hits in `docs/MODEM_INFRASTRUCTURE_MAP.md`. So do `ULTRA_BURST_ESCALATION` (default-ON, +24 % rig) and `ULTRA_ECHO_REANCHOR_GATE` (default-ON, +17 % rig). CLAUDE.md makes the map mandatory-live; a wrong 🟢/🔴 causes a wrong deletion later. | Add them. Fix the three stale map/doc rows in the same pass (R5). |
| R5 | Three doc/code disagreements | The `ULTRA_COMMANDED_GEOMETRY` rows were corrected 2026-08-04. Remaining: `MAP:233` says `ULTRA_MODE_CHANGE_RETRY_MS` defaults to ~5 s ratiometric although the `getAckTimeout()` floor was restored (~18.5 s); `KNOWN_BUGS.md:805` says `ULTRA_ACKLISTEN_SUPPRESS_OFDM` is default-OFF although it is ON. | — | Correct the two remaining rows before relying on them. |
| R6 | `ULTRA_LDPC_MAX_ITERS` — close it with a grep, not a sweep | `src/fec/ldpc_codec.hpp:98` | The motivating observation ("21 of 26 failures stopped at exactly 70 iterations") is a **tautology**: `ldpc_decoder.cpp:234-267` has exactly one non-cap exit and it is success, so every failure reports `iters == max`. | Grep the existing forensics corpus for **successful** `iters=` values and `RETRY OK` density. Worst accepted CW on record is 25 of 50 (`docs/CALIBRATION_AUDIT.md:861`). If no successful decode anywhere exceeds ~30, there is no slow-convergence population and the question closes with **zero new runs**. |
| R7 | Rename the `SELF-ECHO` log | `src/gui/modem/streaming_ofdm_decode.cpp:2672` | `ULTRA_ECHO_REANCHOR_GATE` shipped on a mechanism that was later **proven wrong** (`KNOWN_BUGS.md:812`): the signal was the *peer's tone ACK*, and self-echo is physically excluded on both benches. The fix keyed on the right state (monitor armed) and remains valid, but the log line and `ack_listen_self_echo` still name a mechanism that cannot happen. | Rename. It has already cost one investigation. |

### Tier 1 — ranked experiment candidates

Ranked by *expected value net of rig cost*, not by how good the idea sounds.

| # | Knob | file:line | What it would do | Why it is worth a run | Can the rig resolve it? |
|---|---|---|---|---|---|
| 1 | `ULTRA_CRATER_REANCHOR_HOLD` | `streaming_ofdm_decode.cpp:2691` | Two-crater discipline for the receiver's **sync** re-anchor: hold warm sync through crater #1, re-anchor from #2 | Only knob here with a mechanism metric *and* a positive paired result that was never refuted by a clean solo test. The revert is confounded 3 ways (see below) | Marginal. Effect is now smaller than the +44 % that motivated it → **n≥12** |
| 2 | `ULTRA_BURST_INTERLEAVE` | `connection_policy.hpp:239` | Cross-frame codeword interleave: TX permutation + whole-group ACK semantics + wire flag | Repaired Good@20 local screens are strongly positive for QPSK/8PSK Z81, but the latest real MPG@20 16QAM pairs favored OFF with far fewer craters. The mechanism is modulation/channel-class-dependent *by construction* | Yes, but only **per profile and channel class**, n≥8 each. Do not re-A/B or ship it as a global boolean |
| 3 | `ULTRA_ACK_REPEAT_SILENT_MS` | `app.cpp:3011` | Explicitly enabled second, Tc-decorrelated copy of a distinct tone ACK after an unbroken quiet window | Default-OFF with **no goodput A/B at any n** and seven historical collision-repair patches behind it. Two fresh reverse-link ACK misses now give it a concrete target; the arm-baseline defect is fixed | Yes — first one engagement smoke, then matched alternating OFF/4000 at n≥8 |
| 4 | `ULTRA_KEEPALIVE_ACK` (+`_MS`) | `connection.cpp:5093` / `:5104` | Re-emit the cumulative tone ACK after `_MS` of silence during an active receive | Upside is rig-proven and reproducible (max ACK-silence gap **capped at exactly 8.0 s vs 36.7 s off**). Both original blockers named a specific, bounded fix | **Blocked** — two prerequisites first (below). Prize is ~3–4× smaller than the retracted headline |
| 5 | ~~`ULTRA_CONNECT_ACK_RESCUE_DEFER`~~ | removed 2026-08-03 | Historical defer experiment; replaced by authoritative frame confirmation plus reactive duplicate-CONNECT replay | **CLOSED without the knob.** No proactive full-ACK timer is armed; accepted sync retains the cached ACK, and 0/N groups cannot confirm | No A/B: focused state-machine regressions plus a fresh IONOS transfer are the validation path |
| 6 | `ULTRA_LTS_CFO_SEED_OFF` | `channel_equalizer_lts.cpp:469` | Bypass the seed/CV gate blocking the LTS residual-CFO refinement | **Contested** — two independent adversarial passes reached opposite conclusions (below). Mechanism is proven at n=80/cell, 3–5σ, on Moderate | Not on the rig first. There is a **cheap sim experiment already plumbed** that nobody ran |
| 7 | `ULTRA_BURST_RMS_DIAG` | `streaming_burst_interleave.cpp` `tryDemodulateNextBurstFrame` | Log the erasure-gate inputs per collected burst frame | The level half of the owed measurement is answered: `--rx-gain-db` lowers the receive level without lowering SNR, and at -40 dB (frames ~0.003 RMS) every frame the relative gate keeps decodes (2026-11-10). Still open: frames riding far below their anchor on a cheap card | Yes — **one** instrumented rig transfer, not a campaign |
| 8 | `ULTRA_ACE_PAPR` | `modulator.cpp:285` | Active Constellation Extension: extend outer points outward to shave TX crest | ~1 dB of average power, non-negative by construction at the demapper. CPU cost measured negligible (0.25–0.36 % of a core) | **No** — 0.8–1 dB needs n≈20–30 pairs. Run the free null control instead (below) |
| 9 | `ULTRA_SIM_PAPR_PENALTY` | `app.cpp:3625` | Make the sim TX peak-normalize like hardware instead of RMS-normalizing to a fixed reference | Not a throughput lever. It is a **simulator-fidelity decision**: with it off, OTASim delivers ~10 dB more data-frame average power than any peak-limited PA | N/A — needs a decision, not a measurement |
| 10 | `ULTRA_LATENT_RELAX_DB` | `connection.cpp:2627` | Forgetting rate of the default-ON latent controller's posterior — "THE ONE FREE PARAMETER" | Highest leverage of anything here (it is on every OFDM connection), but its named failure mode was already checked at n=8 and did not fire | **Not yet.** The tie-break probe must be gated first (below), or the sweep is under-powered by construction |

---

### Detail, ranked

#### 1. `ULTRA_CRATER_REANCHOR_HOLD` — `src/gui/modem/streaming_ofdm_decode.cpp:2691`

**Default:** OFF, `e && e[0]=='1' && e[1]=='\0'`. Was DEFAULT-ON 2026-07-21 → 2026-07-23.

**Mechanism.** Default behaviour forces a full chirp+LTS re-anchor
(`expect_full_ofdm_anchor_ = true`) on *every* 0-CW group crater — ~7× per 50 KB transfer,
each costing a ~1.2 s chirp prefix and delaying the ACK into the sender's RTO. The knob
holds warm sync through crater #1 and re-anchors from #2 —
the same restraint the F122 rate rule already uses. The physical argument is real and
code-checkable: a fade attenuates but does not move sample timing, whereas
`expect_full_ofdm_anchor_=true` plus `resetFrameArrivalTrackingLocked` discards the
warm-sync prediction and re-arms a wide blind search. `crater_reanchor_streak_` resets on
any decoded frame (`:2644`), so isolated craters never reach streak 2.

**Evidence.** PRO: F470-481, MPG@20, 6 interleaved pairs — full re-anchors **halved
(25 vs 50)**, delivered goodput +44 % (1.87 vs 1.30 kbps), delivery tied 4/6
(`CHANGELOG:3504`). CON: reverted on F580-591 (`CHANGELOG:3641`) — delivered +11 % but
2/6 vs 1/6 failures = aggregate wash.

**Why the revert does not close it.** Three confounds, all verbatim in `CHANGELOG:3627-3655`:
(1) it was an **all-levers-ON** test (with `ULTRA_RX_EMA_HOLD` and `ULTRA_CHEAP_REANCHOR`)
— no per-lever attribution; (2) the baseline arm ran forced `ULTRA_BURST_INTERLEAVE=1`,
measured at −37 % *the same session*, and the entry itself says the re-anchor gap
"was inflated"; (3) the surviving objection — "the hold levers OVER-HOLD a failing **rung**"
— is an argument about the rate controller, and this is a **sync** knob whose getenv is
nowhere near `updateRxAuthorityCommand`.

**Two things that argue against, and must be priced in.** The crater population it acts on
has **shrunk** since F470-481: `ULTRA_COMMANDED_GEOMETRY` (2026-07-28) removed
stale-geometry 0/N craters, which were never fades and so were never valid targets for this
mechanism, yet were part of the +44 %; and the latent controller cut mode changes 5.0 → 2.2
per run, removing post-switch acquisition craters — and `CHANGELOG:2873` establishes
post-switch as the *riskiest* moment to omit acquisition. Also, the +44 % is over delivered
runs only (4 of 6 pairs) on an epoch the entry itself calls "rough/FAIL-heavy".

**Experiment.**
* Arms: `ULTRA_CRATER_REANCHOR_HOLD=1` **alone** (leave `ULTRA_CHEAP_REANCHOR=0`, so the
  branch is `streak>=2`) vs clean default. Interleaved, same epoch.
* n ≥ **12** — the expected effect is now smaller than the +44 % that motivated it.
* **Primary:** delivered goodput with failed transfers scored as **zero**, plus completion
  rate. Do not report delivered-only means.
* **Secondary:** craters/run and resends/run — the two metrics that killed the sibling
  anchor-skip knob (`CHANGELOG:2873`: re-anchors down, craters 2.0 vs 1.0, resends
  1.0 vs 0.0, goodput a wash).
* **Engagement null control only:** full re-anchors/run. This metric *restates the knob's
  definition* and must never be the primary readout.
* **Pre-committed falsifier:** craters/run or resends/run up → stop.

#### 2. `ULTRA_BURST_INTERLEAVE` — `src/protocol/connection_policy.hpp:239`

**Default:** OFF for all modulations. `if (const char* env = getenv(...)) return env[0]=='1'; return false;` —
note the `Modulation` parameter is **unnamed and ignored**.

**What it controls.** Single source of truth for three coupled things: the TX byte
permutation (each LDPC codeword spread across all N frames of the group), the ARQ semantics
(ON = whole-group ACK/NACK, OFF = per-frame SACK masks via `Connection::burst_interleave_off_`),
and the on-wire `BURST_FLAG_INTERLEAVE` bit. Consumed at `src/gui/modem/modem_mode.cpp:323`.

**Real measurements had opposite signs; the new local screen adds useful but non-live
profile evidence.**

| Date | Config | n | Result |
|---|---|---|---|
| 2026-06-14 | GUI, Good@20, 16QAM R2/3 | 6 | interleave **ON** +47 % (~1270 → 2033-2240 bps) |
| 2026-07-21 (F440-449) | rig MPG@20, forced 16QAM R1/2 | 5 | interleave **OFF** +37 % (1.55 vs 1.13), 5/5 vs 4/5 delivery, **5× fewer full craters (12 vs 62)** |
| 2026-08-02 | `measure_ack_fer`, Good@20, QPSK R3/4 cw3/Z81, seeds 7/11/23/42 | 320 groups/arm | steady complete groups **298 BI0 → 319 BI1**; incomplete groups 22 → 1; equal 8.410 s emitted groups |
| 2026-08-02 | `measure_ack_fer`, Good@20, first/resend FULL DATA, same seeds | 80 groups/arm/profile | QPSK complete **73 → 78** and 8PSK cw4/Z81 **60 → 72**; repaired descriptor/acquisition path, simulator only |

The real campaigns differ in rung, forcing, and epoch. The OFF side is stronger because it
carries a real-rig mechanism metric, but five pairs do not close the question. The new
simulator cells are properly powered for those exact local profiles and show why BI deserves
a real test; they cannot overturn the contrary hardware result or establish a global default.

**The mechanism is class-dependent by construction.** Interleaving turns a frozen
*frequency*-selective null into a recoverable ~1/N nick — but couples a *time*-localized
Watterson fade into every codeword of the group at once, converting one dead frame into a
whole-group 0/N crater. That is a delay-spread-vs-Doppler question, and this knob is a
single global boolean whose modulation argument is discarded: exactly the
`if (mod==X)` / one-channel smell the CLAUDE.md adaptivity rule names.

**What changed.** The project now has a working channel-class discriminator it did not have
in July (fixed 2026-07-25: GOOD +0.606 vs MOD/POOR −0.285, 0.89 separation).

**Experiment.** First run matched-build, byte-exact BI0/BI1 pairs for the exact Z81
modulation under consideration. Then gate the profile on the **measured class** — ON where
delay spread dominates, OFF where Doppler does — and test per class at n>=8 (MPG and MPM).
Do **not** re-flip the global default from either a simulator cell or the 2026-06-14 number.

#### 3. `ULTRA_ACK_REPEAT_SILENT_MS` — `src/gui/app.cpp:3011`

**Default:** **OFF.** Unset, empty, malformed, out-of-range, and explicit `0` all disable it.
An integer in [300,5000] enables that delay; the value is process-latched on first use.

**What it does.** On every distinct tone-burst ACK, stashes a decorrelated copy; the GUI
tick (`App::maybeFireAckRepeatIfSilent`) fires it once, the requested delay after copy-1
airtime ends, and only after an unbroken quiet window. Rationale: the ACK is 4-FSK
across 2400-2625 Hz = a **225 Hz span** (`tone_burst_constants.hpp:63-71`), and ITU Good's
0.5 ms delay spread gives Bc ≈ 2 kHz — so the whole ACK fades **flat**. It has no frequency
diversity by construction; time diversity is its only diversity. A repeat delayed past Tc
rides a decorrelated fade draw (p² instead of p). GUI-only — `ultra_tnc.cpp:509` queues the
tone ACK with no repeat path.

**Evidence.** **No goodput A/B at any n.** The recorded history is six successive
collision-repair patches, each found after the previous version was believed safe: F100
(25 redundant repeats blanked inbound burst heads, 13 craters), F129, F143, F147 (false
chirp locks on idle noise cancelled the repeat; sender went RTO-deaf 40 s), F176/F221
(geometric air-end gate), F222 (dedup-arm), F227 (repeat fired 1.3 s after the peer's burst
SYNC, wiped frame 1, ~3× MC-DPSK goodput loss). A later parser repair restored the intended
default-OFF contract. On 2026-08-04, an immutable 2048-buffer transfer lost two of 18
correctly committed primary ACKs. One loss redundantly resent seven already-delivered frames;
the other added RTO wait before an already-required crater repair. Audit then found that the
broad-signal gate still counted the group that armed the ACK and held the repeat for 13 seconds,
past both measured 9.748/9.054-second post-commit RTO budgets. The arm-relative baseline fix
landed with a policy regression; it has not yet run on hardware.

**Experiment.** New immutable freeze, one 4000-ms engagement smoke, then alternating matched
OFF/4000 at n≥8 on MPG@20. Predeclare exact bytes/SHA, teardown and physical accounting,
nonzero treatment arm/fire, every fire inside the sender listen window and before RTO, zero
DATA overlap/collisions, and sender acceptance of a rescued ACK. Note this still keys a second
TX on a half-duplex channel purely on an *inference* (silence ⇒ our ACK died); the record
shows that inference misfiring repeatedly, so collision gates outrank headline goodput.

#### 4. `ULTRA_KEEPALIVE_ACK` / `ULTRA_KEEPALIVE_ACK_MS` — `connection.cpp:5093` / `:5104`

**Defaults:** knob OFF (`e && e[0]!='\0' && e[0]!='0'`); threshold **8000 ms**, range
[3000,60000]. Was DEFAULT-ON on 2026-07-14 and reverted the same day (`fcfd02e`).
The comment at `connection.cpp:5096` still says "default 25000" — **stale**, and the
5064-5091 block states the default three contradictory ways.

**Upside is real and reproducible.** F290/F291 (MPG@20, `_MS=8000`): max ACK-silence gap
capped at **exactly 8.0 s vs 36.7 s** with it off, across both runs, 13 keepalives/transfer,
cwfails normal. That is an epoch-independent mechanism metric with a null control, not a
goodput draw.

**Three things must be fixed or re-verified before it is worth a pair of rig runs.**

1. **The prize was retracted.** `CHANGELOG:419-452` (null-controlled, ADAPTIVE-RTO logging
   on *both* ends): "The SENDER's configured timeout is 17.4–23.1 s, not 44.7 s. The 44.7 s
   figure was read from the Mac's log, and the Mac is the file RECEIVER." The stall-measurement
   *method* was also withdrawn — it counted gaps between "Burst group complete" events, and
   during a NACK round the sender is transmitting. `KNOWN_BUGS.md:406-410`'s "~+30 %" rests on
   both retracted inputs and was never updated. An 8 s cap against a 17.4–23.1 s RTO saves
   ~9–15 s per stall on only the *idle* fraction.
2. **The collision gate is measured leaky.** `KNOWN_BUGS.md:201-214` (two-station DEBUG
   capture, clocks pinned by three independent ACK arrivals) measured *both* terms of
   `channelBusyForTx() || burstAirSamplesRemaining() > 0` failing at once: CCA read `idle=1`
   mid-burst in a ~7 dB fade trough, and the air-gate is computed from a stale
   `fixed_frame_codewords_` that is zeroed at finalize. The recommended 8 s is *shorter* than
   the 11.5 s escalated burst ceiling, so the leaky gate is the sole protection. **Do not
   adopt the filed refinement** to gate on `burstAirSamplesRemaining` only: that returns 0
   until a group *arms* (`streaming_burst_interleave.cpp:459-469`), and the sync-reject stall
   this knob targets is by definition the case where no group arms — so the geometric term
   reads "clear" for the whole inbound burst.
3. **The rung-neutral keepalive is still required.** The re-emit still stamps the standing
   absolute rung command (`connection.cpp:445-447`, `tba.rate_hint = rx_authority_cmd_ & 0x7`),
   and `noteAnchoredBurstNoGroup` (`:4521-4534`) still drives `observe(cur, k=0, M=5)` into the
   latent posterior mid-stall. v0.5.1 removes the *specific* July driver (a momentary
   usable-8.8 SNR misread) because the latent controller consumes no SNR — but a frozen
   command can be just as depressed, driven by outcome instead. Emit base+bitmap only.

**Experiment (after 2 and 3).** n≥8 interleaved, MPG@20, `_MS=8000` (which is already the
default — passing it changes nothing). **Primary: delivered goodput**, not the silence gap;
F290/F291 already proved the knob does what its name says, which is not the open question.
Carry a collision/self-deafen counter and a `burst_clean_group_streak_` reset count —
`noteArqRoundOutcome` zeroes that streak at `connection.cpp:3628`, and it gates the
group-size escalation lever that shipped at +24 %.

#### 5. ~~`ULTRA_CONNECT_ACK_RESCUE_DEFER`~~ — removed 2026-08-03

**Historical default:** OFF, strict. The env read and defer machinery are removed.

Rig 2026-07-30: with the defer on, **3/3 handshake attempts deadlocked** against a ~1-in-20
baseline; P(3 of 3 | 1/20) = 1.25e-4, so n=3 is adequate here — this is a
catastrophic-failure-rate change, not a 15 % goodput delta. Mechanism: releasing the rescue
fires up to 6 CONNECT_ACK re-sends of 8.3 s each (~50 s keyed up) while the initiator is
transmitting its own CONNECT retries — **both stations keyed at once on a half-duplex
medium**.

**The defect was real and expensive.** BUG-CONNECT-ACK-RESCUE-DISARM: a sync correlation
is *not* proof of a peer state transition. Measured at corr=0.59 and corr=0.80 with the
initiator still retrying nine times into a responder that had already cleared
`connect_ack_frame_` and would never answer — permanent half-open, **420 s per attempt**
(`KNOWN_BUGS.md:733`).

**Closed by a state-machine redesign, not a threshold.** The responder no longer schedules any
proactive full-ACK retry. Accepted OFDM sync retains the cached CONNECT_ACK; it neither confirms
nor destroys recovery. During a wide-OFDM (`OFDM_CHIRP`) half-open interval, only a true expected dual-chirp anchor arms one bounded
raw-ring fixed DBPSK R1/4 CONNECT attempt after the whole four-codeword body arrives; weak OFDM
fallback locks cannot arm it. A recovered duplicate CONNECT silently discards the false OFDM
group, restores the physical-SNR/OFDM/EVM/fading snapshot taken at half-open entry, re-arms the
real OFDM anchor, and immediately replays the cache after the initiator's body ends. Confirmation
and cache retirement require a CRC-valid post-CONNECT frame addressed to the
local station and sourced by the established peer. Empty/all-failed groups and other-station
traffic are explicitly non-authoritative. The raw-ring cross-mode fallback does not currently
cover `OFDM_NARROW`.

#### 6. `ULTRA_LTS_CFO_SEED_OFF` — `src/ofdm/channel_equalizer_lts.cpp:469`

**Default:** OFF (`e != nullptr && e[0] != '\0' && e[0] != '0'`), non-latching.

**What it does.** Production applies the LTS residual CFO only when
`coherent_residual && (trusted_cfo_seed || flat_lts_channel)`. The knob forces `seed_ok`
true, leaving only the coherence test (≥0.70) and the 0.3–5.0 Hz magnitude window.

**This entry is contested — two adversarial reviews reached opposite conclusions.** Both
are recorded because the disagreement is the useful information:

* **Keep it open (UNDER-TESTED).** The FER tier is closed-quality: `measure_ack_fer`,
  QPSK R1/2 Moderate, seeds 7+11, **n=80 frames/cell** — gate ON 65/80 at 20/28/**60** dB
  (a floor flat across 40 dB of SNR is a receiver impairment, not an outage), gate bypassed
  78/80 @20, 80/80 @28, 80/80 @60. Full 12-cell sweep: AWGN byte-identical, Moderate
  **+31/320 (3–5σ)**. The goodput tier was answered at **n=6** against a baseline spread of
  410–1620 bps (4×) — i.e. not answered. Corroborated independently by `KNOWN_BUGS.md:450`
  (BUG-ANCHOR-CFO-KILL) which already names the gate as structurally off on fading.
* **Close it (MEASURED-WASH on the shipping metric).** Good is **−7/320, consistently
  negative** across all four rungs at n=80/cell — the harmful direction on the channel we
  actually operate on. And two **mechanism-level null controls already ran**:
  `ULTRA_LTS_CFO_COH` at 0.70/0.85/0.93 → identical results; `ULTRA_LTS_CFO_MIN` at
  0.3/1.0/1.8 Hz → identical results. Verbatim conclusion: *"the Good cost is a genuine,
  coherent, large rotation whose correction still hurts… A principled fix would weight the
  correction by its persistence (e.g. a per-symbol tracked phase rate rather than a
  frame-constant CFO), not gate it on channel flatness."*

**What both agree on.** The bypass is the wrong *shape*: it applies a frame-constant
extrapolation of an instantaneous two-symbol phase slope across a 26-symbol payload, which
over-reaches when the true offset is ~0 (Good) and helps when drift dominates (Moderate).
Neither coherence nor magnitude separates the two cases. A **persistence** gate is the
named successor, and it has nowhere to live today: `last_lts_residual_cfo_hz` is zeroed in
`OFDMDemodulator::reset()` (`ofdm_stream_processor.cpp:1229`), called per decode, so it
must be hosted in `StreamingDecoder` and pushed down (the pattern already used for
`doppler_coherence_`).

**The cheap experiment nobody ran.** The claim that "the sim cannot answer this because
OTASim has no CFO" is **false**: `SimulatedChannel::setTxCFO` exists at
`src/ota_channel_core/channel.cpp:46-47`, applied at `:107/:119` *independently* of the
Watterson `cfg.cfo_hz` that `:173` zeroes, and `ModemEngine` already plumbs
`config_.tx_cfo_hz` (`modem_engine.cpp:66,270`). The decisive question — does a real nonzero
TX CFO (the rig ppm case) change the verdict — is a **harness plumb-through of an existing
facility**, minutes of work, not an n≥8 GUI campaign. Do that first. Then, if it is still
open, re-A/B the knob *unmodified* on the v0.5.1 default path on **both** Moderate and Good
(Good has never been run through the GUI gate, and its −7/320 cost is the design risk).

**Unstated interaction to price in:** every applied residual is pushed into the persistent
CFO prior via `ingestPilotResidual` (`cfo_tracker.cpp:34-42`), clamped only at 2.0 Hz, and
that file carries a standing warning that ingest happens *before* any LDPC verdict exists.
The knob multiplies the ingest rate ~11× (2→22 applied on one seed).

#### 7. `ULTRA_BURST_RMS_DIAG`

`streaming_burst_interleave.cpp` `tryDemodulateNextBurstFrame`. Not a lever; one owed measurement.
Its A/B sibling `ULTRA_BURST_ERASURE_ABSOLUTE` (the legacy fixed 0.015 gate as the OFF arm) was
deleted 2026-11-11 (REMOVAL_BACKLOG R21).

The burst erasure gate became **operating-level-relative** on 2026-06-16 —
`max(0.055f * burst_anchor_rms_, 0.005f)` (`streaming_burst_interleave.cpp:674-693`) —
because the legacy absolute 0.015 floor implicitly meant "~25 dB below the *sim* anchor
0.27" and on IONOS became only ~5 dB below the anchor, erasing recoverable frames (measured:
data frames 2-6 erased every group at RMS 0.0038-0.0145 → header invalid → ARQ stall). The
fix was justified **analytically** (k=0.055 reproduces 0.015 at the sim anchor, hence zero
sim regression by construction) and the CHANGELOG states plainly that a sim A/B of its
*benefit* is structurally blocked — lowering sim TX level enough to bite the gate also
craters SNR. So the fix that keeps 16 of 20 previously-erased frames has **never been
confirmed to decode them**. `KNOWN_BUGS.md:1279` still carries the owed action.

**Experiment.** One instrumented rig transfer, Mac→Pi5 MPG, `ULTRA_BURST_RMS_DIAG=1`. Read
the kept-vs-erased separation on `next/anchor` and `next/noise`, and confirm the kept frames
actually decode. Cost: one transfer.

**Update 2026-11-10/11.** The structural block above is gone: `gui_qso_scenario.sh --rx-gain-db G`
scales the received audio after the channel, so the level drops while the SNR stays put. At -40 dB
(data frames ~0.003 RMS, under both the old 0.015 floor and the 0.005 backstop) every kept frame
decoded: 1230 bps, 0 erasures, against 40 erasures and 360 bps with the 0.005 backstop still in
(BUG-RX-ABSOLUTE-LEVEL). What only the rig can still show is the cheap-card case, data frames riding
much further below their own anchor than a level change moves them; the diag log alone shows it
(kept frames that fail to decode), so no OFF-arm knob is needed.

#### 8. `ULTRA_ACE_PAPR` — `src/ofdm/modulator.cpp:285`

**Default:** OFF, latching static (each arm needs its own process).

Active Constellation Extension on DATA symbols only (gated on `genie_capture`, so
LTS/probe/preamble stay pristine), coherent mods only. Points move only *outward*, so a
memoryless demapper cannot do worse. Unit-proven in `tests/test_papr_ace.cpp` (CTest
`PaprAce`): 0.7–0.9 dB PAPR reduction plus both safety invariants.

**What is settled.** Rig A/B F273-F278 (interleaved, natural 16QAM, MPG@30): +1.20 / +0.34 /
−0.65 kbps, mean +8 %, recorded verbatim as "INSIDE the ±25 % fade-epoch noise, so
inconclusive (as predicted: 0.8 dB is too small to resolve in 3 pairs)". CPU: **measured
54–76 µs/symbol against 21333 µs of symbol airtime = 0.25–0.36 % of one core** — the
"unmeasured Pi5 CPU cost" blocker is closed, negative.

**What is not settled, and caps the upside.** Two findings that must travel with this knob:

* **The gain saturates at ~1.06 dB.** Only DATA symbols are ACE'd, so the un-ACE'd chirp
  anchor floors the burst peak that per-burst normalization sees. Measured on a real `ptx`
  burst: chirp envelope peaks at 0.708 vs data peak 0.800, so the chirp sits 1.06 dB below.
  Whole-burst gain was **+0.26 dB** on a short single-frame burst (the chirp is ~40 % of it);
  data-segment gain +0.53 dB; synthetic 60-symbol data burst +1.11/+1.13 dB — already at the
  ceiling.
* **"Zero-risk" is false.** `ULTRA_SOFTWARE_ALC` is **default-ON** and declares a burst
  CLIPPED when RX crest factor falls below 6.5 dB (`connection_policy.hpp:304-311`; verdict
  at `streaming_burst_interleave.cpp:996-1006`), a threshold explicitly set ≥2.5 dB below the
  healthy 9–14 dB floor. ACE removes **1.11–1.13 dB** of crest — ~44 % of that designed
  margin. One false CLIPPED verdict costs a −2 dB `tx_drive` down-step, larger than ACE's
  entire gain.

**Next step is not a throughput campaign** (0.8–1 dB needs n≈20–30 pairs). Run the **free
null control**: one transfer with `ULTRA_ACE_PAPR=1`, grep the receiver's `[ALC-RX]` line
(`streaming_burst_interleave.cpp:1017`) for `cf_db` and verdict. If `cf_db` stays ≥8 dB and
no CLIPPED verdict appears, the zero-risk framing is restored and a many-pair campaign is
justified; if CLIPPED appears, the crest threshold needs retuning before ACE can ship at any
n. **Must run on the rig or with `ULTRA_SIM_PAPR_PENALTY=1`** — OTASim's TX is
RMS-normalized and structurally cannot show a PAPR benefit.

#### 9. `ULTRA_SIM_PAPR_PENALTY` — `src/gui/app.cpp:3625` (+ `ULTRA_SIM_TX_PEAK`, `:3633`)

Not a throughput knob — it makes the sim *harder*. It is on the shortlist because it is an
unresolved **fidelity** decision, and CLAUDE.md makes simulator fidelity non-negotiable.

Default sim TX RMS-normalizes every burst to the fixed in-band reference
(`normalizeTxBurstToReference`, `app.cpp:3652`), so a ~14-15 dB-PAPR coherent OFDM data
frame rides at the *same average power* as the low-PAPR chirp/control frames. With the knob
on, the sim runs the same peak normalization as hardware (`app.cpp:3643`), and data-frame
effective SNR drops by exactly its PAPR back-off (~10.26 dB measured for coherent QPSK by
`tools/papr_tx_measure.cpp`).

Consequence: **every PAPR-class lever is structurally invisible to the faithful gate**, and
the 2026-07-02 sim-based PAPR disable was invalidated by exactly this. It is also the only
path on which the `ULTRA_SOFTWARE_ALC` closed loop is exercisable rig-free.

**Decision needed, not a measurement:** either flip the sim default and re-baseline every
sim number, or record in the map *and* CLAUDE.md that OTASim's TX is intentionally
power-optimistic and that **no PAPR or level claim may be made on it**. Leaving it as an
unnoticed knob is the "the simulator does it differently because it's a simulator" failure
mode.

#### 10. `ULTRA_LATENT_RELAX_DB` — `src/protocol/connection.cpp:2627`

**Default:** 0.35 dB, range [0.0,5.0]. Consumed once per group verdict at `:2633`
(`latent_ctl_.relax(kRelaxDb)`); sole call site outside tests.

This is now **the most load-bearing single constant on the default rate path** — the
controller's own header calls it "THE ONE FREE PARAMETER" and warns that too-small a value
freezes the posterior and *looks perfect on OTASim* (stationary Watterson) while failing on
the rig (`latent_rate_controller.hpp:244-251`). It was set once, never swept.

**But its named risk was already checked.** `CHANGELOG:180-186`, over the same 8 interleaved
rig pairs: "On the rig the posterior stayed live: sd 1.4-2.1 throughout, tracking
k = 3/5..5/5. The run that could catch it did not fire it." That is the exact
spread-and-tracking readout a sweep would produce as its first step.

**Prerequisite that blocks the sweep — and is itself the more urgent item.** The tie-break
probe fires **unconditionally** on the shipped default path (`latent_rate_controller.hpp:319-333`,
`kTieBreakPeriod=4`, `kTieBreakMarginFrac=0.15` at `:193-194`) with no env gate. It was
measured at **+16.5 % mean, sd 27.3 % over 7 pairs**, bimodal (+33..+39 % when it reaches
8PSK, −14/−27 % when it does not) — versus 14.0 % sd without it. The p=0.022 result that
justified shipping the controller is the **pre-probe** configuration
(`CHANGELOG:95,123`: "SHIP CANDIDATE IS THE PRE-PROBE CONTROLLER… The probe stays OFF"),
so the binary that is default-ON is not the variant that was validated.

Any relax sweep sits on top of that variance source, and the two are *coupled* — widening
the posterior lowers the 25th percentile (`kDecilePessimism=0.25`) and changes which rungs
fall inside the 15 % near-tie margin. **Gate the probe on posterior confidence first**
(`CHANGELOG:125-129` already names and argues this; it consumes `spreadDb`, the same
observable relax controls), then sweep relax at 0.15 / 0.35 / 0.7, n≥8.

---

## Full reference

### Rate control / ladder

| Knob | Default | file:line | What it does | Status | Evidence |
|---|---|---|---|---|---|
| `ULTRA_LATENT_RELAX_DB` | 0.35 dB, [0,5] | `connection.cpp:2627` | Forgetting rate of the latent posterior | UNDER-TESTED | Controller as a whole: +14.5 %, 8 pairs, p=0.022. Value never swept; named risk checked (sd 1.4-2.1 on rig) |
| `ULTRA_ENTRY_EVM_CAP` | OFF strict | `waveform_selection.hpp:445` → consumed `connection_handlers.cpp:376-398` | Caps the CONNECT-time entry rung using usable-domain EVM floors vs the data-aided connect reading; cap-only | NEVER-TESTED, **blocked** | No A/B. Blocked by `CHANGELOG:2793` pending the §3 anchor re-measure — precondition **unmet** (`kOfdmLegacyAnchorScaleOffsetDb = 8.70f` still live at `connection_policy.hpp:43`). Fights `ULTRA_CONNECT_AFFINE_BASIS` (default-ON, deliberately **slope-0**: `connection_policy.hpp:876-881`), applies a hard threshold with **no confidence gate** against a 3.14 dB sample σ and 1.1–3.1 dB rung spacing, and is absent from the manual-accept path (`connection.cpp:600-670`) |
| `ULTRA_ENTRY_QAM16_SNR` | unset = OFF | `waveform_selection.hpp:937` | Enter the ladder directly at 16QAM R2/3 on a Good channel above a dB threshold | MEASURED-HARMFUL | F73 (`KNOWN_BUGS.md:809`): a **cold** 16QAM entry decodes marginal (quality 0.35, no warm channel estimate) and its slow decode widened the handshake race, collapsing the ladder to R1/4. Mechanism-level, not a bps delta. Now *worse*: the latent controller seeds its prior from the entry rung |
| `ULTRA_MAX_OFDM_RATE` | unset = no ceiling; `R1_2`, `R2_3`, `R3_4` | `latent_rate_controller.hpp::latentConfiguredRungCeiling`; compatibility readers in `connection.cpp` and `connection_handlers.cpp` | Caps **automatic** coherent-ladder selection | FORCE/DEBUG | Fixed on the live path: the latent argmax, bounded startup probe and received authority commands share one absolute canonical-rung ceiling. It never overrides an exact forced profile. Invalid text still silently means no ceiling. The bootstrap/legacy-reader caveat is recorded below. |
| `ULTRA_FORCE_DATA_MOD` + `ULTRA_FORCE_DATA_RATE` | both unset = AUTO; either may be set alone | parser in `waveform_selection.hpp`; handshake in `connection.cpp` / `connection_handlers.cpp` | Negotiates and QSO-pins an exact modulation/rate measurement profile | FORCE/DEBUG | Parsed atomically, carried in CONNECT/CONNECT_ACK, and pinned at both endpoints even when configured on only one station. A valid partial override resolves its AUTO half normally; any present invalid field rejects the whole environment pair. Exact force outranks MAX. The cautionary result still applies: "+10.6 % unanimous over 2 pairs" became **+5.2 %, p=0.73 at n=8**; a forced run does not measure the shipped adaptive selector. |

#### Current forced-profile and MAX contract (2026-08-02)

This is a negotiation contract, not merely a local selector override:

* `forcedDataProfileFromEnvironment()` parses `ULTRA_FORCE_DATA_MOD` and
  `ULTRA_FORCE_DATA_RATE` together. If either variable is present but invalid, the parser
  marks the pair malformed, resets **both** fields to AUTO, logs the error at the handshake
  owner, and contributes no environment force or adaptation pin. A separate explicit
  `ConnectionConfig` force, if any, still stands. A valid one-field environment override is
  supported: the unset half stays AUTO until the ordinary recommendation/configuration path
  resolves it, after which the requested half is applied. It is never left uninitialized.
* The initiator snapshots the resolved config/environment fields into
  `outbound_forced_modulation_` / `outbound_forced_code_rate_` before serializing CONNECT.
  CONNECT retries reuse that attempt-scoped snapshot, so clearing or changing the process
  environment after the first send cannot mutate the requested profile.
* The responder first computes the ordinary automatic entry and its MAX/safety caps, then
  applies a valid responder-local environment force, and finally applies any explicit
  non-AUTO CONNECT fields. It serializes the **actual committed profile** in CONNECT_ACK.
  Duplicate-CONNECT and proactive ACK retries resend the cached ACK bytes, preserving both
  the profile and its provenance.
* A force on only the initiator therefore travels in CONNECT. A force on only the responder
  is echoed in CONNECT_ACK with `Flags::CONNECT_FORCED_PROFILE`. The initiator rejects a
  CONNECT_ACK that conflicts with any field it explicitly requested; the provenance bit
  covers the responder-only case, where the original CONNECT necessarily carried AUTO.
* Either explicit outbound fields or the CONNECT_ACK provenance bit establishes a
  QSO-scoped automatic-rate pin at **both** endpoints before `on_connected` can start data.
  The pin survives later environment removal and gates the latent selector/startup probe,
  receiver-authority commands, automatic escape/demote paths, and the legacy feedback
  controller through `Connection::rateAdaptationActive()`. Manual protocol actions remain
  manual; the promise is that no automatic rate actuator invalidates the measurement rung.
* Precedence is: automatic recommendation and safety/MAX caps, then exact local force, then
  explicit CONNECT fields at the responder; the committed CONNECT_ACK is never rewritten
  by an initiator-local MAX. Thus `ULTRA_FORCE_DATA_MOD=8PSK` plus
  `ULTRA_FORCE_DATA_RATE=R2_3` remains 8PSK R2/3 even beside
  `ULTRA_MAX_OFDM_RATE=R1_2`.

`ULTRA_MAX_OFDM_RATE` is intentionally weaker: it is an **automatic-selector ceiling**, not
a profile negotiation or lock. On the shipped latent/RX-authority route,
`latentConfiguredRungCeiling()` maps it to the absolute canonical QPSK R1/2, R2/3 or R3/4
rung and applies it to normal argmax, startup exploration, and sender-local resolution of a
peer authority command. Two bootstrap compatibility readers (manual accept and auto accept)
still perform the older code-rate-only comparison, and the reader inside
`applyAdaptiveRateFeedback()` belongs to the default-bypassed legacy sender controller.
Those legacy sites may be consolidated, but they are not evidence that the live ceiling is
entry-only, and they must not be used to delete the central latent/authority reader.

### ARQ, timeouts, and the ACK plane

| Knob | Default | file:line | What it does | Status | Evidence |
|---|---|---|---|---|---|
| `ULTRA_INFLIGHT_RTO` | OFF strict | legacy table in `connection.cpp`; legacy consumer in `selective_repeat_arq.cpp` | Sizes the old logical-window RTO from frames outstanding | SUPERSEDED — **closed by unconditional physical-egress timing** | The live MPG@20 trace proved the knob could not repair the motivating tail: a one-frame resend still inherited the stale scalar and repeated at ~23.15 s. The default path now finalizes every DATA egress from the exact serialized frame vector after padding, applies queue carry only to identities that actually go on air, replaces the tone-monitor deadline with the same round timeout, and timeout-suspends later holes until a subsequent physical round emits them. Padding counts toward airtime but cannot alias a slot because matching uses full serialized identity. Focused regressions cover adaptive-state override, singleton tail RTO, near-expiry multi-hole cap shrink, padding alias, synchronous ACK ordering, and monitor replacement. Keep this experimental table OFF; it is now a deletion candidate. |
| `ULTRA_ADAPTIVE_RTO` | OFF strict | `selective_repeat_arq_policy.hpp:255`; `:314-321`; call `selective_repeat_arq.cpp:2313` | Stops `configured_ack_timeout_ms` being the RFC6298 estimator **floor** as well as its ceiling (the legacy `clamp()` collapses to a point) | UNDER-TESTED, low | n=1 paired: 1.12 vs 1.50 kbps. Mechanism measured with a null control (53 `ADAPTIVE-RTO ENGAGED` firings): the adaptive floor moves the RTO by **0–20 %**, not the 7.5× the synthetic unit test implies, because measured srtt is 10.7–12.6 s. **Do not pair it with `ULTRA_INFLIGHT_RTO`**: where INFLIGHT acts (tail, table[1]=8.0 s) ADAPTIVE is inert (min(8.0, ~18)=8.0); where they interact, min(table[16]=47.7 s, ~20.5 s)=20.5 s sits **under the ~21.5 s burst airtime** — a guaranteed spurious mid-burst retransmit. Blast radius is wider than the RTO: it also scales the hole-probe timers (`:1301`,`:1462`), the DATA_REPAIR guard (`:1544`) and the post-fast-retx re-arm (`:1330`) |
| `ULTRA_KEEPALIVE_ACK` | OFF | `connection.cpp:5093` | Re-emit the cumulative tone ACK after `_MS` of silence | UNDER-TESTED → see [shortlist #4](#4-ultra_keepalive_ack--ultra_keepalive_ack_ms--connectioncpp5093--5104) | Reverted 2026-07-14 on a mechanism-level cause (re-emits re-assert the rung command) |
| `ULTRA_KEEPALIVE_ACK_MS` | **8000** ms, [3000,60000] | `connection.cpp:5104` | Silence threshold | sub-parameter | 8000 has a cap measurement with a two-point null control (36.7 → 25.0 → 8.0 s). No room below it: normal group cadence is 9-10 s. Comment at `:5096` says "default 25000" — **stale** |
| `ULTRA_ACK_REPEAT_SILENT_MS` | **OFF**; explicit 300–5000 ms enables | `app.cpp:3011` | Tc-decorrelated repeat of a distinct tone ACK after an unbroken quiet window; broad-signal hold is arm-relative | UNDER-TESTED → see [shortlist #3](#3-ultra_ack_repeat_silent_ms--srcguiappcpp3011) | No goodput A/B at any n; historical collision fixes plus 2026-08-04 arm-baseline regression |
| `ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR` | OFF strict; exact `1` enables per `Connection` | `connection.cpp` ctor, tone-ACK provenance/decision, `flushBurstBuffer`; `app.cpp` anchor reason | On descriptor-proven BI0 partial SACKs, keep the robust full BURST_HEADER but let its following DATA group use the warm light-LTS start and N8 budget. Receiver stamps CRC-covered reserved advisory=3 (old/off sender treats it as HOLD); sender requires exact current-round serialized identities to change from all-unacked to a strict partial set under unchanged mode/CW/Z. Crater, RTO, BI1, singleton, stale/prior ACK, transition, and unproven geometry fail closed to full descriptor + full DATA anchors. Shares the advisory field with software ALC, so campaign runner forbids combining them. | **MEASURED-WASH; ACQUISITION REDESIGN REQUIRED** | Order-balanced immutable MPG@20 A/B, pairs 2--9: all 16 files exact and teardown-clean; physical paired effect +1.468%, 95% CI [-14.006,+16.942]%, `p=0.8289`; sign 5/8, `p=0.7266`; log effect -0.119%. The 61/96 engagements mechanically removed 73.2 s (3.975% mean physical span), but ON/OFF goodput was unchanged and enabled arms saw 12 vs 4 craters. Pair 9 ON lost the entire callback for an engaged light repair after 1/8 delivery; void pair 1 ON repeated that shape after 3/6. An OFF arm also lost a full anchor, so this is a design warning rather than a causal count. Keep OFF; preserve acquisition diversity or fail closed on severe partials before any new trial. Valid-pair log SD 0.194 means n=8 cannot resolve the expected ~4% effect. Normal + ASan/UBSan focused tests pass after adjacent lifecycle fixes. |
| `ULTRA_ACK_CCA_DEFER_MS` | **2500 ms, DEFAULT-ON**, [0,10000] | `app.cpp:739` | Listen-before-ACK: defer a tone ACK while the channel reads busy; send on 3 consecutive quiet ticks or drop at the deadline on decoder evidence | UNDER-TESTED, **do not revisit as a throughput lever** | Enforces the half-duplex rule ("WE DO NOT TRANSMIT WHILE SIGNAL IS ARRIVING", `app.cpp:2962`). Four recorded failures were all the modem keying over inbound audio (F124/F127/F129/F176). Two accuracy notes: the comment claiming `=0` restores immediate-key is **wrong** since F176 (the condition is `(knob>0 && busy) \|\| air_rem>0`, so the geometry gate still defers unconditionally); and 2500 is a bare constant on a path whose every other timing term is ratiometric |
| `ULTRA_MC_ACK_REPEATS` | 0 = derived (3 on fading, 1 on AWGN), [1,3] | `connection.cpp:5447` | Pins the number of staggered MODE_CHANGE-ACK copies | NEVER-TESTED, low | No measurement. Motivating observation: up to 5 receptions per climb. Less urgent now — the latent controller cut mode changes to 2.2/run from 5.0. If a stall ever traces to a lost MODE_CHANGE ACK, measure repeats=1 vs 3 counting **round-trip completions**, not goodput |
| `ULTRA_MODE_CHANGE_RETRY_MS` | 0 = derived (~18.5 s wideband); [1000,60000] pins | `connection.cpp:5398` | Pins the MODE_CHANGE retransmit timer | FORCE/DEBUG | Worked as a bisect instrument: W4 recorded the defect (~74 s of a 328 s transfer in MODE_CHANGE dead air), W5 at ~5 s **livelocked** (72 receptions, no move committed), W5b at ~9 s stalled at 25 %, W6 pinned at 18500 ran clean (1.88 kbps). The fast-timer hypothesis was falsified. **`MAP:233` is stale** — it claims the default is the ~5 s ratiometric round trip; `connection.cpp:5425-5430` restored the `getAckTimeout()` floor |
| `ULTRA_SACK_SALVAGE` | OFF strict | `connection.cpp:379` | On a rate/CW-change abort, salvage the file byte ranges of discarded-but-SACKed slots so the requeue skips them (~1 s/transfer) | NEVER-TESTED, low | Disabled 2026-07-07 on one forensic incident (F181) whose attribution was later **refuted** (`KNOWN_BUGS.md:508-530`, `d5cbfb8`: the real destroyer was a non-monotone receiver byte map). But a second argument replaced it and holds: a sender-local decision to *permanently* never send a byte needs monotone, byte-domain, era-independent evidence, and `slot.acked` is frame-domain, era-relative and retractable. Gating it on trustworthy evidence makes it a no-op; receiver-authored byte evidence needs a wire bit and the tone ACK payload is saturated at 44/44 bits. Prize ~1 s; downside a stranded run |
| ~~`ULTRA_CONNECT_ACK_RESCUE_DEFER`~~ | removed | `connection.cpp` accepted-sync + peer-frame confirmation | Historical defer experiment; replaced by cache-only reactive duplicate-CONNECT replay | CLOSED → see [shortlist #5](#5-ultra_connect_ack_rescue_defer--removed-2026-08-03) | Old defer: 3/3 deadlocks; replacement has focused state-machine regressions |
| `ULTRA_DROP_RX_SEQ` | −1 = off | `connection.cpp:4542` | One-shot: drop the first receipt of DATA seq=N so selective repeat can be proven on demand | FORCE/DEBUG — **keep** | The only deterministic single-hole injector. Right regression instrument for the SACK/durability family (BUG-SACK-DURABILITY-RESIDUAL, `d5cbfb8`) |
| `ULTRA_ACK_MONITOR_GAPLESS` | **DEFAULT-ON** (`=0` opts out) | `streaming_decoder.cpp:219` | Closes a blind hole in the tone-ACK tail sweep: all cadence passes in one `feedAudio(count)` scan the same end-anchored window, so a large append leaves a permanent gap | correctness fix, closed | BUG-POSTTX-ACK-MISS (`KNOWN_BUGS.md:800`). Two alternatives refuted by a capture ledger (fade ruled out — tone arrived at rms 0.033-0.091; bin mismatch ruled out — all ACKs symbol_ms=12). 10-run batch F78-F87, 0 misses |
| `ULTRA_ACKLISTEN_SUPPRESS_OFDM` | **DEFAULT-ON** (`=0` opts out) | `streaming_sync_acquisition.cpp:377` | While the ACK monitor is armed, suppress warm DATA-sync acceptance so the ACK tone cannot false-lock the OFDM searcher | correctness fix, closed | **Half-duplex-provably safe**: the peer physically cannot be sending OFDM in our ACK window. ~280 ACK exchanges, 0 misses. `KNOWN_BUGS.md:805` still says default-OFF — **stale** |
| `ULTRA_ECHO_REANCHOR_GATE` | **DEFAULT-ON** (`=0` opts out) | `streaming_ofdm_decode.cpp:2672` | While our ACK monitor is armed, suppress the full re-anchor a spurious 0-CW decode would force (which would re-arm a 120000-sample blind search and make us miss the crater-demote tone ACK) | closed, +17 % | Rig F65-F68 with clocks aligned. `KNOWN_BUGS.md:812`: the "own burst echo" attribution was **wrong** (it was the peer's tone ACK); the fix keyed on the right state and stands. See R7 |
| `ULTRA_BURST_ESCALATION` | **DEFAULT-ON** (`=0` opts out) | `connection.cpp:5882` | After 2 consecutive clean groups at a **dense** rung, raise the burst airtime ceiling 8600 → 11500 ms (N=8 frames) | shipped winner | F208-F217, alternating ON/OFF, same hour and channel: **1.74 vs 1.40 kbps = +24 %**, 4/5 pairs positive, 10/10 PASS. Dense-rung gate added after streak-alone over-escalated on Moderate (moderate@16 ON FAIL 690 vs OFF PASS 1030). Sibling `ULTRA_BURST_ESC_STREAK` is **harmful on Moderate** (31 craters vs 2) — keep off. **Missing from the map** |

### Burst transport, sync, and anchoring

| Knob | Default | file:line | What it does | Status | Evidence |
|---|---|---|---|---|---|
| `ULTRA_CRATER_REANCHOR_HOLD` | OFF strict | `streaming_ofdm_decode.cpp:2691` | Two-crater discipline for the receiver's sync re-anchor | UNDER-TESTED → [shortlist #1](#1-ultra_crater_reanchor_hold--srcguimodemstreaming_ofdm_decodecpp2691) | F470-481 +44 %, re-anchors 25 vs 50; revert confounded 3 ways |
| `ULTRA_CHEAP_REANCHOR` | OFF strict, **and inert unless HOLD=1** | `streaming_ofdm_decode.cpp:2709` | On a crater: hold warm timing, roll CFO back to the last certified value (`certifyWarm()` fires unconditionally at `streaming_burst_interleave.cpp:1398-1399`), full chirp only every 4th | NEVER-TESTED, low | **Trap:** setting it alone is a strict no-op — the dispatch at `:2714-2721` makes `force_chirp` unconditionally true when HOLD is off, and the rollback lives in the unreachable else-arm. Target is only ~1.2 s/run once HOLD lands (HOLD already cuts re-anchors to ~1/run), i.e. **1.3–1.7 % of transfer time** — an order of magnitude below rig resolution. Also: the documented "1 chirp per 4 craters" is **wrong** — the streak resets on any decoded frame, so it is "no chirp unless 4 **consecutive**", and riding warm through 3 craters injects up to 3 extra `k=0` observations into the latent posterior. Fix the doc (`CHANGELOG:3674`, `MAP:412`); do not spend rig time |
| `ULTRA_COMMANDED_GEOMETRY` | OFF since 2026-07-29 (`4055831`), loose, non-latching | `streaming_decode_policy.hpp`; `streaming_ofdm_decode.cpp`; `streaming_burst_interleave.cpp` | Broad missed-descriptor commanded-rung reconstruction only. The independent truncation guard is unconditional. An exact same-rung Z27→Z81 restoration for already wire-learned BI0 8PSK R2/3 cw4 and QPSK R3/4 cw3 profiles is also independent and default-active. Descriptorless BI0 ignores stale configured `N`, searches under the same airtime/window ceilings that bind the sender, and finalizes only at a CRC-valid physical tail. BI1 fails closed because that flag is unavailable before the `N`-dependent deinterleave. | Broad arm remains UNDER-TESTED; narrow BI0 Z81 fix awaits live revalidation | Positive BI0 regressions prove full 7776/5832-LLR collection. N8-wire/stale-N7 crosses N7 and finalizes exact N8; the corrupt-tail pair engages recovery but emits no callback/ACK. Live BI1 N7/N8 has a no-arm/no-callback regression. A new byte-exact IONOS run is owed. |
| `ULTRA_BURST_INTERLEAVE` | OFF for all mods | `connection_policy.hpp:239` | TX permutation + whole-group ACK semantics + wire flag | UNDER-TESTED → [shortlist #2](#2-ultra_burst_interleave--srcprotocolconnection_policyhpp239) | The repaired Good@20 harness is strongly positive but simulator-only: QPSK R3/4 cw3/Z81 steady groups went 298/320 complete BI0 → 319/320 BI1, and full first/resend groups went 73/80 → 78/80; 8PSK R2/3 cw4/Z81 full groups went 60/80 → 72/80. This still conflicts with the historical real MPG@20 16QAM result: BI0 +37%, 5/5 vs 4/5 delivery, and 12 vs 62 craters. Keep OFF until matched, byte-exact IONOS evidence exists **per modulation/channel class**; do not infer a global boolean from the local screen. |
| ~~`ULTRA_BURST_ERASURE_ABSOLUTE`~~ | removed 2026-11-11 | `streaming_burst_interleave.cpp` erasure gate | Forced the legacy fixed 0.015 broadband-RMS erasure floor | **DELETED** (REMOVAL_BACKLOG R21) | `--rx-gain-db` tests the relative gate at any receive level; the fixed floor erased every frame below it |
| `ULTRA_BURST_RMS_DIAG` | OFF | `streaming_burst_interleave.cpp` `tryDemodulateNextBurstFrame` | Logs the erasure-gate inputs per collected frame | DIAGNOSTIC → [shortlist #7](#7-ultra_burst_rms_diag) | Owed since 2026-06-16 (`KNOWN_BUGS.md:1279`) |
| `ULTRA_HARQ_PROVISIONAL` | OFF (opt-in) | `streaming_decode_policy.hpp`; `streaming_ofdm_decode.cpp` `buildHarqKey`; `frame_v2.cpp` Phase F | On CW0 header-peek failure, key the HARQ soft-combine buffer by the ARQ-mirror predicted seq so CW0-dead frames can combine across retries | UNDER-TESTED; **8PSK cw4/Z81 IONOS A/B in progress** | The 2026-08-03 live 8PSK baseline exposed 12 CW0 key-build failures and repeated retry sagas (seq 3 needed four resends; seq 9/12/15 needed three each). The experiment now authorizes only existing QPSK R3/4 plus exact QAM8 R2/3 cw4/Z81; cw12/Z27, wrong Z/rate, BI, late descriptors, unknown/non-forward cadence, and true greater-than-15-second gaps fail closed. Tail headers may validate the mirror but tail frames cannot use predicted keys. A regression had conflated every sender-announced FULL repair anchor with an RTO gap, making the feature a structural null on selective repairs; cadence and anchor choice are now separate. Context is pulled before the first eligible peek, and at least one current-group real sender+seq match is mandatory before key use. The finalize guard compares source, destination, seq, CW count, and physical-tail state; ULPAD is excluded. A true combine miss no longer invokes fallback. A failed sum gets the exact normal 10-attempt fresh schedule, and a syndrome-valid-but-frame-CRC-invalid sum lazily runs the complete all-fresh frame baseline (including frame-level recovery) before failing. Provisional double-fails retain fresh rather than the suspect sum. Keep OFF until a same-freeze, order-balanced real A/B proves nonzero provisional keys, byte integrity, low mismatch, bounded decoder backlog, and a retry/goodput benefit. |
| `ULTRA_HARQ_SHADOW_FRESH` | OFF strict | `frame_v2.cpp`; `[HARQ]` / `HARQ_FRAME` telemetry | On a successful real combine hit, synchronously decode the exact all-fresh counterfactual with a separate LDPC cache and classify only after full header+frame CRC as `both_pass`, `combine_only`, or `divergent_valid` | DIAGNOSTIC ONLY | This is not a throughput arm: up to ten extra Z81 decodes per combined-success CW (plus first cache construction) run on the RX worker and can bias backlog/goodput. Use clean shadow=0 OFF/ON throughput arms, then a separate ON+shadow diagnostic. Counters are frame-level, provisional-success evidence is admitted only after the same source/destination/seq/CW/tail/ULPAD identity gate as HARQ finalization, and verified double-fails retain the accumulated Chase sum while provisional double-fails retain fresh. `shadow_decode` and `shadow_frame` expose separate count/total/max decoder-region timings (the outer fixed-frame timer remains end-to-end wall time); mandatory shadow-OFF CRC safety work is separately `lazy_decode`/`lazy_frame`. `fresh_rescue` includes CRC-valid hybrid frames; `allfresh_rescue` is the stricter full all-fresh subset. |
| `ULTRA_ANCHOR_SKIP_KEEP_STREAK_ON_SWITCH` | OFF strict, documented permanent | `streaming_encoder.cpp:687` | Preserve the #69 reactive-anchor-skip clean streak across a descriptor mode/rate switch | MEASURED-HARMFUL (closed at n=2) | Stopped on a pre-committed falsifier: chirp SKIP rate **7.7 % ON vs 28.9 % OFF** — the knob's entire purpose was to raise it and it more than halved it. Craters 2.0 vs 1.0/run, resends 1.0 vs 0.0. The recool was doing double duty as a **post-switch acquisition guard**. A future attempt is a different design (explicit post-switch holdoff). **Keep** the `anchor_reason` plumbing (None/Resend/ModeSwitch) — it is what made this measurable |
| `ULTRA_BURST_HEADER_ONCE` | OFF | `modem_engine.cpp:582` | Emit the BURST_HEADER descriptor only on `group_seq == 0` | MEASURED-HARMFUL, **delete** | `PHY_ADAPTATION_DESIGN_2026_05_26.md §14.32`. The gate worked (1 header sent); BRAVO then reported every inner group as `group_seq=0` and re-ACKed seq 0 forever. The header does **three** jobs — format, chirp/sync, **and group sequence number** — and (3) is required every burst. The claimed saving is illusory anyway: the header's airtime is almost entirely chirp+LTS. **KEEP** `ULTRA_BURST_DESCRIPTOR` — the descriptor is the only RX source of the LDPC lifting Z |
| `ULTRA_8PSK_LONG_LDPC` | OFF strict, transfer-scoped | `connection.cpp:42`; profile `:6950-7065` | For connected OFDM file bursts only, substitutes physical cw4/Z81 for the logical QAM8 R2/3 cw12/Z27 rung. Capacity and 7776 coded bits/frame are unchanged; BURST_HEADER announces both CW and Z. A one-frame tail/retry gains one addressed-away ULPAD frame so it cannot escape without a descriptor. **Both endpoints must carry the same flag: this experiment is not capability-negotiated.** The latent selector now prices cw4/Z81 geometry and switch-anchor airtime exactly, but its FER curve is still logical-rung/Z27 calibrated, so that does not graduate automatic Z81 selection | EXPERIMENTAL; REAL IONOS INTEGRITY PROVEN, PROMOTION UNPOWERED | The two-pair post-fix MPG@20 follow-up was byte-exact and directionally positive but below its predeclared n>=8 gate. A later immutable-build `8psk_z81_01` transfer was again byte-exact (51,200 B, 1.83 kbps application / 1.77 kbps physical span, 22/22/22 bursts and 22/22 ACKs), but had 9 partial groups and 2 craters; it is another sequential realization, not a promotion trial. The repaired Good@20 geometry screen favors cw4/Z81 over equal-capacity cw12/Z27 (2174/2400 vs 2113/2400 frames; 348/480 vs 323/480 complete groups) and disfavors shorter cw3/cw2. BI1 lifted a smaller cw4 sample to 753/800 frames and 147/160 groups, but that is simulator-only and conflicts with historical live 16QAM interleave evidence. The initial Z81 capacity/cache/HARQ defects and stale-lifecycle exits are repaired, and the current-group FULL anchor is now announced on wire, but the newest classic descriptor+head-loss recovery still has **no fresh live run yet**. Keep OFF pending matched-build, order-balanced real IONOS evidence and a profile-specific FER curve. |
| `ULTRA_QPSK_R34_LONG_LDPC` | OFF strict, transfer-scoped | `connection.cpp:51`; profile `:7005-7149` | For connected `OFDM_CHIRP` file bursts only, maps the normal unforced logical QPSK R3/4 cw8/Z27 rung to physical cw3/Z81. This is **not** equal airtime: capacity is 461→527 bytes and coded bits are 5184→5832 (+12.5%). Chunking, ARQ serialization, burst budget/RTO, and the BURST_HEADER all consume cw3/Z81; singleton tails/retries gain an addressed-away ULPAD so Z81 never escapes descriptor-less. **Both endpoints must carry the same flag; it is not negotiated.** Selector geometry/switch cost is exact, but the success curve is not yet cw3/Z81-specific | EXPERIMENTAL; REAL IONOS INTEGRITY PROVEN, ACQUISITION FIX NOT LIVE-VALIDATED | The immutable-build `qpsk_r34_z81_01` MPG@20 transfer was byte-exact (51,200 B, 1.85 kbps application / 1.64 kbps physical span, 23 bursts sent / 22 decoded, all 22 committed ACKs accepted). Its first descriptor was lost to the now-fixed future-training clamp; a later 0/5 was a separate real near-0 dB fade, for 13 repaired chunks total. After repairing `measure_ack_fer`, Good@20 steady cw3/Z81 groups reached 1567/1600 frames and 298/320 complete BI0 versus 1595/1600 and 319/320 BI1; full first/resend groups were 390/400 in both arms and 73/80 versus 78/80 complete. This extends the earlier 390/400 matched-frame screen but remains simulator evidence. A new immutable QPSK run after the acquisition/CFO fixes completed byte-exact at 1.712 kbps physical goodput with 100/118 acquired unique-frame attempts successful and no timeout/crater; it was still decisively slower than the paired-profile 8PSK runs. Keep OFF pending matched evidence and a profile-specific FER curve. |
| `ULTRA_WARM_TURNAROUND_OFF` | knob OFF ⇒ **feature DEFAULT-ON** | `modem_engine.cpp:1185` | Setting it to 1 **restores the slow behaviour** | FORCE/DEBUG — **do not set** | The feature is rig-proven on two classes: Good MPG@20 turnaround 2.71 → **1.54 s (−43 %)**, Moderate MPM@20 1.59 s with zero burst-timeout stalls. The old per-ACK echo-clear called `clearRxBuffer` → `reset()`, wiping warm-sync state every turnaround. The name reads like something you would switch on to *get* warm turnaround. Keep the knob — the faithful gate structurally cannot regression-test this (the sim TX returns before the echo-clear, `app.cpp:3040`), so it is the one-line bisect if a rig regression implicates turnaround |

### Channel estimation, LLR, and RX front end

The three closed nulls in this group (`ULTRA_LTS_DFT_DENOISE`, `ULTRA_CHEST_NOISE_SCALE`,
and the COH/MIN pair) are the reason several other estimator knobs rank low. Read
`CHANGELOG:1203-1265` before proposing anything on the channel-mean axis.

| Knob | Default | file:line | What it does | Status | Evidence |
|---|---|---|---|---|---|
| `ULTRA_LTS_DFT_DENOISE` | OFF | `ofdm_demodulator_setup.cpp:34` | Gaussian across-carrier smoothing of the LTS channel estimate | **MEASURED-WASH — the family null control** | Both halves measured. The estimate really improves: truth-referenced NMSE 0.02675 → 0.00660 = **4.05×** (QPSK R3/4), 3.97× (16QAM R2/3). That 4× buys nothing: ITU Good, 4 seeds × n=60 = **240 frames per cell**, eight cells across 11/14/20 dB, deltas +1/−1/0/+3/0/0/0/+2 against 1σ≈7.7. Low-SNR cells are exactly zero. Conclusion: **channel-MEAN estimation is not the throughput limiter**. Explicitly *not* refuted: estimator **uncertainty** semantics (`error_var` drives modelled uncertainty toward zero in a notch — a reliability-*weighting* lever, still unexplored) |
| `ULTRA_LTS_DFT_DENOISE_TAPS` | 0 → effective **W=2** | `ofdm_demodulator_setup.cpp:36`; used `channel_equalizer_lts.cpp:688-692` | Gaussian half-width W (also sets σ = max(0.5, W/1.5)) | UNDER-TESTED, none | Parent is a measured wash, so tuning its width cannot recover an absent benefit. **Free action:** reconcile the contradiction — `channel_equalizer_lts.cpp:667-669` says "harness optimum for Good is W~3, SNR-independent"; `:688`, 21 lines later, says "W=2 is the harness-proven SAFE width (W≥3 smears deep nulls → 16QAM poison)". Both cite the same harness; one is wrong |
| `ULTRA_CHEST_NOISE_SCALE` | OFF, non-latching | `channel_equalizer_lts.cpp:28` | Corrects a real **3.01 dB** scale error: `lts_noise_var = σ²_bin/2` (the averaged estimate's variance) is fed to the MMSE denominator, the LLR σ² and the erasure gate, all of which want per-symbol σ²_bin. The meter path already doubles; the decode path does not | MEASURED-WASH | ITU Good@20, 4 seeds × n=100 = **400 frames/cell**: QPSK R3/4 93.5→93.5, 8PSK R2/3 82.2→81.0, 16QAM R2/3 51.0→49.8, R3/4 27.8→26.8 — all inside σ≈2. Plus a four-arm discriminator (base / noise-fix / LTS-freeze / both) that refuted the mis-scaled-MMSE hypothesis. **Do not "clean up" the inconsistency** — it is kept default-OFF precisely so the meter-vs-decode 2× disagreement stays recorded |
| `ULTRA_ITERATIVE_CHEST` | OFF, non-latching | `iterative_chest.hpp:93`; armed `streaming_burst_interleave.cpp:747-752` | Post-FEC data-aided channel estimation carried **forward across frames** of a burst group: re-encode a fully-verified frame via the production path, push H_dd = Y/X into the existing Wiener | **MEASURED-WASH by family null** | Mechanism is sound and measured (2592 accepted observations/frame; re-modulation bit-exact over 576 configurations; Pi5 CPU 0.49-1.40 % of frame airtime; the inter-frame common phase is real, −75° to +23°). Benefit is not: the gui_qso A/B was 4 seeds of which **2 are unusable** to BUG-GUI-GATE-EARLY-EXIT-FLAKE → n=2, +4.9 %, inside noise, with an explicit "do NOT quote it as a win". It is a channel-mean lever, and `ULTRA_LTS_DFT_DENOISE`'s strictly larger 4× improvement bought exactly zero FER at n=240/cell. Upper bound: at ITU Good, ρ = J₀(2π·0.1·0.35) ≈ 0.988, so a one-frame-old *exact* observation still carries 1−ρ² ≈ 0.024 irreducible error ≈ the entire measured baseline NMSE. Removal scope with an anti-footgun KEEP list is already in `REMOVAL_BACKLOG.md:287` |
| `ULTRA_LTS_CFO_SEED_OFF` | OFF loose, non-latching | `channel_equalizer_lts.cpp:469` | Bypasses the seed/CV half of the residual-CFO gate | **contested** → [shortlist #6](#6-ultra_lts_cfo_seed_off--srcofdmchannel_equalizer_ltscpp469) | Moderate +31/320 (3–5σ) at n=80/cell; Good −7/320 consistently negative; goodput arm n=6 |
| `ULTRA_LTS_CFO_COH` | 0.70, (0,1] | `channel_equalizer_lts.cpp:450` | Coherence threshold for accepting the LTS residual CFO | **MEASURED-WASH (mechanism-level null control)** | Swept 0.70 / 0.85 / 0.93 → **identical results**. Designed to test "the harmful Good corrections are low-coherence garbage" and **refuted** it: they are coherent |
| `ULTRA_LTS_CFO_MIN` | 0.3 Hz, [0,5) | `channel_equalizer_lts.cpp:484` | Minimum residual magnitude worth applying | **MEASURED-WASH (mechanism-level null control)** | Swept 0.3 / 1.0 / 1.8 Hz → **identical results**. The harmful corrections are also **large**. Together with COH this is a positive finding: the Good cost is a genuine, coherent, large rotation whose correction still hurts → the successor is **persistence** weighting, not thresholding |
| `ULTRA_LTS_CFO_AVG` | OFF | `ofdm_demodulator_setup.cpp:39` | Phase-align the N LTS symbols to the last symbol's frame (ML common rotation) before averaging, giving ~−3 dB LS noise; also de-biases the LTS noise estimate on the same alignment | NEVER-TESTED, none | Estimator-level only ("~−3 dB across SNR 8/12/16/20", `tools/lts_estimate_mse.cpp:255-279`). Both of its axes are covered by larger measured nulls: the −3 dB channel-mean axis by DFT_DENOISE's 4×, the σ² axis by CHEST_NOISE_SCALE's full 2× at n=400. And on Good the true offset is ~0, so the de-bias term |H|²(2−2cos φ) vanishes by mechanism. On the rig the *refine* is more likely to fire (chirp CFO ≥0.75 Hz becomes true with ppm offset), which re-estimates from scratch and erases the knob's rationale. `MAP:293` still parks it as "FEAT (task #9, in-flight)" since 2026-05-30 |
| `ULTRA_REL_FADE_MAX` (+`_ONSET` 0.25) | 30.0 = 15 dB cap, [1,1000] | `channel_equalizer_equalize.cpp:112` | Caps the LLR noise-variance inflation applied to carriers in relative fades (inflation = onset/rel, clamped) | NEVER-TESTED, low | No measurement. Both constants were tuned for the **dense 12-pilot** baseline and made env-tunable for a **sparse-pilot** experiment that may never have run — production runs pilots ~every 10 carriers with per-symbol LS tracking, so the premise may not apply. Also: LLR calibration is a recorded **exhausted** lever (≤1.21×, double-counts `softGrayZoneNoiseInflation` immediately below in the same file). If reopened, run `ULTRA_NULL_DIAG` first and read the per-depth-bin reliability — measure, then tune |
| ~~`ULTRA_RX_AGC`~~ | removed 2026-11-11 | `streaming_decoder.cpp` `feedAudio` | Slow, amplify-only, deadbanded RX level normalizer on the ring path | **DELETED** (REMOVAL_BACKLOG R21) | Its benefit was never measured, and its targets are gone: the absolute sync clamps, erasure backstop, tone-ACK floor and the rest no longer read the level (BUG-RX-ABSOLUTE-LEVEL, 2026-11-10) |
| `ULTRA_LDPC_MAX_ITERS` | unset = per-rate table (R3/4 60, R2/3 70, R1/2 80, R1/3 60, R1/4 50); [10,1000] | `ldpc_codec.hpp:98` | Overrides the BP iteration cap for all rates | NEVER-TESTED, low → **R6** | The "21 of 26 failures at exactly 70" signature is a tautology (one non-cap exit, and it is success). A strictly stronger experiment is already default-on: `frame_v2.cpp:2167-2270` runs up to 12 extra full BP passes (4 factor + 5 perturbation + 3 fresh) "to break trapping sets from multiple angles", and logs per-CW `RETRY OK (factor=…, iters=…)`. The ~1000× headroom is measured on the *clean* population (median 4 iters); on the affected population it is ~4×, and `frame_v2.cpp:2206-2211` already records a measured backlog regression from too many retries ("200 ms per failed frame → 5-10 s audio backlog → sync degradation"). Downside is not bounded at CPU: `kIterRef=80` is hardcoded at `streaming_burst_interleave.cpp:1377-1380`, so a *successful* group above 80 iterations grades **quality = 0.0**. `CALIBRATION_AUDIT.md:861-864` already says "raising the iteration ceiling is not supported by this evidence" |

### TX PAPR and level

| Knob | Default | file:line | What it does | Status | Evidence |
|---|---|---|---|---|---|
| `ULTRA_ACE_PAPR` | OFF loose, **latching** | `modulator.cpp:285` | Active Constellation Extension on data symbols | UNDER-TESTED → [shortlist #8](#8-ultra_ace_papr--srcofdmmodulatorcpp285) | Unit-proven; rig n=3 inconclusive; CPU measured negligible; gain ceiling ~1.06 dB; collides with default-ON ALC clip detection |
| `ULTRA_COHERENT_PAPR_DB` | 0.0 = OFF; [4,12] sets the soft-clip threshold | `streaming_encoder.cpp:86` | Bounded soft-clip of the coherent TX waveform; also flips `paprReductionWouldCorruptCoherentPilots` | MEASURED-HARMFUL, **do not re-test in this form** | Sim: 9 dB target → 181 deinterleave-fails vs 30 linear; 12 dB → goodput 2210 → 1320. Rig (interleaved, MPG@30): **channel-state-dependent — +39 % marginal, −58 % clean, net −12 %**. Both signs have a named mechanism (clipping buys power where power binds; buys EVM where EVM binds), which is what legitimately closes it. The 4.5 dB power gain is real and **hardware-only**. Successor filed: an **SNR-gated** clip firing only when RX usable is within ~2 dB of the constellation floor — needs peer-margin plumbing that does not exist |
| `ULTRA_SIM_PAPR_PENALTY` (+`ULTRA_SIM_TX_PEAK`) | OFF, latching | `app.cpp:3625` / `:3633` | Make the sim TX peak-normalize like hardware | FORCE/DEBUG → [shortlist #9](#9-ultra_sim_papr_penalty--srcguiappcpp3625--ultra_sim_tx_peak-3633) | Confirmed its own mechanism and **disproved** the simple story it was built for (even ON, the sim does not reproduce the IONOS stall — the missing deficit was the cheap-card anchor-to-data gap, which OTASim does not model) |

### Simulator and offline harness

| Knob | Default | file:line | What it does | Status | Notes |
|---|---|---|---|---|---|
| `ULTRA_CHANNEL_DELAY_MS` (+`ULTRA_CHANNEL_DOPPLER_HZ`) | unset = model default; **presence-only, `=0` also sets `multipath_enabled=false`** | `ota_channel_core/channel.cpp:184` | Overrides the sim delay spread | DIAGNOSTIC / test apparatus | `DELAY_MS=0 DOPPLER_HZ=0` gives **truth H = 1 at every carrier** — an identity control. If an estimate-vs-truth comparison is still wrong under it, the fault is in the **tool**. Worth documenting prominently: this project has already had an analyzer that manufactured a 2.5 s latency figure from two hardcoded constants |
| `ULTRA_MEASURE_BURST_FULL_ANCHOR` | OFF | `tools/measure_ack_fer.cpp:1184` | Passes a per-request full chirp+LTS option for each burst chunk's DATA group start | FORCE/DEBUG — **do not promote to default** | It reproduces production's **resend / session-first** DATA-anchor regime, not the steady warm handoff. The request option cannot be consumed by the preceding descriptor. The harness now pumps one 4800-sample idle interval through the real channel before the descriptor so a fresh decoder does not skip a chirp beginning at absolute zero, and expected frames use the production `PHYSICAL_BURST_END` stamping policy; those are unconditional fidelity repairs, while this knob remains the explicit FULL-vs-LIGHT regime selector. Descriptor-FULL + DATA-FULL screens also exercise the earlier-complete-candidate recovery because both chirps can occupy one detector window. Do not turn it into a shipping modem default. |
| `ULTRA_MEASURE_BURST_NO_ANCHOR_WAIT` | OFF | `tools/measure_ack_fer.cpp:1102` | Skips the harness's `expectFullOFDMAnchorOnce()` demand so a light-LTS group start (corr ~0.26 vs the 0.52 threshold) is accepted | FORCE/DEBUG | The RX-side alternative to the above. Use to isolate "acquisition-threshold artifact" from "geometry/decode problem" |
| `ULTRA_CHEST_DELAY_SCAN` | OFF, **presence-only (`=0` enables)** | `tools/measure_ack_fer.cpp:752` | Prints residual-vs-delay over ±2560 samples in 64-sample steps | DIAGNOSTIC | The production fine search (±1200 at 0.5-sample steps) runs regardless. The coarse scan is how you would detect the fine search converging to a local minimum — its window is **narrower** than the coarse one. Hygiene: normalise the enable test |
| `ULTRA_CHEST_NMSE_DUMP` | OFF, presence-only | `tools/measure_ack_fer.cpp:689` | First frame only: dumps estimate-array geometry plus \|est\| at **deliberately unoccupied** bins (0, 100, 500, 900) and adjacent carriers | DIAGNOSTIC | A well-built sanity check: energy at an unoccupied bin means the array is not what the caller believes. Guards the same class of assumption error that produced a real stale-geometry mistake (12.67 ms/symbol from the 512-FFT spec on a 1024/128 modem — actual 24.0 ms) |
| `ULTRA_LOG_LEVEL` | unset | `tools/measure_ack_fer.cpp:1275` | Harness verbosity | DIAGNOSTIC | Related and actionable elsewhere: `tools/gui_qso_scenario.sh:497,507` **hardcodes** `--log-level debug --log-category all`, BRAVO emits ~11700 DEMOD lines/run, and that throttles the real-time sim below wall-clock pace. Identical in both arms so A/B validity holds — but it caps seeds per session, which is a direct mechanical contributor to why several knobs here sit at n=3-6 |

### TNC / host interface

| Knob | Default | file:line | What it does | Status | Notes |
|---|---|---|---|---|---|
| `ULTRA_TNC_ACCUM_DISABLE` | **accumulation ON**; `getenv(...) == nullptr` ⇒ **any value, incl. `0`, disables** | `tnc_session.cpp:177` | Opts out of flush batching, reverting to legacy per-chunk flush; also clears the sticky-transport ordering latch | correct by first principles; **hazard** | Default-ON is right on the half-duplex constraint (fewer turnarounds), verified functionally (20 KB JPEG byte-identical, clean teardown). **Hazard:** clearing the sticky latch while `route_burst` (`:517-518`) still bursts any single flush > 4096 B means a host that writes one >4 KB block then a smaller tail reassembles **out of order** against the bursted prefix — the exact corruption the ordering invariant at `tnc_session.hpp:104-108` exists to prevent |
| `ULTRA_TNC_ACCUM_PROBE` | OFF, presence-only | `tnc_session.cpp:178` | Measures accumulated body size per flush — and **forces `bulk_accum_ = true`**, then `std::_Exit(0)` at the first burst-routed flush, before any air TX | FORCE/DEBUG, **not diagnostic** | It **silently overrides `ULTRA_TNC_ACCUM_DISABLE`**. Anyone setting both — natural when A/B-ing accumulation while measuring body size — gets bulk mode in *both* arms and a null result that reads as "accumulation does not matter". Never leave it in a rig launch script |
| `ULTRA_TNC_FLUSH_LOG` | OFF, presence-only | `tnc_session.cpp:179` | Per-flush logging (why each body flushed: burst-worth vs host idle) | DIAGNOSTIC | The true no-side-effect observer for the live path, and how you verify batching is not degenerating to per-chunk because the host trickles |
| `ULTRA_EXPERT_PHY` | OFF | `tools/ultra_tnc_config.cpp:413` | Env alias for `--expert`, applied before argv parsing so an explicit flag composes | FORCE/DEBUG | Exists because the TNC runs headless (service, or launched by Winlink/PAT) where injecting argv is awkward. Document next to whatever `--expert` gates |

---

## Superseded / deletion candidates

These are cleanup, not options. Every one sits downstream of a default-ON bypass, or its
only reader is gone. Route them through `docs/REMOVAL_BACKLOG.md`.

| Knob | file:line | Why it is dead | Anti-footgun: what must NOT be over-cut |
|---|---|---|---|
| `ULTRA_TRUST_LADDER_PICK` | `connection.cpp:3048` | ~400 lines **after** the latent early return at `:2596`. Its hypothesis *won* — the latent controller shipped the same thesis (stability in the estimator, not the actuator) at +14.5 %, p=0.022, vs this arm's +20.7 % (n=6, sign p=0.69) | The F145 lesson: **never do raw arithmetic on rung indices** — disabled anchor rows are holes; snap via `snapRungIndexDownToEnabled`. Keep the enabled-ladder snap and catastrophic-crater escape as the audited-minimum safety set |
| `ULTRA_RUNG_DWELL_MS` | `connection.cpp:3113` | Unreachable (after `:2596`); measured **−13.8 %**; and its premise was refuted — the −0.58 correlation was **reverse causation** | The physics note is correct and worth keeping: the loop runs *inside* the coherence time (MPG Tc ≈ 4.2 s vs an 8-10 s burst), and "crater ⇒ rung too high" is a category error on Rayleigh. 8PSK R2/3 wins on Good@20 **including** its crater rate |
| `ULTRA_RUNG_CLASS_ANCHOR` | `waveform_selection.hpp:597` | **Dead code** — sole reader `rungPredictedSustainable()` has no production caller since the predictive climb was retired (`connection.cpp:2816`); only `tests/test_rx_authority.cpp` calls it. Also measured harmful with a structural cause: the Good-vs-AWGN anchor gaps are non-uniform (QPSK R3/4 +5 dB, 8PSK R2/3 +1 dB) and the climb scans highest-first, so it **inverted the ladder ordering** (12-CW frame usage tripled, 15 → 48) | `rungClassAnchorDb` itself has another caller at `connection.cpp:2770`. And record the finding underneath: the anchor table's Good column claims 8PSK R2/3 holds at 17.0 while QPSK R3/4 needs 20.0 — the two columns are **different kinds of quantity** (throughput crossover vs FER floor) and must never be mixed |
| `ULTRA_FER_FLOOR_ANCHOR` | `waveform_selection.hpp:610` | Same dead reader. Unlucky: it was the *correct* successor, built to dodge the inversion, and its consumer was retired ~2 days later. Never measured | **KEEP `kMeasuredFerFloor` / `rungFerFloorDb`** (`waveform_selection.hpp:538-564`): real data, 9 rungs × 3 classes × SNR 6-28 step 2 × seeds {7,11} × n=40 = **n=80/point**, pinned by `tests/test_waveform_policy.cpp:480`. Includes the honest finding that Moderate has an SNR-independent FER floor. It is the measurement to fit a per-class θ_r from |
| `ULTRA_DENSE_FAST_DEMOTE` | `connection.cpp:77` | After `:2596`; **and** the asymmetry it countered is gone — its whole justification was unwinding the predictive climb's multi-rung jumps (measured idx 3→8), and that climb was retired. A one-rung-up/one-rung-down loop does not have this failure mode; enabling it would only weaken the F122 two-crater grace | The F122 grace itself: a single crater at a ~10 s decision quantum against Tc 2-4 s is an irreducible deep null, not rate evidence |
| `ULTRA_RX_EMA_HOLD` | `connection.cpp:58` | Half (the censored ring feed, `:2500`) still executes but the ring is **never read** by the latent controller (which logs "NO SNR CONSUMED"); half (the hold gate, `:2760`/`:2893`) is after the return. Measured twice at 6 pairs with opposite verdicts (+8.5 % floor-improving vs aggregate wash with 2/6 failures = over-holding a failing rung) | The censoring **insight** is correct and was independently vindicated — a failed group feeds no fresh SNR so the ring reads a stale crest (26-32 dB on a 20 dB channel). The latent controller solves it structurally by fitting to outcomes, so no censoring rule is needed |
| `ULTRA_EVM_DEMOTE` | `connection.cpp:94` (clamp `:2934-3006`) | After `:2596`. Measured harmful with a mechanism-level root cause: it **doubled** the churn it existed to remove (15 vs 7 rung commands/run), because the ladder steers on dial-equivalent SNR while the clamp reads usable dB — measured mean disagreement **14.3 dB**, an irreconcilable second driver (the F128 pattern) | **Keep the estimator and the floor table.** The plumbing is already unconditional (`setBurstEvmObservation` fires whenever `modem.hasLastEvmSnr()`, `modem_protocol_binding.hpp:139-145`), so `burst_obs_evm_snr_db_` is free to any future consumer. The forward path is EVM as an **input** (e.g. the latent prior seed), never an output clamp. The 2026-07-24 "+21 % on 3 pairs" note is **superseded — do not re-cite** |
| `ULTRA_EVM_DEMOTE_CONFIDENT` | `connection.cpp:2965` | Inside the above block. Measured harmful at n=2 **with a null control that proved the gate worked as designed** (8.0 firings/run vs 0) and the outcome still degraded — churn 17.0 vs 7.5 | **Keep the form in the record:** `mean + 1.645·se < floor − margin` over a ring is correct for *any* rung decision fed by a σ≈3 dB estimator against 1.2-3.1 dB spacing; z=1.645 is a probability constant, same family as −ln(0.05) and ln(9). Reuse it if the tie-break probe is ever gated on confidence. Keep `tests/test_rx_authority.cpp:616-657` |
| `ULTRA_LINEAR_SNR_RING` | `connection_policy.hpp:92`; ring `connection.cpp:2519-2534` | Decision-inert: the ring executes but `snr_avg` is unread by the latent controller, and `ofdmAnchorScaleOffsetDb()` now has exactly one production consumer (`modem_protocol_binding.hpp:129-138`) whose value is used only as a freshness gate. Measured wash: 4 pairs, −2.5 %, sign p=0.688, with sizing independently confirmed neutral (ladder input 23.7 vs 23.6) | **Keep the code and the KNOWN_BUGS entry.** The underlying defect is real and large: per-frame dB-mean 13.89 vs power-mean 16.42 over 1023 readings = **1.74 dB**, the single largest identified term in the ~5.4 dB sim-vs-rig discrepancy against a measured analog loss of 0.36 dB. Any future SNR measurement — the §3 anchor re-measure, a per-class θ_r fit — **must be run in the linear-power domain** or it bakes the Jensen error into the new table |
| `ULTRA_GOODPUT_RATE` | `goodput_rate_controller.hpp:91`; path `connection.cpp:2664-2720` | Its early return sits **below** the latent one, so it needs `ULTRA_LATENT_RATE=0` to run. Same thesis, better implementation shipped: n=3, +8.0 % mean, spread −24 % to +43 %, self-described "NOT a result" | Two findings to salvage into notes: (a) its demote rule credits the rung below with f=1, inherited from a helper whose comment calls that "deliberately conservative" — right for crater regrading, a systematic downward bias as a primary rule; (b) **η(16QAM R1/2) = 2.000 and η(8PSK R2/3) = 2.001** — 16QAM R1/2 is a **dominated rung**, buying zero throughput for a denser constellation and worse PAPR (pinned by `test_dominated_rung_is_always_left`). That fact outlives any controller |
| `ULTRA_QAM16_CALM_FADING` | `connection.cpp:4010` | On the **sender's** `applyAdaptiveRateFeedback`, whose sole production call site is the `else` arm at `connection.cpp:2000-2013` — dead since `rxRateAuthorityEnabled()` went default-ON on 2026-07-05, **~26 hours after the knob was written**. It also gates the *wrong* hop: with the PSK8 ladder default-ON the climb target is QAM8 R2/3, not QAM16, and the QAM8→QAM16 step returns before `:4010`. The over-commit it would block was fixed upstream in the rung table (QAM16 R2/3 Good anchor 20.0 → 26.0, measured 51.4 % FER) | The sender-side dense-climb block dies together (`qam16_clean_streak_`, `qam16_r34_clean_streak_`, `qam16_reclimb_cooldown_`, `ULTRA_QAM16_CLIMB`, `ULTRA_QAM16_R34`, `ULTRA_R34_CALM_FADING`) and its **legacy** `ULTRA_MAX_OFDM_RATE` reader can go with it. **KEEP:** `latentConfiguredRungCeiling()` plus the responder/manual-entry MAX readers; those are the live automatic-ceiling contract. The ACK-silence escapes (collapse escape, stuck-frame escape, RTO machinery) are not in this arm; `last_group_quality_` (`:3706`) still feeds the GUI "Adapt:" bar. **Sequence after** the `ULTRA_LATENT_RATE=0` fallback is dropped in 0.5.2 |
| `ULTRA_UNIFIED_SEQ` | comment-only: `connection.cpp:5816`, `streaming_burst_interleave.cpp:242`, `modem_engine.cpp:530` | **The getenv is gone.** `kUnifiedSeqEnabled()` returns `true`; ~1000 lines of `BurstStopAndWaitController` were deleted | Rewrite the three comments without the dead knob name — a reader grepping it today concludes the unified path is optional. **Preserve** the fast-NACK note at `streaming_burst_interleave.cpp:242` and the burst-sizing note at `modem_engine.cpp:530` |
| `ULTRA_TONE_ACK_INTERACTIVE` | comment-only: `connection.cpp:401` | Same merge, same commit | The merge had to explicitly re-wire `applyAdaptiveRateFeedback` into the unified ACK branch. That is the same now-dead sender arm above — do not treat the comment as evidence the path is live |
| `ULTRA_WARM_TURNAROUND` | not a knob | Prefix false positive; the real knob is `ULTRA_WARM_TURNAROUND_OFF` (`modem_engine.cpp:1185`) | **Methodology note:** a plain substring diff of knob names against the map produces prefix false positives. Of the 61 "undocumented" knobs in the audit set, **seven are not env knobs at all** — this one, `ULTRA_HAVE_LIBHAMLIB`, `ULTRA_OTASIM_AUDIO_DIAGNOSTICS`, `ULTRA_TNC_TESTING` (compile-time `#ifdef`/CMake), `ULTRA_PHY_DIAG_LOG` (a log-format name in a Python tool's help text), plus `ULTRA_UNIFIED_SEQ` and `ULTRA_TONE_ACK_INTERACTIVE` above. True count: **54** |

---

## Diagnostics

Pure logging/instrumentation. No behaviour change; useful when debugging, never shipping
candidates. Listed so the map gap can be closed.

| Knob | Default | file:line | What it shows | Why keep it |
|---|---|---|---|---|
| `ULTRA_NULL_DIAG` | OFF loose | `channel_equalizer_equalize.cpp:593` | Per-relative-depth-bin reliability: count, errors, thermal noise, eps_H contribution, total | The right instrument for the only open question about `ULTRA_REL_FADE_MAX/_ONSET` — do carriers in relative nulls actually get high noise variance, or stay overconfident? **Run before tuning** |
| `ULTRA_PSYM_DIAG` | OFF loose | `channel_equalizer_pilot.cpp:615` | Per-symbol carrier-averaged \|H\| at symbol cadence — \|H\|(t) for within-frame coherence | Frame boundaries recovered by interleaving with COH-DIAG in the same log. **Trap:** two different `coh=` fields exist; filter to the RX-AUTHORITY verdict line |
| `ULTRA_COH_DIAG` | OFF loose | `streaming_sync_acquisition.cpp:178` | Per-frame Doppler-discriminator inputs (snap, h_mag, lts_noise_var, lts_sig_pow), **only when a snapshot is admitted** | A **starvation detector**. The 2026-07-25 bug — 0 snapshots on the burst path, `coh=0.00` on 157/157 verdicts, every rate decision falling back to the blind CV — would appear here as silence. Easy to reintroduce (any change to the admission gate), hard to notice, because a starved discriminator does not error. Related durable fact: the across-carrier fading index converges to ~0.523 for **any** Rayleigh channel, so it is blind by construction as a class discriminator |
| `ULTRA_RX_LAG_DIAG` | OFF strict | `streaming_burst_interleave.cpp:591`, `streaming_decoder.cpp:622` | `[BURST_DRAIN]`: next_available ms, ratio to `burst_min_block_`, frame index | Encodes a **decision rule**, not just numbers: ratio ≥2 ⇒ the frames already arrived and per-wake serialization is the lag (drain is a lever); ratio ~1 while waiting ⇒ inherent airtime, no drain helps. One run answers it. RX lag is not cosmetic — running behind live **defeats warm-sync** |
| `ULTRA_GAMMA_DOMAIN_LOG` | OFF, **presence-only (`=0` enables)** | `streaming_burst_interleave.cpp:512` | Per-carrier gamma before/after in-band renormalization | **Last surviving reader** of a vector whose real consumer (the predictive climb) is gone; the +8.70 dB markup here was deleted 2026-08-01. Consolidation candidate — but **sequence after** the legacy ladder deletion, since the constant still lives on the `ULTRA_LATENT_RATE=0` path |
| `ULTRA_HARQ_DEBUG_SEQ` / `_CW` | −1 = no filter | `soft_combine.cpp:35` / `:39`; also `frame_v2.cpp:80` / `:84` | Filters the HARQ soft-combine debug stream to one seq and/or codeword (AND'ed) | Inert unless `harqDebugLogEnabled`. The only way to read HARQ logs at burst volume |
| `ULTRA_DUMP_CFO_PREFIX` / `_CALLS` | unset = off / 1 dump | `channel_equalizer_baseband.cpp:33` / `:37` | Dumps CFO-correction baseband buffers to disk; call cap enforced by atomic CAS | CFO handling has an explicit invariant set (`docs/CFO_CORRECTION_FLOW.md`); capturing the actual baseband a correction was applied to has standing forensic value |
| `ULTRA_QAM16_GENIE_TIMING_CFO` | OFF loose | `streaming_ofdm_decode.cpp:102`, `streaming_sync_acquisition.cpp:40` | Injects **true** timing offset and CFO | Attribution oracle |
| `ULTRA_QAM16_GENIE_CHANNEL_TWOPATH_LS` | OFF loose | `ofdm_stream_processor.cpp:79` | Substitutes an ideal two-path LS channel estimate | Attribution oracle |
| `ULTRA_QAM16_GENIE_SIGMA_EMPIRICAL` | OFF loose | `ofdm_stream_processor.cpp:75` | Uses the empirical per-symbol noise σ for LLR scaling | Attribution oracle. Its subject is already bounded: LLR calibration is an exhausted lever (≤1.21×), so a large gap here means the gap is **not** in σ estimation |
| `ULTRA_QAM16_GENIE_CHANNEL_DELAY_SAMPLES` | 24 (= 0.5 ms = ITU Good), [0,512] | `channel_equalizer_pilot.cpp:77` | Two-path delay assumed by the oracle | **Set 48 on a Moderate capture** (1 ms) or the oracle silently assumes the wrong channel and its "truth" is wrong |
| `ULTRA_GUI_OPERATOR_LOG_SUPPRESS` / `_LOG_SLOW_MS` / `_EVENT_DRAIN_LIMIT` / `_EVENT_QUEUE_LIMIT` | OFF / 0 / 128 / `MAX_OPERATOR_EVENTS` | `app.cpp:413`, `:414`, `:416`, `:418` | GUI operator-log suppression, slow-write threshold, per-pass drain cap, queue depth (oldest dropped at capacity, `:1741`) | Cannot influence any modem decision or wire behaviour. These four use the **cleanest env parsing in the codebase** (`envFlagEnabled` accepts `0/false/FALSE/off/OFF`; `envUInt` has a `strtol` validity check and a 1e6 clamp) — standardise on them if the knob surface is ever normalised |

**Genie classification note:** the `*_GENIE_*` knobs are neither ACTIVE nor DEAD. They feed
the decoder information no receiver can have, so they can never ship — but misfiling them
as DEAD would delete the only apparatus that can partition a failure into estimation error
vs channel outage. File them as a distinct **EXPERIMENTAL/TOOL** category. Relevant
history: three attempts to build a perfect-CSI oracle failed, and an existing default-off
knob answered the same question in 20 minutes. **This family is that already-built
apparatus** — exhaust it before constructing a new oracle.

---

## Not in the infrastructure map

`docs/MODEM_INFRASTRUCTURE_MAP.md` is mandatory-live per CLAUDE.md, and a wrong or missing
entry causes a wrong deletion later. The list below was produced by grepping each knob name
against the map (`grep -c`, zero hits), not by substring diff.

**43 knobs from this audit still have a real getenv site and are absent.** In rough priority order:

**Critical — default-ON behaviour, or the default rate path:**

| Knob | file:line | Why it matters |
|---|---|---|
| `ULTRA_LATENT_RATE` | `latent_rate_controller.hpp:104` | **The default rate controller since 2026-08-01.** Zero map hits |
| `ULTRA_LATENT_RELAX_DB` | `connection.cpp:2627` | Its one free parameter |
| `ULTRA_BURST_ESCALATION` | `connection.cpp:5882` | Default-ON, +24 % rig, with a known Moderate failure mode |
| `ULTRA_ECHO_REANCHOR_GATE` | `streaming_ofdm_decode.cpp:2672` | Default-ON, +17 % rig |
| `ULTRA_ACK_MONITOR_GAPLESS` | `streaming_decoder.cpp:219` | Default-ON correctness fix |
| `ULTRA_ACKLISTEN_SUPPRESS_OFDM` | `streaming_sync_acquisition.cpp:377` | Default-ON; `KNOWN_BUGS.md:805` additionally claims default-OFF |
| `ULTRA_TNC_ACCUM_DISABLE` | `tnc_session.cpp:177` | Governs the Winlink/PAT data path, with inverted semantics |

**Rate / ARQ knobs (mostly deletion candidates, but the map is the deletion reference):**
`ULTRA_TRUST_LADDER_PICK` (`connection.cpp:3048`), `ULTRA_RUNG_DWELL_MS` (`:3113`),
`ULTRA_RUNG_CLASS_ANCHOR` (`waveform_selection.hpp:597`), `ULTRA_FER_FLOOR_ANCHOR`
(`:610`), `ULTRA_QAM16_CALM_FADING` (`connection.cpp:4010`), `ULTRA_ENTRY_QAM16_SNR`
(`waveform_selection.hpp:937`), `ULTRA_INFLIGHT_RTO`
(`selective_repeat_arq_policy.hpp:287`), `ULTRA_KEEPALIVE_ACK` /
`_MS` (`connection.cpp:5093` / `:5104`), `ULTRA_MC_ACK_REPEATS` (`connection.cpp:5447`),
`ULTRA_DROP_RX_SEQ`
(`connection.cpp:4542`).

**PHY / estimator:** `ULTRA_LTS_CFO_SEED_OFF` (`channel_equalizer_lts.cpp:469`),
`ULTRA_LTS_CFO_COH` (`:450`), `ULTRA_LTS_CFO_MIN` (`:484`), `ULTRA_LTS_DFT_DENOISE_TAPS`
(`ofdm_demodulator_setup.cpp:36`), `ULTRA_REL_FADE_MAX` (`channel_equalizer_equalize.cpp:112`),
`ULTRA_LDPC_MAX_ITERS` (`ldpc_codec.hpp:98`), `ULTRA_ACE_PAPR` (`modulator.cpp:285`),
`ULTRA_COHERENT_PAPR_DB` (`streaming_encoder.cpp:86`).

**Diagnostics and tools:** `ULTRA_NULL_DIAG`, `ULTRA_PSYM_DIAG`, `ULTRA_COH_DIAG`,
`ULTRA_RX_LAG_DIAG`, `ULTRA_GAMMA_DOMAIN_LOG`, `ULTRA_HARQ_DEBUG_SEQ`,
`ULTRA_DUMP_CFO_PREFIX`, `ULTRA_QAM16_GENIE_TIMING_CFO`, `ULTRA_CHANNEL_DELAY_MS`,
`ULTRA_CHEST_DELAY_SCAN`, `ULTRA_CHEST_NMSE_DUMP`, `ULTRA_MEASURE_BURST_FULL_ANCHOR`,
`ULTRA_MEASURE_BURST_NO_ANCHOR_WAIT`, `ULTRA_TNC_ACCUM_PROBE`, `ULTRA_TNC_FLUSH_LOG`,
`ULTRA_EXPERT_PHY`.

**Stale rows already in the map (fix, do not add):**

| Location | Says | Actually |
|---|---|---|
| ~~`MAP` + `KNOWN_BUGS` commanded-geometry rows~~ | ~~`ULTRA_COMMANDED_GEOMETRY` DEFAULT-ON~~ | **Corrected 2026-08-04:** broad arm OFF; truncation independent/default-active; exact learned same-rung Z81 recovery default-active for BI0 only, BI1 unknown-N fails closed |
| `MAP:233` | `ULTRA_MODE_CHANGE_RETRY_MS` default ~5 s ratiometric, "replaces the `getAckTimeout()` borrow" | The `getAckTimeout()` floor was **restored** after the W5/W5b/W6 bisect — ~18.5 s |
| `MAP:262` | `ULTRA_TNC_BULK_ACCUM` live | **Deleted**; superseded by `ULTRA_TNC_ACCUM_DISABLE` |
| `MAP:267` | `ULTRA_MAX_OFDM_RATE` at one old `connection.cpp` site | Four readers with different roles: the live canonical ceiling in `latentConfiguredRungCeiling()`, manual-accept and auto-accept bootstrap compatibility readers, and the default-bypassed legacy reader in `applyAdaptiveRateFeedback()` |
| ~~`MAP:247` + `streaming_decoder.hpp:930`~~ | ~~`ULTRA_RX_AGC` makes the CCA quiet gate gain-independent~~ | **Resolved 2026-11-11:** the knob and both claims were deleted (REMOVAL_BACKLOG R21) |
| `MAP:291` + `iterative_chest.hpp:49` | `ULTRA_ITERATIVE_CHEST` "MEASURED NULL" | Accurate **but imprecise** — null by the *family* control (`ULTRA_LTS_DFT_DENOISE`, n=240/cell), not by a direct A/B of this knob. Say which |
| `MAP:293` | `ULTRA_LTS_CFO_AVG` "FEAT (task #9, in-flight)" | Parked since 2026-05-30; closed by mechanism-level null controls |
| `MAP:93` | PAPR "skipped for all coherent mods → inert on data path" | Stale since `ULTRA_COHERENT_PAPR_DB` began self-enabling (`streaming_encoder.cpp:155-165`) |
| `MAP:412` + `CHANGELOG:3674` | `ULTRA_CHEAP_REANCHOR` = "1 chirp per 4 craters" | The streak resets on any decoded frame ⇒ "no chirp unless 4 **consecutive**" |

---

## Appendix — knobs deliberately NOT recommended, and why

So nobody re-proposes them from the name alone:

* **`ULTRA_INFLIGHT_RTO`** — the tail RTO defect was real, but the experimental table acted
  on logical/window state and missed the NACK/RTO retransmission paths. Physical egress now
  commits exact frame identities and pauses unsent holes by default. Keep the knob off and
  delete it separately; do not A/B a superseded correctness mechanism.
* **`ULTRA_ADAPTIVE_RTO` paired with `ULTRA_INFLIGHT_RTO`** — where one acts the other is
  inert, and where they interact the composite RTO can land **under** the burst it is
  timing.
* **`ULTRA_ENTRY_EVM_CAP`** — one-shot-ness concentrates rather than reduces risk now that
  the entry rung seeds the latent prior, and on the proposed MPM test channel the connect
  estimator **saturates**, making the cap a systematic over-cap by construction.
* **`ULTRA_CHEAP_REANCHOR` solo** — a strict no-op without `ULTRA_CRATER_REANCHOR_HOLD=1`,
  and its incremental effect after HOLD is ~1.3-1.7 % of transfer time, an order of
  magnitude below rig resolution.
* **`ULTRA_ITERATIVE_CHEST` / `ULTRA_LTS_CFO_AVG` / `ULTRA_LTS_DFT_DENOISE_TAPS`** — all on
  the channel-mean axis, which a 4× NMSE improvement measured at exactly zero FER across 8
  cells at n=240/cell.
* **`ULTRA_MEASURE_BURST_FULL_ANCHOR` as the harness default** — it reproduces the resend
  regime, not the shipped warm handoff; the harness's real defect is its descriptor-off
  default.

## `ULTRA_MCDPSK_ACK_REPAIR` (added 2026-11-12)
`1` on both stations: an MC-DPSK frame the peer's ACK reports missing is resent at the next burst
instead of at its retransmission timeout, and the receiver ACKs a turn whose last frames or first
frame it lost. Default off until a GUI A/B at low SNR (MC-DPSK sessions) shows it does not hurt;
see CHANGELOG 2026-11-12.

## `ULTRA_MCDPSK_WINDOW_MAX` (added 2026-11-12)
`N` (1-8, same on both stations; default 3): the most frames an MC-DPSK burst carries when the 19 s
key-down target allows more. Only DQPSK and the shorter-symbol geometries reach the cap (DQPSK at
1024 samples/symbol: 6 frames, a 17.0 s burst); DBPSK at 1024 stays at 3. The cap of 3 dates from
#71 (2026-07-01): DQPSK at window 5 timed out with clean decodes because the ACK arrived more than
26 s after the burst; that is BUG-MCDPSK-ACK-DROPPED (every MC-DPSK file ACK held 13 s and dropped,
fixed 2026-11-09). Today the ACK arrives 1.43 s after the burst end (p99 1.54 s, 187 GUI bursts).
First GUI A/B (2 KB, AWGN +3 dB, before the up-step probe cap): 210-211 s vs 208 s - a 6-frame DQPSK
burst is 13 % faster per frame, but the probe schedule then waited 4 clean DBPSK bursts instead of
2. An up-step now probes with the lower rung's window (`mcDpskProbeBurstFrames`); re-test pending.

## `ULTRA_MCDPSK_MEASURED_RTO` (added 2026-11-12)
`1` on the sending station: an MC-DPSK burst's retransmission timeout is its airtime plus the
post-burst ACK latency measured on earlier bursts (RFC 6298, back-off on silent rounds, the formula
as ceiling) instead of the formula's 43.6 s for a 3-frame DBPSK burst (measured round trip 18.4 s).
Saves ~23 s per lost tone ACK. GUI 2026-11-12: Good 0 dB 5 seeds 197 vs 228 s mean; follow matrix
9/9 PASS, but a one-frame DQPSK burst's ACK (2.8 s late, the receiver's first-frame look-ahead)
collided with the 3.0 s timeout - the guard now includes that look-ahead; re-run before any flip.
On a rig the peer's decode speed and CAT latency are measured, not assumed.
