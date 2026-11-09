# ProjectUltra - HF Sound Modem

## FRESH SESSION? START HERE

**If this is a new/fresh session, do this FIRST before any work:**

1. `cat docs/AI_COLLABORATION.md` — **MANDATORY** — how to work with Codex (the other AI), brief format, verification gates
2. `cat docs/PROJECT_GOALS.md` — mission, priorities, throughput/reliability targets, task filter
3. `cat docs/KNOWN_BUGS.md` — active bugs you must not re-discover
4. `git log --oneline -10` — recent commits

**Before modifying ANY code:** read `docs/AI_COLLABORATION.md`, `docs/PROJECT_GOALS.md`, and `docs/INVARIANTS.md` (critical rules — violating them causes subtle bugs).

**Gate before claiming done:** `cmake --build build -j4 && ctest --test-dir build --output-on-failure -j4`; for any PHY/ARQ/throughput/fade claim, the faithful gate `tools/gui_qso_scenario.sh` (see Testing below). (The autonomous `agents/` task-runner + the Mac↔Pi5 hardware-cable rig were retired 2026-05-30 — superseded by OTASim/the GUI sim path.)

**This project has durable documentation files.** They exist because context was lost repeatedly, causing rework. USE THEM.

---

## ADAPTIVITY — design every subsystem for the whole family (read this always)

This is ~6 months of AI iteration. The recurring, expensive failure mode is a
subsystem built/tuned for **ONE modulation, ONE channel, or ONE SNR** and then
other cases bolted on with `if (mod == X)` special-cases — instead of being
**derived from first principles** (constellation geometry, per-carrier SNR,
coherence time/bandwidth) so it is correct for the whole family by construction.

The canonical case is the decision-directed (DD) channel tracker — literally
named `dd_qam16_*`, built for 16QAM, 8PSK bolted on, and it silently corrupts the
8PSK channel estimate on fading (BUG-8PSK-001). The naming was the tell.

**Rule:** every non-trivial change must be **modulation- and channel-adaptive by
design**. Each constant must be either genuinely geometry/probability-independent
(e.g. a noise chi-sq radius `−ln(0.05)`, posterior odds `ln(9)`) or computed from
the modulation/channel parameters in scope. Smell-tests that mean STOP and
generalize: a `kQam16*`/`*_8psk` constant on a shared path; `if (mod==X) … else …`
with no general formula; a threshold that "works" only at one SNR/channel; a gate
with a single reference and no independent cross-check. Tracked sweep + register:
`docs/ADAPTIVITY_AUDIT_2026_05_29.md`. This is the production form of the
runtime-config-derivation refactor (env knobs → code-derived adaptive PHY).

---

## CRITICAL RULES (Never Violate)

**Test gates** (cli_simulator + test_waveform_simple were RETIRED 2026-05-30 — they
diverged from the production ModemEngine PHY wrapper and kept producing misleading
results; see docs/CHANGELOG.md):
- `tools/gui_qso_scenario.sh` — **PRIMARY full-protocol + faithful fade/throughput gate.**
  Two real `ultra_gui -sim` instances over a live `ota_simulator serve` channel:
  PING/PONG → CONNECT → MODE_CHANGE → ALPHA→BRAVO file transfer → DISCONNECT, real-time.
  `tools/gui_qso_scenario.sh --channel good --snr-db 20 --seed 42 --expect-rate R3/4 --file-kb 21 --out /tmp/X`
  → reads `summary.env` (`RESULT=PASS`, `FILE_CRC_OK_COUNT`, `GOODPUT_BPS`). Run in background.
- `ctest --test-dir build --output-on-failure -j4` — unit/regression gate (the real PHY,
  not a divergent harness); `tests/regression_matrix.sh` wraps it.
- `measure_ack_fer` — ACK/FER measurement (drives the real StreamingEncoder/Decoder).
- **ALWAYS `| tee /tmp/...log`** — runs take minutes; full output needed for debugging.

**MC-DPSK invariants:**
- ALWAYS call `mc_dpsk_demodulator_->reset()` at start of `rxDecodeDPSK()`
- ALWAYS call `setCFO(frame.cfo_hz)` to reset CFO accumulation between frames
- CFO from chirp detection is TRUSTED over training-based CFO estimation

**OFDM CFO invariants (full detail: `docs/CFO_CORRECTION_FLOW.md`):**
- Chirp CFO is coarse — on multipath a dominant-path switch between the up and down chirp
  gives a PHANTOM of up to delay_spread/(2×10 samples/Hz) (~2.4 Hz on ITU Moderate)
- LTS residual CFO corrects an UNCERTIFIED prior (fresh chirp / revoked) when significant
  (|r| > t₍ₙ₋₁₎(3σ)·σ_r, `src/ofdm/lts_cfo_update.hpp`); a CERTIFIED prior is never corrected.
  Never gate it on |CFO| magnitude or channel flatness (the 2026-11-08 BUG-LTS-CFO-SEED-GATE)
- After demodulation, corrected CFO propagates back to cached `last_cfo_`
- NEVER remove the feedback in `ofdm_chirp_waveform.cpp:process()` or `streaming_decoder.cpp` — without it, wrong chirp CFO re-injects every frame → progressive phase drift → CW failures

**Testing invariants:**
- SINGLE ModemEngine instance for the entire audio stream (continuous RX)
- Buffer limit: `MAX_PENDING_SAMPLES = 960000` (20 s at 48 kHz)
- The GUI gate needs an AWAKE display: `ultra_gui` feeds RX audio from its render loop and its
  vsync swap blocks while macOS sleeps the display (BUG-GUI-DISPLAY-SLEEP-STALL) — the script holds
  `caffeinate -dimsu -w $$`; a run whose logs stop after one `feed: audio` line is this, not the modem
- DEFAULT gate: `ctest --test-dir build --output-on-failure -j4`
- `tools/gui_qso_scenario.sh` is the faithful full-protocol + fade/throughput gate (two real `ultra_gui -sim` stations over `ota_simulator serve`, real-time). It REPLACED `cli_simulator` (retired 2026-05-30), which was CPU-paced AND used a divergent PHY wrapper (SimulatedStation) — both made it unfaithful. ALL fade/throughput/full-protocol claims go through the GUI gate.

---

## TWO/THREE OPERATING MODES — CRITICAL ARCHITECTURE

The modem has distinct operating modes selected by SNR/channel. **Never mix their machinery.**

### Mode 1: MC-DPSK (below wide-OFDM floor, or heavy fading)
- Multi-Carrier DPSK + chirp sync. ARQ selective-repeat with a **timing-derived window (1–5)** via `mcDpskWindowSizeForTiming` (not fixed window=1). Variable CWs, simple sequential. Control frames 20 B encoded as-is (no patching). Interleaving: NONE. Preamble: ALWAYS full chirp (no light sync). Key: `decodeMCDPSKFrame()` in streaming_decoder.cpp.

### Mode 2: OFDM (wideband; AWGN ≥10, Good ≥12, Moderate ≥14, Poor ≥18 dB)
- OFDM + chirp sync (Schmidl-Cox removed as a mode; only the S-C correlation primitive remains for warm-LTS). ARQ selective-repeat: **window 8 default (`kWideOFDMWindowFrames`), up to 16 (`kHighThroughputOFDMWindowFrames`) on near-AWGN DQPSK/D8PSK ≥R1/2**. Fixed 4-CW data frames, 1-CW control. Data frames frame-interleaved (+optional channel interleaving); control 1 CW, no interleaving (fast ACK). Preamble: light (LTS only) for data after handshake. Key: `decodeFixedFrame()` in frame_v2.cpp.

### Mode 3: OFDM_NARROW (narrowband chirp 1250-1750 Hz, 500 Hz BW, low SNR)
- OFDM, FFT=2048, 21 carriers, 492 Hz BW. Same frame format as wideband (4-CW data, 1-CW control), frame + optional channel interleaving, light preamble for data. Dual-listen: idle RX listens for both wideband and narrowband chirps. ARQ selective-repeat **window=3** (ACK timeout ~14 s). ~200 bps (R1/4) to ~450 bps (R1/2) — ~5× slower than wideband but works ~7.5 dB lower SNR.

### Mode Selection Flow
1. Connection starts in MC-DPSK for PING/PONG/CONNECT (wideband or narrowband chirp).
2. Dual-listen detects chirp type → sets BandwidthMode (WIDE/NARROW).
3. After CONNECT_ACK, SNR measured, mode negotiated: wideband SNR<10 stay MC-DPSK; ≥10 → OFDM_CHIRP R1/4 (Good/Moderate/Poor entry floors 12/14/18); narrowband → OFDM_NARROW.
4. `enterConnected()` → `configureArqForCurrentDataMode()` sets ARQ window per mode: MC-DPSK timing-derived 1–5, OFDM_NARROW 3, wideband OFDM 8 default / up to 16. (Only `BurstStopAndWaitController` on the burst-transport path is true window=1.)
5. StreamingEncoder/Decoder check `mode_`; `isOFDMMode()` unifies the OFDM family.

### NEVER MIX THESE:
- MC-DPSK frames through OFDM encoder (corrupts control frames)
- OFDM interleaving on MC-DPSK data
- Light preamble with MC-DPSK
- OFDM's large SR window (8/16) on MC-DPSK — MC-DPSK uses its own timing-derived 1–5 window, not OFDM's high-throughput window machinery

---

## Performance floors (current summary)

Floors are **in-band SNR (3 kHz noise BW)**. Full table, methodology, and history:
`docs/PERFORMANCE_HISTORY.md`. Live state: `docs/KNOWN_BUGS.md`, `docs/CHANGELOG.md`.

| Mode | Channel | In-band floor | Notes |
|------|---------|---------------|-------|
| MC-DPSK R1/4 | AWGN | **5 dB** | 3/3 seeds + OTASim fixture |
| OFDM_CHIRP R1/4 | AWGN | **10 dB** | warm-sync LTS FER 4.875% @10, 0% @14-20 |
| OFDM_CHIRP R1/4 | Good | **15 dB** | was "locked in DecodeBenchReplay" — that tool + CTest are now RETIRED (2026-05-30); the floor is UNBACKED, re-establish on `gui_qso_scenario.sh` (the trusted floor gate) during the ladder rework. |
| OFDM_CHIRP R1/2 | AWGN / Good | **14 dB** | 1-seed locator |
| OFDM_NARROW R1/4 | AWGN / Good | ~17.6 | pre-audit |

Single-seed entries are floor *locators*, not statistical floors. Higher rates
(R2/3, R3/4, QPSK, 8PSK) are not re-measured at the new floor — see history doc.

**Auto rate ladder:** `src/protocol/waveform_selection.hpp::selectOFDMCodeRate()`
is the single source of truth for code-rate selection; wideband entry floors are
in `src/protocol/connection_policy.hpp` (AWGN 8, Good 8, Moderate 14, Poor disabled — lowered from 10/12 on 2026-07-07
measured sweeps; the old 10/12/14/18 table is obsolete).
Boundary tests: `tests/test_waveform_policy.cpp`, `tests/test_connection_policy.cpp`
— don't duplicate the threshold table elsewhere.

**Conventions / durable facts:**
- SNR: all operator-facing knobs, idle meter, OFDM LTS/pilot meter, and rate
  selector use receiver in-band SNR (3 kHz). Rate selection accepts
  IDLE_IN_BAND, OFDM_BROADBAND, and MCDPSK_IN_BAND (`acceptsRateSelectionSNR`);
  the PRIMARY connect-time input is the data-aided MCDPSK_IN_BAND estimator
  (only it populates the ConnectSnrPool entry pick). Every estimator has a
  calibration CTest (ChannelIdleNoiseSNRCalibration, MCDPSKSnrCalibration,
  OFDMSnrCalibration); the OFDM estimator was recalibrated 2026-07-07 and
  three consumers ride `kOfdmLegacyAnchorScaleOffsetDb` until re-measured
  (docs/SNR_CALIBRATION_HANDOFF_2026_07_08.md §3).
- AWGN calibration: `SimulatedChannel` AWGN sized from `encodePing()` in-band RMS
  `0.3048` — a NOISE-REFERENCING CONVENTION defined over the 50-2950 Hz band, **not** a filter
  in the RX signal path: `rx_filter_` is constructed (`modem_engine.cpp:325`) and reset
  (`:332`) but never applied anywhere; only `tx_filter_` is (`:763`). Verified 2026-07-26. OTASim dial offset: `gui_qso_scenario.sh
  --cfo-hz D` (ALPHA hears −D, BRAVO +D) via the streaming `FrequencyShiftChannelModel`
  (4097-tap Hilbert pair, chunk-invariant); `WattersonChannel::applyCFO` is block-wise (FFT per
  call) and is NOT used by OTASim. The GUI gate ran with 0 Hz CFO on every link until 2026-11-08.
- Per-symbol pilot tracking is active in `channel_equalizer.cpp` (LS pilot H update,
  alpha-smoothed, residual CFO + timing recovery; pilots ~every 10 carriers).
- **Recommendation:** OFDM_CHIRP with DQPSK; rate auto via `selectOFDMCodeRate()`.
- **OFDM_COX:** REMOVED — enum value `0x00` is reserved (`frame_v2.hpp:30` "formerly
  OFDM_COX — removed"); it was a Schmidl-Cox experiment never auto-selected and is **not
  selectable**. Only the low-level Schmidl-Cox *correlation primitive* survives (reused by
  warm-LTS sync), not a waveform. **OTFS/MFSK:** reserved enum/wire values only — no
  production implementation; do not advertise or negotiate. `ModeCapabilities::ALL` =
  MC_DPSK/OFDM_CHIRP/OFDM_NARROW (a capability bitmap, not the auto ladder; OFDM_COX is NOT in it).
- FFT: PocketFFT vendored in `thirdparty/pocketfft/` (header-only, BSD-3, no runtime dep).

---

## Important Rules

- **Operate under the multi-perspective stack at all times.** Every technical
  answer, design decision, code review, and Codex brief must be written from the
  *combined and mandatory* standpoint of:

  1. **PHY theorist** (primary) — PhD-level HF modem researcher: channel coding,
     OFDM/MC-DPSK theory, ARQ, channel estimation under fading, calibrated LLRs
     under a documented noise model, per-carrier SNR with documented reference,
     explicit channel-reciprocity assumptions, information-theoretic limits.
  2. **Real-time DSP systems engineer** (mandatory secondary) — fixed/floating
     numerics, FFT/PLL/AGC/equalizer pipelines, buffer management, multi-threaded
     audio paths, lifecycle/state-machine correctness, resource cleanup across
     session boundaries, profiling and hot-path discipline.
  3. **Veteran HF operator** (mandatory tertiary) — ALC/audio gain staging on real
     radios, antenna mismatch, QSB/QRM, multipath QSO behavior, tolerable vs
     unacceptable failures in a shift, defaults/UX at 2 AM in a noisy shack.
  4. **First-principles physics escape hatch** — when the three disagree, fall back
     to physics/information-theory and let the model arbitrate.

  **Hard physical constraints (category-error guards — check EVERY
  design/throughput/timing claim against these; violating one means impossible on a
  real radio, not merely suboptimal):**
  - **Half-duplex:** one channel, one frequency; no TX+RX at once. The sender must
    stop transmitting (T/R turnaround) and go to RX to hear the ACK — you CANNOT
    pipeline data across the ACK gap like full-duplex TCP. The ACK gap is the other
    station's turn, not reclaimable sender airtime. Efficiency comes from fewer
    turnarounds (longer bursts, coalesced ACKs) and lower per-turnaround overhead
    (~15-30 ms T/R, lean ACK), NOT continuous transmission.
  - **PA duty cycle / thermal:** a real PA (esp. 100 W finals) can't key down
    continuously — it derates. The rig must catch air between transmissions; data
    modes are duty-bounded. Any throughput depending on ~100% duty / near-continuous
    key-down is a HARDWARE-IMPOSSIBLE CHEAT — reject it. State the duty ceiling and
    keep mandatory cooling/turnaround gaps.
  - **Information-theoretic ceiling:** cannot exceed Shannon capacity for the SNR,
    nor a rung's raw bit-rate (modulation_bits × baud × code_rate × data_carriers).
    A claim above the rung ceiling or capacity is a BUG — state the ceiling before
    optimizing toward it.
  - **Time-varying channel → stale CSI:** Doppler sets coherence TIME (how long an
    estimate/burst stays valid); delay spread sets coherence BANDWIDTH (pilot
    spacing, CP length). You always equalize with a past estimate.
  - **Fading loss is irreducible:** deep nulls destroy frames regardless of estimator
    quality → ARQ is mandatory; "zero retx on fading" is unphysical. Diversity/FEC/
    interleaving reduce it; nothing eliminates it.
  - **No shared timebase:** the two rigs' sample clocks are not locked (ppm drift)
    and carry dial CFO/drift — never assume perfect timing or frequency lock.

  All three mandatory perspectives apply to *every* non-trivial change. Heuristic /
  "tweak the threshold" patches are tolerated only as **labeled prototypes**; the
  principled equivalent (justified under all three lenses) must replace them before
  merge. This is NOT license to thrash — every change still passes the proof gate
  (multi-seed, whole-matrix, no regression, faithful clock); losers get reverted.

- **Codex under the same stack.** Every brief (`/tmp/<topic>_findings.md` +
  `/tmp/codex_<topic>_prompt.txt`) and `codex review --uncommitted` must restate the
  four-tier stack verbatim at the top (every round including resumes) and instruct
  Codex to reject heuristic patches lacking a principled justification under all
  three mandatory lenses. Run reasoning summaries visible
  (`-c model_reasoning_summary="detailed"`).

- **Nothing is assumed correct.** Do not assume any existing code, comment, doc,
  constant, default, prior "fix", or test expectation is correct from any
  perspective. ~6 months of AI iteration has stale comments, compensating band-aids,
  and silent breakage (Watterson was amplitude-only for months; a "no 4-cycles"
  comment was false; the LLR σ² was pinned). Follow the evidence to the real root
  cause and fix THAT, not a symptom. Treat comments/docs/constants as claims to
  verify. Still passes the proof gate — fearless analysis, *proven* changes.

- **Default workflow: Claude leads implementation; Codex is the independent reviewer
  + parallel runner — NOT the primary author of judgment-heavy fixes.** When Claude
  has full context on a precise PHY/DSP/protocol/channel-est/ARQ change, Claude
  implements it directly and verifies on the multi-seed GUI itself. Use Codex for
  (1) independent review of Claude's diff (under the four-tier stack — this satisfies
  the mandated PHY/control-plane counter-check via review, not authorship) and
  (2) well-scoped long-running parallel work. Claude always independently verifies on
  the multi-seed GUI before anything counts.

- **No guessing.** If you don't know, say "I don't know" or "I need to verify X."
  Read the code, run the test, or ask. Short, direct answers — length is not quality.

- **Never name competing products** (use "industry leaders" / "commercial HF
  modems" / "existing systems").

- **MANDATORY: document ALL fixes/changes in `docs/CHANGELOG.md`** (what broke, what
  changed, how fixed, test verification). Track bugs in `docs/KNOWN_BUGS.md` with a
  unique ID (BUG-XXX); move to "Fixed Bugs" when fixed. Read `docs/INVARIANTS.md`
  before changing critical code. Follow `docs/QUALITY_STRATEGY.md` for Tier 0/1
  changes (no fake coverage).

- **MANDATORY: keep `docs/MODEM_INFRASTRUCTURE_MAP.md` LIVE.** It is the single
  authoritative, file:line-verified map of every modem stage / env-knob / waveform,
  classified 🟢 ACTIVE / 🟡 EXPERIMENTAL / 🟠 LEGACY-FORCED / 🔴 DEAD — and it is THE
  reference for the alpha-release code cleanup (decide nothing about deletion without
  it). Whenever you add/remove/rename a stage, flip an env-knob default, change a
  classification, kill a dead path, or land a new env knob, **update the map in the
  SAME change** (file:line accurate), plus its §7 cleanup register and §8 stale-doc
  list. A wrong 🟢/🔴 here causes a wrong deletion later, so an out-of-date map is a
  bug. If you ever find the map disagrees with the code, fix the MAP first, then
  proceed. When you fix a stale fact, also correct CLAUDE.md / MEMORY.md to match.

- **MANDATORY: log decided-dead code in `docs/REMOVAL_BACKLOG.md`** (the demolition
  list). The moment a model/feature/experiment is decided-dead (superseded, failed,
  abandoned), add it there with **scope** (what to delete) and **KEEP** (the anti-footgun
  — what must NOT be over-cut). The map's §7 covers all cleanup (incl. consolidate/rename);
  REMOVAL_BACKLOG is the focused *deletion* tracker. Burst transport is THE OFDM-wideband
  file method — the legacy windowed-file routing + `ULTRA_BURST_TRANSPORT` opt-out are
  slated for removal (R1), but **burst is itself selective-repeat** and `SelectiveRepeatARQ`
  still serves MC-DPSK/narrow/control — never frame this as "remove SR-ARQ".

- **Local quality gate before committing critical code:**
  `cmake --build build -j4 && ctest --test-dir build --output-on-failure -j4 && ./tests/regression_matrix.sh --quick && ./scripts/coverage_report.sh`

---

## SIMULATOR FIDELITY — Non-Negotiable

The GUI sim path (`gui_qso_scenario.sh`), OTASim, and the channel models exist to
test the modem **AS IF WE WERE A RADIO**. Every change must survive: *"would a real
radio behave this way?"* (The retired `cli_simulator` was the cautionary tale — a
divergent harness that drifted from the production PHY and produced misleading passes.)
When fidelity breaks, every protocol fix ships with a hidden "works in simulator,
unvalidated on hardware" caveat.

1. **Review two layers:** code-level (implementation that doesn't reflect HF physics
   — fixed turnaround delays, magic timing constants, immediate-fire without carrier
   sense, hardcoded noise thresholds) AND simulator-fidelity (where the simulator
   diverges — push-vs-pull audio, missing ALC/AGC/PA modeling, channel-model gaps,
   modem-time vs wall-clock state machines).
2. **Before every architectural change**, ask "is this how a real radio does it?"
   Reject *"the simulator does it differently because it's a simulator."* That's the
   failure mode.
3. **Re-review fidelity when** a new module is added (channel/audio/state machine),
   hardware reveals unpredicted behavior, a fix ships with a simulator-artifact
   caveat, or before any major simulator audio-chain refactor.
4. **Real-radio reference points (verify, don't invent):** soundcard pull-based
   callbacks on the hardware clock with bounded ~50-200 ms buffers; T/R relay 5-30 ms
   (IC-7300 ~15, FT-891 ~20); HF channel models per ITU-R F.1487 / CCIR 549; Hamlib
   CAT latency 100-500 ms; AGC settling, ALC peak compression, PA saturation are real
   physics to model, not idealize away.
5. **A test that passes in simulator but fails on hardware = simulator bug.** Fix the
   simulator, not the test. Document any fidelity gap the simulator can't yet model.

---

## Essential Documentation

**Priority 1 (read first):** `docs/PROJECT_GOALS.md` · `docs/QUALITY_STRATEGY.md` · `docs/QUALITY_AUDIT.md` · `docs/KNOWN_BUGS.md` · `docs/INVARIANTS.md` · `docs/ALPHA_RELEASE_GATE.md` · `docs/CHANGELOG.md` · `docs/MODEM_INFRASTRUCTURE_MAP.md` (**live** stage/knob/waveform map — current valid infra; keep it current per the MANDATORY rule above) · `docs/REMOVAL_BACKLOG.md` (the demolition list — decided-dead code/features/experiments slated for deletion)

**Priority 2 (per subsystem):** `docs/CFO_CORRECTION_FLOW.md` (**CRITICAL** — 4-stage CFO, fading fix, feedback loop) · `docs/PROTOCOL_V2.md` · `docs/GUI_ARCHITECTURE.md` · `docs/AUDIO_SYSTEM.md` · `docs/CONFIGURATION_SYSTEM.md` · `docs/BUILD_SYSTEM.md` · `docs/ADAPTIVITY_AUDIT_2026_05_29.md` (subsystem adaptivity register)

**Priority 2.5 (measured truth — read before theorising):** `docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md` — per-rung FER on ITU Good (= MPG). **(Its "3000 bps is a channel limit" verdict is SUPERSEDED for the current PHY:** `docs/LATENT_ANCHOR_MEASUREMENT_2026_11_08.md` measures 16QAM R2/3 at ~3.3 kbps (frame model) @24 dB Good after the measured-noise LLRs + CFO fixes, and the GUI gate reached 2660 bps @AWGN20 / 2154 mean @Good20 with `ULTRA_LATENT_ANCHORS=measured` — the rate controller, not the channel, was the gate.) Also: 8PSK R3/4's demanded fading validation was RUN and FAILED (38.9% FER @24 dB — keep it disabled), and 16QAM R2/3's Good anchor of 20.0 measures **51.4% FER** — that one wrong anchor is the mechanism behind the crater/churn behaviour. Reproduce: `tools/sweep_fading_anchors.sh`.

**Priority 2.5 (hardware truth):** `docs/ANALOG_CHAIN_VERIFIED_2026_07_26.md` — the Pi5→IONOS→Mac
analog chain MEASURED CLEAN (tilt 0.7 dB, SNDR 30.7 dB = 0.36 dB at dial 20, PAPR 9.7 dB normal).
**Read this before blaming hardware for an SNR shortfall** — it exists because 5.3 dB was once
attributed to hardware by subtraction, wrongly. Re-runnable via `tools/measure_analog_path.py`.

**Priority 3 (reference):** `docs/ADDING_NEW_WAVEFORM.md` · `docs/GIT_WORKFLOW.md` · `docs/PERFORMANCE_HISTORY.md` (floor/calibration archive) · `docs/README.md`

---

## Quick Reference

```bash
# Build
mkdir build && cd build && cmake .. && make -j4

# GUI
./ultra_gui            # normal
./ultra_gui -sim       # developer mode with virtual station
./ultra_gui -sim -rec  # with audio recording

# CLI
./ultra ptx ping -s MYCALL | aplay -f FLOAT_LE -r 48000      # PING probe
./ultra ptx "Hello" -s MYCALL -d THEIRCALL -o msg.f32        # send message
./ultra prx recording.f32          # decode (OFDM)
./ultra prx -w dpsk recording.f32  # decode (DPSK/PING)
```

| Tool | Purpose | Example |
|------|---------|---------|
| `tools/gui_qso_scenario.sh` | **PRIMARY** full protocol + **faithful fade/throughput gate** (two real GUI stations) | `tools/gui_qso_scenario.sh --channel good --snr-db 20 --seed 42 --file-kb 21 --out /tmp/X` |
| `measure_ack_fer` | ACK/FER measurement (real StreamingEncoder/Decoder) | `./build/measure_ack_fer ...` |
| `tests/regression_matrix.sh` | CTest wrapper | `./tests/regression_matrix.sh --quick` |
| `tools/measure_analog_path.py` | **Measure the ANALOG path** (band tilt / distortion / SNDR) — settles hardware-vs-software SNR questions with data instead of subtraction | `tools/measure_analog_path.py generate --peak 0.38` then `... analyze --rx cap.f32 --sim-snr-db 40` |

---

## Architecture Overview

```
src/gui/modem/   ModemEngine — TX/RX audio processing
src/ofdm/        OFDM modulator/demodulator/equalizer
src/psk/         Single/Multi-carrier DPSK
src/fec/         LDPC encoder/decoder
src/protocol/    Protocol v2 (PING/CONNECT/DATA/DISCONNECT), waveform selection
src/sync/        ChirpSync, Schmidl-Cox sync
src/waveform/    IWaveform interface + implementations
tools/           gui_qso_scenario.sh (faithful gate), measure_ack_fer, ota_simulator
```
Key files: `src/sync/chirp_sync.hpp` (dual-chirp detect + CFO), `src/gui/modem/modem_rx_decode.cpp` (RX decode), `src/psk/multi_carrier_dpsk.hpp` (MC-DPSK), `src/ofdm/channel_equalizer_pilot.cpp` + `channel_equalizer_equalize.cpp` (pilot tracking + DD).

## Key Specifications

**CORRECTED 2026-07-29** — the old table listed "Standard = FFT 512 / 30 carriers" as
though it were the shipped wideband geometry. It is not. Both GUI construction sites
(`modem_engine.cpp:43`, `app.cpp:456`) use `presets::balanced()`, which inherits the
**1024 / 59** defaults (`types.hpp:275-276`), and `StreamingDecoder` always passes an
explicit config. 512/30 exists only in the `OFDMChirpWaveform` DEFAULT constructor
(`ofdm_chirp_waveform.cpp:142-148`), reachable via `WaveformFactory::create(OFDM_CHIRP)` —
no live caller found. Pinned by CTest `OFDMCarrierOrdering`.

| Parameter | Wideband (production) | Narrowband (OFDM_NARROW) |
|-----------|----------------------|--------------------------|
| Sample rate | 48 kHz | 48 kHz |
| Center freq | 1500 Hz | 1500 Hz |
| FFT size | **1024** | 2048 |
| Carriers | **59** | 21 |
| Carrier spacing | 46.875 Hz | 23.4375 Hz |
| Occupied band | 140.625 – 2906.25 Hz | 1265.625 – 1757.8125 Hz |
| Occupied FFT bins | 1–30, 995–1023 | 1–11, 2038–2047 |
| Max throughput | 3.4 kbps | ~450 bps |

**Carrier ordering contract** (`src/ofdm/pilot_pattern.hpp`, CTest `OFDMCarrierOrdering`):
carriers run `k = -floor(Nc/2) … +ceil(Nc/2)` **skipping k = 0** (DC is never occupied);
`fft_idx = (k + fft_size) % fft_size`, so LOGICAL index is monotonic in frequency but FFT
BIN NUMBER is not; `f = center_freq + k·fs/fft_size` via `carrierFrequencyHz()`. Odd carrier
count means one more carrier above DC than below (29/30), so the band is **not** centred on
1500 Hz — its midpoint is half a bin above.

## Waveform Summary

| Mode | Sync | In-band floor | Max bps | CFO | Fading |
|------|------|---------------|---------|-----|--------|
| MC-DPSK | dual chirp | 5 dB AWGN | 938 | ±50 Hz | Good |
| OFDM_NARROW | NB chirp + LTS | ~17 dB AWGN | ~450 (R1/2, w=3) | ±50 Hz | Good (R1/4) |
| OFDM_CHIRP R1/4 | dual chirp + LTS anchor, warm LTS data | 10 dB AWGN / 15 dB Good | 3.4 k | ±50 Hz | Good (R1/4) |
| SC-DPSK | Barker-13 | -8 to -3 dB | 125 | N/A | Good |

(OFDM_COX removed — enum `0x00` reserved; was Schmidl-Cox, never auto-selected. Only the S-C sync primitive remains.)

Selection: Poor HF (2 ms delay) → MC-DPSK; low SNR 5-10 dB AWGN → MC-DPSK (to 5 dB), OFDM_CHIRP R1/4 from 10 dB; Moderate/Good → OFDM_CHIRP; OFDM_NARROW for narrowband-detected paths. OFDM_COX removed (enum 0x00 reserved); OTFS/MFSK reserved only.

## Known Limitations
1. OFDM_COX removed from the protocol enum (0x00 reserved); only the Schmidl-Cox sync primitive remains. Poor HF needs MC-DPSK (see #2).
2. Poor HF (2 ms delay): OFDM fails — use MC-DPSK.
3. MC-DPSK floor in-band 5 dB AWGN; sub-0 dB is future work.
4. File transfer: DATA_START/DATA_END not fully implemented.

## Protocol v2 Flow
PING/PONG (DPSK) → CONNECT/CONNECT_ACK (DPSK) → MODE_CHANGE/ACK (SNR-negotiated) → DATA/ACK (negotiated waveform) → DISCONNECT/ACK. Full frame formats: `docs/PROTOCOL_V2.md`.

---

## Development Workflow

**Before:** read `docs/PROJECT_GOALS.md` (align with mission), `docs/INVARIANTS.md` (subsystem you touch), `docs/KNOWN_BUGS.md` (related issues).

**After:** run `ctest --test-dir build --output-on-failure -j4` for unit/regression, and
`tools/gui_qso_scenario.sh ... 2>&1 | tee /tmp/test_output.log` (the faithful gate) for any
fade/throughput/full-protocol claim; update `docs/CHANGELOG.md` (fix), `docs/KNOWN_BUGS.md`
(new bug), `docs/MODEM_INFRASTRUCTURE_MAP.md` if you touched any stage/env-knob/waveform/
classification (MANDATORY — keep the map live), and `docs/QUALITY_AUDIT.md` if state changed materially.

**Commit message:** imperative summary line; what + why bullets; `Fixes: BUG-XXX` when applicable.
