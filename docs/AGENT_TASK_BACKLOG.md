# Agent Task Backlog

This backlog converts the broad project objective into bounded tasks that an
agent can execute overnight without guessing.

Main objective: maximize reliable HF modem throughput across AWGN, Good,
Moderate, and Poor fading while preserving production reliability, security,
and testability.

## Global Acceptance Rules

Every task must include:

- exact code paths changed,
- exact local gate command and result,
- exact benchmark or hardware command when PHY/ARQ behavior changes,
- before/after metrics when throughput or robustness is touched,
- residual risks.

Reject any task result that:

- lowers frame/file delivery reliability without a documented tradeoff,
- weakens LDPC, ARQ, sync, or hardware calibration invariants,
- removes tests without replacement coverage,
- adds hidden global state, sleeps, or timing assumptions,
- commits logs, prompts, credentials, or host-specific secrets.

## Lane A: Critical Test And Quality Infrastructure

### A1. Cross-Platform Temp/File Hygiene

Goal: remove hardcoded POSIX-only temp paths from maintained tests.

Scope:

- `tests/`
- maintained test helpers

Acceptance:

- Windows, Linux, and macOS CTest must pass.
- Use `std::filesystem::temp_directory_path()` or CTest-provided working dirs.
- No persistent files left in the repo or fixed `/tmp` names.

Gate:

```bash
ctest --test-dir build --output-on-failure -j4
```

### A2. Coverage Map For Critical Blocks

Goal: define coverage expectations by subsystem instead of chasing arbitrary
whole-repo 100%.

Scope:

- LDPC encode/decode
- OFDM sync/demod
- streaming decoder acquisition gates
- selective-repeat ARQ
- file transfer controller
- waveform policy

Acceptance:

- Produce a markdown coverage map with required tests per block.
- Identify untested critical behavior and stale tests.
- No code changes unless needed for testability.

Gate:

```bash
./scripts/coverage_report.sh
```

### A3. Property/Boundary Tests For ARQ

Goal: harden selective-repeat behavior under ACK loss, stale ACKs, holes, and
window wrap.

Scope:

- `src/protocol/selective_repeat_arq.cpp`
- existing ARQ tests

Acceptance:

- Add deterministic tests for stale ACK, cumulative ACK, SACK hole repair,
  timeout repair, duplicate data, and wraparound.
- No sleeps or wall-clock flakiness.

Gate:

```bash
ctest --test-dir build -R 'SelectiveRepeat|Protocol' --output-on-failure
```

## Lane B: Throughput Baseline And Reproducibility

### B1. Reproducible Channel Benchmark Matrix

Goal: make throughput claims comparable across agents and commits.

Scope:

- `tools/`
- `scripts/`
- docs

Acceptance:

- One script runs AWGN/Good/Moderate/Poor for selected SNRs and rates.
- Output includes file size, wall time, data-phase bps, retx, timeouts,
  frame success, AckR, and log directory.
- Script fails if delivery fails.

Gate:

```bash
./build/cli_simulator --snr 15 --fading good --rate r1_2 --test
```

### B2. Decode CPU Attribution Report

Goal: quantify where decode-thread time goes before optimizing.

Scope:

- instrumentation only, preferably compile/runtime gated
- `src/gui/modem/streaming_decoder.cpp`

Acceptance:

- Report timing for sync search, 1-CW control decode, 4-CW data decode,
  LDPC attempts, acquisition rejection, and backlog.
- No production overhead unless explicitly enabled.

Gate:

```bash
ctest --test-dir build --output-on-failure -j4
```

## Lane C: Throughput Improvements

### C1. ACK Rate Reduction Without Stale-Repair Storms

Goal: reduce control-frame load while avoiding the stale timer failure mode.

Scope:

- selective-repeat ARQ ACK scheduling
- ACK repeat policy
- tests

Acceptance:

- Demonstrate lower AckR on Good/Moderate without higher retx/timeouts.
- Must include rollback sentinel for out-of-window/stale repair storms.
- Must pass at least 1 KB and 5 KB injected Good/Moderate hardware smoke.

Gate:

```bash
SSH_KEY="$HOME/.ssh/id_pi5" AGENT_HW_LONG=1 ./agents/run_hardware_smoke.sh
```

### C2. Adaptive RTO By Measured Decode Backlog

Goal: stop spurious ARQ timeouts when decoder backlog temporarily grows.

Scope:

- ARQ timeout policy
- streaming decode telemetry

Acceptance:

- RTO remains bounded.
- Timeout decrease on moderate fading without masking real loss.
- Unit tests cover min/max clamp and backlog spike behavior.

Gate:

```bash
ctest --test-dir build -R 'SelectiveRepeat|Streaming' --output-on-failure
```

### C3. Control Decode Fast-Fail Policy

Goal: reduce wasted 1-CW LDPC attempts on false locks without rejecting real ACKs.

Scope:

- 1-CW control decode path
- LLR/RMS/correlation gates

Acceptance:

- False-lock decode attempts decrease in profiler.
- ACK frame success remains healthy on Good/Moderate SNR15 and SNR12 canary.
- No hardcoded thresholds without documented evidence.

Gate:

```bash
SSH_KEY="$HOME/.ssh/id_pi5" ./agents/run_hardware_smoke.sh
```

## Lane D: Fading Robustness

### D1. Poor-Channel Mode Policy

Goal: define when the modem should switch to lower rate or MC-DPSK on Poor fading.

Scope:

- link adaptation policy
- waveform policy tests

Acceptance:

- Poor-channel tests deliver files reliably, even at lower throughput.
- Rate decisions are documented by SNR/fading evidence.

Gate:

```bash
ctest --test-dir build -R 'Waveform|OFDMLinkAdaptation|Protocol' --output-on-failure
```

### D2. Burst Erasure Recovery Tests

Goal: ensure weak physical bursts become erasures/repairs, not file-transfer
deadlocks.

Scope:

- streaming decoder weak-block handling
- file transfer tests

Acceptance:

- Synthetic tests cover one weak block inside a 4-frame burst.
- ARQ repairs the missing data without stale ACK storms.

Gate:

```bash
ctest --test-dir build --output-on-failure -j4
```

### D3. ALC tolerance — model + decoder sweep

Goal: validate (and measure) whether the modem decodes through realistic
HF transmitter ALC compression. Today there is **no ALC model** in OTASim,
**no ALC tolerance test**, and **no documented evidence** the modem
works through real radio ALC. Operator feedback (KC3VPB 2026-05-08,
`feedback_kc3vpb_alc_handling`) was explicit: design for ALC, don't
dodge it. The current 0.7×/0.8× WAV pre-attenuation is the wrong fix.

Full spec: `docs/ALC_TOLERANCE_WORKSTREAM.md`.

Scope:

- `src/ota_channel_core/models.{cpp,hpp}` — new `AlcCompressor`
  class (peak detect, exponential attack/release in dB domain,
  configurable threshold / ratio / attack / release / overshoot).
- `src/ota_channel_core/session_context.cpp::advanceSessionClock`
  — wire ALC into the mixer chain BEFORE channel noise (ALC is in
  the transmitter; noise is in the channel; order matters).
- `proto/ota_simulator.proto` + `tools/cli_simulator.cpp::setOtaChannel`
  + `tools/otasim_ctl.cpp` — extend channel model enum and
  `SetChannel` plumbing for `alc` model.
- `tests/test_alc_compressor.cpp` — unit test for the compressor
  itself (steady-state gain, attack/release time, multi-tone IMD).
- `tests/test_decoder_alc_tolerance.cpp` — sweep (modulation, code
  rate, compression ratio, drive above threshold) and produce a
  decode-success-rate table per cell.

Acceptance:

- Compressor unit test passes with all measured parameters within
  ±20% of configured values.
- Decoder sweep produces a numerical baseline table for the
  production mode ladder (DQPSK R1/4 / R1/2 / R2/3 / R3/4, QPSK
  R1/2 / R2/3, D8PSK R2/3) at compression ratios {1:1, 2:1, 4:1,
  8:1, ∞:1} and drives {threshold, +6 dB, +12 dB}.
- DQPSK R1/2 (the production baseline for most QSOs) **must**
  decode at 4:1 / +6 dB with ≥95% success. If it doesn't, STOP
  and flag as a PHY issue — that's the real-radio deployment
  blocker.
- Baseline table committed as a regression gate for future ALC
  model or modulator changes.
- `docs/INVARIANTS.md` updated with "decoder must tolerate ALC at
  X ratio Y compression" once baseline is established.

Out of scope:

- PAPR reduction in the modulator (clip-and-filter, ACE) — separate
  workstream, becomes interesting *after* baseline numbers exist.
- Changing the default `tx_drive` value — per-radio operator choice.
- Adding ALC to the Mac↔Pi5 cable rig — would need a Pi-side
  software ALC injector; outside the OTASim contract.

Gate:

```bash
ctest --test-dir build -R "AlcCompressor|DecoderAlcTolerance" --output-on-failure -j1
```

Risks:

- Decoder may fail at modest ALC (≤4:1) on higher-order modulations
  (QPSK, D8PSK). If so, the conclusion is "production-safe modes
  are DQPSK only" — a real finding worth surfacing, not a test
  failure to suppress.
- Compressor parameters must match real-radio behavior; if model
  is too gentle, we miss real failures. Pin parameters to documented
  HF transceiver specs (FT-891, IC-7300, K3 — common rig classes).

## Lane E: Security And Agent Governance

### E1. Secret/Artifact Leak Gate

Goal: prevent agent prompts, reports, logs, keys, and host-specific artifacts from
being committed.

Scope:

- `.gitignore`
- agent scripts
- CI

Acceptance:

- Queue/archive/reports/tmp remain ignored by default.
- CI or local script detects common secret patterns in tracked files.
- GitHub Actions jobs use least-privilege token permissions.

Gate:

```bash
git status --ignored agents .claude
git diff --check
```

### E2. Agent PR Template And Review Checklist

Goal: make every agent PR auditable.

Scope:

- `agents/`
- `.github/`

Acceptance:

- PR body requires task ID, gates, benchmark evidence, risks, and rollback notes.
- Throughput PRs must include before/after metrics.
- Security-sensitive PRs must state whether permissions changed.

Gate:

```bash
ruby -e 'require "yaml"; YAML.load_file(".github/workflows/build-matrix.yml")'
```
