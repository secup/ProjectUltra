# ALC tolerance — workstream spec

**Status:** specified, not started
**Owner:** ProjectUltra core
**Origin:** KC3VPB OTA feedback 2026-05-08 + audit conversation 2026-05-18
**Memory link:** `feedback_kc3vpb_alc_handling.md`

## Why this exists

ProjectUltra targets real HF radios. Every SSB transmitter in the
deployment chain has an Automatic Level Control (ALC) loop that
compresses audio peaks to protect the final amp. The modem's TX
audio passes through ALC **on the way out of the operator's radio**
before it ever reaches the antenna, the propagation path, or a
remote receiver.

Today the modem has **zero validation** that it tolerates ALC
compression:

- OTASim channel models are AWGN + Watterson only. Perfectly linear.
- Mac↔Pi5 cable rig has no radio in the loop.
- No CTest gate, no invariant, no fixture exercises compression.
- The only ALC exposure was the KC3VPB 2026-05-07 OTA session, where
  decode was poor and the response was to **pre-attenuate the WAV**
  (the wrong fix, per KC3VPB's pushback).

Until this workstream lands, "does the modem work on a real radio"
is a guess.

## Scope

This workstream is **two pieces**:

1. **An ALC compression channel model** in OTASim, configurable
   per session, exposed via `SetChannel` / `InjectEffect`.
2. **A decoder tolerance test sweep** covering the production
   mode ladder (DQPSK R1/4 → DQPSK R1/2 → DQPSK R2/3 → DQPSK R3/4
   → QPSK R1/2 → QPSK R2/3 → D8PSK R2/3) at realistic ALC
   parameters.

**Out of scope** (do not pull these in):

- Adding PAPR reduction (clip-and-filter, ACE) in the modulator.
  That's a separate workstream that becomes interesting *after*
  we measure ALC tolerance.
- Changing the default `tx_drive` value. Operator-tunable per radio.
- Adding ALC to the cable rig (Mac↔Pi5). Outside the OTASim
  contract; would need a Pi-side software ALC injector.

## Background: how SSB ALC actually behaves

**Detector**: peak detector with fast attack. Some radios mix peak
+ short-window RMS to be less twitchy on syllabic speech, but for
continuous-power data modes the peak detector dominates.

**Threshold**: set by the manufacturer to clamp at roughly the
"safe drive" point (the radio's nominal full output). User's "mic
gain" knob moves the input level, not the threshold.

**Ratio**: 4:1 to 10:1 for normal compression range; effectively
∞:1 (limiter) above ALC range — radio refuses to output more
power regardless of input level.

**Attack**: 1-10 ms typical. Has to catch a syllabic peak that
lasts ~50-200 ms. Fast attack prevents over-power on the first
peak of a new syllable.

**Release**: 50-500 ms typical. Slow release keeps gain reduced
through the rest of a syllable.

**Behavior on data modes** (continuous-power, no syllabic structure):

- ALC engages on first sample above threshold.
- Stays engaged because release is much slower than the modem's
  symbol period.
- After ~1-2 ms attack settles, the output is **steady-state
  compressed** — a near-constant gain reduction across the entire
  TX burst.
- The transient at TX onset (first ~1-2 ms) is uncompressed,
  producing a brief "punch" — sometimes called the "ALC overshoot"
  — which can saturate the final stage on hot drive.
- Multi-tone signals (OFDM) suffer **intermodulation distortion**:
  ALC's nonlinear gain creates IM products at carrier sum/difference
  frequencies. Splatter.

## Implementation: ALC channel-model node

**Location**: `src/ota_channel_core/models.cpp` + `models.hpp`.

**Class shape**:

```cpp
struct AlcConfig {
    float threshold_dbfs = -1.0f;   // hard threshold in dBFS (default -1 dBFS)
    float ratio = 4.0f;             // compression ratio above threshold
    float attack_ms = 5.0f;         // ms to reach 90% gain reduction
    float release_ms = 150.0f;      // ms to release after below threshold
    float makeup_gain_db = 0.0f;    // post-compressor gain (default 0)
    bool include_overshoot = true;  // model the ~1-2 ms transient
};

class AlcCompressor {
public:
    explicit AlcCompressor(AlcConfig config, float sample_rate);
    void process(std::span<float> samples);  // in-place, mono
    void reset();                            // for new TX burst
};
```

Implementation notes:

- Peak detector with exponential attack/release in dB domain.
- Gain reduction = `(input_db - threshold_db) * (1 - 1/ratio)`,
  clamped to non-negative.
- Overshoot model: don't apply gain reduction for the first
  `attack_ms` of a new TX burst (detected by `reset()` call on
  TX start).
- Run before AWGN noise in the OTASim mixer chain — ALC is in
  the *transmitter*, noise is in the *channel*. Order matters.

**Wire format**: add an `alc` model to `ChannelType` / `parseChannelType`.
Configurable via `InjectEffect` for mid-session changes, and via
`SetChannel` with a JSON params field for session-wide setting.

Estimated: ~150-200 LOC implementation + 50 LOC test for the
compressor itself.

## Tests

### Unit test (mechanical correctness)

`tests/test_alc_compressor.cpp` — drive a known sine + noise into
the compressor, verify:

- Steady-state output peak at threshold (±0.5 dB).
- Steady-state gain reduction matches `(input_db - threshold) * (1 - 1/ratio)`.
- Attack time within ±20% of configured value.
- Release time within ±20% of configured value.
- Overshoot present when configured, absent when disabled.
- Multi-tone IMD products appear at expected frequencies.

### Integration test (decoder tolerance)

`tests/test_decoder_alc_tolerance.cpp` (or extend
`test_otasim_serve_smoke` with an ALC scenario).

For each (modulation, code rate) pair in the production ladder,
run a CONNECT + 1 KB DATA + DISCONNECT scenario through OTASim
with ALC enabled at:

| Compression ratio | Drive above threshold | Expected outcome (initial)   |
|------------------:|:---------------------:|:-----------------------------|
| 1:1 (no ALC)      | n/a                   | baseline (must match)        |
| 2:1               | +6 dB                 | decode (~no degradation)     |
| 4:1               | +6 dB                 | decode (mild degradation)    |
| 4:1               | +12 dB                | likely decode for DQPSK; QPSK marginal |
| 8:1               | +12 dB                | DQPSK ok, higher mods fail   |
| ∞:1 (limiter)     | +12 dB                | DQPSK probably ok            |
| ∞:1 (limiter)     | +20 dB                | broad failure expected       |

Pass criterion: produce a table per (mode × rate × compression)
of decode success rate. This is a **measurement**, not a binary.
First run sets the baseline; subsequent runs gate against regression.

### Hardware OTA (out of CTest, in `agents/run_hardware_smoke.sh`)

Re-run KC3VPB-style real-radio test with the default `tx_drive`
deliberately set hot enough to engage ALC, decode rate measured.
Compare to OTASim ALC-on prediction.

## Milestones

1. **M1 — Compressor core** (~1 day): `AlcCompressor` class + unit
   test. Builds, tests pass.
2. **M2 — Wire into OTASim** (~half day): channel-model node,
   `SetChannel` plumbing, integration with mixer order
   (ALC → channel noise).
3. **M3 — Decoder sweep** (~1 day): integration test, run the
   table above, commit the baseline numbers as a regression gate.
4. **M4 — Document** (~half day): CHANGELOG entry, update
   `docs/INVARIANTS.md` with "decoder must tolerate ALC at X
   ratio Y compression" once baseline is established.
5. **M5 — Hardware confirm** (deferred, requires Mac↔real-radio):
   re-run KC3VPB scenario, document.

Total: ~3 dev-days to M4. M5 needs operator availability.

## Anti-stall rules

- **No PAPR-reduction work in this round.** That's tempting once
  we see decoder fails — but the right sequence is *measure first,
  fix second*. PAPR reduction in the modulator becomes its own
  workstream after we have baseline numbers.
- **No default-value tweaks** (`tx_drive`, `output_gain`, etc.) in
  this round. The fix is decoder tolerance, not avoidance.
- **If decoder fails at 4:1 / +6 dB on DQPSK R1/2** (the production
  baseline for most QSOs): STOP and flag as a real PHY issue, not
  a test-tuning issue.

## Three-perspective check

**PHY theorist**: ALC introduces *amplitude nonlinearity* into a
DPSK system. DPSK is amplitude-insensitive at the symbol-decision
level (only phase matters), but pilot-based channel estimation
in OFDM assumes roughly stable amplitudes. Mild compression
(2:1, ≤3 dB) should be invisible to the demodulator. Heavy
compression (∞:1, >6 dB) creates IMD products at carrier
sum/difference frequencies — for OFDM with 50+ carriers, this
spreads across the band. Real impact depends on whether the
demod's per-carrier SNR estimate sees the IMD as noise (probably
yes) or interprets it as signal (probably no, since pilots
calibrate amplitude per carrier).

**DSP systems engineer**: a sample-rate-accurate peak-detect
+ exponential gain follower is a textbook ~50-line implementation.
Numerical care: dB conversion (avoid `log(0)`), attack/release
filter coefficients computed from time constants, proper handling
of TX-burst boundaries (call `reset()` on new burst to model the
overshoot transient).

**Veteran HF operator**: this is the missing piece that separates
"works on the cable" from "works on the radio". KC3VPB called it
out 10 days ago. Until this lands, every test result is suspect
for real-radio deployment claims. The right operator answer to
"how do I drive?" is "set ALC to barely flicker" — but they
should EXPECT it to flicker, and the modem should EXPECT to see
some compression. This workstream validates that expectation.

## References

- `feedback_kc3vpb_alc_handling.md` (memory) — 2026-05-08 operator
  pushback.
- `src/ota_channel_core/models.cpp` — current channel models, the
  add-on point.
- `src/ota_channel_core/session_context.cpp::advanceSessionClock` —
  the mixer call order (ALC must run BEFORE noise).
- `tools/cli_simulator.cpp::setOtaChannel` — pattern for the
  `SetChannel` plumbing.
- `proto/ota_simulator.proto` — `SetChannelRequest`, `InjectEffectRequest`.
- `docs/CHANGELOG.md` 2026-05-14 "SimulatedChannel AWGN is
  continuous RX noise" — precedent for non-trivial channel work.
