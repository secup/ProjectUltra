# CFO Correction Flow - Complete Reference

This document describes the complete CFO (Carrier Frequency Offset) correction flow, including the fading-channel correction loop and the 2026-02-12 verification tooling updates.

**Status:** Working and verified (100% CW on AWGN, 100% CW on good fading, ~83% on moderate)

---

## Overview

CFO correction has THREE runtime stages:
1. **Chirp-based coarse estimation** — during handshake (dual chirp preamble)
2. **LTS-based residual correction** — per frame (training symbol phase comparison)
3. **Pilot-based tracking** — per symbol (pilot carrier phase differences)

Plus a **feedback loop** that propagates corrections back to the cached CFO.

```
                    HANDSHAKE                    CONNECTED MODE (per frame)
                    ─────────                    ──────────────────────────

 Chirp Detection ──→ coarse CFO ──→ cached    ──→ LTS residual fix ──→ Pilot tracking
 (up/down gap)       (±50 Hz)      last_cfo_      (±5 Hz range)        (per-symbol)
                                      ↑                                      │
                                      └──────── CFO feedback loop ───────────┘
```

---

## The Fading Channel Problem (Fixed 2026-02-03)

### What went wrong

On fading channels, multipath distorts the chirp waveform. The up-chirp and down-chirp correlation peaks shift by different amounts due to frequency-selective fading. This causes a **false CFO estimate** (e.g., -1.4 Hz when actual CFO is 0 Hz).

Before the fix, this wrong CFO was:
1. Stored as `last_cfo_` in StreamingDecoder during handshake
2. Re-injected into every subsequent frame via `waveform_->setFrequencyOffset(sync_cfo_)`
3. Never corrected — the feedback loop didn't exist

### Evidence

- **BRAVO** (seed 43): Got CFO ≈ 0.0 Hz from chirp → **100% decode success**
- **ALPHA** (seed 42): Got CFO ≈ -1.4 Hz from chirp → **ALL failures on ALPHA**
- Same fading parameters, same SNR — the ONLY difference was the chirp CFO error

### Impact of wrong CFO

A 1.4 Hz error with 1280-sample symbols at 48 kHz causes:
- Phase drift per symbol: `2π × 1.4 × 1280 / 48000 ≈ 0.234 rad ≈ 13.4°`
- After 10 data symbols: **134° total drift**
- After 25 data symbols (full frame): **335° total drift**
- DQPSK decision boundary: 45° → fails after ~3 symbols of accumulation

### The fix (three parts)

1. **LTS residual CFO estimation** (`channel_equalizer.cpp`): Compare H estimates from 2 LTS training symbols. Phase rotation across all carriers = residual CFO.
2. **CFO feedback loop** (`ofdm_chirp_waveform.cpp` + `streaming_decoder.cpp`): After demodulation, propagate pilot-corrected CFO back to cached value.
3. **Re-process training** with corrected CFO for accurate channel estimate.

---

## Detailed Code Flow

### Stage 1: Chirp-Based Coarse CFO (Handshake Only)

**When:** During PING/PONG and CONNECT handshake (full preamble with dual chirp)

**File:** `src/sync/chirp_sync.hpp`

```
Up-chirp (300→2700 Hz) detected at position up_pos
Down-chirp (2700→300 Hz) detected at position down_pos

expected_gap = chirp_len + gap_samples    (28800 samples)
actual_gap = down_pos - up_pos
gap_error = actual_gap - expected_gap

CFO = gap_error / (2 × cfo_to_samples)
where cfo_to_samples = sample_rate / chirp_rate = 48000 / 4800 = 10 samples/Hz
```

**Why dual chirp works:** Up-chirp peak shifts LEFT by CFO×10 samples. Down-chirp peak shifts RIGHT by CFO×10 samples. The gap changes by 2×CFO×10.

**Accuracy:** ±0.5 Hz on AWGN, but ±2 Hz on fading channels due to multipath peak distortion.

**Storage path:**
```
chirp_sync.detectDualChirp() → result.cfo_hz
  → StreamingDecoder stores as last_cfo_
  → Passed to waveform via setFrequencyOffset()
  → Demodulator stores as freq_offset_hz, sets chirp_cfo_estimated = true
```

**Structure acquisition (`ULTRA_BURST_ACQ=structure`, 2026-11-08).** Connected group starts
carry no chirp, so there is no chirp CFO for them. The structure lock reports the TRACKED CFO
(`cfo_tracker_.tracked()`) as its CFO and uses it as the detector's known CFO (the χ statistic
tolerates a ±3 Hz unknown residual, test `FrameStructureDetector` §4);
`seedFromChirpConnectedAnchor` keeps the warm value, and Stage 2 below refines it per frame
exactly as after a warm chirp lock. Cold acquisition is unchanged (handshake chirps).

### Stage 2: LTS Residual CFO Correction (Per Frame)

**When:** Every frame, during `processPresynced()` → `estimateChannelFromLTS()`

**File:** `src/ofdm/channel_equalizer.cpp` (in `estimateChannelFromLTS()`)

**How it works:**
1. Two LTS training symbols are processed through `toBaseband()` (with current CFO correction)
2. For each data carrier, compute H[sym0] and H[sym1] (channel estimate per symbol)
3. Measure phase rotation: `phase_diff = arg(H[sym1] × conj(H[sym0]))`
4. Average across all carriers for robust estimate
5. Convert to residual CFO: `residual = avg_phase / (2π × T_symbol)`
6. If residual > 0.3 Hz: correct `freq_offset_hz` and re-process training symbols

```cpp
// Simplified logic:
Complex phase_diff_sum(0, 0);
for (each carrier i) {
    Complex diff = h_per_symbol[1][i] * conj(h_per_symbol[0][i]);
    phase_diff_sum += normalize(diff);
}
float avg_phase = atan2(phase_diff_sum.imag(), phase_diff_sum.real());
float residual_cfo = avg_phase * sample_rate / (2π × symbol_samples);

if (abs(residual_cfo) > 0.3 && abs(residual_cfo) < 5.0) {
    freq_offset_hz += residual_cfo;
    // Re-process training with corrected CFO
}
```

**Why 0.3 Hz threshold:** Below 0.3 Hz, the phase drift per symbol is < 5°, which DQPSK handles fine (45° margin). Above that, progressive drift accumulates and causes errors.

**Why re-process training:** The channel estimate from the first pass used the wrong CFO, so H has a built-in phase error. Re-processing with the corrected CFO gives a clean H for data symbols.

### Stage 3: Pilot-Based CFO Tracking (Per Symbol)

**When:** Every data symbol, in `updateChannelEstimate()`

**File:** `src/ofdm/channel_equalizer.cpp` (in `updateChannelEstimate()`)

**How it works:**
1. For each pilot carrier, compute current H from known pilot sequence
2. Compare with previous symbol's H: `phase_diff = arg(H_curr × conj(H_prev))`
3. Average phase difference = residual CFO × T_symbol
4. Update `freq_offset_hz` with adaptive alpha (fast at start, slow later)
5. Also updates `pilot_phase_correction` for equalization

```cpp
float residual_cfo = avg_phase_diff / (2π × symbol_duration);
float total_cfo = freq_offset_hz + residual_cfo;
freq_offset_filtered = alpha * total_cfo + (1 - alpha) * freq_offset_filtered;
freq_offset_hz = clamp(freq_offset_filtered, -MAX_CFO_HZ, MAX_CFO_HZ);
```

**Adaptive alpha:**
- First `CFO_ACQUISITION_SYMBOLS` symbols: alpha = 0.9 (fast acquisition)
- After acquisition: alpha = `FREQ_OFFSET_ALPHA` (slow tracking)
- Large residual (>10 Hz): alpha = 0.9 (emergency correction)

### Stage 4: CFO Feedback Loop (Per Frame)

**When:** After each frame is demodulated

**Files:**
- `src/waveform/ofdm_chirp_waveform.cpp` (in `process()`)
- `src/gui/modem/streaming_decoder.cpp` (after `waveform_->process()`)

**How it works:**
1. After `processPresynced()` completes, the demodulator has a pilot-corrected CFO
2. Waveform reads it back: `cfo_hz_ = demodulator_->getFrequencyOffset()`
3. Also updates `last_cfo_` so `estimatedCFO()` returns the corrected value
4. StreamingDecoder reads it: `last_cfo_.store(waveform_->estimatedCFO())`
5. Also updates `sync_cfo_` so the next frame uses the corrected value

```
Frame N processing:
  setFrequencyOffset(cached_cfo)     ←── may be wrong from chirp
  processPresynced()                  ←── LTS fixes residual, pilots track per-symbol
  cfo_hz_ = demodulator->getCFO()    ←── read back corrected value
  last_cfo_ = cfo_hz_                ←── update cached value

Frame N+1 processing:
  setFrequencyOffset(cached_cfo)     ←── now uses corrected value from frame N!
```

**Why this matters:** Without the feedback loop, every frame gets the same wrong chirp CFO. With it, the first frame might have some errors, but all subsequent frames use the corrected value.

---

## Stage 5: Simulator TX CFO Injection and Internal Chain Proof

The simulator now supports deterministic CFO stress testing and direct internal validation of the correction path.

### TX CFO injection (simulator)

**File:** `tools/cli_simulator.cpp`

- `--tx-cfo <Hz>` (alias `--cfo`) injects a transmitter CFO shift in the simulator path.
- Injection is applied with analytic-signal single-sideband shifting and per-direction phase continuity.
- This avoids image artifacts and preserves realistic stream behavior for long transfers.

Example:

```bash
./build/cli_simulator --snr 20 --channel awgn --waveform ofdm_chirp \
  --mod dqpsk --rate r1_2 --tx-cfo 50 --seed 42 --test
```

### Internal pre/post correction dumps

**File:** `src/ofdm/channel_equalizer.cpp` (inside `toBaseband()`)

Set environment variables before running simulator:

```bash
ULTRA_DUMP_CFO_PREFIX=/tmp/cfo_chain_dump
ULTRA_DUMP_CFO_CALLS=6
```

For each dump index `i`, the demod path writes:
- `<prefix>_<i>_pre.cf32` (after mixer/downconversion, before CFO correction)
- `<prefix>_<i>_post.cf32` (after CFO correction)
- `<prefix>_<i>_meta.txt` (sample rate, CFO, phase info)

### One-command verification harness

**Files:**
- `tests/verify_cfo_chain.sh`
- `tools/verify_cfo_chain_dump.py`

Run:

```bash
./tests/verify_cfo_chain.sh --cfo 50 --channel awgn --snr 20 --seed 42
```

The Python verifier estimates applied correction from:
- `post * conj(pre)` phase slope

Expected result:
- Applied correction near `-expected_cfo` (within tolerance), for all dumped frames.

---

## StreamingDecoder CFO Drift Limiting

**File:** `src/gui/modem/streaming_decoder.cpp` (lines 425-440)

When connected, new chirp-based CFO measurements are sanity-checked against the cached value:

```cpp
constexpr float MAX_CFO_DRIFT_HZ = 1.0f;
if (connected_ && abs(known_cfo) > 0.01f) {
    float cfo_diff = new_cfo - known_cfo;
    if (abs(cfo_diff) > MAX_CFO_DRIFT_HZ) {
        new_cfo = known_cfo;  // Reject noisy measurement
    }
}
```

**Purpose:** Multipath fading can cause chirp peak position errors that look like CFO changes. Real oscillator drift is slow (~0.1 Hz/minute). Anything faster than 1 Hz/frame is noise.

**Note:** This drift limiting works WITH the feedback loop. After the first frame corrects the CFO via pilots, the corrected value becomes the new baseline for drift limiting.

---

## Per-Sample CFO Correction

**File:** `src/ofdm/channel_equalizer.cpp` (in `toBaseband()`)

Every sample is corrected by a rotating phasor:

```cpp
phase_increment = -2π × freq_offset_hz / sample_rate;  // radians per sample

for (each sample i) {
    Complex osc = mixer.next();                    // Downconvert to baseband
    Complex mixed = samples[i] * conj(osc);        // Remove carrier

    Complex correction(cos(freq_correction_phase),  // CFO correction phasor
                       sin(freq_correction_phase));
    mixed *= correction;                            // Apply correction

    freq_correction_phase += phase_increment;       // Accumulate phase
}
```

**Key:** `freq_correction_phase` is continuous across symbols within a frame. It accumulates and wraps to [-π, π]. Between frames, it's reset by `setFrequencyOffsetWithPhase()`.

---

## Initial Phase Calculation

**File:** `src/waveform/ofdm_chirp_waveform.cpp` (in `process()`)

For connected-mode frames, the CFO correction must start at the correct accumulated phase:

```cpp
float initial_phase_rad = -2π × cfo_hz × training_start_sample / sample_rate;
demodulator_->setFrequencyOffsetWithPhase(cfo_hz_, initial_phase_rad);
```

**Why:** CFO is applied to the entire audio stream from sample 0. By the time we reach the current frame's training symbols, the CFO has already accumulated `training_start_sample` worth of phase. We must start correction from this accumulated phase, not from 0.

---

## DQPSK Reference and CFO Interaction

**File:** `src/ofdm/channel_equalizer.cpp` (in `estimateChannelFromLTS()`)

DQPSK differential decoding needs a per-carrier reference that captures the phase error from equalization. The reference is:

```
ref[i] = conj(H_est[i]) / |H_est[i]|
```

This ensures differential detection cancels the equalization phase error:
```
diff = eq_data × conj(ref) = TX_data × e^{-jφ} × conj(e^{-jφ}) = TX_data  ✓
```

**CFO interaction:** If CFO is corrected AFTER computing H_est (Stage 2 residual fix), we re-process the training symbols so H_est reflects the corrected CFO. Without re-processing, the reference would have a built-in phase mismatch with data symbols.

---

## Files Reference

| File | CFO Role |
|------|----------|
| `src/sync/chirp_sync.hpp` | Chirp generation, dual-chirp CFO estimation |
| `src/ofdm/demodulator.cpp` | `setFrequencyOffset()`, `processPresynced()`, `getFrequencyOffset()` |
| `src/ofdm/channel_equalizer.cpp` | `toBaseband()` per-sample correction, LTS residual estimation, pilot tracking |
| `src/ofdm/demodulator_impl.hpp` | State: `freq_offset_hz`, `freq_correction_phase`, `chirp_cfo_estimated` |
| `src/waveform/ofdm_chirp_waveform.cpp` | `process()` CFO feedback, initial phase calculation |
| `src/gui/modem/streaming_decoder.cpp` | `last_cfo_` caching, drift limiting, feedback update |
| `src/ofdm/ofdm_sync.cpp` | `estimateCFOFromTraining()` (fallback when no chirp) |
| `tools/cli_simulator.cpp` | TX CFO injection (`--tx-cfo`) and signal capture flags |
| `tests/verify_cfo_chain.sh` | End-to-end CFO verification gate command |
| `tools/verify_cfo_chain_dump.py` | Numeric verification of internal pre/post CFO correction |

---

## Invariants (DO NOT VIOLATE)

1. **Chirp CFO > Training CFO** — Chirp uses 1+ second of signal vs ~100ms for training. Always prefer chirp when available.
2. **LTS can REFINE chirp CFO** — On fading channels, chirp can be wrong by 1-2 Hz. LTS residual estimation corrects this. This is NOT the same as "training CFO estimation" (which is unreliable).
3. **CFO must be set BEFORE process()** — `setFrequencyOffset()` → `process()`, never reversed.
4. **Feedback loop must update cached CFO** — After demodulation, `cfo_hz_` and `last_cfo_` in the waveform AND `last_cfo_` in StreamingDecoder must be updated with the pilot-corrected value.
5. **Drift limiting threshold: 1 Hz** — Reject chirp CFO measurements that differ by >1 Hz from cached value when connected. Real oscillator drift is slow.
6. **Re-process training after CFO correction** — If LTS residual correction changes `freq_offset_hz`, the training symbols must be re-processed to get a clean channel estimate.
7. **Pilot-CFO feedback is OFDM-only** — Do not apply OFDM pilot-based CFO updates to MC-DPSK frames.

---

## Common Failure Modes

| Symptom | Cause | Fix |
|---------|-------|-----|
| One station 100%, other all failures | Fading distorted chirp → wrong CFO on one direction | LTS residual correction + feedback loop |
| Progressive phase drift in constellation | Wrong CFO not corrected between frames | CFO feedback loop |
| First frame fails, rest OK | LTS residual not estimated | Check threshold (0.3 Hz) |
| All frames fail equally | Systematic CFO error > 5 Hz | Check chirp detection, increase LTS correction range |
| CW[1]/CW[3] fail more than CW[0]/CW[2] | LLR imbalance in DQPSK demapper | Check `soft_demap.hpp` bit0 vs bit1 scaling |

---

*Document created: 2026-01-26*
*Major update: 2026-02-03 — Added fading CFO correction, LTS residual estimation, feedback loop*
*Major update: 2026-02-12 — Added simulator TX CFO injection, internal pre/post dump hooks, and one-command verification harness*
*Verified: 100% CW on good fading (3/3 runs, 120/120 CWs), 100% on AWGN*
