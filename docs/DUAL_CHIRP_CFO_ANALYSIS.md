# Dual Chirp CFO Detection and Position Handling Analysis

**Date:** 2026-01-27
**Purpose:** Detailed analysis of dual chirp (radar technique) CFO estimation and sample position handling for IWaveform interface

---

## Executive Summary

The dual chirp system works correctly. CFO estimation is accurate (±0.5 Hz) and the returned `start_sample` position is consistent across both waveforms. **No additional CFO recalculation is needed downstream** - the chirp CFO is the most accurate and should be trusted.

Key findings:
1. **CFO calculation is correct** - Uses gap timing difference between up/down chirp peaks
2. **Position handling is now consistent** - Both MC-DPSK and OFDM_CHIRP return position of TRAINING start
3. **No duplicate CFO estimation needed** - Training-based CFO is less accurate and should be skipped when chirp CFO is available

---

## Part 1: Dual Chirp CFO Estimation (Radar Technique)

### How It Works

**Location:** `src/sync/chirp_sync.hpp:349-472`

The dual chirp uses an up-chirp (300→2700 Hz) followed by a down-chirp (2700→300 Hz). With CFO present:

```
Signal:         UP-CHIRP (300+CFO → 2700+CFO)   GAP   DOWN-CHIRP (2700+CFO → 300+CFO)
                        |                        |              |
                        v                        v              v
Correlation peak:   shifts LEFT by Δn       (gap shrinks)   shifts RIGHT by Δn

Where: Δn = CFO × (Fs / chirp_rate) = CFO × 10 samples per Hz
```

**Key insight:** The up and down chirps shift in **opposite directions**. The gap between them changes by `2 × Δn`:

```cpp
// chirp_sync.hpp:439-456
float T = config_.duration_ms / 1000.0f;  // 0.5 sec
float chirp_rate = (config_.f_end - config_.f_start) / T;  // 4800 Hz/s
float cfo_to_samples = config_.sample_rate / chirp_rate;   // 48000/4800 = 10 samples/Hz

int expected_gap = chirp_len + gap_samples;  // ~28800 samples (500ms chirp + 100ms gap)
int actual_gap = down_pos - up_pos;

// CFO shifts up_pos LEFT, down_pos RIGHT → gap increases by 2×CFO×10
float gap_error = actual_gap - expected_gap;
result.cfo_hz = gap_error / (2.0f * cfo_to_samples);  // Accurate to ±0.5 Hz!
```

### Complex Correlation (CFO-Tolerant Detection)

**Location:** `src/sync/chirp_sync.hpp:605-628`

Real correlation oscillates at CFO beat frequency. Complex correlation is CFO-invariant:

```cpp
// I/Q correlation
float corr_I = 0.0f;  // In-phase (cos)
float corr_Q = 0.0f;  // Quadrature (sin)

for (size_t i = 0; i < tmpl_sin.size(); i++) {
    float s = samples[offset + i];
    corr_I += s * tmpl_cos[i];
    corr_Q += s * tmpl_sin[i];
}

// Magnitude is CFO-invariant!
return std::sqrt(corr_I * corr_I + corr_Q * corr_Q) / denom;
```

**Why it works:**
- CFO adds phase rotation: `e^{j*2π*CFO*t}`
- Over correlation window, this rotates the I/Q phasor
- But `sqrt(I² + Q²)` = magnitude is constant regardless of rotation angle

### Position Correction

After finding raw peak positions, they're corrected using the estimated CFO:

```cpp
// chirp_sync.hpp:459-461
float up_correction = result.cfo_hz * cfo_to_samples;
result.up_chirp_start = round(up_pos + up_correction);    // Corrected TRUE position
result.down_chirp_start = round(down_pos - up_correction);
```

---

## Part 2: Sample Position Returned by detectSync()

### The Historical Issue

Previously there was confusion about what `SyncResult.start_sample` meant:
- **MC-DPSK expected:** Training start (process() expects training+ref+data)
- **OFDM_CHIRP expected:** Training start (processPresynced expects 2 LTS + data)
- **But acquisition thread returned:** Data start (post-chirp position)

This caused OFDM_CHIRP to fail because it needed to "rewind" to find training symbols.

### Current Implementation (CORRECT)

Both waveforms now return `start_sample` pointing to **TRAINING START**.

#### MC-DPSK (`mc_dpsk_waveform.cpp:114-137`)

```cpp
// detectSync() returns position of TRAINING start
if (chirp_result.success) {
    // Layout: [UP-CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING][REF][DATA...]
    //                                         ^-- start_sample points here
    size_t chirp_samples = chirp_sync_->getChirpSamples();  // 24000
    size_t gap_samples = config_.sample_rate * gap_ms / 1000.0f;  // 4800

    if (config_.use_dual_chirp) {
        // After dual chirp: 2×chirp + 2×gap
        result.start_sample = chirp_result.up_chirp_start +
                              2 * chirp_samples + 2 * gap_samples;
    } else {
        result.start_sample = chirp_result.up_chirp_start +
                              chirp_samples + gap_samples;
    }
    // NOTE: Do NOT add training_samples + ref_samples - process() needs them
}
```

#### OFDM_CHIRP (`ofdm_chirp_waveform.cpp:141-157`)

```cpp
// detectSync() returns position of TRAINING start
if (chirp_result.success) {
    // Layout: [UP-CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING_SYMBOLS][DATA...]
    //                                         ^-- start_sample points here
    size_t chirp_samples = chirp_sync_->getChirpSamples();  // 24000
    size_t gap_samples = config_.sample_rate * 100.0f / 1000.0f;  // 4800

    result.start_sample = chirp_result.up_chirp_start +
                          2 * chirp_samples + 2 * gap_samples;
    // NOTE: Do NOT add training_samples - process() needs them for channel estimation
}
```

### Frame Layout Comparison

```
MC-DPSK Frame:
┌────────────┬──────┬────────────┬──────┬──────────┬─────┬──────────┐
│ UP-CHIRP   │ GAP  │ DOWN-CHIRP │ GAP  │ TRAINING │ REF │ DATA...  │
│ 500ms      │100ms │ 500ms      │100ms │ 8 sym    │ 1   │          │
│ 24000 smp  │ 4800 │ 24000      │ 4800 │ 4096     │ 512 │ variable │
└────────────┴──────┴────────────┴──────┴──────────┴─────┴──────────┘
                                        ↑
                                   start_sample


OFDM_CHIRP Frame:
┌────────────┬──────┬────────────┬──────┬────────────────┬──────────┐
│ UP-CHIRP   │ GAP  │ DOWN-CHIRP │ GAP  │ 2× LTS (OFDM)  │ DATA...  │
│ 500ms      │100ms │ 500ms      │100ms │ 2× 640 samples │          │
│ 24000 smp  │ 4800 │ 24000      │ 4800 │ 1280 total     │ variable │
└────────────┴──────┴────────────┴──────┴────────────────┴──────────┘
                                        ↑
                                   start_sample
```

---

## Part 3: CFO Flow Through the System

### Complete CFO Path

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ 1. CHIRP DETECTION (chirp_sync.hpp)                                          │
│    detectDualChirp() → DualChirpResult { cfo_hz, up_chirp_start }           │
│    CFO accuracy: ±0.5 Hz (from gap timing difference)                        │
└──────────────────────────────────────────┬──────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ 2. IWaveform::detectSync() (mc_dpsk_waveform.cpp / ofdm_chirp_waveform.cpp) │
│    SyncResult { cfo_hz = chirp_result.cfo_hz, start_sample = ... }          │
└──────────────────────────────────────────┬──────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ 3. IWaveform::setFrequencyOffset(cfo_hz) - STORE CFO                        │
│                                                                              │
│    MC-DPSK:      cfo_hz_ = cfo_hz; demodulator_->setCFO(cfo_hz);            │
│    OFDM_CHIRP:   cfo_hz_ = cfo_hz; demodulator_->setFrequencyOffset(cfo_hz);│
└──────────────────────────────────────────┬──────────────────────────────────┘
                                           │
                                           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ 4. IWaveform::process(samples) - APPLY CFO CORRECTION                        │
│                                                                              │
│    MC-DPSK:                                                                  │
│      demodulator_->setChirpDetected(cfo_hz_);  // Set state + CFO           │
│      demodulator_->process(samples);                                         │
│        └→ processGotChirp():                                                 │
│             └→ applyCFOCorrection(sample_buffer_, cfo_hz_)  // Hilbert      │
│                 - CFO corrected BEFORE training/demodulation                 │
│                 - cfo_hz_ reset to 0 after correction                        │
│             └→ processTraining() // Skipped if we had chirp CFO            │
│             └→ demodulateSoft()  // Uses corrected samples                  │
│                                                                              │
│    OFDM_CHIRP:                                                               │
│      demodulator_->setFrequencyOffset(cfo_hz_);                             │
│        └→ sets chirp_cfo_estimated = true (flag to trust chirp CFO)         │
│      demodulator_->processPresynced(samples, 2);                             │
│        └→ toBaseband(): applies CFO correction via complex mixing            │
│        └→ Channel estimation from training symbols                           │
│        └→ DQPSK demodulation                                                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Why NO Additional CFO Estimation is Needed

1. **Chirp CFO is the most accurate**
   - Uses 1+ second of signal (two 500ms chirps)
   - Gap timing difference is CFO-invariant timing-wise
   - Accuracy: ±0.5 Hz

2. **Training-based CFO is LESS accurate**
   - Uses ~100ms of signal
   - Measures phase advance between symbols (noisy on fading)
   - Can give wrong estimates (e.g., 11 Hz when actual is 20 Hz)

3. **Both waveforms now preserve chirp CFO**

   **MC-DPSK** (`multi_carrier_dpsk.hpp:578-588`):
   ```cpp
   // Save chirp CFO before training processing
   float saved_cfo = cfo_hz_;
   bool has_chirp_cfo = external_chirp_detected_ && std::abs(dual_chirp_cfo) > 0.1f;

   processTraining(train_span);  // May corrupt cfo_hz_

   if (has_chirp_cfo) {
       cfo_hz_ = saved_cfo;  // Restore chirp CFO - it's more accurate
   }
   ```

   **OFDM_CHIRP** (`demodulator_impl.hpp` flag):
   ```cpp
   // When setFrequencyOffset() is called, it sets:
   chirp_cfo_estimated = true;

   // In processPresynced(), this flag tells us to trust chirp CFO:
   if (!chirp_cfo_estimated) {
       // Re-estimate from training symbols
   }
   // Otherwise use the chirp CFO as-is
   ```

---

## Part 4: IWaveform Interface Contract

### The Pattern for Using IWaveform

```cpp
// 1. Create waveform
OFDMChirpWaveform waveform;
// or: MCDPSKWaveform waveform(8);

// 2. Detect sync
SyncResult sync;
if (!waveform.detectSync(audio_samples, sync, 0.15f)) {
    // Not found
}

// 3. Apply CFO (stores it internally)
waveform.setFrequencyOffset(sync.cfo_hz);

// 4. Process samples from start_sample position
SampleSpan data_span(audio + sync.start_sample, remaining_len);
if (waveform.process(data_span)) {
    // 5. Get soft bits
    auto soft_bits = waveform.getSoftBits();
    // 6. LDPC decode...
}
```

### Key Interface Methods

| Method | Purpose | Notes |
|--------|---------|-------|
| `detectSync()` | Find chirp, estimate CFO, return training start | Returns `start_sample` = training start |
| `setFrequencyOffset(cfo)` | Store CFO for correction | Must call BEFORE process() |
| `process(samples)` | Demodulate, apply CFO correction | Samples start at training |
| `getSoftBits()` | Get demodulated soft bits | After process() returns true |
| `reset()` | Clear state between frames | Does NOT clear stored CFO |

### Critical Assumptions

1. **Samples passed to process() start at TRAINING, not DATA**
   - Both waveforms expect: `[TRAINING][REF/more-training][DATA...]`
   - The chirp preamble has already been consumed by detectSync()

2. **CFO correction happens inside process()**
   - Caller does NOT need to pre-correct samples
   - The waveform applies Hilbert-based frequency shift internally

3. **CFO is preserved across reset()**
   - For continuous tracking in connected mode
   - Use `setFrequencyOffset(0)` to explicitly clear

---

## Part 5: Test Verification

### test_iwaveform.cpp Usage Pattern

```cpp
// Line 106-203: decodeOFDMChirpFrame() shows correct usage

// 1. Detect
SyncResult sync_result;
if (!waveform.detectSync(audio, sync_result, 0.15f)) {
    return false;  // Not found
}

// 2. Apply CFO
waveform.setFrequencyOffset(sync_result.cfo_hz);

// 3. Process from start_sample (training start!)
SampleSpan data_span(audio.data() + sync_result.start_sample, ...);
waveform.process(data_span);

// 4. Get soft bits and decode
auto soft_bits = waveform.getSoftBits();
```

### Test Results (Verified 2026-01-27)

| Waveform | CFO (Hz) | Channel | Decode Rate |
|----------|----------|---------|-------------|
| MC-DPSK | 0 | AWGN | 100% |
| MC-DPSK | ±30 | AWGN | 100% |
| MC-DPSK | ±50 | AWGN | 100% |
| MC-DPSK | ±30 | Moderate | 100% |
| OFDM_CHIRP | 0 | AWGN | 100% |
| OFDM_CHIRP | ±30 | AWGN | 100% |
| OFDM_CHIRP | ±50 | AWGN | 100% |
| OFDM_CHIRP | ±30 | Moderate | 60-80% |

---

## Part 6: Remaining Issues / Future Work

### 1. ModemEngine Integration Not Using IWaveform for RX

The current `processRxBuffer_*` methods in ModemEngine still use the legacy demodulators directly, not the IWaveform interface. The RxPipeline was supposed to fix this but has bugs.

**Current state:**
- `test_iwaveform.cpp` works because it uses IWaveform directly
- ModemEngine's acquisition thread routes ALL chirp frames to MC-DPSK decoder (wrong for OFDM_CHIRP)

### 2. Acquisition Thread Doesn't Know Waveform Type

The acquisition thread detects chirps but doesn't know if the frame is MC-DPSK or OFDM_CHIRP. It routes everything through `rxDecodeDPSK()`, which is wrong for OFDM_CHIRP frames.

**Solution options:**
- A) Detect frame type from first symbol after chirp
- B) Have separate acquisition paths per waveform
- C) Use IWaveform interface in acquisition thread

### 3. RxPipeline Has Bugs with Chirp Detection

The RxPipeline was supposed to provide a clean streaming interface using IWaveform, but it has issues with chirp detection (see WIP commits).

---

## Summary

| Question | Answer |
|----------|--------|
| Is dual chirp CFO accurate? | **YES** - ±0.5 Hz accuracy |
| Is additional CFO estimation needed? | **NO** - Chirp CFO should be trusted |
| Where does start_sample point? | **TRAINING START** - consistent for both waveforms |
| Does caller need to correct CFO? | **NO** - process() handles it internally |
| Is this tested? | **YES** - test_iwaveform.cpp verifies the pattern |
| Is ModemEngine using this? | **NO** - Still uses legacy code paths |
