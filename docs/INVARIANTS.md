# Critical Invariants

**Purpose:** Document ALL critical rules and invariants that MUST be maintained. Violating these will cause subtle bugs that are hard to debug. Read this before making changes.

**IMPORTANT:** If you change any of these invariants, you MUST:
1. Update this document
2. Add entry to CHANGELOG.md explaining why
3. Update all affected code paths
4. Run full regression tests

---

## CFO (Carrier Frequency Offset) Invariants

### INV-CFO-001: Chirp CFO is Always More Accurate Than Training CFO

**Rule:** When chirp-based CFO is available, ALWAYS use it. Never let training-based CFO estimation overwrite it.

**Why:** Chirp uses 1+ second of signal (two 500ms chirps). Training uses ~100ms. Longer signal = more accurate.

**Implementation:**
```cpp
// MC-DPSK: Save and restore chirp CFO around training processing
float saved_cfo = cfo_hz_;
processTraining(samples);  // May corrupt cfo_hz_
if (has_chirp_cfo) {
    cfo_hz_ = saved_cfo;  // Restore chirp CFO
}

// OFDM: Flag to skip training CFO estimation
if (chirp_cfo_estimated) {
    // Use pre-set CFO, skip estimateCFOFromTraining()
}
```

**Locations:**
- `src/psk/multi_carrier_dpsk.hpp:578-588`
- `src/ofdm/demodulator_impl.hpp` (chirp_cfo_estimated flag)

---

### INV-CFO-002: CFO Must Be Applied BEFORE Demodulation

**Rule:** Call `setFrequencyOffset()` BEFORE `process()`. Never after.

**Why:** CFO correction happens sample-by-sample during baseband conversion. If you set it after processing, it's too late.

**Correct order:**
```cpp
waveform->reset();
waveform->detectSync(samples, result);
waveform->setFrequencyOffset(result.cfo_hz);  // BEFORE process!
waveform->process(samples);
```

**Wrong order (WILL FAIL):**
```cpp
waveform->process(samples);
waveform->setFrequencyOffset(cfo);  // TOO LATE!
```

---

### INV-CFO-003: CFO Simulation Must Use Hilbert Transform

**Rule:** To simulate CFO in test signals, use Hilbert transform for single-sideband frequency shift. Simple `cos(2π·CFO·t)` multiplication creates mirror images.

**Why:** Real frequency shift is `e^{j·2π·CFO·t}` multiplication on complex signal. For real signals, we need Hilbert transform to get the analytic signal first.

**Correct implementation (test_iwaveform.cpp:76-101):**
```cpp
// 1. Compute Hilbert transform (imaginary part)
std::vector<float> hilbert = computeHilbert(signal);

// 2. Create analytic signal: z = signal + j·hilbert
// 3. Multiply by e^{j·2π·CFO·t}
// 4. Take real part
for (size_t i = 0; i < signal.size(); i++) {
    float phase = 2.0f * M_PI * cfo_hz * i / sample_rate;
    float cos_p = std::cos(phase);
    float sin_p = std::sin(phase);
    signal[i] = signal[i] * cos_p - hilbert[i] * sin_p;
}
```

**Wrong (creates images):**
```cpp
signal[i] *= std::cos(2.0f * M_PI * cfo_hz * i / sample_rate);  // WRONG!
```

---

## Chirp Detection Invariants

### INV-CHIRP-001: Use Complex Correlation for Chirp Detection

**Rule:** Always use I/Q (complex) correlation, never real-only correlation.

**Why:** Real correlation oscillates at the CFO beat frequency, causing position errors. Complex correlation magnitude is CFO-invariant.

**Correct:**
```cpp
float corr_I = sum(signal * cos(chirp_phase));  // In-phase
float corr_Q = sum(signal * sin(chirp_phase));  // Quadrature
float magnitude = sqrt(corr_I*corr_I + corr_Q*corr_Q);  // CFO-invariant!
```

**Location:** `src/sync/chirp_sync.hpp:605-628`

---

### INV-CHIRP-002: Dual Chirp Gap Timing Gives CFO

**Rule:** CFO is calculated from the gap change between up and down chirp peaks.

**Formula:**
```
expected_gap = chirp_length + gap_duration  (samples)
actual_gap = down_pos - up_pos
gap_error = actual_gap - expected_gap
CFO = gap_error / (2 × cfo_to_samples)

where: cfo_to_samples = sample_rate / chirp_rate = 48000 / 4800 = 10 samples/Hz
```

**Why opposite shifts:**
- Up chirp (300→2700 Hz): CFO shifts correlation peak LEFT by CFO×10 samples
- Down chirp (2700→300 Hz): CFO shifts correlation peak RIGHT by CFO×10 samples
- Total gap change = 2 × CFO × 10 samples

**Location:** `src/sync/chirp_sync.hpp:439-456`

---

## Sample Position Invariants

### INV-POS-001: detectSync() Returns TRAINING Start, Not Data Start

**Rule:** `SyncResult.start_sample` points to where TRAINING symbols begin, not where data begins.

**Why:** Both MC-DPSK and OFDM need training symbols for channel estimation. The waveform's `process()` method expects to receive training+data, not just data.

**Frame layout:**
```
[CHIRP][GAP][CHIRP][GAP][TRAINING][REF/MORE-TRAINING][DATA...]
                        ^
                        start_sample points HERE
```

**Locations:**
- `src/waveform/mc_dpsk_waveform.cpp:114-137`
- `src/waveform/ofdm_chirp_waveform.cpp:141-157`

---

### INV-POS-002: Position Correction After CFO Estimation

**Rule:** Raw chirp peak positions must be corrected using estimated CFO before returning.

**Formula:**
```cpp
float up_correction = cfo_hz * cfo_to_samples;
corrected_up_pos = raw_up_pos + up_correction;
corrected_down_pos = raw_down_pos - up_correction;
```

**Why:** The correlation peak shifts with CFO. We correct the position to get the true chirp start.

**Location:** `src/sync/chirp_sync.hpp:459-461`

---

## IWaveform Interface Invariants

### INV-WAVE-001: Call Sequence Must Be Exact

**Rule:** The IWaveform usage pattern is:
```cpp
1. waveform->reset();                           // Clear state
2. waveform->detectSync(samples, result);       // Find preamble
3. waveform->setFrequencyOffset(result.cfo_hz); // Store CFO
4. waveform->process(samples_from_start);       // Demodulate
5. auto bits = waveform->getSoftBits();         // Get output
```

**Violations that will fail:**
- Calling `process()` before `detectSync()` - no sync position
- Calling `reset()` after `setFrequencyOffset()` - clears CFO
- Calling `getSoftBits()` before `process()` returns true - no data

---

### INV-WAVE-002: reset() Clears CFO State

**Rule:** `reset()` MUST clear `cfo_hz_` to prevent stale values from previous frames.

**Why:** If CFO isn't cleared, a frame with 30 Hz CFO followed by a frame with 0 Hz CFO would incorrectly use 30 Hz.

**Implementation:**
```cpp
void reset() {
    demodulator_->reset();
    synced_ = false;
    cfo_hz_ = 0.0f;      // MUST clear!
    last_cfo_ = 0.0f;    // MUST clear!
}
```

---

## LDPC Invariants

### INV-LDPC-001: Soft Bits Are Log-Likelihood Ratios

**Rule:** Soft bits passed to LDPC decoder are LLRs: positive = likely 0, negative = likely 1.

**Scale:** Typical range is [-10, +10]. Values outside this may indicate scaling issues.

**Sign convention:**
- LLR > 0 → bit is likely 0
- LLR < 0 → bit is likely 1
- |LLR| = confidence (higher = more certain)

---

### INV-LDPC-002: Codeword Size is Fixed at 648 Bits

**Rule:** All LDPC codewords are exactly 648 bits. Frame must contain integer number of codewords.

**Data bits by rate:**
- R1/4: 162 data bits per codeword
- R1/2: 324 data bits per codeword
- R2/3: 432 data bits per codeword
- R3/4: 486 data bits per codeword
- R5/6: 540 data bits per codeword

---

## Thread Safety Invariants

### INV-THREAD-001: Acquisition and Decode Threads Share Buffer

**Rule:** The sample buffer is written by acquisition thread and read by decode thread. Access must be synchronized.

**Current implementation:** Ring buffer with read/write indices.

**Hazard:** If decode thread reads while acquisition writes, data corruption occurs.

---

### INV-THREAD-002: State Variables Need Atomic Access

**Rule:** Variables accessed by multiple threads (e.g., `connected_`, `waveform_mode_`) need atomic operations or mutex protection.

**Locations to audit:**
- `src/gui/modem/modem_engine.hpp` - member variables
- `src/gui/modem/modem_rx.cpp` - acquisition thread
- `src/gui/modem/modem_rx_decode.cpp` - decode thread

---

## Testing Invariants

### INV-TEST-001: SNR Calculated from Active Samples Only

**Rule:** SNR must be calculated from the portion of the signal that contains actual transmission, not including silence.

**Why:** Including silence inflates noise power, giving wrong SNR.

**Correct:**
```cpp
float signal_power = computePower(signal, chirp_start, signal_end);
float noise_power = computePower(noise, chirp_start, signal_end);
float snr_db = 10 * log10(signal_power / noise_power);
```

---

### INV-TEST-002: Channel Must Be Applied to Entire Signal

**Rule:** HF channel simulation (fading, delay spread) must be applied to the ENTIRE signal including preamble, not just data.

**Why:** Real HF affects everything. Testing only data portion would be unrealistic.

---

## Modulation Invariants

### INV-MOD-001: DQPSK Uses Previous Symbol as Reference

**Rule:** Differential QPSK encodes data as phase CHANGE from previous symbol, not absolute phase.

**Decode:** `data = current_phase - previous_phase`

**Why this is robust:** Immune to constant phase offset from channel. Only needs phase to be stable between adjacent symbols.

---

### INV-MOD-002: Pilot Symbols Required for Coherent Modulation

**Rule:** Coherent modes (QPSK, 16QAM, 32QAM) REQUIRE pilot symbols for channel estimation. Differential modes (DQPSK, D8PSK) do NOT.

**OFDM_COX:** Supports both, automatically inserts pilots for coherent modes.
**OFDM_CHIRP:** Hardcoded to DQPSK, no pilots needed.
**MC-DPSK:** Always DQPSK, no pilots.

---

## Summary Checklist

Before submitting code changes, verify:

- [ ] CFO from chirp is not overwritten by training estimation
- [ ] `setFrequencyOffset()` called before `process()`
- [ ] CFO simulation uses Hilbert transform (if applicable)
- [ ] Complex correlation used for chirp detection
- [ ] `start_sample` points to training start
- [ ] `reset()` clears CFO state
- [ ] Thread-shared variables are protected
- [ ] SNR calculated from active samples only
- [ ] All tests pass with CFO = 0 AND CFO = ±30 Hz
