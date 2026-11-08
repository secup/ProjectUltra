# Critical Invariants

**Purpose:** Document ALL critical rules and invariants that MUST be maintained. Violating these will cause subtle bugs that are hard to debug. Read this before making changes.

**IMPORTANT:** If you change any of these invariants, you MUST:
1. Update this document
2. Add entry to CHANGELOG.md explaining why
3. Update all affected code paths
4. Run full regression tests

---

## Simulator Channel Invariants

### INV-SIM-AWGN-001: AWGN Is Continuous RX Noise

**Rule:** `SimulatedChannel` synthetic AWGN is added at RX time to every
returned sample when the channel is explicitly configured as `ChannelType::AWGN`.
Noise must not depend on whether a peer TX buffer has audio, silence, or no
samples. Do not add synthetic AWGN on both TX and RX.

**Reference:** AWGN standard deviation is computed from the fixed modem PING
reference measured from `StreamingEncoder::encodePing()` output: broadband RMS
`0.3180724`, then receiver-FIR in-band RMS `0.30482664`. Configured `snr_db` is
operator-facing in-band SNR: the generated broadband white-noise sigma is
increased by `1 / sqrt(kModemInBandNoisePowerFraction)` so the 50-2950 Hz
receive FIR leaves the requested in-band noise power.

**Why:** Real HF receivers hear band noise continuously. A simulator that is
quiet between bursts lets sync, PING detection, SNR estimation, and decode
admission gates calibrate against impossible silence.

**Boundaries:**
- Watterson fading owns its own channel noise and must not receive an extra
  `SimulatedChannel` AWGN layer.
- The real-HF WAV noise overlay remains an independent additive RX overlay.
- No modem PHY threshold should be tuned against simulator silence floors.

**Location:** `tools/sim/simulated_station.hpp::SimulatedChannel`.

---

## CFO (Carrier Frequency Offset) Invariants

### INV-CFO-001: Chirp CFO is Coarse — LTS Refines It

**Rule:** Chirp CFO is more accurate than training-based estimation on AWGN, but on fading channels it can be wrong by ±2 Hz. The LTS residual estimation in `estimateChannelFromLTS()` REFINES chirp CFO per frame. The pilot tracking FURTHER refines it per symbol. The feedback loop propagates corrections back.

**Why:** Fading distorts chirp peaks → false CFO. LTS compares 2 known training symbols to measure the residual. Pilots track per-symbol drift. Without this pipeline, a 1.4 Hz error causes 13°/symbol phase drift → DQPSK decode failures.

**Implementation (4 stages):**
```
1. Chirp coarse CFO → stored as last_cfo_ (±50 Hz range, ±2 Hz accuracy on fading)
2. LTS residual fix → corrects freq_offset_hz if residual > 0.3 Hz
3. Pilot tracking → refines freq_offset_hz per data symbol
4. Feedback loop → propagates corrected CFO back to waveform and StreamingDecoder
```

**See `docs/CFO_CORRECTION_FLOW.md` for full details.**

**Locations:**
- `src/sync/chirp_sync.hpp` (chirp detection)
- `src/ofdm/channel_equalizer.cpp` (LTS residual, pilot tracking, toBaseband)
- `src/waveform/ofdm_chirp_waveform.cpp` (feedback: `cfo_hz_ = demodulator_->getFrequencyOffset()`)
- `src/gui/modem/streaming_decoder.cpp` (feedback: `last_cfo_.store(corrected_cfo)`)
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

**Correct implementation:** see the FFT/Hilbert CFO injection used by the maintained simulator/test tools.
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

## Structure Acquisition Invariants (`ULTRA_BURST_ACQ=structure`, 2026-11-08)

### INV-SYNC-STRUCT-001: Structure detection is for GROUP STARTS only

**Rule:** `IWaveform::detectFrameStructure` runs only in the connected group-start search
(`searchForSync`, `structure_anchor_search` — where chirp mode waits for the dual chirp). Never
route `detectDataSync`, warm windows or the generic light search through it.

**Why:** its template spans 10 symbols (11 200 samples); the warm narrow window is 4 416 and the
light window 9 600 samples, so it can never fire there. The first build hooked it into
`detectDataSync` and stalled every transfer at 6 % (no data frame was ever acquired → RTO loop).

**Location:** `streaming_sync_acquisition.cpp` `searchForSync`; test `FrameStructureDetector` §5.

### INV-SYNC-STRUCT-002: Test BOTH marker hypotheses

**Rule:** a group-start frame carries LTS1 negated (`markFirstLTSSymbolForBurstGroup`). Any
coherent statistic that sums LTS1-referenced and LTS2-referenced pairs together must evaluate
both signs (χ₊ = Σ|B+A|²/n, χ₋ = Σ|B−A|²/n) and count both in the union bound.

**Why:** the single sum cancels on marked frames: the marked head was missed, the unmarked
frame 2 locked as a "group start", and every member was dropped as HEADNULL.

**Location:** `ofdm_chirp_waveform.cpp` `detectFrameStructure` (`chiOf`); test §6.

### INV-SYNC-STRUCT-003: The search window is derived, not chosen

**Rule:** window = template span + `SyncController::kSearchBacktrackSamples` +
`kRmsProbeSamples` (`structureSearchWindowSamples`). Once the RMS probe sees a frame, the window
starts one backtrack before the cursor, so the frame can begin up to backtrack + probe into it.
Change the backtrack or the probe only through those constants.

### INV-SYNC-STRUCT-004: The threshold comes from a false-lock budget

**Rule:** per-pass budget = (false locks per hour) × `CORRELATION_STEP` / (3600 · fs), split by
the detector over candidates × templates × 2 marker hypotheses; threshold = Gamma(K,1) inverse
survival. Never replace it with a tuned χ number.

### INV-SYNC-STRUCT-005: Where the chirp stays

**Rule:** handshake (PING/PONG/CONNECT), MC-DPSK, OFDM_NARROW, the session-first burst and the
handshake-confirmation window (the half-open MC-DPSK CONNECT fallback needs a dual-chirp lock),
and RTO resends (no ACK evidence ⇒ cold re-anchor). A chirp-less burst start gets the settling
lead-in fill (30 ms silent T/R floor first).

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

### INV-THREAD-001: Audio Feed Thread and Decode Thread Share Decoder Buffer

**Rule:** `StreamingDecoder` buffer/state is concurrently touched by:
- audio feed path (`feedAudio()`), and
- decode path (`processBuffer()`).

Access to shared decoder internals must stay synchronized through decoder-owned locks/atomics.

**Current implementation:** bounded circular buffer in `StreamingDecoder` with mutex + condition variable coordination.

**Hazard:** unsynchronized pointer/state updates can produce false syncs, dropped frames, or buffer corruption.

---

### INV-THREAD-002: Shared Runtime State Requires Atomic/Mutex Protection

**Rule:** cross-thread state (connection flags, TX/RX indicators, queue contents, meters) must use atomics or mutexes.

**Locations to audit:**
- `src/gui/modem/streaming_decoder.cpp` - shared RX decode state
- `src/gui/modem/modem_rx.cpp` - decode-thread lifecycle and mode switching
- `src/gui/audio_engine.cpp` - SDL callback queue access
- `src/gui/app.cpp` - simulator thread shared buffers/flags

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
