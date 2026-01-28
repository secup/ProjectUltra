# ProjectUltra Change Log

This log tracks all bug fixes and behavioral changes to prevent re-doing work due to lost context.

**Format:** Each entry must include:
1. What was broken (symptom + root cause)
2. What was changed (files, code)
3. How it's properly fixed (why it works, invariants)
4. Test verification (command + expected output)

---

## 2026-01-27: Improved Channel Interleaver Symbol Separation

**What was broken:**
- OFDM_CHIRP fading performance was lower than expected (~60% on good HF)
- Interleaver only separated consecutive bits by 1 symbol (step=61, separation=1)
- Burst errors from fading affected adjacent bits, making LDPC correction harder

**What was changed:**
- `src/fec/ldpc_decoder.cpp`: Modified `findCoprimeStep()` to target step = 3 × bits_per_symbol
- For 60 bits/symbol: step changed from 61 to 181, separation from 1 to 3

**How it's properly fixed:**
- Consecutive input bits now land in OFDM symbols 3 apart instead of adjacent
- When fading causes a burst error in one symbol, the affected bits are spread
  across the codeword after deinterleaving
- LDPC can correct scattered errors better than clustered errors

**Test verification:**
```bash
# Good HF channel at 20 dB
for seed in 1 2 3 4 5; do
  ./build/test_iwaveform --snr 20 -w ofdm_chirp --channel good --frames 5 --seed $seed
done
# Expected: 80-100% (was 60-100%)
```

---

## 2026-01-27: OFDM_CHIRP CFO Initial Phase Fix

**What was broken:**
- OFDM_CHIRP failed at any CFO > 0 Hz (CFO=30 Hz: 0% success)
- CFO=0 worked perfectly (100%)
- MC-DPSK at CFO=30 Hz worked (100%), proving chirp detection was correct
- Root cause: CFO correction started from phase 0 instead of accumulated phase

**Root cause analysis:**
1. Test harness applies CFO to entire audio from sample 0
2. By training start (sample ~136,800), CFO has accumulated ~307° of phase
3. `processPresynced()` reset `freq_correction_phase = 0`, losing this accumulated phase
4. First training symbol got wrong CFO correction, corrupting H estimate
5. DQPSK differential decoding failed due to phase mismatch

**What was changed:**
1. `include/ultra/ofdm.hpp` + `src/ofdm/demodulator.cpp`:
   - Added `setFrequencyOffsetWithPhase(float cfo_hz, float initial_phase_rad)`
   - Sets both CFO and initial correction phase

2. `src/waveform/ofdm_chirp_waveform.hpp` + `.cpp`:
   - Added `training_start_sample_` member variable
   - `detectSync()`: Stores training start position
   - `process()`: Calculates initial phase = -2π × CFO × training_start / sample_rate
   - Calls `setFrequencyOffsetWithPhase()` instead of `setFrequencyOffset()`

3. `src/ofdm/demodulator.cpp`:
   - `processPresynced()`: Removed reset of `freq_correction_phase` to preserve initial phase

4. `src/ofdm/channel_equalizer.cpp`:
   - Simplified `lts_carrier_phases` to use (1,0) reference
   - With correct initial phase, no phase compensation needed
   - Previous `conj(h_unit) * phase_advance` was wrong with correct initial phase

**How it's properly fixed:**
- Initial CFO phase = -2π × CFO × training_start_sample / sample_rate
- This matches the accumulated CFO phase in the signal at training start
- CFO correction is now continuous from sample 0 (effectively)
- Signal's +φ and correction's -φ cancel exactly: corrected = TX
- DQPSK reference = (1,0) because equalized = TX (no extra phase)

**Test verification:**
```bash
# Test full CFO range
for cfo in -50 -30 0 30 50; do
  ./build/test_iwaveform -w ofdm_chirp --snr 17 --cfo $cfo --frames 3
done
# Expected: 100% success for all CFO values
```

**Results:** OFDM_CHIRP now works with ±50 Hz CFO at 10-20 dB SNR.

---

## 2026-01-27: OFDM_CHIRP CFO Test Harness Fix

**What was broken:**
- OFDM_CHIRP decoding failed for most CFO values (only CFO=0 reliable)
- CFO=10 Hz: 0% success, CFO=30 Hz: 20% success
- Root cause: FIR Hilbert transform (127-tap) in test_iwaveform had 63-sample group delay
- This caused CFO-dependent timing shifts that broke OFDM symbol alignment

**What was changed:**
- `tools/test_iwaveform.cpp`: Replaced FIR Hilbert with FFT-based Hilbert (no group delay)
  - FFT signal -> zero negative frequencies, double positive -> IFFT
  - This creates perfect analytic signal without timing artifacts
- `src/sync/chirp_sync.hpp`: Removed HILBERT_GROUP_DELAY (63 sample) correction
  - Was compensating for old FIR delay which no longer exists

**How it's properly fixed:**
- FFT-based Hilbert has zero group delay (unlike FIR which has N/2 delay)
- CFO simulation now shifts frequency without shifting timing
- Chirp position correction only accounts for CFO-induced peak shift, not filter delay

**Test verification:**
```bash
# Test CFO range -45 to +50 Hz
for cfo in -45 -30 0 30 50; do
  ./test_iwaveform -w ofdm_chirp --snr 15 --cfo $cfo --frames 1
done
# Expected: 100% success for all CFO values
```

**Note:** This was a TEST HARNESS bug, not a demodulator bug. Real radios don't have this issue.

---

## 2026-01-27: CFO Accumulation Bug Fix

**What was broken:**
- MC-DPSK failed on subsequent frames when CFO ~0 Hz
- Frame 1 decoded, Frames 2+ failed LDPC
- Residual CFO from training accumulated via `cfo_hz_ += residual_cfo`

**What was changed:**
- `src/gui/modem/modem_rx_decode.cpp`: Always call `setCFO(frame.cfo_hz)` to reset accumulated CFO
- Previously only called when `abs(cfo_hz) > 0.1f`
- Fixed in 3 places: PING decode, CW0 decode, full frame decode

**How it's properly fixed:**
- `setCFO()` resets `cfo_hz_` to the chirp-detected value
- This prevents residual CFO from training from accumulating across frames
- Chirp CFO is then re-estimated for each frame independently

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 0 --channel awgn -w mc_dpsk --frames 5
# Expected: 100% decode rate (was 20% before fix)
```

**Commit:** `a2e6bed Fix CFO accumulation bug and improve test_iwaveform continuous RX`

---

## 2026-01-27: Demodulator Reset Per Frame

**What was broken:**
- Continuous RX decode degraded on subsequent frames at marginal SNR
- Demodulator state from previous frame affected current decode

**What was changed:**
- `src/gui/modem/modem_rx_decode.cpp`: Added `mc_dpsk_demodulator_->reset()` at start of `rxDecodeDPSK()`

**How it's properly fixed:**
- Reset clears carrier phases, previous symbols, and other state
- CFO is then set from chirp detection via `setCFO()`
- Each frame gets clean demodulator state

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 30 --channel awgn -w mc_dpsk --frames 5
# Expected: 100% decode rate
```

**Commit:** `e52705b Add demodulator reset at start of each DPSK frame decode`

---

## 2026-01-27: test_iwaveform Continuous RX Mode

**What was broken:**
- test_iwaveform created fresh RX ModemEngine per frame ("cheating")
- Didn't test realistic continuous audio streaming
- Buffer overflow when feeding too much audio at once

**What was changed:**
- `tools/test_iwaveform.cpp`: Use single RX ModemEngine for entire audio stream
- Add throttling pauses every 5 seconds to let acquisition process
- Reduce gap between frames (1.5s) to fit under MAX_PENDING_SAMPLES (960000)
- Track decoded frames by sequence number using std::set

**How it's properly fixed:**
- Realistic test: audio streamed continuously like from HF rig
- Throttling prevents buffer overflow (acquisition can't keep up with instant feed)
- Single RX instance tests state management between frames

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 30 --channel awgn -w mc_dpsk --frames 5
# Expected: 100% decode rate
```

**Commit:** `a2e6bed Fix CFO accumulation bug and improve test_iwaveform continuous RX`

---

## 2026-01-27: IWaveform Interface Documentation

**What was done:**
- Created comprehensive documentation for refactoring reference

**Files created:**
- `docs/MODEM_ENGINE_ARCHITECTURE.md` - Complete ModemEngine analysis
- `docs/DUAL_CHIRP_CFO_ANALYSIS.md` - CFO detection and position handling
- `docs/TESTING_METHODOLOGY.md` - Test tools and requirements

**Why it matters:**
- ModemEngine has two parallel code paths (old direct modulators, new IWaveform)
- RxPipeline integration has bugs - old `processRxBuffer_*` methods still work
- CFO must be applied via Hilbert transform, not simple multiplication

---

## 2026-01-27: OFDM_CHIRP Support in test_iwaveform

**What was broken:**
- test_iwaveform.cpp could not decode OFDM_CHIRP frames
- ModemEngine's acquisition thread routes ALL chirp frames to MC-DPSK decoder
- OFDMChirpWaveform::process() only returned 648 soft bits instead of all

**What was changed:**
- `tools/test_iwaveform.cpp`: Added `decodeOFDMChirpFrame()` that uses IWaveform directly
- `tools/test_iwaveform.cpp`: Added `setConnectWaveform()` call for TX (connect_waveform_ is used for disconnected mode TX, not waveform_mode_)
- `src/waveform/ofdm_chirp_waveform.cpp`: Fixed `process()` to loop and retrieve ALL soft bits from demodulator

**How it's properly fixed:**
- OFDM_CHIRP decode bypasses ModemEngine and uses IWaveform directly
- TX uses `setConnectWaveform(mode)` in addition to `setWaveformMode(mode)`
- `process()` now calls `demodulator_->getSoftBits()` in a loop until `hasPendingData()` returns false

**Test verification:**
```bash
./test_iwaveform --snr 17 --cfo 30 --channel awgn -w ofdm_chirp --frames 10
# Expected: 100% decode rate
```

**Commit:** `84bb563 Add OFDM_CHIRP support to test_iwaveform with CFO correction`

---

## 2026-01-27: MC-DPSK CFO Correction for Training/Reference Samples

**What was broken:**
- MC-DPSK decode failed with CFO on fading channels
- Training and reference samples were receiving UNCORRECTED signal
- `processTraining()` was estimating wrong residual CFO

**What was changed:**
- `src/psk/multi_carrier_dpsk.hpp`: CFO correction applied to training/ref samples BEFORE `processTraining()`
- Added public `applyCFO()` wrapper method that preserves `cfo_hz_` after correction

**How it's properly fixed:**
- CFO correction must happen BEFORE `processTraining()`, not after
- The demodulator's `applyCFOCorrection()` resets `cfo_hz_` to 0, so we save/restore it
- Chirp CFO is trusted over training CFO (more accurate from 1+ second signal)

**Invariants:**
1. CFO from chirp detection is the most accurate - trust it
2. Apply CFO to ALL samples (training, ref, data) before demodulation
3. Don't let `processTraining()` overwrite chirp CFO estimate

**Test verification:**
```bash
./test_iwaveform --snr 10 --cfo 30 --channel moderate -w mc_dpsk --frames 10
# Expected: 100% decode rate
```

**Commit:** `48e6271 Fix MC-DPSK CFO correction for training/reference samples`

---

## 2026-01-26: Complex Correlation for CFO-Tolerant Chirp Detection

**What was broken:**
- Real-valued chirp correlation oscillated at CFO beat frequency
- Detection position varied with CFO (±24-48 samples error)
- CFO estimation was inaccurate (~11.7 Hz for 20 Hz actual)

**What was changed:**
- `src/sync/chirp_sync.hpp`: Added cosine templates alongside sine templates
- `src/sync/chirp_sync.hpp`: New `computeComplexTemplateCorrelation()` returns magnitude √(I² + Q²)

**How it's properly fixed:**
- Complex correlation: I = Σ signal × cos(phase), Q = Σ signal × sin(phase)
- Magnitude √(I² + Q²) is CFO-invariant (phase rotation doesn't change magnitude)
- Peak position is now consistent regardless of CFO

**Invariants:**
1. Always use complex correlation for chirp detection
2. Dual chirp gap timing gives CFO estimate (up shifts left, down shifts right)
3. Position correction: `true_pos = detected_pos + CFO × 10`

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 50 --channel awgn -w mc_dpsk --frames 10
# Expected: 100% decode rate
```

---

## 2026-01-26: OFDM_CHIRP CFO - Trust Chirp Estimate

**What was broken:**
- OFDM_CHIRP decode failed with CFO
- Training symbol CFO estimation was overwriting correct chirp CFO
- Training was measuring carrier phase advance (wrong metric)

**What was changed:**
- `src/ofdm/demodulator_impl.hpp`: Added `chirp_cfo_estimated` flag
- Flag is set when `setFrequencyOffset()` is called
- `processPresynced()` trusts chirp CFO instead of re-estimating from training

**How it's properly fixed:**
- When chirp-based CFO is available, skip training-based re-estimation
- Training-based CFO is less accurate (100ms vs 1+ second signal)
- The `toBaseband()` function applies CFO correction before FFT

**Invariants:**
1. Chirp CFO > Training CFO in accuracy
2. Set `chirp_cfo_estimated = true` when CFO comes from chirp detection
3. Apply CFO in `toBaseband()` before FFT

**Test verification:**
```bash
./test_iwaveform --snr 17 --cfo 50 --channel awgn -w ofdm_chirp --frames 10
# Expected: 100% decode rate
```
