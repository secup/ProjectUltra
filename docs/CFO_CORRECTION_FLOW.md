# CFO Correction Flow - Working Implementation Reference

This document describes the working CFO (Carrier Frequency Offset) correction flow in `ModemEngine`. Use this as reference when implementing CFO support in the new `IWaveform` interface.

**Status:** Working and verified on 2026-01-26 (100% decode at SNR=20dB, CFO=±50Hz)

---

## Overview

CFO correction requires coordination between:
1. **TX side:** Apply CFO to chirp preamble AND OFDM carrier
2. **RX side:** Detect chirp, estimate CFO, apply correction before demodulation

The key insight: **Chirp-based CFO estimation is more robust than training symbol estimation** at low SNR. We trust the chirp CFO and skip re-estimation from training symbols.

---

## TX Path (Simulating Radio Frequency Error)

### 1. ChirpSync - Chirp Preamble Generation

**File:** `src/sync/chirp_sync.hpp` (lines 84-101)

```cpp
// In ChirpSync::generate():
float cfo = config_.tx_cfo_hz;  // e.g., 30 Hz

// UP-CHIRP: 300 → 2700 Hz, shifted by CFO
float f_start_up = config_.f_start + cfo;  // 300 + 30 = 330 Hz
for (size_t i = 0; i < chirp_samples; i++) {
    float t = static_cast<float>(i) / config_.sample_rate;
    float phase = 2.0f * M_PI * (f_start_up * t + 0.5f * k * t * t);
    output[i] = config_.amplitude * std::sin(phase);
}

// DOWN-CHIRP: 2700 → 300 Hz, also shifted by CFO
float f_start_down = config_.f_end + cfo;  // 2700 + 30 = 2730 Hz
// ... similar
```

**Key:** Both up and down chirps are shifted by the same CFO. This creates a gap difference that the RX can measure.

### 2. OFDMModulator - OFDM Carrier Shift

**File:** `src/ofdm/modulator.cpp` (lines 130-137)

```cpp
// In OFDMModulator::Impl constructor:
mixer(cfg.center_freq + cfg.tx_cfo_hz, cfg.sample_rate)  // 1500 + 30 = 1530 Hz

if (std::abs(cfg.tx_cfo_hz) > 0.01f) {
    printf("[OFDM_MOD] *** CFO=%.1f Hz applied (mixer: %.1f + %.1f = %.1f Hz) ***\n",
           cfg.tx_cfo_hz, (float)cfg.center_freq, cfg.tx_cfo_hz,
           cfg.center_freq + cfg.tx_cfo_hz);
}
```

**Key:** The OFDM carrier frequency is shifted by the same CFO as the chirp.

### 3. ModemEngine Configuration

**File:** `src/gui/modem/modem_engine.cpp` (lines 64-68, 146-150)

```cpp
// Pass TX CFO to chirp generator
chirp_cfg.tx_cfo_hz = config_.tx_cfo_hz;
chirp_sync_ = std::make_unique<sync::ChirpSync>(chirp_cfg);

// OFDM modulator also gets tx_cfo_hz via ModemConfig
```

---

## RX Path (Detecting and Correcting CFO)

### 1. Dual Chirp Detection and CFO Estimation

**File:** `src/sync/chirp_sync.hpp` (lines 349-472)

```cpp
DualChirpResult ChirpSync::detectDualChirp(SampleSpan samples, float threshold) {
    // Detect UP chirp using COMPLEX correlation (CFO-tolerant!)
    auto [up_pos, up_corr] = detectChirpTemplate(samples, up_chirp_template_,
                                                  up_chirp_template_cos_, ...);

    // Detect DOWN chirp
    auto [down_pos_rel, down_corr] = detectChirpTemplate(down_search, down_chirp_template_,
                                                          down_chirp_template_cos_, ...);

    // Calculate CFO from position difference
    // - UP chirp correlation peak shifts by: -CFO * (Fs / chirp_rate)
    // - DOWN chirp correlation peak shifts by: +CFO * (Fs / chirp_rate)
    float cfo_to_samples = config_.sample_rate / chirp_rate;  // = 10 samples/Hz

    int expected_gap = chirp_len + gap_samples;  // 28800 samples
    int actual_gap = down_pos - up_pos;
    float gap_error = actual_gap - expected_gap;

    // CFO = gap_error / (2 * cfo_to_samples)
    result.cfo_hz = gap_error / (2.0f * cfo_to_samples);

    // Correct positions using estimated CFO
    float up_correction = result.cfo_hz * cfo_to_samples;
    result.up_chirp_start = up_pos + up_correction;  // True chirp start
}
```

**Key insight:** Complex correlation (I/Q) is CFO-invariant in magnitude. The correlation peak shifts by CFO, but the magnitude remains the same. Dual chirp lets us measure the shift and calculate CFO.

### 2. ModemEngine RX - Chirp Detection

**File:** `src/gui/modem/modem_rx_decode.cpp` (lines 967-997)

```cpp
void ModemRxDecode::processOFDMChirp() {
    // Detect dual chirp
    auto chirp_result = chirp_sync_->detectDualChirp(samples_span, CHIRP_THRESHOLD);

    if (chirp_result.success) {
        int chirp_start = chirp_result.up_chirp_start;  // CFO-corrected position
        float cfo_hz = chirp_result.cfo_hz;             // Estimated CFO

        // Set CFO on demodulator BEFORE processing training symbols
        if (std::abs(cfo_hz) > 0.5f) {
            ofdm_demodulator_->setFrequencyOffset(cfo_hz);  // THE KEY CALL!
        } else {
            ofdm_demodulator_->setFrequencyOffset(0.0f);
        }
    }
}
```

**Key:** `setFrequencyOffset()` must be called BEFORE `processPresynced()`.

### 3. OFDMDemodulator - Set CFO and Mark as Trusted

**File:** `src/ofdm/demodulator.cpp` (lines 767-776)

```cpp
void OFDMDemodulator::setFrequencyOffset(float cfo_hz) {
    impl_->freq_offset_hz = cfo_hz;
    impl_->freq_offset_filtered = cfo_hz;
    impl_->freq_correction_phase = 0.0f;  // Reset phase for fresh start

    // CRITICAL: Mark that CFO was provided by chirp (trusted source)
    impl_->chirp_cfo_estimated = true;
}
```

**Key:** `chirp_cfo_estimated = true` tells `processPresynced()` to SKIP training-based CFO re-estimation.

### 4. OFDMDemodulator::processPresynced - Trust Chirp CFO

**File:** `src/ofdm/demodulator.cpp` (lines 862-879)

```cpp
bool OFDMDemodulator::processPresynced(SampleSpan samples, int training_symbols) {
    // ...reset state but PRESERVE freq_offset_hz...

    // CRITICAL: Skip training CFO estimation if chirp provided CFO
    if (training_symbols >= 2 && !impl_->chirp_cfo_estimated &&
        std::abs(impl_->freq_offset_hz) < 0.1f) {
        // Only estimate from training if NO chirp CFO
        float cfo = impl_->estimateCFOFromTraining(ptr, training_symbols);
        impl_->freq_offset_hz = cfo;
    } else {
        // USE the pre-set CFO from chirp detection
        LOG_SYNC(INFO, "Using pre-set CFO: %.1f Hz (chirp=%s)",
                 impl_->freq_offset_hz,
                 impl_->chirp_cfo_estimated ? "yes" : "no");
    }

    // Continue with channel estimation using training symbols...
}
```

**Why skip training CFO?** Training-based CFO estimation requires correct baseband conversion, which needs CFO correction - chicken-and-egg problem. Chirp-based estimation works directly on passband signal.

### 5. toBaseband - Apply CFO Correction

**File:** `src/ofdm/channel_equalizer.cpp` (lines 19-54)

```cpp
std::vector<Complex> OFDMDemodulator::Impl::toBaseband(SampleSpan samples) {
    // Phase increment per sample: -2π * CFO / Fs
    float phase_increment = -2.0f * M_PI * freq_offset_hz / config.sample_rate;

    for (size_t i = 0; i < samples.size(); ++i) {
        // Mix down to baseband (remove carrier)
        Complex osc = mixer.next();
        Complex mixed = samples[i] * std::conj(osc);

        // Apply CFO correction by counter-rotating
        if (std::abs(freq_offset_hz) > 0.01f) {
            Complex correction(std::cos(freq_correction_phase),
                               std::sin(freq_correction_phase));
            mixed *= correction;  // Multiply by e^(-j*phase)
            freq_correction_phase += phase_increment;
            // Wrap phase to [-π, π]
        }

        baseband[i] = mixed;
    }
    return baseband;
}
```

**Key:** CFO correction is a continuous phase rotation applied sample-by-sample. The phase accumulates across the entire frame.

---

## Call Sequence for IWaveform Implementation

When implementing CFO for the new `IWaveform` interface, follow this exact sequence:

```cpp
// 1. Reset waveform (clears demodulator state, including CFO)
waveform->reset();

// 2. Detect chirp and get CFO estimate
SyncResult sync_result;
bool found = waveform->detectSync(rx_samples, sync_result, threshold);

// 3. MUST set CFO on waveform BEFORE process()
//    This should internally call demodulator->setFrequencyOffset()
//    which sets chirp_cfo_estimated = true
waveform->setFrequencyOffset(sync_result.cfo_hz);

// 4. Process audio starting from training symbols
//    The demodulator will use the pre-set CFO in toBaseband()
SampleSpan process_span(rx_samples.data() + training_start, len);
bool ready = waveform->process(process_span);

// 5. Get soft bits and decode
auto soft_bits = waveform->getSoftBits();
```

**Critical points:**
1. `reset()` MUST clear `cfo_hz_` in the waveform (to avoid stale values)
2. `setFrequencyOffset()` MUST call `demodulator_->setFrequencyOffset()` internally
3. `process()` should NOT have a threshold check like `if (cfo_hz_ > 0.1f)` - ALWAYS apply CFO
4. CFO of 0 Hz is a VALID estimate - don't skip it!

---

## Common Mistakes to Avoid

### Mistake 1: Conditional CFO Application
```cpp
// WRONG - skips CFO if small
if (std::abs(cfo_hz_) > 0.1f) {
    demodulator_->setFrequencyOffset(cfo_hz_);
}

// CORRECT - always apply
demodulator_->setFrequencyOffset(cfo_hz_);
```

### Mistake 2: Not Resetting CFO State
```cpp
// WRONG - cfo_hz_ retains stale value
void reset() {
    demodulator_->reset();
    synced_ = false;
}

// CORRECT - clear CFO state too
void reset() {
    demodulator_->reset();
    synced_ = false;
    cfo_hz_ = 0.0f;
    last_cfo_ = 0.0f;
}
```

### Mistake 3: Calling reset() After setFrequencyOffset()
```cpp
// WRONG - reset clears the CFO we just set!
waveform->setFrequencyOffset(30.0f);
waveform->reset();  // Oops, CFO is now 0
waveform->process(samples);

// CORRECT - reset THEN set CFO
waveform->reset();
waveform->setFrequencyOffset(30.0f);
waveform->process(samples);
```

### Mistake 4: Re-estimating CFO from Training
```cpp
// WRONG - training CFO estimation is buggy and less accurate
float cfo = estimateCFOFromTraining(samples);  // Overrides chirp CFO!

// CORRECT - trust chirp CFO, skip training estimation
if (!chirp_cfo_estimated) {
    float cfo = estimateCFOFromTraining(samples);
}
```

---

## Test Commands

```bash
# Test OFDM_CHIRP with CFO (should be 100% at 20 dB)
./test_hf_modem -w chirp --snr 20 --cfo 30 --frames 10

# Test with ±50 Hz CFO (edge of tolerance)
./test_hf_modem -w chirp --snr 20 --cfo 50 --frames 10
./test_hf_modem -w chirp --snr 20 --cfo -50 --frames 10

# Test MC-DPSK with CFO
./test_hf_modem -w dpsk --snr 5 --cfo 30 --frames 10
```

---

## Files Reference

| File | Purpose |
|------|---------|
| `src/sync/chirp_sync.hpp` | Chirp generation and CFO-tolerant detection |
| `src/ofdm/modulator.cpp` | OFDM TX with CFO offset in mixer |
| `src/ofdm/demodulator.cpp` | `setFrequencyOffset()`, `processPresynced()` |
| `src/ofdm/channel_equalizer.cpp` | `toBaseband()` - actual CFO correction |
| `src/ofdm/demodulator_impl.hpp` | State: `freq_offset_hz`, `chirp_cfo_estimated` |
| `src/gui/modem/modem_rx_decode.cpp` | ModemEngine RX path calling chirp detection |

---

*Document created: 2026-01-26*
*Based on working test: 100% decode at SNR=20dB, CFO=±50Hz*
