# Testing Methodology Analysis

**Date:** 2026-01-27
**Purpose:** Comprehensive documentation of modem testing approach for future refactoring

---

## Executive Summary

### Test Tool Strategy

**PRIMARY tools (keep and improve):**

| Tool | Purpose | CFO Method | Status |
|------|---------|------------|--------|
| `test_iwaveform` | IWaveform interface testing | Hilbert transform (correct) | ✅ WORKING |
| `cli_simulator` | Full protocol (2 stations) | tx_cfo_hz (BROKEN - BUG-001) | ⚠️ NEEDS FIX |

**LEGACY tools (to be removed):**

| Tool | Purpose | Notes |
|------|---------|-------|
| `test_hf_modem` | Old RxPipeline testing | Reference only, will be deprecated |

### Future Plan

1. **test_iwaveform** → Rename to `test_modem` (or similar)
   - Will become THE official modem test tool
   - Already has correct CFO simulation
   - Tests all waveforms via IWaveform interface

2. **cli_simulator** → Fix BUG-001, keep for protocol testing
   - Full PING → CONNECT → DATA → DISCONNECT flow
   - Two-station simulation
   - Needs Hilbert-based CFO in applyChannel()

3. **test_hf_modem** → Remove after refactor complete
   - Legacy approach, kept for reference only
   - Will be deleted once test_iwaveform covers all cases

**Critical requirement:** Testing must simulate real HF rig audio - continuous streaming, proper CFO simulation, and no "cheating" by knowing frame positions.

---

## Part 1: Signal Generation Chain

### 1.1 Frame Creation

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ v2::ConnectFrame::makeConnect("SRC", "DST", capabilities, waveform_mode)    │
│                                                                              │
│ Serializes to:                                                               │
│ [MAGIC 0x55 0x4C][TYPE][LEN][SEQ][SRC callsign][DST callsign][CAPS][CRC]   │
│                                                                              │
│ Size: ~20-40 bytes depending on callsign length                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 LDPC Encoding

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ LDPCEncoder(CodeRate rate)                                                   │
│                                                                              │
│ Input:  Frame bytes (padded to 81 bytes per codeword)                        │
│ Output: 648-bit codewords as packed bytes                                    │
│                                                                              │
│ Code rates: R1/4, R1/3, R1/2, R2/3, R3/4, R5/6                              │
│   R1/4: 81 bytes input → 648 bits output (lowest, most robust)              │
│   R3/4: 81 bytes input → 648 bits output (highest, fastest)                 │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.3 Channel Interleaving (Optional)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ ChannelInterleaver(bits_per_symbol)                                          │
│                                                                              │
│ Purpose: Spread burst errors across multiple codewords                       │
│ Method:  Transpose bit matrix (writes columns, reads rows)                   │
│                                                                              │
│ bits_per_symbol = num_carriers × bits_per_carrier                            │
│   OFDM_CHIRP: 30 carriers × 2 bits = 60                                     │
│   MC-DPSK:    8 carriers × 2 bits = 16                                      │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.4 TX Modulation (via IWaveform)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ IWaveform::generatePreamble() + IWaveform::modulate(encoded_bytes)          │
│                                                                              │
│ OFDM_CHIRP:                                                                  │
│   Preamble: [DUAL_CHIRP ~1.2s][2× OFDM Training Symbols]                    │
│   Data:     [OFDM DQPSK symbols, 30 carriers, 86 baud]                      │
│   Total:    ~2-4 seconds depending on payload                                │
│                                                                              │
│ MC-DPSK:                                                                     │
│   Preamble: [DUAL_CHIRP ~1.2s][TRAINING 8 sym][REF 1 sym]                   │
│   Data:     [Multi-carrier DQPSK symbols, 8 carriers, 94 baud]              │
│   Total:    ~3-5 seconds depending on payload                                │
│                                                                              │
│ OFDM_COX (Schmidl-Cox):                                                      │
│   Preamble: [STS 2 symbols][LTS 2 symbols]                                   │
│   Data:     [OFDM symbols, configurable modulation]                          │
│   Total:    ~1-2 seconds                                                     │
│                                                                              │
│ Output: Real-valued samples at 48 kHz, 300-2700 Hz band, center 1500 Hz     │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Part 2: CFO Simulation (Critical for Realism)

### 2.1 Why CFO Matters

Real HF radios have tuning errors of **±10-50 Hz** due to:
- Crystal oscillator aging
- Temperature drift
- Manual tuning imprecision
- Doppler from ionospheric motion

If we don't test with CFO, we're not testing real conditions.

### 2.2 CORRECT Method: Hilbert Transform

**Location:** `test_hf_modem.cpp:185-210`, `test_iwaveform.cpp:64-97`

```cpp
void applyCFO_TimeDomain(std::vector<float>& samples, float cfo_hz) {
    // 1. Create Hilbert transform to get analytic signal
    HilbertTransform hilbert(255);  // 255-tap FIR filter

    // 2. Process in chunks
    for (size_t offset = 0; offset < N; offset += chunk_size) {
        SampleSpan span(samples.data() + offset, len);
        auto analytic = hilbert.process(span);  // Returns I + jQ

        // 3. Multiply by complex exponential (frequency shift)
        for (size_t i = 0; i < len; i++) {
            Complex rot(std::cos(phase), std::sin(phase));
            samples[offset + i] = std::real(analytic[i] * rot);
            phase += phase_inc;
        }
    }
}
```

**Why Hilbert transform is required:**

Simple multiplication with `cos(2π·CFO·t)` creates IMAGE frequencies:
```
x(t) × cos(2π·f_cfo·t) = 0.5·x(t)·e^{j2πf_cfo·t} + 0.5·x(t)·e^{-j2πf_cfo·t}
                          ↑ wanted                    ↑ unwanted image!
```

Hilbert transform creates analytic signal (single-sideband):
```
x_a(t) = x(t) + j·H{x(t)}    (analytic signal, only positive frequencies)
x_a(t) × e^{j2πf_cfo·t}      (clean frequency shift, no image)
output = Re{x_a(t) × e^{j2πf_cfo·t}}
```

### 2.3 INCORRECT Method: Modem tx_cfo_hz

**Location:** `cli_simulator.cpp:243-249`

```cpp
// This only shifts the CHIRP frequencies, not the entire signal!
auto cfg = modem_.getConfig();
cfg.tx_cfo_hz = cfo_hz;
modem_.setConfig(cfg);
```

**Problem:** This approach only affects chirp generation, not the data symbols. The chirp shifts but the OFDM/DPSK data doesn't, causing inconsistent CFO across the frame.

### 2.4 Order of Operations (CRITICAL)

```
1. Generate clean TX signal (frames placed in audio buffer)
2. Apply CFO via Hilbert transform to ENTIRE signal
3. Apply HF fading channel (if enabled)
4. Add AWGN noise

Order matters! CFO must be applied BEFORE fading and noise.
```

---

## Part 3: Channel Simulation

### 3.1 AWGN (Baseline)

**Location:** `test_hf_modem.cpp:162-179`

```cpp
void addNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
    // Calculate signal power from ACTIVE samples only (excludes silence)
    float signal_power = 0.0f;
    size_t active_samples = 0;
    for (float s : samples) {
        if (std::abs(s) > 1e-6f) {
            signal_power += s * s;
            active_samples++;
        }
    }
    signal_power /= active_samples;

    // Calculate noise power from SNR
    float noise_power = signal_power / pow(10, snr_db / 10);
    float noise_std = sqrt(noise_power);

    // Add Gaussian noise to ALL samples
    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : samples) {
        s += noise(rng);
    }
}
```

**Key insight:** SNR is calibrated from **active signal power**, not total audio power (which would include silence gaps).

### 3.2 Watterson HF Fading (ITU-R F.1487)

**Location:** `src/sim/hf_channel.hpp`

```cpp
// Two-tap Rayleigh fading model:
//
//   y(t) = h1(t)·x(t) + h2(t)·x(t-τ)
//
// Where:
//   h1, h2 = complex Rayleigh fading coefficients (IIR filtered Gaussian)
//   τ = delay spread (0.5-2.0 ms)
//   Doppler = fading rate (0.1-10 Hz)

struct Config {
    float snr_db;
    float delay_spread_ms;      // Second path delay
    float doppler_spread_hz;    // Fading rate
    float path1_gain;           // Direct path
    float path2_gain;           // Delayed path
};
```

**Standard channel conditions (ITU-R F.1487):**

| Condition | Delay (ms) | Doppler (Hz) | Use Case |
|-----------|------------|--------------|----------|
| AWGN | 0 | 0 | Baseline, no fading |
| Good | 0.5 | 0.1 | Quiet mid-latitude |
| Moderate | 1.0 | 0.5 | Typical conditions |
| Poor | 2.0 | 1.0 | Disturbed ionosphere |
| Flutter | 0.5 | 10.0 | Auroral/polar paths |

---

## Part 4: RX Processing (No Cheating!)

### 4.1 Real Receiver Simulation

A real HF receiver doesn't know:
- When frames start
- How many frames are in the audio
- What the CFO is

**Correct approach** (test_hf_modem.cpp):

```cpp
// Create ONE RX pipeline for the ENTIRE audio stream
RxPipeline rx_pipeline("RX");
rx_pipeline.setWaveform(rx_waveform.get());

// Feed ENTIRE audio stream in chunks (like a real receiver)
size_t chunk_size = 960;  // 20ms chunks (typical audio callback)
for (size_t j = 0; j < full_audio.size(); j += chunk_size) {
    size_t len = std::min(chunk_size, full_audio.size() - j);
    rx_pipeline.feedAudio(full_audio.data() + j, len);

    // Poll for decoded frames
    while (rx_pipeline.hasFrame()) {
        auto result = rx_pipeline.getFrame();
        // Process decoded frame...
    }
}
```

**INCORRECT approach** (would be cheating):

```cpp
// DON'T DO THIS - it knows frame positions!
for (int i = 0; i < num_frames; i++) {
    // Extract exact frame region (cheating!)
    SampleSpan frame_audio(audio + frames[i].start, frames[i].len);
    decode(frame_audio);  // Too easy - we know exactly where it is
}
```

### 4.2 RxPipeline Operation

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ RxPipeline.feedAudio(samples)                                                │
│                                                                              │
│ 1. Accumulate samples in internal buffer                                     │
│ 2. When buffer >= MIN_SAMPLES_FOR_SYNC:                                      │
│    a. Call IWaveform::detectSync() on buffer                                │
│    b. If sync found:                                                         │
│       - Get CFO from SyncResult                                              │
│       - Call IWaveform::setFrequencyOffset(cfo)                             │
│       - Call IWaveform::process(samples from start_sample)                  │
│       - Get soft bits, LDPC decode, parse frame                             │
│       - Queue result for getFrame()                                          │
│       - Consume processed samples from buffer                                │
│    c. If no sync: trim old samples from buffer                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Part 5: Test Tools Comparison

### 5.1 test_hf_modem.cpp (IWaveform + RxPipeline)

**Purpose:** Test the new IWaveform + RxPipeline architecture directly.

**Signal flow:**
```
TX: Frame → LDPC → Interleave → IWaveform.modulate()
    ↓
Channel: CFO (Hilbert) → Fading → AWGN
    ↓
RX: RxPipeline.feedAudio() → detectSync → process → getSoftBits → LDPC
```

**Command line:**
```bash
./test_hf_modem --snr 20 -w chirp --channel moderate --cfo 30 --frames 10
```

**Pros:**
- Tests IWaveform interface directly
- Single continuous audio stream (realistic)
- Correct CFO simulation via Hilbert transform

**Cons:**
- RxPipeline has bugs (may not work for all waveforms)
- Doesn't test ModemEngine integration

### 5.2 test_iwaveform.cpp (IWaveform via ModemEngine)

**Purpose:** Test IWaveform with ModemEngine's TX, but direct IWaveform decode for RX.

**Signal flow:**
```
TX: Frame → ModemEngine.transmit() [uses IWaveform internally]
    ↓
Channel: CFO (Hilbert) → Fading → AWGN
    ↓
RX (MC-DPSK): ModemEngine.feedAudio() → acquisition thread → decode
RX (OFDM_CHIRP): IWaveform.detectSync() → process() → getSoftBits() → LDPC
```

**Command line:**
```bash
./test_iwaveform --snr 15 --cfo 30 --channel moderate -w mc_dpsk
./test_iwaveform --snr 17 --cfo 30 --channel awgn -w ofdm_chirp
```

**Pros:**
- Tests ModemEngine TX path
- Correct CFO simulation via Hilbert transform
- OFDM_CHIRP uses IWaveform directly (bypasses ModemEngine RX bugs)

**Cons:**
- OFDM_CHIRP decode path is separate from MC-DPSK
- Doesn't test full protocol

### 5.3 cli_simulator.cpp (Full Protocol)

**Purpose:** Test complete protocol flow with two simulated stations.

**Signal flow:**
```
ALPHA: protocol_.connect() → modem_.transmit() → channel_to_virtual_
                                                        ↓
                                              applyChannel() (fading + noise)
                                                        ↓
BRAVO: virtual_modem_->feedAudio() ← channel_to_virtual_
       virtual_protocol_.onRxData()
                ↓
       virtual_modem_->transmit() → channel_to_us_
                                         ↓
                                applyChannel()
                                         ↓
ALPHA: modem_.feedAudio() ← channel_to_us_
       protocol_.onRxData()
```

**Command line:**
```bash
./cli_simulator --snr 20 --channel moderate --cfo 30 --force-waveform DPSK
```

**Pros:**
- Tests full protocol (PING → CONNECT → DATA → DISCONNECT)
- Tests ARQ, mode negotiation, handshake
- Two independent ModemEngine instances

**Cons:**
- CFO via modem tx_cfo_hz is INCORRECT (only affects chirp, not data)
- ModemEngine RX path has issues with OFDM_CHIRP
- Complex state machine makes debugging hard

---

## Part 6: Requirements for Refactoring

### 6.1 CFO Simulation Must Be Correct

**MUST DO:**
- Apply CFO via Hilbert transform to the ENTIRE signal AFTER generation
- Apply CFO BEFORE channel fading and noise
- Use proper analytic signal (single-sideband shift)

**MUST NOT DO:**
- Apply CFO only to chirp via tx_cfo_hz (incomplete)
- Apply CFO via simple multiplication (creates images)
- Apply CFO after noise (wrong order)

### 6.2 Audio Streaming Must Be Realistic

**MUST DO:**
- Feed audio in small chunks (10-20ms, like real audio callbacks)
- Use a SINGLE receiver for entire audio stream
- Let receiver detect sync autonomously

**MUST NOT DO:**
- Extract exact frame regions (cheating)
- Create new receiver per frame
- Tell receiver where frames are

### 6.3 SNR Calculation Must Be Accurate

**MUST DO:**
- Calculate signal power from ACTIVE samples only
- Exclude silence gaps from power calculation
- Apply noise to entire audio (including silence)

**MUST NOT DO:**
- Include silence in signal power (underestimates SNR)
- Apply different noise to different parts
- Use hardcoded power levels

### 6.4 Channel Model Must Be Standard

**MUST DO:**
- Use ITU-R F.1487 / Watterson model
- Two-tap Rayleigh fading
- Configurable delay spread and Doppler

**Options (nice to have):**
- CCIR presets (good/moderate/poor/flutter)
- Random seed for reproducibility
- CFO via channel (if properly implemented)

---

## Part 7: Unified Test Framework (Future)

### 7.1 Proposed Architecture

```cpp
class ModemTestHarness {
public:
    // Configuration
    void setWaveform(WaveformMode mode);
    void setModulation(Modulation mod);
    void setCodeRate(CodeRate rate);
    void setSNR(float snr_db);
    void setCFO(float cfo_hz);
    void setChannel(ChannelCondition cond);
    void setInterleavingEnabled(bool enabled);

    // TX - Generate frames into audio buffer
    void addFrame(const v2::Frame& frame);
    void addGap(float duration_sec);

    // Channel - Apply CFO, fading, noise
    void applyChannel();  // Must be called AFTER all TX, BEFORE RX

    // RX - Single streaming decode
    int runDecode();  // Returns number of frames decoded

    // Verification
    bool allFramesDecoded() const;
    std::vector<DecodeResult> getResults() const;
};

// Usage:
ModemTestHarness test;
test.setWaveform(WaveformMode::OFDM_CHIRP);
test.setSNR(15.0f);
test.setCFO(30.0f);
test.setChannel(ChannelCondition::Moderate);

for (int i = 0; i < 10; i++) {
    test.addFrame(makeConnectFrame(i));
    test.addGap(2.0f);
}

test.applyChannel();  // CFO + fading + noise
int decoded = test.runDecode();  // Single streaming RX

EXPECT_EQ(decoded, 10);
```

### 7.2 Key Invariants

1. **CFO is applied via Hilbert transform** - Not modem config, not simple multiplication
2. **Single RX instance for entire audio** - No per-frame decoders
3. **Chunk-based feeding** - 10-20ms chunks, like real audio
4. **Reproducible with seed** - Same seed = same results

---

## Part 8: Current Test Gaps

### 8.1 cli_simulator CFO is Wrong

**Problem:** Uses `modem_.getConfig().tx_cfo_hz` which only shifts chirp.

**Fix needed:** Apply CFO via Hilbert transform in `applyChannel()`:

```cpp
void applyChannel(std::vector<float>& samples) {
    // Apply CFO FIRST (before fading/noise)
    if (std::abs(cfo_hz_) > 0.1f) {
        applyCFO_Hilbert(samples, cfo_hz_);
    }

    // Then apply fading channel
    if (hf_channel_) {
        samples = hf_channel_->process(samples);
    } else {
        addAWGN(samples, snr_db_);
    }
}
```

### 8.2 ModemEngine OFDM_CHIRP RX is Broken

**Problem:** Acquisition thread routes all chirp frames to MC-DPSK decoder.

**Workaround in test_iwaveform.cpp:**
```cpp
if (waveform_mode == WaveformMode::OFDM_CHIRP) {
    // Use IWaveform directly instead of ModemEngine
    decodeOFDMChirpFrame(audio, expected_data, verbose);
} else {
    // Use ModemEngine for MC-DPSK
    rx_modem.feedAudio(samples);
}
```

### 8.3 RxPipeline Has Bugs

**Problem:** Chirp detection works in isolation but fails through RxPipeline.

**Status:** WIP commits show debugging in progress. For now, use IWaveform directly.

---

## Summary: What Must Be Preserved in Refactoring

| Requirement | Current Location | Notes |
|-------------|-----------------|-------|
| Hilbert CFO simulation | test_hf_modem.cpp:185-210 | MUST keep this method |
| AWGN from active samples | test_hf_modem.cpp:162-179 | Correct SNR calculation |
| ITU-R F.1487 channel | src/sim/hf_channel.hpp | Standard model |
| Single-stream RX | test_hf_modem.cpp:555-657 | No cheating |
| Chunk-based feeding | 960 samples = 20ms | Realistic audio |
| IWaveform interface | src/waveform/*.cpp | Clean abstraction |
| Frame sequence numbers | v2::ConnectFrame.seq | For result matching |
