# ModemEngine Architecture Analysis

**Date:** 2026-01-27
**Purpose:** Detailed documentation of ModemEngine internals for future refactoring
**Status:** Current implementation with partial IWaveform/RxPipeline integration

---

## Executive Summary

ModemEngine is the central class handling TX/RX audio processing. It currently has **two parallel code paths**:

1. **OLD PATH:** Direct modulator/demodulator usage with per-waveform `processRxBuffer_*()` methods
2. **NEW PATH:** IWaveform + RxPipeline abstraction (partially integrated, has issues)

The old path is **production-ready** and handles all waveforms correctly. The new path was introduced for cleaner architecture but has bugs (WIP commits show debugging issues with chirp detection in RxPipeline).

---

## File Structure

```
src/gui/modem/
├── modem_engine.hpp          # Main class definition (~355 lines)
├── modem_engine.cpp          # Constructor, TX, test signals (~934 lines)
├── modem_rx.cpp              # Thread management, feedAudio, buffer helpers (~370 lines)
├── modem_rx_decode.cpp       # Decode logic for all waveforms (~1321 lines)
├── modem_mode.cpp            # Mode switching, waveform control (~319 lines)
├── modem_rx_constants.hpp    # RX timing constants
├── modem_types.hpp           # DetectedFrame, FrameQueue, etc.
├── rx_pipeline.hpp           # NEW: IWaveform-based RX pipeline (~199 lines)
└── rx_pipeline.cpp           # NEW: RxPipeline implementation
```

---

## Thread Model

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                            AUDIO THREAD                                      │
│  feedAudio() → writes to rx_sample_buffer_ (disconnected)                   │
│             → routes to rx_pipeline_ (connected, NEW but buggy)             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                       ACQUISITION THREAD (disconnected only)                 │
│                                                                              │
│  acquisitionLoop():                                                          │
│    - Polls every 100ms                                                       │
│    - Skips if connected_ == true                                             │
│    - Searches for chirp preamble in rx_sample_buffer_                        │
│    - On chirp found:                                                         │
│      a) Check for data after chirp → DetectedFrame to frame queue           │
│      b) No data → PING callback                                              │
│                                                                              │
│  Key state:                                                                  │
│    - connected_: false                                                       │
│    - rx_frame_state_.active: false                                           │
│    - detected_frame_queue_.size(): 0                                         │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        RX DECODE THREAD                                      │
│                                                                              │
│  rxDecodeLoop():                                                             │
│    IF connected_:                                                            │
│      → NEW PATH: Poll rx_pipeline_ for decoded frames (BUGGY)               │
│      → OLD PATH: Not used when connected (was processRxBuffer_*)            │
│                                                                              │
│    IF NOT connected_:                                                        │
│      → Wait for DetectedFrame from acquisition thread                        │
│      → Call rxDecodeDPSK(frame) for MC-DPSK frames                          │
│      → OFDM/OTFS frames go to processRxBuffer_* (not used in disconnected)  │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Thread Synchronization

| Resource | Mutex | Writers | Readers |
|----------|-------|---------|---------|
| `rx_sample_buffer_` | `rx_buffer_mutex_` | feedAudio, consumeSamples | getBufferSnapshot, getBufferSize |
| `detected_frame_queue_` | Internal | acquisitionLoop | rxDecodeLoop |
| `rx_data_queue_` | `rx_mutex_` | deliverFrame | getReceivedData |
| `stats_` | `stats_mutex_` | updateStats | getStats |

---

## State Machine

### Connection State Variables

```cpp
// Primary state
bool connected_ = false;           // True after CONNECT_ACK received
bool handshake_complete_ = false;  // True after first post-ACK frame

// Waveform selection (complex logic!)
protocol::WaveformMode waveform_mode_;        // Target mode (after negotiation)
protocol::WaveformMode connect_waveform_;     // Mode for CONNECT frames
protocol::WaveformMode last_rx_waveform_;     // Last received frame's waveform
protocol::WaveformMode disconnect_waveform_; // Saved for ACK after disconnect

// Special flag
bool use_connected_waveform_once_ = false;   // For DISCONNECT ACK only
```

### TX Waveform Selection Logic (transmit())

```cpp
// Priority order for selecting active_waveform:
if (use_connected_waveform_once_) {
    active_waveform = disconnect_waveform_;  // DISCONNECT ACK
    use_connected_waveform_once_ = false;
} else if (!connected_) {
    active_waveform = connect_waveform_;     // CONNECT, PING
} else if (!handshake_complete_) {
    active_waveform = last_rx_waveform_;     // Echo back received mode
} else {
    active_waveform = waveform_mode_;        // Normal connected mode
}
```

### RX Routing by State

| State | Audio Route | Decode Method |
|-------|-------------|---------------|
| Disconnected | `rx_sample_buffer_` | `acquisitionLoop()` → `rxDecodeDPSK()` |
| Connected (OLD) | `rx_sample_buffer_` | `processRxBuffer_*()` (NOT USED NOW) |
| Connected (NEW) | `rx_pipeline_` | `rx_pipeline_->feedAudio()` + callbacks |

---

## TX Path Details

### transmit(const Bytes& data)

**Location:** `modem_engine.cpp:233-587`

```
Input: Bytes (raw data or v2 frame)
                │
                ▼
┌───────────────────────────────────────┐
│ 1. Detect v2 frame (magic "UL")       │
│    is_v2_frame = data[0:2] == 0x554C  │
└───────────────────────────────────────┘
                │
                ▼
┌───────────────────────────────────────┐
│ 2. LDPC Encoding                      │
│                                       │
│    IF v2 frame:                       │
│      - DATA frame: CW0=R1/4, CW1+=    │
│        data_code_rate_                │
│      - Control frame: all R1/4        │
│                                       │
│    ELSE (raw):                        │
│      - Use current config rate        │
└───────────────────────────────────────┘
                │
                ▼
┌───────────────────────────────────────┐
│ 3. Channel Interleaving (OFDM only)   │
│    IF OFDM_COX or OFDM_CHIRP:         │
│      Apply per-codeword interleaving  │
└───────────────────────────────────────┘
                │
                ▼
┌───────────────────────────────────────┐
│ 4. Waveform-Specific Modulation       │
│                                       │
│    MC-DPSK:                           │
│      chirp + training + ref + data    │
│                                       │
│    OTFS:                              │
│      per-codeword OTFS frames         │
│                                       │
│    OFDM_CHIRP:                        │
│      chirp + 2 LTS + DQPSK data       │
│                                       │
│    OFDM_COX:                          │
│      Schmidl-Cox preamble + data      │
└───────────────────────────────────────┘
                │
                ▼
┌───────────────────────────────────────┐
│ 5. Output Assembly                    │
│    150ms lead-in + preamble + data    │
│    + tail guard                       │
│                                       │
│ 6. TX Filter (optional)               │
│ 7. Amplitude scaling (0.8 max)        │
└───────────────────────────────────────┘
                │
                ▼
Output: std::vector<float> audio samples
```

---

## RX Path Details (Old Path - Production)

### 1. Disconnected Mode: acquisitionLoop()

**Location:** `modem_rx.cpp:38-146`

```cpp
// Search for chirp every 100ms
while (acquisition_running_) {
    if (connected_) continue;  // Skip when connected

    samples = getBufferSnapshot();
    if (samples.size() < MIN_SAMPLES_FOR_ACQUISITION) continue;

    // Dual chirp detection (CFO-tolerant)
    chirp_result = chirp_sync_->detectDualChirp(search_span, 0.15f);

    if (chirp_result.success) {
        // Check for data after chirp
        if (has_data_after) {
            // MC-DPSK frame → push to queue
            DetectedFrame frame;
            frame.data_start = chirp_end + training + ref;
            frame.waveform = WaveformMode::MC_DPSK;
            frame.cfo_hz = chirp_result.cfo_hz;
            detected_frame_queue_.push(frame);
        } else {
            // PING (chirp only)
            consumeSamples(chirp_end);
            ping_received_callback_(getCurrentSNR());
        }
    }
}
```

### 2. Disconnected Decode: rxDecodeDPSK()

**Location:** `modem_rx_decode.cpp:160-448`

This is the **most complex decode function**. It handles MC-DPSK frames with chirp preamble.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           rxDecodeDPSK(frame)                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Step 1: Wait for PING samples                                              │
│    waitForSamples(frame.data_start + samples_for_ping)                      │
│                                                                              │
│  Step 2: Setup demodulator with CFO correction                              │
│    IF frame.has_chirp_preamble:                                             │
│      - Set CFO from dual chirp estimate                                      │
│      - Apply CFO to training/ref samples BEFORE processTraining()           │
│      - mc_dpsk_demodulator_->processTraining(corrected_training)            │
│      - mc_dpsk_demodulator_->setReference(corrected_ref)                    │
│    ELSE:                                                                    │
│      - Legacy Barker-13: just setReference()                                │
│                                                                              │
│  Step 3: Check for PING                                                     │
│    Apply CFO to ping samples                                                │
│    soft_bits = demodulateSoft(ping_samples)                                 │
│    IF detectPing(soft_bits) → return success                                │
│                                                                              │
│  Step 4: Decode CW0 to get codeword count                                   │
│    waitForSamples(frame.data_start + samples_per_codeword)                  │
│    Re-setup demodulator (same CFO logic)                                    │
│    Apply CFO to CW0 samples                                                 │
│    soft_bits = demodulateSoft(cw0_samples)                                  │
│    cw0_data = LDPC_decode(soft_bits, R1/4)                                  │
│    expected_codewords = parseHeader(cw0_data).total_cw                      │
│                                                                              │
│  Step 5: Wait for all codewords and decode                                  │
│    waitForSamples(frame.data_start + total_samples)                         │
│    Re-setup demodulator (third time!)                                       │
│    Apply CFO to all data samples                                            │
│    soft_bits = demodulateSoft(all_data)                                     │
│    result = decodeCodewords(soft_bits, expected_codewords, ...)             │
│                                                                              │
│  Step 6: Deliver frame                                                      │
│    deliverFrame(result.frame_data)                                          │
│    Save peer_cfo_hz_ for connected mode                                     │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

**CFO Correction Flow (Critical for Fading Channels):**

```cpp
// 1. Get CFO from dual chirp detection (in acquisition thread)
frame.cfo_hz = chirp_result.cfo_hz;

// 2. Set CFO on demodulator (tells it what to correct)
mc_dpsk_demodulator_->setCFO(frame.cfo_hz);

// 3. Apply CFO to training/ref BEFORE processTraining
//    This is CRITICAL - otherwise processTraining estimates wrong residual
Samples training_corrected = ...;
mc_dpsk_demodulator_->applyCFO(training_corrected);

// 4. Apply CFO to data samples BEFORE demodulation
Samples data_corrected = ...;
mc_dpsk_demodulator_->applyCFO(data_corrected);
soft_bits = mc_dpsk_demodulator_->demodulateSoft(data_corrected);
```

### 3. Connected Mode (OLD Path): processRxBuffer_*

**NOT CURRENTLY USED** - The new RxPipeline is supposed to replace these, but it has bugs.

#### processRxBuffer_OFDM()
**Location:** `modem_rx_decode.cpp:454-587`

Standard OFDM with Schmidl-Cox preamble:
1. Feed samples to `ofdm_demodulator_->process()`
2. Accumulate soft bits when `frame_ready`
3. Probe CW0 to get expected codeword count
4. Wait for all codewords
5. Decode and deliver

#### processRxBuffer_OFDM_CHIRP()
**Location:** `modem_rx_decode.cpp:940-1218`

Chirp sync + OFDM DQPSK:
1. Search for dual chirp (if not already found)
2. Apply CFO from chirp to demodulator
3. Use `processPresynced()` (bypasses Schmidl-Cox)
4. Accumulate soft bits
5. Decode (same as OFDM)

#### processRxBuffer_DPSK()
**Location:** `modem_rx_decode.cpp:792-938`

Connected mode MC-DPSK (streaming):
1. Feed samples to demodulator
2. Demodulator handles chirp detection internally
3. Accumulate soft bits when ready
4. Decode when all codewords received

#### processRxBuffer_OTFS()
**Location:** `modem_rx_decode.cpp:596-785`

OTFS frames (1 frame = 1 codeword):
1. Feed samples to OTFS demodulator
2. Loop to extract all ready frames
3. Accumulate codewords
4. Decode when complete

---

## NEW Path: IWaveform + RxPipeline (WIP/BUGGY)

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        ModemEngine (connected mode)                          │
│                                                                              │
│  feedAudio() ─────────────────────────────────────────────┐                 │
│                                                           ▼                 │
│  rx_waveform_ofdm_       ┐                          rx_pipeline_             │
│  rx_waveform_ofdm_chirp_ │ IWaveform instances           │                   │
│  rx_waveform_mc_dpsk_    ┘                               │                   │
│           ▲                                              │                   │
│           │                                              │                   │
│  active_rx_waveform_ ◄─────────────── setWaveform() ◄────┘                  │
│           │                                                                  │
│           └──────────────────────────────────────────────────────────────────│
│                                                                              │
│  rxDecodeLoop() polls rx_pipeline_->hasFrame()                               │
│    → result = rx_pipeline_->getFrame()                                       │
│    → update stats, deliver via callbacks                                     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### RxPipeline Flow (Intended)

```cpp
// In feedAudio() when connected:
rx_pipeline_->feedAudio(samples, count);

// RxPipeline internally:
void RxPipeline::feedAudio(samples, count) {
    rx_buffer_.append(samples);

    while (rx_buffer_.size() >= MIN_SAMPLES_FOR_SYNC) {
        if (!tryProcessBuffer()) break;
    }
}

bool RxPipeline::tryProcessBuffer() {
    SyncResult sync;
    if (!waveform_->detectSync(rx_buffer_, sync)) {
        // Trim old samples
        return false;
    }

    waveform_->setFrequencyOffset(sync.cfo_hz);

    SampleSpan data_span(rx_buffer_ + sync.start_sample, ...);
    if (waveform_->process(data_span)) {
        auto soft_bits = waveform_->getSoftBits();
        auto result = decodeFrame(soft_bits, ...);
        frame_queue_.push(result);
    }

    return true;
}
```

### Known Issues (WIP Commits)

1. **Chirp detection fails in RxPipeline** - Works in `test_iwaveform.cpp` directly but not through RxPipeline
2. **OFDMChirpWaveform::process()** - Was only returning 648 bits instead of all soft bits (FIXED in recent session)
3. **Buffer management** - Unclear when to consume samples vs keep for retry

---

## Waveform Selection Summary

### Disconnected Mode RX

Only **MC-DPSK with chirp preamble** is detected by acquisition thread. All incoming frames must use chirp sync.

### Connected Mode RX

Depends on `waveform_mode_`:

| Mode | Sync Method | Demodulator | Code Path |
|------|-------------|-------------|-----------|
| OFDM_COX | Schmidl-Cox | OFDMDemodulator | processRxBuffer_OFDM |
| OFDM_CHIRP | Dual Chirp | OFDMDemodulator (presynced) | processRxBuffer_OFDM_CHIRP |
| MC_DPSK | Dual Chirp | MultiCarrierDPSKDemodulator | processRxBuffer_DPSK |
| OTFS_* | Zadoff-Chu | OTFSDemodulator | processRxBuffer_OTFS |

---

## Key Insights for Refactoring

### 1. Why RxPipeline Failed

The old `processRxBuffer_*` methods have **waveform-specific buffer management** that's hard to generalize:

- **OFDM_CHIRP:** Need to find chirp first, then process samples starting from chirp end
- **MC-DPSK:** Demodulator handles chirp detection internally with `process()`
- **OFDM_COX:** Schmidl-Cox sync is inside demodulator

RxPipeline tried to use `IWaveform::detectSync()` → `setFrequencyOffset()` → `process()` uniformly, but:

- Different waveforms expect different sample spans
- CFO correction timing differs (before processTraining vs inside process)
- Some waveforms consume samples during detection, others don't

### 2. CFO Correction is Critical

The MC-DPSK CFO fix required:
1. Getting CFO from dual chirp detection
2. Setting CFO on demodulator
3. **Applying CFO to training/ref samples BEFORE processTraining()** (not after!)
4. Applying CFO to data samples before demodulation

This pattern is **NOT captured in IWaveform interface** - it just has `setFrequencyOffset()` which each waveform interprets differently.

### 3. The Demodulator Setup Pattern

All decode paths do this pattern **multiple times** (once for ping check, once for CW0, once for full frame):

```cpp
// 1. Calculate offsets
training_offset = ref_offset - training_samples;
ref_offset = data_start - symbol_samples;

// 2. Apply CFO to training/ref (if chirp frame)
if (has_chirp_preamble && abs(cfo_hz) > 0.1f) {
    training_corrected = buffer[training_offset:training_offset+training_len];
    ref_corrected = buffer[ref_offset:ref_offset+symbol_len];
    demodulator->applyCFO(training_corrected);
    demodulator->applyCFO(ref_corrected);
}

// 3. Process training and set reference
demodulator->processTraining(training_corrected);
demodulator->setReference(ref_corrected);

// 4. Apply CFO to data and demodulate
data_corrected = buffer[data_start:data_start+data_len];
demodulator->applyCFO(data_corrected);
soft_bits = demodulator->demodulateSoft(data_corrected);
```

### 4. What a Clean Refactor Would Need

```cpp
class IWaveform {
    // Current interface
    bool detectSync(samples, result);
    void setFrequencyOffset(cfo_hz);
    bool process(samples);
    vector<float> getSoftBits();

    // MISSING: Unified frame processing
    // Instead of the caller doing CFO correction manually,
    // the waveform should handle everything:

    RxResult processFrame(SampleSpan samples, int training_symbols = 2) {
        // 1. Detect sync
        // 2. Apply CFO internally
        // 3. Process training
        // 4. Demodulate data
        // 5. Return soft bits + SNR + CFO
    }
};
```

But this would require each waveform to know about training symbols, reference symbols, etc., which is already embedded in the demodulators.

---

## Recommendations

### Short Term (Current State)

1. **Keep using old `processRxBuffer_*` methods** - They work and have correct CFO handling
2. **Use `test_iwaveform.cpp` for IWaveform testing** - Bypasses ModemEngine complexity
3. **Don't enable RxPipeline in connected mode** - It has bugs

### Medium Term

1. **Document each waveform's expected input format** (where does data start, training length, etc.)
2. **Add `processFrame()` method to IWaveform** that handles full decode internally
3. **Move CFO correction inside waveform implementations** instead of caller

### Long Term (Full Refactor)

1. **ModemEngine becomes thin orchestrator** - Just routes audio to/from waveforms
2. **Each waveform is self-contained** - Handles sync, CFO, demod, LDPC internally
3. **WaveformState class** - Manages the 7 mode variables cleanly
4. **Remove legacy modulator/demodulator pointers** from ModemEngine

---

## Appendix: Member Variable Reference

### Modulators/Demodulators (Legacy)

```cpp
// OFDM (Schmidl-Cox)
std::unique_ptr<OFDMModulator> ofdm_modulator_;
std::unique_ptr<OFDMDemodulator> ofdm_demodulator_;

// OTFS
std::unique_ptr<OTFSModulator> otfs_modulator_;
std::unique_ptr<OTFSDemodulator> otfs_demodulator_;

// Single-carrier DPSK (unused?)
std::unique_ptr<DPSKModulator> dpsk_modulator_;
std::unique_ptr<DPSKDemodulator> dpsk_demodulator_;

// Multi-carrier DPSK
std::unique_ptr<MultiCarrierDPSKModulator> mc_dpsk_modulator_;
std::unique_ptr<MultiCarrierDPSKDemodulator> mc_dpsk_demodulator_;

// Chirp sync (shared by MC-DPSK and OFDM_CHIRP)
std::unique_ptr<sync::ChirpSync> chirp_sync_;
```

### IWaveform (New)

```cpp
// TX waveform (not fully used)
std::unique_ptr<IWaveform> active_tx_waveform_;

// RX waveforms (one per mode)
std::unique_ptr<IWaveform> rx_waveform_ofdm_;
std::unique_ptr<IWaveform> rx_waveform_ofdm_chirp_;
std::unique_ptr<IWaveform> rx_waveform_mc_dpsk_;

// Active waveform for connected mode
IWaveform* active_rx_waveform_ = nullptr;

// RxPipeline
std::unique_ptr<RxPipeline> rx_pipeline_;
```

### Accumulation State

```cpp
// OFDM accumulation
std::vector<float> ofdm_accumulated_soft_bits_;
int ofdm_expected_codewords_ = 0;
bool ofdm_chirp_found_ = false;

// DPSK accumulation
std::vector<float> dpsk_accumulated_soft_bits_;
int dpsk_expected_codewords_ = 0;

// OTFS accumulation
std::vector<float> otfs_accumulated_soft_bits_;
int otfs_expected_codewords_ = 0;
```

---

## Appendix: Constants

```cpp
// From modem_rx_constants.hpp
namespace rx_constants {

// Buffer thresholds
constexpr size_t MIN_SAMPLES_FOR_OFDM_SYNC = 8000;    // ~167ms @ 48kHz
constexpr size_t MIN_SAMPLES_FOR_DPSK = 4000;         // ~83ms @ 48kHz
constexpr size_t MIN_SAMPLES_FOR_ACQUISITION = 10000; // ~208ms @ 48kHz
constexpr size_t MAX_BUFFER_BEFORE_TRIM = 48000;      // 1 second @ 48kHz
constexpr size_t BUFFER_TRIM_TARGET = 24000;          // Keep 500ms after trim

// Thread timing
constexpr int ACQUISITION_POLL_MS = 50;
constexpr int DECODE_POLL_MS = 10;
constexpr int WAIT_FOR_SAMPLES_MS = 10;

// Audio injection
constexpr size_t INJECT_CHUNK_SIZE = 4800;            // 100ms chunks
constexpr int INJECT_CHUNK_DELAY_MS = 10;
constexpr int INJECT_FINAL_DELAY_MS = 500;

// Carrier sense
constexpr size_t ENERGY_WINDOW_SAMPLES = 480;         // 10ms @ 48kHz

}

// From modem_engine.hpp
constexpr size_t MAX_PENDING_SAMPLES = 960000;        // 20 seconds @ 48kHz
```
