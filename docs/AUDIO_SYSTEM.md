# Audio I/O System

**Purpose:** Document audio handling for debugging and future development.

---

## Overview

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48,000 Hz (fixed) |
| Channels | 1 (mono) |
| Format | 32-bit float |
| Buffer Size | 1024 samples (~21ms) |
| Backend | SDL2 |

---

## AudioEngine Class

**Location:** `src/gui/audio_engine.hpp/cpp`

### Key Methods

| Method | Purpose |
|--------|---------|
| `getOutputDevices()` | List available speakers/outputs |
| `getInputDevices()` | List available microphones/inputs |
| `openOutput(device)` | Open output device |
| `openInput(device)` | Open input device |
| `startPlayback()` | Begin TX streaming |
| `startCapture()` | Begin RX capture |
| `queueTxSamples(samples)` | Add samples to TX queue |
| `setRxCallback(fn)` | Register RX handler |
| `setLoopbackEnabled(bool)` | Enable TX→RX simulation |
| `setLoopbackSNR(db)` | Set simulation SNR |
| `setInputGain(gain)` | Set RX gain (0.0-2.0) |

---

## TX Path

```
Protocol Engine
    ↓ TxDataCallback
ModemEngine::transmit(data)
    ↓ LDPC encode → modulate → filter → normalize (0.8 peak)
    ↓
AudioEngine::queueTxSamples(samples)
    ↓ mutex-protected queue
    ↓
[SDL Audio Thread]
AudioEngine::outputCallback()
    ↓ pop from queue, fill buffer
    ↓
Output Device (speaker/HF rig)
```

### TX Buffer
- Type: `std::queue<float>` (FIFO)
- Mutex-protected
- No size limit (assumes producer matches 48kHz consumer)
- Outputs silence (0.0f) when empty

---

## RX Path

```
Input Device (microphone/HF rig)
    ↓
[SDL Audio Thread]
AudioEngine::inputCallback()
    ↓ DC blocking filter: y[n] = x[n] - x[n-1] + 0.995*y[n-1]
    ↓ Apply input gain
    ↓ Update RMS metering
    ↓
User RX Callback (if set)
    ↓
ModemEngine::feedAudio(samples)
    ↓
if connected:
    RxPipeline::feedAudio() → streaming decode
else:
    rx_sample_buffer_ → acquisition thread
```

### RX Buffer Management

| Buffer | Location | Max Size | Purpose |
|--------|----------|----------|---------|
| AudioEngine RX | audio_engine.cpp | 96,000 (2s) | Loopback mode |
| ModemEngine RX | modem_engine.hpp | 960,000 (20s) | Disconnected acquisition |
| RxPipeline | rx_pipeline.hpp | 960,000 (20s) | Connected streaming |

**Overflow handling:** Oldest samples dropped (no error notification)

---

## Loopback Simulation

For testing without hardware:

```cpp
audio_.setLoopbackEnabled(true);
audio_.setLoopbackSNR(15.0f);  // 15 dB SNR
```

**How it works:**
1. TX samples queued via `queueTxSamples()`
2. AWGN noise added based on SNR setting
3. Result injected directly into RX buffer
4. Retrieved by next `feedAudio()` call

**Noise generation:**
```cpp
SNR_linear = 10^(SNR_dB / 10)
noise_stddev = sqrt(signal_power / SNR_linear)
// Box-Muller transform for Gaussian noise
```

**Note:** Loopback doesn't use RX callback (avoids recursion).

---

## DC Blocking Filter

Applied in RX path to remove DC offset:

```cpp
y[n] = x[n] - x[n-1] + 0.995 * y[n-1]
```

- High-pass filter with ~15 Hz cutoff at 48 kHz
- Prevents DC offset from causing false sync detection
- Alpha coefficient (0.995) optimized for modem bandwidth

---

## Threading Model

```
MAIN THREAD:
├─ Device enumeration
├─ Open/close devices
└─ Configuration changes

AUDIO OUTPUT THREAD (SDL):
├─ outputCallback() called ~47 times/sec
├─ Pops from TX queue
└─ Must complete in <21ms

AUDIO INPUT THREAD (SDL):
├─ inputCallback() called ~47 times/sec
├─ Applies DC filter + gain
├─ Calls user RX callback
└─ Must complete in <21ms

MODEM THREADS:
├─ Acquisition: Searches for preambles
└─ Decode: LDPC decoding
```

**Synchronization:**
- TX/RX queues: mutex-protected
- Status flags: `std::atomic<bool>`
- Level meters: `std::atomic<float>`

---

## Level Metering

Both TX and RX have RMS level indicators:

```cpp
float output_level_;  // TX RMS (0.0-1.0)
float input_level_;   // RX RMS (0.0-1.0)
```

Calculated per callback buffer, displayed in GUI.

---

## Configuration Options

| Setting | Type | Default | Purpose |
|---------|------|---------|---------|
| Input device | string | "Default" | Microphone selection |
| Output device | string | "Default" | Speaker selection |
| Input gain | float | 1.0 | RX level adjustment (0.0-2.0) |
| TX drive | float | 0.8 | TX output level |
| TX delay | int | 50ms | PTT → TX delay |
| TX tail | int | 50ms | TX → PTT release delay |

---

## Latency Characteristics

| Stage | Latency |
|-------|---------|
| Audio capture | ~21ms (buffer) |
| DC filter | <1ms |
| ModemEngine processing | 100-500ms |
| RxPipeline accumulation | 1-5s (frame dependent) |
| LDPC decode | 100-500ms |
| **Total RX** | **1-6 seconds** |
| Audio output queue | ~21-100ms |
| ModemEngine TX | 100-500ms |
| **Total TX** | **150-600ms** |

---

## Error Handling

**Device open failure:**
- Returns false, logs via `SDL_GetError()`
- Caller should fallback to default device

**Buffer overflow:**
- Silent drop of oldest samples
- No error callback (by design)

**Device disconnection:**
- SDL may report errors in callbacks
- Should re-enumerate and reopen

---

## Key Files

| File | Purpose |
|------|---------|
| `src/gui/audio_engine.hpp` | Interface (130 lines) |
| `src/gui/audio_engine.cpp` | Implementation (344 lines) |
| `src/gui/modem/modem_rx.cpp` | feedAudio() handling |
| `src/gui/modem/rx_pipeline.hpp` | Streaming decode buffer |

---

## Adding Audio Features

1. **New audio effect (e.g., AGC):**
   - Add to `inputCallback()` after DC filter
   - Keep processing time <5ms

2. **New device type:**
   - Extend enumeration in `getInputDevices()`/`getOutputDevices()`
   - May need SDL2 plugin or alternative backend

3. **Sample rate change:**
   - Currently hardcoded to 48kHz
   - Would require changes in: AudioEngine, ModemEngine, all timing calculations
   - Not recommended (48kHz is standard for HF modems)
