# GUI Architecture

**Purpose:** Document the GUI structure for future development and maintenance.

---

## Overview

ProjectUltra uses **Dear ImGui + SDL2 + OpenGL 2.1** for cross-platform GUI.

| Component | Purpose |
|-----------|---------|
| Dear ImGui | Immediate-mode UI framework |
| SDL2 | Platform abstraction (audio, video, input) |
| OpenGL 2.1 | Rendering backend (chosen for hardware compatibility) |

**Entry Point:** `src/gui/main_gui.cpp`
**Window Size:** 1024x600 with dark theme

---

## Core Classes

### App (`src/gui/app.hpp/cpp` - 1,473 lines)

The main application class that coordinates all subsystems:

```cpp
class App {
    // Core components
    ModemEngine modem_;              // Our modem
    ProtocolEngine protocol_;        // Our protocol
    AudioEngine audio_;              // Audio I/O

    // Virtual station (simulation mode)
    ModemEngine virtual_modem_;      // Simulated remote station
    ProtocolEngine virtual_protocol_;
    std::thread sim_thread_;

    // UI state
    AppSettings settings_;           // Persisted settings
    std::vector<RxLogEntry> rx_log_; // Message history (max 20)

    // Widgets
    WaterfallWidget waterfall_;
    ConstellationWidget constellation_;
};
```

### ModemEngine (`src/gui/modem/modem_engine.hpp`)

All TX/RX audio processing:

| Method | Purpose |
|--------|---------|
| `transmit(data)` | Encode + modulate → audio samples |
| `feedAudio(samples)` | RX audio → decode pipeline |
| `setWaveformMode(mode)` | Switch between OFDM/DPSK/etc |
| `setDataMode(mod, rate)` | Change modulation + code rate |

### AudioEngine (`src/gui/audio_engine.hpp`)

SDL2-based audio I/O:

| Method | Purpose |
|--------|---------|
| `queueTxSamples(samples)` | Queue for TX playback |
| `setRxCallback(fn)` | Register RX sample handler |
| `setLoopbackEnabled(bool)` | Enable TX→RX simulation |
| `setLoopbackSNR(db)` | Set simulation SNR |

---

## Widget System

All widgets are stateless rendering functions (ImGui pattern):

### ConstellationWidget (`widgets/constellation.hpp`)
- Displays received symbols in I/Q plane
- Overlays ideal constellation grid
- Supports: DBPSK, DQPSK, QPSK, D8PSK, 16QAM, 32QAM

### WaterfallWidget (`widgets/waterfall.hpp`)
- Real-time frequency-domain display
- FFT: 2048-point
- History: 200 lines
- Range: 0-3000 Hz, -60 to 0 dB
- OpenGL texture rendering

### SettingsWindow (`widgets/settings.hpp`)
- Modal window with tabs: Station | Radio | Audio | Expert
- Persists to INI file
- Callbacks for all setting changes

### StatusWidget / ControlsWidget
- Channel status panel
- SNR bar with quality indicator
- Waveform and modulation display

---

## Virtual Station Simulator

Activated by: `./ultra_gui -sim`

**Architecture:**
```
Our TX → Queue → [Channel Noise] → Virtual Modem RX → Virtual Protocol
                                                              ↓
Our RX ← [Channel Noise] ← Queue ← Virtual TX ← Virtual Response
```

**Data Flow:**
- Two independent ModemEngine + ProtocolEngine instances
- Simulator thread handles bidirectional streaming
- AWGN noise injection based on SNR slider
- Chunk-based processing (480 samples = 10ms)

---

## Threading Model

```
MAIN THREAD (ImGui):
├─ 60 FPS render loop
├─ Protocol ticking
├─ Widget rendering
└─ Settings persistence

AUDIO THREAD (SDL):
├─ outputCallback: TX sample streaming
└─ inputCallback: RX capture → modem_.feedAudio()

ACQUISITION THREAD:
├─ Searches for preambles
└─ Queues DetectedFrame

RX/DECODE THREAD:
├─ LDPC decoding
└─ Frame delivery via callbacks

SIMULATOR THREAD (if -sim):
├─ Bidirectional streaming
├─ Channel effects
└─ Virtual protocol ticking
```

---

## UI Layout

```
┌─────────────────────────────────────────────────────────────┐
│ ProjectUltra | Settings                                     │
├─────────────────────────────────────────────────────────────┤
│  ┌───────────────────────┬────────────────────────────────┐ │
│  │   LEFT (32%)          │   RIGHT (68%)                  │ │
│  │                       │                                │ │
│  │ ┌───────────────────┐ │ ┌────────────────────────────┐ │ │
│  │ │ Constellation     │ │ │ Simulator [Enable] [SNR]   │ │ │
│  │ └───────────────────┘ │ ├────────────────────────────┤ │ │
│  │                       │ │ [Callsign] [Remote] [Conn] │ │ │
│  │ ┌───────────────────┐ │ ├────────────────────────────┤ │ │
│  │ │ Channel Status    │ │ │ [Message Input] [Send]     │ │ │
│  │ │ - Waveform        │ │ ├────────────────────────────┤ │ │
│  │ │ - SNR bar         │ │ │ RX Log (20 lines)          │ │ │
│  │ │ - Frame stats     │ │ │ [SYS] Connected...         │ │ │
│  │ └───────────────────┘ │ │ [RX] Hello!                │ │ │
│  │                       │ └────────────────────────────┘ │ │
│  │ ┌───────────────────┐ │                                │ │
│  │ │ Waterfall         │ │                                │ │
│  │ │ (0-3000 Hz)       │ │                                │ │
│  │ └───────────────────┘ │                                │ │
│  └───────────────────────┴────────────────────────────────┘ │
├─────────────────────────────────────────────────────────────┤
│ Status: Mode=RX | SNR=15.3dB | TX=2 | RX=5 | 2.5kbps       │
└─────────────────────────────────────────────────────────────┘
```

---

## Callback Architecture

All components use callbacks for loose coupling:

**ModemEngine → App:**
- `setRawDataCallback()` - Decoded frame bytes
- `setStatusCallback()` - Progress messages
- `setPingReceivedCallback()` - PING detection

**ProtocolEngine → App:**
- `setTxDataCallback()` - Transmit frame
- `setMessageReceivedCallback()` - Decoded message
- `setConnectionChangedCallback()` - State change
- `setModeNegotiatedCallback()` - Waveform switch

**AudioEngine → App:**
- `setRxCallback()` - Incoming audio samples

---

## Connection State Machine

```
DISCONNECTED → PROBING → CONNECTING → CONNECTED → DISCONNECTING
     ↑                                      ↓
     └──────────────────────────────────────┘
```

State transitions trigger:
1. Modem waveform mode update
2. RX log message
3. UI status update
4. Protocol frame transmission

---

## Key Files

| File | Lines | Purpose |
|------|-------|---------|
| `src/gui/app.cpp` | 1,473 | Main application |
| `src/gui/main_gui.cpp` | 144 | Entry point + event loop |
| `src/gui/audio_engine.cpp` | 344 | Audio I/O |
| `src/gui/modem/modem_engine.cpp` | 1,000+ | Modem control |
| `src/gui/widgets/settings.cpp` | 600 | Settings dialog |
| `src/gui/widgets/waterfall.cpp` | 300 | Waterfall display |
| `src/gui/widgets/constellation.cpp` | 200 | Constellation display |

---

## Adding New GUI Features

1. **New Widget:**
   - Create `src/gui/widgets/my_widget.hpp/cpp`
   - Add to CMakeLists.txt
   - Call from `App::render()`

2. **New Setting:**
   - Add field to `AppSettings` struct
   - Add to `load()` and `save()` methods
   - Add UI in `SettingsWindow::render()`
   - Add callback if needed

3. **New Status Display:**
   - Add state variable to `App`
   - Update in appropriate callback
   - Render in `renderCompactChannelStatus()`
