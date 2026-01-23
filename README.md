# ProjectUltra

**High-performance HF modem for amateur radio**

*Last updated: 2026-01-22*

> **EXPERIMENTAL SOFTWARE - WORK IN PROGRESS**
>
> This project is under active development and is **not ready for production use**.
> Features may be incomplete, untested, or broken. APIs and protocols may change
> without notice. Performance numbers are from simulation only - real-world HF
> testing is ongoing. **Use at your own risk for experimentation and development only.**

ProjectUltra is a software modem that achieves reliable, high-speed data transfer over HF radio. It uses adaptive waveform selection to maintain communication across varying ionospheric conditions - from quiet bands to disturbed polar paths.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: Experimental](https://img.shields.io/badge/Status-Experimental-orange.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg)]()

---

## Features

- **Adaptive Waveforms**: Automatically selects optimal waveform based on channel conditions
- **Wide SNR Range**: Operates from -17 dB (reported) to 30+ dB
- **Strong FEC**: LDPC codes with rates from R1/4 to R5/6
- **ARQ Protocol**: Reliable delivery with automatic retransmission
- **GUI Application**: Real-time waterfall, constellation display, and message log
- **CLI Tools**: Scriptable transmit/receive for automation

---

## Performance

| Channel Condition | Waveform | Throughput | Min SNR |
|-------------------|----------|------------|---------|
| Very low SNR | MFSK | 47 bps | -17 dB* |
| Poor/Flutter HF | Single-carrier DPSK | 300-1100 bps | 0-15 dB |
| Good HF | OFDM DQPSK | 1.1-2.3 kbps | 17-25 dB |
| Excellent | OFDM D8PSK | 3.4 kbps | 25+ dB |

*Reported SNR in 2500 Hz bandwidth; actual SNR in signal bandwidth is higher.

### Waveform Strategy

ProjectUltra uses three waveform types, each optimized for different conditions:

```
SNR Range           Waveform              Why
─────────────────────────────────────────────────────────────
< 0 dB              MFSK 8-tone           Narrow bandwidth, long integration
0-17 dB             Single-carrier DPSK   Full power concentration, no ICI
17+ dB              OFDM                  High throughput, parallel carriers
```

**Key insight**: On challenging HF channels (multipath, flutter), single-carrier DPSK achieves 100% success where multi-carrier schemes fail. This is because DPSK concentrates all power in one carrier and differential encoding cancels phase distortion.

---

## Getting Started

### Requirements

- Linux, macOS, or Windows
- CMake 3.16+
- SDL2 (for GUI)
- C++20 compiler (GCC 10+, Clang 12+, MSVC 2019+)

### Building

```bash
# Install dependencies
# Ubuntu/Debian: sudo apt install libsdl2-dev cmake build-essential
# macOS: brew install sdl2 cmake
# Windows: vcpkg install sdl2

git clone https://github.com/mfrigerio/ProjectUltra.git
cd ProjectUltra
mkdir build && cd build
cmake ..
make -j4
```

### Running

**GUI Application:**
```bash
./ultra_gui              # Normal mode
./ultra_gui -sim         # Simulator mode (no radio needed)
```

**CLI Tools:**
```bash
# Transmit
./ultra -s MYCALL -d THEIRCALL ptx "Hello World" | aplay -f FLOAT_LE -r 48000

# Receive
arecord -f FLOAT_LE -r 48000 | ./ultra prx

# Loopback test
./ultra ptx "Test message" | ./ultra prx
```

---

## How It Works

### Protocol

1. **PING/PONG** - Fast presence probe (~1 sec each) to check if remote station is listening
2. **CONNECT** - Full callsign exchange after successful probe (FCC Part 97.119 compliance)
3. **MODE_CHANGE** - Negotiates optimal modulation/coding based on measured SNR
4. **DATA** - Transfers payload with per-frame acknowledgment
5. **DISCONNECT** - Graceful termination with callsign ID (regulatory compliance)

The PING/PONG probe allows quick "anyone home?" detection before committing to the
full CONNECT sequence. If no response after 5 PINGs (15 seconds), connection fails fast.

### Signal Parameters

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48 kHz |
| Bandwidth | ~2.8 kHz |
| Center Frequency | 1500 Hz (audio) |
| OFDM Carriers | 30 |
| LDPC Codeword | 648 bits |

### LDPC Codes

| Rate | Info Bytes | Use Case |
|------|------------|----------|
| R1/4 | 20 | Low SNR, maximum reliability |
| R1/2 | 40 | Moderate SNR, good balance |
| R2/3 | 54 | Good SNR, higher throughput |
| R3/4 | 60 | Very good SNR |
| R5/6 | 67 | Excellent SNR, maximum throughput |

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│              GUI Application (ImGui + SDL2)             │
├─────────────────────────────────────────────────────────┤
│     Protocol Engine (Connection, ARQ, File Transfer)    │
├─────────────────────────────────────────────────────────┤
│        Modem Engine (TX/RX coordination, SNR est)       │
├──────────────┬──────────────┬───────────────────────────┤
│ OFDM Mod/Dem │ DPSK Mod/Dem │ MFSK Mod/Dem             │
├──────────────┴──────────────┴───────────────────────────┤
│  LDPC Encoder/Decoder  │  Interleaver  │  Sync/CFO     │
├─────────────────────────────────────────────────────────┤
│                    Audio I/O (SDL2)                     │
└─────────────────────────────────────────────────────────┘
```

---

## Testing

```bash
cd build

# Run unit tests
ctest

# Comprehensive modem test
./tests/test_comprehensive_modem

# CLI protocol simulator
./cli_simulator --snr 20 --test

# OFDM + LDPC integration test
./test_ofdm_ldpc_r23
```

---

## Radio Setup

### Requirements
- SSB transceiver with 2.8+ kHz filter bandwidth
- Audio interface (SignaLink, RigBlaster, or direct soundcard connection)
- PTT control (VOX, CAT, or hardware)

### Audio Levels
- TX: Adjust for clean signal without ALC compression
- RX: Set for comfortable listening level (avoid clipping)

### Recommended Operating Frequencies

ProjectUltra uses **~2.8 kHz bandwidth**. Operate in wideband digital segments,
not narrow-band FT8/PSK31 areas.

| Band | Frequency (USB) | Notes |
|------|-----------------|-------|
| 80m | 3.590 MHz | Above narrow digital, below voice |
| 40m | 7.102 MHz | Common for VARA/Winlink |
| 30m | 10.145 MHz | Check for WSPR at 10.140 |
| 20m | 14.108 MHz | Above FT8 crowd, wideband digital |
| 15m | 21.110 MHz | Above narrow digital segment |
| 10m | 28.120 MHz | Plenty of room |

**Avoid:**
- 14.070-14.095 MHz (FT8/PSK31)
- Any .074 MHz frequencies (FT8)
- 14.100 MHz (NCDXF beacons)

**Best practice:** Listen for 10-15 seconds before transmitting. Use minimum
power necessary. Be ready to QSY if causing interference.

---

## Current Status

> **Note**: All features below have been tested in simulation only (ITU-R F.1487 Watterson
> channel model). On-air testing with real HF equipment is in progress. Real-world
> performance may differ significantly from simulation results.

**Working (in simulation):**
- All LDPC code rates (R1/4 through R5/6)
- OFDM with DQPSK/D8PSK modulation
- Single-carrier DPSK (DBPSK/DQPSK/D8PSK)
- MFSK (2/4/8/16/32 tones)
- ARQ protocol with retransmission
- GUI with waterfall and constellation
- CLI transmit/receive tools
- HF channel simulator (ITU-R F.1487)

**In Progress:**
- On-air testing
- Automatic waveform switching
- File transfer optimization

---

## Contributing

Contributions welcome! Areas of interest:

- On-air testing and performance reports
- Documentation and tutorials
- Bug fixes and optimizations

Please open an issue before submitting large PRs.

### Help Wanted: Real HF Recordings

**We need real-world HF recordings to validate the modem beyond simulation.**

If you're a licensed amateur radio operator, you can help by recording your own
transmissions via WebSDR. This is legitimate amateur experimentation under
FCC Part 97 (advancement of the radio art).

**How to record your own signal:**

1. Pick a frequency from the recommended list above (e.g., 14.108 MHz USB)
2. Open a WebSDR receiver (websdr.org or kiwisdr.com) and tune to that frequency
3. Start recording on the WebSDR (or use Audacity to capture system audio)
4. Transmit using ProjectUltra (see options below)
5. Stop recording and save the file

**What to transmit:**

| Frame Type | Duration | Best For |
|------------|----------|----------|
| **PING probe** | ~1 second | Quick propagation test |
| **Full CONNECT** | ~8 seconds | Complete handshake test |

- **PING probe**: Short DPSK burst with "ULTR" marker. Fast way to check if your
  signal is reaching the WebSDR. In GUI: click Connect, recording starts immediately
  with the PING. Cancel after 2-3 seconds if you just want the probe.

- **Full CONNECT**: Complete connection attempt with callsign exchange (3 codewords).
  More comprehensive test of sync, LDPC decode, and protocol parsing.

**What to submit:**
- Recording file (48kHz mono WAV or F32 preferred)
- Your callsign and transmit location
- Band/frequency and time (UTC)
- WebSDR location used (e.g., "Twente WebSDR, Netherlands")
- Approximate path distance
- Band conditions if known (S-meter readings)
- Frame type transmitted (PING or CONNECT)

**Why this helps:**
Real ionospheric propagation has characteristics that simulation can't capture.
Even "failed" recordings where the signal didn't decode are valuable - they
help us improve sync detection and weak-signal performance.

**How to submit:** Open a GitHub issue with the "Recording" label.

---

## License

MIT License. See [LICENSE](LICENSE) for details.

---

## Acknowledgments

- [Dear ImGui](https://github.com/ocornut/imgui) - GUI framework
- [SDL2](https://libsdl.org/) - Audio and windowing
- [miniz](https://github.com/richgel999/miniz) - Compression
