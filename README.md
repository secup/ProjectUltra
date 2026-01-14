# ProjectUltra

**An adaptive HF modem for amateur radio digital communication**

ProjectUltra is a software modem designed for sending messages and files over HF radio. It uses modern signal processing techniques to maintain reliable communication across a wide range of band conditions, from quiet channels to paths affected by fading and interference.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

---

## Overview

ProjectUltra connects to your transceiver through a standard audio interface and handles the complete data link: modulation, error correction, synchronization, and an ARQ protocol for guaranteed delivery.

**Capabilities:**
- Text messaging between stations
- File transfer with integrity verification
- Automatic adaptation to channel conditions
- Works with any HF transceiver via audio interface

**Speed:**
| Conditions | Throughput | Example |
|------------|------------|---------|
| Poor (fading/interference) | ~1 kbps | 10 KB file in ~1.5 min |
| Typical | ~3-6 kbps | 10 KB file in ~15-30 sec |
| Good (clear channel) | ~10+ kbps | 10 KB file in ~8 sec |

**Design Goals:**
- Reliable operation under real-world HF propagation
- Throughput that scales with channel quality
- Simple setup with no specialized hardware required

---

## Getting Started

### Requirements

- A computer running macOS, Linux, or Windows
- An HF transceiver with audio input/output
- A sound card interface (SignaLink, RigBlaster, or similar) or direct audio connection

### Building

```bash
# Install dependencies
# macOS: brew install sdl2 cmake
# Linux: sudo apt install libsdl2-dev cmake build-essential

git clone https://github.com/yourusername/ProjectUltra.git
cd ProjectUltra
mkdir build && cd build
cmake ..
cmake --build .
```

### Running

```bash
./ultra_gui
```

1. Open **Settings** and enter your callsign
2. Select your audio devices (input from radio, output to radio)
3. Use **Test Mode** to verify operation without transmitting
4. Switch to **Operate Mode** to go on-air

---

## Features

### Communication
- **Messaging:** Send and receive text messages with callsign addressing
- **File Transfer:** Transfer files of any type with automatic chunking and verification
- **Connection Management:** Establish links with remote stations, with proper handshaking

### Reliability
- **Error Correction:** LDPC codes provide strong forward error correction
- **ARQ Protocol:** Automatic retransmission ensures data integrity
- **Adaptive Operation:** Adjusts modulation and coding based on channel quality

### Interface
- **Waterfall Display:** Real-time spectrum visualization
- **Constellation View:** Monitor signal quality and demodulation
- **Message Log:** Track sent and received traffic

---

## Use Cases

- **General Operation:** Day-to-day data communication between amateur stations
- **File Exchange:** Share documents, images, or small files over HF
- **Portable/Field Operation:** Lightweight software solution for field deployments
- **Emergency Communication:** Reliable messaging when other infrastructure is unavailable

---

## Current Status

The core modem and protocol are functional. The software is in active development and testing.

**Implemented:**
- OFDM modulator and demodulator
- LDPC encoder and decoder
- ARQ protocol with connection management
- File transfer with compression
- GUI with spectrum and constellation displays
- Simulated channel testing

**In Progress:**
- Extended on-air validation
- Documentation and tutorials

---

## Documentation

- [Building on Windows](docs/windows-build.md) *(coming soon)*
- [Audio Interface Setup](docs/audio-setup.md) *(coming soon)*
- [Protocol Specification](docs/protocol.md) *(coming soon)*

---

## Contributing

Contributions are welcome. If you're interested in helping with development, testing, or documentation, please open an issue to discuss.

Priority areas:
- On-air testing across different paths and conditions
- User interface improvements
- Documentation

---

## License

Released under the MIT License. See [LICENSE](LICENSE) for details.

---

## Technical Reference

<details>
<summary>Click to expand technical details</summary>

### Physical Layer

ProjectUltra uses OFDM (Orthogonal Frequency-Division Multiplexing) with the following parameters:

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48 kHz |
| Bandwidth | ~2.8 kHz |
| FFT Size | 512 |
| Subcarriers | 30 (25 data, 5 pilot) |
| Subcarrier Spacing | 93.75 Hz |
| Symbol Duration | ~11 ms |
| Cyclic Prefix | Adaptive |

### Modulation Schemes

Supports BPSK, QPSK, 16-QAM, 64-QAM, and 256-QAM with automatic or manual selection.

### Throughput

Measured throughput under simulated HF channel conditions (ITU-R F.1487 Watterson model):

| Mode | Good Channel | Moderate | Poor |
|------|--------------|----------|------|
| BPSK R1/4 | 0.5 kbps | 0.5 kbps | 0.5 kbps |
| BPSK R1/2 | 1.1 kbps | 1.1 kbps | 1.1 kbps |
| QPSK R1/2 | 2.1 kbps | 2.1 kbps | - |
| QPSK R3/4 | 3.2 kbps | 3.2 kbps | - |
| 16-QAM R1/2 | 4.3 kbps | 4.3 kbps | - |
| 16-QAM R3/4 | 6.4 kbps | 6.4 kbps | - |
| 64-QAM R3/4 | 9.6 kbps | 6.1 kbps | - |
| 256-QAM R3/4 | 12.8 kbps | - | - |

Lower modes (BPSK) work reliably in poor conditions. Higher modes require better SNR but deliver faster throughput.

### Error Correction

LDPC (Low-Density Parity-Check) codes with rates 1/4, 1/2, 2/3, 3/4, and 5/6. The decoder uses min-sum belief propagation with early termination.

### Synchronization

Preamble-based synchronization using Schmidl-Cox correlation. Tolerates carrier frequency offsets up to ±20 Hz without external AFC.

### Channel Estimation

Pilot-aided least-squares estimation with interpolation across subcarriers and temporal smoothing.

### Protocol

Stop-and-wait ARQ with sequence numbering. Supports connection establishment, data transfer, and graceful disconnection.

### Testing

Validated using the ITU-R F.1487 Watterson HF channel model under various conditions including multipath and Doppler spread.

### Architecture

```
┌──────────────────────────────────────────────────┐
│              Application Layer                    │
├──────────────────────────────────────────────────┤
│    Protocol (ARQ, Connection, File Transfer)     │
├──────────────────────────────────────────────────┤
│              Modem Engine                         │
├──────────────────────────────────────────────────┤
│  LDPC Codec  │  Interleaver  │  OFDM Mod/Demod  │
├──────────────────────────────────────────────────┤
│              Audio Interface                      │
└──────────────────────────────────────────────────┘
```

### References

- ITU-R F.1487: Testing of HF modems with bandwidths up to about 12 kHz
- IEEE 802.11n/ac: LDPC code structure
- Schmidl, T. M., & Cox, D. C.: Robust frequency and timing synchronization for OFDM

</details>

---

## Acknowledgments

Built with [Dear ImGui](https://github.com/ocornut/imgui), [SDL2](https://libsdl.org/), and [miniz](https://github.com/richgel999/miniz).
