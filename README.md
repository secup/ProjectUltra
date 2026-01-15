# ProjectUltra

**A high-performance adaptive HF modem for amateur radio digital communication**

> ⚠️ **Work in Progress** — This project is under active development. No official release yet. APIs and protocols may change. Use for experimentation and testing only.

ProjectUltra is a software modem designed for reliable data transfer over HF radio. It combines modern signal processing techniques—OFDM, OTFS, LDPC codes, and adaptive equalization—to maintain communication across challenging ionospheric conditions including multipath fading, Doppler spread, and interference.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: WIP](https://img.shields.io/badge/Status-Work%20in%20Progress-yellow.svg)]()

---

## Key Features

- **Dual Waveform Support:** OFDM for stable channels, OTFS for harsh conditions
- **Strong Error Correction:** LDPC codes with rates from 1/4 to 5/6
- **Selective Repeat ARQ:** 2-4x throughput improvement over stop-and-wait
- **Adaptive Equalization:** LMS/RLS tracking for time-varying channels
- **Implicit Channel Probing:** Automatic link adaptation without overhead
- **File Transfer:** Compressed transfers with integrity verification

---

## Speed

| Mode | Code Rate | Raw Rate | Typical Use |
|------|-----------|----------|-------------|
| BPSK | R1/4 | 540 bps | Polar flutter, worst conditions |
| BPSK | R1/2 | 1.1 kbps | Poor/disturbed paths |
| QPSK | R1/2 | 2.1 kbps | Moderate conditions, reliable |
| QPSK | R3/4 | 3.2 kbps | Good conditions |
| 16-QAM | R1/2 | 4.3 kbps | Good SNR |
| 16-QAM | R3/4 | 6.4 kbps | Very good conditions |
| 64-QAM | R3/4 | 9.6 kbps | Excellent SNR |
| 64-QAM | R5/6 | 10.7 kbps | Near-ideal channel |

*Rates are theoretical maximums. Actual throughput depends on ARQ retransmissions and channel conditions.*

---

## Performance

Tested against **ITU-R F.1487** Watterson HF channel model at 15 dB SNR:

### Frame Success Rate by Channel Condition

| Channel | Delay | Doppler | OFDM QPSK | OTFS-RAW | Best Choice |
|---------|-------|---------|-----------|----------|-------------|
| AWGN | - | - | 100% | 100% | Either |
| Good | 0.5 ms | 0.1 Hz | 66% | 90% | OTFS |
| Moderate | 1.0 ms | 0.5 Hz | 82% | 42% | OFDM |
| Poor | 2.0 ms | 1.0 Hz | 10% | 20% | OTFS |
| Flutter | 0.5 ms | 10 Hz | 0% | 0% | BPSK mode |

### Harsh Channel Performance (Poor & Flutter)

BPSK with R1/4 coding provides reliable operation where higher-order modulation fails:

| Channel | Mode | OFDM | OTFS-RAW | Winner |
|---------|------|------|----------|--------|
| Poor | BPSK R1/4 | 54% | **72%** | OTFS |
| Poor | BPSK R1/2 | 6% | **66%** | OTFS |
| Flutter | BPSK R1/4 | 30% | **58%** | OTFS |
| Flutter | BPSK R1/2 | 0% | **14%** | OTFS |

**OTFS significantly outperforms OFDM on doubly-selective channels.**

### Throughput

| Conditions | Mode | Effective Rate |
|------------|------|----------------|
| Excellent (>25 dB) | 64-QAM R5/6 | ~10 kbps |
| Good (20-25 dB) | 16-QAM R3/4 | ~6 kbps |
| Moderate (15-20 dB) | QPSK R1/2 | ~2 kbps |
| Poor (10-15 dB) | BPSK R1/2 | ~1 kbps |
| Flutter (<10 dB) | BPSK R1/4 | ~0.5 kbps |

---

## Technology

### OTFS (Orthogonal Time Frequency Space)

OTFS is a 2D modulation scheme designed for doubly-selective channels—those with both multipath (delay spread) and Doppler spread. Unlike OFDM which suffers from inter-carrier interference under fast fading, OTFS spreads each data symbol across the entire time-frequency grid, providing diversity against both delay and Doppler.

**When to use OTFS:**
- Polar/auroral paths with flutter fading
- Mobile or aeronautical HF
- Long paths with significant Doppler shift
- Disturbed ionospheric conditions

### Selective Repeat ARQ

The protocol supports both Stop-and-Wait and Selective Repeat ARQ modes:

| Metric | Stop-and-Wait | Selective Repeat |
|--------|---------------|------------------|
| Window Size | 1 | 4-8 configurable |
| Throughput | Baseline | 2-4x improvement |
| Latency (10 frames) | 20-30 sec | 5-10 sec |
| Complexity | Simple | Moderate |

Selective Repeat uses a sliding window with per-frame acknowledgments, dramatically improving throughput on paths with long round-trip times.

### Adaptive Equalization

Two equalizer algorithms track time-varying channel responses:

- **LMS (Least Mean Squares):** Lower complexity, good for slow fading
- **RLS (Recursive Least Squares):** Faster convergence, better for flutter

Decision-directed mode uses decoded symbols as references between pilots, enabling continuous channel tracking.

### Implicit Channel Probing

Link adaptation happens automatically without explicit probing overhead:

1. When station B decodes any frame from station A, the demodulator measures:
   - SNR (from pilot symbols)
   - Delay spread (from channel impulse response)
   - Doppler spread (from channel time variation)

2. Station B includes these measurements in its response (ACK or CONNECT_ACK)

3. Station A adjusts modulation/coding for subsequent transmissions

This eliminates dedicated probe frames while maintaining accurate channel awareness.

---

## Getting Started

### Requirements

- macOS, Linux, or Windows
- HF transceiver with audio I/O
- **SSB filter width: 2.8 kHz or wider** (signal bandwidth is ~2.8 kHz)
- Sound card interface (SignaLink, RigBlaster, etc.) or direct connection

### Building

```bash
# Install dependencies
# macOS: brew install sdl2 cmake
# Linux: sudo apt install libsdl2-dev cmake build-essential

git clone https://github.com/secup/ProjectUltra.git
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
2. Select audio devices (input from radio, output to radio)
3. Use **Test Mode** to verify operation without transmitting
4. Switch to **Operate Mode** to go on-air

### Test Mode

Two virtual stations (TEST1, TEST2) with cross-wired audio for development:

- **Loopback:** Single modem, TX audio loops to RX
- **Protocol Test:** Full two-modem simulation with channel model

---

## Use Cases

- **General HF Data:** Day-to-day messaging between amateur stations
- **File Exchange:** Documents, images, small files over HF
- **Portable/Field Ops:** Lightweight software for field deployments
- **NVIS Networks:** Regional coverage via 40m/80m near-vertical skywave
- **Emergency Comms:** Reliable messaging when infrastructure is down
- **Challenging Paths:** Polar, auroral, or mobile HF with OTFS mode

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    GUI Application (ImGui)                       │
├─────────────────────────────────────────────────────────────────┤
│         Protocol Engine (ARQ, Connection, File Transfer)         │
├─────────────────────────────────────────────────────────────────┤
│         Modem Engine (TX/RX coordination, carrier sense)         │
├─────────────────────────────────────────────────────────────────┤
│   LDPC Encoder/Decoder  │  Interleaver  │  OFDM/OTFS Mod/Demod  │
├─────────────────────────────────────────────────────────────────┤
│                      Audio I/O (SDL2)                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## Technical Details

<details>
<summary>Click to expand</summary>

### OFDM Parameters

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48 kHz |
| FFT Size | 512 |
| Subcarrier Spacing | 93.75 Hz |
| Total Subcarriers | 30 |
| Data Subcarriers | 25 |
| Pilot Subcarriers | 5 (every 6th) |
| **Signal Bandwidth** | **~2.8 kHz** |
| Center Frequency | 1500 Hz |
| Symbol Duration | ~11 ms |
| Cyclic Prefix | Adaptive (SHORT/MEDIUM/LONG) |

> **Note:** Requires SSB filter set to 2.8 kHz or wider. Standard 2.4 kHz filters will attenuate upper subcarriers.

### OTFS Parameters

| Parameter | Value |
|-----------|-------|
| Delay Bins (M) | 32 |
| Doppler Bins (N) | 16 (QPSK) / 32 (BPSK) |
| Grid Size | 512-1024 symbols |
| Transform | ISFFT/SFFT (radix-2) |

### LDPC Codes

| Rate | Info Bits (k) | Codeword (n) | Overhead |
|------|---------------|--------------|----------|
| R1/4 | 162 | 648 | 4x |
| R1/2 | 324 | 648 | 2x |
| R2/3 | 432 | 648 | 1.5x |
| R3/4 | 486 | 648 | 1.33x |
| R5/6 | 540 | 648 | 1.2x |

Decoder: Min-sum belief propagation with early termination (max 50 iterations).

### Modulation Schemes

- **BPSK:** 1 bit/symbol, most robust
- **QPSK:** 2 bits/symbol, good balance
- **16-QAM:** 4 bits/symbol, requires good SNR
- **64-QAM:** 6 bits/symbol, excellent conditions only
- **256-QAM:** 8 bits/symbol, exceptional conditions

### Synchronization

Schmidl-Cox preamble correlation with:
- Coarse CFO estimation (±20 Hz tolerance)
- Fine timing recovery
- Adaptive threshold (0.7 default)

### Protocol Frames

| Field | Size | Description |
|-------|------|-------------|
| Magic | 2 bytes | Frame identifier (0x55AA) |
| Type | 1 byte | CONNECT, DATA, ACK, etc. |
| Flags | 1 byte | MORE_DATA, COMPRESSED, etc. |
| Sequence | 1 byte | ARQ sequence number |
| Src Call | 8 bytes | Source callsign |
| Dst Call | 8 bytes | Destination callsign |
| Length | 1 byte | Payload length |
| Payload | 0-255 bytes | Frame data |
| CRC | 2 bytes | CRC-16 checksum |

### Channel Model

Testing uses ITU-R F.1487 Watterson model:

| Condition | Delay Spread | Doppler Spread | Description |
|-----------|--------------|----------------|-------------|
| AWGN | 0 | 0 | Ideal channel |
| Good | 0.5 ms | 0.1 Hz | Quiet band |
| Moderate | 1.0 ms | 0.5 Hz | Typical daytime |
| Poor | 2.0 ms | 1.0 Hz | Disturbed path |
| Flutter | 0.5 ms | 10 Hz | Auroral/polar |

### References

- ITU-R F.1487: Testing of HF modems with bandwidths up to about 12 kHz
- Hadani et al. (2017): "Orthogonal Time Frequency Space Modulation"
- IEEE 802.11n/ac: LDPC code structure
- Schmidl & Cox: Robust frequency and timing synchronization for OFDM

</details>

---

## Current Status

**What Works:**
- OFDM and OTFS modulation/demodulation
- LDPC encoding/decoding (all rates)
- Selective Repeat ARQ protocol
- File transfer with compression
- GUI with waterfall and constellation displays
- Channel simulation (ITU-R F.1487 Watterson model)
- Two-modem test mode

**In Progress:**
- On-air testing and validation
- Adaptive mode selection (auto OFDM/OTFS switching)
- RLS equalizer tuning for flutter channels
- Documentation and tutorials

**Not Yet Implemented:**
- Multi-carrier bonding
- Automatic frequency control (AFC) beyond ±20 Hz

---

## Contributing

Contributions welcome! Priority areas:

- On-air testing across different paths and conditions
- OTFS optimization for specific channel types
- User interface improvements
- Documentation and tutorials

Please open an issue to discuss before submitting PRs.

---

## License

Released under the MIT License. See [LICENSE](LICENSE) for details.

---

## Acknowledgments

Built with [Dear ImGui](https://github.com/ocornut/imgui), [SDL2](https://libsdl.org/), and [miniz](https://github.com/richgel999/miniz).
