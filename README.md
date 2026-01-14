# ProjectUltra

**A high-performance HF sound modem for amateur radio data transmission**

ProjectUltra implements adaptive OFDM modulation with near-Shannon-limit LDPC coding, designed to operate reliably over real HF channels including disturbed polar paths and auroral flutter conditions.

> **Status:** Work in progress. Core modem functional, GUI operational, no official releases yet.

---

## Key Capabilities

- **Throughput:** 0.5 - 10 kbps depending on channel conditions
- **Frequency Tolerance:** ±20 Hz carrier offset without external AFC
- **Modulation:** BPSK through 256-QAM with automatic fallback
- **Error Correction:** LDPC codes with rates 1/4 through 5/6
- **Robustness:** R1/4 mode achieves 100% decode under auroral flutter (2 Hz Doppler, 4 ms delay spread)

---

## Technical Overview

### Physical Layer

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48 kHz |
| Center Frequency | 1500 Hz |
| Bandwidth | 2812 Hz |
| FFT Size | 512 |
| Subcarrier Spacing | 93.75 Hz |
| Carriers | 30 (25 data, 5 pilots) |
| Symbol Duration | 10.7 - 11.8 ms |
| Cyclic Prefix | 32/48/64 samples (adaptive) |
| Frame Size | 648 bits |

### Modulation and Coding

| Modulation | Code Rate | Info Bits | Throughput | Required SNR (AWGN) |
|------------|-----------|-----------|------------|---------------------|
| BPSK | R1/4 | 162 | 0.5 kbps | ~0 dB |
| BPSK | R1/2 | 324 | 1.1 kbps | ~3 dB |
| QPSK | R1/2 | 324 | 2.1 kbps | ~6 dB |
| QPSK | R3/4 | 486 | 3.2 kbps | ~9 dB |
| 16-QAM | R3/4 | 486 | 6.4 kbps | ~15 dB |
| 64-QAM | R3/4 | 486 | 9.6 kbps | ~20 dB |
| 256-QAM | R3/4 | 486 | 12.8 kbps | ~26 dB |

Real HF channels require 5-10 dB additional margin due to fading.

### Signal Processing

**Transmit Chain:**
```
Data → LDPC Encode → Block Interleave (24×27) → QAM Mapping → OFDM (IFFT + CP) → Upconvert → Audio
```

**Receive Chain:**
```
Audio → Sync Detection → Coarse CFO → Downconvert → OFDM (FFT) → Channel Estimate → Equalize → Soft Demap → Deinterleave → LDPC Decode
```

**Synchronization:**
- Preamble: 4 short training symbols (STS) + 2 long training symbols (LTS)
- Detection: Schmidl-Cox correlation using analytic signal (Hilbert transform)
- CFO estimation: Preamble phase measurement, refined by pilot tracking
- Tolerance: ±20 Hz verified, algorithm supports ±40 Hz

**Channel Estimation:**
- Pilot-based least-squares estimation
- Linear interpolation across data carriers
- Exponential smoothing (α=0.9) for temporal tracking
- SNR estimation from pilot noise variance

**LDPC Decoder:**
- Min-Sum belief propagation with 0.75 scaling factor
- Maximum 50 iterations with early termination on parity check
- Systematic code structure: H = [H_data | I] for O(n) encoding

---

## Performance Validation

Performance tested using ITU-R F.1487 Watterson HF channel model with CCIR standard conditions:

| Condition | Delay Spread | Doppler | SNR | Description |
|-----------|--------------|---------|-----|-------------|
| Good | 0.5 ms | 0.1 Hz | 20 dB | Quiet mid-latitude, NVIS |
| Moderate | 1.0 ms | 0.5 Hz | 15 dB | Typical mid-latitude DX |
| Poor | 2.0 ms | 1.0 Hz | 10 dB | Disturbed, high-latitude |
| Flutter | 4.0 ms | 2.0 Hz | 8 dB | Auroral, polar paths |

### Decode Success Rate (100 frames per test)

| Mode | AWGN 20dB | Good | Moderate | Poor | Flutter |
|------|-----------|------|----------|------|---------|
| BPSK R1/4 | 100% | 100% | 100% | 100% | 100% |
| BPSK R1/2 | 100% | 100% | 100% | 100% | 69% |
| QPSK R1/2 | 100% | 100% | 100% | 25% | 0% |
| QPSK R3/4 | 100% | 100% | 100% | 0% | 0% |
| 16-QAM R3/4 | 100% | 100% | 100% | 0% | 0% |
| 64-QAM R3/4 | 100% | 100% | 64% | 0% | 0% |
| 256-QAM R3/4 | 100% | 81% | 0% | 0% | 0% |

### Frequency Offset Tolerance

| Offset | Sync Detection | Decode Success |
|--------|----------------|----------------|
| 0 Hz | Yes | 100% |
| ±5 Hz | Yes | 100% |
| ±10 Hz | Yes | 100% |
| ±15 Hz | Yes | 100% |
| ±20 Hz | Yes | 100% |

### Comparison

| System | Typical Throughput | Peak Throughput | License |
|--------|-------------------|-----------------|---------|
| VARA HF | 2-4 kbps | 8.5 kbps | Proprietary |
| PACTOR IV | 3-5 kbps | 10.5 kbps | Proprietary |
| ARDOP | 1-2 kbps | 2.4 kbps | Open |
| ProjectUltra | 2-5 kbps | 10+ kbps | MIT |

---

## Building

### Requirements

- CMake 3.16+
- C++20 compiler (GCC 10+, Clang 12+, MSVC 2019+)
- SDL2 (GUI only)
- OpenGL 2.1+ (GUI only)

### macOS

```bash
brew install sdl2 cmake
mkdir build && cd build
cmake ..
make -j$(sysctl -n hw.ncpu)
```

### Linux

```bash
sudo apt install libsdl2-dev cmake build-essential
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Windows

```powershell
vcpkg install sdl2:x64-windows
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

---

## Usage

```bash
./ultra_gui          # GUI application (TEST mode for loopback, OPERATE for radio)
./ultra_sim          # Full simulation test suite
./ultra_sim --quick  # Quick performance benchmark
./ultra_tests        # Unit tests
```

---

## Project Structure

```
ProjectUltra/
├── include/ultra/
│   ├── types.hpp       # Configuration, enums, type definitions
│   ├── ofdm.hpp        # OFDMModulator, OFDMDemodulator
│   ├── fec.hpp         # LDPCEncoder, LDPCDecoder, Interleaver
│   ├── dsp.hpp         # FFT, NCO, filters
│   └── logging.hpp     # Debug logging
├── src/
│   ├── ofdm/           # OFDM modulator and demodulator
│   ├── fec/            # LDPC encoder and decoder
│   ├── dsp/            # FFT, NCO, Hilbert transform
│   ├── gui/            # ImGui-based application
│   ├── protocol/       # ARQ, connection management, file transfer
│   └── sim/            # Watterson channel model, test harness
├── tests/              # Unit tests
└── thirdparty/         # Dear ImGui, miniz compression (bundled)
```

---

## Implementation Status

**Complete:**
- OFDM modulator/demodulator with pilot-based equalization
- LDPC encoder/decoder (rates 1/4, 1/2, 2/3, 3/4, 5/6)
- CFO-tolerant synchronization (±20 Hz)
- Block interleaver matched to LDPC frame size
- Watterson HF channel simulator
- GUI with constellation display and loopback testing
- Waterfall spectrum display
- File transfer protocol with chunked ARQ delivery
- Compression support (zlib-compatible)
- ARQ with selective repeat and connection management

**In Progress:**
- Soundcard loopback validation
- Over-the-air testing

---

## References

- ITU-R F.1487: Testing of HF modems with bandwidths up to about 12 kHz
- IEEE 802.11n/ac: LDPC code design principles
- Schmidl & Cox: Robust frequency and timing synchronization for OFDM

---

## License

MIT License

---

## Contributing

Contributions welcome. Priority areas:
- Real HF path validation
- Protocol layer improvements
- GUI enhancements
- Performance optimization
