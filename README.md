# ProjectUltra

> **Note:** This project is a work in progress. No official releases are available yet.

A high-performance HF sound modem designed for amateur radio data transmission over voice channels. Features adaptive OFDM modulation, near-Shannon-limit LDPC forward error correction, CFO-tolerant synchronization, and a real-time GUI with constellation diagram.

## Features

- **OFDM modulation** with 30 carriers in 2.8 kHz bandwidth
- **Adaptive modulation**: BPSK, QPSK, 16-QAM, 64-QAM, 256-QAM
- **LDPC FEC** with code rates 1/4, 1/2, 2/3, 3/4, 5/6
- **CFO-tolerant sync**: ±20 Hz frequency offset tolerance using analytic signal correlation
- **Throughput**: 0.5-10 kbps depending on conditions
- **Watterson HF channel simulator** for realistic testing (ITU-R F.1487)
- **Cross-platform GUI** using Dear ImGui + SDL2 + OpenGL 2.1
- **Real-time constellation diagram** and signal quality metrics
- **ARQ protocol layer** for reliable data transfer

## Technical Specifications

### Physical Layer Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Sample Rate | 48 kHz | Standard audio rate |
| Center Frequency | 1500 Hz | Centered in SSB passband |
| Bandwidth | 2812 Hz | Fits 2.8 kHz HF channel |
| FFT Size | 512 | 93.75 Hz bin spacing |
| Subcarrier Spacing | 93.75 Hz | Robust against Doppler |
| Total Carriers | 30 | 25 data + 5 pilots |
| Pilot Spacing | Every 6th carrier | 17% overhead |
| Symbol Duration | 10.7-11.8 ms | Depending on CP mode |
| Cyclic Prefix | 32/48/64 samples | SHORT/MEDIUM/LONG |
| Frame Size | 648 bits | One LDPC codeword |

### Modulation Schemes

| Modulation | Bits/Carrier | Bits/Symbol | Min SNR (AWGN) | Raw Throughput |
|------------|--------------|-------------|----------------|----------------|
| BPSK | 1 | 25 | ~3 dB | 1.1 kbps |
| QPSK | 2 | 50 | ~6 dB | 2.1 kbps |
| 16-QAM | 4 | 100 | ~12 dB | 4.3 kbps |
| 64-QAM | 6 | 150 | ~18 dB | 6.4 kbps |
| 256-QAM | 8 | 200 | ~24 dB | 12.8 kbps |

*Note: Real HF channels require 5-10 dB higher SNR due to Rayleigh fading and multipath.*

### Forward Error Correction

| Parameter | Value |
|-----------|-------|
| Code Type | LDPC (Low-Density Parity-Check) |
| Block Size | 648 bits (81 bytes) |
| Code Rates | 1/4, 1/2, 2/3, 3/4, 5/6 |
| Decoder | Min-Sum Belief Propagation |
| Max Iterations | 50 |
| Scaling Factor | 0.75 (improves convergence) |

#### Code Rate Details

| Rate | Info Bits | Parity Bits | Redundancy | Use Case |
|------|-----------|-------------|------------|----------|
| R1/4 | 162 | 486 | 75% | Extreme conditions (auroral, polar) |
| R1/2 | 324 | 324 | 50% | Poor to moderate conditions |
| R2/3 | 432 | 216 | 33% | Good conditions |
| R3/4 | 486 | 162 | 25% | Very good conditions |
| R5/6 | 540 | 108 | 17% | Excellent conditions |

### Speed Profiles

| Profile | Modulation | Code Rate | CP Mode | Target Conditions |
|---------|------------|-----------|---------|-------------------|
| Conservative | QPSK | R1/2 | Long (64) | Poor HF, high multipath |
| Balanced | 64-QAM | R3/4 | Medium (48) | Typical mid-latitude |
| Turbo | 256-QAM | R5/6 | Short (32) | Excellent NVIS/groundwave |

---

## Signal Processing Architecture

### Transmit Chain

```
Data → LDPC Encode → Interleave → QAM Map → OFDM Modulate → Upconvert → Audio Out
         ↓              ↓           ↓            ↓              ↓
    Add parity    Spread bursts  Complex    Add pilots     Mix to 1500 Hz
    (systematic)   (24×27 block) symbols    + IFFT + CP    center freq
```

### Receive Chain

```
Audio In → Downconvert → Sync Detect → CFO Correct → OFDM Demod → Equalize
              ↓              ↓             ↓             ↓           ↓
         Mix to baseband  Schmidl-Cox   Phase rotation  FFT      Channel est
         (NCO at 1500 Hz) correlation   (NCO tracking)  + CP rem  from pilots

         → Soft Demap → Deinterleave → LDPC Decode → Data Out
              ↓              ↓              ↓
           Compute LLRs   Reverse spread  Belief prop
           (noise-aware)   (24×27 block)  iteration
```

### Synchronization

The modem uses a modified **Schmidl-Cox** algorithm for preamble detection with several HF-specific enhancements:

1. **Preamble Structure**: 4 Short Training Symbols (STS) + 2 Long Training Symbols (LTS)
   - STS: Repeated Zadoff-Chu sequence for timing sync
   - LTS: Known pattern for initial channel estimation

2. **CFO-Tolerant Correlation**: Uses FFT-based Hilbert transform to create analytic signals
   ```
   z(t) = s(t) + j·HT(s(t))  // Analytic signal
   P = Σ conj(z[n]) · z[n+L]  // Complex correlation
   |P|/R ≈ 1.0 for matching symbols (independent of carrier frequency)
   ```
   This allows sync detection even with ±40 Hz frequency offset.

3. **Coarse CFO Estimation**: From preamble correlation phase
   ```
   Δf = arg(P) / (2π · T_symbol)
   ```

4. **Fine CFO Tracking**: Pilot phase differences across symbols
   ```
   Δf_fine = Δφ_pilots / (2π · T_symbol)
   ```
   Exponentially smoothed (α=0.3) to reduce noise.

### Channel Estimation & Equalization

- **Pilot-based LS estimation**: H = Rx_pilot / Tx_pilot
- **Interpolation**: Linear between pilot carriers
- **Smoothing**: α=0.9 for fast adaptation to channel changes
- **SNR estimation**: From pilot noise variance

### LDPC Implementation

The LDPC codes use a **systematic** structure with H = [H_data | I]:
- Encoding: parity = H_data × info (mod 2) — O(n) complexity
- Decoding: Min-Sum belief propagation with 0.75 scaling factor

**Matrix Construction** (PEG-like algorithm):
- Target check node degree: ~4 (regardless of rate)
- Variable node degree: Scaled by rate (higher for low-rate codes)
- Ensures good distance properties and decoding performance

---

## Performance

### Test Environment

Performance measured using the **ITU-R F.1487 Watterson HF channel model** with CCIR standard conditions:

| Condition | Delay Spread | Doppler | SNR | Description |
|-----------|--------------|---------|-----|-------------|
| AWGN | — | — | 20 dB | Ideal (baseline) |
| Good | 0.5 ms | 0.1 Hz | 20 dB | Quiet mid-latitude NVIS |
| Moderate | 1.0 ms | 0.5 Hz | 15 dB | Typical mid-latitude DX |
| Poor | 2.0 ms | 1.0 Hz | 10 dB | Disturbed/high-latitude |
| Flutter | 4.0 ms | 2.0 Hz | 8 dB | Auroral/polar paths |

### Measured Performance by Mode

| Mode | AWGN | Good | Moderate | Poor | Flutter |
|------|------|------|----------|------|---------|
| **BPSK R1/4** | 100% | 100% | 100% | 100% | **100%** |
| BPSK R1/2 | 100% | 100% | 100% | 100% | 69% |
| QPSK R1/2 | 100% | 100% | 100% | 25% | — |
| QPSK R3/4 | 100% | 100% | 100% | — | — |
| 16-QAM R1/2 | 100% | 100% | 100% | — | — |
| 16-QAM R3/4 | 100% | 100% | 100% | — | — |
| 64-QAM R3/4 | 100% | 100% | 100% | — | — |
| 256-QAM R3/4 | 100% | 81% | — | — | — |

*"—" = Mode not viable (<20% success), requires fallback to lower mode*
*BPSK R1/4 is the only mode that achieves 100% under Flutter conditions*

### Throughput Summary

| Mode | Throughput | Best Use Case |
|------|------------|---------------|
| BPSK R1/4 | 0.5 kbps | Extreme flutter/auroral |
| BPSK R1/2 | 1.1 kbps | Poor conditions |
| QPSK R1/2 | 2.1 kbps | Moderate conditions |
| QPSK R3/4 | 3.2 kbps | Good conditions |
| 16-QAM R3/4 | 6.4 kbps | Very good conditions |
| 64-QAM R3/4 | 9.6 kbps | Excellent NVIS |
| 256-QAM R3/4 | 12.8 kbps | Near-ideal only |

### Frequency Offset Tolerance

| Offset | Sync | Decode | Notes |
|--------|------|--------|-------|
| 0 Hz | YES | 100% | Baseline |
| ±5 Hz | YES | 100% | — |
| ±10 Hz | YES | 100% | — |
| ±15 Hz | YES | 100% | — |
| ±20 Hz | YES | 100% | Maximum tested |

The modem tolerates **±20 Hz** frequency offset with 100% decode success, covering typical HF radio drift without external AFC.

### Comparison to Other HF Modes

| Mode | Typical Throughput | Peak Throughput |
|------|-------------------|-----------------|
| VARA HF | 2-4 kbps | 8.5 kbps |
| PACTOR IV | 3-5 kbps | 10.5 kbps |
| ARDOP | 1-2 kbps | 2.4 kbps |
| **ProjectUltra** | 2-5 kbps | 10+ kbps |

---

## Building

### Prerequisites

- CMake 3.16+
- C++20 compiler (GCC 10+, Clang 12+, MSVC 2019+)
- SDL2 (for GUI)
- OpenGL 2.1+ (for GUI)

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
# Install vcpkg and SDL2
vcpkg install sdl2:x64-windows
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg-root]/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

## Usage

### GUI Application

```bash
./ultra_gui
```

**Modes:**
- **TEST**: Software loopback with simulated channel (adjustable SNR)
- **OPERATE**: Real audio I/O for over-the-air operation

### Simulation Test

```bash
./ultra_sim              # Full test suite
./ultra_sim --quick      # Quick performance test
```

Runs performance tests through the Watterson HF channel model with CCIR standard conditions.

### Unit Tests

```bash
./ultra_tests            # Run all unit tests
```

---

## Project Structure

```
ProjectUltra/
├── include/ultra/           # Public headers
│   ├── types.hpp            # Core types, ModemConfig, enums
│   ├── ofdm.hpp             # OFDMModulator, OFDMDemodulator
│   ├── fec.hpp              # LDPCEncoder, LDPCDecoder, Interleaver
│   ├── dsp.hpp              # FFT, NCO, HilbertTransform, filters
│   └── logging.hpp          # Logging infrastructure
├── src/
│   ├── ofdm/
│   │   ├── modulator.cpp    # OFDM TX: QAM mapping, IFFT, CP
│   │   └── demodulator.cpp  # OFDM RX: sync, FFT, equalization
│   ├── fec/
│   │   ├── ldpc_encoder.cpp # Systematic LDPC encoding
│   │   └── ldpc_decoder.cpp # Min-Sum BP decoding
│   ├── dsp/
│   │   ├── fft.cpp          # Radix-2 FFT implementation
│   │   └── filters.cpp      # NCO, Hilbert transform
│   ├── gui/
│   │   ├── app.cpp          # Main application
│   │   ├── modem_engine.cpp # TX/RX processing
│   │   └── audio_engine.cpp # PortAudio interface
│   ├── protocol/
│   │   └── arq_engine.cpp   # ARQ state machine
│   └── sim/
│       ├── hf_channel.hpp   # Watterson channel model
│       └── loopback_test.cpp # Performance benchmarks
├── thirdparty/
│   └── imgui/               # Dear ImGui (bundled)
└── tests/                   # Unit tests
    ├── test_ofdm.cpp
    ├── test_ldpc.cpp
    └── test_freq_offset.cpp
```

---

## Implementation Notes

### Key Design Decisions

1. **Systematic LDPC**: Allows O(n) encoding with H = [H_data | I] structure
2. **Block interleaver (24×27)**: Matches LDPC block size, spreads burst errors
3. **Analytic signal correlation**: Enables CFO-tolerant sync using Hilbert transform
4. **Min-Sum decoding**: Good performance with lower complexity than Sum-Product
5. **Scattered pilots**: Track channel variations across frequency and time

### Known Limitations

- **R1/4 only mode viable under Flutter**: Higher modes require Good+ conditions
- **256-QAM sensitive to fading**: Only reliable under AWGN or near-ideal conditions
- **ARQ not extensively tested**: Protocol layer needs more real-world validation

### Future Improvements

- [ ] Soundcard loopback testing mode
- [ ] Waterfall/spectrum display
- [ ] File transfer mode with progress
- [ ] Compression support (zlib/lz4)
- [ ] Band-specific optimization presets
- [ ] Improved GUI diagnostics

---

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions welcome! Please open an issue or pull request.

## Acknowledgments

- Dear ImGui by Omar Cornut
- SDL2 by Sam Lantinga
- ITU-R F.1487 for the Watterson channel model
- Inspired by VARA, JS8Call, ARDOP, and other HF digital modes
