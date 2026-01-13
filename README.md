# ProjectUltra

A high-performance HF sound modem designed for amateur radio data transmission over voice channels. Features adaptive modulation, LDPC forward error correction, and a real-time GUI with constellation diagram.

## Features

- **OFDM modulation** with 30 carriers in 2.8 kHz bandwidth
- **Adaptive modulation**: BPSK, QPSK, 16-QAM, 64-QAM, 256-QAM
- **LDPC FEC** with multiple code rates (1/4 to 7/8)
- **Realistic throughput**: 2-10 kbps typical (see Performance section)
- **Watterson HF channel simulator** for realistic testing
- **Cross-platform GUI** using Dear ImGui + SDL2 + OpenGL 2.1
- **Real-time constellation diagram** visualization

## Specifications

### Physical Layer

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48 kHz |
| Center Frequency | 1500 Hz |
| Bandwidth | 2812 Hz |
| FFT Size | 512 |
| Subcarrier Spacing | 93.75 Hz |
| Total Carriers | 30 (25 data + 5 pilots) |
| Symbol Duration | 10.7-11.8 ms (depending on CP) |
| Cyclic Prefix | 32/48/64 samples (adaptive) |

### Modulation Schemes

| Modulation | Bits/Symbol | Min SNR (AWGN) | Raw Throughput |
|------------|-------------|----------------|----------------|
| BPSK | 1 | ~3 dB | 1.1 kbps |
| QPSK | 2 | ~6 dB | 2.1 kbps |
| 16-QAM | 4 | ~12 dB | 4.3 kbps |
| 64-QAM | 6 | ~18 dB | 6.4 kbps |
| 256-QAM | 8 | ~24 dB | 12.8 kbps |

*Note: Real HF channels require 5-10 dB higher SNR due to fading and multipath.*

### Forward Error Correction

- **Code**: LDPC (Low-Density Parity-Check)
- **Block Size**: 648 bits
- **Code Rates**: 1/4, 1/3, 1/2, 2/3, 3/4, 5/6, 7/8
- **Decoder**: Belief propagation with early termination

### Speed Profiles

| Profile | Modulation | Code Rate | CP Mode | Use Case |
|---------|------------|-----------|---------|----------|
| Conservative | QPSK | 1/2 | Long | Poor HF conditions |
| Balanced | 64-QAM | 3/4 | Medium | Typical HF |
| Turbo | 256-QAM | 7/8 | Short | Excellent conditions |

## Performance

### Realistic HF Channel Simulation

Performance tested using the ITU-R F.1487 Watterson HF channel model with CCIR standard conditions:

| Condition | Delay Spread | Doppler | SNR | Description |
|-----------|--------------|---------|-----|-------------|
| AWGN | - | - | 20 dB | Ideal (baseline) |
| Good | 0.5 ms | 0.1 Hz | 20 dB | Quiet mid-latitude |
| Moderate | 1.0 ms | 0.5 Hz | 15 dB | Typical mid-latitude |
| Poor | 2.0 ms | 1.0 Hz | 10 dB | Disturbed/high-latitude |
| Flutter | 4.0 ms | 2.0 Hz | 8 dB | Auroral/polar paths |

### Measured Throughput by Mode

| Mode | AWGN | Good | Moderate | Poor |
|------|------|------|----------|------|
| BPSK R1/2 | 1.1 kbps | 1.1 kbps | 1.1 kbps | 1.0 kbps |
| QPSK R1/2 | 2.1 kbps | 2.1 kbps | 2.1 kbps | — |
| QPSK R3/4 | 3.2 kbps | 3.2 kbps | 3.2 kbps | — |
| 16-QAM R3/4 | 6.4 kbps | 6.4 kbps | 6.4 kbps | — |
| 64-QAM R3/4 | 9.6 kbps | 9.6 kbps | 9.6 kbps | — |
| 256-QAM R3/4 | 12.8 kbps | 12.8 kbps | 4.1 kbps | — |

*"—" indicates mode not viable for that condition (requires fallback to lower mode)*

### Expected Real-World Performance

| Channel Condition | Recommended Mode | Expected Throughput |
|-------------------|------------------|---------------------|
| Excellent (NVIS, quiet band) | 64-QAM R3/4 | 6-10 kbps |
| Good (typical mid-latitude) | 16-QAM R3/4 | 4-6 kbps |
| Moderate (average DX) | QPSK R3/4 | 2-4 kbps |
| Poor (disturbed, polar) | QPSK R1/2 | 1-2 kbps |
| Extreme (auroral flutter) | BPSK R1/2 | 0.5-1 kbps |

### Comparison with Other HF Modes

| Mode | Typical Throughput | Peak Throughput |
|------|-------------------|-----------------|
| VARA HF | 2-4 kbps | 8.5 kbps |
| PACTOR IV | 3-5 kbps | 10.5 kbps |
| ARDOP | 1-2 kbps | 2.4 kbps |
| **ProjectUltra** | **2-6 kbps** | **~10 kbps** |

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

### Simulation Test

```bash
./ultra_sim
```

Runs performance tests through a realistic Watterson HF channel model (ITU-R F.1487) with CCIR standard conditions. Tests all modulation modes against Good, Moderate, Poor, and Flutter fading conditions to measure realistic throughput.

## Project Structure

```
ProjectUltra/
├── include/ultra/       # Public headers
│   ├── types.hpp        # Core types and configuration
│   ├── ofdm.hpp         # OFDM modulator/demodulator
│   ├── fec.hpp          # LDPC encoder/decoder
│   ├── dsp.hpp          # FFT, NCO, filters
│   └── logging.hpp      # Logging infrastructure
├── src/
│   ├── ofdm/            # OFDM implementation
│   ├── fec/             # LDPC codec
│   ├── dsp/             # DSP primitives
│   ├── gui/             # GUI application
│   ├── protocol/        # ARQ protocol engine
│   └── sim/             # HF channel simulator
│       ├── hf_channel.hpp   # Watterson channel model
│       └── loopback_test.cpp # Performance testing
├── thirdparty/
│   └── imgui/           # Dear ImGui (bundled)
└── tests/               # Unit tests
```

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions welcome! Please open an issue or pull request.

## Acknowledgments

- Dear ImGui by Omar Cornut
- SDL2 by Sam Lantinga
- Inspired by various HF digital modes (VARA, JS8Call, etc.)
