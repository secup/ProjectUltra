# ProjectUltra

A high-performance HF sound modem designed for amateur radio data transmission over voice channels. Features adaptive modulation, LDPC forward error correction, and a real-time GUI with constellation diagram.

## Features

- **OFDM modulation** with 30 carriers in 2.8 kHz bandwidth
- **Adaptive modulation**: BPSK, QPSK, 16-QAM, 64-QAM, 256-QAM
- **LDPC FEC** with multiple code rates (1/4 to 7/8)
- **Throughput**: 1-16 kbps depending on channel conditions
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

| Modulation | Bits/Symbol | Min SNR | Typical Throughput |
|------------|-------------|---------|-------------------|
| BPSK | 1 | -3 dB | ~1 kbps |
| QPSK | 2 | 3 dB | ~2 kbps |
| 16-QAM | 4 | 10 dB | ~4 kbps |
| 64-QAM | 6 | 16 dB | ~8 kbps |
| 256-QAM | 8 | 24 dB | ~16 kbps |

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

Runs a full loopback test through the OFDM chain with simulated HF channel.

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
│   └── sim/             # Simulation/test tools
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
