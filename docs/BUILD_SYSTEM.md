# Build System

**Purpose:** Document build configuration, dependencies, and how to add new components.

---

## Quick Start

```bash
# Linux/macOS
mkdir build && cd build
cmake ..
make -j4

# With GUI
sudo apt install libsdl2-dev libgl1-mesa-dev  # Linux
brew install sdl2                              # macOS

# With FFTW optimization
sudo apt install libfftw3-dev                  # Linux
brew install fftw                              # macOS
```

---

## CMake Configuration

**Minimum Version:** 3.16
**C++ Standard:** C++20 (required)

### Build Options

| Option | Default | Purpose |
|--------|---------|---------|
| `ULTRA_BUILD_TESTS` | ON | Enable unit tests |
| `ULTRA_BUILD_TOOLS` | ON | Enable CLI tools |
| `ULTRA_BUILD_GUI` | ON | Enable GUI (needs SDL2+OpenGL) |
| `ULTRA_USE_FFTW` | ON | Use FFTW3 for FFT (if found) |

### Example Custom Build
```bash
cmake .. -DULTRA_BUILD_GUI=OFF -DULTRA_BUILD_TESTS=OFF
make -j4
# Builds only: ultra, cli_simulator, test tools
```

---

## Dependencies

### Required
| Dependency | Purpose | Detection |
|------------|---------|-----------|
| Threads | Multi-threading | `find_package(Threads REQUIRED)` |
| C++20 STL | Core library | Implicit |

### Optional
| Dependency | Purpose | Detection | Fallback |
|------------|---------|-----------|----------|
| FFTW3 | FFT acceleration | PkgConfig | Built-in Kiss FFT |
| SDL2 | Audio/video | CMake or PkgConfig | GUI disabled |
| OpenGL | Rendering | CMake | GUI disabled |

### Vendored (in thirdparty/)
| Library | Purpose |
|---------|---------|
| Dear ImGui | GUI framework |
| Miniz | ZIP compression |

---

## Build Targets

### Core Library
**Target:** `ultra_core` (static library)

**Source directories:**
- `src/ofdm/` - OFDM modulation
- `src/otfs/` - OTFS modulation
- `src/fec/` - LDPC encoding/decoding
- `src/dsp/` - FFT, filters, resampling
- `src/protocol/` - Protocol v2
- `src/waveform/` - IWaveform implementations
- `src/psk/` - DPSK implementations
- `src/sync/` - Chirp sync
- `thirdparty/miniz/` - Compression

### Executables

| Target | Source | Dependencies | Purpose |
|--------|--------|--------------|---------|
| `ultra` | main.cpp + modem_engine | ultra_core | CLI tool |
| `ultra_gui` | gui/*.cpp + imgui | ultra_core, SDL2, OpenGL | GUI app |
| `cli_simulator` | cli_simulator.cpp | ultra_core | Protocol simulator |

### Test Tools (23 executables)

| Tool | Purpose |
|------|---------|
| `test_iwaveform` | IWaveform interface testing |
| `test_hf_modem` | Full pipeline test (legacy) |
| `test_mc_dpsk` | MC-DPSK mode testing |
| `profile_acquisition` | Sync timing profiler |
| `test_chirp_*` | Chirp detection tests |
| ... | See CMakeLists.txt for full list |

### Unit Tests (23 tests in tests/)

Run with: `cd build && ctest --verbose`

---

## Header Organization

### Public API (include/ultra/)
```
include/ultra/
├── types.hpp          - Core types (Modulation, CodeRate, etc.)
├── ofdm.hpp           - OFDM API
├── dsp.hpp            - DSP utilities
├── fec.hpp            - FEC interface
├── arq.hpp            - ARQ types
├── otfs.hpp           - OTFS API
├── modem.hpp          - Top-level modem
└── logging.hpp        - Logging
```

### Internal Headers (src/)
```
src/
├── ofdm/demodulator_impl.hpp
├── psk/dpsk.hpp
├── psk/multi_carrier_dpsk.hpp
├── sync/chirp_sync.hpp
├── waveform/waveform_interface.hpp
├── protocol/frame_v2.hpp
└── ...
```

### Include Paths
```cmake
target_include_directories(ultra_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include    # Public API
    ${CMAKE_CURRENT_SOURCE_DIR}/src        # Internal
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty # Vendored
)
```

---

## Platform-Specific Notes

### Linux
```bash
sudo apt install build-essential cmake
sudo apt install libfftw3-dev           # Optional FFT
sudo apt install libsdl2-dev libgl1-mesa-dev  # For GUI
```

### macOS
```bash
brew install cmake fftw sdl2
# OpenGL is built-in
```

### Windows (MSVC)
- Compiler: MSVC 2019+ (C++20 required)
- Flags: `/MP` (parallel), `/utf-8`, warning suppressions
- SDL2/OpenGL: Use vcpkg or manual setup

### Raspberry Pi
```bash
sudo apt install build-essential cmake libfftw3-dev
# For GUI:
sudo apt install libsdl2-dev libgl1-mesa-dev
make -j3  # Use -j3 to avoid memory pressure
```

**Note:** No x86-specific SIMD - fully portable code.

---

## Adding New Components

### New Source File to ultra_core

1. Create file: `src/mymodule/myfile.cpp`
2. Add to CMakeLists.txt:
   ```cmake
   add_library(ultra_core STATIC
       # ... existing files ...
       src/mymodule/myfile.cpp
   )
   ```
3. Create header in `src/mymodule/myfile.hpp` (internal)
   or `include/ultra/myfile.hpp` (public API)

### New Test Tool

```cmake
if(ULTRA_BUILD_TOOLS)
    add_executable(test_myfeature tools/test_myfeature.cpp)
    target_include_directories(test_myfeature PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_myfeature PRIVATE ultra_core)
endif()
```

### New Unit Test

In `tests/CMakeLists.txt`:
```cmake
add_executable(test_myfeature test_myfeature.cpp)
target_link_libraries(test_myfeature PRIVATE ultra_core)
add_test(NAME MyFeature COMMAND test_myfeature)
```

### New External Dependency

```cmake
# Required dependency
find_package(NewLib REQUIRED)
target_link_libraries(ultra_core PUBLIC NewLib::NewLib)

# Optional dependency
find_package(OptLib)
if(OptLib_FOUND)
    add_compile_definitions(ULTRA_HAS_OPTLIB)
    target_link_libraries(ultra_core PUBLIC OptLib::OptLib)
endif()
```

### New Vendored Library

1. Place in `thirdparty/mylib/`
2. Add to CMakeLists.txt:
   ```cmake
   add_library(ultra_core STATIC
       # ... existing files ...
       thirdparty/mylib/mylib.c
   )
   ```

---

## Build Output

After `cmake .. && make`:

```
build/
├── libultra_core.a      # Core library
├── ultra                 # CLI tool
├── ultra_gui             # GUI app (if SDL2+OpenGL)
├── cli_simulator         # Protocol simulator
├── test_iwaveform        # Primary test tool
├── test_hf_modem         # Legacy test
├── ... (more test tools)
├── compile_commands.json # IDE integration
└── tests/
    ├── test_layers
    ├── test_protocol_modem
    └── ... (unit tests)
```

---

## Dependency Graph

```
ultra_core
├── Threads::Threads
├── PkgConfig::FFTW3 (optional)
└── miniz (vendored)

ultra
├── ultra_core
└── modem_engine files

ultra_gui
├── ultra_core
├── SDL2::SDL2
├── OpenGL::GL
└── ImGui (vendored)

test_iwaveform
└── ultra_core
```

---

## Compile Commands Database

Generated: `build/compile_commands.json`

Used by IDEs (VSCode, CLion) for:
- Code completion
- Error highlighting
- Go to definition

Enabled by: `CMAKE_EXPORT_COMPILE_COMMANDS ON`

---

## Troubleshooting

### "SDL2 not found"
```bash
# Linux
sudo apt install libsdl2-dev

# macOS
brew install sdl2

# Or disable GUI
cmake .. -DULTRA_BUILD_GUI=OFF
```

### "FFTW not found"
Not critical - falls back to built-in FFT. For optimization:
```bash
sudo apt install libfftw3-dev
```

### "C++20 required"
Update compiler:
```bash
# GCC
sudo apt install g++-10
export CXX=g++-10

# Or Clang
sudo apt install clang-12
export CXX=clang++-12
```

### Raspberry Pi out of memory
```bash
make -j2  # Use fewer parallel jobs
# Or add swap space
```
