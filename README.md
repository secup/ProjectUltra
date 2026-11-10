# ProjectUltra

**An adaptive HF data modem for amateur radio.**

[![Build Matrix](https://github.com/secup/ProjectUltra/actions/workflows/build-matrix.yml/badge.svg?branch=main)](https://github.com/secup/ProjectUltra/actions/workflows/build-matrix.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: Pre-alpha](https://img.shields.io/badge/Status-Pre--alpha-orange.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg)]()

ProjectUltra moves files and messages byte-exact between two HF stations over SSB. It picks its
waveform, modulation and code rate from the channel it measures, and keeps adapting as
conditions change. Use it headless as a TCP TNC that speaks the command/data interface of
existing HF data clients, or as a desktop application with a waterfall and a constellation
view.

> **Pre-alpha.** Under active development. The over-the-air protocol can change between
> versions, so both stations must run the same build. Performance is measured on software and
> hardware HF channel simulators; it is not yet characterized over the air at scale.

*Updated 2026-11-10. Latest release: [`v0.6.1-pre-alpha`](https://github.com/secup/ProjectUltra/releases/tag/v0.6.1-pre-alpha).*

---

## Highlights

- **Adaptive wideband OFDM.** 59 carriers in 2.8 kHz, coherent QPSK through 16QAM. The receiver
  learns the link quality from frame outcomes and from each frame's measured signal quality,
  and moves to a faster rung as soon as the frames say it would decode.
- **Strong coding.** Each OFDM data frame is one long turbo codeword; control frames use LDPC.
- **Efficient ARQ.** Selective repeat over multi-frame bursts, soft combining of repeated frames,
  and short tone-burst acknowledgements.
- **Low overhead.** A sync chirp opens the connection; after that each burst is found from its
  own structure. The receiver tracks the dial offset and the soundcard clock offset between the
  two stations.
- **Low-SNR and crowded-band modes.** MC-DPSK carries the link down to about 5 dB SNR and
  through heavy multipath; a 500 Hz narrowband OFDM mode fits crowded bands.
- **Interference resilience.** Per-carrier erasure turns deep notches and in-band QRM into
  recoverable erasures instead of lost frames.
- **TNC and GUI.** `ultra_tnc` serves the TCP command (8300) and data (8301) ports used by HF
  data clients; `ultra_gui` adds a waterfall, constellation, message log and ARQ health view.
- **Cross-platform.** Linux, macOS and Windows, including the Raspberry Pi 5.

---

## Performance

All figures are goodput: file bytes delivered and CRC-verified per second of transfer, with
retransmissions and turnarounds included. Connection setup (about 14 s on `main`, 24 s in v0.6.1) is
not included.

### v0.6.0, software channel simulator

Two complete stations over OTASim (ITU-R F.1487 Watterson fading, 7.3 Hz dial offset between the
radios), full protocol in real time. 21 KB file, mean of 5 seeds:

| Channel | 12 dB | 15 dB | 20 dB | 25 dB |
|---|---:|---:|---:|---:|
| AWGN | 2.9 kbps | — | 3.6 kbps | — |
| ITU Good (0.1 Hz / 0.5 ms) | 1.7 kbps | 2.6 kbps | 2.9 kbps | 3.1 kbps |
| ITU Moderate (0.5 Hz / 1 ms) | — | 1.7 kbps | 2.1 kbps | 2.5 kbps |

A 50 KB file on ITU Good at 20 dB averages 3.3 kbps (2.9–3.8 across seeds). On AWGN at 6–7 dB
the link runs OFDM at 1.0–1.2 kbps. Measured on v0.6.0; v0.6.1 leaves these figures unchanged.

With a real station's transmit levels (every burst scaled to the same peak power, as a real
transmitter does), v0.6.1 sends OFDM data about 2.4 dB louder than v0.6.0 at the same peak. On
AWGN in the simulator, the link now runs OFDM at 770–970 bps from 15 dB on the simulator's scale.

### Previous release, hardware channel simulator

`v0.5.2.1-pre-alpha` between a Raspberry Pi 5 and a Mac through a hardware HF channel simulator,
real soundcards and real half-duplex turnaround (August 2026). 50 KB on ITU Good at 20 dB:
16 of 16 transfers byte-exact, mean 1.98 kbps with the rung held at 8PSK R2/3, 1.50–2.32 kbps
under automatic rate control. v0.6.0 and v0.6.1 have not yet been measured on hardware.

Fading channels vary a lot from run to run: identical builds on the same path have differed by
0.7 kbps. Compare distributions, not best runs. No claim is made about performance relative to
other HF data modems; that needs paired on-air testing.

### Raw PHY rates (wideband OFDM)

`data_carriers × bits_per_symbol × 41.667 symbols/s × code_rate`, before any protocol overhead:

| Rung | Data carriers | Raw rate | Status |
|---|---:|---:|---|
| QPSK R1/4 | 47 | 979 bps | entry rung |
| QPSK R1/2 | 51 | 2125 bps | enabled |
| QPSK R2/3 | 51 | 2833 bps | enabled |
| QPSK R3/4 | 51 | 3188 bps | enabled |
| 8PSK R2/3 | 51 | 4250 bps | enabled |
| 16QAM R2/3 | 51 | 5667 bps | enabled (strong channels) |
| 8PSK R3/4, 16QAM R1/2, R3/4 | 51 | 4250–6375 bps | disabled: did not hold on fading |

MC-DPSK runs 47–375 bps; narrowband OFDM 193–386 bps.

---

## Getting started

### Release bundles

Download `projectultra-<platform>.zip` from the
[GitHub releases](https://github.com/secup/ProjectUltra/releases): `ultra_tnc`, `ultra_gui`,
`ultra`, an example TNC configuration and the operator docs. Simulator and bench tools ship
separately as `dev-tools-<platform>.zip`.

On macOS, downloaded binaries are quarantined. Clear the flag once per binary
(`xattr -d com.apple.quarantine ./ultra_gui`) or open it from Finder with right-click → Open.

### Build from source

Requires CMake 3.16+, a C++20 compiler (GCC 10+, Clang 12+, MSVC 2019+), SDL2, and gRPC with
Protobuf.

```bash
# Ubuntu / Debian
sudo apt install libsdl2-dev cmake build-essential pkg-config \
  libgrpc++-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc \
  libabsl-dev libc-ares-dev libssl-dev libre2-dev zlib1g-dev

# macOS
brew install sdl2 cmake pkg-config grpc protobuf abseil c-ares

git clone https://github.com/secup/ProjectUltra.git
cd ProjectUltra
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
```

On Windows, install the dependencies with vcpkg (`vcpkg install sdl2 grpc`) and pass its
toolchain: `cmake -S . -B build -A x64 -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"`,
then `cmake --build build --config Release`.

### Run

```bash
# Headless TNC for an HF data client
./build/ultra_tnc --audio-output "USB Audio Device" --audio-input "USB Audio Device" --callsign N0CALL

# Desktop application
./build/ultra_gui
./build/ultra_gui -sim      # simulator mode: no radio needed
```

The operator guide is [`docs/RUNNING.md`](docs/RUNNING.md).

---

## TNC interface

`ultra_tnc` exposes the modem on TCP port 8300 (commands) and 8301 (data). Clients that drive an
HF data modem over this interface connect to it unchanged.

```bash
printf 'VERSION\r' | nc 127.0.0.1 8300
```

Supported: `VERSION`, `MYCALL`, `LISTEN`, `CONNECT`, `DISCONNECT`, `ABORT`, `BW500` / `BW2300` /
`BW2750`, `BUFFER`, `SN`, `BITRATE`, `COMPRESSION`, `CHAT`, `CWID`, and the common client
compatibility no-ops. ProjectUltra adds `STATS`, a one-line ARQ and PHY snapshot for diagnosing a
stalled session. Full reference: [`docs/TNC_INTERFACE.md`](docs/TNC_INTERFACE.md).

Validated end to end with a real HF mail client between two machines over audio: complete
message-exchange sessions, text and binary attachments, both directions. Known limitation: a new
session started within about 30 s of the previous one does not always connect on the first try.

---

## Radio setup

- SSB transceiver with a 2.8 kHz or wider filter (500 Hz for the narrowband mode).
- Audio interface (USB soundcard, SignaLink, RigBlaster or a direct cable) and PTT by VOX, CAT or
  hardware line.
- TX audio clean, without ALC compression; RX audio with peaks below 0.9 of full scale.

Suggested frequencies (USB, 2.8 kHz):

| Band | Frequency | Notes |
|---|---:|---|
| 80 m | 3.590 MHz | above narrow digital, below voice |
| 40 m | 7.102 MHz | common for wideband digital |
| 30 m | 10.145 MHz | watch WSPR at 10.140 |
| 20 m | 14.108 MHz | above the FT8 segment |
| 15 m | 21.110 MHz | above the narrow digital segment |
| 10 m | 28.120 MHz | plenty of room |

Avoid 14.070–14.095 MHz (FT8/PSK31), any .074 MHz, and 14.100 MHz (NCDXF beacons). Listen
before transmitting, use the minimum power needed, and be ready to move.

---

## How it works

1. **PING / PONG.** A short presence probe before committing to a connection.
2. **CONNECT.** Callsign exchange over MC-DPSK, with each side's measured SNR and fading.
3. **MODE CHANGE.** Both stations switch to the waveform and rate the channel supports: wideband
   OFDM from 6 dB on a clean channel (8 dB on Good fading, 14 dB on Moderate), MC-DPSK below
   that or on heavy multipath, narrowband OFDM on a 500 Hz channel.
4. **DATA.** Bursts of frames with selective-repeat ARQ. The receiver commands the rate for the
   next burst from what it measured on this one.
5. **DISCONNECT.** Graceful teardown with callsign identification.

| Parameter | MC-DPSK | Wideband OFDM | Narrowband OFDM |
|---|---:|---:|---:|
| Bandwidth | ~2.4 kHz | 2.8 kHz | 500 Hz |
| Carriers | 8 | 59 | 21 |
| FFT size | — | 1024 | 2048 |
| Cyclic prefix | — | 128 samples (2.67 ms) | 192 samples (4.0 ms) |
| Modulation | DBPSK / DQPSK / D8PSK | QPSK, 8PSK, 16QAM | QPSK |
| Sample rate | 48 kHz | 48 kHz | 48 kHz |

---

## Documentation

| | |
|---|---|
| [`docs/RUNNING.md`](docs/RUNNING.md) | operator guide |
| [`docs/TNC_INTERFACE.md`](docs/TNC_INTERFACE.md) | TNC command reference |
| [`docs/PROTOCOL_V2.md`](docs/PROTOCOL_V2.md) | protocol and frame formats |
| [`docs/TESTING.md`](docs/TESTING.md) | test gates, simulators and measurement rules |
| [`docs/PERFORMANCE_HISTORY.md`](docs/PERFORMANCE_HISTORY.md) | measurement archive |
| [`docs/CHANGELOG.md`](docs/CHANGELOG.md), [`docs/KNOWN_BUGS.md`](docs/KNOWN_BUGS.md) | change log and open issues |

---

## Roadmap

- Validate v0.6.1 on the hardware channel simulator (both stations on the same build).
- Shorten connection setup further (about 14 s on `main`; the CONNECT/CONNECT_ACK exchange is
  most of it).
- Leaner frame headers, which matter most at the lowest rung.
- Broader two-way over-the-air testing, especially at low SNR.

---

## Contributing

Contributions are welcome. Good starting points: on-air test reports (include the `STATS`
output), reports from HF data clients about what worked and what did not, bug fixes, DSP
optimizations (profile first), and documentation. Please open an issue before a large pull
request.

**Real HF recordings help.** Simulation cannot capture everything the ionosphere does, and even
a failed transfer is useful. Record your own signal from a WebSDR or KiwiSDR tuned to your TX
frequency, and attach it to a GitHub issue with the label "Recording", your callsign, location,
band, frequency, UTC time, the receiver you used and the path distance.

---

## License

MIT. See [LICENSE](LICENSE).

## Acknowledgments

- Community on-air testers, especially **KC3VPB**, for real-station logs that helped diagnose
  sync and buffer edge cases.
- [Dear ImGui](https://github.com/ocornut/imgui) (GUI), [SDL2](https://libsdl.org/) (audio and
  windowing), [PocketFFT](https://github.com/mreineck/pocketfft) (FFT),
  [miniz](https://github.com/richgel999/miniz) (compression).
