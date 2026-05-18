# ProjectUltra

**High-performance HF modem for amateur radio**

*Last updated: 2026-05-07*

> **EXPERIMENTAL SOFTWARE — WORK IN PROGRESS**
>
> Active development. Not production-ready. APIs and protocols may change.
> Use at your own risk for experimentation and amateur-radio research.

ProjectUltra is a software modem for reliable HF data transfer. The normal
operator path is:

- **Headless TCP TNC** — `ultra_tnc` exposes the modem via the same TCP
  command/data API used by existing HF data clients. This is the primary
  on-air integration path on Linux, macOS, and Windows.
- **GUI application** — `ultra_gui` provides a local operator UI with
  waterfall, constellation, message log, and ARQ health view.

The same repo also contains the modem core and diagnostic / lab tools
(`cli_simulator`, `decode_bench`, `session_decode`, raw frame CLI). Those are
for validation, profiling, replay, and development; they are not the first-run
operator path.

[![Build Matrix](https://github.com/secup/ProjectUltra/actions/workflows/build-matrix.yml/badge.svg?branch=main)](https://github.com/secup/ProjectUltra/actions/workflows/build-matrix.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Status: Experimental](https://img.shields.io/badge/Status-Experimental-orange.svg)]()
[![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg)]()

---

## Performance

ProjectUltra exposes throughput at two layers. Both matter; they answer
different questions.

### Raw PHY (theoretical maximum)

Strict raw-PHY rate: `data_carriers × bits_per_symbol × symbol_rate ×
code_rate`. No subtraction for preamble, frame header, ARQ, or ACK
turnaround — that's the ceiling the modulator could feed downstream
on a steady-state channel.

Production geometry used below:

- MC-DPSK: 8 carriers, 512 samples/symbol → 93.75 sym/s, DQPSK,
  data-mode pinned to R1/4 by `recommendDataMode()`. Adaptive ladder
  extends down to -5 dB via DBPSK + slower symbol rates
  (`docs/CHANGELOG.md` 2026-05-10).
- OFDM-CHIRP wideband: 1024-FFT, LONG CP=128 → 1152 samples/symbol,
  41.667 sym/s. 59 occupied carriers; pilot count comes from
  `recommendedPilotSpacing(mod, rate)`.
- OFDM-NARROW: 21 occupied carriers, 2240 samples/symbol → 21.429 sym/s,
  pilot spacing 10 → 18 data carriers.

| SNR  | Mode                 | Data carriers | Raw PHY     | Notes |
|------|----------------------|--------------:|------------:|-------|
| -5+  | MC-DPSK adaptive ladder  |        8 | 47–375 bps  | DBPSK→DQPSK + variable symbol rate via auto-rung |
| 8+   | OFDM-NARROW DQPSK R1/4   |       18 |     193 bps | 500 Hz BW for crowded bands |
| 8+   | OFDM-NARROW DQPSK R1/2   |       18 |     386 bps | 500 Hz BW |
| 10+  | OFDM-CHIRP DQPSK R1/4    |       53 |    1104 bps | Fading-tolerant baseline (6 pilots @ spacing 10) |
| 15+  | OFDM-CHIRP DQPSK R1/2    |       53 |    2208 bps | Good + moderate fading (6 pilots @ spacing 10) |
| 15+  | OFDM-CHIRP DQPSK R2/3    |       53 |    2944 bps | Near-AWGN only (6 pilots @ spacing 10) |
| 15+  | OFDM-CHIRP DQPSK R3/4    |       55 |    3438 bps | AWGN-class channel only (4 pilots @ spacing 15) |
| 25+  | OFDM-CHIRP 16QAM R3/4    |       51 |    6375 bps | Stable paths (NVIS, ground wave; 8 pilots @ spacing 8) |
| 30+  | OFDM-CHIRP 32QAM R3/4    |       47 |    7344 bps | Stable paths only (12 pilots @ spacing 5) |

(2026-05-09: prior table mixed CP=MEDIUM/LONG arithmetic and inherited a
938 bps "MC-DPSK" constant from a 20-carrier preset that production
never selects. Numbers above are now derived directly from
`recommendedPilotSpacing()` and the production CP setting.)

### End-to-end measured (real hardware)

What a user actually sees over the cable / air, including handshake,
ACK roundtrips, retransmissions, and auto-rate downgrades. Auto-rate
ladder is on; Connection picks among R1/4 / R1/2 / R2/3 / R3/4 based
on measured SNR and fading.

| Test                          | Channel                | Wall   | Throughput      | Notes |
|-------------------------------|------------------------|-------:|----------------:|-------|
| 50 KB Mac↔Pi5 cable           | Clean USB cable        |  174 s |      2354 bps   | Auto DQPSK R3/4 @ SNR=28 — handshake ≈ 3% of wall |
| 20 KB Mac↔Pi5 injected ×5     | AWGN, SNR=15           |  ~94 s | **1736.5 bps**  | Median of 5; range [1730.1, 1739.8]; R1/2; 0 retx all 5; handshake ≈ 5% (near steady-state) |
| 20 KB Mac↔Pi5 injected ×5     | Watterson Good, SNR=15 |  ~94 s | **1733.1 bps**  | Median of 5; range [1726.8, 1739.6]; R1/2; 0 retx all 5; handshake ≈ 5% (near steady-state) |
| 5 KB Mac↔Pi5 injected ×5      | Watterson Good, SNR=15 |  ~27 s | **1540.6 bps**  | Median of 5; range [1540.3, 1550.0]; R1/2; 0 retx all 5; handshake ≈ 19% (fixed ~5 s overhead dominates short transfers) |
| 500 KB Mac↔Pi5 injected       | Watterson Good, SNR=15 | 3742 s |      1094 bps   | R1/2; 1346 retx, 0 failed, byte-exact; handshake negligible — slowdown vs 20 KB is from retransmissions on a long fading run |
| 1 KB Mac↔Pi5 injected         | Watterson Mod, SNR=0   |  235 s |    **34.9 bps** | MC-DPSK Robust-Mid (DBPSK R1/4) + continuous burst + ARQ-tuned window=3; 0 retx; below the OFDM SNR floor |
| 1 KB Mac↔Pi5 injected         | Watterson Good, SNR=5  |  101 s |    **81.0 bps** | MC-DPSK Robust (DQPSK R1/4 4-CW) + continuous burst + ARQ-tuned window=5; 0 retx; ladder above Robust-Mid |
| 1 KB Mac↔Pi5 injected         | Watterson Mod, SNR=-5  |  608 s |    **13.5 bps** | MC-DPSK Robust-Low (DBPSK 2048sps R1/4 3-CW); 0 retx; preamble-dominated, window=1 |

Throughput rises with payload size as the fixed ~5 s handshake (PING/PONG → CONNECT → MODE_CHANGE) amortizes. The 1.83-1.90 kbps wall-clock asymptote at R1/2 SNR=15 sits below the 2208 bps raw-PHY ceiling because the 8-CW frame carries 301 useful payload bytes inside 51 OFDM symbols (`2 LTS + ceil(8*648/(53*2))` = 1.224 s) — that effective single-frame payload rate is ~1967 bps before ARQ overhead, and ACK turnaround takes the rest. Beyond ~50 KB, throughput is bounded by per-frame airtime + ACK roundtrip, not handshake.

End-to-end throughput is wall-clock measured by `tools/run_hw_test.sh`
between A's `Connection: Starting file transfer` and the final ACK
received at A. Includes handshake + ACK turnaround latency (what an
operator actually waits for). The Mac↔Pi5 hardware harness is the
two-machine synthetic-channel test rig described in
`docs/AGENT_DEDICATED_ENV_MACOS.md`.

End-to-end results match or exceed real-world numbers reported for
existing commercial HF data modems in equivalent conditions. The 500 KB
Good15 result is the realistic-HF baseline; 50 KB cable is the upper
bound given a clean channel.

### Features

**Waveforms.** MC-DPSK (chirp sync, low-SNR robust), OFDM-CHIRP
(wideband 2.8 kHz, 59 carriers), OFDM-NARROW (500 Hz crowded-band
mode), OFDM-COX (Schmidl-Cox sync, forceable/legacy only),
SC-DPSK (very low SNR).

**Modulation + FEC.** DBPSK / DQPSK / D8PSK / BPSK / QPSK with
802.11n LDPC at four code rates (R1/4, R1/2, R2/3, R3/4).
Min-sum belief-propagation decoder.

**Synchronization.** Dual-chirp detection with PocketFFT-accelerated
correlation, Schmidl-Cox training, light-preamble (LTS-only) for
in-session frames, LTS-residual CFO refinement, per-symbol pilot
tracking with common-phase-error correction.

**ARQ.** Selective-repeat with cumulative + selective ACKs,
window 1 (DPSK / narrowband) or 8–16 (wideband OFDM), variable
1–8 codeword frames, wire-negotiated CW count, frame +
optional channel + burst interleavers.

**Adaptive rate.** SNR + fading-index ladder (R3/4 / R2/3 /
R1/2 / R1/4) with bootstrap cap, per-burst clean-window upgrade,
two-window hysteresis on downshift to prevent panic-downshift on
short fading transfers. Exact thresholds live in
`src/protocol/waveform_selection.hpp::selectOFDMCodeRate()`.

**Per-carrier RX erasure.** Each OFDM-CHIRP frame computes
`γ_k = |H_k|² / σ²_k` from its own LTS + pilots; carriers below
-6 dB emit `LLR = 0` to LDPC after a persistence gate (3
consecutive symbols or 2 consecutive multi-CW frames). Bits are
spread across LDPC base columns by the CarrierLDPC v1
interleaver `a = (307·i) mod (648·Ncw)`. Silent on AWGN /
light fading; converts deep stationary notches and in-band QRM
from a TEST FAILED into a clean decode (validated A/B on a
fixed -25 dB notched carrier: 2,271 bps vs 15-frame loss
baseline).

**HARQ Chase soft-combining.** LLR-accumulating buffer keyed by
full PHY digest (rate, modulation, interleaver, carrier mask,
erasure-policy epoch). Currently active only when the frame's
header CW decodes; broader CW0-fail integration is in
experimental branches awaiting further hardware validation.

**Channel testing.** Built-in Watterson HF channel injector
(ITU-R F.1487) with AWGN, Good (0.1 Hz / 0.5 ms), Moderate
(0.5 Hz / 1 ms), Poor (1 Hz / 2 ms) presets. Two-machine hardware
harness (Mac ↔ Pi5 over USB sound cards) with byte-exact
end-to-end validation.

**OTASim — network HF channel server.** Long-running daemon
(`ota_simulator serve`) that hosts the HF medium in software:
multiple clients join a session, audio flows over gRPC (control) +
UDP (samples), and the channel model is applied server-side.
`ultra_gui`, `ultra_tnc`, and `cli_simulator` can all run as OTASim
clients instead of opening a soundcard, so protocol/feature work
needs no cable rig and no audio hardware. The server holds a
session reference clock that bridges per-host audio-clock drift,
so cross-machine runs (e.g. Mac ↔ Pi5 ↔ Windows over LAN or VPN)
behave as a single timeline.

**Protocol v2.** PING / PONG, CONNECT / CONNECT_ACK,
MODE_CHANGE, DATA, ACK / SACK, DISCONNECT. Wire-level CRC-16
on every frame, capability flags, measured-SNR + fading-index
exchange.

**TNC integration.** `ultra_tnc` daemon exposes the modem over
the same TCP command/data API used by existing HF data clients
(cmd port 8300 / data port 8301); verified end-to-end with
real client sessions across all major B2F message types.

**GUI application.** `ultra_gui` with real-time waterfall,
constellation, message log, and ARQ health view (ImGui +
SDL2). Virtual-station / simulator mode for development.

### Waveform selection (automatic)

```
SNR         Waveform              Reason
─────────────────────────────────────────────────────────────────────
-5 to +9 dB MC-DPSK (auto-rung)   DBPSK→DQPSK + variable SPS, adaptive
5–10 dB     OFDM-NARROW (500 Hz)  Crowded bands, low SNR
10+ dB      OFDM-CHIRP (1024)     Production auto ladder
forced      OFDM-COX (1024)       Implemented, not auto-selected
forced      OFDM 16QAM            Coherent + pilot tracking
```

Selection happens during CONNECT (peer-advertised SNR + fading index)
and continues adapting during the session. See `docs/PROJECT_GOALS.md`
for the throughput/reliability targets driving this work.

---

## TNC Integration

`ultra_tnc` is a daemon that exposes ProjectUltra's modem through a
legacy-compatible HF TCP TNC command/data interface. Existing clients
can connect to it the same way they connect to a commercial HF data
modem, with no protocol changes on the client side.

```
┌──────────────┐  TCP 8300 (cmd)   ┌──────────────┐
│  HF data     │  TCP 8301 (data)  │  ultra_tnc   │  Audio   ┌─────────┐
│  client /    │ ◄──────────────► │  (modem +    │ ◄──────► │  HF     │
│  app         │                   │   TCP shell) │          │  Radio  │
└──────────────┘                   └──────────────┘          └─────────┘
```

### Quick start

```bash
# Build (see Getting Started below)
cmake -S . -B build && cmake --build build -j 4

# Listen on default ports 8300/8301
./build/ultra_tnc --audio-output "USB Audio Device" \
                  --audio-input  "USB Audio Device" \
                  --callsign     N0CALL

# Smoke test from another terminal
printf 'VERSION\r' | nc 127.0.0.1 8300
# -> VERSION 0.3.1
```

Full command reference: [`docs/TNC_INTERFACE.md`](docs/TNC_INTERFACE.md).

### Supported commands

Standard TNC shell: `VERSION`, `MYCALL`, `LISTEN`, `CONNECT`, `DISCONNECT`,
`ABORT`, `BW500` / `BW2300` / `BW2750`, `BUFFER`, `SN`, `BITRATE`,
`COMPRESSION`, `CHAT`, `CWID`, plus legacy-client compatibility
no-ops (`PUBLIC`, `P2P`, client-mode probes, `IGNOREKISSDCD`,
`RETRIES`, `CALLINT`).

ProjectUltra extension:

- **`STATS`** — single-line ARQ + PHY snapshot for debugging stalled
  sessions: `frames_sent`, `frames_recv`, `retx`, `timeouts`, `failed`,
  `out_of_order`, current `rate` / `mod` / `mode`, `snr`, `bps`,
  `backlog`. Existing clients ignore unknown commands, so this is safe
  to leave on.

### Status

- Cross-platform: Linux + macOS + Windows. CI matrix all green.
- ctest: **38/38** (TNC parser, TCP integration, bridge tests, throughput utility,
  decode-bench replay fixtures, plus
  modem regressions including the new `CarrierLDPC v1` math gate
  and per-carrier mask plumbing).
- End-to-end byte-exact transfers (hardware harness, Mac ↔ Pi5
  over USB sound cards): see throughput table at the top of this
  README. 50 KB cable run hits 2,354 bps; injected Watterson
  Good at SNR=15 holds 1,631 bps clean; forced R3/4 at SNR=15
  AWGN delivers 2,676 bps clean (2026-05-07 calibration).
- **Real HF data client validated end-to-end** across Mac and Pi5
  over real audio cable: full B2F session
  matrix passes byte-exact (empty connect/disconnect, text up
  to 12.5 KB, binary attachments, bidirectional, both
  directions). Five real bugs found + fixed during integration;
  full audit at `docs/TNC_CLIENT_AUDIT.md`.
- Known TNC limitation: back-to-back sessions within ~30 s of
  teardown don't always recover cleanly (~1/3 retry success).
  Root cause traced to a reference-client listener race where
  inbound connections can arrive before `Accept()` has re-armed.
  Single-session flows are reliable.
- Mainstream Windows HF mail client: spec-compatible, not yet
  manually tested.

---

## Getting Started

### Alpha operator bundle

For a release build, download `projectultra-<platform>.zip` from the
GitHub release assets. That is the operator bundle: `ultra_tnc`,
`ultra_gui`, `ultra`, `tools/ultra_tnc.conf.example`, and operator
docs. Do not use the source-code archive as the operator download.
Simulator and bench binaries are published separately as
`dev-tools-<platform>.zip`.

### Requirements

- Linux, macOS, or Windows
- CMake 3.16+
- C++20 compiler (GCC 10+, Clang 12+, MSVC 2019+)
- SDL2 (GUI + audio I/O for `cli_simulator` / `ultra_tnc`)
- gRPC + Protobuf + Abseil + c-ares + OpenSSL + RE2 + zlib
  (network audio plane for `ota_simulator` and the OTASim client modes
  of `ultra_gui` / `ultra_tnc` / `cli_simulator`)

### Building

```bash
# Ubuntu/Debian
sudo apt install \
  libsdl2-dev cmake build-essential pkg-config \
  libgrpc++-dev libprotobuf-dev protobuf-compiler protobuf-compiler-grpc \
  libabsl-dev libc-ares-dev libssl-dev libre2-dev zlib1g-dev

# macOS
brew install sdl2 cmake pkg-config grpc protobuf abseil c-ares

# Windows (vcpkg)
vcpkg install sdl2:x64-windows grpc:x64-windows

git clone https://github.com/secup/ProjectUltra.git
cd ProjectUltra
cmake -S . -B build
cmake --build build -j 4
```

On Windows, point CMake at the vcpkg toolchain:

```powershell
cmake -S . -B build -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build -j 4 --config Release
```

### Running

#### Operator path

**TNC** (headless TCP shell for existing HF data clients):

```bash
./build/ultra_tnc --audio-output "USB Audio" --audio-input "USB Audio"
```

**GUI** (operator UI with waterfall and constellation):

```bash
./build/ultra_gui          # Normal mode
./build/ultra_gui -sim     # Developer / simulator mode (no radio needed)
```

> **macOS Gatekeeper note (downloaded prebuilt binaries only).**
> macOS quarantines anything downloaded from the internet. If you
> grabbed a release bundle and macOS shows *"cannot be verified"*
> when you try to launch it, run this once per binary to remove
> the quarantine flag:
>
> ```bash
> xattr -d com.apple.quarantine /path/to/ultra_gui
> ```
>
> Or right-click the binary in Finder → **Open** → confirm
> *Open* in the warning dialog (one-time exception). Binaries
> built locally from source are not affected. Proper code-sign
> + notarization is on the post-alpha release roadmap.

#### Diagnostic / lab tools

**OTASim server** (network HF channel medium — multiple clients share a
session, audio flows over gRPC + UDP, channel model is server-side):

```bash
# Token allowlist file: one `<token>:<station_id>:<description>` per line.
cat >/tmp/ota_tokens.conf <<EOF
alice_token:ALPHA:Alpha station
bob_token:BRAVO:Bravo station
EOF

# Long-running daemon — bind to LAN or localhost.
./build/ota_simulator serve \
  --bind 0.0.0.0:50051 --udp-bind 0.0.0.0:50052 \
  --tokens /tmp/ota_tokens.conf

# Clients (any combination, same or different hosts):
./build/ultra_gui  -sim --ota-host <server>:50051 --station-id ALPHA --token alice_token
./build/ultra_tnc       --sim-audio --ota-host <server>:50051 --station-id BRAVO --token bob_token
./build/cli_simulator   --snr 15 --channel good --rate auto --test
```

Use OTASim instead of the cable rig for protocol/feature work: no
soundcard needed, fully portable, and the server-side reference clock
bridges per-host audio-clock drift across multiple machines.

**CLI simulator** (full protocol, two-station, channel injection; not the
operator TNC):

```bash
./build/cli_simulator --snr 15 --channel good --rate auto --test
```

**Replay and capture analysis** (deterministic decode fixtures and recorded
sessions):

```bash
./build/decode_bench --mode bench --connected \
  --wav fixtures/ofdm_chirp_r14_dqpsk_clean.wav --rate r1_4
./build/session_decode --wav \
  recordings/ota_full_session_2026-05-07/full_session_r1_2.wav
```

**Raw frame CLI** (single-frame transmit/decode, for offline analysis):

```bash
./build/ultra ptx "Hello" -s MYCALL -d THEIRCALL | aplay -f FLOAT_LE -r 48000
arecord -f FLOAT_LE -r 48000 | ./build/ultra prx
```

---

## How It Works

### Protocol stack

```
┌────────────────────────────────────────────────────────┐
│  HF data client / mail app / your own client           │
├────────────────────────────────────────────────────────┤
│  ultra_tnc      (TCP cmd 8300, data 8301)              │
│  TNCSession     (TNC command parser + state machine)   │
│  TNCBridge      (ModemAdapter ↔ ProtocolEngine)        │
├────────────────────────────────────────────────────────┤
│  Connection     (PING/CONNECT/MODE_CHANGE/DATA/DISC)   │
│  ARQ            (Selective-Repeat, window=16, SACKs)   │
│  Frame v2       (4-CW fixed frames + 1-CW control)     │
├────────────────────────────────────────────────────────┤
│  Waveforms      (OFDM-CHIRP, OFDM-NARROW, OFDM-COX,    │
│                  MC-DPSK + adaptive selection)         │
│  LDPC           (IEEE 802.11n; data CLI R1/4 to R3/4)  │
│  Sync / CFO     (dual chirp + Schmidl-Cox + LTS)       │
├────────────────────────────────────────────────────────┤
│  Audio I/O      (SDL2 — Linux ALSA, macOS CoreAudio,   │
│                  Windows WASAPI)                       │
└────────────────────────────────────────────────────────┘
```

### Connection flow

1. **PING/PONG** — fast presence probe (~1 s each) before committing
   to a full CONNECT.
2. **CONNECT** — callsign exchange (FCC Part 97.119 compliant) over
   MC-DPSK.
3. **MODE_CHANGE** — picks data waveform/rate from peer-advertised SNR
   and fading.
4. **DATA** — Selective-Repeat ARQ with cumulative SACKs.
5. **DISCONNECT** — graceful with callsign ID.

If no PONG after 5 PINGs (~15 s), connection fails fast.

### Signal parameters

| Parameter        | MC-DPSK    | OFDM       |
|------------------|-----------:|-----------:|
| Sample rate      | 48 kHz     | 48 kHz     |
| Bandwidth        | ~2.4 kHz   | ~2.8 kHz   |
| Center freq.     | 1500 Hz    | 1500 Hz    |
| Carriers         | 8          | 59         |
| FFT size         | n/a        | 1024       |
| Symbol rate      | ~94 baud   | 41.667 baud |
| Cyclic prefix    | n/a        | 128 (2.667 ms) |
| Sync             | Dual chirp | Dual chirp / Schmidl-Cox |
| LDPC codeword    | 648 bits   | 648 bits   |

---

## Testing

### Unit + regression gate

```bash
cmake --build build -j 4
ctest --test-dir build --output-on-failure -j 4
```

38 tests covering modem primitives, protocol/ARQ, TNC parser, TNC TCP
reactor, TNC bridge, and deterministic `decode_bench` replay fixtures.
CI runs the full matrix on Linux + macOS + Windows with ASAN/UBSAN and
coverage gates.

Hardware smoke remains opt-in. Configure with either
`ULTRA_HARDWARE_TESTS=1 cmake -S . -B build-hw` or
`cmake -S . -B build-hw -DULTRA_BUILD_HARDWARE_TESTS=ON`, then run:

```bash
ctest --test-dir build-hw -R HardwareSmoke --output-on-failure
```

### Full-protocol simulator

```bash
# Default regression (full handshake + ARQ data + disconnect)
./build/cli_simulator --snr 15 --channel good --rate auto --test

# Force specific PHY config
./build/cli_simulator --snr 20 --channel awgn --mod dqpsk --rate r2_3 --test

# CFO chain verification
./tests/verify_cfo_chain.sh --cfo 50 --channel awgn --snr 20 --seed 42
```

### Hardware loopback (two stations)

```bash
# 50 KB Mac↔Pi over real audio cable
SSH_KEY=$HOME/.ssh/id_pi5 PAYLOAD_SIZE=51200 \
  ./tools/tnc_loopback_test.sh
```

`tnc_loopback_test.sh` orchestrates two `ultra_tnc` instances (one
local, one over SSH), pushes a binary payload through the TCP data
port, and CRC-checks delivery.

### Manual modulation / rate selection

`--mod`: operator tools expose `auto` / `dqpsk` by default. Lab-only
forced modes (`d8psk`, `dbpsk`, `qpsk`, `bpsk`, `qam16`, `qam32`,
`qam64`) require `--expert` or `ULTRA_EXPERT_PHY=1` and are not
production ladder rungs.

`--rate`: `auto` (default), `r1_4`, `r1_2`, `r2_3`, `r3_4`. The
operator parsers intentionally reject higher LDPC rates until they are
part of the maintained on-air ladder.

---

## Radio Setup

### Requirements

- SSB transceiver with 2.8+ kHz filter bandwidth (or 500 Hz for
  OFDM-NARROW)
- Audio interface (SignaLink, RigBlaster, USB soundcard, or direct
  cable)
- PTT control (VOX, CAT, or hardware)

### Audio levels

- TX: clean signal without ALC compression
- RX: comfortable listening level, avoid clipping (peak < 0.9)

### Recommended frequencies (2.8 kHz BW, USB)

| Band | Frequency  | Notes |
|------|-----------:|-------|
| 80m  | 3.590 MHz  | Above narrow digital, below voice |
| 40m  | 7.102 MHz  | Common for wideband digital |
| 30m  | 10.145 MHz | Check for WSPR at 10.140 |
| 20m  | 14.108 MHz | Above FT8 crowd |
| 15m  | 21.110 MHz | Above narrow digital segment |
| 10m  | 28.120 MHz | Plenty of room |

**Avoid:** 14.070–14.095 MHz (FT8/PSK31), any .074 MHz (FT8), 14.100
MHz (NCDXF beacons). Listen 10–15 s before TX. Use minimum power
necessary. Be ready to QSY.

---

## Status & Roadmap

### Solid (hardware-validated)

- MC-DPSK baseline (5+ dB SNR, ±50 Hz CFO tolerance).
- OFDM-CHIRP DQPSK R1/4 -> R3/4 with adaptive ladder
  (`selectOFDMCodeRate()` is the exact threshold source).
- OFDM-NARROW (500 Hz) for crowded bands or low-SNR conditions.
- Per-carrier RX erasure with CarrierLDPC v1 interleaver
  (deep notch / QRM survival on OFDM-CHIRP).
- Adaptive MODE_CHANGE with hysteresis (BUG-RATE-001 fixed:
  no panic-downshift on short fading transfers).
- Selective-repeat ARQ with cumulative + selective ACKs,
  hardened DISCONNECT, no timeout storms.
- TNC subsystem: cross-platform Linux / macOS / Windows,
  byte-exact end-to-end, validated with real HF data client sessions.
- Hardware-in-the-loop test rig (Mac ↔ Pi5 with Watterson
  injection) with byte-exact file-transfer validation.
- **First OTA full-session decode (2026-05-07):** full
  chirp + CONNECT + DATA + DISCONNECT sessions decoded from
  Pennsylvania TX audio captured by the Vermont KiwiSDR at
  `recordings/ota_capture_2026-05-07_k1vl/`. R1/4 and R1/2
  completed handshake and byte-exact DATA decode; R3/4 captured
  chirp but lost the handshake, matching the current auto-rate
  exclusion for fading channels.

### Experimental (in tree, not on by default)

- HARQ Chase soft-combining: math validated, infrastructure
  in place; broader CW0-fail integration is on
  `experimental/harq-audit-2026-05-06` pending further
  hardware validation.
- D8PSK on fading channels: variable run-to-run, gated to
  high SNR + AWGN-class fading only.
- Coherent QPSK / 16QAM / 32QAM: stable-path only (NVIS,
  ground wave, clean cable).

### Active work

- Long-running stability soak (multi-hour `ultra_tnc` uptime).
- Bootstrap-rate cap relaxation: short transfers can't yet
  reach R3/4 even on clean AWGN because the initial-rate
  cap requires SNR ≥ 24; hardware validation needed before
  lowering.
- Real over-the-air validation expansion: the 2026-05-07
  KiwiSDR replay path works for full sessions; the remaining
  work is broader live two-way coverage and better low-SNR
  OTA margins.

### Deliberately deferred

- Higher-order constellations (16/32/64-QAM) as production
  ladder rungs: enum reserved, no production code. Real
  capacity headroom (~2× peak throughput) but needs proper
  EVM gating, coherent phase tracking, IQ-imbalance
  handling, and a new auto-rate policy layer — multi-week
  scope, not autonomous-friendly.
- Iterative LDPC ↔ equalizer (turbo) loops.
- OTFS / MFSK: enum reserved for wire compatibility, no
  production implementation.
- Code-sign + notarization for prebuilt macOS binaries
  (Apple Developer ID needed; on the post-alpha roadmap).

Active engineering goal lives in `docs/PROJECT_GOALS.md`.
Recent design + audit notes are in `docs/CHANGELOG.md`,
`docs/PHASE2_CARRIER_MASK_DESIGN.md`, and the
historical reports under `docs/archive/`. Speculative /
archived research is historical only, not part of the
production build.

---

## Contributing

Contributions welcome. Easiest entry points:

- On-air testing reports (especially with `STATS` output included).
- HF data client interop reports - what worked, what did not.
- Bug fixes and DSP optimizations (profile first; see
  `docs/QUALITY_STRATEGY.md`).
- Documentation.

Please open an issue before submitting large PRs.

### Help wanted: real HF recordings

Real ionospheric propagation has characteristics simulation can't
capture. Even "failed" recordings are valuable.

To record your own signal: tune a WebSDR (websdr.org or kiwisdr.com)
to your TX frequency, start recording, transmit using ProjectUltra,
stop and save the file. Submit via GitHub issue with the "Recording"
label, including: callsign, location, band/freq/UTC, WebSDR used, path
distance, and S-meter readings if known.

---

## License

MIT License. See [LICENSE](LICENSE).

---

## Acknowledgments

- Community OTA testers, especially **KC3VPB**, for sharing real-station
  logs that helped diagnose post-handshake sync rejects and buffer-
  overflow edge cases.
- [Dear ImGui](https://github.com/ocornut/imgui) — GUI framework.
- [SDL2](https://libsdl.org/) — audio and windowing.
- [PocketFFT](https://github.com/mreineck/pocketfft) — Fast Fourier Transform.
- [miniz](https://github.com/richgel999/miniz) — compression.
