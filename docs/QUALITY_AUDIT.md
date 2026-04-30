# ProjectUltra Quality Audit

Last updated: 2026-04-30

## Purpose

This is the current quality baseline and hardening backlog. `QUALITY_STRATEGY.md`
defines the policy; this file tracks where the codebase stands against it.

## Current Test Baseline

Registered CTest targets: 29

Current maintained local gates:

```bash
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build --output-on-failure -j4
./tests/regression_matrix.sh --quick
./scripts/coverage_report.sh
git diff --check
```

Latest measured coverage after the current hardening pass:
- Line: 56.44%
- Function: 62.04%
- Branch: 44.74%

This is a baseline, not an acceptable final state for critical modem code.

## Tier 0 Coverage Snapshot

Measured from `build-coverage/coverage.txt`.

| Area | Line Coverage | Function Coverage | Branch Coverage | Assessment |
|------|---------------|-------------------|-----------------|------------|
| `src/fec` | 81.09% | 86.42% | 76.20% | Stronger after codec wrapper/factory tests; LDPC internals still need edge cases |
| `src/ofdm` | 57.74% | 56.57% | 49.72% | Improved by waveform loopback; sync/equalizer branches still weak |
| `src/arq/arq_controller.cpp` | 97.84% | 94.44% | 77.78% | Legacy ARQ TX split, ACK/NACK, timeout, RX reorder, stale-ACK, and zero-capacity paths covered |
| `src/framing/frame_builder.cpp` | 82.63% | 92.31% | 88.46% | Legacy public frame builder/parser now covered, including CRC rejection and empty control frames |
| `src/protocol/frame_v2.cpp` | 53.77% | 68.92% | 32.85% | Edge cases improved; fixed-frame recovery paths still weak |
| `src/protocol/frame_v2.hpp` | 89.29% | 95.24% | 76.42% | Strong helper coverage; keep malformed-frame tests growing |
| `src/protocol/selective_repeat_arq.cpp` | 68.72% | 86.67% | 45.63% | Strong behavior tests; branch coverage now needs loss-pattern edge cases |
| `src/protocol/selective_repeat_arq_policy.hpp` | 100.00% | 100.00% | 96.67% | Extracted and tested from ACK/SACK, timer, fast-hole, ACK-repeat, and RTT/RTO policy |
| `src/protocol/connection.cpp` | 49.16% | 62.79% | 32.68% | Timing policy extracted; remaining state-machine transitions still need direct tests |
| `src/protocol/connection_handlers.cpp` | 50.00% | 58.33% | 28.15% | Negotiation helpers extracted; frame handler branches still need transition tests |
| `src/protocol/connection_policy.hpp` | 97.06% | 100.00% | 81.82% | Extracted and tested from OFDM timing, ACK timeout, ACK repeat, SACK delay, fading label, and negotiation policy |
| `src/protocol/file_transfer.cpp` | 72.08% | 83.33% | 52.66% | Compressed final-chunk and duplicate-name regressions covered; malformed/error paths still need direct tests |
| `src/waveform` | 54.06% | 46.21% | 35.30% | Loopback coverage added; wrapper edge cases remain |
| `src/psk` | 63.56% | 42.55% | 53.57% | Needs direct DPSK edge/vector tests |
| `src/sync` | 37.61% | 42.31% | 48.56% | Needs chirp false-lock/CFO/timing tests |
| `src/dsp` | 91.23% | 92.68% | 85.85% | Strong direct primitive coverage; FFT/resampler edge cases remain |
| `src/gui/modem/streaming_buffer_policy.hpp` | 95.92% | 100.00% | 83.33% | Extracted and tested from `feedAudio()` backlog/overflow policy |
| `src/gui/modem/streaming_decode_policy.hpp` | 93.55% | 100.00% | 90.00% | Extracted and tested from decode sample-sizing policy |
| `src/gui/modem/streaming_frame_policy.hpp` | 100.00% | 100.00% | 95.00% | Extracted and tested from ping, false-lock, sync-recovery, and frame-consumption policy |
| `src/gui/modem/streaming_signal_policy.hpp` | 100.00% | 100.00% | 92.42% | Extracted and tested from LLR, LTS, light-sync, sync CFO, and pilot CFO policies |
| `src/gui/modem/streaming_decoder.cpp` | 31.44% | 60.42% | 19.47% | Geometry/config, buffer, sample, frame, and signal policies covered; decode state machine still needs extraction |
| `src/gui/modem/streaming_encoder.cpp` | 35.75% | 65.00% | 18.06% | Geometry/config covered; frame encoding branches still need extraction |

## Highest-Risk Files

Large and under-tested:
- `src/gui/modem/streaming_decoder.cpp`
- `src/gui/modem/streaming_encoder.cpp`
- `src/protocol/connection.cpp`
- `tools/cli_simulator.cpp`
- `src/ofdm/channel_equalizer.cpp`
- `src/ofdm/demodulator.cpp`
- `src/sync/chirp_sync.hpp`

These should not be "covered" by superficial tests. They need extraction into
smaller units with direct tests around their real invariants.

## Immediate Hardening Backlog

1. Codec/FEC direct tests:
- `CodecFactory` name/type behavior and unimplemented-codec failures.
- `LDPCCodec` wrapper rate changes, iteration policy, encode/decode success/fail paths.
- More LDPC invalid-input and edge-size tests.

2. DSP direct tests:
- FIR low/high/bandpass sanity and reset behavior.
- Biquad low/high/bandpass/notch finite output and reset behavior.
- AGC convergence and clamp behavior.
- NCO phase/frequency behavior.
- Hilbert finite analytic output.

3. Protocol/framing tests:
- More fixed-frame false-positive/recovery paths.
- Reserved waveform values not advertised.
- Callsign hash collision handling expectations.
- Connection state-machine transitions extracted from `Connection`.

4. Sync/OFDM tests:
- Chirp detection with timing offset, silence lead-in, false-noise rejection.
- CFO stress with known offsets and absolute phase handling.
- LTS/data-preamble timing tolerance.
- Equalizer pilot tracking and fading-index behavior.

5. Streaming TX/RX tests:
- Extract and test frame candidate selection.
- Extract and test 1-CW control decode retry policy.
- Extract and test multi-CW decode/deinterleave decisions.
- Add deterministic streaming false-lock/ACK-loss regression traces from hardware logs.

6. CI and tooling:
- Keep CTest, sanitizer, and coverage gates mandatory.
- Add static analysis only after current warnings are triaged.
- Add full seeded alpha gate as scheduled/nightly, not per-PR.
- `scripts/run_bench_matrix.sh` is a maintained, evidence-only `cli_simulator`
  sweep. Quick mode runs an AWGN + Good baseline; default mode adds Moderate
  fading and a small file transfer. It writes a CSV (commit, channel, SNR,
  rate, retransmissions, timeouts, frame success, elapsed, data-phase bps,
  log path) under `${TMPDIR:-/tmp}/ultra_bench_<ts>` and exits non-zero on any
  failed delivery so it can run in CI without becoming a coverage gate.

## Hardening Added In This Pass

- CI now runs multi-platform CTest, Linux sanitizer, and Linux coverage before packaging.
- Local coverage is reproducible with `scripts/coverage_report.sh`.
- New maintained tests cover codec factory/LDPC wrapper behavior, DSP primitives,
  stop-and-wait ARQ compatibility, waveform loopback, OFDM link geometry, and
  streaming TX/RX config parity.
- `test_frame_v2` now covers callsign sanitation, ping frames, channel report
  quantization, connect-frame CRCs, malformed headers, `CodewordStatus` edge
  cases, and fixed-frame helper policy.
- OFDM pilot/data-carrier geometry is centralized in `ofdm_link_adaptation` and
  reused by streaming TX/RX, waveform sample/throughput estimates, modem-rate
  estimates, and connection timeout calculations.
- Streaming RX ring-buffer overflow/backlog policy is extracted into
  `streaming_buffer_policy.hpp` with wraparound, pointer-drift, overflow, and
  telemetry tests.
- Streaming RX decode sample-sizing policy is extracted into
  `streaming_decode_policy.hpp` with robust OFDM control-frame sizing and
  pending-CW/burst/control-peek selection tests.
- Streaming RX signal-decision policy is extracted into
  `streaming_signal_policy.hpp` with direct tests for LLR quality gates, invalid
  OFDM LTS training rejection, light-sync weak acceptance, and sync CFO clamping.
- Pilot-CFO feedback now goes through the same tested policy in normal decode,
  sync-recovery, and burst paths. The sync-recovery path now adds pre-correction
  CFO back to the residual pilot estimate instead of storing the residual alone.
- Streaming frame-level decisions are extracted into `streaming_frame_policy.hpp`
  with direct tests for PING RMS classification, false-lock advancement,
  control-first OFDM peek eligibility, sync-recovery gating, and non-data frame
  sample consumption.
- Selective Repeat ARQ ACK/SACK decisions are extracted into
  `selective_repeat_arq_policy.hpp` with direct tests for legacy/wide SACK
  decoding, sequence wraparound, stale/future ACK rejection, duplicate-ACK
  suppression, delayed SACK timers, fast-hole admission, hole-probe timers, and
  ACK-repeat timing jitter.
- Selective Repeat ARQ RTT/RTO estimator math is also covered in
  `selective_repeat_arq_policy.hpp`, including Karn-safe eligibility,
  sample clamping, SRTT/RTTVAR EWMA updates, and configured timeout floor/ceiling
  behavior.
- Connection timing and negotiation decisions are extracted into
  `connection_policy.hpp`, including wide/narrow OFDM frame timing, hardware-safe
  ACK timeout floors/ceilings, near-AWGN window/batch policy, ACK repeat policy,
  SACK-delay policy, fading labels, and waveform capability selection.
- File-transfer receive reassembly now preserves a compressed transfer's final
  chunk marker across out-of-order buffering. `test_file_transfer_controller`
  covers the regression where the final compressed chunk arrives before the
  gap-filling chunks and must still finalize once the buffer drains.
- File-transfer receive-path naming now uses path-aware duplicate suffixing and
  bounded receive-buffer preallocation. This fixes duplicate extensionless
  filenames inside dotted receive directories and avoids unbounded reserve from
  unauthenticated metadata.
- Legacy `FrameBuilder`/`FrameParser` is confirmed as public API rather than
  dead code. Empty SYNC/PROBE/DISCONNECT frames now use the same CRC-bearing
  layout the parser requires, and legacy ARQ now refuses zero-capacity frame
  configs instead of looping forever.
- Legacy `ARQController` now has direct tests for TX chunk splitting,
  cumulative ACK removal, NACK/timeout retransmission, reset behavior, RX
  out-of-order buffering, and ACK advancement after buffered frames drain.

## Definition Of Progress

Progress is not "number of tests." Progress is:
- more Tier 0 branch coverage,
- fewer untested state transitions,
- fewer huge untestable files,
- more bugs represented by deterministic regressions,
- and CI catching regressions before hardware time is wasted.
