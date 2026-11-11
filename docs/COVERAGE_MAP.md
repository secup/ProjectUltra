# Coverage Map: Critical Modem Blocks

Last updated: 2026-04-30

## Purpose

This map turns `docs/PROJECT_GOALS.md`, `docs/QUALITY_STRATEGY.md`, and the
current `docs/QUALITY_AUDIT.md` into module-specific coverage expectations.
Use it to choose tests for modem-critical work. It is not a replacement for the
quality strategy, the audit baseline, or the local gate.

The project goal is reliability-first HF file transfer: no silent corruption,
bounded and diagnosable retries, robust sync across AWGN/Good/Moderate/Poor
profiles, and hardware evidence that distinguishes PHY loss, audio level
problems, false sync, ACK loss, scheduling backlog, decode CPU pressure, and ARQ
bugs.

## Why Global 100% Coverage Is Not The Goal

Global line coverage is a trend signal, not the primary release gate. A global
100% target can reward superficial tests over GUI, glue, dead, or reserved paths
while still missing critical modem behavior such as false sync rejection, stale
ACK handling, LDPC failure paths, low-LLR decisions, or burst-tail loss.

For this codebase, release confidence comes from meaningful Tier 0/Tier 1
behavior coverage, seeded regressions, simulator gates, coverage deltas on the
critical scope, and hardware or replay evidence when the change touches hardware
reality.

## Priority Order

1. Payload integrity and no silent corruption: CRC/header validation, LDPC,
   frame and burst interleaving, fixed-frame recovery, and file reassembly.
2. Sync, CFO, and demodulation decisions: chirp/LTS/light sync, CFO feedback,
   LLR quality gates, OFDM equalization, and DPSK reset behavior.
3. ARQ and control reliability: 1-CW control frames, ACK/SACK/NACK behavior,
   stale/future sequence rejection, timeout policy, and retransmission bounds.
4. Streaming TX/RX pipeline: continuous-audio buffering, candidate selection,
   mode-specific frame sizing, false-lock advancement, and decode state.
5. Channel and hardware evidence: seeded channel matrix, hardware harness
   calibration behavior, audio-level diagnostics, and replayable logs/captures.

Priority labels in the tables mean: P0 should be covered first when touched, P1
is high-value hardening for active modem paths, and P2 is supporting coverage
that keeps gates and runtime evidence trustworthy.

## Tier 0 Map

Tier 0 modules can corrupt payloads, lose frames, collapse throughput, or make
hardware tests misleading. Tests for these areas should be deterministic where
possible and should exercise failure behavior, not only happy paths.

| Area | Required Behavior Coverage | Existing Test Signals | Missing Or Weak Tests | Priority |
|------|----------------------------|-----------------------|-----------------------|----------|
| `src/fec/` LDPC wrappers and codec factory | Enforce 648-bit codeword contract, rate-specific payload sizes, LLR sign convention, decode success/failure reporting, invalid/truncated/non-codeword-aligned input rejection, and safe unimplemented-codec failures. | `CodecFactory`, `MultiblockLDPC`, `ComprehensiveModem` | More edge cases for rate changes, extreme/near-zero LLRs, corrupted multi-CW blocks, decoder iteration policy, and hardware-derived weak-block regressions. | P0 |
| `src/fec/` frame and burst interleavers | Prove reversible mapping, exact length preservation, deterministic ordering, partial/failure handling, and no cross-CW corruption when one physical burst is weak. | `Interleaver`, `FrameV2`, `WaveformLoopback` | Burst-tail replay tests from weak physical blocks and explicit 4-CW interleaver failure-locality checks. | P0 |
| `src/dsp/` primitives and FFT helpers | Keep filter, FFT, Hilbert, NCO, and resampler outputs finite and deterministic; preserve reset behavior; avoid phase/frequency drift; and use Hilbert-style CFO simulation where tests inject CFO. | `DSPPrimitives`, `FFT`, `Layers`, `ComprehensiveModem` | More FFT/resampler edge cases, Hilbert analytic-signal checks under short buffers, and numeric-stability tests for unusual sample windows. | P1 |
| `src/protocol/frame_v2.*` and legacy frame builder/parser | Reject bad magic/CRC/header sizes, preserve callsign hashes and sanitized callsigns, distinguish control vs data frames, keep 1-CW control frames and 4-CW fixed data frames, recover failed CWs with NACK bitmap behavior, and never accept malformed payloads as valid data. | `FrameV2`, `LegacyFrameBuilder`, `Protocol` | More fixed-frame false-positive/recovery paths, callsign-hash collision expectations, and reserved-mode capability checks. | P0 |
| `src/sync/` chirp and timing acquisition | Complex chirp correlation, dual-chirp CFO gap math, corrected training start position, timing-offset tolerance, noise/silence false-lock rejection, and narrow/wide chirp discrimination. | `SyncDetection`, `WaveformLoopback`, `StreamingSignalPolicy` | Direct false-lock tests under noise-only and leading-silence inputs, known CFO offsets, narrowband edge cases, and replay fixtures for marginal real captures. | P1 |
| `src/ofdm/` modulation, demodulation, equalization, and link adaptation | OFDM carrier geometry consistency, finite LLR output, pilot/CPE tracking behavior, LTS residual CFO refinement, DQPSK/D8PSK soft demap behavior, sane SNR/fading estimates, and no stale CFO reuse. | `Layers`, `OFDM`, `OFDMLinkAdaptation`, `WaveformLoopback`, `ComprehensiveModem`, `StreamingSignalPolicy` | Branch coverage for `channel_equalizer.cpp` and `demodulator.cpp`, two-stage CFO work tracked by `BUG-CFO-001`, fading-index edge cases, and absolute phase/CFO stress fixtures. | P1 |
| `src/psk/` MC-DPSK and DPSK paths | Reset demodulator state for every frame, apply frame CFO before decode, keep MC-DPSK separate from OFDM interleaving/light preamble behavior, and verify low-SNR stop-and-wait operation. | `StreamingMCDPSK`, `ComprehensiveModem`, `WaveformPolicy` | Direct vector tests for DPSK differential decisions, reset/CFO carryover, and low-SNR false-positive rejection. | P1 |
| `src/waveform/` wrappers and factory | Preserve `IWaveform` call order assumptions, clear CFO on reset, expose training positions, produce finite soft bits, select only production-supported modes, and reject reserved/unsupported modes. | `WaveformLoopback`, `WaveformPolicy`, `OFDMLinkAdaptation` | Wrapper error paths, reset-after-sync cases, reserved enum behavior, and light-preamble edge cases. | P1 |
| `src/protocol/selective_repeat_arq.*` and `src/protocol/arq.*` | Window clamping, ACK/SACK decoding, stale/future ACK rejection, duplicate ACK suppression, RTT/RTO policy, timeout retransmit, fast-hole handling, sequence wraparound, RX reordering, and zero-capacity refusal. | `SelectiveRepeatARQ`, `SelectiveRepeatPolicy`, `StopWaitARQ` | More loss-pattern edge cases, long wraparound sessions, mixed ACK/SACK loss, and retransmission storm bounds under seeded drops. | P0 |
| `src/protocol/file_transfer.*` | Reassemble out-of-order chunks, preserve final compressed chunk markers, reject unauthenticated oversized metadata, avoid unsafe duplicate names, and prove received bytes match original bytes. | `FileTransferController`, `Protocol` | Malformed metadata/error paths, DATA_START/DATA_END workflow completion, large-file seeded transfer integrity, and replay tests from file-transfer failures. | P0 |
| `src/gui/modem/streaming_encoder.*` and `streaming_decoder.*` | Continuous single-decoder audio stream, bounded ring buffer, mode-specific sample sizing, 1-CW control-first peek, 4-CW data decode/deinterleave, false-lock advancement, LLR/LTS/CFO gates, and correct sample consumption. | `StreamingConfig`, `StreamingBufferPolicy`, `StreamingDecodePolicy`, `StreamingFramePolicy`, `StreamingSignalPolicy`, `StreamingMCDPSK` | Decode state-machine extraction, candidate selection around burst/data escalation, 1-CW control retry policy, ACK-loss traces, and hardware-derived false-lock regressions. | P1 |
| `src/sim/hf_channel.hpp` channel injection used by release gates | Deterministic seeded AWGN/Good/Moderate/Poor profiles, accurate SNR/fading knobs, no clipping by default, and reproducible loss classes for diagnostics. | `ComprehensiveModem`, `cli_simulator` via local/regression gates | Direct channel-statistics tests and profile fixtures that catch accidental changes to release-gate channels. | P2 |

## Tier 1 Map

Tier 1 modules break real usage when wrong even if the core math is correct.
Prefer state-machine and two-station tests over chasing line coverage in
large, coupled runtime files.

| Area | Required Behavior Coverage | Existing Test Signals | Missing Or Weak Tests | Priority |
|------|----------------------------|-----------------------|-----------------------|----------|
| `src/protocol/connection.*` and handlers | PING/PONG/CONNECT/MODE_CHANGE/DATA/DISCONNECT transitions, waveform capability negotiation, wide/narrow timing, ACK repeat policy, SACK delay, timeout floors/ceilings, and safe fallback under fading. | `ConnectionPolicy`, `Protocol`, `cli_simulator --test` through regression gates | Direct transition tests for remaining `connection.cpp` and `connection_handlers.cpp` branches, rare CONNECT-leg retry race from `BUG-CTRL-001`, and stale disconnect/data ACK separation. | P1 |
| `src/protocol/protocol_engine.*` and `src/modem/modem.cpp` | Route frames to the correct protocol state, keep sequence/payload integrity across encode/decode, surface errors without silent delivery, and maintain mode boundaries. | `Protocol`, `FrameV2`, `cli_simulator --test` | More negative-path state tests and replay fixtures for protocol errors observed during full transfers. | P2 |
| `tools/cli_simulator.cpp` | Two-station full-protocol coverage with light preamble, connected-mode configuration, selected rate/profile, retries, delivery result, and useful logs for diagnosing failure class. | `tests/regression_matrix.sh --quick`, `agents/run_local_gate.sh` | The simulator itself is large and under-tested; extract pure harness policy, add seeded matrix coverage for 50 KB transfers, and keep logs structured enough for automated classification. | P1 |
| `src/gui/modem/modem_engine.*`, `modem_rx.*`, `modem_mode.*` | Thread-safe RX/TX state, mode switching, hardware-safe timing, decode-thread lifecycle, telemetry integrity, and no unsynchronized access to shared runtime state. | `StreamingConfig`, policy tests, CTest integration coverage | Direct state tests after extracting pure logic, race/backlog tests, and hardware log replay around mode switches and ACK tail behavior. | P2 |
| `src/gui/audio_engine.*` and hardware test scripts | Preserve device selection, sample-rate assumptions, calibrated RMS/peak interpretation, hardware lock discipline, and clear distinction between clipping, silence, and modem decode failure. | `AudioBasic`, `WavLoopback`, maintained hardware smoke scripts when explicitly required | Automated offline checks for captured audio levels, hardware harness failure classification, and replay fixtures from Mac/Pi captures. | P2 |

## Evidence Expectations

For any future Tier 0/Tier 1 task, report:

- the rows above that the change touches;
- the specific behavior expectation covered;
- the exact test or replay command and result;
- whether coverage changed for the touched critical files;
- any missing tests that remain as residual risk.

Use `ctest --test-dir build --output-on-failure -j4` and
`./tests/regression_matrix.sh --quick` as maintained software gates. Use
`./agents/run_local_gate.sh` for queued agent tasks. Run hardware smoke only
when the task requires hardware-facing validation and the hardware lock is held.
