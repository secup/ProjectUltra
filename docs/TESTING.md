# Testing and measurement

How ProjectUltra is tested, and the rules for quoting a throughput number. Moved out of the
README on 2026-11-09; `CLAUDE.md` holds the condensed version for contributors working with AI
assistants.

## Measuring throughput (read before quoting a number)

A fading channel is a noisy measurement instrument, and this project has retracted claims for
ignoring that. The rules that survived contact with the hardware:

- **Paired and interleaved.** Run arm A and arm B back to back and compare the per-pair delta.
  The channel drifts; comparing arm means across epochs has read −10% purely from a rougher
  epoch.
- **n ≥ 8 pairs.** Paired standard deviation is 14–36%, so n=8 resolves about 15% and **n=3
  resolves nothing**. Single runs are observations, not evidence.
- **Report both tests.** A paired t-test and a sign test, and say so when they disagree.
- **Prove the knob engaged.** Grep the log for it. A knob that silently did not engage has
  produced confident null results more than once.
- **Report the mechanism, not just goodput** — craters, mode changes, retransmissions. Goodput
  alone has called levers a "wash" that mechanism metrics then decided.
- **Simulator caveat.** On the seeded GUI gate every decision shifts the run's timeline, so two
  runs of the same seed are not paired samples on fading channels; compare means and spreads.
  AWGN runs are deterministic to within a few bps.

## Unit + regression gate

```bash
cmake --build build -j 4
ctest --test-dir build --output-on-failure -j 4
```

The suite covers modem primitives, protocol/ARQ, the TNC parser, TCP reactor and bridge, the
channel models, and the long Monte-Carlo calibration gates (label `slow`). CI runs it on Linux,
macOS and Windows (optimized builds, test asserts kept live), plus ASan/UBSan (`-LE slow`) and a
coverage gate (`scripts/coverage_report.sh`).

Hardware smoke tests are opt-in:

```bash
cmake -S . -B build-hw -DULTRA_BUILD_HARDWARE_TESTS=ON   # or ULTRA_HARDWARE_TESTS=1
ctest --test-dir build-hw -R HardwareSmoke --output-on-failure
```

## Full-protocol gate

Two real `ultra_gui -sim` stations over a live `ota_simulator serve` channel: PING/PONG →
CONNECT → MODE_CHANGE → file transfer → DISCONNECT, in real time. This is the gate that backs a
fade, throughput or full-protocol claim.

```bash
# Full handshake + ARQ file transfer + disconnect; reads RESULT/GOODPUT_BPS from summary.env
tools/gui_qso_scenario.sh --channel good --snr-db 20 --seed 42 --file-kb 21 --out /tmp/X

# The two radios' dial offset and soundcard clock offset (ALPHA hears -D Hz / +P ppm)
tools/gui_qso_scenario.sh --channel awgn --snr-db 20 --cfo-hz 7.3 --clock-ppm 50 --out /tmp/X

# Force a specific PHY rung (measurement only — bypasses rate control)
ULTRA_FORCE_DATA_MOD=QAM8 ULTRA_FORCE_DATA_RATE=R2_3 ULTRA_LOCK_RATE=1 \
  tools/gui_qso_scenario.sh --channel good --snr-db 20 --out /tmp/X
```

Runs from one checkout must not overlap: the script stops any `ultra_gui` built in that tree
when it starts. Parallel runs need separate copies of `build/` and `tools/`.

(`cli_simulator` was retired 2026-05-30: it was CPU-paced and wrapped a PHY that had diverged
from production, so it produced misleading passes.)

## OTASim network channel server

A long-running daemon that hosts the HF medium in software: clients join a session, audio flows
over gRPC (control) + UDP (samples), and the channel model is applied server-side. `ultra_gui`
and `ultra_tnc` can run as OTASim clients instead of opening a soundcard.

```bash
# Token allowlist file: one `<token>:<station_id>:<description>` per line.
cat >/tmp/ota_tokens.conf <<EOF
alice_token:ALPHA:Alpha station
bob_token:BRAVO:Bravo station
EOF

./build/ota_simulator serve \
  --bind 0.0.0.0:50051 --udp-bind 0.0.0.0:50052 \
  --tokens /tmp/ota_tokens.conf

./build/ultra_gui -sim --ota-host <server>:50051 --station-id ALPHA --token alice_token
./build/ultra_tnc --sim-audio --ota-host <server>:50051 --station-id BRAVO --token bob_token
```

Channel models: AWGN and ITU-R F.1487 Watterson Good (0.1 Hz / 0.5 ms), Moderate (0.5 Hz / 1 ms)
and Poor (1 Hz / 2 ms), plus a per-receiver dial offset (`--lobby-rx-cfo-hz`) and soundcard clock
offset (`--lobby-rx-clock-ppm`).

## Hardware testing (two stations)

Hardware runs use two machines with real soundcards (or the OTASim client mode on each) and the
same scenario on both: `ultra_tnc` with an HF data client, or `ultra_gui` with its file transfer.
Run both stations on the same build; protocol changes are not negotiated between versions. Quote
results by the rules above (paired, interleaved, n ≥ 8).

## Replay and measurement tools

```bash
# Replay a diagnostics bundle (Report button / ultra_report) through the live decoder
./build/ultra_replay --bundle report.zip [--realtime]

# Frame / ACK error rate through the real encoder and decoder over a channel model
./build/measure_ack_fer --snr 20 --config data4_full --channel good --mod qpsk --rate r2_3 --seed 1 --n 200

# Single-frame transmit / decode
./build/ultra ptx "Hello" -s MYCALL -d THEIRCALL | aplay -f FLOAT_LE -r 48000
arecord -f FLOAT_LE -r 48000 | ./build/ultra prx
```

## Manual modulation / rate selection

`--mod`: operator tools expose `auto` / `dqpsk`. Lab-only forced modes (`d8psk`, `dbpsk`, `qpsk`,
`bpsk`, `qam16`, `qam32`, `qam64`) require `--expert` or `ULTRA_EXPERT_PHY=1` and are not
production ladder rungs. `--rate`: `auto` (default), `r1_4`, `r1_2`, `r2_3`, `r3_4`.
