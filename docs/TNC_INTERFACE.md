# Ultra TNC Interface

`ultra_tnc` exposes a legacy-compatible TCP TNC shell backed by ProjectUltra's
ProtocolEngine, StreamingEncoder/StreamingDecoder, and SDL audio I/O.

## Run

```bash
./build/ultra_tnc \
  --audio-output "USB Audio Device" \
  --audio-input "USB Audio Device" \
  --callsign N0CALL \
  --port 8300
```

The command port is `--port` and the binary data port is `--port + 1`.
By default, `ultra_tnc` binds to `127.0.0.1:8300` and `127.0.0.1:8301`.
Use `--bind 0.0.0.0` only when you intentionally want LAN access.

For TCP-only smoke tests, use `--audio-output none --audio-input none`.

## Client Configuration

Point the client at the command port:

```toml
[[ax25]]
  port = "ultra"

[[transport.legacy_tnc]]
  address = "127.0.0.1:8300"
```

If your client separates command and data ports, use:

- Command: `127.0.0.1:8300`
- Data: `127.0.0.1:8301`

## Supported TNC Commands

The TNC shell accepts the Phase 1/2 command set:

- `VERSION`
- `MYCALL <call> [secondary...]`
- `LISTEN ON|OFF|CQ`
- `CONNECT <src> <dst>`
- `DISCONNECT`
- `ABORT`
- `BW500`, `BW2300`, `BW2750`
- `BUFFER`
- `SN`
- `BITRATE`
- `STATS` (ProjectUltra extension; see below.)
- `COMPRESSION`, `CHAT`, `CWID`
- `PUBLIC`, `P2P`, client-mode probes, `IGNOREKISSDCD`, `RETRIES`, `CALLINT`

### STATS

`STATS\r` returns a single-line snapshot of ARQ counters and the current
PHY configuration:

```text
STATS frames_sent=42 frames_recv=38 retx=5 timeouts=2 failed=0 \
  out_of_order=1 rate=R1_2 mod=DQPSK mode=OFDM_CHIRP snr=15 bps=2300 \
  backlog=128
```

Fields are space-separated `key=value` pairs. Counters are session-
scoped (cleared on disconnect/reset). `rate`, `mod`, `mode` track the
current adaptive selection; `snr`/`bps` mirror the existing `SN`/`BITRATE`
events; `backlog` mirrors `BUFFER`. Existing clients ignore unknown
commands, so this is safe to leave on.

Bandwidth mapping:

- `BW500` selects `OFDM_NARROW`
- `BW2300` (the TNC's startup setting) selects a wideband session with the waveform left
  automatic, as in the GUI: the responder picks MC-DPSK or `OFDM_CHIRP` from the measured SNR
  and the waveform follow moves the session between them as the channel changes
- `BW2750` is accepted for legacy compatibility and behaves as `BW2300`

Until 2026-11-14 `BW2300`/`BW2750` forced `OFDM_CHIRP`, so a TNC session below the OFDM floor
started in OFDM (BUG-TNC-BW-FORCES-OFDM). `BITRATE` reports the session rung's information rate
(one data frame's payload over its airtime: 1962 bps at OFDM QPSK R1/2, 77 bps on 16-FSK-F) at
`CONNECTED` and again whenever the rung changes; `STATS bps` is the same number. Until
2026-11-23 it was the bandwidth's nominal OFDM rate whatever the session ran.

Binary payload bytes are sent on the data port while connected. Received
ProtocolEngine binary payloads are emitted on the data port.

## Quick Check

```bash
./build/ultra_tnc --audio-output none --audio-input none --port 18300
printf 'VERSION\r' | nc 127.0.0.1 18300
```

Expected reply (the build's version):

```text
VERSION 0.6.4.1
```

## Session behavior

- **LISTEN** is off when `ultra_tnc` starts. Until the host sends `LISTEN ON` the TNC neither accepts
  calls nor answers a PING probe, so it never keys the transmitter on its own. `LISTEN ON` stays in
  force until `LISTEN OFF`: after a session (inbound or outbound) the TNC listens again. (Before
  2026-11-23 a TNC answered every PING whatever LISTEN said, and dropped back to not listening after
  its first session while the modem kept accepting calls.)
- **ABORT** ends the session at once: `OK`, then `BUFFER 0` if data was queued, then `DISCONNECTED`.
  The peer is not told; a peer that was receiving declares the link lost once the sender's whole
  retry budget has passed (about 4 minutes on OFDM, up to about 12 minutes on MC-DPSK).
- **DISCONNECT** during call setup cancels the attempt: `OK`, then `DISCONNECTED`.
- **STATS** outside a session reports `rate=none mod=none mode=none snr=none bps=0`.

## Testing

`tools/tnc_scenario.py` runs two real `ultra_tnc --sim-audio` stations over `ota_simulator serve`
and drives them through the command and data ports the way a messaging client does: short messages,
turn-taking, simultaneous sends, chunked writes, a bulk body then short replies, back-to-back
sessions, ABORT, a call nobody answers, a call to a station not listening, cancelling a call, two
stations calling each other at the same moment. CTest runs a set of them on every build
(`TncScenarioShort`, `TncScenarioTurns`, `TncScenarioSessions`, `TncScenarioListen`).

```bash
tools/tnc_scenario.py --scenario all --channel awgn --snr-db 10 --out /tmp/tnc_all
tools/tnc_scenario.py --scenario short,pingpong --channel good --snr-db 8 --seed 43 --out /tmp/tnc_good
```

Each scenario writes `<out>/<scenario>/{summary.txt,events.log,alpha.log,bravo.log}`; `events.log`
has every host-visible line with its time.
