# OTA Simulator Live Channel Plan (v3 — Final Scope)

**Status:** Locked design as of 2026-05-16 (third rewrite). Supersedes
v1 (1488-line exploratory draft, in git history) and v2 (681-line
test-platform framing, in git history). Reflects the final scope
landed in `docs/OTA_SIMULATOR_DESIGN_LOG.md` Round 10.

**Decision authority:** `docs/OTA_SIMULATOR_DESIGN_LOG.md` (Rounds
1–10). This plan is **prescriptive**. The log is the historical
record of how we got here, including two scope corrections and
significant cuts. Disagreements with this doc reopen the relevant
question in the log; do not edit this doc unilaterally.

**The scope question:** earlier rounds drifted into SaaS-product
territory (TLS-required public bind, rate limits, token CLI with
hashed allowlist, lobby rotation engine, signed receipts, separate
test-runner binary). The user gut-checked: not a SaaS product, just
"something solid that supports 4-5 operators jumping in for tests."
This plan reflects that scope.

---

## Purpose

ProjectUltra needs a small external HF channel service so that:

- The GUI behaves as one station attached to a real medium, not a
  two-station hosting process
- 4–5 friends can join the same server from anywhere (Tailscale /
  Wireguard / private LAN) and exchange live HF-like audio over a
  shared simulated channel
- Existing modem regression tests (currently in `cli_simulator`)
  keep running, just routing audio through the server instead of
  embedding the channel

```text
ota_simulator serve  (long-lived daemon on AWS / laptop / Tailscale)
  └── the HF medium; channel + sessions + tokens + captures

ultra_gui -sim --ota-host host:port --station-id alice
  └── one station, real gRPC + UDP client of the server

ultra_tnc --sim-audio host:port/alice
  └── same, headless

cli_simulator (refactored)
  └── runs scripted modem regressions; now talks to the server
      over gRPC instead of embedding SimulatedChannel
```

SDL remains the normal hardware-audio backend; this work adds a
network-audio backend, it does not replace SDL.

---

## Non-Goals (explicit)

- Not a SaaS product
- No public-internet exposure required (Tailscale / VPN is the
  hosting model)
- No TLS required for MVP (private network — add later if needed)
- No rate limiting (4–5 known friends do not DoS each other)
- No abuse handling / live kick / token revoke as live admin ops
  (restart server with new config)
- No token CLI / hashed allowlist / SIGHUP reload (plain config
  file with N tokens; edit + restart)
- No signed receipts (captures + JSONL events are sufficient
  evidence)
- No separate `ota_test_runner` binary (cli_simulator already
  plays that role)
- No lobby rotation engine (default channel + operator override is
  enough)
- No web monitor dashboard
- No federation / multi-region
- No hardware radio bridge
- No KiwiSDR live ingestion in MVP (V1 candidate if useful)

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│  ota_simulator serve  (long-lived daemon)                │
│  ┌────────────────────────────────────────────────┐     │
│  │  Channel Core (static library)                  │    │
│  │   - mixer (sample-index-ordered)                │    │
│  │   - channel models                              │    │
│  │     · null/passthrough                          │    │
│  │     · AWGN                                       │   │
│  │     · Watterson good/moderate/poor              │    │
│  │   - effect scheduler (live operator commands)   │    │
│  └────────────────────────────────────────────────┘     │
│  ┌────────────────────────────────────────────────┐     │
│  │  Session Manager                                │    │
│  │   - SessionContext (per-session isolation)      │    │
│  │   - Lobby (always-on default session)           │    │
│  │   - Private sessions (created on demand)        │    │
│  └────────────────────────────────────────────────┘     │
│  ┌────────────────────────────────────────────────┐     │
│  │  Auth (token bearer against config file)        │    │
│  └────────────────────────────────────────────────┘     │
│  ┌────────────────────────────────────────────────┐     │
│  │  gRPC control plane + UDP audio plane           │    │
│  └────────────────────────────────────────────────┘     │
│  ┌────────────────────────────────────────────────┐     │
│  │  Capture writer + event stream                  │    │
│  └────────────────────────────────────────────────┘     │
└────────────────┬────────────────────────────────────────┘
                 │
        ┌────────┼────────┬──────────────┐
        │        │        │              │
   ultra_gui  ultra_tnc  cli_simulator   ota_observer
   (operator) (operator) (CI test       (listen-only,
                          harness)       optional)
```

**Boundary discipline:**
- Server is the **medium**. It never has a modem stack.
- Clients own their modem + protocol stack.
- Channel core is a **standalone static library** that can be
  unit-tested without networking, sessions, or gRPC.
- gRPC + audio plane sit above the channel core + session manager.

---

## Server Scope

### What the server does

- Listens for gRPC + UDP audio on a configurable bind address (no
  TLS required for MVP)
- Validates bearer tokens in gRPC metadata against a config file
- Manages multiple concurrent sessions (`SessionContext` per
  session, no shared mutable channel state)
- Spawns a lobby session on startup (well-known session ID,
  always-on, default channel from config)
- Accepts live channel commands per session (set channel, inject
  effect)
- Mixes per-station TX → per-station RX via the channel core
- Captures per-session audio (WAV) + events (JSONL) to disk
- Streams events to subscribed clients
- Gracefully drains sessions on SIGTERM
- Reports liveness via `/healthz`

### What the server explicitly does NOT do

- Scenario parsing / scripted event timeline
- Assertions / pass-fail evaluation
- Signed receipts with claim levels
- TLS termination (MVP)
- Rate limiting (MVP)
- Self-service token issuance (MVP)
- Federation / cross-server session handoff

These are either client-side (scenarios live in cli_simulator) or
deferred to V1+ if the need actually materializes.

---

## gRPC Service (Illustrative)

```proto
syntax = "proto3";

package projectultra.otasim.v1;

import "google/protobuf/empty.proto";
import "google/protobuf/timestamp.proto";

service OtaSimulatorControl {
  // Station + audio attachment
  rpc RegisterStation(RegisterStationRequest) returns (StationLease);
  rpc NegotiateAudio(NegotiateAudioRequest) returns (AudioLease);
  rpc Heartbeat(HeartbeatRequest) returns (HeartbeatResponse);

  // Session lifecycle
  rpc CreateSession(CreateSessionRequest) returns (SessionInfo);
  rpc ListSessions(ListSessionsRequest) returns (ListSessionsResponse);
  rpc JoinSession(JoinSessionRequest) returns (JoinSessionResponse);
  rpc LeaveSession(LeaveSessionRequest) returns (google.protobuf.Empty);

  // Session-scoped channel control
  rpc GetChannel(GetChannelRequest) returns (ChannelState);
  rpc SetChannel(SetChannelRequest) returns (CommandAck);
  rpc InjectEffect(InjectEffectRequest) returns (CommandAck);
  rpc CancelEffect(CancelEffectRequest) returns (CommandAck);

  // Capture + events
  rpc StartCapture(StartCaptureRequest) returns (CaptureInfo);
  rpc StopCapture(StopCaptureRequest) returns (CaptureInfo);
  rpc StreamEvents(StreamEventsRequest) returns (stream ServerEvent);

  // Liveness
  rpc Health(HealthRequest) returns (HealthResponse);
}

message ServerEvent {
  string event_id = 1;
  string session_id = 2;
  uint64 sample_index = 3;
  string type = 4;
  bytes payload_json = 5;     // canonical JSON for log
}
```

Tokens are carried in gRPC metadata (e.g., `authorization: Bearer
<token>`). Audio UDP packets carry the session id + station id +
short-lived lease id returned from `NegotiateAudio`.

---

## UDP Audio Packet

```
OtaAudioPacketHeader {
  uint32 magic;           // 'OASM'
  uint16 version;         // 1
  uint16 flags;
  uint64 lease_id;        // from NegotiateAudio
  uint64 seq;
  uint64 start_sample;
  uint32 sample_count;
  uint16 sample_format;   // float32 LE = 1
  uint16 channel_count;   // 1
}
```

Payload: `sample_count` interleaved samples. Server orders by
`start_sample` per stream, drops late duplicates, treats absent
TX for a tick as zero-signal contribution.

---

## Channel Models (Sealed Built-in Set)

| Model | Use |
|---|---|
| `passthrough` / `null` | Mixer correctness test fixture |
| `awgn` | Calibrated AWGN with seeded noise |
| `watterson_good` | Quiet HF, multipath + Doppler |
| `watterson_moderate` | Average HF |
| `watterson_poor` | Storm / auroral |

No public plugin registry. Adding a new model = writing C++ in
the channel core library, with unit tests.

---

## Live Effect Injection

Per-session operator commands (anyone in the session can issue;
no role distinction in MVP):

- `set_channel(model, snr_db, seed)` — replace the channel
- `inject_effect(fade_in_db, fade_out_db, duration_ms, ramp_ms)`
- `inject_effect(blackout_ms)`
- `inject_effect(level_change_db, ramp_ms)`
- `inject_effect(cfo_offset_hz, ramp_ms)`
- `inject_effect(delay_ms, ramp_ms)`
- `cancel_effect(effect_id)`

Effects are scheduled at server sample-index boundaries (not wall
time) for reproducibility within a session.

---

## Lobby

- Well-known session ID (default `"lobby"`, configurable in
  server config)
- Spawned on server startup with default channel from config
- Any valid token can join
- Any operator in the lobby can change the channel via `SetChannel`
  (no role distinction in MVP — friends are trusted)
- Default capacity: 16 stations (server config can override)
- Continuous capture: rolling 1-hour buffer (WAV + JSONL) so
  operators can grab "what just happened" without explicit
  capture-start

---

## Captures + Events

Per-session output layout:

```
captures/<session_id>/
  events.jsonl
  alice_rx_48k_f32.wav
  alice_tx_48k_f32.wav
  bob_rx_48k_f32.wav
  bob_tx_48k_f32.wav
  manifest.json    (server version, channel config, seed, station list)
```

No signed receipts. No claim levels. Captures + events + manifest
are sufficient evidence for a live session.

---

## Auth

`tokens.conf` (server config file):

```
# format: <token>:<callsign>:<label>
otasim_alice_token_random:8P9QC:Mathieu
otasim_bob_token_random:KC3VPB:Caleb
otasim_carol_token_random:N0CALL:Carol
otasim_dave_token_random:N0CALL:Dave
```

- Edit file + restart server to add/remove tokens (no SIGHUP
  reload in MVP)
- Tokens are opaque strings carried in gRPC metadata
- Server logs which token authenticated each session join
- No expiry, no revocation list (just delete from file + restart)

---

## Server-Internal Tests

The server's own tests verify the server as code, not the modem.
Small surface:

| Test file | What it verifies |
|---|---|
| `test_channel_core_models.cpp` | Seed in → expected output samples per model (passthrough, AWGN, Watterson) |
| `test_channel_core_mixer.cpp` | Passthrough = bitcopy; mixer summation correctness |
| `test_session_manager.cpp` | Create/join/leave/close lifecycle; SessionContext isolation |
| `test_auth_tokens.cpp` | Valid token accepted, invalid rejected, callsign extracted |
| `test_audio_plane.cpp` | UDP packet reordering by sample-index; late-drop policy |
| `test_otasim_serve_smoke.cpp` | Server starts, two test clients join one session, audio flows, captures produced, server shuts down cleanly |

These do **not** replace modem/protocol regression coverage —
that lives in `cli_simulator`.

---

## Client Refactors

### `ultra_gui -sim`

Currently embeds a two-station virtual simulator in-process. After
refactor: `-sim` mode connects as a single-station gRPC + UDP
client of `ota_simulator serve`. For two-station testing, run two
GUI instances pointing at the same server.

CLI:
```
ultra_gui -sim --ota-host host:port --token <token> --station-id alice
```

### `ultra_tnc --sim-audio`

New mode: TNC uses the server as audio backend (gRPC + UDP)
instead of SDL.

CLI:
```
ultra_tnc --sim-audio host:port --token <token> --station-id alice
```

### `cli_simulator` (refactored)

Currently embeds `SimulatedChannel` + two `ModemEngine` instances
in one process. After refactor: connects to `ota_simulator serve`,
spawns N modem clients per scenario, drives them, asserts on
outcomes. Same scenario JSON, same regression matrix, same
pass/fail semantics. Audio just routes through the server now.

This preserves all existing CI/regression infrastructure.
`cli_simulator` is NOT deleted.

---

## Timeline

| Week | Work |
|---|---|
| **1** | Channel core library extraction (`src/ota_channel_core/`) + unit tests + SessionContext + session manager + lobby spawn |
| **2** | gRPC service + UDP audio plane + token auth + per-session capture writer |
| **3** | `ultra_gui -sim` + `ultra_tnc --sim-audio` as real gRPC + UDP clients + integration smoke test |
| **4** | `cli_simulator` refactored to use server + first 4–5-operator live session over Tailscale |

~3–4 weeks of implementation. No V2 infrastructure.

---

## Deployment

**Local development:**
```
ota_simulator serve --bind 127.0.0.1:47000 --tokens tokens.conf
```

**Private network (Tailscale / Wireguard):**
```
ota_simulator serve --bind 100.x.x.x:47000 --tokens tokens.conf
```

Operators connect via the Tailscale IP, using their token from
the server admin (you share it out-of-band: Signal, email,
whatever).

**Sizing:** small instance is fine for MVP (~1–2 GiB RAM, a few
hundred MB disk for captures). Real sizing depends on session
count + capture retention; MVP defaults are bounded.

---

## What V1 Would Add (Not Now)

When/if the friend lab grows beyond 4–5 trusted operators, V1
could add:
- TLS for public-internet bind
- Token CLI with revocation
- Basic rate limiting
- Prometheus metrics + Grafana dashboard
- Container / systemd packaging
- Capture retention policy (TTL, S3 offload)

None of this blocks the 4-week MVP.

---

## What V2 Would Add (Even Further Out)

- KiwiSDR live ingestion as noise source
- Recorded-channel fixture library
- Web session viewer / spectator dashboard
- Federation across multiple servers
- Plugin channel model registry
- Conformance test suite for external modem implementations
- Hardware-in-the-loop bridge (dropped earlier — only revisit if
  there's a real need)

---

## Implementation Queue

Queued separately in `agents/queue/` after this plan doc lands:

1. **`01_otasim_channel_core_lib.md`** — Extract SimulatedChannel
   + mixer + channel models into `src/ota_channel_core/` static
   library; unit tests.
2. **`02_otasim_session_manager.md`** — SessionContext, session
   pool, lobby spawn on startup; in-process tests.
3. **`03_otasim_grpc_audio_auth.md`** — gRPC service + UDP audio
   plane + token validation against config file + per-session
   capture writer.
4. **`04_otasim_serve_binary.md`** — `ota_simulator serve` daemon
   entry point with SIGTERM drain, /healthz, integration smoke
   test.
5. **`05_ultra_gui_ota_client.md`** — Refactor `ultra_gui -sim`
   as real gRPC + UDP client; delete embedded simulator path.
6. **`06_ultra_tnc_ota_client.md`** — New `--sim-audio` mode for
   TNC.
7. **`07_cli_simulator_otasim_backend.md`** — Refactor
   `cli_simulator` to drive scenarios through the server's gRPC
   API; preserve existing scenario JSON and assertion semantics.

~6–7 tasks (down from 13 in the MVP-B plan).

---

## References

- `docs/OTA_SIMULATOR_DESIGN_LOG.md` — Round-by-round design
  conversation (Rounds 1–10, all closed)
- `docs/PROJECT_GOALS.md` — Mission filter
- `docs/INVARIANTS.md` — Modem invariants this preserves
- `docs/AI_COLLABORATION.md` — Claude ↔ Codex workflow
- `CLAUDE.md` — Project rules, no-backwards-compat policy
- v1 plan doc (1488 lines) preserved in git: commit `33f6b4e`
  and before
- v2 plan doc (681 lines, test-platform framing) preserved in
  git: commit `c486499`

---

## Document Conventions

This doc is the **single source of truth** for OTASim design.
Disagreements:
1. Reopen the relevant decision in the design log
2. Run a new design round (Round 11+) with Codex
3. Update this doc to match agreed decisions

Do not edit this doc unilaterally to reflect a design change
without a corresponding design log round.
