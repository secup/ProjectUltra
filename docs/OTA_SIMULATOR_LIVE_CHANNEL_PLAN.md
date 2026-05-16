# OTA Simulator Live Channel Plan (v2 — Locked Design)

**Status:** Locked design as of 2026-05-16. Supersedes the v1 exploratory
draft (preserved in git history; see commit `33f6b4e` and prior).

**Decision authority:** `docs/OTA_SIMULATOR_DESIGN_LOG.md` (Rounds 1–5).
This document is **prescriptive**. The log is the historical record of
how we got here. Disagreements with this doc reopen the relevant OQ in
the log rather than modifying this doc unilaterally.

**Conversation summary:** Codex drafted a 1488-line v1 architecture →
Claude responded with 5 Tier-1 engineering contracts + 10 open
questions + 6 asks → Codex closed all 10 OQs and answered the asks →
Claude accepted with carve-outs and raised implementation-readiness
asks A11–A14 → Codex closed A11–A14 with first-task identification +
deletion table → Claude confirmed close-out. All 14 asks closed,
zero contested decisions.

---

## Purpose

ProjectUltra needs an external HF channel service so that:
- The GUI behaves as one station attached to a real medium, not a
  two-station hosting process
- Scripted tests, hardware-in-the-loop tests, and live multi-operator
  tests share the same channel model, capture format, and regression
  vocabulary
- Every result is a **reproducible experiment**, not a vibe
- Friends can join a shared session over private network (V1) or
  public internet (V2)

```text
ota_simulator serve
  owns the simulated HF medium
  accepts live audio clients (gRPC control, UDP audio)
  emits continuous receive audio per station
  accepts live control/injection commands
  captures evidence with signed receipts

ultra_gui -sim
  one local modem/protocol stack
  TX audio goes to ota_simulator
  RX audio comes from ota_simulator
  optional speaker monitor uses SDL only as a tap

ultra_tnc --sim-audio host:port/station_id
  same simulated audio backend
  legacy TNC command/data TCP ports stay unchanged
```

SDL remains the normal hardware-audio backend. The new work adds a
second backend for lab simulation; it does not replace SDL.

---

## Non-Goals

- Do not create a fake OS sound device as the first solution
- Do not remove SDL audio support
- Do not keep the GUI responsible for running a peer modem stack
- Do not let `ota_simulator serve` instantiate ProjectUltra modem
  endpoints in live mode — it is the medium, not the stations
- Do not use lossy audio codecs for modem transport
- Do not expose unauthenticated public internet ports
- Do not let arbitrary connected stations change channel conditions
- **Do not bridge real radios** (dropped 2026-05-16 — license
  complexity, audio level calibration, PTT timing over network)
- **Do not add a standalone console binary** (deleted from scope —
  gRPC API + `grpcurl` cover the same surface)
- **Do not implement dynamic role grant/revoke** (deleted — static
  allowlist only)
- **Do not implement monitor-tap-targeted impairment injection**
  (deleted — confounds evidence)

---

## Engineering Contracts (Non-Negotiable)

These five contracts define what makes OTASim an *instrument* rather
than just a convenience server. They are not features; they are
acceptance criteria for any code that ships under the OTASim name.

### T1.1 Determinism Contract

> **"Any scripted session is bit-exact reproducible from
> `(scenario_hash, server_version, seed, artifact_hashes)`."**

- All RNGs use named PRNG streams seeded from a master seed
  (e.g., `channel:path:alice:bob:awgn`)
- No `std::random_device` in the data path
- Server-authoritative sample clock; client packets timestamped to
  **sample index**, not wall time
- Mixer summation order is stable (canonical station id ordering)
- Single mixer thread advances the sample clock; network/UI threads
  cannot mutate channel state directly
- Live human sessions are evidence-preserving receipts, NOT bit-
  exact claims (unless TX/control inputs are recorded for replay)
- See Round 2 Item A2 in the design log for the full 14-item audit

**Acceptance test:** Run scenario X twice with identical inputs.
Byte-compare all captures and the deterministic event log. Any
difference fails the determinism gate.

### T1.2 Control Plane = gRPC + Protocol Buffers

- Audio plane stays separate (framed PCM, UDP for V1, TCP-framed
  PCM only as MVP localhost bridge)
- One `.proto` definition generates SDKs for C++ / Python /
  TypeScript clients
- `grpcurl` is sufficient for operator scripting; no separate
  console binary
- No bespoke TCP+JSON control surface

### T1.3 Canonical JSON + JSON Schema

- All scenarios, events, receipts, and provenance manifests are
  canonical JSON on disk
- Every file declares `schema_version`
- Server rejects unknown fields (no silent typo absorption)
- Hashes are computed over canonical JSON form
- YAML may exist only as authoring-tool UX layer (`ota_scenario`
  CLI / web editor) — never as a stored artifact, never hashed,
  never signed

### T1.4 Session Receipts

At session end, the server emits a signed JSON receipt:

```json
{
  "schema_version": "1",
  "receipt_id": "...",
  "session_id": "...",
  "claim_level": "deterministic | replayable_live | observational | diagnostic",
  "server_version": "0.7.2",
  "server_build_commit": "abc123",
  "scenario_hash": "sha256:...",
  "resolved_config_hash": "sha256:...",
  "rng": {
    "algorithm": "pcg64",
    "master_seed": "0xdeadbeef",
    "named_streams": {"channel:awgn:alice": "...", ...},
    "per_effect_seeds": {"effect_001": "...", ...}
  },
  "sample_clock": {
    "sample_rate": 48000,
    "start_sample": 0,
    "end_sample": 14400000,
    "duration_samples": 14400000,
    "time_scale": 1.0
  },
  "clients": [
    {"station_id": "alice", "callsign": "8P9QC", "client_type": "gui",
     "client_version": "0.7.2", "join_sample": 0, "leave_sample": 14400000}
  ],
  "inputs": {"scenario": "sha256:...", "noise_beds": ["sha256:..."]},
  "outputs": {
    "deterministic_event_log": "sha256:...",
    "full_event_log": "sha256:...",
    "captures": {"alice_rx": "sha256:...", "bob_rx": "sha256:..."}
  },
  "assertions": [{"id": "transfer_complete", "expected": "...", "pass": true}],
  "provenance_manifest_hash": "sha256:...",
  "signature": {"algorithm": "ed25519", "key_id": "...", "value": "..."}
}
```

**Claim levels:**
- `deterministic` — scripted scenario, bit-exact replayable;
  signature REQUIRED for CI/shared claims
- `replayable_live` — live session with full TX/control recording,
  replayable bit-exact; signature required for claims
- `observational` — live session, not replayable; unsigned allowed
- `diagnostic` — developer iteration; unsigned allowed, may NOT be
  cited as pass evidence

**Server invariant:** validator rejects unsigned receipts with
`claim_level=deterministic` or `replayable_live`. Server cannot
silently downgrade `claim_level`.

Optional fields (not required for claim-grade): wall-clock
timestamps, host/OS/CPU labels, operator display names, command
lines, KiwiSDR/HF metadata when applicable, OTel trace IDs.

### T1.5 Capture Provenance Manifests

Every capture set ships with a manifest covering:
- Simulator config + version
- Artifact hashes (raw audio, event log, scenario)
- Seeds (master + per-stream)
- Sample clock (rate, start/end)
- Software versions (server, clients)
- Source type (`synthetic` | `recorded` | `kiwi_live` | `passthrough`)

KiwiSDR-specific fields (frequency, receiver model, Kp index,
sunspot number, site) are conditional — required only when source
is `kiwi_live` or `recorded` from a Kiwi source.

---

## Architecture

```text
                       ┌──────────────────────────────────┐
                       │     ota_simulator serve           │
                       │  ┌────────────────────────────┐   │
                       │  │  Channel Core (library)    │   │
                       │  │   - station registry       │   │
                       │  │   - mixer (sample-index)   │   │
                       │  │   - channel models         │   │
                       │  │     · AWGN                 │   │
                       │  │     · Watterson g/m/p      │   │
                       │  │     · Noise-bed/replay     │   │
                       │  │     · Null/passthrough     │   │
                       │  │   - scripted effects       │   │
                       │  │   - capture writer         │   │
                       │  │   - event log              │   │
                       │  │   - receipt writer         │   │
                       │  └────────────────────────────┘   │
                       │  ┌────────────────────────────┐   │
                       │  │  gRPC control plane         │  │
                       │  │  (OtaSimulatorControl)      │  │
                       │  └────────────────────────────┘   │
                       │  ┌────────────────────────────┐   │
                       │  │  Audio plane                │  │
                       │  │  UDP (V1) / TCP-framed MVP  │  │
                       │  └────────────────────────────┘   │
                       └────────┬──────────────┬───────────┘
                                │              │
            ┌───────────────────┘              └──────────────────┐
            │                                                       │
   ┌────────────────┐                                     ┌────────────────┐
   │ ultra_gui -sim │                                     │  ultra_tnc     │
   │                │                                     │  --sim-audio   │
   │  ModemEngine   │                                     │  ModemEngine   │
   │  ProtocolEngine│                                     │  ProtocolEngine│
   │  OtaAudioClient│                                     │  OtaAudioClient│
   └────────────────┘                                     └────────────────┘
```

**Boundary discipline:**
- Server is the **medium**, never a station
- Clients own their modem + protocol stack
- Channel core is a **standalone library** that runs without
  networking (this enables deterministic CI tests)
- gRPC + audio plane are independent processes/sockets layered on top

---

## gRPC Control Plane (Illustrative Service)

```proto
syntax = "proto3";

package projectultra.otasim.v1;

import "google/protobuf/empty.proto";

service OtaSimulatorControl {
  rpc CreateSession(CreateSessionRequest) returns (SessionInfo);
  rpc LoadScenario(LoadScenarioRequest) returns (ScenarioValidation);
  rpc JoinSession(JoinSessionRequest) returns (JoinSessionResponse);
  rpc LeaveSession(LeaveSessionRequest) returns (google.protobuf.Empty);
  rpc NegotiateAudio(NegotiateAudioRequest) returns (AudioLease);
  rpc GetStatus(StatusRequest) returns (StatusResponse);
  rpc StreamEvents(EventStreamRequest) returns (stream SessionEvent);
  rpc SubmitCommand(ControlCommand) returns (CommandAck);
  rpc StartCapture(CaptureRequest) returns (CaptureInfo);
  rpc StopCapture(StopCaptureRequest) returns (CaptureInfo);
  rpc EndSession(EndSessionRequest) returns (SessionReceipt);
}
```

Detailed message definitions are deferred to the implementation
phase (first task: protobuf schema + receipt JSON Schema as part of
channel-core extraction).

---

## Channel Models (Sealed Set Through V1)

| Model | Class | Use |
|---|---|---|
| `passthrough` / `null` | mixer-test fixture | Validate mixer correctness without channel confounds |
| `awgn` | stochastic, deterministic w/ seed | Calibrated noise floor |
| `watterson_good` | stochastic, multipath + Doppler | Quiet HF |
| `watterson_moderate` | stochastic | Average HF |
| `watterson_poor` | stochastic | Storm/auroral conditions |
| `noise_bed_replay` | deterministic | Pre-recorded real-HF noise |
| Scripted effects | deterministic | Tone / fade / blackout / level / CFO / delay |

**OFDM_NARROW coverage requirement (MVP):** narrowband variants of
AWGN and Watterson-good must be available in the MVP since
OFDM_NARROW is a production-supported mode (per CLAUDE.md).

No public plugin registry until V2. No KiwiSDR live ingestion until
V2. Recorded noise-bed/replay covers most of the "real-HF flavor"
use cases without the lifecycle/license complexity.

---

## Audio Plane

**MVP (week 1–2):** localhost TCP-framed PCM. Simple, debuggable,
single-machine. Wire format is UDP-ready so the same packet
definitions reuse for V1.

**V1:** UDP audio plane with sequence numbers, sample-index
timestamps, and a jitter buffer.

**Audio packet header (binary):**

```
OtaAudioPacketHeader {
  uint32 magic;             // 'OASM'
  uint16 version;           // 1
  uint16 flags;
  uint64 session_id;
  uint32 stream_id;         // station+direction
  uint64 seq;
  uint64 start_sample;
  uint32 sample_count;
  uint16 sample_format;     // float32 LE = 1
  uint16 channel_count;     // 1
}
```

Payload: `sample_count` interleaved samples in `sample_format`.

**Jitter buffer:** server reorders by `start_sample`, drops late
duplicates deterministically. Absent TX for a tick contributes
zero signal (not silence-of-unknown-origin).

---

## Receipts + Provenance

See T1.4 above for receipt schema. Receipt-writer is a first-class
component of the channel core library — every scenario run produces
a receipt regardless of pass/fail.

**Storage:** receipts live next to captures in the session output
directory, named `receipt.json`. Signature lives in the same file.

**Verification:** `ota_simulator verify <receipt.json>` re-runs the
scenario (if `deterministic` claim level) and re-computes hashes,
exits 0 on match.

---

## Scenarios (Canonical JSON + JSON Schema)

Scenario file is one JSON document with `schema_version`, endpoint
definitions, channel selection, and an event timeline. Server
validates against schema on load — unknown fields fail loud.

Example (illustrative):
```json
{
  "schema_version": "1",
  "scenario_id": "two_endpoint_qso_snr15",
  "duration_s": 300.0,
  "channel": {"model": "watterson_good", "snr_db": 15.0, "seed": 0xdeadbeef},
  "endpoints": {
    "alice": {"callsign": "8P9QC", "initial_state": "DISCONNECTED"},
    "bob":   {"callsign": "KC3VPB", "initial_state": "DISCONNECTED"}
  },
  "events": [
    {"t_s": 0.0, "type": "command", "endpoint": "alice", "action": "connect_to", "peer_callsign": "KC3VPB"},
    {"t_s": 30.0, "type": "assert", "endpoint": "alice", "state": "CONNECTED"},
    {"t_s": 31.0, "type": "command", "endpoint": "alice", "action": "send_message", "text": "hello"}
  ],
  "output": {"session_log": "session.jsonl", "captures_dir": "captures/"}
}
```

YAML authoring tool may emit this JSON, but the YAML form is never
stored or hashed.

---

## Captures + Evidence

Every session writes:
- Per-station RX audio capture (WAV, float32, 48 kHz)
- Per-station TX audio capture
- Deterministic event log (JSONL, ordered by sample-index)
- Full event log (JSONL, includes wall-clock metadata excluded from
  deterministic hash)
- Receipt (JSON, signed for claim-grade)
- Provenance manifest (JSON)

Layout:
```
session_<id>/
  receipt.json
  manifest.json
  events_deterministic.jsonl
  events_full.jsonl
  captures/
    alice_rx_48k_f32.wav
    alice_tx_48k_f32.wav
    bob_rx_48k_f32.wav
    bob_tx_48k_f32.wav
```

---

## Determinism Acceptance Gate (MVP Ship Criterion)

`tests/test_otasim_determinism.cpp` runs the following scenarios
**twice each** with identical inputs and byte-compares all captures
+ the deterministic event log. Any mismatch fails the gate.

Minimum scenario set for MVP ship:
1. `passthrough_smoke` — mixer correctness, no channel
2. `awgn_snr15` — single stochastic channel, seeded
3. `watterson_good_snr15` — stateful fading, seeded
4. `noise_bed_replay` — file-driven, deterministic
5. `narrowband_awgn_snr8` — OFDM_NARROW coverage

All 5 must produce byte-identical output across runs to ship MVP.

---

## Roles + Auth (Staged)

**MVP:** single local/full-access token (file on disk). Localhost
binding only. No multi-tenant.

**V1:** static allowlist file mapping tokens to roles:
- `station` — can join a session, transmit, receive
- `listen_only` — can join, receive, never transmit
- `operator_admin` — can join, transmit, receive, plus inject
  effects, advance scripted events, end session, view full receipt

No dynamic role grant/revoke (deleted from scope). Operators
add/remove tokens by editing the allowlist file and reloading
(`ota_simulator reload-auth`).

**V2:** see "Operations Workstream" section below.

---

## Phasing: MVP / V1 / V2

### MVP (2-week local scripted instrument)

**Goal:** replace `cli_simulator`-style regression evidence. Local
only, no network. Used by Mathieu solo for regression testing.

- Extract `SimulatedChannel` into a standalone library
  (`libota_channel_core`)
- Channel models: passthrough, AWGN, Watterson good/moderate/poor,
  noise-bed replay, OFDM_NARROW variants
- Deterministic RNG infrastructure (seeded, named streams)
- Sample-index-authoritative mixer
- Capture writer (WAV + JSONL events)
- Receipt writer (unsigned for `diagnostic`, signed when key
  provided)
- Scenario loader with JSON Schema validation
- Protobuf service definitions (proto files only; no impl yet)
- Determinism acceptance gate test
- `ota_simulator run` command using the new library
- Headless Python test client (no SDL, no GUI) for CI

### V1 (private hosted lab over VPN/Tailscale)

**Goal:** Mathieu + KC3VPB + 1–2 friends can run shared sessions
over a private network.

- `ota_simulator serve` listens on configurable port
- gRPC control plane implementation
- UDP audio plane
- Jitter buffer + late-drop policy
- `ultra_tnc --sim-audio host:port/station_id`
- `ultra_gui -sim --ota-host host:port --station-id X`
- Static allowlist auth with three roles
- Live scripted scenario execution
- Live injection: tone, WAV, fade, blackout, level, CFO, delay
- Container/package for self-hosted deployment
- TLS termination (operator-provided cert) on gRPC

### V2 (research, scale, comfort)

**Goal:** public-internet hosted, research-grade reproducibility,
ecosystem features.

- Public-internet hardening (see "Operations Workstream")
- Plugin channel-model registry (`IChannelModel`)
- KiwiSDR live ingestion + recorded-channel fixture library
- Web session viewer/dashboard (HTML5 + WebSocket)
- OpenTelemetry tracing + Prometheus metrics
- Conformance test suite for external modem implementations
- Time-debugger primitives (pause/step/rewind/breakpoint)
- Multi-room hosting
- Paper-grade reproducibility packaging
- Deleting the old embedded GUI simulator path after V1 replacement
  gates pass

---

## Deletion Plan (MVP Ship)

Per `project_no_backwards_compat` memory — pre-deployment project,
no compatibility hedges. The following are deleted or refactored at
MVP ship:

| Path | Action | Rationale |
|---|---|---|
| `tools/cli_simulator.cpp` | Delete | Superseded by OTASim scenario/receipt evidence |
| `tools/threaded_simulator.cpp` | Delete | Duplicate harness with wall-clock scheduling |
| `tools/ota_simulator/clip_gen.cpp` | Delete | Legacy fixture generator |
| `tools/ota_simulator/clip_gen.hpp` | Delete | Header for clip_gen |
| `tools/ota_simulator/runner.cpp` | Delete | V1 single-endpoint runner superseded |
| `tools/ota_simulator/runner.hpp` | Delete | Header for runner |
| `tools/ota_simulator/scripted_audio_port.cpp` | Delete | Replaced by channel-core scheduling |
| `tools/ota_simulator/scripted_audio_port.hpp` | Delete | Header for scripted_audio_port |
| `tools/ota_simulator/runner_v2.cpp` | Refactor/move | Becomes new OTASim runner library |
| `tools/ota_simulator/runner_v2.hpp` | Refactor/move | Public API becomes new OTASim runner API |
| `tools/ota_simulator/scenario.cpp` | Refactor/move | Replace permissive parser with JSON Schema validation |
| `tools/ota_simulator/scenario.hpp` | Refactor/move | Scenario structs become schema-backed |
| `tools/ota_simulator/session_log.cpp` | Refactor/move | Replace ad hoc JSONL with deterministic event log |
| `tools/ota_simulator/session_log.hpp` | Refactor/move | Header for new artifact/receipt boundary |
| `tools/ota_simulator.cpp` | Rewrite in place | Binary name stays; legacy `gen`/old `run` dispatch removed |

**No legacy file kept as-is.** Every "keep" is debt.

CMake/CTest/docs/agent gates that reference `cli_simulator`,
`threaded_simulator`, or legacy `ota_simulator run` must be
retargeted or deleted in the MVP implementation series. SDL hardware
backend is NOT deleted; only the simulator harness using it is.

---

## Operations Workstream (V2, Separate Design)

The locked design covers up to V1 ("private friend-lab over VPN").
For real long-running public hosted operation, the following
concerns are **not yet designed** and need a separate design
log (`docs/OTA_SIMULATOR_OPERATIONS_LOG.md`) before V2 ships:

- Process supervision (systemd / container restart, readiness probes)
- Capture retention (TTL, S3 offload, signed-receipt-forever)
- Session limits (per-token concurrent sessions, max duration,
  idle kick, scenario size cap)
- Rate limiting (RPS, bandwidth, CPU/audio-hour quotas)
- Storage management (disk quotas, watermarks, cold archive)
- Auth at scale (token issuance API, revocation, expiry, audit log)
- Health/metrics (Prometheus full export, Grafana dashboard,
  alerting)
- TLS / cert renewal (ACME automation, rotation, mTLS for client
  identity)
- Update strategy (rolling restart with session drain, version
  pinning)
- Abuse handling (scenario content review, spam detection, kick/ban)
- Cost accounting (per-tenant tracking, billing hook)
- Audit log (tamper-evident admin action log, separate from session
  events)
- Privacy / PII (IP/callsign retention, GDPR erasure)
- DoS protection (connection caps, slow-loris timeouts, payload
  caps, WAF)
- Federation (cross-server session handoff, scenario marketplace)

**Do not address these in MVP or V1.** Friend-lab over Tailscale
does not need them. Open the operations design log when V1 is
deployed and there's a real demand signal for public hosting.

---

## Test Strategy

### Unit Tests (MVP)
- Channel model determinism (seeded RNG produces identical output)
- Mixer correctness (passthrough produces bitcopy)
- Capture writer (WAV bytes match expected for known input)
- JSON Schema validator (rejects unknown fields)
- Receipt signing/verification

### Integration Tests (MVP)
- Determinism acceptance gate (5 scenarios × 2 runs, byte-compare)
- `ota_simulator run <scenario>` end-to-end produces valid receipt
- Python test client connects, sends audio, reads audio, validates
  capture

### End-to-End Tests (V1)
- Two `ultra_tnc --sim-audio` instances complete a file transfer
  through the server
- `ultra_gui -sim` connects, displays waterfall, decodes messages
- Live injection (operator role) modifies channel mid-session

### Performance Tests (V1)
- Server sustains 4 concurrent sessions × 2 stations each at 48 kHz
  on commodity hardware
- Audio plane latency under 50 ms one-way at LAN speeds
- Memory bounded under 24-hour soak

---

## Compatibility and Migration

**MVP ship is the cutover.** No wrapper, no compatibility shim. Per
Round 4 decision (option B):

1. MVP development proceeds alongside `cli_simulator`
2. When MVP determinism + protocol evidence gates pass:
   - Delete `cli_simulator` from build
   - Delete test references
   - Delete doc references
   - Delete agent-gate references
   - Rewrite tests that expected old CLI surface to use OTASim
     scenarios + receipts + captures
3. The shipped MVP is the new evidence path. Period.

There is no transition period during which both paths produce
"evidence". Two evidence paths = ambiguous results = 2 AM bug
report ambiguity.

---

## Implementation Queue

First implementation task (queued separately in `agents/queue/`):

**`01_otasim_channel_core_extraction.md`** — Extract the channel
core into a standalone library. Scope:
- Move `SimulatedChannel` + audio port + mixer logic from
  `tools/sim/` and `tools/ota_simulator/` into
  `src/ota_channel_core/` (new directory)
- Build as a static library `libota_channel_core.a`
- Add seeded RNG infrastructure (named streams)
- Add sample-index-authoritative scheduling
- Implement the 5 MVP channel models (passthrough + AWGN +
  Watterson g/m/p + noise-bed-replay)
- Add JSON Schema for scenarios + events + receipts +
  provenance manifests
- Add receipt writer (unsigned MVP; signing optional)
- Implement determinism acceptance gate
  (`tests/test_otasim_determinism.cpp`)
- Update `ota_simulator run` to use the new library

**Acceptance:** Gate passes (5 scenarios × 2 runs byte-compare).
`cli_simulator` still works (this task does not delete it yet).

Subsequent tasks (queued after #1):

- **`02_otasim_cmake_ctest_retarget.md`** — Purge legacy simulator
  references from CMake/CTest/docs/agent gates
- **`03_otasim_grpc_control_plane.md`** — Implement gRPC service
- **`04_otasim_audio_plane_mvp.md`** — Localhost TCP-framed audio
- **`05_otasim_serve_command.md`** — `ota_simulator serve` entry
  point with gRPC + audio plane
- **`06_otasim_python_client.md`** — Headless test client

---

## References

- `docs/OTA_SIMULATOR_DESIGN_LOG.md` — Round-by-round design
  conversation (closed Round 5)
- `docs/PROJECT_GOALS.md` — Mission filter
- `docs/INVARIANTS.md` — Modem invariants this preserves
- `docs/AI_COLLABORATION.md` — Claude ↔ Codex workflow
- `CLAUDE.md` — Project rules, no-backwards-compat policy
- v1 draft preserved in git history (commit `33f6b4e` and prior
  conversations referenced in the design log)

---

## Document Conventions

This doc is the **single source of truth** for OTASim design.
Disagreements:
1. Reopen the relevant OQ in the design log
2. Run a new design round (Round 6, 7, ...) with Codex
3. Update Decisions Locked when both sides agree
4. Then update THIS doc to match

Do not edit this doc unilaterally to reflect a design change
without a corresponding design log round.
