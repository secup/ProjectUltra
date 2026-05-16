# OTA Simulator Live Channel Plan

## Purpose

ProjectUltra should stop relying on the GUI's embedded two-station simulator for
serious modem testing. The GUI should behave as one station attached to an
external HF channel service. The external service should be based on
`ota_simulator`, so scripted tests and live tests share the same channel model,
capture logic, and regression vocabulary.

The desired outcome is:

```text
ota_simulator serve
  owns the simulated HF medium
  accepts live audio clients
  emits continuous receive audio
  accepts live control/injection commands
  captures evidence

ultra_gui -sim
  one local modem/protocol stack
  TX audio goes to ota_simulator
  RX audio comes from ota_simulator
  optional speaker monitor uses SDL only as a tap

ultra_tnc --sim-audio ...
  same simulated audio backend
  legacy TNC command/data TCP ports stay unchanged
```

SDL must remain the normal hardware-audio backend. The new work adds a second
audio backend for lab simulation; it does not replace SDL.

## Non-Goals

- Do not create a fake OS sound device as the first solution.
- Do not remove SDL audio support.
- Do not keep the GUI responsible for running a peer modem/protocol stack.
- Do not make `ota_simulator serve` instantiate ProjectUltra modem endpoints in
  live mode. It should be the medium, not the stations.
- Do not use lossy audio codecs for modem transport unless intentionally
  testing codec damage.
- Do not expose unauthenticated public internet ports for AWS tests.
- Do not let arbitrary connected stations change channel conditions or inject
  audio unless their role explicitly allows it.

## Current Repo Anchors

Existing pieces that should be reused:

- `tools/ota_simulator.cpp`
  - Current `gen` and `run` entry point.
- `tools/ota_simulator/scenario.*`
  - Scenario model, endpoint config, channel config, assertions, output config.
- `tools/ota_simulator/runner_v2.cpp`
  - Two-endpoint scripted scenario execution using `SimulatedChannel`.
- `tools/ota_simulator/scripted_audio_port.*`
  - Noise bed and injection logic.
- `tools/sim/simulated_station.hpp`
  - `SimulatedChannel`, `AudioPort`, `VirtualAudioPort`, and the current 10 ms
    sample cadence.
- `src/gui/audio_engine.*`
  - SDL device backend for real hardware and optional monitor playback.
- `tools/ultra_tnc.cpp`
  - Headless TNC runtime, currently SDL-backed.
- `src/gui/app.cpp`
  - Current GUI `-sim` embedded virtual station path to replace.

## Target User Workflows

### Local GUI Lab

```bash
./build/ota_simulator serve \
  --bind 127.0.0.1 \
  --port 47000 \
  --snr 15 \
  --channel good \
  --capture-dir /tmp/projectultra_ota_live

./build/ultra_gui \
  -sim \
  --sim-server 127.0.0.1:47000 \
  --sim-station alice
```

GUI `-sim` should show an OTA Simulator panel, not the old embedded peer
simulator. The panel should include:

- Server address.
- Station id.
- Connect/disconnect.
- RX/TX transport status.
- Server-reported channel metrics.
- Optional monitor audio checkbox.
- Optional SDL monitor output device.

### Local TNC Lab

```bash
./build/ota_simulator serve \
  --bind 127.0.0.1 \
  --port 47000 \
  --snr 15 \
  --channel good \
  --capture-dir /tmp/projectultra_ota_live

./build/ultra_tnc \
  --sim-audio 127.0.0.1:47000/alice \
  --callsign ALICE \
  --port 18300

./build/ultra_tnc \
  --sim-audio 127.0.0.1:47000/bob \
  --callsign BOB \
  --port 18400
```

Client applications still connect to the existing TNC command/data ports. They
should not need to know whether `ultra_tnc` is using SDL hardware audio or OTA
simulated audio.

### AWS or Remote Lab

Initial remote deployments should be behind WireGuard, Tailscale, or SSH
tunnels. A public internet mode can come later after auth, TLS, and transport
metrics are stable.

Example private-network command:

```bash
ota_simulator serve \
  --bind 0.0.0.0 \
  --port 47000 \
  --snr 15 \
  --channel moderate \
  --auth-token-file /etc/projectultra/ota_token \
  --capture-dir /var/log/projectultra/ota_live
```

Remote use requires network impairment reporting so modem failures are not
confused with packet loss or jitter.

### Shared Friend Lab

The live server should support trusted friends joining the same channel session
from different locations. This changes the design from "local test process" to
"small shared lab room."

Example:

```bash
ota_simulator serve \
  --bind 0.0.0.0 \
  --port 47000 \
  --room-id sunday-net \
  --auth-token-file /etc/projectultra/sunday_net_tokens \
  --max-clients 8 \
  --capture-dir /var/log/projectultra/sunday_net
```

Friend-lab requirements:

- Every client must authenticate.
- Most friends will connect as transmitting `station` participants, not just
  observers.
- Every transmitting station must have a unique callsign within the room.
- A station id may default to the normalized callsign, with a suffix only when
  the same operator needs multiple devices.
- Roles must separate normal transmitting stations from listen-only observers
  and from operators/admins who can change the lab conditions.
- Runtime injections must be attributed to the operator who issued them.
- Server logs must record joins, leaves, role, remote address, callsign, station
  id, and protocol version.
- The server must allow listen-only monitor clients that cannot transmit.
- The server must allow a moderator/operator to mute, kick, or deauthorize a
  station without stopping the session.
- Capture files must include enough metadata to reconstruct who was connected
  and what controls were changed during the run.

## Architecture

### Components

```text
src/ota_audio/
  ota_audio_protocol.hpp
  ota_audio_client.hpp/cpp
  ota_audio_server.hpp/cpp
  jitter_buffer.hpp/cpp
  pcm_codec.hpp/cpp

tools/ota_simulator.cpp
  gen
  run
  serve

tools/ultra_tnc.cpp
  SDL backend for hardware mode
  OTA audio backend for sim mode

src/gui/app.cpp
  SDL backend for hardware mode
  OTA audio backend for -sim mode
  optional SDL monitor output in -sim mode
```

Exact paths can change during implementation, but the module boundary should
remain: shared audio protocol/client code must not be buried in GUI code.

### Server Role

`ota_simulator serve` owns:

- The authoritative 48 kHz medium clock.
- Continuous idle noise generation.
- Per-station RX streams.
- Per-station TX ingestion.
- Sender-to-receiver path effects.
- Collision behavior by summing overlapping transmitters.
- Server-side captures and JSONL evidence.
- Channel and transport metrics.

`ota_simulator serve` does not own:

- ProtocolEngine state.
- ARQ.
- File transfer.
- Callsign routing beyond station metadata.
- GUI state.
- TNC command/data ports.

### Client Role

Each ProjectUltra station owns:

- ProtocolEngine.
- StreamingEncoder/StreamingDecoder or ModemEngine.
- TX waveform generation.
- RX decode.
- TNC command/data API if running `ultra_tnc`.
- Optional local audio monitor.

The client treats OTA audio as a replacement for the radio soundcard, not as a
replacement for the modem.

### Live Buses

`ota_simulator serve` should be modeled as a room with explicit buses:

```text
TX injection bus
  station audio clients push modem TX audio into the medium

RF/channel mixer
  server applies noise, fading, path effects, collision summing, and clipping

RX stream bus
  server sends each station the continuous audio it hears

Monitor bus
  observer clients subscribe to taps such as alice_rx, bob_rx, rf_mix, noise_only

Control bus
  authorized operators change channel conditions and inject impairments live
```

This structure keeps modem audio, human monitoring, and live test control from
being confused with each other. It also allows friends to connect as real
stations while an operator injects noise/QRM or changes path conditions during a
file transfer.

### Server Tick Loop

The server owns the clock. A useful first model is a 10 ms tick:

```cpp
for each station:
    tx_chunk[station] = pull_tx_audio_or_zero(station, tick);

for each receiver:
    rx = noise_source.render(receiver, tick);

    for each transmitter != receiver:
        path_audio =
            path_model[transmitter][receiver].process(tx_chunk[transmitter]);
        rx += path_audio;

    rx = clip_or_soft_limit(rx);
    send_rx_audio(receiver, rx);
    publish_monitor_taps(receiver, rx);
    capture_rx(receiver, rx);
```

Rules:

- Missing TX audio for a tick means zero transmitted signal for that station.
- RX audio is always produced, even during idle periods.
- A station does not hear its own TX unless self-monitor is explicitly enabled.
- Monitor clients receive copies of taps; monitor playback is not part of the
  modem decode path.
- Every tick should update metrics for active transmitters, clipping, queue
  depth, underruns, and network jitter.

## Audio Backend Model

Introduce a small transport-neutral audio interface around the operations the
modem runtime already needs.

Candidate API:

```cpp
class IAudioBackend {
public:
    virtual ~IAudioBackend() = default;

    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual void queueTxSamples(const std::vector<float>& samples) = 0;
    virtual std::vector<float> getRxSamples(size_t max_samples) = 0;
    virtual size_t getRxBufferSize() const = 0;

    virtual bool isRunning() const = 0;
    virtual std::string statusText() const = 0;
};
```

Implementations:

```text
SdlAudioBackend
  wraps current AudioEngine behavior for real soundcards

OtaAudioBackend
  wraps OtaAudioClient and connects to ota_simulator serve
```

This can be introduced incrementally. If a full interface is too invasive for
the first patch, add `OtaAudioClient` and wire it only into `ultra_tnc` first.
Do not let the GUI-specific code become the only implementation.

## GUI `-sim` Redesign

Current behavior:

```text
ultra_gui -sim
  shows simulator panel
  creates virtual_modem_
  creates virtual_protocol_
  runs embedded channel loop
```

New behavior:

```text
ultra_gui -sim
  shows OTA Simulator panel
  does not create virtual_modem_
  does not create virtual_protocol_
  connects local station audio to ota_simulator serve
```

TX path:

```text
ProtocolEngine callback
  -> local ModemEngine transmit
  -> OtaAudioBackend.queueTxSamples()
```

RX path:

```text
OtaAudioBackend.getRxSamples()
  -> local ModemEngine.feedAudio()
  -> local ProtocolEngine
```

Optional monitor path:

```text
same RX samples from OtaAudioBackend
  -> SDL output device
  -> regular speaker
```

Monitor playback must be a tap only. It must not feed the decoder through the
speaker/microphone path.

## TNC Sim-Audio Mode

Add one of these CLI shapes:

Preferred concise form:

```bash
ultra_tnc --sim-audio 127.0.0.1:47000/alice
```

More explicit form:

```bash
ultra_tnc \
  --audio-backend ota \
  --audio-server 127.0.0.1:47000 \
  --audio-station alice
```

Behavior in sim-audio mode:

- Do not open SDL input/output for modem audio.
- Do not key real PTT.
- Disable or simulate radio PTT behavior.
- Queue TX waveform samples to the OTA backend.
- Poll OTA RX samples into the decoder.
- Preserve existing TNC command/data TCP behavior.
- Preserve diagnostics and session summaries.

Optional monitor:

```bash
ultra_tnc \
  --sim-audio 127.0.0.1:47000/alice \
  --monitor-output default
```

## Live Transport

### Recommended End State

Use separate control and audio planes:

```text
Control plane: TCP/TLS
Audio plane: UDP datagrams
```

Control plane:

- Auth.
- Protocol version negotiation.
- Station registration.
- Channel config.
- Metrics.
- Capture control.
- Remote console sessions.
- Keepalive.
- Shutdown.
- Runtime injection commands.
- Room membership and role updates.

Audio plane:

- Fixed-size PCM packets.
- Sequence numbers.
- Sample timestamps.
- Jitter buffer.
- Loss/late packet detection.

This design avoids TCP head-of-line blocking for real-time audio. Late audio is
usually worse than missing audio because the modem clock must keep moving.

### MVP Transport

For localhost development, TCP-only is acceptable if the packet framing is
designed to be UDP-ready from day one.

MVP constraints:

- Every audio packet has `seq`.
- Every audio packet has `sample_time` or `sample_index`.
- Every audio packet has `sample_count`.
- Audio payload format is explicit.
- Missing or late packets can be represented in metrics even if TCP normally
  hides loss.

This allows the implementation to start simple while avoiding a protocol shape
that blocks UDP later.

### Audio Packet Format

Recommended audio cadence:

```text
sample_rate: 48000 Hz
channels: 1
frame_samples: 480
frame_duration: 10 ms
```

Recommended network PCM:

```text
int16 little-endian mono
```

Rationale:

- Float32 mono at 48 kHz is about 1.536 Mbps per direction per client before
  framing overhead.
- Int16 mono at 48 kHz is about 768 kbps per direction per client before
  framing overhead.
- The modem can convert int16 back to float internally.
- Int16 is easier to inspect and capture in common tools.

Avoid Opus or other lossy codecs for baseline modem testing.

Candidate packet header:

```cpp
struct OtaAudioPacketHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t stream_id;
    uint64_t seq;
    uint64_t sample_time;
    uint16_t sample_rate;
    uint16_t channels;
    uint16_t sample_format;
    uint16_t sample_count;
    uint32_t payload_bytes;
    uint32_t crc32;
};
```

For UDP, keep packets below the path MTU. A 480-sample int16 frame is 960 bytes
plus header, which is acceptable for typical Ethernet/VPN paths.

### Jitter Buffer Rules

Client TX to server:

- Client sends 10 ms audio frames when it has TX audio.
- If client has no TX audio, it can either send explicit silence/no-TX markers
  or rely on server timeout per tick.
- Server treats missing TX from a client as zero signal for that tick.

Server RX to client:

- Server sends continuous 10 ms RX frames.
- Client keeps a small jitter buffer before feeding the modem.
- Default target buffer can start at 40-80 ms for remote use.
- Localhost can use a smaller buffer.
- Late RX frames are dropped.
- Missing RX frames are filled with server-equivalent noise if possible, or
  local silence/noise with a metric flag.

Important metrics:

- `network_loss_pct`
- `network_late_packets`
- `network_jitter_ms_p50`
- `network_jitter_ms_p95`
- `rx_underruns`
- `tx_underruns`
- `server_tick_overruns`

These must be reported separately from:

- `channel_snr_db`
- `channel_type`
- `fading_index`
- `active_tx_count`
- `clipped_samples`

## Runtime Control and Injection

Dynamic injection is a core requirement, not a later convenience. The server
should accept live commands while stations are connected and while file transfers
are in progress.

### Remote Console

The server needs a remote console for operators. This is separate from station
audio and from monitor audio. It should connect to the control plane, authenticate
as an `operator` or `admin`, then provide a live view plus command entry.

Initial CLI console:

```bash
ota_simulator console \
  --server example:47000 \
  --callsign 8P9QC \
  --token-file ~/.config/projectultra/ota_console_token
```

Useful console commands:

```text
status
clients
metrics
paths
effects
captures
inject tone --to KC3VPB --freq 1475 --gain-db -12 --duration 20s
inject wav --to all --file hf_crash.wav --gain-db -6 --at now+5s
fade --path 8P9QC:KC3VPB --from-snr 20 --to-snr 5 --duration 30s
mute --callsign KC3VPB --duration 60s
cancel fx-00042
record start
record stop
watch events
watch spectrum --tap rf_mix
```

Console modes:

```text
read_only
  can view clients, metrics, events, paths, and captures; cannot change state

operator
  can inject effects, load scripts, change path/channel settings, and cancel
  own effects

admin
  can grant roles, mute/kick stations, stop captures, end the room, and cancel
  any effect
```

The console should be usable over VPN/SSH/Tailscale/AWS. A web dashboard can come
later, but the first console should be CLI-first so it works over a plain remote
shell and can be scripted.

Remote console safety rules:

- Every command must be authenticated and authorized by role.
- Every state-changing command must be assigned a command id.
- Every state-changing command must be logged before execution and again on
  completion or failure.
- Destructive room actions such as `kick all`, `record delete`, or `shutdown`
  should require `--confirm` or an interactive confirmation.
- The console should have a dry-run mode for scripts:

```bash
ota_simulator console --server example:47000 load-script qrm.json --dry-run
```

- The server, not the console client, is authoritative for command scheduling
  and sample times.

### Control Entry Points

Initial control can be a CLI speaking to the control plane:

```bash
ota_simulator control --server 127.0.0.1:47000 status
ota_simulator control --server 127.0.0.1:47000 clients
ota_simulator control --server 127.0.0.1:47000 metrics
```

Then add active controls:

```bash
ota_simulator control set-path \
  --path alice:bob \
  --snr 10 \
  --channel poor

ota_simulator control fade \
  --path bob:alice \
  --from-snr 20 \
  --to-snr 5 \
  --duration 30s

ota_simulator control inject-tone \
  --to bob \
  --freq 1475 \
  --gain-db -12 \
  --duration 20s

ota_simulator control inject-wav \
  --to all \
  --file fixtures/hf_crash.wav \
  --gain-db -6 \
  --at now+5s

ota_simulator control inject-impulses \
  --to alice \
  --rate 3/s \
  --peak-db -3 \
  --duration 60s
```

Control commands should be accepted by a running server without restarting the
session. Each accepted command must create a JSONL event with:

- command id.
- issuing user/station.
- wall-clock timestamp.
- server sample time.
- target station/path.
- parameters.
- authorization result.
- start/end sample time for scheduled effects.
- command source: `console`, `control-cli`, `live-script`, or future dashboard.

### Injection Targets

Supported targets should include:

```text
--to alice
--to bob
--to all
--to monitors
--path alice:bob
--path all:bob
--tap rf_mix
--tap noise_only
```

Receiver-targeted injection means "add this impairment to what the receiver
hears." Path-targeted injection means "affect only transmissions from this
sender to this receiver." Tap-targeted injection is for monitor-only experiments
and should not affect modem clients.

### Injection Types

Prioritize these live effects:

- `inject-tone`: single carrier, drifting carrier, or chirped tone.
- `inject-wav`: arbitrary WAV clip mixed into a station RX stream.
- `inject-noise`: temporary noise floor increase.
- `inject-impulses`: clicks/crashes with rate and peak controls.
- `fade`: smooth SNR/path change over time.
- `blackout`: drop a receiver or path for a duration.
- `mute`: administrative mute of a station TX.
- `level`: station or path gain change.
- `cfo`: path frequency offset change.
- `delay`: path propagation or buffer delay change.
- `network-impair`: packet loss/jitter injection separate from RF impairment.

Every effect should have:

- target.
- start time.
- duration.
- gain/level.
- optional seed.
- unique id.
- cancel command.

Example:

```bash
ota_simulator control cancel-effect --id fx-00042
```

### Scheduling

Operators need both immediate and scheduled controls:

```bash
--at now
--at now+5s
--at sample:1440000
--duration 30s
--repeat every:60s,count:5
```

Scheduled effects should be deterministic against the server sample clock. This
is important for replay and for comparing runs.

### Live Scenario Scripts

In addition to one-off control commands, `serve` should be able to load a script
of timed effects while clients are attached:

```bash
ota_simulator serve \
  --port 47000 \
  --live-script tests/fixtures/ota_simulator/live_fade_and_qrm.json
```

The same script format should also be injectable while running:

```bash
ota_simulator control load-script live_fade_and_qrm.json --start now+10s
```

This bridges deterministic `ota_simulator run` scenarios and human-driven live
sessions.

## Channel Semantics

The live channel should behave like a shared RF medium:

- Each station receives continuous audio.
- A station normally receives other stations' transmissions plus noise.
- A station should not receive its own TX unless loopback is explicitly enabled.
- Simultaneous transmitters are summed at each receiver.
- Collisions are physical audio collisions, not scheduler conflicts.
- Clipping should be measured and reported.
- Per-direction path models should be possible later.

Initial server options:

```bash
--snr 15
--channel awgn|good|moderate|poor|flutter
--seed 42
--noise-bed FILE.wav
--noise-bed-loop
--capture-dir DIR
--max-clients N
--loopback-self-audio false
```

Later options:

```bash
--path alice:bob:snr=12,channel=poor,cfo=20
--rx-blackout-model ptt
--tx-delay-ms 80
--level alice=-3db,bob=0db
--drop-rate 0.01
--jitter-ms 20
```

## Friends, Rooms, and Roles

Because friends may connect to a shared server, the live service needs basic
room and permission semantics.

### Rooms

The server should start one named room by default:

```bash
ota_simulator serve --room-id sunday-net --max-clients 8
```

The room defines:

- channel config.
- client limit.
- capture directory.
- auth policy.
- default role for authenticated callsigns, usually `station`.
- allowed monitor taps.
- whether unlisted callsigns may join.
- callsign collision policy.

Multi-room support can wait. A single process with one active room is enough for
the first remote lab.

### Roles

Suggested roles:

```text
station
  normal friend/user participant; may transmit modem audio and receive assigned
  RX audio

listen_only
  may receive one or more monitor taps, cannot transmit

operator
  may issue live injection and path-control commands

admin
  may change auth, kick/mute stations, end session, and manage captures
```

Role permissions should be explicit. A friend connecting as `station` is allowed
to transmit like a normal radio participant, but should not automatically be
allowed to inject QRM, change SNR, kick users, or modify room settings.

### Identity

Each connection should register:

- auth identity.
- callsign.
- station id.
- client type: `ultra_gui`, `ultra_tnc`, `monitor`, `control`.
- client version.
- protocol version.

For human participants, callsign is the primary identity shown in status,
captures, monitor taps, and logs. Station id is a transport/session identifier.
By default the station id should be derived from the callsign, for example:

```text
callsign=KC3VPB -> station_id=kc3vpb
callsign=8P9QC  -> station_id=8p9qc
```

If one operator connects multiple devices under the same callsign, use explicit
station ids:

```text
callsign=KC3VPB station_id=kc3vpb-gui
callsign=KC3VPB station_id=kc3vpb-tnc
```

The modem protocol still owns RF callsign behavior. The server uses callsigns
for room identity, authorization, logging, monitor labels, and collision
avoidance.

### Callsign Registration

For friend labs, a simple token file can map callsigns to roles:

```text
KC3VPB station token_hash=...
8P9QC station,operator token_hash=...
N0CALL listen_only token_hash=...
```

Open rooms can allow unlisted callsigns with `station` role, but closed rooms
should require pre-authorized callsigns. The server should reject duplicate
callsigns by default unless the connection presents an allowed multi-device
station id or an admin explicitly replaces the old connection.

### Moderation and Safety

Required control commands:

```bash
ota_simulator control clients
ota_simulator control grant-role --callsign 8P9QC --role operator
ota_simulator control revoke-role --callsign 8P9QC --role operator
ota_simulator control mute --callsign KC3VPB --duration 60s
ota_simulator control unmute --callsign KC3VPB
ota_simulator control kick --callsign KC3VPB --reason duplicate-id
ota_simulator control set-role --identity mathieu --role operator
ota_simulator control record start
ota_simulator control record stop
```

The server should protect the lab from accidental or abusive clients:

- unique station id enforcement.
- unique callsign enforcement for normal one-device users.
- max TX level per station.
- optional max duty cycle per station.
- per-client audio packet rate limits.
- command rate limits.
- console session idle timeout.
- console command audit log.
- capture disk quota.
- idle timeout.
- clean stale-client eviction.

### Friend-Friendly Monitor Mode

Allow people to join without transmitting:

```bash
ota_simulator monitor \
  --server example:47000 \
  --tap rf_mix \
  --output default
```

Monitor taps:

```text
alice_rx
bob_rx
alice_tx
bob_tx
rf_mix
noise_only
events
```

This lets friends listen to the simulated HF room or watch tests without adding
another transmitter.

## AWS and Security

AWS support should be planned, but the first public internet version must not be
unauthenticated raw audio.

Minimum private deployment:

- Bind to private network interface or VPN.
- Use WireGuard, Tailscale, or SSH tunnel.
- Save server-side captures.
- Include transport metrics in the session log.

Minimum public deployment before exposing ports:

- Auth token or mTLS.
- TLS for control plane.
- UDP audio session keys derived from authenticated control plane.
- Remote console disabled by default unless auth is configured.
- Remote console can be reached over VPN and can inject/cancel effects with
  operator credentials.
- Rate limits per station.
- Max client count.
- Capture storage limits.
- Clear logs showing callsign, station id, remote address, and negotiated
  protocol version.
- Role-based authorization for injection and admin commands.
- Optional allowlist of callsigns, station ids, and identities.

NAT notes:

- If clients connect outbound to AWS, UDP should work for most home networks.
- Keep control TCP open for session ownership and keepalive.
- If UDP fails, support TCP audio fallback with a warning that results are not
  equivalent under packet loss.

## Evidence and Captures

`ota_simulator serve` should write:

```text
capture_dir/
  session.jsonl
  server_metrics.csv
  alice_tx.wav
  alice_rx.wav
  bob_tx.wav
  bob_rx.wav
```

Session log events:

- server start.
- client connect/disconnect.
- client auth success/failure.
- remote console connect/disconnect.
- remote console command accepted/rejected.
- role assignment and role change.
- channel config.
- live control command accepted/rejected.
- live injection effect start/end/cancel.
- per-station TX active/inactive.
- active transmitter count.
- station mute/kick/admin actions.
- clipping events.
- transport loss/jitter/underrun events.
- capture file paths.
- clean shutdown.

This is important because remote tests can otherwise become ambiguous. A failed
decode must be attributable to channel impairment, modem behavior, or network
transport impairment.

## Additional Design Items

These are not all first-patch requirements, but they should be part of the design
review so the first implementation does not block them later.

### Versioning and Capability Negotiation

The control handshake should negotiate:

- protocol version.
- supported audio transports: TCP, UDP.
- supported PCM formats: int16, float32.
- supported frame sizes.
- supported monitor taps.
- supported injection commands.
- whether auth, TLS, and role enforcement are active.

Clients should fail clearly when a required capability is missing instead of
silently falling back to behavior that makes test results invalid.

### Calibration and Reference Levels

The server should expose its signal and noise calibration explicitly:

- modem reference RMS/power.
- configured SNR definition.
- current noise RMS.
- per-station TX RMS and peak.
- per-path gain.
- clipping/soft-limit policy.

This matters because throughput comparisons are only meaningful when SNR and
drive levels are comparable across local, hardware, and remote runs.

### Reproducibility

Every run should be replayable or at least diagnosable:

- session seed.
- per-effect seed.
- exact channel profile.
- exact live commands and sample times.
- client versions.
- server build version and git commit.
- capture checksums.

Live sessions with human/friend traffic will not be perfectly deterministic, but
the server should still preserve enough evidence to explain what happened.

### Reconnect and Late Join Behavior

The room should define what happens when a client disconnects and returns:

- same callsign reconnects within timeout.
- duplicate callsign connects while old session is stale.
- monitor joins after session start.
- station joins while others are already transferring data.
- client clock or packet sequence restarts.

Recommended default: reconnect replaces a stale connection for the same callsign
after a short timeout, but replacing an active connection requires admin action.

### Privacy and Recording Notice

Friend labs should make recording explicit. Server-side captures may contain
callsigns, modem payload metadata, and monitor audio.

The server should log a recording notice at join time, and clients should expose
that notice in GUI/TNC logs:

```text
room recording is enabled; TX/RX audio and control events are captured
```

### Room Presets

Add named room/channel presets so tests are repeatable:

```bash
--preset local-good
--preset aws-jitter-good
--preset poor-hidden-terminal
--preset qrm-fade-file-transfer
```

Presets should expand to explicit config in the session log so a run can be
recreated later without relying on a mutable preset definition.

### Health and Observability

The server should expose simple health views:

- server tick latency.
- queue depths.
- per-client packet loss/jitter.
- per-client TX/RX RMS.
- active effects.
- disk/capture usage.
- connected clients and roles.

Initial form can be `ota_simulator console status` and `metrics`. A structured
JSON status endpoint can come later for dashboards.

### Failure Policy

The server should define behavior for common failures:

- capture disk full.
- overloaded mixer tick.
- malformed audio packets.
- invalid effect parameters.
- unauthorized control command.
- auth token reload failure.
- client sends audio before registration completes.

Each failure should have a clear policy: reject client, drop packet, disable
capture, continue degraded, or terminate the room.

### Security Boundaries

Even in a friendly lab, do not treat station clients as trusted operators.

Default security posture:

- station role can transmit and receive only.
- listen-only role cannot transmit.
- operator role can inject/control impairments.
- admin role can manage people and room lifecycle.
- public bind requires auth.
- remote console requires auth.
- destructive commands require confirmation.

## Implementation Phases

### Phase 0: Design Review

Deliver this document and get review on:

- Module boundaries.
- Transport choices.
- Whether `IAudioBackend` should be introduced immediately.
- How much of `SimulatedChannel` should be extracted.
- Whether TCP-only MVP is acceptable before UDP audio plane.

### Phase 1: Extract Shared Live Channel Core

Create a reusable live-channel mixer that can be used by `ota_simulator serve`.

Responsibilities:

- Maintain station list.
- Accept per-station TX chunks.
- Produce per-station RX chunks every 10 ms.
- Reuse existing AWGN/Watterson/noise-bed behavior.
- Capture TX/RX streams.
- Report metrics.

Do not change GUI behavior in this phase.

### Phase 2: Add `ota_simulator serve` Local TCP MVP

Add:

```bash
ota_simulator serve \
  --bind 127.0.0.1 \
  --port 47000 \
  --room-id local \
  --snr 15 \
  --channel good
```

MVP can use TCP for both control and audio if packet framing is UDP-ready.

Acceptance:

- One server accepts two test clients.
- Server emits continuous RX frames.
- A TX appears in B RX.
- A TX does not appear in A RX unless configured.
- Simultaneous TX is mixed.
- Captures are written.
- Server exposes a local-only status/control socket or subcommand path.

### Phase 3: Add `OtaAudioClient`

Add reusable client code:

```cpp
class OtaAudioClient {
public:
    bool connect(const std::string& host,
                 uint16_t port,
                 const std::string& station_id,
                 const std::string& callsign);
    void disconnect();

    void queueTxSamples(const std::vector<float>& samples);
    std::vector<float> getRxSamples(size_t max_samples);
    size_t getRxBufferSize() const;

    bool isConnected() const;
    OtaAudioMetrics metrics() const;
};
```

Acceptance:

- Unit tests for packet encode/decode.
- Unit tests for RX ring behavior.
- Simulated server/client smoke test.
- No dependency on SDL.

### Phase 4: Add Runtime Control, Injection, and Roles

Add live control before wiring the GUI. Headless control is easier to validate
and is central to the value of `serve`.

Acceptance:

- `ota_simulator control status` lists server, room, clients, and metrics.
- `ota_simulator control clients` lists callsign, station id, role, client type,
  remote address, packet counters, and TX active state.
- `ota_simulator console` can connect remotely, authenticate, show status, and
  stream events.
- `inject-tone` can target one station while a session is running.
- `inject-wav` can target all stations while a session is running.
- `fade` can change one path over a timed duration.
- `cancel-effect` can stop a scheduled or active effect.
- Every control command is logged with identity, target, parameters, and sample
  time.
- Basic roles exist: `station`, `listen_only`, `operator`, `admin`.
- A `station` client cannot issue injection/admin commands.
- A read-only console cannot issue state-changing commands.
- Duplicate callsigns/station ids are rejected or require explicit admin
  replacement.

### Phase 5: Wire `ultra_tnc --sim-audio`

Implement headless TNC first because it is easier to test than the GUI.

Acceptance:

```bash
ota_simulator serve ...
ultra_tnc --sim-audio .../alice --port 18300
ultra_tnc --sim-audio .../bob --port 18400
```

Then:

- Connect from Alice to Bob.
- Send message.
- Send file.
- Verify byte-exact file reception.
- Confirm captures and session logs.
- Confirm normal SDL-backed `ultra_tnc` still works.

### Phase 6: Wire GUI `-sim` to OTA Backend

Change `-sim` meaning:

- Show OTA Simulator panel.
- Do not start the embedded virtual station.
- Connect local station to `ota_simulator serve`.
- Add optional monitor playback using SDL output.

Keep old embedded sim code present but unreachable or hidden for one transition
period if needed. Remove it after external simulation passes equivalent tests.

Acceptance:

- GUI can connect to server.
- GUI can hear optional monitor audio through speakers.
- GUI modem decodes RX directly from OTA backend, not from microphone.
- GUI can connect to a TNC peer through the server.
- GUI can send and receive a file through the server.

### Phase 7: Add UDP Audio Plane

Add UDP audio after the local TCP MVP is stable.

Acceptance:

- UDP packet sequence/loss detection.
- Jitter buffer tests.
- Late packet drop behavior.
- TCP fallback warning.
- Transport metrics appear in JSONL.
- Local loss/jitter injection tests can separate network impairment from RF
  channel impairment.

### Phase 8: AWS-Ready Packaging

Add deployment artifacts while keeping the source in this repo:

```text
packaging/docker/ota_simulator/
  Dockerfile
  README.md
```

Acceptance:

- Build container.
- Run `ota_simulator serve` in container.
- Bind local or VPN interface.
- Configure auth token.
- Capture directory mounted as volume.
- Remote `ultra_tnc --sim-audio` can attach over VPN.
- Remote friend stations can join with station role and transmit under their
  callsigns.
- Remote monitor clients can join with listen-only role.
- Operator/admin tokens are required for live injection commands.
- Remote console access is disabled or read-only when no operator/admin auth is
  configured.

## Test Plan

### Unit Tests

- Packet header serialization/deserialization.
- PCM int16/float conversion and clipping.
- Jitter buffer ordering.
- Jitter buffer late packet drop.
- RX ring buffer bounds.
- Metrics counters.
- Runtime effect scheduling and cancellation.
- Role/permission checks.

### Integration Tests

- Server accepts clients.
- Continuous idle RX frames.
- A-to-B routing.
- B-to-A routing.
- Self audio disabled by default.
- Simultaneous TX mixing.
- Capture writing.
- Server shutdown cleanup.
- Runtime `inject-tone` affects only the selected target.
- Runtime `inject-wav --to all` appears in all receiver captures.
- Runtime path fade changes metrics and decode conditions without restarting.
- Unauthorized station clients cannot issue injection commands.
- Duplicate callsigns/station ids are rejected or explicitly replaced by admin
  action.

### End-to-End Tests

- Two `ultra_tnc --sim-audio` instances exchange a message.
- Two `ultra_tnc --sim-audio` instances transfer a byte-exact file.
- Two or more friend/station clients can join the same room with unique
  callsigns and transmit.
- A listen-only monitor can hear `rf_mix` without transmitting.
- An operator can inject QRM during a file transfer and the event appears in the
  session log and captures.
- A remote console can inject and cancel an effect while friend stations remain
  connected.
- GUI `-sim` talks to `ultra_tnc --sim-audio`.
- Existing `ota_simulator run` fixtures still pass.
- Existing `cli_simulator` and relevant protocol tests still pass.
- Normal SDL hardware mode still starts and lists devices.

### Network Impairment Tests

- Inject packet loss.
- Inject jitter.
- Inject late packets.
- Inject server tick overrun.
- Confirm metrics distinguish network impairment from RF channel impairment.

## Compatibility and Migration

Short-term:

- Keep existing `ota_simulator run` behavior unchanged.
- Keep existing SDL hardware paths unchanged.
- Add new `serve` behavior.
- Add new `--sim-audio` or `--audio-backend ota` path.

Medium-term:

- Make GUI `-sim` external-only.
- Hide or remove embedded virtual-station GUI controls.
- Delete `virtual_modem_` and `virtual_protocol_` path after replacement is
  validated.

Long-term:

- Consider moving `ota_simulator serve` into a separate repository only after:
  - wire protocol is versioned,
  - client/server contract tests exist,
  - deployment cadence truly differs from ProjectUltra,
  - GUI and TNC integrations are stable.

Until then, keep it in ProjectUltra as a separate deployable binary.

## Review Questions for Claude Code

1. Should `IAudioBackend` be introduced before `OtaAudioClient`, or should the
   first patch wire `OtaAudioClient` directly into `ultra_tnc`?
2. Should the live channel core reuse `SimulatedChannel` directly, or should the
   current `SimulatedChannel` be split into a cleaner reusable mixer plus path
   model?
3. Is TCP-only MVP acceptable if the framing is UDP-ready, or should UDP audio
   be implemented immediately?
4. Is int16 the right network PCM format, or should local builds keep float32
   first and add int16 before AWS?
5. How should GUI monitor playback avoid blocking or underrunning when server
   RX stalls?
6. What is the safest migration path for removing the GUI embedded simulator
   without breaking current developer workflows?
7. Which tests should become required gates before deleting `virtual_modem_` and
   `virtual_protocol_`?
8. What is the minimum role/auth model needed before inviting friends to connect
   over VPN or AWS?
9. Should runtime injection commands share the same JSON schema as scripted
   `ota_simulator run` events, or should live effects use a new schema?
10. Should monitor clients receive PCM audio only, or should they also receive
    event/metrics streams for a future dashboard?
11. Should the first remote console be an interactive CLI only, or should it also
    expose a simple HTTP/WebSocket dashboard for read-only monitoring?

## Recommended First Patch

The first implementation patch should not touch GUI code.

Recommended scope:

1. Add packet definitions and encode/decode tests.
2. Add a local TCP `ota_simulator serve` skeleton.
3. Add a tiny test client or smoke test.
4. Prove continuous RX frames and A-to-B sample routing.
5. Write captures and JSONL session events.
6. Add a local-only `status` control command.

After that works, add one live injection command and basic roles, then wire
`ultra_tnc --sim-audio`. Only then should the GUI `-sim` path be replaced.
