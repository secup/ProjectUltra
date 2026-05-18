# ProjectUltra Change Log

This log tracks all bug fixes and behavioral changes to prevent re-doing work due to lost context.

**Format:** Each entry must include:
1. What was broken (symptom + root cause)
2. What was changed (files, code)
3. How it's properly fixed (why it works, invariants)
4. Test verification (command + expected output)

---

## 2026-05-18: OTASim admin role + otasim_ctl admin CLI

**Fixed:** Any authenticated OTASim token could call destructive RPCs
(`SetChannel`, `InjectEffect`, `CancelEffect`, `CreateSession`,
`StartCapture`, `StopCapture`) — meaning any joined operator could
reconfigure the channel mid-QSO, kill an effect that another operator
just injected, or start/stop session recordings. Not safe for the
multi-operator friend-lab deployment the OTASim design was built for.

**Added:** Two-level RBAC on the token allowlist.

- Token-file format gains an optional 4th field for the role:
  ```
  alice_tok:ALPHA:Alpha station                     # implicit operator
  bob_tok:BRAVO:Bravo station:operator              # explicit operator
  admin_tok:ADMIN:operator + admin:admin            # admin role
  ```
  Existing 3-field lines remain valid as operator-role.

- `AuthPrincipal` gains a `bool admin` field. Default `false`.

- `requireAdmin(principal)` helper in `OtaSimulatorService` returns
  `PERMISSION_DENIED` with an actionable error message when an operator-
  role token attempts an admin-only RPC.

- Admin-only RPCs (all 6 destructive ones above) now call
  `requireAdmin(principal)` immediately after `authenticate()`.
  Read-only and audio-path RPCs (`RegisterStation`, `NegotiateAudio`,
  `Heartbeat`, `ListSessions`, `JoinSession`, `LeaveSession`,
  `GetChannel`, `Health`, `StreamEvents`) remain available to any
  authenticated token.

**Added:** `tools/otasim_ctl.cpp` — small admin CLI for the OTASim
server. Subcommands: `health`, `list-sessions`, `get-channel`,
`set-channel`. Useful for live demos (bump SNR without restarting),
ops debugging, scripted scenarios. Token via `--token` or
`OTASIM_TOKEN` env var.

```
./build/otasim_ctl --token admin_tok set-channel --model awgn --snr 20
./build/otasim_ctl --token admin_tok set-channel \
    --model watterson_moderate --snr 12
./build/otasim_ctl --token alpha_tok get-channel
./build/otasim_ctl --token alpha_tok list-sessions
```

**Verification:**

```bash
cmake --build build -j4
ctest --test-dir build -R "AuthAllowlist|OtasimServe|UltraGuiOta|UltraTncSimAudio|SessionContext" \
    --output-on-failure -j1   # 9/9 pass

# Manual end-to-end check (with the server running):
./build/otasim_ctl --token alpha_tok set-channel --model awgn --snr 20
# -> "SetChannel failed: admin role required for this RPC; token 'ALPHA' is operator-only"
./build/otasim_ctl --token admin_tok set-channel --model awgn --snr 20
# -> "ok session=lobby model=awgn snr_db=20.00"
./build/otasim_ctl --token alpha_tok get-channel
# -> "session=lobby model=awgn snr_db=20.00 ..."
```

**Migration:** existing token files keep working — operator role is the
default. Add an admin entry to a new token file line when you want a
principal that can also reconfigure the channel.

---

## 2026-05-18: OTASim two-station GUI connect end-to-end

**Fixed:** Two `ultra_gui -sim` instances pointed at the same
`ota_simulator serve` daemon could PING/PONG but never complete the
CONNECT/MODE_CHANGE handshake. The connect attempt would stall and the
two GUIs sat in state 1 (PING_SENT) or state 2 (PONG_RECEIVED) for the
session lifetime.

**Root causes (three independent bugs, all compounding):**

1. **OTASim client RX buffer cap was 20 s.**
   `kMaxRxBufferSamples = 960000` at 48 kHz meant the client could
   silently accumulate up to 20 seconds of audio before dropping any
   samples. Server's session-clock tick emits continuous samples at
   real-time rate (silence + audio, like a real soundcard); when the
   GUI render loop briefly stalled (waterfall scroll, ImGui spike) the
   audio piled up and never recovered. Real audio frames then sat
   behind multi-second silence, well past the ARQ timeouts and the
   modem's sync-search window.

2. **`-sim` mode left modem callsign at default `8P9QC`.**
   The GUI's `Connect to <remote>` uses the OTASim `--station-id` as
   the destination callsign in the frame header. But the modem's local
   callsign defaulted to `8P9QC` from settings; nothing forced it to
   match the `--station-id`. `deliverFrame()` parses the header, sees
   `dst=ALPHA` vs `local=8P9QC`, classifies the frame as "different
   station", and drops it silently at TRACE level. LDPC was decoding
   3/3 CWs successfully, the frame was then dropped before reaching
   the protocol layer.

3. **`--log-file` only captured the App-constructor startup logs.**
   `App::initLog()` unconditionally called `ultra::setLogFile(g_gui_log_file)`
   after opening `logs/gui.log`, overriding whatever `main_gui.cpp` had
   set from `--log-file`. So per-station log files would receive ~12
   lines of modem init and then go silent for the rest of the session,
   making per-station debug impossible without juggling working
   directories.

**Changed:**

- `src/otasim_client/ota_audio_backend.cpp` — `kMaxRxBufferSamples`
  reduced from `960000` (20 s) to `23040` (480 ms). Behaves like a
  real soundcard's driver buffer: continuous samples in, consumer
  drains at real-time rate, oldest drops on consumer stall. 480 ms
  ≈ 8x a 60 Hz render budget, which absorbs typical jitter without
  building multi-second latency.
- `src/otasim_client/ota_audio_backend.{cpp,hpp}` — optional
  `#ifdef ULTRA_OTASIM_AUDIO_DIAGNOSTICS` counters log RX queue depth
  every 100 packets. Off by default.
- `src/gui/app.cpp` — in `-sim` mode with non-empty `--station-id`,
  force the modem's local callsign to the station id (overrides the
  settings callsign for the protocol-address check only). Without this,
  every inbound frame is dropped as "different station".
- `src/gui/app.cpp` — `initLog()` adopts `ultra::g_log_file` if it is
  already set externally (by `main_gui.cpp`'s `--log-file` parser),
  instead of blindly opening `logs/gui.log` and clobbering the user's
  chosen sink.
- `tests/test_ultra_gui_ota_client.cpp` — extended to time the
  passthrough latency (must be < 150 ms in-process) and to bound the
  idle RX backlog at the new soundcard-like cap.

**ACK diversity + CONNECT_ACK rescue retry are intentional.** Once the
handshake completes you will see each ACK delivered twice (~440 ms
apart) and one proactive CONNECT_ACK re-send. Both mechanisms exist
for real HF where the dominant loss mode is plain cumulative ACKs
disappearing into a fade — see comment in
`src/protocol/selective_repeat_arq.cpp:1266`. On OTASim's clean AWGN
channel they are visible but harmless; SR-ARQ correctly de-duplicates
at the base/bitmap level. Do not propose disabling them.

**Verification:**

```bash
cmake --build build -j4
ctest --test-dir build -R "Otasim|UltraGuiOta|UltraTncSimAudio|SessionContext" \
  --output-on-failure -j1   # 3/3 (or 4/4) pass

# Manual two-station QSO over OTASim (localhost):
./build/ota_simulator serve --bind 127.0.0.1:50051 --udp-bind 127.0.0.1:50052 \
  --tokens /tmp/ota_tokens.conf &
./build/ultra_gui -sim --ota-host 127.0.0.1:50051 \
  --station-id ALPHA --token alpha_tok --monitor-audio \
  --log-file /tmp/alpha.log --log-level debug &
./build/ultra_gui -sim --ota-host 127.0.0.1:50051 \
  --station-id BRAVO --token bravo_tok --monitor-audio \
  --log-file /tmp/bravo.log --log-level debug &
# In one GUI: Connect to other station.
# Expected: both reach state 3 (CONNECTED), MODE_CHANGE to OFDM-CHIRP
# DQPSK R1/4, in-session ACKs decode in OFDM control profile.
```

Verified end-to-end on 2026-05-18: full PING → PONG → CONNECT →
CONNECT_ACK → MODE_CHANGE → CONNECTED on two macOS GUIs against a
local `ota_simulator serve` daemon.

---

## 2026-05-15: CONNECT call-collision handling

**Fixed:** Inbound CONNECT frames arriving while the local station was in
`PROBING` were rejected as "busy" at
`src/protocol/connection_handlers.cpp:103-145`, producing CONNECT_NAK even
though the outbound probe was just the symmetric call attempt. `handleConnect`
now cancels the outbound probe with `cancelOutboundProbe()` at
`src/protocol/connection_handlers.cpp:77-86` and falls through to the normal
responder path.

**Fixed:** The true simultaneous CONNECT race now resolves deterministically by
callsign order at `src/protocol/connection_handlers.cpp:109-138`. The
lexicographically lower callsign keeps its outbound CONNECT attempt and ignores
the inbound CONNECT; the higher callsign cancels its outbound CONNECT with
`cancelOutboundConnect()` at `src/protocol/connection_handlers.cpp:88-97` and
accepts as responder. No wire format, PING/PONG, CONNECT encode/decode, PHY, or
modem path changed.

**Verification:** Added `ConnectionCallCollision` in
`tests/test_connection_call_collision.cpp` covering the live PROBING collision
and the simultaneous CONNECT tiebreaker. `cmake --build build -j4` passed and
`ctest --test-dir build --output-on-failure -j4` passed `53/53`.

---

## 2026-05-15: PONG-TX half-duplex timing race fix

**What was broken:** A disconnected station that decoded an incoming PING fired
the `on_ping_received_` callback synchronously, so GUI/TNC PONG TX could start
before the peer radio had finished PTT-off and RX-path settling. In the real-HF
repro, the operator-side symptom was "ping sent, no response decoded
peer-side": 4 consecutive PING retries over 60 s, 4 local PONGs, and 0 CONNECTs.

**What changed:** `ConnectionConfig::pong_tx_delay_ms` defaults to 500 ms at
`src/protocol/connection.hpp:31`, with pending callback state at
`src/protocol/connection.hpp:332-333`. The DISCONNECTED PING/PONG branch now
schedules or immediately fires the callback at
`src/protocol/connection_handlers.cpp:39-50`, and `Connection::tick()` drains
the deferred callback before the existing state timers at
`src/protocol/connection.cpp:1359-1373`. Cancellation is centralized at
`src/protocol/connection.cpp:1573-1579` and runs on `connect()` plus connected,
disconnect/reset paths.

**Why it is properly fixed:** The wire format and modem PONG waveform are
unchanged; only the protocol-layer callback schedule moved. A re-PING while the
callback is pending restarts the delay, and `pong_tx_delay_ms=0` preserves an
immediate operator override.

**Verification:** `cmake --build build -j4` passed. `ctest --test-dir build
--output-on-failure -j4` passed `56/56`, including
`ConnectionPongDelayDeferred`, `ConnectionPongDelayCancelOnConnect`,
`ConnectionPongDelayRepingRestarts`, `ConnectionPongDelayZeroDelay`,
`Protocol`, and `ConnectionAdaptive`.

---

## 2026-05-15: GUI image send presets

**Added:** The GUI send-file path now detects JPEG/PNG inputs and presents
operator-selectable send presets before queueing image bytes: Thumbnail
(`320x240`, JPEG q=70), Preview (`640x480`, JPEG q=75), or Full size
(original file). Non-image files keep the previous direct `sendFile()`
behavior.

**What changed:** Added `src/gui/image_util.*` with magic-byte sniffing,
`stb_image_info` metadata reads, and gamma-correct
`stbir_resize_uint8_srgb` resize plus JPEG encode. Wired the vendored STB
headers into CMake, added `tests/test_image_util.cpp` with a tiny JPEG
fixture, and added an ImGui modal around the existing file Send button.

**Why it is properly fixed:** Wire format and `FileTransferController` stay
unchanged; resized images are written to temp JPEG files and sent through the
existing byte-transparent file-transfer path. Wire-time estimates use
`ProtocolEngine::getCurrentBitrate_bps()` and label the 1400 bps fallback as
an R1/2 Good estimate so operators do not get unmarked pre-connection timing.

**Verification:** `cmake -S . -B build`, `cmake --build build -j4`, and
`ctest --test-dir build -R ImageUtil --output-on-failure` pass. Native GUI
manual smoke could not be completed in this sandbox because SDL reports
`The video driver did not add any displays`.

---

## 2026-05-15: ota_simulator data-mode auto-ladder fix

**Fixed:** `initial_mode` now selects the MC-DPSK handshake preset without forcing post-CONNECT data mode; `force_data_mode` is default-false parser plumbing and the runner gate is at `tools/ota_simulator/runner_v2.cpp:417-420` (the brief's `:607-608` force calls).
**Fixed:** `tests/fixtures/ota_simulator/two_endpoint_noisy_handshake.json:13,24` opts into `force_data_mode` to preserve the real-HF R1/4 survival test, and the OTA CTest cases are marked `RUN_SERIAL` because they are wall-clock DSP simulations.
**Verification:** `ctest --test-dir build --output-on-failure -R OTASimulator` passed 4/4; after CTest reconfigure, `ctest --test-dir build --output-on-failure -j4 -R OTASimulator` passed 4/4. A same-waveform decoder rebuild exposed by repeated noisy runs is fixed at `src/gui/modem/streaming_decoder.cpp:428-447`.

---

## 2026-05-14: MC-DPSK real-HF hardening Phase 1 harness fix

**What was broken:** `ota_simulator` v2 `noise_bed` scenarios without
`channel.snr_db` added the WAV overlay to the channel but left station SNR
metadata at the simulator default `20 dB`. The adaptive ladder therefore
negotiated OFDM-CHIRP for real-HF connected data even when decoded CONNECT
frames reported roughly `-4 dB` idle-noise SNR.

**What changed:** `tools/ota_simulator/runner_v2.cpp` now scales the loaded
noise bed to `target_rms`, estimates station SNR from the scaled 50-2950 Hz
FIR-bandpassed noise against `sim::kModemReferencePower`, and applies that
metadata to both endpoints when no explicit `channel.snr_db` is present.
Explicit `channel.snr_db` still wins.

**Why it is properly fixed:** The harness now uses the same finite-FIR energy
normalization documented for the idle-noise SNR estimator instead of treating
real-HF in-band energy as a harmless full-band RMS overlay. No wire format,
channel calibration constant, mode-ladder threshold, or OFDM decoder behavior
changed.

**Verification:** Before the patch,
`/tmp/ota_realhf_sweep/realhf_snr5.json` negotiated `OFDM-CHIRP` at default
`SNR=20.0 dB` and failed 3 assertions. After the patch,
`./build/ota_simulator run --scenario /tmp/ota_realhf_sweep/realhf_snr5.json`
prints `noise_bed station_snr_db=-3.98699`, negotiates/enters `MC-DPSK`,
decodes the DATA message, and passes all assertions. `cmake --build build
--target ota_simulator -j4` is clean.

**Phase 2 forced-waveform diagnostic path:** Added
`channel.force_connected_waveform` for v2 scenarios and wired it to the
existing CONNECT preferred-mode negotiation. Added
`tests/fixtures/ota_simulator/two_endpoint_mcdpsk_realhf_snr_minus3.json`,
which forces MC-DPSK over the real-HF noise bed at measured
`station_snr_db=-3.00088`. Verification:
`./build/ota_simulator run --scenario
tests/fixtures/ota_simulator/two_endpoint_mcdpsk_realhf_snr_minus3.json`
passes; logs show `Using remote preferred mode: MC-DPSK`, DATA `4/4 CW`
decoded at `snr_db=-3.22`, ACKs decoded, and clean disconnect. Captured
session JSON: `/tmp/phase2_mcdpsk_realhf_snr_minus3_session.jsonl`.

---

## 2026-05-14: MC-DPSK idle-noise SNR meter local validation complete

**What was broken:** Non-OFDM frames, including MC-DPSK CONNECT and
CONNECT_ACK, still published the chirp-correlation SNR. On calibrated
AWGN SNR 15, the chirp matched filter reads about `27.9 dB`; the
handshake therefore carried a saturated number rather than an honest
receiver noise-floor measurement.

**What changed:** Added `IdleNoiseSNREstimator`, wired it into
`StreamingDecoder`, and substituted its value in `populateDecodeMetrics()`
for non-OFDM frames when an idle estimate is available. The estimator uses
the same 101-tap Blackman FIR bandpass family as the input filter and
documents the coefficient-energy normalization at the correction site:
`E{y^2} = sigma^2 * sum(h^2)`, so idle filtered RMS is divided by the
actual FIR energy before comparing to `kModemReferenceRms^2`. No wire
format, channel calibration, or mode-ladder threshold changed.

**Validation through local Phase 5:**

| Check | Result |
|-------|--------|
| Phase 1 AWGN probe | `idle_snr_probe` measured configured SNR +0.04 dB across -5..20 dB AWGN |
| Phase 2 decoder wiring | `idle_snr_probe --streaming` measured configured SNR -0.01 dB across -5..20 dB AWGN |
| Phase 3 CTest | `ChannelIdleNoiseSNRCalibration` PASS: AWGN mean bias -0.03 dB, Good/Moderate -0.02 dB |
| Phase 4 protocol context | AWGN15 CONNECT/CONNECT_ACK logs show `chirp_snr=27.9 dB idle_snr=15.0 dB`; decoded frame SNR publishes `15.0 dB` |
| Phase 5 mode picks | Good15 10/10 `OFDM-CHIRP DQPSK R1/2`; Moderate15 10/10 `OFDM-CHIRP DQPSK R1/2`; Good10 10/10 `OFDM-CHIRP DQPSK R1/4` |

**Phase 6 — Pi5 hardware smoke (2026-05-14):** branch synced to
pi5tester, rebuild clean. Audio path verified within spec
(Pi→Mac RMS=0.123 peak=0.305; Mac→Pi RMS=0.250 peak=0.850).
`AGENT_HW_AUDIO_CHECK=0 ./agents/run_hardware_smoke.sh`: 3/3 PASS
(AWGN/Good/Moderate × R1/2 SNR=15 1KB). Report bundle:
`agents/reports/hardware_20260514_194039/`. Full ctest 51/51 PASS
including new `ChannelIdleNoiseSNRCalibration` (±1.5 dB AWGN,
±3 dB Good/Moderate).

**Status:** **ready for review and merge.** Combined with the prior
`feat/calibrated-snr-meter` (OFDM honest SNR), this completes the
honest-SNR-everywhere stack. After both branches merge, GUI display,
MODE_CHANGE handshake, per-frame logs, ARQ stats, and the auto-rate
ladder will all read calibrated SNR values across the entire session
(PING/PONG/CONNECT/CONNECT_ACK via idle-noise; DATA frames via
calibrated pilot residual).

**Operational note from this session:** the autonomous Codex run got
stuck for ~47 min on `check_hw_audio_path.sh` (SSH child died but
parent never noticed). The ctest "failures" Codex reported during
Phase 6 were stale-state artifacts: a leftover literal-string
`ultra_cli_notch.XXXXXX.log` file from a killed prior run blocked
`mktemp`, and `ctest -j4` parallel execution had timing flakes on
the OTA simulator tests. Direct re-runs of all three "failing"
tests passed cleanly; full serial ctest = 51/51. The
implementation itself was correct.

---

## 2026-05-14: Calibrated absolute OFDM SNR meter ready for review

**What was broken:** The residual-SNR diagnostic was linear on AWGN but read
about `+2.71 dB` high at the SNR=15 reference cell, and the old two-LTS
difference path compressed under Watterson fading because `H1-H0` contained
real channel motion as well as AWGN. That made it unsuitable as an
operator-facing broadband SNR meter.

**What changed:** `channel_equalizer_lts.cpp` now documents the OFDM
calibration derivation at the application site. The fixed AWGN constant is the
two-LTS residual normalization: `E{|H1-H0|^2}/4` is half the single-symbol
FFT-bin noise power, so the old meter was high by `10*log10(2)`. For fading,
the calibrated pilot meter uses positive-frequency FFT guard bins adjacent to
the occupied OFDM carriers as the broadband noise reference; these bins share
the same unnormalized `N_fft * sigma^2` noise scaling as active carriers but
do not contain transmitted subcarrier energy. `populateDecodeMetrics()` now
publishes that calibrated OFDM value through `result.snr_db`; MC-DPSK keeps the
chirp-derived fallback. LTS remains a diagnostic sibling only.

**Why it is properly fixed:** The constants trace to the actual modulator and
FFT behavior: data, LTS, and pilot subcarriers are unit power; TX IFFT scales
by `1/N`; RX FFT is unnormalized; real passband up/downconversion contributes
a common carrier factor that cancels for LS residuals; and broadband SNR is
`N_fft * kModemReferencePower / noise_bin`. No wire format, channel
calibration, or mode-ladder thresholds changed.

**Validation:** `tools/snr_meter_validation.sh 5` now reports calibrated pilot
PASS on all channel families:

| Channel | Pilot slope | Bias @ SNR=15 |
|---------|-------------|---------------|
| AWGN | 1.00 | +0.25 dB |
| Good | 0.99 | +0.07 dB |
| Moderate | 0.98 | -0.15 dB |

New CTest `ChannelModemSNRMeterCalibration` passes AWGN at ±1.5 dB and
Good/Moderate at ±3 dB. Full local CTest passes `50/50`. Protocol ladder
checks remain unchanged: Good15 negotiates `R1/2`, Moderate15 negotiates
`R1/2`, and Good10 negotiates `R1/4`. Pi5 hardware validation passed after
fetching and rebuilding `feat/calibrated-snr-meter`: audio path check
`/tmp/ultra_audio_path_20260514_161009`, hardware smoke `3/3 PASS`, report
bundle `agents/reports/hardware_20260514_161038`.

**Status:** Ready for review and merge from branch
`feat/calibrated-snr-meter`.

---

## 2026-05-14: OFDM residual-SNR diagnostic plumbing (no operator-facing change)

**What was investigated:** OFDM `frame.rx.snr_db` is the chirp-correlation
estimate, which saturates in the mid/high 20s once the dual-chirp matched
filter has enough processing gain. DATA frames at honest broadband SNR around
10-15 dB look like 25+ dB to logs, operator display, and the MODE_CHANGE
handshake. The auto rate-ladder `selectOFDMCodeRate(snr_db, fading_index)`
discriminates in practice on `fading_index` (which **is** calibrated and
honest) — the `snr_db` threshold gate never effectively triggers because the
saturated value sits above every threshold.

**What landed (instrumentation only):**

1. OFDM demodulation now accumulates a residual-derived `last_snr_db_estimate`
   from same-frame LTS residual noise and per-symbol pilot residuals, exposed
   through new `hasLastSNREstimate()` / `getLastSNREstimate()` accessors on
   the OFDM demodulator and waveform interfaces.
2. `DecodeResult` gained sibling fields `pilot_snr_db`, `has_pilot_snr_db`,
   and `lts_snr_db` (the latter was already present but is now consistently
   populated). These are diagnostic-only and do not replace `snr_db`.
3. New tool `tools/ofdm_snr_probe` runs a single OFDM frame through
   `SimulatedChannel` at configured SNR and prints
   `sync_snr,pilot_snr,lts_snr,fading_index` for offline calibration work.
4. Debug log line in `populateDecodeMetrics()` reports
   `chirp_snr / pilot_snr / lts_snr / fading` together so operators can
   compare the three estimators side-by-side.

**What did NOT change (and why):** The 2026-05-14 investigation tried two
substitutions for `frame.rx.snr_db` (`pilot_snr` in Phase 2, then `lts_snr`
in Phase 5). Both were **reverted** because protocol-context and probe-context
measurements show both estimators carry channel- and phase-dependent bias of
5-9 dB. Trading a saturating-but-stable wrong reading for a varying wrong
reading is not a fix. The operator-facing `frame.rx.snr_db` retains its prior
chirp-derived behavior; no production decoding path changes.

**Phase-1 feasibility data (from the diagnostic probe):** monotonic tracking
of configured channel SNR with Pearson `r=1.000` AWGN, `0.987` Good,
`0.980` Moderate — but with channel-dependent absolute-value bias of
+3.5 dB (pilot, AWGN), +9 dB (LTS, AWGN), and -5-7 dB (handshake-phase
frames). Correlation is necessary but not sufficient for a calibrated meter.

**Phase-2/3 protocol context:** ladder picks unchanged on the documented
floor cells (Good15 / Moderate15 / Good10 all retain DQPSK R1/2 or R1/4 as
before). Hardware smoke on Pi5 (3/3 AWGN/Good/Moderate at R1/2 SNR=15 1KB)
passes both before and after this branch — confirming `snr_db` was not a
load-bearing decoder input.

**Why no behavior change is correct:** The mode-ladder was already running
on `fading_index` for its discrimination. The displayed and handshake-exchanged
SNR was wrong before this branch and remains the same wrong value after this
branch lands. The right path forward is a calibrated absolute-SNR meter as
its own workstream, scoped in [`docs/SNR_METER_DESIGN.md`](SNR_METER_DESIGN.md).

**Test verification:** `cmake --build build -j4` clean; full CTest `49/49`
including `ChannelSNRCalibration` (separate ±1.5 dB channel-side gate
preserved). Pi5 hardware smoke `agents/run_hardware_smoke.sh` 3/3 PASS.

**Follow-ups flagged:** The calibrated absolute-SNR meter is the next
workstream — see `docs/SNR_METER_DESIGN.md` for the six-step plan
(define noise model, build estimator, add CTest calibration gate at
±1.5 dB, per-channel validation, protocol-context validation, hardware
validation). Once that lands, the operator-facing field can finally be
swapped from chirp to a calibrated value with confidence.

---

## 2026-05-14: SimulatedChannel AWGN is continuous RX noise

**What was broken:** `SimulatedChannel` synthetic AWGN was not a real channel
noise floor. Active TX chunks got TX-side AWGN from `awgn::addAWGN()`, but
silent TX chunks measured zero active signal power and passed through as pure
zeros. Separately, idle RX underflow noise used a fixed `0.01` reference power
that was decoupled from modem TX RMS. The result was an artificially quiet
silence model; modem regressions that quoted "AWGN SNR=X dB" were easier than
a receiver hearing continuous HF band noise.

**What changed:** `tools/sim/simulated_station.hpp::SimulatedChannel` now sizes
synthetic AWGN from a fixed modem reference RMS and injects it once per RX
sample in `receiveForA()` and `receiveForB()`, regardless of peer buffer state.
The TX-side AWGN call was removed from `applyChannel()`, leaving that path for
Watterson processing only. AWGN is enabled only for explicit
`configure(snr, ChannelType::AWGN)`; Watterson modes keep their own channel
noise, and the WAV noise overlay remains independent and additive.

**Reference measurement:** `StreamingEncoder::encodePing()` was measured via
`./build/ota_simulator gen --frame PING --callsign 8P9QC --peer-callsign KC3VPB
--out /tmp/projectultra_ping_ref.wav`. The generated float WAV contains the
raw `encodePing()` samples; `sox ... stat` and direct float parsing measured
`62208` samples, RMS `0.318072406640`, power `0.101170055866`, peak `1.0`.
The hard-coded simulator reference is `kModemReferenceRms = 0.3180724f`.

**Before/after SNR sweep:** two-endpoint v2 QSO, DQPSK R1/4 initial mode,
AUTO waveform negotiation, connect -> message -> disconnect.

| Configured SNR | Pre-fix result | Continuous-AWGN result |
|----------------|----------------|------------------------|
| +20 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| +15 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| +10 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| +5 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| 0 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| -3 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| -5 dB | pass, DATA ok, DISCO ok | pass, DATA ok, DISCO ok |
| -8 dB | pass, DATA ok, DISCO ok | fail: connection/message/disconnect assertions |

Refinement after the required table: -6 dB passes, -7 dB fails the 30 s
connected-state assertions and the 60 s message assertion. The changed sweep
cell in the required table is therefore the old -8 dB pass; the nearest
measured passing continuous-AWGN point is -6 dB, a +2 dB correction from
-8 dB, and the one-dB break is between -6 and -7 dB.

**Test verification:** pre-fix baseline `ctest --test-dir build
--output-on-failure -j4` was 48/48 PASS. After the patch, `cmake --build
build -j4` was clean and the same CTest command was 48/48 PASS. No CTest
pass/fail statuses changed and no SNR thresholds were raised. The
`CLISyntheticNotch`, `OTASimulatorTwoEndpointClean`, and real-HF-noise
`OTASimulatorTwoEndpointNoisy` gates still pass.

**Continuous-noise sanity check:** a direct one-shot `SimulatedChannel` probe
configured AWGN 15 dB, pulled one idle RX second, then transmitted 480 all-zero
peer samples and pulled those RX samples. Measured RMS was `idle_rms=0.0563506`
and `silent_tx_rx_rms=0.0570799`, verifying that RX is not silent during idle
or all-zero TX chunks.

**Follow-ups flagged, not fixed in this round:** other test/helper paths still
call `awgn::addAWGN()` with `activeSignalPower()` and may share the "implicitly
calibrated to active signal only" smell outside `SimulatedChannel`:
`decode_bench`, `threaded_simulator`, `test_waveform_simple`, GUI audio
simulation, and several waveform/unit tests. The final -7 dB run now fails
during connection establishment, which is a real low-SNR modem/ARQ behavior to
analyze separately if that floor matters. No PHY code was changed here.

---

## 2026-05-14: ota_simulator v2 two-endpoint real-HF QSO regression

**What was missing:** v1 could inject scripted WAV audio into one live endpoint,
but it could not validate a bidirectional QSO where peer ACKs, DATA timing, and
DISCONNECT responses depend on the actual live protocol exchange.

**What changed:** extended `ota_simulator run` with strict `version: 2` JSON:
two named endpoints, local `command` events, endpoint-scoped assertions, an
optional channel block, and endpoint-tagged JSONL session events. Added
`runner_v2` to wire two `SimulatedStation` instances through the existing
`SimulatedChannel`/`VirtualAudioPort` path used by `cli_simulator`. Added
`SimulatedChannel::setNoiseOverlay()` for a looped 48 kHz real-HF WAV bed mixed
additively into both receive directions after RMS normalization, plus clean
channel behavior when v2 omits `snr_db`.

**Why this is the right fix:** the harness now exercises the live handshake,
MODE negotiation, DATA/ACK, and DISCONNECT path while keeping PHY code
unchanged. The real-HF overlay is sample-rate matched at load time, bounded to
the preloaded WAV vector, and applied at the channel boundary rather than at an
endpoint's TX or RX implementation.

**Verification:** registered `OTASimulatorTwoEndpointClean` and
`OTASimulatorTwoEndpointNoisy` with 120 s CTest timeouts. The noisy scenario
uses `tests/fixtures/ota_ping/ota_noise_no_ping.wav` looped at
`target_rms=0.05`. v2.1 follow-ups remain out of scope: per-direction
impairments, Watterson scenario plumbing, file-transfer scenarios, ARQ failure
injection, and third-station QRM-style overlays.

---

## 2026-05-13: ota_simulator v1 scripted external-audio regression rig

**What was missing:** after BUG-PING-DETECTOR-001, there was no deterministic
CTest-gateable way to inject externally captured or synthetic audio into one
live endpoint while the full `SimulatedStation` protocol state machine was
running. Decoder-only replay could validate PHY decode behavior, and
`cli_simulator` could validate two live endpoints, but neither covered
"scripted peer audio enters a disconnected endpoint and the endpoint must emit
the correct protocol response."

**What changed:** added `ota_simulator` with `gen` and `run` subcommands.
`gen` writes single-frame 48 kHz float WAV clips through `StreamingEncoder`.
`run` loads a strict v1 JSON scenario, drives a single `SimulatedStation`
through a new `ScriptedAudioPort`, mixes optional normalized noise-bed audio
with scheduled clip injections, captures endpoint TX audio to `out_tx.wav`,
and writes replay-compatible JSONL events to `out_session.jsonl`. TX frame
identification is done by back-decoding the captured TX audio with
`StreamingDecoder`; PING/PONG use the protocol context because their wire image
is intentionally identical.

**Why this is the right fix:** the injected path is additive 48 kHz audio into
the same streaming decoder and connection callbacks used by the simulator
station, so it exercises the bug class without touching PHY hot paths. The
audio callback only mixes preloaded bounded vectors and appends TX samples;
the heavier TX back-decode and assertions run in the scenario runner.

**Verification:** registered `OTASimulatorSmoke`, which injects
`tests/fixtures/ota_simulator/clips/peer_ping.wav` into a DISCONNECTED
`8P9QC` endpoint and asserts a decoded TX `PONG` within 4 s. Deferred v2 items:
two-endpoint mode, Watterson/CFO/fading impairment plumbing, packaged replay
bundles, and richer real-HF noise-bed libraries.

---

## 2026-05-13: BUG-PING-DETECTOR-001 real-HF PING classifier fallback

**What was broken:** real OTA PINGs locked the dual chirp correctly but were
dropped before PONG because the disconnected MC-DPSK PING detector relied only
on post-training RMS being quiet. On a busy HF band, the post-preamble band
noise can be close to training RMS, so `data_rms / training_rms` looked like a
DATA frame even when the waveform was actually a chirp+training+reference PING.

**Root cause:** the old discriminator was implicitly calibrated to clean-cable
or simulator AWGN noise floors. That is not a signal-model invariant: background
QRM changes the denominator/ numerator relationship even though the PING wire
image did not change.

**What changed:** `streaming_frame_policy.hpp::evaluatePingFrame()` now keeps
the existing PATH 1 RMS silence test and adds PATH 2:
`chirp_corr >= 0.30`, `abs(gap_error_samples) <= 1000`, and no valid LDPC frame
(`ldpc_decode_succeeded && ldpc_magic_valid` is false). The call site passes
the already-computed chirp correlation and dual-chirp gap error from sync, and
the existing MC-DPSK LDPC decode outcome. Clean PINGs still return early through
PATH 1; PATH 2 only runs after the decoder already tried LDPC because PATH 1
did not fire.

**Why this is the right fix:** PING and DATA are identical through chirp,
training, and reference; they differ only in the LDPC data region. The chirp
correlation is the matched-filter response against the known ProjectUltra chirp
template, and the dual-chirp gap error verifies the expected up/down chirp
geometry. LDPC success plus `0x55 0x4C` magic is the binary truth signal for a
valid data/control frame. Both parts are invariant to background noise floor by
construction: a matched filter normalizes the known-template lock, and LDPC
validity is pass/fail on the decoded codeword structure rather than an energy
ratio against whatever QRM follows the preamble.

**Verification:** added `tests/test_ping_detector.cpp` with the two real OTA
PING fixtures, a regenerated direct-`StreamingEncoder::encodePing()` AWGN SNR15
fixture, and a no-PING noise fixture. The real OTA captures fire PATH 2 with
gap errors 145 and 83 samples; the synthetic AWGN fixture fires PATH 1. Ran
`cmake --build build -j4` and `ctest --test-dir build --output-on-failure`:
44/44 PASS.

---

## 2026-05-13: Diagnostics cleanup eats the just-saved report

**What was broken:** clicking "Save Bundle" in `ultra_gui` would log
`[DIAG] Report created: …/reports/ultra-report-…zip` but the .zip
file would be deleted within milliseconds of being written. Discovered
when an operator clicked Save 3 times in a row and got zero zips on
disk despite three successful "Report created" log lines.

**Root cause:** `DiagnosticsRecorder::cleanupStorage()` in
`src/diagnostics/diagnostics_recorder.cpp` ran after every `freeze()`,
computed total diagnostics dir size, and if over `kStorageCapBytes`
(then 64 MB = `AudioRing::kHardCapBytes`) it walked `reports/`
sorted oldest-first and deleted until under cap. The sessions cleanup
above it only removed sessions by count (>100) or age (>30 days) —
never by size. A normal operator accumulating ~50 sessions @ ~3 MB
each = 150 MB > 64 MB cap, then the freshly-created report became
the only/oldest candidate in `reports/` and got eaten on the same
`freeze()` call that created it.

**What changed:**
1. Storage cap raised from `1x AudioRing::kHardCapBytes` (64 MB) to
   `8x` (512 MB). The previous value was 1.5x a single report's
   size, leaving no operational headroom for normal session
   accumulation.
2. Cleanup is now a three-pass cascade:
   - Pass 1 (unchanged): drop sessions older than 30 days or beyond
     the 100-newest, current session preserved.
   - Pass 2 (new): if still over cap, evict OLDEST sessions until
     under cap; current session always preserved.
   - Pass 3 (existing, fixed): if still over cap, evict oldest
     reports — but **NEVER the single newest report**. The operator-
     visible artifact MUST survive the same `freeze()` call that
     created it.

**Why this is the right fix:** the producer/consumer contract of
`freeze()` is that the returned path is durable. A cleanup pass that
violates that contract is a correctness bug, not a tuning knob. The
"never the newest report" invariant is enforced in code by stopping
the eviction loop at `candidates.size() - 1`. The cap raise to 512 MB
is operator-realistic headroom — a shift's worth of sessions plus a
handful of saved reports.

**Test verification:** ctest 43/43 PASS. Manual test plan: open
`ultra_gui`, click Save Bundle, verify the zip remains on disk after
the log line prints.

---

## 2026-05-13: TNC session reset after disconnect

**What was broken:** a persistent `ultra_tnc` process could poison the
next PAT session after a clean disconnect. The second peer's fresh
MC-DPSK CONNECT chirp was detected, but the decoder reached the early
reject path (`cw_ok=0/cw_fail=0/is_ping=0`) because StreamingDecoder
state, cached CFO, and post-negotiation waveform state survived the
session boundary. R1 added the decoder/session reset, but hardware
showed that reset alone was insufficient: resetting decoder positions
while SDL/CoreAudio capture continued producing input could dump a stale
capture backlog into the freshly reset decoder, leading to multi-megasample
RX buffer drops before the next CONNECT_ACK window. The same lifecycle
boundary also needed an in-flight decode guard so the decoded-frame callback
could not reset the decoder and then let the old decode path commit stale
cursor positions afterward.

**What changed:** `tools/ultra_tnc.cpp::setConnected(false)` now pauses
audio input before the reset, performs a full `StreamingDecoder::reset()`,
re-enters disconnected MC-DPSK search, re-establishes the MC-DPSK decoder
mode and DQPSK R1/4 handshake decode profile, restores the TX encoder to
MC-DPSK DQPSK R1/4 for the next handshake, disables burst interleaving,
clears the cached negotiated CFO, drains queued input audio, and resumes
capture; the TNC also serializes input polling/feed with that reset so no
already-dequeued capture vector can enter the decoder after the reset.
`src/gui/audio_engine.{hpp,cpp}` adds input-only pause, drain, and resume
helpers that preserve the open device and only discard queued RX samples.
`src/gui/modem/streaming_ofdm_decode.cpp` now checks the decoder reset
generation after decoded-frame callbacks and abandons stale post-callback
cursor updates if the disconnect reset ran.

**Why this is the right fix:** disconnect is the modem session boundary.
Resetting RX there matches the empirically good process-restart state
without widening detection thresholds or changing the wire image. The
audio quiesce makes that state transition producer/consumer coherent:
the decoder is reset only while the input producer is paused, and stale
kernel/user-space capture samples are discarded before the next session's
fresh chirp is allowed through. The reset-generation check makes the consumer
side coherent as well: an in-flight decode cannot overwrite the freshly reset
search cursor with pre-disconnect positions. `StreamingEncoder::setMode()`
preserves `narrowband_control_`, so the TX handshake reset does not discard
a narrowband-control override.

**Verification:** added `TwoSessionsSameEnginePairBothSucceed` to
`tests/test_tnc_session.cpp`, which reuses one engine/encoder/decoder pair
for two back-to-back sessions and reverses the initiator on session 2.
Run with `cmake --build build --target test_tnc_session -j4`,
`build/tests/test_tnc_session`, and
`ctest --test-dir build --output-on-failure`.

---

## 2026-05-11: OTA field diagnostics (Phase 1 + 2)

**Why:** to start OTA testing with non-developer operators, we need
to recover what happened on a remote machine when contact fails or
behaves oddly — without expecting the operator to read logs or run
terminal commands. Two operators sending a small archive each is the
minimum viable evidence per Codex's design memo (2026-05-11).

**Phase 1 (`a328d70`):** local black box + report bundle.
- `src/diagnostics/`: recorder, bounded RX/TX PCM rings,
  in-memory event tail, zip bundle builder (miniz), redaction
- `tools/ultra_report.cpp` CLI: `--list / --create / --inspect /
  --replay-prep`
- `include/ultra/build_info.hpp.in`: version, git commit, dirty flag,
  build time, OS surfaced in `--version` and report manifest
- Choke-point event sinks in AudioEngine / ModemEngine /
  ProtocolEngine: `session.state`, `waveform.negotiated`, `frame.tx`,
  `frame.rx`, `decode.fail`, `audio.overrun`, `fault.triggered`,
  `report.created`
- GUI "Create report" dialog with note + consent
- ultra_tnc crash-tombstone signal handler (SIGSEGV / ABRT / ILL /
  FPE / BUS) + next-launch detection

Bundle layout (zip, universally double-clickable):
```
manifest.json
events/session.jsonl
audio/rx_48k_s16.wav
audio/tx_48k_s16.wav  (optional, default off)
config/effective_config.redacted.json
logs/operator.log
system/system.json
notes/operator_note.txt
replay/README.md
```

Audio capture default ON with first-run consent. Lossless PCM16
authoritative. RX/TX ring is preallocated atomic PCM16; recordRx /
recordTx / emit are non-blocking, lock-free, allocation-free, run
from the audio callback; background writer thread does all
filesystem I/O.

**Phase 2 (`b6b9acd`):** per-session always-on debrief.
- `src/diagnostics/session_summary.{hpp,cpp}`: reducer over the
  JSONL journal producing operator-readable debrief (outcome, wall
  time, mode timeline, file transfer result, ARQ counters, channel
  SNR/CFO/fading min/median/max, decode failures, audio stats,
  faults, disconnect reason). Callsigns redacted by default.
- Every session writes its journal incrementally to
  `diagnostics/sessions/session-<utc>-<id>.jsonl`; on session end
  the summary lands next to it at `.txt`.
- `freeze()` now sources `events/session.jsonl` from the on-disk
  journal rather than the bounded in-memory tail.
- GUI end-of-session debrief modal with "Save debrief" + "Create
  full report" buttons.
- TNC end-of-session log block prints outcome / wall time / mode /
  decode failures + path to `.txt`.
- `ultra_report --list` shows sessions + reports; `--summary
  newest|<id>` re-renders the debrief on demand.
- Best-effort session retention: 30 days or 100 sessions, whichever
  comes first, evaluated at startup.

**Transport (first OTA round):** email. Operator emails the local
zip; you triage manually. No GitHub auto-upload, no telemetry, no
network calls in the diagnostics path. GitHub issue body templating
and Claude-Agent inbox triage are deferred follow-ups.

**Verification:** ctest 40/40 PASS (was 39 — added test_diagnostics
with summary fixtures for connected / handshake-fail / mode-stuck
cases, journal consumption, ring drop counters, event wraparound,
bundle build/inspect roundtrip, tombstone parsing). cli_simulator
regression unchanged. Phase 1 hardware-smoked at 1047.8 bps Good
SNR=15 R1/2, 0 retx. Manual `ultra_report --create` + `--summary
newest` produces a real `.txt` debrief, `--inspect` reads back the
zip's 8-file layout.

**No wire-format change.** Real-time audio thread unchanged. macOS +
Linux + Pi5 in scope; Windows deferred (zip container chosen so
Windows can read bundles natively when scope expands).

---

## 2026-05-11: MC-DPSK ARQ tuning for continuous bursts

**What was sub-optimal (post-burst transport):**
After the continuous burst landed, ARQ window/SACK/timeout parameters
were still sized from single-frame airtime. Continuous burst pays
chirp+training once at burst start (~1.2 s preamble + ~0.4 s overhead)
and then streams data-only frames. The transport never fully filled
the 19 s burst budget — Robust-Mid was running window=2 when window=3
fits, Robust was at window=4 when window=5 fits. SACK tail delay was
also a flat 2 s regardless of rung, and Robust-Low had a hard-coded
72 s ACK timeout that double-counted burst airtime.

**What changed (`bfcfee0`):**

- `connection_policy.hpp` — `MCDPSKFrameTiming` gains overhead_symbols /
  data_only_symbols / overhead_ms / data_only_ms fields. New
  `mcDpskBurstAirtimeMs(timing, window)` computes physical burst length
  (preamble + overhead + N × data-only). `mcDpskWindowSizeForTiming()`
  now takes the full timing struct and picks the largest window whose
  burst fits a 19 s budget (max 5). `mcDpskSackDelayMs()` uses
  data-only continuation time. New `mcDpskSackTailDelayMs()` returns
  `overhead_ms + 400 ms` clamped to [500, 1000].
- `connection.cpp` — `configureArqForCurrentDataMode()` passes timing
  to the window selector, uses the new tail delay for
  `setSackDelayShort`, and treats the Robust-Low 72 s timeout as a
  36 s **floor** over the computed timeout rather than a hard override.
- `test_connection_policy.cpp` — assertions for burst window math,
  SACK delay decomposition, and Robust-Low timeout floor.

Window sizing impact:
- Robust-Mid (DBPSK 1024sps): window 2 → 3
- Robust (DQPSK 1024sps): window 4 → 5
- Robust-Low (DBPSK 2048sps): window 1 (unchanged — single frame fills
  burst budget)

**Hardware-validated (Mac↔Pi5, --inject --inject-gain 0.70, 1KB):**

| Cell                         | Pre-tuning | Post-tuning | Delta | Retx |
|------------------------------|-----------:|------------:|------:|-----:|
| Robust-Mid 0 dB Mod          | 30.3 bps   | **34.9 bps**| +15%  | 0    |
| Robust +5 dB Good            | 75.7 bps   | **81.0 bps**| +7%   | 0    |
| Robust-Low -5 dB Mod         | 13.5 bps   | 13.5 bps    | 0%    | 0    |
| Adaptive +15 dB Good 20KB OFDM | 1733 bps | 1739.9 bps  | flat  | 0    |

Robust-Low shows no gain because the airtime floor at 2048sps DBPSK is
preamble-dominated; window=1 is correct there. OFDM-CHIRP regression
check confirms MC-DPSK-only scope.

**Verification:** ctest 39/39 PASS. Forced-preset AWGN smokes (all 4)
pass. Adaptive SNR=0 and SNR=15 sim paths pass.

---

## 2026-05-11: MC-DPSK continuous burst — amortize chirp/training

**What was slow (transport efficiency):**
After 2026-05-10's adaptive ladder landed, MC-DPSK cells were measuring
20-30% of their coded PHY ceilings — Robust-Mid at 28 bps with a 94 bps
ceiling, Robust at 43-56 bps with a 188 bps ceiling. Each data frame
carried its own ~1s chirp + training preamble, then ~5-15 s of LDPC
payload, then ACK/turnaround. Per-frame preamble cost dominated.

**What changed (1 commit on exp/mc-dpsk-continuous-burst):**

`746433a` — MC-DPSK continuous burst. TX emits one chirp + training
preamble, then packages multiple logical DATA frames into a single
physical burst (each frame modulated separately to preserve
differential phase; symbol-boundary padding keeps RX cursor aligned).
RX maintains CFO, differential phase, and codeword cursor across the
burst. Frame-level SACK at burst end unchanged. Per-CW SACK/repair
semantics preserved.

Wire format: existing DATA frames packaged into one MC-DPSK physical
burst (no DATA_SUPER frame added) — avoids burning an extra LDPC
header and keeps DATA_REPAIR/NACK paths intact.

**Hardware-validated multi-run (Mac<->Pi5, --inject --inject-gain
0.70, --rate auto, 1KB):**

| Cell             | Pre-burst (1 run) | Burst (3 runs)                   | Multiplier |
|------------------|------------------:|----------------------------------|-----------:|
| Robust-Mid 0 Mod | 27.3 bps          | 30.3 / 30.2 / 30.3 (0 retx ×3)   | 1.11x      |
| Robust  +5 Good  | 42.6 bps          | 75.7 / 75.7 / 75.8 (0 retx ×3)   | **1.78x**  |
| Robust  +8 Good  | 55.8 bps          | 75.7 / 75.8 / 75.8 (0 retx ×3)   | 1.36x      |
| Adaptive +15 Good 20KB | 1703 bps    | 1718 bps (0 retx)                | 1.01x ✓    |

All cells 3/3 PASS with zero retransmissions. Robust rung gets the
predicted 1.36-1.78x speedup (within Codex's strategic estimate of
1.6-2.4x). Robust-Mid sees a modest 1.05-1.11x because it is already
PHY-ceiling bound on DBPSK 1024sps. OFDM_CHIRP regression check
clean.

**What's NOT done (future work):**
- Robust-Mid speed-up: the rung sits near its PHY ceiling; further
  gains need either a faster code rate (e.g. R1/2 DBPSK) or a Robust
  rung extension.
- Standard-Plus rung (8c DQPSK 512sps R1/2 for Good/AWGN +8 to +9
  cells, target 110-150 bps net) attempted overnight but Codex API
  service was unreliable through the night with three consecutive
  stalls; deferred to next session.
- Per-CW soft-combining HARQ — would help cells with retransmissions;
  current 0-retx cells leave it as a future lever for marginal
  channels.
- Multi-run validation on the lower-SNR cells (Robust-Low at -5 Mod,
  Robust-Mid at -3 Mod) — burst should not have regressed these,
  but worth a sweep for completeness.

ctest 39/39 pass. Single-frame DATA path preserved for all forced
preset paths. OFDM_CHIRP / OFDM_NARROW behavior fully preserved.

---

## 2026-05-10: MC-DPSK speed ladder + adaptive rung negotiation

**What was missing (architectural feature):**
Production MC-DPSK had a single preset (level8: 8c DQPSK 512sps R1/4)
with a documented reliable cell of SNR≥10 dB Moderate fading. The
modem could not serve sub-10 dB cells. Cold-call between stations
required both endpoints to agree on a preset out-of-band — there was
no adaptive rung negotiation.

**What changed (9 commits on exp/mc-dpsk-ladder-2026-05-10):**

1. `68923a1` — **Robust-Low preset** (8c DBPSK 2048sps R1/4 3-CW
   variable frames). CLI flag `--mc-dpsk-preset robust_low`.
2. `06487e1` — **3-CW frame bound** for DBPSK MC-DPSK file transfer
   (4-CW frames span too many fade coherence windows).
3. `448879d` — **Robust-Mid preset** (8c DBPSK 1024sps R1/4 3-CW).
   Pure SPS halving over Robust-Low: 2× bps for 3 dB cost.
4. `ec36395` — **Pipelined ARQ** with bitmap SACK for MC-DPSK file
   transfer (window>1, timing-derived per profile). Robust-Mid jumped
   from 19.2→28.9 bps at -3 Mod.
5. `c253739` — **Per-CW repair Phase 1**: decoder surfaces partial
   CW data; NACK gains `missing_cw_bitmap` field.
6. `e0b239b` — **Per-CW repair Phase 2**: compact `DATA_REPAIR`
   frames carry only failed CWs. Coord guard prevents double-tx
   when timeout fires near NACK arrival. +32% throughput at +8 Mod.
7. `3194b8c` — **Noise-floor-relative chirp RMS gate**: replaces
   fixed 0.025 RMS skip with a sweep-max + noise-floor-adaptive
   gate. Never raises above 0.025 (no lower-SNR regression). Fixes
   chirp acquisition failures at high-SNR fading.
8. `6e600a7` — **Adaptive rung negotiation**: LadderRungId enum +
   `selectLadderRung(snr, fading)` policy. Cold-call/listen defaults
   to Robust-Mid; responder picks rung in CONNECT_ACK. `cli_simulator`
   without flags = adaptive. `setMCDPSKProfile()` early-outs on
   no-op reconfigurations (fixed an SNR=0 segfault that two prior
   attempts had).

**Wire format additions (pre-deployment, no compat needed):**
- `LadderRungId` enum (UNKNOWN/ROBUST_LOW/ROBUST_MID/ROBUST/
  OFDM_CHIRP/OFDM_NARROW/STANDARD) encoded in reserved bits 4-6 of
  CONNECT_ACK CW-count byte and MODE_CHANGE payload[5].
- NACK `missing_cw_bitmap` field for per-CW selective NAK.
- `DATA_REPAIR = 0x34` frame type for compact partial retransmission.

**Architecture (cold-call flow):**
1. Both endpoints listen at Robust-Mid (universal hearable -3 to +5
   dB Mod).
2. PING/PONG/CONNECT/CONNECT_ACK at Robust-Mid.
3. Responder measures SNR + fading on CONNECT, calls
   `selectLadderRung()`, embeds chosen rung_id in CONNECT_ACK.
4. Both endpoints reconfigure for DATA via the data-mode-change
   callback (skipping no-op reconfigurations).
5. DATA frames at the negotiated rung.
6. MODE_CHANGE handles mid-session re-negotiation through the same
   path.

**Rung selection policy (per channel classification):**

| Channel | OFDM_CHIRP floor | Robust floor | Robust-Mid floor |
|---------|-----------------:|-------------:|-----------------:|
| AWGN | +8 dB | +3 dB | -5 dB |
| GOOD | +9 dB | +4 dB | -4 dB |
| MODERATE | +10 dB | +5 dB | -3 dB |
| POOR | +12 dB | +7 dB | -1 dB |

Below Robust-Mid floor: ROBUST_LOW.

**Hardware validation (single-run, Mac↔Pi5, `--inject --inject-gain
0.70`, 1KB or 20KB file, `--rate auto`, no `--mc-dpsk-preset`):**

| Channel | SNR | Auto-picked | bps | Retx |
|---------|----:|-------------|----:|----:|
| AWGN | +15 | OFDM-CHIRP DQPSK R2/3 | 2337 | 0 |
| Good | +15 | OFDM-CHIRP DQPSK R1/2 | 1703 | 0 |
| Good | 0 | MC-DPSK DBPSK R1/4 (Robust-Mid) | 28.9 | 0 |
| Moderate | 0 | MC-DPSK DBPSK R1/4 (Robust-Mid) | 27.3 | 0 |

All 4 cells delivered cleanly with zero retransmissions. The auto-
selector spans an 85× throughput range based on measured channel
conditions.

**Forced-preset behavior preserved** as override path:
`--mc-dpsk-preset {standard, robust_low, robust_mid, robust}` still
selects explicit configs. Production deployments can use the flag
during diagnostics or testing.

**Per-rung validated throughputs (forced presets, prior hardware
data):**

| Rung | Cell | bps | Notes |
|------|------|----:|-------|
| Robust-Low | -5 dB Mod | 12.2 | multi-run validated |
| Robust-Mid | -3 to +5 dB Mod | ~28 | pipelined ARQ win=2 |
| Robust + Phase 2 | +6 to +9 dB Mod | 42-56 | per-CW repair helps when channel is borderline-good |

**Test verification:**
- `ctest --test-dir build --output-on-failure`: 39/39 pass
- `cli_simulator --snr 0 --channel awgn --rate r1_4 --test`: PASS,
  adaptive picks Robust-Mid
- `cli_simulator --snr 15 --channel awgn --rate r1_4 --test`: PASS,
  adaptive picks OFDM-CHIRP
- 4 hardware cells listed above: PASS, 0 retx each

**What's NOT done (future work):**
- Multi-run validation (3+ runs per cell) per project ship-gate rule
- Mid-session re-negotiation under realistic channel transitions
- GUI display of active rung
- Further speed work above current ladder ceilings

---

## 2026-05-09: Correct README Raw PHY table — strict bits-on-air definition

**What was broken (documentation accuracy):**
The README's "Raw PHY (theoretical maximum)" table mixed methodologies
across rows and inherited stale constants. Specifically:

- **MC-DPSK row showed 938 bps** for "8 carriers". 938 bps only matches
  the 20-carrier `level11_ultra` preset (20 × 2 × 93.75 × 0.25), not
  the 8-carrier preset that production actually uses. The number was
  inherited from a stale `recommendWaveformAndRate()` constant
  (`waveform_selection.hpp`) and propagated into the GUI / TNC
  reporting paths via `TNCBridge::bitrateEstimate(MC_DPSK)`.
- **All OFDM-CHIRP rows used CP=MEDIUM arithmetic** (1120 sample
  symbol → 42.857 sym/s). Production runs `cp_mode = LONG`
  (1152 samples → 41.667 sym/s). Every OFDM row was ~3 % optimistic.
- **R1/4 was treated as pilotless** (59 data carriers). The actual
  `recommendedPilotSpacing(DQPSK, R1/4)` returns 10 → 6 pilots →
  53 data carriers. Real R1/4 raw PHY is 1104 bps, not 1264.
- **The R1/2 row's 1967 bps** was the *effective single-frame payload
  rate* (8-CW frame airtime including 2 LTS preamble symbols and the
  19-byte v2 header), not raw PHY. Mixing methodologies in one column.
- **OFDM-NARROW R1/4 (103 bps) and R1/2 (230 bps)** appear to have been
  measured throughput from the pre-window=3 ARQ era, not raw PHY at all.
- **16QAM / 32QAM** rows assumed 44 data carriers (no source).
  Production: 16QAM R3/4 → spacing 8 → 51 data; 32QAM R3/4 → spacing 5
  → 47 data.

**What changed:**
- `README.md` — replaced the Raw PHY table with strict-definition
  values (`data_carriers × bits_per_symbol × symbol_rate × code_rate`)
  derived from `recommendedPilotSpacing()` and the production CP
  setting. Added a derivation paragraph above the table. Reworded the
  R1/2 prose paragraph below to label 1967 bps as the
  effective single-frame payload rate, separate from the 2208 bps
  raw-PHY ceiling.
- `src/protocol/waveform_selection.hpp` —
  `estimated_throughput_bps` constants: 938→375 (MC-DPSK), 3900→3438
  (R3/4), 3200→2944 (R2/3), 2300→2208 (R1/2), 1150→1104 (R1/4). Updated
  the header comment table to match the strict definition.
- `src/tnc/tnc_bridge.cpp` — `bitrateEstimate()` returns: MC_DPSK 375
  (was 938), OFDM_NARROW 386 (was 230), OFDM_CHIRP 2208 (was 2300).
  Added a comment pointing to the README derivation.
- `tests/test_tnc_bridge.cpp` — updated the "bitrate event mismatch"
  expected value from 2300 → 2208 to track the new
  `bitrateEstimate(OFDM_CHIRP)`.

**Why this is correct:**
Strict raw PHY = `data_carriers × bits_per_symbol × symbol_rate ×
code_rate`. No subtraction for preamble, frame header, ARQ, or ACK
turnaround — that's the ceiling the modulator could feed downstream
on a steady-state channel. Effective single-frame payload rates (LTS
+ header overhead) and end-to-end measured wall-clock rates are
genuinely different quantities and now sit in their own columns.

**Test verification:**
- `cmake --build build -j4 && ctest --test-dir build --output-on-failure`
  → 39/39 pass after the test_tnc_bridge expected-bitrate update.
- Manually re-derived each row: math now matches `IWaveform::getThroughput()`
  for every supported (mode, modulation, rate) tuple.

---

## 2026-05-08: Refactor/Optimize Round 3 - OFDM scratch preallocation

**What was wasteful:**
OFDM modulation and demodulation allocated short-lived vectors inside
per-symbol paths: frequency-domain bins, time-domain symbols, CP-appended
symbols, real output blocks, equalized carriers, differential scratch, and
interpolation scratch.

**What changed:**
`OFDMModulator::Impl` and `OFDMDemodulator::Impl` now own reusable scratch
buffers sized to the configured FFT/carrier geometry. OFDM hot paths reuse those
buffers and use pointer-based FFT calls with pre-sized output storage.

**Why this is safe:**
The patch changes allocation policy, not modulation, coding, ARQ behavior,
wire format, carrier masks, or CarrierLDPC negotiation. Scratch is instance
local and is consumed before the next scratch-producing call overwrites it.

**Verification:**
- `ctest --test-dir build --output-on-failure -j4` passed 37/37.
- 20 KB Good R1/2 SNR=15: 1705.7 bps, 0 retx, PASS.
- 20 KB AWGN R1/2 SNR=15: 1714.8 bps, 0 retx, PASS.
- 5 KB Good R1/2 SNR=15: 1492.1 bps, 0 retx, PASS.
- `./agents/run_hardware_smoke.sh` passed:
  AWGN 1 KB 1043.0 bps / 0 retx; Good 1 KB 1043.7 bps / 0 retx;
  Moderate 1 KB 877.8 bps / 0 retx.

---

## 2026-05-08: Refactor/Optimize Round 3 - decoder ring parameterization

**What was risky:**
The first F#3 patch changed `% 480000` ring arithmetic into `% buffer_capacity_samples_`.
On arm64 that moved the audio callback sample-copy loop from constant-divisor
strength reduction to a runtime `udiv/msub` pair. The 20 KB Good-channel
failure did not reproduce on replay, but the instruction-level change was real.

**What changed:**
`StreamingDecoder` now accepts a validated ring capacity while keeping the
default ring on the `kDefaultBufferSamples` fast path. Custom rings use an
explicit runtime-capacity path. Tests cover default capacity, a compact
144000-sample ring wrap, and rejection below the 120000-sample sync window.

**Why this is safe:**
Default construction preserves the historical 10 s ring and restores
constant-divisor code in the per-sample copy path. Smaller rings are opt-in and
cannot undercut the largest sync-search window.

**Verification:**
- `cmake --build build -j4 --clean-first` passed.
- `ctest --test-dir build --output-on-failure -j4` passed 37/37.
- 20 KB Good R1/2 SNR=15: 1708.6 bps, 0 retx, PASS.
- 5 KB Good R1/2 SNR=15: 1501.4 bps, 0 retx, PASS.
- `./agents/run_hardware_smoke.sh` passed:
  AWGN 1 KB 1022.3 bps / 0 retx; Good 1 KB 1046.2 bps / 0 retx;
  Moderate 1 KB 875.4 bps / 0 retx.

---

## 2026-05-08: Refactor/Optimize Round 2 - tooling and warnings

**What was broken:**
`tools/profile_acquisition.cpp` still built, but it reproduced the
catalog failure: an OFDM trial exited successfully while reporting
`FAIL` and `No successful decodes!`. That output is not valid timing
evidence.

**What changed:**
1. Deleted the stale `profile_acquisition` source and CMake target.
   `docs/BUILD_SYSTEM.md` and `docs/RESOURCE_FOOTPRINT_ANALYSIS.md`
   now point at maintained `StreamingDecoder` decode buckets from
   `cli_simulator`, `decode_bench`, and hardware-smoke runs.
2. Added source-scoped `-Wall -Wextra` only for tool/test source files.
   The warning policy is not enabled on `ultra_core`, `ultra_gui`, or
   `ultra_tnc` runtime targets.
3. Test source files now undefine `NDEBUG` so assert-based tests run
   their checks in the existing Release build directory.
4. Fixed warnings in tool/test owned code and removed the redundant
   `decode_bench` direct `ultra_core` link.

**Why this is safe:**
This removes a misleading standalone tool and tightens build hygiene
without changing modem algorithms, ARQ parameters, wire format, or
runtime target warning policy.

**Verification:**
- Before: Round 1 ended with `ctest --test-dir build --output-on-failure -j4`
  passing 37/37.
- After: `cmake --build build -j4 --clean-first` passed with no compiler
  warnings in the scoped tool/test sources.
- After: `ctest --test-dir build --output-on-failure -j4` passed 37/37.
- `./agents/run_hardware_smoke.sh` passed:
  AWGN 1 KB 1023.1 bps / 0 retx; Good 1 KB 1022.6 bps / 0 retx;
  Moderate 1 KB 1021.6 bps / 0 retx.

---

## 2026-05-08: Refactor/Optimize Round 1 - docs and cleanup

**What was stale:**
The auto-rate ladder was copied into multiple docs, which creates a
second source of truth beside `selectOFDMCodeRate()`. README and docs
also named specific competing products despite the project naming
policy. The B1 CRC consolidation item was rechecked after legacy modem
removal.

**What changed:**
1. `CLAUDE.md`, `docs/PROTOCOL_V2.md`, and `README.md` now point to
   `src/protocol/waveform_selection.hpp::selectOFDMCodeRate()` and
   `tests/test_waveform_policy.cpp` instead of duplicating exact
   OFDM rate thresholds.
2. `README.md`, `docs/TNC_INTERFACE.md`,
   `docs/MODEM_IMPROVEMENT_BACKLOG.md`, historical changelog text, and
   `docs/CLEANUP_OVERNIGHT_2026-05-07.md` now use neutral naming for
   legacy-compatible TCP TNC clients.
3. The old product-named client audit was renamed to
   `docs/TNC_CLIENT_AUDIT.md` and neutralized.

**B1 decision:**
Skipped. Modern protocol CRC16 already routes through
`v2::ControlFrame::calculateCRC()`. The remaining CRC32 implementation
is file-transfer payload integrity, not a duplicate frame-wire CRC path.

**Verification:**
- Before: `ctest --test-dir build --output-on-failure -j4` passed
  37/37.
- After: `cmake --build build -j4 --clean-first` passed.
- After: `ctest --test-dir build --output-on-failure -j4` passed
  37/37.
- `./agents/run_hardware_smoke.sh` passed:
  AWGN 1 KB 1022.1 bps / 0 retx; Good 1 KB 1022.3 bps / 0 retx;
  Moderate 1 KB 861.5 bps / 0 retx.

---

## 2026-05-07: First successful OTA full-session decode + session_decode tool

**What was missing:**
End-to-end OTA validation of the modem at multiple code rates with full
session artefacts (chirp + handshake + light-preamble data + DISCONNECT).
Earlier OTA tests captured only synthetic-fixture or post-CONNECT_ACK
audio that the existing decoders could not consume as a real session.

**What was added:**
1. `tools/sim/simulated_station.{hpp,cpp}` — extracted `SimulatedStation`,
   `VirtualAudioPort`, and `SimulatedChannel` from `cli_simulator.cpp`
   into a shared static library `ultra_sim_station`. Pure mechanical
   move; observation accessors and a `decoded_frame_callback_` were
   added so external tools can read negotiated state without
   re-implementing the protocol layer.
2. `tools/session_decode.cpp` — new standalone OTA decoder. Loads any
   WAV (PCM s16, PCM s24, IEEE float32, mono/stereo, any sample rate),
   auto-resamples to 48 kHz via the repo `Resampler`, and runs a real
   `ModemEngine` against it through a new `WavReplayAudioPort`. Prints
   a single summary block with chirp correlation + CFO + frame counts +
   per-DATA-frame bytes + decoded message text + ARQ/LDPC stats.
3. `recordings/ota_full_session_2026-05-07/full_session_{r1_4,r1_2,r3_4}.wav` —
   25 s OTA-replayable source WAVs containing chirp + CONNECT
   handshake + 7 messages + DISCONNECT. RMS −16 dBFS, peak 1.0
   (minor PAPR clipping; decoder validates byte-exact end to end).
4. `recordings/ota_capture_2026-05-07_k1vl/` — KC3VPB (PA, 100 W)
   played the three WAVs into his rig, recorded at sdr.k1vl.com
   (Vermont KiwiSDR, 7121 kHz USB, AGC off, 12 kHz PCM16). Three
   `ota_*_kc3vpb_to_k1vl.wav` captures plus `RESULTS.md` documenting
   the decode pipeline (sox +30..+36 dB pre-conditioning →
   session_decode) and the per-rate verdict.

**OTA results (first full-session decode):**
| Rate | Chirp | Handshake | Negotiated | DATA byte-exact | LDPC fail | DISCONNECT |
|------|------:|----------:|------------|----------------:|----------:|-----------:|
| r1_4 | 0.857 | ✓ | OFDM-CHIRP DQPSK R1/4 4-CW | 1/1 (4 B)   | 6 | ✓ |
| r1_2 | 0.763 | ✓ | OFDM-CHIRP DQPSK R1/2 8-CW | 1/1 (126 B) | 5 | ✓ |
| r3_4 | 0.781 | ✗ | (lost CONNECT_ACK)         | 0/0         | 1 | ✗ |

The chirp + LTS feedback path delivered clean coarse CFO at all three
rates (~0.85 Hz, consistent across captures); wire-side rate/CW-count
negotiation completed cleanly OTA at r1_4 and r1_2; r3_4 confirmed the
auto-rate gate (`fading_index<0.10`) is correctly excluding it from
fading channels. LDPC failures on most DATA frames are honest baseline
data — Vermont KiwiSDR with AGC off delivers ~−50 dBFS audio, so the
receive SNR is well below the modem's SNR≥15 design target.

**Discarded as part of this round:**
- `tools/capture_session_audio.sh` — generated post-CONNECT_ACK-only
  WAVs that cannot be decoded over the air without prior chirp lock.
  The light preamble (LTS, ~80 samples) does not survive the cumulative
  CFO of a TX → radio → KiwiSDR → ADC clock chain.
- `recordings/ota_session_2026-05-07/post_connect_*.wav` — the WAVs
  the script produced. Replaced by `ota_full_session_2026-05-07/`.
- `.gitignore` `ota_session_*` exception (no longer needed).

**Verification:**
- `cmake --build build -j4` clean.
- `ctest --test-dir build --output-on-failure -j4` 37/37.
- `./build/session_decode --wav recordings/ota_full_session_2026-05-07/full_session_r1_2.wav`
  → CONNECTED, 7 DATA byte-exact 7/7, 7 messages, DISCONNECT, 0 LDPC
  fails, 0 ARQ retx (self-loopback floor).
- `./build/session_decode --wav <kiwisdr.wav>` after `sox … gain +30dB`
  → reproduces the OTA verdicts in the table above.

---

## 2026-05-05: BUG-RATE-001 — adaptive MODE_CHANGE panic-downshift hardened

**What was broken:**
On short Watterson-Good SNR=15 transfers (5 KB), the connection-layer
adaptive rate controller would panic-downshift R1/2 → R1/4 on the
first fade-induced retransmit and never re-evaluate upward inside the
session. 5-seed sweep on Mac↔Pi5 hardware showed 1/5 seeds at 444 bps
end-to-end (vs 1,440 bps median — a 3.2× tail loss). The remaining
4/5 seeds completed cleanly at the auto-rate target, so the bug only
bites when a fade burst happens early in a short transfer.

Root cause was three combining factors in
`updateAdaptiveModeController` (`src/protocol/connection.cpp:1153`):

1. `hasAdaptiveRetryPressure` (line 68) returned true on a *single*
   1-second eval window with `retransmissions >= 2`. No requirement
   that pressure persist across multiple windows. A single fade burst
   that produced 2-3 retx in the same 1 s window met the threshold
   for queueing a downgrade.
2. `ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS = 15000` — sized for long-haul
   transfers. On a 5 KB file that takes 28-45 s end-to-end, downgrading
   at t ≈ 44 s locks out upgrade until t ≈ 59 s; the file is already
   done.
3. `ADAPTIVE_CLEAN_WINDOWS_FOR_UPGRADE = 3` requires 3 s of clean
   windows. Combined with (2), short transfers literally cannot
   upshift in time.

Filed as `KNOWN_BUGS.md:BUG-RATE-001`.

**What was changed:**
- **`src/protocol/connection.hpp`**:
  - `ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS`: `15000` → `5000` ms.
  - New `ADAPTIVE_PRESSURE_WINDOWS_FOR_DOWNGRADE = 2`.
  - New `Connection::adaptive_pressure_windows_` member.
- **`src/protocol/connection.cpp`**:
  - `updateAdaptiveModeController` now increments
    `adaptive_pressure_windows_` when retry-pressure is true,
    resets on clean windows, and gates downgrade-queueing at
    `>= ADAPTIVE_PRESSURE_WINDOWS_FOR_DOWNGRADE`.
  - Counter reset on acknowledged downgrade, controller reset, file
    transfer stop, and forced-mode override (matching existing
    `adaptive_clean_windows_` reset sites).
- **`tests/test_connection_adaptive.cpp`**:
  - New regression `test_adaptive_downgrade_hysteresis_and_short_lockout_upgrade`
    asserting (a) single retry-pressure window does NOT queue a
    downgrade, (b) two consecutive windows DO queue, (c) post-5 s
    lockout + 3 clean windows queues an upgrade.
  - Existing tests updated to inject a second window of retx pressure
    so they continue to test the downgrade path correctly.

Total diff: +79 / -2 across 3 files. `ADAPTIVE_DOWNGRADE_FORCE_MS = 6000`
left unchanged — the forced-downgrade escape valve is preserved for
genuinely sustained channel collapse.

**Why it's properly fixed:**
The trigger now reflects *consecutive* observation rather than a
single-window snapshot, which is the standard hysteresis pattern for
control loops with noisy measurements. Lockout reduction matches the
duration of a typical short-transfer session, so upshift is reachable
within the same session if the channel recovers. The forced-downgrade
path stays in place so a genuinely collapsed channel still gets a
fast rate cut (`ADAPTIVE_DOWNGRADE_FORCE_MS = 6000` ≥
2 × `ADAPTIVE_EVAL_INTERVAL_MS`, so any sustained pressure still
triggers a forced downgrade after 6 s).

**Test verification:**
- `cmake --build build -j4`: success (with `-DULTRA_BUILD_GUI=OFF`).
- `ctest --test-dir build --output-on-failure -j4`: 35/35 passed,
  including the new `test_adaptive_downgrade_hysteresis_and_short_lockout_upgrade`.
- 5-seed Mac↔Pi5 hardware test, 5 KB Watterson Good SNR=15
  (BUG-RATE-001 reproducer):
  ```
  Seed 1: 1,449 bps  0 retx 0 timeouts  PASS
  Seed 2: 1,440 bps  0 retx 0 timeouts  PASS
  Seed 3: 1,440 bps  0 retx 0 timeouts  PASS
  Seed 4:   684 bps 11 retx 7 timeouts  PASS (no MODE_CHANGE)
  Seed 5: 1,459 bps  0 retx 0 timeouts  PASS
  ```
  vs pre-fix:
  ```
  Seed 1: 1,440 bps  0 retx 0 timeouts  PASS
  Seed 2: 1,440 bps  0 retx 0 timeouts  PASS
  Seed 3: 1,439 bps  0 retx 0 timeouts  PASS
  Seed 4:   444 bps 12 retx 7 timeouts  PASS (panic R1/2→R1/4)
  Seed 5: 1,440 bps  0 retx 0 timeouts  PASS
  ```
  Worst-case throughput on the panic seed improved 444 → 684 bps
  (+54 %), with rate held at R1/2 throughout (no MODE_CHANGE
  downgrade). The remaining loss on seed 4 is genuine bad-channel
  time on R1/2 — addressing that residual is the work for backlog
  #5 phase-2a (TX-aware carrier mask).

---

## 2026-05-04: Wire-level negotiation of fixed-frame CW count

**What was broken:**
Throughput on DQPSK R1/2 SNR=15 good fading was bottlenecked at
~1077 bps because every data frame carried only 4 codewords (the
`kDefaultFixedFrameCodewords` default). Mac↔Pi5 hardware A/B with
manual `--cw-count 8` on both peers showed 1615 bps (+50 %, with
**fewer** retx because larger frames amortize the 5.3 s SACK-defer
overhead across twice the payload). The frame format already supported
1–8 CW (`kMaxFixedFrameCodewords = 8`, count in the frame header at
`frame_v2.cpp:808`) — the dial just wasn't being turned for everyday
auto-rate connections.

A first attempt set CW from a host-side data-mode-changed callback
that called `protocol_.setForcedFrameCodewords()`. That re-entered
`ProtocolEngine::mutex_` (a non-recursive `std::mutex` — see
`protocol_engine.hpp:34`), deadlocking the responder's protocol
thread. Symptom: BRAVO logged "Adaptive CW count 4 -> 8", then went
silent forever; CONNECT_ACK was queued in `tx_queue_` but
`defer_tx_` never reset (line 222 of `protocol_engine.cpp:onRxData`
unreachable past the deadlock); ALPHA timed out waiting at 120 s.
Reproduced 100 % with `--seed 1`.

Codex (gpt-5.5 xhigh) review of the redesign also surfaced three
hazards I'd missed: stale CONNECT_ACK retry timer (computed before
CW finalized), decoder fallback to configured `fixed_frame_codewords_`
when the header read fails (so the wire-byte alone doesn't save us
when peers disagree on configured CW), and the general "callbacks
fire under the protocol mutex — host code must not call back in".
Codex's bottom line: don't ship "both sides recompute" as the
agreement mechanism — make CW an explicit negotiated parameter on
the wire.

**What was changed:**
- **Wire format** (`src/protocol/frame_v2.{hpp,cpp}`):
  - `ConnectFrame::PAYLOAD_SIZE` 25 → 26 B; new `data_frame_cw_count`
    byte appended after `measured_snr`. Frame total 44 → 45 B.
  - `CONNECT` carries initiator's forced CW (0 = AUTO);
    `CONNECT_ACK` carries responder's chosen value (1..8). Initiator
    applies the echoed value via `frame.data_frame_cw_count`.
  - `ControlFrame::ModeChangeInfo` gains `data_frame_cw_count` via
    `payload[5]` (was a reserved byte — no size change).
- **Policy** (`src/protocol/connection_policy.hpp`):
  - `recommendCWCount(rate)` is rate-only: R1/2, R2/3, R3/4 → 8;
    R1/4 → 4. No SNR/fading dependency, so cross-peer agreement
    collapses to "both peers ran the same rate negotiation".
- **Connection** (`src/protocol/connection.{hpp,cpp,_handlers.cpp}`):
  - `applyDataMode(mod, rate, cw_count = 0)`: explicit CW from
    MODE_CHANGE wire byte, else auto via `recommendCWCount(rate)`.
    Triggers `requeuePendingChunks` on rate-changed OR cw-changed
    (was rate-changed only).
  - `setForcedFrameCodewords(cw, forced = true)`: `forced = true`
    marks `config_.forced_cw_count` for one-sided wire propagation
    (initiator embeds in CONNECT, responder honors and echoes).
    `forced = false` is the boot-time path (host wiring up
    encoder/decoder before connection) — does NOT mark forced and
    so does not bypass the responder's auto-pick.
  - Responder picks negotiated CW BEFORE building CONNECT_ACK and
    BEFORE computing the retry timer (closes the stale-timer hazard).
- **Callback** (`src/protocol/connection.hpp`):
  - `DataModeChangedCallback` signature now
    `(mod, rate, cw_count, snr_db, peer_fading)`. Hosts (cli_simulator,
    ultra_gui real + virtual, ultra_tnc, threaded_simulator) update
    encoder + decoder directly from the param. **No** call to
    `protocol_.setForcedFrameCodewords()` inside the callback — the
    rule is now spelled out in a comment on the typedef.
- **CLI** (`tools/cli_simulator.cpp`):
  - `cw_count_forced_` flag: only `--cw-count N` flips it to true.
    Boot init at `SimController::initStation` passes `forced=false`
    so the default 4 doesn't bypass auto-pick.

**How it's properly fixed:**
Both peers see the negotiated CW count on the wire (CONNECT_ACK byte
for initial, MODE_CHANGE byte for mid-transfer). They set their local
`data_frame_cw_count_` from the wire, never from independent
re-derivation, so peers cannot disagree even if their channel
measurements drift. The `recommendCWCount` function is rate-only so
even in fallback paths there's no SNR/fading-driven divergence. The
encoder/decoder are updated directly from the callback param, which
removes the protocol-mutex re-entry that caused the deadlock.

**Test verification:**
- Sim regression: `./build/cli_simulator --snr 15 --fading good
  --rate auto --file 5120 --max-time 200 --seed 1` → both peers log
  "Negotiated CW count: 8 for DQPSK R1/2", handshake at 10.5/11.0 s,
  transfer done by 36 s.
- `--cw-count 4` override: ALPHA logs `forced_cw=4`, both peers
  configure cw=4. Wire negotiation honors the override one-sided.
- ctest: 35/35 green (incl. `ConnectionPolicy`, `ConnectionAdaptive`,
  `FrameV2` — the suites that broke on the prior abandoned attempt).
- Hardware A/B (Mac↔Pi5 audio loopback, `--inject good --snr 15`,
  DQPSK R1/2 5KB, no `--cw-count`):
  - Run 1 (boot-init bug had forced=true): 1233 bps, 39 frames, 0 retx
  - Run 2 (bug fixed): **1448 bps, 19 frames, 0 retx** (+17 %
    in-session, frames halved 39→19 confirms CW=8 in effect).

**Commit:** `1a98b4d`.

---

## 2026-05-02: TNC Phase 5 — Windows cross-platform support

**Goal:**
Make the TNC server build and run on Windows (CI's `windows-latest`
target) without breaking POSIX behavior. The TNC was the only
POSIX-only piece in tonight's new code; everything else (modem core,
GUI, audio) already had Windows guards.

**What was added:**
- `src/tnc/socket_compat.{hpp,cpp}` — cross-platform abstraction:
  - `socket_t` type alias (int on POSIX, `SOCKET` on Windows)
  - `kInvalidSocket`, `closeSocket()`, `shutdownSocket()`
  - `pollSockets()` (wraps `poll` / `WSAPoll`)
  - `setNonblocking()` (wraps `fcntl(O_NONBLOCK)` / `ioctlsocket(FIONBIO)`)
  - `socketPair()` — POSIX uses `pipe()`; Windows uses standard
    bind+listen+connect+accept loopback pattern (listener active
    before connect → no race)
  - `WinsockInit` RAII for `WSAStartup`/`WSACleanup` lifecycle

**What was changed:**
- `src/tnc/tnc_server.{cpp,hpp}` — refactored to use the new
  abstractions. All `int` socket fds → `socket_t`. `close()` for
  sockets → `closeSocket()`. `poll()` → `pollSockets()`.
  `fcntl()` → `setNonblocking()`. `pipe()` → `socketPair()`.
  `signal(SIGPIPE, SIG_IGN)` guarded with `#ifndef _WIN32`.
  Early `WinsockInit` construction.
- `CMakeLists.txt` — adds `socket_compat.cpp` to `ultra_core`;
  links `ws2_32` on Windows.
- `tests/test_tnc_server.cpp` — adds a `socketPair()` smoke test
  that runs on both platforms and verifies the loopback pair is
  bidirectional. CTest target count unchanged at 34 (test runs
  inside the existing `TNCServer` test).

**Verification (macOS):**
- `cmake --build build -j4` passed
- `ctest --test-dir build --output-on-failure` passed: 34/34
- The new socketPair smoke runs and passes
- Sandbox-blocked localhost bind still gets the existing graceful
  preflight skip

**Verification (Windows):**
- Will be validated automatically by CI's `windows-latest` build job
  on push. Existing CI matrix covers it; no new vcpkg / toolchain
  dependency beyond the system `ws2_32` library.

**WSAPoll caveat:**
`POLLHUP`/`POLLERR` semantics can differ from POSIX `poll()`. The
reactor handles this by polling `POLLIN|POLLERR|POLLHUP|POLLNVAL`,
reading on readiness, and evicting on `recv()==0` or hard errors.
Worst case some close detection may wait one extra poll cycle —
acceptable.

**Path to ultra_tnc.exe:**
After this commit reaches origin/main, CI's `windows-latest` build
job should produce `ultra_tnc.exe` automatically. Manual smoke can
then be done from a Windows host:
```
.\ultra_tnc.exe --audio-output none --audio-input none --port 18300
echo VERSION | nc 127.0.0.1 18300  # or PowerShell equivalent
```
Should return `VERSION 4.9.0 registered\r` exactly as on
POSIX.

---

## 2026-05-02: TNC Phase 4 — hardware loopback test script

**What was added:**
`tools/tnc_loopback_test.sh` — a shell-driven end-to-end test that
runs two `ultra_tnc` instances (Mac local + Pi via SSH, mirroring
`run_hw_test.sh`'s pattern) and validates a binary file transfer
between them via the legacy TNC interface.

Flow:
1. Starts ultra_tnc on Pi via SSH (audio device, callsign, port)
2. Starts ultra_tnc on Mac (audio device, callsign, port)
3. Waits 5s for socket binding, then polls up to 20s
4. Opens persistent cmd-port TCP connection to each side
5. Drives via legacy TNC commands: MYCALL, BW2300, COMPRESSION TEXT,
   LISTEN ON (Pi), CONNECT (Mac initiates)
6. Waits up to 60s for CONNECTED event on both sides
7. Streams a generated payload (default 5 KB) into Mac's data port
8. Pi-side data port captured to file via parallel `nc`
9. Sends DISCONNECT after backlog drains
10. Compares source vs received via cksum + cmp; reports throughput

Tooling: pure bash + ssh + nc + dd + cksum + cmp + awk + grep + sed
+ mkfifo. No python, no `timeout`/`gtimeout`, no extra deps.

**Important legacy TNC quirk handled:**
Closing/reopening the cmd-port TCP connection mid-session would
evict the active TNCSession (single-client semantics from Phase 2).
The script keeps cmd sockets persistently open via FIFO-backed nc
processes; commands are written to FIFO, output is tailed.

**Verification:**
- `bash -n tools/tnc_loopback_test.sh`: passed (syntax clean)
- ctest: 34/34 (no source code modified)
- The actual hardware run is gated on the soundcard being free — the
  500 KB sweep is still using it. Will run after sweep completes.

**Acceptance:**
Once executed and passing, this is the proof-of-life that the TNC
bridge works against real audio + real ProtocolEngine + real ARQ +
real soundcard. Currently Phase 1+2+3a+3b are validated by ctest +
the manual `VERSION` smoke. Phase 4 is the integration validation.

---

## 2026-05-02: TNC Phase 3b — TNCBridge + ultra_tnc binary (working legacy TNC)

**Goal:**
Tie all the TNC pieces together. After this phase ships, ProjectUltra
exposes a legacy-compatible HF TCP TNC interface that **client software
(reference client, mainstream Windows HF mail client, packet-router client, alternative TNC client) can use as a drop-in legacy TNC
replacement** at the TCP API level.

**What was added:**
- `src/tnc/tnc_bridge.{cpp,hpp}` — `TNCBridge` class. Implements
  `ModemAdapter` on top of `ProtocolEngine` + `AudioEngine`:
  - Bandwidth → waveform mapping: BW2300→OFDM_CHIRP,
    BW500→OFDM_NARROW, BW2750→OFDM_CHIRP (preserved as 2750 in
    CONNECTED event for client compat)
  - PTT inference: polls `AudioEngine::isTxQueueEmpty()` from the
    TNC reactor's tick loop, emits `PTT ON` on non-empty,
    `PTT OFF` after 200 ms drained tail
  - Subscribes to ProtocolEngine callbacks (connection state,
    data received) → marshals to TNCServer's reactor queue via
    `postModemConnected/Disconnected/PTT/...`
  - Thread-safe state with `state_mutex_` + `ptt_mutex_`; PE
    callbacks only snapshot bridge state and queue events (no
    re-entrant calls into PE)

- `tools/ultra_tnc.cpp` — new binary. Assembles AudioEngine +
  StreamingEncoder/Decoder + ProtocolEngine + TNCBridge + TNCServer
  in one process. Pattern matches `cli_simulator` single-station
  mode. CLI flags:
  ```
  --audio-output <name|none>   SDL audio output (or "none" for
                                tests without soundcard)
  --audio-input  <name|none>   SDL audio input
  --port <N>                   TNC base port (default 8300; data=N+1)
  --bind <addr>                Bind address (default 127.0.0.1)
  --callsign <call>            Default callsign (overridden by MYCALL)
  --inject-channel [type]      Optional channel injection for cable
                               testing
  --snr <db> --rate ... --mod ... --ofdm-config <default|nvis>
  ```
  Tick loop runs at ~20 ms cadence to drive PTT polling + TNCSession
  IAMALIVE/BUFFER timers.

- `docs/TNC_INTERFACE.md` — user-facing TNC docs: how to run
  ultra_tnc, how to point reference client/HF mail at it, supported legacy TNC commands
  + behavior notes.

- `tests/test_tnc_bridge.cpp` — 16 unit cases against a mock
  ProtocolEngine + mock TNCServer. Covers: setMyCall propagation,
  startConnect + bandwidth params, sendBinary, getTxBacklogBytes,
  PE connection callback → server.postModemConnected, AudioEngine
  queue state → postModemPTT, etc.

**Verification:**
- `cmake --build build -j4`: passed
- `ctest --test-dir build --output-on-failure`: 33/33 → **34/34**
  (`test_tnc_bridge` runs 16 cases internally)
- `./build/ultra_tnc --help`: prints usage
- **Manual TCP smoke test:**
  ```
  ./build/ultra_tnc --audio-output none --audio-input none --port 18300
  $ printf "VERSION\r" | nc 127.0.0.1 18300 | xxd
  00000000: 5641 5241 2076 6572 7369 6f6e 2034 2e39  VERSION 4.9
  00000010: 2e30 2072 6567 6973 7465 7265 640d       .0 registered.
  ```
  Returns the exact `VERSION 4.9.0 registered\r` string reference TCP client
  regexes for. **TNC is functional end-to-end.**

**What this delivers:**
- ✅ ProjectUltra exposes a legacy-compatible HF TCP TNC (8300/8301)
- ✅ Existing client software (reference client, mainstream Windows HF mail client, packet-router client, alternative TNC client)
  can use ProjectUltra as if it were legacy HF TNC — no code changes on
  client side
- ✅ Single binary `ultra_tnc` assembles the full stack
- ✅ Single-thread reactor model (no per-client threads, no
  ProtocolEngine reentrancy risk)
- ✅ PTT inferred correctly from audio queue state

**What's still unverified (Phase 4+):**
- Real reference client client connecting to ultra_tnc (manual operator test)
- Two-station hardware test where both ends run ultra_tnc and
  exchange HF-mail-style email
- Real mainstream Windows HF mail client on Windows
- Long-running stability (multi-hour sessions, repeated connect/
  disconnect cycles)
- `--inject-channel` integration testing

**Important compatibility note:**
Drop-in for **client software API** (TCP), NOT for **over-the-air
protocol**. Both ends in a conversation must run ProjectUltra; we
are not wire-compatible with legacy TNC's actual on-air waveforms. This
matches reference TNC's positioning — same TCP TNC API, custom on-air
protocol. Useful for:
- Private/emergency HF-mail-style HF email networks
- Replacing legacy TNC in self-contained meshes
- Free + open-source alternative to legacy TNC's $60–100 license

NOT useful for joining the existing global HF mail HF gateway
network on-air (those gateways run actual legacy TNC).

---

## 2026-05-02: TNC Phase 3a — ProtocolEngine surgery for TNC bridge

**What was added/fixed:**
Four protocol-layer changes to enable a future `ModemAdapter` bridge
(Phase 3b) to drive `ProtocolEngine` from the TNC reactor without
needing further surgery:

1. **Duplicate-data-callback fix** (real bug for raw-binary consumers):
   `connection_handlers.cpp:425+` previously fired
   `DataReceivedCallback` once per fragment AND once for the
   reassembled payload — would duplicate bytes on a TCP data stream.
   Fixed: intermediate fragments accumulate, callback fires once with
   complete payload. Codex repo-grepped for existing consumers and
   found none (`cli_simulator`, GUI, `modem_engine` use
   message/file/raw-modem-frame callbacks, not the
   `Connection::DataReceivedCallback`).

2. **`sendBinary(Bytes)` API**:
   - `Connection::sendBinary(Bytes)` — same SR-ARQ path as
     `sendMessage`, but emits v2 `DATA_START/CONT/END` frame types
     for unframed binary payloads (vs the text-marked DATA frames
     used by `sendMessage`).
   - `ProtocolEngine::sendBinary(Bytes)` proxy.
   - Refactored `Connection::sendMessage` through a shared
     `sendPayload()` helper. Existing message + file paths
     unchanged.

3. **`getTxBacklogBytes()` snapshot API**:
   - `Connection::getTxBacklogBytes()` returns total un-ACKed
     payload bytes (in-flight frames + pending fragments).
   - `SelectiveRepeatARQ::getTxInFlightPayloadBytes()` for the
     ARQ-window contribution.
   - `ProtocolEngine::getTxBacklogBytes()` proxy with mutex.

4. **`ProtocolEngine` data-received-callback proxy**:
   - `setDataReceivedCallback(...)` — wraps
     `Connection::setDataReceivedCallback`. Stored under the engine
     mutex; invoked from inside `onRxData()` while the engine mutex
     is held (matches existing callback patterns; no re-lock inside
     the lambda).

**Tests added:**
- `tests/test_protocol.cpp`: 3 new cases — binary fragment reassembly
  with single callback, arbitrary binary roundtrip via `sendBinary`,
  TX backlog snapshot accuracy. `test_protocol` internal count went
  19 → 22; CTest target count unchanged at 33.

**ctest:** 33/33 still pass.

**File summary:**
- `src/protocol/connection.{cpp,hpp}` — `sendBinary`,
  `getTxBacklogBytes`, refactored `sendMessage`
- `src/protocol/connection_handlers.cpp` — duplicate-callback fix
- `src/protocol/selective_repeat_arq.{cpp,hpp}` — typed DATA send
  helpers + RX frame-type tracking + payload-bytes snapshot
- `src/protocol/protocol_engine.{cpp,hpp}` — proxy methods
- `tests/test_protocol.cpp` — 3 new cases

**Wire format:** unchanged. Binary payloads use existing v2
`DATA_START/CONT/END` frame types. Pi side doesn't need a rebuild
to receive binary from a Mac running Phase 3a. (Concretely: the
500KB auto-rate sweep currently mid-flight on the cable continues
unaffected; Mac side is running the new binary, Pi side the old —
they interop because the wire is unchanged.)

**Known regressions risks (all assessed by Codex):**
- The duplicate-callback fix is the highest-risk change, but no
  existing consumer of `setDataReceivedCallback` was found. File
  transfer uses `FileTransferController` callbacks (different
  surface). Message TX uses `MessageReceivedCallback` (different
  surface). Codex marked this as the rollback candidate if hardware
  regression is observed.
- `sendBinary` and `getTxBacklogBytes` are additive — no
  behavioral change unless called.

**Next phase 3b:** create the `TNCBridge` that implements `ModemAdapter`
on top of the new ProtocolEngine APIs, plus the `ultra_tnc` binary
(audio + ProtocolEngine + bridge + TNCServer in one process).

---

## 2026-05-02: TNC Phase 2 — TCP reactor + integration tests

**Goal:**
Add the TCP socket layer for the legacy TNC interface. Single-thread
`poll()` reactor pattern (matching reference TNC's `tcp_interfaces.c`) so
all socket I/O + TNCSession dispatch + timers run on one thread,
avoiding ProtocolEngine reentrancy risk.

**What was added:**
- `src/tnc/tnc_server.{cpp,hpp}` — `TNCServer` with `TNCServerConfig`.
  Single poll() reactor thread that owns:
  - cmd port listener (default 8300, configurable; ephemeral 0 for tests)
  - data port listener (cmd_port+1)
  - the active client cmd + data fds (single client per port)
  - timer cadence (100ms tick → drives IAMALIVE, BUFFER rate-limit)
  - wakeup pipe + thread-safe queue for cross-thread modem events
- Single-client eviction: new cmd connection closes prior fds,
  resets TNCSession to IDLE, accepts the new client.
- Modem-side push API (`postModemConnected/Disconnected/PTT/...`)
  marshals events via the wakeup pipe; reactor drains queue and
  invokes TNCSession callbacks on its own thread.
- Reactor uses `signal(SIGPIPE, SIG_IGN)` and `SO_NOSIGPIPE` (macOS)
  + `TCP_NODELAY` on the cmd socket.

- `tests/test_tnc_server.cpp` — 18 integration cases: bind/ports,
  cmd/data clients, split-line input, eviction/reset, IAMALIVE
  override (test fast clock), modem post marshalling, buffer pacing
  override, data in/out, disconnect, stop/restart.

**ctest:** 32/32 → **33/33** (added `test_tnc_server`).

**Threading model:**
- Reactor thread is the ONLY thread that calls `TNCSession`. Modem
  callbacks marshal events; reactor drains and dispatches.
- Stop is cooperative: `stop_requested_` set, wakeup pipe written,
  thread joins cleanly, sockets closed.
- Restart is supported: `start()` after `stop()` re-binds. Tests
  cover this.

**Sandbox quirk:**
Codex flagged that the codex sandbox blocks localhost `bind()` with
EPERM, so the test binary has a preflight skip in that environment.
On the dev Mac (and the Pi when we deploy there) the tests run for
real. `ctest` passes either way.

**Phase 3 next:** wire the `ModemAdapter` interface to a real bridge
class that:
- Drives `ProtocolEngine` (CONNECT, DISCONNECT, sendBinary)
- Subscribes to ProtocolEngine state callbacks
- Fixes the duplicate-data callback in `connection_handlers.cpp:425-488`
- Adds a binary-bytes send API to `ProtocolEngine` (current
  `sendMessage(string)` is wrong abstraction for unframed TCP bytes)
- Adds a byte-level TX backlog snapshot to `Connection`/`ProtocolEngine`
- Infers PTT from `AudioEngine` queue state (not ARQ queue depth)
- Creates the `ultra_tnc` binary (audio + ProtocolEngine + bridge +
  TNCServer, all in one process)

---

## 2026-05-02: TNC Phase 1 — legacy-compatible TNC scaffold

**Goal:**
Add a legacy-compatible HF TCP TNC interface to ProjectUltra so existing
HF software (mainstream Windows HF mail client, reference client, packet-router client, alternative TNC client) can use this modem
as a drop-in legacy TNC replacement. This is Phase 1 of a 5-phase project
documented in `/tmp/tnc_architecture_plan.md` (private brief; will
be promoted to `docs/TNC_INTERFACE.md` when public-facing).

Phase 1 scope: standalone protocol module, no sockets, no real modem
hookup, no threading. Just the parser + state machine + a
`ModemAdapter` abstraction that Phase 3 will implement against the
real `ProtocolEngine`.

**What was added:**
- `src/tnc/tnc_events.hpp` (51 lines) — `TNCEvent` types + state enum
- `src/tnc/modem_adapter.hpp` (29 lines) — `ModemAdapter` abstract
  interface (setMyCall, setBandwidth, setListen, startConnect,
  disconnect, abort, sendBinary + snapshot accessors)
- `src/tnc/tnc_session.{hpp,cpp}` (806 lines) — `TNCSession` parser,
  FSM dispatcher, command handlers, event emitters. Implements 13
  legacy TNC core commands (MYCALL, BW2300/500/2750, LISTEN, CONNECT,
  DISCONNECT, ABORT, COMPRESSION, CHAT, VERSION, BUFFER, SN, BITRATE,
  CWID) + 7 reference-extension no-ops (P2P SESSION, CLIENT SESSION,
  PUBLIC, IGNOREKISSDCD, RETRIES, CALLINT, CQFRAME) for client
  compatibility. Async event helpers for CONNECTED, DISCONNECTED,
  PTT, BUFFER (rate-limited 1/sec), SN, IAMALIVE (60s timer).
- `tests/test_tnc_session.cpp` (748 lines, 88 unit cases) — covers:
  parser (10 cases), MYCALL (8), state transitions (18), modem
  events (17), data flow (6), tick/IAMALIVE (4), bandwidth (7),
  queries + no-ops (18). Includes `FakeModemAdapter` for tests.
- CMakeLists.txt + tests/CMakeLists.txt wiring.

**ctest:** 31/31 → **32/32** with new TNCSession test target.

**Architecture decisions (per Codex review of plan):**
- TNCSession lives outside ProtocolEngine (boundary preserved)
- reference-extension no-ops accepted silently (clients probe these)
- BW2750 accepted (not WRONG) — clients probe all bandwidths
- VERSION emits exact string `VERSION 4.9.0 registered\r` for
  reference TCP client regex compatibility
- BUFFER events rate-limited (1 emit per second + on change) per
  reference TNC reference
- IAMALIVE every 60s (reference client enforces 2-min read deadline)
- LISTEN OFF mid-session emits WRONG (per legacy TNC quirk; would tear
  link otherwise)

**Phase 2 next:** TCP reactor (single-thread `poll`-based, mirroring
reference TNC), localhost integration tests, single-client eviction
semantics. Reactor will own both ports + IAMALIVE timer; no
per-client threads (Codex flagged reentrancy risk in ProtocolEngine
if multi-threaded).

**Phase 3 next-next:** wire to ProtocolEngine. Will require fixing
the duplicate-data callback in `connection_handlers.cpp:425-488`
(currently emits both fragment + reassembled payload — would
duplicate bytes on the legacy TNC data stream), adding a binary-bytes
send API, and adding a byte-level TX backlog snapshot.

---

## 2026-05-02: Promote NVIS config to OFDM_COX default (round 7)

**What was changed:**
The `OFDMNvisWaveform()` default constructor now uses 1024-FFT, 59
carriers, MEDIUM CP — what used to be the explicit `createNvisMode()`
"NVIS preset". The old 512-FFT/30-carrier default is gone.

**Why:**
The 1024/59 config is strictly better in every measurement:
- Aligns with OFDM_CHIRP's geometry (which is also 1024/59 since
  commit `549349f` "Make 1024 FFT / 59 carriers the default OFDM
  config")
- More data carriers → higher gross throughput
- Narrower carrier spacing (46.875 vs 93.75 Hz) → measurable
  frequency-selective fading robustness (per tonight's QAM fading
  sweep, only the 1024/59 config decoded QAM16 R3/4 on Good fading
  at SNR=25; 512/30 default failed at every SNR)
- No backward-compat user — OFDM_COX was experimental and not
  yet wired into the auto-rate ladder

**Hardware verification:**
5 KB OFDM_COX QAM16 R3/4 SNR=22 AWGN with no `--ofdm-config` flag:
**2005 bps, 2 retx, 0 failed**. Matches the prior NVIS-preset numbers.

**ctest:** 31/31 still passing.

**File:** `src/waveform/ofdm_cox_waveform.cpp::OFDMNvisWaveform()`
constructor — replaced the 512/30 init with the 1024/59 init.
`createNvisMode()` factory still exists (now equivalent to default
construction) for backward compat with any caller still using it.

**The `--ofdm-config nvis` flag (round 6) is now a no-op** — both
"default" and "nvis" produce the same 1024/59 config. The flag is
kept for compatibility with existing test scripts; can be retired
later.

---

## 2026-05-02: OFDM_COX NVIS preset CLI wiring (round 6) — +25% throughput

**What was added:**
The `OFDMNvisWaveform::createNvisMode()` factory exists at
`src/waveform/ofdm_cox_waveform.cpp:36` with 1024-FFT, 59 carriers,
MEDIUM cyclic prefix — roughly 2× the data carriers vs the default
512-FFT/30-carrier preset. But it wasn't reachable from the CLI.
This round wires it up.

**What was changed:**
- `tools/cli_simulator.cpp`: new `--ofdm-config <default|nvis>` CLI flag.
  `default` = current 512-FFT/30-carrier behavior. `nvis` = factory
  preset. `--help` updated.
- Plumbed the preset into `Station` construction for both sim and
  hardware paths so the OFDM_COX waveform is created via
  `createNvisMode()` when `nvis` is selected.
- `tests/test_waveform_loopback.cpp`: new factory-derived QAM16 R3/4
  4-CW fixed-frame loopback test through the NVIS preset.
- `tests/test_ofdm_link_adaptation.cpp`: 59-carrier spacing-5
  pilot/data-carrier sanity checks (12 pilots, 47 data carriers).

**Test verification:**
- ctest: 31/31 pass.
- WaveformLoopback: 377/377; OFDMLinkAdaptation: 34/34.

**Hardware verification (Mac↔Pi cable + injected AWGN, OFDM_COX QAM16 R3/4):**

  | Test                         | Throughput | retx | Note |
  |------------------------------|------------|------|------|
  | 5 KB default config SNR=22   | 2007 bps   | 1    | (round 5c) |
  | 50 KB default cable AWGN     | n/a        | n/a  | not run today |
  | **50 KB NVIS preset SNR=22** | **2587 bps** | **3** | **+29% vs default** |

50 KB amortizes the inter-burst SACK round-trip more than 5 KB,
exposing more of the NVIS data-carrier advantage.

**Throughput plan progress (cumulative wins this overnight session):**
- Round 1: CW aggregation +15-22%
- Round 2b: HARQ -53% retx on hard channels
- Round 4: OFDM_COX end-to-end working
- Rounds 5a/5b: QAM16/32/64 selectable + decode integration
- Round 5c: QAM32 R3/4 fix (pilot density)
- Round 6: NVIS preset → 2587 bps QAM16 R3/4 (vs 2007 bps default)

**Compatibility caveat:**
`--ofdm-config nvis` is not on-air negotiated. Both peers must be
launched with the same flag, or OFDM_COX payloads will not be
compatible. For a hardware-loop test (where we control both sides),
this is straightforward via `EXTRA_CLI_ARGS` in run_hw_test.sh.

**Known limitation:**
- The `wideOFDMFrameTiming()` formula in connection_policy.hpp still
  uses the default OFDM-COX timing constants. Sample sizing comes
  from `getSamplesPerSymbol()` / `getMinSamplesForCWCount()` which
  ARE FFT-aware, so the path works — but the ACK-timeout formula
  may be slightly off for the NVIS config. Worth tuning if
  retx-storm patterns appear.
- QAM64 R3/4 still has the cliff issue (rolled back round 5d after
  it broke QAM32 R3/4). Separate round.

---

## 2026-05-02: QAM32 R3/4 pilot density fix (round 5c)

**What was broken:**
After rounds 5a+5b QAM16/32/64 were selectable on the CLI and decoded
correctly through the OFDM_COX path. But QAM32 R3/4 + QAM64 R3/4
both failed reliably on hardware at all tested SNRs (25, 28, 30 dB)
with the same pattern: 15-16 retx, 1 frame at max retries. QAM16 R3/4
worked fine at SNR=22+. R1/2 paths for all QAM modes worked.

**Root cause:**
`recommendedPilotSpacing()` in `include/ultra/ofdm_link_adaptation.hpp`
returned spacing=8 for **all** coherent R3/4 modes (QAM16/32/64).
That's fine for QAM16 — the constellation has enough min-distance
margin that loose pilot tracking still decodes. For QAM32/QAM64 at
R3/4 (low FEC redundancy + denser constellation), channel-estimate
drift between distant pilots accumulates phase error that exceeds
the constellation's decision regions before the next pilot arrives.

**What was changed:**
- `include/ultra/ofdm_link_adaptation.hpp`: when modulation is
  QAM32 or QAM64 AND code rate is R3/4, return pilot spacing=5
  (one pilot every 5 carriers) instead of 8.
  QAM16 R3/4 stays at spacing=8 (works fine, no need to pay the
  extra pilot overhead).
- `tests/test_ofdm_link_adaptation.cpp`: assertions for the new
  policy.
- `tests/test_waveform_loopback.cpp`: AWGN-margin loopback tests
  for QAM32 R3/4 at 25 dB and QAM64 R3/4 at 28 dB.

**Cost:**
Spacing 5 vs 8 means 1 pilot every 5 carriers vs every 8. On
59-carrier OFDM_COX, that's 12 pilots vs 7 → 47 data carriers vs 51
(8% reduction in data carriers). Modest cost in exchange for
unlocking QAM32 R3/4 throughput.

**Test verification:**
- ctest: 31/31 + WaveformLoopback 361/361 + OFDMLinkAdaptation 32/32
- Hardware test (Mac↔Pi cable + injected AWGN, 5 KB):

  | Mode  | Rate | SNR | Pre-fix     | Post-fix      |
  |-------|------|-----|-------------|---------------|
  | QAM16 | R3/4 | 22  | PASS (2007) | PASS (2058)   |
  | QAM32 | R3/4 | 25  | **FAIL**    | **PASS (2058)** |
  | QAM32 | R3/4 | 28  | **FAIL**    | **PASS (1959)** |
  | QAM64 | R3/4 | 28  | FAIL        | still FAIL    |
  | QAM64 | R3/4 | 30  | FAIL        | still FAIL    |

QAM32 R3/4 is now working.

**QAM64 R3/4 still failing — known limitation:**
Even with spacing=5 pilots, QAM64 R3/4 fails at SNR up to 30 dB.
The 64-point constellation has half the min-distance of 32-QAM, so
the same pilot density that works for QAM32 isn't enough. Likely
needs additional work (spacing=4 or even 3, decision-directed
channel tracking, or per-symbol equalizer changes). Out of scope
for this round.

**Throughput note:**
QAM32 R3/4 at 2058 bps matches QAM16 R3/4 in this test — both are
hitting the ARQ inter-burst SACK-round-trip ceiling on the 5 KB
test, not the modulation ceiling. Larger files would amortize the
gap further. The throughput "ladder" effect of higher QAM only
manifests on sustained transfers where the ARQ loop is amortized.

---

## 2026-05-02: QAM16/32/64 modes (round 5a + 5b)

**What was added:**
QAM16, QAM32, QAM64 modulation now wired through OFDM_COX end-to-end.
The modulator + demodulator + soft-demap for these constellations
already existed in the codebase (`src/ofdm/modulator.cpp`,
`soft_demap.hpp`); this work adds the integration so they're
actually selectable and decode on hardware.

**What was changed:**
- Round 5a — CLI exposure (`tools/cli_simulator.cpp`): `--mod`
  flag now accepts `qam16`/`qam32`/`qam64`. Help text updated.
  Unit-test additions in `tests/test_waveform_loopback.cpp` (333/333
  WaveformLoopback): roundtrip tests for QAM16/32/64 × R1/2 + R3/4
  via OFDM_COX, plus a deterministic AWGN-margin test
  (QAM16 R1/2 at 17 dB clean loopback).
- Round 5b — streaming integration fix (`src/gui/modem/streaming_decoder.cpp`,
  `src/gui/modem/streaming_decode_policy.hpp`): the connected-OFDM
  peek-escalation check was `soft_bits.size() < 2 * LDPC_BLOCK`.
  QAM16's robust control-sized peek produces *exactly* 2 complete CWs
  (1296 bits), which slipped through that test, so the receiver
  skipped escalation to a 4-CW fixed-frame decode and returned
  `cw_ok=0 cw_fail=0`. Added a sub-fixed-frame check
  (`hasSubFixedFrameSoftBits()` in the policy header) that also
  fires when 1–3 CWs of soft bits are present but a full fixed
  frame requires more — gated to OFDM_COX so the existing
  OFDM_CHIRP behavior is unchanged.
- Test in `tests/test_streaming_decode_policy.cpp`:
  `test_qam16_control_peek_is_subfixed` — verifies the 1296-bit
  QAM16 peek correctly triggers escalation.

**Hardware verification (Mac↔Pi cable + injected AWGN):**

Working ladder (5 KB R1/2 + R3/4 forced via `--mod` CLI):

| Mode  | Rate | SNR | Result | Throughput |
|-------|------|-----|--------|-----------|
| QPSK  | R1/2 | 20  | PASS   | 1011 bps (baseline) |
| QAM16 | R1/2 | 20  | PASS   | 1399 bps (+38%) |
| QAM16 | R3/4 | 22  | PASS   | **2007 bps** (+98% — top working) |
| QAM32 | R1/2 | 22  | PASS   | 1383 bps |
| QAM64 | R1/2 | 25  | PASS   | 1359 bps |

**Known limitations (R3/4 cliff for QAM32+):**
- QAM32 R3/4 fails at SNR=25 and SNR=28 (16 retx, 1 frame at max retries)
- QAM64 R3/4 fails at SNR=28 and SNR=30 (same pattern)
- QAM16 R3/4 works cleanly through SNR=22+

The cliff suggests phase-noise / channel-tracking limits at the
combination of dense constellation + low FEC redundancy. This is
under investigation as a follow-up round.

**Throughput plateau on R1/2:**
QAM16/32/64 R1/2 all cluster around ~1400 bps (data_phase ≈ 29 s
for 5 KB). At R1/2 the modulation gain is masked by the
inter-burst SACK round-trip ceiling. R3/4 has fewer round-trips
per file → real throughput reveal (QAM16 R3/4 = 2007 bps).
Larger files would amortize this further.

**ctest:** 31/31 + WaveformLoopback 339/339 + StreamingDecodePolicy
new test passes.

---

## 2026-05-02: Fix OFDM_COX end-to-end on hardware (round 4)

**What was broken:**
OFDM_COX was failing on hardware with `frames_sent=16, retx=224, failed=15` —
TEST FAILED on the simplest cable smoke test (5KB R1/2 AWGN SNR=20). The
mode worked enough to handshake and detect Schmidl-Cox sync, but data
frames never decoded. Per CLAUDE.md OFDM_COX was supposed to be working
at SNR=20+, but no recent hardware verification confirmed that.

**Root cause (real, not the brief's hypotheses):**
Two distinct bugs:

1. **Sample-sizing contract.** RX path's CW0 peek would escalate to a
   4-CW frame after reading TOTAL_CW from the header, but
   `OFDMNvisWaveform::getMinSamplesForFrame()` was still returning a
   1-CW-sized slice (~9216 samples ≈ 8 OFDM symbols). The decoder then
   fed only ~708 soft bits to LDPC — not enough to form even one
   648-bit codeword — and bailed with `cw_ok=0 cw_fail=0`. This
   matched the 16-retx pattern: every burst frame failed at the
   sample-sizing step.
2. **Schmidl-Cox sync alignment.** `OFDMDemodulator::searchForSync()`
   was returning the LTS position one OFDM symbol too early, landing
   on the final STS symbol instead of the first LTS pair. Subsequent
   frame demod started from a bad anchor.

The OFDM_COX path had drifted away from the multi-CW fixed-frame
geometry that round 1 (CW aggregation) introduced.

**What was changed:**
- `src/waveform/ofdm_cox_waveform.cpp`:
  - Corrected COX full preamble length to 7 OFDM symbols (was 6).
  - `getMinSamplesForFrame()` now reflects the default 4-CW fixed
    frame, not 1 CW.
  - Added 1-CW control sizing helper.
  - Added exact `getMinSamplesForCWCount()` so the consumer can
    request the correct sample count based on the actual CW count
    after CW0 peek.
- `src/ofdm/demodulator.cpp::searchForSync()`: external Schmidl-Cox
  LTS-start selection now subtracts an OFDM symbol only when the
  previous position is actually an LTS pair (verified via
  correlation magnitude check, threshold 0.85). Avoids the
  off-by-one that was landing on the trailing STS.
- `tests/test_waveform_loopback.cpp`:
  - `test_ofdm_cox_fixed_frame_roundtrip` — encode/decode a 4-CW R1/2
    OFDM_COX fixed frame, verify payload roundtrip
  - `test_ofdm_cox_16_frame_burst_roundtrip` — encode 16 frames in
    a burst, decode all 16, verify each.

These tests would have caught the bug before hardware time.

**How it's properly fixed:**
- The sample-sizing contract is now consistent across CW0 peek and
  the full-frame decode: both ask `getMinSamplesForCWCount(N)` for
  the right N CWs at the current rate.
- The sync alignment fix is gated on a magnitude check, so it only
  fires when the previous position is plausibly an LTS pair —
  doesn't introduce false alignments at low SNR.
- No shared-state changes, no mutex changes, no thread handoffs
  affected. Round 3's mutex-crash failure mode does not apply here.

**Test verification:**
- `cmake --build build -j4`: passed.
- `ctest --test-dir build --output-on-failure`: 31/31 pass.
- Internal `WaveformLoopback` count went from 216 → 218 (the 2 new
  COX tests).
- Hardware test (Mac↔Pi cable + injected AWGN SNR=20):
  ```
  EXTRA_CLI_ARGS="--waveform ofdm_cox" ./tools/run_hw_test.sh \
    --file 5120 --rate r1_2 --snr 20 --channel awgn --inject
  ```
  Result: PASS, 39 frames sent, 16 retx, 0 failed, 1093 bps.
  Pre-fix: 16 frames sent, 224 retx, 15 failed, transfer FAILED.
- Logs: `/tmp/ultra_hw_20260501_223533`.

**Throughput note:**
1093 bps OFDM_COX QPSK is slightly below 1280 bps OFDM_CHIRP DQPSK at
the same R1/2 — because the 16 retx ate airtime. The lighter Schmidl-Cox
preamble gives a per-frame airtime advantage that this run didn't
realize because of the retx storm. Unlocking COX's actual throughput
advantage requires either tuning sync stability further, OR — much
bigger leverage — wiring QAM16/32/64 modulation through the COX path
(round 5+). At QAM16 R1/2 the theoretical rate is roughly 2x QPSK; at
QAM64 R3/4 around 6x.

**Known limitations:**
- 40% retx rate on this run is high. The sync detection still has
  some marginal positions that fail to decode cleanly even at SNR=20
  AWGN. Worth investigating if retx rate stays high on QAM tests.
- Default OFDM_COX config is 512-FFT/30-carrier QPSK. The NVIS-style
  1024-FFT/59-carrier preset (`createNvisMode()`) is not yet wired
  through the cli_simulator path.
- Auto-rate ladder still doesn't promote to OFDM_COX or to QAM modes.
  This round only validates the path works; promotion is a separate
  round.

**Path forward:**
Round 5a: wire QAM16 modulator + demodulator + soft-bit demap.
Round 5b: hardware-validate QAM16 R1/2 + R3/4 at SNR=20+.
Round 6: QAM32. Then auto-rate ladder integration.

---

## 2026-05-01: RX-side soft-combining HARQ (Chase combining) — round 2b

**What was missing:**
On retx-heavy channels (Moderate/Poor/Flutter fading, low SNR),
every retransmitted frame was wasted airtime — receiver would
discard the failed soft bits, demand a fresh copy, decode that
in isolation. Commercial modems (LTE/HSDPA HARQ pattern) accumulate
soft LLRs across attempts so each retx delivers coding gain
(~3 dB per doubling of attempts).

**What was added:**
Receiver-side **Chase combining**. When a fixed-frame fails LDPC
decode, the receiver retains the soft LLRs keyed by (sender_hash,
seq, rate, cw_count). On the next retransmission, new LLRs are
arithmetic-averaged with the stored ones, then LDPC runs on the
combined buffer. After N attempts the effective SNR margin is
~10·log10(N) dB. Default OFF — opt-in via `Connection::setSoftCombiningHARQ(true)`
or `cli_simulator --harq`.

TX path unchanged: Chase combining only requires identical retx
bits, which we already have. (Incremental Redundancy would need
TX-side surgery; out of scope for this round.)

**Files added/changed:**
- `src/fec/soft_combine.{cpp,hpp}` — new `SoftCombineBuffer` class
  with TTL eviction (default 30 s), LRU at max_entries (default 32),
  arithmetic-average LLR accumulation, drop on success.
- `src/protocol/frame_v2.{cpp,hpp}` — `decodeFixedFrame()` accepts
  optional `harq_buffer*` and `key`. When non-null, combines LLRs
  before LDPC and stores combined output if decode fails.
- `src/gui/modem/streaming_decoder.{cpp,hpp}` — owns the buffer,
  builds the key from decoded CW0 header (peek-and-probe path),
  passes both into `decodeFixedFrame()`.
- `src/protocol/connection.{cpp,hpp}` — manages buffer lifecycle:
  `setSoftCombiningHARQ(bool)` API, `tick()` evicts old entries,
  `enterDisconnected()` clears.
- `tools/cli_simulator.cpp` — `--harq` CLI flag.
- `tests/test_soft_combine.cpp` — 7 unit tests covering no-op when
  disabled, identity on first attempt, averaging math, drop on
  success, TTL eviction, max-entries LRU eviction, key
  disambiguation.

**Memory bound:**
LLR vector at CW=6 R1/2 = 6 × 324 bits = 1944 floats = ~7.6 KB/entry.
At CW=8 R1/4 = 8 × 486 = ~15 KB/entry (worst case). Default 32-entry
buffer ≈ 250–500 KB peak.

**Test verification:**
- ctest: 31/31 pass (added SoftCombine 7/7).
- Hardware sweep, 5 KB R1/2 forced, 4 channels × HARQ on/off:

  | Channel | HARQ=off | HARQ=on | Δ |
  |---------|----------|---------|---|
  | GOOD15 CW=6 R1/2 | 1451 bps, 0 retx | 1443 bps, 0 retx | -0.6% (within noise; no regression on clean) |
  | MOD12 CW=6 R1/2  | 1468 bps, 0 retx | 1460 bps, 0 retx | within noise; channel too clean |
  | POOR15 CW=6 R1/4 | 244 bps, **55 retx, 44 to** | 257 bps, **26 retx, 19 to** | **+5% throughput, −53% retx, −57% timeouts** |
  | FLUTTER15 CW=4 R1/4 | TEST FAILED (channel limit) | TEST FAILED | 10 Hz Doppler exceeds R1/4 even with HARQ |

  Hardware logs: `/tmp/ultra_hw_20260501_2025*` and `/tmp/harq_sweep_summary.txt`.

**Adopted policy: opt-in default OFF.** Hardware confirms HARQ engages
correctly on retx-heavy channels (Poor fading) and is a no-op on clean
channels. The retx reduction is the headline win — the modem stops
burning airtime on duplicate-without-progress retransmissions. Default
stays off until we collect more field data; promote when ready.

**Known limitations:**
- Flutter (10 Hz Doppler) still exceeds R1/4 PHY decode capability
  even with HARQ — this is a frame-length-vs-coherence-time mismatch,
  not a HARQ bug. Round 3 (longer LDPC codewords) might help; round
  3a (per-CW partial recovery) almost certainly will.
- Key includes (sender_hash, seq, rate, cw_count) but not modulation
  or session epoch. A same-rate/same-CW modulation change before TTL
  could combine wrong frames; mitigated by 30 s TTL and
  enterDisconnected() clear.
- Default OFF; not yet wired to auto-enable based on observed retx
  rate. Add later if the use case warrants.

**Throughput plan progress (cumulative):**
Round 1 (CW aggregation): +15-22% on every channel — DONE.
Round 2b (RX HARQ): -53% retx on retx-heavy channels — DONE.
Round 2a (per-CW partial recovery / block-ACK): pending — would
help Flutter and Poor R1/4 cliff cases.
Round 3 (longer LDPC, 1944-bit): pending.
Round 4 (D8PSK R3/4 hw validation): pending.

---

## 2026-05-01: Adaptive CWs-per-frame aggregation — +15-22% throughput

**What was broken:**
Fixed-frame data carried exactly 4 LDPC codewords. Per-frame ACK
overhead capped throughput at ~1280 bps for 5 KB R1/2 transfers
across all SNR/fading conditions where retx≈0. Commercial HF modems
amortize over larger aggregates (e.g. 802.11n A-MPDU). Codex review
of throughput plan recommended adaptive CWs-per-frame as round 1
(vs blind switch to 8) — measure 4/6/8 across channels.

Three sub-bugs surfaced during the work:
1. Solo-frame RX path used stale CW count — every solo frame retx'd
   once before the receiver could decode.
2. ACK timeout formula clamped at 16 s. CW=6 needs ~24 s, CW=8 ~31 s.
   With clamp, A timed out before B could SACK, all frames retx'd.
3. `queued_tail_margin_ms` in the timeout formula double-counted
   `tx_burst_ms` — added an extra `(window-4) * data_ms` of margin.

**What was changed:**
Round 1 — variable CWs per fixed data frame (default 4, selectable
1–8):
- `src/protocol/frame_v2.{cpp,hpp}` — `FIXED_FRAME_CODEWORDS` lifted
  from constexpr to a runtime parameter. `getFixedFramePayloadCapacity()`
  + `makeFixedDataFrame()` + `decodeFixedFrame()` now take a
  `cw_count`. Receiver validation relaxed from `== 4` to
  `1..kMaxFixedFrameCodewords`. The wire format already carried
  `TOTAL_CW` in the header.
- `src/fec/{frame,burst}_interleaver.{cpp,hpp}` — interleavers
  parametric on CW count.
- `src/protocol/selective_repeat_arq.{cpp,hpp}` — TXSlot tracks
  CW count; `sendFixedDataWithFlags()` accepts it.
- `src/protocol/connection.{cpp,hpp}` — `data_frame_cw_count_`
  member + `setForcedFrameCodewords()` setter, propagated through
  `applyDataMode()`.
- `src/protocol/connection_policy.hpp` — `wideOFDMFrameTiming()`
  scales `data_ms` with CW count.
- `src/gui/modem/streaming_{encoder,decoder}.{cpp,hpp}` — encoder
  + decoder pick up the configured count.
- `tools/cli_simulator.cpp` — `--cw-count <N>` CLI flag.

Round 1.5 — fix solo-frame RX path:
- `src/gui/modem/streaming_decoder.cpp` — RX peek-and-probe now
  reads `TOTAL_CW` from the decoded CW0 header and re-issues
  `decodeFixedFrame()` with the header-derived count if it
  differs from the initially-tried count. Frame interleaver gate
  also sized by header count.

Round 1.6 — ACK timeout formula:
- `src/protocol/connection_policy.hpp` `computeWideOFDMAckTimeoutMs()`:
  removed the `queued_tail_margin_ms` double-count; clamp ceiling
  is now `max(16000u, 3 * tx_burst_ms)` for `cw_count > 4`, kept
  at strict 16 s for default 4-CW behavior.

**How it's properly fixed:**
- The wire format already supported variable counts (TOTAL_CW byte
  in the header). The work was uniformly threading `cw_count`
  through every encode/decode/interleave site.
- Receiver header-driven retry handles edge cases where the
  initial guess was wrong (stale config, mode change races).
- The expanded ACK timeout means SACK-round-trip airtime fits
  within the timeout budget for CW=6/8 windows.

**Test verification:**
- `cmake --build build -j4 && ctest --test-dir build --output-on-failure`
  → 30/30 pass, plus `FrameV2: 29/29` (added 12 roundtrips for
  `4/6/8 CW × 4 rates`) and `ConnectionPolicy: 74/74` (added
  CW-count-scaling tests for the timeout formula).
- 9-cell hardware sweep (3 channels × 3 CW counts × forced R1/2,
  Mac↔Pi cable + `--inject-channel`):

  | Channel | CW=4 | CW=6 | CW=8 |
  |---|---|---|---|
  | AWGN20 | 1286 (0r) | **1476** (0r) | **1477** (0r) |
  | GOOD15 | 1280 (0r) | **1477** (0r) | 713 (1r) ⚠ |
  | MOD15 | 1274 (0r) | **1477** (0r) | **1469** (0r) |

- 50 KB GOOD15 CW=6: **1560 bps**, 0 retx, 100% success — confirms
  the gain scales modestly with file size.
- Pre-fix CW=6/8 sweep showed 25/25 retx on AWGN/Good/Moderate
  (channel-independent, identical numbers — proved structural bug).
  Logs: `/tmp/ultra_hw_20260501_185153` etc.

**Adopt CW=6 as default OFDM data-frame size for OFDM_CHIRP.** It's a
+15-22% throughput win, channel-robust (0 retx across AWGN/Good/Moderate),
and ctest green. CW=8 still has a Good-fading edge case (one bad cell
showed 1 retx + 30 s recovery) — not yet recommended for default.

**Known limitations:**
- CW=8 on Good fading: long frame TX (~1.2 s) at 0.1 Hz Doppler can
  span a coherence dip; one fade ≈ 30 s recovery. Don't ship CW=8
  default until per-CW partial recovery (round 2a) lands.
- The 1477 bps "ceiling" on 5 KB R1/2 CW=6 is bounded by ARQ
  inter-burst SACK round-trip gap, not channel quality. Larger
  files do better (50 KB → 1560 bps).
- Default for now stays at CW=4 to avoid regressing existing tests
  + workflows; opt-in via `--cw-count 6` until promoted.

---

## 2026-05-01: Adaptive code-rate selection — full end-to-end working

**What was broken:**
The adaptive mode controller (introduced earlier in the session) shipped with
the right shape but the wrong lifetime. On hardware tests it manifested in
four layers, each surfaced only after the previous was fixed:

1. **Stuck downgrade under retry pressure.** `tryIssueAdaptiveModeChangeAtBoundary()`
   required `availableSlots == windowSize` (full window drain) before any
   MODE_CHANGE could fire. Retx storms keep the window populated — exactly
   the case where a downgrade is needed — so the queued downgrade got stuck
   indefinitely.

2. **Thrashing after recoverable downgrade.** Once the boundary check was
   relaxed, downgrades fired correctly, but the controller would re-upgrade
   immediately because the next 3 evaluation windows looked "clean" — they
   only looked clean because the rate was just lowered. Hardware test
   showed 7 mode changes in 200 s and final failure at max retries.

3. **Stuck downgrade under severe pressure.** With the half-window relaxation,
   sustained timeouts kept `availableSlots * 2 < windowSize`. The downgrade
   queued every 1 s for >150 s without firing; first frame never delivered.

4. **In-flight retx ignored rate change.** Even after the controller fired
   correctly, ARQ retransmits the **cached** `tx_window_[slot].frame_data`
   bytes — encoded at the OLD rate. After MODE_CHANGE, those bytes are
   payload-too-large for the new rate's fixed-frame; receiver can't decode.
   ARQ retries 15× → max retries → fail.

**What was changed:**
Four-round patch series, all on top of the existing controller:

- `src/protocol/connection.cpp,hpp`:
  - **Round 1:** `canIssueAdaptiveModeChange(bool is_downgrade)` accepts
    `available_slots * 2 >= window_size` for downgrades only. Upgrades keep
    strict `==` to avoid losing in-flight DATA on the more-robust rate.
  - **Round 2:** `ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS = 15000` blocks
    upgrades for 15 s after a downgrade fires. Re-armed after each
    `applyDataMode()` resets state.
  - **Round 3:** `ADAPTIVE_DOWNGRADE_FORCE_MS = 6000` — when a downgrade
    has been queued >6 s without firing because of the boundary check,
    force the MODE_CHANGE regardless of window state. WARN-logged.
- `src/protocol/selective_repeat_arq.cpp,hpp` (Round 4):
  - `setCodeRate()` moved out-of-line; on rate change, walks `tx_window_`,
    aborts active+un-ACK'd slots, resets in-flight bookkeeping, rewinds
    TX seq to the current ACK base. Logs WARN with abort count.
- `src/protocol/file_transfer.cpp,hpp` (Round 4): adds requeue path so the
  chunker rewinds the file offset when ARQ aborts the in-flight slots,
  letting the next pull regenerate the right chunks at the new rate.

Also added in this session by ChatGPT 5.5 (Codex):
- `dataFrameFlags()` helper preserves `VERSION_V2` bit on data frames
  (was being clobbered by `frame.flags = flags`). Regression test in
  `tests/test_selective_repeat.cpp`.
- `connectAckRetransmitDelayMs()` adapts CONNECT_ACK rescue retransmit
  timing so the retx doesn't fire into the responder's first OFDM
  burst-interleaver group on the success path.
- `isAddressedToCallsign()` filter drops cross-talk frames at
  `deliverFrame`, `processRxBuffer`, and the cli_simulator RX path.
- `makeOFDMBurstPadPayload()` uses xorshift32 + 0x7F discriminator
  instead of all-zero pad, reducing fading-tail "4/4 CWs OK but frame
  invalid" artifacts.
- `ofdmWindowSizeForChannel()` channel-aware window size wrapper.
- `applyDataMode()` / `configureArqForCurrentDataMode()` refactor pulls
  shared logic out of `enterConnected()` and `handleModeChange()`.

Tests added:
- `tests/test_connection_adaptive.cpp` — new file, 28 tests covering:
  initial-mode pick, bootstrap cap, upgrade backlog gate, downgrade
  retry-pressure trigger, half-window boundary, post-downgrade lockout
  arming + expiry, stuck-downgrade force-after-timeout, upgrade NOT
  forced after timeout, forced-rate disables controller.
- `tests/test_selective_repeat.cpp` — `test_data_flags_preserve_version_bit`
  and `test_code_rate_change_aborts_in_flight_fixed_frames`.
- `tests/test_connection_policy.cpp` — `connectAckRetransmitDelayMs()`
  expectations.

**How it's properly fixed:**
- Asymmetric boundary check matches asymmetric semantics. Downgrades are
  recovery (in-flight frames at the failing rate are doomed anyway);
  upgrades risk losing good progress (in-flight frames at the safer
  rate need to clear cleanly first).
- Lockout prevents the controller from interpreting "no retx after
  downgrade" as channel improvement.
- Force-after-timeout is the escape hatch when even half-window can't
  drain — at that point the in-flight frames will fail anyway, so
  switching to a safer rate is strictly better than waiting.
- ARQ abort + file-transfer rewind keeps the frame-encoding rate
  consistent with the ARQ window contents. The cost is a few seconds
  of duplicated TX work; the alternative is the frame-encoding rate
  drift bug (frames pre-encoded at old rate sent forever after rate
  change).

**Test verification:**
- `cmake --build build -j4 && ctest --test-dir build --output-on-failure`
  → 30/30 pass, including 28 new ConnectionAdaptive tests.
- Hardware test (Mac↔Pi USB cable, channel injection):
  `SSH_KEY=$HOME/.ssh/id_pi5 ./tools/run_hw_test.sh --file 51200 \
   --rate auto --snr 20 --channel awgn --inject`
  → 50 KB delivered, 0 frames failed, 478 frames sent, 84 retx,
  4 forced downgrades + 4 normal MODE_CHANGEs, 986 bps throughput.
  Logs at `/tmp/ultra_hw_20260501_173642`.
- Pre-patch result on the same workload: failed at max retries
  (`/tmp/ultra_hw_20260501_170101` — 6 frames failed at seq=25-31
  because retx kept transmitting old-rate-encoded payloads).

**Auto rate ladder honored:**
`recommendDataModeForWaveform(snr, fading)` (the existing ladder in
`waveform_selection.hpp`) is the source of truth. Bootstrap cap drops
the initial pick one notch on borderline OFDM channels. Adaptive
controller can move freely up/down within that ladder during a file
transfer based on observed retx pressure and clean-window count.

**Known limitations:**
- Auto on the 50 KB AWGN-injected test runs ~58% of forced-R1/2
  throughput (986 vs 1692 bps). The auto path pays time at every
  rate including R1/4 transitions; if SNR is *known* to support R1/2,
  forcing it is faster. Auto is the right call when channel is unknown
  or varying.
- Non-file in-flight DATA (single messages) doesn't have an equivalent
  rewind path. If MODE_CHANGE fires while a non-file payload is
  in-flight, that payload is dropped. Acceptable for now: messages
  are short and unlikely to overlap with adaptive transitions.

---

## 2026-04-26: ack_repeat=1 on near-AWGN — sustained file-transfer throughput

**What was broken:**
After the previous "ACK repeats only for selective SACKs" change in `beb86cb` and
the SRTT-aware timeout floor, sustained 50 KB transfers at SNR=20 AWGN (DQPSK
R2/3) still showed wide variance: 199s/290s/229s wall across 3 seeds. The bad
seed (290s) burned channel time on duplicate SACK copies that were never needed
— BRAVO was scheduling 2 ACK_REPEAT copies for every selective SACK, but at
near-AWGN with SNR≥15 a single SACK is delivered cleanly.

**What was changed:**
- `src/protocol/connection.cpp:1033-1041` (in `enterConnected()` OFDM branch):
  drop `ack_repeat_count` from 2 to 1 when `fading_index_ < 0.30f &&
  measured_snr_db_ >= 15.0f`. D8PSK R1/2 path (which forces ack_repeat=3 for
  diversity) is unchanged. Good fading and worse remain at 2.

**How it's properly fixed:**
- The threshold matches the auto-selector's true-AWGN bucket (< 0.15 in CLAUDE.md
  but expanded to 0.30 to absorb measurement jitter at the boundary).
- SRTT-aware ACK timeout (~750ms on these profiles) recovers any genuinely-lost
  SACK quickly enough that the duplicate copy isn't structurally needed.
- Conservative: tested expanding to `< 0.65` (good fading) and saw a real
  regression — SNR=20 good seed 1 went from r=33/t=17 (pass) to r=81/t=60 (fail).
  Without the redundant copy, brief fading nulls cause SACK loss and trigger
  retx storms. Stayed at near-AWGN.

**Test verification:**
50 KB at SNR=20 AWGN, 3 seeds:
| Seed | v2 (pre-fix) | post-fix |
|---|---|---|
| 1 | 199s, retx=5, timeouts=3 | 201s, retx=8, timeouts=4 |
| 2 | 290s, retx=131, timeouts=124 | 205s, retx=5, timeouts=3 |
| 3 | 229s, retx=44, timeouts=37 | 211s, retx=21, timeouts=16 |

Mean wall: 239s → 206s (~14% faster, much tighter variance). The bulk of the
seed-2 win comes from the SRTT-floor fix landing alongside; the ack_repeat
reduction contributes a steadier ~3-5% on its own.

No regressions on SNR=15/20 good (criteria didn't activate). Did not improve
SNR=15 moderate (criteria didn't activate; that cell's bottleneck is PHY+ARQ
thrashing, not control-frame overhead).

**Invariants:**
- The 0.30 threshold is a soft floor — moving it up to 0.65 (good fading)
  caused regression. Don't widen without re-measuring on borderline good-fading
  seeds.
- SRTT-aware ACK timeout floor is required for this to work safely; with the
  pre-fix 2250ms floor a lost SACK would have meant a 2.25s wait, making the
  redundant copy load-bearing.

---

## 2026-04-26: SRTT-aware adaptive ACK-timeout floor — file-transfer throughput recovery

**What was broken:**
After the prior "Stabilize OFDM ARQ under ACK decoder load" commit (`beb86cb`)
bounded the OFDM retx storm by shrinking the window to 4 and skipping ACK
repeats for cumulative-only ACKs, sustained file transfers (50 KB+) at
DQPSK R1/2 still showed pathological timeout counts (8–15 timeouts on
SNR=15 good seeds where PHY decode succeeds 99.9% of the time). On the
hardest production cell (DQPSK R1/2 SNR=15 moderate), 50 KB transfers still
fell over the 300s test budget with retx=66–76, timeouts=8–16.

Root cause: in `selective_repeat_arq.cpp:665` the adaptive ACK-timeout
floor was `std::max(1200u, config_.ack_timeout_ms / 2)`. For OFDM DQPSK R1/2
with window=4, `config_.ack_timeout_ms` is clamped at 4500ms (lower bound
in `connection.cpp:56`), so the floor evaluated to **2250ms** — over 3×
the typical observed RTT (~600ms). Every "lost ACK" recovery cost 2.25s
of pure wait, even on clean channels. The retx-skipping change in
`beb86cb` made this worse: cumulative ACKs that get lost now wait the full
2.25s before retx, instead of being saved by a redundant repeat copy.

**What was changed:**
- `src/protocol/selective_repeat_arq.cpp:665`: split the floor into
  pre-RTT and post-RTT cases. Once `have_rtt_estimator_` is true, the
  floor becomes `clamp(srtt_ms_ * 1.5f, 600, 2500)`. Until the first
  valid RTT sample arrives, keep the original conservative floor.

**How it's properly fixed:**
- The 1.5× SRTT floor lets the estimator collapse close to actual RTT
  on a clean channel — where SRTT settles around 500ms, RTO can drop to
  ~750ms instead of being pinned at 2250ms. That's a 3× reduction in
  per-timeout wait cost.
- Bounded by 600ms hard minimum (premature retx still hurts) and 2500ms
  upper (so a transient RTT spike can't sabotage the floor permanently).
- Karn safety preserved: retransmitted slots are still flagged
  `rtt_sample_eligible = false` (line 646), so the estimator only sees
  unambiguous round-trip samples.

**Test verification:**
```
./build/cli_simulator --snr 15 --channel good --seed 1 --file 25600
```
Pre-fix (`beb86cb`): expected ~12–15 timeouts based on 50 KB extrapolation,
~250s wall.
Post-fix: 25 KB transferred in **146.7s data-phase (1396 bps), 170s wall**,
ARQ stats `retransmissions=20 timeouts=4` (timeout count dropped ~3×, retx
mix shifted to SACK/hole-probe driven instead of timeout-driven).

**Invariants:**
- The post-RTT floor must stay ≥ 600ms. Below that, normal scheduling
  jitter (sack_delay=120ms, ack_repeat_delay=220ms, decode latency) starts
  fighting the timer.
- The Karn-style RTT-eligibility flag must continue to skip retransmitted
  slots — without it, the estimator would be biased low on stormy seeds
  and the floor would stay too tight for safety.

---

## 2026-04-26: Proactive CONNECT_ACK retransmission — handshake recovery on faded seeds

**What was broken:**
Auto-mode baseline (cli_simulator, no `--mod`/`--rate` forcing) at DQPSK R1/2 SNR=15
moderate fading showed 4/5 message tests and 2/3 file 2048 tests passing. The single
failure was always the same fingerprint: ALPHA never received CONNECT_ACK, sat in
CONNECTING state until cli_simulator's 30s PHASE 1 timeout cut the test off. The
protocol's `connect_timeout_ms = 60000` would have triggered a CONNECT retry
eventually, but only well after the harness gave up — and in production, real users
would just see a "connection timeout" with no recovery in 30s.

Root cause: BRAVO (responder) sends a single CONNECT_ACK and then waits. If ALPHA's
LDPC decode of that one MC-DPSK ACK fails on a faded seed, there's no retry. The
existing 2.2s "responder fail-safe" only forced internal handshake completion on
BRAVO — it didn't re-send the ACK, and BRAVO's encoder/decoder were already past
the handshake state by then.

**What was changed:**
- `src/protocol/connection.hpp`: added `connect_ack_frame_`, `connect_ack_retransmit_ms_`,
  `connect_ack_retx_remaining_` member state + `CONNECT_ACK_RETRANSMIT_MS = 6000`,
  `CONNECT_ACK_MAX_RETX = 1` constants. Public `isInitiator()` and `isHandshakeConfirmed()`
  accessors for modem-layer use. (Cap is 1 — see "Why 1 retx, not 2" below.)
- `src/protocol/connection_handlers.cpp`: in `handleConnect()`, after `transmitFrame(ack_data)`,
  cache `connect_ack_frame_ = ack_data` and arm the retx interval/counter.
- `src/protocol/connection.cpp`:
  - Tick CONNECTED state: when `negotiated_mode_ == OFDM_CHIRP` and retx_remaining > 0,
    re-send the cached ACK every `CONNECT_ACK_RETRANSMIT_MS`. Decoupled from
    `handshake_confirmed_` so it survives the 2.2s fail-safe.
  - In `onFrameReceived()`: any frame from initiator clears retx state immediately —
    "ALPHA spoke" is sufficient signal that the original ACK got through.
  - `enterDisconnected()` and the `cancelTx()` reset path also clear retx state.

**How it's properly fixed:**
- First retx fires 6s after the original CONNECT_ACK send. That's *after* the OFDM
  round-trip (~5s for ALPHA to decode + send first DATA), so the success case clears
  retx state via `onFrameReceived()` before any retx fires. Verified in v6 baseline:
  retx mean dropped from 1 → 0 at the targeted cell.
- The retx is gated to OFDM_CHIRP only. MC-DPSK and OFDM_NARROW have ~12-16s round
  trips — retx at 6s would clog the channel ahead of the first ACK and hurt more
  than help. Empirically confirmed in v3/v4 attempts where ungated retx regressed
  SNR=5 MC-DPSK from 5/5 → 3/5.
- `transmitFrame()` in cli_simulator (and modem_engine.cpp's symmetric path) already
  special-cases CONNECT/CONNECT_ACK frame types (0x12/0x13) to encode in MC-DPSK
  regardless of negotiated waveform — so the cached bytes go out in MC-DPSK on each
  retx even though BRAVO's encoder mode is OFDM_CHIRP by then. No modem-layer changes
  required.
- Fail-safe (RESPONDER_HANDSHAKE_FAILSAFE_MS = 2200) unchanged — preserves the existing
  "first OFDM data frame lost" recovery path.

**Test verification:**
```
./build/cli_simulator --snr 15 --channel moderate --seed 5
```
Pre-fix: TEST FAILED at PHASE 1 timeout (30s wall, ALPHA never decoded ACK).
Post-fix: TEST PASSED. Log shows `Re-sending CONNECT_ACK (proactive, 1 retx remaining)`
at ~14.5s, ALPHA decodes retx by ~17s, full handshake + 7-message data exchange
completes by 30s.

Auto-mode baseline (5 seeds msg + 3 seeds file across 6 SNR×channel cells, 48 runs total):
- Pre-fix: 46/48 pass. 2 failures, both DQPSK R1/2 moderate SNR=15 handshake.
- Post-fix: 47/48 pass. The targeted cells (m_snr15_moderate, f_snr15_moderate) are now
  5/5 and 3/3. Remaining 1 failure is f_snr05_good seed 1 — MC-DPSK file mode where retx
  is intentionally not enabled; this seed is unstable across re-runs (cli_simulator's
  wall-clock-driven pacing introduces nondeterminism), not caused by this fix.

**Why 1 retx, not 2:**
File-transfer timing analysis on a PHY-stress seed (SNR=15 good seed 7) showed BRAVO's
LDPC decode chain stuck in false-sync rejections for ~13s after ALPHA's first burst.
During that window neither retx fired the `clear-on-onFrameReceived` hook, so both
retx attempts went out — each an extra ~5s of MC-DPSK audio in BRAVO's TX queue,
delaying real ACK traffic and triggering ARQ timeout cascades. The targeted bug
(m_snr15_moderate seed 5) recovered with the 1st retx in v6 testing — the 2nd was
already redundant. 1-retx version validated: m_snr15_moderate stayed 5/5, no
regressions on OFDM cells.

**Invariants:**
- Retx only fires when `negotiated_mode_ == WaveformMode::OFDM_CHIRP`. Do not extend
  to MC-DPSK or OFDM_NARROW without re-validating round-trip timing — those modes'
  RTT is longer than the retx interval and would cause channel congestion.
- The retx of a cached ACK is fire-and-forget — `transmitFrame()` overrides the
  encoder mode for type 0x13 frames. If you ever rip out that override, this fix
  silently goes out in OFDM and ALPHA (still in MC-DPSK CONNECTING) won't decode it.
- `connect_ack_frame_` must be cleared on disconnect/reset paths to avoid stale
  retx after a subsequent connection.

---

## 2026-03-15: CPE correction for differential modes — higher throughput on fading

**What was broken:**
DQPSK/D8PSK modes had no per-symbol phase tracking. Channel estimate phase was frozen from
LTS training symbols. On fading channels, channel phase drifts mid-frame (~5°/symbol at 0.5 Hz
Doppler), degrading MMSE equalization quality and causing ~89% CW success on moderate fading.
R2/3 required SNR≥20 even on good fading because the stale phase caused too many CW failures.

**What was changed:**
- `src/ofdm/channel_equalizer.cpp`: Removed `if (!is_differential)` gate on CPE correction block.
  Now estimates Common Phase Error from pilot LS vs channel_estimate per symbol for ALL modes.
  For differential modes, CPE is clamped to ±15° per symbol to prevent overcorrection from noisy
  fading pilots (6 pilots, ~4° estimation noise at SNR=15).
- `src/protocol/waveform_selection.hpp`: Lowered R2/3 SNR threshold from 20→15 for good fading.
  Updated bootstrap cap from SNR≥24 to SNR≥18 for R2/3.

**Why it works:**
CPE correction rotates the entire channel_estimate by the common phase drift estimated from pilots
each symbol. DQPSK differential decoding is unaffected because both eq[n] and eq[n-1] use the
same CPE-corrected H — the common phase cancels in diff = eq[n] × conj(eq[n-1]). The residual
(CPE change between consecutive symbols) is ~5° at 0.5 Hz Doppler, well within DQPSK's 45° margin.
The real benefit is better MMSE equalization (H tracks actual channel phase → less noise amplification).

**Test verification:**
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test` → PASS, 0 retx
- `./build/cli_simulator --snr 15 --fading good --rate r2_3 --test` → 10/10 seeds PASS, avg 1.5 retx
- `./build/cli_simulator --snr 15 --fading moderate --rate r1_4 --test` → 5/5 seeds PASS, avg 1.4 retx
- `./build/cli_simulator --snr 15 --fading moderate --rate r1_2 --test` → 5/5 seeds PASS, avg 2.4 retx
- `./build/cli_simulator --snr 20 --fading good --rate r2_3 --test` → PASS, 0 retx (no regression)
- `./build/cli_simulator --snr 15 --rate r1_4 --test` → PASS, 0 retx (AWGN no regression)

---

## 2026-03-01: Add OFDM_NARROW 500 Hz narrowband mode

**What was added:**
New 500 Hz narrowband OFDM mode (OFDM_NARROW) for reliable operation at much lower SNR than wideband.
Provides ~7.5 dB noise bandwidth advantage, enabling communication at SNR 5-10 dB where wideband fails.

**Key parameters:**
- FFT=2048, 21 carriers, 23.4 Hz bin spacing, 492 Hz occupied bandwidth
- Narrowband chirp: 1250-1750 Hz sweep, 1000ms duration
- Narrowband MC-DPSK handshake: 4 carriers @ 1300-1700 Hz
- Symbol duration: 46.7ms (2240 samples), CP=192 samples (MEDIUM)
- ARQ: window=1 (stop-and-wait), timeout ~7.16s
- Pilots: 0 for R1/4 (21 data carriers), 3 for R1/2+ (18 data carriers)

**Files changed:**
- `include/ultra/types.hpp` - BandwidthMode enum, chirp fields in ModemConfig, narrowband presets
- `src/protocol/frame_v2.hpp/.cpp` - WaveformMode::OFDM_NARROW (0x06), isOFDMMode() helper
- `src/psk/multi_carrier_dpsk.hpp` - mc_dpsk_presets::narrowband() (4 carriers, 1300-1700 Hz)
- `src/waveform/ofdm_chirp_waveform.hpp/.cpp` - Config-driven chirp parameters, mode_ field
- `src/waveform/waveform_factory.hpp/.cpp` - OFDM_NARROW creation, createNarrowbandMCDPSK()
- `src/protocol/waveform_selection.hpp` - SNR 5-10 recommends OFDM_NARROW
- `src/gui/modem/streaming_decoder.cpp` - Dual-listen (wideband + narrowband chirps), narrowband LTS thresholds
- `src/gui/modem/streaming_encoder.hpp/.cpp` - narrowband_control_ flag, narrowband MC-DPSK persistence
- `src/gui/modem/modem_engine.cpp` - bandwidth_mode_ propagation, OFDM_NARROW in mode checks
- `src/protocol/connection.cpp/.cpp` - isOFDMMode() throughout, OFDM_NARROW timing
- `tools/cli_simulator.cpp` - --waveform ofdm_narrow, dual-listen, extended narrowband timeouts
- `src/main.cpp`, `src/gui/app.cpp` - CLI and GUI support

**Key design decisions:**
1. Dual-listen: RX always listens for both wideband and narrowband chirps when idle
2. Narrowband chirp auto-identifies the mode — no manual pre-agreement needed
3. narrowband_control_ flag persists across encoder mode switches during handshake
4. LTS threshold lowered to 0.50 for narrowband (21 carriers produce ~0.71 correlation vs 59-carrier ~0.95)
5. Legacy (wide-only) stations won't detect narrowband chirps → caller sees normal PING timeout

**Verification:**
```
# AWGN
./build/cli_simulator --snr 8 --waveform ofdm_narrow --rate r1_4 --test
# → TEST PASSED: 100% frame success, 0 retransmissions

# Good fading
./build/cli_simulator --snr 8 --fading good --waveform ofdm_narrow --rate r1_4 --test
# → TEST PASSED: 100% RX frame success, 92.9% TX (ACK loss), all 7 messages delivered via ARQ

# Wideband regression
./build/cli_simulator --snr 15 --fading good --rate r1_4 --test
# → TEST PASSED: 100% frame success, 0 retransmissions (no regression)
```

**Performance:**
| Condition | Rate | Frame Success | Throughput |
|-----------|------|--------------|------------|
| AWGN SNR=8 | R1/4 | 100% | ~103 bps |
| Good fading SNR=8 | R1/4 | 100% (data), 93% (ACK) | ~60 bps (with retx) |

---

## 2026-02-11: Alpha gate harness + OFDM SR-ARQ window stabilization

**What was broken:**
- Alpha-readiness was not reproducible; no single deterministic command produced a pass/fail release verdict.
- OFDM SR-ARQ in-flight window at 8 caused higher hole pressure and retransmission tails on fading file transfer (notably DQPSK R2/3, 2048B files).

**What was changed:**
- Added deterministic release harness:
  - `scripts/run_alpha_gate.sh`
  - Produces per-seed logs, CSV metrics, and markdown gate summary.
- Added and documented release gate source-of-truth:
  - `docs/ALPHA_RELEASE_GATE.md`
- Added ARQ cause/debug counters to simulator summary:
  - `tools/cli_simulator.cpp`
  - `src/protocol/arq_interface.hpp`
  - `src/protocol/selective_repeat_arq.hpp/.cpp`
- Reduced OFDM SR-ARQ window from 8 to 4 (aligned with 4-frame burst interleaver groups):
  - `src/protocol/connection.cpp`

**Why this works:**
- Window 4 lowers control-path burst pressure (fewer simultaneous outstanding frames), reducing persistent base-hole amplification and timeout tail behavior on fading channels.
- The harness makes release decisions auditable and repeatable, rather than anecdotal.

**Verification:**
```
scripts/run_alpha_gate.sh --seed-start 42 --seed-count 30 --out-dir /tmp/alpha_gate_full_w4
```

Observed gate report:
- `g1_r14_good`: PASS
- `g2_r14_moderate`: PASS
- `g3_r12_good`: PASS
- `g4_r23_good_msg`: PASS
- `g5_r23_good_file`: PASS (avg retransmissions 2.07, p90 3, max 4)

Overall:
- **Alpha gate status: PASS**
- Report: `/tmp/alpha_gate_full_w4/summary.md`

---

## 2026-02-11: Configurable ACK repeat with delayed copy for fading reliability

**What was broken:**
- D8PSK R1/2 on good fading at SNR=20 had ~45% ACK loss rate (BRAVO sent 11 ACKs,
  ALPHA received 6). This caused 8 timeouts and 8 retransmissions (seed 45).
- The old hole-only ACK repeat logic only fired when the SACK bitmap had holes
  (bit0=0, higher bits set). Pure cumulative ACKs (bitmap=0x00) were never repeated,
  leaving them vulnerable to single-frame loss on fading channels.

**Root cause:**
- Control frames (ACKs) use R1/4 coding but D8PSK modulation, making them fragile
  on fading channels. A single lost ACK causes the sender to wait for a full 9s
  timeout before retransmitting.

**Files modified:**
- `src/protocol/selective_repeat_arq.hpp` — Added ACK repeat config fields
  (`ack_repeat_count_`, `ack_repeat_delay_ms_`) and pending repeat state
  (`ack_repeat_pending_`, `ack_repeat_timer_ms_`, `ack_repeats_remaining_`,
  `ack_repeat_data_`). Added public setters `setAckRepeatCount()`, `setAckRepeatDelay()`.
- `src/protocol/selective_repeat_arq.cpp` — Replaced hole-only repeat in `sendSack()`
  with configurable delayed repeat scheduling. Added delayed ACK repeat handling at
  top of `tick()`. Added repeat state cleanup to `reset()`.
- `src/protocol/connection.cpp` — In `enterConnected()`: set repeat=2/80ms for OFDM,
  repeat=1 for MC-DPSK (stop-and-wait, no benefit from repeat).

**How it works:**
- After sending a SACK, if `ack_repeat_count_ > 1`, schedules delayed copies with
  `ack_repeat_delay_ms_` between them (default 80ms for time diversity).
- `tick()` fires the delayed copies via the existing `transmitData()` path.
- 80ms delay provides time diversity against short fading nulls.
- MC-DPSK keeps repeat=1 (stop-and-wait ACK timing is different).

**Test verification:**
```
./build/cli_simulator --snr 15 --fading good --rate r1_4 --test     → PASS, 0 retx (no regression)
./build/cli_simulator --snr 15 --fading moderate --rate r1_2 --test → PASS, 0 retx (no regression)
./build/cli_simulator --snr 20 --fading good --mod d8psk --rate r1_2 --seed 45 --test
  → PASS, timeouts=1 (was 8), retx=2 (was 8)
./build/cli_simulator --snr 10 --fading moderate --test             → PASS, 0 retx (MC-DPSK unaffected)
```

---

## 2026-02-10: SACK bitmap parsing + hole-based fast retransmit

**What was broken:**
- SACK bitmap was built and transmitted by the receiver but never parsed by the sender.
  Lost ACKs caused a full 12s timeout stall before retransmission.
- No fast retransmit mechanism — even when the receiver's bitmap clearly showed which
  frames were missing, the sender waited for timeout on every lost frame.

**Root cause:**
- `handleAckFrame()` only processed the cumulative ACK sequence number, ignoring the
  SACK bitmap byte entirely. The bitmap was dead data on the wire.
- ACK timeout (12s) was set conservatively for worst-case but was excessive for typical
  OFDM burst timing (~6.7s worst case for 8-frame burst + decode + ACK).

**Files modified:**
- `src/protocol/selective_repeat_arq.hpp` — Added `hole_ack_count` and `fast_retransmitted`
  guard fields to TXSlot struct
- `src/protocol/selective_repeat_arq.cpp` — Major rewrite of `handleAckFrame()`:
  - Stale-ACK guard: reject ACKs with seq strictly older than tx_base_seq_ - 1
  - Far-future guard: reject ACKs implausibly ahead of window
  - Positive-only SACK bitmap: only mark frames receiver confirms (1-bits), never
    interpret 0-bits as lost
  - Hole-based fast retransmit: when bitmap shows bit0=0 and higher bits set, immediately
    retransmit base frame (one-shot per gap, guarded by `fast_retransmitted` flag)
  - Reset guard fields when base sequence advances
  - Conditional ACK repeat in `sendSack()`: duplicate ACK only when hole bitmap detected
  - INFO-level logs for bitmap parsing, guard decisions, fast-retransmit triggers
- `src/protocol/connection.cpp` — OFDM ACK timeout reduced from 12000 → 9000ms

**How it works:**
- Positive-only SACK: only 1-bits are processed (safe — never triggers spurious retransmit
  for in-flight frames). Selectively-acked frames allow `advanceTXWindow()` to skip past
  contiguous acked frames when the gap is later filled.
- Hole detection: `bitmap & 0x01 == 0` (base not received) + `bitmap & 0xFE != 0` (higher
  frames received) → base frame is likely lost → fast retransmit immediately.
- Per-slot `fast_retransmitted` flag prevents duplicate fast retransmits for the same gap.
  Guards reset when tx_base_seq_ advances (new window position).
- Conditional ACK repeat: receiver sends ACK twice only when hole bitmap is detected,
  increasing probability the sender sees the SACK info. No blanket duplication.

**Test verification:**
- DQPSK R1/4 good fading SNR=15: PASSED (all messages delivered)
- DQPSK R1/2 good fading SNR=15: PASSED (all messages delivered)
- D8PSK R1/2 good fading SNR=20 (10 seeds): All 10 PASSED, fast retransmit fired on 3/10 seeds
- MC-DPSK moderate fading SNR=10: PASSED (unaffected — window=1, no SACK)

---

## 2026-02-10: Fix coherent pilot/interleaver geometry mismatch

**What was broken:**
- QPSK R1/2 on good fading averaged 86.4% first-attempt frame success (30-seed survey).
- The channel interleaver in both encoder and decoder assumed `pilot_spacing=10` (53 data carriers,
  106 bits/symbol) regardless of modulation, but `OFDMChirpWaveform::configurePilotsForCodeRate()`
  sets `pilot_spacing=5` (47 data carriers, 94 bits/symbol) for QPSK/BPSK coherent modes.
- Since TX and RX were consistently wrong, data decoded — but the interleaver's symbol-boundary
  assumptions were misaligned with physical OFDM symbols, reducing frequency diversity.

**Root cause:**
- Encoder: `createWaveform()` calls `configure(mod, rate)` which updates the waveform's internal
  pilot_spacing, but never synced this back to `ofdm_config_.pilot_spacing`. The `setDataMode()`
  early-return (when mod/rate unchanged) prevented the fix from running via that path.
- Decoder: `setDataMode()` hardcoded a rate-only switch for pilot_spacing, ignoring modulation.

**Files modified:**
- `src/waveform/waveform_interface.hpp` — Added `virtual int getPilotSpacing() const { return 0; }`
- `src/waveform/ofdm_chirp_waveform.hpp` — Override returning `config_.pilot_spacing`
- `src/waveform/ofdm_cox_waveform.hpp` — Override returning `config_.pilot_spacing`
- `src/gui/modem/streaming_encoder.cpp` — Sync pilot_spacing from waveform in `createWaveform()`
  and `setDataMode()` (after `waveform_->configure()`)
- `src/gui/modem/streaming_decoder.cpp` — Query `waveform_->getPilotSpacing()` in `setDataMode()`
  and `getConfig()` instead of hardcoded values

**Test verification:**
- QPSK R1/2 AWGN SNR=20: PASSED (100%, 0 retransmissions)
- DQPSK R1/4 fading SNR=15: PASSED (100%, 0 retransmissions)
- QPSK R1/2 fading SNR=20 (5 seeds 42-46): avg 93.3% first-attempt (up from 86.4%)

---

## 2026-02-09: Burst-level long interleaver for OFDM_CHIRP

**What was added:**
- Burst-level long interleaver that spreads coded bytes across 4-frame groups (~2.8s).
  Coherent QPSK R1/2 on fading channels hits ~78% frame success because deep spectral nulls
  zero out groups of carriers, and frame interleaving only spreads bits within ONE frame (~0.7s).
  With burst interleaving, each CW's bytes are distributed across 4 physical frames — a total
  frame loss means each CW loses only 25% of its bits, within R1/2 LDPC capacity.

**Files created:**
- `src/fec/burst_interleaver.hpp` / `.cpp` — Byte-level row-column block interleaver
  - TX: `interleave()` permutes coded bytes across N frames (flat_pos = N*b + f)
  - RX: `deinterleave()` operates on 8-float byte groups of soft bits

**Files modified:**
- `src/waveform/waveform_interface.hpp` — Added `virtual bool wasBurstInterleaved() const`
- `src/waveform/ofdm_chirp_waveform.hpp/.cpp` — LTS sign-negation marker for burst detection:
  - TX: negate first LTS symbol for burst-interleaved group starts
  - RX: detect via `P_real < 0` in autocorrelation, undo negation before channel estimation
  - Two-flag design: one-shot for `process()`, latched for decoder query
- `src/gui/modem/streaming_encoder.hpp/.cpp` — `encodeBurstLight()` groups frames into 4-frame
  subgroups, applies burst interleaving and LTS negation for group starts
- `src/gui/modem/streaming_decoder.hpp/.cpp` — New `BURST_ACCUMULATING` state machine:
  - `tryDemodulateNextBurstFrame()` with tri-state result (SUCCESS/WAITING/FAILED)
  - `finalizeBurstGroup()` deinterleaves and decodes all 4 logical frames
  - `accumulateBurstFrames()` handles timeout and frame-by-frame accumulation
- `src/gui/modem/modem_engine.hpp` — `setBurstInterleave(bool)` API
- `tools/cli_simulator.cpp` — `--burst-test` mode (3x 600-byte messages), `--no-burst-interleave` flag
- `CMakeLists.txt` — Added `burst_interleaver.cpp` to build

**Design decisions:**
- Only OFDM_CHIRP mode supports burst interleaving (OFDM_COX uses Schmidl-Cox, incompatible marker)
- 4-frame subgroups within window-8 ARQ: 8-frame burst → 2 groups of 4, partial remainders decode individually
- Enabled automatically in connected OFDM_CHIRP mode, disabled on disconnect

**Test verification:**
```
# AWGN regression: 0 retransmissions
./build/cli_simulator --snr 20 --rate r1_2 --mod qpsk --test
# DQPSK R1/4 fading regression: 0 retransmissions
./build/cli_simulator --snr 15 --fading good --rate r1_4 --test
# Burst validation: all 3 large messages delivered, burst groups detected
./build/cli_simulator --snr 20 --fading good --rate r1_2 --mod qpsk --seed 42 --burst-test
# Multi-seed A/B: 11 total retx (burst) vs 13 (no burst) across seeds 42-46
```

---

## 2026-02-09: Coherent QPSK channel tracking for fading channels

**What was broken:**
- Coherent QPSK on fading channels achieved only ~35% frame success (vs DQPSK ~82%).
  Root cause: LTS-derived per-carrier phases become stale as the channel evolves.
  Pilots only provide 6 phase measurements per symbol — insufficient for 53 data carriers
  with independent phase drift from frequency-selective fading.

**What was changed (6 improvements):**

1. **Phase-slope-compensated complex interpolation** (`channel_equalizer.cpp`)
   - Estimate linear phase gradient from LTS (typically ~19°/carrier from timing offset)
   - Remove slope before pilot interpolation, interpolate in de-sloped domain, restore slope
   - Prevents phase aliasing (190° between 10-spaced pilots exceeds 180° Nyquist limit)
   - Differential modes still use magnitude-only interpolation (preserves LTS phases)

2. **CPE (Common Phase Error) correction** (`channel_equalizer.cpp`)
   - Estimate average phase drift across all pilots, apply to all carriers each symbol
   - Replaces unreliable pilot-based CFO tracking which drifted on both AWGN and fading
   - Standard approach used in WiFi 802.11a/g/n receivers

3. **Decision-directed per-carrier phase tracking** (`channel_equalizer.cpp`)
   - After QPSK hard-decision, measure per-carrier phase error
   - Store snapshot corrections, apply in next symbol's updateChannelEstimate() after interpolation
   - Blend factor 0.3 (empirically optimal: 0.15→73.1%, 0.3→74.1%, 0.5→65.6%)
   - Single-snapshot (no accumulation) — IIR accumulation diverges due to positive feedback

4. **Denser pilots for coherent modes** (`ofdm_chirp_waveform.cpp`)
   - QPSK/BPSK: pilot_spacing=5 (12 pilots, 47 data carriers, ~95° inter-pilot phase)
   - Differential: unchanged at spacing=10 (6 pilots, 53 data carriers)
   - 11% throughput cost offset by dramatically better phase interpolation

5. **1-sample sync refinement** (`ofdm_chirp_waveform.cpp`)
   - detectDataSync() coarse search uses 8-sample steps → up to 4 samples off-peak
   - Added ±4 sample refinement with 1-sample steps around coarse peak
   - 4-sample offset causes ~40° phase error at edge carriers — critical for QPSK

6. **Modulation-dependent sync confidence threshold** (`streaming_decoder.cpp`)
   - Coherent modes: 0.88 (reject corr 0.82-0.85 frames that always fail for QPSK)
   - Differential modes: 0.70 (unchanged)
   - Rejected frames trigger ARQ retransmission instead of wasting time on guaranteed failures

**Also fixed:**
- `carrier_noise_var` MMSE formula: `σ²/mmse_denom` instead of `σ²/|H|²` (correct post-eq noise)
- Pilot H uses last training symbol (not average) for phase consistency with data carriers
- Preserved LTS noise_variance estimate (don't overwrite with temporal pilot comparison)
- Disabled pilot-based CFO tracking for all modes (replaced by CPE for coherent)

**Test results (final configuration):**
| Test | Result |
|------|--------|
| DQPSK R1/4 fading SNR=15 | 100% (no regression) |
| QPSK R1/2 AWGN SNR=20 | 100% (0 retransmissions) |
| QPSK R1/2 fading SNR=20 (5 runs) | avg 78% (75, 69, 82, 75, 89) |
| QPSK R1/2 fading SNR=15 | 100% |

**Verification:** `./build/cli_simulator --snr 20 --fading good --rate r1_2 --mod qpsk --test`

---

## 2026-02-08: Enable coherent QPSK for OFDM_CHIRP

**What was broken:**
- OFDM_CHIRP forced differential modulation (DQPSK/DBPSK/D8PSK) only. Coherent QPSK was
  blocked despite all components (modulator, demodulator, soft demapper, equalizer) already
  supporting it. Differential decoding wastes ~3 dB SNR due to noise doubling.

**What was changed:**

1. **Allow QPSK/BPSK modulations** (`src/waveform/ofdm_chirp_waveform.cpp`)
   - Constructor and `configure()`: accept QPSK and BPSK in addition to differential modes
   - `getThroughput()` and `getMinSamplesForCWCount()`: explicit QPSK/BPSK switch cases

2. **CLI support** (`tools/cli_simulator.cpp`, `tools/test_waveform_simple.cpp`)
   - Added `--mod qpsk` and `--mod bpsk` options

3. **Skip carrier_phase_correction for coherent modes** (`src/ofdm/channel_equalizer.cpp`)
   - carrier_phase_correction removes common phase from H but not from rx signal,
     leaving residual e^(jθ) in equalized output — fatal for QPSK, harmless for differential
   - Fix: identity correction for coherent modes (LTS provides accurate H)

4. **Magnitude-only interpolation for all modes** (`src/ofdm/channel_equalizer.cpp`)
   - DFT interpolation from 6 pilots corrupts per-carrier phases for both differential and
     coherent modes. Now all modes use magnitude-only linear interpolation between pilots,
     preserving the accurate LTS-derived phases at data carriers.

5. **Remove timing recovery** (`src/ofdm/channel_equalizer.cpp`)
   - Timing recovery estimated offset from absolute pilot LS phases, which include channel
     phase. This produced spurious timing offsets (up to 4.6 samples on AWGN) that added
     up to 80° phase rotation at edge carriers — fatal for QPSK equalization.
   - Was also disabled for differential modes (fading corrupts the slope).
   - Removed entirely since it was broken for all modes.

**How it works:**
- QPSK uses same 2 bits/carrier as DQPSK — same frame format, interleaving, throughput
- Coherent MMSE equalization: eq = conj(H) × rx / (|H|² + σ²) with LTS-derived H
- Phase-frozen H (magnitude-only tracking) works because LTS phases are accurate for
  the entire frame on AWGN channels
- On fading channels, QPSK performs worse than DQPSK (~35% vs ~82% frame success at
  R1/2 SNR=20 good fading) because LTS phases become stale

**Test verification:**
- QPSK AWGN SNR=20: `./build/cli_simulator --snr 20 --rate r1_2 --mod qpsk --test` → PASS, 0 retransmissions
- QPSK AWGN SNR=15: `./build/cli_simulator --snr 15 --rate r1_2 --mod qpsk --test` → PASS, 0 retransmissions
- DQPSK regression: `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test` → PASS, 0 retransmissions
- QPSK fading SNR=20: `./build/cli_simulator --snr 20 --fading good --rate r1_2 --mod qpsk --test` → PASS (10 retransmissions)

---

## 2026-02-08: DFT-based channel interpolation + magnitude-only pilot tracking

**What was broken:**
- Linear interpolation between 6 pilots across 59 carriers produced suboptimal H estimates
  at data carriers far from pilots, especially on frequency-selective fading channels
- For differential modes (DQPSK, DBPSK, D8PSK), `updateChannelEstimate()` was completely
  skipped — H was frozen from LTS for the entire frame (~700ms). On fading channels, |H|
  drifts, causing stale MMSE scaling and incorrect LLR confidence

**What was changed:**

1. **DFT-based interpolation** (`src/ofdm/channel_equalizer.cpp`)
   - Replaced linear interpolation with IDFT→window→DFT approach
   - Builds N-point H from pilot LS estimates + linear fill
   - IDFT to CIR, window to L=5 taps (±1.8ms delay spread coverage), DFT back
   - Exploits finite HF channel delay spread for noise suppression
   - Used for coherent modes during data symbols and for all modes during LTS

2. **Magnitude-only pilot tracking for differential** (`src/ofdm/channel_equalizer.cpp`)
   - Enabled `updateChannelEstimate()` for differential modes (was skipped entirely)
   - Pilot H: update |H| via alpha=0.5 smoothing, keep phase frozen from LTS
   - Data carriers: linearly interpolate MAGNITUDES ONLY from pilots, preserve existing phases
   - Skip DFT interpolation for differential (would corrupt phase relationships)
   - Guard CFO estimation and timing recovery with `!is_differential` (fading-induced
     phase changes get misattributed to CFO on fading channels)
   - Guard noise_variance updates with `!is_differential` (preserve LTS-based estimate)

**Why it works:**
- DFT interpolation: noise suppression from CIR windowing produces smoother, more accurate
  H estimates. The HF channel's finite delay spread means high-delay CIR taps are pure noise.
- Magnitude tracking: MMSE equalization `eq = rx × conj(H) / (|H|² + σ²)` needs correct |H|
  for amplitude scaling. Phase errors cancel in differential decoding (diff = eq[n] × conj(eq[n-1]))
  but magnitude errors affect LLR confidence.
- Phase must NOT be updated for differential because the decode relies on phase DIFFERENCES
  between consecutive equalized symbols — changing H phase between symbols introduces
  artificial differential phase errors.

**Test verification:**
- R1/4 AWGN SNR=15: 100%, 0 retx (no regression)
- R1/4 good fading SNR=15: 100%, 0 retx (no regression)
- R1/2 AWGN SNR=20: 100%, 0 retx (no regression)
- R1/2 good fading SNR=20 (seeds 42-46): avg 2.0 retx (was 3.2 baseline — 37.5% reduction)

## 2026-02-08: Per-carrier adaptive LLR scaling

**What was broken:**
- When fading was detected, a **global** scale factor was applied to ALL carriers equally:
  `ce_error_margin *= (1 + 10 × fading_index²)`. This reduced LLR confidence on good carriers
  too, wasting LDPC correction capacity. On frequency-selective fades, some carriers are fine
  while others are deeply faded — the global scale couldn't distinguish between them.

**What was changed:**

1. **Per-carrier |eq| magnitude tracking** (`src/ofdm/demodulator.cpp`)
   - Track EMA of `|equalized[i]|` per carrier across symbols within a frame
   - Track EMA of `(|eq| - ema)²` per carrier (magnitude variance)
   - First symbol initializes EMA; subsequent symbols update with α=0.3

2. **Per-carrier noise inflation** (`src/ofdm/demodulator.cpp`)
   - Replaced global `fading_scale` block with per-carrier scaling in the LLR loop
   - `norm_var = carrier_eq_mag_var[i] / (carrier_eq_mag_ema[i]² + ε)`
   - `nv *= (1 + K × norm_var)` where K=10 (CARRIER_ADAPTIVE_K constant)
   - Applied in both `demodulateSymbol()` and `demodulateD8PSKTwoPass()` pass-2 loop

3. **State management** (`src/ofdm/demodulator_impl.hpp`, `demodulator_constants.hpp`)
   - Added `carrier_eq_mag_ema_` and `carrier_eq_mag_var_` vectors to Impl
   - Added `CARRIER_ADAPTIVE_K = 10.0f` constant
   - Cleared in `processPresynced()`, `reset()`, and all Schmidl-Cox state transitions

**How it works:**
- Stable carrier: low variance → `norm_var ≈ 0` → no noise inflation → full LLR confidence
- Fading carrier: high variance → `norm_var > 0` → inflated noise → LDPC knows not to trust it
- AWGN: all carriers stable → zero variance → no scaling whatsoever (zero regression)

**Test verification:**
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test` → PASS, 0 retransmissions
- `./build/cli_simulator --snr 20 --fading good --rate r1_2 --test` → PASS, all messages delivered
- `./build/cli_simulator --snr 15 --rate r1_4 --test` → PASS, 0 retransmissions, perfect LLRs

---

## 2026-02-08: Frequency-domain interleaving for OFDM

**What was broken:**
- Adjacent coded bits mapped to adjacent carriers. When a cluster of carriers fades
  together (common on HF), all bits in that cluster are wrong. LDPC can't fix a burst
  of confident wrong bits. This was the main cause of R1/2 retransmissions on fading channels.

**What was changed:**

1. **Coprime-step carrier permutation** (`src/ofdm/modulator.cpp`, `src/ofdm/demodulator.cpp`)
   - TX: `perm[c] = (c * 23) mod N` maps logical carrier c to physical carrier perm[c]
   - RX: `inv_perm[p] = c` where `(c * 23) mod N = p` reverses the mapping on soft bits
   - Step=23 ensures adjacent logical carriers map ~23 physical carriers apart
   - Applied in `modulate()` (TX) and `demodulateSymbol()` + `demodulateD8PSKTwoPass()` (RX)
   - Permutation is fixed across all OFDM symbols — differential encoding chains are coherent

2. **Public API** (`include/ultra/ofdm.hpp`, waveform files)
   - `setFrequencyInterleave(bool)` on OFDMModulator and OFDMDemodulator
   - Forwarded through OFDMChirpWaveform, OFDMNvisWaveform, IWaveform interface
   - StreamingEncoder/StreamingDecoder forward setting and persist across waveform recreation

3. **CLI flag** (`tools/cli_simulator.cpp`)
   - `--no-freq-interleave` / `--nfi` to disable, `--freq-interleave` / `--fi` to enable
   - Default: ON

**How it works:**
- Example: Physical carriers 20-25 fading → logical positions {1, 8, 17, 24, 31, 47}
  (scattered across 53 carriers). LDPC sees scattered errors, not burst errors.
- Works correctly with differential encoding because permutation is fixed per-symbol.
  TX state `dbpsk_prev_symbols[c]` tracks logical carrier c; physical carrier `perm[c]`
  always carries the same logical chain.

**Test verification:**
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test` → PASS, 0 retransmissions
- `./build/cli_simulator --snr 20 --fading good --rate r1_2 --test` → PASS, all messages delivered
- `./build/cli_simulator --snr 15 --rate r1_4 --test` → PASS, AWGN 0 retransmissions
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --no-freq-interleave --test` → PASS

---

## 2026-02-08: LDPC false positive recovery via CRC-guided bit-flip search

**What was broken:**
- At SNR=20 with good fading, R1/4 averaged ~1.0 retransmissions per test run.
- Root cause: LDPC min-sum decoder occasionally converges to a wrong-but-valid codeword
  (syndrome passes but information bits are wrong). Frame-level CRC catches this, but the
  frame is discarded and retransmitted.
- These "false positives" account for most retransmissions at SNR=20 (genuine CW failures
  from deep fades cause the remainder).

**What was changed:**

1. **CRC-guided bit-flip recovery** (`src/protocol/frame_v2.cpp`)
   - Two recovery cases: Case 1 (header CRC error in CW0) and Case 2 (frame CRC error)
   - Case 1: Direct magic + header CRC check on CW0 without parseHeader (avoids logging
     spam from thousands of failed trials). 1-bit and 2-bit brute force in CW0.
   - Case 2: CRC delta table — precompute `delta[p] = CRC(data^e_p) XOR CRC(data)` for
     each data bit position p. Exploits CRC linearity for efficient search:
     - 1-bit: O(n) — check if delta[p] == syndrome
     - 2-bit: O(n) with hash map — for each p1, look up `syndrome ^ delta[p1]`
     - 3-bit: O(n²) with hash map — for each (p1,p2), look up `syndrome ^ delta[p1] ^ delta[p2]`
   - Suspect-augmented search for 4-6 bit errors: identifies LDPC-flipped info bits
     (bits where decoder output disagrees with channel LLR sign) as suspects, searches
     C(K,4) through C(K,6) subsets among K=30 suspects
   - Hybrid 2+2 search: 2 suspect bits + 2 arbitrary bits via delta_map

2. **Fallback LDPC re-decode** with different min-sum factors {0.75, 0.625, 0.5, 0.875}
   after CRC-guided search fails.

3. **Added `#include <unordered_map>`** for delta_map hash table.

**Recovery effectiveness (observed over 20-run batch):**
- 87.5% of detected false positives recovered (14/16)
- Most recovered via 1-bit or 2-bit fix (specific trapping set patterns)
- Unrecoverable FPs have 7+ bit errors (beyond practical search space)
- Remaining retransmissions from genuine CW decode failures during deep fades

**Test verification:**
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test` — PASS, 0 retransmissions
- `./build/cli_simulator --snr 20 --rate r1_4 --test` — PASS (AWGN), 0 retransmissions
- SNR=20 good fading: reduced from avg ~1.0 to ~0.5 retransmissions per run
  (high variance due to non-deterministic fading; ~70-93% of runs achieve 0 retransmissions)

---

## 2026-02-07: Fix DISCONNECT decode failure on fading + false LTS detection

**What was broken:**
- At SNR=20 with good fading, R1/4 showed 12+ retransmissions while SNR=15 showed 0.
- Two distinct failure types:
  1. DISCONNECT always failed (all 4 CWs fail, |llr|=3.3-4.2) — BRAVO never saw ALPHA's DISCONNECT
  2. False LTS detection (corr=0.63 on data, threshold 0.50) — phantom frame trigger, all CWs fail

**What was changed:**

1. **Route OFDM DISCONNECT through `encodeFixedFrame()` for frame interleaving**
   (`src/gui/modem/streaming_encoder.cpp`)
   - DISCONNECT was encoded via `encodeFrameWithLDPC()` (sequential, no interleaving) — each CW's
     bits map to consecutive OFDM symbols, so temporal fading wipes entire CWs
   - Changed `is_variable_cw_frame` logic: `isControlFrame()` → true (1-CW ACK path),
     `isConnectFrame()` → false (4-CW interleaved path via `encodeFixedFrame()`)
   - Decoder needs no change: "try both" strategy in `decodeFrame()` falls through to
     `try_frame_interleave = true` and succeeds
   - `ConnectFrame::serialize()` already hardcodes `total_cw=4`, matching `encodeFixedFrame()` expectations

2. **Raise LTS confidence threshold from 0.50 to 0.70**
   (`src/gui/modem/streaming_decoder.cpp`)
   - Data autocorrelation can produce spurious peaks up to 0.63, triggering false frame detection
   - Real LTS correlation is always >0.81 even on moderate fading
   - Changed `LIGHT_SYNC_MIN_CONFIDENCE` from 0.50f to 0.70f

**Test verification:**
- `./build/cli_simulator --snr 20 --fading good --rate r1_4 --test` — PASS, retransmissions 12+ → 3
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test` — PASS, 0 retransmissions (regression OK)
- `./build/cli_simulator --snr 10 --fading moderate --test` — PASS, MC-DPSK unaffected
- `./build/cli_simulator --snr 20 --fading good --rate r1_2 --test` — PASS, DISCONNECT decoded 4/4 CWs

---

## 2026-02-06: Restructure variable-CW frame handling — fix DISCONNECT at R1/2

**What was broken:**
- DISCONNECT frame decode failed at R1/2 OFDM. BRAVO never saw ALPHA's DISCONNECT, connection timed out.
- Three root causes:
  1. `ConnectFrame::serialize()` hardcodes `total_cw=4` (frame_v2.cpp:755), but actual LDPC encoding
     produces 2 CWs at R1/2 and 3 CWs at R1/4 for 44-byte ConnectFrames.
  2. No way for decoder to compute exact buffer size for N CWs — `getMinSamplesForCWCount(int)` was
     private in OFDMChirpWaveform, inaccessible to decoder.
  3. OFDM decoder always processed full 4-CW buffer (31104 samples). For 2-CW DISCONNECT (17280 samples),
     the extra 13824 samples of noise degraded LLR quality → decode failure.

**What was changed:**

1. **Promoted `getMinSamplesForCWCount(int)` to IWaveform interface** with default implementation.
   OFDMChirpWaveform overrides with exact calculation. Added override to MCDPSKWaveform with
   proper `training + ref + N × data_per_cw` calculation.

2. **Encoder patches `total_cw` for OFDM ConnectFrames**: After LDPC encoding, compares actual CW
   count with header's total_cw. If different, patches byte 12 (total_cw), recalculates header CRC
   (bytes 15-16) and frame CRC (last 2 bytes), then re-encodes.

3. **Decoder restructured with CW0 peek-first strategy**:
   - MC-DPSK: Always starts with 1-CW buffer, peeks CW0 header for total_cw, waits for exact size.
   - Connected OFDM: Starts with full 4-CW buffer (data frames use frame interleaving, CW0 can't be
     decoded independently). If 4-CW decode fails, falls back to small-frame recovery: 1-CW peek →
     read total_cw → reprocess with exact `getMinSamplesForCWCount(N)` size.
   - Disconnected OFDM: 1-CW initial buffer for control frame detection.

4. **Exact consumed-sample calculation**: Non-data frames advance by `getMinSamplesForCWCount(actual_cw)`
   instead of full 4-CW frame size. E.g., 2-CW DISCONNECT advances 17280 samples, not 31104.

5. **`checkIfReadyToDecode()` uses exact calculations**: Replaced crude `(min_frame * 9) / 10`
   arithmetic with three-way logic based on pending_total_cw, connected OFDM, or MC-DPSK/disconnected.

**Files changed:**
- `src/waveform/waveform_interface.hpp`: Added virtual `getMinSamplesForCWCount(int)` to IWaveform
- `src/waveform/ofdm_chirp_waveform.hpp`: Moved method from private to public with override
- `src/waveform/mc_dpsk_waveform.hpp`: Added `getMinSamplesForCWCount` override declaration
- `src/waveform/mc_dpsk_waveform.cpp`: Added implementation with proper sample calculation
- `src/gui/modem/streaming_encoder.cpp`: Added total_cw patching for OFDM ConnectFrames
- `src/gui/modem/streaming_decoder.cpp`: Restructured `checkIfReadyToDecode()` and `decodeCurrentFrame()`

**Test verification:**
- R1/2 AWGN SNR=20: PASSED, 0 retransmissions, DISCONNECT decoded as 2/2 CWs
- R1/4 good fading SNR=15 regression: PASSED, 0 retransmissions, 100% CW success
- MC-DPSK moderate fading SNR=10 regression: PASSED, 0 retransmissions, 100% success
- R1/2 good fading SNR=20: PASSED, 8 retransmissions (all 7 messages delivered)

---

## 2026-02-06: OFDM throughput improvements — 1-CW ACK + R1/2 rate selection

**What was changed:**

1. **1-CW OFDM ACK frames:** OFDM control frames (ACK, NACK, MODE_CHANGE, etc.) are only 20 bytes
   = 1 codeword. Previously encoded as 4-CW fixed frames with frame interleaving (25 data symbols,
   0.648s). Now encoded as 1-CW frames without interleaving (7 data symbols, 0.216s). Data frames
   still use full 4-CW frame interleaving for fading protection.

2. **R1/2 rate selection enabled:** `selectOFDMCodeRate()` was hardcoded to R1/4. Now selects R1/2
   when channel conditions allow:
   - AWGN (fading < 0.15) at SNR >= 15: R1/2
   - Good fading (< 0.65) at SNR >= 20: R1/2
   - Everything else: R1/4

**Files changed:**
- `src/gui/modem/streaming_encoder.cpp`: Control frames use `encodeFrameWithLDPC()` (1 CW)
  instead of `encodeFixedFrame()` (4 CWs). Detection via `v2::isControlFrame()`.
- `src/protocol/waveform_selection.hpp`: `selectOFDMCodeRate()` SNR/fading thresholds for R1/2.
  `recommendWaveformAndRate()` uses dynamic rate selection instead of hardcoded R1/4.
- `src/waveform/ofdm_chirp_waveform.cpp`: Added `getMinSamplesForControlFrame()` and shared
  `getMinSamplesForCWCount()` helper.
- `src/waveform/ofdm_chirp_waveform.hpp`: Declared new methods.
- `src/waveform/waveform_interface.hpp`: Added virtual `getMinSamplesForControlFrame()` to IWaveform.

**Decoder:** Existing "try CW0 non-interleaved" path in streaming_decoder.cpp already handles
1-CW frames — no decoder changes needed. The decoder waits for full 4-CW sample threshold,
but 1-CW frames arrive faster (shorter TX), so the decoder naturally processes them sooner.

**Impact:**
- ACK time: 0.648s → 0.216s (3× faster)
- R1/2 doubles payload per frame: 61 → 141 bytes
- Combined: ~2.5× throughput improvement on good channels

**Test verification:**
- R1/2 AWGN SNR=20: PASSED, 0 retransmissions
- R1/2 good fading SNR=20: PASSED, 16 retransmissions (all delivered)
- R1/4 good fading SNR=15 regression: PASSED, 0 retransmissions, 100% CW success

---

## 2026-02-06: Fix three bugs found during 1-CW ACK + R1/2 verification

### Bug 1: detectDataSync() false peaks from LDPC zero-padding

**What was broken:**
- 1-CW ACK frames failed to decode. detectDataSync() locked onto wrong sample position.
- Root cause: LDPC zero-padding in 1-CW frames (20 bytes payload + 20 bytes zero pad → bytes 20-40
  all zeros → DQPSK 0° phase change → identical adjacent data symbols). Schmidl-Cox autocorrelation
  found ~1.0 for both real LTS pair AND false data1-data2 pair. Since detectDataSync() picks the
  BEST peak, it chose the later (wrong) data peak over the earlier (correct) LTS peak.

**What was changed:**
- `src/waveform/ofdm_chirp_waveform.cpp`: Added early exit in detectDataSync() when correlation
  exceeds 0.95. The real LTS is always the FIRST high-confidence peak in the search window.
  False peaks from identical data symbols appear later and are now never reached.

### Bug 2: 1-CW frame sample overconsumption in decoder

**What was broken:**
- After correctly decoding a 1-CW ACK, subsequent data frames failed with all 4 CWs failing.
- Root cause: decodeCurrentFrame() consumed 31104 samples (4-CW frame size) regardless of actual
  frame size. A 1-CW ACK is only 10368 samples (2 LTS + 7 data symbols). The extra 20736 samples
  consumed belonged to the next data frame, causing false sync detection at correlation ~0.67.

**What was changed:**
- `src/gui/modem/streaming_decoder.cpp`: After decoding a 1-CW control frame, advance by
  `getMinSamplesForControlFrame()` instead of full frame_buffer size. Also skip burst continuation
  for 1-CW control frames (ACKs are standalone, not part of a data burst).

### Bug 3: ARQ advanceRXWindow delivers frames with wrong MORE_FRAG flag

**What was broken:**
- Multi-frame messages occasionally failed to reassemble after retransmission filled a gap.
  Message 7 of 7 would never complete despite all frames being received.
- Root cause: When `advanceRXWindow()` delivered multiple buffered frames in sequence (e.g.,
  seq=8,9,10 after retransmission fills gap at seq=8), `lastRxHadMoreData()` returned the
  MORE_FRAG flag from the LAST ARRIVED frame (the gap-filler, which had MORE_FRAG=true), not
  from the frame being delivered. So seq=10 (last fragment, MORE_FRAG=false) was treated as an
  intermediate fragment, preventing message completion.

**What was changed:**
- `src/protocol/selective_repeat_arq.cpp`: In `advanceRXWindow()`, update `last_rx_flags_` and
  `last_rx_more_data_` from each slot's stored flags BEFORE calling the delivery callback.
  Each RX slot already stored the correct per-frame flags from `handleDataFrame()`.

**Test verification:**
- R1/4 good fading SNR=15: PASSED, 7/7 messages, 0 retransmissions
- R1/2 AWGN SNR=20: PASSED, 7/7 messages, 1 retransmission (marginal CW[1])
- R1/2 good fading SNR=15: PASSED, 7/7 messages, 1 retransmission

---

## 2026-02-06: Diagnostic cleanup + file transfer test

**Diagnostic cleanup:**
- `src/ofdm/demodulator.cpp`: Removed per-carrier DQPSK diagnostic logging that fired for every
  symbol (root cause: `snr_symbol_count` only incremented in two-pass paths, stayed at 2 in
  single-pass, so `< 6` condition was always true). Removed entry/histogram diagnostics.
  Changed remaining diagnostics to DEBUG level.
- `src/ofdm/channel_equalizer.cpp`: Changed LTS carrier phase log from INFO to DEBUG.

**File transfer test:**
- `tools/cli_simulator.cpp`: Made DISCONNECT timeout non-fatal in `runFileTransferTest()` (matching
  `runProtocolTest()` behavior). File data transfer is the real test; disconnect is best-effort.
- R1/2 AWGN SNR=20 file transfer: PASSED (256 bytes, 0 retransmissions, ~994 bps)
- R1/2 good fading SNR=20 file transfer: PASSED (256 bytes, 0 retransmissions)

---

## 2026-02-06: Fix MC-DPSK at low SNR (two issues)

**What was broken:**
- MC-DPSK failed at SNR=5 AWGN — CW0 decode failed every time. PING never detected,
  connection timed out after 3 retries.
- Two independent root causes:

1. **PING detection used fixed RMS threshold (0.04):** PING frames are chirp-only (no data).
   Detection checks if data region RMS < 0.04. At SNR=5, noise RMS is ~0.056, exceeding the
   threshold. Decoder mistakenly tried to LDPC-decode noise, producing garbage.

2. **MC-DPSK soft bits used fixed confidence scaling:** `confidence = mag × num_carriers × 4`
   produced LLRs of magnitude ~20-32, hard-clipped to ±10. At low SNR, wrong bits also clipped
   to ±10, making them indistinguishable from correct bits. LDPC couldn't converge.

**What was changed:**
- `src/gui/modem/streaming_decoder.cpp`: Replaced fixed PING RMS threshold with **relative
  ratio** (data_RMS / training_RMS). PING has ratio < 0.5 at any SNR; DATA frames have ratio
  ~0.9-1.2. Works across all SNR levels since it's a relative measurement.

- `src/psk/multi_carrier_dpsk.hpp`: Restructured `demodulateSoft()` into two passes:
  - **Pass 1**: Demodulate, cache differential phases, estimate phase noise variance from
    nearest-constellation-point errors.
  - **Pass 2**: Compute LLRs using SNR-proportional scale: `2 × sqrt(1/phase_noise_var)`,
    capped at 20.0, floored via phase_noise_var minimum of 0.01.
  - Raised clip limit from ±10 to ±20 to match OFDM's MAX_LLR.

**How it works:**
- Phase noise variance is naturally proportional to 1/SNR for differential modulation.
  At SNR=5: var≈0.03, scale≈12. At SNR=20: var≈0.01, scale=20 (cap). This produces
  appropriately soft LLRs at low SNR that LDPC can distinguish and correct.
- Relative PING threshold: training region has chirp signal, data region has only noise for
  PING. The ratio is SNR-independent since both regions see the same noise floor.

**Test verification:**
- `./build/cli_simulator --snr 5 --rate r1_4 --test`: PASSED (100% CW, 0 retransmissions)
- `./build/cli_simulator --snr 0 --fading moderate --rate r1_4 --test`: PASSED (90% CW, 1 retransmission, all 7 messages)
- `./build/cli_simulator --snr 10 --fading moderate --rate r1_4 --test`: PASSED (100% CW, 0 retransmissions)
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test`: PASSED (100% CW, 0 retransmissions — OFDM regression)

---

## 2026-02-06: Fix burst block detection in detectDataSync

**What was broken:**
- Burst blocks 2-4 failed to decode (corr=0.76→0.65 degrading). File transfer timed out.
- Root cause: `detectDataSync()` energy gate was designed for silence→signal transitions.
  In burst continuation, the search buffer starts with previous block's data (noise_floor=0.21),
  causing the energy threshold to never be exceeded. The 4-symbol search window from signal_start=0
  was too narrow to reach the actual LTS training at offset ~9600 in the search buffer.

**What was changed:**
- `src/waveform/ofdm_chirp_waveform.cpp`: Modified `detectDataSync()` to detect when the buffer
  starts with signal (noise_floor >= 0.05) vs silence (noise_floor < 0.05).
  - Silence: Use existing energy gate + narrow search window (skip quiet region efficiently)
  - Signal present: Skip energy gate, search entire buffer. LTS autocorrelation (~0.99) is
    distinctive enough to stand out from data autocorrelation (~0.2-0.4).
- `src/gui/modem/streaming_decoder.cpp`: Removed unused `LEAD_IN_SAMPLES` constant.

**How it works:**
- The LTS training has two identical OFDM symbols, giving Schmidl-Cox autocorrelation ~0.99.
  Random OFDM data gives ~0.2-0.4. This contrast is sufficient for detection without energy gating.
- Each burst block still has its own 2 LTS training symbols for per-block channel estimation.

**Test verification:**
- `./build/cli_simulator --snr 20 --rate r1_4 --test`: PASSED (0 retransmissions)
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test`: PASSED (0 retransmissions)
- `./build/cli_simulator --snr 20 --rate r1_4 --file 512`: PASSED (512 bytes transferred, verified)

---

## 2026-02-06: OFDM burst mode for multi-frame transmission

**What was broken:**
- OFDM file transfer and long message fragmentation sent each frame with its own LTS preamble.
  With ARQ window=4, frames 3-4 could fail because the decoder returned to SEARCHING state
  and couldn't re-acquire LTS fast enough (overlapping search windows).

**What was changed:**
- `src/gui/modem/streaming_encoder.hpp/.cpp`: Added `encodeBurstLight()` — encodes multiple
  frames as a single burst with one LTS preamble. First frame uses `encodeFrameLight()`,
  subsequent frames get training symbols + modulated data appended directly.
- `src/gui/modem/streaming_decoder.hpp/.cpp`: Added burst continuation logic in
  `decodeCurrentFrame()`. After successful decode in connected OFDM mode, checks for energy
  at the expected next block position. If energy present, processes as continuation block
  via `waveform_->process()` with CFO tracking. Loops for up to 8 continuation blocks.
- `src/protocol/connection.hpp/.cpp`: Added burst TX buffering. `sendNextFileChunk()` and
  `sendNextFragment()` accumulate frames when in OFDM mode, then flush as a single burst.
  `TransmitBurstCallback` added for the burst TX path. ACK timeout increased 5s→8s for
  burst RTT.
- `src/protocol/protocol_engine.hpp/.cpp`: Passthrough for `setTransmitBurstCallback()`.
- `src/gui/modem/modem_engine.hpp/.cpp`: Added `transmitBurst()` method.
- `src/gui/app.cpp`: Wired burst callbacks for main and virtual station protocols.
- `tools/cli_simulator.cpp`: Added `transmitBurst()` and burst callback in `SimulatedStation`.

**How it works:**
- TX: Burst format is `[LTS][train+data_0][train+data_1]...[train+data_N]`
- RX: Burst continuation checks energy at known position after each block decode.
  In synchronous simulator, continuation rarely fires (audio not yet buffered), but
  blocks are decoded via normal LTS re-sync since each block has 2 LTS training symbols.
  In real-time GUI mode, burst continuation provides direct block-to-block decode.
- OFDM-only: all burst logic gated on `is_ofdm` checks. MC-DPSK path unaffected.
- ARQ unchanged: per-frame seq nums, SACK bitmap, retransmission all preserved.

**Test verification:**
- `./build/cli_simulator --snr 20 --rate r1_4 --test`: PASSED (0 retransmissions)
- `./build/cli_simulator --snr 15 --fading good --rate r1_4 --test`: PASSED (3 retransmissions)
- `./build/cli_simulator --snr 20 --rate r1_4 --file 1024`: PASSED (1024 bytes transferred, verified)

---

## 2026-02-05: Long message fragmentation for OFDM

**What was broken:**
- Long text messages (>61 bytes at R1/4) were silently truncated by `encodeFixedFrame()` to fit
  the 4-CW OFDM frame. The receiver got truncated data, couldn't parse the protocol frame
  (payload_len field says 233 bytes but only 63 bytes arrived), and never sent an ACK.
  The sender retransmitted forever.

**What was changed:**
- `src/protocol/connection.hpp`:
  - Added `pending_tx_fragments_`, `next_fragment_idx_`, `rx_reassembly_buffer_` members
  - Added `sendNextFragment()` method declaration
- `src/protocol/connection.cpp`:
  - `sendMessage()`: Checks `getFixedFramePayloadCapacity(data_code_rate_)`, fragments if needed
  - `sendNextFragment()`: Drip-feeds fragments with MORE_FRAG flag via ARQ window
  - `sendComplete` callback: Handles fragment ACKs, sends more or fires on_message_sent_
  - `enterDisconnected()` / `reset()`: Clear fragment buffers
- `src/protocol/connection_handlers.cpp`:
  - `handleDataPayload()`: Accumulates fragments when `more_data=true`, delivers complete
    reassembled message when final fragment arrives (no MORE_FRAG)
- `tools/cli_simulator.cpp`:
  - Added 2 long test messages (132b, 126b) to the test suite alongside the 5 short ones

**How it works:**
- TX: `sendMessage()` splits into chunks of `getFixedFramePayloadCapacity()` bytes, queues them,
  and feeds them through ARQ with `MORE_FRAG` flag on all but the last chunk
- RX: `handleDataPayload()` accumulates payloads with `more_data=true` into `rx_reassembly_buffer_`,
  then delivers the complete message when the final fragment (no flag) arrives
- Single-frame messages are unchanged (backwards compatible)

**Test verification:**
```
./build/cli_simulator --snr 15 --fading good --rate r1_4 --test
# All 7 messages (5 short + 2 long) delivered correctly
# 132-byte message: 3 fragments, reassembled correctly
# 126-byte message: 3 fragments, reassembled correctly
# TEST PASSED
```

---

## 2026-02-03: Refactor ModemEngine TX to use StreamingEncoder

**What was broken:**
- ModemEngine::transmit() had ~300 lines of inline TX encoding (LDPC, frame interleaving,
  CW patching, waveform creation) that duplicated StreamingEncoder
- Config mismatch bugs between GUI and cli_simulator (pilot settings, CRC, CFO)
- Two divergent TX paths to maintain
- Control frames (ACK/NACK) encoded as 1-CW in GUI but 4-CW in cli_simulator

**What was changed:**
- `src/gui/modem/modem_engine.hpp`:
  - Added `StreamingEncoder` member, removed `encoder_` (fec::CodecPtr),
    `active_tx_waveform_`, `channel_interleaver_`, `ack_4cw_enabled_`,
    `interleaving_enabled_`, `interleaver_bits_per_symbol_`, `frame_interleaving_enabled_`
  - Removed `ensureTxWaveform()`, `updateChannelInterleaver()`, `setInterleavingEnabled()`
  - Added `postProcessTx()` helper
- `src/gui/modem/modem_engine.cpp`:
  - Constructor creates StreamingEncoder instead of encoder_/channel_interleaver_
  - `transmit()` reduced from ~280 lines to ~60 lines: waveform decision + StreamingEncoder delegation
  - `transmitPing()/transmitPong()` delegate to `streaming_encoder_->encodePing()`
  - `transmitTestPattern()/transmitRawOFDM()` use StreamingEncoder
  - Extracted `postProcessTx()` for lead-in, filter, scale, stats
  - Deleted `ensureTxWaveform()` and `updateChannelInterleaver()`
- `src/gui/modem/modem_mode.cpp`:
  - `setWaveformMode()`, `setConnected()`, `setDataMode()` now mirror config to StreamingEncoder
  - `setCodecType()` no longer recreates encoder_ (StreamingEncoder manages its own)
- `CMakeLists.txt`: Added streaming_encoder.cpp to ultra_gui, threaded_simulator, and the then-existing acquisition profiler

**Key behavioral change:**
- OFDM control frames (ACK/NACK) now get 4-CW frame interleaving via StreamingEncoder,
  matching cli_simulator behavior. Should reduce ACK loss on fading channels.

**Test verification:**
```
./build/cli_simulator --snr 20 --test              # AWGN: PASS, 0 retransmissions
./build/cli_simulator --snr 15 --fading good --rate r1_4 --test   # Good fading: PASS, 0 retransmissions
./build/cli_simulator --snr 15 --fading moderate --rate r1_4 --test  # Moderate: PASS, 2 retransmissions (expected)
```

---

## 2026-02-02: Fix Light Sync Timing Errors on Fading Channels (68%→93%)

**What was broken:**
- OFDM R1/4 on moderate fading had ~68% CW success rate instead of expected ~100%
- Frames with low light sync correlation (0.5-0.8) failed completely with random LLR
- All 4 CWs would fail with |llr|_avg ~2.5 (random) instead of ~5-7 (valid)

**Root cause:**
- Light sync (Schmidl-Cox on LTS) uses 0.5 correlation threshold
- On fading channels, multipath can cause timing errors in sync detection
- Low correlation (0.6-0.75) indicates sync found at wrong position
- Wrong timing → wrong channel estimate → complete frame corruption

**Files modified:**
- `src/gui/modem/streaming_decoder.cpp`:
  - Raised LIGHT_SYNC_CONFIDENCE from 0.5 to 0.8
  - Marginal syncs now fall back to full chirp with accurate timing
  - Added CFO drift limit (±1 Hz) when connected to reject multipath-induced false CFO

**How it works:**
- Light sync with corr < 0.8 triggers fallback to chirp sync
- Chirp sync has sub-sample timing accuracy from dual chirp gap measurement
- Full chirp takes ~1.2s longer but gives reliable timing on fading channels

**Test verification:**
```bash
./build/cli_simulator --snr 25 --fading moderate --test
# Before: 68% CW success (48/71)
# After: 93% CW success (130/140 over 3 tests, including 1 test at 100%)
```

---

## 2026-02-02: Fix Two-Pass DQPSK Not Triggering on Fading Channels

**What was broken:**
- Two-pass DQPSK decoding (phase error correction) never triggered on fading channels
- Log showed no "DQPSK two-pass" messages during moderate fading tests
- Moderate fading CW success was ~63% when it should be ~68% with two-pass

**Root cause:**
- `demodulateSymbol()` called `computeFadingIndex()` to decide if two-pass should trigger
- `computeFadingIndex()` computes coefficient of variation from `channel_estimate[]` array
- After sync, `channel_estimate` is reset to unity (all 1.0) at line 814 in demodulator.cpp
- Unity channel estimate has zero variance → `computeFadingIndex()` returns 0
- Two-pass threshold (0.12) was never exceeded because fading index was always 0

**Files modified:**
- `src/ofdm/demodulator.cpp`:
  - Changed from `float fading_index = computeFadingIndex();`
  - To: `float fading_index = last_fading_index;`
  - `last_fading_index` is measured from pilot variance (correct source)
  - Also changed LOG_DEMOD(DEBUG) to LOG_DEMOD(INFO) to see triggering in logs

**How it works:**
- `last_fading_index` is updated during pilot tracking from actual pilot magnitude variance
- This correctly reflects channel fading state (0.12-0.50 on fading channels)
- Two-pass now triggers when fading > 0.12, applying per-carrier phase correction

**Test verification:**
```bash
./build/cli_simulator --snr 25 --fading moderate --test 2>&1 | grep "DQPSK two-pass"
# Expected: Many lines showing "DQPSK two-pass: fading=0.xxx > 0.120, applying correction"
# ✓ TEST PASSED - two-pass triggers, moderate fading CW success improved to ~68%
```

---

## 2026-02-02: Fix CW[0] LDPC Decode Failures in OFDM

**What was broken:**
- OFDM_CHIRP at SNR 20 dB with R1/4 intermittently failed to decode CW[0]
- CW[0] hit 50 iterations (max) and failed while CW[1-3] decoded with 3-5 iterations
- LLR statistics showed low |llr|_avg (~1.0-1.2) instead of expected 3-4 for SNR 20

**Root cause:**
- In `updateChannelEstimate()`, the first symbol fallback path sets `noise_count=1`
- But the noise variance update condition was `if (noise_count > 1)`, which FAILED
- Result: `noise_variance` stayed at hardcoded 0.1f instead of estimated ~0.01
- This compressed LLRs by ~3x, causing borderline decodes that sometimes failed
- CW[0] was more affected because its data has mixed bit polarity (llr_avg≈0)

**Files modified:**
- `src/ofdm/channel_equalizer.cpp`:
  - Changed condition from `noise_count > 1` to `noise_count > 0`
  - Handle single-sample fallback case (noise_count==1) separately
- `src/protocol/frame_v2.cpp`:
  - Added CW decode logging with LLR statistics for debugging

**How it works:**
- First symbol: noise_count=1 (fallback), now updates noise_variance from estimated 15dB SNR
- Subsequent symbols: noise_count=6 (from 6 pilots), updates from temporal comparison
- Correct noise_variance → correct LLR scaling → reliable LDPC decode

**Test verification:**
```bash
./build/cli_simulator --snr 20 -w ofdm_chirp --rate r1_4 --test
# Expected: All frames decode with 4/4 CWs
# ✓ TEST PASSED - all 5 messages transferred, all CW[0] decode OK
```

---

## 2026-02-02: Fix BUG-006 - Re-enable Channel Interleaving

**What was broken:**
- Channel interleaving was completely non-functional - the `--channel-interleave` flag did nothing
- When enabled, CW1 specifically failed to decode while CW0, CW2, CW3 succeeded
- The bug report said interleaving "caused" failures, but actually it wasn't being applied at all

**Root cause:**
- In `encodeFixedFrame()` and `decodeFixedFrame()`, the `use_channel_interleave` parameter was cast to void:
  ```cpp
  (void)use_channel_interleave;  // Disabled due to BUG-006
  ```
- This completely disabled channel interleaving at the protocol level
- The StreamingEncoder/Decoder were properly configured but frame_v2.cpp ignored the setting

**Files modified:**
- `src/protocol/frame_v2.cpp`:
  - `encodeFixedFrame()`: Added ChannelInterleaver creation and interleave call after LDPC encode
  - `decodeFixedFrame()`: Added ChannelInterleaver creation and deinterleave call before LDPC decode
  - Both use consistent `BITS_PER_SYMBOL = 106` (53 data carriers × 2 bits for DQPSK)

**How it works:**
- Channel interleaving spreads consecutive bits across OFDM symbols for fading resistance
- Interleaver is created with (bits_per_symbol=106, total_bits=648) matching LDPC codeword size
- TX: After LDPC encode, interleave coded bits before frame interleaving
- RX: After frame deinterleaving, channel-deinterleave before LDPC decode
- The order is: LDPC encode → channel interleave → frame interleave (TX); reverse for RX

**Test verification:**
```bash
# Clean AWGN with channel interleaving
./build/cli_simulator --snr 20 -w ofdm_chirp --rate r1_4 --channel-interleave --test
# Expected: Shows "Channel interleaving: ENABLED" and all frames decode
# ✓ TEST PASSED - all 5 messages transferred
```

---

## 2026-01-31: Fix MC-DPSK AUTO Rate Bug

**What was broken:**
- When forcing `--waveform mc_dpsk` without `--rate`, the system selected R1/2 instead of R1/4
- The algorithm in `waveform_selection.hpp` specifies MC-DPSK should ALWAYS use R1/4
- Log showed: `Connection: Initial data mode DQPSK R1/2 (SNR=10.0 dB, forced_mod=255, forced_rate=255)`

**Root cause:**
- `recommendDataModeWithFading()` auto-selected a waveform based on SNR/fading, ignoring the negotiated waveform
- At SNR=10/AWGN, it auto-selected OFDM_CHIRP, then passed that to `recommendDataMode()`
- Since OFDM (not MC-DPSK) was passed, the OFDM rate logic ran → R1/2 at SNR=10

**Files modified:**
- `src/protocol/connection_handlers.cpp`:
  - Renamed `recommendDataModeWithFading()` to `recommendDataModeForWaveform()`
  - Changed function to take waveform as INPUT instead of auto-selecting it
  - Call site now passes `negotiated_mode_` (the forced/negotiated waveform) instead of ignoring it

**How it works:**
- Waveform negotiation happens FIRST via `negotiateMode()` (respects forced waveform)
- If AUTO, select waveform based on SNR/fading
- Then call `recommendDataModeForWaveform()` with the negotiated waveform
- MC-DPSK now correctly triggers the R1/4 path in `recommendDataMode()`

**Test verification:**
```bash
./build/cli_simulator --snr 10 --test --waveform mc_dpsk 2>&1 | grep "Initial data mode"
# Expected: DQPSK R1/4
# ✓ Connection: Initial data mode DQPSK R1/4 (SNR=10.0 dB, forced_mod=255, forced_rate=255)

./build/cli_simulator --snr 8 --test 2>&1 | grep "Initial data mode"
# Expected: AUTO selects MC-DPSK R1/4 at low SNR
# ✓ Connection: Initial data mode DQPSK R1/4 (SNR=8.0 dB)
```

---

## 2026-01-28: Fix Disconnect ACK Code Rate (GUI Simulator)

**What was broken:**
- GUI simulator: After receiving DISCONNECT, the ACK was sent with R1/4 instead of R2/3
- Initiator couldn't decode ACK → timeout after 30 seconds
- Sequence: ACK queued → setConnected(false) called → ACK transmitted with wrong rate

**Root cause:**
- V2 Frame Path at modem_engine.cpp:283 checked `connected_ && handshake_complete_`
- When `setConnected(false)` was called, `connected_` became false
- The queued ACK was then transmitted with R1/4 instead of negotiated rate

**Files modified:**
- `src/gui/modem/modem_engine.cpp`: Added `use_connected_waveform_once_` to code rate check
  ```cpp
  // Before:
  tx_code_rate = (connected_ && handshake_complete_) ? data_code_rate_ : CodeRate::R1_4;
  // After:
  tx_code_rate = ((connected_ && handshake_complete_) || use_connected_waveform_once_) ? data_code_rate_ : CodeRate::R1_4;
  ```

**How it works:**
- `use_connected_waveform_once_` is set true when `setConnected(false)` is called
- This flag preserves the negotiated code rate for the disconnect ACK
- Flag is cleared after the ACK is transmitted

**Test verification:**
```bash
./build/cli_simulator --snr 20 --test
# Expected: DISCONNECT phase completes without timeout
# ✓ Disconnected!
```

---

## 2026-01-28: Fix total_cw Mismatch for Negotiated Code Rate Frames

**What was broken:**
- DISCONNECT frame (type=0x15) showed "PARTIAL (1/3 codewords)" on receiver
- Header had `total_cw=3` (calculated assuming R1/4) but encoded with R2/3 (1 codeword)
- `ConnectFrame::serialize()` calculates total_cw using R1/4 (default), but TX uses negotiated rate

**Root cause:**
- Frame serialization happens before code rate is determined
- `total_cw` in header is calculated at serialize time, not encode time
- Disconnect frame: 44 bytes payload → 3 codewords at R1/4, but 1 codeword at R2/3

**Files modified:**
- `src/gui/modem/modem_engine.cpp`: Added total_cw patching before LDPC encoding
  - Only patches data/connect frames (types 0x10-0x19 and 0x30-0x3F)
  - Control frames (ACK 0x20, NACK 0x21, etc.) are fixed 20 bytes = 1 codeword, no patching
  - Recalculates header CRC after patching

**How it works:**
1. Check if frame is data or connect type (needs total_cw field)
2. Read payload_len from header bytes 13-14
3. Calculate correct total_cw for actual TX code rate
4. Patch byte 12 if different
5. Recalculate header CRC (bytes 15-16)
6. Encode patched frame with LDPC

**Test verification:**
```bash
./build/cli_simulator --snr 20 --test
# Expected: DISCONNECT phase completes
# ✓ Disconnected!
# DISCONNECT frame shows total_cw=1 (not 3)
```

---

## 2026-01-28: Fix OFDM_COX Minimum Samples for Short Frames

**What was broken:**
- After receiving DATA, StreamingDecoder couldn't find ACK or subsequent frames
- OFDM_COX min_samples was set to 48000 but short frames (ACK = ~18000 samples) are smaller
- Available samples (19452) < min_samples (48000) caused decoder to skip

**Files modified:**
- `src/gui/modem/streaming_decoder.cpp`:
  - Changed OFDM_COX min_samples from `max(48000, getMinSamplesForFrame() * 2)` (was wrong)
  - To `max(15000, getMinSamplesForFrame() * 2)` (~14000 samples sufficient)

**Test verification:**
```bash
./build/cli_simulator --snr 20 --test
# Expected: All 3 messages received correctly
# ✓ Message 1 received correctly!
# ✓ Message 2 received correctly!
# ✓ Message 3 received correctly!
```

---

## 2026-01-28: Fix Control Frame Code Rate When Connected

**What was broken:**
- After connection, control frames (ACK, NACK, DISCONNECT) were sent with R1/4
- But receiver expected negotiated rate (e.g., R2/3)
- Caused ACK decode failures after DATA received correctly

**Root cause:**
- `modem_engine.cpp` line 283: `tx_code_rate = (is_data_frame && connected_) ? data_code_rate_ : CodeRate::R1_4;`
- This only used negotiated rate for DATA frames, not control frames

**Files modified:**
- `src/gui/modem/modem_engine.cpp`:
  - Changed rate selection: `tx_code_rate = (connected_ && handshake_complete_) ? data_code_rate_ : CodeRate::R1_4;`
  - Now ALL frames (data AND control) use negotiated rate after handshake

**How it works:**
1. Pre-connection (PING/PONG/CONNECT): Use R1/4 for robustness
2. During handshake (CONNECT_ACK): Still use R1/4 (remote not confirmed yet)
3. Post-handshake: ALL frames use negotiated rate for proper decoding

**Test verification:**
```bash
./build/cli_simulator --snr 20 --test
# Expected: All 3 messages received + ACKs decoded correctly
```

---

## 2026-01-28: Fix PING Detection in cli_simulator (Connection Phase)

**What was broken:**
- PING frames (chirp-only) were not being detected by StreamingDecoder
- Two root causes:
  1. Receiver needed MIN_SAMPLES_FOR_SEARCH (144000) but PING/PONG was only 57600 samples
  2. PING detection logic checked `codewords_ok == 0` but LDPC "succeeded" on garbage (codewords_ok=1)

**Files modified:**
- `src/gui/modem/modem_engine.cpp`: Added 100000 samples trailing silence to PING/PONG so receiver buffer fills
- `src/gui/modem/streaming_decoder.cpp`: Fixed PING detection logic
  - Changed check from `!result.success && result.codewords_ok == 0 && result.frame_data.empty()`
  - To `!result.success && result.frame_data.empty()` (catches LDPC "success" on garbage)
  - Added handlePingDetection() lambda for cleaner PING handling

**How it works:**
1. PING = chirp only (no training/data after)
2. After chirp detection, try to decode data
3. If no valid "UL" magic header found → it's a PING (regardless of LDPC success on noise)
4. Trailing silence ensures receiver has enough samples for chirp detection

**Test verification:**
```bash
./build/cli_simulator --snr 20
# Expected: Phase 1 CONNECTION shows "✓ Connected!"
# PING→PONG→CONNECT→CONNECT_ACK flow works
```

**Known limitation:** DATA phase still failing (separate issue with OFDM codeword handling)

---

## 2026-01-28: Add Fading Detection for Mode Negotiation

**What was changed:**
- Added per-carrier magnitude variance tracking to detect frequency-selective fading
- Mode negotiation now considers both SNR and fading index

**Files modified:**
- `src/psk/multi_carrier_dpsk.hpp`: Added `carrier_magnitudes_`, `getFadingIndex()`, `isFading()`
- `src/waveform/waveform_interface.hpp`: Added virtual `getFadingIndex()`, `isFading()`
- `src/waveform/mc_dpsk_waveform.hpp/cpp`: Override fading methods
- `src/gui/modem/streaming_decoder.hpp/cpp`: Added `last_fading_index_`, `getLastFadingIndex()`
- `src/gui/modem/modem_engine.hpp/cpp`: Added `getFadingIndex()`, `isFading()`
- `src/protocol/connection.hpp/cpp`: Added `fading_index_`, `setChannelQuality()`
- `src/protocol/connection_handlers.cpp`: Updated `negotiateMode()` with fading-aware logic
- `tools/cli_simulator.cpp`: Pass channel quality (SNR + fading) to protocol

**Mode selection logic:**
- SNR < 0 dB: MFSK (not implemented yet)
- SNR 0-10 dB: MC_DPSK
- SNR 10-17 dB: OFDM_CHIRP if fading, MC_DPSK if stable
- SNR > 17 dB: OFDM_COX if stable, OFDM_CHIRP if fading

**Fading index calculation:**
Coefficient of variation (std_dev / mean) of per-carrier magnitudes. Values > 0.4 indicate significant fading.

---

## 2026-01-28: Delete RxPipeline (Cleanup)

**What was changed:**
Removed the deprecated RxPipeline class. StreamingDecoder now handles all RX processing.

**Files removed:**
- `src/gui/modem/rx_pipeline.hpp` - DELETED
- `src/gui/modem/rx_pipeline.cpp` - DELETED

**Files modified:**
- `modem_engine.hpp`: Removed `rx_pipeline_` member and include
- `modem_engine.cpp`: Removed `rx_pipeline_` reset block
- `modem_mode.cpp`: Removed `rx_pipeline_` mode handling
- `fec/codec_interface.hpp`: Removed outdated comment
- `CMakeLists.txt`: Removed rx_pipeline.cpp from all 9 build targets

**Benefits:**
- Removed ~400 lines of deprecated code
- Cleaner codebase with single RX path (StreamingDecoder)
- Reduced binary size

**Test verification:**
```bash
./tests/regression_matrix.sh
# Expected: ALL TESTS PASSED! (11/11)
```

---

## 2026-01-28: TX Path Unification (Phase 4)

**What was changed:**
The TX path in `transmit()` now uses IWaveform abstraction instead of direct modulator calls.

**Before:** 4 separate if-else branches with direct modulator calls:
- MC-DPSK: `mc_dpsk_modulator_->modulate()` + `chirp_sync_->generate()`
- OFDM_CHIRP: `OFDMModulator chirp_modulator` + `chirp_sync_->generate()`
- OFDM_COX: `ofdm_modulator_->generatePreamble()` + `ofdm_modulator_->modulate()`
- OTFS: `otfs_modulator_->generatePreamble()` + `otfs_modulator_->modulate()`

**After:** Single IWaveform path for MC_DPSK, OFDM_CHIRP, OFDM_COX:
```cpp
ensureTxWaveform(active_waveform, tx_modulation, tx_code_rate);
preamble = active_tx_waveform_->generatePreamble();
modulated = active_tx_waveform_->modulate(to_modulate);
```

**OTFS:** Kept legacy path (no OTFSWaveform yet)

**Benefits:**
- Adding new waveform only requires implementing IWaveform (no TX code changes)
- Reduced code duplication (~50 lines removed)
- Consistent TX interface across all waveforms

**Test verification:**
```bash
./tests/regression_matrix.sh
# Expected: ALL TESTS PASSED! (11/11)
```

---

## 2026-01-28: Remove Legacy Acquisition Thread

**What was changed:**
The acquisition thread was running but its output (`detected_frame_queue_`) was never consumed.
StreamingDecoder now handles all RX processing, making the acquisition thread dead code.

**Files removed/modified:**
- `modem_engine.hpp`: Removed acquisition thread members, legacy RX buffer, processRxBuffer_* declarations
- `modem_rx.cpp`: Removed acquisitionLoop(), startAcquisitionThread(), stopAcquisitionThread(), buffer helpers
- `modem_rx_decode.cpp`: Removed ~1200 lines of legacy decode code (rxDecodeDPSK, processRxBuffer_*, etc.)
- `modem_engine.cpp`: Removed acquisition thread start/stop calls
- `modem_mode.cpp`: Replaced legacy buffer clears with `streaming_decoder_->reset()`

**Removed components:**
- `acquisition_thread_`, `acquisition_running_`, `acquisition_cv_`, `acquisition_mutex_`
- `rx_sample_buffer_`, `samples_consumed_`, `rx_buffer_mutex_`
- `detected_frame_queue_`, `rx_frame_state_`
- `rxDecodeDPSK()`, `processRxBuffer_OFDM/OTFS/DPSK/OFDM_CHIRP()`
- `waitForSamples()`, `deinterleaveCodewords()`, `detectPing()`
- Legacy accumulation state (ofdm_accumulated_soft_bits_, dpsk_accumulated_soft_bits_, etc.)

**Architecture after cleanup:**
- RX decode thread runs `rxDecodeLoop()` which drives `streaming_decoder_->processBuffer()`
- `feedAudio()` only feeds to StreamingDecoder
- Frame delivery via callbacks set in ModemEngine constructor
- Mode switches call `streaming_decoder_->reset()` instead of clearing legacy buffers

**Test verification:**
```bash
./tests/regression_matrix.sh
# Expected: ALL TESTS PASSED! (11/11)
```

---

## 2026-01-28: StreamingDecoder Becomes Primary Decoder

**What was broken:**
- StreamingDecoder frame decoding worked (3/3 codewords) but ConnectFrame::deserialize() failed
- CW0 decoded to 21 bytes instead of expected 20 bytes
- Frame reassembly used 21 bytes from CW0, causing 1-byte shift and CRC failure

**Root cause:**
LDPC R1/4 has 162 info bits = 20.25 bytes. Decoder returns `ceil(162/8) = 21` bytes,
but protocol `getBytesPerCodeword(R1_4)` returns `162/8 = 20` bytes (integer division).
The extra byte at position 20 is padding from fractional bits.

**What was changed:**
- `streaming_decoder.cpp`: Added CW0 resize to `bytes_per_cw` (20 bytes) after LDPC decode
- `modem_engine.hpp`: Fixed `setMCDPSKCarriers()` to recreate TX modulator and update StreamingDecoder
- `streaming_decoder.hpp/cpp`: Added `setMCDPSKCarriers()` method for carrier count sync

**How it's properly fixed:**
After LDPC decode, resize CW0 to exactly 20 bytes (discard padding):
```cpp
if (cw0_data.size() > bytes_per_cw) {
    cw0_data.resize(bytes_per_cw);  // Truncate to 20 bytes
}
```

**CFO handling verified:**
- Python analysis confirmed carrier frequencies shift by exactly the expected CFO amount
- CFO=30Hz: All 8 carriers shifted by 29.3-30.8 Hz (mean=30.0 Hz)
- CFO=0Hz: No shift (all 0.0 Hz)

**Test verification:**
```bash
./test_iwaveform --snr 10 -w mc_dpsk --frames 3 --cfo 30
# Expected: Decoded: 3/3 (100%)

./tests/regression_matrix.sh
# Expected: ALL TESTS PASSED! (11/11)
```

---

## 2026-01-28: StreamingDecoder Created (Fixes BUG-002: RxPipeline Broken)

**What was broken:**
- RxPipeline failed to detect chirps when integrated into ModemEngine
- test_iwaveform worked 100% using IWaveform directly
- RxPipeline integration in ModemEngine failed

**Root cause analysis:**
RxPipeline had incorrect IWaveform call sequence:
1. Line 147: `waveform_->setFrequencyOffset(sync_result.cfo_hz);` - CFO applied
2. Line 172: `waveform_->reset();` - CFO CLEARED (violates INV-WAVE-002!)
3. Line 173: `waveform_->process(process_span);` - Process with wrong CFO

Per INV-WAVE-002: "reset() MUST clear cfo_hz_ to prevent stale values"
This means calling reset() AFTER setFrequencyOffset() erases the CFO.

**What was changed:**
- Created `src/gui/modem/streaming_decoder.hpp` (~230 lines)
- Created `src/gui/modem/streaming_decoder.cpp` (~460 lines)
- Correct call sequence: reset() → detectSync() → setFrequencyOffset() → process()
- Circular buffer with bounded size (4 seconds max)
- Sliding window search (like test_iwaveform)
- Thread-safe with condition variable for blocking wait
- PING detection via energy ratio
- SNR estimation from chirp correlation
- Added to CMakeLists.txt for all executables

**How it's properly fixed:**
StreamingDecoder uses the correct IWaveform call sequence per INV-WAVE-001:
```cpp
waveform->reset();                           // Clear state
waveform->detectSync(samples, sync_result);  // Find preamble
waveform->setFrequencyOffset(sync_result.cfo_hz);  // Store CFO
waveform->process(samples_from_start);       // Demodulate
auto bits = waveform->getSoftBits();         // Get output
```

**Test verification:**
```bash
# Build with StreamingDecoder
make -j4 test_iwaveform  # Should compile without errors

# Regression tests pass
./tests/regression_matrix.sh
# Expected: ALL TESTS PASSED!
```

**Next steps:**
1. ~~Integrate StreamingDecoder into ModemEngine~~ DONE 2026-01-28
2. Make StreamingDecoder the primary decoder (currently parallel)
3. Remove acquisition thread
4. Replace processRxBuffer_* methods
5. Delete RxPipeline after integration verified

---

## 2026-01-28: StreamingDecoder Integration (Phase 2)

**What was changed:**
- `src/gui/modem/modem_engine.hpp`: Added `streaming_decoder_` member
- `src/gui/modem/modem_engine.cpp`: Initialize StreamingDecoder in constructor, set callbacks
- `src/gui/modem/modem_rx.cpp`:
  - feedAudio(): Feeds to StreamingDecoder in parallel with legacy path
  - rxDecodeLoop(): Checks StreamingDecoder for decoded frames

**Integration approach:**
Running in parallel mode for safety:
- Audio is fed to BOTH StreamingDecoder AND legacy path
- Legacy path (acquisition thread) still does primary decoding
- StreamingDecoder is receiving audio and processing but not yet primary

**Test verification:**
```bash
# All regression tests pass
./tests/regression_matrix.sh
# Expected: 11/11 PASS
```

**Status:** Parallel mode working. Next: Make StreamingDecoder primary.

---

## 2026-01-28: PING vs DPSK Frame Detection Fix (cli_simulator)

**What was broken:**
- cli_simulator connection phase failed - PING frames misdetected as "Chirp+DPSK frames"
- Energy threshold (0.05f) was absolute, failed at high SNR where noise exceeded threshold
- Overlapping chirps in buffer caused detection confusion

**Root cause analysis:**
1. Energy threshold was absolute (0.05f), not relative to signal level
2. At 20dB SNR, noise RMS (~0.057) exceeded the threshold
3. Multiple PINGs could pile up in buffer before processing
4. Energy ratio between chirp and post-chirp didn't account for fading or overlapping chirps

**What was changed:**
- `src/gui/modem/modem_rx.cpp`:
  - Changed PING/DPSK detection from absolute threshold to energy ratio (post_rms/chirp_rms)
  - Ratio < 0.3 = PING (post-chirp is noise)
  - Ratio 0.3-1.4 = DPSK data (similar energy levels)
  - Ratio > 1.4 = Another chirp starting (different transmission)
  - Added chirp detection in suspicious range (1.1-1.4): search for chirp in post-chirp region
  - Added 200ms guard period after consuming PING samples
- `src/gui/modem/modem_rx_constants.hpp`:
  - Reduced MIN_SAMPLES_FOR_ACQUISITION from 90000 to 65000 (PING is only 57600 samples)

**How it's properly fixed:**
- Energy ratio is SNR-independent (compares signal to signal, not signal to absolute)
- Fading channels can have ratio up to 1.3 due to energy variation - 1.4 threshold accommodates this
- When ratio is suspicious (1.1-1.4), quick chirp search in post region distinguishes overlapping chirps
- Guard period prevents partial chirp detection from overlapping transmissions

**Test verification:**
```bash
# CLI simulator should connect (PING/PONG, CONNECT, CONNECT_ACK)
./build/cli_simulator --snr 20 --test
# Expected: Connection phase succeeds, "✓ Connected!" displayed

# Regression tests all pass
./tests/regression_matrix.sh --quick
# Expected: 11/11 PASS, including MC-DPSK on fading channels
```

---

## 2026-01-28: MC-DPSK CFO Per-Segment Initial Phase Fix

**What was broken:**
- MC-DPSK degraded massively with CFO on fading channels
- Poor fading + CFO=30: 20% success (should be ~80%)
- Moderate fading + CFO=30: 40% success (should be ~80%)
- CFO=0 worked fine (80-100%), proving the issue was CFO handling

**Root cause analysis:**
- Each segment (training, ref, data) starts at a different sample position
- Each segment needs its OWN initial phase for CFO correction
- Bug: We set initial phase once for training_start, then used it for ALL segments
- Result: ref and data segments got wrong CFO correction, causing phase errors

**What was changed:**
- `src/gui/modem/modem_rx_decode.cpp` (3 locations in rxDecodeDPSK):
  - Added `calcInitialPhase` lambda to compute wrapped phase for any absolute position
  - Calculate separate initial phases: training_start_abs, ref_start_abs, data_start_abs
  - Call `setCFOWithPhase()` before each `applyCFO()` with the correct phase for that segment
  - Set final phase for data segment after processing training/ref

**How it's properly fixed:**
- Training at position T gets phase: -2π × CFO × T / sr
- Ref at position T+training_len gets phase: -2π × CFO × (T+training_len) / sr
- Data at position T+training_len+ref_len gets its own phase
- Each segment's CFO correction now starts at the correct accumulated phase
- Signal and correction cancel exactly for each segment independently

**Test verification:**
```bash
# MC-DPSK on poor fading with CFO
./build/test_iwaveform --snr 15 -w mc_dpsk --channel poor --cfo 30 --frames 5
# Expected: 80% (was 20% before fix)

# MC-DPSK on moderate fading with CFO
./build/test_iwaveform --snr 15 -w mc_dpsk --channel moderate --cfo 30 --frames 5
# Expected: 80% (was 40% before fix)
```

**Results after fix:**
| Channel | CFO=0 | CFO=30 |
|---------|-------|--------|
| Poor | 80% | 80% |
| Moderate | 80% | 80% |

---

## 2026-01-28: OFDM_CHIRP CFO Initial Phase in modem_rx_decode.cpp

**What was broken:**
- OFDM_CHIRP in modem_rx_decode.cpp used `setFrequencyOffset()` which resets phase to 0
- The IWaveform path (`ofdm_chirp_waveform.cpp`) already used `setFrequencyOffsetWithPhase()`
- modem_rx_decode.cpp path wasn't updated, causing CFO failures

**What was changed:**
- `src/gui/modem/modem_rx_decode.cpp` in `processRxBuffer_OFDM_CHIRP()`:
  - Track `buffer_start_abs` when taking samples from buffer
  - Calculate `training_start_abs = buffer_start_abs + chirp_end_offset`
  - Compute initial phase: -2π × CFO × training_start_abs / sr
  - Call `setFrequencyOffsetWithPhase(cfo_hz, initial_phase)` instead of `setFrequencyOffset(cfo_hz)`

**Test verification:**
```bash
./build/test_iwaveform --snr 15 -w ofdm_chirp --channel awgn --cfo 30 --rate r1_4 --frames 5
# Expected: 100%
```

---

## 2026-01-28: R1/4 Code Rate Required for Fading Channels

**What was broken:**
- OFDM_CHIRP with R1/2 (default): 0% on moderate fading
- R1/2 doesn't have enough redundancy for fading channels
- This was misdiagnosed as CFO or channel equalization issues

**What was changed:**
- No code changes - this is a configuration/usage discovery
- Added `--rate` flag to test_iwaveform.cpp for testing different rates

**How it's properly fixed:**
- Use R1/4 for fading channels (4x redundancy)
- R1/2 is only suitable for AWGN or very good channels
- MC-DPSK already uses R1/4 by default (protocol-defined)

**Test verification:**
```bash
# R1/2 on moderate fading - FAILS
./build/test_iwaveform --snr 15 -w ofdm_chirp --channel moderate --rate r1_2 --frames 5
# Expected: 0-20%

# R1/4 on moderate fading - WORKS
./build/test_iwaveform --snr 15 -w ofdm_chirp --channel moderate --rate r1_4 --frames 5
# Expected: 100%
```

**Performance comparison at 15dB:**
| Waveform | AWGN | Moderate (R1/2) | Moderate (R1/4) |
|----------|------|-----------------|-----------------|
| OFDM_CHIRP | 100% | 0% | 100% |
| MC-DPSK | 100% | 80% | 80% |

---

## 2026-01-27: Improved Channel Interleaver Symbol Separation

**What was broken:**
- OFDM_CHIRP fading performance was lower than expected (~60% on good HF)
- Interleaver only separated consecutive bits by 1 symbol (step=61, separation=1)
- Burst errors from fading affected adjacent bits, making LDPC correction harder

**What was changed:**
- `src/fec/ldpc_decoder.cpp`: Modified `findCoprimeStep()` to target step = 3 × bits_per_symbol
- For 60 bits/symbol: step changed from 61 to 181, separation from 1 to 3

**How it's properly fixed:**
- Consecutive input bits now land in OFDM symbols 3 apart instead of adjacent
- When fading causes a burst error in one symbol, the affected bits are spread
  across the codeword after deinterleaving
- LDPC can correct scattered errors better than clustered errors

**Test verification:**
```bash
# Good HF channel at 20 dB
for seed in 1 2 3 4 5; do
  ./build/test_iwaveform --snr 20 -w ofdm_chirp --channel good --frames 5 --seed $seed
done
# Expected: 80-100% (was 60-100%)
```

---

## 2026-01-27: OFDM_CHIRP CFO Initial Phase Fix

**What was broken:**
- OFDM_CHIRP failed at any CFO > 0 Hz (CFO=30 Hz: 0% success)
- CFO=0 worked perfectly (100%)
- MC-DPSK at CFO=30 Hz worked (100%), proving chirp detection was correct
- Root cause: CFO correction started from phase 0 instead of accumulated phase

**Root cause analysis:**
1. Test harness applies CFO to entire audio from sample 0
2. By training start (sample ~136,800), CFO has accumulated ~307° of phase
3. `processPresynced()` reset `freq_correction_phase = 0`, losing this accumulated phase
4. First training symbol got wrong CFO correction, corrupting H estimate
5. DQPSK differential decoding failed due to phase mismatch

**What was changed:**
1. `include/ultra/ofdm.hpp` + `src/ofdm/demodulator.cpp`:
   - Added `setFrequencyOffsetWithPhase(float cfo_hz, float initial_phase_rad)`
   - Sets both CFO and initial correction phase

2. `src/waveform/ofdm_chirp_waveform.hpp` + `.cpp`:
   - Added `training_start_sample_` member variable
   - `detectSync()`: Stores training start position
   - `process()`: Calculates initial phase = -2π × CFO × training_start / sample_rate
   - Calls `setFrequencyOffsetWithPhase()` instead of `setFrequencyOffset()`

3. `src/ofdm/demodulator.cpp`:
   - `processPresynced()`: Removed reset of `freq_correction_phase` to preserve initial phase

4. `src/ofdm/channel_equalizer.cpp`:
   - Simplified `lts_carrier_phases` to use (1,0) reference
   - With correct initial phase, no phase compensation needed
   - Previous `conj(h_unit) * phase_advance` was wrong with correct initial phase

**How it's properly fixed:**
- Initial CFO phase = -2π × CFO × training_start_sample / sample_rate
- This matches the accumulated CFO phase in the signal at training start
- CFO correction is now continuous from sample 0 (effectively)
- Signal's +φ and correction's -φ cancel exactly: corrected = TX
- DQPSK reference = (1,0) because equalized = TX (no extra phase)

**Test verification:**
```bash
# Test full CFO range
for cfo in -50 -30 0 30 50; do
  ./build/test_iwaveform -w ofdm_chirp --snr 17 --cfo $cfo --frames 3
done
# Expected: 100% success for all CFO values
```

**Results:** OFDM_CHIRP now works with ±50 Hz CFO at 10-20 dB SNR.

---

## 2026-01-27: OFDM_CHIRP CFO Test Harness Fix

**What was broken:**
- OFDM_CHIRP decoding failed for most CFO values (only CFO=0 reliable)
- CFO=10 Hz: 0% success, CFO=30 Hz: 20% success
- Root cause: FIR Hilbert transform (127-tap) in test_iwaveform had 63-sample group delay
- This caused CFO-dependent timing shifts that broke OFDM symbol alignment

**What was changed:**
- `tools/test_iwaveform.cpp`: Replaced FIR Hilbert with FFT-based Hilbert (no group delay)
  - FFT signal -> zero negative frequencies, double positive -> IFFT
  - This creates perfect analytic signal without timing artifacts
- `src/sync/chirp_sync.hpp`: Removed HILBERT_GROUP_DELAY (63 sample) correction
  - Was compensating for old FIR delay which no longer exists

**How it's properly fixed:**
- FFT-based Hilbert has zero group delay (unlike FIR which has N/2 delay)
- CFO simulation now shifts frequency without shifting timing
- Chirp position correction only accounts for CFO-induced peak shift, not filter delay

**Test verification:**
```bash
# Test CFO range -45 to +50 Hz
for cfo in -45 -30 0 30 50; do
  ./test_iwaveform -w ofdm_chirp --snr 15 --cfo $cfo --frames 1
done
# Expected: 100% success for all CFO values
```

**Note:** This was a TEST HARNESS bug, not a demodulator bug. Real radios don't have this issue.

---

## 2026-01-27: CFO Accumulation Bug Fix

**What was broken:**
- MC-DPSK failed on subsequent frames when CFO ~0 Hz
- Frame 1 decoded, Frames 2+ failed LDPC
- Residual CFO from training accumulated via `cfo_hz_ += residual_cfo`

**What was changed:**
- `src/gui/modem/modem_rx_decode.cpp`: Always call `setCFO(frame.cfo_hz)` to reset accumulated CFO
- Previously only called when `abs(cfo_hz) > 0.1f`
- Fixed in 3 places: PING decode, CW0 decode, full frame decode

**How it's properly fixed:**
- `setCFO()` resets `cfo_hz_` to the chirp-detected value
- This prevents residual CFO from training from accumulating across frames
- Chirp CFO is then re-estimated for each frame independently

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 0 --channel awgn -w mc_dpsk --frames 5
# Expected: 100% decode rate (was 20% before fix)
```

**Commit:** `a2e6bed Fix CFO accumulation bug and improve test_iwaveform continuous RX`

---

## 2026-01-27: Demodulator Reset Per Frame

**What was broken:**
- Continuous RX decode degraded on subsequent frames at marginal SNR
- Demodulator state from previous frame affected current decode

**What was changed:**
- `src/gui/modem/modem_rx_decode.cpp`: Added `mc_dpsk_demodulator_->reset()` at start of `rxDecodeDPSK()`

**How it's properly fixed:**
- Reset clears carrier phases, previous symbols, and other state
- CFO is then set from chirp detection via `setCFO()`
- Each frame gets clean demodulator state

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 30 --channel awgn -w mc_dpsk --frames 5
# Expected: 100% decode rate
```

**Commit:** `e52705b Add demodulator reset at start of each DPSK frame decode`

---

## 2026-01-27: test_iwaveform Continuous RX Mode

**What was broken:**
- test_iwaveform created fresh RX ModemEngine per frame ("cheating")
- Didn't test realistic continuous audio streaming
- Buffer overflow when feeding too much audio at once

**What was changed:**
- `tools/test_iwaveform.cpp`: Use single RX ModemEngine for entire audio stream
- Add throttling pauses every 5 seconds to let acquisition process
- Reduce gap between frames (1.5s) to fit under MAX_PENDING_SAMPLES (960000)
- Track decoded frames by sequence number using std::set

**How it's properly fixed:**
- Realistic test: audio streamed continuously like from HF rig
- Throttling prevents buffer overflow (acquisition can't keep up with instant feed)
- Single RX instance tests state management between frames

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 30 --channel awgn -w mc_dpsk --frames 5
# Expected: 100% decode rate
```

**Commit:** `a2e6bed Fix CFO accumulation bug and improve test_iwaveform continuous RX`

---

## 2026-01-27: IWaveform Interface Documentation

**What was done:**
- Created comprehensive documentation for refactoring reference

**Files created:**
- `docs/archive/MODEM_ENGINE_ARCHITECTURE.md` - Complete ModemEngine analysis
- `docs/archive/DUAL_CHIRP_CFO_ANALYSIS.md` - CFO detection and position handling
- `docs/archive/TESTING_METHODOLOGY.md` - Test tools and requirements

**Why it matters:**
- ModemEngine has two parallel code paths (old direct modulators, new IWaveform)
- RxPipeline integration has bugs - old `processRxBuffer_*` methods still work
- CFO must be applied via Hilbert transform, not simple multiplication

---

## 2026-01-27: OFDM_CHIRP Support in test_iwaveform

**What was broken:**
- test_iwaveform.cpp could not decode OFDM_CHIRP frames
- ModemEngine's acquisition thread routes ALL chirp frames to MC-DPSK decoder
- OFDMChirpWaveform::process() only returned 648 soft bits instead of all

**What was changed:**
- `tools/test_iwaveform.cpp`: Added `decodeOFDMChirpFrame()` that uses IWaveform directly
- `tools/test_iwaveform.cpp`: Added `setConnectWaveform()` call for TX (connect_waveform_ is used for disconnected mode TX, not waveform_mode_)
- `src/waveform/ofdm_chirp_waveform.cpp`: Fixed `process()` to loop and retrieve ALL soft bits from demodulator

**How it's properly fixed:**
- OFDM_CHIRP decode bypasses ModemEngine and uses IWaveform directly
- TX uses `setConnectWaveform(mode)` in addition to `setWaveformMode(mode)`
- `process()` now calls `demodulator_->getSoftBits()` in a loop until `hasPendingData()` returns false

**Test verification:**
```bash
./test_iwaveform --snr 17 --cfo 30 --channel awgn -w ofdm_chirp --frames 10
# Expected: 100% decode rate
```

**Commit:** `84bb563 Add OFDM_CHIRP support to test_iwaveform with CFO correction`

---

## 2026-01-27: MC-DPSK CFO Correction for Training/Reference Samples

**What was broken:**
- MC-DPSK decode failed with CFO on fading channels
- Training and reference samples were receiving UNCORRECTED signal
- `processTraining()` was estimating wrong residual CFO

**What was changed:**
- `src/psk/multi_carrier_dpsk.hpp`: CFO correction applied to training/ref samples BEFORE `processTraining()`
- Added public `applyCFO()` wrapper method that preserves `cfo_hz_` after correction

**How it's properly fixed:**
- CFO correction must happen BEFORE `processTraining()`, not after
- The demodulator's `applyCFOCorrection()` resets `cfo_hz_` to 0, so we save/restore it
- Chirp CFO is trusted over training CFO (more accurate from 1+ second signal)

**Invariants:**
1. CFO from chirp detection is the most accurate - trust it
2. Apply CFO to ALL samples (training, ref, data) before demodulation
3. Don't let `processTraining()` overwrite chirp CFO estimate

**Test verification:**
```bash
./test_iwaveform --snr 10 --cfo 30 --channel moderate -w mc_dpsk --frames 10
# Expected: 100% decode rate
```

**Commit:** `48e6271 Fix MC-DPSK CFO correction for training/reference samples`

---

## 2026-01-26: Complex Correlation for CFO-Tolerant Chirp Detection

**What was broken:**
- Real-valued chirp correlation oscillated at CFO beat frequency
- Detection position varied with CFO (±24-48 samples error)
- CFO estimation was inaccurate (~11.7 Hz for 20 Hz actual)

**What was changed:**
- `src/sync/chirp_sync.hpp`: Added cosine templates alongside sine templates
- `src/sync/chirp_sync.hpp`: New `computeComplexTemplateCorrelation()` returns magnitude √(I² + Q²)

**How it's properly fixed:**
- Complex correlation: I = Σ signal × cos(phase), Q = Σ signal × sin(phase)
- Magnitude √(I² + Q²) is CFO-invariant (phase rotation doesn't change magnitude)
- Peak position is now consistent regardless of CFO

**Invariants:**
1. Always use complex correlation for chirp detection
2. Dual chirp gap timing gives CFO estimate (up shifts left, down shifts right)
3. Position correction: `true_pos = detected_pos + CFO × 10`

**Test verification:**
```bash
./test_iwaveform --snr 5 --cfo 50 --channel awgn -w mc_dpsk --frames 10
# Expected: 100% decode rate
```

---

## 2026-01-26: OFDM_CHIRP CFO - Trust Chirp Estimate

**What was broken:**
- OFDM_CHIRP decode failed with CFO
- Training symbol CFO estimation was overwriting correct chirp CFO
- Training was measuring carrier phase advance (wrong metric)

**What was changed:**
- `src/ofdm/demodulator_impl.hpp`: Added `chirp_cfo_estimated` flag
- Flag is set when `setFrequencyOffset()` is called
- `processPresynced()` trusts chirp CFO instead of re-estimating from training

**How it's properly fixed:**
- When chirp-based CFO is available, skip training-based re-estimation
- Training-based CFO is less accurate (100ms vs 1+ second signal)
- The `toBaseband()` function applies CFO correction before FFT

**Invariants:**
1. Chirp CFO > Training CFO in accuracy
2. Set `chirp_cfo_estimated = true` when CFO comes from chirp detection
3. Apply CFO in `toBaseband()` before FFT

**Test verification:**
```bash
./test_iwaveform --snr 17 --cfo 50 --channel awgn -w ofdm_chirp --frames 10
# Expected: 100% decode rate
```
