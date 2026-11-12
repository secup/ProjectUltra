# Critical Invariants

**Purpose:** Document ALL critical rules and invariants that MUST be maintained. Violating these will cause subtle bugs that are hard to debug. Read this before making changes.

**IMPORTANT:** If you change any of these invariants, you MUST:
1. Update this document
2. Add entry to CHANGELOG.md explaining why
3. Update all affected code paths
4. Run full regression tests

---

## Simulator Channel Invariants

### INV-SIM-AWGN-001: AWGN Is Continuous RX Noise

**Rule:** `SimulatedChannel` synthetic AWGN is added at RX time to every
returned sample when the channel is explicitly configured as `ChannelType::AWGN`.
Noise must not depend on whether a peer TX buffer has audio, silence, or no
samples. Do not add synthetic AWGN on both TX and RX.

**Reference:** AWGN standard deviation is computed from the fixed modem PING
reference measured from `StreamingEncoder::encodePing()` output: broadband RMS
`0.3180724`, then receiver-FIR in-band RMS `0.30482664`. Configured `snr_db` is
operator-facing in-band SNR: the generated broadband white-noise sigma is
increased by `1 / sqrt(kModemInBandNoisePowerFraction)` so the 50-2950 Hz
receive FIR leaves the requested in-band noise power.

**Why:** Real HF receivers hear band noise continuously. A simulator that is
quiet between bursts lets sync, PING detection, SNR estimation, and decode
admission gates calibrate against impossible silence.

**Boundaries:**
- Watterson fading owns its own channel noise and must not receive an extra
  `SimulatedChannel` AWGN layer.
- The real-HF WAV noise overlay remains an independent additive RX overlay.
- No modem PHY threshold should be tuned against simulator silence floors.

**Location:** `tools/sim/simulated_station.hpp::SimulatedChannel`.

---

## CFO (Carrier Frequency Offset) Invariants

### INV-CFO-001: Chirp CFO is Coarse — LTS Refines It

**Rule:** Chirp CFO is more accurate than training-based estimation on AWGN, but on fading channels it can be wrong by ±2 Hz. The LTS residual estimation in `estimateChannelFromLTS()` REFINES chirp CFO per frame. The pilot tracking FURTHER refines it per symbol. The feedback loop propagates corrections back.

**Why:** Fading distorts chirp peaks → false CFO. LTS compares 2 known training symbols to measure the residual. Pilots track per-symbol drift. Without this pipeline, a 1.4 Hz error causes 13°/symbol phase drift → DQPSK decode failures.

**Implementation (4 stages):**
```
1. Chirp coarse CFO → stored as last_cfo_ (±50 Hz range; on multipath a PHANTOM of up to
   delay_spread / (2 kappa) Hz, kappa = 10 samples/Hz: ~2.4 Hz on ITU Moderate, 1 ms)
2. LTS residual fix → K·r when the CFO prior is UNCERTIFIED and |r| > t(n-1, 3 sigma) sigma_r
   (src/ofdm/lts_cfo_update.hpp); a CERTIFIED prior is never corrected
3. Pilot tracking → refines freq_offset_hz per data symbol
4. Feedback loop → propagates corrected CFO back to waveform and StreamingDecoder
```

**2026-11-08 corollary (the bug this rule replaced):** never gate the LTS correction on the
MAGNITUDE of the prior CFO or on channel flatness. The decoder pre-corrects every OFDM frame and
hands the demod 0 Hz, so a "|seed| ≥ 0.75 Hz" test is dead code, and a flatness test is false on
every fading channel — together they left a wrong chirp CFO uncorrected on fading (measured
Moderate QPSK R1/2 14.6 % FER vs 0.0 % with the true CFO). The decision depends only on the
prior's STATE (certified or not) and the measurement's own significance.

**See `docs/CFO_CORRECTION_FLOW.md` for full details.**

**Locations:**
- `src/sync/chirp_sync.hpp` (chirp detection)
- `src/ofdm/channel_equalizer.cpp` (LTS residual, pilot tracking, toBaseband)
- `src/waveform/ofdm_chirp_waveform.cpp` (feedback: `cfo_hz_ = demodulator_->getFrequencyOffset()`)
- `src/gui/modem/streaming_decoder.cpp` (feedback: `last_cfo_.store(corrected_cfo)`)
- `src/ofdm/demodulator_impl.hpp` (chirp_cfo_estimated flag)

---

### INV-CFO-004: Never shift a frame with a truncated Hilbert; mix it (2026-11-08)

**Rule:** remove CFO with a complex mix inside the demodulator — the OFDM `toBaseband` NCO, the
MC-DPSK per-carrier correlator (`demodulateOneSymbol`, CFO term continuous across the frame and
every burst continuation) — never with a short FIR Hilbert pre-shift of a finite frame block.

**Why:** a 65-tap (OFDM) / 127-tap (MC-DPSK) full-band Hilbert is accurate only above ~3·fs/N
(2.2 / 1.1 kHz); below that each carrier keeps an image that the shift moves 2·CFO away, the
filter delay (32 / 63 samples) slides every symbol window, and the MC-DPSK version zeroed the CFO
after use so burst continuations got no correction at all. Invisible at 0 Hz (the shift is skipped).

**Corollary (2026-11-11):** the correlator's outputs are CFO-free but the samples are not, so any
model of the received samples rebuilt from those outputs (the MC-DPSK training-SNR fit,
`updateTrainingSNREstimate`) must re-apply the same continuous CFO phase. Without it the training
estimate read -2.4 dB at any SNR on a +7.3 Hz link (BUG-MCDPSK-TRAINING-SNR-CFO). CTest
`MCDPSKSnrCalibration` gates CFO invariance (0 / +7.3 / -23.5 Hz).

### INV-CFO-005: The chirp guard may only fall back to a MEASURED CFO (2026-11-08)

**Rule:** `limitConnectedCFODrift` clamps a chirp to the tracked value only when
`CFOTracker::established()` (the value came from a measurement since the last reset). An
unestablished tracker accepts the chirp; the first OFDM frame's LTS stage corrects a phantom.

**Why:** the default 0 Hz is not information. Falling back to it turned a correct 7.3 Hz chirp
into a 7.3 Hz error on 27/48 Good@20 frames — and looked correct for months because the simulator's
truth was 0 Hz. Never judge "established" from |CFO| > ε either (a true 0 Hz is established too).

**Same rule for the preamble-less detectors (2026-11-12):** structure acquisition and the light-LTS
sync report the burst's own CFO (`SyncResult::measured_cfo_hz`); with the tracker unestablished
`searchForSync` acquires on it, never on the 0 Hz default it searched at
(BUG-STRUCTURE-ACQ-DROPS-CFO). **Test:** `FrameStructureDetector` §8 (−15..+15 Hz, cold or
tracked, within 4σ).


### INV-CFO-006: A burst that proves nothing falls back to the session's measurement, never to 0 Hz (2026-11-10)

**Rule:** when the burst outcome revokes the tracked CFO and no delivered group has certified a
value yet, `CFOTracker` restores the SESSION REFERENCE — the handshake's dual-chirp CFO, set by
`StreamingDecoder::setKnownCFO` at connect — established but uncertified (cold, so the LTS still
corrects it within the phantom bound). Only a tracker with no session reference falls to the
0 Hz default. A certificate always takes precedence; `reset()` clears both.
**Why:** the old revoke-to-0 Hz turned the first unproven burst of a session (descriptor lost at
low SNR) into a permanent 7.3 Hz error: the LTS residual (7.29 Hz, chi 36) is rejected beyond
the 5 Hz phantom bound, every later frame demodulated as noise (|LLR| ~0.1) and the link never
recovered (BUG-CFO-REVOKE-TO-ZERO). **Test:** `StreamingSignalPolicy`
(`test_burst_outcome_cfo_reconciliation`: all three revoke paths, certificate precedence,
reset, non-finite reference).

### INV-CHIRP-003: A chirp search must start kappa x CFO_max before the expected chirp

**Rule:** a connected chirp search window may begin `SyncController::kChirpCfoPeakLeadSamples`
(κ·100 Hz = 1000 samples) before the post-frame search floor; LTS-type detectors on the same
buffer skip that lead (`SearchWindowResult::pre_floor_lead_samples`).

**Why:** the up-chirp's correlation peak precedes the chirp start by κ·CFO for a positive CFO. A
chirp starting exactly at the floor (every FULL-anchored resend group, right after its
descriptor) was never found at +7.3 Hz: mistimed fallback, group lost, resend repeated.

---

### INV-CFO-002: CFO Must Be Applied BEFORE Demodulation

**Rule:** Call `setFrequencyOffset()` BEFORE `process()`. Never after.

**Why:** CFO correction happens sample-by-sample during baseband conversion. If you set it after processing, it's too late.

**Correct order:**
```cpp
waveform->reset();
waveform->detectSync(samples, result);
waveform->setFrequencyOffset(result.cfo_hz);  // BEFORE process!
waveform->process(samples);
```

**Wrong order (WILL FAIL):**
```cpp
waveform->process(samples);
waveform->setFrequencyOffset(cfo);  // TOO LATE!
```

---

### INV-CFO-003: CFO Simulation Must Use Hilbert Transform

**Rule:** To simulate CFO in test signals, use Hilbert transform for single-sideband frequency shift. Simple `cos(2π·CFO·t)` multiplication creates mirror images.

**Why:** Real frequency shift is `e^{j·2π·CFO·t}` multiplication on complex signal. For real signals, we need Hilbert transform to get the analytic signal first.

**Correct implementation:** see the FFT/Hilbert CFO injection used by the maintained simulator/test tools.
```cpp
// 1. Compute Hilbert transform (imaginary part)
std::vector<float> hilbert = computeHilbert(signal);

// 2. Create analytic signal: z = signal + j·hilbert
// 3. Multiply by e^{j·2π·CFO·t}
// 4. Take real part
for (size_t i = 0; i < signal.size(); i++) {
    float phase = 2.0f * M_PI * cfo_hz * i / sample_rate;
    float cos_p = std::cos(phase);
    float sin_p = std::sin(phase);
    signal[i] = signal[i] * cos_p - hilbert[i] * sin_p;
}
```

**Wrong (creates images):**
```cpp
signal[i] *= std::cos(2.0f * M_PI * cfo_hz * i / sample_rate);  // WRONG!
```

---

## Chirp Detection Invariants

### INV-CHIRP-001: Use Complex Correlation for Chirp Detection

**Rule:** Always use I/Q (complex) correlation, never real-only correlation.

**Why:** Real correlation oscillates at the CFO beat frequency, causing position errors. Complex correlation magnitude is CFO-invariant.

**Correct:**
```cpp
float corr_I = sum(signal * cos(chirp_phase));  // In-phase
float corr_Q = sum(signal * sin(chirp_phase));  // Quadrature
float magnitude = sqrt(corr_I*corr_I + corr_Q*corr_Q);  // CFO-invariant!
```

**Location:** `src/sync/chirp_sync.hpp:605-628`

---

### INV-CHIRP-002: Dual Chirp Gap Timing Gives CFO

**Rule:** CFO is calculated from the gap change between up and down chirp peaks.

**Formula:**
```
expected_gap = chirp_length + gap_duration  (samples)
actual_gap = down_pos - up_pos
gap_error = actual_gap - expected_gap
CFO = gap_error / (2 × cfo_to_samples)

where: cfo_to_samples = sample_rate / chirp_rate = 48000 / 4800 = 10 samples/Hz
```

**Why opposite shifts:**
- Up chirp (300→2700 Hz): CFO shifts correlation peak LEFT by CFO×10 samples
- Down chirp (2700→300 Hz): CFO shifts correlation peak RIGHT by CFO×10 samples
- Total gap change = 2 × CFO × 10 samples

**Location:** `src/sync/chirp_sync.hpp:439-456`

**cfo_to_samples depends on the chirp:** 10 samples/Hz on the wideband chirp, 96 on the narrowband
handshake chirp (500 Hz in 1 s). A bound on a CFO-dependent timing quantity must be written in Hz and
converted with the chirp's own rate, never as a fixed sample count (BUG-NARROW-PING-GAP: 1000 samples
was +/-50 Hz wideband and +/-5.2 Hz narrowband).

---

### INV-SYNC-CHIRP-002: a normalized chirp correlation never exceeds 1 (2026-11-10)

**Rule:** `detectChirpTemplateFFT` normalizes by window energies from a DOUBLE running sum, floored
at 1e-6 of the whole buffer's energy. Never normalize an FFT correlation by a window energy that can
fall below the FFT's rounding noise.
**Why:** the FFT correlation's rounding noise (about u log2 N of sqrt(E_buffer E_template)) sits at
every lag, however quiet the window there. Divided by a near-zero window energy it gave 3477 and
false chirps that hid real ones (BUG-CHIRP-NORM-SILENCE, a 44.1 kHz WAV).
**Test:** `CliLoopback`; scan of a resampled multi-frame file (CHANGELOG 2026-11-10).

---

### INV-SYNC-LTS-001: an LTS lock is the TOP of the first peak, placed by the matched filter (2026-11-10)

**Rule:** `detectDataSync` may stop at the first high-confidence Schmidl-Cox peak (later identical
data symbols make false peaks), but it reports that peak's location, not its confidence crossing
and not the maximum of its flat top: when the matched filter against the known LTS clearly sees
the peak, the lock is the matched filter's maximum over the WHOLE peak region (one symbol either
side of the crossing; searching one side only finds its side peak at the second LTS symbol, one
symbol late). Never report a threshold crossing as a timing.
**Why:** two identical LTS symbols give a Schmidl-Cox peak about one symbol wide whose top is
flat; at high SNR it crosses 0.95 tens of samples before the start. Every member of a burst is
sliced at stride from this lock (INV-SCO-001), so the error hit whole groups: ~33 samples early,
~6 dB of EVM (BUG-LTS-EARLY-EXIT).
**Test:** `WaveformLoopback` (`test_ofdm_chirp_data_preamble_timing_high_snr`).

## Structure Acquisition Invariants (`ULTRA_BURST_ACQ=structure`, 2026-11-08)

### INV-SYNC-STRUCT-001: Structure detection is for GROUP STARTS only

**Rule:** `IWaveform::detectFrameStructure` runs only in the connected group-start search
(`searchForSync`, `structure_anchor_search` — where chirp mode waits for the dual chirp). Never
route `detectDataSync`, warm windows or the generic light search through it.

**Why:** its template spans 10 symbols (11 200 samples); the warm narrow window is 4 416 and the
light window 9 600 samples, so it can never fire there. The first build hooked it into
`detectDataSync` and stalled every transfer at 6 % (no data frame was ever acquired → RTO loop).

**Location:** `streaming_sync_acquisition.cpp` `searchForSync`; test `FrameStructureDetector` §5.

### INV-SYNC-STRUCT-002: Test BOTH marker hypotheses

**Rule:** a group-start frame carries LTS1 negated (`markFirstLTSSymbolForBurstGroup`). Any
coherent statistic that sums LTS1-referenced and LTS2-referenced pairs together must evaluate
both signs (χ₊ = Σ|B+A|²/n, χ₋ = Σ|B−A|²/n) and count both in the union bound.

**Why:** the single sum cancels on marked frames: the marked head was missed, the unmarked
frame 2 locked as a "group start", and every member was dropped as HEADNULL.

**Location:** `ofdm_chirp_waveform.cpp` `detectFrameStructure` (`chiOf`); test §6.

### INV-SYNC-STRUCT-003: The search window is derived, not chosen

**Rule:** window = template span + `SyncController::kSearchBacktrackSamples` +
`kRmsProbeSamples` (`structureSearchWindowSamples`). Once the RMS probe sees a frame, the window
starts one backtrack before the cursor, so the frame can begin up to backtrack + probe into it.
Change the backtrack or the probe only through those constants.

### INV-SYNC-STRUCT-004: The threshold comes from a false-lock budget

**Rule:** per-pass budget = (false locks per hour) × `CORRELATION_STEP` / (3600 · fs), split by
the detector over candidates × templates × 2 marker hypotheses; threshold = Gamma(K,1) inverse
survival. Never replace it with a tuned χ number.

### INV-SYNC-STRUCT-005: Where the chirp stays

**Rule:** handshake (PING/PONG/CONNECT), MC-DPSK, OFDM_NARROW, the session-first burst and the
handshake-confirmation window (the half-open MC-DPSK CONNECT fallback needs a dual-chirp lock),
and RTO resends (no ACK evidence ⇒ cold re-anchor). A chirp-less burst start gets the settling
lead-in fill (30 ms silent T/R floor first).

---

## Tone-Burst ACK Invariants (2026-11-08)

### INV-TONE-ACK-001: The staircase can only emit rungs the monitor scans

**Rule:** `kStaircaseSymbolMs` (`tone_burst_constants.hpp`) is BOTH the set `symbolMsForSNR`
returns from and the production monitor's scan set (`streaming_decoder.cpp` assigns it and sizes
the monitor buffer from its slowest member). Never add a rung to one without the other.
**Why:** a rung the sender emits but the monitor does not scan is a guaranteed miss and a full
RTO; until 2026-11-08 the staircase floor was 200 ms, which the monitor could not even buffer.
**Test:** `ToneBurstAckPayload` (`symbol_ms_staircase`: every return value is in the set).

### INV-TONE-ACK-002: Diversity is lockstep and power-combined

**Rule:** the two-subband ACK (`kToneFreqHzDiversity`, `ULTRA_TONE_ACK_DIVERSITY`, default ON) is
a wire change — both stations must run the same setting. TX sends each symbol on both subbands at
HALF amplitude (same peak as one tone); RX square-law combines (|X_A|^2 + |X_B|^2 per tone) and
the monitor's ideal-peak reference is scaled by 1/sqrt(2). The offset is derived (minimax of
cos^2(pi df tau) over the ITU delays, IMD3 out of band) — pinned by `ToneBurstAckPayload`
`constants_sanity`; do not move a subband without re-running that derivation.

### INV-TONE-ACK-003: The tone detector's carrier-offset prior is the tracked reverse-link CFO

**Rule:** before every monitor feed the decoder sets the detector prior to
`cfo_tracker_.tracked()` (the physical shift of the received audio, rx = tx shifted by +cfo) with
half-width 0 when pilot-certified, the dual-chirp phantom bound (tau_max x chirp_rate / 2 =
4.8 Hz) when uncertified, and the +-50 Hz acquisition range when no estimate exists. The detector
probes f + prior on a 1/(2T) grid. **Why:** the Goertzel main lobe is 2/T wide — 7.3 Hz of dial
offset cost the 100 ms rung > 6 dB. **Test:** `StreamingDecoderToneBurstMonitor` (a +7.3 Hz
100 ms ACK at -4 dB: tracker +7.3 decodes 10/10, -7.3 decodes 0/10 — the sign is pinned) and
`ToneBurstAckWatterson` (`cfo_*` cells).

### INV-TONE-ACK-004: Timing candidates are lobe maxima above an ABSOLUTE floor

**Rule:** `detectAndDecode` tries the local maxima of the coarse Costas statistic, strongest
first, while above the caller's absolute floor (the monitor: `min_peak_fraction` x
`idealCostasPeak`), within the distinct-candidate budget. Never gate candidates on a statistic of
the window itself (average, median): the ACK fills most of a tail window, so such a gate admits
only the global maximum, and a payload containing the Costas sequence (~10 % of payloads) then
loses its ACK whenever the alias outscores the preamble (BUG-TONE-ACK-CLEAN-MISS).
**Test:** `ToneBurstAckRoundTrip` (`costas_alias_in_payload_does_not_hide_the_preamble`).

### INV-TONE-ACK-005: a deferred ACK is held only by evidence newer than its own decode

**Rule:** a tone ACK that cannot be keyed at once (energy CCA busy, declared airtime still ahead,
or no completed-turn provenance with recent decoder evidence) is decided on every GUI and headless
TNC tick by `connection_policy::decideDeferredToneAck`: three consecutive quiet polls send; at the
deadline (2.5 s, or the remaining declared airtime + 0.5 s) declared airtime or NEW decoder evidence
drops it and energy-only busy sends it (F127, F129, F176). For an ACK produced by a decode, new
decoder evidence is a substantive stamp (accepted sync, consumed descriptor, decoded codewords)
newer than the one present when the ACK was deferred (`deferredToneAckSeesNewInbound`) — never the
age of any evidence: the decode that produced the ACK is itself evidence, so an age window held
every such ACK through its deadline (BUG-ASYNC-ACK-ALWAYS-DROPPED). Never count the broad stamp;
noise false-lock attempts set it (F147). The anchored-burst backstop's re-confirm (recognized by
the decoder's backstop stamp, `toneAckIsTimerReconfirm`) keeps the age rule: it fires about when
the sender's RTO resend starts, before that resend has produced any evidence
(BUG-BACKSTOP-RACES-RTO). For the same reason the connection withholds any re-confirm that could
not END before the sender's resend of the last framed group (`anchoredBackstopReconfirmFits`; the
window is timed from the anchor's training start), and drops its pending delayed SACK: no sensor here
sees a resend's first ~1.5 s. And a DEFERRED asynchronous ACK is dropped, at every poll, once keying it
could not end before the sender's resend of the group whose anchor this end accepted
(`deferredToneAckCannotEndBeforeResend`, GUI and TNC alike; BUG-DEFERRED-ACK-INTO-RESEND). `ULTRA_ASYNC_ACK_NOVELTY=0` applies the age rule to every ACK (and the
TNC's immediate drop). **Test:** `ConnectionPolicy` (`test_deferred_tone_ack_policy`).

---

## Handshake Invariants (2026-11-10)

### INV-HANDSHAKE-001: a connect frame carries its sender's callsign and fits the smallest frame

**Rule:** CONNECT / CONNECT_ACK / CONNECT_NAK carry the SENDER's full callsign (station
identification) and address the peer by `dst_hash` only; nothing that is display-only goes on
air. The MC-DPSK fixed frame is `v2::kConnectFrameCodewords`, the smallest R1/4 codeword count
that holds the frame (two static_asserts pin it), and every connect-specific site — the encoder's
fixed CONNECT, the receiver's fixed-CONNECT wait and decode, the half-open fallback span, the
connect airtime and retry interval — uses that constant, never `kDefaultFixedFrameCodewords`
(the data-frame default). **Why:** each handshake codeword costs 81 DBPSK symbols (1.73 s); the
destination callsign and a padding codeword made the two connect frames 16.6 s of a 24 s
handshake. **Test:** `FrameV2` (35-byte roundtrip, `TOTAL_CW` = 2), `StreamingMCDPSK` (CONNECT
loopback, half-open fallback), `UltraTncSimAudio` (two TNCs connect end to end).

### INV-HANDSHAKE-002: only a silent PING is decided early

**Rule:** a disconnected MC-DPSK frame is first released at `pingWindowSamples()` — exactly the
span PATH1's silence test reads (training skip + check window; `decodeCurrentFrame` asserts the
two agree). PATH1 may emit a PING there; any other outcome holds the frame, with
`sync_position_` untouched, until the full control requirement has arrived, so the CW0 /
fixed-CONNECT decode, the pre-LDPC guards and PATH2 never see a shorter slice than before.
**Why:** the verdict used to wait for a whole one-codeword slot (1.92 s) it never reads; PING and
PONG each cost 1.6 s more than their content. **Test:** `PingDetector`, `StreamingMCDPSK`,
`UltraTncSimAudio`; GUI gate PONG at 3.5 s instead of 6.7 s.

---

## Rate Selection Invariants (2026-11-09)

### INV-RATE-EVM-001: EVM decode thresholds belong to ONE PHY configuration

**Rule:** `evmDecodeThresholdLdpcDb` / `evmDecodeThresholdTurboDb` are the per-frame EVM SNR at
which each rung's frame decodes half the time, for the CURRENT pilot pattern, codeword geometry,
equalizer and LLR model, per data-frame FEC. Any change to one of those moves them: re-measure
with `measure_ack_fer --frame-log` (Good fits) AND the GUI gate's own burst frames (the only
source for turbo, BUG-HARNESS-TURBO-NO-RESULT) before trusting `ULTRA_RATE_EVM`. Never derive
them from the channel-SNR waterfall: EVM and channel SNR differ by fading- and rung-dependent
amounts, and only the EVM-domain threshold is channel-independent.
**Test:** `EvmRateSelector` (`test_thresholds_are_ordered`).

### INV-RATE-EVM-002: the outer-loop margin calibrates; it never tracks fades

**Rule:** the margin moves only by a group's residual between delivered frames and the success
its OWN frames' EVMs predicted, on fully measured groups. A loss the EVM predicted (a fade) leaves
it unchanged; making it react to raw losses (as an SNR-driven outer loop must) would make the
selector chase fades.
**Test:** `EvmRateSelector` (`test_margin_moves_on_calibration_error_only`).

### INV-RATE-EVM-003: compare a threshold with frames of another constellation only through the EVM model (2026-11-10)

**Rule:** a rung's EVM decode threshold is measured on that rung's own frames. When the selector
judges it from frames of a different constellation it uses `evmThresholdOnFramesDb(rung,
frame_mod)`, which carries the threshold through `ofdm/dd_evm_model.hpp` (the reading the frames'
constellation gives at the same true SNR). Never compare raw EVMs across constellations, and
keep the model's geometry equal to the transmitter's (CTest `DdEvmModel` checks it against a
brute-force run of the receiver's estimator).
**Why:** the decision-directed EVM reads high by a constellation-dependent amount near each
constellation's operating point: at a true 12 dB, QPSK reads 12.3, 16QAM 13.0, 32QAM 15.2. Raw
comparison left 32QAM R2/3 unreachable from 16QAM frames on channels where it loses no frame.
**Test:** `EvmRateSelector` (`test_thresholds_carried_between_constellations`), `DdEvmModel`.

### INV-RATE-RUNG-001: a ladder rung exists only on the PHY it was measured on (2026-11-10)

**Rule:** `CoherentRung::turbo_only` rows (16QAM R3/4, 32QAM R2/3) are holes unless OFDM data
frames are turbo (`ofdmDataFrameFec()`); every consumer of the ladder (connect pick, locally
enabled, calibration and class anchors) goes through `coherentRungUsableWithDataFec`. Canonical
rung indices stay below `kRungAuthorityStartupProbeFlag` (the tone-ACK command carries 4 bits).
**Why:** with LDPC data frames 16QAM R3/4 has a 20 % error floor on ITU Good and 32QAM was never
measured; `ULTRA_FAST=0` must stay the untouched baseline of the pending rig A/B.
**Test:** `WaveformPolicy` and `WaveformPolicyLdpc` (the same checks with `ULTRA_DATA_FEC=ldpc`).

### INV-RATE-PRICE-001: price only the anchors the sender will actually emit

**Rule:** the latent selector's candidate geometry charges the descriptor-switch full anchor only
when `flushBurstBuffer` sends one — chirp acquisition. Structure acquisition never anchors a
switch; pricing one there is a ~25 % phantom penalty that kept the controller off 16QAM R2/3.
**Test:** GUI gate arm C (`docs/EVM_RATE_SELECTION_2026_11_08.md` §4).

### INV-ACK-TURN-001: the ACK for a turn's last frame carries completed-turn provenance

**Rule:** every physical DATA turn whose end the receiver must ACK carries a CRC-protected
`PHYSICAL_BURST_END` on its last frame (EVERY OFDM and EVERY MC-DPSK turn, singletons included,
on EVERY transmit path: the burst encoder, the singleton flush and a timeout batch sent frame by
frame, `Connection::physicalTurnEndWireCopy`; OFDM single frames since 2026-11-12,
BUG-OFDM-SINGLETON-UNMARKED), and the receiver reports
`physical_turn_complete` for it BEFORE the frame callback, so
the ACK takes the completed-group path. An ACK without that provenance goes through
listen-before-ACK's evidence-novelty rule (INV-TONE-ACK-005), which is right only for genuinely
asynchronous ACKs (timer backstops, descriptor loss): it waits out a peer that is still on the air.
Never infer the end of an MC-DPSK turn from MORE_FRAG: it says the FILE continues. A frame decoded
at an accepted anchor proves a standalone turn only when a DUAL CHIRP found that anchor: the
frame-structure detector locks on any frame's LTS pair, a mid-burst member after a lost head included
(`armProvisionalBurstAirGate(..., proven_full_chirp = !used_structure_anchor)`,
BUG-STRUCTURE-MEMBER-STANDALONE).
**Test:** `StreamingFramePolicy` (`test_physical_burst_end_wire_policy`), `BurstStructureMemberAnchor`;
GUI gate AWGN@7 MC-DPSK.

### INV-GUI-LOOP-001: the GUI loop never waits on the display

**Rule:** `ultra_gui` drains RX audio and ticks the protocol from its render loop, so nothing in
that loop may block on the display: no vsync swap interval, no `SDL_RENDERER_PRESENTVSYNC`. The
loop paces itself (`main_gui.cpp`, ~60 Hz). SDL's Cocoa vsync waits on a CVDisplayLink that stops
while the display sleeps — an unattended station's receiver froze
(BUG-GUI-DISPLAY-SLEEP-STALL).
**Test:** `tools/gui_qso_scenario.sh` with the display asleep (`pmset displaysleepnow`, gate copy
without `caffeinate -d/-u`).

### INV-FAST-001: `ULTRA_FAST` sets defaults, never overrides

**Rule:** `fastProfileEnabled()` is consulted ONLY when the member knob is unset
(`ULTRA_BURST_ACQ`, `ULTRA_LATENT_ANCHORS`, `ULTRA_DATA_FEC`, `ULTRA_RATE_EVM`,
`ULTRA_OFDM_FLOOR_AWGN_DB`), so a component can be A/B'd inside the profile. Its members are
lockstep: both stations run the same profile. Since 2026-11-09 the profile is ON unless
`ULTRA_FAST=0`, which restores every member's pre-flip default (the rig A/B baseline arm). A test
that encodes a pre-flip contract pins it explicitly (ctest `ENVIRONMENT`, or the member's
`set...ForTesting`), never by relying on the process default.


### INV-SCO-001: in-burst members follow the session drift model, never their own estimate

**Rule:** a burst member is sliced at the fixed frame stride plus `ScoTracker::correctionSamples`
(the session clock-drift slope x the member's nominal distance from the group anchor). A member's
own LTS timing estimate is evidence for the model, never a re-slice of that member: on fading a
single faded frame's estimate is wrong, and per-frame re-slicing lost ~50 % of bursts (§14.25).
The slope is applied only when significant against the between-group spread (>= 2 groups, 3
standard errors), so a fading trend never moves a slice. The model belongs to the soundcard pair:
reset at session end, not on the per-turnaround decoder reset. `ULTRA_SCO_TRACK=0` measures but
never corrects.

### INV-SNR-PAPR-001: an SNR measured on one waveform is converted before judging another

**Rule:** the connect SNR is measured on the peer's MC-DPSK CONNECT frames. A transmitter that
scales every burst to the same PEAK (the hardware TX path; any real PA) sends OFDM data weaker in
average power, by the station's measured TX PAPR offset. Judge OFDM candidacy and its entry rate at
`measured - offset` (`selectLadderRungPaprCorrected`); never shift the MC-DPSK rungs, which were
measured on MC-DPSK. The offset comes from the station's own normalized bursts (0 when the TX
normalizes average power, as the default simulator does), is handed to `Connection` lock-free, and
is disabled with `ULTRA_PAPR_SNR_CORRECTION=0` on a bench whose noise tracks the signal level.
The prior used before the station has measured its own bursts is 1.0 dB with whitened control
frames (`peakNormalizedOfdmBelowMcDpskPriorDb`; 3.4 dB with `ULTRA_CONTROL_WHITEN=0`); re-measure
it from a `--tx-model hardware --record` TX recording whenever a burst's opening content changes.

### INV-OFDM-FRAME-001: a session's OFDM DATA frame says where it sits in its physical turn

**Rule:** with turbo data frames, a sender inside a session (ModemEngine has the session's addresses)
sends OFDM DATA frames in the compact layout (`protocol/ofdm_data_frame.hpp`: FLAGS SEQ POS LEN HCRC +
payload, CRC-32C seeded from the direction's callsign hashes), each with its true physical position
(index among the turn's DATA frames, and their count) set by `encodeBurstLight` per frame and 0/1 for a
frame sent on its own. Every capacity that sizes a session's OFDM DATA (`Connection` chunker and rung
sites, the SR-ARQ's fixed-frame accept) is `v2::ofdmSessionPayloadCapacity`, so a chunk always fits the
layout the encoder will use; pad frames (addressed away) keep the v2 layout and size. The receiver
accepts both layouts in one decode and rebuilds the v2 frame. A decoded member's position is used only
where the slicer put it (accumulated index == position index): then it proves a descriptorless group's N
(`noteBurstMemberPosition`: the group finalizes and acknowledges at N instead of being abandoned) and,
on the classic path, replaces the descriptorless air ceiling with the turn's real end.

**Why:** the v2 layout spent 19 bytes of a turbo frame on framing the codec already provides (24 % of a
QPSK R1/4 4-codeword frame), and no frame told the receiver how long its burst was: a group whose
descriptor and tail were lost could not be acknowledged even when its middle decoded.

### INV-DESC-KNOWN-001: a connected receiver decodes every descriptor it acquires, and uses what the protocol fixes

**Rule:** in a connected OFDM session the control-first decode of a one-codeword control frame never
skips on its LLR magnitude (frame-structure detection or an anchored search found it; the LDPC decode and
the CRC arbitrate). When that decode yields no control frame, the codeword is decoded once more as a
BURST_HEADER descriptor with its protocol-fixed information bits pinned
(`v2::burstDescriptorKnownBits`: magic, type, VERSION_V2 flags, the two hashes of
`kBurstDescriptorIdentity`, the pad bits) and accepted only as a CRC-valid BURST_HEADER. Anything that
changes a fixed descriptor field (its identity, flags or layout) must change the table with it - the
CTest `DescriptorKnownBits` compares the table with real descriptors bit for bit.

**Why:** at the OFDM floor a real descriptor's mean |LLR| is 0.6-1.5; the old 1.5 pre-screen lost 41 % of
them at ITU Good 5 dB (BUG-CONTROL-PRESCREEN-DROPS-DESCRIPTORS), and pinning the 82 fixed bits is a
shortened code worth ~2 dB on a fading channel, from the same air bits.

### INV-FRAME-WHITEN-001: nothing but the data sets an OFDM burst's peak

**Rule:** every burst is scaled to its loudest sample on a peak-limited transmitter, so no
deterministic content in a burst may peak above the data that follows. One-codeword OFDM control
frames (the BURST_HEADER descriptor leads every burst) are XORed with `v2::kControlFrameWhitening`
before LDPC encoding; the receiver restores them after decoding by their whitened magic (the LDPC
decode does not depend on the whitening), in `ControlUnwhiteningCodec` / `robustDecodeSingleCW` /
`decodeSingleCodeword`, before any header peek. The chirp-less burst's settling fill redraws any
symbol whose sample peak exceeds a typical data symbol's (10 dB over RMS). **Why:** unscrambled,
the descriptor's systematic header bits put one QPSK point on 34 of 59 carriers and its symbol
peaked 15.3 dB over the burst RMS against the data's own 12.3-13.3 dB, holding every OFDM burst
2.0-2.9 dB down; a fixed fill symbol at 13.1 dB cost chirp-less bursts up to 0.7 dB more.
**Test:** `ControlFrameWhitening` (both forms round-trip byte-exact, raw headers and data
codewords untouched, the codec wrapper, descriptor envelope peak 13.1 -> 10.3 dB, fill bound).
---

## Sample Position Invariants

### INV-POS-001: detectSync() Returns TRAINING Start, Not Data Start

**Rule:** `SyncResult.start_sample` points to where TRAINING symbols begin, not where data begins.

**Why:** Both MC-DPSK and OFDM need training symbols for channel estimation. The waveform's `process()` method expects to receive training+data, not just data.

**Frame layout:**
```
[CHIRP][GAP][CHIRP][GAP][TRAINING][REF/MORE-TRAINING][DATA...]
                        ^
                        start_sample points HERE
```

**Locations:**
- `src/waveform/mc_dpsk_waveform.cpp:114-137`
- `src/waveform/ofdm_chirp_waveform.cpp:141-157`

---

### INV-POS-002: Position Correction After CFO Estimation

**Rule:** Raw chirp peak positions must be corrected using estimated CFO before returning.

**Formula:**
```cpp
float up_correction = cfo_hz * cfo_to_samples;
corrected_up_pos = raw_up_pos + up_correction;
corrected_down_pos = raw_down_pos - up_correction;
```

**Why:** The correlation peak shifts with CFO. We correct the position to get the true chirp start.

**Location:** `src/sync/chirp_sync.hpp:459-461`

---

## IWaveform Interface Invariants

### INV-WAVE-001: Call Sequence Must Be Exact

**Rule:** The IWaveform usage pattern is:
```cpp
1. waveform->reset();                           // Clear state
2. waveform->detectSync(samples, result);       // Find preamble
3. waveform->setFrequencyOffset(result.cfo_hz); // Store CFO
4. waveform->process(samples_from_start);       // Demodulate
5. auto bits = waveform->getSoftBits();         // Get output
```

**Violations that will fail:**
- Calling `process()` before `detectSync()` - no sync position
- Calling `reset()` after `setFrequencyOffset()` - clears CFO
- Calling `getSoftBits()` before `process()` returns true - no data

---

### INV-WAVE-002: reset() Clears CFO State

**Rule:** `reset()` MUST clear `cfo_hz_` to prevent stale values from previous frames.

**Why:** If CFO isn't cleared, a frame with 30 Hz CFO followed by a frame with 0 Hz CFO would incorrectly use 30 Hz.

**Implementation:**
```cpp
void reset() {
    demodulator_->reset();
    synced_ = false;
    cfo_hz_ = 0.0f;      // MUST clear!
    last_cfo_ = 0.0f;    // MUST clear!
}
```

---

## LDPC Invariants

### INV-LDPC-001: Soft Bits Are Log-Likelihood Ratios

**Rule:** Soft bits passed to LDPC decoder are LLRs: positive = likely 0, negative = likely 1.

**Scale:** Typical range is [-10, +10]. Values outside this may indicate scaling issues.

**Sign convention:**
- LLR > 0 → bit is likely 0
- LLR < 0 → bit is likely 1
- |LLR| = confidence (higher = more certain)

---

### INV-LDPC-002: Codeword Size is Fixed at 648 Bits

**Rule:** All LDPC codewords are exactly 648 bits. Frame must contain integer number of codewords.

**Data bits by rate:**
- R1/4: 162 data bits per codeword
- R1/2: 324 data bits per codeword
- R2/3: 432 data bits per codeword
- R3/4: 486 data bits per codeword
- R5/6: 540 data bits per codeword

---

## Thread Safety Invariants

### INV-THREAD-001: Audio Feed Thread and Decode Thread Share Decoder Buffer

**Rule:** `StreamingDecoder` buffer/state is concurrently touched by:
- audio feed path (`feedAudio()`), and
- decode path (`processBuffer()`).

Access to shared decoder internals must stay synchronized through decoder-owned locks/atomics.

**Current implementation:** bounded circular buffer in `StreamingDecoder` with mutex + condition variable coordination.

**Hazard:** unsynchronized pointer/state updates can produce false syncs, dropped frames, or buffer corruption.

---

### INV-THREAD-002: Shared Runtime State Requires Atomic/Mutex Protection

**Rule:** cross-thread state (connection flags, TX/RX indicators, queue contents, meters) must use atomics or mutexes.

**Locations to audit:**
- `src/gui/modem/streaming_decoder.cpp` - shared RX decode state
- `src/gui/modem/modem_rx.cpp` - decode-thread lifecycle and mode switching
- `src/gui/audio_engine.cpp` - SDL callback queue access
- `src/gui/app.cpp` - simulator thread shared buffers/flags

---

## Testing Invariants

### INV-TEST-001: SNR Calculated from Active Samples Only

**Rule:** SNR must be calculated from the portion of the signal that contains actual transmission, not including silence.

**Why:** Including silence inflates noise power, giving wrong SNR.

**Correct:**
```cpp
float signal_power = computePower(signal, chirp_start, signal_end);
float noise_power = computePower(noise, chirp_start, signal_end);
float snr_db = 10 * log10(signal_power / noise_power);
```

---

### INV-TEST-002: Channel Must Be Applied to Entire Signal

**Rule:** HF channel simulation (fading, delay spread) must be applied to the ENTIRE signal including preamble, not just data.

**Why:** Real HF affects everything. Testing only data portion would be unrealistic.

---

### INV-TEST-003: the CLI is a ModemEngine station, not a harness (2026-11-10)

**Rule:** `ultra ptx`/`prx` drive `ModemEngine` only: the session state comes from
`ModemEngine::joinConnectedSession` (as in the GUI's `--monitor-ofdm`), receive goes through
`feedAudio` + `processRxBuffer`, and data frames are built by the connection's own builder
(`v2::makeFixedDataFrame` at the session's codeword count). No private `StreamingDecoder` setup,
no decoder features the GUI does not use, no file-splitting around the receivers.
**Why:** `cli_simulator` was retired for diverging from the production modem wrapper; the old CLI
diverged the same way and could not decode most frames (BUG-CLI-PTX-PRX-BROKEN). A frame `prx`
decodes is a frame a GUI station in that state decodes.
**Test:** `CliLoopback`.

### INV-GEOM-NARROW-001: the OFDM geometry follows the waveform inside ModemEngine (2026-11-10)

**Rule:** `ModemEngine::applyOfdmGeometryFor` (from `setWaveformMode` and `setConfig`) puts
OFDM_NARROW on `presets::narrowbandOFDM()` (FFT 2048, 21 carriers, 1250-1750 Hz chirp) and every
other waveform on the configured wideband geometry, keeping the data mode. Frontends do not pick
the geometry themselves.
**Why:** only `ultra_tnc` switched it; the GUI ran OFDM_NARROW on the wideband geometry and could
not talk to a TNC in narrowband (BUG-GUI-NARROW-WIDEBAND-GEOMETRY).
**Test:** `CliLoopback` (`ofdm_narrow`); `ptx -w narrow` occupies 1247-1755 Hz.

---

## Narrowband Invariants (coherent OFDM_NARROW, 2026-11-10)

### INV-NARROW-ANCHOR-001: every narrowband burst is chirp-anchored, and its receiver searches for the chirp
**Rule:** connected-OFDM acquisition (full-anchor search, `expectFullOFDMAnchorOnce`, light-reject
escalation, crater re-anchor, burst-group marker, arrival prediction, ACK-listen suppression,
connect-time full preamble) is gated on `isOFDMMode`, never on `OFDM_CHIRP`. Structure acquisition is
the one wideband-only part, on BOTH ends: the receiver runs it only for OFDM_CHIRP and the sender never
omits a narrowband chirp (`chirp_skip_allowed`). A `detectDataSync` window is valid from the LTS pair
plus one candidate offset.
**Why:** the wideband-only gates left coherent narrowband unanchored (BUG-NARROW-NO-ANCHOR); a fixed
3-symbol window minimum rejected every narrowband warm window (6656 < 3 x 2240).
**Test:** `measure_ack_fer --waveform narrow --config data4_full` (19/20 at 12-25 dB, 7.3 Hz); GUI gate
`ULTRA_FORCE_WAVEFORM=OFDM_NARROW`.

### INV-NARROW-TIMING-001: narrowband timing is derived from the rung's own layout and includes the anchor
**Rule:** `narrowOFDMFrameTiming(mod, rate, cw)` uses `recommendedPilotSpacing(mod, rate)` on the 21
carriers (control frames at QPSK R1/4); the ACK timeout budgets `kNarrowOFDMFullAnchorExtraMs` (2 x
1000 ms chirp + 2 x 100 ms gap), the descriptor, the window's frames, the turnaround and the receiver
SACK hold.
**Why:** a fixed pilot spacing timed QPSK R1/4 frames (83 symbols) as 74 and the anchor was not
budgeted: the ACK of an R1/4 burst arrived 0.4 s after the monitor window closed.
**Test:** `ConnectionPolicy` (`test_narrow_ofdm_timing_and_timeout`, coverage loop).

### INV-NARROW-ENTRY-001: an entry cap applies only to a session whose rate adapts
**Rule:** `capInitialOFDMRate` and the RX-authority first-group probe run only when
`Connection::entryRateCapsApply()` (wideband OFDM, or any OFDM mode with `ULTRA_RATE_ADAPT=1`).
**Why:** both start a session below its measured pick for the rate controller to climb from; a
narrowband session has no controller, so they pinned it at QPSK R1/2 for the whole session.
**Test:** GUI gate `ULTRA_FORCE_WAVEFORM=OFDM_NARROW` at AWGN 20 dB enters at the ladder's QPSK R3/4.

### INV-TONE-ACK-BAND-001: tone ACKs stay inside the session's channel
**Rule:** both stations set `ToneAckBand` from the negotiated waveform (`ModemEngine::setWaveformMode`);
OFDM_NARROW uses the narrowband plan (1375-1600 Hz, no diversity). The detector searches the shifted
tones and reports the carrier offset without the shift.
**Why:** the wideband plan (2400-2625 + 1075-1300 Hz) does not pass a 500 Hz filter
(BUG-NARROW-ACK-OUT-OF-BAND).
**Test:** `ToneBurstAckRoundTrip` (`test_narrowband_band_plan`); GUI gate `--rx-filter 1250-1750`.

### INV-RATE-EVIDENCE-001: a rung decision rests on evidence about the rung (2026-11-12)
**Rule:** (1) a climb of more than one rung needs EVM frames from at least two groups
(`EvmRateSelector::kMinGroupsForMultiRungClimb`): a group's frames are one fade realization;
(2) an escape-drop needs the latest round to have drawn an ACK (`escapeHasPeerEvidence`):
retransmission timeouts in a row say the channel or the return path is dead, which no rung fixes;
under the receiver authority the collapse escape (whose rounds are timeouts only) never acts, and
the stuck-frame escape counts the ACKs a frame was missing from (`heard_miss_count`), never its
resends into silence;
(3) the down-crossing is decided when a round ends without an ACK (its timeout), never while a
burst it sent is still on the air or awaiting its ACK.
**Why:** one crest group took 8PSK R2/3 -> 32QAM R2/3 and the next group failed whole; escapes
fired in a dead channel and committed a needless drop at the return (22 s handshake); an empty ACK
after two silent rounds fired the collapse escape on the trough's rounds; a crossing abandoned a
20 dB resend 6 s into its air (BUG-POST-CROSSING-OVERCLIMB, BUG-ESCAPE-STALE-AFTER-DEAD-CHANNEL,
2026-11-12 QSB seed 44).
**Test:** `EVMRateSelector`, `WaveformFollow` (`test_first_ofdm_group_climbs_one_rung`,
`test_escape_needs_the_peer_heard`).

### INV-THREAD-001: state shared between the engine's threads is atomic or locked (2026-11-12)
**Rule:** the receiving engine runs one decode thread per decoder (two under dual-listen), the GUI's
main thread (protocol tick, mode changes, TX) and its render reads. Anything two of them touch is an
atomic or is read and written under one mutex; a lazily built table is built by `std::call_once`
or a function-local static initializer, never behind a plain flag; a function-level `static` on a
decode, demap, sync or TX path is atomic (or `const`). The render thread reads decoder snapshots
only (`captureConstellationSnapshot`), never the waveform.
**Why:** a plain-flag lazy table let two threads resize the same vectors (heap corruption); plain
fields raced between the engine's thread and the decode thread; the render thread's waveform read
crashed the GUI (BUG-DECODER-THREAD-RACES, BUG-GUI-CONSTELLATION-RACE).
**Test:** `EngineConcurrency` under `-fsanitize=thread` (zero reports), `DualListenEngine`.

### INV-FILE-LEDGER-001: the file ledger rewinds exactly when the ARQ abandons frames (2026-11-11)
**Rule:** the file transfer re-queues its pending chunks (`requeuePendingChunks`) if and only if
the ARQ abandoned the in-flight frames that carry them - `SelectiveRepeatARQ::txAbortGeneration`
advanced across the ARQ reconfiguration - never on a prediction of it (a geometry change).
**Why:** the ARQ keeps in-flight frames across a change that leaves the frame payload capacity alone
(modulation only: 32QAM R2/3 <-> 16QAM R2/3) and retransmits them. A file layer rewound for it sends
the same bytes again as new frames and its sent/acked counts fall out of step with the ARQ's; the
next real abort then re-queues nothing and a chunk is lost while the sender reports the file
complete (BUG-FILE-REQUEUE-WITHOUT-ARQ-ABORT).
**Test:** `ConnectionAdaptive` `test_mod_only_move_keeps_the_file_ledger_with_the_arq`.

### INV-RX-LEVEL-001: the receiver reads no absolute level
**Rule:** every receive-side presence, noise or SNR decision is a ratio of two measured quantities
or relative to a measured noise reference, never a fixed RMS/power/amplitude: a station hears its
peer at whatever level its operator set (AF gain, USB audio level, sound-card gain), tens of dB
apart between stations. The one absolute test allowed is digital silence
(`signal_policy::kDigitalSilenceRms` = 1e-7, under a 24-bit converter step), and digital silence
is never a noise observation: a half-duplex radio's receive while its own transmitter is keyed, a
muted or unplugged input (the carrier-sense floor learned it and read busy for most of a session,
BUG-CCA-SILENCE-FLOOR).
**Why:** the simulator delivered every station's audio at its peer's transmit level, so gates
calibrated there (sync search 0.015-0.040 RMS, tone-ACK floor at 5 % of the transmit amplitude,
MC-DPSK burst tail 0.012, PING ceiling 0.16, carrier-sense floor 0.005, burst-erasure backstop
0.005, an idle SNR referenced to 0.3048) never failed a test and failed a real station: with both receivers 30 dB quieter a link
connected and moved no data, at -40 dB it never connected (BUG-RX-ABSOLUTE-LEVEL).
**Test:** `gui_qso_scenario.sh --rx-gain-db G` (-40 .. +10 dB must all pass);
`ToneBurstAckRoundTrip` (`test_detection_is_level_invariant`, `test_noise_only_never_reaches_the_floor`),
`ChannelIdleNoiseSNRCalibration` (`checkLevelInvariance`), `ChannelBusyDetector` (-70 .. -10 dBFS).

### INV-TX-LEVEL-001: every OFDM geometry leaves the modulator at the same per-symbol power
**Rule:** the modulator scales by `output_scale` x N/1024 x sqrt(59/Ncar) (Parseval with the 1/N
inverse FFT), so a symbol's average power does not depend on FFT size or carrier count; the wideband
geometry is the reference (factor 1, byte-identical).
**Why:** one scale for every geometry put narrowband data 10.5 dB under wideband in the modulator and,
after the 0.8 peak normalization, 6.7 dB under the transmitter's budget (BUG-NARROW-TX-LEVEL): ~9 dB
per narrowband rung. The level reference itself (`output_scale = 40`) is still hand-set
(BUG-TX-LEVEL-HANDSET).
**Test:** `ptx` narrowband vs wideband frame RMS (0.194 / 0.175); wideband `ptx` frame byte-identical.

### INV-MCDPSK-ORTHO-001: MC-DPSK carriers are orthogonal over a symbol
**Rule:** carrier spacing x symbol time is a whole number (>= 2 preferred) for any shipped MC-DPSK
layout. Production layouts: 8 carriers 500-2500 Hz at 1024 samples (6.1, -28.5 dB worst leakage) and
the narrowband preset (3.0, exact).
**Why:** a spacing of 1.42 symbol rates leaked -10 dB between carriers; the narrowband handshake never
decoded (BUG-NARROW-MCDPSK-CONNECT).
**Test:** `CliLoopback` (`narrow_connect`); reported SNR of a noiseless narrowband frame is ~48 dB.

### INV-MCDPSK-RATE-001: an MC-DPSK constellation change happens between bursts and only there
**Rule:** the sender changes its MC-DPSK data constellation (DBPSK <-> DQPSK) only between bursts:
on the receiver's command in a tone ACK, or one rung down before an ARQ tick that resends on timeout
(`Connection::maybeFallBackMcDpskRungOnTimeout`). Only the constellation changes; every MC-DPSK rung
shares sps, code rate and codewords per frame (`kMcDpskRungModulations`), so the ARQ geometry and
the frames already cut stay valid. The receiver never assumes the sender heard its command: a
connected burst's first frame whose first codeword fails is re-demodulated at the other
constellation before any false-lock or PING verdict (`retryMcDpskFirstFrameConstellation`), and the
first-frame look-ahead covers one DBPSK codeword (`MCDPSKWaveform::getMinSamplesForFrame`).
**Why:** a lost ACK would otherwise leave the two ends on different constellations with no way
back. A mid-burst change would need per-codeword trial decoding (continuation frames carry no
preamble), and changing the codeword count aborts in-flight frames (`setFixedFrameGeometry`).
**Test:** `StreamingMCDPSK` (`DQPSK burst at a DBPSK receiver` and the reverse; disabling the retry
fails it), `ConnectionPolicy` `test_mcdpsk_rate_controller`.

### INV-MCDPSK-FRAME-001: a compact MC-DPSK DATA frame's length is fixed for the session
**Rule:** in a connected MC-DPSK session every DATA frame is ONE turbo codeword of exactly the
session's codewords x 648 air bits (`protocol/mcdpsk_frame`), at any constellation; its CRC-32C is
seeded from the direction's callsign hashes (the sender takes them from the v2 frame, the receiver
from `setMcDpskSessionAddresses`), and the receiver rebuilds the v2 DATA frame for the protocol.
Control frames keep the v2 codeword chain. The receiver decodes a burst at fixed frame positions past
a failure, at most the sender's window (`mcDpskWindowSizeForTiming`), and hands the chirp search back
the first position of any trailing failures.
**Why:** a frame whose length lives in its own first codeword makes one failure lose the rest of the
burst; a fixed length lets the end-of-turn frame still arrive, so the sender gets a SACK with the hole
instead of waiting out its timeout. The window cap and the search hand-back keep the decoder from
eating the next burst, which the sender queues back to back.
**Test:** `StreamingMCDPSK` (`compact burst, middle frame lost`), `MCDPSKFrame`.

### INV-MCDPSK-TAIL-001: a burst continuation trusts a decoded frame's flags over a power test
**Rule:** a continuation never runs the in-band burst-tail test on a slot the previous frame proved: a
CRC-valid DATA frame with MORE_FRAG and without PHYSICAL_BURST_END means the sender's next frame is on
the air (the marker is stamped on the last DATA frame of every MC-DPSK turn,
`shouldStampPhysicalBurstEnd`, `Connection::physicalTurnEndWireCopy`). The test (window power against
the inter-chirp gap's noise, `inBandPowerReadsAsNoise`) decides only after a lost (compact) or partial
(v2) frame, which may have been the turn's last. State: `StreamingDecoder::mc_burst_next_slot_proven_`,
set by `startMCDPSKBurstContinuation(..., next_slot_proven)` and by each continuation frame.
**Why:** the reference is the 60 ms centre of one 100 ms gap (7.6 % spread, threshold 1.23x): a -5 dB
frame (1.32x the noise) read as noise in about one burst out of five and a static crash in the gap
ended bursts at any SNR; each false end lost the rest of the burst and cost a 25-45 s timeout
(BUG-MCDPSK-FALSE-TAIL).
**Test:** `DualListenEngine` (compact) and `DualListenEngineV2GapCrash` (v2): a static crash in the
gap, 3 of 3 frames.

### INV-MCDPSK-HANDSHAKE-001: handshake frames ride the control profile on both ends
**Rule:** CONNECT, CONNECT_ACK and CONNECT_NAK go out at the MC-DPSK control profile (DBPSK R1/4,
`streaming_control_profile::profileForMCDPSK`; `StreamingEncoder::encodeFrame`,
`isHandshakeNegotiationFrameBytes`, #72) whatever the session's data constellation, and a receiver
that waits for a fixed CONNECT sizes and demodulates it at that profile too, connected or not
(`StreamingDecoder::mc_handshake_profile_frame_`), restoring its own constellation afterwards.
**Why:** after a lost CONNECT_ACK the responder is already on its session constellation while the
initiator, unconnected, can only repeat the CONNECT at DBPSK; a responder demodulating the repeat at
DQPSK never replayed its CONNECT_ACK and the connection failed after ten attempts
(BUG-MCDPSK-CONNECT-AFTER-DQPSK-PICK; narrowband twin BUG-NARROW-MCDPSK-CONNECT).
**Test:** `DualListenEngine` (connected DQPSK responder decodes a repeated CONNECT).

### INV-FOLLOW-001: a waveform crossing switches the modem's waveform before its data mode
**Rule:** with `ULTRA_WAVEFORM_FOLLOW`, a session crosses between MC-DPSK and wideband OFDM only
through `Connection::applyWaveformCrossing`, which fires the crossing callback (modem: waveform first,
then data mode and frame geometry, `ModemEngine::crossConnectedWaveform`) BEFORE the data-mode
callback; `StreamingDecoder::setMode` to a non-OFDM mode cancels a queued connected-OFDM
reconfiguration. The command is a session rung word in the tone ACK (1..10 OFDM, 11/12 MC-DPSK); the
receiver crosses on the tick after that ACK leaves, the sender when it reads it. An up-crossing is on
probation until the first OFDM burst (docs/WAVEFORM_FOLLOW_DESIGN_2026_11_11.md).
**Why:** the decoder applies a connected-OFDM reconfiguration later (top of `processBuffer`) while a
switch to MC-DPSK takes effect at once: a data-mode call made while the modem was still OFDM would put
the decoder back into OFDM after the switch. And a crossing applied inside the ARQ's ACK callback
would reconfigure the ARQ in the middle of its own operation.
**Corollaries:** follow needs the receiver authority (`ULTRA_RX_RATE_AUTHORITY`, default on): the
commands are its rung bits, and only an authority sender reads them and ends its probation, so
`waveformFollowActive()` requires it. A down-crossing restarts the channel evidence (SNR pool, the
connection's physical measurement and the decoder's physical ring): the way back up is decided on
what the MC-DPSK spell measures, never on the channel that collapsed (BUG-FOLLOW-STALE-EVIDENCE).
The way back up reads only the burst being acknowledged: pool readings younger than this end's
previous burst verdict minus one ACK airtime, capped at the window span (`mcDpskBurstSpanMs`) - never
a fixed span, which holds earlier bursts when the sender's are shorter than its window. A fallback
spell shortens bursts on the TRANSMIT side only (`setTxInFlightCap`, `followFallbackSmallWindowMs`):
the burst length is the sender's choice and the receiver acknowledges at the burst's marked last
frame, so a receive window must never shrink with it (the two ends' spells age differently). The
pick's physical bound comes from the same frames (each reading carries its frame's physical SNR,
`ConnectSnrPool::physicalMeanDb`), never from the decoder's spell-wide ring. "Its frame's" is
literal: a compact burst continuation has no training span and measures its own data span
(`updateDataSpanPhysicalSNR`); a frame that cannot measure publishes nothing - never the burst's
first-frame snapshot again (BUG-MCDPSK-PHYSICAL-REPUBLISHED: one fade state read as three). A frame whose
decoder never measured a fading index carries none (NAN), never 0.0: the spell keeps the session's
measured channel class (BUG-FOLLOW-SPELL-FADING-AWGN) - the fading POOLED over the OFDM spell at the
down-crossing (`follow_spell_fading_`, shrunk by its own cluster count), never the last frame's single
reading (BUG-FOLLOW-SPELL-CLASS-ONE-FRAME: 0.21 shrunk to AWGN stranded a 5 dB session on MC-DPSK). A repeat of the ACK that
launched an up-crossing, heard while the first OFDM group is on air, is rejected in the monitor's
acceptance predicate (`isCrossingLaunchAckCopy`): a detection delivered to `onToneBurstAck` has
already disarmed the monitor, so consuming it there loses the group's real ACK.
**Test:** `WaveformFollow` (`test_waveform_follow_*`, own binary at the production defaults:
ConnectionAdaptive pins the authority off), `ConnectionPolicy` (word codec, gate, revert window),
`StreamingMCDPSK` (`runCompactBurstPhysicalPerFrame`).

---

### INV-DUAL-LISTEN-001: a listener's failures never reach the protocol; its successes switch first (2026-11-11)

**Rule:** with `ULTRA_WAVEFORM_FOLLOW=2` the engine's second decoder (the listener, on the wideband
waveform the session is not running) delivers only frames and groups that decoded; every failure
report (MC-DPSK frame lost, anchored burst without a group, burst outcome unknown, descriptor, data
sync, PING, decode-failure diagnostics) comes from the primary alone (`wireDecoderRoleCallbacks`,
`wireDecoderCoreCallbacks`). A listener's successful delivery is preceded by the waveform report
(`noteListenerDelivery`), so the connection crosses to that waveform and the decoders flip before the
frame is processed. Decoder threads are bound to slots (objects), never to roles. The listener runs
from the moment the session exists (`connected_`), not from handshake confirmation: a responder
confirms on the first frame it decodes from the initiator, and when the session started on a waveform
the channel cannot carry, that frame is the initiator's follow crossing, which only the listener
hears (BUG-LISTENER-WAITS-FOR-CONFIRMATION). A decoder that BECOMES the MC-DPSK listener by a flip
(the session crossed up to OFDM) is configured exactly as a new one (`configureListener`): the
down-crossing target ROBUST_MID, never the session's own MC-DPSK profile - a session that connected on
Robust (DQPSK, 4 codewords) otherwise slices every fallback burst wrong and never follows the sender
down (BUG-LISTENER-KEEPS-SESSION-PROFILE). A listener never learns idle noise
(`setIdleNoiseLearning(false)`): it cannot lock the session waveform's bursts, so it would learn the
peer's traffic as noise and carry that floor into its next role as primary
(BUG-PHYSICAL-REF-IDLE-FLOOR-CONTAMINATED).
**Why:** the MC-DPSK chirp and the OFDM anchor chirp are identical, so each decoder locks on the
other's bursts and fails them: an OFDM listener would fire the anchored-burst backstop ACK on every
MC-DPSK frame, an MC-DPSK listener would feed the constellation controller a loss per OFDM anchor.
A frame processed before the switch would land in the wrong ARQ geometry and ACK semantics.
**Test:** `DualListenEngine` (both frames delivered, each after its report, receiver already on its
waveform; the control without dual-listen misses the MC-DPSK frame; a Robust session hears the
ROBUST_MID fallback after following the sender up), `WaveformFollow` dual-listen
cases (receiver commands but does not switch; a repeat ACK carries the same command; it follows what
arrives; a missed up-crossing backs off).

---

## Modulation Invariants

### INV-MOD-001: DQPSK Uses Previous Symbol as Reference

**Rule:** Differential QPSK encodes data as phase CHANGE from previous symbol, not absolute phase.

**Decode:** `data = current_phase - previous_phase`

**Why this is robust:** Immune to constant phase offset from channel. Only needs phase to be stable between adjacent symbols.

---

### INV-MOD-002: Pilot Symbols Required for Coherent Modulation

**Rule:** Coherent modes (QPSK, 16QAM, 32QAM) REQUIRE pilot symbols for channel estimation. Differential modes (DQPSK, D8PSK) do NOT.

**OFDM_COX:** Supports both, automatically inserts pilots for coherent modes.
**OFDM_CHIRP:** Hardcoded to DQPSK, no pilots needed.
**MC-DPSK:** Always DQPSK, no pilots.

---

## Summary Checklist

Before submitting code changes, verify:

- [ ] CFO from chirp is not overwritten by training estimation
- [ ] `setFrequencyOffset()` called before `process()`
- [ ] CFO simulation uses Hilbert transform (if applicable)
- [ ] Complex correlation used for chirp detection
- [ ] `start_sample` points to training start
- [ ] `reset()` clears CFO state
- [ ] Thread-shared variables are protected
- [ ] SNR calculated from active samples only
- [ ] All tests pass with CFO = 0 AND CFO = ±30 Hz
