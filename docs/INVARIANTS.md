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

### INV-CFO-005: The chirp guard may only fall back to a MEASURED CFO (2026-11-08)

**Rule:** `limitConnectedCFODrift` clamps a chirp to the tracked value only when
`CFOTracker::established()` (the value came from a measurement since the last reset). An
unestablished tracker accepts the chirp; the first OFDM frame's LTS stage corrects a phantom.

**Why:** the default 0 Hz is not information. Falling back to it turned a correct 7.3 Hz chirp
into a 7.3 Hz error on 27/48 Good@20 frames — and looked correct for months because the simulator's
truth was 0 Hz. Never judge "established" from |CFO| > ε either (a true 0 Hz is established too).

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

---

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

### INV-RATE-PRICE-001: price only the anchors the sender will actually emit

**Rule:** the latent selector's candidate geometry charges the descriptor-switch full anchor only
when `flushBurstBuffer` sends one — chirp acquisition. Structure acquisition never anchors a
switch; pricing one there is a ~25 % phantom penalty that kept the controller off 16QAM R2/3.
**Test:** GUI gate arm C (`docs/EVM_RATE_SELECTION_2026_11_08.md` §4).

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
(`ULTRA_BURST_ACQ`, `ULTRA_LATENT_ANCHORS`, `ULTRA_DATA_FEC`, `ULTRA_RATE_EVM`), so a component can
be A/B'd inside the profile. Its members are lockstep: both stations run the same profile.

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
