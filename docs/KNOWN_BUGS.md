# Known Bugs and Issues

**Purpose:** Track all known bugs, limitations, and issues that need fixing. This prevents context loss from causing us to forget about problems or re-discover them.

**Format:** Each bug must have: ID, Status, Description, Impact, Workaround (if any), and Fix Plan.

---

## Active Bugs

### BUG-001: cli_simulator CFO Simulation is Broken

**Status:** OPEN - HIGH PRIORITY
**Discovered:** 2026-01-27
**Location:** `tools/cli_simulator.cpp`

**Description:**
The `tx_cfo_hz` parameter only shifts the chirp preamble frequency, NOT the OFDM/DPSK data. This means CFO testing through cli_simulator is invalid - the chirp detector sees CFO but the demodulator sees 0 Hz offset.

**Root Cause:**
```cpp
// In cli_simulator, tx_cfo_hz is passed to:
chirp_cfg.tx_cfo_hz = tx_cfo_hz;  // Chirp gets CFO ✓
// But NOT to:
ofdm_modulator_cfg.tx_cfo_hz     // OFDM data has NO CFO ✗
```

**Impact:**
- cli_simulator cannot be used to verify CFO correction
- Full protocol testing with CFO is not possible
- Only test_iwaveform.cpp has correct CFO simulation (via Hilbert transform)

**Workaround:**
Use `test_iwaveform` for CFO testing instead of `cli_simulator`.

**Fix Plan:**
Option A: Apply Hilbert-based CFO shift in `applyChannel()` function
Option B: Pass `tx_cfo_hz` to all modulators consistently

---

### BUG-002: RxPipeline Chirp Detection Fails

**Status:** OPEN - HIGH PRIORITY
**Discovered:** 2026-01-26
**Location:** `src/gui/modem/rx_pipeline.cpp`

**Description:**
RxPipeline was created to provide a clean IWaveform-based RX path, but chirp detection fails when used through the pipeline. Works fine when IWaveform is used directly (test_iwaveform.cpp).

**Evidence:**
- Commits `398cbd0` and `ffc979c` are marked "WIP" for this issue
- test_iwaveform works 100% by calling IWaveform directly
- RxPipeline integration in ModemEngine fails

**Root Cause:**
Unknown - needs investigation. Possibly:
- Buffer management issues
- State not being reset properly between frames
- Timing/threading issues when integrated into ModemEngine

**Impact:**
- ModemEngine cannot use the new IWaveform interface for RX
- Still using legacy `processRxBuffer_*` methods
- Refactor is blocked on this

**Workaround:**
ModemEngine uses old code paths. IWaveform works for isolated testing.

**Fix Plan:**
1. Add detailed logging to RxPipeline
2. Compare buffer contents between working (test_iwaveform) and failing (RxPipeline) paths
3. Identify where they diverge

---

### BUG-003: ModemEngine Routes ALL Chirp Frames to MC-DPSK

**Status:** OPEN - MEDIUM PRIORITY
**Discovered:** 2026-01-27
**Location:** `src/gui/modem/modem_rx_decode.cpp`

**Description:**
The acquisition thread detects chirp preambles but doesn't know if the frame is MC-DPSK or OFDM_CHIRP. It routes everything through `rxDecodeDPSK()`, which fails for OFDM_CHIRP frames.

**Root Cause:**
No frame type indicator in the chirp preamble. Both MC-DPSK and OFDM_CHIRP use the same dual-chirp sync.

**Impact:**
- Cannot receive OFDM_CHIRP frames through ModemEngine
- Only works in test_iwaveform where waveform type is explicitly specified

**Workaround:**
Use test_iwaveform for OFDM_CHIRP testing. In production, use MC-DPSK for connection (it's more robust anyway).

**Fix Plan:**
Options:
A) Add frame type indicator after chirp (e.g., short tone burst)
B) Try MC-DPSK first, if decode fails try OFDM_CHIRP
C) Detect from training symbol structure (different between MC-DPSK and OFDM)

---

### BUG-004: OFDM_COX CFO Correction Not Verified

**Status:** OPEN - MEDIUM PRIORITY
**Discovered:** 2026-01-27
**Location:** `src/waveform/ofdm_cox_waveform.cpp`

**Description:**
OFDM_COX (Schmidl-Cox sync) has CFO estimation code, but it hasn't been tested with the IWaveform interface. The old ModemEngine code path may work, but the new interface hasn't been verified.

**Impact:**
- Real-world OFDM_COX usage (17+ dB) may fail with radio frequency offset
- Only AWGN with CFO=0 is verified

**Workaround:**
Use OFDM_CHIRP for 10-17 dB range (CFO verified).

**Fix Plan:**
1. Add OFDM_COX to test_iwaveform
2. Test with CFO = ±30, ±50 Hz
3. Verify Schmidl-Cox CFO estimation is applied correctly

---

## Fixed Bugs (Reference)

### BUG-F001: MC-DPSK CFO Correction for Training Samples
**Status:** FIXED - 2026-01-27
**Commit:** `48e6271`
**Details:** See docs/CHANGELOG.md

### BUG-F002: Chirp Detection Position Shift with CFO
**Status:** FIXED - 2026-01-26
**Commit:** `b2592a0`
**Details:** See docs/CHANGELOG.md

### BUG-F003: OFDM_CHIRP Training Symbol CFO Override
**Status:** FIXED - 2026-01-26
**Commit:** `f1fce06`
**Details:** See docs/CHANGELOG.md

---

---

## Planned Improvements

### IMP-001: Rename test_iwaveform

**Status:** TODO
**Priority:** LOW

**Description:**
`test_iwaveform` will become the primary/official modem test tool. The name should reflect this importance.

**Suggested names:**
- `test_modem` - Simple, clear
- `modem_test` - Matches convention
- `ultra_test` - Project-branded

**Action items:**
1. Rename `tools/test_iwaveform.cpp` → `tools/test_modem.cpp`
2. Update CMakeLists.txt
3. Update regression_matrix.sh
4. Update all documentation references

---

### IMP-002: Remove Legacy test_hf_modem

**Status:** TODO - After refactor complete
**Priority:** LOW

**Description:**
`test_hf_modem` is legacy code kept for reference. Once `test_iwaveform` (renamed) is fully verified and the refactor is complete, remove it.

**Blocked by:** Refactor completion, full test coverage in test_iwaveform

---

## Limitations (Not Bugs)

### LIM-001: Poor HF Channel Performance
**Description:** OFDM modes fail on poor HF channels (2ms delay spread) because multipath exceeds cyclic prefix.
**Not a bug:** This is a fundamental limitation of OFDM with current CP length.
**Mitigation:** Use MC-DPSK for poor channels (90-100% success).

### LIM-002: Schmidl-Cox Requires 17+ dB
**Description:** Schmidl-Cox sync detection needs ~17 dB SNR minimum.
**Not a bug:** This is expected behavior for the algorithm.
**Mitigation:** Use chirp sync (MC-DPSK, OFDM_CHIRP) for lower SNR.

### LIM-003: Single Carrier DPSK Floor at -5 dB
**Description:** Even single-carrier DPSK fails below -5 dB (20-40% success).
**Not a bug:** Approaching theoretical limits.
**Mitigation:** -3 dB is the reliable threshold, use higher rate codes above 0 dB.

---

## Bug Tracking Process

When discovering a new bug:
1. Add entry here with unique ID (BUG-XXX)
2. Set status: OPEN, IN_PROGRESS, or FIXED
3. Document root cause if known
4. Document workaround if available
5. When fixed, move to "Fixed Bugs" section with commit hash

When fixing a bug:
1. Update status to FIXED
2. Add commit hash
3. Add entry to docs/CHANGELOG.md with full details
4. Move entry to "Fixed Bugs" section
