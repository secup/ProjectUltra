# Refactoring Progress Tracker

**Purpose:** Track progress on the architecture refactoring plan. This prevents re-doing work after context loss.

**Source Plan:** `~/.claude/plans/eager-percolating-naur.md`

**Last Updated:** 2026-01-27

---

## Overall Progress: ~40% Complete

```
Phase 1: Core Interfaces     [####------] 40%
Phase 2: Waveform Impl       [########--] 80%
Phase 3: ModemEngine Refactor[##--------] 20%
Phase 4: Sync Methods        [----------]  0%
Phase 5: Configuration       [----------]  0%
Phase 6: Bug Fixes           [####------] 40%
```

---

## Phase 1: Core Interfaces (Foundation)

### 1.1 IWaveform Interface
- [x] Create `src/waveform/waveform_interface.hpp`
- [x] Define SyncResult struct
- [x] Define WaveformCapabilities struct
- [x] Define IWaveform abstract class
- [x] Methods: detectSync, process, getSoftBits, reset
- [x] Methods: setFrequencyOffset, isSynced, hasData
- [x] Methods: generatePreamble, modulate (TX)
- [ ] Methods: getStatusString, getCarrierCount (GUI support)

**Status:** ✅ MOSTLY COMPLETE
**Files:** `src/waveform/waveform_interface.hpp`
**Commit:** `ed32e05`

### 1.2 ISyncMethod Interface
- [ ] Create `src/sync/sync_interface.hpp`
- [ ] Define ISyncMethod abstract class
- [ ] Methods: detect, generatePreamble, estimateCFO

**Status:** ❌ NOT STARTED
**Notes:** Low priority - waveforms currently own their sync methods internally

### 1.3 ICodec Interface
- [ ] Create `src/fec/codec_interface.hpp`
- [ ] Define ICodec abstract class
- [ ] Methods: encode, decode, setRate, getRate

**Status:** ❌ NOT STARTED
**Notes:** Low priority - LDPC codec works fine as-is

---

## Phase 2: Waveform Implementations

### 2.1 Concrete Waveforms

#### MCDPSKWaveform
- [x] Create `src/waveform/mc_dpsk_waveform.hpp`
- [x] Create `src/waveform/mc_dpsk_waveform.cpp`
- [x] Implement detectSync with chirp detection
- [x] Implement process with demodulation
- [x] Implement getSoftBits
- [x] Implement setFrequencyOffset
- [x] CFO correction working
- [x] Tested with test_iwaveform

**Status:** ✅ COMPLETE
**Commit:** `ed32e05`, `48e6271`

#### OFDMChirpWaveform
- [x] Create `src/waveform/ofdm_chirp_waveform.hpp`
- [x] Create `src/waveform/ofdm_chirp_waveform.cpp`
- [x] Implement detectSync with chirp detection
- [x] Implement process with OFDM demodulation
- [x] Implement getSoftBits (fixed loop issue)
- [x] Implement setFrequencyOffset
- [x] CFO correction working
- [x] Tested with test_iwaveform

**Status:** ✅ COMPLETE
**Commit:** `ed32e05`, `84bb563`

#### OFDMCoxWaveform (was OFDM_NVIS)
- [x] Create `src/waveform/ofdm_cox_waveform.hpp`
- [x] Create `src/waveform/ofdm_cox_waveform.cpp`
- [x] Implement detectSync with Schmidl-Cox
- [x] Implement process
- [x] Implement getSoftBits
- [ ] Implement setFrequencyOffset (needs verification)
- [ ] CFO correction verified
- [ ] Tested with test_iwaveform

**Status:** ⚠️ PARTIAL - CFO not verified
**Commit:** `20a2643`

#### OTFSWaveform
- [ ] Create `src/waveform/otfs_waveform.hpp`
- [ ] Create `src/waveform/otfs_waveform.cpp`
- [ ] Wrap existing OTFS modulator/demodulator

**Status:** ❌ NOT STARTED
**Notes:** Deferred until core refactor is complete

### 2.2 WaveformFactory
- [x] Create `src/waveform/waveform_factory.hpp`
- [x] Create `src/waveform/waveform_factory.cpp`
- [x] Implement create() method
- [x] Support MC_DPSK, OFDM_CHIRP, OFDM_COX

**Status:** ✅ COMPLETE
**Commit:** `20a2643`

---

## Phase 3: Refactor ModemEngine

### 3.1 Simplify ModemEngine
- [ ] Replace 8 modulator/demodulator pointers with single IWaveform
- [ ] Remove waveform-specific if-else chains
- [ ] Use WaveformFactory for waveform creation

**Status:** ❌ NOT STARTED
**Blocked by:** RxPipeline bugs (BUG-002)

### 3.2 Create WaveformState Class
- [ ] Create `src/gui/modem/waveform_state.hpp`
- [ ] Encapsulate mode state machine
- [ ] Methods: getPhase, getActiveMode, getModeForTx

**Status:** ❌ NOT STARTED

### 3.3 Refactor TX Path
- [x] TX uses IWaveform for OFDM_CHIRP (partial)
- [ ] TX uses IWaveform for all modes
- [ ] Remove waveform-specific TX code

**Status:** ⚠️ PARTIAL

### 3.4 Refactor RX Path with RxPipeline
- [x] Create `src/gui/modem/rx_pipeline.hpp`
- [x] Create `src/gui/modem/rx_pipeline.cpp`
- [x] Implement process() with template method
- [ ] Fix chirp detection issues (BUG-002)
- [ ] Integrate into ModemEngine
- [ ] Replace processRxBuffer_* methods

**Status:** ⚠️ EXISTS BUT BUGGY
**Blocked by:** BUG-002

---

## Phase 4: Sync Method Implementations

### 4.1 Wrap Existing Sync Methods
- [ ] Create ChirpSyncMethod wrapper
- [ ] Create SchmidlCoxMethod wrapper
- [ ] Create BarkerSyncMethod wrapper
- [ ] Create ZadoffChuMethod wrapper

**Status:** ❌ NOT STARTED
**Notes:** Low priority - current internal ownership works

---

## Phase 5: Configuration System

### 5.1 Centralized Config
- [ ] Create `src/config/modem_config.hpp`
- [ ] Define ChirpConfig, OFDMConfig structs
- [ ] YAML file support (optional)

**Status:** ❌ NOT STARTED
**Notes:** Nice to have, not blocking

---

## Phase 6: Bug Fixes During Refactor

### 6.1 Fix CFO Handling
- [x] MC-DPSK CFO correction
- [x] OFDM_CHIRP CFO correction
- [ ] OFDM_COX CFO verification

**Status:** ⚠️ MOSTLY COMPLETE

### 6.2 Acquisition Thread Routing
- [ ] Detect frame type from preamble
- [ ] Route to correct decoder

**Status:** ❌ NOT STARTED
**Tracked as:** BUG-003

### 6.3 Thread Safety Audit
- [ ] Audit shared variables
- [ ] Add proper synchronization

**Status:** ❌ NOT STARTED

### 6.4 Disconnect ACK Waveform
- [x] Fixed: Uses disconnect_waveform_

**Status:** ✅ COMPLETE
**Commit:** `cdfacbb`

---

## Test Tool Status

| Tool | Status | Future |
|------|--------|--------|
| **test_iwaveform** | ✅ WORKING | **PRIMARY** - Will become official test tool (rename planned) |
| **cli_simulator** | ❌ CFO BROKEN | **PRIMARY** - Full protocol testing (needs BUG-001 fix) |
| test_hf_modem | ⚠️ LEGACY | **DEPRECATE** - Reference only, will be removed |
| profile_acquisition | ⚠️ UNKNOWN | TBD |

### Testing Strategy

**Official test tools (keep and improve):**
1. `test_iwaveform` → Rename to `test_modem` or `modem_test`
   - Tests IWaveform interface directly
   - CFO simulation works correctly (Hilbert transform)
   - Single-frame and multi-frame testing
   - All waveforms, channels, SNR, CFO combinations

2. `cli_simulator` → Fix CFO, keep for protocol testing
   - Full protocol flow (PING → CONNECT → DATA → DISCONNECT)
   - Two-station simulation
   - Needs BUG-001 fix for CFO testing

**Legacy (to be removed):**
- `test_hf_modem` - Old approach, reference only

---

## Documentation Status

| Document | Status | Purpose |
|----------|--------|---------|
| `INVARIANTS.md` | ✅ COMPLETE | Critical rules |
| `KNOWN_BUGS.md` | ✅ COMPLETE | Bug tracking |
| `CHANGELOG.md` | ✅ COMPLETE | Change history |
| `REFACTOR_PROGRESS.md` | ✅ COMPLETE | This file |
| `MODEM_ENGINE_ARCHITECTURE.md` | ✅ COMPLETE | ModemEngine internals |
| `DUAL_CHIRP_CFO_ANALYSIS.md` | ✅ COMPLETE | CFO handling |
| `TESTING_METHODOLOGY.md` | ✅ COMPLETE | Test approach |
| `CFO_CORRECTION_FLOW.md` | ✅ COMPLETE | CFO code paths |
| `PROTOCOL_V2.md` | ✅ COMPLETE | Protocol spec |
| `GUI_ARCHITECTURE.md` | ✅ COMPLETE | GUI structure |
| `AUDIO_SYSTEM.md` | ✅ COMPLETE | Audio I/O |
| `CONFIGURATION_SYSTEM.md` | ✅ COMPLETE | Settings/config |
| `BUILD_SYSTEM.md` | ✅ COMPLETE | CMake/deps |
| `ADDING_NEW_WAVEFORM.md` | ✅ COMPLETE | How-to guide |

**All documentation complete!** Ready for any future development.

---

## Blocking Issues

These must be fixed before continuing refactor:

1. **BUG-002: RxPipeline chirp detection** - Blocks ModemEngine integration
2. **BUG-003: Acquisition routing** - Blocks multi-waveform RX

---

## Next Steps (Priority Order)

1. [ ] Debug RxPipeline chirp detection (BUG-002)
2. [ ] Verify OFDM_COX CFO correction
3. [ ] Add OFDM_COX to test_iwaveform
4. [ ] Fix cli_simulator CFO (BUG-001)
5. [ ] Integrate RxPipeline into ModemEngine
6. [ ] Replace processRxBuffer_* methods
7. [ ] Create WaveformState class
8. [ ] Simplify ModemEngine

---

## Commits Reference

| Commit | Description |
|--------|-------------|
| `ed32e05` | Add IWaveform interface and concrete waveform implementations |
| `20a2643` | Rename OFDM_NVIS to OFDM_COX and integrate waveform factory |
| `e5baa72` | Integrate RxPipeline into ModemEngine for connected mode RX |
| `ffc979c` | WIP: Update RxPipeline and test_hf_modem for IWaveform interface |
| `398cbd0` | WIP: Debug RxPipeline chirp detection issues |
| `48e6271` | Fix MC-DPSK CFO correction for training/reference samples |
| `84bb563` | Add OFDM_CHIRP support to test_iwaveform with CFO correction |
| `cdfacbb` | Fix disconnect ACK waveform and improve GUI status display |
