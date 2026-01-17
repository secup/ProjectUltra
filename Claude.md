# Claude Code Guidelines for ProjectUltra

> **STOP AND READ THIS ENTIRE DOCUMENT BEFORE MAKING ANY CHANGES TO THIS CODEBASE.**

## MANDATORY RULES - NO EXCEPTIONS

**These rules are NON-NEGOTIABLE. Claude MUST strictly follow every rule in this document without deviation. There are no circumstances where these rules can be bypassed, ignored, or worked around.**

---

## Project Classification: CRITICAL SOFTWARE

**ProjectUltra is an HF modem software in early development. This is CRITICAL infrastructure software that may be used in emergency communication situations where lives could depend on reliable operation.**

---

## Core Principles (STRICTLY ENFORCED)

### 1. Zero Tolerance for Failure
- This software could be used when all other communication infrastructure has failed
- Emergency responders, disaster relief, and remote operations may rely on this modem
- Every component must be designed, implemented, and tested with the assumption that failure is not an option

### 2. Rigorous Testing Requirements (MANDATORY)

**RULE: NEVER test against simplified models. ALWAYS test the full modem pipeline. This is NOT optional.**

- All tests MUST go through the complete signal processing chain:
  - TX: Data → LDPC Encoding → Interleaving → Modulation (OFDM/OTFS) → Audio output
  - RX: Audio input → Synchronization → Demodulation → Equalization → De-interleaving → LDPC Decoding → Data
- Unit tests for individual components are necessary but NOT sufficient
- Integration tests through the full pipeline are MANDATORY
- All new features must include tests that exercise the complete modem chain

### 3. Channel Simulation Standards (MANDATORY)

**RULE: Always use realistic channel models for testing. Testing only with AWGN or ideal conditions is FORBIDDEN.**

- ITU-R F.1487 Watterson HF Channel Model (`src/sim/hf_channel.hpp`)
- Test against ALL standard channel conditions:
  - AWGN (baseline)
  - Good (0.5ms delay, 0.1Hz Doppler)
  - Moderate (1.0ms delay, 0.5Hz Doppler)
  - Poor (2.0ms delay, 1.0Hz Doppler)
  - Flutter (0.5ms delay, 10Hz Doppler - auroral/polar paths)
- NEVER assume "good enough" performance on AWGN alone

### 4. Build a Proven Foundation

As early development software, we must:
- Establish rock-solid fundamentals before adding features
- Verify every layer independently AND as part of the integrated system
- Document all assumptions and edge cases
- Maintain comprehensive test coverage

### 5. Test-Driven Development (CRITICAL!)

**RULE: Always test from simple to complex. Never debug a complex system when the basics haven't been proven.**

When testing OFDM or any signal processing:
1. **Start with the absolute basics** - raw FFT roundtrip, single carrier, no extras
2. **Add one feature at a time** - pilots, then CP, then NCO, then sync
3. **Each layer must pass 100%** before adding the next layer
4. **Disable all heuristics for initial tests** - no phase inversion detection, no adaptive EQ
5. **Use known test patterns** - start with simple patterns (0x00, 0xFF, 0xAA) before complex data

Example test progression:
```
Layer 1: FFT/IFFT roundtrip
Layer 2: Carrier mapping (data on specific bins)
Layer 3: Cyclic prefix add/remove
Layer 4: NCO up/down conversion
Layer 5: Single OFDM symbol (manual construction)
Layer 6: Multi-symbol sequence
Layer 7: Pilot-based channel estimation
Layer 8: Full modulator → demodulator (NO extras)
Layer 9: Add sync detection
Layer 10: Add LDPC encoding/decoding
Layer 11: Full pipeline with realistic channel
```

**If a higher layer fails, the bug is almost certainly in a lower layer that wasn't tested properly.**

### 6. Continuous Listening Mode (Target Architecture)

The modem must support **continuous operation** where it:
- Runs in "always listening" mode waiting for transmissions
- Detects preambles and syncs automatically
- Demodulates incoming data
- Returns to listening after transmission ends
- Handles back-to-back transmissions

**Software loopback testing** must simulate this continuous mode before hardware cross-wire testing:
1. TX generates signal with preamble + data + silence
2. RX runs in continuous mode, detects sync, extracts data
3. RX returns to listening, ready for next transmission
4. Test with multiple transmissions in sequence

## Key Components to Test Rigorously

### Physical Layer
- OFDM modulator/demodulator (`src/ofdm/`)
- OTFS modulator/demodulator (`src/otfs/`)
- Synchronization (Schmidl-Cox preamble correlation) (`src/sync/`)
- Channel estimation and equalization (`src/ofdm/channel_estimator.cpp`)
- Frequency offset tolerance
- Timing recovery

### Forward Error Correction
- LDPC encoder/decoder (`src/fec/`)
- All code rates: R1/4, R1/2, R2/3, R3/4, R5/6
- Multi-block LDPC operation
- Soft demapping accuracy

### Protocol Layer
- ARQ mechanisms (`src/protocol/arq.cpp`, `src/protocol/selective_repeat_arq.cpp`)
- Connection establishment and teardown
- File transfer integrity
- Frame parsing and building (`src/framing/`)

### DSP Components
- FFT implementation (`src/dsp/fft.cpp`)
- Resampling (`src/dsp/resampler.cpp`)
- Filtering (`src/dsp/filters.cpp`)

## Testing Checklist for Every Change (MANDATORY)

**Claude MUST verify these items before considering any code change complete:**

1. [ ] Does the change affect signal processing? Run full pipeline tests
2. [ ] Does the change affect protocol? Test full connection cycle with channel impairments
3. [ ] Have tests been run with Watterson channel model at multiple SNR levels?
4. [ ] Have edge cases been tested (boundary conditions, error paths)?
5. [ ] Does the modem still work under Poor and Flutter channel conditions?

## CRITICAL: Hardware Recording Test Case

### Ground Truth Recording: `tests/data/f3_deadbeef_3bursts.raw`

This file contains **3 DEADBEEF bursts** recorded from actual hardware:
- **Format**: 32-bit float, 48kHz, mono, raw PCM
- **Duration**: ~22.5 seconds
- **Bursts**: 3 transmissions at ~3.9s, ~6.7s, ~8.0s (each ~0.7s duration)
- **TX Settings**: DQPSK modulation, R1/4 LDPC, 21 bytes DEADBEEF pattern per burst
- **Expected decode**: Each burst should decode to `DE AD BE EF DE AD BE EF...` (21 bytes)

### Primary Objective

**The `ultra` binary MUST be able to decode this recording automatically.**

```bash
# Target command (must work):
cat /tmp/f3_recording.raw | ./ultra rx -m dqpsk -c 1/4
# Expected output: 3x DEADBEEF patterns
```

### Debugging Methodology (If Decode Fails)

If the recording fails to decode, systematically test each layer:

1. **Preamble Detection**: Can we detect the 3 sync points?
2. **Downconversion**: Is NCO frequency correct? Phase aligned?
3. **FFT/Symbol Extraction**: Are OFDM symbols correctly windowed?
4. **Channel Estimation**: Are pilots being found and H estimated?
5. **Equalization**: Is data being correctly equalized?
6. **Soft Demapping**: Are LLRs correct sign/magnitude for DQPSK?
7. **LDPC Decoding**: Does soft decoding converge?

**Create standalone test for EACH layer** until the failure point is found.

### Recording Test Commands

```bash
# Record new test file (USB soundcard input):
parecord -d alsa_input.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.mono-fallback \
  --format=float32le --rate=48000 --channels=1 --file-format=raw recording.raw

# Analyze recording for bursts:
# (Check for signal presence, timing, levels)
```

### F3 Test Pattern Encoding (GUI)

The GUI's F3 key sends DEADBEEF with these exact settings:
- **Data**: 21 bytes of `{0xDE, 0xAD, 0xBE, 0xEF}` repeated
- **LDPC**: R1/4 (forced, regardless of GUI setting)
- **Modulation**: DQPSK (for link establishment compatibility)
- **Preamble**: Standard Schmidl-Cox preamble prepended

Any decoder MUST match these settings to decode F3 transmissions.

---

## Existing Test Suite

Located in `tests/`:
- `test_modem_loopback.cpp` - Full pipeline loopback testing
- `test_protocol_modem.cpp` - Protocol over modem pipeline
- `test_ofdm.cpp` - OFDM modulation/demodulation
- `test_otfs.cpp` - OTFS modulation
- `test_multiblock_ldpc.cpp` - FEC with multiple LDPC blocks
- `test_freq_offset.cpp` - Frequency offset tolerance
- `test_timing_offset.cpp` - Timing recovery
- `test_selective_repeat.cpp` - Selective Repeat ARQ
- `test_adaptive_mode.cpp` - Adaptive modulation/coding
- `test_adaptive_link.cpp` - Link adaptation

Simulation tool: `ultra_sim` - Full system simulation with channel models

## Development Philosophy

> "In emergency communications, 'it works most of the time' is not acceptable.
> The modem must work when it matters most - when conditions are worst."

- Prefer correctness over cleverness
- Prefer robustness over performance (optimize only proven-correct code)
- Every edge case is a potential point of failure in an emergency
- When in doubt, add more tests

---

## ENFORCEMENT

**These rules exist because this software may be the last line of communication in a disaster. Claude MUST:**

1. **NEVER** skip full pipeline testing to save time
2. **NEVER** assume a component works without testing it through the complete modem chain
3. **NEVER** test only against ideal channel conditions
4. **NEVER** merge or suggest merging code that hasn't been validated against Watterson channel models
5. **ALWAYS** treat every line of code as if someone's life depends on it working correctly

**Violations of these rules are unacceptable. There are no shortcuts in critical software.**
