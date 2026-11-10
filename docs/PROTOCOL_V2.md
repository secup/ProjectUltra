# ULTRA Protocol v2 (Implementation-Aligned Spec)

Status: aligned with `main` as of 2026-02-12.

## Scope

This document describes how ULTRA v2 currently behaves in code, not historical intent.

Primary sources:
- `src/protocol/frame_v2.hpp`
- `src/protocol/frame_v2.cpp`
- `src/protocol/connection.hpp`
- `src/protocol/connection.cpp`
- `src/protocol/connection_handlers.cpp`
- `src/protocol/selective_repeat_arq.hpp`
- `src/protocol/selective_repeat_arq.cpp`
- `src/protocol/waveform_selection.hpp`
- `src/protocol/file_transfer.hpp`
- `src/protocol/file_transfer.cpp`
- `src/protocol/compression.hpp`
- `src/protocol/compression.cpp`

## Key Constants

- v2 magic: `0x554C` (`"UL"`)
- Ping/Pong raw marker: `"ULTR"` (4 bytes)
- Control frame size: 20 bytes
- Data frame header: 17 bytes
- Data frame CRC trailer: 2 bytes
- LDPC coded block: 648 bits (81 coded bytes)
- Callsign sanitization length: 8 chars (`sanitizeCallsign`)
- Connect payload callsign fields: 10 bytes each (null-terminated)

## Waveform Modes and Capabilities

`WaveformMode` values:
- `0x00` `OFDM_COX`
- `0x01` `OTFS_EQ` (reserved; not advertised by production builds)
- `0x02` `OTFS_RAW` (reserved; not advertised by production builds)
- `0x03` `MFSK` (reserved; not implemented)
- `0x04` `MC_DPSK`
- `0x05` `OFDM_CHIRP`
- `0x06` `OFDM_NARROW`
- `0xFF` `AUTO`

`ModeCapabilities` bitmap:
- `0x01` `OFDM_COX`
- `0x02` `OTFS_EQ` (reserved; not in `ALL`)
- `0x04` `OTFS_RAW` (reserved; not in `ALL`)
- `0x08` `MFSK` (reserved; not in `ALL`)
- `0x10` `MC_DPSK`
- `0x20` `OFDM_CHIRP`
- `0x40` `OFDM_NARROW`
- `0x71` `ALL` production-supported modes

## Frame Families

There are 4 practical families on air.

1. Raw Ping/Pong (no LDPC frame container)
2. ControlFrame (20 bytes)
3. ConnectFrame (serialized as Data-style frame, 35 bytes)
4. DataFrame (variable serialized size)

### Raw Ping/Pong

- On air: the MC-DPSK dual chirp + 8 training symbols + 1 reference symbol (`encodePing` =
  `generatePreamble`), no data, no LDPC, no addressing (66 816 samples, 1.39 s). A PONG is the
  same waveform. The `PingFrame` "ULTR" struct is not sent; it survives only in tools and tests.
- Meaning comes from local state: a bare chirp received while DISCONNECTED is a PING (answer
  with a PONG); while PROBING it is the PONG (send the CONNECT).
- The receiver classifies it by the silence after the training symbols (PATH1, data/training RMS
  over the next 5 000 samples) and, at low SNR, by a solid chirp lock with no decodable payload
  (PATH2). Since 2026-11-10 PATH1 is judged as soon as those 14 216 samples have arrived, not
  after a whole one-codeword slot (INV-HANDSHAKE-002).

### ControlFrame (20 bytes)

Layout:
- `MAGIC(2) TYPE(1) FLAGS(1) SEQ(2) SRC_HASH(3) DST_HASH(3) PAYLOAD(6) CRC16(2)`

Used by frame types where `v2::isControlFrame(type)` is true:
- `PROBE`, `PROBE_ACK`, `KEEPALIVE`, `MODE_CHANGE`, `ACK`, `NACK`, `BEACON`

### DataFrame (variable)

Layout:
- `MAGIC(2) TYPE(1) FLAGS(1) SEQ(2) SRC_HASH(3) DST_HASH(3) TOTAL_CW(1) LEN(2) HCRC(2) PAYLOAD(LEN) FCRC(2)`

### ConnectFrame (35 bytes serialized)

ConnectFrame uses DataFrame-style header + fixed payload (INV-HANDSHAKE-001):
- Header: 17 bytes (`TOTAL_CW` = `kConnectFrameCodewords` = 2, `LEN` = 16)
- Payload: 16 bytes
- Frame CRC: 2 bytes
- Total serialized bytes: 35, sent over MC-DPSK as a fixed 2-codeword R1/4 frame with
  frame-level interleaving (4.8 s of DBPSK airtime; 45 bytes in 4 codewords, 8.3 s, before
  2026-11-10)

Payload fields:
- `SRC_CALL[10]` — the SENDER's callsign (station identification: CONNECT carries the
  initiator's, CONNECT_ACK the responder's). The destination is `DST_HASH` only.
- `mode_capabilities[1]` (CONNECT_ACK: the responder's fading index)
- `negotiated_mode[1]`
- `initial_modulation[1]`
- `initial_code_rate[1]`
- `measured_snr[1]` (CONNECT_ACK)
- `cw_count[1]` (bits 0-3 CW count, 4-6 ladder rung on CONNECT_ACK, 7 PHY_MASK_V1 on CONNECT_ACK)

Used for:
- `CONNECT`, `CONNECT_ACK`, `CONNECT_NAK` (DISCONNECT is a ControlFrame)

Special value:
- `DISCONNECT_SEQ = 0xFFFF`

## Frame Types (Current Status)

| Type | Value | Family | Current Runtime Usage |
|---|---:|---|---|
| `PING` | `0x01` | Raw | Active |
| `PONG` | `0x02` | Raw | Active |
| `PROBE` | `0x10` | Control | Defined, currently ignored by Connection |
| `PROBE_ACK` | `0x11` | Control | Defined, currently ignored by Connection |
| `CONNECT` | `0x12` | Connect | Active |
| `CONNECT_ACK` | `0x13` | Connect | Active |
| `CONNECT_NAK` | `0x14` | Connect | Active |
| `DISCONNECT` | `0x15` | Connect | Active |
| `KEEPALIVE` | `0x16` | Control | Defined, not used in current flow |
| `MODE_CHANGE` | `0x17` | Control | Active |
| `ACK` | `0x20` | Control | Active |
| `NACK` | `0x21` | Control | Active |
| `DATA` | `0x30` | Data | Active |
| `DATA_START` | `0x31` | Data | Defined, not emitted in current app flow |
| `DATA_CONT` | `0x32` | Data | Defined, not emitted in current app flow |
| `DATA_END` | `0x33` | Data | Defined, not emitted in current app flow |
| `BEACON` | `0x40` | Control | Defined, not used in current flow |

## Flags

`v2::Flags` bit layout:
- Bit 0: `VERSION_V2` (`0x01`)
- Bit 1: `URGENT` (`0x02`)
- Bit 2: `COMPRESSED` (`0x04`)
- Bit 3: `ENCRYPTED` (`0x08`)
- Bit 4: `MORE_FRAG` (`0x10`)
- Bit 5: `FINAL` (`0x20`)
- Bits 6-7: rate markers (`RATE_1_4`, `RATE_1_2`, `RATE_2_3`, `RATE_3_4`)

Current practical notes:
- `MORE_FRAG` is actively used for message/file chunk continuation.
- File compression signaling uses `FileFlags::COMPRESSED` in file metadata payload, not DataFrame `Flags::COMPRESSED`.
- `FINAL` and flag-based rate bits are defined but not a primary control signal in current transport flow.

## Connection Lifecycle

### Initiator path

1. `connect(remote)` enters `PROBING` and sends raw `PING`.
2. On `PONG`, enters `CONNECTING` and sends `CONNECT` ConnectFrame.
3. On `CONNECT_ACK`, applies negotiated waveform + initial data mode and enters `CONNECTED`.

### Responder path

1. On valid `CONNECT` in `DISCONNECTED`, negotiates waveform/mode.
2. Sends `CONNECT_ACK` carrying:
- negotiated waveform
- initial modulation
- initial code rate
- measured SNR
3. Enters `CONNECTED` as responder.

### Handshake confirmation behavior

- Initiator: handshake is confirmed immediately on `CONNECT_ACK`.
- Responder: waits for first valid post-ACK frame from initiator.
- Responder fail-safe: if no post-ACK frame arrives within `2200 ms`, force handshake confirmation to avoid staying on handshake TX behavior forever.

### Connect waveform fallback

- Default connect waveform is `MC_DPSK`.
- MFSK fallback is disabled because MFSK is reserved but not implemented.

## Negotiation and Mode Selection

Negotiation inputs:
- Local capabilities
- Remote capabilities
- Remote preference (or `AUTO`)
- Local preference (or `AUTO`)
- Measured SNR
- Fading index
- Optional forced modulation/code rate from initiator

Selection logic source:
- `recommendWaveformAndRate(...)`
- `recommendDataMode(...)`
- `selectOFDMCodeRate(...)`
- `capInitialOFDMRate(...)`

OFDM code-rate thresholds live in
`src/protocol/waveform_selection.hpp::selectOFDMCodeRate()`. Keep
`tests/test_waveform_policy.cpp` as the regenerable boundary record
instead of copying the ladder into this protocol overview.

Current waveform auto-selection:
- `MC_DPSK`: `snr < 10`
- `OFDM_CHIRP`: `snr >= 10`
- `OFDM_COX`: implemented and forceable, but not selected by the production
  auto ladder

Current data modulation policy:
- OFDM: DQPSK default path
- D8PSK is selected only by the guarded `recommendDataMode(...)` high-SNR /
  low-fading policy. Exact gates live in
  `src/protocol/waveform_selection.hpp` and `tests/test_waveform_policy.cpp`.
- Manual non-DQPSK forcing is a lab-only tool path behind `--expert`; it is
  not part of the operator auto-rate ladder.

## Encoding/Decoding Strategy by Waveform

### MC-DPSK path

- Uses variable-CW LDPC framing (`encodeFrameWithLDPC`)
- Decoder uses sequential MC-DPSK CW decode path
- For non-20-byte frames, encoder patches `TOTAL_CW` + CRCs to match actual encoded CW count before TX

Typical handshake observation:
- The 35-byte ConnectFrame goes out as a fixed 2-codeword interleaved R1/4 frame (the
  receiver waits for `kConnectFrameCodewords` when CW0 alone does not decode)

### OFDM path

Two distinct paths are used.

1. Control frames (`isControlFrame == true`)
- Encoded as 1-CW control at hardened profile: `DQPSK + R1/4`
- Receiver first tries OFDM control-profile decode before data-profile decode
- Whitened (2026-11-10, INV-FRAME-WHITEN-001): the sender XORs the 20 frame bytes with the fixed
  sequence `v2::kControlFrameWhitening` before LDPC encoding, so the header cannot line the
  symbol's carriers up into a peak. A whitened frame decodes to the whitened magic `38 50`; the
  receiver XORs it back (`v2::unwhitenControlCodeword`, in its LDPC codec wrapper). Raw-magic frames
  (`55 4C`: MC-DPSK control frames, a sender with `ULTRA_CONTROL_WHITEN=0`) are accepted unchanged,
  so the two stations need not agree; a pre-2026-11-10 receiver cannot read whitened frames.

2. Non-control frames (data/connect)
- Encoded through fixed 4-CW frame path with frame-level interleaving
- Data payload capacity depends on code rate

Fixed 4-CW payload capacities (`getFixedFramePayloadCapacity`):
- `R1/4`: 61 bytes
- `R1/2`: 141 bytes
- `R2/3`: 197 bytes
- `R3/4`: 221 bytes

## ARQ Semantics (Selective Repeat)

ULTRA connection currently uses `SelectiveRepeatARQ`.

### ACK meaning

ACKs are cumulative plus selective bitmap:
- `ACK.seq` = cumulative base (`rx_base_seq - 1`)
- `ACK.payload[2]` = positive SACK bitmap for up to 8 frames ahead

Sender behavior:
- Advance base cumulatively through `ACK.seq`
- Mark additional received frames from bitmap bits
- Detect base-hole patterns and trigger fast retransmit logic

### NACK meaning

- `NACK.seq` references sequence
- In SR path, NACK currently triggers retransmit of that frame sequence
- Per-codeword bitmap exists in payload format, but SR main path is frame-oriented

### Runtime ARQ configuration by mode

When entering `CONNECTED`:

MC-DPSK:
- window = 1
- ack timeout = 18000 ms
- sack delay = 2000 ms
- ack repeat count = 1

OFDM:
- window = 4
- max retries = 15
- sack delay = 120 ms
- ack repeat default = 2 (`220 ms` spacing)
- ack repeat for D8PSK R1/2 = 3 (`250 ms` spacing)
- base ack timeout computed by `computeOfdmAckTimeoutMs(...)`, clamped to `4500..14000 ms`
- adaptive RTT estimator then refines timeout (`currentAckTimeoutMs()`)

Additional reliability logic present:
- stale ACK guard
- far-future ACK guard
- duplicate ACK dedup window
- hole-based fast retransmit
- hole-probe retransmit timer
- ACK repeat queue coalescing and jitter

## Disconnect Hardening

### Initiator side

- Sends `DISCONNECT` ConnectFrame with `seq = 0xFFFF`
- Enters `DISCONNECTING`
- Retransmits DISCONNECT every 5s, up to 3 retries
- Times out on `disconnect_timeout_ms` if no valid disconnect ACK

### ACK qualification during disconnect

While in `DISCONNECTING`, only ACK with `seq == DISCONNECT_SEQ (0xFFFF)` is accepted as disconnect confirmation.

This prevents stale data ACKs from being misinterpreted as disconnect ACK.

### Responder side grace period

On DISCONNECT reception:
- Sends ACK immediately
- Stays connected for 5s grace period
- Re-sends disconnect ACK every 2s during grace
- Drops to disconnected when grace expires

## Fragmentation and Reassembly

### Message fragmentation

- OFDM fixed-frame mode fragments application messages beyond per-frame payload capacity
- Intermediate fragments carry `MORE_FRAG`
- Receiver reassembles buffered fragments; final fragment clears `MORE_FRAG`

### File transfer payload protocol

All file transfer uses `FrameType::DATA` and a payload type byte:
- `0x01` `FILE_START`: flags, original size, CRC32, filename
- `0x02` `FILE_DATA`: 32-bit offset + data chunk

Sender:
- Reads full file into memory
- Rejects files larger than `UINT32_MAX`
- Sends one metadata chunk then data chunks
- For OFDM, chunk payload is capped to fixed-frame capacity

Receiver:
- Validates offsets to reject duplicate/overlap/gap conditions
- Uses `MORE_FRAG` plus size rules for finalization
- Verifies CRC32 before declaring success

### Compression

File transfer compression behavior:
- Compression algorithm: Deflate via miniz
- Compression considered when input size >= 32 bytes
- `shouldCompress` uses quick sample ratio threshold `< 0.9`
- Compression flag is carried in file metadata (`FileFlags::COMPRESSED`)

## Callsign Hashing

Routing hash is 24-bit DJB2-style hash (`hashCallsign`) over sanitized callsign.

Behavior notes:
- Runtime callsign sanitization currently truncates to 8 chars.
- ConnectFrame still carries explicit callsign strings for station identification.

## Fading Index and Adaptation Inputs

Connection-level channel quality uses:
- SNR (dB)
- fading index (`freq_cv + temporal_cv` concept in waveform layer)

Operational threshold used by connection helper:
- `isFading()` true when `fading_index > 0.65`

## Defined But Not Fully Active

The following protocol elements are defined in enums/constructors but not primary in current live flow:
- `PROBE`, `PROBE_ACK`, `KEEPALIVE`, `BEACON`
- `DATA_START`, `DATA_CONT`, `DATA_END`
- DataFrame `Flags::FINAL`
- DataFrame `Flags::COMPRESSED` generic path
- Flag rate bits as primary transport control signal

## Version

- `v2.3 (2026-02-12)`: Spec rewritten to match current implementation behavior
- `v2.2 (2026-01-26)`: older negotiation clarification (historical)
- `v2.1 (2026-01-24)`: earlier PHY section expansion (historical)
- `v2 (2026-01-18)`: initial v2 draft (historical)
