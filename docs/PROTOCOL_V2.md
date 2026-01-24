# ULTRA Protocol v2 Specification

## Overview

ULTRA (Universal Lightweight Transport for Radio Amateurs) Protocol v2 is designed for reliable data transfer over HF radio channels. It supports:

- Short messages (text, position reports)
- Large file transfers (images, documents up to 16MB)
- Adaptive modulation based on channel conditions
- Selective Repeat ARQ for reliable delivery
- Efficient codeword utilization

## Design Constraints

### LDPC Codeword Capacity (R1/4)
- k = 162 info bits = **20.25 bytes** per codeword
- n = 648 coded bits = 81 bytes transmitted per codeword
- **Key insight**: Headers should fit in 1 codeword (≤20 bytes) when possible

### Data Rates (Approximate)
| Code Rate | Info bits/codeword | Effective Rate* |
|-----------|-------------------|-----------------|
| R1/4      | 162 bits          | ~35 bytes/sec   |
| R1/2      | 324 bits          | ~70 bytes/sec   |
| R2/3      | 432 bits          | ~95 bytes/sec   |

*At 30 bits/symbol, ~1125 bits/sec raw OFDM rate

---

## Layer Architecture

```
┌─────────────────────────────────────────────────────┐
│  Application Layer (File Transfer, Messaging)       │
├─────────────────────────────────────────────────────┤
│  Transport Layer (Segmentation, ARQ, Flow Control)  │
├─────────────────────────────────────────────────────┤
│  Link Layer (Framing, Addressing, CRC)              │
├─────────────────────────────────────────────────────┤
│  FEC Layer (LDPC Encoding/Decoding)                 │
├─────────────────────────────────────────────────────┤
│  Physical Layer (OFDM/DPSK/OTFS Modulation, Sync)   │
└─────────────────────────────────────────────────────┘
```

---

## Physical Layer

### Waveform Modes

The protocol supports multiple waveforms optimized for different channel conditions:

| Mode | Value | Use Case | Min SNR | Max Throughput |
|------|-------|----------|---------|----------------|
| DPSK | 0x00 | Connection, low SNR | -8 dB | 250 bps |
| OFDM | 0x01 | Data, good channels | 17 dB | 7.2 kbps |
| OTFS_RAW | 0x02 | Data, Doppler channels | 20 dB | 3.4 kbps |
| OTFS_EQ | 0x03 | Data, stable channels | 20 dB | 3.4 kbps |

### Common Parameters

| Parameter | Value |
|-----------|-------|
| Sample Rate | 48,000 Hz |
| Center Frequency | 1,500 Hz |
| Audio Bandwidth | ~2.8 kHz (300-3100 Hz) |
| LDPC Codeword | 648 bits (81 bytes encoded) |

### DPSK (Single-Carrier Differential PSK)

Used for connection establishment and low-SNR conditions.

| Parameter | Value |
|-----------|-------|
| Modulation | DQPSK (4-phase differential) |
| Symbol Rate | 125 baud |
| Samples/Symbol | 384 |
| Preamble | Barker-13 sequence (×32 repeats) |
| Sync Threshold | -8 dB SNR |

**Frame Structure:**
```
┌──────────────┬─────────────────────────────────────┐
│   Preamble   │            LDPC Codewords           │
│  (13×32×384  │  (648 bits each, DQPSK modulated)   │
│   samples)   │                                     │
└──────────────┴─────────────────────────────────────┘
```

### OFDM (Orthogonal Frequency Division Multiplexing)

Used for high-throughput data transfer on good channels.

| Parameter | Standard Mode | NVIS Mode |
|-----------|---------------|-----------|
| FFT Size | 512 | 1024 |
| Cyclic Prefix | 64 samples | 128 samples |
| Data Carriers | 30 | 59 |
| Symbol Duration | 12 ms | 24 ms |
| Preamble | Schmidl-Cox (STS+LTS) | Same |

**Supported Modulations:**
- DQPSK, D8PSK (differential - no pilots needed)
- QPSK, 16QAM, 32QAM (coherent - requires pilots)

**Frame Structure:**
```
┌─────────┬─────────┬────────────────────────────────┐
│   STS   │   LTS   │         OFDM Symbols           │
│ (sync)  │ (chan)  │   (data + optional pilots)     │
└─────────┴─────────┴────────────────────────────────┘
```

### OTFS (Orthogonal Time Frequency Space)

Delay-Doppler domain modulation for channels with time-frequency dispersion.

| Parameter | Value |
|-----------|-------|
| Delay Bins (M) | 32 |
| Doppler Bins (N) | 16 |
| FFT Size | 512 |
| Cyclic Prefix | 64 samples |
| Symbols per Frame | 512 (M × N) |
| Bits per Frame | 648 (1 LDPC codeword) |
| Preamble | Zadoff-Chu sequence (×4 repeats) |

**OTFS Modes:**
- **OTFS_RAW**: No TF equalization, relies on DD-domain diversity
- **OTFS_EQ**: TF equalization from preamble, better for stable channels

**Frame Structure:**
```
┌──────────────┬────────────────────────────────────┐
│   Preamble   │         OTFS Data Frame            │
│  (4× ZC sym) │  (N OFDM symbols, M subcarriers)   │
│  2304 samp   │        9216 samples                │
└──────────────┴────────────────────────────────────┘
```

**Multi-Codeword Messages:**
OTFS transmits 1 LDPC codeword per frame. Multi-codeword messages use multiple frames with 10ms gaps:
```
┌─────────┬─────────┬─────┬─────────┬─────────┬─────┬─────────┬─────────┐
│ Lead-in │ Preamble│Data │   Gap   │ Preamble│Data │   Gap   │  Tail   │
│  150ms  │  (CW0)  │     │  10ms   │  (CW1)  │     │  10ms   │         │
└─────────┴─────────┴─────┴─────────┴─────────┴─────┴─────────┴─────────┘
```

### PING/PONG Probe

Fast presence detection before full connection handshake.

| Parameter | Value |
|-----------|-------|
| Duration | ~1 second |
| Waveform | DPSK (no FEC) |
| Preamble | Barker-13 (×32) |
| Payload | 4 bytes: "ULTR" magic |
| Min SNR | -8 dB |

**Probe Flow:**
```
Station A                          Station B
    │                                   │
    │──── PING "ULTR" ─────────────────►│
    │                                   │ (detected in ~50ms)
    │◄─── PONG "ULTR" ──────────────────│
    │                                   │
    │    (proceed to CONNECT if needed) │
```

### Waveform Negotiation

1. **CONNECT** always uses DPSK (reliable at low SNR)
2. **CONNECT** frame includes MODE_CAPS bitmap of supported waveforms
3. Responder selects best mutual waveform based on measured SNR
4. **CONNECT_ACK** includes NEGOTIATED waveform mode
5. Subsequent DATA frames use negotiated waveform

**MODE_CAPS Bitmap:**
```
Bit 0: DPSK supported (always 1)
Bit 1: OFDM supported
Bit 2: OTFS_RAW supported
Bit 3: OTFS_EQ supported
Bits 4-7: Reserved
```

---

## Frame Format v2

### Design Goals
1. **Control frames**: Fit in 1 codeword (≤20 bytes) for efficiency
2. **Data frames**: Variable length with clear codeword count signaling
3. **Backward compatible**: Can coexist with v1 frames

### Frame Types

| Type | Value | Description | Size |
|------|-------|-------------|------|
| PROBE | 0x10 | Channel probe request | 1 CW (control) |
| PROBE_ACK | 0x11 | Channel probe response | 1 CW (control) |
| CONNECT | 0x12 | Connection request | 3 CW (full callsigns) |
| CONNECT_ACK | 0x13 | Connection accepted | 3 CW (full callsigns) |
| CONNECT_NAK | 0x14 | Connection rejected | 3 CW (full callsigns) |
| DISCONNECT | 0x15 | End connection | 3 CW (full callsigns) |
| KEEPALIVE | 0x16 | Maintain connection | 1 CW (control) |
| ACK | 0x20 | Acknowledge data | 1 CW (control) |
| NACK | 0x21 | Request retransmit (bitmap) | 1 CW (control) |
| DATA | 0x30 | Data segment | Variable |
| DATA_START | 0x31 | First segment (with metadata) | Variable |
| DATA_END | 0x32 | Last segment (with checksum) | Variable |
| BEACON | 0x40 | CQ broadcast | 1 CW (control) |

**Frame Categories:**
- **Control frames** (1 CW): Use 24-bit callsign hashes for efficiency
- **Connect frames** (3 CW): Include full callsigns for station identification
- **Data frames** (variable): Use 24-bit hashes, payload in CW1+

### Control Frame Format (20 bytes - fits in 1 codeword)

```
┌────────┬──────┬───────┬───────┬──────────┬──────────┬─────────┬───────┐
│ MAGIC  │ TYPE │ FLAGS │ SEQ   │ SRC_HASH │ DST_HASH │ PAYLOAD │ CRC16 │
│  2B    │  1B  │  1B   │  2B   │    3B    │    3B    │   6B    │  2B   │
└────────┴──────┴───────┴───────┴──────────┴──────────┴─────────┴───────┘
Total: 20 bytes = 160 bits < 162 bits (fits in 1 R1/4 codeword)
```

**Fields:**
- **MAGIC** (2 bytes): `0x554C` ("UL") - identifies ULTRA v2 frame
- **TYPE** (1 byte): Frame type identifier
- **FLAGS** (1 byte): Frame flags (see below)
- **SEQ** (2 bytes): Sequence number (0-65535)
- **SRC_HASH** (3 bytes): 24-bit hash of source callsign
- **DST_HASH** (3 bytes): 24-bit hash of destination callsign (0xFFFFFF = broadcast)
- **PAYLOAD** (6 bytes): Type-specific payload data
- **CRC16** (2 bytes): CRC-16 CCITT over all preceding bytes

### Connect Frame Format (41 bytes - 3 codewords)

Used for: CONNECT, CONNECT_ACK, CONNECT_NAK, DISCONNECT

These frames include **full callsigns** (up to 9 characters each) for proper station identification, which is required by amateur radio regulations at the start and end of contacts.

```
┌────────┬──────┬───────┬───────┬──────────┬──────────┬──────────┬─────┬───────┐
│ MAGIC  │ TYPE │ FLAGS │ SEQ   │ SRC_HASH │ DST_HASH │ TOTAL_CW │ LEN │ HCRC  │
│  2B    │  1B  │  1B   │  2B   │    3B    │    3B    │    1B    │ 2B  │  2B   │
├────────┴──────┴───────┴───────┴──────────┴──────────┴──────────┴─────┴───────┤
│                              PAYLOAD (22 bytes)                              │
│  ┌─────────────┬─────────────┬──────────────┬────────────────┐               │
│  │ SRC_CALL    │ DST_CALL    │ MODE_CAPS    │ NEGOTIATED     │               │
│  │   10B       │   10B       │     1B       │     1B         │               │
│  └─────────────┴─────────────┴──────────────┴────────────────┘               │
├──────────────────────────────────────────────────────────────────────────────┤
│                              FCRC (2 bytes)                                  │
└──────────────────────────────────────────────────────────────────────────────┘
Header: 17 bytes, Payload: 22 bytes, CRC: 2 bytes = 41 bytes total (3 codewords)
```

**Payload Fields:**
- **SRC_CALL** (10 bytes): Full source callsign (null-terminated, max 9 chars)
- **DST_CALL** (10 bytes): Full destination callsign (null-terminated, max 9 chars)
- **MODE_CAPS** (1 byte): Supported waveform modes bitmap
- **NEGOTIATED** (1 byte): Negotiated/preferred mode

**Rationale for 3 Codewords:**
- Full callsigns ensure proper identification per amateur radio regulations
- Used at connection start (CONNECT) and end (DISCONNECT)
- DATA frames use efficient 24-bit hashes since callsigns already exchanged
- Overhead: Only +2 CW per session compared to hash-only DISCONNECT

### Data Frame Format (Variable length)

Data frames use a 17-byte header optimized for the codeword structure:

```
┌────────┬──────┬───────┬───────┬──────────┬──────────┬──────────┬─────┬───────┬──────┬───────┐
│ MAGIC  │ TYPE │ FLAGS │ SEQ   │ SRC_HASH │ DST_HASH │ TOTAL_CW │ LEN │ HCRC  │ DATA │ FCRC  │
│  2B    │  1B  │  1B   │  2B   │    3B    │    3B    │    1B    │ 2B  │  2B   │  N   │  2B   │
└────────┴──────┴───────┴───────┴──────────┴──────────┴──────────┴─────┴───────┴──────┴───────┘
Header: 17 bytes, Trailer: 2 bytes
```

**Fields:**
- **MAGIC** (2 bytes): `0x554C` ("UL")
- **TYPE** (1 byte): DATA (0x30), DATA_START (0x31), etc.
- **FLAGS** (1 byte): Frame flags
- **SEQ** (2 bytes): Sequence number
- **SRC_HASH** (3 bytes): 24-bit source callsign hash
- **DST_HASH** (3 bytes): 24-bit destination callsign hash
- **TOTAL_CW** (1 byte): Number of codewords (1-255)
- **LEN** (2 bytes): Payload length
- **HCRC** (2 bytes): Header CRC (covers bytes 0-14)
- **DATA** (N bytes): Payload data
- **FCRC** (2 bytes): Frame CRC over entire frame

### Flags Byte

```
Bit 7: ENCRYPTED    - Payload is encrypted (future)
Bit 6: COMPRESSED   - Payload is compressed
Bit 5: FINAL        - Final frame of transfer
Bit 4: MORE_FRAG    - More fragments follow
Bit 3-2: CODE_RATE  - 00=R1/4, 01=R1/2, 10=R2/3, 11=R3/4
Bit 1: URGENT       - High priority
Bit 0: VERSION      - 0=v1 compat, 1=v2
```

---

## Callsign Hashing

To fit callsigns in 3 bytes while maintaining collision resistance:

```c
uint32_t hashCallsign(const char* call) {
    uint32_t hash = 5381;
    while (*call) {
        hash = ((hash << 5) + hash) ^ toupper(*call++);
    }
    return hash & 0xFFFFFF;  // 24 bits
}
```

- 24 bits = 16.7 million unique hashes
- Full callsign sent in CONNECT handshake for verification
- Hash collision probability: ~0.006% for 1000 active stations

---

## Codeword Strategy

### Self-Identifying Codewords

Every codeword is **self-identifying** through its first bytes. This enables robust recovery when codewords are lost or arrive out of order.

#### CW0 (Header Codeword) - 20 bytes
```
┌───────┬──────┬───────┬───────┬──────────┬──────────┬──────────┬─────┬───────┬──────────┐
│ MAGIC │ TYPE │ FLAGS │ SEQ   │ SRC_HASH │ DST_HASH │ TOTAL_CW │ LEN │ HCRC  │ PAYLOAD  │
│0x554C │  1B  │  1B   │  2B   │    3B    │    3B    │    1B    │ 2B  │  2B   │   3B     │
└───────┴──────┴───────┴───────┴──────────┴──────────┴──────────┴─────┴───────┴──────────┘
```

- **MAGIC** (0x554C = "UL"): Identifies this as a header codeword
- **TOTAL_CW**: Total codewords in frame (1-255)
- **LEN**: Payload length for reassembly
- **HCRC**: Header CRC (covers bytes 0-14)
- **PAYLOAD**: First 3 bytes of payload data

#### CW1+ (Data Codewords) - 20 bytes each
```
┌────────┬───────┬─────────────────────────────────────────────────┐
│ MARKER │ INDEX │                  PAYLOAD                        │
│  0xD5  │  1B   │                   18B                           │
└────────┴───────┴─────────────────────────────────────────────────┘
```

- **MARKER** (0xD5): Identifies this as a data codeword
- **INDEX**: Codeword position (1-254) within the frame
- **PAYLOAD**: 18 bytes of frame data

### Codeword Identification Logic

```cpp
if (cw[0] == 0x55 && cw[1] == 0x4C) {
    // Header codeword (CW0) - parse full header
} else if (cw[0] == 0xD5) {
    // Data codeword (CW1+) - extract index from cw[1]
} else {
    // Unknown/corrupted - discard
}
```

### Why Self-Identifying?

**Problem**: If CW0 fails LDPC decode but CW1-3 succeed, the old design would lose the entire frame because data CWs had no identification.

**Solution**: With marker + index, the receiver can:
1. Buffer data CWs by their index while waiting for CW0
2. Send NACK requesting CW0 retransmit
3. When CW0 arrives, merge buffered CWs and reassemble
4. Overhead: 2 bytes per data CW (10%) - acceptable tradeoff

### Codeword Count Calculation

For a data frame with payload of `P` bytes:
```
total_frame = 17 (header) + P (payload) + 2 (frame CRC) = 19 + P bytes

CW0 carries: 20 bytes (header 17B + first 3B of remaining data)
CW1+ each carry: 18 bytes of payload (due to 2B marker+index overhead)

If total_frame <= 20: need 1 codeword
Otherwise: need 1 + ceil((total_frame - 20) / 18) codewords
```

### Transmission Structure

```
┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
│   Preamble      │      CW0        │      CW1        │      CWn        │
│   (Sync)        │ [0x554C header] │ [0xD5 01 data]  │ [0xD5 nn data]  │
│                 │   (81 bytes)    │   (81 bytes)    │   (81 bytes)    │
└─────────────────┴─────────────────┴─────────────────┴─────────────────┘
```

### Frame Size Examples

| Frame Type | Payload | Frame Size | Codewords (R1/4) |
|------------|---------|------------|------------------|
| Control (PROBE, ACK, NACK) | 6B | 20B | 1 |
| Connect (CONNECT, DISCONNECT) | 22B | 41B | 3 |
| Short text (20B) | 20B | 39B | 2 |
| Medium text (50B) | 50B | 69B | 4 |
| Long text (100B) | 100B | 119B | 7 |
| Large DATA (256B) | 256B | 275B | 16 |

---

## Segmentation and Reassembly

### Maximum Segment Size (MSS)

Default MSS = 256 bytes payload per DATA frame
- With 14B header + 2B CRC = 272 bytes total
- Needs 14 codewords at R1/4
- ~32 seconds per segment

### Large File Transfer Protocol

**1. Transfer Initiation (DATA_START):**
```
Sender → Receiver:
  DATA_START {
    xfer_id: random 16-bit ID,
    total_size: 4 bytes (up to 4GB),
    filename: up to 200 bytes,
    checksum: 4 bytes (CRC32 of complete file)
  }
```

**2. Data Segments (DATA):**
```
Sender → Receiver:
  DATA {
    xfer_id: matches DATA_START,
    seq: 0, 1, 2, ... (per-transfer sequence),
    frag_offset: byte offset in file (3 bytes = up to 16MB),
    payload: MSS bytes of data
  }
```

**3. Transfer Completion (DATA_END):**
```
Sender → Receiver:
  DATA_END {
    xfer_id: matches DATA_START,
    final_checksum: CRC32 of complete file
  }
```

### Reassembly Buffer

Receiver maintains per-transfer state:
```c
struct Transfer {
    uint16_t xfer_id;
    uint32_t total_size;
    uint32_t expected_checksum;
    uint8_t* buffer;           // Reassembly buffer
    uint64_t received_bitmap;  // Track received segments
    time_t last_activity;
    char filename[201];
};
```

---

## Per-Codeword Recovery

### NACK Frame for Codeword Retransmit

When one or more codewords fail LDPC decode, the receiver sends a NACK control frame:

```
NACK Payload (6 bytes in control frame):
┌───────────────┬────────────────────────────────────────┐
│ FRAME_SEQ (2B)│ CW_BITMAP (4B = 32 bits)               │
└───────────────┴────────────────────────────────────────┘
```

- **FRAME_SEQ**: Sequence number of the frame with errors
- **CW_BITMAP**: Bit i = 1 means codeword i failed (supports up to 32 CWs)

### Recovery Flow

```
Sender                              Receiver
  │                                    │
  │──── CW0 (header) ─────────────────►│  ✓ LDPC OK
  │──── CW1 (data) ───────────────────►│  ✓ LDPC OK
  │──── CW2 (data) ────────X (noise)   │  ✗ LDPC FAIL
  │──── CW3 (data) ───────────────────►│  ✓ LDPC OK
  │                                    │
  │◄─── NACK seq=N, bitmap=0x04 ───────│  (CW2 failed)
  │                                    │
  │──── CW2 (retransmit) ─────────────►│  ✓ LDPC OK
  │                                    │
  │◄─── ACK seq=N ─────────────────────│  (frame complete)
```

### Out-of-Order Codeword Handling

Because CW1+ have marker + index, the receiver can handle out-of-order arrival:

1. **CW2 arrives first**: Buffer with index=2
2. **CW1 arrives second**: Buffer with index=1
3. **CW0 arrives last**: Parse header, learn TOTAL_CW, merge buffered CWs
4. **Reassemble**: All codewords present, frame complete

This is especially useful when CW0 fails and must be retransmitted - the data CWs are already buffered.

---

## ARQ (Automatic Repeat reQuest)

### Selective Repeat Protocol

**Window Size:** 32 segments (configurable)
**Timeout:** RTT × 2 + channel delay (typically 10-30 seconds on HF)

### ACK Frame Payload (6 bytes in control frame)

```
┌──────────────┬─────────────────────────────────────────┐
│ BASE_SEQ (2B)│ BITMAP (4B = 32 bits)                   │
└──────────────┴─────────────────────────────────────────┘
```

- **BASE_SEQ**: All segments up to and including this are ACKed
- **BITMAP**: Bit i = segment (BASE_SEQ + 1 + i) received

### Flow Control

**Sliding Window:**
1. Sender transmits up to WINDOW_SIZE segments
2. Receiver sends SACK every N segments (default N=4)
3. Sender retransmits only missing segments
4. Window advances when BASE_SEQ advances

**Example Flow:**
```
Sender                              Receiver
  │                                    │
  │──── DATA seq=0 ──────────────────►│
  │──── DATA seq=1 ──────────────────►│
  │──── DATA seq=2 ────────X (lost)   │
  │──── DATA seq=3 ──────────────────►│
  │                                    │
  │◄─── SACK base=1, bitmap=0010 ─────│ (received 0,1,3, missing 2)
  │                                    │
  │──── DATA seq=2 (retransmit) ─────►│
  │──── DATA seq=4 ──────────────────►│
  │                                    │
  │◄─── SACK base=4, bitmap=0000 ─────│ (all through 4 received)
```

---

## Connection Management

### Connection Establishment (3-way handshake)

```
Initiator                           Responder
    │                                   │
    │──── PROBE ───────────────────────►│
    │                                   │ (measure channel)
    │◄─── PROBE_ACK (channel report) ───│
    │                                   │
    │──── CONNECT (full callsigns) ────►│
    │                                   │
    │◄─── CONNECT_ACK ─────────────────│
    │                                   │
    │         CONNECTION ESTABLISHED    │
```

### Keepalive

- Send KEEPALIVE every 60 seconds of inactivity
- Disconnect after 3 missed keepalives (3 minutes)

### Disconnection

DISCONNECT frames include **full callsigns** (same format as CONNECT) to ensure proper station identification at the end of the contact, as required by amateur radio regulations.

```
    │──── DISCONNECT (full callsigns) ─►│
    │◄─── DISCONNECT (optional ack) ────│
```

The DISCONNECT frame is 3 codewords (41 bytes), containing both station callsigns for clear identification of who is ending the connection.

---

## Complete Transfer Example

**Sending a 50KB image:**

```
Total: 50,000 bytes
MSS: 256 bytes
Segments: ceil(50000/256) = 196 segments

Timeline:
  0:00  PROBE → PROBE_ACK            (~5 sec)
  0:05  CONNECT → CONNECT_ACK        (~5 sec)
  0:10  DATA_START (filename, size)  (~2 sec)
  0:12  DATA seq=0..3                (~2 min)
  2:12  SACK base=3
  2:14  DATA seq=4..7                (~2 min)
  ...
  (continues with window-based flow control)
  ...
 ~25:00 DATA_END (final checksum)    (~2 sec)
 25:02  ACK (transfer complete)

Total time: ~25-30 minutes for 50KB
```

---

## Error Handling

### CRC Failures
- Frame discarded silently
- Sender will timeout and retransmit

### Sequence Gaps
- Request retransmit via SACK bitmap
- Or send NAK for specific sequence

### Checksum Mismatch (Transfer)
- Receiver sends NAK with error code
- Sender can retry entire transfer or specific segments

### Connection Timeout
- No ACK for 60 seconds: retransmit
- No response for 3 minutes: connection dropped

---

## Compression

When FLAGS.COMPRESSED is set:

- Payload is compressed with zlib (DEFLATE)
- Minimum size to compress: 100 bytes
- If compressed size >= original, send uncompressed

---

## Future Extensions

### Encryption (FLAGS.ENCRYPTED)
- Key exchange in CONNECT handshake
- AES-128-GCM for authenticated encryption
- Per-frame nonce based on sequence number

### Forward Error Correction Negotiation
- Adaptive code rate based on channel quality
- Signaled in PROBE_ACK and CONNECT

### Multi-hop Routing
- Digipeater support via relay flags
- Path tracking in extended header

---

## Implementation Notes

### Callsign Verification
Full callsigns are exchanged in CONNECT frame. The 24-bit hash is only used for frame routing efficiency. Receivers MUST verify full callsign matches on CONNECT.

### Codeword Boundary Handling
The LDPC decoder MUST accumulate ALL codewords for a frame before decoding, then extract exactly k info bits per codeword and concatenate at bit level (not byte level) to avoid boundary corruption.

### Rate Adaptation
Start with R1/4 (most robust). If PROBE_ACK indicates SNR > 10dB, can negotiate R1/2 or higher in CONNECT.

---

## Version

**v2.1 (2026-01-24)** - Added Physical Layer section with OFDM/DPSK/OTFS specs, PING/PONG probe, waveform negotiation

**v2 (2026-01-18)** - Initial v2 protocol specification
