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
│  Physical Layer (OFDM/OTFS Modulation, Sync)        │
└─────────────────────────────────────────────────────┘
```

---

## Frame Format v2

### Design Goals
1. **Control frames**: Fit in 1 codeword (≤20 bytes) for efficiency
2. **Data frames**: Variable length with clear codeword count signaling
3. **Backward compatible**: Can coexist with v1 frames

### Frame Types

| Type | Value | Description | Typical Size |
|------|-------|-------------|--------------|
| PROBE | 0x10 | Channel probe request | 1 codeword |
| PROBE_ACK | 0x11 | Channel probe response | 1 codeword |
| CONNECT | 0x12 | Connection request | 1 codeword |
| CONNECT_ACK | 0x13 | Connection accepted | 1 codeword |
| CONNECT_NAK | 0x14 | Connection rejected | 1 codeword |
| DISCONNECT | 0x15 | End connection | 1 codeword |
| KEEPALIVE | 0x16 | Maintain connection | 1 codeword |
| ACK | 0x20 | Acknowledge data | 1 codeword |
| SACK | 0x21 | Selective ACK (bitmap) | 1 codeword |
| DATA | 0x30 | Data segment | Variable |
| DATA_START | 0x31 | First segment (with metadata) | Variable |
| DATA_END | 0x32 | Last segment (with checksum) | Variable |
| BEACON | 0x40 | CQ broadcast | 1 codeword |

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

### Data Frame Format (Variable length)

```
┌────────┬──────┬───────┬───────┬───────────┬───────────┬─────────┬──────┬───────┐
│ MAGIC  │ TYPE │ FLAGS │ SEQ   │ XFER_ID   │ FRAG_INFO │ LEN     │ DATA │ CRC16 │
│  2B    │  1B  │  1B   │  2B   │    2B     │    4B     │  2B     │  N   │  2B   │
└────────┴──────┴───────┴───────┴───────────┴───────────┴─────────┴──────┴───────┘
Header: 14 bytes, Trailer: 2 bytes
```

**Additional Fields for DATA frames:**
- **XFER_ID** (2 bytes): Transfer identifier (for concurrent transfers)
- **FRAG_INFO** (4 bytes): Fragment offset (3B) + Total fragments (1B) or Total size for DATA_START
- **LEN** (2 bytes): Payload length in this frame (0-65535)
- **DATA** (N bytes): Payload data
- **CRC16** (2 bytes): CRC-16 over entire frame

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

### Codeword Count Calculation

For a frame with total size `S` bytes and code rate with `k` info bits:

```
bytes_per_codeword = floor(k / 8)  // Integer bytes that fit
codewords_needed = ceil(S / bytes_per_codeword)
```

For R1/4 (k=162 bits = 20.25 bytes):
- Use 20 bytes per codeword (discard 0.25 bytes capacity for simplicity)
- Frame of N bytes needs: `ceil(N / 20)` codewords

### Transmission Structure

```
┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
│   Preamble      │   Codeword 1    │   Codeword 2    │   Codeword N    │
│   (Sync)        │   (81 bytes)    │   (81 bytes)    │   (81 bytes)    │
└─────────────────┴─────────────────┴─────────────────┴─────────────────┘
```

### Frame Size Examples

| Frame Type | Size (bytes) | Codewords (R1/4) | TX Time |
|------------|--------------|------------------|---------|
| Control (PROBE, ACK) | 20 | 1 | ~2.3 sec |
| Small DATA (100B payload) | 116 | 6 | ~14 sec |
| Medium DATA (256B payload) | 272 | 14 | ~32 sec |
| Large DATA (1KB payload) | 1038 | 52 | ~2 min |

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

```
    │──── DISCONNECT ──────────────────►│
    │◄─── DISCONNECT (optional ack) ────│
```

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

## Version History

- v2.0 (2026-01-18): Initial specification
  - Optimized control frames for 1 codeword
  - 16-bit sequence numbers
  - Callsign hashing
  - Selective Repeat ARQ
  - Large file transfer support
