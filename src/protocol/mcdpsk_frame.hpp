#pragma once
// Compact MC-DPSK DATA frame (2026-11-11): ONE turbo codeword per frame, a 4-byte header.
//
// Why. A v2 MC-DPSK DATA frame is a chain of 648-bit LDPC codewords: a 17-byte header in the
// first, a 2-byte marker on every other, a 2-byte frame CRC. A 3-codeword DBPSK frame (61 info
// bytes) carried 37 bytes of data (40 % overhead), and the receiver learned a frame's length only
// from its first codeword, so a failed frame also lost every frame after it in the burst.
//
// Format. The frame occupies `codewords` x 648 air bits (the session's frame size, fixed for the
// session, so the receiver knows every frame's length before decoding it). One turbo codeword
// (fec::turbo, the code wideband OFDM data frames already use) carries:
//   byte 0      the v2 flags byte, verbatim (MORE_FRAG, FINAL, PHYSICAL_BURST_END, COMPRESSED,
//               move-epoch bits: the protocol's semantics are unchanged)
//   bytes 1-2   seq (big-endian)
//   byte 3      payload length
//   bytes 4..   payload, zero-padded to the codeword's information size
// and the codec's CRC-32C, seeded from the direction's callsign hashes: a frame from another
// session fails the check (2^-32), so addressing costs no byte. The type is implicit (DATA).
// 3 codewords at R1/4: 56 information bytes, 52 of them payload (was 37).
//
// The receiver rebuilds the v2 DATA frame (with the session's hashes), so the protocol layer
// above the modem sees exactly what it saw before.
#include "frame_v2.hpp"
#include "ultra/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ultra {
namespace protocol {
namespace mcdpsk_frame {

constexpr int kCodewordAirBits = 648;
constexpr size_t kHeaderBytes = 4;

inline int airBitsFor(int codewords) { return kCodewordAirBits * codewords; }

// Information bytes in the turbo codeword: floor(air_bits x rate / 8) minus the codec's 4-byte
// CRC. 0 when the frame is too small to carry a header.
size_t infoBytesFor(int codewords, CodeRate rate);

// Payload bytes a frame carries (capped at 255, the length byte).
size_t payloadCapacity(int codewords, CodeRate rate);

// CRC seed for the direction src -> dst (24-bit callsign hashes); never 0.
uint32_t sessionCrcSeed(uint32_t src_hash, uint32_t dst_hash);

// Encode a serialized v2 DATA frame into the frame's air bits, packed MSB-first
// (airBitsFor(codewords) / 8 bytes). nullopt when the bytes are not a valid v2 DATA frame or
// the payload does not fit.
std::optional<Bytes> encode(ByteSpan v2_data_frame, int codewords, CodeRate rate);

struct DecodeResult {
    bool ok = false;
    Bytes v2_frame;                    // the rebuilt, serialized v2 DATA frame when ok
    int iterations = 0;
    float extrinsic_agreement = 0.5f;  // ~0.5 on noise, near 1 on a real (even failed) codeword
};

// Decode one frame from its air LLRs (positive = bit 0, air order; at least
// airBitsFor(codewords) values). src_hash/dst_hash: the direction the frame travelled.
DecodeResult decode(std::span<const float> air_llrs, int codewords, CodeRate rate,
                    uint32_t src_hash, uint32_t dst_hash);

}  // namespace mcdpsk_frame
}  // namespace protocol
}  // namespace ultra
