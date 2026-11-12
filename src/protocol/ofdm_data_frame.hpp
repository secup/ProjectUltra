#pragma once
// Compact OFDM DATA frame (INV-OFDM-FRAME-001, 2026-11-12): the session's turbo data frame with an
// 8-byte header that says where the frame sits in its physical turn.
//
// Why. A v2 OFDM DATA frame inside the one-codeword turbo frame spends 19 bytes on framing (a
// 17-byte header with the magic, type, both callsign hashes, codeword count and header CRC, plus a
// 2-byte frame CRC) although the turbo codec already carries a CRC-32C - 24 % of a QPSK R1/4
// 4-codeword frame (80 information bytes). And nothing in a frame tells the receiver how long the
// burst it belongs to is: that comes only from the group's descriptor (a 40 % loss at ITU Good
// 5 dB before the known-bit decode) or from the last frame's end-of-turn flag, so a group whose
// descriptor and tail were lost could not be acknowledged even when its middle decoded.
//
// Format. Same air geometry, same turbo code, same information size as the v2 turbo frame
// (cw x the rate's information bytes per 648-bit block):
//   byte 0      the v2 flags byte, verbatim (MORE_FRAG, FINAL, PHYSICAL_BURST_END, COMPRESSED,
//               move-epoch bits: the protocol's semantics are unchanged)
//   bytes 1-2   seq (big-endian)
//   byte 3      physical position: (index << 4) | (count - 1) - this frame is number `index` of
//               the `count` DATA frames (1..16) of its physical turn; 0xF0 = not known
//   bytes 4-5   payload length (big-endian)
//   bytes 6-7   header CRC-16 over bytes 0-5 (XOR the session seed's low 16 bits): a FAILED
//               decode's hard decisions still identify the frame for chase combining, as the v2
//               header's CRC did
//   bytes 8..   payload, padded with the fixed-frame PRBS
// and the codec's CRC-32C, seeded from the direction's callsign hashes: a frame from another
// session fails the check (2^-32), so addressing costs no byte. The type is implicit (DATA).
// Payload per frame = information bytes - 8 (v2: - 19): QPSK R1/4 cw 4 61 -> 72 bytes (+18 %),
// QPSK R1/2 cw 5 181 -> 192 (+6 %), 16QAM R2/3 cw 8 413 -> 424 (+3 %).
//
// The receiver accepts both layouts in one turbo decode (the CRC seed tells them apart: seeded =
// compact, plain = v2), rebuilds the v2 frame (with the session's hashes), so the protocol above
// the modem sees exactly what it saw before. The sender uses the compact layout inside a session
// (ModemEngine has the session's addresses) unless ULTRA_OFDM_COMPACT_FRAME=0.
#include "frame_v2.hpp"
#include "ultra/types.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace ultra {
namespace protocol {
namespace ofdm_frame {

constexpr size_t kHeaderBytes = v2::kOfdmCompactHeaderBytes;
constexpr uint8_t kPositionUnknown = 0xF0;  // index 15 of a 1-frame turn: impossible

struct Position {
    int index = -1;  // 0-based index of this DATA frame in its physical turn
    int count = 0;   // DATA frames in the turn; 0 = not known
    bool known() const { return count > 0 && index >= 0 && index < count; }
};

uint8_t packPosition(const Position& pos);
Position unpackPosition(uint8_t byte);

// CRC seed for the direction src -> dst (24-bit callsign hashes); never 0, and distinct from the
// MC-DPSK compact frame's seed for the same pair.
uint32_t sessionCrcSeed(uint32_t src_hash, uint32_t dst_hash);

// Information bytes of the turbo codeword (the CRC-32C excluded): identical to the v2 turbo frame.
size_t infoBytesFor(CodeRate rate, int cw_count, int lifting_z);

// Encode a serialized v2 DATA frame into the frame's air bits (cw x 24 x Z bits, packed MSB-first,
// air order). nullopt when the bytes are not a v2 DATA frame or the payload does not fit.
std::optional<Bytes> encode(ByteSpan v2_data_frame, CodeRate rate, int cw_count, int lifting_z,
                            const Position& position);

struct Header {
    uint8_t flags = 0;
    uint16_t seq = 0;
    Position position;
    uint16_t length = 0;
    uint16_t header_crc = 0;
};

// The compact header of `info` (decoded or hard-decision information bytes) when its header CRC
// passes for the direction src -> dst and the length fits; nullopt otherwise.
std::optional<Header> parseHeader(const Bytes& info, CodeRate rate, int cw_count, int lifting_z,
                                  uint32_t src_hash, uint32_t dst_hash);

struct DecodeResult {
    bool ok = false;
    bool compact = false;  // the decoded frame used the compact layout
    Bytes frame;           // ok: compact -> the rebuilt v2 frame; v2 -> the info bytes as sent
    Position position;     // compact frames only
    Bytes hard_info;       // de-whitened hard decisions (valid or not)
    int iterations = 0;
};

// One turbo decode that accepts the compact layout for the direction src -> dst (when
// `session_known`) and the v2 layout. src_hash: the sender (the receiver's peer); dst_hash: us.
DecodeResult decode(std::span<const float> air_llrs, CodeRate rate, int cw_count, int lifting_z,
                    uint32_t src_hash, uint32_t dst_hash, bool session_known);

// The decoder's view: decode() wrapped in the LDPC-era CodewordStatus contract of
// v2::decodeTurboFixedFrame (success => every entry decoded, data[0] = the v2 frame).
v2::CodewordStatus decodeStatus(const std::vector<float>& air_soft, CodeRate rate, int cw_count,
                                int lifting_z, uint32_t src_hash, uint32_t dst_hash,
                                bool session_known, DecodeResult* detail = nullptr);

}  // namespace ofdm_frame
}  // namespace protocol
}  // namespace ultra
