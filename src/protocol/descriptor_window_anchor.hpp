#pragma once

// The burst descriptor carries the sender's ARQ window anchor (INV-DESC-WINDOW-001).
//
// A BURST_HEADER's 16-bit header seq field was always 0 on the unified-seq path (Connection
// passed group_seq=0 for every burst). It now carries the sender's ARQ move epoch and its
// transmit base: the seq of its oldest unacknowledged frame, below which everything is
// already retired at the sender. Layout:
//
//   bit 15      present (0 = no anchor: a tool or legacy caller, or the knob is off)
//   bits 13-14  the sender's move epoch (its tx_epoch_, mod 4)
//   bits 0-12   the sender's tx base mod 8192
//
// Why the receiver needs it. A rate change that re-cuts the sender's window starts a new era:
// the sender stamps it on every DATA frame and ignores any ACK of another era, because that
// ACK's seq claims were made on the old grid. Until this field, the receiver learned the era
// only from a decoded DATA frame, so after a rate step-down whose first burst failed entirely
// it kept answering in the old era - every "nothing decoded, drop a rung" ACK was discarded,
// and the sender repeated the failing rung until the waveform follow gave up on OFDM (GUI,
// ITU Good 20 -> 5 dB drop: 4 rounds, ~63 s, in every run). And when the era's first frame was
// the one lost, the receiver could not anchor the era at all and went ACK-silent until the
// sender's timeout resend. The descriptor reaches the receiver before any of the group's
// frames, so the receiver now anchors to the sender's grid at every group start.
//
// Safety. Anchoring the receive base at the sender's tx base can only make the receiver's
// cumulative ACK claim seqs the sender has already retired or abandoned - the same argument as
// a DATA frame's EPOCH_REBASE flag. 13 bits suffice: the sender's base trails the receiver's
// next-expected seq by at most one window, and runs ahead of it only by the frames an era change
// abandoned (at most one window per change, three changes per epoch cycle).

#include <cstdint>
#include <cstdlib>

namespace ultra {
namespace protocol {
namespace descriptor_window_anchor {

constexpr uint16_t kPresent = 0x8000u;
constexpr int kEpochShift = 13;
constexpr uint16_t kEpochMask = 0x3u;
constexpr uint16_t kBaseMask = 0x1FFFu;
constexpr uint16_t kBaseModulus = 0x2000u;

struct Anchor {
    bool present = false;
    uint8_t epoch = 0;
    uint16_t base_low = 0;  // the sender's tx base mod 8192
};

inline uint16_t pack(uint8_t epoch, uint16_t tx_base) {
    return static_cast<uint16_t>(kPresent |
                                 ((static_cast<uint16_t>(epoch) & kEpochMask) << kEpochShift) |
                                 (tx_base & kBaseMask));
}

inline Anchor unpack(uint16_t word) {
    Anchor a;
    a.present = (word & kPresent) != 0;
    if (!a.present) return a;
    a.epoch = static_cast<uint8_t>((word >> kEpochShift) & kEpochMask);
    a.base_low = static_cast<uint16_t>(word & kBaseMask);
    return a;
}

// Signed distance from the receiver's base to the sender's, from the sender's 13 low bits:
// the representative nearest to rx_base (|distance| <= 4096).
inline int baseDistance(uint16_t base_low, uint16_t rx_base) {
    const uint16_t diff = static_cast<uint16_t>((base_low - rx_base) & kBaseMask);
    return diff >= kBaseModulus / 2 ? static_cast<int>(diff) - static_cast<int>(kBaseModulus)
                                    : static_cast<int>(diff);
}

inline uint16_t reconstructBase(uint16_t base_low, uint16_t rx_base) {
    return static_cast<uint16_t>(static_cast<int>(rx_base) + baseDistance(base_low, rx_base));
}

// ULTRA_DESCRIPTOR_WINDOW_ANCHOR (default ON; "0" = off): the sender stamps the anchor and the
// receiver adopts it. Either end off restores the previous behavior (a zero field, or an
// ignored one), so a mismatch is safe.
inline bool enabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_DESCRIPTOR_WINDOW_ANCHOR");
        return !(e != nullptr && e[0] == '0' && e[1] == '\0');
    }();
    return on;
}

}  // namespace descriptor_window_anchor
}  // namespace protocol
}  // namespace ultra
