#pragma once
// M-FSK handshake frames (2026-11-22): the CONNECT / CONNECT_ACK / CONNECT_NAK frame sent as
// kHandshakeMfskTones-FSK, one kHandshakeMfskRate codeword (protocol/frame_v2.hpp). Shared by the
// encoder (what to send) and the decoder (what to try on a failed DBPSK CONNECT).
#include "protocol/frame_v2.hpp"
#include "psk/multi_carrier_dpsk.hpp"

namespace ultra {
namespace gui {
namespace handshake_mfsk {

// The connect pick's thresholds (OFDM entry floors, the MC-DPSK rungs, the PAPR prior) are
// calibrated on the DBPSK handshake frame's data-aided reading. The M-FSK frame carries more average
// power at the same peak (constant envelope vs the clipped 8-carrier sum), so on the same channel its
// reading is higher by this much; it is reported DBPSK-equivalent. Measured: tools/lowsnr_floor
// --connect (production encoder + the h = 6 control clip, 0.5 peak, AWGN, 7.3 Hz, 10 seeds per
// point): M-FSK minus DBPSK reading +1.8 / +2.1 / +1.8 / +1.9 dB at the gate's 0 / -2 / -4 / -6 dB.
inline constexpr float kMfskOverDbpskReadingDb = 1.9f;

inline MultiCarrierDPSKConfig configFor(const MultiCarrierDPSKConfig& profile) {
    MultiCarrierDPSKConfig cfg = profile;
    cfg.mfsk_tones = protocol::v2::kHandshakeMfskTones;
    return cfg;
}

// The M-FSK frame replaces the DBPSK one only where it is the same span after the chirp (so the
// handshake's timers and the receiver's frame window hold) and its tones are orthogonal with margin
// (spacing >= 2 symbol rates; a residual CFO of a few Hz then costs nothing). The 8-carrier wideband
// profile: 162 = 162 symbols, 133 Hz >= 94 Hz. The 4-carrier narrowband profile (420 Hz) fails the
// spacing and keeps DBPSK.
inline bool fits(const MultiCarrierDPSKConfig& profile) {
    const MultiCarrierDPSKConfig mfsk = configFor(profile);
    constexpr int kBits = protocol::v2::LDPC_CODEWORD_BITS;
    const int dbpsk_carriers = std::max(1, profile.num_carriers);  // DBPSK: 1 bit per carrier
    const int dbpsk_symbols =
        (protocol::v2::kConnectFrameCodewords * kBits + dbpsk_carriers - 1) / dbpsk_carriers;
    const int bps = mfsk.bitsPerSymbolTime();
    const int mfsk_symbols = (protocol::v2::kHandshakeMfskCodewords * kBits + bps - 1) / bps;
    const float spacing = (mfsk.freq_high - mfsk.freq_low) /
                          static_cast<float>(std::max(1, mfsk.mfsk_tones - 1));
    return dbpsk_symbols == mfsk_symbols && spacing >= 2.0f * mfsk.getSymbolRate();
}

}  // namespace handshake_mfsk
}  // namespace gui
}  // namespace ultra
