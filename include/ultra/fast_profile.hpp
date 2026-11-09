#pragma once

// ULTRA_FAST — the speed stack validated on the GUI gate on 2026-11-08/09
// (docs/EVM_RATE_SELECTION_2026_11_08.md §4), ON BY DEFAULT since 2026-11-09:
// preamble-less burst acquisition (ULTRA_BURST_ACQ=structure), the measured latent rate anchors
// (ULTRA_LATENT_ANCHORS=measured), the frame-level turbo code (ULTRA_DATA_FEC=turbo), the
// effective-SNR climb (ULTRA_RATE_EVM=climb) and the measured 6 dB AWGN OFDM entry floor
// (ULTRA_OFDM_FLOOR_AWGN_DB=6).
//
// ULTRA_FAST=0 restores every member's pre-2026-11-09 default (chirp per burst, July anchors,
// LDPC data frames, no EVM climb, 8 dB AWGN floor) — the baseline arm of the pending rig A/B.
// It only changes DEFAULTS: every member knob that is set explicitly wins, so one component can
// be A/B'd inside the profile (e.g. ULTRA_DATA_FEC=ldpc). LOCKSTEP — both stations must run the
// same setting: turbo data frames and chirp-less group starts are not negotiated (no deployed
// stations to stay compatible with; design decision). Read once.

#include <cstdlib>
#include <cstring>

namespace ultra {

inline bool fastProfileEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_FAST");
        return !(e != nullptr && std::strcmp(e, "0") == 0);
    }();
    return on;
}

}  // namespace ultra
