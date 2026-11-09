#pragma once

// ULTRA_FAST=1 — one lockstep switch for the speed stack validated on the GUI gate on
// 2026-11-08/09 (docs/EVM_RATE_SELECTION_2026_11_08.md §4): preamble-less burst acquisition
// (ULTRA_BURST_ACQ=structure), the measured latent rate anchors (ULTRA_LATENT_ANCHORS=measured),
// the frame-level turbo code (ULTRA_DATA_FEC=turbo), the effective-SNR climb
// (ULTRA_RATE_EVM=climb) and the measured 6 dB AWGN OFDM entry floor (ULTRA_OFDM_FLOOR_AWGN_DB=6).
//
// It only changes DEFAULTS: every member knob that is set explicitly wins, so one component can
// be A/B'd inside the profile (e.g. ULTRA_FAST=1 ULTRA_DATA_FEC=ldpc). LOCKSTEP — set it on BOTH
// stations: turbo data frames and chirp-less group starts are not negotiated (design decision).
// Read once.

#include <cstdlib>
#include <cstring>

namespace ultra {

inline bool fastProfileEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_FAST");
        return e != nullptr && std::strcmp(e, "1") == 0;
    }();
    return on;
}

}  // namespace ultra
