#pragma once

// Burst acquisition mode (Stage 2 of the 2026-11-08 speed plan).
//
// ULTRA_BURST_ACQ=structure: the connected wideband GROUP-START search runs the
// preamble-less chi structure detector (IWaveform::detectFrameStructure, dispatched from
// StreamingDecoder::searchForSync) instead of the dual-chirp search, and warm burst
// descriptors are sent WITHOUT the 1.2 s chirp (in-group frames are unchanged). LOCKSTEP:
// both stations must run the same mode (no negotiation, by design decision). Read once; the
// encoder, the decoder and the connection all consult this single function so the two halves
// can never disagree in one process. Invariants: docs/INVARIANTS.md INV-SYNC-STRUCT-001..005.

#include <cstdlib>
#include <string>

#include "ultra/fast_profile.hpp"

namespace ultra {
namespace sync {

// Explicit ULTRA_BURST_ACQ wins; unset, the ULTRA_FAST profile selects structure.
inline bool structureAcquisitionEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_BURST_ACQ");
        if (e != nullptr) return std::string(e) == "structure";
        return fastProfileEnabled();
    }();
    return on;
}

}  // namespace sync
}  // namespace ultra
