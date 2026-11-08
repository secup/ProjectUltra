// tone_burst_encoder.hpp — generates phase-continuous 4-FSK audio for the
// tone-burst ACK. PHY_ADAPTATION_DESIGN §15 step 2.

#pragma once

#include "tone_burst_constants.hpp"
#include "tone_burst_payload.hpp"

#include <cstdint>
#include <vector>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

using Samples = std::vector<float>;

// ============================================================================
// ToneBurstEncoder — payload + duration -> phase-continuous 4-FSK audio
// ============================================================================
//
// Phase-continuous FSK: when switching frequency at a symbol boundary, the
// instantaneous phase carries over from the previous symbol. This avoids
// spectral splatter (sidelobes from instantaneous phase jumps) that would
// otherwise leak energy into the adjacent OFDM data band. Mandatory for HF
// — discontinuous FSK ("CPFSK with reset") is brutally broad-spectrum and
// real radios would never accept it.
//
// Modulation is sample-by-sample:
//   phase[n+1] = phase[n] + 2*pi*f_tone / Fs
//   sample[n]  = A * sin(phase[n])
// where f_tone is the current symbol's tone frequency. At a symbol
// boundary, phase carries over; only the increment changes.
class ToneBurstEncoder {
public:
    ToneBurstEncoder() : diversity_(toneAckDiversityEnabled()) {}

    // Frequency diversity (kToneFreqHzDiversity): each symbol on both subbands at half
    // amplitude each (same peak). Defaults to toneAckDiversityEnabled().
    void setDiversity(bool on) { diversity_ = on; }
    bool diversity() const { return diversity_; }

    // Encode one full tone-burst ACK: [4 Costas symbols][kPayloadSymbols (30) payload symbols].
    // symbol_ms picks the duration per the §15.5 staircase (default = baseline
    // ~25 ms = mid-SNR regime). Each symbol is symbol_ms * Fs / 1000 samples.
    Samples encode(const ToneBurstAckPayload& payload,
                   uint32_t symbol_ms = kBaselineSymbolMs);

    // Low-level: encode an arbitrary dibit sequence (caller is responsible
    // for prepending the Costas if they want one). Used by tests that probe
    // specific symbol patterns.
    Samples encodeDibits(const std::vector<uint8_t>& dibits,
                         uint32_t symbol_ms = kBaselineSymbolMs);

    // Reset the internal phase accumulator to 0. Tests use this for
    // determinism; production resets at the start of each burst anyway.
    void resetPhase() { phase_ = 0.0f; phase_b_ = 0.0f; }

    float currentPhase() const { return phase_; }

private:
    // Current instantaneous phase (radians), preserved across encode() calls.
    // Tests can reset() before each call for byte-exact comparison.
    float phase_ = 0.0f;
    float phase_b_ = 0.0f;  // diversity subband
    bool diversity_ = true;
};

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
