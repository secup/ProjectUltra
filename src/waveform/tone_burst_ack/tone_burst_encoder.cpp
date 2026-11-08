// tone_burst_encoder.cpp — phase-continuous 4-FSK modulation.

#include "tone_burst_encoder.hpp"

#include <cmath>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

namespace {

constexpr float kTwoPi = 6.283185307179586f;

// Phase-wrap to [0, 2*pi). Cheaper than fmod for small accumulations.
inline float wrapPhase(float p) {
    while (p >= kTwoPi) p -= kTwoPi;
    while (p < 0.0f)   p += kTwoPi;
    return p;
}

}  // namespace

Samples ToneBurstEncoder::encode(const ToneBurstAckPayload& payload,
                                  uint32_t symbol_ms) {
    return encodeDibits(buildOnAirDibits(payload), symbol_ms);
}

Samples ToneBurstEncoder::encodeDibits(const std::vector<uint8_t>& dibits,
                                        uint32_t symbol_ms) {
    const uint32_t samples_per_symbol = (kSampleRate * symbol_ms) / 1000u;
    Samples out;
    out.reserve(dibits.size() * samples_per_symbol);

    const float dt = 1.0f / static_cast<float>(kSampleRate);

    for (uint8_t dibit : dibits) {
        const uint8_t tone_idx = (dibit < kNumTones) ? dibit : 0;
        const float freq = kToneFreqHz[tone_idx];
        const float phase_inc = kTwoPi * freq * dt;
        const float phase_inc_b = kTwoPi * kToneFreqHzDiversity[tone_idx] * dt;

        for (uint32_t n = 0; n < samples_per_symbol; ++n) {
            if (diversity_) {
                // Two phase-continuous tones at half amplitude: same peak as one tone.
                out.push_back(0.5f * kToneAmplitude * (std::sin(phase_) + std::sin(phase_b_)));
                phase_b_ += phase_inc_b;
            } else {
                out.push_back(kToneAmplitude * std::sin(phase_));
            }
            phase_ += phase_inc;
        }
        // Keep the phase accumulator small to preserve float precision over
        // long bursts. Modulo into [0, 2*pi).
        phase_ = wrapPhase(phase_);
        phase_b_ = wrapPhase(phase_b_);
    }
    return out;
}

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
