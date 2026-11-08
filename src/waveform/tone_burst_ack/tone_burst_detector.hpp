// tone_burst_detector.hpp — matched-filter detector for the tone-burst ACK.
// PHY_ADAPTATION_DESIGN §15 step 2.
//
// Detection pipeline:
//   1. (optional) Costas correlation sweep to find timing (detectAndDecode).
//   2. Per-symbol Goertzel at the 4 tone frequencies; pick the strongest.
//   3. Dibits -> coded bits -> Hamming(15,11) decode -> CRC check -> payload.
//
// Timing search cost (2026-11-08): the coarse Costas sweep evaluates window DFTs at every
// sweep offset; recomputing a Goertzel per offset cost O(offsets x N) and, with the
// frequency-diversity subband doubling the tone count, took ~35 % of a desktop core per armed
// pass. The sweep now builds GRID PREFIX SUMS of the mixed signal per probed frequency (Goertzel
// recursion in double + one complex extraction per grid point) so every window DFT is a
// difference of two prefix values: O(window) per frequency. The fine sweep slides the four
// Costas-window DFTs one sample at a time. Magnitudes equal the per-window Goertzel to rounding.
//
// Carrier offset (2026-11-08): an SSB dial offset shifts every tone by the same delta. With
// no compensation a 7.3 Hz offset cost the 100 ms rung > 6 dB and 20 Hz cost the 25 ms rung
// > 4 dB (measure_ack_staircase, AWGN): the Goertzel main lobe is only 2/T wide. The
// detector now probes every tone at f + delta for a set of offset hypotheses around a PRIOR
// (setFrequencyPrior), spaced 1/(2T) (half a bin: worst-case scalloping sinc^2(1/4) =
// -0.91 dB) and spanning the prior's stated uncertainty.

#pragma once

#include "tone_burst_constants.hpp"
#include "tone_burst_payload.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace ultra {
namespace waveform {
namespace tone_burst_ack {

using SampleSpan = const float*;

struct DecodeAlignedResult {
    std::optional<ToneBurstAckPayload> payload;
    PayloadDecodeStats stats;
    // Per-payload-symbol confidence: ratio of strongest-tone magnitude to
    // second-strongest. 1.0 = tie (worst), >>1 = clearly the right tone.
    std::vector<float> per_symbol_confidence;
    // Aggregate confidence: minimum over the payload symbols. Useful as a
    // single-number quality metric.
    float min_confidence = 0.0f;
};

struct DetectAndDecodeResult {
    DecodeAlignedResult decode;
    // Index of the FIRST sample of the FIRST Costas symbol within the
    // input buffer. Only meaningful when costas_correlation_peak > threshold.
    size_t detected_offset_samples = 0;
    // Carrier-offset hypothesis (Hz) the reported burst was decoded at (or, without a decode,
    // the hypothesis of the strongest timing candidate), and how many were searched.
    float detected_frequency_offset_hz = 0.0f;
    int frequency_hypotheses = 1;
    // Costas-pattern matched-filter peak energy. Higher = stronger
    // pattern lock. Detection threshold is symbol_ms-dependent.
    float costas_correlation_peak = 0.0f;
    bool sync_acquired = false;

    // A CRC-valid payload is not necessarily meaningful in the current protocol
    // window (CRC-12 collisions are rare per candidate, but the timing search tries
    // a family of candidates).  When an acceptance predicate is supplied below, the
    // detector keeps searching after such a candidate instead of returning it as a
    // successful physical ACK.  Preserve the strongest rejection for monitor-level
    // diagnostics without making the predicate itself stateful.
    size_t semantic_rejections = 0;
    std::optional<ToneBurstAckPayload> strongest_rejected_payload;
    size_t strongest_rejected_offset_samples = 0;
    float strongest_rejected_correlation_peak = 0.0f;
    float strongest_rejected_min_confidence = 0.0f;
    int strongest_rejected_hamming_corrected_blocks = 0;
};

using ToneBurstAckAcceptancePredicate =
    std::function<bool(const ToneBurstAckPayload&)>;

class ToneBurstDetector {
public:
    ToneBurstDetector() : diversity_(toneAckDiversityEnabled()) {}

    // Square-law combine the diversity subband (kToneFreqHzDiversity) into every tone
    // statistic. Defaults to toneAckDiversityEnabled().
    void setDiversity(bool on) { diversity_ = on; }
    bool diversity() const { return diversity_; }

    // Carrier-offset prior: the expected shift (Hz) of every tone and the half-width (Hz) of
    // its uncertainty. detectAndDecode searches prior + j * step, step =
    // frequencyHypothesisStepHz(symbol_ms), |j| <= frequencyHypothesesPerSide(halfwidth,
    // symbol_ms); halfwidth 0 = a single hypothesis at the prior. decodeAligned demaps at the
    // prior. Default 0 / 0 = the legacy fixed-frequency detector.
    void setFrequencyPrior(float offset_hz, float halfwidth_hz) {
        freq_prior_hz_ = offset_hz;
        freq_halfwidth_hz_ = (halfwidth_hz > 0.0f) ? halfwidth_hz : 0.0f;
    }
    float frequencyPriorHz() const { return freq_prior_hz_; }
    float frequencyHalfWidthHz() const { return freq_halfwidth_hz_; }
    // Half a Goertzel bin, 1 / (2 T): the hypothesis spacing.
    static float frequencyHypothesisStepHz(uint32_t symbol_ms) {
        return (symbol_ms > 0) ? 1000.0f / (2.0f * static_cast<float>(symbol_ms)) : 0.0f;
    }
    // Smallest J with J * step + step / 2 >= halfwidth, i.e. every offset inside the prior's
    // interval lies within half a step of a hypothesis.
    static int frequencyHypothesesPerSide(float halfwidth_hz, uint32_t symbol_ms);

    // Demap and decode an ALIGNED tone-burst. samples[0..kTotalSymbols*N-1]
    // must be the on-air burst starting at the first Costas symbol.
    // Used for tests + post-sync demap.
    DecodeAlignedResult decodeAligned(SampleSpan samples,
                                      size_t num_samples,
                                      uint32_t symbol_ms = kBaselineSymbolMs) {
        return decodeAlignedAt(samples, num_samples, symbol_ms, freq_prior_hz_);
    }
    // As decodeAligned, with every tone shifted by offset_hz.
    DecodeAlignedResult decodeAlignedAt(SampleSpan samples, size_t num_samples,
                                        uint32_t symbol_ms, float offset_hz);

    // Sweep `samples` for the Costas pattern, then demap+decode the burst.
    // Used when timing is uncertain (sender's "expected window" + guard).
    // `sweep_step_samples` controls the timing-search granularity (smaller
    // = finer + slower).
    DetectAndDecodeResult detectAndDecode(SampleSpan samples,
                                          size_t num_samples,
                                          uint32_t symbol_ms = kBaselineSymbolMs,
                                          uint32_t sweep_step_samples = 8,
                                          const ToneBurstAckAcceptancePredicate&
                                              acceptance_predicate = {});

    // ----- helpers exposed for testing -----

    // Compute Goertzel magnitudes for all 4 tones over a sample window.
    // Caller provides the window (single symbol worth of samples).
    static std::vector<float> toneMagnitudes(SampleSpan window_samples,
                                             size_t window_samples_count,
                                             bool diversity = false,
                                             float offset_hz = 0.0f);

    // Pick the strongest of 4 tone magnitudes and return its dibit index +
    // confidence (strongest/second-strongest magnitude ratio).
    static uint8_t demapDibit(const std::vector<float>& magnitudes,
                              float& confidence);

    // Costas correlation at a given timing offset. Returns the sum of
    // expected-tone Goertzel magnitudes over the 4 Costas symbols. Higher
    // = better sync. samples[start_offset..start_offset+4*N-1] is used.
    static float costasCorrelationAtOffset(SampleSpan samples,
                                           size_t num_samples,
                                           size_t start_offset,
                                           uint32_t samples_per_symbol,
                                           bool diversity = false,
                                           float offset_hz = 0.0f);

private:
    bool diversity_ = true;
    float freq_prior_hz_ = 0.0f;
    float freq_halfwidth_hz_ = 0.0f;
};

}  // namespace tone_burst_ack
}  // namespace waveform
}  // namespace ultra
