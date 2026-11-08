#include "frame_demodulator.hpp"

#include "ultra/dsp.hpp"       // HilbertTransform, SampleSpan, Complex
#include "ultra/logging.hpp"   // LOG_MODEM

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace ultra {
namespace gui {

// §7 C-FD-1: moved verbatim from StreamingDecoder::applyCFOPreCorrection (the Hilbert pre-corrector).
// Same DSP, same 0.75 Hz skip threshold, same per-sample rotation, same log → byte-identical; only
// pre_correction_cfo_ (now the member) and the log prefix (now a param) changed home.
float FrameDemodulator::applyCFOPreCorrection(std::vector<float>& samples, float cfo_hz,
                                              size_t absolute_start_sample, const char* log_prefix) {
    // The Hilbert pre-corrector operates on a finite frame whose first samples
    // are the OFDM LTS. At sub-Hz offsets the real CFO over one frame is
    // negligible, while the finite analytic-filter edge transient can imprint a
    // false per-carrier magnitude ripple on the LTS/pilots. Leave those small
    // offsets for the demodulator's complex baseband correction path.
    constexpr float kMinCFOForHilbertPreCorrectionHz = 0.75f;
    // DEFAULT OFF since 2026-11-08 (ULTRA_CFO_PRECORRECT=1 restores it for a rig A/B). This
    // pre-corrector builds its analytic signal with a 65-tap full-band Hilbert FIR whose
    // quadrature gain G is accurate only above ~fs*3/N = 2.2 kHz; across the modem band
    // (140-2906 Hz) every carrier keeps an image of relative level (1-G)/(1+G) — about -1.7 dB at
    // 140 Hz, -11 dB at 1 kHz — which the -CFO rotation moves to f + 2 CFO (14.6 Hz from its
    // carrier at 7.3 Hz: inter-carrier interference over the lower band). It also delays the
    // frame by 32 samples and corrupts its first 32 with the filter start-up. The demodulator
    // applies CFO EXACTLY in its complex NCO (toBaseband), so the pre-correction is not needed.
    // Measured (measure_ack_fer, Good@20 QPSK R2/3, true CFO 7.3 Hz forced): 13.5 % FER with
    // the pre-correction, 4.2 % without = the 0 Hz baseline. The GUI gate never ran it before
    // 2026-11-08: OTASim had 0 Hz CFO and the pre-correction skips |CFO| < 0.75 Hz.
    static const bool kPreCorrect = [] {
        const char* e = std::getenv("ULTRA_CFO_PRECORRECT");
        return e && e[0] == '1';
    }();
    if (!kPreCorrect || std::abs(cfo_hz) < kMinCFOForHilbertPreCorrectionHz || samples.empty()) {
        pre_correction_cfo_ = 0.0f;
        return 0.0f;  // Skip if negligible
    }

    // Convert to analytic signal (real + j*Hilbert) for proper frequency shift
    HilbertTransform hilbert(65);
    auto analytic = hilbert.process(SampleSpan(samples.data(), samples.size()));

    // Apply CFO correction: multiply analytic signal by exp(-j*2π*CFO*t)
    // Phase formula matches toBaseband(): phase_inc = -2π × CFO / fs per sample
    const float fs = 48000.0f;
    float phase_inc = -2.0f * static_cast<float>(M_PI) * cfo_hz / fs;
    float phase = -2.0f * static_cast<float>(M_PI) * cfo_hz *
                  static_cast<float>(absolute_start_sample) / fs;
    // Wrap to [-π, π]
    phase = std::fmod(phase, 2.0f * static_cast<float>(M_PI));
    if (phase > static_cast<float>(M_PI)) phase -= 2.0f * static_cast<float>(M_PI);
    if (phase < -static_cast<float>(M_PI)) phase += 2.0f * static_cast<float>(M_PI);

    // Apply rotation and take real part
    size_t len = std::min(samples.size(), analytic.size());
    for (size_t i = 0; i < len; i++) {
        Complex correction(std::cos(phase), std::sin(phase));
        samples[i] = (analytic[i] * correction).real();
        phase += phase_inc;
        if (phase > static_cast<float>(M_PI)) phase -= 2.0f * static_cast<float>(M_PI);
        else if (phase < -static_cast<float>(M_PI)) phase += 2.0f * static_cast<float>(M_PI);
    }

    pre_correction_cfo_ = cfo_hz;
    LOG_MODEM(DEBUG, "[%s] CFO pre-correction: %.2f Hz, abs_start=%zu, %zu samples",
              log_prefix, cfo_hz, absolute_start_sample, len);
    return cfo_hz;
}

}  // namespace gui
}  // namespace ultra
