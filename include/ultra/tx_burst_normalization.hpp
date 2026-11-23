#pragma once

#include <cstddef>
#include <span>
#include <vector>

namespace ultra::sim {

inline constexpr float kTxBurstActiveThreshold = 1.0e-6f;
inline constexpr float kTxBurstPeakWarningThreshold = 0.95f;
inline constexpr float kTxBurstPeakClipThreshold = 1.0f;
// Empirical lower bound from the current operational modem TX paths locked by
// regression: the shortest valid burst is the light OFDM ACK active interval
// at 10080 samples. Shorter active intervals are callback/packet fragments,
// not PHY bursts, and must be moved upstream before normalization.
inline constexpr size_t kTxBurstMinimumActiveSamples = 10080;

inline constexpr float kHardwareTxDefaultPeakTarget = 0.5f;
// Primary-source industry baseline: one leading HF OFDM implementation
// normalizes modulator output to 16384 / 32767 = 0.50015 full scale.
// This is the -6 dBFS default peak target for real soundcard output.

inline constexpr float kHardwareTxMaxPeakTarget = 0.7f;
// Operator-tunable ceiling after clean no-ALC verification on the specific
// radio/audio chain. Above this, DAC clipping and radio ALC pumping risk rise
// sharply. 0.7 full scale is -3.1 dBFS.

inline constexpr float kSoftwareAlcMaxPeakTarget = 0.85f;
// Absolute digital ceiling for the CLOSED-LOOP software-ALC (2026-07-02,
// BUG-QAM16-RIG-LEVEL-BUDGET). Higher than the 0.7 operator ceiling because the
// loop — unlike a blind static setting — is guarded by the receiver's per-burst
// crest-factor clip detector, which pulls drive down -2 dB the moment the chain
// clips. 0.85 is -1.4 dBFS: still real headroom below the 0.95 peak-warning /
// 1.0 clip thresholds, and 0.5 -> 0.85 is the +4.6 dB walk that covers the
// measured ~4-5 dB unused level headroom on the rig. normalizeTxBurstForHardware
// clamps its target to THIS value; the operator settings slider stays clamped to
// kHardwareTxMaxPeakTarget.

inline constexpr float kHardwareTxMinPeakTarget = 0.05f;
// Practical lower bound for real soundcard drive: below this, DAC/input noise
// floor dominates and SNR collapses regardless of channel conditions.

// Simulator reference-band filter contract:
//   - Bandwidth: 50-2950 Hz, 48 kHz sample rate
//   - 101-tap Blackman-windowed FIR
//   - Power-normalized convention: a unit-RMS passband tone remains unit-RMS
//   - Used to define kModemReferenceInBandRms; receiver code may share these
//     coefficients, but the calibration contract is owned by the simulator
inline constexpr size_t kReferenceBandFirTaps = 101;
inline constexpr float kReferenceBandLowHz = 50.0f;
inline constexpr float kReferenceBandHighHz = 2950.0f;
inline constexpr float kReferenceSampleRateHz = 48000.0f;
const std::vector<float>& referenceBandFirCoefficients();

struct TxBurstRmsMeasurement {
    size_t active_begin = 0;       // First active-region sample (|x| > threshold)
    size_t active_end = 0;         // One past last active-region sample
    size_t active_samples = 0;     // active_end - active_begin
    float broadband_rms = 0.0f;    // RMS over [active_begin, active_end)
    float in_band_rms = 0.0f;      // RMS after referenceBandFir over active region
    // The burst's typical sustained in-band level: the median 50 ms window RMS over the active
    // region, near-silent windows excluded. A preamble shorter than half the burst (a chirp at
    // full peak over a DPSK payload ~5 dB below it) does not move it, so it is the level the
    // peer's payload SNR reading is taken on (the TX PAPR offset compares these, 2026-11-22).
    float in_band_typical_rms = 0.0f;
    float gain_to_reference = 1.0f; // kModemReferenceInBandRms / in_band_rms
    float peak_after_gain = 0.0f;  // Max |x * gain| over the full burst
    size_t peak_warning_samples = 0; // Full-burst samples with |x * gain| > 0.95
    size_t peak_clip_samples = 0;  // Full-burst samples with |x * gain| > 1.0
    bool peak_warning = false;     // peak_after_gain > 0.95
    bool peak_clip_error = false;  // peak_after_gain > 1.0
    bool burst_fragment_warning = false; // active_samples < kTxBurstMinimumActiveSamples
};

struct TxBurstHardwareMeasurement {
    size_t active_begin = 0;
    size_t active_end = 0;
    size_t active_samples = 0;
    float peak_before_gain = 0.0f;
    float gain_to_target = 1.0f;
    float peak_after_gain = 0.0f;
    float target_peak = 0.0f;
    bool burst_fragment_warning = false;
};

// Measure only. Does not modify the samples.
TxBurstRmsMeasurement measureTxBurstInBandRms(std::span<const float> samples);

// Active in-band signal-power normalization to the simulator reference level
// before channel/noise injection. The gain is derived from the active-region
// RMS, then applied to the whole burst including leading/trailing/embedded
// silence. Clipping after normalization is expected for high-PAPR bursts and
// matches real-radio PA overdrive behavior; it does not invalidate the SNR
// delivery contract because active in-band RMS energy is preserved within
// tolerance. A future expert absolute-noise-density mode can bypass the normal
// SNR-to-reference-noise formula without changing this measurement contract.
TxBurstRmsMeasurement normalizeTxBurstToReference(std::vector<float>& samples);

// Hardware peak normalization. Scales a whole burst so its maximum |sample|
// equals target_peak. Active-region detection uses kTxBurstActiveThreshold and
// kTxBurstMinimumActiveSamples, matching the simulator measurement contract. If
// the active region is shorter than the minimum, normalization is bypassed and
// the original samples are returned unchanged with burst_fragment_warning=true.
//
// PHY lens: peak normalization preserves waveform shape and PAPR character. It
// accepts lower average power on high-PAPR bursts because the hardware chain is
// peak-limited.
//
// DSP lens: one scalar is computed off the audio callback and applied to the
// full burst before queueing. Broadband peak, not in-band FIR RMS, is the
// operational quantity for DAC clip protection.
//
// Operator lens: target_peak is the soundcard output peak. The default 0.5
// leaves practical headroom; operators may tune upward only after confirming
// clean ALC behavior on their own station.
TxBurstHardwareMeasurement normalizeTxBurstForHardware(
    std::vector<float>& samples,
    float target_peak = kHardwareTxDefaultPeakTarget);

}  // namespace ultra::sim
