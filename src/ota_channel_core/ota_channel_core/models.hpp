#pragma once

#include "ota_channel_core/rng.hpp"

#include <complex>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string_view>
#include <vector>

namespace ultra::ota_channel_core {

inline constexpr uint32_t kDefaultSampleRate = 48000;
inline constexpr float kModemReferenceBroadbandRms = 0.3180724f;
inline constexpr float kModemReferenceInBandRms = 0.30482664f;
inline constexpr float kModemReferenceRms = kModemReferenceInBandRms;
inline constexpr double kModemReferencePower =
    static_cast<double>(kModemReferenceRms) *
    static_cast<double>(kModemReferenceRms);
inline constexpr double kModemInBandNoisePowerFraction = 0.10858718;
inline constexpr double kModemBroadbandToInBandSnrOffsetDb = 9.64221445;

enum class ChannelType {
    PASSTHROUGH,
    AWGN,
    GOOD,
    MODERATE,
    POOR,
    FLUTTER,
    REAL_HF_LOOP
};

struct ChannelConfig {
    ChannelType type = ChannelType::PASSTHROUGH;
    float snr_db = 20.0f;
    uint64_t seed = 42;
    uint32_t sample_rate = kDefaultSampleRate;
    std::shared_ptr<const std::vector<float>> real_hf_loop_noise;
    // Carrier frequency offset (Hz) the RECEIVER sees on this path: the sender's oscillator
    // error minus the receiver's (SSB dial offset). 0 = no shift (byte-identical models). A
    // non-zero value wraps the model in FrequencyShiftChannelModel (streaming, continuous).
    float cfo_hz = 0.0f;
    // Sample-clock offset (ppm) the RECEIVER sees on this path: the sender's soundcard clock
    // error minus the receiver's. 0 = no time scaling (byte-identical models). A non-zero value
    // wraps the model (outermost) in ClockOffsetChannelModel.
    float clock_ppm = 0.0f;
};

float modemReferenceNoiseStddev(float snr_db);

// The FIR that DEFINES the in-band noise reference. Exposed 2026-07-29 (Phase 0)
// so the noise convention is testable rather than asserted: kModemInBandNoise-
// PowerFraction is the fraction of broadband white-noise power this filter
// retains, and kModemBroadbandToInBandSnrOffsetDb is 10*log10 of its reciprocal.
//
// IMPORTANT: this is a NOISE-REFERENCING CONVENTION, not a stage in the RX signal
// path. ModemEngine constructs and resets an rx_filter_ but never applies it; only
// tx_filter_ is applied. Do not mistake this for receive filtering.
std::vector<float> modemBandpassFirCoefficients();
const char* channelTypeName(ChannelType type);
std::optional<ChannelType> parseChannelType(std::string_view value);
std::vector<float> loadRealHfLoopNoiseBedWav(std::string_view path);

class IChannelModel {
public:
    virtual ~IChannelModel() = default;
    virtual void reset() = 0;
    virtual void process(std::span<const float> input,
                         uint64_t start_sample,
                         std::vector<float>& output) = 0;

    void process(std::span<const float> input, std::vector<float>& output) {
        process(input, 0, output);
    }

    std::vector<float> process(std::span<const float> input) {
        std::vector<float> output;
        process(input, output);
        return output;
    }

    std::vector<float> process(std::span<const float> input, uint64_t start_sample) {
        std::vector<float> output;
        process(input, start_sample, output);
        return output;
    }
};

// Streaming carrier-frequency offset applied to the OUTPUT of any channel model (a dial offset
// between two transceivers). Analytic signal from a band-limited Hilbert FIR pair (Blackman,
// cutoff 3050 Hz), then rotation by a continuous phase: every frequency moves by +cfo_hz with no
// block-edge artifacts. (WattersonChannel::applyCFO is block-wise — one FFT analytic signal per
// process() call, circular at every block edge — so OTASim never enabled it; this replaces it.)
// 4097 taps: the Hilbert transition from DC is ~70 Hz wide, so the image is < -40 dB from the
// lowest OFDM carrier (140.6 Hz) up (the 1793-tap fading-path design left -29 dB at 160 Hz).
// Adds the FIR's constant group delay, (taps-1)/2 = 2048 samples (42.7 ms at 48 kHz).
class FrequencyShiftChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    FrequencyShiftChannelModel(std::unique_ptr<IChannelModel> inner, float cfo_hz,
                               uint32_t sample_rate);

    void reset() override;
    void process(std::span<const float> input,
                 uint64_t start_sample,
                 std::vector<float>& output) override;

    float cfoHz() const { return cfo_hz_; }
    static size_t groupDelaySamples();

private:
    std::unique_ptr<IChannelModel> inner_;
    float cfo_hz_ = 0.0f;
    double phase_inc_ = 0.0;
    double phase_ = 0.0;
    std::vector<float> g_inphase_;     // reversed taps (contiguous dot product)
    std::vector<float> g_quadrature_;
    std::vector<float> line_;          // doubled delay line, length 2 x taps
    size_t idx_ = 0;
    std::vector<float> inner_out_;
};

// Streaming SAMPLE-CLOCK offset between two soundcards, applied to the OUTPUT of any channel
// model. Receiver sample n reads the sender's stream at position p0 + n * (1 + ppm * 1e-6): a
// sender clock that runs fast by ppm delivers its stream time-compressed (a 10 s burst at
// +50 ppm arrives 24 samples short, its symbol timing drifts 0.5 ms across the burst) and every
// tone shifted up by f * ppm * 1e-6. That drift — not the tiny tone shift — is what a modem's
// timing recovery and burst-start prediction must survive on two real radios.
//
// Interpolator: 32-tap Kaiser(beta 8) windowed sinc, 512 phases blended linearly; each phase is
// normalized to unit DC gain. The modem band (< 3 kHz at 48 kHz) sits far inside its passband.
// The read position is computed from the absolute output count (p0 + n * step), so the output
// does not depend on how the stream is cut into process() calls and no rounding accumulates.
//
// Latency: the link starts kGuard samples late. For ppm > 0 the read position catches up with
// the stream at ppm * 1e-6 samples per sample, so the guard holds kClockOffsetMaxSessionS of
// drift (900 s: 2160 samples = 45 ms at +50 ppm); past it the model re-establishes the guard
// (counted in slips(), an xrun-like discontinuity). For ppm < 0 the latency grows instead. The
// reverse link of a two-station session drifts the opposite way, so a round trip is constant to
// first order. Per-RECEIVER convention like cfo_hz: exact for two stations.
class ClockOffsetChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    static constexpr int kHalfTaps = 16;
    static constexpr int kPhases = 512;
    static constexpr double kClockOffsetMaxSessionS = 900.0;

    ClockOffsetChannelModel(std::unique_ptr<IChannelModel> inner, float ppm,
                            uint32_t sample_rate);

    void reset() override;
    void process(std::span<const float> input,
                 uint64_t start_sample,
                 std::vector<float>& output) override;

    float ppm() const { return ppm_; }
    size_t initialLatencySamples() const { return guard_; }
    uint64_t slips() const { return slips_; }

private:
    std::unique_ptr<IChannelModel> inner_;
    float ppm_ = 0.0f;
    double step_ = 1.0;
    size_t guard_ = 0;
    std::vector<float> table_;   // (kPhases + 1) rows of 2 * kHalfTaps taps
    std::vector<float> buf_;     // inner samples [buf_base_, written_)
    int64_t buf_base_ = 0;
    int64_t written_ = 0;
    double origin_ = 0.0;        // read position of output sample 0 (moves only on a slip)
    int64_t out_count_ = 0;
    uint64_t slips_ = 0;
    std::vector<float> inner_out_;
};

class PassthroughChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    void reset() override {}
    void process(std::span<const float> input,
                 uint64_t start_sample,
                 std::vector<float>& output) override;
};

class AWGNChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    AWGNChannelModel(float snr_db, uint32_t seed);

    void reset() override {}
    void setSNR(float snr_db);
    void process(std::span<const float> input,
                 uint64_t start_sample,
                 std::vector<float>& output) override;

private:
    float noise_stddev_ = 0.0f;
    uint32_t seed_ = 0;
};

class RealHfLoopChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    RealHfLoopChannelModel(float snr_db,
                           std::vector<float> normalized_loop,
                           uint64_t seed_for_phase = 0);
    RealHfLoopChannelModel(float snr_db,
                           std::shared_ptr<const std::vector<float>> normalized_loop,
                           uint64_t seed_for_phase = 0);

    void reset() override;
    void setSNR(float snr_db);
    void process(std::span<const float> input,
                 uint64_t start_sample,
                 std::vector<float>& output) override;

private:
    float scale_ = 0.0f;
    double loop_in_band_power_ = 0.0;
    std::shared_ptr<const std::vector<float>> loop_;
    size_t start_position_ = 0;
    size_t position_ = 0;
};

class WattersonChannel {
public:
    struct Config {
        float snr_db = 15.0f;
        float delay_spread_ms = 2.0f;
        float doppler_spread_hz = 1.0f;
        float cfo_hz = 0.0f;
        float random_cfo_max_hz = 0.0f;
        float path1_gain = 0.707f;
        float path2_gain = 0.707f;
        uint32_t sample_rate = kDefaultSampleRate;
        bool fading_enabled = true;
        bool multipath_enabled = true;
        bool noise_enabled = true;
        bool cfo_enabled = true;
    };

    explicit WattersonChannel(const Config& config, uint64_t seed = 42);

    void reset();
    void setSNR(float snr_db);
    void process(std::span<const float> input, std::vector<float>& output);
    std::vector<float> process(std::span<const float> input);

    // 2026-07-29 Phase 0 (EFFECTIVE_SINR handoff §9): expose simulator truth
    // H[k,t]. The tap structure in processWithComplexFading is exactly
    //     out(n) = x_a(n)*g1*a1(n) + x_a(n-D)*g2*a2(n)
    // so the instantaneous frequency response at the CURRENT tap state is
    //     H(f) = g1*a1 + g2*a2*exp(-j*2*pi*f*D/fs).
    // This is ground truth for estimator NMSE, which is the measurement the
    // whole "is channel estimation the limiter" question turns on. Both prior
    // oracles were invalid (see src/ofdm/genie_true_h.hpp); this replaces them
    // with the channel model's own state rather than a second decode pass.
    //
    // NOTE ON DOMAIN: the returned H does NOT include the analytic (Hilbert)
    // filter's response or its group delay, nor the receiver's FFT-window
    // timing convention. Comparisons against a receiver estimate must fit out a
    // global complex scale and a linear phase ramp first -- those are exactly
    // the conventions the equalizer is invariant to.
    std::complex<float> trueFrequencyResponse(float freq_hz) const;
    size_t multipathDelaySamples() const { return delay_samples_; }

    float actualCFO() const { return actual_cfo_hz_; }
    float fadingMagnitude() const;
    std::complex<float> fadingTap1ForDiagnostics() const { return fading1_; }
    std::complex<float> fadingTap2ForDiagnostics() const { return fading2_; }
    float fadingAlphaForDiagnostics() const { return fading_alpha_; }
    float fadingSigmaHzForDiagnostics() const { return fading_sigma_hz_; }
    size_t fadingSosOscillatorCountForDiagnostics() const {
        return fading1_oscillators_.size();
    }
    void stepFadingForDiagnostics();
    void setFadingTapsForDiagnostics(std::complex<float> tap1,
                                      std::complex<float> tap2);
    static std::vector<std::complex<float>> applyComplexMultipathForDiagnostics(
        std::span<const std::complex<float>> input,
        size_t delay_samples,
        std::complex<float> tap1,
        std::complex<float> tap2,
        float path1_gain,
        float path2_gain);
    const Config& config() const { return config_; }

private:
    struct FadingOscillator {
        std::complex<double> phasor{1.0, 0.0};
        std::complex<double> initial_phasor{1.0, 0.0};
        std::complex<double> rotation{1.0, 0.0};
        double amplitude = 0.0;
    };

    void initializeHilbert();
    void initializeGaussianDoppler(uint64_t seed);
    void initializeGaussianDopplerTap(std::vector<FadingOscillator>& oscillators,
                                      std::mt19937& rng);
    void resetFadingOscillators();
    std::complex<float> evaluateFadingTap(std::vector<FadingOscillator>& oscillators);
    std::complex<float> currentFadingTap(const std::vector<FadingOscillator>& oscillators) const;
    void renormalizeFadingOscillators();
    std::complex<float> analyticSample(float sample);
    void processWithoutFading(std::span<const float> input,
                              std::vector<float>& output);
    void processWithComplexFading(std::span<const float> input,
                                  std::vector<float>& output);
    void updateFading();
    void applyCFO(std::vector<float>& samples);

    Config config_;
    std::mt19937 rng_;
    std::normal_distribution<float> gaussian_{0.0f, 1.0f};
    std::deque<float> delay_line_;
    std::deque<std::complex<float>> analytic_delay_line_;
    std::vector<float> hilbert_inphase_coeffs_;
    std::vector<float> hilbert_coeffs_;
    std::vector<float> hilbert_delay_line_;
    std::vector<FadingOscillator> fading1_oscillators_;
    std::vector<FadingOscillator> fading2_oscillators_;
    size_t delay_samples_ = 0;
    size_t hilbert_delay_idx_ = 0;
    size_t hilbert_delay_samples_ = 0;
    float noise_stddev_ = 0.0f;
    float fading_alpha_ = 0.0f;
    float fading_sigma_hz_ = 0.0f;
    std::complex<float> fading1_{1.0f, 0.0f};
    std::complex<float> fading2_{1.0f, 0.0f};
    uint64_t fading_sample_index_ = 0;
    bool diagnostic_fading_frozen_ = false;
    float cfo_phase_ = 0.0f;
    float cfo_phase_inc_ = 0.0f;
    float actual_cfo_hz_ = 0.0f;
};

class WattersonChannelModel final : public IChannelModel {
public:
    using IChannelModel::process;

    WattersonChannelModel(const WattersonChannel::Config& config, uint64_t seed);

    void reset() override;
    void setSNR(float snr_db);
    void process(std::span<const float> input,
                 uint64_t start_sample,
                 std::vector<float>& output) override;

    const WattersonChannel::Config& config() const;

    // 2026-07-29 Phase 0 diag: reach the tap state so truth H[k,t] can be read.
    const WattersonChannel& channelForDiagnostics() const { return channel_; }

private:
    WattersonChannel channel_;
};

namespace itu_r_f1487 {
WattersonChannel::Config good(float snr_db = 20.0f);
WattersonChannel::Config moderate(float snr_db = 20.0f);
WattersonChannel::Config poor(float snr_db = 20.0f);
WattersonChannel::Config flutter(float snr_db = 20.0f);
WattersonChannel::Config awgn(float snr_db = 20.0f);
}  // namespace itu_r_f1487

WattersonChannel::Config configForWatterson(ChannelType type,
                                            float snr_db,
                                            uint32_t sample_rate = kDefaultSampleRate);
std::unique_ptr<IChannelModel> createChannelModel(const ChannelConfig& config,
                                                  const RngRoot& rng_root,
                                                  std::string_view stream_name);

}  // namespace ultra::ota_channel_core
