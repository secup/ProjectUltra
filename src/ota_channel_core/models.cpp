#include "ota_channel_core/models.hpp"
#include "ultra/tx_burst_normalization.hpp"

#define POCKETFFT_CACHE_SIZE 64
#if defined(__APPLE__) || defined(__unix__)
#define POCKETFFT_USE_POSIX_MEMALIGN
#endif

#include "pocketfft/pocketfft_hdronly.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace ultra::ota_channel_core {
namespace {

using Complex = std::complex<float>;

constexpr float kPi = 3.14159265358979323846f;
constexpr uint16_t kWavFormatPcm = 1;
constexpr uint16_t kRealHfLoopChannels = 1;
constexpr uint16_t kRealHfLoopBitsPerSample = 16;
constexpr size_t kWattersonHilbertTaps = 1793;
constexpr float kWattersonAnalyticLowpassHz = 3050.0f;
constexpr size_t kGaussianDopplerSosOscillators = 128;
constexpr uint64_t kGaussianDopplerRenormalizeInterval = 4096;
constexpr size_t kRealHfLoopPowerProbeSamples =
    static_cast<size_t>(kDefaultSampleRate) * 10u;

double unitFromU64(uint64_t value) {
    constexpr double kScale = 1.0 / 9007199254740992.0;  // 2^53
    return (static_cast<double>(value >> 11) + 0.5) * kScale;
}

float deterministicNormal(uint32_t seed, uint64_t sample_index) {
    const uint64_t key = (static_cast<uint64_t>(seed) << 32) ^
                         splitmix64(sample_index);
    const double u1 = std::max(
        unitFromU64(splitmix64(key ^ 0xd1b54a32d192ed03ull)),
        std::numeric_limits<double>::min());
    const double u2 = unitFromU64(splitmix64(key ^ 0xabc98388fb8fac03ull));
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double theta = 6.28318530717958647692 * u2;
    return static_cast<float>(r * std::cos(theta));
}

double inverseNormalCdf(double p) {
    p = std::clamp(p, std::numeric_limits<double>::min(),
                   1.0 - std::numeric_limits<double>::epsilon());

    constexpr double a[] = {
        -3.969683028665376e+01,  2.209460984245205e+02,
        -2.759285104469687e+02,  1.383577518672690e+02,
        -3.066479806614716e+01,  2.506628277459239e+00};
    constexpr double b[] = {
        -5.447609879822406e+01,  1.615858368580409e+02,
        -1.556989798598866e+02,  6.680131188771972e+01,
        -1.328068155288572e+01};
    constexpr double c[] = {
        -7.784894002430293e-03, -3.223964580411365e-01,
        -2.400758277161838e+00, -2.549732539343734e+00,
         4.374664141464968e+00,  2.938163982698783e+00};
    constexpr double d[] = {
         7.784695709041462e-03,  3.224671290700398e-01,
         2.445134137142996e+00,  3.754408661907416e+00};
    constexpr double p_low = 0.02425;
    constexpr double p_high = 1.0 - p_low;

    if (p < p_low) {
        const double q = std::sqrt(-2.0 * std::log(p));
        return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q +
                  c[4]) * q + c[5]) /
               ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }
    if (p > p_high) {
        const double q = std::sqrt(-2.0 * std::log(1.0 - p));
        return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q +
                   c[4]) * q + c[5]) /
                ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0);
    }

    const double q = p - 0.5;
    const double r = q * q;
    return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r +
              a[4]) * r + a[5]) * q /
           (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r +
              b[4]) * r + 1.0);
}

float modemReferenceBroadbandNoiseStddev(float broadband_snr_db) {
    // The SNR reference is the PING's receiver in-band RMS. The "broadband"
    // part of this helper names the generated white-noise sigma before the
    // receiver bandpass removes out-of-band noise.
    return kModemReferenceRms *
           std::pow(10.0f, -broadband_snr_db / 20.0f);
}


float processFirSample(float sample,
                       const std::vector<float>& coeffs,
                       std::vector<float>& delay_line,
                       size_t& delay_idx) {
    delay_line[delay_idx] = sample;

    float out = 0.0f;
    size_t j = delay_idx;
    for (size_t i = 0; i < coeffs.size(); ++i) {
        out += coeffs[i] * delay_line[j];
        if (j == 0) {
            j = coeffs.size();
        }
        --j;
    }

    delay_idx = (delay_idx + 1) % coeffs.size();
    return out;
}

double measureLoopInBandPower(std::span<const float> loop) {
    if (loop.empty()) {
        return 0.0;
    }

    const std::vector<float> coeffs = modemBandpassFirCoefficients();
    std::vector<float> delay_line(coeffs.size(), 0.0f);
    size_t delay_idx = 0;

    const size_t measurement_samples =
        std::min(std::max(loop.size(), static_cast<size_t>(kDefaultSampleRate)),
                 kRealHfLoopPowerProbeSamples);
    const size_t warmup_samples = coeffs.size();
    double sum_sq = 0.0;
    for (size_t i = 0; i < warmup_samples + measurement_samples; ++i) {
        const float filtered = processFirSample(loop[i % loop.size()],
                                                coeffs,
                                                delay_line,
                                                delay_idx);
        if (i >= warmup_samples) {
            sum_sq += static_cast<double>(filtered) * static_cast<double>(filtered);
        }
    }
    return sum_sq / static_cast<double>(measurement_samples);
}

void analyticFrequencyShift(std::vector<float>& samples,
                            float cfo_hz,
                            uint32_t sample_rate,
                            float& phase_acc) {
    if (samples.empty() || std::abs(cfo_hz) < 0.001f) {
        return;
    }

    size_t fft_size = 1;
    while (fft_size < samples.size()) {
        fft_size <<= 1;
    }

    std::vector<Complex> time(fft_size, Complex(0.0f, 0.0f));
    std::vector<Complex> freq(fft_size, Complex(0.0f, 0.0f));
    std::vector<Complex> analytic(fft_size, Complex(0.0f, 0.0f));
    for (size_t i = 0; i < samples.size(); ++i) {
        time[i] = Complex(samples[i], 0.0f);
    }

    const pocketfft::shape_t shape{fft_size};
    const pocketfft::stride_t stride{static_cast<ptrdiff_t>(sizeof(Complex))};
    const pocketfft::shape_t axes{0};
    pocketfft::c2c<float>(shape, stride, stride, axes,
                          pocketfft::FORWARD, time.data(), freq.data(), 1.0f);

    if (fft_size >= 2) {
        for (size_t i = 1; i < fft_size / 2; ++i) {
            freq[i] *= 2.0f;
        }
        for (size_t i = fft_size / 2 + 1; i < fft_size; ++i) {
            freq[i] = Complex(0.0f, 0.0f);
        }
    }

    pocketfft::c2c<float>(shape, stride, stride, axes,
                          pocketfft::BACKWARD, freq.data(), analytic.data(),
                          1.0f / static_cast<float>(fft_size));

    const float phase_inc = 2.0f * kPi * cfo_hz /
                            static_cast<float>(sample_rate);
    float phase = phase_acc;
    for (size_t i = 0; i < samples.size(); ++i) {
        const Complex rot(std::cos(phase), std::sin(phase));
        samples[i] = std::real(analytic[i] * rot);
        phase += phase_inc;
        if (phase > kPi) {
            phase -= 2.0f * kPi;
        } else if (phase < -kPi) {
            phase += 2.0f * kPi;
        }
    }
    phase_acc = phase;
}

uint16_t readLe16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           static_cast<uint16_t>(p[1] << 8);
}

uint32_t readLe32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

bool readExact(std::istream& in, void* dst, size_t len) {
    in.read(static_cast<char*>(dst), static_cast<std::streamsize>(len));
    return static_cast<size_t>(in.gcount()) == len;
}

std::string normalizedChannelToken(std::string_view value) {
    std::string out;
    out.reserve(value.size());
    for (const char c : value) {
        out.push_back(c == '-'
            ? '_'
            : static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}

std::runtime_error realHfLoopWavError(std::string_view path,
                                      const std::string& detail) {
    return std::runtime_error("real_hf_loop noise-bed WAV " +
                              std::string(path) + ": " + detail);
}

}  // namespace

std::vector<float> modemBandpassFirCoefficients() {
    const auto& coeffs = ultra::sim::referenceBandFirCoefficients();
    return std::vector<float>(coeffs.begin(), coeffs.end());
}

float modemReferenceNoiseStddev(float snr_db) {
    return modemReferenceBroadbandNoiseStddev(
        snr_db - static_cast<float>(kModemBroadbandToInBandSnrOffsetDb));
}

const char* channelTypeName(ChannelType type) {
    switch (type) {
        case ChannelType::PASSTHROUGH: return "passthrough";
        case ChannelType::AWGN:        return "awgn";
        case ChannelType::GOOD:        return "good";
        case ChannelType::MODERATE:    return "moderate";
        case ChannelType::POOR:        return "poor";
        case ChannelType::FLUTTER:     return "flutter";
        case ChannelType::REAL_HF_LOOP:return "real_hf_loop";
        default:                       return "unknown";
    }
}

std::optional<ChannelType> parseChannelType(std::string_view value) {
    const std::string v = normalizedChannelToken(value);
    if (v.empty() || v == "passthrough" || v == "null") {
        return ChannelType::PASSTHROUGH;
    }
    if (v == "awgn") {
        return ChannelType::AWGN;
    }
    if (v == "good" || v == "watterson_good") {
        return ChannelType::GOOD;
    }
    if (v == "moderate" || v == "watterson_moderate") {
        return ChannelType::MODERATE;
    }
    if (v == "poor" || v == "watterson_poor") {
        return ChannelType::POOR;
    }
    if (v == "flutter" || v == "watterson_flutter") {
        return ChannelType::FLUTTER;
    }
    if (v == "real_hf_loop") {
        return ChannelType::REAL_HF_LOOP;
    }
    return std::nullopt;
}

std::vector<float> loadRealHfLoopNoiseBedWav(std::string_view path_view) {
    const std::string path(path_view);
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw realHfLoopWavError(path_view, "failed to open");
    }

    uint8_t riff_header[12] = {};
    if (!readExact(in, riff_header, sizeof(riff_header)) ||
        std::memcmp(riff_header, "RIFF", 4) != 0 ||
        std::memcmp(riff_header + 8, "WAVE", 4) != 0) {
        throw realHfLoopWavError(path_view, "not a RIFF/WAVE file");
    }

    std::vector<uint8_t> fmt_chunk;
    std::vector<uint8_t> data_chunk;
    while (in) {
        uint8_t chunk_header[8] = {};
        if (!readExact(in, chunk_header, sizeof(chunk_header))) {
            break;
        }
        const uint32_t chunk_size = readLe32(chunk_header + 4);
        std::vector<uint8_t> chunk(chunk_size);
        if (chunk_size > 0 && !readExact(in, chunk.data(), chunk_size)) {
            throw realHfLoopWavError(path_view, "truncated chunk");
        }
        if (chunk_size & 1u) {
            in.seekg(1, std::ios::cur);
        }

        if (std::memcmp(chunk_header, "fmt ", 4) == 0) {
            fmt_chunk = std::move(chunk);
        } else if (std::memcmp(chunk_header, "data", 4) == 0) {
            data_chunk = std::move(chunk);
        }
    }

    if (fmt_chunk.size() < 16) {
        throw realHfLoopWavError(path_view, "missing fmt chunk");
    }
    if (data_chunk.empty()) {
        throw realHfLoopWavError(path_view, "missing data chunk");
    }

    const uint16_t format = readLe16(fmt_chunk.data());
    const uint16_t channels = readLe16(fmt_chunk.data() + 2);
    const uint32_t sample_rate = readLe32(fmt_chunk.data() + 4);
    const uint16_t block_align = readLe16(fmt_chunk.data() + 12);
    const uint16_t bits = readLe16(fmt_chunk.data() + 14);
    if (format != kWavFormatPcm ||
        channels != kRealHfLoopChannels ||
        sample_rate != kDefaultSampleRate ||
        bits != kRealHfLoopBitsPerSample ||
        block_align != sizeof(int16_t)) {
        throw realHfLoopWavError(
            path_view,
            "must be 16-bit PCM mono 48000 Hz; no resampling is applied");
    }
    if ((data_chunk.size() % sizeof(int16_t)) != 0) {
        throw realHfLoopWavError(path_view, "data chunk is not aligned to 16-bit samples");
    }

    std::vector<float> samples;
    samples.reserve(data_chunk.size() / sizeof(int16_t));
    double sum_squares = 0.0;
    for (size_t i = 0; i < data_chunk.size(); i += sizeof(int16_t)) {
        const int16_t raw = static_cast<int16_t>(readLe16(data_chunk.data() + i));
        const float sample = static_cast<float>(raw) / 32768.0f;
        samples.push_back(sample);
        sum_squares += static_cast<double>(sample) * static_cast<double>(sample);
    }
    if (samples.empty()) {
        throw realHfLoopWavError(path_view, "contains no samples");
    }

    const double rms = std::sqrt(sum_squares / static_cast<double>(samples.size()));
    if (!(rms > std::numeric_limits<double>::min())) {
        throw realHfLoopWavError(path_view, "RMS is zero");
    }
    const float inv_rms = static_cast<float>(1.0 / rms);
    for (float& sample : samples) {
        sample *= inv_rms;
    }
    return samples;
}

void PassthroughChannelModel::process(std::span<const float> input,
                                      uint64_t,
                                      std::vector<float>& output) {
    output.assign(input.begin(), input.end());
}

AWGNChannelModel::AWGNChannelModel(float snr_db, uint32_t seed)
    : seed_(seed) {
    setSNR(snr_db);
}

void AWGNChannelModel::setSNR(float snr_db) {
    noise_stddev_ = modemReferenceNoiseStddev(snr_db);
}

void AWGNChannelModel::process(std::span<const float> input,
                               uint64_t start_sample,
                               std::vector<float>& output) {
    output.resize(input.size());
    for (size_t i = 0; i < input.size(); ++i) {
        output[i] = input[i] + noise_stddev_ *
                                 deterministicNormal(seed_, start_sample + i);
    }
}

RealHfLoopChannelModel::RealHfLoopChannelModel(float snr_db,
                                               std::vector<float> normalized_loop,
                                               uint64_t seed_for_phase)
    : RealHfLoopChannelModel(
          snr_db,
          std::make_shared<const std::vector<float>>(std::move(normalized_loop)),
          seed_for_phase) {}

RealHfLoopChannelModel::RealHfLoopChannelModel(
    float snr_db,
    std::shared_ptr<const std::vector<float>> normalized_loop,
    uint64_t seed_for_phase)
    : loop_(std::move(normalized_loop)) {
    if (loop_ && !loop_->empty() && seed_for_phase != 0) {
        start_position_ = static_cast<size_t>(
            seed_for_phase % static_cast<uint64_t>(loop_->size()));
        position_ = start_position_;
    }
    if (loop_ && !loop_->empty()) {
        loop_in_band_power_ = measureLoopInBandPower(*loop_);
    }
    setSNR(snr_db);
}

void RealHfLoopChannelModel::reset() {
    position_ = start_position_;
}

void RealHfLoopChannelModel::setSNR(float snr_db) {
    if (!loop_ || loop_->empty() ||
        !(loop_in_band_power_ > std::numeric_limits<double>::min())) {
        scale_ = 0.0f;
        return;
    }
    const double target_in_band_power =
        kModemReferencePower * std::pow(10.0, -static_cast<double>(snr_db) / 10.0);
    scale_ = static_cast<float>(
        std::sqrt(target_in_band_power / loop_in_band_power_));
}

void RealHfLoopChannelModel::process(std::span<const float> input,
                                     uint64_t,
                                     std::vector<float>& output) {
    output.resize(input.size());
    if (!loop_ || loop_->empty()) {
        std::copy(input.begin(), input.end(), output.begin());
        return;
    }

    const auto& loop = *loop_;
    for (size_t i = 0; i < input.size(); ++i) {
        output[i] = input[i] + scale_ * loop[position_];
        position_ = (position_ + 1) % loop.size();
    }
}

WattersonChannel::WattersonChannel(const Config& config, uint64_t seed)
    : config_(config),
      rng_(static_cast<uint32_t>(seed)) {
    delay_samples_ = static_cast<size_t>(
        config_.delay_spread_ms * static_cast<float>(config_.sample_rate) / 1000.0f);
    delay_line_.assign(delay_samples_, 0.0f);
    analytic_delay_line_.assign(delay_samples_, Complex(0.0f, 0.0f));
    initializeHilbert();
    noise_stddev_ = modemReferenceNoiseStddev(config_.snr_db);

    const float normalized_doppler =
        config_.doppler_spread_hz / static_cast<float>(config_.sample_rate);
    fading_sigma_hz_ = std::max(0.0f, config_.doppler_spread_hz * 0.5f);
    // Retained only for legacy diagnostics: the Watterson fading process below
    // is Gaussian-Doppler SoS, not a first-order AR process.
    fading_alpha_ = 1.0f - std::exp(-2.0f * kPi * normalized_doppler);
    if (fading_alpha_ <= 0.0f) {
        fading_alpha_ = 1.0f;
    }
    initializeGaussianDoppler(seed);

    actual_cfo_hz_ = config_.cfo_hz;
    if (config_.random_cfo_max_hz > 0.0f) {
        std::uniform_real_distribution<float> cfo_dist(
            -config_.random_cfo_max_hz, config_.random_cfo_max_hz);
        actual_cfo_hz_ = cfo_dist(rng_);
    }
    cfo_phase_inc_ = 2.0f * kPi * actual_cfo_hz_ /
                     static_cast<float>(config_.sample_rate);
}

void WattersonChannel::reset() {
    std::fill(delay_line_.begin(), delay_line_.end(), 0.0f);
    std::fill(analytic_delay_line_.begin(),
              analytic_delay_line_.end(),
              Complex(0.0f, 0.0f));
    std::fill(hilbert_delay_line_.begin(), hilbert_delay_line_.end(), 0.0f);
    hilbert_delay_idx_ = 0;
    resetFadingOscillators();
    cfo_phase_ = 0.0f;
}

void WattersonChannel::setSNR(float snr_db) {
    config_.snr_db = snr_db;
    noise_stddev_ = modemReferenceNoiseStddev(snr_db);
}

void WattersonChannel::initializeHilbert() {
    const size_t taps = kWattersonHilbertTaps;
    hilbert_inphase_coeffs_.assign(taps, 0.0f);
    hilbert_coeffs_.assign(taps, 0.0f);
    hilbert_delay_line_.assign(taps, 0.0f);
    hilbert_delay_idx_ = 0;
    hilbert_delay_samples_ = (taps - 1) / 2;

    const float cutoff_hz = std::min(
        kWattersonAnalyticLowpassHz,
        0.45f * static_cast<float>(config_.sample_rate));
    const float cutoff = cutoff_hz / static_cast<float>(config_.sample_rate);
    const float omega_c = 2.0f * kPi * cutoff;
    const int midpoint = static_cast<int>(hilbert_delay_samples_);
    for (int n = 0; n < static_cast<int>(taps); ++n) {
        const int k = n - midpoint;
        if (k == 0) {
            hilbert_inphase_coeffs_[static_cast<size_t>(n)] = 2.0f * cutoff;
        } else {
            const float kf = static_cast<float>(k);
            hilbert_inphase_coeffs_[static_cast<size_t>(n)] =
                std::sin(omega_c * kf) / (kPi * kf);
            hilbert_coeffs_[static_cast<size_t>(n)] =
                (1.0f - std::cos(omega_c * kf)) / (kPi * kf);
        }
        const float w = 2.0f * kPi * static_cast<float>(n) /
                        static_cast<float>(taps - 1);
        const float window =
            0.42f - 0.5f * std::cos(w) + 0.08f * std::cos(2.0f * w);
        hilbert_inphase_coeffs_[static_cast<size_t>(n)] *= window;
        hilbert_coeffs_[static_cast<size_t>(n)] *= window;
    }
}

void WattersonChannel::initializeGaussianDoppler(uint64_t seed) {
    fading1_oscillators_.clear();
    fading2_oscillators_.clear();
    fading_sample_index_ = 0;
    diagnostic_fading_frozen_ = false;

    if (!config_.fading_enabled || fading_sigma_hz_ <= 0.0f ||
        config_.sample_rate == 0) {
        fading1_ = Complex(1.0f, 0.0f);
        fading2_ = Complex(1.0f, 0.0f);
        return;
    }

    RngRoot root(seed);
    std::mt19937 tap1_rng(root.childSeed("watterson:gaussian_doppler:tap1"));
    std::mt19937 tap2_rng(root.childSeed("watterson:gaussian_doppler:tap2"));
    initializeGaussianDopplerTap(fading1_oscillators_, tap1_rng);
    initializeGaussianDopplerTap(fading2_oscillators_, tap2_rng);
    fading1_ = currentFadingTap(fading1_oscillators_);
    fading2_ = currentFadingTap(fading2_oscillators_);
}

void WattersonChannel::initializeGaussianDopplerTap(
    std::vector<FadingOscillator>& oscillators,
    std::mt19937& rng) {
    oscillators.clear();
    oscillators.reserve(kGaussianDopplerSosOscillators);

    const double sigma = static_cast<double>(fading_sigma_hz_);
    const double sample_rate = static_cast<double>(config_.sample_rate);

    std::vector<double> frequencies(kGaussianDopplerSosOscillators);
    std::uniform_real_distribution<double> cdf_jitter(0.05, 0.95);
    double mean = 0.0;
    for (size_t i = 0; i < frequencies.size(); ++i) {
        const double p =
            (static_cast<double>(i) + cdf_jitter(rng)) /
            static_cast<double>(frequencies.size());
        frequencies[i] = sigma * inverseNormalCdf(p);
        mean += frequencies[i];
    }
    mean /= static_cast<double>(frequencies.size());

    double variance = 0.0;
    for (double& frequency : frequencies) {
        frequency -= mean;
        variance += frequency * frequency;
    }
    variance /= static_cast<double>(frequencies.size());
    if (variance > std::numeric_limits<double>::min()) {
        const double scale = sigma / std::sqrt(variance);
        for (double& frequency : frequencies) {
            frequency *= scale;
        }
    }

    std::uniform_real_distribution<double> phase_dist(
        0.0, 2.0 * static_cast<double>(kPi));
    const double amplitude =
        1.0 / std::sqrt(static_cast<double>(kGaussianDopplerSosOscillators));
    for (double frequency_hz : frequencies) {
        const double phase = phase_dist(rng);
        const double phase_inc =
            2.0 * static_cast<double>(kPi) * frequency_hz / sample_rate;

        FadingOscillator oscillator;
        oscillator.initial_phasor = std::polar(1.0, phase);
        oscillator.phasor = oscillator.initial_phasor;
        oscillator.rotation = std::polar(1.0, phase_inc);
        oscillator.amplitude = amplitude;
        oscillators.push_back(oscillator);
    }
}

void WattersonChannel::resetFadingOscillators() {
    fading_sample_index_ = 0;
    diagnostic_fading_frozen_ = false;
    for (FadingOscillator& oscillator : fading1_oscillators_) {
        oscillator.phasor = oscillator.initial_phasor;
    }
    for (FadingOscillator& oscillator : fading2_oscillators_) {
        oscillator.phasor = oscillator.initial_phasor;
    }
    if (!fading1_oscillators_.empty()) {
        fading1_ = currentFadingTap(fading1_oscillators_);
    } else {
        fading1_ = Complex(1.0f, 0.0f);
    }
    if (!fading2_oscillators_.empty()) {
        fading2_ = currentFadingTap(fading2_oscillators_);
    } else {
        fading2_ = Complex(1.0f, 0.0f);
    }
}

std::complex<float> WattersonChannel::currentFadingTap(
    const std::vector<FadingOscillator>& oscillators) const {
    if (oscillators.empty()) {
        return Complex(1.0f, 0.0f);
    }

    std::complex<double> sum(0.0, 0.0);
    for (const FadingOscillator& oscillator : oscillators) {
        sum += oscillator.amplitude * oscillator.phasor;
    }
    return Complex(static_cast<float>(sum.real()),
                   static_cast<float>(sum.imag()));
}

std::complex<float> WattersonChannel::evaluateFadingTap(
    std::vector<FadingOscillator>& oscillators) {
    if (oscillators.empty()) {
        return Complex(1.0f, 0.0f);
    }

    std::complex<double> sum(0.0, 0.0);
    for (FadingOscillator& oscillator : oscillators) {
        sum += oscillator.amplitude * oscillator.phasor;
        oscillator.phasor *= oscillator.rotation;
    }
    return Complex(static_cast<float>(sum.real()),
                   static_cast<float>(sum.imag()));
}

void WattersonChannel::renormalizeFadingOscillators() {
    auto renormalize = [](std::vector<FadingOscillator>& oscillators) {
        for (FadingOscillator& oscillator : oscillators) {
            const double magnitude = std::abs(oscillator.phasor);
            if (magnitude > std::numeric_limits<double>::min()) {
                oscillator.phasor /= magnitude;
            }
        }
    };
    renormalize(fading1_oscillators_);
    renormalize(fading2_oscillators_);
}

Complex WattersonChannel::analyticSample(float sample) {
    hilbert_delay_line_[hilbert_delay_idx_] = sample;

    float in_phase = 0.0f;
    float quadrature = 0.0f;
    size_t j = hilbert_delay_idx_;
    for (size_t i = 0; i < hilbert_coeffs_.size(); ++i) {
        in_phase += hilbert_inphase_coeffs_[i] * hilbert_delay_line_[j];
        quadrature += hilbert_coeffs_[i] * hilbert_delay_line_[j];
        if (j == 0) {
            j = hilbert_coeffs_.size();
        }
        --j;
    }

    hilbert_delay_idx_ = (hilbert_delay_idx_ + 1) % hilbert_coeffs_.size();
    return Complex(in_phase, quadrature);
}

void WattersonChannel::process(std::span<const float> input,
                               std::vector<float>& output) {
    if (config_.fading_enabled) {
        processWithComplexFading(input, output);
    } else {
        processWithoutFading(input, output);
    }

    if (config_.cfo_enabled && std::abs(actual_cfo_hz_) > 0.001f) {
        applyCFO(output);
    }
}

void WattersonChannel::processWithoutFading(std::span<const float> input,
                                            std::vector<float>& output) {
    output.resize(input.size());

    for (size_t i = 0; i < input.size(); ++i) {
        const float sample = input[i];

        float out = 0.0f;
        if (config_.multipath_enabled && delay_samples_ > 0) {
            out += sample * config_.path1_gain;

            const float delayed = delay_line_.front();
            delay_line_.pop_front();
            delay_line_.push_back(sample);
            out += delayed * config_.path2_gain;
        } else {
            out = sample;
        }

        if (config_.noise_enabled) {
            out += noise_stddev_ * gaussian_(rng_);
        }
        output[i] = out;
    }
}

void WattersonChannel::processWithComplexFading(std::span<const float> input,
                                                std::vector<float>& output) {
    output.resize(input.size());

    for (size_t i = 0; i < input.size(); ++i) {
        const Complex analytic = analyticSample(input[i]);
        updateFading();

        Complex out(0.0f, 0.0f);
        if (config_.multipath_enabled && delay_samples_ > 0) {
            out += analytic * (config_.path1_gain * fading1_);

            const Complex delayed = analytic_delay_line_.front();
            analytic_delay_line_.pop_front();
            analytic_delay_line_.push_back(analytic);
            out += delayed * (config_.path2_gain * fading2_);
        } else {
            out = analytic * fading1_;
        }

        float real_out = std::real(out);
        if (config_.noise_enabled) {
            real_out += noise_stddev_ * gaussian_(rng_);
        }
        output[i] = real_out;
    }
}

std::vector<float> WattersonChannel::process(std::span<const float> input) {
    std::vector<float> output;
    process(input, output);
    return output;
}

std::complex<float> WattersonChannel::trueFrequencyResponse(float freq_hz) const {
    // Mirrors processWithComplexFading exactly:
    //   out(n) = x_a(n)*g1*a1(n) + x_a(n-D)*g2*a2(n)
    // A pure delay of D samples is exp(-j*2*pi*f*D/fs) in frequency.
    const Complex tap1 = config_.fading_enabled ? fading1_ : Complex(1.0f, 0.0f);
    if (!config_.multipath_enabled || delay_samples_ == 0) {
        // The no-multipath branch applies fading1_ alone (and, with fading off,
        // passes the sample through untouched).
        return config_.fading_enabled ? tap1 : Complex(1.0f, 0.0f);
    }
    const Complex tap2 = config_.fading_enabled ? fading2_ : Complex(1.0f, 0.0f);
    const float phase = -2.0f * static_cast<float>(M_PI) * freq_hz *
                        static_cast<float>(delay_samples_) /
                        static_cast<float>(config_.sample_rate);
    const Complex delay_term(std::cos(phase), std::sin(phase));
    return config_.path1_gain * tap1 + config_.path2_gain * tap2 * delay_term;
}

float WattersonChannel::fadingMagnitude() const {
    return std::abs(fading1_);
}

void WattersonChannel::stepFadingForDiagnostics() {
    updateFading();
}

void WattersonChannel::setFadingTapsForDiagnostics(Complex tap1, Complex tap2) {
    fading1_ = tap1;
    fading2_ = tap2;
    diagnostic_fading_frozen_ = true;
}

std::vector<Complex> WattersonChannel::applyComplexMultipathForDiagnostics(
    std::span<const Complex> input,
    size_t delay_samples,
    Complex tap1,
    Complex tap2,
    float path1_gain,
    float path2_gain) {
    std::vector<Complex> output(input.size(), Complex(0.0f, 0.0f));
    std::deque<Complex> delay(delay_samples, Complex(0.0f, 0.0f));

    for (size_t i = 0; i < input.size(); ++i) {
        output[i] += input[i] * (path1_gain * tap1);
        if (delay_samples > 0) {
            const Complex delayed = delay.front();
            delay.pop_front();
            delay.push_back(input[i]);
            output[i] += delayed * (path2_gain * tap2);
        } else {
            output[i] += input[i] * (path2_gain * tap2);
        }
    }
    return output;
}

void WattersonChannel::updateFading() {
    if (diagnostic_fading_frozen_) {
        return;
    }
    fading1_ = evaluateFadingTap(fading1_oscillators_);
    fading2_ = evaluateFadingTap(fading2_oscillators_);
    ++fading_sample_index_;
    if ((fading_sample_index_ % kGaussianDopplerRenormalizeInterval) == 0) {
        renormalizeFadingOscillators();
    }
}

void WattersonChannel::applyCFO(std::vector<float>& samples) {
    analyticFrequencyShift(samples, actual_cfo_hz_, config_.sample_rate, cfo_phase_);
}

WattersonChannelModel::WattersonChannelModel(const WattersonChannel::Config& config,
                                             uint64_t seed)
    : channel_(config, seed) {}

void WattersonChannelModel::reset() {
    channel_.reset();
}

void WattersonChannelModel::setSNR(float snr_db) {
    channel_.setSNR(snr_db);
}

void WattersonChannelModel::process(std::span<const float> input,
                                    uint64_t,
                                    std::vector<float>& output) {
    channel_.process(input, output);
}

const WattersonChannel::Config& WattersonChannelModel::config() const {
    return channel_.config();
}

namespace itu_r_f1487 {

WattersonChannel::Config good(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 0.5f,
            .doppler_spread_hz = 0.1f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config moderate(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 1.0f,
            .doppler_spread_hz = 0.5f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config poor(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 2.0f,
            .doppler_spread_hz = 1.0f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config flutter(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 0.5f,
            .doppler_spread_hz = 10.0f,
            .path1_gain = 0.707f,
            .path2_gain = 0.707f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = true,
            .multipath_enabled = true,
            .noise_enabled = true};
}

WattersonChannel::Config awgn(float snr_db) {
    return {.snr_db = snr_db,
            .delay_spread_ms = 0.0f,
            .doppler_spread_hz = 0.0f,
            .path1_gain = 1.0f,
            .path2_gain = 0.0f,
            .sample_rate = kDefaultSampleRate,
            .fading_enabled = false,
            .multipath_enabled = false,
            .noise_enabled = true};
}

}  // namespace itu_r_f1487

WattersonChannel::Config configForWatterson(ChannelType type,
                                            float snr_db,
                                            uint32_t sample_rate) {
    WattersonChannel::Config cfg;
    switch (type) {
        case ChannelType::GOOD:
            cfg = itu_r_f1487::good(snr_db);
            break;
        case ChannelType::MODERATE:
            cfg = itu_r_f1487::moderate(snr_db);
            break;
        case ChannelType::POOR:
            cfg = itu_r_f1487::poor(snr_db);
            break;
        case ChannelType::FLUTTER:
            cfg = itu_r_f1487::flutter(snr_db);
            break;
        case ChannelType::AWGN:
            cfg = itu_r_f1487::awgn(snr_db);
            break;
        case ChannelType::PASSTHROUGH:
            cfg = itu_r_f1487::awgn(snr_db);
            cfg.noise_enabled = false;
            break;
        default:
            throw std::invalid_argument("unknown channel type");
    }
    cfg.sample_rate = sample_rate;
    return cfg;
}

namespace {
constexpr size_t kFrequencyShiftTaps = 4097;
}  // namespace

size_t FrequencyShiftChannelModel::groupDelaySamples() {
    return (kFrequencyShiftTaps - 1) / 2;
}

FrequencyShiftChannelModel::FrequencyShiftChannelModel(std::unique_ptr<IChannelModel> inner,
                                                       float cfo_hz, uint32_t sample_rate)
    : inner_(std::move(inner)), cfo_hz_(cfo_hz) {
    const size_t taps = kFrequencyShiftTaps;
    std::vector<double> hi(taps, 0.0), hq(taps, 0.0);
    const double fs = static_cast<double>(sample_rate);
    const double cutoff = std::min(static_cast<double>(kWattersonAnalyticLowpassHz), 0.45 * fs) / fs;
    const double omega_c = 2.0 * M_PI * cutoff;
    const int mid = static_cast<int>(groupDelaySamples());
    for (int n = 0; n < static_cast<int>(taps); ++n) {
        const int k = n - mid;
        if (k == 0) {
            hi[static_cast<size_t>(n)] = 2.0 * cutoff;
        } else {
            hi[static_cast<size_t>(n)] = std::sin(omega_c * k) / (M_PI * k);
            hq[static_cast<size_t>(n)] = (1.0 - std::cos(omega_c * k)) / (M_PI * k);
        }
        const double w = 2.0 * M_PI * n / static_cast<double>(taps - 1);
        const double window = 0.42 - 0.5 * std::cos(w) + 0.08 * std::cos(2.0 * w);
        hi[static_cast<size_t>(n)] *= window;
        hq[static_cast<size_t>(n)] *= window;
    }
    // y[n] = sum_i h[i] x[n-i] = sum_m g[m] line[p+1+m] with g[m] = h[taps-1-m].
    g_inphase_.resize(taps);
    g_quadrature_.resize(taps);
    for (size_t m = 0; m < taps; ++m) {
        g_inphase_[m] = static_cast<float>(hi[taps - 1 - m]);
        g_quadrature_[m] = static_cast<float>(hq[taps - 1 - m]);
    }
    line_.assign(2 * taps, 0.0f);
    phase_inc_ = 2.0 * M_PI * static_cast<double>(cfo_hz) / fs;
}

void FrequencyShiftChannelModel::reset() {
    if (inner_) inner_->reset();
    std::fill(line_.begin(), line_.end(), 0.0f);
    idx_ = 0;
    phase_ = 0.0;
}

void FrequencyShiftChannelModel::process(std::span<const float> input,
                                         uint64_t start_sample,
                                         std::vector<float>& output) {
    inner_->process(input, start_sample, inner_out_);
    output.resize(inner_out_.size());
    const size_t taps = g_inphase_.size();
    const float* gi = g_inphase_.data();
    const float* gq = g_quadrature_.data();
    for (size_t s = 0; s < inner_out_.size(); ++s) {
        line_[idx_] = inner_out_[s];
        line_[idx_ + taps] = inner_out_[s];
        const float* x = line_.data() + idx_ + 1;  // oldest .. newest, contiguous
        float ai[8] = {0, 0, 0, 0, 0, 0, 0, 0}, aq[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        size_t m = 0;
        for (; m + 8 <= taps; m += 8) {
            for (int u = 0; u < 8; ++u) {
                ai[u] += gi[m + u] * x[m + u];
                aq[u] += gq[m + u] * x[m + u];
            }
        }
        float in_phase = 0.0f, quadrature = 0.0f;
        for (; m < taps; ++m) {
            in_phase += gi[m] * x[m];
            quadrature += gq[m] * x[m];
        }
        for (int u = 0; u < 8; ++u) {
            in_phase += ai[u];
            quadrature += aq[u];
        }
        idx_ = (idx_ + 1) % taps;
        output[s] = static_cast<float>(in_phase * std::cos(phase_) - quadrature * std::sin(phase_));
        phase_ += phase_inc_;
        if (phase_ > M_PI) phase_ -= 2.0 * M_PI;
        else if (phase_ < -M_PI) phase_ += 2.0 * M_PI;
    }
}

namespace {
double besselI0(double x) {
    double sum = 1.0, term = 1.0;
    const double q = 0.25 * x * x;
    for (int k = 1; k < 64; ++k) {
        term *= q / (static_cast<double>(k) * static_cast<double>(k));
        sum += term;
        if (term < 1e-17 * sum) break;
    }
    return sum;
}
}  // namespace

ClockOffsetChannelModel::ClockOffsetChannelModel(std::unique_ptr<IChannelModel> inner, float ppm,
                                                 uint32_t sample_rate)
    : inner_(std::move(inner)), ppm_(ppm) {
    step_ = 1.0 + static_cast<double>(ppm) * 1e-6;
    const double ahead = std::max(0.0, static_cast<double>(ppm) * 1e-6) *
                         static_cast<double>(sample_rate) * kClockOffsetMaxSessionS;
    guard_ = static_cast<size_t>(kHalfTaps) + 2u + static_cast<size_t>(std::ceil(ahead));
    // Row j holds h(k - j/kPhases) for k = -kHalfTaps+1 .. kHalfTaps (tap index k + kHalfTaps - 1).
    constexpr int taps = 2 * kHalfTaps;
    constexpr double beta = 8.0;
    const double i0_beta = besselI0(beta);
    table_.assign(static_cast<size_t>(kPhases + 1) * taps, 0.0f);
    for (int j = 0; j <= kPhases; ++j) {
        const double frac = static_cast<double>(j) / kPhases;
        double row[taps];
        double sum = 0.0;
        for (int t = 0; t < taps; ++t) {
            const double x = static_cast<double>(t - kHalfTaps + 1) - frac;
            const double sinc = (std::abs(x) < 1e-12) ? 1.0 : std::sin(M_PI * x) / (M_PI * x);
            const double r = x / static_cast<double>(kHalfTaps);
            const double w = (std::abs(r) >= 1.0) ? 0.0
                                                   : besselI0(beta * std::sqrt(1.0 - r * r)) / i0_beta;
            row[t] = sinc * w;
            sum += row[t];
        }
        for (int t = 0; t < taps; ++t) {
            table_[static_cast<size_t>(j) * taps + static_cast<size_t>(t)] =
                static_cast<float>(row[t] / sum);
        }
    }
    reset();
}

void ClockOffsetChannelModel::reset() {
    if (inner_) inner_->reset();
    buf_.clear();
    buf_base_ = 0;
    written_ = 0;
    origin_ = -static_cast<double>(guard_);
    out_count_ = 0;
    slips_ = 0;
}

void ClockOffsetChannelModel::process(std::span<const float> input,
                                      uint64_t start_sample,
                                      std::vector<float>& output) {
    inner_->process(input, start_sample, inner_out_);
    buf_.insert(buf_.end(), inner_out_.begin(), inner_out_.end());
    written_ += static_cast<int64_t>(inner_out_.size());
    output.resize(inner_out_.size());
    constexpr int taps = 2 * kHalfTaps;
    for (size_t n = 0; n < output.size(); ++n) {
        double pos = origin_ + static_cast<double>(out_count_) * step_;
        int64_t i = static_cast<int64_t>(std::floor(pos));
        if (i + kHalfTaps >= written_) {
            // The read position caught up with the stream (positive drift past the guard):
            // re-establish the guard, as a real capture buffer would after an xrun.
            origin_ -= static_cast<double>(guard_);
            ++slips_;
            pos = origin_ + static_cast<double>(out_count_) * step_;
            i = static_cast<int64_t>(std::floor(pos));
        }
        const double phase = (pos - static_cast<double>(i)) * kPhases;
        int j = static_cast<int>(phase);
        if (j >= kPhases) j = kPhases - 1;
        const float a = static_cast<float>(phase - j);
        const float* r0 = &table_[static_cast<size_t>(j) * taps];
        const float* r1 = r0 + taps;
        float acc = 0.0f;
        const int64_t first = i - kHalfTaps + 1;
        for (int t = 0; t < taps; ++t) {
            const int64_t idx = first + t;
            if (idx < buf_base_ || idx >= written_) continue;  // before the stream began: silence
            const float h = r0[t] + a * (r1[t] - r0[t]);
            acc += h * buf_[static_cast<size_t>(idx - buf_base_)];
        }
        output[n] = acc;
        ++out_count_;
    }
    // Keep kHalfTaps of history behind the next read position.
    const double next = origin_ + static_cast<double>(out_count_) * step_;
    const int64_t keep_from = static_cast<int64_t>(std::floor(next)) - kHalfTaps;
    if (keep_from > buf_base_) {
        const int64_t drop = std::min<int64_t>(keep_from - buf_base_,
                                               static_cast<int64_t>(buf_.size()));
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(drop));
        buf_base_ += drop;
    }
}

std::optional<TxPaConfig> parseTxPaSpec(std::string_view spec) {
    TxPaConfig c;
    c.enabled = true;
    if (spec.empty() || spec == "on" || spec == "1") return c;
    size_t pos = 0;
    while (pos < spec.size()) {
        const size_t comma = spec.find(',', pos);
        const std::string_view item =
            spec.substr(pos, comma == std::string_view::npos ? std::string_view::npos : comma - pos);
        const size_t eq = item.find('=');
        if (eq == std::string_view::npos || eq == 0) return std::nullopt;
        const std::string key(item.substr(0, eq));
        const std::string val(item.substr(eq + 1));
        char* end = nullptr;
        const float v = std::strtof(val.c_str(), &end);
        if (end == val.c_str() || *end != '\0' || !std::isfinite(v)) return std::nullopt;
        if (key == "pep" && v > 0.0f && v <= 1.0f) c.pep = v;
        else if (key == "drive_db" && v >= -40.0f && v <= 40.0f) c.drive_db = v;
        else if (key == "p" && v >= 0.5f && v <= 20.0f) c.rapp_p = v;
        else if (key == "sat_db" && v >= 0.0f && v <= 20.0f) c.sat_db = v;
        else if (key == "attack_ms" && v > 0.0f && v <= 1000.0f) c.attack_ms = v;
        else if (key == "release_ms" && v > 0.0f && v <= 10000.0f) c.release_ms = v;
        else return std::nullopt;
        if (comma == std::string_view::npos) break;
        pos = comma + 1;
    }
    return c;
}

TxPaAlcChannelModel::TxPaAlcChannelModel(std::unique_ptr<IChannelModel> inner,
                                         const TxPaConfig& config, uint32_t sample_rate)
    : inner_(std::move(inner)), config_(config) {
    const double fs = static_cast<double>(sample_rate);
    drive_ = std::pow(10.0, static_cast<double>(config_.drive_db) / 20.0);
    a_att_ = 1.0 - std::exp(-1.0 / (static_cast<double>(config_.attack_ms) * 1e-3 * fs));
    a_rel_ = 1.0 - std::exp(-1.0 / (static_cast<double>(config_.release_ms) * 1e-3 * fs));
    sat_ = static_cast<double>(config_.pep) * std::pow(10.0, static_cast<double>(config_.sat_db) / 20.0);
    // Band-limited Hilbert transformer (envelope estimate only), Blackman window.
    const double cutoff = std::min(static_cast<double>(kWattersonAnalyticLowpassHz), 0.45 * fs) / fs;
    const double omega_c = 2.0 * M_PI * cutoff;
    std::vector<double> h(kHilbertTaps, 0.0);
    for (int n = 0; n < kHilbertTaps; ++n) {
        const int k = n - kDelaySamples;
        if (k != 0) h[static_cast<size_t>(n)] = (1.0 - std::cos(omega_c * k)) / (M_PI * k);
        const double w = 2.0 * M_PI * n / static_cast<double>(kHilbertTaps - 1);
        h[static_cast<size_t>(n)] *= 0.42 - 0.5 * std::cos(w) + 0.08 * std::cos(2.0 * w);
    }
    hq_.resize(kHilbertTaps);
    for (int m = 0; m < kHilbertTaps; ++m) {
        hq_[static_cast<size_t>(m)] = static_cast<float>(h[static_cast<size_t>(kHilbertTaps - 1 - m)]);
    }
    reset();
}

void TxPaAlcChannelModel::reset() {
    if (inner_) inner_->reset();
    line_.assign(2 * static_cast<size_t>(kHilbertTaps), 0.0f);
    idx_ = 0;
    gain_ = 1.0;
}

void TxPaAlcChannelModel::process(std::span<const float> input, uint64_t start_sample,
                                  std::vector<float>& output) {
    const size_t taps = static_cast<size_t>(kHilbertTaps);
    pa_out_.resize(input.size());
    const double pep = static_cast<double>(config_.pep);
    const double two_p = 2.0 * static_cast<double>(config_.rapp_p);
    for (size_t s = 0; s < input.size(); ++s) {
        // Doubled delay line: the newest `taps` samples are contiguous at line_[idx_+1 ..].
        line_[idx_] = input[s];
        line_[idx_ + taps] = input[s];
        const float* win = &line_[idx_ + 1];
        double q = 0.0;
        for (size_t m = 0; m < taps; ++m) q += static_cast<double>(hq_[m]) * win[m];
        const double i = static_cast<double>(win[taps - 1 - static_cast<size_t>(kDelaySamples)]);
        idx_ = (idx_ + 1) % taps;
        const double env = drive_ * std::sqrt(i * i + q * q);
        const double want = (env * gain_ > pep && env > 0.0) ? pep / env : 1.0;
        gain_ += (want < gain_ ? a_att_ : a_rel_) * (want - gain_);
        const double u = gain_ * env;
        const double rapp = (u > 0.0) ? 1.0 / std::pow(1.0 + std::pow(u / sat_, two_p), 1.0 / two_p) : 1.0;
        pa_out_[s] = static_cast<float>(drive_ * gain_ * rapp * i);
    }
    inner_->process(std::span<const float>(pa_out_.data(), pa_out_.size()), start_sample, output);
}

namespace {
std::unique_ptr<IChannelModel> createChannelModelUnshifted(const ChannelConfig& config,
                                                           const RngRoot& rng_root,
                                                           std::string_view stream_name);
}  // namespace

std::unique_ptr<IChannelModel> createChannelModel(const ChannelConfig& config,
                                                  const RngRoot& rng_root,
                                                  std::string_view stream_name) {
    auto model = createChannelModelUnshifted(config, rng_root, stream_name);
    if (config.tx_pa.enabled) {
        // The sender's ALC/PA acts on its audio BEFORE fading and noise.
        model = std::make_unique<TxPaAlcChannelModel>(std::move(model), config.tx_pa,
                                                      config.sample_rate);
    }
    if (std::abs(config.cfo_hz) > 0.001f) {
        model = std::make_unique<FrequencyShiftChannelModel>(std::move(model), config.cfo_hz,
                                                             config.sample_rate);
    }
    if (std::abs(config.clock_ppm) > 0.001f) {
        model = std::make_unique<ClockOffsetChannelModel>(std::move(model), config.clock_ppm,
                                                          config.sample_rate);
    }
    return model;
}

namespace {
std::unique_ptr<IChannelModel> createChannelModelUnshifted(const ChannelConfig& config,
                                                           const RngRoot& rng_root,
                                                           std::string_view stream_name) {
    switch (config.type) {
        case ChannelType::PASSTHROUGH:
            return std::make_unique<PassthroughChannelModel>();
        case ChannelType::AWGN:
            return std::make_unique<AWGNChannelModel>(
                config.snr_db, rng_root.childSeed(stream_name));
        case ChannelType::REAL_HF_LOOP:
            if (!config.real_hf_loop_noise || config.real_hf_loop_noise->empty()) {
                throw std::invalid_argument(
                    "real_hf_loop requires a normalized noise-bed loop");
            }
            if (config.sample_rate != kDefaultSampleRate) {
                throw std::invalid_argument("real_hf_loop requires 48000 Hz sample rate");
            }
            return std::make_unique<RealHfLoopChannelModel>(
                config.snr_db,
                config.real_hf_loop_noise,
                rng_root.childSeed(stream_name));
        case ChannelType::GOOD:
        case ChannelType::MODERATE:
        case ChannelType::POOR:
        case ChannelType::FLUTTER: {
            auto cfg = configForWatterson(config.type, config.snr_db, config.sample_rate);
            cfg.cfo_hz = 0.0f;
            return std::make_unique<WattersonChannelModel>(
                cfg, rng_root.childSeed(stream_name));
        }
        default:
            throw std::invalid_argument("unknown channel type");
    }
}
}  // namespace

}  // namespace ultra::ota_channel_core
