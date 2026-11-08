#include "ota_channel_core/models.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <iostream>
#include <numeric>
#include <vector>

using ultra::ota_channel_core::AWGNChannelModel;
using ultra::ota_channel_core::ChannelConfig;
using ultra::ota_channel_core::ChannelType;
using ultra::ota_channel_core::PassthroughChannelModel;
using ultra::ota_channel_core::RngRoot;
using ultra::ota_channel_core::WattersonChannel;
using ultra::ota_channel_core::createChannelModel;

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kSampleRate = ultra::ota_channel_core::kDefaultSampleRate;

void assertNear(const std::vector<float>& a,
                const std::vector<float>& b,
                float eps = 1.0e-6f) {
    assert(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        assert(std::abs(a[i] - b[i]) <= eps);
    }
}

void assertNear(float actual, float expected, float eps) {
    const float tolerance = eps * std::max(1.0f, std::abs(expected));
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "assertNear failed actual=" << actual
                  << " expected=" << expected
                  << " tolerance=" << tolerance << "\n";
    }
    assert(std::abs(actual - expected) <= tolerance);
}

double meanOf(const std::vector<float>& samples) {
    return std::accumulate(samples.begin(), samples.end(), 0.0) /
           static_cast<double>(samples.size());
}

double varianceOf(const std::vector<float>& samples, double mean) {
    double sum = 0.0;
    for (float sample : samples) {
        const double centered = static_cast<double>(sample) - mean;
        sum += centered * centered;
    }
    return sum / static_cast<double>(samples.size());
}

std::vector<float> modemBandpassFirCoefficients() {
    constexpr size_t taps = 101;
    constexpr float low_hz = 50.0f;
    constexpr float high_hz = 2950.0f;
    std::vector<float> coeffs(taps);
    const float fc_low = low_hz / static_cast<float>(kSampleRate);
    const float fc_high = high_hz / static_cast<float>(kSampleRate);
    const int midpoint = static_cast<int>((taps - 1) / 2);

    for (int n = 0; n < static_cast<int>(taps); ++n) {
        if (n == midpoint) {
            coeffs[static_cast<size_t>(n)] = 2.0f * (fc_high - fc_low);
        } else {
            const float x = kPi * static_cast<float>(n - midpoint);
            coeffs[static_cast<size_t>(n)] =
                (std::sin(2.0f * fc_high * x) -
                 std::sin(2.0f * fc_low * x)) / x;
        }
        const float w = 2.0f * kPi * static_cast<float>(n) /
                        static_cast<float>(taps - 1);
        coeffs[static_cast<size_t>(n)] *=
            0.42f - 0.5f * std::cos(w) + 0.08f * std::cos(2.0f * w);
    }
    return coeffs;
}

double inBandPowerOf(const std::vector<float>& samples) {
    const std::vector<float> coeffs = modemBandpassFirCoefficients();
    std::vector<float> delay_line(coeffs.size(), 0.0f);
    size_t delay_idx = 0;
    double sum_sq = 0.0;
    size_t measured = 0;
    for (size_t n = 0; n < samples.size(); ++n) {
        delay_line[delay_idx] = samples[n];
        float out = 0.0f;
        size_t j = delay_idx;
        for (float coeff : coeffs) {
            out += coeff * delay_line[j];
            if (j == 0) {
                j = coeffs.size();
            }
            --j;
        }
        delay_idx = (delay_idx + 1) % coeffs.size();
        if (n >= coeffs.size()) {
            sum_sq += static_cast<double>(out) * static_cast<double>(out);
            ++measured;
        }
    }
    return measured == 0 ? 0.0 : sum_sq / static_cast<double>(measured);
}

double averageDftBinPower(const std::vector<float>& samples,
                          const std::vector<int>& bins,
                          size_t block_size,
                          size_t blocks) {
    double sum_power = 0.0;
    size_t count = 0;
    for (size_t block = 0; block < blocks; ++block) {
        const size_t offset = block * block_size;
        for (int bin : bins) {
            std::complex<double> acc(0.0, 0.0);
            for (size_t n = 0; n < block_size; ++n) {
                const double phase = -2.0 * static_cast<double>(kPi) *
                                     static_cast<double>(bin) *
                                     static_cast<double>(n) /
                                     static_cast<double>(block_size);
                acc += static_cast<double>(samples[offset + n]) *
                       std::complex<double>(std::cos(phase), std::sin(phase));
            }
            sum_power += std::norm(acc) / static_cast<double>(block_size);
            ++count;
        }
    }
    return sum_power / static_cast<double>(count);
}

double rmsOf(const std::vector<float>& samples, size_t begin = 0, size_t end = 0) {
    if (end == 0 || end > samples.size()) {
        end = samples.size();
    }
    double sum_sq = 0.0;
    for (size_t i = begin; i < end; ++i) {
        sum_sq += static_cast<double>(samples[i]) * static_cast<double>(samples[i]);
    }
    return std::sqrt(sum_sq / static_cast<double>(end - begin));
}

double estimateToneFrequencyHz(const std::vector<float>& samples,
                               size_t begin,
                               size_t end) {
    int crossings = 0;
    for (size_t i = begin + 1; i < end; ++i) {
        if (samples[i - 1] < 0.0f && samples[i] >= 0.0f) {
            ++crossings;
        }
    }
    const double seconds = static_cast<double>(end - begin) /
                           static_cast<double>(kSampleRate);
    return static_cast<double>(crossings) / seconds;
}

std::vector<float> runModel(ChannelType type, uint64_t seed) {
    const std::vector<float> input{0.0f, 0.1f, -0.2f, 0.3f, -0.4f, 0.0f, 0.25f, -0.15f};
    RngRoot root(seed);
    auto model = createChannelModel(
        ChannelConfig{.type = type, .snr_db = 18.0f, .seed = seed},
        root,
        "model:test");
    return model->process(input);
}

void checkRepeat(ChannelType type) {
    const auto first = runModel(type, 0xfeedbeefu);
    const auto second = runModel(type, 0xfeedbeefu);
    assertNear(first, second, 0.0f);
}

void checkDelaySpread(ChannelType type, size_t expected_delay) {
    auto cfg = ultra::ota_channel_core::configForWatterson(type, 80.0f);
    cfg.noise_enabled = false;
    cfg.fading_enabled = false;
    WattersonChannel channel(cfg, 123);

    std::vector<float> impulse(expected_delay + 8, 0.0f);
    impulse[0] = 1.0f;
    const auto out = channel.process(impulse);

    assert(std::abs(out[0] - cfg.path1_gain) <= 1.0e-6f);
    assert(std::abs(out[expected_delay] - cfg.path2_gain) <= 1.0e-6f);
}

void checkAwgnStatisticsAndSpectrum() {
    constexpr float snr_db = 18.0f;
    constexpr size_t sample_count = 65536;
    RngRoot root(0x123456u);
    AWGNChannelModel model(snr_db, root.childSeed("awgn:statistics"));
    const auto noise = model.process(std::vector<float>(sample_count, 0.0f));

    const float sigma = ultra::ota_channel_core::modemReferenceNoiseStddev(snr_db);
    const double mean = meanOf(noise);
    const double variance = varianceOf(noise, mean);
    assert(std::abs(mean) < static_cast<double>(sigma) * 0.02);
    assert(std::abs(variance - static_cast<double>(sigma) * sigma) /
           (static_cast<double>(sigma) * sigma) < 0.015);

    constexpr size_t block_size = 1024;
    constexpr size_t blocks = 64;
    const double low = averageDftBinPower(noise, {4, 8, 12, 16}, block_size, blocks);
    const double mid = averageDftBinPower(noise, {64, 96, 128, 160}, block_size, blocks);
    const double high = averageDftBinPower(noise, {256, 320, 384, 448}, block_size, blocks);
    const double max_band = std::max({low, mid, high});
    const double min_band = std::min({low, mid, high});
    const double flatness_db = 10.0 * std::log10(max_band / min_band);
    std::cout << "awgn_statistics snr_db=" << snr_db
              << " mean=" << mean
              << " variance=" << variance
              << " expected_variance=" << static_cast<double>(sigma) * sigma
              << " spectrum_flatness_db=" << flatness_db
              << "\n";
    assert(flatness_db < 1.0);
}

void checkWattersonNoiseAndFading() {
    constexpr float snr_db = 18.0f;
    constexpr size_t sample_count = 65536;
    auto noise_cfg = ultra::ota_channel_core::itu_r_f1487::awgn(snr_db);
    noise_cfg.cfo_enabled = false;
    WattersonChannel noise_channel(noise_cfg, 0xabcdefu);
    const auto noise = noise_channel.process(std::vector<float>(sample_count, 0.0f));
    const float sigma = ultra::ota_channel_core::modemReferenceNoiseStddev(snr_db);
    const double mean = meanOf(noise);
    const double variance = varianceOf(noise, mean);
    std::cout << "watterson_noise snr_db=" << snr_db
              << " mean=" << mean
              << " variance=" << variance
              << " expected_variance=" << static_cast<double>(sigma) * sigma
              << "\n";
    assert(std::abs(mean) < static_cast<double>(sigma) * 0.02);
    assert(std::abs(variance - static_cast<double>(sigma) * sigma) /
           (static_cast<double>(sigma) * sigma) < 0.015);

    auto flutter_cfg = ultra::ota_channel_core::itu_r_f1487::flutter(80.0f);
    flutter_cfg.noise_enabled = false;
    flutter_cfg.multipath_enabled = false;
    flutter_cfg.cfo_enabled = false;
    WattersonChannel flutter(flutter_cfg, 0x13579u);
    std::vector<float> tone(sample_count);
    for (size_t i = 0; i < tone.size(); ++i) {
        tone[i] = static_cast<float>(
            std::sin(2.0 * static_cast<double>(kPi) * 1000.0 *
                     static_cast<double>(i) /
                     static_cast<double>(kSampleRate)));
    }
    const auto faded = flutter.process(tone);
    const double output_power =
        std::inner_product(faded.begin() + 4096, faded.end(),
                           faded.begin() + 4096, 0.0) /
        static_cast<double>(faded.size() - 4096);
    const double input_power =
        std::inner_product(tone.begin() + 4096, tone.end(),
                           tone.begin() + 4096, 0.0) /
        static_cast<double>(tone.size() - 4096);
    const double mean_square_gain = output_power / input_power;
    std::cout << "watterson_flutter_tone mean_square_gain=" << mean_square_gain
              << " doppler_hz=" << flutter_cfg.doppler_spread_hz
              << "\n";
    assert(mean_square_gain > 0.80 && mean_square_gain < 1.20);
}

void checkRealHfLoopScaling() {
    constexpr float snr_db = 12.0f;
    constexpr size_t sample_count = 96000;
    std::vector<float> loop(sample_count);
    const double amplitude = std::sqrt(2.0);
    for (size_t i = 0; i < loop.size(); ++i) {
        loop[i] = static_cast<float>(
            amplitude * std::sin(2.0 * static_cast<double>(kPi) * 1000.0 *
                                 static_cast<double>(i) /
                                 static_cast<double>(kSampleRate)));
    }

    ultra::ota_channel_core::RealHfLoopChannelModel model(snr_db, loop, 0);
    const auto output = model.process(std::vector<float>(sample_count, 0.0f));
    const double measured_power = inBandPowerOf(output);
    const double expected_power =
        ultra::ota_channel_core::kModemReferencePower *
        std::pow(10.0, -static_cast<double>(snr_db) / 10.0);
    const double error_db = 10.0 * std::log10(measured_power / expected_power);
    std::cout << "real_hf_loop_scaling snr_db=" << snr_db
              << " measured_in_band_power=" << measured_power
              << " expected_in_band_power=" << expected_power
              << " error_db=" << error_db
              << "\n";
    assert(std::abs(error_db) < 0.10);
}

void checkCfoInjectionPreservesBandEdgeTone() {
    constexpr size_t sample_count = 65536;
    constexpr float cfo_hz = 37.0f;
    const double tone_hz = 3800.0 * static_cast<double>(kSampleRate) /
                           static_cast<double>(sample_count);
    std::vector<float> tone(sample_count);
    for (size_t i = 0; i < tone.size(); ++i) {
        tone[i] = static_cast<float>(
            std::sin(2.0 * static_cast<double>(kPi) * tone_hz *
                     static_cast<double>(i) /
                     static_cast<double>(kSampleRate)));
    }

    ultra::ota_channel_core::WattersonChannel::Config cfg;
    cfg.snr_db = 80.0f;
    cfg.delay_spread_ms = 0.0f;
    cfg.doppler_spread_hz = 0.0f;
    cfg.cfo_hz = cfo_hz;
    cfg.path1_gain = 1.0f;
    cfg.path2_gain = 0.0f;
    cfg.sample_rate = kSampleRate;
    cfg.fading_enabled = false;
    cfg.multipath_enabled = false;
    cfg.noise_enabled = false;
    cfg.cfo_enabled = true;

    WattersonChannel channel(cfg, 0x2468u);
    const auto shifted = channel.process(tone);

    constexpr size_t edge = 4096;
    const double rms_ratio =
        rmsOf(shifted, edge, shifted.size() - edge) /
        rmsOf(tone, edge, tone.size() - edge);
    const double estimated_hz =
        estimateToneFrequencyHz(shifted, edge, shifted.size() - edge);
    std::cout << "cfo_injection tone_hz=" << tone_hz
              << " cfo_hz=" << cfo_hz
              << " estimated_hz=" << estimated_hz
              << " rms_ratio=" << rms_ratio
              << "\n";
    assertNear(static_cast<float>(rms_ratio), 1.0f, 0.03f);
    assertNear(static_cast<float>(estimated_hz),
               static_cast<float>(tone_hz + cfo_hz),
               0.003f);
}

}  // namespace

// FrequencyShiftChannelModel (2026-11-08): the receiver-side dial offset must be CHUNK-INVARIANT
// (OTASim feeds ~tick-sized chunks: chunked output must equal a single-call output exactly —
// that is what "no block-edge artifacts" means), and must move every in-band frequency by
// exactly +cfo with the image (f - cfo) and the unshifted residual (f) far down. Also: cfo = 0
// must not wrap the model (byte-identical default path).
double probeDb(const std::vector<float>& y, size_t b, size_t e, double f_hz) {
    // Blackman-Harris windowed DFT magnitude at an arbitrary frequency.
    std::complex<double> acc(0.0, 0.0);
    double wsum = 0.0;
    const double len = static_cast<double>(e - b - 1);
    for (size_t n = b; n < e; ++n) {
        const double x = 2.0 * M_PI * static_cast<double>(n - b) / len;
        const double w = 0.35875 - 0.48829 * std::cos(x) + 0.14128 * std::cos(2 * x) -
                         0.01168 * std::cos(3 * x);
        acc += w * static_cast<double>(y[n]) *
               std::polar(1.0, -2.0 * M_PI * f_hz * static_cast<double>(n) / kSampleRate);
        wsum += w;
    }
    return 20.0 * std::log10(std::abs(acc) / wsum + 1e-30);
}

void checkStreamingCfoAcrossChunks() {
    using ultra::ota_channel_core::FrequencyShiftChannelModel;
    constexpr double kCfo = 3.7;
    for (double tone_hz : {141.0, 1500.0, 2880.0}) {
        ChannelConfig cfg;
        cfg.type = ChannelType::PASSTHROUGH;
        cfg.cfo_hz = static_cast<float>(kCfo);
        RngRoot root(0x5151u);
        auto chunked_model = createChannelModel(cfg, root, "cfo-test");
        auto whole_model = createChannelModel(cfg, root, "cfo-test");
        assert(dynamic_cast<FrequencyShiftChannelModel*>(chunked_model.get()) != nullptr);
        const size_t n = 144000;
        std::vector<float> tone(n);
        for (size_t i = 0; i < n; ++i) {
            tone[i] = static_cast<float>(std::sin(2.0 * M_PI * tone_hz * i / kSampleRate));
        }
        std::vector<float> chunked;
        chunked.reserve(n);
        uint32_t lcg = 12345u;
        size_t pos = 0;
        while (pos < n) {
            lcg = lcg * 1664525u + 1013904223u;
            const size_t chunk = std::min<size_t>(n - pos, 96 + (lcg >> 8) % 1824);  // 2-40 ms
            std::vector<float> in(tone.begin() + pos, tone.begin() + pos + chunk), o;
            chunked_model->process(in, pos, o);
            chunked.insert(chunked.end(), o.begin(), o.end());
            pos += chunk;
        }
        std::vector<float> whole;
        whole_model->process(tone, 0, whole);
        assert(chunked == whole);  // chunk-invariant, bit for bit
        const size_t b = 2 * FrequencyShiftChannelModel::groupDelaySamples(), e = n;
        const double wanted = probeDb(chunked, b, e, tone_hz + kCfo);
        const double image = probeDb(chunked, b, e, tone_hz - kCfo) - wanted;
        const double residual = probeDb(chunked, b, e, tone_hz) - wanted;
        double peak_f = tone_hz + kCfo, peak_db = -1e9;
        for (double f = tone_hz + kCfo - 0.3; f <= tone_hz + kCfo + 0.3; f += 0.01) {
            const double d = probeDb(chunked, b, e, f);
            if (d > peak_db) { peak_db = d; peak_f = f; }
        }
        std::cout << "streaming_cfo tone_hz=" << tone_hz << " peak_hz=" << peak_f
                  << " image_db=" << image << " residual_db=" << residual << "\n";
        assert(std::abs(peak_f - (tone_hz + kCfo)) <= 0.02);
        assert(image < -40.0 && residual < -40.0);
    }
    {
        ChannelConfig cfg;
        cfg.type = ChannelType::AWGN;
        cfg.cfo_hz = 0.0f;
        RngRoot root(0x5151u);
        auto model = createChannelModel(cfg, root, "cfo-zero");
        assert(dynamic_cast<FrequencyShiftChannelModel*>(model.get()) == nullptr);
    }
}

int main() {
    checkStreamingCfoAcrossChunks();
    {
        PassthroughChannelModel model;
        const std::vector<float> input{0.0f, 1.0f, -0.5f, 0.25f};
        assertNear(model.process(input), input, 0.0f);
    }

    {
        RngRoot root(0xabcdefu);
        AWGNChannelModel a(15.0f, root.childSeed("awgn"));
        AWGNChannelModel b(15.0f, root.childSeed("awgn"));
        const std::vector<float> zeros(16, 0.0f);
        assertNear(a.process(zeros), b.process(zeros), 0.0f);
    }

    checkRepeat(ChannelType::AWGN);
    checkRepeat(ChannelType::GOOD);
    checkRepeat(ChannelType::MODERATE);
    checkRepeat(ChannelType::POOR);

    checkDelaySpread(ChannelType::GOOD, 24);
    checkDelaySpread(ChannelType::MODERATE, 48);
    checkDelaySpread(ChannelType::POOR, 96);

    checkAwgnStatisticsAndSpectrum();
    checkWattersonNoiseAndFading();
    checkRealHfLoopScaling();
    checkCfoInjectionPreservesBandEdgeTone();

    const auto awgn = runModel(ChannelType::AWGN, 0x1234u);
    const std::vector<float> expected_awgn{
        -0.014517599f, 0.017606046f, -0.134695336f, 0.469496787f,
        -0.440251589f, 0.006981332f, 0.318243772f, -0.168747187f};
    assertNear(awgn, expected_awgn, 1.0e-6f);

    std::cout << "channel core models deterministic\n";
    return 0;
}
