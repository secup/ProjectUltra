#include "gui/modem/idle_noise_snr_estimator.hpp"
#include "ota_channel_core/models.hpp"
#include "sim/channel_calibration.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
        } \
    } while (0)

enum class MatrixChannel {
    AWGN,
    RealHfLoop
};

const char* channelName(MatrixChannel channel) {
    switch (channel) {
        case MatrixChannel::AWGN: return "AWGN";
        case MatrixChannel::RealHfLoop: return "real_hf_loop";
    }
    return "UNKNOWN";
}

struct EstimateResult {
    bool valid = false;
    float snr_db = 0.0f;
    float latest_snr_db = 0.0f;
    double true_in_band_snr_db = 0.0;
    double delta_db = 0.0;
    float in_band_noise_rms = 0.0f;
    double fir_energy = 0.0;
    double enbw_hz = 0.0;
    size_t windows = 0;
    size_t reference_windows = 0;
};

struct InBandReference {
    double smoothed_snr_db = 0.0;
    size_t windows = 0;
};

std::vector<float> loadRealHfLoopNoise() {
    namespace channel = ultra::ota_channel_core;
    const std::vector<std::string> candidates = {
        "recordings/ota_noise_bed_2026-05-18_20m_14100/noise_bed.wav",
        "../recordings/ota_noise_bed_2026-05-18_20m_14100/noise_bed.wav",
        "../../recordings/ota_noise_bed_2026-05-18_20m_14100/noise_bed.wav",
    };

    std::string last_error;
    for (const std::string& path : candidates) {
        try {
            return channel::loadRealHfLoopNoiseBedWav(path);
        } catch (const std::exception& e) {
            last_error = e.what();
        }
    }

    throw std::runtime_error("failed to load real_hf_loop noise bed: " + last_error);
}

std::vector<float> synthesizeIdleNoise(
    MatrixChannel channel,
    float snr_db,
    uint32_t seed,
    const std::shared_ptr<const std::vector<float>>& real_hf_loop,
    size_t count) {
    std::vector<float> silence(count, 0.0f);

    if (channel == MatrixChannel::AWGN) {
        ultra::ota_channel_core::RngRoot root(seed);
        ultra::ota_channel_core::AWGNChannelModel model(
            snr_db,
            root.childSeed("idle-noise-snr-awgn"));
        return model.process(silence);
    }

    ultra::ota_channel_core::RealHfLoopChannelModel model(
        snr_db,
        real_hf_loop,
        seed);
    return model.process(silence);
}

InBandReference computeInBandReference(
    const std::vector<float>& samples,
    const ultra::gui::IdleNoiseSNREstimator::Config& config) {
    auto filter = ultra::FIRFilter::bandpass(
        static_cast<size_t>(std::max(3, config.filter_taps)),
        config.band_low_hz,
        config.band_high_hz,
        config.sample_rate);

    const float alpha = std::clamp(config.ema_alpha, 0.0f, 1.0f);
    double window_sum_sq = 0.0;
    size_t window_fill = 0;
    bool valid = false;
    InBandReference reference;

    for (float sample : samples) {
        const float filtered = filter.process(sample);
        window_sum_sq += static_cast<double>(filtered) * static_cast<double>(filtered);
        ++window_fill;

        if (window_fill < config.window_samples) {
            continue;
        }

        const double noise_power =
            std::max(window_sum_sq / static_cast<double>(config.window_samples),
                     std::numeric_limits<double>::min());
        const double instant_snr_db =
            10.0 * std::log10(ultra::sim::kModemReferencePower / noise_power);
        if (!valid) {
            reference.smoothed_snr_db = instant_snr_db;
            valid = true;
        } else {
            reference.smoothed_snr_db =
                static_cast<double>(alpha) * instant_snr_db +
                (1.0 - static_cast<double>(alpha)) * reference.smoothed_snr_db;
        }
        ++reference.windows;

        window_sum_sq = 0.0;
        window_fill = 0;
    }

    return reference;
}

EstimateResult measureIdleSNR(
    MatrixChannel channel,
    float snr_db,
    uint32_t seed,
    const std::shared_ptr<const std::vector<float>>& real_hf_loop) {
    constexpr size_t kIdleSamples = 48000 * 4;

    ultra::gui::IdleNoiseSNREstimator::Config config;

    const std::vector<float> rx =
        synthesizeIdleNoise(channel, snr_db, seed, real_hf_loop, kIdleSamples);
    const InBandReference reference = computeInBandReference(rx, config);

    ultra::gui::IdleNoiseSNREstimator estimator(config);
    // The peer measured at the reference level (what the decoder measures from the chirp when
    // the station hears it at the simulator's level).
    estimator.setSignalReferencePower(ultra::sim::kModemReferencePower);
    estimator.observeIdleAudio(rx.data(), rx.size());
    const auto snapshot = estimator.snapshot();

    EstimateResult result;
    result.valid = snapshot.valid;
    result.snr_db = snapshot.idle_in_band_snr_db;
    result.latest_snr_db = snapshot.latest_instant_idle_in_band_snr_db;
    result.true_in_band_snr_db = reference.smoothed_snr_db;
    result.delta_db = static_cast<double>(snapshot.idle_in_band_snr_db) -
                      reference.smoothed_snr_db;
    result.in_band_noise_rms = snapshot.normalized_noise_rms;
    result.fir_energy = snapshot.fir_energy;
    result.enbw_hz = snapshot.equivalent_noise_bandwidth_hz;
    result.windows = snapshot.windows_observed;
    result.reference_windows = reference.windows;
    return result;
}

void checkChannel(
    MatrixChannel channel,
    const std::shared_ptr<const std::vector<float>>& real_hf_loop) {
    constexpr float kToleranceDb = 1.5f;
    const std::vector<float> snrs = {-6.0f, 0.0f, 6.0f, 12.0f, 18.0f, 24.0f};
    const std::vector<uint32_t> seeds = {1u, 2u, 3u, 4u, 5u};

    // The channel knob and idle meter are both in-band. The reference below
    // independently filters the generated noise and verifies the estimator is
    // still reporting the true FIR-band noise power.
    for (float snr_db : snrs) {
        double reported_sum = 0.0;
        double true_sum = 0.0;
        double max_abs_delta = 0.0;
        int count = 0;
        EstimateResult last;

        for (uint32_t seed : seeds) {
            const EstimateResult r = measureIdleSNR(channel, snr_db, seed, real_hf_loop);
            last = r;
            CHECK(r.valid, std::string(channelName(channel)) + " idle SNR should be valid");
            CHECK(r.windows >= 10,
                  std::string(channelName(channel)) + " should observe multiple idle windows");
            CHECK(r.windows == r.reference_windows,
                  std::string(channelName(channel)) + " should match reference window count");
            CHECK(std::isfinite(r.snr_db),
                  std::string(channelName(channel)) + " idle SNR should be finite");
            CHECK(std::abs(r.delta_db) <= kToleranceDb,
                  std::string(channelName(channel)) +
                      " idle-noise SNR should match true in-band SNR");
            if (r.valid && std::isfinite(r.snr_db)) {
                reported_sum += r.snr_db;
                true_sum += r.true_in_band_snr_db;
                max_abs_delta = std::max(max_abs_delta, std::abs(r.delta_db));
                ++count;
            }
        }

        CHECK(count == static_cast<int>(seeds.size()),
              std::string(channelName(channel)) + " should produce all seed estimates");
        if (count == 0) {
            continue;
        }

        const double reported = reported_sum / static_cast<double>(count);
        const double true_snr = true_sum / static_cast<double>(count);
        const double delta = reported - true_snr;
        std::cout << std::fixed << std::setprecision(2)
                  << channelName(channel)
                  << " configured=" << snr_db
                  << " true_in_band=" << true_snr
                  << " reported=" << reported
                  << " delta=" << delta
                  << " max_abs_delta=" << max_abs_delta
                  << " tolerance=" << kToleranceDb
                  << " in_band_noise_rms=" << last.in_band_noise_rms
                  << " fir_energy=" << std::setprecision(8) << last.fir_energy
                  << " enbw_hz=" << std::setprecision(2) << last.enbw_hz
                  << "\n";
    }
}

}  // namespace

// INV-RX-LEVEL-001: the idle SNR reads the same at any receive level. The same noise 40 dB
// quieter, with the peer measured 40 dB quieter too (the decoder measures its chirp through the
// same chain), must give the same SNR; and no SNR exists before a peer is measured.
void checkLevelInvariance() {
    ultra::gui::IdleNoiseSNREstimator::Config config;
    const std::vector<float> rx = synthesizeIdleNoise(MatrixChannel::AWGN, 12.0f, 7u, nullptr,
                                                      48000 * 4);
    std::vector<float> quiet(rx);
    for (float& v : quiet) v *= 0.01f;  // -40 dB
    ultra::gui::IdleNoiseSNREstimator loud_est(config), quiet_est(config);
    loud_est.observeIdleAudio(rx.data(), rx.size());
    quiet_est.observeIdleAudio(quiet.data(), quiet.size());
    CHECK(loud_est.snapshot().valid && !loud_est.snapshot().snr_valid,
          "noise floor measured but no SNR before a peer's signal power is known");
    loud_est.setSignalReferencePower(ultra::sim::kModemReferencePower);
    quiet_est.setSignalReferencePower(ultra::sim::kModemReferencePower * 1e-4);
    const auto a = loud_est.snapshot();
    const auto b = quiet_est.snapshot();
    std::cout << "level invariance: 0 dB " << a.idle_in_band_snr_db << " dB, -40 dB "
              << b.idle_in_band_snr_db << " dB\n";
    CHECK(a.snr_valid && b.snr_valid, "SNR valid once the peer is measured");
    CHECK(std::abs(a.idle_in_band_snr_db - b.idle_in_band_snr_db) < 0.01f,
          "idle SNR is independent of the receive level");
}

int main() {
    ultra::setLogLevel(ultra::LogLevel::ERROR);

    const auto real_hf_loop = std::make_shared<const std::vector<float>>(
        loadRealHfLoopNoise());

    checkChannel(MatrixChannel::AWGN, real_hf_loop);
    checkChannel(MatrixChannel::RealHfLoop, real_hf_loop);
    checkLevelInvariance();

    if (tests_failed == 0) {
        std::cout << "PASS: Idle-noise in-band SNR calibration ("
                  << tests_run << " checks)\n";
        return 0;
    }
    std::cout << "FAIL: " << tests_failed << "/" << tests_run << " checks failed\n";
    return 1;
}
