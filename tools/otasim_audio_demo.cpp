// otasim_audio_demo - generates a ~60s WAV showing the channel models
// the OTASim server applies. 4 segments x 15s through:
//   passthrough  -> AWGN SNR=10  -> Watterson good SNR=10  -> Watterson moderate SNR=5
// Each segment: 2 s silence header, 6 s steady tone @ 1500 Hz, 1 s gap,
//               6 s frequency sweep 300 -> 2700 Hz.

#include "io/wav_io.hpp"
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/rng.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr uint32_t kSampleRate = 48000;
constexpr float kSegmentSeconds = 15.0f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kSignalAmplitude = 0.20f;  // -14 dBFS; same scale as modem
                                            // reference RMS so AWGN SNR is calibrated

std::vector<float> generateSegment() {
    const size_t n = static_cast<size_t>(kSegmentSeconds * kSampleRate);
    std::vector<float> out(n, 0.0f);

    const size_t silence_end = 2 * kSampleRate;
    const size_t tone_end    = silence_end + 6 * kSampleRate;
    const size_t gap_end     = tone_end + 1 * kSampleRate;
    const size_t sweep_end   = gap_end + 6 * kSampleRate;

    constexpr float tone_hz = 1500.0f;
    float phase = 0.0f;
    for (size_t i = silence_end; i < tone_end && i < n; ++i) {
        phase += kTwoPi * tone_hz / static_cast<float>(kSampleRate);
        if (phase > kTwoPi) phase -= kTwoPi;
        out[i] = kSignalAmplitude * std::sin(phase);
    }

    constexpr float sweep_start_hz = 300.0f;
    constexpr float sweep_end_hz = 2700.0f;
    const size_t sweep_len = sweep_end > gap_end ? sweep_end - gap_end : 0;
    phase = 0.0f;
    for (size_t i = gap_end; i < sweep_end && i < n; ++i) {
        const float t = static_cast<float>(i - gap_end) /
                        static_cast<float>(sweep_len);
        const float freq = sweep_start_hz + (sweep_end_hz - sweep_start_hz) * t;
        phase += kTwoPi * freq / static_cast<float>(kSampleRate);
        if (phase > kTwoPi) phase -= kTwoPi;
        out[i] = kSignalAmplitude * std::sin(phase);
    }
    return out;
}

std::vector<float> applyChannel(ultra::ota_channel_core::ChannelType type,
                                float snr_db,
                                uint64_t seed,
                                const std::vector<float>& input) {
    using namespace ultra::ota_channel_core;
    ChannelConfig cfg;
    cfg.type = type;
    cfg.snr_db = snr_db;
    cfg.seed = seed;
    cfg.sample_rate = kSampleRate;
    RngRoot rng(seed);
    auto model = createChannelModel(cfg, rng, "demo");
    return model->process(std::span<const float>(input.data(), input.size()));
}

}  // namespace

int main(int argc, char** argv) {
    std::string out_path = "otasim_channel_demo.wav";
    if (argc > 1) {
        out_path = argv[1];
    }

    using namespace ultra::ota_channel_core;

    struct Step {
        ChannelType type;
        float snr_db;
        const char* label;
    };
    const Step steps[] = {
        {ChannelType::PASSTHROUGH, 80.0f, "passthrough (clean)"},
        {ChannelType::AWGN,        10.0f, "AWGN SNR=10 dB"},
        {ChannelType::GOOD,        10.0f, "Watterson Good SNR=10 dB"},
        {ChannelType::MODERATE,     5.0f, "Watterson Moderate SNR=5 dB"},
    };

    std::vector<float> all;
    all.reserve(static_cast<size_t>(kSegmentSeconds * kSampleRate * 4));

    const auto signal = generateSegment();
    uint64_t seed = 0xC0FFEEull;
    for (const auto& step : steps) {
        std::cerr << "[demo] segment " << step.label << "\n";
        auto out = applyChannel(step.type, step.snr_db, seed++, signal);
        if (out.size() != signal.size()) {
            std::cerr << "[demo] channel " << channelTypeName(step.type)
                      << " returned " << out.size() << " samples (expected "
                      << signal.size() << "); padding/truncating\n";
            out.resize(signal.size(), 0.0f);
        }
        all.insert(all.end(), out.begin(), out.end());
    }

    if (!ultra::tools::io::writeWavPCM16Mono(out_path, all, kSampleRate)) {
        std::cerr << "[demo] failed to write WAV to " << out_path << "\n";
        return 1;
    }
    std::cerr << "[demo] wrote " << all.size() << " samples ("
              << static_cast<double>(all.size()) / kSampleRate
              << " s) to " << out_path << "\n";
    return 0;
}
