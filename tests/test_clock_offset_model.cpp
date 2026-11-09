// ClockOffsetChannelModel: the OTASim sample-clock offset between two soundcards.
//
//  1. zero offset is an exact delay of the guard (the interpolator is the identity at frac 0);
//  2. a tone comes out exactly where a receiver clocked (1 + ppm 1e-6) slower would sample it,
//     for low / mid / band-edge tones and both signs (in-band error < -80 dB);
//  3. a 10 s burst at +50 ppm arrives 24 samples short (the drift a modem must track);
//  4. the output does not depend on how the stream is cut into process() calls;
//  5. the factory wraps only for a non-zero offset, and SessionConfig routes it per receiver.
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/rng.hpp"
#include "ota_channel_core/session_config.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

using namespace ultra::ota_channel_core;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
    std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++g_fail;
}

std::vector<float> runModel(IChannelModel& m, const std::vector<float>& in, size_t block) {
    std::vector<float> out, chunk;
    out.reserve(in.size());
    for (size_t pos = 0; pos < in.size(); pos += block) {
        const size_t n = std::min(block, in.size() - pos);
        std::span<const float> s(in.data() + pos, n);
        m.process(s, pos, chunk);
        out.insert(out.end(), chunk.begin(), chunk.end());
    }
    return out;
}

ClockOffsetChannelModel makeModel(float ppm) {
    return ClockOffsetChannelModel(std::make_unique<PassthroughChannelModel>(), ppm,
                                   kDefaultSampleRate);
}
}  // namespace

int main() {
    const double fs = kDefaultSampleRate;

    // 1. Zero offset: an exact delay of the guard.
    {
        std::mt19937 rng(7);
        std::normal_distribution<float> g(0.0f, 0.3f);
        std::vector<float> in(20000);
        for (auto& v : in) v = g(rng);
        auto m = makeModel(0.0f);
        const size_t d = m.initialLatencySamples();
        const auto out = runModel(m, in, 480);
        double err = 0.0;
        for (size_t n = d; n < in.size(); ++n) err = std::max(err, std::abs(double(out[n]) - in[n - d]));
        check(d == static_cast<size_t>(ClockOffsetChannelModel::kHalfTaps) + 2u,
              "0 ppm: guard is the interpolator half-length + 2");
        check(err < 1e-6, "0 ppm: output is the input delayed by the guard");
    }

    // 2. Tones land where a (1 + ppm 1e-6)-scaled receiver samples them.
    for (const float ppm : {100.0f, -100.0f, 50.0f}) {
        for (const double f : {300.0, 1500.0, 2900.0}) {
            const size_t N = static_cast<size_t>(fs * 12.0);
            std::vector<float> in(N);
            for (size_t k = 0; k < N; ++k) in[k] = static_cast<float>(std::sin(2.0 * M_PI * f * k / fs));
            auto m = makeModel(ppm);
            const auto out = runModel(m, in, 480);
            const double step = 1.0 + ppm * 1e-6;
            const double p0 = -static_cast<double>(m.initialLatencySamples());
            double e2 = 0.0, s2 = 0.0;
            for (size_t n = 0; n < N; ++n) {
                const double p = p0 + n * step;
                if (p < 64.0 || p > N - 64.0) continue;
                const double want = std::sin(2.0 * M_PI * f * p / fs);
                e2 += (out[n] - want) * (out[n] - want);
                s2 += want * want;
            }
            const double err_db = 10.0 * std::log10(e2 / s2);
            char msg[160];
            std::snprintf(msg, sizeof(msg), "%+.0f ppm, %.0f Hz tone: time-scaled exactly (error %.1f dB < -80)",
                          ppm, f, err_db);
            check(err_db < -80.0 && m.slips() == 0, msg);
        }
    }

    // 3. Two clicks 10 s apart arrive 24 samples closer at +50 ppm (480000 / 1.00005).
    {
        const size_t N = static_cast<size_t>(fs * 12.0);
        std::vector<float> in(N, 0.0f);
        const size_t a = 48000, b = a + 480000;
        in[a] = 1.0f;
        in[b] = 1.0f;
        auto m = makeModel(50.0f);
        const auto out = runModel(m, in, 1000);
        auto peak = [&](size_t lo, size_t hi) {
            size_t best = lo;
            for (size_t n = lo; n < hi; ++n) if (out[n] > out[best]) best = n;
            // sub-sample peak from a parabola through the three samples around it
            const double y0 = out[best - 1], y1 = out[best], y2 = out[best + 1];
            return best + 0.5 * (y0 - y2) / (y0 - 2.0 * y1 + y2);
        };
        const double pa = peak(a - 2000 + m.initialLatencySamples(), a + 2000 + m.initialLatencySamples());
        const double pb = peak(b - 2000, b + 2000 + m.initialLatencySamples());
        const double spacing = pb - pa;
        char msg[160];
        std::snprintf(msg, sizeof(msg), "+50 ppm: clicks 480000 apart arrive %.2f apart (expect 479976.00)", spacing);
        check(std::abs(spacing - 480000.0 / 1.00005) < 0.1, msg);
    }

    // 4. Chunking does not change a single output sample.
    {
        std::mt19937 rng(11);
        std::normal_distribution<float> g(0.0f, 0.3f);
        std::vector<float> in(200000);
        for (auto& v : in) v = g(rng);
        auto whole = makeModel(-73.0f);
        const auto ref = runModel(whole, in, in.size());
        auto chunked = makeModel(-73.0f);
        std::vector<float> out, chunk;
        std::uniform_int_distribution<size_t> len(1, 3000);
        for (size_t pos = 0; pos < in.size();) {
            const size_t n = std::min(len(rng), in.size() - pos);
            chunked.process(std::span<const float>(in.data() + pos, n), pos, chunk);
            out.insert(out.end(), chunk.begin(), chunk.end());
            pos += n;
        }
        bool same = out.size() == ref.size();
        for (size_t n = 0; same && n < out.size(); ++n) same = out[n] == ref[n];
        check(same, "-73 ppm: random chunk sizes give the identical output stream");
    }

    // 5. Factory and per-receiver routing.
    {
        RngRoot root(42);
        ChannelConfig plain{.type = ChannelType::PASSTHROUGH};
        auto m0 = createChannelModel(plain, root, "rx");
        check(dynamic_cast<ClockOffsetChannelModel*>(m0.get()) == nullptr,
              "0 ppm: the factory does not wrap (byte-identical models)");
        ChannelConfig skewed = plain;
        skewed.clock_ppm = 40.0f;
        auto m1 = createChannelModel(skewed, root, "rx");
        auto* c = dynamic_cast<ClockOffsetChannelModel*>(m1.get());
        check(c != nullptr && std::abs(c->ppm() - 40.0f) < 1e-6f,
              "non-zero ppm: the factory wraps the model in ClockOffsetChannelModel");

        SessionConfig session;
        session.rx_clock_ppm = {{"ALPHA", 25.0f}, {"BRAVO", -25.0f}};
        check(session.channelConfigFor("ALPHA").clock_ppm == 25.0f &&
                  session.channelConfigFor("BRAVO").clock_ppm == -25.0f &&
                  session.channelConfigFor("CHARLIE").clock_ppm == 0.0f,
              "SessionConfig routes the offset per receiver (absent = 0)");
    }

    if (g_fail == 0) {
        std::printf("PASS: clock offset model\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
