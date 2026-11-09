// TxPaAlcChannelModel: the sending radio's ALC and PA in OTASim (gui_qso_scenario.sh --tx-pa).
//
//  1. with the PA far from saturation and no ALC action, the output is the input delayed by the
//     Hilbert group delay (the envelope estimate never touches the signal path);
//  2. an overdriven tone is held at PEP by the ALC (gain -> PEP / envelope) and the PA's Rapp
//     compression at that point;
//  3. the ALC attacks within a few attack constants and releases with the release constant;
//  4. driving an OFDM-like 59-tone signal 3 dB into ALC lowers its peak-to-average ratio and
//     raises its average power relative to the same signal held linear (the physical trade);
//  5. random chunking gives the identical stream; the spec parser and the factory wiring.
#include "ota_channel_core/models.hpp"
#include "ota_channel_core/rng.hpp"

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

constexpr double kFs = 48000.0;
constexpr int kDelay = TxPaAlcChannelModel::kDelaySamples;

std::vector<float> run(TxPaAlcChannelModel& m, const std::vector<float>& in, size_t block) {
    std::vector<float> out, chunk;
    for (size_t pos = 0; pos < in.size(); pos += block) {
        const size_t n = std::min(block, in.size() - pos);
        m.process(std::span<const float>(in.data() + pos, n), pos, chunk);
        out.insert(out.end(), chunk.begin(), chunk.end());
    }
    return out;
}

TxPaAlcChannelModel make(TxPaConfig c) {
    c.enabled = true;
    return TxPaAlcChannelModel(std::make_unique<PassthroughChannelModel>(), c,
                               static_cast<uint32_t>(kFs));
}

std::vector<float> tone(double amp, double f, size_t n) {
    std::vector<float> v(n);
    for (size_t k = 0; k < n; ++k) v[k] = static_cast<float>(amp * std::sin(2.0 * M_PI * f * k / kFs));
    return v;
}

double peakAbs(const std::vector<float>& v, size_t from, size_t to) {
    double p = 0.0;
    for (size_t k = from; k < to; ++k) p = std::max(p, std::abs(static_cast<double>(v[k])));
    return p;
}

double rms(const std::vector<float>& v, size_t from, size_t to) {
    double s = 0.0;
    for (size_t k = from; k < to; ++k) s += static_cast<double>(v[k]) * v[k];
    return std::sqrt(s / static_cast<double>(to - from));
}
}  // namespace

int main() {
    // 1. Far from saturation, below PEP: a delayed copy.
    {
        TxPaConfig c;
        c.sat_db = 20.0f;
        auto m = make(c);
        const auto in = tone(0.3, 1234.0, 48000);
        const auto out = run(m, in, 480);
        double err = 0.0;
        for (size_t k = kDelay + 2000; k < in.size(); ++k) {
            err = std::max(err, std::abs(static_cast<double>(out[k]) - in[k - kDelay]));
        }
        check(err < 2e-4, "below PEP, PA far from saturation: output = input delayed by the Hilbert delay");
    }

    // 2. A tone at 0.8 against PEP 0.5: the ALC settles at 0.625 and the output peak sits at
    //    PEP times the Rapp gain there (1 / (1 + 10^(-sat_db/5))^(1/4) = 0.946 for 3 dB, p = 2).
    {
        TxPaConfig c;
        auto m = make(c);
        const auto in = tone(0.8, 1500.0, 48000);
        const auto out = run(m, in, 960);
        const double pk = peakAbs(out, 24000, 48000);
        const double want = 0.5 / std::pow(1.0 + std::pow(10.0, -3.0 / 5.0), 0.25);
        char msg[160];
        std::snprintf(msg, sizeof(msg), "overdriven tone held at PEP: peak %.4f (expect %.4f), ALC gain %.3f (0.625)",
                      pk, want, m.alcGain());
        check(std::abs(pk - want) < 0.01 && std::abs(m.alcGain() - 0.625) < 0.01, msg);
    }

    // 3. Attack / release: 0.3 -> 0.8 at 0.5 s, back to 0.3 at 1.5 s.
    {
        TxPaConfig c;  // attack 2 ms, release 300 ms
        auto m = make(c);
        std::vector<float> in(static_cast<size_t>(2.5 * kFs));
        for (size_t k = 0; k < in.size(); ++k) {
            const double t = k / kFs;
            const double a = (t >= 0.5 && t < 1.5) ? 0.8 : 0.3;
            in[k] = static_cast<float>(a * std::sin(2.0 * M_PI * 1000.0 * k / kFs));
        }
        std::vector<float> chunk;
        double g_after_attack = 0.0, g_after_release = 0.0;
        for (size_t pos = 0; pos < in.size(); pos += 48) {
            m.process(std::span<const float>(in.data() + pos, std::min<size_t>(48, in.size() - pos)), pos, chunk);
            const double t = (pos + 48) / kFs;
            if (g_after_attack == 0.0 && t >= 0.5 + kDelay / kFs + 0.012) g_after_attack = m.alcGain();
            if (g_after_release == 0.0 && t >= 1.5 + kDelay / kFs + 0.300) g_after_release = m.alcGain();
        }
        const double want_rel = 1.0 - (1.0 - 0.625) * std::exp(-1.0);
        char msg[200];
        std::snprintf(msg, sizeof(msg),
                      "ALC attack: gain %.3f 12 ms after the step (0.625 target); release: %.3f after 300 ms (%.3f)",
                      g_after_attack, g_after_release, want_rel);
        check(std::abs(g_after_attack - 0.625) < 0.02 && std::abs(g_after_release - want_rel) < 0.03, msg);
    }

    // 4. A 59-tone OFDM-like signal (random phases) driven 3 dB into ALC vs held linear.
    {
        std::mt19937 rng(9);
        std::uniform_real_distribution<double> ph(0.0, 2.0 * M_PI);
        const size_t n = static_cast<size_t>(2.0 * kFs);
        std::vector<float> x(n, 0.0f);
        for (int c = 0; c < 59; ++c) {
            const double f = 140.625 + 46.875 * c;
            const double p0 = ph(rng);
            for (size_t k = 0; k < n; ++k) x[k] += static_cast<float>(std::cos(2.0 * M_PI * f * k / kFs + p0));
        }
        const double xpk = peakAbs(x, 0, n);
        for (auto& v : x) v = static_cast<float>(v * 0.5 / xpk);  // peak exactly at PEP
        TxPaConfig lin;
        lin.sat_db = 20.0f;
        auto ml = make(lin);
        const auto yl = run(ml, x, 4800);
        TxPaConfig hot;
        hot.drive_db = 3.0f;
        auto mh = make(hot);
        const auto yh = run(mh, x, 4800);
        const size_t a = n / 2, b = n;
        const double papr_l = 20.0 * std::log10(peakAbs(yl, a, b) / rms(yl, a, b));
        const double papr_h = 20.0 * std::log10(peakAbs(yh, a, b) / rms(yh, a, b));
        const double gain_db = 20.0 * std::log10(rms(yh, a, b) / rms(yl, a, b));
        char msg[200];
        std::snprintf(msg, sizeof(msg),
                      "59-tone signal 3 dB into ALC: PAPR %.2f -> %.2f dB, average power %+.2f dB, peak %.3f <= PEP",
                      papr_l, papr_h, gain_db, peakAbs(yh, a, b));
        check(papr_h < papr_l - 0.3 && gain_db > 0.0 && peakAbs(yh, a, b) <= 0.5 + 1e-3, msg);
    }

    // 5. Chunk invariance, parser, factory.
    {
        std::mt19937 rng(3);
        std::normal_distribution<float> g(0.0f, 0.3f);
        std::vector<float> in(100000);
        for (auto& v : in) v = g(rng);
        TxPaConfig c;
        c.drive_db = 4.0f;
        auto whole = make(c);
        const auto ref = run(whole, in, in.size());
        auto chunked = make(c);
        std::vector<float> out, chunk;
        std::uniform_int_distribution<size_t> len(1, 2000);
        for (size_t pos = 0; pos < in.size();) {
            const size_t k = std::min(len(rng), in.size() - pos);
            chunked.process(std::span<const float>(in.data() + pos, k), pos, chunk);
            out.insert(out.end(), chunk.begin(), chunk.end());
            pos += k;
        }
        bool same = out.size() == ref.size();
        for (size_t k = 0; same && k < out.size(); ++k) same = out[k] == ref[k];
        check(same, "random chunk sizes give the identical output stream");

        const auto d = parseTxPaSpec("on");
        const auto s = parseTxPaSpec("pep=0.4,drive_db=6,p=3,sat_db=2,attack_ms=5,release_ms=150");
        check(d && d->enabled && d->pep == 0.5f && d->drive_db == 0.0f, "\"on\" gives the documented defaults");
        check(s && s->pep == 0.4f && s->drive_db == 6.0f && s->rapp_p == 3.0f && s->sat_db == 2.0f &&
                  s->attack_ms == 5.0f && s->release_ms == 150.0f,
              "a full spec parses every key");
        check(!parseTxPaSpec("pep=2") && !parseTxPaSpec("volume=3") && !parseTxPaSpec("drive_db=x"),
              "out-of-range values, unknown keys and garbage are rejected");

        RngRoot root(1);
        ChannelConfig plain{.type = ChannelType::PASSTHROUGH};
        auto m0 = createChannelModel(plain, root, "rx");
        ChannelConfig withpa = plain;
        withpa.tx_pa = *d;
        auto m1 = createChannelModel(withpa, root, "rx");
        check(dynamic_cast<TxPaAlcChannelModel*>(m0.get()) == nullptr &&
                  dynamic_cast<TxPaAlcChannelModel*>(m1.get()) != nullptr,
              "the factory adds the ALC/PA only when enabled");
    }

    if (g_fail == 0) {
        std::printf("PASS: TX ALC/PA model\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
