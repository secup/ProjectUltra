// mfsk_proto: feasibility of noncoherent M-FSK on a PEAK-limited transmitter, before any protocol
// work. One LDPC frame per trial through the production SimulatedChannel at the GUI gate's
// --snr-db scale (noise sized from the 0.3048 in-band reference; the burst's PEAK at 0.5, as
// --tx-model hardware does), decoded with the production LDPC decoder.
//
// TX: continuous-phase FSK, M tones evenly spaced over [f_lo, f_hi], one tone per symbol of
// `sps` samples - constant envelope, so the average power IS the peak power (PEP/2).
// RX: per symbol, the complex correlation with each tone (sample-exact timing, CFO known to a
// residual); noise power per correlator from the median of the non-maximal tones; signal
// amplitude from the mean maximal energy; symbol metric log I0(2 a |r| / s2) (the noncoherent
// likelihood), bit LLRs by log-sum-exp over the tones carrying each bit value.
//
// Usage: mfsk_proto [--channel awgn|good|moderate|poor] [--trials N]
#include "ultra/fec.hpp"
#include "ota_channel_core/channel.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace ultra;
using Cx = std::complex<double>;

namespace {

double logI0(double x) {
    // log of the modified Bessel function I0, stable for large x
    if (x < 3.75) {
        const double t = (x / 3.75) * (x / 3.75);
        return std::log(1.0 + t * (3.5156229 + t * (3.0899424 + t * (1.2067492 + t * (0.2659732 +
                        t * (0.0360768 + t * 0.0045813))))));
    }
    const double t = 3.75 / x;
    return x - 0.5 * std::log(x) +
           std::log(0.39894228 + t * (0.01328592 + t * (0.00225319 + t * (-0.00157565 +
                    t * (0.00916281 + t * (-0.02057706 + t * (0.02635537 + t * (-0.01647633 +
                    t * 0.00392377))))))));
}

struct Cfg {
    int M = 8;
    int sps = 1024;
    double f_lo = 500, f_hi = 2500;
    CodeRate rate = CodeRate::R1_4;
    int cws = 2;
};

double rateValue(CodeRate r) {
    switch (r) {
        case CodeRate::R1_4: return 0.25;
        case CodeRate::R1_2: return 0.5;
        case CodeRate::R2_3: return 2.0 / 3.0;
        case CodeRate::R3_4: return 0.75;
        default: return 0.5;
    }
}

bool trial(const Cfg& c, float snr_db, ota_channel_core::ChannelType ch_type, uint32_t seed,
           double cfo_resid_hz, double* airtime_s) {
    std::mt19937 rng(seed * 7919u + 13u);
    const int bps = int(std::lround(std::log2(c.M)));
    LDPCEncoder enc(c.rate);
    LDPCDecoder dec(c.rate);
    dec.setMaxIterations(50);
    const int k = int(std::lround(648 * rateValue(c.rate)));
    std::vector<Bytes> info(c.cws);
    std::vector<uint8_t> bits;
    for (int w = 0; w < c.cws; ++w) {
        info[w].resize(size_t(k / 8));
        for (auto& b : info[w]) b = uint8_t(rng());
        const Bytes cw = enc.encode(info[w]);
        for (int i = 0; i < 648; ++i) bits.push_back((cw[size_t(i / 8)] >> (7 - i % 8)) & 1);
    }
    // random bit interleaver across the frame (spreads a fade over every codeword)
    std::vector<int> perm(bits.size());
    for (size_t i = 0; i < perm.size(); ++i) perm[i] = int(i);
    std::mt19937 prng(12345);
    std::shuffle(perm.begin(), perm.end(), prng);
    std::vector<uint8_t> tx_bits(bits.size());
    for (size_t i = 0; i < bits.size(); ++i) tx_bits[i] = bits[size_t(perm[i])];
    while (tx_bits.size() % size_t(bps)) tx_bits.push_back(0);
    const size_t nsym = tx_bits.size() / size_t(bps);

    std::vector<double> f(size_t(c.M));
    for (int m = 0; m < c.M; ++m) f[size_t(m)] = c.f_lo + (c.f_hi - c.f_lo) * m / (c.M - 1);
    const double fs = 48000.0;
    std::vector<float> audio(48000, 0.0f);
    double ph = 0.0;
    std::vector<int> sym_tone(nsym);
    for (size_t s = 0; s < nsym; ++s) {
        int t = 0;
        for (int b = 0; b < bps; ++b) t = (t << 1) | tx_bits[s * size_t(bps) + size_t(b)];
        sym_tone[s] = t;
        const double w = 2.0 * M_PI * (f[size_t(t)] + cfo_resid_hz) / fs;
        for (int i = 0; i < c.sps; ++i) {
            audio.push_back(float(0.5 * std::cos(ph)));
            ph += w;
        }
        ph = std::fmod(ph, 2.0 * M_PI);
    }
    const size_t data_start = 48000;
    audio.resize(audio.size() + 48000, 0.0f);
    *airtime_s = double(nsym) * c.sps / fs;

    ota_channel_core::SimulatedChannel ch;
    ch.setSeed(seed);
    ch.configure(snr_db, ch_type);
    ch.transmitFromA(audio);
    const auto rx = ch.receiveForB(audio.size());

    // The channel model's own latency (fading taps, Hilbert pair): found once per channel type
    // from the known transmission at seed 1 - what the chirp's timing gives the real receiver.
    static std::vector<long> delay_cache(8, -1);
    long& delay = delay_cache[size_t(ch_type)];
    if (delay < 0) {
        double best = -1; long best_d = 0;
        auto score = [&](long d) {
            double acc = 0;
            for (size_t i = data_start; i < data_start + 24000 && i + size_t(d) < rx.size(); ++i)
                acc += double(audio[i]) * double(rx[i + size_t(d)]);
            return std::fabs(acc);
        };
        for (long d = 0; d < 8192; d += 1) { const double v = score(d); if (v > best) { best = v; best_d = d; } }
        delay = best_d;
        std::fprintf(stderr, "channel %d latency %ld samples\n", int(ch_type), delay);
    }
    const size_t rx_start = data_start + size_t(delay);
    // correlators
    std::vector<double> E(nsym * size_t(c.M));
    std::vector<double> mag(nsym * size_t(c.M));
    for (size_t s = 0; s < nsym; ++s) {
        const size_t s0 = rx_start + s * size_t(c.sps);
        for (int m = 0; m < c.M; ++m) {
            Cx acc(0, 0);
            const double w = 2.0 * M_PI * f[size_t(m)] / fs;
            for (int i = 0; i < c.sps; ++i) acc += double(rx[s0 + size_t(i)]) * std::polar(1.0, -w * i);
            mag[s * size_t(c.M) + size_t(m)] = std::abs(acc);
            E[s * size_t(c.M) + size_t(m)] = std::norm(acc);
        }
    }
    // noise: median of all non-maximal tone energies (exponential with mean s2: median = s2 ln 2)
    std::vector<double> nm;
    double sum_max = 0;
    for (size_t s = 0; s < nsym; ++s) {
        auto b = E.begin() + long(s * size_t(c.M));
        auto mx = std::max_element(b, b + c.M);
        sum_max += *mx;
        for (auto it = b; it != b + c.M; ++it) if (it != mx) nm.push_back(*it);
    }
    std::nth_element(nm.begin(), nm.begin() + long(nm.size() / 2), nm.end());
    const double s2 = nm[nm.size() / 2] / std::log(2.0);
    // E[sum over tones] = a^2 + M s2 (one tone carries the signal): unbiased at any SNR, where the
    // maximal tone is often a noise peak and its mean overstates a^2.
    double sum_all = 0;
    for (double v : E) sum_all += v;
    (void)sum_max;
    const double a2 = std::max(1e-12, sum_all / double(nsym) - double(c.M) * s2);
    const double a = std::sqrt(a2);
    if (std::getenv("MFSK_DEBUG") && seed == 1) {
        const double peak_snr = 0.125 / (0.30482664 * 0.30482664 / std::pow(10.0, snr_db / 10.0));
        std::fprintf(stderr, "snr %5.1f M %2d: measured Es/N0 %6.2f dB, theory (peak SNR x 3000 x T) %6.2f dB\n",
                     snr_db, c.M, 10 * std::log10(a2 / s2),
                     10 * std::log10(peak_snr * 3000.0 * c.sps / fs));
    }

    // Fading: each tone's amplitude moves with time and frequency. EM: posteriors of the sent tone
    // under the current amplitudes, then each tone's a^2 from its posterior-weighted excess energy
    // over a window of symbols and the neighbouring tones; MFSK_EM=0 keeps one global amplitude.
    const int em_iters = std::getenv("MFSK_EM") ? std::atoi(std::getenv("MFSK_EM")) : 2;
    const int W = std::getenv("MFSK_W") ? std::atoi(std::getenv("MFSK_W")) : 12;
    std::vector<double> amp(nsym * size_t(c.M), a);
    std::vector<double> lam(size_t(c.M));
    for (int it = 0; it < em_iters; ++it) {
        std::vector<double> post(nsym * size_t(c.M));
        for (size_t s = 0; s < nsym; ++s) {
            double mx = -1e300;
            for (int m = 0; m < c.M; ++m) {
                const size_t i = s * size_t(c.M) + size_t(m);
                lam[size_t(m)] = logI0(2.0 * amp[i] * mag[i] / s2) - amp[i] * amp[i] / s2;
                mx = std::max(mx, lam[size_t(m)]);
            }
            double z = 0;
            for (int m = 0; m < c.M; ++m) z += std::exp(lam[size_t(m)] - mx);
            for (int m = 0; m < c.M; ++m)
                post[s * size_t(c.M) + size_t(m)] = std::exp(lam[size_t(m)] - mx) / z;
        }
        for (size_t s = 0; s < nsym; ++s) {
            for (int m = 0; m < c.M; ++m) {
                double num = 0, den = 0;
                for (long ds = -W; ds <= W; ++ds) {
                    const long ss = long(s) + ds;
                    if (ss < 0 || ss >= long(nsym)) continue;
                    for (int dm = -1; dm <= 1; ++dm) {
                        const int mm = m + dm;
                        if (mm < 0 || mm >= c.M) continue;
                        const size_t i = size_t(ss) * size_t(c.M) + size_t(mm);
                        num += post[i] * (E[i] - s2);
                        den += post[i];
                    }
                }
                // a prior of one global-amplitude observation keeps a sparse window sane
                const double a2l = (num + a2) / (den + 1.0);
                amp[s * size_t(c.M) + size_t(m)] = std::sqrt(std::max(a2l, 0.01 * a2));
            }
        }
    }

    std::vector<float> llr_tx(tx_bits.size(), 0.0f);
    for (size_t s = 0; s < nsym; ++s) {
        for (int m = 0; m < c.M; ++m) {
            const size_t i = s * size_t(c.M) + size_t(m);
            lam[size_t(m)] = logI0(2.0 * amp[i] * mag[i] / s2) - amp[i] * amp[i] / s2;
        }
        for (int b = 0; b < bps; ++b) {
            double l0 = -1e300, l1 = -1e300;
            for (int m = 0; m < c.M; ++m) {
                const bool one = (m >> (bps - 1 - b)) & 1;
                double& acc = one ? l1 : l0;
                const double v = lam[size_t(m)];
                acc = (acc < v) ? v + std::log1p(std::exp(acc - v)) : acc + std::log1p(std::exp(v - acc));
            }
            llr_tx[s * size_t(bps) + size_t(b)] = float(std::clamp(l0 - l1, -30.0, 30.0));
        }
    }
    std::vector<float> llr(bits.size());
    for (size_t i = 0; i < bits.size(); ++i) llr[size_t(perm[i])] = llr_tx[i];
    for (int w = 0; w < c.cws; ++w) {
        const auto out = dec.decodeSoft(std::span<const float>(llr.data() + w * 648, 648));
        if (!dec.lastDecodeSuccess()) return false;
        if (!std::equal(info[size_t(w)].begin(), info[size_t(w)].end(), out.begin())) return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    int trials = 40;
    auto ch_type = ota_channel_core::ChannelType::AWGN;
    std::string ch_name = "awgn";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--trials") && i + 1 < argc) trials = std::atoi(argv[++i]);
        else if (!std::strcmp(argv[i], "--channel") && i + 1 < argc) {
            ch_name = argv[++i];
            if (ch_name == "good") ch_type = ota_channel_core::ChannelType::GOOD;
            else if (ch_name == "moderate") ch_type = ota_channel_core::ChannelType::MODERATE;
            else if (ch_name == "poor") ch_type = ota_channel_core::ChannelType::POOR;
        }
    }
    struct Case { const char* name; Cfg c; };
    std::vector<Case> cases;
    auto add = [&](const char* n, int M, int sps, CodeRate r, int cws) {
        Cfg c; c.M = M; c.sps = sps; c.rate = r; c.cws = cws; cases.push_back({n, c});
    };
    add("8FSK  1024 R1/4 2cw", 8, 1024, CodeRate::R1_4, 2);
    add("8FSK  1024 R1/2 1cw", 8, 1024, CodeRate::R1_2, 1);
    add("16FSK 1024 R1/4 2cw", 16, 1024, CodeRate::R1_4, 2);
    add("16FSK 1024 R1/2 1cw", 16, 1024, CodeRate::R1_2, 1);
    add("32FSK 1024 R1/4 2cw", 32, 1024, CodeRate::R1_4, 2);
    add("32FSK 1024 R1/2 1cw", 32, 1024, CodeRate::R1_2, 1);
    std::printf("M-FSK on a peak-limited TX, channel %s, %d trials, gate --snr-db scale "
                "(peak SNR = snr + 0.7 dB), residual CFO 1 Hz\n", ch_name.c_str(), trials);
    std::printf("%-22s %7s %7s |", "case", "bps", "air_s");
    const std::vector<float> snrs = {-16, -14, -12, -10, -8, -6};
    for (float s : snrs) std::printf(" %5.0f", s);
    std::printf("\n");
    for (const auto& cs : cases) {
        double air = 0;
        std::vector<int> ok(snrs.size(), 0);
        for (size_t si = 0; si < snrs.size(); ++si)
            for (int t = 0; t < trials; ++t) ok[si] += trial(cs.c, snrs[si], ch_type, uint32_t(t + 1), 1.0, &air);
        const double info_bits = cs.c.cws * 648 * rateValue(cs.c.rate);
        std::printf("%-22s %7.1f %7.2f |", cs.name, info_bits / air, air);
        for (int v : ok) std::printf(" %5d", v);
        std::printf("\n");
        std::fflush(stdout);
    }
    return 0;
}
