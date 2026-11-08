// Frame-level turbo code — see turbo_codec.hpp for the design and its rationale.

#include "turbo_codec.hpp"

#include "crc32c.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace ultra {
namespace fec {
namespace turbo {

namespace {

// Domain tags keep the three generators independent of each other.
constexpr uint64_t kDomainInterleaver = 0x5455524250493031ULL;  // "TURBPI01"
constexpr uint64_t kDomainChannel = 0x5455524243483031ULL;      // "TURBCH01"
constexpr uint64_t kDomainWhitening = 0x5455524257483031ULL;    // "TURBWH01"

constexpr float kNeg = -1.0e30f;
constexpr int kSRandomAttempts = 16;

// ---------------------------------------------------------------------------
// Constituent RSC trellis. State s = (s1 << 2) | (s2 << 1) | s3, s1 = newest.
//   feedback a = u ^ s2 ^ s3          (1 + D^2 + D^3, 13 octal)
//   parity   z = a ^ s1 ^ s3          (1 + D + D^3,   15 octal)
//   next       = (a, s1, s2)
// Termination input u = s2 ^ s3 forces a = 0 (state walks to 0 in 3 steps).
struct Trellis {
    uint8_t next[8][2];
    uint8_t par[8][2];
    uint8_t tail_u[8];
};

constexpr Trellis makeTrellis() {
    Trellis t{};
    for (int s = 0; s < 8; ++s) {
        const int s1 = (s >> 2) & 1, s2 = (s >> 1) & 1, s3 = s & 1;
        for (int u = 0; u < 2; ++u) {
            const int a = u ^ s2 ^ s3;
            t.par[s][u] = static_cast<uint8_t>(a ^ s1 ^ s3);
            t.next[s][u] = static_cast<uint8_t>((a << 2) | (s1 << 1) | s2);
        }
        t.tail_u[s] = static_cast<uint8_t>(s2 ^ s3);
    }
    return t;
}
constexpr Trellis kTrellis = makeTrellis();

uint64_t seedFor(uint64_t domain, uint64_t a, uint64_t b, uint64_t c) {
    uint64_t st = domain ^ (a * 0x9E3779B97F4A7C15ULL) ^ (b << 32) ^ (c << 48) ^ (c * 0xD1B54A32D192ED03ULL);
    (void)splitmix64Next(st);
    return st;
}

// Unbiased integer in [0, n) by rejection — integer-only, platform-identical.
uint32_t below(uint64_t& st, uint32_t n) {
    const uint64_t limit = UINT64_MAX - (UINT64_MAX % n);
    uint64_t x;
    do {
        x = splitmix64Next(st);
    } while (x >= limit);
    return static_cast<uint32_t>(x % n);
}

int isqrt(int v) {
    int r = static_cast<int>(std::sqrt(static_cast<double>(v)));
    while (r > 0 && r * r > v) --r;
    while ((r + 1) * (r + 1) <= v) ++r;
    return r;
}

// S-random interleaver: |pi(i) - pi(j)| >= S whenever 0 < i - j < S. Greedy over a
// shuffled pool with a forbidden-count array (O(K*S) per attempt). S0 = isqrt(K/2) is
// the covering bound (the S-1 window entries forbid ~2S^2 >= K values), so a greedy
// construction only completes near ~0.5*S0 (measured 0.45-0.65*S0 for K = 352..7712);
// starting at 0.6*S0 keeps generation fast (it runs at a mid-transfer rung switch on a
// Pi) without losing spread. Spread sets the error floor, which sits orders of
// magnitude below the FER at which ARQ operates.
std::vector<uint32_t> buildSRandom(int K, int& achieved) {
    const int S0 = std::max(1, isqrt(K / 2));
    for (int S = std::max(1, (S0 * 3) / 5); S >= 1; --S) {
        for (int attempt = 0; attempt < kSRandomAttempts; ++attempt) {
            uint64_t st = seedFor(kDomainInterleaver, static_cast<uint64_t>(K),
                                  static_cast<uint64_t>(S), static_cast<uint64_t>(attempt));
            std::vector<uint32_t> pool(static_cast<size_t>(K));
            std::iota(pool.begin(), pool.end(), 0u);
            for (int i = K - 1; i > 0; --i) {
                std::swap(pool[static_cast<size_t>(i)],
                          pool[below(st, static_cast<uint32_t>(i + 1))]);
            }
            std::vector<uint32_t> pi;
            pi.reserve(static_cast<size_t>(K));
            std::vector<int> forbid(static_cast<size_t>(K), 0);
            auto mark = [&](uint32_t v, int delta) {
                const int lo = std::max(0, static_cast<int>(v) - S + 1);
                const int hi = std::min(K - 1, static_cast<int>(v) + S - 1);
                for (int w = lo; w <= hi; ++w) forbid[static_cast<size_t>(w)] += delta;
            };
            bool ok = true;
            for (int i = 0; i < K; ++i) {
                if (i - S >= 0) mark(pi[static_cast<size_t>(i - S)], -1);
                size_t j = 0;
                while (j < pool.size() && forbid[pool[j]] > 0) ++j;
                if (j == pool.size()) {
                    ok = false;
                    break;
                }
                const uint32_t v = pool[j];
                pool[j] = pool.back();
                pool.pop_back();
                pi.push_back(v);
                mark(v, +1);
            }
            if (ok) {
                achieved = S;
                return pi;
            }
        }
    }
    achieved = 0;
    std::vector<uint32_t> id(static_cast<size_t>(K));
    std::iota(id.begin(), id.end(), 0u);
    return id;  // unreachable: S = 1 always succeeds
}

// Bresenham selection: keep(t) in {0,1}, sum over t in [0,K) == E (E <= K).
inline bool keepBit(int64_t t, int64_t E, int64_t phi, int64_t K) {
    return ((t + 1) * E + phi) / K - (t * E + phi) / K != 0;
}

uint64_t fnv1a(uint64_t h, uint32_t v) {
    for (int b = 0; b < 4; ++b) {
        h ^= (v >> (8 * b)) & 0xFFu;
        h *= 0x100000001B3ULL;
    }
    return h;
}

std::unique_ptr<Tables> buildTables(int K, int N) {
    if (K <= 0 || N <= 0 || (K % 8) != 0 || (N % 8) != 0) {
        throw std::invalid_argument("turbo: K and N must be positive multiples of 8");
    }
    const int E = N - K - kTailSlots;
    if (E < 0) {
        throw std::invalid_argument("turbo: N < K + 12 (rate above 1)");
    }
    auto t = std::make_unique<Tables>();
    t->K = K;
    t->N = N;
    t->pi = buildSRandom(K, t->spread);

    int R = 0;
    if (E <= 2 * K) {
        t->E1 = (E + 1) / 2;
        t->E2 = E / 2;
    } else {
        t->E1 = K;
        t->E2 = K;
        R = E - 2 * K;
        if (R > 3 * K) {
            throw std::invalid_argument("turbo: rate too low for systematic+parity repetition");
        }
    }
    t->R = R;

    // Coded order: S | P1 kept | P2 kept | tails | repeats -> decoder slot per coded bit.
    std::vector<uint32_t> coded_slot;
    coded_slot.reserve(static_cast<size_t>(N));
    for (int i = 0; i < K; ++i) coded_slot.push_back(static_cast<uint32_t>(i));
    for (int i = 0; i < K; ++i) {
        if (keepBit(i, t->E1, 0, K)) coded_slot.push_back(static_cast<uint32_t>(K + i));
    }
    for (int i = 0; i < K; ++i) {
        if (keepBit(i, t->E2, K / 2, K)) coded_slot.push_back(static_cast<uint32_t>(2 * K + i));
    }
    for (int i = 0; i < kTailSlots; ++i) coded_slot.push_back(static_cast<uint32_t>(3 * K + i));
    // Repeats: systematic first (they feed both decoders), cycling into parity.
    const int r_sys = std::min(R, K);
    const int r_p1 = std::min(std::max(R - K, 0), K);
    const int r_p2 = std::min(std::max(R - 2 * K, 0), K);
    for (int i = 0; i < K; ++i) {
        if (r_sys > 0 && keepBit(i, r_sys, K / 4, K)) coded_slot.push_back(static_cast<uint32_t>(i));
    }
    for (int i = 0; i < K; ++i) {
        if (r_p1 > 0 && keepBit(i, r_p1, K / 4, K)) coded_slot.push_back(static_cast<uint32_t>(K + i));
    }
    for (int i = 0; i < K; ++i) {
        if (r_p2 > 0 && keepBit(i, r_p2, K / 4, K)) coded_slot.push_back(static_cast<uint32_t>(2 * K + i));
    }
    if (static_cast<int>(coded_slot.size()) != N) {
        throw std::logic_error("turbo: rate matcher produced " + std::to_string(coded_slot.size()) +
                               " bits, expected " + std::to_string(N));
    }

    // Frame-wide random channel permutation: coded position c -> air position chan[c].
    std::vector<uint32_t> chan(static_cast<size_t>(N));
    std::iota(chan.begin(), chan.end(), 0u);
    uint64_t st = seedFor(kDomainChannel, static_cast<uint64_t>(N), 0, 0);
    for (int i = N - 1; i > 0; --i) {
        std::swap(chan[static_cast<size_t>(i)], chan[below(st, static_cast<uint32_t>(i + 1))]);
    }
    t->air_slot.assign(static_cast<size_t>(N), 0u);
    for (int c = 0; c < N; ++c) t->air_slot[chan[static_cast<size_t>(c)]] = coded_slot[static_cast<size_t>(c)];

    uint64_t h = 0xCBF29CE484222325ULL;
    for (uint32_t v : t->pi) h = fnv1a(h, v);
    for (uint32_t v : t->air_slot) h = fnv1a(h, v);
    t->digest = h;
    return t;
}

void rscEncode(const std::vector<uint8_t>& bits, const std::vector<uint32_t>* perm,
               std::vector<uint8_t>& par, uint8_t tail_x[3], uint8_t tail_z[3]) {
    const size_t K = bits.size();
    par.resize(K);
    int s = 0;
    for (size_t i = 0; i < K; ++i) {
        const int u = perm ? bits[(*perm)[i]] : bits[i];
        par[i] = kTrellis.par[s][u];
        s = kTrellis.next[s][u];
    }
    for (int i = 0; i < 3; ++i) {
        const int u = kTrellis.tail_u[s];
        tail_x[i] = static_cast<uint8_t>(u);
        tail_z[i] = kTrellis.par[s][u];
        s = kTrellis.next[s][u];
    }
}

// Max-log-MAP SISO over K info steps + 3 terminating steps.
// sys/par hold K+3 LLRs (positive = 0). Writes the a-posteriori LLR of each info bit.
void siso(const float* sys, const float* par, int K, float* app, std::vector<float>& alpha) {
    const int T = K + 3;
    alpha.assign(static_cast<size_t>(T + 1) * 8, kNeg);
    alpha[0] = 0.0f;
    for (int t = 0; t < T; ++t) {
        const float* a = &alpha[static_cast<size_t>(t) * 8];
        float* an = &alpha[static_cast<size_t>(t + 1) * 8];
        const float hs = 0.5f * sys[t], hp = 0.5f * par[t];
        const bool tail = t >= K;
        for (int s = 0; s < 8; ++s) {
            if (a[s] <= kNeg * 0.5f) continue;
            for (int u = 0; u < 2; ++u) {
                if (tail && u != kTrellis.tail_u[s]) continue;
                const float g = (u ? -hs : hs) + (kTrellis.par[s][u] ? -hp : hp);
                const int ns = kTrellis.next[s][u];
                an[ns] = std::max(an[ns], a[s] + g);
            }
        }
        float m = kNeg;
        for (int s = 0; s < 8; ++s) m = std::max(m, an[s]);
        if (m > kNeg * 0.5f) {
            for (int s = 0; s < 8; ++s) {
                if (an[s] > kNeg * 0.5f) an[s] -= m;
            }
        }
    }
    float beta[8], bnext[8];
    for (int s = 0; s < 8; ++s) bnext[s] = kNeg;
    bnext[0] = 0.0f;
    for (int t = T - 1; t >= 0; --t) {
        const float* a = &alpha[static_cast<size_t>(t) * 8];
        const float hs = 0.5f * sys[t], hp = 0.5f * par[t];
        const bool tail = t >= K;
        float m0 = kNeg, m1 = kNeg;
        for (int s = 0; s < 8; ++s) {
            float b = kNeg;
            for (int u = 0; u < 2; ++u) {
                if (tail && u != kTrellis.tail_u[s]) continue;
                const float g = (u ? -hs : hs) + (kTrellis.par[s][u] ? -hp : hp);
                const float bn = bnext[kTrellis.next[s][u]];
                if (bn <= kNeg * 0.5f) continue;
                b = std::max(b, g + bn);
                if (!tail && a[s] > kNeg * 0.5f) {
                    const float metric = a[s] + g + bn;
                    if (u == 0) m0 = std::max(m0, metric);
                    else m1 = std::max(m1, metric);
                }
            }
            beta[s] = b;
        }
        if (!tail) app[t] = m0 - m1;
        float m = kNeg;
        for (int s = 0; s < 8; ++s) m = std::max(m, beta[s]);
        for (int s = 0; s < 8; ++s) {
            bnext[s] = (beta[s] > kNeg * 0.5f && m > kNeg * 0.5f) ? beta[s] - m : kNeg;
        }
    }
}

std::mutex g_tables_mu;
std::map<std::pair<int, int>, std::unique_ptr<Tables>>& tableCache() {
    static std::map<std::pair<int, int>, std::unique_ptr<Tables>> cache;
    return cache;
}

}  // namespace

uint64_t splitmix64Next(uint64_t& state) {
    uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

std::vector<uint8_t> whitening(size_t nbytes) {
    std::vector<uint8_t> w(nbytes);
    uint64_t st = kDomainWhitening;
    for (size_t i = 0; i < nbytes; i += 8) {
        const uint64_t x = splitmix64Next(st);
        for (size_t b = 0; b < 8 && i + b < nbytes; ++b) {
            w[i + b] = static_cast<uint8_t>((x >> (8 * b)) & 0xFFu);
        }
    }
    return w;
}

const Tables& tables(int K, int N) {
    std::lock_guard<std::mutex> lock(g_tables_mu);
    auto& cache = tableCache();
    auto it = cache.find({K, N});
    if (it != cache.end()) return *it->second;
    auto built = buildTables(K, N);
    const Tables& ref = *built;
    cache.emplace(std::make_pair(K, N), std::move(built));
    return ref;
}

std::vector<uint8_t> infoBits(const Bytes& info_bytes) {
    const auto w = whitening(info_bytes.size());
    Bytes s(info_bytes.size());
    for (size_t i = 0; i < s.size(); ++i) s[i] = static_cast<uint8_t>(info_bytes[i] ^ w[i]);
    const uint32_t crc = crc32c(s.data(), s.size());
    s.push_back(static_cast<uint8_t>(crc >> 24));
    s.push_back(static_cast<uint8_t>(crc >> 16));
    s.push_back(static_cast<uint8_t>(crc >> 8));
    s.push_back(static_cast<uint8_t>(crc));
    std::vector<uint8_t> bits(s.size() * 8);
    for (size_t i = 0; i < s.size(); ++i) {
        for (int b = 0; b < 8; ++b) bits[i * 8 + static_cast<size_t>(b)] = (s[i] >> (7 - b)) & 1u;
    }
    return bits;
}

Bytes encode(const Bytes& info_bytes, int air_bits) {
    const int K = infoBitsForBytes(info_bytes.size());
    const Tables& tb = tables(K, air_bits);
    const std::vector<uint8_t> b = infoBits(info_bytes);
    std::vector<uint8_t> p1, p2;
    uint8_t x1[3], z1[3], x2[3], z2[3];
    rscEncode(b, nullptr, p1, x1, z1);
    rscEncode(b, &tb.pi, p2, x2, z2);
    uint8_t tails[kTailSlots];
    for (int i = 0; i < 3; ++i) {
        tails[2 * i] = x1[i];
        tails[2 * i + 1] = z1[i];
        tails[6 + 2 * i] = x2[i];
        tails[6 + 2 * i + 1] = z2[i];
    }
    Bytes out(static_cast<size_t>(air_bits / 8), 0);
    for (int a = 0; a < air_bits; ++a) {
        const uint32_t slot = tb.air_slot[static_cast<size_t>(a)];
        uint8_t bit;
        if (slot < static_cast<uint32_t>(K)) bit = b[slot];
        else if (slot < static_cast<uint32_t>(2 * K)) bit = p1[slot - K];
        else if (slot < static_cast<uint32_t>(3 * K)) bit = p2[slot - 2 * K];
        else bit = tails[slot - 3 * K];
        if (bit) out[static_cast<size_t>(a / 8)] |= static_cast<uint8_t>(0x80u >> (a % 8));
    }
    return out;
}

DecodeResult decode(std::span<const float> air_llrs, size_t info_bytes,
                    const DecodeOptions& opts, float clip_level) {
    DecodeResult res;
    res.info_bytes.assign(info_bytes, 0);
    const int K = infoBitsForBytes(info_bytes);
    const int N = static_cast<int>(air_llrs.size());
    const Tables& tb = tables(K, N);

    thread_local std::vector<float> L, Ls, sys1, par1, sys2, par2, La1, Le1, app1, app2, alpha;
    thread_local std::vector<uint8_t> hard, prev_hard;
    L.assign(static_cast<size_t>(3 * K + kTailSlots), 0.0f);
    double abs_sum = 0.0;
    size_t clipped = 0;
    for (int a = 0; a < N; ++a) {
        float v = air_llrs[static_cast<size_t>(a)];
        if (!std::isfinite(v)) v = 0.0f;
        abs_sum += std::abs(v);
        if (std::abs(v) >= clip_level - 1.0e-3f) ++clipped;
        L[tb.air_slot[static_cast<size_t>(a)]] += v;
    }
    res.mean_abs_llr = N > 0 ? static_cast<float>(abs_sum / N) : 0.0f;
    res.clip_fraction = N > 0 ? static_cast<float>(clipped) / static_cast<float>(N) : 0.0f;
    if (!(res.mean_abs_llr >= 1.0e-3f)) {
        res.stop = Stop::NoEnergy;
        return res;
    }

    const size_t T = static_cast<size_t>(K) + 3;
    Ls.assign(L.begin(), L.begin() + K);
    sys1.assign(T, 0.0f);
    par1.assign(T, 0.0f);
    sys2.assign(T, 0.0f);
    par2.assign(T, 0.0f);
    for (int i = 0; i < K; ++i) {
        par1[static_cast<size_t>(i)] = L[static_cast<size_t>(K + i)];
        par2[static_cast<size_t>(i)] = L[static_cast<size_t>(2 * K + i)];
    }
    for (int i = 0; i < 3; ++i) {
        sys1[static_cast<size_t>(K + i)] = L[static_cast<size_t>(3 * K + 2 * i)];
        par1[static_cast<size_t>(K + i)] = L[static_cast<size_t>(3 * K + 2 * i + 1)];
        sys2[static_cast<size_t>(K + i)] = L[static_cast<size_t>(3 * K + 6 + 2 * i)];
        par2[static_cast<size_t>(K + i)] = L[static_cast<size_t>(3 * K + 6 + 2 * i + 1)];
    }
    La1.assign(static_cast<size_t>(K), 0.0f);
    Le1.assign(static_cast<size_t>(K), 0.0f);
    app1.assign(static_cast<size_t>(K), 0.0f);
    app2.assign(static_cast<size_t>(K), 0.0f);
    hard.assign(static_cast<size_t>(K), 0);
    prev_hard.assign(static_cast<size_t>(K), 2);

    const auto w = whitening(info_bytes);
    Bytes bytes(static_cast<size_t>(K / 8));
    int still = 0;
    const float scale = opts.extrinsic_scale;
    for (int it = 1; it <= std::max(1, opts.max_iterations); ++it) {
        for (int i = 0; i < K; ++i) sys1[static_cast<size_t>(i)] = Ls[static_cast<size_t>(i)] + La1[static_cast<size_t>(i)];
        siso(sys1.data(), par1.data(), K, app1.data(), alpha);
        for (int i = 0; i < K; ++i) {
            Le1[static_cast<size_t>(i)] = scale * (app1[static_cast<size_t>(i)] - sys1[static_cast<size_t>(i)]);
        }
        for (int i = 0; i < K; ++i) {
            const uint32_t j = tb.pi[static_cast<size_t>(i)];
            sys2[static_cast<size_t>(i)] = Ls[j] + Le1[j];
        }
        siso(sys2.data(), par2.data(), K, app2.data(), alpha);
        int flips = 0;
        int agree = 0, voted = 0;
        for (int i = 0; i < K; ++i) {
            const uint32_t j = tb.pi[static_cast<size_t>(i)];
            La1[j] = scale * (app2[static_cast<size_t>(i)] - sys2[static_cast<size_t>(i)]);
            if (La1[j] != 0.0f && Le1[j] != 0.0f) {
                ++voted;
                agree += ((La1[j] > 0.0f) == (Le1[j] > 0.0f)) ? 1 : 0;
            }
            const uint8_t h = app2[static_cast<size_t>(i)] < 0.0f ? 1 : 0;
            if (h != prev_hard[j]) ++flips;
            hard[j] = h;
        }
        res.iterations = it;
        res.extrinsic_agreement = voted > 0 ? static_cast<float>(agree) / static_cast<float>(voted) : 0.5f;
        std::fill(bytes.begin(), bytes.end(), 0);
        for (int i = 0; i < K; ++i) {
            if (hard[static_cast<size_t>(i)]) bytes[static_cast<size_t>(i / 8)] |= static_cast<uint8_t>(0x80u >> (i % 8));
        }
        const size_t nb = bytes.size() - kCrcBytes;
        const uint32_t got = (static_cast<uint32_t>(bytes[nb]) << 24) | (static_cast<uint32_t>(bytes[nb + 1]) << 16) |
                             (static_cast<uint32_t>(bytes[nb + 2]) << 8) | static_cast<uint32_t>(bytes[nb + 3]);
        if (crc32c(bytes.data(), nb) == got) {
            res.crc_ok = true;
            res.stop = Stop::Crc;
            break;
        }
        still = (flips == 0) ? still + 1 : 0;
        if (opts.early_fail && it >= 3 && still >= 2) {
            res.stop = Stop::Stuck;
            break;
        }
        prev_hard = hard;
        res.stop = Stop::MaxIterations;
    }
    for (size_t i = 0; i < info_bytes; ++i) res.info_bytes[i] = static_cast<uint8_t>(bytes[i] ^ w[i]);
    return res;
}

}  // namespace turbo
}  // namespace fec
}  // namespace ultra
