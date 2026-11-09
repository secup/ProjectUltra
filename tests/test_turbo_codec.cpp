// Frame-level turbo code (src/fec/turbo_codec.hpp): structural invariants, noiseless
// round trips at every production frame geometry, AWGN performance bounds, erasure
// robustness, CRC traps and pinned table digests (Mac/Pi determinism).
#include "fec/crc32c.hpp"
#include "fec/turbo_codec.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace ultra;
namespace tc = ultra::fec::turbo;

namespace {
int g_fail = 0;
void check(bool ok, const char* msg) {
    std::printf("%s %s\n", ok ? "ok:  " : "FAIL:", msg);
    if (!ok) ++g_fail;
}

struct Geo {
    const char* name;
    int cw;
    int bytes_per_cw;  // LDPC Z=27 info bytes per codeword at this rate
};
// Production rungs (connection_policy::recommendCWCount) at Z=27.
const Geo kGeos[] = {
    {"R1/4 cw4", 4, 20},  {"R1/2 cw8", 8, 40},   {"R2/3 cw8", 8, 54},   {"R3/4 cw8", 8, 60},
    {"R5/6 cw8", 8, 67},  {"R2/3 cw12", 12, 54}, {"R2/3 cw16", 16, 54}, {"R3/4 cw16", 16, 60},
    {"R1/2 cw1", 1, 40},  {"R3/4 cw2", 2, 60},
};

Bytes randomBytes(std::mt19937_64& rng, size_t n) {
    Bytes b(n);
    for (auto& v : b) v = static_cast<uint8_t>(rng() & 0xFF);
    return b;
}

std::vector<float> bpskLlrs(const Bytes& air, float esn0_lin, std::mt19937_64& rng, bool noisy) {
    const size_t N = air.size() * 8;
    std::vector<float> llr(N);
    const double sigma2 = 1.0 / (2.0 * esn0_lin);  // per real dimension, unit-energy BPSK
    std::normal_distribution<double> nd(0.0, std::sqrt(sigma2));
    for (size_t a = 0; a < N; ++a) {
        const int bit = (air[a / 8] >> (7 - (a % 8))) & 1;
        const double x = bit ? -1.0 : 1.0;
        const double y = noisy ? x + nd(rng) : x;
        llr[a] = static_cast<float>(std::clamp(2.0 * y / sigma2, -20.0, 20.0));
    }
    return llr;
}

int countFailures(int cw, int bpc, double ebn0_db, int frames, uint64_t seed) {
    std::mt19937_64 rng(seed);
    const size_t info = static_cast<size_t>(cw * bpc);
    const int N = cw * 648;
    const double rate = static_cast<double>(tc::infoBitsForBytes(info)) / N;
    const double esn0 = std::pow(10.0, ebn0_db / 10.0) * rate;
    int fails = 0;
    for (int f = 0; f < frames; ++f) {
        const Bytes payload = randomBytes(rng, info);
        const Bytes air = tc::encode(payload, N);
        const auto llr = bpskLlrs(air, static_cast<float>(esn0), rng, true);
        const auto r = tc::decode(llr, info);
        if (!r.crc_ok || r.info_bytes != payload) ++fails;
    }
    return fails;
}
}  // namespace

// The 250 ms generation bound is a property of the OPTIMIZED generator: the tables are built
// on first use, at a mid-transfer rung switch, by the shipped (optimized) binary. Unoptimized,
// sanitizer and coverage builds are 10-40x slower and only print the time (same tables: the
// digests below are checked in every build). Measured: 16.7 ms optimized on an M-series Mac,
// 535 ms in the CI Debug build, K = 7712.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(memory_sanitizer)
#define ULTRA_TEST_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define ULTRA_TEST_SANITIZED 1
#endif
#if (defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))) && \
    !defined(ULTRA_TEST_SANITIZED) && !defined(ULTRA_COVERAGE_BUILD)
constexpr bool kTimingBoundsApply = true;
#else
constexpr bool kTimingBoundsApply = false;
#endif

int main() {
    // CRC-32C check value.
    {
        const char* s = "123456789";
        check(fec::crc32c(reinterpret_cast<const uint8_t*>(s), 9) == 0xE3069283u, "crc32c check value");
    }

    // Tables: permutation, spread, rate-matching accounting, generation time.
    for (const auto& g : kGeos) {
        const int K = tc::infoBitsForBytes(static_cast<size_t>(g.cw * g.bytes_per_cw));
        const int N = g.cw * 648;
        const auto t0 = std::chrono::steady_clock::now();
        const auto& tb = tc::tables(K, N);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const int S0 = static_cast<int>(std::floor(std::sqrt(K / 2.0)));
        std::printf("      %-9s K=%5d N=%5d S=%d (S0=%d) E1=%d E2=%d R=%d gen=%.1f ms digest=%016llx\n", g.name, K, N,
                    tb.spread, S0, tb.E1, tb.E2, tb.R, ms, static_cast<unsigned long long>(tb.digest));
        std::vector<int> seen(static_cast<size_t>(K), 0);
        bool perm = tb.pi.size() == static_cast<size_t>(K);
        for (uint32_t v : tb.pi) {
            if (v >= static_cast<uint32_t>(K) || seen[v]++) perm = false;
        }
        char msg[128];
        std::snprintf(msg, sizeof(msg), "%s: interleaver is a permutation", g.name);
        check(perm, msg);
        bool spread_ok = true;
        for (int i = 0; i < K && spread_ok; ++i) {
            for (int d = 1; d < tb.spread && i - d >= 0; ++d) {
                if (std::abs(static_cast<int>(tb.pi[i]) - static_cast<int>(tb.pi[i - d])) < tb.spread) {
                    spread_ok = false;
                    break;
                }
            }
        }
        std::snprintf(msg, sizeof(msg), "%s: S-random spread holds and S >= 0.4*S0", g.name);
        check(spread_ok && tb.spread >= static_cast<int>(0.4 * S0), msg);
        std::vector<int> slot_count(static_cast<size_t>(3 * K + tc::kTailSlots), 0);
        for (uint32_t s : tb.air_slot) slot_count[s]++;
        int sys_missing = 0, p1 = 0, p2 = 0, tails_ok = 1, total = 0;
        for (int i = 0; i < K; ++i) sys_missing += slot_count[static_cast<size_t>(i)] == 0;
        for (int i = 0; i < K; ++i) p1 += slot_count[static_cast<size_t>(K + i)];
        for (int i = 0; i < K; ++i) p2 += slot_count[static_cast<size_t>(2 * K + i)];
        for (int i = 0; i < tc::kTailSlots; ++i) tails_ok &= slot_count[static_cast<size_t>(3 * K + i)] == 1;
        for (int c : slot_count) total += c;
        std::snprintf(msg, sizeof(msg), "%s: every systematic bit sent, tails once, air count exact", g.name);
        check(sys_missing == 0 && tails_ok && total == N, msg);
        std::snprintf(msg, sizeof(msg), "%s: kept parity matches E1/E2 (+repeats)", g.name);
        check((tb.R == 0 && p1 == tb.E1 && p2 == tb.E2) || (tb.R > 0 && p1 >= tb.E1 && p2 >= tb.E2), msg);
        if (kTimingBoundsApply) {
            std::snprintf(msg, sizeof(msg), "%s: table generation < 250 ms on this host", g.name);
            check(ms < 250.0, msg);
        }
    }

    // Pinned digests: the code is a WIRE contract between two stations that may run on
    // different platforms (macOS libc++ / Linux libstdc++). Any change to the
    // generators, seeds, interleaver or rate matcher breaks interoperability and must
    // fail here first. Update only together with a deliberate wire change.
    check(tc::tables(2592, 5184).digest == 0xf516fdbfdf0b4bdbULL, "pinned digest K=2592 N=5184 (QPSK R1/2 cw8)");
    check(tc::tables(5216, 7776).digest == 0x5ba9642070558b6cULL, "pinned digest K=5216 N=7776 (8PSK R2/3 cw12)");

    // Noiseless round trips at every geometry: one iteration, exact payload.
    {
        std::mt19937_64 rng(7);
        for (const auto& g : kGeos) {
            const size_t info = static_cast<size_t>(g.cw * g.bytes_per_cw);
            const int N = g.cw * 648;
            const Bytes payload = randomBytes(rng, info);
            const Bytes air = tc::encode(payload, N);
            const auto llr = bpskLlrs(air, 100.0f, rng, false);
            const auto r = tc::decode(llr, info);
            char msg[96];
            std::snprintf(msg, sizeof(msg), "%s: noiseless round trip in 1 iteration", g.name);
            check(air.size() * 8 == static_cast<size_t>(N) && r.crc_ok && r.iterations == 1 && r.info_bytes == payload,
                  msg);
        }
    }

    // CRC traps: an all-zero LLR frame and the all-zero codeword must never pass.
    {
        std::vector<float> zero(5184, 0.0f);
        const auto r0 = tc::decode(zero, 320);
        check(!r0.crc_ok && r0.stop == tc::Stop::NoEnergy, "all-zero LLRs => NoEnergy, no CRC pass");
        std::vector<float> all0(5184, 10.0f);  // every air bit decided 0
        const auto r1 = tc::decode(all0, 320);
        check(!r1.crc_ok, "all-zero codeword (valid PCCC word) fails CRC-32C");
    }

    // AWGN/BPSK performance bounds (deliberately generous vs known turbo performance;
    // capacity for R1/2 BPSK is Eb/N0 = 0.19 dB).
    {
        const int f20 = countFailures(8, 40, 2.0, 60, 11);
        const int f00 = countFailures(8, 40, 0.0, 30, 12);
        const int f38 = countFailures(8, 60, 3.8, 60, 13);
        std::printf("      R1/2 cw8: FER %.1f%% @2.0 dB, %.1f%% @0.0 dB; R3/4 cw8: FER %.1f%% @3.8 dB\n",
                    100.0 * f20 / 60, 100.0 * f00 / 30, 100.0 * f38 / 60);
        check(f20 <= 6, "R1/2 cw8 (K=2592): FER <= 10% at Eb/N0 2.0 dB");
        check(f00 >= 24, "R1/2 cw8: FER >= 80% at Eb/N0 0.0 dB (no false successes near capacity)");
        check(f38 <= 6, "R3/4 cw8 (K=3872): FER <= 10% at Eb/N0 3.8 dB");
    }

    // Erasure robustness: 20% contiguous air positions erased at high SNR, R1/2 decodes.
    {
        std::mt19937_64 rng(21);
        const Bytes payload = randomBytes(rng, 320);
        const Bytes air = tc::encode(payload, 5184);
        auto llr = bpskLlrs(air, 10.0f, rng, true);
        for (size_t a = 1000; a < 1000 + 5184 / 5; ++a) llr[a] = 0.0f;
        const auto r = tc::decode(llr, 320);
        check(r.crc_ok && r.info_bytes == payload, "R1/2: 20% contiguous erasure still decodes");
    }

    // Decode time at the largest production K (informational; Pi 5 budget is 20 ms).
    {
        std::mt19937_64 rng(31);
        const Bytes payload = randomBytes(rng, 960);
        const Bytes air = tc::encode(payload, 10368);
        const auto llr = bpskLlrs(air, 1.0f, rng, true);
        tc::DecodeOptions o;
        o.early_fail = false;
        const auto t0 = std::chrono::steady_clock::now();
        const auto r = tc::decode(llr, 960, o);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        std::printf("      K=7712 decode: %d iterations in %.2f ms (crc_ok=%d)\n", r.iterations, ms, r.crc_ok ? 1 : 0);
    }

    if (g_fail == 0) {
        std::printf("PASS: turbo codec\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
