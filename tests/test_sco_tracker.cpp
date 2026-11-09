// ScoTracker: the session clock-drift model behind in-burst member slicing
// (BUG-SCO-NOT-TRACKED-IN-BURST).
//
//  1. a constant clock offset (100 ppm, LTS-like noise) is found and corrected from group 2 on;
//  2. a faded member's wild estimate does not move a group's line (Theil-Sen);
//  3. fading-like trends that change sign from group to group are NEVER corrected — the
//     §14.25 hazard (per-frame re-slicing on fading lost ~50 % of bursts);
//  4. a 0-ppm link with timing noise stays uncorrected;
//  5. implausible lines are rejected, too few points or groups never correct, reset clears.
#include "sync/sco_tracker.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using ultra::sync::ScoTracker;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) {
    std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok) ++g_fail;
}

constexpr double kStride = 57600.0;  // ~1.2 s data frame at 48 kHz

// One group of n members: offset = c + slope * dt + noise.
void feedGroup(ScoTracker& t, int n, double slope, double c, double noise_sd, std::mt19937& rng,
               int outlier_at = -1, double outlier = 0.0) {
    std::normal_distribution<double> g(0.0, noise_sd);
    t.beginGroup();
    for (int k = 0; k < n; ++k) {
        const double dt = k * kStride;
        double off = c + slope * dt + (noise_sd > 0.0 ? g(rng) : 0.0);
        if (k == outlier_at) off += outlier;
        t.addFrame(dt, off);
    }
}
}  // namespace

int main() {
    std::mt19937 rng(5);

    // 1. Constant 100 ppm (receiver hears the sender's clock fast: offsets fall ~5.76 per frame).
    {
        ScoTracker t;
        feedGroup(t, 8, -100e-6, 3.0, 0.3, rng);
        t.endGroup();
        check(!t.significant() && t.correctionSamples(8 * kStride) == 0,
              "100 ppm: one group alone never corrects (needs the between-group check)");
        feedGroup(t, 8, -100e-6, 2.5, 0.3, rng);
        t.endGroup();
        const long corr = t.correctionSamples(7 * kStride);
        char msg[160];
        std::snprintf(msg, sizeof(msg),
                      "100 ppm: corrected from group 2 on (slope %.2f ppm, member 8 shifted %ld samples, expect -40)",
                      t.slope() * 1e6, corr);
        check(t.significant() && std::abs(t.slope() * 1e6 + 100.0) < 2.0 && std::abs(corr + 40) <= 1, msg);
    }

    // 2. One faded member 40 samples off does not tilt the group's line.
    {
        const auto clean = [&] {
            std::vector<ScoTracker::Point> p;
            for (int k = 0; k < 8; ++k) p.push_back({k * kStride, 3.0 - 50e-6 * k * kStride});
            return p;
        }();
        auto faded = clean;
        faded[5].offset += 40.0;
        const auto a = ScoTracker::fit(clean);
        const auto b = ScoTracker::fit(faded);
        char msg[160];
        std::snprintf(msg, sizeof(msg), "faded member: group slope %.2f ppm vs %.2f clean (Theil-Sen)",
                      b.slope * 1e6, a.slope * 1e6);
        check(a.valid && b.valid && std::abs(a.slope - b.slope) < 1e-6, msg);
    }

    // 3. Fading-like trends: a clean +-60 ppm line in every group, sign alternating.
    {
        ScoTracker t;
        bool ever = false;
        for (int g = 0; g < 8; ++g) {
            feedGroup(t, 8, (g % 2 == 0 ? 60e-6 : -60e-6), 0.0, 0.5, rng);
            t.endGroup();
            ever = ever || t.correctionSamples(7 * kStride) != 0;
        }
        check(!ever, "fading trends that flip sign between groups are never corrected (§14.25)");
    }

    // 4. 0 ppm with timing noise: no correction across 8 groups x 20 seeds.
    {
        int corrections = 0, decisions = 0;
        for (int seed = 0; seed < 20; ++seed) {
            std::mt19937 r(100 + seed);
            ScoTracker t;
            for (int g = 0; g < 8; ++g) {
                feedGroup(t, 8, 0.0, 2.0, 3.0, r);
                t.endGroup();
                ++decisions;
                corrections += t.correctionSamples(7 * kStride) != 0;
            }
        }
        char msg[160];
        std::snprintf(msg, sizeof(msg), "0 ppm, 3-sample timing noise: %d of %d decisions corrected (expect ~0)",
                      corrections, decisions);
        check(corrections <= 2, msg);
    }

    // 5. Guards.
    {
        std::vector<ScoTracker::Point> steep;
        for (int k = 0; k < 6; ++k) steep.push_back({k * kStride, -900e-6 * k * kStride});
        check(!ScoTracker::fit(steep).valid, "a 900 ppm line is rejected as implausible");
        std::vector<ScoTracker::Point> two = {{0.0, 1.0}, {kStride, -4.0}};
        check(!ScoTracker::fit(two).valid, "two points never make a fit");
        ScoTracker t;
        feedGroup(t, 8, -100e-6, 0.0, 0.2, rng);
        feedGroup(t, 8, -100e-6, 0.0, 0.2, rng);
        t.endGroup();
        check(t.groups() == 2 && t.significant(), "two consistent groups fold and become significant");
        t.reset();
        check(t.groups() == 0 && t.folds() == 0 && t.correctionSamples(7 * kStride) == 0,
              "reset clears the session model");
    }

    if (g_fail == 0) {
        std::printf("PASS: SCO tracker\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
