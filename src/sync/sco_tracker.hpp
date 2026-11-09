#pragma once

// SAMPLE-CLOCK-OFFSET (SCO) TRACKING for in-burst frame slicing (BUG-SCO-NOT-TRACKED-IN-BURST).
//
// Two soundcards never share a clock: 20-50 ppm apart is normal, cheap USB dongles reach ~100.
// A burst group is synchronized once (its group-start frame) and its members are then sliced at
// the fixed frame stride, so the sender's clock error accumulates member after member: at 100 ppm
// a 1.2 s frame slips ~6 samples, the FFT window of the 8th member sits ~40 samples late, and a
// LATE window reads the next symbol (no cyclic-prefix protection on that side). Measured with the
// OTASim clock offset (--clock-ppm -100, AWGN@20): member EVM 17.7 -> 10.8 dB across one 8-frame
// 16QAM R2/3 group.
//
// The slip is a straight line in the members' LTS timing offsets: offset(dt) = c + s * dt, with
// dt the nominal sample distance from the group anchor and s the clock-rate difference. Each group
// gives a Theil-Sen (median pairwise slope) fit, which ignores a faded member's wild estimate. A
// single group's line is NOT trusted, though: on a multipath channel the fading channel's own
// mean delay wanders within a burst and can look like a slope, but that trend changes from group
// to group while a clock offset does not. So the session slope is the mean over recent groups and
// it is applied only when it is significant against the BETWEEN-group spread (or the within-group
// standard error, whichever is larger): |s| > kSignificance * se, with at least two groups. A
// faded frame or a fading trend therefore never re-slices anything — the failure that made
// per-frame timing correction harmful on Good fading (§14.25, May 2026) — while a real clock offset,
// identical in every group, is removed from the second or third group on.
//
// Units: offsets and corrections in samples, positive = move the slice LATER (the convention of
// IWaveform::getLastTimingOffsetSamples(), whose value is ADDED to a slice position); slope in
// samples per sample (== ppm * 1e-6). ULTRA_SCO_TRACK=0 measures but never corrects.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <limits>
#include <vector>

namespace ultra {
namespace sync {

inline bool scoTrackingEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("ULTRA_SCO_TRACK");
        return !(e != nullptr && std::strcmp(e, "0") == 0);
    }();
    return on;
}

class ScoTracker {
public:
    struct Point {
        double dt = 0.0;      // nominal samples from the group anchor
        double offset = 0.0;  // total timing offset of that frame (applied + measured), samples
    };
    struct GroupFit {
        bool valid = false;
        double slope = 0.0;   // samples per sample
        double se = 0.0;      // standard error of the slope
        size_t points = 0;
    };

    // A member residual beyond this is a mislock or a deep fade, not clock drift: three quarters
    // of the 128-sample CP (an uncorrected 100 ppm group legitimately reaches ~60 by its end).
    static constexpr double kMaxPointOffsetSamples = 96.0;
    // No soundcard pair is 500 ppm apart; a group line steeper than that is a misfit.
    static constexpr double kMaxPlausibleSlope = 500e-6;
    static constexpr size_t kMinPoints = 3;
    static constexpr size_t kMinGroups = 2;
    static constexpr size_t kWindowGroups = 8;
    // Three standard errors: a 0-ppm link applies a spurious correction on ~0.3 % of decisions.
    static constexpr double kSignificance = 3.0;
    // Timing precision floor: corrections are whole samples, and a uniform quantizer's error has
    // standard deviation 1/sqrt(12) samples.
    static constexpr double kOffsetSigmaFloor = 0.28867513459481287;

    void reset() {
        points_.clear();
        groups_.clear();
        last_ = {};
        folds_ = 0;
    }

    // Start a new group; the previous group's points are folded into the session estimate.
    void beginGroup() {
        fold();
        points_.clear();
    }

    void addFrame(double dt, double total_offset) {
        if (!std::isfinite(dt) || !std::isfinite(total_offset)) return;
        points_.push_back({dt, total_offset});
    }

    // Fold the current group now (the next beginGroup() then folds nothing).
    void endGroup() {
        fold();
        points_.clear();
    }

    size_t groups() const { return groups_.size(); }
    size_t folds() const { return folds_; }  // groups fitted (valid or rejected) since reset
    const GroupFit& lastFit() const { return last_; }

    double slope() const {
        if (groups_.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& g : groups_) sum += g.slope;
        return sum / static_cast<double>(groups_.size());
    }

    double standardError() const {
        const size_t n = groups_.size();
        if (n < 2) return std::numeric_limits<double>::infinity();
        const double mean = slope();
        double between = 0.0, within = 0.0;
        for (const auto& g : groups_) {
            between += (g.slope - mean) * (g.slope - mean);
            within += g.se * g.se;
        }
        between /= static_cast<double>(n - 1);
        within /= static_cast<double>(n);
        return std::sqrt(std::max(between, within) / static_cast<double>(n));
    }

    bool significant() const {
        return groups_.size() >= kMinGroups && std::abs(slope()) > kSignificance * standardError();
    }

    // Whole-sample correction to ADD to a member slice at nominal distance dt from the anchor.
    long correctionSamples(double dt) const {
        if (!scoTrackingEnabled() || !significant()) return 0;
        return std::lround(slope() * dt);
    }

    static GroupFit fit(const std::vector<Point>& pts) {
        GroupFit g;
        g.points = pts.size();
        if (pts.size() < kMinPoints) return g;
        std::vector<double> slopes;
        slopes.reserve(pts.size() * (pts.size() - 1) / 2);
        for (size_t i = 0; i < pts.size(); ++i) {
            for (size_t j = i + 1; j < pts.size(); ++j) {
                const double ddt = pts[j].dt - pts[i].dt;
                if (std::abs(ddt) < 1.0) continue;
                slopes.push_back((pts[j].offset - pts[i].offset) / ddt);
            }
        }
        if (slopes.empty()) return g;
        const double s = median(slopes);
        std::vector<double> intercepts;
        intercepts.reserve(pts.size());
        for (const auto& p : pts) intercepts.push_back(p.offset - s * p.dt);
        const double b = median(intercepts);
        std::vector<double> abs_res;
        abs_res.reserve(pts.size());
        double mean_dt = 0.0;
        for (const auto& p : pts) {
            abs_res.push_back(std::abs(p.offset - (b + s * p.dt)));
            mean_dt += p.dt;
        }
        mean_dt /= static_cast<double>(pts.size());
        double sxx = 0.0;
        for (const auto& p : pts) sxx += (p.dt - mean_dt) * (p.dt - mean_dt);
        if (!(sxx > 0.0)) return g;
        const double sigma = std::max(1.482602218505602 * median(abs_res), kOffsetSigmaFloor);
        g.slope = s;
        g.se = sigma / std::sqrt(sxx);
        g.valid = std::abs(s) <= kMaxPlausibleSlope;
        return g;
    }

private:
    static double median(std::vector<double> v) {
        const size_t n = v.size();
        std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n / 2), v.end());
        const double hi = v[n / 2];
        if (n % 2 == 1) return hi;
        const double lo = *std::max_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(n / 2));
        return 0.5 * (lo + hi);
    }

    void fold() {
        if (points_.empty()) return;
        ++folds_;
        last_ = fit(points_);
        if (!last_.valid) return;
        groups_.push_back(last_);
        while (groups_.size() > kWindowGroups) groups_.pop_front();
    }

    std::vector<Point> points_;
    std::deque<GroupFit> groups_;
    GroupFit last_;
    size_t folds_ = 0;
};

}  // namespace sync
}  // namespace ultra
