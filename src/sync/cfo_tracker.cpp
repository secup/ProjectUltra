#include "sync/cfo_tracker.hpp"

#include "sync/signal_policy.hpp"   // limitConnectedCFODrift + kMaxSyncCFODriftHz
#include "ultra/logging.hpp"        // LOG_MODEM

#include <cmath>

namespace ultra {
namespace sync {

// §7 C-CFO-2: moved verbatim from the post-found CFO-handling block of
// StreamingDecoder::searchForSync (the `limitConnectedCFODrift` clamp + its "CFO sanity" log).
// Same policy call, same log, same return (measured when un-clamped, accepted when clamped) →
// byte-identical; only the home moved. The caller still applies the genie-timing diag override and
// stores the final value back into the tracker.
float CFOTracker::seedFromChirp(float measured_cfo, float correlation, bool connected,
                                const char* log_prefix) const {
    const float known = cfo_.load();
    if (!std::isfinite(measured_cfo)) {
        LOG_MODEM(WARN, "[%s] CFO sanity: non-finite chirp estimate rejected", log_prefix);
        return std::isfinite(known) ? known : 0.0f;
    }
    const auto decision =
        signal_policy::limitConnectedCFODrift(connected, measured_cfo, known, correlation,
                                              established_.load());
    if (decision.clamped) {
        LOG_MODEM(INFO,
                  "[%s] CFO sanity: measured=%.1f, known=%.1f, corr=%.2f, diff=%.1f > %.1f, using known",
                  log_prefix, measured_cfo, known, correlation, decision.diff_hz,
                  signal_policy::kMaxSyncCFODriftHz);
        return decision.accepted_cfo;  // low-confidence or established-drift -> trust tracked CFO
    }
    return measured_cfo;
}

// §7 C-CFO-3: moved from the per-decode-path "feed back pilot-corrected CFO to cached value" blocks.
// combinePilotCFO + the store(accepted) were repeated at 5 sites; centralizing the store here makes
// the feedback invariant (docs/CFO_CORRECTION_FLOW.md) un-droppable. The caller keeps its
// site-specific tail (logs / sync_cfo_ / burst_cfo_) using the returned update. Byte-identical: same
// combine, same stored value; the store just happens inside this call instead of a few lines later
// (nothing reads the tracker in between).
signal_policy::PilotCFOUpdate CFOTracker::ingestPilotResidual(
    float pre_correction, float residual, float current, bool clamp_drift) {
    auto update = signal_policy::combinePilotCFO(pre_correction, residual, current, clamp_drift);
    if (!std::isfinite(update.accepted_cfo)) {
        const float tracked_now = tracked();
        update.accepted_cfo = std::isfinite(current)
            ? current
            : (std::isfinite(tracked_now) ? tracked_now : 0.0f);
        update.clamped = true;
    }
    store(update.accepted_cfo);
    // NOTE (BUG-ANCHOR-CFO-KILL): ingest does NOT certify the warm value — burst
    // frames ingest residuals BEFORE any LDPC verdict exists, so a crater feeds
    // noise here. Certification is owned by decode OUTCOMES (certifyWarm on a
    // delivered group / classic decode success; revokeWarm on a 0/N group).
    return update;
}

// BUG-ANCHOR-CFO-KILL (2026-07-05): connected full-anchor seed — the chirp is
// timing-only once the tracker is pilot-refined; the warm value wins (see
// signal_policy::connectedAnchorCFOSeed for the measured evidence: 25% of
// full-anchor groups killed at 16QAM vs 0% of warm-LTS groups over 4 gate runs).
float CFOTracker::seedFromChirpConnectedAnchor(float measured_cfo, float correlation,
                                               const char* log_prefix) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const float known = cfo_.load();
    if (!std::isfinite(measured_cfo)) {
        LOG_MODEM(WARN, "[%s] Full-anchor CFO: non-finite chirp estimate rejected", log_prefix);
        return std::isfinite(known) ? known : 0.0f;
    }
    const auto seed = signal_policy::connectedAnchorCFOSeed(
        measured_cfo, known, correlation, pilot_seeded_.load(), established_.load());
    if (!seed.used_warm && seed.accepted_cfo != measured_cfo) {
        LOG_MODEM(INFO,
                  "[%s] Full-anchor CFO sanity: chirp %.2f Hz (corr=%.2f) rejected to "
                  "measured %.2f Hz",
                  log_prefix, measured_cfo, correlation, known);
    }
    if (seed.used_warm) {
        LOG_MODEM(INFO,
                  "[%s] Full-anchor CFO: warm %.2f Hz kept (chirp read %.2f Hz, corr=%.2f)",
                  log_prefix, known, measured_cfo, correlation);
    }
    return seed.accepted_cfo;
}

}  // namespace sync
}  // namespace ultra
