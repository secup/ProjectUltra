#include "sync/sync_controller.hpp"

#include "ultra/logging.hpp"   // LOG_MODEM (shared logging header; src/sync already uses LOG_SYNC)

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <utility>

// SyncController — SCAFFOLD implementation (2026-05-31).
//
// These are intentionally inert stubs: the object exists and compiles, but it is NOT yet wired
// into the decode loop, so behavior is unchanged. The migration (docs/SYNC_ACQUISITION_FIX_PLAN
// _2026_05_31.md §7.5) fills these in step by step, flag-gated:
//   1. detect()             ← move the detector dispatch from streaming_sync_acquisition.cpp
//   2. reportFrameOutcome() ← the WARM position+LDPC acceptance + COLD/WARM/RE_ACQUIRE transitions
//   3. noteGroupBoundary()  ← the descriptor-anchor re-arm + cadence seed
// Until then nothing calls these.

namespace ultra {
namespace sync {

void SyncController::reset(protocol::WaveformMode mode, IWaveform* wf, bool is_coherent) {
    waveform_mode_ = mode;
    waveform_ = wf;
    is_coherent_ = is_coherent;
    // (§7 C-CFO-1: last_cfo_ relocated to the decoder's CFOTracker, reset via cfo_tracker_.reset().)
    frame_arrival_confidence_ = 0.0f;
    consecutive_sync_misses_ = 0;
    next_expected_frame_sample_ = 0;
    next_expected_frame_sample_valid_ = false;
    expected_frame_gap_samples_ = 0;
    have_burst_descriptor_ = false;
    last_burst_descriptor_ = {};
}

SyncDecision SyncController::detect(SampleSpan buffer, size_t buffer_len, size_t buffer_abs_start) {
    // SCAFFOLD: detector dispatch not migrated yet — returns "no decision".
    (void)buffer;
    (void)buffer_len;
    (void)buffer_abs_start;
    SyncDecision d;
    d.mode = mode();
    return d;
}

void SyncController::reportFrameOutcome(bool ldpc_ok, size_t frame_end_abs) {
    // SCAFFOLD: position+LDPC acceptance + state transitions land here.
    (void)ldpc_ok;
    (void)frame_end_abs;
}

void SyncController::noteGroupBoundary(size_t descriptor_end_abs, size_t expected_frame_gap_samples) {
    next_expected_frame_sample_ = descriptor_end_abs;
    expected_frame_gap_samples_ = expected_frame_gap_samples;
    next_expected_frame_sample_valid_ = true;
}

// §7 C4: moved verbatim from streaming_burst_interleave.cpp (the "refresh warm-sync state on every
// delivered group" block) so the decoder no longer writes consecutive_sync_misses_ /
// frame_arrival_confidence_ / warm_sync_active_ / expect_full_ofdm_anchor_ directly. Same writes,
// same log, same order → byte-identical.
void SyncController::noteGroupDelivered(uint32_t group_seq, bool retransmission_required) {
    // Force WARM: misses=0 + active=true ⇒ derivePhase()==WARM (§7 collapse — verified byte-identical:
    // active was always already true here, the faithful translation of the old warm_sync_phase_=WARM).
    consecutive_sync_misses_ = 0;
    frame_arrival_confidence_ = std::max(frame_arrival_confidence_, 0.5f);
    warm_sync_active_ = true;
    // 2026-05-29 ROOT-CAUSE FIX: re-arm the full-chirp anchor expectation for the NEXT group's
    // BURST_HEADER. The encoder emits a chirp-bearing descriptor at the start of EVERY burst group, so
    // a precise chirp timing anchor is always on the wire. Without re-arming, warm-handoff left this
    // false after the first group, so bravo searched LIGHT-only, its data LTS correlated at ~0.54 (vs
    // ~0.91 right after a chirp anchor), forcing low-threshold marginal decodes + retries. The
    // BURST_HEADER-consume keeper then flips this back to false so the CONTIGUOUS group data stays light.
    //
    // #69 PERIODIC FULL ANCHOR + WARM-SKIP (ULTRA_ANCHOR_SKIP_K, default 1 = always re-arm =
    // byte-identical). When K>1 the sender skips the chirp on ~(K-1)/K of groups (LIGHT descriptor)
    // and ANNOUNCES the next group's anchor via BURST_FLAG_NEXT_LIGHT_ANCHOR (read into
    // next_group_light_anchor_ at descriptor parse). Arm the RIGHT search for the next group from
    // that announcement: full-search a chirp group, light-search a skip group — IMMEDIATELY, no
    // grinding through light rejects (the ~30 s/resend stall that crawled K=3). A dropped descriptor
    // leaves next_group_light_anchor_ false → expect full (safe). Reactive resends are unannounced
    // and always full chirp. When this group needs a resend, the previous descriptor's NEXT_LIGHT
    // bit no longer describes the next physical burst; force full immediately instead of missing
    // the resend descriptor and grinding through the §16.4 light-reject escalation first.
    static const int kAnchorSkipK = [] {
        const char* e = std::getenv("ULTRA_ANCHOR_SKIP_K");
        return (e && *e) ? std::max(1, std::atoi(e)) : 1;  // matches reliability-default encoder
    }();
    expect_full_ofdm_anchor_ = retransmission_required ||
                               (kAnchorSkipK <= 1) || !next_group_light_anchor_;
    LOG_MODEM(INFO,
        "[%s] s16-warm-handoff: refreshed warm-sync state on "
        "delivered group_seq=%u (conf=%.2f phase=WARM misses=0, "
        "next-anchor=%s K=%d resend=%d)",
        log_prefix_.c_str(),
        static_cast<unsigned>(group_seq),
        frame_arrival_confidence_,
        expect_full_ofdm_anchor_ ? "FULL-chirp" : "LIGHT-skip(announced)", kAnchorSkipK,
        retransmission_required ? 1 : 0);
}

// --- arrival-tracking transition logic (§7.4 A2) ---------------------------------------------
// Moved verbatim from StreamingDecoder::{resetFrameArrivalTrackingLocked, noteFrameArrival
// SuccessLocked (minus the connected_/OFDM_CHIRP guard, which stays in the decoder forwarder),
// noteFrameArrivalSyncMissLocked}. Same computations, same order, same log format/prefix → the
// behavior + log output are byte-identical; only the home moved.

void SyncController::resetFrameArrivalTracking() {
    warm_sync_active_ = false;
    next_expected_frame_sample_valid_ = false;
    next_expected_frame_sample_ = 0;
    frame_arrival_confidence_ = 0.0f;
    consecutive_sync_misses_ = 0;
    last_frame_arrival_valid_ = false;
    last_frame_start_sample_ = 0;
    last_frame_end_sample_ = 0;
    last_frame_arrival_error_valid_ = false;
    last_frame_arrival_error_samples_ = 0;
}

void SyncController::noteFrameArrivalSuccess(size_t frame_start_abs, size_t frame_end_abs) {
    if (!next_expected_frame_sample_valid_ &&
        !expect_full_ofdm_anchor_ &&
        expectedFrameGapSamples() == 0) {
        return;
    }

    const auto previous_phase = derivePhase();
    const auto update = frame_arrival_policy::updateOnSuccessfulFrame(
        next_expected_frame_sample_valid_,
        next_expected_frame_sample_,
        frame_arrival_confidence_,
        frame_start_abs,
        frame_end_abs,
        expectedFrameGapSamples());

    next_expected_frame_sample_valid_ = true;
    warm_sync_active_ = true;
    next_expected_frame_sample_ = update.next_expected_frame_sample;
    frame_arrival_confidence_ = update.confidence;
    consecutive_sync_misses_ = update.consecutive_sync_misses;
    last_frame_arrival_valid_ = true;
    last_frame_start_sample_ = frame_start_abs;
    last_frame_end_sample_ = frame_end_abs;
    last_frame_arrival_error_valid_ = update.has_arrival_error;
    last_frame_arrival_error_samples_ = update.arrival_error_samples;

    if (update.has_arrival_error) {
        LOG_MODEM(DEBUG, "[%s] warm-sync arrival: start=%zu end=%zu next=%zu error=%lld confidence=%.2f",
                  log_prefix_.c_str(), frame_start_abs, frame_end_abs,
                  next_expected_frame_sample_,
                  static_cast<long long>(last_frame_arrival_error_samples_),
                  frame_arrival_confidence_);
    } else {
        LOG_MODEM(DEBUG, "[%s] warm-sync arrival seeded: start=%zu end=%zu next=%zu confidence=%.2f",
                  log_prefix_.c_str(), frame_start_abs, frame_end_abs,
                  next_expected_frame_sample_, frame_arrival_confidence_);
    }

    const auto new_phase = derivePhase();
    if (previous_phase != new_phase) {
        LOG_MODEM(INFO, "[%s] warm-sync state: %s -> %s",
                  log_prefix_.c_str(),
                  frame_arrival_policy::warmSyncPhaseName(previous_phase),
                  frame_arrival_policy::warmSyncPhaseName(new_phase));
    }
}

void SyncController::noteFrameArrivalSyncMiss() {
    const auto previous_phase = derivePhase();
    consecutive_sync_misses_ = frame_arrival_policy::incrementSyncMisses(consecutive_sync_misses_);
    frame_arrival_confidence_ =
        frame_arrival_policy::confidenceAfterSyncMiss(frame_arrival_confidence_);

    if (next_expected_frame_sample_valid_ && last_frame_arrival_valid_) {
        const size_t last_duration =
            last_frame_end_sample_ >= last_frame_start_sample_
                ? (last_frame_end_sample_ - last_frame_start_sample_)
                : 0;
        const size_t cadence = last_duration + expectedFrameGapSamples();
        if (cadence > 0) {
            next_expected_frame_sample_ += cadence;
        }
    }

    if (consecutive_sync_misses_ >= frame_arrival_policy::kWarmSyncMissesBeforeRecovery) {
        warm_sync_active_ = false;
        next_expected_frame_sample_valid_ = false;
        frame_arrival_confidence_ = 0.0f;
    }

    const auto new_phase = derivePhase();
    if (previous_phase != new_phase) {
        LOG_MODEM(INFO, "[%s] warm-sync state: %s -> %s (misses=%d)",
                  log_prefix_.c_str(),
                  frame_arrival_policy::warmSyncPhaseName(previous_phase),
                  frame_arrival_policy::warmSyncPhaseName(new_phase),
                  consecutive_sync_misses_);
    }
}

// --- connected-data light-LTS acceptance (§7.4 chunk B) --------------------------------------
// Moved verbatim from StreamingDecoder::searchForSync (the `if (!found)` acceptance block):
// signal_policy::evaluateLightSyncCandidate + the ULTRA_S16_WARM_HANDOFF warm-override +
// the WARM position-gating, folded into one verdict. Same computations, same order, same
// log format/prefix → byte-identical behavior + log output; only the home moved. The decoder
// still runs the detector, builds the search buffer, and applies the position gate to
// sync_result (start_sample/cfo_hz/correlation); this owns the sync_reject_streak_ update.
LightSyncAcceptance SyncController::acceptLightSyncCandidate(
    bool detector_found, float correlation, bool is_coherent, bool connected,
    float known_cfo, size_t search_start, size_t search_window_len,
    const signal_policy::LightSyncThresholds& thresholds) {
    LightSyncAcceptance result;

    // Reject clear false positives (noise floor is ~0.2-0.4)
    auto sync_decision = signal_policy::evaluateLightSyncCandidate(
        detector_found, correlation, is_coherent, connected,
        sync_reject_streak_, thresholds);
    // §16.8 step 2 (ULTRA_S16_WARM_HANDOFF): the coherent-QPSK
    // sync threshold is 0.90 because stale LTS phases can't be
    // recovered by DD tracking alone. In the warm-handoff regime we
    // are NOT stale — the BURST_HEADER just decoded with a fresh full
    // chirp+LTS anchor and seeded last_cfo_. This override is a
    // BACKSTOP for a group-start DATA frame whose light-LTS dips just
    // under 0.90 right after a known-good anchor. The PRIMARY fix for
    // group-boundary acquisition is re-arming the descriptor chirp
    // anchor every group (streaming_burst_interleave.cpp end-of-group),
    // which keeps the contiguous data correlating high (~0.91); this
    // override should rarely fire once that anchor is used.
    constexpr float kS16WarmHandoffMinCorrelation = 0.55f;
    const bool s16_warm_override =
        is_coherent &&
        derivePhase() == frame_arrival_policy::WarmSyncPhase::WARM &&
        sync_decision.rejected && detector_found &&
        correlation >= kS16WarmHandoffMinCorrelation;
    if (s16_warm_override) {
        LOG_MODEM(INFO,
            "[%s] s16-warm-handoff: ACCEPT light-LTS sync corr=%.2f "
            "(WARM phase, conf=%.2f, threshold-floor=%.2f); coherent "
            "0.90 gate bypassed",
            log_prefix_.c_str(), correlation,
            frame_arrival_confidence_, kS16WarmHandoffMinCorrelation);
        sync_decision.found = true;
        sync_decision.rejected = false;
        sync_decision.next_reject_streak = 0;
    } else if (detector_found && correlation < thresholds.min_confidence) {
        if (sync_decision.weak_accept) {
            LOG_MODEM(INFO, "[%s] DATA sync weak-accepted (corr=%.2f < %.2f, streak=%llu)",
                      log_prefix_.c_str(), correlation,
                      thresholds.min_confidence,
                      static_cast<unsigned long long>(sync_reject_streak_));
        } else if (sync_decision.rejected) {
            LOG_MODEM(INFO, "[%s] DATA sync rejected (corr=%.2f < %.2f, streak=%llu)",
                      log_prefix_.c_str(), correlation,
                      thresholds.min_confidence,
                      static_cast<unsigned long long>(sync_decision.next_reject_streak));
        }
    }

    // §16.8 WARM position-gating (low-SNR fix, 2026-05-31 — see
    // docs/SYNC_ACQUISITION_FIX_PLAN_2026_05_31.md). At low SNR the warm light-LTS
    // correlation floors at NOISE (~0.15 measured at DQPSK R1/4 AWGN@10), so the frame is
    // UNFINDABLE by search and the normal gate rejects it — even though the cadence
    // prediction is correct (descriptor-seeded, contiguous frames) and the data decodes
    // (legacy: 776 CW on the same signal). The audit §9.7 fix: in WARM with the narrow
    // predicted window, do NOT gate on LTS correlation — PROCESS at the predicted position
    // and let LDPC be the acceptance decision (the LTS there still gives the channel
    // estimate H). detectDataSync's reported position is noise here, so we use
    // next_expected. Engages ONLY when the normal correlation path already failed, so
    // higher-SNR locks (which find the true peak) are byte-identical. On a misprediction
    // the frame's LDPC simply fails → existing NACK / §16.4 full-chirp escalation handles
    // it. (§16.8 warm-handoff — now unconditional; promoted past ULTRA_S16_WARM_HANDOFF.)
    if (!sync_decision.found &&
        derivePhase() == frame_arrival_policy::WarmSyncPhase::WARM &&
        thresholds.narrow_expected_window &&
        next_expected_frame_sample_valid_ &&
        next_expected_frame_sample_ >= search_start &&
        (next_expected_frame_sample_ - search_start) < search_window_len) {
        result.position_gated = true;
        result.position_gate_abs = next_expected_frame_sample_;
        result.position_gate_cfo = known_cfo;
        sync_decision.found = true;
        sync_decision.rejected = false;
        sync_decision.next_reject_streak = 0;
        LOG_MODEM(INFO,
            "[%s] WARM position-gated: processing predicted frame at abs=%llu "
            "(light-LTS corr below noise floor; cadence-located, LDPC validates)",
            log_prefix_.c_str(),
            static_cast<unsigned long long>(next_expected_frame_sample_));
    }

    result.found = sync_decision.found;
    sync_reject_streak_ = sync_decision.next_reject_streak;
    return result;
}

void SyncController::seedArrivalAfterDelay(size_t total_fed_abs, size_t delay_samples,
                                           float confidence) {
    const auto previous_phase = derivePhase();
    warm_sync_active_ = true;
    next_expected_frame_sample_valid_ = true;
    next_expected_frame_sample_ = total_fed_abs + delay_samples;
    frame_arrival_confidence_ =
        frame_arrival_policy::clampConfidence(std::max(frame_arrival_confidence_, confidence));
    consecutive_sync_misses_ = 0;
    last_frame_arrival_error_valid_ = false;
    last_frame_arrival_error_samples_ = 0;

    LOG_MODEM(DEBUG,
              "[%s] warm-sync arrival seeded from local TX: now=%zu delay=%zu next=%zu confidence=%.2f",
              log_prefix_.c_str(), total_fed_abs, delay_samples,
              next_expected_frame_sample_, frame_arrival_confidence_);
    const auto new_phase = derivePhase();
    if (previous_phase != new_phase) {
        LOG_MODEM(INFO, "[%s] warm-sync state: %s -> %s",
                  log_prefix_.c_str(),
                  frame_arrival_policy::warmSyncPhaseName(previous_phase),
                  frame_arrival_policy::warmSyncPhaseName(new_phase));
    }
}

// --- warm-window planning (§7.4 chunk-B tail) ------------------------------------------------
// Moved verbatim from StreamingDecoder::searchForSync (the s16_skip_short_lead +
// expected_sync_search_sample + planWarmSearchWindow + short-reanchor-lead adjustment). Same
// computations, same order, no log output → byte-identical. The decoder still derives the ring
// values (oldest_abs / correlation_abs) it passes in and does the actual buffer extraction.
// R4: the short-chirp re-anchor was removed (superseded by warm-handoff, now default), so the
// warm search anchors directly at next_expected_frame_sample_ — no lead shift, no post-adjust.
frame_arrival_policy::WarmSearchWindowPlan SyncController::planWarmSearch(
    bool use_light_search, size_t total_fed, size_t oldest_abs,
    bool search_floor_valid, size_t search_floor_abs, size_t correlation_abs,
    size_t symbol_samples, size_t correlation_step) {
    return frame_arrival_policy::planWarmSearchWindow(
        use_light_search,
        warm_sync_active_,
        next_expected_frame_sample_valid_,
        next_expected_frame_sample_,
        frame_arrival_confidence_,
        consecutive_sync_misses_,
        total_fed,
        oldest_abs,
        search_floor_valid,
        search_floor_abs,
        correlation_abs,
        symbol_samples,
        correlation_step);
}

// §7 C3 Phase 3a: search-window PRODUCTION moved verbatim out of StreamingDecoder::searchForSync.
// The controller owns the ring, so it owns extracting the next search window from it (ring access
// + planWarmSearch + RMS gate + post-frame floor). Same computations, logs, order, and early-return
// semantics -> byte-identical; the decoder still runs the detector dispatch on the returned window.
SearchWindowResult SyncController::acquireSearchWindow(
    bool use_light_search, bool connected_data_preamble, bool disconnected_mc_dpsk,
    size_t min_search, size_t data_symbol_samples, bool audio_active,
    size_t correlation_step, float corr_noise_threshold) {
    SearchWindowResult result;
    std::vector<float> search_buffer;
    size_t search_start = 0;
    size_t search_start_abs = 0;
    bool used_warm_timed_window = false;
    bool used_warm_narrow_window = false;
    size_t warm_narrow_end_abs = 0;
    size_t warm_narrow_candidate_span_samples = 0;

    {
        std::lock_guard<std::mutex> lock(ring_.buffer_mutex_);

        float audio_sec = ring_.total_fed_ / 48000.0f;

        // Initialize ring_.correlation_pos_ if needed
        if (ring_.correlation_pos_ == 0 && ring_.total_fed_ > 0) {
            if (ring_.total_fed_ < ring_.buffer_capacity_samples_) {
                ring_.correlation_pos_ = 0;
            } else {
                ring_.correlation_pos_ = ring_.write_pos_;
            }
        }

        const size_t oldest_abs = (ring_.total_fed_ > ring_.buffer_capacity_samples_)
            ? (ring_.total_fed_ - ring_.buffer_capacity_samples_)
            : 0;
        const size_t correlation_abs = ring_.ringPosToAbsoluteLocked(ring_.correlation_pos_);
        // §7.4 chunk-B tail: the warm-window PLANNING decision (s16 skip-short-lead +
        // expected search anchor + planWarmSearchWindow + short-reanchor-lead adjustment)
        // now lives on the controller, which owns the warm-sync state it reads. The decoder
        // still owns the ring buffer — it derived oldest_abs / correlation_abs above and does
        // the wait / activate / extraction below from the returned plan.
        auto warm_plan = planWarmSearch(
            use_light_search,
            ring_.total_fed_,
            oldest_abs,
            ring_.search_floor_abs_valid_,
            ring_.search_floor_abs_,
            correlation_abs,
            data_symbol_samples,
            correlation_step);

        if (warm_plan.wait_for_more_samples) {
            static int warm_wait_count = 0;
            if (++warm_wait_count % 50 == 1) {
                LOG_MODEM(INFO,
                          "[%s] warm-sync: wait for expected window, need_abs=%zu total=%zu expected=%zu",
                          log_prefix_.c_str(), warm_plan.search_end_abs, ring_.total_fed_,
                          next_expected_frame_sample_);
            }
            return result;
        }

        if (warm_plan.active) {
            used_warm_timed_window = true;
            used_warm_narrow_window = warm_plan.lower_threshold;
            warm_narrow_end_abs = warm_plan.search_end_abs;
            warm_narrow_candidate_span_samples = warm_plan.candidate_span_samples;
            min_search = warm_plan.search_size_samples;
            search_start = ring_.absoluteToRingLocked(warm_plan.search_start_abs);
        }
        // §16.8 step 2 v2 diagnostic: log warm-window decision once per
        // search invocation. ULTRA_S16_TRACE_WARM_WINDOW=1 enables.
        {
            const char* trace = std::getenv("ULTRA_S16_TRACE_WARM_WINDOW");
            if (trace && std::atoi(trace) != 0) {
                LOG_MODEM(INFO,
                    "[%s] s16-warm-window: active=%d wait=%d lower_threshold=%d "
                    "phase=%s active_flag=%d has_pred=%d expected=%llu "
                    "conf=%.2f misses=%d use_light=%d total=%llu "
                    "search[%llu..%llu] span=%zu",
                    log_prefix_.c_str(),
                    warm_plan.active ? 1 : 0,
                    warm_plan.wait_for_more_samples ? 1 : 0,
                    warm_plan.lower_threshold ? 1 : 0,
                    frame_arrival_policy::warmSyncPhaseName(derivePhase()),
                    warm_sync_active_ ? 1 : 0,
                    next_expected_frame_sample_valid_ ? 1 : 0,
                    static_cast<unsigned long long>(next_expected_frame_sample_),
                    frame_arrival_confidence_,
                    consecutive_sync_misses_,
                    use_light_search ? 1 : 0,
                    static_cast<unsigned long long>(ring_.total_fed_),
                    static_cast<unsigned long long>(warm_plan.search_start_abs),
                    static_cast<unsigned long long>(warm_plan.search_end_abs),
                    warm_plan.candidate_span_samples);
            }
        }

        // Need minimum samples before we can search
        if (ring_.total_fed_ < min_search) {
            static int skip_count = 0;
            if (++skip_count % 50 == 1)
                LOG_MODEM(INFO, "[%s] searchForSync: SKIP not enough samples, total=%.2fs, need=%.2fs",
                          log_prefix_.c_str(), audio_sec, min_search / 48000.0f);
            return result;
        }

        // Calculate unsearched data available.
        //
        // BUG-SYNC-CURSOR-AHEAD (2026-07-30): the else-branch below is a raw modular ring
        // distance and it is correct ONLY if the ring has wrapped. If correlation_pos_ is
        // instead simply AHEAD of write_pos_ — which happens whenever a cursor is parked at
        // sync_position_ + frame_len past live audio (streaming_ofdm_decode.cpp writes 15+
        // such cursors, none clamped, unlike their already-clamped sibling
        // setSearchFloorLocked) — this returns capacity - lead, i.e. a phantom near-full
        // buffer. On the Pi 5 that printed as "LOAD-SHED 2187129 samples (45.6 s) — search
        // fell behind live" when only 2.6 s of audio had EVER been fed. The search had not
        // fallen behind at all; it was 1.8 s ahead. Reconstruction:
        //     shed 2187129 + backlog_cap 124800 = 2311929 = 2400000 - 88071
        // i.e. a cursor 88071 samples (1.835 s) in the future.
        //
        // GUARD: unsearched can never exceed the number of samples ever fed. That invariant
        // holds under every wrap state and needs no knowledge of which writer misbehaved, so
        // it catches the whole family rather than one instance. Violation means the cursor is
        // ahead of live; the only sound recovery is to treat the search as caught up (there
        // is by definition nothing beyond live to search) and re-anchor the cursor to the
        // write head so the next pass starts from real audio.
        size_t unsearched;
        if (ring_.write_pos_ >= ring_.correlation_pos_) {
            unsearched = ring_.write_pos_ - ring_.correlation_pos_;
        } else {
            unsearched = ring_.buffer_capacity_samples_ - ring_.correlation_pos_ + ring_.write_pos_;
        }
        if (unsearched > ring_.total_fed_) {
            static int cursor_ahead_count = 0;
            if (++cursor_ahead_count % 20 == 1) {
                LOG_MODEM(ERROR,
                          "[%s] searchForSync: CURSOR AHEAD OF LIVE — unsearched=%zu > "
                          "total_fed=%zu (corr_pos=%zu write_pos=%zu). Impossible state; "
                          "re-anchoring to the write head (x%d)",
                          log_prefix_.c_str(), unsearched, ring_.total_fed_,
                          ring_.correlation_pos_, ring_.write_pos_, cursor_ahead_count);
            }
            ring_.correlation_pos_ = ring_.write_pos_;
            unsearched = 0;
        }

        // LOAD-SHED (BUG-DECODE-BACKLOG-COLLISIONS, F176/F186): under deep-fade
        // search thrash the per-buffer correlation cost exceeds real time and
        // the search position detaches from the antenna's "now" (F176: a burst
        // AIRED at t≈218, its anchor was accepted at t≈239 — every receiver
        // response left ~20 s stale and collided with the sender's recovery
        // bursts; F186: 7,668 anchor-wait rejects, died at 75 %). A real radio
        // that falls behind MUST drop audio, not time-travel: cap the search
        // backlog at ~2 s of real time — jump the search floor forward and eat
        // the loss (the skipped audio is stale by definition; a chirp inside
        // it is an old chirp, and the sender's RTO covers anything real).
        // Frame ASSEMBLY for an already-anchored group is untouched (it reads
        // ring positions independently of the search cursor).
        // Cap must NEVER trim below what the search itself requires (min_search
        // can be 2.5 s for the full chirp window; the first cut of this shed
        // capped at 2 s flat and starved the search into total deafness — the
        // faithful gate caught it with a hard failure before it reached the rig).
        const size_t backlog_cap =
            std::max<size_t>(96000, min_search + correlation_step);
        if (real_time_audio_ && unsearched > backlog_cap) {
            const size_t shed = unsearched - backlog_cap;
            ring_.correlation_pos_ =
                ring_.wrapRingIndexLocked(ring_.correlation_pos_ + shed);
            unsearched = backlog_cap;
            static int shed_log_count = 0;
            if (++shed_log_count % 5 == 1) {
                LOG_MODEM(WARN,
                          "[%s] searchForSync: LOAD-SHED %zu samples (%.1f s) — "
                          "search fell behind live; staying within %.1f s of the "
                          "antenna (x%d)",
                          log_prefix_.c_str(), shed, shed / 48000.0f,
                          backlog_cap / 48000.0f, shed_log_count);
            }
        }

        // Need at least min_search unsearched samples
        if (!used_warm_timed_window && unsearched < min_search) {
            static int skip_count2 = 0;
            if (++skip_count2 % 50 == 1)
                LOG_MODEM(INFO, "[%s] searchForSync: SKIP unsearched=%zu < min=%zu, total=%.2fs, corr_pos=%zu",
                          log_prefix_.c_str(), unsearched, min_search, audio_sec, ring_.correlation_pos_);
            return result;
        }

        // Quick RMS check for signal presence. For disconnected MC-DPSK chirps,
        // use the strongest 20ms slice across the next 100ms search step. A
        // Watterson notch can erase one narrow chirp segment while the rest of
        // the sweep remains detectable; the correlator is the real detector,
        // this gate only keeps silence from burning CPU.
        const size_t rms_probe_pos = used_warm_timed_window ? search_start : ring_.correlation_pos_;
        float rms = 0.0f;
        for (size_t i = 0; i < kRmsProbeSamples; i++) {
            float s = ring_.buffer_[ring_.wrapRingIndexLocked(rms_probe_pos + i)];
            rms += s * s;
        }
        rms = std::sqrt(rms / static_cast<float>(kRmsProbeSamples));

        if (disconnected_mc_dpsk) {
            float max_slice_rms = rms;
            constexpr size_t RMS_SLICE_SAMPLES = 1000;
            for (size_t off = RMS_SLICE_SAMPLES;
                 off + RMS_SLICE_SAMPLES <= correlation_step;
                 off += RMS_SLICE_SAMPLES) {
                float slice_sum = 0.0f;
                for (size_t i = 0; i < RMS_SLICE_SAMPLES; ++i) {
                    float s = ring_.buffer_[ring_.wrapRingIndexLocked(rms_probe_pos + off + i)];
                    slice_sum += s * s;
                }
                max_slice_rms = std::max(
                    max_slice_rms,
                    std::sqrt(slice_sum / static_cast<float>(RMS_SLICE_SAMPLES)));
            }
            rms = max_slice_rms;
        }

        // OTA-connected mode can run at lower absolute amplitudes than simulator
        // defaults. Use an adaptive gate so valid low-level frames are not skipped.
        float rms_gate = corr_noise_threshold;
        if (disconnected_mc_dpsk) {
            float noise_floor = std::max(0.0005f, ring_.noise_floor_);
            if (rms < corr_noise_threshold) {
                ring_.noise_floor_ = 0.98f * noise_floor + 0.02f * rms;
            } else {
                ring_.noise_floor_ = 0.995f * noise_floor + 0.005f * rms;
            }

            // Before sync there is no SNR estimate. Use the measured audio
            // floor, but never raise the historical 0.025 gate; this only
            // relaxes acquisition when high-SNR fading leaves low absolute RMS.
            rms_gate = std::clamp(ring_.noise_floor_ * 3.0f, 0.006f, corr_noise_threshold);
            if (audio_active) {
                rms_gate = std::min(rms_gate, 0.012f);
            }
        } else if (connected_data_preamble) {
            float noise_floor = std::max(0.001f, ring_.noise_floor_);
            if (rms < noise_floor * 3.0f) {
                ring_.noise_floor_ = 0.98f * noise_floor + 0.02f * rms;
            } else {
                ring_.noise_floor_ = 0.995f * noise_floor + 0.005f * rms;
            }

            // Typical OTA values observed around 0.02-0.04 RMS; keep floor low enough
            // to avoid starving detectDataSync() while still skipping true silence.
            rms_gate = std::clamp(ring_.noise_floor_ * 2.2f, 0.015f, 0.040f);
            if (sync_reject_streak_ >= 8) {
                float relax = std::min(0.010f,
                                       0.001f * static_cast<float>(sync_reject_streak_ - 7));
                rms_gate = std::max(0.012f, rms_gate - relax);
            }
        }

        if (rms < rms_gate) {
            // No signal - advance by small step (100ms = 4800 samples)
            static int rms_skip_count = 0;
            if (++rms_skip_count % 10 == 1)
                LOG_MODEM(INFO, "[%s] searchForSync: RMS skip, rms=%.4f < %.3f, corr_pos=%zu, total=%.2fs",
                          log_prefix_.c_str(), rms, rms_gate, ring_.correlation_pos_, audio_sec);
            ring_.correlation_pos_ = ring_.wrapRingIndexLocked(ring_.correlation_pos_ + correlation_step);
            return result;
        }

        // Signal detected - log before running correlation (only occasionally to reduce spam)
        static int run_log_count = 0;
        if (++run_log_count % 10 == 1) {
            LOG_MODEM(INFO, "[%s] searchForSync: RUNNING correlation, rms=%.4f, corr_pos=%zu, total=%.2fs",
                      log_prefix_.c_str(), rms, ring_.correlation_pos_, audio_sec);
        }

        // Signal present - back up search start to catch chirp that might have started
        // in the lead-in silence. The TX lead-in is ~150ms (7200 samples), so we should
        // back up at least that much to ensure the chirp START is in our search window.
        // FIX: We may have skipped past the chirp start during low-RMS phases.
        constexpr size_t SEARCH_BACKTRACK = kSearchBacktrackSamples; // slightly more than lead-in

        if (!used_warm_timed_window) {
            if (ring_.correlation_pos_ >= SEARCH_BACKTRACK) {
                search_start = ring_.correlation_pos_ - SEARCH_BACKTRACK;
            } else if (ring_.total_fed_ < ring_.buffer_capacity_samples_) {
                // Buffer hasn't wrapped yet, start from beginning
                search_start = 0;
            } else {
                // Buffer wrapped, handle underflow
                search_start = ring_.wrapRingIndexLocked(ring_.buffer_capacity_samples_ + ring_.correlation_pos_ - SEARCH_BACKTRACK);
            }
        }

        // Do not let the backtrack window re-enter audio that a previous decode
        // already consumed. On sustained OFDM ACK traffic, searching the tail of a
        // just-decoded 1-CW control frame can find false LTS-like peaks; those
        // false locks then escalate into expensive fixed-frame LDPC attempts and delay
        // real ACKs long enough to trigger ARQ retransmission storms.
        if (!used_warm_timed_window && ring_.search_floor_abs_valid_) {
            if (ring_.search_floor_abs_ < oldest_abs) {
                ring_.search_floor_abs_ = oldest_abs;
            }
            if (ring_.search_floor_abs_ > ring_.total_fed_) {
                ring_.search_floor_abs_ = ring_.total_fed_;
            }

            size_t search_start_abs = ring_.ringPosToAbsoluteLocked(search_start);
            // CHIRP searches may start up to kChirpCfoPeakLeadSamples BEFORE the floor: with a
            // positive CFO the up-chirp's correlation peak precedes the chirp's first sample by
            // kappa * CFO samples, so a chirp that starts exactly at the floor (a FULL-anchored
            // resend group right after its descriptor) had its peak cut off — never found at
            // +7.3 Hz (corr 0.08), group lost, resend repeated (measured 2026-11-08, the first
            // non-zero-CFO GUI runs). The floor guards against false LTS locks in decoded audio;
            // a chirp correlator cannot lock on OFDM symbols, so the lead is safe for it.
            const size_t floor_abs = (!use_light_search && connected_data_preamble &&
                                      ring_.search_floor_abs_ > oldest_abs + kChirpCfoPeakLeadSamples)
                                         ? ring_.search_floor_abs_ - kChirpCfoPeakLeadSamples
                                         : ring_.search_floor_abs_;
            if (search_start_abs < floor_abs) {
                if (ring_.total_fed_ - floor_abs < min_search) {
                    static int floor_wait_count = 0;
                    if (++floor_wait_count % 50 == 1) {
                        LOG_MODEM(INFO,
                                  "[%s] searchForSync: SKIP post-frame floor, available=%zu < min=%zu",
                                  log_prefix_.c_str(), ring_.total_fed_ - floor_abs, min_search);
                    }
                    return result;
                }
                search_start = ring_.absoluteToRingLocked(floor_abs);
            }
            const size_t start_abs_final = ring_.ringPosToAbsoluteLocked(search_start);
            if (start_abs_final < ring_.search_floor_abs_) {
                result.pre_floor_lead_samples = ring_.search_floor_abs_ - start_abs_final;
            }
        }

        search_buffer.resize(min_search);
        for (size_t i = 0; i < min_search; i++) {
            search_buffer[i] = ring_.buffer_[ring_.wrapRingIndexLocked(search_start + i)];
        }
        search_start_abs = ring_.ringPosToAbsoluteLocked(search_start);

        if (used_warm_timed_window) {
            ring_.correlation_pos_ = ring_.absoluteToRingLocked(warm_narrow_end_abs);
            LOG_MODEM(INFO,
                      "[%s] warm-sync: %s LTS search expected=%zu start_abs=%zu size=%zu confidence=%.2f",
                      log_prefix_.c_str(),
                      used_warm_narrow_window ? "narrow" : "degraded",
                      next_expected_frame_sample_,
                      warm_plan.search_start_abs, min_search,
                      frame_arrival_confidence_);
        } else {
            // Advance by small step (100ms = 4800 samples) for accurate detection
            ring_.correlation_pos_ = ring_.wrapRingIndexLocked(ring_.correlation_pos_ + correlation_step);
        }
    }

    result.ready = true;
    result.search_buffer = std::move(search_buffer);
    result.search_start = search_start;
    result.search_start_abs = search_start_abs;
    result.min_search = min_search;
    result.used_warm_timed_window = used_warm_timed_window;
    result.used_warm_narrow_window = used_warm_narrow_window;
    result.warm_narrow_end_abs = warm_narrow_end_abs;
    result.warm_narrow_candidate_span_samples = warm_narrow_candidate_span_samples;
    return result;
}

// §7 C3 Phase 3b: the connected-data light-LTS dispatch, moved verbatim from the
// `if (use_light_search)` branch of StreamingDecoder::searchForSync. Same detector call, same
// acceptance + position-gate, same logs + §16.4 escalation, same order → byte-identical; only the
// data_sync_accepted_callback_ fire stays in the decoder (this returns found, the decoder fires it).
bool SyncController::detectConnectedLightSync(
    IWaveform* waveform, const float* search_data, size_t search_len, size_t search_start,
    bool is_coherent, bool connected, protocol::WaveformMode mode,
    const signal_policy::LightSyncThresholds& thresholds, float corr_detect_threshold,
    float known_cfo, SyncResult& sync_result) {
    // ACK-LISTEN tone-lock guard (see the setter comment in the header): while the
    // sender awaits a tone-burst ACK, the peer cannot be sending OFDM — the only
    // signal on the medium is the ACK tone, which S&C-false-locks this detector
    // (sc~0.9/mf~0.1). Return not-found WITHOUT running the detector and WITHOUT
    // touching the reject streak / §16.4 escalation (a suppressed window is not
    // acquisition-failure evidence).
    if (ack_listen_suppress_data_sync_ && connected) {
        if (!ack_listen_suppress_logged_) {
            ack_listen_suppress_logged_ = true;
            LOG_MODEM(INFO,
                      "[%s] ACK-LISTEN: warm data-sync suppressed (tone window; "
                      "S&C tone-lock guard)",
                      log_prefix_.c_str());
        }
        return false;
    }
    // known_cfo is the tracked CFO (§7 C-CFO-1: passed in from the decoder's CFOTracker).

    bool found = waveform->detectDataSync(
        SampleSpan(search_data, search_len),
        sync_result, known_cfo, corr_detect_threshold);

    auto accept = acceptLightSyncCandidate(
        found, sync_result.correlation, is_coherent, connected,
        known_cfo, search_start, search_len,
        thresholds);
    if (accept.position_gated) {
        sync_result.start_sample =
            static_cast<int>(accept.position_gate_abs - search_start);
        sync_result.cfo_hz = accept.position_gate_cfo;
        sync_result.correlation = 0.0f;  // position-gated, not correlation-found
    }
    found = accept.found;

    if (found) {
        if (thresholds.narrow_expected_window) {
            LOG_MODEM(INFO,
                      "[%s] DATA sync detected in warm window (known CFO=%.1f Hz, corr=%.2f, threshold=%.2f, window_reduction=%.2fx)",
                      log_prefix_.c_str(), known_cfo, sync_result.correlation,
                      thresholds.min_confidence,
                      thresholds.false_positive_window_reduction);
        } else {
            LOG_MODEM(INFO, "[%s] DATA sync detected (training only, known CFO=%.1f Hz, corr=%.2f)",
                      log_prefix_.c_str(), known_cfo, sync_result.correlation);
        }
    }

    // §16.4 escalation: warm/light group-start acquisition has failed for many consecutive
    // candidates; arm a full chirp+LTS re-anchor (the sender pays for a chirp on its RESEND).
    // expect_full_ofdm_anchor_ is cleared again after the next clean data decode — a one-group
    // escalation, not a permanent revert to per-group chirps. Unconditional (promoted past
    // ULTRA_S16_WARM_HANDOFF).
    // #69: under ULTRA_ANCHOR_SKIP_K>1 a RESEND (always full chirp, reactive → unannounced by the
    // next-light flag) arrives while the RX is light-searching an announced-skip group. The default
    // 12-reject escalation took ~30 s/resend (the K=3 crawl). Escalate FAST (4 rejects, ~8 s) when
    // anchor-skip is active so resends re-acquire promptly; the periodic skips themselves no longer
    // grind (the wire flag arms full-search on chirp groups directly).
    static const uint64_t kEscalateStreak = [] {
        const char* e = std::getenv("ULTRA_ANCHOR_SKIP_K");
        const int k = (e && *e) ? std::max(1, std::atoi(e)) : 1;  // matches reliability-default encoder
        return (k > 1) ? uint64_t{4} : signal_policy::kConnectedOFDMReanchorEscalateStreak;
    }();
    if (!found && connected &&
        mode == protocol::WaveformMode::OFDM_CHIRP &&
        !expect_full_ofdm_anchor_ &&
        sync_reject_streak_ >= kEscalateStreak) {
        std::lock_guard<std::mutex> lock(ring_.buffer_mutex_);
        expect_full_ofdm_anchor_ = true;
        sync_reject_streak_ = 0;
        LOG_MODEM(INFO,
            "[%s] §16.4 escalation: %llu light rejects at group boundary; "
            "arming full chirp+LTS re-anchor for sender RESEND",
            log_prefix_.c_str(),
            static_cast<unsigned long long>(kEscalateStreak));
    }

    return found;
}

// §7 C3 Phase 3b: the connected full-anchor light-LTS fallback, moved verbatim from the
// `if (!found && use_full_ofdm_anchor_search)` block of searchForSync's else branch. Same trace,
// detector call, control-threshold evaluation, streak update, weak-accept logs, and accept — same
// order → byte-identical; only the data_sync_accepted_callback_ fire stays in the decoder.
FullAnchorFallbackResult SyncController::detectStructureAnchor(
    IWaveform* waveform, const float* search_data, size_t search_len, bool connected,
    float known_cfo, double false_alarm_budget, SyncResult& sync_result) {
    FullAnchorFallbackResult result;
    // Half-duplex: while this station awaits the peer's tone-burst ACK the peer cannot be
    // sending OFDM, so no group start can be on the air. Same gate as the DATA fallback below
    // (the structure statistic is tone-immune by construction; the gate also saves the CPU).
    if (ack_listen_suppress_data_sync_ && connected) {
        return result;
    }
    SyncResult structure_result;
    if (waveform->detectFrameStructure(SampleSpan(search_data, search_len), structure_result,
                                       known_cfo, false_alarm_budget)) {
        result.found = true;
        sync_result = structure_result;
        LOG_MODEM(INFO,
                  "[%s] Group start acquired by frame structure (rho=%.2f, no chirp)",
                  log_prefix_.c_str(), sync_result.correlation);
    }
    return result;
}

FullAnchorFallbackResult SyncController::detectFullAnchorFallback(
    IWaveform* waveform, const float* search_data, size_t search_len, size_t search_start,
    size_t min_search, bool is_narrowband, bool connected, float corr_detect_threshold,
    float known_cfo, SyncResult& sync_result) {
    FullAnchorFallbackResult result;

    // ACK-LISTEN tone-lock guard (see the setter comment in the header). This fallback
    // is the exact path that ACCEPTED the peer's ACK tone at corr=0.94 during
    // full-anchor-wait (F74) — the tone scores above every threshold, so only an
    // unconditional gate works. The dual-chirp detectSync above this fallback stays
    // live (a tone cannot fake the up+down chirp pair), so a genuine full-anchor
    // frame is still acquired during the window.
    if (ack_listen_suppress_data_sync_ && connected) {
        if (!ack_listen_suppress_logged_) {
            ack_listen_suppress_logged_ = true;
            LOG_MODEM(INFO,
                      "[%s] ACK-LISTEN: full-anchor DATA fallback suppressed (tone "
                      "window; S&C tone-lock guard)",
                      log_prefix_.c_str());
        }
        return result;
    }

    // §16 Phase 5 instrumentation: detectSync failed to lock the descriptor chirp.
    // sync_result.correlation holds the peak dual-chirp correlation (max up/down) even on failure.
    // Logging it disambiguates a THRESHOLD miss (peak just under thr) from a NO-CHIRP-IN-WINDOW miss
    // (peak ≈ 0 → search window misaligned). ULTRA_S16_TRACE_WARM_WINDOW gates it.
    if (const char* t = std::getenv("ULTRA_S16_TRACE_WARM_WINDOW");
        t && std::atoi(t) != 0) {
        LOG_MODEM(INFO,
            "[%s] s16-phase5: detectSync MISS chirp_peak=%.3f (thr=%.2f) "
            "corr_pos=%zu total=%zu search_start=%zu min_search=%zu",
            log_prefix_.c_str(), sync_result.correlation,
            corr_detect_threshold, ring_.correlation_pos_, ring_.total_fed_,
            search_start, min_search);
    }
    SyncResult light_sync_result;
    // known_cfo is the tracked CFO (§7 C-CFO-1: passed in from the decoder's CFOTracker).
    const bool light_found = waveform->detectDataSync(
        SampleSpan(search_data, search_len),
        light_sync_result, known_cfo, corr_detect_threshold);
    // In connected OFDM the next frame type is unknown at acquisition time. Even when the data
    // profile is coherent QPSK/QAM, ACK/SACK/TURN controls use a hardened R1/4 control profile and
    // are validated by the downstream control-first LDPC parse. Do not apply the coherent-data sync
    // threshold to this full-anchor fallback, or real control frames in Good fading can be rejected
    // before the robust decoder can see them.
    const bool unknown_frame_uses_control_sync_threshold = false;
    const auto fallback_thresholds = signal_policy::lightSyncThresholds(
        unknown_frame_uses_control_sync_threshold, is_narrowband,
        connected, sync_reject_streak_);
    auto sync_decision = signal_policy::evaluateLightSyncCandidate(
        light_found, light_sync_result.correlation,
        unknown_frame_uses_control_sync_threshold, connected,
        sync_reject_streak_, fallback_thresholds);
    if (light_found && light_sync_result.correlation < fallback_thresholds.min_confidence) {
        if (sync_decision.weak_accept) {
            LOG_MODEM(INFO,
                      "[%s] Full-anchor wait fell back to weak DATA sync (corr=%.2f < %.2f, streak=%llu)",
                      log_prefix_.c_str(), light_sync_result.correlation,
                      fallback_thresholds.min_confidence,
                      static_cast<unsigned long long>(sync_reject_streak_));
        } else if (sync_decision.rejected) {
            LOG_MODEM(INFO,
                      "[%s] Full-anchor wait rejected DATA fallback (corr=%.2f < %.2f, streak=%llu)",
                      log_prefix_.c_str(), light_sync_result.correlation,
                      fallback_thresholds.min_confidence,
                      static_cast<unsigned long long>(sync_decision.next_reject_streak));
        }
    }
    sync_reject_streak_ = sync_decision.next_reject_streak;
    if (sync_decision.found) {
        result.found = true;
        sync_result = light_sync_result;
        result.used_full_anchor_fallback = true;
        LOG_MODEM(INFO,
                  "[%s] Full OFDM anchor not found; accepted connected DATA sync fallback (corr=%.2f)",
                  log_prefix_.c_str(), sync_result.correlation);
    }
    return result;
}

}  // namespace sync
}  // namespace ultra
