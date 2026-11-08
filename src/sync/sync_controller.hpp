#pragma once

// SyncController — the single owner of OFDM sync STATE and the acquisition DECISION.
//
// Target architecture for the sync-acquisition refactor (docs/SYNC_ACQUISITION_FIX_PLAN_2026_05_31.md
// §7, building on docs/SYNC_SUBSYSTEM_AUDIT_2026_05_29.md §9). It collapses the current sprawl —
// detectors in chirp_sync/waveforms, orchestration in streaming_sync_acquisition.cpp, 12 loose
// state members on StreamingDecoder, policy in signal_policy/arrival_policy, 6 entry points, one
// flag flipped in 11 places — into ONE object with a 3-state machine, two detectors it CALLS, and
// one acceptance rulebook.
//
// What it OWNS:        the SyncMode, the timing prediction, confidence, miss counters, last_cfo,
//                      the which-detector/which-window/accept-or-not decision, the acceptance rule.
// What it does NOT own: the DSP detectors (IWaveform::detectSync / detectDataSync, chirp_sync —
//                      it CALLS them) and the demod + LDPC decode (the loop runs those and reports
//                      the outcome back via reportFrameOutcome).
//
// The core fix lives here, expressed once: in WARM contiguous-data, acceptance is
// position + LDPC (the `tentative` + reportFrameOutcome pair), NOT an LTS-correlation gate. The
// 0.90/0.52/0.50 correlation thresholds apply only to COLD / RE_ACQUIRE. MC-DPSK needs no special
// case: supportsDataPreamble()==false ⇒ the WARM "light LTS" call falls back to the full chirp.
//
// MIGRATION STATUS (2026-05-31): the controller OWNS the sync state + the policy decisions
// (acceptLightSyncCandidate / planWarmSearch / the noteFrameArrival* + seedArrivalAfterDelay
// transition machine + the derived phase). The decode loop still calls these from
// StreamingDecoder::searchForSync (detect()/reportFrameOutcome() remain the future single-entry
// dispatch). The warm-handoff path is now the PRODUCTION DEFAULT — promoted past the
// ULTRA_S16_WARM_HANDOFF flag (removed 2026-05-31, validated on the AWGN floor + Good@12 + no-regress).

#include "waveform/waveform_interface.hpp"   // IWaveform, SyncResult, SampleSpan
#include "protocol/frame_v2.hpp"             // protocol::WaveformMode
#include "sync/frame_arrival_policy.hpp"     // WarmSyncPhase + warm-sync timing helpers
#include "sync/signal_policy.hpp"            // LightSyncThresholds + light-sync acceptance
#include "sync/sync_ring_buffer.hpp"         // SyncRingBuffer — the owned audio ring (refactor §7 C3)

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace ultra {
namespace sync {

// The three (and only three) acquisition states. Collapses the frame_arrival_policy::WarmSyncPhase
// {COLD,WARM,DEGRADED,RECOVERY} 4-state machine (DEGRADED→WARM, RECOVERY→RE_ACQUIRE) — derived by
// mode() from (warm_sync_active_, consecutive_sync_misses_). NOTE: expect_full_ofdm_anchor_ is a
// SEPARATE "next search uses the full chirp anchor" flag — it is set in HEALTHY per-group contexts
// (the BURST_HEADER descriptor re-arm), NOT only on sync loss, so it does NOT fold into RE_ACQUIRE.
enum class SyncMode {
    COLD,        // no timing lock — full chirp+LTS, wide window, strict threshold
    WARM,        // locked + predicting — group boundary re-anchors on the descriptor chirp;
                 // contiguous data accepted by position + LDPC (no correlation gate)
    RE_ACQUIRE,  // lost (LDPC failed N× at the predicted position) — force a full chirp next anchor
};

// One detect() tick's decision for the decode loop.
struct SyncDecision {
    bool   found     = false;  // a real lock happened (COLD / group-boundary chirp)
    bool   tentative = false;  // WARM contiguous: process at `pos`, let LDPC judge (NOT accepted yet)
    size_t pos       = 0;      // absolute sample position where the frame starts
    float  cfo       = 0.0f;   // carrier-frequency offset to apply
    SyncMode mode    = SyncMode::COLD;
};

// The connected-data ("light LTS") acceptance verdict for one detect tick.
// Returned by acceptLightSyncCandidate(): folds signal_policy::evaluateLightSyncCandidate,
// the ULTRA_S16_WARM_HANDOFF warm-override, and the WARM position-gating into one decision.
// When position_gated, the decoder maps position_gate_abs → sync_result.start_sample
// (= position_gate_abs - search_start), applies position_gate_cfo, and zeroes correlation.
struct LightSyncAcceptance {
    bool   found = false;             // accept this candidate as a frame start
    bool   position_gated = false;    // WARM position-gating fired (light-LTS corr at noise)
    size_t position_gate_abs = 0;     // absolute sample to process at (when position_gated)
    float  position_gate_cfo = 0.0f;  // known CFO to apply at the gated position
};

// The extracted search window for one searchForSync tick (§7 C3 Phase 3a). Produced by
// acquireSearchWindow() from the controller-owned ring: the slice of audio the decoder then runs
// detectors over, plus the warm-window geometry. ready=false ⇒ no search this tick (the caller
// returns); the ring-state side effects (correlation_pos_ advance, noise_floor_/search_floor_
// updates) already happened inside the call.
struct SearchWindowResult {
    bool   ready = false;
    std::vector<float> search_buffer;
    size_t search_start = 0;
    // Absolute identity of search_buffer[0], captured under the same ring lock as
    // the copy.  Consumers must not reconstruct this later from the modular index:
    // the audio writer may wrap the ring between the copy and detector completion.
    size_t search_start_abs = 0;
    size_t min_search = 0;
    bool   used_warm_timed_window = false;
    bool   used_warm_narrow_window = false;
    size_t warm_narrow_end_abs = 0;
    size_t warm_narrow_candidate_span_samples = 0;
    // Samples at the START of search_buffer that lie before the post-frame search floor
    // (the kChirpCfoPeakLeadSamples lead of a chirp search). LTS-type detectors must skip them.
    size_t pre_floor_lead_samples = 0;
};

// The full-anchor fallback verdict (§7 C3 Phase 3b). When a connected full-chirp re-anchor search
// misses the descriptor chirp, the controller tries a light-LTS DATA fallback; this reports whether
// that accepted (found) and whether it was the fallback path (used_full_anchor_fallback, which the
// decoder surfaces as sync_from_full_anchor_fallback_). On found, sync_result is overwritten with the
// fallback result; the decoder fires its data_sync_accepted_callback_.
struct FullAnchorFallbackResult {
    bool found = false;
    bool used_full_anchor_fallback = false;
};

class SyncController {
public:
    // §7 C3: the controller OWNS the shared audio ring. Sized at construction from the decoder's
    // buffer capacity (StreamingDecoder forwards its ctor arg as sync_controller_(capacity)); the
    // default keeps standalone/test construction (`SyncController sc;`) at the production 50 s buffer.
    explicit SyncController(size_t ring_capacity = SyncRingBuffer::kDefaultBufferSamples)
        : ring_(ring_capacity) {}

    // Reset to COLD for a new connection / mode change. `wf` is the active waveform whose detectors
    // we call; `is_coherent` selects the COLD/RE_ACQUIRE strict thresholds.
    void reset(protocol::WaveformMode mode, IWaveform* wf, bool is_coherent);

    // Per decode-loop tick: decide which detector/window and run it.
    //  COLD / group boundary : found=true only on a real chirp lock.
    //  WARM contiguous       : tentative=true at the predicted position (the loop demods there;
    //                          LDPC is the real accept/reject).
    SyncDecision detect(SampleSpan buffer, size_t buffer_len, size_t buffer_abs_start);

    // The loop calls this AFTER it demods + LDPC-decodes the frame at SyncDecision.pos.
    // This is the position+LDPC acceptance + the state transitions (advance prediction on ok;
    // count a miss and escalate WARM→RE_ACQUIRE after N consecutive failures).
    void reportFrameOutcome(bool ldpc_ok, size_t frame_end_abs);

    // A new burst group started (fresh descriptor anchor expected); seeds the WARM cadence.
    void noteGroupBoundary(size_t descriptor_end_abs, size_t expected_frame_gap_samples);

    // A burst group was DELIVERED as a unit (descriptor acquired + all frames demodulated — i.e.
    // warm sync WORKED — even if LDPC then failed the DATA and ARQ resends). Refreshes warm-sync to
    // HEALTHY (force WARM: misses=0 + active, confidence >=0.5). A clean group may honor its decoded
    // descriptor's NEXT_LIGHT announcement; a group requiring retransmission must instead arm a full
    // descriptor search because reactive resends are always full and the old announcement is stale.
    // The owner of these four warm-sync-prediction fields (§7 C4: moved verbatim from
    // streaming_burst_interleave.cpp so the decoder stops writing them directly).
    void noteGroupDelivered(uint32_t group_seq, bool retransmission_required);

    // --- arrival-tracking transition logic (§7.4 A2; moved verbatim from StreamingDecoder) ---
    // These own the warm-sync phase machine + cadence prediction + confidence. They fold into
    // detect()/reportFrameOutcome() in the detect-dispatch chunk; kept as a faithful trio for now.
    // Callers hold StreamingDecoder::buffer_mutex_ (the *Locked convention); these do not lock.
    //   resetFrameArrivalTracking : connection/mode reset → COLD, clears prediction + memory.
    //   noteFrameArrivalSuccess   : a frame decoded at (start,end) → advance prediction, WARM.
    //   noteFrameArrivalSyncMiss  : a predicted frame missed → decay confidence, escalate phase.
    void resetFrameArrivalTracking();
    void noteFrameArrivalSuccess(size_t frame_start_abs, size_t frame_end_abs);
    void noteFrameArrivalSyncMiss();

    // Seed the WARM cadence from a known local-TX turnaround (the predicted next frame
    // arrives `delay_samples` after `total_fed_abs`) — used when this station's own TX
    // tells us when the peer's reply group will land. Sibling to the noteFrameArrival*
    // trio; moved verbatim from StreamingDecoder::seedExpectedFrameArrivalAfterSamples (the
    // connected_/OFDM_CHIRP guard + lock stay in the decoder forwarder).
    void seedArrivalAfterDelay(size_t total_fed_abs, size_t delay_samples, float confidence);

    // The connected-data light-LTS acceptance decision (§7.4 chunk B; moved verbatim from
    // StreamingDecoder::searchForSync). Folds signal_policy::evaluateLightSyncCandidate, the
    // ULTRA_S16_WARM_HANDOFF warm-override, and the WARM position-gating into one verdict, and
    // updates the owned sync_reject_streak_. The decoder still runs the detector and builds the
    // search buffer; it hands the candidate (detector_found, correlation) + window geometry
    // (search_start, search_window_len) here, then applies the returned position gate to
    // sync_result. `connected`/`is_coherent` mirror the decoder's connected_ / coherent-data flag.
    LightSyncAcceptance acceptLightSyncCandidate(
        bool detector_found, float correlation, bool is_coherent, bool connected,
        float known_cfo, size_t search_start, size_t search_window_len,
        const signal_policy::LightSyncThresholds& thresholds);

    // The warm-window PLANNING decision (§7.4 chunk-B tail; moved verbatim from
    // StreamingDecoder::searchForSync). Computes the expected search anchor from the owned
    // warm-sync state and calls frame_arrival_policy::planWarmSearchWindow — returning the
    // window plan the decoder acts on (wait / activate / build buffer). The decoder still
    // owns the ring buffer: it derives oldest_abs / correlation_abs and does the actual
    // extraction. Caller holds StreamingDecoder::buffer_mutex_ (reads the same warm-sync
    // fields the noteFrameArrival* trio writes under that lock); this does not lock.
    frame_arrival_policy::WarmSearchWindowPlan planWarmSearch(
        bool use_light_search, size_t total_fed, size_t oldest_abs,
        bool search_floor_valid, size_t search_floor_abs, size_t correlation_abs,
        size_t symbol_samples, size_t correlation_step);

    // The lock-held search-window PRODUCTION (§7 C3 Phase 3a; moved verbatim from
    // StreamingDecoder::searchForSync). Locks the owned ring, runs planWarmSearch + the RMS gate +
    // the post-frame search-floor logic, advances correlation_pos_, and extracts the next slice the
    // decoder runs detectors over. Returns SearchWindowResult{ready=false} for the wait/skip ticks
    // (the ring side effects already applied). The decoder passes the inputs it derives from the
    // waveform/connection: use_light_search / connected_data_preamble / disconnected_mc_dpsk, the
    // initial min_search, data_symbol_samples, an audio-activity snapshot, and the step/threshold
    // constants. This LOCKS ring_.buffer_mutex_ internally (unlike the *Locked helpers).
    SearchWindowResult acquireSearchWindow(
        bool use_light_search, bool connected_data_preamble, bool disconnected_mc_dpsk,
        size_t min_search, size_t data_symbol_samples, bool audio_active,
        size_t correlation_step, float corr_noise_threshold);

    // The connected-data light-LTS DETECTION + acceptance + §16.4 re-anchor escalation (§7 C3
    // Phase 3b; moved verbatim from the `if (use_light_search)` branch of searchForSync). Runs the
    // passed waveform's detectDataSync, applies acceptLightSyncCandidate's verdict + the WARM
    // position-gate to sync_result, and — on a sustained reject streak — arms the full-chirp
    // re-anchor (owns expect_full_ofdm_anchor_ / sync_reject_streak_). The waveform is PASSED (the
    // decoder's current one — the controller's own member could be stale across dual-listen swaps).
    // Returns found; the decoder fires its data_sync_accepted_callback_ when found.
    bool detectConnectedLightSync(
        IWaveform* waveform, const float* search_data, size_t search_len, size_t search_start,
        bool is_coherent, bool connected, protocol::WaveformMode mode,
        const signal_policy::LightSyncThresholds& thresholds, float corr_detect_threshold,
        float known_cfo, SyncResult& sync_result);

    // The connected full-anchor light-LTS fallback (§7 C3 Phase 3b; moved verbatim from the
    // `if (!found && use_full_ofdm_anchor_search)` block of searchForSync's else branch). Runs after
    // a full-chirp re-anchor detectSync miss: tries detectDataSync, evaluates it with the control
    // (non-coherent) threshold, updates the owned sync_reject_streak_, and accepts a weak DATA sync
    // so real control frames in fading aren't rejected before the robust decoder sees them. The
    // waveform is PASSED; sync_result carries the failed-chirp correlation in (for the §16 phase-5
    // trace) and the accepted fallback out. The decoder fires its callback when found.
    // Search-window geometry (acquireSearchWindow): once the RMS presence probe at the cursor
    // passes, the extracted window starts kSearchBacktrackSamples BEFORE the cursor.
    static constexpr size_t kSearchBacktrackSamples = 9600;
    static constexpr size_t kRmsProbeSamples = 1000;
    // How far BEFORE the search floor a connected chirp/structure search may start: the dual
    // chirp's up-peak precedes the chirp start by kappa * CFO samples (kappa = fs / chirp_rate
    // = 48000 / 4800 = 10 samples per Hz) and ChirpSync accepts |CFO| <= 100 Hz -> 1000 samples.
    static constexpr size_t kChirpCfoPeakLeadSamples = 10 * 100;

    // Window for the connected group-start STRUCTURE search (ULTRA_BURST_ACQ=structure). The
    // first pass whose RMS probe sees a frame has the cursor in (x - probe, x - probe + step]
    // for a frame starting at x, so the frame can begin up to backtrack + probe samples into
    // the extracted window; the window must still hold the detector's whole template span
    // after that. (The next pass, one step later, then also holds it: two chances per frame.)
    static size_t structureSearchWindowSamples(size_t structure_span_samples) {
        return structure_span_samples + kSearchBacktrackSamples + kRmsProbeSamples;
    }

    // Connected group-start acquisition by frame STRUCTURE (no chirp): see
    // IWaveform::detectFrameStructure. Honours the half-duplex ACK-listen suppression.
    FullAnchorFallbackResult detectStructureAnchor(
        IWaveform* waveform, const float* search_data, size_t search_len, bool connected,
        float known_cfo, double false_alarm_budget, SyncResult& sync_result);

    FullAnchorFallbackResult detectFullAnchorFallback(
        IWaveform* waveform, const float* search_data, size_t search_len, size_t search_start,
        size_t min_search, bool is_narrowband, bool connected, float corr_detect_threshold,
        float known_cfo, SyncResult& sync_result);

    void setLogPrefix(const std::string& prefix) { log_prefix_ = prefix; }

    // The live 3-state acquisition mode (§7 target machine), DERIVED from the same
    // (warm_sync_active_, consecutive_sync_misses_) state as derivePhase() — the 4-state
    // WarmSyncPhase collapses COLD→COLD, (WARM|DEGRADED)→WARM, RECOVERY→RE_ACQUIRE. No stored
    // mode_ field: like the phase, it can never drift from the miss counter.
    SyncMode mode() const {
        switch (derivePhase()) {
            case frame_arrival_policy::WarmSyncPhase::COLD:     return SyncMode::COLD;
            case frame_arrival_policy::WarmSyncPhase::WARM:
            case frame_arrival_policy::WarmSyncPhase::DEGRADED: return SyncMode::WARM;
            case frame_arrival_policy::WarmSyncPhase::RECOVERY: return SyncMode::RE_ACQUIRE;
        }
        return SyncMode::COLD;
    }
    bool isWarm() const { return mode() == SyncMode::WARM; }

    // Phase-D prep: the 4-state WarmSyncPhase is provably a PURE FUNCTION of (warm_sync_active_,
    // consecutive_sync_misses_) — the transitions set it via phaseAfterSyncMiss(misses)/
    // phaseAfterSuccessfulFrame(), and active is cleared exactly when misses hits
    // kWarmSyncMissesBeforeRecovery. derivePhase() recomputes it; the equivalence
    // (stored == derivePhase()) is asserted after every transition and validated deterministically
    // by test_sync_controller_phase. The §7 collapse removes the stored enum and keeps this as the
    // basis for the 3-state SyncMode (phase==WARM ⟺ active && misses<kWarmSyncMissesBeforeDegraded;
    // DEGRADED wide-window ⟺ misses>=that; RECOVERY ⟺ !active && misses>=kWarmSyncMissesBeforeRecovery).
    frame_arrival_policy::WarmSyncPhase derivePhase() const {
        if (!warm_sync_active_) {
            return consecutive_sync_misses_ >= frame_arrival_policy::kWarmSyncMissesBeforeRecovery
                       ? frame_arrival_policy::WarmSyncPhase::RECOVERY
                       : frame_arrival_policy::WarmSyncPhase::COLD;
        }
        if (consecutive_sync_misses_ >= frame_arrival_policy::kWarmSyncMissesBeforeRecovery)
            return frame_arrival_policy::WarmSyncPhase::RECOVERY;
        if (consecutive_sync_misses_ >= frame_arrival_policy::kWarmSyncMissesBeforeDegraded)
            return frame_arrival_policy::WarmSyncPhase::DEGRADED;
        return frame_arrival_policy::WarmSyncPhase::WARM;
    }

    // Burst declared-z (§7.6): the single RX source of truth for "what LDPC lifting
    // size did this transfer's BURST_HEADER declare?". The per-frame extraction z is
    // DERIVED from (frame-class, declared-z) in the behavioral phase — never toggled.
    int activeBurstLiftingZ() const {
        return (have_burst_descriptor_ && last_burst_descriptor_.lifting_z == 81) ? 81 : 27;
    }

    // --- migration accessors (shell-move §7.5#1) ---------------------------------
    // Temporary getters/setters so StreamingDecoder can move its state in member-by-
    // member while the orchestration still lives there. Each folds into
    // detect()/reportFrameOutcome()/noteGroupBoundary() as that logic migrates.
    size_t expectedFrameGapSamples() const { return expected_frame_gap_samples_; }
    void setExpectedFrameGapSamples(size_t samples) { expected_frame_gap_samples_ = samples; }

    // Last-frame arrival memory (read-only; written only by noteFrameArrival* / seedArrivalAfterDelay
    // / resetFrameArrivalTracking). Accessors let the decoder build its FrameArrivalSnapshot and log
    // without touching the now-private fields.
    bool    lastFrameArrivalValid() const { return last_frame_arrival_valid_; }
    size_t  lastFrameStartSample() const { return last_frame_start_sample_; }
    size_t  lastFrameEndSample() const { return last_frame_end_sample_; }
    bool    lastFrameArrivalErrorValid() const { return last_frame_arrival_error_valid_; }
    int64_t lastFrameArrivalErrorSamples() const { return last_frame_arrival_error_samples_; }

    // Warm-sync prediction state (§7 C4: now PRIVATE — the decoder no longer writes these; it reads
    // them via these getters for diagnostics/snapshot, and mutates them only through the controller's
    // methods: noteGroupDelivered (force-WARM refresh), noteFrameArrival* (the cadence machine), and
    // clearRejectStreak (clear the COLD/RE_ACQUIRE light-sync reject counter on a fresh lock/reset)).
    uint64_t syncRejectStreak() const { return sync_reject_streak_; }
    int      consecutiveSyncMisses() const { return consecutive_sync_misses_; }
    float    frameArrivalConfidence() const { return frame_arrival_confidence_; }
    bool     warmSyncActive() const { return warm_sync_active_; }
    void     clearRejectStreak() { sync_reject_streak_ = 0; }

    // ACK-LISTEN tone-lock guard (ULTRA_ACKLISTEN_SUPPRESS_OFDM, 2026-07-05).
    // While the sender's tone-burst ACK monitor is armed, the ONLY signal the peer can
    // emit is the 4-FSK tone ACK (half-duplex: it is the peer's ACK turn). The tone's
    // periodic segments (carrier lead-in + repeated FSK symbols) score ~0.9+ on the
    // Schmidl-Cox metric while the LTS matched filter stays ~0.1 (sc-high/mf-low), so
    // the warm DATA-sync detectors false-lock on the peer's ACK, decode garbage, and
    // the ensuing re-search races the tone monitor for the same samples — the measured
    // cause of missed ACKs / RTO spirals (F73/F74; previously misattributed to
    // "self-echo": OTASim excludes self-audio by construction (mixer.cpp:40) and rig
    // capture is stopped during TX (app.cpp:3360-3364), so a true echo was never
    // physically possible on either bench).
    // While set, BOTH warm data-sync acceptance paths (detectConnectedLightSync,
    // detectFullAnchorFallback) return not-found unconditionally — a threshold cannot
    // gate this (the tone scores 0.94, above every decayed threshold). The full
    // dual-chirp path (detectSync) stays live: a tone cannot fake an up+down chirp
    // pair at the exact 28800-sample gap, so a real full-anchor control frame (e.g. a
    // legacy MODE_CHANGE_ACK) is still acquired during the window. Not-found leaves
    // reject streaks / §16.4 escalation untouched (a suppressed window is not
    // acquisition failure evidence). Set per search pass by the decoder from
    // (knob && connected && monitor.isArmed()); auto-clears when the ACK decodes
    // (monitor disarm) or the window expires — it can never wedge the search.
    void setAckListenSuppressDataSync(bool v) {
        if (v != ack_listen_suppress_data_sync_) ack_listen_suppress_logged_ = false;
        ack_listen_suppress_data_sync_ = v;
    }
    bool ackListenSuppressDataSync() const { return ack_listen_suppress_data_sync_; }

    // --- TRANSITIONAL PUBLIC shell-move state (refactor §7.5#1) -------------------
    // These were StreamingDecoder members; relocated here verbatim so the (still-
    // external) orchestration reads/writes them as `sync_controller_.<name>`. They
    // are public ONLY during the shell-move and get re-privatized behind
    // detect()/reportFrameOutcome()/noteGroupBoundary() in the behavioral phase.
    // KEEP names identical to the old StreamingDecoder members (mechanical move).
    //
    // §7 C3 Phase 2: the controller now OWNS the shared 48 kHz audio ring (producer =
    // StreamingDecoder::feedAudio, consumers = searchForSync + the decode path). Transitional-public —
    // the decoder still reaches buffer/cursors/floor/helpers as sync_controller_.ring_.X until
    // detect() absorbs the search loop (Phase 3). Constructed first (the ctor sizes it from capacity).
    SyncRingBuffer ring_;

    // (§7 C4: sync_reject_streak_ / frame_arrival_confidence_ / consecutive_sync_misses_ /
    // warm_sync_active_ moved to PRIVATE below — the decoder no longer writes them.)
    size_t   next_expected_frame_sample_ = 0;        // predicted next-frame absolute sample
    bool     next_expected_frame_sample_valid_ = false;
    size_t   expected_frame_gap_samples_ = 0;        // cadence gap (§1.2 never-set bug)
    bool     expect_full_ofdm_anchor_ = false;       // force a full chirp on the next anchor
    // BUG-DECODE-BACKLOG load-shed applies ONLY to real-time audio (GUI/rig/
    // OTASim). Batch decode (tests, file tools) legitimately feeds faster than
    // real time and must never shed. Default OFF; the production engine opts in.
    bool     real_time_audio_ = false;
                                                     // (the 11-flip flag; toggled by the burst decode path)
    // #69 anchor-skip: the LAST decoded BURST_HEADER's BURST_FLAG_NEXT_LIGHT_ANCHOR — the sender's
    // announcement that the NEXT group's descriptor is light (chirp-less). Set by the decoder at
    // BURST_HEADER parse (setNextGroupLightAnchor); read in noteGroupDelivered to arm the right
    // search type for the next group (full-search chirp groups, light-search skip groups — no
    // grinding). Default false → expect full chirp (safe; also the K=1 / dropped-descriptor case).
    bool     next_group_light_anchor_ = false;
    void     setNextGroupLightAnchor(bool light) { next_group_light_anchor_ = light; }

    // §7 Phase-D collapse (2026-05-31): the stored 4-state warm_sync_phase_ field is GONE.
    // The phase is now DERIVED on demand from (warm_sync_active_, consecutive_sync_misses_) via
    // derivePhase() — proven equivalent (test_sync_controller_phase + Good@12 0-mismatch run), so
    // it can never drift from the miss counter. The decoder reads it via derivePhase().
    // (last-frame arrival memory is now PRIVATE — see below — read via the lastFrame* accessors.)
    // (§7 C-CFO-1: the CFO acquisition state `last_cfo_` was relocated OUT of here into the decoder's
    // sync::CFOTracker. The detect* methods take the known CFO as a param now; the chirp seed +
    // pilot feedback store route through CFOTracker.)

    // Burst z-state (§7.6): the transfer's declared LDPC-lifting descriptor. Latch
    // PERSISTS across the transfer (a fade-lost descriptor still decodes at the
    // declared z); reset() → COLD clears it. The per-frame extraction z is DERIVED
    // from (frame-class, this) in the behavioral phase, not a standalone toggle.
    bool have_burst_descriptor_ = false;
    protocol::v2::ControlFrame::BurstHeaderInfo last_burst_descriptor_{};

private:
    // Warm-sync prediction state (§7 C4 RE-PRIVATIZED: the decoder no longer writes these — it reads
    // them via syncRejectStreak()/consecutiveSyncMisses()/frameArrivalConfidence()/warmSyncActive(),
    // and mutates them only through noteGroupDelivered / the noteFrameArrival* cadence machine /
    // clearRejectStreak). The controller's own methods + derivePhase() use them directly.
    uint64_t sync_reject_streak_ = 0;        // consecutive COLD/RE_ACQUIRE light-sync rejects
    float    frame_arrival_confidence_ = 0.0f;
    int      consecutive_sync_misses_ = 0;
    bool     warm_sync_active_ = false;      // in the warm (locked+predicting) regime

    // ACK-LISTEN tone-lock guard state (see setAckListenSuppressDataSync above).
    bool ack_listen_suppress_data_sync_ = false;
    bool ack_listen_suppress_logged_ = false;    // once-per-window log throttle

    // Last-frame arrival memory (RE-PRIVATIZED §7.4: written only by noteFrameArrival* /
    // seedArrivalAfterDelay / resetFrameArrivalTracking; read by the decoder via the lastFrame*
    // accessors). NOT Phase-D-collapsing — this is genuine arrival state the controller keeps.
    bool     last_frame_arrival_valid_ = false;
    size_t   last_frame_start_sample_ = 0;
    size_t   last_frame_end_sample_ = 0;
    bool     last_frame_arrival_error_valid_ = false;
    int64_t  last_frame_arrival_error_samples_ = 0;

    // --- migrated from StreamingDecoder (audit §1.2) — the single home for this state ---
    // (C2: the stored SyncMode mode_ field was removed — mode() derives it, like the phase.)
    protocol::WaveformMode waveform_mode_ = protocol::WaveformMode::OFDM_CHIRP;
    IWaveform* waveform_ = nullptr;        // borrowed; detectors live here (NOT owned)
    bool  is_coherent_ = false;

    // WARM→RE_ACQUIRE escalation threshold (consecutive predicted-position LDPC failures).
    static constexpr int kReacquireAfterMisses = 2;

    // Log prefix mirrored from StreamingDecoder (e.g. "[BRAVO]") so warm-sync log lines keep
    // their station tag now that the transition logic emits them from here.
    std::string log_prefix_ = "StreamingDecoder";
};

}  // namespace sync
}  // namespace ultra
