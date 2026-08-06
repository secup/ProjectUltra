// Connection frame handlers
// Split from connection.cpp for maintainability

#include "connection.hpp"
#include "connection_policy.hpp"
#include "waveform_selection.hpp"  // Shared waveform/rate selection algorithm
#include "ultra/logging.hpp"

#include <algorithm>
#include <cstdint>

namespace ultra {
namespace protocol {

// Recommend data mode based on SNR, fading, and the NEGOTIATED waveform
// Uses shared recommendDataMode() algorithm from waveform_selection.hpp
// IMPORTANT: waveform should be the negotiated/forced waveform, NOT auto-selected
static void recommendDataModeForWaveform(float snr_db, SNRSource snr_source,
                                          float fading_index,
                                          WaveformMode waveform,
                                          Modulation& mod, CodeRate& rate) {
    // Use shared algorithm for modulation and rate selection
    recommendDataMode(snr_db, waveform, mod, rate, fading_index);

    // SNR_sel: the caller passes the #58 basis-CORRECTED selection value here (not the
    // raw measurement) — label it so rig notes don't read it as measured SNR.
    LOG_MODEM(INFO, "recommendDataModeForWaveform: SNR_sel=%.1f (%s), fading=%.2f, waveform=%s -> %s %s",
              snr_db, snrSourceToString(snr_source), fading_index,
              waveformModeToString(waveform), modulationToString(mod), codeRateToString(rate));
}

static bool commonSupportsWaveform(uint8_t local_caps, uint8_t remote_caps, WaveformMode mode) {
    const uint8_t bit = connection_policy::modeToCapabilityBit(mode);
    return bit != 0 && (local_caps & bit) != 0 && (remote_caps & bit) != 0;
}

// =============================================================================
// PING/PONG HANDLING
// =============================================================================

void Connection::onPongReceived() {
    if (state_ != ConnectionState::PROBING) {
        // Not in probing state - this might be an incoming ping from someone else
        // Notify via ping_received callback (they're calling us)
        if (state_ == ConnectionState::DISCONNECTED && on_ping_received_) {
            LOG_MODEM(INFO,
                      "Connection: Received incoming PING while disconnected, firing PONG TX callback");
            on_ping_received_();
        }
        return;
    }

    // We were probing and got a PONG - remote station exists!
    LOG_MODEM(INFO, "Connection: PONG received, sending full CONNECT");
    sendFullConnect();
}

void Connection::sendFullConnect() {
    // Transition to CONNECTING and send full CONNECT frame
    state_ = ConnectionState::CONNECTING;
    connect_retry_count_ = 0;
    timeout_remaining_ms_ = connectRetryIntervalMs();

    // Notify about state change (PROBING -> CONNECTING)
    if (on_state_changed_) {
        on_state_changed_(ConnectionState::CONNECTING, remote_call_);
    }

    const EnvironmentForcedDataProfile environment_force =
        forcedDataProfileFromEnvironment();
    if (environment_force.malformed) {
        LOG_MODEM(ERROR,
                  "Connection: malformed ULTRA_FORCE_DATA_MOD/RATE profile; "
                  "ignoring the environment override as one unit");
    }
    outbound_forced_modulation_ = config_.forced_modulation;
    outbound_forced_code_rate_ = config_.forced_code_rate;
    if (environment_force.forced()) {
        if (environment_force.modulation != Modulation::AUTO) {
            outbound_forced_modulation_ = environment_force.modulation;
        }
        if (environment_force.code_rate != CodeRate::AUTO) {
            outbound_forced_code_rate_ = environment_force.code_rate;
        }
    }

    auto connect_frame = v2::ConnectFrame::makeConnect(local_call_, remote_call_,
                                                        config_.mode_capabilities,
                                                        static_cast<uint8_t>(config_.preferred_mode),
                                                        static_cast<uint8_t>(outbound_forced_modulation_),
                                                        static_cast<uint8_t>(outbound_forced_code_rate_),
                                                        config_.forced_cw_count);
    Bytes connect_data = connect_frame.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT via %s (%zu bytes, forced_mod=%d, forced_rate=%d, forced_cw=%d)",
              waveformModeToString(connect_waveform_), connect_data.size(),
              static_cast<int>(outbound_forced_modulation_),
              static_cast<int>(outbound_forced_code_rate_),
              static_cast<int>(config_.forced_cw_count));
    transmitFrame(connect_data);
}

void Connection::cancelOutboundProbe() {
    LOG_MODEM(INFO, "[CONNECT] Outbound probe canceled by inbound CONNECT");
    ping_retry_count_ = 0;
    timeout_remaining_ms_ = 0;
    remote_call_.clear();
    remote_hash_ = 0;
    pending_remote_call_.clear();
    pending_remote_hash_ = 0;
    state_ = ConnectionState::DISCONNECTED;
}

void Connection::cancelOutboundConnect() {
    LOG_MODEM(INFO, "[CONNECT] Outbound CONNECT canceled by inbound CONNECT");
    connect_retry_count_ = 0;
    timeout_remaining_ms_ = 0;
    remote_call_.clear();
    remote_hash_ = 0;
    pending_remote_call_.clear();
    pending_remote_hash_ = 0;
    state_ = ConnectionState::DISCONNECTED;
}

// =============================================================================
// CONNECT FRAME HANDLERS
// =============================================================================

void Connection::handleConnect(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ == ConnectionState::PROBING) {
        LOG_MODEM(INFO,
                  "Connection: Inbound CONNECT during PROBING - accepting "
                  "(call collision; aborting local probe)");
        cancelOutboundProbe();
    } else if (state_ == ConnectionState::CONNECTING) {
        if (src_call.empty()) {
            LOG_MODEM(WARN, "Connection: Simultaneous CONNECT without callsign, rejecting");
            auto nak = v2::ConnectFrame::makeConnectNakByHash(local_call_, frame.src_hash);
            transmitFrame(nak.serialize());
            return;
        }

        if (local_call_ < src_call) {
            LOG_MODEM(INFO,
                      "Connection: Simultaneous CONNECT, local %s < remote %s, "
                      "keeping our connect attempt",
                      local_call_.c_str(), src_call.c_str());
            return;
        }

        if (local_call_ > src_call) {
            LOG_MODEM(INFO,
                      "Connection: Simultaneous CONNECT, local %s > remote %s, "
                      "abandoning local and accepting inbound",
                      local_call_.c_str(), src_call.c_str());
            cancelOutboundConnect();
        } else {
            LOG_MODEM(WARN,
                      "Connection: Simultaneous CONNECT with identical callsign %s, rejecting",
                      local_call_.c_str());
            auto nak = v2::ConnectFrame::makeConnectNakByHash(local_call_, frame.src_hash);
            transmitFrame(nak.serialize());
            return;
        }
    } else if (state_ == ConnectionState::CONNECTED && !is_initiator_) {
        const bool same_peer =
            (!src_call.empty() && src_call == remote_call_) ||
            (frame.src_hash != 0 && frame.src_hash == v2::hashCallsign(remote_call_)) ||
            (remote_hash_ != 0 && frame.src_hash == remote_hash_);
        if (same_peer && !connect_ack_frame_.empty()) {
            LOG_MODEM(WARN,
                      "Connection: Duplicate CONNECT from %s while responder ACK unconfirmed; re-sending cached CONNECT_ACK",
                      remote_call_.c_str());
            transmitFrame(connect_ack_frame_);
            return;
        }

        LOG_MODEM(WARN, "Connection: Rejecting CONNECT (busy, state=%s)",
                  connectionStateToString(state_));
        auto nak = v2::ConnectFrame::makeConnectNakByHash(local_call_, frame.src_hash);
        transmitFrame(nak.serialize());
        return;
    } else if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Rejecting CONNECT (busy, state=%s)",
                  connectionStateToString(state_));
        auto nak = v2::ConnectFrame::makeConnectNakByHash(local_call_, frame.src_hash);
        transmitFrame(nak.serialize());
        return;
    }

    // BUG-MC-RETRY-SPURIOUS fix 3: a genuinely NEW inbound connection is being
    // established (every duplicate/busy/collision case returned above). Clear the
    // MODE_CHANGE dedup tuple so a restarted peer's reused seq can't be mistaken
    // for a duplicate of the previous session. Placed here (not enterConnected —
    // connection.cpp is owned by the main session) so it also covers the manual
    // acceptCall() establishment path, which enters connected in connection.cpp.
    last_applied_mode_change_valid_ = false;

    // Get capabilities from ConnectFrame
    uint8_t remote_caps = frame.mode_capabilities;
    WaveformMode remote_pref = static_cast<WaveformMode>(frame.negotiated_mode);

    LOG_MODEM(INFO, "Connection: Incoming call from %s (caps=0x%02X, pref=%s)",
              src_call.c_str(), remote_caps, waveformModeToString(remote_pref));
    stats_.connects_received++;

    remote_capabilities_ = remote_caps;
    remote_preferred_ = remote_pref;
    setPhyMaskV1Negotiated(hasPhyMaskV1Capability(config_.mode_capabilities) &&
                           hasPhyMaskV1Capability(remote_caps));

    // Default channel estimates for mode selection
    float snr_db = 15.0f;
    float delay_spread_ms = 1.0f;
    float doppler_spread_hz = 0.5f;

    if (config_.auto_accept) {
        // Store callsign if known, otherwise use placeholder with hash for display
        remote_call_ = src_call.empty() ? "REMOTE" : src_call;
        remote_hash_ = frame.src_hash;  // Store hash for routing

        // Use measured SNR from modem (set via setMeasuredSNR). #58 increment 3:
        // under ULTRA_CONNECT_SNR_POOL this is the connect-SNR-pool aggregate
        // (clustered dB-mean of this handshake's data-aided readings — CONNECT +
        // retry decodes), else exactly the raw last-write-wins scalar. The same
        // value feeds the selection, the CW pick, the CONNECT_ACK wire byte and the
        // logs below, so all views of the pick stay consistent.
        snr_db = rateSelectionSnrDb();
        // #58 increment 4: the entry-pick FADING is pooled exactly like the SNR
        // (single-frame fading at Watterson Good scatters 0.24-0.74 around the
        // 0.65 Good/Moderate boundary — one 0.66 CONNECT reading mis-classed a
        // dial-20 Good channel Moderate -> QPSK R1/4 entry). Knob-off (or a pool
        // with no fading-carrying reading) this IS fading_index_, byte-identical.
        // The pool can only average what it has: a clean first-try handshake gives
        // N_eff=1, so ALSO shrink the classification input by the standard error
        // (entryClassificationFadingIndex — see the sigma provenance there). Only
        // when the pool knob is on; raw kept in fading_index_ for logs/wire.
        const float entry_fading_raw = rateSelectionFadingIndex();
        const float entry_fading =
            connection_policy::connectSnrPoolEnabled()
                ? connection_policy::entryClassificationFadingIndex(
                      entry_fading_raw,
                      connect_snr_pool_.effectiveCount(
                          connectSnrPoolTcMs(), /*handshake_only=*/true,
                          /*max_age_ms=*/UINT64_MAX))
                : entry_fading_raw;
        const SNRSource snr_source = measured_snr_source_;
        const bool rate_selection_snr_valid = measured_snr_valid_;
        // #58: SELECTION uses the basis-corrected value (fade-effective reading vs
        // dial-calibrated floors/anchors, connection_policy::connectSelectionSnrDb);
        // the +5 fade basis is applied exactly ONCE, here, downstream of the pool
        // aggregation (the pool population is the SAME one the basis was calibrated
        // on, so it composes unchanged).
        const bool selection_snr_data_aided = rateSelectionSnrDataAided();
        const float selection_snr_db =
            connection_policy::connectSelectionSnrDb(snr_db, entry_fading,
                                                     selection_snr_data_aided,
                                                     physical_channel_mean_db_,
                                                     physical_channel_n_);

        const Modulation forced_mod = static_cast<Modulation>(frame.initial_modulation);
        const CodeRate forced_rate = static_cast<CodeRate>(frame.initial_code_rate);
        const EnvironmentForcedDataProfile environment_force =
            forcedDataProfileFromEnvironment();
        if (environment_force.malformed) {
            LOG_MODEM(ERROR,
                      "Connection: malformed ULTRA_FORCE_DATA_MOD/RATE profile on "
                      "responder; ignoring the environment override as one unit");
        }
        const Modulation environment_forced_mod = environment_force.modulation;
        const CodeRate environment_forced_rate = environment_force.code_rate;
        const bool environment_forced_rung = environment_force.forced();
        const bool forced_profile =
            forced_mod != Modulation::AUTO || forced_rate != CodeRate::AUTO ||
            frame.data_frame_cw_count != 0 || environment_forced_rung;
        const bool forced_waveform =
            remote_pref != WaveformMode::AUTO ||
            config_.preferred_mode != WaveformMode::AUTO ||
            narrowband_override_ != WaveformMode::AUTO;
        // Rate/channel decisions use the Doppler-coherence-refined fading index: equals the
        // pooled entry fading until enough OFDM data has pooled (so this CONNECT-time pick,
        // with no prior OFDM data, honors the pool), then the Good/Moderate coherence verdict.
        const float rate_fading = connection_policy::coherenceAdjustedFadingIndex(
            entry_fading, coherence_score_, coherence_valid_);
        const auto selected_rung = rate_selection_snr_valid
            ? connection_policy::selectLadderRung(selection_snr_db, rate_fading)
            : connection_policy::ladderRungForId(LadderRungId::ROBUST);
        const bool ladder_selected =
            rate_selection_snr_valid &&
            !forced_profile && !forced_waveform &&
            commonSupportsWaveform(config_.mode_capabilities, remote_caps,
                                   selected_rung.waveform);

        // #58 increment 3 (ULTRA_CONNECT_PICK_DEFER; requires ULTRA_CONNECT_SNR_POOL):
        // when the pick rests on ONE effective fade sample AND would land sub-OFDM on a
        // fading channel (the 15-40x cost-asymmetry zone — W3: one 3.9 dB trough
        // reading bought a ~90 bps DBPSK session on a dial-20 channel), buy a second
        // decorrelated sample instead of inflating the value: withhold CONNECT_ACK ONCE
        // and let the initiator's EXISTING CONNECT retransmit (10-attempt budget,
        // spacing >= ~2 Tc on Good) re-enter this handler with N_eff=2. Auto-accept
        // only — a manual acceptCall is an operator override and never defers. Fully
        // wire-compatible: to the initiator a deferred CONNECT is indistinguishable
        // from a decode failure; worst case (no retry) equals today's lost-CONNECT.
        if (connection_policy::connectSnrPoolEnabled() &&
            connection_policy::connectPickDeferEnabled() &&
            ladder_selected &&
            connection_policy::shouldDeferConnectPick(
                connect_snr_pool_.effectiveCount(connectSnrPoolTcMs(),
                                                 /*handshake_only=*/true,
                                                 /*max_age_ms=*/UINT64_MAX),
                entry_fading, selected_rung.waveform,
                connect_pick_deferred_once_)) {
            connect_pick_deferred_once_ = true;
            LOG_MODEM(INFO,
                      "Connection: CONNECT pick DEFERRED once (N_eff=1, fading=%.2f, "
                      "pick would land %s at sel=%.1f) -> withholding CONNECT_ACK; the "
                      "initiator's CONNECT retry supplies a decorrelated second reading",
                      entry_fading, selected_rung.name, selection_snr_db);
            return;
        }

        if (ladder_selected) {
            negotiated_mode_ = selected_rung.waveform;
            LOG_MODEM(INFO,
                      "Connection: Adaptive ladder selected %s (SNR=%.1f sel=%.1f (%s), fading=%.2f %s (raw %.2f))",
                      selected_rung.name, snr_db, selection_snr_db,
                      snrSourceToString(snr_source), entry_fading,
                      connection_policy::fadingLabel(entry_fading), fading_index_);
        } else {
            // Forced presets and explicit waveform preferences keep legacy
            // negotiation semantics.
            negotiated_mode_ = negotiateMode(remote_caps, remote_pref);
        }

        LOG_MODEM(INFO, "Connection: Negotiated waveform mode: %s",
                  waveformModeToString(negotiated_mode_));

        // We are the responder - we received CONNECT and are sending CONNECT_ACK
        is_initiator_ = false;
        handshake_confirmed_ = false;  // Responder waits for first frame to confirm
        responder_handshake_wait_ms_ = responderHandshakeFailSafeMs();

        Modulation rec_mod;
        CodeRate rec_rate;

        // If waveform is AUTO, select based on SNR/fading first
        if (negotiated_mode_ == WaveformMode::AUTO) {
            auto rec = recommendWaveformAndRate(selection_snr_db, rate_fading);
            negotiated_mode_ = rec.waveform;
            LOG_MODEM(INFO, "Connection: Auto-selected waveform %s based on SNR=%.1f (%s), fading=%.2f",
                      waveformModeToString(negotiated_mode_), snr_db,
                      snrSourceToString(snr_source), entry_fading);
        }

        // Get recommended mode based on SNR, fading AND the negotiated waveform
        // This ensures MC-DPSK uses R1/4, OFDM uses appropriate rate, etc.
        recommendDataModeForWaveform(selection_snr_db, snr_source, rate_fading,
                                     negotiated_mode_, rec_mod, rec_rate);

        LadderRungId rung_id = LadderRungId::UNKNOWN;
        if (ladder_selected) {
            rung_id = selected_rung.id;
            if (selected_rung.waveform == WaveformMode::MC_DPSK) {
                rec_mod = selected_rung.modulation;
                rec_rate = selected_rung.code_rate;
            }
        }

        // Bootstrap safety: the connect-time reading can overestimate first OFDM frame
        // quality (historically the chirp snapshot; since #58 it is the data-aided
        // fade-averaged estimate). Start one step more robust when channel is
        // borderline; ULTRA_ENTRY_CAP_R34 (default OFF) lets a data-aided reading that
        // clears the R3/4 anchor by >= 1 sigma enter at R3/4 (waveform_selection.hpp).
        if (isOFDMMode(negotiated_mode_)) {
            CodeRate capped = capInitialOFDMRate(selection_snr_db, rate_fading, rec_rate, rec_mod,
                                                 selection_snr_data_aided);
            if (capped != rec_rate) {
                LOG_MODEM(INFO, "Connection: Bootstrap cap %s -> %s for initial OFDM setup (SNR=%.1f (%s), fading=%.2f)",
                          codeRateToString(rec_rate), codeRateToString(capped), snr_db,
                          snrSourceToString(snr_source), entry_fading);
                rec_rate = capped;
            }
        }

        // ── USABLE-DOMAIN ENTRY CAP (ULTRA_ENTRY_EVM_CAP, default-off, 2026-07-25) ────
        // Rig-measured failure (MPM@20): the responder measured an HONEST data-aided
        // SNR of 8.3 dB (mcdpsk_in_band) and still entered at QPSK R3/4, because
        // connectSelectionSnrDb -> dialEquivalentSnrDb converts that usable reading into
        // a ~17 dB DIAL-EQUIVALENT (and the physical entry cap did not bite: the physical
        // meter also read 17.7, the same ~9 dB hardware-loss gap the +8.70 anchor offset
        // papers over). QPSK R3/4 needs ~12.5 dB USABLE, so the link cratered 5 groups
        // (0/6,0/6,0/6,0/8,0/8) and burned ~136 s before the ladder walked down to R1/4.
        //
        // The fix uses the anchor table that is already in USABLE units: the EVM-usable
        // rung floors (evmUsableFloorDbForRung, measured on the OTASim AWGN sweep). The
        // connect-time data-aided reading is ITSELF a usable-domain measurement (the
        // MC-DPSK differential-EVM estimator — the same family as the OFDM decision-
        // directed EVM), so it can be compared to those floors directly, with no dial
        // conversion and no bench constant.
        //
        // CAP ONLY, and only on a DATA-AIDED reading: it can lower the entry rung, never
        // raise it (a training-snapshot reading fade-crest OVER-reads, so it must never
        // license a higher rung). Worst case is a conservative entry the ladder climbs
        // out of in a few clean groups — vastly cheaper than 136 s of craters.
        if (isOFDMMode(negotiated_mode_) && selection_snr_data_aided &&
            entryEvmCapEnabled()) {
            const uint8_t want = coherentRungIndexFor(rec_mod, rec_rate);
            if (want != kRungIdxNone) {
                uint8_t cap = highestRungSupportedByEvm(snr_db, kEvmDemoteHoldMarginDb);
                const uint8_t snapped = snapRungIndexDownToEnabled(cap);
                cap = (snapped != kRungIdxNone) ? snapped : kRungIdxQpskR14;
                if (cap < want) {
                    const CoherentPick capped_pick = coherentRungFromIndex(cap);
                    LOG_MODEM(INFO,
                              "Connection: ENTRY-EVM cap %s %s -> %s %s "
                              "(usable data-aided SNR=%.1f dB < rung floor %.1f dB; "
                              "selection basis was %.1f dB dial-equivalent)",
                              modulationToString(rec_mod), codeRateToString(rec_rate),
                              modulationToString(capped_pick.mod),
                              codeRateToString(capped_pick.rate),
                              snr_db, evmUsableFloorDbForRung(want), selection_snr_db);
                    rec_mod = capped_pick.mod;
                    rec_rate = capped_pick.rate;
                }
            }
        }

        // ULTRA_ENTRY_QAM16_SNR (experiment): start AT 16QAM R2/3 on a strong Good-class
        // connect instead of QPSK-and-climb (the fade-riding strategy). AFTER the bootstrap
        // cap (so it overrides the QPSK R2/3 pin) and BEFORE forced overrides (so an operator
        // force still wins). The initiator applies whatever we stamp into CONNECT_ACK.
        if (isOFDMMode(negotiated_mode_) &&
            entryQam16Promote(selection_snr_db, rate_fading, rec_mod,
                              selection_snr_data_aided)) {
            LOG_MODEM(INFO,
                      "Connection: ENTRY-QAM16 promote %s %s -> 16QAM R2/3 (data-aided SNR=%.1f, fading=%.2f)",
                      modulationToString(rec_mod), codeRateToString(rec_rate),
                      selection_snr_db, rate_fading);
            rec_mod = Modulation::QAM16;
            rec_rate = CodeRate::R2_3;
        }

        // 2026-05-28: ULTRA_MAX_OFDM_RATE responder-side bootstrap cap.
        if (const char* env = std::getenv("ULTRA_MAX_OFDM_RATE")) {
            const std::string s(env);
            const CodeRate cap = (s == "R1_2" || s == "r1_2") ? CodeRate::R1_2
                               : (s == "R2_3" || s == "r2_3") ? CodeRate::R2_3
                               : (s == "R3_4" || s == "r3_4") ? CodeRate::R3_4
                               : CodeRate::AUTO;  // anything else = no cap (AUTO sentinel)
            if (cap != CodeRate::AUTO && rec_rate > cap) {
                LOG_MODEM(INFO, "Connection: ULTRA_MAX_OFDM_RATE responder cap %s -> %s",
                          codeRateToString(rec_rate), codeRateToString(cap));
                rec_rate = cap;
            }
        }

        // Exact environment forces have the same final precedence as the serialized
        // CONNECT fields below.  ULTRA_MAX_OFDM_RATE and the entry safety policies are
        // automatic caps; they cannot rewrite an explicit measurement rung.
        if (environment_forced_mod != Modulation::AUTO) {
            rec_mod = environment_forced_mod;
        }
        if (environment_forced_rate != CodeRate::AUTO) {
            rec_rate = environment_forced_rate;
        }

        // Keep the automatic connect verdict as the latent controller's prior;
        // the conservative first-group rung below is only what goes on air.
        const uint8_t automatic_latent_seed_rung =
            (isOFDMMode(negotiated_mode_) && rxRateAuthorityEnabled())
                ? coherentRungIndexFor(rec_mod, rec_rate)
                : kRungIdxNone;

        // Five real MPG@20 transfers disproved the former R3/4 cap: every first
        // group was incomplete (11/25 frames total) despite healthy acquisition,
        // while QPSK R1/2 subsequently ran 24/24 groups clean.  The connect sample
        // is MC-DPSK-domain evidence, not a coherent-OFDM payload measurement.
        const auto authority_entry = capRxAuthorityInitialRung(
            rec_mod, rec_rate, rxRateAuthorityEnabled(), environment_forced_rung);
        if (authority_entry.mod != rec_mod || authority_entry.rate != rec_rate) {
            LOG_MODEM(INFO,
                      "Connection: RX-AUTHORITY measured-entry probe %s %s -> QPSK R1/2 "
                      "(first coherent group owns the next decision)",
                      modulationToString(rec_mod), codeRateToString(rec_rate));
            rec_mod = authority_entry.mod;
            rec_rate = authority_entry.rate;
        }

        // Override with forced values if specified
        if (forced_mod != Modulation::AUTO) {
            rec_mod = forced_mod;
            LOG_MODEM(INFO, "Connection: Using FORCED modulation %s from initiator",
                      modulationToString(rec_mod));
        }

        if (forced_rate != CodeRate::AUTO) {
            rec_rate = forced_rate;
            LOG_MODEM(INFO, "Connection: Using FORCED code rate %s from initiator",
                      codeRateToString(rec_rate));
        }

        const bool forced_rung =
            environment_forced_rung ||
            forced_mod != Modulation::AUTO || forced_rate != CodeRate::AUTO;
        latent_bootstrap_rung_ =
            (isOFDMMode(negotiated_mode_) && rxRateAuthorityEnabled())
                ? (forced_rung
                       ? coherentRungIndexFor(rec_mod, rec_rate)
                       : automatic_latent_seed_rung)
                : kRungIdxNone;

        LOG_MODEM(INFO, "Connection: Initial data mode %s %s (SNR=%.1f dB (%s), forced_mod=%d, forced_rate=%d)",
                  modulationToString(rec_mod), codeRateToString(rec_rate), snr_db,
                  snrSourceToString(snr_source),
                  static_cast<int>(forced_mod), static_cast<int>(forced_rate));

        // Set our local data mode immediately
        data_modulation_ = rec_mod;
        data_code_rate_ = rec_rate;
        arq_.setCodeRate(data_code_rate_);  // Update ARQ for correct total_cw calculation

        // Pick negotiated CW count. Honor initiator's forced value if it sent
        // one (frame.data_frame_cw_count != 0), else auto-pick from rate.
        // We store the result in data_frame_cw_count_ here BEFORE building
        // the CONNECT_ACK so it reflects the value the initiator will see on
        // the wire.
        int negotiated_cw = (frame.data_frame_cw_count != 0)
            ? v2::sanitizeFixedFrameCodewords(frame.data_frame_cw_count)
            : (ladder_selected && selected_rung.waveform == WaveformMode::MC_DPSK
                ? selected_rung.cw_count
                : connection_policy::recommendCWCountForChannel(
                      rec_mod, rec_rate, negotiated_mode_, entry_fading, snr_db));

        if (rung_id == LadderRungId::UNKNOWN) {
            if (negotiated_mode_ == WaveformMode::MC_DPSK) {
                rung_id = connection_policy::rungForMCDPSKConfig(
                    rec_mod, config_.mc_dpsk_num_carriers,
                    config_.mc_dpsk_samples_per_symbol, negotiated_cw).id;
            } else if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
                rung_id = LadderRungId::OFDM_CHIRP;
            } else if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
                rung_id = LadderRungId::OFDM_NARROW;
            }
        }

        // 2026-05-28: ULTRA_FRAME_CW env override (responder bootstrap path).
        if (const char* env = std::getenv("ULTRA_FRAME_CW")) {
            const int v = std::atoi(env);
            if (v >= v2::kMinFixedFrameCodewords && v <= v2::kMaxFixedFrameCodewords) {
                LOG_MODEM(INFO, "Connection: ULTRA_FRAME_CW responder override %d -> %d",
                          negotiated_cw, v);
                negotiated_cw = v;
            }
        }
        applyDataMode(rec_mod, rec_rate, negotiated_cw, rung_id);
        const uint8_t cw_byte = static_cast<uint8_t>(data_frame_cw_count_);

        // Prefer full-callsign CONNECT_ACK when the initiator callsign is known,
        // fallback to hash-only ACK when we only have src_hash.
        // Wire fading byte: entry_fading — the SAME treatment as the SNR byte (the
        // pool aggregate under ULTRA_CONNECT_SNR_POOL, the raw scalar knob-off), so
        // the initiator's display shows the value that actually drove this pick.
        Bytes ack_data;
        if (!src_call.empty()) {
            auto ack = v2::ConnectFrame::makeConnectAck(local_call_, src_call,
                                                        static_cast<uint8_t>(negotiated_mode_),
                                                        data_modulation_, data_code_rate_,
                                                        snr_db, entry_fading,
                                                        cw_byte, rung_id);
            if (forced_rung) {
                ack.flags |= v2::Flags::CONNECT_FORCED_PROFILE;
            }
            if (phy_mask_v1_negotiated_) {
                v2::setPhyMaskV1Capability(ack);
            }
            ack_data = ack.serialize();
        } else {
            auto ack = v2::ConnectFrame::makeConnectAckByHash(local_call_, frame.src_hash,
                                                               static_cast<uint8_t>(negotiated_mode_),
                                                               data_modulation_, data_code_rate_,
                                                               snr_db, entry_fading,
                                                               cw_byte, rung_id);
            if (forced_rung) {
                ack.flags |= v2::Flags::CONNECT_FORCED_PROFILE;
            }
            if (phy_mask_v1_negotiated_) {
                v2::setPhyMaskV1Capability(ack);
            }
            ack_data = ack.serialize();
        }
        LOG_MODEM(INFO, "Connection: Sending CONNECT_ACK (%zu bytes, SNR=%.1f dB (%s))",
                  ack_data.size(), snr_db, snrSourceToString(snr_source));
        transmitFrame(ack_data);

        // Cache the ACK for collision-safe reactive replay.  Never key this full
        // 8.3 s control frame on a timer: it overlaps the initiator's CONNECT
        // retry on a half-duplex channel. A decoded duplicate CONNECT is the
        // only proof that the peer's retry body has ended and replay is safe.
        connect_ack_frame_ = ack_data;
        const uint32_t responder_handshake_failsafe_ms = responderHandshakeFailSafeMs();
        LOG_MODEM(INFO,
                  "Connection: CONNECT_ACK cached for reactive duplicate-CONNECT replay (no timer)");

        // Preserve an operator-forced modulation/rate exactly. Establish the gate
        // before on_connected can synchronously start payload transmission.
        enterConnected(/*automatic_rate_allowed=*/!forced_rung);
        responder_handshake_wait_ms_ = responder_handshake_failsafe_ms;
        // NOTE: Don't call on_handshake_confirmed_ yet - wait for first frame from initiator

        // Notify application of initial data mode. snr_is_wire=false: this is the
        // responder's OWN reading (the GUI used to mislabel it "(wire_peer)" — the
        // tell was peer_fading==local_fading on every connect line). entry_fading:
        // the GUI connect line shows the fading that drove the pick (== raw scalar
        // knob-off), matching the pooled snr_db it already shows.
        notifyDataModeChanged(snr_db, entry_fading, /*snr_is_wire=*/false);
    } else {
        pending_remote_call_ = src_call.empty() ? "REMOTE" : src_call;
        pending_remote_hash_ = frame.src_hash;  // Store hash for later
        // Store forced modes from initiator for later use in acceptCall()
        pending_forced_modulation_ = static_cast<Modulation>(frame.initial_modulation);
        pending_forced_code_rate_ = static_cast<CodeRate>(frame.initial_code_rate);
        pending_forced_cw_count_ = frame.data_frame_cw_count;  // 0 = AUTO
        if (on_incoming_call_) {
            on_incoming_call_(pending_remote_call_);
        }
    }
}

void Connection::handleConnectAck(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::CONNECTING) {
        LOG_MODEM(DEBUG, "Connection: Ignoring CONNECT_ACK (state=%s)",
                  connectionStateToString(state_));
        return;
    }

    // BUG-MC-RETRY-SPURIOUS fix 3: new session on the initiator side — clear the
    // MODE_CHANGE dedup tuple (see handleConnect for the responder-side clear).
    last_applied_mode_change_valid_ = false;

    // Get negotiated waveform mode from ConnectFrame
    WaveformMode mode = static_cast<WaveformMode>(frame.negotiated_mode);
    negotiated_mode_ = mode;
    setPhyMaskV1Negotiated(hasPhyMaskV1Capability(config_.mode_capabilities) &&
                           v2::hasPhyMaskV1Capability(frame));

    // Get initial data mode from CONNECT_ACK (eliminates separate MODE_CHANGE)
    Modulation init_mod = static_cast<Modulation>(frame.initial_modulation);
    CodeRate init_rate = static_cast<CodeRate>(frame.initial_code_rate);
    float snr_db = v2::decodeSNR(frame.measured_snr);
    float peer_fading = v2::decodeFadingIndex(frame.mode_capabilities);

    const bool forced_mod_conflict =
        outbound_forced_modulation_ != Modulation::AUTO &&
        init_mod != outbound_forced_modulation_;
    const bool forced_rate_conflict =
        outbound_forced_code_rate_ != CodeRate::AUTO &&
        init_rate != outbound_forced_code_rate_;
    if (forced_mod_conflict || forced_rate_conflict) {
        LOG_MODEM(ERROR,
                  "Connection: CONNECT_ACK conflicts with explicit outbound profile "
                  "(requested=%s %s, acknowledged=%s %s); aborting to prevent PHY split",
                  modulationToString(outbound_forced_modulation_),
                  codeRateToString(outbound_forced_code_rate_),
                  modulationToString(init_mod), codeRateToString(init_rate));
        enterDisconnected("CONNECT_ACK forced-profile conflict");
        return;
    }

    // CONNECT_ACK is the responder/rate-authority's committed physical profile.  Never
    // rewrite it locally: the responder already applied its automatic MAX cap before
    // serializing the ACK, while a unilateral initiator cap creates an immediate PHY
    // split (especially when explicit CONNECT force fields intentionally outrank MAX).

    // Negotiated CW count from responder. Falls back to recommendCWCount(rate)
    // if responder advertised 0 (interoperability with un-upgraded peer that
    // hasn't been built since the protocol change — defensive only).
    int negotiated_cw = (frame.data_frame_cw_count != 0)
        ? v2::sanitizeFixedFrameCodewords(frame.data_frame_cw_count)
        : connection_policy::recommendCWCount(init_mod, init_rate, negotiated_mode_);

    // Apply the initial data mode immediately.
    // 2026-05-28: ULTRA_FRAME_CW env override (initiator CONNECT_ACK path).
    if (const char* env = std::getenv("ULTRA_FRAME_CW")) {
        const int v = std::atoi(env);
        if (v >= v2::kMinFixedFrameCodewords && v <= v2::kMaxFixedFrameCodewords) {
            LOG_MODEM(INFO, "Connection: ULTRA_FRAME_CW initiator override %d -> %d",
                      negotiated_cw, v);
            negotiated_cw = v;
        }
    }
    applyDataMode(init_mod, init_rate, negotiated_cw, frame.ladder_rung_id);

    // Update remote callsign if we got it from the frame
    if (!src_call.empty() && (remote_call_.empty() || remote_call_ == "REMOTE")) {
        remote_call_ = src_call;
    }

    LOG_MODEM(INFO, "Connection: Connected to %s (waveform=%s, data=%s %s, SNR=%.1f dB (wire_peer), peer_fading=%.2f)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_),
              modulationToString(data_modulation_), codeRateToString(data_code_rate_), snr_db, peer_fading);

    // We are the initiator - we sent CONNECT and received CONNECT_ACK
    is_initiator_ = true;
    handshake_confirmed_ = true;  // Handshake complete for initiator
    responder_handshake_wait_ms_ = 0;

    const bool outbound_forced_rung =
        outbound_forced_modulation_ != Modulation::AUTO ||
        outbound_forced_code_rate_ != CodeRate::AUTO;
    const bool responder_forced_rung =
        (frame.flags & v2::Flags::CONNECT_FORCED_PROFILE) != 0;
    if (responder_forced_rung && !outbound_forced_rung) {
        LOG_MODEM(INFO,
                  "Connection: responder marked CONNECT_ACK profile operator-forced; "
                  "pinning automatic rate movement for this QSO");
    }
    enterConnected(
        /*automatic_rate_allowed=*/!(outbound_forced_rung || responder_forced_rung));

    // Initiator can switch to negotiated waveform immediately
    if (on_handshake_confirmed_) {
        on_handshake_confirmed_();
    }

    // Notify application of initial data mode (wire: the responder's reading,
    // echoed in CONNECT_ACK)
    notifyDataModeChanged(snr_db, peer_fading, /*snr_is_wire=*/true);
}

void Connection::handleConnectNak(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::CONNECTING) {
        return;
    }

    LOG_MODEM(WARN, "Connection: Connection rejected by %s", remote_call_.c_str());
    stats_.connects_failed++;
    enterDisconnected("Connection rejected");
}

// =============================================================================
// DISCONNECT HANDLERS
// =============================================================================

void Connection::handleDisconnect(const v2::ControlFrame& frame, const std::string& src_call) {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    const bool crossed_close = state_ == ConnectionState::DISCONNECTING;

    LOG_MODEM(INFO, "Connection: Disconnect from %s", remote_call_.c_str());

    auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
    disconnect_ack_frame_ = ack.serialize();

    if (disconnect_pending_) {
        // Already in grace period from a previous DISCONNECT — reset first so
        // synchronous host callbacks always observe the renewed close lifetime.
        LOG_MODEM(INFO, "Connection: Re-sent disconnect ACK (retransmit detected)");
        disconnect_pending_ms_ = disconnectResponderGraceMs();
        disconnect_ack_retransmit_ms_ = disconnectAckRetransmitMs();
        disconnect_ack_epoch_elapsed_ms_ = 0;
        disconnect_ack_repeat_count_ = crossed_close
            ? disconnectAckMaxProactiveRepeats()
            : 0;
        setDisconnectTeardownActive(true);
        transmitFrame(disconnect_ack_frame_);
        return;
    }

    // Enter grace period: stay connected so we can re-send ACK if initiator retransmits
    // A local disconnect() already counted this session.  Receiving the peer's
    // matching close intent while DISCONNECTING is one mutual close, not a second
    // disconnect event.
    if (!crossed_close) {
        stats_.disconnects++;
    }
    disconnect_pending_ = true;
    disconnect_pending_ms_ = disconnectResponderGraceMs();
    disconnect_ack_retransmit_ms_ = disconnectAckRetransmitMs();
    disconnect_ack_epoch_elapsed_ms_ = 0;
    disconnect_ack_repeat_count_ = crossed_close
        ? disconnectAckMaxProactiveRepeats()
        : 0;
    setDisconnectTeardownActive(true);
    transmitFrame(disconnect_ack_frame_);
    if (crossed_close) {
        LOG_MODEM(INFO,
                  "Connection: Crossed DISCONNECT received; suppressing local retries "
                  "during mutual-close grace");
    }
    LOG_MODEM(INFO,
              "Connection: Disconnect ACK sent, grace period %ums "
              "(up to %d proactive repeats every %ums; duplicate re-ACK reactive)",
              disconnect_pending_ms_, crossed_close
                  ? 0
                  : disconnectAckMaxProactiveRepeats(),
              disconnect_ack_retransmit_ms_);
}

void Connection::handleDisconnectFrame(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    const bool crossed_close = state_ == ConnectionState::DISCONNECTING;

    LOG_MODEM(INFO, "Connection: Disconnect from %s", src_call.c_str());

    // Send ACK for the disconnect
    auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
    disconnect_ack_frame_ = ack.serialize();

    if (disconnect_pending_) {
        // Already in grace period — reset before re-sending (see control form above).
        LOG_MODEM(INFO, "Connection: Re-sent disconnect ACK (retransmit detected)");
        disconnect_pending_ms_ = disconnectResponderGraceMs();
        disconnect_ack_retransmit_ms_ = disconnectAckRetransmitMs();
        disconnect_ack_epoch_elapsed_ms_ = 0;
        disconnect_ack_repeat_count_ = crossed_close
            ? disconnectAckMaxProactiveRepeats()
            : 0;
        setDisconnectTeardownActive(true);
        transmitFrame(disconnect_ack_frame_);
        return;
    }

    // Enter grace period: stay connected so we can re-send ACK if initiator retransmits
    if (!crossed_close) {
        stats_.disconnects++;
    }
    disconnect_pending_ = true;
    disconnect_pending_ms_ = disconnectResponderGraceMs();
    disconnect_ack_retransmit_ms_ = disconnectAckRetransmitMs();
    disconnect_ack_epoch_elapsed_ms_ = 0;
    disconnect_ack_repeat_count_ = crossed_close
        ? disconnectAckMaxProactiveRepeats()
        : 0;
    setDisconnectTeardownActive(true);
    transmitFrame(disconnect_ack_frame_);
    if (crossed_close) {
        LOG_MODEM(INFO,
                  "Connection: Crossed DISCONNECT received; suppressing local retries "
                  "during mutual-close grace");
    }
    LOG_MODEM(INFO,
              "Connection: Disconnect ACK sent, grace period %ums "
              "(up to %d proactive repeats every %ums; duplicate re-ACK reactive)",
              disconnect_pending_ms_, crossed_close
                  ? 0
                  : disconnectAckMaxProactiveRepeats(),
              disconnect_ack_retransmit_ms_);
}

// =============================================================================
// MODE CHANGE HANDLING
// =============================================================================

void Connection::handleModeChange(const v2::ControlFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(DEBUG, "Connection: Ignoring MODE_CHANGE (not connected)");
        return;
    }

    // Parse MODE_CHANGE payload
    auto info = frame.getModeChangeInfo();

    // BUG-MC-RETRY-SPURIOUS fix 3: dedup re-arriving copies of an ALREADY-APPLIED
    // MODE_CHANGE (the sender's diversity copies and its request-time-anchored
    // spurious retries — rig E1/D1/D3 saw a retry EVERY trough exchange even though
    // copy #1 was ACKed). Re-applying is not idempotent from the operator's seat: it
    // re-notifies the GUI (duplicate [MODE] lines) and schedules a FRESH fading-aware
    // 3-copy ACK set per copy (up to 9 control frames of airtime per exchange on a
    // fading channel — half-duplex airtime the data plane pays for). The duplicate
    // DOES carry information: the sender may have missed our ACKs. Calibrated
    // response = exactly ONE re-ACK copy per duplicate reception (any staggered
    // repeats from the first reception still drain independently). Dedup keys on the
    // (seq, mod, rate) tuple — see the state block in connection.hpp for the
    // seq-reuse rationale.
    if (last_applied_mode_change_valid_ &&
        frame.seq == last_applied_mode_change_seq_ &&
        info.modulation == last_applied_mode_change_mod_ &&
        info.code_rate == last_applied_mode_change_rate_) {
        auto dup_ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
        transmitFrame(dup_ack.serialize());
        LOG_MODEM(INFO,
                  "Connection: duplicate MODE_CHANGE seq=%u (%s %s) from %s — mode already applied; single re-ACK only",
                  static_cast<unsigned>(frame.seq),
                  modulationToString(info.modulation),
                  codeRateToString(info.code_rate),
                  remote_call_.c_str());
        return;
    }

    const char* reason_str = "unknown";
    switch (info.reason) {
        case v2::ModeChangeReason::CHANNEL_IMPROVED: reason_str = "channel improved"; break;
        case v2::ModeChangeReason::CHANNEL_DEGRADED: reason_str = "channel degraded"; break;
        case v2::ModeChangeReason::USER_REQUEST:     reason_str = "user request"; break;
        case v2::ModeChangeReason::INITIAL_SETUP:    reason_str = "initial setup"; break;
        case v2::ModeChangeReason::STARTUP_PROBE_TIMEOUT:
            reason_str = "startup probe timeout";
            break;
        case v2::ModeChangeReason::STARTUP_PROBE_BEGIN:
            reason_str = "startup probe begin";
            break;
    }

    LOG_MODEM(INFO, "Connection: MODE_CHANGE from %s: %s %s (SNR=%.1f dB (wire_peer), fading=%.2f, reason=%s)",
              remote_call_.c_str(),
              modulationToString(info.modulation),
              codeRateToString(info.code_rate),
              info.snr_db,
              info.fading_index,
              reason_str);

    if (info.reason == v2::ModeChangeReason::STARTUP_PROBE_TIMEOUT) {
        // The sender got no ACK for the one-shot target group.  It is synchronizing
        // back before any retransmission; consume the receiver's standing probe so a
        // later clean base group cannot re-arm the same QSO experiment.
        if (latent_startup_probe_waiting_) {
            failClosedLatentStartupProbeUnknown("peer reported startup-probe timeout");
        } else {
            // The target may have decoded and our success ACK may be what was lost.
            // The sender's timeout verdict still wins: both ends must converge on the
            // base and the same QSO must not recreate the one-shot probe.
            latent_startup_probe_spent_ = true;
            latent_startup_probe_clean_groups_ = 0;
            latent_startup_probe_pending_base_groups_ = 0;
            latent_startup_probe_rollback_pending_ = true;
            latent_startup_probe_failed_ = true;
            const uint8_t current = coherentRungIndexFor(
                data_modulation_, data_code_rate_);
            rx_authority_cmd_ =
                (current != kRungIdxNone && current < kRungIdxQpskR12)
                    ? current
                    : kRungIdxQpskR12;
        }
    }

    // Update local state and refresh the ARQ profile for the new fixed-frame
    // capacity/window/timing. The requester waits for this ACK before sending
    // more DATA in the new mode. The cw_count field in MODE_CHANGE is the
    // requester's chosen value — receiver applies it directly so both peers
    // stay in lockstep on frame geometry.
    applyDataMode(info.modulation, info.code_rate,
                  info.data_frame_cw_count, info.ladder_rung_id);

    // BUG-MC-RETRY-SPURIOUS fix 3: record what we just applied so duplicate copies
    // of this exact request short-circuit above (single re-ACK, no re-apply).
    last_applied_mode_change_valid_ = true;
    last_applied_mode_change_seq_ = frame.seq;
    last_applied_mode_change_mod_ = info.modulation;
    last_applied_mode_change_rate_ = info.code_rate;

    // Send ACK for the MODE_CHANGE
    auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
    const Bytes ack_data = ack.serialize();
    transmitFrame(ack_data);
    scheduleModeChangeAckRepeats(ack_data, frame.seq);

    // Notify application of mode change (wire: the requester's embedded reading)
    notifyDataModeChanged(info.snr_db, info.fading_index, /*snr_is_wire=*/true);

    runDeferredArqRefill();
}

void Connection::handleTurnover(const v2::ControlFrame& frame, const std::string& src_call) {
    (void)frame;
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }

    local_data_turn_ = true;
    peer_data_turn_requested_ = false;
    local_turn_request_pending_ = false;
    yielded_data_turn_waiting_for_peer_data_ = false;
    data_turn_yield_pending_ = false;
    turn_request_retransmit_ms_ = 0;
    turn_request_holdoff_ms_ = 0;
    resetDataTurnFairness();
    arq_.clearPendingAckRepeats();
    armDataTurnTxGuard(dataTurnControlGuardMs());

    LOG_MODEM(INFO, "Connection: RX TURNOVER from %s; local station is now ISS",
              src_call.empty() ? remote_call_.c_str() : src_call.c_str());

    // We just took the DATA turn. Our first transmission must carry a full
    // chirp+LTS anchor so the peer (which has only tracked the previous sender's
    // timing) can re-acquire us — without it the half-duplex B2F exchange stalls
    // on endless "burst marker timing retry" (BUG-TNC-B2F-001). Set the encoder's
    // one-shot full-preamble BEFORE draining any queued payload below.
    if (on_data_turn_acquired_) {
        on_data_turn_acquired_();
    }

    runDeferredArqRefill();
    sendNextQueuedPayloadIfReady();
}

void Connection::handleTurnRequest(const v2::ControlFrame& frame, const std::string& src_call) {
    (void)frame;
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }

    if (!local_data_turn_) {
        if (yielded_data_turn_waiting_for_peer_data_) {
            auto turnover = v2::ControlFrame::makeTurnover(local_call_, remote_call_);
            LOG_MODEM(INFO,
                      "Connection: RX TURN_REQUEST from %s while waiting for peer DATA; reasserting TURNOVER",
                      src_call.empty() ? remote_call_.c_str() : src_call.c_str());
            transmitFrame(turnover.serialize());
            local_turn_request_pending_ = false;
            turn_request_retransmit_ms_ = 0;
            turn_request_holdoff_ms_ = turnRequestHoldoffAfterDataMs();
            armDataTurnTxGuard(dataTurnControlGuardMs());
            // Re-arm: the peer asking again proves the previous grant did not land, so the
            // recovery timer restarts from this re-assert rather than from the first one.
            armTurnoverRepeat();
        }
        return;
    }

    peer_data_turn_requested_ = true;
    LOG_MODEM(INFO, "Connection: RX TURN_REQUEST from %s",
              src_call.empty() ? remote_call_.c_str() : src_call.c_str());
    maybeYieldDataTurn();
}

void Connection::handleFileCancel(const v2::ControlFrame& frame, const std::string& src_call) {
    (void)frame;
    if (state_ != ConnectionState::CONNECTED) {
        return;
    }

    LOG_MODEM(INFO, "Connection: RX FILE_CANCEL from %s",
              src_call.empty() ? remote_call_.c_str() : src_call.c_str());

    const bool had_active_transfer =
        file_transfer_.isBusy() || queued_file_path_.has_value();
    if (!had_active_transfer && file_cancel_reassert_ms_ > 0) {
        LOG_MODEM(INFO, "Connection: RX FILE_CANCEL confirmation; cancel reassertion cleared");
        clearFileCancelReassertion();
        file_cancel_rx_drain_ms_ = 0;
        file_cancel_confirm_pending_ = false;
        return;
    }
    if (!had_active_transfer) {
        LOG_MODEM(INFO, "Connection: RX duplicate FILE_CANCEL with no active transfer; ignoring");
        return;
    }

    const bool was_local_iss = local_data_turn_;
    queued_file_path_.reset();
    if (file_transfer_.isBusy()) {
        file_transfer_.cancel("Transfer cancelled");
    }
    clearFileTransferArqState();
    file_cancel_rx_drain_ms_ = FILE_CANCEL_RX_DRAIN_MS;
    armDataTurnTxGuard(fileCancelTxGuardMs());
    file_cancel_confirm_pending_ = had_active_transfer;
    data_turn_yield_pending_ = false;
    resetDataTurnFairness();

    if (was_local_iss) {
        // A FILE_CANCEL aborts the file transfer, not the DATA turn. If the peer
        // cancelled while receiving our file, we keep ISS ownership so locally
        // queued operator payloads can go out immediately after the abort.
        peer_data_turn_requested_ = false;
        local_turn_request_pending_ = false;
        turn_request_retransmit_ms_ = 0;
        turn_request_holdoff_ms_ = 0;
    } else {
        local_turn_request_pending_ = false;
        turn_request_retransmit_ms_ = 0;
        sendTurnRequestIfNeeded();
    }

    sendNextQueuedPayloadIfReady();
}

void Connection::requestModeChange(Modulation new_mod, CodeRate new_rate,
                                    float measured_snr, uint8_t reason) {
    if (state_ != ConnectionState::CONNECTED || disconnect_teardown_active_) {
        LOG_MODEM(WARN,
                  "Connection: Cannot request mode change (link is not data-ready)");
        return;
    }

    // Don't send new MODE_CHANGE if one is already pending
    if (mode_change_pending_) {
        LOG_MODEM(DEBUG, "Connection: MODE_CHANGE already pending, ignoring request");
        return;
    }

    // When the wire value is the stale sentinel, say SO and say what the geometry was
    // actually decided on. Without this the log reads `SNR=-10.0` and gives no hint that
    // the decision used something else — which is exactly how the pre-fix rig trace
    // looked, and why the defect survived a read of that log.
    if (connection_policy::isStaleSnrSentinel(measured_snr)) {
        LOG_MODEM(INFO,
                  "Connection: Requesting MODE_CHANGE to %s %s (wire SNR=unknown/stale; decided on %.1f dB (%s))",
                  modulationToString(new_mod), codeRateToString(new_rate),
                  decisionSnrDb(), snrSourceToString(measured_snr_source_));
    } else {
        LOG_MODEM(INFO, "Connection: Requesting MODE_CHANGE to %s %s (SNR=%.1f dB (%s))",
                  modulationToString(new_mod), codeRateToString(new_rate), measured_snr,
                  snrSourceToString(measured_snr_source_));
    }

    // Store pending mode change parameters for retry
    pending_modulation_ = new_mod;
    pending_code_rate_ = new_rate;
    pending_snr_db_ = measured_snr;
    // Freshness-gated like the SNR byte (rig W5b/W8: peer_fading frozen on the
    // wire for 300+ s); stale -> -1 -> wire "unknown" -> receiver's n/a render.
    pending_fading_index_ = wireFadingIndex();
    pending_reason_ = reason;
    mode_change_pending_ = true;
    mode_change_retry_count_ = 0;
    mode_change_timeout_ms_ = modeChangeRetryMs();
    pending_ladder_rung_id_ = currentLadderRungId();

    // Pick the CW count for the new rate. If the operator forced a CW
    // count via --cw-count (config_.forced_cw_count != 0), preserve that
    // override across mode changes — otherwise auto-pick from rate.
    // Without this, a later MODE_CHANGE would silently drop the user's
    // override (caught by Codex, 2026-05-04).
    // CW count for the new rung: uses the Doppler-coherence-REFINED fading index
    // (BUG-DOPPLER-COHERENCE-MODECHANGE-WIPE follow-through: this is the mid-stream
    // negotiate site, where the coherence verdict CAN be valid — equals the raw
    // fading_index until it is).
    // measured_snr is the WIRE value and may be the stale sentinel (-10.0 = "no
    // measurement"). It is correct to REPORT that, but a CW recommendation made from
    // it would read a healthy link as catastrophic and pick the most conservative
    // geometry. Decide on the last real measurement instead; the wire byte is
    // unaffected (pending_snr_db_ above still carries the honest "unknown").
    pending_cw_count_ = static_cast<uint8_t>((config_.forced_cw_count != 0)
        ? v2::sanitizeFixedFrameCodewords(config_.forced_cw_count)
        : connection_policy::recommendCWCountForChannel(
              new_mod, new_rate, negotiated_mode_,
              connection_policy::coherenceAdjustedFadingIndex(
                  fading_index_, coherence_score_, coherence_valid_),
              decisionSnrDb()));

    mode_change_seq_++;
    auto frame = v2::ControlFrame::makeModeChange(local_call_, remote_call_,
                                                   mode_change_seq_, new_mod, new_rate,
                                                   measured_snr, fading_index_, reason,
                                                   pending_cw_count_,
                                                   pending_ladder_rung_id_);
    transmitFrame(frame.serialize());

    // NOTE: Don't update local mode until ACK is received
    // This prevents mode mismatch if the remote doesn't receive our MODE_CHANGE
}

// =============================================================================
// DATA PAYLOAD HANDLING
// =============================================================================

void Connection::handleDataPayload(const Bytes& payload, bool more_data,
                                   v2::FrameType frame_type, uint8_t flags) {
    if (payload.empty()) {
        return;
    }

    // EPOCH_REBASE also appears at ordinary same-epoch window boundaries, so the
    // bit alone is not an application-object reset. Only a changed epoch plus a
    // rebase proves the sender abandoned older unresolved DATA identities. ARQ
    // discards its slots at that adoption; application reassembly must follow.
    const uint8_t frame_epoch = v2::epochFromFlags(flags);
    if ((flags & v2::Flags::EPOCH_REBASE) != 0 &&
        rx_reassembly_active_ && frame_epoch != rx_reassembly_epoch_) {
        LOG_MODEM(WARN,
                  "Connection: Dropping %zu-byte stale RX fragment prefix at move-epoch change %u -> %u",
                  rx_reassembly_buffer_.size(),
                  static_cast<unsigned>(rx_reassembly_epoch_),
                  static_cast<unsigned>(frame_epoch));
        rx_reassembly_buffer_.clear();
        rx_reassembly_active_ = false;
        rx_reassembly_binary_ = false;
    }
    received_peer_data_since_connect_ = true;
    yielded_data_turn_waiting_for_peer_data_ = false;
    turn_request_holdoff_ms_ = turnRequestHoldoffAfterDataMs();

    const bool binary_payload =
        frame_type == v2::FrameType::DATA_START ||
        frame_type == v2::FrameType::DATA_CONT ||
        frame_type == v2::FrameType::DATA_END;

    if (!binary_payload && file_transfer_.processPayload(payload, more_data)) {
        LOG_MODEM(DEBUG, "Connection: Processed file transfer payload (%zu bytes)",
                  payload.size());
        return;
    }

    if (more_data) {
        if (rx_reassembly_active_ &&
            rx_reassembly_binary_ != binary_payload) {
            LOG_MODEM(WARN,
                      "Connection: RX fragment kind changed mid-object; discarding %zu stale bytes",
                      rx_reassembly_buffer_.size());
            rx_reassembly_buffer_.clear();
            rx_reassembly_active_ = false;
        }
        if (!rx_reassembly_active_) {
            rx_reassembly_active_ = true;
            rx_reassembly_binary_ = binary_payload;
            rx_reassembly_epoch_ = frame_epoch;
        }
        // Fragment with MORE_FRAG - accumulate
        rx_reassembly_buffer_.insert(rx_reassembly_buffer_.end(), payload.begin(), payload.end());
        LOG_MODEM(DEBUG, "Connection: Accumulated fragment (%zu bytes, buffer now %zu bytes)",
                  payload.size(), rx_reassembly_buffer_.size());
        return;
    }

    // The payload stream is complete here (FINAL / single frame). Clear the "incoming
    // burst" GUI indicator — files clear via setFileReceivedCallback, but a MESSAGE or
    // binary payload completes on THIS path, so without this the flash kept pulsing
    // "received group X/Y" after the message was already delivered (the resend of a faded
    // final fragment arrives on the per-frame path, which never re-sets the indicator, so
    // it sat at the last group's stale value).
    burst_activity_ = BurstActivity{};

    // Final or single frame
    Bytes complete_payload;
    if (rx_reassembly_active_ && rx_reassembly_binary_ != binary_payload) {
        LOG_MODEM(WARN,
                  "Connection: RX final kind does not match buffered fragments; discarding %zu stale bytes",
                  rx_reassembly_buffer_.size());
        rx_reassembly_buffer_.clear();
        rx_reassembly_active_ = false;
    }
    if (!rx_reassembly_buffer_.empty()) {
        // Last fragment - stitch into a fresh buffer to avoid aliasing/overflow false positives.
        complete_payload.reserve(rx_reassembly_buffer_.size() + payload.size());
        complete_payload.insert(complete_payload.end(),
                                rx_reassembly_buffer_.begin(),
                                rx_reassembly_buffer_.end());
        complete_payload.insert(complete_payload.end(), payload.begin(), payload.end());
        rx_reassembly_buffer_.clear();
        rx_reassembly_active_ = false;
        LOG_MODEM(INFO, "Connection: Reassembled %zu-byte %s from fragments",
                  complete_payload.size(), binary_payload ? "binary payload" : "message");
    } else {
        // Single-frame message (backwards compatible)
        rx_reassembly_active_ = false;
        complete_payload = payload;
    }
    rx_reassembly_binary_ = false;
    rx_reassembly_epoch_ = 0;

    if (binary_payload) {
        if (on_data_received_) {
            on_data_received_(complete_payload, false);
        }
        return;
    }

    // Strip the message-object type prefix.
    size_t start = 0;
    if (!complete_payload.empty()) {
        if (complete_payload[0] ==
                static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT) &&
            complete_payload.size() >= kMessageObjectPrefixBytes) {
            const uint8_t object_id = complete_payload[1];
            start = kMessageObjectPrefixBytes;
            // Idempotent-resend contract (BUG-MESSAGE-LOST-ON-FORCED-DEMOTE). The
            // SENDER cannot tell "never received" from "received, ACK still in
            // flight" — the classic ARQ ambiguity — so it is allowed to re-send an
            // object it re-gridded onto a new geometry. The RECEIVER can tell, and
            // suppresses the copy here, before any application callback. Dropping a
            // duplicate is silent on purpose: nothing was lost.
            if (messageObjectAlreadyDelivered(object_id)) {
                LOG_MODEM(INFO,
                          "Connection: Dropping duplicate message object id=%u (%zu bytes) — already delivered",
                          static_cast<unsigned>(object_id),
                          complete_payload.size() - kMessageObjectPrefixBytes);
                return;
            }
            noteMessageObjectDelivered(object_id);
        } else if (complete_payload[0] ==
                   static_cast<uint8_t>(PayloadType::TEXT_MESSAGE)) {
            // Legacy bare text object: no identity, so no duplicate suppression.
            start = 1;
        } else {
            // Every text object we emit is TEXT_MESSAGE_OBJECT, so an unknown
            // discriminator means this is not a whole object: a stale fragment
            // prefix that survived a move-epoch change and got stitched to a fresh
            // one, or a FILE payload the file controller declined. Handing those
            // bytes to the operator as a message would present corruption as
            // content. Drop it loudly instead.
            LOG_MODEM(WARN,
                      "Connection: Discarding %zu-byte payload with unknown message discriminator 0x%02X",
                      complete_payload.size(),
                      static_cast<unsigned>(complete_payload[0]));
            return;
        }
    }

    // Application bytes only — consumers never see the transport prefix.
    Bytes application_payload(
        complete_payload.begin() + static_cast<std::ptrdiff_t>(
            std::min(start, complete_payload.size())),
        complete_payload.end());

    std::string text(application_payload.begin(), application_payload.end());

    if (!text.empty() && on_message_received_) {
        on_message_received_(text);
    }

    if (on_data_received_) {
        on_data_received_(application_payload, false);
    }
}

// =============================================================================
// WAVEFORM MODE NEGOTIATION
// =============================================================================

WaveformMode Connection::negotiateMode(uint8_t remote_caps, WaveformMode remote_pref) {
    uint8_t common = config_.mode_capabilities & remote_caps;
    const bool rate_selection_snr_valid = measured_snr_valid_;
    // #58: selection compares against dial-calibrated thresholds — use the
    // basis-corrected value (fade-effective reading + fading penalty). Increment 3:
    // the input is the pool aggregate under ULTRA_CONNECT_SNR_POOL (same accessor as
    // handleConnect/acceptCall, so the forced/legacy negotiation path can't disagree
    // with the ladder path within one handshake); knob-off = the raw scalar.
    // Increment 4: the fading input is pooled the same way (rateSelectionFadingIndex).
    const float entry_fading_raw = rateSelectionFadingIndex();
    // Entry classification shrinkage — same rationale as handleConnect (see the
    // helper's provenance comment); knob-off keeps the raw scalar path.
    const float entry_fading =
        connection_policy::connectSnrPoolEnabled()
            ? connection_policy::entryClassificationFadingIndex(
                  entry_fading_raw,
                  connect_snr_pool_.effectiveCount(
                      connectSnrPoolTcMs(), /*handshake_only=*/true,
                      /*max_age_ms=*/UINT64_MAX))
            : entry_fading_raw;
    float snr = rate_selection_snr_valid
        ? connection_policy::connectSelectionSnrDb(rateSelectionSnrDb(), entry_fading,
                                                   rateSelectionSnrDataAided(),
                                                   physical_channel_mean_db_,
                                                   physical_channel_n_)
        : 0.0f;
    WaveformMode selected = connection_policy::selectNegotiatedMode(
        config_.mode_capabilities,
        remote_caps,
        remote_pref,
        narrowband_override_,
        config_.preferred_mode,
        snr,
        entry_fading);

    if (common == 0) {
        LOG_MODEM(WARN, "Connection: No common waveform modes! Falling back to OFDM");
        return selected;
    }

    // If remote has explicit preference, honor it if we support it
    if (remote_pref != WaveformMode::AUTO) {
        uint8_t pref_bit = connection_policy::modeToCapabilityBit(remote_pref);
        if ((common & pref_bit) && selected == remote_pref) {
            LOG_MODEM(INFO, "Connection: Using remote preferred mode: %s",
                      waveformModeToString(remote_pref));
            return selected;
        }
    }

    // If narrowband chirp was detected this session, override for this connection
    if (narrowband_override_ != WaveformMode::AUTO) {
        uint8_t pref_bit = connection_policy::modeToCapabilityBit(narrowband_override_);
        if ((common & pref_bit) && selected == narrowband_override_) {
            LOG_MODEM(INFO, "Connection: Using narrowband override: %s",
                      waveformModeToString(narrowband_override_));
            return selected;
        }
    }

    // If we have explicit preference, use it if remote supports it
    if (config_.preferred_mode != WaveformMode::AUTO) {
        uint8_t pref_bit = connection_policy::modeToCapabilityBit(config_.preferred_mode);
        if ((common & pref_bit) && selected == config_.preferred_mode) {
            LOG_MODEM(INFO, "Connection: Using local preferred mode: %s",
                      waveformModeToString(config_.preferred_mode));
            return selected;
        }
    }

    // AUTO mode: Use shared algorithm from waveform_selection.hpp
    // This ensures negotiateMode and recommendDataModeWithFading use same logic
    LOG_MODEM(INFO, "Connection: AUTO mode selection, SNR_sel=%.1f dB (%s), fading_index=%.2f (%s)",
              snr, snrSourceToString(measured_snr_source_), entry_fading,
              connection_policy::fadingLabel(entry_fading));

    auto rec = recommendWaveformAndRate(
        snr, connection_policy::coherenceAdjustedFadingIndex(entry_fading, coherence_score_,
                                                             coherence_valid_));

    // Check if selected mode is supported by both sides
    if (selected == rec.waveform &&
        (common & connection_policy::modeToCapabilityBit(selected))) {
        LOG_MODEM(INFO, "Connection: Selected %s (SNR_sel=%.1f (%s), fading=%.2f %s)",
                  waveformModeToString(selected),
                  snr,
                  snrSourceToString(measured_snr_source_),
                  entry_fading,
                  connection_policy::fadingLabel(entry_fading));
        return selected;
    }

    // Fallback if selected mode is not supported by both peers.
    // Production priority excludes reserved OTFS/MFSK values.
    return selected;
}

} // namespace protocol
} // namespace ultra
