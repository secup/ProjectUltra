// StreamingDecoder module

#include "streaming_decoder.hpp"
#include "sync/acquisition_mode.hpp"
#include "streaming_buffer_policy.hpp"
#include "streaming_decode_policy.hpp"
#include "streaming_decoder_debug.hpp"
#include "sync/frame_arrival_policy.hpp"
#include "streaming_frame_policy.hpp"
#include "sync/signal_policy.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "fec/frame_interleaver.hpp"
#include "fec/burst_interleaver.hpp"
#include "ultra/fec.hpp"
#include "fec/ldpc_codec.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/timing_profiler.hpp"
#include "protocol/connection_policy.hpp"  // kCoherence{Good,Moderate}Threshold (single source of truth)
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <stdexcept>

namespace ultra {
namespace gui {

namespace decode_policy = streaming_decode_policy;
namespace arrival_policy = ::ultra::sync::frame_arrival_policy;
namespace signal_policy = ::ultra::sync::signal_policy;

namespace {

bool qam16GenieTimingCfoEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_QAM16_GENIE_TIMING_CFO");
        return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
    }();
    return enabled;
}

// ULTRA_CONNECT_RATIOMETRIC_SNR (now DEFAULT-ON; opt-out via =0): route the ratiometric
// MC-DPSK training-symbol SNR for the UN-connected handshake (PING/CONNECT/
// CONNECT_ACK) control frames, not only for connected MC-DPSK data flow.
//
// WHY: the default connect-time rate decision consumes idle_in_band SNR, which
// is 10*log10(kModemReferencePower / measured_noise) -- a NOISE-ONLY meter that
// ASSUMES the received signal sits at the simulator's reference level
// (kModemReferenceInBandRms = 0.30482664). In the faithful sim that holds (AWGN
// is sized from encodePing() at exactly that RMS) so idle is correct. On a real
// radio the RX operating level is several dB below that reference, so idle
// credits signal power the link does not have and OVER-READS the SNR by the
// level deficit -> too-aggressive connect-time rate picks (e.g. MPG@10 -> QPSK
// R1/2 that stalls in the fades). The MC-DPSK training SNR
// (updateTrainingSNREstimate: 10*log10(signal_power/residual_power), both terms
// 50-2950 Hz in-band filtered) is a pure ratio of measured powers -> level-
// invariant by construction (= 10log10(|H|^2/noise_var)), honest at any level,
// and it is already populated for the inbound handshake preamble and already an
// accepted rate-selection source (MCDPSK_IN_BAND). The ONLY thing suppressing
// it during the handshake is the connected_ gate below; this knob relaxes it.
//
// PROMOTED to DEFAULT-ON 2026-07-01 after the multi-channel rig A/B this note demanded:
// on the rig the idle meter over-reads (RX below the reference level) and mis-selects a
// too-aggressive mode that STALLS on BOTH Good (#74, MPG@10) and Moderate (MPM@8, idle
// -> OFDM QPSK R1/2 no-delivery), while ratiometric picks the correct robust mode and
// delivers. The over-read is a LEVEL deficit, channel-independent -> generalizes to AWGN/
// Poor. On the faithful gui_qso gate (level AT reference) the ~-0.45 dB scale offset can
// nudge a pick at a ladder boundary but is more-conservative and never regresses delivery
// (verified good@20 no-regress). See docs/CHANGELOG.md (BUG-CONNECT-SNR-LEVEL).
bool connectRatiometricSnrEnabled() {
    // #74/#71 (2026-07-01): now DEFAULT-ON; opt OUT via ULTRA_CONNECT_RATIOMETRIC_SNR=0.
    // Promoted after multi-channel rig A/B showed the level-dependent idle meter OVER-READS
    // on a real radio and mis-selects a too-aggressive mode that STALLS, while the
    // level-invariant ratiometric SNR picks the correct robust mode and delivers:
    //   MPG@10 (Good, #74): idle 13.1 -> QPSK R1/2 stall; ratiometric 8.0 -> robust rung.
    //   MPM@8  (Moderate):  idle -> OFDM QPSK R1/2 STALL (0 ACKs, 8 timeout resends, no
    //                       delivery); ratiometric ~1 dB -> MC-DPSK DBPSK R1/4 CRC-clean 0-retx.
    // The over-read is a LEVEL deficit (RX below the 0.3048 sim reference), independent of
    // channel type, so it generalizes to AWGN/Poor. On the faithful sim (level AT reference)
    // the two estimators differ by only ~0.45 dB -- can nudge a pick at a ladder boundary but
    // never regresses delivery (more conservative). See docs/CHANGELOG.md.
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_CONNECT_RATIOMETRIC_SNR");
        if (!value || value[0] == '\0') return true;      // default-ON
        return !(value[0] == '0' && value[1] == '\0');    // "0" opts out
    }();
    return enabled;
}

// #58 BUG-CONNECT-SNR-VARIANCE (2026-07-02): when a decoded MC-DPSK frame also
// carries a data-aided whole-frame SNR estimate, route THAT as MCDPSK_IN_BAND
// instead of the ~170 ms training snapshot. Same basis (level-invariant,
// in-band 50-2950 Hz ratio), lower variance: the training window is ONE fade
// state (Tc ~ 4.2 s on Good -> rig connect snapshots spread ~10 dB), while the
// frame's CWs span seconds, so the per-symbol average is fade-averaged by
// construction. Decode-then-measure: only routed when THIS frame's LDPC decode
// succeeded. Default-ON; opt OUT via ULTRA_CONNECT_DATA_AIDED_SNR=0.
bool connectDataAidedSnrEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_CONNECT_DATA_AIDED_SNR");
        if (!value || value[0] == '\0') return true;      // default-ON
        return !(value[0] == '0' && value[1] == '\0');    // "0" opts out
    }();
    return enabled;
}

}  // namespace

void StreamingDecoder::populateDecodeMetrics(DecodeResult& result, bool is_ofdm,
                                             float residual_cfo_hz,
                                             bool channel_evidence_ok) const {
    stampRxSignal();  // F129: decoder evidence of incoming signal (per-frame)
    // Handoff §2: per-frame physical channel SNR from the waveform (power
    // ratio over the exact training span vs THIS frame's burst-time noise
    // ref) — replaces the old handshake-latched decoder-side computation.
    if (!is_ofdm && waveform_ && waveform_->hasPhysicalSNR()) {
        const float phys = waveform_->getPhysicalSNRdB();
        last_physical_snr_db_.store(phys);
        last_physical_snr_valid_.store(true);
        notePhysicalSnrSample(phys);  // §5: distribution ring
    }
    result.sync_correlation = sync_correlation_;
    result.sync_quality_db = result.snr_db;
    const auto idle_snr = idle_noise_snr_estimator_.snapshot();
    result.has_idle_in_band_snr_db =
        idle_snr.valid && std::isfinite(idle_snr.idle_in_band_snr_db);
    result.idle_in_band_snr_db = idle_snr.idle_in_band_snr_db;

    if (is_ofdm && waveform_) {
        result.ofdm_internal_snr_db = waveform_->estimatedSNR();
        result.has_ofdm_broadband_snr_db = waveform_->hasLastOFDMBroadbandSNREstimate();
        result.ofdm_broadband_snr_db = waveform_->getLastOFDMBroadbandSNREstimate();
        // One finalized, one-shot observation from THIS physical demodulation
        // pass.  A no-process erasure therefore cannot reuse the preceding
        // frame's EVM, and the group tail is available before its callback.
        result.has_evm_snr_db = waveform_->takeEvmSnrEstimate(
            result.evm_snr_db, result.evm_carrier_count);
        result.lts_fading_index = waveform_->getFadingIndex();
        result.lts_timing_offset_samples = waveform_->getLastTimingOffsetSamples();
        result.pilot_frequency_cv = waveform_->getLastPilotFrequencyCV();
        result.pilot_temporal_cv = waveform_->getLastPilotTemporalCV();
        result.pilot_symbol_mean_cv = waveform_->getLastPilotSymbolMeanCV();
        result.lts_residual_cfo_hz = waveform_->getLastLTSResidualCFOHz();
        // Good/Moderate discriminator: feed one per-frame |H|^2 snapshot (the LTS channel
        // magnitude) into the decoder-hosted Doppler-coherence estimator. populateDecodeMetrics
        // runs ~once per decoded OFDM frame, so the snapshot cadence IS the inter-frame spacing
        // (~1-2 s) — the lag at which Good (slow fading) and Moderate (fast) diverge. Hosting it
        // here (not in the demodulator) survives the per-group demodulator recreation.
        const float lts_mag = waveform_->getLastLTSChannelMagnitude();
        // ESTIMATOR HYGIENE (F142): feed the coherence estimator ONLY from
        // successful decodes. Failed/false-lock attempts (noise windows, our own
        // deferred-ACK echo) contributed |H| snapshots of TONES (across-carrier
        // CV ~3) — four such snapshots poisoned the cumulative lag-1 score below
        // the Moderate threshold, coherenceAdjustedFadingIndex then PINNED
        // effective fading at 0.85 while the real channel read 0.15-0.5, and the
        // ladder ground to the R1/4 basement on a 24 dB link. The decode verdict
        // is the admission ticket for channel-statistics evidence.
        // STARVATION FIX (2026-07-25): `result.success` alone is FALSE by construction on
        // the burst path — populateDecodeMetrics is handed a default-constructed metrics
        // TEMPLATE built BEFORE the LDPC verdict exists. Since 1820987 (2026-07-06) that
        // starved this estimator to ZERO snapshots on the wideband file path, so
        // coherenceScore() returned a literal 0.0f, valid() stayed false, and
        // coherenceAdjustedFadingIndex silently fell back to the blind CV for every
        // rate decision (rig: 157/157 verdicts read coh=0.00 on BOTH channels).
        // `channel_evidence_ok` carries the caller's REAL verdict where one exists. The
        // F142 hygiene contract is unchanged — evidence is still admitted ONLY on a
        // successful decode; this just lets the burst path say so truthfully.
        if ((result.success || channel_evidence_ok) &&
            std::isfinite(lts_mag) && lts_mag > 0.0f) {
            doppler_coherence_.addSnapshot(lts_mag * lts_mag);
            // COH-DIAG (read-only diagnostic, env ULTRA_COH_DIAG=1, default OFF): per-frame
            // raw disc inputs for the noise-floor / Doppler-discriminator investigation —
            // snap=|H|^2, h_mag=mean_c|H_c|, lts_noise_var=E|H1-H0|^2/4, lts_sig_pow=mean_c|H_c|^2.
            // Used to classify whether the snapshot decorrelation is LTS estimation noise
            // (de-biasable) vs common-mode cheap-card wander. Gated so it does not spam logs.
            static const bool kCohDiag = [] {
                const char* e = std::getenv("ULTRA_COH_DIAG");
                return e && e[0] != '\0' && !(e[0] == '0' && e[1] == '\0');
            }();
            if (kCohDiag) {
                const float lts_noise_var = waveform_->getLastLTSNoiseVariance();
                const float lts_sig_pow = waveform_->getLastLTSSignalPower();
                LOG_MODEM(INFO,
                          "[%s] COH-DIAG snap=%.6g h_mag=%.6g lts_noise_var=%.6g lts_sig_pow=%.6g",
                          log_prefix_.c_str(), lts_mag * lts_mag, lts_mag, lts_noise_var,
                          lts_sig_pow);
            }
        }
        result.doppler_coherence_score = doppler_coherence_.coherenceScore();
        result.doppler_hz = doppler_coherence_.dopplerHz();
        result.doppler_coherence_valid = doppler_coherence_.valid();
        last_doppler_coherence_score_.store(result.doppler_coherence_score);
        last_doppler_hz_.store(result.doppler_hz);
        last_doppler_coherence_valid_.store(result.doppler_coherence_valid);
        if (result.doppler_coherence_valid) {
            const float sc = result.doppler_coherence_score;
            const bool confident_good = sc >= protocol::connection_policy::kCoherenceGoodThreshold;
            const bool confident_mod = sc <= protocol::connection_policy::kCoherenceModerateThreshold;
            const char* cls = confident_good ? "GOOD"
                                             : (confident_mod ? "MODERATE/POOR" : "uncertain");
            // ADAPTIVITY_AUDIT Case #2: de-pessimize the Wiener correlation model ONLY on a
            // confident Good (slower 0.1 Hz / 0.5 ms keeps more pilot history -> better
            // estimate). Uncertain / Moderate keep the env-aware Moderate-HF default — and the
            // per-group demod rebuild resets the override, so we never strand a stale Good.
            if (confident_good) waveform_->setWienerChannelParams(0.1f, 0.5e-3f);
            // Stage A (read-only): the scale-invariant coherence-AREA verdict, logged alongside the
            // legacy lag-1 score for live cross-platform validation (the area is the radio-agnostic
            // discriminator; threshold ~0.19, hysteresis enter 0.20/exit 0.12 — see
            // docs/SCALE_INVARIANT_COHERENCE_DISC_2026_06_20.md). NOT yet consumed — Stage B/C wire it.
            const float area = doppler_coherence_.coherenceArea();
            const char* acls = (area >= protocol::connection_policy::kCoherenceAreaEnterGood) ? "GOOD"
                             : (area <  protocol::connection_policy::kCoherenceAreaExitGood) ? "MOD/POOR"
                                                                                            : "uncertain";
            LOG_MODEM(INFO,
                      "[%s] Doppler coherence: score=%.3f doppler=%.3f Hz [%s] area=%.3f [%s] (snaps=%zu) "
                      "vs fading_index=%.3f%s",
                      log_prefix_.c_str(), sc, result.doppler_hz, cls, area, acls,
                      doppler_coherence_.snapshotCount(), result.lts_fading_index,
                      confident_good ? " (Wiener->Good 0.1Hz/0.5ms)" : "");
        }
        result.snr_source = SNRSource::SYNC_QUALITY;
        if (result.has_ofdm_broadband_snr_db) {
            // §5: the recalibrated OFDM broadband reading is truth-calibrated
            // (OFDMSnrCalibration ±1.5 dB) — feed the channel-SNR distribution
            // ring so the mean±spread display keeps updating through OFDM
            // transfers (MC-DPSK frames stop at the handshake).
            notePhysicalSnrSample(result.ofdm_broadband_snr_db);
            result.snr_db = result.ofdm_broadband_snr_db;
            result.snr_source = SNRSource::OFDM_BROADBAND;
            last_ofdm_broadband_snr_db_valid_.store(true);
            last_ofdm_broadband_snr_db_.store(result.ofdm_broadband_snr_db);
        }
        // Frame-local EVM provenance is independent of whether the broadband
        // estimator happened to publish.  Always overwrite validity so a failed
        // or no-process frame cannot leave a previous value live.
        last_evm_snr_db_valid_.store(result.has_evm_snr_db);
        last_evm_snr_db_.store(result.has_evm_snr_db ? result.evm_snr_db : 0.0f);
        LOG_MODEM(DEBUG, "[%s] OFDM quality: sync_quality=%.1f dB "
                  "ofdm_broadband=%s%.1f dB ofdm_internal=%.1f dB "
                  "idle_in_band=%s%.1f dB routed_snr=%.1f dB (%s) fading=%.3f",
                  log_prefix_.c_str(), result.sync_quality_db,
                  result.has_ofdm_broadband_snr_db ? "" : "unavailable/",
                  result.ofdm_broadband_snr_db, result.ofdm_internal_snr_db,
                  result.has_idle_in_band_snr_db ? "" : "unavailable/",
                  result.idle_in_band_snr_db, result.snr_db,
                  snrSourceToString(result.snr_source),
                  result.lts_fading_index);
    } else {
        result.snr_source = SNRSource::SYNC_QUALITY;
        const auto* mc_waveform = dynamic_cast<const MCDPSKWaveform*>(waveform_.get());
        const bool training_snr_valid =
            mc_waveform && mc_waveform->hasEstimatedSNR() &&
            std::isfinite(mc_waveform->estimatedSNR());
        const bool data_aided_snr_valid =
            mc_waveform && mc_waveform->hasDataAidedSNR() &&
            std::isfinite(mc_waveform->getDataAidedSNRdB());
        if (training_snr_valid) {
            result.has_mcdpsk_training_snr_db = true;
            result.mcdpsk_training_snr_db = mc_waveform->estimatedSNR();
        }
        if (data_aided_snr_valid) {
            result.has_mcdpsk_data_aided_snr_db = true;
            result.mcdpsk_data_aided_snr_db = mc_waveform->getDataAidedSNRdB();
        }
        // connected_ OR the opt-in knob: when ULTRA_CONNECT_RATIOMETRIC_SNR is
        // set, the level-invariant MC-DPSK training SNR also routes for the
        // un-connected handshake frames (so the connect-time rate decision uses
        // an honest, level-agnostic SNR instead of the reference-assuming idle
        // meter). Knob OFF => (connected_ || false) == connected_ => byte-identical.
        if ((connected_ || connectRatiometricSnrEnabled()) &&
            mode_ == protocol::WaveformMode::MC_DPSK && training_snr_valid) {
            // #58 BUG-CONNECT-SNR-VARIANCE: prefer the whole-frame data-aided
            // fade-AVERAGED estimate over the single-fade-state training
            // snapshot when this frame's LDPC decode SUCCEEDED
            // (decode-then-measure). Same source tag: same basis, lower
            // variance. Fall back to the training value otherwise.
            const bool route_data_aided = connectDataAidedSnrEnabled() &&
                                          result.success && data_aided_snr_valid;
            result.snr_db = route_data_aided ? result.mcdpsk_data_aided_snr_db
                                             : result.mcdpsk_training_snr_db;
            result.snr_source = SNRSource::MCDPSK_IN_BAND;
            result.mcdpsk_snr_routed_data_aided = route_data_aided;
            if (data_aided_snr_valid) {
                LOG_MODEM(INFO,
                          "[%s] MC-DPSK SNR: training=%.1f data_aided=%.1f (routed=%s)",
                          log_prefix_.c_str(), result.mcdpsk_training_snr_db,
                          result.mcdpsk_data_aided_snr_db,
                          route_data_aided ? "data_aided" : "training");
            } else {
                LOG_MODEM(INFO,
                          "[%s] MC-DPSK SNR: training=%.1f data_aided=n/a (routed=training)",
                          log_prefix_.c_str(), result.mcdpsk_training_snr_db);
            }
        } else if (result.has_idle_in_band_snr_db) {
            result.snr_db = result.idle_in_band_snr_db;
            result.snr_source = SNRSource::IDLE_IN_BAND;
        }
        result.ofdm_internal_snr_db = 0.0f;
        result.has_ofdm_broadband_snr_db = false;
        result.ofdm_broadband_snr_db = 0.0f;
        result.lts_fading_index = 0.0f;
        result.lts_residual_cfo_hz = residual_cfo_hz;
        last_ofdm_broadband_snr_db_valid_.store(false);
        last_ofdm_broadband_snr_db_.store(0.0f);
        last_evm_snr_db_valid_.store(false);
        last_evm_snr_db_.store(0.0f);
        LOG_MODEM(DEBUG, "[%s] non-OFDM quality: sync_quality=%.1f dB "
                  "idle_in_band=%s%.1f dB routed_snr=%.1f dB (%s) windows=%zu",
                  log_prefix_.c_str(), result.sync_quality_db,
                  result.has_idle_in_band_snr_db ? "" : "unavailable/",
                  result.idle_in_band_snr_db, result.snr_db,
                  snrSourceToString(result.snr_source), idle_snr.windows_observed);

        // ═══ SNR-SANITY (handoff §4): usable can NEVER exceed channel ═══
        // The demod-usable (effective) SNR is the channel's physical S:N minus
        // implementation loss — physics forbids usable > channel. When this
        // frame carries BOTH a routed usable reading and the per-frame physical
        // measurement, a violation beyond the combined measurement slop means
        // ONE OF THE METERS IS BROKEN for this frame: log it and downgrade the
        // routed source to SYNC_QUALITY, which acceptsRateSelectionSNR already
        // rejects — the suspect reading cannot steer a rate decision. One
        // frame, never a latch. Margin 2.0 dB = the physical readout's known
        // −0.66 dB noise-ref residual + both estimators' per-frame variance;
        // tighten alongside the §2 residual fix. This exact invariant caught
        // the pre-recalibration OFDM optimism from a single operator log line.
        // ATTRIBUTION THRESHOLD (rig F238, 2026-07-13): the gate may only
        // EXCLUDE a reading when its physical reference is a DISTRIBUTION
        // (>=3 ring samples — the display's own trust bar), not a single
        // snapshot. At the handshake the ring holds 1-2 samples in an unknown
        // fade state; excluding the connect frame's ONLY reading left the
        // entry pick blind (protocol default 15.0/source-none -> DQPSK
        // fallback). Single-sample disagreement is logged by the WARN below
        // when it trips with n>=3; below that, silence — a snapshot cannot
        // convict a meter.
        // F241 (2026-07-13): two hard-won rules for exclusion authority.
        // (1) NEVER pre-connected — at the handshake the physical entry CAP
        //     already bounds a hot reading physically, so exclusion buys
        //     nothing there and a false conviction blinds the entry pick
        //     entirely (15.0/source-none default -> DQPSK fallback; F238/239/
        //     241 all died this way, each against a single trough/ref-glitch
        //     physical sample).
        // (2) Convict against the ring DISTRIBUTION (mean + spread + margin),
        //     never a single sample — on fading, usable and any one physical
        //     sample are different fade instants; the invariant usable<=channel
        //     only binds same-moment or distribution-vs-distribution.
        float ph_mean = 0.0f, ph_spread = 0.0f;
        const size_t ph_n = physicalSnrStats(ph_mean, ph_spread);
        if (result.snr_source == SNRSource::MCDPSK_IN_BAND &&
            connected_ &&
            ph_n >= 3 &&
            result.snr_db > ph_mean + ph_spread + 2.0f) {
            LOG_MODEM(WARN,
                      "[%s] SNR-SANITY: usable %.1f dB EXCEEDS channel "
                      "%.1f±%.1f dB (n=%zu, margin 2.0) — meter suspect, "
                      "reading excluded from rate selection for this frame",
                      log_prefix_.c_str(), result.snr_db, ph_mean, ph_spread,
                      ph_n);
            result.snr_source = SNRSource::SYNC_QUALITY;
        }
    }
}

void StreamingDecoder::armProvisionalBurstAirGate(uint64_t anchor_abs,
                                                  bool proven_full_chirp) {
    const uint64_t guard_samples =
        static_cast<uint64_t>(
            protocol::connection_policy::maxWideOFDMPhysicalTurnAfterAnchorTrainingMs()) *
        static_cast<uint64_t>(protocol::connection_policy::kOFDMSampleRate) / 1000u;
    provisional_burst_anchor_abs_.store(anchor_abs, std::memory_order_relaxed);
    provisional_burst_anchor_full_chirp_.store(proven_full_chirp,
                                                std::memory_order_relaxed);
    burst_air_end_abs_.store(anchor_abs + guard_samples, std::memory_order_relaxed);
    burst_air_end_provisional_.store(true, std::memory_order_relaxed);
}

void StreamingDecoder::clearProvisionalBurstAirGate() {
    if (burst_air_end_provisional_.exchange(false, std::memory_order_relaxed)) {
        provisional_burst_anchor_abs_.store(0, std::memory_order_relaxed);
        provisional_burst_anchor_full_chirp_.store(false, std::memory_order_relaxed);
        burst_air_end_abs_.store(0, std::memory_order_relaxed);
        // The same accepted full anchor arms the no-group backstop.  Reaching
        // this helper means a standalone physical transmission was positively
        // completed (classic frame at that anchor, physical tail, or control),
        // so a synthetic crater ACK ~14.6 s later would be false and can collide
        // with the peer's next turn.  Only disarm when the provisional gate was
        // actually live; later light non-FINAL frames intentionally keep both.
        anchored_burst_backstop_armed_ = false;
        anchored_burst_payload_seen_ = false;
    }
}

void StreamingDecoder::searchForSync() {
    if (!waveform_) return;

    // ACK-LISTEN tone-lock guard (ULTRA_ACKLISTEN_SUPPRESS_OFDM): while this station's
    // tone-burst ACK monitor is armed (we sent a burst and await the peer's 4-FSK ACK —
    // half-duplex, the peer cannot be sending OFDM), suppress the warm DATA-sync
    // acceptance paths in the SyncController so the ACK tone cannot S&C-false-lock the
    // OFDM searcher and race the tone monitor for the same samples (the F73/F74
    // missed-ACK spirals). The dual-chirp path stays live. Refreshed every search
    // pass; disarms automatically with the monitor.
    static const bool kAckListenSuppressOfdm = [] {
        const char* e = std::getenv("ULTRA_ACKLISTEN_SUPPRESS_OFDM");
        // DEFAULT-ON 2026-07-05 (=0 opts out): rig-proven — F75 record 2.62 kbps +
        // 10-run batch F78-F87 (~280 ack exchanges, 0 misses, 0 expired-undetected);
        // half-duplex-provably safe (the peer cannot send OFDM during our ACK window).
        return !(e && e[0] == '0');
    }();
    sync_controller_.setAckListenSuppressDataSync(
        kAckListenSuppressOfdm && connected_ &&
        mode_ == protocol::WaveformMode::OFDM_CHIRP && tone_burst_monitor_.isArmed());

    // Save generation counter - if reset() is called during our search,
    // we'll detect it and discard our results
    uint32_t gen_at_start = reset_generation_.load();

    // Get preamble size from waveform
    size_t preamble = static_cast<size_t>(waveform_->getPreambleSamples());

    // Search buffer sizing depends on sync mode:
    // - Chirp sync (disconnected): needs ~120k samples for dual chirp correlation
    // - Light sync (connected, LTS): needs only ~24k samples (LTS is ~1024 samples)
    // Using a smaller buffer when connected cuts per-hop latency from ~2.8s to <1s
    constexpr size_t CHIRP_MAX_SEARCH = 120000;   // ~2.5s for dual chirp detection
    constexpr size_t LIGHT_SEARCH_SIZE = 9600;    // ~0.20s for connected LTS-only detection

    // F165 ANCHORED-BURST ACK BACKSTOP: fire once when the sample clock passes
    // the max group window since an accepted expected anchor with nothing
    // framed (descriptor unreadable in a deep fade -> no group -> no ack -> the
    // sender RTO-starves at a pinned rung). ~12.5 s covers descriptor + anchor
    // + an 8-frame group; the sender's RTO fires ~11 s AFTER its burst ends
    // (~anchor+21 s), so the backstop ack beats it comfortably.
    if (anchored_burst_backstop_armed_) {
        size_t fed_now;
        {
            std::lock_guard<std::mutex> abl(sync_controller_.ring_.buffer_mutex_);
            fed_now = sync_controller_.ring_.total_fed_;
        }
        // DERIVED from the burst policy (GROUP-SIZE co-fix #1): the ceiling
        // clamp maxes at 12,000 ms of group air + 1,410 ms full descriptor +
        // ~1.2 s margin ≈ 14.6 s. A fixed 600k (12.5 s) fired MID-BURST once
        // groups exceed ~10 frames — the adaptivity-bug class. Still beats the
        // sender RTO (~air-end + 11.4 s) by ~9 s.
        constexpr size_t kBackstopWindowSamples = 48 * (12000 + 1410 + 1200);
        if (fed_now > anchored_burst_backstop_arm_abs_ + kBackstopWindowSamples) {
            const bool payload_seen = anchored_burst_payload_seen_;
            anchored_burst_backstop_armed_ = false;
            anchored_burst_payload_seen_ = false;
            LOG_MODEM(WARN,
                      "[%s] ANCHORED-BURST BACKSTOP: expected anchor framed no "
                      "group within the window — requesting re-confirm ack "
                      "(payload_seen=%d)",
                      log_prefix_.c_str(), payload_seen ? 1 : 0);
            // Provenance for the frontend's listen-before-ACK: the ACK this callback
            // produces was requested by a TIMER, not by a decode (INV-TONE-ACK-005).
            stampAnchoredBackstop();
            if (anchored_burst_no_group_callback_) {
                anchored_burst_no_group_callback_(payload_seen);
            }
        }
    }
    size_t chirp_min_search = std::min(preamble + 65000, CHIRP_MAX_SEARCH);
    bool connected_data_preamble = connected_ && waveform_->supportsDataPreamble();
    bool use_full_ofdm_anchor_search =
        connected_data_preamble && sync_controller_.expect_full_ofdm_anchor_ &&
        mode_ == protocol::WaveformMode::OFDM_CHIRP;
    // F147: each search attempt re-derives the expected-anchor mark; only the
    // connected full-anchor accept below sets it.
    last_sync_expected_full_anchor_ = false;
    bool use_light_search = connected_data_preamble && !use_full_ofdm_anchor_search;
    bool used_full_anchor_fallback = false;
    // STRUCTURE ACQUISITION (ULTRA_BURST_ACQ=structure, lockstep with the sender's chirp-less
    // groups). The connected GROUP-START search — the one that waits for a dual chirp in chirp
    // mode — instead runs the chi frame-structure detector on a SHORT window
    // (SyncController::structureSearchWindowSamples: every frame is wholly inside the first
    // two windows that see it), so a group start is acquired ~0.45 s after it begins instead of
    // after a 2.5 s chirp-search window. In-group frames keep the warm light-LTS path unchanged.
    // The handshake-confirmation window keeps the chirp search: the half-open MC-DPSK CONNECT
    // fallback needs a real dual-chirp lock (MC-DPSK has no OFDM structure to detect).
    const size_t structure_span =
        (use_full_ofdm_anchor_search && sync::structureAcquisitionEnabled())
            ? waveform_->frameStructureSpanSamples()
            : 0;
    const bool structure_anchor_search =
        structure_span > 0 && !handshake_confirmation_pending_.load(std::memory_order_acquire);
    bool used_structure_anchor = false;
    // (R4: the adaptive short-chirp re-anchor was removed — superseded by warm-handoff,
    // which is now the default. The light/full-anchor search are the only group-boundary paths.)
    //
    // Buffer sizing by search type:
    //  - light LTS: small (latency-optimized; the LTS is ~1k samples).
    //  - full dual-chirp anchor: MUST be the full chirp search size. The reduced
    //    connected window (preamble+65k ≈ 96k) is a light-LTS latency optimization and
    //    is WRONG for dual-chirp detection: when the search position lags live audio
    //    (e.g. after a burst group, BRAVO trails ~2.4s), the up-chirp lands deep in the
    //    window and the down-chirp (≈28.8k samples later) falls outside a 96k buffer →
    //    detectDualChirp reports "down NOT found" and MISSes despite a 0.99 up-chirp.
    //    This stranded the post-burst Winlink-B2F FF/turnaround frame (BUG-TNC-B2F turnaround).
    //  - disconnected chirp: the existing chirp_min_search (no lag at connect time).
    size_t min_search =
        use_light_search
            ? LIGHT_SEARCH_SIZE
            : (structure_anchor_search
                   ? sync::SyncController::structureSearchWindowSamples(structure_span)
                   : (use_full_ofdm_anchor_search ? CHIRP_MAX_SEARCH : chirp_min_search));
    const size_t data_symbol_samples =
        (use_light_search && waveform_)
            ? static_cast<size_t>(std::max(1, waveform_->getSamplesPerSymbol()))
            : 0;

    const bool disconnected_mc_dpsk =
        !connected_ && mode_ == protocol::WaveformMode::MC_DPSK;
    std::vector<float> search_buffer;
    size_t search_start = 0;
    size_t search_start_abs = 0;
    bool used_warm_timed_window = false;
    bool used_warm_narrow_window = false;
    size_t warm_narrow_end_abs = 0;
    size_t warm_narrow_candidate_span_samples = 0;
    bool replaying_deferred_future_sync = false;
    size_t pre_floor_lead = 0;

    // A previous detector pass may have proved the chirps while announcing a
    // training start just beyond live audio. Do not advance/search elsewhere in
    // the meantime. Once that exact absolute position arrives, replay the exact
    // immutable detector window; the candidate is re-found without reading any
    // unwritten ring memory or relying on a modular cursor reconstruction.
    {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        if (deferred_future_sync_.valid) {
            if (deferred_future_sync_.generation != gen_at_start) {
                deferred_future_sync_ = {};
            } else if (sync_controller_.ring_.total_fed_ <
                       deferred_future_sync_.wait_until_abs) {
                return;
            } else {
                search_buffer = std::move(deferred_future_sync_.search_buffer);
                search_start_abs = deferred_future_sync_.search_start_abs;
                search_start = sync_controller_.ring_.absoluteToRingLocked(search_start_abs);
                min_search = search_buffer.size();
                used_warm_timed_window =
                    deferred_future_sync_.used_warm_timed_window;
                used_warm_narrow_window =
                    deferred_future_sync_.used_warm_narrow_window;
                warm_narrow_end_abs = deferred_future_sync_.warm_narrow_end_abs;
                warm_narrow_candidate_span_samples =
                    deferred_future_sync_.warm_narrow_candidate_span_samples;
                deferred_future_sync_ = {};
                replaying_deferred_future_sync = true;
            }
        }
    }

    if (!replaying_deferred_future_sync) {
        auto win = sync_controller_.acquireSearchWindow(
            use_light_search, connected_data_preamble, disconnected_mc_dpsk,
            min_search, data_symbol_samples,
            audio_activity_.load(std::memory_order_relaxed),
            CORRELATION_STEP, CORR_NOISE_THRESHOLD);
        if (!win.ready) return;
        search_buffer = std::move(win.search_buffer);
        search_start = win.search_start;
        search_start_abs = win.search_start_abs;
        min_search = win.min_search;
        used_warm_timed_window = win.used_warm_timed_window;
        used_warm_narrow_window = win.used_warm_narrow_window;
        warm_narrow_end_abs = win.warm_narrow_end_abs;
        warm_narrow_candidate_span_samples = win.warm_narrow_candidate_span_samples;
        pre_floor_lead = std::min(win.pre_floor_lead_samples, search_buffer.size());
    }

    // DEBUG: Dump the search buffer on first few searches
    static int search_dump_count = 0;
    if (g_debug_dumps_enabled && search_dump_count < 5) {
        char label[64];
        snprintf(label, sizeof(label), "search_%d_pos%zu", search_dump_count, search_start);

        // Dump search buffer to file
        char filename[256];
        snprintf(filename, sizeof(filename), "%s_%s.f32", g_dump_prefix, label);
        std::ofstream file(filename, std::ios::binary);
        if (file) {
            file.write(reinterpret_cast<const char*>(search_buffer.data()),
                       search_buffer.size() * sizeof(float));
            file.close();

            // Compute stats
            float rms = 0, max_val = 0;
            for (size_t i = 0; i < std::min(search_buffer.size(), size_t(10000)); i++) {
                rms += search_buffer[i] * search_buffer[i];
                max_val = std::max(max_val, std::abs(search_buffer[i]));
            }
            rms = std::sqrt(rms / std::min(search_buffer.size(), size_t(10000)));

            LOG_MODEM(DEBUG, "StreamingDecoder: Search #%d: dumped %zu samples to %s (start=%zu, RMS=%.4f, peak=%.4f)",
                      search_dump_count, search_buffer.size(), filename, search_start, rms, max_val);
        }
        search_dump_count++;
    }

    // Search for sync (no lock held - this is the slow part)
    auto search_start_time = std::chrono::steady_clock::now();

    waveform_->reset();
    SyncResult sync_result;
    bool found = false;
    bool narrow_sync_found = false;

    // When connected, use light sync (LTS training symbols, no chirp) after the
    // first connected OFDM frame has established an OFDM-specific chirp+LTS anchor.
    // Reject false positives where data autocorrelation produces spurious peaks
    // (observed up to 0.63). Real LTS correlation is always >0.81 even on
    // moderate fading.
    // Coherent modes need higher sync quality — badly-synced frames always fail
    // because stale LTS phases can't be recovered by DD tracking alone.
    const bool is_coherent = (current_modulation_ == Modulation::QPSK ||
                              current_modulation_ == Modulation::BPSK);
    // Narrowband LTS has ~35% of wideband energy (21 vs 59 carriers) → lower correlation peak
    const bool is_narrowband = (mode_ == protocol::WaveformMode::OFDM_NARROW);
    // LTS sync thresholds.
    // True LTS peaks: 0.85-0.99 (clean), but Moderate SNR12 hardware traces
    // show real tail DATA can dip to ~0.52-0.56. Data autocorrelation noise is
    // usually 0.20-0.45; candidates admitted near the low end still pass the
    // downstream LLR/LDPC gates before they can be accepted as frames.
    const size_t light_sync_candidate_window_samples =
        used_warm_narrow_window ? warm_narrow_candidate_span_samples : LIGHT_SEARCH_SIZE;
    const auto light_sync_thresholds = signal_policy::lightSyncThresholds(
        is_coherent, is_narrowband, connected_, sync_controller_.syncRejectStreak(),
        used_warm_narrow_window, LIGHT_SEARCH_SIZE,
        light_sync_candidate_window_samples);

    if (use_light_search) {
        // §7 C3 Phase 3b: the connected-data light-LTS DETECTION + acceptance + §16.4 escalation
        // now lives on the controller (detectConnectedLightSync); the decoder passes its current
        // waveform + the search window and fires the data-sync-accepted callback on success.
        found = sync_controller_.detectConnectedLightSync(
            waveform_.get(), search_buffer.data(), search_buffer.size(), search_start,
            is_coherent, connected_, mode_, light_sync_thresholds, CORR_DETECT_THRESHOLD,
            cfo_tracker_.tracked(), sync_result);
        // Short re-anchor fallback is enabled only by negotiated fading class;
        // otherwise the connected path remains LTS-only.
    } else if (structure_anchor_search) {
        // One search pass per CORRELATION_STEP of new audio: a per-pass false-lock budget of
        // step / (1 h of audio) bounds false group-start locks to one per hour of continuous
        // connected searching (a probability constant; the detector splits it over its tests).
        constexpr double kStructureFalseLocksPerHour = 1.0;
        const double false_alarm_budget =
            kStructureFalseLocksPerHour * static_cast<double>(CORRELATION_STEP) /
            (3600.0 * static_cast<double>(protocol::connection_policy::kOFDMSampleRate));
        auto st = sync_controller_.detectStructureAnchor(
            waveform_.get(), search_buffer.data(), search_buffer.size(), connected_,
            cfo_tracker_.tracked(), false_alarm_budget, sync_result);
        found = st.found;
        used_structure_anchor = st.found;
    } else {
        // Use full sync detection with chirp (wideband)
        found = waveform_->detectSync(
            SampleSpan(search_buffer.data(), search_buffer.size()),
            sync_result, CORR_DETECT_THRESHOLD);

        if (!found && use_full_ofdm_anchor_search) {
            // §7 C3 Phase 3b: the full-anchor light-LTS fallback (the §16 phase-5 trace +
            // detectDataSync + control-threshold evaluation + streak update + weak-accept) lives on
            // the controller; the decoder passes its waveform + the search window and fires the
            // data-sync-accepted callback on success.
            // The LTS-type fallback keeps the strict post-frame floor: skip the chirp-only lead
            // (false LTS locks in a decoded frame's tail are what the floor exists to stop).
            auto fb = sync_controller_.detectFullAnchorFallback(
                waveform_.get(), search_buffer.data() + pre_floor_lead,
                search_buffer.size() - pre_floor_lead, search_start,
                min_search, is_narrowband, connected_, CORR_DETECT_THRESHOLD,
                cfo_tracker_.tracked(), sync_result);
            if (fb.found) {
                found = true;
                used_full_anchor_fallback = fb.used_full_anchor_fallback;
                sync_result.start_sample += static_cast<int>(pre_floor_lead);
            }
        }

        // Dual-listen: if wideband didn't find anything, try narrowband chirp
        if (!found && !connected_) {
            // Lazy-init narrowband waveform on first use
            if (!narrow_waveform_initialized_) {
                // Create narrowband MC-DPSK waveform for chirp detection
                // Uses 1250-1750 Hz chirp, 4 carriers @ 1300-1700 Hz
                narrow_waveform_ = WaveformFactory::createNarrowbandMCDPSK();
                narrow_waveform_initialized_ = true;
                LOG_MODEM(INFO, "[%s] Dual-listen: narrowband MC-DPSK waveform initialized",
                          log_prefix_.c_str());
            }

            if (narrow_waveform_) {
                SyncResult narrow_result;
                bool narrow_found = narrow_waveform_->detectSync(
                    SampleSpan(search_buffer.data(), search_buffer.size()),
                    narrow_result, CORR_DETECT_THRESHOLD);

                if (narrow_found) {
                    found = true;
                    narrow_sync_found = true;
                    sync_result = narrow_result;
                }
            }
        }
    }

    auto search_end_time = std::chrono::steady_clock::now();
    float search_ms = std::chrono::duration<float, std::milli>(search_end_time - search_start_time).count();

    // Check if reset() was called during our search - if so, discard results
    if (reset_generation_.load() != gen_at_start) {
        LOG_MODEM(INFO, "[%s] searchForSync: ABORTED - reset() called during search", log_prefix_.c_str());
        return;
    }

    signal_policy::TrainingStartDecision training_start;
    if (found) {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
        training_start = signal_policy::classifyDetectedTrainingStart(
            search_start_abs, sync_result.start_sample,
            sync_controller_.ring_.total_fed_);
        if (training_start.disposition ==
            signal_policy::TrainingStartDisposition::INVALID) {
            LOG_MODEM(ERROR,
                      "[%s] searchForSync: detector returned invalid training offset=%d "
                      "for search_start_abs=%zu; rejecting candidate",
                      log_prefix_.c_str(), sync_result.start_sample, search_start_abs);
            found = false;
        } else if (training_start.disposition ==
                   signal_policy::TrainingStartDisposition::DEFER) {
            // Preserve the detector's immutable input instead of preserving a
            // modular cursor. The latter can wrap or be load-shed while we wait;
            // the former deterministically re-finds the exact chirp candidate.
            deferred_future_sync_ = {};
            deferred_future_sync_.valid = true;
            deferred_future_sync_.generation = gen_at_start;
            deferred_future_sync_.wait_until_abs = training_start.detected_abs;
            deferred_future_sync_.search_start_abs = search_start_abs;
            deferred_future_sync_.search_buffer = std::move(search_buffer);
            deferred_future_sync_.used_warm_timed_window = used_warm_timed_window;
            deferred_future_sync_.used_warm_narrow_window = used_warm_narrow_window;
            deferred_future_sync_.warm_narrow_end_abs = warm_narrow_end_abs;
            deferred_future_sync_.warm_narrow_candidate_span_samples =
                warm_narrow_candidate_span_samples;
            LOG_MODEM(INFO,
                      "[%s] searchForSync: detector training starts after live audio "
                      "(abs=%zu total_fed=%zu missing=%zu); deferring exact-window "
                      "replay",
                      log_prefix_.c_str(), training_start.detected_abs,
                      sync_controller_.ring_.total_fed_,
                      training_start.samples_missing);
            return;
        }
    }

    if (found) {
        if (narrow_sync_found) {
            detected_bandwidth_ = BandwidthMode::NARROW;
            // Switch only after the training position is live. A deferred lock
            // must not mutate the active decoder before it is accepted.
            waveform_ = WaveformFactory::createNarrowbandMCDPSK();
            LOG_MODEM(INFO,
                      "[%s] Dual-listen: NARROWBAND chirp detected! corr=%.3f, "
                      "CFO=%.1f Hz, switched to narrowband MC-DPSK",
                      log_prefix_.c_str(), sync_result.correlation,
                      sync_result.cfo_hz);
        } else if (detected_bandwidth_ != BandwidthMode::NARROW) {
            detected_bandwidth_ = BandwidthMode::WIDE;
        }

        const bool expected_full_anchor_found =
            use_full_ofdm_anchor_search && !used_full_anchor_fallback;
        if (expected_full_anchor_found) {
            LOG_MODEM(INFO,
                      "[%s] Full OFDM anchor sync detected while connected (corr=%.2f)",
                      log_prefix_.c_str(), sync_result.correlation);
            // Only an accepted, live training position may arm group/backstop
            // side effects. The former clamp fired these early for a candidate
            // whose training did not exist in the ring yet.
            last_sync_expected_full_anchor_ = true;
            anchored_burst_backstop_armed_ = true;
            anchored_burst_payload_seen_ = false;
            {
                std::lock_guard<std::mutex> lock(
                    sync_controller_.ring_.buffer_mutex_);
                anchored_burst_backstop_arm_abs_ =
                    sync_controller_.ring_.total_fed_;
            }
        }
        if (connected_data_preamble && data_sync_accepted_callback_) {
            data_sync_accepted_callback_(sync_result.correlation);
        }
    }

    // The acceptance callback is allowed to trigger a lifecycle transition.
    // It used to run before the generation check above; keep the same safety
    // contract after delaying the callback until the training position is live.
    if (reset_generation_.load() != gen_at_start) {
        LOG_MODEM(INFO,
                  "[%s] searchForSync: ABORTED - reset() called by sync acceptance",
                  log_prefix_.c_str());
        return;
    }

    // Log timing: sync_controller_.ring_.total_fed_ tells us how much audio has arrived
    float audio_sec = sync_controller_.ring_.total_fed_ / 48000.0f;
    if (found || search_ms > 100) {  // Log if found or if search was slow
        LOG_MODEM(INFO, "[%s] searchForSync: audio=%.2fs, search=%.1fms, found=%d, corr=%.3f",
                  log_prefix_.c_str(), audio_sec, search_ms, found ? 1 : 0, sync_result.correlation);
    }

    if (found) {
        // #70: the audio BEFORE the detected up-chirp START is ambient by
        // construction — feed it to the idle-noise estimator (state is still
        // SEARCHING here, so the candidate gate accepts it). The no-lock branch
        // below only observes after a full search pass finds nothing, which
        // never happens for a station that hears the peer's probe on its very
        // first pass (fresh boot answering a handshake) — leaving the
        // noise-relative PING gap test without a floor reference exactly when
        // the responder needs it. Uses preamble_start_sample (frame start), NOT
        // start_sample (training start — the two chirps before it are full-level
        // signal and would poison the floor 2-3x high). BOOT-ONLY (hasEstimate
        // gate): in steady state the true-idle passes are the clean source; a
        // prefix can contain an undetected earlier transmission's tail, which
        // would over-read the floor and widen the false-PING window. Cap at
        // 1 s: five estimator windows, enough to go valid in one shot.
        if (sync_result.preamble_start_sample > 0 &&
            !idle_noise_snr_estimator_.hasEstimate()) {
            const size_t chirp_off = std::min(
                static_cast<size_t>(sync_result.preamble_start_sample),
                search_buffer.size());
            const size_t prefix = std::min(chirp_off, static_cast<size_t>(48000));
            observeIdleNoiseCandidate(
                search_buffer.data() + (chirp_off - prefix), prefix);
        }

        // Burst-time noise reference: in-band RMS of the silent inter-chirp gap
        // of THIS frame. Correct on constant-noise channels (real radio, OTASim)
        // AND on S:N-holding channel sims whose noise level tracks the signal
        // (idle floor != burst floor there — F224). Center 60% of the gap only:
        // the skipped leading edge doubles as FIR-transient priming and dodges
        // chirp ringing + multipath tails; the trailing edge dodges timing error.
        sync_noise_ref_rms_ = 0.0f;
        if (sync_result.interchirp_gap_len > 0 &&
            sync_result.interchirp_gap_start_sample >= 0) {
            const size_t gs = std::min(
                static_cast<size_t>(sync_result.interchirp_gap_start_sample),
                search_buffer.size());
            const size_t glen = std::min(
                static_cast<size_t>(sync_result.interchirp_gap_len),
                search_buffer.size() - gs);
            const size_t skip = glen / 5;
            if (glen > 2 * skip + 480) {  // >=10 ms usable
                FIRFilter ref_filter =
                    FIRFilter::bandpass(101, 50.0f, 2950.0f, 48000.0f);
                for (size_t i = gs; i < gs + skip; ++i) {
                    ref_filter.process(search_buffer[i]);
                }
                double sum_sq = 0.0;
                const size_t meas_end = gs + glen - skip;
                for (size_t i = gs + skip; i < meas_end; ++i) {
                    const float y = ref_filter.process(search_buffer[i]);
                    sum_sq += static_cast<double>(y) * static_cast<double>(y);
                }
                sync_noise_ref_rms_ = static_cast<float>(
                    std::sqrt(sum_sq / static_cast<double>(meas_end - gs - skip)));
                // Handoff §6 second half (F233 trough gap): the CHIRP span is
                // a channel power sample that survives fade nulls — the
                // data-span physical correctly abstains in a trough (its
                // anti-noise gate), which left the entry cap blind exactly
                // where the affine boost is most dangerous (F233 entered off
                // a wire-0 trough with an empty ring). Every dual-chirp anchor
                // (PING, PONG, CONNECT, CONNECT_ACK) now contributes one
                // sample: in-band power of the up-chirp span vs THIS frame's
                // gap noise ref, latched only ≥3 dB above it (a false lock on
                // noise reads ~0 and is refused). Convention: chirp-referenced
                // ≈ dial within ~1 dB (the reference power is chirp-dominated;
                // the ring's consumers carry a 2 dB margin).
                if (sync_result.preamble_start_sample >= 0 &&
                    sync_noise_ref_rms_ > 0.0f) {
                    const size_t cs = std::min(
                        static_cast<size_t>(sync_result.preamble_start_sample),
                        search_buffer.size());
                    const size_t clen = std::min(
                        static_cast<size_t>(sync_result.interchirp_gap_start_sample >
                                                sync_result.preamble_start_sample
                                            ? sync_result.interchirp_gap_start_sample -
                                                  sync_result.preamble_start_sample
                                            : 0),
                        search_buffer.size() - cs);
                    if (clen > 4800) {  // >=0.1 s of chirp
                        FIRFilter chirp_filter =
                            FIRFilter::bandpass(101, 50.0f, 2950.0f, 48000.0f);
                        const size_t skip = clen / 10;
                        for (size_t i = cs; i < cs + skip; ++i) {
                            chirp_filter.process(search_buffer[i]);
                        }
                        double sum_sq = 0.0;
                        for (size_t i = cs + skip; i < cs + clen; ++i) {
                            const float y = chirp_filter.process(search_buffer[i]);
                            sum_sq += static_cast<double>(y) * y;
                        }
                        const double p_c =
                            sum_sq / static_cast<double>(clen - skip);
                        const double p_n =
                            static_cast<double>(sync_noise_ref_rms_) *
                            static_cast<double>(sync_noise_ref_rms_);
                        if (p_n > 0.0 && p_c > 2.0 * p_n) {
                            notePhysicalSnrSample(static_cast<float>(
                                10.0 * std::log10((p_c - p_n) / p_n)));
                        }
                    }
                }
                // Handoff §2: hand THIS frame's burst-time noise reference to
                // the waveform so its training-span decode computes the
                // physical channel SNR (per-frame — no stale latch).
                // ROBUSTIFIED (F231, measured on the IONOS bench): that box's
                // S:N-tracking noise BREATHES around bursts — the inter-chirp
                // gap measured 0.021-0.035 vs a steady idle floor of 0.044
                // (up to 6 dB quieter, 2x frame-to-frame swing), which made
                // the physical readout unstable there. A noise estimate can
                // read falsely LOW (tracker decay in silence) but the
                // min-statistics idle floor bounds it from below, so the
                // reference is the MAX of the two independent measurements.
                // On constant-noise channels (real radios, OTASim) gap == floor
                // and this is a no-op.
                if (waveform_) {
                    float ref = sync_noise_ref_rms_;
                    const auto ns = idle_noise_snr_estimator_.snapshot();
                    if (ns.valid && ns.floor_noise_rms > ref) {
                        ref = ns.floor_noise_rms;
                    }
                    waveform_->setNoiseReferenceRMS(ref);
                }
                LOG_MODEM(INFO,
                          "[%s] burst-noise ref: rms=%.4f gap=[%zu+%zu..%zu) of buf=%zu",
                          log_prefix_.c_str(), sync_noise_ref_rms_, gs, skip,
                          meas_end, search_buffer.size());
            }
        }

        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);

        // training_start was classified against total_fed_ before any accept
        // side effect. READY is therefore a proof that this exact absolute
        // sample is live; no clamp or modular-distance guess is involved.
        sync_position_ = sync_controller_.ring_.absoluteToRingLocked(
            training_start.detected_abs);

        const bool timing_cfo_genie =
            qam16GenieTimingCfoEnabled() &&
            connected_data_preamble &&
            current_modulation_ == Modulation::QAM16 &&
            use_light_search &&
            used_warm_timed_window &&
            sync_controller_.next_expected_frame_sample_valid_;
        if (timing_cfo_genie) {
            const size_t detected_abs = sync_controller_.ring_.ringPosToAbsoluteLocked(sync_position_);
            const size_t expected_abs = sync_controller_.next_expected_frame_sample_;
            sync_position_ = sync_controller_.ring_.absoluteToRingLocked(expected_abs);
            LOG_MODEM(WARN,
                      "[%s] DIAG genie-timing-cfo: overriding light-LTS sync "
                      "detected_abs=%zu expected_abs=%zu delta=%lld corr=%.3f",
                      log_prefix_.c_str(),
                      detected_abs,
                      expected_abs,
                      static_cast<long long>(
                          arrival_policy::signedSampleError(detected_abs, expected_abs)),
                      sync_result.correlation);
        }

        // Anti-replay: reject sync at same position as last decoded frame (circular distance)
        if (last_decoded_sync_pos_ != SIZE_MAX) {
            size_t d1 = (sync_position_ >= last_decoded_sync_pos_)
                ? (sync_position_ - last_decoded_sync_pos_)
                : (sync_controller_.ring_.buffer_capacity_samples_ - last_decoded_sync_pos_ + sync_position_);
            size_t dist = std::min(d1, sync_controller_.ring_.buffer_capacity_samples_ - d1);
            if (dist < 200) {
                LOG_MODEM(INFO, "[%s] Anti-replay: duplicate sync at pos=%zu (prev=%zu), skipping",
                          log_prefix_.c_str(), sync_position_, last_decoded_sync_pos_);
                constexpr size_t SEARCH_BACKTRACK = 9600;
                sync_controller_.ring_.correlation_pos_ = sync_controller_.ring_.wrapRingIndexLocked(sync_position_ + SEARCH_BACKTRACK + CORRELATION_STEP);
                return;
            }
        }

        // Provide absolute training position to waveform so initial CFO phase is aligned.
        const size_t abs_training_pos =
            sync_controller_.ring_.ringPosToAbsoluteLocked(sync_position_);
        if (waveform_) {
            waveform_->setAbsoluteTrainingPosition(abs_training_pos);
        }

        // BUG-HALFOPEN-CONNECT: during the responder's half-open interval the
        // expected first OFDM full anchor may instead be the initiator repeating
        // its fixed-profile MC-DPSK CONNECT because CONNECT_ACK was lost.  Both
        // PHYs intentionally share the dual chirps and therefore the exact
        // training origin.  Retain only a REAL expected dual-chirp lock (never a
        // weak LTS/data fallback); processBuffer will make one bounded raw-ring
        // MC decode after all four fixed codewords have arrived.
        if (last_sync_expected_full_anchor_ && !used_structure_anchor &&
            handshake_confirmation_pending_.load(std::memory_order_acquire)) {
            MultiCarrierDPSKConfig fallback_config = mc_dpsk_config_;
            fallback_config.bits_per_symbol = 1;  // fixed handshake profile = DBPSK
            const size_t coded_bits =
                static_cast<size_t>(v2::kDefaultFixedFrameCodewords) *
                v2::LDPC_CODEWORD_BITS;
            const size_t bits_per_symbol = static_cast<size_t>(
                std::max(1, fallback_config.num_carriers));
            const size_t data_symbols =
                (coded_bits + bits_per_symbol - 1) / bits_per_symbol;
            const size_t required_samples =
                (static_cast<size_t>(std::max(0, fallback_config.training_symbols)) +
                 1 + data_symbols) *
                static_cast<size_t>(
                    std::max(1, fallback_config.samples_per_symbol));

            pending_handshake_mcdpsk_connect_.valid = true;
            pending_handshake_mcdpsk_connect_.generation = gen_at_start;
            pending_handshake_mcdpsk_connect_.training_abs = abs_training_pos;
            pending_handshake_mcdpsk_connect_.required_samples = required_samples;
            pending_handshake_mcdpsk_connect_.cfo_hz = sync_result.cfo_hz;
            pending_handshake_mcdpsk_connect_.correlation =
                sync_result.correlation;
            pending_handshake_mcdpsk_connect_.config = fallback_config;
            pending_handshake_mcdpsk_connect_.measurements =
                half_open_measurement_snapshot_;
            LOG_MODEM(INFO,
                      "[%s] Half-open CONNECT fallback armed at training_abs=%zu "
                      "for %zu raw samples (corr=%.3f CFO=%.2f)",
                      log_prefix_.c_str(), abs_training_pos, required_samples,
                      sync_result.correlation, sync_result.cfo_hz);
        }
        const bool provisional_already_active =
            burst_air_end_provisional_.load(std::memory_order_relaxed);
        if (use_full_ofdm_anchor_search &&
            (last_sync_expected_full_anchor_ || !provisional_already_active)) {
            // The descriptor and the marked group head can both be lost while later
            // duplicate DATA frames still decode through the classic per-frame path.
            // Publish a conservative physical-air gate as soon as the expected full
            // anchor is accepted; exact group geometry will replace it if framing
            // succeeds. A later full-search LTS fallback must NOT move an already-live
            // gate to that duplicate — its deadline stays anchored to the original
            // physical turn. A genuine new chirp lock (`last_sync_expected...`) may
            // replace it. Thus one early decode cannot key an ACK mid-burst.
            armProvisionalBurstAirGate(
                static_cast<uint64_t>(abs_training_pos),
                /*proven_full_chirp=*/last_sync_expected_full_anchor_);
        }

        // CFO handling: On fading channels, chirp-based CFO measurement can be corrupted
        // by multipath (peaks shift differently for up vs down chirp).
        // When connected, trust the established CFO and limit drift.
        // §7 C-CFO-2: the chirp-CFO drift clamp now lives on the tracker — it reads its own tracked
        // value as `known` and logs the clamp. On fading a multipath-distorted chirp can read a false
        // CFO, so the per-frame drift is clamped to the established estimate.
        // BUG-ANCHOR-CFO-KILL (2026-07-05): the CONNECTED full-anchor re-anchor is a
        // TIMING event — once the tracker is pilot-refined, its warm CFO (<0.1 Hz)
        // beats the fade-jittered chirp gap estimate (sigma 0.3-1.15 Hz), whose
        // sub-clamp phantoms killed 25% of full-anchor groups at 16QAM (0/N, all
        // frames, confident-but-rotated LLRs) vs 0% on the warm-LTS path. Cold /
        // idle / PING / MC-DPSK / narrow keep full chirp trust (other arm + the
        // pilot_seeded fallback inside).
        float new_cfo = use_full_ofdm_anchor_search
            ? cfo_tracker_.seedFromChirpConnectedAnchor(
                  sync_result.cfo_hz, sync_result.correlation, log_prefix_.c_str())
            : cfo_tracker_.seedFromChirp(sync_result.cfo_hz, sync_result.correlation,
                                         connected_, log_prefix_.c_str());
        if (timing_cfo_genie) {
            LOG_MODEM(WARN,
                      "[%s] DIAG genie-timing-cfo: forcing sync CFO %.2f Hz -> 0.00 Hz",
                      log_prefix_.c_str(), new_cfo);
            new_cfo = 0.0f;
        }
        // DIAG (ULTRA_DIAG_FORCE_SYNC_CFO=<Hz>, unset = inert): force every sync CFO to a known
        // value — the simulator's true CFO — so a CFO-ESTIMATION error can be separated from a
        // channel limit (the genie arm that root-caused BUG-LTS-CFO-SEED-GATE). Never on a radio.
        if (const char* g = std::getenv("ULTRA_DIAG_FORCE_SYNC_CFO"); g && *g) {
            LOG_MODEM(INFO, "[%s] DIAG force-sync-cfo: measured %.2f Hz (chirp %.2f) -> %.2f Hz",
                      log_prefix_.c_str(), new_cfo, sync_result.cfo_hz, std::atof(g));
            new_cfo = static_cast<float>(std::atof(g));
        }

        sync_cfo_ = new_cfo;
        sync_snr_ = chirpSyncQualityDb(sync_result.correlation, sync_controller_.ring_.noise_floor_);
        sync_correlation_ = sync_result.correlation;
        sync_gap_error_samples_ = sync_result.gap_error_samples;
        sync_start_time_ = std::chrono::steady_clock::now();
        pending_total_cw_ = 0;
        sync_from_warm_timed_window_ = used_warm_timed_window;
        sync_from_full_anchor_fallback_ = used_full_anchor_fallback;
        sync_from_structure_anchor_ = used_structure_anchor;

        state_ = DecoderState::SYNC_FOUND;
        stampRxSignal();  // F129: sync itself is RX evidence

        last_snr_.store(sync_snr_);
        cfo_tracker_.store(sync_cfo_);

        LOG_MODEM(INFO, "[%s] SYNC at pos=%zu, CFO=%.1f Hz, SNR=%.1f dB (%s)",
                  log_prefix_.c_str(), sync_position_, sync_cfo_, sync_snr_,
                  snrSourceToString(SNRSource::SYNC_QUALITY));

        // NOTE: Do NOT advance sync_controller_.ring_.correlation_pos_ past the frame here.
        // It was already advanced by CORRELATION_STEP at line 323 during search.
        // The post-decode skip at decodeCurrentFrame() line 718 handles advancing
        // past the decoded frame when we return to SEARCHING.
        //
        // Previously, this code jumped sync_controller_.ring_.correlation_pos_ past the entire frame,
        // which could place it AHEAD of sync_controller_.ring_.write_pos_ in circular buffer space
        // (especially after buffer wraps). This caused feedAudio()'s overflow
        // check to compute unsearched ≈ buffer_size, triggering spurious
        // buffer overflows and data loss during async decode.
    } else if (!search_buffer.empty()) {
        if (used_warm_timed_window) {
            std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);
            sync_controller_.ring_.setSearchFloorLocked(warm_narrow_end_abs);
            noteFrameArrivalSyncMissLocked();
            LOG_MODEM(INFO,
                      "[%s] warm-sync: no LTS in %s expected window, misses=%d confidence=%.2f",
                      log_prefix_.c_str(),
                      used_warm_narrow_window ? "narrow" : "degraded",
                      sync_controller_.consecutiveSyncMisses(),
                      sync_controller_.frameArrivalConfidence());
        }
        // IDLE-NOISE ADMISSION GATE (2026-07-25). This branch runs when the sync search found
        // NO frame — which during an ACTIVE exchange does not mean the air is quiet. The buffer
        // can hold this station's own tone-burst ACK tail, the multipath echo of the burst just
        // received, or a burst the search simply failed to lock. Feeding those in as "idle
        // noise" put SIGNAL into the floor: rig xfer_1 ratcheted the reference to 0.1380 —
        // +24.5 dB above what a 20 dB S:N channel can physically produce — and pinned the ALC
        // at a false LOW for the whole transfer, which drives the sender to ramp TX drive into
        // compression on a real radio.
        //
        // Only observe when the decoder has seen NO RX activity recently. stampRxSignal() is
        // stamped on every sync/frame event, so this is decoder EVIDENCE of quiet, not a guess.
        // The window is one burst cycle (~12 s) so it also covers our own ACK and its echo.
        // Site 1 above is already guarded this way in spirit ("BOOT-ONLY ... a prefix can
        // contain an undetected earlier transmission's tail, which would over-read the floor").
        constexpr int64_t kIdleQuietMs = 12000;
        if (!rxSignalActiveWithin(kIdleQuietMs)) {
            const size_t idle_count = std::min(search_buffer.size(), CORRELATION_STEP);
            observeIdleNoiseCandidate(search_buffer.data(), idle_count);
        }
    }
}

void StreamingDecoder::checkIfReadyToDecode() {
    if (!waveform_) {
        clearClassicLongGeometry();
        state_ = DecoderState::SEARCHING;
        return;
    }

    // How many samples do we have from sync position?
    size_t available;
    {
        std::lock_guard<std::mutex> lock(sync_controller_.ring_.buffer_mutex_);

        if (sync_controller_.ring_.write_pos_ >= sync_position_) {
            available = sync_controller_.ring_.write_pos_ - sync_position_;
        } else {
            available = sync_controller_.ring_.buffer_capacity_samples_ - sync_position_ + sync_controller_.ring_.write_pos_;
        }
    }

    // Check elapsed time after selecting the sample requirement. Large variable
    // OFDM frames can be longer than the legacy 5s fixed-frame wait.
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - sync_start_time_).count();

    // Calculate how much we need — must match decodeCurrentFrame() buffer sizing.
    bool is_ofdm_here = protocol::isOFDMMode(mode_);
    const bool control_only_receive =
        is_ofdm_here && connected_ &&
        control_only_receive_.load(std::memory_order_acquire);
    // The lifecycle may enter DISCONNECTING after a connected DATA peek has
    // armed a fixed-CW continuation but before the rest of that DATA geometry
    // arrives.  Waiting for the obsolete length here prevents decodeCurrentFrame
    // from reaching its control-only cleanup and can strand the real hardened
    // DISCONNECT ACK behind that wait.  Retire decoder-owned DATA continuation at
    // this readiness boundary; decodeCurrentFrame repeats the guard to cover a
    // phase change between these two processBuffer iterations.
    if (control_only_receive && pending_total_cw_ > 0) {
        LOG_MODEM(INFO,
                  "[%s] Control-only RX readiness: discarding pending %d-CW DATA escalation",
                  log_prefix_.c_str(), pending_total_cw_);
        pending_total_cw_ = 0;
        clearClassicLongGeometry();
    }
    // 2026-05-29 channel-adaptive interleaver (RX decouple): mirror streaming_ofdm_decode —
    // burst_latched = group-start marker detected (interleave-independent); the
    // ConnectedOFDMBurst full-frame sizing is keyed on the marker + the burst regime.
    bool burst_latched =
        !control_only_receive && waveform_ && waveform_->wasBurstInterleaved();
    const bool burst_regime_active = use_burst_interleave_ || burst_transport_rx_;
    const size_t pending_samples = pending_total_cw_ > 0
        ? static_cast<size_t>(waveform_->getMinSamplesForCWCount(pending_total_cw_))
        : 0;
    const size_t ofdm_control_samples = (is_ofdm_here && connected_)
        ? getOFDMControlFrameSamplesForCurrentMode()
        : 0;
    // BUG-BURST-STALE-GEOMETRY E8b — THIS MIRROR IS NOT OPTIONAL. It is the readiness
    // gate for decodeCurrentFrame's slice. If it releases the frame after only the
    // LATCHED (shorter) sample count while the decode then asks for the COMMANDED
    // length, `frame_len = std::min(frame_len, available)` silently truncates the
    // group-start frame — and burst_next_pos_ / burst_data_start_abs_ are both
    // derived from that length, so the whole group lands early and the half-duplex
    // ack interlock opens into the sender's key-down. Both sites resolve the SAME
    // tuple; decodeCurrentFrame additionally refuses to arm on any residual
    // truncation (the two resolves are separated in time by one processBuffer).
    // SNAPSHOTTED (2026-07-28 review fix): the decode RE-USES this exact result
    // instead of resolving again, so the two cannot disagree even if the standing
    // command or the latched profile moves in between. A shrinking requirement is the
    // dangerous direction — it produces a SHORT slice with no truncation to detect.
    const auto cg_ready = resolveCommandedGeometry(is_ofdm_here, burst_latched);
    cg_snapshot_ = cg_ready;
    cg_snapshot_sync_pos_ = sync_position_ + 1;  // +1 so 0 stays "none"
    const size_t full_frame_samples = cg_ready.armed
        ? cg_ready.frame_samples
        : ((is_ofdm_here && connected_)
               ? static_cast<size_t>(
                     waveform_->getMinSamplesForCWCount(fixed_frame_codewords_))
               : 0);
    const size_t control_frame_samples =
        static_cast<size_t>(waveform_->getMinSamplesForControlFrame());
    auto requirement = decode_policy::selectDecodeSampleRequirement(
        pending_total_cw_,
        is_ofdm_here,
        connected_,
        burst_regime_active,
        burst_latched,
        pending_samples,
        ofdm_control_samples,
        full_frame_samples,
        control_frame_samples);
    // Structure acquisition: a structure lock lands ~0.45 s into a frame (its window is the
    // template span + backtrack + probe), long before a 2.5 s chirp-window lock would. When it
    // is NOT a group head (no marker) while a descriptor-declared group is in flight, it is a
    // member found after a missed head; the HEADNULL late-join needs that member's WHOLE frame
    // in the ring, so wait for it here instead of failing the join and dropping every member
    // (measured Moderate@20: one missed head -> 4 HEADNULL drops). Readiness only: the decode
    // slice and the group geometry are unchanged.
    if (sync_from_structure_anchor_ && !burst_latched &&
        sync_controller_.have_burst_descriptor_ && full_frame_samples > requirement.samples) {
        requirement.samples = full_frame_samples;
    }

    static constexpr int kAudioSampleRateHz = 48000;
    const int required_audio_ms = static_cast<int>(
        (requirement.samples * 1000 + kAudioSampleRateHz - 1) / kAudioSampleRateHz);
    const int frame_timeout_ms = std::max(FRAME_TIMEOUT_MS, required_audio_ms + 2000);
    if (elapsed > frame_timeout_ms) {
        LOG_MODEM(WARN, "[%s] Frame timeout after %lld ms (need=%zu samples, timeout=%d ms)",
                  log_prefix_.c_str(), (long long)elapsed, requirement.samples, frame_timeout_ms);
        pending_total_cw_ = 0;
        clearClassicLongGeometry();
        state_ = DecoderState::SEARCHING;
        return;
    }

    if (available >= requirement.samples) {
        state_ = DecoderState::DECODING;
    }
}

// Observability counters for robust decode paths (check via debugger or periodic log)

} // namespace gui
} // namespace ultra
