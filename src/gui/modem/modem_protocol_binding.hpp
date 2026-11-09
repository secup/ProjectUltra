#pragma once

#include "gui/modem/modem_engine.hpp"
#include "protocol/protocol_engine.hpp"

#include <functional>
#include <utility>
#include <vector>

namespace ultra {
namespace gui {

// wireModemToProtocol — the SINGLE place the modem (ModemEngine) is bound to the protocol
// (ProtocolEngine). BOTH frontends (the GUI app and headless ultra_tnc) call this, so the
// modem->protocol forwarding lives in exactly ONE spot: RX-data, burst-group delivery,
// data-sync acceptance, tone-burst GROUP_ACK, and HARQ provisional context.
//
// Why this exists: previously this glue lived inline in app.cpp ONLY. ultra_tnc owns a
// ModemEngine too but re-wired a *subset* of the callbacks by hand, silently MISSING
// setBurstGroupCallback + setToneBurstAckCallback -> it decoded burst frames it never
// assembled/delivered/ACKed, so file transfer timed out. Centralizing the binding makes
// that whole divergence class structurally impossible: add a forwarding here once, both
// frontends get it.
//
// Frontend-specific REACTIONS are deliberately NOT in this binding — ping/status callbacks
// stay frontend-owned (GUI: UI log lines; TNC: operator log), and the optional
// `after_rx_data` hook lets a frontend observe each decoded frame AFTER the core protocol
// forwarding (GUI: monitor-mode log + adaptive advisory) without re-implementing the core.
struct ModemProtocolFrontendHooks {
    // Fires after the core RX-data -> protocol forwarding (SNR measurement + onRxData).
    // GUI uses it for monitor-mode logging + adaptive advisory; TNC leaves it empty.
    std::function<void(const Bytes& data, float snr_db, float fading,
                       SNRSource snr_source, bool used_for_quality)>
        after_rx_data;
};

inline void wireModemToProtocol(ModemEngine& modem,
                                protocol::ProtocolEngine& protocol,
                                ModemProtocolFrontendHooks hooks = {}) {
    // HARQ provisional context: the decoder pulls it from the protocol's soft-combine
    // buffer when assembling provisional codewords.
    modem.setHarqProvisionalContextCallback(
        [&protocol]() { return protocol.harqProvisionalContext(); });

    // Core RX-data path: every decoded frame -> SNR/channel-quality measurement + onRxData.
    // The frontend hook observes afterwards (no duplication of the core forwarding).
    // (hooks captured by copy here AND in the burst-group binding below — both delivery
    // paths must feed the frontend's SNR observation.)
    modem.setRawDataCallback(
        [&modem, &protocol, hooks](const Bytes& data, bool physical_turn_complete) {
            const auto stats = modem.getStats();
            const float snr_db = stats.snr_db;
            const SNRSource snr_source = stats.snr_source;
            const float fading = modem.getFadingIndex();
            const bool use_quality = protocol.shouldUseRxFrameForChannelQuality(data);
            const bool snr_data_aided = stats.mcdpsk_snr_routed_data_aided;
            protocol.setMeasuredSNR(snr_db, snr_source, snr_data_aided);
            {
                // §6: fade-averaged physical channel stats (decoder ring) — the
                // physics ceiling for the entry pick.
                float pm = 0.0f, ps = 0.0f;
                const size_t pn = modem.physicalSnrStats(pm, ps);
                protocol.setPhysicalChannelStats(pm, ps, static_cast<uint32_t>(pn));
            }
            if (use_quality) {
                protocol.setChannelQuality(snr_db, fading, snr_source, snr_data_aided);
                // Doppler coherence (Good/Moderate discriminator) — refines the channel
                // classification once enough OFDM data has pooled (see design doc). The
                // approximate dopplerHz readout rides the same feed (retx trough-pacing
                // deferral only — docs/RETX_PACING_DESIGN_2026_07_03.md).
                protocol.setChannelCoherence(modem.getDopplerCoherenceScore(),
                                             modem.getDopplerCoherenceDopplerHz(),
                                             modem.getDopplerCoherenceValid());
            }
            protocol.onRxData(data, physical_turn_complete);
            if (hooks.after_rx_data) {
                hooks.after_rx_data(data, snr_db, fading, snr_source, use_quality);
            }
        });

    // §14.27: a decoded interleaved burst delivered as a unit -> protocol burst transport
    // (deliver-once + single GROUP_ACK). THIS is the forwarding ultra_tnc was missing.
    modem.setBurstGroupCallback(
        [&modem, &protocol, hooks](uint16_t group_seq, const std::vector<Bytes>& frames,
                                   bool all_ok, float quality, uint16_t frame_mask,
                                   bool interleaved, uint8_t group_size,
                                   bool geometry_proven) {
            // BUG-ACK-STAIRCASE-FADE-BIN layer 2 (2026-07-01): burst-as-unit delivery
            // bypasses setRawDataCallback, so the frontend's SNR observation STARVED
            // during file transfers — the GUI's §15.5 ACK-duration cache held the stale
            // handshake reading (mcdpsk_in_band, an untrusted staircase source) and the
            // fast ACK never engaged even at 19-22 dB broadband. Feed the hook from the
            // modem's per-frame decode stats — the group's logical frames updated them
            // (frame_callback_ fires per frame, streaming_burst_interleave.cpp ~704)
            // BEFORE this group callback. Must run BEFORE onBurstGroupReceived: that
            // call emits THIS group's tone-burst ACK, whose §15.5 duration reads the
            // cache. Data payload is not re-forwarded (the protocol gets the frames
            // below); used_for_quality=false — protocol channel-quality/coherence stay
            // on their existing paths, this is frontend observation only.
            if (hooks.after_rx_data) {
                // stats.snr_db/snr_source are stats-queue-drained and stay stale on
                // this path (measured: frozen at the handshake mcdpsk reading while
                // fading updated live) — the decoder's lock-free last-broadband
                // atomics are the honest per-logical-frame feed.
                if (modem.hasLastOFDMBroadbandSNR()) {
                    hooks.after_rx_data(Bytes{}, modem.getLastOFDMBroadbandSNR(),
                                        modem.getFadingIndex(),
                                        SNRSource::OFDM_BROADBAND,
                                        /*used_for_quality=*/false);
                } else {
                    const auto stats = modem.getStats();
                    hooks.after_rx_data(Bytes{}, stats.snr_db, modem.getFadingIndex(),
                                        stats.snr_source, /*used_for_quality=*/false);
                }
            }
            // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET): feed the decoder's per-burst
            // RX level verdict BEFORE onBurstGroupReceived — that call emits THIS
            // group's tone-burst ACK, which carries the drive advisory derived from
            // the verdict. The seq lets the Connection ignore stale re-feeds (e.g. a
            // timed-out group re-delivering the previous measurement).
            protocol.setRxLevelVerdict(modem.getRxLevelVerdict(),
                                       modem.getRxLevelVerdictSeq());
            // RX-AUTHORITY (2026-07-05): feed the receiver's FRESH per-group channel
            // measurements BEFORE onBurstGroupReceived — that call computes the rung
            // command this group's ACK will carry. The Connection's own copies
            // (measured_snr_db_/fading_index_/coherence) are fed only by the CLASSIC
            // frame path and are handshake-stale during burst transfers; the honest
            // live sources are the decoder's lock-free per-frame atomics.
            if (modem.hasLastOFDMBroadbandSNR()) {
                // kOfdmLegacyAnchorScaleOffsetDb: the rung anchors were measured
                // on the pre-2026-07-07 estimator scale — compensate until the
                // anchor table is re-measured (see connection_policy.hpp).
                protocol.setBurstChannelObservation(
                    modem.getLastOFDMBroadbandSNR() +
                        protocol::connection_policy::ofdmAnchorScaleOffsetDb(),
                    modem.getFadingIndex(),
                    modem.getDopplerCoherenceScore(), modem.getDopplerCoherenceValid(),
                    modem.getDopplerCoherenceDopplerHz());
            }
            // RX-AUTHORITY EVM DEMOTE (ULTRA_EVM_DEMOTE, Stage 2): the group's
            // radio-agnostic decision-directed EVM usable-SNR — reads usable dB
            // directly (constant-free, non-inflating), so NO kOfdmLegacyAnchorScaleOffsetDb
            // markup. Feed it BEFORE onBurstGroupReceived so the demote clamp rides
            // THIS group's ACK. Every callback writes validity explicitly: an
            // incomplete/no-process group clears the member instead of reusing the
            // prior group's scalar when the legacy EVM-demote path is enabled.
            // (The default latent selector returns before EVM is consulted.)
            if (modem.hasLastEvmSnr()) {
                protocol.setBurstEvmObservation(modem.getLastEvmSnrDb());
            } else {
                protocol.clearBurstEvmObservation();
            }
            // RX-AUTHORITY PREDICTIVE: the group's per-carrier SNR snapshot —
            // delivered AND cratered groups alike (constellation-independent;
            // the survivor-bias kill). Must land BEFORE onBurstGroupReceived:
            // the verdict this group's ACK carries consumes it.
            protocol.setBurstCarrierGammas(modem.getLastGroupCarrierGammas());
            // ULTRA_RATE_EVM: the group's per-frame effective SNR for the rate selector.
            protocol.setBurstFrameEvms(modem.getLastGroupEvmSnrFrames());
            protocol.onBurstGroupReceived(group_seq, frames, all_ok, quality, frame_mask,
                                          interleaved, group_size, geometry_proven);
        });

    // DESC-SWITCH Phase 1 (ULTRA_DESCRIPTOR_MODE_SWITCH): a mode-hop BURST_HEADER
    // descriptor -> protocol follow-through (RX-side applyDataMode: window/timers/
    // chunk capacity track the sender's announced mode; NO MODE_CHANGE ACK fires).
    // The decoder only emits this when the knob is ON, and Connection re-gates it —
    // byte-identical while OFF. Same decoder-thread -> engine-mutex class as the
    // burst-group forwarding above.
    modem.setAnchoredBurstNoGroupCallback(
        [&protocol](bool payload_seen) {
            protocol.onAnchoredBurstNoGroup(payload_seen);
        });
    modem.setBurstOutcomeUnknownCallback(
        [&protocol]() { protocol.onBurstOutcomeUnknown(); });
    modem.setDescriptorModeChangeCallback(
        [&protocol](Modulation mod, CodeRate rate, int cw_per_frame) {
            protocol.onDescriptorModeChange(mod, rate, cw_per_frame);
        });

    // Accepted OFDM data-sync -> protocol (warm-sync / burst-cadence bookkeeping).
    modem.setDataSyncAcceptedCallback(
        [&protocol](float sync_correlation) {
            protocol.onAcceptedOFDMDataSync(sync_correlation);
        });

    // §15: tone-burst ACK detections from the always-on monitor -> protocol. Connection
    // matches against the in-flight burst group and either advances (ACK) or triggers a
    // fast resend (NACK). THIS is the other half ultra_tnc was missing (the GROUP_ACK
    // reverse path), so its sender never learned a group landed and retransmitted to the
    // retry cap.
    // CRC-12 validates the wire bits, not whether those bits can describe the
    // sender's current ARQ window. Gate each CRC-valid timing candidate before the
    // monitor commits/disarms so a strong impossible collision cannot hide a weaker
    // valid ACK later in the same audio window.
    modem.setToneBurstAckAcceptancePredicate(
        [&protocol](
            const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload) {
            return protocol.isToneBurstAckCandidatePlausible(payload);
        });
    modem.setToneBurstAckCallback(
        [&protocol](const ultra::waveform::tone_burst_ack::ToneBurstAckDetection& d) {
            protocol.onToneBurstAck(d);
        });
}

}  // namespace gui
}  // namespace ultra
