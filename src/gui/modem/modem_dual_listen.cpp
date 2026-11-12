// modem_dual_listen.cpp - ModemEngine's receive decoders: callback wiring for any decoder slot and
// dual-listen (a second decoder on the other waveform while a wideband session is connected).
// Design: docs/DUAL_LISTEN_DESIGN_2026_11_11.md.

#include "modem_engine.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "diagnostics/diagnostics_recorder.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"

#include <cmath>
#include <cstdio>

namespace ultra {
namespace gui {

void copyDecodeSnrMetrics(LoopbackStats& stats, const DecodeResult& result) {
    stats.snr_db = result.snr_db;
    stats.snr_source = result.snr_source;
    stats.mcdpsk_snr_routed_data_aided = result.mcdpsk_snr_routed_data_aided;
    stats.has_idle_in_band_snr_db = result.has_idle_in_band_snr_db;
    stats.idle_in_band_snr_db = result.idle_in_band_snr_db;
    stats.has_ofdm_broadband_snr_db = result.has_ofdm_broadband_snr_db;
    stats.ofdm_broadband_snr_db = result.ofdm_broadband_snr_db;
    stats.ofdm_internal_snr_db = result.ofdm_internal_snr_db;
    stats.sync_quality_db = result.sync_quality_db;
    stats.has_frame_physical_snr_db = result.has_physical_snr_db;
    stats.frame_physical_snr_db = result.physical_snr_db;
}

// The callbacks every decoder slot gets (they were the constructor's). A decoder that is not the
// primary - dual-listen's listener on the other waveform - delivers only what decoded: its failures
// are locks on the other waveform's identical chirp, not this session's evidence. Before a
// listener's frame or group is delivered, the engine reports its waveform (noteListenerDelivery), so
// the protocol switches to it first.
void ModemEngine::wireDecoderCoreCallbacks(StreamingDecoder* decoder) {
    decoder->setFrameCallback([this, decoder](const DecodeResult& result) {
        const bool delivered = result.success && !result.frame_data.empty();
        if (!streaming_decoder_.isPrimary(decoder)) {
            if (!delivered) {
                return;
            }
            // DATA on the other waveform is the peer's traffic moving there; a control frame is
            // delivered as it came (a late repeat must not move the session).
            if (v2::isDataFrame(result.frame_type)) {
                noteListenerDelivery(decoder);
            }
        }
        // Update SNR/sync stats before delivering frame so downstream callbacks
        // (ProtocolEngine via raw_data_callback_) read fresh channel estimates.
        updateStats([&](LoopbackStats& s) {
            copyDecodeSnrMetrics(s, result);
            s.synced = result.success;
        });

        if (delivered) {
            if (decode_observer_) decode_observer_(result);
            if (deliverFrame(result)) {
                notifyFrameParsed(result.frame_data, result.frame_type);
            }
        } else if (!result.success && result.codewords_failed > 0) {
            char fields[192];
            std::snprintf(fields, sizeof(fields),
                          "{\"cw_ok\":%d,\"cw_failed\":%d,\"snr_db\":%.1f,"
                          "\"snr_source\":\"%s\"}",
                          result.codewords_ok, result.codewords_failed, result.snr_db,
                          snrSourceToString(result.snr_source));
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "phy", "decode.fail", fields);
        }
        // Save peer CFO for future frames
        if (std::abs(result.cfo_hz) > 0.1f) {
            peer_cfo_hz_ = result.cfo_hz;
        }
        last_rx_complete_time_ = std::chrono::steady_clock::now();
    });

    // §14.27: forward a decoded interleaved burst (delivered as a unit) to the
    // protocol layer's burst transport. Only fires when burst-transport RX is
    // enabled on the decoder.
    decoder->setBurstGroupCallback(
        [this, decoder](uint16_t group_seq, const std::vector<Bytes>& frames, bool all_ok,
                        float quality, uint16_t frame_mask, bool interleaved,
                        uint8_t group_size, bool geometry_proven) {
            if (!streaming_decoder_.isPrimary(decoder)) {
                if (!all_ok && frame_mask == 0) {
                    return;  // an empty group on the listener: the other waveform's chirp
                }
                noteListenerDelivery(decoder);
            }
            updateStats([&](LoopbackStats& s) { s.synced = all_ok; });
            if (burst_group_callback_) {
                burst_group_callback_(group_seq, frames, all_ok, quality, frame_mask,
                                      interleaved, group_size, geometry_proven);
            }
            last_rx_complete_time_ = std::chrono::steady_clock::now();
        });

    decoder->setDataSyncAcceptedCallback([this, decoder](float sync_correlation) {
        if (data_sync_accepted_callback_ && streaming_decoder_.isPrimary(decoder)) {
            data_sync_accepted_callback_(sync_correlation);
        }
    });

    decoder->setPingCallback([this, decoder](float snr_db, float cfo_hz) {
        if (!streaming_decoder_.isPrimary(decoder)) {
            return;  // a listener runs only inside a session
        }
        // If narrowband chirp detected, switch control waveform for PONG/CONNECT_ACK
        if (decoder->getDetectedBandwidth() == BandwidthMode::NARROW) {
            if (streaming_encoder_) {
                streaming_encoder_->setNarrowbandControl(true);
            }
            LOG_MODEM(INFO, "Narrowband chirp detected, switching control waveform");
        }
        if (ping_observer_) ping_observer_(snr_db, cfo_hz);
        if (ping_received_callback_) {
            ping_received_callback_(snr_db);
        }
        updateStats([&](LoopbackStats& s) {
            s.frames_received++;
            s.snr_db = snr_db;
            s.snr_source = SNRSource::SYNC_QUALITY;
            s.sync_quality_db = snr_db;
            s.synced = true;
        });
        last_rx_complete_time_ = std::chrono::steady_clock::now();
    });
}

// The protocol's decoder callbacks, on every slot: the failure reports only from the primary.
void ModemEngine::wireDecoderRoleCallbacks() {
    forEachRxDecoder([this](StreamingDecoder* d) {
        d->setAnchoredBurstNoGroupCallback([this, d](bool payload_seen) {
            if (anchored_no_group_cb_ && streaming_decoder_.isPrimary(d)) {
                anchored_no_group_cb_(payload_seen);
            }
        });
        d->setMcDpskMarginCallback([this, d](bool upper_rung_margin) {
            if (mcdpsk_margin_cb_ && streaming_decoder_.isPrimary(d)) {
                mcdpsk_margin_cb_(upper_rung_margin);
            }
        });
        d->setMcDpskFrameLostCallback([this, d](int modulation) {
            if (mcdpsk_frame_lost_cb_ && streaming_decoder_.isPrimary(d)) {
                mcdpsk_frame_lost_cb_(modulation);
            }
        });
        d->setBurstOutcomeUnknownCallback([this, d]() {
            if (burst_outcome_unknown_cb_ && streaming_decoder_.isPrimary(d)) {
                burst_outcome_unknown_cb_();
            }
        });
        d->setDescriptorModeChangeCallback(
            [this, d](Modulation mod, CodeRate rate, int cw_per_frame) {
                if (descriptor_mode_change_cb_ && streaming_decoder_.isPrimary(d)) {
                    descriptor_mode_change_cb_(mod, rate, cw_per_frame);
                }
            });
    });
}

void ModemEngine::applyStoredDecoderSettings(StreamingDecoder* decoder) {
    decoder->setRealTimeAudio(true);
    decoder->setLogPrefix(decoder == streaming_decoder_.slot(1) ? log_prefix_ + "/listen"
                                                                 : log_prefix_);
    if (soft_combine_buffer_) decoder->setSoftCombineBuffer(soft_combine_buffer_);
    if (harq_context_cb_) decoder->setHarqProvisionalContextCallback(harq_context_cb_);
    if (tone_ack_cb_) decoder->setToneBurstAckCallback(tone_ack_cb_);
    if (tone_ack_predicate_) decoder->setToneBurstAckAcceptancePredicate(tone_ack_predicate_);
    decoder->setMcDpskSessionAddresses(mcdpsk_local_hash_, mcdpsk_peer_hash_);
    decoder->setCodecType(codec_type_);
    decoder->setBurstTransportRxEnabled(burst_transport_rx_enabled_);
    if (carrier_ldpc_interleaver_set_) decoder->setCarrierLdpcInterleaver(carrier_ldpc_interleaver_);
}

// The listener's waveform for the session: the wideband one the primary is not running.
void ModemEngine::configureListener(StreamingDecoder* listener, protocol::WaveformMode waveform) {
    listener->reset();
    listener->setHandshakeConfirmationPending(false);
    if (waveform == protocol::WaveformMode::OFDM_CHIRP) {
        // The first OFDM burst after a switch carries the full dual chirp and its descriptor names
        // the rung; the floor data mode only seeds the decoder until then.
        ModemConfig ofdm = config_;
        ofdm.modulation = Modulation::QPSK;
        ofdm.code_rate = CodeRate::R1_4;
        ofdm.use_pilots = true;
        ofdm.pilot_spacing =
            ofdm_link_adaptation::recommendedPilotSpacing(Modulation::QPSK, CodeRate::R1_4);
        listener->setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP, ofdm, Modulation::QPSK,
                                       CodeRate::R1_4);
        listener->expectFullOFDMAnchorOnce();
    } else {
        // The MC-DPSK geometry a down-crossing lands on (ROBUST_MID: 8 carriers, 1024 samples per
        // symbol, DBPSK, 3 codewords); the first-frame retry reads DQPSK too.
        const auto rung = protocol::connection_policy::ladderRungForId(
            protocol::LadderRungId::ROBUST_MID);
        MultiCarrierDPSKConfig mc = mc_dpsk_config_;
        mc.num_carriers = 8;
        mc.samples_per_symbol = 1024;
        mc.bits_per_symbol = 1;
        listener->setMode(protocol::WaveformMode::MC_DPSK, /*connected=*/true);
        listener->setMCDPSKConfig(mc);
        listener->setDataMode(rung.modulation, rung.code_rate);
        listener->setFixedFrameCodewords(rung.cw_count);
    }
    if (std::abs(peer_cfo_hz_) > 0.01f) {
        listener->setKnownCFO(peer_cfo_hz_);
    }
    listener_waveform_.store(waveform, std::memory_order_release);
    LOG_MODEM(INFO, "[%s] Listener: listening for %s alongside %s", log_prefix_.c_str(),
              protocol::waveformModeToString(waveform),
              protocol::waveformModeToString(waveform_mode_));
}

// Dual-listen runs while a wideband session is connected: the listener takes the waveform the
// primary is not running.
void ModemEngine::updateDualListen() {
    const bool wideband = waveform_mode_ == protocol::WaveformMode::MC_DPSK ||
                          waveform_mode_ == protocol::WaveformMode::OFDM_CHIRP;
    const bool want = dual_listen_enabled_ && connected_ && handshake_complete_ && wideband;
    if (!want) {
        if (streaming_decoder_.listenerActive()) {
            // The other slot is no longer fed; its thread idles on its own condition variable.
            // Whichever slot is primary stays primary (the caller reconfigures it).
            streaming_decoder_.setListenerActive(false);
            LOG_MODEM(INFO, "[%s] Listener: listener stopped", log_prefix_.c_str());
        }
        return;
    }
    if (streaming_decoder_.listenerActive()) {
        return;
    }
    const int listener_slot = 1 - streaming_decoder_.primaryIndex();
    StreamingDecoder* listener = streaming_decoder_.slot(listener_slot);
    if (listener == nullptr) {
        streaming_decoder_.setSlot(listener_slot, std::make_unique<StreamingDecoder>());
        listener = streaming_decoder_.slot(listener_slot);
        wireDecoderCoreCallbacks(listener);
        wireDecoderRoleCallbacks();
    }
    applyStoredDecoderSettings(listener);
    configureListener(listener, waveform_mode_ == protocol::WaveformMode::MC_DPSK
                                    ? protocol::WaveformMode::OFDM_CHIRP
                                    : protocol::WaveformMode::MC_DPSK);
    streaming_decoder_.setListenerActive(true);
    startRxListenerThread();
}

// Slot 1's decode thread: started with the second decoder and kept for the engine's life, whichever
// role the object has (slot 0's is rx_decode_thread_).
void ModemEngine::startRxListenerThread() {
    StreamingDecoder* decoder = streaming_decoder_.slot(1);
    if (decoder == nullptr || synchronous_mode_ || rx_listener_running_) {
        return;
    }
    rx_listener_running_ = true;
    rx_listener_thread_ =
        std::thread([this, decoder] { rxDecodeLoopFor(decoder, rx_listener_running_); });
}

// A listener decoded something: the peer is on the listener's waveform. Report it before the frame
// is delivered; the protocol crosses to it (Connection::noteRxWaveform), which flips the roles here
// (setWaveformMode), so the frame reaches a protocol and a primary on its own waveform.
void ModemEngine::noteListenerDelivery(StreamingDecoder* decoder) {
    (void)decoder;
    const protocol::WaveformMode waveform = listener_waveform_.load(std::memory_order_acquire);
    LOG_MODEM(INFO, "[%s] Listener: %s arrived while the session ran %s", log_prefix_.c_str(),
              protocol::waveformModeToString(waveform),
              protocol::waveformModeToString(waveform_mode_));
    if (rx_waveform_observed_cb_) {
        rx_waveform_observed_cb_(waveform);
    }
}

}  // namespace gui
}  // namespace ultra
