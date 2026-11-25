// modem_mode.cpp - Waveform and mode control for ModemEngine

#include "modem_engine.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/connection_policy.hpp"
#include <cstdio>

namespace ultra {
namespace gui {

// Helper to get mode description string
static const char* getModeDescription(Modulation mod, CodeRate rate) {
    static char buf[32];
    const char* mod_name;
    switch (mod) {
        case Modulation::DBPSK: mod_name = "DBPSK"; break;
        case Modulation::BPSK:  mod_name = "BPSK"; break;
        case Modulation::DQPSK: mod_name = "DQPSK"; break;
        case Modulation::QPSK:  mod_name = "QPSK"; break;
        case Modulation::D8PSK: mod_name = "D8PSK"; break;
        case Modulation::QAM8:  mod_name = "8QAM"; break;
        case Modulation::QAM16: mod_name = "16QAM"; break;
        case Modulation::QAM32: mod_name = "32QAM"; break;
        case Modulation::QAM64: mod_name = "64QAM"; break;
        default: mod_name = "???"; break;
    }
    const char* rate_name;
    switch (rate) {
        case CodeRate::R1_4: rate_name = "R1/4"; break;
        case CodeRate::R1_2: rate_name = "R1/2"; break;
        case CodeRate::R2_3: rate_name = "R2/3"; break;
        case CodeRate::R3_4: rate_name = "R3/4"; break;
        case CodeRate::R5_6: rate_name = "R5/6"; break;
        default: rate_name = "R?"; break;
    }
    snprintf(buf, sizeof(buf), "%s %s", mod_name, rate_name);
    return buf;
}

void ModemEngine::setWaveformMode(protocol::WaveformMode mode) {
    if (waveform_mode_ == mode) return;

    LOG_MODEM(INFO, "Switching waveform mode: %d -> %d",
              static_cast<int>(waveform_mode_.load()), static_cast<int>(mode));

    // Dual-listen: the listener already runs `mode` with live state - it becomes the primary and
    // the primary the listener, no decoder is rebuilt (docs/DUAL_LISTEN_DESIGN_2026_11_11.md).
    const bool flip = connected_ && streaming_decoder_.listenerActive() &&
                      listener_waveform_.load(std::memory_order_acquire) == mode;
    if (flip) {
        listener_waveform_.store(waveform_mode_, std::memory_order_release);
        streaming_decoder_.flip();
        // The new primary hears the session waveform: it learns the idle noise again (as a
        // listener it learned nothing - BUG-PHYSICAL-REF-IDLE-FLOOR-CONTAMINATED); the new listener
        // stops below (configureListener) or here.
        streaming_decoder_.primary()->setIdleNoiseLearning(true);
        streaming_decoder_.primary()->setMfskLookaheadAllowed(true);  // a primary may see 16-FSK
        if (StreamingDecoder* now_listening =
                streaming_decoder_.slot(1 - streaming_decoder_.primaryIndex())) {
            now_listening->setIdleNoiseLearning(false);
            now_listening->setMfskLookaheadAllowed(false);
        }
        LOG_MODEM(INFO, "[%s] Listener: %s decoder is now primary, %s listens",
                  log_prefix_.c_str(), protocol::waveformModeToString(mode),
                  protocol::waveformModeToString(waveform_mode_));
    }
    waveform_mode_ = mode;
    if (flip && listener_waveform_.load(std::memory_order_acquire) ==
                    protocol::WaveformMode::MC_DPSK) {
        // The decoder that now listens for MC-DPSK kept the SESSION's MC-DPSK profile - the connect
        // pick, e.g. Robust (DQPSK, 4 codewords) - but a sender leaving OFDM always lands on the
        // down-crossing target, ROBUST_MID (DBPSK, 3 codewords, 8 carriers, 1024 samples/symbol),
        // whatever profile the session started on. Sliced at the old profile, no fallback burst ever
        // decoded and the receiver never followed the sender down (BUG-LISTENER-KEEPS-SESSION-PROFILE,
        // GUI ITU Good 5 dB seed 48: 330 s of fallback bursts found at 20 dB sync and lost, then a
        // terminal ARQ failure). Configure it exactly as a new listener (updateDualListen).
        StreamingDecoder* listener = streaming_decoder_.slot(1 - streaming_decoder_.primaryIndex());
        if (listener != nullptr) {
            applyStoredDecoderSettings(listener);
            configureListener(listener, protocol::WaveformMode::MC_DPSK);
        }
    }
    applyOfdmGeometryFor(mode);
    // Tone-burst ACKs follow the session's channel: a narrowband session keeps them inside its
    // 500 Hz (tone_burst_constants.hpp, band plan). Same decision on both stations.
    {
        const auto band = mode == protocol::WaveformMode::OFDM_NARROW
                              ? ultra::waveform::tone_burst_ack::ToneAckBand::Narrowband
                              : ultra::waveform::tone_burst_ack::ToneAckBand::Wideband;
        if (streaming_encoder_) streaming_encoder_->setToneAckBand(band);
        forEachRxDecoder([&](StreamingDecoder* d) { d->setToneAckBand(band); });
    }

    // Update StreamingDecoder to use the new waveform
    // When disconnected, always use MC_DPSK for PING detection (chirp-based sync)
    if (streaming_decoder_ && !flip) {
        protocol::WaveformMode decoder_mode = connected_ ? mode : protocol::WaveformMode::MC_DPSK;
        if (connected_ && protocol::isOFDMMode(mode)) {
            streaming_decoder_->setConnectedOFDMMode(mode, config_, data_modulation_, data_code_rate_);
            streaming_decoder_->expectFullOFDMAnchorOnce();  // any OFDM waveform anchors on a chirp
            LOG_MODEM(INFO, "setWaveformMode: StreamingDecoder connected OFDM config set (FFT=%d, carriers=%d)",
                      config_.fft_size, config_.num_carriers);
        } else {
            streaming_decoder_->setMode(decoder_mode, connected_);

            // For OFDM modes, propagate the current config (for custom FFT/carriers like NVIS mode)
            if (protocol::isOFDMMode(mode)) {
                streaming_decoder_->setOFDMConfig(config_);
                LOG_MODEM(INFO, "setWaveformMode: StreamingDecoder OFDM config set (FFT=%d, carriers=%d)",
                          config_.fft_size, config_.num_carriers);
            }
        }
    }

    // Update StreamingEncoder to match
    if (streaming_encoder_) {
        streaming_encoder_->setMode(mode);
        if (protocol::isOFDMMode(mode)) {
            streaming_encoder_->setOFDMConfig(config_);
            if (connected_) {
                streaming_encoder_->forceNextFrameFullPreamble();
            }
        }
    }

    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:
            LOG_MODEM(INFO, "DPSK mode active: %d-PSK, %d samples/sym, %.1f bps",
                      dpsk_config_.num_phases(),
                      dpsk_config_.samples_per_symbol,
                      dpsk_config_.raw_bps());
            break;

        case protocol::WaveformMode::OFDM_CHIRP:
            LOG_MODEM(INFO, "OFDM_CHIRP mode active: %d carriers, DQPSK (differential)",
                      config_.num_carriers);
            break;

        case protocol::WaveformMode::OFDM_NARROW:
            LOG_MODEM(INFO, "OFDM_NARROW mode active: %d carriers, 500 Hz narrowband",
                      config_.num_carriers);
            break;

        default:
            LOG_MODEM(INFO, "OFDM mode active: %d carriers, %s",
                      config_.num_carriers,
                      connected_ ? "connected" : "disconnected");
            break;
    }
    updateDualListen();
}

// OFDM_NARROW is its own geometry (FFT 2048, 21 carriers, 1250-1750 Hz chirp), not a label on
// the wideband one. Until 2026-11-10 only ultra_tnc switched to it; the GUI (and so `ultra ptx`)
// ran OFDM_NARROW on the wideband 1024/59 geometry, which a TNC station could not decode
// (BUG-GUI-NARROW-WIDEBAND-GEOMETRY). The data mode (modulation, rate, pilots) carries over.
void ModemEngine::applyOfdmGeometryFor(protocol::WaveformMode mode) {
    const bool narrow = mode == protocol::WaveformMode::OFDM_NARROW;
    if (narrow == narrow_geometry_) return;
    ModemConfig next;
    if (narrow) {
        wideband_config_ = config_;
        next = presets::narrowbandOFDM();
    } else {
        next = wideband_config_;
    }
    next.modulation = config_.modulation;
    next.code_rate = config_.code_rate;
    next.use_pilots = true;
    next.pilot_spacing =
        ofdm_link_adaptation::recommendedPilotSpacing(next.modulation, next.code_rate);
    config_ = next;
    narrow_geometry_ = narrow;
    LOG_MODEM(INFO, "[%s] OFDM geometry for %s: FFT=%d, carriers=%d, chirp %.0f-%.0f Hz",
              log_prefix_.c_str(), protocol::waveformModeToString(mode), config_.fft_size,
              config_.num_carriers, config_.chirp_f_start, config_.chirp_f_end);
}

void ModemEngine::joinConnectedSession(protocol::WaveformMode mode, Modulation mod,
                                       CodeRate rate) {
    if (mode == protocol::WaveformMode::MC_DPSK) {
        mod = mcDpskModulationForConfig(mc_dpsk_config_);
    }
    // The order the connection's callbacks run in: data mode, waveform, connected, confirmed.
    setDataMode(mod, rate);
    setWaveformMode(mode);
    setConnected(true);
    setHandshakeComplete(true);
    if (protocol::isOFDMMode(mode)) {
        setFixedFrameCodewords(protocol::connection_policy::recommendCWCount(mod, rate, mode));
    }
}

void ModemEngine::crossConnectedWaveform(protocol::WaveformMode mode, Modulation mod,
                                         CodeRate rate, int cw_count, int mc_dpsk_num_carriers,
                                         int mc_dpsk_samples_per_symbol) {
    LOG_MODEM(INFO, "[%s] Waveform crossing: %s -> %s %s %s cw=%d", log_prefix_.c_str(),
              protocol::waveformModeToString(waveform_mode_), protocol::waveformModeToString(mode),
              modulationToString(mod), codeRateToString(rate), cw_count);
    if (mode == protocol::WaveformMode::MC_DPSK && mc_dpsk_num_carriers > 0 &&
        mc_dpsk_samples_per_symbol > 0) {
        setMCDPSKProfile(mc_dpsk_num_carriers, mc_dpsk_samples_per_symbol,
                         (mod == Modulation::DBPSK || isMfskModulation(mod)) ? 1 : mod == Modulation::D8PSK ? 3 : 2);
    }
    // The data mode the new waveform is built with (setWaveformMode reads these); the encoder and
    // decoder are reconfigured once the waveform is the new one.
    data_modulation_ = mod;
    data_code_rate_ = rate;
    config_.modulation = mod;
    config_.code_rate = rate;
    config_.use_pilots = true;
    config_.pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    setWaveformMode(mode);
    if (mode == protocol::WaveformMode::MC_DPSK) {
        // A session crosses into MC-DPSK only when its channel collapsed: the physical-SNR
        // distribution restarts, as at a new QSO (the connection drops its copy with the crossing).
        // After the switch, so it is the MC-DPSK decoder's (dual-listen flips the decoders).
        streaming_decoder_->resetPhysicalSnrEvidence();
    }
    setDataMode(mod, rate);
    if (cw_count > 0) {
        setFixedFrameCodewords(cw_count);
    }
}

void ModemEngine::setConnectWaveform(protocol::WaveformMode mode) {
    LOG_MODEM(INFO, "Switching connect waveform: %s -> %s",
              protocol::waveformModeToString(connect_waveform_),
              protocol::waveformModeToString(mode));

    connect_waveform_ = mode;

    // Clear any leftover flag from previous disconnect - we're starting fresh
    use_connected_waveform_once_ = false;

    // Configure DPSK for medium preset (DQPSK 62b R1/4) for connection attempts
    if (mode == protocol::WaveformMode::MC_DPSK) {
        dpsk_config_ = dpsk_presets::medium();  // DQPSK 62.5 baud
        LOG_MODEM(INFO, "DPSK connect mode: %d-PSK, %.1f baud",
                  dpsk_config_.num_phases(), dpsk_config_.symbol_rate());
    }
}

void ModemEngine::setConnected(bool connected) {
    LOG_MODEM(INFO, "[%s] setConnected(%d) called, was connected_=%d",
              log_prefix_.c_str(), connected ? 1 : 0, connected_ ? 1 : 0);

    if (connected_ == connected) return;

    connected_ = connected;
    updateDualListen();  // off before a disconnect reconfigures the primary

    if (connected) {
        // Reset handshake state - we'll complete it when we receive first post-ACK frame
        handshake_complete_ = false;
        use_connected_waveform_once_ = false;  // Clear any leftover flag

        // OFDM is coherent-only (thread A 2026-05-31): OFDM control frames always
        // ride coherent QPSK R1/4 (profileForDataMode), so there is no longer a
        // DQPSK-vs-coherent control profile to negotiate at setConnected().
        // CONNECT/CONNECT_ACK ride MC-DPSK (control_waveform_, a separate decoder).

        // Configure OFDM config FIRST so it's correct when propagated to decoder
        config_.modulation = data_modulation_;
        config_.code_rate = data_code_rate_;
        config_.use_pilots = true;  // Always — must match OFDMChirpWaveform

        config_.pilot_spacing =
            ofdm_link_adaptation::recommendedPilotSpacing(data_modulation_, data_code_rate_);

        decoder_->setRate(data_code_rate_);

        // CRITICAL: Update StreamingDecoder when entering connected state
        // Config must be set above BEFORE propagating here
        if (streaming_decoder_) {
            streaming_decoder_->setMode(waveform_mode_, true);  // true = connected
            // CONNECT_ACK is not authoritative until the peer proves receipt
            // with a post-handshake frame.  Keep the decoder's bounded
            // MC-DPSK duplicate-CONNECT recovery alive during that half-open
            // responder interval.
            streaming_decoder_->setHandshakeConfirmationPending(true);

            // For OFDM modes, propagate the correct config (with proper pilot settings)
            if (protocol::isOFDMMode(waveform_mode_)) {
                streaming_decoder_->setConnectedOFDMMode(
                    waveform_mode_, config_, data_modulation_, data_code_rate_);
            }

            // Propagate known CFO from handshake to StreamingDecoder
            // This seeds the CFO feedback loop so the first OFDM frame uses correct CFO
            if (peer_cfo_known_) {
                streaming_decoder_->setKnownCFO(peer_cfo_hz_);
                LOG_MODEM(INFO, "setConnected: seeded CFO=%.2f Hz from handshake", peer_cfo_hz_);
            }
            if (protocol::isOFDMMode(waveform_mode_)) {
                streaming_decoder_->expectFullOFDMAnchorOnce();
            }
        }

        // Update StreamingEncoder for connected state
        if (streaming_encoder_) {
            streaming_encoder_->setMode(waveform_mode_);
            if (protocol::isOFDMMode(waveform_mode_)) {
                streaming_encoder_->setOFDMConfig(config_);
                streaming_encoder_->setDataMode(data_modulation_, data_code_rate_);
                streaming_encoder_->forceNextFrameFullPreamble();
            }
        }

        LOG_MODEM(INFO, "Entered connected state, configured for %s %s (pilots=%d, spacing=%d)",
                  modulationToString(data_modulation_), codeRateToString(data_code_rate_),
                  config_.use_pilots ? 1 : 0, config_.pilot_spacing);
    } else {
        // Switching to disconnected state - use robust mode for RX
        decoder_->setRate(CodeRate::R1_4);

        // SESSION SNR LIFECYCLE (external review 2026-07-10, finding 5): the
        // LoopbackStats cache is refreshed only by decode events, so without
        // this the previous QSO's SNR/fading stayed visible on the meters and
        // could be attached to the next session's first PONG. Clear the
        // channel-measurement fields at teardown; the idle meter (live overlay
        // in getStats) repopulates within ~1 s of idle audio.
        updateStats([&](LoopbackStats& st) {
            st.snr_db = 0.0f;
            st.snr_source = SNRSource::NONE;
            st.has_ofdm_broadband_snr_db = false;
            st.ofdm_broadband_snr_db = 0.0f;
            st.ofdm_internal_snr_db = 0.0f;
            st.sync_quality_db = 0.0f;
            st.synced = false;
        });

        // BUG-TNC-SESSION-001 fix (port from tools/ultra_tnc.cpp): a persistent
        // ModemEngine across multiple sessions must perform the same RX/TX
        // state reset on disconnect that ultra_tnc does, otherwise the next
        // session's first CONNECT_ACK gets rejected at the early
        // `cw_ok=0/cw_fail=0/is_ping=0` path because stale negotiated
        // modulation/code-rate, encoder mode, and cached peer CFO survive
        // the session boundary. Audio-engine pause/drain/resume is handled
        // by the App layer around this call (see app.cpp connection-changed
        // callback).
        if (streaming_decoder_) {
            streaming_decoder_->setHandshakeConfirmationPending(false);
            streaming_decoder_->reset();
            streaming_decoder_->setMode(protocol::WaveformMode::MC_DPSK, false);
            streaming_decoder_->setDataMode(Modulation::DQPSK, CodeRate::R1_4);
        }
        if (streaming_encoder_) {
            streaming_encoder_->setMode(protocol::WaveformMode::MC_DPSK);
            streaming_encoder_->setDataMode(Modulation::DQPSK, CodeRate::R1_4);
        }
        data_modulation_ = Modulation::DQPSK;
        data_code_rate_ = CodeRate::R1_4;
        peer_cfo_hz_ = 0.0f;
        peer_cfo_known_ = false;

        // Keep using connected waveform for the next TX (DISCONNECT ACK)
        // Save the current negotiated waveform BEFORE it might be reset
        disconnect_waveform_ = waveform_mode_;
        use_connected_waveform_once_ = true;
        LOG_MODEM(INFO, "Switched to disconnected mode (RX: DQPSK R1/4, next TX uses disconnect_waveform_=%d)",
                  static_cast<int>(disconnect_waveform_));
        handshake_complete_ = false;  // Reset for next connection
    }
    updateDualListen();
}

void ModemEngine::setHandshakeComplete(bool complete) {
    if (handshake_complete_ == complete) return;

    handshake_complete_ = complete;
    if (streaming_decoder_) {
        streaming_decoder_->setHandshakeConfirmationPending(
            connected_ && !complete);
    }

    if (complete) {
        LOG_MODEM(INFO, "Handshake complete, TX now uses waveform_mode_=%d",
                  static_cast<int>(waveform_mode_.load()));
    }
    updateDualListen();
}

void ModemEngine::setDataMode(Modulation mod, CodeRate rate) {
    data_modulation_ = mod;
    data_code_rate_ = rate;

    // CRITICAL: Pilots are ALWAYS enabled — OFDMChirpWaveform::configurePilotsForCodeRate()
    // always sets use_pilots=true regardless of differential/coherent modulation.
    // The pilot carriers are used for CFO tracking and channel estimation even in
    // differential mode. Disabling them causes data/pilot layout mismatch.
    config_.modulation = mod;
    config_.code_rate = rate;
    config_.use_pilots = true;  // Always — must match OFDMChirpWaveform

    config_.pilot_spacing =
        ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);

    // If already connected, update both TX and RX to match
    if (connected_) {
        decoder_->setRate(rate);

        LOG_MODEM(INFO, "TX/RX OFDM config updated: mod=%d, rate=%d, use_pilots=%d, pilot_spacing=%d",
                  static_cast<int>(mod), static_cast<int>(rate),
                  config_.use_pilots ? 1 : 0, config_.pilot_spacing);
    }

    // Update StreamingDecoder's waveform configuration
    // CRITICAL: Set OFDM config BEFORE setDataMode so decoder has correct pilot layout
    if (streaming_decoder_) {
        if (connected_ && protocol::isOFDMMode(waveform_mode_)) {
            streaming_decoder_->setConnectedOFDMMode(
                waveform_mode_, config_, mod, rate);
        } else {
            if (waveform_mode_ != protocol::WaveformMode::MC_DPSK) {
                streaming_decoder_->setOFDMConfig(config_);
            }
            streaming_decoder_->setDataMode(mod, rate);
        }
    }

    // Update StreamingEncoder to match
    if (streaming_encoder_) {
        if (waveform_mode_ != protocol::WaveformMode::MC_DPSK) {
            streaming_encoder_->setOFDMConfig(config_);
        }
        streaming_encoder_->setDataMode(mod, rate);
    }

    // File-class composite (design §14.14): enable cross-frame burst interleave
    // for connected coherent-QPSK OFDM data. Validated offline — R3/4 Good@20
    // frame recovery ~33%->~92% at group=8. Coherent QPSK uses the wide
    // (kWideOFDMWindowFrames=8) ARQ window, which equals kBurstInterleaveGroupFrames,
    // so a steady-state window fills exactly one interleave group. Both TX and RX
    // run setDataMode from negotiation, keeping the de-interleave matched.
    //
    // 2026-05-28: extended to coherent 8PSK (QAM8). 2026-05-30: extended to ALL OFDM
    // data modulations (incl. QAM16) — the burst interleaver + SR-ARQ chunker operate on
    // post-LDPC bits and are constellation-agnostic, so there is NO reason to special-case
    // the modulation here. The previous QPSK||QAM8 hardcode silently excluded QAM16: the
    // encoder left it un-interleaved while the connection's ARQ stayed whole-group, so
    // ALPHA skipped partial-group holes (the QAM16 offset-skip bug). file_class_composite
    // now means "OFDM file-class data of any modulation".
    const bool file_class_composite =
        connected_ && protocol::isOFDMMode(waveform_mode_);
    // Cross-frame byte-interleave is a SINGLE shared profile (default OFF -> per-frame
    // SR-ARQ), the same source of truth the connection's ARQ semantics and the on-wire
    // descriptor bit derive from, so encoder/ARQ/descriptor can never disagree.
    const bool burst_interleave_on =
        file_class_composite &&
        protocol::connection_policy::burstCrossFrameInterleaveOn(data_modulation_);
    const int burst_group =
        static_cast<int>(protocol::connection_policy::burstInterleaveGroupFrames());
    if (streaming_encoder_) {
        streaming_encoder_->setBurstInterleaveGroupSize(burst_group);
        streaming_encoder_->setBurstInterleave(burst_interleave_on);
    }
    if (streaming_decoder_) {
        streaming_decoder_->setBurstInterleaveGroupSize(burst_group);
        streaming_decoder_->setBurstInterleave(burst_interleave_on);
    }

    // burst_interleave reports the ACTUAL byte-permutation decision (burst_interleave_on),
    // not file_class_composite — the latter is only "is this a burst-eligible QPSK/QAM8
    // file frame" and is 1 even when ULTRA_BURST_INTERLEAVE=0 turns the permutation off.
    LOG_MODEM(INFO, "Data mode set to: %s (pilots=%d, spacing=%d, burst_interleave=%d)",
              getModeDescription(mod, rate), config_.use_pilots ? 1 : 0,
              config_.pilot_spacing, burst_interleave_on ? 1 : 0);
}

// R4: setAdaptivePreamblePeerFading + syncAdaptiveShortDataPreamble removed — the adaptive
// short-chirp re-anchor was superseded by the warm-sync hand-off (now the production default).

// NOTE: recommendDataMode() removed - use protocol::recommendDataMode() from waveform_selection.hpp

protocol::WaveformMode ModemEngine::recommendWaveformMode(float snr_db) {
    // Legacy SNR-only selection (use recommendWaveformAndRate for better results)
    // DPSK works down to -11 dB SNR (tested), so we use it for low SNR
    // OFDM requires ~27 dB on the unified in-band meter for reliable sync detection
    if (snr_db < 27.0f) {
        return protocol::WaveformMode::MC_DPSK;
    } else {
        return protocol::WaveformMode::OFDM_CHIRP;
    }
}

ModemEngine::WaveformRecommendation ModemEngine::recommendWaveformAndRate(float snr_db, float fading_index) {
    // Delegate to shared algorithm in protocol namespace
    auto rec = protocol::recommendWaveformAndRate(snr_db, fading_index);
    LOG_MODEM(DEBUG, "recommendWaveformAndRate: SNR=%.1f (%s), fading=%.2f -> %s %s (%.0f bps)",
              snr_db, snrSourceToString(SNRSource::NONE), fading_index,
              protocol::waveformModeToString(rec.waveform),
              codeRateToString(rec.rate),
              rec.estimated_throughput_bps);
    return rec;
}

void ModemEngine::setDPSKMode(DPSKModulation mod, int samples_per_symbol) {
    // Configure DPSK based on modulation type and symbol rate
    dpsk_config_.modulation = mod;
    dpsk_config_.samples_per_symbol = samples_per_symbol;

    const char* mod_name = "DQPSK";
    switch (mod) {
        case DPSKModulation::DBPSK: mod_name = "DBPSK"; break;
        case DPSKModulation::DQPSK: mod_name = "DQPSK"; break;
        case DPSKModulation::D8PSK: mod_name = "D8PSK"; break;
    }

    LOG_MODEM(INFO, "DPSK mode set: %s, %d samples/sym (%.1f baud), %.1f bps",
              mod_name,
              dpsk_config_.samples_per_symbol,
              dpsk_config_.symbol_rate(),
              dpsk_config_.raw_bps());
}

void ModemEngine::setCodecType(fec::CodecType type) {
    if (codec_type_ == type) return;  // No change

    codec_type_ = type;

    // Recreate decoder with new codec type (encoder is managed by StreamingEncoder)
    decoder_ = fec::CodecFactory::create(type, data_code_rate_);

    // Update the StreamingDecoders to use the same codec
    forEachRxDecoder([&](StreamingDecoder* d) { d->setCodecType(type); });

    LOG_MODEM(INFO, "Codec type set to: %s", decoder_->getName().c_str());
}

fec::CodecType ModemEngine::recommendCodecType(float snr_db) {
    // Codec selection based on SNR:
    // - LDPC: Works well at moderate-high in-band SNR (>15 dB), steep waterfall curve
    // - Convolutional: Better at very low in-band SNR (<15 dB), graceful degradation
    // - Turbo: Excellent near Shannon limit, but high latency
    //
    // For now, always use LDPC since it's the only implemented codec.
    // When convolutional codec is implemented, use it for in-band SNR < 15 dB.

    if (snr_db < 15.0f) {
        // Low SNR: Would prefer convolutional, but LDPC is all we have
        // return fec::CodecType::CONVOLUTIONAL;  // Future
        return fec::CodecType::LDPC;
    } else {
        // Moderate to high SNR: LDPC is optimal
        return fec::CodecType::LDPC;
    }
}

fec::CodecType ModemEngine::getCodecForWaveform(protocol::WaveformMode mode) {
    // Map waveform modes to optimal codec types:
    // - MC-DPSK (low SNR): Would benefit from convolutional (when implemented)
    // - OFDM_CHIRP (medium/high SNR): LDPC with R1/4 up through higher rates
    //
    // For now, always return LDPC since it's the only implemented codec.

    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:
            // Low SNR waveform - would prefer convolutional
            // return fec::CodecType::CONVOLUTIONAL;  // Future
            return fec::CodecType::LDPC;

        case protocol::WaveformMode::OFDM_CHIRP:
        case protocol::WaveformMode::OFDM_NARROW:
        default:
            return fec::CodecType::LDPC;
    }
}

int ModemEngine::recommendMCDPSKCarriers(float snr_db, float fading_index) {
    // MC-DPSK is used for roughly in-band SNR 10-20 dB range
    // (above 20 dB switches to OFDM)
    // Testing with 20Hz CFO shows 8 carriers is optimal for this range:
    //   - 8 carriers: 100% at legacy SNR 5 (about 15 dB in-band),
    //     moderate fading, 20Hz CFO
    //   - 9+ carriers: 40-60% at same conditions
    //
    // Always use 8 carriers for MC-DPSK - it's the most robust choice
    // for the challenging conditions where MC-DPSK is selected.
    (void)snr_db;       // Unused - always 8
    (void)fading_index; // Unused - always 8

    return 8;
}

} // namespace gui
} // namespace ultra
