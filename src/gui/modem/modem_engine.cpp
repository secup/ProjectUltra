// ModemEngine - Main implementation
// Constructor, destructor, configuration, and TX functions

#include <chrono>
#include "modem_engine.hpp"
#include "diagnostics/diagnostics_recorder.hpp"
#include "protocol/connection_policy.hpp"
#include "ultra/logging.hpp"
#include <cstring>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <utility>

namespace ultra {
namespace gui {

namespace {
Modulation mcDpskModulationForConfig(const MultiCarrierDPSKConfig& config) {
    if (config.bits_per_symbol == 1) return Modulation::DBPSK;
    if (config.bits_per_symbol == 3) return Modulation::D8PSK;
    return Modulation::DQPSK;
}

void copySNRMetrics(LoopbackStats& stats, const DecodeResult& result) {
    stats.snr_db = result.snr_db;
    stats.snr_source = result.snr_source;
    stats.mcdpsk_snr_routed_data_aided = result.mcdpsk_snr_routed_data_aided;
    stats.has_idle_in_band_snr_db = result.has_idle_in_band_snr_db;
    stats.idle_in_band_snr_db = result.idle_in_band_snr_db;
    stats.has_ofdm_broadband_snr_db = result.has_ofdm_broadband_snr_db;
    stats.ofdm_broadband_snr_db = result.ofdm_broadband_snr_db;
    stats.ofdm_internal_snr_db = result.ofdm_internal_snr_db;
    stats.sync_quality_db = result.sync_quality_db;
}

}

ModemEngine::ModemEngine()
    : ModemEngine(mc_dpsk_presets::robust_mid()) {
}

ModemEngine::ModemEngine(const MultiCarrierDPSKConfig& mc_dpsk_config) {
    config_ = presets::balanced();

    // CRITICAL: Disable pilots for DQPSK mode - uses all 30 carriers for data
    // This doubles throughput (30 data carriers vs 15 with pilots)
    // DQPSK is differential and doesn't need pilots for channel estimation
    config_.use_pilots = false;

    decoder_ = fec::CodecFactory::create(fec::CodecType::LDPC, CodeRate::R1_4);
    // NOTE: TX uses streaming_encoder_, RX uses streaming_decoder_

    // DPSK config (used by StreamingDecoder, kept here for API compatibility)
    dpsk_config_ = dpsk_presets::medium();

    // Chirp sync for robust presence detection on fading channels
    // Dual chirp (up + down) enables CFO estimation via radar technique
    sync::ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = config_.sample_rate;
    chirp_cfg.f_start = 300.0f;     // Start frequency (Hz)
    chirp_cfg.f_end = 2700.0f;      // End frequency (Hz)
    chirp_cfg.duration_ms = 500.0f; // 500ms per chirp (up + down = 1.0s chirps + gaps)
    chirp_cfg.gap_ms = 100.0f;      // Gap between up and down chirps
    chirp_cfg.use_dual_chirp = true; // Enable dual chirp for CFO estimation
    chirp_cfg.tx_cfo_hz = config_.tx_cfo_hz;  // Pass TX CFO for simulation
    chirp_sync_ = std::make_unique<sync::ChirpSync>(chirp_cfg);

    // Multi-Carrier DPSK (for fading channels - frequency diversity)
    // Default cold-call PHY is Robust-Mid: 8 carriers, 46.875 baud, DBPSK.
    // Test presets can override samples_per_symbol and DBPSK at construction.
    // 8 carriers is more robust than 13 at low SNR with CFO (tested: 100% vs 40% at moderate fading)
    // IMPORTANT: Sync chirp config with modem's chirp_sync_ so TX and RX use same chirp
    mc_dpsk_config_ = mc_dpsk_config;
    mc_dpsk_config_.chirp_f_start = chirp_cfg.f_start;
    mc_dpsk_config_.chirp_f_end = chirp_cfg.f_end;
    mc_dpsk_config_.chirp_duration_ms = chirp_cfg.duration_ms;
    mc_dpsk_config_.use_dual_chirp = chirp_cfg.use_dual_chirp;
    // Note: Actual MC-DPSK modulation is done by IWaveform via StreamingDecoder

    // Initialize StreamingEncoder (unified TX path)
    streaming_encoder_ = std::make_unique<StreamingEncoder>();
    streaming_encoder_->setOFDMConfig(config_);
    streaming_encoder_->setMCDPSKConfig(mc_dpsk_config_);
    streaming_encoder_->setDataMode(mcDpskModulationForConfig(mc_dpsk_config_), CodeRate::R1_4);

    // Initialize audio filters
    rebuildFilters();

    // ========================================================================
    // Initialize StreamingDecoder (primary RX path)
    // ========================================================================
    streaming_decoder_ = std::make_unique<StreamingDecoder>();
    // Production engine = real-time audio: enable the search load-shed
    // (BUG-DECODE-BACKLOG). Batch decoders (tests/tools) construct
    // StreamingDecoder directly and never shed.
    streaming_decoder_->setRealTimeAudio(true);

    // Set callbacks to wire into existing ModemEngine callbacks
    streaming_decoder_->setFrameCallback([this](const DecodeResult& result) {
        // Update SNR/sync stats before delivering frame so downstream callbacks
        // (ProtocolEngine via raw_data_callback_) read fresh channel estimates.
        updateStats([&](LoopbackStats& s) {
            copySNRMetrics(s, result);
            s.synced = result.success;
        });

        if (result.success && !result.frame_data.empty()) {
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
    streaming_decoder_->setBurstGroupCallback(
        [this](uint16_t group_seq, const std::vector<Bytes>& frames, bool all_ok,
               float quality, uint16_t frame_mask, bool interleaved, uint8_t group_size,
               bool geometry_proven) {
            updateStats([&](LoopbackStats& s) { s.synced = all_ok; });
            if (burst_group_callback_) {
                burst_group_callback_(group_seq, frames, all_ok, quality, frame_mask,
                                      interleaved, group_size, geometry_proven);
            }
            last_rx_complete_time_ = std::chrono::steady_clock::now();
        });

    streaming_decoder_->setDataSyncAcceptedCallback([this](float sync_correlation) {
        if (data_sync_accepted_callback_) {
            data_sync_accepted_callback_(sync_correlation);
        }
    });

    streaming_decoder_->setPingCallback([this](float snr_db, float cfo_hz) {
        // If narrowband chirp detected, switch control waveform for PONG/CONNECT_ACK
        if (streaming_decoder_->getDetectedBandwidth() == BandwidthMode::NARROW) {
            if (streaming_encoder_) {
                streaming_encoder_->setNarrowbandControl(true);
            }
            LOG_MODEM(INFO, "Narrowband chirp detected, switching control waveform");
        }
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

    // Sync StreamingDecoder with initial waveform mode
    // When disconnected, use MC_DPSK for PING detection (chirp-based sync)
    // When connected, use the negotiated waveform
    protocol::WaveformMode decoder_mode = connected_ ? waveform_mode_ : protocol::WaveformMode::MC_DPSK;
    if (connected_ || decoder_mode != protocol::WaveformMode::MC_DPSK) {
        streaming_decoder_->setMode(decoder_mode, connected_);
    } else {
        // StreamingDecoder ctor already initializes MC_DPSK disconnected defaults.
    }

    // Sync MC-DPSK carrier count with ModemEngine's config
    if (mc_dpsk_config_.num_carriers != 8 ||
        mc_dpsk_config_.samples_per_symbol != 512 ||
        mc_dpsk_config_.bits_per_symbol != 2) {
        streaming_decoder_->setMCDPSKConfig(mc_dpsk_config_);
        streaming_decoder_->setDataMode(mcDpskModulationForConfig(mc_dpsk_config_), CodeRate::R1_4);
    } else {
    }

    // Defer RX decode thread startup until audio is actually fed.
    // This reduces startup-time failure surface on low-end systems.
}

ModemEngine::~ModemEngine() {
    stopRxDecodeThread();
}

void ModemEngine::setLogPrefix(const std::string& prefix) {
    log_prefix_ = prefix;
    if (streaming_decoder_) {
        streaming_decoder_->setLogPrefix(prefix);
    }
}

void ModemEngine::setLocalCallsign(const std::string& call) {
    // The RX address filter in deliverFrame() must match against the operator's LIVE
    // callsign, which can change after construction (e.g. a VARA host issuing MYCALL).
    // log_prefix_ is only a logging label seeded from the config callsign; relying on it
    // silently dropped inbound frames once the live callsign diverged from the label
    // (a TNC config callsign of ALPHA vs a Winlink MYCALL of VA2MVR — BUG-CALLSIGN-FILTER).
    filter_callsign_ = call;
}

void ModemEngine::setSoftCombineBuffer(fec::SoftCombineBuffer* buffer) {
    if (streaming_decoder_) {
        streaming_decoder_->setSoftCombineBuffer(buffer);
    }
}

void ModemEngine::setHarqProvisionalContextCallback(
    StreamingDecoder::HarqProvisionalContextCallback cb) {
    if (streaming_decoder_) {
        streaming_decoder_->setHarqProvisionalContextCallback(std::move(cb));
    }
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void ModemEngine::setConfig(const ModemConfig& config) {
    LOG_MODEM(INFO, "setConfig called: new code_rate=%d, modulation=%d",
              static_cast<int>(config.code_rate), static_cast<int>(config.modulation));
    config_ = config;

    // BUG FIX: Don't change decoder rate here - it should stay at R1_4 for disconnected mode
    // The decoder rate should only change when setConnected(true) is called
    LOG_MODEM(INFO, "setConfig: decoder_rate=%d (decoder unchanged for disconnected)",
              static_cast<int>(decoder_->getRate()));

    // Propagate OFDM config to StreamingEncoder
    if (streaming_encoder_) {
        streaming_encoder_->setOFDMConfig(config_);
    }

    // Propagate OFDM config to StreamingDecoder for OFDM modes
    // This allows custom FFT/carrier settings (like NVIS mode with 1024 FFT)
    if (streaming_decoder_ && protocol::isOFDMMode(waveform_mode_)) {
        streaming_decoder_->setOFDMConfig(config_);
        LOG_MODEM(INFO, "setConfig: StreamingDecoder OFDM config updated (FFT=%d, carriers=%d)",
                  config_.fft_size, config_.num_carriers);
    }

    // Recreate chirp sync with new CFO setting (for simulation)
    sync::ChirpConfig chirp_cfg;
    chirp_cfg.sample_rate = config_.sample_rate;
    chirp_cfg.f_start = 300.0f;
    chirp_cfg.f_end = 2700.0f;
    chirp_cfg.duration_ms = 500.0f;  // 500ms per chirp (up + down = 1.0s chirps + gaps)
    chirp_cfg.gap_ms = 100.0f;       // Gap between up and down chirps
    chirp_cfg.use_dual_chirp = true; // Enable dual chirp for CFO estimation
    chirp_cfg.tx_cfo_hz = config_.tx_cfo_hz;
    chirp_sync_ = std::make_unique<sync::ChirpSync>(chirp_cfg);

    // Rebuild filters with new sample rate
    rebuildFilters();

    reset();
}

void ModemEngine::setFilterConfig(const FilterConfig& config) {
    filter_config_ = config;
    rebuildFilters();
}

void ModemEngine::setFilterEnabled(bool enabled) {
    filter_config_.enabled = enabled;
}

void ModemEngine::setPaprReductionEnabled(bool enabled) {
    if (streaming_encoder_) {
        streaming_encoder_->setPaprReductionEnabled(enabled);
    }
}

bool ModemEngine::isPaprReductionEnabled() const {
    return streaming_encoder_ && streaming_encoder_->getPaprReductionEnabled();
}

void ModemEngine::setMCDPSKConfig(const MultiCarrierDPSKConfig& config) {
    mc_dpsk_config_ = config;
    if (streaming_decoder_) {
        streaming_decoder_->setMCDPSKConfig(mc_dpsk_config_);
    }
    if (streaming_encoder_) {
        streaming_encoder_->setMCDPSKConfig(mc_dpsk_config_);
    }
    LOG_MODEM(INFO, "ModemEngine: MC-DPSK config carriers=%d sps=%d bits/sym=%d raw=%.1f bps",
              mc_dpsk_config_.num_carriers, mc_dpsk_config_.samples_per_symbol,
              mc_dpsk_config_.bits_per_symbol, mc_dpsk_config_.getRawBitRate());
}

void ModemEngine::rebuildFilters() {
    // Create bandpass filters for TX and RX
    // Use separate instances so they maintain independent state
    float sample_rate = static_cast<float>(config_.sample_rate);
    float low = filter_config_.lowFreq();
    float high = filter_config_.highFreq();
    int taps = filter_config_.taps;

    // Ensure valid frequency range
    low = std::max(50.0f, low);
    high = std::min(sample_rate / 2.0f - 50.0f, high);

    if (low < high) {
        auto filter = FIRFilter::bandpass(taps, low, high, sample_rate);
        tx_filter_ = std::make_unique<FIRFilter>(filter);
        rx_filter_ = std::make_unique<FIRFilter>(filter);
        LOG_MODEM(INFO, "Audio filters configured: %.0f-%.0f Hz, %d taps",
                  low, high, taps);
    } else {
        LOG_MODEM(WARN, "Invalid filter range: %.0f-%.0f Hz, filters disabled",
                  low, high);
        tx_filter_.reset();
        rx_filter_.reset();
    }
}

// ============================================================================
// TX: TRANSMIT
// ============================================================================

std::vector<float> ModemEngine::transmit(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return transmit(data);
}

std::vector<float> ModemEngine::transmit(const Bytes& data) {
    if (data.empty()) {
        return {};
    }

    LOG_MODEM(INFO, "[%s] TX: Input %zu bytes, first 4: %02x %02x %02x %02x",
              log_prefix_.c_str(),
              data.size(),
              data.size() > 0 ? data[0] : 0,
              data.size() > 1 ? data[1] : 0,
              data.size() > 2 ? data[2] : 0,
              data.size() > 3 ? data[3] : 0);

    // ========================================================================
    // 1. Determine waveform mode (4-way decision)
    // ========================================================================
    protocol::WaveformMode tx_waveform_mode;
    if (use_connected_waveform_once_) {
        tx_waveform_mode = disconnect_waveform_;
        LOG_MODEM(INFO, "[%s] TX: use_connected_waveform_once_ -> disconnect_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(tx_waveform_mode));
    } else if (!connected_) {
        tx_waveform_mode = connect_waveform_;
        LOG_MODEM(INFO, "[%s] TX: NOT connected -> connect_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(tx_waveform_mode));
    } else if (!handshake_complete_) {
        tx_waveform_mode = last_rx_waveform_;
        LOG_MODEM(INFO, "[%s] TX: Handshake mode -> last_rx_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(tx_waveform_mode));
    } else {
        tx_waveform_mode = waveform_mode_;
        LOG_MODEM(INFO, "[%s] TX: Connected+handshake -> waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(tx_waveform_mode));
    }
    bool is_ofdm = protocol::isOFDMMode(tx_waveform_mode);

    // ========================================================================
    // 2. Determine modulation and code rate
    // ========================================================================
    Modulation tx_modulation = protocol::isOFDMMode(tx_waveform_mode)
        ? Modulation::DQPSK
        : mcDpskModulationForConfig(mc_dpsk_config_);
    CodeRate tx_code_rate = CodeRate::R1_4;

    if ((connected_ && handshake_complete_) || use_connected_waveform_once_) {
        tx_modulation = data_modulation_;
        tx_code_rate = data_code_rate_;
        LOG_MODEM(INFO, "[%s] TX: Using %s %s (%s)",
                  log_prefix_.c_str(),
                  modulationToString(tx_modulation),
                  codeRateToString(tx_code_rate),
                  connected_ ? "negotiated" : "disconnect ACK");
    }

    // Save one-shot flag before clearing (needed for light preamble decision below)
    bool is_disconnect_ack = use_connected_waveform_once_;
    if (use_connected_waveform_once_) {
        use_connected_waveform_once_ = false;
    }

    // ========================================================================
    // 3. Configure StreamingEncoder and encode
    // ========================================================================
    streaming_encoder_->setMode(tx_waveform_mode);
    streaming_encoder_->setDataMode(tx_modulation, tx_code_rate);

    const auto header = protocol::v2::parseHeader(data);
    const bool is_data_frame = header.valid && protocol::v2::isDataFrame(header.type);
    const bool is_disconnect_teardown_ack = header.valid &&
        header.type == protocol::v2::FrameType::ACK &&
        header.seq == protocol::v2::DISCONNECT_SEQ;
    // ULTRA_LIGHT_TURN_TOKENS (default OFF => byte-identical): let the DATA-turn
    // NEGOTIATION pair ride a light preamble.
    //
    // These two are NOT in the same acquisition situation as the burst-coordination
    // tokens below, even though they were bundled with them. GROUP_ACK crosses a
    // turnaround after an ~11 s data burst, so the peer is genuinely cold (measured:
    // light correlates ~0.88, rejected by the 0.90 gate) — that stays full-anchored.
    // TURNOVER/TURN_REQUEST are exchanged every ~4 s DURING a negotiation, so the peer
    // decoded one of our frames seconds ago and is warm. They also change no geometry
    // (unlike MODE_CHANGE), so the peer's sync remains valid by construction.
    //
    // The cost of the bundling, measured on IONOS: a 20-byte turn frame with a full
    // dual-chirp anchor is 67,680 samples = 1.41 s, versus 10,080 = 0.21 s light — 6.7x.
    // Two of those per exchange occupy 2.82 s of a 4.33 s retry cycle (~65%), which is
    // why the grant cannot fit in the requester's listening window: 121 grants
    // transmitted / 7 decoded, handovers 41-176 s, half the return transfers lost.
    // At 0.21 s the same exchange occupies ~10% and the collision window disappears
    // rather than being tuned around.
    static const bool kLightTurnTokens = [] {
        const char* e = std::getenv("ULTRA_LIGHT_TURN_TOKENS");
        return e != nullptr && std::atoi(e) != 0;
    }();
    const bool is_turn_negotiation = header.valid &&
        (header.type == protocol::v2::FrameType::TURNOVER ||
         header.type == protocol::v2::FrameType::TURN_REQUEST);

    const bool is_turn_control = header.valid &&
        ((!kLightTurnTokens &&
          (header.type == protocol::v2::FrameType::TURNOVER ||
           header.type == protocol::v2::FrameType::TURN_REQUEST)) ||
         header.type == protocol::v2::FrameType::FILE_CANCEL ||
         // §14.27: the GROUP_ACK is the one-way burst transport's whole-burst
         // coordination token. It crosses a half-duplex turnaround AFTER an ~11 s
         // data burst, so the initiator is NOT warm — a light preamble's short
         // 100 ms chirp only correlates ~0.88 and is rejected by the 0.90 data-sync
         // gate. Carry a full chirp+LTS anchor (corr ~0.93+, reliably accepted),
         // exactly like the other link-state transition tokens above.
         header.type == protocol::v2::FrameType::GROUP_ACK ||
         // DISCONNECT is the session-ending link-state transition — the most
         // important one to land. After a one-way burst transfer the receiver is
         // NOT warm, and a light-preamble DISCONNECT's short chirp is missed by
         // the receiver's coarse post-transfer chirp search, stranding it
         // connected forever (responder never tears down / exits). Send it with a
         // full chirp+LTS anchor so it is robustly acquired, like the GROUP_ACK.
         header.type == protocol::v2::FrameType::DISCONNECT ||
         // The sentinel ACK is the other half of the same close transaction.
         // The initiator has entered control-only cold acquisition after a full
         // DISCONNECT turn, so treating seq=0xFFFF like an ordinary warm/light
         // DATA ACK makes teardown depend on a fading-sensitive fallback. Give
         // both directions the same full-anchor acquisition contract.
         is_disconnect_teardown_ack ||
         // GROUP_NACK is the fast-resend coordination token (§14.30); like the
         // GROUP_ACK it crosses the turnaround to a non-warm sender, so full anchor.
         header.type == protocol::v2::FrameType::GROUP_NACK);
    const bool is_mode_change = header.valid &&
        header.type == protocol::v2::FrameType::MODE_CHANGE;
    if (is_ofdm && connected_ && handshake_complete_ && is_data_frame && streaming_decoder_) {
        streaming_decoder_->clearFullOFDMAnchorExpectation();
    }

    // A DATA frame emitted through transmit() is a separate half-duplex turn,
    // not an in-burst continuation. The peer may have just sent ACK/SACK
    // control and correctly armed a full-anchor expectation, so DATA repairs
    // must re-anchor with the full chirp. Short re-anchors are retained for
    // physically contiguous frames inside transmitBurst().
    //
    // Connected OFDM ACK/NACK control frames stay light for turnaround
    // efficiency. FILE_CANCEL, the burst-coordination tokens (GROUP_ACK/NACK,
    // DISCONNECT) and MODE_CHANGE carry a full anchor: the first three cross to a
    // peer that is genuinely cold, and MODE_CHANGE changes the physical geometry the
    // peer would need the anchor to re-acquire.
    //
    // TURNOVER/TURN_REQUEST are split out under ULTRA_LIGHT_TURN_TOKENS (see above):
    // they are exchanged every few seconds during a negotiation against a WARM peer
    // and change no geometry, so the full anchor buys robustness that is already
    // there and costs 6.7x the airtime on the one frame that must fit in a gap.
    bool use_light = ((connected_ && handshake_complete_) || is_disconnect_ack) &&
                     is_ofdm &&
                     !is_data_frame &&
                     !is_turn_control &&
                     !is_mode_change;

    // BUG-TNC-B2F-002: revert the encoder's LDPC lifting Z to the non-burst default
    // (27) before encoding ANY frame on this transmit() path. A burst lifts the
    // encoder to z=81 (n=1944) via setLDPCLiftingZ(81) in transmitBurst(), but
    // nothing reverted it for the NEXT non-burst frame. So after a file transfer an
    // interactive/SR-ARQ DATA frame — the Winlink-B2F FF terminator, a chat line, an
    // ARQ repair — was still encoded at z=81 (~106880 samples) while the receiver,
    // which DOES revert at group-end (finalizeBurstGroup → setActiveLDPCLiftingZ(27)),
    // decoded it as z=27 (~17920 samples). BRAVO read the first ~17% of a z=81 frame
    // as a z=27 frame → saturated-magnitude/random-sign LLRs (|H|≈0) → LDPC 0/CW →
    // never ACKed → ALPHA retransmits forever → transfer stalls. EVERY frame emitted
    // through transmit() is non-burst (burst DATA rides transmitBurst()/encodeBurstLight),
    // so z=27 is always correct here; the burst path re-lifts to 81 itself per group.
    if (is_ofdm && streaming_encoder_) {
        streaming_encoder_->setLDPCLiftingZ(27);
    }

    if (is_turn_negotiation) {
        LOG_MODEM(INFO, "[%s] TURN-TOKEN %s preamble=%s", log_prefix_.c_str(),
                  protocol::v2::frameTypeToString(header.type),
                  use_light ? "LIGHT" : "FULL");
    }
    auto samples = use_light ? streaming_encoder_->encodeFrameLight(data)
                             : streaming_encoder_->encodeFrame(data);

    if (samples.empty()) {
        LOG_MODEM(ERROR, "[%s] TX: StreamingEncoder returned empty samples", log_prefix_.c_str());
        return {};
    }

    LOG_MODEM(INFO, "[%s] TX: %zu bytes -> %zu samples (%s, %s, %s preamble)",
              log_prefix_.c_str(), data.size(), samples.size(),
              protocol::waveformModeToString(tx_waveform_mode),
              modulationToString(tx_modulation),
              use_light ? "light" : "full");
    char fields[256];
    std::snprintf(fields, sizeof(fields),
                  "{\"bytes\":%zu,\"samples\":%zu,\"waveform\":\"%s\","
                  "\"mod\":\"%s\",\"rate\":\"%s\"}",
                  data.size(), samples.size(),
                  protocol::waveformModeToString(tx_waveform_mode),
                  modulationToString(tx_modulation),
                  codeRateToString(tx_code_rate));
    ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
        "phy", "frame.tx", fields);

    if (std::getenv("ULTRA_TXLAT_DIAG")) {
        // Logs the SAMPLE COUNT so burst airtime is measured (n/48000) rather than modelled
        // from data_ms — the modelling error is what inflated the old turnaround figure.
        const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        LOG_MODEM(WARN, "TXLAT audio_ready us=%lld n=%zu",
                  static_cast<long long>(now_us), samples.size());
    }
    auto processed = postProcessTx(samples);
    if (is_ofdm && streaming_decoder_) {
        const size_t turn_samples =
            static_cast<size_t>(48000ULL * static_cast<uint64_t>(turnaround_delay_ms_) / 1000ULL);
        streaming_decoder_->seedExpectedFrameArrivalAfterSamples(processed.size() + turn_samples);
    }
    return processed;
}

// ============================================================================
// PING/PONG PROBE (minimal presence check)
// ============================================================================

std::vector<float> ModemEngine::transmitBurst(
    const std::vector<Bytes>& frame_data_list,
    uint16_t group_seq,
    BurstAnchorOptions anchor_options) {
    if (frame_data_list.empty()) return {};
    // §14.27: stamp the group sequence into the burst descriptor so the RX can
    // whole-burst-ACK the right group. 0 for legacy/single-shot callers.
    streaming_encoder_->setBurstGroupSeq(group_seq);

    // Burst mode is for connected OFDM and MC-DPSK DATA windows.
    streaming_encoder_->setMode(waveform_mode_);
    streaming_encoder_->setDataMode(data_modulation_, data_code_rate_);

    // TRANSPORT MERGE (ULTRA_UNIFIED_SEQ): the unified path sizes each burst to the
    // half-duplex airtime BUDGET, which is often SMALLER than the configured burst
    // group size (e.g. 4 frames vs a group of 6). encodeBurstLight forms groups as
    // floor(frames / group_size), so a sub-group-size burst yields ZERO groups → it
    // emits NO BURST_HEADER → the RX never group-decodes (falls to the per-frame path,
    // where my group-boundary ack can't run). Each transmitBurst() IS exactly one
    // logical burst, so declare the group size = this burst's frame count: encodeBurst
    // then forms one self-describing group and stamps a BURST_HEADER(group=N). The RX
    // adapts its own group size from that descriptor (streaming_ofdm_decode.cpp:713-714).
    // TRANSPORT MERGE (2026-06-06): unified is THE OFDM path now — always size the group to
    // this burst's frame count so encodeBurstLight forms exactly one self-describing group +
    // BURST_HEADER(group=N), which the RX adapts to (streaming_ofdm_decode.cpp:713-714).
    if (protocol::isOFDMMode(waveform_mode_) && !frame_data_list.empty()) {
        streaming_encoder_->setBurstInterleaveGroupSize(
            static_cast<int>(frame_data_list.size()));
    }

    if (protocol::isOFDMMode(waveform_mode_) && connected_ && handshake_complete_ && streaming_decoder_) {
        const bool has_data_frame = std::any_of(
            frame_data_list.begin(), frame_data_list.end(),
            [](const Bytes& frame) {
                const auto header = protocol::v2::parseHeader(frame);
                return header.valid && protocol::v2::isDataFrame(header.type);
            });
        if (has_data_frame) {
            streaming_decoder_->clearFullOFDMAnchorExpectation();
        }
    }

    // Self-describing burst (design §14.17/§14.19): the BURST_HEADER descriptor is
    // emitted INSIDE encodeBurstLight — a full-anchor 1-CW control frame at the head
    // of the interleaved group — so the receiver configures its group decode from
    // the SENDER's declared params (fixing the cross-station 0/8). Proven on the
    // faithful harness incl. a deliberately mis-configured decoder
    // (measure_ack_fer --burst-descriptor 1: AWGN40 40/40, Good@20 recovers to
    // baseline) once the descriptor-consume profile-restore (§14.23) was added.
    // Enabled by default; set ULTRA_BURST_DESCRIPTOR=0 to disable as an escape hatch.
    static const bool kBurstDescriptorEnabled = [] {
        const char* env = std::getenv("ULTRA_BURST_DESCRIPTOR");
        return !(env && env[0] == '0');  // default ON; "0" disables
    }();
    // §14.32 experiment: "one BURST_HEADER per file". The format (group size,
    // cw/frame, mod/rate, interleave) is identical for every group in a transfer,
    // and the receiver remembers it (have_burst_descriptor_/fixed_frame_codewords_
    // are sticky). Under stop-and-wait, group 0 (+ its header) is ACKed before
    // group 1 ships — and resends of group 0 keep group_seq==0 — so the receiver
    // always has the format before any header-less inner burst. Emitting the
    // header only on group 0 drops ~1.4 s of redundant re-announcement per inner
    // burst. RISK: inner bursts then carry no chirp, so the receiver must re-acquire
    // them by warm-sync timing prediction + the group-start LTS marker alone.
    // Opt-in via ULTRA_BURST_HEADER_ONCE=1 (default OFF = header every burst).
    static const bool kBurstHeaderOnce = [] {
        const char* env = std::getenv("ULTRA_BURST_HEADER_ONCE");
        return env && env[0] == '1';
    }();
    const bool emit_descriptor_this_burst =
        kBurstDescriptorEnabled && (!kBurstHeaderOnce || group_seq == 0);
    if (emit_descriptor_this_burst && protocol::isOFDMMode(waveform_mode_)) {
        streaming_encoder_->setBurstDescriptorEnabled(true);
        streaming_encoder_->setBurstDescriptorIdentity("", "");
    } else {
        streaming_encoder_->setBurstDescriptorEnabled(false);
    }

    // Per-burst LDPC lifting selection. Scoped connection-layer profiles may
    // opt DATA into long LDPC (Z=81, n=1944); MC-DPSK and OFDM control stay at
    // short Z=27 (n=648). A raw environment override is not accepted here:
    // serializer, timing, descriptor and PHY must move as one policy tuple.
    //
    // CW and Z are independent descriptor fields. In particular, the
    // ULTRA_8PSK_LONG_LDPC experiment substitutes cw4/Z81 for cw12/Z27:
    // identical payload capacity and 7776 coded bits, but four longer LDPC
    // blocks. Derive CW from the serialized DATA header instead of imposing a
    // historical z81=>cw2 rule that would contradict both frame and descriptor.
    if (protocol::isOFDMMode(waveform_mode_)) {
        // Per-burst Z is decided by the connection-layer traffic-class policy
        // (Connection::selectBurstLiftingZ) and pushed here via setBurstLiftingZ
        // — no env read. The encoder writes it into the BURST_HEADER descriptor
        // so the RX matches.
        int declared_cw = 0;
        for (const auto& frame : frame_data_list) {
            const auto header = protocol::v2::parseHeader(frame);
            if (!header.valid || header.is_control ||
                header.total_cw < protocol::v2::kMinFixedFrameCodewords ||
                header.total_cw > protocol::v2::kMaxFixedFrameCodewords) {
                continue;
            }
            if (declared_cw != 0 && declared_cw != header.total_cw) {
                LOG_MODEM(ERROR,
                          "[%s] TX Burst: mixed fixed-CW headers in OFDM group "
                          "(%d then %u) — refusing an ambiguous descriptor",
                          log_prefix_.c_str(), declared_cw,
                          static_cast<unsigned>(header.total_cw));
                return {};
            }
            declared_cw = header.total_cw;
        }
        if (burst_lifting_z_ == 81 && !emit_descriptor_this_burst) {
            LOG_MODEM(ERROR,
                      "[%s] TX Burst: Z=81 requires a BURST_HEADER on this "
                      "physical group — refusing unannounced long LDPC",
                      log_prefix_.c_str());
            return {};
        }
        streaming_encoder_->setLDPCLiftingZ(burst_lifting_z_);
        if (declared_cw != 0) {
            streaming_encoder_->setFixedFrameCodewords(declared_cw);
        } else if (burst_lifting_z_ == 81) {
            LOG_MODEM(ERROR,
                      "[%s] TX Burst: Z=81 group has no fixed DATA geometry "
                      "to announce — refusing descriptor-less long LDPC",
                      log_prefix_.c_str());
            return {};
        }
    } else {
        streaming_encoder_->setLDPCLiftingZ(27);  // MC-DPSK always short
    }

    // 2026-05-28: wrap the encoder TX path so any uncaught exception (e.g. a
    // BurstInterleaver / FrameInterleaver size mismatch on a not-yet-validated
    // (z, cw) combination) is surfaced as a WARN log rather than propagating
    // through the audio thread and silently killing the process. The earlier
    // z=81/cw=1 silent abort had no IPS crash dump because it was an uncaught
    // throw, not SIGSEGV — this catch turns "alpha disappears" into "ERROR:
    // <message>" so the actual failure point is visible.
    std::vector<float> samples;
    try {
        samples = streaming_encoder_->encodeBurstLight(frame_data_list,
                                                        anchor_options);
    } catch (const std::exception& e) {
        LOG_MODEM(ERROR, "[%s] TX Burst: encodeBurstLight THREW '%s' (frames=%zu, z=%u, cw=%d, mod=%s, rate=%s) — TX aborted, no samples",
                  log_prefix_.c_str(), e.what(), frame_data_list.size(),
                  static_cast<unsigned>(streaming_encoder_->getLDPCLiftingZ()),
                  streaming_encoder_->getFixedFrameCodewords(),
                  modulationToString(data_modulation_),
                  codeRateToString(data_code_rate_));
        return {};
    } catch (...) {
        LOG_MODEM(ERROR, "[%s] TX Burst: encodeBurstLight threw UNKNOWN exception — TX aborted",
                  log_prefix_.c_str());
        return {};
    }

    if (samples.empty()) {
        LOG_MODEM(ERROR, "[%s] TX Burst: encodeBurstLight returned empty", log_prefix_.c_str());
        return {};
    }

    LOG_MODEM(INFO, "[%s] TX Burst: %zu frames -> %zu samples (%s, %s)",
              log_prefix_.c_str(), frame_data_list.size(), samples.size(),
              protocol::waveformModeToString(waveform_mode_),
              modulationToString(data_modulation_));

    if (std::getenv("ULTRA_TXLAT_DIAG")) {
        // Logs the SAMPLE COUNT so burst airtime is measured (n/48000) rather than modelled
        // from data_ms — the modelling error is what inflated the old turnaround figure.
        const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        LOG_MODEM(WARN, "TXLAT audio_ready us=%lld n=%zu",
                  static_cast<long long>(now_us), samples.size());
    }
    auto processed = postProcessTx(samples);
    if (protocol::isOFDMMode(waveform_mode_) && streaming_decoder_) {
        const size_t turn_samples =
            static_cast<size_t>(48000ULL * static_cast<uint64_t>(turnaround_delay_ms_) / 1000ULL);
        streaming_decoder_->seedExpectedFrameArrivalAfterSamples(processed.size() + turn_samples);

        // §14.27: in the one-way burst transport, after we send a group the reply
        // is a FULL chirp+LTS anchor GROUP_ACK that arrives after a long, variable
        // group decode — NOT a quick warm-sync light reply at a predictable offset.
        // The warm-sync narrow reply-prediction window (sized/placed for a light
        // LTS) mis-times it and the batched fallback chirp search runs after the
        // chirp has passed (strong rms, ~0.16 corr → rejected → group resent on
        // timeout). Arm full-anchor wide acquisition so the sender catches the
        // GROUP_ACK's chirp cleanly. This OVERRIDES the warm-sync narrow window
        // (use_full_ofdm_anchor_search disables use_light_search). Gated on the
        // burst transport so the normal QSO warm-sync turnaround is unchanged.
        if (burst_transport_rx_enabled_ && connected_ && handshake_complete_) {
            streaming_decoder_->expectFullOFDMAnchorOnce();
        }
    }
    return processed;
}

std::vector<float> ModemEngine::transmitPing() {
    auto samples = streaming_encoder_->encodePing();

    LOG_MODEM(INFO, "[%s] TX PING (chirp): %zu samples",
              log_prefix_.c_str(), samples.size());

    return postProcessTx(samples);
}

std::vector<float> ModemEngine::transmitToneBurstAck(
    const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload,
    uint32_t symbol_ms) {
    // §15 step 4d-iii. Delegate to StreamingEncoder::encodeToneBurstAck (4c).
    // Returned samples flow through the same postProcessTx pipeline as every
    // other TX (peak normalization, etc.) so the audio output amplitude
    // matches what the rest of the modem produces.
    if (!streaming_encoder_) {
        LOG_MODEM(ERROR, "[%s] transmitToneBurstAck: no streaming encoder",
                  log_prefix_.c_str());
        return {};
    }
    // BUG-BURST-STALE-GEOMETRY (2026-07-28): snoop the rung WE are commanding.
    // Under RX rate authority this ack IS the sender's next rung command, so the
    // receiver knows the sender's next frame geometry before the sender does. Hand
    // the canonical index to the decoder so a MISSED BURST_HEADER can be sliced with
    // the commanded rung instead of the latched (previous) one.
    // No protocol-layer change: connection.cpp already stamped these five bits.
    // This is the single point every ToneBurstAckPayload passes through from BOTH
    // frontends (GUI app + ultra_tnc), and ModemEngine owns both encoder and decoder.
    // Four mandatory rejects, all present: authority OFF (the bits then mean a
    // quantized quality + a RELATIVE demote — reconstructing an index is garbage),
    // kRungCmdReserved (the WAITING-REBASE voice, which would decode as index 24),
    // kRungIdxNone (no command), and out-of-range.
    if (streaming_decoder_ && protocol::rxRateAuthorityEnabled() &&
        payload.rung_cmd != ultra::waveform::tone_burst_ack::kRungCmdReserved) {
        const uint8_t authority_word = static_cast<uint8_t>(
            (payload.rate_hint & 0x7) | ((payload.rung_cmd & 0x3) << 3));
        const uint8_t rung_idx = static_cast<uint8_t>(
            authority_word & protocol::kRungAuthorityIndexMask);
        if (rung_idx != protocol::kRungIdxNone && rung_idx < protocol::kRungIdxCount) {
            streaming_decoder_->setCommandedRungIndex(rung_idx);
        }
    }

    auto samples = streaming_encoder_->encodeToneBurstAck(payload, symbol_ms);
    LOG_MODEM(INFO,
              "[%s] TX ToneBurstAck: group_seq=%u type=%s frame_mask=0x%04X "
              "samples=%zu",
              log_prefix_.c_str(),
              static_cast<unsigned>(payload.group_seq),
              payload.type == ultra::waveform::tone_burst_ack::AckType::Nack
                  ? "NACK"
                  : "ACK",
              static_cast<unsigned>(payload.frame_mask),
              samples.size());
    // #68: the ACK's lead-in lands squarely in the half-duplex turnaround, and an ACK is a tiny
    // (~324ms) low-PA-thermal tone-burst — the safest TX to shorten. ULTRA_TX_ACK_LEADIN_MS (default
    // -1 = use the global lead-in, i.e. unchanged) lets the ACK lead-in be reduced independently of
    // the higher-PA-duty data bursts. Conservative-by-default until real-radio-proven.
    static const int kAckLeadInMs = [] {
        const char* e = std::getenv("ULTRA_TX_ACK_LEADIN_MS");
        return (e && *e) ? std::max(0, std::atoi(e)) : -1;
    }();
    return postProcessTx(samples, /*lead_in_ms=*/kAckLeadInMs);
}

std::vector<float> ModemEngine::transmitPong() {
    // Pong is identical to ping - context determines meaning
    // (Ping = initiator probe, Pong = responder reply)
    LOG_MODEM(INFO, "[%s] TX PONG (same as PING)", log_prefix_.c_str());
    return transmitPing();
}

// ============================================================================
// TX POST-PROCESSING (lead-in, filter, scale, stats)
// ============================================================================

std::vector<float> ModemEngine::postProcessTx(const std::vector<float>& samples,
                                              int lead_in_ms, int tail_ms) {
    // TX guard timing (#68). The lead-in is purely TX/PA-side margin (PTT relay + PA ramp + ALC
    // settling) so the chirp/signal isn't clipped during key-up — the RECEIVER does not need it
    // (the chirp detector searches). The legacy fixed 150ms is early-project (Jan/Feb 2026)
    // over-provisioning vs real radios (IC-7300 T/R ~15ms, FT-891 ~20ms; ALC settling tens of ms);
    // it sits FIXED on EVERY TX (data, ACK, ping), and the ACK's copy lands squarely in the
    // half-duplex turnaround. Now CONFIGURABLE (default-unchanged): callers may pass a shorter
    // lead-in (the ACK path does — ACKs are tiny + low PA-thermal), or override globally via
    // ULTRA_TX_LEADIN_MS / ULTRA_TX_TAIL_MS. FIDELITY CAVEAT: the cheap-card rig has no real 100W
    // PA, so a reduction stays CONFIGURABLE + conservative-by-default until real-radio-proven.
    const int eff_lead_in_ms = (lead_in_ms >= 0)
        ? lead_in_ms
        : protocol::connection_policy::configuredTxLeadInMs();
    const int eff_tail_ms = (tail_ms >= 0)
        ? tail_ms
        : protocol::connection_policy::configuredTxTailMs();

    // Combine lead-in + signal + tail guard
    const size_t LEAD_IN_SAMPLES = static_cast<size_t>(
        protocol::connection_policy::txGuardSamplesForMs(eff_lead_in_ms));
    const size_t TAIL_SAMPLES = static_cast<size_t>(
        protocol::connection_policy::txGuardSamplesForMs(eff_tail_ms));

    std::vector<float> output;
    output.reserve(LEAD_IN_SAMPLES + samples.size() + TAIL_SAMPLES);
    output.resize(LEAD_IN_SAMPLES, 0.0f);
    output.insert(output.end(), samples.begin(), samples.end());
    output.resize(output.size() + TAIL_SAMPLES, 0.0f);

    // Apply TX bandpass filter
    if (filter_config_.enabled && tx_filter_) {
        SampleSpan span(output.data(), output.size());
        output = tx_filter_->process(span);
    }

    // Scale for audio output
    float max_val = 0.0f;
    for (float s : output) {
        max_val = std::max(max_val, std::abs(s));
    }
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) {
            s *= scale;
        }
    }

    // Update stats
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_sent++;

        int bits_per_carrier = static_cast<int>(getBitsPerSymbol(config_.modulation));
        float code_rate = getCodeRateValue(config_.code_rate);
        float symbol_rate = config_.sample_rate / (float)config_.getSymbolDuration();
        stats_.throughput_bps = static_cast<int>(
            config_.getDataCarriers() * bits_per_carrier * code_rate * symbol_rate
        );
    }

    return output;
}

// ============================================================================
// TEST SIGNAL GENERATION
// ============================================================================

std::vector<float> ModemEngine::generateTestTone(float duration_sec) {
    size_t num_samples = static_cast<size_t>(config_.sample_rate * duration_sec);
    std::vector<float> tone(num_samples);

    float freq = 1500.0f;
    float phase = 0.0f;
    float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

    for (size_t i = 0; i < num_samples; i++) {
        tone[i] = 0.7f * std::sin(phase);
        phase += phase_inc;
        if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
    }

    LOG_MODEM(INFO, "Generated test tone: %.1f Hz, %.1f sec, %zu samples",
              freq, duration_sec, num_samples);
    return tone;
}

std::vector<float> ModemEngine::transmitTestPattern(int pattern) {
    Bytes test_data(21);

    switch (pattern) {
        case 0:
            std::fill(test_data.begin(), test_data.end(), 0x00);
            LOG_MODEM(INFO, "TX Test Pattern: ALL ZEROS (%zu bytes)", test_data.size());
            break;
        case 1:
            {
                uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
                for (size_t i = 0; i < test_data.size(); i++) {
                    test_data[i] = deadbeef[i % 4];
                }
            }
            LOG_MODEM(INFO, "TX Test Pattern: DEADBEEF (%zu bytes)", test_data.size());
            break;
        case 2:
            std::fill(test_data.begin(), test_data.end(), 0x55);
            LOG_MODEM(INFO, "TX Test Pattern: ALTERNATING 0101 (%zu bytes)", test_data.size());
            break;
        default:
            std::fill(test_data.begin(), test_data.end(), 0xAA);
            LOG_MODEM(INFO, "TX Test Pattern: ALTERNATING 1010 (%zu bytes)", test_data.size());
    }

    // Use StreamingEncoder for test pattern TX
    streaming_encoder_->setMode(protocol::WaveformMode::OFDM_CHIRP);
    streaming_encoder_->setDataMode(Modulation::DQPSK, CodeRate::R1_4);
    auto samples = streaming_encoder_->encodeFrame(test_data);

    if (samples.empty()) {
        LOG_MODEM(ERROR, "TX Test: Failed to encode");
        return {};
    }

    LOG_MODEM(INFO, "TX Test: %zu bytes -> %zu samples (R1/4 forced)", test_data.size(), samples.size());
    return postProcessTx(samples);
}

std::vector<float> ModemEngine::transmitRawOFDM(int pattern) {
    // Generate raw OFDM test (no LDPC) for layer-by-layer debugging
    size_t test_size = 81;  // Size of one R1/4 encoded codeword
    Bytes test_data(test_size);

    switch (pattern) {
        case 0:
            for (size_t i = 0; i < test_size; i++) {
                test_data[i] = (i % 2 == 0) ? 0xAA : 0x55;
            }
            LOG_MODEM(INFO, "TX Raw OFDM: AA/55 alternating (%zu bytes)", test_size);
            break;
        case 1:
            {
                uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
                for (size_t i = 0; i < test_size; i++) {
                    test_data[i] = deadbeef[i % 4];
                }
            }
            LOG_MODEM(INFO, "TX Raw OFDM: DEADBEEF (%zu bytes)", test_size);
            break;
        default:
            std::fill(test_data.begin(), test_data.end(), 0xAA);
            LOG_MODEM(INFO, "TX Raw OFDM: ALL 0xAA (%zu bytes)", test_size);
    }

    // Use StreamingEncoder's waveform directly for raw (no LDPC) modulation
    streaming_encoder_->setMode(protocol::WaveformMode::OFDM_CHIRP);
    streaming_encoder_->setDataMode(Modulation::DQPSK, CodeRate::R1_4);
    IWaveform* wfm = streaming_encoder_->getWaveform();
    if (!wfm) {
        LOG_MODEM(ERROR, "TX Raw OFDM: Failed to create waveform");
        return {};
    }

    Samples preamble = wfm->generatePreamble();
    Samples modulated = wfm->modulate(test_data);

    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());

    // Scale for audio output
    float max_val = 0.0f;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) s *= scale;
    }

    LOG_MODEM(INFO, "TX Raw OFDM: %zu bytes -> %zu samples",
              test_size, output.size());
    return output;
}

// ============================================================================
// STATUS & DATA ACCESS
// ============================================================================

bool ModemEngine::hasReceivedData() const {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return !rx_data_queue_.empty();
}

std::string ModemEngine::getReceivedText() {
    Bytes data = getReceivedData();
    std::string text(data.begin(), data.end());
    text.erase(std::remove(text.begin(), text.end(), '\0'), text.end());
    return text;
}

Bytes ModemEngine::getReceivedData() {
    std::lock_guard<std::mutex> lock(rx_mutex_);

    if (rx_data_queue_.empty()) {
        return {};
    }

    Bytes data = rx_data_queue_.front();
    rx_data_queue_.pop();
    return data;
}

LoopbackStats ModemEngine::getStats() const {
    LoopbackStats out;
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        out = stats_;
    }
    // LIVE OVERLAY (external review 2026-07-10, finding 3): the cache above is
    // refreshed only by the per-frame callback, which the burst transport
    // suppresses (frames deliver as a unit) — so during a file transfer every
    // getStats() consumer (GUI meters, polled TNC) froze at the handshake-era
    // values while the protocol correctly consumed the decoder's lock-free
    // atomics. Overlay those same live values here so one snapshot serves all
    // consumers: per-frame OFDM broadband (updated inside bursts), the live
    // idle-noise meter, and the current fading index.
    if (streaming_decoder_) {
        if (streaming_decoder_->hasLastOFDMBroadbandSNREstimate()) {
            const float bb = streaming_decoder_->getLastOFDMBroadbandSNREstimate();
            if (std::isfinite(bb)) {
                out.snr_db = bb;
                out.snr_source = SNRSource::OFDM_BROADBAND;
                out.has_ofdm_broadband_snr_db = true;
                out.ofdm_broadband_snr_db = bb;
            }
        }
        const auto idle = streaming_decoder_->getIdleNoiseSNRSnapshot();
        if (idle.valid && std::isfinite(idle.idle_in_band_snr_db)) {
            out.has_idle_in_band_snr_db = true;
            out.idle_in_band_snr_db = idle.idle_in_band_snr_db;
        }
    }
    return out;
}

DecoderStats ModemEngine::getDecoderStats() const {
    if (streaming_decoder_) {
        return streaming_decoder_->getStats();
    }
    return DecoderStats{};
}

bool ModemEngine::isSynced() const {
    if (streaming_decoder_) {
        return streaming_decoder_->isSynced();
    }
    return false;
}

float ModemEngine::getCurrentSNR() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_.snr_db;
}

float ModemEngine::getFadingIndex() const {
    if (streaming_decoder_) {
        return streaming_decoder_->getLastFadingIndex();
    }
    return 0.0f;
}

float ModemEngine::getDopplerCoherenceScore() const {
    return streaming_decoder_ ? streaming_decoder_->getLastDopplerCoherenceScore() : 0.0f;
}

bool ModemEngine::getDopplerCoherenceValid() const {
    return streaming_decoder_ ? streaming_decoder_->getLastDopplerCoherenceValid() : false;
}

bool ModemEngine::rxSignalActive(int64_t within_ms) const {
    return streaming_decoder_ && streaming_decoder_->rxSignalActiveWithin(within_ms);
}

int64_t ModemEngine::lastRxSignalMs() const {
    return streaming_decoder_ ? streaming_decoder_->lastRxSignalMs() : -1000000;
}

bool ModemEngine::hasLastPhysicalSnr() const {
    return streaming_decoder_ && streaming_decoder_->hasLastPhysicalSnr();
}

float ModemEngine::lastPhysicalSnrDb() const {
    return streaming_decoder_ ? streaming_decoder_->lastPhysicalSnrDb() : 0.0f;
}

size_t ModemEngine::physicalSnrStats(float& mean_db, float& spread_db) const {
    if (!streaming_decoder_) { mean_db = 0.0f; spread_db = 0.0f; return 0; }
    return streaming_decoder_->physicalSnrStats(mean_db, spread_db);
}

int64_t ModemEngine::lastRxSubstantiveMs() const {
    return streaming_decoder_ ? streaming_decoder_->lastRxSubstantiveMs() : -1000000;
}

float ModemEngine::getDopplerCoherenceDopplerHz() const {
    return streaming_decoder_ ? streaming_decoder_->getLastMeasuredDopplerHz() : 0.0f;
}

int ModemEngine::getRxLevelVerdict() const {
    return streaming_decoder_ ? streaming_decoder_->getRxLevelVerdict() : 0;
}

uint32_t ModemEngine::getRxLevelVerdictSeq() const {
    return streaming_decoder_ ? streaming_decoder_->getRxLevelVerdictSeq() : 0;
}

bool ModemEngine::hasLastOFDMBroadbandSNR() const {
    return streaming_decoder_ ? streaming_decoder_->hasLastOFDMBroadbandSNREstimate() : false;
}

float ModemEngine::getLastOFDMBroadbandSNR() const {
    return streaming_decoder_ ? streaming_decoder_->getLastOFDMBroadbandSNREstimate() : 0.0f;
}

bool ModemEngine::hasLastEvmSnr() const {
    return streaming_decoder_ ? streaming_decoder_->hasLastEvmSnr() : false;
}

float ModemEngine::getLastEvmSnrDb() const {
    return streaming_decoder_ ? streaming_decoder_->getLastEvmSnrDb() : 0.0f;
}

bool ModemEngine::isFading() const {
    return getFadingIndex() > 0.65f;
}

ChannelQuality ModemEngine::getChannelQuality() const {
    // ChannelQuality not exposed through streaming_decoder_ yet
    // Return default for now
    return ChannelQuality{};
}

void ModemEngine::setKnownCFO(float cfo_hz) {
    if (streaming_decoder_) {
        streaming_decoder_->setKnownCFO(cfo_hz);
    }
}

std::vector<std::complex<float>> ModemEngine::getConstellationSymbols() const {
    if (streaming_decoder_) {
        return streaming_decoder_->getConstellationSymbols();
    }
    return {};
}

Modulation ModemEngine::getConstellationModulation() const {
    if (streaming_decoder_) {
        return streaming_decoder_->getConstellationModulation();
    }
    return Modulation::QPSK;
}

bool ModemEngine::isNarrowbandDetected() const {
    if (streaming_decoder_) {
        return streaming_decoder_->getDetectedBandwidth() == BandwidthMode::NARROW;
    }
    return false;
}

void ModemEngine::setNarrowbandControl(bool narrowband) {
    if (streaming_encoder_) {
        streaming_encoder_->setNarrowbandControl(narrowband);
    }
}

void ModemEngine::reset() {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    std::queue<Bytes> empty;
    std::swap(rx_data_queue_, empty);

    adaptive_.reset();
    use_connected_waveform_once_ = false;

    // Reset StreamingDecoder (primary decoder)
    if (streaming_decoder_) {
        streaming_decoder_->reset();
    }

    {
        std::lock_guard<std::mutex> lock2(stats_mutex_);
        stats_ = LoopbackStats{};
    }
}

void ModemEngine::clearRxBuffer(bool for_tx_echo) {
    // Clear streaming decoder buffer to discard any pending audio
    // Use this before TX to prevent decoding our own transmission (acoustic echo).
    // PRESERVE the slow Doppler-coherence estimator (reset_doppler_coherence=false): this fires
    // every half-duplex turnaround (before each ACK), and the disc needs ~31 per-frame snapshots
    // across many groups to validate — wiping it here kept it permanently invalid on real
    // transfers (task #55). The disc still resets on a true connection/mode reset (ModemEngine::reset).
    if (!streaming_decoder_) return;

    // #67 WARM TURNAROUND (now DEFAULT-ON; opt-out ULTRA_WARM_TURNAROUND_OFF=1): the pre-TX
    // half-duplex echo-clear (for_tx_echo) fires before EVERY ACK during a connected OFDM transfer.
    // The decoder reset() it triggers zeroes the ring timeline (total_fed_) and wipes the warm-sync
    // frame-arrival prediction, so every subsequent burst was COLD-re-acquired (min_search ~2.5 s)
    // instead of warm (~0.2 s) — measured ~2 s/cycle (~27% goodput) on the IONOS rig. The audio side
    // (setRxMuted + stopCapture + AudioEngine::clearRxBuffer) already prevents echo, so on this path
    // the decoder reset is harmful overkill. Skipping it makes the rig behave like OTASim, which never
    // runs this path at all (its sim-TX returns before the echo-clear) and keeps warm-sync alive across
    // turnarounds. Correct by construction; worst case is a stale prediction → cold fallback (== the old
    // behavior), never a stranded frame. Default-ON after rig-proven on BOTH Good (MPG@20: turnaround
    // 2.71→1.54s median, −43%) AND Moderate (MPM@20: turnaround 1.59s, 0 stalls), all CRC-clean. The
    // faithful OTASim gate cannot exercise this path (sim TX skips it), so the rig is the proving ground.
    static const bool kWarmTurnaroundOff = [] {
        const char* e = std::getenv("ULTRA_WARM_TURNAROUND_OFF");
        return e && e[0] == '1';
    }();
    if (for_tx_echo && !kWarmTurnaroundOff && connected_ &&
        protocol::isOFDMMode(waveform_mode_)) {
        return;  // preserve warm-sync state + ring timeline across the turnaround
    }

    streaming_decoder_->reset(/*reset_doppler_coherence=*/false);
}

} // namespace gui
} // namespace ultra
