// StreamingEncoder - Unified TX encoder for all waveform types
//
// Mirrors StreamingDecoder to ensure TX/RX use identical configurations.

#include "streaming_encoder.hpp"
#include "sync/acquisition_mode.hpp"
#include "streaming_control_profile.hpp"
#include "streaming_frame_policy.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "waveform/mc_dpsk_waveform.hpp"
#include "waveform/tone_burst_ack/tone_burst_encoder.hpp"
#include "fec/frame_interleaver.hpp"
#include "fec/burst_interleaver.hpp"
#include "protocol/connection_policy.hpp"
#include "ofdm/genie_tx_capture.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/phy_diagnostics.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;

namespace {

bool isControlFrameBytes(const Bytes& frame_data) {
    if (frame_data.size() < 3) {
        return false;
    }
    auto ft = static_cast<v2::FrameType>(frame_data[2]);
    return v2::isControlFrame(ft);
}

// #72: the handshake-NEGOTIATION frames (CONNECT / CONNECT_ACK / CONNECT_NAK) are
// the ONLY MC-DPSK frames sent while the two peers may be on DIFFERENT constellations
// (the responder has decided the DATA rung but the initiator hasn't adopted it yet).
// They must ride the fixed control profile (DBPSK/1024). NOT DISCONNECT/ACK/MODE_CHANGE
// — those go out AFTER both sides agree on the data mode, so they ride it. (CONNECT is
// effectively already DBPSK at the default, so pinning it is a harmless no-op.)
bool isHandshakeNegotiationFrameBytes(const Bytes& frame_data) {
    if (frame_data.size() < 3) {
        return false;
    }
    auto ft = static_cast<v2::FrameType>(frame_data[2]);
    return ft == v2::FrameType::CONNECT || ft == v2::FrameType::CONNECT_ACK ||
           ft == v2::FrameType::CONNECT_NAK;
}

void markFirstLTSSymbolForBurstGroup(Samples& preamble, size_t symbol_samples) {
    if (preamble.empty() || symbol_samples == 0) {
        return;
    }

    // Full OFDM preambles are [sync][LTS][LTS]; light data preambles are
    // [LTS][LTS]. The burst marker belongs on the first LTS symbol, not on
    // the chirp/STS sync portion of a full anchor.
    const size_t lts_pair_samples = symbol_samples * 2;
    const size_t lts_start = preamble.size() > lts_pair_samples
        ? preamble.size() - lts_pair_samples
        : 0;
    const size_t lts_symbol_end = std::min(preamble.size(), lts_start + symbol_samples);
    for (size_t j = lts_start; j < lts_symbol_end; ++j) {
        preamble[j] = -preamble[j];
    }
}

// Coherent-OFDM PAPR policy (2026-07-02, rig 16QAM diagnosis). The old blanket
// skip ("pilots require a linear transmit waveform") left coherent bursts at
// their natural ~10-12 dB PAPR — and the TX chain PEAK-normalizes each burst
// to tx_drive, so every dB of crest is a dB of lost AVERAGE power on the air.
// Measured at the Mac's RX input (wire capture, MPG@20): the 16QAM data
// segments arrived ~6-7 dB below the burst's own anchor and only single-digit
// dB above the noise floor — below 16QAM's requirement while the meters (LTS/
// anchor-referenced) read healthy. A bounded soft-clip at a CONSERVATIVE
// threshold touches only rare peaks (small EVM cost, shared by pilots and
// data so the LS pilot estimate stays representative) and converts directly
// into arriving data SNR under peak normalization — with the burst PEAK
// unchanged, so no new clipping risk at the radio/IONOS input.
// ULTRA_COHERENT_PAPR_DB: 0 = off (legacy linear TX), [4..12] dB = clip
// threshold; default 9 dB (conservative; sim A/B must show ~no decode cost,
// the rig A/B shows the net win).
inline float coherentPaprThresholdDb() {
    static const float v = [] {
        if (const char* e = std::getenv("ULTRA_COHERENT_PAPR_DB")) {
            const float p = std::strtof(e, nullptr);
            if (p <= 0.0f) return 0.0f;
            if (p >= 4.0f && p <= 12.0f) return p;
        }
        return 0.0f;  // DEFAULT OFF (2026-07-02 sim A/B: EVM cost > benefit for
                      // 16QAM at every tested depth — 9dB target: 181 deint-fails
                      // vs 30 linear, hard FAIL; 12dB: 78 fails, goodput 2210->1320.
                      // The blanket linear-TX rationale stands; knob kept for
                      // future PAPR-reduction implementations w/ pilot protection).
    }();
    return v;
}

bool paprReductionWouldCorruptCoherentPilots(Modulation mod) {
    return ofdm_link_adaptation::isCoherentModulation(mod) &&
           coherentPaprThresholdDb() <= 0.0f;
}

}  // namespace

// ============================================================================
// CONSTRUCTION / DESTRUCTION
// ============================================================================

StreamingEncoder::StreamingEncoder() {

    // Initialize with default OFDM config (NVIS mode)
    ofdm_config_.fft_size = 1024;
    ofdm_config_.num_carriers = 59;
    ofdm_config_.sample_rate = 48000;
    ofdm_config_.center_freq = 1500.0f;
    ofdm_config_.cp_mode = CyclicPrefixMode::LONG;
    ofdm_config_.modulation = Modulation::DQPSK;
    ofdm_config_.code_rate = CodeRate::R1_4;
    ofdm_config_.use_pilots = true;
    ofdm_config_.pilot_spacing = 10;

    // Create default MC-DPSK waveform
    createWaveform();
    updateInterleaver();

    LOG_MODEM(INFO, "StreamingEncoder: Initialized (mode=%s, carriers=%d)",
              protocol::waveformModeToString(mode_), ofdm_config_.num_carriers);
}

StreamingEncoder::~StreamingEncoder() = default;

void StreamingEncoder::setPaprReductionEnabled(bool enable) {
    papr_reduction_enabled_ = enable;
    last_papr_reduction_ = {};
}

void StreamingEncoder::setPaprReductionThresholdDb(float threshold_db) {
    if (!std::isfinite(threshold_db) || !(threshold_db > 0.0f)) {
        threshold_db = phy::kOfdmPaprReductionDefaultThresholdDb;
    }
    papr_reduction_threshold_db_ = threshold_db;
}

void StreamingEncoder::applyPaprReductionIfNeeded(std::vector<float>& samples,
                                                  bool is_ofdm,
                                                  bool is_control_frame,
                                                  const char* label) {
    last_papr_reduction_ = {};
    // ULTRA_COHERENT_PAPR_DB self-enables for coherent mods (2026-07-13): the
    // GUI setPaprReductionEnabled flag is the only enable path, which is why the
    // RIG A/B this feature needs was never run headlessly. Setting the coherent
    // threshold knob IS the intent to enable — the 2026-07-02 "cost>benefit"
    // verdict was measured in SIM, which structurally CANNOT show the benefit
    // (no peak-limited PA → peak-normalization gain is zero by construction);
    // the rig is the only valid A/B. Knob default 0 = off = byte-identical.
    const bool coherent_papr_knob_on =
        ofdm_link_adaptation::isCoherentModulation(modulation_) &&
        coherentPaprThresholdDb() > 0.0f;
    if ((!papr_reduction_enabled_ && !coherent_papr_knob_on) || !is_ofdm ||
        is_control_frame || samples.empty()) {
        return;
    }

    if (paprReductionWouldCorruptCoherentPilots(modulation_)) {
        LOG_MODEM(DEBUG,
                  "[%s] PAPR reduction skipped for %s %s: coherent OFDM "
                  "linear TX requested (ULTRA_COHERENT_PAPR_DB=0)",
                  log_prefix_.c_str(),
                  modulationToString(modulation_),
                  codeRateToString(code_rate_));
        return;
    }

    // Coherent mods clip at their own conservative threshold; differential
    // mods keep the historical default.
    const float threshold_db =
        ofdm_link_adaptation::isCoherentModulation(modulation_)
            ? coherentPaprThresholdDb()
            : papr_reduction_threshold_db_;
    last_papr_reduction_ = phy::applyPaprReduction(
        samples, threshold_db, true);
    if (last_papr_reduction_.applied) {
        LOG_MODEM(INFO,
                  "[%s] PAPR reduction %s threshold=%.2f dB pre=%.2f dB "
                  "post=%.2f dB reduction=%.2f dB clips=%zu rms_delta=%.2f dB",
                  log_prefix_.c_str(),
                  label ? label : "-",
                  last_papr_reduction_.threshold_db,
                  last_papr_reduction_.pre_papr_db,
                  last_papr_reduction_.post_papr_db,
                  last_papr_reduction_.pre_papr_db -
                      last_papr_reduction_.post_papr_db,
                  last_papr_reduction_.clipped_samples,
                  last_papr_reduction_.in_band_rms_delta_db);
    }
}

void StreamingEncoder::setBurstInterleaveGroupSize(int size) {
    burst_group_size_ = ofdm_link_adaptation::sanitizeBurstGroupSize(size);
}

// R4: the adaptive short-chirp re-anchor was removed (superseded by warm-handoff, now default).
std::vector<float> StreamingEncoder::settlingLeadInFill(size_t samples) {
    if (!waveform_ || samples == 0) {
        return {};
    }
    const size_t symbol_samples =
        static_cast<size_t>(std::max(1, waveform_->getSamplesPerSymbol()));
    const size_t count = (samples + symbol_samples - 1) / symbol_samples;
    Samples symbols = waveform_->generateSettlingSymbols(count);
    if (symbols.size() < samples) {
        return {};
    }
    std::vector<float> fill(symbols.end() - static_cast<std::ptrdiff_t>(samples), symbols.end());
    // Raised-cosine ramp-up (5 ms): no key-click splatter at the fill start; the ALC sees a
    // smooth rise to the data level.
    const size_t ramp = std::min(fill.size(), static_cast<size_t>(48000 * 5 / 1000));
    for (size_t i = 0; i < ramp; ++i) {
        const double w = 0.5 - 0.5 * std::cos(M_PI * static_cast<double>(i) / static_cast<double>(ramp));
        fill[i] = static_cast<float>(fill[i] * w);
    }
    return fill;
}

Samples StreamingEncoder::connectedDataPreambleForFrame() {
    if (!waveform_) {
        return {};
    }
    if (!waveform_->supportsDataPreamble()) {
        return waveform_->generatePreamble();
    }
    return waveform_->generateDataPreamble();
}

void StreamingEncoder::setCarrierMask(uint64_t active_mask) {
    carrier_mask_ = active_mask;
    if (waveform_) {
        waveform_->setCarrierMask(active_mask);
    }
}

void StreamingEncoder::setCarrierLdpcInterleaver(bool enable) {
    use_carrier_ldpc_interleaver_ = enable;
    if (waveform_) {
        waveform_->setCarrierLdpcInterleaverEnabled(enable);
    }
}

// ============================================================================
// MODE CONTROL
// ============================================================================

void StreamingEncoder::setMode(protocol::WaveformMode mode) {
    if (mode_ == mode) return;

    mode_ = mode;
    createWaveform();
    updateInterleaver();

    LOG_MODEM(INFO, "[%s] Mode changed to %s",
              log_prefix_.c_str(), protocol::waveformModeToString(mode));
}

void StreamingEncoder::setDataMode(Modulation mod, CodeRate rate) {
    if (modulation_ == mod && code_rate_ == rate) return;

    modulation_ = mod;
    code_rate_ = rate;
    ofdm_config_.modulation = mod;
    ofdm_config_.code_rate = rate;

    // Update waveform configuration
    if (waveform_) {
        waveform_->configure(mod, rate);
        // Sync pilot_spacing from waveform (coherent modes use denser pilots)
        int spacing = waveform_->getPilotSpacing();
        if (spacing > 0) ofdm_config_.pilot_spacing = spacing;
    }

    // Update interleaver (bits_per_symbol may change with modulation)
    updateInterleaver();

    LOG_MODEM(INFO, "[%s] Data mode: %s R%s",
              log_prefix_.c_str(),
              mod == Modulation::DQPSK ? "DQPSK" :
              mod == Modulation::D8PSK ? "D8PSK" : "other",
              rate == CodeRate::R1_4 ? "1/4" :
              rate == CodeRate::R1_2 ? "1/2" :
              rate == CodeRate::R2_3 ? "2/3" : "other");
}

void StreamingEncoder::setOFDMConfig(const ModemConfig& config) {
    ofdm_config_ = config;
    modulation_ = config.modulation;
    code_rate_ = config.code_rate;

    // Recreate waveform with new config
    createWaveform();
    updateInterleaver();

    LOG_MODEM(INFO, "[%s] OFDM config updated: FFT=%d, carriers=%d, pilots=%s spacing=%d",
              log_prefix_.c_str(), config.fft_size, config.num_carriers,
              config.use_pilots ? "yes" : "no", config.pilot_spacing);
}

void StreamingEncoder::setMCDPSKCarriers(int num_carriers) {
    if (mc_dpsk_carriers_ == num_carriers) return;

    mc_dpsk_carriers_ = num_carriers;
    mc_dpsk_config_.num_carriers = num_carriers;

    // Recreate waveform if currently in MC-DPSK mode
    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        createWaveform();
    }

    // Always update control waveform
    control_waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);

    LOG_MODEM(INFO, "[%s] MC-DPSK carriers: %d", log_prefix_.c_str(), num_carriers);
}

void StreamingEncoder::setMCDPSKConfig(const MultiCarrierDPSKConfig& config) {
    const bool changed =
        mc_dpsk_config_.num_carriers != config.num_carriers ||
        mc_dpsk_config_.samples_per_symbol != config.samples_per_symbol ||
        mc_dpsk_config_.bits_per_symbol != config.bits_per_symbol ||
        mc_dpsk_config_.freq_low != config.freq_low ||
        mc_dpsk_config_.freq_high != config.freq_high ||
        mc_dpsk_config_.chirp_f_start != config.chirp_f_start ||
        mc_dpsk_config_.chirp_f_end != config.chirp_f_end ||
        mc_dpsk_config_.chirp_duration_ms != config.chirp_duration_ms ||
        mc_dpsk_config_.use_dual_chirp != config.use_dual_chirp;
    if (!changed) return;

    mc_dpsk_config_ = config;
    mc_dpsk_carriers_ = config.num_carriers;

    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        createWaveform();
    }
    control_waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);

    LOG_MODEM(INFO, "[%s] MC-DPSK config: carriers=%d sps=%d bits/sym=%d raw=%.1f bps",
              log_prefix_.c_str(), mc_dpsk_config_.num_carriers,
              mc_dpsk_config_.samples_per_symbol, mc_dpsk_config_.bits_per_symbol,
              mc_dpsk_config_.getRawBitRate());
}

void StreamingEncoder::setNarrowbandControl(bool narrowband) {
    narrowband_control_ = narrowband;
    if (narrowband) {
        control_waveform_ = WaveformFactory::createNarrowbandMCDPSK();
        // When in MC-DPSK mode (handshake), also switch main waveform to narrowband
        // so CONNECT/CONNECT_ACK frames use narrowband chirp + carriers
        if (mode_ == protocol::WaveformMode::MC_DPSK) {
            waveform_ = WaveformFactory::createNarrowbandMCDPSK();
        }
        LOG_MODEM(INFO, "[%s] Control waveform: narrowband MC-DPSK (500 Hz)", log_prefix_.c_str());
    } else {
        control_waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);
        if (mode_ == protocol::WaveformMode::MC_DPSK) {
            waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);
        }
        LOG_MODEM(INFO, "[%s] Control waveform: wideband MC-DPSK (%d carriers)", log_prefix_.c_str(), mc_dpsk_carriers_);
    }
}

// ============================================================================
// ENCODING
// ============================================================================

std::vector<float> StreamingEncoder::encodeFrame(const Bytes& frame_data, bool prefer_short_anchor,
                                                 bool light_preamble) {
    if (!waveform_) {
        LOG_MODEM(ERROR, "[%s] No waveform!", log_prefix_.c_str());
        return {};
    }

    bool is_ofdm = protocol::isOFDMMode(mode_);
    // #72: MC-DPSK control/handshake frames also ride a fixed control profile
    // (DBPSK R1/4) so CONNECT/CONNECT_ACK are peer-decodable regardless of the
    // negotiated DATA constellation. Baud is standardized at 1024, so configure()
    // only swaps the constellation; the CONNECT FEC is already R1/4 (encodeFrameBytes).
    const bool is_mc_dpsk = (mode_ == protocol::WaveformMode::MC_DPSK);
    bool use_control_profile =
        (is_ofdm && isControlFrameBytes(frame_data)) ||
        (is_mc_dpsk && isHandshakeNegotiationFrameBytes(frame_data));
    const auto control_profile =
        is_mc_dpsk ? streaming_control_profile::profileForMCDPSK()
                   : streaming_control_profile::profileForDataMode(modulation_);
    Modulation tx_mod = use_control_profile ? control_profile.modulation : modulation_;
    CodeRate tx_rate = use_control_profile ? control_profile.rate : code_rate_;

    if (force_full_preamble_once_ && is_ofdm) {
        force_full_preamble_once_ = false;
        LOG_MODEM(INFO, "[%s] Full preamble OFDM timing anchor emitted",
                  log_prefix_.c_str());
    }

    if (use_control_profile) {
        LOG_MODEM(INFO, "[%s] control profile TX: %s %s (data=%s %s)",
                  log_prefix_.c_str(), modulationToString(tx_mod),
                  codeRateToString(tx_rate), modulationToString(modulation_),
                  codeRateToString(code_rate_));
        waveform_->configure(tx_mod, tx_rate);
    }

    // Encode frame bytes (LDPC + interleaving)
    Bytes encoded = encodeFrameBytes(frame_data);

    // Generate preamble (full dual chirp, or the Phase 2a short single-chirp warm re-anchor, or the
    // #69 light LTS-only descriptor for ULTRA_ANCHOR_SKIP_K — no chirp; warm-acquired by the RX).
    Samples preamble = light_preamble       ? connectedDataPreambleForFrame()
                       : prefer_short_anchor ? waveform_->generateShortAnchorPreamble()
                                             : waveform_->generatePreamble();

    // DIAGNOSTIC (ULTRA_GENIE_DATA_AIDED, default off): tell the genie capture where
    // this frame's data symbols land in the transmitted stream, so the receiver can
    // address them by absolute sample position. genie_stream_base_ is the offset of
    // THIS call's output within the stream the caller is assembling (0 for a
    // standalone frame; set by encodeBurstLight for the in-burst descriptor).
    if (ultra::genie::txCapture().enabled) {
        ultra::genie::txCapture().frame_data_base =
            genie_stream_base_ + static_cast<long long>(preamble.size());
    }

    // Modulate
    Samples modulated = waveform_->modulate(encoded);

    // Combine preamble + data
    std::vector<float> result;
    result.reserve(preamble.size() + modulated.size());
    result.insert(result.end(), preamble.begin(), preamble.end());
    result.insert(result.end(), modulated.begin(), modulated.end());

    if (use_control_profile && (modulation_ != tx_mod || code_rate_ != tx_rate)) {
        waveform_->configure(modulation_, code_rate_);
    }

    applyPaprReductionIfNeeded(result, is_ofdm, use_control_profile, "frame");

    LOG_MODEM(INFO, "[%s] Encoded frame: %zu bytes -> %zu coded -> %zu samples",
              log_prefix_.c_str(), frame_data.size(), encoded.size(), result.size());

    return result;
}

std::vector<float> StreamingEncoder::encodeFrameLight(const Bytes& frame_data) {
    if (force_full_preamble_once_) {
        force_full_preamble_once_ = false;
        LOG_MODEM(INFO, "[%s] Full preamble forced for OFDM timing anchor",
                  log_prefix_.c_str());
        return encodeFrame(frame_data);
    }

    if (!waveform_) {
        LOG_MODEM(ERROR, "[%s] No waveform!", log_prefix_.c_str());
        return {};
    }

    bool is_ofdm = protocol::isOFDMMode(mode_);
    const bool is_mc_dpsk = (mode_ == protocol::WaveformMode::MC_DPSK);  // #72
    bool use_control_profile =
        (is_ofdm && isControlFrameBytes(frame_data)) ||
        (is_mc_dpsk && isHandshakeNegotiationFrameBytes(frame_data));
    const auto control_profile =
        is_mc_dpsk ? streaming_control_profile::profileForMCDPSK()
                   : streaming_control_profile::profileForDataMode(modulation_);
    Modulation tx_mod = use_control_profile ? control_profile.modulation : modulation_;
    CodeRate tx_rate = use_control_profile ? control_profile.rate : code_rate_;

    if (use_control_profile) {
        LOG_MODEM(INFO, "[%s] control profile TX(light): %s %s (data=%s %s)",
                  log_prefix_.c_str(), modulationToString(tx_mod),
                  codeRateToString(tx_rate), modulationToString(modulation_),
                  codeRateToString(code_rate_));
        waveform_->configure(tx_mod, tx_rate);
    }

    // Encode frame bytes
    Bytes encoded = encodeFrameBytes(frame_data);

    Samples preamble = connectedDataPreambleForFrame();

    // DIAGNOSTIC (ULTRA_GENIE_DATA_AIDED, default off) — see encodeFrame().
    if (ultra::genie::txCapture().enabled) {
        ultra::genie::txCapture().frame_data_base =
            genie_stream_base_ + static_cast<long long>(preamble.size());
    }

    // Modulate
    Samples modulated = waveform_->modulate(encoded);

    // Combine
    std::vector<float> result;
    result.reserve(preamble.size() + modulated.size());
    result.insert(result.end(), preamble.begin(), preamble.end());
    result.insert(result.end(), modulated.begin(), modulated.end());

    if (use_control_profile && (modulation_ != tx_mod || code_rate_ != tx_rate)) {
        waveform_->configure(modulation_, code_rate_);
    }

    applyPaprReductionIfNeeded(result, is_ofdm, use_control_profile, "frame-light");

    LOG_MODEM(DEBUG, "[%s] Encoded frame (light): %zu bytes -> %zu samples",
              log_prefix_.c_str(), frame_data.size(), result.size());

    return result;
}

std::vector<float> StreamingEncoder::encodeBurstLight(
    const std::vector<Bytes>& frame_data_list,
    BurstAnchorOptions anchor_options) {
    if (frame_data_list.empty()) return {};
    if (!waveform_) {
        LOG_MODEM(ERROR, "[%s] No waveform!", log_prefix_.c_str());
        return {};
    }

    last_burst_starts_chirpless_ = false;
    const bool current_group_full_anchor =
        anchor_options.force_full_group_start;
    const bool current_group_keep_skip_streak =
        current_group_full_anchor && anchor_options.keep_skip_streak;
    const bool current_descriptor_full_anchor =
        current_group_full_anchor || anchor_options.force_full_descriptor;

    // The ARQ owns logical frames; physical grouping belongs to this encoder call.
    // Sanitize a private copy on every emission so a seq regrouped by selective
    // retry cannot retain stale tail status. Only a multi-frame, independently
    // decodable OFDM burst gets the zero-airtime end marker in Phase 1.
    const bool stamp_physical_end =
        streaming_frame_policy::shouldStampPhysicalBurstEnd(
            frame_data_list.size(), mode_, waveform_->supportsDataPreamble(),
            use_burst_interleave_);
    const std::vector<Bytes> wire_frame_data_list =
        streaming_frame_policy::preparePhysicalBurstFrames(
            frame_data_list, stamp_physical_end);

    // A one-frame connected OFDM DATA turn has no following in-burst frame to
    // refresh timing if the receiver's warm LTS state is stale. Treat it as a
    // new physical turn and emit a full chirp+LTS anchor.
    if (frame_data_list.size() == 1) {
        if (protocol::isOFDMMode(mode_) && waveform_->supportsDataPreamble()) {
            if (force_full_preamble_once_) {
                LOG_MODEM(INFO, "[%s] Full preamble forced for single OFDM burst frame timing anchor",
                          log_prefix_.c_str());
            } else {
                LOG_MODEM(INFO, "[%s] Full preamble emitted for single OFDM burst frame timing anchor",
                          log_prefix_.c_str());
            }
            force_full_preamble_once_ = false;
            return encodeFrame(wire_frame_data_list[0]);
        }
        return encodeFrameLight(wire_frame_data_list[0]);
    }

    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        force_full_preamble_once_ = false;
        size_t total_cw = 0;
        std::vector<Bytes> encoded_frames;
        encoded_frames.reserve(wire_frame_data_list.size());
        for (const auto& fd : wire_frame_data_list) {
            Bytes encoded = encodeFrameBytes(fd);
            if (encoded.empty()) {
                LOG_MODEM(WARN, "[%s] MC-DPSK burst: dropping empty encoded frame",
                          log_prefix_.c_str());
                return {};
            }
            total_cw += encoded.size() / v2::LDPC_CODEWORD_BYTES;
            encoded_frames.push_back(std::move(encoded));
        }

        Samples preamble = waveform_->generatePreamble();
        std::vector<float> result;
        result.reserve(preamble.size());
        result.insert(result.end(), preamble.begin(), preamble.end());
        for (const auto& encoded : encoded_frames) {
            Samples modulated = waveform_->modulate(encoded);
            result.insert(result.end(), modulated.begin(), modulated.end());
        }

        LOG_MODEM(INFO, "[%s] MC-DPSK continuous burst: %zu frames, %zu CWs -> %zu samples",
                  log_prefix_.c_str(), frame_data_list.size(), total_cw, result.size());
        return result;
    }

    // Phase 1: LDPC encode all frames
    std::vector<Bytes> encoded_frames;
    for (const auto& fd : wire_frame_data_list) {
        encoded_frames.push_back(encodeFrameBytes(fd));
    }

    // Phase 2: Group into N-frame subgroups, burst-interleave each group
    // Track which groups are burst-interleaved (for LTS marker)
    const int BURST_GROUP_SIZE = std::max(2, burst_group_size_);
    std::vector<bool> frame_is_group_start(encoded_frames.size(), false);
    // Frames that belong to a formed burst-interleaved group (start + members).
    // The RX accumulates these at a FIXED stride (burst_min_block_ =
    // getMinSamplesForCWCount = [LTS + data]), so every group frame must be
    // exactly that — no per-frame short re-anchor chirp prefix, which would push
    // each frame past the stride and progressively misalign the FFT window
    // (timing offset ~360 samples -> guard-bin leakage -> noise_var blow-up ->
    // 0/8 deinterleave). See §14.25.
    std::vector<bool> frame_in_burst_group(encoded_frames.size(), false);
    size_t interleaved_groups = 0;  // burst GROUPS formed (whether or not byte-permuted)

    // 2026-05-29: form the burst GROUP structure (descriptor + group-start anchor +
    // group-ACK) for the file burst REGARDLESS of interleave — cross-frame interleave
    // is now a per-burst FLAG carried in the BURST_HEADER, not a precondition for the
    // transport. The actual byte permutation is applied only when use_burst_interleave_
    // (ON for Moderate/Poor where the faster/selective fade gives the N-frame group real
    // diversity; OFF for Good/AWGN where N≈1×Tc buys ~nothing and per-frame SACK is the
    // lever). The RX reads the flag from the descriptor (setBurstInterleave(
    // bi.burst_interleave)) and deinterleaves only when set, so TX/RX stay matched
    // per burst — and flipping the flag is how we'd activate interleave on Moderate.
    {
        size_t full_groups = encoded_frames.size() / BURST_GROUP_SIZE;
        for (size_t g = 0; g < full_groups; g++) {
            size_t base = g * BURST_GROUP_SIZE;

            if (use_burst_interleave_) {
                // Burst-interleave the coded bytes across the group. At z=81 (N=1944)
                // each codeword is 243 bytes instead of 81.
                std::vector<Bytes> group(encoded_frames.begin() + base,
                                         encoded_frames.begin() + base + BURST_GROUP_SIZE);
                const int ldpc_z_for_burst = static_cast<int>(ldpc_lifting_z_);
                const int bytes_per_cw = (ldpc_z_for_burst == 81) ? 243 : 81;
                auto interleaved = fec::BurstInterleaver::interleave(
                    group, fixed_frame_codewords_, bytes_per_cw);
                for (int i = 0; i < BURST_GROUP_SIZE; i++) {
                    encoded_frames[base + i] = interleaved[i];
                }
                LOG_MODEM(INFO, "[%s] Burst interleaved group %zu: frames %zu-%zu",
                          log_prefix_.c_str(), g, base, base + BURST_GROUP_SIZE - 1);
            }

            // Group MARKING — always, whether or not the bytes were permuted, so the
            // descriptor + group-start anchor + group-ACK fire either way.
            for (int i = 0; i < BURST_GROUP_SIZE; i++) {
                frame_in_burst_group[base + i] = true;
            }
            frame_is_group_start[base] = true;
            interleaved_groups++;
        }
    }

    if (ultra::phyDiagnosticsEnabled()) {
        std::ostringstream oss;
        oss << "burst_tx use_bi=" << (use_burst_interleave_ ? 1 : 0)
            << " group_size=" << BURST_GROUP_SIZE
            << " in_frames=" << encoded_frames.size()
            << " groups_formed=" << interleaved_groups;
        ultra::phyDiagLine(oss.str());
    }

    // Phase 3: Modulate with preambles
    std::vector<float> result;

    // current_group_full_anchor is the per-call wire truth for the
    // immediately-following DATA group. The separate
    // force_full_preamble_once_ belongs to the descriptor/control frame and is
    // consumed by encodeFrame below; it must not turn an ordinary first-attempt
    // DATA marker into a second redundant full preamble.
    // Self-describing burst head (§14.17/§14.19): for an interleaved OFDM group,
    // emit a full-anchor 1-CW BURST_HEADER descriptor that DECLARES the group's
    // decode params (group size, cw/frame, mod/rate, interleave flags). The RX
    // configures itself from this declaration, fixing the cross-station 0/8 where
    // the receiver's local config did not match the sender's. The descriptor is a
    // control frame (DQPSK control profile via encodeFrame), so it carries its own
    // full chirp+LTS anchor and seeds the warm timing the group-start marker rides.
    if (emit_burst_descriptor_ && interleaved_groups > 0 &&
        protocol::isOFDMMode(mode_) && waveform_->supportsDataPreamble()) {
        // #69 PERIODIC FULL ANCHOR + WARM-SKIP (ULTRA_ANCHOR_SKIP_K, default 1 = full chirp every
        // group; K=2 explicitly opts into every-other-group reactive skip). The 2026-08-01 live
        // integrity run invalidated the former default-on claim: 2/3 LIGHT descriptors were lost
        // versus 1/30 chirp-bearing descriptors, producing 17-70 s recovery episodes.
        // chirp every group). Research (project_chirp_anchor_skip_not_shrink): the chirp is
        // near-optimal (TB=1200 = fade margin; shrinking it craters), so SKIP it on benign groups
        // instead of shrinking it. Emit the full chirp only when ordinal % K == 0; skipped groups
        // get a LIGHT (LTS-only, no chirp) descriptor. warm_descriptor excludes the session-first
        // burst + resends (cold-start / lost-tail keep the full chirp). The full chirp every K
        // groups is the robustness BACKSTOP + drift reset. Enabled by the no-op turnaround fix
        // (58eed53) which keeps the RX warm-sync state alive across the turnaround.
        //
        // WIRE FLAG (refinement after K=3 crater): the descriptor ANNOUNCES the next group's anchor
        // type (BURST_FLAG_NEXT_LIGHT_ANCHOR) so the RX full-searches chirp groups + light-searches
        // skip groups IMMEDIATELY — no grinding through ~12 light rejects (the ~30 s/resend stall
        // that crawled K=3). Resends are reactive (can't be announced); the RX's K-gated fast §16.4
        // escalation catches those. Computed BEFORE makeBurstHeader so the flag lands in the payload.
        const bool warm_descriptor =
            !(force_full_preamble_once_ || current_descriptor_full_anchor);
        static const int kAnchorSkipK = [] {
            const char* e = std::getenv("ULTRA_ANCHOR_SKIP_K");
            if (e && *e) return std::max(1, std::atoi(e));
            // Structure acquisition (ULTRA_BURST_ACQ=structure): the receiver no longer needs
            // the chirp to find a warm descriptor, so every warm descriptor goes chirp-less
            // (K effectively infinite; the full chirp stays for cold-start / resends / the
            // reactive gate below).
            return sync::structureAcquisitionEnabled() ? (1 << 30) : 1;  // reliability default; =2 opt-in
        }();
        // REACTIVE GATE (2026-06-20, radio-agnostic) — the gate that lets the skip default-on safely.
        // The IONOS rig DISPROVED a PREDICTED coherence label: a Moderate channel read clean-Good for
        // a whole ~60 s transfer (coherence-area +0.20, raw acf lags 1-5 all +0.2-0.3), because a
        // 60 s observation is too few fade cycles to pin Doppler and the channel is non-stationary.
        // (docs/SCALE_INVARIANT_COHERENCE_DISC + project_disc_radio_agnostic_fuzzy_boundary.) So gate
        // the skip on OBSERVED delivery, not a prediction: a resend / cold-start / §16.4 escalation
        // forces a full chirp (warm_descriptor==false) and RESETS the clean streak; only after
        // kReactiveCleanStreak consecutive clean (warm-delivered) groups do we begin skipping. The
        // half-duplex ACK turnaround means warm_descriptor==false reliably reflects the PRIOR group's
        // outcome (the connection sets force_full on its resend) — so this is delivery-driven, not
        // send-optimistic. Radio-agnostic by construction (no channel model, no calibration): a
        // fading channel that keeps triggering resends stays full-chirp; a genuinely clean run earns
        // the skip and reverts the instant it craters. The streak-reset IS the cooldown.
        static const uint32_t kReactiveCleanStreak = [] {
            const char* e = std::getenv("ULTRA_ANCHOR_SKIP_CLEAN_STREAK");
            if (e && *e) return static_cast<uint32_t>(std::max(0, std::atoi(e)));
            // Under structure acquisition the descriptor is always the same control-profile
            // template, so a post-switch group is no harder to acquire: one clean group is
            // enough evidence. The legacy LTS-only light sync needed 4.
            return sync::structureAcquisitionEnabled() ? 1u : 4u;
        }();
        // THE RECOOL DOES DOUBLE DUTY — do not "fix" it again (2026-07-26, measured
        // HARMFUL; CHANGELOG). ULTRA_ANCHOR_SKIP_KEEP_STREAK_ON_SWITCH stays default OFF
        // PERMANENTLY.
        //
        // The obvious reading is that this streak is DELIVERY evidence, so a resend must
        // recool it while a descriptor mode/rate SWITCH — a mere configuration event —
        // should not. That reading is true as far as it goes, and acting on it made
        // everything worse: rig A/B (knob on the sender) measured chirp-skip rate FALLING
        // 28.9% -> 7.7%, mean anchor prefix RISING 1.221 -> 1.733 s, craters 1.0 -> 2.0
        // and resends 0.0 -> 1.0 per run.
        //
        // Because the recool is ALSO a POST-SWITCH ACQUISITION GUARD. Preserving the
        // streak lets the skip fire on the very NEXT burst after a geometry change
        // (measured distance-to-first-skip 1 burst, vs 4-5 with the recool) — the riskiest
        // possible moment to omit a chirp, since the receiver must re-acquire on a new
        // constellation/rate. Those groups crater, the crater forces a resend, and the
        // resend recools the streak anyway: strictly more craters AND less skipping.
        //
        // Any future attempt at this lever MUST add an explicit post-switch skip holdoff.
        static const bool kKeepStreakOnSwitch = [] {
            const char* e = std::getenv("ULTRA_ANCHOR_SKIP_KEEP_STREAK_ON_SWITCH");
            return e && e[0] == '1' && e[1] == '\0';
        }();
        const bool keep_streak =
            kKeepStreakOnSwitch && current_group_keep_skip_streak;
        if (!warm_descriptor && !keep_streak) anchor_skip_clean_streak_ = 0;
        const bool reactive_skip_enabled =
            kAnchorSkipK > 1 && anchor_skip_clean_streak_ >= kReactiveCleanStreak;
        const uint32_t anchor_ordinal = burst_anchor_ordinal_++;
        const bool skip_chirp_descriptor =
            reactive_skip_enabled && warm_descriptor &&
            (static_cast<int>(anchor_ordinal % static_cast<uint32_t>(kAnchorSkipK)) != 0);
        last_burst_starts_chirpless_ = skip_chirp_descriptor;  // the descriptor leads the burst
        // Credit the prior clean delivery represented by warm_descriptor before deciding what this
        // descriptor announces for the NEXT group.  The current group's own anchor decision above
        // intentionally uses the pre-credit streak: it may only go LIGHT when the PREVIOUS descriptor
        // announced that fact.  Live IONOS caught the old boundary contradiction at ordinal 12 -> 13:
        // ordinal 12 announced FULL from the pre-credit streak, the credit opened the gate, and ordinal
        // 13 emitted LIGHT; the receiver correctly waited for a chirp and lost the whole group.
        if (warm_descriptor) ++anchor_skip_clean_streak_;  // count the just-observed clean prior group
        const bool next_group_light =
            streaming_frame_policy::shouldAnnounceNextLightAnchor(
                anchor_ordinal, kAnchorSkipK, anchor_skip_clean_streak_, kReactiveCleanStreak);

        // per-burst flags: interleave applies to THIS group; anchor bits
        // describe the immediately-following DATA group and the next descriptor.
        const uint8_t flags = streaming_frame_policy::burstDescriptorFlags(
            use_burst_interleave_, use_carrier_ldpc_interleaver_,
            next_group_light, current_group_full_anchor);
        auto descriptor = protocol::v2::ControlFrame::makeBurstHeader(
            burst_descriptor_src_, burst_descriptor_dst_, /*seq=*/burst_group_seq_,
            static_cast<uint8_t>(BURST_GROUP_SIZE),
            static_cast<uint8_t>(fixed_frame_codewords_),
            modulation_, code_rate_, flags,
            /*lifting_z=*/ldpc_lifting_z_);
        Bytes descriptor_bytes = descriptor.serialize();
        // Short DUAL chirp anchor (default-off shortAnchorEnabled = ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS).
        // On a chirp-bearing group the warm descriptor can SHORTEN the chirp (250 ms dual vs 500 ms);
        // the RX auto-falls-back to the short detector on a full-detector miss (ofdm_chirp_waveform.cpp:478),
        // so no wire flag is needed. Mutually exclusive with the #69 light skip.
        //   GATING: the original Phase-2a gate was R3/4-as-a-clean-proxy. #62 REACTIVE SHORT CHIRP
        //   (ULTRA_REACTIVE_SHORT_CHIRP=1) replaces that with the SAME delivery-driven clean-streak the
        //   skip uses (reactive_skip_enabled): short chirp on a proven-clean run, full chirp the instant
        //   a resend/crater resets the streak. So a clean Good run emits skip-light + short-chirp
        //   alternating (max airtime); a fady run reverts to full chirp every group. Radio-agnostic.
        static const bool kReactiveShortChirp = [] {
            const char* e = std::getenv("ULTRA_REACTIVE_SHORT_CHIRP");
            return e && e[0] != '\0' && !(e[0] == '0' && e[1] == '\0');
        }();
        const bool short_anchor_gate =
            kReactiveShortChirp
                ? reactive_skip_enabled
                : protocol::connection_policy::shouldUseWarmShortAnchorDescriptor(
                      waveform_->getMode(), modulation_, code_rate_);
        const bool descriptor_short_anchor =
            warm_descriptor && waveform_ && waveform_->shortAnchorEnabled() && short_anchor_gate;
        // DIAGNOSTIC (ULTRA_GENIE_DATA_AIDED, default off): the descriptor is emitted
        // INTO result, so its data symbols sit at result.size() + its own preamble.
        genie_stream_base_ = static_cast<long long>(result.size());
        std::vector<float> descriptor_samples =
            encodeFrame(descriptor_bytes, descriptor_short_anchor && !skip_chirp_descriptor,
                        /*light_preamble=*/skip_chirp_descriptor);
        genie_stream_base_ = 0;
        if (kAnchorSkipK > 1 || kReactiveShortChirp) {
            const char* anchor_kind = skip_chirp_descriptor ? "LIGHT(no-chirp)"
                                    : (descriptor_short_anchor ? "SHORT-chirp" : "FULL-chirp");
            LOG_MODEM(INFO, "[%s] #69 anchor: ordinal=%u K=%d streak=%u/%u reactive=%s shortchirp=%d -> %s (announce next=%s)",
                      log_prefix_.c_str(), anchor_ordinal, kAnchorSkipK,
                      anchor_skip_clean_streak_, kReactiveCleanStreak,
                      reactive_skip_enabled ? "ON" : "OFF(full-chirp)",
                      kReactiveShortChirp ? 1 : 0, anchor_kind,
                      next_group_light ? "LIGHT" : "FULL");
        }
        if (!descriptor_samples.empty()) {
            result.insert(result.end(), descriptor_samples.begin(), descriptor_samples.end());
            LOG_MODEM(INFO,
                      "[%s] TX Burst descriptor: group=%d cw/frame=%d %s %s z=%u flags=0x%02x "
                      "(%zu samples ahead of group)",
                      log_prefix_.c_str(), BURST_GROUP_SIZE, fixed_frame_codewords_,
                      modulationToString(modulation_), codeRateToString(code_rate_),
                      static_cast<unsigned>(ldpc_lifting_z_),
                      flags, descriptor_samples.size());
        } else {
            LOG_MODEM(WARN, "[%s] TX Burst descriptor: encodeFrame returned empty (skipping)",
                      log_prefix_.c_str());
        }
    }

    // §16.4: the group-start full-chirp anchor fires for a session's first burst
    // (force_full_preamble_once_) OR this exact request's resend/mode-switch
    // anchor. Either forces the group-start preamble below to the full chirp+LTS
    // instead of warm light LTS.
    const bool force_first_full_preamble =
        force_full_preamble_once_ || current_group_full_anchor;
    force_full_preamble_once_ = false;

    for (size_t i = 0; i < encoded_frames.size(); i++) {
        // Generate preamble (LTS training symbols)
        Samples preamble;
        if (frame_is_group_start[i] && protocol::isOFDMMode(mode_) &&
            waveform_->supportsDataPreamble()) {
            // 2026-05-28: FULL chirp+LTS preamble at group-start.
            //
            // Earlier we tried two lighter alternatives, neither worked:
            //   - pure light LTS: stalled Group 1+ because bravo's warm-sync
            //     went DEGRADED across the half-duplex BURST_HEADER → data
            //     gap and the data sync corr stayed <0.52 (full-anchor
            //     threshold), so bravo never re-acquired sync.
            //   - 500 ms short re-anchor (chirp prefix + LTS): broke frame-
            //     stride timing on group members — bravo's LTS phase slope
            //     drifted 47°→126° per carrier over 4 frames, the demod
            //     produced ~50% zero LLRs, and bravo died silently in the
            //     demod code on frame 5/6.
            //
            // Full preamble (the legacy waveform_->generatePreamble() that
            // every CONTROL frame uses) gives bravo a deterministic chirp+LTS
            // anchor per group. Costs ~1.4 s airtime per group; we accept that
            // for reliability and will reclaim the overhead later (likely via
            // a proper FSK ACK channel + tightened warm-sync hand-off so the
            // group-start can shrink back to light LTS).
            //
            // §16.8 step 2 (ULTRA_S16_WARM_HANDOFF): the design hypothesis
            // is that the historical "pure light LTS" failure was caused by
            // bravo throwing away the BURST_HEADER's warm-sync state in
            // resetFrameArrivalTrackingLocked. With the matching bravo-side
            // change in streaming_ofdm_decode.cpp (skip the reset, keep
            // expect_full_ofdm_anchor_=false), the BURST_HEADER itself
            // becomes the per-group anchor — no need for a second full
            // chirp+LTS right after it. Saves ~1.4 s per group. Now UNCONDITIONAL
            // (promoted past ULTRA_S16_WARM_HANDOFF, 2026-05-31; ~+16% goodput on the
            // §17.1 baseline). Falls back to full anchor on the first group of a
            // session (force_first_full_preamble).
            if (!force_first_full_preamble) {
                preamble = connectedDataPreambleForFrame();
                LOG_MODEM(INFO,
                          "[%s] s16-warm-handoff: light LTS preamble for burst group-start "
                          "(skipping full chirp+LTS; BURST_HEADER anchor still emitted)",
                          log_prefix_.c_str());
            } else {
                preamble = waveform_->generatePreamble();
                LOG_MODEM(INFO,
                          "[%s] Full chirp+LTS preamble emitted for burst group-start (reliability mode)",
                          log_prefix_.c_str());
            }
        } else if (i == 0 && (force_first_full_preamble || waveform_->supportsDataPreamble())) {
            preamble = waveform_->generatePreamble();
            if (force_first_full_preamble) {
                LOG_MODEM(INFO, "[%s] Full preamble forced for first burst frame OFDM timing anchor",
                          log_prefix_.c_str());
            } else {
                LOG_MODEM(INFO, "[%s] Full preamble emitted for first OFDM burst frame timing anchor",
                          log_prefix_.c_str());
            }
        } else if (i == 0) {
            // Waveforms without a separate data preamble already use their full preamble.
            preamble = waveform_->generatePreamble();
        } else if (frame_in_burst_group[i]) {
            // Burst-interleaved group MEMBER (not the start): plain light LTS, NO
            // short re-anchor chirp. The RX accumulates the group at a fixed
            // burst_min_block_ = [LTS + data] stride, so a per-frame chirp prefix
            // would push every member past the stride and progressively misalign
            // the FFT window. MUST match the group-start preamble above. (§14.25)
            preamble = connectedDataPreambleForFrame();
        } else {
            const auto header = protocol::v2::parseHeader(wire_frame_data_list[i]);
            const bool is_data_frame =
                header.valid && protocol::v2::isDataFrame(header.type);
            preamble = connectedDataPreambleForFrame();
        }

        // Negate first LTS symbol for burst-interleaved group starts
        if (frame_is_group_start[i] && !preamble.empty()) {
            // LTS preamble = 2 identical symbols. Negate the first one.
            // detectDataSync() correlates sym[n] with sym[n+L]:
            //   Normal: P_real > 0 (same signs multiply to positive)
            //   Negated: P_real < 0 (opposite signs multiply to negative)
            // abs() ensures detection still works; sign indicates burst marker.
            markFirstLTSSymbolForBurstGroup(preamble, ofdm_config_.getSymbolDuration());
            LOG_MODEM(INFO, "[%s] LTS marker: negated first symbol for frame %zu (group start)",
                      log_prefix_.c_str(), i);
        }

        // DIAGNOSTIC (ULTRA_GENIE_DATA_AIDED, default off): absolute offset of this
        // frame's first data symbol in the burst stream = current fill + preamble.
        // The preamble is appended AFTER modulate() below, hence the explicit add.
        if (ultra::genie::txCapture().enabled) {
            ultra::genie::txCapture().frame_data_base =
                static_cast<long long>(result.size() + preamble.size());
        }

        // Modulate data
        Samples modulated = waveform_->modulate(encoded_frames[i]);

        result.insert(result.end(), preamble.begin(), preamble.end());
        result.insert(result.end(), modulated.begin(), modulated.end());
    }

    // burst_interleave reports the ACTUAL byte-permutation flag (use_burst_interleave_),
    // NOT the group count — interleaved_groups is the number of 6-frame transport groups
    // formed (always ≥1 for a file burst, whether or not the bytes were permuted), so
    // printing it here read as "interleave=yes" even on the interleave-OFF SR-ARQ path.
    LOG_MODEM(INFO, "[%s] Encoded burst: %zu blocks -> %zu samples (burst_interleave=%s groups=%zu)",
              log_prefix_.c_str(), frame_data_list.size(), result.size(),
              use_burst_interleave_ ? "yes" : "no", interleaved_groups);

    applyPaprReductionIfNeeded(result, protocol::isOFDMMode(mode_), false, "burst-light");

    return result;
}

std::vector<float> StreamingEncoder::encodePing() {
    // PING is just the preamble (chirp) with no data
    // Always use MC-DPSK waveform for PING
    if (!control_waveform_) {
        control_waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);
    }

    auto preamble = control_waveform_->generatePreamble();
    return std::vector<float>(preamble.begin(), preamble.end());
}

std::vector<float> StreamingEncoder::encodeToneBurstAck(
    const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload,
    uint32_t symbol_ms) {
    // PHY_ADAPTATION_DESIGN §15 step 4c. Build the tone-burst ACK audio by
    // delegating to the standalone ToneBurstEncoder. Local instance: encoder
    // state is just a phase accumulator that wraps every ~850 ms (one burst),
    // so there's no benefit to caching across calls and a fresh instance
    // avoids any cross-call phase leak.
    ultra::waveform::tone_burst_ack::ToneBurstEncoder enc;
    auto samples = enc.encode(payload, symbol_ms);
    LOG_MODEM(INFO,
              "[%s] ToneBurstAck encoded: group_seq=%u type=%s frame_mask=0x%04X "
              "rate_hint=%u drive_advisory=%u symbol_ms=%u samples=%zu (%.0f ms airtime)",
              log_prefix_.c_str(),
              static_cast<unsigned>(payload.group_seq),
              payload.type == ultra::waveform::tone_burst_ack::AckType::Nack
                  ? "NACK"
                  : "ACK",
              static_cast<unsigned>(payload.frame_mask),
              static_cast<unsigned>(payload.rate_hint),
              static_cast<unsigned>(payload.drive_advisory),
              static_cast<unsigned>(symbol_ms),
              samples.size(),
              static_cast<float>(samples.size()) * 1000.0f /
                  static_cast<float>(ultra::waveform::tone_burst_ack::kSampleRate));
    return samples;
}

std::vector<float> StreamingEncoder::encodeDataOnly(const Bytes& frame_data) {
    if (!waveform_) {
        LOG_MODEM(ERROR, "[%s] No waveform!", log_prefix_.c_str());
        return {};
    }

    // Encode and modulate without preamble
    Bytes encoded = encodeFrameBytes(frame_data);
    Samples modulated = waveform_->modulate(encoded);

    return std::vector<float>(modulated.begin(), modulated.end());
}

// ============================================================================
// CONFIGURATION ACCESS
// ============================================================================

EncoderConfig StreamingEncoder::getConfig() const {
    EncoderConfig cfg;
    cfg.mode = mode_;
    cfg.modulation = modulation_;
    cfg.code_rate = code_rate_;
    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        cfg.num_carriers = mc_dpsk_config_.num_carriers;
        cfg.data_carriers = mc_dpsk_config_.num_carriers;
        cfg.bits_per_symbol = mc_dpsk_config_.num_carriers * mc_dpsk_config_.bits_per_symbol;
        cfg.use_pilots = false;
        cfg.pilot_spacing = 0;
    } else {
        cfg.num_carriers = ofdm_config_.num_carriers;
        cfg.data_carriers = calculateDataCarriers();
        cfg.bits_per_symbol = ofdm_link_adaptation::bitsPerOFDMSymbol(
            static_cast<int>(ofdm_config_.num_carriers),
            ofdm_config_.use_pilots,
            static_cast<int>(ofdm_config_.pilot_spacing),
            modulation_);
        cfg.use_pilots = ofdm_config_.use_pilots;
        cfg.pilot_spacing = ofdm_config_.pilot_spacing;
    }
    cfg.use_channel_interleave = use_channel_interleave_;
    cfg.use_frame_interleave = use_frame_interleave_;
    return cfg;
}

std::string StreamingEncoder::verifyConfigMatch(const EncoderConfig& other) const {
    auto mine = getConfig();
    std::string mismatches;

    if (mine.mode != other.mode) {
        mismatches += "mode mismatch; ";
    }
    if (mine.modulation != other.modulation) {
        mismatches += "modulation mismatch; ";
    }
    if (mine.code_rate != other.code_rate) {
        mismatches += "code_rate mismatch; ";
    }
    if (mine.num_carriers != other.num_carriers) {
        char buf[64];
        snprintf(buf, sizeof(buf), "num_carriers: TX=%d RX=%d; ",
                 mine.num_carriers, other.num_carriers);
        mismatches += buf;
    }
    if (mine.data_carriers != other.data_carriers) {
        char buf[64];
        snprintf(buf, sizeof(buf), "data_carriers: TX=%d RX=%d; ",
                 mine.data_carriers, other.data_carriers);
        mismatches += buf;
    }
    if (mine.bits_per_symbol != other.bits_per_symbol) {
        char buf[64];
        snprintf(buf, sizeof(buf), "bits_per_symbol: TX=%d RX=%d; ",
                 mine.bits_per_symbol, other.bits_per_symbol);
        mismatches += buf;
    }
    if (mine.use_pilots != other.use_pilots) {
        mismatches += "use_pilots mismatch; ";
    }
    if (mine.pilot_spacing != other.pilot_spacing) {
        char buf[64];
        snprintf(buf, sizeof(buf), "pilot_spacing: TX=%d RX=%d; ",
                 mine.pilot_spacing, other.pilot_spacing);
        mismatches += buf;
    }
    if (mine.use_channel_interleave != other.use_channel_interleave) {
        char buf[64];
        snprintf(buf, sizeof(buf), "channel_interleave: TX=%s RX=%s; ",
                 mine.use_channel_interleave ? "yes" : "no",
                 other.use_channel_interleave ? "yes" : "no");
        mismatches += buf;
    }
    if (mine.use_frame_interleave != other.use_frame_interleave) {
        mismatches += "frame_interleave mismatch; ";
    }

    return mismatches;
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

void StreamingEncoder::createWaveform() {

    // Always have MC-DPSK ready for control frames
    if (!control_waveform_) {
        control_waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);
    }

    switch (mode_) {
        case protocol::WaveformMode::MC_DPSK:
            if (narrowband_control_) {
                waveform_ = WaveformFactory::createNarrowbandMCDPSK();
            } else {
                waveform_ = WaveformFactory::createMCDPSK(mc_dpsk_config_);
            }
            break;

        case protocol::WaveformMode::OFDM_NARROW:
            waveform_ = std::make_unique<OFDMChirpWaveform>(ofdm_config_, protocol::WaveformMode::OFDM_NARROW);
            static_cast<OFDMChirpWaveform*>(waveform_.get())->configure(
                modulation_, code_rate_);
            break;

        case protocol::WaveformMode::OFDM_CHIRP:
        default:
            waveform_ = std::make_unique<OFDMChirpWaveform>(ofdm_config_);
            static_cast<OFDMChirpWaveform*>(waveform_.get())->configure(
                modulation_, code_rate_);
            break;
    }

    // Sync pilot_spacing from waveform after configure()
    // (coherent modes like QPSK use denser pilots than the config may specify)
    if (waveform_) {
        int spacing = waveform_->getPilotSpacing();
        if (spacing > 0) ofdm_config_.pilot_spacing = spacing;
        waveform_->setCarrierMask(carrier_mask_);
        waveform_->setCarrierLdpcInterleaverEnabled(use_carrier_ldpc_interleaver_);
    }

    LOG_MODEM(DEBUG, "[%s] Created waveform: %s",
              log_prefix_.c_str(), protocol::waveformModeToString(mode_));
}

void StreamingEncoder::updateInterleaver() {
    if (mode_ == protocol::WaveformMode::MC_DPSK) {
        // No channel interleaving for MC-DPSK
        channel_interleaver_.reset();
        return;
    }

    // Calculate bits per OFDM symbol
    int data_carriers = calculateDataCarriers();
    int bits_per_carrier = static_cast<int>(getBitsPerSymbol(modulation_));
    int bits_per_symbol = ofdm_link_adaptation::bitsPerOFDMSymbol(
        static_cast<int>(ofdm_config_.num_carriers),
        ofdm_config_.use_pilots,
        static_cast<int>(ofdm_config_.pilot_spacing),
        modulation_);

    // Create channel interleaver. At z=81 (N=1944) each LDPC codeword is 1944 bits
    // instead of 648, so the interleaver block size must match the active z.
    // Sourced from ldpc_lifting_z_ — the single TX source of truth, set per-burst
    // by the connection-layer policy and announced in BURST_HEADER payload[5].
    const size_t ldpc_codeword_bits_ci =
        (ldpc_lifting_z_ == 81) ? size_t{1944} : v2::LDPC_CODEWORD_BITS;
    channel_interleaver_ = std::make_unique<ChannelInterleaver>(
        bits_per_symbol, ldpc_codeword_bits_ci);

    LOG_MODEM(INFO, "[%s] Channel interleaver: %d data carriers × %d bits = %d bits/symbol",
              log_prefix_.c_str(), data_carriers, bits_per_carrier, bits_per_symbol);
}

int StreamingEncoder::calculateDataCarriers() const {
    return ofdm_link_adaptation::dataCarrierCount(
        static_cast<int>(ofdm_config_.num_carriers),
        ofdm_config_.use_pilots,
        static_cast<int>(ofdm_config_.pilot_spacing));
}

Bytes StreamingEncoder::encodeFrameBytes(const Bytes& frame_data) {
    Bytes tx_data = frame_data;  // Mutable copy for header patching

    bool is_ofdm = protocol::isOFDMMode(mode_);

    if (!is_ofdm) {
        // MC-DPSK: Simple variable CW encoding (no frame interleaving)
        // Control frames (20 bytes): ACK, NACK, etc. - encode as-is, no patching
        // Data frames (>20 bytes): May need total_cw patching

        bool is_connect_frame = false;
        if (tx_data.size() >= 3) {
            const auto frame_type = static_cast<v2::FrameType>(tx_data[2]);
            is_connect_frame =
                tx_data.size() > v2::ControlFrame::SIZE &&
                v2::isConnectFrame(frame_type);
        }

        if (is_connect_frame) {
            if (tx_data.size() >= v2::DataFrame::HEADER_SIZE &&
                tx_data[12] != v2::kConnectFrameCodewords) {
                tx_data[12] = v2::kConnectFrameCodewords;
                uint16_t hcrc = v2::ControlFrame::calculateCRC(tx_data.data(), 15);
                tx_data[15] = (hcrc >> 8) & 0xFF;
                tx_data[16] = hcrc & 0xFF;
                const size_t fcrc_offset = tx_data.size() - 2;
                uint16_t fcrc = v2::ControlFrame::calculateCRC(tx_data.data(), fcrc_offset);
                tx_data[fcrc_offset] = (fcrc >> 8) & 0xFF;
                tx_data[fcrc_offset + 1] = fcrc & 0xFF;
            }

            Bytes encoded = v2::encodeFixedFrame(
                tx_data, CodeRate::R1_4, v2::kConnectFrameCodewords);
            LOG_MODEM(INFO, "[%s] MC-DPSK fixed CONNECT: %zu bytes -> %d CWs (%zu coded, frame_interleave=yes)",
                      log_prefix_.c_str(), tx_data.size(),
                      v2::kConnectFrameCodewords, encoded.size());
            return encoded;
        }

        if (tx_data.size() >= 3 &&
            tx_data[2] == static_cast<uint8_t>(v2::FrameType::DATA_REPAIR)) {
            auto repair = v2::DataRepairFrame::deserialize(tx_data);
            if (!repair) {
                LOG_MODEM(WARN, "[%s] MC-DPSK: dropping invalid DATA_REPAIR frame",
                          log_prefix_.c_str());
                return {};
            }
            auto cws = v2::encodeInfoCodewordsWithLDPC(repair->infoCodewords(), repair->rate);
            Bytes encoded;
            for (const auto& cw : cws) {
                encoded.insert(encoded.end(), cw.begin(), cw.end());
            }
            LOG_MODEM(INFO, "[%s] MC-DPSK DATA_REPAIR: seq=%d bitmap=0x%04X repair_cw=%d (%zu coded)",
                      log_prefix_.c_str(), repair->target_seq, repair->repair_bitmap,
                      repair->repair_count, encoded.size());
            return encoded;
        }

        // Check if this is a control frame (20 bytes, type 0x10-0x21 or 0x40)
        bool is_control_frame = false;
        if (tx_data.size() == 20 && tx_data.size() >= 3) {
            uint8_t frame_type = tx_data[2];
            // Control frame types: PROBE(0x10), PROBE_ACK(0x11), CONNECT(0x12),
            // CONNECT_ACK(0x13), CONNECT_NAK(0x14), DISCONNECT(0x15), KEEPALIVE(0x16),
            // MODE_CHANGE(0x17), ACK(0x20), NACK(0x21), BEACON(0x40)
            is_control_frame = (frame_type >= 0x10 && frame_type <= 0x21) ||
                               (frame_type == 0x40);
        }

        auto cws = v2::encodeFrameWithLDPC(tx_data, code_rate_);

        // Only patch DATA frames (not control frames)
        // DATA frames have total_cw at byte 12, header_crc at bytes 15-16
        if (!is_control_frame && tx_data.size() >= 17) {
            uint8_t actual_cw = static_cast<uint8_t>(cws.size());
            if (tx_data[12] != actual_cw) {
                LOG_MODEM(DEBUG, "[%s] Patching total_cw %d -> %d",
                          log_prefix_.c_str(), tx_data[12], actual_cw);
                tx_data[12] = actual_cw;
                // Recalculate header CRC (over first 15 bytes for DATA frames)
                uint16_t hcrc = v2::ControlFrame::calculateCRC(tx_data.data(), 15);
                tx_data[15] = (hcrc >> 8) & 0xFF;
                tx_data[16] = hcrc & 0xFF;
                // Recalculate frame CRC (covers patched bytes 12, 15-16)
                if (tx_data.size() >= v2::DataFrame::HEADER_SIZE + v2::DataFrame::CRC_SIZE) {
                    uint16_t fcrc = v2::ControlFrame::calculateCRC(tx_data.data(), tx_data.size() - 2);
                    tx_data[tx_data.size() - 2] = (fcrc >> 8) & 0xFF;
                    tx_data[tx_data.size() - 1] = fcrc & 0xFF;
                }
                // Re-encode with corrected header and frame CRC
                cws = v2::encodeFrameWithLDPC(tx_data, code_rate_);
            }
        }

        // Concatenate codewords
        Bytes encoded;
        for (const auto& cw : cws) {
            encoded.insert(encoded.end(), cw.begin(), cw.end());
        }

        LOG_MODEM(DEBUG, "[%s] MC-DPSK: %zu bytes -> %zu CWs (%zu coded, control=%s)",
                  log_prefix_.c_str(), tx_data.size(), cws.size(), encoded.size(),
                  is_control_frame ? "yes" : "no");
        return encoded;
    }

    // OFDM: Check if this is a control frame or data/connect frame
    // Control frames (ACK, NACK, MODE_CHANGE, DISCONNECT, etc.) are 20 bytes = 1 CW, no interleaving
    // Connect handshake frames are always MC-DPSK and do not reach this path.
    // Data frames use fixed-CW frame encoding with frame interleaving
    bool is_variable_cw_frame = false;
    if (tx_data.size() >= 3) {
        uint8_t frame_type = tx_data[2];
        auto ft = static_cast<v2::FrameType>(frame_type);
        if (v2::isControlFrame(ft)) {
            is_variable_cw_frame = true;
        }
        // Non-control frames go through encodeFixedFrame() for fixed-CW interleaving
    }

    if (is_variable_cw_frame) {
        // Variable-CW encoding (no frame interleaving needed)
        // Control frames = 1 CW
        // Control frames always use R1/4: exact fit (20 bytes = 162 info bits / 8)
        // and maximum LDPC redundancy for fading resilience
        // INV-FRAME-WHITEN-001: a one-codeword control frame is whitened before LDPC encoding so
        // its systematic header bits cannot line its symbols' carriers up into a peak that sets
        // the whole burst's level on a peak-limited transmitter (frame_v2.hpp).
        const bool whiten_control = tx_data.size() == v2::ControlFrame::SIZE &&
                                    v2::controlFrameWhiteningEnabled();
        auto cws = v2::encodeFrameWithLDPC(
            whiten_control ? v2::whitenControlFrame(tx_data) : tx_data, CodeRate::R1_4);
        uint8_t actual_cw = static_cast<uint8_t>(cws.size());

        // Patch total_cw for ConnectFrames (CONNECT, DISCONNECT, etc.)
        // ConnectFrame::serialize() hardcodes total_cw=4 at byte 12, but actual
        // encoding may produce fewer CWs (e.g. 2 at R1/2 for 44-byte ConnectFrame)
        // ControlFrames (20 bytes) don't have total_cw field — parseHeader() returns 1
        auto ft = static_cast<v2::FrameType>(tx_data[2]);
        if (v2::isConnectFrame(ft) &&
            tx_data.size() > v2::ControlFrame::SIZE &&
            tx_data[12] != actual_cw) {
            LOG_MODEM(INFO, "[%s] OFDM: Patching ConnectFrame total_cw %d -> %d",
                      log_prefix_.c_str(), tx_data[12], actual_cw);
            tx_data[12] = actual_cw;
            // Recalculate header CRC (bytes 0..14 → CRC at 15-16)
            uint16_t hcrc = v2::ControlFrame::calculateCRC(tx_data.data(), 15);
            tx_data[15] = (hcrc >> 8) & 0xFF;
            tx_data[16] = hcrc & 0xFF;
            // Recalculate frame CRC (last 2 bytes cover everything before them)
            size_t fcrc_offset = tx_data.size() - 2;
            uint16_t fcrc = v2::ControlFrame::calculateCRC(tx_data.data(), fcrc_offset);
            tx_data[fcrc_offset] = (fcrc >> 8) & 0xFF;
            tx_data[fcrc_offset + 1] = fcrc & 0xFF;
            // Re-encode with patched header (always R1/4 for control frames)
            cws = v2::encodeFrameWithLDPC(tx_data, CodeRate::R1_4);
        }

        Bytes encoded;
        for (const auto& cw : cws) {
            encoded.insert(encoded.end(), cw.begin(), cw.end());
        }

        LOG_MODEM(INFO, "[%s] OFDM control: %zu bytes -> %zu CW (%zu coded bytes), rate=R1/4 (hardened)",
                  log_prefix_.c_str(), tx_data.size(), cws.size(), encoded.size());
        return encoded;
    }

    // Large OFDM block frames explicitly advertise a size beyond the fixed
    // aggregation range and use
    // variable-CW encoding. The fixed-frame path remains the default for normal
    // ARQ chunks so they keep frame-level interleaving.
    if (tx_data.size() >= v2::DataFrame::HEADER_SIZE &&
        tx_data[12] > v2::kMaxFixedFrameCodewords) {
        auto cws = v2::encodeFrameWithLDPC(tx_data, code_rate_);
        Bytes encoded;
        for (const auto& cw : cws) {
            encoded.insert(encoded.end(), cw.begin(), cw.end());
        }

        LOG_MODEM(INFO, "[%s] OFDM variable data: %zu bytes -> %zu CWs (%zu coded)",
                  log_prefix_.c_str(), tx_data.size(), cws.size(), encoded.size());
        return encoded;
    }

    int frame_cw_count = fixed_frame_codewords_;
    if (tx_data.size() >= v2::DataFrame::HEADER_SIZE &&
        tx_data[12] >= v2::kMinFixedFrameCodewords &&
        tx_data[12] <= v2::kMaxFixedFrameCodewords) {
        frame_cw_count = tx_data[12];
    }

    // Data frames: fixed-CW frame encoding with frame interleaving
    // Channel interleaving is controlled by use_channel_interleave_ flag
    size_t bps = static_cast<size_t>(ofdm_link_adaptation::bitsPerOFDMSymbol(
        static_cast<int>(ofdm_config_.num_carriers),
        ofdm_config_.use_pilots,
        static_cast<int>(ofdm_config_.pilot_spacing),
        modulation_));
    // LDPC lifting Z sourced from the active member ldpc_lifting_z_ (27 or 81) —
    // the single TX source of truth, set per-burst by the connection-layer policy
    // and announced in BURST_HEADER payload[5] so the RX matches.
    const int ldpc_z = static_cast<int>(ldpc_lifting_z_);

    // Stage 1 of the 2026-11-08 speed plan: ONE turbo codeword over the whole frame when the
    // lockstep data-frame FEC is turbo (ULTRA_DATA_FEC / ULTRA_FAST); same air geometry and capacity.
    Bytes encoded = (v2::ofdmDataFrameFec() == v2::DataFrameFec::TURBO)
        ? v2::encodeTurboFixedFrame(tx_data, code_rate_, frame_cw_count, ldpc_z)
        : v2::encodeFixedFrame(tx_data, code_rate_, frame_cw_count,
                               use_channel_interleave_, bps,
                               ldpc_z);

    LOG_MODEM(DEBUG, "[%s] OFDM data: %zu bytes -> %d CWs (%zu coded, frame_interleave=%s, channel_interleave=%s, ldpc_z=%d)",
              log_prefix_.c_str(), tx_data.size(), frame_cw_count, encoded.size(),
              use_frame_interleave_ ? "yes" : "no",
              use_channel_interleave_ ? "yes" : "no",
              ldpc_z);

    return encoded;
}

} // namespace gui
} // namespace ultra
