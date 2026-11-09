#pragma once

// ModemEngine - Main modem interface class
// Handles TX/RX audio processing with OFDM and DPSK modulation

#include "modem_types.hpp"
#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"  // Interleaver, ChannelInterleaver
#include "fec/codec_factory.hpp"  // ICodec, CodecFactory
#include "ultra/dsp.hpp"  // FIRFilter
#include "psk/dpsk.hpp"   // DPSKModulator, DPSKDemodulator
#include "psk/multi_carrier_dpsk.hpp"  // MultiCarrierDPSK for fading channels
#include "sync/chirp_sync.hpp"  // ChirpSync for robust fading channel detection
#include "../adaptive_mode.hpp"
#include "fec/soft_combine.hpp"
#include "protocol/frame_v2.hpp"  // v2::FrameType
#include "protocol/waveform_selection.hpp"  // WaveformRecommendation, recommendWaveformAndRate
#include "waveform/waveform_interface.hpp"  // IWaveform abstraction
#include "audio/channel_busy_detector.hpp"  // shared carrier-sense (listen-before-talk)
#include "streaming_decoder.hpp"  // StreamingDecoder - primary decoder
#include "streaming_encoder.hpp"  // StreamingEncoder - unified TX encoder
#include <memory>
#include <vector>
#include <queue>
#include <mutex>
#include <functional>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>

namespace ultra {
namespace gui {

// Real modem engine using OFDM modulator/demodulator and LDPC codec
class ModemEngine {
public:
    ModemEngine();
    explicit ModemEngine(const MultiCarrierDPSKConfig& mc_dpsk_config);
    ~ModemEngine();

    // Set a name/prefix for logging (e.g., "OUR" or "SIM")
    void setLogPrefix(const std::string& prefix);
    const std::string& getLogPrefix() const { return log_prefix_; }
    // Set the LIVE local callsign used by the RX address filter (deliverFrame). Distinct from
    // the logging prefix: this tracks runtime callsign changes (e.g. a VARA host MYCALL) so
    // inbound frames addressed to the operator's current callsign are not dropped.
    void setLocalCallsign(const std::string& call);
    void setSoftCombineBuffer(fec::SoftCombineBuffer* buffer);
    void setHarqProvisionalContextCallback(
        StreamingDecoder::HarqProvisionalContextCallback cb);

    // ========================================================================
    // CONFIGURATION
    // ========================================================================
    void setConfig(const ModemConfig& config);
    const ModemConfig& getConfig() const { return config_; }

    void setFilterConfig(const FilterConfig& config);
    const FilterConfig& getFilterConfig() const { return filter_config_; }
    void setFilterEnabled(bool enabled);
    bool isFilterEnabled() const { return filter_config_.enabled; }
    void setPaprReductionEnabled(bool enabled);
    bool isPaprReductionEnabled() const;

    // ========================================================================
    // TX: Convert data to audio samples
    // ========================================================================
    std::vector<float> transmit(const std::string& text);
    std::vector<float> transmit(const Bytes& data);

    // Transmit multiple frames as a single waveform burst.
    // Used for connected OFDM and MC-DPSK DATA windows.
    std::vector<float> transmitBurst(
        const std::vector<Bytes>& frame_data_list,
        uint16_t group_seq = 0,
        BurstAnchorOptions anchor_options = {});

    // Minimal ping/pong probe (fast presence check, ~1 sec vs ~16 sec CONNECT)
    // Returns: preamble + raw DPSK "ULTR" bytes (no LDPC encoding)
    std::vector<float> transmitPing();
    std::vector<float> transmitPong();  // Same as ping, context determines meaning

    // §15 step 4d-iii: encode a tone-burst ACK to raw audio samples (4-FSK
    // in the 2400-2700 Hz subband). Delegates to StreamingEncoder::
    // encodeToneBurstAck (step 4c). The caller queues the returned samples
    // on the audio output ring just like any other transmit*() result.
    std::vector<float> transmitToneBurstAck(
        const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload,
        uint32_t symbol_ms =
            ultra::waveform::tone_burst_ack::kBaselineSymbolMs);

    // Test signal generation
    std::vector<float> generateTestTone(float duration_sec = 1.0f);
    std::vector<float> transmitTestPattern(int pattern = 0);
    std::vector<float> transmitRawOFDM(int pattern = 0);

    // ========================================================================
    // RX: Audio input (thread-safe, non-blocking)
    // ========================================================================
    // Feed audio samples - call from any thread (audio callback, simulator, etc.)
    // Samples are buffered internally and processed by background threads.
    // Decoded frames trigger the RawDataCallback automatically.
    void feedAudio(const float* samples, size_t count);
    void feedAudio(const std::vector<float>& samples);

    // Synchronous mode: caller drives decode instead of internal thread.
    // Use for simulation where feed+decode must be lockstep (no overflows).
    void setSynchronousMode(bool enabled);
    bool isSynchronousMode() const { return synchronous_mode_; }

    // Process RX buffer synchronously (call after feedAudio in sync mode).
    // Does one round of search/decode, same as the internal decode thread would.
    void processRxBuffer();

    // Inject test signal from file (for debugging/testing)
    size_t injectSignalFromFile(const std::string& filepath);

    // Check if we have decoded data ready (for polling if not using callbacks)
    bool hasReceivedData() const;

    // Get received data (clears the buffer) - use callbacks instead when possible
    std::string getReceivedText();
    Bytes getReceivedData();

    // ========================================================================
    // STATUS & CALLBACKS
    // ========================================================================
    LoopbackStats getStats() const;
    DecoderStats getDecoderStats() const;
    bool isSynced() const;
    float getCurrentSNR() const;
    float getFadingIndex() const;  // Fading index from per-carrier variance (0-1)
    bool isFading() const;         // True if fading_index > 0.65
    // Doppler coherence (Good/Moderate discriminator) from the OFDM demodulator.
    float getDopplerCoherenceScore() const;
    // ULTRA_RATE_EVM: every data frame's EVM SNR of the group just delivered to the
    // group callback (NaN where unmeasured). Decode-thread / group-callback scope.
    std::vector<float> getLastGroupEvmSnrFrames() const;
    bool getDopplerCoherenceValid() const;
    // RMS Doppler (Hz) from the same estimator — SECONDARY/approximate readout (fixed
    // nominal cadence), 0 when not estimable. Rides the coherence feed into the protocol
    // for the retx trough-pacing deferral only (docs/RETX_PACING_DESIGN_2026_07_03.md).
    float getDopplerCoherenceDopplerHz() const;
    // F129: decoder-evidence RX activity (sync/frame metrics within `within_ms`).
    bool rxSignalActive(int64_t within_ms = 1600) const;
    int64_t lastRxSignalMs() const;
    int64_t lastRxSubstantiveMs() const;  // F147: accepted-decode evidence only
    // Two-SNR model: last PHYSICAL in-band channel SNR (power ratio vs the
    // burst-time noise ref) measured at the connect sequence. BETA — see
    // BUG-PHYSICAL-SNR-RIG-REF. Valid=false until first data-bearing frame.
    bool hasLastPhysicalSnr() const;
    float lastPhysicalSnrDb() const;
    size_t physicalSnrStats(float& mean_db, float& spread_db) const;
    ChannelQuality getChannelQuality() const;
    // Live OFDM in-band (broadband) SNR from the decoder's lock-free last-estimate
    // atomics — updated per logical frame on BOTH delivery paths (incl. burst-as-unit,
    // which bypasses the stats-queue drain). The honest feed for the §15.5 ACK
    // staircase (BUG-ACK-STAIRCASE-FADE-BIN layer 2).
    bool hasLastOFDMBroadbandSNR() const;
    float getLastOFDMBroadbandSNR() const;
    // Radio-agnostic decision-directed EVM usable-SNR (Stage 2) from the decoder's
    // lock-free last-estimate atomics — measures the equalized-constellation scatter,
    // constant-free and non-inflating. The honest usable-dB feed for the RX-AUTHORITY
    // EVM demote authority (ULTRA_EVM_DEMOTE).
    bool hasLastEvmSnr() const;
    float getLastEvmSnrDb() const;
    // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET): latest per-burst RX level verdict
    // (connection_policy::RxLevelVerdict as int: 0=OK, 1=LOW, 2=CLIPPED) + its
    // measurement seq (fresh-vs-stale dedup). Lock-free decoder atomics.
    int getRxLevelVerdict() const;
    uint32_t getRxLevelVerdictSeq() const;

    // Set known CFO for light preamble mode (for testing or external CFO source)
    void setKnownCFO(float cfo_hz);
    std::vector<std::complex<float>> getConstellationSymbols() const;
    Modulation getConstellationModulation() const;

    using DataCallback = std::function<void(const std::string&)>;
    void setDataCallback(DataCallback callback) { data_callback_ = callback; }

    // The second argument is decoder-owned physical-turn provenance.  Keep a
    // one-argument overload for tools/tests that only consume decoded bytes;
    // those clients cannot accidentally manufacture completion evidence.
    using RawDataCallback = std::function<void(const Bytes&, bool physical_turn_complete)>;
    using LegacyRawDataCallback = std::function<void(const Bytes&)>;
    void setRawDataCallback(RawDataCallback callback) {
        raw_data_callback_ = std::move(callback);
    }
    void setRawDataCallback(LegacyRawDataCallback callback) {
        raw_data_callback_ =
            [callback = std::move(callback)](const Bytes& data, bool) {
                if (callback) callback(data);
            };
    }

    // §14.27 one-way burst transport: a decoded interleaved burst delivered as a
    // unit (group_seq, ordered DATA frames, all-logical-frames-decoded flag).
    using BurstGroupCallback =
        std::function<void(uint16_t group_seq, const std::vector<Bytes>& frames, bool all_ok,
                           float quality, uint16_t frame_mask, bool interleaved,
                           uint8_t group_size, bool geometry_proven)>;
    void setBurstGroupCallback(BurstGroupCallback callback) {
        burst_group_callback_ = std::move(callback);
    }
    // DESC-SWITCH Phase 1 (ULTRA_DESCRIPTOR_MODE_SWITCH): delegate the decoder's
    // mode-hop BURST_HEADER notification (streaming_decoder.hpp typedef) to the
    // protocol layer — wired by wireModemToProtocol like the tone-burst ACK path.
    // Only fires when the knob is ON (decoder-gated); byte-identical OFF.
    uint64_t burstAirSamplesRemaining() const {
        return streaming_decoder_ ? streaming_decoder_->burstAirSamplesRemaining() : 0;
    }
    const std::vector<float>& getLastGroupCarrierGammas() const {
        static const std::vector<float> kEmpty;
        return streaming_decoder_ ? streaming_decoder_->getLastGroupCarrierGammas()
                                  : kEmpty;
    }
    void setAnchoredBurstNoGroupCallback(
        std::function<void(bool payload_seen)> callback) {
        if (streaming_decoder_) {
            streaming_decoder_->setAnchoredBurstNoGroupCallback(std::move(callback));
        }
    }
    void setBurstOutcomeUnknownCallback(std::function<void()> callback) {
        if (streaming_decoder_) {
            streaming_decoder_->setBurstOutcomeUnknownCallback(std::move(callback));
        }
    }
    void setDescriptorModeChangeCallback(DescriptorModeChangeCallback callback) {
        if (streaming_decoder_) {
            streaming_decoder_->setDescriptorModeChangeCallback(std::move(callback));
        }
    }
    void setBurstTransportRxEnabled(bool enabled) {
        burst_transport_rx_enabled_ = enabled;
        if (streaming_decoder_) streaming_decoder_->setBurstTransportRxEnabled(enabled);
    }

    using DataSyncAcceptedCallback = std::function<void(float sync_correlation)>;
    void setDataSyncAcceptedCallback(DataSyncAcceptedCallback callback) {
        data_sync_accepted_callback_ = callback;
    }

    using StatusCallback = std::function<void(const std::string&)>;
    void setStatusCallback(StatusCallback callback) { status_callback_ = callback; }

    // Ping received callback - called when "ULTR" magic detected via DPSK
    // The measured_snr is estimated from preamble energy
    using PingReceivedCallback = std::function<void(float measured_snr)>;
    void setPingReceivedCallback(PingReceivedCallback callback) { ping_received_callback_ = callback; }

    // §15 step 4d-i: Tone-burst ACK detection callback. Delegates to the
    // StreamingDecoder's always-on tone-burst ACK monitor (installed in
    // step 4b). The owner of ModemEngine (typically the GUI / ProtocolEngine)
    // installs this to bridge tone-burst detections to the protocol layer
    // — step 4d-ii will translate the detection into a synthetic GROUP_ACK
    // for Connection's ACK-arrived handler.
    using ToneBurstAckCallback =
        ultra::waveform::tone_burst_ack::ToneBurstAckCallback;
    using ToneBurstAckAcceptancePredicate =
        ultra::waveform::tone_burst_ack::ToneBurstAckAcceptancePredicate;
    void setToneBurstAckCallback(ToneBurstAckCallback callback) {
        if (streaming_decoder_) {
            streaming_decoder_->setToneBurstAckCallback(std::move(callback));
        }
    }
    void setToneBurstAckAcceptancePredicate(
        ToneBurstAckAcceptancePredicate predicate) {
        if (streaming_decoder_) {
            streaming_decoder_->setToneBurstAckAcceptancePredicate(
                std::move(predicate));
        }
    }

    // §15 step 4d-late: arm the receiver's tone-burst ACK monitor for an
    // expected ACK arrival. Connection calls this right after queueing a
    // data burst; the monitor runs detection at a tight cadence until a
    // successful decode fires or the window elapses.
    void armToneBurstAckMonitor(uint32_t window_ms) {
        if (streaming_decoder_) {
            streaming_decoder_->armToneBurstMonitor(window_ms);
        }
    }

    void rearmToneBurstAckMonitor(uint32_t window_ms) {
        if (streaming_decoder_) {
            streaming_decoder_->rearmToneBurstMonitor(window_ms);
        }
    }

    // Check if last detected chirp was narrowband (valid after ping callback)
    bool isNarrowbandDetected() const;

    // Switch encoder to narrowband control chirps (for initiator forcing OFDM_NARROW)
    void setNarrowbandControl(bool narrowband);

    void reset();
    // Clear RX audio buffer (use before TX to prevent echo). for_tx_echo=true marks the
    // pre-TX half-duplex echo-clear (fires before every ACK/burst); during a connected OFDM
    // transfer that path can preserve warm-sync state (ULTRA_WARM_TURNAROUND) instead of a
    // full decoder reset that wipes warm re-acquisition (the ~27% turnaround penalty, #67).
    void clearRxBuffer(bool for_tx_echo = false);

    // Carrier sense (listen-before-talk) lives in the shared ChannelBusyDetector
    // (src/audio/channel_busy_detector.cpp), reached by simulator/TNC stations via
    // AudioPort::isChannelIdleFor(). The old fixed-threshold ModemEngine energy
    // detector was an unwired dead stub and has been removed.

    // Half-duplex turnaround delay
    void setTurnaroundDelay(uint32_t delay_ms) { turnaround_delay_ms_ = delay_ms; }
    uint32_t getTurnaroundDelay() const { return turnaround_delay_ms_; }
    bool isTurnaroundActive() const;
    uint32_t getTurnaroundRemaining() const;

    // Carrier sense (listen-before-talk). Energy detection is only meaningful in
    // the OFDM regime: at MC-DPSK SNRs the peer sits at/below the noise floor and
    // is not energy-detectable, so CCA is skipped there (stop-and-wait turnaround
    // coordinates instead). Gating to OFDM also matches where the half-duplex
    // collision actually occurs (multi-frame OFDM bursts).
    bool carrierSenseActiveForTx() const {
        return protocol::isOFDMMode(waveform_mode_);
    }
    bool channelIdleForTx(std::chrono::milliseconds guard = std::chrono::milliseconds(0)) const {
        return !carrierSenseActiveForTx() || channel_busy_detector_.isIdleFor(guard);
    }
    bool channelBusyForTx() const {
        return !channelIdleForTx();
    }
    float channelRms() const { return channel_busy_detector_.currentRms(); }
    float channelQuietThreshold() const { return channel_busy_detector_.quietThreshold(); }

    // ========================================================================
    // WAVEFORM & MODE CONTROL
    // ========================================================================
    void setWaveformMode(protocol::WaveformMode mode);
    protocol::WaveformMode getWaveformMode() const { return waveform_mode_; }

    void setConnected(bool connected);
    bool isConnected() const { return connected_; }

    // Teardown keeps the negotiated OFDM receiver live for ACK/control frames but
    // forbids speculative fallback into the connected DATA geometry.
    void setControlOnlyReceive(bool enabled) {
        if (streaming_decoder_) {
            streaming_decoder_->setControlOnlyReceive(enabled);
        }
    }

    void setUseConnectedWaveformOnce() { use_connected_waveform_once_ = true; }

    void setConnectWaveform(protocol::WaveformMode mode);
    protocol::WaveformMode getConnectWaveform() const { return connect_waveform_; }

    void setLastRxWaveform(protocol::WaveformMode mode) { last_rx_waveform_ = mode; }
    protocol::WaveformMode getLastRxWaveform() const { return last_rx_waveform_; }

    void setHandshakeComplete(bool complete);
    bool isHandshakeComplete() const { return handshake_complete_; }

    // #70: forward the handshake-stage gate (bare-chirp-expected) to the decoder so
    // ULTRA_ROBUST_IDLE_PING only fires a robust chirp-lock PONG when the local
    // station expects a BARE CHIRP next (idle/probing), never a data control frame
    // (connecting -> CONNECT_ACK). No-op unless the knob is set.
    void setBareChirpExpected(bool v) {
        if (streaming_decoder_) streaming_decoder_->setBareChirpExpected(v);
    }

    void setDataMode(Modulation mod, CodeRate rate);
    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    // NOTE: For modulation/rate selection, use protocol::recommendDataMode()
    // from waveform_selection.hpp instead
    static protocol::WaveformMode recommendWaveformMode(float snr_db);

    // Waveform + rate recommendation based on SNR AND fading
    // This is the primary recommendation function - uses both metrics
    // Delegates to protocol::recommendWaveformAndRate() for the algorithm
    using WaveformRecommendation = protocol::WaveformRecommendation;
    static WaveformRecommendation recommendWaveformAndRate(float snr_db, float fading_index);

    void setDPSKMode(DPSKModulation mod, int samples_per_symbol = 384);
    DPSKModulation getDPSKModulation() const { return dpsk_config_.modulation; }
    const DPSKConfig& getDPSKConfig() const { return dpsk_config_; }

    // MC-DPSK configuration
    int getMCDPSKCarriers() const { return mc_dpsk_config_.num_carriers; }

    // §14.36 live decoder state for the GUI's RX panel. Reflects the descriptor-
    // driven rate switch from adaptive bursts (not just the negotiated rate).
    CodeRate getDecoderCodeRate() const {
        return streaming_decoder_ ? streaming_decoder_->getCodeRate() : CodeRate::AUTO;
    }
    Modulation getDecoderModulation() const {
        return streaming_decoder_ ? streaming_decoder_->getModulation() : Modulation::DQPSK;
    }
    float getMCDPSKThroughput() const { return mc_dpsk_config_.getRawBitRate() * 0.25f; } // R1/4 FEC
    void setMCDPSKConfig(const MultiCarrierDPSKConfig& config);
    void setMCDPSKProfile(int num_carriers, int samples_per_symbol, int bits_per_symbol) {
        if (mc_dpsk_config_.num_carriers == num_carriers &&
            mc_dpsk_config_.samples_per_symbol == samples_per_symbol &&
            mc_dpsk_config_.bits_per_symbol == bits_per_symbol) {
            return;
        }
        mc_dpsk_config_.num_carriers = num_carriers;
        mc_dpsk_config_.samples_per_symbol = samples_per_symbol;
        mc_dpsk_config_.bits_per_symbol = bits_per_symbol;
        setMCDPSKConfig(mc_dpsk_config_);
    }
    void setMCDPSKCarriers(int num_carriers) {
        mc_dpsk_config_.num_carriers = num_carriers;
        setMCDPSKConfig(mc_dpsk_config_);
    }

    // Recommend MC-DPSK carrier count based on channel conditions
    // Returns 8 for fading/low SNR, up to 13 for stable/high SNR
    static int recommendMCDPSKCarriers(float snr_db, float fading_index);

    // Burst interleave control (for connected OFDM_CHIRP mode)
    void setBurstInterleave(bool enable) {
        if (streaming_encoder_) streaming_encoder_->setBurstInterleave(enable);
        if (streaming_decoder_) streaming_decoder_->setBurstInterleave(enable);
    }

    // Force fixed-frame codeword count on encoder + decoder. Used by
    // the GUI's monitor-mode override so the decoder doesn't default
    // to 1-CW control-frame demod on incoming OFDM data frames.
    void setFixedFrameCodewords(int cw_count) {
        if (streaming_encoder_) streaming_encoder_->setFixedFrameCodewords(cw_count);
        if (streaming_decoder_) streaming_decoder_->setFixedFrameCodewords(cw_count);
    }
    // Per-burst LDPC lifting Z for the TX encoder, pushed from the connection
    // policy (Connection::selectBurstLiftingZ) by the app. The encoder writes it
    // into the BURST_HEADER descriptor so the RX matches via the wire. Default 27.
    void setBurstLiftingZ(uint8_t z) { burst_lifting_z_ = (z == 81) ? 81 : 27; }
    void expectFullOFDMAnchorOnce() {
        if (streaming_decoder_) streaming_decoder_->expectFullOFDMAnchorOnce();
    }
    void clearFullOFDMAnchorExpectation() {
        if (streaming_decoder_) streaming_decoder_->clearFullOFDMAnchorExpectation();
    }
    void setFixedFrameHeaderDiscovery(bool enable) {
        if (streaming_decoder_) streaming_decoder_->setFixedFrameHeaderDiscovery(enable);
    }
    void setCarrierLdpcInterleaver(bool enable) {
        if (streaming_encoder_) streaming_encoder_->setCarrierLdpcInterleaver(enable);
        if (streaming_decoder_) streaming_decoder_->setCarrierLdpcInterleaver(enable);
    }

    // Pass-throughs so the headless ultra_tnc can drive the modem through ModemEngine
    // (rather than owning a raw StreamingEncoder/StreamingDecoder) — thin forwarders that
    // mirror the wrappers above, giving both frontends one shared modem object.
    void setOFDMConfig(const ModemConfig& config) {
        if (streaming_encoder_) streaming_encoder_->setOFDMConfig(config);
        if (streaming_decoder_) streaming_decoder_->setOFDMConfig(config);
    }
    void setConnectedOFDMMode(protocol::WaveformMode mode, const ModemConfig& config,
                              Modulation mod, CodeRate rate) {
        if (streaming_decoder_)
            streaming_decoder_->setConnectedOFDMMode(mode, config, mod, rate);
    }
    void seedExpectedFrameArrivalAfterSamples(size_t delay_samples,
                                              float confidence = 0.50f) {
        if (streaming_decoder_)
            streaming_decoder_->seedExpectedFrameArrivalAfterSamples(delay_samples, confidence);
    }
    BandwidthMode getDetectedBandwidth() const {
        return streaming_decoder_ ? streaming_decoder_->getDetectedBandwidth()
                                  : BandwidthMode::WIDE;
    }
    void forceNextFrameFullPreamble() {
        if (streaming_encoder_) streaming_encoder_->forceNextFrameFullPreamble();
    }

    // FEC codec control
    void setCodecType(fec::CodecType type);
    fec::CodecType getCodecType() const { return codec_type_; }
    static fec::CodecType recommendCodecType(float snr_db);
    static fec::CodecType getCodecForWaveform(protocol::WaveformMode mode);

private:
    ModemConfig config_;
    std::string log_prefix_ = "MODEM";
    // Live local callsign for the RX address filter (deliverFrame). Empty => fall back to the
    // log_prefix_-derived label (preserves existing GUI behavior where label == callsign).
    std::string filter_callsign_;

    // Waveform mode state
    protocol::WaveformMode waveform_mode_ = protocol::WaveformMode::OFDM_CHIRP;
    protocol::WaveformMode connect_waveform_ = protocol::WaveformMode::MC_DPSK;

    // Per-burst LDPC lifting Z, pushed from the connection policy (default 27).
    uint8_t burst_lifting_z_ = 27;
    protocol::WaveformMode last_rx_waveform_ = protocol::WaveformMode::MC_DPSK;
    protocol::WaveformMode disconnect_waveform_ = protocol::WaveformMode::MC_DPSK;  // Saved for ACK
    bool connected_ = false;
    bool handshake_complete_ = false;
    bool use_connected_waveform_once_ = false;

    // Data frame modulation (negotiated after probing)
    Modulation data_modulation_ = Modulation::DBPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;
    fec::CodecType codec_type_ = fec::CodecType::LDPC;  // FEC codec type

    // TX chain - StreamingEncoder (unified encoding for all waveform types)
    std::unique_ptr<StreamingEncoder> streaming_encoder_;

    // RX chain - OFDM
    fec::CodecPtr decoder_;  // ICodec for decoding (currently LDPC) - mostly unused, StreamingDecoder handles RX
    // NOTE: RX uses streaming_decoder_ (which has its own waveform) - no separate demodulator

    // DPSK config (used by StreamingDecoder, kept here for API compatibility)
    DPSKConfig dpsk_config_;

    // Multi-Carrier DPSK config (actual modulation done by IWaveform via StreamingDecoder)
    MultiCarrierDPSKConfig mc_dpsk_config_;

    // Chirp sync for robust presence detection on fading channels
    std::unique_ptr<sync::ChirpSync> chirp_sync_;

    // ========================================================================
    // StreamingDecoder (primary RX path)
    // ========================================================================
    // StreamingDecoder handles BOTH connected and disconnected modes:
    // - Circular buffer with bounded size
    // - Sliding window chirp detection
    // - Correct IWaveform call sequence (fixes BUG-002)
    // - Thread-safe with condition variable
    std::unique_ptr<StreamingDecoder> streaming_decoder_;

    // ========================================================================
    // RX ARCHITECTURE
    // ========================================================================

    // RX/Decode thread (disabled in synchronous mode)
    bool synchronous_mode_ = false;
    std::thread rx_decode_thread_;
    std::atomic<bool> rx_decode_running_{false};
    std::condition_variable rx_decode_cv_;
    std::mutex rx_decode_mutex_;

    void rxDecodeLoop();
    void startRxDecodeThread();
    void stopRxDecodeThread();

    // RX decode helpers (implemented in modem_rx_decode.cpp)
    // Returns false when a valid frame is addressed to another station (including
    // our own reflected TX).  Callers must suppress protocol/UI/diagnostic receive
    // notifications for such frames.
    bool deliverFrame(const DecodeResult& result);
    void notifyFrameParsed(const Bytes& frame_data, protocol::v2::FrameType frame_type);
    void updateStats(std::function<void(LoopbackStats&)> updater);

    // Buffer limit
    static constexpr size_t MAX_PENDING_SAMPLES = 960000;
    std::queue<Bytes> rx_data_queue_;
    mutable std::mutex rx_mutex_;

    // Statistics
    LoopbackStats stats_;
    mutable std::mutex stats_mutex_;

    // Callbacks
    DataCallback data_callback_;
    RawDataCallback raw_data_callback_;
    BurstGroupCallback burst_group_callback_;
    bool burst_transport_rx_enabled_ = true;  // unconditional — burst is THE OFDM file path (no env gate)
    DataSyncAcceptedCallback data_sync_accepted_callback_;
    StatusCallback status_callback_;
    PingReceivedCallback ping_received_callback_;

    // Adaptive modulation controller
    AdaptiveModeController adaptive_;

    // Audio filters
    FilterConfig filter_config_;
    std::unique_ptr<FIRFilter> tx_filter_;
    std::unique_ptr<FIRFilter> rx_filter_;

    // Half-duplex turnaround
    std::chrono::steady_clock::time_point last_rx_complete_time_;
    uint32_t turnaround_delay_ms_ = 200;

    // Carrier sense (listen-before-talk): the shared adaptive detector, fed RX
    // audio in feedAudio(). Same component the simulator/TNC stations use.
    //
    // RATIOMETRIC, level-independent calibration (no absolute RMS constants, so
    // it works at any HF noise level / AF-gain setting):
    //   - MEDIAN floor (percentile 0.50): tracks the typical noise, robust to
    //     the bursty excursions of real HF and to a transient signal (minority
    //     of a long window).
    //   - busy threshold = floor x 2.0 (+6 dB): dimensionless; rides over noise
    //     peaks and trips on any occupant >= ~6 dB above the floor.
    //   - signal kept out of the floor by the detector's RATIOMETRIC admission
    //     gate (samples > floor x multiplier are not learned as "noise") — no
    //     absolute ceiling needed, so estimate_ceiling stays disabled (0).
    //   - bootstrap admits any startup level (ceiling 2.0 ~ full scale):
    //     assume-idle-at-startup / listen-before-talk, so the floor seeds to
    //     whatever the real noise is regardless of absolute level.
    // Validated live (real_hf_loop OTASim bed, 2026-05-23) at multiple noise
    // levels; see docs/tools probe. Energy CCA is only consulted in OFDM mode
    // (carrierSenseActiveForTx()) — at MC-DPSK SNRs the peer is at/below the
    // noise floor and is not energy-detectable.
    ultra::audio::ChannelBusyDetector channel_busy_detector_{
        ultra::audio::ratiometricHfCarrierSenseConfig()};
    uint64_t cca_samples_since_log_ = 0;

    // Estimated CFO from peer (detected during connection, persists for connected mode)
    float peer_cfo_hz_ = 0.0f;

    // Helper methods
    void rebuildFilters();

    // Post-process TX samples (lead-in, filter, scale, stats). lead_in_ms/tail_ms < 0 → use the
    // configurable defaults (ULTRA_TX_LEADIN_MS / ULTRA_TX_TAIL_MS, default 150/50). The ACK path
    // passes a shorter lead-in (ACKs are tiny + low PA-thermal) — see transmitToneBurstAck.
    // lead_in_fill (optional): settling fill written over the END of the lead-in (key-up
    // airtime unchanged); see StreamingEncoder::settlingLeadInFill.
    std::vector<float> postProcessTx(const std::vector<float>& samples,
                                     int lead_in_ms = -1, int tail_ms = -1,
                                     const std::vector<float>* lead_in_fill = nullptr);
};

} // namespace gui
} // namespace ultra
