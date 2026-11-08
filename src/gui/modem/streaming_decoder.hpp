#pragma once

// StreamingDecoder - Unified RX decoder for all waveform types
//
// Clean sliding-window receiver design:
//   1. Circular buffer (bounded) vs growing vector (unbounded)
//   2. Sliding window search vs periodic full search
//   3. Correct IWaveform call sequence: reset(), detectSync(), setFrequencyOffset(), process()
//   4. Thread-safe with condition variable for blocking wait
//   5. PING detection via energy ratio after chirp
//   6. Sync-quality estimation from chirp correlation strength
//
// Thread model:
//   - Audio thread: feedAudio() - fast, just copies samples to buffer
//   - Decode thread: processBuffer() - does heavy work (chirp detection, LDPC)
//
// Usage:
//   StreamingDecoder decoder;
//   decoder.setMode(WaveformMode::MC_DPSK, false);  // Disconnected
//
//   // Audio thread:
//   decoder.feedAudio(samples, count);
//
//   // Decode thread:
//   while (running) {
//       decoder.processBuffer();  // Blocks until data available
//       while (decoder.hasFrame()) {
//           auto result = decoder.getFrame();
//           // Handle result
//       }
//   }
//
//   // Shutdown:
//   decoder.stop();  // Wakes decode thread to exit

#include "waveform/waveform_interface.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"
#include "ofdm/doppler_coherence_estimator.hpp"
#include "idle_noise_snr_estimator.hpp"
#include "ultra/dsp.hpp"
#include "waveform/waveform_factory.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/fec.hpp"
#include "fec/codec_factory.hpp"  // ICodec for FEC decoding
#include "fec/soft_combine.hpp"
#include "sync/frame_arrival_policy.hpp"
#include "sync/sync_controller.hpp"   // SyncController — sync/z state owner (refactor §7)
#include "sync/sync_ring_buffer.hpp"  // SyncRingBuffer — the shared audio ring (refactor §7 C3)
#include "sync/cfo_tracker.hpp"       // CFOTracker — the tracked-CFO state (refactor §7 C-CFO)
#include "frame_demodulator.hpp"      // FrameDemodulator — per-frame demod stage(s) (refactor §7 C-FD)
#include "frame_decoder.hpp"          // FrameDecoder — soft-bits→frame FEC decode (refactor §7 C-FDec)
#include "streaming_decode_policy.hpp"  // commandedGeometryEnabled / cadence constant
#include "protocol/waveform_selection.hpp"  // kRungIdx* canonical rung indices
#include <array>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <functional>
#include <chrono>
#include <optional>

namespace ultra {
namespace gui {

// Forward declarations
namespace v2 = protocol::v2;

// Decoder state machine for continuous correlation
// Real receivers run correlation continuously, not in batches
enum class DecoderState {
    SEARCHING,              // Running correlation on incoming samples
    SYNC_FOUND,             // Chirp detected, collecting frame samples
    DECODING,               // Have enough samples, decoding in progress
    BURST_ACCUMULATING,     // Burst marker detected, waiting for all 4 frames
    MCDPSK_BURST_CONTINUING, // Data-only MC-DPSK frames after one chirp/training
};

// Result of decoding a frame
struct DecodeResult {
    bool success = false;           // True if frame decoded successfully
    Bytes frame_data;               // Decoded frame payload
    v2::FrameType frame_type = v2::FrameType::PROBE;
    // True only when the decoder has physical proof that this classic frame
    // closes the peer's current transmission (a standalone frame at the
    // accepted full anchor, or a CRC-protected physical-burst-end marker).
    // This is deliberately distinct from decode success: the protocol uses it
    // to emit a causally safe reverse ACK without mistaking an arbitrary/timer
    // callback for a half-duplex turn boundary.
    bool physical_turn_complete = false;
    // A failed one-codeword (turbo) data frame whose own LTS pair proves a real, aligned
    // peer frame (LTS-pair chi above its noise law). LDPC frames show the same thing via
    // codewords_ok > 0; a turbo frame is all-or-nothing, so without this every fade
    // would read as a sync loss and force a full chirp re-anchor.
    bool frame_evidence = false;
    float snr_db = 0.0f;            // Consumer-facing routed value; see snr_source.
    SNRSource snr_source = SNRSource::NONE;
    float cfo_hz = 0.0f;            // Measured CFO
    int codewords_ok = 0;           // Number of successful LDPC decodes
    int codewords_failed = 0;       // Number of failed LDPC decodes
    bool is_ping = false;           // True if this is a PING (chirp-only) frame
    bool has_idle_in_band_snr_db = false;
    float idle_in_band_snr_db = 0.0f;       // Receiver passband/in-band idle SNR.
    bool has_ofdm_broadband_snr_db = false;
    float ofdm_broadband_snr_db = 0.0f;     // Historical field name; OFDM in-band SNR.
    bool has_evm_snr_db = false;
    float evm_snr_db = 0.0f;                 // Radio-agnostic decision-directed EVM SNR (Stage 1).
    size_t evm_carrier_count = 0;             // Fresh post-EQ decisions behind evm_snr_db.
    // #58 BUG-CONNECT-SNR-VARIANCE: both MC-DPSK in-band estimates, surfaced so
    // consumers/tests can compare spreads. training = ~170 ms preamble snapshot
    // (ONE fade state); data_aided = whole-frame differential estimate
    // (fade-averaged). snr_db routes data_aided when this frame decoded OK.
    bool has_mcdpsk_training_snr_db = false;
    float mcdpsk_training_snr_db = 0.0f;
    bool has_mcdpsk_data_aided_snr_db = false;
    float mcdpsk_data_aided_snr_db = 0.0f;
    // True when snr_db carries the data-aided (fade-averaged differential-EVM)
    // estimate rather than the training snapshot. The saturation lower-bound rule
    // in connectSelectionSnrDb() is only valid for the data-aided source.
    bool mcdpsk_snr_routed_data_aided = false;
    float ofdm_internal_snr_db = 0.0f;      // Demodulator internal LLR/channel-quality scale.
    float sync_quality_db = 0.0f;           // Chirp correlation confidence; not physical SNR.
    float lts_fading_index = 0.0f;  // Per-carrier LTS/pilot fading index
    float lts_timing_offset_samples = 0.0f;  // LTS phase-slope timing diagnostic.
    float pilot_frequency_cv = 0.0f;  // Data-pilot corroboration of static selectivity.
    float pilot_temporal_cv = 0.0f;
    float pilot_symbol_mean_cv = 0.0f;
    float sync_correlation = 0.0f;  // Light/full preamble sync correlation
    float lts_residual_cfo_hz = 0.0f;  // Residual CFO reported by OFDM waveform
    float doppler_coherence_score = 0.0f;  // |H|^2 autocorr @~1s lag; high=Good slow fading
    float doppler_hz = 0.0f;               // RMS Doppler spread (Hz), regression-derived
    bool doppler_coherence_valid = false;  // enough pooled data for a trustworthy verdict
    float ping_training_rms = 0.0f;
    float ping_data_rms = 0.0f;
    float ping_data_to_training_ratio = 0.0f;
    float ping_chirp_corr = 0.0f;
    float ping_gap_error_samples = 0.0f;
    bool ping_by_silence = false;
    bool ping_by_chirp_lock = false;
    bool ping_ldpc_attempted = false;
    bool ping_ldpc_decode_succeeded = false;
    bool ping_ldpc_magic_valid = false;
    bool has_partial_codewords = false;  // MC-DPSK only: CW0 parsed, frame incomplete.
    v2::PartialFrameCodewords partial_codewords;

    // ULTRA_ITERATIVE_CHEST (default OFF, src/ofdm/iterative_chest.hpp). The exact
    // air bytes this frame was transmitted as, recovered by re-encoding a frame
    // that passed LDPC parity on EVERY codeword AND the frame CRC. Empty unless
    // the knob is on and the frame is fully verified. Consumed by the burst path,
    // which re-modulates them to X and forms H = Y/X against the receive grid the
    // demodulator still holds for THIS frame.
    Bytes data_aided_air_bytes;

    // Failure-attribution diagnostics. Populated for OFDM fixed-codeword
    // decode attempts; unused by normal receive behavior.
    std::vector<uint8_t> cw_decoded;
    std::vector<int> cw_iterations;
    std::vector<int> cw_unsatisfied_checks;
    std::vector<float> cw_llr_abs_mean;
    std::vector<float> cw_llr_abs_min;
    std::vector<float> cw_llr_abs_p10;
    std::vector<float> cw_llr_abs_p50;
    std::vector<float> cw_llr_abs_p90;
    std::vector<uint8_t> cw_used_perturbation;
    std::vector<int> cw_harq_attempts;
};

// Decoder statistics for GUI display
struct DecoderStats {
    uint64_t frames_decoded = 0;
    uint64_t frames_failed = 0;
    uint64_t pings_received = 0;
    uint64_t buffer_overflows = 0;
    uint64_t overflow_samples_dropped = 0;
    uint64_t overflow_state_resets = 0;
    uint64_t current_unsearched_samples = 0;
    uint64_t peak_unsearched_samples = 0;
    float backlog_ms = 0.0f;
    float peak_backlog_ms = 0.0f;
    float buffer_fill_percent = 0.0f;
    float avg_decode_time_ms = 0.0f;
    uint64_t sync_recovery_attempts = 0;
    uint64_t sync_recovery_successes = 0;
    uint64_t sync_recovery_delta_p8 = 0;
    uint64_t sync_recovery_delta_m8 = 0;
    uint64_t sync_recovery_delta_p16 = 0;
    uint64_t sync_recovery_delta_m16 = 0;
    uint64_t sync_recovery_delta_p24 = 0;
    uint64_t sync_recovery_delta_m24 = 0;
    uint64_t sync_recovery_delta_p32 = 0;
    uint64_t sync_recovery_delta_m32 = 0;
};

// Callbacks for frame delivery
using FrameDecodedCallback = std::function<void(const DecodeResult&)>;
using StreamingPingCallback = std::function<void(float snr_db, float cfo_hz)>;
using DataSyncAcceptedCallback = std::function<void(float sync_correlation)>;
// §14.27 one-way burst transport: a decoded interleaved burst is delivered as a
// UNIT (the group either deinterleaves whole or fails whole). frames carries the
// serialized DATA frames of the group in order; all_ok is true only if every
// logical frame of the group decoded (a partial group is undecodable and must be
// whole-burst-resent). Only emitted when burst-transport RX is enabled; the
// SR-ARQ burst path keeps its per-frame delivery.
// quality (§14.36): decode headroom of the group in [0,1] (0 = failed) — drives
// the receiver's BER-driven rate recommendation. Worst-codeword iteration headroom.
// frame_mask (2026-05-29 channel-adaptive SR-ARQ): bit i = logical frame i of the group
// decoded OK. interleaved = the group's bytes were byte-interleaved. When interleaved,
// only all_ok is meaningful (a partial group is undecodable → whole-burst resend); when
// NOT interleaved, frame_mask is a true per-frame SACK and the sender resends only the
// 0-bit frames + refills the burst (the Good/AWGN SR-ARQ path). 16 bits end-to-end
// (2026-07-02, matches the tone-burst wire mask kPayloadFrameMaskBits).
// geometry_proven is true only when group_size came from a decoded BURST_HEADER
// descriptor or from a CRC-valid PHYSICAL_BURST_END DATA frame. A configured/stale
// fallback size is carried for accounting but is not transmitter-owned evidence.
using BurstGroupCallback =
    std::function<void(uint16_t group_seq, const std::vector<Bytes>& frames, bool all_ok,
                       float quality, uint16_t frame_mask, bool interleaved,
                       uint8_t group_size, bool geometry_proven)>;
// DESC-SWITCH (ULTRA_DESCRIPTOR_MODE_SWITCH Phase 1, docs/MODE_SWITCH_PIGGYBACK_
// DESIGN_2026_07_03.md §5.1 step 4b): fired when a consumed BURST_HEADER descriptor
// declares a mod/rate that DIFFERS from the decoder's current data mode (the demod
// itself already self-reconfigures via the deferred pending_descriptor_* channel).
// Lets the protocol layer follow the announced mode (window/timers/chunk capacity)
// without a MODE_CHANGE exchange. Only fired when the knob is ON (byte-identical OFF).
using DescriptorModeChangeCallback =
    std::function<void(Modulation mod, CodeRate rate, int cw_per_frame)>;

// StreamingDecoder - Unified RX decoder for all waveform types
class StreamingDecoder {
public:
    // Robust-Low MC-DPSK (2048 sps DBPSK) can produce 30s+ multi-CW data
    // frames; the streaming ring must hold the full frame plus scheduler slack.
    // Source of truth lives in SyncRingBuffer (§7 C3); aliased here for the public ctor
    // default arg + external refs (e.g. test_streaming_config).
    static constexpr size_t kDefaultBufferSamples = sync::SyncRingBuffer::kDefaultBufferSamples;
    static constexpr size_t kMinimumBufferSamples = sync::SyncRingBuffer::kMinimumBufferSamples;

    explicit StreamingDecoder(size_t buffer_capacity_samples = kDefaultBufferSamples);
    ~StreamingDecoder();

    // ========================================================================
    // AUDIO THREAD INTERFACE
    // ========================================================================

    // Feed audio samples into the decoder
    // Called from audio callback - must be fast (<1ms)
    // Samples are copied to internal circular buffer
    void feedAudio(const float* samples, size_t count);
    void feedAudio(const std::vector<float>& samples) { feedAudio(samples.data(), samples.size()); }

    // ========================================================================
    // DECODE THREAD INTERFACE
    // ========================================================================

    // Process buffered audio, detect chirps, decode frames
    // Blocks until data available (or timeout/shutdown)
    // Call this in a loop from decode thread
    void processBuffer();

    // Check if any decoded frames are available
    bool hasFrame() const;

    // Get the next decoded frame (removes from queue)
    // Returns empty result if no frames available
    DecodeResult getFrame();

    // ========================================================================
    // MODE CONTROL
    // ========================================================================

    // Set the waveform mode and connection state
    // Disconnected: always decode as MC-DPSK (PING/CONNECT)
    // Connected: decode using negotiated waveform
    void setMode(protocol::WaveformMode mode, bool connected);

    // Set MC-DPSK carrier count (recreates waveform if currently in MC-DPSK mode)
    void setMCDPSKCarriers(int num_carriers);

    // Set the full MC-DPSK PHY preset
    void setMCDPSKConfig(const MultiCarrierDPSKConfig& config);

    // Set OFDM config (recreates waveform with the given config)
    // Use this for NVIS mode (1024 FFT, 59 carriers) or custom OFDM settings
    void setOFDMConfig(const ModemConfig& config);

    // Atomically apply OFDM connected-mode settings to avoid transient
    // mode/config mismatches during handshake transitions.
    void setConnectedOFDMMode(protocol::WaveformMode mode,
                              const ModemConfig& config,
                              Modulation mod,
                              CodeRate rate);

    // Set data mode (modulation and code rate) for the waveform
    // Called when connection is established with negotiated settings
    void setDataMode(Modulation mod, CodeRate rate);

    // §14.36 live GUI display: the decoder's CURRENT rate/modulation, including
    // any mid-transfer descriptor-driven switch (applyDataModeUnlocked). Reads
    // are safe without a lock (single-word atomics-equivalent on assignment).
    CodeRate getCodeRate() const { return code_rate_; }
    Modulation getModulation() const { return current_modulation_; }

    // Set FEC codec type (for dynamic codec switching based on SNR)
    // Recreates the codec if type changes
    void setCodecType(fec::CodecType type);
    fec::CodecType getCodecType() const { return codec_type_; }

    void setChannelInterleave(bool enable) { use_channel_interleave_ = enable; }
    bool getChannelInterleave() const { return use_channel_interleave_; }

    void setCarrierMask(uint64_t active_mask);
    uint64_t getCarrierMask() const { return carrier_mask_; }
    void setCarrierLdpcInterleaver(bool enable);
    bool getCarrierLdpcInterleaver() const { return use_carrier_ldpc_interleaver_; }

    void setFixedFrameCodewords(int cw_count) {
        fixed_frame_codewords_ = v2::sanitizeFixedFrameCodewords(cw_count);
    }
    int getFixedFrameCodewords() const { return fixed_frame_codewords_; }
    void setFixedFrameHeaderDiscovery(bool enable) { fixed_frame_header_discovery_ = enable; }

    void setSoftCombineBuffer(fec::SoftCombineBuffer* buffer) { harq_buffer_ = buffer; }
    using HarqProvisionalContextCallback =
        std::function<std::optional<fec::SoftCombineBuffer::ProvisionalContext>()>;
    void setHarqProvisionalContextCallback(HarqProvisionalContextCallback cb) {
        harq_context_callback_ = std::move(cb);
    }

    // Burst-level long interleaver (N-frame groups)
    void setBurstInterleave(bool enable) { use_burst_interleave_ = enable; }
    bool getBurstInterleave() const { return use_burst_interleave_; }
    void setBurstInterleaveGroupSize(int size);
    // PHANTOM-FRAME fix (2026-07-04): BURST_HEADER consume path — the wire
    // descriptor group size always wins immediately; the config setter above is
    // the descriptor-missed FALLBACK, ignored while a descriptor-declared group
    // is mid-collection (see .cpp comment for the DESC-SWITCH phantom incident).
    void setBurstGroupSizeFromDescriptor(int size);
    int getBurstInterleaveGroupSize() const { return burst_group_size_; }

    // Last received burst descriptor (§14.17), for GUI display of the burst type.
    // The burst z-state lives in the SyncController (refactor §7.6); these forward.
    bool hasBurstDescriptor() const { return sync_controller_.have_burst_descriptor_; }
    v2::ControlFrame::BurstHeaderInfo lastBurstDescriptor() const {
        return sync_controller_.last_burst_descriptor_;
    }

    // Single RX source of truth for the active LDPC lifting Z: normally the value
    // announced in BURST_HEADER (payload[5]); narrowly, the temporary classic
    // fallback restores an earlier wire-learned Z81 tuple for one candidate.
    // 81 means n=1944; 27 means n=648 (legacy / cold-start / control).
    // NO env: the descriptor is the wire contract. A frame decoded before its
    // group's descriptor arrives correctly falls back to 27. Owned by the
    // SyncController (§7.6); replaces 5 scattered getenv("ULTRA_LDPC_Z") reads.
    int activeBurstLiftingZ() const {
        return testing_lifting_z_override_ != 0
            ? testing_lifting_z_override_
            : (classic_long_geometry_recovery_active_ ? 81
                                                       : sync_controller_.activeBurstLiftingZ());
    }

    // Get current mode
    protocol::WaveformMode getMode() const { return mode_; }
    bool isConnected() const { return connected_; }

    // Get detected bandwidth from dual-listen (valid after sync detection when disconnected)
    BandwidthMode getDetectedBandwidth() const { return detected_bandwidth_; }
    void resetDetectedBandwidth() { detected_bandwidth_ = BandwidthMode::WIDE; }

    // Get current configuration (for comparison with encoder)
    // Returns: mode, modulation, code_rate, carriers, interleaving settings
    struct DecoderConfig {
        protocol::WaveformMode mode = protocol::WaveformMode::MC_DPSK;
        Modulation modulation = Modulation::DQPSK;
        CodeRate code_rate = CodeRate::R1_4;
        int num_carriers = 59;
        int data_carriers = 53;
        int bits_per_symbol = 106;
        bool use_pilots = true;
        int pilot_spacing = 10;
        bool use_channel_interleave = false;
        bool use_frame_interleave = true;
    };
    DecoderConfig getConfig() const;

    // ========================================================================
    // CALLBACKS
    // ========================================================================

    void setFrameCallback(FrameDecodedCallback callback) { frame_callback_ = callback; }
    void setBurstGroupCallback(BurstGroupCallback callback) { burst_group_callback_ = callback; }
    // DESC-SWITCH Phase 1: mode-hop descriptor notification (see the typedef above).
    void setDescriptorModeChangeCallback(DescriptorModeChangeCallback callback) {
        descriptor_mode_change_callback_ = std::move(callback);
    }
    // Enable §14.27 burst-transport RX: finalizeBurstGroup emits the group as a
    // unit via the burst-group callback and suppresses per-frame queue delivery
    // (so the file group does not also double-process through the SR-ARQ path).
    void setBurstTransportRxEnabled(bool enabled) { burst_transport_rx_ = enabled; }
    void setPingCallback(StreamingPingCallback callback) { ping_callback_ = callback; }
    void setDataSyncAcceptedCallback(DataSyncAcceptedCallback callback) { data_sync_accepted_callback_ = callback; }
    // #70 stage 2: handshake-stage gate for ULTRA_ROBUST_IDLE_PING. TRUE when the
    // local station next expects a BARE-CHIRP control frame (idle->PING, probing
    // ->PONG); set FALSE while it expects a DATA control frame (connecting->
    // CONNECT_ACK) so a badly-FADED CONNECT_ACK (low LLR -> trips false-lock reject)
    // is NOT mis-PONGed (the IONOS #27 case). Default TRUE = robust-ping eligible.
    // TWO writer threads: the decode thread (onPongReceived -> sendFullConnect ->
    // CONNECTING, serialized in-line inside this PING's own emit, which is what
    // actually closes #27 on the initiator) and the GUI thread (protocol tick
    // CONNECT timeout). atomic/relaxed: a one-frame stale read is benign and the
    // gate only matters when the env knob is on.
    void setBareChirpExpected(bool v) {
        bare_chirp_expected_.store(v, std::memory_order_relaxed);
    }
    // DISCONNECTING is still a connected coherent-OFDM receive phase, but only
    // hardened control frames are legal.  The protocol/GUI thread writes this;
    // the decode thread reads it while sizing and routing an acquired frame.
    void setControlOnlyReceive(bool enabled) {
        control_only_receive_.store(enabled, std::memory_order_release);
    }
    bool controlOnlyReceiveForTesting() const {
        return control_only_receive_.load(std::memory_order_acquire);
    }
    void setLogPrefix(const std::string& prefix) {
        log_prefix_ = prefix;
        sync_controller_.setLogPrefix(prefix);  // warm-sync logs now emit from the controller
    }

    // BUG-BURST-STALE-GEOMETRY (2026-07-28): the canonical rung index THIS receiver
    // put on the wire in its last tone-burst ack. Under ULTRA_RX_RATE_AUTHORITY the
    // receiver IS the rate authority, so this ack is the sender's NEXT geometry —
    // we know it before the sender does. On a MISSED burst descriptor the decoder
    // slices with this rung instead of the latched one.
    //
    // Written on the ack-EMIT thread, which is the RX-decode thread for per-group
    // acks (modem_rx -> protocol -> app) but the GUI thread for tick()-driven
    // SACK/ack-repeat re-emits (SelectiveRepeatARQ::tick). Read on the decode
    // thread. ONE relaxed atomic word so mod and rate can never be torn apart —
    // deliberately NOT the plain-int pattern of setFixedFrameCodewords (which is an
    // existing latent race on the very variable this bug is about; do not add a
    // second one beside it).
    void setCommandedRungIndex(uint8_t idx) {
        // A NEW target clears the "sender declined it" evidence; a re-carried copy of
        // the SAME target must NOT (the receiver re-stamps its standing command on
        // every ACK, so clearing on every stamp would erase the one observation that
        // proves the sender is not obeying — see resolveCommandedGeometry).
        if (commanded_rung_idx_.exchange(idx, std::memory_order_relaxed) != idx) {
            commanded_rung_declined_.store(false, std::memory_order_relaxed);
        }
    }
    uint8_t commandedRungIndexForTesting() const {
        return commanded_rung_idx_.load(std::memory_order_relaxed);
    }
    bool commandedRungDeclinedForTesting() const {
        return commanded_rung_declined_.load(std::memory_order_relaxed);
    }
    void armAnchoredBurstBackstopForTesting(size_t arm_abs);
    bool anchoredBurstBackstopArmedForTesting() const;
    size_t anchoredBurstBackstopArmAbsForTesting() const;
    // Seed the learned rung->geometry table without a wire descriptor (tests only —
    // production populates it ONLY from decoded BURST_HEADER descriptors).
    void seedRungGeometryForTesting(Modulation mod, CodeRate rate, int cw, int lifting_z);
    // One synchronized snapshot so a lifecycle/concurrency regression can verify
    // that session-boundary clearing never exposes a torn cw/Z tuple.
    std::array<int, 2> learnedRungGeometryForTesting(Modulation mod,
                                                     CodeRate rate) const;
    uint64_t learnedRungGeometrySessionEpochForTesting() const;
    bool seedRungGeometryForSessionForTesting(Modulation mod, CodeRate rate,
                                              int cw, int lifting_z,
                                              uint64_t session_epoch);
    // Force the active LDPC lifting size without a wire descriptor (tests/tools
    // only). The override persists across reset(), matching a measure_ack_fer
    // session that reuses one configured decoder for many independent frames.
    // Production never calls this: its sole authority remains BURST_HEADER.
    void setLDPCLiftingZForTesting(int lifting_z);
    // Replay the adoption-evidence observation a decoded BURST_HEADER would make.
    void noteDescriptorRungForTesting(Modulation mod, CodeRate rate) {
        noteDescriptorRungObserved(mod, rate);
    }
    size_t burstMinBlockSamplesForTesting() const { return burst_min_block_; }
    uint64_t burstDataStartAbsForTesting() const { return burst_data_start_abs_; }
    // Guard-(A) observability: how many times a burst group-start frame arrived
    // TRUNCATED (the sizing requirement grew between the readiness check and the
    // decode) and was deferred instead of arming the group from a short slice.
    uint32_t truncatedBurstHoldsForTesting() const { return truncated_burst_holds_; }
    bool truncationHoldActiveForTesting() const {
        return truncation_hold_sync_pos_ != 0 || truncation_hold_frame_len_ != 0;
    }
    uint32_t commandedGeometryArmsForTesting() const { return commanded_geometry_arms_; }
    uint32_t classicLongGeometryArmsForTesting() const {
        return classic_long_geometry_arms_;
    }
    bool classicLongGeometryActiveForTesting() const {
        return classic_long_geometry_recovery_active_;
    }
    // Exercise only the eligibility predicate. The integration trial below this
    // hook still drives the real sync/control-first/data path; this hook keeps
    // negative correlation/provenance cases deterministic rather than depending
    // on a particular noisy LTS realization landing in a narrow correlation band.
    bool classicLongGeometryEligibleForTesting(
        float correlation, bool burst_latched,
        uint64_t prior_group_start_abs = 0,
        uint64_t anchor_abs = 0,
        uint64_t candidate_abs = 1);
    uint64_t commandedGeometryAirGateSpanAtArmForTesting() const {
        return commanded_geometry_air_gate_span_at_arm_;
    }
    DecoderState stateForTesting() const { return state_; }
    int pendingTotalCodewordsForTesting() const { return pending_total_cw_; }
    size_t waveformFrameSamplesForTesting(int cw_count) const {
        return waveform_
            ? static_cast<size_t>(waveform_->getMinSamplesForCWCount(cw_count))
            : 0u;
    }
    void expireBurstTimeoutForTesting() {
        burst_start_time_ = std::chrono::steady_clock::now() -
                            std::chrono::hours(1);
    }
    size_t burstErasureSoftBitsForTesting() const {
        return makeActiveBurstErasure().size();
    }
    void finalizeMalformedBurstForTesting(
        std::vector<std::vector<float>> physical_soft,
        int exact_physical_group_size) {
        burst_soft_buffer_ = std::move(physical_soft);
        burst_arm_provenance_ = BurstArmProvenance::DESCRIPTOR;
        finalizeBurstGroup(exact_physical_group_size);
        burst_soft_buffer_.clear();
        burst_predecoded_.clear();
        burst_metric_templates_.clear();
        descriptor_group_size_locked_ = false;
    }
    void finalizeGroupEvmForTesting(const std::vector<DecodeResult>& metrics,
                                    bool geometry_proven,
                                    int expected_physical_frames) {
        burst_metric_templates_ = metrics;
        finalizeGroupEvmSnr(geometry_proven, expected_physical_frames);
    }

    // ========================================================================
    // STATUS
    // ========================================================================

    // Get last chirp sync-quality score from the most recent chirp detection.
    float getLastSyncQualityDb() const { return last_snr_.load(); }

    // Get last measured CFO
    float getLastCFO() const { return cfo_tracker_.tracked(); }

    // Set known CFO (for testing or when CFO is known from other source)
    void setKnownCFO(float cfo_hz) { cfo_tracker_.store(cfo_hz); }

    // Outcome-side CFO contract probe: a CRC/LDPC-valid connected control-first
    // OFDM frame must restore this certificate after a 0/N group revoked it.
    bool warmCFOCertifiedForTesting() const { return cfo_tracker_.pilotSeeded(); }

    // Expect the next connected OFDM frame to carry full chirp+LTS preamble.
    // This bootstraps OFDM-specific timing after an MC-DPSK handshake.
    void expectFullOFDMAnchorOnce();
    void clearFullOFDMAnchorExpectation();
    bool expectsFullOFDMAnchorForTesting() const;

    // The connection layer has entered the negotiated OFDM mode, but the peer
    // has not yet proved that it received our CONNECT_ACK.  During this short
    // responder-side half-open interval a repeated CONNECT still rides the
    // fixed MC-DPSK control PHY.  Keep one bounded cross-mode recovery candidate
    // for a true expected dual-chirp anchor; clear it as soon as an authoritative
    // post-handshake frame arrives.
    void setHandshakeConfirmationPending(bool pending);

    // §14.36: setConnectedOFDMMode / descriptor rate changes are DEFERRED to the
    // safe top-of-processBuffer boundary (they rebuild waveform_ and must not race
    // the RX decode thread). Production flushes them on the next processBuffer()
    // cycle. Tests that assert config state synchronously (no audio thread running)
    // call this to apply pending changes immediately — single-threaded only.
    void applyPendingConfigForTesting();

    // Get last measured fading index (from per-carrier magnitude variance)
    // 0-1 range, > 0.4 indicates significant fading
    float getLastFadingIndex() const { return last_fading_index_.load(); }

    // Channel coherence-TIME (Doppler) discriminator: separates Good (slow fading, high
    // score) from Moderate (fast, low score) where fading_index is blind. Trust the score
    // only when getLastDopplerCoherenceValid() is true (enough pooled data); else fall back
    // to fading_index. See docs/CHANNEL_DISCRIMINATOR_DESIGN_2026_06_15.md.
    float getLastDopplerCoherenceScore() const { return last_doppler_coherence_score_.load(); }
    float getLastMeasuredDopplerHz() const { return last_doppler_hz_.load(); }
    bool getLastDopplerCoherenceValid() const { return last_doppler_coherence_valid_.load(); }

    // Get last residual-derived OFDM in-band SNR estimate. Returns false
    // until an OFDM LTS/pilot residual has landed.
    bool hasLastOFDMBroadbandSNREstimate() const {
        return last_ofdm_broadband_snr_db_valid_.load();
    }

    // F129: decoder-evidence RX activity (see last_rx_signal_ms_).
    bool rxSignalActiveWithin(int64_t within_ms) const {
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
        return (now - last_rx_signal_ms_.load()) <= within_ms;
    }
    // F143: raw stamp accessor — consumers comparing against an ARM time (repeat
    // cancel must count only evidence NEWER than the arm, not the decode of the
    // very group whose ack armed it).
    int64_t lastRxSignalMs() const { return last_rx_signal_ms_.load(); }
    void stampRxSignal() const {
        last_rx_signal_ms_.store(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }
    // F147: SUBSTANTIVE evidence — proof a peer transmission is genuinely
    // arriving (accepted sync, consumed descriptor, codewords decoding), as
    // opposed to the broad stamp above, which fires on every decode ATTEMPT
    // including false-lock rejects on idle noise. The ack-repeat cancel must
    // use THIS one: in F147 the repeat (armed to save a one-way-faded ack) was
    // canceled by the decoder rejecting noise candidates in a silent gap —
    // "inbound transmission in progress" with nothing on the air. The broad
    // stamp stays for listen-before-ACK, where conservative deferral is right.
    int64_t lastRxSubstantiveMs() const { return last_rx_substantive_ms_.load(); }
    bool hasLastPhysicalSnr() const { return last_physical_snr_valid_.load(); }
    float lastPhysicalSnrDb() const { return last_physical_snr_db_.load(); }
    // Handoff §5: distribution over the recent physical readings.
    // Returns readings-count; mean is linear-domain (fade-averaged), spread
    // is the dB standard deviation of the samples.
    size_t physicalSnrStats(float& mean_db, float& spread_db) const {
        std::lock_guard<std::mutex> lk(physical_ring_mutex_);
        const size_t n = std::min(physical_ring_count_, kPhysicalSnrRing);
        if (n == 0) { mean_db = 0.0f; spread_db = 0.0f; return 0; }
        double lin_sum = 0.0, db_sum = 0.0;
        for (size_t i = 0; i < n; ++i) {
            lin_sum += std::pow(10.0, physical_ring_db_[i] / 10.0);
            db_sum += physical_ring_db_[i];
        }
        mean_db = static_cast<float>(10.0 * std::log10(lin_sum / n));
        const double db_mean = db_sum / n;
        double var = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double d = physical_ring_db_[i] - db_mean;
            var += d * d;
        }
        spread_db = static_cast<float>(std::sqrt(var / n));
        return n;
    }
    void notePhysicalSnrSample(float db) const {
        std::lock_guard<std::mutex> lk(physical_ring_mutex_);
        physical_ring_db_[physical_ring_count_ % kPhysicalSnrRing] = db;
        ++physical_ring_count_;
    }
    uint64_t burstAirSamplesRemaining() {
        const uint64_t end = burst_air_end_abs_.load(std::memory_order_relaxed);
        if (end == 0) return 0;
        uint64_t fed;
        {
            std::lock_guard<std::mutex> lk(sync_controller_.ring_.buffer_mutex_);
            fed = sync_controller_.ring_.total_fed_;
        }
        return end > fed ? end - fed : 0;
    }
    const std::vector<float>& getLastGroupCarrierGammas() const {
        return last_group_carrier_gammas_;  // decode-thread only (group callback)
    }
    void setRealTimeAudio(bool v) { sync_controller_.real_time_audio_ = v; }
    void setAnchoredBurstNoGroupCallback(std::function<void(bool payload_seen)> cb) {
        anchored_burst_no_group_callback_ = std::move(cb);
    }
    // A marker-armed candidate was abandoned without trustworthy group geometry.
    // This is state-only provenance: consumers may cancel a pending experiment,
    // but must not synthesize an ACK, delivery outcome, or rate observation.
    void setBurstOutcomeUnknownCallback(std::function<void()> cb) {
        burst_outcome_unknown_callback_ = std::move(cb);
    }
    void stampRxSubstantive() const {
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
        last_rx_substantive_ms_.store(now);
        last_rx_signal_ms_.store(now);  // substantive implies signal
    }
    float getLastOFDMBroadbandSNREstimate() const {
        return last_ofdm_broadband_snr_db_.load();
    }

    // Radio-agnostic decision-directed EVM SNR (dB) — captured at decode time (same as
    // the broadband value) so it survives the per-burst demod reset. Nothing consumes it
    // yet — Stage 1 parallel source for tests/logging/Stage 2.
    bool hasLastEvmSnr() const {
        return last_evm_snr_db_valid_.load();
    }
    float getLastEvmSnrDb() const {
        return last_evm_snr_db_.load();
    }
    const std::vector<float>& getLastGroupEvmSnrFrames() const {
        return last_group_evm_snr_frames_;  // decode-thread/group-callback scope
    }
    const std::vector<size_t>& getLastGroupEvmCarrierCounts() const {
        return last_group_evm_carrier_counts_;  // decode-thread/group-callback scope
    }

    bool hasIdleNoiseSNREstimate() const { return idle_noise_snr_estimator_.hasEstimate(); }
    float getIdleNoiseSNREstimate() const { return idle_noise_snr_estimator_.snrDb(); }
    IdleNoiseSNREstimator::Snapshot getIdleNoiseSNRSnapshot() const {
        return idle_noise_snr_estimator_.snapshot();
    }

    // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET): latest per-burst RX level verdict
    // (protocol::connection_policy::RxLevelVerdict as int: 0=OK, 1=LOW, 2=CLIPPED)
    // and its measurement sequence number (bumps once per fresh per-group
    // measurement; consumers dedup on it). Lock-free.
    int getRxLevelVerdict() const { return rx_level_verdict_.load(std::memory_order_relaxed); }
    uint32_t getRxLevelVerdictSeq() const {
        return rx_level_verdict_seq_.load(std::memory_order_relaxed);
    }

    // Get buffer fill level (0-100%)
    float getBufferFillPercent() const;

    // Get decoder statistics
    DecoderStats getStats() const;

    struct FrameArrivalSnapshot {
        bool warm_sync_active = false;
        sync::frame_arrival_policy::WarmSyncPhase warm_sync_phase =
            sync::frame_arrival_policy::WarmSyncPhase::COLD;
        bool has_prediction = false;
        size_t next_expected_frame_sample = 0;
        float frame_arrival_confidence = 0.0f;
        int consecutive_sync_misses = 0;
        bool has_last_frame = false;
        size_t last_frame_start_sample = 0;
        size_t last_frame_end_sample = 0;
        bool has_last_arrival_error = false;
        int64_t last_arrival_error_samples = 0;
        size_t expected_frame_gap_samples = 0;
    };

    // Passive warm-sync timing state. The decoder derives this only from
    // successfully decoded OFDM frames; protocol code may set a known TX-turn
    // gap, but no ARQ coupling is required for contiguous OFDM bursts.
    FrameArrivalSnapshot getFrameArrivalSnapshot() const;
    void setExpectedFrameGapSamples(size_t samples);
    void seedExpectedFrameArrivalAfterSamples(size_t delay_samples,
                                              float confidence = 0.50f);

    // Get number of samples in buffer
    size_t samplesInBuffer() const;
    size_t bufferCapacitySamples() const { return sync_controller_.ring_.buffer_capacity_samples_; }

    // Check if waveform is synchronized
    bool isSynced() const;

    // Get constellation symbols for display
    std::vector<std::complex<float>> getConstellationSymbols() const;
    Modulation getConstellationModulation() const;

    // ========================================================================
    // LIFECYCLE
    // ========================================================================

    // Reset decoder state (clear buffer, reset waveform).
    // reset_doppler_coherence=false preserves the slow Doppler-coherence estimator's snapshot
    // pool across the reset — used by the pre-TX clearRxBuffer echo-clear, which fires every
    // half-duplex turnaround; wiping the disc there starves it below its 8-snapshot minimum
    // so it never validates on a real transfer (task #55).
    void reset(bool reset_doppler_coherence = true);

    // Signal shutdown - wakes processBuffer() to return
    void stop();

    // Clear shutdown flag (call after stop() + join() to reuse decoder)
    // Needed when switching from async decode thread to synchronous mode
    void clearShutdown() { shutdown_.store(false); }

    // Check if decoder is running (not stopped)
    bool isRunning() const { return !shutdown_.load(); }

    // ========================================================================
    // TONE-BURST ACK MONITOR (§15 step 4b)
    // ========================================================================
    //
    // Always-on detector running in parallel with the OFDM decode path. Fed
    // the same audio chunks as the OFDM buffer (call before the buffer
    // mutex_ guard so OFDM decode timing is unaffected). When a tone-burst
    // ACK is detected, the installed callback fires synchronously on the
    // audio thread.
    //
    // In step 4b the default callback is log-only; step 4d will replace it
    // with a hook into Connection::onGroupAck(group_seq, quality_q,
    // frame_mask) to make the tone-burst path the authoritative ACK source.
    using ToneBurstAckCallback =
        ultra::waveform::tone_burst_ack::ToneBurstAckCallback;
    using ToneBurstAckAcceptancePredicate =
        ultra::waveform::tone_burst_ack::ToneBurstAckAcceptancePredicate;
    using ToneBurstAckDetection =
        ultra::waveform::tone_burst_ack::ToneBurstAckDetection;

    // Replace the tone-burst detection callback. Default callback emits an
    // INFO log line summarizing the detection.
    void setToneBurstAckCallback(ToneBurstAckCallback cb);
    void setToneBurstAckAcceptancePredicate(
        ToneBurstAckAcceptancePredicate predicate) {
        tone_burst_monitor_.setAcceptancePredicate(std::move(predicate));
    }

    // For tests + diagnostics: how many tone-burst ACKs has the always-on
    // monitor decoded since construction or last reset.
    uint64_t toneBurstAcksDetected() const {
        return tone_burst_monitor_.detectionsEmitted();
    }
    uint64_t toneBurstAckCandidatesRejected() const {
        return tone_burst_monitor_.semanticCandidatesRejected();
    }

    // §15 step 4d-late: arm the tone-burst monitor for an expected ACK
    // arrival window. The Connection layer calls this right after queueing
    // a data burst on the TX path; the monitor then runs detection at a
    // tight cadence until either a successful decode fires or the window
    // elapses. Outside the armed window, detection idles and the audio
    // thread carries zero monitor CPU cost — the fix for the always-on
    // polling jitter that was visible in step 4d-iv.
    void armToneBurstMonitor(uint32_t window_ms) {
        const size_t window_samples =
            static_cast<size_t>(ultra::waveform::tone_burst_ack::kSampleRate) *
            window_ms / 1000u;
        tone_burst_monitor_.arm(window_samples);
    }

    void rearmToneBurstMonitor(uint32_t window_ms) {
        const size_t window_samples =
            static_cast<size_t>(ultra::waveform::tone_burst_ack::kSampleRate) *
            window_ms / 1000u;
        tone_burst_monitor_.rearm(window_samples);
    }

private:
    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    // §14.36: setDataMode body without acquiring buffer_mutex_. Callable from
    // contexts that already hold it (the BURST_HEADER intercept inside
    // processBuffer) so the receiver can switch rate from the descriptor's
    // declaration mid-transfer for per-block rate adaptation.
    void applyDataModeUnlocked(Modulation mod, CodeRate rate);

    // §14.36 crash-safe descriptor-driven rate switch: the descriptor intercept
    // sets these (under the local buffer_mutex_ scope it already holds) instead
    // of calling applyDataModeUnlocked() directly. The actual reconfigure (which
    // replaces modulator_/demodulator_/chirp_sync_ unique_ptrs) runs at the TOP
    // of the NEXT processBuffer call — a clean boundary where no decode state
    // is mid-flight. Without this defer, the configure() ran inside processBuffer
    // and the rest of the loop continued with internal state that the configure()
    // had just replaced, crashing in HilbertTransport::process (SIGSEGV at
    // /Users/mathieuvachon/.../ultra_gui-2026-05-28-000112.ips).
    bool pending_descriptor_rate_change_ = false;
    Modulation pending_descriptor_mod_ = Modulation::DQPSK;
    CodeRate pending_descriptor_rate_ = CodeRate::R1_4;
    void applyPendingDescriptorDataMode();  // called at top of processBuffer

    // End the descriptor-owned LDPC geometry as one invariant-preserving
    // operation.  The descriptor latch is the decoder-side source of truth,
    // while OFDMChirpWaveform and FrameDecoder own live sample/LLR geometry;
    // clearing only one leaves the next Z=27 control/BURST_HEADER sized or
    // deinterleaved against Z=81. The explicit test-only lifting override
    // remains authoritative. This synchronous form is DECODE-THREAD-ONLY.
    void clearActiveBurstDescriptorGeometry();
    void rebuildFrameDecoderInterleaverForLiftingZ(int lifting_z);

    // feedAudio(), reset(), and setMode() may run while the decode thread is
    // inside a waveform method. They clear the logical descriptor latch under
    // ring_.buffer_mutex_ but defer mutation of the live waveform/interleaver
    // until the safe top-of-processBuffer boundary.
    void deferActiveBurstDescriptorGeometryResetLocked();
    void applyPendingActiveBurstDescriptorGeometryReset();
    std::atomic<bool> pending_active_burst_geometry_reset_{false};

    // DESC-SWITCH Phase 1 knob (ULTRA_DESCRIPTOR_MODE_SWITCH, read once in the ctor —
    // lockstep with the Connection-side read; default OFF = byte-identical). Gates the
    // mode-hop warm-handoff demotion + the descriptor_mode_change_callback_ emit in the
    // BURST_HEADER intercept (streaming_ofdm_decode.cpp).
    bool descriptor_mode_switch_enabled_ = false;

    // §14.36 crash fix v3 (2026-05-28): setConnectedOFDMMode also replaces
    // waveform_ (constructs a new OFDMChirpWaveform). Same race as v2 — if
    // it runs while RX is mid-searchForSync (which releases buffer_mutex_
    // for the work), the destructor of the old waveform_ invalidates state
    // the RX thread is still using -> SIGSEGV in HilbertTransform::process
    // (ultra_gui-2026-05-28-033631.ips). Defer to safe boundary like v2.
    bool pending_connected_ofdm_change_ = false;
    protocol::WaveformMode pending_connected_ofdm_mode_ = protocol::WaveformMode::OFDM_CHIRP;
    ModemConfig pending_connected_ofdm_config_;
    Modulation pending_connected_ofdm_mod_ = Modulation::DQPSK;
    CodeRate pending_connected_ofdm_rate_ = CodeRate::R1_4;
    void applyPendingConnectedOFDMMode();  // called at top of processBuffer

    // Search for sync in recent samples
    void searchForSync();

    // A repeated fixed-profile CONNECT has the same dual-chirp timing anchor as
    // connected OFDM, but its 4-CW MC-DPSK body is much longer than the first
    // OFDM decode attempt.  Poll the retained raw ring asynchronously and make
    // one MC-DPSK attempt after the complete body is resident.  Returns true
    // only when a CONNECT was delivered (the caller must end this decode tick).
    struct PendingHandshakeMCDPSKConnect;
    bool tryDecodePendingHandshakeMCDPSKConnect();
    bool resetFalseOFDMStateAfterRecoveredConnect(
        const PendingHandshakeMCDPSKConnect& candidate,
        size_t frame_end_abs);

    // Check if we have enough samples to decode
    void checkIfReadyToDecode();

    // Decode the current frame
    void decodeCurrentFrame();

    int expectedOFDMCodewordsForSamples(size_t sample_count) const;
    size_t getOFDMControlFrameSamplesForCurrentMode() const;
    bool processWaveformForCodewords(SampleSpan samples, int expected_codewords);

    // Chirp correlation confidence score, historically dB-scaled but not physical SNR.
    float chirpSyncQualityDb(float correlation, float noise_floor);

    // Decode soft bits into frame data
    DecodeResult decodeFrame(const std::vector<float>& soft_bits, float snr, float cfo);

    // Cache latest non-empty constellation snapshot so GUI can render
    // even when control-path profile switching temporarily clears waveform state.
    void captureConstellationSnapshot();

    // MC-DPSK specific decode (simple sequential, no frame interleaving)
    DecodeResult decodeMCDPSKFrame(const std::vector<float>& soft_bits,
                                    CodeRate rate, size_t bytes_per_cw,
                                    float snr, float cfo);

    // CFO pre-correction: remove CFO from real passband samples before demodulation.
    // Uses Hilbert transform → analytic signal → complex rotation → real part.
    // After pre-correction, waveform should be told CFO=0.
    // Returns the CFO value used for pre-correction (for feedback adjustment).
    // channel_evidence_ok: the caller's REAL decode verdict for this frame, used ONLY to
    // admit channel-statistics evidence (the Doppler-coherence snapshot). Needed because the
    // BURST path calls this with a freshly default-constructed metrics TEMPLATE whose
    // .success is false by construction (the LDPC verdict does not exist yet), which starved
    // the coherence estimator to zero snapshots on the only path that carries file data.
    // Defaults false so unchanged call sites keep exactly today's behaviour.
    void populateDecodeMetrics(DecodeResult& result, bool is_ofdm,
                               float residual_cfo_hz,
                               bool channel_evidence_ok = false) const;
    void observeIdleNoiseCandidate(const float* samples, size_t count);
    void resetFrameArrivalTrackingLocked();
    void noteFrameArrivalSuccess(size_t frame_start_abs, size_t frame_end_abs);
    void noteFrameArrivalSuccessLocked(size_t frame_start_abs, size_t frame_end_abs);
    void noteFrameArrivalSyncMissLocked();

    // Ring/absolute sample helpers moved to SyncRingBuffer (sync_controller_.ring_.*); call only
    // while sync_controller_.ring_.buffer_mutex_ is held.

    // Burst interleave accumulation
    enum class BurstFrameResult {
        SUCCESS,    // Soft bits appended; may be demodulated data or an erasure block
        PHYSICAL_TAIL, // CRC-valid DATA says this is the exact final physical member
        WAITING,    // Not enough samples yet — caller should return and wait
        FAILED,     // Unrecoverable alignment/state failure — abort group
    };
    struct BurstPhysicalDiag {
        size_t abs_start_sample = 0;
        float rms = 0.0f;
        float mean_abs_llr = 0.0f;
        float near_zero_fraction = 1.0f;
        float pre_cfo_hz = 0.0f;
        float residual_cfo_hz = 0.0f;
        float accepted_cfo_hz = 0.0f;
        float fading_index = 0.0f;
        float lts_signal_power = 0.0f;
        float lts_channel_magnitude = 0.0f;
        float timing_offset_samples = 0.0f;
        bool erasure = false;
        bool process_ok = false;
    };
    void accumulateBurstFrames();
    BurstFrameResult tryDemodulateNextBurstFrame();
    // exact_physical_group_size is used only when a CRC-protected DATA tail proves
    // the wire ended before stale fallback geometry. Zero keeps descriptor/configured
    // geometry (including late-join head-erasure reconstruction).
    void finalizeBurstGroup(int exact_physical_group_size = 0);
    // Every synthetic physical-frame erasure must use the descriptor-owned lifting Z.
    // The legacy BurstInterleaver default is Z27/81 bytes per CW; relying on it inside
    // a cw4/Z81 group creates 2592 LLRs where deinterleave requires 7776.
    std::vector<float> makeActiveBurstErasure() const;
    // LATE-JOIN (ULTRA_DESC_ARMED_ACCUM, docs/DESC_ARMED_ACCUMULATION_DESIGN_2026_07_05.md):
    // arm accumulation from a mid-group member sync when the group HEAD died (the
    // BUG-BURST-HEADNULL-DROP recovery). Returns false when a full data frame is not
    // yet in the ring (caller keeps the legacy drop; the next member retries).
    bool lateJoinBurstAccumulation(size_t frame_sync_abs);
    // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET): derive the per-burst RX level
    // verdict (OK/LOW/CLIPPED) from the group's kept-data-frame RMS/peak vs the
    // idle chain-noise floor. Called at the top of finalizeBurstGroup (fresh
    // measurement per completed group) — the burst-group callback chain then
    // carries it to the Connection BEFORE this group's tone-burst ACK is emitted.
    void computeBurstLevelVerdict();
    void beginBurstDiagnosticsGroup(size_t abs_start_sample,
                                    const std::vector<float>& soft_bits,
                                    float rms,
                                    float pre_cfo_hz,
                                    float residual_cfo_hz,
                                    float accepted_cfo_hz);
    void appendBurstPhysicalDiagnostics(size_t abs_start_sample,
                                        const std::vector<float>& soft_bits,
                                        float rms,
                                        float pre_cfo_hz,
                                        float residual_cfo_hz,
                                        float accepted_cfo_hz,
                                        bool erasure,
                                        bool process_ok);
    void logBurstDiagnosticsAbort(const char* reason, size_t collected_frames);
    void clearBurstDiagnostics();
    void startMCDPSKBurstContinuation(size_t next_pos, size_t next_abs,
                                      float snr_db, float cfo_hz);
    void continueMCDPSKBurst();
    void finishMCDPSKBurstContinuation(size_t search_pos, size_t search_abs);

    // ========================================================================
    // STATE
    // ========================================================================

    // The circular audio ring + its cursors/floor/noise + buffer_mutex_/data_cv_ are owned by
    // SyncRingBuffer, which now lives INSIDE sync_controller_ (§7 C3 Phase 2). Reached as
    // sync_controller_.ring_.* — the controller owns the buffer it searches.

    // Continuous correlation state machine (like real receivers)
    DecoderState state_ = DecoderState::SEARCHING;
    size_t sync_position_ = 0;        // Buffer position where sync was found
    // A dual-chirp detector may prove the chirps before the following training
    // samples have reached the live ring. Preserve the exact searched audio and
    // replay that deterministic detector window after the announced training
    // position arrives. This avoids both losing the candidate and ever parking
    // sync_position_ beyond total_fed_. Protected by ring_.buffer_mutex_.
    struct DeferredFutureSyncSearch {
        bool valid = false;
        uint32_t generation = 0;
        size_t wait_until_abs = 0;
        size_t search_start_abs = 0;
        std::vector<float> search_buffer;
        bool used_warm_timed_window = false;
        bool used_warm_narrow_window = false;
        size_t warm_narrow_end_abs = 0;
        size_t warm_narrow_candidate_span_samples = 0;
    } deferred_future_sync_;

    // BUG-HALFOPEN-CONNECT: CONNECT and the first connected OFDM burst share the
    // full dual-chirp timing anchor.  If CONNECT_ACK was lost, an OFDM-only
    // responder otherwise accepts the repeated CONNECT chirp as OFDM, fails its
    // body as a synthetic 0/N group, and never reaches Connection's duplicate-
    // CONNECT ACK replay.  Protected by ring_.buffer_mutex_; the pending flag is
    // also written by the protocol/GUI thread.
    static constexpr size_t kPhysicalSnrRing = 8;

    struct HalfOpenMeasurementSnapshot {
        // A repeated MC-DPSK CONNECT can spend several seconds in the false
        // OFDM path before its complete body is resident.  Failed OFDM member
        // attempts still update physical-SNR/display atomics. Preserve the
        // state at the START of the responder's half-open interval (before even
        // weak OFDM fallbacks can run) and restore it only when a later anchor
        // is proven to be CONNECT. A genuine first OFDM frame confirms the
        // handshake and discards this snapshot without restoring it.
        bool valid = false;
        std::array<float, kPhysicalSnrRing> physical_ring_db{};
        size_t physical_ring_count = 0;
        bool ofdm_snr_valid = false;
        float ofdm_snr_db = 0.0f;
        bool evm_snr_valid = false;
        float evm_snr_db = 0.0f;
        float fading_index = 0.0f;
    } half_open_measurement_snapshot_;

    struct PendingHandshakeMCDPSKConnect {
        bool valid = false;
        uint32_t generation = 0;
        size_t training_abs = 0;
        size_t required_samples = 0;
        float cfo_hz = 0.0f;
        float correlation = 0.0f;
        MultiCarrierDPSKConfig config;
        HalfOpenMeasurementSnapshot measurements;
    } pending_handshake_mcdpsk_connect_;
    std::atomic<bool> handshake_confirmation_pending_{false};
    size_t samples_since_sync_ = 0;   // How many samples collected since sync
    float sync_cfo_ = 0.0f;           // CFO from sync detection
    float sync_snr_ = 0.0f;           // Chirp sync-quality score
    float sync_correlation_ = 0.0f;   // LTS/light-sync confidence for current frame
    // In-band RMS of the current frame's silent inter-chirp gap — the frame's
    // own BURST-TIME noise reference for the PING payload-absence gate.
    // 0 = unavailable (LTS/warm sync, single chirp). Set at sync-found,
    // consumed by the same frame's decode on the same thread.
    float sync_noise_ref_rms_ = 0.0f;
    // PHYSICAL in-band channel SNR from the last chirp-synced MC-DPSK frame:
    // 10log10((P_train - P_noise)/P_noise), training-span power vs the
    // burst-time inter-chirp noise ref. A pure POWER ratio — no phase model —
    // so it reads the CHANNEL's SNR where the coherence-weighted
    // mcdpsk_in_band estimator reads the demod-usable EFFECTIVE SNR (CFO/
    // clock/jitter wander lands in its residual: rig WGN@10 reads ~9.3 phys
    // vs ~6.5 effective; sim reads both ~10). Display/telemetry: BOTH numbers;
    // rate selection stays on effective (#74 — the demod lives there).
    mutable std::atomic<float> last_physical_snr_db_{0.0f};
    mutable std::atomic<bool> last_physical_snr_valid_{false};
    // Handoff §5: fading channel SNR is a DISTRIBUTION, not a number — each
    // per-frame physical reading samples one fade state. Keep a short ring
    // and report mean ± spread: mean in the LINEAR power domain (the honest
    // fade average — dB-averaging under-reads by the Jensen penalty), spread
    // as the dB standard deviation. ~8 readings ≈ 15-40 s of frames ≈ several
    // coherence times on Good.
    mutable std::mutex physical_ring_mutex_;
    mutable std::array<float, kPhysicalSnrRing> physical_ring_db_{};
    mutable size_t physical_ring_count_ = 0;
    float sync_gap_error_samples_ = 0.0f; // Dual-chirp timing error for current frame
    size_t last_decoded_sync_pos_ = SIZE_MAX;  // Last successfully decoded sync position (to prevent duplicates)
    bool sync_from_warm_timed_window_ = false;
    bool sync_from_full_anchor_fallback_ = false;
    // Lever #3 (ULTRA_CRATER_REANCHOR_HOLD): consecutive 0-CW crater re-anchor
    // triggers since the last decoded frame. The sync layer forces a full
    // chirp+LTS re-anchor on the FIRST crater; the RATE controller learned long ago
    // (F122 two-crater rule) that a single crater is a fade the ARQ absorbs, not
    // evidence — this counter gives the SYNC re-anchor the same discipline: hold warm
    // sync through the first crater, re-anchor only on the second consecutive one.
    int crater_reanchor_streak_ = 0;
    // warm_sync_phase_ + last_frame_* relocated into sync_controller_ (§7.4 shell-move A1,
    // 2026-05-31); accessed as sync_controller_.<field> until A2 moves the transition logic in.

    // The single owner of OFDM sync + burst-z state (refactor §7). Sync state is
    // being migrated into it member-by-member (shell-move §7.5#1); already holds the
    // cadence gap. Eventually owns the SyncMode, prediction, confidence, misses,
    // last_cfo, and the burst declared-z.
    sync::SyncController sync_controller_;
    // §7 C-CFO-1: the tracked RX carrier-frequency-offset, relocated out of SyncController (it was
    // last_cfo_). The chirp/LTS/pilot estimators feed it; acquisition reads it as the known CFO and
    // the per-frame demod feedback stores the pilot-corrected value back (the feedback invariant).
    sync::CFOTracker cfo_tracker_;
    // §7 C-FD-1: the per-frame demod stage (CFO pre-correction; grows to own the demod orchestration).
    FrameDemodulator frame_demodulator_;
    // §7 C-FDec-1: the FEC decode stage. Owns the decode primitives (codec + channel interleaver);
    // grows to own decodeFrame/decodeMCDPSKFrame. Distinct concern from FrameDemodulator.
    FrameDecoder frame_decoder_;

    // Reset generation counter - incremented on reset(), checked after slow operations
    // to detect if state was reset mid-operation (e.g., during correlation)
    std::atomic<uint32_t> reset_generation_{0};

    // Audio-activity instrumentation: independent of chirp-search, observes
    // RMS transitions in the incoming sample stream. Each high-RMS arrival is
    // logged so post-run analysis can compare audio events to sync events.
    std::atomic<bool> audio_activity_{false};
    std::atomic<uint64_t> audio_activity_events_{0};

    // RX operating-level AGC (ULTRA_RX_AGC, default off). A SLOW, AMPLIFY-ONLY normalizer that
    // raises a low received operating level toward the modem reference so the absolute-amplitude
    // thresholds (burst erasure gate, sync RMS floor, CCA quiet gate) work regardless of gain
    // staging / channel attenuation. Tracks the SIGNAL level over seconds (gated on activity so it
    // never chases idle noise or fade nulls), engages only below a deadband (so a normally-leveled
    // signal — every TX-normalized sim run — is an exact no-op), is slewed slowly so the gain never
    // moves within a burst (which would flatten fading / corrupt the constellation), and only ever
    // amplifies (a hot signal already clears every low threshold). Applied to the ring-buffer path
    // only; the §15 tone-burst ACK monitor stays on raw audio. Single audio thread → no lock needed.
    float agc_level_est_ = 0.5f;    // smoothed active-signal broadband RMS. Init HIGH so the
                                    // amplify-only gain never engages until the EMA has converged
                                    // DOWN and confirmed a genuinely low operating level (a normal-
                                    // level sim run stays above the engage point → exact no-op; a
                                    // transient noise blip can't trigger spurious amplification)
    float agc_gain_ = 1.0f;         // current applied gain (slewed; always >= 1)
    uint64_t agc_log_counter_ = 0;  // throttle the engaged-gain log

    // Active waveform for demodulation (handles its own sync internally)
    WaveformFactory waveform_factory_;
    std::unique_ptr<IWaveform> waveform_;

    // BUG-DISCONNECT-WAVEFORM-SWAP-SIGSEGV (2026-07-29). setMode() runs on the
    // protocol/GUI thread and replaces waveform_ while the RX decode thread may be
    // executing INSIDE that object — detectFullAnchorFallback (sync_controller.cpp:793)
    // calls waveform->detectDataSync() through a raw IWaveform* and does NOT hold
    // ring_.buffer_mutex_, so the mutex setMode takes protects the ring buffer, not this
    // pointer. Destroying it there is a use-after-free that null-derefs in
    // HilbertTransform::process.
    //
    // Retired objects park here instead of being freed at the swap, and are released at
    // the top of processBuffer() on the DECODE thread, where no waveform call can be in
    // flight. Swap semantics are unchanged — only the free is postponed. Bounded: at most
    // one entry per mode change, cleared on the next decode iteration.
    std::vector<std::unique_ptr<IWaveform>> retired_waveforms_;
    protocol::WaveformMode mode_ = protocol::WaveformMode::MC_DPSK;
    bool connected_ = false;
    // #70: see setBareChirpExpected. Default TRUE so an idle receiver (the common
    // case) is always robust-ping eligible; the App flips it FALSE during CONNECTING.
    std::atomic<bool> bare_chirp_expected_{true};

    // Dual-listen: narrowband waveform for detecting narrowband chirps when disconnected
    // Lazy-initialized on first search to avoid startup cost
    std::unique_ptr<IWaveform> narrow_waveform_;
    bool narrow_waveform_initialized_ = false;
    BandwidthMode detected_bandwidth_ = BandwidthMode::WIDE;
    int mc_dpsk_carriers_ = 8;  // MC-DPSK carrier count (default 8)
    MultiCarrierDPSKConfig mc_dpsk_config_ = mc_dpsk_presets::level8();
    int ofdm_carriers_ = 30;    // OFDM carrier count (default 30 for standard mode)
    int ofdm_data_carriers_ = 30;  // Data carriers after pilot allocation (for interleaver)
    Modulation current_modulation_ = Modulation::DQPSK;  // Current modulation for interleaver
    CodeRate code_rate_ = CodeRate::R1_4;  // Code rate for FEC decode
    fec::CodecType codec_type_ = fec::CodecType::LDPC;  // FEC codec type
    size_t mode_switch_write_pos_ = 0;  // write_pos at mode switch (skip old data)

    // Interleaver + FEC codec moved into frame_demodulator_ (§7 C-FD-2a) — accessed as
    // frame_demodulator_.interleaver_ / .codec_.
    bool use_channel_interleave_ = true;
    uint64_t carrier_mask_ = UINT64_MAX;
    bool use_carrier_ldpc_interleaver_ = false;
    // ULTRA_ITERATIVE_CHEST (default OFF). Latched once per burst group from the
    // environment — deliberately NOT a function-local `static const`, so a test can
    // toggle the knob in-process. Read once per group, never on a hot path.
    bool data_aided_chest_enabled_ = false;
    int fixed_frame_codewords_ = v2::kDefaultFixedFrameCodewords;
    bool fixed_frame_header_discovery_ = false;

    // Burst descriptor / z-state (§14.17) now lives in sync_controller_ (refactor
    // §7.6): have_burst_descriptor_ / last_burst_descriptor_ / activeBurstLiftingZ().
    // §14.27: group_seq of the in-flight burst (from the descriptor frame header seq).
    // Burst-transport RX group-as-unit delivery is UNCONDITIONAL — burst is THE
    // OFDM-wideband file path (2026-06-02; ULTRA_BURST_TRANSPORT gate removed). The
    // default is true so every StreamingDecoder owner (GUI ModemEngine, raw ultra_tnc/
    // measure_ack_fer) gets it without a separate enable call. The field + setter are
    // slated to fold away with the legacy windowed-file routing (R1 deletion).
    uint16_t last_burst_group_seq_ = 0;
    // Monotonic RX-side counter of DELIVERED burst groups, for logging only. The
    // unified-seq path ignores the descriptor's group_seq (connection.cpp does
    // `(void)group_seq`), so last_burst_group_seq_ stays 0 for the whole transfer and
    // every group logs as "group_seq=0" — which reads as a WEDGED transfer during a
    // live debug (it cost real time on 2026-07-25). This ordinal makes consecutive
    // groups distinguishable without changing any behaviour.
    uint32_t burst_group_ordinal_ = 0;
    bool burst_transport_rx_ = true;

    fec::SoftCombineBuffer* harq_buffer_ = nullptr;  // Non-owning; Connection owns lifecycle.
    // Turbo whole-frame HARQ same-reception guard: the timing-retry path re-demodulates
    // and re-decodes the SAME reception within milliseconds; combining it again would
    // double-count one noise realization. A genuine ARQ retransmission of the same seq
    // arrives a full turnaround cycle later (>= ~8 s at OFDM rates), so a key seen within
    // kTurboHarqSameReceptionMs is treated as the same reception.
    fec::SoftCombineBuffer::Key turbo_harq_last_key_{};
    std::chrono::steady_clock::time_point turbo_harq_last_time_{};
    HarqProvisionalContextCallback harq_context_callback_;

    // Decoded frame queue
    std::queue<DecodeResult> frame_queue_;
    mutable std::mutex queue_mutex_;

    // Callbacks
    FrameDecodedCallback frame_callback_;
    BurstGroupCallback burst_group_callback_;
    DescriptorModeChangeCallback descriptor_mode_change_callback_;  // DESC-SWITCH Phase 1
    StreamingPingCallback ping_callback_;
    DataSyncAcceptedCallback data_sync_accepted_callback_;

    // Statistics
    DecoderStats stats_;
    mutable std::mutex stats_mutex_;

    // Status (atomic for lock-free read from GUI)
    mutable std::atomic<float> last_snr_{0.0f};
    mutable std::atomic<bool> last_ofdm_broadband_snr_db_valid_{false};
    mutable std::atomic<bool> last_evm_snr_db_valid_{false};
    mutable std::atomic<float> last_evm_snr_db_{0.0f};
    // RX-SIGNAL ACTIVITY (F129): steady-clock ms of the last decoder EVIDENCE of an
    // incoming transmission (sync detection / per-frame metrics). The F129 census
    // proved the energy CCA lies in both directions on the rig (threshold learns
    // burst body as floor -> idle DURING bursts; learns AGC dips as floor -> busy
    // on ambient) while the DECODER had already synced before every self-TX
    // collision. Level-independent processing-gain evidence; consumers ask
    // rxSignalActiveWithin(~1600ms) — one frame interval plus slack.
    mutable std::atomic<int64_t> last_rx_signal_ms_{-1000000};
    mutable std::atomic<int64_t> last_rx_substantive_ms_{-1000000};  // F147
    // F147: the current lock came from the connected full-anchor search the
    // receiver ARMED (a sender resend was expected). The pre-LDPC false-lock
    // LLR gate must hold this lock instead of bouncing to re-search: rejecting
    // a genuine armed anchor forfeits the whole group AND its ack (F147: two
    // 8-frame bursts + 40 s lost after corr=0.76 anchors were rejected on
    // faded first-frame LLRs).
    bool last_sync_expected_full_anchor_ = false;
    // F165 ANCHORED-BURST ACK BACKSTOP: an ACCEPTED expected full anchor whose
    // burst produces NO group (descriptor unreadable in a deep fade) leaves the
    // sender ack-starved — no group boundary, no endGroupReceiveAndAck, pure
    // RTO dead-air, no crater verdict, rung pins. Arm at the accept; disarm on
    // BURST_HEADER consume (the standard group/ack path owns it from there);
    // fire ONCE when the sample clock passes the max group window with nothing
    // framed — the Connection then emits a re-confirm ack + crater verdict.
    bool anchored_burst_backstop_armed_ = false;
    size_t anchored_burst_backstop_arm_abs_ = 0;
    // Payload decoded through the classic fallback while the descriptor/group
    // callback was missing. The timer still owes a cumulative async SACK if the
    // physical tail is lost, but this evidence forbids fabricating a 0/N crater.
    bool anchored_burst_payload_seen_ = false;
    std::function<void(bool payload_seen)> anchored_burst_no_group_callback_;
    std::function<void()> burst_outcome_unknown_callback_;
    mutable std::atomic<float> last_ofdm_broadband_snr_db_{0.0f};
    IdleNoiseSNREstimator idle_noise_snr_estimator_;
    // Software-ALC RX level verdict (protocol::connection_policy::RxLevelVerdict as
    // int) + a monotonically increasing measurement sequence so consumers can tell a
    // FRESH per-burst measurement from a stale re-read (e.g. a timed-out group's
    // callback re-feeding the previous verdict must not extend a LOW streak).
    std::atomic<int> rx_level_verdict_{0};
    std::atomic<uint32_t> rx_level_verdict_seq_{0};
    std::atomic<float> last_fading_index_{0.0f};
    mutable std::atomic<float> last_doppler_coherence_score_{0.0f};
    mutable std::atomic<float> last_doppler_hz_{0.0f};
    mutable std::atomic<bool> last_doppler_coherence_valid_{false};
    // Good/Moderate discriminator, HOSTED HERE (persists across the per-group OFDM demodulator
    // recreation that burst transport performs). Fed one per-frame |H|^2 snapshot from the
    // OFDM LTS channel magnitude in populateDecodeMetrics. See the design doc.
    mutable DopplerCoherenceEstimator doppler_coherence_;
    // (§7 C-FD-1: pre_correction_cfo_ moved into frame_demodulator_ — read via preCorrectionCfo().)
    uint64_t overflow_events_ = 0;

    // Constellation cache (protected by buffer_mutex_)
    mutable std::vector<std::complex<float>> constellation_cache_;
    mutable std::chrono::steady_clock::time_point constellation_cache_time_{};
    static constexpr int CONSTELLATION_CACHE_HOLD_MS = 1500;

    // Lifecycle
    std::atomic<bool> shutdown_{false};
    bool new_data_available_ = false;  // Flag to wake decode thread immediately

    // Logging
    std::string log_prefix_ = "StreamingDecoder";
    int feed_iter_ = 0;         // Feed counter (per-instance)

    // Burst mode continuation (OFDM only)
    // After decoding a frame in connected OFDM mode, check for next block at known position
    int burst_blocks_decoded_ = 0;     // Blocks decoded in current burst
    static constexpr int MAX_BURST_BLOCKS = 8;  // Safety limit

    // Burst interleave accumulation state (valid only in BURST_ACCUMULATING)
    bool use_burst_interleave_ = false;
    // BUG-BURST-HEADNULL-DROP observability: sync-accepted frames consumed by the
    // mid-burst re-search with no decode attempt (group head nulled -> accumulation
    // never armed). Monotonic since construction; [HEADNULL] log per event.
    uint32_t headnull_resync_drop_count_ = 0;
    // LATE-JOIN state (lateJoinBurstAccumulation): the group was armed from a
    // mid-group member (head died) — finalize tail-anchors with leading erasures;
    // the member clock drives the group-end inference in accumulateBurstFrames.
    bool late_join_head_missing_ = false;
    std::chrono::steady_clock::time_point late_join_last_frame_time_{};
    enum class BurstArmProvenance : uint8_t {
        NONE,
        DESCRIPTOR,
        COMMANDED_GEOMETRY,
        UNPROVEN_MARKER,
        DESCRIPTOR_LATE_JOIN,
    };
    BurstArmProvenance burst_arm_provenance_ = BurstArmProvenance::NONE;
    size_t burst_substantive_members_ = 0;
    uint64_t burst_unproven_start_abs_ = 0;
    // A sign-only negated-LTS decision can classify a standalone repair/noise as a
    // group head. Likewise, commanded geometry can restore mod/rate/cw/Z but cannot
    // restore physical N. Process-valid soft grids at a guessed stride are not
    // transmitter-owned geometry; only a decoded descriptor or CRC-valid physical
    // tail may authorize a group outcome/reverse transmission.
    bool abandonUnprovenGeometryGroup(const char* reason);
    // HARQ provisional keys (2026-07-01, restricted design — fable_analysis/09 §3.4):
    // when a burst logical frame's CW0 peek fails, key its soft bits by the
    // POSITION-PREDICTED seq (receiver's ARQ mirror, pulled once per group via
    // harq_context_callback_) so the resend can chase-combine. The default-off
    // experiment is restricted to descriptor-proven QPSK R3/4 or the measured
    // 8PSK R2/3 cw4/Z81 tuple and non-interleaved groups. Tail headers may
    // validate the prediction but never use a provisional key. Known warm
    // cadence, one current-group identity match, prediction consistency and
    // session context remain mandatory; late-join and timeout groups are excluded.
    int burst_logical_index_ = -1;
    std::optional<fec::SoftCombineBuffer::ProvisionalContext> burst_harq_ctx_;
    bool burst_harq_ctx_pulled_ = false;
    bool burst_harq_prediction_invalid_ = false;
    bool burst_harq_prediction_validated_ = false;
    bool burst_harq_cadence_blocked_ = true;
    uint32_t last_burst_src_hash_ = 0;
    uint64_t last_descriptor_abs_sample_ = 0;  // sample-clock gap gate (D2)

    // ========================================================================
    // BUG-BURST-STALE-GEOMETRY (2026-07-28) — commanded-geometry fallback
    // ========================================================================
    // See setCommandedRungIndex for the threading contract.
    std::atomic<uint8_t> commanded_rung_idx_{protocol::kRungIdxNone};
    // ADOPTION EVIDENCE (2026-07-28 adversarial-review fix). "We commanded X" is NOT
    // "the sender is at X": Connection::maybeObeyAuthorityCommand has FIVE paths that
    // decline a standing command (ULTRA_LOCK_RATE / !rateAdaptationActive; a legacy
    // mode change already in flight; a rung the sender's ladder disabled, which is
    // snapped to a DIFFERENT index; an UP command deferred to a clean send boundary
    // while its TX window drains; and tryDescriptorModeSwitch falling back to the
    // MODE_CHANGE round trip). Set TRUE when a decoded descriptor announces a rung
    // that is NOT our standing command — direct wire evidence the sender has not
    // taken it — and cleared when the command changes or the sender adopts it.
    std::atomic<bool> commanded_rung_declined_{false};

    // Connection lifecycle phase that cannot be represented by connected_: while
    // true, robust OFDM control decode stays enabled but DATA/fixed-CW fallback is
    // forbidden.  See setControlOnlyReceive().
    std::atomic<bool> control_only_receive_{false};

    // Learned rung -> frame geometry. cw/frame is NOT rate-derivable: it depends on
    // ULTRA_QAM16_CW16 / the 8PSK ladder (connection_policy::recommendCWCount) and is
    // then walked DOWN against the SENDER's own measured coherence time
    // (recommendCWCountForChannel). The receiver cannot reproduce that derivation, so
    // it LEARNS the tuple from descriptors it has already decoded — wire truth.
    // A rung with no entry is a MISS -> fall back to today's behaviour; never
    // substitute a neighbour rung's cw.
    // Descriptor consume and both resolver reads normally run on the decode thread,
    // but a true connected->disconnected lifecycle transition runs setMode() on the
    // protocol/GUI thread. Protect the complete table with one mutex so that session
    // clearing is synchronized with every reader and a cw/Z tuple is copied as one
    // coherent value. The mutex is intentionally distinct from ring_.buffer_mutex_:
    // decode/search releases the ring lock before calling these resolvers. The
    // session epoch rejects a descriptor decode that began before teardown but
    // reaches the table only after the protocol thread cleared it.
    struct LearnedRungGeometry {
        uint8_t cw_per_frame = 0;
        uint8_t lifting_z = 27;
    };
    mutable std::mutex learned_rung_geometry_mutex_;
    std::array<LearnedRungGeometry, protocol::kRungIdxCount> learned_rung_geometry_{};
    uint64_t learned_rung_geometry_session_epoch_ = 0;

    // Non-zero only when an explicitly test-only caller needs to exercise the
    // long-LDPC frame path without first sending a production BURST_HEADER.
    // Single-threaded test contract; deliberately not an environment variable.
    int testing_lifting_z_override_ = 0;

    // Sample-clock stamp of the last GROUP START — NOT the last DESCRIPTOR. A run of
    // missed descriptors would inflate last_descriptor_abs_sample_ past the RTO gap
    // and mis-classify a steady-state group as a post-RTO resend, silently disarming
    // the fix in exactly the repeat case this bug produces.
    uint64_t last_group_start_abs_ = 0;

    // Resolved per-frame geometry after a descriptor was missed.
    struct CommandedGeometry {
        bool armed = false;
        Modulation mod = Modulation::QPSK;
        CodeRate rate = CodeRate::R1_4;
        int cw = 0;
        int lifting_z = 27;
        size_t frame_samples = 0;
    };
    // Non-const: it takes the ring lock for the cadence reference. It mutates no
    // decoder state (the arm itself happens at the call site).
    CommandedGeometry resolveCommandedGeometry(bool is_ofdm, bool burst_latched);
    CommandedGeometry resolveClassicLongGeometry(bool is_ofdm,
                                                 bool burst_latched,
                                                 uint64_t frame_sync_abs);
    void armClassicLongGeometry(const CommandedGeometry& geometry);
    void clearClassicLongGeometry();
    LearnedRungGeometry learnedRungGeometrySnapshot(uint8_t idx) const;
    uint64_t learnedRungGeometrySessionEpochSnapshot() const;
    void clearLearnedRungGeometryAtSessionBoundary();
    bool learnRungGeometry(
        Modulation mod, CodeRate rate, int cw, int lifting_z,
        std::optional<uint64_t> expected_session_epoch = std::nullopt);
    void noteDescriptorRungObserved(Modulation mod, CodeRate rate);
    // Absolute sample index of the current sync position (brief ring-lock read).
    uint64_t absoluteForSyncPosition();

    // ONE RESOLVE PER FRAME. checkIfReadyToDecode sizes the readiness gate and
    // decodeCurrentFrame sizes the slice, and they run in DIFFERENT processBuffer
    // iterations — commanded_rung_idx_ (ACK-emit thread) or the latched profile
    // (deferred applies at the top of processBuffer) can move between them. If the
    // two disagree the frame is truncated (requirement grew) or over-long
    // (requirement shrank, which the truncation guard CANNOT see). So the readiness
    // resolve is SNAPSHOT here and re-used by the decode, keyed on sync_position_+1
    // (0 = none); a key miss falls back to a fresh resolve.
    CommandedGeometry cg_snapshot_;
    size_t cg_snapshot_sync_pos_ = 0;

    uint32_t truncated_burst_holds_ = 0;    // guard (A) fired (episodes, not retries)
    uint32_t commanded_geometry_arms_ = 0;  // guard (B) armed
    // Narrow BI0 recovery for a descriptor + marked-head loss after an accepted
    // full anchor. This is deliberately not represented as a descriptor: cw/Z are
    // wire-learned, but physical N is unknown and must never be fabricated.
    bool classic_long_geometry_recovery_active_ = false;
    size_t classic_long_geometry_sync_pos_ = 0;  // sync_position_ + 1; 0 = none
    uint32_t classic_long_geometry_arms_ = 0;
    // Conservative unknown-N physical span was published in the same arm
    // transaction, before control returns to the event loop.
    uint64_t commanded_geometry_air_gate_span_at_arm_ = 0;
    // sync_position_ + 1 of the current hold episode (0 = none), so a multi-retry
    // deferral counts and logs ONCE.
    size_t truncation_hold_sync_pos_ = 0;
    // frame_len at the last hold of this episode. The deferral is bounded by
    // PROGRESS, not by a timer: hold only while the missing audio is still arriving.
    // Once a hold makes no progress the peer has stopped transmitting and the frame
    // will never complete — proceed with the truncated decode (exact pre-fix
    // behaviour) so the group still ARMS and still emits its no-progress tone-ACK.
    // Silently dropping it instead would convert a ~9 s fast-NACK cycle into a full
    // ARQ RTO, which is the regression BUG-ANCHOR-WAIT-NO-ACK-STALL was fixed to
    // prevent.
    size_t truncation_hold_frame_len_ = 0;
    std::vector<std::vector<float>> burst_soft_buffer_;  // collected soft bits per frame
    std::vector<DecodeResult> burst_metric_templates_;   // per-physical-frame LTS metrics
    // EARLY-FRAME-DECODE (2026-07-05, turnaround lever): for NON-interleaved groups
    // (QPSK/8PSK — each physical frame IS one logical frame) LDPC-decode each frame
    // as it ARRIVES, in the inter-frame idle, so finalize only decodes the last
    // frame before the ACK emits (~(N-1) frames of LDPC off the decode tail; also
    // smooths RX CPU — no LDPC burst at group end). FAIL-SAFE INVARIANT: the cache
    // is consulted ONLY when burst_predecoded_.size() == burst_soft_buffer_.size()
    // AND the entry is marked valid AND the group is non-interleaved AND no
    // late-join head insertion happened — any mismatch falls back to the full
    // group-end decode (a missed invalidation degrades to old behavior, never a
    // wrong decode). Cleared at every burst_soft_buffer_ clear site.
    struct PredecodedFrame {
        bool valid = false;
        DecodeResult result;
    };
    std::vector<PredecodedFrame> burst_predecoded_;
    // RX-AUTHORITY PREDICTIVE (docs/RX_AUTHORITY_PREDICTIVE_2026_07_07.md):
    // per-carrier linear-SNR accumulator across the current group's successfully
    // demodulated frames (constellation-independent — craters contribute too;
    // hosted here because the demodulator is recreated per group). Consumed into
    // last_group_carrier_gammas_ (normalized to the in-band scale) just before
    // the group callback fires; the binding forwards it to the Connection.
    std::vector<double> burst_gamma_sum_;
    size_t burst_gamma_frames_ = 0;
    std::vector<float> last_group_carrier_gammas_;
    // Per-physical-frame finalized EVM observations for the last group. Missing
    // provenance is retained as NaN at its physical position; the group scalar
    // is published only with complete, geometry-proven coverage.
    std::vector<float> last_group_evm_snr_frames_;
    std::vector<size_t> last_group_evm_carrier_counts_;
    void accumulateBurstCarrierGamma();
    void finalizeGroupCarrierGammas();
    void finalizeGroupEvmSnr(bool geometry_proven, int expected_physical_frames);
    // F176 GEOMETRIC ACK GATE: absolute sample at which the burst currently
    // being received stops ARRIVING. An accepted expected full anchor first
    // publishes a conservative provisional ceiling; descriptor/marker-backed
    // group geometry replaces it with the exact end. The ack paths defer until
    // the ring's fed counter passes it — the receiver never keys an ack into the
    // tail of the burst it is acking, regardless of what the (fade-fragile)
    // energy CCA thinks. 0 = no burst in flight.
    std::atomic<uint64_t> burst_air_end_abs_{0};
    std::atomic<bool> burst_air_end_provisional_{false};
    std::atomic<uint64_t> provisional_burst_anchor_abs_{0};
    // True only when the provisional anchor came from an actual dual-chirp lock.
    // A connected DATA-sync fallback may conservatively arm the same air gate,
    // but its own sample position is not proof that the DATA is a singleton.
    std::atomic<bool> provisional_burst_anchor_full_chirp_{false};
    void armProvisionalBurstAirGate(uint64_t anchor_abs, bool proven_full_chirp);
    void clearProvisionalBurstAirGate();
    // F221: absolute sample where the group's DATA frames begin (just past the
    // consumed BURST_HEADER). The air-end derives from this + N x the group's
    // ACTUAL per-frame sample count (burst_min_block_) — the first cut used the
    // header frame's own length (10,080) and declared a 30 s R1/4 burst as
    // 1.7 s of air: the gate opened immediately and every repeat/hole ack keyed
    // into the sender's resend (operator-caught at MPG@10, where rung geometry
    // diverges from the sp8 slot the gate was built on).
    uint64_t burst_data_start_abs_ = 0;
    void refreshBurstAirEnd();
    std::vector<BurstPhysicalDiag> burst_physical_diag_;
    uint64_t burst_diag_next_group_index_ = 0;
    uint64_t burst_diag_group_index_ = 0;
    size_t burst_diag_group_start_abs_ = 0;
    size_t burst_next_pos_ = 0;          // buffer position for next continuation frame
    size_t burst_min_block_ = 0;         // samples per frame (cached from first frame)
    float burst_snr_ = 0.0f;             // Chirp sync-quality score
    float burst_cfo_ = 0.0f;             // CFO (updated per frame from pilot tracking)
    float burst_anchor_rms_ = 0.0f;      // group anchor (chirp-dominated, low-PAPR) sample
                                         // RMS — the relative-erasure-gate reference; tracks
                                         // the per-group RX operating level so the gate is
                                         // not pinned to an absolute sim-reference amplitude
    // Software-ALC per-burst RX level accumulation (BUG-QAM16-RIG-LEVEL-BUDGET):
    // broadband sum-of-squares + peak over the group's KEPT data frames (frames
    // 2..N; the erasure-gated and the hot chirp-anchored frame-1 are excluded so
    // the measurement is the pure data-segment operating level). Reset at group
    // start (streaming_ofdm_decode.cpp burst-marker branch), folded into a
    // verdict by computeBurstLevelVerdict().
    double burst_level_sum_sq_ = 0.0;
    size_t burst_level_sample_count_ = 0;
    float burst_level_peak_ = 0.0f;
    int rx_level_last_logged_verdict_ = -1;  // LEVEL ADVISORY log: once per change
    std::chrono::steady_clock::time_point burst_start_time_;  // timeout reference
    int burst_group_size_ = 8;
    // PHANTOM-FRAME fix v2 (2026-07-04): TRUE from BURST_HEADER consume until the
    // group finalizes/aborts (cleared at every burst_soft_buffer_ clear site).
    // v1 keyed on buffer-non-empty — WRONG: the DESC-SWITCH adopt fires ~2 ms
    // after header consume, BEFORE any frame is collected, so the policy clobber
    // (5-&gt;6 phantom / 9-&gt;6 deinterleave scramble) walked past an empty buffer.
    bool descriptor_group_size_locked_ = false;
    static constexpr int BURST_TIMEOUT_MS_BASE = 8000;  // 4 frames × ~0.7s + margin

    // MC-DPSK continuous burst state. The first frame is decoded through the
    // ordinary chirp/training path; continuation frames are data-only and rely on
    // the waveform's preserved differential phase cursor.
    size_t mc_burst_next_pos_ = 0;
    size_t mc_burst_next_abs_ = 0;
    float mc_burst_snr_ = 0.0f;
    float mc_burst_cfo_ = 0.0f;
    int mc_burst_frames_decoded_ = 0;
    bool mc_burst_pending_frame_ = false;
    v2::FrameType mc_burst_pending_type_ = v2::FrameType::DATA;
    int mc_burst_pending_total_cw_ = 0;
    size_t mc_burst_pending_start_pos_ = 0;
    size_t mc_burst_pending_start_abs_ = 0;
    size_t mc_burst_pending_total_samples_ = 0;
    size_t mc_burst_pending_consumed_samples_ = 0;
    std::vector<float> mc_burst_pending_soft_bits_;
    std::chrono::steady_clock::time_point mc_burst_wait_start_time_;
    static constexpr int MC_DPSK_MAX_BURST_FRAMES = 8;

    // Pending frame state for multi-codeword frames
    // After reading header, if more codewords needed, wait for more samples
    int pending_total_cw_ = 0;                // Total codewords expected (0 = unknown)
    std::chrono::steady_clock::time_point sync_start_time_;  // When sync was found
    static constexpr int FRAME_TIMEOUT_MS = 5000;  // Give up after 5 seconds

    // Constants - Buffer sizes
    // Need enough for chirp (~1.2s) + frame (~1s) + search margin.
    // Smaller constructor-supplied rings reduce RAM but raise overflow risk
    // under sustained decode backlog; the default preserves the historical
    // 10-second ring used by simulator backlog scenarios.
    static constexpr size_t CHIRP_SAMPLES = 57600;          // ~1.2 second (dual chirp)
    static constexpr size_t CORRELATION_STEP = 4800;        // 100ms at 48kHz (faster search)
    static constexpr size_t CORR_INVARIANT_GUARD = 9600;    // 200ms guard to detect pointer drift
    static constexpr size_t OVERFLOW_RECOVERY_KEEP = 120000; // Keep ~2.5s newest audio when overloaded

    // Constants - Adaptive acquisition thresholds (disabled for now)
    // Disconnected MC-DPSK acquisition must tolerate deep fades on the control
    // chirp. Hardware Good-fading traces showed valid CONNECT energy around
    // 0.04 RMS; the old 0.05 gate skipped it before correlation could run.
    static constexpr float CORR_NOISE_THRESHOLD = 0.025f;   // Below = pure noise, don't advance
    static constexpr float CORR_WEAK_THRESHOLD = 0.10f;     // Below = weak, advance slowly
    static constexpr float CORR_DETECT_THRESHOLD = 0.15f;   // At/above = detected
    static constexpr float ENERGY_GATE_MULTIPLIER = 0.0f;   // Disabled - noise floor estimation is inaccurate
    static constexpr float PING_ENERGY_RATIO = 0.3f;        // Post-chirp/chirp energy ratio

    // §15 step 4b: always-on tone-burst ACK monitor. Construction-time tuned
    // for production cadence (longer detect_interval than the unit-test
    // default to bound CPU; a burst is 850+ ms so a 480 ms cadence still
    // guarantees full coverage). See constructor body.
    ultra::waveform::tone_burst_ack::ToneBurstAckMonitor tone_burst_monitor_;
};

} // namespace gui
} // namespace ultra
