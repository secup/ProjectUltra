#pragma once

// StreamingEncoder - Unified TX encoder for all waveform types
//
// Mirrors StreamingDecoder to ensure TX/RX use identical configurations.
// This is critical for debugging issues like channel interleaving mismatches.
//
// Key features:
//   1. Same waveform configuration as StreamingDecoder
//   2. Same carrier/pilot allocation
//   3. Same interleaver parameters (bits_per_symbol, total_bits)
//   4. Centralized encoding logic (was scattered in cli_simulator)
//
// Usage:
//   StreamingEncoder encoder;
//   encoder.setMode(WaveformMode::OFDM_CHIRP);
//   encoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
//
//   // Encode and transmit
//   auto samples = encoder.encodeFrame(frame_data);
//   audio_output(samples);
//
//   // Or for PING only
//   auto ping = encoder.encodePing();

#include "waveform/waveform_interface.hpp"
#include "waveform/waveform_factory.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/fec.hpp"
#include "ultra/papr_reduction.hpp"
#include <vector>
#include <memory>

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;

// Encoder configuration for debugging/verification
struct EncoderConfig {
    protocol::WaveformMode mode = protocol::WaveformMode::MC_DPSK;
    Modulation modulation = Modulation::DQPSK;
    CodeRate code_rate = CodeRate::R1_4;
    int num_carriers = 59;          // Total carriers (OFDM)
    int data_carriers = 53;         // Data carriers after pilot allocation
    int bits_per_symbol = 106;      // data_carriers * bits_per_carrier (DQPSK=2)
    bool use_pilots = true;
    int pilot_spacing = 10;
    bool use_channel_interleave = true;   // Spreads coded bits across carriers for fading resistance
    bool use_frame_interleave = true;
};

// Per-encode ownership for the DATA group's group-start anchor.  Keeping this
// on the transmit request (rather than only in an encoder-global one-shot) is
// essential when a front end CCA-defers more than one burst: each queued burst
// must retain its own resend/mode-switch anchor semantics until it is encoded.
struct BurstAnchorOptions {
    bool force_full_group_start = false;
    bool keep_skip_streak = false;  // mode-switch/config anchor, not delivery failure
    // Force the BURST_HEADER descriptor itself to use its robust full chirp and
    // recool delivery-earned descriptor skipping, without necessarily forcing a
    // second full chirp at the following DATA group start.  Kept separate from
    // force_full_group_start so an exact, progress-bearing partial-SACK repair can
    // re-establish the receiver's acquisition state at one anchor cost.
    bool force_full_descriptor = false;
};

class StreamingEncoder {
public:
    StreamingEncoder();
    ~StreamingEncoder();

    // ========================================================================
    // MODE CONTROL (mirrors StreamingDecoder)
    // ========================================================================

    // Set waveform mode (MC_DPSK, OFDM_CHIRP, OFDM_NARROW)
    void setMode(protocol::WaveformMode mode);

    // Set data mode (modulation and code rate)
    void setDataMode(Modulation mod, CodeRate rate);

    // Set OFDM config (for custom carrier/pilot settings)
    void setOFDMConfig(const ModemConfig& config);

    // Set MC-DPSK carrier count
    void setMCDPSKCarriers(int num_carriers);

    // Set the full MC-DPSK PHY preset
    void setMCDPSKConfig(const MultiCarrierDPSKConfig& config);

    // Switch control waveform to narrowband MC-DPSK (for narrowband sessions)
    void setNarrowbandControl(bool narrowband);

    // ========================================================================
    // ENCODING
    // ========================================================================

    // Encode frame data -> audio samples (preamble + modulated data)
    // Uses full preamble (chirp sync); prefer_short_anchor uses the Phase 2a short single-chirp
    // warm re-anchor (for the BURST_HEADER descriptor when sync is warm). #69 light_preamble emits
    // an LTS-only (NO chirp) descriptor for the ULTRA_ANCHOR_SKIP_K periodic-anchor experiment —
    // overrides prefer_short_anchor; the receiver acquires it via warm prediction + light-LTS.
    std::vector<float> encodeFrame(const Bytes& frame_data, bool prefer_short_anchor = false,
                                   bool light_preamble = false);

    // Encode frame with light preamble (for connected mode, faster turnaround)
    // Only works if waveform supports data preamble
    std::vector<float> encodeFrameLight(const Bytes& frame_data);

    // Force/mark the next OFDM frame encode as the full chirp+LTS anchor.
    // Used once when entering connected OFDM so the receiver can establish an
    // OFDM-specific timing anchor before switching to LTS-only warm sync.
    void forceNextFrameFullPreamble() { force_full_preamble_once_ = true; }

    // Encode multiple frames as a single burst with one LTS preamble
    // Each frame gets its own training symbols for per-block channel estimation
    // Returns: [LTS] + [train+data_0] + [train+data_1] + ... + [train+data_N]
    std::vector<float> encodeBurstLight(
        const std::vector<Bytes>& frame_data_list,
        BurstAnchorOptions anchor_options = {});

    // Encode PING (chirp preamble only, no data)
    std::vector<float> encodePing();

    // §15 step 4c: encode a tone-burst ACK (narrowband 4-FSK in the
    // 2400-2700 Hz subband). Replaces the 1500 ms OFDM 1-CW ACK with
    // an 850 ms-baseline tone burst (≥10 dB lower SNR floor, ~5× faster
    // at high SNR). Per §15.5 the symbol_ms picks a duration on the
    // SNR-adaptive staircase: 12 ms (≥18 dB SNR) through 200 ms
    // (< -5 dB SNR). The payload carries group_seq, per-frame mask,
    // rate hint, ACK/NACK type, all wrapped in CRC-12 + (15,11) Hamming.
    //
    // This produces RAW audio (no chirp preamble, no LTS, no OFDM
    // structure — the Costas pattern at the start is its own sync).
    // Caller is responsible for queueing the samples on the audio
    // output ring at the right time after the burst-data turnaround.
    std::vector<float> encodeToneBurstAck(
        const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload,
        uint32_t symbol_ms =
            ultra::waveform::tone_burst_ack::kBaselineSymbolMs);

    // Encode just the data portion (no preamble) - for testing
    std::vector<float> encodeDataOnly(const Bytes& frame_data);

    // ========================================================================
    // CONFIGURATION ACCESS (for verification with decoder)
    // ========================================================================

    protocol::WaveformMode getMode() const { return mode_; }
    Modulation getModulation() const { return modulation_; }
    CodeRate getCodeRate() const { return code_rate_; }

    // Get current configuration (for debugging/comparison with decoder)
    EncoderConfig getConfig() const;

    // Get underlying OFDM config
    const ModemConfig& getOFDMConfig() const { return ofdm_config_; }

    // Get the waveform (for advanced use)
    IWaveform* getWaveform() { return waveform_.get(); }

    // Verify config matches a decoder config (returns mismatch description or empty string)
    std::string verifyConfigMatch(const EncoderConfig& decoder_config) const;

    // ========================================================================
    // INTERLEAVING CONTROL
    // ========================================================================

    void setChannelInterleave(bool enable) { use_channel_interleave_ = enable; }
    bool getChannelInterleave() const { return use_channel_interleave_; }

    void setCarrierMask(uint64_t active_mask);
    uint64_t getCarrierMask() const { return carrier_mask_; }

    void setCarrierLdpcInterleaver(bool enable);
    bool getCarrierLdpcInterleaver() const { return use_carrier_ldpc_interleaver_; }

    // Frame interleaving is always enabled for OFDM
    bool getFrameInterleave() const { return use_frame_interleave_; }

    void setFixedFrameCodewords(int cw_count) {
        fixed_frame_codewords_ = v2::sanitizeFixedFrameCodewords(cw_count);
    }
    int getFixedFrameCodewords() const { return fixed_frame_codewords_; }

    // Burst-level long interleaver (spreads CW bytes across N-frame groups)
    void setBurstInterleave(bool enable) { use_burst_interleave_ = enable; }
    bool getBurstInterleave() const { return use_burst_interleave_; }
    void setBurstInterleaveGroupSize(int size);
    int getBurstInterleaveGroupSize() const { return burst_group_size_; }

    // Self-describing burst (§14.17/§14.19): when enabled, encodeBurstLight emits
    // a full-anchor 1-CW BURST_HEADER descriptor at the head of an interleaved
    // group so the receiver decodes the group from the sender's declared params
    // (group size, cw/frame, mod/rate, interleave flags) — not from local config.
    // Identity is non-addressing; the RX consumes the descriptor by payload.
    // §14.27: group sequence stamped into the BURST_HEADER descriptor so the RX
    // can ACK the right group (whole-burst stop-and-wait). Set per burst by the
    // transmit path; defaults 0 (single-shot / legacy).
    void setBurstGroupSeq(uint16_t group_seq) { burst_group_seq_ = group_seq; }
    void setBurstDescriptorEnabled(bool enable) { emit_burst_descriptor_ = enable; }
    bool getBurstDescriptorEnabled() const { return emit_burst_descriptor_; }
    void setBurstDescriptorIdentity(const std::string& src, const std::string& dst) {
        burst_descriptor_src_ = src;
        burst_descriptor_dst_ = dst;
    }
    // 2026-05-28 Phase 2: LDPC lifting size Z for the OFDM data path. 27 -> n=648
    // (legacy short LDPC), 81 -> n=1944 (long LDPC, ~3 dB more FEC margin). Set
    // per-burst by the connection layer; the value is announced in BURST_HEADER
    // payload[5] so the RX configures the LDPC decoder at the right N. Default 27.
    // Phase 3 also propagates to the waveform so getMinSamplesForCWCount returns
    // the right airtime for z=81 codewords.
    void setLDPCLiftingZ(uint8_t z) {
        ldpc_lifting_z_ = (z == 81) ? 81 : 27;  // defensive — only 27/81 allowed
        if (waveform_) waveform_->setActiveLDPCLiftingZ(ldpc_lifting_z_);
    }
    uint8_t getLDPCLiftingZ() const { return ldpc_lifting_z_; }


    void setPaprReductionEnabled(bool enable);
    bool getPaprReductionEnabled() const { return papr_reduction_enabled_; }
    void setPaprReductionThresholdDb(float threshold_db);
    float getPaprReductionThresholdDb() const { return papr_reduction_threshold_db_; }
    const phy::PaprReductionMeasurement& getLastPaprReductionMeasurement() const {
        return last_papr_reduction_;
    }

private:
    // ========================================================================
    // INTERNAL HELPERS
    // ========================================================================

    // Create/recreate waveform based on current mode
    void createWaveform();

    // Create channel interleaver with correct parameters
    void updateInterleaver();

    // Encode frame bytes to coded bytes (LDPC + interleaving)
    Bytes encodeFrameBytes(const Bytes& frame_data);

    // Calculate data carriers from OFDM config
    int calculateDataCarriers() const;

    void applyPaprReductionIfNeeded(std::vector<float>& samples,
                                    bool is_ofdm,
                                    bool is_control_frame,
                                    const char* label);
    Samples connectedDataPreambleForFrame();
public:
    // True when the last encodeBurstLight() output STARTS with a chirp-less frame (a LIGHT
    // descriptor): the caller then fills the key-up lead-in with settling symbols.
    bool lastBurstStartsChirpless() const { return last_burst_starts_chirpless_; }
    // `samples` of settling fill (the LAST samples of whole settling symbols, so the fill
    // ends on a symbol boundary right before the burst), raised-cosine ramped at its start.
    std::vector<float> settlingLeadInFill(size_t samples);
private:
    bool last_burst_starts_chirpless_ = false;

    // ========================================================================
    // STATE (mirrors StreamingDecoder)
    // ========================================================================

    // Waveform
    std::unique_ptr<IWaveform> waveform_;
    std::unique_ptr<IWaveform> control_waveform_;  // MC-DPSK for control frames
    protocol::WaveformMode mode_ = protocol::WaveformMode::MC_DPSK;

    // Modulation/coding
    Modulation modulation_ = Modulation::DQPSK;
    CodeRate code_rate_ = CodeRate::R1_4;

    // OFDM config
    ModemConfig ofdm_config_;
    int mc_dpsk_carriers_ = 8;
    MultiCarrierDPSKConfig mc_dpsk_config_ = mc_dpsk_presets::level8();
    bool narrowband_control_ = false;  // Use narrowband MC-DPSK for control/handshake

    // Interleaving
    std::unique_ptr<ChannelInterleaver> channel_interleaver_;
    bool use_channel_interleave_ = true;
    bool use_frame_interleave_ = true;     // Always on for OFDM
    uint64_t carrier_mask_ = UINT64_MAX;
    // Enabled only after both peers negotiate PHY_MASK_V1. A local-only
    // CarrierLDPC permutation is a different wire image and is not decodable
    // by legacy peers.
    bool use_carrier_ldpc_interleaver_ = false;
    int fixed_frame_codewords_ = v2::kDefaultFixedFrameCodewords;
    bool use_burst_interleave_ = false;    // Burst-level long interleaver (N-frame groups)
    int burst_group_size_ = 8;
    // #69 monotonic GROUP ordinal for ULTRA_ANCHOR_SKIP_K (NOT the ARQ base seq, which jumps by
    // group size + retx). Increments per descriptor build; drives the periodic full-chirp pattern.
    uint32_t burst_anchor_ordinal_ = 0;
    // #69 REACTIVE gate: consecutive clean (warm-delivered) groups. A resend/cold-start/escalation
    // (warm_descriptor==false) resets it to 0; the skip engages only once it reaches the configured
    // streak. Radio-agnostic delivery-driven gate — replaces the rig-disproven predicted disc label.
    uint32_t anchor_skip_clean_streak_ = 0;
    bool emit_burst_descriptor_ = false;   // §14.17 self-describing BURST_HEADER head
    uint16_t burst_group_seq_ = 0;         // §14.27 group seq stamped into descriptor
    std::string burst_descriptor_src_;
    std::string burst_descriptor_dst_;
    // 2026-05-28 Phase 2: LDPC lifting Z for the OFDM data path (27 -> n=648,
    // 81 -> n=1944). Announced in BURST_HEADER payload[5]. Default 27 keeps
    // pre-flip behavior; the connection layer sets 81 when the OFDM data burst
    // should use long LDPC.
    uint8_t ldpc_lifting_z_ = 27;
    // DIAGNOSTIC ONLY (ULTRA_GENIE_DATA_AIDED, default off; see genie_tx_capture.hpp).
    // Offset of the current encode call's output within the stream the caller is
    // assembling, so the data-aided genie can stamp each captured TX symbol with its
    // absolute transmit-stream sample position. 0 for a standalone frame; set only by
    // encodeBurstLight around the in-burst descriptor. Never read unless the genie is
    // enabled, and never affects the emitted samples.
    long long genie_stream_base_ = 0;
    bool force_full_preamble_once_ = false;
    bool papr_reduction_enabled_ = phy::kPaprReductionDefaultEnabled;
    float papr_reduction_threshold_db_ = phy::kOfdmPaprReductionDefaultThresholdDb;
    phy::PaprReductionMeasurement last_papr_reduction_;

    // Logging
    std::string log_prefix_ = "StreamingEncoder";
};

} // namespace gui
} // namespace ultra
