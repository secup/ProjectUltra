#pragma once

#include "frame_v2.hpp"
#include "arq.hpp"
#include "selective_repeat_arq.hpp"
#include "connection_policy.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"
#include "rate_controller.hpp"
#include "goodput_rate_controller.hpp"
#include "latent_rate_controller.hpp"
#include "file_transfer.hpp"
#include "ultra/types.hpp"
#include "fec/soft_combine.hpp"
#include <cmath>
#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <utility>

namespace ultra {
namespace protocol {

// Connection states
enum class ConnectionState {
    DISCONNECTED,
    PROBING,       // Sending PING, waiting for PONG (fast presence check)
    CONNECTING,    // Received PONG, sending full CONNECT
    CONNECTED,
    DISCONNECTING
};

const char* connectionStateToString(ConnectionState state);

// Live "a burst is arriving" status for the GUI — surfaced BEFORE a file transfer is
// established (FILE_START), so the operator can see the modem is actually working: the
// group number advances and the per-group decoded-frame count climbs, instead of a dead
// UI that only wakes up once metadata finally decodes. Once a real file transfer is
// RECEIVING, the GUI swaps this flashing indicator for the file progress bar (and can
// still show the group # alongside it).
struct BurstActivity {
    bool active = false;          // a burst is currently being received
    uint32_t group_seq = 0;       // latest burst group number (advances = progress)
    uint8_t frames_decoded = 0;   // frames that decoded in the latest group (X)
    uint8_t frames_in_group = 0;  // group size from the burst descriptor (Y)
    uint32_t groups_seen = 0;     // running count of group receptions (liveness)
};

// Connection configuration
struct ConnectionConfig {
    ARQConfig arq;
    uint32_t connect_timeout_ms = 60000;  // 60s for DPSK (16s TX + 16s RX + margin)
    uint32_t disconnect_timeout_ms = 30000;
    int connect_retries = 10;  // Robust MC-DPSK control attempts
    bool auto_accept = true;

    uint8_t mode_capabilities = ModeCapabilities::ALL | ModeCapabilities::PHY_MASK_V1;
    WaveformMode preferred_mode = WaveformMode::AUTO;  // Forced waveform (0xFF=AUTO)

    // Forced data mode - operator can override SNR-based selection
    // 0xFF (AUTO) = let responder decide based on SNR
    // Any other value = force that specific mode
    Modulation forced_modulation = Modulation::AUTO;
    CodeRate forced_code_rate = CodeRate::AUTO;
    int fixed_frame_codewords = v2::kDefaultFixedFrameCodewords;
    int mc_dpsk_num_carriers = 8;
    int mc_dpsk_samples_per_symbol = 1024;
    // Initiator-side forced CW override (0 = AUTO, responder picks via
    // recommendCWCount(rate)). When non-zero, the initiator embeds this
    // value in CONNECT.data_frame_cw_count and the responder honors it
    // in CONNECT_ACK + applyDataMode. Set via setForcedFrameCodewords().
    uint8_t forced_cw_count = 0;
};

// Connection statistics
struct ConnectionStats {
    ARQStats arq;
    int connects_initiated = 0;
    int connects_received = 0;
    int connects_failed = 0;
    int disconnects = 0;
    uint32_t connected_time_ms = 0;
    // DESC-SWITCH (ULTRA_DESCRIPTOR_MODE_SWITCH Phase 1) telemetry: descriptor-committed
    // rate/mod moves — sender commits + receiver adopts. 0 while the knob is OFF.
    int descriptor_mode_switches = 0;
};

// One source of truth for the logical-to-physical file-profile substitution.  The sender
// resolves it from transfer-scoped arms; the receiver resolves the same tuple from its
// local, default-off experiment policy when pricing a counterfactual rate command.
struct OFDMDataWireProfile {
    int logical_cw = 0;
    int physical_cw = 0;
    int lifting_z = 27;
};

// Exact counterfactual wire tuple used by the outcome-fitted rate selector.  Keep the
// physical representation alongside the scored value: the long-LDPC file experiments
// preserve the negotiated logical rung while changing CW, lifting Z, payload capacity,
// frame budget, and airtime on the wire.
struct LatentRateCandidateGeometry {
    LatentRateController::RungGeometry value;
    int logical_cw = 0;
    int physical_cw = 0;
    int lifting_z = 27;
    size_t frames_per_cycle = 0;
    uint32_t burst_airtime_ms = 0;
    bool force_full_group_start = false;
};

/**
 * Connection Manager
 *
 * Handles connection establishment and teardown with callsign addressing.
 * Wraps ARQ controller for reliable data transfer once connected.
 * Uses v2 frame format exclusively.
 */
class Connection {
    friend struct ConnectionAdaptiveTestAccess;
public:
    // Callback types - all use serialized Bytes (v2 frames)
    using TransmitCallback = std::function<void(const Bytes&)>;
    using TransmitInfoCallback =
        std::function<void(const Bytes&, bool expect_full_ofdm_anchor_after_tx)>;
    // §15 step 4d-iii: parallel emit path for tone-burst ACK. Carries the
    // decoded payload, not raw bytes — the modem layer encodes it to audio
    // via StreamingEncoder::encodeToneBurstAck() (step 4c) and queues the
    // samples on the audio output. Fires alongside the OFDM GROUP_ACK
    // transmitFrame() so both paths are on the wire; the receiver's
    // monitor wins on speed.
    // `inbound_group_complete` is a physical-egress provenance bit. True means
    // this ACK was emitted synchronously by endGroupReceiveAndAck(), after the
    // decoder declared the inbound burst complete; false covers timer/standalone
    // ACKs whose descriptor may have been lost and which therefore require the
    // front end's conservative listen-before-talk guard.
    using TransmitToneBurstAckCallback = std::function<void(
        const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&,
        bool inbound_group_complete)>;
    using LegacyTransmitToneBurstAckCallback = std::function<void(
        const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&)>;
    // §15 step 4d-late: arm-the-monitor callback. Fires right after a
    // data burst is queued for transmission so the receiver-side tone-
    // burst ACK monitor wakes up for the duration of the expected ACK
    // window. window_ms = the ack_timeout duration (sender's RTO).
    using ArmToneBurstAckMonitorCallback = std::function<void(uint32_t window_ms)>;
    using ConnectedCallback = std::function<void()>;
    using DisconnectedCallback = std::function<void(const std::string& reason)>;
    // Explicit wire-teardown phase.  This differs from ConnectionState because a
    // responder deliberately remains CONNECTED for a bounded grace period after
    // ACKing DISCONNECT so it can hear and react to a duplicate request.  Hosts use
    // this signal to switch the modem to control-only acquisition and to quarantine
    // any application DATA already queued outside Connection.
    using DisconnectTeardownCallback = std::function<void(bool active)>;
    using MessageReceivedCallback = std::function<void(const std::string& text)>;
    using MessageSentCallback = std::function<void(bool success)>;
    enum class MessageTxStatus {
        SUBMITTED,
        DELIVERED,
        FAILED
    };
    struct MessageTxStatusEvent {
        MessageTxStatus status = MessageTxStatus::SUBMITTED;
        uint16_t first_seq = 0;
        uint16_t last_seq = 0;
        bool sequence_valid = false;
        std::string remote_call;
        std::string text;
        uint32_t elapsed_ms = 0;
    };
    using MessageTxStatusCallback = std::function<void(const MessageTxStatusEvent& event)>;
    using IncomingCallCallback = std::function<void(const std::string& remote_call)>;
    using DataReceivedCallback = std::function<void(const Bytes& data, bool more_data)>;

    // Ping/Pong callbacks (for fast presence check before full CONNECT)
    using PingTxCallback = std::function<void()>;  // Request modem to transmit ping
    using PingReceivedCallback = std::function<void()>;  // Called when receiver detects our ping (incoming call)

    // State change callback (for internal state transitions like PROBING → CONNECTING)
    using StateChangedCallback = std::function<void(ConnectionState state, const std::string& info)>;

    // File transfer callbacks
    using FileProgressCallback = FileTransferController::ProgressCallback;
    using FileReceivedCallback = FileTransferController::ReceivedCallback;
    using FileSentCallback = FileTransferController::SentCallback;

    explicit Connection(const ConnectionConfig& config = ConnectionConfig{});
    ~Connection();

    // --- Configuration ---

    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const { return local_call_; }

    void setAutoAccept(bool auto_accept) { config_.auto_accept = auto_accept; }
    bool getAutoAccept() const { return config_.auto_accept; }

    // --- Connection Control ---

    bool connect(const std::string& remote_call);
    void acceptCall();
    void rejectCall();
    void disconnect();
    void abortTxNow();

    // --- Data Transfer ---

    bool sendMessage(const std::string& text);
    bool sendBinary(const Bytes& data);
    bool sendMessages(const std::vector<std::string>& texts);  // Batch: burst-interleaved
    bool isReadyToSend() const;
    size_t getTxBacklogBytes() const;

    // --- File Transfer ---

    bool sendFile(const std::string& filepath);
    void setReceiveDirectory(const std::string& dir);
    void cancelFileTransfer();
    bool isFileTransferInProgress() const;
    FileTransferProgress getFileProgress() const;
    // Live burst-arrival status (group #, X/Y frames) for the "incoming burst" GUI
    // indicator, valid before a file transfer is established.
    BurstActivity getBurstActivity() const { return burst_activity_; }

    // --- Frame Processing ---

    // Process received frame data (v2 serialized bytes)
    void onFrameReceived(const Bytes& frame_data,
                         bool physical_turn_complete = false);
    void onMCDPSKPartialFrame(const v2::PartialFrameCodewords& partial);
    void onAcceptedOFDMDataSync(float sync_correlation);

    // §14.27 / §SR-ARQ: a decoded burst delivered by the decoder (group_seq, ordered
    // serialized DATA frames that decoded, all-logical-frames-decoded flag, per-frame
    // SACK mask, interleaved flag). Two channel-adaptive modes:
    //   - interleaved (Moderate/Poor): a partial group is undecodable → only all_ok is
    //     delivered+ACKed; a partial is NACKed for a whole-burst resend.
    //   - NOT interleaved (Good/AWGN SR-ARQ): each decoded frame is delivered immediately
    //     (offset-keyed assembler) and the tone-burst ACK carries the true frame_mask so
    //     the sender resends only the 0-bit frames + refills the burst.
    // Inert unless use_burst_transport_.
    void onBurstGroupReceived(uint16_t group_seq, const std::vector<Bytes>& frames,
                              bool all_ok, float quality, uint16_t frame_mask = 0xFFFF,
                              bool interleaved = true, uint8_t group_size = 0,
                              bool geometry_proven = false);

    // §14.36 Phase 5c GUI observability: decode headroom of the most recent burst
    // group [0,1] (<0 = none yet) and a short human-readable adaptive action
    // ("rate R3/4 -> R2/3 (q=0.18)" / "hold R3/4 (q=0.85)"). Empty when off.
    bool adaptiveRateEnabled() const { return adaptive_rate_enabled_; }
    float lastGroupQuality() const { return last_group_quality_; }

    // Enable half-duplex INTERACTIVE (bidirectional) data: the TNC/B2F path where
    // both stations alternately transmit. Keeps the ISS/IRS turn gate on burst
    // file sends so the two directions serialize instead of colliding. See the
    // half_duplex_interactive_ member for the full rationale.
    void setHalfDuplexInteractive(bool v) { half_duplex_interactive_ = v; }
    bool halfDuplexInteractive() const { return half_duplex_interactive_; }
    const std::string& lastAdaptiveAction() const { return last_adaptive_action_; }

    void tick(uint32_t elapsed_ms);

    // --- Callbacks ---

    void setTransmitCallback(TransmitCallback cb);
    void setTransmitInfoCallback(TransmitInfoCallback cb);
    // BUG-MC-RETRY-SPURIOUS (2026-07-04): host-provided "is our TX keyed right now"
    // predicate (the app's tx_in_progress_ atomic). While WE are transmitting we
    // physically cannot have decoded the peer's control ACK (half-duplex), so the
    // MODE_CHANGE retry deadline HOLDS during own TX instead of ticking — the timer
    // effectively anchors at key-UP of the burst that carried the frame. Unwired
    // (headless/tests) => nullptr => legacy request-anchored behavior.
    void setTxActiveProvider(std::function<bool()> provider) {
        tx_active_provider_ = std::move(provider);
    }

    // Channel-busy query (keepalive ACK universal gating, 2026-07-14): returns
    // true when a burst is currently arriving / the channel is busy. The
    // keepalive checks this BEFORE emitting so it never TX's over an inbound
    // burst — replacing the GUI-only listen-before-ACK dependency that blocked
    // default-on (the headless TNC lacked it). Both the GUI and the TNC set it
    // from the modem (channelBusyForTx || burstAirSamplesRemaining>0). Unwired
    // (tests) => nullptr => keepalive falls back to threshold-only safety.
    void setChannelBusyQuery(std::function<bool()> query) {
        channel_busy_query_ = std::move(query);
    }

    void setTransmitToneBurstAckCallback(TransmitToneBurstAckCallback cb) {
        on_transmit_tone_burst_ack_ = std::move(cb);
        arq_.setToneBurstSackTransportReady(
            static_cast<bool>(on_transmit_tone_burst_ack_));
    }
    void setTransmitToneBurstAckCallback(LegacyTransmitToneBurstAckCallback cb) {
        setTransmitToneBurstAckCallback(
            [cb = std::move(cb)](
                const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload,
                bool /*inbound_group_complete*/) {
                cb(payload);
            });
    }
    void setArmToneBurstAckMonitorCallback(ArmToneBurstAckMonitorCallback cb) {
        on_arm_tone_burst_ack_monitor_ = std::move(cb);
    }

    // Burst mode TX callback - transmits multiple frames as single audio burst.
    // Used for OFDM connected mode and MC-DPSK DATA file-window bursts.
    // group_seq stamps the burst descriptor (§14.27) for whole-burst GROUP_ACK;
    // 0 for the legacy arq_ burst path / single-shot.
    // anchor_reason selects descriptor and DATA-group anchor ownership for this exact
    // physical request. RESEND and MODE_SWITCH robustly anchor both the descriptor and
    // the following DATA group. PROVEN_PARTIAL_REPAIR robustly anchors the descriptor
    // but keeps the DATA group on its announced light LTS handoff; it is emitted only
    // by a strict default-off experiment with exact completed-group provenance.
    //
    // The REASON matters to the #69 anchor-skip clean streak, which is DELIVERY
    // evidence ("this channel syncs fine without a chirp"). A resend is genuine
    // negative delivery evidence and must recool the streak. A descriptor-committed
    // MODE/RATE SWITCH is a CONFIGURATION event — it legitimately needs one full
    // anchor for the geometry change, but it says nothing about whether the channel
    // still supports skipping, so erasing the streak there throws away evidence and
    // (measured, 2026-07-26) is why the skip arms far less often than it could.
    // uint8_t rather than an enum class so existing bool call sites still convert.
    static constexpr uint8_t kAnchorReasonNone = 0;
    static constexpr uint8_t kAnchorReasonResend = 1;      // delivery evidence -> recool
    static constexpr uint8_t kAnchorReasonModeSwitch = 2;  // config event -> keep streak
    static constexpr uint8_t kAnchorReasonProvenPartialRepair = 3;
    using TransmitBurstCallback =
        std::function<void(const std::vector<Bytes>&, uint16_t group_seq,
                           uint8_t anchor_reason)>;
    void setTransmitBurstCallback(TransmitBurstCallback cb);

    void setConnectedCallback(ConnectedCallback cb);
    void setDisconnectedCallback(DisconnectedCallback cb);
    void setDisconnectTeardownCallback(DisconnectTeardownCallback cb);
    void setMessageReceivedCallback(MessageReceivedCallback cb);
    void setMessageSentCallback(MessageSentCallback cb);
    void setMessageTxStatusCallback(MessageTxStatusCallback cb);
    void setIncomingCallCallback(IncomingCallCallback cb);
    void setDataReceivedCallback(DataReceivedCallback cb);

    // Ping/Pong (fast presence check)
    void setPingTxCallback(PingTxCallback cb) { on_ping_tx_ = cb; }
    void setPingReceivedCallback(PingReceivedCallback cb) { on_ping_received_ = cb; }
    void setStateChangedCallback(StateChangedCallback cb) { on_state_changed_ = cb; }
    void onPongReceived();  // Call when modem detects response to our PING

    // §15 step 4d-ii: hand a decoded tone-burst ACK to the protocol. Mirrors
    // the OFDM GROUP_ACK arrival path (connection.cpp:2516) but skips the
    // OFDM frame parse — the payload is already decoded by the receiver's
    // ToneBurstAckMonitor. Safe to call from the audio thread; method
    // resolves group_seq against the in-flight burst (lower-6-bit match)
    // and advances burst_transport_ if the ACK is for the expected group.
    // Returns true iff the detection matched an in-flight group; false if
    // the detection was stale/out-of-context (silently dropped).
    bool onToneBurstAck(
        const ultra::waveform::tone_burst_ack::ToneBurstAckDetection& detection);
    bool isToneBurstAckCandidatePlausible(
        const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload) const;

    void setFileProgressCallback(FileProgressCallback cb);
    void setFileReceivedCallback(FileReceivedCallback cb);
    void setFileSentCallback(FileSentCallback cb);

    // --- State ---

    ConnectionState getState() const { return state_; }
    std::string getRemoteCallsign() const { return remote_call_; }
    bool isConnected() const { return state_ == ConnectionState::CONNECTED; }
    bool isDisconnectTeardownActive() const {
        return disconnect_teardown_active_;
    }
    ConnectionStats getStats() const;
    void resetStats();

    // --- Waveform Mode ---

    WaveformMode getNegotiatedMode() const { return negotiated_mode_; }
    bool isInitiator() const { return is_initiator_; }
    bool isHandshakeConfirmed() const { return handshake_confirmed_; }
    void setPreferredMode(WaveformMode mode) { config_.preferred_mode = mode; }
    void setModeCapabilities(uint8_t caps) { config_.mode_capabilities = caps; }
    bool isPhyMaskV1Negotiated() const { return phy_mask_v1_negotiated_; }

    // Session-scoped narrowband override (cleared on disconnect/reset)
    // Set when responder detects narrowband chirp — overrides config_.preferred_mode for this session only
    void setNarrowbandOverride(WaveformMode mode) { narrowband_override_ = mode; }

    // Forced data mode - operator can override SNR-based selection
    void setForcedModulation(Modulation mod) { config_.forced_modulation = mod; }
    void setForcedCodeRate(CodeRate rate) { config_.forced_code_rate = rate; }
    void setMCDPSKConfig(int num_carriers, int samples_per_symbol);
    // forced=true marks this as an operator override: the initiator will
    // embed it in CONNECT.data_frame_cw_count and the responder will
    // honor + echo it. forced=false is the boot-time default path used
    // by host wiring (encoder/decoder bootstrap) — does NOT mark forced,
    // so the responder still gets to auto-pick via recommendCWCount(rate).
    void setForcedFrameCodewords(int cw_count, bool forced = true);
    Modulation getForcedModulation() const { return config_.forced_modulation; }
    CodeRate getForcedCodeRate() const { return config_.forced_code_rate; }
    int getForcedFrameCodewords() const { return data_frame_cw_count_; }

    // Single TX-side source of truth for the per-burst LDPC lifting Z (27 -> n=648,
    // 81 -> n=1944). The connection owns this because it owns traffic class; the
    // result is announced on the wire in BURST_HEADER payload[5] (encoder reads the
    // same value) so the RX matches via the descriptor, and the app pushes it to
    // ModemEngine::setBurstLiftingZ so the encoder Z matches the chunker. Traffic
    // class: long LDPC (81) only for a scoped descriptor-bearing profile, short
    // (27) for control / interactive / MC-DPSK. The old raw ULTRA_LDPC_Z override
    // is intentionally ignored here because it could create unannounced singleton
    // Z=81 traffic. See docs/LDPC_Z_DERIVATION_DESIGN_2026_05_30.md.
    int selectBurstLiftingZ() const;

    // Half-duplex airtime ceiling for ONE key-down: bound the per-burst frame count
    // so a single transmission can't run too long (PA duty/thermal, T/R turnaround,
    // and ack latency — the burst must not outlive its own tone-burst ack window).
    // FULLY DERIVED, not a per-mode constant: the frame count falls out of the live
    // per-frame airtime (wideOFDMBurstAirtimeMs at the active modulation/rate/cw/
    // fading), so it adapts across the whole rung family by construction. Returns a
    // count in [1, max_frames]. The only fixed input is the ~6 s airtime ceiling.
    size_t burstAirtimeBudgetFrames(size_t max_frames,
                                    bool force_full_group_start = false) const;

    // ACK timeout for the unified group-ack burst path, sized to the ACTUAL physical
    // frame count rather than the maximum window. The shared policy includes DATA
    // airtime, the peer's receive/SACK hold, decode jitter, ACK return, turnaround and
    // resend-anchor reserve, all scaled by modulation/rate/CW/Z.
    uint32_t unifiedBurstAckTimeoutMs(size_t burst_frames) const;

    // Prepare ONE wide-OFDM unified burst window, SHARED by the file (sendNextFileChunk) and
    // message (sendNextFragment) paths so both key down as one budget-sized group:
    // returns only the per-burst frame cap (= airtime budget). Physical egress later
    // commits the exact timer once padding and the final frame vector are known. Returns
    // SIZE_MAX (no cap, legacy behavior) off the unified wide-OFDM path.
    size_t prepareUnifiedBurstWindow(bool repair_turn = false);

    // Commit a wide-OFDM deadline derived from the physical frame vector that will actually go
    // on air (including interleave pads) plus any modeled audio queue delay. Only exact
    // matching ARQ slots are re-armed; pads affect geometry but cannot match a slot.
    // Other live holes are timeout-suspended until a later physical round emits them.
    // The return value drives the same monitor window.
    uint32_t finalizeUnifiedBurstWindow(const std::vector<Bytes>& transmitted_frames,
                                        uint32_t queue_delay_ms = 0,
                                        bool force_full_group_start = false);

    // Arm the receiver-side tone-burst ACK monitor for the ack of a data burst we just
    // (re)sent. Must fire on EVERY burst that expects an ack — the INITIAL send AND each
    // timeout RESEND — or the listen window expires after the first send and the sender
    // goes deaf to every subsequent ack (the original burst sender re-armed per group).
    void armToneBurstAckListenWindow(uint32_t explicit_timeout_ms = 0);

    void setSoftCombiningHARQ(bool enable);
    bool getSoftCombiningHARQ() const { return soft_combine_harq_.enabled(); }
    fec::SoftCombineBuffer* softCombineBuffer() { return &soft_combine_harq_; }
    std::optional<fec::SoftCombineBuffer::ProvisionalContext>
    harqProvisionalContext() const;

    using ModeNegotiatedCallback = std::function<void(WaveformMode mode)>;
    void setModeNegotiatedCallback(ModeNegotiatedCallback cb) { on_mode_negotiated_ = cb; }

    // Callback when handshake is confirmed (safe to switch to negotiated waveform)
    // For initiator: called immediately after CONNECT_ACK received
    // For responder: called when first frame received after sending CONNECT_ACK
    using HandshakeConfirmedCallback = std::function<void()>;
    void setHandshakeConfirmedCallback(HandshakeConfirmedCallback cb) { on_handshake_confirmed_ = cb; }

    // Callback when the protocol has just emitted an OFDM ACK that makes the
    // peer's next DATA turn likely to begin with a full OFDM anchor.
    using FullOFDMAnchorExpectedCallback = std::function<void()>;
    void setFullOFDMAnchorExpectedCallback(FullOFDMAnchorExpectedCallback cb) {
        on_full_ofdm_anchor_expected_ = std::move(cb);
    }

    // Callback when this station has just ACQUIRED the half-duplex DATA turn
    // (received a TURNOVER → now ISS). The new sender's first burst must carry a
    // FULL chirp+LTS anchor: the new receiver has been tracking only the PREVIOUS
    // sender's timing and cannot warm-sync to us, so without this it logs endless
    // "burst marker timing retry" and never decodes (BUG-TNC-B2F-001). The TNC
    // wires this to ModemEngine::forceNextFrameFullPreamble(); the peer that
    // yielded already armed expectFullOFDMAnchorOnce() (expectsFullOFDMAnchorAfterTx
    // returns true for TURNOVER/TURN_REQUEST), so the two sides meet.
    using DataTurnAcquiredCallback = std::function<void()>;
    void setDataTurnAcquiredCallback(DataTurnAcquiredCallback cb) {
        on_data_turn_acquired_ = std::move(cb);
    }

    // Callback when the connection-attempt waveform changes.
    using ConnectWaveformChangedCallback = std::function<void(WaveformMode mode)>;
    void setConnectWaveformChangedCallback(ConnectWaveformChangedCallback cb) { on_connect_waveform_changed_ = cb; }

    using PhyMaskV1NegotiatedCallback = std::function<void(bool enabled)>;
    void setPhyMaskV1NegotiatedCallback(PhyMaskV1NegotiatedCallback cb) {
        on_phy_mask_v1_negotiated_ = cb;
    }

    // Get current waveform being used for connection attempts
    WaveformMode getConnectWaveform() const { return connect_waveform_; }

    // Set initial waveform for next connection tests.
    void setInitialConnectWaveform(WaveformMode mode) { connect_waveform_ = mode; }

    // --- Data Mode (modulation + code rate) ---

    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    // Set measured SNR from modem layer (call this when decoding frames).
    // data_aided: snr_db is the MC-DPSK data-aided (fade-averaged) estimate — the
    // only source the connectSelectionSnrDb saturation bound may act on.
    // (§6 physical entry-cap inputs live below as physical_channel_*_.)
    void setMeasuredSNR(float snr_db, SNRSource source = SNRSource::NONE,
                        bool data_aided = false) {
        if (!std::isfinite(snr_db) || !acceptsRateSelectionSNR(source)) {
            return;
        }
        measured_snr_db_ = snr_db;
        measured_snr_source_ = source;
        measured_snr_data_aided_ = (source == SNRSource::MCDPSK_IN_BAND) && data_aided;
        measured_snr_valid_ = true;
        // #58 increment 3: qualifying readings ALSO enter the connect-SNR pool
        // (population contract enforced inside addReading: data-aided MCDPSK_IN_BAND
        // + OFDM_BROADBAND only). Fed exactly where the scalar is assigned so the
        // trust/addressing surface is byte-identical to the scalar's.
        connect_snr_pool_.addReading(snr_db, source, data_aided);
    }
    float getMeasuredSNR() const { return measured_snr_db_; }
    SNRSource getMeasuredSNRSource() const { return measured_snr_source_; }

    // Set channel quality including fading detection
    // fading_index: combined freq_cv + temporal_cv, where > 0.65 indicates significant fading
    void setChannelQuality(float snr_db, float fading_index,
                           SNRSource source = SNRSource::NONE,
                           bool data_aided = false) {
        if (!std::isfinite(snr_db) || !acceptsRateSelectionSNR(source)) {
            return;
        }
        measured_snr_db_ = snr_db;
        measured_snr_source_ = source;
        measured_snr_data_aided_ = (source == SNRSource::MCDPSK_IN_BAND) && data_aided;
        measured_snr_valid_ = true;
        // #58 increment 3: see setMeasuredSNR — same feed, same contract. This feed
        // has the fading observed with the reading, so it rides along (increment 4:
        // the entry pick pools FADING like it pools SNR — single-frame fading at
        // Watterson Good scatters 0.24-0.74 around the 0.65 class boundary).
        connect_snr_pool_.addReading(snr_db, source, data_aided, fading_index);
        if (std::isfinite(fading_index)) {
            fading_index_ = fading_index;
            ms_since_fading_update_ = 0;
        }
    }
    float getFadingIndex() const { return fading_index_; }
    bool isFading() const { return fading_index_ > 0.65f; }

    // Doppler coherence (channel coherence-TIME) measured by the OFDM demodulator. Unlike
    // fading_index (fade DEPTH), this discriminates Good (slow fading) from Moderate (fast);
    // valid only after enough OFDM data has pooled (~8 frames). Consumed via
    // connection_policy::coherenceAdjustedFadingIndex in the rate-decision handlers.
    // See docs/CHANNEL_DISCRIMINATOR_DESIGN_2026_06_15.md.
    // Handoff §6: the fade-averaged PHYSICAL channel SNR distribution measured
    // across the handshake (decoder ring, linear-mean, dial convention) — the
    // physics ceiling for the entry pick (connectSelectionSnrDb cap).
    void setPhysicalChannelStats(float mean_db, float spread_db, uint32_t n) {
        physical_channel_mean_db_ = mean_db;
        physical_channel_spread_db_ = spread_db;
        physical_channel_n_ = n;
    }

    void setChannelCoherence(float coherence_score, float doppler_hz, bool valid) {
        // BUG-DOPPLER-COHERENCE-MODECHANGE-WIPE fix (2026-07-02): while CONNECTED, a valid
        // Good/Moderate verdict is CARRIED at the Connection layer across any modem-layer
        // rebuild (MODE_CHANGE waveform recreation, full re-anchor, RX drains). The estimator
        // is a cumulative mean that never un-validates on its own, so an invalid feed while
        // connected can only mean "the decoder-side pool was reset" — hold the last valid
        // verdict instead of reverting the rate ladder to the blind fading_index during the
        // ~30 s re-pooling window. Cleared at the connection boundary (enterConnected /
        // reset), so a stale verdict never leaks across connections or into the CONNECT-time
        // pick (which stays byte-identical: coherence is always invalid at CONNECT).
        //
        // doppler_hz (§RETX-PACING plumb, 2026-07-03) rides the same feed with the same
        // hold-last-valid semantics. It is the estimator's SECONDARY/approximate RMS-Doppler
        // readout (fixed nominal cadence, doppler_coherence_estimator.hpp) — consumed ONLY
        // by the order-of-magnitude retx trough-pacing deferral (clamp-bounded), never by a
        // decode decision. 0 = "not estimable" (the policy falls back to the fading-index-
        // derived ITU-R design Doppler).
        if (!valid && coherence_valid_ && state_ == ConnectionState::CONNECTED) {
            return;
        }
        coherence_score_ = coherence_score;
        coherence_doppler_hz_ =
            (std::isfinite(doppler_hz) && doppler_hz > 0.0f) ? doppler_hz : 0.0f;
        coherence_valid_ = valid;
    }
    float getCoherenceScore() const { return coherence_score_; }
    float getCoherenceDopplerHz() const { return coherence_doppler_hz_; }
    bool coherenceValid() const { return coherence_valid_; }

    // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET) receiver side: per-burst RX level
    // verdict from the decoder (connection_policy::RxLevelVerdict as int) with its
    // measurement seq. Fed by the modem binding BEFORE onBurstGroupReceived so the
    // advisory derived here rides on THIS group's tone-burst ACK. A repeated seq
    // (stale re-feed, e.g. a timed-out group) is ignored — LOW streaks only grow on
    // fresh measurements.
    void setRxLevelVerdict(int verdict, uint32_t seq);

    // RX-AUTHORITY (2026-07-05): fresh per-group receiver channel observation from
    // the decoder's lock-free atomics (broadband SNR EMA, per-frame fading index,
    // coherence disc). Fed by the modem binding BEFORE onBurstGroupReceived so the
    // rung command derived from it rides THIS group's ACK. No-op storage when the
    // knob is off (cheap floats; verdict computation is knob-gated).
    void setBurstChannelObservation(float snr_db, float fading_index,
                                    float coherence_score, bool coherence_valid,
                                    float doppler_hz) {
        burst_obs_snr_db_ = snr_db;
        burst_obs_fading_ = fading_index;
        burst_obs_coh_score_ = coherence_score;
        burst_obs_coh_valid_ = coherence_valid;
        burst_obs_doppler_hz_ = doppler_hz;
    }

    // RX-AUTHORITY EVM DEMOTE (2026-07-24): this group's radio-agnostic
    // decision-directed EVM usable-SNR (dB) from the decoder's finalized estimate.
    // Fed alongside setBurstChannelObservation, BEFORE onBurstGroupReceived, so the
    // demote clamp derived from it rides THIS group's ACK. The default latent-rate
    // path returns before consulting EVM; this remains a legacy-selector clamp when
    // ULTRA_LATENT_RATE=0. Validity is nevertheless per-group so toggling paths can
    // never revive an older observation.
    void setBurstEvmObservation(float evm_snr_db) {
        burst_obs_evm_snr_db_ = evm_snr_db;
    }
    void clearBurstEvmObservation() {
        burst_obs_evm_snr_db_ = -1.0f;
    }

    // RX-AUTHORITY PREDICTIVE: the group's per-carrier linear-SNR snapshot
    // (in-band-normalized), delivered AND cratered groups (survivor-bias kill).
    // Ring of the last kRxAuthGammaRing snapshots; an up-jump target must pass
    // EESM prediction on ALL fresh ones (docs/RX_AUTHORITY_PREDICTIVE_2026_07_07.md).
    void setBurstCarrierGammas(const std::vector<float>& gammas) {
        if (gammas.empty()) return;
        rx_auth_gamma_ring_[rx_auth_gamma_next_] = gammas;
        rx_auth_gamma_time_[rx_auth_gamma_next_] = std::chrono::steady_clock::now();
        rx_auth_gamma_next_ = (rx_auth_gamma_next_ + 1) % kRxAuthGammaRing;
        if (rx_auth_gamma_count_ < kRxAuthGammaRing) ++rx_auth_gamma_count_;
    }

    // Software-ALC sender side: fires when a decoded tone-burst ACK carries a
    // non-hold drive advisory (1=up, 2=down) while we have in-flight data. Runs
    // under the ProtocolEngine mutex — the host must NOT call back into the
    // protocol from it (atomics + logging only). group_seq lets the host dedup
    // repeat-ACK detections (one adjustment per ACKed group).
    using DriveAdvisoryCallback = std::function<void(uint8_t advisory, uint8_t group_seq)>;
    void setDriveAdvisoryCallback(DriveAdvisoryCallback cb) {
        on_drive_advisory_ = std::move(cb);
    }

    // Callback when remote station requests mode change
    // Data-mode-changed callback. cw_count is the negotiated fixed-frame CW
    // count for the new rate (1..8) — host updates encoder/decoder from this
    // value directly. Host MUST NOT call back into ProtocolEngine from this
    // callback (mutex held; re-entry will deadlock).
    // snr_is_wire: true when snr_db arrived over the wire (peer's measurement:
    // CONNECT_ACK on the initiator, MODE_CHANGE on the receiver); false when it is
    // this station's OWN local reading (responder connect-time pick, sender-side
    // MODE_CHANGE commit) — the GUI must label the source accordingly (the responder
    // connect line used to mislabel a LOCAL reading as "(wire_peer)").
    using DataModeChangedCallback = std::function<void(Modulation mod, CodeRate rate,
                                                        int cw_count,
                                                        float snr_db, float peer_fading_index,
                                                        int mc_dpsk_num_carriers,
                                                        int mc_dpsk_samples_per_symbol,
                                                        bool snr_is_wire)>;
    void setDataModeChangedCallback(DataModeChangedCallback cb) { on_data_mode_changed_ = cb; }

    // Request mode change to remote station
    void requestModeChange(Modulation new_mod, CodeRate new_rate, float measured_snr, uint8_t reason);

    // DESC-SWITCH RX notification (ULTRA_DESCRIPTOR_MODE_SWITCH Phase 1,
    // docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.1 step 4b): the decoder
    // consumed a BURST_HEADER descriptor whose mod/rate differs from the current data
    // mode — follow it at the protocol layer (RX-side applyDataMode + ARQ reconfig +
    // GUI notify). Fires NO MODE_CHANGE ACK machinery — confirmation is the group's
    // tone-burst ACK, implicit and free. No-op while the knob is OFF (byte-identical).
    // Wired decoder → ModemEngine → frontend binding → ProtocolEngine, mirroring
    // onBurstGroupReceived (same thread/locking class).
    void onDescriptorModeChange(Modulation mod, CodeRate rate, int cw_per_frame);

    void reset();

private:
    void setPhyMaskV1Negotiated(bool enabled);
    static bool acceptsRateSelectionSNR(SNRSource source) {
        return source == SNRSource::NONE ||
               source == SNRSource::IDLE_IN_BAND ||
               source == SNRSource::OFDM_BROADBAND ||
               source == SNRSource::MCDPSK_IN_BAND;
    }

    // ── #58 increment 3: connect-SNR pool accessors (all knob-gated, default-OFF) ──
    // Tc for decorrelation clustering / wire freshness — the SAME derivation chain as
    // the retx trough pacing (measured Doppler when the coherence verdict is valid,
    // else the ITU-R design Doppler of the coherence-adjusted fading class); no tuned
    // ms constants. AWGN class => UINT32_MAX (all readings are one stationary cluster).
    uint64_t connectSnrPoolTcMs() const {
        const float doppler_hz = coherence_valid_ ? coherence_doppler_hz_ : 0.0f;
        return connection_policy::coherenceTimeMsForDoppler(
            connection_policy::retxTroughDopplerHz(doppler_hz, fading_index_,
                                                   coherence_score_, coherence_valid_));
    }
    // Entry-pick value: clustered dB-mean of the handshake population (data-aided
    // MCDPSK_IN_BAND readings; no age gate — the pool is cleared at connection
    // boundaries, so its horizon IS the handshake scope). Falls back to the scalar
    // when the knob is off or the pool has no qualifying reading (e.g. training-only
    // decodes) — knob-OFF is byte-identical by construction.
    float rateSelectionSnrDb() const {
        if (!connection_policy::connectSnrPoolEnabled()) {
            return measured_snr_db_;
        }
        const float agg = connect_snr_pool_.clusteredDbMeanDb(
            connectSnrPoolTcMs(), /*handshake_only=*/true, /*max_age_ms=*/UINT64_MAX);
        return std::isfinite(agg) ? agg : measured_snr_db_;
    }
    // Entry-pick FADING, parallel to rateSelectionSnrDb() (#58 increment 4,
    // BUG-CONNECT-FADING-VARIANCE): the fading pooled from the same handshake
    // readings, clustered by the same Tc. A SINGLE CONNECT frame's fading at
    // Watterson Good scatters 0.24-0.74 (48-entry rig MPG@20 ledger, 18.8%
    // false-Moderate) around the Good/Moderate boundary 0.65 — one 0.66 reading
    // mis-classed a true-Good channel Moderate and entered at QPSK R1/4 (see
    // docs/CONNECT_ENTRY_CALIBRATION_2026_07_03.md). Falls back to the scalar
    // fading_index_ when the knob is off or no qualifying reading carries fading
    // — knob-OFF is byte-identical by construction. Entry-pick consumers ONLY;
    // non-entry uses of fading_index_ (window sizing, file-block sizing, wire
    // freshness, Tc derivation) intentionally keep the live scalar.
    float rateSelectionFadingIndex() const {
        if (!connection_policy::connectSnrPoolEnabled()) {
            return fading_index_;
        }
        const float agg = connect_snr_pool_.clusteredFadingIndex(
            connectSnrPoolTcMs(), /*handshake_only=*/true, /*max_age_ms=*/UINT64_MAX);
        return std::isfinite(agg) ? agg : fading_index_;
    }
    // data_aided flag matching rateSelectionSnrDb()'s VALUE: the pool aggregate is
    // data-aided by construction (population contract); the scalar fallback keeps the
    // scalar's flag. Keeps the connectSelectionSnrDb saturation-bound semantics exact.
    bool rateSelectionSnrDataAided() const {
        if (!connection_policy::connectSnrPoolEnabled()) {
            return measured_snr_data_aided_;
        }
        const float agg = connect_snr_pool_.clusteredDbMeanDb(
            connectSnrPoolTcMs(), /*handshake_only=*/true, /*max_age_ms=*/UINT64_MAX);
        return std::isfinite(agg) ? true : measured_snr_data_aided_;
    }
    // MODE_CHANGE wire embed: pool mean over ALL qualifying readings younger than
    // 3*Tc, else the -10 dB stale sentinel (wire byte 0 — the receiver's existing
    // "peer SNR n/a" rendering; no receiver change). Knob-OFF => the raw scalar,
    // byte-identical to main.
    float wireSnrDb() const {
        if (!connection_policy::wireSnrFreshEnabled()) {
            return measured_snr_db_;
        }
        const uint64_t tc_ms = connectSnrPoolTcMs();  // <= UINT32_MAX: 3*Tc can't overflow
        const float agg = connect_snr_pool_.clusteredDbMeanDb(
            tc_ms, /*handshake_only=*/false,
            /*max_age_ms=*/tc_ms * connection_policy::kConnectWireSnrFreshTcMultiple);
        return std::isfinite(agg) ? agg : connection_policy::kConnectSnrStaleSentinelDb;
    }

    // Same freshness contract as wireSnrDb for the MODE_CHANGE fading byte: the
    // sender's fading_index_ freezes between sparse control decodes exactly like
    // the SNR did (rig W5b/W8: peer_fading pinned at 0.42/0.70 for 300+ s on the
    // wire). Stale -> -1.0, which encodeFadingIndex maps to the existing wire
    // "unknown" byte 0 -> the receiver's n/a render; zero receiver change.
    float wireFadingIndex() const {
        if (!connection_policy::wireSnrFreshEnabled()) {
            return fading_index_;
        }
        const uint64_t fresh_ms =
            connectSnrPoolTcMs() * connection_policy::kConnectWireSnrFreshTcMultiple;
        return ms_since_fading_update_ <= fresh_ms ? fading_index_ : -1.0f;
    }

    ConnectionConfig config_;
    ConnectionState state_ = ConnectionState::DISCONNECTED;

    // Callsigns
    std::string local_call_;
    std::string remote_call_;
    std::string pending_remote_call_;

    // Remote station hashes (for routing when callsign unknown)
    uint32_t remote_hash_ = 0;
    uint32_t pending_remote_hash_ = 0;

    // Pending forced modes from incoming CONNECT (for manual accept flow)
    Modulation pending_forced_modulation_ = Modulation::AUTO;
    CodeRate pending_forced_code_rate_ = CodeRate::AUTO;
    uint8_t pending_forced_cw_count_ = 0;  // 0 = AUTO (responder chooses)

    // Exact operator fields placed in our outbound CONNECT.  Environment forces are
    // resolved into the wire request here (they do not otherwise live in config_), and
    // the CONNECT_ACK must echo every non-AUTO field or the initiator aborts rather than
    // running a split PHY.  Also supplies the QSO-scoped adaptation pin after env changes.
    Modulation outbound_forced_modulation_ = Modulation::AUTO;
    CodeRate outbound_forced_code_rate_ = CodeRate::AUTO;

    // Waveform mode
    WaveformMode narrowband_override_ = WaveformMode::AUTO;  // Session-scoped, cleared on disconnect/reset
    WaveformMode negotiated_mode_ = WaveformMode::OFDM_CHIRP;
    uint8_t remote_capabilities_ = ModeCapabilities::OFDM_CHIRP;
    WaveformMode remote_preferred_ = WaveformMode::OFDM_CHIRP;
    bool phy_mask_v1_negotiated_ = false;

    // Data modulation and code rate (adaptive)
    Modulation data_modulation_ = Modulation::DQPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;
    int data_frame_cw_count_ = v2::kDefaultFixedFrameCodewords;
    // Transfer-scoped arms for the default-off long-LDPC file experiments. They
    // are set before FileTransferController::startSend(), because chunk capacity
    // is selected before that call changes the file state to SENDING. The active
    // wire geometry remains exactly rung-gated; enabling an env knob cannot alter
    // control, interactive, forced-CW, narrowband, or nonmatching DATA traffic.
    bool experimental_8psk_long_ldpc_transfer_active_ = false;
    bool experimental_qpsk_r34_long_ldpc_transfer_active_ = false;
    LadderRungId data_ladder_rung_id_ = LadderRungId::UNKNOWN;
    uint16_t mode_change_seq_ = 0;  // Sequence number for MODE_CHANGE frames
    float measured_snr_db_ = 15.0f;  // Routed SNR measured by modem (see source).
    float physical_channel_mean_db_ = 0.0f;   // §6 entry cap (dial convention)
    float physical_channel_spread_db_ = 0.0f;
    uint32_t physical_channel_n_ = 0;
    SNRSource measured_snr_source_ = SNRSource::NONE;
    bool measured_snr_data_aided_ = false;  // measured_snr_db_ is the data-aided MC-DPSK estimate
    bool measured_snr_valid_ = false;
    // #58 increment 3 (BUG-CONNECT-SNR-VARIANCE): ring of the last qualifying SNR
    // readings (see ConnectSnrPool contract in connection_policy.hpp). Aged from
    // Connection::tick (modem-time); cleared in reset()/enterDisconnected() so
    // nothing leaks across sessions. Consumed via rateSelectionSnrDb()/wireSnrDb().
    connection_policy::ConnectSnrPool connect_snr_pool_;
    // ULTRA_CONNECT_PICK_DEFER one-shot: this handshake already spent its single
    // CONNECT_ACK withhold. Cleared at connection boundaries (reset/enterConnected/
    // enterDisconnected) — one defer per handshake, never a defer loop.
    bool connect_pick_deferred_once_ = false;
    float fading_index_ = 0.0f;      // Fading index (0-2, > 0.65 = significant fading)
    // Modem-time ms since fading_index_ was last measured (saturating; aged in
    // tick beside the SNR pool). Starts stale so a never-measured value can't
    // masquerade as fresh on the wire.
    uint32_t ms_since_fading_update_ = 0x7FFFFFFF;
    float coherence_score_ = 0.0f;   // Doppler coherence (|H|^2 autocorr); high=Good slow fading
    float coherence_doppler_hz_ = 0.0f;  // measured RMS Doppler (Hz) riding the coherence feed;
                                         // 0 = not estimable (retx trough pacing falls back to
                                         // the fading-index-derived design Doppler)
    bool coherence_valid_ = false;   // true once enough OFDM data pooled for a trusted verdict

    // MODE_CHANGE timeout/retry tracking
    bool mode_change_pending_ = false;
    uint32_t mode_change_timeout_ms_ = 0;
    int mode_change_retry_count_ = 0;
    Modulation pending_modulation_ = Modulation::DQPSK;
    CodeRate pending_code_rate_ = CodeRate::R1_4;
    uint8_t pending_cw_count_ = 0;  // 0 = use applyDataMode's default
    LadderRungId pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    float pending_snr_db_ = 15.0f;
    float pending_fading_index_ = 0.0f;
    uint8_t pending_reason_ = 0;
    // DESC-SWITCH (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.1, knob
    // ULTRA_DESCRIPTOR_MODE_SWITCH, read ONCE in the ctor like ULTRA_ARQ_MOVE_EPOCH;
    // default OFF = byte-identical; SEMANTICS-BREAKING lockstep when ON — both ends).
    bool descriptor_mode_switch_enabled_ = false;
    // One-shot §2.6-arm-3 mitigation: the first burst after a descriptor-committed
    // switch MUST carry a full chirp+LTS anchor (fresh |H| under the new pilot/carrier
    // geometry). Armed by commitLocalModeSwitch, consumed by flushBurstBuffer's
    // on_transmit_burst_ force_full_preamble argument; cleared at session boundaries.
    bool desc_switch_full_anchor_pending_ = false;
    // Default-OFF progress-bearing partial-SACK experiment.  Both peers may enable
    // independently without changing legacy behavior: an enabled receiver stamps
    // exact completed-group provenance into reserved drive-advisory value 3, which old
    // senders already treat as HOLD; an enabled sender fails closed unless it sees that
    // stamp plus fresh partial progress under unchanged physical geometry.
    bool partial_sack_descriptor_repair_enabled_ = false;
    struct PartialSackRoundContext {
        bool descriptor_light_repair_eligible = false;
        size_t physical_frames = 0;
        size_t arq_frames = 0;  // excludes addressed-away ULPAD geometry members
        // Exact serialized identities from this physical round.  The cumulative ARQ
        // progress counter is deliberately not a physical k/N counter: filling an old
        // base hole can retire previously SACKed frames again and legitimately report
        // progress > this round's N.  Retain the identities so the next ACK can ask the
        // ARQ which members of THIS group remain unacknowledged.
        std::vector<Bytes> arq_frame_identities;
        WaveformMode mode = WaveformMode::OFDM_CHIRP;
        Modulation modulation = Modulation::DQPSK;
        CodeRate code_rate = CodeRate::R1_4;
        int logical_cw = 0;
        int physical_cw = 0;
        int lifting_z = 27;
    } partial_sack_last_round_;
    // Synchronous, one-refill ownership token.  It is armed only inside an accepted
    // tone-ACK callback and consumed before the corresponding physical request can be
    // CCA-deferred; it never survives a deferred/no-op refill.
    bool partial_sack_descriptor_repair_scope_ = false;
    // RX-RATE-CMD Phase 2 (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md §5.2, knob
    // ULTRA_RX_RATE_CMD, read ONCE in the ctor like ULTRA_DESCRIPTOR_MODE_SWITCH;
    // default OFF = byte-identical — the tone-ACK rung_cmd bits stay 0 and the CRC
    // span stays 28 bits). SEMANTICS-BREAKING lockstep when ON (live command bits +
    // widened CRC span in tone_burst_payload — both ends must run it; requires
    // Phase 1 for the descriptor-committed consume path, falls back to the legacy
    // MODE_CHANGE exchange otherwise).
    bool rx_rate_cmd_enabled_ = false;
    // Four retries at the single-control geometry deadline. Wide OFDM defaults to
    // 8 s (also >= one capped coherence interval), and retry egress is carrier-sense
    // gated. The receiver sends one immediate ACK; duplicate requests earn one
    // fresh re-ACK, so every retry is useful time diversity rather than an ACK train
    // inside one fade.
    static constexpr int MODE_CHANGE_MAX_RETRIES = 4;
    struct ModeChangeAckRepeatJob {
        Bytes frame_data;
        uint16_t seq = 0;
        uint32_t timer_ms = 0;
        int copy_index = 0;
    };
    std::deque<ModeChangeAckRepeatJob> mode_change_ack_repeat_jobs_;

    // ==== BUG-MC-RETRY-SPURIOUS fix 3 (receiver MODE_CHANGE dedup) — 2026-07-04 ====
    // Last MODE_CHANGE actually APPLIED this connection. A re-arriving copy with the
    // same (seq, mod, rate) is the sender's diversity copy / spurious retry: it means
    // the sender may have missed our ACKs, so the calibrated response is ONE re-ACK
    // copy — not a re-apply, not a GUI re-notify, not a fresh fading-aware repeat set
    // (handleModeChange, connection_handlers.cpp). Dedup keys on the full tuple, not
    // seq alone: mode_change_seq_ is never reset per-session on the sender, but a peer
    // RESTART restarts its counter — the tuple guards that seq-reuse corner. State is
    // written ONLY in connection_handlers.cpp (handleModeChange applies; handleConnect/
    // handleConnectAck clear at session establishment). INTEGRATION NOTE for the main
    // session: ideally ALSO cleared in enterConnected()/enterDisconnected()
    // (connection.cpp — deliberately not edited by the parallel session that added this).
    bool last_applied_mode_change_valid_ = false;
    uint16_t last_applied_mode_change_seq_ = 0;
    Modulation last_applied_mode_change_mod_ = Modulation::DQPSK;
    CodeRate last_applied_mode_change_rate_ = CodeRate::R1_4;
    // ==== end BUG-MC-RETRY-SPURIOUS fix 3 block ====

    // ARQ for reliable data transfer (Selective Repeat for higher throughput)
    SelectiveRepeatARQ arq_;
    fec::SoftCombineBuffer soft_combine_harq_;

    // File transfer controller
    FileTransferController file_transfer_;
    std::optional<std::string> queued_file_path_;
    uint64_t queued_file_order_ = 0;
    uint64_t next_queued_operation_order_ = 1;

    // Message fragmentation (TX) - splits long messages across multiple ARQ frames
    struct QueuedPayload {
        Bytes data;
        bool binary_payload = false;
        uint64_t message_token = 0;
        uint64_t enqueue_order = 0;
    };
    std::deque<QueuedPayload> queued_payloads_;
    std::vector<Bytes> pending_tx_fragments_;
    std::vector<uint8_t> pending_tx_fragment_flags_;  // Per-fragment flags (for sendMessages batch)
    std::vector<v2::FrameType> pending_tx_fragment_types_;  // DATA_START/CONT/END for binary streams
    std::vector<uint64_t> pending_tx_fragment_message_tokens_;
    size_t next_fragment_idx_ = 0;
    size_t acked_fragment_count_ = 0;  // Actual ACKs received (vs next_fragment_idx_ = submitted)
    struct OutboundMessageTxRecord {
        uint64_t token = 0;
        std::string text;
        std::string remote_call;
        size_t expected_fragments = 0;
        size_t assigned_fragments = 0;
        uint16_t first_seq = 0;
        uint16_t last_seq = 0;
        bool first_seq_valid = false;
        bool submitted_reported = false;
        bool terminal_reported = false;
        std::chrono::steady_clock::time_point submitted_at{};
    };
    std::deque<OutboundMessageTxRecord> outbound_message_tx_records_;
    uint64_t next_outbound_message_token_ = 1;
    uint64_t arq_submit_message_token_ = 0;
    std::deque<MessageTxStatusEvent> pending_message_tx_status_events_;
    size_t message_tx_status_deferral_depth_ = 0;
    bool message_tx_status_dispatch_active_ = false;

    // ARQ records a sequence before invoking the physical transport. Hold
    // application-visible status callbacks until the complete logical submit
    // stack has unwound, so a SUBMITTED callback may safely abort/reset/re-enter.
    class MessageTxStatusDeferralGuard {
    public:
        explicit MessageTxStatusDeferralGuard(Connection& owner)
            : owner_(owner) {
            ++owner_.message_tx_status_deferral_depth_;
        }
        ~MessageTxStatusDeferralGuard() {
            if (owner_.message_tx_status_deferral_depth_ > 0) {
                --owner_.message_tx_status_deferral_depth_;
            }
            owner_.drainMessageTxStatusEvents();
        }
        MessageTxStatusDeferralGuard(const MessageTxStatusDeferralGuard&) = delete;
        MessageTxStatusDeferralGuard& operator=(
            const MessageTxStatusDeferralGuard&) = delete;

    private:
        Connection& owner_;
    };

    // Message reassembly (RX) - accumulates fragments into complete messages
    Bytes rx_reassembly_buffer_;
    bool rx_reassembly_active_ = false;
    bool rx_reassembly_binary_ = false;
    uint8_t rx_reassembly_epoch_ = 0;

    // Connection timing
    uint32_t timeout_remaining_ms_ = 0;
    int connect_retry_count_ = 0;
    uint32_t connected_time_ms_ = 0;

    // Disconnect retransmission (initiator side)
    Bytes disconnect_frame_;                // Cached DISCONNECT frame for retransmission
    int disconnect_retry_count_ = 0;
    uint32_t disconnect_retransmit_ms_ = 0; // Time until next retransmit
    static constexpr uint32_t DISCONNECT_RETRANSMIT_FLOOR_MS = 5000;
    static constexpr int DISCONNECT_MAX_RETRIES = 3;

    // Disconnect grace period (responder side)
    // After receiving DISCONNECT, stay connected briefly and re-send ACK
    // periodically to ensure the initiator gets it (fading can lose frames)
    bool disconnect_teardown_active_ = false;
    bool disconnect_pending_ = false;
    uint32_t disconnect_pending_ms_ = 0;
    uint32_t disconnect_ack_retransmit_ms_ = 0; // Time until next ACK re-send
    uint32_t disconnect_ack_epoch_elapsed_ms_ = 0;
    int disconnect_ack_repeat_count_ = 0;
    Bytes disconnect_ack_frame_;            // Cached ACK for re-sending
    static constexpr uint32_t DISCONNECT_GRACE_LEGACY_MS = 5000;
    static constexpr uint32_t DISCONNECT_ACK_RETRANSMIT_FLOOR_MS = 2000;
    static constexpr int DISCONNECT_ACK_MAX_PROACTIVE_REPEATS_CAP = 2;

    // Calling waveform for PING/CONNECT control frames.
    WaveformMode connect_waveform_ = WaveformMode::MC_DPSK;

    // Statistics
    ConnectionStats stats_;

    // Burst mode TX buffering (OFDM and MC-DPSK DATA)
    std::vector<Bytes> burst_tx_buffer_;
    bool burst_mode_active_ = false;
    TransmitBurstCallback on_transmit_burst_;

    // ARQ timeout dispatch is transactional. SelectiveRepeatARQ publishes exact frame
    // identities during tick() without consuming retry/Karn/terminal state; Connection
    // evaluates its stuck/collapse escape at the safe post-tick boundary and then commits
    // or cancels them. Otherwise the escape can queue a replacement burst behind 7-10
    // seconds of waveform it has just declared obsolete (or a discarded intent can burn
    // retry budget without ever going on air).
    bool arq_tick_in_progress_ = false;
    std::vector<Bytes> staged_timeout_batch_;
    Modulation staged_timeout_mod_ = Modulation::QPSK;
    CodeRate staged_timeout_rate_ = CodeRate::R1_4;
    int staged_timeout_cw_ = 0;

    // TRANSPORT MERGE (2026-06-06): "burst framing on" — the unified arq_ path always
    // bursts+interleaves OFDM data (encodeBurstLight + BURST_HEADER descriptor). The
    // legacy BurstStopAndWaitController group controller is removed. This flag stays true
    // (it gates Z-selection / descriptor / CONNECT_ACK-rescue / rate); collapsing it to a
    // constant is a follow-up cleanup (docs/REMOVAL_BACKLOG.md R1b).
    bool use_burst_transport_ = true;

    // Half-duplex INTERACTIVE mode (the TNC / Winlink-B2F path). The default
    // one-way burst file push lets the sole sender bypass the ISS/IRS turn gate
    // (design §14.27: ALPHA sends, BRAVO only listens+ACKs). That is WRONG for a
    // bidirectional B2F exchange where BOTH stations alternately transmit — they
    // key up uncoordinated and collide on the half-duplex channel. When this flag
    // is set, sendFile() keeps the turn gate: a station only starts its burst when
    // it holds local_data_turn_, otherwise it queues + TURN_REQUESTs, and the peer
    // yields (TURNOVER) so the two directions serialize. Set true by ultra_tnc;
    // false for the GUI one-way file transfer.
    bool half_duplex_interactive_ = false;
    // Half-duplex INTERACTIVE: VARA-HF convention — the ISS with an empty TX buffer
    // turns the link over to the IRS. For Winlink-B2F the connecting station (initiator/
    // ISS) has nothing to send first (the answering station sends the SID), so it yields
    // the DATA turn to the responder once connected + settled. Done once per session.
    bool interactive_initiator_yield_done_ = false;
    uint32_t interactive_yield_log_throttle_ms_ = 0;
    uint32_t interactive_anchor_rearm_ms_ = 0;

    // §14.36 Phase 5c BER-driven per-block rate adaptation. The SENDER runs the controller
    // on the receiver's per-group decode headroom (carried on the GROUP_ACK; a GROUP_NACK
    // feeds quality 0 -> step down).
    void applyAdaptiveRateFeedback(float quality);
    // Mid-transfer rate/mod MOVES are live. DEFAULT-ON for connected wideband OFDM
    // (2026-07-02 fade-riding ladder); opt out with ULTRA_RATE_ADAPT=0 or pin with
    // ULTRA_LOCK_RATE=1. adaptive_rate_enabled_/ULTRA_ADAPTIVE_RATE only enables the
    // feedback computation + GUI bar; this gates the rate physically changing.
    bool rateAdaptationActive() const;
    // Polled from tick(): a frame stuck at a too-aggressive rate (the fade troughs keep killing it,
    // so it produces no group ACK and the clean-boundary gate can't help) escape-drops one rung.
    void maybeEscapeStuckFrame();
    // ── Retx trough pacing + collapse-conditioned escape ──────────────────────────────
    // docs/RETX_PACING_DESIGN_2026_07_03.md. Both default-OFF (ULTRA_RETX_TROUGH_PACING /
    // ULTRA_COLLAPSE_ESCAPE_ROUNDS) ⇒ byte-identical unset: no hold is ever armed, no ARQ
    // timer touched, no escape fired; only the inert round counter ticks.
    //
    // Shared escape ACTION (§2.2 — refactored out of maybeEscapeStuckFrame, reuse not fork):
    // QAM16 (either rate) drops STRAIGHT to QPSK R3/4 + noteQam16Demoted(2); otherwise one
    // code-rate rung + noteRungFailed. Guards (mode_change_pending_, floor, in-flight) are
    // the CALLER's responsibility — both callers keep first refusal.
    void executeEscapeDrop(const char* trigger);
    // Polled from tick() beside maybeEscapeStuckFrame (never fired from inside an ARQ
    // callback): ≥ N consecutive zero-progress rounds with ≥⌈burst_cap/2⌉ frames in flight
    // ⇒ the WINDOW is collapsing (the g43/rig frozen-base signature) ⇒ escape the rung.
    // g42-protective by construction: any delivered/SACKed frame resets the round counter,
    // so a lone straggler retrying amid deliveries can never trip it (§2.3).
    void maybeCollapseEscape();
    // §1.3 scope gate: CONNECTED wideband OFDM (OFDM_CHIRP only — MC-DPSK/OFDM_NARROW are
    // explicitly out of scope, §6.2) on the unified tone-burst path, file SENDING with
    // in-flight bytes.
    bool retxPacingScopeActive() const;
    // §1.1 round accounting: progress_frames = the ARQ's lastAckProgressFrames() at a round
    // boundary (>0 progress ⇒ reset streak + early-release any hold; 0 ⇒ zero-progress round
    // ⇒ count it and, knob-gated, arm the §1.2 deferral on BOTH triggers; <0 ⇒ no fresh ack
    // (dup/stale) ⇒ not a round).
    void noteArqRoundOutcome(int progress_frames, const char* origin);
    // TROUGH AMNESTY (ULTRA_TROUGH_AMNESTY): on the first progress-bearing ack after a
    // zero-progress episode, restore the pre-episode rung (trough demotes are not rate
    // evidence — see the member comment at trough_episode_active_). Called from the
    // toneburst-ack path inside the defer-refill bracket, after noteArqRoundOutcome.
    void maybeTroughAmnesty(int progress_frames, uint8_t rung_cmd);
    struct PhysicalDataRoundTiming {
        uint64_t waveform_samples = 0;
        uint64_t keyed_samples = 0;
        // RF waveform extent before ModemEngine's PA/audio guards.
        uint32_t waveform_airtime_ms = 0;
        // Complete processed buffer duration (waveform + configured lead/tail).
        // This is the physical play-head duration used by noteDataBurstKeydown.
        uint32_t airtime_ms = 0;
        uint32_t ack_timeout_ms = 0;
        uint32_t total_codewords = 0;
        uint8_t max_codewords = 0;
        size_t variable_frames = 0;
    };
    // For OFDM_CHIRP, read each serialized frame's advertised total_cw and derive the exact physical
    // airtime/RTO. Variable frames (>16 CW) and every standalone frame use the
    // encoder's z=27 path; fixed multi-frame bursts use the active burst lifting Z.
    PhysicalDataRoundTiming physicalDataRoundTiming(
        const std::vector<Bytes>& transmitted_frames,
        bool force_full_group_start = false) const;
    // Record the modeled end-of-key-down time of an OFDM data burst (flush time + exact
    // serialized-frame airtime) — the reference for T_defer's subtraction (§1.2).
    // Recording is unconditional and behavior-free; all decisions stay knob-gated. Wall
    // clock is correct here: the faithful gate and the rig both run wall==sample time.
    uint32_t noteDataBurstKeydown(const std::vector<Bytes>& transmitted_frames,
                                  bool force_full_group_start = false);
    uint32_t elapsedSinceLastDataBurstEndMs() const;
    // Consecutive zero-progress resend rounds at the current rung (§1.1); reset on ANY
    // progress, on mode change (applyDataMode — a new era), enterConnected and reset().
    int zero_progress_rounds_ = 0;
    // Consecutive escape drops with NO intervening ACK progress. Used to bound a
    // collapse episode through a fade null; every escape itself uses synchronized
    // MODE_CHANGE because silence does not prove descriptor receipt. Reset on any
    // ACK progress (noteArqRoundOutcome) and at QSO boundaries.
    int consecutive_escape_drops_ = 0;
    // TROUGH AMNESTY (ULTRA_TROUGH_AMNESTY, 2026-07-05): snapshot of the rung ACTIVE
    // when a zero-progress episode began. A fade null is time-bounded — a rung proven
    // clean seconds before the null is not invalidated by it, yet the escape + EMA
    // demotes (down) and ssthresh + climb streaks (up) treat trough evidence as rate
    // evidence (F89/F91: a ~20 s null sentenced transfers to minutes of R1/4 while
    // delivering 8/8 q=0.9+). When the episode ends (first progress-bearing ack),
    // restore this rung directly; if the channel genuinely worsened, one cheap crater
    // (~3.5 s rung-command demote) re-drops it — bounded loss, minutes gained.
    bool trough_episode_active_ = false;
    Modulation pre_episode_mod_ = Modulation::QPSK;
    CodeRate pre_episode_rate_ = CodeRate::R1_4;
    // Active trough-pacing hold (sender-local, ticks down in the CONNECTED tick BEFORE
    // runDeferredArqRefill; gates the turn refill — trigger #1. The slot-RTO trigger #2 is
    // gated by SelectiveRepeatARQ::deferPendingRetransmits armed alongside this).
    uint32_t retx_pace_hold_ms_ = 0;
    // Modeled end time of the last OFDM data-burst key-down (see noteDataBurstKeydown).
    std::chrono::steady_clock::time_point last_data_burst_end_{};
    bool last_data_burst_end_valid_ = false;
    bool adaptive_rate_enabled_ = true;  // default ON: drives the GUI "Adapt:" observability bar +
                                         // decode-headroom quality feedback. The actual rate CHANGE
                                         // is gated by rateAdaptationActive() (default-ON for
                                         // wideband OFDM since 2026-07-02; ULTRA_RATE_ADAPT=0 or
                                         // ULTRA_LOCK_RATE=1 opts out). Opt out of the feedback
                                         // computation itself with ULTRA_ADAPTIVE_RATE=0.
    RateController rate_controller_;
    uint8_t pending_ack_quality_q_ = 0xFF;  // RX: byte to stamp on the next GROUP_ACK
    float last_group_quality_ = -1.0f;      // GUI: most recent group decode headroom
    // Software-ALC receiver-side state (per-connection; reset in enterConnected /
    // enterDisconnected). low_streak counts CONSECUTIVE fresh LOW verdicts (>=
    // kAlcLowStreakForUp -> advise "up"); clipped latches the latest verdict's clip
    // signature (advise "down" IMMEDIATELY); seq_seen dedups stale verdict re-feeds.
    int rx_level_low_streak_ = 0;
    bool rx_level_clipped_ = false;
    uint32_t rx_level_verdict_seq_seen_ = 0;
    bool rx_level_verdict_pending_for_group_ = false;
    // ── RX-RATE-CMD Phase 2 (ULTRA_RX_RATE_CMD) state ────────────────────────────
    // RECEIVER side: standing rung command stamped on every outgoing tone-burst ACK
    // (bits 42-43). Recomputed per burst-group event in updateRxRateCommandFromGroup:
    //   - set to kRungCmdDownHard on a TOTAL crater (frame_mask == 0: zero frames
    //     delivered — the same zero-progress evidence class the collapse escape
    //     counts) at QAM16 (the modulation whose demote-on-one-bad-group policy is
    //     already codified sender-side, kQam16DemoteBadStreak = 1);
    //   - cleared the moment any group delivers frames (the crater state ended —
    //     a stale demote command must not ride a healthy channel's ACKs), and in
    //     applyDataMode when mod/rate actually change (the sender's adoption
    //     observed — the once-per-committed-move idempotency latch).
    // Repeated ACK emits between crater and adoption re-carry the SAME command
    // (diversity against ACK loss); the SENDER dedups by group_seq (below), which
    // is frozen during a crater (zero progress ⇒ base cannot advance), so the
    // command acts at most once per move. Only ever non-zero when the knob is ON.
    uint8_t rx_rate_cmd_pending_ = 0;
    // SENDER side: last group_seq whose non-zero rung command was consumed
    // (-1 = none this connection) — the drive_advisory dedup pattern. NOT reset on
    // mode change (a straggler duplicate ACK of the consumed command must stay
    // deduped after the demote commits); reset in enterConnected/enterDisconnected.
    int rx_rate_cmd_seq_seen_ = -1;
    // WAITING-REBASE voice identity/dedup (BUG-UNANCHORED-SILENCE-ESCAPE,
    // design §5.3). The unified BURST_HEADER transport seq is intentionally not
    // an ARQ sequence and currently stays zero, so it cannot identify successive
    // physical outcomes. The receiver instead stamps each locally-created voice
    // event with this dedicated mod-64 identity; cached RF repeats retain the
    // already-stamped payload. The sender remembers only the last identity so a
    // cached copy cannot re-expire the base slot. Zero-progress reset still runs
    // on EVERY voice copy (cheap, idempotent, and the point).
    uint8_t rebase_voice_event_seq_ = 0;
    int rx_rebase_voice_seq_seen_ = -1;
    // PARTIAL-CRATER latency fix (2026-07-04, F27): consecutive all_ok=false groups
    // at QAM16 (receiver side). 2+ -> DownOne command (a total crater still commands
    // immediately). Reset on any clean group, non-QAM16, and session boundaries.
    int qam16_rx_bad_streak_ = 0;
    // HALF-OPEN timeout accumulator (2026-07-04, F29): ms in CONNECTED with the
    // responder handshake never confirmed; 240 s -> release the session.
    uint32_t responder_half_open_ms_ = 0;
    // Receiver emit-side decision (called from onBurstGroupReceived before the
    // group's tone-burst ACK is emitted) and sender consume-side action (called
    // from onToneBurstAck; mod/rate_at_ack = the mode snapshot taken BEFORE
    // applyAdaptiveRateFeedback ran, so one ACK's evidence moves the rung at most
    // once). Both are hard no-ops while the knob is OFF.
    // BUG-RESPONDER-HANDSHAKE-NEVER-CONFIRMS (2026-07-04): single confirm site,
    // called from BOTH the classic frame path and the burst-group path (see .cpp).
    // The predicate prevents a mere PHY sync, an empty/failed group, or a frame from
    // another station from confirming a half-open responder session.
    bool isAuthoritativeResponderHandshakeFrame(const v2::HeaderInfo& header) const;
    void maybeConfirmResponderHandshake(const char* evidence);
    void updateRxRateCommandFromGroup(bool all_ok, uint16_t frame_mask);
    void maybeApplyRxRateCommand(uint8_t cmd, uint8_t group_seq,
                                 Modulation mod_at_ack, CodeRate rate_at_ack);

    // ── RX-AUTHORITY (ULTRA_RX_RATE_AUTHORITY, 2026-07-05) ──────────────────────
    // The receiver measures, the receiver DECIDES: per burst group it maps its
    // fresh channel observation (broadband SNR EMA + coherence-adjusted fading,
    // fed by setBurstChannelObservation from the decoder's lock-free atomics)
    // through selectCoherentOFDM and stamps the resulting CANONICAL RUNG INDEX
    // (waveform_selection.hpp kRungIdx*) into the ACK's reinterpreted
    // [rate_hint(3)|rung_cmd(2)] bits. The sender obeys (descriptor commit); its
    // own mid-transfer drivers are inert under the knob. Sender ack-SILENCE
    // escapes stay live (no command crosses a blackout).
    // RECEIVER: fresh observation (never aged — each group overwrites).
    float burst_obs_snr_db_ = -1.0f;      // <0 = never fed this connection
    float burst_obs_fading_ = -1.0f;
    float burst_obs_coh_score_ = 0.0f;
    bool burst_obs_coh_valid_ = false;
    float burst_obs_doppler_hz_ = -1.0f;
    // RECEIVER: this group's radio-agnostic decision-directed EVM usable-SNR (dB),
    // fed by setBurstEvmObservation from the decoder BEFORE onBurstGroupReceived.
    // <0 = never fed / no OFDM decode this group. Consumed by the EVM demote authority
    // (ULTRA_EVM_DEMOTE) in updateRxAuthorityCommand — a constant-free, non-inflating
    // clamp that reads usable dB directly (no dial/hardware offset).
    float burst_obs_evm_snr_db_ = -1.0f;
    // RECEIVER: canonical rung this end commands (0 = none); re-stamped on every
    // ACK emit until superseded by the next group's verdict.
    uint8_t rx_authority_cmd_ = 0;
    // RECEIVER: fade-averaging ring for the verdict SNR. The per-frame broadband
    // EMA is a fade SNAPSHOT (measured ±5 dB swing on a dial-20 Good channel) —
    // rate anchors are calibrated on dial-equivalent SNR, so the verdict input is
    // the dB mean over the last few groups (~30 s ≈ many Tc), not the instant
    // fade state. Entries age out via tick (stale channel must not steer).
    static constexpr size_t kRxAuthObsRing = 6;
    static constexpr uint32_t kRxAuthObsMaxAgeMs = 180000;
    float rx_auth_obs_db_[kRxAuthObsRing] = {0};
    uint32_t rx_auth_obs_age_ms_[kRxAuthObsRing] = {0};
    size_t rx_auth_obs_count_ = 0;
    size_t rx_auth_obs_next_ = 0;
    // RECEIVER: ring of recent per-group EVM usable-SNR samples, for the CONFIDENCE-GATED
    // form of the clamp (ULTRA_EVM_DEMOTE_CONFIDENT). The per-group estimate is NOISY —
    // measured sd 3.1-4.4 dB on the rig — while adjacent rung EVM floors are only
    // 1.2-3.1 dB apart, so a single sample cannot resolve which rung is supported
    // (sigma/sqrt(N) <= gap/2 needs N ~= 27 at the tightest gap). Clamping on one sample
    // therefore scatters the target across 2-3 rungs from noise alone, which is exactly
    // what the 2026-07-26 A/B measured (rung commands 15 vs 7 per run against the
    // ladder's SMOOTHED snr_avg). Same depth as kRxAuthObsRing so both inputs to the
    // verdict are averaged over comparable evidence.
    float rx_auth_evm_ring_[kRxAuthObsRing] = {0};
    size_t rx_auth_evm_count_ = 0;
    size_t rx_auth_evm_next_ = 0;
    // RECEIVER: per-rung crater-margin memory — the anchor map is a PRIOR; the
    // observed crater rate at a rung is the POSTERIOR (second-probe finding: at
    // avg 22 dB the map re-commanded 16QAM R2/3 after every crater — anchor 20
    // said yes, the decode said no every ~2.5 groups). A crater AT a rung raises
    // the extra dB margin needed to re-command it (+2, cap 6); every clean group
    // decays all penalties (0.25) — the fade epoch ends, the evidence expires.
    // Receiver-side analogue of the sender's ssthresh, measured where the
    // channel actually is. Sized by kRungIdxCount (waveform_selection.hpp).
    float rx_auth_rung_penalty_db_[16] = {0};
    // TWO-CRATER rule (F122 finding: 10 moves/283 s, each paying the full-anchor
    // + requeue-rewind tax): at a ~10 s decision quantum vs Tc 2-4 s a SINGLE
    // crater is an irreducible deep null — the ARQ's job, not the ladder's.
    // Only CONSECUTIVE craters demote (and charge the crater margin).
    int rx_auth_crater_streak_ = 0;
    // DECODE-EVIDENCE class veto (F125): consecutive clean (all_ok) verdicts. A
    // fading-class DEGRADATION is vetoed while decodes stay clean — the rig's
    // fading/coherence estimators carry hardware artifacts and their class
    // thresholds are sim-calibrated (documented 2026-06-17: "[MODERATE] on
    // everything"); a rung delivering q>=0.9 at 24 dB REFUTES "Moderate". The
    // posterior (decode record) vetoes the prior (classifier) — a genuinely
    // degrading channel fails within a couple of groups and lifts the veto.
    int rx_auth_clean_streak_ = 0;
    // FADING input conditioning (F123 finding: the SNR was averaged but the
    // fading/coherence input was a raw per-frame snapshot flapping 0.15<->0.30 —
    // and the anchor table quantizes it into three CLIFF-EDGED columns, so class
    // flaps swung the verdict across whole columns at rock-steady 24 dB SNR:
    // AWGN-snapshot -> 8PSK R3/4, Moderate-snapshot -> QPSK R1/2). Ring-average
    // the coherence-adjusted fading like the SNR, and make the CLASS sticky:
    // a column switch needs the smoothed value's class to persist 2 consecutive
    // verdicts (the Good/Moderate boundary is documented intrinsically fuzzy).
    float rx_auth_fading_ring_[kRxAuthObsRing] = {0};
    int rx_auth_class_sticky_ = 1;      // FadingClass::GOOD — sane starting column
    int rx_auth_class_streak_ = 0;
    float rx_auth_fading_passed_ = 0.3f;  // last fading fed to the map (in-class)
    // RX-AUTHORITY PREDICTIVE state (see setBurstCarrierGammas).
    static constexpr size_t kRxAuthGammaRing = 4;
    static constexpr int64_t kRxAuthGammaMaxAgeMs = 180000;
    std::array<std::vector<float>, kRxAuthGammaRing> rx_auth_gamma_ring_;
    std::array<std::chrono::steady_clock::time_point, kRxAuthGammaRing> rx_auth_gamma_time_;
    size_t rx_auth_gamma_next_ = 0;
    size_t rx_auth_gamma_count_ = 0;
    // F149/F160: clean groups required before up-commands unlock after a
    // confirmed crater (post-episode reality must displace the survivor-biased
    // crest reads; 3 = gate re-betting without taxing recovery at slow rungs).
    static constexpr int kRxAuthClimbDwellGroups = 3;
    int rx_auth_climb_dwell_ = 0;
    // GROUP-SIZE LEVER: consecutive clean (full-retire, no-hole) ack rounds at
    // the current rung; >=2 unlocks the escalated burst-airtime ceiling.
    int burst_clean_group_streak_ = 0;
    // SENDER: last non-zero command index acted on (dedup — ACK repeats re-carry
    // the same command; obey once per distinct target).
    uint8_t tx_authority_last_obeyed_ = 0;
    // MINIMUM RUNG DWELL (ULTRA_RUNG_DWELL_MS), scoped to one QSO. Measured:
    // correlation(rung changes, goodput) = -0.58 over 15 rig runs; held (<=1 change)
    // 1.93 kbps vs churned (>=3) 1.59.
    // Consecutive TOTAL craters that bypass the dwell — a 0/N group is unambiguous, unlike a
    // partial crater, and holding a rung that delivers nothing is a stall not adaptivity.
    static constexpr int kCatastrophicDwellBypassCraters = 2;
    std::chrono::steady_clock::time_point rx_auth_last_change_{};
    bool rx_auth_last_change_valid_ = false;
    void updateRxAuthorityCommand(bool all_ok, float quality, bool full_crater = false,
                                  float delivered_fraction = -1.0f,
                                  uint8_t delivered_frames = 0,
                                  uint8_t group_size = 0,
                                  bool geometry_proven = false);
    void failClosedLatentStartupProbeUnknown(const char* origin);
    void resetAdaptiveDecisionState();

    // GOODPUT-MAXIMIZING RATE CONTROL (ULTRA_GOODPUT_RATE, default OFF). Replaces the
    // SNR-anchor pick + corrective stack in updateRxAuthorityCommand when enabled; inert
    // and untouched when off. See goodput_rate_controller.hpp for why it reads no SNR.
    GoodputRateController goodput_ctl_;
    // ULTRA_LATENT_RATE: outcome-fitted latent state; consumes no SNR, so no calibration
    // offset can reach the rate decision.
    LatentRateController latent_ctl_;
    // The automatic connect selector's coherent rung BEFORE the conservative
    // first-group cap.  The cap controls only what goes on air; it must not tell
    // the outcome estimator that a strong link was initially believed weak.
    // Consumed exactly once when the first real group seeds latent_ctl_.
    uint8_t latent_bootstrap_rung_ = kRungIdxNone;
    // One-shot startup exploration for the otherwise unidentifiable R1/2 saturation
    // region.  A clean robust rung cannot reveal how much margin exists above it, so an
    // unforced QSO may spend exactly one R2/3 group after two trusted clean R1/2 groups.
    // The receiver keeps the command standing until descriptor adoption; the first
    // non-clean or geometry-unknown result rolls straight back and spends the probe.
    // `allowed` is also the QSO-scoped operator-profile gate: a CONNECT-forced rung
    // freezes the entire latent command, not merely this special probe.
    bool latent_startup_probe_allowed_ = true;
    bool latent_startup_probe_waiting_ = false;
    bool latent_startup_probe_spent_ = false;
    int latent_startup_probe_clean_groups_ = 0;
    int latent_startup_probe_pending_base_groups_ = 0;
    bool latent_startup_probe_rollback_pending_ = false;
    bool latent_startup_probe_failed_ = false;
    // Sender half of the one-shot contract. If the first higher-rung physical group
    // reaches RTO with no ACK, timeout egress is canceled and a synchronized base-rung
    // rollback runs before any retransmission.
    bool tx_latent_startup_probe_active_ = false;
    bool tx_latent_startup_probe_airborne_ = false;
    bool tx_latent_startup_probe_timeout_rollback_pending_ = false;
    uint8_t tx_latent_startup_probe_base_rung_ = kRungIdxNone;
    // One tone-ACK seq/epoch episode may revise its cumulative mask and authority
    // command down (probe UP then timeout rollback), never back up via a delayed
    // stale copy carrying an older mask.
    bool tx_authority_ack_identity_valid_ = false;
    uint8_t tx_authority_ack_group_seq_ = 0;
    uint16_t tx_authority_ack_frame_mask_ = 0;
    uint8_t tx_authority_ack_move_epoch_ = 0;
    uint8_t tx_authority_ack_lowest_cmd_ = kRungIdxNone;
    std::chrono::steady_clock::time_point goodput_last_verdict_{};
    bool goodput_last_verdict_valid_ = false;

public:
    // F165 ANCHORED-BURST ACK BACKSTOP: the decoder accepted an expected full
    // anchor but the burst framed NO group (descriptor unreadable in a deep
    // fade). Emit exactly one re-confirm ack + crater verdict so the sender is
    // never ack-starved by an anchored burst.
    void noteAnchoredBurstNoGroup(bool payload_seen = false);
    // Decoder state-only provenance: an attempted physical burst could not yield
    // trustworthy k/M. Cancel a pending startup probe without emitting an ACK or
    // feeding a fabricated outcome to the selector.
    void noteBurstOutcomeUnknown();

private:
    void maybeObeyAuthorityCommand(uint8_t cmd_idx,
                                   bool accepted_clean_round = false);
    bool acceptAuthorityCommandRevision(uint8_t cmd_word, uint8_t group_seq,
                                        uint16_t frame_mask, uint8_t move_epoch);
    bool startupProbeHasSufficientPayload() const;
    // Sender-side economic guard for ordinary automatic UP commands.  The latent
    // selector prices a complete target-rung burst; do not execute that steady-state
    // choice when the active file tail cannot supply one such burst.  Reliability
    // demotions deliberately bypass this guard.
    bool authorityClimbHasSufficientPayload(Modulation target_mod,
                                            CodeRate target_rate,
                                            int post_ack_clean_streak) const;
    void handleLatentStartupProbeTimeoutRollback();
    std::string last_adaptive_action_;      // GUI: short human-readable action
    // QAM16 R2/3 cross-modulation climb (ULTRA_QAM16_CLIMB, default-ON since 2026-07-02). See
    // applyAdaptiveRateFeedback. clean_streak = consecutive clean groups while pinned at QPSK
    // R3/4 (the climb gate AND a low-variance Good/Moderate proxy — a Moderate channel's fades
    // keep resetting it); bad_streak = consecutive bad groups on QAM16 (the prompt-demote
    // trigger).
    int qam16_clean_streak_ = 0;
    int qam16_bad_streak_ = 0;
    // 16QAM R3/4 crest rung (ULTRA_QAM16_R34, default-OFF A/B knob). Consecutive clean groups
    // at QAM16 R2/3 toward the within-QAM16 R2/3 -> R3/4 walk — a PARALLEL counter to
    // qam16_clean_streak_ (which tracks the QPSK-pinned streak toward the modulation hop).
    // Reset on any bad group, on every QAM16 demote (noteQam16Demoted), and in enterConnected.
    int qam16_r34_clean_streak_ = 0;
    // Re-climb COOLDOWN (2026-07-02, replaces the 06-17 sticky no-reclimb). Under fade-riding
    // the QPSK<->QAM16 oscillation IS the mechanism — the rung SHOULD oscillate with the
    // ~10-20 s Good fade cycle (crest -> 16QAM R2/3, trough -> QPSK R3/4); the 06-17 sticky
    // design assumed oscillation was pathology and permanently forfeited every later crest.
    // Instead, after a demote the climb streak may not begin again until this many CLEAN
    // groups pass (base 3, ULTRA_QAM16_RECLIMB_COOLDOWN), DOUBLING per demote this connection
    // (cap x4 — ssthresh-family multiplicative backoff): a channel that keeps cratering QAM16
    // re-probes ever more rarely, bounding worst-case MODE_CHANGE move overhead <10% of
    // airtime (per-move cost arithmetic: CHANGELOG 2026-07-02). Reset in enterConnected.
    int qam16_reclimb_cooldown_ = 0;
    int qam16_demote_count_ = 0;
    void noteQam16Demoted(int weight);
    // [LADDER] per-transfer telemetry (sender-side, pure observability): time-in-rung
    // percentages + mid-stream move count, logged ONCE at transfer completion, e.g.
    //   [LADDER] qpsk_r23=54% qam16_r23=38% moves=9 (52s, ok)
    // Started in startFileTransferNow; a rung segment closes on every applyDataMode
    // mod/rate change while active; finished (logged) by the setFileSentCallback wrapper.
    struct LadderRungStat { Modulation mod; CodeRate rate; double seconds; };
    bool ladder_telemetry_active_ = false;
    int ladder_moves_ = 0;
    std::chrono::steady_clock::time_point ladder_transfer_start_{};
    std::chrono::steady_clock::time_point ladder_rung_start_{};
    Modulation ladder_cur_mod_ = Modulation::QPSK;
    CodeRate ladder_cur_rate_ = CodeRate::R1_4;
    std::vector<LadderRungStat> ladder_rung_stats_;
    void ladderTelemetryStart();
    void ladderTelemetryNoteRung(Modulation mod, CodeRate rate, bool count_move);
    void ladderTelemetryFinish(bool success);
    // Raw file payload (TYPE+OFFSET headers stripped) + a byte cursor used by the
    // chunk-at-rate form fn. Populated by startBurstFileTransfer when adaptive
    // rate is on; the cursor advances by burst_pending_advance_ only on ACK.
    std::vector<uint8_t> burst_file_payload_;
    size_t burst_file_cursor_ = 0;
    size_t burst_pending_advance_ = 0;
    uint16_t burst_chunk_seq_ = 0;
    // FILE_START / FILE_BLOCK metadata chunks held intact (different header from
    // FILE_DATA's TYPE+OFFSET — must NOT be byte-stripped or chunked at-rate).
    // Drained into the FIRST burst group's frame slots (one chunk per frame) so
    // the receiver picks them up via the same burst-deinterleave path as file
    // data — file_transfer_ enters RECEIVING before any FILE_DATA arrives.
    std::deque<Bytes> burst_metadata_queue_;
    size_t burst_pending_metadata_consumed_ = 0;
    // §SR-ARQ (2026-05-29, channel-adaptive): when the byte-interleave is OFF
    // (Good/AWGN), each burst frame is independently decodable, so the transport
    // runs Selective-Repeat at the FRAME granularity instead of whole-group
    // stop-and-wait. burst_interleave_off_ selects that path (set at transfer
    // start from ULTRA_BURST_INTERLEAVE). burst_inflight_frames_ are the exact
    // serialized frames of the in-flight burst (position-aligned with the
    // receiver's frame_mask); on a partial mask the 0-bit positions are pushed
    // (identical bytes — no re-chunk/re-offset bookkeeping) into
    // burst_resend_frames_ and drained into the next burst before new frames, so
    // every burst stays full. Pads (filler) are never re-queued.
    bool burst_interleave_off_ = false;
    std::deque<Bytes> burst_resend_frames_;
    std::vector<Bytes> burst_inflight_frames_;
    std::vector<bool> burst_inflight_is_pad_;
    // §SR-ARQ correctness: the receiver's file assembler SILENTLY DROPS FILE_DATA
    // that arrives before the FILE_START metadata establishes RECEIVING. In a
    // partial burst the metadata frame can fail while data frames in the SAME burst
    // decode — acking those (per the decoder's frame_mask) tells the sender they
    // landed when they were dropped → a permanent gap (the offset-0 loss that
    // stalled seed 44). So the RX must NOT ack a burst delivered before RECEIVING;
    // it NACKs the whole burst until metadata establishes the stream. This latch
    // distinguishes "not receiving yet" (NACK) from "already finalized" (a late dup
    // after our final ACK was lost → ack so the sender completes, not retries to death).
    bool burst_rx_ever_receiving_ = false;
    // RX group assembly for the burst transport: frames of the in-flight group
    // accumulate here (keyed by the descriptor group_seq) until the group is
    // complete, then are handed to burst_transport_.onGroupReceived().
    uint16_t burst_rx_group_seq_ = 0;
    bool burst_rx_group_open_ = false;
    std::vector<Bytes> burst_rx_group_frames_;

    // Live "incoming burst" status for the GUI (see BurstActivity above).
    BurstActivity burst_activity_;

    // ARQ ACK callbacks can acknowledge several slots from one cumulative ACK.
    // Defer window refill until ARQ finishes freeing all slots so OFDM stays
    // burst-oriented instead of collapsing into one-frame steady-state TX.
    bool arq_callback_defer_refill_ = false;
    bool deferred_file_refill_ = false;
    bool deferred_fragment_refill_ = false;
    // Set from the ARQ terminal-failure callback and consumed only after the ARQ
    // call stack unwinds. Directly clearing the window from that callback would race
    // retransmitFrame()'s own slot accounting. The deferred abort converts a fatal
    // per-frame failure into one atomic logical-transfer failure and prevents any
    // suspended tail holes from becoming immortal.
    bool deferred_arq_failure_abort_ = false;
    bool deferred_arq_failure_seq_valid_ = false;
    uint16_t deferred_arq_failure_seq_ = 0;

    // In-QSO HF ARQ DATA turn ownership. Exactly one peer is ISS (allowed to
    // originate DATA) at a time; the other queues operator payloads and requests
    // a deterministic changeover via ACK flag or TURN_REQUEST control.
    bool local_data_turn_ = false;
    bool peer_data_turn_requested_ = false;
    bool local_turn_request_pending_ = false;
    bool received_peer_data_since_connect_ = false;
    bool yielded_data_turn_waiting_for_peer_data_ = false;
    bool data_turn_yield_pending_ = false;
    uint64_t data_turn_payload_bytes_sent_ = 0;
    uint32_t data_turn_contended_ms_ = 0;
    uint32_t data_turn_tx_guard_ms_ = 0;
    uint32_t turn_request_retransmit_ms_ = 0;
    uint32_t turn_request_holdoff_ms_ = 0;
    uint32_t file_cancel_rx_drain_ms_ = 0;
    uint32_t file_cancel_reassert_ms_ = 0;
    uint32_t file_cancel_reassert_cooldown_ms_ = 0;
    bool file_cancel_confirm_pending_ = false;
    static constexpr uint32_t DATA_TURN_ACK_DIVERSITY_GUARD_FLOOR_MS = 250;
    static constexpr uint32_t DATA_TURN_CONNECT_GUARD_FLOOR_MS = 500;
    static constexpr uint32_t DATA_TURN_CONTROL_GUARD_FLOOR_MS = 500;
    static constexpr uint32_t TURN_REQUEST_HOLDOFF_FLOOR_MS = 2000;
    static constexpr uint32_t TURN_REQUEST_RETRANSMIT_FLOOR_MS = 2500;
    static constexpr uint32_t FILE_CANCEL_TX_GUARD_FLOOR_MS = 1500;
    static constexpr uint32_t FILE_CANCEL_CONFIRM_DATA_GUARD_FLOOR_MS = 1500;
    static constexpr uint32_t FILE_CANCEL_RX_DRAIN_MS = 5000;
    static constexpr uint32_t FILE_CANCEL_REASSERT_WINDOW_MS = 30000;
    static constexpr uint32_t FILE_CANCEL_REASSERT_COOLDOWN_MS = 1500;
    static constexpr uint64_t DATA_TURN_FAIR_BURST_BYTES = 4096;
    static constexpr uint64_t DATA_TURN_FAIR_MIN_BYTES_FOR_TIME_YIELD = 1024;
    static constexpr uint32_t DATA_TURN_FAIR_BURST_MS = 24000;
    uint32_t currentDataFrameAirtimeMs() const;
    uint32_t currentControlFrameAirtimeMs() const;
    uint32_t currentBurstAnchorAirtimeMs() const;
    uint32_t dataTurnAckDiversityGuardMs(const v2::ControlFrame& ack) const;
    uint32_t dataTurnConnectGuardMs() const;
    uint32_t dataTurnControlGuardMs() const;
    uint32_t turnRequestHoldoffAfterDataMs() const;
    uint32_t turnRequestRetransmitMs() const;
    uint32_t turnRequestAckEmbeddedRetransmitMs() const;
    uint32_t fileCancelTxGuardMs() const;
    uint32_t fileCancelConfirmDataGuardMs() const;
    uint32_t modeChangeRetryMs() const;
    uint32_t disconnectControlKeydownMs() const;
    uint32_t disconnectRetryIntervalMs() const;
    uint32_t disconnectResponderGraceMs() const;
    uint32_t disconnectAckRetransmitMs() const;
    int disconnectUsefulRetryCount() const;
    int disconnectAckMaxProactiveRepeats() const;
    void scheduleModeChangeAckRepeats(const Bytes& ack_data, uint16_t ack_seq);
    void tickModeChangeAckRepeats(uint32_t elapsed_ms);
    uint32_t connectControlFrameAirtimeMs() const;
    uint32_t connectRetryIntervalMs() const;
    uint32_t responderHandshakeFailSafeMs() const;

    void handleArqTimeoutBatch(const std::vector<Bytes>& frame_data_list);
    void flushStagedArqTimeoutBatch();
    void transmitFrameBatch(const std::vector<Bytes>& frame_data_list,
                            bool account_rto_round = true);
    // A group containing ACK-revealed holes normally carries the same full-anchor
    // evidence as an RTO repair.  descriptor_only_partial_repair is an exact-request,
    // default-OFF exception: its descriptor is full, while the announced DATA group
    // remains light.  RTOs never call this exception.
    void flushBurstBuffer(bool retransmission_required = false,
                          bool descriptor_only_partial_repair = false);
    void processArqFrame(const Bytes& frame_data);
    void runDeferredArqRefill();
    bool drainDeferredArqFailureAbort();
    void configureArqForCurrentDataMode();
    void configureSoftCombineHARQBounds();
    uint32_t pingTimeoutMsForCurrentProfile() const;
    bool usesBoundedVariableMCDPSKFrames() const;
    bool usesExperimental8PSKLongLDPCFor(Modulation mod, CodeRate rate,
                                         int logical_cw) const;
    bool usesExperimental8PSKLongLDPC() const;
    bool usesExperimentalQPSKR34LongLDPCFor(Modulation mod, CodeRate rate,
                                            int logical_cw) const;
    bool usesExperimentalQPSKR34LongLDPC() const;
    bool usesExperimentalLongLDPCFor(Modulation mod, CodeRate rate,
                                     int logical_cw) const;
    bool usesExperimentalLongLDPC() const;
    OFDMDataWireProfile resolveExperimentalLongLDPCWireProfileFor(
        Modulation mod, CodeRate rate, int logical_cw,
        bool psk8_long_enabled, bool qpsk_r34_long_enabled) const;
    LatentRateCandidateGeometry latentRateCandidateGeometryFor(
        Modulation mod, CodeRate rate, bool force_full_group_start) const;
    int physicalDataFrameCodewordsFor(Modulation mod, CodeRate rate,
                                      int logical_cw) const;
    int physicalDataFrameCodewords() const;
    int selectBurstLiftingZFor(Modulation mod, CodeRate rate,
                               int logical_cw) const;
    void setExperimentalLongLDPCTransferProfilesActive(bool psk8_active,
                                                       bool qpsk_r34_active);
    void appendExperimentalLongLDPCTailPad(std::vector<Bytes>& frames) const;
    size_t currentDataPayloadCapacity() const;
    // Apply a new data mode. cw_count: 0 = compute via recommendCWCount(rate),
    // 1..8 = explicit (used when MODE_CHANGE wire byte specifies a value).
    void applyDataMode(Modulation mod, CodeRate rate, int cw_count = 0,
                       LadderRungId rung_id = LadderRungId::UNKNOWN);
    void commitPendingModeChange(const char* outcome);
    void notifyDataModeChanged(float snr_db, float peer_fading_index, bool snr_is_wire);
    LadderRungId currentLadderRungId() const;
    // DESC-SWITCH sender side (Phase 1 §5.1): scope gate + commit for a CLEAN-BOUNDARY
    // wideband-OFDM ladder move. tryDescriptorModeSwitch returns true iff it committed
    // (the caller then SKIPS requestModeChange); false = fall back to the legacy
    // MODE_CHANGE exchange (knob OFF / out of scope — see the .cpp block comment).
    bool tryDescriptorModeSwitch(Modulation mod, CodeRate rate, float measured_snr,
                                 uint8_t reason);
    void commitLocalModeSwitch(Modulation mod, CodeRate rate, int cw_count,
                               float measured_snr, uint8_t reason);

    // Callbacks
    TransmitCallback on_transmit_;
    TransmitInfoCallback on_transmit_info_;
    TransmitToneBurstAckCallback on_transmit_tone_burst_ack_;
    bool tone_ack_group_complete_context_ = false;
    // Software-ALC provenance exists only while onBurstGroupReceived is processing
    // the current decoder callback. `has_decoded_data` is derived from that
    // callback's CRC-valid, locally-addressed frame vector -- never from the ARQ's
    // cumulative SACK bitmap. Both are cleared before any timer/classic/backstop
    // ACK can be emitted, so an old LOW/CLIPPED verdict cannot steer a new ACK.
    bool tone_ack_alc_group_context_ = false;
    bool tone_ack_group_has_decoded_data_context_ = false;
    // Stronger than physical-turn-complete: true only while processing a
    // descriptor-proven group with exact k/M geometry. Classic tails and timer
    // backstops deliberately leave this false.
    bool tone_ack_exact_group_geometry_context_ = false;
    std::function<bool()> tx_active_provider_;  // BUG-MC-RETRY-SPURIOUS: see setter
    std::function<bool()> channel_busy_query_;  // keepalive universal busy gate
    DriveAdvisoryCallback on_drive_advisory_;  // software-ALC sender-side hook
    ArmToneBurstAckMonitorCallback on_arm_tone_burst_ack_monitor_;
    ConnectedCallback on_connected_;
    DisconnectedCallback on_disconnected_;
    DisconnectTeardownCallback on_disconnect_teardown_;
    MessageReceivedCallback on_message_received_;
    MessageSentCallback on_message_sent_;
    MessageTxStatusCallback on_message_tx_status_;
    IncomingCallCallback on_incoming_call_;
    DataReceivedCallback on_data_received_;
    FileSentCallback on_file_sent_;
    ModeNegotiatedCallback on_mode_negotiated_;
    DataModeChangedCallback on_data_mode_changed_;
    ConnectWaveformChangedCallback on_connect_waveform_changed_;
    PhyMaskV1NegotiatedCallback on_phy_mask_v1_negotiated_;
    HandshakeConfirmedCallback on_handshake_confirmed_;
    FullOFDMAnchorExpectedCallback on_full_ofdm_anchor_expected_;
    DataTurnAcquiredCallback on_data_turn_acquired_;
    PingTxCallback on_ping_tx_;
    PingReceivedCallback on_ping_received_;
    StateChangedCallback on_state_changed_;

    // Probing state (PING/PONG fast presence check)
    int ping_retry_count_ = 0;
    static constexpr int MAX_PING_RETRIES = 5;  // Try 5 pings before giving up
    static constexpr uint32_t PING_TIMEOUT_MS = 8000;  // 8 seconds per ping (PING=3.3s + PONG=3.3s + margin)
    static constexpr uint32_t ROBUST_LOW_PING_TIMEOUT_MS = 20000;

    // Handshake state - responder waits for first frame before confirming
    bool is_initiator_ = false;           // True if we initiated the connection
    bool handshake_confirmed_ = false;    // True after handshake is fully confirmed
    uint32_t responder_handshake_wait_ms_ = 0;  // Fail-safe timer for responder handshake
    static constexpr uint32_t RESPONDER_HANDSHAKE_FAILSAFE_MS = 2200;

    // Collision-safe CONNECT_ACK recovery (responder side).  Never retransmit
    // this cache on a timer: live IONOS proved the full 8.3 s ACK overlaps the
    // initiator's own CONNECT retry and deadlocks both half-duplex stations.
    // A fully decoded duplicate CONNECT is the sole replay trigger.
    Bytes connect_ack_frame_;

    // Internal handlers for v2 frames
    void handleConnect(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleConnectAck(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleConnectNak(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleDisconnect(const v2::ControlFrame& frame, const std::string& src_call);
    void handleDisconnectFrame(const v2::ConnectFrame& frame, const std::string& src_call);
    // Service the responder ACK/grace window in both CONNECTED and crossed-close
    // DISCONNECTING states. Returns true when the grace completes the session.
    bool tickDisconnectResponderGrace(uint32_t elapsed_ms);
    void setDisconnectTeardownActive(bool active);
    static bool isDisconnectTeardownWireFrame(const Bytes& frame_data);
    bool isAllowedDisconnectTeardownRx(const v2::HeaderInfo& header) const;
    void handleModeChange(const v2::ControlFrame& frame, const std::string& src_call);
    bool sendPayload(const Bytes& data, bool binary_payload);
    bool startPayloadNow(const Bytes& data, bool binary_payload, uint64_t message_token = 0);
    uint64_t createOutboundMessageRecord(const Bytes& data);
    void setOutboundMessageExpectedFragments(uint64_t token, size_t fragments);
    void dropOutboundMessageRecord(uint64_t token);
    void failOutboundMessageRecord(uint64_t token);
    std::vector<OutboundMessageTxRecord> detachOutboundMessageRecords();
    void emitFailedMessageRecords(
        const std::vector<OutboundMessageTxRecord>& records);
    void clearOutboundMessageTracking();
    uint64_t allocateQueuedOperationOrder();
    bool sendArqPayloadFrame(const Bytes& chunk, v2::FrameType frame_type, uint8_t flags,
                             bool fixed_frame, uint64_t message_token);
    void handleArqFrameSubmitted(uint16_t seq);
    void handleArqTxBaseAdvanced(uint16_t base_seq);
    void handleArqFrameFailed(uint16_t seq);
    void emitMessageTxStatus(const OutboundMessageTxRecord& record,
                             MessageTxStatus status);
    void drainMessageTxStatusEvents();
    bool shouldQueuePayloadForLinkTurn() const;
    bool hasGeometryBoundDataOperation() const;
    bool hasLocalOutboundDataTurn() const;
    bool hasLocalInFlightDataTurn() const;
    bool hasLocalDataWaitingForTurn() const;
    bool dataTurnFairBudgetMet() const;
    bool shouldPauseLocalDataForPeerRequest() const;
    bool shouldRequestDataTurnOnAck() const;
    bool noteTurnRequestOnAckIfNeeded();
    void resetDataTurnFairness();
    void noteDataTurnPayloadStarted(size_t payload_bytes);
    void sendTurnRequestIfNeeded();
    bool maybeYieldDataTurn();
    void armDataTurnTxGuard(uint32_t guard_ms);
    void transmitFileCancelControl(const char* reason);
    void armFileCancelReassertion();
    void clearFileCancelReassertion();
    void maybeReassertFileCancelForStaleData();
    bool startFileTransferNow(const std::string& filepath);
    bool tryStartQueuedFileIfReady();
    void sendNextQueuedPayloadIfReady();
    void clearFileTransferArqState();
    void handleDataPayload(const Bytes& payload, bool more_data,
                           v2::FrameType frame_type, uint8_t flags);
    void handleTurnover(const v2::ControlFrame& frame, const std::string& src_call);
    void handleTurnRequest(const v2::ControlFrame& frame, const std::string& src_call);
    void handleFileCancel(const v2::ControlFrame& frame, const std::string& src_call);

    void transmitFrame(const Bytes& frame_data);
    void enterConnected(bool automatic_rate_allowed = true);
    void enterDisconnected(const std::string& reason);
    void sendFullConnect();  // Send full CONNECT frame after successful PING/PONG
    void cancelOutboundProbe();
    void cancelOutboundConnect();

    WaveformMode negotiateMode(uint8_t remote_caps, WaveformMode remote_pref);
    void sendNextFileChunk();
    size_t sendNextFragment();
};

} // namespace protocol
} // namespace ultra
