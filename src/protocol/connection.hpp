#pragma once

#include "frame_v2.hpp"
#include "arq.hpp"
#include "selective_repeat_arq.hpp"
#include "file_transfer.hpp"
#include "ultra/types.hpp"
#include "fec/soft_combine.hpp"
#include <functional>
#include <string>

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

// Connection configuration
struct ConnectionConfig {
    ARQConfig arq;
    uint32_t connect_timeout_ms = 60000;  // 60s for DPSK (16s TX + 16s RX + margin)
    uint32_t disconnect_timeout_ms = 30000;
    uint32_t pong_tx_delay_ms = 500;      // Let peer RX settle after its PING PTT-off
    uint32_t post_connect_data_delay_ms = 500;  // Hold first DATA after CONNECTED; peer's PTT-off settling on CONNECT_ACK TX needs this window
    uint32_t ack_tx_delay_ms = 500;       // Hold ACK after inbound DATA; peer's PTT-off settling on DATA TX needs this window
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
    using ConnectedCallback = std::function<void()>;
    using DisconnectedCallback = std::function<void(const std::string& reason)>;
    using MessageReceivedCallback = std::function<void(const std::string& text)>;
    using MessageSentCallback = std::function<void(bool success)>;
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

    // --- Frame Processing ---

    // Process received frame data (v2 serialized bytes)
    void onFrameReceived(const Bytes& frame_data);
    void onMCDPSKPartialFrame(const v2::PartialFrameCodewords& partial);
    void onAcceptedOFDMDataSync(float sync_correlation);

    void tick(uint32_t elapsed_ms);

    // --- Callbacks ---

    void setTransmitCallback(TransmitCallback cb);

    // Burst mode TX callback - transmits multiple frames as single audio burst.
    // Used for OFDM connected mode and MC-DPSK DATA file-window bursts.
    using TransmitBurstCallback = std::function<void(const std::vector<Bytes>&)>;
    void setTransmitBurstCallback(TransmitBurstCallback cb);

    void setConnectedCallback(ConnectedCallback cb);
    void setDisconnectedCallback(DisconnectedCallback cb);
    void setMessageReceivedCallback(MessageReceivedCallback cb);
    void setMessageSentCallback(MessageSentCallback cb);
    void setIncomingCallCallback(IncomingCallCallback cb);
    void setDataReceivedCallback(DataReceivedCallback cb);

    // Ping/Pong (fast presence check)
    void setPingTxCallback(PingTxCallback cb) { on_ping_tx_ = cb; }
    void setPingReceivedCallback(PingReceivedCallback cb) { on_ping_received_ = cb; }
    void setStateChangedCallback(StateChangedCallback cb) { on_state_changed_ = cb; }
    void onPongReceived();  // Call when modem detects response to our PING

    void setFileProgressCallback(FileProgressCallback cb);
    void setFileReceivedCallback(FileReceivedCallback cb);
    void setFileSentCallback(FileSentCallback cb);

    // --- State ---

    ConnectionState getState() const { return state_; }
    std::string getRemoteCallsign() const { return remote_call_; }
    bool isConnected() const { return state_ == ConnectionState::CONNECTED; }
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

    void setSoftCombiningHARQ(bool enable);
    bool getSoftCombiningHARQ() const { return soft_combine_harq_.enabled(); }
    fec::SoftCombineBuffer* softCombineBuffer() { return &soft_combine_harq_; }

    using ModeNegotiatedCallback = std::function<void(WaveformMode mode)>;
    void setModeNegotiatedCallback(ModeNegotiatedCallback cb) { on_mode_negotiated_ = cb; }

    // Callback when handshake is confirmed (safe to switch to negotiated waveform)
    // For initiator: called immediately after CONNECT_ACK received
    // For responder: called when first frame received after sending CONNECT_ACK
    using HandshakeConfirmedCallback = std::function<void()>;
    void setHandshakeConfirmedCallback(HandshakeConfirmedCallback cb) { on_handshake_confirmed_ = cb; }

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

    // Set measured SNR from modem layer (call this when decoding frames)
    void setMeasuredSNR(float snr_db) { measured_snr_db_ = snr_db; }
    float getMeasuredSNR() const { return measured_snr_db_; }

    // Set channel quality including fading detection
    // fading_index: combined freq_cv + temporal_cv, where > 0.65 indicates significant fading
    void setChannelQuality(float snr_db, float fading_index) {
        measured_snr_db_ = snr_db;
        fading_index_ = fading_index;
    }
    float getFadingIndex() const { return fading_index_; }
    bool isFading() const { return fading_index_ > 0.65f; }

    // Callback when remote station requests mode change
    // Data-mode-changed callback. cw_count is the negotiated fixed-frame CW
    // count for the new rate (1..8) — host updates encoder/decoder from this
    // value directly. Host MUST NOT call back into ProtocolEngine from this
    // callback (mutex held; re-entry will deadlock).
    using DataModeChangedCallback = std::function<void(Modulation mod, CodeRate rate,
                                                        int cw_count,
                                                        float snr_db, float peer_fading_index,
                                                        int mc_dpsk_num_carriers,
                                                        int mc_dpsk_samples_per_symbol)>;
    void setDataModeChangedCallback(DataModeChangedCallback cb) { on_data_mode_changed_ = cb; }

    // Request mode change to remote station
    void requestModeChange(Modulation new_mod, CodeRate new_rate, float measured_snr, uint8_t reason);

    void reset();

private:
    void setPhyMaskV1Negotiated(bool enabled);

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

    // Waveform mode
    WaveformMode narrowband_override_ = WaveformMode::AUTO;  // Session-scoped, cleared on disconnect/reset
    WaveformMode negotiated_mode_ = WaveformMode::OFDM_COX;
    uint8_t remote_capabilities_ = ModeCapabilities::OFDM_COX;
    WaveformMode remote_preferred_ = WaveformMode::OFDM_COX;
    bool phy_mask_v1_negotiated_ = false;

    // Data modulation and code rate (adaptive)
    Modulation data_modulation_ = Modulation::DQPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;
    int data_frame_cw_count_ = v2::kDefaultFixedFrameCodewords;
    LadderRungId data_ladder_rung_id_ = LadderRungId::UNKNOWN;
    uint16_t mode_change_seq_ = 0;  // Sequence number for MODE_CHANGE frames
    float measured_snr_db_ = 15.0f;  // SNR measured by modem (updated via setMeasuredSNR)
    float fading_index_ = 0.0f;      // Fading index (0-2, > 0.65 = significant fading)

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
    static constexpr uint32_t MODE_CHANGE_TIMEOUT_MS = 45000;  // 45s for DPSK round trip
    static constexpr int MODE_CHANGE_MAX_RETRIES = 2;

    // ARQ for reliable data transfer (Selective Repeat for higher throughput)
    SelectiveRepeatARQ arq_;
    fec::SoftCombineBuffer soft_combine_harq_;

    // File transfer controller
    FileTransferController file_transfer_;

    // Message fragmentation (TX) - splits long messages across multiple ARQ frames
    std::vector<Bytes> pending_tx_fragments_;
    std::vector<uint8_t> pending_tx_fragment_flags_;  // Per-fragment flags (for sendMessages batch)
    std::vector<v2::FrameType> pending_tx_fragment_types_;  // DATA_START/CONT/END for binary streams
    size_t next_fragment_idx_ = 0;
    size_t acked_fragment_count_ = 0;  // Actual ACKs received (vs next_fragment_idx_ = submitted)

    // Message reassembly (RX) - accumulates fragments into complete messages
    Bytes rx_reassembly_buffer_;

    // Connection timing
    uint32_t timeout_remaining_ms_ = 0;
    int connect_retry_count_ = 0;
    uint32_t connected_time_ms_ = 0;
    bool pending_pong_callback_ = false;
    uint32_t pong_callback_delay_remaining_ms_ = 0;

    // After entering CONNECTED, block outbound DATA TX for this window so
    // peer's PTT-off transition (after CONNECT_ACK / DATA TX) completes
    // before our reply hits their antenna. Symmetric to pong_tx_delay_ms.
    // DATA-class frames are held in held_data_frames_ and released on
    // hold expiry; ACK/control frames pass through unaffected.
    bool post_connect_data_hold_active_ = false;
    uint32_t post_connect_data_hold_remaining_ms_ = 0;
    std::vector<Bytes> held_data_frames_;

    // After receiving DATA, block outbound ACK TX for this window so the
    // sender's PTT-off transition after DATA TX completes before our ACK
    // arrives. ACK frames are held and released when the timer expires.
    uint32_t ack_hold_remaining_ms_ = 0;
    std::vector<Bytes> held_ack_frames_;

    // Disconnect retransmission (initiator side)
    Bytes disconnect_frame_;                // Cached DISCONNECT frame for retransmission
    int disconnect_retry_count_ = 0;
    uint32_t disconnect_retransmit_ms_ = 0; // Time until next retransmit
    static constexpr uint32_t DISCONNECT_RETRANSMIT_INTERVAL_MS = 5000;
    static constexpr int DISCONNECT_MAX_RETRIES = 3;

    // Disconnect grace period (responder side)
    // After receiving DISCONNECT, stay connected briefly and re-send ACK
    // periodically to ensure the initiator gets it (fading can lose frames)
    bool disconnect_pending_ = false;
    uint32_t disconnect_pending_ms_ = 0;
    uint32_t disconnect_ack_retransmit_ms_ = 0; // Time until next ACK re-send
    Bytes disconnect_ack_frame_;            // Cached ACK for re-sending
    static constexpr uint32_t DISCONNECT_GRACE_MS = 5000;            // 5s total grace period
    static constexpr uint32_t DISCONNECT_ACK_RETRANSMIT_MS = 2000;   // Re-send ACK every 2s

    // Calling waveform for PING/CONNECT control frames.
    WaveformMode connect_waveform_ = WaveformMode::MC_DPSK;

    // Statistics
    ConnectionStats stats_;

    // Burst mode TX buffering (OFDM and MC-DPSK DATA)
    std::vector<Bytes> burst_tx_buffer_;
    bool burst_mode_active_ = false;
    TransmitBurstCallback on_transmit_burst_;

    // ARQ ACK callbacks can acknowledge several slots from one cumulative ACK.
    // Defer window refill until ARQ finishes freeing all slots so OFDM stays
    // burst-oriented instead of collapsing into one-frame steady-state TX.
    bool arq_callback_defer_refill_ = false;
    bool deferred_file_refill_ = false;
    bool deferred_fragment_refill_ = false;

    void flushBurstBuffer();
    void processArqFrame(const Bytes& frame_data);
    void runDeferredArqRefill();
    void configureArqForCurrentDataMode();
    void cancelPendingPongCallback();
    uint32_t pingTimeoutMsForCurrentProfile() const;
    bool usesBoundedVariableMCDPSKFrames() const;
    size_t currentDataPayloadCapacity() const;
    // Apply a new data mode. cw_count: 0 = compute via recommendCWCount(rate),
    // 1..8 = explicit (used when MODE_CHANGE wire byte specifies a value).
    void applyDataMode(Modulation mod, CodeRate rate, int cw_count = 0,
                       LadderRungId rung_id = LadderRungId::UNKNOWN);
    void notifyDataModeChanged(float snr_db, float peer_fading_index);
    LadderRungId currentLadderRungId() const;
    void resetAdaptiveModeController();
    void updateAdaptiveModeController(uint32_t elapsed_ms);
    bool tryIssueAdaptiveModeChangeAtBoundary();
    bool canIssueAdaptiveModeChange(bool is_downgrade) const;
    bool hasAdaptiveUpgradeBacklog(CodeRate target_rate) const;
    size_t adaptiveBacklogFrames(CodeRate rate) const;

    struct AdaptiveModeTarget {
        bool pending = false;
        Modulation modulation = Modulation::DQPSK;
        CodeRate rate = CodeRate::R1_4;
        uint8_t reason = v2::ModeChangeReason::CHANNEL_IMPROVED;
    };
    AdaptiveModeTarget adaptive_target_;
    ARQStats adaptive_last_stats_;
    uint32_t adaptive_eval_elapsed_ms_ = 0;
    uint32_t adaptive_cooldown_ms_ = 0;
    uint32_t adaptive_post_downgrade_lockout_ms_ = 0;
    uint32_t adaptive_downgrade_queue_age_ms_ = 0;
    int adaptive_clean_windows_ = 0;
    int adaptive_pressure_windows_ = 0;
    static constexpr uint32_t ADAPTIVE_EVAL_INTERVAL_MS = 1000;
    static constexpr uint32_t ADAPTIVE_MODE_CHANGE_COOLDOWN_MS = 3000;
    static constexpr uint32_t ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS = 5000;
    static constexpr uint32_t ADAPTIVE_DOWNGRADE_FORCE_MS = 6000;
    static constexpr int ADAPTIVE_CLEAN_WINDOWS_FOR_UPGRADE = 3;
    static constexpr int ADAPTIVE_PRESSURE_WINDOWS_FOR_DOWNGRADE = 2;

    // Callbacks
    TransmitCallback on_transmit_;
    ConnectedCallback on_connected_;
    DisconnectedCallback on_disconnected_;
    MessageReceivedCallback on_message_received_;
    MessageSentCallback on_message_sent_;
    IncomingCallCallback on_incoming_call_;
    DataReceivedCallback on_data_received_;
    ModeNegotiatedCallback on_mode_negotiated_;
    DataModeChangedCallback on_data_mode_changed_;
    ConnectWaveformChangedCallback on_connect_waveform_changed_;
    PhyMaskV1NegotiatedCallback on_phy_mask_v1_negotiated_;
    HandshakeConfirmedCallback on_handshake_confirmed_;
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

    // CONNECT_ACK retransmission (responder side, BUG-CTRL-001)
    // ALPHA can fail to decode the single MC-DPSK CONNECT_ACK on faded seeds.
    // We proactively re-send it from BRAVO until the handshake is confirmed by
    // a first frame from the initiator, mirroring the disconnect-ACK pattern.
    Bytes connect_ack_frame_;                  // Cached CONNECT_ACK for re-sending
    uint32_t connect_ack_retransmit_ms_ = 0;   // Time until next retransmit
    uint32_t connect_ack_retransmit_interval_ms_ = 6000;
    int connect_ack_retx_remaining_ = 0;       // Retries left (counts down to 0)
    // First retx fires AFTER the OFDM round-trip window so the success path
    // (ALPHA decoded the original ACK and started sending) clears retx state
    // before any retx is sent — keeping the retx pure overhead-on-failure only.
    // Capped at 1 retx because additional retx fire uselessly when BRAVO's PHY
    // is stuck decoding (e.g. burst of false syncs at the LTS): the original
    // ACK either reached ALPHA or didn't, and the 1st retx is the only second
    // chance worth paying for. The runtime interval is lengthened for OFDM so
    // the retry does not transmit into the first burst-interleaver group.
    static constexpr uint32_t CONNECT_ACK_RETRANSMIT_MS = 6000;
    static constexpr int CONNECT_ACK_MAX_RETX = 1;

    // Internal handlers for v2 frames
    void handleConnect(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleConnectAck(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleConnectNak(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleDisconnect(const v2::ControlFrame& frame, const std::string& src_call);
    void handleDisconnectFrame(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleModeChange(const v2::ControlFrame& frame, const std::string& src_call);
    bool sendPayload(const Bytes& data, bool binary_payload);
    void handleDataPayload(const Bytes& payload, bool more_data, v2::FrameType frame_type);

    void transmitFrame(const Bytes& frame_data);
    void enterConnected();
    void enterDisconnected(const std::string& reason);
    void sendFullConnect();  // Send full CONNECT frame after successful PING/PONG
    void cancelOutboundProbe();
    void cancelOutboundConnect();

    WaveformMode negotiateMode(uint8_t remote_caps, WaveformMode remote_pref);
    void sendNextFileChunk();
    void sendNextFragment();
};

} // namespace protocol
} // namespace ultra
