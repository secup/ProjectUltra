#pragma once

#include "frame_v2.hpp"
#include "arq.hpp"
#include "file_transfer.hpp"
#include "ultra/types.hpp"
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
    int connect_retries = 10;  // 5 DPSK attempts + 5 MFSK attempts (DPSK_ATTEMPTS = 5)
    bool auto_accept = true;

    uint8_t mode_capabilities = ModeCapabilities::ALL;
    WaveformMode preferred_mode = WaveformMode::AUTO;  // Forced waveform (0xFF=AUTO)

    // Forced data mode - operator can override SNR-based selection
    // 0xFF (AUTO) = let responder decide based on SNR
    // Any other value = force that specific mode
    Modulation forced_modulation = Modulation::AUTO;
    CodeRate forced_code_rate = CodeRate::AUTO;
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

    // --- Data Transfer ---

    bool sendMessage(const std::string& text);
    bool isReadyToSend() const;

    // --- File Transfer ---

    bool sendFile(const std::string& filepath);
    void setReceiveDirectory(const std::string& dir);
    void cancelFileTransfer();
    bool isFileTransferInProgress() const;
    FileTransferProgress getFileProgress() const;

    // --- Frame Processing ---

    // Process received frame data (v2 serialized bytes)
    void onFrameReceived(const Bytes& frame_data);

    void tick(uint32_t elapsed_ms);

    // --- Callbacks ---

    void setTransmitCallback(TransmitCallback cb);
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
    void setPreferredMode(WaveformMode mode) { config_.preferred_mode = mode; }
    void setModeCapabilities(uint8_t caps) { config_.mode_capabilities = caps; }

    // Forced data mode - operator can override SNR-based selection
    void setForcedModulation(Modulation mod) { config_.forced_modulation = mod; }
    void setForcedCodeRate(CodeRate rate) { config_.forced_code_rate = rate; }
    Modulation getForcedModulation() const { return config_.forced_modulation; }
    CodeRate getForcedCodeRate() const { return config_.forced_code_rate; }

    using ModeNegotiatedCallback = std::function<void(WaveformMode mode)>;
    void setModeNegotiatedCallback(ModeNegotiatedCallback cb) { on_mode_negotiated_ = cb; }

    // Callback when handshake is confirmed (safe to switch to negotiated waveform)
    // For initiator: called immediately after CONNECT_ACK received
    // For responder: called when first frame received after sending CONNECT_ACK
    using HandshakeConfirmedCallback = std::function<void()>;
    void setHandshakeConfirmedCallback(HandshakeConfirmedCallback cb) { on_handshake_confirmed_ = cb; }

    // Callback when connection attempt waveform changes (DPSK -> MFSK fallback)
    using ConnectWaveformChangedCallback = std::function<void(WaveformMode mode)>;
    void setConnectWaveformChangedCallback(ConnectWaveformChangedCallback cb) { on_connect_waveform_changed_ = cb; }

    // Get current waveform being used for connection attempts
    WaveformMode getConnectWaveform() const { return connect_waveform_; }

    // Set initial waveform for next connection (for testing MFSK directly)
    void setInitialConnectWaveform(WaveformMode mode) { connect_waveform_ = mode; }

    // --- Data Mode (modulation + code rate) ---

    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    // Set measured SNR from modem layer (call this when decoding frames)
    void setMeasuredSNR(float snr_db) { measured_snr_db_ = snr_db; }
    float getMeasuredSNR() const { return measured_snr_db_; }

    // Callback when remote station requests mode change
    using DataModeChangedCallback = std::function<void(Modulation mod, CodeRate rate, float snr_db)>;
    void setDataModeChangedCallback(DataModeChangedCallback cb) { on_data_mode_changed_ = cb; }

    // Request mode change to remote station
    void requestModeChange(Modulation new_mod, CodeRate new_rate, float measured_snr, uint8_t reason);

    void reset();

private:
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

    // Waveform mode
    WaveformMode negotiated_mode_ = WaveformMode::OFDM_NVIS;
    uint8_t remote_capabilities_ = ModeCapabilities::OFDM_NVIS;
    WaveformMode remote_preferred_ = WaveformMode::OFDM_NVIS;

    // Data modulation and code rate (adaptive)
    Modulation data_modulation_ = Modulation::DQPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;
    uint16_t mode_change_seq_ = 0;  // Sequence number for MODE_CHANGE frames
    float measured_snr_db_ = 15.0f;  // SNR measured by modem (updated via setMeasuredSNR)

    // MODE_CHANGE timeout/retry tracking
    bool mode_change_pending_ = false;
    uint32_t mode_change_timeout_ms_ = 0;
    int mode_change_retry_count_ = 0;
    Modulation pending_modulation_ = Modulation::DQPSK;
    CodeRate pending_code_rate_ = CodeRate::R1_4;
    float pending_snr_db_ = 15.0f;
    uint8_t pending_reason_ = 0;
    static constexpr uint32_t MODE_CHANGE_TIMEOUT_MS = 45000;  // 45s for DPSK round trip
    static constexpr int MODE_CHANGE_MAX_RETRIES = 2;

    // ARQ for reliable data transfer
    StopAndWaitARQ arq_;

    // File transfer controller
    FileTransferController file_transfer_;

    // Connection timing
    uint32_t timeout_remaining_ms_ = 0;
    int connect_retry_count_ = 0;
    uint32_t connected_time_ms_ = 0;

    // Adaptive calling waveform (DPSK first, fallback to MFSK)
    // Start with DPSK medium (DQPSK 62b R1/4), switch to MFSK after 5 attempts
    WaveformMode connect_waveform_ = WaveformMode::DPSK;
    static constexpr int DPSK_ATTEMPTS = 5;  // Attempts 1-5 use DPSK
    // Attempts 6-10 use MFSK (config_.connect_retries = 10)

    // Statistics
    ConnectionStats stats_;

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
    HandshakeConfirmedCallback on_handshake_confirmed_;
    PingTxCallback on_ping_tx_;
    PingReceivedCallback on_ping_received_;
    StateChangedCallback on_state_changed_;

    // Probing state (PING/PONG fast presence check)
    int ping_retry_count_ = 0;
    static constexpr int MAX_PING_RETRIES = 5;  // Try 5 pings before giving up
    static constexpr uint32_t PING_TIMEOUT_MS = 3000;  // 3 seconds per ping attempt

    // Handshake state - responder waits for first frame before confirming
    bool is_initiator_ = false;           // True if we initiated the connection
    bool handshake_confirmed_ = false;    // True after handshake is fully confirmed

    // Internal handlers for v2 frames
    void handleConnect(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleConnectAck(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleConnectNak(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleDisconnect(const v2::ControlFrame& frame, const std::string& src_call);
    void handleDisconnectFrame(const v2::ConnectFrame& frame, const std::string& src_call);
    void handleModeChange(const v2::ControlFrame& frame, const std::string& src_call);
    void handleDataPayload(const Bytes& payload, bool more_data);

    void transmitFrame(const Bytes& frame_data);
    void enterConnected();
    void enterDisconnected(const std::string& reason);
    void sendFullConnect();  // Send full CONNECT frame after successful PING/PONG

    WaveformMode negotiateMode(uint8_t remote_caps, WaveformMode remote_pref);
    void sendNextFileChunk();
};

} // namespace protocol
} // namespace ultra
