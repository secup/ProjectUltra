#pragma once

#include "frame_v2.hpp"
#include "arq.hpp"
#include "file_transfer.hpp"
#include "ultra/types.hpp"  // For ChannelQuality
#include <functional>
#include <string>

namespace ultra {
namespace protocol {

// Connection states
enum class ConnectionState {
    DISCONNECTED,   // Idle, listening for incoming calls
    PROBING,        // Sent PROBE, waiting for channel report
    CONNECTING,     // Sent CONNECT, waiting for ACK
    CONNECTED,      // Link established, can exchange data
    DISCONNECTING   // Sent DISCONNECT, waiting for ACK
};

const char* connectionStateToString(ConnectionState state);

// Connection configuration
struct ConnectionConfig {
    ARQConfig arq;
    uint32_t connect_timeout_ms = 10000;    // Time to wait for CONNECT_ACK
    uint32_t disconnect_timeout_ms = 5000;  // Time to wait for DISCONNECT ack
    uint32_t probe_timeout_ms = 15000;      // Time to wait for PROBE_ACK (longer - needs channel measurement)
    int connect_retries = 3;                // CONNECT retry attempts
    int probe_retries = 2;                  // PROBE retry attempts
    bool auto_accept = true;                // Automatically accept incoming calls

    // Waveform mode settings
    uint8_t mode_capabilities = ModeCapabilities::ALL;  // Which modes we support
    WaveformMode preferred_mode = WaveformMode::AUTO;   // Preferred mode (AUTO = let remote decide)
};

// Connection statistics
struct ConnectionStats {
    ARQStats arq;                   // ARQ layer stats
    int connects_initiated = 0;
    int connects_received = 0;
    int connects_failed = 0;
    int disconnects = 0;
    uint32_t connected_time_ms = 0; // Time in CONNECTED state
};

/**
 * Connection Manager
 *
 * Handles connection establishment and teardown with callsign addressing.
 * Wraps ARQ controller for reliable data transfer once connected.
 *
 * Typical flow (initiator):
 *   1. Call connect(remote_callsign)
 *   2. Wait for on_connected callback
 *   3. Use sendMessage() to send data
 *   4. Receive data via on_message_received callback
 *   5. Call disconnect() when done
 *
 * Typical flow (responder):
 *   1. Set on_incoming_call callback
 *   2. Receive call, auto-accept or manual accept
 *   3. Exchange data
 *   4. Receive disconnect or call disconnect()
 */
class Connection {
public:
    // Callback types
    using TransmitCallback = std::function<void(const Frame&)>;
    using RawTransmitCallback = std::function<void(const Bytes&)>;  // For v2 frames
    using ConnectedCallback = std::function<void()>;
    using DisconnectedCallback = std::function<void(const std::string& reason)>;
    using MessageReceivedCallback = std::function<void(const std::string& text)>;
    using MessageSentCallback = std::function<void(bool success)>;
    using IncomingCallCallback = std::function<void(const std::string& remote_call)>;
    using DataReceivedCallback = std::function<void(const Bytes& data, bool more_data)>;

    // File transfer callbacks
    using FileProgressCallback = FileTransferController::ProgressCallback;
    using FileReceivedCallback = FileTransferController::ReceivedCallback;
    using FileSentCallback = FileTransferController::SentCallback;

    // Channel probe callback (receives channel report from PROBE_ACK)
    using ProbeCompleteCallback = std::function<void(const ChannelReport& report)>;

    // Channel measurement provider callback (returns current channel quality from modem)
    // Called when we need to measure channel for PROBE_ACK response
    using ChannelMeasurementCallback = std::function<ChannelQuality()>;

    explicit Connection(const ConnectionConfig& config = ConnectionConfig{});

    // --- Configuration ---

    // Set local callsign (required)
    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const { return local_call_; }

    // Set auto-accept mode
    void setAutoAccept(bool auto_accept) { config_.auto_accept = auto_accept; }
    bool getAutoAccept() const { return config_.auto_accept; }

    // --- Connection Control ---

    // Initiate connection to remote station
    // Returns false if already connected/connecting
    bool connect(const std::string& remote_call);

    // Accept incoming call (if auto_accept is false)
    void acceptCall();

    // Reject incoming call
    void rejectCall();

    // --- Channel Probing (Link Establishment) ---

    // Probe channel to remote station before connecting
    // Sends PROBE frame, waits for PROBE_ACK with channel report
    // Returns false if already busy
    bool probe(const std::string& remote_call);

    // Check if probing is in progress
    bool isProbing() const { return state_ == ConnectionState::PROBING; }

    // Get last received channel report (valid after probe completes)
    const ChannelReport& getLastChannelReport() const { return last_channel_report_; }

    // Connect using the results from a previous probe
    // Uses the recommended mode from the channel report
    bool connectAfterProbe();

    // Disconnect from remote station
    void disconnect();

    // --- Data Transfer (only valid when CONNECTED) ---

    // Send a text message
    // Returns false if not connected or busy
    bool sendMessage(const std::string& text);

    // Check if ready to send
    bool isReadyToSend() const;

    // --- File Transfer (only valid when CONNECTED) ---

    // Send a file. Returns false if not connected, busy, or file not found.
    bool sendFile(const std::string& filepath);

    // Set directory for received files
    void setReceiveDirectory(const std::string& dir);

    // Cancel ongoing file transfer
    void cancelFileTransfer();

    // Check if file transfer is in progress
    bool isFileTransferInProgress() const;

    // Get file transfer progress
    FileTransferProgress getFileProgress() const;

    // --- Frame Processing ---

    // Process a received frame (call from modem RX callback)
    void onFrameReceived(const Frame& frame);

    // Periodic update (call from main loop)
    // Pass elapsed milliseconds since last call
    void tick(uint32_t elapsed_ms);

    // --- Callbacks ---

    void setTransmitCallback(TransmitCallback cb);
    void setRawTransmitCallback(RawTransmitCallback cb);  // For v2 frames (raw bytes)
    void setConnectedCallback(ConnectedCallback cb);
    void setDisconnectedCallback(DisconnectedCallback cb);
    void setMessageReceivedCallback(MessageReceivedCallback cb);
    void setMessageSentCallback(MessageSentCallback cb);
    void setIncomingCallCallback(IncomingCallCallback cb);
    void setDataReceivedCallback(DataReceivedCallback cb);

    // File transfer callbacks
    void setFileProgressCallback(FileProgressCallback cb);
    void setFileReceivedCallback(FileReceivedCallback cb);
    void setFileSentCallback(FileSentCallback cb);

    // Probe callback (called when PROBE_ACK received with channel report)
    void setProbeCompleteCallback(ProbeCompleteCallback cb) { on_probe_complete_ = cb; }

    // Set channel measurement provider (used when responding to PROBE)
    void setChannelMeasurementCallback(ChannelMeasurementCallback cb) { on_measure_channel_ = cb; }

    // --- State ---

    ConnectionState getState() const { return state_; }
    std::string getRemoteCallsign() const { return remote_call_; }
    bool isConnected() const { return state_ == ConnectionState::CONNECTED; }
    ConnectionStats getStats() const;
    void resetStats();

    // --- Waveform Mode ---

    // Get current negotiated waveform mode
    WaveformMode getNegotiatedMode() const { return negotiated_mode_; }

    // Set preferred mode for future connections
    void setPreferredMode(WaveformMode mode) { config_.preferred_mode = mode; }

    // Set mode capabilities
    void setModeCapabilities(uint8_t caps) { config_.mode_capabilities = caps; }

    // Callback for mode negotiation (notifies when mode is agreed)
    using ModeNegotiatedCallback = std::function<void(WaveformMode mode)>;
    void setModeNegotiatedCallback(ModeNegotiatedCallback cb) { on_mode_negotiated_ = cb; }

    // Full reset (back to DISCONNECTED)
    void reset();

private:
    ConnectionConfig config_;
    ConnectionState state_ = ConnectionState::DISCONNECTED;

    // Callsigns
    std::string local_call_;
    std::string remote_call_;
    std::string pending_remote_call_;  // For incoming calls before accept

    // Waveform mode
    WaveformMode negotiated_mode_ = WaveformMode::OFDM;
    uint8_t remote_capabilities_ = ModeCapabilities::OFDM;
    WaveformMode remote_preferred_ = WaveformMode::OFDM;

    // ARQ for reliable data transfer
    ARQController arq_;

    // File transfer controller
    FileTransferController file_transfer_;

    // Connection timing
    uint32_t timeout_remaining_ms_ = 0;
    int connect_retry_count_ = 0;
    int probe_retry_count_ = 0;
    uint32_t connected_time_ms_ = 0;

    // Channel probing state
    ChannelReport last_channel_report_;  // Last received channel report
    bool probe_complete_ = false;        // True if we have a valid channel report
    bool connect_after_probe_ = false;   // If true, auto-connect after probe completes

    // Statistics
    ConnectionStats stats_;

    // Callbacks
    TransmitCallback on_transmit_;
    RawTransmitCallback on_transmit_raw_;  // For v2 frames
    ConnectedCallback on_connected_;
    DisconnectedCallback on_disconnected_;
    MessageReceivedCallback on_message_received_;
    MessageSentCallback on_message_sent_;
    IncomingCallCallback on_incoming_call_;
    DataReceivedCallback on_data_received_;
    ModeNegotiatedCallback on_mode_negotiated_;
    ProbeCompleteCallback on_probe_complete_;
    ChannelMeasurementCallback on_measure_channel_;

    // Internal handlers
    void handleConnect(const Frame& frame);
    void handleConnectAck(const Frame& frame);
    void handleConnectNak(const Frame& frame);
    void handleDisconnect(const Frame& frame);
    void handleProbe(const Frame& frame);
    void handleProbeAck(const Frame& frame);
    void handleDataPayload(const Bytes& payload, bool more_data);

    void transmitFrame(const Frame& frame);
    void enterConnected();
    void enterDisconnected(const std::string& reason);

    // Mode negotiation helper
    // Given local and remote capabilities + preferences, choose best mode
    WaveformMode negotiateMode(uint8_t remote_caps, WaveformMode remote_pref);

    // File transfer helpers
    void sendNextFileChunk();
};

} // namespace protocol
} // namespace ultra
