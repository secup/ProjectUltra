#pragma once

#include "connection.hpp"
#include "frame_v2.hpp"
#include "frame.hpp"
#include <functional>
#include <mutex>

namespace ultra {
namespace protocol {

/**
 * Protocol Engine
 *
 * High-level interface that integrates the protocol stack:
 * - Frame serialization/deserialization
 * - Connection management
 * - ARQ for reliable delivery
 *
 * Connects to ModemEngine for actual TX/RX of audio.
 *
 * Usage:
 *   1. Create ProtocolEngine
 *   2. Set callbacks (on_tx_data for modem output, on_message for received text)
 *   3. Call setLocalCallsign()
 *   4. Call connect() to establish link, or just listen
 *   5. Use sendMessage() to send text
 *   6. Call onRxData() when modem receives decoded bytes
 *   7. Call tick() periodically from main loop
 */
class ProtocolEngine {
public:
    // TX callback: called when protocol layer needs to transmit data
    // The application should pass this to ModemEngine::transmit()
    using TxDataCallback = std::function<void(const Bytes& data)>;

    // RX callbacks
    using MessageReceivedCallback = std::function<void(const std::string& from, const std::string& text)>;
    using ConnectionChangedCallback = std::function<void(ConnectionState state, const std::string& remote)>;
    using IncomingCallCallback = std::function<void(const std::string& from)>;

    // File transfer callbacks
    using FileProgressCallback = Connection::FileProgressCallback;
    using FileReceivedCallback = Connection::FileReceivedCallback;
    using FileSentCallback = Connection::FileSentCallback;

    // Mode negotiation callback
    using ModeNegotiatedCallback = Connection::ModeNegotiatedCallback;

    // Channel probing callback
    using ProbeCompleteCallback = Connection::ProbeCompleteCallback;

    explicit ProtocolEngine(const ConnectionConfig& config = ConnectionConfig{});

    // --- Configuration ---

    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const;

    // Set auto-accept mode (whether to auto-accept incoming calls)
    void setAutoAccept(bool auto_accept);
    bool getAutoAccept() const;

    // --- Callbacks ---

    void setTxDataCallback(TxDataCallback cb);
    void setMessageReceivedCallback(MessageReceivedCallback cb);
    void setConnectionChangedCallback(ConnectionChangedCallback cb);
    void setIncomingCallCallback(IncomingCallCallback cb);

    // --- Connection Control ---

    bool connect(const std::string& remote_call);
    void acceptCall();
    void rejectCall();
    void disconnect();

    // --- Channel Probing (Link Establishment) ---

    // Probe channel to remote station (measures SNR, delay/Doppler spread)
    // After probe completes, call connectAfterProbe() to connect
    bool probe(const std::string& remote_call);

    // Connect using probe results (uses recommended mode from channel report)
    bool connectAfterProbe();

    // Check if probing is in progress
    bool isProbing() const;

    // Get last channel report (valid after probe completes)
    const ChannelReport& getLastChannelReport() const;

    // Set callback for when probe completes
    void setProbeCompleteCallback(ProbeCompleteCallback cb);

    // Set channel measurement provider (modem provides channel quality)
    using ChannelMeasurementCallback = Connection::ChannelMeasurementCallback;
    void setChannelMeasurementCallback(ChannelMeasurementCallback cb);

    // --- Data Transfer ---

    bool sendMessage(const std::string& text);
    bool isReadyToSend() const;

    // --- File Transfer ---

    bool sendFile(const std::string& filepath);
    void setReceiveDirectory(const std::string& dir);
    void cancelFileTransfer();
    bool isFileTransferInProgress() const;
    FileTransferProgress getFileProgress() const;

    void setFileProgressCallback(FileProgressCallback cb);
    void setFileReceivedCallback(FileReceivedCallback cb);
    void setFileSentCallback(FileSentCallback cb);

    // --- Modem Interface ---

    // Call this when modem receives decoded data
    void onRxData(const Bytes& data);

    // Periodic update (call from main loop)
    void tick(uint32_t elapsed_ms);

    // --- State ---

    ConnectionState getState() const;
    std::string getRemoteCallsign() const;
    bool isConnected() const;

    ConnectionStats getStats() const;
    void resetStats();
    void reset();

    // --- Waveform Mode ---

    // Get the negotiated waveform mode (valid after connection established)
    WaveformMode getNegotiatedMode() const;

    // Set preferred mode for outgoing connections
    void setPreferredMode(WaveformMode mode);

    // Set which modes this station supports
    void setModeCapabilities(uint8_t caps);

    // Callback when mode is negotiated
    void setModeNegotiatedCallback(ModeNegotiatedCallback cb);

private:
    Connection connection_;

    // Callbacks
    TxDataCallback on_tx_data_;
    MessageReceivedCallback on_message_received_;
    ConnectionChangedCallback on_connection_changed_;
    IncomingCallCallback on_incoming_call_;

    // Thread safety
    mutable std::mutex mutex_;

    // RX frame assembly buffer
    Bytes rx_buffer_;

    // TX queue for deferred transmission (avoids re-entrancy during RX processing)
    std::vector<Bytes> tx_queue_;
    bool defer_tx_ = false;  // When true, queue TX instead of immediate send

    // Handle transmit request from Connection
    void handleTxFrame(const Frame& frame);

    // Try to parse frames from RX buffer
    void processRxBuffer();
};

} // namespace protocol
} // namespace ultra
