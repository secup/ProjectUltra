#pragma once

#include "connection.hpp"
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

    explicit ProtocolEngine(const ConnectionConfig& config = ConnectionConfig{});

    // --- Configuration ---

    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const;

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

    // --- Data Transfer ---

    bool sendMessage(const std::string& text);
    bool isReadyToSend() const;

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

    // Handle transmit request from Connection
    void handleTxFrame(const Frame& frame);

    // Try to parse frames from RX buffer
    void processRxBuffer();
};

} // namespace protocol
} // namespace ultra
