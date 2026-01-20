#pragma once

#include "connection.hpp"
#include "frame_v2.hpp"
#include <functional>
#include <mutex>

namespace ultra {
namespace protocol {

/**
 * Protocol Engine
 *
 * High-level interface that integrates the protocol stack:
 * - Frame serialization/deserialization (v2 format)
 * - Connection management
 * - ARQ for reliable delivery
 *
 * Connects to ModemEngine for actual TX/RX of audio.
 */
class ProtocolEngine {
public:
    using TxDataCallback = std::function<void(const Bytes& data)>;
    using MessageReceivedCallback = std::function<void(const std::string& from, const std::string& text)>;
    using ConnectionChangedCallback = std::function<void(ConnectionState state, const std::string& remote)>;
    using IncomingCallCallback = std::function<void(const std::string& from)>;

    using FileProgressCallback = Connection::FileProgressCallback;
    using FileReceivedCallback = Connection::FileReceivedCallback;
    using FileSentCallback = Connection::FileSentCallback;

    using ModeNegotiatedCallback = Connection::ModeNegotiatedCallback;

    explicit ProtocolEngine(const ConnectionConfig& config = ConnectionConfig{});

    // --- Configuration ---

    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const;

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

    void onRxData(const Bytes& data);
    void tick(uint32_t elapsed_ms);

    // --- State ---

    ConnectionState getState() const;
    std::string getRemoteCallsign() const;
    bool isConnected() const;

    ConnectionStats getStats() const;
    void resetStats();
    void reset();

    // --- Waveform Mode ---

    WaveformMode getNegotiatedMode() const;
    void setPreferredMode(WaveformMode mode);
    void setModeCapabilities(uint8_t caps);
    void setModeNegotiatedCallback(ModeNegotiatedCallback cb);

    // --- Adaptive Data Mode ---

    // Set measured SNR from modem (used for adaptive mode selection)
    void setMeasuredSNR(float snr_db);
    float getMeasuredSNR() const;

    // Get current data mode
    Modulation getDataModulation() const;
    CodeRate getDataCodeRate() const;

    // Set callback for data mode changes
    using DataModeChangedCallback = Connection::DataModeChangedCallback;
    void setDataModeChangedCallback(DataModeChangedCallback cb);

private:
    Connection connection_;

    TxDataCallback on_tx_data_;
    MessageReceivedCallback on_message_received_;
    ConnectionChangedCallback on_connection_changed_;
    IncomingCallCallback on_incoming_call_;

    mutable std::mutex mutex_;

    Bytes rx_buffer_;

    std::vector<Bytes> tx_queue_;
    bool defer_tx_ = false;

    void handleTxFrame(const Bytes& frame_data);
    void processRxBuffer();
};

} // namespace protocol
} // namespace ultra
