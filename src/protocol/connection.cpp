#include "connection.hpp"
#include "ultra/logging.hpp"

namespace ultra {
namespace protocol {

const char* connectionStateToString(ConnectionState state) {
    switch (state) {
        case ConnectionState::DISCONNECTED:  return "DISCONNECTED";
        case ConnectionState::CONNECTING:    return "CONNECTING";
        case ConnectionState::CONNECTED:     return "CONNECTED";
        case ConnectionState::DISCONNECTING: return "DISCONNECTING";
        default: return "UNKNOWN";
    }
}

Connection::Connection(const ConnectionConfig& config)
    : config_(config)
    , arq_(config.arq)
{
    // Wire up ARQ callbacks
    arq_.setTransmitCallback([this](const Frame& f) {
        transmitFrame(f);
    });

    arq_.setDataReceivedCallback([this](const Bytes& data) {
        // Route through data payload handler for file/text discrimination
        handleDataPayload(data, arq_.lastRxHadMoreData());
    });

    arq_.setSendCompleteCallback([this](bool success) {
        // Check if this was a file transfer chunk
        if (file_transfer_.getState() == FileTransferState::SENDING) {
            if (success) {
                file_transfer_.onChunkAcked();
                // Send next chunk if available
                sendNextFileChunk();
            } else {
                file_transfer_.onSendFailed();
            }
        } else {
            // Regular message
            if (on_message_sent_) {
                on_message_sent_(success);
            }
        }
    });

    // Wire up file transfer callbacks (forward to our callbacks)
    file_transfer_.setProgressCallback([this](const FileTransferProgress& p) {
        // Progress callbacks are set directly on file_transfer_
    });
}

void Connection::setLocalCallsign(const std::string& call) {
    local_call_ = Frame::sanitizeCallsign(call);
    LOG_MODEM(INFO, "Connection: Local callsign set to %s", local_call_.c_str());
}

bool Connection::connect(const std::string& remote_call) {
    if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot connect, state=%s",
                  connectionStateToString(state_));
        return false;
    }

    if (local_call_.empty()) {
        LOG_MODEM(ERROR, "Connection: Local callsign not set");
        return false;
    }

    remote_call_ = Frame::sanitizeCallsign(remote_call);
    if (remote_call_.empty() || !Frame::isValidCallsign(remote_call_)) {
        LOG_MODEM(ERROR, "Connection: Invalid remote callsign: %s", remote_call.c_str());
        return false;
    }

    LOG_MODEM(INFO, "Connection: Connecting to %s", remote_call_.c_str());

    // Send CONNECT frame
    Frame connect_frame = Frame::makeConnect(local_call_, remote_call_);
    transmitFrame(connect_frame);

    state_ = ConnectionState::CONNECTING;
    connect_retry_count_ = 0;
    timeout_remaining_ms_ = config_.connect_timeout_ms;
    stats_.connects_initiated++;

    return true;
}

void Connection::acceptCall() {
    if (state_ != ConnectionState::DISCONNECTED || pending_remote_call_.empty()) {
        LOG_MODEM(WARN, "Connection: No pending call to accept");
        return;
    }

    LOG_MODEM(INFO, "Connection: Accepting call from %s", pending_remote_call_.c_str());

    remote_call_ = pending_remote_call_;
    pending_remote_call_.clear();

    // Send CONNECT_ACK
    Frame ack_frame = Frame::makeConnectAck(local_call_, remote_call_);
    transmitFrame(ack_frame);

    enterConnected();
}

void Connection::rejectCall() {
    if (pending_remote_call_.empty()) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Rejecting call from %s", pending_remote_call_.c_str());

    // Send CONNECT_NAK
    Frame nak_frame = Frame::makeConnectNak(local_call_, pending_remote_call_);
    transmitFrame(nak_frame);

    pending_remote_call_.clear();
}

void Connection::disconnect() {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    if (state_ == ConnectionState::CONNECTING) {
        // Cancel connection attempt
        enterDisconnected("Cancelled");
        return;
    }

    if (state_ == ConnectionState::CONNECTED) {
        LOG_MODEM(INFO, "Connection: Disconnecting from %s", remote_call_.c_str());

        // Send DISCONNECT frame
        Frame disc_frame = Frame::makeDisconnect(local_call_, remote_call_);
        transmitFrame(disc_frame);

        state_ = ConnectionState::DISCONNECTING;
        timeout_remaining_ms_ = config_.disconnect_timeout_ms;
        stats_.disconnects++;
    }
}

bool Connection::sendMessage(const std::string& text) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send, not connected");
        return false;
    }

    return arq_.sendData(text);
}

bool Connection::isReadyToSend() const {
    return state_ == ConnectionState::CONNECTED && arq_.isReadyToSend() &&
           !file_transfer_.isBusy();
}

// --- File Transfer ---

bool Connection::sendFile(const std::string& filepath) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send file, not connected");
        return false;
    }

    if (file_transfer_.isBusy()) {
        LOG_MODEM(WARN, "Connection: File transfer already in progress");
        return false;
    }

    if (!arq_.isReadyToSend()) {
        LOG_MODEM(WARN, "Connection: ARQ busy, cannot start file transfer");
        return false;
    }

    LOG_MODEM(INFO, "Connection: Starting file transfer: %s", filepath.c_str());

    if (!file_transfer_.startSend(filepath)) {
        LOG_MODEM(ERROR, "Connection: Failed to start file transfer");
        return false;
    }

    // Send first chunk
    sendNextFileChunk();
    return true;
}

void Connection::setReceiveDirectory(const std::string& dir) {
    file_transfer_.setReceiveDirectory(dir);
}

void Connection::cancelFileTransfer() {
    file_transfer_.cancel();
}

bool Connection::isFileTransferInProgress() const {
    return file_transfer_.isBusy();
}

FileTransferProgress Connection::getFileProgress() const {
    return file_transfer_.getProgress();
}

void Connection::sendNextFileChunk() {
    if (file_transfer_.getState() != FileTransferState::SENDING) {
        return;
    }

    if (!arq_.isReadyToSend()) {
        return;
    }

    Bytes chunk = file_transfer_.getNextChunk();
    if (chunk.empty()) {
        return;
    }

    // Set MORE_DATA flag if more chunks follow
    uint8_t flags = file_transfer_.hasMoreChunks() ? FrameFlags::MORE_DATA : FrameFlags::NONE;

    arq_.sendDataWithFlags(chunk, flags);
}

void Connection::handleDataPayload(const Bytes& payload, bool more_data) {
    if (payload.empty()) {
        return;
    }

    // Check if this is a file transfer frame
    if (file_transfer_.processPayload(payload, more_data)) {
        // Was a file transfer frame, already handled
        LOG_MODEM(DEBUG, "Connection: Processed file transfer payload (%zu bytes)",
                  payload.size());

        // Also notify raw data callback if set
        if (on_data_received_) {
            on_data_received_(payload, more_data);
        }
        return;
    }

    // Regular text message (starts with TEXT_MESSAGE type or is legacy)
    // Skip type byte if present
    size_t start = 0;
    if (!payload.empty() && payload[0] == static_cast<uint8_t>(PayloadType::TEXT_MESSAGE)) {
        start = 1;
    }

    std::string text(payload.begin() + start, payload.end());

    if (on_message_received_) {
        on_message_received_(text);
    }

    if (on_data_received_) {
        on_data_received_(payload, more_data);
    }
}

void Connection::onFrameReceived(const Frame& frame) {
    // Check if frame is for us
    if (!frame.isForCallsign(local_call_)) {
        return;
    }

    LOG_MODEM(DEBUG, "Connection: Received %s from %s",
              frameTypeToString(frame.type), frame.src_call.c_str());

    switch (frame.type) {
        case FrameType::CONNECT:
            handleConnect(frame);
            break;
        case FrameType::CONNECT_ACK:
            handleConnectAck(frame);
            break;
        case FrameType::CONNECT_NAK:
            handleConnectNak(frame);
            break;
        case FrameType::DISCONNECT:
            handleDisconnect(frame);
            break;
        case FrameType::DATA:
        case FrameType::ACK:
        case FrameType::NAK:
            // Pass to ARQ layer
            if (state_ == ConnectionState::CONNECTED) {
                arq_.onFrameReceived(frame);
            }
            break;
        case FrameType::BEACON:
            // Could notify application of heard stations
            LOG_MODEM(DEBUG, "Connection: Heard beacon from %s", frame.src_call.c_str());
            break;
    }
}

void Connection::handleConnect(const Frame& frame) {
    if (state_ != ConnectionState::DISCONNECTED) {
        // Already busy, reject
        LOG_MODEM(WARN, "Connection: Rejecting CONNECT from %s (busy, state=%s)",
                  frame.src_call.c_str(), connectionStateToString(state_));
        Frame nak = Frame::makeConnectNak(local_call_, frame.src_call);
        transmitFrame(nak);
        return;
    }

    LOG_MODEM(INFO, "Connection: Incoming call from %s", frame.src_call.c_str());
    stats_.connects_received++;

    if (config_.auto_accept) {
        // Auto-accept the connection
        remote_call_ = frame.src_call;
        Frame ack = Frame::makeConnectAck(local_call_, remote_call_);
        transmitFrame(ack);
        enterConnected();
    } else {
        // Store for manual accept/reject
        pending_remote_call_ = frame.src_call;
        if (on_incoming_call_) {
            on_incoming_call_(frame.src_call);
        }
    }
}

void Connection::handleConnectAck(const Frame& frame) {
    if (state_ != ConnectionState::CONNECTING) {
        LOG_MODEM(DEBUG, "Connection: Ignoring CONNECT_ACK (state=%s)",
                  connectionStateToString(state_));
        return;
    }

    if (frame.src_call != remote_call_) {
        LOG_MODEM(WARN, "Connection: CONNECT_ACK from wrong station %s (expected %s)",
                  frame.src_call.c_str(), remote_call_.c_str());
        return;
    }

    LOG_MODEM(INFO, "Connection: Connected to %s", remote_call_.c_str());
    enterConnected();
}

void Connection::handleConnectNak(const Frame& frame) {
    if (state_ != ConnectionState::CONNECTING) {
        return;
    }

    if (frame.src_call != remote_call_) {
        return;
    }

    LOG_MODEM(WARN, "Connection: Connection rejected by %s", remote_call_.c_str());
    stats_.connects_failed++;
    enterDisconnected("Connection rejected");
}

void Connection::handleDisconnect(const Frame& frame) {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    if (frame.src_call != remote_call_) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Disconnect from %s", remote_call_.c_str());

    // ACK the disconnect
    Frame ack = Frame::makeAck(local_call_, remote_call_, 0);
    transmitFrame(ack);

    stats_.disconnects++;
    enterDisconnected("Remote disconnected");
}

void Connection::tick(uint32_t elapsed_ms) {
    switch (state_) {
        case ConnectionState::CONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                // Connection timeout
                connect_retry_count_++;
                if (connect_retry_count_ >= config_.connect_retries) {
                    LOG_MODEM(ERROR, "Connection: Connect failed after %d attempts",
                              config_.connect_retries);
                    stats_.connects_failed++;
                    enterDisconnected("Connection timeout");
                } else {
                    // Retry
                    LOG_MODEM(WARN, "Connection: Connect timeout, retrying (%d/%d)",
                              connect_retry_count_ + 1, config_.connect_retries);
                    Frame connect_frame = Frame::makeConnect(local_call_, remote_call_);
                    transmitFrame(connect_frame);
                    timeout_remaining_ms_ = config_.connect_timeout_ms;
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        case ConnectionState::CONNECTED:
            connected_time_ms_ += elapsed_ms;
            stats_.connected_time_ms = connected_time_ms_;
            arq_.tick(elapsed_ms);
            break;

        case ConnectionState::DISCONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                // Disconnect timeout - just go to disconnected anyway
                LOG_MODEM(DEBUG, "Connection: Disconnect timeout, forcing disconnect");
                enterDisconnected("Disconnect timeout");
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        default:
            break;
    }
}

void Connection::transmitFrame(const Frame& frame) {
    LOG_MODEM(INFO, "TX >> %s [%s -> %s] seq=%d payload=%zu bytes",
              frameTypeToString(frame.type),
              frame.src_call.c_str(), frame.dst_call.c_str(),
              frame.sequence, frame.payload.size());
    if (on_transmit_) {
        on_transmit_(frame);
    }
}

void Connection::enterConnected() {
    state_ = ConnectionState::CONNECTED;
    connected_time_ms_ = 0;

    // Configure ARQ with callsigns
    arq_.setCallsigns(local_call_, remote_call_);
    arq_.reset();

    LOG_MODEM(INFO, "Connection: Now CONNECTED to %s", remote_call_.c_str());

    if (on_connected_) {
        on_connected_();
    }
}

void Connection::enterDisconnected(const std::string& reason) {
    state_ = ConnectionState::DISCONNECTED;
    std::string old_remote = remote_call_;
    remote_call_.clear();
    pending_remote_call_.clear();
    arq_.reset();
    file_transfer_.cancel();

    LOG_MODEM(INFO, "Connection: Disconnected from %s (%s)",
              old_remote.c_str(), reason.c_str());

    if (on_disconnected_) {
        on_disconnected_(reason);
    }
}

void Connection::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void Connection::setConnectedCallback(ConnectedCallback cb) {
    on_connected_ = std::move(cb);
}

void Connection::setDisconnectedCallback(DisconnectedCallback cb) {
    on_disconnected_ = std::move(cb);
}

void Connection::setMessageReceivedCallback(MessageReceivedCallback cb) {
    on_message_received_ = std::move(cb);
}

void Connection::setMessageSentCallback(MessageSentCallback cb) {
    on_message_sent_ = std::move(cb);
}

void Connection::setIncomingCallCallback(IncomingCallCallback cb) {
    on_incoming_call_ = std::move(cb);
}

void Connection::setDataReceivedCallback(DataReceivedCallback cb) {
    on_data_received_ = std::move(cb);
}

void Connection::setFileProgressCallback(FileProgressCallback cb) {
    file_transfer_.setProgressCallback(std::move(cb));
}

void Connection::setFileReceivedCallback(FileReceivedCallback cb) {
    file_transfer_.setReceivedCallback(std::move(cb));
}

void Connection::setFileSentCallback(FileSentCallback cb) {
    file_transfer_.setSentCallback(std::move(cb));
}

ConnectionStats Connection::getStats() const {
    ConnectionStats s = stats_;
    s.arq = arq_.getStats();
    return s;
}

void Connection::resetStats() {
    stats_ = ConnectionStats{};
    arq_.resetStats();
}

void Connection::reset() {
    state_ = ConnectionState::DISCONNECTED;
    remote_call_.clear();
    pending_remote_call_.clear();
    timeout_remaining_ms_ = 0;
    connect_retry_count_ = 0;
    connected_time_ms_ = 0;
    arq_.reset();
    file_transfer_.cancel();
    LOG_MODEM(DEBUG, "Connection: Full reset");
}

} // namespace protocol
} // namespace ultra
