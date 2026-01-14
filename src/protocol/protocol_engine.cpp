#include "protocol_engine.hpp"
#include "ultra/logging.hpp"

namespace ultra {
namespace protocol {

ProtocolEngine::ProtocolEngine(const ConnectionConfig& config)
    : connection_(config)
{
    // Wire up Connection callbacks
    connection_.setTransmitCallback([this](const Frame& f) {
        handleTxFrame(f);
    });

    connection_.setConnectedCallback([this]() {
        if (on_connection_changed_) {
            on_connection_changed_(ConnectionState::CONNECTED, connection_.getRemoteCallsign());
        }
    });

    connection_.setDisconnectedCallback([this](const std::string& reason) {
        if (on_connection_changed_) {
            on_connection_changed_(ConnectionState::DISCONNECTED, reason);
        }
    });

    connection_.setMessageReceivedCallback([this](const std::string& text) {
        if (on_message_received_) {
            on_message_received_(connection_.getRemoteCallsign(), text);
        }
    });

    connection_.setIncomingCallCallback([this](const std::string& from) {
        if (on_incoming_call_) {
            on_incoming_call_(from);
        }
    });
}

void ProtocolEngine::setLocalCallsign(const std::string& call) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.setLocalCallsign(call);
}

std::string ProtocolEngine::getLocalCallsign() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.getLocalCallsign();
}

void ProtocolEngine::setAutoAccept(bool auto_accept) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.setAutoAccept(auto_accept);
}

bool ProtocolEngine::getAutoAccept() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.getAutoAccept();
}

void ProtocolEngine::setTxDataCallback(TxDataCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_tx_data_ = std::move(cb);
}

void ProtocolEngine::setMessageReceivedCallback(MessageReceivedCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_message_received_ = std::move(cb);
}

void ProtocolEngine::setConnectionChangedCallback(ConnectionChangedCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_connection_changed_ = std::move(cb);
}

void ProtocolEngine::setIncomingCallCallback(IncomingCallCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    on_incoming_call_ = std::move(cb);
}

bool ProtocolEngine::connect(const std::string& remote_call) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool result = connection_.connect(remote_call);
    if (result && on_connection_changed_) {
        on_connection_changed_(ConnectionState::CONNECTING, remote_call);
    }
    return result;
}

void ProtocolEngine::acceptCall() {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.acceptCall();
}

void ProtocolEngine::rejectCall() {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.rejectCall();
}

void ProtocolEngine::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.disconnect();
    if (on_connection_changed_) {
        on_connection_changed_(ConnectionState::DISCONNECTING, "");
    }
}

bool ProtocolEngine::sendMessage(const std::string& text) {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.sendMessage(text);
}

bool ProtocolEngine::isReadyToSend() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.isReadyToSend();
}

// --- File Transfer ---

bool ProtocolEngine::sendFile(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.sendFile(filepath);
}

void ProtocolEngine::setReceiveDirectory(const std::string& dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.setReceiveDirectory(dir);
}

void ProtocolEngine::cancelFileTransfer() {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.cancelFileTransfer();
}

bool ProtocolEngine::isFileTransferInProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.isFileTransferInProgress();
}

FileTransferProgress ProtocolEngine::getFileProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.getFileProgress();
}

void ProtocolEngine::setFileProgressCallback(FileProgressCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.setFileProgressCallback(std::move(cb));
}

void ProtocolEngine::setFileReceivedCallback(FileReceivedCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.setFileReceivedCallback(std::move(cb));
}

void ProtocolEngine::setFileSentCallback(FileSentCallback cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.setFileSentCallback(std::move(cb));
}

void ProtocolEngine::onRxData(const Bytes& data) {
    std::lock_guard<std::mutex> lock(mutex_);

    LOG_MODEM(INFO, "Protocol RX: %zu bytes from modem (buffer now %zu)",
              data.size(), rx_buffer_.size() + data.size());

    // Append to RX buffer
    rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.end());

    // Defer TX during RX processing to avoid re-entrancy
    // TX will be flushed during tick()
    defer_tx_ = true;

    // Try to parse frames
    processRxBuffer();

    defer_tx_ = false;
    // NOTE: Don't flush TX queue here - do it in tick() to completely
    // separate RX and TX processing paths
}

void ProtocolEngine::processRxBuffer() {
    // Look for frame magic byte
    while (!rx_buffer_.empty()) {
        // Find magic byte
        auto it = std::find(rx_buffer_.begin(), rx_buffer_.end(), Frame::MAGIC);
        if (it == rx_buffer_.end()) {
            // No magic byte found, clear buffer
            rx_buffer_.clear();
            return;
        }

        // Discard bytes before magic
        if (it != rx_buffer_.begin()) {
            rx_buffer_.erase(rx_buffer_.begin(), it);
        }

        // Check if we have minimum frame size
        if (rx_buffer_.size() < Frame::MIN_SIZE) {
            // Need more data
            return;
        }

        // Try to parse frame
        ByteSpan span(rx_buffer_.data(), rx_buffer_.size());
        auto frame_opt = Frame::deserialize(span);

        if (frame_opt) {
            // Successfully parsed frame
            Frame& frame = *frame_opt;
            LOG_MODEM(INFO, "RX << %s [%s -> %s] seq=%d payload=%zu bytes",
                      frameTypeToString(frame.type),
                      frame.src_call.c_str(), frame.dst_call.c_str(),
                      frame.sequence, frame.payload.size());

            // Calculate frame size and remove from buffer
            size_t frame_size = Frame::HEADER_SIZE + frame.payload.size() + Frame::CRC_SIZE;
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_size);

            // Pass to connection
            connection_.onFrameReceived(frame);

        } else {
            // Parse failed - could be:
            // 1. Not enough data yet (need more bytes)
            // 2. Corrupted frame (bad CRC)

            // Debug: dump entire frame buffer (up to 64 bytes)
            if (rx_buffer_.size() >= 24) {
                char hex[200];
                size_t dump_len = std::min(rx_buffer_.size(), size_t(64));
                for (size_t i = 0; i < dump_len; i++) {
                    snprintf(hex + i*3, 4, "%02x ", rx_buffer_[i]);
                }
                LOG_MODEM(INFO, "RX buffer (%zu bytes): %s", rx_buffer_.size(), hex);
            }

            // Check if we have enough data for the frame based on length field
            if (rx_buffer_.size() >= Frame::HEADER_SIZE) {
                // Read length field (bytes 20-21, after callsigns)
                uint16_t payload_len = (static_cast<uint16_t>(rx_buffer_[20]) << 8) |
                                        static_cast<uint16_t>(rx_buffer_[21]);

                if (payload_len <= Frame::MAX_PAYLOAD) {
                    size_t expected_size = Frame::HEADER_SIZE + payload_len + Frame::CRC_SIZE;

                    if (rx_buffer_.size() >= expected_size) {
                        // We have all the data but CRC failed - corrupted frame
                        LOG_MODEM(WARN, "ProtocolEngine: Corrupted frame (CRC failed), discarding");
                        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + expected_size);
                        continue;
                    }
                }
            }

            // Not enough data yet, wait for more
            return;
        }
    }
}

void ProtocolEngine::tick(uint32_t elapsed_ms) {
    // First, flush any pending TX (queued during RX processing)
    // Do this OUTSIDE the mutex to avoid holding lock during TX
    std::vector<Bytes> to_send;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        to_send = std::move(tx_queue_);
        tx_queue_.clear();
    }

    // Send queued frames
    for (const auto& tx_data : to_send) {
        if (on_tx_data_) {
            on_tx_data_(tx_data);
        }
    }

    // Now do the regular tick (with lock)
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.tick(elapsed_ms);
}

ConnectionState ProtocolEngine::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.getState();
}

std::string ProtocolEngine::getRemoteCallsign() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.getRemoteCallsign();
}

bool ProtocolEngine::isConnected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.isConnected();
}

ConnectionStats ProtocolEngine::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connection_.getStats();
}

void ProtocolEngine::resetStats() {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.resetStats();
}

void ProtocolEngine::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_.reset();
    rx_buffer_.clear();
    tx_queue_.clear();
    defer_tx_ = false;
}

void ProtocolEngine::handleTxFrame(const Frame& frame) {
    Bytes data = frame.serialize();
    LOG_MODEM(INFO, "Protocol TX: %zu bytes -> modem%s", data.size(),
              defer_tx_ ? " (queued)" : "");

    // Debug: dump entire frame (up to 64 bytes)
    {
        char hex[200];
        size_t dump_len = std::min(data.size(), size_t(64));
        for (size_t i = 0; i < dump_len; i++) {
            snprintf(hex + i*3, 4, "%02x ", data[i]);
        }
        LOG_MODEM(INFO, "TX frame (%zu bytes): %s", data.size(), hex);
    }

    if (defer_tx_) {
        // Queue for later transmission (we're inside RX processing)
        tx_queue_.push_back(std::move(data));
    } else if (on_tx_data_) {
        // Send immediately
        on_tx_data_(data);
    }
}

} // namespace protocol
} // namespace ultra
