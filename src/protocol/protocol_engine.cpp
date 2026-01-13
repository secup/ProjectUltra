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

void ProtocolEngine::onRxData(const Bytes& data) {
    std::lock_guard<std::mutex> lock(mutex_);

    LOG_MODEM(INFO, "Protocol RX: %zu bytes from modem (buffer now %zu)",
              data.size(), rx_buffer_.size() + data.size());

    // Append to RX buffer
    rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.end());

    // Try to parse frames
    processRxBuffer();
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
}

void ProtocolEngine::handleTxFrame(const Frame& frame) {
    if (on_tx_data_) {
        Bytes data = frame.serialize();
        LOG_MODEM(INFO, "Protocol TX: %zu bytes -> modem", data.size());
        on_tx_data_(data);
    }
}

} // namespace protocol
} // namespace ultra
