#include "arq.hpp"
#include "ultra/logging.hpp"

namespace ultra {
namespace protocol {

StopAndWaitARQ::StopAndWaitARQ(const ARQConfig& config)
    : config_(config)
{
}

void StopAndWaitARQ::setCallsigns(const std::string& local, const std::string& remote) {
    local_call_ = sanitizeCallsign(local);
    remote_call_ = sanitizeCallsign(remote);
}

bool StopAndWaitARQ::sendData(const Bytes& data) {
    if (state_ != State::IDLE) {
        LOG_MODEM(WARN, "ARQ: Cannot send, state=%d (busy)", static_cast<int>(state_));
        return false;
    }

    if (local_call_.empty() || remote_call_.empty()) {
        LOG_MODEM(ERROR, "ARQ: Callsigns not set");
        return false;
    }

    // Create DATA frame
    pending_frame_ = Frame::makeData(local_call_, remote_call_, tx_seq_, data);
    retry_count_ = 0;

    // Transmit
    transmitFrame(pending_frame_);
    state_ = State::WAIT_ACK;
    timeout_remaining_ms_ = config_.ack_timeout_ms;

    stats_.frames_sent++;
    LOG_MODEM(DEBUG, "ARQ: Sent DATA seq=%d, %zu bytes", tx_seq_, data.size());

    return true;
}

bool StopAndWaitARQ::sendData(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return sendData(data);
}

bool StopAndWaitARQ::sendDataWithFlags(const Bytes& data, uint8_t flags) {
    if (state_ != State::IDLE) {
        LOG_MODEM(WARN, "ARQ: Cannot send, state=%d (busy)", static_cast<int>(state_));
        return false;
    }

    if (local_call_.empty() || remote_call_.empty()) {
        LOG_MODEM(ERROR, "ARQ: Callsigns not set");
        return false;
    }

    // Create DATA frame with custom flags
    pending_frame_ = Frame::makeData(local_call_, remote_call_, tx_seq_, data);
    pending_frame_.flags = flags;
    retry_count_ = 0;

    // Transmit
    transmitFrame(pending_frame_);
    state_ = State::WAIT_ACK;
    timeout_remaining_ms_ = config_.ack_timeout_ms;

    stats_.frames_sent++;
    LOG_MODEM(DEBUG, "ARQ: Sent DATA seq=%d, %zu bytes, flags=0x%02x",
              tx_seq_, data.size(), flags);

    return true;
}

bool StopAndWaitARQ::isReadyToSend() const {
    return state_ == State::IDLE;
}

void StopAndWaitARQ::onFrameReceived(const Frame& frame) {
    // Check if frame is for us
    if (!frame.isForCallsign(local_call_)) {
        LOG_MODEM(TRACE, "ARQ: Ignoring frame for %s (we are %s)",
                  frame.dst_call.c_str(), local_call_.c_str());
        return;
    }

    LOG_MODEM(DEBUG, "ARQ: Received %s from %s seq=%d",
              frameTypeToString(frame.type), frame.src_call.c_str(), frame.sequence);

    switch (frame.type) {
        case FrameType::DATA:
            handleDataFrame(frame);
            break;
        case FrameType::ACK:
            handleAckFrame(frame);
            break;
        case FrameType::NAK:
            handleNakFrame(frame);
            break;
        default:
            // Other frame types handled by Connection layer
            break;
    }
}

void StopAndWaitARQ::handleDataFrame(const Frame& frame) {
    // Track flags from this frame
    last_rx_flags_ = frame.flags;
    last_rx_more_data_ = (frame.flags & FrameFlags::MORE_DATA) != 0;

    if (frame.sequence == rx_expected_seq_) {
        // In-order frame, deliver data
        LOG_MODEM(DEBUG, "ARQ: DATA seq=%d accepted, delivering %zu bytes, flags=0x%02x",
                  frame.sequence, frame.payload.size(), frame.flags);

        stats_.frames_received++;

        // Deliver to application
        if (on_data_received_) {
            on_data_received_(frame.payload);
        }

        // Queue ACK
        pending_ack_ = Frame::makeAck(local_call_, frame.src_call, frame.sequence);
        stats_.acks_sent++;

        // Advance expected sequence
        rx_expected_seq_ = (rx_expected_seq_ + 1) & 0xFF;

    } else if (frame.sequence == ((rx_expected_seq_ - 1) & 0xFF)) {
        // Duplicate (already received), re-ACK
        LOG_MODEM(DEBUG, "ARQ: Duplicate DATA seq=%d, re-ACKing", frame.sequence);
        pending_ack_ = Frame::makeAck(local_call_, frame.src_call, frame.sequence);
        stats_.acks_sent++;

    } else {
        // Out of order or too old, NAK
        LOG_MODEM(WARN, "ARQ: Out-of-order DATA seq=%d (expected %d), NAKing",
                  frame.sequence, rx_expected_seq_);
        pending_ack_ = Frame::makeNak(local_call_, frame.src_call, rx_expected_seq_);
    }

    // Transmit ACK/NAK immediately
    if (pending_ack_ && on_transmit_) {
        on_transmit_(*pending_ack_);
        pending_ack_.reset();
    }
}

void StopAndWaitARQ::handleAckFrame(const Frame& frame) {
    if (state_ != State::WAIT_ACK) {
        LOG_MODEM(DEBUG, "ARQ: Ignoring ACK, not waiting for one");
        return;
    }

    if (frame.sequence == tx_seq_) {
        // ACK for our pending frame
        LOG_MODEM(DEBUG, "ARQ: ACK received for seq=%d", tx_seq_);
        stats_.acks_received++;

        // Advance TX sequence
        tx_seq_ = (tx_seq_ + 1) & 0xFF;

        // Return to idle BEFORE callback so isReadyToSend() is true
        state_ = State::IDLE;

        // Notify success (callback may want to send next chunk)
        if (on_send_complete_) {
            on_send_complete_(true);
        }

    } else {
        LOG_MODEM(WARN, "ARQ: ACK seq=%d doesn't match pending seq=%d",
                  frame.sequence, tx_seq_);
    }
}

void StopAndWaitARQ::handleNakFrame(const Frame& frame) {
    if (state_ != State::WAIT_ACK) {
        LOG_MODEM(DEBUG, "ARQ: Ignoring NAK, not waiting for ACK");
        return;
    }

    LOG_MODEM(DEBUG, "ARQ: NAK received for seq=%d, retransmitting", frame.sequence);

    // Treat NAK as immediate timeout - retransmit
    retransmit();
}

void StopAndWaitARQ::tick(uint32_t elapsed_ms) {
    if (state_ == State::WAIT_ACK) {
        if (elapsed_ms >= timeout_remaining_ms_) {
            // Timeout!
            LOG_MODEM(WARN, "ARQ: Timeout waiting for ACK seq=%d", tx_seq_);
            stats_.timeouts++;
            retransmit();
        } else {
            timeout_remaining_ms_ -= elapsed_ms;
        }
    }
}

void StopAndWaitARQ::retransmit() {
    retry_count_++;

    if (retry_count_ >= config_.max_retries) {
        sendFailed();
        return;
    }

    LOG_MODEM(DEBUG, "ARQ: Retransmitting seq=%d (attempt %d/%d)",
              tx_seq_, retry_count_ + 1, config_.max_retries);

    stats_.retransmissions++;
    transmitFrame(pending_frame_);
    timeout_remaining_ms_ = config_.ack_timeout_ms;
}

void StopAndWaitARQ::sendFailed() {
    LOG_MODEM(ERROR, "ARQ: Send failed after %d retries for seq=%d",
              config_.max_retries, tx_seq_);

    stats_.failed++;

    // Notify failure
    if (on_send_complete_) {
        on_send_complete_(false);
    }

    // Return to idle (drop the frame)
    state_ = State::IDLE;
    tx_seq_ = (tx_seq_ + 1) & 0xFF;  // Still advance seq to avoid confusion
}

void StopAndWaitARQ::transmitFrame(const Frame& frame) {
    if (on_transmit_) {
        on_transmit_(frame);
    }
}

void StopAndWaitARQ::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void StopAndWaitARQ::setDataReceivedCallback(DataReceivedCallback cb) {
    on_data_received_ = std::move(cb);
}

void StopAndWaitARQ::setSendCompleteCallback(SendCompleteCallback cb) {
    on_send_complete_ = std::move(cb);
}

void StopAndWaitARQ::reset() {
    state_ = State::IDLE;
    tx_seq_ = 0;
    rx_expected_seq_ = 0;
    retry_count_ = 0;
    timeout_remaining_ms_ = 0;
    pending_ack_.reset();
    LOG_MODEM(DEBUG, "ARQ: Reset");
}

} // namespace protocol
} // namespace ultra
