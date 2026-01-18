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

    // Create v2 DATA frame
    auto frame = v2::DataFrame::makeData(local_call_, remote_call_, tx_seq_, data);
    pending_frame_data_ = frame.serialize();
    retry_count_ = 0;

    // Transmit
    transmitData(pending_frame_data_);
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

    // Create v2 DATA frame with custom flags
    auto frame = v2::DataFrame::makeData(local_call_, remote_call_, tx_seq_, data);
    frame.flags = flags;
    pending_frame_data_ = frame.serialize();
    retry_count_ = 0;

    // Transmit
    transmitData(pending_frame_data_);
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

void StopAndWaitARQ::onFrameReceived(const Bytes& frame_data) {
    if (frame_data.size() < 2) {
        return;
    }

    // Check magic
    uint16_t magic = (static_cast<uint16_t>(frame_data[0]) << 8) | frame_data[1];
    if (magic != v2::MAGIC_V2) {
        LOG_MODEM(TRACE, "ARQ: Ignoring frame with wrong magic");
        return;
    }

    // Check if this is our frame by parsing header
    auto header = v2::parseHeader(frame_data);
    if (!header.valid) {
        LOG_MODEM(TRACE, "ARQ: Ignoring frame with invalid header");
        return;
    }

    // Check if frame is for us
    uint32_t our_hash = v2::hashCallsign(local_call_);
    if (header.dst_hash != our_hash && header.dst_hash != 0xFFFFFF) {
        LOG_MODEM(TRACE, "ARQ: Ignoring frame for different station");
        return;
    }

    LOG_MODEM(DEBUG, "ARQ: Received %s seq=%d",
              v2::frameTypeToString(header.type), header.seq);

    // Handle based on frame type
    if (header.is_control) {
        auto ctrl = v2::ControlFrame::deserialize(frame_data);
        if (ctrl) {
            switch (ctrl->type) {
                case v2::FrameType::ACK:
                    handleAckFrame(*ctrl);
                    break;
                case v2::FrameType::NACK:
                    handleNackFrame(*ctrl);
                    break;
                default:
                    // Other control frames handled by Connection layer
                    break;
            }
        }
    } else {
        auto data_frame = v2::DataFrame::deserialize(frame_data);
        if (data_frame) {
            handleDataFrame(*data_frame);
        }
    }
}

void StopAndWaitARQ::handleDataFrame(const v2::DataFrame& frame) {
    // Track flags from this frame
    last_rx_flags_ = frame.flags;
    last_rx_more_data_ = (frame.flags & v2::Flags::MORE_FRAG) != 0;

    if (frame.seq == rx_expected_seq_) {
        // In-order frame, deliver data
        LOG_MODEM(DEBUG, "ARQ: DATA seq=%d accepted, delivering %zu bytes, flags=0x%02x",
                  frame.seq, frame.payload.size(), frame.flags);

        stats_.frames_received++;

        // Deliver to application
        if (on_data_received_) {
            on_data_received_(frame.payload);
        }

        // Queue ACK
        auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
        pending_ack_ = ack.serialize();
        stats_.acks_sent++;

        // Advance expected sequence
        rx_expected_seq_ = (rx_expected_seq_ + 1) & 0xFFFF;

    } else if (frame.seq == ((rx_expected_seq_ - 1) & 0xFFFF)) {
        // Duplicate (already received), re-ACK
        LOG_MODEM(DEBUG, "ARQ: Duplicate DATA seq=%d, re-ACKing", frame.seq);
        auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
        pending_ack_ = ack.serialize();
        stats_.acks_sent++;

    } else {
        // Out of order or too old, NACK
        LOG_MODEM(WARN, "ARQ: Out-of-order DATA seq=%d (expected %d), NACKing",
                  frame.seq, rx_expected_seq_);
        auto nack = v2::ControlFrame::makeNack(local_call_, remote_call_, rx_expected_seq_, 0);
        pending_ack_ = nack.serialize();
    }

    // Transmit ACK/NACK immediately
    if (pending_ack_ && on_transmit_) {
        on_transmit_(*pending_ack_);
        pending_ack_.reset();
    }
}

void StopAndWaitARQ::handleAckFrame(const v2::ControlFrame& frame) {
    if (state_ != State::WAIT_ACK) {
        LOG_MODEM(DEBUG, "ARQ: Ignoring ACK, not waiting for one");
        return;
    }

    if (frame.seq == tx_seq_) {
        // ACK for our pending frame
        LOG_MODEM(DEBUG, "ARQ: ACK received for seq=%d", tx_seq_);
        stats_.acks_received++;

        // Advance TX sequence
        tx_seq_ = (tx_seq_ + 1) & 0xFFFF;

        // Return to idle BEFORE callback so isReadyToSend() is true
        state_ = State::IDLE;

        // Notify success (callback may want to send next chunk)
        if (on_send_complete_) {
            on_send_complete_(true);
        }

    } else {
        LOG_MODEM(WARN, "ARQ: ACK seq=%d doesn't match pending seq=%d",
                  frame.seq, tx_seq_);
    }
}

void StopAndWaitARQ::handleNackFrame(const v2::ControlFrame& frame) {
    if (state_ != State::WAIT_ACK) {
        LOG_MODEM(DEBUG, "ARQ: Ignoring NACK, not waiting for ACK");
        return;
    }

    LOG_MODEM(DEBUG, "ARQ: NACK received for seq=%d, retransmitting", frame.seq);

    // Treat NACK as immediate timeout - retransmit
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
    transmitData(pending_frame_data_);
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
    tx_seq_ = (tx_seq_ + 1) & 0xFFFF;  // Still advance seq to avoid confusion
}

void StopAndWaitARQ::transmitData(const Bytes& data) {
    if (on_transmit_) {
        on_transmit_(data);
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
    pending_frame_data_.clear();
    pending_ack_.reset();
    LOG_MODEM(DEBUG, "ARQ: Reset");
}

} // namespace protocol
} // namespace ultra
