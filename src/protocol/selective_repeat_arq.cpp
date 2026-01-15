#include "selective_repeat_arq.hpp"
#include "ultra/logging.hpp"

namespace ultra {
namespace protocol {

SelectiveRepeatARQ::SelectiveRepeatARQ(const ARQConfig& config)
    : config_(config)
{
    // Validate window size
    if (config_.window_size > MAX_WINDOW) {
        config_.window_size = MAX_WINDOW;
    }
    if (config_.window_size < 1) {
        config_.window_size = 1;
    }
}

void SelectiveRepeatARQ::setCallsigns(const std::string& local, const std::string& remote) {
    local_call_ = Frame::sanitizeCallsign(local);
    remote_call_ = Frame::sanitizeCallsign(remote);
}

bool SelectiveRepeatARQ::sendData(const Bytes& data) {
    return sendDataWithFlags(data, FrameFlags::NONE);
}

bool SelectiveRepeatARQ::sendData(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return sendData(data);
}

bool SelectiveRepeatARQ::sendDataWithFlags(const Bytes& data, uint8_t flags) {
    if (!isReadyToSend()) {
        LOG_MODEM(WARN, "SR-ARQ: Window full, cannot send");
        return false;
    }

    if (local_call_.empty() || remote_call_.empty()) {
        LOG_MODEM(ERROR, "SR-ARQ: Callsigns not set");
        return false;
    }

    // Find empty slot in window
    size_t slot = seqToSlot(tx_next_seq_);

    // Create and store frame
    Frame frame = Frame::makeData(local_call_, remote_call_, tx_next_seq_, data);
    frame.flags = flags;

    tx_window_[slot].active = true;
    tx_window_[slot].frame = frame;
    tx_window_[slot].timeout_ms = config_.ack_timeout_ms;
    tx_window_[slot].retry_count = 0;
    tx_window_[slot].acked = false;

    // Transmit
    transmitFrame(frame);

    LOG_MODEM(DEBUG, "SR-ARQ: Sent DATA seq=%d slot=%zu, window=[%d,%d)",
              tx_next_seq_, slot, tx_base_seq_, (tx_next_seq_ + 1) & 0xFF);

    stats_.frames_sent++;
    tx_next_seq_ = (tx_next_seq_ + 1) & 0xFF;
    tx_in_flight_++;

    return true;
}

bool SelectiveRepeatARQ::isReadyToSend() const {
    return getAvailableSlots() > 0;
}

size_t SelectiveRepeatARQ::getAvailableSlots() const {
    size_t window = config_.window_size;
    return (tx_in_flight_ < window) ? (window - tx_in_flight_) : 0;
}

void SelectiveRepeatARQ::onFrameReceived(const Frame& frame) {
    // Check if frame is for us
    if (!frame.isForCallsign(local_call_)) {
        LOG_MODEM(TRACE, "SR-ARQ: Ignoring frame for %s (we are %s)",
                  frame.dst_call.c_str(), local_call_.c_str());
        return;
    }

    LOG_MODEM(DEBUG, "SR-ARQ: Received %s from %s seq=%d",
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
        case FrameType::SACK:
            handleSackFrame(frame);
            break;
        default:
            break;
    }
}

void SelectiveRepeatARQ::handleDataFrame(const Frame& frame) {
    last_rx_flags_ = frame.flags;
    last_rx_more_data_ = (frame.flags & FrameFlags::MORE_DATA) != 0;

    uint8_t seq = frame.sequence;

    if (isInRXWindow(seq)) {
        size_t slot = seqToSlot(seq);

        if (!rx_window_[slot].received) {
            // New frame, store it
            rx_window_[slot].received = true;
            rx_window_[slot].frame = frame;
            stats_.frames_received++;

            LOG_MODEM(DEBUG, "SR-ARQ: DATA seq=%d stored in slot %zu", seq, slot);

            // If this is the expected frame, deliver in-order frames
            if (seq == rx_base_seq_) {
                advanceRXWindow();
            } else {
                stats_.out_of_order++;
                LOG_MODEM(DEBUG, "SR-ARQ: Out-of-order seq=%d (expected %d)",
                          seq, rx_base_seq_);
            }
        } else {
            LOG_MODEM(DEBUG, "SR-ARQ: Duplicate DATA seq=%d", seq);
        }

        // Send SACK
        sendSack();

    } else {
        // Outside window - might be very old duplicate
        LOG_MODEM(WARN, "SR-ARQ: DATA seq=%d outside window [%d, %d)",
                  seq, rx_base_seq_, (rx_base_seq_ + config_.window_size) & 0xFF);

        // Send SACK anyway to help TX know where we are
        sendSack();
    }
}

void SelectiveRepeatARQ::handleAckFrame(const Frame& frame) {
    // Simple ACK - treat as cumulative
    uint8_t seq = frame.sequence;

    LOG_MODEM(DEBUG, "SR-ARQ: ACK seq=%d (base=%d)", seq, tx_base_seq_);

    // Mark this and all previous as ACKed
    while (tx_in_flight_ > 0 && tx_base_seq_ != ((seq + 1) & 0xFF)) {
        size_t slot = seqToSlot(tx_base_seq_);
        if (tx_window_[slot].active) {
            tx_window_[slot].active = false;
            tx_window_[slot].acked = true;
            tx_in_flight_--;
            stats_.acks_received++;

            // Notify completion
            if (on_send_complete_) {
                on_send_complete_(true);
            }
        }
        tx_base_seq_ = (tx_base_seq_ + 1) & 0xFF;
    }
}

void SelectiveRepeatARQ::handleNakFrame(const Frame& frame) {
    uint8_t seq = frame.sequence;

    LOG_MODEM(DEBUG, "SR-ARQ: NAK seq=%d", seq);

    if (isInTXWindow(seq)) {
        size_t slot = seqToSlot(seq);
        if (tx_window_[slot].active && !tx_window_[slot].acked) {
            // Immediate retransmit
            retransmitFrame(slot);
        }
    }
}

void SelectiveRepeatARQ::handleSackFrame(const Frame& frame) {
    uint8_t base_seq = frame.sequence;
    uint8_t bitmap = frame.payload.empty() ? 0 : frame.payload[0];

    LOG_MODEM(DEBUG, "SR-ARQ: SACK base=%d bitmap=0x%02X", base_seq, bitmap);
    stats_.sacks_received++;

    // Cumulative ACK up to and including base_seq
    while (tx_in_flight_ > 0 && tx_base_seq_ != ((base_seq + 1) & 0xFF)) {
        size_t slot = seqToSlot(tx_base_seq_);
        if (tx_window_[slot].active) {
            tx_window_[slot].active = false;
            tx_in_flight_--;
            stats_.acks_received++;

            if (on_send_complete_) {
                on_send_complete_(true);
            }
        }
        tx_base_seq_ = (tx_base_seq_ + 1) & 0xFF;
    }

    // Process bitmap for selective ACKs
    // Bit i means (base_seq + 1 + i) was received
    for (int i = 0; i < 8 && i < static_cast<int>(config_.window_size); i++) {
        uint8_t seq = (base_seq + 1 + i) & 0xFF;
        if (isInTXWindow(seq)) {
            size_t slot = seqToSlot(seq);
            if (tx_window_[slot].active) {
                if (bitmap & (1 << i)) {
                    // Frame received, mark as ACKed (but don't advance window yet)
                    tx_window_[slot].acked = true;
                } else {
                    // Frame not received, retransmit if timeout isn't imminent
                    // (Let timeout handle it to avoid excessive retransmits)
                }
            }
        }
    }

    // Advance window if head frames are ACKed
    advanceTXWindow();
}

void SelectiveRepeatARQ::tick(uint32_t elapsed_ms) {
    // Process timeouts for each active slot
    for (size_t i = 0; i < config_.window_size; i++) {
        size_t slot = seqToSlot((tx_base_seq_ + i) & 0xFF);
        TXSlot& s = tx_window_[slot];

        if (s.active && !s.acked) {
            if (elapsed_ms >= s.timeout_ms) {
                // Timeout - retransmit
                stats_.timeouts++;
                retransmitFrame(slot);
            } else {
                s.timeout_ms -= elapsed_ms;
            }
        }
    }
}

void SelectiveRepeatARQ::retransmitFrame(size_t slot) {
    TXSlot& s = tx_window_[slot];

    s.retry_count++;
    if (s.retry_count >= config_.max_retries) {
        // Failed after max retries
        LOG_MODEM(ERROR, "SR-ARQ: Frame seq=%d failed after %d retries",
                  s.frame.sequence, config_.max_retries);
        stats_.failed++;

        s.active = false;
        tx_in_flight_--;

        if (on_send_complete_) {
            on_send_complete_(false);
        }

        // Advance window
        advanceTXWindow();
        return;
    }

    LOG_MODEM(DEBUG, "SR-ARQ: Retransmitting seq=%d (attempt %d/%d)",
              s.frame.sequence, s.retry_count + 1, config_.max_retries);

    stats_.retransmissions++;
    s.timeout_ms = config_.ack_timeout_ms;
    transmitFrame(s.frame);
}

void SelectiveRepeatARQ::advanceTXWindow() {
    // Advance base while head slots are complete (ACKed or failed)
    while (tx_in_flight_ > 0) {
        size_t slot = seqToSlot(tx_base_seq_);
        if (tx_window_[slot].active && !tx_window_[slot].acked) {
            break;  // Still waiting for this frame
        }
        if (tx_window_[slot].active) {
            // Frame was ACKed, clear it
            tx_window_[slot].active = false;
            tx_in_flight_--;

            if (on_send_complete_) {
                on_send_complete_(true);
            }
        }
        tx_base_seq_ = (tx_base_seq_ + 1) & 0xFF;
    }
}

void SelectiveRepeatARQ::advanceRXWindow() {
    // Deliver consecutive in-order frames
    while (true) {
        size_t slot = seqToSlot(rx_base_seq_);
        if (!rx_window_[slot].received) {
            break;  // Gap in sequence
        }

        // Deliver frame
        LOG_MODEM(DEBUG, "SR-ARQ: Delivering seq=%d", rx_base_seq_);

        if (on_data_received_) {
            on_data_received_(rx_window_[slot].frame.payload);
        }

        // Clear slot
        rx_window_[slot].received = false;
        rx_base_seq_ = (rx_base_seq_ + 1) & 0xFF;
    }
}

void SelectiveRepeatARQ::sendSack() {
    uint8_t bitmap = buildRXBitmap();

    Frame sack = Frame::makeSack(local_call_, remote_call_,
                                 (rx_base_seq_ - 1) & 0xFF, bitmap);

    stats_.sacks_sent++;
    stats_.acks_sent++;
    transmitFrame(sack);

    LOG_MODEM(DEBUG, "SR-ARQ: Sent SACK base=%d bitmap=0x%02X",
              (rx_base_seq_ - 1) & 0xFF, bitmap);
}

uint8_t SelectiveRepeatARQ::buildRXBitmap() const {
    uint8_t bitmap = 0;

    // Bit i = (rx_base_seq_ + i) received
    for (int i = 0; i < 8 && i < static_cast<int>(config_.window_size); i++) {
        size_t slot = seqToSlot((rx_base_seq_ + i) & 0xFF);
        if (rx_window_[slot].received) {
            bitmap |= (1 << i);
        }
    }

    return bitmap;
}

size_t SelectiveRepeatARQ::seqToSlot(uint8_t seq) const {
    return seq % MAX_WINDOW;
}

bool SelectiveRepeatARQ::isInTXWindow(uint8_t seq) const {
    // Check if seq is in [tx_base_seq_, tx_next_seq_)
    uint8_t diff = (seq - tx_base_seq_) & 0xFF;
    return diff < config_.window_size;
}

bool SelectiveRepeatARQ::isInRXWindow(uint8_t seq) const {
    // Check if seq is in [rx_base_seq_, rx_base_seq_ + window_size)
    uint8_t diff = (seq - rx_base_seq_) & 0xFF;
    return diff < config_.window_size;
}

void SelectiveRepeatARQ::transmitFrame(const Frame& frame) {
    if (on_transmit_) {
        on_transmit_(frame);
    }
}

void SelectiveRepeatARQ::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void SelectiveRepeatARQ::setDataReceivedCallback(DataReceivedCallback cb) {
    on_data_received_ = std::move(cb);
}

void SelectiveRepeatARQ::setSendCompleteCallback(SendCompleteCallback cb) {
    on_send_complete_ = std::move(cb);
}

void SelectiveRepeatARQ::reset() {
    // Clear TX window
    for (auto& slot : tx_window_) {
        slot.active = false;
        slot.acked = false;
    }
    tx_base_seq_ = 0;
    tx_next_seq_ = 0;
    tx_in_flight_ = 0;

    // Clear RX window
    for (auto& slot : rx_window_) {
        slot.received = false;
    }
    rx_base_seq_ = 0;

    last_rx_more_data_ = false;
    last_rx_flags_ = 0;

    LOG_MODEM(DEBUG, "SR-ARQ: Reset");
}

} // namespace protocol
} // namespace ultra
