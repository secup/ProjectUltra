#include "ultra/arq.hpp"
#include <queue>
#include <map>
#include <mutex>

namespace ultra {

struct ARQController::Impl {
    ModemConfig config;

    // Callbacks
    SendFrameCallback send_callback;
    DeliveryCallback delivery_callback;

    // TX state
    uint16_t tx_seq = 0;
    std::map<uint16_t, Bytes> tx_buffer;  // Unacked frames
    std::map<uint16_t, std::chrono::steady_clock::time_point> tx_times;
    std::map<uint16_t, int> retry_counts;

    // RX state
    uint16_t rx_expected = 0;
    std::map<uint16_t, Bytes> rx_buffer;  // Out-of-order frames
    uint16_t last_acked = 0;

    // Frame builder/parser
    FrameBuilder builder;
    FrameParser parser;

    // Channel quality from remote
    ChannelQuality remote_quality;

    // Mutex for thread safety
    std::mutex mtx;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , builder(cfg)
        , parser(cfg)
    {}

    void sendFrame(const Bytes& frame) {
        if (send_callback) {
            send_callback(frame);
        }
    }

    void deliverData(const Bytes& data) {
        if (delivery_callback) {
            delivery_callback(data);
        }
    }

    void tryDeliverInOrder() {
        // Deliver any frames that are now in order
        while (rx_buffer.count(rx_expected)) {
            auto it = rx_buffer.find(rx_expected);
            deliverData(it->second);
            rx_buffer.erase(it);
            ++rx_expected;
        }
    }
};

ARQController::ARQController(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

ARQController::~ARQController() = default;

void ARQController::sendData(ByteSpan data) {
    std::lock_guard<std::mutex> lock(impl_->mtx);

    size_t max_payload = impl_->builder.maxPayloadSize();
    size_t offset = 0;

    while (offset < data.size()) {
        size_t chunk_size = std::min(max_payload, data.size() - offset);
        ByteSpan chunk(data.data() + offset, chunk_size);

        uint16_t seq = impl_->tx_seq++;
        Bytes frame = impl_->builder.buildDataFrame(seq, chunk);

        // Store for potential retransmission
        impl_->tx_buffer[seq] = frame;
        impl_->retry_counts[seq] = 0;

        // Send
        impl_->sendFrame(frame);

        offset += chunk_size;
    }
}

void ARQController::onFrameSent(uint16_t seq_num) {
    std::lock_guard<std::mutex> lock(impl_->mtx);
    impl_->tx_times[seq_num] = std::chrono::steady_clock::now();
}

void ARQController::onAckReceived(uint16_t ack_seq, bool is_nack,
                                   const ChannelQuality& remote_quality) {
    std::lock_guard<std::mutex> lock(impl_->mtx);

    impl_->remote_quality = remote_quality;

    if (is_nack) {
        // Retransmit the requested frame
        auto it = impl_->tx_buffer.find(ack_seq);
        if (it != impl_->tx_buffer.end()) {
            if (impl_->retry_counts[ack_seq] < static_cast<int>(impl_->config.max_retries)) {
                impl_->sendFrame(it->second);
                impl_->retry_counts[ack_seq]++;
                impl_->tx_times[ack_seq] = std::chrono::steady_clock::now();
            }
        }
    } else {
        // ACK: remove all frames up to and including ack_seq
        for (auto it = impl_->tx_buffer.begin(); it != impl_->tx_buffer.end();) {
            // Handle sequence number wraparound
            int16_t diff = static_cast<int16_t>(it->first - ack_seq);
            if (diff <= 0) {
                impl_->tx_times.erase(it->first);
                impl_->retry_counts.erase(it->first);
                it = impl_->tx_buffer.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void ARQController::setSendCallback(SendFrameCallback cb) {
    std::lock_guard<std::mutex> lock(impl_->mtx);
    impl_->send_callback = std::move(cb);
}

void ARQController::onDataReceived(uint16_t seq_num, ByteSpan data) {
    std::lock_guard<std::mutex> lock(impl_->mtx);

    int16_t diff = static_cast<int16_t>(seq_num - impl_->rx_expected);

    if (diff < 0) {
        // Already received, ignore (but still ACK)
        return;
    } else if (diff == 0) {
        // In order, deliver immediately
        impl_->deliverData(Bytes(data.begin(), data.end()));
        impl_->rx_expected++;
        impl_->last_acked = seq_num;

        // Try to deliver any buffered frames
        impl_->tryDeliverInOrder();
    } else {
        // Out of order, buffer
        impl_->rx_buffer[seq_num] = Bytes(data.begin(), data.end());

        // Could send NACK for missing frames here
    }
}

void ARQController::setDeliveryCallback(DeliveryCallback cb) {
    std::lock_guard<std::mutex> lock(impl_->mtx);
    impl_->delivery_callback = std::move(cb);
}

Bytes ARQController::generateAck() {
    std::lock_guard<std::mutex> lock(impl_->mtx);
    // ACK the last successfully received in-order frame
    ChannelQuality local_quality;  // Would get from demodulator
    return impl_->builder.buildAckFrame(impl_->last_acked, local_quality);
}

Bytes ARQController::generateNack(uint16_t missing_seq) {
    return impl_->builder.buildNackFrame(missing_seq);
}

void ARQController::tick(std::chrono::steady_clock::time_point now) {
    std::lock_guard<std::mutex> lock(impl_->mtx);

    auto timeout = std::chrono::milliseconds(impl_->config.arq_timeout_ms);

    for (auto& [seq, time] : impl_->tx_times) {
        if (now - time > timeout) {
            // Timeout - retransmit
            auto it = impl_->tx_buffer.find(seq);
            if (it != impl_->tx_buffer.end()) {
                if (impl_->retry_counts[seq] < static_cast<int>(impl_->config.max_retries)) {
                    impl_->sendFrame(it->second);
                    impl_->retry_counts[seq]++;
                    time = now;
                }
            }
        }
    }
}

size_t ARQController::framesInFlight() const {
    return impl_->tx_buffer.size();
}

size_t ARQController::framesPendingRetry() const {
    size_t count = 0;
    for (const auto& [seq, retries] : impl_->retry_counts) {
        if (retries > 0) ++count;
    }
    return count;
}

void ARQController::reset() {
    std::lock_guard<std::mutex> lock(impl_->mtx);
    impl_->tx_seq = 0;
    impl_->tx_buffer.clear();
    impl_->tx_times.clear();
    impl_->retry_counts.clear();
    impl_->rx_expected = 0;
    impl_->rx_buffer.clear();
    impl_->last_acked = 0;
}

} // namespace ultra
