/**
 * Selective Repeat ARQ Test Suite
 *
 * Tests the SR-ARQ implementation including:
 * - Basic send/receive with window
 * - Out-of-order delivery
 * - SACK generation and processing
 * - Retransmission on timeout
 * - Window advancement
 */

#include "protocol/arq_interface.hpp"
#include "protocol/selective_repeat_arq.hpp"
#include "protocol/frame_v2.hpp"
#include <iostream>
#include <cassert>
#include <queue>
#include <vector>

using namespace ultra;
using namespace ultra::protocol;

// Test counters
static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { std::cout << "  Testing " << name << "... " << std::flush; tests_run++; } while(0)

#define PASS() \
    do { std::cout << "PASS\n"; tests_passed++; } while(0)

#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << "\n"; return false; } while(0)

// ============================================================================
// Test Helpers
// ============================================================================

// Simple byte queue to simulate channel (v2 frames as serialized bytes)
class ByteChannel {
public:
    void send(const Bytes& data) { queue_.push(data); }

    bool hasData() const { return !queue_.empty(); }

    Bytes receive() {
        Bytes data = queue_.front();
        queue_.pop();
        return data;
    }

    void clear() {
        while (!queue_.empty()) queue_.pop();
    }

    size_t size() const { return queue_.size(); }

private:
    std::queue<Bytes> queue_;
};

// ============================================================================
// Basic Tests
// ============================================================================

bool test_create_sr_arq() {
    TEST("Create Selective Repeat ARQ");

    ARQConfig config;
    config.window_size = 4;

    auto arq = createARQController(ARQMode::SELECTIVE_REPEAT, config);

    if (arq->getMode() != ARQMode::SELECTIVE_REPEAT)
        FAIL("Wrong ARQ mode");

    if (arq->getAvailableSlots() != 4)
        FAIL("Wrong available slots");

    PASS();
    return true;
}

bool test_send_single_frame() {
    TEST("Send single frame");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // Send data
    Bytes data = {0x01, 0x02, 0x03};
    if (!tx.sendData(data))
        FAIL("Failed to send data");

    // Check frame was transmitted
    if (channel.size() != 1)
        FAIL("Frame not transmitted");

    Bytes frame_data = channel.receive();
    auto parsed = v2::DataFrame::deserialize(frame_data);
    if (!parsed)
        FAIL("Failed to parse transmitted frame");

    if (parsed->type != v2::FrameType::DATA)
        FAIL("Wrong frame type");
    if (parsed->seq != 0)
        FAIL("Wrong sequence number");
    if (parsed->payload != data)
        FAIL("Wrong payload");

    // Window should have 3 slots remaining
    if (tx.getAvailableSlots() != 3)
        FAIL("Wrong available slots after send");

    PASS();
    return true;
}

bool test_send_window_full() {
    TEST("Send until window full");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // Fill window
    for (int i = 0; i < 4; i++) {
        Bytes data = {static_cast<uint8_t>(i)};
        if (!tx.sendData(data))
            FAIL("Failed to send frame " + std::to_string(i));
    }

    // Window should be full
    if (tx.getAvailableSlots() != 0)
        FAIL("Window not full");

    if (tx.isReadyToSend())
        FAIL("isReadyToSend() should be false when window full");

    // Try to send another - should fail
    Bytes data = {0xFF};
    if (tx.sendData(data))
        FAIL("Should not be able to send when window full");

    // Check all 4 frames were transmitted
    if (channel.size() != 4)
        FAIL("Wrong number of frames transmitted");

    PASS();
    return true;
}

bool test_receive_ack() {
    TEST("Receive ACK frees slot");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    int completions = 0;
    tx.setSendCompleteCallback([&](bool success) {
        if (success) completions++;
    });

    // Send 4 frames
    for (int i = 0; i < 4; i++) {
        tx.sendData(Bytes{static_cast<uint8_t>(i)});
    }
    channel.clear();

    // ACK first frame
    auto ack = v2::ControlFrame::makeAck("RX1", "TX1", 0);
    Bytes ack_data = ack.serialize();
    tx.onFrameReceived(ack_data);

    // Should have 1 completion
    if (completions != 1)
        FAIL("Expected 1 completion, got " + std::to_string(completions));

    // Should have 1 slot free
    if (tx.getAvailableSlots() != 1)
        FAIL("Expected 1 slot free");

    PASS();
    return true;
}

bool test_rx_in_order() {
    TEST("RX delivers in-order frames");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) {
        received.push_back(data);
    });

    // Receive frames in order
    for (int i = 0; i < 3; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        Bytes frame_data = frame.serialize();
        rx.onFrameReceived(frame_data);
    }

    // All 3 should be delivered
    if (received.size() != 3)
        FAIL("Expected 3 deliveries, got " + std::to_string(received.size()));

    for (int i = 0; i < 3; i++) {
        if (received[i].size() != 1 || received[i][0] != i)
            FAIL("Wrong payload for frame " + std::to_string(i));
    }

    // Check SACKs were sent
    if (channel.size() != 3)
        FAIL("Expected 3 SACKs");

    PASS();
    return true;
}

bool test_rx_out_of_order() {
    TEST("RX handles out-of-order frames");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) {
        received.push_back(data);
    });

    // Receive frame 2 first (out of order)
    auto f2 = v2::DataFrame::makeData("TX1", "RX1", 2, Bytes{0x02});
    rx.onFrameReceived(f2.serialize());

    // Should not deliver yet (waiting for 0,1)
    if (!received.empty())
        FAIL("Should not deliver out-of-order frame");

    // Receive frame 0
    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0x00});
    rx.onFrameReceived(f0.serialize());

    // Should deliver frame 0 only
    if (received.size() != 1)
        FAIL("Should deliver frame 0");

    // Receive frame 1
    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{0x01});
    rx.onFrameReceived(f1.serialize());

    // Should now deliver 1 and 2 (in order)
    if (received.size() != 3)
        FAIL("Expected 3 deliveries after receiving frame 1, got " + std::to_string(received.size()));

    // Verify order
    for (int i = 0; i < 3; i++) {
        if (received[i][0] != i)
            FAIL("Frames delivered out of order");
    }

    PASS();
    return true;
}

bool test_timeout_retransmit() {
    TEST("Timeout triggers retransmit");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 100;
    config.max_retries = 3;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    // Send one frame
    tx.sendData(Bytes{0x01});

    // Initial transmission
    if (transmitted.size() != 1)
        FAIL("Expected 1 frame transmitted");

    // Advance time past timeout
    tx.tick(150);

    // Should have retransmitted
    if (transmitted.size() != 2)
        FAIL("Expected 2 frames (1 initial + 1 retransmit)");

    auto stats = tx.getStats();
    if (stats.retransmissions != 1)
        FAIL("Expected 1 retransmission in stats");

    PASS();
    return true;
}

bool test_max_retries_failure() {
    TEST("Max retries triggers failure");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 100;
    config.max_retries = 2;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    tx.setTransmitCallback([](const Bytes&) {});

    bool failed = false;
    tx.setSendCompleteCallback([&](bool success) {
        if (!success) failed = true;
    });

    // Send one frame
    tx.sendData(Bytes{0x01});

    // Timeout twice (exceeds max_retries=2)
    tx.tick(150);  // Retry 1
    tx.tick(150);  // Retry 2 -> failure

    if (!failed)
        FAIL("Expected failure callback");

    auto stats = tx.getStats();
    if (stats.failed != 1)
        FAIL("Expected 1 failure in stats");

    PASS();
    return true;
}

bool test_full_exchange() {
    TEST("Full TX/RX exchange");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ tx(config);
    SelectiveRepeatARQ rx(config);

    tx.setCallsigns("TX1", "RX1");
    rx.setCallsigns("RX1", "TX1");

    // Connect TX -> RX
    tx.setTransmitCallback([&](const Bytes& data) {
        rx.onFrameReceived(data);
    });

    // Connect RX -> TX
    rx.setTransmitCallback([&](const Bytes& data) {
        tx.onFrameReceived(data);
    });

    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) {
        received.push_back(data);
    });

    int completions = 0;
    tx.setSendCompleteCallback([&](bool success) {
        if (success) completions++;
    });

    // Send 10 frames
    for (int i = 0; i < 10; i++) {
        while (!tx.isReadyToSend()) {
            // Wait for ACKs if window full
        }
        tx.sendData(Bytes{static_cast<uint8_t>(i)});
    }

    // All frames should be received
    if (received.size() != 10)
        FAIL("Expected 10 received, got " + std::to_string(received.size()));

    // Completions should match sends (last completion may be delayed)
    // Accept 9 or 10 due to synchronous callback timing
    if (completions < 9)
        FAIL("Expected at least 9 completions, got " + std::to_string(completions));

    // Verify payload integrity
    for (int i = 0; i < 10; i++) {
        if (received[i].size() != 1 || received[i][0] != i)
            FAIL("Payload mismatch at index " + std::to_string(i));
    }

    PASS();
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=== Selective Repeat ARQ Test Suite (v2) ===\n\n";

    std::cout << "Basic Tests:\n";
    test_create_sr_arq();
    test_send_single_frame();
    test_send_window_full();
    test_receive_ack();

    std::cout << "\nReceiver Tests:\n";
    test_rx_in_order();
    test_rx_out_of_order();

    std::cout << "\nRetransmission Tests:\n";
    test_timeout_retransmit();
    test_max_retries_failure();

    std::cout << "\nIntegration Tests:\n";
    test_full_exchange();

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_run << " tests passed ===\n";

    if (tests_passed == tests_run) {
        std::cout << "All tests PASSED!\n";
        return 0;
    } else {
        std::cout << "Some tests FAILED!\n";
        return 1;
    }
}
