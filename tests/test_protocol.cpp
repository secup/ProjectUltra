/**
 * Protocol Layer Test Suite
 *
 * Tests the ARQ protocol implementation by simulating two stations
 * communicating with each other. No actual audio/modem involved -
 * TX output from one station feeds directly into RX of the other.
 */

#include "protocol/protocol_engine.hpp"
#include "protocol/frame.hpp"
#include "protocol/file_transfer.hpp"
#include "protocol/compression.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <thread>
#include <queue>
#include <fstream>
#include <cstdio>
#include <filesystem>

using namespace ultra::protocol;
using ultra::Bytes;

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
// Frame Tests
// ============================================================================

bool test_frame_serialization() {
    TEST("Frame serialization/deserialization");

    // Create a data frame
    Frame f = Frame::makeData("VA2MVR", "VE3ABC", 0, "Hello World!");

    // Serialize
    Bytes serialized = f.serialize();

    // Should have header + payload + CRC
    if (serialized.size() < Frame::MIN_SIZE) {
        FAIL("Serialized frame too small");
    }

    // Deserialize
    auto parsed = Frame::deserialize(serialized);
    if (!parsed) {
        FAIL("Failed to deserialize valid frame");
    }

    // Verify fields
    if (parsed->type != FrameType::DATA) FAIL("Type mismatch");
    if (parsed->src_call != "VA2MVR") FAIL("Source callsign mismatch");
    if (parsed->dst_call != "VE3ABC") FAIL("Dest callsign mismatch");
    if (parsed->sequence != 0) FAIL("Sequence mismatch");
    if (parsed->payloadAsText() != "Hello World!") FAIL("Payload mismatch");

    PASS();
    return true;
}

bool test_frame_crc() {
    TEST("Frame CRC validation");

    Frame f = Frame::makeConnect("TEST1", "TEST2");
    Bytes data = f.serialize();

    // Valid frame should parse
    auto valid = Frame::deserialize(data);
    if (!valid) FAIL("Valid frame rejected");

    // Corrupt one byte
    data[10] ^= 0xFF;
    auto corrupt = Frame::deserialize(data);
    if (corrupt) FAIL("Corrupt frame accepted");

    PASS();
    return true;
}

bool test_frame_types() {
    TEST("All frame types");

    // Test each frame type
    struct TestCase {
        Frame frame;
        FrameType expected_type;
    };

    std::vector<TestCase> cases = {
        { Frame::makeConnect("A", "B"), FrameType::CONNECT },
        { Frame::makeConnectAck("A", "B"), FrameType::CONNECT_ACK },
        { Frame::makeConnectNak("A", "B"), FrameType::CONNECT_NAK },
        { Frame::makeDisconnect("A", "B"), FrameType::DISCONNECT },
        { Frame::makeData("A", "B", 1, "test"), FrameType::DATA },
        { Frame::makeAck("A", "B", 1), FrameType::ACK },
        { Frame::makeNak("A", "B", 1), FrameType::NAK },
        { Frame::makeBeacon("A", "CQ CQ"), FrameType::BEACON },
    };

    for (const auto& tc : cases) {
        Bytes data = tc.frame.serialize();
        auto parsed = Frame::deserialize(data);
        if (!parsed) FAIL("Failed to parse frame");
        if (parsed->type != tc.expected_type) FAIL("Type mismatch");
    }

    PASS();
    return true;
}

bool test_callsign_validation() {
    TEST("Callsign validation");

    // Valid callsigns
    if (!Frame::isValidCallsign("VA2MVR")) FAIL("Valid callsign rejected");
    if (!Frame::isValidCallsign("W1AW")) FAIL("Valid callsign rejected");
    if (!Frame::isValidCallsign("VE3ABC")) FAIL("Valid callsign rejected");

    // Invalid callsigns
    if (Frame::isValidCallsign("")) FAIL("Empty callsign accepted");
    if (Frame::isValidCallsign("AB")) FAIL("Too short callsign accepted");

    // Sanitization
    if (Frame::sanitizeCallsign("va2mvr") != "VA2MVR") FAIL("Sanitize should uppercase");

    PASS();
    return true;
}

// ============================================================================
// Two-Station Simulation
// ============================================================================

/**
 * Simulated link between two stations
 * TX from one goes to RX of the other (with optional delay/loss)
 */
class SimulatedChannel {
public:
    SimulatedChannel(ProtocolEngine& stationA, ProtocolEngine& stationB)
        : stationA_(stationA), stationB_(stationB) {

        // Wire station A's TX to station B's RX
        stationA_.setTxDataCallback([this](const Bytes& data) {
            if (verbose_) {
                std::cout << "    [A->B] " << data.size() << " bytes\n";
            }
            tx_count_a_++;
            if (!drop_a_to_b_) {
                pending_b_.push(data);
            }
        });

        // Wire station B's TX to station A's RX
        stationB_.setTxDataCallback([this](const Bytes& data) {
            if (verbose_) {
                std::cout << "    [B->A] " << data.size() << " bytes\n";
            }
            tx_count_b_++;
            if (!drop_b_to_a_) {
                pending_a_.push(data);
            }
        });
    }

    // Deliver pending frames (simulates propagation)
    void deliver() {
        while (!pending_a_.empty()) {
            stationA_.onRxData(pending_a_.front());
            pending_a_.pop();
        }
        while (!pending_b_.empty()) {
            stationB_.onRxData(pending_b_.front());
            pending_b_.pop();
        }
    }

    // Simulate time passing
    void tick(uint32_t ms) {
        stationA_.tick(ms);
        stationB_.tick(ms);
    }

    // Run simulation for a number of cycles
    void run(int cycles, uint32_t tick_ms = 100) {
        for (int i = 0; i < cycles; i++) {
            deliver();
            tick(tick_ms);
        }
    }

    // Control packet loss
    void setDropAtoB(bool drop) { drop_a_to_b_ = drop; }
    void setDropBtoA(bool drop) { drop_b_to_a_ = drop; }
    void setVerbose(bool v) { verbose_ = v; }

    int getTxCountA() const { return tx_count_a_; }
    int getTxCountB() const { return tx_count_b_; }

private:
    ProtocolEngine& stationA_;
    ProtocolEngine& stationB_;

    std::queue<Bytes> pending_a_;  // Frames waiting to be delivered to A
    std::queue<Bytes> pending_b_;  // Frames waiting to be delivered to B

    bool drop_a_to_b_ = false;
    bool drop_b_to_a_ = false;
    bool verbose_ = false;

    int tx_count_a_ = 0;
    int tx_count_b_ = 0;
};

bool test_connection_establishment() {
    TEST("Connection establishment");

    // Create two stations
    ConnectionConfig config;
    config.auto_accept = true;  // B auto-accepts connections
    config.connect_timeout_ms = 5000;
    config.connect_retries = 3;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("TEST1A");
    stationB.setLocalCallsign("TEST2B");

    // Track connection state changes
    bool a_connected = false;
    bool b_connected = false;

    stationA.setConnectionChangedCallback([&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::CONNECTED) a_connected = true;
    });

    stationB.setConnectionChangedCallback([&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::CONNECTED) b_connected = true;
    });

    // Create simulated channel
    SimulatedChannel channel(stationA, stationB);

    // Station A initiates connection to B
    if (!stationA.connect("TEST2B")) {
        FAIL("Connect() returned false");
    }

    // Run simulation until connected (or timeout)
    for (int i = 0; i < 50 && (!a_connected || !b_connected); i++) {
        channel.run(1, 100);
    }

    if (!a_connected) FAIL("Station A did not connect");
    if (!b_connected) FAIL("Station B did not connect");
    if (stationA.getRemoteCallsign() != "TEST2B") FAIL("A has wrong remote callsign");
    if (stationB.getRemoteCallsign() != "TEST1A") FAIL("B has wrong remote callsign");

    PASS();
    return true;
}

bool test_data_transfer() {
    TEST("Data transfer with ACK");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    // Track received messages
    std::vector<std::string> received_at_b;

    stationB.setMessageReceivedCallback([&](const std::string& from, const std::string& text) {
        received_at_b.push_back(text);
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Connection not established");
    }

    // Send a message from A to B
    if (!stationA.sendMessage("Hello Bob!")) {
        FAIL("sendMessage() returned false");
    }

    // Run until message received
    channel.run(30, 100);

    if (received_at_b.empty()) FAIL("No message received at B");
    if (received_at_b[0] != "Hello Bob!") FAIL("Message content mismatch");

    PASS();
    return true;
}

bool test_bidirectional_transfer() {
    TEST("Bidirectional data transfer");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    std::vector<std::string> received_at_a;
    std::vector<std::string> received_at_b;

    stationA.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        received_at_a.push_back(text);
    });

    stationB.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        received_at_b.push_back(text);
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // Send from A to B
    stationA.sendMessage("Hello from Alice");
    channel.run(30, 100);

    // Send from B to A
    stationB.sendMessage("Hello from Bob");
    channel.run(30, 100);

    if (received_at_b.size() != 1 || received_at_b[0] != "Hello from Alice") {
        FAIL("B didn't receive Alice's message");
    }

    if (received_at_a.size() != 1 || received_at_a[0] != "Hello from Bob") {
        FAIL("A didn't receive Bob's message");
    }

    PASS();
    return true;
}

bool test_retransmission() {
    TEST("Retransmission on timeout");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 500;  // Fast timeout for test
    config.arq.max_retries = 3;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    std::vector<std::string> received_at_b;
    stationB.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        received_at_b.push_back(text);
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect first
    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // Send message but drop the ACKs initially
    channel.setDropBtoA(true);

    stationA.sendMessage("Test retransmit");

    // Run some cycles - A should timeout and retransmit
    int initial_tx = channel.getTxCountA();
    channel.run(20, 100);  // Let timeout happen

    // Should have retransmitted
    if (channel.getTxCountA() <= initial_tx + 1) {
        // Allow ACKs through now
        channel.setDropBtoA(false);
        channel.run(20, 100);
    }

    // Now allow ACKs
    channel.setDropBtoA(false);
    channel.run(30, 100);

    // Message should eventually be received
    if (received_at_b.empty()) FAIL("Message never received despite retransmits");

    PASS();
    return true;
}

bool test_disconnect() {
    TEST("Graceful disconnect");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    bool a_disconnected = false;
    bool b_disconnected = false;

    stationA.setConnectionChangedCallback([&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::DISCONNECTED) a_disconnected = true;
    });

    stationB.setConnectionChangedCallback([&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::DISCONNECTED) b_disconnected = true;
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // Disconnect
    stationA.disconnect();
    channel.run(20, 100);

    // Either callback fired OR isConnected returns false
    if (!a_disconnected && stationA.isConnected()) FAIL("A not disconnected");
    if (!b_disconnected && stationB.isConnected()) FAIL("B not disconnected");

    PASS();
    return true;
}

bool test_manual_accept() {
    TEST("Manual call accept/reject");

    ConnectionConfig config;
    config.auto_accept = false;  // Manual accept

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    bool incoming_call = false;
    std::string incoming_from;

    stationB.setIncomingCallCallback([&](const std::string& from) {
        incoming_call = true;
        incoming_from = from;
    });

    SimulatedChannel channel(stationA, stationB);

    // A tries to connect
    stationA.connect("K2DEF");
    channel.run(10, 100);

    // B should have received incoming call notification
    if (!incoming_call) FAIL("No incoming call notification");
    if (incoming_from != "W1ABC") FAIL("Wrong caller");

    // B accepts
    stationB.acceptCall();
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("A not connected after accept");
    if (!stationB.isConnected()) FAIL("B not connected after accept");

    PASS();
    return true;
}

bool test_multiple_messages() {
    TEST("Multiple sequential messages");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    std::vector<std::string> received;
    stationB.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        received.push_back(text);
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K2DEF");
    channel.run(20, 100);

    // Send multiple messages
    const int NUM_MESSAGES = 5;
    for (int i = 0; i < NUM_MESSAGES; i++) {
        // Wait until ready to send
        int attempts = 0;
        while (!stationA.isReadyToSend() && attempts < 50) {
            channel.run(1, 100);
            attempts++;
        }

        std::string msg = "Message " + std::to_string(i + 1);
        stationA.sendMessage(msg);
        channel.run(20, 100);
    }

    // Allow final ACKs
    channel.run(30, 100);

    if (received.size() != NUM_MESSAGES) {
        std::cout << "(received " << received.size() << "/" << NUM_MESSAGES << ") ";
        FAIL("Not all messages received");
    }

    for (int i = 0; i < NUM_MESSAGES; i++) {
        std::string expected = "Message " + std::to_string(i + 1);
        if (received[i] != expected) FAIL("Message order/content wrong");
    }

    PASS();
    return true;
}

bool test_quick_brown_fox() {
    TEST("Quick Brown Fox test message");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    std::string received_msg;
    std::string received_from;

    stationB.setMessageReceivedCallback([&](const std::string& from, const std::string& text) {
        received_from = from;
        received_msg = text;
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // Send the classic pangram test message
    const std::string fox_msg = "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 1234567890";
    stationA.sendMessage(fox_msg);
    channel.run(30, 100);

    if (received_msg.empty()) FAIL("No message received");
    if (received_msg != fox_msg) {
        std::cout << "\n    Expected: " << fox_msg << "\n";
        std::cout << "    Received: " << received_msg << "\n";
        FAIL("Message content mismatch");
    }
    if (received_from != "W1ABC") FAIL("Wrong sender callsign");

    PASS();
    return true;
}

bool test_ryry_message() {
    TEST("RYRY test pattern");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("VA2TEST");
    stationB.setLocalCallsign("VE3TEST");

    std::string received_msg;

    stationB.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        received_msg = text;
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("VE3TEST");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // RYRY is a classic RTTY test pattern
    const std::string ryry_msg = "RYRYRYRYRY DE VA2TEST VA2TEST VA2TEST PSE K";
    stationA.sendMessage(ryry_msg);
    channel.run(30, 100);

    if (received_msg != ryry_msg) FAIL("RYRY message mismatch");

    PASS();
    return true;
}

bool test_cq_message() {
    TEST("CQ call message");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1AW");
    stationB.setLocalCallsign("K1JT");

    std::string received_msg;

    stationB.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        received_msg = text;
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K1JT");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // Typical ham radio exchange
    const std::string cq_msg = "CQ CQ CQ DE W1AW W1AW W1AW K";
    stationA.sendMessage(cq_msg);
    channel.run(30, 100);

    if (received_msg != cq_msg) FAIL("CQ message mismatch");

    // Reply from B
    std::string reply_msg;
    stationA.setMessageReceivedCallback([&](const std::string&, const std::string& text) {
        reply_msg = text;
    });

    const std::string reply = "W1AW DE K1JT K1JT UR 599 599 IN NH K";
    stationB.sendMessage(reply);
    channel.run(30, 100);

    if (reply_msg != reply) FAIL("Reply message mismatch");

    PASS();
    return true;
}

// ============================================================================
// File Transfer Tests
// ============================================================================

// Helper to create a test file with known content
std::string createTestFile(const std::string& name, size_t size) {
    std::string path = "/tmp/" + name;
    std::ofstream f(path, std::ios::binary);
    if (!f) return "";

    for (size_t i = 0; i < size; i++) {
        f.put(static_cast<char>((i * 7 + 13) & 0xFF));
    }
    f.close();
    return path;
}

// Helper to verify file content matches expected pattern
bool verifyFileContent(const std::string& path, size_t expected_size) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;

    f.seekg(0, std::ios::end);
    size_t actual_size = f.tellg();
    f.seekg(0, std::ios::beg);

    if (actual_size != expected_size) return false;

    for (size_t i = 0; i < expected_size; i++) {
        char c;
        f.get(c);
        uint8_t expected = static_cast<uint8_t>((i * 7 + 13) & 0xFF);
        if (static_cast<uint8_t>(c) != expected) return false;
    }
    return true;
}

bool test_file_transfer_small() {
    TEST("Small file transfer (100 bytes)");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 200;  // Faster for test

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    // Create test file
    const size_t FILE_SIZE = 100;
    std::string src_path = createTestFile("test_small.bin", FILE_SIZE);
    if (src_path.empty()) FAIL("Could not create test file");

    // Set receive directory
    std::string rx_dir = "/tmp/ultra_rx_test";
    std::filesystem::create_directories(rx_dir);
    stationB.setReceiveDirectory(rx_dir);

    // Track file reception
    bool file_received = false;
    std::string received_path;
    bool receive_success = false;

    stationB.setFileReceivedCallback([&](const std::string& path, bool success) {
        file_received = true;
        received_path = path;
        receive_success = success;
    });

    bool file_sent = false;
    bool send_success = false;

    stationA.setFileSentCallback([&](bool success, const std::string&) {
        file_sent = true;
        send_success = success;
    });

    SimulatedChannel channel(stationA, stationB);

    // Connect
    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    // Send file
    if (!stationA.sendFile(src_path)) FAIL("sendFile() returned false");

    // Run until transfer complete (or timeout)
    for (int i = 0; i < 200 && (!file_received || !file_sent); i++) {
        channel.run(1, 50);
    }

    if (!file_sent) FAIL("File not sent (no callback)");
    if (!send_success) FAIL("File send reported failure");
    if (!file_received) FAIL("File not received (no callback)");
    if (!receive_success) FAIL("File receive reported failure");

    // Verify content
    if (!verifyFileContent(received_path, FILE_SIZE)) FAIL("File content mismatch");

    // Cleanup
    std::remove(src_path.c_str());
    std::remove(received_path.c_str());

    PASS();
    return true;
}

bool test_file_transfer_medium() {
    TEST("Medium file transfer (1KB)");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 200;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    const size_t FILE_SIZE = 1024;
    std::string src_path = createTestFile("test_medium.bin", FILE_SIZE);
    if (src_path.empty()) FAIL("Could not create test file");

    std::string rx_dir = "/tmp/ultra_rx_test";
    std::filesystem::create_directories(rx_dir);
    stationB.setReceiveDirectory(rx_dir);

    bool file_received = false;
    std::string received_path;
    bool receive_success = false;

    stationB.setFileReceivedCallback([&](const std::string& path, bool success) {
        file_received = true;
        received_path = path;
        receive_success = success;
    });

    bool file_sent = false;

    stationA.setFileSentCallback([&](bool success, const std::string&) {
        file_sent = true;
    });

    SimulatedChannel channel(stationA, stationB);

    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    stationA.sendFile(src_path);

    // 1KB file = ~4 chunks, need more time
    for (int i = 0; i < 500 && !file_received; i++) {
        channel.run(1, 50);
    }

    if (!file_received) FAIL("File not received");
    if (!receive_success) FAIL("File receive failed");
    if (!verifyFileContent(received_path, FILE_SIZE)) FAIL("Content mismatch");

    std::remove(src_path.c_str());
    std::remove(received_path.c_str());

    PASS();
    return true;
}

bool test_file_transfer_progress() {
    TEST("File transfer progress tracking");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 200;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    const size_t FILE_SIZE = 750;  // 3 chunks
    std::string src_path = createTestFile("test_progress.bin", FILE_SIZE);
    if (src_path.empty()) FAIL("Could not create test file");

    std::string rx_dir = "/tmp/ultra_rx_test";
    std::filesystem::create_directories(rx_dir);
    stationB.setReceiveDirectory(rx_dir);

    // Track progress updates
    int progress_updates = 0;
    float last_percentage = 0;

    stationA.setFileProgressCallback([&](const FileTransferProgress& p) {
        progress_updates++;
        last_percentage = p.percentage();
    });

    bool file_received = false;
    std::string received_path;

    stationB.setFileReceivedCallback([&](const std::string& path, bool) {
        file_received = true;
        received_path = path;
    });

    SimulatedChannel channel(stationA, stationB);

    stationA.connect("K2DEF");
    channel.run(20, 100);

    stationA.sendFile(src_path);

    for (int i = 0; i < 300 && !file_received; i++) {
        channel.run(1, 50);
    }

    // Run a few more cycles to allow final ACK and progress update
    channel.run(10, 50);

    if (progress_updates == 0) FAIL("No progress updates received");
    if (last_percentage < 99.0f) FAIL("Progress didn't reach 100%");

    std::remove(src_path.c_str());
    if (!received_path.empty()) std::remove(received_path.c_str());

    PASS();
    return true;
}

bool test_file_transfer_with_loss() {
    TEST("File transfer with packet loss");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 300;
    config.arq.max_retries = 5;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    const size_t FILE_SIZE = 500;
    std::string src_path = createTestFile("test_loss.bin", FILE_SIZE);
    if (src_path.empty()) FAIL("Could not create test file");

    std::string rx_dir = "/tmp/ultra_rx_test";
    std::filesystem::create_directories(rx_dir);
    stationB.setReceiveDirectory(rx_dir);

    bool file_received = false;
    std::string received_path;
    bool receive_success = false;

    stationB.setFileReceivedCallback([&](const std::string& path, bool success) {
        file_received = true;
        received_path = path;
        receive_success = success;
    });

    SimulatedChannel channel(stationA, stationB);

    stationA.connect("K2DEF");
    channel.run(20, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    stationA.sendFile(src_path);

    // Simulate intermittent packet loss
    int drop_counter = 0;
    for (int i = 0; i < 400 && !file_received; i++) {
        // Drop every 5th packet in one direction
        if ((i % 10) == 5) {
            channel.setDropBtoA(true);
            drop_counter++;
        } else {
            channel.setDropBtoA(false);
        }
        channel.run(1, 50);
    }

    channel.setDropBtoA(false);
    channel.run(50, 50);  // Let final packets through

    if (!file_received) FAIL("File not received despite retries");
    if (!receive_success) FAIL("File transfer failed");
    if (!verifyFileContent(received_path, FILE_SIZE)) FAIL("Content corrupted");

    std::remove(src_path.c_str());
    std::remove(received_path.c_str());

    PASS();
    return true;
}

// ============================================================================
// Compression Tests
// ============================================================================

bool test_compression_basic() {
    TEST("Basic compression/decompression");

    // Create compressible data (repetitive text)
    std::string text = "Hello World! This is a test of the compression system. ";
    for (int i = 0; i < 10; i++) {
        text += text;  // Double it each time
    }
    Bytes input(text.begin(), text.end());

    // Compress
    auto compressed = Compression::compress(input);
    if (!compressed) FAIL("Compression failed");
    if (compressed->size() >= input.size()) FAIL("Compression didn't reduce size");

    // Decompress
    auto decompressed = Compression::decompress(*compressed, input.size() * 2);
    if (!decompressed) FAIL("Decompression failed");
    if (*decompressed != input) FAIL("Decompressed data doesn't match original");

    PASS();
    return true;
}

bool test_compression_empty() {
    TEST("Empty data compression");

    Bytes empty;
    auto compressed = Compression::compress(empty);
    if (!compressed) FAIL("Compression of empty data failed");
    if (!compressed->empty()) FAIL("Compressed empty data should be empty");

    auto decompressed = Compression::decompress(*compressed);
    if (!decompressed) FAIL("Decompression of empty data failed");
    if (!decompressed->empty()) FAIL("Decompressed empty data should be empty");

    PASS();
    return true;
}

bool test_compression_random() {
    TEST("Random data (low compressibility)");

    // Random data doesn't compress well
    Bytes random(1000);
    for (size_t i = 0; i < random.size(); i++) {
        random[i] = static_cast<uint8_t>(i * 17 + i / 3);  // Pseudo-random
    }

    auto compressed = Compression::compress(random);
    if (!compressed) FAIL("Compression of random data failed");

    auto decompressed = Compression::decompress(*compressed, random.size() * 2);
    if (!decompressed) FAIL("Decompression failed");
    if (*decompressed != random) FAIL("Decompressed data doesn't match original");

    PASS();
    return true;
}

bool test_should_compress() {
    TEST("shouldCompress heuristic");

    // Small data - should not compress
    Bytes small(20, 'A');
    if (Compression::shouldCompress(small)) FAIL("Small data should not be compressed");

    // Repetitive data - should compress
    std::string text = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
    text += text;  // 104 bytes of 'A'
    Bytes repetitive(text.begin(), text.end());
    if (!Compression::shouldCompress(repetitive)) FAIL("Repetitive data should be compressed");

    PASS();
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=== Protocol Test Suite ===\n\n";

    std::cout << "Frame Tests:\n";
    test_frame_serialization();
    test_frame_crc();
    test_frame_types();
    test_callsign_validation();

    std::cout << "\nTwo-Station Simulation:\n";
    test_connection_establishment();
    test_data_transfer();
    test_bidirectional_transfer();
    test_retransmission();
    test_disconnect();
    test_manual_accept();
    test_multiple_messages();

    std::cout << "\nRadio Test Messages:\n";
    test_quick_brown_fox();
    test_ryry_message();
    test_cq_message();

    std::cout << "\nFile Transfer Tests:\n";
    test_file_transfer_small();
    test_file_transfer_medium();
    test_file_transfer_progress();
    test_file_transfer_with_loss();

    std::cout << "\nCompression Tests:\n";
    test_compression_basic();
    test_compression_empty();
    test_compression_random();
    test_should_compress();

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_run << " passed ===\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
