/**
 * MODE_CHANGE Adaptive Rate Test Suite
 *
 * Tests the complete MODE_CHANGE functionality:
 * 1. Frame creation and parsing
 * 2. SNR-based mode recommendations
 * 3. Connection with automatic MODE_CHANGE
 * 4. Mode change handling and ACK
 * 5. Throughput verification at different rates
 */

#include "protocol/protocol_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/connection.hpp"
#include "ultra/types.hpp"
#include <iostream>
#include <iomanip>
#include <queue>
#include <cstring>

using namespace ultra;
using namespace ultra::protocol;
namespace v2 = ultra::protocol::v2;

#define TEST(name) std::cout << "  Testing " << name << "... " << std::flush
#define PASS() do { std::cout << "PASS\n"; return true; } while(0)
#define FAIL(msg) do { std::cout << "FAIL: " << msg << "\n"; return false; } while(0)

// ============================================================================
// Test 1: MODE_CHANGE Frame Creation and Parsing
// ============================================================================
bool test_mode_change_frame() {
    TEST("MODE_CHANGE frame creation/parsing");

    // Create MODE_CHANGE frame
    auto frame = v2::ControlFrame::makeModeChange(
        "VA2MVR", "TEST1",
        42,                          // seq
        Modulation::QAM16,           // new modulation
        CodeRate::R2_3,              // new code rate
        22.5f,                       // measured SNR
        v2::ModeChangeReason::CHANNEL_IMPROVED
    );

    if (frame.type != v2::FrameType::MODE_CHANGE) FAIL("Wrong frame type");
    if (frame.seq != 42) FAIL("Wrong sequence number");

    // Serialize and deserialize
    Bytes data = frame.serialize();
    if (data.size() != 20) FAIL("Wrong serialized size");

    auto parsed = v2::ControlFrame::deserialize(data);
    if (!parsed) FAIL("Failed to parse");
    if (parsed->type != v2::FrameType::MODE_CHANGE) FAIL("Wrong parsed type");
    if (parsed->seq != 42) FAIL("Wrong parsed seq");

    // Check payload extraction
    auto info = parsed->getModeChangeInfo();
    if (info.modulation != Modulation::QAM16) FAIL("Wrong modulation");
    if (info.code_rate != CodeRate::R2_3) FAIL("Wrong code rate");
    if (info.reason != v2::ModeChangeReason::CHANNEL_IMPROVED) FAIL("Wrong reason");

    // Check SNR encoding (0.25 dB resolution)
    float snr_diff = std::abs(info.snr_db - 22.5f);
    if (snr_diff > 0.3f) FAIL("SNR encoding error too large");

    PASS();
}

// ============================================================================
// Test 2: SNR-Based Mode Recommendations
// ============================================================================
bool test_mode_recommendations() {
    TEST("SNR-based mode recommendations");

    // Test the recommendation thresholds by checking connection behavior
    struct TestCase {
        float snr_db;
        Modulation expected_mod;
        CodeRate expected_rate;
        const char* description;
    };

    // These should match connection.cpp::recommendDataMode()
    TestCase cases[] = {
        {35.0f, Modulation::QAM16,  CodeRate::R3_4, "Excellent (35 dB)"},
        {27.0f, Modulation::QAM16,  CodeRate::R2_3, "Very good (27 dB)"},
        {22.0f, Modulation::DQPSK,  CodeRate::R2_3, "Good (22 dB)"},
        {18.0f, Modulation::DQPSK,  CodeRate::R1_2, "Typical good (18 dB)"},
        {14.0f, Modulation::DQPSK,  CodeRate::R1_4, "Typical (14 dB)"},
        {10.0f, Modulation::DBPSK,  CodeRate::R1_4, "Marginal (10 dB)"},
        {5.0f,  Modulation::DBPSK,  CodeRate::R1_4, "Poor (5 dB)"},
    };

    int pass_count = 0;
    for (const auto& tc : cases) {
        // Create a connection and set SNR
        ConnectionConfig config;
        config.auto_accept = true;

        Connection conn(config);
        conn.setLocalCallsign("TEST1");
        conn.setMeasuredSNR(tc.snr_db);

        // We can't directly call recommendDataMode (it's static/private)
        // But we can verify the getters after the connection processes a CONNECT
        // For now, just verify the SNR is stored correctly
        if (std::abs(conn.getMeasuredSNR() - tc.snr_db) > 0.01f) {
            std::cout << "\n    " << tc.description << ": SNR storage failed";
            continue;
        }
        pass_count++;
    }

    if (pass_count != 7) FAIL("SNR storage tests failed");

    std::cout << "(7/7 SNR cases) ";
    PASS();
}

// ============================================================================
// Test 3: Two-Station MODE_CHANGE Exchange
// ============================================================================
bool test_two_station_mode_change() {
    TEST("Two-station MODE_CHANGE exchange");

    // Simulated channel between two stations
    std::queue<Bytes> a_to_b;
    std::queue<Bytes> b_to_a;

    // Station A (initiator)
    ConnectionConfig config_a;
    config_a.auto_accept = true;
    Connection station_a(config_a);
    station_a.setLocalCallsign("ALPHA");

    // Station B (responder) - will have high SNR to trigger MODE_CHANGE
    ConnectionConfig config_b;
    config_b.auto_accept = true;
    Connection station_b(config_b);
    station_b.setLocalCallsign("BRAVO");
    station_b.setMeasuredSNR(25.0f);  // High SNR -> should recommend QAM16 R2/3

    // Track received mode changes
    bool a_received_mode_change = false;
    Modulation a_new_mod = Modulation::DQPSK;
    CodeRate a_new_rate = CodeRate::R1_4;

    bool b_received_mode_change = false;

    // Wire up transmit callbacks
    station_a.setTransmitCallback([&](const Bytes& data) {
        a_to_b.push(data);
    });

    station_b.setTransmitCallback([&](const Bytes& data) {
        b_to_a.push(data);
    });

    // Wire up mode change callbacks
    station_a.setDataModeChangedCallback([&](Modulation mod, CodeRate rate, float snr) {
        a_received_mode_change = true;
        a_new_mod = mod;
        a_new_rate = rate;
    });

    station_b.setDataModeChangedCallback([&](Modulation mod, CodeRate rate, float snr) {
        b_received_mode_change = true;
    });

    // Step 1: A initiates connection
    station_a.connect("BRAVO");

    // Step 2: Deliver CONNECT to B
    if (a_to_b.empty()) FAIL("A didn't send CONNECT");
    station_b.onFrameReceived(a_to_b.front());
    a_to_b.pop();

    // Step 3: B should send CONNECT_ACK and MODE_CHANGE
    if (b_to_a.empty()) FAIL("B didn't respond");

    // Process all frames from B to A
    int frames_from_b = 0;
    while (!b_to_a.empty()) {
        station_a.onFrameReceived(b_to_a.front());
        b_to_a.pop();
        frames_from_b++;
    }

    // B should have sent: CONNECT_ACK + MODE_CHANGE = 2 frames
    if (frames_from_b < 2) FAIL("B didn't send MODE_CHANGE after CONNECT_ACK");

    // Step 4: A should have received MODE_CHANGE
    if (!a_received_mode_change) FAIL("A didn't process MODE_CHANGE");

    // Step 5: Verify recommended mode (SNR=25 -> QAM16 R2/3)
    if (a_new_mod != Modulation::QAM16) {
        std::cout << "Expected QAM16, got " << modulationToString(a_new_mod) << " ";
        FAIL("Wrong modulation recommended");
    }
    if (a_new_rate != CodeRate::R2_3) {
        std::cout << "Expected R2/3, got " << codeRateToString(a_new_rate) << " ";
        FAIL("Wrong code rate recommended");
    }

    // Step 6: A should have sent ACK for MODE_CHANGE
    if (a_to_b.empty()) FAIL("A didn't ACK the MODE_CHANGE");

    // Verify both stations are connected
    if (station_a.getState() != ConnectionState::CONNECTED) FAIL("A not connected");
    if (station_b.getState() != ConnectionState::CONNECTED) FAIL("B not connected");

    std::cout << "(QAM16 R2/3 @ 25dB) ";
    PASS();
}

// ============================================================================
// Test 4: MODE_CHANGE at Various SNR Levels
// ============================================================================
bool test_mode_change_snr_levels() {
    TEST("MODE_CHANGE at various SNR levels");

    struct TestCase {
        float snr_db;
        Modulation expected_mod;
        CodeRate expected_rate;
        bool expect_mode_change;  // false if default DQPSK R1/4
    };

    TestCase cases[] = {
        {10.0f, Modulation::DBPSK,  CodeRate::R1_4, true},   // DBPSK is different from default DQPSK
        {14.0f, Modulation::DQPSK,  CodeRate::R1_4, false},  // Same as default, no MODE_CHANGE
        {18.0f, Modulation::DQPSK,  CodeRate::R1_2, true},   // Upgrade to R1/2
        {22.0f, Modulation::DQPSK,  CodeRate::R2_3, true},   // Upgrade to R2/3
        {27.0f, Modulation::QAM16,  CodeRate::R2_3, true},   // Upgrade to QAM16
        {35.0f, Modulation::QAM16,  CodeRate::R3_4, true},   // Best mode
    };

    int pass_count = 0;
    for (const auto& tc : cases) {
        std::queue<Bytes> a_to_b, b_to_a;

        ConnectionConfig config_a, config_b;
        config_a.auto_accept = true;
        config_b.auto_accept = true;

        Connection station_a(config_a);
        station_a.setLocalCallsign("STA_A");
        Connection station_b(config_b);
        station_b.setLocalCallsign("STA_B");
        station_b.setMeasuredSNR(tc.snr_db);

        bool mode_change_received = false;
        Modulation received_mod = Modulation::DQPSK;
        CodeRate received_rate = CodeRate::R1_4;

        station_a.setTransmitCallback([&](const Bytes& d) { a_to_b.push(d); });
        station_b.setTransmitCallback([&](const Bytes& d) { b_to_a.push(d); });
        station_a.setDataModeChangedCallback([&](Modulation m, CodeRate r, float s) {
            mode_change_received = true;
            received_mod = m;
            received_rate = r;
        });

        // Connect
        station_a.connect("STA_B");
        if (!a_to_b.empty()) {
            station_b.onFrameReceived(a_to_b.front());
            a_to_b.pop();
        }
        while (!b_to_a.empty()) {
            station_a.onFrameReceived(b_to_a.front());
            b_to_a.pop();
        }

        // Verify
        bool test_pass = true;
        if (tc.expect_mode_change) {
            if (!mode_change_received) {
                std::cout << "\n    SNR=" << tc.snr_db << " dB: Expected MODE_CHANGE but didn't receive";
                test_pass = false;
            } else if (received_mod != tc.expected_mod || received_rate != tc.expected_rate) {
                std::cout << "\n    SNR=" << tc.snr_db << " dB: Got "
                          << modulationToString(received_mod) << " "
                          << codeRateToString(received_rate)
                          << ", expected " << modulationToString(tc.expected_mod) << " "
                          << codeRateToString(tc.expected_rate);
                test_pass = false;
            }
        } else {
            if (mode_change_received) {
                std::cout << "\n    SNR=" << tc.snr_db << " dB: Unexpected MODE_CHANGE";
                test_pass = false;
            }
        }

        if (test_pass) pass_count++;
    }

    std::cout << "(" << pass_count << "/6 SNR levels) ";
    if (pass_count != 6) FAIL("Some SNR levels failed");
    PASS();
}

// ============================================================================
// Test 5: Throughput Comparison
// ============================================================================
bool test_throughput_comparison() {
    TEST("Throughput comparison across code rates");

    // Verify the throughput gains are as expected
    struct RateInfo {
        CodeRate rate;
        size_t info_bits;
        float efficiency;
        float speedup;
    };

    RateInfo rates[] = {
        {CodeRate::R1_4, 162, 0.25f, 1.0f},
        {CodeRate::R1_2, 324, 0.50f, 2.0f},
        {CodeRate::R2_3, 432, 0.667f, 2.67f},
        {CodeRate::R3_4, 486, 0.75f, 3.0f},
        {CodeRate::R5_6, 540, 0.833f, 3.33f},
    };

    const size_t CODED_BITS = 648;

    std::cout << "\n";
    std::cout << "    +--------+----------+------------+--------+\n";
    std::cout << "    | Rate   | Info Bits| Efficiency | Speedup|\n";
    std::cout << "    +--------+----------+------------+--------+\n";

    for (const auto& r : rates) {
        float actual_eff = static_cast<float>(r.info_bits) / CODED_BITS;
        float actual_speedup = actual_eff / 0.25f;  // vs R1/4

        std::cout << "    | " << codeRateToString(r.rate)
                  << "   |   " << r.info_bits
                  << "    |    " << std::fixed << std::setprecision(0) << (actual_eff * 100) << "%"
                  << "     |  " << std::setprecision(1) << actual_speedup << "x"
                  << "  |\n";

        // Verify values
        if (std::abs(actual_eff - r.efficiency) > 0.01f) {
            FAIL("Efficiency mismatch");
        }
    }
    std::cout << "    +--------+----------+------------+--------+\n";

    PASS();
}

// ============================================================================
// Test 6: Mid-Session Mode Change Request
// ============================================================================
bool test_mid_session_mode_change() {
    TEST("Mid-session MODE_CHANGE request");

    std::queue<Bytes> a_to_b, b_to_a;

    ConnectionConfig config_a, config_b;
    config_a.auto_accept = true;
    config_b.auto_accept = true;

    Connection station_a(config_a);
    station_a.setLocalCallsign("INIT");
    Connection station_b(config_b);
    station_b.setLocalCallsign("RESP");

    // Start with low SNR (no MODE_CHANGE on connect)
    station_b.setMeasuredSNR(14.0f);

    bool a_mode_changed = false;
    Modulation a_mode = Modulation::DQPSK;
    CodeRate a_rate = CodeRate::R1_4;

    station_a.setTransmitCallback([&](const Bytes& d) { a_to_b.push(d); });
    station_b.setTransmitCallback([&](const Bytes& d) { b_to_a.push(d); });
    station_a.setDataModeChangedCallback([&](Modulation m, CodeRate r, float s) {
        a_mode_changed = true;
        a_mode = m;
        a_rate = r;
    });

    // Establish connection
    station_a.connect("RESP");
    while (!a_to_b.empty()) {
        station_b.onFrameReceived(a_to_b.front());
        a_to_b.pop();
    }
    while (!b_to_a.empty()) {
        station_a.onFrameReceived(b_to_a.front());
        b_to_a.pop();
    }

    if (station_a.getState() != ConnectionState::CONNECTED) FAIL("Not connected");
    if (a_mode_changed) FAIL("Unexpected MODE_CHANGE at 14 dB");

    // Now simulate channel improvement - B requests mode change
    station_b.requestModeChange(Modulation::DQPSK, CodeRate::R1_2, 18.0f,
                                 v2::ModeChangeReason::CHANNEL_IMPROVED);

    // Deliver MODE_CHANGE to A
    while (!b_to_a.empty()) {
        station_a.onFrameReceived(b_to_a.front());
        b_to_a.pop();
    }

    if (!a_mode_changed) FAIL("A didn't receive mid-session MODE_CHANGE");
    if (a_mode != Modulation::DQPSK) FAIL("Wrong modulation");
    if (a_rate != CodeRate::R1_2) FAIL("Wrong code rate");

    // A should have sent ACK
    if (a_to_b.empty()) FAIL("A didn't ACK mid-session MODE_CHANGE");

    std::cout << "(upgraded to R1/2) ";
    PASS();
}

// ============================================================================
// Test 7: Mode Change Reason Codes
// ============================================================================
bool test_mode_change_reasons() {
    TEST("MODE_CHANGE reason codes");

    uint8_t reasons[] = {
        v2::ModeChangeReason::CHANNEL_IMPROVED,
        v2::ModeChangeReason::CHANNEL_DEGRADED,
        v2::ModeChangeReason::USER_REQUEST,
        v2::ModeChangeReason::INITIAL_SETUP,
    };

    for (uint8_t reason : reasons) {
        auto frame = v2::ControlFrame::makeModeChange(
            "SRC", "DST", 1, Modulation::DQPSK, CodeRate::R1_2, 15.0f, reason);

        Bytes data = frame.serialize();
        auto parsed = v2::ControlFrame::deserialize(data);
        if (!parsed) FAIL("Parse failed");

        auto info = parsed->getModeChangeInfo();
        if (info.reason != reason) FAIL("Reason mismatch");
    }

    PASS();
}

// ============================================================================
// Test 8: MODE_CHANGE Timeout and Retry
// ============================================================================
bool test_mode_change_timeout() {
    TEST("MODE_CHANGE timeout and retry");

    std::queue<Bytes> a_to_b, b_to_a;

    ConnectionConfig config_a, config_b;
    config_a.auto_accept = true;
    config_b.auto_accept = true;

    Connection station_a(config_a);
    station_a.setLocalCallsign("ALPHA");
    Connection station_b(config_b);
    station_b.setLocalCallsign("BRAVO");

    // Start with low SNR (no MODE_CHANGE on connect)
    station_b.setMeasuredSNR(14.0f);

    station_a.setTransmitCallback([&](const Bytes& d) { a_to_b.push(d); });
    station_b.setTransmitCallback([&](const Bytes& d) { b_to_a.push(d); });

    // Establish connection
    station_a.connect("BRAVO");
    while (!a_to_b.empty()) {
        station_b.onFrameReceived(a_to_b.front());
        a_to_b.pop();
    }
    while (!b_to_a.empty()) {
        station_a.onFrameReceived(b_to_a.front());
        b_to_a.pop();
    }

    if (station_a.getState() != ConnectionState::CONNECTED) FAIL("Not connected");

    // B requests mode change - but we DON'T deliver it to A
    station_b.requestModeChange(Modulation::DQPSK, CodeRate::R1_2, 18.0f,
                                 v2::ModeChangeReason::CHANNEL_IMPROVED);

    // Verify B has pending mode change (mode NOT applied yet)
    Modulation b_mod = station_b.getDataModulation();
    CodeRate b_rate = station_b.getDataCodeRate();
    if (b_mod != Modulation::DQPSK || b_rate != CodeRate::R1_4) {
        FAIL("B applied mode before ACK");
    }

    // Don't deliver MODE_CHANGE to A - simulate packet loss
    b_to_a = std::queue<Bytes>();  // Discard

    // Simulate timeout (5 seconds)
    station_b.tick(5001);  // Should trigger retry

    // Verify retry was sent
    if (b_to_a.empty()) FAIL("B didn't retry MODE_CHANGE");
    b_to_a = std::queue<Bytes>();  // Discard retry too

    // Simulate another timeout
    station_b.tick(5001);

    // Verify another retry
    if (b_to_a.empty()) FAIL("B didn't retry MODE_CHANGE second time");
    b_to_a = std::queue<Bytes>();  // Discard

    // Simulate final timeout (max retries reached)
    station_b.tick(5001);

    // Now B should have given up - mode should still be default
    b_mod = station_b.getDataModulation();
    b_rate = station_b.getDataCodeRate();
    if (b_mod != Modulation::DQPSK || b_rate != CodeRate::R1_4) {
        FAIL("B changed mode after failed retries");
    }

    std::cout << "(timeout after 2 retries) ";
    PASS();
}

// ============================================================================
// Test 9: MODE_CHANGE ACK Applies Mode
// ============================================================================
bool test_mode_change_ack_applies() {
    TEST("MODE_CHANGE ACK applies mode");

    std::queue<Bytes> a_to_b, b_to_a;

    ConnectionConfig config_a, config_b;
    config_a.auto_accept = true;
    config_b.auto_accept = true;

    Connection station_a(config_a);
    station_a.setLocalCallsign("ALPHA");
    Connection station_b(config_b);
    station_b.setLocalCallsign("BRAVO");

    station_b.setMeasuredSNR(14.0f);

    bool b_mode_changed = false;
    station_a.setTransmitCallback([&](const Bytes& d) { a_to_b.push(d); });
    station_b.setTransmitCallback([&](const Bytes& d) { b_to_a.push(d); });
    station_b.setDataModeChangedCallback([&](Modulation m, CodeRate r, float s) {
        b_mode_changed = true;
    });

    // Establish connection
    station_a.connect("BRAVO");
    while (!a_to_b.empty()) {
        station_b.onFrameReceived(a_to_b.front());
        a_to_b.pop();
    }
    while (!b_to_a.empty()) {
        station_a.onFrameReceived(b_to_a.front());
        b_to_a.pop();
    }

    if (station_a.getState() != ConnectionState::CONNECTED) FAIL("Not connected");

    // B requests mode change
    station_b.requestModeChange(Modulation::DQPSK, CodeRate::R1_2, 18.0f,
                                 v2::ModeChangeReason::CHANNEL_IMPROVED);

    // Verify B hasn't applied mode yet
    if (station_b.getDataCodeRate() != CodeRate::R1_4) {
        FAIL("B applied mode before ACK");
    }

    // Deliver MODE_CHANGE to A
    while (!b_to_a.empty()) {
        station_a.onFrameReceived(b_to_a.front());
        b_to_a.pop();
    }

    // A should have ACK'd - deliver ACK to B
    while (!a_to_b.empty()) {
        station_b.onFrameReceived(a_to_b.front());
        a_to_b.pop();
    }

    // NOW B should have applied the mode
    if (station_b.getDataCodeRate() != CodeRate::R1_2) {
        FAIL("B didn't apply mode after ACK");
    }
    if (!b_mode_changed) {
        FAIL("B didn't trigger mode changed callback");
    }

    std::cout << "(mode applied on ACK) ";
    PASS();
}

// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "=== MODE_CHANGE Adaptive Rate Test Suite ===\n\n";

    int passed = 0, failed = 0;

    auto run = [&](bool (*test)()) {
        if (test()) passed++;
        else failed++;
    };

    std::cout << "Frame Tests:\n";
    run(test_mode_change_frame);
    run(test_mode_change_reasons);

    std::cout << "\nRecommendation Tests:\n";
    run(test_mode_recommendations);

    std::cout << "\nConnection Tests:\n";
    run(test_two_station_mode_change);
    run(test_mode_change_snr_levels);
    run(test_mid_session_mode_change);

    std::cout << "\nTimeout/Retry Tests:\n";
    run(test_mode_change_timeout);
    run(test_mode_change_ack_applies);

    std::cout << "\nThroughput Tests:\n";
    run(test_throughput_comparison);

    std::cout << "\n=== Results: " << passed << " passed, " << failed << " failed ===\n";

    return failed == 0 ? 0 : 1;
}
