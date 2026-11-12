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

#include "env_compat.hpp"
#include "protocol/arq_interface.hpp"
#include "protocol/selective_repeat_arq.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/file_transfer.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <memory>
#include <cstdlib>
#include <queue>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::protocol;

namespace ultra {
namespace protocol {

struct SelectiveRepeatARQTestAccess {
    static std::optional<size_t> slotFor(const SelectiveRepeatARQ& arq,
                                         const Bytes& frame) {
        return arq.matchingLiveTXSlot(frame);
    }

    static int retryCount(const SelectiveRepeatARQ& arq, const Bytes& frame) {
        const auto slot = slotFor(arq, frame);
        return slot ? arq.tx_window_[*slot].retry_count : -1;
    }

    static bool rttSampleEligible(const SelectiveRepeatARQ& arq,
                                  const Bytes& frame) {
        const auto slot = slotFor(arq, frame);
        return slot && arq.tx_window_[*slot].rtt_sample_eligible;
    }

    static bool transitionSuspended(const SelectiveRepeatARQ& arq,
                                    const Bytes& frame) {
        const auto slot = slotFor(arq, frame);
        return slot && arq.tx_window_[*slot].timeout_suspended &&
               arq.tx_window_[*slot].timeout_transition_suspended;
    }
};

} // namespace protocol
} // namespace ultra

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

static v2::ControlFrame makeSackAck(uint16_t seq, uint32_t bitmap) {
    auto sack = v2::ControlFrame::makeNack("RX1", "TX1", seq, bitmap);
    sack.type = v2::FrameType::ACK;
    return sack;
}

static bool expectDataSeq(const Bytes& frame_data, uint16_t expected_seq,
                          const std::string& context) {
    auto parsed = v2::DataFrame::deserialize(frame_data);
    if (!parsed)
        FAIL(context + ": retransmitted frame did not parse as DATA");
    if (parsed->type != v2::FrameType::DATA)
        FAIL(context + ": retransmitted frame was not DATA");
    if (parsed->seq != expected_seq)
        FAIL(context + ": expected DATA seq=" + std::to_string(expected_seq) +
             ", got seq=" + std::to_string(parsed->seq));
    return true;
}

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
    config.ack_timeout_ms = 4000;
    config.sack_delay_ms = 100;

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

bool test_data_flags_preserve_version_bit() {
    TEST("DATA flags preserve v2 version bit");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    tx.setCodeRate(CodeRate::R1_2);

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    if (!tx.sendDataWithFlags(Bytes{0x01}, v2::Flags::MORE_FRAG))
        FAIL("sendDataWithFlags failed");
    if (!tx.sendFixedDataWithFlags(Bytes{0x02}, v2::Flags::MORE_FRAG))
        FAIL("sendFixedDataWithFlags failed");
    if (!tx.sendVariableDataWithFlags(Bytes{0x03}, v2::Flags::MORE_FRAG))
        FAIL("sendVariableDataWithFlags failed");

    while (channel.hasData()) {
        auto parsed = v2::DataFrame::deserialize(channel.receive());
        if (!parsed)
            FAIL("transmitted DATA frame did not parse");
        if ((parsed->flags & v2::Flags::VERSION_V2) == 0)
            FAIL("VERSION_V2 bit was not preserved");
        if ((parsed->flags & v2::Flags::MORE_FRAG) == 0)
            FAIL("MORE_FRAG bit was not preserved");
    }

    PASS();
    return true;
}

bool test_code_rate_change_aborts_in_flight_fixed_frames() {
    TEST("Code rate change aborts in-flight fixed frames");

    ARQConfig config;
    config.window_size = 3;
    config.ack_timeout_ms = 1000;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    tx.setCodeRate(CodeRate::R1_2);

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    int completions = 0;
    tx.setSendCompleteCallback([&](bool) { completions++; });

    for (int i = 0; i < 3; i++) {
        Bytes data = {static_cast<uint8_t>(0xA0 + i)};
        if (!tx.sendFixedDataWithFlags(data, v2::Flags::MORE_FRAG))
            FAIL("Failed to send fixed DATA frame " + std::to_string(i));
    }

    if (tx.getAvailableSlots() != 0)
        FAIL("Window should be full before code-rate change");

    channel.clear();
    tx.setCodeRate(CodeRate::R1_4);

    if (tx.getCodeRate() != CodeRate::R1_4)
        FAIL("Code rate was not updated");
    if (tx.getAvailableSlots() != 3)
        FAIL("Window was not fully available after code-rate change");
    if (completions != 0)
        FAIL("Abort should not report ACK/failure completion");

    tx.tick(2000);
    if (channel.size() != 0)
        FAIL("Aborted in-flight frames were retransmitted after code-rate change");

    PASS();
    return true;
}

// BUG-MCDPSK-RTO-FULL-WINDOW: with an in-flight timeout table, a lone outstanding frame times out
// after table[1] and its RESEND is armed with table[1] too (the resend used to inherit the full
// scalar). Without a table, the scalar applies.
bool test_retransmit_timeout_follows_inflight_table() {
    TEST("Resend timeout follows the in-flight table");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 40000;
    config.sack_delay_ms = 100;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    const uint32_t table[] = {0, 10000, 20000, 30000, 40000};
    tx.setAckTimeoutTable(table, 5);

    if (!tx.sendData(Bytes{0x01, 0x02}))
        FAIL("Failed to send data");
    if (channel.size() != 1)
        FAIL("Frame not transmitted");
    channel.clear();
    tx.tick(9000);
    if (channel.size() != 0)
        FAIL("Resent before table[1]");
    tx.tick(1001);
    if (channel.size() != 1)
        FAIL("Lone frame not resent at table[1]");
    channel.clear();
    tx.tick(10001);
    if (channel.size() != 1)
        FAIL("The resend waited longer than table[1]");

    // No table: the scalar.
    SelectiveRepeatARQ tx2(config);
    tx2.setCallsigns("TX1", "RX1");
    ByteChannel channel2;
    tx2.setTransmitCallback([&](const Bytes& data) { channel2.send(data); });
    tx2.setAckTimeoutTable(table, 5);
    tx2.setAckTimeoutTable(nullptr, 0);
    tx2.sendData(Bytes{0x03});
    channel2.clear();
    tx2.tick(20000);
    if (channel2.size() != 0)
        FAIL("A cleared table must fall back to the scalar");
    tx2.tick(20001);
    if (channel2.size() != 1)
        FAIL("Scalar timeout not honoured");

    PASS();
    return true;
}

// A frame's escape evidence counts the ACK rounds it was missing from, never retransmission
// timeouts (INV-RATE-EVIDENCE-001): a dead channel resends without anyone hearing.
bool test_heard_misses_count_ack_rounds_not_timeouts() {
    TEST("Heard misses count ACK rounds, not timeouts");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 10000;
    config.sack_delay_ms = 100;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    for (uint8_t i = 0; i < 2; ++i) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(0x10 + i)}))
            FAIL("Failed to send data");
    }
    for (int round = 0; round < 6; ++round) tx.tick(10001);  // six silent rounds
    if (tx.maxInFlightRetryCount() < 6)
        FAIL("Six timeouts did not resend six times");
    if (tx.maxInFlightHeardMissCount() != 0)
        FAIL("Timeouts counted as heard misses");

    // The peer answers: seq 1 arrived, seq 0 did not (nothing in order -> ACK seq 0xFFFF; SACK bit
    // i is seq 0xFFFF + 1 + i, so bit 1 = seq 1). Each fresh ACK is one heard round, seq 0 missing.
    for (int round = 1; round <= 3; ++round) {
        auto ack = v2::ControlFrame::makeNack("RX1", "TX1", 0xFFFF, 0x2);
        ack.type = v2::FrameType::ACK;
        tx.onFrameReceived(ack.serialize());
        if (tx.maxInFlightHeardMissCount() != round) {
            std::cout << "(heard misses " << tx.maxInFlightHeardMissCount() << ", round " << round
                      << ") ";
            FAIL("A fresh ACK without the frame is one heard miss");
        }
        tx.tick(5000);  // past the duplicate-ACK window, short of another timeout
    }
    PASS();
    return true;
}

// ULTRA_MCDPSK_MEASURED_RTO: a 3-frame DBPSK burst (1392 + 3 x 5184 = 16944 ms) whose ACK comes
// 1433 ms after it ends. The in-flight table (the formula) holds 43552 ms for 3 frames.
namespace measured_rto {
constexpr uint32_t kOverheadMs = 1392, kFrameMs = 5184, kGuardMs = 1920;
constexpr uint32_t kBurstMs = kOverheadMs + 3 * kFrameMs;  // 16944
constexpr uint32_t kLatencyMs = 1433;
constexpr uint32_t kCeilingMs = 43552;

struct Rig {
    ARQConfig config;
    std::unique_ptr<SelectiveRepeatARQ> tx;
    ByteChannel channel;
    uint16_t next_seq = 0;
    explicit Rig(bool measured) {
        config.window_size = 3;
        config.ack_timeout_ms = kCeilingMs;
        config.sack_delay_ms = 100;
        tx = std::make_unique<SelectiveRepeatARQ>(config);
        tx->setCallsigns("TX1", "RX1");
        tx->setTransmitCallback([this](const Bytes& data) { channel.send(data); });
        const uint32_t table[4] = {0, 22048, 32800, kCeilingMs};
        tx->setAckTimeoutTable(table, 4);
        if (measured) tx->setMeasuredBurstTimeouts(kOverheadMs, kFrameMs, kGuardMs, 0);
    }
    bool sendBurst() {  // three frames in one tick: one physical burst
        for (int i = 0; i < 3; ++i) {
            if (!tx->sendData(Bytes{static_cast<uint8_t>(next_seq + i)})) return false;
        }
        next_seq = static_cast<uint16_t>(next_seq + 3);
        return true;
    }
    void ackAll() {  // cumulative ACK of everything sent
        auto ack = v2::ControlFrame::makeNack("RX1", "TX1",
                                              static_cast<uint16_t>(next_seq - 1), 0);
        ack.type = v2::FrameType::ACK;
        tx->onFrameReceived(ack.serialize());
    }
    // A resend fires within `deadline_ms` (+100) of now, and not before (-50).
    bool resendsAt(uint32_t deadline_ms) {
        const int before = tx->maxInFlightRetryCount();
        tx->tick(deadline_ms - 50);
        if (tx->maxInFlightRetryCount() != before) return false;
        tx->tick(150);
        return tx->maxInFlightRetryCount() == before + 1;
    }
};
}  // namespace measured_rto

bool test_measured_burst_timeout() {
    TEST("Measured burst timeout: burst airtime + measured ACK latency, back-off to the table");
    using namespace measured_rto;
    {
        Rig control(false);
        if (!control.sendBurst()) FAIL("send");
        control.tx->tick(kBurstMs + kLatencyMs);
        control.ackAll();
        if (!control.sendBurst()) FAIL("send 2");
        if (!control.resendsAt(kCeilingMs)) FAIL("control: without the model the table governs");
    }
    Rig rig(true);
    if (!rig.sendBurst()) FAIL("send");
    if (!rig.resendsAt(kCeilingMs)) FAIL("before any sample the table governs");
    rig.ackAll();  // after a resend: Karn, no sample
    if (rig.tx->haveMeasuredPostBurstLatency()) FAIL("a resent burst gave a latency sample");

    rig.tx->tick(5000);
    if (!rig.sendBurst()) FAIL("send 2");
    rig.tx->tick(kBurstMs + kLatencyMs);
    rig.ackAll();
    if (!rig.tx->haveMeasuredPostBurstLatency() ||
        std::abs(rig.tx->postBurstLatencySrttMs() - kLatencyMs) > 1.0f) {
        std::cout << "(srtt " << rig.tx->postBurstLatencySrttMs() << ") ";
        FAIL("the sample is the ACK's arrival after the modelled burst end");
    }
    // First sample: rttvar = L/2 = 716.5, 4 x rttvar = 2866 > guard.
    const uint32_t base = kBurstMs + kLatencyMs + 2866;  // 21243
    if (!rig.sendBurst()) FAIL("send 3");
    if (!rig.resendsAt(base)) FAIL("the burst's timeout is its airtime + latency + 4 rttvar");
    if (!rig.resendsAt(2 * base)) FAIL("a silent round doubles it");
    if (!rig.resendsAt(kCeilingMs)) FAIL("back-off stops at the table");
    rig.ackAll();  // the peer is heard again (no sample: resent)
    rig.tx->tick(5000);
    if (!rig.sendBurst()) FAIL("send 4");
    if (!rig.resendsAt(base)) FAIL("an ACK ends the back-off");
    PASS();
    return true;
}

bool test_measured_burst_timeout_survives_constellation_change() {
    TEST("Measured burst timeout: a model change between send and ACK keeps the sample exact");
    using namespace measured_rto;
    Rig rig(true);
    if (!rig.sendBurst()) FAIL("send");
    // The ACK that closes this DBPSK burst also commands DQPSK: the model changes first.
    constexpr uint32_t kDqpskFrameMs = 2624;
    rig.tx->setMeasuredBurstTimeouts(kOverheadMs, kDqpskFrameMs, 1067, 0);
    rig.tx->tick(kBurstMs + kLatencyMs);
    rig.ackAll();
    if (std::abs(rig.tx->postBurstLatencySrttMs() - kLatencyMs) > 1.0f) {
        std::cout << "(srtt " << rig.tx->postBurstLatencySrttMs() << ") ";
        FAIL("the sample used the airtime the burst was sent at");
    }
    if (!rig.sendBurst()) FAIL("send 2");
    if (!rig.resendsAt(kOverheadMs + 3 * kDqpskFrameMs + kLatencyMs + 2866))
        FAIL("the DQPSK burst's timeout uses the DQPSK airtime");
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

bool test_stale_ack_older_than_base_minus_one_is_ignored() {
    TEST("Stale ACK older than base-1 is ignored without freeing TX slots");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    int completions = 0;
    tx.setSendCompleteCallback([&](bool success) {
        if (success) completions++;
    });

    for (int i = 0; i < 4; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to fill TX window at seq=" + std::to_string(i));
    }
    if (tx.getAvailableSlots() != 0)
        FAIL("Expected full TX window before ACK setup");

    auto ack1 = v2::ControlFrame::makeAck("RX1", "TX1", 1);
    tx.onFrameReceived(ack1.serialize());
    if (completions != 2)
        FAIL("Setup ACK should complete seq=0..1, got completions=" +
             std::to_string(completions));
    if (tx.getAvailableSlots() != 2)
        FAIL("Setup ACK should free exactly 2 TX slots");

    const size_t slots_before = tx.getAvailableSlots();
    const int completions_before = completions;

    auto stale = makeSackAck(0, 0x0Cu);
    tx.onFrameReceived(stale.serialize());

    if (tx.getAvailableSlots() != slots_before)
        FAIL("Stale ACK advanced/free'd TX window slots");
    if (completions != completions_before)
        FAIL("Stale ACK caused duplicate send-complete callbacks");
    if (transmitted.size() != 4)
        FAIL("Stale ACK unexpectedly triggered a retransmission");

    auto stats = tx.getStats();
    if (stats.stale_acks_ignored != 1)
        FAIL("Expected stale_acks_ignored=1, got " +
             std::to_string(stats.stale_acks_ignored));
    if (stats.sacks_received != 1)
        FAIL("Rejected stale ACK must not inflate the one accepted setup SACK");

    PASS();
    return true;
}

bool test_future_ack_too_far_ahead_is_ignored() {
    TEST("Future ACK too far ahead is ignored without freeing TX slots");

    ARQConfig config;
    config.window_size = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    int completions = 0;
    tx.setSendCompleteCallback([&](bool success) {
        if (success) completions++;
    });

    for (int i = 0; i < 4; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to fill TX window at seq=" + std::to_string(i));
    }

    auto future = makeSackAck(6, 0x0Fu);
    tx.onFrameReceived(future.serialize());

    if (tx.getAvailableSlots() != 0)
        FAIL("Future ACK advanced/free'd TX window slots");
    if (completions != 0)
        FAIL("Future ACK caused send-complete callbacks");
    if (transmitted.size() != 4)
        FAIL("Future ACK unexpectedly triggered a retransmission");

    auto stats = tx.getStats();
    if (stats.future_acks_ignored != 1)
        FAIL("Expected future_acks_ignored=1, got " +
             std::to_string(stats.future_acks_ignored));
    if (stats.sacks_received != 0)
        FAIL("Rejected future ACK must not count as an accepted SACK");
    if (stats.acks_received != 0)
        FAIL("Future ACK should not count as a received cumulative ACK");

    PASS();
    return true;
}

bool test_duplicate_sack_hole_is_suppressed_without_duplicate_retx_accounting() {
    TEST("Duplicate SACK hole is suppressed without duplicate retransmission accounting");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    for (int i = 0; i < 3; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to send DATA seq=" + std::to_string(i));
    }
    if (tx.getAvailableSlots() != 1)
        FAIL("Expected one free slot before SACK-hole test setup");

    // ACK base-1 with bit1 set: receiver is missing seq=0 but has seq=1.
    auto base_hole = makeSackAck(0xFFFF, 0x02u);
    tx.onFrameReceived(base_hole.serialize());

    auto after_first = tx.getStats();
    if (after_first.hole_events != 1)
        FAIL("First base-hole SACK should record one hole event");
    if (after_first.retransmissions != 0)
        FAIL("First hole indication should not retransmit before two confirmations");
    if (tx.getAvailableSlots() != 1)
        FAIL("Base-hole SACK should not free the missing base slot");

    tx.onFrameReceived(base_hole.serialize());

    auto stats = tx.getStats();
    if (stats.duplicate_acks_ignored != 1)
        FAIL("Expected duplicate_acks_ignored=1, got " +
             std::to_string(stats.duplicate_acks_ignored));
    if (stats.hole_events != 1)
        FAIL("Duplicate SACK should not double-count hole events");
    if (stats.retransmissions != 0 ||
        stats.retransmissions_fast_hole != 0 ||
        stats.retransmissions_timeout != 0)
        FAIL("Duplicate SACK caused retransmission accounting");
    if (transmitted.size() != 3)
        FAIL("Duplicate SACK unexpectedly transmitted a repair frame");

    PASS();
    return true;
}

bool test_hole_probe_not_rearmed_while_fast_hole_in_flight() {
    TEST("hole-probe is not re-armed while a fast-hole retx is in flight");

    // Regression for the faithful-clock (GUI/hardware) double-retx: a base hole
    // is repaired by a fast-hole retransmit, but subsequent hole-confirmation
    // SACKs still show the gap (the repair hasn't landed + been ACKed across the
    // half-duplex turnaround yet). Before the fix, that re-armed the hole-probe,
    // which then fired a redundant retransmit of a frame the fast-hole already
    // recovered. The fix suppresses re-arming while fast_retx_cooldown_ms > 0.
    // ack_timeout=5000 keeps the per-slot RTO (5000ms) well clear of the
    // hole-probe timer (clamp(2500,1200,2500)=2500ms) so this test exercises the
    // probe path, not the RTO. fast-hole cooldown = clamp(833,300,1200)=833ms.
    ARQConfig config;
    config.window_size = 8;
    config.ack_timeout_ms = 5000;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    for (int i = 0; i < 8; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to send DATA seq=" + std::to_string(i));
    }
    if (transmitted.size() != 8)
        FAIL("Expected 8 initial transmits");

    // Evolving base-hole SACKs (seq0 missing throughout): distinct bitmaps so
    // each counts as a fresh hole confirmation (not a dedup'd duplicate).
    tx.onFrameReceived(makeSackAck(0xFFFF, 0x02u).serialize());  // confirm #1 -> arm probe
    tx.tick(50);
    tx.onFrameReceived(makeSackAck(0xFFFF, 0x06u).serialize());  // confirm #2 -> fast-hole fires, disarm probe

    auto mid = tx.getStats();
    if (mid.retransmissions_fast_hole != 1)
        FAIL("Expected exactly one fast-hole repair, got " +
             std::to_string(mid.retransmissions_fast_hole));

    // Third confirmation arrives while the fast-hole repair is still in flight
    // (cooldown active). With the fix this must NOT re-arm the hole-probe.
    tx.tick(100);  // cooldown 833 -> 733, still active
    tx.onFrameReceived(makeSackAck(0xFFFF, 0x0Eu).serialize());  // confirm #3 (cooldown still > 0)

    // Tick past the hole-probe timer (2500ms) but short of the RTO (5000ms):
    // if the probe had been re-armed it would fire a redundant retransmit here.
    tx.tick(2600);

    auto stats = tx.getStats();
    if (stats.retransmissions_hole_probe != 0)
        FAIL("Hole-probe redundantly retransmitted a frame the fast-hole repair "
             "already covered (retransmissions_hole_probe=" +
             std::to_string(stats.retransmissions_hole_probe) + ")");
    if (stats.retransmissions_timeout != 0)
        FAIL("RTO should not fire within the window (got timeout retx=" +
             std::to_string(stats.retransmissions_timeout) + ")");
    if (stats.retransmissions_fast_hole != 1)
        FAIL("Fast-hole repair count changed unexpectedly: " +
             std::to_string(stats.retransmissions_fast_hole));
    if (transmitted.size() != 9)  // 8 initial + 1 fast-hole; NO probe/timeout retx
        FAIL("Expected 9 transmits (8 + 1 fast-hole), got " +
             std::to_string(transmitted.size()));

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

    // Default ACK batching sends a SACK at window threshold (4 frames), not
    // after every in-order frame. Three frames should only arm the tail timer.
    if (channel.size() != 0)
        FAIL("Expected no immediate SACK before default batch threshold");

    PASS();
    return true;
}

bool test_duplicate_data_is_not_delivered_twice_and_sends_recovery_sack() {
    TEST("Duplicate DATA is ignored without duplicate delivery and keeps SACK recovery sane");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) {
        received.push_back(data);
    });

    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0x42});
    rx.onFrameReceived(f0.serialize());

    if (received.size() != 1)
        FAIL("Initial DATA seq=0 should be delivered once");
    if (channel.size() != 0)
        FAIL("Initial in-order frame should only arm delayed SACK at this batch size");

    rx.onFrameReceived(f0.serialize());

    if (received.size() != 1)
        FAIL("Duplicate DATA seq=0 was delivered twice");
    if (channel.size() != 1)
        FAIL("Duplicate delivered DATA should send one recovery SACK, got " +
             std::to_string(channel.size()));

    auto ack = v2::ControlFrame::deserialize(channel.receive());
    if (!ack || ack->type != v2::FrameType::ACK || ack->seq != 0)
        FAIL("Duplicate DATA recovery SACK should cumulatively ACK seq=0");
    if (v2::NackPayload::decode(ack->payload).cw_bitmap != 0)
        FAIL("Duplicate DATA recovery SACK should have empty current-window bitmap");

    rx.tick(config.sack_delay_ms);
    if (channel.size() != 0)
        FAIL("Duplicate DATA left a stale delayed SACK pending");

    auto stats = rx.getStats();
    if (stats.frames_received != 1)
        FAIL("Duplicate DATA should not increment frames_received twice");
    if (stats.sack_trigger_out_of_window != 1)
        FAIL("Expected duplicate delivered DATA to use out-of-window SACK trigger");

    PASS();
    return true;
}

// BUG-ARQ-SEQ-COLLISION interim salvage: below-window FILE_START/FILE_DATA payloads are
// handed up the normal delivery callback (the file layer is offset-idempotent) instead of
// dying at the seq-keyed dedup — but ONLY with ULTRA_BELOW_WINDOW_FILE_SALVAGE=1, ONLY for
// FILE payload types, and ONLY strictly below the window (never far-future).
bool test_below_window_file_salvage() {
    TEST("Below-window FILE frames salvaged only with knob on");

    const Bytes file_chunk_a = {0x02, 0x00, 0x00, 0x00, 0x00, 0xAA, 0xBB};  // FILE_DATA off=0
    const Bytes file_chunk_b = {0x02, 0x00, 0x00, 0x00, 0x04, 0xCC};        // FILE_DATA off=4 (regrid)
    const Bytes file_start   = {0x01, 0x00, 0x00, 0x00, 0x00, 0x10,
                                0x12, 0x34, 0x56, 0x78, 'f'};               // FILE_START
    const Bytes text_msg     = {0x00, 'h', 'i'};                            // TEXT_MESSAGE

    ARQConfig config;
    config.window_size = 4;

    // --- Knob ON ---
    setenv("ULTRA_BELOW_WINDOW_FILE_SALVAGE", "1", 1);
    SelectiveRepeatARQ rx(config);       // latches the knob in the ctor
    unsetenv("ULTRA_BELOW_WINDOW_FILE_SALVAGE");
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) { received.push_back(data); });

    // In-order seq=0 delivers normally and advances rx_base to 1.
    rx.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, file_chunk_a).serialize());
    if (received.size() != 1)
        FAIL("In-order FILE_DATA seq=0 should deliver once");
    if (channel.size() != 0)
        FAIL("In-order frame below batch threshold should not SACK yet");

    // Below-window FILE_DATA (seq=0 again, DIFFERENT bytes — the W16 regrid case):
    // must be salvaged up the delivery callback AND still emit the recovery SACK.
    rx.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, file_chunk_b).serialize());
    if (received.size() != 2)
        FAIL("Below-window FILE_DATA was not salvaged with knob on");
    if (received[1] != file_chunk_b)
        FAIL("Salvaged payload does not match the below-window frame");
    if (channel.size() != 1)
        FAIL("Salvage must not suppress the out-of-window recovery SACK");
    {
        auto ack = v2::ControlFrame::deserialize(channel.receive());
        if (!ack || ack->type != v2::FrameType::ACK)
            FAIL("Out-of-window recovery frame should still be a SACK");
    }

    // Below-window FILE_START is salvaged too (idempotent at the file layer).
    rx.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, file_start).serialize());
    if (received.size() != 3)
        FAIL("Below-window FILE_START was not salvaged with knob on");

    // Below-window TEXT_MESSAGE must NEVER be salvaged (would duplicate the message).
    rx.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, text_msg).serialize());
    if (received.size() != 3)
        FAIL("Below-window TEXT_MESSAGE must not be salvaged");

    // Far-FUTURE out-of-window FILE_DATA (beyond the window, not behind it) must NOT
    // be salvaged — the salvage is strictly below-window.
    rx.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 100, file_chunk_b).serialize());
    if (received.size() != 3)
        FAIL("Far-future FILE_DATA must not be salvaged");

    // --- Knob OFF (explicit =0 opt-out; DEFAULT is ON since 2026-07-03 evening,
    // 9/9 rig field engagements): below-window FILE_DATA dropped as before ---
    setenv("ULTRA_BELOW_WINDOW_FILE_SALVAGE", "0", 1);
    SelectiveRepeatARQ rx_off(config);
    unsetenv("ULTRA_BELOW_WINDOW_FILE_SALVAGE");
    rx_off.setCallsigns("RX1", "TX1");
    ByteChannel channel_off;
    rx_off.setTransmitCallback([&](const Bytes& data) { channel_off.send(data); });
    std::vector<Bytes> received_off;
    rx_off.setDataReceivedCallback([&](const Bytes& data) { received_off.push_back(data); });

    rx_off.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, file_chunk_a).serialize());
    rx_off.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, file_chunk_b).serialize());
    if (received_off.size() != 1)
        FAIL("Knob off: below-window FILE_DATA must be dropped (default unchanged)");
    if (channel_off.size() != 1)
        FAIL("Knob off: out-of-window recovery SACK should still be sent");

    PASS();
    return true;
}

// ============================================================================
// MOVE-EPOCH (BUG-ARQ-SEQ-COLLISION structural fix, ULTRA_ARQ_MOVE_EPOCH)
// ============================================================================

namespace {

// Build a receiver-crafted DATA frame with explicit move-epoch flag bits.
Bytes makeEpochData(uint16_t seq, const Bytes& payload, uint8_t epoch, bool rebase) {
    auto f = v2::DataFrame::makeData("TX1", "RX1", seq, payload);
    f.flags = static_cast<uint8_t>(f.flags | v2::epochToFlags(epoch) |
                                   (rebase ? v2::Flags::EPOCH_REBASE : 0));
    return f.serialize();
}

}  // namespace

// TX side: the epoch counter bumps (mod 4) on the setCodeRate abort that rewinds
// tx_next_seq_ (the collision precondition), every DATA frame carries the current
// epoch in flags bits 6-7, and the frame created at the window base carries
// EPOCH_REBASE.
bool test_move_epoch_bump_on_abort_and_stamp() {
    TEST("MOVE-EPOCH: bump on rate-change abort + flags stamp");

    ARQConfig config;
    config.window_size = 4;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ tx(config);  // latches the knob in the ctor
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    tx.sendData(Bytes{0x01});
    tx.sendData(Bytes{0x02});
    if (transmitted.size() != 2)
        FAIL("Expected 2 initial transmissions");
    {
        auto f0 = v2::DataFrame::deserialize(transmitted[0]);
        auto f1 = v2::DataFrame::deserialize(transmitted[1]);
        if (!f0 || !f1) FAIL("Initial frames did not parse");
        if (v2::epochFromFlags(f0->flags) != 0 || v2::epochFromFlags(f1->flags) != 0)
            FAIL("Pre-abort frames must carry epoch 0");
        if ((f0->flags & v2::Flags::EPOCH_REBASE) == 0)
            FAIL("Frame created at the window base must carry EPOCH_REBASE");
        if ((f1->flags & v2::Flags::EPOCH_REBASE) != 0)
            FAIL("Mid-window frame must NOT carry EPOCH_REBASE");
    }

    // Rate-change TX abort (in-flight > 0) => rewind + epoch bump.
    tx.setCodeRate(CodeRate::R1_2);
    tx.sendData(Bytes{0x03});  // re-uses seq 0 on the new grid
    if (transmitted.size() != 3)
        FAIL("Expected the post-abort frame to transmit");
    {
        auto f = v2::DataFrame::deserialize(transmitted[2]);
        if (!f) FAIL("Post-abort frame did not parse");
        if (f->seq != 0)
            FAIL("Post-abort frame should re-use the rewound base seq 0");
        if (v2::epochFromFlags(f->flags) != 1)
            FAIL("Post-abort frame must carry the bumped epoch 1");
        if ((f->flags & v2::Flags::EPOCH_REBASE) == 0)
            FAIL("Post-abort base frame must carry EPOCH_REBASE");
    }

    PASS();
    return true;
}

// TX side, second bump site (2026-07-04 fix, Phase-2 adversarial-review finding):
// a mid-window regrid through the CW-change abort ALONE — same code rate, so
// setCodeRate early-returns — must ALSO enter a new era. This is the Phase-2
// receiver-commanded QAM16 R3/4 -> QPSK R3/4 demote shape: without the bump the
// receiver's in-order rx_base is stranded below the abandoned seqs with no
// rebase-anchor (BUG-ARQ-SEQ-COLLISION by the other door).
bool test_move_epoch_bump_on_cw_abort_same_rate() {
    TEST("MOVE-EPOCH: bump on same-rate CW-change abort (Phase-2 mid-window regrid)");

    ARQConfig config;
    config.window_size = 4;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ tx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    tx.sendData(Bytes{0x01});
    tx.sendData(Bytes{0x02});
    if (transmitted.size() != 2) FAIL("Expected 2 initial transmissions");
    if (tx.txMoveEpoch() != 0) FAIL("Epoch must start at 0");

    // Same-rate mid-window regrid: ONLY the CW count changes (mod-change shape).
    tx.setFixedFrameCodewords(2);
    if (tx.txMoveEpoch() != 1)
        FAIL("CW-change abort dropping live payload must bump the epoch");

    tx.sendData(Bytes{0x03});  // first frame of the new era at the advanced base
    {
        auto f = v2::DataFrame::deserialize(transmitted.back());
        if (!f) FAIL("Post-abort frame did not parse");
        if (v2::epochFromFlags(f->flags) != 1)
            FAIL("Post-abort frame must carry the bumped epoch 1");
        if ((f->flags & v2::Flags::EPOCH_REBASE) == 0)
            FAIL("Post-abort base frame must carry EPOCH_REBASE (the rx re-anchor)");
    }

    // An abort with NOTHING live must NOT bump (no remap happened — e.g. teardown
    // on an idle window must not burn mod-4 epoch space).
    tx.abortPendingTx();  // window already empty? no: seq 0x03 is live — drain first
    const uint8_t epoch_after_live_abort = tx.txMoveEpoch();
    if (epoch_after_live_abort != 2)
        FAIL("abortPendingTx dropping a live frame must bump");
    tx.abortPendingTx();  // now truly empty
    if (tx.txMoveEpoch() != epoch_after_live_abort)
        FAIL("abortPendingTx on an empty window must NOT bump");

    PASS();
    return true;
}

// RX side: the W16 regrid case. A below-window seq with a NEWER epoch and the
// EPOCH_REBASE anchor is NOT a duplicate — the receiver adopts the era,
// re-anchors rx_base to the rebase seq, and DELIVERS the re-gridded content
// (TEXT payload here, proving it is the epoch machinery — not the FILE salvage —
// accepting it). Its ACKs then echo the new epoch in SACK bitmap bits 16-17.
bool test_move_epoch_regrid_resend_accepted() {
    TEST("MOVE-EPOCH: below-window regrid resend adopted + delivered");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 100;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ rx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) { received.push_back(data); });

    // Era 0: deliver seqs 0..2 (one-way ACK loss at the sender is invisible here —
    // the receiver's base simply runs ahead, the W16 precondition).
    for (uint16_t s = 0; s < 3; ++s) {
        rx.onFrameReceived(makeEpochData(s, Bytes{static_cast<uint8_t>(s)}, 0, s == 0));
    }
    if (received.size() != 3)
        FAIL("Era-0 frames should deliver in order");
    channel.clear();

    // Era 1 regrid: the sender rewound to ITS stale base 0 and re-chunked
    // DIFFERENT bytes under the retired seqs (TEXT — the payload type the interim
    // salvage must never touch).
    rx.onFrameReceived(makeEpochData(0, Bytes{0x00, 0xAA}, 1, /*rebase=*/true));
    if (received.size() != 4)
        FAIL("Regridded rebase frame was not delivered (adoption failed)");
    if (received[3] != Bytes({0x00, 0xAA}))
        FAIL("Delivered payload is not the re-gridded content");

    rx.onFrameReceived(makeEpochData(1, Bytes{0x01, 0xBB}, 1, /*rebase=*/false));
    if (received.size() != 5)
        FAIL("Post-anchor era-1 frame should deliver in order");
    if (rx.getRxBaseSeq() != 2)
        FAIL("rx_base should have re-anchored and advanced to 2");

    // Flush the delayed SACK and verify the epoch echo (bitmap bits 16-17 == 1).
    rx.tick(config.sack_delay_ms);
    if (channel.size() < 1)
        FAIL("Expected a SACK after the delayed timer");
    {
        auto ack = v2::ControlFrame::deserialize(channel.receive());
        if (!ack || ack->type != v2::FrameType::ACK)
            FAIL("Expected an ACK-type SACK");
        const uint32_t bitmap = v2::NackPayload::decode(ack->payload).cw_bitmap;
        if (((bitmap >> 16) & 0x3u) != 1u)
            FAIL("SACK must echo the adopted epoch in bitmap bits 16-17");
        if (ack->seq != 1)
            FAIL("SACK cumulative base should be rx_base-1 = 1 in the new era");
    }

    PASS();
    return true;
}

// TX side: an ACK whose epoch echo predates the current TX epoch is IGNORED for
// retirement (the W16 phantom-retire arm), and a current-epoch ACK then retires.
bool test_move_epoch_stale_ack_ignored() {
    TEST("MOVE-EPOCH: stale-epoch ACK ignored, fresh-epoch ACK retires");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 4000;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ tx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    tx.sendData(Bytes{0x01});
    tx.sendData(Bytes{0x02});
    tx.sendData(Bytes{0x03});

    // Rate-change abort => epoch 1, rewind, re-send new content under seqs 0..2.
    tx.setCodeRate(CodeRate::R1_2);
    tx.sendData(Bytes{0x11});
    tx.sendData(Bytes{0x12});
    tx.sendData(Bytes{0x13});
    if (tx.getAvailableSlots() != 1)
        FAIL("Expected 3 frames in flight after the post-abort refill");

    // Stale-era ACK (epoch bits 0) cumulatively acking seqs 0..2 — the exact W16
    // out-of-window SACK. Must NOT retire anything.
    if (tx.onToneBurstAck(/*group_seq6=*/2, /*bitmap=*/0, /*move_epoch=*/0))
        FAIL("Stale-epoch tone ACK must report rejected to its Connection caller");
    if (tx.getAvailableSlots() != 1)
        FAIL("Stale-epoch ACK must not retire in-flight frames");
    if (tx.getStats().stale_epoch_acks_ignored != 1)
        FAIL("stale_epoch_acks_ignored counter should be 1");

    // Fresh-era tone ACK is accepted and retires normally.
    if (!tx.onToneBurstAck(/*group_seq6=*/2, /*bitmap=*/0, /*move_epoch=*/1))
        FAIL("Fresh-epoch tone ACK must report accepted to its Connection caller");
    if (tx.getAvailableSlots() != 4)
        FAIL("Fresh-epoch ACK should retire all three frames");

    // The same valid physical ACK can be heard again after the frames retire. It is
    // an accepted no-progress duplicate (so Connection may treat it as a legitimate
    // keepalive), not a fabricated/stale payload.
    if (!tx.onToneBurstAck(/*group_seq6=*/2, /*bitmap=*/0, /*move_epoch=*/1))
        FAIL("Valid same-epoch duplicate tone ACK must remain accepted");
    if (tx.getAvailableSlots() != 4)
        FAIL("Valid duplicate tone ACK must not disturb the empty TX window");

    PASS();
    return true;
}

// RX side: adopting a new era from a NON-rebase frame (era head lost) enters the
// unanchored interregnum — no window bookkeeping, TOTAL ack silence (a cumulative
// claim from the old rx_base would fabricate delivery of new-era seqs), FILE
// payloads salvaged. The late rebase frame anchors and normal operation resumes.
bool test_move_epoch_unanchored_wait_for_rebase() {
    TEST("MOVE-EPOCH: unanchored interregnum (ack-silent) until rebase anchor");

    const Bytes file_chunk = {0x02, 0x00, 0x00, 0x00, 0x00, 0xAA};  // FILE_DATA

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 100;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ rx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) { received.push_back(data); });

    // Era 0 anchor point.
    rx.onFrameReceived(makeEpochData(0, Bytes{0x00}, 0, true));
    if (received.size() != 1) FAIL("Era-0 frame should deliver");
    channel.clear();

    // Era 1 arrives WITHOUT the rebase anchor (head lost): adopt, but stay silent.
    rx.onFrameReceived(makeEpochData(5, Bytes{0x00, 0x55}, 1, /*rebase=*/false));
    if (received.size() != 1)
        FAIL("Unanchored TEXT frame must not be delivered");
    if (channel.size() != 0)
        FAIL("Unanchored receiver must not emit any SACK (fabrication hazard)");

    // FILE payloads are salvaged during the interregnum (offset-idempotent layer).
    rx.onFrameReceived(makeEpochData(6, file_chunk, 1, /*rebase=*/false));
    if (received.size() != 2 || received[1] != file_chunk)
        FAIL("Unanchored FILE frame should be salvage-delivered");
    if (channel.size() != 0)
        FAIL("FILE salvage while unanchored must stay ack-silent");

    // Delayed timers must not leak an ack either.
    rx.tick(config.sack_delay_ms * 2);
    if (channel.size() != 0)
        FAIL("No delayed SACK may fire while unanchored");

    // The era base arrives on the sender's RTO resend: anchor + deliver in order.
    rx.onFrameReceived(makeEpochData(4, Bytes{0x04, 0xCC}, 1, /*rebase=*/true));
    if (received.size() != 3 || received[2] != Bytes({0x04, 0xCC}))
        FAIL("Rebase frame should anchor and deliver");
    if (rx.getRxBaseSeq() != 5)
        FAIL("rx_base should be anchored at rebase seq + 1");
    rx.onFrameReceived(makeEpochData(5, Bytes{0x05, 0xDD}, 1, false));
    if (received.size() != 4)
        FAIL("Post-anchor frame should deliver in order");
    rx.tick(config.sack_delay_ms);
    if (channel.size() < 1)
        FAIL("Ack flow should resume after anchoring");

    PASS();
    return true;
}

// Pins the ADOPTION-group window arithmetic (written as the F144/F145 "lost
// last frame" repro; the repro came back GREEN and exposed a LOG-CONVENTION
// misread instead: the ack wire value is highest-received = rx_base-1, while
// the adoption log prints next-expected — ack base=9 after group 5..9 is
// CORRECT). Kept as the regression pin: a rewound epoch group must deliver
// every frame and advance rx_base past its last seq.
bool test_move_epoch_adoption_group_advances_past_last_frame() {
    TEST("MOVE-EPOCH: adoption group advances rx_base past its LAST frame");

    ARQConfig config;
    config.window_size = 8;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ rx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    rx.setCallsigns("RX1", "TX1");

    std::vector<Bytes> delivered;
    rx.setDataReceivedCallback([&](const Bytes& p) { delivered.push_back(p); });
    rx.setTransmitCallback([](const Bytes&) {});

    // Epoch-1 history: seqs 0..7 delivered in order, base -> 8 (the receiver's
    // pre-switch state; F145 sat at base 8 when the rate change hit).
    rx.beginGroupReceive();
    for (uint16_t s = 0; s <= 7; ++s) {
        rx.onFrameReceived(makeEpochData(s, Bytes{static_cast<uint8_t>(s)}, 1, s == 0));
    }
    rx.endGroupReceiveAndAck();
    if (rx.getRxBaseSeq() != 8)
        FAIL("Precondition: epoch-1 in-order group should leave rx_base at 8");
    const size_t epoch1_delivered = delivered.size();

    // The sender rewinds to 5 on the rate-change abort (epoch 2) and sends
    // 5..9 as one group, EPOCH_REBASE on 5.
    rx.beginGroupReceive();
    for (uint16_t s = 5; s <= 9; ++s) {
        rx.onFrameReceived(
            makeEpochData(s, Bytes{static_cast<uint8_t>(0x50 + s)}, 2, s == 5));
    }
    rx.endGroupReceiveAndAck();

    if (delivered.size() - epoch1_delivered != 5)
        FAIL("All 5 epoch-2 frames must deliver (rig lost the last one)");
    if (rx.getRxBaseSeq() != 10)
        FAIL("Adoption group 5..9 must advance rx_base to 10 (rig acked 9)");

    PASS();
    return true;
}

// F163 FIX-4: a rate-change abort must SALVAGE SACKed slots (peer-confirmed
// frames) via the discard callback before clearing them — the file layer skips
// those byte ranges on requeue instead of re-sending bytes the receiver holds.
bool test_rate_abort_salvages_sacked_frames() {
    TEST("rate-change abort fires salvage callback for SACKed slots only");

    ARQConfig config;
    config.window_size = 4;
    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    tx.setTransmitCallback([](const Bytes&) {});

    std::vector<Bytes> salvaged;
    tx.setSackedFrameDiscardedCallback(
        [&](const Bytes& f) { salvaged.push_back(f); });

    // Send 3 frames; SACK the MIDDLE one (seq 1) via an ack with a hole at 0.
    tx.sendData(Bytes{0x10});
    tx.sendData(Bytes{0x11});
    tx.sendData(Bytes{0x12});
    auto sack = v2::ControlFrame::makeNack("RX1", "TX1", /*seq=*/0xFFFF,
                                           /*bitmap=*/0x2);  // base-1, bit->seq 1
    sack.type = v2::FrameType::ACK;
    tx.onFrameReceived(sack.serialize());

    tx.setCodeRate(CodeRate::R1_2);  // abort: clears slots, salvages SACKed

    if (salvaged.size() != 1)
        FAIL("expected exactly 1 salvaged frame (the SACKed seq 1), got " +
             std::to_string(salvaged.size()));
    auto f = v2::DataFrame::deserialize(salvaged[0]);
    if (!f || f->seq != 1)
        FAIL("salvaged frame was not seq 1");

    PASS();
    return true;
}

// F168 SACKED-BYTES DURABILITY: buffered out-of-order FILE frames (SACKed —
// the peer may retire them via the F163 sender salvage and NEVER resend) must
// be DELIVERED to the file layer when an epoch adoption discards the RX
// window, or the bytes are gone forever (F168: sender "complete" at 100%,
// receiver stranded at 50%, 640 s wedge).
bool test_epoch_adoption_salvages_buffered_file_frames() {
    TEST("epoch adoption delivers buffered FILE frames before discarding");

    ARQConfig config;
    config.window_size = 8;
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ rx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    rx.setCallsigns("RX1", "TX1");
    rx.setTransmitCallback([](const Bytes&) {});
    std::vector<Bytes> delivered;
    rx.setDataReceivedCallback([&](const Bytes& p) { delivered.push_back(p); });

    auto filePayload = [](uint8_t tag) {
        Bytes p{static_cast<uint8_t>(PayloadType::FILE_DATA), 0, 0, 0, tag};
        p.push_back(0xA0);
        p.push_back(tag);
        return p;
    };

    // In-order 0..2 (epoch 1, rebase on 0), then OUT-OF-ORDER 4 and 5 (hole at
    // 3) — 4/5 are buffered + SACKed, never delivered in-order.
    rx.beginGroupReceive();
    for (uint16_t q = 0; q <= 2; ++q) {
        rx.onFrameReceived(makeEpochData(q, filePayload(static_cast<uint8_t>(q)), 1, q == 0));
    }
    rx.onFrameReceived(makeEpochData(4, filePayload(4), 1, false));
    rx.onFrameReceived(makeEpochData(5, filePayload(5), 1, false));
    rx.endGroupReceiveAndAck();
    const size_t before = delivered.size();
    if (before != 3) FAIL("precondition: 3 in-order deliveries expected");

    // Epoch-2 rebase (sender rate-change abort rewound to 3): adoption discards
    // the buffered slots — the FILE payloads of 4 and 5 must be delivered first.
    rx.onFrameReceived(makeEpochData(3, filePayload(0x33), 2, /*rebase=*/true));

    bool got4 = false, got5 = false;
    for (size_t i = before; i < delivered.size(); ++i) {
        if (delivered[i].size() >= 5 && delivered[i][4] == 4) got4 = true;
        if (delivered[i].size() >= 5 && delivered[i][4] == 5) got5 = true;
    }
    if (!got4 || !got5)
        FAIL("buffered SACKed FILE frames were discarded without delivery "
             "(got4=" + std::to_string(got4) + " got5=" + std::to_string(got5) + ")");

    PASS();
    return true;
}

// Knob OFF (default): flags bits 6-7 / EPOCH_REBASE stay zero, SACK bitmaps carry
// no epoch bits, and a below-window TEXT re-send is dropped exactly as before —
// byte-identical wire behavior.
bool test_move_epoch_knob_off_byte_identical() {
    TEST("MOVE-EPOCH: knob off is byte-identical");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 100;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "0", 1);  // DEFAULT-ON since 2026-07-05; this
                                             // test pins the OPT-OUT wire identity
    SelectiveRepeatARQ tx(config);  // ctor-read: explicit =0 => OFF
    tx.setCallsigns("TX1", "RX1");
    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });
    tx.sendData(Bytes{0x01});
    tx.setCodeRate(CodeRate::R1_2);  // abort path runs, but no epoch machinery
    tx.sendData(Bytes{0x02});
    for (const auto& raw : transmitted) {
        auto f = v2::DataFrame::deserialize(raw);
        if (!f) FAIL("Frame did not parse");
        if ((f->flags & (v2::Flags::EPOCH_MASK | v2::Flags::EPOCH_REBASE)) != 0)
            FAIL("Knob off: epoch flag bits must stay zero");
    }
    if (tx.getStats().stale_epoch_acks_ignored != 0)
        FAIL("Knob off: stale-epoch counter must stay zero");

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    std::vector<Bytes> received;
    rx.setDataReceivedCallback([&](const Bytes& data) { received.push_back(data); });

    rx.onFrameReceived(v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0x00, 0x01}).serialize());
    if (received.size() != 1) FAIL("In-order frame should deliver");
    // Below-window TEXT under a "new era" the OFF receiver cannot see: dropped
    // (seq-keyed dedup), recovery SACK sent, no epoch echo in the bitmap.
    rx.onFrameReceived(makeEpochData(0, Bytes{0x00, 0x02}, 1, true));
    if (received.size() != 1)
        FAIL("Knob off: below-window TEXT must be dropped (legacy dedup)");
    if (channel.size() < 1)
        FAIL("Knob off: out-of-window recovery SACK expected");
    {
        auto ack = v2::ControlFrame::deserialize(channel.receive());
        if (!ack) FAIL("SACK did not parse");
        const uint32_t bitmap = v2::NackPayload::decode(ack->payload).cw_bitmap;
        if ((bitmap & 0xFFFF0000u) != 0)
            FAIL("Knob off: SACK bitmap must carry no epoch bits");
    }

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

bool test_timeout_repair_retransmits_only_missing_slot_and_resets_timer() {
    TEST("Timeout repair retransmits only missing slot and resets its timer");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 100;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    for (int i = 0; i < 3; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to send DATA seq=" + std::to_string(i));
    }
    if (transmitted.size() != 3)
        FAIL("Expected three initial DATA transmissions");

    // Cumulatively ACK seq=0 and SACK seq=2, leaving only seq=1 as a hole.
    auto sack_seq2 = makeSackAck(0, 0x02u);
    tx.onFrameReceived(sack_seq2.serialize());

    const size_t before_timeout_tx = transmitted.size();
    tx.tick(101);

    if (transmitted.size() != before_timeout_tx + 1)
        FAIL("Timeout should retransmit exactly one missing DATA frame");
    if (!expectDataSeq(transmitted.back(), 1, "timeout repair"))
        return false;

    auto stats = tx.getStats();
    if (stats.timeouts != 1 || stats.retransmissions != 1 ||
        stats.retransmissions_timeout != 1)
        FAIL("Expected one timeout retransmission, got timeouts=" +
             std::to_string(stats.timeouts) + " retransmissions=" +
             std::to_string(stats.retransmissions) + " timeout_retx=" +
             std::to_string(stats.retransmissions_timeout));

    tx.tick(99);
    if (transmitted.size() != before_timeout_tx + 1)
        FAIL("Timeout repair did not reset the slot timer to a full ACK timeout");

    tx.tick(1);
    if (transmitted.size() != before_timeout_tx + 2)
        FAIL("Slot did not retransmit again after the full reset timeout elapsed");
    if (!expectDataSeq(transmitted.back(), 1, "second timeout repair"))
        return false;

    PASS();
    return true;
}

bool test_timeout_window_retransmits_as_one_batch_when_callback_present() {
    TEST("Timeout window retransmits as one batch when callback present");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 100;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    std::vector<std::vector<Bytes>> batches;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });
    tx.setTransmitBatchCallback([&](const std::vector<Bytes>& frames) {
        batches.push_back(frames);
    });

    for (int i = 0; i < 4; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to send DATA seq=" + std::to_string(i));
    }
    if (transmitted.size() != 4)
        FAIL("Expected four initial DATA transmissions");
    if (!batches.empty())
        FAIL("Initial DATA submissions should not use timeout batch callback");

    tx.tick(101);

    if (transmitted.size() != 4)
        FAIL("Timeout batch should not also transmit frames individually");
    if (batches.size() != 1 || batches[0].size() != 4)
        FAIL("Expected one four-frame timeout retransmission batch");
    for (uint16_t seq = 0; seq < 4; ++seq) {
        if (!expectDataSeq(batches[0][seq], seq, "timeout batch"))
            return false;
    }

    auto stats = tx.getStats();
    if (stats.timeouts != 4 || stats.retransmissions_timeout != 4)
        FAIL("Expected four timeout retransmissions in stats");

    PASS();
    return true;
}

bool test_deferred_timeout_cancel_preserves_retry_and_karn_state() {
    TEST("Deferred timeout cancel preserves retry and Karn state");

    ARQConfig config;
    config.window_size = 1;
    config.ack_timeout_ms = 100;
    config.max_retries = 3;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    std::vector<Bytes> initial;
    std::vector<Bytes> intents;
    tx.setTransmitCallback([&](const Bytes& frame) { initial.push_back(frame); });
    tx.setTransmitBatchCallback(
        [&](const std::vector<Bytes>& frames) { intents = frames; });
    tx.setDeferredTimeoutCommitEnabled(true);

    if (!tx.sendData(Bytes{0x51}) || initial.size() != 1)
        FAIL("failed to seed one live DATA identity");
    const Bytes identity = initial.front();
    if (SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 0 ||
        !SelectiveRepeatARQTestAccess::rttSampleEligible(tx, identity))
        FAIL("fresh DATA must start at retry zero and remain RTT-sample eligible");

    tx.tick(100);
    auto after_intent = tx.getStats();
    if (intents.size() != 1 || after_intent.timeouts != 1 ||
        after_intent.retransmissions != 0 || after_intent.failed != 0)
        FAIL("timeout intent must count expiry but not retransmission/failure");
    if (SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 0 ||
        !SelectiveRepeatARQTestAccess::rttSampleEligible(tx, identity))
        FAIL("publishing an intent must not consume retry or Karn eligibility");

    if (tx.cancelDeferredTimeoutRetransmits(intents, true) != 1 ||
        !SelectiveRepeatARQTestAccess::transitionSuspended(tx, identity))
        FAIL("cancel should suspend the exact live timeout identity");
    intents.clear();
    tx.tick(1000);
    const auto while_suspended = tx.getStats();
    if (!intents.empty() || while_suspended.timeouts != 1 ||
        while_suspended.retransmissions != 0 || while_suspended.failed != 0)
        FAIL("a transition-suspended intent must stay inert across later ticks");
    if (SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 0 ||
        !SelectiveRepeatARQTestAccess::rttSampleEligible(tx, identity))
        FAIL("suspension must remain retry/Karn neutral");

    if (tx.resumeDeferredTimeoutRetransmits(25) != 1)
        FAIL("mode-resolution resume did not release the canceled identity");
    tx.tick(24);
    if (!intents.empty())
        FAIL("resumed identity fired before its explicit deadline");
    tx.tick(1);
    if (intents.size() != 1)
        FAIL("resumed identity did not publish a fresh timeout intent");
    auto committed = tx.commitDeferredTimeoutRetransmits(intents);
    if (committed.terminal_failure || committed.frames.size() != 1)
        FAIL("nonterminal commit should yield exactly one physical frame");
    const auto after_commit = tx.getStats();
    if (after_commit.retransmissions != 1 ||
        after_commit.retransmissions_timeout != 1 ||
        SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 1 ||
        SelectiveRepeatARQTestAccess::rttSampleEligible(tx, identity))
        FAIL("only commit may consume retry, timeout-retx stats, and Karn state");

    PASS();
    return true;
}

bool test_deferred_timeout_cancel_cannot_terminal_fail_without_commit() {
    TEST("Deferred timeout cancel cannot terminal-fail without commit");

    ARQConfig config;
    config.window_size = 1;
    config.ack_timeout_ms = 100;
    config.max_retries = 2;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    std::vector<Bytes> initial;
    std::vector<Bytes> intents;
    bool failed = false;
    tx.setTransmitCallback([&](const Bytes& frame) { initial.push_back(frame); });
    tx.setTransmitBatchCallback(
        [&](const std::vector<Bytes>& frames) { intents = frames; });
    tx.setDeferredTimeoutCommitEnabled(true);
    tx.setSendCompleteCallback([&](bool success) {
        if (!success) failed = true;
    });

    if (!tx.sendData(Bytes{0x61}) || initial.size() != 1)
        FAIL("failed to seed terminal-boundary fixture");
    const Bytes identity = initial.front();

    tx.tick(100);
    auto first_commit = tx.commitDeferredTimeoutRetransmits(intents);
    if (first_commit.terminal_failure || first_commit.frames.size() != 1 ||
        SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 1)
        FAIL("first timeout should commit the one allowed retransmission");

    intents.clear();
    tx.tick(100);
    if (intents.size() != 1 || failed || tx.getStats().failed != 0 ||
        SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 1)
        FAIL("max-boundary expiry must remain a reversible intent before commit");
    if (tx.cancelDeferredTimeoutRetransmits(intents, true) != 1)
        FAIL("failed to cancel max-boundary timeout intent");
    intents.clear();
    tx.tick(1000);
    if (!intents.empty() || failed || tx.getStats().failed != 0 ||
        SelectiveRepeatARQTestAccess::retryCount(tx, identity) != 1)
        FAIL("discarded max-boundary intent terminal-failed without egress commit");

    if (tx.resumeDeferredTimeoutRetransmits(1) != 1)
        FAIL("failed to resume max-boundary identity after mode resolution");
    tx.tick(1);
    auto terminal_commit = tx.commitDeferredTimeoutRetransmits(intents);
    if (!terminal_commit.terminal_failure || !terminal_commit.frames.empty() ||
        !failed || tx.getStats().failed != 1)
        FAIL("terminal failure must occur exactly at a committed timeout boundary");

    PASS();
    return true;
}

// §RETX-PACING (docs/RETX_PACING_DESIGN_2026_07_03.md §1.1): lastAckProgressFrames() is the
// zero-progress ROUND detector's ground truth — (frames retired by base advance) + (newly
// set SACK bits) for the most recent FRESH ack; −1 after consumption; dedup-suppressed
// duplicate acks must NEVER produce a fresh reading (phantom rounds).
bool test_ack_progress_accessor_counts_and_dedups() {
    TEST("lastAckProgressFrames counts base advance + new SACK bits; duplicates stay -1");

    ARQConfig config;
    config.window_size = 8;
    config.ack_timeout_ms = 10000;
    config.max_retries = 5;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    tx.setTransmitCallback([](const Bytes&) {});

    if (tx.lastAckProgressFrames() != -1)
        FAIL("initial progress sentinel should be -1 (no ack yet)");

    for (int i = 0; i < 4; i++) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(i)}))
            FAIL("Failed to send DATA seq=" + std::to_string(i));
    }

    // Fresh ack: cumulative through seq=1 (retires seq 0,1 -> base=2) + SACK bit1
    // (= seq 3 after the advance) -> progress = 2 + 1 = 3.
    tx.onFrameReceived(makeSackAck(1, 0x02u).serialize());
    if (tx.lastAckProgressFrames() != 3)
        FAIL("expected progress 3 (2 retired + 1 new SACK bit), got " +
             std::to_string(tx.lastAckProgressFrames()));

    // Round consumption re-arms the sentinel.
    tx.consumeAckProgress();
    if (tx.lastAckProgressFrames() != -1)
        FAIL("consumeAckProgress should reset the sentinel to -1");

    // The identical ack re-heard inside the dedup window is SUPPRESSED by the existing
    // ack-signature dedup -> it must NOT publish a fresh (phantom-round) reading.
    tx.onFrameReceived(makeSackAck(1, 0x02u).serialize());
    if (tx.lastAckProgressFrames() != -1)
        FAIL("duplicate ack must not publish a progress reading (phantom round)");

    // A fresh ZERO-progress ack (base-1 cumulative, no new SACK bit) publishes 0 —
    // the §1.1 zero-progress round signal.
    tx.onFrameReceived(makeSackAck(1, 0x00u).serialize());
    if (tx.lastAckProgressFrames() != 0)
        FAIL("fresh no-advance/no-new-bit ack should publish progress 0, got " +
             std::to_string(tx.lastAckProgressFrames()));

    PASS();
    return true;
}

// §RETX-PACING §1.3 trigger #2: deferPendingRetransmits(ms) extends every pending slot's
// RTO so tick() cannot blind-fire a timeout batch inside a trough-pacing hold; after the
// hold elapses the RTO fires normally. Acked/inactive slots are untouched (by inspection:
// the loop skips !active || acked).
bool test_defer_pending_retransmits_gates_slot_rto() {
    TEST("deferPendingRetransmits extends pending slot timers across the hold");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;
    config.max_retries = 10;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    if (!tx.sendData(Bytes{0xAA}))
        FAIL("Failed to send first DATA frame");
    if (!tx.sendData(Bytes{0xBB}))
        FAIL("Failed to send second DATA frame");
    transmitted.clear();

    tx.tick(900);  // 100 ms left on both slot timers
    if (!transmitted.empty())
        FAIL("no retransmit expected before the RTO");

    tx.deferPendingRetransmits(2000);  // the trough hold: timers now at 2100 ms

    tx.tick(1500);  // would have fired the original RTO 1.4 s ago without the deferral
    if (!transmitted.empty())
        FAIL("slot RTO must not blind-fire inside the pacing hold");

    tx.tick(700);  // 600 ms remained -> both slots expire now
    if (transmitted.size() != 2)
        FAIL("both pending frames should retransmit after the hold expires, got " +
             std::to_string(transmitted.size()));

    auto stats = tx.getStats();
    if (stats.timeouts != 2 || stats.retransmissions_timeout != 2)
        FAIL("expected exactly the two post-hold timeout retransmissions");

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

    // The last partial batch is intentionally ACKed by the delayed SACK timer.
    rx.tick(config.sack_delay_ms);

    if (completions != 10)
        FAIL("Expected 10 completions, got " + std::to_string(completions));

    // Verify payload integrity
    for (int i = 0; i < 10; i++) {
        if (received[i].size() != 1 || received[i][0] != i)
            FAIL("Payload mismatch at index " + std::to_string(i));
    }

    PASS();
    return true;
}

// ============================================================================
// ack_batch_size decoupling tests (Phase 1b)
// ============================================================================

bool test_ack_batch_threshold_independent() {
    TEST("ack_batch_size threshold fires independently of window_size");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;  // Large so timer doesn't interfere

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckBatchSize(2);  // ACK every 2 frames, not every 4

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // Feed 4 in-order frames; expect SACK after frames 2 and 4 = 2 SACKs total.
    // Without the decoupling (or with ack_batch_size=0), we'd get either zero
    // (timer didn't fire) or one (after frame 4).
    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 2)
        FAIL("Expected 2 SACKs at batch=2 over 4 frames, got " + std::to_string(channel.size()));

    PASS();
    return true;
}

bool test_ack_batch_out_of_order_safety_valve() {
    TEST("Out-of-order frame triggers immediate SACK regardless of batch state");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckBatchSize(4);  // High batch — without safety valve, no SACK until 4 frames

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // Feed seq=0 (in order, frames_since_ack=1), then seq=2 (hole at seq=1 →
    // out-of-order). Out-of-order MUST short-circuit the batch threshold and
    // fire SACK immediately.
    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0});
    rx.onFrameReceived(f0.serialize());
    if (channel.size() != 0)
        FAIL("Did not expect SACK after 1st in-order frame at batch=4, got " + std::to_string(channel.size()));

    auto f2 = v2::DataFrame::makeData("TX1", "RX1", 2, Bytes{2});
    rx.onFrameReceived(f2.serialize());
    if (channel.size() != 1)
        FAIL("Expected immediate SACK on out-of-order seq=2, got " + std::to_string(channel.size()));

    PASS();
    return true;
}

bool test_legacy_final_out_of_order_frame_sends_explicit_frame_nack() {
    TEST("legacy FINAL out-of-order frame sends explicit frame NACK plus SACK");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    f1.flags |= v2::Flags::FINAL;
    rx.onFrameReceived(f1.serialize());

    if (channel.size() != 2)
        FAIL("Expected frame NACK and SACK for FINAL out-of-order gap, got " +
             std::to_string(channel.size()));

    auto nack = v2::ControlFrame::deserialize(channel.receive());
    if (!nack || nack->type != v2::FrameType::NACK || nack->seq != 0)
        FAIL("First control should be explicit NACK for missing seq=0");
    if (v2::NackPayload::decode(nack->payload).cw_bitmap != 0)
        FAIL("Frame NACK should carry cw bitmap=0 for full-frame retransmit");

    auto sack = v2::ControlFrame::deserialize(channel.receive());
    if (!sack || sack->type != v2::FrameType::ACK || sack->seq != 0xFFFF)
        FAIL("Second control should be SACK at base-1 for the same hole");
    if (v2::NackPayload::decode(sack->payload).cw_bitmap != 0x2)
        FAIL("SACK bitmap should confirm seq=1 while seq=0 is missing");

    PASS();
    return true;
}

bool test_tone_sack_final_gap_uses_one_feedback_plane() {
    TEST("tone-SACK FINAL gap emits one tone response and no legacy frame NACK");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    // Any bytes reaching this callback become a classic OFDM control frame in the
    // production modem.  It must stay empty while the tone-SACK plane owns ACKs.
    ByteChannel legacy_control;
    rx.setTransmitCallback([&](const Bytes& data) { legacy_control.send(data); });

    struct ToneAck {
        uint16_t base = 0;
        uint32_t bitmap = 0;
        bool has_final = false;
        uint8_t move_epoch = 0;
    };
    std::vector<ToneAck> tone_acks;
    rx.setEmitToneBurstSackCallback(
        [&](uint16_t base, uint32_t bitmap, bool has_final, uint8_t move_epoch) {
            tone_acks.push_back({base, bitmap, has_final, move_epoch});
        });

    // Reproduce the file-tail shape from the rig at a minimal sequence number:
    // seq=0 is missing and the out-of-order FINAL seq=1 survives.  The group bracket
    // suppresses per-frame feedback until its physical end, exactly like
    // Connection::onBurstGroupReceived().
    rx.beginGroupReceive();
    auto final = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    final.flags |= v2::Flags::FINAL;
    rx.onFrameReceived(final.serialize());

    if (legacy_control.size() != 0)
        FAIL("tone-SACK path queued a legacy frame NACK inside the receive group");
    if (!tone_acks.empty())
        FAIL("group bracket emitted the tone SACK before the physical group ended");

    rx.endGroupReceiveAndAck();

    if (legacy_control.size() != 0)
        FAIL("tone-SACK tail gap must never queue a legacy OFDM control frame");
    if (tone_acks.size() != 1)
        FAIL("expected exactly one tone SACK at group end, got " +
             std::to_string(tone_acks.size()));
    if (tone_acks[0].base != 0xFFFF || tone_acks[0].bitmap != 0x2)
        FAIL("tone SACK did not report base-1 plus the surviving FINAL frame");
    if (tone_acks[0].has_final)
        FAIL("out-of-order FINAL must not claim final delivery before the hole closes");

    PASS();
    return true;
}

bool test_unready_tone_sack_transport_falls_back_to_legacy() {
    TEST("unready tone-SACK transport retains legacy tail-gap feedback");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    ByteChannel legacy_control;
    rx.setTransmitCallback([&](const Bytes& data) { legacy_control.send(data); });

    size_t tone_ack_count = 0;
    rx.setEmitToneBurstSackCallback(
        [&](uint16_t, uint32_t, bool, uint8_t) { ++tone_ack_count; });
    // The protocol callback exists, but the host has not supplied a physical tone
    // transmitter. This is Connection's construction-time state.
    rx.setToneBurstSackTransportReady(false);

    auto final = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    final.flags |= v2::Flags::FINAL;
    rx.onFrameReceived(final.serialize());

    if (tone_ack_count != 0)
        FAIL("unready tone transport must not consume/drop the feedback event");
    if (legacy_control.size() != 2)
        FAIL("expected legacy explicit NACK plus cumulative SACK fallback");

    auto explicit_nack = v2::ControlFrame::deserialize(legacy_control.receive());
    auto cumulative_sack = v2::ControlFrame::deserialize(legacy_control.receive());
    if (!explicit_nack || explicit_nack->type != v2::FrameType::NACK ||
        explicit_nack->seq != 0)
        FAIL("fallback explicit NACK did not identify missing base seq0");
    if (!cumulative_sack || cumulative_sack->type != v2::FrameType::ACK ||
        cumulative_sack->seq != 0xFFFF)
        FAIL("fallback cumulative SACK did not retain the base-1 identity");

    PASS();
    return true;
}

bool test_single_fixed_tail_retry_preserves_12cw_geometry() {
    TEST("single fixed tail retry preserves negotiated 12-CW frame geometry");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 100;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    tx.setCodeRate(CodeRate::R2_3);
    tx.setFixedFrameCodewords(12);

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    const size_t capacity = v2::getFixedFramePayloadCapacity(CodeRate::R2_3, 12);
    if (capacity != 629)
        FAIL("test precondition: R2/3 12-CW payload capacity should be 629 bytes");
    Bytes payload(capacity, 0x5A);
    if (!tx.sendFixedDataWithFlags(payload, v2::Flags::FINAL))
        FAIL("failed to send the fixed 12-CW tail frame");

    if (transmitted.size() != 1)
        FAIL("expected one initial fixed DATA frame");
    auto initial = v2::DataFrame::deserialize(transmitted[0]);
    if (!initial || initial->total_cw != 12 || initial->payload != payload)
        FAIL("initial tail frame did not preserve 12-CW geometry and full payload");

    tx.tick(101);
    if (transmitted.size() != 2)
        FAIL("single tail frame did not retransmit after its ACK timeout");
    auto retry = v2::DataFrame::deserialize(transmitted[1]);
    if (!retry || retry->seq != initial->seq || retry->total_cw != 12 ||
        retry->payload != payload)
        FAIL("tail retry changed sequence, payload, or negotiated 12-CW geometry");

    PASS();
    return true;
}

bool test_long_lifting_fixed_frame_preserves_full_payload() {
    TEST("fixed ARQ serializer preserves cw4/Z81 payload capacity and retry bytes");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 100;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    tx.setCodeRate(CodeRate::R2_3);
    tx.setFixedFrameGeometry(/*cw_count=*/4, /*lifting_z=*/81);

    const size_t capacity =
        v2::getFixedFramePayloadCapacityZ(CodeRate::R2_3, 4, 81);
    if (capacity != 629)
        FAIL("test precondition: R2/3 cw4/Z81 capacity should be 629 bytes");

    Bytes payload(capacity, 0xA5);
    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });
    if (!tx.sendFixedDataWithFlags(payload, v2::Flags::FINAL))
        FAIL("failed to send fixed cw4/Z81 DATA frame");

    if (transmitted.size() != 1)
        FAIL("expected one initial cw4/Z81 frame");
    auto initial = v2::DataFrame::deserialize(transmitted.front());
    if (!initial || initial->total_cw != 4 || initial->payload.size() != capacity ||
        initial->payload != payload)
        FAIL("ARQ serializer truncated or changed the cw4/Z81 payload");
    if (tx.getFixedFrameCodewords() != 4 || tx.getFixedFrameLiftingZ() != 81)
        FAIL("ARQ did not retain the complete fixed-frame geometry");

    tx.tick(101);
    if (transmitted.size() != 2 || transmitted[1] != transmitted[0])
        FAIL("cw4/Z81 retry did not preserve the exact serialized frame");

    PASS();
    return true;
}

bool test_lifting_only_geometry_change_aborts_old_window() {
    TEST("lifting-only fixed geometry change aborts the old ARQ window");

    ARQConfig config;
    config.window_size = 2;
    config.ack_timeout_ms = 100;
    config.max_retries = 4;

    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    SelectiveRepeatARQ tx(config);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    tx.setCallsigns("TX1", "RX1");
    tx.setCodeRate(CodeRate::R2_3);
    tx.setFixedFrameGeometry(/*cw_count=*/4, /*lifting_z=*/27);

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });
    if (!tx.sendFixedDataWithFlags(Bytes(100, 0x27), v2::Flags::NONE))
        FAIL("failed to seed the short-Z ARQ window");

    tx.setFixedFrameGeometry(/*cw_count=*/4, /*lifting_z=*/81);
    if (tx.getFixedFrameCodewords() != 4 || tx.getFixedFrameLiftingZ() != 81)
        FAIL("lifting-only regrid did not adopt cw4/Z81 atomically");
    if (tx.txMoveEpoch() != 1)
        FAIL("lifting-only regrid dropping live payload must bump move epoch");

    tx.tick(1000);
    if (transmitted.size() != 1)
        FAIL("old Z=27 identity retransmitted after the lifting-only regrid");
    if (!tx.sendFixedDataWithFlags(Bytes(629, 0x81), v2::Flags::FINAL))
        FAIL("failed to send a fresh long-Z identity after regrid");
    auto fresh = v2::DataFrame::deserialize(transmitted.back());
    if (!fresh || fresh->payload.size() != 629 || fresh->total_cw != 4)
        FAIL("post-regrid identity did not serialize at cw4/Z81");

    PASS();
    return true;
}

bool test_transmit_round_timeout_shrinks_live_tail_after_rtt_sample() {
    TEST("actual one-frame round timeout overrides adaptive/full-burst state");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    // Establish RTT-estimator state first. setAckTimeout() intentionally preserves
    // that estimator, so this proves the per-round override is a distinct mechanism
    // and still wins after a real sample exists.
    if (!tx.sendData(Bytes{0x11}))
        FAIL("failed to send RTT-training frame");
    tx.tick(200);
    auto first_ack = v2::ControlFrame::makeAck("RX1", "TX1", 0);
    tx.onFrameReceived(first_ack.serialize());

    if (!tx.sendData(Bytes{0x22}))
        FAIL("failed to send tail frame");
    if (transmitted.size() != 2)
        FAIL("test setup should have exactly two first transmissions");

    const size_t rearmed =
        tx.rearmTransmittedDataFrames({transmitted.back()}, 100);
    if (rearmed != 1)
        FAIL("exact transmitted-frame identity did not match the live tail slot");
    if (tx.getAckTimeout() != 1000)
        FAIL("per-round timeout must not overwrite the mode-wide timeout");

    tx.tick(99);
    if (transmitted.size() != 2)
        FAIL("tail retransmitted before its exact one-frame deadline");
    tx.tick(1);
    if (transmitted.size() != 3)
        FAIL("tail did not retransmit at its exact one-frame deadline");

    auto retry = v2::DataFrame::deserialize(transmitted.back());
    if (!retry || retry->seq != 1 || retry->payload != Bytes{0x22})
        FAIL("round-timeout retry changed the live tail frame");

    PASS();
    return true;
}

bool test_transmitted_frame_timeout_does_not_rearm_unsent_hole() {
    TEST("physical-round timeout suspends unsent holes and ignores padding aliases");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    if (!tx.sendData(Bytes{0x10}) || !tx.sendData(Bytes{0x20}))
        FAIL("failed to seed two live slots");

    // Put both original timers near expiry, then commit only seq0 to the new physical
    // round. A valid DATA-shaped pad/stale alias deliberately reuses seq1 but not its
    // complete serialized identity, so it must affect geometry only and never match.
    tx.tick(900);
    auto pad_alias =
        v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{0xEE}).serialize();
    if (tx.rearmTransmittedDataFrames({transmitted[0], pad_alias}, 100) != 1)
        FAIL("failed to rearm the one physically selected frame");

    tx.tick(100);
    if (transmitted.size() != 3)
        FAIL("near-expiry unsent hole launched a colliding second retry");
    auto retry = v2::DataFrame::deserialize(transmitted.back());
    if (!retry || retry->seq != 0)
        FAIL("unselected live slot was incorrectly rearmed/retried");

    // Once seq1 is selected for a later physical round, it becomes eligible and its
    // exact one-frame timer runs normally.
    if (tx.rearmTransmittedDataFrames({transmitted[1]}, 50) != 1)
        FAIL("later physical round did not unsuspend the remaining hole");
    tx.tick(49);
    if (transmitted.size() != 3)
        FAIL("remaining hole retried before its later-round deadline");
    tx.tick(1);
    if (transmitted.size() != 4)
        FAIL("remaining hole did not retry after becoming physically eligible");
    retry = v2::DataFrame::deserialize(transmitted.back());
    if (!retry || retry->seq != 1)
        FAIL("later physical round retried the wrong identity");

    PASS();
    return true;
}

bool test_transmitted_round_suspends_wrapped_window_slots() {
    TEST("physical-round suspension covers nonzero and wrapped TX-slot indices");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });

    // Advance the logical base to 14. seqToSlot uses the fixed 16-entry ring, so the
    // next live window occupies physical indices 14,15,0,1 rather than [0,4).
    for (uint16_t seq = 0; seq < 14; ++seq) {
        if (!tx.sendData(Bytes{static_cast<uint8_t>(seq)}))
            FAIL("failed to advance the TX base for wrapped-slot setup");
        auto ack = v2::ControlFrame::makeAck("RX1", "TX1", seq);
        tx.onFrameReceived(ack.serialize());
    }
    if (!tx.sendData(Bytes{0xA0}) || !tx.sendData(Bytes{0xB0}))
        FAIL("failed to seed wrapped live slots seq14/15");
    const Bytes seq14_frame = transmitted[14];

    tx.tick(900);
    if (tx.rearmTransmittedDataFrames({seq14_frame}, 100) != 1)
        FAIL("failed to commit wrapped seq14 physical round");
    tx.tick(100);

    if (transmitted.size() != 17)
        FAIL("wrapped unsent seq15 escaped suspension and collided with seq14 retry");
    auto retry = v2::DataFrame::deserialize(transmitted.back());
    if (!retry || retry->seq != 14)
        FAIL("wrapped-slot round retried the wrong physical identity");

    PASS();
    return true;
}

bool test_waiting_rebase_unsuspends_near_expiry_base() {
    TEST("WAITING-REBASE voice unsuspends a near-expiry physical base slot");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;
    config.max_retries = 4;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");
    std::vector<Bytes> transmitted;
    tx.setTransmitCallback([&](const Bytes& data) { transmitted.push_back(data); });
    if (!tx.sendData(Bytes{0x10}) || !tx.sendData(Bytes{0x20}))
        FAIL("failed to seed rebase suspension setup");

    tx.tick(900);  // base seq0 has only 100 ms left
    if (tx.rearmTransmittedDataFrames({transmitted[1]}, 500) != 1)
        FAIL("failed to suspend base behind the seq1 physical round");
    tx.expireBaseSlotTimerForRebase();
    tx.tick(1);

    if (transmitted.size() != 3)
        FAIL("WAITING-REBASE voice left the near-expiry base suspended");
    auto retry = v2::DataFrame::deserialize(transmitted.back());
    if (!retry || retry->seq != 0)
        FAIL("WAITING-REBASE voice did not force the era-base identity");

    PASS();
    return true;
}

bool test_more_frag_out_of_order_uses_sack_without_explicit_frame_nack() {
    TEST("MORE_FRAG out-of-order frame keeps guarded SACK-only path");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    f1.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f1.serialize());

    if (channel.size() != 1)
        FAIL("Expected only SACK for MORE_FRAG out-of-order gap, got " +
             std::to_string(channel.size()));

    auto sack = v2::ControlFrame::deserialize(channel.receive());
    if (!sack || sack->type != v2::FrameType::ACK)
        FAIL("MORE_FRAG out-of-order control should remain a SACK");

    PASS();
    return true;
}

bool test_explicit_frame_nack_retransmits_before_long_rto() {
    TEST("explicit frame NACK retransmits before long RTO");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 15000;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    if (!tx.sendDataWithFlags(Bytes{0}, v2::Flags::FINAL))
        FAIL("failed to send seq=0");
    if (!tx.sendDataWithFlags(Bytes{1}, v2::Flags::FINAL))
        FAIL("failed to send seq=1");

    if (channel.size() != 2)
        FAIL("expected two original DATA frames");
    (void)channel.receive();
    (void)channel.receive();

    auto nack = v2::ControlFrame::makeNack("RX1", "TX1", 0, 0);
    tx.onFrameReceived(nack.serialize());

    if (channel.size() != 1)
        FAIL("explicit NACK should immediately enqueue one retransmission");
    if (!expectDataSeq(channel.receive(), 0, "explicit NACK retx"))
        return false;

    auto stats = tx.getStats();
    if (stats.timeouts != 0)
        FAIL("NACK retransmit should occur before timeout accounting");
    if (stats.retransmissions_nack != 1 || stats.retransmissions != 1)
        FAIL("Expected exactly one NACK retransmission");
    if (stats.retransmissions_fast_hole != 0 || stats.retransmissions_hole_probe != 0)
        FAIL("Explicit NACK should not count as fast_hole or hole_probe");

    PASS();
    return true;
}

bool test_ack_batch_setter_clamping() {
    TEST("setWindowSize() clamps ack_batch_size when shrinking below");

    ARQConfig config;
    config.window_size = 8;

    SelectiveRepeatARQ rx(config);
    rx.setAckBatchSize(6);

    if (rx.getAckBatchSize() != 6)
        FAIL("Expected ack_batch_size=6 after setAckBatchSize(6), got " + std::to_string(rx.getAckBatchSize()));

    // Shrink window below batch_size. Batch should clamp down.
    rx.setWindowSize(4);

    if (rx.getAckBatchSize() != 4)
        FAIL("Expected ack_batch_size clamped to 4 after window shrink, got " + std::to_string(rx.getAckBatchSize()));

    // Default (batch_size = 0) should NOT be touched by setWindowSize — sentinel
    // 0 means "track window_size implicitly".
    rx.setAckBatchSize(0);
    rx.setWindowSize(2);
    if (rx.getAckBatchSize() != 0)
        FAIL("Expected ack_batch_size=0 sentinel preserved after window change");

    PASS();
    return true;
}

bool test_ack_batch_default_matches_window() {
    TEST("ack_batch_size=0 (default) tracks window_size — bit-identical to prior code");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;
    // ack_batch_size = 0 by default

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // Feed exactly window_size=4 in-order frames. Threshold path should fire 1 SACK.
    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 1)
        FAIL("Expected 1 SACK at threshold (default = window_size = 4), got " + std::to_string(channel.size()));

    PASS();
    return true;
}

bool test_ack_batch_defers_more_frag_until_tail() {
    TEST("ack_batch_size threshold defers MORE_FRAG frames until stream tail");

    ARQConfig config;
    config.window_size = 8;
    config.sack_delay_ms = 5000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckBatchSize(4);
    rx.setSackDelayShort(50);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        frame.flags |= v2::Flags::MORE_FRAG;
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 0)
        FAIL("MORE_FRAG batch threshold sent a mid-stream SACK");

    rx.tick(1000);
    if (channel.size() != 0)
        FAIL("Long in-stream SACK timer fired too early");

    auto tail = v2::DataFrame::makeData("TX1", "RX1", 4, Bytes{4});
    rx.onFrameReceived(tail.serialize());
    if (channel.size() != 1)
        FAIL("Stream-tail threshold did not send immediate SACK");

    auto ack = v2::ControlFrame::deserialize(channel.receive());
    if (!ack || ack->type != v2::FrameType::ACK || ack->seq != 4)
        FAIL("Tail SACK did not cumulatively acknowledge seq=4");

    PASS();
    return true;
}

bool test_ack_batch_can_fire_through_more_frag_for_mc_dpsk() {
    TEST("ack_batch_size threshold can fire through MORE_FRAG when enabled");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckBatchThroughMoreFrag(true);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        frame.flags |= v2::Flags::MORE_FRAG;
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 1)
        FAIL("Expected threshold SACK through MORE_FRAG, got " + std::to_string(channel.size()));

    auto ack = v2::ControlFrame::deserialize(channel.receive());
    if (!ack || ack->type != v2::FrameType::ACK || ack->seq != 3)
        FAIL("Threshold SACK through MORE_FRAG did not cumulatively acknowledge seq=3");

    PASS();
    return true;
}

bool test_wide_sack_bitmap_serializes_beyond_8_frames() {
    TEST("SACK bitmap preserves bits beyond 8-frame legacy window");

    ARQConfig config;
    config.window_size = 12;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // An out-of-order frame at seq=10 must be represented as bit 10 in the
    // SACK bitmap. The old 8-bit path truncated this information.
    auto f10 = v2::DataFrame::makeData("TX1", "RX1", 10, Bytes{10});
    rx.onFrameReceived(f10.serialize());

    if (channel.size() != 1)
        FAIL("Expected one immediate SACK for out-of-order seq=10");

    auto ctrl = v2::ControlFrame::deserialize(channel.receive());
    if (!ctrl)
        FAIL("Serialized SACK did not parse as a control frame");

    auto np = v2::NackPayload::decode(ctrl->payload);
    const uint32_t expected = 1u << 10;
    if (np.cw_bitmap != expected)
        FAIL("Expected SACK bitmap 0x" + std::to_string(expected) +
             ", got " + std::to_string(np.cw_bitmap));

    PASS();
    return true;
}

bool test_cumulative_ack_repeats_when_enabled() {
    TEST("cumulative ACK repeats when ack_repeat_count > 1");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 10000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckRepeatCount(2);
    rx.setAckRepeatDelay(100);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        if (i == 3) {
            frame.flags |= v2::Flags::FINAL;
        }
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 1)
        FAIL("Expected primary cumulative ACK only before repeat timer");

    auto primary = v2::ControlFrame::deserialize(channel.receive());
    if (!primary || primary->seq != 3)
        FAIL("Primary cumulative ACK did not acknowledge seq=3");
    if (v2::NackPayload::decode(primary->payload).cw_bitmap != 0)
        FAIL("Expected primary cumulative ACK bitmap=0");

    rx.tick(99);
    if (channel.size() != 0)
        FAIL("Cumulative ACK repeat fired before configured delay");

    rx.tick(40);
    if (channel.size() != 1)
        FAIL("Expected one delayed cumulative ACK repeat");

    auto repeat = v2::ControlFrame::deserialize(channel.receive());
    if (!repeat || repeat->seq != 3)
        FAIL("Delayed cumulative ACK repeat did not acknowledge seq=3");
    if (v2::NackPayload::decode(repeat->payload).cw_bitmap != 0)
        FAIL("Expected delayed cumulative ACK repeat bitmap=0");

    auto stats = rx.getStats();
    if (stats.sacks_sent != 1)
        FAIL("ACK repeat should not increment sacks_sent");
    if (stats.acks_sent != 2)
        FAIL("Expected acks_sent=2 including repeat, got " + std::to_string(stats.acks_sent));

    PASS();
    return true;
}

bool test_non_final_ack_repeat_waits_past_peer_burst_guard() {
    TEST("non-final ACK repeat waits past peer burst guard");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckRepeatCount(2);
    rx.setAckRepeatDelay(100);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 1)
        FAIL("Expected primary ACK only before guarded repeat timer");

    auto primary = v2::ControlFrame::deserialize(channel.receive());
    if (!primary || primary->seq != 3)
        FAIL("Primary non-final ACK did not acknowledge seq=3");

    rx.tick(500);
    if (channel.size() != 0)
        FAIL("Non-final ACK repeat fired before peer burst guard elapsed");

    rx.tick(200);
    if (channel.size() != 1)
        FAIL("Expected one delayed non-final ACK repeat after peer burst guard");

    auto repeat = v2::ControlFrame::deserialize(channel.receive());
    if (!repeat || repeat->seq != 3)
        FAIL("Guarded ACK repeat did not acknowledge seq=3");

    PASS();
    return true;
}

bool test_ack_repeat_peer_guard_can_be_zero_after_physical_hold() {
    TEST("ACK repeat peer guard can be zero after physical SACK hold");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckRepeatCount(2);
    rx.setAckRepeatDelay(100);
    rx.setAckRepeatPeerBurstGuardMs(0);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        rx.onFrameReceived(frame.serialize());
    }

    if (channel.size() != 1)
        FAIL("Expected primary ACK only before unguarded repeat timer");

    auto primary = v2::ControlFrame::deserialize(channel.receive());
    if (!primary || primary->seq != 3)
        FAIL("Primary non-final ACK did not acknowledge seq=3");

    rx.tick(99);
    if (channel.size() != 0)
        FAIL("Unguarded ACK repeat fired before configured delay");

    rx.tick(40);
    if (channel.size() != 1)
        FAIL("Expected one delayed ACK repeat without re-applying the physical SACK hold");

    auto repeat = v2::ControlFrame::deserialize(channel.receive());
    if (!repeat || repeat->seq != 3)
        FAIL("Unguarded ACK repeat did not acknowledge seq=3");

    PASS();
    return true;
}

bool test_hole_sack_repeat_stays_prompt() {
    TEST("hole-bearing SACK repeat stays prompt");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckRepeatCount(2);
    rx.setAckRepeatDelay(100);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto frame = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    rx.onFrameReceived(frame.serialize());

    if (channel.size() != 1)
        FAIL("Expected primary out-of-order SACK before repeat timer");

    auto primary = v2::ControlFrame::deserialize(channel.receive());
    if (!primary)
        FAIL("Primary SACK did not parse");
    if (v2::NackPayload::decode(primary->payload).cw_bitmap == 0)
        FAIL("Expected primary SACK to carry a hole bitmap");

    rx.tick(99);
    if (channel.size() != 0)
        FAIL("Hole-bearing SACK repeat fired before configured delay");

    rx.tick(40);
    if (channel.size() != 1)
        FAIL("Hole-bearing SACK repeat was incorrectly held by peer burst guard");

    auto repeat = v2::ControlFrame::deserialize(channel.receive());
    if (!repeat)
        FAIL("Repeated SACK did not parse");
    if (v2::NackPayload::decode(repeat->payload).cw_bitmap == 0)
        FAIL("Expected repeated SACK to carry the hole bitmap");

    PASS();
    return true;
}

bool test_cumulative_ack_repeat_coalesces_superseded_state() {
    TEST("new cumulative ACK coalesces superseded pending repeat");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setAckRepeatCount(2);
    rx.setAckRepeatDelay(100);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    for (int i = 0; i < 4; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        rx.onFrameReceived(frame.serialize());
    }
    if (channel.size() != 1)
        FAIL("Expected first primary ACK");

    for (int i = 4; i < 8; i++) {
        auto frame = v2::DataFrame::makeData("TX1", "RX1", i, Bytes{static_cast<uint8_t>(i)});
        rx.onFrameReceived(frame.serialize());
    }
    if (channel.size() != 2)
        FAIL("Expected second primary ACK before repeat timers");

    auto first = v2::ControlFrame::deserialize(channel.receive());
    auto second = v2::ControlFrame::deserialize(channel.receive());
    if (!first || first->seq != 3 || !second || second->seq != 7)
        FAIL("Expected primary ACK sequence 3 then 7");

    rx.tick(700);
    if (channel.size() != 1)
        FAIL("Expected only the latest cumulative ACK repeat after coalescing");

    auto repeat = v2::ControlFrame::deserialize(channel.receive());
    if (!repeat || repeat->seq != 7)
        FAIL("Expected repeated ACK for latest seq=7, not superseded seq=3");

    auto stats = rx.getStats();
    if (stats.ack_repeat_jobs_coalesced != 1)
        FAIL("Expected one superseded repeat coalesced, got " +
             std::to_string(stats.ack_repeat_jobs_coalesced));

    PASS();
    return true;
}

// ============================================================================
// Stream-aware SACK timer tests (post-Phase 3 plan)
// ============================================================================

bool test_sack_timer_final_short_collapses_long() {
    TEST("FINAL frame collapses long timer to short via std::min");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;
    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setSackDelayShort(50);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // Frame 0 with MORE_FRAG=1 → timer arms at 500ms
    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0});
    f0.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f0.serialize());
    rx.tick(60);  // 440ms remaining
    if (channel.size() != 0)
        FAIL("Long timer fired prematurely after 60ms");

    // Frame 1 with FINAL → pick_ms=50, std::min(440, 50) = 50
    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    f1.flags |= v2::Flags::FINAL;
    rx.onFrameReceived(f1.serialize());
    rx.tick(60);  // 50 - 60 ≤ 0 → fires
    if (channel.size() != 1)
        FAIL("Short-collapsed timer did not fire after 60ms tick (got " +
             std::to_string(channel.size()) + " SACKs)");

    PASS();
    return true;
}

bool test_sack_timer_message_boundary_uses_long_without_final() {
    TEST("MORE_FRAG=0 message boundary uses long timer unless FINAL is set");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;
    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setSackDelayShort(50);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0});
    f0.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f0.serialize());
    rx.tick(60);  // 440ms remaining on the long timer

    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    // No MORE_FRAG, but also no FINAL: this can be an ordinary message
    // boundary inside a multi-message physical burst.
    rx.onFrameReceived(f1.serialize());
    rx.tick(60);
    if (channel.size() != 0)
        FAIL("Non-FINAL message boundary incorrectly used short SACK timer");

    rx.tick(400);
    if (channel.size() != 1)
        FAIL("Long SACK timer did not fire for non-FINAL message boundary");

    PASS();
    return true;
}

bool test_out_of_order_sack_can_defer_until_physical_hold() {
    TEST("out-of-order SACK can defer until physical hold clears");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;
    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setImmediateOutOfOrderSackEnabled(false);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    f1.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f1.serialize());
    if (channel.size() != 0)
        FAIL("Out-of-order MORE_FRAG frame sent immediate SACK despite physical hold");

    rx.tick(499);
    if (channel.size() != 0)
        FAIL("Deferred out-of-order SACK fired before physical hold");

    rx.tick(1);
    if (channel.size() != 1)
        FAIL("Deferred out-of-order SACK did not fire after physical hold");

    auto sack = v2::ControlFrame::deserialize(channel.receive());
    if (!sack || sack->seq != 65535)
        FAIL("Deferred out-of-order SACK did not report base-1 for missing seq=0");
    if (v2::NackPayload::decode(sack->payload).cw_bitmap == 0)
        FAIL("Deferred out-of-order SACK should carry a hole bitmap");

    PASS();
    return true;
}

bool test_sack_timer_more_frag_does_not_extend() {
    TEST("MORE_FRAG=1 subsequent frames do not extend long timer");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;
    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setSackDelayShort(50);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0});
    f0.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f0.serialize());
    rx.tick(450);  // ~50ms remaining

    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    f1.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f1.serialize());
    // pick_ms = 500, but std::min(50, 500) keeps 50 — does NOT extend
    rx.tick(60);
    if (channel.size() != 1)
        FAIL("Timer did not fire on schedule with subsequent MORE_FRAG=1 frame (extended unexpectedly?)");

    PASS();
    return true;
}

bool test_sack_timer_slides_on_more_frag_when_enabled() {
    TEST("MORE_FRAG=1 subsequent frames slide quiet SACK timer when enabled");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 500;
    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    rx.setSackDelayShort(50);
    rx.setSackDelaySlidesOnData(true);

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0});
    f0.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f0.serialize());
    rx.tick(450);  // ~50ms remaining on the first quiet timer

    auto f1 = v2::DataFrame::makeData("TX1", "RX1", 1, Bytes{1});
    f1.flags |= v2::Flags::MORE_FRAG;
    rx.onFrameReceived(f1.serialize());
    rx.tick(60);
    if (channel.size() != 0)
        FAIL("Sliding quiet timer fired before the newest MORE_FRAG frame's delay elapsed");

    rx.tick(450);
    if (channel.size() != 1)
        FAIL("Sliding quiet timer did not fire after the re-armed SACK delay");

    auto ack = v2::ControlFrame::deserialize(channel.receive());
    if (!ack || ack->type != v2::FrameType::ACK || ack->seq != 1)
        FAIL("Sliding quiet timer SACK did not cumulatively acknowledge the newest frame");

    PASS();
    return true;
}

bool test_sack_delay_short_zero_sentinel_preserves_legacy() {
    TEST("sack_delay_short=0 sentinel uses sack_delay_ms even for FINAL");

    ARQConfig config;
    config.window_size = 4;
    config.sack_delay_ms = 120;
    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    // Do NOT call setSackDelayShort — sack_delay_short_ms_ stays 0 (sentinel)

    ByteChannel channel;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    // FINAL frame — under sentinel should arm at 120ms (sack_delay_ms), not
    // at 0 or any short-override value.
    auto f0 = v2::DataFrame::makeData("TX1", "RX1", 0, Bytes{0});
    f0.flags |= v2::Flags::FINAL;
    rx.onFrameReceived(f0.serialize());
    rx.tick(50);
    if (channel.size() != 0)
        FAIL("Timer fired too early under sentinel (50ms < 120ms expected)");
    rx.tick(80);
    if (channel.size() != 1)
        FAIL("Timer did not fire after 130ms total under sentinel");

    PASS();
    return true;
}

bool test_partial_mc_dpsk_cw_nack_and_merge() {
    TEST("Partial MC-DPSK CW state sends NACK and merges later CWs");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");

    ByteChannel channel;
    Bytes delivered;
    rx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });
    rx.setDataReceivedCallback([&](const Bytes& data) { delivered = data; });

    Bytes payload(50, 0x42);
    auto frame = v2::DataFrame::makeData("TX1", "RX1", 0, payload, CodeRate::R1_4);
    auto serialized = frame.serialize();
    auto cws = v2::splitIntoCodewords(serialized);
    if (cws.size() != 4) {
        FAIL("test fixture expected a 4-CW MC-DPSK frame");
    }

    v2::PartialFrameCodewords first;
    first.type = frame.type;
    first.flags = frame.flags;
    first.seq = frame.seq;
    first.src_hash = frame.src_hash;
    first.dst_hash = frame.dst_hash;
    first.total_cw = static_cast<uint8_t>(cws.size());
    first.data.assign(cws.size(), Bytes{});
    first.decoded_bitmap = (1u << 0) | (1u << 2);
    first.data[0] = cws[0];
    first.data[2] = cws[2];

    rx.onPartialFrame(first);

    if (channel.size() != 1) {
        FAIL("partial frame should emit one CW_NACK");
    }
    auto nack = v2::ControlFrame::deserialize(channel.receive());
    if (!nack || nack->type != v2::FrameType::NACK) {
        FAIL("partial frame response was not a NACK");
    }
    auto np = v2::NackPayload::decode(nack->payload);
    if (np.frame_seq != 0 || np.cw_bitmap != 0x0A) {
        FAIL("CW_NACK bitmap should request CW1 and CW3");
    }
    if (!delivered.empty()) {
        FAIL("partial frame delivered payload before all CWs arrived");
    }

    v2::PartialFrameCodewords second = first;
    second.decoded_bitmap = (1u << 0) | (1u << 1) | (1u << 3);
    second.data[1] = cws[1];
    second.data[3] = cws[3];

    rx.onPartialFrame(second);

    if (delivered != payload) {
        FAIL("merged partial CWs did not deliver original payload");
    }
    auto stats = rx.getStats();
    if (stats.partial_frames_completed != 1 || stats.cw_nacks_sent != 1) {
        FAIL("partial completion/NACK stats not updated");
    }

    PASS();
    return true;
}

bool test_cw_nack_triggers_compact_data_repair_phase2() {
    TEST("CW_NACK triggers compact DATA_REPAIR in Phase 2");

    ARQConfig config;
    config.window_size = 4;
    config.ack_timeout_ms = 1000;

    SelectiveRepeatARQ tx(config);
    tx.setCallsigns("TX1", "RX1");

    ByteChannel channel;
    tx.setTransmitCallback([&](const Bytes& data) { channel.send(data); });

    if (!tx.sendData(Bytes(50, 0x24))) {
        FAIL("initial send failed");
    }
    if (channel.size() != 1) {
        FAIL("initial DATA was not transmitted");
    }
    Bytes full_frame = channel.receive();
    auto original = v2::DataFrame::deserialize(full_frame);
    if (!original) {
        FAIL("initial DATA frame did not deserialize");
    }
    auto original_cws = v2::splitIntoCodewords(full_frame);
    if (original_cws.size() != 4) {
        FAIL("test fixture expected a 4-CW initial frame");
    }

    auto nack = v2::ControlFrame::makeNack("RX1", "TX1", 0, 0x0A);
    tx.onFrameReceived(nack.serialize());

    if (channel.size() != 1) {
        FAIL("CW_NACK should emit one DATA_REPAIR frame");
    }
    auto repair = v2::DataRepairFrame::deserialize(channel.receive());
    if (!repair) {
        FAIL("CW_NACK response was not a valid DATA_REPAIR frame");
    }
    if (repair->target_seq != 0 || repair->repair_bitmap != 0x0A ||
        repair->repair_count != 2 || repair->repair_codewords.size() != 2) {
        FAIL("DATA_REPAIR header did not carry requested CW bitmap/count");
    }
    if (repair->repair_codewords[0] != original_cws[1] ||
        repair->repair_codewords[1] != original_cws[3]) {
        FAIL("DATA_REPAIR did not carry the original missing info CWs");
    }
    auto stats = tx.getStats();
    if (stats.cw_nacks_received != 1 || stats.retransmissions_nack != 1 ||
        stats.data_repairs_sent != 1 || stats.data_repair_cws_sent != 2) {
        FAIL("DATA_REPAIR retransmission stats not updated");
    }
    tx.onFrameReceived(nack.serialize());
    if (channel.size() != 0) {
        FAIL("duplicate CW_NACK should not emit a second DATA_REPAIR or full DATA frame");
    }
    tx.tick(999);
    if (channel.size() != 0) {
        FAIL("DATA_REPAIR guard should suppress shadow full-frame retx before cooldown");
    }
    tx.tick(10000);
    if (channel.size() != 1) {
        FAIL("DATA_REPAIR guard expiry should permit full-frame fallback");
    }
    auto fallback = v2::DataFrame::deserialize(channel.receive());
    if (!fallback || fallback->seq != original->seq || fallback->payload != original->payload) {
        FAIL("full-frame fallback after DATA_REPAIR guard was invalid");
    }

    SelectiveRepeatARQ rx(config);
    rx.setCallsigns("RX1", "TX1");
    Bytes delivered;
    rx.setDataReceivedCallback([&](const Bytes& data) { delivered = data; });
    ByteChannel rx_channel;
    rx.setTransmitCallback([&](const Bytes& data) { rx_channel.send(data); });

    v2::PartialFrameCodewords first;
    first.type = original->type;
    first.flags = original->flags;
    first.seq = original->seq;
    first.src_hash = original->src_hash;
    first.dst_hash = original->dst_hash;
    first.total_cw = static_cast<uint8_t>(original_cws.size());
    first.data.assign(original_cws.size(), Bytes{});
    first.decoded_bitmap = (1u << 0) | (1u << 2);
    first.data[0] = original_cws[0];
    first.data[2] = original_cws[2];
    rx.onPartialFrame(first);
    if (rx_channel.size() != 1) {
        FAIL("receiver partial state should emit one CW_NACK before repair");
    }
    (void)rx_channel.receive();

    rx.onFrameReceived(repair->serialize());
    if (delivered != Bytes(50, 0x24)) {
        FAIL("DATA_REPAIR did not merge and deliver original payload");
    }
    auto rx_stats = rx.getStats();
    if (rx_stats.data_repairs_received != 1 || rx_stats.data_repair_cws_merged != 2 ||
        rx_stats.partial_frames_completed != 1) {
        FAIL("DATA_REPAIR receive/merge stats not updated");
    }

    PASS();
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=== Selective Repeat ARQ Test Suite (v2) ===\n\n";

    // Run new Phase 1b + stream-aware tests FIRST so they're observable even
    // if a later existing test hangs (test_full_exchange has a known hang
    // issue, unrelated to these changes).
    std::cout << "In-flight timeout:\n";
    test_retransmit_timeout_follows_inflight_table();
    test_heard_misses_count_ack_rounds_not_timeouts();
    test_measured_burst_timeout();
    test_measured_burst_timeout_survives_constellation_change();

    std::cout << "ack_batch_size Decoupling Tests (Phase 1b):\n";
    test_ack_batch_threshold_independent();
    test_ack_batch_out_of_order_safety_valve();
    test_legacy_final_out_of_order_frame_sends_explicit_frame_nack();
    test_tone_sack_final_gap_uses_one_feedback_plane();
    test_unready_tone_sack_transport_falls_back_to_legacy();
    test_single_fixed_tail_retry_preserves_12cw_geometry();
    test_long_lifting_fixed_frame_preserves_full_payload();
    test_lifting_only_geometry_change_aborts_old_window();
    test_transmit_round_timeout_shrinks_live_tail_after_rtt_sample();
    test_transmitted_frame_timeout_does_not_rearm_unsent_hole();
    test_transmitted_round_suspends_wrapped_window_slots();
    test_waiting_rebase_unsuspends_near_expiry_base();
    test_more_frag_out_of_order_uses_sack_without_explicit_frame_nack();
    test_explicit_frame_nack_retransmits_before_long_rto();
    test_ack_batch_setter_clamping();
    test_ack_batch_default_matches_window();
    test_ack_batch_defers_more_frag_until_tail();
    test_ack_batch_can_fire_through_more_frag_for_mc_dpsk();
    test_wide_sack_bitmap_serializes_beyond_8_frames();
    test_cumulative_ack_repeats_when_enabled();
    test_non_final_ack_repeat_waits_past_peer_burst_guard();
    test_ack_repeat_peer_guard_can_be_zero_after_physical_hold();
    test_hole_sack_repeat_stays_prompt();
    test_cumulative_ack_repeat_coalesces_superseded_state();

    std::cout << "\nStream-Aware SACK Timer Tests:\n";
    test_sack_timer_final_short_collapses_long();
    test_sack_timer_message_boundary_uses_long_without_final();
    test_out_of_order_sack_can_defer_until_physical_hold();
    test_sack_timer_more_frag_does_not_extend();
    test_sack_timer_slides_on_more_frag_when_enabled();
    test_sack_delay_short_zero_sentinel_preserves_legacy();

    std::cout << "\nARQ Boundary/Property Tests:\n";
    test_partial_mc_dpsk_cw_nack_and_merge();
    test_cw_nack_triggers_compact_data_repair_phase2();
    test_stale_ack_older_than_base_minus_one_is_ignored();
    test_future_ack_too_far_ahead_is_ignored();
    test_duplicate_sack_hole_is_suppressed_without_duplicate_retx_accounting();
    test_hole_probe_not_rearmed_while_fast_hole_in_flight();
    test_duplicate_data_is_not_delivered_twice_and_sends_recovery_sack();
    test_timeout_repair_retransmits_only_missing_slot_and_resets_timer();
    test_timeout_window_retransmits_as_one_batch_when_callback_present();
    test_deferred_timeout_cancel_preserves_retry_and_karn_state();
    test_deferred_timeout_cancel_cannot_terminal_fail_without_commit();
    test_ack_progress_accessor_counts_and_dedups();
    test_defer_pending_retransmits_gates_slot_rto();

    std::cout << "\nBasic Tests:\n";
    test_create_sr_arq();
    test_send_single_frame();
    test_data_flags_preserve_version_bit();
    test_code_rate_change_aborts_in_flight_fixed_frames();
    test_send_window_full();
    test_receive_ack();

    std::cout << "\nReceiver Tests:\n";
    test_rx_in_order();
    test_below_window_file_salvage();
    test_rx_out_of_order();

    std::cout << "\nMOVE-EPOCH Tests (BUG-ARQ-SEQ-COLLISION structural fix):\n";
    test_move_epoch_bump_on_abort_and_stamp();
    test_move_epoch_bump_on_cw_abort_same_rate();
    test_move_epoch_regrid_resend_accepted();
    test_move_epoch_stale_ack_ignored();
    test_move_epoch_unanchored_wait_for_rebase();
    test_move_epoch_adoption_group_advances_past_last_frame();
    test_rate_abort_salvages_sacked_frames();
    test_epoch_adoption_salvages_buffered_file_frames();
    test_move_epoch_knob_off_byte_identical();

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
