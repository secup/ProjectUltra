/**
 * Protocol Layer Test Suite
 *
 * Tests the ARQ protocol implementation by simulating two stations
 * communicating with each other. No actual audio/modem involved -
 * TX output from one station feeds directly into RX of the other.
 */

#include "env_compat.hpp"
#include "protocol/protocol_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/file_transfer.hpp"
#include "ultra/types.hpp"
#include "protocol/compression.hpp"
#include "helpers/temp_dir.hpp"
#include <iostream>
#include <cassert>
#include <thread>
#include <queue>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <system_error>
#include <algorithm>

using namespace ultra::protocol;
using ultra::Bytes;
using ultra::CodeRate;
using ultra::Modulation;

namespace {

class ScopedEnvVar {
public:
    explicit ScopedEnvVar(const char* name) : name_(name) {
        if (const char* value = std::getenv(name)) {
            had_value_ = true;
            old_value_ = value;
        }
    }

    ~ScopedEnvVar() {
        if (had_value_) {
            setenv(name_.c_str(), old_value_.c_str(), 1);
        } else {
            unsetenv(name_.c_str());
        }
    }

    void set(const char* value) { setenv(name_.c_str(), value, 1); }
    void unset() { unsetenv(name_.c_str()); }

private:
    std::string name_;
    std::string old_value_;
    bool had_value_ = false;
};

}  // namespace

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
// v2 Frame Tests
// ============================================================================

bool test_control_frame_serialization() {
    TEST("v2 Control frame serialization/deserialization");

    // Create a PROBE frame
    auto probe = v2::ControlFrame::makeProbe("VA2MVR", "VE3ABC");
    Bytes serialized = probe.serialize();

    // Should be exactly 20 bytes (ControlFrame::SIZE)
    if (serialized.size() != v2::ControlFrame::SIZE) {
        FAIL("Serialized control frame wrong size");
    }

    // Deserialize
    auto parsed = v2::ControlFrame::deserialize(serialized);
    if (!parsed) {
        FAIL("Failed to deserialize valid control frame");
    }

    // Verify fields
    if (parsed->type != v2::FrameType::PROBE) FAIL("Type mismatch");
    if (parsed->src_hash != v2::hashCallsign("VA2MVR")) FAIL("Source hash mismatch");
    if (parsed->dst_hash != v2::hashCallsign("VE3ABC")) FAIL("Dest hash mismatch");

    PASS();
    return true;
}

bool test_data_frame_serialization() {
    TEST("v2 Data frame serialization/deserialization");

    // Create a DATA frame
    Bytes payload(100, 0x42);  // 100 bytes of 'B'
    auto frame = v2::DataFrame::makeData("VA2MVR", "VE3ABC", 5, payload);
    Bytes serialized = frame.serialize();

    // Deserialize
    auto parsed = v2::DataFrame::deserialize(serialized);
    if (!parsed) {
        FAIL("Failed to deserialize valid data frame");
    }

    // Verify fields
    if (parsed->type != v2::FrameType::DATA) FAIL("Type mismatch");
    if (parsed->seq != 5) FAIL("Sequence mismatch");
    if (parsed->payload != payload) FAIL("Payload mismatch");

    PASS();
    return true;
}

bool test_frame_crc() {
    TEST("v2 Frame CRC validation");

    auto probe = v2::ControlFrame::makeProbe("TEST1", "TEST2");
    Bytes data = probe.serialize();

    // Valid frame should parse
    auto valid = v2::ControlFrame::deserialize(data);
    if (!valid) FAIL("Valid frame rejected");

    // Corrupt one byte in middle
    data[10] ^= 0xFF;
    auto corrupt = v2::ControlFrame::deserialize(data);
    if (corrupt) FAIL("Corrupt frame accepted");

    PASS();
    return true;
}

bool test_control_frame_types() {
    TEST("All v2 control frame types");

    // Test ControlFrames (1 codeword)
    std::vector<std::pair<v2::ControlFrame, v2::FrameType>> control_cases = {
        { v2::ControlFrame::makeProbe("A", "B"), v2::FrameType::PROBE },
        { v2::ControlFrame::makeAck("A", "B", 1), v2::FrameType::ACK },
        { v2::ControlFrame::makeNack("A", "B", 1, 0), v2::FrameType::NACK },
        { v2::ControlFrame::makeDisconnect("A", "B"), v2::FrameType::DISCONNECT },
        { v2::ControlFrame::makeFileCancel("A", "B"), v2::FrameType::FILE_CANCEL },
        { v2::ControlFrame::makeKeepalive("A", "B"), v2::FrameType::KEEPALIVE },
        { v2::ControlFrame::makeModeChange("A", "B", 1, ultra::Modulation::QAM16, ultra::CodeRate::R2_3, 20.0f, 0.50f, 0), v2::FrameType::MODE_CHANGE },
    };

    for (const auto& [frame, expected_type] : control_cases) {
        Bytes data = frame.serialize();
        auto parsed = v2::ControlFrame::deserialize(data);
        if (!parsed) FAIL("Failed to parse control frame");
        if (parsed->type != expected_type) FAIL("Type mismatch");
    }

    // Test ConnectFrames (3 codewords)
    std::vector<std::pair<v2::ConnectFrame, v2::FrameType>> connect_cases = {
        { v2::ConnectFrame::makeConnect("CALLSIGN1", "CALLSIGN2", 0x07, 0), v2::FrameType::CONNECT },
        { v2::ConnectFrame::makeConnectAck("CALLSIGN1", "CALLSIGN2", 0, ultra::Modulation::DQPSK, ultra::CodeRate::R1_4, 15.0f, 0.60f, 4), v2::FrameType::CONNECT_ACK },
        { v2::ConnectFrame::makeConnectNak("CALLSIGN1", "CALLSIGN2"), v2::FrameType::CONNECT_NAK },
        { v2::ConnectFrame::makeDisconnect("CALLSIGN1", "CALLSIGN2"), v2::FrameType::DISCONNECT },
    };

    for (const auto& [frame, expected_type] : connect_cases) {
        Bytes data = frame.serialize();
        auto parsed = v2::ConnectFrame::deserialize(data);
        if (!parsed) FAIL("Failed to parse connect frame");
        if (parsed->type != expected_type) FAIL("Type mismatch");
    }

    PASS();
    return true;
}

bool test_callsign_validation() {
    TEST("Callsign validation");

    // Valid callsigns
    if (!isValidCallsign("VA2MVR")) FAIL("Valid callsign rejected");
    if (!isValidCallsign("W1AW")) FAIL("Valid callsign rejected");
    if (!isValidCallsign("VE3ABC")) FAIL("Valid callsign rejected");

    // Invalid callsigns
    if (isValidCallsign("")) FAIL("Empty callsign accepted");
    if (isValidCallsign("AB")) FAIL("Too short callsign accepted");

    // Sanitization
    if (sanitizeCallsign("va2mvr") != "VA2MVR") FAIL("Sanitize should uppercase");

    PASS();
    return true;
}

bool test_callsign_hash() {
    TEST("Callsign hash");

    // Hash should be deterministic
    uint32_t h1 = v2::hashCallsign("VA2MVR");
    uint32_t h2 = v2::hashCallsign("VA2MVR");
    if (h1 != h2) FAIL("Hash not deterministic");

    // Different callsigns should (likely) have different hashes
    uint32_t h3 = v2::hashCallsign("VE3ABC");
    if (h1 == h3) FAIL("Different callsigns same hash");

    // Hash should be 24-bit
    if (h1 > 0xFFFFFF) FAIL("Hash exceeds 24 bits");

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

        stationA_.setTxDataCallback([this](const Bytes& data, bool) {
            if (verbose_) {
                std::cout << "    [A->B] " << data.size() << " bytes\n";
            }
            tx_count_a_++;
            observeControlFrame(data, true);
            if (shouldDropNextFileCancel(data, true)) {
                return;
            }
            if (shouldDropDisconnectAck(data)) {
                return;
            }
            if (!drop_a_to_b_) {
                pending_b_.push(data);
            }
        });

        stationB_.setTxDataCallback([this](const Bytes& data, bool) {
            if (verbose_) {
                std::cout << "    [B->A] " << data.size() << " bytes\n";
            }
            tx_count_b_++;
            observeControlFrame(data, false);
            if (shouldDropNextFileCancel(data, false)) {
                return;
            }
            if (shouldDropDisconnectAck(data)) {
                return;
            }
            if (!drop_b_to_a_) {
                pending_a_.push(data);
            }
        });
    }

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

    void tick(uint32_t ms) {
        stationA_.tick(ms);
        stationB_.tick(ms);
    }

    void run(int cycles, uint32_t tick_ms = 100) {
        for (int i = 0; i < cycles; i++) {
            deliver();
            tick(tick_ms);
        }
    }

    void setDropAtoB(bool drop) { drop_a_to_b_ = drop; }
    void setDropBtoA(bool drop) { drop_b_to_a_ = drop; }
    void dropNextFileCancelAtoB() { drop_next_file_cancel_a_to_b_ = true; }
    void dropNextFileCancelBtoA() { drop_next_file_cancel_b_to_a_ = true; }
    void setDropDisconnectAcks(bool drop) { drop_disconnect_acks_ = drop; }
    void setVerbose(bool v) { verbose_ = v; }

    int getTxCountA() const { return tx_count_a_; }
    int getTxCountB() const { return tx_count_b_; }
    int getTurnoverCountA() const { return turnover_count_a_; }
    int getTurnoverCountB() const { return turnover_count_b_; }
    int getTurnRequestCountA() const { return turn_request_count_a_; }
    int getTurnRequestCountB() const { return turn_request_count_b_; }
    int getFileCancelCountA() const { return file_cancel_count_a_; }
    int getFileCancelCountB() const { return file_cancel_count_b_; }
    int getDisconnectCountA() const { return disconnect_count_a_; }
    int getDisconnectCountB() const { return disconnect_count_b_; }
    int getDisconnectAckCountA() const { return disconnect_ack_count_a_; }
    int getDisconnectAckCountB() const { return disconnect_ack_count_b_; }
    bool lastConnectAckForcedFromB() const {
        return last_connect_ack_forced_from_b_;
    }

private:
    bool shouldDropNextFileCancel(const Bytes& data, bool from_a) {
        auto ctrl = v2::ControlFrame::deserialize(data);
        if (!ctrl || ctrl->type != v2::FrameType::FILE_CANCEL) {
            return false;
        }
        bool& drop_next = from_a ? drop_next_file_cancel_a_to_b_
                                 : drop_next_file_cancel_b_to_a_;
        if (!drop_next) {
            return false;
        }
        drop_next = false;
        return true;
    }

    bool shouldDropDisconnectAck(const Bytes& data) const {
        if (!drop_disconnect_acks_) {
            return false;
        }
        const auto ctrl = v2::ControlFrame::deserialize(data);
        return ctrl && ctrl->type == v2::FrameType::ACK &&
               ctrl->seq == v2::DISCONNECT_SEQ;
    }

    void observeControlFrame(const Bytes& data, bool from_a) {
        // Observe the serialized header directly: this assertion is specifically about
        // what crossed the wire, independent of the receiver parser exercised next.
        if (!from_a && data.size() > 3 &&
            data[2] == static_cast<uint8_t>(v2::FrameType::CONNECT_ACK)) {
            last_connect_ack_forced_from_b_ =
                (data[3] & v2::Flags::CONNECT_FORCED_PROFILE) != 0;
        }
        auto ctrl = v2::ControlFrame::deserialize(data);
        if (!ctrl) {
            return;
        }
        if (ctrl->type == v2::FrameType::TURNOVER) {
            if (from_a) {
                turnover_count_a_++;
            } else {
                turnover_count_b_++;
            }
        } else if (ctrl->type == v2::FrameType::TURN_REQUEST) {
            if (from_a) {
                turn_request_count_a_++;
            } else {
                turn_request_count_b_++;
            }
        } else if (ctrl->type == v2::FrameType::FILE_CANCEL) {
            if (from_a) {
                file_cancel_count_a_++;
            } else {
                file_cancel_count_b_++;
            }
        } else if (ctrl->type == v2::FrameType::DISCONNECT) {
            if (from_a) {
                disconnect_count_a_++;
            } else {
                disconnect_count_b_++;
            }
        } else if (ctrl->type == v2::FrameType::ACK &&
                   ctrl->seq == v2::DISCONNECT_SEQ) {
            if (from_a) {
                disconnect_ack_count_a_++;
            } else {
                disconnect_ack_count_b_++;
            }
        }
    }

    ProtocolEngine& stationA_;
    ProtocolEngine& stationB_;

    std::queue<Bytes> pending_a_;
    std::queue<Bytes> pending_b_;

    bool drop_a_to_b_ = false;
    bool drop_b_to_a_ = false;
    bool drop_next_file_cancel_a_to_b_ = false;
    bool drop_next_file_cancel_b_to_a_ = false;
    bool drop_disconnect_acks_ = false;
    bool verbose_ = false;

    int tx_count_a_ = 0;
    int tx_count_b_ = 0;
    int turnover_count_a_ = 0;
    int turnover_count_b_ = 0;
    int turn_request_count_a_ = 0;
    int turn_request_count_b_ = 0;
    int file_cancel_count_a_ = 0;
    int file_cancel_count_b_ = 0;
    int disconnect_count_a_ = 0;
    int disconnect_count_b_ = 0;
    int disconnect_ack_count_a_ = 0;
    int disconnect_ack_count_b_ = 0;
    bool last_connect_ack_forced_from_b_ = false;
};

bool test_connection_establishment() {
    TEST("Connection establishment");

    ConnectionConfig config;
    config.auto_accept = true;
    config.connect_timeout_ms = 5000;
    config.connect_retries = 3;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("TEST1A");
    stationB.setLocalCallsign("TEST2B");

    bool a_connected = false;
    bool b_connected = false;

    stationA.setConnectionChangedCallback([&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::CONNECTED) a_connected = true;
    });

    stationB.setConnectionChangedCallback([&](ConnectionState state, const std::string&) {
        if (state == ConnectionState::CONNECTED) b_connected = true;
    });

    SimulatedChannel channel(stationA, stationB);

    if (!stationA.connect("TEST2B")) {
        FAIL("Connect() returned false");
    }

    for (int i = 0; i < 50 && (!a_connected || !b_connected); i++) {
        channel.run(1, 100);
    }

    if (!a_connected) FAIL("Station A did not connect");
    if (!b_connected) FAIL("Station B did not connect");

    PASS();
    return true;
}

bool test_environment_force_handshake_precedence() {
    TEST("Environment-forced data rung survives handshake entry policy");

    ScopedEnvVar force_mod("ULTRA_FORCE_DATA_MOD");
    ScopedEnvVar force_rate("ULTRA_FORCE_DATA_RATE");
    ScopedEnvVar lock_rate("ULTRA_LOCK_RATE");
    ScopedEnvVar max_rate("ULTRA_MAX_OFDM_RATE");

    auto make_config = [](bool auto_accept) {
        ConnectionConfig config;
        config.auto_accept = auto_accept;
        config.mode_capabilities =
            ModeCapabilities::OFDM_CHIRP | ModeCapabilities::PHY_MASK_V1;
        config.preferred_mode = WaveformMode::OFDM_CHIRP;
        return config;
    };

    auto verify_pair = [&](bool auto_accept, Modulation expected_mod,
                           CodeRate expected_rate) -> bool {
        ProtocolEngine stationA(make_config(true));
        ProtocolEngine stationB(make_config(auto_accept));
        stationA.setLocalCallsign("W1ABC");
        stationB.setLocalCallsign("K2DEF");
        stationB.setChannelQuality(20.0f, 0.50f,
                                   ultra::SNRSource::MCDPSK_IN_BAND,
                                   /*data_aided=*/true);

        SimulatedChannel channel(stationA, stationB);
        if (!stationA.connect("K2DEF")) return false;
        if (!auto_accept) {
            channel.run(30, 100);
            stationB.acceptCall();
        }
        channel.run(50, 100);

        return stationA.isConnected() && stationB.isConnected() &&
               stationA.getDataModulation() == expected_mod &&
               stationB.getDataModulation() == expected_mod &&
               stationA.getDataCodeRate() == expected_rate &&
               stationB.getDataCodeRate() == expected_rate;
    };

    // This is the exact live diagnostic contract. The valid env request is serialized
    // into CONNECT, survives the weaker automatic MAX ceiling, and is echoed exactly.
    force_mod.set("8PSK");
    force_rate.set("R2_3");
    lock_rate.unset();
    max_rate.set("R1_2");
    if (!verify_pair(/*auto_accept=*/true, Modulation::QAM8, CodeRate::R2_3)) {
        FAIL("auto-accept MAX cap replaced valid env 8PSK R2/3 force");
    }
    if (!verify_pair(/*auto_accept=*/false, Modulation::QAM8, CodeRate::R2_3)) {
        FAIL("manual-accept MAX cap replaced valid env 8PSK R2/3 force");
    }

    // Initiator-only environment force: sendFullConnect resolves it into the wire frame.
    // Remove every live env knob before the responder sees CONNECT; both peers must still
    // converge on the requested profile and the initiator's QSO pin must not depend on
    // the variable remaining exported.
    {
        ProtocolEngine stationA(make_config(true));
        ProtocolEngine stationB(make_config(true));
        stationA.setLocalCallsign("E1ABC");
        stationB.setLocalCallsign("E2DEF");
        stationB.setChannelQuality(20.0f, 0.50f,
                                   ultra::SNRSource::MCDPSK_IN_BAND,
                                   /*data_aided=*/true);
        SimulatedChannel channel(stationA, stationB);
        if (!stationA.connect("E2DEF")) {
            FAIL("initiator-only env-force CONNECT was not queued");
        }
        force_mod.unset();
        force_rate.unset();
        max_rate.unset();
        channel.run(50, 100);
        if (!stationA.isConnected() || !stationB.isConnected() ||
            stationA.getDataModulation() != Modulation::QAM8 ||
            stationB.getDataModulation() != Modulation::QAM8 ||
            stationA.getDataCodeRate() != CodeRate::R2_3 ||
            stationB.getDataCodeRate() != CodeRate::R2_3) {
            FAIL("initiator-only env force was not serialized into CONNECT");
        }
    }

    // The same resolved wire profile must survive CONNECT loss. Retries occur after the
    // environment may have changed, so they must reuse the attempt-scoped outbound force
    // rather than reconstructing the request from static ConnectionConfig.
    force_mod.set("8PSK");
    force_rate.set("R2_3");
    {
        ProtocolEngine stationA(make_config(true));
        ProtocolEngine stationB(make_config(true));
        stationA.setLocalCallsign("L1ABC");
        stationB.setLocalCallsign("L2DEF");
        stationB.setChannelQuality(20.0f, 0.50f,
                                   ultra::SNRSource::MCDPSK_IN_BAND,
                                   /*data_aided=*/true);
        SimulatedChannel channel(stationA, stationB);
        channel.setDropAtoB(true);
        if (!stationA.connect("L2DEF")) {
            FAIL("loss/retry env-force CONNECT was not initiated");
        }
        force_mod.unset();
        force_rate.unset();
        channel.setDropAtoB(false);
        // CONNECT retry cadence is derived from four robust MC-DPSK control-frame
        // airtimes (~34 s with production geometry), so advance past one full interval.
        channel.run(400, 100);
        if (!stationA.isConnected() || !stationB.isConnected() ||
            stationA.getDataModulation() != Modulation::QAM8 ||
            stationB.getDataModulation() != Modulation::QAM8 ||
            stationA.getDataCodeRate() != CodeRate::R2_3 ||
            stationB.getDataCodeRate() != CodeRate::R2_3) {
            FAIL("CONNECT retry discarded the attempt-scoped environment force");
        }
    }

    // Responder-only force: the initial CONNECT is already on the wire with AUTO fields
    // when the responder's local operator profile appears. The ACK must carry explicit
    // force provenance so the initiator pins the same QSO instead of adapting away later.
    force_mod.unset();
    force_rate.unset();
    {
        ProtocolEngine stationA(make_config(true));
        ProtocolEngine stationB(make_config(true));
        stationA.setLocalCallsign("P1ABC");
        stationB.setLocalCallsign("P2DEF");
        stationB.setChannelQuality(20.0f, 0.50f,
                                   ultra::SNRSource::MCDPSK_IN_BAND,
                                   /*data_aided=*/true);
        SimulatedChannel channel(stationA, stationB);
        if (!stationA.connect("P2DEF")) {
            FAIL("responder-only env-force CONNECT was not initiated");
        }
        force_mod.set("8PSK");
        force_rate.set("R2_3");
        if (!environmentForcesDataProfile()) {
            FAIL("responder-only env force was not visible before CONNECT delivery");
        }
        channel.deliver();  // CONNECT reaches B; ProtocolEngine defers its reentrant ACK.
        channel.tick(1);    // Flush the serialized ACK onto the simulated wire, but not to A.
        if (!channel.lastConnectAckForcedFromB()) {
            FAIL("responder-only force was not marked in CONNECT_ACK");
        }
        force_mod.unset();
        force_rate.unset();
        channel.run(50, 100);
        if (!stationA.isConnected() || !stationB.isConnected() ||
            stationA.getDataModulation() != Modulation::QAM8 ||
            stationB.getDataModulation() != Modulation::QAM8 ||
            stationA.getDataCodeRate() != CodeRate::R2_3 ||
            stationB.getDataCodeRate() != CodeRate::R2_3) {
            FAIL("responder-only forced profile did not converge at both endpoints");
        }
    }

    // A mixed valid+invalid environment profile fails atomically.  The valid half must
    // not become an accidental partial force; normal automatic entry receives the
    // conservative QPSK R1/2 first-coherent-group cap.
    force_mod.set("8PSK");
    force_rate.set("R2_3_typo");
    max_rate.unset();
    if (!verify_pair(/*auto_accept=*/true, Modulation::QPSK, CodeRate::R1_2)) {
        FAIL("mixed malformed env profile was applied partially");
    }

    // The serialized force remains the highest-priority interoperability path.  Local
    // invalid env values must neither mask nor reinterpret explicit CONNECT fields.
    ConnectionConfig initiator_config = make_config(true);
    initiator_config.forced_modulation = Modulation::QAM8;
    initiator_config.forced_code_rate = CodeRate::R2_3;
    max_rate.set("R1_2");
    ProtocolEngine stationA(initiator_config);
    ProtocolEngine stationB(make_config(true));
    stationA.setLocalCallsign("N1ABC");
    stationB.setLocalCallsign("N2DEF");
    stationB.setChannelQuality(20.0f, 0.50f,
                               ultra::SNRSource::MCDPSK_IN_BAND,
                               /*data_aided=*/true);
    SimulatedChannel channel(stationA, stationB);
    stationA.connect("N2DEF");
    channel.run(50, 100);
    if (!stationA.isConnected() || !stationB.isConnected() ||
        stationA.getDataModulation() != Modulation::QAM8 ||
        stationB.getDataModulation() != Modulation::QAM8 ||
        stationA.getDataCodeRate() != CodeRate::R2_3 ||
        stationB.getDataCodeRate() != CodeRate::R2_3) {
        FAIL("explicit CONNECT force lost precedence over local entry policy");
    }

    // A rate-only serialized request is still a forced profile. It must bypass MAX,
    // converge at both endpoints, and not be omitted from the responder's forced gate.
    force_mod.unset();
    force_rate.unset();
    ConnectionConfig rate_only_initiator = make_config(true);
    rate_only_initiator.forced_code_rate = CodeRate::R2_3;
    ProtocolEngine rateA(rate_only_initiator);
    ProtocolEngine rateB(make_config(true));
    rateA.setLocalCallsign("R1ABC");
    rateB.setLocalCallsign("R2DEF");
    rateB.setChannelQuality(20.0f, 0.50f,
                            ultra::SNRSource::MCDPSK_IN_BAND,
                            /*data_aided=*/true);
    SimulatedChannel rate_channel(rateA, rateB);
    rateA.connect("R2DEF");
    rate_channel.run(50, 100);
    if (!rateA.isConnected() || !rateB.isConnected() ||
        rateA.getDataCodeRate() != CodeRate::R2_3 ||
        rateB.getDataCodeRate() != CodeRate::R2_3 ||
        rateA.getDataModulation() != rateB.getDataModulation()) {
        FAIL("rate-only CONNECT force was capped or split between endpoints");
    }

    PASS();
    return true;
}

bool test_nonphysical_snr_sources_do_not_drive_negotiation() {
    TEST("Non-physical SNR sources do not drive negotiation");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("TEST1A");
    stationB.setLocalCallsign("TEST2B");

    stationB.setMeasuredSNR(30.0f, ultra::SNRSource::SYNC_QUALITY);
    stationB.setMeasuredSNR(35.0f, ultra::SNRSource::OFDM_INTERNAL);

    SimulatedChannel channel(stationA, stationB);

    if (!stationA.connect("TEST2B")) {
        FAIL("Connect() returned false");
    }

    channel.run(50, 100);

    if (!stationA.isConnected()) FAIL("Station A did not connect");
    if (!stationB.isConnected()) FAIL("Station B did not connect");
    if (stationB.getMeasuredSNRSource() == ultra::SNRSource::SYNC_QUALITY) {
        FAIL("SYNC_QUALITY was stored as rate-selection SNR");
    }
    if (stationB.getMeasuredSNRSource() == ultra::SNRSource::OFDM_INTERNAL) {
        FAIL("OFDM_INTERNAL was stored as rate-selection SNR");
    }
    if (stationB.getNegotiatedMode() != WaveformMode::MC_DPSK) {
        FAIL("non-physical SNR promoted responder out of MC-DPSK fallback");
    }
    if (stationA.getNegotiatedMode() != WaveformMode::MC_DPSK) {
        FAIL("non-physical SNR promoted initiator out of MC-DPSK fallback");
    }

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

    stationA.connect("K2DEF");
    channel.run(30, 100);

    if (!stationA.isConnected()) FAIL("Not connected");

    stationA.disconnect();
    channel.run(20, 100);

    // The responder intentionally stays connected during a disconnect grace
    // period so it can re-send the final ACK if the initiator retransmits the
    // DISCONNECT. Advance beyond that production grace window before asserting.
    channel.run(60, 100);

    if (!a_disconnected && stationA.isConnected()) FAIL("A not disconnected");
    if (!b_disconnected && stationB.isConnected()) FAIL("B not disconnected");

    PASS();
    return true;
}

bool test_disconnected_disconnect_does_not_emit_stale_state() {
    TEST("Disconnected close request does not emit stale DISCONNECTING state");

    ProtocolEngine station;
    station.setLocalCallsign("W1ABC");

    std::vector<ConnectionState> states;
    station.setConnectionChangedCallback(
        [&](ConnectionState state, const std::string&) { states.push_back(state); });

    station.disconnect();

    if (station.getState() != ConnectionState::DISCONNECTED) {
        FAIL("No-op disconnect changed the underlying connection state");
    }
    if (std::find(states.begin(), states.end(), ConnectionState::DISCONNECTING) !=
        states.end()) {
        FAIL("ProtocolEngine emitted DISCONNECTING after a terminal/no-op close");
    }

    PASS();
    return true;
}

bool test_teardown_purges_earlier_deferred_protocol_tx() {
    TEST("Teardown purges non-close TX queued earlier in the same RX callback");

    ConnectionConfig config;
    config.auto_accept = true;
    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    SimulatedChannel channel(stationA, stationB);
    if (!stationA.connect("K2DEF")) {
        FAIL("Connection setup did not start");
    }
    channel.run(50, 100);
    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Connection setup did not complete");
    }

    std::vector<Bytes> emitted;
    stationB.setTxDataCallback(
        [&](const Bytes& data, bool) { emitted.push_back(data); });

    const Bytes mode_change = v2::ControlFrame::makeModeChange(
        "W1ABC", "K2DEF", 77, Modulation::QPSK, CodeRate::R1_2,
        20.0f, 0.50f, 0).serialize();
    const Bytes disconnect =
        v2::ControlFrame::makeDisconnect("W1ABC", "K2DEF").serialize();
    Bytes same_callback = mode_change;
    same_callback.insert(same_callback.end(), disconnect.begin(), disconnect.end());

    // MODE_CHANGE first queues an ordinary ACK because onRxData() defers host TX.
    // DISCONNECT later in this same callback activates teardown and must evict it,
    // while retaining the sentinel close ACK generated for the second frame.
    stationB.onRxData(same_callback, true);
    stationB.tick(1);

    if (emitted.size() != 1) {
        FAIL("Deferred non-close response escaped (or close ACK was lost)");
    }
    const auto header = v2::parseHeader(emitted.front());
    if (!header.valid || header.type != v2::FrameType::ACK ||
        header.seq != v2::DISCONNECT_SEQ) {
        FAIL("Only the reserved DISCONNECT ACK may survive teardown activation");
    }

    PASS();
    return true;
}

bool test_crossed_disconnect_converges_without_acks() {
    TEST("Crossed disconnect converges through mutual-close grace");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(30, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Stations did not connect");
    }
    if (!stationA.isInitiator() || stationB.isInitiator()) {
        FAIL("Connection teardown ownership role is wrong");
    }

    // Both operators close before either request arrives. Drop every ACK for the
    // reserved DISCONNECT sequence: the two decoded close requests must themselves
    // be sufficient to converge, without phase-locked DISCONNECT retransmissions.
    channel.setDropDisconnectAcks(true);
    stationA.disconnect();
    stationB.disconnect();
    channel.run(2, 100);

    if (stationA.getState() != ConnectionState::DISCONNECTING ||
        stationB.getState() != ConnectionState::DISCONNECTING) {
        FAIL("Crossed close did not enter mutual DISCONNECTING grace");
    }

    channel.run(60, 100);

    if (stationA.getState() != ConnectionState::DISCONNECTED ||
        stationB.getState() != ConnectionState::DISCONNECTED) {
        FAIL("Crossed close did not converge after responder grace");
    }
    if (channel.getDisconnectCountA() != 1 || channel.getDisconnectCountB() != 1) {
        FAIL("Local DISCONNECT retransmitted after peer close intent was decoded");
    }
    if (channel.getDisconnectAckCountA() != 1 ||
        channel.getDisconnectAckCountB() != 1) {
        FAIL("Crossed close must use one reactive ACK without symmetric repeat trains");
    }
    if (stationA.getStats().disconnects != 1 || stationB.getStats().disconnects != 1) {
        FAIL("Crossed close was double-counted in disconnect statistics");
    }

    PASS();
    return true;
}

bool test_manual_accept() {
    TEST("Manual call accept/reject");

    ConnectionConfig config;
    config.auto_accept = false;

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

    stationA.connect("K2DEF");
    channel.run(30, 100);

    if (!incoming_call) FAIL("No incoming call notification");

    stationB.acceptCall();
    channel.run(30, 100);

    if (!stationA.isConnected()) FAIL("A not connected after accept");
    if (!stationB.isConnected()) FAIL("B not connected after accept");

    PASS();
    return true;
}

// ============================================================================
// Capability Negotiation Tests
// ============================================================================

bool test_phy_mask_v1_negotiation() {
    TEST("PHY_MASK_V1 negotiation gates CarrierLDPC");

    ConnectionConfig modern;
    modern.auto_accept = true;

    ProtocolEngine stationA(modern);
    ProtocolEngine stationB(modern);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    bool a_enabled = false;
    bool b_enabled = false;
    stationA.setPhyMaskV1NegotiatedCallback([&](bool enabled) { a_enabled = enabled; });
    stationB.setPhyMaskV1NegotiatedCallback([&](bool enabled) { b_enabled = enabled; });

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(50, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) FAIL("modern stations did not connect");
    if (!stationA.isPhyMaskV1Negotiated() || !stationB.isPhyMaskV1Negotiated()) {
        FAIL("modern stations did not negotiate PHY_MASK_V1");
    }
    if (!a_enabled || !b_enabled) FAIL("PHY_MASK_V1 callback not raised on both sides");

    ConnectionConfig legacy = modern;
    legacy.mode_capabilities = ModeCapabilities::ALL;

    ProtocolEngine stationC(modern);
    ProtocolEngine stationD(legacy);
    stationC.setLocalCallsign("W3ABC");
    stationD.setLocalCallsign("K4DEF");

    bool legacy_enabled = false;
    stationC.setPhyMaskV1NegotiatedCallback([&](bool enabled) { legacy_enabled = legacy_enabled || enabled; });
    stationD.setPhyMaskV1NegotiatedCallback([&](bool enabled) { legacy_enabled = legacy_enabled || enabled; });

    SimulatedChannel legacy_channel(stationC, stationD);
    stationC.connect("K4DEF");
    legacy_channel.run(50, 100);

    if (!stationC.isConnected() || !stationD.isConnected()) FAIL("legacy pair did not connect");
    if (stationC.isPhyMaskV1Negotiated() || stationD.isPhyMaskV1Negotiated()) {
        FAIL("PHY_MASK_V1 negotiated with legacy peer");
    }
    if (legacy_enabled) FAIL("PHY_MASK_V1 callback enabled with legacy peer");

    PASS();
    return true;
}

// ============================================================================
// Binary stream / TNC-facing API Tests
// ============================================================================

bool test_physical_turn_provenance_stays_on_input_boundary() {
    TEST("Physical-turn provenance stays on the modem input boundary");

    ConnectionConfig config;
    config.auto_accept = true;
    config.mode_capabilities = ModeCapabilities::OFDM_CHIRP;
    config.preferred_mode = WaveformMode::OFDM_CHIRP;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");
    stationB.setMeasuredSNR(20.0f, ultra::SNRSource::OFDM_BROADBAND);

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(50, 100);
    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Connection not established");
    }

    std::vector<bool> ack_provenance;
    stationB.setTransmitToneBurstAckCallback(
        [&](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&,
            bool physical_complete) {
            ack_provenance.push_back(physical_complete);
        });

    auto make_frame = [&](uint16_t seq, uint8_t flags) {
        auto frame = v2::makeFixedDataFrame(
            "W1ABC", "K2DEF", seq, Bytes{0x42}, stationB.getDataCodeRate());
        frame.flags |= flags;
        return frame.serialize();
    };

    // A frame split across two onRxData calls inherits proof only from the call
    // that supplies its physical end boundary.
    const Bytes split = make_frame(0, v2::Flags::MORE_FRAG);
    const size_t cut = split.size() / 2;
    stationB.onRxData(Bytes(split.begin(), split.begin() + cut), false);
    stationB.onRxData(Bytes(split.begin() + cut, split.end()), true);
    if (ack_provenance.size() != 1 || !ack_provenance.back()) {
        FAIL("Split frame did not receive exactly one physical-boundary ACK");
    }

    // With two complete frames in one callback, only the final parsed frame ends
    // the physical input. FINAL makes the first frame emit an observable ordinary
    // ACK; the second is the single physically-complete boundary ACK.
    const size_t before_concat = ack_provenance.size();
    Bytes concat = make_frame(1, v2::Flags::FINAL);
    const Bytes second = make_frame(2, v2::Flags::MORE_FRAG);
    concat.insert(concat.end(), second.begin(), second.end());
    stationB.onRxData(concat, true);
    if (ack_provenance.size() != before_concat + 2 ||
        ack_provenance[before_concat] || !ack_provenance[before_concat + 1]) {
        FAIL("Concatenated input leaked physical provenance to a non-final member");
    }

    // A malformed physical callback must not leave a sticky bit that a later,
    // unrelated ordinary frame can inherit.
    stationB.onRxData(Bytes{0x00, 0x01}, true);
    const size_t before_ordinary = ack_provenance.size();
    stationB.onRxData(make_frame(3, v2::Flags::FINAL), false);
    if (ack_provenance.size() != before_ordinary + 1 || ack_provenance.back()) {
        FAIL("Malformed input leaked physical provenance to a later frame");
    }

    PASS();
    return true;
}

bool test_binary_fragment_reassembly_single_callback() {
    TEST("Binary fragment reassembly emits one data callback");

    ConnectionConfig config;
    config.auto_accept = true;
    config.mode_capabilities = ModeCapabilities::OFDM_CHIRP;
    config.preferred_mode = WaveformMode::OFDM_CHIRP;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    int data_callbacks = 0;
    int message_callbacks = 0;
    bool saw_more_data = false;
    Bytes received;

    stationB.setDataReceivedCallback([&](const Bytes& data, bool more_data) {
        data_callbacks++;
        saw_more_data = saw_more_data || more_data;
        received = data;
    });
    stationB.setMessageReceivedCallback([&](const std::string&, const std::string&) {
        message_callbacks++;
    });

    SimulatedChannel channel(stationA, stationB);

    stationA.connect("K2DEF");
    channel.run(50, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) FAIL("Connection not established");

    Bytes payload(1024);
    payload[0] = static_cast<uint8_t>(PayloadType::FILE_START);
    payload[1] = static_cast<uint8_t>(PayloadType::FILE_DATA);
    payload[2] = static_cast<uint8_t>(PayloadType::FILE_BLOCK);
    for (size_t i = 3; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>((i * 37 + 11) & 0xFF);
    }

    if (!stationA.sendBinary(payload)) FAIL("sendBinary() returned false");

    for (int i = 0; i < 300 && data_callbacks == 0; ++i) {
        channel.run(1, 50);
    }
    channel.run(60, 50);

    if (data_callbacks != 1) {
        std::cout << "(callbacks " << data_callbacks << ") ";
        FAIL("Expected exactly one reassembled data callback");
    }
    if (saw_more_data) FAIL("Data callback should only receive complete payloads");
    if (message_callbacks != 0) FAIL("Binary payload should not be delivered as text");
    if (received != payload) FAIL("Reassembled binary payload mismatch");

    PASS();
    return true;
}

bool test_send_binary_roundtrip_arbitrary_bytes() {
    TEST("sendBinary roundtrip preserves arbitrary bytes");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");
    stationB.setMeasuredSNR(25.0f);  // Keep this byte-preservation test on OFDM.

    Bytes received;
    stationB.setDataReceivedCallback([&](const Bytes& data, bool more_data) {
        if (!more_data) {
            received = data;
        }
    });

    SimulatedChannel channel(stationA, stationB);

    stationA.connect("K2DEF");
    channel.run(50, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) FAIL("Connection not established");

    Bytes payload;
    payload.reserve(260);
    payload.push_back(static_cast<uint8_t>(PayloadType::FILE_START));
    payload.push_back(0x00);
    payload.push_back(0xFF);
    payload.push_back(static_cast<uint8_t>(PayloadType::FILE_DATA));
    payload.push_back(static_cast<uint8_t>(PayloadType::FILE_BLOCK));
    for (int i = 0; i < 255; ++i) {
        payload.push_back(static_cast<uint8_t>(i));
    }

    if (!stationA.sendBinary(payload)) FAIL("sendBinary() returned false");

    for (int i = 0; i < 360 && received.empty(); ++i) {
        channel.run(1, 50);
    }

    if (received != payload) FAIL("Binary roundtrip payload mismatch");

    PASS();
    return true;
}

bool test_forced_tiny_capacity_text_message_roundtrip_and_status() {
    TEST("255-byte text message fragments exactly at tiny negotiated capacity");

    ConnectionConfig config;
    config.auto_accept = true;
    config.mode_capabilities = ModeCapabilities::OFDM_CHIRP;
    config.preferred_mode = WaveformMode::OFDM_CHIRP;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");
    stationA.setForcedModulation(Modulation::QPSK);
    stationA.setForcedCodeRate(CodeRate::R1_3);
    stationA.setForcedFrameCodewords(1, true);

    std::vector<std::pair<std::string, std::string>> received;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    stationB.setMessageReceivedCallback(
        [&](const std::string& from, const std::string& text) {
            received.emplace_back(from, text);
        });
    stationA.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(80, 100);
    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Connection not established");
    }
    if (stationA.getDataCodeRate() != CodeRate::R1_3 ||
        stationA.getForcedFrameCodewords() != 1) {
        FAIL("Tiny forced data geometry was not negotiated");
    }

    const std::string payload(255, 'M');
    if (!stationA.sendMessage(payload)) FAIL("sendMessage() returned false");
    for (int i = 0; i < 1500 && received.empty(); ++i) {
        channel.run(1, 25);
    }
    channel.run(20, 25);

    if (received.size() != 1 || received.front().first != "W1ABC" ||
        received.front().second != payload) {
        FAIL("Fragmented 255-byte text message was not delivered exactly once and byte-exact");
    }
    int submitted = 0;
    int delivered = 0;
    for (const auto& event : statuses) {
        if (event.text != payload || event.remote_call != "K2DEF") continue;
        submitted += event.status == Connection::MessageTxStatus::SUBMITTED ? 1 : 0;
        delivered += event.status == Connection::MessageTxStatus::DELIVERED ? 1 : 0;
    }
    if (submitted != 1 || delivered != 1) {
        FAIL("Fragmented text message did not publish one SUBMITTED and one DELIVERED result");
    }

    PASS();
    return true;
}

bool test_message_status_dispatch_allows_ordered_reentrant_reset() {
    TEST("Message status FIFO survives reentrant ProtocolEngine reset");

    ConnectionConfig config;
    config.auto_accept = true;
    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(80, 100);
    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Connection not established");
    }

    std::vector<Connection::MessageTxStatusEvent> statuses;
    bool reset_from_submitted = false;
    stationA.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
            if (!reset_from_submitted &&
                event.status == Connection::MessageTxStatus::SUBMITTED) {
                reset_from_submitted = true;
                // This re-enters the engine while its callback dispatcher is
                // active. FAILED must append behind SUBMITTED, never overtake it
                // or deadlock on the non-recursive state mutex.
                stationA.reset();
            }
        });

    const int tx_before = channel.getTxCountA();
    if (!stationA.sendMessage("engine-reset-after-submit")) {
        FAIL("sendMessage() returned false");
    }
    if (channel.getTxCountA() != tx_before + 1) {
        FAIL("Physical DATA handoff did not complete before SUBMITTED callback");
    }
    if (!reset_from_submitted || stationA.getState() != ConnectionState::DISCONNECTED) {
        FAIL("Reentrant reset did not leave ProtocolEngine disconnected");
    }
    if (statuses.size() != 2 ||
        statuses[0].status != Connection::MessageTxStatus::SUBMITTED ||
        statuses[1].status != Connection::MessageTxStatus::FAILED ||
        !statuses[0].sequence_valid || !statuses[1].sequence_valid ||
        statuses[0].text != "engine-reset-after-submit" ||
        statuses[1].text != "engine-reset-after-submit") {
        FAIL("Status dispatcher did not preserve SUBMITTED then FAILED ordering");
    }

    PASS();
    return true;
}

bool test_binary_delivery_callback_can_reenter_protocol_engine() {
    TEST("Binary delivery callback can re-enter ProtocolEngine");

    ConnectionConfig config;
    config.auto_accept = true;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);
    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");
    stationB.setMeasuredSNR(25.0f);  // Keep this callback-contract test on OFDM.

    bool callback_entered = false;
    bool callback_saw_connected = false;
    size_t callback_backlog = 0;
    Bytes received;
    stationB.setDataReceivedCallback([&](const Bytes& data, bool more_data) {
        if (more_data) {
            return;
        }
        callback_entered = true;
        received = data;
        // Public callbacks run outside ProtocolEngine's non-recursive mutex.
        // Before the pending-data queue, either getter deadlocked here.
        callback_saw_connected = stationB.isConnected();
        callback_backlog = stationB.getTxBacklogBytes();
    });

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(50, 100);
    if (!stationA.isConnected() || !stationB.isConnected()) {
        FAIL("Connection not established");
    }

    const Bytes payload{0x00, 0x42, 0xFF, 0x17};
    if (!stationA.sendBinary(payload)) FAIL("sendBinary() returned false");
    for (int i = 0; i < 300 && !callback_entered; ++i) {
        channel.run(1, 50);
    }

    if (!callback_entered) FAIL("Binary callback did not run");
    if (!callback_saw_connected) FAIL("Re-entrant state getter returned stale state");
    if (callback_backlog != 0) FAIL("Receiver unexpectedly had TX backlog");
    if (received != payload) FAIL("Re-entrant callback payload mismatch");

    PASS();
    return true;
}

std::string createPseudoRandomTestFile(const std::filesystem::path& dir,
                                       const std::string& name,
                                       size_t size) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) return "";

    std::string path = (dir / name).string();
    std::ofstream f(path, std::ios::binary);
    if (!f) return "";

    uint32_t state = 0x13579BDFu;
    for (size_t i = 0; i < size; i++) {
        state = state * 1664525u + 1013904223u;
        f.put(static_cast<char>((state >> 24) & 0xFF));
    }
    f.close();
    return path;
}

bool test_file_transfer_receiver_cancel_propagates_and_frees_link() {
    TEST("Receiver file cancel propagates and frees link");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 200;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    ultra::test::TempDir temp_dir("ultra_protocol_rx_cancel_test");
    if (!temp_dir.valid()) FAIL("Could not create temp test directory");
    const auto& test_dir = temp_dir.path();

    const size_t FILE_SIZE = 32768;
    std::string src_path = createPseudoRandomTestFile(test_dir, "rx_cancel_source.bin", FILE_SIZE);
    if (src_path.empty()) FAIL("Could not create cancel test file");

    std::string rx_dir = (test_dir / "rx").string();
    std::filesystem::create_directories(rx_dir);
    stationB.setReceiveDirectory(rx_dir);

    bool receive_started = false;
    bool receive_cancelled = false;
    bool sender_cancelled = false;
    stationB.setFileProgressCallback([&](const FileTransferProgress& p) {
        if (!p.is_sending && p.total_bytes > 0) {
            receive_started = true;
        }
    });
    stationB.setFileReceivedCallback([&](const std::string&, bool success, const std::string& error) {
        if (!success && error == "Transfer cancelled") {
            receive_cancelled = true;
        }
    });
    stationA.setFileSentCallback([&](bool success, const std::string& error) {
        if (!success && error == "Transfer cancelled") {
            sender_cancelled = true;
        }
    });

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(30, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) FAIL("Connection not established");
    if (!stationA.sendFile(src_path)) FAIL("A sendFile() returned false");

    for (int i = 0; i < 400 && !receive_started; i++) {
        channel.run(1, 50);
    }
    if (!receive_started) FAIL("B did not start receiving file");

    stationB.cancelFileTransfer();
    for (int i = 0; i < 1200 && (!receive_cancelled || !sender_cancelled); i++) {
        channel.run(1, 50);
    }

    if (channel.getFileCancelCountB() < 1) FAIL("Receiver did not transmit FILE_CANCEL");
    if (!receive_cancelled) FAIL("Receiver did not report transfer cancelled");
    if (!sender_cancelled) FAIL("Sender did not report transfer cancelled");

    PASS();
    return true;
}

bool test_file_transfer_sender_cancel_propagates_and_frees_link() {
    TEST("Sender file cancel propagates and frees link");

    ConnectionConfig config;
    config.auto_accept = true;
    config.arq.ack_timeout_ms = 200;

    ProtocolEngine stationA(config);
    ProtocolEngine stationB(config);

    stationA.setLocalCallsign("W1ABC");
    stationB.setLocalCallsign("K2DEF");

    ultra::test::TempDir temp_dir("ultra_protocol_tx_cancel_test");
    if (!temp_dir.valid()) FAIL("Could not create temp test directory");
    const auto& test_dir = temp_dir.path();

    const size_t FILE_SIZE = 32768;
    std::string src_path = createPseudoRandomTestFile(test_dir, "tx_cancel_source.bin", FILE_SIZE);
    if (src_path.empty()) FAIL("Could not create sender-cancel test file");

    std::string rx_dir = (test_dir / "rx").string();
    std::filesystem::create_directories(rx_dir);
    stationB.setReceiveDirectory(rx_dir);

    bool receive_started = false;
    bool receive_cancelled = false;
    bool sender_cancelled = false;
    stationB.setFileProgressCallback([&](const FileTransferProgress& p) {
        if (!p.is_sending && p.total_bytes > 0) {
            receive_started = true;
        }
    });
    stationB.setFileReceivedCallback([&](const std::string&, bool success, const std::string& error) {
        if (!success && error == "Transfer cancelled") {
            receive_cancelled = true;
        }
    });
    stationA.setFileSentCallback([&](bool success, const std::string& error) {
        if (!success && error == "Transfer cancelled") {
            sender_cancelled = true;
        }
    });

    SimulatedChannel channel(stationA, stationB);
    stationA.connect("K2DEF");
    channel.run(30, 100);

    if (!stationA.isConnected() || !stationB.isConnected()) FAIL("Connection not established");
    if (!stationA.sendFile(src_path)) FAIL("A sendFile() returned false");

    for (int i = 0; i < 400 && !receive_started; i++) {
        channel.run(1, 50);
    }
    if (!receive_started) FAIL("B did not start receiving file");

    stationA.cancelFileTransfer();
    for (int i = 0; i < 1200 && (!receive_cancelled || !sender_cancelled); i++) {
        channel.run(1, 50);
    }

    if (channel.getFileCancelCountA() < 1) FAIL("Sender did not transmit FILE_CANCEL");
    if (!sender_cancelled) FAIL("Sender did not report transfer cancelled");
    if (!receive_cancelled) FAIL("Receiver did not report transfer cancelled");

    PASS();
    return true;
}

// ============================================================================
// Compression Tests
// ============================================================================

bool test_compression_basic() {
    TEST("Basic compression/decompression");

    std::string text = "Hello World! This is a test of the compression system. ";
    for (int i = 0; i < 10; i++) {
        text += text;
    }
    Bytes input(text.begin(), text.end());

    auto compressed = Compression::compress(input);
    if (!compressed) FAIL("Compression failed");
    if (compressed->size() >= input.size()) FAIL("Compression didn't reduce size");

    auto decompressed = Compression::decompress(*compressed, input.size() * 2);
    if (!decompressed) FAIL("Decompression failed");
    if (*decompressed != input) FAIL("Decompressed data doesn't match original");

    PASS();
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=== Protocol Test Suite (v2 Frames) ===\n\n";

    std::cout << "v2 Frame Tests:\n";
    test_control_frame_serialization();
    test_data_frame_serialization();
    test_frame_crc();
    test_control_frame_types();
    test_callsign_validation();
    test_callsign_hash();

    std::cout << "\nTwo-Station Simulation:\n";
    test_connection_establishment();
    test_environment_force_handshake_precedence();
    test_nonphysical_snr_sources_do_not_drive_negotiation();
    test_disconnect();
    test_disconnected_disconnect_does_not_emit_stale_state();
    test_teardown_purges_earlier_deferred_protocol_tx();
    test_crossed_disconnect_converges_without_acks();
    test_manual_accept();

    std::cout << "\nCapability Negotiation:\n";
    test_phy_mask_v1_negotiation();

    std::cout << "\nBinary Stream / TNC API Tests:\n";
    test_physical_turn_provenance_stays_on_input_boundary();
    test_binary_fragment_reassembly_single_callback();
    test_send_binary_roundtrip_arbitrary_bytes();
    test_forced_tiny_capacity_text_message_roundtrip_and_status();
    test_message_status_dispatch_allows_ordered_reentrant_reset();
    test_binary_delivery_callback_can_reenter_protocol_engine();

    std::cout << "\nFile Transfer Tests:\n";
    // NOTE (TRANSPORT MERGE 2026-06-06): the in-process SimulatedChannel file/binary-send
    // tests (small-file, queue-during-guard, tx-backlog, receiver-cancel-retains-turn) were
    // removed — the unified path bursts through the modem (on_transmit_burst_ ->
    // encodeBurstLight -> RX burst_group_callback_), which a frame-level SimulatedChannel
    // cannot carry. File transfer is gated on the faithful GUI/OTASim path (gui_qso_scenario.sh).
    test_file_transfer_receiver_cancel_propagates_and_frees_link();
    test_file_transfer_sender_cancel_propagates_and_frees_link();

    std::cout << "\nCompression Tests:\n";
    test_compression_basic();

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_run << " passed ===\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
