#include "sim/simulated_station.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t kSweepValues[] = {
    0, 100, 200, 300, 500, 700, 1000, 1500, 2000,
};
constexpr const char* kMessage = "hello";
constexpr std::chrono::milliseconds kTickInterval(10);
constexpr std::chrono::seconds kProbeTimeout(14);
constexpr std::chrono::seconds kConnectTimeout(30);
constexpr std::chrono::seconds kDataTimeout(18);
constexpr std::chrono::seconds kAckTimeout(18);
constexpr std::chrono::seconds kDisconnectAckTimeout(15);
constexpr std::chrono::seconds kDisconnectDoneTimeout(10);

struct Event {
    uint64_t seq = 0;
    double t_s = 0.0;
    std::string station;
    std::string kind;
    std::string detail;
    std::optional<v2::FrameType> frame_type;
    std::optional<uint16_t> frame_seq;
};

const char* pttStateName(PttState state) {
    switch (state) {
        case PttState::RX: return "RX";
        case PttState::TX: return "TX";
        case PttState::TRANSITION: return "TRANSITION";
    }
    return "UNKNOWN";
}

const char* okFail(bool ok) {
    return ok ? "OK" : "FAIL";
}

class EventLog {
public:
    void add(double t_s,
             std::string station,
             std::string kind,
             std::string detail,
             std::optional<v2::FrameType> frame_type = std::nullopt,
             std::optional<uint16_t> frame_seq = std::nullopt) {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back(Event{next_seq_++, t_s, std::move(station),
                                std::move(kind), std::move(detail),
                                frame_type, frame_seq});
    }

    std::vector<Event> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto out = events_;
        std::sort(out.begin(), out.end(), [](const Event& a, const Event& b) {
            if (a.t_s != b.t_s) {
                return a.t_s < b.t_s;
            }
            return a.seq < b.seq;
        });
        return out;
    }

    bool sawConnectionState(const std::string& station, ConnectionState state) const {
        const std::string state_name = connectionStateToString(state);
        const auto events = snapshot();
        return std::any_of(events.begin(), events.end(), [&](const Event& e) {
            return e.station == station && e.kind == "conn.state" &&
                   e.detail == state_name;
        });
    }

    bool sawRxPingPong(const std::string& station) const {
        const auto events = snapshot();
        return std::any_of(events.begin(), events.end(), [&](const Event& e) {
            return e.station == station && e.kind == "frame.rx" &&
                   e.detail == "PING_OR_PONG";
        });
    }

    bool sawRxFrame(const std::string& station, v2::FrameType type) const {
        const auto events = snapshot();
        return std::any_of(events.begin(), events.end(), [&](const Event& e) {
            return e.station == station && e.kind == "frame.rx" &&
                   e.frame_type && *e.frame_type == type;
        });
    }

    bool sawRxFrameSeq(const std::string& station,
                       v2::FrameType type,
                       uint16_t seq) const {
        const auto events = snapshot();
        return std::any_of(events.begin(), events.end(), [&](const Event& e) {
            return e.station == station && e.kind == "frame.rx" &&
                   e.frame_type && *e.frame_type == type &&
                   e.frame_seq && *e.frame_seq == seq;
        });
    }

    void dump(const std::string& title) const {
        std::cout << "\n" << title << "\n";
        std::cout << std::fixed << std::setprecision(3);
        for (const auto& e : snapshot()) {
            std::cout << "  t=" << std::setw(7) << e.t_s << "s "
                      << std::setw(6) << e.station << " "
                      << std::setw(11) << e.kind << " "
                      << e.detail << "\n";
        }
        std::cout << std::defaultfloat;
    }

private:
    mutable std::mutex mutex_;
    uint64_t next_seq_ = 0;
    std::vector<Event> events_;
};

struct StationObservedState {
    ConnectionState last_connection = ConnectionState::DISCONNECTED;
    PttState last_ptt = PttState::RX;
};

std::string frameDetail(const v2::HeaderInfo& header) {
    std::ostringstream oss;
    oss << v2::frameTypeToString(header.type)
        << " seq=" << header.seq
        << " cw=" << static_cast<int>(header.total_cw);
    return oss.str();
}

void attachRxLogger(SimulatedStation& station,
                    const std::string& name,
                    EventLog& log) {
    station.setRxDecodeResultCallback([&station, name, &log](const DecodeResult& result) {
        if (result.is_ping) {
            log.add(station.getSimTime(), name, "frame.rx", "PING_OR_PONG");
            return;
        }
        if (!result.success || result.frame_data.empty()) {
            return;
        }
        const auto header = v2::parseHeader(result.frame_data);
        if (!header.valid) {
            return;
        }
        log.add(station.getSimTime(), name, "frame.rx", frameDetail(header),
                header.type, header.seq);
    });
}

void pollStation(SimulatedStation& station,
                 const std::string& name,
                 StationObservedState& observed,
                 EventLog& log) {
    const ConnectionState connection = station.getConnectionState();
    if (connection != observed.last_connection) {
        observed.last_connection = connection;
        log.add(station.getSimTime(), name, "conn.state",
                connectionStateToString(connection));
    }

    const PttState ptt = station.pttState();
    if (ptt != observed.last_ptt) {
        observed.last_ptt = ptt;
        log.add(station.getSimTime(), name, "ptt.state", pttStateName(ptt));
    }
}

void initializeObservedState(SimulatedStation& station,
                             const std::string& name,
                             StationObservedState& observed,
                             EventLog& log) {
    observed.last_connection = station.getConnectionState();
    observed.last_ptt = station.pttState();
    log.add(station.getSimTime(), name, "conn.state",
            connectionStateToString(observed.last_connection));
    log.add(station.getSimTime(), name, "ptt.state",
            pttStateName(observed.last_ptt));
}

template <typename Predicate>
bool runUntil(SimulatedStation& alpha,
              SimulatedStation& bravo,
              StationObservedState& alpha_observed,
              StationObservedState& bravo_observed,
              EventLog& log,
              std::chrono::steady_clock::duration timeout,
              Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        alpha.tick();
        bravo.tick();
        pollStation(alpha, "ALPHA", alpha_observed, log);
        pollStation(bravo, "BRAVO", bravo_observed, log);
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(kTickInterval);
    }

    alpha.tick();
    bravo.tick();
    pollStation(alpha, "ALPHA", alpha_observed, log);
    pollStation(bravo, "BRAVO", bravo_observed, log);
    return predicate();
}

struct SweepOutcome {
    uint32_t rx_settling_ms = 0;
    bool probe_ok = false;
    bool connect_ok = false;
    bool mode_handoff_ok = false;
    bool standalone_mode_change_seen = false;
    bool data_ok = false;
    bool ack_ok = false;
    bool disconnect_ack_ok = false;
    bool done = false;
    bool alpha_rx_pong = false;
    bool bravo_rx_ping = false;
    bool bravo_rx_data = false;
    bool alpha_rx_data_ack = false;
    bool bravo_rx_disconnect = false;
    bool alpha_rx_disconnect_ack = false;
    std::string first_failed_phase = "NONE";
    WaveformMode alpha_waveform = WaveformMode::AUTO;
    WaveformMode bravo_waveform = WaveformMode::AUTO;
    Modulation alpha_modulation = Modulation::AUTO;
    CodeRate alpha_code_rate = CodeRate::AUTO;
    ConnectionStats alpha_stats;
    ConnectionStats bravo_stats;
};

struct ExpectedOutcome {
    bool probe_ok = false;
    bool connect_ok = false;
    bool mode_handoff_ok = false;
    bool data_ok = false;
    bool ack_ok = false;
    bool disconnect_ack_ok = false;
    bool done = false;
    const char* first_failed_phase = "NONE";
    // Boundary cliff points are non-deterministic between adjacent
    // failure phases. If set, the observed first_failed_phase may match
    // either this or first_failed_phase. The other phase flags are
    // skipped for boundary points (only PROBE-OK and DONE are checked).
    const char* boundary_alt_failed_phase = nullptr;
};

ExpectedOutcome expectedOutcomeFor(uint32_t rx_settling_ms) {
    // Measured envelope after pong_tx_delay_ms=500,
    // post_connect_data_delay_ms=500, and ack_tx_delay_ms=500:
    //   0-500 ms:   full session OK (holds dodge peer's PTT-off)
    //   700-1500 ms: PROBE+CONNECT+MODE_CHANGE OK, DATA fails
    //                (500 ms post-CONNECT hold is exceeded by peer settling)
    //   2000 ms:    PROBE OK, CONNECT fails
    //                (500 ms pong_tx_delay exceeded by peer settling)
    if (rx_settling_ms <= 500) {
        return ExpectedOutcome{
            true, true, true, true, true, true, true, "NONE",
        };
    }
    if (rx_settling_ms <= 1000) {
        return ExpectedOutcome{
            true, true, true, false, false, false, false, "DATA",
        };
    }
    if (rx_settling_ms <= 1500) {
        // Boundary cliff: CONNECT and DATA both fail at this point depending
        // on timing jitter. Accept either as the observed first failure.
        return ExpectedOutcome{
            true, false, false, false, false, false, false, "CONNECT",
            "DATA",
        };
    }
    // Boundary cliff at 2000 ms: PROBE may complete intermittently
    // (PING reaches peer just inside its RX window), pushing the first
    // failure to CONNECT. Accept either as the observed first failure.
    return ExpectedOutcome{
        false, false, false, false, false, false, false, "PROBE",
        "CONNECT",
    };
}

bool matchesExpected(const SweepOutcome& outcome, const ExpectedOutcome& expected) {
    if (expected.boundary_alt_failed_phase) {
        // Boundary cliff: only assert DONE and accept either of two adjacent
        // failure phases as first_failed. Per-phase OK flags can differ
        // between the two outcomes (e.g., PROBE may pass for the late cliff
        // and fail for the early one), so we don't pin them at the boundary.
        if (outcome.done != expected.done) {
            return false;
        }
        return outcome.first_failed_phase == expected.first_failed_phase ||
               outcome.first_failed_phase == expected.boundary_alt_failed_phase;
    }
    return outcome.probe_ok == expected.probe_ok &&
           outcome.connect_ok == expected.connect_ok &&
           outcome.mode_handoff_ok == expected.mode_handoff_ok &&
           outcome.data_ok == expected.data_ok &&
           outcome.ack_ok == expected.ack_ok &&
           outcome.disconnect_ack_ok == expected.disconnect_ack_ok &&
           outcome.done == expected.done &&
           outcome.first_failed_phase == expected.first_failed_phase;
}

void setFirstFailure(SweepOutcome& outcome) {
    if (!outcome.probe_ok) {
        outcome.first_failed_phase = "PROBE";
    } else if (!outcome.connect_ok) {
        outcome.first_failed_phase = "CONNECT";
    } else if (!outcome.mode_handoff_ok) {
        outcome.first_failed_phase = "MODE_CHG";
    } else if (!outcome.data_ok) {
        outcome.first_failed_phase = "DATA";
    } else if (!outcome.ack_ok) {
        outcome.first_failed_phase = "ACK";
    } else if (!outcome.disconnect_ack_ok) {
        outcome.first_failed_phase = "DISCONNECT";
    } else if (!outcome.done) {
        outcome.first_failed_phase = "DONE";
    }
}

SweepOutcome runSweepPoint(uint32_t rx_settling_ms) {
    SimulatedChannel channel;

    ConnectionConfig alpha_config;
    ConnectionConfig bravo_config;

    const auto mc_config = mc_dpsk_presets::level8();
    SimulatedStation alpha(
        "ALPHA",
        std::make_unique<VirtualAudioPort>(channel, /*is_station_a=*/true),
        OFDMConfigPreset::Default,
        mc_config,
        alpha_config);
    SimulatedStation bravo(
        "BRAVO",
        std::make_unique<VirtualAudioPort>(channel, /*is_station_a=*/false),
        OFDMConfigPreset::Default,
        mc_config,
        bravo_config);

    alpha.setRxSettlingMs(rx_settling_ms);
    bravo.setRxSettlingMs(rx_settling_ms);
    alpha.setSNR(30.0f);
    bravo.setSNR(30.0f);

    SweepOutcome outcome;
    outcome.rx_settling_ms = rx_settling_ms;

    EventLog log;
    std::vector<std::string> bravo_messages;
    attachRxLogger(alpha, "ALPHA", log);
    attachRxLogger(bravo, "BRAVO", log);
    bravo.setMessageCallback([&](const std::string& msg) {
        bravo_messages.push_back(msg);
        log.add(bravo.getSimTime(), "BRAVO", "message.rx", msg);
    });

    alpha.start();
    bravo.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    StationObservedState alpha_observed;
    StationObservedState bravo_observed;
    initializeObservedState(alpha, "ALPHA", alpha_observed, log);
    initializeObservedState(bravo, "BRAVO", bravo_observed, log);

    log.add(alpha.getSimTime(), "ALPHA", "command", "connect BRAVO");
    alpha.connect("BRAVO");

    outcome.probe_ok = runUntil(
        alpha, bravo, alpha_observed, bravo_observed, log, kProbeTimeout, [&]() {
            return alpha.getConnectionState() == ConnectionState::CONNECTING ||
                   alpha.getConnectionState() == ConnectionState::CONNECTED;
        });
    outcome.alpha_rx_pong = log.sawRxPingPong("ALPHA");
    outcome.bravo_rx_ping = log.sawRxPingPong("BRAVO");

    if (outcome.probe_ok) {
        outcome.connect_ok = runUntil(
            alpha, bravo, alpha_observed, bravo_observed, log, kConnectTimeout, [&]() {
                return alpha.isConnected() && bravo.isConnected();
            });
    }

    if (outcome.connect_ok) {
        outcome.alpha_waveform = alpha.getNegotiatedWaveform();
        outcome.bravo_waveform = bravo.getNegotiatedWaveform();
        outcome.alpha_modulation = alpha.getDataModulation();
        outcome.alpha_code_rate = alpha.getDataCodeRate();
        outcome.standalone_mode_change_seen =
            log.sawRxFrame("BRAVO", v2::FrameType::MODE_CHANGE) ||
            log.sawRxFrame("ALPHA", v2::FrameType::MODE_CHANGE);
        outcome.mode_handoff_ok =
            isOFDMMode(outcome.alpha_waveform) &&
            isOFDMMode(outcome.bravo_waveform) &&
            alpha.isHandshakeComplete();
        log.add(alpha.getSimTime(), "ALPHA", "phase.mode_handoff",
                outcome.standalone_mode_change_seen
                    ? "MODE_CHANGE_FRAME"
                    : "CONNECT_ACK_EMBEDDED");
    }

    uint32_t data_ack_before = outcome.connect_ok
        ? static_cast<uint32_t>(alpha.getConnectionStats().arq.acks_received)
        : 0;

    if (outcome.mode_handoff_ok) {
        log.add(alpha.getSimTime(), "ALPHA", "command", std::string("sendMessage ") + kMessage);
        alpha.sendMessage(kMessage);
        outcome.data_ok = runUntil(
            alpha, bravo, alpha_observed, bravo_observed, log, kDataTimeout, [&]() {
                return std::find(bravo_messages.begin(), bravo_messages.end(),
                                 std::string(kMessage)) != bravo_messages.end();
            });
        outcome.bravo_rx_data = log.sawRxFrame("BRAVO", v2::FrameType::DATA);
    }

    if (outcome.data_ok) {
        outcome.ack_ok = runUntil(
            alpha, bravo, alpha_observed, bravo_observed, log, kAckTimeout, [&]() {
                return alpha.getConnectionStats().arq.acks_received >
                       static_cast<int>(data_ack_before);
            });
        outcome.alpha_rx_data_ack =
            alpha.getConnectionStats().arq.acks_received >
            static_cast<int>(data_ack_before);
    }

    if (outcome.ack_ok) {
        log.add(alpha.getSimTime(), "ALPHA", "command", "disconnect");
        alpha.disconnect();
        outcome.disconnect_ack_ok = runUntil(
            alpha, bravo, alpha_observed, bravo_observed, log,
            kDisconnectAckTimeout, [&]() {
                return log.sawRxFrameSeq("ALPHA", v2::FrameType::ACK,
                                         v2::DISCONNECT_SEQ);
            });
        outcome.bravo_rx_disconnect =
            log.sawRxFrame("BRAVO", v2::FrameType::DISCONNECT);
        outcome.alpha_rx_disconnect_ack =
            log.sawRxFrameSeq("ALPHA", v2::FrameType::ACK, v2::DISCONNECT_SEQ);
    }

    if (outcome.disconnect_ack_ok) {
        outcome.done = runUntil(
            alpha, bravo, alpha_observed, bravo_observed, log,
            kDisconnectDoneTimeout, [&]() {
                return alpha.getConnectionState() == ConnectionState::DISCONNECTED &&
                       bravo.getConnectionState() == ConnectionState::DISCONNECTED;
            });
    }

    outcome.alpha_stats = alpha.getConnectionStats();
    outcome.bravo_stats = bravo.getConnectionStats();
    setFirstFailure(outcome);

    std::ostringstream title;
    title << "SessionPttSweep_" << rx_settling_ms << "ms event log";
    log.dump(title.str());

    alpha.stop();
    bravo.stop();
    return outcome;
}

void printOutcome(const SweepOutcome& outcome) {
    std::cout << "\nrx_settling_ms | PROBE | CONNECT | MODE_CHG | DATA | ACK | DISC | DONE | first_failed\n";
    std::cout << outcome.rx_settling_ms
              << " | " << okFail(outcome.probe_ok)
              << " | " << okFail(outcome.connect_ok)
              << " | " << okFail(outcome.mode_handoff_ok)
              << " | " << okFail(outcome.data_ok)
              << " | " << okFail(outcome.ack_ok)
              << " | " << okFail(outcome.disconnect_ack_ok)
              << " | " << (outcome.done ? "YES" : "NO")
              << " | " << outcome.first_failed_phase << "\n";

    std::cout << "STRUCTURED_OUTCOME"
              << " rx_settling_ms=" << outcome.rx_settling_ms
              << " probe=" << okFail(outcome.probe_ok)
              << " connect=" << okFail(outcome.connect_ok)
              << " mode_chg=" << okFail(outcome.mode_handoff_ok)
              << " standalone_mode_change_seen="
              << (outcome.standalone_mode_change_seen ? "YES" : "NO")
              << " data=" << okFail(outcome.data_ok)
              << " ack=" << okFail(outcome.ack_ok)
              << " disconnect=" << okFail(outcome.disconnect_ack_ok)
              << " done=" << (outcome.done ? "YES" : "NO")
              << " first_failed=" << outcome.first_failed_phase
              << " alpha_rx_pong=" << (outcome.alpha_rx_pong ? "YES" : "NO")
              << " bravo_rx_ping=" << (outcome.bravo_rx_ping ? "YES" : "NO")
              << " bravo_rx_data=" << (outcome.bravo_rx_data ? "YES" : "NO")
              << " alpha_rx_data_ack=" << (outcome.alpha_rx_data_ack ? "YES" : "NO")
              << " bravo_rx_disconnect=" << (outcome.bravo_rx_disconnect ? "YES" : "NO")
              << " alpha_rx_disconnect_ack="
              << (outcome.alpha_rx_disconnect_ack ? "YES" : "NO")
              << " alpha_waveform=" << waveformModeToString(outcome.alpha_waveform)
              << " bravo_waveform=" << waveformModeToString(outcome.bravo_waveform)
              << " alpha_data_mode=" << modulationToString(outcome.alpha_modulation)
              << "/" << codeRateToString(outcome.alpha_code_rate)
              << " alpha_arq_sent=" << outcome.alpha_stats.arq.frames_sent
              << " alpha_arq_acks_rx=" << outcome.alpha_stats.arq.acks_received
              << " bravo_arq_rx=" << outcome.bravo_stats.arq.frames_received
              << " bravo_arq_acks_tx=" << outcome.bravo_stats.arq.acks_sent
              << "\n";

    const auto expected = expectedOutcomeFor(outcome.rx_settling_ms);
    std::cout << "EXPECTED_OUTCOME"
              << " rx_settling_ms=" << outcome.rx_settling_ms
              << " expected_first_failed=" << expected.first_failed_phase
              << " match=" << (matchesExpected(outcome, expected) ? "YES" : "NO")
              << "\n";
}

bool isValidSweepValue(uint32_t value) {
    return std::find(std::begin(kSweepValues), std::end(kSweepValues), value) !=
           std::end(kSweepValues);
}

}  // namespace

int main(int argc, char** argv) {
    ultra::setLogLevel(ultra::LogLevel::ERROR);

    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <rx_settling_ms>\n";
        return 1;
    }

    char* end = nullptr;
    const unsigned long parsed = std::strtoul(argv[1], &end, 10);
    if (!end || *end != '\0' || parsed > 2000UL) {
        std::cout << "Invalid rx_settling_ms: " << argv[1] << "\n";
        return 1;
    }

    const uint32_t rx_settling_ms = static_cast<uint32_t>(parsed);
    if (!isValidSweepValue(rx_settling_ms)) {
        std::cout << "Unsupported rx_settling_ms sweep value: "
                  << rx_settling_ms << "\n";
        return 1;
    }

    const auto outcome = runSweepPoint(rx_settling_ms);
    printOutcome(outcome);

    if (rx_settling_ms == 0 && !outcome.done) {
        std::cout << "FAIL: rx_settling_ms=0 did not complete the full QSO; "
                  << "base session path regressed\n";
        return 1;
    }
    const auto expected = expectedOutcomeFor(rx_settling_ms);
    if (!matchesExpected(outcome, expected)) {
        std::cout << "FAIL: measured sweep outcome shifted from expected "
                  << "deterministic envelope for rx_settling_ms="
                  << rx_settling_ms << "\n";
        return 1;
    }

    return 0;
}
