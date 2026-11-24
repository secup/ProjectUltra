#include "tnc/tnc_session.hpp"

#include "protocol/compression.hpp"
#include "ultra/version.hpp"

#include <filesystem>
#include <algorithm>
#include <cstddef>
#include <deque>
#include <functional>
#include <iostream>
#include <initializer_list>
#include <stdexcept>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

// The two-station integration case (two ModemEngines pumping audio by hand, no channel) was
// removed 2026-11-23: it predated OTASim and had been disabled since 2026-05-30. Two real
// ultra_tnc stations over ota_simulator now cover it: tools/tnc_scenario.py (reconnect, short,
// pingpong, abort, ...) and CTest UltraTncSimAudio.

using ultra::tnc::ModemAdapter;
using ultra::tnc::ModemStats;
using ultra::tnc::State;
using ultra::tnc::TNCSession;

namespace {

std::string versionLine() {
    return std::string("VERSION ") + ultra::kProjectUltraVersion + "\r";
}

// The host's data is staged and flushed once the port has been quiet for
// TNCSession::kDataTxBulkQuietMs (1500 ms, flush-batching is the default); tick past it.
constexpr uint32_t kBulkQuietFlushMs = 1600;

struct FakeModemAdapter : ModemAdapter {
    struct ConnectCall {
        std::string src;
        std::string dst;
    };

    std::vector<std::vector<std::string>> set_mycall_calls;
    std::vector<int> set_bandwidth_calls;
    std::vector<bool> set_listen_calls;
    std::vector<ConnectCall> start_connect_calls;
    int disconnect_calls = 0;
    int abort_calls = 0;
    std::vector<std::vector<uint8_t>> send_binary_calls;

    int backlog_bytes = 0;
    int snr_db = 0;
    int bitrate_bps = 0;
    State state = State::IDLE;
    ModemStats stats {};

    void setMyCall(const std::vector<std::string>& calls) override {
        set_mycall_calls.push_back(calls);
    }

    void setBandwidth(int hz) override {
        set_bandwidth_calls.push_back(hz);
    }

    void setListen(bool on) override {
        set_listen_calls.push_back(on);
    }

    void startConnect(const std::string& src, const std::string& dst) override {
        start_connect_calls.push_back({src, dst});
    }

    void disconnect() override {
        ++disconnect_calls;
    }

    void abort() override {
        ++abort_calls;
    }

    bool send_binary_should_fail = false;
    bool sendBinary(const std::vector<uint8_t>& bytes) override {
        send_binary_calls.push_back(bytes);
        return !send_binary_should_fail;
    }

    std::vector<std::string> send_file_calls;
    bool sendFile(const std::string& path) override {
        send_file_calls.push_back(path);
        return !send_binary_should_fail;
    }

    int getTxBackloggBytes() const override {
        return backlog_bytes;
    }

    int getCurrentSNR_db() const override {
        return snr_db;
    }

    int getCurrentBitrate_bps() const override {
        return bitrate_bps;
    }

    State getState() const override {
        return state;
    }

    ModemStats getStats() const override {
        return stats;
    }
};

struct Harness {
    FakeModemAdapter modem;
    std::vector<std::string> lines;
    std::vector<std::vector<uint8_t>> data_out;
    TNCSession session;

    Harness()
        : session(
              modem,
              [this](std::string_view line) { lines.emplace_back(line); },
              [this](const std::vector<uint8_t>& bytes) { data_out.push_back(bytes); }) {}

    void clear() {
        lines.clear();
        data_out.clear();
    }
};

struct Runner {
    int tests_run = 0;
    int tests_failed = 0;
    std::string current_group;

    void group(const std::string& name) {
        current_group = name;
        std::cout << "\n[" << name << "]\n";
    }

    void run(const std::string& name, const std::function<void()>& body) {
        ++tests_run;
        try {
            body();
            std::cout << "  PASS " << name << "\n";
        } catch (const std::exception& ex) {
            ++tests_failed;
            std::cout << "  FAIL " << name << ": " << ex.what() << "\n";
        }
    }
};

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expectState(State actual, State expected, const std::string& message) {
    if (actual != expected) {
        throw std::runtime_error(message);
    }
}

void expectLines(const Harness& h, std::initializer_list<std::string> expected) {
    std::vector<std::string> want(expected);
    if (h.lines != want) {
        std::string msg = "line mismatch: got [";
        for (const auto& line : h.lines) {
            msg += line;
            msg += ",";
        }
        msg += "] expected [";
        for (const auto& line : want) {
            msg += line;
            msg += ",";
        }
        msg += "]";
        throw std::runtime_error(msg);
    }
}

void expectNoLines(const Harness& h) {
    expect(h.lines.empty(), "expected no emitted control lines");
}

void enterReady(Harness& h) {
    h.session.handleControlLine("MYCALL VK2XYZ");
    h.clear();
}

void enterListening(Harness& h) {
    enterReady(h);
    h.session.handleControlLine("LISTEN ON");
    h.clear();
}

void enterConnecting(Harness& h) {
    enterReady(h);
    h.session.handleControlLine("CONNECT VK2XYZ VK2ABC");
    h.clear();
}

void enterConnected(Harness& h) {
    enterConnecting(h);
    h.session.onModemConnected("VK2XYZ", "VK2ABC", 2300);
    h.clear();
}


} // namespace

int main() {
    Runner runner;

    runner.group("Parser");
    runner.run("empty line emits nothing", [] {
        Harness h;
        expect(!h.session.handleControlLine(""), "empty line should return false");
        expectNoLines(h);
    });
    runner.run("whitespace line emits nothing", [] {
        Harness h;
        expect(!h.session.handleControlLine("   \t  "), "whitespace line should return false");
        expectNoLines(h);
    });
    runner.run("CRLF residue emits WRONG once", [] {
        Harness h;
        expect(!h.session.handleControlLine("\r\n"), "CRLF residue should be invalid");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("trailing whitespace is trimmed", [] {
        Harness h;
        h.session.handleControlLine("VERSION   ");
        expectLines(h, {versionLine()});
    });
    runner.run("unknown command emits WRONG", [] {
        Harness h;
        h.session.handleControlLine("BOGUS");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("lowercase command parses", [] {
        Harness h;
        h.session.handleControlLine("version");
        expectLines(h, {versionLine()});
    });
    runner.run("leading whitespace is trimmed", [] {
        Harness h;
        h.session.handleControlLine("   VERSION");
        expectLines(h, {versionLine()});
    });
    runner.run("extra space between command and args is accepted", [] {
        Harness h;
        h.session.handleControlLine("MYCALL    VK2XYZ");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::READY, "MYCALL should enter READY");
    });
    runner.run("trailing comment is not supported", [] {
        Harness h;
        h.session.handleControlLine("VERSION # comment");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("embedded newline is invalid", [] {
        Harness h;
        h.session.handleControlLine("VERSION\n");
        expectLines(h, {"WRONG\r"});
    });

    runner.group("MYCALL");
    runner.run("MYCALL from IDLE enters READY", [] {
        Harness h;
        h.session.handleControlLine("MYCALL VK2XYZ");
        expectState(h.session.getState(), State::READY, "state should be READY");
        expect(h.session.getMyCall() == "VK2XYZ", "primary callsign should be stored");
        expectLines(h, {"OK\r"});
        expect(h.modem.set_mycall_calls.size() == 1, "modem setMyCall should be called");
        expect(h.modem.set_mycall_calls[0] == std::vector<std::string>{"VK2XYZ"}, "setMyCall args mismatch");
    });
    runner.run("MYCALL normalizes lowercase callsigns", [] {
        Harness h;
        h.session.handleControlLine("MYCALL vk2xyz");
        expect(h.session.getMyCall() == "VK2XYZ", "callsign should be uppercased");
        expect(h.modem.set_mycall_calls[0][0] == "VK2XYZ", "modem callsign should be uppercased");
    });
    runner.run("MYCALL accepts primary plus four secondaries", [] {
        Harness h;
        h.session.handleControlLine("MYCALL VK2XYZ VK2-1 VK2-2 VK2-3 VK2-4");
        expectLines(h, {"OK\r"});
        expect(h.modem.set_mycall_calls[0].size() == 5, "expected five calls");
        expect(h.modem.set_mycall_calls[0][4] == "VK2-4", "secondary callsign missing");
    });
    runner.run("MYCALL without args is WRONG", [] {
        Harness h;
        h.session.handleControlLine("MYCALL");
        expectLines(h, {"WRONG\r"});
        expectState(h.session.getState(), State::IDLE, "state should remain IDLE");
    });
    runner.run("MYCALL rejects six calls", [] {
        Harness h;
        h.session.handleControlLine("MYCALL VK2XYZ VK2-1 VK2-2 VK2-3 VK2-4 VK2-5");
        expectLines(h, {"WRONG\r"});
        expect(h.modem.set_mycall_calls.empty(), "modem should not be called");
    });
    runner.run("MYCALL rejects invalid punctuation", [] {
        Harness h;
        h.session.handleControlLine("MYCALL invalid_call");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("MYCALL rejects invalid SSID", [] {
        Harness h;
        h.session.handleControlLine("MYCALL VK2XYZ-99");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("MYCALL while connected is WRONG", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("MYCALL VK2NEW");
        expectLines(h, {"WRONG\r"});
        expectState(h.session.getState(), State::CONNECTED, "state should remain CONNECTED");
    });

    runner.group("State Transitions");
    runner.run("LISTEN ON from IDLE is WRONG", [] {
        Harness h;
        h.session.handleControlLine("LISTEN ON");
        expectLines(h, {"WRONG\r"});
        expectState(h.session.getState(), State::IDLE, "state should remain IDLE");
    });
    runner.run("LISTEN ON from READY enters LISTENING", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("LISTEN ON");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::LISTENING, "state should be LISTENING");
        expect(h.modem.set_listen_calls == std::vector<bool>{true}, "listen true not sent");
    });
    runner.run("LISTEN CQ behaves like LISTEN ON", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("LISTEN CQ");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::LISTENING, "state should be LISTENING");
    });
    runner.run("LISTEN OFF from LISTENING returns READY", [] {
        Harness h;
        enterListening(h);
        h.session.handleControlLine("LISTEN OFF");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::READY, "state should be READY");
        expect(h.modem.set_listen_calls == std::vector<bool>{true, false}, "listen false not sent");
    });
    runner.run("LISTEN OFF from READY is an OK no-op", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("LISTEN OFF");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::READY, "state should stay READY");
        expect(h.modem.set_listen_calls.empty(), "READY LISTEN OFF should not call modem");
    });
    runner.run("LISTEN ON survives the end of an inbound session", [] {
        // BUG-TNC-LISTEN-LOST-AFTER-SESSION: the modem still accepts calls after a session ends;
        // the host must still be told about the next one.
        Harness h;
        enterListening(h);
        h.session.onModemConnected("VK2ABC", "VK2XYZ", 2300);
        h.session.onModemDisconnected();
        expectState(h.session.getState(), State::LISTENING, "a standing LISTEN ON must resume");
        h.clear();
        h.session.onModemConnected("VK2DEF", "VK2XYZ", 2300);
        expectState(h.session.getState(), State::CONNECTED, "the second call must reach the host");
    });
    runner.run("LISTEN ON resumes after an outbound session", [] {
        Harness h;
        enterListening(h);
        h.session.handleControlLine("CONNECT VK2XYZ VK2ABC");
        h.session.onModemConnected("VK2XYZ", "VK2ABC", 2300);
        h.session.onModemDisconnected();
        expectState(h.session.getState(), State::LISTENING, "listening must resume after the call");
        expect(!h.modem.set_listen_calls.empty() && h.modem.set_listen_calls.back(),
               "the modem must accept calls again");
    });
    runner.run("MYCALL keeps a standing LISTEN ON", [] {
        Harness h;
        enterListening(h);
        h.session.handleControlLine("MYCALL VK2XYZ");
        expectState(h.session.getState(), State::LISTENING, "MYCALL must not end listening");
    });
    runner.run("LISTEN OFF while CONNECTED is WRONG", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("LISTEN OFF");
        expectLines(h, {"WRONG\r"});
        expectState(h.session.getState(), State::CONNECTED, "state should remain CONNECTED");
    });
    runner.run("CONNECT from READY starts CONNECTING", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("CONNECT VK2XYZ VK2ABC");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::CONNECTING, "state should be CONNECTING");
        expect(h.modem.start_connect_calls.size() == 1, "startConnect should be called");
        expect(h.modem.start_connect_calls[0].src == "VK2XYZ", "source call mismatch");
        expect(h.modem.start_connect_calls[0].dst == "VK2ABC", "dest call mismatch");
    });
    runner.run("CONNECT from IDLE is WRONG", [] {
        Harness h;
        h.session.handleControlLine("CONNECT VK2XYZ VK2ABC");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("CONNECT while CONNECTED is WRONG", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("CONNECT VK2XYZ VK2ABC");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("CONNECT from LISTENING disables listen and starts CONNECTING", [] {
        Harness h;
        enterListening(h);
        h.session.handleControlLine("CONNECT VK2XYZ VK2ABC");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::CONNECTING, "state should be CONNECTING");
        expect(h.modem.set_listen_calls == std::vector<bool>{true, false}, "listen should be disabled");
    });
    runner.run("CONNECT without args is WRONG", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("CONNECT");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("CONNECT rejects invalid callsigns", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("CONNECT VK2XYZ bad_call");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("DISCONNECT from CONNECTED enters DISCONNECTING", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("DISCONNECT");
        expectLines(h, {"OK\r"});
        expectState(h.session.getState(), State::DISCONNECTING, "state should be DISCONNECTING");
        expect(h.modem.disconnect_calls == 1, "disconnect should be called");
    });
    runner.run("DISCONNECT from READY is WRONG", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("DISCONNECT");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("repeated DISCONNECT while DISCONNECTING is OK no-op", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("DISCONNECT");
        h.clear();
        h.session.handleControlLine("DISCONNECT");
        expectLines(h, {"OK\r"});
        expect(h.modem.disconnect_calls == 1, "disconnect should not be called twice");
    });
    runner.run("ABORT from CONNECTING returns READY", [] {
        Harness h;
        enterConnecting(h);
        h.session.handleControlLine("ABORT");
        expectLines(h, {"OK\r", "DISCONNECTED\r"});  // BUG-TNC-ABORT-NO-DISCONNECTED
        expectState(h.session.getState(), State::READY, "state should be READY");
        expect(h.modem.abort_calls == 1, "abort should be called");
    });
    runner.run("ABORT from CONNECTED returns READY", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("ABORT");
        expectLines(h, {"OK\r", "DISCONNECTED\r"});  // BUG-TNC-ABORT-NO-DISCONNECTED
        expectState(h.session.getState(), State::READY, "state should be READY");
        expect(h.modem.abort_calls == 1, "abort should be called");
    });
    runner.run("ABORT from READY is WRONG", [] {
        Harness h;
        enterReady(h);
        h.session.handleControlLine("ABORT");
        expectLines(h, {"WRONG\r"});
    });

    runner.group("Modem Events");
    runner.run("connected event from CONNECTING emits CONNECTED", [] {
        Harness h;
        enterConnecting(h);
        h.session.onModemConnected("VK2XYZ", "VK2ABC", 2300);
        expectLines(h, {"CONNECTED VK2XYZ VK2ABC 2300\r"});
        expectState(h.session.getState(), State::CONNECTED, "state should be CONNECTED");
    });
    runner.run("connected event from LISTENING emits PENDING then CONNECTED", [] {
        Harness h;
        enterListening(h);
        h.session.onModemConnected("VK2A", "VK2B", 2300);
        expectLines(h, {"PENDING\r", "CONNECTED VK2A VK2B 2300\r"});
        expectState(h.session.getState(), State::CONNECTED, "state should be CONNECTED");
    });
    runner.run("incoming call while LISTENING emits PENDING", [] {
        Harness h;
        enterListening(h);
        h.session.onModemIncomingCall("VK2ABC");
        expectLines(h, {"PENDING\r"});
    });
    runner.run("pending incoming disconnect emits CANCELPENDING", [] {
        Harness h;
        enterListening(h);
        h.session.onModemIncomingCall("VK2ABC");
        h.clear();
        h.session.onModemDisconnected();
        expectLines(h, {"CANCELPENDING\r"});
        expectState(h.session.getState(), State::LISTENING, "state should remain LISTENING");
    });
    runner.run("incoming call then connected does not duplicate PENDING", [] {
        Harness h;
        enterListening(h);
        h.session.onModemIncomingCall("VK2ABC");
        h.session.onModemConnected("VK2ABC", "VK2XYZ", 500);
        expectLines(h, {"PENDING\r", "CONNECTED VK2ABC VK2XYZ 500\r"});
    });
    runner.run("disconnect event from CONNECTED emits DISCONNECTED", [] {
        Harness h;
        enterConnected(h);
        h.session.onModemDisconnected();
        expectLines(h, {"DISCONNECTED\r"});
        expectState(h.session.getState(), State::READY, "state should be READY");
    });
    runner.run("disconnect event from DISCONNECTING emits DISCONNECTED", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("DISCONNECT");
        h.clear();
        h.session.onModemDisconnected();
        expectLines(h, {"DISCONNECTED\r"});
        expectState(h.session.getState(), State::READY, "state should be READY");
    });
    runner.run("disconnect event from READY is ignored", [] {
        Harness h;
        enterReady(h);
        h.session.onModemDisconnected();
        expectNoLines(h);
    });
    runner.run("PTT ON event emits exact line", [] {
        Harness h;
        h.session.onModemPTT(true);
        expectLines(h, {"PTT ON\r"});
    });
    runner.run("PTT OFF event emits exact line", [] {
        Harness h;
        h.session.onModemPTT(false);
        expectLines(h, {"PTT OFF\r"});
    });
    runner.run("buffer level emits on first change", [] {
        Harness h;
        h.session.onModemBufferLevel(1024);
        expectLines(h, {"BUFFER 1024\r"});
    });
    runner.run("same buffer level is suppressed", [] {
        Harness h;
        h.session.onModemBufferLevel(1024);
        h.clear();
        h.session.onModemBufferLevel(1024);
        expectNoLines(h);
    });
    runner.run("changed buffer level is rate limited to one second", [] {
        Harness h;
        h.session.onModemBufferLevel(1024);
        h.clear();
        h.session.onModemBufferLevel(2048);
        expectNoLines(h);
        h.session.tick(999);
        expectNoLines(h);
        h.session.tick(1);
        expectLines(h, {"BUFFER 2048\r"});
    });
    runner.run("negative buffer level is clamped to zero", [] {
        Harness h;
        h.session.onModemBufferLevel(-4);
        expectLines(h, {"BUFFER 0\r"});
    });
    runner.run("onModemBufferLevel(0) reports staging bytes when present", [] {
        // Codex review #10: when the modem engine drains to 0 but
        // local TCP staging (data_tx_buffer_) still holds bytes,
        // BUFFER 0 must NOT fire — Pat would believe transmission
        // completed and close the session before staged bytes go
        // out. The level reported must include staging.
        Harness h;
        enterConnected(h);
        h.modem.backlog_bytes = 0;       // engine has nothing
        std::vector<uint8_t> staging(50, 'A');
        h.session.handleDataBytes(staging);  // stages locally, not yet flushed
        h.clear();
        // Engine reports zero; staging holds 50. Level must be 50.
        // Staging was already reported as 50 when it arrived; the engine draining to 0 must not
        // lower the reported level to 0 while those bytes are still staged.
        h.session.onModemBufferLevel(0);
        expectNoLines(h);
        h.session.handleControlLine("BUFFER");
        expectLines(h, {"BUFFER 50\r"});
    });
    runner.run("BUFFER counts the host's bytes, not the unit's wire bytes", [] {
        // BUG-TNC-BUFFER-WIRE-BYTES: a 50-byte write went out as a 51-byte unit (raw marker)
        // and the host saw BUFFER 51.
        Harness h;
        enterConnected(h);
        h.session.handleDataBytes(std::vector<uint8_t>(50, 'A'));
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 1, "the write should be flushed as one unit");
        const int wire = static_cast<int>(h.modem.send_binary_calls[0].size());
        h.clear();
        h.modem.backlog_bytes = wire;
        h.session.onModemBufferLevel(wire);
        h.session.handleControlLine("BUFFER");
        std::string got = h.lines.empty() ? "none" : h.lines.back();
        expect(!h.lines.empty() && h.lines.back() == "BUFFER 50\r",
               "got " + got + " (wire " + std::to_string(wire) + "): the whole unit outstanding must read as the host's 50 bytes");
        h.modem.backlog_bytes = 0;
        h.session.onModemBufferLevel(0);
        h.session.handleControlLine("BUFFER");
        expect(!h.lines.empty() && h.lines.back() == "BUFFER 0\r", "delivered must read 0");
    });
    runner.run("a staged bulk unit stays on disk until its transfer can start", [] {
        // BUG-TNC-STAGED-UNIT-DELETED-EARLY: the next flush deleted the previous unit's file while
        // the modem still held it queued behind the peer's DATA turn; it failed to start and the
        // 4096-byte half of a 5000-byte write was lost.
        Harness h;
        enterConnected(h);
        std::vector<uint8_t> body(5000);
        for (size_t i = 0; i < body.size(); ++i) body[i] = static_cast<uint8_t>(i * 131 + 7);
        h.session.handleDataBytes(body);
        h.session.tick(kBulkQuietFlushMs);
        h.session.handleDataBytes(body);
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_file_calls.size() == 2, "two bulk writes should stage two units");
        expect(std::filesystem::exists(h.modem.send_file_calls[0]),
               "the first unit must survive the second flush");
        h.session.onModemDisconnected();
        expect(!std::filesystem::exists(h.modem.send_file_calls[0]) &&
                   !std::filesystem::exists(h.modem.send_file_calls[1]),
               "the session's end removes every staged unit");
    });
    runner.run("BUFFER 0 transition bypasses rate limit", [] {
        // Pat's Flush() blocks on BUFFER 0. If we let the 1 s rate
        // limit hold the zero transition, Flush() stalls for up to
        // a second after the modem is genuinely idle.
        Harness h;
        h.session.onModemBufferLevel(1024);
        h.clear();
        // Backlog drains immediately on the same tick — must emit
        // BUFFER 0 right away even though only milliseconds have
        // elapsed since the previous emission.
        h.session.onModemBufferLevel(0);
        expectLines(h, {"BUFFER 0\r"});
    });
    runner.run("SNR event is ignored until CHAT ON and CONNECTED", [] {
        Harness h;
        enterConnected(h);
        h.session.onModemSNR(7.5f);
        expectNoLines(h);
    });
    runner.run("SNR event emits with CHAT ON while CONNECTED", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("CHAT ON");
        h.clear();
        h.session.onModemSNR(7.5f);
        expectLines(h, {"SN 7.5\r"});
    });
    runner.run("bitrate event emits VARA-style line", [] {
        Harness h;
        h.session.onModemBitrate(1200);
        expectLines(h, {"BITRATE (0) 1200 BPS\r"});
    });

    runner.group("Data Flow");
    runner.run("data input while CONNECTED calls modem sendBinary after quiet timer", [] {
        Harness h;
        enterConnected(h);
        h.session.handleDataBytes({1, 2, 3});
        expect(h.modem.send_binary_calls.empty(), "sendBinary should not fire immediately");
        h.session.tick(kBulkQuietFlushMs);  // past the 1.5 s bulk quiet flush
        // 0x00 prefix = raw payload (compression OFF by default).
        expect(h.modem.send_binary_calls ==
                   std::vector<std::vector<uint8_t>>{{0x00, 1, 2, 3}},
               "sendBinary bytes mismatch after flush");
    });
    runner.run("multiple TCP chunks coalesce into one sendBinary", [] {
        Harness h;
        enterConnected(h);
        // TCP delivers 50KB in many chunks; TNC must batch into one
        // sendBinary call so Connection::sendPayload doesn't strand
        // mid-transfer fragments.
        h.session.handleDataBytes({1, 2, 3});
        h.session.tick(50);
        h.session.handleDataBytes({4, 5, 6});
        h.session.tick(50);
        h.session.handleDataBytes({7, 8, 9});
        expect(h.modem.send_binary_calls.empty(), "should batch, not fire per chunk");
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 1, "expected exactly one batched sendBinary");
        expect(h.modem.send_binary_calls.front() ==
                   std::vector<uint8_t>{0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9},
               "batched bytes mismatch");
    });
    runner.run("data input while READY is discarded", [] {
        Harness h;
        enterReady(h);
        h.session.handleDataBytes({1, 2, 3});
        expect(h.modem.send_binary_calls.empty(), "sendBinary should not be called");
    });
    runner.run("empty data input while CONNECTED is ignored", [] {
        Harness h;
        enterConnected(h);
        h.session.handleDataBytes({});
        expect(h.modem.send_binary_calls.empty(), "empty send should not call modem");
    });
    runner.run("modem data while CONNECTED goes to data_out", [] {
        Harness h;
        enterConnected(h);
        // 0x00 prefix = raw payload — receiver strips marker, forwards rest.
        h.session.onModemDataReceived({0x00, 9, 8, 7});
        expect(h.data_out == std::vector<std::vector<uint8_t>>{{9, 8, 7}}, "data_out mismatch");
    });
    runner.run("modem data while READY is discarded", [] {
        Harness h;
        enterReady(h);
        h.session.onModemDataReceived({0x00, 9, 8, 7});
        expect(h.data_out.empty(), "data_out should not be called");
    });
    runner.run("empty modem data while CONNECTED is ignored", [] {
        Harness h;
        enterConnected(h);
        h.session.onModemDataReceived({});
        expect(h.data_out.empty(), "empty data_out should not be called");
    });
    runner.run("modem data with unknown marker passes through unchanged", [] {
        Harness h;
        enterConnected(h);
        // Pre-compression peers send raw bytes with no marker. Pass them
        // through so old/new combinations stay roughly compatible.
        h.session.onModemDataReceived({0x42, 9, 8, 7});
        expect(h.data_out == std::vector<std::vector<uint8_t>>{{0x42, 9, 8, 7}},
               "unknown-marker passthrough mismatch");
    });

    runner.group("Compression");
    runner.run("COMPRESSION OFF sends raw with 0x00 marker", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("COMPRESSION OFF");
        h.clear();
        // Highly compressible payload (long run of repeated bytes)
        std::vector<uint8_t> payload(256, 'A');
        h.session.handleDataBytes(payload);
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 1, "expected one sendBinary");
        expect(h.modem.send_binary_calls.front().front() == 0x00,
               "expected raw marker when compression disabled");
        expect(h.modem.send_binary_calls.front().size() == payload.size() + 1,
               "raw payload should be uncompressed plus marker");
    });
    runner.run("COMPRESSION ON shrinks compressible payload", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("COMPRESSION TEXT");
        h.clear();
        std::vector<uint8_t> payload(1024, 'A');  // 1KB of repeated 'A'
        h.session.handleDataBytes(payload);
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 1, "expected one sendBinary");
        const auto& wire = h.modem.send_binary_calls.front();
        expect(wire.front() == 0x01, "expected deflate marker");
        expect(wire.size() < payload.size() / 4,
               "deflated 1KB of repeated bytes should shrink to <256 bytes");
    });
    runner.run("COMPRESSION ON keeps small payloads raw", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("COMPRESSION TEXT");
        h.clear();
        // Below MIN_COMPRESS_SIZE — marker stays 0x00.
        h.session.handleDataBytes({1, 2, 3, 4, 5});
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 1, "expected one sendBinary");
        expect(h.modem.send_binary_calls.front().front() == 0x00,
               "small payloads should stay raw");
    });
    runner.run("COMPRESSION ON falls back to raw if compress doesn't help", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("COMPRESSION TEXT");
        h.clear();
        // Pseudo-random bytes — deflate adds overhead, can't shrink.
        std::vector<uint8_t> payload;
        payload.reserve(256);
        for (int i = 0; i < 256; ++i) {
            payload.push_back(static_cast<uint8_t>((i * 31 + 7) & 0xFF));
        }
        h.session.handleDataBytes(payload);
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 1, "expected one sendBinary");
        // Uncompressible input should ship raw, not as inflated deflate.
        expect(h.modem.send_binary_calls.front().front() == 0x00,
               "uncompressible input should fall back to raw marker");
    });
    runner.run("RX corrupt deflate payload is silently dropped", [] {
        // 0x01 marker says "deflate-compressed payload follows" but
        // garbage bytes follow. The RX path must drop instead of
        // forwarding garbage to the data port.
        Harness h;
        enterConnected(h);
        std::vector<uint8_t> wire = {0x01, 0xFF, 0xFE, 0xFD, 0xFC, 0xFB};
        h.session.onModemDataReceived(wire);
        expect(h.data_out.empty(),
               "corrupt deflate must NOT forward to data port");
    });
    runner.run("encodePayloadForWire: raw marker when compression disabled", [] {
        std::vector<uint8_t> payload(512, 'Z');
        auto wire = TNCSession::encodePayloadForWire(payload, /*compression*/ false);
        expect(wire.size() == payload.size() + 1, "wire is payload + 1-byte marker");
        expect(wire.front() == 0x00, "raw marker when compression disabled");
        expect(std::equal(payload.begin(), payload.end(), wire.begin() + 1),
               "payload bytes must be preserved verbatim");
    });
    runner.run("encodePayloadForWire: raw marker below MIN_COMPRESS_SIZE", [] {
        // 8 bytes < MIN_COMPRESS_SIZE (32) — even with compression on,
        // we must skip deflate and stay raw to avoid the marker overhead.
        std::vector<uint8_t> payload = {'h','e','l','l','o','!','!','!'};
        auto wire = TNCSession::encodePayloadForWire(payload, /*compression*/ true);
        expect(wire.front() == 0x00, "small payload stays raw");
        expect(wire.size() == payload.size() + 1, "no compression for small payload");
    });
    runner.run("encodePayloadForWire: deflate marker when compression saves bytes", [] {
        // 1024 'Z' bytes is highly compressible — deflate should
        // beat raw-plus-marker easily.
        std::vector<uint8_t> payload(1024, 'Z');
        auto wire = TNCSession::encodePayloadForWire(payload, /*compression*/ true);
        expect(wire.front() == 0x01, "compressible payload gets deflate marker");
        expect(wire.size() < payload.size() + 1,
               "deflate output must be smaller than raw+marker");
    });
    runner.run("encodePayloadForWire: falls back to raw when deflate would expand", [] {
        // Pseudo-random uncompressible payload above MIN_COMPRESS_SIZE.
        // Compression should be attempted but rejected, falling back
        // to raw so we never ship MORE bytes than the input.
        std::vector<uint8_t> payload;
        for (int i = 0; i < 256; ++i) {
            payload.push_back(static_cast<uint8_t>((i * 31 + 7) & 0xFF));
        }
        auto wire = TNCSession::encodePayloadForWire(payload, /*compression*/ true);
        expect(wire.front() == 0x00, "uncompressible payload falls back to raw");
        expect(wire.size() == payload.size() + 1,
               "raw fallback is payload + 1-byte marker");
    });
    runner.run("flushDataTxBuffer keeps staging on engine reject", [] {
        // Codex review #15: ModemAdapter::sendBinary now returns bool.
        // When the engine refuses (queue full, not CONNECTED, etc.)
        // staged TCP bytes must NOT be silently dropped — Pat trusts
        // BUFFER N to count them. The session keeps data_tx_buffer_
        // intact so the next quiet-period flush retries.
        Harness h;
        enterConnected(h);
        h.modem.send_binary_should_fail = true;
        std::vector<uint8_t> staged(64, 'X');
        h.session.handleDataBytes(staged);
        h.session.tick(kBulkQuietFlushMs);  // past the 1.5 s bulk quiet flush
        expect(h.modem.send_binary_calls.size() == 1,
               "flush attempt was made");
        // BUFFER snapshot should still report the staged bytes.
        h.clear();
        h.session.handleControlLine("BUFFER");
        expectLines(h, {"BUFFER 64\r"});
        // Now allow success on retry.
        h.modem.send_binary_should_fail = false;
        h.session.tick(kBulkQuietFlushMs);
        expect(h.modem.send_binary_calls.size() == 2,
               "retry attempt should fire after next quiet period");
        h.clear();
        h.session.handleControlLine("BUFFER");
        expectLines(h, {"BUFFER 0\r"});
    });
    runner.run("encodePayloadForWire: empty payload produces single marker byte", [] {
        std::vector<uint8_t> payload;
        auto wire = TNCSession::encodePayloadForWire(payload, /*compression*/ true);
        expect(wire.size() == 1, "empty payload encodes as just the marker");
        expect(wire.front() == 0x00, "empty payload uses raw marker");
    });
    runner.run("RX deflate marker is decompressed", [] {
        Harness h;
        enterConnected(h);
        // Encode a known plaintext via the same path the peer would use.
        const std::vector<uint8_t> plaintext(512, 'Z');
        auto compressed = ultra::protocol::Compression::compress(plaintext);
        expect(static_cast<bool>(compressed), "compression helper should succeed");
        std::vector<uint8_t> wire;
        wire.push_back(0x01);
        wire.insert(wire.end(), compressed->begin(), compressed->end());
        h.session.onModemDataReceived(wire);
        expect(h.data_out.size() == 1, "expected one data_out call");
        expect(h.data_out.front() == plaintext, "decompressed bytes mismatch");
    });


    runner.group("Tick");
    runner.run("30 seconds does not emit IAMALIVE", [] {
        Harness h;
        h.session.tick(30000);
        expectNoLines(h);
    });
    runner.run("60 seconds emits IAMALIVE", [] {
        Harness h;
        h.session.tick(60000);
        expectLines(h, {"IAMALIVE\r"});
    });
    runner.run("two 30 second ticks emit once", [] {
        Harness h;
        h.session.tick(30000);
        h.session.tick(30000);
        expectLines(h, {"IAMALIVE\r"});
    });
    runner.run("120 seconds emits two keepalives", [] {
        Harness h;
        h.session.tick(120000);
        expectLines(h, {"IAMALIVE\r", "IAMALIVE\r"});
    });

    runner.group("Bandwidth");
    runner.run("BW2300 is accepted", [] {
        Harness h;
        h.session.handleControlLine("BW2300");
        expectLines(h, {"OK\r"});
        expect(h.modem.set_bandwidth_calls == std::vector<int>{2300}, "BW2300 not sent");
    });
    runner.run("BW500 is accepted", [] {
        Harness h;
        h.session.handleControlLine("BW500");
        expectLines(h, {"OK\r"});
        expect(h.modem.set_bandwidth_calls == std::vector<int>{500}, "BW500 not sent");
    });
    runner.run("BW2750 is accepted", [] {
        Harness h;
        h.session.handleControlLine("BW2750");
        expectLines(h, {"OK\r"});
        expect(h.modem.set_bandwidth_calls == std::vector<int>{2750}, "BW2750 not sent");
    });
    runner.run("BW1234 is WRONG", [] {
        Harness h;
        h.session.handleControlLine("BW1234");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("BW with args is WRONG", [] {
        Harness h;
        h.session.handleControlLine("BW2300 EXTRA");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("lowercase bandwidth command parses", [] {
        Harness h;
        h.session.handleControlLine("bw2300");
        expectLines(h, {"OK\r"});
    });
    runner.run("bandwidth while CONNECTED is WRONG", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("BW500");
        expectLines(h, {"WRONG\r"});
    });

    runner.group("Queries And No-Ops");
    runner.run("VERSION emits project version string", [] {
        Harness h;
        h.session.handleControlLine("VERSION");
        expectLines(h, {versionLine()});
    });
    runner.run("BUFFER command emits snapshot", [] {
        Harness h;
        h.modem.backlog_bytes = 321;
        h.session.handleControlLine("BUFFER");
        expectLines(h, {"BUFFER 321\r"});
    });
    runner.run("BUFFER includes data_tx_buffer staging bytes", [] {
        // Pat's Flush() blocks on BUFFER 0. If our 200 ms staging
        // buffer (data_tx_buffer_) is invisible to BUFFER reports,
        // Pat can see "all done" while bytes are still in staging.
        // Regression guard for commit 6679641.
        Harness h;
        enterConnected(h);
        h.modem.backlog_bytes = 100;
        // Push bytes into the session's TX staging buffer (handleDataBytes
        // accumulates without flushing until tick).
        h.session.handleDataBytes({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});
        h.clear();
        h.session.handleControlLine("BUFFER");
        expectLines(h, {"BUFFER 110\r"});  // 100 engine + 10 staging
    });
    runner.run("SN command emits snapshot with one decimal", [] {
        Harness h;
        h.modem.snr_db = 8;
        h.session.handleControlLine("SN");
        expectLines(h, {"SN 8.0\r"});
    });
    runner.run("BITRATE command emits snapshot", [] {
        Harness h;
        h.modem.bitrate_bps = 1800;
        h.session.handleControlLine("BITRATE");
        expectLines(h, {"BITRATE (0) 1800 BPS\r"});
    });
    runner.run("STATS command emits ARQ + PHY snapshot", [] {
        Harness h;
        h.modem.stats.frames_sent = 42;
        h.modem.stats.frames_received = 38;
        h.modem.stats.retransmissions = 5;
        h.modem.stats.timeouts = 2;
        h.modem.stats.failed = 0;
        h.modem.stats.out_of_order = 1;
        h.modem.stats.code_rate = "R1_2";
        h.modem.stats.modulation = "DQPSK";
        h.modem.stats.waveform = "OFDM_CHIRP";
        h.modem.stats.snr_db = 15;
        h.modem.stats.snr_source = "ofdm_broadband";
        h.modem.stats.bitrate_bps = 2300;
        h.modem.stats.tx_backlog_bytes = 128;
        enterConnected(h);  // rate/mod/mode/bps describe a session; idle STATS reports none
        h.clear();
        h.session.handleControlLine("STATS");
        expectLines(h, {"STATS frames_sent=42 frames_recv=38 retx=5 timeouts=2 "
                        "failed=0 out_of_order=1 rate=R1_2 mod=DQPSK "
                        "mode=OFDM_CHIRP snr=15 snr_source=ofdm_broadband "
                        "bps=2300 backlog=128\r"});
    });
    runner.run("STATS with arguments is WRONG", [] {
        Harness h;
        h.session.handleControlLine("STATS extra");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("STATS outside a session reports no link and no SNR", [] {
        Harness h;
        h.session.handleControlLine("STATS");
        expectLines(h, {"STATS frames_sent=0 frames_recv=0 retx=0 timeouts=0 "
                        "failed=0 out_of_order=0 rate=none mod=none mode=none snr=none "
                        "snr_source=none bps=0 backlog=0\r"});
    });
    runner.run("STATS in a session with unknown fields reports placeholders", [] {
        Harness h;
        enterConnected(h);
        h.clear();
        h.session.handleControlLine("STATS");
        expectLines(h, {"STATS frames_sent=0 frames_recv=0 retx=0 timeouts=0 "
                        "failed=0 out_of_order=0 rate=? mod=? mode=? snr=none "
                        "snr_source=none bps=0 backlog=0\r"});
    });
    runner.run("COMPRESSION TEXT is OK", [] {
        Harness h;
        h.session.handleControlLine("COMPRESSION TEXT");
        expectLines(h, {"OK\r"});
    });
    runner.run("COMPRESSION invalid mode is WRONG", [] {
        Harness h;
        h.session.handleControlLine("COMPRESSION BAD");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("CHAT ON is OK", [] {
        Harness h;
        h.session.handleControlLine("CHAT ON");
        expectLines(h, {"OK\r"});
    });
    runner.run("CHAT OFF suppresses later SNR events", [] {
        Harness h;
        enterConnected(h);
        h.session.handleControlLine("CHAT ON");
        h.session.handleControlLine("CHAT OFF");
        h.clear();
        h.session.onModemSNR(9.5f);
        expectNoLines(h);
    });
    runner.run("P2P SESSION is OK", [] {
        Harness h;
        h.session.handleControlLine("P2P SESSION");
        expectLines(h, {"OK\r"});
    });
    runner.run("P2P without args is OK", [] {
        Harness h;
        h.session.handleControlLine("P2P");
        expectLines(h, {"OK\r"});
    });
    runner.run("WINLINK SESSION is OK", [] {
        Harness h;
        h.session.handleControlLine("WINLINK SESSION");
        expectLines(h, {"OK\r"});
    });
    runner.run("PUBLIC ON is OK", [] {
        Harness h;
        h.session.handleControlLine("PUBLIC ON");
        expectLines(h, {"OK\r"});
    });
    runner.run("IGNOREKISSDCD accepts any args", [] {
        Harness h;
        h.session.handleControlLine("IGNOREKISSDCD ANYTHING HERE");
        expectLines(h, {"OK\r"});
    });
    runner.run("RETRIES accepts non-negative integer", [] {
        Harness h;
        h.session.handleControlLine("RETRIES 10");
        expectLines(h, {"OK\r"});
    });
    runner.run("RETRIES rejects invalid integer", [] {
        Harness h;
        h.session.handleControlLine("RETRIES nope");
        expectLines(h, {"WRONG\r"});
    });
    runner.run("CALLINT accepts zero", [] {
        Harness h;
        h.session.handleControlLine("CALLINT 0");
        expectLines(h, {"OK\r"});
    });
    runner.run("CWID ON is OK", [] {
        Harness h;
        h.session.handleControlLine("CWID ON");
        expectLines(h, {"OK\r"});
    });
    runner.run("CWID invalid mode is WRONG", [] {
        Harness h;
        h.session.handleControlLine("CWID MAYBE");
        expectLines(h, {"WRONG\r"});
    });

    std::cout << "\nTNCSession unit cases run: " << runner.tests_run << "\n";
    if (runner.tests_failed != 0) {
        std::cout << "TNCSession unit cases failed: " << runner.tests_failed << "\n";
        return 1;
    }

    std::cout << "All TNCSession unit cases passed\n";
    return 0;
}
