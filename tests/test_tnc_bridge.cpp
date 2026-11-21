#include "gui/audio_engine.hpp"
#include "tnc/tnc_bridge.hpp"

#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using ultra::Bytes;
using ultra::CodeRate;
using ultra::Modulation;
using ultra::SNRSource;
using ultra::protocol::ConnectionState;
using ultra::protocol::WaveformMode;
using ultra::tnc::ProtocolEnginePort;
using ultra::tnc::State;
using ultra::tnc::TNCBridge;
using ultra::tnc::TNCBridgeEventSink;

namespace {

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

struct FakeProtocolEngine : ProtocolEnginePort {
    std::string local_call;
    std::string remote_call;
    bool auto_accept = false;
    bool connect_result = true;
    int set_local_calls = 0;
    int connect_calls = 0;
    int disconnect_calls = 0;
    int abort_calls = 0;
    int set_preferred_calls = 0;
    std::vector<Bytes> sent_binary;
    std::vector<std::string> sent_files;
    size_t backlog_bytes = 0;
    float measured_snr = 12.4f;
    SNRSource measured_snr_source = SNRSource::OFDM_BROADBAND;
    ConnectionState state = ConnectionState::DISCONNECTED;
    WaveformMode preferred_mode = WaveformMode::AUTO;
    WaveformMode negotiated_mode = WaveformMode::OFDM_CHIRP;

    ConnectionChangedCallback connection_cb;
    IncomingCallCallback incoming_cb;
    DataReceivedCallback data_cb;
    FileReceivedCallback file_cb;

    void setLocalCallsign(const std::string& call) override {
        ++set_local_calls;
        local_call = call;
    }

    std::string getLocalCallsign() const override {
        return local_call;
    }

    void setAutoAccept(bool enabled) override {
        auto_accept = enabled;
    }

    bool connect(const std::string& remote) override {
        ++connect_calls;
        remote_call = remote;
        if (connect_result) {
            state = ConnectionState::PROBING;
        }
        return connect_result;
    }

    void disconnect() override {
        ++disconnect_calls;
        state = ConnectionState::DISCONNECTING;
    }

    void abortTxNow() override {
        ++abort_calls;
    }

    bool sendBinary(const Bytes& data) override {
        sent_binary.push_back(data);
        return true;
    }

    bool sendFile(const std::string& path) override {
        sent_files.push_back(path);
        return true;
    }

    size_t getTxBacklogBytes() const override {
        return backlog_bytes;
    }

    ConnectionState getState() const override {
        return state;
    }

    std::string getRemoteCallsign() const override {
        return remote_call;
    }

    float getMeasuredSNR() const override {
        return measured_snr;
    }

    SNRSource getMeasuredSNRSource() const override {
        return measured_snr_source;
    }

    WaveformMode getNegotiatedMode() const override {
        return negotiated_mode;
    }

    void setPreferredMode(WaveformMode mode) override {
        ++set_preferred_calls;
        preferred_mode = mode;
    }

    ultra::protocol::ConnectionStats stats {};
    Modulation data_modulation = Modulation::DQPSK;
    CodeRate data_code_rate = CodeRate::R1_2;

    ultra::protocol::ConnectionStats getStats() const override {
        return stats;
    }

    Modulation getDataModulation() const override {
        return data_modulation;
    }

    CodeRate getDataCodeRate() const override {
        return data_code_rate;
    }

    void setConnectionChangedCallback(ConnectionChangedCallback cb) override {
        connection_cb = std::move(cb);
    }

    void setIncomingCallCallback(IncomingCallCallback cb) override {
        incoming_cb = std::move(cb);
    }

    void setDataReceivedCallback(DataReceivedCallback cb) override {
        data_cb = std::move(cb);
    }

    void setFileReceivedCallback(FileReceivedCallback cb) override {
        file_cb = std::move(cb);
    }

    void emitConnection(ConnectionState next, const std::string& info) {
        state = next;
        if (connection_cb) {
            connection_cb(next, info);
        }
    }

    void emitData(const Bytes& bytes, bool more = false) {
        if (data_cb) {
            data_cb(bytes, more);
        }
    }

    void emitIncoming(const std::string& peer) {
        if (incoming_cb) {
            incoming_cb(peer);
        }
    }
};

struct FakeSink : TNCBridgeEventSink {
    struct Connected {
        std::string src;
        std::string dst;
        int bw = 0;
    };

    std::vector<Connected> connected;
    int disconnected = 0;
    std::vector<bool> ptt;
    std::vector<Bytes> data;
    std::vector<int> buffer_levels;
    std::vector<float> snr;
    std::vector<int> bitrate;
    std::vector<std::string> incoming;

    void postModemConnected(const std::string& src, const std::string& dst, int bw) override {
        connected.push_back({src, dst, bw});
    }

    void postModemDisconnected() override {
        ++disconnected;
    }

    void postModemPTT(bool on) override {
        ptt.push_back(on);
    }

    void postModemDataReceived(std::vector<uint8_t> bytes) override {
        data.push_back(std::move(bytes));
    }

    void postModemBufferLevel(int bytes) override {
        buffer_levels.push_back(bytes);
    }

    void postModemSNR(float db) override {
        snr.push_back(db);
    }

    void postModemBitrate(int bps) override {
        bitrate.push_back(bps);
    }

    void postModemIncomingCall(std::string peer) override {
        incoming.push_back(std::move(peer));
    }
};

struct Harness {
    FakeProtocolEngine engine;
    ultra::gui::AudioEngine audio;
    FakeSink sink;
    TNCBridge bridge;

    Harness()
        : bridge(engine, audio) {
        bridge.attachEventSink(&sink);
    }
};

struct Runner {
    int tests_run = 0;
    int tests_failed = 0;

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

} // namespace

int main() {
    Runner runner;

    runner.run("setMyCall forwards primary callsign", [] {
        Harness h;
        h.bridge.setMyCall({"vk2xyz", "VK2A"});
        expect(h.engine.local_call == "VK2XYZ", "local callsign mismatch");
        expect(h.engine.set_local_calls == 1, "setLocalCallsign not called once");
        expect(h.bridge.getState() == State::READY, "bridge should be READY");
    });

    runner.run("empty setMyCall is ignored", [] {
        Harness h;
        h.bridge.setMyCall({});
        expect(h.engine.set_local_calls == 0, "empty MYCALL should not call engine");
    });

    runner.run("setListen toggles auto accept and state", [] {
        Harness h;
        h.bridge.setListen(true);
        expect(h.engine.auto_accept, "LISTEN ON should enable auto accept");
        expect(h.bridge.getState() == State::LISTENING, "bridge should be LISTENING");
        h.bridge.setListen(false);
        expect(!h.engine.auto_accept, "LISTEN OFF should disable auto accept");
        expect(h.bridge.getState() == State::READY, "bridge should be READY");
    });

    // BUG-TNC-BW-FORCES-OFDM: a wideband BW leaves the waveform to the SNR pick (AUTO), so a
    // TNC session below the OFDM floor starts in MC-DPSK like a GUI session.
    runner.run("BW2300 leaves the wideband waveform to the SNR pick", [] {
        Harness h;
        std::vector<WaveformMode> callback_modes;
        h.bridge.setPreferredWaveformChangedCallback(
            [&](WaveformMode mode) { callback_modes.push_back(mode); });
        h.bridge.setBandwidth(500);
        expect(h.engine.preferred_mode == WaveformMode::OFDM_NARROW, "BW500 mapping mismatch");
        h.bridge.setBandwidth(2300);
        expect(h.engine.preferred_mode == WaveformMode::AUTO,
               "BW2300 must not force a waveform (was OFDM_CHIRP)");
        expect(callback_modes == std::vector<WaveformMode>({WaveformMode::OFDM_NARROW,
                                                           WaveformMode::AUTO}),
               "preferred waveform callback mismatch");
    });

    runner.run("BW500 maps to OFDM_NARROW", [] {
        Harness h;
        h.bridge.setBandwidth(500);
        expect(h.engine.preferred_mode == WaveformMode::OFDM_NARROW, "BW500 mapping mismatch");
    });

    runner.run("BW2750 is accepted as an automatic wideband session", [] {
        Harness h;
        h.bridge.setMyCall({"VK2A"});
        h.bridge.setBandwidth(500);
        h.bridge.setBandwidth(2750);
        h.bridge.start();
        h.engine.emitConnection(ConnectionState::CONNECTED, "VK2B");
        expect(h.engine.preferred_mode == WaveformMode::AUTO, "BW2750 mapping mismatch");
        expect(h.sink.bitrate == std::vector<int>({2208}),
               "BW2750 posts the wideband OFDM nominal bitrate");
        expect(h.sink.connected.size() == 1, "connected event missing");
        expect(h.sink.connected[0].bw == 2750, "connected bandwidth should preserve requested BW2750");
    });

    runner.run("startConnect sets source and calls engine connect with destination", [] {
        Harness h;
        h.bridge.startConnect("VK2A", "VK2B");
        expect(h.engine.local_call == "VK2A", "source callsign not applied");
        expect(h.engine.remote_call == "VK2B", "destination not passed to connect");
        expect(h.engine.connect_calls == 1, "connect not called once");
        expect(h.bridge.getState() == State::CONNECTING, "bridge should be CONNECTING");
    });

    runner.run("failed startConnect posts disconnected", [] {
        Harness h;
        h.engine.connect_result = false;
        h.bridge.startConnect("VK2A", "VK2B");
        expect(h.sink.disconnected == 1, "failed connect should post disconnected");
        expect(h.bridge.getState() == State::READY, "failed connect should return READY");
    });

    runner.run("sendBinary forwards payload", [] {
        Harness h;
        h.bridge.sendBinary({1, 2, 3, 4});
        expect(h.engine.sent_binary.size() == 1, "sendBinary not forwarded");
        expect(h.engine.sent_binary[0] == Bytes({1, 2, 3, 4}), "payload mismatch");
    });

    runner.run("getTxBacklogBytes returns engine backlog", [] {
        Harness h;
        h.engine.backlog_bytes = 321;
        expect(h.bridge.getTxBackloggBytes() == 321, "ABI backlog getter mismatch");
        expect(h.bridge.getTxBacklogBytes() == 321, "helper backlog getter mismatch");
    });

    runner.run("disconnect and abort forward control calls", [] {
        Harness h;
        h.bridge.disconnect();
        h.bridge.abort();
        expect(h.engine.disconnect_calls == 2, "disconnect should be called directly and by abort");
        expect(h.engine.abort_calls == 1, "abortTxNow not called");
    });

    runner.run("Outbound CONNECTED line lists us first (initiator)", [] {
        Harness h;
        h.bridge.setMyCall({"VK2A"});
        h.bridge.start();
        // Drive startConnect first so previous state is CONNECTING.
        h.bridge.startConnect("VK2A", "VK2B");
        h.engine.emitConnection(ConnectionState::CONNECTED, "VK2B");
        expect(h.sink.connected.size() == 1, "connected event not posted");
        expect(h.sink.connected[0].src == "VK2A", "outbound: src should be us (initiator)");
        expect(h.sink.connected[0].dst == "VK2B", "outbound: dst should be peer (responder)");
        expect(h.sink.connected[0].bw == 2300, "connected bandwidth mismatch");
        expect(h.sink.bitrate == std::vector<int>({2208}), "bitrate event mismatch");
    });
    runner.run("Inbound CONNECTED line lists peer first (initiator)", [] {
        Harness h;
        h.bridge.setMyCall({"VK2A"});
        h.bridge.start();
        // No startConnect — peer dialed us. State stays READY → CONNECTED.
        // pat-vara dispatches CONNECTED by parts[2] == myCall to detect
        // inbound, so the line must read "<peer> <us> <bw>".
        h.engine.emitConnection(ConnectionState::CONNECTED, "VK2B");
        expect(h.sink.connected.size() == 1, "connected event not posted");
        expect(h.sink.connected[0].src == "VK2B", "inbound: src should be peer (initiator)");
        expect(h.sink.connected[0].dst == "VK2A", "inbound: dst should be us (responder)");
    });

    runner.run("ProtocolEngine disconnected callback posts modem disconnected", [] {
        Harness h;
        h.bridge.setMyCall({"VK2A"});
        h.bridge.start();
        h.engine.emitConnection(ConnectionState::PROBING, "VK2B");
        h.engine.emitConnection(ConnectionState::DISCONNECTED, "timeout");
        expect(h.sink.disconnected == 1, "disconnected event not posted");
    });

    runner.run("ProtocolEngine data callback posts data", [] {
        Harness h;
        h.engine.backlog_bytes = 77;
        h.bridge.start();
        h.engine.emitData({9, 8, 7});
        expect(h.sink.data.size() == 1, "data event not posted");
        expect(h.sink.data[0] == Bytes({9, 8, 7}), "data payload mismatch");
        // tick() polls TX backlog and emits BUFFER events; start() runs
        // an initial tick so the first reading goes out. We assert the
        // shape rather than emptiness now.
        expect(h.sink.buffer_levels == std::vector<int>({77}),
               "expected initial BUFFER 77 from start()'s tick");
    });
    runner.run("Backlog change emits BUFFER event on tick", [] {
        Harness h;
        h.engine.backlog_bytes = 100;
        h.bridge.start();
        h.sink.buffer_levels.clear();  // discard the start() tick reading
        h.engine.backlog_bytes = 0;
        h.bridge.tick(50);
        expect(h.sink.buffer_levels == std::vector<int>({0}),
               "expected BUFFER 0 after backlog drained");
        // No further emit if backlog stays the same
        h.bridge.tick(50);
        expect(h.sink.buffer_levels == std::vector<int>({0}),
               "no duplicate BUFFER emit when value unchanged");
    });

    runner.run("ProtocolEngine incoming call callback posts incoming call", [] {
        Harness h;
        h.bridge.start();
        h.engine.emitIncoming("VK2CALL");
        expect(h.sink.incoming == std::vector<std::string>({"VK2CALL"}), "incoming call mismatch");
    });

    runner.run("AudioEngine queue transition posts PTT on and tail-delayed off", [] {
        Harness h;
        h.bridge.start();
        h.audio.queueTxSamples({0.1f, 0.2f, 0.3f});
        h.bridge.tick(0);
        expect(h.sink.ptt == std::vector<bool>({true}), "PTT ON missing");
        h.audio.clearTxQueue();
        h.bridge.tick(199);
        expect(h.sink.ptt == std::vector<bool>({true}), "PTT OFF emitted too early");
        h.bridge.tick(1);
        expect(h.sink.ptt == std::vector<bool>({true, false}), "PTT OFF missing after tail");
    });

    // GitHub #38 issue 3: PTT ON with no PTT OFF while retries went on. Audio that is queued but
    // never played (stalled output device) must not hold the transmitter keyed.
    runner.run("stalled TX audio is dropped and PTT unkeys", [] {
        Harness h;
        h.bridge.start();
        h.audio.queueTxSamples({0.1f, 0.2f, 0.3f});  // no device: never drains
        h.bridge.tick(0);
        expect(h.sink.ptt == std::vector<bool>({true}), "PTT ON missing");
        h.bridge.tick(500);
        expect(!h.audio.isTxQueueEmpty(), "queue dropped before the stall limit");
        h.bridge.tick(500);  // 1000 ms without a sample played
        expect(h.audio.isTxQueueEmpty(), "stalled queue not dropped at the stall limit");
        h.bridge.tick(200);  // PTT tail
        expect(h.sink.ptt == std::vector<bool>({true, false}), "PTT not released after a stall");
    });

    runner.run("setPttChangedCallback fires alongside event sink", [] {
        // Hardware-PTT hook in ultra_tnc relies on this callback. The
        // cmd-port PTT events still fire via the sink; the callback is
        // an additional path for driving a serial RTS/DTR line.
        Harness h;
        std::vector<bool> hardware_ptt;
        h.bridge.setPttChangedCallback([&](bool on) { hardware_ptt.push_back(on); });
        h.bridge.start();
        h.audio.queueTxSamples({0.1f});
        h.bridge.tick(0);
        expect(hardware_ptt == std::vector<bool>({true}),
               "hardware-PTT callback should fire on PTT ON transition");
        h.audio.clearTxQueue();
        h.bridge.tick(250);
        expect(hardware_ptt == std::vector<bool>({true, false}),
               "hardware-PTT callback should fire on PTT OFF after tail");
    });

    runner.run("setPttChangedCallback can be cleared safely", [] {
        Harness h;
        h.bridge.setPttChangedCallback([](bool) {});
        h.bridge.setPttChangedCallback({});  // clear
        h.bridge.start();
        h.audio.queueTxSamples({0.1f});
        h.bridge.tick(0);  // would have called the cleared cb
        // No assertion needed — surviving the tick without crash is the
        // assertion. Sink-side PTT still fires:
        expect(h.sink.ptt == std::vector<bool>({true}), "sink PTT still fires");
    });

    runner.run("setConnectionChangedCallback proxies to engine", [] {
        Harness h;
        bool fired = false;
        h.bridge.setConnectionChangedCallback(
            [&](ConnectionState, const std::string&) { fired = true; });
        // Bridge's wirePECallbacks() runs in start() and registers an
        // engine callback that in turn invokes the user callback.
        h.bridge.start();
        h.engine.emitConnection(ConnectionState::PROBING, "VK2B");
        expect(fired, "user callback should be invoked when engine emits");
    });

    runner.run("getCurrentSNR_db rounds to nearest int", [] {
        Harness h;
        h.engine.measured_snr = 12.4f;
        expect(h.bridge.getCurrentSNR_db() == 12, "12.4 → 12");
        h.engine.measured_snr = 12.6f;
        expect(h.bridge.getCurrentSNR_db() == 13, "12.6 → 13");
        h.engine.measured_snr = std::numeric_limits<float>::quiet_NaN();
        expect(h.bridge.getCurrentSNR_db() == 0, "NaN should map to 0");
    });

    runner.run("getStats fills snapshot from engine + bridge", [] {
        Harness h;
        h.engine.backlog_bytes = 256;
        h.engine.measured_snr = 18.2f;
        h.engine.data_modulation = ultra::Modulation::DQPSK;
        h.engine.data_code_rate = ultra::CodeRate::R1_2;
        ultra::tnc::ModemStats s = h.bridge.getStats();
        expect(s.tx_backlog_bytes == 256, "tx_backlog_bytes mismatch");
        expect(s.snr_db == 18, "snr_db should be rounded engine value");
        expect(s.snr_source == "ofdm_broadband", "snr_source should be named");
        expect(s.modulation == "DQPSK", "modulation string");
        expect(s.code_rate == "R1/2", "code_rate string");
    });

    std::cout << "\nTNCBridge tests run: " << runner.tests_run
              << ", failed: " << runner.tests_failed << "\n";
    return runner.tests_failed == 0 ? 0 : 1;
}
