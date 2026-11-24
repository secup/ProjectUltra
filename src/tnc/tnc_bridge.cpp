#include "tnc_bridge.hpp"

#include "gui/audio_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/waveform_selection.hpp"
#include "tnc_server.hpp"
#include "ultra/logging.hpp"
#include "ultra/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <system_error>
#include <utility>
#include <vector>

namespace ultra::tnc {

namespace {

class RealProtocolEnginePort final : public ProtocolEnginePort {
public:
    explicit RealProtocolEnginePort(protocol::ProtocolEngine& engine)
        : engine_(engine) {}

    void setLocalCallsign(const std::string& call) override {
        engine_.setLocalCallsign(call);
    }

    std::string getLocalCallsign() const override {
        return engine_.getLocalCallsign();
    }

    void setAutoAccept(bool auto_accept) override {
        engine_.setAutoAccept(auto_accept);
    }

    void setAnswerProbes(bool answer) override {
        engine_.setAnswerProbes(answer);
    }

    int connectedAsInitiator() const override { return engine_.connectedAsInitiator(); }

    bool connect(const std::string& remote_call) override {
        return engine_.connect(remote_call);
    }

    void disconnect() override {
        engine_.disconnect();
    }

    void abortTxNow() override {
        engine_.abortTxNow();
    }

    void abortSession() override {
        engine_.abort();
    }

    bool sendBinary(const ultra::Bytes& data) override {
        return engine_.sendBinary(data);
    }

    bool sendFile(const std::string& path) override {
        return engine_.sendFile(path);
    }

    size_t getTxBacklogBytes() const override {
        return engine_.getTxBacklogBytes();
    }

    protocol::ConnectionState getState() const override {
        return engine_.getState();
    }

    std::string getRemoteCallsign() const override {
        return engine_.getRemoteCallsign();
    }

    float getMeasuredSNR() const override {
        return engine_.getMeasuredSNR();
    }

    SNRSource getMeasuredSNRSource() const override {
        return engine_.getMeasuredSNRSource();
    }

    protocol::WaveformMode getNegotiatedMode() const override {
        return engine_.getNegotiatedMode();
    }

    void setPreferredMode(protocol::WaveformMode mode) override {
        engine_.setPreferredMode(mode);
    }

    protocol::ConnectionStats getStats() const override {
        return engine_.getStats();
    }

    Modulation getDataModulation() const override {
        return engine_.getDataModulation();
    }

    CodeRate getDataCodeRate() const override {
        return engine_.getDataCodeRate();
    }

    void setConnectionChangedCallback(ConnectionChangedCallback cb) override {
        engine_.setConnectionChangedCallback(std::move(cb));
    }

    void setIncomingCallCallback(IncomingCallCallback cb) override {
        engine_.setIncomingCallCallback(std::move(cb));
    }

    void setPeerClosedCallback(std::function<void()> cb) override {
        engine_.setPeerClosedCallback(std::move(cb));
    }

    bool peerClosedGraceActive() const override { return engine_.peerClosedGraceActive(); }

    void setSendFailedCallback(std::function<void(const std::string&)> cb) override {
        if (!cb) {
            engine_.setFileSentCallback({});
            engine_.setMessageTxStatusCallback({});
            return;
        }
        engine_.setFileSentCallback([cb](bool success, const std::string& error) {
            if (!success) cb(error.empty() ? "file unit not delivered" : error);
        });
        engine_.setMessageTxStatusCallback(
            [cb](const protocol::Connection::MessageTxStatusEvent& event) {
                if (event.status == protocol::Connection::MessageTxStatus::FAILED) {
                    cb("block not delivered");
                }
            });
    }

    void setDataReceivedCallback(DataReceivedCallback cb) override {
        engine_.setDataReceivedCallback(std::move(cb));
    }

    void setFileReceivedCallback(FileReceivedCallback cb) override {
        engine_.setFileReceivedCallback(std::move(cb));
    }

private:
    protocol::ProtocolEngine& engine_;
};

class TNCServerEventSink final : public TNCBridgeEventSink {
public:
    explicit TNCServerEventSink(TNCServer& server)
        : server_(server) {}

    void postModemConnected(const std::string& src, const std::string& dst, int bw) override {
        server_.postModemConnected(src, dst, bw);
    }

    void postModemDisconnected() override {
        server_.postModemDisconnected();
    }

    void postModemPTT(bool on) override {
        server_.postModemPTT(on);
    }

    void postModemDataReceived(std::vector<uint8_t> bytes) override {
        server_.postModemDataReceived(std::move(bytes));
    }

    void postModemBufferLevel(int bytes) override {
        server_.postModemBufferLevel(bytes);
    }

    void postModemSNR(float db) override {
        server_.postModemSNR(db);
    }

    void postModemBitrate(int bps) override {
        server_.postModemBitrate(bps);
    }

    void postModemIncomingCall(std::string peer) override {
        server_.postModemIncomingCall(std::move(peer));
    }

private:
    TNCServer& server_;
};

int clampSizeToInt(size_t value) {
    if (value > static_cast<size_t>(std::numeric_limits<int>::max())) {
        return std::numeric_limits<int>::max();
    }
    return static_cast<int>(value);
}

} // namespace

TNCBridge::TNCBridge(protocol::ProtocolEngine& engine, gui::AudioEngine& audio)
    : owned_engine_(std::make_unique<RealProtocolEnginePort>(engine)),
      engine_(*owned_engine_),
      audio_(audio) {
    applyListen(false);
}

TNCBridge::TNCBridge(ProtocolEnginePort& engine, gui::AudioEngine& audio)
    : engine_(engine),
      audio_(audio) {
    applyListen(false);
}

// A TNC takes calls only after its host says LISTEN ON (TNCSession starts not listening). The
// engine defaults to auto-accept and answering probes, as the GUI wants: a TNC whose host never
// said LISTEN ON keyed up for every PING it heard and accepted a CONNECT the host was never told
// about (BUG-TNC-ANSWERS-WHILE-NOT-LISTENING).
void TNCBridge::applyListen(bool on) {
    engine_.setAutoAccept(on);
    engine_.setAnswerProbes(on);
}

TNCBridge::~TNCBridge() {
    stop();
}

void TNCBridge::attachServer(TNCServer* server) {
    std::shared_ptr<TNCBridgeEventSink> next;
    if (server) {
        next = std::make_shared<TNCServerEventSink>(*server);
    }
    std::lock_guard<std::mutex> lock(event_sink_mutex_);
    event_sink_ = std::move(next);
}

void TNCBridge::attachEventSink(TNCBridgeEventSink* sink) {
    // External ownership: wrap in shared_ptr with a no-op deleter so
    // the bridge doesn't destroy state it doesn't own. Concurrent
    // readers that copied the previous shared_ptr keep that sink
    // alive (or the no-op-deleter wrapper) until they're done.
    std::shared_ptr<TNCBridgeEventSink> next;
    if (sink) {
        next = std::shared_ptr<TNCBridgeEventSink>(sink, [](TNCBridgeEventSink*) {});
    }
    std::lock_guard<std::mutex> lock(event_sink_mutex_);
    event_sink_ = std::move(next);
}

std::shared_ptr<TNCBridgeEventSink> TNCBridge::snapshotEventSink() const {
    std::lock_guard<std::mutex> lock(event_sink_mutex_);
    return event_sink_;
}

void TNCBridge::setConnectionChangedCallback(ConnectionChangedCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    connection_changed_cb_ = std::move(cb);
}

void TNCBridge::setPreferredWaveformChangedCallback(PreferredWaveformChangedCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    preferred_waveform_changed_cb_ = std::move(cb);
}

void TNCBridge::setPttChangedCallback(PttChangedCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    ptt_changed_cb_ = std::move(cb);
}

void TNCBridge::setLocalCallChangedCallback(LocalCallChangedCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    local_call_changed_cb_ = std::move(cb);
}

void TNCBridge::notifyLocalCallChanged(const std::string& call) {
    LocalCallChangedCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = local_call_changed_cb_;
    }
    if (cb) {
        cb(call);
    }
}

void TNCBridge::setMyCall(const std::vector<std::string>& calls) {
    if (calls.empty()) {
        return;
    }

    const std::string primary = protocol::sanitizeCallsign(calls.front());
    if (primary.empty()) {
        return;
    }

    engine_.setLocalCallsign(primary);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_call_ = primary;
    }
    notifyLocalCallChanged(primary);
    state_.store(State::READY, std::memory_order_release);
}

void TNCBridge::setBandwidth(int hz) {
    const protocol::WaveformMode mode = waveformForBandwidth(hz);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        requested_bw_ = hz;
    }

    engine_.setPreferredMode(mode);

    PreferredWaveformChangedCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = preferred_waveform_changed_cb_;
    }
    if (cb) {
        cb(mode);
    }
}

void TNCBridge::setListen(bool on) {
    applyListen(on);
    state_.store(on ? State::LISTENING : State::READY, std::memory_order_release);
}

void TNCBridge::startConnect(const std::string& src, const std::string& dst) {
    const std::string local = protocol::sanitizeCallsign(src);
    const std::string remote = protocol::sanitizeCallsign(dst);
    if (local.empty() || remote.empty()) {
        return;
    }

    engine_.setLocalCallsign(local);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_call_ = local;
        remote_call_ = remote;
    }
    notifyLocalCallChanged(local);

    // A new call during the re-ACK grace after the peer hung up: the host was told DISCONNECTED
    // already, so the call ends the grace (quietly - the state is READY) and goes out at once. The
    // only cost is to a peer whose ACK faded: it retries its DISCONNECT and gives up on its own.
    if (engine_.peerClosedGraceActive()) {
        // Drop a queued DISCONNECT-ACK copy with it: left in the TX queue, it delayed the PING by
        // its own airtime, the peer's PONG then met this station's transmit-to-receive edge and its
        // first chirp was lost - every quick redial took a 20 s PING retry (TNC marathon: 35 s
        // connects against 15 s).
        audio_.clearTxQueue();
        {
            std::lock_guard<std::mutex> lock(ptt_mutex_);
            postPTT(false);
            ptt_tail_ms_ = 0;
        }
        engine_.abortSession();
    }
    state_.store(State::CONNECTING, std::memory_order_release);
    if (!engine_.connect(remote)) {
        state_.store(State::READY, std::memory_order_release);
        if (auto sink = snapshotEventSink()) {
            sink->postModemDisconnected();
        }
    }
}

void TNCBridge::disconnect() {
    if (engine_.peerClosedGraceActive()) {
        // The peer already closed and the host was told: nothing to send, end the grace quietly.
        engine_.abortSession();
        state_.store(State::READY, std::memory_order_release);
        return;
    }
    state_.store(State::DISCONNECTING, std::memory_order_release);
    engine_.disconnect();
}

void TNCBridge::abort() {
    // Unkey first: the queued audio is dropped, so nothing is left to transmit.
    audio_.clearTxQueue();
    {
        std::lock_guard<std::mutex> lock(ptt_mutex_);
        postPTT(false);
        ptt_tail_ms_ = 0;
    }
    // A hard abort, not disconnect(): disconnect() from CONNECTED starts the DISCONNECT handshake
    // (seconds of retries), during which a new CONNECT is refused (GitHub #38).
    engine_.abortSession();
    state_.store(State::READY, std::memory_order_release);
}

bool TNCBridge::sendBinary(const std::vector<uint8_t>& bytes) {
    if (bytes.empty()) {
        // Nothing to send is a no-op success — the caller has nothing
        // to retry. Distinguishes from engine refusal of real bytes.
        return true;
    }
    return engine_.sendBinary(bytes);
}

bool TNCBridge::sendFile(const std::string& path) {
    return engine_.sendFile(path);
}

int TNCBridge::getTxBackloggBytes() const {
    return getTxBacklogBytes();
}

int TNCBridge::getTxBacklogBytes() const {
    return clampSizeToInt(engine_.getTxBacklogBytes());
}

int TNCBridge::getCurrentSNR_db() const {
    const float snr = engine_.getMeasuredSNR();
    if (!std::isfinite(snr)) {
        return 0;
    }
    return static_cast<int>(std::lround(snr));
}

int TNCBridge::getCurrentBitrate_bps() const {
    const int session = session_rate_bps_.load(std::memory_order_acquire);
    return session > 0 ? session : bitrateEstimate(engine_.getNegotiatedMode());
}

void TNCBridge::reportSessionDataRate(int bps) {
    if (bps <= 0 || session_rate_bps_.exchange(bps, std::memory_order_acq_rel) == bps) {
        return;
    }
    // Before CONNECTED the rate rides the CONNECTED report; after it, a change is its own BITRATE.
    if (state_.load(std::memory_order_acquire) != State::CONNECTED) {
        return;
    }
    if (auto sink = snapshotEventSink()) {
        sink->postModemBitrate(bps);
    }
}

State TNCBridge::getState() const {
    return state_.load(std::memory_order_acquire);
}

ModemStats TNCBridge::getStats() const {
    ModemStats out;
    const protocol::ConnectionStats stats = engine_.getStats();
    out.frames_sent = stats.arq.frames_sent;
    out.frames_received = stats.arq.frames_received;
    out.retransmissions = stats.arq.retransmissions;
    out.timeouts = stats.arq.timeouts;
    out.failed = stats.arq.failed;
    out.out_of_order = stats.arq.out_of_order;
    out.code_rate = codeRateToString(engine_.getDataCodeRate());
    out.modulation = modulationToString(engine_.getDataModulation());
    out.waveform = protocol::waveformModeToString(engine_.getNegotiatedMode());
    out.snr_db = getCurrentSNR_db();
    out.snr_source = snrSourceToString(engine_.getMeasuredSNRSource());
    out.bitrate_bps = getCurrentBitrate_bps();
    out.tx_backlog_bytes = getTxBacklogBytes();
    return out;
}

void TNCBridge::start() {
    if (started_.exchange(true, std::memory_order_acq_rel)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_call_ = engine_.getLocalCallsign();
        remote_call_ = engine_.getRemoteCallsign();
    }
    state_.store(toTNCState(engine_.getState()), std::memory_order_release);
    wirePECallbacks();
    tick(0);
}

void TNCBridge::stop() {
    if (!started_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }

    clearPECallbacks();
    {
        std::lock_guard<std::mutex> lock(ptt_mutex_);
        postPTT(false);
        ptt_tail_ms_ = 0;
    }
}

void TNCBridge::tick(uint32_t elapsed_ms) {
    if (!started_.load(std::memory_order_acquire)) {
        return;
    }
    // Stuck-transmitter guard: queued audio that stops draining (the output device stalled) would
    // hold PTT keyed forever - a host keying a real radio from PTT ON/OFF leaves the transmitter
    // on (GitHub #38: PTT ON, three connect retries, no PTT OFF). A playing device drains its
    // queue continuously; one that has not consumed a sample for kTxStallMs (about 3x the largest
    // device buffer the engine allows, 16384 samples = 341 ms) is not playing. Drop the audio and
    // let the PTT tail unkey.
    const size_t queued = audio_.getTxQueueSize();
    if (queued == 0 || queued < tx_queue_last_size_) {
        tx_queue_stall_ms_ = 0;
    } else {
        tx_queue_stall_ms_ += elapsed_ms;
        if (tx_queue_stall_ms_ >= kTxStallMs) {
            LOG_ERROR("TNC", "TX audio stalled: %zu samples queued, none played for %u ms; dropping "
                             "the queue and unkeying", queued, tx_queue_stall_ms_);
            audio_.clearTxQueue();
            tx_queue_stall_ms_ = 0;
        }
    }
    tx_queue_last_size_ = audio_.getTxQueueSize();
    onAudioQueueState(!audio_.isTxQueueEmpty(), elapsed_ms);

    if (failed_receive_disconnect_.exchange(false, std::memory_order_acq_rel) &&
        engine_.getState() == protocol::ConnectionState::CONNECTED) {
        disconnect();
    }

    // Pat / Winlink need periodic BUFFER N updates to know when their
    // outbound data has finished transmitting. Poll the engine's TX
    // backlog and forward any change. TNCSession rate-limits non-zero
    // BUFFER N emissions to once per kBufferEmitIntervalMs (1 s) so a
    // long send doesn't flood the cmd port; BUFFER 0 transitions
    // bypass the rate limit so Pat's Flush() unblocks immediately.
    const int backlog = clampSizeToInt(engine_.getTxBacklogBytes());
    if (backlog != last_reported_backlog_) {
        last_reported_backlog_ = backlog;
        if (auto sink = snapshotEventSink()) {
            sink->postModemBufferLevel(backlog);
        }
    }
}

void TNCBridge::wirePECallbacks() {
    engine_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        onConnectionChanged(state, info);
    });
    engine_.setDataReceivedCallback([this](const ultra::Bytes& bytes, bool more_data) {
        onDataReceived(bytes, more_data);
    });
    engine_.setFileReceivedCallback(
        [this](const std::string& path, bool success, const std::string& error) {
            onFileReceived(path, success, error);
        });
    engine_.setIncomingCallCallback([this](const std::string& peer) {
        onIncomingCall(peer);
    });
    engine_.setPeerClosedCallback([this]() { onPeerClosed(); });
    engine_.setSendFailedCallback([this](const std::string& what) { onSendFailed(what); });
}

void TNCBridge::clearPECallbacks() {
    engine_.setConnectionChangedCallback({});
    engine_.setDataReceivedCallback({});
    engine_.setFileReceivedCallback({});
    engine_.setIncomingCallCallback({});
    engine_.setPeerClosedCallback({});
    engine_.setSendFailedCallback({});
}

void TNCBridge::onConnectionChanged(protocol::ConnectionState state, const std::string& info) {
    const State tnc_state = toTNCState(state);
    const State previous = state_.exchange(tnc_state, std::memory_order_acq_rel);

    std::string local;
    std::string remote;
    int bw = 2300;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local = local_call_;
        if (state == protocol::ConnectionState::CONNECTED) {
            remote_call_ = info;
        } else if (state == protocol::ConnectionState::PROBING ||
                   state == protocol::ConnectionState::CONNECTING) {
            remote_call_ = info;
        }
        remote = remote_call_;
        bw = requested_bw_;
        if (state == protocol::ConnectionState::DISCONNECTED) {
            inbound_pending_ = false;
            session_rate_bps_.store(0, std::memory_order_release);
        }
    }
    // Inbound = we didn't initiate. Detect this from the prior TNC state:
    // a CONNECTING previous means we issued startConnect (outbound). Anything
    // else (LISTENING / READY / IDLE) is the peer dialing us. The
    // on_incoming_call_ callback only fires when auto_accept is OFF, but
    // pat-vara always sets auto_accept=true and fields the inbound case
    // via the CONNECTED line itself, so we can't rely on that signal.
    // The protocol's own role when it reports one: in a call collision the station that placed a
    // call can end up answering the peer's CONNECT, and both hosts then claimed to be the caller
    // (BUG-SIMULTANEOUS-CALL-LOCKSTEP). The prior-state inference remains the fallback.
    const int role = engine_.connectedAsInitiator();
    const bool inbound = (role >= 0) ? (role == 0) : (previous != State::CONNECTING);

    if (state == protocol::ConnectionState::CONNECTED && previous != State::CONNECTED) {
        if (remote.empty()) {
            remote = info;
        }
        // CONNECTED line must always be "<initiator> <responder> <bw>".
        // When the peer called us (inbound), they are the initiator.
        // Otherwise we initiated, so we are first.
        const std::string& initiator = inbound ? remote : local;
        const std::string& responder = inbound ? local : remote;
        if (auto sink = snapshotEventSink()) {
            sink->postModemConnected(initiator, responder, bw);
            // The negotiated rung's rate when the protocol already reported it, else the
            // bandwidth's nominal rate (the next data-mode report replaces it).
            const int session = session_rate_bps_.load(std::memory_order_acquire);
            sink->postModemBitrate(session > 0 ? session : nominalBitrateForBandwidth(bw));
        }
    } else if (state == protocol::ConnectionState::DISCONNECTED && previous != State::IDLE &&
               previous != State::READY && previous != State::LISTENING) {
        if (auto sink = snapshotEventSink()) {
            sink->postModemDisconnected();
        }
    }

    ConnectionChangedCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = connection_changed_cb_;
    }
    if (cb) {
        cb(state, info);
    }
}

void TNCBridge::onDataReceived(const ultra::Bytes& bytes, bool more_data) {
    (void)more_data;
    if (bytes.empty()) {
        return;
    }

    if (auto sink = snapshotEventSink()) {
        sink->postModemDataReceived(bytes);
    }
}

void TNCBridge::onFileReceived(const std::string& path, bool success,
                               const std::string& error) {
    if (!success) {
        // The host's byte stream now has a hole the far TNC believes it filled (its ARQ
        // acknowledged every frame): carrying on would hand the host corrupt or missing data
        // with no error. End the session - both hosts see DISCONNECTED and retry
        // (BUG-TNC-FAILED-BLOCK-SILENT). Not from here: this runs under the engine lock.
        LOG_ERROR("TNC", "received block failed (%s); ending the session so the hosts retry",
                  error.c_str());
        failed_receive_disconnect_.store(true, std::memory_order_release);
        return;
    }
    if (path.empty()) {
        return;
    }
    // Read the reconstructed transport unit (the wire bytes the far TNC staged for this
    // bulk flush) and hand it to the SAME sink streamed bytes use, so TNCSession decodes
    // (deflate/raw marker) and delivers it out the data port. Then drop the temp file —
    // the transfer is complete.
    std::vector<uint8_t> wire;
    {
        std::ifstream in(path, std::ios::binary | std::ios::ate);
        if (in.good()) {
            const std::streamoff size = in.tellg();
            in.seekg(0, std::ios::beg);
            if (size > 0) {
                wire.resize(static_cast<size_t>(size));
                in.read(reinterpret_cast<char*>(wire.data()), size);
            }
        }
    }
    std::error_code ec;
    std::filesystem::remove(path, ec);
    if (wire.empty()) {
        return;
    }
    if (auto sink = snapshotEventSink()) {
        sink->postModemDataReceived(std::move(wire));
    }
}

void TNCBridge::onIncomingCall(const std::string& peer) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        remote_call_ = peer;
        inbound_pending_ = true;
    }

    if (auto sink = snapshotEventSink()) {
        sink->postModemIncomingCall(peer);
    }
}

// The peer hung up (BUG-TNC-PEER-DISCONNECTED-LATE): its DISCONNECT arrived and was ACKed. The
// protocol stays on the air for its re-ACK grace (~24 s, in case the ACK fades), but for the host
// the session is over now - its peer's host was told at once. Runs under the engine lock: no
// engine calls here. READY first, so the protocol's own DISCONNECTED at the grace's end is not
// reported a second time (onConnectionChanged reports it only from a connected state).
void TNCBridge::onPeerClosed() {
    state_.store(State::READY, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        inbound_pending_ = false;
        session_rate_bps_.store(0, std::memory_order_release);
    }
    if (auto sink = snapshotEventSink()) {
        sink->postModemDisconnected();
    }
}

// A piece of the host's byte stream will never reach the peer: the stream the peer's host gets
// would have a hole while this host saw its BUFFER fall as if delivered. End the session so both
// hosts retry (BUG-TNC-SEND-FAILURE-SILENT). Under the engine lock: the disconnect runs from tick().
void TNCBridge::onSendFailed(const std::string& what) {
    if (state_.load(std::memory_order_acquire) != State::CONNECTED) {
        return;  // the session is already over (records failed at teardown)
    }
    LOG_ERROR("TNC", "outbound data lost (%s); ending the session so the hosts retry", what.c_str());
    failed_receive_disconnect_.store(true, std::memory_order_release);
}

void TNCBridge::onAudioQueueState(bool active, uint32_t elapsed_ms) {
    // Compute the PTT transition decision under the tail-tracking mutex,
    // then release before invoking postPTT() — postPTT calls user
    // callbacks (hardware PTT serial I/O), which could be slow or
    // re-enter bridge state and deadlock if held under ptt_mutex_.
    enum class Action { None, On, Off };
    Action action = Action::None;
    {
        std::lock_guard<std::mutex> lock(ptt_mutex_);
        if (active) {
            ptt_tail_ms_ = 0;
            action = Action::On;
        } else if (!ptt_active_.load(std::memory_order_acquire)) {
            ptt_tail_ms_ = 0;
        } else {
            ptt_tail_ms_ = std::min<uint32_t>(ptt_tail_ms_ + elapsed_ms, kPttTailMs);
            if (ptt_tail_ms_ >= kPttTailMs) {
                ptt_tail_ms_ = 0;
                action = Action::Off;
            }
        }
    }
    if (action == Action::On) {
        postPTT(true);
    } else if (action == Action::Off) {
        postPTT(false);
    }
}

// BWxxx picks the occupied bandwidth, not a waveform. 500 Hz has one waveform family
// (OFDM_NARROW). A wideband session stays AUTO, as the GUI's default does: the responder picks
// MC-DPSK or OFDM_CHIRP from the measured SNR and the waveform follow moves it as the channel
// changes. Until 2026-11-14 BW2300/BW2750 (and the TNC's startup BW2300) forced OFDM_CHIRP, so a
// TNC session below the OFDM floor connected in OFDM and moved almost nothing until the follow
// crossed it to MC-DPSK (AWGN 0 dB: 155 s at 6 B/s; BUG-TNC-BW-FORCES-OFDM).
protocol::WaveformMode TNCBridge::waveformForBandwidth(int hz) {
    if (hz == 500) {
        return protocol::WaveformMode::OFDM_NARROW;
    }
    return protocol::WaveformMode::AUTO;
}

// The BITRATE line at CONNECTED: the nominal rate of the bandwidth's OFDM waveform. It is posted
// from the connection callback, which runs under the engine's mutex, so the negotiated mode
// cannot be read there (getCurrentBitrate_bps() reads it outside the callback).
int TNCBridge::nominalBitrateForBandwidth(int hz) {
    return bitrateEstimate(hz == 500 ? protocol::WaveformMode::OFDM_NARROW
                                     : protocol::WaveformMode::OFDM_CHIRP);
}

int TNCBridge::bitrateEstimate(protocol::WaveformMode mode) {
    return protocol::estimatedBitrateBpsForMode(mode);
}

State TNCBridge::toTNCState(protocol::ConnectionState state) {
    switch (state) {
    case protocol::ConnectionState::DISCONNECTED:
        return State::READY;
    case protocol::ConnectionState::PROBING:
    case protocol::ConnectionState::CONNECTING:
        return State::CONNECTING;
    case protocol::ConnectionState::CONNECTED:
        return State::CONNECTED;
    case protocol::ConnectionState::DISCONNECTING:
        return State::DISCONNECTING;
    }
    return State::IDLE;
}

void TNCBridge::postPTT(bool on) {
    const bool previous = ptt_active_.exchange(on, std::memory_order_acq_rel);
    if (previous == on) {
        return;
    }

    if (auto sink = snapshotEventSink()) {
        sink->postModemPTT(on);
    }

    PttChangedCallback ptt_cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        ptt_cb = ptt_changed_cb_;
    }
    if (ptt_cb) {
        ptt_cb(on);
    }
}

} // namespace ultra::tnc
