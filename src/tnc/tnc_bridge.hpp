#pragma once

#include "modem_adapter.hpp"
#include "protocol/protocol_engine.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace ultra::gui {
class AudioEngine;
}

namespace ultra::tnc {

class TNCServer;

class TNCBridgeEventSink {
public:
    virtual ~TNCBridgeEventSink() = default;

    virtual void postModemConnected(const std::string& src, const std::string& dst, int bw) = 0;
    virtual void postModemDisconnected() = 0;
    virtual void postModemPTT(bool on) = 0;
    virtual void postModemDataReceived(std::vector<uint8_t> bytes) = 0;
    virtual void postModemBufferLevel(int bytes) = 0;
    virtual void postModemSNR(float db) = 0;
    virtual void postModemBitrate(int bps) = 0;
    virtual void postModemIncomingCall(std::string peer) = 0;
};

class ProtocolEnginePort {
public:
    using ConnectionChangedCallback = protocol::ProtocolEngine::ConnectionChangedCallback;
    using IncomingCallCallback = protocol::ProtocolEngine::IncomingCallCallback;
    using DataReceivedCallback = protocol::ProtocolEngine::DataReceivedCallback;
    using FileReceivedCallback = protocol::ProtocolEngine::FileReceivedCallback;

    virtual ~ProtocolEnginePort() = default;

    virtual void setLocalCallsign(const std::string& call) = 0;
    virtual std::string getLocalCallsign() const = 0;
    virtual void setAutoAccept(bool auto_accept) = 0;
    virtual bool connect(const std::string& remote_call) = 0;
    virtual void disconnect() = 0;
    virtual void abortTxNow() = 0;
    virtual bool sendBinary(const ultra::Bytes& data) = 0;
    virtual bool sendFile(const std::string& path) = 0;
    virtual size_t getTxBacklogBytes() const = 0;
    virtual protocol::ConnectionState getState() const = 0;
    virtual std::string getRemoteCallsign() const = 0;
    virtual float getMeasuredSNR() const = 0;
    virtual SNRSource getMeasuredSNRSource() const = 0;
    virtual protocol::WaveformMode getNegotiatedMode() const = 0;
    virtual void setPreferredMode(protocol::WaveformMode mode) = 0;
    virtual protocol::ConnectionStats getStats() const = 0;
    virtual Modulation getDataModulation() const = 0;
    virtual CodeRate getDataCodeRate() const = 0;

    virtual void setConnectionChangedCallback(ConnectionChangedCallback cb) = 0;
    virtual void setIncomingCallCallback(IncomingCallCallback cb) = 0;
    virtual void setDataReceivedCallback(DataReceivedCallback cb) = 0;
    virtual void setFileReceivedCallback(FileReceivedCallback cb) = 0;
};

class TNCBridge : public ModemAdapter {
public:
    using ConnectionChangedCallback = protocol::ProtocolEngine::ConnectionChangedCallback;
    using PreferredWaveformChangedCallback = std::function<void(protocol::WaveformMode mode)>;
    // Fires alongside the existing PTT event-sink notification so a host
    // binary (e.g. ultra_tnc) can drive a hardware PTT line without
    // routing through the TCP cmd port.
    using PttChangedCallback = std::function<void(bool on)>;
    // Fires when the local callsign changes (MYCALL / startConnect) so a host binary can keep
    // the modem's RX address filter in sync with the operator's live callsign.
    using LocalCallChangedCallback = std::function<void(const std::string& call)>;

    TNCBridge(protocol::ProtocolEngine& engine, gui::AudioEngine& audio);
    TNCBridge(ProtocolEnginePort& engine, gui::AudioEngine& audio);
    ~TNCBridge() override;

    void attachServer(TNCServer* server);
    void attachEventSink(TNCBridgeEventSink* sink);

    void setConnectionChangedCallback(ConnectionChangedCallback cb);
    void setPreferredWaveformChangedCallback(PreferredWaveformChangedCallback cb);
    void setPttChangedCallback(PttChangedCallback cb);
    void setLocalCallChangedCallback(LocalCallChangedCallback cb);

    void setMyCall(const std::vector<std::string>& calls) override;
    void setBandwidth(int hz) override;
    void setListen(bool on) override;
    void startConnect(const std::string& src, const std::string& dst) override;
    void disconnect() override;
    void abort() override;
    bool sendBinary(const std::vector<uint8_t>& bytes) override;
    bool sendFile(const std::string& path) override;

    int getTxBackloggBytes() const override;
    int getTxBacklogBytes() const;
    int getCurrentSNR_db() const override;
    int getCurrentBitrate_bps() const override;
    State getState() const override;
    ModemStats getStats() const override;

    void start();
    void stop();
    void tick(uint32_t elapsed_ms);

private:
    void wirePECallbacks();
    void clearPECallbacks();
    void notifyLocalCallChanged(const std::string& call);
    void onConnectionChanged(protocol::ConnectionState state, const std::string& info);
    void onDataReceived(const ultra::Bytes& bytes, bool more_data);
    // A modem FILE TRANSFER completed inbound: the reconstructed file (the wire bytes the
    // far TNC staged) lands on disk; we read it, hand it to the same data-received sink as
    // streamed bytes (so the session decodes + delivers it out the data port), then remove
    // the temp file. This is the RX half of the bulk-stream-over-file-transport path.
    void onFileReceived(const std::string& path, bool success, const std::string& error);
    void onIncomingCall(const std::string& peer);
    void onAudioQueueState(bool active, uint32_t elapsed_ms);

    static protocol::WaveformMode waveformForBandwidth(int hz);
    static int nominalBitrateForBandwidth(int hz);
    static int bitrateEstimate(protocol::WaveformMode mode);
    static State toTNCState(protocol::ConnectionState state);
    void postPTT(bool on);

    // Mutex-guarded snapshot. Returns a copy of the shared_ptr so
    // the caller can drop the lock before invoking sink methods.
    std::shared_ptr<TNCBridgeEventSink> snapshotEventSink() const;

    std::unique_ptr<ProtocolEnginePort> owned_engine_;
    ProtocolEnginePort& engine_;
    gui::AudioEngine& audio_;

    // Shared-ptr ownership so a concurrent reader that's already
    // copied the sink keeps it alive across an attachServer() /
    // attachEventSink() teardown. Mutex protects only the swap, not
    // the sink method calls — readers copy the shared_ptr, drop the
    // mutex, then invoke methods without holding a lock through
    // potentially slow TCP I/O. External raw-pointer sinks are
    // wrapped with a no-op deleter so the bridge doesn't destroy
    // state it doesn't own.
    mutable std::mutex event_sink_mutex_;
    std::shared_ptr<TNCBridgeEventSink> event_sink_;

    mutable std::mutex state_mutex_;
    std::string local_call_;
    std::string remote_call_;
    int requested_bw_ = 2300;
    // True when the upcoming CONNECTED transition is from a peer-initiated
    // call (we received a CONNECT). Used to emit CONNECTED with the
    // correct initiator/responder order. pat-vara dispatches by which
    // callsign sits in parts[1] vs parts[2]: matching local_call in
    // parts[1] is the outbound case, parts[2] is the inbound case.
    bool inbound_pending_ = false;

    std::atomic<State> state_{State::IDLE};
    std::atomic<bool> started_{false};
    std::atomic<bool> ptt_active_{false};
    std::mutex ptt_mutex_;
    uint32_t ptt_tail_ms_ = 0;

    // BUFFER event tracking. Pat / Winlink rely on BUFFER N updates from
    // the modem to know when their outgoing data has actually finished
    // transmitting (BUFFER 0 = TX queue drained). Without this, Pat hangs
    // after each write waiting for a confirmation we never send.
    int last_reported_backlog_ = -1;

    mutable std::mutex callback_mutex_;
    ConnectionChangedCallback connection_changed_cb_;
    PreferredWaveformChangedCallback preferred_waveform_changed_cb_;
    PttChangedCallback ptt_changed_cb_;
    LocalCallChangedCallback local_call_changed_cb_;

    static constexpr uint32_t kPttTailMs = 200;
};

} // namespace ultra::tnc
