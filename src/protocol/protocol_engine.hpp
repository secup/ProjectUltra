#pragma once

#include "connection.hpp"
#include "frame_v2.hpp"
#include <functional>
#include <mutex>
#include <optional>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifdef ERROR
#undef ERROR
#endif
#endif

namespace ultra {
namespace protocol {

#ifdef _WIN32
class ProtocolEngineMutex {
public:
    ProtocolEngineMutex() noexcept = default;
    ProtocolEngineMutex(const ProtocolEngineMutex&) = delete;
    ProtocolEngineMutex& operator=(const ProtocolEngineMutex&) = delete;

    void lock() noexcept { AcquireSRWLockExclusive(&lock_); }
    void unlock() noexcept { ReleaseSRWLockExclusive(&lock_); }

private:
    SRWLOCK lock_ = SRWLOCK_INIT;
};
#else
using ProtocolEngineMutex = std::mutex;
#endif

/**
 * Protocol Engine
 *
 * High-level interface that integrates the protocol stack:
 * - Frame serialization/deserialization (v2 format)
 * - Connection management
 * - ARQ for reliable delivery
 *
 * Connects to ModemEngine for actual TX/RX of audio.
 */
class ProtocolEngine {
public:
    using TxDataCallback =
        std::function<void(const Bytes& data, bool expect_full_ofdm_anchor_after_tx)>;
    using MessageReceivedCallback = std::function<void(const std::string& from, const std::string& text)>;
    using MessageTxStatus = Connection::MessageTxStatus;
    using MessageTxStatusEvent = Connection::MessageTxStatusEvent;
    using MessageTxStatusCallback = Connection::MessageTxStatusCallback;
    using ConnectionChangedCallback = std::function<void(ConnectionState state, const std::string& remote)>;
    using DisconnectTeardownCallback = Connection::DisconnectTeardownCallback;
    using IncomingCallCallback = std::function<void(const std::string& from)>;
    using DataReceivedCallback = Connection::DataReceivedCallback;

    using FileProgressCallback = Connection::FileProgressCallback;
    using FileReceivedCallback = Connection::FileReceivedCallback;
    using FileSentCallback = Connection::FileSentCallback;

    using ModeNegotiatedCallback = Connection::ModeNegotiatedCallback;
    using ConnectWaveformChangedCallback = Connection::ConnectWaveformChangedCallback;

    explicit ProtocolEngine(const ConnectionConfig& config = ConnectionConfig{});

    // --- Configuration ---

    void setLocalCallsign(const std::string& call);
    std::string getLocalCallsign() const;

    void setAutoAccept(bool auto_accept);
    bool getAutoAccept() const;

    // --- Callbacks ---

    void setTxDataCallback(TxDataCallback cb);
    void setMessageReceivedCallback(MessageReceivedCallback cb);
    void setMessageTxStatusCallback(MessageTxStatusCallback cb);
    void setConnectionChangedCallback(ConnectionChangedCallback cb);
    // Fine-grained close phase, including responder grace where ConnectionState
    // intentionally remains CONNECTED. Frontends use this to restrict the PHY
    // and their own egress queues to DISCONNECT / sentinel-ACK traffic only.
    void setDisconnectTeardownCallback(DisconnectTeardownCallback cb);
    void setIncomingCallCallback(IncomingCallCallback cb);
    void setDataReceivedCallback(DataReceivedCallback cb);

    // --- Connection Control ---

    bool connect(const std::string& remote_call);
    void acceptCall();
    void rejectCall();
    void disconnect();
    void abortTxNow();

    // --- Data Transfer ---

    bool sendMessage(const std::string& text);
    bool sendBinary(const Bytes& data);
    bool sendMessages(const std::vector<std::string>& texts);  // Batch: burst-interleaved
    bool isReadyToSend() const;
    size_t getTxBacklogBytes() const;

    // --- File Transfer ---

    bool sendFile(const std::string& filepath);
    void setReceiveDirectory(const std::string& dir);
    void cancelFileTransfer();
    bool isFileTransferInProgress() const;
    // Per-burst LDPC lifting Z from the connection traffic-class policy (27/81).
    // The app pushes this to ModemEngine::setBurstLiftingZ so the TX encoder Z
    // matches the chunker. Forwards to Connection::selectBurstLiftingZ().
    int selectBurstLiftingZ() const { return connection_.selectBurstLiftingZ(); }
    FileTransferProgress getFileProgress() const;
    BurstActivity getBurstActivity() const;

    void setFileProgressCallback(FileProgressCallback cb);
    void setFileReceivedCallback(FileReceivedCallback cb);
    void setFileSentCallback(FileSentCallback cb);

    // --- Modem Interface ---

    void onRxData(const Bytes& data, bool physical_turn_complete = false);
    void onMCDPSKPartialFrame(const v2::PartialFrameCodewords& partial);
    void onAcceptedOFDMDataSync(float sync_correlation);
    // §14.27: a decoded interleaved burst delivered as a unit (group_seq, ordered
    // DATA frames, all-logical-frames-decoded) for the one-way burst transport.
    void onBurstGroupReceived(uint16_t group_seq, const std::vector<Bytes>& frames,
                              bool all_ok, float quality, uint16_t frame_mask = 0xFFFF,
                              bool interleaved = true, uint8_t group_size = 0,
                              bool geometry_proven = false);

    // DESC-SWITCH Phase 1 (ULTRA_DESCRIPTOR_MODE_SWITCH): the receiver's decoder
    // consumed a BURST_HEADER descriptor announcing a different mod/rate — the
    // protocol layer follows it (RX-side applyDataMode; no MODE_CHANGE ACK
    // machinery). Delegates to Connection::onDescriptorModeChange; no-op while
    // the knob is OFF.
    void onAnchoredBurstNoGroup(bool payload_seen);
    // State-only notification: the decoder abandoned a marker-armed candidate
    // without wire-proven group geometry. No ACK or selector sample is emitted.
    void onBurstOutcomeUnknown();
    void setBurstCarrierGammas(const std::vector<float>& gammas);
    void onDescriptorModeChange(Modulation mod, CodeRate rate, int cw_per_frame);

    // §15 step 4d-ii: hand a tone-burst ACK detection from the receiver's
    // always-on ToneBurstAckMonitor (installed in StreamingDecoder) to the
    // protocol layer. Delegates to Connection::onToneBurstAck. Returns
    // true iff the detection matched an in-flight burst-transport group.
    bool onToneBurstAck(
        const ultra::waveform::tone_burst_ack::ToneBurstAckDetection& detection);
    bool isToneBurstAckCandidatePlausible(
        const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& payload) const;
    void tick(uint32_t elapsed_ms);

    // §14.36 Phase 5c live GUI surface: the sender's most recent decode-headroom
    // sample and the last adaptive action (e.g. "rate R3/4 -> R2/3 (q=0.18)" or
    // "hold R3/4 (q=0.85)"). lastGroupQuality < 0 means no sample yet. Safe to
    // call from the GUI thread; both proxy the Connection getters under the
    // protocol engine mutex.
    bool adaptiveRateEnabled() const;
    float lastGroupQuality() const;
    std::string lastAdaptiveAction() const;

    // --- State ---

    ConnectionState getState() const;
    std::string getRemoteCallsign() const;
    bool isConnected() const;
    // True for the station that initiated the current connection. Scripted
    // scenarios use this to give exactly one endpoint teardown ownership.
    bool isInitiator() const;

    ConnectionStats getStats() const;
    void resetStats();
    void reset();

    // --- Waveform Mode ---

    WaveformMode getNegotiatedMode() const;
    // Raw-PHY bitrate estimate for the currently negotiated waveform.
    // Returns 0 if no waveform is negotiated yet. See
    // waveform_selection.hpp::estimatedBitrateBpsForMode.
    int getCurrentBitrate_bps() const;
    void setPreferredMode(WaveformMode mode);
    void setModeCapabilities(uint8_t caps);
    bool isPhyMaskV1Negotiated() const;

    // Session-scoped narrowband override (cleared on disconnect/reset)
    void setNarrowbandOverride(WaveformMode mode);

    // Forced data mode - operator can override SNR-based selection
    // Set before calling connect(). Values are sent in CONNECT frame.
    // 0xFF (AUTO) = let responder decide based on SNR
    void setForcedModulation(Modulation mod);
    void setForcedCodeRate(CodeRate rate);
    void setMCDPSKConfig(int num_carriers, int samples_per_symbol);
    void setForcedFrameCodewords(int cw_count, bool forced = true);
    Modulation getForcedModulation() const;
    CodeRate getForcedCodeRate() const;
    int getForcedFrameCodewords() const;
    void setSoftCombiningHARQ(bool enable);
    fec::SoftCombineBuffer* softCombineBuffer();
    std::optional<fec::SoftCombineBuffer::ProvisionalContext>
    harqProvisionalContext() const;

    void setModeNegotiatedCallback(ModeNegotiatedCallback cb);
    void setConnectWaveformChangedCallback(ConnectWaveformChangedCallback cb);
    WaveformMode getConnectWaveform() const;
    void setInitialConnectWaveform(WaveformMode mode);

    using PhyMaskV1NegotiatedCallback = Connection::PhyMaskV1NegotiatedCallback;
    void setPhyMaskV1NegotiatedCallback(PhyMaskV1NegotiatedCallback cb);

    // Callback when handshake is confirmed (safe to switch to negotiated waveform)
    using HandshakeConfirmedCallback = Connection::HandshakeConfirmedCallback;
    void setHandshakeConfirmedCallback(HandshakeConfirmedCallback cb);

    // --- Adaptive Data Mode ---

    // Set measured SNR from modem (used for adaptive mode selection).
    // data_aided: the value is the MC-DPSK data-aided (fade-averaged) estimate,
    // not the training snapshot — see connectSelectionSnrDb's saturation bound.
    void setMeasuredSNR(float snr_db, SNRSource source = SNRSource::NONE,
                        bool data_aided = false);
    // §6: fade-averaged physical channel SNR stats (entry-pick physics cap).
    void setPhysicalChannelStats(float mean_db, float spread_db, uint32_t n);
    float getMeasuredSNR() const;
    SNRSource getMeasuredSNRSource() const;

    // Set channel quality including fading detection
    void setChannelQuality(float snr_db, float fading_index,
                           SNRSource source = SNRSource::NONE,
                           bool data_aided = false);
    // Set the OFDM Doppler-coherence verdict (Good/Moderate discriminator), measured
    // separately from fading_index. See docs/CHANNEL_DISCRIMINATOR_DESIGN_2026_06_15.md.
    // doppler_hz: the estimator's SECONDARY/approximate RMS-Doppler readout (0 = not
    // estimable), riding the same feed — consumed only by the retx trough-pacing deferral
    // (docs/RETX_PACING_DESIGN_2026_07_03.md), never a decode decision.
    void setChannelCoherence(float coherence_score, float doppler_hz, bool valid);
    // Software-ALC (BUG-QAM16-RIG-LEVEL-BUDGET): feed the decoder's per-burst RX level
    // verdict (connection_policy::RxLevelVerdict as int) + measurement seq. Call BEFORE
    // onBurstGroupReceived so the drive advisory rides that group's tone-burst ACK.
    void setRxLevelVerdict(int verdict, uint32_t seq);
    // RX-AUTHORITY (2026-07-05): fresh per-group receiver channel measurements
    // (broadband SNR EMA, fading index, coherence disc) from the decoder's lock-free
    // atomics. Call BEFORE onBurstGroupReceived so the rung command rides that
    // group's ACK.
    void setBurstChannelObservation(float snr_db, float fading_index,
                                    float coherence_score, bool coherence_valid,
                                    float doppler_hz);
    // RX-AUTHORITY EVM DEMOTE (ULTRA_EVM_DEMOTE): forward this group's radio-agnostic
    // decision-directed EVM usable-SNR (dB) to the Connection. Call BEFORE
    // onBurstGroupReceived so the demote clamp rides that group's ACK.
    void setBurstEvmObservation(float evm_snr_db);
    // Explicit invalid observation for a group without complete physical-frame
    // EVM coverage. Prevents reuse of the preceding group's scalar.
    void clearBurstEvmObservation();
    bool shouldUseRxFrameForChannelQuality(const Bytes& data) const;
    float getFadingIndex() const;

    // Get current data mode
    Modulation getDataModulation() const;
    CodeRate getDataCodeRate() const;

    // Set callback for data mode changes
    using DataModeChangedCallback = Connection::DataModeChangedCallback;
    void setDataModeChangedCallback(DataModeChangedCallback cb);

    // --- Ping/Pong (fast presence check) ---

    // Callback when protocol wants to send a ping (modem transmits raw "ULTR")
    // Burst mode TX callback - transmits multiple frames as single audio burst (OFDM only)
    using TransmitBurstCallback = Connection::TransmitBurstCallback;
    void setTransmitBurstCallback(TransmitBurstCallback cb);

    // §15 step 4d-iii: parallel emit path for the tone-burst ACK. Fires
    // ALONGSIDE the OFDM GROUP_ACK on every group ACK; the receiver's
    // monitor decodes whichever arrives first. The OFDM ACK stays in
    // place as the existing baseline path until the tone-burst route
    // is multi-seed verified.
    using TransmitToneBurstAckCallback = Connection::TransmitToneBurstAckCallback;
    using LegacyTransmitToneBurstAckCallback =
        Connection::LegacyTransmitToneBurstAckCallback;
    void setTransmitToneBurstAckCallback(TransmitToneBurstAckCallback cb) {
        connection_.setTransmitToneBurstAckCallback(std::move(cb));
    }
    void setTransmitToneBurstAckCallback(LegacyTransmitToneBurstAckCallback cb) {
        connection_.setTransmitToneBurstAckCallback(std::move(cb));
    }

    using ArmToneBurstAckMonitorCallback = Connection::ArmToneBurstAckMonitorCallback;
    void setArmToneBurstAckMonitorCallback(ArmToneBurstAckMonitorCallback cb) {
        connection_.setArmToneBurstAckMonitorCallback(std::move(cb));
    }

    // Software-ALC sender-side hook: a decoded ACK carried a non-hold drive
    // advisory. Fires under the engine mutex — host must not re-enter the protocol
    // from it (atomics + logging only).
    using DriveAdvisoryCallback = Connection::DriveAdvisoryCallback;
    void setDriveAdvisoryCallback(DriveAdvisoryCallback cb) {
        connection_.setDriveAdvisoryCallback(std::move(cb));
    }

    // BUG-MC-RETRY-SPURIOUS (2026-07-04): host TX-keyed predicate — see
    // Connection::setTxActiveProvider. Set once at wiring; called from the engine
    // tick under the mutex, so it must be trivially thread-safe (read an atomic).
    void setTxActiveProvider(std::function<bool()> provider) {
        connection_.setTxActiveProvider(std::move(provider));
    }

    void setChannelBusyQuery(std::function<bool()> query) {
        connection_.setChannelBusyQuery(std::move(query));
    }

    // Half-duplex INTERACTIVE (bidirectional) data path — the TNC / Winlink-B2F
    // case where both stations alternately transmit. Keeps the ISS/IRS turn gate
    // on burst sends so the directions serialize instead of colliding.
    void setHalfDuplexInteractive(bool v) { connection_.setHalfDuplexInteractive(v); }

    // Invoked when this station acquires the half-duplex DATA turn — the TNC wires
    // it to ModemEngine::forceNextFrameFullPreamble() so the first post-turn-flip
    // transmission re-anchors the new receiver (BUG-TNC-B2F-001).
    using DataTurnAcquiredCallback = Connection::DataTurnAcquiredCallback;
    void setDataTurnAcquiredCallback(DataTurnAcquiredCallback cb) {
        connection_.setDataTurnAcquiredCallback(std::move(cb));
    }

    // Arm the RX decoder to cold-acquire a full chirp+LTS anchor — the TNC wires it
    // to ModemEngine::expectFullOFDMAnchorOnce(). Re-armed each tick while yielded and
    // waiting for the peer's first burst, so the expectation survives the turn-flip gap
    // (BUG-TNC-B2F-001).
    using FullOFDMAnchorExpectedCallback = Connection::FullOFDMAnchorExpectedCallback;
    void setFullOFDMAnchorExpectedCallback(FullOFDMAnchorExpectedCallback cb) {
        connection_.setFullOFDMAnchorExpectedCallback(std::move(cb));
    }

    using PingTxCallback = Connection::PingTxCallback;
    void setPingTxCallback(PingTxCallback cb);

    // Callback when we receive an incoming ping while disconnected (someone's calling)
    using PingReceivedCallback = Connection::PingReceivedCallback;
    void setPingReceivedCallback(PingReceivedCallback cb);

    // Call when modem detects "ULTR" magic (either PONG to our PING, or incoming PING)
    void onPingReceived();

private:
    Connection connection_;

    TxDataCallback on_tx_data_;
    MessageReceivedCallback on_message_received_;
    MessageTxStatusCallback on_message_tx_status_;
    ConnectionChangedCallback on_connection_changed_;
    DisconnectTeardownCallback on_disconnect_teardown_;
    IncomingCallCallback on_incoming_call_;
    DataReceivedCallback on_data_received_;

    mutable ProtocolEngineMutex mutex_;

    Bytes rx_buffer_;

    struct PendingTxFrame {
        Bytes data;
        bool expect_full_ofdm_anchor_after_tx = false;
    };
    std::vector<PendingTxFrame> tx_queue_;
    bool defer_tx_ = false;

    struct PendingMessageReceived {
        std::string from;
        std::string text;
    };
    struct PendingDataReceived {
        Bytes data;
        bool more_data = false;
    };
    struct PendingCallbackEvent {
        enum class Type {
            MESSAGE_RECEIVED,
            MESSAGE_TX_STATUS,
            DATA_RECEIVED,
        };

        Type type = Type::MESSAGE_RECEIVED;
        PendingMessageReceived message;
        MessageTxStatusEvent tx_status;
        PendingDataReceived data;
    };
    struct PendingCallbackBatch {
        MessageReceivedCallback message_received;
        MessageTxStatusCallback message_tx_status;
        DataReceivedCallback data_received;
        std::vector<PendingCallbackEvent> events;
    };
    std::vector<PendingCallbackEvent> pending_callback_events_;
    bool pending_callback_dispatch_active_ = false;

    void handleTxFrame(const Bytes& frame_data, bool expect_full_ofdm_anchor_after_tx);
    void processRxBuffer(bool input_physical_turn_complete = false);
    PendingCallbackBatch takePendingCallbacksLocked();
    static void emitPendingCallbacks(PendingCallbackBatch callbacks);
    void dispatchPendingCallbacks();
};

} // namespace protocol
} // namespace ultra
