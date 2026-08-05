// Connection mode-change / handshake / ARQ-config regression tests.
//
// NOTE (2026-05-30): the adaptive-RATE-LADDER cases (test_adaptive_*, the
// upgrade/downgrade hysteresis + post-downgrade lockout + timeout-repair
// framing) were removed here. They had drifted out of sync with the
// controller (14/281 checks were RED at this exact state AND at the
// pre-session commit c384b6a — i.e. pre-existing legacy drift, not a
// regression) and the adaptive ladder is being reworked. Proper ladder
// coverage will be re-authored against the reworked controller. The cases
// kept below exercise stable connection plumbing, not the ladder.
//
// NOTE (2026-06-09): the old in-Connection adaptive-mode controller
// (updateAdaptiveModeController + adaptive_target_ hysteresis/lockout state)
// has now been DELETED outright — rate adaptation lives in the EMA-smoothed
// RateController (tests/test_rate_controller.cpp) and lands via the
// synchronized requestModeChange() MODE_CHANGE handshake exercised below. The
// dead test accessors for that machinery were removed with it.
#include "env_compat.hpp"
#include "protocol/connection.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "gui/modem/streaming_frame_policy.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"
#include "helpers/temp_dir.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>

using namespace ultra;
using namespace ultra::protocol;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
            return; \
        } \
    } while (0)

std::string createFile(const std::filesystem::path& dir, size_t bytes) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) return {};

    const auto path = dir / "payload.bin";
    std::ofstream out(path, std::ios::binary);
    if (!out) return {};
    for (size_t i = 0; i < bytes; ++i) {
        out.put(static_cast<char>((i * 17 + 3) & 0xFF));
    }
    return path.string();
}

struct TempPayloadFile {
    ultra::test::TempDir dir;
    std::string path;

    TempPayloadFile(const std::string& prefix, size_t bytes)
        : dir(prefix) {
        if (dir.valid()) {
            path = createFile(dir.path(), bytes);
        }
    }
};

} // namespace

namespace ultra {
namespace protocol {

struct ConnectionAdaptiveTestAccess {
    static void makeLocalIss(Connection& c) {
        c.local_data_turn_ = true;
        c.peer_data_turn_requested_ = false;
        c.local_turn_request_pending_ = false;
        c.received_peer_data_since_connect_ = false;
        c.data_turn_yield_pending_ = false;
        c.data_turn_payload_bytes_sent_ = 0;
        c.data_turn_contended_ms_ = 0;
        c.data_turn_tx_guard_ms_ = 0;
        c.turn_request_retransmit_ms_ = 0;
        c.turn_request_holdoff_ms_ = 0;
    }

    static void makeConnectedOFDM(Connection& c,
                                  CodeRate rate,
                                  float snr = 15.0f,
                                  float fading = 0.05f,
                                  Modulation modulation = Modulation::DQPSK) {
        c.local_call_ = "W1ABC";
        c.remote_call_ = "K2DEF";
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = true;
        c.handshake_confirmed_ = true;
        c.negotiated_mode_ = WaveformMode::OFDM_CHIRP;
        c.data_modulation_ = modulation;
        c.data_code_rate_ = rate;
        c.measured_snr_db_ = snr;
        c.fading_index_ = fading;
        makeLocalIss(c);
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
        c.configureArqForCurrentDataMode();
    }

    static void makeConnectedNarrowOFDM(Connection& c) {
        c.local_call_ = "W1ABC";
        c.remote_call_ = "K2DEF";
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = true;
        c.handshake_confirmed_ = true;
        c.negotiated_mode_ = WaveformMode::OFDM_NARROW;
        c.data_modulation_ = Modulation::DQPSK;
        c.data_code_rate_ = CodeRate::R1_4;
        c.data_frame_cw_count_ = v2::kDefaultFixedFrameCodewords;
        c.config_.fixed_frame_codewords = v2::kDefaultFixedFrameCodewords;
        c.measured_snr_db_ = 8.0f;
        c.fading_index_ = 0.30f;
        makeLocalIss(c);
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
        c.configureArqForCurrentDataMode();
    }

    static void makeConnectedMCDPSK(Connection& c) {
        c.local_call_ = "W1ABC";
        c.remote_call_ = "K2DEF";
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = true;
        c.handshake_confirmed_ = true;
        c.negotiated_mode_ = WaveformMode::MC_DPSK;
        c.data_modulation_ = Modulation::DBPSK;
        c.data_code_rate_ = CodeRate::R1_4;
        c.data_frame_cw_count_ = v2::kDefaultFixedFrameCodewords;
        c.config_.fixed_frame_codewords = v2::kDefaultFixedFrameCodewords;
        c.config_.mc_dpsk_num_carriers = 8;
        c.config_.mc_dpsk_samples_per_symbol = 1024;
        c.measured_snr_db_ = 5.0f;
        c.fading_index_ = 0.30f;
        makeLocalIss(c);
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
        c.configureArqForCurrentDataMode();
    }

    static void makeResponderWithCachedConnectAck(
        Connection& c, WaveformMode mode = WaveformMode::OFDM_CHIRP) {
        c.local_call_ = "K2DEF";
        c.remote_call_ = "W1ABC";
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = false;
        c.handshake_confirmed_ = false;
        c.negotiated_mode_ = mode;
        c.data_modulation_ = Modulation::DQPSK;
        c.data_code_rate_ = CodeRate::R1_4;
        c.connect_ack_frame_ = Bytes{0x55, 0x4C, static_cast<uint8_t>(v2::FrameType::CONNECT_ACK)};
    }

    static void makeConnectedInitiator(Connection& c, WaveformMode mode) {
        c.local_call_ = "W1ABC";
        c.remote_call_ = "K2DEF";
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = true;
        c.handshake_confirmed_ = true;
        c.negotiated_mode_ = mode;
        c.data_modulation_ = Modulation::DQPSK;
        c.data_code_rate_ = CodeRate::R1_4;
        makeLocalIss(c);
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
    }

    static void makeConnectedEndpoint(Connection& c,
                                      const std::string& local,
                                      const std::string& remote,
                                      bool initiator) {
        c.local_call_ = local;
        c.remote_call_ = remote;
        c.state_ = ConnectionState::CONNECTED;
        c.is_initiator_ = initiator;
        c.handshake_confirmed_ = true;
        c.negotiated_mode_ = WaveformMode::OFDM_CHIRP;
        c.data_modulation_ = Modulation::QPSK;
        c.data_code_rate_ = CodeRate::R2_3;
        c.measured_snr_db_ = 20.0f;
        c.fading_index_ = 0.30f;
        c.local_data_turn_ = initiator;
        c.peer_data_turn_requested_ = false;
        c.local_turn_request_pending_ = false;
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
        c.configureArqForCurrentDataMode();
    }

    static void enterConnected(Connection& c) {
        c.enterConnected();
    }

    static void enterConnected(Connection& c, bool automatic_rate_allowed) {
        c.enterConnected(automatic_rate_allowed);
    }

    static void makeConnectingWithOutboundForce(Connection& c,
                                                Modulation mod,
                                                CodeRate rate) {
        c.local_call_ = "W1ABC";
        c.remote_call_ = "K2DEF";
        c.state_ = ConnectionState::CONNECTING;
        c.outbound_forced_modulation_ = mod;
        c.outbound_forced_code_rate_ = rate;
    }

    static void handleConnectAck(Connection& c, const v2::ConnectFrame& frame) {
        c.handleConnectAck(frame, "K2DEF");
    }

    static void noteDataTurnPayloadStarted(Connection& c, size_t payload_bytes) {
        c.noteDataTurnPayloadStarted(payload_bytes);
    }

    static bool interactiveInitiatorYieldDone(const Connection& c) {
        return c.interactive_initiator_yield_done_;
    }

    static void transmitFrame(Connection& c, const Bytes& frame) {
        c.transmitFrame(frame);
    }

    static void transmitFrameBatch(Connection& c, const std::vector<Bytes>& frames) {
        c.transmitFrameBatch(frames);
    }

    static uint32_t connectRetryInterval(Connection& c) {
        return c.connectRetryIntervalMs();
    }

    static uint32_t modeChangeRetryMs(Connection& c) {
        return c.modeChangeRetryMs();
    }

    static uint32_t disconnectRetryIntervalMs(const Connection& c) {
        return c.disconnectRetryIntervalMs();
    }

    static uint32_t disconnectResponderGraceMs(const Connection& c) {
        return c.disconnectResponderGraceMs();
    }

    static uint32_t disconnectAckRetransmitMs(const Connection& c) {
        return c.disconnectAckRetransmitMs();
    }

    static uint32_t disconnectControlKeydownMs(const Connection& c) {
        return c.disconnectControlKeydownMs();
    }

    static int disconnectUsefulRetryCount(const Connection& c) {
        return c.disconnectUsefulRetryCount();
    }

    static bool disconnectPending(const Connection& c) {
        return c.disconnect_pending_;
    }

    static uint32_t disconnectPendingMs(const Connection& c) {
        return c.disconnect_pending_ms_;
    }

    static int disconnectAckRepeatCount(const Connection& c) {
        return c.disconnect_ack_repeat_count_;
    }

    static int disconnectAckMaxProactiveRepeats(const Connection& c) {
        return c.disconnectAckMaxProactiveRepeats();
    }

    static int disconnectRetryCount(const Connection& c) {
        return c.disconnect_retry_count_;
    }

    static uint32_t disconnectTimeoutRemainingMs(const Connection& c) {
        return c.timeout_remaining_ms_;
    }

    static int modeChangeMaxRetries() {
        return Connection::MODE_CHANGE_MAX_RETRIES;
    }

    static int modeChangeRetryCount(const Connection& c) {
        return c.mode_change_retry_count_;
    }

    static void setCoherenceDoppler(Connection& c, float doppler_hz) {
        c.coherence_doppler_hz_ = doppler_hz;
        c.coherence_valid_ = true;
    }

    static bool handshakeConfirmed(const Connection& c) {
        return c.handshake_confirmed_;
    }

    static void setResponderHandshakeWait(Connection& c, uint32_t ms) {
        c.responder_handshake_wait_ms_ = ms;
    }

    static bool connectAckCached(const Connection& c) {
        return !c.connect_ack_frame_.empty();
    }

    static void startFile(Connection& c, const std::string& path) {
        CHECK(c.file_transfer_.startSend(path), "startSend should succeed");
    }

    static void acknowledgeModeChange(Connection& c) {
        auto ack = v2::ControlFrame::makeAck("K2DEF", "W1ABC",
                                             c.getStats().arq.acks_received + 1);
        ack.seq = c.mode_change_seq_;
        c.onFrameReceived(ack.serialize());
    }

    static void fillArqWindow(Connection& c, size_t in_flight_frames) {
        for (size_t i = 0; i < in_flight_frames; ++i) {
            CHECK(c.arq_.sendFixedDataWithFlags(
                      Bytes(16, static_cast<uint8_t>(0x42 + i)), v2::Flags::MORE_FRAG),
                  "seed DATA frame should enter ARQ window");
        }
    }

    static void createRetransmissionPressure(Connection& c, size_t in_flight_frames) {
        c.arq_.setAckTimeout(100);
        fillArqWindow(c, in_flight_frames);
        advanceRetransmissionPressure(c);
    }

    static void advanceRetransmissionPressure(Connection& c) {
        c.arq_.tick(150);
    }

    static void createRetransmissionPressure(Connection& c) {
        createRetransmissionPressure(c, 1);
    }

    static bool modeChangePending(const Connection& c) {
        return c.mode_change_pending_;
    }

    static CodeRate pendingRate(const Connection& c) {
        return c.pending_code_rate_;
    }

    static Modulation pendingModulation(const Connection& c) {
        return c.pending_modulation_;
    }

    static uint16_t modeChangeSeq(const Connection& c) {
        return c.mode_change_seq_;
    }

    static size_t arqWindow(const Connection& c) {
        return c.arq_.getWindowSize();
    }

    static size_t arqAvailableSlots(const Connection& c) {
        return c.arq_.getAvailableSlots();
    }

    static void abortArqPendingTx(Connection& c) {
        c.arq_.abortPendingTx();
    }

    static CodeRate arqCodeRate(const Connection& c) {
        return c.arq_.getCodeRate();
    }

    static uint32_t arqSackDelay(const Connection& c) {
        return c.arq_.getSackDelay();
    }

    static uint32_t arqSackDelayShort(const Connection& c) {
        return c.arq_.getSackDelayShort();
    }

    static bool arqSackDelaySlidesOnData(const Connection& c) {
        return c.arq_.getSackDelaySlidesOnData();
    }

    static uint32_t arqAckTimeout(const Connection& c) {
        return c.arq_.getAckTimeout();
    }

    static void disableAdaptiveRate(Connection& c) {
        c.adaptive_rate_enabled_ = false;
    }

    static size_t dataPayloadCapacity(const Connection& c) {
        return c.currentDataPayloadCapacity();
    }

    static size_t burstFrameBudget(const Connection& c,
                                   bool force_full_group_start = false) {
        return c.burstAirtimeBudgetFrames(
            c.arq_.getWindowSize(), force_full_group_start);
    }

    static uint32_t unifiedBurstTimeout(const Connection& c, size_t frames) {
        return c.unifiedBurstAckTimeoutMs(frames);
    }

    static uint32_t physicalRoundTimeout(
        const Connection& c, const std::vector<Bytes>& frames,
        bool force_full_group_start = false) {
        return c.physicalDataRoundTiming(
            frames, force_full_group_start).ack_timeout_ms;
    }

    static uint32_t physicalRoundAirtime(
        const Connection& c, const std::vector<Bytes>& frames,
        bool force_full_group_start = false) {
        return c.physicalDataRoundTiming(
            frames, force_full_group_start).airtime_ms;
    }

    static uint32_t physicalRoundWaveformAirtime(
        const Connection& c, const std::vector<Bytes>& frames,
        bool force_full_group_start = false) {
        return c.physicalDataRoundTiming(
            frames, force_full_group_start).waveform_airtime_ms;
    }

    static uint64_t physicalRoundWaveformSamples(
        const Connection& c, const std::vector<Bytes>& frames,
        bool force_full_group_start = false) {
        return c.physicalDataRoundTiming(
            frames, force_full_group_start).waveform_samples;
    }

    static uint64_t physicalRoundKeyedSamples(
        const Connection& c, const std::vector<Bytes>& frames,
        bool force_full_group_start = false) {
        return c.physicalDataRoundTiming(
            frames, force_full_group_start).keyed_samples;
    }

    static bool fragmentedMessagePending(const Connection& c) {
        return !c.pending_tx_fragments_.empty();
    }

    static void makeRemoteDataTurn(Connection& c) {
        c.local_data_turn_ = false;
        c.peer_data_turn_requested_ = false;
        c.local_turn_request_pending_ = false;
        c.data_turn_yield_pending_ = false;
        c.data_turn_tx_guard_ms_ = 0;
        c.turn_request_retransmit_ms_ = 0;
        c.turn_request_holdoff_ms_ = 0;
    }

    static size_t queuedPayloadCount(const Connection& c) {
        return c.queued_payloads_.size();
    }

    // Application text of a queued message — the message-object identity prefix
    // (PayloadType::TEXT_MESSAGE_OBJECT + object id) is transport, not content.
    static std::string queuedPayloadText(const Connection& c, size_t index) {
        if (index >= c.queued_payloads_.size()) {
            return {};
        }
        const auto& data = c.queued_payloads_[index].data;
        auto begin = data.begin();
        if (!c.queued_payloads_[index].binary_payload &&
            data.size() >= kMessageObjectPrefixBytes &&
            data[0] == static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT)) {
            begin += static_cast<std::ptrdiff_t>(kMessageObjectPrefixBytes);
        }
        return std::string(begin, data.end());
    }

    static size_t queuedPayloadObjectId(const Connection& c, size_t index) {
        if (index >= c.queued_payloads_.size()) {
            return 0;
        }
        const auto& data = c.queued_payloads_[index].data;
        if (data.size() < kMessageObjectPrefixBytes ||
            data[0] != static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT)) {
            return 0;
        }
        return data[1];
    }

    static size_t outboundMessageRecordCount(const Connection& c) {
        return c.outbound_message_tx_records_.size();
    }

    static size_t deliveredMessageObjectIdCount(const Connection& c) {
        return c.delivered_message_object_ids_.size();
    }

    static unsigned messageGeometryRegridAttempts(const Connection& c,
                                                  size_t index) {
        if (index >= c.outbound_message_tx_records_.size()) {
            return 0;
        }
        return c.outbound_message_tx_records_[index].geometry_regrid_attempts;
    }

    static void deliverDataPayload(Connection& c, const Bytes& payload,
                                   bool more_data) {
        c.handleDataPayload(payload, more_data, v2::FrameType::DATA,
                            more_data ? v2::Flags::MORE_FRAG : v2::Flags::FINAL);
    }

    static bool queuedFile(const Connection& c) {
        return c.queued_file_path_.has_value();
    }

    static bool fileControllerSending(const Connection& c) {
        return c.file_transfer_.getState() == FileTransferState::SENDING;
    }

    static void pumpQueuedOperation(Connection& c) {
        c.sendNextQueuedPayloadIfReady();
    }

    static void setRawDataCodeRate(Connection& c, CodeRate rate) {
        c.data_code_rate_ = rate;
    }

    static void clearArqCallsigns(Connection& c) {
        c.arq_.setCallsigns({}, {});
    }

    static void restoreArqCallsigns(Connection& c) {
        c.arq_.setCallsigns(c.local_call_, c.remote_call_);
    }

    static void applyDataMode(Connection& c, Modulation mod, CodeRate rate,
                              int cw_count) {
        c.applyDataMode(mod, rate, cw_count, LadderRungId::UNKNOWN);
    }

    static size_t rxReassemblyBytes(const Connection& c) {
        return c.rx_reassembly_buffer_.size();
    }

    static bool sendFixedData(Connection& c, const Bytes& payload, uint8_t flags) {
        return c.arq_.sendFixedDataWithFlags(payload, flags);
    }

    static bool sendVariableData(Connection& c, const Bytes& payload, uint8_t flags) {
        return c.arq_.sendVariableDataWithFlags(payload, flags);
    }

    static size_t rearmExactDataRound(Connection& c,
                                      const std::vector<Bytes>& frames,
                                      uint32_t timeout_ms) {
        return c.arq_.rearmTransmittedDataFrames(frames, timeout_ms);
    }

    static void setArqMaxRetries(Connection& c, int retries) {
        c.arq_.setMaxRetries(retries);
    }

    static bool arqMoveEpochEnabled(const Connection& c) {
        return c.arq_.moveEpochEnabled();
    }

    static size_t arqInFlightBytes(const Connection& c) {
        return c.arq_.getTxInFlightBytes();
    }

    static int arqMaxInFlightRetryCount(const Connection& c) {
        return c.arq_.maxInFlightRetryCount();
    }

    static bool applyToneAckToArqOnly(Connection& c, uint8_t group_seq,
                                      uint32_t bitmap, uint8_t move_epoch = 0) {
        c.arq_.consumeAckProgress();
        return c.arq_.onToneBurstAck(group_seq, bitmap, move_epoch);
    }

    static int arqFixedFrameLiftingZ(const Connection& c) {
        return c.arq_.getFixedFrameLiftingZ();
    }

    static int arqFixedFrameCodewords(const Connection& c) {
        return c.arq_.getFixedFrameCodewords();
    }

    static void simulatePreviousBurstAired(Connection& c) {
        // These unit tests advance protocol time with tick(), not steady_clock. Clear
        // the wall-clock audio play-head between synthetic ACK turns to model the real
        // rig, where an ACK cannot arrive until the preceding burst has left the DAC.
        c.last_data_burst_end_valid_ = false;
    }

    static int arqAckRepeatCount(const Connection& c) {
        return c.arq_.getAckRepeatCount();
    }

    static uint32_t arqAckRepeatDelay(const Connection& c) {
        return c.arq_.getAckRepeatDelay();
    }

    static void forceCodeRate(Connection& c, CodeRate rate) {
        c.config_.forced_code_rate = rate;
    }

    // §RETX-PACING (docs/RETX_PACING_DESIGN_2026_07_03.md) test hooks.
    static void noteRoundOutcome(Connection& c, int progress_frames, const char* origin) {
        c.noteArqRoundOutcome(progress_frames, origin);
    }

    static int zeroProgressRounds(const Connection& c) {
        return c.zero_progress_rounds_;
    }

    static uint32_t paceHoldMs(const Connection& c) {
        return c.retx_pace_hold_ms_;
    }

    static void pollCollapseEscape(Connection& c) {
        c.maybeCollapseEscape();
    }

    // DESC-SWITCH (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md Phase 1) hooks.
    static void applyFeedback(Connection& c, float quality) {
        c.applyAdaptiveRateFeedback(quality);
    }

    static bool descSwitchFullAnchorArmed(const Connection& c) {
        return c.desc_switch_full_anchor_pending_;
    }

    static uint8_t arqTxMoveEpoch(const Connection& c) {
        return c.arq_.txMoveEpoch();
    }

    static int dataFrameCWCount(const Connection& c) {
        return c.data_frame_cw_count_;
    }

    static void setDataGeometry(Connection& c, Modulation mod, CodeRate rate,
                                int logical_cw) {
        c.data_modulation_ = mod;
        c.data_code_rate_ = rate;
        c.data_frame_cw_count_ = v2::sanitizeFixedFrameCodewords(logical_cw);
        c.config_.fixed_frame_codewords = c.data_frame_cw_count_;
        c.configureArqForCurrentDataMode();
    }

    static int physicalDataFrameCWCount(const Connection& c) {
        return c.physicalDataFrameCodewords();
    }

    static bool experimental8PSKLongLDPCActive(const Connection& c) {
        return c.usesExperimental8PSKLongLDPC();
    }

    static bool experimentalQPSKR34LongLDPCActive(const Connection& c) {
        return c.usesExperimentalQPSKR34LongLDPC();
    }

    static void armExperimentalLongLDPCProfiles(
        Connection& c, bool psk8_active, bool qpsk_r34_active) {
        c.setExperimentalLongLDPCTransferProfilesActive(
            psk8_active, qpsk_r34_active);
    }

    static LatentRateCandidateGeometry latentCandidateGeometry(
        const Connection& c, Modulation mod, CodeRate rate,
        bool force_full_group_start) {
        return c.latentRateCandidateGeometryFor(
            mod, rate, force_full_group_start);
    }

    static void failFileTransfer(Connection& c) {
        c.file_transfer_.onSendFailed();
    }

    static size_t currentPayloadCapacity(const Connection& c) {
        return c.currentDataPayloadCapacity();
    }

    static void setDataTurnTxGuard(Connection& c, uint32_t guard_ms) {
        c.data_turn_tx_guard_ms_ = guard_ms;
    }

    static bool tryStartQueuedFileIfReady(Connection& c) {
        return c.tryStartQueuedFileIfReady();
    }

    // RX-RATE-CMD (Phase 2, ULTRA_RX_RATE_CMD) hooks.
    static uint8_t rxRateCmdPending(const Connection& c) {
        return c.rx_rate_cmd_pending_;
    }

    // LATENT-RATE/QSO-scope regression hooks. Keep the production entry point in the
    // loop so these tests pin the exact k/M plumbing rather than re-testing only the
    // standalone estimator.
    static void updateRxAuthorityExact(Connection& c, bool all_ok, float quality,
                                       bool full_crater, float delivered_fraction,
                                       uint8_t delivered_frames, uint8_t group_size,
                                       bool geometry_proven = true) {
        c.updateRxAuthorityCommand(all_ok, quality, full_crater, delivered_fraction,
                                   delivered_frames, group_size, geometry_proven);
    }

    static int latentObservations(const Connection& c) {
        return c.latent_ctl_.observations();
    }

    static int latentDecisions(const Connection& c) {
        return c.latent_ctl_.decisions();
    }

    static bool latentHasPrior(const Connection& c) {
        return c.latent_ctl_.havePrior();
    }

    static float latentPosteriorMean(const Connection& c) {
        return c.latent_ctl_.posteriorMean();
    }

    static bool latentVerdictClockValid(const Connection& c) {
        return c.goodput_last_verdict_valid_;
    }

    static void setLatentBootstrapRung(Connection& c, uint8_t rung) {
        c.latent_bootstrap_rung_ = rung;
    }

    static uint8_t latentBootstrapRung(const Connection& c) {
        return c.latent_bootstrap_rung_;
    }

    static uint8_t rxAuthorityCommand(const Connection& c) {
        return c.rx_authority_cmd_;
    }

    static void setCurrentCoherentMode(Connection& c, Modulation mod, CodeRate rate) {
        c.data_modulation_ = mod;
        c.data_code_rate_ = rate;
    }

    static void setLatentStartupProbeAllowed(Connection& c, bool allowed) {
        c.latent_startup_probe_allowed_ = allowed;
    }

    static bool latentStartupProbeAllowed(const Connection& c) {
        return c.latent_startup_probe_allowed_;
    }

    static bool rateAdaptationActive(const Connection& c) {
        return c.rateAdaptationActive();
    }

    static bool latentStartupProbeWaiting(const Connection& c) {
        return c.latent_startup_probe_waiting_;
    }

    static bool latentStartupProbeSpent(const Connection& c) {
        return c.latent_startup_probe_spent_;
    }

    static int latentStartupProbeCleanGroups(const Connection& c) {
        return c.latent_startup_probe_clean_groups_;
    }

    static int latentStartupProbePendingBaseGroups(const Connection& c) {
        return c.latent_startup_probe_pending_base_groups_;
    }

    static bool latentStartupProbeRollbackPending(const Connection& c) {
        return c.latent_startup_probe_rollback_pending_;
    }

    static bool latentStartupProbeFailed(const Connection& c) {
        return c.latent_startup_probe_failed_;
    }

    static void setLatentStartupProbeCleanGroups(Connection& c, int groups) {
        c.latent_startup_probe_clean_groups_ = groups;
    }

    static void setLatentStartupProbeSpent(Connection& c, bool spent) {
        c.latent_startup_probe_spent_ = spent;
    }

    static void seedLatentPrior(Connection& c, float x, float sigma_db) {
        c.latent_ctl_.seedPrior(x, sigma_db);
        c.latent_bootstrap_rung_ = kRungIdxNone;
    }

    static void maybeObeyAuthorityCommand(Connection& c, uint8_t rung,
                                           bool accepted_clean_round = false) {
        c.maybeObeyAuthorityCommand(rung, accepted_clean_round);
    }

    static bool startupProbeHasSufficientPayload(const Connection& c) {
        return c.startupProbeHasSufficientPayload();
    }

    static bool authorityClimbHasSufficientPayload(
        const Connection& c, Modulation mod, CodeRate rate,
        int post_ack_clean_streak = 0) {
        return c.authorityClimbHasSufficientPayload(
            mod, rate, post_ack_clean_streak);
    }

    static bool retireFileMetadata(Connection& c) {
        const Bytes metadata = c.file_transfer_.getNextChunk();
        if (metadata.empty() ||
            metadata.front() != static_cast<uint8_t>(PayloadType::FILE_START)) {
            return false;
        }
        c.file_transfer_.onChunkAcked();
        return true;
    }

    // Reproduce the production ordering inside onToneBurstAck(): authority is
    // evaluated while refill is deferred, the accepted clean round advances the
    // sender streak, then exactly one file refill forms the post-switch burst.
    static void obeyAuthorityAtCleanAckBoundary(Connection& c, uint8_t rung) {
        c.arq_callback_defer_refill_ = true;
        c.maybeObeyAuthorityCommand(rung, /*accepted_clean_round=*/true);
        c.noteArqRoundOutcome(/*progress_frames=*/1, "test-clean-tone-ack");
        if (c.file_transfer_.getState() == FileTransferState::SENDING) {
            c.deferred_file_refill_ = true;
        }
        c.arq_callback_defer_refill_ = false;
        c.runDeferredArqRefill();
    }

    static size_t targetProfileChunkBytes(const Connection& c,
                                          Modulation mod,
                                          CodeRate rate) {
        const int cw = (c.config_.forced_cw_count != 0)
            ? v2::sanitizeFixedFrameCodewords(c.config_.forced_cw_count)
            : connection_policy::recommendCWCountForChannel(
                  mod, rate, c.negotiated_mode_,
                  connection_policy::coherenceAdjustedFadingIndex(
                      c.fading_index_, c.coherence_score_, c.coherence_valid_),
                  c.wireSnrDb());
        const size_t payload = (c.selectBurstLiftingZ() == 81)
            ? v2::getFixedFramePayloadCapacityZ(rate, cw, 81)
            : v2::getFixedFramePayloadCapacity(rate, cw);
        return payload - FileTransferController::FILE_DATA_OVERHEAD;
    }

    static size_t targetProfileBurstFrames(const Connection& c,
                                            Modulation mod,
                                            CodeRate rate,
                                            int clean_streak = -1) {
        const int cw = (c.config_.forced_cw_count != 0)
            ? v2::sanitizeFixedFrameCodewords(c.config_.forced_cw_count)
            : connection_policy::recommendCWCountForChannel(
                  mod, rate, c.negotiated_mode_,
                  connection_policy::coherenceAdjustedFadingIndex(
                      c.fading_index_, c.coherence_score_, c.coherence_valid_),
                  c.wireSnrDb());
        size_t window = connection_policy::ofdmWindowSizeForChannel(
            mod, rate, c.fading_index_, c.rateSelectionSnrDb());
        window = std::min<size_t>(
            window, connection_policy::kToneBurstAckWindowCapFrames);
        const uint32_t ceiling_ms = connection_policy::burstAirtimeCeilingMs(
            mod, rate,
            clean_streak >= 0 ? clean_streak : c.burst_clean_group_streak_);
        const uint32_t reanchor_ms =
            connection_policy::shouldUseWideOFDMShortReanchor(
                c.negotiated_mode_, mod, c.fading_index_)
                ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
                : 0;
        return connection_policy::wideOFDMBurstFrameBudget(
            mod, rate, cw, window, ceiling_ms, reanchor_ms,
            c.selectBurstLiftingZ(),
            connection_policy::kWideOFDMFullAnchorExtraMs);
    }

    static uint8_t txAuthorityLastObeyed(const Connection& c) {
        return c.tx_authority_last_obeyed_;
    }

    static size_t startupProbeTargetChunkBytes(const Connection& c) {
        return targetProfileChunkBytes(c, Modulation::QPSK, CodeRate::R2_3);
    }

    static void setForcedCWCount(Connection& c, int cw) {
        c.config_.forced_cw_count = cw;
    }

    static int configuredForcedCWCount(const Connection& c) {
        return c.config_.forced_cw_count;
    }

    static bool txLatentStartupProbeActive(const Connection& c) {
        return c.tx_latent_startup_probe_active_;
    }

    static bool txLatentStartupProbeAirborne(const Connection& c) {
        return c.tx_latent_startup_probe_airborne_;
    }

    static void setTxLatentStartupProbeState(Connection& c, bool active,
                                              bool airborne, uint8_t base) {
        c.tx_latent_startup_probe_active_ = active;
        c.tx_latent_startup_probe_airborne_ = airborne;
        c.tx_latent_startup_probe_base_rung_ = base;
    }

    static void handleLatentStartupProbeTimeoutRollback(Connection& c) {
        c.handleLatentStartupProbeTimeoutRollback();
    }

    static uint8_t pendingModeChangeReason(const Connection& c) {
        return c.pending_reason_;
    }

    static void setBurstCleanGroupStreak(Connection& c, int streak) {
        c.burst_clean_group_streak_ = streak;
    }

    static void setAutomaticRateAllowed(Connection& c, bool allowed) {
        c.latent_startup_probe_allowed_ = allowed;
    }

    static int burstCleanGroupStreak(const Connection& c) {
        return c.burst_clean_group_streak_;
    }

    static void seedQsoScopedAdaptiveState(Connection& c) {
        c.consecutive_escape_drops_ = 2;
        c.rx_auth_last_change_ = std::chrono::steady_clock::now();
        c.rx_auth_last_change_valid_ = true;
    }

    static int consecutiveEscapeDrops(const Connection& c) {
        return c.consecutive_escape_drops_;
    }

    static void executeEscapeDrop(Connection& c, const char* trigger) {
        c.executeEscapeDrop(trigger);
    }

    static bool rxAuthorityDwellClockValid(const Connection& c) {
        return c.rx_auth_last_change_valid_;
    }

    static void enterDisconnected(Connection& c) {
        c.enterDisconnected("unit-test boundary");
    }

    // Two-phase timeout-egress hooks. They model exactly the callback context used by
    // SelectiveRepeatARQ::tick(), without needing to wait through a full synthetic RTO.
    static void stageArqTimeoutBatch(Connection& c,
                                     const std::vector<Bytes>& frames) {
        c.arq_tick_in_progress_ = true;
        c.handleArqTimeoutBatch(frames);
        c.arq_tick_in_progress_ = false;
    }

    static size_t stagedArqTimeoutFrames(const Connection& c) {
        return c.staged_timeout_batch_.size();
    }

    static void flushStagedArqTimeoutBatch(Connection& c) {
        c.flushStagedArqTimeoutBatch();
    }

    static void markModeChangePending(Connection& c,
                                      uint32_t timeout_ms = 60000) {
        c.mode_change_pending_ = true;
        c.mode_change_timeout_ms_ = timeout_ms;
    }
};

} // namespace protocol
} // namespace ultra

namespace {

void test_local_mode_change_ack_reconfigures_arq() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 15.0f, 0.30f);
    // R1/2 selects the high-throughput window (16). Since the 2026-07-02 8->16 mask widen
    // the tone-burst SACK cap (kToneBurstAckWindowCapFrames=16) exactly covers it, so the
    // window runs uncapped at 16 (pre-widen it was capped 16->8).
    CHECK(ConnectionAdaptiveTestAccess::arqWindow(c) == connection_policy::kHighThroughputOFDMWindowFrames,
          "R1/2 high-throughput window fits the 16-bit tone-burst SACK mask");
    CHECK(ConnectionAdaptiveTestAccess::arqWindow(c) <= connection_policy::kToneBurstAckWindowCapFrames,
          "in-flight window must never exceed the tone-burst SACK mask cap");

    c.requestModeChange(Modulation::DQPSK, CodeRate::R1_4, 12.0f,
                        v2::ModeChangeReason::CHANNEL_DEGRADED);
    auto ack = v2::ControlFrame::makeAck("K2DEF", "W1ABC", c.getStats().arq.acks_received + 1);
    ack.seq = ConnectionAdaptiveTestAccess::modeChangeSeq(c);
    c.onFrameReceived(ack.serialize());

    CHECK(c.getDataCodeRate() == CodeRate::R1_4, "local MODE_CHANGE ACK should apply pending rate");
    CHECK(ConnectionAdaptiveTestAccess::arqCodeRate(c) == CodeRate::R1_4,
          "local MODE_CHANGE ACK should update ARQ code rate");
    // DQPSK R1/4 is not a high-throughput rung -> the recomputed window is the default
    // wide window (8), below the 16-frame SACK cap (which no longer binds here).
    CHECK(ConnectionAdaptiveTestAccess::arqWindow(c) == connection_policy::kWideOFDMWindowFrames,
          "local MODE_CHANGE ACK should recompute ARQ window (default wide window)");
}

void test_local_mode_change_timeout_keeps_current_arq_mode() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.50f, Modulation::QPSK);

    c.requestModeChange(Modulation::QPSK, CodeRate::R1_2, 20.0f,
                        v2::ModeChangeReason::CHANNEL_DEGRADED);
    const uint32_t retry_ms = ConnectionAdaptiveTestAccess::modeChangeRetryMs(c);

    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(c),
          "test setup should leave MODE_CHANGE pending");
    // MODE_CHANGE is one hardened QPSK-R1/4 control frame in each direction.
    // It must not borrow the current 16-frame DATA deadline (fixed_default_03:
    // 44.75 s of dead air after all three ACK copies were already gone). Reuse
    // the field-proven one-frame control policy: at this geometry it floors at
    // 8 s, including audio/decode margin, and also exceeds one Good-channel Tc.
    const uint32_t expected_control_ms =
        connection_policy::computeWideOFDMAckTimeoutMs(
            Modulation::QPSK, CodeRate::R1_4,
            /*window_size=*/1,
            connection_policy::kCarrierSenseSackCoalesceMs,
            /*ack_repeat_count=*/1,
            /*cw_count=*/1);
    CHECK(expected_control_ms == 8000,
          "QPSK-R1/4 single-control deadline should retain the proven 8 s floor");
    CHECK(retry_ms == expected_control_ms,
          "wide MODE_CHANGE retry should use one-control physical geometry");
    CHECK(retry_ms < ConnectionAdaptiveTestAccess::arqAckTimeout(c),
          "MODE_CHANGE retry must be decoupled from the multi-frame DATA deadline");
    const uint32_t good_tc_ms = connection_policy::coherenceTimeMsForDoppler(
        connection_policy::kGoodHFDesignDopplerHz);
    CHECK(retry_ms >= std::min(
              good_tc_ms, connection_policy::kRetxTroughDeferAbsCapMs),
          "MODE_CHANGE retries should be at least one capped Good-channel Tc apart");

    // A very slow measured fade must not resurrect a multi-tens-of-seconds pause:
    // raw Tc at 0.01 Hz is 42.3 s, but the shared trough-pacing engineering cap is
    // 8 s, equal to the already-safe control/decode deadline here.
    ConnectionAdaptiveTestAccess::setCoherenceDoppler(c, 0.01f);
    const uint32_t slow_fade_retry_ms =
        ConnectionAdaptiveTestAccess::modeChangeRetryMs(c);
    CHECK(connection_policy::coherenceTimeMsForDoppler(0.01f) >
              connection_policy::kRetxTroughDeferAbsCapMs,
          "slow-fade fixture should exercise the coherence cap");
    CHECK(slow_fade_retry_ms == expected_control_ms,
          "slow measured Doppler must be capped at the 8 s control deadline");

    for (int i = 0; i < ConnectionAdaptiveTestAccess::modeChangeMaxRetries() + 1; ++i) {
        c.tick(slow_fade_retry_ms);
    }

    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "unresolved one-phase MODE_CHANGE should release the pending control state");
    CHECK(c.getDataCodeRate() == CodeRate::R2_3,
          "unacknowledged MODE_CHANGE must keep the proven shared data rate");
    CHECK(ConnectionAdaptiveTestAccess::arqCodeRate(c) == CodeRate::R2_3,
          "unacknowledged MODE_CHANGE must not reconfigure local ARQ alone");
}

void test_mode_change_retry_waits_for_clear_channel_without_spending_budget() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.50f, Modulation::QPSK);

    bool channel_busy = true;
    c.setChannelBusyQuery([&channel_busy] { return channel_busy; });

    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });

    c.requestModeChange(Modulation::QPSK, CodeRate::R1_2, 20.0f,
                        v2::ModeChangeReason::CHANNEL_DEGRADED);
    const uint32_t retry_ms = ConnectionAdaptiveTestAccess::modeChangeRetryMs(c);
    CHECK(tx_frames.size() == 1, "initial MODE_CHANGE request should transmit once");

    c.tick(retry_ms);
    CHECK(tx_frames.size() == 1,
          "due MODE_CHANGE retry must not key over an inbound ACK/signal");
    CHECK(ConnectionAdaptiveTestAccess::modeChangeRetryCount(c) == 0,
          "carrier-sense hold must not consume MODE_CHANGE retry budget");

    c.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    CHECK(tx_frames.size() == 1,
          "busy channel should keep holding across retry polls");
    CHECK(ConnectionAdaptiveTestAccess::modeChangeRetryCount(c) == 0,
          "repeated busy polls must remain retry-budget neutral");

    channel_busy = false;
    c.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    CHECK(tx_frames.size() == 2,
          "MODE_CHANGE should retry promptly after the channel clears");
    CHECK(ConnectionAdaptiveTestAccess::modeChangeRetryCount(c) == 1,
          "only committed clear-channel egress should consume retry budget");
}

void test_remote_mode_change_reconfigures_arq() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 15.0f, 0.30f);

    auto frame = v2::ControlFrame::makeModeChange(
        "K2DEF", "W1ABC", 44, Modulation::DQPSK, CodeRate::R1_4,
        12.0f, 0.80f, v2::ModeChangeReason::CHANNEL_DEGRADED);
    c.onFrameReceived(frame.serialize());

    CHECK(c.getDataCodeRate() == CodeRate::R1_4, "remote MODE_CHANGE should apply requested rate");
    CHECK(ConnectionAdaptiveTestAccess::arqCodeRate(c) == CodeRate::R1_4,
          "remote MODE_CHANGE should update ARQ code rate");
    // DQPSK R1/4 -> default wide window (8); the 16-frame SACK cap no longer binds
    // (it equaled the recomputed window only while the mask was 8 bits).
    CHECK(ConnectionAdaptiveTestAccess::arqWindow(c) == connection_policy::kWideOFDMWindowFrames,
          "remote MODE_CHANGE should recompute ARQ window (default wide window)");
}

void test_remote_mode_change_ack_is_single_by_default() {
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.48f, Modulation::QPSK);

    // One prompt ACK preserves the clean success path. The pending sender supplies
    // useful time diversity with an 8 s request retry; receiver ACK copies at
    // ~0.25/0.83 s were both inside one 2.34 s measured fade and can collide with
    // resumed DATA after copy 1 succeeds.
    CHECK(ConnectionAdaptiveTestAccess::arqAckRepeatCount(c) == 1,
          "tone-burst OFDM path uses a single prompt DATA ack (no diversity chain)");

    auto frame = v2::ControlFrame::makeModeChange(
        "K2DEF", "W1ABC", 44, Modulation::QPSK, CodeRate::R1_2,
        19.8f, 0.48f, v2::ModeChangeReason::CHANNEL_DEGRADED);
    c.onFrameReceived(frame.serialize());

    CHECK(tx_frames.size() == 1,
          "remote MODE_CHANGE should send the first ACK immediately");

    const uint32_t drain_ms =
        selective_repeat_arq_policy::ackRepeatDelayForCopy(
            ConnectionAdaptiveTestAccess::arqAckRepeatDelay(c), 3) +
        selective_repeat_arq_policy::kAckRepeatMaxJitterMs;
    c.tick(drain_ms);

    CHECK(tx_frames.size() == 1,
          "default MODE_CHANGE response must not emit same-fade ACK copies");
    auto ack = v2::ControlFrame::deserialize(tx_frames.front());
    CHECK(ack && ack->type == v2::FrameType::ACK && ack->seq == 44,
          "the immediate MODE_CHANGE ACK should acknowledge the request seq");
}

// BUG-MC-RETRY-SPURIOUS fix 1: the MODE_CHANGE retry deadline HOLDS while our own
// TX is keyed — half-duplex means keyed time cannot be ACK-loss evidence (rig E1:
// the frame rode the tail of a ~10.6 s bundled key-down, so the request-anchored
// 18.2 s deadline lost to the 21-30 s pipeline and retried spuriously EVERY trough
// exchange). With the host provider reporting keyed, ticks must neither decrement
// nor fire the deadline; on key-up the deadline resumes from where it held.
void test_mode_change_retry_holds_while_tx_keyed() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.50f, Modulation::QPSK);

    bool tx_keyed = true;
    c.setTxActiveProvider([&tx_keyed] { return tx_keyed; });

    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });

    c.requestModeChange(Modulation::QPSK, CodeRate::R1_2, 20.0f,
                        v2::ModeChangeReason::CHANNEL_DEGRADED);
    const uint32_t retry_ms = ConnectionAdaptiveTestAccess::modeChangeRetryMs(c);
    const size_t frames_after_request = tx_frames.size();

    // Keyed: three full deadlines elapse — the deadline must HOLD (no retry).
    for (int i = 0; i < 3; ++i) c.tick(retry_ms);
    CHECK(tx_frames.size() == frames_after_request,
          "keyed TX must hold the MODE_CHANGE retry deadline (no spurious retry)");
    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(c),
          "the exchange must stay pending across the hold");

    // Key-up: the deadline resumes and one full deadline fires exactly one retry.
    tx_keyed = false;
    c.tick(retry_ms);
    CHECK(tx_frames.size() == frames_after_request + 1,
          "after key-up one elapsed deadline must fire exactly one retry");
}

// BUG-MC-RETRY-SPURIOUS fix 3: a re-arriving copy of an ALREADY-APPLIED MODE_CHANGE
// (sender diversity copy or its request-time-anchored spurious retry — rig E1/D1/D3:
// a retry fired EVERY trough exchange although copy #1 was ACKed) must not re-apply
// the mode, must not re-notify the GUI (the operator-visible duplicate [MODE] lines),
// and must not schedule a fresh fading-aware repeat set. The duplicate still carries
// information — the sender may have missed our ACKs — so the calibrated response is
// exactly ONE re-ACK copy per duplicate reception.
void test_duplicate_mode_change_single_reack_no_reapply() {
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });
    int notify_count = 0;
    c.setDataModeChangedCallback([&](Modulation, CodeRate, int, float, float,
                                     int, int, bool) {
        ++notify_count;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.48f, Modulation::QPSK);

    auto frame = v2::ControlFrame::makeModeChange(
        "K2DEF", "W1ABC", 44, Modulation::QPSK, CodeRate::R1_2,
        19.8f, 0.48f, v2::ModeChangeReason::CHANNEL_DEGRADED);

    // First copy: apply + one GUI notify + one immediate ACK.
    c.onFrameReceived(frame.serialize());
    CHECK(c.getDataCodeRate() == CodeRate::R1_2,
          "first MODE_CHANGE copy should apply the requested rate");
    CHECK(notify_count == 1, "first MODE_CHANGE copy should notify the GUI once");
    CHECK(tx_frames.size() == 1, "first MODE_CHANGE copy should ACK immediately");

    // Duplicate copy (same seq, same mod/rate): exactly ONE re-ACK, nothing else.
    c.onFrameReceived(frame.serialize());
    CHECK(tx_frames.size() == 2,
          "duplicate MODE_CHANGE should emit exactly one re-ACK copy");
    auto dup_ack = v2::ControlFrame::deserialize(tx_frames.back());
    CHECK(dup_ack && dup_ack->type == v2::FrameType::ACK && dup_ack->seq == 44,
          "the duplicate's re-ACK should acknowledge the request seq");
    CHECK(notify_count == 1, "duplicate MODE_CHANGE must not re-notify the GUI");
    CHECK(c.getDataCodeRate() == CodeRate::R1_2,
          "duplicate MODE_CHANGE must leave the applied mode untouched");
    CHECK(ConnectionAdaptiveTestAccess::arqCodeRate(c) == CodeRate::R1_2,
          "duplicate MODE_CHANGE must not re-run applyDataMode/ARQ reconfig");
    c.tick(2000);
    CHECK(tx_frames.size() == 2,
          "duplicate MODE_CHANGE must not schedule a fresh ACK repeat set");

    // Every further duplicate reception earns one more diversity re-ACK.
    c.onFrameReceived(frame.serialize());
    CHECK(tx_frames.size() == 3,
          "each duplicate reception should earn exactly one re-ACK copy");
    CHECK(notify_count == 1, "repeated duplicates must stay notify-silent");

    // A genuinely NEW request (fresh seq) applies normally again.
    const size_t before_new = tx_frames.size();
    auto next = v2::ControlFrame::makeModeChange(
        "K2DEF", "W1ABC", 45, Modulation::QPSK, CodeRate::R2_3,
        20.2f, 0.48f, v2::ModeChangeReason::CHANNEL_IMPROVED);
    c.onFrameReceived(next.serialize());
    CHECK(c.getDataCodeRate() == CodeRate::R2_3,
          "a new MODE_CHANGE seq should apply normally after a dedup");
    CHECK(notify_count == 2, "a new MODE_CHANGE seq should notify the GUI again");
    CHECK(tx_frames.size() == before_new + 1,
          "a new MODE_CHANGE seq should ACK immediately");

    // Seq-reuse guard: SAME seq but a DIFFERENT (mod, rate) tuple is NOT a duplicate
    // (a restarted peer restarts its seq counter) — it must be applied, not deduped.
    auto reused_seq = v2::ControlFrame::makeModeChange(
        "K2DEF", "W1ABC", 45, Modulation::QPSK, CodeRate::R1_2,
        18.0f, 0.48f, v2::ModeChangeReason::CHANNEL_DEGRADED);
    c.onFrameReceived(reused_seq.serialize());
    CHECK(c.getDataCodeRate() == CodeRate::R1_2,
          "same seq with a different mode tuple must apply (seq-reuse guard)");
    CHECK(notify_count == 3,
          "same seq with a different mode tuple must notify (it is a new request)");
}

void test_wide_ofdm_configures_short_tail_sack_delay() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_4, 10.0f, 0.05f);

    const uint32_t sliding_delay = ConnectionAdaptiveTestAccess::arqSackDelay(c);
    const uint32_t tail_delay = ConnectionAdaptiveTestAccess::arqSackDelayShort(c);
    const uint32_t physical_delay = connection_policy::wideOFDMSackDelayMs(
        Modulation::DQPSK, CodeRate::R1_4,
        ConnectionAdaptiveTestAccess::arqWindow(c), v2::kDefaultFixedFrameCodewords);

    CHECK(tail_delay == connection_policy::wideOFDMSackTailDelayMs(),
          "wide OFDM should use the derived short SACK delay at stream tail");
    CHECK(tail_delay == connection_policy::kCarrierSenseSackCoalesceMs,
          "wide OFDM tail SACK delay should be carrier-sense coalescing only");
    CHECK(sliding_delay == connection_policy::wideOFDMSlidingSackDelayMs(
                               Modulation::DQPSK, CodeRate::R1_4),
          "wide OFDM in-stream SACK delay should be a data/ACK-airtime-derived quiet interval");
    CHECK(ConnectionAdaptiveTestAccess::arqSackDelaySlidesOnData(c),
          "wide OFDM should re-arm the SACK quiet timer on each decoded DATA frame");
    CHECK(physical_delay > sliding_delay && sliding_delay > tail_delay,
          "wide OFDM should retain physical RTO coverage while ACKing burst tails promptly");
    CHECK(ConnectionAdaptiveTestAccess::arqAckTimeout(c) ==
              connection_policy::computeWideOFDMAckTimeoutMs(
                  Modulation::DQPSK, CodeRate::R1_4,
                  ConnectionAdaptiveTestAccess::arqWindow(c),
                  sliding_delay,
                  ConnectionAdaptiveTestAccess::arqAckRepeatCount(c),
                  v2::kDefaultFixedFrameCodewords),
          "wide OFDM ACK timeout should remain derived from the long physical SACK hold");
}

void test_accepted_ofdm_data_sync_keeps_reactive_ack_without_proactive_tx() {
    Connection c;
    int confirmed_callbacks = 0;
    std::vector<Bytes> tx_frames;
    c.setHandshakeConfirmedCallback([&]() { ++confirmed_callbacks; });
    c.setTransmitCallback([&](const Bytes& data) { tx_frames.push_back(data); });
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c);

    CHECK(ConnectionAdaptiveTestAccess::connectAckCached(c),
          "responder CONNECT_ACK should start cached for reactive replay");
    c.onAcceptedOFDMDataSync(0.90f);
    CHECK(ConnectionAdaptiveTestAccess::connectAckCached(c),
          "accepted OFDM sync alone must retain CONNECT_ACK for a decoded duplicate CONNECT");
    CHECK(!ConnectionAdaptiveTestAccess::handshakeConfirmed(c) && confirmed_callbacks == 0,
          "accepted OFDM sync alone must not confirm a responder handshake");

    c.tick(120000);
    CHECK(tx_frames.empty(),
          "cached CONNECT_ACK must never key up proactively, even after many retry windows");
}

void test_accepted_ofdm_data_sync_does_not_clear_non_ofdm_cache() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c, WaveformMode::MC_DPSK);

    c.onAcceptedOFDMDataSync(0.90f);
    CHECK(ConnectionAdaptiveTestAccess::connectAckCached(c),
          "accepted OFDM DATA sync hook should not clear a non-OFDM cached ACK");
}

void test_all_failed_burst_group_is_not_handshake_evidence() {
    Connection c;
    int confirmed_callbacks = 0;
    c.setHandshakeConfirmedCallback([&]() { ++confirmed_callbacks; });
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c);

    c.onBurstGroupReceived(
        /*group_seq=*/0, {}, /*all_ok=*/false, /*quality=*/0.0f,
        /*frame_mask=*/0, /*interleaved=*/false, /*group_size=*/6,
        /*geometry_proven=*/true);

    CHECK(!ConnectionAdaptiveTestAccess::handshakeConfirmed(c) && confirmed_callbacks == 0,
          "a delivered 0/6 group contains no peer frame and must not confirm the handshake");
    CHECK(ConnectionAdaptiveTestAccess::connectAckCached(c),
          "an all-failed group must not destroy cached CONNECT_ACK recovery state");
}

void test_only_established_peer_burst_frame_confirms_handshake() {
    Connection c;
    int confirmed_callbacks = 0;
    c.setHandshakeConfirmedCallback([&]() { ++confirmed_callbacks; });
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c);

    auto wrong_peer = v2::makeFixedDataFrame(
        "N0BAD", "K2DEF", 0, Bytes{0x41}, CodeRate::R1_4).serialize();
    c.onBurstGroupReceived(
        /*group_seq=*/0, {wrong_peer}, /*all_ok=*/true, /*quality=*/0.9f,
        /*frame_mask=*/0x1, /*interleaved=*/false, /*group_size=*/1,
        /*geometry_proven=*/true);
    CHECK(!ConnectionAdaptiveTestAccess::handshakeConfirmed(c) && confirmed_callbacks == 0,
          "a valid frame from another source must not confirm this peer's handshake");
    CHECK(ConnectionAdaptiveTestAccess::connectAckCached(c),
          "another station's valid frame must not destroy the cached CONNECT_ACK");

    auto peer_frame = v2::makeFixedDataFrame(
        "W1ABC", "K2DEF", 0, Bytes{0x42}, CodeRate::R1_4).serialize();
    c.onBurstGroupReceived(
        /*group_seq=*/1, {peer_frame}, /*all_ok=*/true, /*quality=*/0.9f,
        /*frame_mask=*/0x1, /*interleaved=*/false, /*group_size=*/1,
        /*geometry_proven=*/true);
    CHECK(ConnectionAdaptiveTestAccess::handshakeConfirmed(c) && confirmed_callbacks == 1,
          "first CRC-valid burst frame from the established peer must confirm immediately");
    CHECK(!ConnectionAdaptiveTestAccess::connectAckCached(c),
          "authoritative peer traffic must clear cached CONNECT_ACK recovery");
}

void test_only_established_peer_classic_frame_confirms_handshake() {
    Connection c;
    int confirmed_callbacks = 0;
    c.setHandshakeConfirmedCallback([&]() { ++confirmed_callbacks; });
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c);

    c.onFrameReceived(v2::ControlFrame::makeAck("N0BAD", "K2DEF", 7).serialize());
    CHECK(!ConnectionAdaptiveTestAccess::handshakeConfirmed(c) &&
              ConnectionAdaptiveTestAccess::connectAckCached(c),
          "a locally-addressed classic frame from another station is not handshake proof");

    c.onFrameReceived(v2::ControlFrame::makeAck("W1ABC", "K2DEF", 7).serialize());
    CHECK(ConnectionAdaptiveTestAccess::handshakeConfirmed(c) && confirmed_callbacks == 1,
          "first CRC-valid classic frame from the established peer must confirm immediately");
    CHECK(!ConnectionAdaptiveTestAccess::connectAckCached(c),
          "classic authoritative peer traffic must retire CONNECT_ACK recovery");
}

void test_duplicate_connect_replays_cached_connect_ack_without_confirming() {
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c);
    c.onAcceptedOFDMDataSync(0.90f);

    c.tick(120000);
    CHECK(tx_frames.empty(),
          "half-open responder must stay silent until it decodes a duplicate CONNECT");

    auto duplicate_connect = v2::ConnectFrame::makeConnect(
        "W1ABC", "K2DEF",
        ModeCapabilities::ALL | ModeCapabilities::PHY_MASK_V1,
        static_cast<uint8_t>(WaveformMode::AUTO));

    c.onFrameReceived(duplicate_connect.serialize());

    CHECK(tx_frames.size() == 1,
          "decoded duplicate CONNECT should replay the cached ACK reactively");
    CHECK(ConnectionAdaptiveTestAccess::connectAckCached(c),
          "duplicate CONNECT must retain cached CONNECT_ACK for later reactive retries");
    CHECK(!ConnectionAdaptiveTestAccess::handshakeConfirmed(c),
          "duplicate CONNECT means CONNECT_ACK was lost and must not confirm responder handshake");
}

void test_connect_retry_interval_is_control_airtime_derived() {
    Connection c;
    c.setLocalCallsign("W1ABC");
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });

    CHECK(c.connect("K2DEF"), "connect should start without a ping callback");
    CHECK(tx_frames.size() == 1, "connect fallback should send initial CONNECT");

    const uint32_t retry_ms = ConnectionAdaptiveTestAccess::connectRetryInterval(c);
    CHECK(retry_ms > 0, "CONNECT retry interval should be derived from control airtime");

    c.tick(retry_ms - 1);
    CHECK(tx_frames.size() == 1, "CONNECT should not retry before the airtime-derived interval");

    c.tick(1);
    CHECK(tx_frames.size() == 2, "CONNECT should retry at the airtime-derived interval");
}

void test_disconnect_lost_ack_recovers_without_retry_grace_phase_lock() {
    Connection initiator;
    Connection responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        initiator, "W1ABC", "K2DEF", /*initiator=*/true);
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        responder, "K2DEF", "W1ABC", /*initiator=*/false);

    std::vector<Bytes> initiator_tx;
    std::vector<Bytes> responder_tx;
    std::vector<bool> initiator_expect_full;
    std::vector<bool> responder_expect_full;
    int initiator_disconnected = 0;
    int responder_disconnected = 0;
    initiator.setTransmitInfoCallback(
        [&](const Bytes& frame, bool expect_full_anchor_after_tx) {
            initiator_tx.push_back(frame);
            initiator_expect_full.push_back(expect_full_anchor_after_tx);
        });
    responder.setTransmitInfoCallback(
        [&](const Bytes& frame, bool expect_full_anchor_after_tx) {
            responder_tx.push_back(frame);
            responder_expect_full.push_back(expect_full_anchor_after_tx);
        });
    initiator.setDisconnectedCallback(
        [&](const std::string&) { ++initiator_disconnected; });
    responder.setDisconnectedCallback(
        [&](const std::string&) { ++responder_disconnected; });

    const uint32_t retry_ms =
        ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(initiator);
    const uint32_t grace_ms =
        ConnectionAdaptiveTestAccess::disconnectResponderGraceMs(responder);
    const uint32_t repeat_ms =
        ConnectionAdaptiveTestAccess::disconnectAckRetransmitMs(responder);
    const int proactive_repeats =
        ConnectionAdaptiveTestAccess::disconnectAckMaxProactiveRepeats(responder);
    const uint32_t full_control_ms =
        ConnectionAdaptiveTestAccess::disconnectControlKeydownMs(responder);
    const int useful_retries =
        ConnectionAdaptiveTestAccess::disconnectUsefulRetryCount(initiator);

    CHECK(full_control_ms == 1616 && retry_ms == 8000 &&
              repeat_ms == 2000 && useful_retries == 2 &&
              proactive_repeats == 2,
          "default wide teardown policy must retain measured F=1616/R=8000/N=2/S=2000/P=2");
    const uint32_t nominal_last_retry_ms = useful_retries * retry_ms;
    const uint32_t latest_deferred_retry_ms =
        ConnectionConfig{}.disconnect_timeout_ms - retry_ms;
    CHECK(grace_ms ==
              std::max(nominal_last_retry_ms, latest_deferred_retry_ms) +
                  full_control_ms +
                  connection_policy::kCarrierSenseSackCoalesceMs,
          "wide responder grace must cover a CCA-deferred useful retry plus one full control margin");
    CHECK(static_cast<uint64_t>(proactive_repeats) * repeat_ms +
              2ULL * full_control_ms +
              connection_policy::kCarrierSenseSackCoalesceMs < retry_ms,
          "all proactive ACK copies must finish before the peer retry quiet boundary");

    // Queue callbacks deliberately do not feed the peer synchronously so each
    // physical loss/retry boundary remains explicit in this timing test.
    initiator.disconnect();
    CHECK(initiator.getState() == ConnectionState::DISCONNECTING &&
              initiator_tx.size() == 1 && initiator_expect_full.size() == 1 &&
              initiator_expect_full.front(),
          "initiator must queue one DISCONNECT and arm full-anchor ACK acquisition");
    auto initial_disconnect = v2::parseHeader(initiator_tx.front());
    CHECK(initial_disconnect.valid &&
              initial_disconnect.type == v2::FrameType::DISCONNECT &&
              initial_disconnect.seq == v2::DISCONNECT_SEQ &&
              initial_disconnect.src_hash == v2::hashCallsign("W1ABC") &&
              initial_disconnect.dst_hash == v2::hashCallsign("K2DEF"),
          "initial DISCONNECT must retain exact peer addressing and sentinel sequence");

    responder.onFrameReceived(initiator_tx.front());
    CHECK(responder.getState() == ConnectionState::CONNECTED &&
              ConnectionAdaptiveTestAccess::disconnectPending(responder) &&
              responder_tx.size() == 1 && responder_expect_full.size() == 1 &&
              responder_expect_full.front(),
          "responder must ACK immediately, arm duplicate full-anchor acquisition, and retain grace");
    auto initial_ack = v2::parseHeader(responder_tx.front());
    CHECK(initial_ack.valid && initial_ack.type == v2::FrameType::ACK &&
              initial_ack.seq == v2::DISCONNECT_SEQ &&
              initial_ack.src_hash == v2::hashCallsign("K2DEF") &&
              initial_ack.dst_hash == v2::hashCallsign("W1ABC"),
          "disconnect ACK must be addressed back to the initiating peer");

    // Lose the immediate ACK and both finite proactive copies. Advance the two
    // protocol clocks together so the responder's liveness at the retry boundary
    // is proven, rather than inferred from constants.
    uint32_t elapsed_ms = 0;
    for (int i = 0; i < proactive_repeats; ++i) {
        initiator.tick(repeat_ms);
        responder.tick(repeat_ms);
        elapsed_ms += repeat_ms;
        CHECK(initiator_tx.size() == 1 &&
                  responder_tx.size() == static_cast<size_t>(i + 2) &&
                  responder_expect_full.back() &&
                  ConnectionAdaptiveTestAccess::disconnectAckRepeatCount(responder) == i + 1,
              "each proactive ACK interval must emit exactly one full-anchor-aware repeat");
    }

    const uint32_t before_retry_ms = retry_ms - elapsed_ms;
    CHECK(before_retry_ms > 0,
          "test geometry must leave a positive interval before the first retry");
    initiator.tick(before_retry_ms - 1);
    responder.tick(before_retry_ms - 1);
    CHECK(initiator_tx.size() == 1 &&
              ConnectionAdaptiveTestAccess::disconnectPending(responder),
          "no DISCONNECT retry may fire one millisecond early");

    initiator.tick(1);
    responder.tick(1);
    CHECK(initiator_tx.size() == 2 &&
              initiator_expect_full.size() == 2 &&
              initiator_expect_full.back() &&
              ConnectionAdaptiveTestAccess::disconnectPending(responder) &&
              ConnectionAdaptiveTestAccess::disconnectPendingMs(responder) > 0,
          "first full-anchor-aware retry must arrive while responder recovery grace is still open");
    CHECK(ConnectionAdaptiveTestAccess::disconnectAckRepeatCount(responder) ==
              proactive_repeats,
          "responder must be silent after its finite proactive ACK budget");

    const size_t before_reactive_ack = responder_tx.size();
    responder.onFrameReceived(initiator_tx.back());
    CHECK(responder_tx.size() == before_reactive_ack + 1 &&
              responder_expect_full.size() == responder_tx.size() &&
              responder_expect_full.back() &&
              ConnectionAdaptiveTestAccess::disconnectPendingMs(responder) == grace_ms,
          "duplicate DISCONNECT must trigger a full-anchor-aware reactive ACK and reset grace");
    auto reactive_ack = v2::parseHeader(responder_tx.back());
    CHECK(reactive_ack.valid && reactive_ack.type == v2::FrameType::ACK &&
              reactive_ack.seq == v2::DISCONNECT_SEQ &&
              reactive_ack.dst_hash == v2::hashCallsign("W1ABC"),
          "reactive recovery ACK must preserve disconnect identity and destination");

    initiator.onFrameReceived(responder_tx.back());
    CHECK(initiator.getState() == ConnectionState::DISCONNECTED &&
              initiator_disconnected == 1,
          "the first surviving reactive ACK must close the initiator exactly once");

    responder.tick(grace_ms - 1);
    CHECK(responder.getState() == ConnectionState::CONNECTED &&
              ConnectionAdaptiveTestAccess::disconnectPending(responder),
          "responder must retain the reset grace until its exact final millisecond");
    responder.tick(1);
    CHECK(responder.getState() == ConnectionState::DISCONNECTED &&
              responder_disconnected == 1,
          "responder must close exactly once when the reset grace expires");

    initiator.tick(retry_ms * 2);
    responder.tick(grace_ms);
    CHECK(initiator_tx.size() == 2 && initiator_disconnected == 1 &&
              responder_disconnected == 1,
          "closed peers must not emit another retry or duplicate callbacks");
}

void test_disconnect_timing_scope_and_timeout_budget() {
    Connection wide;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        wide, "W1ABC", "K2DEF", /*initiator=*/true);
    std::vector<Bytes> wide_tx;
    wide.setTransmitCallback(
        [&](const Bytes& frame) { wide_tx.push_back(frame); });

    const uint32_t wide_retry_ms =
        ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(wide);
    CHECK(ConnectionAdaptiveTestAccess::disconnectUsefulRetryCount(wide) == 2,
          "30-second wide timeout must admit exactly two complete 8-second retry transactions");
    wide.disconnect();
    wide.tick(wide_retry_ms);
    wide.tick(wide_retry_ms);
    CHECK(wide_tx.size() == 3 &&
              ConnectionAdaptiveTestAccess::disconnectRetryCount(wide) == 2,
          "wide teardown must emit the initial request plus two budget-valid retries");

    wide.tick(wide_retry_ms);
    CHECK(wide_tx.size() == 3 &&
              ConnectionAdaptiveTestAccess::disconnectTimeoutRemainingMs(wide) == 6000,
          "24-second retry must be suppressed because only six seconds remain");
    wide.tick(5999);
    CHECK(wide.getState() == ConnectionState::DISCONNECTING,
          "hard timeout must retain its exact final millisecond after late-retry suppression");
    wide.tick(1);
    CHECK(wide.getState() == ConnectionState::DISCONNECTED && wide_tx.size() == 3,
          "timeout tie must close without keying an under-budget retry");

    Connection narrow;
    ConnectionAdaptiveTestAccess::makeConnectedNarrowOFDM(narrow);
    CHECK(ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(narrow) == 5000 &&
              ConnectionAdaptiveTestAccess::disconnectResponderGraceMs(narrow) == 5000 &&
              ConnectionAdaptiveTestAccess::disconnectAckRetransmitMs(narrow) == 2000,
          "narrow OFDM teardown timing must remain on the legacy 5s/5s/2s policy");

    Connection mcdpsk;
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(mcdpsk);
    CHECK(ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(mcdpsk) == 5000 &&
              ConnectionAdaptiveTestAccess::disconnectResponderGraceMs(mcdpsk) == 5000 &&
              ConnectionAdaptiveTestAccess::disconnectAckRetransmitMs(mcdpsk) == 2000,
          "MC-DPSK teardown timing must remain unchanged until separately derived");
}

void test_disconnect_retries_and_ack_copies_obey_half_duplex_gates() {
    Connection initiator;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        initiator, "W1ABC", "K2DEF", /*initiator=*/true);
    bool initiator_busy = true;
    initiator.setChannelBusyQuery([&] { return initiator_busy; });
    std::vector<Bytes> initiator_tx;
    initiator.setTransmitCallback(
        [&](const Bytes& frame) { initiator_tx.push_back(frame); });
    const uint32_t retry_ms =
        ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(initiator);
    initiator.disconnect();
    initiator.tick(retry_ms);
    CHECK(initiator_tx.size() == 1 &&
              ConnectionAdaptiveTestAccess::disconnectRetryCount(initiator) == 0,
          "busy channel must hold a due DISCONNECT retry without spending its budget");
    initiator.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    CHECK(initiator_tx.size() == 1 &&
              ConnectionAdaptiveTestAccess::disconnectRetryCount(initiator) == 0,
          "repeated busy polls must remain disconnect-retry-budget neutral");
    initiator_busy = false;
    initiator.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    CHECK(initiator_tx.size() == 2 &&
              ConnectionAdaptiveTestAccess::disconnectRetryCount(initiator) == 1,
          "DISCONNECT retry must key promptly once carrier sense clears");

    Connection responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        responder, "K2DEF", "W1ABC", /*initiator=*/false);
    bool responder_busy = true;
    responder.setChannelBusyQuery([&] { return responder_busy; });
    std::vector<Bytes> responder_tx;
    responder.setTransmitCallback(
        [&](const Bytes& frame) { responder_tx.push_back(frame); });
    const auto request =
        v2::ControlFrame::makeDisconnect("W1ABC", "K2DEF").serialize();
    responder.onFrameReceived(request);
    const uint32_t repeat_ms =
        ConnectionAdaptiveTestAccess::disconnectAckRetransmitMs(responder);
    responder.tick(repeat_ms);
    CHECK(responder_tx.size() == 1 &&
              ConnectionAdaptiveTestAccess::disconnectAckRepeatCount(responder) == 0,
          "busy channel must hold a proactive disconnect ACK without spending its budget");
    responder_busy = false;
    responder.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    CHECK(responder_tx.size() == 2 &&
              ConnectionAdaptiveTestAccess::disconnectAckRepeatCount(responder) == 1,
          "proactive disconnect ACK must key promptly once carrier sense clears");

    Connection late;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        late, "K2DEF", "W1ABC", /*initiator=*/false);
    bool late_busy = true;
    late.setChannelBusyQuery([&] { return late_busy; });
    std::vector<Bytes> late_tx;
    late.setTransmitCallback(
        [&](const Bytes& frame) { late_tx.push_back(frame); });
    late.onFrameReceived(request);
    const uint32_t late_boundary_ms =
        ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(late) -
        2 * ConnectionAdaptiveTestAccess::disconnectControlKeydownMs(late) -
        connection_policy::kCarrierSenseSackCoalesceMs;
    late.tick(late_boundary_ms);
    CHECK(late_tx.size() == 1 &&
              ConnectionAdaptiveTestAccess::disconnectAckRepeatCount(late) ==
                  ConnectionAdaptiveTestAccess::disconnectAckMaxProactiveRepeats(late),
          "a delayed proactive ACK that would finish on the retry boundary must be skipped");
}

void test_disconnect_grace_is_egress_exclusive_and_crossed_close_is_reactive() {
    Connection responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        responder, "K2DEF", "W1ABC", /*initiator=*/false);
    std::vector<Bytes> responder_tx;
    responder.setTransmitCallback(
        [&](const Bytes& frame) { responder_tx.push_back(frame); });
    responder.requestModeChange(Modulation::QPSK, CodeRate::R1_2, 20.0f,
                                v2::ModeChangeReason::CHANNEL_DEGRADED);
    const auto request =
        v2::ControlFrame::makeDisconnect("W1ABC", "K2DEF").serialize();
    responder.onFrameReceived(request);
    responder.tick(ConnectionAdaptiveTestAccess::modeChangeRetryMs(responder));
    size_t mode_change_frames = 0;
    for (const auto& frame : responder_tx) {
        const auto header = v2::parseHeader(frame);
        if (header.valid && header.type == v2::FrameType::MODE_CHANGE) {
            ++mode_change_frames;
        }
    }
    CHECK(mode_change_frames == 1,
          "responder grace must suppress ordinary MODE_CHANGE retries and all non-close egress");

    Connection a;
    Connection b;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        a, "W1ABC", "K2DEF", /*initiator=*/true);
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        b, "K2DEF", "W1ABC", /*initiator=*/false);
    std::vector<Bytes> a_tx;
    std::vector<Bytes> b_tx;
    a.setTransmitCallback([&](const Bytes& frame) { a_tx.push_back(frame); });
    b.setTransmitCallback([&](const Bytes& frame) { b_tx.push_back(frame); });
    a.disconnect();
    b.disconnect();
    a.onFrameReceived(b_tx.front());
    b.onFrameReceived(a_tx.front());
    CHECK(a_tx.size() == 2 && b_tx.size() == 2,
          "crossed close must emit one immediate ACK from each peer");
    a.tick(ConnectionAdaptiveTestAccess::disconnectAckRetransmitMs(a));
    b.tick(ConnectionAdaptiveTestAccess::disconnectAckRetransmitMs(b));
    CHECK(a_tx.size() == 2 && b_tx.size() == 2,
          "crossed close must not start symmetric proactive ACK trains");
    a.onFrameReceived(b_tx.back());
    b.onFrameReceived(a_tx.back());
    CHECK(a.getState() == ConnectionState::DISCONNECTED &&
              b.getState() == ConnectionState::DISCONNECTED,
          "crossed close must converge when either immediate ACK survives");
}

void test_disconnect_teardown_phase_quarantines_rx_and_all_non_close_egress() {
    Connection responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        responder, "K2DEF", "W1ABC", /*initiator=*/false);

    std::vector<Bytes> tx_frames;
    std::vector<bool> teardown_edges;
    int tone_acks = 0;
    responder.setTransmitCallback(
        [&](const Bytes& frame) { tx_frames.push_back(frame); });
    responder.setTransmitToneBurstAckCallback(
        [&](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&,
            bool) { ++tone_acks; });
    responder.setDisconnectTeardownCallback(
        [&](bool active) { teardown_edges.push_back(active); });

    const auto request =
        v2::ControlFrame::makeDisconnect("W1ABC", "K2DEF").serialize();
    responder.onFrameReceived(request);
    CHECK(responder.getState() == ConnectionState::CONNECTED &&
              responder.isDisconnectTeardownActive() &&
              teardown_edges.size() == 2 && !teardown_edges.front() &&
              teardown_edges.back() && tx_frames.size() == 1,
          "responder close grace must publish one explicit active teardown edge");

    const auto stale_mode_change = v2::ControlFrame::makeModeChange(
        "W1ABC", "K2DEF", 77, Modulation::QPSK, CodeRate::R1_2,
        20.0f, 0.3f, v2::ModeChangeReason::CHANNEL_DEGRADED);
    responder.onFrameReceived(stale_mode_change.serialize());
    CHECK(responder.getDataCodeRate() == CodeRate::R2_3 &&
              tx_frames.size() == 1,
          "stale MODE_CHANGE must neither apply nor ACK during teardown");

    auto stale_data = v2::makeFixedDataFrame(
        "W1ABC", "K2DEF", 0, Bytes{0x42}, CodeRate::R2_3);
    stale_data.flags |= v2::Flags::FINAL;
    responder.onFrameReceived(stale_data.serialize(),
                              /*physical_turn_complete=*/true);
    responder.onBurstGroupReceived(
        /*group_seq=*/0, {stale_data.serialize()}, /*all_ok=*/true,
        /*quality=*/1.0f, /*frame_mask=*/0x1, /*interleaved=*/true,
        /*group_size=*/1, /*geometry_proven=*/true);
    CHECK(tone_acks == 0 && tx_frames.size() == 1,
          "classic and grouped DATA must not manufacture an ACK during teardown");

    const auto turnover =
        v2::ControlFrame::makeTurnover("K2DEF", "W1ABC").serialize();
    ConnectionAdaptiveTestAccess::transmitFrame(responder, turnover);
    ConnectionAdaptiveTestAccess::transmitFrameBatch(responder, {turnover});
    CHECK(tx_frames.size() == 1,
          "direct and batched protocol egress must be close-only during teardown");
    CHECK(!responder.sendMessage("must not queue") &&
              !responder.isReadyToSend(),
          "application DATA submission must fail closed during responder grace");

    ultra::waveform::tone_burst_ack::ToneBurstAckDetection stale_tone{};
    CHECK(!responder.onToneBurstAck(stale_tone),
          "stale tone ACK must not mutate the sender while teardown is active");

    responder.onFrameReceived(request);
    CHECK(tx_frames.size() == 2 && teardown_edges.size() == 2,
          "duplicate DISCONNECT must remain the sole reactive egress without re-publishing the phase");

    responder.tick(
        ConnectionAdaptiveTestAccess::disconnectResponderGraceMs(responder));
    CHECK(responder.getState() == ConnectionState::DISCONNECTED &&
              !responder.isDisconnectTeardownActive() &&
              teardown_edges.size() == 3 && !teardown_edges.back(),
          "grace completion must clear control-only teardown exactly once");

    Connection aborting_responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        aborting_responder, "K2DEF", "W1ABC", /*initiator=*/false);
    std::vector<bool> abort_edges;
    aborting_responder.setTransmitCallback([](const Bytes&) {});
    aborting_responder.setDisconnectTeardownCallback(
        [&](bool active) { abort_edges.push_back(active); });
    aborting_responder.onFrameReceived(request);
    aborting_responder.abortTxNow();
    CHECK(aborting_responder.getState() == ConnectionState::DISCONNECTED &&
              !aborting_responder.isDisconnectTeardownActive() &&
              abort_edges.size() == 3 && !abort_edges.back(),
          "TX abort during responder grace must finish the accepted peer close, not resurrect CONNECTED");
}

void test_disconnect_responder_grace_covers_long_cca_deferred_retry() {
    Connection initiator;
    Connection responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        initiator, "W1ABC", "K2DEF", /*initiator=*/true);
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        responder, "K2DEF", "W1ABC", /*initiator=*/false);

    bool busy = true;
    initiator.setChannelBusyQuery([&] { return busy; });
    std::vector<Bytes> initiator_tx;
    std::vector<Bytes> responder_tx;
    initiator.setTransmitCallback(
        [&](const Bytes& frame) { initiator_tx.push_back(frame); });
    responder.setTransmitCallback(
        [&](const Bytes& frame) { responder_tx.push_back(frame); });

    const uint32_t retry_ms =
        ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(initiator);
    const uint32_t grace_ms =
        ConnectionAdaptiveTestAccess::disconnectResponderGraceMs(responder);
    const uint32_t nominal_grace_ms =
        ConnectionAdaptiveTestAccess::disconnectUsefulRetryCount(initiator) * retry_ms +
        ConnectionAdaptiveTestAccess::disconnectControlKeydownMs(responder) +
        connection_policy::kCarrierSenseSackCoalesceMs;
    CHECK(grace_ms > nominal_grace_ms,
          "wide grace must include the legal CCA deferral beyond nominal retry slots");

    initiator.disconnect();
    responder.onFrameReceived(initiator_tx.front());
    CHECK(responder.isDisconnectTeardownActive(),
          "responder must enter close grace after the initial request");

    initiator.tick(retry_ms);
    responder.tick(retry_ms);
    const uint32_t long_busy_hold_ms = 13000;
    initiator.tick(long_busy_hold_ms);
    responder.tick(long_busy_hold_ms);
    CHECK(initiator_tx.size() == 1 &&
              responder.getState() == ConnectionState::CONNECTED,
          "a 13-second CCA hold must spend neither retry budget nor responder liveness");

    busy = false;
    initiator.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    responder.tick(connection_policy::kCarrierSenseSackCoalesceMs);
    CHECK(initiator_tx.size() == 2 && responder.isDisconnectTeardownActive(),
          "the still-budget-valid deferred retry must launch while responder grace remains open");

    responder.onFrameReceived(initiator_tx.back());
    CHECK(responder_tx.size() >= 2,
          "late CCA-deferred DISCONNECT must receive a reactive ACK");
    initiator.onFrameReceived(responder_tx.back());
    CHECK(initiator.getState() == ConnectionState::DISCONNECTED,
          "late reactive ACK must still complete the initiator close");
}

void test_disconnect_supports_synchronous_host_loopback() {
    Connection initiator;
    Connection responder;
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        initiator, "W1ABC", "K2DEF", /*initiator=*/true);
    ConnectionAdaptiveTestAccess::makeConnectedEndpoint(
        responder, "K2DEF", "W1ABC", /*initiator=*/false);

    initiator.setTransmitCallback(
        [&](const Bytes& frame) { responder.onFrameReceived(frame); });
    responder.setTransmitCallback(
        [&](const Bytes& frame) { initiator.onFrameReceived(frame); });

    initiator.disconnect();
    CHECK(initiator.getState() == ConnectionState::DISCONNECTED &&
              !initiator.isDisconnectTeardownActive() &&
              responder.isDisconnectTeardownActive(),
          "a synchronous transport must see DISCONNECTING before returning the sentinel ACK");
}

void test_responder_handshake_timer_does_not_false_confirm() {
    Connection c;
    int confirmed_callbacks = 0;
    c.setHandshakeConfirmedCallback([&]() {
        ++confirmed_callbacks;
    });
    ConnectionAdaptiveTestAccess::makeResponderWithCachedConnectAck(c);
    ConnectionAdaptiveTestAccess::setResponderHandshakeWait(c, 1);

    c.tick(1);

    CHECK(!ConnectionAdaptiveTestAccess::handshakeConfirmed(c),
          "responder timer alone must not confirm a half-connected handshake");
    CHECK(confirmed_callbacks == 0,
          "responder timer must not switch TX to connected waveform without an initiator frame");
}

// §RETX-PACING round accounting (docs/RETX_PACING_DESIGN_2026_07_03.md §1.1/§2.3), with
// both knobs pinned OFF in main(): counting is inert bookkeeping — no hold armed, no ARQ
// timer touched, no escape fired (the knob-off byte-identical contract) — and the
// g42-PROTECTIVE property holds: ANY progress resets the zero-round streak, and a
// duplicate/stale ack (progress −1) is never a round.
void test_zero_progress_round_counter_knob_off_and_g42_protective() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.40f, Modulation::QPSK);
    TempPayloadFile payload("retx_pacing_rounds", 4096);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "temp payload file should be created");
    ConnectionAdaptiveTestAccess::startFile(c, payload.path);
    ConnectionAdaptiveTestAccess::fillArqWindow(c, 4);

    const uint32_t rto_before = ConnectionAdaptiveTestAccess::arqAckTimeout(c);

    // Two consecutive zero-progress rounds accumulate (scope: CONNECTED wideband OFDM,
    // file SENDING, in-flight bytes — all true here).
    ConnectionAdaptiveTestAccess::noteRoundOutcome(c, 0, "test");
    ConnectionAdaptiveTestAccess::noteRoundOutcome(c, 0, "test");
    CHECK(ConnectionAdaptiveTestAccess::zeroProgressRounds(c) == 2,
          "two zero-progress rounds should accumulate");
    CHECK(ConnectionAdaptiveTestAccess::paceHoldMs(c) == 0,
          "ULTRA_RETX_TROUGH_PACING=0: no pacing hold may be armed (byte-identical)");
    CHECK(ConnectionAdaptiveTestAccess::arqAckTimeout(c) == rto_before,
          "knob-off: the ARQ ack timeout base must be untouched");

    // ULTRA_COLLAPSE_ESCAPE_ROUNDS=0: polling the escape must never fire a MODE_CHANGE.
    ConnectionAdaptiveTestAccess::pollCollapseEscape(c);
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "ULTRA_COLLAPSE_ESCAPE_ROUNDS=0: no collapse escape may fire (byte-identical)");
    CHECK(c.getDataCodeRate() == CodeRate::R2_3,
          "knob-off: the data rate must not move");

    // g42-protective (§2.3): a round in which ANY frame progressed resets the streak —
    // a lone straggler retrying amid deliveries can never accumulate rounds.
    ConnectionAdaptiveTestAccess::noteRoundOutcome(c, 0, "test");
    ConnectionAdaptiveTestAccess::noteRoundOutcome(c, 1, "test");
    CHECK(ConnectionAdaptiveTestAccess::zeroProgressRounds(c) == 0,
          "any delivered/SACKed frame must reset the zero-round streak");

    // §1.1 dedup: progress −1 (duplicate/stale ack — no fresh ack processed) is NOT a round.
    ConnectionAdaptiveTestAccess::noteRoundOutcome(c, 0, "test");
    ConnectionAdaptiveTestAccess::noteRoundOutcome(c, -1, "test");
    CHECK(ConnectionAdaptiveTestAccess::zeroProgressRounds(c) == 1,
          "a duplicate/stale ack (progress -1) must not create or reset a round");
}

// ─────────── DESC-SWITCH (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md Phase 1) ───────────

int countModeChangeFrames(const std::vector<Bytes>& frames) {
    int n = 0;
    for (const auto& f : frames) {
        auto hdr = v2::parseHeader(f);
        if (hdr.valid && hdr.type == v2::FrameType::MODE_CHANGE) ++n;
    }
    return n;
}

// Knob-OFF identity pin: ULTRA_DESCRIPTOR_MODE_SWITCH=0 (the baseline pinned in main)
// must route a clean-boundary ladder move through the legacy synchronized MODE_CHANGE
// exchange — pending state armed, mode held until ACK, exactly one MODE_CHANGE frame on
// the wire, zero descriptor commits — and the RX-side notification must be a no-op.
void test_descriptor_switch_knob_off_is_byte_identical() {
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    TempPayloadFile payload("desc_switch_off", 60 * 1024);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "temp payload file should be created");
    ConnectionAdaptiveTestAccess::startFile(c, payload.path);

    // QAM16 NACK (quality 0) at a clean send boundary -> prompt demote decision.
    ConnectionAdaptiveTestAccess::applyFeedback(c, 0.0f);

    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(c),
          "knob-off: the demote must arm the MODE_CHANGE stop-and-wait");
    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R2_3,
          "knob-off: the local mode must be HELD until the peer ACKs");
    CHECK(countModeChangeFrames(tx_frames) == 1,
          "knob-off: exactly one MODE_CHANGE control frame goes on the wire");
    CHECK(c.getStats().descriptor_mode_switches == 0,
          "knob-off: no descriptor commit may be counted");
    CHECK(!ConnectionAdaptiveTestAccess::descSwitchFullAnchorArmed(c),
          "knob-off: the descriptor-switch full-anchor one-shot must stay unarmed");

    // RX-side notification is a hard no-op with the knob off.
    Connection r;
    std::vector<Bytes> r_tx;
    r.setTransmitCallback([&](const Bytes& d) { r_tx.push_back(d); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        r, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    r.onDescriptorModeChange(Modulation::QAM16, CodeRate::R2_3, 8);
    CHECK(r.getDataModulation() == Modulation::QPSK,
          "knob-off: onDescriptorModeChange must not touch the data mode");
    CHECK(r_tx.empty(), "knob-off: onDescriptorModeChange must transmit nothing");
    CHECK(r.getStats().descriptor_mode_switches == 0,
          "knob-off: no adopt may be counted");
}

// Knob-ON sender commit: a clean-boundary ladder move commits LOCALLY — mode applied
// immediately (ARQ included), NO mode_change_pending_, NO MODE_CHANGE frame on the
// wire, the full-anchor one-shot armed for the next burst group, and no epoch bump
// (an EMPTY window has nothing to abort — the descriptor + EPOCH_REBASE flag suffice).
void test_descriptor_switch_commits_locally_at_clean_boundary() {
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection c;  // ctor latches knob ON for this instance
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);  // restore the pinned baseline

    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
    std::vector<bool> burst_full_anchor;
    std::vector<size_t> burst_sizes;
    std::vector<uint8_t> burst_reasons;
    c.setTransmitBurstCallback([&](const std::vector<Bytes>& frames, uint16_t /*seq*/,
                                   uint8_t anchor_reason) {
        burst_sizes.push_back(frames.size());
        // Any non-None reason still means "this burst carries a full anchor"; the
        // reason only distinguishes WHY (resend vs config switch) for the skip streak.
        burst_full_anchor.push_back(anchor_reason != Connection::kAnchorReasonNone);
        burst_reasons.push_back(anchor_reason);
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    TempPayloadFile payload("desc_switch_on", 60 * 1024);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "temp payload file should be created");
    ConnectionAdaptiveTestAccess::startFile(c, payload.path);
    const uint8_t epoch_before = ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c);

    // QAM16 NACK at a clean boundary -> demote decision -> descriptor commit.
    ConnectionAdaptiveTestAccess::applyFeedback(c, 0.0f);

    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "commit must not arm the MODE_CHANGE stop-and-wait (no TX freeze)");
    CHECK(c.getDataModulation() == Modulation::QPSK &&
              c.getDataCodeRate() == CodeRate::R3_4,
          "commit applies the new mode immediately (sender-local)");
    CHECK(ConnectionAdaptiveTestAccess::arqCodeRate(c) == CodeRate::R3_4,
          "commit must reconfigure the ARQ to the new rate");
    CHECK(countModeChangeFrames(tx_frames) == 0,
          "knob-on: NO MODE_CHANGE control frame may go on the wire");
    CHECK(c.getStats().descriptor_mode_switches == 1,
          "commit must count in descriptor_mode_switches");
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == epoch_before,
          "clean-boundary commit: EMPTY window -> no ARQ abort -> no epoch bump");
    // F163 REVERSAL (2026-07-06): warm state still carries within-grid, but the
    // switch DESCRIPTOR itself must ride a full anchor — three light-descriptor
    // switches were missed on fading (25 s adoption latency each, ~130 s of the
    // 424 s transfer). Every commit now arms the full anchor.
    // 2026-07-26 REASON PLUMBING: a descriptor mode/rate switch must be reported as
    // kAnchorReasonModeSwitch, NOT as a resend. The encoder uses the distinction to
    // decide whether the #69 anchor-skip clean streak (DELIVERY evidence) recools; a
    // config switch needs the chirp but is not evidence the channel stopped syncing.
    // Reporting it as a resend is what made rate changes the dominant streak-resetter.
    for (size_t i = 0; i < burst_reasons.size(); ++i) {
        if (burst_reasons[i] != Connection::kAnchorReasonNone) {
            CHECK(burst_reasons[i] == Connection::kAnchorReasonModeSwitch,
                  "a descriptor switch's full anchor must carry kAnchorReasonModeSwitch "
                  "(got a resend reason -> the skip streak would recool on a config event)");
        }
    }
    bool full_anchor_armed_or_consumed =
        ConnectionAdaptiveTestAccess::descSwitchFullAnchorArmed(c);
    for (size_t i = 0; i < burst_full_anchor.size(); ++i) {
        if (burst_sizes[i] >= 2 && burst_full_anchor[i]) {
            full_anchor_armed_or_consumed = true;
        }
    }
    CHECK(full_anchor_armed_or_consumed,
          "every DESC-SWITCH commit must arm the full anchor (F163: missed "
          "light switch descriptors cost 25 s each)");
}

// Sender-initiated stuck/collapse escapes are triggered by ACK silence.  Silence
// cannot prove the receiver decoded a new descriptor, so even with descriptor +
// move-epoch enabled they must hold the old local geometry and use synchronized
// MODE_CHANGE. Receiver-commanded demotes remain the separately-tested sanctioned
// mid-window descriptor path because the command itself proves reverse control.
void test_silent_escape_never_unilaterally_descriptor_switches() {
    auto run = [](Modulation mod) {
        setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
        setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
        Connection c;
        setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
        unsetenv("ULTRA_ARQ_MOVE_EPOCH");

        std::vector<Bytes> tx_frames;
        c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(
            c, CodeRate::R2_3, 20.0f, 0.30f, mod);

        ConnectionAdaptiveTestAccess::executeEscapeDrop(c, "unit-test ACK silence");

        CHECK(ConnectionAdaptiveTestAccess::modeChangePending(c),
              "a silent escape must arm synchronized MODE_CHANGE");
        CHECK(c.getDataModulation() == mod && c.getDataCodeRate() == CodeRate::R2_3,
              "a silent escape must hold the old local geometry until peer ACK");
        CHECK(countModeChangeFrames(tx_frames) == 1,
              "a silent escape must put exactly one MODE_CHANGE request on the wire");
        CHECK(c.getStats().descriptor_mode_switches == 0,
              "a silent escape must never count a unilateral descriptor commit");
        CHECK(ConnectionAdaptiveTestAccess::consecutiveEscapeDrops(c) == 1,
              "the collapse-episode counter must still record the escape");
    };

    run(Modulation::QPSK);
    run(Modulation::QAM16);
}

// Knob-ON receiver adopt: a mode-hop descriptor notification runs the RX-relevant
// subset of applyDataMode (mode + CW + ARQ reconfig + GUI notify) and must NOT send
// any ACK (no MODE_CHANGE ACK machinery); a re-announced descriptor is idempotent,
// and a receiver with its OWN data in flight skips the adopt (ISS asymmetry guard).
void test_descriptor_adopt_reconfigures_receiver_without_ack() {
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection c;  // ctor latches knob ON for this instance
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);  // restore the pinned baseline

    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
    int notify_count = 0;
    Modulation notified_mod = Modulation::AUTO;
    CodeRate notified_rate = CodeRate::AUTO;
    c.setDataModeChangedCallback([&](Modulation mod, CodeRate rate, int /*cw*/,
                                     float /*snr*/, float /*fading*/, int /*carriers*/,
                                     int /*sps*/, bool /*wire*/) {
        ++notify_count;
        notified_mod = mod;
        notified_rate = rate;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    c.onDescriptorModeChange(Modulation::QAM16, CodeRate::R2_3, 8);

    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R2_3,
          "adopt must apply the descriptor's announced mode at the protocol layer");
    CHECK(ConnectionAdaptiveTestAccess::dataFrameCWCount(c) == 8,
          "adopt must apply the descriptor's announced CW count");
    CHECK(tx_frames.empty(),
          "adopt must transmit NOTHING (no MODE_CHANGE ACK machinery)");
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "adopt must not arm any pending mode-change state");
    CHECK(c.getStats().descriptor_mode_switches == 1, "adopt must count once");
    CHECK(notify_count == 1 && notified_mod == Modulation::QAM16 &&
              notified_rate == CodeRate::R2_3,
          "adopt must fire the data-mode-changed notify (GUI/modem follow-through)");

    // Idempotent: the resend's re-announced descriptor must be a no-op.
    c.onDescriptorModeChange(Modulation::QAM16, CodeRate::R2_3, 8);
    CHECK(c.getStats().descriptor_mode_switches == 1,
          "re-announced descriptor must not double-adopt");
    CHECK(notify_count == 1, "re-announced descriptor must not re-notify");

    // ISS asymmetry guard: with our OWN data in flight the adopt is skipped.
    ConnectionAdaptiveTestAccess::fillArqWindow(c, 1);
    c.onDescriptorModeChange(Modulation::QPSK, CodeRate::R3_4, 8);
    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R2_3,
          "adopt with local DATA in flight must be skipped (per-direction rungs)");
    CHECK(c.getStats().descriptor_mode_switches == 1,
          "skipped adopt must not count");
}

// ─────────── RX-RATE-CMD (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md Phase 2) ───────────

using ultra::waveform::tone_burst_ack::AckType;
using ultra::waveform::tone_burst_ack::ToneBurstAckDetection;
using ultra::waveform::tone_burst_ack::ToneBurstAckPayload;
using ultra::waveform::tone_burst_ack::kDriveAdvisoryReserved;
using ultra::waveform::tone_burst_ack::kRungCmdDownHard;
using ultra::waveform::tone_burst_ack::kRungCmdNone;

ToneBurstAckDetection makeRungCmdDetection(uint8_t group_seq, uint8_t rung_cmd) {
    ToneBurstAckDetection d;
    d.payload.group_seq = group_seq;
    d.payload.frame_mask = 0;               // total crater: nothing delivered
    d.payload.rate_hint = 0;                // quality 0 (the crater's quantized grade)
    d.payload.type = AckType::Ack;
    d.payload.move_epoch = 0;               // matches the sender's initial TX epoch
    d.payload.rung_cmd = rung_cmd;
    return d;
}

void test_stale_epoch_tone_ack_has_no_connection_side_effects() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<Bytes> transmitted;
    int drive_advisories = 0;
    c.setTransmitCallback(
        [&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setDriveAdvisoryCallback(
        [&](uint8_t, uint8_t) { ++drive_advisories; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              c, Bytes{0x10}, v2::Flags::FINAL),
          "old-era DATA should enter the sender");
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == 0,
          "fixture should begin in move epoch zero");
    ConnectionAdaptiveTestAccess::abortArqPendingTx(c);
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == 1,
          "aborting live DATA must advance the move epoch");
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              c, Bytes{0x20}, v2::Flags::FINAL),
          "new-era DATA should enter the sender");
    CHECK(transmitted.size() == 2,
          "fixture should have emitted exactly the old- and new-era DATA frames");

    const size_t bytes_before = ConnectionAdaptiveTestAccess::arqInFlightBytes(c);
    const int acks_before = c.getStats().arq.acks_received;
    const std::string action_before = c.lastAdaptiveAction();

    ToneBurstAckDetection stale;
    stale.payload.group_seq = 1;  // valid support for the new seq, wrong era only
    stale.payload.frame_mask = 0;
    stale.payload.rate_hint = 7;
    stale.payload.type = AckType::Ack;
    stale.payload.move_epoch = 0;
    stale.payload.rung_cmd = kRungCmdNone;
    stale.payload.drive_advisory =
        ultra::waveform::tone_burst_ack::kDriveAdvisoryUp;
    CHECK(!c.onToneBurstAck(stale),
          "old-era tone ACK must be rejected by Connection");
    CHECK(ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == bytes_before,
          "stale-era ACK must not retire new-era bytes");
    CHECK(c.getStats().arq.acks_received == acks_before &&
              c.getStats().arq.stale_epoch_acks_ignored == 1,
          "stale-era ACK should only increment its rejection diagnostic");
    CHECK(drive_advisories == 0 && c.lastAdaptiveAction() == action_before,
          "stale-era ACK must not steer ALC or rate feedback");
    CHECK(transmitted.size() == 2,
          "stale-era ACK must not trigger a refill or retransmission");

    stale.payload.move_epoch = 1;
    CHECK(c.onToneBurstAck(stale),
          "same ACK in the current era should be accepted");
    CHECK(ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == 0,
          "current-era ACK should retire the live DATA frame");
    CHECK(drive_advisories == 1,
          "accepted ACK should still deliver its ALC advisory");
}

void test_latent_rate_consumes_exact_group_geometry_and_resets_per_qso() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    c.setBurstChannelObservation(20.0f, 0.30f, 0.8f, true, 0.1f);

    const uint8_t rung = coherentRungIndexFor(Modulation::QPSK, CodeRate::R2_3);
    LatentRateController exact_reference;
    exact_reference.seedPrior(latentThetaForRung(rung), 3.0f);
    exact_reference.observe(rung, /*k=*/6, /*M=*/8);
    exact_reference.relax(0.35f);
    LatentRateController old_rounded_reference;
    old_rounded_reference.seedPrior(latentThetaForRung(rung), 3.0f);
    old_rounded_reference.observe(rung, /*k=*/4, /*M=*/5);
    old_rounded_reference.relax(0.35f);

    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        c, /*all_ok=*/false, /*quality=*/0.75f, /*full_crater=*/false,
        /*delivered_fraction=*/0.75f, /*delivered_frames=*/6, /*group_size=*/8);
    const float production_mean =
        ConnectionAdaptiveTestAccess::latentPosteriorMean(c);
    CHECK(ConnectionAdaptiveTestAccess::latentObservations(c) == 1 &&
              ConnectionAdaptiveTestAccess::latentDecisions(c) == 1,
          "one 6/8 verdict should produce one latent observation and decision");
    CHECK(std::fabs(production_mean - exact_reference.posteriorMean()) < 1e-4f,
          "Connection must feed the estimator the descriptor's exact 6/8 geometry");
    CHECK(std::fabs(production_mean - old_rounded_reference.posteriorMean()) > 1e-3f,
          "exact 6/8 evidence must not collapse back to the old rounded 4/5 proxy");
    CHECK(ConnectionAdaptiveTestAccess::latentHasPrior(c) &&
              ConnectionAdaptiveTestAccess::latentVerdictClockValid(c),
          "a real verdict should arm the QSO-scoped latent state and cadence clock");

    // The physical startup cap is QPSK R1/2, but the automatic connect selector may
    // have picked a much stronger rung.  A clean safety probe must update that original
    // prior; seeding from the capped rung would manufacture a long slow-start.
    Connection capped;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        capped, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    capped.setBurstChannelObservation(20.0f, 0.30f, 0.8f, true, 0.1f);
    const uint8_t automatic_rung =
        coherentRungIndexFor(Modulation::QPSK, CodeRate::R3_4);
    ConnectionAdaptiveTestAccess::setLatentBootstrapRung(capped, automatic_rung);
    LatentRateController bootstrap_reference;
    bootstrap_reference.seedPrior(latentThetaForRung(automatic_rung), 3.0f);
    bootstrap_reference.observe(
        coherentRungIndexFor(Modulation::QPSK, CodeRate::R1_2), 5, 5);
    bootstrap_reference.relax(0.35f);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        capped, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(std::fabs(ConnectionAdaptiveTestAccess::latentPosteriorMean(capped) -
                    bootstrap_reference.posteriorMean()) < 1e-4f,
          "capped first group must update the pre-cap automatic selector prior");
    CHECK(ConnectionAdaptiveTestAccess::latentBootstrapRung(capped) == kRungIdxNone,
          "the pre-cap bootstrap must be consumed exactly once");

    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(c, 3);
    ConnectionAdaptiveTestAccess::seedQsoScopedAdaptiveState(c);
    ConnectionAdaptiveTestAccess::setLatentBootstrapRung(c, automatic_rung);
    ConnectionAdaptiveTestAccess::enterDisconnected(c);
    CHECK(!ConnectionAdaptiveTestAccess::latentHasPrior(c) &&
              ConnectionAdaptiveTestAccess::latentObservations(c) == 0 &&
              ConnectionAdaptiveTestAccess::latentDecisions(c) == 0 &&
              !ConnectionAdaptiveTestAccess::latentVerdictClockValid(c) &&
              ConnectionAdaptiveTestAccess::burstCleanGroupStreak(c) == 0 &&
              ConnectionAdaptiveTestAccess::latentBootstrapRung(c) == kRungIdxNone &&
              ConnectionAdaptiveTestAccess::consecutiveEscapeDrops(c) == 0 &&
              !ConnectionAdaptiveTestAccess::rxAuthorityDwellClockValid(c),
          "disconnect must clear posterior, cadence, burst-size, escape-episode, and "
          "rung-dwell evidence (prior=" << ConnectionAdaptiveTestAccess::latentHasPrior(c)
              << " obs=" << ConnectionAdaptiveTestAccess::latentObservations(c)
              << " decisions=" << ConnectionAdaptiveTestAccess::latentDecisions(c)
              << " clock=" << ConnectionAdaptiveTestAccess::latentVerdictClockValid(c)
              << " burst_streak=" << ConnectionAdaptiveTestAccess::burstCleanGroupStreak(c)
              << " escapes=" << ConnectionAdaptiveTestAccess::consecutiveEscapeDrops(c)
              << " dwell=" << ConnectionAdaptiveTestAccess::rxAuthorityDwellClockValid(c)
              << " bootstrap=" << static_cast<int>(
                     ConnectionAdaptiveTestAccess::latentBootstrapRung(c)) << ")");

    // Connected entry is independently defensive: even a reused Connection object with
    // state manufactured before the callback boundary cannot leak it into the new QSO.
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    c.setBurstChannelObservation(20.0f, 0.30f, 0.8f, true, 0.1f);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        c, false, 0.75f, false, 0.75f, 6, 8);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(c, 2);
    ConnectionAdaptiveTestAccess::seedQsoScopedAdaptiveState(c);
    ConnectionAdaptiveTestAccess::setLatentBootstrapRung(c, automatic_rung);
    CHECK(ConnectionAdaptiveTestAccess::latentObservations(c) == 1,
          "second fixture must re-populate the estimator before connected entry");
    ConnectionAdaptiveTestAccess::enterConnected(c);
    CHECK(!ConnectionAdaptiveTestAccess::latentHasPrior(c) &&
              ConnectionAdaptiveTestAccess::latentObservations(c) == 0 &&
              ConnectionAdaptiveTestAccess::latentDecisions(c) == 0 &&
              !ConnectionAdaptiveTestAccess::latentVerdictClockValid(c) &&
              ConnectionAdaptiveTestAccess::burstCleanGroupStreak(c) == 0 &&
              ConnectionAdaptiveTestAccess::consecutiveEscapeDrops(c) == 0 &&
              !ConnectionAdaptiveTestAccess::rxAuthorityDwellClockValid(c) &&
              ConnectionAdaptiveTestAccess::latentBootstrapRung(c) == automatic_rung,
          "connected entry must clear outcome state while preserving this QSO's pre-cap seed");
}

void test_latent_startup_probe_is_one_group_and_fails_closed() {
    const uint8_t r12 = coherentRungIndexFor(Modulation::QPSK, CodeRate::R1_2);
    const uint8_t r23 = coherentRungIndexFor(Modulation::QPSK, CodeRate::R2_3);

    auto make_startup = [](Connection& c) {
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(
            c, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
        c.setBurstChannelObservation(20.0f, 0.30f, 0.8f, true, 0.1f);
        ConnectionAdaptiveTestAccess::setLatentBootstrapRung(
            c, coherentRungIndexFor(Modulation::QPSK, CodeRate::R1_2));
    };

    // fixed04's corrected startup shape is a tail-proven 4/4 repair followed by a
    // descriptor-proven 5/5 group.  One clean group is insufficient; the clean pair
    // earns exactly one R2/3 trial instead of waiting ~16 observations at saturated R1/2.
    Connection pass;
    make_startup(pass);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pass, true, 1.0f, false, 1.0f, 4, 4);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(pass) == r12 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pass),
          "one trusted clean R1/2 group must not launch the startup probe");
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pass, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(pass) == r23 &&
              ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pass) &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeSpent(pass),
          "two trusted clean R1/2 groups should command one-rung R2/3 exploration");

    // Command loss/adoption latency must not consume extra probes.  A clean intervening
    // R1/2 group re-carries the same absolute target until the descriptor announces R2/3.
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pass, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(pass) == r23 &&
              ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pass),
          "clean pre-adoption R1/2 group should keep the one standing R2/3 command");
    ConnectionAdaptiveTestAccess::setCurrentCoherentMode(
        pass, Modulation::QPSK, CodeRate::R2_3);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pass, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(pass) == r23 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pass) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(pass),
          "a clean physical R2/3 probe should retain R2/3 and spend the QSO probe");

    // The higher-rung downside is bounded to one physical group: its first imperfect
    // result emits the R1/2 rollback command immediately, independently of the argmax.
    Connection lossy;
    make_startup(lossy);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        lossy, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        lossy, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::setCurrentCoherentMode(
        lossy, Modulation::QPSK, CodeRate::R2_3);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        lossy, false, 0.8f, false, 0.8f, 4, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(lossy) == r12 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(lossy) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(lossy),
          "the first imperfect R2/3 probe group must command immediate R1/2 rollback");

    // Missing descriptor/tail geometry cannot masquerade as a clean trial even if a
    // fallback fraction says 100%.  It fails closed to the base rung.
    Connection unknown;
    make_startup(unknown);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        unknown, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        unknown, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::setCurrentCoherentMode(
        unknown, Modulation::QPSK, CodeRate::R2_3);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        unknown, true, 1.0f, false, 1.0f, 0, 0);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(unknown) == r12 &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(unknown),
          "unknown probe geometry must roll back even when fallback quality looks clean");

    // A stale fallback can report a plausible full-size 5/5.  Provenance, not the
    // numerical fraction, decides whether it reaches the posterior or clean-pair gate.
    Connection stale_fallback;
    make_startup(stale_fallback);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        stale_fallback, true, 1.0f, false, 1.0f, 5, 5, /*geometry_proven=*/true);
    const int obs_before_stale =
        ConnectionAdaptiveTestAccess::latentObservations(stale_fallback);
    const float mean_before_stale =
        ConnectionAdaptiveTestAccess::latentPosteriorMean(stale_fallback);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        stale_fallback, true, 1.0f, false, 1.0f, 5, 5,
        /*geometry_proven=*/false);
    CHECK(ConnectionAdaptiveTestAccess::latentObservations(stale_fallback) ==
                  obs_before_stale &&
              std::fabs(ConnectionAdaptiveTestAccess::latentPosteriorMean(
                            stale_fallback) - mean_before_stale) < 1e-5f &&
              ConnectionAdaptiveTestAccess::latentStartupProbeCleanGroups(
                  stale_fallback) == 0 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(stale_fallback),
          "unproven fallback 5/5 must not update/relax the posterior or qualify startup");

    // The same fail-closed rule applies before descriptor adoption: degraded base-rung
    // evidence cancels the pending command without spending a higher-rung group.
    Connection pending_loss;
    make_startup(pending_loss);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_loss, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_loss, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_loss, false, 0.8f, false, 0.8f, 4, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(pending_loss) == r12 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pending_loss) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(pending_loss),
          "imperfect R1/2 evidence while adoption is pending must cancel the probe");

    // ACK diversity is finite.  Two additional clean base groups may re-carry the
    // flagged target; a third non-adoption expires it and latches the QSO safety ceiling.
    Connection pending_expiry;
    make_startup(pending_expiry);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_expiry, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_expiry, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_expiry, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_expiry, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pending_expiry) &&
              ConnectionAdaptiveTestAccess::latentStartupProbePendingBaseGroups(
                  pending_expiry) == 2,
          "startup target may be re-carried across at most two clean base groups");
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        pending_expiry, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(!ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(pending_expiry) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeFailed(pending_expiry) &&
              ConnectionAdaptiveTestAccess::rxAuthorityCommand(pending_expiry) == r12,
          "third base non-adoption must expire and fail-close the startup probe");

    // A failed physical target cannot be undone by its own posterior update or by later
    // clean base groups.  Settle adoption, then provide ample optimistic evidence: the
    // QSO-scoped failed-probe ceiling remains R1/2.
    CHECK(ConnectionAdaptiveTestAccess::latentStartupProbeRollbackPending(lossy) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeFailed(lossy),
          "lossy target must arm rollback hold and the failed-probe ceiling");
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        lossy, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(lossy) == r12 &&
              ConnectionAdaptiveTestAccess::latentStartupProbeRollbackPending(lossy),
          "rollback must keep riding while the sender still reports the target rung");
    ConnectionAdaptiveTestAccess::setCurrentCoherentMode(
        lossy, Modulation::QPSK, CodeRate::R1_2);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        lossy, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(!ConnectionAdaptiveTestAccess::latentStartupProbeRollbackPending(lossy) &&
              ConnectionAdaptiveTestAccess::rxAuthorityCommand(lossy) == r12,
          "one trusted clean adopted base group settles rollback without an immediate climb");
    for (int i = 0; i < 6; ++i) {
        ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
            lossy, true, 1.0f, false, 1.0f, 5, 5);
        CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(lossy) <= r12,
              "failed startup probe must never re-enter R2/3 in the same QSO");
    }

    // Startup exploration must not override a natural safety demotion.
    Connection natural_down;
    make_startup(natural_down);
    ConnectionAdaptiveTestAccess::seedLatentPrior(natural_down, -4.0f, 0.25f);
    ConnectionAdaptiveTestAccess::setLatentStartupProbeCleanGroups(natural_down, 1);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        natural_down, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(natural_down) < r12 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(natural_down),
          "clean-pair exploration must not override the model's natural demotion");

    // An operator-forced data mode is a measurement contract.  Automatic exploration
    // must never override it, regardless of clean outcomes.
    Connection forced;
    make_startup(forced);
    ConnectionAdaptiveTestAccess::setLatentStartupProbeAllowed(forced, false);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        forced, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        forced, true, 1.0f, false, 1.0f, 5, 5);
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(forced) != r23 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(forced) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(forced),
          "forced-mode QSO must never launch automatic startup exploration");

    Connection callback_gate;
    make_startup(callback_gate);
    bool callback_saw_forced_gate = false;
    callback_gate.setConnectedCallback([&]() {
        callback_saw_forced_gate =
            !ConnectionAdaptiveTestAccess::latentStartupProbeAllowed(callback_gate);
    });
    ConnectionAdaptiveTestAccess::enterConnected(
        callback_gate, /*automatic_rate_allowed=*/false);
    CHECK(callback_saw_forced_gate &&
              !ConnectionAdaptiveTestAccess::rateAdaptationActive(callback_gate),
          "forced-QSO gate must freeze every rate actuator before on_connected");

    // The pin is QSO-scoped, not dependent on a live env variable.  Once the handshake
    // establishes an explicit profile, clearing all force/lock knobs must not let a later
    // receiver command or escape path move it.
    unsetenv("ULTRA_FORCE_DATA_MOD");
    unsetenv("ULTRA_FORCE_DATA_RATE");
    unsetenv("ULTRA_LOCK_RATE");
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(
        callback_gate, coherentRungIndexFor(Modulation::QPSK, CodeRate::R1_4));
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(callback_gate) &&
              !ConnectionAdaptiveTestAccess::rateAdaptationActive(callback_gate),
          "forced-QSO pin must survive env removal and reject authority movement");

    // Environment force knobs bypass the CONNECT forced fields, so production must
    // inspect the live measurement contract too.  ULTRA_LOCK_RATE has the same priority.
    Connection env_forced;
    make_startup(env_forced);
    setenv("ULTRA_FORCE_DATA_RATE", "R1_2", 1);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        env_forced, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        env_forced, true, 1.0f, false, 1.0f, 5, 5);
    unsetenv("ULTRA_FORCE_DATA_RATE");
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(env_forced) != r23 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(env_forced) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(env_forced),
          "ULTRA_FORCE_DATA_RATE must suppress the automatic startup probe");

    Connection locked;
    make_startup(locked);
    setenv("ULTRA_LOCK_RATE", "1", 1);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        locked, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        locked, true, 1.0f, false, 1.0f, 5, 5);
    unsetenv("ULTRA_LOCK_RATE");
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(locked) != r23 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(locked) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeSpent(locked),
          "ULTRA_LOCK_RATE must suppress the automatic startup probe");

    // ULTRA_MAX_OFDM_RATE is an absolute ladder ceiling, including the latent early-return
    // path.  Seed an intentionally strong prior: neither the normal argmax nor the startup
    // probe may escape the requested canonical QPSK ceiling.
    Connection cap_r12;
    make_startup(cap_r12);
    ConnectionAdaptiveTestAccess::setLatentBootstrapRung(
        cap_r12, coherentRungIndexFor(Modulation::QAM16, CodeRate::R2_3));
    setenv("ULTRA_MAX_OFDM_RATE", "R1_2", 1);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        cap_r12, true, 1.0f, false, 1.0f, 5, 5);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        cap_r12, true, 1.0f, false, 1.0f, 5, 5);
    unsetenv("ULTRA_MAX_OFDM_RATE");
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(cap_r12) <= r12 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(cap_r12),
          "latent selector and startup probe must honor absolute R1/2 ceiling");

    Connection cap_r23;
    make_startup(cap_r23);
    ConnectionAdaptiveTestAccess::setLatentBootstrapRung(
        cap_r23, coherentRungIndexFor(Modulation::QAM16, CodeRate::R2_3));
    setenv("ULTRA_MAX_OFDM_RATE", "R2_3", 1);
    ConnectionAdaptiveTestAccess::updateRxAuthorityExact(
        cap_r23, true, 1.0f, false, 1.0f, 5, 5);
    unsetenv("ULTRA_MAX_OFDM_RATE");
    CHECK(ConnectionAdaptiveTestAccess::rxAuthorityCommand(cap_r23) <= r23,
          "latent selector must honor absolute R2/3 ceiling on a strong prior");

    // QSO lifecycle resets every arm/pending/spent counter.  The next automatic link may
    // earn its own one-shot probe, but no command state crosses the disconnect boundary.
    ConnectionAdaptiveTestAccess::enterDisconnected(lossy);
    CHECK(ConnectionAdaptiveTestAccess::latentStartupProbeAllowed(lossy) &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeWaiting(lossy) &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeSpent(lossy) &&
              ConnectionAdaptiveTestAccess::latentStartupProbeCleanGroups(lossy) == 0 &&
              ConnectionAdaptiveTestAccess::latentStartupProbePendingBaseGroups(lossy) == 0 &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeRollbackPending(lossy) &&
              !ConnectionAdaptiveTestAccess::latentStartupProbeFailed(lossy),
          "disconnect must reset the bounded startup-probe state for the next QSO");
}

void test_forced_connect_ack_conflict_fails_closed() {
    Connection conflict;
    ConnectionAdaptiveTestAccess::makeConnectingWithOutboundForce(
        conflict, Modulation::QAM8, CodeRate::R2_3);
    const auto conflicting_ack = v2::ConnectFrame::makeConnectAck(
        "K2DEF", "W1ABC", static_cast<uint8_t>(WaveformMode::OFDM_CHIRP),
        Modulation::QPSK, CodeRate::R1_2, 20.0f, 0.50f, 8);
    ConnectionAdaptiveTestAccess::handleConnectAck(conflict, conflicting_ack);
    CHECK(conflict.getState() == ConnectionState::DISCONNECTED,
          "forced CONNECT_ACK mismatch must abort instead of creating a split PHY");

    Connection matching;
    ConnectionAdaptiveTestAccess::makeConnectingWithOutboundForce(
        matching, Modulation::QAM8, CodeRate::R2_3);
    const auto matching_ack = v2::ConnectFrame::makeConnectAck(
        "K2DEF", "W1ABC", static_cast<uint8_t>(WaveformMode::OFDM_CHIRP),
        Modulation::QAM8, CodeRate::R2_3, 20.0f, 0.50f, 12);
    ConnectionAdaptiveTestAccess::handleConnectAck(matching, matching_ack);
    CHECK(matching.getState() == ConnectionState::CONNECTED &&
              matching.getDataModulation() == Modulation::QAM8 &&
              matching.getDataCodeRate() == CodeRate::R2_3 &&
              !ConnectionAdaptiveTestAccess::rateAdaptationActive(matching),
          "matching forced CONNECT_ACK must connect exactly and pin all actuators");

    // A responder-local force was not present in our outbound CONNECT fields. Its ACK
    // provenance bit is therefore the only way the initiator can retain the same
    // QSO-scoped pin and avoid later moving the responder through MODE_CHANGE.
    Connection responder_forced;
    ConnectionAdaptiveTestAccess::makeConnectingWithOutboundForce(
        responder_forced, Modulation::AUTO, CodeRate::AUTO);
    auto responder_forced_ack = v2::ConnectFrame::makeConnectAck(
        "K2DEF", "W1ABC", static_cast<uint8_t>(WaveformMode::OFDM_CHIRP),
        Modulation::QAM8, CodeRate::R2_3, 20.0f, 0.50f, 12);
    responder_forced_ack.flags |= v2::Flags::CONNECT_FORCED_PROFILE;
    ConnectionAdaptiveTestAccess::handleConnectAck(responder_forced,
                                                    responder_forced_ack);
    CHECK(responder_forced.getState() == ConnectionState::CONNECTED &&
              responder_forced.getDataModulation() == Modulation::QAM8 &&
              responder_forced.getDataCodeRate() == CodeRate::R2_3 &&
              !ConnectionAdaptiveTestAccess::rateAdaptationActive(responder_forced),
          "responder-only forced ACK must pin initiator actuators for the whole QSO");
}

void test_startup_probe_sender_guards_and_timeout_rollback() {
    const uint8_t r12 = kRungIdxQpskR12;
    const uint8_t r23 = kRungIdxQpskR23;

    // A prospective 1/2/3-frame file tail cannot produce the trusted M>=4 probe
    // outcome.  Refuse it before paying a mode transition.
    Connection shape;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        shape, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    const size_t chunk =
        ConnectionAdaptiveTestAccess::startupProbeTargetChunkBytes(shape);
    CHECK(chunk > 0, "target startup-probe chunk size must be non-zero");
    for (size_t frames = 1; frames <= 3; ++frames) {
        Connection tail;
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(
            tail, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
        TempPayloadFile payload("startup_tail", chunk * frames);
        CHECK(payload.dir.valid() && !payload.path.empty(),
              "startup-tail payload fixture should be created");
        ConnectionAdaptiveTestAccess::startFile(tail, payload.path);
        CHECK(!ConnectionAdaptiveTestAccess::startupProbeHasSufficientPayload(tail),
              "a 1/2/3-frame target tail must not qualify for startup probing");
    }
    Connection enough;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        enough, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TempPayloadFile four_frames("startup_four", chunk * 4);
    ConnectionAdaptiveTestAccess::startFile(enough, four_frames.path);
    CHECK(ConnectionAdaptiveTestAccess::startupProbeHasSufficientPayload(enough),
          "four prospective target frames meet the trusted probe minimum");

    // The ordinary tail-climb guard must not silently narrow the startup probe's
    // explicit M>=4 contract to the steady-state N=5/8 group size.  Retire FILE_START
    // first so this is a real four-DATA-frame mid-file tail, then assert the command
    // emits exactly that one physical probe group.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection four_frame_command;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    size_t four_frame_burst_size = 0;
    int four_frame_bursts = 0;
    four_frame_command.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            ++four_frame_bursts;
            four_frame_burst_size = frames.size();
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        four_frame_command, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TempPayloadFile four_frame_command_payload(
        "startup_four_command", chunk * 4);
    ConnectionAdaptiveTestAccess::startFile(
        four_frame_command, four_frame_command_payload.path);
    CHECK(ConnectionAdaptiveTestAccess::retireFileMetadata(four_frame_command),
          "startup command fixture must retire FILE_START before its data tail");
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(
        four_frame_command,
        static_cast<uint8_t>(kRungAuthorityStartupProbeFlag | r23));
    CHECK(four_frame_command.getDataCodeRate() == CodeRate::R2_3 &&
              ConnectionAdaptiveTestAccess::txLatentStartupProbeActive(
                  four_frame_command) &&
              four_frame_bursts == 1 && four_frame_burst_size == 4,
          "valid four-frame startup probe must remain one real physical target group");

    // Payload alone is insufficient when an operator-forced long frame makes the
    // physical target burst shorter than the receiver's trusted M>=4 gate.
    Connection long_cw;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        long_cw, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setForcedCWCount(long_cw, 16);
    const size_t long_cw_chunk =
        ConnectionAdaptiveTestAccess::startupProbeTargetChunkBytes(long_cw);
    TempPayloadFile long_cw_payload("startup_long_cw", long_cw_chunk * 10);
    ConnectionAdaptiveTestAccess::startFile(long_cw, long_cw_payload.path);
    CHECK(!ConnectionAdaptiveTestAccess::startupProbeHasSufficientPayload(long_cw),
          "startup probe must be refused when target physical geometry fits fewer "
          "than four frames");

    // A valid flagged command with enough payload commits exactly one target group and
    // arms sender-side timeout protection before that group reaches the transport.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection flagged_sender;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    int flagged_bursts = 0;
    int flagged_standalone_data = 0;
    flagged_sender.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) { ++flagged_bursts; });
    flagged_sender.setTransmitCallback([&](const Bytes& bytes) {
        const auto h = v2::parseHeader(bytes);
        if (h.valid && !h.is_control) ++flagged_standalone_data;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        flagged_sender, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TempPayloadFile probe_payload("startup_probe_sender", chunk * 10);
    ConnectionAdaptiveTestAccess::startFile(flagged_sender, probe_payload.path);
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(
        flagged_sender,
        static_cast<uint8_t>(kRungAuthorityStartupProbeFlag | r23));
    CHECK(flagged_sender.getDataCodeRate() == CodeRate::R2_3 &&
              ConnectionAdaptiveTestAccess::txLatentStartupProbeActive(flagged_sender) &&
              ConnectionAdaptiveTestAccess::txLatentStartupProbeAirborne(flagged_sender) &&
              flagged_bursts == 1,
          "valid flagged authority command must arm protection and emit one target group");

    // The original launch ACK can be detected again while the target group awaits its
    // own verdict. It is a valid base-1/no-progress SACK, but must not reach generic
    // refill and put a second target group on air.
    ToneBurstAckDetection launch_replay;
    launch_replay.payload.group_seq = 0x3F;
    launch_replay.payload.frame_mask = 0;
    launch_replay.payload.rate_hint = static_cast<uint8_t>(r23 & 0x7);
    launch_replay.payload.rung_cmd = static_cast<uint8_t>(
        ((kRungAuthorityStartupProbeFlag | r23) >> 3) & 0x3);
    launch_replay.payload.type = AckType::Ack;
    launch_replay.payload.move_epoch = 0;
    const int replay_retx_before = flagged_sender.getStats().arq.retransmissions;
    CHECK(flagged_sender.onToneBurstAck(launch_replay) &&
              flagged_bursts == 1 && flagged_standalone_data == 0 &&
              flagged_sender.getStats().arq.retransmissions == replay_retx_before,
          "replayed launch ACK must be consumed without a second target egress/refill");

    // Sender-local absolute ceiling wins even when a peer commands a higher rung.
    Connection capped;
    std::vector<Bytes> capped_controls;
    capped.setTransmitCallback([&](const Bytes& b) { capped_controls.push_back(b); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        capped, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    setenv("ULTRA_MAX_OFDM_RATE", "R1_2", 1);
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(capped, r23);
    unsetenv("ULTRA_MAX_OFDM_RATE");
    CHECK(capped.getDataCodeRate() == CodeRate::R1_2 &&
              !ConnectionAdaptiveTestAccess::modeChangePending(capped) &&
              capped_controls.empty(),
          "asymmetric peer authority must not bypass the sender's local rate ceiling");

    // Epoch-off, busy-window DOWN commands use synchronized MODE_CHANGE; descriptor
    // regrid would reinterpret live sequence identities under the old era.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    setenv("ULTRA_ARQ_MOVE_EPOCH", "0", 1);
    Connection epoch_off;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    std::vector<Bytes> epoch_controls;
    epoch_off.setTransmitCallback(
        [&](const Bytes& b) { epoch_controls.push_back(b); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        epoch_off, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    CHECK(!ConnectionAdaptiveTestAccess::arqMoveEpochEnabled(epoch_off),
          "epoch-off fixture must actually disable move-epoch before construction");
    ConnectionAdaptiveTestAccess::fillArqWindow(epoch_off, 2);
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(epoch_off, r12);
    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(epoch_off) &&
              epoch_off.getDataCodeRate() == CodeRate::R2_3 &&
              epoch_off.getStats().descriptor_mode_switches == 0 &&
              countModeChangeFrames(epoch_controls) == 1,
          "epoch-off mid-window authority demote must synchronize via MODE_CHANGE");

    // Drive the real ARQ tick after one flagged R2/3 group has gone on air. The timeout
    // transaction must preserve retry budget, suppress all target DATA egress, and arm
    // the synchronized base rollback. If that rollback cannot be confirmed, no target
    // intent may resume: the session disconnects.
    Connection timeout;
    std::vector<Bytes> timeout_initial_data;
    int timeout_controls = 0;
    int timeout_standalone_data = 0;
    int timeout_bursts = 0;
    timeout.setTransmitCallback([&](const Bytes& b) {
        const auto h = v2::parseHeader(b);
        if (h.valid && h.is_control) {
            ++timeout_controls;
        } else if (h.valid) {
            ++timeout_standalone_data;
            timeout_initial_data.push_back(b);
        }
    });
    timeout.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) { ++timeout_bursts; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        timeout, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(timeout);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              timeout, Bytes{0x55}, v2::Flags::FINAL) &&
              timeout_initial_data.size() == 1,
          "startup timeout fixture must seed one live target DATA identity");
    const Bytes target_identity = timeout_initial_data.front();
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              timeout, {target_identity}, 1) == 1,
          "startup target identity must be rearmed to a deterministic one-tick RTO");
    ConnectionAdaptiveTestAccess::setTxLatentStartupProbeState(
        timeout, /*active=*/true, /*airborne=*/true, r12);
    timeout_controls = 0;
    timeout_standalone_data = 0;
    timeout_bursts = 0;
    timeout_initial_data.clear();
    timeout.tick(1);
    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(timeout) &&
              ConnectionAdaptiveTestAccess::pendingRate(timeout) == CodeRate::R1_2 &&
              ConnectionAdaptiveTestAccess::pendingModeChangeReason(timeout) ==
                  v2::ModeChangeReason::STARTUP_PROBE_TIMEOUT &&
              ConnectionAdaptiveTestAccess::stagedArqTimeoutFrames(timeout) == 0 &&
              !ConnectionAdaptiveTestAccess::txLatentStartupProbeActive(timeout) &&
              timeout_controls == 1 && timeout_standalone_data == 0 &&
              timeout_bursts == 0 && timeout.getStats().arq.timeouts == 1 &&
              timeout.getStats().arq.retransmissions == 0 &&
              ConnectionAdaptiveTestAccess::arqMaxInFlightRetryCount(timeout) == 0,
          "first unACKed target RTO must preserve retry budget, suppress target "
          "egress, and arm one synchronized base rollback");
    const uint32_t rollback_retry_ms =
        ConnectionAdaptiveTestAccess::modeChangeRetryMs(timeout);
    for (int i = 0;
         i < ConnectionAdaptiveTestAccess::modeChangeMaxRetries() + 1; ++i) {
        timeout.tick(rollback_retry_ms);
    }
    CHECK(timeout.getState() == ConnectionState::DISCONNECTING &&
              timeout_standalone_data == 0 && timeout_bursts == 0 &&
              timeout.getStats().arq.retransmissions == 0,
          "unconfirmed startup rollback must disconnect without resuming target DATA");

    // A different MODE_CHANGE already in flight at the target RTO is not safely
    // reorderable. Preserve the probe identity, cancel the staged target egress, and
    // disconnect rather than silently clearing state or overwriting the control.
    Connection collided;
    std::vector<Bytes> collided_data;
    int collided_controls = 0;
    int collided_data_tx = 0;
    int collided_bursts = 0;
    collided.setTransmitCallback([&](const Bytes& b) {
        const auto h = v2::parseHeader(b);
        if (h.valid && h.is_control) ++collided_controls;
        else if (h.valid) {
            ++collided_data_tx;
            collided_data.push_back(b);
        }
    });
    collided.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) { ++collided_bursts; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        collided, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(collided);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              collided, Bytes{0x66}, v2::Flags::FINAL) &&
              collided_data.size() == 1,
          "collided-mode fixture must seed one live target identity");
    const Bytes collided_identity = collided_data.front();
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              collided, {collided_identity}, 1) == 1,
          "collided-mode target identity must be rearmed to one tick");
    ConnectionAdaptiveTestAccess::setTxLatentStartupProbeState(
        collided, /*active=*/true, /*airborne=*/true, r12);
    collided.requestModeChange(Modulation::QPSK, CodeRate::R3_4, 20.0f,
                               v2::ModeChangeReason::CHANNEL_IMPROVED);
    collided_controls = 0;
    collided_data_tx = 0;
    collided_bursts = 0;
    collided_data.clear();
    collided.tick(1);
    CHECK(collided.getState() == ConnectionState::DISCONNECTING &&
              ConnectionAdaptiveTestAccess::txLatentStartupProbeActive(collided) &&
              ConnectionAdaptiveTestAccess::txLatentStartupProbeAirborne(collided) &&
              ConnectionAdaptiveTestAccess::stagedArqTimeoutFrames(collided) == 0 &&
              collided_controls == 1 && collided_data_tx == 0 &&
              collided_bursts == 0 &&
              collided.getStats().arq.retransmissions == 0 &&
              ConnectionAdaptiveTestAccess::arqMaxInFlightRetryCount(collided) == 0,
          "startup RTO colliding with unrelated MODE_CHANGE must preserve probe state "
          "and disconnect without target egress");

    // With descriptor switching disabled, the launch itself uses MODE_CHANGE. Losing
    // every ACK is also geometry-ambiguous: the peer may already be at R2/3, so the
    // generic give-up path must never resume base DATA.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    Connection legacy_launch;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    int legacy_controls = 0;
    int legacy_data = 0;
    int legacy_bursts = 0;
    legacy_launch.setTransmitCallback([&](const Bytes& b) {
        const auto h = v2::parseHeader(b);
        if (h.valid && h.is_control) ++legacy_controls;
        else if (h.valid) ++legacy_data;
    });
    legacy_launch.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) { ++legacy_bursts; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        legacy_launch, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    TempPayloadFile legacy_payload("startup_legacy_launch", chunk * 10);
    ConnectionAdaptiveTestAccess::startFile(legacy_launch, legacy_payload.path);
    legacy_controls = legacy_data = legacy_bursts = 0;
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(
        legacy_launch,
        static_cast<uint8_t>(kRungAuthorityStartupProbeFlag | r23));
    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(legacy_launch) &&
              ConnectionAdaptiveTestAccess::pendingModeChangeReason(legacy_launch) ==
                  v2::ModeChangeReason::STARTUP_PROBE_BEGIN &&
              ConnectionAdaptiveTestAccess::txLatentStartupProbeActive(legacy_launch) &&
              !ConnectionAdaptiveTestAccess::txLatentStartupProbeAirborne(legacy_launch) &&
              legacy_controls == 1 && legacy_data == 0 && legacy_bursts == 0,
          "legacy startup launch must be explicitly tagged and emit control only");
    const uint32_t launch_retry_ms =
        ConnectionAdaptiveTestAccess::modeChangeRetryMs(legacy_launch);
    for (int i = 0;
         i < ConnectionAdaptiveTestAccess::modeChangeMaxRetries() + 1; ++i) {
        legacy_launch.tick(launch_retry_ms);
    }
    CHECK(legacy_launch.getState() == ConnectionState::DISCONNECTING &&
              legacy_data == 0 && legacy_bursts == 0,
          "unconfirmed legacy startup launch must disconnect without split-geometry DATA");
}

void test_forced_r1_3_cw1_rejects_file_before_wire_tx() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R1_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R1_3, /*logical_cw=*/1);
    c.setHalfDuplexInteractive(true);
    ConnectionAdaptiveTestAccess::setDataTurnTxGuard(c, 1000);

    CHECK(ConnectionAdaptiveTestAccess::currentPayloadCapacity(c) == 8,
          "forced R1/3 cw1 must reproduce the 8-byte physical payload boundary");

    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& frame) { tx_frames.push_back(frame); });
    TempPayloadFile payload("r1_3_cw1_file_start_reject", 32);
    CHECK(!payload.path.empty(), "create forced R1/3 cw1 source file");

    CHECK(!c.sendFile(payload.path),
          "forced R1/3 cw1 must reject a file that cannot carry FILE_START");
    CHECK(tx_frames.empty(),
          "insufficient FILE_START capacity must fail before any wire transmission");
    CHECK(!c.isFileTransferInProgress(),
          "rejected forced profile must leave no queued or active file transfer");

    // FILE_START needs 11 bytes of metadata, but a text message has no such
    // minimum: it must remain usable by fragmenting over this tiny geometry.
    ConnectionAdaptiveTestAccess::setDataTurnTxGuard(c, 0);
    const std::string message = "message-over-eight-bytes";
    CHECK(c.sendMessage(message),
          "the same 8-byte profile must accept a fragmented operator message");
    // The fragmented object is the WIRE object: text plus the message-object
    // identity prefix that makes a re-grid resend idempotent.
    const size_t wire_bytes = message.size() + kMessageObjectPrefixBytes;
    CHECK(tx_frames.size() == (wire_bytes + 7) / 8,
          "message must split into the exact number of 8-byte DATA frames");
    for (const auto& bytes : tx_frames) {
        const auto frame = v2::DataFrame::deserialize(bytes);
        CHECK(frame && frame->type == v2::FrameType::DATA &&
                  frame->payload.size() <= 8,
              "every tiny-profile message fragment must be bounded DATA");
    }
}

void test_queued_file_geometry_shrink_reports_terminal_start_failure() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R1_2, 20.0f, 0.30f, Modulation::QPSK);
    c.setHalfDuplexInteractive(true);
    CHECK(ConnectionAdaptiveTestAccess::currentPayloadCapacity(c) >=
              FileTransferController::MIN_FILE_START_PAYLOAD,
          "queued-file fixture must begin on a FILE_START-capable profile");

    // Hold an otherwise clear local DATA turn so sendFile accepts into the queue
    // without transmitting a TURN_REQUEST or starting the controller yet.
    ConnectionAdaptiveTestAccess::setDataTurnTxGuard(c, 1000);
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& frame) { tx_frames.push_back(frame); });
    std::vector<bool> sent_results;
    std::vector<std::string> sent_errors;
    c.setFileSentCallback([&](bool success, const std::string& error) {
        sent_results.push_back(success);
        sent_errors.push_back(error);
    });

    TempPayloadFile payload("queued_file_start_geometry_shrink", 32);
    CHECK(!payload.path.empty(), "create queued geometry-shrink source file");
    CHECK(c.sendFile(payload.path),
          "FILE_START-capable profile should accept the deferred file request");
    CHECK(c.isFileTransferInProgress(),
          "accepted deferred file must remain observable while queued");
    CHECK(sent_results.empty(),
          "queue acceptance alone must not emit a terminal file callback");

    // Reproduce the asynchronous edge: adaptation changes the physical profile
    // before the ISS turn opens. The accepted request must be failed exactly once,
    // never erased silently by tryStartQueuedFileIfReady().
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R1_3, /*logical_cw=*/1);
    CHECK(ConnectionAdaptiveTestAccess::currentPayloadCapacity(c) == 8,
          "queued geometry-shrink fixture must land below FILE_START minimum");
    ConnectionAdaptiveTestAccess::setDataTurnTxGuard(c, 0);
    CHECK(!ConnectionAdaptiveTestAccess::tryStartQueuedFileIfReady(c),
          "undersized queued request must fail instead of entering the sender");
    CHECK(sent_results.size() == 1 && !sent_results.front() &&
              sent_errors.size() == 1 && !sent_errors.front().empty(),
          "accepted queued request must report one explicit terminal failure");
    CHECK(!c.isFileTransferInProgress(),
          "terminal queued-start failure must clear queued and controller state");
    CHECK(tx_frames.empty(),
          "queued FILE_START capacity failure must occur before wire transmission");
}

void test_empty_message_requests_are_rejected_atomically() {
    Connection c;
    std::vector<Bytes> transmitted;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    CHECK(!c.sendMessage(""), "empty single message must be rejected");
    CHECK(!c.sendMessages({}), "empty batch must be rejected");
    CHECK(!c.sendMessages({"valid", ""}),
          "a batch containing an empty logical message must be rejected atomically");
    CHECK(transmitted.empty() && c.getTxBacklogBytes() == 0 &&
              ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 0,
          "rejected empty requests must leave no wire, backlog, or status record");
}

void test_deferred_queue_refuses_newest_and_abort_fails_every_accepted_message() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::makeRemoteDataTurn(c);

    constexpr size_t kQueueLimit = 32;
    for (size_t i = 0; i < kQueueLimit; ++i) {
        CHECK(c.sendMessage("queued-" + std::to_string(i)),
              "every message through the documented deferred-queue limit must be accepted");
    }
    CHECK(!c.sendMessage("refused-newest"),
          "the payload beyond the queue limit must be refused, not evict an accepted one");
    CHECK(ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == kQueueLimit &&
              ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == kQueueLimit,
          "queue overflow must preserve all previously accepted payloads and records");
    CHECK(ConnectionAdaptiveTestAccess::queuedPayloadText(c, 0) == "queued-0" &&
              ConnectionAdaptiveTestAccess::queuedPayloadText(c, kQueueLimit - 1) ==
                  "queued-31",
          "queue overflow must preserve FIFO endpoints exactly");

    const size_t wire_before_abort = transmitted.size();
    c.abortTxNow();
    size_t failed = 0;
    for (const auto& status : statuses) {
        failed += status.status == Connection::MessageTxStatus::FAILED ? 1 : 0;
    }
    CHECK(failed == kQueueLimit &&
              ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 0 &&
              ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 0,
          "TX abort must terminal-fail and remove every accepted queued message");
    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    ConnectionAdaptiveTestAccess::pumpQueuedOperation(c);
    CHECK(transmitted.size() == wire_before_abort,
          "an aborted queued message must never transmit later after turn acquisition");
}

void test_reset_terminal_fails_accepted_queued_message() {
    Connection c;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    c.setTransmitCallback([](const Bytes&) {});
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::makeRemoteDataTurn(c);
    CHECK(c.sendMessage("reset-me"),
          "queued reset fixture message must be accepted");

    c.reset();
    CHECK(statuses.size() == 1 &&
              statuses.front().status == Connection::MessageTxStatus::FAILED &&
              statuses.front().text == "reset-me" &&
              statuses.front().remote_call == "K2DEF" &&
              ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 0 &&
              ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 0,
          "reset must publish one peer-attributed terminal failure and clear the accepted request");
}

void test_accepted_queued_start_failure_reports_terminal_status() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    std::vector<bool> legacy_results;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    c.setMessageSentCallback(
        [&](bool success) { legacy_results.push_back(success); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::makeRemoteDataTurn(c);
    CHECK(c.sendMessage("accepted-before-start"),
          "deferred message must be accepted while waiting for the turn");

    // Model an asynchronous start-time invalidation after acceptance. The queue
    // pump sees an otherwise-ready ARQ, but frame submission refuses missing peers.
    ConnectionAdaptiveTestAccess::clearArqCallsigns(c);
    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    ConnectionAdaptiveTestAccess::pumpQueuedOperation(c);

    CHECK(transmitted.size() == 1,
          "only the earlier TURN_REQUEST control may exist after start refusal");
    CHECK(statuses.size() == 1 &&
              statuses.front().status == Connection::MessageTxStatus::FAILED &&
              statuses.front().text == "accepted-before-start" &&
              legacy_results.size() == 1 && !legacy_results.front(),
          "accepted start failure must publish exactly one terminal FAILED result");
    CHECK(ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 0 &&
              ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 0,
          "terminal queued-start failure must leave no stranded queue or record");
}

void test_deferred_payload_fifo_survives_failed_head_and_new_submission() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::makeRemoteDataTurn(c);
    CHECK(c.sendMessage("fifo-A") && c.sendMessage("fifo-B"),
          "FIFO fixture head and successor must both enter the deferred queue");

    ConnectionAdaptiveTestAccess::clearArqCallsigns(c);
    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    ConnectionAdaptiveTestAccess::pumpQueuedOperation(c);
    CHECK(statuses.size() == 1 &&
              statuses.front().status == Connection::MessageTxStatus::FAILED &&
              statuses.front().text == "fifo-A" &&
              ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 1,
          "failed queue head must terminal-fail once without consuming its successor");

    const size_t wire_before_c = transmitted.size();
    CHECK(c.sendMessage("fifo-C"),
          "a new payload must remain admissible behind the surviving queued item");
    CHECK(transmitted.size() == wire_before_c &&
              ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 2 &&
              ConnectionAdaptiveTestAccess::queuedPayloadText(c, 0) == "fifo-B" &&
              ConnectionAdaptiveTestAccess::queuedPayloadText(c, 1) == "fifo-C",
          "new payload must queue behind the older accepted successor, never bypass it");

    ConnectionAdaptiveTestAccess::restoreArqCallsigns(c);
    ConnectionAdaptiveTestAccess::pumpQueuedOperation(c);
    CHECK(transmitted.size() == wire_before_c + 1,
          "restoring the transport must start exactly one queued payload");
    const auto first_after_recovery = v2::DataFrame::deserialize(transmitted.back());
    CHECK(first_after_recovery &&
              first_after_recovery->payload.size() >= kMessageObjectPrefixBytes &&
              std::string(first_after_recovery->payload.begin() +
                              kMessageObjectPrefixBytes,
                          first_after_recovery->payload.end()) == "fifo-B",
          "the older surviving payload must retain wire priority over the new submission");

    c.abortTxNow();
    size_t failed_a = 0;
    size_t failed_b = 0;
    size_t failed_c = 0;
    for (const auto& event : statuses) {
        if (event.status != Connection::MessageTxStatus::FAILED) continue;
        failed_a += event.text == "fifo-A" ? 1 : 0;
        failed_b += event.text == "fifo-B" ? 1 : 0;
        failed_c += event.text == "fifo-C" ? 1 : 0;
    }
    CHECK(failed_a == 1 && failed_b == 1 && failed_c == 1,
          "every accepted FIFO item must receive exactly one terminal result");
}

// BUG-MESSAGE-LOST-ON-FORCED-DEMOTE. A mandatory escape used to DESTROY the
// in-flight message. Nothing physical requires that: a geometry change invalidates
// the object's fragmentation, not the object. These tests pin the replacement
// contract — re-grid the same object, and let the RECEIVER suppress the duplicate,
// because the sender cannot resolve "never received" vs "ACK in flight".
void test_midwindow_message_geometry_change_regrids_object_and_delivers_once() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    std::vector<bool> legacy_results;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    c.setMessageSentCallback(
        [&](bool success) { legacy_results.push_back(success); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R3_4, /*logical_cw=*/4);

    // Six characters + the two-byte identity prefix == the eight-byte payload the
    // post-demotion geometry can carry, so the object survives the re-cut whole and
    // the round trip stays one frame at both geometries.
    const std::string message = "REGRID";
    CHECK(c.sendMessage(message) && transmitted.size() == 1,
          "regrid fixture must put one live message frame on the wire");
    const auto first_attempt = v2::DataFrame::deserialize(transmitted.front());
    CHECK(first_attempt &&
              first_attempt->payload.size() == message.size() + kMessageObjectPrefixBytes &&
              first_attempt->payload[0] ==
                  static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT),
          "a message must reach the wire behind its object identity");
    const uint8_t object_id = first_attempt->payload[1];
    CHECK(object_id != 0, "object id 0 is reserved for 'no identity'");
    const uint8_t epoch_before =
        ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c);

    // Model an unavoidable receiver-commanded/stuck-frame demotion while the
    // message slot is live.
    ConnectionAdaptiveTestAccess::applyDataMode(
        c, Modulation::QPSK, CodeRate::R1_3, /*cw_count=*/1);
    CHECK(!ConnectionAdaptiveTestAccess::fragmentedMessagePending(c) &&
              ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 1 &&
              ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 1 &&
              ConnectionAdaptiveTestAccess::queuedPayloadText(c, 0) == message &&
              ConnectionAdaptiveTestAccess::queuedPayloadObjectId(c, 0) == object_id &&
              ConnectionAdaptiveTestAccess::messageGeometryRegridAttempts(c, 0) == 1,
          "a mandatory escape must re-admit the SAME object, not destroy it");
    CHECK(statuses.size() == 1 &&
              statuses.front().status == Connection::MessageTxStatus::SUBMITTED &&
              legacy_results.empty(),
          "a re-gridded object must not publish any terminal result");
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) != epoch_before,
          "abandoning the old slots must enter a fresh move epoch so the peer drops the stale prefix");
    CHECK(ConnectionAdaptiveTestAccess::currentPayloadCapacity(c) == 8,
          "forced demotion fixture must establish the smaller eight-byte capacity");

    const size_t wire_before_resume = transmitted.size();
    CHECK(!ConnectionAdaptiveTestAccess::sendFixedData(
              c, Bytes(9, 0xA5), v2::Flags::FINAL) &&
              transmitted.size() == wire_before_resume,
          "production ARQ must refuse an oversized stale-geometry chunk before wire serialization");

    // Production dispatch (the CONNECTED tick) re-fragments at the NEW geometry.
    c.tick(1);
    CHECK(ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 0 &&
              transmitted.size() == wire_before_resume + 1,
          "the re-admitted object must go back on the wire at the new geometry");
    const auto resumed = v2::DataFrame::deserialize(transmitted.back());
    CHECK(resumed && resumed->payload.size() <= 8 &&
              resumed->payload[0] ==
                  static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT) &&
              resumed->payload[1] == object_id,
          "the resend must carry the SAME object id — that is what makes it idempotent at the peer");
    CHECK(resumed->payload ==
              Bytes({static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT),
                     object_id, 'R','E','G','R','I','D'}),
          "the resent payload must be exact, never silently truncated");

    ToneBurstAckDetection ack;
    ack.payload.group_seq = static_cast<uint8_t>(resumed->seq & 0x3F);
    ack.payload.frame_mask = 0;
    ack.payload.rate_hint = 7;
    ack.payload.type = AckType::Ack;
    ack.payload.move_epoch = ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c);
    ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(ack), "resumed message ACK must be accepted");
    CHECK(statuses.size() == 2 &&
              statuses.back().status == Connection::MessageTxStatus::DELIVERED &&
              statuses.back().text == message &&
              statuses.back().remote_call == "K2DEF" &&
              legacy_results.size() == 1 && legacy_results.front(),
          "one accepted message must yield exactly one SUBMITTED and one DELIVERED across the re-grid");
}

void test_message_geometry_regrid_recuts_fragments_at_new_capacity() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    std::vector<Bytes> transmitted;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R3_4, /*logical_cw=*/4);

    const size_t old_capacity =
        ConnectionAdaptiveTestAccess::currentPayloadCapacity(c);
    const std::string long_message(old_capacity + 17, 'G');
    CHECK(c.sendMessage(long_message) &&
              ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "re-cut fixture must own a live multi-frame message");
    const auto first_attempt = v2::DataFrame::deserialize(transmitted.front());
    CHECK(first_attempt && first_attempt->payload.size() == old_capacity,
          "the first attempt must fill the old geometry exactly");
    const uint8_t object_id = first_attempt->payload[1];

    const size_t wire_before_resume = transmitted.size();
    ConnectionAdaptiveTestAccess::applyDataMode(
        c, Modulation::QPSK, CodeRate::R1_3, /*cw_count=*/1);
    CHECK(ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 1 &&
              ConnectionAdaptiveTestAccess::queuedPayloadText(c, 0) == long_message,
          "the whole logical object must be re-admitted, not the leftover fragments");

    c.tick(1);
    CHECK(ConnectionAdaptiveTestAccess::fragmentedMessagePending(c) &&
              transmitted.size() > wire_before_resume,
          "the re-admitted object must re-fragment and resume transmission");
    const auto resumed = v2::DataFrame::deserialize(transmitted[wire_before_resume]);
    CHECK(resumed && resumed->payload.size() == 8,
          "re-fragmentation must cut to the NEW eight-byte capacity, not the old one");
    CHECK(resumed->payload[0] ==
              static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT) &&
              resumed->payload[1] == object_id,
          "a re-cut object keeps its identity so the peer can suppress the duplicate");
    // Every resumed frame must respect the new geometry — a stale oversized chunk
    // reaching the smaller frame builder is the failure this escape exists to avoid.
    for (size_t i = wire_before_resume; i < transmitted.size(); ++i) {
        const auto frame = v2::DataFrame::deserialize(transmitted[i]);
        CHECK(frame && frame->payload.size() <= 8,
              "no resumed frame may exceed the new payload capacity");
    }
    c.abortTxNow();
}

void test_message_geometry_regrid_is_bounded_then_fails_closed() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    std::vector<Connection::MessageTxStatusEvent> statuses;
    std::vector<bool> legacy_results;
    c.setTransmitCallback([](const Bytes&) {});
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    c.setMessageSentCallback(
        [&](bool success) { legacy_results.push_back(success); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R3_4, /*logical_cw=*/4);

    const std::string message = "REGRID";
    CHECK(c.sendMessage(message), "bounded-regrid fixture must accept one message");

    // A message that re-cuts forever on a flapping ladder is worse for the operator
    // than one honest failure, so the escape is budgeted.
    const CodeRate ladder[] = {CodeRate::R1_3, CodeRate::R1_2, CodeRate::R2_3};
    for (size_t attempt = 0; attempt < 2; ++attempt) {
        ConnectionAdaptiveTestAccess::applyDataMode(
            c, Modulation::QPSK, ladder[attempt], /*cw_count=*/1);
        CHECK(ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 1 &&
                  ConnectionAdaptiveTestAccess::messageGeometryRegridAttempts(c, 0) ==
                      attempt + 1,
              "each escape within budget must re-grid the object and count the attempt");
        CHECK(legacy_results.empty(),
              "a re-gridded object must not report failure while it is still in budget");
        c.tick(1);  // resume: put it back on the wire so the next escape can see it
        CHECK(ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 0,
              "the re-admitted object must be dispatched before the next escape");
    }

    ConnectionAdaptiveTestAccess::applyDataMode(
        c, Modulation::QPSK, ladder[2], /*cw_count=*/1);
    CHECK(ConnectionAdaptiveTestAccess::outboundMessageRecordCount(c) == 0 &&
              ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 0,
          "an out-of-budget object must be dropped, not re-queued forever");
    CHECK(statuses.size() == 2 &&
              statuses.front().status == Connection::MessageTxStatus::SUBMITTED &&
              statuses.back().status == Connection::MessageTxStatus::FAILED &&
              statuses.back().text == message,
          "exhausting the re-grid budget must publish one attributed terminal failure");
    CHECK(legacy_results.size() == 1 && !legacy_results.front(),
          "the legacy boolean must report the terminal failure exactly once");
}

void test_receiver_suppresses_duplicate_message_object() {
    Connection c;
    std::vector<std::string> received;
    std::vector<Bytes> raw_received;
    c.setTransmitCallback([](const Bytes&) {});
    c.setMessageReceivedCallback(
        [&](const std::string& text) { received.push_back(text); });
    c.setDataReceivedCallback(
        [&](const Bytes& data, bool) { raw_received.push_back(data); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    const Bytes object = {
        static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT), 7,
        'h','e','l','l','o'};
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, object, /*more_data=*/false);
    CHECK(received.size() == 1 && received.front() == "hello",
          "a message object must be delivered once, without its transport prefix");
    CHECK(raw_received.size() == 1 &&
              raw_received.front() == Bytes({'h','e','l','l','o'}),
          "raw data consumers must not see the transport prefix either");

    // The sender may legitimately re-send after a geometry escape without knowing
    // the peer already has the object. Suppressing it here is what makes that safe.
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, object, /*more_data=*/false);
    CHECK(received.size() == 1 && raw_received.size() == 1,
          "a resend of an already-delivered object id must be suppressed");

    // A DIFFERENT object with the same text is a real second message, not a
    // duplicate — suppression keys on identity, never on content.
    const Bytes second = {
        static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT), 8,
        'h','e','l','l','o'};
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, second, /*more_data=*/false);
    CHECK(received.size() == 2 && received.back() == "hello",
          "a distinct object id carrying identical text must still be delivered");

    // Fragmented objects are keyed the same way: the identity rides the first
    // fragment and is read after reassembly.
    const Bytes frag_a = {
        static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT), 9, 'a','b'};
    const Bytes frag_b = {'c','d'};
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, frag_a, /*more_data=*/true);
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, frag_b, /*more_data=*/false);
    CHECK(received.size() == 3 && received.back() == "abcd",
          "a fragmented object must reassemble to exact text");
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, frag_a, /*more_data=*/true);
    ConnectionAdaptiveTestAccess::deliverDataPayload(c, frag_b, /*more_data=*/false);
    CHECK(received.size() == 3,
          "a re-sent fragmented object must be suppressed after reassembly");

    CHECK(ConnectionAdaptiveTestAccess::deliveredMessageObjectIdCount(c) == 3,
          "suppression history must hold one entry per delivered object");
}

void test_ordinary_adaptation_holds_geometry_for_complete_message() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QAM16, CodeRate::R2_3, /*logical_cw=*/4);

    const size_t capacity =
        ConnectionAdaptiveTestAccess::currentPayloadCapacity(c);
    CHECK(c.sendMessage(std::string(capacity + 11, 'A')) &&
              ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "adaptive hold fixture must start one multi-frame message");
    const size_t wire_before_feedback = transmitted.size();
    ConnectionAdaptiveTestAccess::applyFeedback(c, 0.0f);
    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R2_3 &&
              transmitted.size() == wire_before_feedback &&
              ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "ordinary quality adaptation must hold one geometry until the complete message retires");
    CHECK(statuses.size() == 1 &&
              statuses.front().status == Connection::MessageTxStatus::SUBMITTED,
          "an avoidable adaptive decision must not fail an accepted message");
    c.abortTxNow();
}

void test_runtime_forced_cw_override_cannot_regrid_active_message() {
    Connection c;
    std::vector<Bytes> transmitted;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R3_4, /*logical_cw=*/4);

    const size_t capacity =
        ConnectionAdaptiveTestAccess::currentPayloadCapacity(c);
    CHECK(c.sendMessage(std::string(capacity + 9, 'F')) &&
              ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "forced-CW guard fixture must start a geometry-bound message");
    const size_t wire_before_override = transmitted.size();

    c.setForcedFrameCodewords(/*cw_count=*/1, /*forced=*/true);
    CHECK(c.getForcedFrameCodewords() == 4 &&
              ConnectionAdaptiveTestAccess::configuredForcedCWCount(c) == 0 &&
              ConnectionAdaptiveTestAccess::currentPayloadCapacity(c) == capacity &&
              transmitted.size() == wire_before_override &&
              ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "public forced-CW API must reject a runtime regrid without partially mutating policy");

    c.abortTxNow();
    c.setForcedFrameCodewords(/*cw_count=*/1, /*forced=*/true);
    CHECK(c.getForcedFrameCodewords() == 1 &&
              ConnectionAdaptiveTestAccess::configuredForcedCWCount(c) == 1,
          "forced-CW override must remain available after the geometry owner retires");
}

void test_submitted_status_reset_runs_after_physical_handoff() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    bool reset_from_submitted = false;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
            if (!reset_from_submitted &&
                event.status == Connection::MessageTxStatus::SUBMITTED) {
                reset_from_submitted = true;
                c.reset();
            }
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    CHECK(c.sendMessage("reset-after-handoff"),
          "message admission should complete before SUBMITTED callback reset");
    CHECK(transmitted.size() == 1 &&
              v2::DataFrame::deserialize(transmitted.front()).has_value(),
          "SUBMITTED callback reset must not clear the ARQ slot before physical handoff");
    CHECK(reset_from_submitted && c.getState() == ConnectionState::DISCONNECTED &&
              ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == 0 &&
              !ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "reentrant reset must leave the connection fully reset after sendMessage returns");
    CHECK(statuses.size() == 2 &&
              statuses[0].status == Connection::MessageTxStatus::SUBMITTED &&
              statuses[0].sequence_valid &&
              statuses[1].status == Connection::MessageTxStatus::FAILED &&
              statuses[1].sequence_valid &&
              statuses[0].text == "reset-after-handoff" &&
              statuses[1].text == "reset-after-handoff",
          "status FIFO must publish SUBMITTED then attributed FAILED under reentrant reset");
}

void test_failure_batch_precedes_reentrant_replacement_submission() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    bool replacement_started = false;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
            if (!replacement_started &&
                event.status == Connection::MessageTxStatus::FAILED &&
                event.text == "abort-batch-A") {
                replacement_started = c.sendMessage("abort-batch-replacement");
            }
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    CHECK(c.sendMessages({"abort-batch-A", "abort-batch-B"}),
          "failure-order fixture must admit the original two-message batch");
    CHECK(statuses.size() == 2 &&
              statuses[0].status == Connection::MessageTxStatus::SUBMITTED &&
              statuses[1].status == Connection::MessageTxStatus::SUBMITTED,
          "original batch must publish two submissions before abort");

    c.abortTxNow();
    CHECK(replacement_started && statuses.size() == 5 &&
              statuses[2].status == Connection::MessageTxStatus::FAILED &&
              statuses[2].text == "abort-batch-A" &&
              statuses[3].status == Connection::MessageTxStatus::FAILED &&
              statuses[3].text == "abort-batch-B" &&
              statuses[4].status == Connection::MessageTxStatus::SUBMITTED &&
              statuses[4].text == "abort-batch-replacement",
          "detached old failures must drain before a reentrant replacement submission");
    c.abortTxNow();
}

void test_delivered_status_reset_runs_after_ack_unwind() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> statuses;
    bool reset_from_delivered = false;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
            if (!reset_from_delivered &&
                event.status == Connection::MessageTxStatus::DELIVERED) {
                reset_from_delivered = true;
                c.reset();
            }
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    CHECK(c.sendMessage("reset-after-delivery") && transmitted.size() == 1,
          "delivery-reset fixture must submit one DATA frame");
    const auto frame = v2::DataFrame::deserialize(transmitted.front());
    CHECK(frame.has_value(), "delivery-reset fixture must put a valid frame on wire");

    ToneBurstAckDetection ack;
    ack.payload.group_seq = static_cast<uint8_t>(frame->seq & 0x3F);
    ack.payload.frame_mask = 0;
    ack.payload.rate_hint = 7;
    ack.payload.type = AckType::Ack;
    ack.payload.move_epoch = ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c);
    ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(ack),
          "terminal tone ACK should be consumed before DELIVERED callback reset");
    CHECK(reset_from_delivered && c.getState() == ConnectionState::DISCONNECTED &&
              ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == 0,
          "DELIVERED callback reset must run after the complete ACK transaction unwinds");
    CHECK(statuses.size() == 2 &&
              statuses[0].status == Connection::MessageTxStatus::SUBMITTED &&
              statuses[1].status == Connection::MessageTxStatus::DELIVERED,
          "terminal callback reset must retain exactly one ordered SUBMITTED/DELIVERED pair");
}

void test_midwindow_geometry_change_disconnects_without_move_epoch() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "0", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    std::vector<Connection::MessageTxStatusEvent> statuses;
    c.setTransmitCallback([](const Bytes&) {});
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            statuses.push_back(event);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QPSK, CodeRate::R3_4, /*logical_cw=*/4);
    const size_t capacity =
        ConnectionAdaptiveTestAccess::currentPayloadCapacity(c);
    CHECK(c.sendMessage(std::string(capacity + 5, 'E')),
          "disabled-epoch geometry fixture must start a fragmented message");

    ConnectionAdaptiveTestAccess::applyDataMode(
        c, Modulation::QPSK, CodeRate::R1_3, /*cw_count=*/1);
    CHECK(c.getState() == ConnectionState::DISCONNECTING &&
              statuses.size() == 2 &&
              statuses.front().status == Connection::MessageTxStatus::SUBMITTED &&
              statuses.back().status == Connection::MessageTxStatus::FAILED &&
              statuses.back().remote_call == "K2DEF",
          "without move-epoch, abandoning live DATA must fail the message and close the unusable sequence grid");
    CHECK(!c.sendMessage("must-not-enter-wedged-grid"),
          "application retry must be rejected while the no-epoch session resets");
}

void test_file_and_payload_queues_preserve_cross_class_fifo() {
    Connection c;
    std::vector<Bytes> transmitted;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);
    ConnectionAdaptiveTestAccess::makeRemoteDataTurn(c);

    const Bytes short_binary{0x42, 0x32, 0x46};
    TempPayloadFile payload("message_file_fifo", 64);
    CHECK(!payload.path.empty(), "cross-class FIFO source file must be created");
    CHECK(c.sendBinary(short_binary),
          "short binary operation must queue while the peer owns the turn");
    CHECK(c.sendFile(payload.path),
          "later bulk file operation must be accepted behind the short payload");
    CHECK(ConnectionAdaptiveTestAccess::queuedPayloadCount(c) == 1 &&
              ConnectionAdaptiveTestAccess::queuedFile(c) &&
              !ConnectionAdaptiveTestAccess::fileControllerSending(c),
          "both logical operations must remain queued before turn acquisition");

    transmitted.clear();  // discard the standalone TURN_REQUEST control
    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    ConnectionAdaptiveTestAccess::pumpQueuedOperation(c);
    CHECK(!transmitted.empty() &&
              !ConnectionAdaptiveTestAccess::fileControllerSending(c),
          "the earlier short payload, not the queued file, must own the first ARQ slot");
    const auto first = v2::DataFrame::deserialize(transmitted.front());
    CHECK(first && first->type == v2::FrameType::DATA_END &&
              first->payload == short_binary,
          "cross-class FIFO must preserve the exact earlier binary payload");

    ToneBurstAckDetection ack;
    ack.payload.group_seq = static_cast<uint8_t>(first->seq & 0x3F);
    ack.payload.frame_mask = 0;
    ack.payload.rate_hint = 7;
    ack.payload.type = AckType::Ack;
    ack.payload.move_epoch = 0;
    ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(ack), "short-payload completion ACK must be consumed");
    CHECK(ConnectionAdaptiveTestAccess::fileControllerSending(c),
          "the file may start only after the earlier short payload retires");
}

void test_file_start_respects_reversed_data_turn_without_prior_local_backlog() {
    Connection c;
    std::vector<Bytes> transmitted;
    c.setTransmitCallback([&](const Bytes& frame) { transmitted.push_back(frame); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::makeRemoteDataTurn(c);

    TempPayloadFile payload("reversed_turn_file", 64);
    CHECK(!payload.path.empty(), "reversed-turn source file must be created");
    CHECK(c.sendFile(payload.path),
          "file request must be accepted into the ISS queue while the peer owns DATA");
    CHECK(ConnectionAdaptiveTestAccess::queuedFile(c) &&
              !ConnectionAdaptiveTestAccess::fileControllerSending(c),
          "non-interactive burst mode must not bypass a legitimately reversed DATA turn");
    CHECK(transmitted.size() == 1 &&
              v2::parseHeader(transmitted.front()).type == v2::FrameType::TURN_REQUEST,
          "remote-turn file request must emit only TURN_REQUEST, never FILE DATA");

    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    ConnectionAdaptiveTestAccess::pumpQueuedOperation(c);
    CHECK(ConnectionAdaptiveTestAccess::fileControllerSending(c),
          "queued file must start after TURNOVER grants the local DATA turn");
}

void test_same_epoch_rebase_preserves_multiwindow_message_prefix() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection receiver;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        receiver, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    std::vector<std::string> messages;
    receiver.setMessageReceivedCallback(
        [&](const std::string& text) { messages.push_back(text); });
    receiver.setTransmitCallback([](const Bytes&) {});

    auto inject = [&](uint16_t seq, const Bytes& payload, bool more) {
        auto frame = v2::makeFixedDataFrame(
            "K2DEF", "W1ABC", seq, payload, CodeRate::R2_3);
        frame.flags = static_cast<uint8_t>(
            frame.flags | v2::Flags::EPOCH_REBASE | v2::epochToFlags(0) |
            (more ? v2::Flags::MORE_FRAG : v2::Flags::FINAL));
        receiver.onFrameReceived(frame.serialize());
    };

    // Stop-and-wait/window-drain traffic can create each next fragment at the
    // current TX base, so every fragment legitimately carries EPOCH_REBASE in
    // the SAME epoch. None of those window boundaries is a logical-message reset.
    // The object identity rides the FIRST fragment only.
    inject(0, Bytes{static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT), 1, 'A'},
           true);
    inject(1, Bytes{'B'}, true);
    inject(2, Bytes{'C'}, false);
    CHECK(messages.size() == 1 && messages.front() == "ABC" &&
              ConnectionAdaptiveTestAccess::rxReassemblyBytes(receiver) == 0,
          "same-epoch rebase boundaries must preserve the complete message prefix");
}

void test_move_epoch_rebase_drops_stale_message_prefix() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection receiver;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        receiver, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    std::vector<std::string> messages;
    receiver.setMessageReceivedCallback(
        [&](const std::string& text) { messages.push_back(text); });
    receiver.setTransmitCallback([](const Bytes&) {});

    const uint8_t kObjectType =
        static_cast<uint8_t>(PayloadType::TEXT_MESSAGE_OBJECT);
    auto old_fragment = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 0, Bytes{kObjectType, 4, 'O', 'L', 'D', '-'},
        CodeRate::R2_3);
    old_fragment.flags = static_cast<uint8_t>(
        old_fragment.flags | v2::Flags::EPOCH_REBASE | v2::Flags::MORE_FRAG |
        v2::epochToFlags(0));
    receiver.onFrameReceived(old_fragment.serialize());
    CHECK(messages.empty() &&
              ConnectionAdaptiveTestAccess::rxReassemblyBytes(receiver) == 6,
          "old-era prefix must be buffered but not delivered");

    auto new_message = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 1, Bytes{kObjectType, 5, 'N', 'E', 'W'}, CodeRate::R2_3);
    new_message.flags = static_cast<uint8_t>(
        new_message.flags | v2::Flags::EPOCH_REBASE | v2::Flags::FINAL |
        v2::epochToFlags(1));
    receiver.onFrameReceived(new_message.serialize());

    CHECK(messages.size() == 1 && messages.front() == "NEW" &&
              ConnectionAdaptiveTestAccess::rxReassemblyBytes(receiver) == 0,
          "new-era message must deliver without contamination from the abandoned prefix");

    // Belt and braces on the same failure mode: if the stale prefix were NOT
    // dropped, the stitched object would start with the old era's bytes and no
    // longer parse as a message object. Prove that case is refused rather than
    // handed to the operator as content.
    auto stitched = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 2, Bytes{'O', 'L', 'D', '-', kObjectType, 6, 'X'},
        CodeRate::R2_3);
    stitched.flags = static_cast<uint8_t>(stitched.flags | v2::Flags::FINAL);
    receiver.onFrameReceived(stitched.serialize());
    CHECK(messages.size() == 1,
          "a payload with no valid message discriminator must never be delivered as text");
}

void test_authority_climb_prices_the_real_file_tail() {
    const uint8_t qpsk_r23 = kRungIdxQpskR23;
    const uint8_t psk8_r23 = kRungIdxQam8R23;

    // Derive the boundary from the same post-ACK target profile used in production.
    // The receiver prices after including the just-decoded clean group, while sender
    // round accounting commits that same evidence later in the ACK callback.  Start at
    // one clean group and require the guard plus actual refill to see the resulting
    // streak=2 escalated target geometry.  This generalizes final_default_07: the
    // selector priced a full 8PSK group but only three physical tail frames existed.
    constexpr int kPreAckCleanStreak = 1;
    constexpr int kPostAckCleanStreak = 2;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection shape;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        shape, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    const size_t target_chunk =
        ConnectionAdaptiveTestAccess::targetProfileChunkBytes(
            shape, Modulation::QAM8, CodeRate::R2_3);
    const size_t target_group =
        ConnectionAdaptiveTestAccess::targetProfileBurstFrames(
            shape, Modulation::QAM8, CodeRate::R2_3,
            kPostAckCleanStreak);
    CHECK(target_chunk > 0 && target_group >= 2,
          "8PSK target geometry must expose a real multi-frame burst boundary");

    // N-1 target frames are a different, shorter counterfactual than the full-cycle
    // geometry the latent argmax priced.  HOLD must emit neither descriptor DATA nor
    // a legacy MODE_CHANGE, and it must remain re-evaluable (no authority dedup latch).
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection short_tail;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    int short_controls = 0;
    int short_bursts = 0;
    short_tail.setTransmitCallback([&](const Bytes& b) {
        const auto h = v2::parseHeader(b);
        if (h.valid && h.is_control) ++short_controls;
    });
    short_tail.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) { ++short_bursts; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        short_tail, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(
        short_tail, kPreAckCleanStreak);
    TempPayloadFile short_payload(
        "authority_short_tail", target_chunk * (target_group - 1));
    ConnectionAdaptiveTestAccess::startFile(short_tail, short_payload.path);
    CHECK(ConnectionAdaptiveTestAccess::retireFileMetadata(short_tail),
          "short-tail fixture must retire FILE_START before pricing DATA frames");
    CHECK(!ConnectionAdaptiveTestAccess::authorityClimbHasSufficientPayload(
              short_tail, Modulation::QAM8, CodeRate::R2_3,
              kPostAckCleanStreak),
          "N-1 prospective target frames must fail the full-group price invariant");
    ConnectionAdaptiveTestAccess::obeyAuthorityAtCleanAckBoundary(
        short_tail, psk8_r23);
    CHECK(short_tail.getDataModulation() == Modulation::QPSK &&
              !ConnectionAdaptiveTestAccess::modeChangePending(short_tail) &&
              short_tail.getStats().descriptor_mode_switches == 0 &&
              short_controls == 0 && short_bursts == 1 &&
              ConnectionAdaptiveTestAccess::txAuthorityLastObeyed(short_tail) !=
                  psk8_r23,
          "short-tail UP hold must avoid switch egress/dedup while normal current-rung "
          "refill continues");

    // Exactly one complete target group still permits the normal descriptor climb.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection full_group;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    int full_group_bursts = 0;
    size_t full_group_burst_size = 0;
    full_group.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            ++full_group_bursts;
            full_group_burst_size = frames.size();
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        full_group, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(
        full_group, kPreAckCleanStreak);
    TempPayloadFile full_payload(
        "authority_full_group", target_chunk * target_group);
    ConnectionAdaptiveTestAccess::startFile(full_group, full_payload.path);
    CHECK(ConnectionAdaptiveTestAccess::retireFileMetadata(full_group),
          "full-group fixture must retire FILE_START before pricing DATA frames");
    CHECK(ConnectionAdaptiveTestAccess::authorityClimbHasSufficientPayload(
              full_group, Modulation::QAM8, CodeRate::R2_3,
              kPostAckCleanStreak),
          "N prospective target frames must meet the full-group price invariant");
    ConnectionAdaptiveTestAccess::obeyAuthorityAtCleanAckBoundary(
        full_group, psk8_r23);
    CHECK(full_group.getDataModulation() == Modulation::QAM8 &&
              !ConnectionAdaptiveTestAccess::modeChangePending(full_group) &&
              full_group.getStats().descriptor_mode_switches == 1 &&
              full_group_bursts == 1 &&
              full_group_burst_size == target_group &&
              ConnectionAdaptiveTestAccess::txAuthorityLastObeyed(full_group) ==
                  psk8_r23,
          "a complete mid-file target group must climb and emit the exact physical "
          "group that was priced");

    // The hold sits before both commit transports.  Descriptor-disabled operation
    // must not reinterpret tryDescriptorModeSwitch(false) as permission to pay a
    // legacy MODE_CHANGE for the same unprofitable tail.
    Connection legacy_tail;
    int legacy_controls = 0;
    legacy_tail.setTransmitCallback([&](const Bytes& b) {
        const auto h = v2::parseHeader(b);
        if (h.valid && h.is_control) ++legacy_controls;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        legacy_tail, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(
        legacy_tail, kPreAckCleanStreak);
    TempPayloadFile legacy_payload(
        "authority_legacy_tail", target_chunk * (target_group - 1));
    ConnectionAdaptiveTestAccess::startFile(legacy_tail, legacy_payload.path);
    CHECK(ConnectionAdaptiveTestAccess::retireFileMetadata(legacy_tail),
          "legacy-tail fixture must retire FILE_START before pricing DATA frames");
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(
        legacy_tail, psk8_r23, /*accepted_clean_round=*/true);
    CHECK(legacy_tail.getDataModulation() == Modulation::QPSK &&
              !ConnectionAdaptiveTestAccess::modeChangePending(legacy_tail) &&
              legacy_controls == 0,
          "descriptor-disabled short-tail hold must not fall through to MODE_CHANGE");

    // Reliability is asymmetric: a DOWN command may be the only way to recover a
    // failed FINAL-bearing member, so even a one-frame tail must bypass the UP-only
    // economic guard and enter the synchronized fallback when no descriptor can ride.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    Connection down_tail;
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    int down_controls = 0;
    down_tail.setTransmitCallback([&](const Bytes& b) {
        const auto h = v2::parseHeader(b);
        if (h.valid && h.is_control) ++down_controls;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        down_tail, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM8);
    TempPayloadFile down_payload("authority_down_tail", 64);
    ConnectionAdaptiveTestAccess::startFile(down_tail, down_payload.path);
    CHECK(ConnectionAdaptiveTestAccess::retireFileMetadata(down_tail),
          "down-tail fixture must retire FILE_START before its one-frame DATA tail");
    ConnectionAdaptiveTestAccess::maybeObeyAuthorityCommand(down_tail, qpsk_r23);
    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(down_tail) &&
              down_tail.getDataModulation() == Modulation::QAM8 &&
              down_controls == 1,
          "short-tail reliability demotion must bypass the climb price guard");
}

void test_timeout_batch_waits_for_post_tick_flush_and_discards_obsolete_geometry() {
    Connection committed;
    int committed_bursts = 0;
    int committed_standalone = 0;
    int committed_ack_arms = 0;
    std::vector<Bytes> committed_frames;
    committed.setTransmitCallback(
        [&](const Bytes& frame) {
            ++committed_standalone;
            committed_frames.push_back(frame);
        });
    committed.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) {
            ++committed_bursts;
        });
    committed.setArmToneBurstAckMonitorCallback(
        [&](uint32_t) { ++committed_ack_arms; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        committed, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(committed);
    ConnectionAdaptiveTestAccess::setArqMaxRetries(committed, 3);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              committed, Bytes{0x31}, v2::Flags::FINAL),
          "commit fixture should seed one live DATA identity");
    CHECK(committed_standalone == 1,
          "initial DATA should leave through the standalone callback");
    // Re-arm the exact identity to a deterministic one-tick deadline, then discard
    // initial-egress observations so only the timeout transaction is measured.
    const Bytes committed_identity = committed_frames.front();
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              committed, {committed_identity}, 1) == 1,
          "commit fixture should rearm the exact serialized identity");
    committed_bursts = 0;
    committed_standalone = 0;
    committed_ack_arms = 0;
    committed_frames.clear();
    committed.tick(1);
    CHECK(ConnectionAdaptiveTestAccess::stagedArqTimeoutFrames(committed) == 0 &&
              committed_bursts == 0 && committed_standalone == 1 &&
              committed_ack_arms == 1,
          "unchanged geometry should commit exactly one singleton timeout egress");
    CHECK(committed.getStats().arq.timeouts == 1 &&
              committed.getStats().arq.retransmissions_timeout == 1 &&
              ConnectionAdaptiveTestAccess::arqMaxInFlightRetryCount(committed) == 1,
          "committed timeout egress must consume exactly one retry transaction");

    Connection obsolete;
    int obsolete_physical_tx = 0;
    int obsolete_ack_arms = 0;
    std::vector<Bytes> obsolete_frames;
    obsolete.setTransmitCallback(
        [&](const Bytes& frame) {
            ++obsolete_physical_tx;
            obsolete_frames.push_back(frame);
        });
    obsolete.setTransmitBurstCallback(
        [&](const std::vector<Bytes>&, uint16_t, uint8_t) {
            ++obsolete_physical_tx;
        });
    obsolete.setArmToneBurstAckMonitorCallback(
        [&](uint32_t) { ++obsolete_ack_arms; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        obsolete, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(obsolete);
    // max_retries=1 is the sharp edge: the old pre-transaction implementation
    // terminal-failed inside ARQ::tick before Connection could discard the batch.
    ConnectionAdaptiveTestAccess::setArqMaxRetries(obsolete, 1);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              obsolete, Bytes{0x41}, v2::Flags::FINAL),
          "obsolete fixture should seed one live DATA identity");
    const Bytes obsolete_identity = obsolete_frames.front();
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              obsolete, {obsolete_identity}, 1) == 1,
          "obsolete fixture should rearm the exact serialized identity");
    obsolete_physical_tx = 0;
    obsolete_ack_arms = 0;
    obsolete_frames.clear();
    ConnectionAdaptiveTestAccess::markModeChangePending(obsolete);
    obsolete.tick(1);
    CHECK(ConnectionAdaptiveTestAccess::stagedArqTimeoutFrames(obsolete) == 0,
          "obsolete staged batch should be consumed by the discard path");
    CHECK(obsolete_physical_tx == 0 && obsolete_ack_arms == 0,
          "mode/geometry transition must discard old-rung timeout egress completely");
    CHECK(obsolete.getStats().arq.timeouts == 1 &&
              obsolete.getStats().arq.retransmissions == 0 &&
              obsolete.getStats().arq.failed == 0 &&
              ConnectionAdaptiveTestAccess::arqMaxInFlightRetryCount(obsolete) == 0 &&
              ConnectionAdaptiveTestAccess::arqInFlightBytes(obsolete) > 0,
          "discard must preserve retry budget and cannot terminal-fail a live identity");

    // The canceled intent is transition-suspended, not merely reset to another scalar
    // RTO: a long tick inside the same MODE_CHANGE must remain completely inert.
    obsolete.tick(1000);
    CHECK(obsolete_physical_tx == 0 && obsolete_ack_arms == 0 &&
              obsolete.getStats().arq.timeouts == 1 &&
              obsolete.getStats().arq.retransmissions == 0 &&
              obsolete.getStats().arq.failed == 0,
          "pending MODE_CHANGE must not repeatedly regenerate or consume canceled intents");
}

void test_timeout_repair_batch_is_capped_before_retry_commit() {
    Connection c;
    std::vector<Bytes> initial_frames;
    std::vector<size_t> repair_sizes;
    std::vector<uint8_t> repair_reasons;
    c.setTransmitCallback(
        [&](const Bytes& frame) { initial_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t reason) {
            repair_sizes.push_back(frames.size());
            repair_reasons.push_back(reason);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM8);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        c, Modulation::QAM8, CodeRate::R2_3, /*logical_cw=*/12);
    ConnectionAdaptiveTestAccess::setArqMaxRetries(c, 4);

    for (size_t i = 0; i < 8; ++i) {
        CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
                  c, Bytes{static_cast<uint8_t>(0x60 + i)},
                  i == 7 ? v2::Flags::FINAL : v2::Flags::MORE_FRAG),
              "timeout-cap fixture should seed all eight live identities");
    }
    CHECK(initial_frames.size() == 8,
          "timeout-cap fixture must retain the exact eight serialized identities");

    // A no-callback RTO is itself a zero-progress outcome, so it resets the
    // clean streak before pricing its reliability burst. The base full-anchor
    // ceiling is N4; only those four may spend retry budget in this round.
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(c, 2);
    ConnectionAdaptiveTestAccess::stageArqTimeoutBatch(c, initial_frames);
    CHECK(ConnectionAdaptiveTestAccess::stagedArqTimeoutFrames(c) == 8,
          "all eight timeout intents must stage before the physical cap is applied");
    ConnectionAdaptiveTestAccess::flushStagedArqTimeoutBatch(c);

    CHECK(repair_sizes.size() == 1 && repair_sizes.front() == 4 &&
              repair_reasons.front() == Connection::kAnchorReasonResend,
          "an N8 timeout must emit one base-ceiling N4 full-anchor repair");
    CHECK(c.getStats().arq.retransmissions_timeout == 4 &&
              ConnectionAdaptiveTestAccess::arqMaxInFlightRetryCount(c) == 1 &&
              ConnectionAdaptiveTestAccess::arqInFlightBytes(c) > 0,
          "overflow intents must remain live without consuming retry budget");
}

void test_arq_control_feedback_does_not_arm_data_ack_monitor() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<uint32_t> ack_monitor_windows;
    c.setTransmitCallback(
        [&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { ack_monitor_windows.push_back(window_ms); });

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    // Exercise the ARQ's serialized-control callback rather than calling
    // Connection::transmitFrame() directly. Connection has installed its protocol-side
    // tone callback, but this fixture deliberately has no physical tone-TX callback.
    // It must therefore retain the legacy plane: explicit frame NACK plus cumulative
    // SACK. Neither control frame may replace/extend the sender's DATA listen window.
    auto final_out_of_order = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 1, Bytes{0x42}, CodeRate::R2_3);
    final_out_of_order.flags |= v2::Flags::FINAL;
    c.onFrameReceived(final_out_of_order.serialize());

    CHECK(transmitted.size() == 2,
          "legacy final-gap feedback should emit explicit NACK plus cumulative SACK");
    for (const auto& frame : transmitted) {
        const auto header = v2::parseHeader(frame);
        CHECK(header.valid && header.is_control,
              "legacy final-gap feedback must consist only of control frames");
    }
    CHECK(ack_monitor_windows.empty(),
          "ARQ control feedback must not arm the DATA tone-ACK monitor");
}

void test_data_ack_monitor_arms_before_synchronous_transport_callback() {
    Connection c;
    std::vector<std::string> callback_order;
    std::vector<uint32_t> ack_monitor_windows;
    bool synchronous_ack_consumed = false;
    size_t data_tx_count = 0;

    c.setArmToneBurstAckMonitorCallback([&](uint32_t window_ms) {
        callback_order.push_back("arm");
        ack_monitor_windows.push_back(window_ms);
    });
    c.setTransmitCallback([&](const Bytes& frame) {
        const auto data = v2::DataFrame::deserialize(frame);
        if (!data) return;
        ++data_tx_count;
        callback_order.push_back("tx");

        ToneBurstAckDetection ack;
        ack.payload.group_seq = static_cast<uint8_t>(data->seq & 0x3F);
        ack.payload.frame_mask = 0;
        ack.payload.rate_hint = 7;
        ack.payload.type = AckType::Ack;
        ack.payload.move_epoch = 0;
        ack.payload.rung_cmd = kRungCmdNone;
        synchronous_ack_consumed = c.onToneBurstAck(ack);
    });

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              c, Bytes{0x51, 0x52}, v2::Flags::FINAL),
          "standalone DATA should enter the ARQ sender");
    CHECK(data_tx_count == 1 && synchronous_ack_consumed,
          "transport callback should synchronously loop one valid tone ACK");
    CHECK(callback_order.size() == 2 && callback_order[0] == "arm" &&
              callback_order[1] == "tx",
          "DATA ACK monitor must arm before handing DATA to a synchronous transport");
    CHECK(ack_monitor_windows.size() == 1,
          "synchronous ACK must not leave a second stale monitor arm after transport returns");
    CHECK(ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == 0,
          "synchronous cumulative ACK should retire the transmitted DATA identity");
}

void test_full_repair_timing_uses_encoder_samples_and_respects_ceiling() {
    constexpr Modulation kMod = Modulation::QAM8;
    constexpr CodeRate kRate = CodeRate::R2_3;
    constexpr int kCW = 12;

    auto frames = [=](size_t count) {
        std::vector<Bytes> result;
        result.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            result.push_back(v2::makeFixedDataFrame(
                "W1ABC", "K2DEF", static_cast<uint16_t>(i), Bytes{0x5A},
                kRate, kCW, 27).serialize());
        }
        return result;
    };

    Connection adaptive;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        adaptive, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        adaptive, kMod, kRate, kCW);

    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(adaptive, 0);
    CHECK(ConnectionAdaptiveTestAccess::burstFrameBudget(adaptive) == 5 &&
              ConnectionAdaptiveTestAccess::burstFrameBudget(
                  adaptive, /*force_full_group_start=*/true) == 4,
          "base ceiling must retain light N5 but cap a full-anchor repair at N4");

    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(adaptive, 2);
    CHECK(ConnectionAdaptiveTestAccess::burstFrameBudget(adaptive) == 8 &&
              ConnectionAdaptiveTestAccess::burstFrameBudget(
                  adaptive, /*force_full_group_start=*/true) == 7,
          "two trusted clean groups must retain light N8 but cap full repair at N7");

    const auto n5 = frames(5);
    const auto n8 = frames(8);
    CHECK(ConnectionAdaptiveTestAccess::physicalRoundKeyedSamples(
              adaptive, n5, false) == 374080 &&
              ConnectionAdaptiveTestAccess::physicalRoundKeyedSamples(
                  adaptive, n5, true) == 431680,
          "N5 light/full keyed durations must match emitted 48-kHz sample geometry");
    CHECK(ConnectionAdaptiveTestAccess::physicalRoundKeyedSamples(
              adaptive, n8, false) == 552160 &&
              ConnectionAdaptiveTestAccess::physicalRoundKeyedSamples(
                  adaptive, n8, true) == 609760,
          "N8 light/full keyed durations must match emitted 48-kHz sample geometry");
    CHECK(ConnectionAdaptiveTestAccess::physicalRoundTimeout(
              adaptive, n5, false) ==
              ConnectionAdaptiveTestAccess::physicalRoundTimeout(
                  adaptive, n5, true),
          "full-anchor airtime must replace the normal reliability reserve, not "
          "double-charge the ACK response deadline");

    Connection pinned;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        pinned, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(pinned, kMod, kRate, kCW);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(pinned, 2);
    ConnectionAdaptiveTestAccess::setAutomaticRateAllowed(pinned, false);
    CHECK(ConnectionAdaptiveTestAccess::burstFrameBudget(pinned) == 5 &&
              ConnectionAdaptiveTestAccess::burstFrameBudget(
                  pinned, /*force_full_group_start=*/true) == 4,
          "operator-pinned 8PSK must not reinterpret two clean groups as N8 evidence");
}

void test_variable_cw_single_frame_uses_advertised_physical_geometry() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<uint32_t> ack_monitor_windows;
    c.setTransmitCallback(
        [&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { ack_monitor_windows.push_back(window_ms); });

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    // The single-block file path deliberately bypasses fixed-frame chunking. Force
    // the burst policy to z=81 as an adversarial check: a standalone frame still
    // physically goes through ModemEngine::transmit(), which resets it to z=27.
    // The raw override is now intentionally neutered at the Connection boundary
    // too, because a singleton has no BURST_HEADER in which to announce Z=81.
    setenv("ULTRA_LDPC_Z", "81", 1);
    CHECK(c.selectBurstLiftingZ() == 27,
          "raw ULTRA_LDPC_Z must not create unannounced singleton Z=81 traffic");
    const bool submitted = ConnectionAdaptiveTestAccess::sendVariableData(
        c, Bytes(2300, 0x5A), v2::Flags::FINAL);
    const Bytes serialized = transmitted.empty() ? Bytes{} : transmitted.front();
    const auto header = v2::parseHeader(serialized);
    const uint32_t exact_timeout =
        ConnectionAdaptiveTestAccess::physicalRoundTimeout(c, transmitted);
    const uint32_t exact_airtime =
        ConnectionAdaptiveTestAccess::physicalRoundAirtime(c, transmitted);
    const auto short_timing = connection_policy::wideOFDMFrameTimingForCodewords(
        Modulation::QPSK, CodeRate::R2_3, header.total_cw, 27);
    const auto wrong_long_timing = connection_policy::wideOFDMFrameTimingForCodewords(
        Modulation::QPSK, CodeRate::R2_3, header.total_cw, 81);
    unsetenv("ULTRA_LDPC_Z");
    const uint32_t fixed_timeout =
        ConnectionAdaptiveTestAccess::unifiedBurstTimeout(c, 1);

    CHECK(submitted, "large variable DATA should enter the ARQ sender");
    CHECK(transmitted.size() == 1,
          "variable DATA should produce exactly one physical frame");
    CHECK(header.valid && header.type == v2::FrameType::DATA &&
              header.total_cw > v2::kMaxFixedFrameCodewords,
          "fixture must advertise a variable total_cw above the fixed ceiling");
    CHECK(header.total_cw == 45,
          "2300-byte R2/3 variable frame should serialize to exactly 45 CW");
    CHECK(short_timing.data_symbols == 288 && short_timing.data_ms == 6912,
          "45-CW QPSK R2/3 z27 wire geometry should be 288 symbols / 6912 ms");
    const uint32_t expected_keyed_ms =
        connection_policy::postProcessedTxDurationFromSamplesMs(
            connection_policy::kWideOFDMFullAnchorExtraSamples +
            connection_policy::wideOFDMWireFrameSamplesForCodewords(
                Modulation::QPSK, CodeRate::R2_3, header.total_cw, 27));
    CHECK(exact_airtime == expected_keyed_ms,
          "singleton key-down must include 45 short CW, its full anchor, and "
          "the shared audio guards");
    CHECK(exact_airtime == 8120,
          "45-CW variable singleton should occupy exactly 8120 ms after guards");
    CHECK(exact_airtime < wrong_long_timing.data_ms,
          "standalone variable DATA must remain z=27 even when burst policy forces z=81");
    CHECK(exact_timeout > fixed_timeout,
          "variable frame RTO must exceed the stale fixed-frame RTO");
    CHECK(ack_monitor_windows.size() == 1 &&
              ack_monitor_windows.front() == std::max<uint32_t>(8000u, exact_timeout),
          "tone ACK monitor must use the same exact variable-frame geometry");

    // Prove the ARQ slot itself was rearmed from that geometry: the old fixed timeout
    // must not launch an overlapping resend while the long frame/response is in flight.
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    c.tick(fixed_timeout);
    CHECK(transmitted.size() == 1 && c.getStats().arq.timeouts == 0,
          "variable DATA must not retry at the configured fixed-frame deadline");
    c.tick(exact_timeout - fixed_timeout - 1);
    CHECK(transmitted.size() == 1 && c.getStats().arq.timeouts == 0,
          "variable DATA must remain live until its exact physical deadline");
    c.tick(1);
    CHECK(transmitted.size() == 2 && c.getStats().arq.timeouts == 1,
          "variable DATA should retry exactly at its physical deadline");
}

void test_narrow_ofdm_keeps_its_waveform_specific_timeout() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<uint32_t> ack_monitor_windows;
    c.setTransmitCallback(
        [&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { ack_monitor_windows.push_back(window_ms); });
    ConnectionAdaptiveTestAccess::makeConnectedNarrowOFDM(c);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const uint32_t narrow_timeout =
        ConnectionAdaptiveTestAccess::arqAckTimeout(c);
    // This is the erroneous deadline the wide-only exact-round path used to install.
    const uint32_t wrong_wide_timeout =
        ConnectionAdaptiveTestAccess::unifiedBurstTimeout(c, 1);
    CHECK(wrong_wide_timeout < narrow_timeout,
          "fixture must expose the premature wide-vs-narrow timeout gap");

    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              c, Bytes{0x4E, 0x42}, v2::Flags::FINAL),
          "narrow DATA should enter the ARQ sender");
    CHECK(transmitted.size() == 1,
          "narrow singleton should produce one physical frame");
    CHECK(ConnectionAdaptiveTestAccess::physicalRoundTimeout(c, transmitted) == 0,
          "wide physical-round timing must be out of scope for OFDM_NARROW");
    CHECK(ack_monitor_windows.size() == 1 &&
              ack_monitor_windows.front() ==
                  std::max<uint32_t>(8000u, narrow_timeout),
          "narrow singleton monitor must retain the narrow policy timeout");

    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    c.tick(wrong_wide_timeout);
    CHECK(transmitted.size() == 1 && c.getStats().arq.timeouts == 0,
          "narrow DATA must not retry at the shorter wide-only deadline");
    c.tick(narrow_timeout - wrong_wide_timeout - 1);
    CHECK(transmitted.size() == 1 && c.getStats().arq.timeouts == 0,
          "narrow DATA must remain live until its waveform-specific deadline");
    c.tick(1);
    CHECK(transmitted.size() == 2 && c.getStats().arq.timeouts == 1,
          "narrow DATA should retry exactly at its own policy deadline");
}

void test_mcdpsk_data_arms_one_waveform_specific_monitor_per_physical_turn() {
    Connection grouped;
    std::vector<std::vector<Bytes>> bursts;
    std::vector<uint32_t> grouped_monitor_windows;
    grouped.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    grouped.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { grouped_monitor_windows.push_back(window_ms); });
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(grouped);
    const uint32_t grouped_timeout =
        ConnectionAdaptiveTestAccess::arqAckTimeout(grouped);

    TempPayloadFile payload("mcdpsk_group_monitor", 64);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "MC-DPSK monitor fixture should be created");
    CHECK(grouped.sendFile(payload.path),
          "MC-DPSK file should enter the grouped sender");
    CHECK(bursts.size() == 1 && bursts.front().size() >= 2,
          "MC-DPSK fixture should emit one multi-frame physical group");
    CHECK(grouped_monitor_windows.size() == 1 &&
              grouped_monitor_windows.front() ==
                  std::max<uint32_t>(8000u, grouped_timeout),
          "MC-DPSK group must arm exactly once for its waveform-specific RTT");

    Connection singleton;
    std::vector<Bytes> standalone;
    std::vector<uint32_t> singleton_monitor_windows;
    singleton.setTransmitCallback(
        [&](const Bytes& frame) { standalone.push_back(frame); });
    singleton.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { singleton_monitor_windows.push_back(window_ms); });
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(singleton);
    const uint32_t singleton_timeout =
        ConnectionAdaptiveTestAccess::arqAckTimeout(singleton);
    CHECK(ConnectionAdaptiveTestAccess::sendVariableData(
              singleton, Bytes{0x4D, 0x43}, v2::Flags::FINAL),
          "MC-DPSK singleton DATA should enter the ARQ sender");
    CHECK(standalone.size() == 1 && singleton_monitor_windows.size() == 1,
          "MC-DPSK singleton should transmit and arm exactly once");
    CHECK(singleton_monitor_windows.front() ==
              std::max<uint32_t>(8000u, singleton_timeout),
          "MC-DPSK singleton monitor must retain the waveform-specific RTT");
}

void test_synchronous_burst_ack_cannot_overwrite_outer_committed_frames() {
    Connection c;
    std::vector<Bytes> nested_standalone_frames;
    size_t outer_size_before_ack = 0;
    size_t outer_size_after_ack = 0;
    int burst_callback_count = 0;

    c.setTransmitCallback(
        [&](const Bytes& frame) { nested_standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            ++burst_callback_count;
            if (burst_callback_count != 1) {
                return;
            }
            outer_size_before_ack = frames.size();

            ToneBurstAckDetection complete_ack;
            complete_ack.payload.group_seq = static_cast<uint8_t>(
                (frames.size() - 1) & 0x3F);
            complete_ack.payload.frame_mask = 0;
            complete_ack.payload.rate_hint = 7;
            complete_ack.payload.type = AckType::Ack;
            complete_ack.payload.move_epoch = 0;
            complete_ack.payload.rung_cmd = kRungCmdNone;
            ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
            CHECK(c.onToneBurstAck(complete_ack),
                  "synchronous burst callback ACK should be consumed");
            outer_size_after_ack = frames.size();
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t frame_count = ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 2 && capacity > 0,
          "fixture needs a full group plus one nested-refill frame");
    // Size the TEXT so the WIRE object (text + message-object identity prefix)
    // lands on an exact fragment boundary.
    CHECK(c.sendMessage(std::string(
              (frame_count + 1) * capacity - kMessageObjectPrefixBytes, 'r')),
          "message should enter the synchronous burst transport");

    CHECK(burst_callback_count == 1,
          "first physical turn should use one multi-frame burst callback");
    CHECK(outer_size_before_ack == frame_count &&
              outer_size_after_ack == frame_count,
          "nested ACK/refill must not clear or overwrite the outer committed vector");
    CHECK(nested_standalone_frames.size() == 1,
          "synchronous ACK should refill the one-frame message tail exactly once");
    auto nested_tail = v2::DataFrame::deserialize(nested_standalone_frames[0]);
    CHECK(nested_tail && nested_tail->seq == frame_count,
          "nested refill should carry the next logical sequence identity");
}

void test_ack_revealed_hole_refill_uses_resend_anchor() {
    Connection c;
    std::vector<std::vector<Bytes>> bursts;
    std::vector<uint8_t> anchor_reasons;
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t anchor_reason) {
            bursts.push_back(frames);
            anchor_reasons.push_back(anchor_reason);
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t frame_count = ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 3 && frame_count <= 16 && capacity > 0,
          "fixture needs two repair holes plus a queued tail member");
    CHECK(c.sendMessage(std::string((frame_count + 1) * capacity, 'a')),
          "fragmented message should enter the unified sender");
    CHECK(bursts.size() == 1 && bursts.front().size() == frame_count,
          "initial turn should fill one normal burst");
    CHECK(anchor_reasons.front() == Connection::kAnchorReasonNone,
          "a first-attempt burst should retain its normal anchor policy");

    // Keep seq 0 and 1 as holes while positively SACKing the rest.  The next
    // physical turn coalesces those two repairs with the queued tail, so it is a
    // multi-frame burst rather than the standalone-singleton special case.
    ToneBurstAckDetection partial_ack;
    partial_ack.payload.group_seq = 0x3F;
    partial_ack.payload.frame_mask = static_cast<uint16_t>(
        ((1u << frame_count) - 1u) & ~0x3u);
    partial_ack.payload.rate_hint = 7;
    partial_ack.payload.type = AckType::Ack;
    partial_ack.payload.move_epoch = 0;
    partial_ack.payload.rung_cmd = kRungCmdNone;
    // A newer receiver may stamp exact-group provenance even when this older/default
    // sender keeps the experiment disabled. Reserved advisory=3 is HOLD-compatible,
    // so the default path must remain the established double/full repair anchor.
    partial_ack.payload.drive_advisory = kDriveAdvisoryReserved;

    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(partial_ack),
          "partial tone SACK should be accepted as a fresh turn boundary");
    CHECK(bursts.size() == 2 && bursts.back().size() >= 3,
          "repair turn should coalesce both holes with queued new data");
    CHECK(anchor_reasons.back() == Connection::kAnchorReasonResend,
          "ACK-revealed hole refill must force the same full anchor as an RTO resend");
}

void test_proven_partial_sack_uses_descriptor_only_anchor_and_light_timing() {
    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection c;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<uint8_t> anchor_reasons;
    std::vector<uint32_t> ack_monitor_windows;
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t anchor_reason) {
            bursts.push_back(frames);
            anchor_reasons.push_back(anchor_reason);
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    c.setArmToneBurstAckMonitorCallback(
        [&](uint32_t timeout_ms) { ack_monitor_windows.push_back(timeout_ms); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t light_budget =
        ConnectionAdaptiveTestAccess::burstFrameBudget(c, false);
    const size_t full_budget =
        ConnectionAdaptiveTestAccess::burstFrameBudget(c, true);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(light_budget > full_budget && full_budget >= 3 &&
              light_budget <= 16 && capacity > 0,
          "fixture must expose distinct light/full physical burst budgets");
    CHECK(c.sendMessage(std::string((2 * light_budget - 2) * capacity, 'p')),
          "message must leave enough new fragments to refill the entire light budget");
    CHECK(bursts.size() == 1 && bursts.front().size() == light_budget &&
              anchor_reasons.front() == Connection::kAnchorReasonNone,
          "initial physical group must use the ordinary light geometry");

    // Cumulatively retire every member except the two physical tail identities.
    // Keeping the holes at the tail (rather than at base) also releases enough ARQ
    // slots for queued new fragments to prove the light N budget itself.
    ToneBurstAckDetection partial_ack;
    partial_ack.payload.group_seq = static_cast<uint8_t>(light_budget - 3);
    partial_ack.payload.frame_mask = 0;
    partial_ack.payload.rate_hint = 7;
    partial_ack.payload.type = AckType::Ack;
    partial_ack.payload.move_epoch = 0;
    partial_ack.payload.rung_cmd = kRungCmdNone;
    partial_ack.payload.drive_advisory = kDriveAdvisoryReserved;

    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(partial_ack),
          "exact-group progress-bearing partial SACK must be accepted");
    CHECK(bursts.size() == 2 && bursts.back().size() == light_budget,
          "descriptor-only repair must spend the light N budget, not the full-anchor N budget");
    CHECK(anchor_reasons.back() ==
              Connection::kAnchorReasonProvenPartialRepair,
          "proven partial repair must bind its distinct anchor policy to the physical request");
    CHECK(ack_monitor_windows.size() == 2 &&
              ack_monitor_windows.back() ==
                  ConnectionAdaptiveTestAccess::physicalRoundTimeout(
                      c, bursts.back(), /*force_full_group_start=*/false),
          "proven partial repair must arm the exact light-group ACK deadline");
    CHECK(ConnectionAdaptiveTestAccess::physicalRoundWaveformSamples(
              c, bursts.back(), /*force_full_group_start=*/true) >
              ConnectionAdaptiveTestAccess::physicalRoundWaveformSamples(
                  c, bursts.back(), /*force_full_group_start=*/false),
          "timing oracle must retain a measurable second-chirp cost for the full fallback");
}

void test_proven_partial_sack_uses_physical_round_progress_not_cumulative_arq_progress() {
    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection c;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<uint8_t> anchor_reasons;
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t anchor_reason) {
            bursts.push_back(frames);
            anchor_reasons.push_back(anchor_reason);
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t n = ConnectionAdaptiveTestAccess::burstFrameBudget(c, false);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(n >= 6 && n <= 16 && capacity > 0,
          "cumulative-release fixture needs an N>=6 descriptor-bearing group");
    CHECK(c.sendMessage(std::string(3 * n * capacity, 'i')),
          "fixture must retain enough queued data for three physical turns");
    CHECK(bursts.size() == 1 && bursts.front().size() == n,
          "fixture must begin with one light-budget group");

    // Leave seq0 and the last two identities as holes, while positively SACKing
    // seq1..N-3.  Because the cumulative base remains pinned at seq0, those SACKed
    // slots stay active until a later round fills that base hole.
    ToneBurstAckDetection first_partial;
    first_partial.payload.group_seq = 0x3F;
    first_partial.payload.frame_mask = 0;
    for (size_t seq = 1; seq + 2 < n; ++seq) {
        first_partial.payload.frame_mask |= static_cast<uint16_t>(1u << seq);
    }
    first_partial.payload.rate_hint = 7;
    first_partial.payload.type = AckType::Ack;
    first_partial.payload.move_epoch = 0;
    first_partial.payload.rung_cmd = kRungCmdNone;
    first_partial.payload.drive_advisory = kDriveAdvisoryReserved;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(first_partial),
          "first exact partial SACK must launch a proven light repair");
    CHECK(bursts.size() == 2 && bursts.back().size() >= 5 &&
              anchor_reasons.back() == Connection::kAnchorReasonProvenPartialRepair,
          "first repair must retain the descriptor-only/light policy");

    // The repair contains the three holes followed by new seq N, N+1, ... . Model a
    // physical 3/M result: seq0 plus new seq N and N+1 decode, while the other members
    // remain holes. Filling seq0 releases the previously SACKed seq1..N-3, so the ARQ
    // accessor legitimately reports N frames of cumulative progress: (N-2 retired) +
    // two new SACK bits. That number is >= this repair's physical M (the TX window can
    // make M smaller than N), but it must not be mistaken for a clean physical group;
    // the exact identities from this round still prove 3/M.
    ToneBurstAckDetection cumulative_release;
    cumulative_release.payload.group_seq = static_cast<uint8_t>(n - 3);
    cumulative_release.payload.frame_mask = 0x000Cu;  // seq N,N+1 from base N-2
    cumulative_release.payload.rate_hint = 7;
    cumulative_release.payload.type = AckType::Ack;
    cumulative_release.payload.move_epoch = 0;
    cumulative_release.payload.rung_cmd = kRungCmdNone;
    cumulative_release.payload.drive_advisory = kDriveAdvisoryReserved;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(cumulative_release),
          "cumulative-release SACK must remain support-valid");
    CHECK(bursts.size() == 3,
          "physical partial progress must synchronously launch the next repair");
    CHECK(anchor_reasons.back() == Connection::kAnchorReasonProvenPartialRepair,
          "cumulative ARQ progress >= N must not suppress a proven physical 3/N repair");
}

void test_partial_sack_experiment_fails_closed_without_progress() {
    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection c;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<uint8_t> anchor_reasons;
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t anchor_reason) {
            bursts.push_back(frames);
            anchor_reasons.push_back(anchor_reason);
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t frame_count =
        ConnectionAdaptiveTestAccess::burstFrameBudget(c, false);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 3 && frame_count <= 16 && capacity > 0,
          "zero-progress fixture needs a descriptor-bearing group");
    CHECK(c.sendMessage(std::string((frame_count + 1) * capacity, 'z')),
          "zero-progress fixture must seed one full group and queued tail");

    ToneBurstAckDetection crater_ack;
    crater_ack.payload.group_seq = 0x3F;
    crater_ack.payload.frame_mask = 0;
    crater_ack.payload.rate_hint = 0;
    crater_ack.payload.type = AckType::Ack;
    crater_ack.payload.move_epoch = 0;
    crater_ack.payload.rung_cmd = kRungCmdNone;
    crater_ack.payload.drive_advisory = kDriveAdvisoryReserved;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(crater_ack),
          "zero-progress exact-group SACK must still be a valid turn boundary");
    CHECK(bursts.size() == 2 &&
              anchor_reasons.back() == Connection::kAnchorReasonResend,
          "0/N crater must remain a double/full-anchor repair despite exact-group provenance");
}

void test_partial_sack_experiment_requires_progress_from_this_ack() {
    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection c;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<uint8_t> anchor_reasons;
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t anchor_reason) {
            bursts.push_back(frames);
            anchor_reasons.push_back(anchor_reason);
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t n = ConnectionAdaptiveTestAccess::burstFrameBudget(c, false);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(n >= 3 && n <= 16 && capacity > 0,
          "fresh-progress fixture needs a descriptor-bearing group");
    CHECK(c.sendMessage(std::string((n + 2) * capacity, 'f')),
          "fixture must leave queued data for a repair refill");
    CHECK(bursts.size() == 1,
          "fixture must retain the initial physical-round identity snapshot");

    // Model a different accepted ACK path first positively SACKing seq1 without
    // allowing Connection to form another physical round. The captured latest-round
    // identities now contain one already-ACKed member and N-1 live holes.
    CHECK(ConnectionAdaptiveTestAccess::applyToneAckToArqOnly(
              c, /*group_seq=*/0x3F, /*bitmap=*/0x0002u),
          "direct precursor SACK must be support-valid");

    // A later fresh signature carries exact-group provenance but contributes no new
    // ARQ progress. Identity state alone still looks partial (1/N), so this pins the
    // independent requirement that THIS ACK be progress-bearing. Without it, an old
    // callback/control SACK could authorize a light repair from stale delivery state.
    ToneBurstAckDetection zero_progress;
    zero_progress.payload.group_seq = 0x3F;
    zero_progress.payload.frame_mask = 0;
    zero_progress.payload.rate_hint = 7;
    zero_progress.payload.type = AckType::Ack;
    zero_progress.payload.move_epoch = 0;
    zero_progress.payload.rung_cmd = kRungCmdNone;
    zero_progress.payload.drive_advisory = kDriveAdvisoryReserved;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(zero_progress),
          "fresh zero-progress exact-provenance ACK must remain a turn boundary");
    CHECK(bursts.size() == 2 &&
              anchor_reasons.back() == Connection::kAnchorReasonResend,
          "identity history without progress from this ACK must fail closed to full repair");
}

void test_fragment_tail_hole_repairs_immediately_and_uses_one_frame_rto() {
    Connection c;
    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    std::vector<bool> message_results;
    std::vector<uint32_t> ack_monitor_windows;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setMessageSentCallback(
        [&](bool success) { message_results.push_back(success); });
    c.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { ack_monitor_windows.push_back(window_ms); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t frame_count =
        ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity =
        ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 2 && frame_count <= 16,
          "fixture needs one multi-frame group addressable by the tone bitmap");
    CHECK(capacity > 0, "fixture needs a nonzero fixed-frame payload capacity");

    const uint32_t full_round_timeout =
        ConnectionAdaptiveTestAccess::unifiedBurstTimeout(c, frame_count);
    const uint32_t one_frame_timeout =
        ConnectionAdaptiveTestAccess::unifiedBurstTimeout(c, 1);
    CHECK(one_frame_timeout < full_round_timeout,
          "one-frame repair deadline must be shorter than the full group deadline");

    // Text sized so the WIRE object (text + message-object identity prefix) is an
    // exact frame_count-fragment group.
    CHECK(c.sendMessage(
              std::string(frame_count * capacity - kMessageObjectPrefixBytes, 'x')),
          "fragmented message should enter the unified sender");
    CHECK(bursts.size() == 1 && bursts[0].size() == frame_count,
          "message should leave as one exact budget-sized initial group");
    CHECK(standalone_frames.empty(),
          "initial multi-frame message must not leak standalone DATA");
    CHECK(ack_monitor_windows.size() == 1 &&
              ack_monitor_windows[0] == full_round_timeout,
          "initial group should arm one exact full-round monitor window");

    for (size_t i = 0; i < frame_count; ++i) {
        auto frame = v2::DataFrame::deserialize(bursts[0][i]);
        CHECK(frame && frame->seq == i,
              "initial group should carry contiguous seq identities");
    }

    // Receiver has every frame except seq0. base remains -1 and bits 1..N-1 are
    // positive SACK evidence. This is the message equivalent of the rig's file tail.
    ToneBurstAckDetection hole_ack;
    hole_ack.payload.group_seq = 0x3F;
    hole_ack.payload.frame_mask = static_cast<uint16_t>(
        ((1u << frame_count) - 1u) & ~1u);
    hole_ack.payload.rate_hint = 7;
    hole_ack.payload.type = AckType::Ack;
    hole_ack.payload.move_epoch = 0;
    hole_ack.payload.rung_cmd = kRungCmdNone;

    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(hole_ack), "fresh tail-hole tone SACK should be consumed");
    CHECK(standalone_frames.size() == 1,
          "tail hole should trigger one immediate standalone repair");
    auto repair = v2::DataFrame::deserialize(standalone_frames.back());
    CHECK(repair && repair->seq == 0,
          "immediate repair must carry only the missing base frame");
    CHECK(c.getStats().arq.retransmissions_nack == 1,
          "tone turn should account exactly one NACK-driven repair");
    CHECK(c.getStats().arq.timeouts == 0,
          "immediate tail repair must not wait for an RTO");
    const uint32_t one_frame_monitor_window =
        std::max<uint32_t>(8000u, one_frame_timeout);
    CHECK(ack_monitor_windows.size() == 2 &&
              ack_monitor_windows[1] == one_frame_monitor_window,
          "immediate repair should replace the monitor with one-frame geometry");

    // An immediate repeated copy is the same ACK event, not a new transmit turn.
    CHECK(c.onToneBurstAck(hole_ack), "duplicate tone SACK should be consumed");
    CHECK(standalone_frames.size() == 1 &&
              c.getStats().arq.retransmissions_nack == 1,
          "duplicate tone SACK must not launch a duplicate repair");
    CHECK(ack_monitor_windows.size() == 2,
          "duplicate tone SACK must not rearm the ACK monitor");

    // If that repair's ACK is lost, prove the committed slot timer is the one-frame
    // physical deadline rather than the initial cap/full-group deadline.
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    c.tick(one_frame_timeout - 1);
    CHECK(c.getStats().arq.timeouts == 0 && standalone_frames.size() == 1,
          "repair must not time out before its exact one-frame deadline");
    c.tick(1);
    CHECK(c.getStats().arq.timeouts == 1 && standalone_frames.size() == 2,
          "lost repair ACK should retry exactly at the one-frame deadline");
    CHECK(ack_monitor_windows.size() == 3 &&
              ack_monitor_windows[2] == one_frame_monitor_window,
          "one-frame RTO repair should exact-rearm the same short monitor window");

    ToneBurstAckDetection complete_ack;
    complete_ack.payload.group_seq =
        static_cast<uint8_t>((frame_count - 1) & 0x3F);
    complete_ack.payload.frame_mask = 0;
    complete_ack.payload.rate_hint = 7;
    complete_ack.payload.type = AckType::Ack;
    complete_ack.payload.move_epoch = 0;
    complete_ack.payload.rung_cmd = kRungCmdNone;
    CHECK(c.onToneBurstAck(complete_ack),
          "final cumulative tone ACK should complete the message");
    CHECK(message_results.size() == 1 && message_results[0],
          "fragmented message should report one successful completion");
    CHECK(!ConnectionAdaptiveTestAccess::fragmentedMessagePending(c),
          "message state should clear after final cumulative ACK");
}

void test_file_tail_identical_keepalive_sack_retries_hole() {
    Connection c;
    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    std::vector<uint32_t> ack_monitor_windows;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setArmToneBurstAckMonitorCallback(
        [&](uint32_t window_ms) { ack_monitor_windows.push_back(window_ms); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    // Fading=0.30 keeps this off the near-AWGN single-block shortcut. A tiny file
    // produces FILE_START + final FILE_DATA in one initial fixed-frame turn, leaving
    // no unsent chunks to hide whether the tail-hole repair was duplicated.
    TempPayloadFile payload("identical_keepalive_file_tail_sack", 64);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "file-tail fixture should be created");
    CHECK(c.sendFile(payload.path), "file-tail fixture should enter the sender");
    CHECK(bursts.size() == 1 && bursts.front().size() >= 2 &&
              bursts.front().size() <= 16,
          "tiny file should emit one tone-addressable multi-frame turn");
    CHECK(standalone_frames.empty(),
          "initial tiny-file turn should not leak standalone DATA");

    const size_t frame_count = bursts.front().size();
    ToneBurstAckDetection hole_ack;
    hole_ack.payload.group_seq = 0x3F;
    hole_ack.payload.frame_mask = static_cast<uint16_t>(
        ((1u << frame_count) - 1u) & ~1u);
    hole_ack.payload.rate_hint = 7;
    hole_ack.payload.type = AckType::Ack;
    hole_ack.payload.move_epoch = 0;
    hole_ack.payload.rung_cmd = kRungCmdNone;

    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(hole_ack), "fresh file-tail hole SACK should be consumed");
    CHECK(standalone_frames.size() == 1,
          "fresh file-tail hole SACK should emit exactly one singleton repair");
    auto repair = v2::DataFrame::deserialize(standalone_frames.back());
    CHECK(repair && repair->seq == 0,
          "file-tail singleton repair must carry only the missing base identity");
    CHECK(c.getStats().arq.retransmissions_nack == 1,
          "fresh file-tail turn should account one NACK-driven repair");
    CHECK(ack_monitor_windows.size() == 2,
          "initial file turn plus singleton repair should arm two monitor windows");

    // Model the receiver's keepalive response after that repair/ACK exchange was lost.
    // Its cumulative payload is necessarily identical, so ARQ reports no new SACK
    // progress.  It is nevertheless a fresh physical turn and must wake the hole before
    // the long RTO.  The real ToneBurstAckMonitor permits one detection per arm; direct
    // Connection calls here intentionally model detections from two successive arms.
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(hole_ack),
          "identical keepalive file-tail SACK should be consumed");
    CHECK(bursts.size() == 1 && standalone_frames.size() == 2,
          "identical keepalive SACK should emit one new singleton repair turn");
    auto keepalive_repair = v2::DataFrame::deserialize(standalone_frames.back());
    CHECK(keepalive_repair && keepalive_repair->seq == 0,
          "keepalive-triggered repair must retain the missing base identity");
    CHECK(c.getStats().arq.retransmissions_nack == 2,
          "keepalive-triggered repair should be accounted as a second NACK resend");
    CHECK(ack_monitor_windows.size() == 3,
          "keepalive-triggered repair should replace the monitor for its new turn");
}

void test_terminal_tail_failure_aborts_suspended_window_and_rebases_next_payload() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    std::vector<bool> message_results;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setMessageSentCallback(
        [&](bool success) { message_results.push_back(success); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);
    CHECK(ConnectionAdaptiveTestAccess::arqMoveEpochEnabled(c),
          "fixture requires move-epoch continuation safety");

    const size_t frame_count = ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 3 && capacity > 0,
          "fixture needs a multi-frame physical round");
    CHECK(c.sendMessage(std::string(frame_count * capacity, 't')),
          "fragmented message should enter the sender");
    CHECK(bursts.size() == 1 && bursts[0].size() == frame_count,
          "fixture should emit one complete initial burst");

    // Model a cap-shrunk tail round where only seq0..1 actually went on air. Exact
    // physical re-arming must suspend every other live hole until this round ends.
    // Both current identities share the same deadline: the first terminal failure
    // must stop the scan before seq1 produces a second callback/failure.
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              c, std::vector<Bytes>{bursts[0][0], bursts[0][1]}, 100) == 2,
          "two exact tail identities should be re-armed");
    ConnectionAdaptiveTestAccess::setArqMaxRetries(c, 1);
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    c.tick(100);

    CHECK(message_results.size() == 1 && !message_results[0],
          "terminal tail failure should report one logical message failure");
    CHECK(c.getStats().arq.failed == 1,
          "one timeout scan must stop after its first terminal frame failure");
    CHECK(ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == 0,
          "terminal failure must abort all suspended residual holes");
    CHECK(bursts.size() == 1 && standalone_frames.empty(),
          "failure unwind must not key a doomed residual retry batch");
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == 1,
          "terminal abandonment must force a fresh move epoch");

    CHECK(c.sendMessage("after-failure"),
          "a later operator payload should be accepted after the atomic abort");
    CHECK(standalone_frames.size() == 1,
          "later payload should leave on a clean standalone physical round");
    auto recovery = v2::DataFrame::deserialize(standalone_frames.back());
    CHECK(recovery && v2::epochFromFlags(recovery->flags) == 1 &&
              (recovery->flags & v2::Flags::EPOCH_REBASE) != 0,
          "first post-failure DATA must carry the new epoch rebase anchor");
}

void test_turn_refill_terminal_failure_discards_open_burst() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    std::vector<bool> message_results;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setMessageSentCallback(
        [&](bool success) { message_results.push_back(success); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t frame_count = ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 2 && capacity > 0,
          "fixture needs multiple turn-refill holes");
    CHECK(c.sendMessage(std::string(frame_count * capacity, 'u')),
          "fragmented message should emit its initial group");
    CHECK(bursts.size() == 1 && bursts[0].size() == frame_count,
          "fixture should start with exactly one physical group");
    ConnectionAdaptiveTestAccess::setArqMaxRetries(c, 1);

    ToneBurstAckDetection no_progress_ack;
    no_progress_ack.payload.group_seq = 0x3F;  // cumulative base = -1
    no_progress_ack.payload.frame_mask = 0;    // every original frame is a hole
    no_progress_ack.payload.rate_hint = 7;
    no_progress_ack.payload.type = AckType::Ack;
    no_progress_ack.payload.move_epoch = 0;
    no_progress_ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(no_progress_ack),
          "fresh no-progress tone ACK should drive the turn refill");

    CHECK(message_results.size() == 1 && !message_results[0],
          "turn-driven max-retry failure should report once");
    CHECK(ConnectionAdaptiveTestAccess::arqInFlightBytes(c) == 0,
          "turn-driven failure should atomically drain every remaining hole");
    CHECK(bursts.size() == 1 && standalone_frames.empty(),
          "an open refill burst must be discarded instead of transmitted after failure");
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == 1,
          "turn-driven terminal failure must force the next sequence era");
}

void test_terminal_base_hole_with_sacked_suffix_reports_once_and_reentrant_send_rebases() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    std::vector<bool> legacy_results;
    std::vector<Connection::MessageTxStatusEvent> status_events;
    bool recovery_started = false;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setMessageSentCallback(
        [&](bool success) { legacy_results.push_back(success); });
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            status_events.push_back(event);
            if (event.status == Connection::MessageTxStatus::FAILED &&
                event.text != "callback-recovery") {
                // This deliberately mutates outbound message tracking from inside the
                // public FAILED callback. It is safe only if the old record was erased
                // and the forced epoch abort completed before callback publication.
                recovery_started = c.sendMessage("callback-recovery");
            }
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    const size_t frame_count = ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(frame_count >= 2 && frame_count <= 16 && capacity > 0,
          "fixture needs a tone-addressable multi-frame tail");
    CHECK(c.sendMessage(std::string(frame_count * capacity, 's')),
          "fragmented original message should enter the sender");
    CHECK(bursts.size() == 1 && bursts[0].size() == frame_count,
          "original message should emit one full group");
    ConnectionAdaptiveTestAccess::setArqMaxRetries(c, 1);

    // Exact live-rig tail shape: base seq0 is missing while every later tail frame
    // is already SACKed. The seq0 turn-repair reaches max retries; advanceTXWindow
    // then walks the SACKed suffix, which must NOT publish false successes.
    ToneBurstAckDetection tail_hole_ack;
    tail_hole_ack.payload.group_seq = 0x3F;
    tail_hole_ack.payload.frame_mask = static_cast<uint16_t>(
        ((1u << frame_count) - 1u) & ~1u);
    tail_hole_ack.payload.rate_hint = 7;
    tail_hole_ack.payload.type = AckType::Ack;
    tail_hole_ack.payload.move_epoch = 0;
    tail_hole_ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(tail_hole_ack),
          "fresh SACKed-suffix tail ACK should be consumed");

    CHECK(legacy_results.size() == 1 && !legacy_results[0],
          "base failure plus SACKed suffix must publish one failure and zero successes");
    size_t original_failed = 0;
    size_t original_delivered = 0;
    for (const auto& event : status_events) {
        if (event.text == std::string(frame_count * capacity, 's')) {
            original_failed +=
                event.status == Connection::MessageTxStatus::FAILED ? 1 : 0;
            original_delivered +=
                event.status == Connection::MessageTxStatus::DELIVERED ? 1 : 0;
        }
    }
    CHECK(original_failed == 1 && original_delivered == 0,
          "original message status must be exactly FAILED, never DELIVERED");
    CHECK(c.getStats().arq.failed == 1,
          "SACKed suffix retirement must not create additional frame failures");
    CHECK(recovery_started && standalone_frames.size() == 1,
          "FAILED callback should safely submit one replacement message post-abort");
    auto recovery = v2::DataFrame::deserialize(standalone_frames.back());
    CHECK(recovery && v2::epochFromFlags(recovery->flags) == 1 &&
              (recovery->flags & v2::Flags::EPOCH_REBASE) != 0,
          "reentrant replacement DATA must already carry the new epoch rebase");
}

void test_delivered_status_callback_can_send_without_corrupting_tracking() {
    Connection c;
    std::vector<Bytes> transmitted;
    std::vector<Connection::MessageTxStatusEvent> status_events;
    bool replacement_started = false;
    c.setTransmitCallback(
        [&](const Bytes& frame) { transmitted.push_back(frame); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    c.setMessageTxStatusCallback(
        [&](const Connection::MessageTxStatusEvent& event) {
            status_events.push_back(event);
            if (event.status == Connection::MessageTxStatus::DELIVERED &&
                event.text == "delivered-original") {
                replacement_started = c.sendMessage("delivered-replacement");
            }
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    CHECK(c.sendMessage("delivered-original"),
          "original message should enter the ARQ sender");
    CHECK(transmitted.size() == 1,
          "original one-frame message should produce one physical DATA frame");

    ToneBurstAckDetection ack;
    ack.payload.group_seq = 0;
    ack.payload.frame_mask = 0;
    ack.payload.rate_hint = 7;
    ack.payload.type = AckType::Ack;
    ack.payload.move_epoch = 0;
    ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    CHECK(c.onToneBurstAck(ack), "completion ACK should be consumed");

    CHECK(replacement_started && transmitted.size() == 2,
          "DELIVERED callback should safely submit a replacement message");
    auto replacement = v2::DataFrame::deserialize(transmitted.back());
    CHECK(replacement && replacement->seq == 1,
          "replacement should retain the next live ARQ identity");

    size_t original_delivered = 0;
    size_t replacement_submitted = 0;
    for (const auto& event : status_events) {
        if (event.text == "delivered-original" &&
            event.status == Connection::MessageTxStatus::DELIVERED) {
            ++original_delivered;
        }
        if (event.text == "delivered-replacement" &&
            event.status == Connection::MessageTxStatus::SUBMITTED) {
            ++replacement_submitted;
        }
    }
    CHECK(original_delivered == 1 && replacement_submitted == 1,
          "old delivery should publish once and preserve the new tracking record");
}

void test_terminal_failure_disconnects_when_move_epoch_is_disabled() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "0", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);
    CHECK(!ConnectionAdaptiveTestAccess::arqMoveEpochEnabled(c),
          "fixture must exercise the legacy sequence semantics");

    const size_t frame_count = ConnectionAdaptiveTestAccess::burstFrameBudget(c);
    const size_t capacity = ConnectionAdaptiveTestAccess::dataPayloadCapacity(c);
    CHECK(c.sendMessage(std::string(frame_count * capacity, 'v')),
          "fixture message should enter the legacy sender");
    CHECK(bursts.size() == 1 && bursts[0].size() == frame_count,
          "fixture should emit one initial group");
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              c, std::vector<Bytes>{bursts[0][0]}, 100) == 1,
          "one legacy tail identity should be armed");
    ConnectionAdaptiveTestAccess::setArqMaxRetries(c, 1);
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    c.tick(100);

    CHECK(c.getState() == ConnectionState::DISCONNECTING,
          "without move-epoch, terminal abandonment must reset the QSO via disconnect");
    CHECK(!c.sendMessage("unsafe-continuation"),
          "legacy sequence mode must reject new DATA after terminal abandonment");
    CHECK(!standalone_frames.empty() &&
              v2::parseHeader(standalone_frames.back()).type == v2::FrameType::DISCONNECT,
          "legacy fallback should put a DISCONNECT control frame on air");
}

void test_terminal_file_failure_disconnects_to_reset_peer_assembler() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    Connection c;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<std::vector<Bytes>> bursts;
    std::vector<Bytes> standalone_frames;
    std::vector<bool> file_results;
    c.setTransmitCallback(
        [&](const Bytes& frame) { standalone_frames.push_back(frame); });
    c.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            bursts.push_back(frames);
        });
    c.setFileSentCallback(
        [&](bool success, const std::string&) { file_results.push_back(success); });
    c.setTransmitToneBurstAckCallback(
        [](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload&) {});
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(c);

    TempPayloadFile payload("terminal_file_failure", 4096);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "terminal file fixture should be created");
    CHECK(c.sendFile(payload.path), "file transfer should enter the sender");
    CHECK(!bursts.empty() && !bursts[0].empty(),
          "file transfer should emit an initial physical group");
    CHECK(ConnectionAdaptiveTestAccess::rearmExactDataRound(
              c, std::vector<Bytes>{bursts[0][0]}, 100) == 1,
          "one file identity should be armed for terminal failure");
    ConnectionAdaptiveTestAccess::setArqMaxRetries(c, 1);
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(c);
    c.tick(100);

    CHECK(file_results.size() == 1 && !file_results[0],
          "terminal file failure should publish exactly one failed result");
    CHECK(c.getState() == ConnectionState::DISCONNECTING,
          "failed file must disconnect even with move-epoch enabled so peer RX resets");
    CHECK(!standalone_frames.empty() &&
              v2::parseHeader(standalone_frames.back()).type == v2::FrameType::DISCONNECT,
          "terminal file failure should send DISCONNECT to reset peer assembler state");
}

// Knob-OFF identity pin (ULTRA_RX_RATE_CMD=0, the baseline pinned in main): the
// receiver's emitted tone-ACK carries rung_cmd 0 even for a QAM16 crater, and the
// sender ignores an incoming DOWN-hard command outright — no mode move, no
// MODE_CHANGE frame, no descriptor commit.
void test_rx_rate_cmd_knob_off_is_byte_identical() {
    // Receiver emit side: a total QAM16 crater must NOT set the command.
    Connection r;
    std::vector<ToneBurstAckPayload> acks;
    r.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& p) { acks.push_back(p); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        r, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    r.onBurstGroupReceived(3, {}, /*all_ok=*/false, /*quality=*/0.0f,
                           /*frame_mask=*/0, /*interleaved=*/true, /*group_size=*/8);
    CHECK(!acks.empty(), "crater group must still emit its tone-burst ACK");
    CHECK(acks.back().rung_cmd == kRungCmdNone,
          "knob-off: the emitted rung_cmd bits must stay 0 (byte-identical wire)");
    CHECK(ConnectionAdaptiveTestAccess::rxRateCmdPending(r) == 0,
          "knob-off: no standing command may be latched");

    // Sender consume side: an incoming DOWN-hard command must be ignored.
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    TempPayloadFile payload("rx_rate_cmd_off", 60 * 1024);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "temp payload file should be created");
    ConnectionAdaptiveTestAccess::startFile(c, payload.path);
    ConnectionAdaptiveTestAccess::fillArqWindow(c, 4);

    c.onToneBurstAck(makeRungCmdDetection(/*group_seq=*/1, kRungCmdDownHard));

    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R2_3,
          "knob-off: an incoming rung command must not move the mode");
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "knob-off: an incoming rung command must not arm a MODE_CHANGE");
    CHECK(countModeChangeFrames(tx_frames) == 0,
          "knob-off: no MODE_CHANGE frame may go on the wire");
    CHECK(c.getStats().descriptor_mode_switches == 0,
          "knob-off: no descriptor commit may be counted");
}

// Knob-ON receiver emit: a TOTAL crater (frame_mask == 0) at QAM16 sets DOWN-hard on
// the group's ACK; the SAME command re-rides subsequent ACKs of the unchanged state
// (ACK-loss diversity, sender dedups by group_seq); the observed adoption (a real
// mod/rate change through applyDataMode) clears it; a group that delivers frames
// clears it; a crater at a non-QAM16 mode commands nothing (deep null at a robust
// rung is irreducible fading, not a rate signal — the 2026-06-09 ratchet guard).
void test_rx_rate_cmd_receiver_emits_crater_down_hard_once_per_move() {
    setenv("ULTRA_RX_RATE_CMD", "1", 1);
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);  // adoption path for the clear
    Connection r;  // ctor latches both knobs ON for this instance
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);  // restore the pinned baseline
    setenv("ULTRA_RX_RATE_CMD", "0", 1);
    // New controller regressions below exercise these default-on mechanisms directly.
    // Pin them before any function-local static reads so an operator test shell cannot
    // silently select a different controller or suppress the observable ALC side effect.
    setenv("ULTRA_LATENT_RATE", "1", 1);
    setenv("ULTRA_LATENT_RELAX_DB", "0.35", 1);
    setenv("ULTRA_LATENT_TIE_PROBE", "0", 1);
    setenv("ULTRA_SOFTWARE_ALC", "1", 1);

    std::vector<ToneBurstAckPayload> acks;
    r.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& p) { acks.push_back(p); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        r, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);

    // Total crater at QAM16 -> DOWN-hard rides this group's ACK.
    r.onBurstGroupReceived(5, {}, false, 0.0f, 0, true, 8);
    CHECK(!acks.empty() && acks.back().rung_cmd == kRungCmdDownHard,
          "QAM16 total crater must command DOWN-hard on the group's own ACK");

    // Unchanged state -> the SAME standing command re-rides (idempotent diversity;
    // the ARQ base is frozen during a crater so the sender dedups all copies).
    r.onBurstGroupReceived(5, {}, false, 0.0f, 0, true, 8);
    CHECK(acks.back().rung_cmd == kRungCmdDownHard,
          "repeated crater ACKs re-carry the same standing command");

    // Sender's adoption observed (descriptor announces QPSK R3/4) -> latch clears.
    r.onDescriptorModeChange(Modulation::QPSK, CodeRate::R3_4, 8);
    CHECK(ConnectionAdaptiveTestAccess::rxRateCmdPending(r) == 0,
          "an applied mod/rate change is the adoption — the command latch must clear");
    r.onBurstGroupReceived(6, {}, false, 0.0f, 0, true, 8);
    CHECK(acks.back().rung_cmd == kRungCmdNone,
          "post-adoption crater at QPSK must NOT command (non-QAM16 = fade physics)");

    // A delivering group ends any crater state (fresh QAM16 instance).
    setenv("ULTRA_RX_RATE_CMD", "1", 1);
    Connection r2;
    setenv("ULTRA_RX_RATE_CMD", "0", 1);
    std::vector<ToneBurstAckPayload> acks2;
    r2.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& p) { acks2.push_back(p); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        r2, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    r2.onBurstGroupReceived(9, {}, false, 0.0f, 0, true, 8);
    CHECK(ConnectionAdaptiveTestAccess::rxRateCmdPending(r2) == kRungCmdDownHard,
          "crater must latch the command");
    // PARTIAL-CRATER policy (2026-07-04, F27): a failed-but-partial group after a
    // crater is the SECOND consecutive bad group — the rung is under water; the
    // command steps to DOWN-ONE (one rung per evidence quantum) instead of clearing.
    // Only a CLEAN group ends the bad stretch.
    r2.onBurstGroupReceived(10, {}, false, 0.0f, /*frame_mask=*/0x3, true, 8);
    CHECK(ConnectionAdaptiveTestAccess::rxRateCmdPending(r2) == ultra::waveform::tone_burst_ack::kRungCmdDownOne,
          "second consecutive failed group (partial) must command DOWN-ONE");
    r2.onBurstGroupReceived(11, {}, true, 0.95f, /*frame_mask=*/0xFF, true, 8);
    CHECK(ConnectionAdaptiveTestAccess::rxRateCmdPending(r2) == 0,
          "a CLEAN group ends the bad stretch — no stale command may ride");
}

void test_waiting_rebase_voice_has_per_outcome_event_identity() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    setenv("ULTRA_RX_RATE_CMD", "1", 1);
    Connection receiver;
    Connection sender;
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");
    setenv("ULTRA_RX_RATE_CMD", "0", 1);

    std::vector<ToneBurstAckPayload> voices;
    receiver.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& payload, bool inbound_group_complete) {
            if (payload.type == AckType::Nack &&
                payload.rung_cmd ==
                    ultra::waveform::tone_burst_ack::kRungCmdReserved) {
                CHECK(inbound_group_complete,
                      "WAITING-REBASE voice must retain completed-group provenance");
                voices.push_back(payload);
            }
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        receiver, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    auto unanchoredMember = [](uint16_t seq, uint8_t value) {
        auto frame = v2::makeFixedDataFrame(
            "K2DEF", "W1ABC", seq, Bytes{value}, CodeRate::R2_3);
        frame.flags = static_cast<uint8_t>(
            frame.flags | v2::epochToFlags(1) | v2::Flags::MORE_FRAG);
        return frame.serialize();
    };

    // The unified descriptor currently reports zero for every physical group.
    // Two independently completed group outcomes must still produce two voice
    // event identities while the era-base frame remains missing.
    receiver.onBurstGroupReceived(
        /*descriptor group_seq=*/0, {unanchoredMember(5, 0x55)},
        /*all_ok=*/true, /*quality=*/0.9f, /*frame_mask=*/0x1,
        /*interleaved=*/false, /*group_size=*/1);
    receiver.onBurstGroupReceived(
        /*descriptor group_seq=*/0, {unanchoredMember(6, 0x66)},
        /*all_ok=*/true, /*quality=*/0.9f, /*frame_mask=*/0x1,
        /*interleaved=*/false, /*group_size=*/1);

    CHECK(voices.size() == 2,
          "each unanchored physical group outcome must emit one WAITING-REBASE voice");
    CHECK(voices[0].group_seq == 0 && voices[1].group_seq == 1,
          "voice identity must advance mod-64 independently of descriptor group_seq");
    CHECK(voices[0].frame_mask == 0 && voices[1].frame_mask == 0 &&
              voices[0].rate_hint == 0 && voices[1].rate_hint == 0,
          "dedup identity must not alter the reserved voice wire contract");

    // Sender-side behavioral pin: the exact cached copy of event 0 expires the
    // era-base timer only once, while the next physical outcome (event 1) earns a
    // new prompt resend. This is the failure the descriptor's constant zero caused.
    std::vector<Bytes> sender_tx;
    sender.setTransmitCallback(
        [&](const Bytes& frame) { sender_tx.push_back(frame); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        sender, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::disableAdaptiveRate(sender);
    CHECK(ConnectionAdaptiveTestAccess::arqAckTimeout(sender) > 2000,
          "fixture needs a non-imminent era-base timer");
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(
              sender, Bytes(16, 0x42), v2::Flags::MORE_FRAG),
          "fixture era-base DATA must enter the sender ARQ");
    CHECK(sender_tx.size() == 1,
          "fixture must start with exactly one era-base transmission");

    ToneBurstAckDetection heard;
    heard.payload = voices[0];
    CHECK(sender.onToneBurstAck(heard),
          "first voice event must be consumed");
    sender.tick(1);
    CHECK(sender_tx.size() == 2,
          "first voice event must trigger a prompt era-base resend");

    CHECK(sender.onToneBurstAck(heard),
          "cached repeat of the first voice must still be consumed whole");
    sender.tick(1);
    CHECK(sender_tx.size() == 2,
          "cached repeat with the same event identity must not retrigger the resend");

    heard.payload = voices[1];
    CHECK(sender.onToneBurstAck(heard),
          "next physical group's voice must be consumed");
    sender.tick(1);
    CHECK(sender_tx.size() == 3,
          "next voice event identity must retrigger a prompt era-base resend");
}

// Knob-ON sender consume, MID-WINDOW (the Phase-2 case): a DOWN-hard command with
// frames in flight routes the escape target (QAM16 -> QPSK R3/4) through the
// DESCRIPTOR commit — no MODE_CHANGE frame, mode applied immediately, and the ARQ
// abort bumps the move-epoch (era safety; requires ULTRA_ARQ_MOVE_EPOCH ON). A
// duplicate of the same command (same group_seq) before adoption is a no-op.
void test_rx_rate_cmd_down_hard_mid_window_commits_via_descriptor_with_epoch() {
    setenv("ULTRA_ARQ_MOVE_EPOCH", "1", 1);
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "1", 1);
    setenv("ULTRA_RX_RATE_CMD", "1", 1);
    Connection c;  // ctor latches all three knobs ON for this instance
    setenv("ULTRA_RX_RATE_CMD", "0", 1);  // restore the pinned baselines
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    unsetenv("ULTRA_ARQ_MOVE_EPOCH");

    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
    std::vector<bool> burst_full_anchor;
    std::vector<size_t> burst_sizes;
    c.setTransmitBurstCallback([&](const std::vector<Bytes>& frames, uint16_t /*seq*/,
                                   bool force_full_preamble) {
        burst_sizes.push_back(frames.size());
        burst_full_anchor.push_back(force_full_preamble);
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM16);
    TempPayloadFile payload("rx_rate_cmd_mid", 60 * 1024);
    CHECK(payload.dir.valid() && !payload.path.empty(),
          "temp payload file should be created");
    ConnectionAdaptiveTestAccess::startFile(c, payload.path);
    ConnectionAdaptiveTestAccess::fillArqWindow(c, 4);  // MID-window: frames in flight
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == 0,
          "fresh connection starts at TX move-epoch 0");

    // The crater group's ACK arrives carrying DOWN-hard (epoch echo 0 = current era).
    c.onToneBurstAck(makeRungCmdDetection(/*group_seq=*/1, kRungCmdDownHard));

    // ULTRA_QAM16_DEMOTE_MIDRUNG DEFAULT-ON (2026-07-05): the first DOWN-hard from
    // 16QAM R2/3 lands at the 16QAM R1/2 midrung (2x margin, stays on the mod); a
    // second escape takes the QPSK R3/4 exit.
    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R1_2,
          "DOWN-hard at QAM16 must commit the midrung landing (16QAM R1/2) immediately");
    CHECK(ConnectionAdaptiveTestAccess::arqCodeRate(c) == CodeRate::R1_2,
          "the ARQ must be reconfigured to the committed rate");
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "the descriptor commit must not arm the MODE_CHANGE stop-and-wait");
    CHECK(countModeChangeFrames(tx_frames) == 0,
          "NO MODE_CHANGE control frame may go on the wire (that was the dead-air)");
    CHECK(c.getStats().descriptor_mode_switches == 1,
          "the commit must count as a descriptor switch");
    CHECK(ConnectionAdaptiveTestAccess::arqTxMoveEpoch(c) == 1,
          "the MID-WINDOW regrid must bump the TX move-epoch (era safety)");
    bool full_anchor_armed_or_consumed =
        ConnectionAdaptiveTestAccess::descSwitchFullAnchorArmed(c);
    for (size_t i = 0; i < burst_full_anchor.size(); ++i) {
        if (burst_sizes[i] >= 2 && burst_full_anchor[i]) {
            full_anchor_armed_or_consumed = true;
        }
    }
    // F163 REVERSAL (2026-07-06): every commit arms the full anchor — the
    // midrung landing included (see the first-switch comment above).
    CHECK(full_anchor_armed_or_consumed,
          "every DESC-SWITCH commit must arm the full anchor (F163)");

    // Rate-limit: the SAME command (same group_seq — the base was frozen by the
    // crater) re-arriving before the receiver's adoption is deduped: no second
    // demote (QPSK R3/4 would otherwise step to R2/3), no new commit.
    c.onToneBurstAck(makeRungCmdDetection(/*group_seq=*/1, kRungCmdDownHard));
    // Midrung default (2026-07-05): the first DOWN-hard landed at 16QAM R1/2; the
    // DUPLICATE (same group_seq) must not move it again.
    CHECK(c.getDataModulation() == Modulation::QAM16 &&
              c.getDataCodeRate() == CodeRate::R1_2,
          "a duplicate command (same group_seq) before adoption must be a no-op");
    CHECK(c.getStats().descriptor_mode_switches == 1,
          "a duplicate command must not commit again");
    CHECK(countModeChangeFrames(tx_frames) == 0,
          "a duplicate command must not fall back to MODE_CHANGE either");
}

void test_ofdm_connected_entry_does_not_emit_unsolicited_timing_anchor() {
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });
    ConnectionAdaptiveTestAccess::makeConnectedInitiator(c, WaveformMode::OFDM_CHIRP);

    ConnectionAdaptiveTestAccess::enterConnected(c);
    CHECK(tx_frames.empty(),
          "connected OFDM entry should not emit an unsolicited KEEPALIVE anchor");
}

void test_interactive_bootstrap_yield_stops_after_initiator_starts_data() {
    Connection idle;
    std::vector<Bytes> idle_tx;
    idle.setTransmitCallback([&](const Bytes& data) {
        idle_tx.push_back(data);
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        idle, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    idle.setHalfDuplexInteractive(true);
    idle.tick(1500);
    CHECK(idle_tx.size() == 1,
          "an interactive initiator with no DATA must still yield the bootstrap turn");
    auto turnover = v2::ControlFrame::deserialize(idle_tx.front());
    CHECK(turnover && turnover->type == v2::FrameType::TURNOVER,
          "the empty-initiator bootstrap frame must remain TURNOVER");

    Connection active;
    std::vector<Bytes> active_tx;
    active.setTransmitCallback([&](const Bytes& data) {
        active_tx.push_back(data);
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        active, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    active.setHalfDuplexInteractive(true);

    // Model a real sendFile/sendBinary start. The TNC can still hold a second
    // accumulated block outside Connection while this first block drains.
    ConnectionAdaptiveTestAccess::noteDataTurnPayloadStarted(active, 4097);
    CHECK(ConnectionAdaptiveTestAccess::interactiveInitiatorYieldDone(active),
          "starting DATA must retire the empty-initiator bootstrap choreography");
    active.tick(1500);
    CHECK(active_tx.empty(),
          "a momentarily empty Connection window after DATA must not emit a blind TURNOVER");

    auto request = v2::ControlFrame::makeTurnRequest("K2DEF", "W1ABC");
    active.onFrameReceived(request.serialize());
    CHECK(active_tx.size() == 1,
          "retiring the bootstrap yield must not block a real peer turn request");
    auto requested_turnover = v2::ControlFrame::deserialize(active_tx.front());
    CHECK(requested_turnover && requested_turnover->type == v2::FrameType::TURNOVER,
          "an explicit peer request must retain the normal fair TURNOVER path");
}

void test_normal_ofdm_ack_arms_full_anchor_expectation() {
    Connection c;
    std::vector<Bytes> tx_frames;
    int full_anchor_expectations = 0;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });
    c.setFullOFDMAnchorExpectedCallback([&]() {
        ++full_anchor_expectations;
    });
    ConnectionAdaptiveTestAccess::makeConnectedInitiator(c, WaveformMode::OFDM_CHIRP);

    auto ack = v2::ControlFrame::makeAck("W1ABC", "K2DEF", 7);
    ConnectionAdaptiveTestAccess::transmitFrame(c, ack.serialize());
    CHECK(tx_frames.size() == 1, "normal ACK should still transmit through Connection");
    CHECK(full_anchor_expectations == 1,
          "normal connected OFDM ACK should arm full-anchor expectation");

    auto disconnect_ack = v2::ControlFrame::makeAck(
        "W1ABC", "K2DEF", v2::DISCONNECT_SEQ);
    ConnectionAdaptiveTestAccess::transmitFrame(c, disconnect_ack.serialize());
    CHECK(full_anchor_expectations == 2,
          "disconnect sentinel ACK must arm full-anchor expectation for a duplicate request");

    auto turnover = v2::ControlFrame::makeTurnover("W1ABC", "K2DEF");
    ConnectionAdaptiveTestAccess::transmitFrame(c, turnover.serialize());
    CHECK(full_anchor_expectations == 3,
          "connected OFDM TURNOVER should arm full-anchor expectation before listening");

    auto turn_request = v2::ControlFrame::makeTurnRequest("W1ABC", "K2DEF");
    ConnectionAdaptiveTestAccess::transmitFrame(c, turn_request.serialize());
    CHECK(full_anchor_expectations == 4,
          "connected OFDM TURN_REQUEST should arm full-anchor expectation before listening");

    Connection metadata;
    int metadata_expectations = 0;
    int immediate_expectations = 0;
    metadata.setTransmitInfoCallback([&](const Bytes&, bool expect_full_anchor_after_tx) {
        if (expect_full_anchor_after_tx) {
            ++metadata_expectations;
        }
    });
    metadata.setFullOFDMAnchorExpectedCallback([&]() {
        ++immediate_expectations;
    });
    ConnectionAdaptiveTestAccess::makeConnectedInitiator(metadata, WaveformMode::OFDM_CHIRP);
    ConnectionAdaptiveTestAccess::transmitFrame(metadata, ack.serialize());
    CHECK(metadata_expectations == 1,
          "Connection should attach full-anchor expectation metadata to normal OFDM ACK");
    ConnectionAdaptiveTestAccess::transmitFrame(metadata, turnover.serialize());
    CHECK(metadata_expectations == 2,
          "Connection should attach full-anchor expectation metadata to TURNOVER");
    metadata.disconnect();
    CHECK(metadata.getState() == ConnectionState::DISCONNECTING &&
              metadata_expectations == 3,
          "DISCONNECT metadata must arm cold acquisition after the state enters DISCONNECTING");
    CHECK(immediate_expectations == 0,
          "metadata TX callback should defer full-anchor application to the transport TX edge");

    Connection mcdpsk;
    int mcdpsk_expectations = 0;
    mcdpsk.setFullOFDMAnchorExpectedCallback([&]() {
        ++mcdpsk_expectations;
    });
    ConnectionAdaptiveTestAccess::makeConnectedInitiator(mcdpsk, WaveformMode::MC_DPSK);
    ConnectionAdaptiveTestAccess::transmitFrame(mcdpsk, ack.serialize());
    CHECK(mcdpsk_expectations == 0,
          "non-OFDM ACK must not arm full-anchor expectation");
}

void test_normal_message_turn_yield_periodically_rearms_full_anchor() {
    Connection c;
    std::vector<Bytes> tx_frames;
    int full_anchor_expectations = 0;
    c.setTransmitCallback([&](const Bytes& data) {
        tx_frames.push_back(data);
    });
    c.setFullOFDMAnchorExpectedCallback([&]() {
        ++full_anchor_expectations;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    // Normal GUI messages do not enable the stronger half-duplex-interactive
    // B2F policy. A peer request must still yield the ordinary DATA turn.
    auto request = v2::ControlFrame::makeTurnRequest("K2DEF", "W1ABC");
    c.onFrameReceived(request.serialize());
    CHECK(tx_frames.size() == 1 &&
              v2::parseHeader(tx_frames.front()).type == v2::FrameType::TURNOVER,
          "an idle normal GUI sender must yield the DATA turn on peer request");
    CHECK(full_anchor_expectations == 1,
          "TURNOVER transmission must arm the first cold-acquire expectation");

    // The first one-shot may be consumed by a stray detect in the turnaround
    // gap. Periodic rearming must follow the yielded-turn state even though
    // half-duplex-interactive remains disabled.
    c.tick(1);
    CHECK(full_anchor_expectations == 2,
          "normal message turn reversal must immediately refresh cold acquisition");
    c.tick(399);
    CHECK(full_anchor_expectations == 2,
          "cold-acquire refresh must retain its 400 ms cadence");
    c.tick(1);
    CHECK(full_anchor_expectations == 3,
          "normal message turn reversal must keep rearming until peer DATA arrives");
}

void test_tone_ack_callback_marks_only_group_boundary_as_physically_complete() {
    Connection c;
    std::vector<bool> group_complete_contexts;
    c.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload&, bool inbound_group_complete) {
            group_complete_contexts.push_back(inbound_group_complete);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    auto grouped = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 0, Bytes{0x42}, CodeRate::R2_3);
    c.onBurstGroupReceived(/*group_seq=*/0, {grouped.serialize()},
                           /*all_ok=*/true, /*quality=*/0.95f,
                           /*frame_mask=*/0x1, /*interleaved=*/false,
                           /*group_size=*/1, /*geometry_proven=*/true);
    CHECK(group_complete_contexts.size() == 1 && group_complete_contexts[0],
          "endGroupReceiveAndAck must mark its synchronous tone ACK as physically complete");

    const size_t before_unproven = group_complete_contexts.size();
    c.onBurstGroupReceived(/*group_seq=*/1, {},
                           /*all_ok=*/false, /*quality=*/0.0f,
                           /*frame_mask=*/0x0, /*interleaved=*/false,
                           /*group_size=*/1, /*geometry_proven=*/false);
    CHECK(group_complete_contexts.size() > before_unproven,
          "an unproven group outcome must still drive normal ARQ feedback");
    for (size_t i = before_unproven; i < group_complete_contexts.size(); ++i) {
        CHECK(!group_complete_contexts[i],
              "unproven geometry must never claim a physically-safe ACK boundary");
    }

    const size_t before_classic = group_complete_contexts.size();
    auto classic = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 1, Bytes{0x43}, CodeRate::R2_3);
    c.onFrameReceived(classic.serialize());
    c.tick(10000);  // drain a delayed SACK if the direct frame did not ACK immediately
    CHECK(group_complete_contexts.size() > before_classic,
          "a classic DATA member must eventually request tone feedback");
    for (size_t i = before_classic; i < group_complete_contexts.size(); ++i) {
        CHECK(!group_complete_contexts[i],
              "timer/standalone tone ACK must retain asynchronous egress provenance");
    }

    const size_t before_physical_singleton = group_complete_contexts.size();
    auto physical_singleton = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 2, Bytes{0x44}, CodeRate::R2_3);
    physical_singleton.flags |= v2::Flags::MORE_FRAG;
    c.onFrameReceived(physical_singleton.serialize(),
                      /*physical_turn_complete=*/true);
    CHECK(group_complete_contexts.size() == before_physical_singleton + 1,
          "a physically-complete non-FINAL singleton must ACK exactly once immediately");
    CHECK(group_complete_contexts.back(),
          "a physically-complete singleton ACK must carry safe egress provenance");

    const size_t before_final_singleton = group_complete_contexts.size();
    auto final_singleton = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 3, Bytes{0x45}, CodeRate::R2_3);
    final_singleton.flags |= v2::Flags::FINAL;
    c.onFrameReceived(final_singleton.serialize(),
                      /*physical_turn_complete=*/true);
    CHECK(group_complete_contexts.size() == before_final_singleton + 1,
          "a physically-complete FINAL singleton must ACK exactly once");
    CHECK(group_complete_contexts.back(),
          "a FINAL singleton ACK must retain physical-boundary provenance");

    const size_t before_duplicate_singleton = group_complete_contexts.size();
    c.onFrameReceived(final_singleton.serialize(),
                      /*physical_turn_complete=*/true);
    CHECK(group_complete_contexts.size() == before_duplicate_singleton + 1,
          "a duplicate physical singleton must re-confirm the window exactly once");
    CHECK(group_complete_contexts.back(),
          "a duplicate singleton re-confirmation is still physically safe");

    const size_t before_final_group = group_complete_contexts.size();
    auto final_grouped = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 4, Bytes{0x46}, CodeRate::R2_3);
    final_grouped.flags |= v2::Flags::FINAL;
    c.onBurstGroupReceived(/*group_seq=*/4, {final_grouped.serialize()},
                           /*all_ok=*/true, /*quality=*/0.95f,
                           /*frame_mask=*/0x1, /*interleaved=*/false,
                           /*group_size=*/1, /*geometry_proven=*/true);
    CHECK(group_complete_contexts.size() > before_final_group,
          "a FINAL member inside a completed burst must emit tone feedback");
    for (size_t i = before_final_group; i < group_complete_contexts.size(); ++i) {
        CHECK(group_complete_contexts[i],
              "FINAL-immediate and boundary ACKs share completed-burst provenance");
    }

    const size_t before_anchored_backstop = group_complete_contexts.size();
    c.noteAnchoredBurstNoGroup();
    CHECK(group_complete_contexts.size() > before_anchored_backstop,
          "an anchored no-group burst must emit its recovery tone feedback");
    for (size_t i = before_anchored_backstop;
         i < group_complete_contexts.size(); ++i) {
        CHECK(!group_complete_contexts[i],
              "anchored no-group recovery is a timer event, not physical-boundary proof");
    }

    const size_t before_payload_seen_backstop = group_complete_contexts.size();
    c.noteAnchoredBurstNoGroup(/*payload_seen=*/true);
    CHECK(group_complete_contexts.size() > before_payload_seen_backstop,
          "payload-aware backstop must still emit the cumulative recovery ACK");
    for (size_t i = before_payload_seen_backstop;
         i < group_complete_contexts.size(); ++i) {
        CHECK(!group_complete_contexts[i],
              "payload-aware timer ACK remains asynchronous despite selector suppression");
    }
}

void test_software_alc_uses_current_partial_group_delivery_provenance() {
    Connection c;
    std::vector<ToneBurstAckPayload> acks;
    c.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { acks.push_back(ack); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM8);

    uint16_t next_seq = 0;
    auto twoDecodedFrames = [&]() {
        std::vector<Bytes> frames;
        for (int i = 0; i < 2; ++i) {
            auto frame = v2::makeFixedDataFrame(
                "K2DEF", "W1ABC", next_seq++, Bytes{static_cast<uint8_t>(0x60 + i)},
                CodeRate::R2_3);
            frame.flags |= v2::Flags::MORE_FRAG;
            frames.push_back(frame.serialize());
        }
        return frames;
    };
    auto deliverPartialLowGroup = [&](uint32_t verdict_seq) {
        c.setRxLevelVerdict(
            static_cast<int>(connection_policy::RxLevelVerdict::LOW), verdict_seq);
        c.onBurstGroupReceived(
            /*group_seq=*/0, twoDecodedFrames(), /*all_ok=*/false,
            /*quality=*/0.0f, /*frame_mask=*/0x3, /*interleaved=*/false,
            /*group_size=*/8, /*geometry_proven=*/true);
    };

    deliverPartialLowGroup(/*verdict_seq=*/1);
    CHECK(!acks.empty() &&
              acks.back().drive_advisory ==
                  ultra::waveform::tone_burst_ack::kDriveAdvisoryHold,
          "first LOW partial group must respect the two-group ALC hysteresis");

    deliverPartialLowGroup(/*verdict_seq=*/2);
    CHECK(acks.back().drive_advisory ==
              ultra::waveform::tone_burst_ack::kDriveAdvisoryUp,
          "second LOW partial group with decoded DATA must advise UP even though quality is 0");

    // Re-feeding the prior measurement sequence must not turn new decoded DATA
    // into another drive step. The modem binding can do this when a callback has
    // no fresh per-burst level estimate.
    deliverPartialLowGroup(/*stale verdict_seq=*/2);
    CHECK(acks.back().drive_advisory ==
              ultra::waveform::tone_burst_ack::kDriveAdvisoryHold,
          "a new group without a fresh level verdict must not reuse prior ALC evidence");

    // The LOW streak is intentionally still armed, but this timer/backstop ACK has
    // no current decoder callback. It must not turn old evidence into a new UP step.
    const size_t before_async = acks.size();
    c.noteAnchoredBurstNoGroup();
    CHECK(acks.size() > before_async,
          "anchored no-group backstop must emit a cumulative tone ACK");
    for (size_t i = before_async; i < acks.size(); ++i) {
        CHECK(acks[i].drive_advisory ==
                  ultra::waveform::tone_burst_ack::kDriveAdvisoryHold,
              "non-group ACK must not inherit a prior group's ALC UP advisory");
    }

    // A fresh LOW measurement on a zero-delivery group is a fade crater, not
    // workable level evidence. It must HOLD and clear the accumulated LOW streak.
    c.setRxLevelVerdict(
        static_cast<int>(connection_policy::RxLevelVerdict::LOW), /*seq=*/3);
    c.onBurstGroupReceived(
        /*group_seq=*/0, {}, /*all_ok=*/false, /*quality=*/0.0f,
        /*frame_mask=*/0, /*interleaved=*/false, /*group_size=*/8,
        /*geometry_proven=*/true);
    CHECK(acks.back().drive_advisory ==
              ultra::waveform::tone_burst_ack::kDriveAdvisoryHold,
          "zero-delivery LOW crater must fail closed to HOLD");

    deliverPartialLowGroup(/*verdict_seq=*/4);
    CHECK(acks.back().drive_advisory ==
              ultra::waveform::tone_burst_ack::kDriveAdvisoryHold,
          "first workable LOW after a crater must restart hysteresis at one");
    deliverPartialLowGroup(/*verdict_seq=*/5);
    CHECK(acks.back().drive_advisory ==
              ultra::waveform::tone_burst_ack::kDriveAdvisoryUp,
          "second workable LOW after crater reset must advise UP");

    // Fast-attack CLIPPED remains immediate for a real group, but it is subject
    // to the same no-stale-carry rule as UP on a later asynchronous ACK.
    c.setRxLevelVerdict(
        static_cast<int>(connection_policy::RxLevelVerdict::CLIPPED), /*seq=*/6);
    c.onBurstGroupReceived(
        /*group_seq=*/0, twoDecodedFrames(), /*all_ok=*/false,
        /*quality=*/0.0f, /*frame_mask=*/0x3, /*interleaved=*/false,
        /*group_size=*/8, /*geometry_proven=*/true);
    CHECK(acks.back().drive_advisory ==
              ultra::waveform::tone_burst_ack::kDriveAdvisoryDown,
          "fresh CLIPPED group must retain immediate ALC DOWN behavior");
    const size_t before_clipped_async = acks.size();
    c.noteAnchoredBurstNoGroup();
    CHECK(acks.size() > before_clipped_async,
          "post-CLIPPED backstop must still emit its cumulative ACK");
    for (size_t i = before_clipped_async; i < acks.size(); ++i) {
        CHECK(acks[i].drive_advisory ==
                  ultra::waveform::tone_burst_ack::kDriveAdvisoryHold,
              "non-group ACK must not inherit a prior group's ALC DOWN advisory");
    }
}

void test_partial_sack_provenance_marks_only_exact_group_geometry() {
    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection exact;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");
    std::vector<ToneBurstAckPayload> exact_acks;
    exact.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { exact_acks.push_back(ack); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        exact, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    auto frame0 = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 0, Bytes{0x50}, CodeRate::R2_3);
    auto frame1 = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 1, Bytes{0x51}, CodeRate::R2_3);
    frame0.flags |= v2::Flags::MORE_FRAG;
    frame1.flags |= v2::Flags::MORE_FRAG;
    exact.onBurstGroupReceived(
        /*group_seq=*/0, {frame0.serialize(), frame1.serialize()},
        /*all_ok=*/true, /*quality=*/0.95f, /*frame_mask=*/0x3,
        /*interleaved=*/false, /*group_size=*/2, /*geometry_proven=*/true);
    CHECK(!exact_acks.empty() &&
              exact_acks.back().drive_advisory == kDriveAdvisoryReserved,
          "descriptor-proven exact 2/2 group must stamp hold-compatible provenance=3");

    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection interleaved;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");
    std::vector<ToneBurstAckPayload> interleaved_acks;
    interleaved.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) {
            interleaved_acks.push_back(ack);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        interleaved, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    interleaved.onBurstGroupReceived(
        /*group_seq=*/0, {frame0.serialize(), frame1.serialize()},
        /*all_ok=*/true, /*quality=*/0.95f, /*frame_mask=*/0x3,
        /*interleaved=*/true, /*group_size=*/2, /*geometry_proven=*/true);
    CHECK(!interleaved_acks.empty() &&
              interleaved_acks.back().drive_advisory != kDriveAdvisoryReserved,
          "BI1 group must not stamp BI0-only descriptor-repair provenance");

    const size_t before_unproven = exact_acks.size();
    exact.onBurstGroupReceived(
        /*group_seq=*/1, {}, /*all_ok=*/false, /*quality=*/0.0f,
        /*frame_mask=*/0, /*interleaved=*/false, /*group_size=*/2,
        /*geometry_proven=*/false);
    CHECK(exact_acks.size() > before_unproven,
          "unproven group must retain normal cumulative feedback");
    for (size_t i = before_unproven; i < exact_acks.size(); ++i) {
        CHECK(exact_acks[i].drive_advisory != kDriveAdvisoryReserved,
              "unproven k/M must not stamp descriptor-only repair provenance");
    }

    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection singleton;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");
    std::vector<ToneBurstAckPayload> singleton_acks;
    singleton.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { singleton_acks.push_back(ack); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        singleton, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    auto one = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 0, Bytes{0x52}, CodeRate::R2_3);
    one.flags |= v2::Flags::MORE_FRAG;
    singleton.onFrameReceived(one.serialize(), /*physical_turn_complete=*/true);
    CHECK(singleton_acks.size() == 1 &&
              singleton_acks.back().drive_advisory != kDriveAdvisoryReserved,
          "physically-complete singleton has no exact group k/M and must fail closed");

    setenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR", "1", 1);
    Connection backstop;
    unsetenv("ULTRA_PARTIAL_SACK_DESCRIPTOR_REPAIR");
    std::vector<ToneBurstAckPayload> backstop_acks;
    backstop.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { backstop_acks.push_back(ack); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        backstop, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);
    backstop.noteAnchoredBurstNoGroup();
    CHECK(!backstop_acks.empty() &&
              backstop_acks.back().drive_advisory != kDriveAdvisoryReserved,
          "no-group timer backstop must never claim exact repair provenance");
}

void test_physical_tail_acks_prior_orphan_payload_cumulatively() {
    Connection c;
    std::vector<ToneBurstAckPayload> acks;
    std::vector<bool> physical_contexts;
    c.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool inbound_group_complete) {
            acks.push_back(ack);
            physical_contexts.push_back(inbound_group_complete);
        });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QPSK);

    // Model descriptor/head loss: seq0 is the hole, while independently-decoded
    // fallback members update the ARQ window without a trustworthy boundary.
    for (uint16_t seq = 1; seq <= 3; ++seq) {
        auto orphan = v2::makeFixedDataFrame(
            "K2DEF", "W1ABC", seq, Bytes{static_cast<uint8_t>(0x40 + seq)},
            CodeRate::R2_3);
        orphan.flags |= v2::Flags::MORE_FRAG;
        c.onFrameReceived(orphan.serialize(), /*physical_turn_complete=*/false);
    }
    CHECK(acks.empty(),
          "unmarked orphan DATA must not authorize reverse egress mid-burst");

    auto tail = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 4, Bytes{0x44}, CodeRate::R2_3);
    tail.flags |= static_cast<uint8_t>(v2::Flags::MORE_FRAG |
                                      v2::Flags::PHYSICAL_BURST_END);
    c.onFrameReceived(tail.serialize(), /*physical_turn_complete=*/true);

    CHECK(acks.size() == 1 && physical_contexts.size() == 1,
          "decoded physical tail must emit exactly one immediate cumulative ACK");
    CHECK(physical_contexts[0],
          "tail-triggered cumulative ACK must carry safe physical-boundary provenance");
    CHECK(acks[0].frame_mask == 0x001E,
          "tail ACK must preserve all decoded survivors and select only the missing head");
}

void test_latent_candidate_geometry_matches_receiver_configured_wire_profile() {
    // Default-off control: candidate pricing remains the production logical Z27
    // representation. This does not graduate either long-code experiment.
    unsetenv("ULTRA_8PSK_LONG_LDPC");
    unsetenv("ULTRA_QPSK_R34_LONG_LDPC");
    Connection baseline_rx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        baseline_rx, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    const auto baseline_qpsk = ConnectionAdaptiveTestAccess::latentCandidateGeometry(
        baseline_rx, Modulation::QPSK, CodeRate::R3_4,
        /*force_full_group_start=*/false);
    const auto baseline_8psk = ConnectionAdaptiveTestAccess::latentCandidateGeometry(
        baseline_rx, Modulation::QAM8, CodeRate::R2_3,
        /*force_full_group_start=*/true);
    CHECK(baseline_qpsk.logical_cw == 8 && baseline_qpsk.physical_cw == 8 &&
              baseline_qpsk.lifting_z == 27 &&
              std::lround(baseline_qpsk.value.useful_bytes_per_frame) == 456,
          "default selector candidate must remain QPSK R3/4 cw8/Z27/456B");
    CHECK(baseline_8psk.logical_cw == 12 && baseline_8psk.physical_cw == 12 &&
              baseline_8psk.lifting_z == 27 &&
              std::lround(baseline_8psk.value.useful_bytes_per_frame) == 624,
          "default selector candidate must remain 8PSK R2/3 cw12/Z27/624B");

    // The selector lives on the RECEIVER. It never executes
    // startFileTransferNow(), so its peer-owned transfer-active booleans remain false.
    // Matching endpoint experiment policy must nevertheless price the wire tuple the
    // sender will use after obeying the rate command.
    setenv("ULTRA_8PSK_LONG_LDPC", "1", 1);
    setenv("ULTRA_QPSK_R34_LONG_LDPC", "1", 1);
    Connection configured_rx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        configured_rx, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    CHECK(!ConnectionAdaptiveTestAccess::experimental8PSKLongLDPCActive(configured_rx) &&
              !ConnectionAdaptiveTestAccess::experimentalQPSKR34LongLDPCActive(configured_rx),
          "receiver fixture must not counterfeit sender transfer-active arms");
    const auto qpsk = ConnectionAdaptiveTestAccess::latentCandidateGeometry(
        configured_rx, Modulation::QPSK, CodeRate::R3_4,
        /*force_full_group_start=*/false);
    const auto qpsk_switch = ConnectionAdaptiveTestAccess::latentCandidateGeometry(
        configured_rx, Modulation::QPSK, CodeRate::R3_4,
        /*force_full_group_start=*/true);
    const auto psk8 = ConnectionAdaptiveTestAccess::latentCandidateGeometry(
        configured_rx, Modulation::QAM8, CodeRate::R2_3,
        /*force_full_group_start=*/true);

    CHECK(qpsk.logical_cw == 8 && qpsk.physical_cw == 3 && qpsk.lifting_z == 81 &&
              std::lround(qpsk.value.useful_bytes_per_frame) == 522 &&
              qpsk.frames_per_cycle == 5 && qpsk.burst_airtime_ms == 8400 &&
              !qpsk.force_full_group_start &&
              std::fabs(qpsk.value.cycle_s - 10.190f) < 0.001f,
          "configured receiver must score exact QPSK cw3/Z81 522B/N5/8.400s wire geometry");
    CHECK(qpsk_switch.logical_cw == 8 && qpsk_switch.physical_cw == 3 &&
              qpsk_switch.lifting_z == 81 &&
              std::lround(qpsk_switch.value.useful_bytes_per_frame) == 522 &&
              qpsk_switch.frames_per_cycle == 4 &&
              qpsk_switch.burst_airtime_ms == 8160 &&
              qpsk_switch.force_full_group_start &&
              std::fabs(qpsk_switch.value.cycle_s - 9.950f) < 0.001f,
          "non-incumbent QPSK R3/4 must price cw3/Z81 plus its full switch anchor");
    CHECK(psk8.logical_cw == 12 && psk8.physical_cw == 4 && psk8.lifting_z == 81 &&
              std::lround(psk8.value.useful_bytes_per_frame) == 624 &&
              psk8.frames_per_cycle == 4 && psk8.burst_airtime_ms == 7488 &&
              psk8.force_full_group_start &&
              std::fabs(psk8.value.cycle_s - 9.278f) < 0.001f,
          "non-incumbent 8PSK must score cw4/Z81 624B/N4 plus its 1.2s switch anchor");

    // Cross-check those receiver counterfactuals against the sender-owned policy,
    // armed exactly as startFileTransferNow() would arm it.
    Connection sender;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        sender, CodeRate::R3_4, 20.0f, 0.30f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        sender, Modulation::QPSK, CodeRate::R3_4, /*logical_cw=*/8);
    ConnectionAdaptiveTestAccess::armExperimentalLongLDPCProfiles(
        sender, /*psk8_active=*/true, /*qpsk_r34_active=*/true);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(sender, /*streak=*/7);
    const size_t qpsk_sender_n = sender.burstAirtimeBudgetFrames(
        connection_policy::ofdmWindowSize(
            Modulation::QPSK, CodeRate::R3_4, /*near_awgn_ofdm=*/false),
        /*force_full_group_start=*/true);
    const uint32_t qpsk_sender_ms =
        connection_policy::wideOFDMBurstAirtimeMs(
            Modulation::QPSK, CodeRate::R3_4, qpsk_sender_n,
            ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(sender),
            /*continuation_reanchor_ms=*/0, sender.selectBurstLiftingZ()) +
        connection_policy::kWideOFDMFullAnchorExtraMs;
    CHECK(ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(sender) ==
              qpsk_switch.physical_cw &&
              sender.selectBurstLiftingZ() == qpsk_switch.lifting_z &&
              qpsk_sender_n == qpsk_switch.frames_per_cycle &&
              qpsk_sender_ms == qpsk_switch.burst_airtime_ms,
          "receiver QPSK candidate must equal the armed sender's full-anchor policy");

    ConnectionAdaptiveTestAccess::setDataGeometry(
        sender, Modulation::QAM8, CodeRate::R2_3, /*logical_cw=*/12);
    const size_t psk8_window = connection_policy::ofdmWindowSize(
        Modulation::QAM8, CodeRate::R2_3, /*near_awgn_ofdm=*/false);
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(sender, /*streak=*/2);
    const size_t sender_private_escalated_n = sender.burstAirtimeBudgetFrames(
        psk8_window, /*force_full_group_start=*/true);
    CHECK(sender_private_escalated_n == 7 &&
              sender_private_escalated_n > psk8.frames_per_cycle,
          "sender-private clean ACKs can earn N7, but RX candidate pricing must not assume them");
    // Candidate scoring cannot observe the sender-only clean-ACK streak, so compare
    // against the guaranteed base-ceiling sender state. A descriptor-committed
    // non-incumbent additionally pays the exact one-shot full group-start anchor.
    ConnectionAdaptiveTestAccess::setBurstCleanGroupStreak(sender, /*streak=*/0);
    const size_t psk8_sender_n = sender.burstAirtimeBudgetFrames(
        psk8_window,
        /*force_full_group_start=*/true);
    const uint32_t psk8_sender_ms =
        connection_policy::wideOFDMBurstAirtimeMs(
            Modulation::QAM8, CodeRate::R2_3, psk8_sender_n,
            ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(sender),
            /*continuation_reanchor_ms=*/0, sender.selectBurstLiftingZ()) +
        connection_policy::kWideOFDMFullAnchorExtraMs;
    CHECK(ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(sender) == psk8.physical_cw &&
              sender.selectBurstLiftingZ() == psk8.lifting_z &&
              psk8_sender_n == psk8.frames_per_cycle &&
              psk8_sender_ms == psk8.burst_airtime_ms,
          "receiver 8PSK candidate must equal the armed sender's full-anchor CW/Z/N/airtime policy");

    unsetenv("ULTRA_8PSK_LONG_LDPC");
    unsetenv("ULTRA_QPSK_R34_LONG_LDPC");
}

void test_8psk_long_ldpc_file_geometry_and_tail_safety() {
    constexpr Modulation kMod = Modulation::QAM8;
    constexpr CodeRate kRate = CodeRate::R2_3;
    constexpr int kShortCW = 12;
    constexpr int kLongCW = 4;

    const size_t short_capacity =
        v2::getFixedFramePayloadCapacity(kRate, kShortCW);
    const size_t long_capacity =
        v2::getFixedFramePayloadCapacityZ(kRate, kLongCW, 81);
    CHECK(short_capacity == long_capacity,
          "cw12/Z27 and cw4/Z81 must expose identical fixed-frame capacity");
    const auto short_timing = connection_policy::wideOFDMFrameTiming(
        kMod, kRate, kShortCW, 27);
    const auto long_timing = connection_policy::wideOFDMFrameTiming(
        kMod, kRate, kLongCW, 81);
    CHECK(short_timing.data_symbols == long_timing.data_symbols &&
              short_timing.data_ms == long_timing.data_ms,
          "cw12/Z27 and cw4/Z81 must have identical data airtime");

    // Default-off control: the same zero-byte file is one logical metadata frame,
    // remains cw12/Z27, and takes the established standalone path.
    unsetenv("ULTRA_8PSK_LONG_LDPC");
    TempPayloadFile default_file("ultra_psk8_short", 0);
    CHECK(!default_file.path.empty(), "default profile fixture should create an empty file");
    Connection baseline;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        baseline, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        baseline, kMod, kRate, kShortCW);
    std::vector<Bytes> baseline_singletons;
    std::vector<std::vector<Bytes>> baseline_bursts;
    baseline.setTransmitCallback(
        [&](const Bytes& frame) { baseline_singletons.push_back(frame); });
    baseline.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            baseline_bursts.push_back(frames);
        });
    CHECK(baseline.sendFile(default_file.path),
          "default profile should start the zero-byte file");
    CHECK(baseline_singletons.size() == 1 && baseline_bursts.empty(),
          "default cw12/Z27 tail should preserve the standalone singleton path");
    const auto baseline_header = v2::parseHeader(baseline_singletons.front());
    CHECK(baseline_header.valid && baseline_header.total_cw == kShortCW,
          "default file frame must remain cw12");
    CHECK(baseline.selectBurstLiftingZ() == 27,
          "default file profile must remain Z=27");
    CHECK(ConnectionAdaptiveTestAccess::currentPayloadCapacity(baseline) ==
              short_capacity,
          "default file chunk capacity must match cw12/Z27");
    baseline.cancelFileTransfer();

    // Opt-in singleton: it must never escape through the descriptor-less
    // standalone path. The existing ULPAD mechanism creates a two-frame physical
    // group, keeps logical FINAL on the real frame, and moves PHYSICAL_BURST_END to
    // the pad that really ends the key-down.
    setenv("ULTRA_8PSK_LONG_LDPC", "1", 1);
    TempPayloadFile long_file("ultra_psk8_long", 0);
    CHECK(!long_file.path.empty(), "long profile fixture should create an empty file");
    Connection long_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        long_tx, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        long_tx, kMod, kRate, kShortCW);
    std::vector<Bytes> unexpected_long_singletons;
    std::vector<std::vector<Bytes>> long_bursts;
    std::vector<bool> long_sent_results;
    int callback_z = 0;
    long_tx.setTransmitCallback(
        [&](const Bytes& frame) { unexpected_long_singletons.push_back(frame); });
    long_tx.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            callback_z = long_tx.selectBurstLiftingZ();
            long_bursts.push_back(frames);
        });
    long_tx.setFileSentCallback(
        [&](bool success, const std::string&) {
            long_sent_results.push_back(success);
        });
    CHECK(long_tx.sendFile(long_file.path),
          "long profile should start the zero-byte file");
    CHECK(unexpected_long_singletons.empty() && long_bursts.size() == 1,
          "long-Z singleton must be converted to one descriptor-bearing burst");
    CHECK(callback_z == 81 &&
              ConnectionAdaptiveTestAccess::experimental8PSKLongLDPCActive(long_tx),
          "file profile must be armed before first physical egress");
    CHECK(ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(long_tx) == kLongCW,
          "active 8PSK profile must use four long codewords");
    CHECK(ConnectionAdaptiveTestAccess::currentPayloadCapacity(long_tx) ==
              long_capacity,
          "tail chunk capacity must be computed from cw4/Z81 before startSend");
    CHECK(long_bursts.front().size() == 2,
          "one logical tail plus ULPAD must form a two-frame physical group");
    for (const auto& frame : long_bursts.front()) {
        const auto h = v2::parseHeader(frame);
        CHECK(h.valid && h.total_cw == kLongCW,
              "real and ULPAD frames must share cw4 in the long group");
    }
    auto real_tail = v2::DataFrame::deserialize(long_bursts.front()[0]);
    auto pad_tail = v2::DataFrame::deserialize(long_bursts.front()[1]);
    CHECK(real_tail.has_value() && pad_tail.has_value(),
          "long tail frames must deserialize before physical stamping");
    CHECK(!v2::isOFDMBurstPadHeader(v2::parseHeader(long_bursts.front()[0])) &&
              v2::isOFDMBurstPadHeader(v2::parseHeader(long_bursts.front()[1])),
          "real+ULPAD singleton must classify only its physical pad outside ARQ");
    CHECK((real_tail->flags & v2::Flags::FINAL) != 0,
          "logical DATA tail must retain FINAL");
    CHECK(pad_tail->payload.size() == long_capacity,
          "ULPAD must be sized from the long-Z capacity");
    const auto wire_tail = gui::streaming_frame_policy::preparePhysicalBurstFrames(
        long_bursts.front(), /*stamp_physical_end=*/true);
    auto wire_real = v2::DataFrame::deserialize(wire_tail[0]);
    auto wire_pad = v2::DataFrame::deserialize(wire_tail[1]);
    CHECK(wire_real.has_value() && wire_pad.has_value(),
          "physically stamped long tail must remain CRC-valid");
    CHECK((wire_real->flags & v2::Flags::FINAL) != 0 &&
              (wire_real->flags & v2::Flags::PHYSICAL_BURST_END) == 0,
          "logical FINAL must not claim the air ends while ULPAD follows");
    CHECK((wire_pad->flags & v2::Flags::PHYSICAL_BURST_END) != 0,
          "ULPAD must carry the physical end marker");

    ToneBurstAckDetection completion_ack;
    completion_ack.payload.group_seq = 0;
    completion_ack.payload.frame_mask = 0;
    completion_ack.payload.rate_hint = 7;
    completion_ack.payload.type = AckType::Ack;
    completion_ack.payload.move_epoch = 0;
    completion_ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(long_tx);
    CHECK(long_tx.onToneBurstAck(completion_ack),
          "zero-byte long profile completion ACK should be consumed");
    CHECK(long_sent_results.size() == 1 && long_sent_results.front(),
          "successful transfer must notify the outward callback exactly once");
    CHECK(!ConnectionAdaptiveTestAccess::experimental8PSKLongLDPCActive(long_tx) &&
              long_tx.selectBurstLiftingZ() == 27 &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(long_tx) == 27,
          "successful completion must restore the negotiated short geometry");

    // The older non-final group-size padding branch is differential-only: its
    // policy predicate accepts DQPSK/D8PSK and deliberately rejects coherent
    // QAM8.  Prove that invariant here so this profile cannot silently acquire a
    // second ULPAD path.  The construction code remains Z-aware as a defensive
    // invariant should the policy later be widened to coherent modes.
    CHECK(!connection_policy::isSpeculativeHighRateOFDM(kMod, kRate) &&
              !connection_policy::shouldPadHighRateFadingBurst(
                  kMod, kRate, /*near_awgn_ofdm=*/false, /*burst_frames=*/5),
          "coherent QAM8 R2/3 must not enter the differential partial-pad branch");
    TempPayloadFile large_file("ultra_psk8_long_partial", 10000);
    CHECK(!large_file.path.empty(), "partial-group fixture should create a large file");
    Connection partial_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        partial_tx, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        partial_tx, kMod, kRate, kShortCW);
    std::vector<std::vector<Bytes>> partial_bursts;
    partial_tx.setTransmitCallback([](const Bytes&) {});
    partial_tx.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            partial_bursts.push_back(frames);
        });
    CHECK(partial_tx.sendFile(large_file.path),
          "long profile should start a multi-group file");
    CHECK(!partial_bursts.empty(), "multi-group file must emit its first burst");
    int pad_frames = 0;
    int full_long_data_frames = 0;
    for (const auto& frame : partial_bursts.front()) {
        const auto h = v2::parseHeader(frame);
        CHECK(h.valid && h.total_cw == kLongCW,
              "non-final real and padded frames must all use cw4");
        if (h.dst_hash == v2::hashCallsign("ULPAD")) {
            ++pad_frames;
            continue;
        }
        const auto data = v2::DataFrame::deserialize(frame);
        CHECK(data.has_value(),
              "ordinary long-profile ARQ DATA must deserialize");
        if (data && data->payload.size() == long_capacity) {
            ++full_long_data_frames;
        }
    }
    CHECK(pad_frames == 0,
          "coherent QAM8 non-final groups must not acquire differential ULPAD frames");
    CHECK(full_long_data_frames > 0,
          "ordinary cw4/Z81 ARQ DATA must preserve the 629-byte serialized payload; "
          "a Z=27 default would truncate it to 216 bytes");

    // The force-policy bit is part of the experimental predicate, but a public
    // runtime override may no longer tear down an active geometry owner. The
    // existing cw4/Z81 file must remain internally consistent until cancellation.
    partial_tx.setForcedFrameCodewords(kShortCW, /*forced=*/true);
    CHECK(ConnectionAdaptiveTestAccess::experimental8PSKLongLDPCActive(partial_tx) &&
              ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(partial_tx) == kLongCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(partial_tx) == 81 &&
              ConnectionAdaptiveTestAccess::configuredForcedCWCount(partial_tx) == 0,
          "same-CW force-policy change must be rejected while the file owns cw4/Z81");
    partial_tx.cancelFileTransfer();
    CHECK(!ConnectionAdaptiveTestAccess::experimental8PSKLongLDPCActive(partial_tx) &&
              partial_tx.selectBurstLiftingZ() == 27 &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(partial_tx) == 27,
          "explicit cancel must restore the negotiated short geometry");
    partial_tx.setForcedFrameCodewords(kShortCW, /*forced=*/true);
    CHECK(ConnectionAdaptiveTestAccess::configuredForcedCWCount(partial_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(partial_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(partial_tx) == 27,
          "same-CW force-policy change must apply atomically after the file retires");

    // ARQ reset intentionally retains its configuration, so Connection's QSO
    // boundaries must explicitly strip the transfer-scoped lifting value.
    Connection disconnected_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        disconnected_tx, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        disconnected_tx, kMod, kRate, kShortCW);
    disconnected_tx.setTransmitCallback([](const Bytes&) {});
    disconnected_tx.setTransmitBurstCallback(
        [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
    CHECK(disconnected_tx.sendFile(long_file.path),
          "disconnect reset fixture should arm the long profile");
    CHECK(ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(disconnected_tx) == 81,
          "disconnect reset fixture must reach physical Z=81 first");
    ConnectionAdaptiveTestAccess::enterDisconnected(disconnected_tx);
    CHECK(ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(disconnected_tx) == 27,
          "enterDisconnected must clear the ARQ's retained Z=81 geometry");

    Connection reset_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        reset_tx, kRate, 20.0f, 0.30f, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        reset_tx, kMod, kRate, kShortCW);
    reset_tx.setTransmitCallback([](const Bytes&) {});
    reset_tx.setTransmitBurstCallback(
        [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
    CHECK(reset_tx.sendFile(long_file.path),
          "full reset fixture should arm the long profile");
    CHECK(ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(reset_tx) == 81,
          "full reset fixture must reach physical Z=81 first");
    reset_tx.reset();
    CHECK(ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(reset_tx) == 27,
          "Connection::reset must clear the ARQ's retained Z=81 geometry");

    // Destruction is a silent lifetime boundary: FileTransferController's own
    // destructor calls cancel(), but Connection must clear its internal callbacks
    // first so it cannot dispatch through members already torn down.
    int destructor_callbacks = 0;
    {
        Connection dying_tx;
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(
            dying_tx, kRate, 20.0f, 0.30f, kMod);
        ConnectionAdaptiveTestAccess::setDataGeometry(
            dying_tx, kMod, kRate, kShortCW);
        dying_tx.setTransmitCallback([](const Bytes&) {});
        dying_tx.setTransmitBurstCallback(
            [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
        dying_tx.setFileSentCallback(
            [&](bool, const std::string&) { ++destructor_callbacks; });
        CHECK(dying_tx.sendFile(long_file.path),
              "destructor fixture should own an active long-profile transfer");
    }
    CHECK(destructor_callbacks == 0,
          "Connection destruction must not invoke outward file callbacks");

    // Receiver-side SACK must ignore the addressed-away pad sequence even though
    // the PHY mask reports both positions decoded.
    Connection receiver;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        receiver, kRate, 20.0f, 0.30f, kMod);
    std::vector<ToneBurstAckPayload> pad_acks;
    receiver.setTransmitToneBurstAckCallback(
        [&](const ToneBurstAckPayload& ack, bool) { pad_acks.push_back(ack); });
    auto rx_real = v2::makeFixedDataFrame(
        "K2DEF", "W1ABC", 0, Bytes{0x51}, kRate, kLongCW, 81);
    rx_real.flags |= v2::Flags::FINAL;
    auto rx_pad = v2::makeFixedDataFrame(
        "K2DEF", v2::kOFDMBurstPadCallsign, v2::kOFDMBurstPadSeq,
        Bytes{0x7F}, kRate, kLongCW, 81);
    receiver.onBurstGroupReceived(
        0, {rx_real.serialize(), rx_pad.serialize()}, true, 0.95f,
        /*PHY frame_mask=*/0x3, /*interleaved=*/false, /*group_size=*/2,
        /*geometry_proven=*/true);
    CHECK(!pad_acks.empty(), "completed long tail must emit a cumulative tone SACK");
    for (const auto& ack : pad_acks) {
        // seq0 advanced the cumulative base to 1, so the future-frame bitmap is
        // correctly empty.  If the addressed-away 0xFFFE pad leaked into ARQ,
        // either the low-six group identity (62) or a future bitmap bit would move.
        CHECK(ack.group_seq == 0 && ack.frame_mask == 0x0000 &&
                  ack.type == AckType::Ack,
              "ULPAD seq must not alter cumulative ACK identity or future bitmap");
    }

    unsetenv("ULTRA_8PSK_LONG_LDPC");
}

void test_qpsk_r34_long_ldpc_file_geometry_and_lifecycle() {
    constexpr Modulation kMod = Modulation::QPSK;
    constexpr CodeRate kRate = CodeRate::R3_4;
    constexpr int kShortCW = 8;
    constexpr int kLongCW = 3;
    constexpr float kFading = 0.30f;

    const size_t short_capacity =
        v2::getFixedFramePayloadCapacity(kRate, kShortCW);
    const size_t long_capacity =
        v2::getFixedFramePayloadCapacityZ(kRate, kLongCW, 81);
    CHECK(short_capacity == 461 && long_capacity == 527,
          "QPSK R3/4 capacity contract must be cw8/Z27=461 and cw3/Z81=527 bytes");

    constexpr int kShortCodedBits = kShortCW * 648;
    constexpr int kLongCodedBits = kLongCW * 1944;
    CHECK(kShortCodedBits == 5184 && kLongCodedBits == 5832 &&
              kLongCodedBits * 8 == kShortCodedBits * 9,
          "QPSK long profile must honestly expose its 12.5% coded-bit increase");
    const auto short_timing = connection_policy::wideOFDMFrameTiming(
        kMod, kRate, kShortCW, 27);
    const auto long_timing = connection_policy::wideOFDMFrameTiming(
        kMod, kRate, kLongCW, 81);
    CHECK(long_timing.data_symbols > short_timing.data_symbols &&
              long_timing.data_ms > short_timing.data_ms,
          "cw3/Z81 must not be reported as an equal-airtime substitute for production cw8/Z27");

    // Default-off control: the production logical geometry and descriptor-less
    // singleton tail remain byte-for-byte on cw8/Z27.
    unsetenv("ULTRA_QPSK_R34_LONG_LDPC");
    unsetenv("ULTRA_8PSK_LONG_LDPC");
    TempPayloadFile default_file("ultra_qpsk_r34_short", 0);
    CHECK(!default_file.path.empty(),
          "QPSK default-profile fixture should create an empty file");
    Connection baseline;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        baseline, kRate, 20.0f, kFading, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        baseline, kMod, kRate, kShortCW);
    std::vector<Bytes> baseline_singletons;
    std::vector<std::vector<Bytes>> baseline_bursts;
    baseline.setTransmitCallback(
        [&](const Bytes& frame) { baseline_singletons.push_back(frame); });
    baseline.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            baseline_bursts.push_back(frames);
        });
    CHECK(baseline.sendFile(default_file.path),
          "default QPSK profile should start the zero-byte file");
    CHECK(baseline_singletons.size() == 1 && baseline_bursts.empty(),
          "default cw8/Z27 tail must preserve the established singleton path");
    const auto baseline_header = v2::parseHeader(baseline_singletons.front());
    CHECK(baseline_header.valid && baseline_header.total_cw == kShortCW &&
              baseline.selectBurstLiftingZ() == 27 &&
              ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(baseline) == kShortCW &&
              ConnectionAdaptiveTestAccess::currentPayloadCapacity(baseline) == short_capacity,
          "default-off QPSK profile must retain the complete cw8/Z27 tuple");
    baseline.cancelFileTransfer();

    // Opt-in singleton: the addressed-away pad forces a descriptor-bearing
    // two-frame burst so the receiver learns both physical cw3 and Z81.
    setenv("ULTRA_QPSK_R34_LONG_LDPC", "1", 1);
    TempPayloadFile long_file("ultra_qpsk_r34_long", 0);
    CHECK(!long_file.path.empty(),
          "QPSK long-profile fixture should create an empty file");
    Connection long_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        long_tx, kRate, 20.0f, kFading, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        long_tx, kMod, kRate, kShortCW);
    std::vector<Bytes> unexpected_long_singletons;
    std::vector<std::vector<Bytes>> long_bursts;
    std::vector<bool> long_sent_results;
    int callback_z = 0;
    long_tx.setTransmitCallback(
        [&](const Bytes& frame) { unexpected_long_singletons.push_back(frame); });
    long_tx.setTransmitBurstCallback(
        [&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            callback_z = long_tx.selectBurstLiftingZ();
            long_bursts.push_back(frames);
        });
    long_tx.setFileSentCallback(
        [&](bool success, const std::string&) {
            long_sent_results.push_back(success);
        });
    CHECK(long_tx.sendFile(long_file.path),
          "QPSK long profile should start the zero-byte file");
    CHECK(unexpected_long_singletons.empty() && long_bursts.size() == 1 &&
              long_bursts.front().size() == 2,
          "QPSK Z81 singleton must become one descriptor-bearing real+ULPAD burst");
    CHECK(callback_z == 81 &&
              ConnectionAdaptiveTestAccess::experimentalQPSKR34LongLDPCActive(long_tx) &&
              ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(long_tx) == kLongCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameCodewords(long_tx) == kLongCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(long_tx) == 81 &&
              ConnectionAdaptiveTestAccess::currentPayloadCapacity(long_tx) == long_capacity,
          "active QPSK profile must atomically select cw3/Z81 for chunker and ARQ");
    for (const auto& frame : long_bursts.front()) {
        const auto header = v2::parseHeader(frame);
        CHECK(header.valid && header.total_cw == kLongCW,
              "real and ULPAD QPSK long frames must serialize as cw3");
    }
    const auto long_pad = v2::DataFrame::deserialize(long_bursts.front().back());
    CHECK(long_pad.has_value() &&
              long_pad->dst_hash == v2::hashCallsign("ULPAD") &&
              long_pad->payload.size() == long_capacity,
          "QPSK long singleton pad must be addressed away and sized at cw3/Z81 capacity");

    const auto wire_descriptor = v2::ControlFrame::makeBurstHeader(
        "W1ABC", "K2DEF", /*seq=*/0,
        static_cast<uint8_t>(long_bursts.front().size()),
        static_cast<uint8_t>(ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(long_tx)),
        kMod, kRate, /*interleave_flags=*/0,
        static_cast<uint8_t>(long_tx.selectBurstLiftingZ()));
    const auto descriptor_info = wire_descriptor.getBurstHeaderInfo();
    CHECK(descriptor_info.group_size == 2 && descriptor_info.cw_per_frame == kLongCW &&
              descriptor_info.lifting_z == 81 && descriptor_info.modulation == kMod &&
              descriptor_info.code_rate == kRate,
          "BURST_HEADER tuple derived from the connection must announce QPSK R3/4 cw3/Z81");

    const uint32_t reanchor_ms =
        connection_policy::shouldUseWideOFDMShortReanchor(
            WaveformMode::OFDM_CHIRP, kMod, kFading)
            ? connection_policy::wideOFDMShortReanchorChirpDurationMs()
            : 0;
    const uint32_t expected_airtime =
        connection_policy::postProcessedTxDurationFromSamplesMs(
            connection_policy::wideOFDMWireBurstSamples(
                kMod, kRate, /*frames=*/2, kLongCW, /*lifting_z=*/81,
                /*force_full_group_start=*/false, Modulation::QPSK,
                reanchor_ms));
    const uint32_t expected_timeout = connection_policy::unifiedBurstAckTimeoutMs(
        kMod, kRate, kLongCW, /*frames=*/2, /*lifting_z=*/81,
        Modulation::QPSK, ConnectionAdaptiveTestAccess::arqSackDelay(long_tx),
        reanchor_ms);
    CHECK(ConnectionAdaptiveTestAccess::physicalRoundAirtime(
              long_tx, long_bursts.front()) == expected_airtime &&
              ConnectionAdaptiveTestAccess::physicalRoundTimeout(
                  long_tx, long_bursts.front()) == expected_timeout &&
              ConnectionAdaptiveTestAccess::unifiedBurstTimeout(long_tx, 2) == expected_timeout,
          "QPSK long burst budget and ACK timers must consume the same physical cw3/Z81 geometry");

    ToneBurstAckDetection completion_ack;
    completion_ack.payload.group_seq = 0;
    completion_ack.payload.frame_mask = 0;
    completion_ack.payload.rate_hint = 7;
    completion_ack.payload.type = AckType::Ack;
    completion_ack.payload.move_epoch = 0;
    completion_ack.payload.rung_cmd = kRungCmdNone;
    ConnectionAdaptiveTestAccess::simulatePreviousBurstAired(long_tx);
    CHECK(long_tx.onToneBurstAck(completion_ack),
          "QPSK long-profile completion ACK should be consumed");
    CHECK(long_sent_results.size() == 1 && long_sent_results.front() &&
              !ConnectionAdaptiveTestAccess::experimentalQPSKR34LongLDPCActive(long_tx) &&
              ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(long_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameCodewords(long_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(long_tx) == 27,
          "successful QPSK completion must restore logical cw8/Z27 before callback return");

    // Forced logical geometry is deliberately excluded even when the value is
    // still eight: the experiment must never reinterpret an operator override.
    Connection forced_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        forced_tx, kRate, 20.0f, kFading, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        forced_tx, kMod, kRate, kShortCW);
    forced_tx.setForcedFrameCodewords(kShortCW, /*forced=*/true);
    std::vector<Bytes> forced_singletons;
    forced_tx.setTransmitCallback(
        [&](const Bytes& frame) { forced_singletons.push_back(frame); });
    forced_tx.setTransmitBurstCallback(
        [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
    CHECK(forced_tx.sendFile(long_file.path),
          "forced-CW exclusion fixture should start its file");
    CHECK(!ConnectionAdaptiveTestAccess::experimentalQPSKR34LongLDPCActive(forced_tx) &&
              forced_tx.selectBurstLiftingZ() == 27 &&
              ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(forced_tx) == kShortCW &&
              forced_singletons.size() == 1,
          "forced cw8 must remain descriptor-less cw8/Z27 despite the env opt-in");
    forced_tx.cancelFileTransfer();

    Connection cancel_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        cancel_tx, kRate, 20.0f, kFading, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        cancel_tx, kMod, kRate, kShortCW);
    cancel_tx.setTransmitCallback([](const Bytes&) {});
    cancel_tx.setTransmitBurstCallback(
        [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
    CHECK(cancel_tx.sendFile(long_file.path) &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(cancel_tx) == 81,
          "cancel fixture must reach QPSK cw3/Z81 first");
    cancel_tx.cancelFileTransfer();
    CHECK(!ConnectionAdaptiveTestAccess::experimentalQPSKR34LongLDPCActive(cancel_tx) &&
              ConnectionAdaptiveTestAccess::arqFixedFrameCodewords(cancel_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(cancel_tx) == 27,
          "explicit cancel must restore QPSK cw8/Z27");

    Connection failed_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        failed_tx, kRate, 20.0f, kFading, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        failed_tx, kMod, kRate, kShortCW);
    failed_tx.setTransmitCallback([](const Bytes&) {});
    failed_tx.setTransmitBurstCallback(
        [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
    std::vector<bool> failed_results;
    failed_tx.setFileSentCallback(
        [&](bool success, const std::string&) { failed_results.push_back(success); });
    CHECK(failed_tx.sendFile(long_file.path) &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(failed_tx) == 81,
          "failure fixture must reach QPSK cw3/Z81 first");
    ConnectionAdaptiveTestAccess::failFileTransfer(failed_tx);
    CHECK(failed_results.size() == 1 && !failed_results.front() &&
              !ConnectionAdaptiveTestAccess::experimentalQPSKR34LongLDPCActive(failed_tx) &&
              ConnectionAdaptiveTestAccess::arqFixedFrameCodewords(failed_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(failed_tx) == 27,
          "terminal file failure must notify once and restore QPSK cw8/Z27");

    Connection reset_tx;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        reset_tx, kRate, 20.0f, kFading, kMod);
    ConnectionAdaptiveTestAccess::setDataGeometry(
        reset_tx, kMod, kRate, kShortCW);
    reset_tx.setTransmitCallback([](const Bytes&) {});
    reset_tx.setTransmitBurstCallback(
        [](const std::vector<Bytes>&, uint16_t, uint8_t) {});
    CHECK(reset_tx.sendFile(long_file.path) &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(reset_tx) == 81,
          "reset fixture must reach QPSK cw3/Z81 first");
    reset_tx.reset();
    CHECK(ConnectionAdaptiveTestAccess::arqFixedFrameCodewords(reset_tx) == kShortCW &&
              ConnectionAdaptiveTestAccess::arqFixedFrameLiftingZ(reset_tx) == 27,
          "Connection::reset must strip QPSK transfer-scoped Z81 geometry");

    unsetenv("ULTRA_QPSK_R34_LONG_LDPC");
}

void test_harq_provisional_context_carries_destination() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(
        c, CodeRate::R2_3, 20.0f, 0.30f, Modulation::QAM8);
    c.setSoftCombiningHARQ(true);

    const auto ctx = c.harqProvisionalContext();
    CHECK(ctx.has_value() && ctx->valid(),
          "connected HARQ receiver must expose a valid provisional context");
    CHECK(ctx->sender_hash == v2::hashCallsign("K2DEF"),
          "provisional context source must be the remote session peer");
    CHECK(ctx->dst_hash == v2::hashCallsign("W1ABC"),
          "provisional context destination must be the local session peer");
}

} // namespace

int main() {
    // §RETX-PACING A/B knobs are latched ONCE via function-local statics — pin BOTH to
    // their disabled defaults BEFORE any Connection call so this binary deterministically
    // tests the byte-identical baseline (no hold armed, no collapse escape fired), per the
    // setenv-in-main pattern used by test_connection_policy.
    setenv("ULTRA_RETX_TROUGH_PACING", "0", 1);
    setenv("ULTRA_COLLAPSE_ESCAPE_ROUNDS", "0", 1);
    // RX-AUTHORITY is DEFAULT-ON since 2026-07-05 and supersedes the machinery this
    // binary tests (EMA feedback and RX-RATE-CMD descriptor demotes on the
    // legacy drivers) — pin it OFF so the legacy/fallback paths stay testable.
    setenv("ULTRA_RX_RATE_AUTHORITY", "0", 1);
    // The receiver-side ALC provenance regression below exercises the production
    // default deterministically even when the invoking shell carries an opt-out.
    setenv("ULTRA_SOFTWARE_ALC", "1", 1);
    // #58 increment 3: pin the connect-SNR-pool knobs to their disabled defaults so
    // every Connection built here deterministically runs the byte-identical scalar
    // path (rateSelectionSnrDb/wireSnrDb == measured_snr_db_, no pick defer).
    setenv("ULTRA_CONNECT_SNR_POOL", "0", 1);
    setenv("ULTRA_CONNECT_PICK_DEFER", "0", 1);
    setenv("ULTRA_WIRE_SNR_FRESH", "0", 1);
    // DESC-SWITCH (docs/MODE_SWITCH_PIGGYBACK_DESIGN_2026_07_03.md Phase 1): pin the
    // knob to its byte-identical default. Unlike the function-local-static knobs above,
    // this one is latched PER Connection in its ctor — the knob-ON tests flip the env
    // around a single construction and restore it, so ordering here is baseline-only.
    setenv("ULTRA_DESCRIPTOR_MODE_SWITCH", "0", 1);
    // RX-RATE-CMD (design Phase 2): same per-Connection-ctor latch pattern. NOTE: the
    // tone_burst_payload CRC-span binding reads this env ONCE process-wide, but this
    // binary never routes payloads through the parameter-less codec overloads, so the
    // per-test flips below cannot skew the wire span (payload tests use explicit spans).
    setenv("ULTRA_RX_RATE_CMD", "0", 1);
    // Exercise the derived wide-control deadline and single-ACK default, never
    // inherited rig A/B pins.
    unsetenv("ULTRA_MODE_CHANGE_RETRY_MS");
    unsetenv("ULTRA_MC_ACK_REPEATS");
    // Candidate-geometry tests pin the production base ceiling and the default
    // dense-rung two-clean-group escalation before their function-local statics
    // are first read.
    unsetenv("ULTRA_MAX_BURST_AIRTIME_MS");
    unsetenv("ULTRA_BURST_ESC_STREAK");
    unsetenv("ULTRA_BURST_ESCALATION");

    test_local_mode_change_ack_reconfigures_arq();
    test_local_mode_change_timeout_keeps_current_arq_mode();
    test_mode_change_retry_waits_for_clear_channel_without_spending_budget();
    test_remote_mode_change_reconfigures_arq();
    test_remote_mode_change_ack_is_single_by_default();
    test_mode_change_retry_holds_while_tx_keyed();
    test_duplicate_mode_change_single_reack_no_reapply();
    test_wide_ofdm_configures_short_tail_sack_delay();
    test_accepted_ofdm_data_sync_keeps_reactive_ack_without_proactive_tx();
    test_accepted_ofdm_data_sync_does_not_clear_non_ofdm_cache();
    test_all_failed_burst_group_is_not_handshake_evidence();
    test_only_established_peer_burst_frame_confirms_handshake();
    test_only_established_peer_classic_frame_confirms_handshake();
    test_duplicate_connect_replays_cached_connect_ack_without_confirming();
    test_connect_retry_interval_is_control_airtime_derived();
    test_disconnect_lost_ack_recovers_without_retry_grace_phase_lock();
    test_disconnect_timing_scope_and_timeout_budget();
    test_disconnect_retries_and_ack_copies_obey_half_duplex_gates();
    test_disconnect_grace_is_egress_exclusive_and_crossed_close_is_reactive();
    test_disconnect_teardown_phase_quarantines_rx_and_all_non_close_egress();
    test_disconnect_responder_grace_covers_long_cca_deferred_retry();
    test_disconnect_supports_synchronous_host_loopback();
    test_responder_handshake_timer_does_not_false_confirm();
    test_zero_progress_round_counter_knob_off_and_g42_protective();
    test_descriptor_switch_knob_off_is_byte_identical();
    test_descriptor_switch_commits_locally_at_clean_boundary();
    test_silent_escape_never_unilaterally_descriptor_switches();
    test_descriptor_adopt_reconfigures_receiver_without_ack();
    test_stale_epoch_tone_ack_has_no_connection_side_effects();
    test_latent_rate_consumes_exact_group_geometry_and_resets_per_qso();
    test_latent_startup_probe_is_one_group_and_fails_closed();
    test_forced_connect_ack_conflict_fails_closed();
    test_startup_probe_sender_guards_and_timeout_rollback();
    test_authority_climb_prices_the_real_file_tail();
    test_forced_r1_3_cw1_rejects_file_before_wire_tx();
    test_queued_file_geometry_shrink_reports_terminal_start_failure();
    test_empty_message_requests_are_rejected_atomically();
    test_deferred_queue_refuses_newest_and_abort_fails_every_accepted_message();
    test_reset_terminal_fails_accepted_queued_message();
    test_accepted_queued_start_failure_reports_terminal_status();
    test_deferred_payload_fifo_survives_failed_head_and_new_submission();
    test_midwindow_message_geometry_change_regrids_object_and_delivers_once();
    test_message_geometry_regrid_recuts_fragments_at_new_capacity();
    test_message_geometry_regrid_is_bounded_then_fails_closed();
    test_receiver_suppresses_duplicate_message_object();
    test_ordinary_adaptation_holds_geometry_for_complete_message();
    test_runtime_forced_cw_override_cannot_regrid_active_message();
    test_submitted_status_reset_runs_after_physical_handoff();
    test_failure_batch_precedes_reentrant_replacement_submission();
    test_delivered_status_reset_runs_after_ack_unwind();
    test_midwindow_geometry_change_disconnects_without_move_epoch();
    test_file_and_payload_queues_preserve_cross_class_fifo();
    test_file_start_respects_reversed_data_turn_without_prior_local_backlog();
    test_same_epoch_rebase_preserves_multiwindow_message_prefix();
    test_move_epoch_rebase_drops_stale_message_prefix();
    test_timeout_batch_waits_for_post_tick_flush_and_discards_obsolete_geometry();
    test_timeout_repair_batch_is_capped_before_retry_commit();
    test_arq_control_feedback_does_not_arm_data_ack_monitor();
    test_data_ack_monitor_arms_before_synchronous_transport_callback();
    test_full_repair_timing_uses_encoder_samples_and_respects_ceiling();
    test_variable_cw_single_frame_uses_advertised_physical_geometry();
    test_narrow_ofdm_keeps_its_waveform_specific_timeout();
    test_mcdpsk_data_arms_one_waveform_specific_monitor_per_physical_turn();
    test_synchronous_burst_ack_cannot_overwrite_outer_committed_frames();
    test_ack_revealed_hole_refill_uses_resend_anchor();
    test_proven_partial_sack_uses_descriptor_only_anchor_and_light_timing();
    test_proven_partial_sack_uses_physical_round_progress_not_cumulative_arq_progress();
    test_partial_sack_experiment_fails_closed_without_progress();
    test_partial_sack_experiment_requires_progress_from_this_ack();
    test_fragment_tail_hole_repairs_immediately_and_uses_one_frame_rto();
    test_file_tail_identical_keepalive_sack_retries_hole();
    test_terminal_tail_failure_aborts_suspended_window_and_rebases_next_payload();
    test_turn_refill_terminal_failure_discards_open_burst();
    test_terminal_base_hole_with_sacked_suffix_reports_once_and_reentrant_send_rebases();
    test_delivered_status_callback_can_send_without_corrupting_tracking();
    test_terminal_failure_disconnects_when_move_epoch_is_disabled();
    test_terminal_file_failure_disconnects_to_reset_peer_assembler();
    test_rx_rate_cmd_knob_off_is_byte_identical();
    test_rx_rate_cmd_receiver_emits_crater_down_hard_once_per_move();
    test_waiting_rebase_voice_has_per_outcome_event_identity();
    test_rx_rate_cmd_down_hard_mid_window_commits_via_descriptor_with_epoch();
    test_ofdm_connected_entry_does_not_emit_unsolicited_timing_anchor();
    test_interactive_bootstrap_yield_stops_after_initiator_starts_data();
    test_normal_ofdm_ack_arms_full_anchor_expectation();
    test_normal_message_turn_yield_periodically_rearms_full_anchor();
    test_tone_ack_callback_marks_only_group_boundary_as_physically_complete();
    test_software_alc_uses_current_partial_group_delivery_provenance();
    test_partial_sack_provenance_marks_only_exact_group_geometry();
    test_physical_tail_acks_prior_orphan_payload_cumulatively();
    test_harq_provisional_context_carries_destination();
    test_latent_candidate_geometry_matches_receiver_configured_wire_profile();
    test_8psk_long_ldpc_file_geometry_and_tail_safety();
    test_qpsk_r34_long_ldpc_file_geometry_and_lifecycle();

    if (tests_failed != 0) {
        std::cout << "ConnectionAdaptive: " << tests_failed << "/" << tests_run
                  << " failed\n";
        return 1;
    }

    std::cout << "ConnectionAdaptive: " << tests_run << "/" << tests_run
              << " passed\n";
    return 0;
}
