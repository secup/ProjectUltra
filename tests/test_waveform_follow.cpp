// Waveform follow (ULTRA_WAVEFORM_FOLLOW): a session crosses between MC-DPSK and OFDM as the channel
// moves (docs/WAVEFORM_FOLLOW_DESIGN_2026_11_11.md, INV-FOLLOW-001).
//
// Its own binary because follow rides the receiver-authority ACK word: the commands are the
// authority's rung bits. This binary runs the production defaults
// (authority on, ULTRA_FAST on), the configuration follow ships in.

#include "protocol/connection.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/ofdm_data_frame.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"
#include "helpers/temp_dir.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include "env_compat.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

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

}  // namespace

namespace ultra {
namespace protocol {

struct ConnectionAdaptiveTestAccess {
    static int physicalDataFrameCWCount(const Connection& c) {
        return c.physicalDataFrameCodewords();
    }
    static void makeLocalIss(Connection& c) {
        c.turn_.setLocalHolds(true);
        c.turn_.setPeerRequested(false);
        c.turn_.setRequestPending(false);
        c.turn_.setPeerDataSinceConnect(false);
        c.turn_.setYieldPending(false);
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
        c.data_modulation_ = Modulation::QPSK;
        c.data_code_rate_ = CodeRate::R1_2;
        c.data_frame_cw_count_ = v2::kDefaultFixedFrameCodewords;
        c.config_.fixed_frame_codewords = v2::kDefaultFixedFrameCodewords;
        c.measured_snr_db_ = 12.0f;
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
        c.data_modulation_ = Modulation::MFSK16_FAST;
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

    // One physical round as the file/message pump builds it: buffered, then flushed.
    static bool sendBufferedSingleFrame(Connection& c, const Bytes& payload) {
        c.burst_mode_active_ = true;
        c.burst_tx_buffer_.clear();
        const bool ok = c.arq_.sendFixedDataWithFlags(payload, 0);
        c.flushBurstBuffer();
        c.burst_mode_active_ = false;
        return ok;
    }

    // ── Waveform follow ──
    static void makeLocalIrsWaitingForTurn(Connection& c) {
        c.turn_.setLocalHolds(false);
        c.turn_.setYieldedWaiting(false);
        c.turn_.setRequestPending(false);
        c.turn_request_holdoff_ms_ = 0;
        c.handshake_confirmed_ = true;
    }
    static uint32_t turnRequestRetransmitMs(const Connection& c) {
        return c.turnRequestRetransmitMs();
    }
    static void notePeerHeard(Connection& c) { c.notePeerHeard(); }
    // A receiver-commanded rung requested by MODE_CHANGE, then applied when its ACK arrives.
    static void commandedModeChangeAcked(Connection& c, Modulation mod, CodeRate rate) {
        c.receiver_commanded_geometry_change_ = true;
        c.requestModeChange(mod, rate, 10.0f, 0);
        c.receiver_commanded_geometry_change_ = false;
        c.commitPendingModeChange("ACKed");
    }
    static void allowWaveformFollow(Connection& c) {
        c.waveform_follow_allowed_ = true;
        c.up_crossing_gate_.reset();
    }
    static WaveformMode negotiatedMode(const Connection& c) { return c.negotiated_mode_; }
    static Modulation dataModulation(const Connection& c) { return c.data_modulation_; }
    static CodeRate dataCodeRate(const Connection& c) { return c.data_code_rate_; }
    static int dataFrameCodewords(const Connection& c) { return c.data_frame_cw_count_; }
    static bool obeyCrossing(Connection& c, uint8_t word) {
        return c.maybeObeyCrossingCommand(word, "test");
    }
    // A receiver-authority (OFDM rung) command read from a tone ACK.
    static void obeyAuthority(Connection& c, uint8_t rung, bool accepted_clean_round) {
        c.maybeObeyAuthorityCommand(rung, accepted_clean_round);
    }
    static bool startFile(Connection& c, const std::string& path) {
        return c.file_transfer_.startSend(path);
    }
    static void pumpFile(Connection& c) { c.sendNextFileChunk(); }
    static bool fileFullyChunked(const Connection& c) {
        const auto p = c.file_transfer_.getProgress();
        return p.total_bytes > 0 && p.transferred_bytes >= p.total_bytes;
    }
    static size_t arqInFlightBytes(const Connection& c) { return c.arq_.getTxInFlightBytes(); }
    static bool modeChangePendingNow(const Connection& c) { return c.mode_change_pending_; }
    // The sender reads the receiver's MC-DPSK rung command from a tone ACK.
    static void obeyMcDpskRung(Connection& c, int rung, uint8_t group_seq) {
        c.maybeObeyMcDpskRungCommand(connection_policy::encodeMcDpskRungCommand(rung), group_seq);
    }
    static size_t txInFlightCap(const Connection& c) { return c.arq_.getTxInFlightCap(); }
    static void setDataFrameCodewords(Connection& c, int cw) {
        c.data_frame_cw_count_ = cw;
        c.config_.fixed_frame_codewords = cw;
        c.configureArqForCurrentDataMode();
    }
    // The receiver's word for the ACK that closes one MC-DPSK burst (clean = end of turn, no hole).
    static uint8_t mcDpskAckWord(Connection& c, bool clean) {
        c.rx_mcdpsk_last_modulation_ = c.data_modulation_;
        ++c.rx_mcdpsk_frames_since_command_;
        c.tone_ack_group_complete_context_ = clean;
        const uint8_t word = c.mcDpskRungCommandForAck(0);
        c.tone_ack_group_complete_context_ = false;
        return word;
    }
    // An ACK in the middle of a burst (a duplicate's re-ACK, an out-of-order SACK): it does not
    // close the turn; bitmap != 0 reports a hole.
    static uint8_t mcDpskMidBurstAckWord(Connection& c, uint32_t sack_bitmap) {
        c.rx_mcdpsk_last_modulation_ = c.data_modulation_;
        ++c.rx_mcdpsk_frames_since_command_;
        c.tone_ack_group_complete_context_ = false;
        return c.mcDpskRungCommandForAck(sack_bitmap);
    }
    static void feedMcDpskSnr(Connection& c, float snr_db, float fading) {
        c.setChannelQuality(snr_db, fading, SNRSource::MCDPSK_IN_BAND, /*data_aided=*/true);
    }
    // A decoded MC-DPSK frame with its physical SNR (training-span power vs burst-time noise).
    static void feedMcDpskFrame(Connection& c, float snr_db, float fading, float physical_db) {
        c.setChannelQuality(snr_db, fading, SNRSource::MCDPSK_IN_BAND, /*data_aided=*/true,
                            physical_db);
    }
    // A decoded OFDM frame's reading with the fading its LTS measured (the binding's feed).
    static void feedOfdmFrame(Connection& c, float snr_db, float fading) {
        c.setChannelQuality(snr_db, fading, SNRSource::OFDM_BROADBAND, /*data_aided=*/false);
    }
    static void noteOfdmGroup(Connection& c, bool delivered_any) {
        c.noteRxOfdmBurstAfterCrossing(delivered_any);
        c.noteRxOfdmGroupForFollow(delivered_any);
    }
    static uint8_t pendingCrossingWord(const Connection& c) {
        return c.rx_command_.pending() ? c.rx_command_.word() : 0;
    }
    static void tickFollow(Connection& c, uint32_t ms) { c.tickWaveformFollow(ms); }
    static void setRxReassembly(Connection& c, size_t bytes) {
        c.rx_reassembly_buffer_.assign(bytes, 0x5A);
    }
    static void setDataMode(Connection& c, Modulation mod, CodeRate rate) {
        c.data_modulation_ = mod;
        c.data_code_rate_ = rate;
    }
    static bool crossingProbeActive(const Connection& c) { return c.probation_.active(); }
    static uint32_t crossingRevertMs(const Connection& c) { return c.probation_.revertMs(); }
    static int upCrossingRun(const Connection& c) { return c.up_crossing_gate_.run(); }
    static void failUpCrossing(Connection& c) { c.up_crossing_gate_.crossingFailed(); }
    static void noteRxWaveform(Connection& c, WaveformMode mode) { c.noteRxWaveform(mode); }
    static bool dualListen(const Connection& c) { return c.dualListenActive(); }
    static uint32_t disconnectRetryIntervalMs(const Connection& c) {
        return c.disconnectRetryIntervalMs();
    }
    static int disconnectUsefulRetryCount(const Connection& c) {
        return c.disconnectUsefulRetryCount();
    }
    // A repeat of the last MC-DPSK ACK (no new frame since the command).
    static uint8_t mcDpskRepeatAckWord(Connection& c) {
        c.tone_ack_group_complete_context_ = false;
        return c.mcDpskRungCommandForAck(0);
    }
    // Down-crossing economics: OFDM deliveries feed the spell meter (busy time comes from ticks).
    // As onBurstGroupReceived does for a delivering group.
    static void deliverOfdm(Connection& c, int frames) {
        c.noteFollowOfdmDelivery(frames);
        c.noteRxOfdmGroupForFollow(true);
    }
    // A group whose descriptor decoded but whose data did not: the descriptor is a decoded frame
    // (onFrameReceived resets the silence), the group delivers nothing.
    static void emptyOfdmGroup(Connection& c) {
        c.rx_follow_silence_ms_ = 0;
        c.noteRxOfdmBurstAfterCrossing(false);
        c.noteRxOfdmGroupForFollow(false);
    }
    static uint64_t downCrossingMs(const Connection& c, bool receiver_side) {
        return c.followDownCrossingMs(receiver_side);
    }
    static double ofdmBytesPerSecond(const Connection& c) { return c.followOfdmBytesPerSecond(); }
    static uint32_t ackTimeoutMs(const Connection& c) { return c.arq_.getAckTimeout(); }
    static int maxRetries(const Connection& c) { return c.arq_.getMaxRetries(); }
    static bool putDataInFlight(Connection& c) {
        return c.arq_.sendFixedDataWithFlags(Bytes(64, 0x42), 0);
    }
    // At a round boundary: the retransmission timeout staged its resend (handleArqTimeoutBatch).
    static void senderCollapsePoll(Connection& c, int rounds_without_progress) {
        c.follow_silent_rounds_ = rounds_without_progress;
        c.staged_timeout_batch_.assign(1, Bytes(8, 0x11));
        c.maybeCrossDownOnCollapse();
        c.staged_timeout_batch_.clear();
    }
    // Mid-round: no timeout this tick.
    static void senderCollapsePollMidRound(Connection& c, int rounds_without_progress) {
        c.follow_silent_rounds_ = rounds_without_progress;
        c.staged_timeout_batch_.clear();
        c.maybeCrossDownOnCollapse();
    }
    // The collapse escape consumed the shared counter: follow keeps its own.
    static void escapeConsumesEvidence(Connection& c) { c.zero_progress_rounds_ = 0; }
    // The sender's poll at a round boundary on the evidence the rounds left (no override).
    static void senderPollCrossing(Connection& c) {
        c.staged_timeout_batch_.assign(1, Bytes(8, 0x11));
        c.maybeCrossDownOnCollapse();
        c.staged_timeout_batch_.clear();
    }
    static int followSilentRounds(const Connection& c) { return c.follow_silent_rounds_; }
    static void setPendingModeChange(Connection& c, Modulation mod, CodeRate rate) {
        c.mode_change_pending_ = true;
        c.pending_modulation_ = mod;
        c.pending_code_rate_ = rate;
        c.pending_ladder_rung_id_ = LadderRungId::OFDM_CHIRP;
    }
    static bool modeChangePending(const Connection& c) { return c.mode_change_pending_; }
    static void setStartupProbeAirborne(Connection& c, bool airborne) {
        c.tx_latent_startup_probe_active_ = airborne;
        c.tx_latent_startup_probe_airborne_ = airborne;
    }
    static void allowRateAdaptation(Connection& c) {
        c.adaptive_rate_enabled_ = true;
        c.latent_startup_probe_allowed_ = true;
    }
    static void agePool(Connection& c, uint32_t ms) { c.ageConnectSnrReadings(ms); }
    static size_t arqWindow(const Connection& c) { return c.arq_.getWindowSize(); }
    static size_t txCap(const Connection& c) { return c.arq_.getTxInFlightCap(); }
    static void noteRound(Connection& c, int progress, const char* origin) {
        c.noteArqRoundOutcome(progress, origin);
    }
    static void setZeroProgressRounds(Connection& c, int n) { c.zero_progress_rounds_ = n; }
    static void pollStuckEscape(Connection& c) { c.maybeEscapeStuckFrame(); }
    static void turnLoss(Connection& c, bool turn_over) { c.noteRxMcDpskTurnLoss(turn_over); }
    // Frames of the current burst decoded since this end's last command (no ACK formed yet).
    static void framesDecoded(Connection& c, int n) {
        c.rx_mcdpsk_last_modulation_ = c.data_modulation_;
        c.rx_mcdpsk_frames_since_command_ += n;
    }
    static bool dualUpCommanded(const Connection& c) { return c.rx_command_.upCommanded(); }
    static void markUpCommanded(Connection& c) { c.rx_command_.markUpCommanded(); }
    static int mcDpskRungCommand(const Connection& c) { return c.mcdpsk_rate_ctl_.command(); }
    static int mcDpskProbeAfter(const Connection& c) { return c.mcdpsk_rate_ctl_.probeAfterCleanBursts(); }
    static void mcDpskFrameLost(Connection& c, int modulation) { c.noteRxMcDpskFrameLost(modulation); }
    static void mcDpskObserveClean(Connection& c, int rung) { c.mcdpsk_rate_ctl_.observe(rung, true); }
    // One retransmission timeout inside the ARQ (no ACK heard).
    static void arqTimeoutRound(Connection& c) { c.arq_.tick(c.arq_.getAckTimeout() + 1); }
    // One heard ACK round in which seq `missing` (the base) did not arrive: an empty ACK.
    static void arqAckWithout(Connection& c, uint16_t missing) {
        auto ack = v2::ControlFrame::makeNack("K2DEF", "W1ABC",
                                              static_cast<uint16_t>(missing - 1), 0x0);
        ack.type = v2::FrameType::ACK;
        c.arq_.onFrameReceived(ack.serialize());
        c.arq_.tick(1000);  // past the duplicate-ACK window, short of a timeout
    }
    static int arqRetries(const Connection& c) { return c.arq_.maxInFlightRetryCount(); }
    static int arqHeardMisses(const Connection& c) { return c.arq_.maxInFlightHeardMissCount(); }
    static bool escapeEvidence(Connection& c) { return c.escapeHasPeerEvidence("test"); }
    // The receiver authority on one delivered OFDM group: its frames' EVMs, then the verdict.
    static uint8_t authorityAfterGroup(Connection& c, const std::vector<float>& evm_db) {
        c.burst_obs_frame_evm_db_ = evm_db;
        const auto n = static_cast<uint8_t>(evm_db.size());
        c.updateRxAuthorityCommand(/*all_ok=*/true, /*quality=*/1.0f, /*full_crater=*/false,
                                   /*delivered_fraction=*/1.0f, n, n, /*geometry_proven=*/true);
        return c.rx_authority_cmd_;
    }
    static bool sendFixedData(Connection& c, const Bytes& payload, uint8_t flags) {
        return c.arq_.sendFixedDataWithFlags(payload, flags);
    }
    // The sender right after obeying an up-crossing launched by the ACK (seq, mask), its first
    // OFDM group on air for another on_air_ms (negative: already off the air).
    static void armCrossingLaunch(Connection& c, uint8_t seq, uint16_t mask, int on_air_ms) {
        c.probation_.beginSender(c.probation_.returnMod());
        c.probation_.markLaunchAck(seq, mask);
        c.last_data_burst_end_valid_ = true;
        c.last_data_burst_end_ =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(on_air_ms);
    }
    static uint64_t fallbackSmallWindowMs(const Connection& c) {
        return c.follow_fallback_small_window_ms_;
    }
    static void applyDataMode(Connection& c, Modulation mod, CodeRate rate, LadderRungId rung) {
        c.applyDataMode(mod, rate, 0, rung);
    }
};

}  // namespace protocol
}  // namespace ultra

namespace {

// ─────────────── Waveform follow (ULTRA_WAVEFORM_FOLLOW) ───────────────

// Sender: an OFDM rung in an MC-DPSK session crosses up, an MC-DPSK rung in an OFDM session
// crosses down; the modem hears the waveform switch BEFORE the data mode of the same change.
void test_waveform_follow_sender_crossings_switch_modem_first() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    std::vector<std::string> events;
    int cross_carriers = -1, cross_sps = -1, cross_cw = -1;
    c.setWaveformCrossingCallback([&](WaveformMode mode, Modulation, CodeRate, int cw,
                                      int carriers, int sps) {
        events.push_back(std::string("cross:") + waveformModeToString(mode));
        cross_cw = cw;
        cross_carriers = carriers;
        cross_sps = sps;
    });
    c.setDataModeChangedCallback([&](Modulation, CodeRate, int, float, float, int, int, bool) {
        events.push_back("data");
    });

    CHECK(!ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(1)),
          "an MC-DPSK rung in an MC-DPSK session is no crossing");
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(c, kRungIdxQam8R23),
          "an OFDM rung in an MC-DPSK session crosses up");
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::OFDM_CHIRP &&
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::QAM8 &&
              ConnectionAdaptiveTestAccess::dataCodeRate(c) == CodeRate::R2_3,
          "up-crossing lands on the commanded OFDM rung");
    CHECK(events.size() == 2 && events[0] == std::string("cross:") +
                                     waveformModeToString(WaveformMode::OFDM_CHIRP) &&
              events[1] == "data" && cross_carriers == 0 && cross_sps == 0 && cross_cw > 0,
          "modem switches waveform first, then the data mode follows");

    events.clear();
    CHECK(!ConnectionAdaptiveTestAccess::obeyCrossing(c, kRungIdxQpskR12),
          "an OFDM rung in an OFDM session is the ladder's business, not a crossing");
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(0)),
          "an MC-DPSK rung in an OFDM session crosses down");
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK &&
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::MFSK16_FAST &&
              ConnectionAdaptiveTestAccess::dataCodeRate(c) == CodeRate::R1_4 &&
              ConnectionAdaptiveTestAccess::dataFrameCodewords(c) == 3,
          "down-crossing lands on the middle rung (16-FSK-F) at the standard 3-codeword geometry");
    CHECK(events.size() == 2 && cross_carriers == 8 && cross_sps == 1024 && cross_cw == 3,
          "the modem gets the MC-DPSK profile with the crossing");
}

// BUG-OFDM-PROBE-PUNISHES-MCDPSK-RUNG: while this receiver has commanded an up-crossing, an MC-DPSK
// frame lost at an unknown rung is the peer's OFDM burst and leaves the MC-DPSK rate controller
// alone; outside that window the same loss still counts.
void test_ofdm_probe_does_not_punish_mcdpsk_rung() {
    for (int commanded = 0; commanded < 2; ++commanded) {
        Connection c;
        ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
        // Clean 16-FSK-F bursts until the controller probes the rung above (DQPSK).
        const int ffast = connection_policy::mcDpskRungIndexFor(Modulation::MFSK16_FAST);
        for (int i = 0; i < 16 && ConnectionAdaptiveTestAccess::mcDpskRungCommand(c) <= ffast; ++i) {
            ConnectionAdaptiveTestAccess::mcDpskObserveClean(
                c, ConnectionAdaptiveTestAccess::mcDpskRungCommand(c));
        }
        CHECK(ConnectionAdaptiveTestAccess::mcDpskRungCommand(c) > ffast,
              "the fixture's controller probes above 16-FSK-F");
        const int rung_before = ConnectionAdaptiveTestAccess::mcDpskRungCommand(c);
        const int probe_before = ConnectionAdaptiveTestAccess::mcDpskProbeAfter(c);
        if (commanded) ConnectionAdaptiveTestAccess::markUpCommanded(c);
        ConnectionAdaptiveTestAccess::mcDpskFrameLost(c, -1);
        const bool untouched =
            ConnectionAdaptiveTestAccess::mcDpskRungCommand(c) == rung_before &&
            ConnectionAdaptiveTestAccess::mcDpskProbeAfter(c) == probe_before;
        if (commanded) {
            CHECK(untouched, "OFDM commanded: an unattributed loss leaves the MC-DPSK rung and probe");
        } else {
            CHECK(!untouched, "no crossing commanded: an unattributed loss still counts");
        }
    }
}

// Knob off (the default): crossing words change nothing.
void test_waveform_follow_knob_off_ignores_crossing_words() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "0", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    int crossings = 0;
    c.setWaveformCrossingCallback(
        [&](WaveformMode, Modulation, CodeRate, int, int, int) { ++crossings; });
    CHECK(!ConnectionAdaptiveTestAccess::obeyCrossing(c, kRungIdxQam8R23) && crossings == 0 &&
              ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK,
          "knob off: an OFDM rung does not move an MC-DPSK session");
    ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    for (int i = 0; i < 4; ++i) {
        const uint8_t word = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
        CHECK(connection_policy::decodeOfdmRungCommand(word) == kRungIdxNone,
              "knob off: the receiver never commands OFDM");
    }
}

// Receiver, MC-DPSK session: a clean burst whose readings pick OFDM -> the ACK commands the picked
// rung, the next tick crosses; with no OFDM burst by the revert deadline it comes back and the next
// crossing needs twice the run.
void test_waveform_follow_receiver_commands_up_and_reverts() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);

    const uint8_t first = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
    const uint8_t rung = connection_policy::decodeOfdmRungCommand(first);
    CHECK(rung != kRungIdxNone, "a clean burst at 22 dB commands an OFDM rung (run 1)");
    CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(c) == first &&
              ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK,
          "the receiver crosses only after the ACK is out (next tick)");
    CHECK(ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true) == first,
          "a repeat of that ACK carries the same command");

    ConnectionAdaptiveTestAccess::tickFollow(c, 10);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::OFDM_CHIRP &&
              ConnectionAdaptiveTestAccess::crossingProbeActive(c),
          "tick applies the crossing and starts probation");
    const uint32_t revert = ConnectionAdaptiveTestAccess::crossingRevertMs(c);
    CHECK(revert > 5000 && revert < 15000, "revert window from the airtime model");
    ConnectionAdaptiveTestAccess::tickFollow(c, revert - 1);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::OFDM_CHIRP,
          "still waiting for the first OFDM burst");
    ConnectionAdaptiveTestAccess::tickFollow(c, 2);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK &&
              !ConnectionAdaptiveTestAccess::crossingProbeActive(c) &&
              ConnectionAdaptiveTestAccess::upCrossingRun(c) == 2,
          "no OFDM burst: back to MC-DPSK, next crossing needs twice the run");

    // Second attempt on fresh readings (the fall-back restarted the evidence): two clean bursts,
    // then OFDM holds. The first decoding group ends probation but the backoff resets only after
    // UpCrossingGate::kHeldGroups delivering groups.
    uint8_t word = 0;
    for (int i = 0; i < 2; ++i) {
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
        word = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
    }
    CHECK(connection_policy::decodeOfdmRungCommand(word) != kRungIdxNone,
          "the backed-off run (2 clean bursts) commands OFDM again");
    ConnectionAdaptiveTestAccess::tickFollow(c, 10);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, true);
    CHECK(!ConnectionAdaptiveTestAccess::crossingProbeActive(c) &&
              ConnectionAdaptiveTestAccess::upCrossingRun(c) == 2,
          "one decoding group ends probation, backoff kept");
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, true);
    CHECK(ConnectionAdaptiveTestAccess::upCrossingRun(c) == 1,
          "two delivering OFDM groups: the crossing held, backoff reset");
}

// Receiver: the first OFDM group after an up-crossing settles it - something decoded holds (run
// resets), nothing decoded calls it off in that group's ACK.
void test_waveform_follow_first_ofdm_group_settles_probation() {
    for (const bool delivered : {true, false}) {
        setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
        Connection c;
        unsetenv("ULTRA_WAVEFORM_FOLLOW");
        ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
        for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
        (void)ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
        ConnectionAdaptiveTestAccess::tickFollow(c, 10);
        CHECK(ConnectionAdaptiveTestAccess::crossingProbeActive(c), "on probation after crossing");
        ConnectionAdaptiveTestAccess::noteOfdmGroup(c, delivered);
        CHECK(!ConnectionAdaptiveTestAccess::crossingProbeActive(c), "first group ends probation");
        if (delivered) {
            CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(c) == 0 &&
                      ConnectionAdaptiveTestAccess::upCrossingRun(c) == 1,
                  "a decoding first group holds the crossing");
        } else {
            CHECK(connection_policy::decodeMcDpskRungCommand(
                      ConnectionAdaptiveTestAccess::pendingCrossingWord(c)) >= 0 &&
                      ConnectionAdaptiveTestAccess::upCrossingRun(c) == 2,
                  "an empty first group commands MC-DPSK and backs off");
            ConnectionAdaptiveTestAccess::tickFollow(c, 10);
            CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK,
                  "the receiver returns once that ACK is out");
        }
    }
}

// Receiver, OFDM session: two empty groups with the second at the floor rung command MC-DPSK;
// above the floor the ladder steps down first; a delivering group resets the count.
void test_waveform_follow_receiver_empty_floor_commands_down() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection floor_rx;
    Connection upper_rx;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(floor_rx, CodeRate::R1_4, 3.0f, 0.05f,
                                                    Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(floor_rx);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(floor_rx, false);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(floor_rx, true);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(floor_rx, false);
    CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(floor_rx) == 0,
          "a delivering group in between resets the count");
    ConnectionAdaptiveTestAccess::noteOfdmGroup(floor_rx, false);
    CHECK(connection_policy::decodeMcDpskRungCommand(
              ConnectionAdaptiveTestAccess::pendingCrossingWord(floor_rx)) == 0,
          "two empty groups at the floor command MC-DPSK DBPSK");
    ConnectionAdaptiveTestAccess::tickFollow(floor_rx, 10);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(floor_rx) == WaveformMode::MC_DPSK,
          "the receiver crosses down after the ACK");

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(upper_rx, CodeRate::R2_3, 3.0f, 0.05f,
                                                    Modulation::QAM16);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(upper_rx);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(upper_rx, false);
    ConnectionAdaptiveTestAccess::setDataMode(upper_rx, Modulation::QPSK, CodeRate::R1_2);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(upper_rx, false);
    CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(upper_rx) == 0,
          "while the ladder steps down it keeps its chance");
    ConnectionAdaptiveTestAccess::noteOfdmGroup(upper_rx, false);
    CHECK(connection_policy::decodeMcDpskRungCommand(
              ConnectionAdaptiveTestAccess::pendingCrossingWord(upper_rx)) == 0,
          "an empty group at the same rung: the ladder is stuck, command MC-DPSK");
}

// A fade (BUG-FOLLOW-STALE-EVIDENCE): an OFDM session on a measured 20 dB channel collapses and crosses
// down. Its pre-fade physical measurement must not bound - and so license - the fading boost on the
// readings of the fade: before the fix, two clean bursts read at -1 dB on Good fading were selected
// as 10 dB (-1 + 11, capped at 20 + 2) and commanded OFDM into the fade. Fresh evidence decides.
void test_waveform_follow_down_crossing_forgets_stale_evidence() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    c.setPhysicalChannelStats(20.0f, 1.5f, 16);  // the pre-fade channel, measured
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(0)),
          "the collapse crosses down");
    // The readings below are DPSK data readings (a 16-FSK frame reports on the control scale and
    // its OFDM judgment is carried to this one, Connection::sessionLadderPick).
    ConnectionAdaptiveTestAccess::setDataMode(c, Modulation::DQPSK, CodeRate::R1_4);
    for (int burst = 0; burst < 4; ++burst) {
        ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, -1.0f, 0.6f);  // no physical reading
        const uint8_t word = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
        CHECK(connection_policy::decodeOfdmRungCommand(word) == kRungIdxNone,
              "a fade read at -1 dB never commands OFDM, whatever the channel was before it");
        ConnectionAdaptiveTestAccess::agePool(c, 19000);  // a 3-frame cycle
    }
    // The fade's own physical measurement bounds the boost the same way.
    c.setPhysicalChannelStats(1.5f, 2.0f, 3);
    for (int burst = 0; burst < 3; ++burst) {
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, -1.0f, 0.6f, 1.5f);
        const uint8_t word = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
        CHECK(connection_policy::decodeOfdmRungCommand(word) == kRungIdxNone,
              "fresh physical evidence of the fade keeps the session in MC-DPSK");
        ConnectionAdaptiveTestAccess::agePool(c, 19000);
    }
    // The channel comes back: fresh readings and a fresh physical measurement.
    c.setPhysicalChannelStats(20.0f, 1.5f, 4);
    ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 14.0f, 0.6f, 20.0f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone,
          "the first clean burst of the returned channel commands OFDM");
}

// Receiver silence: an OFDM receiver that hears nothing of an incoming object for
// followReceiverSilenceMs crosses down by itself. A partly reassembled payload counts (the TNC's
// messages travel as payloads, not files); an idle peer holding the turn does not.
void test_waveform_follow_receiver_silence_needs_an_incoming_object() {
    namespace tba = ultra::waveform::tone_burst_ack;
    const uint32_t limit = connection_policy::followReceiverSilenceMs(
        tba::kTotalSymbols * tba::kStaircaseSymbolMs.back());
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection idle;
    Connection mid_payload;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(idle, CodeRate::R1_2, 20.0f, 0.05f,
                                                    Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(idle);
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::tickFollow(idle, limit);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(idle) == WaveformMode::OFDM_CHIRP,
          "an idle peer's silence never moves the session");

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(mid_payload, CodeRate::R1_2, 20.0f, 0.05f,
                                                    Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(mid_payload);
    ConnectionAdaptiveTestAccess::setRxReassembly(mid_payload, 600);
    ConnectionAdaptiveTestAccess::tickFollow(mid_payload, limit - 1);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(mid_payload) == WaveformMode::OFDM_CHIRP,
          "silence below the limit: still OFDM");
    ConnectionAdaptiveTestAccess::tickFollow(mid_payload, 2);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(mid_payload) == WaveformMode::MC_DPSK,
          "a payload left half-received for the limit: the receiver crosses down by itself");
}

// After a fade the sender resends its window whole (the fade ate the ACKs), and every duplicate
// draws a re-ACK in the middle of the burst. Those ACKs close no turn and report no hole: no burst
// verdict, so they must not reset the up-crossing run. An ACK that reports a hole is a loss.
void test_waveform_follow_mid_burst_acks_are_no_verdict() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::failUpCrossing(c);  // a backed-off run of 2 bursts
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) == kRungIdxNone,
          "first clean burst of a run of 2: no command yet");
    for (int dup = 0; dup < 2; ++dup) {
        CHECK(connection_policy::decodeOfdmRungCommand(
                  ConnectionAdaptiveTestAccess::mcDpskMidBurstAckWord(c, 0)) == kRungIdxNone,
              "a duplicate's re-ACK carries no command");
    }
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone,
          "mid-burst re-ACKs did not reset the run: the second clean burst commands OFDM");

    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection holes_rx;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(holes_rx);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(holes_rx);
    ConnectionAdaptiveTestAccess::failUpCrossing(holes_rx);
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(holes_rx, 22.0f, 0.05f, 23.0f);
    (void)ConnectionAdaptiveTestAccess::mcDpskAckWord(holes_rx, true);
    (void)ConnectionAdaptiveTestAccess::mcDpskMidBurstAckWord(holes_rx, 0x2);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(holes_rx, true)) == kRungIdxNone,
          "an ACK reporting a hole is a loss: the run starts over");
}

// Ski rental (connection_policy::followSkiRentalMs): an OFDM link that was delivering rides out a
// silence far past the evidence floor - MC-DPSK carries a few percent of it and returning costs a
// round - and crosses once the outage outlasts that price, never later than where the ARQ would
// abandon a frame. A link that delivered nothing keeps the evidence floor (the silence test above).
void test_waveform_follow_delivering_link_rides_out_a_silence() {
    namespace tba = ultra::waveform::tone_burst_ack;
    const uint32_t floor_ms = connection_policy::followReceiverSilenceMs(
        tba::kTotalSymbols * tba::kStaircaseSymbolMs.back());
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection rx;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(rx, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(rx);
    ConnectionAdaptiveTestAccess::setRxReassembly(rx, 600);  // an object is incoming
    const uint32_t rto = ConnectionAdaptiveTestAccess::ackTimeoutMs(rx);
    for (int round = 0; round < 6; ++round) {  // six delivering rounds of 8 frames
        ConnectionAdaptiveTestAccess::tickFollow(rx, rto);
        ConnectionAdaptiveTestAccess::deliverOfdm(rx, 8);
    }
    CHECK(ConnectionAdaptiveTestAccess::ofdmBytesPerSecond(rx) > 50.0,
          "the spell meter measured what OFDM delivered");
    const uint64_t wait = ConnectionAdaptiveTestAccess::downCrossingMs(rx, /*receiver_side=*/true);
    const uint64_t horizon =
        static_cast<uint64_t>(ConnectionAdaptiveTestAccess::maxRetries(rx) - 2) * rto - rto / 2;
    CHECK(wait > floor_ms && wait <= connection_policy::kFollowPatienceMs && wait <= horizon,
          "a delivering link waits past the evidence floor, at most the operator's patience");
    ConnectionAdaptiveTestAccess::tickFollow(rx, floor_ms + 1000);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(rx) == WaveformMode::OFDM_CHIRP,
          "past the old 25 s floor: still OFDM (a QSB trough)");
    ConnectionAdaptiveTestAccess::tickFollow(rx, static_cast<uint32_t>(wait - floor_ms - 2000));
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(rx) == WaveformMode::OFDM_CHIRP,
          "just short of the crossing time: still OFDM");
    ConnectionAdaptiveTestAccess::tickFollow(rx, 2000);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(rx) == WaveformMode::MC_DPSK,
          "the outage outlasted what returning costs: MC-DPSK");
}

// The same economics for empty groups (bursts the receiver syncs to but cannot decode): a link that
// was delivering does not leave OFDM on two empty groups; one that never delivered does.
void test_waveform_follow_empty_groups_follow_the_economics() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection rx;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(rx, CodeRate::R1_4, 10.0f, 0.6f,
                                                    Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(rx);
    ConnectionAdaptiveTestAccess::setRxReassembly(rx, 600);
    const uint32_t rto = ConnectionAdaptiveTestAccess::ackTimeoutMs(rx);
    for (int round = 0; round < 6; ++round) {
        ConnectionAdaptiveTestAccess::tickFollow(rx, rto);
        ConnectionAdaptiveTestAccess::deliverOfdm(rx, 8);
    }
    const uint64_t wait = ConnectionAdaptiveTestAccess::downCrossingMs(rx, /*receiver_side=*/true);
    CHECK(wait > 2ull * rto, "the floor rung that delivered still waits several rounds");
    ConnectionAdaptiveTestAccess::emptyOfdmGroup(rx);
    ConnectionAdaptiveTestAccess::tickFollow(rx, rto);
    ConnectionAdaptiveTestAccess::emptyOfdmGroup(rx);
    CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(rx) == 0,
          "two empty groups of a link that was delivering: no command yet");
    for (uint64_t elapsed = rto; elapsed < wait; elapsed += rto) {  // a group per round
        ConnectionAdaptiveTestAccess::tickFollow(rx, rto);
        ConnectionAdaptiveTestAccess::emptyOfdmGroup(rx);
        if (elapsed + rto < wait) {
            CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(rx) == 0,
                  "empty groups short of the crossing time: no command");
        }
    }
    ConnectionAdaptiveTestAccess::tickFollow(rx, rto);
    ConnectionAdaptiveTestAccess::emptyOfdmGroup(rx);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(rx) == WaveformMode::MC_DPSK ||
              connection_policy::decodeMcDpskRungCommand(
                  ConnectionAdaptiveTestAccess::pendingCrossingWord(rx)) == 0,
          "empty past the crossing time: MC-DPSK commanded (or already applied)");
}

// Sender: the collapse evidence (two rounds without progress) crosses at once on a link that never
// delivered, waits the ski-rental time on one that did, and crosses regardless when a frame is about
// to be abandoned.
void test_waveform_follow_sender_waits_out_a_fade() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection fresh;
    Connection delivering;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(fresh, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(fresh);
    CHECK(ConnectionAdaptiveTestAccess::putDataInFlight(fresh), "data in flight");
    ConnectionAdaptiveTestAccess::senderCollapsePoll(fresh, 2);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(fresh) == WaveformMode::MC_DPSK,
          "never delivered: two silent rounds cross");

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(delivering, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(delivering);
    CHECK(ConnectionAdaptiveTestAccess::putDataInFlight(delivering), "data in flight");
    const uint32_t rto = ConnectionAdaptiveTestAccess::ackTimeoutMs(delivering);
    for (int round = 0; round < 6; ++round) {
        ConnectionAdaptiveTestAccess::tickFollow(delivering, rto);
        ConnectionAdaptiveTestAccess::deliverOfdm(delivering, 8);
    }
    ConnectionAdaptiveTestAccess::tickFollow(delivering, 2 * rto);
    ConnectionAdaptiveTestAccess::senderCollapsePoll(delivering, 2);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(delivering) == WaveformMode::OFDM_CHIRP,
          "a delivering link does not leave OFDM on two silent rounds");
    const uint64_t wait = ConnectionAdaptiveTestAccess::downCrossingMs(delivering, false);
    ConnectionAdaptiveTestAccess::tickFollow(delivering, static_cast<uint32_t>(wait));
    ConnectionAdaptiveTestAccess::escapeConsumesEvidence(delivering);
    ConnectionAdaptiveTestAccess::senderCollapsePollMidRound(delivering, 3);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(delivering) == WaveformMode::OFDM_CHIRP,
          "past the crossing time but mid-round (a resend on the air): no crossing yet");
    ConnectionAdaptiveTestAccess::senderCollapsePoll(delivering, 3);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(delivering) == WaveformMode::MC_DPSK,
          "silent past the crossing time: MC-DPSK");
}

// Sender: the down-crossing counts rounds in which the peer was not heard. An ACK of any kind, or
// any CRC-valid frame from the peer, ends that silence: two timeouts, then a MODE_CHANGE ACK, is a
// live link (GUI QSB seed 44 crossed 0.2 s after one); two more timeouts in a row cross.
void test_waveform_follow_sender_silence_ends_when_the_peer_is_heard() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "1", 1);
    Connection framed;
    Connection acked;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    for (Connection* c : {&framed, &acked}) {
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(*c, CodeRate::R2_3, 20.0f, 0.6f,
                                                        Modulation::QAM8);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(*c);
        CHECK(ConnectionAdaptiveTestAccess::putDataInFlight(*c), "data in flight");
        ConnectionAdaptiveTestAccess::noteRound(*c, 0, "rto");
        ConnectionAdaptiveTestAccess::noteRound(*c, 0, "rto");
        CHECK(ConnectionAdaptiveTestAccess::followSilentRounds(*c) == 2,
              "two retransmission timeouts are two silent rounds");
    }
    // The peer's control frame (an ACK of an older request): CRC-valid, from the peer.
    framed.onFrameReceived(v2::ControlFrame::makeAck("K2DEF", "W1ABC", 0x7FF0).serialize(), false);
    CHECK(ConnectionAdaptiveTestAccess::followSilentRounds(framed) == 0,
          "a frame from the peer ends the silence");
    ConnectionAdaptiveTestAccess::senderPollCrossing(framed);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(framed) == WaveformMode::OFDM_CHIRP,
          "a link that just carried the peer's frame does not cross");
    // An empty tone ACK: the peer decoded the descriptor and answered.
    ConnectionAdaptiveTestAccess::noteRound(acked, 0, "toneburst-ack");
    ConnectionAdaptiveTestAccess::senderPollCrossing(acked);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(acked) == WaveformMode::OFDM_CHIRP,
          "an empty ACK is the peer answering: no crossing");
    // Silence again: two timeouts in a row, and the link that never delivered crosses.
    ConnectionAdaptiveTestAccess::noteRound(framed, 0, "rto");
    ConnectionAdaptiveTestAccess::senderPollCrossing(framed);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(framed) == WaveformMode::OFDM_CHIRP,
          "one timeout after the peer was heard: no crossing");
    ConnectionAdaptiveTestAccess::noteRound(framed, 0, "rto");
    ConnectionAdaptiveTestAccess::senderPollCrossing(framed);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(framed) == WaveformMode::MC_DPSK,
          "two timeouts in a row after it: MC-DPSK");
}

// Dual-listen (ULTRA_WAVEFORM_FOLLOW=2): the receiver hears both waveforms. It commands an
// up-crossing but does not switch; a repeat of that ACK carries the same command; it switches when
// OFDM arrives, to the commanded rung; two delivering groups hold the crossing.
void test_dual_listen_receiver_commands_up_and_follows_the_peer() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    CHECK(ConnectionAdaptiveTestAccess::dualListen(c), "knob 2 latches dual-listen");
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    const uint8_t word = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
    const uint8_t rung = connection_policy::decodeOfdmRungCommand(word);
    CHECK(rung != kRungIdxNone, "a clean burst at 22 dB commands OFDM");
    CHECK(ConnectionAdaptiveTestAccess::mcDpskRepeatAckWord(c) == word,
          "a repeat of that ACK carries the same command (not the constellation word)");
    ConnectionAdaptiveTestAccess::tickFollow(c, 30000);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK &&
              !ConnectionAdaptiveTestAccess::crossingProbeActive(c),
          "the receiver does not switch on its own command");
    ConnectionAdaptiveTestAccess::noteRxWaveform(c, WaveformMode::OFDM_CHIRP);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::OFDM_CHIRP &&
              coherentRungIndexFor(ConnectionAdaptiveTestAccess::dataModulation(c),
                                   ConnectionAdaptiveTestAccess::dataCodeRate(c)) == rung,
          "OFDM arrived: the receiver follows, on the rung it commanded");
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, true);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, true);
    CHECK(ConnectionAdaptiveTestAccess::upCrossingRun(c) == 1, "two delivering groups hold it");
}

// Dual-listen receiver that followed the peer down: the first MC-DPSK ACK has no reading from the
// spell yet (the crossing cleared the pool), and the session's pre-fade scalar SNR must not command
// OFDM (GUI 2-minute dip, seed 42: an 18.6 dB pre-fade reading commanded OFDM at 0 dB, the crossing
// failed and doubled the run). A burst that reads well then does.
void test_dual_listen_up_crossing_needs_a_reading_from_the_burst() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 18.6f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::noteRxWaveform(c, WaveformMode::MC_DPSK);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK,
          "the peer's MC-DPSK frame arrived: the receiver followed it down");
    const uint8_t first = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
    CHECK(connection_policy::decodeOfdmRungCommand(first) == kRungIdxNone,
          "no reading from this burst: the pre-fade 18.6 dB does not command OFDM");
    CHECK(ConnectionAdaptiveTestAccess::upCrossingRun(c) == 1, "and nothing failed: the run stays 1");
    // The listener's first frame read a data-aided 40.2 dB in a 0 dB dip, with no physical reading:
    // not physically bounded, no command.
    ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 40.2f, 0.05f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) == kRungIdxNone,
          "a reading with no physical SNR in its burst commands no OFDM");
    for (int i = 0; i < 3; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    const uint8_t next = ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
    CHECK(connection_policy::decodeOfdmRungCommand(next) != kRungIdxNone,
          "a clean burst reading 22 dB commands OFDM at once");
}

// Dual-listen receiver in OFDM: empty groups command MC-DPSK on its ACKs, it does not switch; a
// delivering group withdraws the command; silence never moves it; MC-DPSK arriving does.
void test_dual_listen_receiver_commands_down_and_follows_the_peer() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_4, 3.0f, 0.05f,
                                                    Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::setRxReassembly(c, 600);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, false);
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, false);
    CHECK(connection_policy::decodeMcDpskRungCommand(
              ConnectionAdaptiveTestAccess::pendingCrossingWord(c)) == 0,
          "two empty groups at the floor: the ACK commands MC-DPSK");
    ConnectionAdaptiveTestAccess::tickFollow(c, 120000);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::OFDM_CHIRP,
          "the receiver does not switch on its command, nor on two minutes of silence");
    ConnectionAdaptiveTestAccess::noteOfdmGroup(c, true);
    CHECK(ConnectionAdaptiveTestAccess::pendingCrossingWord(c) == 0,
          "OFDM delivers again: the down command is withdrawn");
    ConnectionAdaptiveTestAccess::noteRxWaveform(c, WaveformMode::MC_DPSK);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK &&
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::MFSK16_FAST,
          "MC-DPSK arrived: the receiver follows, 16-FSK-F");
}

// Dual-listen receiver: it commanded OFDM and the peer's next burst is MC-DPSK again - the crossing
// did not happen (missed command, or the OFDM burst failed and the sender fell back): the next
// up-crossing waits twice as long.
void test_dual_listen_missed_up_crossing_backs_off() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone,
          "commanded OFDM");
    ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) == kRungIdxNone &&
              ConnectionAdaptiveTestAccess::upCrossingRun(c) == 2,
          "MC-DPSK came again: backed off to a run of 2, no command on this burst");
    ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone,
          "the second clean burst of the longer run commands again");
}

// BUG-MCDPSK-FALLBACK-GEOMETRY-UNHEARD: a receiver on the connect pick's MC-DPSK geometry (Robust,
// DQPSK, 4 codewords) commanded OFDM; it never decoded the sender's OFDM burst, and the sender fell
// back to the follow's down target (ROBUST_MID, DBPSK, 3 codewords). Once its decoder hears the peer
// there (3 codewords), the receiver follows: same geometry as the sender, and the failed up-crossing
// makes the next one wait longer.
void test_receiver_follows_the_peer_to_the_fallback_geometry() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::setDataFrameCodewords(c, 4);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 22.0f, 0.05f, 23.0f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone &&
              ConnectionAdaptiveTestAccess::dualUpCommanded(c),
          "the receiver commanded OFDM");
    c.noteRxMcDpskGeometry(4);
    CHECK(ConnectionAdaptiveTestAccess::dataFrameCodewords(c) == 4,
          "the session's own geometry changes nothing");
    c.noteRxMcDpskGeometry(3);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK &&
              ConnectionAdaptiveTestAccess::dataFrameCodewords(c) == 3 &&
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::MFSK16_FAST,
          "the peer is on the fallback geometry: the receiver follows it (16-FSK-F, 3 codewords)");
    CHECK(!ConnectionAdaptiveTestAccess::dualUpCommanded(c) &&
              ConnectionAdaptiveTestAccess::upCrossingRun(c) == 2,
          "the up-crossing it commanded failed: the next one waits a longer run");
}

// A crossing supersedes a pending MODE_CHANGE (a data mode of the waveform being left): kept, its
// next retry went out on MC-DPSK, the peer acknowledged it and the sender committed it - its
// connection back on OFDM while its modem stayed on MC-DPSK (GUI, dual-listen dip, seed 44).
void test_waveform_follow_crossing_supersedes_mode_change() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::setPendingModeChange(c, Modulation::QPSK, CodeRate::R1_2);
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(0)),
          "the down-crossing applies");
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c),
          "the pending MODE_CHANGE is gone with the waveform it described");

    // Belt and braces: a data-mode change that still moves the session across waveforms outside a
    // crossing takes the modem with it.
    int crossings = 0;
    WaveformMode modem_mode = WaveformMode::MC_DPSK;
    c.setWaveformCrossingCallback([&](WaveformMode mode, Modulation, CodeRate, int, int, int) {
        ++crossings;
        modem_mode = mode;
    });
    ConnectionAdaptiveTestAccess::applyDataMode(c, Modulation::QPSK, CodeRate::R1_2,
                                                LadderRungId::OFDM_CHIRP);
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::OFDM_CHIRP &&
              crossings == 1 && modem_mode == WaveformMode::OFDM_CHIRP,
          "a waveform change outside a crossing still reaches the modem");
}

// The way back up reads the burst being acknowledged, not the fade. An MC-DPSK spell's fade
// readings (-5..-1 dB) dragged the returned channel's (14 dB on Good, dial 20) down: every return
// landed on QPSK R1/4..R2/3, below the pre-fade rung (GUI, dual-listen, 6 seeds).
void test_waveform_follow_return_reads_the_latest_burst() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    c.setPhysicalChannelStats(20.0f, 1.5f, 16);
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(0)),
          "the fade crosses down");
    for (float fade : {-3.0f, -5.0f, -1.0f, -4.0f}) {  // the fade, a frame apart
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, fade, 0.6f, fade + 0.5f);
        ConnectionAdaptiveTestAccess::agePool(c, 6000);
    }
    ConnectionAdaptiveTestAccess::agePool(c, 30000);  // the channel is back
    c.setPhysicalChannelStats(18.0f, 2.0f, 6);
    for (int frame = 0; frame < 3; ++frame) {
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 14.0f, 0.6f, 18.0f);
    }
    const uint8_t rung = connection_policy::decodeOfdmRungCommand(
        ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true));
    CHECK(rung >= kRungIdxQam8R23,
          "the return lands where the returned channel says (8PSK R2/3), not on the fade's floor");
}

// With one-frame bursts the burst being acknowledged is one frame. A full window's span (3 frames +
// ACK + hold, ~24 s) still held the dip's last two one-frame bursts (a cycle is ~8.5 s), and the
// return read the fade again; the readings since this end's previous verdict are the burst's own.
void test_waveform_follow_return_reads_a_one_frame_burst() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    c.setPhysicalChannelStats(20.0f, 1.5f, 16);
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(0)),
          "the fade crosses down");
    bool stayed = true;
    for (float fade : {-3.0f, -5.0f, -1.0f}) {  // one-frame bursts in the fade, each acknowledged
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, fade, 0.6f, fade + 0.5f);
        stayed = stayed && connection_policy::decodeOfdmRungCommand(
                               ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) == kRungIdxNone;
        ConnectionAdaptiveTestAccess::agePool(c, 8500);
    }
    CHECK(stayed, "the fade's bursts stay on MC-DPSK");
    c.setPhysicalChannelStats(18.0f, 2.0f, 6);
    ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 14.0f, 0.6f, 18.0f);  // the channel is back
    const uint8_t rung = connection_policy::decodeOfdmRungCommand(
        ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true));
    CHECK(rung != kRungIdxNone && rung >= kRungIdxQam8R23,
          "the first clean one-frame burst commands OFDM where the returned channel says");
}

// The physical cap reads the frames of the burst being acknowledged. The decoder's physical ring
// spans the whole spell: after a dip it held the dip (mean 0.4 dB) and capped every returned burst at
// 2.4 dB (GUI, ITU Good dip, seed 44: two bursts reading 10-15 dB stayed on MC-DPSK, +21 s).
void test_waveform_follow_return_reads_the_burst_physical_evidence() {
    for (const float burst_physical : {16.0f, 1.0f}) {
        setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
        Connection c;
        unsetenv("ULTRA_WAVEFORM_FOLLOW");
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                        Modulation::QAM8);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
        CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
                  c, connection_policy::encodeMcDpskRungCommand(0)),
              "the fade crosses down");
        // DPSK data readings (see test_waveform_follow_down_crossing_forgets_stale_evidence).
        ConnectionAdaptiveTestAccess::setDataMode(c, Modulation::DQPSK, CodeRate::R1_4);
        for (float fade : {-3.0f, -5.0f, -1.0f}) {
            ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, fade, 0.6f, fade + 0.5f);
            (void)ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
            ConnectionAdaptiveTestAccess::agePool(c, 8500);
        }
        c.setPhysicalChannelStats(0.4f, 1.5f, 8);  // the decoder's ring: the dip
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 12.0f, 0.6f, burst_physical);
        const uint8_t rung = connection_policy::decodeOfdmRungCommand(
            ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true));
        if (burst_physical > 10.0f) {
            CHECK(rung != kRungIdxNone,
                  "a returned burst commands OFDM on its own physical evidence, not the spell's");
        } else {
            CHECK(rung == kRungIdxNone,
                  "a burst whose own training sat in the fade stays (its physics caps the boost)");
        }
    }
}

// A fallback spell's MC-DPSK frames measure no fading index (dual-listen's MC-DPSK decoder never ran an
// LTS): they carry none, and the pick keeps the session's measured channel class. Read as 0 (AWGN),
// the returned frame at 5.7 dB (physical 17.6) stayed under the AWGN floor (GUI, ITU Good, seed 46;
// the test places it just under whichever AWGN floor is active).
void test_waveform_follow_spell_keeps_the_measured_channel_class() {
    for (const bool read_as_awgn : {false, true}) {
        setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
        Connection c;
        unsetenv("ULTRA_WAVEFORM_FOLLOW");
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                        Modulation::QAM8);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
        CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
                  c, connection_policy::encodeMcDpskRungCommand(0)),
              "the fade crosses down");
        // DPSK data readings (see test_waveform_follow_down_crossing_forgets_stale_evidence).
        ConnectionAdaptiveTestAccess::setDataMode(c, Modulation::DQPSK, CodeRate::R1_4);
        const float spell_fading = read_as_awgn ? 0.0f : NAN;
        for (float fade : {-3.0f, -5.0f}) {
            ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, fade, spell_fading, fade + 0.5f);
            (void)ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
            ConnectionAdaptiveTestAccess::agePool(c, 8500);
        }
        // The returned frame reads just under the AWGN floor (5.7 dB in the GUI run, against the
        // 6 dB floor of the time), so the class it is judged in decides the crossing.
        const float returned_reading = connection_policy::ofdmEntryFloorFor(
                                           connection_policy::ChannelClassification::AWGN) -
                                       0.3f;
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, returned_reading, spell_fading, 17.6f);
        const uint8_t rung = connection_policy::decodeOfdmRungCommand(
            ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true));
        if (!read_as_awgn) {
            CHECK(rung != kRungIdxNone,
                  "on the session's measured class (Good) the returned frame commands OFDM");
        } else {
            CHECK(rung == kRungIdxNone,
                  "control: read as AWGN the same frame stays under the AWGN floor");
        }
    }
}

// BUG-FOLLOW-SPELL-CLASS-ONE-FRAME: MC-DPSK frames measure no fading, so a fallback spell keeps a class
// from its OFDM spell - the fading pooled over that spell's readings at the down-crossing, never the
// last frame's single value. ITU Good single-frame fading scatters 0.13-0.54: a last reading of 0.21,
// shrunk by the entry classification to 0.08, read AWGN, and the way back up waited for a lucky
// data-aided reading (GUI drop to 5 dB, seed 45: 586 s, then 745 s to the end, on MC-DPSK).
void test_waveform_follow_spell_class_is_pooled_over_the_ofdm_spell() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.4f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    for (float fade : {0.42f, 0.35f, 0.47f, 0.30f, 0.38f, 0.21f}) {
        ConnectionAdaptiveTestAccess::feedOfdmFrame(c, 18.0f, fade);
        ConnectionAdaptiveTestAccess::agePool(c, 3000);
    }
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              c, connection_policy::encodeMcDpskRungCommand(0)),
          "the fade crosses down");
    // A returned frame: data-aided 5.7 dB (a lower bound on fading), physical 9 dB.
    ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 5.7f, NAN, 9.0f);
    const uint8_t rung = connection_policy::decodeOfdmRungCommand(
        ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true));
    CHECK(rung != kRungIdxNone,
          "on the OFDM spell's pooled class (Good) the returned frame commands OFDM");
}

// A fallback spell sends one frame per burst while a return is likely (the burst length is the return
// latency: dips cost 8-27 s waiting for a clean 3-frame burst), the full window once it has lasted; a
// link that delivered nothing in OFDM gets the full window at once.
void test_waveform_follow_fallback_spell_starts_with_single_frames() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection delivering;
    Connection fresh;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(delivering, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(delivering);
    ConnectionAdaptiveTestAccess::setRxReassembly(delivering, 600);
    const uint32_t rto = ConnectionAdaptiveTestAccess::ackTimeoutMs(delivering);
    for (int round = 0; round < 6; ++round) {
        ConnectionAdaptiveTestAccess::tickFollow(delivering, rto);
        ConnectionAdaptiveTestAccess::deliverOfdm(delivering, 8);
    }
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              delivering, connection_policy::encodeMcDpskRungCommand(0)),
          "the delivering link crosses down");
    const uint64_t span = ConnectionAdaptiveTestAccess::fallbackSmallWindowMs(delivering);
    const size_t window = ConnectionAdaptiveTestAccess::arqWindow(delivering);
    // The break-even ran to tens of minutes here (8PSK R2/3); the operator's patience bounds it
    // (BUG-FOLLOW-SPELL-WINDOW-UNBOUNDED): a drop that outlived a minute is not a fade.
    CHECK(span == connection_policy::kFollowPatienceMs &&
              ConnectionAdaptiveTestAccess::txCap(delivering) == 1 && window > 1,
          "its MC-DPSK spell sends one-frame bursts for the patience; the receive window stays whole");
    ConnectionAdaptiveTestAccess::tickFollow(delivering, static_cast<uint32_t>(span - 1));
    CHECK(ConnectionAdaptiveTestAccess::txCap(delivering) == 1, "still single frames");
    ConnectionAdaptiveTestAccess::tickFollow(delivering, 2);
    CHECK(ConnectionAdaptiveTestAccess::txCap(delivering) == 0 &&
              ConnectionAdaptiveTestAccess::arqWindow(delivering) == window,
          "the drop outlived the patience: the full window");

    ConnectionAdaptiveTestAccess::makeConnectedOFDM(fresh, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(fresh);
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(
              fresh, connection_policy::encodeMcDpskRungCommand(0)),
          "a link that never delivered crosses down");
    CHECK(ConnectionAdaptiveTestAccess::txCap(fresh) == 0,
          "nothing to come back to: the full window at once");
}

// The ACK that launched an up-crossing comes again (MC-DPSK ACKs are repeated). While the first OFDM
// group is on air a copy cannot answer it: read as a turn boundary it queued a second group behind the
// first (GUI, seed 44). It must be rejected as a candidate, not consumed: the monitor disarms on the
// detection it delivers, and the group's own ACK then went unheard (GUI, seed 42: crossing abandoned).
void test_waveform_follow_crossing_ack_copy_is_not_a_candidate() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    std::vector<Bytes> sent;
    c.setTransmitCallback([&](const Bytes& frame) { sent.push_back(frame); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.6f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(c, Bytes{0x10}, 0),
          "the first OFDM group's frame is in flight");
    // The launching ACK closed the last MC-DPSK burst: base - 1, no progress for the OFDM frame.
    ultra::waveform::tone_burst_ack::ToneBurstAckPayload copy;
    copy.group_seq = 0x3F;
    copy.frame_mask = 0;
    copy.type = ultra::waveform::tone_burst_ack::AckType::Ack;
    copy.move_epoch = 0;
    const bool arq_verdict = c.isToneBurstAckCandidatePlausible(copy);
    ultra::waveform::tone_burst_ack::ToneBurstAckPayload own = copy;
    own.group_seq = 0;  // the group's own ACK: through the frame on air
    CHECK(arq_verdict && c.isToneBurstAckCandidatePlausible(own),
          "fixture: both are ARQ-plausible candidates");
    ConnectionAdaptiveTestAccess::armCrossingLaunch(c, copy.group_seq, copy.frame_mask, 5000);
    CHECK(!c.isToneBurstAckCandidatePlausible(copy),
          "a copy heard while the group is on air is not a candidate: the monitor keeps listening");
    CHECK(c.isToneBurstAckCandidatePlausible(own),
          "the group's own ACK stays a candidate while the group is on air");
    ConnectionAdaptiveTestAccess::armCrossingLaunch(c, copy.group_seq, copy.frame_mask, -1);
    CHECK(c.isToneBurstAckCandidatePlausible(copy) == arq_verdict,
          "after the group's air end the same ACK is the ARQ's call again (a real report)");
    (void)sent;
}

// A one-frame MC-DPSK resend ends its physical turn and must say so: without PHYSICAL_BURST_END the
// receiver waited for a continuation and sent no ACK until the next RTO (GUI 60 s outage, one-frame
// fallback bursts: two resends lost 22 s each). The burst encoder stamps bursts; the timeout path
// sent single frames unstamped.
void test_mcdpsk_one_frame_resend_ends_its_turn() {
    Connection c;
    std::vector<Bytes> sent;
    c.setTransmitCallback([&](const Bytes& frame) { sent.push_back(frame); });
    // As the GUI and TNC wire it: the burst path owns timeout resends (handleArqTimeoutBatch).
    c.setTransmitBurstCallback([&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
        for (const auto& f : frames) sent.push_back(f);
    });
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(c, Bytes(40, 0x5A), 0),
          "one MC-DPSK DATA frame in flight");
    const size_t first = sent.size();
    for (int ms = 0; ms < 120000 && sent.size() == first; ms += 100) c.tick(100);
    CHECK(sent.size() > first, "the retransmission timeout resends it");
    if (sent.size() > first) {
        const auto resend = v2::DataFrame::deserialize(sent.back());
        CHECK(resend && (resend->flags & v2::Flags::PHYSICAL_BURST_END) != 0,
              "the one-frame resend carries the end-of-turn marker");
    }
}

// An OFDM single frame is its whole physical turn and must say so, first send and resend alike
// (BUG-OFDM-SINGLETON-UNMARKED): frame-structure acquisition finds every frame, so an anchor no
// longer proves a standalone frame and the receiver held the unmarked frame's ACK to the
// descriptorless air ceiling (~11 s), past the sender's ACK window (GUI ITU Good 5 dB: the file's
// last frame decoded three times, its ACK dropped on each resend). Wide and narrow OFDM alike.
void test_ofdm_single_frame_ends_its_turn() {
    for (int narrow = 0; narrow < 2; ++narrow) {
        Connection c;
        std::vector<Bytes> sent;
        size_t burst_frames = 0;  // frames of the latest burst callback (0 = a single frame)
        c.setTransmitCallback([&](const Bytes& frame) {
            sent.push_back(frame);
            burst_frames = 0;
        });
        c.setTransmitBurstCallback([&](const std::vector<Bytes>& frames, uint16_t, uint8_t) {
            for (const auto& f : frames) sent.push_back(f);
            burst_frames = frames.size();
        });
        if (narrow) {
            ConnectionAdaptiveTestAccess::makeConnectedNarrowOFDM(c);
        } else {
            ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 8.0f, 0.4f,
                                                            Modulation::QPSK);
        }
        const char* which = narrow ? "narrow" : "wide";
        CHECK(ConnectionAdaptiveTestAccess::sendBufferedSingleFrame(c, Bytes(40, 0x5A)),
              "one OFDM DATA frame in flight");
        CHECK(sent.size() == 1, "the single frame goes out on its own");
        if (!sent.empty()) {
            const auto first = v2::DataFrame::deserialize(sent.back());
            CHECK(first && (first->flags & v2::Flags::PHYSICAL_BURST_END) != 0,
                  std::string(which) + ": the single frame carries the end-of-turn marker");
        }
        const size_t before = sent.size();
        for (int ms = 0; ms < 120000 && sent.size() == before; ms += 100) c.tick(100);
        CHECK(sent.size() > before, "the retransmission timeout resends it");
        if (sent.size() > before && !narrow) {
            // Wide QPSK R1/2 is above the bottom rung: the resend after a silent round is a
            // two-frame group (frame + addressed-away pad) so a descriptor carries it
            // (BUG-LATENT-CRATER-ONE-RUNG); the burst encoder stamps that group's end.
            CHECK(burst_frames == 2 &&
                      v2::isOFDMBurstPadHeader(v2::parseHeader(sent.back())),
                  std::string(which) + ": the one-frame resend leaves as a padded group");
        } else if (sent.size() > before) {
            const auto resend = v2::DataFrame::deserialize(sent.back());
            CHECK(resend && (resend->flags & v2::Flags::PHYSICAL_BURST_END) != 0,
                  std::string(which) + ": the one-frame resend carries the end-of-turn marker");
        }
    }
}

// After a crossing the EVM window starts empty, and one group is one fade realization: it may lift
// the rung by one, not by three. One group certified 8PSK R2/3 -> 32QAM R2/3 and the next 32QAM group
// failed whole (GUI, ITU Good 20 dB, two seeds). A second group that agrees may jump.
void test_first_ofdm_group_climbs_one_rung() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 22.0f, 0.5f,
                                                    Modulation::QAM8);
    c.setBurstChannelObservation(22.0f, 0.5f, 0.9f, true, 0.1f);
    const uint8_t incumbent = kRungIdxQam8R23;
    const uint8_t one_up = snapRungIndexUpToEnabled(incumbent);
    const std::vector<float> crest(5, 26.0f);
    const uint8_t first = ConnectionAdaptiveTestAccess::authorityAfterGroup(c, crest);
    CHECK(first != kRungIdxNone && first <= one_up,
          "one group of crest frames lifts the rung by one at most");
    const uint8_t second = ConnectionAdaptiveTestAccess::authorityAfterGroup(c, crest);
    CHECK(second > one_up, "a second group that agrees lets the climb go further");
}

// Retransmission timeouts in a row are no evidence against the rung: a dead channel kills every
// rung and the escape's MODE_CHANGE cannot land in it. Queued anyway, it committed at the return
// over a link whose next burst decoded at the old rung and collided with that burst's ACK (GUI,
// follow off, ITU Good dip: 22 s). An escape needs the latest round to have drawn an ACK.
void test_escape_needs_the_peer_heard() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.5f,
                                                    Modulation::QAM8);
    CHECK(ConnectionAdaptiveTestAccess::escapeEvidence(c), "a fresh link has heard its peer");
    ConnectionAdaptiveTestAccess::noteRound(c, 0, "rto");
    CHECK(!ConnectionAdaptiveTestAccess::escapeEvidence(c),
          "a round that drew no ACK: no evidence against the rung");
    ConnectionAdaptiveTestAccess::noteRound(c, 0, "toneburst-ack");
    CHECK(ConnectionAdaptiveTestAccess::escapeEvidence(c),
          "a crater ACK (no progress) is the peer answering: evidence");
    ConnectionAdaptiveTestAccess::noteRound(c, 2, "toneburst-ack");
    ConnectionAdaptiveTestAccess::noteRound(c, 0, "rto");
    ConnectionAdaptiveTestAccess::noteRound(c, 0, "rto");
    CHECK(!ConnectionAdaptiveTestAccess::escapeEvidence(c), "two silent rounds since the last ACK");

}

// The stuck-frame escape counts the ACK rounds a frame was missing from, never retransmissions
// into silence: GUI, follow off, ITU Good dips of 100-145 s - the first ACK after the return carried
// 8-11 silent "retx" and dropped 8PSK R2/3 -> QPSK R3/4, its MODE_CHANGE colliding twice.
void test_stuck_frame_escape_counts_heard_misses() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.5f,
                                                    Modulation::QAM8);
    ConnectionAdaptiveTestAccess::allowRateAdaptation(c);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(c, Bytes(40, 0x21), v2::Flags::MORE_FRAG),
          "a frame in flight (seq 0)");
    // Silent rounds until a frame has been resent six times (the modelled air schedule stretches
    // each deadline behind the queued resends).
    for (int r = 0; r < 60 && ConnectionAdaptiveTestAccess::arqRetries(c) < 6; ++r) {
        ConnectionAdaptiveTestAccess::arqTimeoutRound(c);
    }
    CHECK(ConnectionAdaptiveTestAccess::arqRetries(c) >= 6 &&
              ConnectionAdaptiveTestAccess::arqHeardMisses(c) == 0,
          "six silent resends, no heard miss");
    ConnectionAdaptiveTestAccess::arqAckWithout(c, 0);
    ConnectionAdaptiveTestAccess::noteRound(c, 0, "toneburst-ack");
    ConnectionAdaptiveTestAccess::pollStuckEscape(c);
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c) &&
              c.getDataModulation() == Modulation::QAM8 && c.getDataCodeRate() == CodeRate::R2_3,
          "the first (empty) ACK after the silence: one heard miss, six silent retries do not count");
    for (int r = 0; r < 4; ++r) {
        ConnectionAdaptiveTestAccess::arqAckWithout(c, 0);
        ConnectionAdaptiveTestAccess::noteRound(c, 0, "toneburst-ack");
    }
    CHECK(ConnectionAdaptiveTestAccess::arqHeardMisses(c) >= 5, "five ACKs without seq 0");
    ConnectionAdaptiveTestAccess::pollStuckEscape(c);
    CHECK(ConnectionAdaptiveTestAccess::modeChangePending(c) ||
              c.getDataModulation() != Modulation::QAM8 || c.getDataCodeRate() != CodeRate::R2_3,
          "missing from five ACKs the peer sent: the stuck-frame escape drops a rung");
}

// ULTRA_MCDPSK_ACK_REPAIR, receiver side: a burst whose last frames were lost (the decoder saw its
// tail) gets one end-of-turn ACK, scored as a loss - never a clean burst, so never an up-crossing;
// a lost first frame gets an asynchronous one (listen-before-ACK holds it while the burst may still
// be on the air). Only a receiver the peer is sending to; nothing with the knob off.
void test_mcdpsk_turn_loss_ack() {
    struct Sent { bool group_complete; uint8_t word; };
    auto build = [](bool knob, std::vector<Sent>& sent) {
        setenv("ULTRA_MCDPSK_ACK_REPAIR", knob ? "1" : "0", 1);
        auto c = std::make_unique<Connection>();
        unsetenv("ULTRA_MCDPSK_ACK_REPAIR");
        c->setTransmitToneBurstAckCallback(
            [&sent](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& p, bool complete) {
                sent.push_back({complete, p.rate_hint});
            });
        ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(*c);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(*c);
        return c;
    };
    std::vector<Sent> sent;
    auto c = build(true, sent);
    ConnectionAdaptiveTestAccess::turnLoss(*c, true);
    CHECK(sent.empty(), "no incoming object: no turn-loss ACK");
    ConnectionAdaptiveTestAccess::setRxReassembly(*c, 600);
    // Two frames of the burst decoded with readings that would command OFDM on a clean burst; its
    // last frame lost.
    for (int i = 0; i < 3; ++i) ConnectionAdaptiveTestAccess::feedMcDpskFrame(*c, 22.0f, 0.05f, 23.0f);
    ConnectionAdaptiveTestAccess::framesDecoded(*c, 2);
    sent.clear();
    ConnectionAdaptiveTestAccess::turnLoss(*c, true);
    CHECK(sent.size() == 1 && sent[0].group_complete,
          "the burst's last frames lost: one end-of-turn ACK");
    CHECK(!ConnectionAdaptiveTestAccess::dualUpCommanded(*c),
          "a burst with lost frames is no clean burst: no up-crossing command");
    ConnectionAdaptiveTestAccess::turnLoss(*c, false);
    CHECK(sent.size() == 2 && !sent[1].group_complete,
          "a lost first frame: one asynchronous ACK");

    std::vector<Sent> sent_off;
    auto off = build(false, sent_off);
    ConnectionAdaptiveTestAccess::setRxReassembly(*off, 600);
    ConnectionAdaptiveTestAccess::turnLoss(*off, true);
    ConnectionAdaptiveTestAccess::turnLoss(*off, false);
    CHECK(sent_off.empty(), "knob off: no turn-loss ACK");
}

// A copy of the ACK that launched a latent startup probe is not a monitor candidate while the probe
// group is airborne: consumed after delivery, it had already disarmed the monitor, and the probe's
// own verdict would have gone unheard (BUG-STARTUP-PROBE-COPY-DISARMS-MONITOR; the crossing ACK's
// copy showed what follows).
// BUG-TAIL-DEMOTE-VIA-MODE-CHANGE: at the file tail every chunk is out and none acknowledged.
// tryDescriptorModeSwitch counted only the unchunked bytes (0 there) and refused, so the receiver's
// demote fell back to a MODE_CHANGE exchange that retried into the trough it answered (GUI QSB:
// four 10 s retries, 43 s). The commit's move-epoch regrid re-queues the in-flight frames, which the
// new geometry carries in a descriptor-bearing burst: the demote must commit by descriptor.
void test_tail_demote_commits_by_descriptor() {
    Connection c;
    std::vector<Bytes> tx_frames;
    c.setTransmitCallback([&](const Bytes& d) { tx_frames.push_back(d); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 20.0f, 0.30f,
                                                    Modulation::QAM8);
    ultra::test::TempDir dir("tail_demote");
    CHECK(dir.valid(), "temp dir");
    const auto path = (std::filesystem::path(dir.path()) / "payload.bin").string();
    {
        std::ofstream out(path, std::ios::binary);
        for (size_t i = 0; i < 1100; ++i) out.put(static_cast<char>((i * 37 + 11) & 0xFF));
    }
    CHECK(ConnectionAdaptiveTestAccess::startFile(c, path), "start the file");
    for (int i = 0; i < 4 && !ConnectionAdaptiveTestAccess::fileFullyChunked(c); ++i) {
        ConnectionAdaptiveTestAccess::pumpFile(c);
    }
    CHECK(ConnectionAdaptiveTestAccess::fileFullyChunked(c) &&
              ConnectionAdaptiveTestAccess::arqInFlightBytes(c) > 0,
          "the file tail: every chunk out, none acknowledged");
    size_t mode_changes = 0;
    tx_frames.clear();
    const auto switches_before = c.getStats().descriptor_mode_switches;
    ConnectionAdaptiveTestAccess::obeyAuthority(c, kRungIdxQpskR23, /*accepted_clean_round=*/false);
    for (const auto& f : tx_frames) {
        const auto hdr = v2::parseHeader(f);
        if (hdr.valid && hdr.type == v2::FrameType::MODE_CHANGE) ++mode_changes;
    }
    CHECK(mode_changes == 0 && !ConnectionAdaptiveTestAccess::modeChangePendingNow(c),
          "no MODE_CHANGE exchange for a demote at the file tail");
    CHECK(c.getDataModulation() == Modulation::QPSK && c.getDataCodeRate() == CodeRate::R2_3 &&
              c.getStats().descriptor_mode_switches == switches_before + 1,
          "the demote committed at once, by descriptor");
}

// An MC-DPSK up-step probes with the lower rung's window (mcDpskProbeBurstFrames): with 4 codewords
// per frame DBPSK carries 2 frames per burst and DQPSK 3, so the first DQPSK bursts carry 2 until an
// ACK newer than the commanding one keeps DQPSK (4 codewords: the entry pick's Robust rung). At the
// 3 codewords almost every session uses both windows are 3 and nothing is capped.
void test_mcdpsk_up_step_probes_small() {
    // Rung indices by modulation (16-FSK became the bottom rung on 2026-11-22).
    const int kDbpsk = connection_policy::mcDpskRungIndexFor(Modulation::MFSK16_FAST);
    const int kDqpsk = connection_policy::mcDpskRungIndexFor(Modulation::DQPSK);
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::setDataFrameCodewords(c, 4);
    CHECK(ConnectionAdaptiveTestAccess::txInFlightCap(c) == 0, "no cap at DBPSK");
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(c, kDqpsk, 5);
    CHECK(ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::DQPSK, "steps up to DQPSK");
    CHECK(ConnectionAdaptiveTestAccess::txInFlightCap(c) == 2,
          "the probe carries the DBPSK window (2 frames), not DQPSK's 3");
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(c, kDqpsk, 5);
    CHECK(ConnectionAdaptiveTestAccess::txInFlightCap(c) == 2,
          "a repeated copy of the commanding ACK confirms nothing");
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(c, kDqpsk, 8);
    CHECK(ConnectionAdaptiveTestAccess::txInFlightCap(c) == 0,
          "the probe's own ACK keeps DQPSK: the full window");
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(c, kDbpsk, 11);
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(c, kDqpsk, 14);
    CHECK(ConnectionAdaptiveTestAccess::txInFlightCap(c) == 2, "a second up-step probes small again");
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(c, kDbpsk, 17);
    CHECK(ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::MFSK16_FAST &&
              ConnectionAdaptiveTestAccess::txInFlightCap(c) == 0,
          "a failed probe steps down and clears the cap");

    Connection p;
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(p);
    ConnectionAdaptiveTestAccess::setDataFrameCodewords(p, 3);  // the production MC-DPSK frame
    ConnectionAdaptiveTestAccess::obeyMcDpskRung(p, kDqpsk, 5);
    CHECK(ConnectionAdaptiveTestAccess::dataModulation(p) == Modulation::DQPSK &&
              ConnectionAdaptiveTestAccess::txInFlightCap(p) == 0,
          "at 3 codewords both windows are 3: no cap (unchanged behaviour)");
}

void test_startup_probe_launch_copy_is_not_a_candidate() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 20.0f, 0.5f,
                                                    Modulation::QPSK);
    CHECK(ConnectionAdaptiveTestAccess::sendFixedData(c, Bytes{0x10}, 0), "a frame in flight");
    ultra::waveform::tone_burst_ack::ToneBurstAckPayload flagged;
    flagged.group_seq = 0x3F;
    flagged.type = ultra::waveform::tone_burst_ack::AckType::Ack;
    const uint8_t word = static_cast<uint8_t>(kRungIdxQpskR23 | kRungAuthorityStartupProbeFlag);
    flagged.rate_hint = word & 0x7;
    flagged.rung_cmd = (word >> 3) & 0x3;
    ultra::waveform::tone_burst_ack::ToneBurstAckPayload plain = flagged;
    plain.rate_hint = kRungIdxQpskR23 & 0x7;
    plain.rung_cmd = (kRungIdxQpskR23 >> 3) & 0x3;
    const bool arq_verdict = c.isToneBurstAckCandidatePlausible(flagged);
    CHECK(arq_verdict, "fixture: the payload is ARQ-plausible");
    ConnectionAdaptiveTestAccess::setStartupProbeAirborne(c, true);
    CHECK(!c.isToneBurstAckCandidatePlausible(flagged),
          "while the probe is airborne a flagged copy is not a candidate");
    CHECK(c.isToneBurstAckCandidatePlausible(plain),
          "an unflagged ACK (the probe's own verdict) still is");
    ConnectionAdaptiveTestAccess::setStartupProbeAirborne(c, false);
    CHECK(c.isToneBurstAckCandidatePlausible(flagged) == arq_verdict,
          "with no probe airborne the flag changes nothing");
}

}  // namespace

// BUG-BINARY-FRAGMENT-OVERSIZE: a binary block (the TNC's data port) was cut to the compact DATA
// frame's capacity but sent in DATA_START/DATA_CONT/DATA_END frames, which keep the smaller v2
// layout, so the encoder truncated every fragment and no block over one frame arrived. Binary blocks
// now ride plain DATA frames behind their BINARY_OBJECT identity: every fragment must be a DATA frame
// the session's compact encoder accepts whole. Here (production defaults) because the compact layout
// exists only with the turbo data FEC.
void test_binary_fragments_fit_the_v2_frame() {
    for (const CodeRate rate : {CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4}) {
        Connection c;
        std::vector<Bytes> frames;
        c.setTransmitCallback([&](const Bytes& frame) { frames.push_back(frame); });
        c.setTransmitBurstCallback([&](const std::vector<Bytes>& group, uint16_t, uint8_t) {
            frames.insert(frames.end(), group.begin(), group.end());
        });
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, rate, 20.0f, 0.10f, Modulation::QPSK);
        Bytes block(900);
        for (size_t i = 0; i < block.size(); ++i) block[i] = static_cast<uint8_t>(i * 7 + 3);
        CHECK(c.sendBinary(block), "900-byte binary block should be accepted");
        for (int i = 0; i < 20 && frames.empty(); ++i) {
            c.tick(100);
        }
        CHECK(frames.size() >= 2, "a 900-byte block should go out as several fragments");
        const int cw = ConnectionAdaptiveTestAccess::physicalDataFrameCWCount(c);
        const int z = c.selectBurstLiftingZ();
        for (const auto& f : frames) {
            const auto header = v2::parseHeader(f);
            CHECK(header.valid && header.type == v2::FrameType::DATA,
                  "a binary fragment must be a plain DATA frame");
            protocol::ofdm_frame::Position position;
            position.index = 0;
            position.count = 1;
            CHECK(protocol::ofdm_frame::encode(f, rate, cw, z, position).has_value(),
                  "every binary fragment must fit the compact frame the encoder sends it in");
        }
    }
}

// BUG-DISCONNECT-LOST-IN-FADE: the last useful DISCONNECT retry of a dual-listen OFDM session goes
// out on the robust waveform (MC-DPSK), the earlier copies on the session's OFDM.
// BUG-TURN-CONTROL-LOST-AT-OFDM-FLOOR: a TURN_REQUEST retry into an OFDM link silent for a whole
// retry interval rides MC-DPSK; never while the peer is being heard (a healthy peer holding the turn
// is sending, and a robust frame would pull its dual-listen receiver down mid-transfer).
void test_turn_request_retry_rides_robust_waveform_only_into_silence() {
    for (const bool peer_heard : {false, true}) {
        setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
        Connection c;
        unsetenv("ULTRA_WAVEFORM_FOLLOW");
        int robust = 0;
        std::vector<int> robust_at_request;
        c.setTransmitCallback([&](const Bytes& f) {
            const auto h = v2::parseHeader(f);
            if (h.valid && h.type == v2::FrameType::TURN_REQUEST) robust_at_request.push_back(robust);
        });
        c.setRobustControlOnceCallback([&](Modulation) { ++robust; });
        ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 3.0f, 0.0f,
                                                        Modulation::QPSK);
        ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
        ConnectionAdaptiveTestAccess::makeLocalIrsWaitingForTurn(c);
        CHECK(c.sendMessage("after bulk"), "a message queues while the peer holds the turn");
        const uint32_t retry_ms = ConnectionAdaptiveTestAccess::turnRequestRetransmitMs(c);
        for (int i = 0; i < 40 && robust_at_request.size() < 3; ++i) {
            if (peer_heard) ConnectionAdaptiveTestAccess::notePeerHeard(c);
            c.tick(retry_ms / 4 + 1);
        }
        CHECK(robust_at_request.size() >= 3, "the request is retried");
        if (robust_at_request.size() < 3) continue;
        CHECK(robust_at_request[0] == 0, "the first request goes on the session waveform");
        if (peer_heard) {
            CHECK(robust == 0, "no robust retry while the peer is heard");
        } else {
            CHECK(robust_at_request[1] == 1, "the first retry into silence rides MC-DPSK");
        }
    }
}

// BUG-REPLY-ON-WRONG-WAVEFORM: in an OFDM session a control frame that arrived on MC-DPSK is
// answered on MC-DPSK (DISCONNECT ack, TURNOVER); one that arrived on OFDM is answered on OFDM.
void test_control_reply_rides_the_arrival_waveform() {
    for (const bool arrived_mcdpsk : {true, false}) {
        for (const v2::FrameType req : {v2::FrameType::DISCONNECT, v2::FrameType::TURN_REQUEST}) {
            setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
            Connection c;
            unsetenv("ULTRA_WAVEFORM_FOLLOW");
            int robust = 0;
            int robust_at_reply = -1;
            const v2::FrameType want = req == v2::FrameType::DISCONNECT ? v2::FrameType::ACK
                                                                        : v2::FrameType::TURNOVER;
            c.setTransmitCallback([&](const Bytes& f) {
                const auto h = v2::parseHeader(f);
                if (h.valid && h.type == want && robust_at_reply < 0) robust_at_reply = robust;
            });
            c.setRobustControlOnceCallback([&](Modulation) { ++robust; });
            ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 3.0f, 0.0f,
                                                            Modulation::QPSK);
            ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
            ConnectionAdaptiveTestAccess::makeLocalIss(c);  // holds the turn, nothing to send
            const Bytes frame = req == v2::FrameType::DISCONNECT
                                    ? v2::ControlFrame::makeDisconnect("K2DEF", "W1ABC").serialize()
                                    : v2::ControlFrame::makeTurnRequest("K2DEF", "W1ABC").serialize();
            c.noteRxFrameMcDpsk(arrived_mcdpsk);
            c.onFrameReceived(frame, false);
            for (int i = 0; i < 20 && robust_at_reply < 0; ++i) c.tick(500);
            const char* name = req == v2::FrameType::DISCONNECT ? "DISCONNECT ack" : "TURNOVER";
            CHECK(robust_at_reply >= 0, std::string(name) + " is sent");
            if (arrived_mcdpsk) {
                CHECK(robust_at_reply == 1,
                      std::string(name) + " to an MC-DPSK request rides MC-DPSK");
            } else {
                CHECK(robust_at_reply == 0, std::string(name) + " to an OFDM request stays on OFDM");
            }
        }
    }
}

// BUG-MESSAGE-ABANDONED-ON-FOLLOW-FLAP: a message in flight is re-gridded across every crossing the
// receiver commands; only failure-driven geometry changes spend its regrid allowance (TNC pingpong,
// ITU Good 8 dB: the third crossing abandoned a 180-byte message on a delivering link).
void test_message_survives_receiver_commanded_crossings() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    int failed = 0;
    c.setMessageTxStatusCallback([&](const Connection::MessageTxStatusEvent& e) {
        if (e.status == Connection::MessageTxStatus::FAILED) ++failed;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 8.0f, 0.4f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    CHECK(c.sendMessage(std::string(180, 'm')), "the message is accepted");
    for (int i = 0; i < 4; ++i) c.tick(250);
    int crossings = 0;
    for (int round = 0; round < 3; ++round) {
        crossings += ConnectionAdaptiveTestAccess::obeyCrossing(
                         c, connection_policy::encodeMcDpskRungCommand(0)) ? 1 : 0;
        for (int i = 0; i < 4; ++i) c.tick(250);
        crossings += ConnectionAdaptiveTestAccess::obeyCrossing(c, kRungIdxQpskR12) ? 1 : 0;
        for (int i = 0; i < 4; ++i) c.tick(250);
    }
    CHECK(crossings == 6, "six receiver-commanded crossings");
    CHECK(failed == 0, "no crossing the receiver commanded abandons the message");
}

// The robust copy goes out at the constellation the peer's MC-DPSK decoder runs: its listener's
// ROBUST_MID normally, the rung the session left while this end's up-crossing is unconfirmed (the
// peer's MC-DPSK decoder is still its primary there). TNC reconnect, AWGN 3 dB seed 301.
void test_robust_control_targets_the_peer_mcdpsk_decoder() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    std::vector<Modulation> sent;
    c.setRobustControlOnceCallback([&](Modulation m) { sent.push_back(m); });
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::setDataMode(c, Modulation::DQPSK, CodeRate::R1_4);
    CHECK(ConnectionAdaptiveTestAccess::obeyCrossing(c, kRungIdxQpskR14), "the sender crosses up");
    CHECK(c.robustControlModulationForTest() == Modulation::DQPSK,
          "an unconfirmed up-crossing: robust control at the DQPSK rung the peer still runs");
    // BUG-CLOSE-DURING-UNCONFIRMED-UPCROSS: a close in that state returns to the peer's waveform.
    c.disconnect();
    CHECK(ConnectionAdaptiveTestAccess::negotiatedMode(c) == WaveformMode::MC_DPSK &&
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::DQPSK,
          "the close goes back to MC-DPSK DQPSK, the waveform the peer's primary runs");
    const uint32_t retry_ms = ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(c);
    for (int i = 0; i < 4; ++i) c.tick(retry_ms);
    CHECK(sent.empty(), "every copy is MC-DPSK already: no robust-once switch is needed");

    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection d;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    std::vector<Modulation> sent_d;
    d.setRobustControlOnceCallback([&](Modulation m) { sent_d.push_back(m); });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(d, CodeRate::R1_2, 15.0f, 0.10f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(d);
    d.disconnect();
    for (int i = 0; i < 4; ++i) d.tick(ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(d));
    CHECK(sent_d.size() == 1 &&
              sent_d[0] == connection_policy::ladderRungForId(LadderRungId::ROBUST_MID).modulation,
          "a settled OFDM session: the robust copy at the listener's ROBUST_MID");
}

// BUG-MCDPSK-CLOSE-STUCK-ON-UNPROVEN-RUNG: an unanswered MC-DPSK close steps one rung down per retry.
void test_mcdpsk_close_retries_step_down() {
    Connection c;
    ConnectionAdaptiveTestAccess::makeConnectedMCDPSK(c);
    ConnectionAdaptiveTestAccess::setDataMode(c, Modulation::DQPSK, CodeRate::R1_4);
    std::vector<Modulation> at_copy;
    c.setTransmitCallback([&](const Bytes& f) {
        const auto h = v2::parseHeader(f);
        if (h.valid && h.type == v2::FrameType::DISCONNECT) {
            at_copy.push_back(ConnectionAdaptiveTestAccess::dataModulation(c));
        }
    });
    c.disconnect();
    for (int i = 0; i < 400 && at_copy.size() < 3; ++i) c.tick(500);
    CHECK(at_copy.size() >= 3, "the close is retried");
    if (at_copy.size() >= 3) {
        CHECK(at_copy[0] == Modulation::DQPSK, "the first copy at the session rung");
        CHECK(at_copy[1] == Modulation::MFSK16_FAST, "the first retry one rung down (16-FSK-F)");
        CHECK(at_copy[2] == Modulation::MFSK16, "the next at the bottom rung (16-FSK)");
    }
}

// The commanded rung applied at its MODE_CHANGE ACK keeps the receiver-command provenance (TNC short,
// ITU Good 15 dB seed 412: the third such change abandoned a 900-byte message).
void test_message_survives_commanded_mode_changes() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    int failed = 0;
    c.setMessageTxStatusCallback([&](const Connection::MessageTxStatusEvent& e) {
        if (e.status == Connection::MessageTxStatus::FAILED) ++failed;
    });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R2_3, 15.0f, 0.4f, Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);
    ConnectionAdaptiveTestAccess::makeLocalIss(c);
    CHECK(c.sendMessage(std::string(900, 'm')), "the message is accepted");
    for (int i = 0; i < 4; ++i) c.tick(250);
    const CodeRate rates[] = {CodeRate::R1_2, CodeRate::R2_3, CodeRate::R1_2, CodeRate::R2_3};
    for (const CodeRate r : rates) {
        ConnectionAdaptiveTestAccess::commandedModeChangeAcked(c, Modulation::QPSK, r);
        for (int i = 0; i < 4; ++i) c.tick(250);
    }
    CHECK(failed == 0, "receiver-commanded MODE_CHANGEs never abandon the message");
}

void test_last_disconnect_retry_rides_robust_waveform() {
    setenv("ULTRA_WAVEFORM_FOLLOW", "2", 1);  // the follow knob latches per Connection
    Connection c;
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    int robust = 0;
    std::vector<int> robust_at_copy;
    int copies = 0;
    c.setTransmitCallback([&](const Bytes& f) {
        const auto h = v2::parseHeader(f);
        if (h.valid && h.type == v2::FrameType::DISCONNECT) {
            ++copies;
            robust_at_copy.push_back(robust);
        }
    });
    c.setRobustControlOnceCallback([&](Modulation) { ++robust; });
    ConnectionAdaptiveTestAccess::makeConnectedOFDM(c, CodeRate::R1_2, 15.0f, 0.10f,
                                                    Modulation::QPSK);
    ConnectionAdaptiveTestAccess::allowWaveformFollow(c);  // an automatic-rate session
    CHECK(ConnectionAdaptiveTestAccess::dualListen(c), "the fixture runs dual-listen");
    c.disconnect();
    const uint32_t retry_ms = ConnectionAdaptiveTestAccess::disconnectRetryIntervalMs(c);
    for (int i = 0; i < 4; ++i) c.tick(retry_ms);
    const int useful = ConnectionAdaptiveTestAccess::disconnectUsefulRetryCount(c);
    CHECK(copies == 1 + useful, "the initial DISCONNECT plus every useful retry is sent");
    CHECK(robust == 1, "exactly one copy rides the robust waveform");
    CHECK(!robust_at_copy.empty() && robust_at_copy.back() == 1 &&
              (robust_at_copy.size() < 2 || robust_at_copy[robust_at_copy.size() - 2] == 0),
          "it is the last useful retry");
}

int main() {
    // Production defaults on purpose (see the header); only the follow knob is set per test.
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    if (!connection_policy::waveformFollowEnabled() || !connection_policy::waveformFollowDualListen()) {
        std::cout << "WaveformFollow: the default must be dual-listen follow\n";
        return 1;
    }
    test_waveform_follow_sender_crossings_switch_modem_first();
    test_ofdm_probe_does_not_punish_mcdpsk_rung();
    test_last_disconnect_retry_rides_robust_waveform();
    test_turn_request_retry_rides_robust_waveform_only_into_silence();
    test_control_reply_rides_the_arrival_waveform();
    test_message_survives_receiver_commanded_crossings();
    test_message_survives_commanded_mode_changes();
    test_robust_control_targets_the_peer_mcdpsk_decoder();
    test_mcdpsk_close_retries_step_down();
    test_waveform_follow_knob_off_ignores_crossing_words();
    test_waveform_follow_receiver_commands_up_and_reverts();
    test_waveform_follow_first_ofdm_group_settles_probation();
    test_waveform_follow_receiver_empty_floor_commands_down();
    test_waveform_follow_down_crossing_forgets_stale_evidence();
    test_waveform_follow_receiver_silence_needs_an_incoming_object();
    test_waveform_follow_mid_burst_acks_are_no_verdict();
    test_waveform_follow_delivering_link_rides_out_a_silence();
    test_waveform_follow_empty_groups_follow_the_economics();
    test_waveform_follow_sender_waits_out_a_fade();
    test_waveform_follow_sender_silence_ends_when_the_peer_is_heard();
    test_dual_listen_up_crossing_needs_a_reading_from_the_burst();
    test_stuck_frame_escape_counts_heard_misses();
    test_mcdpsk_turn_loss_ack();
    test_dual_listen_receiver_commands_up_and_follows_the_peer();
    test_dual_listen_receiver_commands_down_and_follows_the_peer();
    test_dual_listen_missed_up_crossing_backs_off();
    test_waveform_follow_crossing_supersedes_mode_change();
    test_waveform_follow_return_reads_the_latest_burst();
    test_waveform_follow_return_reads_a_one_frame_burst();
    test_waveform_follow_return_reads_the_burst_physical_evidence();
    test_waveform_follow_spell_keeps_the_measured_channel_class();
    test_waveform_follow_spell_class_is_pooled_over_the_ofdm_spell();
    test_waveform_follow_fallback_spell_starts_with_single_frames();
    test_waveform_follow_crossing_ack_copy_is_not_a_candidate();
    test_mcdpsk_one_frame_resend_ends_its_turn();
    test_ofdm_single_frame_ends_its_turn();
    test_receiver_follows_the_peer_to_the_fallback_geometry();
    test_first_ofdm_group_climbs_one_rung();
    test_escape_needs_the_peer_heard();
    test_startup_probe_launch_copy_is_not_a_candidate();
    test_mcdpsk_up_step_probes_small();
    test_tail_demote_commits_by_descriptor();
    test_binary_fragments_fit_the_v2_frame();

    if (tests_failed != 0) {
        std::cout << "WaveformFollow: " << tests_failed << "/" << tests_run << " failed\n";
        return 1;
    }
    std::cout << "WaveformFollow: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
