// Waveform follow (ULTRA_WAVEFORM_FOLLOW): a session crosses between MC-DPSK and OFDM as the channel
// moves (docs/WAVEFORM_FOLLOW_DESIGN_2026_11_11.md, INV-FOLLOW-001).
//
// Its own binary because follow rides the receiver-authority ACK word: the commands are the
// authority's rung bits, and only an authority sender reads them. ConnectionAdaptive pins
// ULTRA_RX_RATE_AUTHORITY=0 for its legacy-driver tests; this binary runs the production defaults
// (authority on, ULTRA_FAST on), the configuration follow ships in.

#include "protocol/connection.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "waveform/tone_burst_ack/tone_burst_ack_monitor.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
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

    // ── Waveform follow ──
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
    static void noteOfdmGroup(Connection& c, bool delivered_any) {
        c.noteRxOfdmBurstAfterCrossing(delivered_any);
        c.noteRxOfdmGroupForFollow(delivered_any);
    }
    static uint8_t pendingCrossingWord(const Connection& c) {
        return c.rx_crossing_apply_pending_ ? c.rx_crossing_word_ : 0;
    }
    static void tickFollow(Connection& c, uint32_t ms) { c.tickWaveformFollow(ms); }
    static void setRxReassembly(Connection& c, size_t bytes) {
        c.rx_reassembly_buffer_.assign(bytes, 0x5A);
    }
    static void setDataMode(Connection& c, Modulation mod, CodeRate rate) {
        c.data_modulation_ = mod;
        c.data_code_rate_ = rate;
    }
    static bool crossingProbeActive(const Connection& c) { return c.crossing_probe_active_; }
    static uint32_t crossingRevertMs(const Connection& c) { return c.crossing_probe_revert_ms_; }
    static int upCrossingRun(const Connection& c) { return c.up_crossing_gate_.run(); }
    static void failUpCrossing(Connection& c) { c.up_crossing_gate_.crossingFailed(); }
    static void noteRxWaveform(Connection& c, WaveformMode mode) { c.noteRxWaveform(mode); }
    static bool dualListen(const Connection& c) { return c.dualListenActive(); }
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
    static void pollCollapseEscape(Connection& c) { c.maybeCollapseEscape(); }
    static void pollStuckEscape(Connection& c) { c.maybeEscapeStuckFrame(); }
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
        c.crossing_probe_active_ = true;
        c.crossing_probe_rx_side_ = false;
        c.crossing_launch_ack_valid_ = true;
        c.crossing_launch_ack_seq_ = seq;
        c.crossing_launch_ack_mask_ = mask;
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
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::DBPSK &&
              ConnectionAdaptiveTestAccess::dataCodeRate(c) == CodeRate::R1_4 &&
              ConnectionAdaptiveTestAccess::dataFrameCodewords(c) == 3,
          "down-crossing lands on DBPSK at the standard 3-codeword geometry");
    CHECK(events.size() == 2 && cross_carriers == 8 && cross_sps == 1024 && cross_cw == 3,
          "the modem gets the MC-DPSK profile with the crossing");
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
    ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
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
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);

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
        ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
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
        for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
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
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
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
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(holes_rx, 22.0f, 0.05f);
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
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
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
    for (int i = 0; i < 3; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
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
              ConnectionAdaptiveTestAccess::dataModulation(c) == Modulation::DBPSK,
          "MC-DPSK arrived: the receiver follows, DBPSK");
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
    for (int i = 0; i < 4; ++i) ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone,
          "commanded OFDM");
    ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) == kRungIdxNone &&
              ConnectionAdaptiveTestAccess::upCrossingRun(c) == 2,
          "MC-DPSK came again: backed off to a run of 2, no command on this burst");
    ConnectionAdaptiveTestAccess::feedMcDpskSnr(c, 22.0f, 0.05f);
    CHECK(connection_policy::decodeOfdmRungCommand(
              ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true)) != kRungIdxNone,
          "the second clean burst of the longer run commands again");
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
// the returned frame at 5.7 dB (physical 17.6) stayed under the AWGN floor (GUI, ITU Good, seed 46).
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
        const float spell_fading = read_as_awgn ? 0.0f : NAN;
        for (float fade : {-3.0f, -5.0f}) {
            ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, fade, spell_fading, fade + 0.5f);
            (void)ConnectionAdaptiveTestAccess::mcDpskAckWord(c, true);
            ConnectionAdaptiveTestAccess::agePool(c, 8500);
        }
        ConnectionAdaptiveTestAccess::feedMcDpskFrame(c, 5.7f, spell_fading, 17.6f);
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
    CHECK(span > 10 * 60000 && ConnectionAdaptiveTestAccess::txCap(delivering) == 1 && window > 1,
          "its MC-DPSK spell sends one-frame bursts for minutes; the receive window stays whole");
    ConnectionAdaptiveTestAccess::tickFollow(delivering, static_cast<uint32_t>(span - 1));
    CHECK(ConnectionAdaptiveTestAccess::txCap(delivering) == 1, "still single frames");
    ConnectionAdaptiveTestAccess::tickFollow(delivering, 2);
    CHECK(ConnectionAdaptiveTestAccess::txCap(delivering) == 0 &&
              ConnectionAdaptiveTestAccess::arqWindow(delivering) == window,
          "the drop outlived the break-even: the full window");

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

    // The collapse escape itself, under the receiver authority (this binary's default): two
    // zero-progress rounds in silence do not move the rung, and neither do they once an empty ACK
    // has followed them (GUI QSB seed 44: that ACK lent the trough's rounds "peer heard" and the
    // escape demoted the returned link). The receiver's crater verdict commands the rung. With the
    // authority off the sender is the only driver and the escape acts (ConnectionAdaptive).
    CHECK(rxRateAuthorityEnabled(), "this binary runs the receiver authority (production)");
    ConnectionAdaptiveTestAccess::allowRateAdaptation(c);
    bool queued = true;
    for (int f = 0; f < 8; ++f) {  // a collapsed window: half a burst or more pending
        queued = ConnectionAdaptiveTestAccess::sendFixedData(c, Bytes(40, 0x33),
                                                             v2::Flags::MORE_FRAG) && queued;
    }
    CHECK(queued, "a window of frames is in flight (the escape's scope)");
    ConnectionAdaptiveTestAccess::setZeroProgressRounds(c, 2);
    ConnectionAdaptiveTestAccess::pollCollapseEscape(c);
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c) &&
              c.getDataCodeRate() == CodeRate::R2_3 && c.getDataModulation() == Modulation::QAM8,
          "in silence the collapse escape holds the rung");
    ConnectionAdaptiveTestAccess::noteRound(c, 0, "toneburst-ack");
    ConnectionAdaptiveTestAccess::setZeroProgressRounds(c, 2);
    CHECK(ConnectionAdaptiveTestAccess::escapeEvidence(c), "an empty ACK: the peer is heard again");
    ConnectionAdaptiveTestAccess::pollCollapseEscape(c);
    CHECK(!ConnectionAdaptiveTestAccess::modeChangePending(c) &&
              c.getDataCodeRate() == CodeRate::R2_3 && c.getDataModulation() == Modulation::QAM8,
          "silent rounds then an empty ACK: under the authority the collapse escape holds the rung");
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

// A copy of the ACK that launched a latent startup probe is not a monitor candidate while the probe
// group is airborne: consumed after delivery, it had already disarmed the monitor, and the probe's
// own verdict would have gone unheard (BUG-STARTUP-PROBE-COPY-DISARMS-MONITOR; the crossing ACK's
// copy showed what follows).
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

int main() {
    // Production defaults on purpose (see the header); only the follow knob is set per test.
    unsetenv("ULTRA_WAVEFORM_FOLLOW");
    if (!connection_policy::waveformFollowEnabled() || !connection_policy::waveformFollowDualListen()) {
        std::cout << "WaveformFollow: the default must be dual-listen follow\n";
        return 1;
    }
    if (!rxRateAuthorityEnabled()) {
        std::cout << "WaveformFollow: needs ULTRA_RX_RATE_AUTHORITY on (the production default)\n";
        return 1;
    }
    test_waveform_follow_sender_crossings_switch_modem_first();
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
    test_dual_listen_receiver_commands_up_and_follows_the_peer();
    test_dual_listen_receiver_commands_down_and_follows_the_peer();
    test_dual_listen_missed_up_crossing_backs_off();
    test_waveform_follow_crossing_supersedes_mode_change();
    test_waveform_follow_return_reads_the_latest_burst();
    test_waveform_follow_return_reads_a_one_frame_burst();
    test_waveform_follow_return_reads_the_burst_physical_evidence();
    test_waveform_follow_spell_keeps_the_measured_channel_class();
    test_waveform_follow_fallback_spell_starts_with_single_frames();
    test_waveform_follow_crossing_ack_copy_is_not_a_candidate();
    test_mcdpsk_one_frame_resend_ends_its_turn();
    test_first_ofdm_group_climbs_one_rung();
    test_escape_needs_the_peer_heard();
    test_startup_probe_launch_copy_is_not_a_candidate();

    if (tests_failed != 0) {
        std::cout << "WaveformFollow: " << tests_failed << "/" << tests_run << " failed\n";
        return 1;
    }
    std::cout << "WaveformFollow: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
