#pragma once
// Link-control TX plan (docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md, slice 1).
//
// ONE pure function decides, for any control frame a connected station is about to send, which
// waveform it rides and what the session does first. Today these rules live at 13 call sites in
// connection.cpp / connection_handlers.cpp (answerOnArrivalWaveform, the robust DISCONNECT retry, the
// TURN_REQUEST silence retry, the close-time crossing revert, the MC-DPSK close step-down); slice 2
// routes every site through here, behaviour-identical. Each rule cites the bug that set it.
//
// Pure: no I/O, no clock, no Connection state. The caller passes the facts it holds.
#include <cstdint>

#include "connection_policy.hpp"
#include "frame_v2.hpp"
#include "ultra/types.hpp"

namespace ultra {
namespace protocol {

// What the session knows that the plan depends on.
struct LinkFacts {
    WaveformMode session = WaveformMode::MC_DPSK;  // the negotiated (session) waveform
    Modulation session_mod = Modulation::MFSK16_FAST;  // the session's data constellation
    // The peer runs an MC-DPSK decoder beside its OFDM one (dual-listen follow on both ends).
    bool peer_decodes_robust = false;
    // This end crossed MC-DPSK -> OFDM on the receiver's command and has not yet seen the peer
    // follow (sender-side crossing probation): the peer's PRIMARY decoder is still MC-DPSK.
    bool up_unconfirmed_tx = false;
    Modulation up_return_mod = Modulation::MFSK16_FAST;  // the rung that crossing left
};

enum class ControlKind : uint8_t {
    DisconnectStart,   // the first DISCONNECT of a close
    DisconnectRetry,   // an unanswered DISCONNECT, retry_count already incremented
    DisconnectAck,     // answering a DISCONNECT (first copy or proactive repeat)
    Turnover,          // answering a TURN_REQUEST (yield, reassert, orphaned-turn grant)
    TurnRequest,       // first TURN_REQUEST
    TurnRequestRetry,  // an unanswered TURN_REQUEST
};

struct ControlEvent {
    ControlKind kind = ControlKind::TurnRequest;
    bool answering_mcdpsk = false;    // replies: the frame answered was demodulated as MC-DPSK
    Modulation answering_mod = Modulation::AUTO;  // replies: the MC-DPSK rung it decoded at
    int retry_count = 0;              // DisconnectRetry: this retry's ordinal (1-based)
    int useful_retry_count = 0;       // DisconnectRetry: the last retry that can still be answered
    uint32_t peer_silent_ms = 0;      // TurnRequestRetry: time since anything was heard from the peer
    uint32_t retry_interval_ms = 0;   // TurnRequestRetry: the request's retry interval
    // TurnRequest: this session's turn-request record (TurnRequestRecord, decayed counts).
    float ofdm_answered = 0.0f;       // OFDM requests answered by an OFDM TURNOVER
    float ofdm_lost = 0.0f;           // OFDM requests whose retry came due unanswered
    uint32_t ofdm_round_trip_ms = 0;  // last measured OFDM request -> TURNOVER (0 = none yet)
    uint32_t robust_round_trip_ms = 0;  // last measured MC-DPSK request -> TURNOVER (0 = none yet)
};

// BUG-TURN-REQUEST-OFDM-AT-FLOOR: whether a first TURN_REQUEST reaches the turn sooner on MC-DPSK.
// Expected time to the TURNOVER, from this session's own record: on OFDM p x T_ofdm + (1 - p) x
// (T_retry + T_robust) - a lost OFDM request is followed by the robust retry (R3) - against T_robust
// on MC-DPSK (taken as answered: the same assumption R3 makes). p is the posterior mean of the OFDM
// answer rate under a Beta(2, 1) prior: optimistic, so a session starts on OFDM and comes back to it
// as the decayed losses fade (TurnRequestRecord::decay). Needs a measured MC-DPSK round trip: without
// one there is nothing to compare. TNC ping-pong, ITU Good 8 dB: 4 of 42 OFDM first requests answered;
// a lost one cost 15.8 s before the robust request's 17.5 s exchange (mean 35 s per turn change).
inline bool turnRequestRobustIsFaster(const ControlEvent& e) {
    if (e.robust_round_trip_ms == 0 || e.retry_interval_ms == 0) return false;
    constexpr float kPriorAnswered = 2.0f;
    constexpr float kPriorLost = 1.0f;
    const float p = (e.ofdm_answered + kPriorAnswered) /
                    (e.ofdm_answered + e.ofdm_lost + kPriorAnswered + kPriorLost);
    const float t_robust = static_cast<float>(e.robust_round_trip_ms);
    const float t_ofdm_first =
        p * static_cast<float>(e.ofdm_round_trip_ms) +
        (1.0f - p) * (static_cast<float>(e.retry_interval_ms) + t_robust);
    return t_robust < t_ofdm_first;
}

struct ControlTxPlan {
    // The next frame rides MC-DPSK once, at robust_mod (ModemEngine::transmitNextOnRobustWaveform).
    bool robust_once = false;
    Modulation robust_mod = Modulation::AUTO;
    // Before the close: cross the session back to MC-DPSK at up_return_mod.
    bool revert_up_crossing = false;
    // An MC-DPSK close retry steps the session's constellation down to this rung (AUTO = no change).
    Modulation step_down_to = Modulation::AUTO;
    const char* rule = "";  // which rule fired, for the log
};

// The constellation the peer's MC-DPSK decoder demodulates at: its primary's rung during this
// end's unconfirmed up-crossing, else the dual-listen listener's ROBUST_MID.
inline Modulation peerMcDpskDecoderModulation(const LinkFacts& f) {
    if (f.up_unconfirmed_tx && connection_policy::mcDpskRungIndexFor(f.up_return_mod) >= 0) {
        return f.up_return_mod;
    }
    return connection_policy::ladderRungForId(LadderRungId::ROBUST_MID).modulation;
}

inline ControlTxPlan planControlTx(const LinkFacts& f, const ControlEvent& e) {
    ControlTxPlan p;
    const bool ofdm = f.session == WaveformMode::OFDM_CHIRP;
    const auto robust = [&](const char* rule) {
        p.robust_once = true;
        p.robust_mod = peerMcDpskDecoderModulation(f);
        p.rule = rule;
    };
    switch (e.kind) {
        case ControlKind::DisconnectStart:
            // R2 BUG-CLOSE-DURING-UNCONFIRMED-UPCROSS: the peer's primary is still MC-DPSK.
            if (ofdm && f.up_unconfirmed_tx &&
                connection_policy::mcDpskRungIndexFor(f.up_return_mod) >= 0) {
                p.revert_up_crossing = true;
                p.rule = "R2 close during an unconfirmed up-crossing";
            }
            break;
        case ControlKind::DisconnectRetry:
            if (ofdm) {
                // R4 BUG-DISCONNECT-LOST-IN-FADE: the last useful retry rides MC-DPSK.
                if (f.peer_decodes_robust && e.retry_count >= e.useful_retry_count) {
                    robust("R4 last useful DISCONNECT retry");
                }
            } else if (f.session == WaveformMode::MC_DPSK) {
                // R5 BUG-MCDPSK-CLOSE-STUCK-ON-UNPROVEN-RUNG: an unanswered MC-DPSK close steps down.
                const Modulation below = connection_policy::mcDpskRungBelow(f.session_mod);
                if (below != f.session_mod && connection_policy::mcDpskRungIndexFor(below) >= 0) {
                    p.step_down_to = below;
                    p.rule = "R5 MC-DPSK close retry steps down";
                }
            }
            break;
        case ControlKind::DisconnectAck:
        case ControlKind::Turnover:
            // R1 BUG-REPLY-ON-WRONG-WAVEFORM: a reply rides the waveform its request arrived on.
            if (ofdm && f.peer_decodes_robust && e.answering_mcdpsk) {
                robust("R1 reply on the request's waveform");
            } else if (f.session == WaveformMode::MC_DPSK && e.answering_mcdpsk) {
                // R6 BUG-MCDPSK-CONTROL-REPLY-RUNG: in an MC-DPSK session a reply rides the more
                // robust of this station's own rung and the rung its request decoded at - an
                // LDPC control codeword sits ~1 dB above the turbo data frame of the same rung, so
                // a reply at a data rung the link carries only for turbo frames is lost.
                const int req = connection_policy::mcDpskRungIndexFor(e.answering_mod);
                const int own = connection_policy::mcDpskRungIndexFor(f.session_mod);
                if (req >= 0 && own >= 0 && req < own) {
                    p.robust_once = true;
                    p.robust_mod = e.answering_mod;
                    p.rule = "R6 MC-DPSK reply at the request's more robust rung";
                }
            }
            break;
        case ControlKind::TurnRequest:
            if (ofdm && f.peer_decodes_robust && turnRequestRobustIsFaster(e)) {
                robust("R7 TURN_REQUEST on the waveform that answers sooner");
            }
            break;
        case ControlKind::TurnRequestRetry:
            // R3 BUG-TURN-CONTROL-LOST-AT-OFDM-FLOOR: a retry into a peer silent for a whole
            // retry interval (never while a healthy peer holding the turn is heard).
            if (ofdm && f.peer_decodes_robust && e.peer_silent_ms >= e.retry_interval_ms) {
                robust("R3 TURN_REQUEST retry into silence");
            } else if (f.session == WaveformMode::MC_DPSK && e.peer_silent_ms >= e.retry_interval_ms) {
                // R8 BUG-MCDPSK-TURN-REQUEST-STUCK-ON-RUNG: an unanswered MC-DPSK request steps the
                // session one rung down, as R5 does for the close. The single LDPC control codeword
                // sits ~1 dB above the turbo data frame of its rung, so a rung that carried the last
                // data can lose every request: TNC AWGN 0 dB, 6 of 6 runs re-sent a DQPSK request
                // every 18 s for 240 s, decoded at -7 dB. The peer's tone ACKs step the rung back up.
                const Modulation below = connection_policy::mcDpskRungBelow(f.session_mod);
                if (below != f.session_mod && connection_policy::mcDpskRungIndexFor(below) >= 0) {
                    p.step_down_to = below;
                    p.rule = "R8 MC-DPSK TURN_REQUEST retry steps down";
                }
            }
            break;
    }
    return p;
}

}  // namespace protocol
}  // namespace ultra
