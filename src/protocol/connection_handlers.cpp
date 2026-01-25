// Connection frame handlers
// Split from connection.cpp for maintainability

#include "connection.hpp"
#include "ultra/logging.hpp"

namespace ultra {
namespace protocol {

// Recommend data mode based on measured SNR
// Conservative thresholds calibrated for REAL HF channels (not just AWGN)
// HF has multipath fading that requires 3-6 dB extra margin vs AWGN
static void recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate) {
    // Typical HF SNR: 10-18 dB (50% of time), 18-25 dB (good, ~20%)
    // These thresholds include ~4 dB margin for fading

    if (snr_db >= 30.0f) {
        // Excellent conditions (rare) - use high throughput
        mod = Modulation::QAM16;
        rate = CodeRate::R3_4;  // 3x baseline
    } else if (snr_db >= 25.0f) {
        // Very good conditions
        mod = Modulation::QAM16;
        rate = CodeRate::R2_3;  // 2.7x baseline
    } else if (snr_db >= 20.0f) {
        // Good conditions - sweet spot for speed
        mod = Modulation::DQPSK;
        rate = CodeRate::R2_3;  // 2.7x baseline, differential for robustness
    } else if (snr_db >= 16.0f) {
        // Typical good HF - balanced speed/reliability
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_2;  // 2x baseline
    } else if (snr_db >= 12.0f) {
        // Typical HF - prioritize reliability
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_4;  // Baseline
    } else if (snr_db >= 8.0f) {
        // Marginal conditions
        mod = Modulation::DBPSK;
        rate = CodeRate::R1_4;  // Most robust
    } else {
        // Very poor SNR - maximum robustness
        mod = Modulation::DBPSK;
        rate = CodeRate::R1_4;
    }
}

// =============================================================================
// PING/PONG HANDLING
// =============================================================================

void Connection::onPongReceived() {
    if (state_ != ConnectionState::PROBING) {
        // Not in probing state - this might be an incoming ping from someone else
        // Notify via ping_received callback (they're calling us)
        if (state_ == ConnectionState::DISCONNECTED && on_ping_received_) {
            LOG_MODEM(INFO, "Connection: Received incoming PING while disconnected");
            on_ping_received_();
        }
        return;
    }

    // We were probing and got a PONG - remote station exists!
    LOG_MODEM(INFO, "Connection: PONG received, sending full CONNECT");
    sendFullConnect();
}

void Connection::sendFullConnect() {
    // Transition to CONNECTING and send full CONNECT frame
    state_ = ConnectionState::CONNECTING;
    connect_retry_count_ = 0;
    timeout_remaining_ms_ = config_.connect_timeout_ms;

    // Notify about state change (PROBING -> CONNECTING)
    if (on_state_changed_) {
        on_state_changed_(ConnectionState::CONNECTING, remote_call_);
    }

    auto connect_frame = v2::ConnectFrame::makeConnect(local_call_, remote_call_,
                                                        config_.mode_capabilities,
                                                        static_cast<uint8_t>(config_.preferred_mode),
                                                        static_cast<uint8_t>(config_.forced_modulation),
                                                        static_cast<uint8_t>(config_.forced_code_rate));
    Bytes connect_data = connect_frame.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT via %s (%zu bytes, forced_mod=%d, forced_rate=%d)",
              waveformModeToString(connect_waveform_), connect_data.size(),
              static_cast<int>(config_.forced_modulation), static_cast<int>(config_.forced_code_rate));
    transmitFrame(connect_data);
}

// =============================================================================
// CONNECT FRAME HANDLERS
// =============================================================================

void Connection::handleConnect(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Rejecting CONNECT (busy, state=%s)",
                  connectionStateToString(state_));
        auto nak = v2::ConnectFrame::makeConnectNakByHash(local_call_, frame.src_hash);
        transmitFrame(nak.serialize());
        return;
    }

    // Get capabilities from ConnectFrame
    uint8_t remote_caps = frame.mode_capabilities;
    WaveformMode remote_pref = static_cast<WaveformMode>(frame.negotiated_mode);

    LOG_MODEM(INFO, "Connection: Incoming call from %s (caps=0x%02X, pref=%s)",
              src_call.c_str(), remote_caps, waveformModeToString(remote_pref));
    stats_.connects_received++;

    remote_capabilities_ = remote_caps;
    remote_preferred_ = remote_pref;

    // Default channel estimates for mode selection
    float snr_db = 15.0f;
    float delay_spread_ms = 1.0f;
    float doppler_spread_hz = 0.5f;

    if (config_.auto_accept) {
        // Store callsign if known, otherwise use placeholder with hash for display
        remote_call_ = src_call.empty() ? "REMOTE" : src_call;
        remote_hash_ = frame.src_hash;  // Store hash for routing

        // Use measured SNR from modem (set via setMeasuredSNR)
        snr_db = measured_snr_db_;

        // Let negotiateMode() handle AUTO mode with SNR-based selection
        // Don't override remote_pref here - negotiateMode() has the correct SNR thresholds
        negotiated_mode_ = negotiateMode(remote_caps, remote_pref);

        LOG_MODEM(INFO, "Connection: Negotiated waveform mode: %s",
                  waveformModeToString(negotiated_mode_));

        // We are the responder - we received CONNECT and are sending CONNECT_ACK
        is_initiator_ = false;
        handshake_confirmed_ = false;  // Responder waits for first frame to confirm

        // Check if initiator forced specific modes (0xFF = AUTO, else forced)
        Modulation forced_mod = static_cast<Modulation>(frame.initial_modulation);
        CodeRate forced_rate = static_cast<CodeRate>(frame.initial_code_rate);

        Modulation rec_mod;
        CodeRate rec_rate;

        if (forced_mod != Modulation::AUTO) {
            // Initiator forced a specific modulation - honor it
            rec_mod = forced_mod;
            LOG_MODEM(INFO, "Connection: Using FORCED modulation %s from initiator",
                      modulationToString(rec_mod));
        } else {
            // AUTO - recommend based on measured SNR
            CodeRate dummy_rate;
            recommendDataMode(snr_db, rec_mod, dummy_rate);
        }

        if (forced_rate != CodeRate::AUTO) {
            // Initiator forced a specific code rate - honor it
            rec_rate = forced_rate;
            LOG_MODEM(INFO, "Connection: Using FORCED code rate %s from initiator",
                      codeRateToString(rec_rate));
        } else {
            // AUTO - recommend based on measured SNR
            Modulation dummy_mod;
            recommendDataMode(snr_db, dummy_mod, rec_rate);
        }

        LOG_MODEM(INFO, "Connection: Initial data mode %s %s (SNR=%.1f dB, forced_mod=%d, forced_rate=%d)",
                  modulationToString(rec_mod), codeRateToString(rec_rate), snr_db,
                  static_cast<int>(forced_mod), static_cast<int>(forced_rate));

        // Set our local data mode immediately
        data_modulation_ = rec_mod;
        data_code_rate_ = rec_rate;

        // Use hash-based method since we may not know the actual callsign
        // CONNECT_ACK now carries the initial data mode - no separate MODE_CHANGE needed!
        auto ack = v2::ConnectFrame::makeConnectAckByHash(local_call_, frame.src_hash,
                                                           static_cast<uint8_t>(negotiated_mode_),
                                                           rec_mod, rec_rate, snr_db);
        transmitFrame(ack.serialize());
        enterConnected();
        // NOTE: Don't call on_handshake_confirmed_ yet - wait for first frame from initiator

        // Notify application of initial data mode
        if (on_data_mode_changed_) {
            on_data_mode_changed_(data_modulation_, data_code_rate_, snr_db);
        }
    } else {
        pending_remote_call_ = src_call.empty() ? "REMOTE" : src_call;
        pending_remote_hash_ = frame.src_hash;  // Store hash for later
        // Store forced modes from initiator for later use in acceptCall()
        pending_forced_modulation_ = static_cast<Modulation>(frame.initial_modulation);
        pending_forced_code_rate_ = static_cast<CodeRate>(frame.initial_code_rate);
        if (on_incoming_call_) {
            on_incoming_call_(pending_remote_call_);
        }
    }
}

void Connection::handleConnectAck(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::CONNECTING) {
        LOG_MODEM(DEBUG, "Connection: Ignoring CONNECT_ACK (state=%s)",
                  connectionStateToString(state_));
        return;
    }

    // Get negotiated waveform mode from ConnectFrame
    WaveformMode mode = static_cast<WaveformMode>(frame.negotiated_mode);
    negotiated_mode_ = mode;

    // Get initial data mode from CONNECT_ACK (eliminates separate MODE_CHANGE)
    Modulation init_mod = static_cast<Modulation>(frame.initial_modulation);
    CodeRate init_rate = static_cast<CodeRate>(frame.initial_code_rate);
    float snr_db = v2::decodeSNR(frame.measured_snr);

    // Apply the initial data mode immediately
    data_modulation_ = init_mod;
    data_code_rate_ = init_rate;

    // Update remote callsign if we got it from the frame
    if (!src_call.empty() && (remote_call_.empty() || remote_call_ == "REMOTE")) {
        remote_call_ = src_call;
    }

    LOG_MODEM(INFO, "Connection: Connected to %s (waveform=%s, data=%s %s, SNR=%.1f dB)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_),
              modulationToString(data_modulation_), codeRateToString(data_code_rate_), snr_db);

    // We are the initiator - we sent CONNECT and received CONNECT_ACK
    is_initiator_ = true;
    handshake_confirmed_ = true;  // Handshake complete for initiator

    enterConnected();

    // Initiator can switch to negotiated waveform immediately
    if (on_handshake_confirmed_) {
        on_handshake_confirmed_();
    }

    // Notify application of initial data mode
    if (on_data_mode_changed_) {
        on_data_mode_changed_(data_modulation_, data_code_rate_, snr_db);
    }
}

void Connection::handleConnectNak(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::CONNECTING) {
        return;
    }

    LOG_MODEM(WARN, "Connection: Connection rejected by %s", remote_call_.c_str());
    stats_.connects_failed++;
    enterDisconnected("Connection rejected");
}

// =============================================================================
// DISCONNECT HANDLERS
// =============================================================================

void Connection::handleDisconnect(const v2::ControlFrame& frame, const std::string& src_call) {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Disconnect from %s", remote_call_.c_str());

    auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, 0);
    transmitFrame(ack.serialize());

    stats_.disconnects++;
    enterDisconnected("Remote disconnected");
}

void Connection::handleDisconnectFrame(const v2::ConnectFrame& frame, const std::string& src_call) {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Disconnect from %s", src_call.c_str());

    // Send ACK for the disconnect
    auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
    transmitFrame(ack.serialize());

    stats_.disconnects++;
    enterDisconnected("Remote disconnected");
}

// =============================================================================
// MODE CHANGE HANDLING
// =============================================================================

void Connection::handleModeChange(const v2::ControlFrame& frame, const std::string& src_call) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(DEBUG, "Connection: Ignoring MODE_CHANGE (not connected)");
        return;
    }

    // Parse MODE_CHANGE payload
    auto info = frame.getModeChangeInfo();

    const char* reason_str = "unknown";
    switch (info.reason) {
        case v2::ModeChangeReason::CHANNEL_IMPROVED: reason_str = "channel improved"; break;
        case v2::ModeChangeReason::CHANNEL_DEGRADED: reason_str = "channel degraded"; break;
        case v2::ModeChangeReason::USER_REQUEST:     reason_str = "user request"; break;
        case v2::ModeChangeReason::INITIAL_SETUP:    reason_str = "initial setup"; break;
    }

    LOG_MODEM(INFO, "Connection: MODE_CHANGE from %s: %s %s (SNR=%.1f dB, reason=%s)",
              remote_call_.c_str(),
              modulationToString(info.modulation),
              codeRateToString(info.code_rate),
              info.snr_db,
              reason_str);

    // Update local state
    data_modulation_ = info.modulation;
    data_code_rate_ = info.code_rate;

    // Send ACK for the MODE_CHANGE
    auto ack = v2::ControlFrame::makeAck(local_call_, remote_call_, frame.seq);
    transmitFrame(ack.serialize());

    // Notify application of mode change
    if (on_data_mode_changed_) {
        on_data_mode_changed_(info.modulation, info.code_rate, info.snr_db);
    }
}

void Connection::requestModeChange(Modulation new_mod, CodeRate new_rate,
                                    float measured_snr, uint8_t reason) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot request mode change (not connected)");
        return;
    }

    // Don't send new MODE_CHANGE if one is already pending
    if (mode_change_pending_) {
        LOG_MODEM(DEBUG, "Connection: MODE_CHANGE already pending, ignoring request");
        return;
    }

    LOG_MODEM(INFO, "Connection: Requesting MODE_CHANGE to %s %s (SNR=%.1f dB)",
              modulationToString(new_mod), codeRateToString(new_rate), measured_snr);

    // Store pending mode change parameters for retry
    pending_modulation_ = new_mod;
    pending_code_rate_ = new_rate;
    pending_snr_db_ = measured_snr;
    pending_reason_ = reason;
    mode_change_pending_ = true;
    mode_change_retry_count_ = 0;
    mode_change_timeout_ms_ = MODE_CHANGE_TIMEOUT_MS;

    mode_change_seq_++;
    auto frame = v2::ControlFrame::makeModeChange(local_call_, remote_call_,
                                                   mode_change_seq_, new_mod, new_rate,
                                                   measured_snr, reason);
    transmitFrame(frame.serialize());

    // NOTE: Don't update local mode until ACK is received
    // This prevents mode mismatch if the remote doesn't receive our MODE_CHANGE
}

// =============================================================================
// DATA PAYLOAD HANDLING
// =============================================================================

void Connection::handleDataPayload(const Bytes& payload, bool more_data) {
    if (payload.empty()) {
        return;
    }

    if (file_transfer_.processPayload(payload, more_data)) {
        LOG_MODEM(DEBUG, "Connection: Processed file transfer payload (%zu bytes)",
                  payload.size());

        if (on_data_received_) {
            on_data_received_(payload, more_data);
        }
        return;
    }

    size_t start = 0;
    if (!payload.empty() && payload[0] == static_cast<uint8_t>(PayloadType::TEXT_MESSAGE)) {
        start = 1;
    }

    std::string text(payload.begin() + start, payload.end());

    if (on_message_received_) {
        on_message_received_(text);
    }

    if (on_data_received_) {
        on_data_received_(payload, more_data);
    }
}

// =============================================================================
// WAVEFORM MODE NEGOTIATION
// =============================================================================

WaveformMode Connection::negotiateMode(uint8_t remote_caps, WaveformMode remote_pref) {
    uint8_t common = config_.mode_capabilities & remote_caps;

    if (common == 0) {
        LOG_MODEM(WARN, "Connection: No common waveform modes! Falling back to OFDM");
        return WaveformMode::OFDM_NVIS;
    }

    // Helper to convert WaveformMode to capability bit
    auto modeToBit = [](WaveformMode mode) -> uint8_t {
        switch (mode) {
            case WaveformMode::OFDM_NVIS:     return ModeCapabilities::OFDM_NVIS;
            case WaveformMode::OTFS_EQ:  return ModeCapabilities::OTFS_EQ;
            case WaveformMode::OTFS_RAW: return ModeCapabilities::OTFS_RAW;
            case WaveformMode::MFSK:     return ModeCapabilities::MFSK;
            case WaveformMode::MC_DPSK:     return ModeCapabilities::MC_DPSK;
            default: return 0;
        }
    };

    // If remote has explicit preference, honor it if we support it
    if (remote_pref != WaveformMode::AUTO) {
        uint8_t pref_bit = modeToBit(remote_pref);
        if (common & pref_bit) {
            LOG_MODEM(INFO, "Connection: Using remote preferred mode: %s",
                      waveformModeToString(remote_pref));
            return remote_pref;
        }
    }

    // If we have explicit preference, use it if remote supports it
    if (config_.preferred_mode != WaveformMode::AUTO) {
        uint8_t pref_bit = modeToBit(config_.preferred_mode);
        if (common & pref_bit) {
            LOG_MODEM(INFO, "Connection: Using local preferred mode: %s",
                      waveformModeToString(config_.preferred_mode));
            return config_.preferred_mode;
        }
    }

    // AUTO mode: Select based on measured SNR
    // Thresholds based on testing (see CLAUDE.md):
    //   < 0 dB:  MFSK (works at -17 dB reported)
    //   0-17 dB: DPSK (single-carrier, robust - OFDM sync fails below ~17 dB)
    //   > 17 dB: OFDM (highest throughput)
    float snr = measured_snr_db_;
    LOG_MODEM(INFO, "Connection: AUTO mode selection, SNR=%.1f dB", snr);

    if (snr < 0.0f && (common & ModeCapabilities::MFSK)) {
        LOG_MODEM(INFO, "Connection: Selected MFSK for very low SNR (%.1f dB)", snr);
        return WaveformMode::MFSK;
    }

    if (snr < 17.0f && (common & ModeCapabilities::MC_DPSK)) {
        LOG_MODEM(INFO, "Connection: Selected DPSK for low/mid SNR (%.1f dB)", snr);
        return WaveformMode::MC_DPSK;
    }

    // Default priority for adequate SNR: OFDM > OTFS > DPSK > MFSK
    if (common & ModeCapabilities::OFDM_NVIS) {
        LOG_MODEM(INFO, "Connection: Selected OFDM (SNR=%.1f dB)", snr);
        return WaveformMode::OFDM_NVIS;
    }
    if (common & ModeCapabilities::OTFS_EQ) return WaveformMode::OTFS_EQ;
    if (common & ModeCapabilities::OTFS_RAW) return WaveformMode::OTFS_RAW;
    if (common & ModeCapabilities::MC_DPSK) return WaveformMode::MC_DPSK;
    if (common & ModeCapabilities::MFSK) return WaveformMode::MFSK;

    return WaveformMode::OFDM_NVIS;
}

} // namespace protocol
} // namespace ultra
