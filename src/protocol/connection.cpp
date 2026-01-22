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

const char* connectionStateToString(ConnectionState state) {
    switch (state) {
        case ConnectionState::DISCONNECTED:  return "DISCONNECTED";
        case ConnectionState::CONNECTING:    return "CONNECTING";
        case ConnectionState::CONNECTED:     return "CONNECTED";
        case ConnectionState::DISCONNECTING: return "DISCONNECTING";
        default: return "UNKNOWN";
    }
}

Connection::Connection(const ConnectionConfig& config)
    : config_(config)
    , arq_(config.arq)
{
    // Wire up ARQ callbacks
    arq_.setTransmitCallback([this](const Bytes& data) {
        transmitFrame(data);
    });

    arq_.setDataReceivedCallback([this](const Bytes& data) {
        handleDataPayload(data, arq_.lastRxHadMoreData());
    });

    arq_.setSendCompleteCallback([this](bool success) {
        if (file_transfer_.getState() == FileTransferState::SENDING) {
            if (success) {
                file_transfer_.onChunkAcked();
                sendNextFileChunk();
            } else {
                file_transfer_.onSendFailed();
            }
        } else {
            if (on_message_sent_) {
                on_message_sent_(success);
            }
        }
    });
}

void Connection::setLocalCallsign(const std::string& call) {
    local_call_ = sanitizeCallsign(call);
    LOG_MODEM(INFO, "Connection: Local callsign set to %s", local_call_.c_str());
}

bool Connection::connect(const std::string& remote_call) {
    if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot connect, state=%s",
                  connectionStateToString(state_));
        return false;
    }

    if (local_call_.empty()) {
        LOG_MODEM(ERROR, "Connection: Local callsign not set");
        return false;
    }

    remote_call_ = sanitizeCallsign(remote_call);
    if (remote_call_.empty() || !isValidCallsign(remote_call_)) {
        LOG_MODEM(ERROR, "Connection: Invalid remote callsign: %s", remote_call.c_str());
        return false;
    }

    LOG_MODEM(INFO, "Connection: Connecting to %s", remote_call_.c_str());

    // Use current connect_waveform_ (can be pre-set via setInitialConnectWaveform)
    // Notify the modem of the waveform to use
    if (on_connect_waveform_changed_) {
        on_connect_waveform_changed_(connect_waveform_);
    }

    // Send CONNECT directly (no PROBE phase)
    auto connect_frame = v2::ConnectFrame::makeConnect(local_call_, remote_call_,
                                                        config_.mode_capabilities,
                                                        static_cast<uint8_t>(config_.preferred_mode));
    Bytes connect_data = connect_frame.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT via %s (%zu bytes)",
              waveformModeToString(connect_waveform_), connect_data.size());
    transmitFrame(connect_data);

    state_ = ConnectionState::CONNECTING;
    connect_retry_count_ = 0;
    timeout_remaining_ms_ = config_.connect_timeout_ms;
    stats_.connects_initiated++;

    return true;
}

void Connection::acceptCall() {
    if (state_ != ConnectionState::DISCONNECTED || pending_remote_call_.empty()) {
        LOG_MODEM(WARN, "Connection: No pending call to accept");
        return;
    }

    remote_call_ = pending_remote_call_;
    pending_remote_call_.clear();

    negotiated_mode_ = negotiateMode(remote_capabilities_, remote_preferred_);

    // Recommend data mode based on measured SNR
    Modulation rec_mod;
    CodeRate rec_rate;
    recommendDataMode(measured_snr_db_, rec_mod, rec_rate);

    // Set our local data mode immediately
    data_modulation_ = rec_mod;
    data_code_rate_ = rec_rate;

    LOG_MODEM(INFO, "Connection: Accepting call from %s (waveform=%s, data=%s %s)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_),
              modulationToString(data_modulation_), codeRateToString(data_code_rate_));

    auto ack = v2::ConnectFrame::makeConnectAck(local_call_, remote_call_,
                                                 static_cast<uint8_t>(negotiated_mode_),
                                                 rec_mod, rec_rate, measured_snr_db_);
    Bytes ack_data = ack.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT_ACK (%zu bytes)", ack_data.size());
    transmitFrame(ack_data);

    enterConnected();

    // We are the responder - we received CONNECT and are sending CONNECT_ACK
    is_initiator_ = false;
    handshake_confirmed_ = false;  // Responder waits for first frame to confirm

    // Notify application of initial data mode
    if (on_data_mode_changed_) {
        on_data_mode_changed_(data_modulation_, data_code_rate_, measured_snr_db_);
    }
}

void Connection::rejectCall() {
    if (pending_remote_call_.empty()) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Rejecting call from %s", pending_remote_call_.c_str());

    auto nak = v2::ConnectFrame::makeConnectNak(local_call_, pending_remote_call_);
    Bytes nak_data = nak.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT_NAK (%zu bytes)", nak_data.size());
    transmitFrame(nak_data);

    pending_remote_call_.clear();
}

void Connection::disconnect() {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    if (state_ == ConnectionState::CONNECTING) {
        enterDisconnected("Cancelled");
        return;
    }

    if (state_ == ConnectionState::CONNECTED) {
        LOG_MODEM(INFO, "Connection: Disconnecting from %s", remote_call_.c_str());

        auto disc = v2::ConnectFrame::makeDisconnect(local_call_, remote_call_);
        Bytes disc_data = disc.serialize();

        LOG_MODEM(INFO, "Connection: Sending DISCONNECT (%zu bytes)", disc_data.size());
        transmitFrame(disc_data);

        state_ = ConnectionState::DISCONNECTING;
        timeout_remaining_ms_ = config_.disconnect_timeout_ms;
        stats_.disconnects++;
    }
}

bool Connection::sendMessage(const std::string& text) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send, not connected");
        return false;
    }

    return arq_.sendData(text);
}

bool Connection::isReadyToSend() const {
    return state_ == ConnectionState::CONNECTED && arq_.isReadyToSend() &&
           !file_transfer_.isBusy();
}

// --- File Transfer ---

bool Connection::sendFile(const std::string& filepath) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send file, not connected");
        return false;
    }

    if (file_transfer_.isBusy()) {
        LOG_MODEM(WARN, "Connection: File transfer already in progress");
        return false;
    }

    if (!arq_.isReadyToSend()) {
        LOG_MODEM(WARN, "Connection: ARQ busy, cannot start file transfer");
        return false;
    }

    LOG_MODEM(INFO, "Connection: Starting file transfer: %s", filepath.c_str());

    if (!file_transfer_.startSend(filepath)) {
        LOG_MODEM(ERROR, "Connection: Failed to start file transfer");
        return false;
    }

    sendNextFileChunk();
    return true;
}

void Connection::setReceiveDirectory(const std::string& dir) {
    file_transfer_.setReceiveDirectory(dir);
}

void Connection::cancelFileTransfer() {
    file_transfer_.cancel();
}

bool Connection::isFileTransferInProgress() const {
    return file_transfer_.isBusy();
}

FileTransferProgress Connection::getFileProgress() const {
    return file_transfer_.getProgress();
}

void Connection::sendNextFileChunk() {
    if (file_transfer_.getState() != FileTransferState::SENDING) {
        return;
    }

    if (!arq_.isReadyToSend()) {
        return;
    }

    Bytes chunk = file_transfer_.getNextChunk();
    if (chunk.empty()) {
        return;
    }

    uint8_t flags = file_transfer_.hasMoreChunks() ? v2::Flags::MORE_FRAG : v2::Flags::NONE;
    arq_.sendDataWithFlags(chunk, flags);
}

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

void Connection::onFrameReceived(const Bytes& frame_data) {
    if (frame_data.size() < 2) {
        return;
    }

    // Responder handshake confirmation: when we receive first frame after CONNECT_ACK
    // This means the initiator got our CONNECT_ACK and the handshake is complete
    if (state_ == ConnectionState::CONNECTED && !is_initiator_ && !handshake_confirmed_) {
        LOG_MODEM(INFO, "Connection: Handshake confirmed (received first frame from initiator)");
        handshake_confirmed_ = true;
        if (on_handshake_confirmed_) {
            on_handshake_confirmed_();
        }
        // NOTE: Initial data mode is now carried in CONNECT_ACK, no separate MODE_CHANGE needed
    }

    // Check v2 magic
    uint16_t magic = (static_cast<uint16_t>(frame_data[0]) << 8) | frame_data[1];
    if (magic != v2::MAGIC_V2) {
        LOG_MODEM(TRACE, "Connection: Ignoring frame with wrong magic");
        return;
    }

    auto header = v2::parseHeader(frame_data);
    if (!header.valid) {
        LOG_MODEM(TRACE, "Connection: Ignoring frame with invalid header");
        return;
    }

    // Check if frame is for us
    uint32_t our_hash = v2::hashCallsign(local_call_);
    if (header.dst_hash != our_hash && header.dst_hash != 0xFFFFFF) {
        LOG_MODEM(TRACE, "Connection: Ignoring frame for different station");
        return;
    }

    // Resolve source callsign from hash if possible
    std::string src_call;
    if (!remote_call_.empty() && v2::hashCallsign(remote_call_) == header.src_hash) {
        src_call = remote_call_;
    } else if (!pending_remote_call_.empty() && v2::hashCallsign(pending_remote_call_) == header.src_hash) {
        src_call = pending_remote_call_;
    }

    LOG_MODEM(DEBUG, "Connection: Received %s seq=%d from hash 0x%06X",
              v2::frameTypeToString(header.type), header.seq, header.src_hash);

    // Check frame type category and dispatch accordingly
    if (v2::isConnectFrame(header.type)) {
        // CONNECT/CONNECT_ACK/CONNECT_NAK - parse as ConnectFrame (carries full callsigns)
        auto conn = v2::ConnectFrame::deserialize(frame_data);
        if (conn) {
            // Extract callsign from frame if available
            std::string frame_src_call = conn->getSrcCallsign();
            if (!frame_src_call.empty()) {
                src_call = frame_src_call;  // Use verified callsign from frame
            }

            switch (conn->type) {
                case v2::FrameType::CONNECT:
                    handleConnect(*conn, src_call);
                    break;
                case v2::FrameType::CONNECT_ACK:
                    handleConnectAck(*conn, src_call);
                    break;
                case v2::FrameType::CONNECT_NAK:
                    handleConnectNak(*conn, src_call);
                    break;
                case v2::FrameType::DISCONNECT:
                    handleDisconnectFrame(*conn, src_call);
                    break;
                default:
                    break;
            }
        }
    } else if (v2::isControlFrame(header.type)) {
        // Control frames (DISCONNECT, ACK, NACK, etc) - parse as ControlFrame
        auto ctrl = v2::ControlFrame::deserialize(frame_data);
        if (ctrl) {
            switch (ctrl->type) {
                case v2::FrameType::DISCONNECT:
                    handleDisconnect(*ctrl, src_call);
                    break;
                case v2::FrameType::ACK:
                    if (state_ == ConnectionState::DISCONNECTING) {
                        LOG_MODEM(INFO, "Connection: Disconnect acknowledged");
                        enterDisconnected("Disconnect complete");
                    } else if (state_ == ConnectionState::CONNECTED) {
                        // Check if this ACK is for our pending MODE_CHANGE
                        if (mode_change_pending_ && ctrl->seq == mode_change_seq_) {
                            LOG_MODEM(INFO, "Connection: MODE_CHANGE acknowledged, applying %s %s",
                                      modulationToString(pending_modulation_),
                                      codeRateToString(pending_code_rate_));
                            data_modulation_ = pending_modulation_;
                            data_code_rate_ = pending_code_rate_;
                            mode_change_pending_ = false;

                            // Notify application of mode change
                            if (on_data_mode_changed_) {
                                on_data_mode_changed_(data_modulation_, data_code_rate_, pending_snr_db_);
                            }
                        } else {
                            // Regular data ACK
                            arq_.onFrameReceived(frame_data);
                        }
                    }
                    break;
                case v2::FrameType::NACK:
                    if (state_ == ConnectionState::CONNECTED) {
                        arq_.onFrameReceived(frame_data);
                    }
                    break;
                case v2::FrameType::MODE_CHANGE:
                    handleModeChange(*ctrl, src_call);
                    break;
                case v2::FrameType::PROBE:
                case v2::FrameType::PROBE_ACK:
                    // PROBE not used - ignore (or could respond with CONNECT_NAK)
                    LOG_MODEM(DEBUG, "Connection: Ignoring PROBE (not supported)");
                    break;
                default:
                    break;
            }
        }
    } else {
        // Data frame - pass to ARQ
        if (state_ == ConnectionState::CONNECTED) {
            arq_.onFrameReceived(frame_data);
        }
    }
}

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

        WaveformMode channel_recommended = WaveformMode::OFDM;
        if (doppler_spread_hz > 5.0f) {
            channel_recommended = WaveformMode::OTFS_RAW;
        } else if (snr_db > 20.0f && delay_spread_ms < 1.0f) {
            channel_recommended = WaveformMode::OTFS_EQ;
        }

        if (remote_pref == WaveformMode::AUTO) {
            remote_pref = channel_recommended;
        }

        negotiated_mode_ = negotiateMode(remote_caps, remote_pref);

        LOG_MODEM(INFO, "Connection: Negotiated waveform mode: %s",
                  waveformModeToString(negotiated_mode_));

        // We are the responder - we received CONNECT and are sending CONNECT_ACK
        is_initiator_ = false;
        handshake_confirmed_ = false;  // Responder waits for first frame to confirm

        // Recommend data mode based on measured SNR
        Modulation rec_mod;
        CodeRate rec_rate;
        recommendDataMode(snr_db, rec_mod, rec_rate);

        LOG_MODEM(INFO, "Connection: Initial data mode %s %s based on SNR=%.1f dB",
                  modulationToString(rec_mod), codeRateToString(rec_rate), snr_db);

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

void Connection::tick(uint32_t elapsed_ms) {
    switch (state_) {
        case ConnectionState::CONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                connect_retry_count_++;
                if (connect_retry_count_ >= config_.connect_retries) {
                    LOG_MODEM(ERROR, "Connection: Connect failed after %d attempts",
                              config_.connect_retries);
                    stats_.connects_failed++;
                    char reason[64];
                    snprintf(reason, sizeof(reason), "Connection timeout after %d attempts", config_.connect_retries);
                    enterDisconnected(reason);
                } else {
                    // Check if we need to fall back to MFSK after DPSK_ATTEMPTS
                    if (connect_retry_count_ == DPSK_ATTEMPTS &&
                        connect_waveform_ == WaveformMode::DPSK) {
                        connect_waveform_ = WaveformMode::MFSK;
                        LOG_MODEM(WARN, "Connection: Falling back to MFSK after %d DPSK attempts",
                                  DPSK_ATTEMPTS);
                        if (on_connect_waveform_changed_) {
                            on_connect_waveform_changed_(connect_waveform_);
                        }
                    }

                    LOG_MODEM(WARN, "Connection: Connect timeout, retrying via %s (%d/%d)",
                              waveformModeToString(connect_waveform_),
                              connect_retry_count_ + 1, config_.connect_retries);
                    auto connect_frame = v2::ConnectFrame::makeConnect(local_call_, remote_call_,
                                                                        config_.mode_capabilities,
                                                                        static_cast<uint8_t>(config_.preferred_mode));
                    transmitFrame(connect_frame.serialize());
                    timeout_remaining_ms_ = config_.connect_timeout_ms;
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        case ConnectionState::CONNECTED:
            connected_time_ms_ += elapsed_ms;
            stats_.connected_time_ms = connected_time_ms_;

            // Handle MODE_CHANGE timeout
            if (mode_change_pending_) {
                if (elapsed_ms >= mode_change_timeout_ms_) {
                    mode_change_retry_count_++;
                    if (mode_change_retry_count_ > MODE_CHANGE_MAX_RETRIES) {
                        LOG_MODEM(WARN, "Connection: MODE_CHANGE failed after %d attempts, keeping current mode",
                                  MODE_CHANGE_MAX_RETRIES);
                        mode_change_pending_ = false;
                        // Stay at current mode - don't change anything
                    } else {
                        LOG_MODEM(WARN, "Connection: MODE_CHANGE timeout, retrying (%d/%d)",
                                  mode_change_retry_count_, MODE_CHANGE_MAX_RETRIES);
                        // Resend MODE_CHANGE with same parameters
                        auto frame = v2::ControlFrame::makeModeChange(local_call_, remote_call_,
                                                                       mode_change_seq_, pending_modulation_,
                                                                       pending_code_rate_, pending_snr_db_,
                                                                       pending_reason_);
                        transmitFrame(frame.serialize());
                        mode_change_timeout_ms_ = MODE_CHANGE_TIMEOUT_MS;
                    }
                } else {
                    mode_change_timeout_ms_ -= elapsed_ms;
                }
            }

            arq_.tick(elapsed_ms);
            break;

        case ConnectionState::DISCONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                LOG_MODEM(DEBUG, "Connection: Disconnect timeout, forcing disconnect");
                enterDisconnected("Disconnect timeout");
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        default:
            break;
    }
}

void Connection::transmitFrame(const Bytes& frame_data) {
    LOG_MODEM(DEBUG, "Connection: TX %zu bytes", frame_data.size());
    if (on_transmit_) {
        on_transmit_(frame_data);
    }
}

void Connection::enterConnected() {
    state_ = ConnectionState::CONNECTED;
    connected_time_ms_ = 0;

    arq_.setCallsigns(local_call_, remote_call_);
    arq_.reset();

    LOG_MODEM(INFO, "Connection: Now CONNECTED to %s (mode=%s)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_));

    if (on_mode_negotiated_) {
        on_mode_negotiated_(negotiated_mode_);
    }

    if (on_connected_) {
        on_connected_();
    }
}

void Connection::enterDisconnected(const std::string& reason) {
    state_ = ConnectionState::DISCONNECTED;
    std::string old_remote = remote_call_;
    remote_call_.clear();
    pending_remote_call_.clear();
    mode_change_pending_ = false;
    arq_.reset();
    file_transfer_.cancel();

    // Reset connect waveform to DPSK for next connection attempt
    connect_waveform_ = WaveformMode::DPSK;

    LOG_MODEM(INFO, "Connection: Disconnected from %s (%s)",
              old_remote.c_str(), reason.c_str());

    if (on_disconnected_) {
        on_disconnected_(reason);
    }
}

void Connection::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void Connection::setConnectedCallback(ConnectedCallback cb) {
    on_connected_ = std::move(cb);
}

void Connection::setDisconnectedCallback(DisconnectedCallback cb) {
    on_disconnected_ = std::move(cb);
}

void Connection::setMessageReceivedCallback(MessageReceivedCallback cb) {
    on_message_received_ = std::move(cb);
}

void Connection::setMessageSentCallback(MessageSentCallback cb) {
    on_message_sent_ = std::move(cb);
}

void Connection::setIncomingCallCallback(IncomingCallCallback cb) {
    on_incoming_call_ = std::move(cb);
}

void Connection::setDataReceivedCallback(DataReceivedCallback cb) {
    on_data_received_ = std::move(cb);
}

void Connection::setFileProgressCallback(FileProgressCallback cb) {
    file_transfer_.setProgressCallback(std::move(cb));
}

void Connection::setFileReceivedCallback(FileReceivedCallback cb) {
    file_transfer_.setReceivedCallback(std::move(cb));
}

void Connection::setFileSentCallback(FileSentCallback cb) {
    file_transfer_.setSentCallback(std::move(cb));
}

ConnectionStats Connection::getStats() const {
    ConnectionStats s = stats_;
    s.arq = arq_.getStats();
    return s;
}

void Connection::resetStats() {
    stats_ = ConnectionStats{};
    arq_.resetStats();
}

void Connection::reset() {
    state_ = ConnectionState::DISCONNECTED;
    remote_call_.clear();
    pending_remote_call_.clear();
    timeout_remaining_ms_ = 0;
    connect_retry_count_ = 0;
    connected_time_ms_ = 0;
    negotiated_mode_ = WaveformMode::OFDM;
    remote_capabilities_ = ModeCapabilities::OFDM;
    remote_preferred_ = WaveformMode::OFDM;
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    data_modulation_ = Modulation::DQPSK;
    data_code_rate_ = CodeRate::R1_4;
    connect_waveform_ = WaveformMode::DPSK;  // Reset to DPSK for next connect attempt
    arq_.reset();
    file_transfer_.cancel();
    LOG_MODEM(DEBUG, "Connection: Full reset");
}

WaveformMode Connection::negotiateMode(uint8_t remote_caps, WaveformMode remote_pref) {
    uint8_t common = config_.mode_capabilities & remote_caps;

    if (common == 0) {
        LOG_MODEM(WARN, "Connection: No common waveform modes! Falling back to OFDM");
        return WaveformMode::OFDM;
    }

    // Helper to convert WaveformMode to capability bit
    auto modeToBit = [](WaveformMode mode) -> uint8_t {
        switch (mode) {
            case WaveformMode::OFDM:     return ModeCapabilities::OFDM;
            case WaveformMode::OTFS_EQ:  return ModeCapabilities::OTFS_EQ;
            case WaveformMode::OTFS_RAW: return ModeCapabilities::OTFS_RAW;
            case WaveformMode::MFSK:     return ModeCapabilities::MFSK;
            case WaveformMode::DPSK:     return ModeCapabilities::DPSK;
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
    //   0-10 dB: DPSK (single-carrier, robust on flutter)
    //   > 10 dB: OFDM (highest throughput)
    float snr = measured_snr_db_;
    LOG_MODEM(INFO, "Connection: AUTO mode selection, SNR=%.1f dB", snr);

    if (snr < 0.0f && (common & ModeCapabilities::MFSK)) {
        LOG_MODEM(INFO, "Connection: Selected MFSK for very low SNR (%.1f dB)", snr);
        return WaveformMode::MFSK;
    }

    if (snr < 10.0f && (common & ModeCapabilities::DPSK)) {
        LOG_MODEM(INFO, "Connection: Selected DPSK for low SNR (%.1f dB)", snr);
        return WaveformMode::DPSK;
    }

    // Default priority for adequate SNR: OFDM > OTFS > DPSK > MFSK
    if (common & ModeCapabilities::OFDM) {
        LOG_MODEM(INFO, "Connection: Selected OFDM (SNR=%.1f dB)", snr);
        return WaveformMode::OFDM;
    }
    if (common & ModeCapabilities::OTFS_EQ) return WaveformMode::OTFS_EQ;
    if (common & ModeCapabilities::OTFS_RAW) return WaveformMode::OTFS_RAW;
    if (common & ModeCapabilities::DPSK) return WaveformMode::DPSK;
    if (common & ModeCapabilities::MFSK) return WaveformMode::MFSK;

    return WaveformMode::OFDM;
}

} // namespace protocol
} // namespace ultra
