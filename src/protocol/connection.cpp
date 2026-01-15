#include "connection.hpp"
#include "ultra/logging.hpp"

namespace ultra {
namespace protocol {

const char* connectionStateToString(ConnectionState state) {
    switch (state) {
        case ConnectionState::DISCONNECTED:  return "DISCONNECTED";
        case ConnectionState::PROBING:       return "PROBING";
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
    arq_.setTransmitCallback([this](const Frame& f) {
        transmitFrame(f);
    });

    arq_.setDataReceivedCallback([this](const Bytes& data) {
        // Route through data payload handler for file/text discrimination
        handleDataPayload(data, arq_.lastRxHadMoreData());
    });

    arq_.setSendCompleteCallback([this](bool success) {
        // Check if this was a file transfer chunk
        if (file_transfer_.getState() == FileTransferState::SENDING) {
            if (success) {
                file_transfer_.onChunkAcked();
                // Send next chunk if available
                sendNextFileChunk();
            } else {
                file_transfer_.onSendFailed();
            }
        } else {
            // Regular message
            if (on_message_sent_) {
                on_message_sent_(success);
            }
        }
    });

    // Wire up file transfer callbacks (forward to our callbacks)
    file_transfer_.setProgressCallback([this](const FileTransferProgress& p) {
        // Progress callbacks are set directly on file_transfer_
    });
}

void Connection::setLocalCallsign(const std::string& call) {
    local_call_ = Frame::sanitizeCallsign(call);
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

    remote_call_ = Frame::sanitizeCallsign(remote_call);
    if (remote_call_.empty() || !Frame::isValidCallsign(remote_call_)) {
        LOG_MODEM(ERROR, "Connection: Invalid remote callsign: %s", remote_call.c_str());
        return false;
    }

    LOG_MODEM(INFO, "Connection: Connecting to %s (probing first for optimal mode)",
              remote_call_.c_str());

    // Start with PROBE to measure channel quality, then auto-connect with optimal mode
    // This ensures we always use the best modulation for current channel conditions
    Frame probe_frame = Frame::makeProbe(local_call_, remote_call_, config_.mode_capabilities);
    transmitFrame(probe_frame);

    state_ = ConnectionState::PROBING;
    probe_retry_count_ = 0;
    probe_complete_ = false;
    connect_after_probe_ = true;  // Auto-connect after probe completes
    timeout_remaining_ms_ = config_.probe_timeout_ms;
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

    // Negotiate waveform mode (remote_capabilities_ was stored in handleConnect)
    negotiated_mode_ = negotiateMode(remote_capabilities_, remote_preferred_);
    last_channel_report_.recommended_mode = negotiated_mode_;

    LOG_MODEM(INFO, "Connection: Accepting call from %s (mode=%s)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_));

    // Send CONNECT_ACK with negotiated mode AND channel report
    // (last_channel_report_ was populated in handleConnect when we decoded their CONNECT)
    Frame ack_frame = Frame::makeConnectAckWithReport(local_call_, remote_call_,
                                                      negotiated_mode_, last_channel_report_);
    transmitFrame(ack_frame);

    enterConnected();
}

void Connection::rejectCall() {
    if (pending_remote_call_.empty()) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Rejecting call from %s", pending_remote_call_.c_str());

    // Send CONNECT_NAK
    Frame nak_frame = Frame::makeConnectNak(local_call_, pending_remote_call_);
    transmitFrame(nak_frame);

    pending_remote_call_.clear();
}

bool Connection::probe(const std::string& remote_call) {
    if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot probe, state=%s",
                  connectionStateToString(state_));
        return false;
    }

    if (local_call_.empty()) {
        LOG_MODEM(ERROR, "Connection: Local callsign not set");
        return false;
    }

    remote_call_ = Frame::sanitizeCallsign(remote_call);
    if (remote_call_.empty() || !Frame::isValidCallsign(remote_call_)) {
        LOG_MODEM(ERROR, "Connection: Invalid remote callsign: %s", remote_call.c_str());
        return false;
    }

    LOG_MODEM(INFO, "Connection: Probing channel to %s (caps=0x%02X)",
              remote_call_.c_str(), config_.mode_capabilities);

    // Send PROBE frame with our capabilities
    // The PROBE frame includes a long training sequence for channel estimation
    Frame probe_frame = Frame::makeProbe(local_call_, remote_call_, config_.mode_capabilities);
    transmitFrame(probe_frame);

    state_ = ConnectionState::PROBING;
    probe_retry_count_ = 0;
    probe_complete_ = false;
    connect_after_probe_ = false;  // Standalone probe - don't auto-connect
    timeout_remaining_ms_ = config_.probe_timeout_ms;

    return true;
}

bool Connection::connectAfterProbe() {
    if (state_ != ConnectionState::DISCONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot connect, state=%s",
                  connectionStateToString(state_));
        return false;
    }

    if (!probe_complete_) {
        LOG_MODEM(WARN, "Connection: No probe results available");
        return false;
    }

    if (remote_call_.empty()) {
        LOG_MODEM(ERROR, "Connection: No remote callsign from probe");
        return false;
    }

    // Use recommended mode from channel report as our preference
    config_.preferred_mode = last_channel_report_.recommended_mode;

    LOG_MODEM(INFO, "Connection: Connecting to %s after probe (recommended mode: %s)",
              remote_call_.c_str(), waveformModeToString(config_.preferred_mode));

    // Send CONNECT frame with our mode capabilities and the recommended preference
    Frame connect_frame = Frame::makeConnect(local_call_, remote_call_,
                                             config_.mode_capabilities,
                                             config_.preferred_mode);
    transmitFrame(connect_frame);

    state_ = ConnectionState::CONNECTING;
    connect_retry_count_ = 0;
    timeout_remaining_ms_ = config_.connect_timeout_ms;
    stats_.connects_initiated++;

    return true;
}

void Connection::disconnect() {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    if (state_ == ConnectionState::CONNECTING || state_ == ConnectionState::PROBING) {
        // Cancel connection/probe attempt
        enterDisconnected("Cancelled");
        return;
    }

    if (state_ == ConnectionState::CONNECTED) {
        LOG_MODEM(INFO, "Connection: Disconnecting from %s", remote_call_.c_str());

        // Send DISCONNECT frame
        Frame disc_frame = Frame::makeDisconnect(local_call_, remote_call_);
        transmitFrame(disc_frame);

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

    // Send first chunk
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

    // Set MORE_DATA flag if more chunks follow
    uint8_t flags = file_transfer_.hasMoreChunks() ? FrameFlags::MORE_DATA : FrameFlags::NONE;

    arq_.sendDataWithFlags(chunk, flags);
}

void Connection::handleDataPayload(const Bytes& payload, bool more_data) {
    if (payload.empty()) {
        return;
    }

    // Check if this is a file transfer frame
    if (file_transfer_.processPayload(payload, more_data)) {
        // Was a file transfer frame, already handled
        LOG_MODEM(DEBUG, "Connection: Processed file transfer payload (%zu bytes)",
                  payload.size());

        // Also notify raw data callback if set
        if (on_data_received_) {
            on_data_received_(payload, more_data);
        }
        return;
    }

    // Regular text message (starts with TEXT_MESSAGE type or is legacy)
    // Skip type byte if present
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

void Connection::onFrameReceived(const Frame& frame) {
    // Check if frame is for us
    if (!frame.isForCallsign(local_call_)) {
        return;
    }

    LOG_MODEM(DEBUG, "Connection: Received %s from %s",
              frameTypeToString(frame.type), frame.src_call.c_str());

    switch (frame.type) {
        case FrameType::CONNECT:
            handleConnect(frame);
            break;
        case FrameType::CONNECT_ACK:
            handleConnectAck(frame);
            break;
        case FrameType::CONNECT_NAK:
            handleConnectNak(frame);
            break;
        case FrameType::DISCONNECT:
            handleDisconnect(frame);
            break;
        case FrameType::PROBE:
            handleProbe(frame);
            break;
        case FrameType::PROBE_ACK:
            handleProbeAck(frame);
            break;
        case FrameType::DATA:
        case FrameType::NAK:
        case FrameType::SACK:
            // Pass to ARQ layer
            if (state_ == ConnectionState::CONNECTED) {
                arq_.onFrameReceived(frame);
            }
            break;
        case FrameType::ACK:
            // Handle ACK to our DISCONNECT
            if (state_ == ConnectionState::DISCONNECTING) {
                if (frame.src_call == remote_call_) {
                    LOG_MODEM(INFO, "Connection: Disconnect acknowledged by %s",
                              frame.src_call.c_str());
                    enterDisconnected("Disconnect complete");
                }
            } else if (state_ == ConnectionState::CONNECTED) {
                // Pass to ARQ layer for data ACKs
                arq_.onFrameReceived(frame);
            }
            break;
        case FrameType::BEACON:
            // Could notify application of heard stations
            LOG_MODEM(DEBUG, "Connection: Heard beacon from %s", frame.src_call.c_str());
            break;
    }
}

void Connection::handleConnect(const Frame& frame) {
    if (state_ != ConnectionState::DISCONNECTED) {
        // Already busy, reject
        LOG_MODEM(WARN, "Connection: Rejecting CONNECT from %s (busy, state=%s)",
                  frame.src_call.c_str(), connectionStateToString(state_));
        Frame nak = Frame::makeConnectNak(local_call_, frame.src_call);
        transmitFrame(nak);
        return;
    }

    // Parse mode capabilities from CONNECT frame
    uint8_t remote_caps = ModeCapabilities::OFDM;  // Default for old clients
    WaveformMode remote_pref = WaveformMode::OFDM;
    Frame::parseConnectPayload(frame.payload, remote_caps, remote_pref);

    LOG_MODEM(INFO, "Connection: Incoming call from %s (caps=0x%02X, pref=%s)",
              frame.src_call.c_str(), remote_caps, waveformModeToString(remote_pref));
    stats_.connects_received++;

    // Store remote capabilities for negotiation
    remote_capabilities_ = remote_caps;
    remote_preferred_ = remote_pref;

    // Get our channel measurements - we just decoded their frame, so we have
    // real measurements of the channel from our perspective (implicit probing!)
    ChannelReport our_report;
    if (on_measure_channel_) {
        ChannelQuality quality = on_measure_channel_();
        our_report.snr_db = quality.snr_db;
        our_report.delay_spread_ms = quality.delay_spread_ms;
        our_report.doppler_spread_hz = quality.doppler_hz;
        LOG_MODEM(INFO, "Connection: Channel from %s: SNR=%.1f dB, delay=%.2f ms, doppler=%.2f Hz (%s)",
                  frame.src_call.c_str(), quality.snr_db, quality.delay_spread_ms,
                  quality.doppler_hz, our_report.getConditionName());
    } else {
        // Fallback defaults
        our_report.snr_db = 15.0f;
        our_report.delay_spread_ms = 1.0f;
        our_report.doppler_spread_hz = 0.5f;
    }
    our_report.capabilities = config_.mode_capabilities;

    if (config_.auto_accept) {
        // Auto-accept: negotiate mode and connect
        remote_call_ = frame.src_call;

        // Negotiate waveform mode - consider our channel measurements
        // If channel conditions suggest a different mode, we can override remote preference
        WaveformMode channel_recommended = WaveformMode::OFDM;
        if (our_report.doppler_spread_hz > 5.0f) {
            channel_recommended = WaveformMode::OTFS_RAW;  // Flutter
        } else if (our_report.snr_db > 20.0f && our_report.delay_spread_ms < 1.0f) {
            channel_recommended = WaveformMode::OTFS_EQ;   // Excellent conditions
        }

        // If remote prefers AUTO, use our channel-based recommendation
        if (remote_pref == WaveformMode::AUTO) {
            remote_pref = channel_recommended;
        }

        negotiated_mode_ = negotiateMode(remote_caps, remote_pref);
        our_report.recommended_mode = negotiated_mode_;

        LOG_MODEM(INFO, "Connection: Negotiated waveform mode: %s",
                  waveformModeToString(negotiated_mode_));

        // Send CONNECT_ACK with negotiated mode AND our channel report
        // This lets the caller know our view of the channel (implicit probe response)
        Frame ack = Frame::makeConnectAckWithReport(local_call_, remote_call_,
                                                    negotiated_mode_, our_report);
        transmitFrame(ack);
        enterConnected();
    } else {
        // Store for manual accept/reject
        pending_remote_call_ = frame.src_call;
        // Store channel report for when acceptCall() is called
        last_channel_report_ = our_report;
        if (on_incoming_call_) {
            on_incoming_call_(frame.src_call);
        }
    }
}

void Connection::handleConnectAck(const Frame& frame) {
    if (state_ != ConnectionState::CONNECTING) {
        LOG_MODEM(DEBUG, "Connection: Ignoring CONNECT_ACK (state=%s)",
                  connectionStateToString(state_));
        return;
    }

    if (frame.src_call != remote_call_) {
        LOG_MODEM(WARN, "Connection: CONNECT_ACK from wrong station %s (expected %s)",
                  frame.src_call.c_str(), remote_call_.c_str());
        return;
    }

    // Parse negotiated mode and optional channel report from CONNECT_ACK
    WaveformMode mode = WaveformMode::OFDM;  // Default for old clients
    ChannelReport remote_report;
    bool has_report = false;
    Frame::parseConnectAckPayload(frame.payload, mode, remote_report, has_report);
    negotiated_mode_ = mode;

    if (has_report) {
        // Remote included their channel measurements - this is their view of the channel
        // (implicit probe response - they measured the channel when decoding our CONNECT)
        last_channel_report_ = remote_report;
        LOG_MODEM(INFO, "Connection: Connected to %s (mode=%s) - Remote reports: "
                  "SNR=%.1f dB, delay=%.1f ms, doppler=%.1f Hz (%s)",
                  remote_call_.c_str(), waveformModeToString(negotiated_mode_),
                  remote_report.snr_db, remote_report.delay_spread_ms,
                  remote_report.doppler_spread_hz, remote_report.getConditionName());
    } else {
        LOG_MODEM(INFO, "Connection: Connected to %s (mode=%s)",
                  remote_call_.c_str(), waveformModeToString(negotiated_mode_));
    }

    enterConnected();
}

void Connection::handleConnectNak(const Frame& frame) {
    if (state_ != ConnectionState::CONNECTING) {
        return;
    }

    if (frame.src_call != remote_call_) {
        return;
    }

    LOG_MODEM(WARN, "Connection: Connection rejected by %s", remote_call_.c_str());
    stats_.connects_failed++;
    enterDisconnected("Connection rejected");
}

void Connection::handleDisconnect(const Frame& frame) {
    if (state_ == ConnectionState::DISCONNECTED) {
        return;
    }

    if (frame.src_call != remote_call_) {
        return;
    }

    LOG_MODEM(INFO, "Connection: Disconnect from %s", remote_call_.c_str());

    // ACK the disconnect
    Frame ack = Frame::makeAck(local_call_, remote_call_, 0);
    transmitFrame(ack);

    stats_.disconnects++;
    enterDisconnected("Remote disconnected");
}

void Connection::handleProbe(const Frame& frame) {
    // Someone is probing us - they want to measure the channel
    // We respond with PROBE_ACK containing our channel measurements

    LOG_MODEM(INFO, "Connection: Received PROBE from %s", frame.src_call.c_str());

    // Parse their capabilities
    uint8_t remote_caps = ModeCapabilities::OFDM;  // Default
    if (!frame.payload.empty()) {
        remote_caps = frame.payload[0];
    }

    // Get channel measurements from modem (if callback is set)
    ChannelReport report;
    if (on_measure_channel_) {
        // Get real channel measurements from modem layer
        ChannelQuality quality = on_measure_channel_();
        report.snr_db = quality.snr_db;
        report.delay_spread_ms = quality.delay_spread_ms;
        report.doppler_spread_hz = quality.doppler_hz;
        LOG_MODEM(DEBUG, "Connection: Got channel measurements from modem: "
                  "SNR=%.1f dB, delay=%.2f ms, doppler=%.2f Hz",
                  quality.snr_db, quality.delay_spread_ms, quality.doppler_hz);
    } else {
        // Fallback to reasonable defaults if no measurement callback
        LOG_MODEM(WARN, "Connection: No channel measurement callback, using defaults");
        report.snr_db = 15.0f;
        report.delay_spread_ms = 1.0f;
        report.doppler_spread_hz = 0.5f;
    }
    report.capabilities = config_.mode_capabilities;

    // Determine recommended mode based on channel conditions
    // Priority: handle pathological cases first, then optimize for good conditions
    if (report.doppler_spread_hz > 5.0f) {
        // High Doppler (flutter) - use OTFS-RAW (no TF equalization)
        // OTFS handles time-varying channels better than OFDM
        report.recommended_mode = WaveformMode::OTFS_RAW;
    } else if (report.delay_spread_ms > 2.0f) {
        // Very high delay spread (severe multipath) - use standard OFDM (most robust)
        report.recommended_mode = WaveformMode::OFDM;
    } else if (report.snr_db < 10.0f) {
        // Low SNR - use OFDM (most mature, best low-SNR performance)
        report.recommended_mode = WaveformMode::OFDM;
    } else if (report.snr_db > 20.0f && report.delay_spread_ms < 1.0f && report.doppler_spread_hz < 1.0f) {
        // Excellent conditions - OTFS-EQ can provide best throughput
        report.recommended_mode = WaveformMode::OTFS_EQ;
    } else if (report.doppler_spread_hz > 1.0f) {
        // Moderate Doppler - OTFS-RAW handles this better
        report.recommended_mode = WaveformMode::OTFS_RAW;
    } else {
        // Default to OFDM (most compatible and robust)
        report.recommended_mode = WaveformMode::OFDM;
    }

    LOG_MODEM(INFO, "Connection: Sending PROBE_ACK to %s (SNR=%.1f dB, delay=%.1f ms, "
              "doppler=%.1f Hz, condition=%s, recommended=%s)",
              frame.src_call.c_str(), report.snr_db, report.delay_spread_ms,
              report.doppler_spread_hz, report.getConditionName(),
              waveformModeToString(report.recommended_mode));

    // Send PROBE_ACK with channel report
    Frame probe_ack = Frame::makeProbeAck(local_call_, frame.src_call, report);
    transmitFrame(probe_ack);
}

void Connection::handleProbeAck(const Frame& frame) {
    if (state_ != ConnectionState::PROBING) {
        LOG_MODEM(DEBUG, "Connection: Ignoring PROBE_ACK (state=%s)",
                  connectionStateToString(state_));
        return;
    }

    if (frame.src_call != remote_call_) {
        LOG_MODEM(WARN, "Connection: PROBE_ACK from wrong station %s (expected %s)",
                  frame.src_call.c_str(), remote_call_.c_str());
        return;
    }

    // Parse channel report from payload
    ChannelReport report;
    if (!Frame::parseProbeAckPayload(frame.payload, report)) {
        LOG_MODEM(WARN, "Connection: Failed to parse PROBE_ACK payload");
        return;
    }

    LOG_MODEM(INFO, "Connection: Received channel report from %s: SNR=%.1f dB, "
              "delay=%.1f ms, doppler=%.1f Hz, recommended=%s, condition=%s",
              frame.src_call.c_str(), report.snr_db, report.delay_spread_ms,
              report.doppler_spread_hz, waveformModeToString(report.recommended_mode),
              report.getConditionName());

    // Store the channel report
    last_channel_report_ = report;
    probe_complete_ = true;

    // Store remote capabilities for later negotiation
    remote_capabilities_ = report.capabilities;

    // Notify callback (before potentially transitioning to CONNECTING)
    if (on_probe_complete_) {
        on_probe_complete_(report);
    }

    // Auto-connect if this was initiated via connect() (not standalone probe())
    if (connect_after_probe_) {
        connect_after_probe_ = false;

        // Use recommended mode from channel report as our preference
        config_.preferred_mode = report.recommended_mode;

        LOG_MODEM(INFO, "Connection: Auto-connecting to %s (recommended mode: %s)",
                  remote_call_.c_str(), waveformModeToString(config_.preferred_mode));

        // Send CONNECT frame with optimal mode
        Frame connect_frame = Frame::makeConnect(local_call_, remote_call_,
                                                 config_.mode_capabilities,
                                                 config_.preferred_mode);
        transmitFrame(connect_frame);

        state_ = ConnectionState::CONNECTING;
        connect_retry_count_ = 0;
        timeout_remaining_ms_ = config_.connect_timeout_ms;
    } else {
        // Standalone probe - return to DISCONNECTED, user can call connectAfterProbe()
        state_ = ConnectionState::DISCONNECTED;
    }
}

void Connection::tick(uint32_t elapsed_ms) {
    switch (state_) {
        case ConnectionState::PROBING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                // Probe timeout
                probe_retry_count_++;
                if (probe_retry_count_ >= config_.probe_retries) {
                    LOG_MODEM(ERROR, "Connection: Probe failed after %d attempts",
                              config_.probe_retries);
                    enterDisconnected("Probe timeout");
                } else {
                    // Retry
                    LOG_MODEM(WARN, "Connection: Probe timeout, retrying (%d/%d)",
                              probe_retry_count_ + 1, config_.probe_retries);
                    Frame probe_frame = Frame::makeProbe(local_call_, remote_call_,
                                                         config_.mode_capabilities);
                    transmitFrame(probe_frame);
                    timeout_remaining_ms_ = config_.probe_timeout_ms;
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        case ConnectionState::CONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                // Connection timeout
                connect_retry_count_++;
                if (connect_retry_count_ >= config_.connect_retries) {
                    LOG_MODEM(ERROR, "Connection: Connect failed after %d attempts",
                              config_.connect_retries);
                    stats_.connects_failed++;
                    enterDisconnected("Connection timeout");
                } else {
                    // Retry
                    LOG_MODEM(WARN, "Connection: Connect timeout, retrying (%d/%d)",
                              connect_retry_count_ + 1, config_.connect_retries);
                    Frame connect_frame = Frame::makeConnect(local_call_, remote_call_,
                                                             config_.mode_capabilities,
                                                             config_.preferred_mode);
                    transmitFrame(connect_frame);
                    timeout_remaining_ms_ = config_.connect_timeout_ms;
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

        case ConnectionState::CONNECTED:
            connected_time_ms_ += elapsed_ms;
            stats_.connected_time_ms = connected_time_ms_;
            arq_.tick(elapsed_ms);
            break;

        case ConnectionState::DISCONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                // Disconnect timeout - just go to disconnected anyway
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

void Connection::transmitFrame(const Frame& frame) {
    LOG_MODEM(INFO, "TX >> %s [%s -> %s] seq=%d payload=%zu bytes",
              frameTypeToString(frame.type),
              frame.src_call.c_str(), frame.dst_call.c_str(),
              frame.sequence, frame.payload.size());
    if (on_transmit_) {
        on_transmit_(frame);
    }
}

void Connection::enterConnected() {
    state_ = ConnectionState::CONNECTED;
    connected_time_ms_ = 0;

    // Configure ARQ with callsigns
    arq_.setCallsigns(local_call_, remote_call_);
    arq_.reset();

    LOG_MODEM(INFO, "Connection: Now CONNECTED to %s (mode=%s)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_));

    // Notify about negotiated mode
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
    arq_.reset();
    file_transfer_.cancel();

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
    probe_retry_count_ = 0;
    connected_time_ms_ = 0;
    negotiated_mode_ = WaveformMode::OFDM;
    remote_capabilities_ = ModeCapabilities::OFDM;
    remote_preferred_ = WaveformMode::OFDM;
    probe_complete_ = false;
    connect_after_probe_ = false;
    arq_.reset();
    file_transfer_.cancel();
    LOG_MODEM(DEBUG, "Connection: Full reset");
}

WaveformMode Connection::negotiateMode(uint8_t remote_caps, WaveformMode remote_pref) {
    // Find common capabilities
    uint8_t common = config_.mode_capabilities & remote_caps;

    if (common == 0) {
        // No common modes - shouldn't happen if both support OFDM
        LOG_MODEM(WARN, "Connection: No common waveform modes! Falling back to OFDM");
        return WaveformMode::OFDM;
    }

    // If remote has a preference and we support it, use it
    if (remote_pref != WaveformMode::AUTO) {
        uint8_t pref_bit = 0;
        switch (remote_pref) {
            case WaveformMode::OFDM:     pref_bit = ModeCapabilities::OFDM; break;
            case WaveformMode::OTFS_EQ:  pref_bit = ModeCapabilities::OTFS_EQ; break;
            case WaveformMode::OTFS_RAW: pref_bit = ModeCapabilities::OTFS_RAW; break;
            default: break;
        }
        if (common & pref_bit) {
            LOG_MODEM(INFO, "Connection: Using remote preferred mode: %s",
                      waveformModeToString(remote_pref));
            return remote_pref;
        }
    }

    // If we have a preference and it's supported, use it
    if (config_.preferred_mode != WaveformMode::AUTO) {
        uint8_t pref_bit = 0;
        switch (config_.preferred_mode) {
            case WaveformMode::OFDM:     pref_bit = ModeCapabilities::OFDM; break;
            case WaveformMode::OTFS_EQ:  pref_bit = ModeCapabilities::OTFS_EQ; break;
            case WaveformMode::OTFS_RAW: pref_bit = ModeCapabilities::OTFS_RAW; break;
            default: break;
        }
        if (common & pref_bit) {
            LOG_MODEM(INFO, "Connection: Using local preferred mode: %s",
                      waveformModeToString(config_.preferred_mode));
            return config_.preferred_mode;
        }
    }

    // Default priority: OFDM > OTFS_EQ > OTFS_RAW
    // (OFDM is most compatible/robust default)
    if (common & ModeCapabilities::OFDM) {
        LOG_MODEM(INFO, "Connection: Defaulting to OFDM (most compatible)");
        return WaveformMode::OFDM;
    }
    if (common & ModeCapabilities::OTFS_EQ) {
        LOG_MODEM(INFO, "Connection: Using OTFS-EQ");
        return WaveformMode::OTFS_EQ;
    }
    if (common & ModeCapabilities::OTFS_RAW) {
        LOG_MODEM(INFO, "Connection: Using OTFS-RAW");
        return WaveformMode::OTFS_RAW;
    }

    // Fallback (should never reach)
    return WaveformMode::OFDM;
}

} // namespace protocol
} // namespace ultra
