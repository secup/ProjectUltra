// Connection state machine - core logic
// Frame handlers are in connection_handlers.cpp

#include "connection.hpp"
#include "connection_policy.hpp"
#include "gui/startup_trace.hpp"
#include "waveform_selection.hpp"
#include "ultra/logging.hpp"
#include <algorithm>

namespace ultra {
namespace protocol {

namespace {
constexpr size_t kOFDMFileBlockPayloadLimit = 2300;
constexpr const char* kOFDMBurstPadCallsign = "ULPAD";
constexpr uint16_t kOFDMBurstPadSeq = 0xFFFE;

Bytes makeOFDMBurstPadPayload(CodeRate rate, int cw_count, size_t pad_index) {
    const size_t capacity = v2::getFixedFramePayloadCapacity(rate, cw_count);
    Bytes payload(capacity);
    if (payload.empty()) {
        return payload;
    }

    // Fill the dummy frame instead of sending an empty DATA payload. Empty pad
    // frames leave most fixed-frame info bits as all-zero LDPC codewords, which
    // are prone to ugly "4/4 CWs OK but frame invalid" tail artifacts in fading.
    uint32_t x = 0xA5C35A7Du ^
                 (static_cast<uint32_t>(rate) << 24) ^
                 static_cast<uint32_t>(pad_index * 0x9E3779B1u);
    for (size_t i = 0; i < payload.size(); ++i) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        payload[i] = static_cast<uint8_t>(x & 0xFF);
    }
    payload[0] = 0x7F;  // reserved dummy discriminator if a hash collision ever delivers it
    return payload;
}

bool shouldUseSingleOFDMFileBlock(float fading_index, float snr_db, CodeRate rate) {
    return connection_policy::isNearAwgnOFDM(fading_index, snr_db) &&
           getCodeRateValue(rate) >= getCodeRateValue(CodeRate::R2_3);
}

bool shouldPadPartialOFDMBurst(WaveformMode mode,
                               Modulation modulation,
                               CodeRate rate,
                               float fading_index,
                               float snr_db,
                               FileTransferState file_state,
                               size_t burst_frames) {
    if (!isOFDMMode(mode)) {
        return false;
    }
    if (file_state != FileTransferState::SENDING) {
        return connection_policy::shouldPadBurstInterleaveGroup(burst_frames);
    }

    return connection_policy::shouldPadHighRateFadingBurst(
        modulation,
        rate,
        connection_policy::isNearAwgnOFDM(fading_index, snr_db),
        burst_frames);
}

int statDelta(int now, int before) {
    return std::max(0, now - before);
}

bool hasAdaptiveRetryPressure(const ARQStats& now, const ARQStats& before) {
    const int retransmissions =
        statDelta(now.retransmissions, before.retransmissions);
    const int timeouts = statDelta(now.timeouts, before.timeouts);
    const int failed = statDelta(now.failed, before.failed);
    const int holes = statDelta(now.hole_events, before.hole_events);

    return timeouts > 0 || failed > 0 || retransmissions >= 2 || holes >= 2;
}

bool hasCleanAdaptiveWindow(const ARQStats& now, const ARQStats& before) {
    return statDelta(now.retransmissions, before.retransmissions) == 0 &&
           statDelta(now.timeouts, before.timeouts) == 0 &&
           statDelta(now.failed, before.failed) == 0 &&
           statDelta(now.hole_events, before.hole_events) == 0;
}

bool isFasterRate(CodeRate candidate, CodeRate current) {
    return getCodeRateValue(candidate) > getCodeRateValue(current);
}

bool isMoreRobustRate(CodeRate candidate, CodeRate current) {
    return getCodeRateValue(candidate) < getCodeRateValue(current);
}

CodeRate oneStepMoreRobust(CodeRate rate) {
    switch (rate) {
        case CodeRate::R3_4: return CodeRate::R2_3;
        case CodeRate::R2_3: return CodeRate::R1_2;
        case CodeRate::R1_2: return CodeRate::R1_4;
        default: return CodeRate::R1_4;
    }
}

CodeRate adaptiveDowngradeTarget(CodeRate current, CodeRate recommended) {
    CodeRate one_step = oneStepMoreRobust(current);
    if (isMoreRobustRate(recommended, one_step)) {
        return recommended;
    }
    return one_step;
}
}

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

// =============================================================================
// CONSTRUCTOR
// =============================================================================

Connection::Connection(const ConnectionConfig& config)
    : config_(config)
    , arq_(config.arq)
{
    ultra::gui::startupTrace("Connection", "ctor-enter");
    config_.pong_tx_delay_ms = std::min<uint32_t>(config_.pong_tx_delay_ms, 2000u);
    data_frame_cw_count_ = v2::sanitizeFixedFrameCodewords(config_.fixed_frame_codewords);
    config_.fixed_frame_codewords = data_frame_cw_count_;
    arq_.setFixedFrameCodewords(data_frame_cw_count_);

    // Wire up ARQ callbacks
    arq_.setTransmitCallback([this](const Bytes& data) {
        transmitFrame(data);
    });

    arq_.setDataReceivedCallback([this](const Bytes& data) {
        handleDataPayload(data, arq_.lastRxHadMoreData(), arq_.lastRxFrameType());
    });

    arq_.setSendCompleteCallback([this](bool success) {
        if (file_transfer_.getState() == FileTransferState::SENDING) {
            if (success) {
                file_transfer_.onChunkAcked();
                if (arq_callback_defer_refill_) {
                    deferred_file_refill_ = true;
                } else {
                    sendNextFileChunk();
                }
            } else {
                file_transfer_.onSendFailed();
            }
        } else if (!pending_tx_fragments_.empty()) {
            if (!success) {
                // Fragment send failed - abort remaining fragments
                LOG_MODEM(WARN, "Connection: Fragment send failed, aborting remaining %zu fragments",
                          pending_tx_fragments_.size() - next_fragment_idx_);
                pending_tx_fragments_.clear();
                pending_tx_fragment_flags_.clear();
                pending_tx_fragment_types_.clear();
                next_fragment_idx_ = 0;
                acked_fragment_count_ = 0;
                if (on_message_sent_) {
                    on_message_sent_(false);
                }
            } else {
                acked_fragment_count_++;

                if (next_fragment_idx_ < pending_tx_fragments_.size()) {
                    // More fragments to submit to ARQ
                    if (arq_callback_defer_refill_) {
                        deferred_fragment_refill_ = true;
                    } else {
                        sendNextFragment();
                    }
                }

                if (acked_fragment_count_ >= pending_tx_fragments_.size()) {
                    // ALL fragments truly ACKed
                    LOG_MODEM(INFO, "Connection: All %zu fragments sent and ACKed",
                              pending_tx_fragments_.size());
                    pending_tx_fragments_.clear();
                    pending_tx_fragment_flags_.clear();
                    pending_tx_fragment_types_.clear();
                    next_fragment_idx_ = 0;
                    acked_fragment_count_ = 0;
                    if (on_message_sent_) {
                        on_message_sent_(true);
                    }
                }
            }
        } else {
            if (on_message_sent_) {
                on_message_sent_(success);
            }
        }
    });
    ultra::gui::startupTrace("Connection", "ctor-exit");
}

// =============================================================================
// CONFIGURATION
// =============================================================================

void Connection::setLocalCallsign(const std::string& call) {
    ultra::gui::startupTrace("Connection", "setLocalCallsign-enter");
    local_call_ = sanitizeCallsign(call);
    ultra::gui::startupTrace("Connection", "setLocalCallsign-exit");
}

// =============================================================================
// CONNECTION CONTROL
// =============================================================================

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

    LOG_MODEM(INFO, "Connection: Connecting to %s (starting with PING probe)", remote_call_.c_str());

    cancelPendingPongCallback();

    // Use current connect_waveform_ (can be pre-set via setInitialConnectWaveform)
    // Notify the modem of the waveform to use
    if (on_connect_waveform_changed_) {
        on_connect_waveform_changed_(connect_waveform_);
    }

    // Start with PROBING state - send PING for fast presence check
    state_ = ConnectionState::PROBING;
    ping_retry_count_ = 0;
    timeout_remaining_ms_ = pingTimeoutMsForCurrentProfile();
    stats_.connects_initiated++;

    // Send PING (modem will generate preamble + "ULTR")
    if (on_ping_tx_) {
        LOG_MODEM(INFO, "Connection: Sending PING via %s",
                  waveformModeToString(connect_waveform_));
        on_ping_tx_();
    } else {
        // Fallback: no ping callback, send CONNECT directly
        LOG_MODEM(WARN, "Connection: No ping callback, sending CONNECT directly");
        sendFullConnect();
    }

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

    // Check if initiator forced specific modes (0xFF = AUTO, else forced)
    Modulation rec_mod;
    CodeRate rec_rate;

    // Use centralized algorithm from waveform_selection.hpp
    recommendDataMode(measured_snr_db_, negotiated_mode_, rec_mod, rec_rate, fading_index_);

    // Bootstrap safety: chirp SNR can overestimate first OFDM frame quality.
    if (isOFDMMode(negotiated_mode_)) {
        CodeRate capped = capInitialOFDMRate(measured_snr_db_, fading_index_, rec_rate);
        if (capped != rec_rate) {
            LOG_MODEM(INFO, "Connection: Bootstrap cap %s -> %s for initial OFDM setup (SNR=%.1f, fading=%.2f)",
                      codeRateToString(rec_rate), codeRateToString(capped), measured_snr_db_, fading_index_);
            rec_rate = capped;
        }
    }

    if (pending_forced_modulation_ != Modulation::AUTO) {
        // Initiator forced a specific modulation - honor it
        rec_mod = pending_forced_modulation_;
        LOG_MODEM(INFO, "Connection: Using FORCED modulation %s from initiator",
                  modulationToString(rec_mod));
    }

    if (pending_forced_code_rate_ != CodeRate::AUTO) {
        // Initiator forced a specific code rate - honor it
        rec_rate = pending_forced_code_rate_;
        LOG_MODEM(INFO, "Connection: Using FORCED code rate %s from initiator",
                  codeRateToString(rec_rate));
    }

    // Pick negotiated CW count (honor initiator's forced value, else auto).
    // Computed BEFORE building CONNECT_ACK so the embedded byte and the
    // initiator's view match what we'll actually use locally.
    int negotiated_cw = (pending_forced_cw_count_ != 0)
        ? v2::sanitizeFixedFrameCodewords(pending_forced_cw_count_)
        : connection_policy::recommendCWCount(rec_mod, rec_rate, negotiated_mode_);

    // Clear pending forced modes
    pending_forced_modulation_ = Modulation::AUTO;
    pending_forced_code_rate_ = CodeRate::AUTO;
    pending_forced_cw_count_ = 0;

    LadderRungId rung_id = LadderRungId::UNKNOWN;
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        rung_id = connection_policy::rungForMCDPSKConfig(
            rec_mod, config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol, negotiated_cw).id;
    } else if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        rung_id = LadderRungId::OFDM_CHIRP;
    } else if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        rung_id = LadderRungId::OFDM_NARROW;
    }

    // Set our local data mode immediately.
    applyDataMode(rec_mod, rec_rate, negotiated_cw, rung_id);

    LOG_MODEM(INFO, "Connection: Accepting call from %s (waveform=%s, data=%s %s, cw=%d)",
              remote_call_.c_str(), waveformModeToString(negotiated_mode_),
              modulationToString(data_modulation_), codeRateToString(data_code_rate_),
              data_frame_cw_count_);

    auto ack = v2::ConnectFrame::makeConnectAck(local_call_, remote_call_,
                                                 static_cast<uint8_t>(negotiated_mode_),
                                                 data_modulation_, data_code_rate_,
                                                 measured_snr_db_, fading_index_,
                                                 static_cast<uint8_t>(data_frame_cw_count_),
                                                 rung_id);
    Bytes ack_data = ack.serialize();

    LOG_MODEM(INFO, "Connection: Sending CONNECT_ACK (%zu bytes)", ack_data.size());
    transmitFrame(ack_data);

    enterConnected();

    // We are the responder - we received CONNECT and are sending CONNECT_ACK
    is_initiator_ = false;
    handshake_confirmed_ = false;  // Responder waits for first frame to confirm
    responder_handshake_wait_ms_ = RESPONDER_HANDSHAKE_FAILSAFE_MS;

    // Notify application of initial data mode
    notifyDataModeChanged(measured_snr_db_, fading_index_);
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

        auto disc = v2::ControlFrame::makeDisconnect(local_call_, remote_call_);
        disconnect_frame_ = disc.serialize();

        LOG_MODEM(INFO, "Connection: Sending DISCONNECT (%zu bytes)", disconnect_frame_.size());
        transmitFrame(disconnect_frame_);

        state_ = ConnectionState::DISCONNECTING;
        timeout_remaining_ms_ = config_.disconnect_timeout_ms;
        disconnect_retry_count_ = 0;
        disconnect_retransmit_ms_ = DISCONNECT_RETRANSMIT_INTERVAL_MS;
        stats_.disconnects++;
    }
}

void Connection::abortTxNow() {
    // Cancel all outbound ARQ activity (data retransmit timers, delayed ACK repeats,
    // delayed SACK, in-flight TX slots) while preserving RX reassembly state.
    arq_.abortPendingTx();

    // Cancel pending local TX assembly/burst state.
    bool had_pending_message = !pending_tx_fragments_.empty();
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    arq_callback_defer_refill_ = false;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;

    // Cancel file TX if active. Keep RX file state untouched.
    if (file_transfer_.getState() == FileTransferState::SENDING) {
        file_transfer_.cancel();
    }

    // Cancel pending control-path retries/timeouts.
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    disconnect_pending_ = false;
    disconnect_pending_ms_ = 0;
    disconnect_ack_retransmit_ms_ = 0;
    disconnect_ack_frame_.clear();
    disconnect_frame_.clear();
    disconnect_retry_count_ = 0;
    disconnect_retransmit_ms_ = 0;
    timeout_remaining_ms_ = 0;
    connect_retry_count_ = 0;
    ping_retry_count_ = 0;
    responder_handshake_wait_ms_ = 0;
    connect_ack_frame_.clear();
    connect_ack_retransmit_ms_ = 0;
    connect_ack_retransmit_interval_ms_ = CONNECT_ACK_RETRANSMIT_MS;
    connect_ack_retx_remaining_ = 0;
    resetAdaptiveModeController();

    // Stop transient connection attempts immediately.
    if (state_ == ConnectionState::PROBING ||
        state_ == ConnectionState::CONNECTING ||
        state_ == ConnectionState::DISCONNECTING) {
        enterDisconnected("TX aborted");
        return;
    }

    // Connected state remains established; only outbound transfer is aborted.
    if (state_ == ConnectionState::CONNECTED && had_pending_message && on_message_sent_) {
        on_message_sent_(false);
    }

    LOG_MODEM(INFO, "Connection: TX abort applied (state=%s)",
              connectionStateToString(state_));
}

void Connection::setForcedFrameCodewords(int cw_count, bool forced) {
    cw_count = v2::sanitizeFixedFrameCodewords(cw_count);
    if (forced) {
        // Operator override: initiator embeds in CONNECT.data_frame_cw_count
        // so responder honors and echoes via CONNECT_ACK. One-sided
        // propagation — caller only needs to set this on one peer.
        config_.forced_cw_count = static_cast<uint8_t>(cw_count);
    }
    // Note: !forced is the boot-time default path (host wiring up encoder/
    // decoder before connection). It MUST NOT touch config_.forced_cw_count
    // or every connect would advertise the default as a forced override
    // and bypass auto-pick on the responder.

    if (cw_count == data_frame_cw_count_) {
        return;
    }

    data_frame_cw_count_ = cw_count;
    config_.fixed_frame_codewords = cw_count;
    arq_.setFixedFrameCodewords(cw_count);

    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    if (isOFDMMode(negotiated_mode_) || bounded_variable_mc_dpsk) {
        file_transfer_.setMaxChunkPayload(currentDataPayloadCapacity());
        configureArqForCurrentDataMode();
    }

    LOG_MODEM(INFO, "Connection: Fixed data frame CW count set to %d", data_frame_cw_count_);
}

// =============================================================================
// DATA TRANSFER
// =============================================================================

bool Connection::sendMessage(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return sendPayload(data, false);
}

bool Connection::sendBinary(const Bytes& data) {
    return sendPayload(data, true);
}

bool Connection::sendPayload(const Bytes& data, bool binary_payload) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send, not connected");
        return false;
    }

    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    const size_t capacity = currentDataPayloadCapacity();

    if (!is_ofdm && !bounded_variable_mc_dpsk) {
        if (binary_payload) {
            return arq_.sendDataWithTypeAndFlags(data, v2::FrameType::DATA_END, v2::Flags::NONE);
        }
        return arq_.sendData(data);
    }

    if (capacity == 0) {
        LOG_MODEM(ERROR, "Connection: Data frame payload capacity is zero for current mode");
        return false;
    }

    if (data.size() <= capacity) {
        if (binary_payload) {
            return is_ofdm
                ? arq_.sendFixedDataWithTypeAndFlags(data, v2::FrameType::DATA_END, v2::Flags::NONE)
                : arq_.sendDataWithTypeAndFlags(data, v2::FrameType::DATA_END, v2::Flags::NONE);
        }
        return is_ofdm ? arq_.sendFixedDataWithFlags(data, v2::Flags::NONE)
                       : arq_.sendDataWithFlags(data, v2::Flags::NONE);
    }

    // Fragment the message into chunks that fit in one frame each
    LOG_MODEM(INFO, "Connection: Fragmenting %zu byte %s into %zu-byte chunks",
              data.size(), binary_payload ? "binary payload" : "message", capacity);

    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;

    for (size_t offset = 0; offset < data.size(); offset += capacity) {
        size_t chunk_size = std::min(capacity, data.size() - offset);
        pending_tx_fragments_.emplace_back(data.begin() + offset, data.begin() + offset + chunk_size);
        if (binary_payload) {
            const bool first = (offset == 0);
            const bool last = (offset + chunk_size >= data.size());
            pending_tx_fragment_types_.push_back(
                first ? v2::FrameType::DATA_START :
                (last ? v2::FrameType::DATA_END : v2::FrameType::DATA_CONT));
        }
    }

    LOG_MODEM(INFO, "Connection: Split into %zu fragments", pending_tx_fragments_.size());

    sendNextFragment();
    return true;
}

bool Connection::sendMessages(const std::vector<std::string>& texts) {
    if (state_ != ConnectionState::CONNECTED) {
        LOG_MODEM(WARN, "Connection: Cannot send, not connected");
        return false;
    }

    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    size_t capacity = (is_ofdm || bounded_variable_mc_dpsk) ? currentDataPayloadCapacity() : SIZE_MAX;

    // Pre-fragment all messages into a flat list of frame payloads with flags
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;

    for (const auto& text : texts) {
        Bytes data(text.begin(), text.end());

        if (data.size() <= capacity) {
            // Single frame — no MORE_FRAG
            pending_tx_fragments_.push_back(data);
            pending_tx_fragment_flags_.push_back(v2::Flags::NONE);
        } else {
            // Fragment this message
            for (size_t offset = 0; offset < data.size(); offset += capacity) {
                size_t chunk_size = std::min(capacity, data.size() - offset);
                Bytes chunk(data.begin() + offset, data.begin() + offset + chunk_size);
                bool is_last = (offset + chunk_size >= data.size());
                pending_tx_fragments_.push_back(chunk);
                pending_tx_fragment_flags_.push_back(
                    is_last ? v2::Flags::NONE : v2::Flags::MORE_FRAG);
            }
        }
    }

    LOG_MODEM(INFO, "Connection: Batch queued %zu messages as %zu frames",
              texts.size(), pending_tx_fragments_.size());

    // Send first window-worth via sendNextFragment() (handles burst buffering)
    sendNextFragment();
    return !pending_tx_fragments_.empty();
}

bool Connection::isReadyToSend() const {
    return state_ == ConnectionState::CONNECTED && arq_.isReadyToSend() &&
           !file_transfer_.isBusy();
}

size_t Connection::getTxBacklogBytes() const {
    size_t bytes = arq_.getTxInFlightBytes();

    if (next_fragment_idx_ < pending_tx_fragments_.size()) {
        for (size_t i = next_fragment_idx_; i < pending_tx_fragments_.size(); ++i) {
            bytes += pending_tx_fragments_[i].size();
        }
    }

    if (file_transfer_.getState() == FileTransferState::SENDING) {
        bytes += file_transfer_.remainingTxBytes();
    }

    return bytes;
}

// =============================================================================
// FILE TRANSFER
// =============================================================================

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

    // Set chunk size to match frame capacity for bounded frame geometries.
    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool bounded_variable_mc_dpsk = usesBoundedVariableMCDPSKFrames();
    if (is_ofdm || bounded_variable_mc_dpsk) {
        size_t capacity = currentDataPayloadCapacity();
        if (capacity <= FileTransferController::FILE_DATA_OVERHEAD) {
            LOG_MODEM(ERROR,
                      "Connection: File chunk payload capacity %zu is too small for FILE_DATA overhead",
                      capacity);
            return false;
        }
        file_transfer_.setMaxChunkPayload(capacity);
        LOG_MODEM(INFO, "Connection: File chunk payload limited to %zu bytes (%s %s, cw=%d)",
                  capacity,
                  is_ofdm ? "OFDM fixed-frame" : "MC-DPSK variable-frame",
                  codeRateToString(data_code_rate_),
                  data_frame_cw_count_);
    }

    LOG_MODEM(INFO, "Connection: Starting file transfer: %s", filepath.c_str());

    if (!file_transfer_.startSend(filepath)) {
        LOG_MODEM(ERROR, "Connection: Failed to start file transfer");
        return false;
    }

    if (is_ofdm && shouldUseSingleOFDMFileBlock(fading_index_, measured_snr_db_, data_code_rate_)) {
        Bytes block = file_transfer_.getSingleBlockPayload(kOFDMFileBlockPayloadLimit);
        if (!block.empty()) {
            LOG_MODEM(INFO, "Connection: Sending file as single OFDM block (%zu bytes payload)",
                      block.size());
            if (!arq_.sendVariableDataWithFlags(block, v2::Flags::NONE)) {
                file_transfer_.onSendFailed();
                return false;
            }
            return true;
        }
    } else if (is_ofdm) {
        LOG_MODEM(INFO, "Connection: Using interleaved OFDM chunks for file (SNR=%.1f, fading=%.2f, rate=%s)",
                  measured_snr_db_, fading_index_, codeRateToString(data_code_rate_));
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

    bool is_ofdm = isOFDMMode(negotiated_mode_);

    const bool is_mc_dpsk = negotiated_mode_ == WaveformMode::MC_DPSK;

    // Enable burst buffering for OFDM and MC-DPSK data-window mode.
    if ((is_ofdm || is_mc_dpsk) && on_transmit_burst_) {
        burst_mode_active_ = true;
        burst_tx_buffer_.clear();
    }

    // Fill the ARQ window with as many chunks as possible
    // Selective Repeat ARQ can have multiple frames in flight
    while (arq_.isReadyToSend() && file_transfer_.hasMoreChunks()) {
        Bytes chunk = file_transfer_.getNextChunk();
        if (chunk.empty()) {
            break;
        }

        // MORE_FRAG indicates more data remaining in file (not burst)
        uint8_t flags = file_transfer_.hasMoreChunks() ? v2::Flags::MORE_FRAG : v2::Flags::NONE;
        if (is_ofdm) {
            arq_.sendFixedDataWithFlags(chunk, flags);
        } else {
            arq_.sendDataWithFlags(chunk, flags);
        }
    }

    // Flush burst buffer
    if ((is_ofdm || is_mc_dpsk) && on_transmit_burst_) {
        burst_mode_active_ = false;
        flushBurstBuffer();
    }
}

void Connection::sendNextFragment() {
    bool is_ofdm = isOFDMMode(negotiated_mode_);
    const bool pipeline_fragments = is_ofdm;

    // Enable burst buffering for OFDM mode
    if (is_ofdm && on_transmit_burst_) {
        burst_mode_active_ = true;
        burst_tx_buffer_.clear();
    }

    size_t submitted_this_call = 0;
    while (arq_.isReadyToSend() && next_fragment_idx_ < pending_tx_fragments_.size()) {
        const Bytes& chunk = pending_tx_fragments_[next_fragment_idx_];

        // Use pre-computed flags if available (from sendMessages batch),
        // otherwise derive from position (single-message fragmentation)
        uint8_t flags;
        if (next_fragment_idx_ < pending_tx_fragment_flags_.size()) {
            flags = pending_tx_fragment_flags_[next_fragment_idx_];
        } else {
            bool is_last = (next_fragment_idx_ + 1 == pending_tx_fragments_.size());
            flags = is_last ? v2::Flags::NONE : v2::Flags::MORE_FRAG;
        }
        v2::FrameType frame_type = v2::FrameType::DATA;
        if (next_fragment_idx_ < pending_tx_fragment_types_.size()) {
            frame_type = pending_tx_fragment_types_[next_fragment_idx_];
        }

        LOG_MODEM(DEBUG, "Connection: Sending fragment %zu/%zu (%zu bytes, type=%s, flags=0x%02X)",
                  next_fragment_idx_ + 1, pending_tx_fragments_.size(), chunk.size(),
                  v2::frameTypeToString(frame_type), flags);

        if (is_ofdm) {
            arq_.sendFixedDataWithTypeAndFlags(chunk, frame_type, flags);
        } else {
            arq_.sendDataWithTypeAndFlags(chunk, frame_type, flags);
        }
        next_fragment_idx_++;
        submitted_this_call++;

        // MC-DPSK bulk-file transfer uses sendNextFileChunk() and is safe to
        // pipeline. The message-fragment path still shares OFDM-oriented
        // MORE_FRAG/SACK semantics, so keep it stop-and-wait until it has its
        // own hardware-validated burst pacing.
        if (!pipeline_fragments && submitted_this_call >= 1) {
            break;
        }
    }

    // Flush burst buffer
    if (is_ofdm && on_transmit_burst_) {
        burst_mode_active_ = false;
        flushBurstBuffer();
    }
}

// =============================================================================
// FRAME DISPATCHING
// =============================================================================

void Connection::onFrameReceived(const Bytes& frame_data) {
    if (frame_data.size() < 2) {
        return;
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

    // Any frame from the initiator means our CONNECT_ACK got through — stop
    // proactive ACK retx regardless of whether the formal handshake-confirmed
    // bit has flipped (the 2.2s fail-safe sets that bit before first DATA
    // arrives, so without this clear, retx fires uselessly during early data
    // phase and clogs the channel).
    if (state_ == ConnectionState::CONNECTED && !is_initiator_ &&
        !connect_ack_frame_.empty()) {
        connect_ack_frame_.clear();
        connect_ack_retx_remaining_ = 0;
    }

    // Responder handshake confirmation: first valid protocol frame after CONNECT_ACK
    // means the initiator received our ACK and switched to data/control exchange.
    if (state_ == ConnectionState::CONNECTED && !is_initiator_ && !handshake_confirmed_) {
        LOG_MODEM(INFO, "Connection: Handshake confirmed (received first valid frame from initiator)");
        handshake_confirmed_ = true;
        responder_handshake_wait_ms_ = 0;
        if (on_handshake_confirmed_) {
            on_handshake_confirmed_();
        }
        // Initial data mode is already carried in CONNECT_ACK.
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

    // DISCONNECT now uses control-frame encoding (20 bytes) for hardened
    // 1-CW handling. Keep ConnectFrame fallback for legacy peers/log replay.
    if (header.type == v2::FrameType::DISCONNECT) {
        if (auto ctrl = v2::ControlFrame::deserialize(frame_data)) {
            handleDisconnect(*ctrl, src_call);
            return;
        }

        if (auto conn = v2::ConnectFrame::deserialize(frame_data)) {
            std::string frame_src_call = conn->getSrcCallsign();
            if (!frame_src_call.empty()) {
                src_call = frame_src_call;
            }
            handleDisconnectFrame(*conn, src_call);
            return;
        }

        LOG_MODEM(WARN, "Connection: Failed to parse DISCONNECT frame");
        return;
    }

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
                default:
                    break;
            }
        }
    } else if (v2::isControlFrame(header.type)) {
        // Control frames (DISCONNECT, ACK, NACK, etc) - parse as ControlFrame
        auto ctrl = v2::ControlFrame::deserialize(frame_data);
        if (ctrl) {
            switch (ctrl->type) {
                case v2::FrameType::ACK:
                    if (state_ == ConnectionState::DISCONNECTING) {
                        if (ctrl->seq == v2::DISCONNECT_SEQ) {
                            LOG_MODEM(INFO, "Connection: Disconnect acknowledged (seq=0x%04X)", ctrl->seq);
                            enterDisconnected("Disconnect complete");
                        } else {
                            LOG_MODEM(DEBUG, "Connection: Ignoring stale data ACK seq=%d while disconnecting", ctrl->seq);
                        }
                    } else if (state_ == ConnectionState::CONNECTED) {
                        // Check if this ACK is for our pending MODE_CHANGE
                        if (mode_change_pending_ && ctrl->seq == mode_change_seq_) {
                            const bool was_downgrade =
                                isMoreRobustRate(pending_code_rate_, data_code_rate_);
                            LOG_MODEM(INFO, "Connection: MODE_CHANGE acknowledged, applying %s %s",
                                      modulationToString(pending_modulation_),
                                      codeRateToString(pending_code_rate_));
                            applyDataMode(pending_modulation_, pending_code_rate_,
                                          pending_cw_count_, pending_ladder_rung_id_);
                            if (was_downgrade) {
                                adaptive_post_downgrade_lockout_ms_ =
                                    ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS;
                                adaptive_clean_windows_ = 0;
                                adaptive_pressure_windows_ = 0;
                            }
                            mode_change_pending_ = false;
                            pending_ladder_rung_id_ = LadderRungId::UNKNOWN;

                            // Notify application of mode change
                            notifyDataModeChanged(pending_snr_db_, pending_fading_index_);
                            runDeferredArqRefill();
                        } else {
                            // Regular data ACK
                            processArqFrame(frame_data);
                        }
                    }
                    break;
                case v2::FrameType::NACK:
                    if (state_ == ConnectionState::CONNECTED) {
                        processArqFrame(frame_data);
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
            if (v2::isDataFrame(header.type) && config_.ack_tx_delay_ms > 0) {
                ack_hold_remaining_ms_ = config_.ack_tx_delay_ms;
                LOG_MODEM(INFO, "Connection: Received DATA, holding outbound ACK for %u ms (peer PTT-off settling)",
                          config_.ack_tx_delay_ms);
            }
            processArqFrame(frame_data);
        }
    }
}

void Connection::processArqFrame(const Bytes& frame_data) {
    const bool outermost = !arq_callback_defer_refill_;
    if (outermost) {
        arq_callback_defer_refill_ = true;
    }

    arq_.onFrameReceived(frame_data);

    if (outermost) {
        arq_callback_defer_refill_ = false;
        runDeferredArqRefill();
    }
}

void Connection::onMCDPSKPartialFrame(const v2::PartialFrameCodewords& partial) {
    if (state_ != ConnectionState::CONNECTED || negotiated_mode_ != WaveformMode::MC_DPSK) {
        return;
    }
    if (!partial.valid()) {
        return;
    }
    const uint32_t our_hash = v2::hashCallsign(local_call_);
    if (partial.dst_hash != our_hash && partial.dst_hash != 0xFFFFFF) {
        return;
    }

    const bool outermost = !arq_callback_defer_refill_;
    if (outermost) {
        arq_callback_defer_refill_ = true;
    }

    arq_.onPartialFrame(partial);

    if (outermost) {
        arq_callback_defer_refill_ = false;
        runDeferredArqRefill();
    }
}

void Connection::onAcceptedOFDMDataSync(float sync_correlation) {
    if (state_ != ConnectionState::CONNECTED || is_initiator_ ||
        !isOFDMMode(negotiated_mode_)) {
        return;
    }
    if (connect_ack_frame_.empty() && connect_ack_retx_remaining_ <= 0) {
        return;
    }

    connect_ack_frame_.clear();
    connect_ack_retx_remaining_ = 0;
    connect_ack_retransmit_ms_ = 0;
    LOG_MODEM(INFO,
              "Connection: Accepted OFDM DATA sync (corr=%.2f); clearing cached CONNECT_ACK rescue retry",
              sync_correlation);
}

void Connection::runDeferredArqRefill() {
    if (arq_callback_defer_refill_) {
        return;
    }

    if (mode_change_pending_) {
        return;
    }

    if (tryIssueAdaptiveModeChangeAtBoundary()) {
        return;
    }

    const bool refill_file = deferred_file_refill_;
    const bool refill_fragments = deferred_fragment_refill_;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;

    if (state_ != ConnectionState::CONNECTED) {
        return;
    }

    if (refill_file && file_transfer_.getState() == FileTransferState::SENDING) {
        sendNextFileChunk();
    }

    if (refill_fragments &&
        !pending_tx_fragments_.empty() &&
        next_fragment_idx_ < pending_tx_fragments_.size()) {
        sendNextFragment();
    }
}

void Connection::resetAdaptiveModeController() {
    adaptive_target_ = AdaptiveModeTarget{};
    adaptive_last_stats_ = arq_.getStats();
    adaptive_eval_elapsed_ms_ = 0;
    adaptive_cooldown_ms_ = ADAPTIVE_MODE_CHANGE_COOLDOWN_MS;
    adaptive_post_downgrade_lockout_ms_ = 0;
    adaptive_downgrade_queue_age_ms_ = 0;
    adaptive_clean_windows_ = 0;
    adaptive_pressure_windows_ = 0;
}

bool Connection::canIssueAdaptiveModeChange(bool is_downgrade) const {
    if (state_ != ConnectionState::CONNECTED || !isOFDMMode(negotiated_mode_)) {
        return false;
    }
    if (config_.forced_modulation != Modulation::AUTO ||
        config_.forced_code_rate != CodeRate::AUTO) {
        return false;
    }
    if (mode_change_pending_ || disconnect_pending_) {
        return false;
    }
    if (file_transfer_.getState() != FileTransferState::SENDING) {
        return false;
    }
    if (!is_initiator_ && !handshake_confirmed_) {
        return false;
    }
    if (!pending_tx_fragments_.empty()) {
        return false;
    }

    const size_t available_slots = arq_.getAvailableSlots();
    const size_t window_size = arq_.getWindowSize();
    if (is_downgrade) {
        // Downgrades are recovery-oriented. Waiting for a fully drained window
        // can keep us transmitting into the rate that is already failing, so
        // allow the MODE_CHANGE once at least half of the ARQ window is free.
        return available_slots * 2 >= window_size;
    }

    // Upgrades keep the strict boundary so in-flight DATA at the old, more
    // robust rate clears before switching to a faster rate.
    return available_slots == window_size;
}

size_t Connection::adaptiveBacklogFrames(CodeRate rate) const {
    size_t frames = 0;
    const size_t capacity = v2::getFixedFramePayloadCapacity(rate, data_frame_cw_count_);
    const size_t file_data_payload =
        capacity > FileTransferController::FILE_DATA_OVERHEAD
            ? capacity - FileTransferController::FILE_DATA_OVERHEAD
            : capacity;

    if (file_transfer_.getState() == FileTransferState::SENDING) {
        frames += file_transfer_.pendingChunkCount();

        const size_t remaining = file_transfer_.remainingTxBytes();
        if (remaining > 0 && file_data_payload > 0) {
            frames += (remaining + file_data_payload - 1) / file_data_payload;
        } else if (file_transfer_.hasMoreChunks()) {
            frames += 1;
        }
    }

    if (!pending_tx_fragments_.empty() &&
        acked_fragment_count_ < pending_tx_fragments_.size()) {
        frames += pending_tx_fragments_.size() - acked_fragment_count_;
    }

    return frames;
}

bool Connection::hasAdaptiveUpgradeBacklog(CodeRate target_rate) const {
    if (file_transfer_.getState() != FileTransferState::SENDING) {
        return false;
    }
    return adaptiveBacklogFrames(target_rate) >= arq_.getWindowSize();
}

bool Connection::tryIssueAdaptiveModeChangeAtBoundary() {
    if (!adaptive_target_.pending) {
        return false;
    }

    const bool is_downgrade =
        isMoreRobustRate(adaptive_target_.rate, data_code_rate_);
    const bool downgrade_stuck =
        is_downgrade &&
        adaptive_downgrade_queue_age_ms_ >= ADAPTIVE_DOWNGRADE_FORCE_MS;
    const bool boundary_ready = canIssueAdaptiveModeChange(is_downgrade);
    const bool force_downgrade =
        downgrade_stuck &&
        !boundary_ready &&
        state_ == ConnectionState::CONNECTED &&
        isOFDMMode(negotiated_mode_) &&
        config_.forced_modulation == Modulation::AUTO &&
        config_.forced_code_rate == CodeRate::AUTO &&
        !mode_change_pending_ &&
        !disconnect_pending_ &&
        file_transfer_.getState() == FileTransferState::SENDING &&
        (is_initiator_ || handshake_confirmed_) &&
        pending_tx_fragments_.empty();
    if (!boundary_ready && !force_downgrade) {
        return false;
    }

    const size_t backlog_frames = adaptiveBacklogFrames(adaptive_target_.rate);
    if (backlog_frames == 0) {
        adaptive_target_ = AdaptiveModeTarget{};
        adaptive_downgrade_queue_age_ms_ = 0;
        return false;
    }

    const bool is_upgrade = isFasterRate(adaptive_target_.rate, data_code_rate_);
    if (is_upgrade && !hasAdaptiveUpgradeBacklog(adaptive_target_.rate)) {
        adaptive_target_ = AdaptiveModeTarget{};
        adaptive_downgrade_queue_age_ms_ = 0;
        return false;
    }

    if (adaptive_target_.modulation == data_modulation_ &&
        adaptive_target_.rate == data_code_rate_) {
        adaptive_target_ = AdaptiveModeTarget{};
        adaptive_downgrade_queue_age_ms_ = 0;
        return false;
    }

    if (force_downgrade) {
        LOG_MODEM(WARN,
                  "Connection: Forced downgrade after %ums queue age (window state ignored): %s -> %s",
                  adaptive_downgrade_queue_age_ms_,
                  codeRateToString(data_code_rate_),
                  codeRateToString(adaptive_target_.rate));
    }

    LOG_MODEM(INFO, "Connection: Adaptive MODE_CHANGE at TX boundary: %s %s -> %s %s (SNR=%.1f, fading=%.2f, backlog=%zu frames)",
              modulationToString(data_modulation_), codeRateToString(data_code_rate_),
              modulationToString(adaptive_target_.modulation),
              codeRateToString(adaptive_target_.rate),
              measured_snr_db_, fading_index_,
              backlog_frames);

    requestModeChange(adaptive_target_.modulation,
                      adaptive_target_.rate,
                      measured_snr_db_,
                      adaptive_target_.reason);
    adaptive_target_ = AdaptiveModeTarget{};
    adaptive_downgrade_queue_age_ms_ = 0;
    adaptive_cooldown_ms_ = ADAPTIVE_MODE_CHANGE_COOLDOWN_MS;
    adaptive_clean_windows_ = 0;
    adaptive_pressure_windows_ = 0;
    if (is_downgrade && mode_change_pending_) {
        adaptive_post_downgrade_lockout_ms_ = ADAPTIVE_POST_DOWNGRADE_LOCKOUT_MS;
    }
    return mode_change_pending_;
}

void Connection::updateAdaptiveModeController(uint32_t elapsed_ms) {
    if (adaptive_cooldown_ms_ > 0) {
        if (elapsed_ms >= adaptive_cooldown_ms_) {
            adaptive_cooldown_ms_ = 0;
        } else {
            adaptive_cooldown_ms_ -= elapsed_ms;
        }
    }
    if (adaptive_post_downgrade_lockout_ms_ > 0) {
        if (elapsed_ms >= adaptive_post_downgrade_lockout_ms_) {
            adaptive_post_downgrade_lockout_ms_ = 0;
        } else {
            adaptive_post_downgrade_lockout_ms_ -= elapsed_ms;
        }
    }

    if (state_ != ConnectionState::CONNECTED || !isOFDMMode(negotiated_mode_)) {
        resetAdaptiveModeController();
        return;
    }
    if (config_.forced_modulation != Modulation::AUTO ||
        config_.forced_code_rate != CodeRate::AUTO) {
        adaptive_last_stats_ = arq_.getStats();
        adaptive_target_ = AdaptiveModeTarget{};
        adaptive_post_downgrade_lockout_ms_ = 0;
        adaptive_downgrade_queue_age_ms_ = 0;
        adaptive_clean_windows_ = 0;
        adaptive_pressure_windows_ = 0;
        return;
    }
    if (file_transfer_.getState() != FileTransferState::SENDING) {
        adaptive_last_stats_ = arq_.getStats();
        adaptive_target_ = AdaptiveModeTarget{};
        adaptive_post_downgrade_lockout_ms_ = 0;
        adaptive_downgrade_queue_age_ms_ = 0;
        adaptive_clean_windows_ = 0;
        adaptive_pressure_windows_ = 0;
        return;
    }

    if (adaptive_target_.pending &&
        isMoreRobustRate(adaptive_target_.rate, data_code_rate_)) {
        adaptive_downgrade_queue_age_ms_ += elapsed_ms;
        if (adaptive_downgrade_queue_age_ms_ >= ADAPTIVE_DOWNGRADE_FORCE_MS &&
            tryIssueAdaptiveModeChangeAtBoundary()) {
            return;
        }
    } else {
        adaptive_downgrade_queue_age_ms_ = 0;
    }

    adaptive_eval_elapsed_ms_ += elapsed_ms;
    if (adaptive_eval_elapsed_ms_ < ADAPTIVE_EVAL_INTERVAL_MS) {
        return;
    }
    adaptive_eval_elapsed_ms_ = 0;

    const ARQStats current_stats = arq_.getStats();
    const bool retry_pressure =
        hasAdaptiveRetryPressure(current_stats, adaptive_last_stats_);
    const bool clean_window =
        hasCleanAdaptiveWindow(current_stats, adaptive_last_stats_);
    adaptive_last_stats_ = current_stats;
    if (retry_pressure) {
        adaptive_pressure_windows_++;
    } else {
        adaptive_pressure_windows_ = 0;
    }

    Modulation recommended_mod = data_modulation_;
    CodeRate recommended_rate = data_code_rate_;
    recommendDataMode(measured_snr_db_, negotiated_mode_,
                      recommended_mod, recommended_rate, fading_index_);

    if (retry_pressure &&
        adaptive_pressure_windows_ >= ADAPTIVE_PRESSURE_WINDOWS_FOR_DOWNGRADE &&
        isFasterRate(data_code_rate_, CodeRate::R1_4)) {
        adaptive_clean_windows_ = 0;
        adaptive_target_.pending = true;
        adaptive_target_.modulation = recommended_mod;
        adaptive_target_.rate = adaptiveDowngradeTarget(data_code_rate_, recommended_rate);
        adaptive_target_.reason = v2::ModeChangeReason::CHANNEL_DEGRADED;
        LOG_MODEM(INFO, "Connection: Adaptive downgrade queued: %s -> %s (recommended=%s, SNR=%.1f, fading=%.2f)",
                  codeRateToString(data_code_rate_),
                  codeRateToString(adaptive_target_.rate),
                  codeRateToString(recommended_rate),
                  measured_snr_db_, fading_index_);
        tryIssueAdaptiveModeChangeAtBoundary();
        return;
    }

    if (clean_window) {
        adaptive_clean_windows_++;
    } else {
        adaptive_clean_windows_ = 0;
    }

    if (adaptive_cooldown_ms_ == 0 &&
        adaptive_post_downgrade_lockout_ms_ == 0 &&
        !adaptive_target_.pending &&
        adaptive_clean_windows_ >= ADAPTIVE_CLEAN_WINDOWS_FOR_UPGRADE &&
        isFasterRate(recommended_rate, data_code_rate_) &&
        hasAdaptiveUpgradeBacklog(recommended_rate)) {
        adaptive_target_.pending = true;
        adaptive_target_.modulation = recommended_mod;
        adaptive_target_.rate = recommended_rate;
        adaptive_target_.reason = v2::ModeChangeReason::CHANNEL_IMPROVED;
        LOG_MODEM(INFO, "Connection: Adaptive upgrade queued: %s -> %s (SNR=%.1f, fading=%.2f, clean=%d, backlog=%zu frames)",
                  codeRateToString(data_code_rate_),
                  codeRateToString(adaptive_target_.rate),
                  measured_snr_db_, fading_index_,
                  adaptive_clean_windows_,
                  adaptiveBacklogFrames(recommended_rate));
        tryIssueAdaptiveModeChangeAtBoundary();
    }
}

// =============================================================================
// TIMER / TICK
// =============================================================================

void Connection::tick(uint32_t elapsed_ms) {
    soft_combine_harq_.tick(elapsed_ms);

    if (pending_pong_callback_) {
        if (elapsed_ms >= pong_callback_delay_remaining_ms_) {
            pending_pong_callback_ = false;
            pong_callback_delay_remaining_ms_ = 0;
            if (state_ == ConnectionState::DISCONNECTED && on_ping_received_) {
                LOG_MODEM(INFO, "Connection: Firing deferred PONG TX callback");
                on_ping_received_();
            }
        } else {
            pong_callback_delay_remaining_ms_ -= elapsed_ms;
        }
    }

    if (post_connect_data_hold_active_) {
        if (elapsed_ms >= post_connect_data_hold_remaining_ms_) {
            post_connect_data_hold_active_ = false;
            post_connect_data_hold_remaining_ms_ = 0;
            LOG_MODEM(INFO, "Connection: post-CONNECT data hold expired, releasing %zu held DATA frame(s)",
                      held_data_frames_.size());
            auto released = std::move(held_data_frames_);
            held_data_frames_.clear();
            for (auto& frame : released) {
                transmitFrame(frame);
            }
        } else {
            post_connect_data_hold_remaining_ms_ -= elapsed_ms;
        }
    }

    if (ack_hold_remaining_ms_ > 0) {
        if (elapsed_ms >= ack_hold_remaining_ms_) {
            ack_hold_remaining_ms_ = 0;
            LOG_MODEM(INFO, "Connection: ACK TX hold expired, releasing %zu held ACK frame(s)",
                      held_ack_frames_.size());
            auto released = std::move(held_ack_frames_);
            held_ack_frames_.clear();
            for (auto& frame : released) {
                transmitFrame(frame);
            }
        } else {
            ack_hold_remaining_ms_ -= elapsed_ms;
        }
    }

    switch (state_) {
        case ConnectionState::PROBING:
            // Fast presence check via PING/PONG
            if (elapsed_ms >= timeout_remaining_ms_) {
                ping_retry_count_++;
                if (ping_retry_count_ >= MAX_PING_RETRIES) {
                    // No response after all PINGs - give up
                    LOG_MODEM(INFO, "Connection: No response after %d PINGs, giving up",
                              MAX_PING_RETRIES);
                    stats_.connects_failed++;
                    enterDisconnected("No response");
                } else {
                    LOG_MODEM(INFO, "Connection: PING timeout, retrying (%d/%d)",
                              ping_retry_count_, MAX_PING_RETRIES);
                    if (on_ping_tx_) {
                        on_ping_tx_();
                    }
                    timeout_remaining_ms_ = pingTimeoutMsForCurrentProfile();
                }
            } else {
                timeout_remaining_ms_ -= elapsed_ms;
            }
            break;

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
                    LOG_MODEM(WARN, "Connection: Connect timeout, retrying via %s (%d/%d)",
                              waveformModeToString(connect_waveform_),
                              connect_retry_count_ + 1, config_.connect_retries);
                    auto connect_frame = v2::ConnectFrame::makeConnect(local_call_, remote_call_,
                                                                        config_.mode_capabilities,
                                                                        static_cast<uint8_t>(config_.preferred_mode),
                                                                        static_cast<uint8_t>(config_.forced_modulation),
                                                                        static_cast<uint8_t>(config_.forced_code_rate),
                                                                        config_.forced_cw_count);
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

            // Proactive CONNECT_ACK retransmission (responder side, BUG-CTRL-001).
            // ALPHA can miss the single MC-DPSK ACK on faded seeds — without retx
            // the only recovery is the 60s connect_timeout, which is too slow.
            // Gated to OFDM_CHIRP data mode: when MC-DPSK is the negotiated data
            // mode, the round-trip is ~12-16s and retx clogs ALPHA's RX buffer
            // ahead of the first ACK, hurting more than it helps. The interval is
            // set long enough for the first OFDM burst-interleaver group to decode
            // and clear this cached ACK on the success path.
            if (!is_initiator_ &&
                negotiated_mode_ == WaveformMode::OFDM_CHIRP &&
                !connect_ack_frame_.empty() && connect_ack_retx_remaining_ > 0) {
                if (elapsed_ms >= connect_ack_retransmit_ms_) {
                    connect_ack_retransmit_ms_ = connect_ack_retransmit_interval_ms_;
                    connect_ack_retx_remaining_--;
                    LOG_MODEM(INFO, "Connection: Re-sending CONNECT_ACK (proactive, %d retx remaining)",
                              connect_ack_retx_remaining_);
                    transmitFrame(connect_ack_frame_);
                } else {
                    connect_ack_retransmit_ms_ -= elapsed_ms;
                }
            }

            // Responder fail-safe: if first post-ACK frame is lost, don't stay in
            // handshake waveform forever. After a short grace period, force
            // handshake completion so TX uses negotiated waveform/control path.
            if (!is_initiator_ && !handshake_confirmed_) {
                if (elapsed_ms >= responder_handshake_wait_ms_) {
                    responder_handshake_wait_ms_ = 0;
                    handshake_confirmed_ = true;
                    LOG_MODEM(WARN, "Connection: Handshake fail-safe triggered (no post-ACK frame), switching to negotiated waveform");
                    if (on_handshake_confirmed_) {
                        on_handshake_confirmed_();
                    }
                } else {
                    responder_handshake_wait_ms_ -= elapsed_ms;
                }
            }

            // Handle MODE_CHANGE timeout
            if (mode_change_pending_) {
                if (elapsed_ms >= mode_change_timeout_ms_) {
                    mode_change_retry_count_++;
                    if (mode_change_retry_count_ > MODE_CHANGE_MAX_RETRIES) {
                        LOG_MODEM(WARN, "Connection: MODE_CHANGE failed after %d attempts, keeping current mode",
                                  MODE_CHANGE_MAX_RETRIES);
                        mode_change_pending_ = false;
                        pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
                        resetAdaptiveModeController();
                        runDeferredArqRefill();
                        // Stay at current mode - don't change anything
                    } else {
                        LOG_MODEM(WARN, "Connection: MODE_CHANGE timeout, retrying (%d/%d)",
                                  mode_change_retry_count_, MODE_CHANGE_MAX_RETRIES);
                        // Resend MODE_CHANGE with same parameters (incl. CW)
                        auto frame = v2::ControlFrame::makeModeChange(local_call_, remote_call_,
                                                                       mode_change_seq_, pending_modulation_,
                                                                       pending_code_rate_, pending_snr_db_,
                                                                       pending_fading_index_,
                                                                       pending_reason_,
                                                                       pending_cw_count_,
                                                                       pending_ladder_rung_id_);
                        transmitFrame(frame.serialize());
                        mode_change_timeout_ms_ = MODE_CHANGE_TIMEOUT_MS;
                    }
                } else {
                    mode_change_timeout_ms_ -= elapsed_ms;
                }
            }

            // Disconnect grace period (responder side): stay connected and
            // proactively re-send ACK until initiator confirms (goes silent)
            if (disconnect_pending_) {
                if (elapsed_ms >= disconnect_pending_ms_) {
                    disconnect_pending_ = false;
                    disconnect_ack_frame_.clear();
                    enterDisconnected("Remote disconnected");
                    break;
                }
                disconnect_pending_ms_ -= elapsed_ms;

                // Proactively re-send ACK periodically (fading may have lost it)
                if (!disconnect_ack_frame_.empty()) {
                    if (elapsed_ms >= disconnect_ack_retransmit_ms_) {
                        disconnect_ack_retransmit_ms_ = DISCONNECT_ACK_RETRANSMIT_MS;
                        LOG_MODEM(INFO, "Connection: Re-sending disconnect ACK (proactive, %dms remaining)",
                                  disconnect_pending_ms_);
                        transmitFrame(disconnect_ack_frame_);
                    } else {
                        disconnect_ack_retransmit_ms_ -= elapsed_ms;
                    }
                }
            }

            arq_.tick(elapsed_ms);
            updateAdaptiveModeController(elapsed_ms);
            break;

        case ConnectionState::DISCONNECTING:
            if (elapsed_ms >= timeout_remaining_ms_) {
                LOG_MODEM(INFO, "Connection: Disconnect timeout, forcing disconnect");
                enterDisconnected("Disconnect timeout");
            } else {
                timeout_remaining_ms_ -= elapsed_ms;

                // Retransmit DISCONNECT periodically (fading can lose the frame)
                if (elapsed_ms >= disconnect_retransmit_ms_) {
                    disconnect_retransmit_ms_ = DISCONNECT_RETRANSMIT_INTERVAL_MS;
                    if (disconnect_retry_count_ < DISCONNECT_MAX_RETRIES && !disconnect_frame_.empty()) {
                        disconnect_retry_count_++;
                        LOG_MODEM(INFO, "Connection: Retransmitting DISCONNECT (%d/%d)",
                                  disconnect_retry_count_, DISCONNECT_MAX_RETRIES);
                        transmitFrame(disconnect_frame_);
                    }
                } else {
                    disconnect_retransmit_ms_ -= elapsed_ms;
                }
            }
            break;

        default:
            break;
    }
}

// =============================================================================
// STATE TRANSITIONS
// =============================================================================

void Connection::transmitFrame(const Bytes& frame_data) {
    LOG_MODEM(DEBUG, "Connection: TX %zu bytes", frame_data.size());

    // Post-CONNECT data hold: defer DATA-class frames so peer's PTT-off
    // transition has time to complete. ACK/control frames are NOT held
    // because the peer was in RX when it sent CONNECT_ACK and stays in
    // RX while we ACK; only our outbound DATA hits their TRANSITION.
    if (post_connect_data_hold_active_) {
        const auto header = v2::parseHeader(frame_data);
        if (header.valid && v2::isDataFrame(header.type)) {
            held_data_frames_.push_back(frame_data);
            return;
        }
    }

    // ACK TX hold: after inbound DATA, defer ACK-class responses until the
    // sender has completed its PTT-off transition from DATA TX.
    if (ack_hold_remaining_ms_ > 0) {
        const auto header = v2::parseHeader(frame_data);
        if (header.valid && header.type == v2::FrameType::ACK) {
            held_ack_frames_.push_back(frame_data);
            return;
        }
    }

    // If burst mode is active, buffer instead of transmitting immediately
    if (burst_mode_active_ && on_transmit_burst_) {
        burst_tx_buffer_.push_back(frame_data);
        return;
    }

    if (on_transmit_) {
        on_transmit_(frame_data);
    }
}

void Connection::cancelPendingPongCallback() {
    if (pending_pong_callback_) {
        LOG_MODEM(DEBUG, "Connection: Cancelling deferred PONG TX callback");
    }
    pending_pong_callback_ = false;
    pong_callback_delay_remaining_ms_ = 0;
}

void Connection::configureArqForCurrentDataMode() {
    arq_.setCodeRate(data_code_rate_);
    arq_.setFixedFrameCodewords(data_frame_cw_count_);
    arq_.setAckBatchThroughMoreFrag(false);

    if (isOFDMMode(negotiated_mode_) || usesBoundedVariableMCDPSKFrames()) {
        file_transfer_.setMaxChunkPayload(currentDataPayloadCapacity());
    }

    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        const auto timing = connection_policy::mcDpskFrameTiming(
            data_modulation_,
            config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol,
            data_frame_cw_count_);
        const size_t window_size = connection_policy::mcDpskWindowSizeForTiming(timing);
        const uint32_t sack_delay_ms = connection_policy::mcDpskSackDelayMs(timing, window_size);
        const uint32_t sack_tail_delay_ms = connection_policy::mcDpskSackTailDelayMs(timing);
        const bool pipelined_dqpsk = window_size > 1 && data_modulation_ == Modulation::DQPSK;
        const int ack_repeat_count = pipelined_dqpsk ? 2 : 1;
        const uint32_t ack_repeat_delay_ms = pipelined_dqpsk
            ? std::clamp<uint32_t>(timing.ack_ms + 250u, 750u, 3000u)
            : 220u;
        arq_.setWindowSize(window_size);
        arq_.setSackDelay(sack_delay_ms);
        arq_.setSackDelayShort(window_size > 1 ? sack_tail_delay_ms : 0);
        arq_.setAckBatchThroughMoreFrag(true);
        arq_.setAckRepeatCount(ack_repeat_count);
        arq_.setAckRepeatDelay(ack_repeat_delay_ms);
        uint32_t ack_timeout_ms = connection_policy::computeMCDPSKAckTimeoutMs(
            timing, window_size, arq_.getSackDelay(), ack_repeat_count);
        if (data_modulation_ == Modulation::DBPSK &&
            config_.mc_dpsk_samples_per_symbol >= 2048) {
            ack_timeout_ms = std::max<uint32_t>(
                ack_timeout_ms, connection_policy::kMCDPSKRobustLowAckTimeoutFloorMs);
        }
        arq_.setAckTimeout(ack_timeout_ms);
        LOG_MODEM(INFO, "Connection: ARQ window=%zu, timeout=%.1fs (data=%ums, ack=%ums x%d), sack_delay=%ums/%ums, ack_repeat=%d/%ums, cw=%d (MC-DPSK %s %s, carriers=%d, sps=%d)",
                  window_size,
                  ack_timeout_ms / 1000.0f,
                  timing.data_ms,
                  timing.ack_ms,
                  ack_repeat_count,
                  arq_.getSackDelay(),
                  arq_.getSackDelayShort(),
                  ack_repeat_count,
                  ack_repeat_delay_ms,
                  data_frame_cw_count_,
                  modulationToString(data_modulation_),
                  codeRateToString(data_code_rate_),
                  config_.mc_dpsk_num_carriers,
                  config_.mc_dpsk_samples_per_symbol);
    } else if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        // Selective-repeat window=3 — chosen after A/B in cli_simulator
        // SNR=8 good fading R1/4 7-message test:
        //   window=1 (was): 180 s wall-clock
        //   window=2:        116 s (-36 %)
        //   window=3:         92 s (-49 %, +96 % throughput)
        //   window=4:         69 s (-62 %)
        //   window=8:         87 s (diminishing returns)
        // Settled on 3 because Codex audit explicitly capped at "2, maybe
        // 3 after tests"; window=4 hasn't been audited even though it
        // also passes every documented baseline. 30 m fading coherence
        // on real channels could correlate across a 4-frame burst (each
        // frame ~3.4 s) and turn one fade into 4 retransmits. If real-
        // OTA testing later shows we have headroom, bump to 4 after a
        // fresh audit. If correlated fades chew throughput, drop to 2.
        constexpr size_t kNarrowWindow = 3;
        arq_.setWindowSize(kNarrowWindow);
        arq_.setMaxRetries(15);
        arq_.setSackDelay(120);
        arq_.setSackDelayShort(0);
        arq_.setAckRepeatCount(1);

        const auto timing = connection_policy::narrowOFDMFrameTiming(
            data_modulation_, data_frame_cw_count_);
        uint32_t timeout_ms = connection_policy::computeNarrowOFDMAckTimeoutMs(
            data_modulation_, data_frame_cw_count_, kNarrowWindow);
        arq_.setAckTimeout(timeout_ms);

        LOG_MODEM(INFO, "Connection: ARQ window=%zu, timeout=%.2fs (data=%ums, ack=%ums), ack_repeat=1, cw=%d (OFDM_NARROW %s %s)",
                  kNarrowWindow, timeout_ms / 1000.0f, timing.data_ms, timing.ack_ms,
                  data_frame_cw_count_,
                  modulationToString(data_modulation_), codeRateToString(data_code_rate_));
    } else {
        const bool near_awgn_ofdm =
            connection_policy::isNearAwgnOFDM(fading_index_, measured_snr_db_);
        arq_.setWindowSize(connection_policy::ofdmWindowSizeForChannel(
            data_modulation_, data_code_rate_, fading_index_, measured_snr_db_));
        arq_.setMaxRetries(15);
        arq_.setAckBatchSize(connection_policy::ofdmAckBatchSize(near_awgn_ofdm));

        const auto timing = connection_policy::wideOFDMFrameTiming(
            data_modulation_, data_code_rate_, data_frame_cw_count_);
        const bool defer_sack_to_burst_tail =
            arq_.getWindowSize() > connection_policy::kWideOFDMWindowFrames;
        const auto sack = connection_policy::ofdmSackDelays(
            defer_sack_to_burst_tail,
            arq_.getWindowSize(),
            timing.data_ms);
        arq_.setSackDelay(sack.delay_ms);
        arq_.setSackDelayShort(sack.short_delay_ms);

        const auto ack_repeat = connection_policy::ofdmAckRepeatProfile(
            data_modulation_, data_code_rate_, near_awgn_ofdm);
        arq_.setAckRepeatCount(ack_repeat.count);
        arq_.setAckRepeatDelay(ack_repeat.delay_ms);

        uint32_t ack_timeout_ms = connection_policy::computeWideOFDMAckTimeoutMs(
            data_modulation_,
            data_code_rate_,
            arq_.getWindowSize(),
            arq_.getSackDelay(),
            ack_repeat.count,
            data_frame_cw_count_);
        arq_.setAckTimeout(ack_timeout_ms);

        LOG_MODEM(INFO,
                  "Connection: ARQ window=%zu, timeout=%.2fs (data=%ums, ack=%ums x%d), max_retries=%d, ack_batch=%u, sack_delay=%ums/%ums, ack_repeat=%d/%ums, cw=%d (OFDM %s %s)",
                  arq_.getWindowSize(),
                  ack_timeout_ms / 1000.0f,
                  timing.data_ms,
                  timing.ack_ms,
                  ack_repeat.count,
                  arq_.getMaxRetries(),
                  arq_.getAckBatchSize(),
                  arq_.getSackDelay(),
                  arq_.getSackDelayShort(),
                  ack_repeat.count,
                  ack_repeat.delay_ms,
                  data_frame_cw_count_,
                  modulationToString(data_modulation_),
                  codeRateToString(data_code_rate_));
    }
}

uint32_t Connection::pingTimeoutMsForCurrentProfile() const {
    // DBPSK MC-DPSK PING/PONG detection has to wait through the same slower
    // training/ref energy check as CONNECT detection. Keeping the
    // standard timer unchanged avoids slowing normal retries while preventing
    // robust DBPSK probe retries from overlapping the following CONNECT.
    const bool robust_dbpsk_probe =
        connect_waveform_ == WaveformMode::MC_DPSK &&
        (config_.forced_modulation == Modulation::DBPSK ||
         config_.mc_dpsk_samples_per_symbol >= 1024);
    return robust_dbpsk_probe ? ROBUST_LOW_PING_TIMEOUT_MS : PING_TIMEOUT_MS;
}

bool Connection::usesBoundedVariableMCDPSKFrames() const {
    return negotiated_mode_ == WaveformMode::MC_DPSK;
}

size_t Connection::currentDataPayloadCapacity() const {
    if (isOFDMMode(negotiated_mode_)) {
        return v2::getFixedFramePayloadCapacity(data_code_rate_, data_frame_cw_count_);
    }
    if (usesBoundedVariableMCDPSKFrames()) {
        return v2::getVariableFramePayloadCapacity(data_code_rate_, data_frame_cw_count_);
    }
    return SIZE_MAX;
}

LadderRungId Connection::currentLadderRungId() const {
    if (negotiated_mode_ == WaveformMode::OFDM_CHIRP) {
        return LadderRungId::OFDM_CHIRP;
    }
    if (negotiated_mode_ == WaveformMode::OFDM_NARROW) {
        return LadderRungId::OFDM_NARROW;
    }
    if (negotiated_mode_ == WaveformMode::MC_DPSK) {
        return connection_policy::rungForMCDPSKConfig(
            data_modulation_, config_.mc_dpsk_num_carriers,
            config_.mc_dpsk_samples_per_symbol, data_frame_cw_count_).id;
    }
    return LadderRungId::UNKNOWN;
}

void Connection::notifyDataModeChanged(float snr_db, float peer_fading_index) {
    if (!on_data_mode_changed_) {
        return;
    }
    const bool mc_dpsk = negotiated_mode_ == WaveformMode::MC_DPSK;
    on_data_mode_changed_(data_modulation_, data_code_rate_, data_frame_cw_count_,
                          snr_db, peer_fading_index,
                          mc_dpsk ? config_.mc_dpsk_num_carriers : 0,
                          mc_dpsk ? config_.mc_dpsk_samples_per_symbol : 0);
}

void Connection::applyDataMode(Modulation mod, CodeRate rate, int cw_count,
                               LadderRungId rung_id) {
    if (rung_id != LadderRungId::UNKNOWN) {
        const auto rung = connection_policy::ladderRungForId(rung_id);
        if (rung.id != LadderRungId::UNKNOWN) {
            negotiated_mode_ = rung.waveform;
            if (rung.waveform == WaveformMode::MC_DPSK) {
                config_.mc_dpsk_num_carriers = rung.num_carriers;
                config_.mc_dpsk_samples_per_symbol = rung.samples_per_symbol;
                mod = rung.modulation;
                rate = rung.code_rate;
                if (cw_count == 0) {
                    cw_count = rung.cw_count;
                }
            }
        }
    }

    // Resolve final CW count: explicit value if specified (e.g. from
    // MODE_CHANGE wire byte), else auto-pick from rate.
    const int new_cw = (cw_count > 0)
        ? v2::sanitizeFixedFrameCodewords(cw_count)
        : connection_policy::recommendCWCount(mod, rate, negotiated_mode_);
    const bool rate_changed = rate != data_code_rate_;
    const bool cw_changed = new_cw != data_frame_cw_count_;
    // Pending chunks must be re-encoded if rate OR CW changed: the ARQ payload
    // capacity depends on both, and chunks queued under the old geometry will
    // overflow / mis-align under the new one.
    const bool requeue_file =
        (rate_changed || cw_changed) &&
        file_transfer_.getState() == FileTransferState::SENDING &&
        file_transfer_.hasPendingChunks();
    const bool refill_file =
        (rate_changed || cw_changed) &&
        file_transfer_.getState() == FileTransferState::SENDING;
    if (requeue_file) {
        file_transfer_.requeuePendingChunks();
    }

    data_modulation_ = mod;
    data_code_rate_ = rate;
    data_frame_cw_count_ = new_cw;
    config_.fixed_frame_codewords = new_cw;
    data_ladder_rung_id_ = (rung_id != LadderRungId::UNKNOWN)
        ? rung_id
        : currentLadderRungId();
    configureArqForCurrentDataMode();
    resetAdaptiveModeController();

    if (refill_file) {
        deferred_file_refill_ = true;
    }
}

void Connection::enterConnected() {
    cancelPendingPongCallback();

    state_ = ConnectionState::CONNECTED;
    connected_time_ms_ = 0;

    // Hold first outbound DATA so peer's PTT-off transition after their
    // CONNECT_ACK TX has time to complete. Without this, every rig with
    // PTT settling >=100ms drops the first DATA frame.
    if (config_.post_connect_data_delay_ms > 0) {
        post_connect_data_hold_active_ = true;
        post_connect_data_hold_remaining_ms_ = config_.post_connect_data_delay_ms;
        LOG_MODEM(INFO, "Connection: CONNECTED, holding outbound DATA for %u ms (peer PTT-off settling)",
                  config_.post_connect_data_delay_ms);
    } else {
        post_connect_data_hold_active_ = false;
        post_connect_data_hold_remaining_ms_ = 0;
    }
    if (is_initiator_ || handshake_confirmed_) {
        responder_handshake_wait_ms_ = 0;
    } else if (responder_handshake_wait_ms_ == 0) {
        responder_handshake_wait_ms_ = RESPONDER_HANDSHAKE_FAILSAFE_MS;
    }

    arq_.setCallsigns(local_call_, remote_call_);
    arq_.reset();
    configureArqForCurrentDataMode();
    resetAdaptiveModeController();

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
    is_initiator_ = false;
    handshake_confirmed_ = false;
    setPhyMaskV1Negotiated(false);
    narrowband_override_ = WaveformMode::AUTO;  // Clear session-scoped narrowband override
    std::string old_remote = remote_call_;
    remote_call_.clear();
    pending_remote_call_.clear();
    mode_change_pending_ = false;
    disconnect_frame_.clear();
    disconnect_pending_ = false;
    disconnect_ack_frame_.clear();
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    post_connect_data_hold_active_ = false;
    post_connect_data_hold_remaining_ms_ = 0;
    held_data_frames_.clear();
    ack_hold_remaining_ms_ = 0;
    held_ack_frames_.clear();
    responder_handshake_wait_ms_ = 0;
    connect_ack_frame_.clear();
    connect_ack_retransmit_ms_ = 0;
    connect_ack_retransmit_interval_ms_ = CONNECT_ACK_RETRANSMIT_MS;
    connect_ack_retx_remaining_ = 0;
    data_ladder_rung_id_ = LadderRungId::UNKNOWN;
    pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    cancelPendingPongCallback();
    arq_callback_defer_refill_ = false;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;
    arq_.reset();
    soft_combine_harq_.clear();
    resetAdaptiveModeController();
    file_transfer_.cancel();
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;
    rx_reassembly_buffer_.clear();

    // Reset connect waveform to DPSK for next connection attempt
    connect_waveform_ = WaveformMode::MC_DPSK;

    LOG_MODEM(INFO, "Connection: Disconnected from %s (%s)",
              old_remote.c_str(), reason.c_str());

    if (on_disconnected_) {
        on_disconnected_(reason);
    }
}

// =============================================================================
// CALLBACKS
// =============================================================================

void Connection::setTransmitCallback(TransmitCallback cb) {
    on_transmit_ = std::move(cb);
}

void Connection::setTransmitBurstCallback(TransmitBurstCallback cb) {
    on_transmit_burst_ = std::move(cb);
}

void Connection::setMCDPSKConfig(int num_carriers, int samples_per_symbol) {
    config_.mc_dpsk_num_carriers = std::clamp(num_carriers, 1, 64);
    config_.mc_dpsk_samples_per_symbol = std::clamp(samples_per_symbol, 1, 8192);
}

void Connection::setPhyMaskV1Negotiated(bool enabled) {
    if (phy_mask_v1_negotiated_ == enabled) {
        return;
    }
    phy_mask_v1_negotiated_ = enabled;
    LOG_MODEM(INFO, "Connection: PHY_MASK_V1 %s",
              enabled ? "negotiated" : "disabled");
    if (on_phy_mask_v1_negotiated_) {
        on_phy_mask_v1_negotiated_(enabled);
    }
}

void Connection::flushBurstBuffer() {
    if (burst_tx_buffer_.empty()) return;

    const size_t real_frame_count = burst_tx_buffer_.size();
    if (shouldPadPartialOFDMBurst(negotiated_mode_,
                                  data_modulation_,
                                  data_code_rate_,
                                  fading_index_,
                                  measured_snr_db_,
                                  file_transfer_.getState(),
                                  real_frame_count)) {
        const size_t remainder =
            real_frame_count % connection_policy::kBurstInterleaveGroupFrames;
        const size_t pad_count =
            connection_policy::kBurstInterleaveGroupFrames - remainder;
        for (size_t i = 0; i < pad_count; ++i) {
            auto pad_frame = v2::makeFixedDataFrame(
                local_call_,
                kOFDMBurstPadCallsign,
                static_cast<uint16_t>(kOFDMBurstPadSeq - i),
                makeOFDMBurstPadPayload(data_code_rate_, data_frame_cw_count_, i),
                data_code_rate_,
                data_frame_cw_count_).serialize();
            burst_tx_buffer_.push_back(pad_frame);
        }
        LOG_MODEM(INFO,
                  "Connection: Padded OFDM burst %zu -> %zu frames for burst interleaver",
                  real_frame_count,
                  burst_tx_buffer_.size());
    }

    if (burst_tx_buffer_.size() == 1 && on_transmit_) {
        // Single frame, no burst needed
        on_transmit_(burst_tx_buffer_[0]);
    } else if (on_transmit_burst_) {
        LOG_MODEM(INFO, "Connection: Flushing burst of %zu frames", burst_tx_buffer_.size());
        on_transmit_burst_(burst_tx_buffer_);
    } else if (on_transmit_) {
        // Fallback: send individually
        for (const auto& frame : burst_tx_buffer_) {
            on_transmit_(frame);
        }
    }
    burst_tx_buffer_.clear();
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

// =============================================================================
// STATS & RESET
// =============================================================================

ConnectionStats Connection::getStats() const {
    ConnectionStats s = stats_;
    s.arq = arq_.getStats();
    return s;
}

void Connection::resetStats() {
    stats_ = ConnectionStats{};
    arq_.resetStats();
    resetAdaptiveModeController();
}

void Connection::setSoftCombiningHARQ(bool enable) {
    soft_combine_harq_.setEnabled(enable);
    LOG_MODEM(INFO, "Connection: soft-combining HARQ %s",
              enable ? "ENABLED" : "disabled");
}

void Connection::reset() {
    state_ = ConnectionState::DISCONNECTED;
    is_initiator_ = false;
    handshake_confirmed_ = false;
    setPhyMaskV1Negotiated(false);
    remote_call_.clear();
    pending_remote_call_.clear();
    timeout_remaining_ms_ = 0;
    connect_retry_count_ = 0;
    connected_time_ms_ = 0;
    narrowband_override_ = WaveformMode::AUTO;  // Clear session-scoped narrowband override
    negotiated_mode_ = WaveformMode::OFDM_COX;
    remote_capabilities_ = ModeCapabilities::OFDM_COX;
    remote_preferred_ = WaveformMode::OFDM_COX;
    mode_change_pending_ = false;
    mode_change_timeout_ms_ = 0;
    mode_change_retry_count_ = 0;
    pending_ladder_rung_id_ = LadderRungId::UNKNOWN;
    cancelPendingPongCallback();
    data_modulation_ = Modulation::DQPSK;
    data_code_rate_ = CodeRate::R1_4;
    data_ladder_rung_id_ = LadderRungId::UNKNOWN;
    connect_waveform_ = WaveformMode::MC_DPSK;  // Reset to DPSK for next connect attempt
    responder_handshake_wait_ms_ = 0;
    connect_ack_frame_.clear();
    connect_ack_retransmit_ms_ = 0;
    connect_ack_retransmit_interval_ms_ = CONNECT_ACK_RETRANSMIT_MS;
    connect_ack_retx_remaining_ = 0;
    burst_mode_active_ = false;
    burst_tx_buffer_.clear();
    arq_callback_defer_refill_ = false;
    deferred_file_refill_ = false;
    deferred_fragment_refill_ = false;
    arq_.reset();
    soft_combine_harq_.clear();
    resetAdaptiveModeController();
    file_transfer_.cancel();
    pending_tx_fragments_.clear();
    pending_tx_fragment_flags_.clear();
    pending_tx_fragment_types_.clear();
    next_fragment_idx_ = 0;
    acked_fragment_count_ = 0;
    rx_reassembly_buffer_.clear();
    LOG_MODEM(DEBUG, "Connection: Full reset");
}

} // namespace protocol
} // namespace ultra
