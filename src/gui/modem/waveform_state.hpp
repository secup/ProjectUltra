#pragma once

// WaveformState - Encapsulates waveform mode state machine
//
// Manages the complex state transitions during connection lifecycle:
// - IDLE: Disconnected, listening for incoming PING/CONNECT
// - CONNECTING: Sent CONNECT, waiting for ACK
// - CONNECTED: Active connection with negotiated waveform
// - DISCONNECTING: Sent DISCONNECT, waiting for ACK
//
// This class consolidates the 7 state variables from ModemEngine:
// - waveform_mode_, connect_waveform_, last_rx_waveform_, disconnect_waveform_
// - connected_, handshake_complete_, use_connected_waveform_once_

#include "protocol/frame_v2.hpp"
#include "ultra/types.hpp"
#include <string>

namespace ultra {
namespace gui {

class WaveformState {
public:
    enum class Phase {
        IDLE,           // Disconnected, listening
        CONNECTING,     // Sent CONNECT, waiting for ACK
        CONNECTED,      // Active connection
        DISCONNECTING   // Sent DISCONNECT, waiting for ACK
    };

    WaveformState() = default;

    // ========================================================================
    // STATE QUERIES
    // ========================================================================

    Phase getPhase() const { return phase_; }
    bool isConnected() const { return phase_ == Phase::CONNECTED; }
    bool isDisconnected() const { return phase_ == Phase::IDLE; }
    bool isHandshakeComplete() const { return handshake_complete_; }

    // Get the waveform mode to use for the NEXT transmission
    // This handles the complex logic of:
    // - Using connect_waveform_ when initiating connection
    // - Using negotiated_waveform_ when connected
    // - Using disconnect_waveform_ for DISCONNECT ACK
    protocol::WaveformMode getModeForTx() const {
        if (use_negotiated_once_) {
            return disconnect_waveform_;
        }

        switch (phase_) {
            case Phase::IDLE:
            case Phase::CONNECTING:
                return connect_waveform_;

            case Phase::CONNECTED:
                if (!handshake_complete_) {
                    // Still in handshake, use connect waveform
                    return connect_waveform_;
                }
                return negotiated_waveform_;

            case Phase::DISCONNECTING:
                return disconnect_waveform_;
        }

        return connect_waveform_;  // Default fallback
    }

    // Get the current active waveform mode (for RX)
    protocol::WaveformMode getActiveMode() const {
        return active_waveform_;
    }

    // Get the waveform used for connection handshake
    protocol::WaveformMode getConnectWaveform() const {
        return connect_waveform_;
    }

    // Get the negotiated data waveform
    protocol::WaveformMode getNegotiatedWaveform() const {
        return negotiated_waveform_;
    }

    // Get the last received waveform
    protocol::WaveformMode getLastRxWaveform() const {
        return last_rx_waveform_;
    }

    // Get current data modulation/rate
    Modulation getDataModulation() const { return data_modulation_; }
    CodeRate getDataCodeRate() const { return data_code_rate_; }

    // ========================================================================
    // STATE TRANSITIONS
    // ========================================================================

    // Set the waveform to use for connection attempts (before CONNECT is sent)
    void setConnectWaveform(protocol::WaveformMode mode) {
        connect_waveform_ = mode;
        use_negotiated_once_ = false;  // Clear any leftover flag
    }

    // Set the active waveform mode (affects RX immediately)
    void setActiveWaveform(protocol::WaveformMode mode) {
        active_waveform_ = mode;
    }

    // Record the waveform from the last received frame
    void setLastRxWaveform(protocol::WaveformMode mode) {
        last_rx_waveform_ = mode;
    }

    // Start connection process (we sent CONNECT)
    void startConnection(protocol::WaveformMode mode) {
        phase_ = Phase::CONNECTING;
        connect_waveform_ = mode;
        handshake_complete_ = false;
        use_negotiated_once_ = false;
    }

    // Connection established (received CONNECT_ACK)
    // negotiated_mode is what we'll use for data frames
    void connectionEstablished(protocol::WaveformMode negotiated_mode,
                               Modulation data_mod, CodeRate data_rate) {
        phase_ = Phase::CONNECTED;
        negotiated_waveform_ = negotiated_mode;
        active_waveform_ = negotiated_mode;
        data_modulation_ = data_mod;
        data_code_rate_ = data_rate;
        handshake_complete_ = false;  // Will be set true after first data exchange
    }

    // Mark handshake as complete (first post-ACK frame exchanged)
    void setHandshakeComplete() {
        handshake_complete_ = true;
    }

    // Start disconnect process (we're sending DISCONNECT)
    void startDisconnect() {
        phase_ = Phase::DISCONNECTING;
        // Save the negotiated waveform for the ACK
        disconnect_waveform_ = negotiated_waveform_;
        use_negotiated_once_ = true;
    }

    // Disconnect complete (connection ended)
    void disconnected() {
        phase_ = Phase::IDLE;
        handshake_complete_ = false;
        use_negotiated_once_ = false;
        // Keep connect_waveform_ for next connection attempt
        // Reset active to connect waveform for listening
        active_waveform_ = connect_waveform_;
    }

    // Clear the "use negotiated once" flag (after DISCONNECT ACK sent)
    void clearUseNegotiatedOnce() {
        use_negotiated_once_ = false;
    }

    // Set data modulation mode
    void setDataMode(Modulation mod, CodeRate rate) {
        data_modulation_ = mod;
        data_code_rate_ = rate;
    }

    // ========================================================================
    // DIAGNOSTICS
    // ========================================================================

    std::string getPhaseString() const {
        switch (phase_) {
            case Phase::IDLE: return "IDLE";
            case Phase::CONNECTING: return "CONNECTING";
            case Phase::CONNECTED: return "CONNECTED";
            case Phase::DISCONNECTING: return "DISCONNECTING";
        }
        return "UNKNOWN";
    }

    std::string getStatusString() const {
        std::string status = getPhaseString();
        status += " [";
        status += protocol::waveformModeToString(getModeForTx());
        status += "]";
        if (handshake_complete_) {
            status += " (handshake OK)";
        }
        return status;
    }

private:
    Phase phase_ = Phase::IDLE;

    // Waveform modes
    protocol::WaveformMode active_waveform_ = protocol::WaveformMode::MC_DPSK;
    protocol::WaveformMode connect_waveform_ = protocol::WaveformMode::MC_DPSK;
    protocol::WaveformMode negotiated_waveform_ = protocol::WaveformMode::OFDM_COX;
    protocol::WaveformMode disconnect_waveform_ = protocol::WaveformMode::MC_DPSK;
    protocol::WaveformMode last_rx_waveform_ = protocol::WaveformMode::MC_DPSK;

    // State flags
    bool handshake_complete_ = false;
    bool use_negotiated_once_ = false;

    // Data mode configuration
    Modulation data_modulation_ = Modulation::DQPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;
};

} // namespace gui
} // namespace ultra
