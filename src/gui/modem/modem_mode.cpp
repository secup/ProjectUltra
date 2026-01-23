// modem_mode.cpp - Waveform and mode control for ModemEngine

#include "modem_engine.hpp"
#include "ultra/logging.hpp"
#include "protocol/frame_v2.hpp"
#include <cstdio>

namespace ultra {
namespace gui {

// Helper to get mode description string
static const char* getModeDescription(Modulation mod, CodeRate rate) {
    static char buf[32];
    const char* mod_name;
    switch (mod) {
        case Modulation::DBPSK: mod_name = "DBPSK"; break;
        case Modulation::BPSK:  mod_name = "BPSK"; break;
        case Modulation::DQPSK: mod_name = "DQPSK"; break;
        case Modulation::QPSK:  mod_name = "QPSK"; break;
        case Modulation::D8PSK: mod_name = "D8PSK"; break;
        case Modulation::QAM8:  mod_name = "8QAM"; break;
        case Modulation::QAM16: mod_name = "16QAM"; break;
        case Modulation::QAM32: mod_name = "32QAM"; break;
        case Modulation::QAM64: mod_name = "64QAM"; break;
        default: mod_name = "???"; break;
    }
    const char* rate_name;
    switch (rate) {
        case CodeRate::R1_4: rate_name = "R1/4"; break;
        case CodeRate::R1_2: rate_name = "R1/2"; break;
        case CodeRate::R2_3: rate_name = "R2/3"; break;
        case CodeRate::R3_4: rate_name = "R3/4"; break;
        case CodeRate::R5_6: rate_name = "R5/6"; break;
        default: rate_name = "R?"; break;
    }
    snprintf(buf, sizeof(buf), "%s %s", mod_name, rate_name);
    return buf;
}

void ModemEngine::setWaveformMode(protocol::WaveformMode mode) {
    if (waveform_mode_ == mode) return;

    LOG_MODEM(INFO, "Switching waveform mode: %d -> %d",
              static_cast<int>(waveform_mode_), static_cast<int>(mode));

    waveform_mode_ = mode;

    // Reset the appropriate demodulator for the new mode
    // Clear RX state when switching modes
    rx_frame_state_.clear();
    detected_frame_queue_.clear();
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        rx_sample_buffer_.clear();
    }

    switch (mode) {
        case protocol::WaveformMode::DPSK:
            dpsk_demodulator_->reset();
            LOG_MODEM(INFO, "DPSK mode active: %d-PSK, %d samples/sym, %.1f bps",
                      dpsk_config_.num_phases(),
                      dpsk_config_.samples_per_symbol,
                      dpsk_config_.raw_bps());
            break;

        case protocol::WaveformMode::MFSK:
            mfsk_demodulator_->reset();
            LOG_MODEM(INFO, "MFSK mode active: %d tones, %.1f effective bps",
                      mfsk_config_.num_tones,
                      mfsk_config_.effective_bps());
            break;

        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
            otfs_demodulator_->reset();
            LOG_MODEM(INFO, "OTFS mode active: M=%d, N=%d",
                      otfs_config_.M, otfs_config_.N);
            break;

        case protocol::WaveformMode::OFDM:
        default:
            ofdm_demodulator_->reset();
            LOG_MODEM(INFO, "OFDM mode active: %d carriers, %s",
                      config_.num_carriers,
                      connected_ ? "connected" : "disconnected");
            break;
    }
}

void ModemEngine::setConnectWaveform(protocol::WaveformMode mode) {
    LOG_MODEM(INFO, "Switching connect waveform: %s -> %s",
              protocol::waveformModeToString(connect_waveform_),
              protocol::waveformModeToString(mode));

    connect_waveform_ = mode;

    // Clear any leftover flag from previous disconnect - we're starting fresh
    use_connected_waveform_once_ = false;

    // Configure DPSK for medium preset (DQPSK 62b R1/4) for connection attempts
    if (mode == protocol::WaveformMode::DPSK) {
        dpsk_config_ = dpsk_presets::medium();  // DQPSK 62.5 baud
        dpsk_modulator_ = std::make_unique<DPSKModulator>(dpsk_config_);
        dpsk_demodulator_ = std::make_unique<DPSKDemodulator>(dpsk_config_);
        LOG_MODEM(INFO, "DPSK connect mode: %d-PSK, %.1f baud",
                  dpsk_config_.num_phases(), dpsk_config_.symbol_rate());
    }
}

void ModemEngine::setConnected(bool connected) {
    LOG_MODEM(INFO, "[%s] setConnected(%d) called, was connected_=%d",
              log_prefix_.c_str(), connected ? 1 : 0, connected_ ? 1 : 0);

    if (connected_ == connected) return;

    connected_ = connected;

    if (connected) {
        // Reset handshake state - we'll complete it when we receive first post-ACK frame
        handshake_complete_ = false;
        use_connected_waveform_once_ = false;  // Clear any leftover flag

        // CRITICAL: Clear RX buffer and state when entering connected state
        // Old samples from DPSK/MFSK handshake would corrupt OFDM preamble detection
        rx_frame_state_.clear();
        detected_frame_queue_.clear();
        {
            std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
            rx_sample_buffer_.clear();
        }
        ofdm_demodulator_->reset();

        // NOTE: Don't overwrite waveform_mode_ here - it was already set to the negotiated
        // mode (e.g., OFDM) by the on_mode_negotiated_ callback before setConnected() is called.
        // During handshake, TX uses last_rx_waveform_ (set by RX path when we received CONNECT/CONNECT_ACK)

        LOG_MODEM(INFO, "Entered connected state, RX buffer cleared (negotiated=%d, TX uses last_rx=%d)",
                  static_cast<int>(waveform_mode_), static_cast<int>(last_rx_waveform_));
    } else {
        // Switching to disconnected state - use robust mode for RX
        ModemConfig rx_config = config_;
        rx_config.modulation = Modulation::DQPSK;
        rx_config.code_rate = CodeRate::R1_4;

        decoder_->setRate(CodeRate::R1_4);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

        // CRITICAL: Clear RX state so acquisition thread can detect new PINGs
        rx_frame_state_.clear();
        detected_frame_queue_.clear();
        {
            std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
            rx_sample_buffer_.clear();
        }
        dpsk_demodulator_->reset();

        // Keep using connected waveform for the next TX (DISCONNECT ACK)
        // This handles the case where the ACK is queued before setConnected(false) is called
        use_connected_waveform_once_ = true;
        LOG_MODEM(INFO, "Switched to disconnected mode (RX: DQPSK R1/4, next TX uses connected waveform)");
        handshake_complete_ = false;  // Reset for next connection
    }
}

void ModemEngine::setHandshakeComplete(bool complete) {
    if (handshake_complete_ == complete) return;

    handshake_complete_ = complete;

    if (complete) {
        LOG_MODEM(INFO, "Handshake complete, TX now uses waveform_mode_=%d",
                  static_cast<int>(waveform_mode_));
    }
}

void ModemEngine::setDataMode(Modulation mod, CodeRate rate) {
    data_modulation_ = mod;
    data_code_rate_ = rate;

    // Determine if modulation is differential (doesn't need pilots)
    bool is_differential = (mod == Modulation::DBPSK ||
                           mod == Modulation::DQPSK ||
                           mod == Modulation::D8PSK);

    // If already connected, update both TX and RX to match
    if (connected_) {
        // Update base config for TX modulator
        config_.modulation = mod;
        config_.code_rate = rate;
        config_.use_pilots = !is_differential;  // QAM needs pilots, DQPSK doesn't

        // Recreate modulator with new config
        ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);

        // Recreate demodulator with matching config
        decoder_->setRate(rate);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(config_);

        LOG_MODEM(INFO, "TX/RX OFDM updated: mod=%d, rate=%d, use_pilots=%d",
                  static_cast<int>(mod), static_cast<int>(rate), config_.use_pilots ? 1 : 0);
    }

    LOG_MODEM(INFO, "Data mode set to: %s", getModeDescription(mod, rate));
}

void ModemEngine::recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate) {
    // Conservative thresholds calibrated for REAL HF channels (not just AWGN)
    // HF has multipath fading that requires 3-6 dB extra margin vs AWGN
    // These match the thresholds in connection.cpp for consistency
    if (snr_db >= 30.0f) {
        // Excellent conditions (rare) - use high throughput
        mod = Modulation::QAM16;
        rate = CodeRate::R3_4;
    } else if (snr_db >= 25.0f) {
        // Very good conditions
        mod = Modulation::QAM16;
        rate = CodeRate::R2_3;
    } else if (snr_db >= 20.0f) {
        // Good conditions - sweet spot for speed
        mod = Modulation::DQPSK;
        rate = CodeRate::R2_3;
    } else if (snr_db >= 16.0f) {
        // Typical good HF - balanced speed/reliability
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_2;
    } else if (snr_db >= 12.0f) {
        // Typical HF - prioritize reliability
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_4;
    } else if (snr_db >= 8.0f) {
        // Marginal conditions
        mod = Modulation::BPSK;
        rate = CodeRate::R1_4;
    } else {
        // Very poor conditions - maximum robustness
        mod = Modulation::BPSK;
        rate = CodeRate::R1_4;
    }
}

protocol::WaveformMode ModemEngine::recommendWaveformMode(float snr_db) {
    // Waveform selection based on measured SNR
    // Thresholds based on testing (test_mode_snr tool):
    //   < 0 dB:  MFSK (works at -17 dB reported / 0 dB actual)
    //   0-17 dB: DPSK (single-carrier, OFDM sync fails below ~17 dB)
    //   > 17 dB: OFDM (highest throughput, needs reliable sync)

    if (snr_db < 0.0f) {
        return protocol::WaveformMode::MFSK;
    } else if (snr_db < 17.0f) {
        return protocol::WaveformMode::DPSK;
    } else {
        return protocol::WaveformMode::OFDM;
    }
}

void ModemEngine::setMFSKMode(int num_tones) {
    // Select appropriate MFSK preset based on number of tones
    switch (num_tones) {
        case 2:
            mfsk_config_ = mfsk_presets::robust();  // ~8 bps, most robust
            break;
        case 4:
            mfsk_config_ = mfsk_presets::low_snr();  // ~21 bps
            break;
        case 8:
            mfsk_config_ = mfsk_presets::medium();  // ~47 bps, default
            break;
        case 16:
            mfsk_config_ = mfsk_presets::fast();  // ~62 bps
            break;
        case 32:
            mfsk_config_ = mfsk_presets::turbo();  // ~156 bps, fastest MFSK
            break;
        default:
            LOG_MODEM(WARN, "Invalid MFSK tones %d, using 8", num_tones);
            mfsk_config_ = mfsk_presets::medium();
            break;
    }

    // Recreate modulator/demodulator with new config
    mfsk_modulator_ = std::make_unique<MFSKModulator>(mfsk_config_);
    mfsk_demodulator_ = std::make_unique<MFSKDemodulator>(mfsk_config_);

    LOG_MODEM(INFO, "MFSK mode set: %d tones, %.1f bps",
              mfsk_config_.num_tones, mfsk_config_.effective_bps());
}

void ModemEngine::setDPSKMode(DPSKModulation mod, int samples_per_symbol) {
    // Configure DPSK based on modulation type and symbol rate
    dpsk_config_.modulation = mod;
    dpsk_config_.samples_per_symbol = samples_per_symbol;

    // Recreate modulator/demodulator with new config
    dpsk_modulator_ = std::make_unique<DPSKModulator>(dpsk_config_);
    dpsk_demodulator_ = std::make_unique<DPSKDemodulator>(dpsk_config_);

    const char* mod_name = "DQPSK";
    switch (mod) {
        case DPSKModulation::DBPSK: mod_name = "DBPSK"; break;
        case DPSKModulation::DQPSK: mod_name = "DQPSK"; break;
        case DPSKModulation::D8PSK: mod_name = "D8PSK"; break;
    }

    LOG_MODEM(INFO, "DPSK mode set: %s, %d samples/sym (%.1f baud), %.1f bps",
              mod_name,
              dpsk_config_.samples_per_symbol,
              dpsk_config_.symbol_rate(),
              dpsk_config_.raw_bps());
}

} // namespace gui
} // namespace ultra
