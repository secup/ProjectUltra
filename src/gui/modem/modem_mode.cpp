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

        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
            otfs_demodulator_->reset();
            LOG_MODEM(INFO, "OTFS mode active: M=%d, N=%d",
                      otfs_config_.M, otfs_config_.N);
            break;

        case protocol::WaveformMode::OFDM_CHIRP_PILOTS: {
            // OFDM_CHIRP_PILOTS uses coherent QPSK with pilots for fading channels
            // Chirp provides robust sync, pilots enable channel tracking
            ModemConfig chirp_pilots_config = config_;
            chirp_pilots_config.modulation = Modulation::QPSK;
            chirp_pilots_config.use_pilots = true;
            chirp_pilots_config.pilot_spacing = 4;  // Every 4th carrier = pilot
            chirp_pilots_config.scattered_pilots = true;  // Rotate pilots each symbol
            ofdm_modulator_ = std::make_unique<OFDMModulator>(chirp_pilots_config);
            ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(chirp_pilots_config);
            uint32_t data_carriers = chirp_pilots_config.getDataCarriers();
            LOG_MODEM(INFO, "OFDM_CHIRP_PILOTS mode active: %d carriers (%d data, %d pilots, QPSK)",
                      chirp_pilots_config.num_carriers, data_carriers,
                      chirp_pilots_config.num_carriers - data_carriers);
            break;
        }

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
        // Old samples from DPSK handshake would corrupt OFDM preamble detection
        rx_frame_state_.clear();
        detected_frame_queue_.clear();
        {
            std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
            rx_sample_buffer_.clear();
        }

        // Configure OFDM modulator/demodulator to match data_modulation_
        // This ensures TX and RX use the same constellation when connected
        bool is_differential = (data_modulation_ == Modulation::DBPSK ||
                               data_modulation_ == Modulation::DQPSK ||
                               data_modulation_ == Modulation::D8PSK);
        config_.modulation = data_modulation_;
        config_.code_rate = data_code_rate_;
        config_.use_pilots = !is_differential;

        ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);
        decoder_->setRate(data_code_rate_);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(config_);

        LOG_MODEM(INFO, "Entered connected state, configured for %s %s (pilots=%d)",
                  modulationToString(data_modulation_), codeRateToString(data_code_rate_),
                  config_.use_pilots ? 1 : 0);
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
    // DPSK works down to -11 dB SNR (tested), so we use it for low SNR
    // OFDM requires ~17 dB for reliable sync detection
    if (snr_db < 17.0f) {
        return protocol::WaveformMode::DPSK;
    } else {
        return protocol::WaveformMode::OFDM;
    }
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
