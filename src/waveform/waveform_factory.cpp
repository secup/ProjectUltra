// WaveformFactory - Implementation

#include "waveform_factory.hpp"
#include "mc_dpsk_waveform.hpp"
#include "ofdm_chirp_waveform.hpp"
#include "ultra/logging.hpp"

namespace ultra {

WaveformPtr WaveformFactory::create(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:
            return std::make_unique<MCDPSKWaveform>();

        case protocol::WaveformMode::OFDM_CHIRP:
            return std::make_unique<OFDMChirpWaveform>();

        case protocol::WaveformMode::OFDM_NARROW:
            return std::make_unique<OFDMChirpWaveform>(
                presets::narrowbandOFDM(), protocol::WaveformMode::OFDM_NARROW);

        case protocol::WaveformMode::AUTO:
            // Default to MC-DPSK for AUTO (most robust for connection)
            return std::make_unique<MCDPSKWaveform>();

        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
            LOG_MODEM(WARN, "WaveformFactory: OTFS is not part of the production build");
            return nullptr;

        case protocol::WaveformMode::MFSK:
            LOG_MODEM(WARN, "WaveformFactory: MFSK is reserved but not implemented");
            return nullptr;

        default:
            LOG_MODEM(ERROR, "WaveformFactory: Unknown mode %d", static_cast<int>(mode));
            return nullptr;
    }
}

WaveformPtr WaveformFactory::create(protocol::WaveformMode mode, const ModemConfig& config) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK: {
            MultiCarrierDPSKConfig mc_cfg;
            mc_cfg.sample_rate = static_cast<float>(config.sample_rate);
            // Other fields use defaults
            return std::make_unique<MCDPSKWaveform>(mc_cfg);
        }

        case protocol::WaveformMode::OFDM_CHIRP:
            return std::make_unique<OFDMChirpWaveform>(config);

        case protocol::WaveformMode::OFDM_NARROW:
            return std::make_unique<OFDMChirpWaveform>(config, protocol::WaveformMode::OFDM_NARROW);

        default:
            return create(mode);
    }
}

WaveformPtr WaveformFactory::createMCDPSK(int num_carriers) {
    return std::make_unique<MCDPSKWaveform>(num_carriers);
}

WaveformPtr WaveformFactory::createMCDPSK(const MultiCarrierDPSKConfig& config) {
    return std::make_unique<MCDPSKWaveform>(config);
}

WaveformPtr WaveformFactory::createNarrowbandMCDPSK() {
    return std::make_unique<MCDPSKWaveform>(mc_dpsk_presets::narrowband());
}

WaveformPtr WaveformFactory::createNarrowbandMCDPSK(const MultiCarrierDPSKConfig& session) {
    // Until 2026-11-10 the modem built narrowband waveforms from the preset alone (DQPSK) while
    // the handshake rides DBPSK (streaming_control_profile::profileForMCDPSK): the receiver
    // demodulated every narrowband CONNECT with the wrong constellation (BUG-NARROW-MCDPSK-CONNECT).
    MultiCarrierDPSKConfig cfg = mc_dpsk_presets::narrowband();
    cfg.bits_per_symbol = session.bits_per_symbol;
    cfg.track_clock_offset = session.track_clock_offset;
    cfg.tx_cfo_hz = session.tx_cfo_hz;
    return std::make_unique<MCDPSKWaveform>(cfg);
}

std::vector<protocol::WaveformMode> WaveformFactory::getAvailableModes() {
    return {
        protocol::WaveformMode::MC_DPSK,
        protocol::WaveformMode::OFDM_CHIRP,
        protocol::WaveformMode::OFDM_NARROW,
    };
}

bool WaveformFactory::isSupported(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:
        case protocol::WaveformMode::OFDM_CHIRP:
        case protocol::WaveformMode::OFDM_NARROW:
        case protocol::WaveformMode::AUTO:
            return true;

        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
        case protocol::WaveformMode::MFSK:
            return false;

        default:
            return false;
    }
}

std::string WaveformFactory::getModeName(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:    return "MC-DPSK";
        case protocol::WaveformMode::OFDM_CHIRP: return "OFDM-Chirp";
        case protocol::WaveformMode::OFDM_NARROW: return "OFDM-Narrow";
        case protocol::WaveformMode::OTFS_EQ:    return "OTFS-EQ";
        case protocol::WaveformMode::OTFS_RAW:   return "OTFS-Raw";
        case protocol::WaveformMode::MFSK:       return "MFSK";
        case protocol::WaveformMode::AUTO:       return "AUTO";
        default:                                 return "Unknown";
    }
}

protocol::WaveformMode WaveformFactory::recommendMode(float snr_db) {
    // Legacy SNR-only thresholds on the unified in-band SNR convention.
    // HF has multipath fading that requires margin vs AWGN

    if (snr_db < 20.0f) {
        // Low SNR: Use MC-DPSK with chirp sync
        // Reliable from ~7 dB to 20 dB in-band
        return protocol::WaveformMode::MC_DPSK;
    } else {
        // Mid/high SNR: OFDM with chirp sync (coherent modulation at high SNR)
        return protocol::WaveformMode::OFDM_CHIRP;
    }
}

// NOTE: recommendDataMode() removed - use protocol::recommendDataMode() from waveform_selection.hpp

float WaveformFactory::getMinSNR(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:    return 7.0f;
        case protocol::WaveformMode::OFDM_CHIRP: return 20.0f;
        case protocol::WaveformMode::OFDM_NARROW: return 13.0f;
        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
        case protocol::WaveformMode::MFSK:       return 0.0f;
        default:                                 return 0.0f;
    }
}

float WaveformFactory::getMaxThroughput(protocol::WaveformMode mode) {
    // Approximate max throughput at high SNR with R3/4
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK:    return 1500.0f;   // 20 carriers, DQPSK R3/4
        case protocol::WaveformMode::OFDM_CHIRP: return 4000.0f;   // 30 carriers, D8PSK R2/3
        case protocol::WaveformMode::OFDM_NARROW: return 230.0f;  // 21 carriers, DQPSK R1/2
        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
        case protocol::WaveformMode::MFSK:       return 0.0f;
        default:                                 return 1000.0f;
    }
}

int WaveformFactory::recommendMCDPSKCarriers(float snr_db) {
    // More carriers = higher throughput but need better SNR
    // Carrier spacing affects fading diversity

    if (snr_db < 10.0f) {
        return 5;   // 234 bps with R1/4
    } else if (snr_db < 13.0f) {
        return 5;   // 234 bps
    } else if (snr_db < 18.0f) {
        return 8;   // 375 bps
    } else if (snr_db < 20.0f) {
        return 10;  // 469 bps
    } else if (snr_db < 25.0f) {
        return 13;  // 609 bps
    } else {
        return 20;  // 938 bps
    }
}

} // namespace ultra
