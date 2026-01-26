// MCDPSKWaveform - Implementation

#include "mc_dpsk_waveform.hpp"
#include "ultra/logging.hpp"
#include <cmath>
#include <sstream>

namespace ultra {

MCDPSKWaveform::MCDPSKWaveform() {
    // Default: 8 carriers for balanced performance
    config_.num_carriers = 8;
    initComponents();
}

MCDPSKWaveform::MCDPSKWaveform(int num_carriers) {
    config_.num_carriers = std::max(3, std::min(20, num_carriers));
    initComponents();
}

MCDPSKWaveform::MCDPSKWaveform(const MultiCarrierDPSKConfig& config)
    : config_(config)
{
    initComponents();
}

void MCDPSKWaveform::initComponents() {
    modulator_ = std::make_unique<MultiCarrierDPSKModulator>(config_);
    demodulator_ = std::make_unique<MultiCarrierDPSKDemodulator>(config_);
    chirp_sync_ = std::make_unique<sync::ChirpSync>(config_.getChirpConfig());
}

WaveformCapabilities MCDPSKWaveform::getCapabilities() const {
    WaveformCapabilities caps;
    caps.supports_cfo_correction = true;    // Via dual chirp detection
    caps.supports_doppler_correction = true; // Frequency diversity
    caps.requires_pilots = false;            // Differential modulation
    caps.supports_differential = true;
    caps.min_snr_db = -3.0f;                // Reliable threshold
    caps.max_snr_db = 15.0f;                // Above this, use OFDM
    caps.max_throughput_bps = getThroughput(CodeRate::R1_4);
    caps.preamble_duration_ms = config_.chirp_duration_ms * 2 + config_.getChirpConfig().gap_ms * 2;
    return caps;
}

void MCDPSKWaveform::configure(Modulation mod, CodeRate rate) {
    modulation_ = mod;
    code_rate_ = rate;

    // MC-DPSK always uses DQPSK internally
    // The modulation parameter affects how we interpret the throughput
    if (mod != Modulation::DQPSK && mod != Modulation::DBPSK && mod != Modulation::D8PSK) {
        LOG_MODEM(WARN, "MCDPSKWaveform: Unsupported modulation %d, using DQPSK",
                  static_cast<int>(mod));
        modulation_ = Modulation::DQPSK;
    }

    // Update demodulator if modulation changed
    if (mod == Modulation::DBPSK) {
        config_.bits_per_symbol = 1;
    } else if (mod == Modulation::D8PSK) {
        config_.bits_per_symbol = 3;
    } else {
        config_.bits_per_symbol = 2;  // DQPSK default
    }

    // Reinitialize with new config
    initComponents();
}

void MCDPSKWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    if (demodulator_) {
        demodulator_->setCFO(cfo_hz);
    }
}

Samples MCDPSKWaveform::generatePreamble() {
    if (!modulator_) {
        return Samples();
    }
    return modulator_->generatePreamble();
}

Samples MCDPSKWaveform::modulate(const Bytes& encoded_data) {
    if (!modulator_) {
        return Samples();
    }
    return modulator_->modulate(encoded_data);
}

bool MCDPSKWaveform::detectSync(SampleSpan samples, SyncResult& result, float threshold) {
    if (!chirp_sync_) {
        return false;
    }

    // Use dual chirp detection for CFO-tolerant sync
    auto chirp_result = chirp_sync_->detectDualChirp(samples, threshold);

    result.detected = chirp_result.success;
    result.start_sample = chirp_result.up_chirp_start;
    result.correlation = std::max(chirp_result.up_correlation, chirp_result.down_correlation);
    result.cfo_hz = chirp_result.cfo_hz;
    result.has_training = true;  // MC-DPSK has training sequence after chirp

    if (chirp_result.success) {
        synced_ = true;
        last_cfo_ = chirp_result.cfo_hz;

        // Calculate data start position
        // Layout: [CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING][REF][DATA...]
        size_t chirp_samples = chirp_sync_->getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.getChirpConfig().gap_ms / 1000.0f);
        size_t training_samples = config_.training_symbols * config_.samples_per_symbol;
        size_t ref_samples = config_.samples_per_symbol;

        if (config_.use_dual_chirp) {
            // After dual chirp: training starts after second gap
            result.start_sample = chirp_result.up_chirp_start +
                                  2 * chirp_samples + 2 * gap_samples +
                                  training_samples + ref_samples;
        } else {
            // Single chirp
            result.start_sample = chirp_result.up_chirp_start +
                                  chirp_samples + gap_samples +
                                  training_samples + ref_samples;
        }

        LOG_MODEM(INFO, "MCDPSKWaveform: Chirp detected at %d, CFO=%.1f Hz, data_start=%d",
                  chirp_result.up_chirp_start, chirp_result.cfo_hz, result.start_sample);
    }

    return result.detected;
}

bool MCDPSKWaveform::process(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    // Apply CFO correction if set
    if (std::abs(cfo_hz_) > 0.1f) {
        demodulator_->setCFO(cfo_hz_);
    }

    // Process samples through demodulator
    bool ready = demodulator_->process(samples);

    if (ready) {
        soft_bits_ = demodulator_->demodulateSoft(samples);
        synced_ = true;
    }

    return ready;
}

std::vector<float> MCDPSKWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

void MCDPSKWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    synced_ = false;
}

bool MCDPSKWaveform::isSynced() const {
    return synced_ || (demodulator_ && demodulator_->isSynced());
}

bool MCDPSKWaveform::hasData() const {
    return !soft_bits_.empty() || (demodulator_ && demodulator_->hasPendingData());
}

float MCDPSKWaveform::estimatedSNR() const {
    return last_snr_;
}

float MCDPSKWaveform::estimatedCFO() const {
    if (demodulator_) {
        return demodulator_->getEstimatedCFO();
    }
    return last_cfo_;
}

std::vector<std::complex<float>> MCDPSKWaveform::getConstellationSymbols() const {
    // MC-DPSK demodulator doesn't track constellation symbols
    // For now, return empty vector
    // TODO: Add constellation tracking to MultiCarrierDPSKDemodulator if needed for GUI
    return {};
}

std::string MCDPSKWaveform::getStatusString() const {
    std::ostringstream oss;
    oss << "MC-DPSK " << config_.num_carriers << " carriers @ "
        << static_cast<int>(getThroughput(code_rate_)) << " bps";
    if (std::abs(cfo_hz_) > 0.5f) {
        oss << " (CFO=" << static_cast<int>(cfo_hz_) << " Hz)";
    }
    return oss.str();
}

float MCDPSKWaveform::getThroughput(CodeRate rate) const {
    // Raw bit rate = symbol_rate * carriers * bits_per_symbol
    float raw_bps = config_.getRawBitRate();

    // Apply code rate
    float code_ratio = 0.25f;  // Default R1/4
    switch (rate) {
        case CodeRate::R1_4: code_ratio = 0.25f; break;
        case CodeRate::R1_3: code_ratio = 0.333f; break;
        case CodeRate::R1_2: code_ratio = 0.5f; break;
        case CodeRate::R2_3: code_ratio = 0.667f; break;
        case CodeRate::R3_4: code_ratio = 0.75f; break;
        case CodeRate::R5_6: code_ratio = 0.833f; break;
    }

    return raw_bps * code_ratio;
}

int MCDPSKWaveform::getPreambleSamples() const {
    if (chirp_sync_) {
        return static_cast<int>(chirp_sync_->getTotalSamples());
    }
    // Fallback calculation
    size_t chirp_samples = static_cast<size_t>(config_.sample_rate * config_.chirp_duration_ms / 1000.0f);
    size_t gap_samples = static_cast<size_t>(config_.sample_rate * 100.0f / 1000.0f);  // 100ms gap
    return static_cast<int>(config_.use_dual_chirp ? 2 * chirp_samples + 2 * gap_samples
                                                   : chirp_samples + gap_samples);
}

void MCDPSKWaveform::setCarrierCount(int carriers) {
    config_.num_carriers = std::max(3, std::min(20, carriers));
    initComponents();
}

} // namespace ultra
