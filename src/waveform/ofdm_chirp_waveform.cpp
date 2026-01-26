// OFDMChirpWaveform - Implementation

#include "ofdm_chirp_waveform.hpp"
#include "ultra/logging.hpp"
#include <sstream>

namespace ultra {

OFDMChirpWaveform::OFDMChirpWaveform() {
    // Default OFDM_CHIRP configuration
    config_.fft_size = 512;
    config_.num_carriers = 30;
    config_.modulation = Modulation::DQPSK;  // Differential for fading
    config_.code_rate = CodeRate::R1_2;
    config_.use_pilots = false;  // DQPSK doesn't need pilots
    initComponents();
}

OFDMChirpWaveform::OFDMChirpWaveform(const ModemConfig& config)
    : config_(config)
{
    // Force differential modulation for chirp mode
    if (config_.modulation != Modulation::DBPSK &&
        config_.modulation != Modulation::DQPSK &&
        config_.modulation != Modulation::D8PSK) {
        config_.modulation = Modulation::DQPSK;
    }
    config_.use_pilots = false;
    initComponents();
}

void OFDMChirpWaveform::initComponents() {
    modulator_ = std::make_unique<OFDMModulator>(config_);
    demodulator_ = std::make_unique<OFDMDemodulator>(config_);
    chirp_sync_ = std::make_unique<sync::ChirpSync>(getChirpConfig());
}

sync::ChirpConfig OFDMChirpWaveform::getChirpConfig() const {
    sync::ChirpConfig cfg;
    cfg.sample_rate = static_cast<float>(config_.sample_rate);
    cfg.f_start = 300.0f;
    cfg.f_end = 2700.0f;
    cfg.duration_ms = 500.0f;
    cfg.gap_ms = 100.0f;
    cfg.use_dual_chirp = true;  // For CFO estimation
    return cfg;
}

WaveformCapabilities OFDMChirpWaveform::getCapabilities() const {
    WaveformCapabilities caps;
    caps.supports_cfo_correction = true;    // Via dual chirp
    caps.supports_doppler_correction = true; // OFDM with differential
    caps.requires_pilots = false;            // Differential modulation
    caps.supports_differential = true;
    caps.min_snr_db = 10.0f;                // Lower than Schmidl-Cox
    caps.max_snr_db = 20.0f;                // Above this, use OFDM_COX
    caps.max_throughput_bps = getThroughput(CodeRate::R2_3);

    // Chirp preamble + training
    caps.preamble_duration_ms = chirp_sync_ ? (chirp_sync_->getTotalSamples() * 1000.0f / config_.sample_rate)
                                            : 1200.0f;

    return caps;
}

void OFDMChirpWaveform::configure(Modulation mod, CodeRate rate) {
    // Only differential modulations allowed for chirp mode
    if (mod != Modulation::DBPSK && mod != Modulation::DQPSK && mod != Modulation::D8PSK) {
        LOG_MODEM(WARN, "OFDMChirpWaveform: Unsupported modulation %d, using DQPSK",
                  static_cast<int>(mod));
        mod = Modulation::DQPSK;
    }

    config_.modulation = mod;
    config_.code_rate = rate;
    config_.use_pilots = false;

    // Reinitialize
    initComponents();

    LOG_MODEM(INFO, "OFDMChirpWaveform: configured for %s %s",
              modulationToString(mod), codeRateToString(rate));
}

void OFDMChirpWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    if (demodulator_) {
        demodulator_->setFrequencyOffset(cfo_hz);
    }
}

Samples OFDMChirpWaveform::generatePreamble() {
    if (!chirp_sync_ || !modulator_) {
        return Samples();
    }

    // Generate: [CHIRP][TRAINING_SYMBOLS]
    Samples chirp = chirp_sync_->generate();
    Samples training = modulator_->generateTrainingSymbols(2);

    Samples preamble;
    preamble.reserve(chirp.size() + training.size());
    preamble.insert(preamble.end(), chirp.begin(), chirp.end());
    preamble.insert(preamble.end(), training.begin(), training.end());

    return preamble;
}

Samples OFDMChirpWaveform::modulate(const Bytes& encoded_data) {
    if (!modulator_) {
        return Samples();
    }
    ByteSpan span(encoded_data.data(), encoded_data.size());
    return modulator_->modulate(span, config_.modulation);
}

bool OFDMChirpWaveform::detectSync(SampleSpan samples, SyncResult& result, float threshold) {
    if (!chirp_sync_) {
        return false;
    }

    // Use dual chirp detection for CFO-tolerant sync
    auto chirp_result = chirp_sync_->detectDualChirp(samples, threshold);

    result.detected = chirp_result.success;
    result.correlation = std::max(chirp_result.up_correlation, chirp_result.down_correlation);
    result.cfo_hz = chirp_result.cfo_hz;
    result.has_training = true;

    if (chirp_result.success) {
        synced_ = true;
        last_cfo_ = chirp_result.cfo_hz;

        // Calculate where OFDM data starts
        // Layout: [CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING_SYMBOLS][DATA...]
        size_t chirp_samples = chirp_sync_->getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * 100.0f / 1000.0f);
        size_t training_samples = 2 * getSamplesPerSymbol();  // 2 training OFDM symbols

        result.start_sample = chirp_result.up_chirp_start +
                              2 * chirp_samples + 2 * gap_samples +
                              training_samples;

        LOG_MODEM(INFO, "OFDMChirpWaveform: Chirp detected at %d, CFO=%.1f Hz, OFDM_start=%d",
                  chirp_result.up_chirp_start, chirp_result.cfo_hz, result.start_sample);
    }

    return result.detected;
}

bool OFDMChirpWaveform::process(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    // Apply CFO correction if we have an estimate
    if (std::abs(cfo_hz_) > 0.1f) {
        demodulator_->setFrequencyOffset(cfo_hz_);
    }

    // Use pre-synced processing (chirp provides timing)
    bool ready = demodulator_->processPresynced(samples, 2);

    if (ready) {
        soft_bits_ = demodulator_->getSoftBits();
        last_snr_ = demodulator_->getEstimatedSNR();
    }

    return ready;
}

std::vector<float> OFDMChirpWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

void OFDMChirpWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    synced_ = false;
}

bool OFDMChirpWaveform::isSynced() const {
    return synced_ || (demodulator_ && demodulator_->isSynced());
}

bool OFDMChirpWaveform::hasData() const {
    return !soft_bits_.empty() || (demodulator_ && demodulator_->hasPendingData());
}

float OFDMChirpWaveform::estimatedSNR() const {
    if (demodulator_) {
        return demodulator_->getEstimatedSNR();
    }
    return last_snr_;
}

float OFDMChirpWaveform::estimatedCFO() const {
    if (std::abs(last_cfo_) > 0.1f) {
        return last_cfo_;
    }
    if (demodulator_) {
        return demodulator_->getFrequencyOffset();
    }
    return cfo_hz_;
}

std::vector<std::complex<float>> OFDMChirpWaveform::getConstellationSymbols() const {
    if (demodulator_) {
        return demodulator_->getConstellationSymbols();
    }
    return {};
}

std::string OFDMChirpWaveform::getStatusString() const {
    std::ostringstream oss;
    oss << "OFDM-Chirp " << config_.num_carriers << " carriers, "
        << modulationToString(config_.modulation) << " "
        << codeRateToString(config_.code_rate);
    if (std::abs(last_cfo_) > 0.5f) {
        oss << " (CFO=" << static_cast<int>(last_cfo_) << " Hz)";
    }
    return oss.str();
}

float OFDMChirpWaveform::getThroughput(CodeRate rate) const {
    // Bits per symbol per carrier (differential modes)
    int bits_per_carrier = 2;  // Default DQPSK
    switch (config_.modulation) {
        case Modulation::DBPSK: bits_per_carrier = 1; break;
        case Modulation::DQPSK: bits_per_carrier = 2; break;
        case Modulation::D8PSK: bits_per_carrier = 3; break;
        default: bits_per_carrier = 2; break;
    }

    // All carriers are data (no pilots for differential)
    int data_carriers = config_.num_carriers;

    // Symbol rate
    float symbol_rate = static_cast<float>(config_.sample_rate) / getSamplesPerSymbol();

    // Raw bit rate
    float raw_bps = symbol_rate * data_carriers * bits_per_carrier;

    // Apply code rate
    float code_ratio = 0.5f;
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

int OFDMChirpWaveform::getSamplesPerSymbol() const {
    if (modulator_) {
        return static_cast<int>(modulator_->samplesPerSymbol());
    }
    // Fallback calculation
    int cp_samples = 0;
    switch (config_.cp_mode) {
        case CyclicPrefixMode::SHORT:  cp_samples = config_.fft_size / 8; break;
        case CyclicPrefixMode::MEDIUM: cp_samples = config_.fft_size / 4; break;
        case CyclicPrefixMode::LONG:   cp_samples = config_.fft_size / 2; break;
    }
    return config_.fft_size + cp_samples;
}

int OFDMChirpWaveform::getPreambleSamples() const {
    int chirp_total = chirp_sync_ ? static_cast<int>(chirp_sync_->getTotalSamples())
                                  : static_cast<int>(config_.sample_rate * 1.2f);  // ~1.2 sec default
    int training = 2 * getSamplesPerSymbol();  // 2 OFDM training symbols
    return chirp_total + training;
}

} // namespace ultra
