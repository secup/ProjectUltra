// OFDMNvisWaveform - Implementation

#include "ofdm_cox_waveform.hpp"
#include "ultra/logging.hpp"
#include <sstream>

namespace ultra {

OFDMNvisWaveform::OFDMNvisWaveform() {
    // Default OFDM_COX configuration
    config_.fft_size = 512;
    config_.num_carriers = 30;
    config_.modulation = Modulation::QPSK;
    config_.code_rate = CodeRate::R1_2;
    config_.use_pilots = true;  // Coherent mode needs pilots
    initComponents();
}

OFDMNvisWaveform::OFDMNvisWaveform(const ModemConfig& config)
    : config_(config)
{
    initComponents();
}

void OFDMNvisWaveform::initComponents() {
    modulator_ = std::make_unique<OFDMModulator>(config_);
    demodulator_ = std::make_unique<OFDMDemodulator>(config_);
}

WaveformCapabilities OFDMNvisWaveform::getCapabilities() const {
    WaveformCapabilities caps;
    caps.supports_cfo_correction = true;    // Via Schmidl-Cox
    caps.supports_doppler_correction = true; // Via pilot tracking
    caps.requires_pilots = config_.use_pilots;
    caps.supports_differential = true;       // Can use DQPSK

    // Coherent modulation needs better SNR
    bool is_differential = (config_.modulation == Modulation::DBPSK ||
                           config_.modulation == Modulation::DQPSK ||
                           config_.modulation == Modulation::D8PSK);
    caps.min_snr_db = is_differential ? 12.0f : 17.0f;
    caps.max_snr_db = 35.0f;
    caps.max_throughput_bps = getThroughput(CodeRate::R3_4);

    // Schmidl-Cox preamble is ~2 OFDM symbols
    caps.preamble_duration_ms = 2.0f * getSamplesPerSymbol() * 1000.0f / config_.sample_rate;

    return caps;
}

void OFDMNvisWaveform::configure(Modulation mod, CodeRate rate) {
    config_.modulation = mod;
    config_.code_rate = rate;

    // Automatically determine if we need pilots
    bool is_differential = (mod == Modulation::DBPSK ||
                           mod == Modulation::DQPSK ||
                           mod == Modulation::D8PSK);
    config_.use_pilots = !is_differential;

    // Reinitialize with new config
    initComponents();

    LOG_MODEM(INFO, "OFDMNvisWaveform: configured for %s %s (pilots=%d)",
              modulationToString(mod), codeRateToString(rate), config_.use_pilots ? 1 : 0);
}

void OFDMNvisWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    if (demodulator_) {
        demodulator_->setFrequencyOffset(cfo_hz);
    }
}

void OFDMNvisWaveform::setTxFrequencyOffset(float cfo_hz) {
    // Set TX CFO in config and reinitialize
    config_.tx_cfo_hz = cfo_hz;
    initComponents();

    LOG_MODEM(INFO, "OFDMNvisWaveform: TX CFO set to %.1f Hz", cfo_hz);
}

Samples OFDMNvisWaveform::generatePreamble() {
    if (!modulator_) {
        return Samples();
    }
    return modulator_->generatePreamble();
}

Samples OFDMNvisWaveform::modulate(const Bytes& encoded_data) {
    if (!modulator_) {
        return Samples();
    }
    ByteSpan span(encoded_data.data(), encoded_data.size());
    return modulator_->modulate(span, config_.modulation);
}

bool OFDMNvisWaveform::detectSync(SampleSpan samples, SyncResult& result, float threshold) {
    // OFDMDemodulator handles Schmidl-Cox sync internally in process()
    // We can't easily separate detection from processing with current interface
    // So this method is mainly used to check if demodulator found sync

    // Process samples and check for sync
    if (!demodulator_) {
        return false;
    }

    demodulator_->process(samples);

    if (demodulator_->isSynced()) {
        result.detected = true;
        result.start_sample = static_cast<int>(demodulator_->getLastSyncOffset());
        result.cfo_hz = demodulator_->getFrequencyOffset();
        result.snr_estimate = demodulator_->getEstimatedSNR();
        result.has_training = true;
        return true;
    }

    return false;
}

bool OFDMNvisWaveform::process(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    bool ready = demodulator_->process(samples);

    if (ready) {
        soft_bits_ = demodulator_->getSoftBits();
    }

    return ready;
}

std::vector<float> OFDMNvisWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

void OFDMNvisWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    // NOTE: CFO is intentionally preserved across reset() for continuous tracking
    // Use setFrequencyOffset(0) to explicitly clear if needed
}

bool OFDMNvisWaveform::isSynced() const {
    return demodulator_ && demodulator_->isSynced();
}

bool OFDMNvisWaveform::hasData() const {
    return !soft_bits_.empty() || (demodulator_ && demodulator_->hasPendingData());
}

float OFDMNvisWaveform::estimatedSNR() const {
    if (demodulator_) {
        return demodulator_->getEstimatedSNR();
    }
    return 0.0f;
}

float OFDMNvisWaveform::estimatedCFO() const {
    if (demodulator_) {
        return demodulator_->getFrequencyOffset();
    }
    return cfo_hz_;
}

std::vector<std::complex<float>> OFDMNvisWaveform::getConstellationSymbols() const {
    if (demodulator_) {
        return demodulator_->getConstellationSymbols();
    }
    return {};
}

std::string OFDMNvisWaveform::getStatusString() const {
    std::ostringstream oss;
    oss << "OFDM-COX " << config_.num_carriers << " carriers, "
        << modulationToString(config_.modulation) << " "
        << codeRateToString(config_.code_rate);
    if (config_.use_pilots) {
        oss << " (pilots)";
    }
    return oss.str();
}

float OFDMNvisWaveform::getThroughput(CodeRate rate) const {
    // Bits per symbol per carrier
    int bits_per_carrier = 2;  // Default QPSK
    switch (config_.modulation) {
        case Modulation::DBPSK:
        case Modulation::BPSK:  bits_per_carrier = 1; break;
        case Modulation::DQPSK:
        case Modulation::QPSK:  bits_per_carrier = 2; break;
        case Modulation::D8PSK:
        case Modulation::QAM8:  bits_per_carrier = 3; break;
        case Modulation::QAM16: bits_per_carrier = 4; break;
        case Modulation::QAM32: bits_per_carrier = 5; break;
        case Modulation::QAM64: bits_per_carrier = 6; break;
        default: break;
    }

    // Number of data carriers
    int data_carriers = config_.num_carriers;
    if (config_.use_pilots && config_.pilot_spacing > 0) {
        data_carriers = config_.num_carriers - config_.num_carriers / config_.pilot_spacing;
    }

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

int OFDMNvisWaveform::getSamplesPerSymbol() const {
    if (modulator_) {
        return static_cast<int>(modulator_->samplesPerSymbol());
    }
    // Fallback calculation: FFT + CP
    int cp_samples = 0;
    switch (config_.cp_mode) {
        case CyclicPrefixMode::SHORT:  cp_samples = config_.fft_size / 8; break;
        case CyclicPrefixMode::MEDIUM: cp_samples = config_.fft_size / 4; break;
        case CyclicPrefixMode::LONG:   cp_samples = config_.fft_size / 2; break;
    }
    return config_.fft_size + cp_samples;
}

int OFDMNvisWaveform::getPreambleSamples() const {
    // Schmidl-Cox preamble: 2 OFDM symbols (STS repeated, LTS for fine sync)
    return 2 * getSamplesPerSymbol();
}

int OFDMNvisWaveform::getMinSamplesForFrame() const {
    // Training symbols + minimum data for 1 codeword (648 bits)
    int training_samples = 2 * getSamplesPerSymbol();  // 2 training symbols

    // Bits per carrier based on modulation
    int bits_per_carrier = 2;  // Default QPSK
    switch (config_.modulation) {
        case Modulation::BPSK:
        case Modulation::DBPSK: bits_per_carrier = 1; break;
        case Modulation::QPSK:
        case Modulation::DQPSK: bits_per_carrier = 2; break;
        case Modulation::D8PSK: bits_per_carrier = 3; break;
        case Modulation::QAM16: bits_per_carrier = 4; break;
        case Modulation::QAM32: bits_per_carrier = 5; break;
        case Modulation::QAM64: bits_per_carrier = 6; break;
        default: bits_per_carrier = 2; break;
    }

    // Number of data carriers
    int data_carriers = static_cast<int>(config_.num_carriers);
    if (config_.use_pilots && config_.pilot_spacing > 0) {
        data_carriers = config_.num_carriers - config_.num_carriers / config_.pilot_spacing;
    }

    int bits_per_symbol = data_carriers * bits_per_carrier;
    int data_symbols = (648 + bits_per_symbol - 1) / bits_per_symbol;
    int data_samples = data_symbols * getSamplesPerSymbol();

    return training_samples + data_samples;
}

void OFDMNvisWaveform::setUsePilots(bool use_pilots) {
    config_.use_pilots = use_pilots;
    initComponents();
}

} // namespace ultra
