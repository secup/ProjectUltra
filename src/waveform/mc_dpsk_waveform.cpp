// MCDPSKWaveform - Implementation

#include "mc_dpsk_waveform.hpp"
#include "ultra/logging.hpp"
#include <cmath>

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

    // Keep constructor logging allocation-light for older Windows runtimes.
    // Avoid LOG_MODEM in this early constructor path on older Win10 runtimes.
    (void)config_.num_carriers;
    (void)config_.samples_per_symbol;
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

void MCDPSKWaveform::setTxFrequencyOffset(float cfo_hz) {
    // Set TX CFO in config and reinitialize
    config_.tx_cfo_hz = cfo_hz;
    initComponents();

    LOG_MODEM(INFO, "MCDPSKWaveform: TX CFO set to %.1f Hz", cfo_hz);
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
    result.preamble_start_sample = chirp_result.up_chirp_start;
    result.correlation = std::max(chirp_result.up_correlation, chirp_result.down_correlation);
    result.cfo_hz = chirp_result.cfo_hz;
    result.gap_error_samples = chirp_result.gap_error_samples;
    result.has_training = true;  // MC-DPSK has training sequence after chirp

    if (chirp_result.success) {
        synced_ = true;
        last_cfo_ = chirp_result.cfo_hz;

        // Calculate where TRAINING starts (process() needs training+ref+data)
        // Layout: [UP-CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING][REF][DATA...]
        //                                         ^-- start_sample points here
        //
        // IMPORTANT: Use down_chirp position for training_start calculation!
        // With CFO, up_chirp and down_chirp positions shift in OPPOSITE directions:
        //   up_chirp: shifts by -CFO × cfo_to_samples
        //   down_chirp: shifts by +CFO × cfo_to_samples
        // Using up_chirp_start + fixed_offset gives growing error with CFO.
        // Using down_chirp_start gives more accurate training position.
        // (Same approach as OFDMChirpWaveform)
        size_t chirp_samples = chirp_sync_->getChirpSamples();
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * config_.getChirpConfig().gap_ms / 1000.0f);

        if (config_.use_dual_chirp) {
            // Training starts after down chirp + gap
            result.start_sample = chirp_result.down_chirp_start +
                                  chirp_samples + gap_samples;
            // Silent inter-chirp gap [up_end, down_start): the frame's own
            // noise-only region, used as the burst-time noise reference for
            // the PING payload-absence gate.
            const int gap_start = chirp_result.up_chirp_start +
                                  static_cast<int>(chirp_samples);
            const int gap_len = chirp_result.down_chirp_start - gap_start;
            if (gap_len > 0) {
                result.interchirp_gap_start_sample = gap_start;
                result.interchirp_gap_len = gap_len;
            }
        } else {
            // Single chirp
            result.start_sample = chirp_result.up_chirp_start +
                                  chirp_samples + gap_samples;
        }
        // NOTE: Do NOT add training_samples + ref_samples - process() needs them

        LOG_MODEM(INFO, "MCDPSKWaveform: Chirp detected at up=%d, down=%d, CFO=%.1f Hz, training_start=%d",
                  chirp_result.up_chirp_start, chirp_result.down_chirp_start,
                  chirp_result.cfo_hz, result.start_sample);
    }

    return result.detected;
}

bool MCDPSKWaveform::process(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    // Debug: Check signal energy at different positions
    // Training should be at start (samples[0]), ref at training_end, data after
    size_t training_samples = config_.training_symbols * config_.samples_per_symbol;
    size_t ref_samples = config_.samples_per_symbol;

    auto calcRMS = [&](size_t start, size_t len) {
        float e = 0.0f;
        for (size_t i = start; i < start + len && i < samples.size(); i++) {
            e += samples[i] * samples[i];
        }
        return std::sqrt(e / std::min(len, samples.size() - start));
    };

    LOG_MODEM(DEBUG, "MCDPSKWaveform: process: samples=%zu, training=%zu, ref=%zu",
              samples.size(), training_samples, ref_samples);
    LOG_MODEM(DEBUG, "MCDPSKWaveform: RMS: training[0]=%f, ref[%zu]=%f, data[%zu]=%f",
              calcRMS(0, 512), training_samples, calcRMS(training_samples, 512),
              training_samples + ref_samples, calcRMS(training_samples + ref_samples, 512));

    // Tell demodulator that chirp was already detected externally via detectSync()
    // This puts it in GOT_CHIRP state so it processes data directly without
    // looking for chirp in the samples
    LOG_MODEM(DEBUG, "MCDPSKWaveform: process: setting CFO=%.1f Hz in demodulator", cfo_hz_);
    demodulator_->setChirpDetected(cfo_hz_);

    // Process samples through demodulator
    bool ready = demodulator_->process(samples);

    LOG_MODEM(DEBUG, "MCDPSKWaveform: process: input_samples=%zu, ready=%d, demod_cfo=%.1f",
              samples.size(), ready, demodulator_->getEstimatedCFO());

    if (ready) {
        // Get soft bits from demodulator's internal state (computed in processGotChirp)
        soft_bits_ = demodulator_->getSoftBits();
        last_snr_valid_ = demodulator_->hasEstimatedSNR();
        if (last_snr_valid_) {
            last_snr_ = demodulator_->getEstimatedSNR();
        }
        last_data_aided_snr_valid_ = demodulator_->hasDataAidedSNR();
        if (last_data_aided_snr_valid_) {
            last_data_aided_snr_ = demodulator_->getDataAidedSNRdB();
        }
        LOG_MODEM(DEBUG, "MCDPSKWaveform: got %zu soft bits", soft_bits_.size());
        synced_ = true;
    }

    return ready;
}

std::vector<float> MCDPSKWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

std::vector<float> MCDPSKWaveform::takeUntrackedSoftBits() {
    return demodulator_ ? demodulator_->takeUntrackedSoftBits() : std::vector<float>{};
}

bool MCDPSKWaveform::processDataOnly(SampleSpan samples) {
    if (!demodulator_) {
        return false;
    }

    soft_bits_ = demodulator_->demodulateDataOnly(samples);
    if (demodulator_->hasEstimatedSNR()) {
        last_snr_valid_ = true;
        last_snr_ = demodulator_->getEstimatedSNR();
    }
    if (demodulator_->hasDataAidedSNR()) {
        last_data_aided_snr_valid_ = true;
        last_data_aided_snr_ = demodulator_->getDataAidedSNRdB();
    }
    synced_ = !soft_bits_.empty();
    LOG_MODEM(DEBUG, "MCDPSKWaveform: data-only process: samples=%zu, soft_bits=%zu",
              samples.size(), soft_bits_.size());
    return synced_;
}

void MCDPSKWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    synced_ = false;
    last_snr_valid_ = false;
    last_snr_ = 0.0f;
    last_data_aided_snr_valid_ = false;
    last_data_aided_snr_ = 0.0f;
    // NOTE: CFO is intentionally preserved across reset() for continuous tracking
    // Use setFrequencyOffset(0) to explicitly clear if needed
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

    return raw_bps * getCodeRateValue(rate);
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

int MCDPSKWaveform::getMinSamplesForFrame() const {
    int training_samples = config_.training_symbols * config_.samples_per_symbol;
    int ref_samples = config_.samples_per_symbol;
    return training_samples + ref_samples + getDataOnlySamplesForCWCount(1);
}

int MCDPSKWaveform::getMinSamplesForCWCount(int num_cw) const {
    // Training + reference + exactly the packed data symbols for num_cw LDPC CWs.
    // The modulator packs the whole frame bitstream before rounding to a symbol
    // boundary; rounding each CW independently drifts the cursor in continuous
    // MC-DPSK bursts.
    int training_samples = config_.training_symbols * config_.samples_per_symbol;
    int ref_samples = config_.samples_per_symbol;
    return training_samples + ref_samples + getDataOnlySamplesForCWCount(num_cw);
}

int MCDPSKWaveform::getDataOnlySamplesForCWCount(int num_cw) const {
    num_cw = std::max(1, num_cw);
    constexpr int LDPC_BLOCK_SIZE = 648;
    int bits_per_symbol = std::max(1, config_.num_carriers * config_.bits_per_symbol);
    int data_bits = num_cw * LDPC_BLOCK_SIZE;
    int data_symbols = (data_bits + bits_per_symbol - 1) / bits_per_symbol;
    return data_symbols * config_.samples_per_symbol;
}

float MCDPSKWaveform::getFadingIndex() const {
    if (demodulator_) {
        return demodulator_->getFadingIndex();
    }
    return 0.0f;
}

bool MCDPSKWaveform::isFading() const {
    if (demodulator_) {
        return demodulator_->isFading();
    }
    return false;
}

} // namespace ultra
