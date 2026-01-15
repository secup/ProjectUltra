#include "modem_engine.hpp"
#include "ultra/logging.hpp"
#include <cstring>
#include <algorithm>

namespace ultra {
namespace gui {

ModemEngine::ModemEngine() {
    config_ = presets::balanced();

    encoder_ = std::make_unique<LDPCEncoder>(config_.code_rate);
    decoder_ = std::make_unique<LDPCDecoder>(config_.code_rate);

    // OFDM modulator/demodulator (always available - used for control frames)
    ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);
    ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(config_);

    // OTFS modulator/demodulator (used for data frames when negotiated)
    otfs_config_.M = config_.num_carriers;
    otfs_config_.N = 16;  // 16 OFDM symbols per OTFS frame
    otfs_config_.fft_size = config_.fft_size;
    otfs_config_.cp_length = config_.getCyclicPrefix();
    otfs_config_.sample_rate = config_.sample_rate;
    otfs_config_.center_freq = config_.center_freq;
    otfs_modulator_ = std::make_unique<OTFSModulator>(otfs_config_);
    otfs_demodulator_ = std::make_unique<OTFSDemodulator>(otfs_config_);

    // Initialize audio filters
    rebuildFilters();
}

ModemEngine::~ModemEngine() = default;

void ModemEngine::setConfig(const ModemConfig& config) {
    config_ = config;

    encoder_->setRate(config.code_rate);
    decoder_->setRate(config.code_rate);

    // Recreate OFDM modulator/demodulator with new config
    ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);
    ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(config_);

    // Recreate OTFS modulator/demodulator with new config
    otfs_config_.M = config_.num_carriers;
    otfs_config_.fft_size = config_.fft_size;
    otfs_config_.cp_length = config_.getCyclicPrefix();
    otfs_config_.sample_rate = config_.sample_rate;
    otfs_config_.center_freq = config_.center_freq;
    otfs_modulator_ = std::make_unique<OTFSModulator>(otfs_config_);
    otfs_demodulator_ = std::make_unique<OTFSDemodulator>(otfs_config_);

    // Rebuild filters with new sample rate
    rebuildFilters();

    reset();
}

void ModemEngine::setFilterConfig(const FilterConfig& config) {
    filter_config_ = config;
    rebuildFilters();
}

void ModemEngine::setFilterEnabled(bool enabled) {
    filter_config_.enabled = enabled;
}

void ModemEngine::rebuildFilters() {
    // Create bandpass filters for TX and RX
    // Use separate instances so they maintain independent state
    float sample_rate = static_cast<float>(config_.sample_rate);
    float low = filter_config_.lowFreq();
    float high = filter_config_.highFreq();
    int taps = filter_config_.taps;

    // Ensure valid frequency range
    low = std::max(50.0f, low);
    high = std::min(sample_rate / 2.0f - 50.0f, high);

    if (low < high) {
        auto filter = FIRFilter::bandpass(taps, low, high, sample_rate);
        tx_filter_ = std::make_unique<FIRFilter>(filter);
        rx_filter_ = std::make_unique<FIRFilter>(filter);
        LOG_MODEM(INFO, "Audio filters configured: %.0f-%.0f Hz, %d taps",
                  low, high, taps);
    } else {
        LOG_MODEM(WARN, "Invalid filter range: %.0f-%.0f Hz, filters disabled",
                  low, high);
        tx_filter_.reset();
        rx_filter_.reset();
    }
}

std::vector<float> ModemEngine::transmit(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return transmit(data);
}

std::vector<float> ModemEngine::transmit(const Bytes& data) {
    if (data.empty()) {
        return {};
    }

    // Adaptive modulation: adjust mode based on measured SNR
    if (config_.speed_profile == SpeedProfile::ADAPTIVE) {
        float snr = getCurrentSNR();
        if (adaptive_.update(snr)) {
            // Mode changed - update config
            config_.modulation = adaptive_.getModulation();
            config_.code_rate = adaptive_.getCodeRate();
            encoder_->setRate(config_.code_rate);
            // Note: modulator uses modulation per-call, no recreation needed
        }
    }

    // Step 1: LDPC encode
    Bytes encoded = encoder_->encode(data);

    // Step 1.5: Interleave (optional - spreads burst errors for LDPC decoder)
    Bytes to_modulate = interleaving_enabled_ ? interleaver_.interleave(encoded) : encoded;

    // Step 2: Generate preamble
    Samples preamble = ofdm_modulator_->generatePreamble();

    // Step 3: OFDM modulate
    Samples modulated = ofdm_modulator_->modulate(to_modulate, config_.modulation);

    // Step 4: Combine preamble + data
    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());

    // Add preamble
    output.insert(output.end(), preamble.begin(), preamble.end());

    // Add modulated data
    output.insert(output.end(), modulated.begin(), modulated.end());

    // Apply TX bandpass filter (before scaling to maintain filter characteristics)
    if (filter_config_.enabled && tx_filter_) {
        SampleSpan span(output.data(), output.size());
        output = tx_filter_->process(span);
    }

    // Scale for audio output (prevent clipping)
    float max_val = 0.0f;
    for (float s : output) {
        max_val = std::max(max_val, std::abs(s));
    }
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;  // Leave some headroom
        for (float& s : output) {
            s *= scale;
        }
    }

    // Update stats
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_sent++;

        // Calculate throughput
        int bits_per_carrier = static_cast<int>(config_.modulation);
        float code_rate = getCodeRateValue(config_.code_rate);
        float symbol_rate = config_.sample_rate / (float)config_.getSymbolDuration();
        stats_.throughput_bps = static_cast<int>(
            config_.getDataCarriers() * bits_per_carrier * code_rate * symbol_rate
        );
    }

    return output;
}

void ModemEngine::receiveAudio(const std::vector<float>& samples) {
    // FAST PATH: Only lock the pending mutex, just append samples
    // This is safe to call from SDL audio callback
    std::lock_guard<std::mutex> lock(rx_pending_mutex_);
    rx_pending_samples_.insert(rx_pending_samples_.end(), samples.begin(), samples.end());
}

bool ModemEngine::pollRxAudio() {
    // SLOW PATH: Move pending samples to main buffer and process
    // Call this from the main loop, NOT from audio callback!

    // First, quickly grab any pending samples
    std::vector<float> new_samples;
    {
        std::lock_guard<std::mutex> lock(rx_pending_mutex_);
        if (!rx_pending_samples_.empty()) {
            new_samples = std::move(rx_pending_samples_);
            rx_pending_samples_.clear();
            rx_pending_samples_.reserve(4096);  // Pre-allocate for next batch
        }
    }

    if (new_samples.empty()) {
        return false;  // Nothing to process
    }

    // Apply RX bandpass filter
    if (filter_config_.enabled && rx_filter_) {
        SampleSpan span(new_samples.data(), new_samples.size());
        new_samples = rx_filter_->process(span);
    }

    // Update carrier sense energy detection
    updateChannelEnergy(new_samples);

    // Now do the slow processing with rx_mutex_
    std::lock_guard<std::mutex> lock(rx_mutex_);

    LOG_MODEM(DEBUG, "pollRxAudio: got %zu new samples", new_samples.size());

    rx_sample_buffer_.insert(rx_sample_buffer_.end(), new_samples.begin(), new_samples.end());

    // Process buffer if we have enough samples
    processRxBuffer();
    return true;
}

void ModemEngine::processRxBuffer() {
    // Need at least one symbol worth of samples (or demodulator has pending data)
    size_t symbol_samples = config_.getSymbolDuration();
    if (rx_sample_buffer_.size() < symbol_samples * 2 && !ofdm_demodulator_->hasPendingData()) {
        LOG_MODEM(TRACE, "RX: Not enough samples: %zu < %zu",
                  rx_sample_buffer_.size(), symbol_samples * 2);
        return;
    }

    LOG_MODEM(DEBUG, "RX: Processing %zu samples...", rx_sample_buffer_.size());

    // Feed samples to demodulator (it maintains its own buffer)
    SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());

    // Clear now - demodulator takes ownership
    rx_sample_buffer_.clear();

    // Loop to process all available codewords (multi-codeword frames)
    bool frame_ready = ofdm_demodulator_->process(span);

    // Update SNR continuously when synced (not just after decode)
    if (ofdm_demodulator_->isSynced()) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.snr_db = ofdm_demodulator_->getEstimatedSNR();
        stats_.synced = true;
    }

    // CRITICAL: Accumulate ALL soft bits from all codewords of this frame FIRST,
    // then decode once. This is necessary because k=486 info bits per codeword
    // doesn't align on byte boundaries (486 bits = 60.75 bytes). Decoding
    // codeword-by-codeword and concatenating bytes causes corruption at boundaries.
    std::vector<float> accumulated_soft_bits;
    int codewords_collected = 0;

    while (frame_ready) {
        // Get soft bits from demodulator (one codeword worth = n bits)
        auto soft_bits = ofdm_demodulator_->getSoftBits();
        LOG_MODEM(DEBUG, "RX: Codeword ready, got %zu soft bits", soft_bits.size());

        if (!soft_bits.empty()) {
            codewords_collected++;
            // Deinterleave if enabled (reverses TX interleaving) - per codeword
            auto to_accumulate = interleaving_enabled_ ? interleaver_.deinterleave(soft_bits) : soft_bits;

            // Accumulate soft bits
            accumulated_soft_bits.insert(accumulated_soft_bits.end(),
                                         to_accumulate.begin(), to_accumulate.end());
            LOG_MODEM(DEBUG, "RX: Accumulated %zu soft bits (total: %zu)",
                      to_accumulate.size(), accumulated_soft_bits.size());
        }

        // Try to get next codeword from demodulator's internal buffer
        SampleSpan empty_span;
        frame_ready = ofdm_demodulator_->process(empty_span);
    }

    // Now decode all accumulated soft bits at once (handles bit-level boundaries correctly)
    if (!accumulated_soft_bits.empty()) {
        LOG_MODEM(DEBUG, "RX: Decoding %d codewords (%zu soft bits total)",
                  codewords_collected, accumulated_soft_bits.size());

        Bytes decoded = decoder_->decodeSoft(accumulated_soft_bits);
        bool success = decoder_->lastDecodeSuccess();

        LOG_MODEM(DEBUG, "RX: LDPC decode %s, got %zu bytes",
                  success ? "SUCCESS" : "FAILED", decoded.size());

        // Debug: print first few bytes as hex and text
        if (g_log_level >= LogLevel::DEBUG && g_log_categories.modem && !decoded.empty()) {
            char hex_buf[80], text_buf[32];
            size_t n = std::min(decoded.size(), size_t(24));
            for (size_t i = 0; i < n; i++) {
                snprintf(hex_buf + i*3, 4, "%02x ", decoded[i]);
                char c = decoded[i];
                text_buf[i] = (c >= 32 && c < 127) ? c : '.';
            }
            text_buf[n] = '\0';
            LOG_MODEM(DEBUG, "RX: First %zu bytes hex: %s", n, hex_buf);
            LOG_MODEM(DEBUG, "RX: As text: '%s'", text_buf);
        }

        // Update stats
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            if (success) {
                stats_.frames_received++;

                // Queue the received data
                rx_data_queue_.push(decoded);

                // Call raw callback first (for protocol layer)
                if (raw_data_callback_) {
                    raw_data_callback_(decoded);
                }

                // Call text callback (filtered for display)
                if (data_callback_) {
                    std::string text(decoded.begin(), decoded.end());
                    // Remove null characters and non-printable
                    text.erase(std::remove_if(text.begin(), text.end(),
                        [](char c) { return c == '\0' || !isprint(c); }), text.end());
                    if (!text.empty()) {
                        data_callback_(text);
                    }
                }
            } else {
                stats_.frames_failed++;
            }

            // Update channel quality
            ChannelQuality quality = ofdm_demodulator_->getChannelQuality();
            stats_.snr_db = quality.snr_db;
            stats_.ber = quality.ber_estimate;
            stats_.synced = true;
        }

        LOG_MODEM(DEBUG, "RX: Decoded %d codewords -> %zu bytes", codewords_collected, decoded.size());
    }
}

bool ModemEngine::hasReceivedData() const {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return !rx_data_queue_.empty();
}

std::string ModemEngine::getReceivedText() {
    Bytes data = getReceivedData();
    std::string text(data.begin(), data.end());
    // Clean up null characters
    text.erase(std::remove(text.begin(), text.end(), '\0'), text.end());
    return text;
}

Bytes ModemEngine::getReceivedData() {
    std::lock_guard<std::mutex> lock(rx_mutex_);

    if (rx_data_queue_.empty()) {
        return {};
    }

    Bytes data = rx_data_queue_.front();
    rx_data_queue_.pop();
    return data;
}

LoopbackStats ModemEngine::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

bool ModemEngine::isSynced() const {
    // Query demodulator's actual state, not cached value
    return ofdm_demodulator_->isSynced();
}

float ModemEngine::getCurrentSNR() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_.snr_db;
}

ChannelQuality ModemEngine::getChannelQuality() const {
    // Get channel quality from the demodulator
    // This provides SNR, delay spread, Doppler spread, and BER estimate
    return ofdm_demodulator_->getChannelQuality();
}

std::vector<std::complex<float>> ModemEngine::getConstellationSymbols() const {
    return ofdm_demodulator_->getConstellationSymbols();
}

void ModemEngine::reset() {
    // Clear pending samples first
    {
        std::lock_guard<std::mutex> lock(rx_pending_mutex_);
        rx_pending_samples_.clear();
    }

    std::lock_guard<std::mutex> lock(rx_mutex_);

    rx_sample_buffer_.clear();
    std::queue<Bytes> empty;
    std::swap(rx_data_queue_, empty);

    ofdm_demodulator_->reset();
    adaptive_.reset();

    // Reset carrier sense
    channel_energy_.store(0.0f);

    {
        std::lock_guard<std::mutex> lock2(stats_mutex_);
        stats_ = LoopbackStats{};
    }
}

// === Carrier Sense (Listen Before Talk) ===

void ModemEngine::updateChannelEnergy(const std::vector<float>& samples) {
    if (samples.empty()) return;

    // Calculate RMS energy of samples
    float sum_sq = 0.0f;
    for (float s : samples) {
        sum_sq += s * s;
    }
    float rms = std::sqrt(sum_sq / samples.size());

    // Smooth the energy estimate (exponential moving average)
    float current = channel_energy_.load();
    float smoothed = ENERGY_SMOOTHING * rms + (1.0f - ENERGY_SMOOTHING) * current;
    channel_energy_.store(smoothed);
}

bool ModemEngine::isChannelBusy() const {
    // Channel is busy if energy is above threshold (someone is transmitting)
    // Note: We don't check isSynced() here because:
    // 1. Sync state persists after decode (would block response TX)
    // 2. Energy detection is the true carrier sense - if there's RF energy, channel is busy
    // 3. Protocol layer handles half-duplex timing separately
    return channel_energy_.load() > carrier_sense_threshold_;
}

float ModemEngine::getChannelEnergy() const {
    return channel_energy_.load();
}

void ModemEngine::setCarrierSenseThreshold(float threshold) {
    carrier_sense_threshold_ = std::max(0.0f, std::min(1.0f, threshold));
}

float ModemEngine::getCarrierSenseThreshold() const {
    return carrier_sense_threshold_;
}

} // namespace gui
} // namespace ultra
