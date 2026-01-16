#include "modem_engine.hpp"
#include "ultra/logging.hpp"
#include <cstring>
#include <algorithm>

namespace ultra {
namespace gui {

ModemEngine::ModemEngine() {
    config_ = presets::balanced();

    encoder_ = std::make_unique<LDPCEncoder>(config_.code_rate);

    // OFDM modulator uses config (TX modulation is determined per-frame)
    ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);

    // RX starts in disconnected state, so use BPSK R1/4 for link establishment
    // This will be switched to negotiated mode when setConnected(true) is called
    ModemConfig rx_config = config_;
    rx_config.modulation = Modulation::BPSK;
    rx_config.code_rate = CodeRate::R1_4;
    decoder_ = std::make_unique<LDPCDecoder>(rx_config.code_rate);
    ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

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
    LOG_MODEM(INFO, "setConfig called: new code_rate=%d, modulation=%d",
              static_cast<int>(config.code_rate), static_cast<int>(config.modulation));
    config_ = config;

    encoder_->setRate(config.code_rate);
    // BUG FIX: Don't change decoder rate here - it should stay at R1_4 for disconnected mode
    // The decoder rate should only change when setConnected(true) is called
    // decoder_->setRate(config.code_rate);  // DISABLED - causes rate mismatch!
    LOG_MODEM(INFO, "setConfig: encoder_rate=%d, decoder_rate=%d (decoder unchanged for disconnected)",
              static_cast<int>(config.code_rate), static_cast<int>(decoder_->getRate()));

    // Recreate OFDM modulator with new config (TX uses config directly)
    ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);

    // For RX: if disconnected, use BPSK R1/4 (robust mode), not config's mode
    if (!connected_) {
        ModemConfig rx_config = config_;
        rx_config.modulation = Modulation::BPSK;
        rx_config.code_rate = CodeRate::R1_4;
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);
        LOG_MODEM(INFO, "setConfig: RX using disconnected mode (BPSK R1/4)");
    } else {
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(config_);
        LOG_MODEM(INFO, "setConfig: RX using connected mode (config settings)");
    }

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

    // Determine if this is a link establishment frame
    // These must use robust modulation (BPSK R1/4) to get through at any SNR
    // Frame format: [MAGIC:1][TYPE:1][...]
    bool is_link_frame = false;
    if (data.size() >= 2 && data[0] == protocol::Frame::MAGIC) {
        uint8_t frame_type = data[1];
        // Link establishment frames that must work in poor conditions
        is_link_frame = (frame_type == static_cast<uint8_t>(protocol::FrameType::PROBE) ||
                         frame_type == static_cast<uint8_t>(protocol::FrameType::PROBE_ACK) ||
                         frame_type == static_cast<uint8_t>(protocol::FrameType::CONNECT) ||
                         frame_type == static_cast<uint8_t>(protocol::FrameType::CONNECT_ACK) ||
                         frame_type == static_cast<uint8_t>(protocol::FrameType::CONNECT_NAK) ||
                         frame_type == static_cast<uint8_t>(protocol::FrameType::BEACON));
    }

    // Select modulation and code rate
    Modulation tx_modulation;
    CodeRate tx_code_rate;

    if (is_link_frame) {
        // Link establishment: Always use most robust mode (works down to ~5 dB SNR)
        tx_modulation = Modulation::BPSK;
        tx_code_rate = CodeRate::R1_4;
    } else if (connected_) {
        // Connected: Use negotiated mode (from probing/adaptive)
        tx_modulation = data_modulation_;
        tx_code_rate = data_code_rate_;
    } else {
        // Fallback: Use robust mode
        tx_modulation = Modulation::BPSK;
        tx_code_rate = CodeRate::R1_4;
    }

    // Adaptive modulation: update data mode based on measured SNR
    // (Only affects data frames, not link establishment)
    if (connected_ && config_.speed_profile == SpeedProfile::ADAPTIVE) {
        float snr = getCurrentSNR();
        if (adaptive_.update(snr)) {
            // Mode changed - update data mode
            data_modulation_ = adaptive_.getModulation();
            data_code_rate_ = adaptive_.getCodeRate();
            if (!is_link_frame) {
                tx_modulation = data_modulation_;
                tx_code_rate = data_code_rate_;
            }
        }
    }

    // Step 1: LDPC encode with appropriate code rate
    encoder_->setRate(tx_code_rate);
    LOG_MODEM(INFO, "TX: Using code_rate=%d, modulation=%d, is_link=%d, connected=%d",
              static_cast<int>(tx_code_rate), static_cast<int>(tx_modulation),
              is_link_frame, connected_);
    Bytes encoded = encoder_->encode(data);

    // Step 1.5: Interleave (optional - spreads burst errors for LDPC decoder)
    Bytes to_modulate = interleaving_enabled_ ? interleaver_.interleave(encoded) : encoded;

    // Step 2: Generate preamble
    Samples preamble = ofdm_modulator_->generatePreamble();

    // Step 3: OFDM modulate with appropriate modulation
    Samples modulated = ofdm_modulator_->modulate(to_modulate, tx_modulation);

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

    LOG_MODEM(DEBUG, "pollRxAudio: got %zu new samples, buffer=%zu", new_samples.size(), rx_sample_buffer_.size());

    rx_sample_buffer_.insert(rx_sample_buffer_.end(), new_samples.begin(), new_samples.end());

    // Process buffer if we have enough samples
    processRxBuffer();
    return true;
}

void ModemEngine::processRxBuffer() {
    // Need at least one symbol worth of samples (or demodulator has pending data)
    size_t symbol_samples = config_.getSymbolDuration();
    if (rx_sample_buffer_.size() < symbol_samples * 2 && !ofdm_demodulator_->hasPendingData()) {
        // Log when we skip processing (at TRACE level to avoid spam)
        if (rx_sample_buffer_.size() > 0) {
            LOG_MODEM(TRACE, "RX: Buffering %zu samples (need %zu)",
                      rx_sample_buffer_.size(), symbol_samples * 2);
        }
        return;
    }

    LOG_MODEM(DEBUG, "RX: Processing %zu samples (synced=%d)...", rx_sample_buffer_.size(), ofdm_demodulator_->isSynced());

    // Feed samples to demodulator (it maintains its own buffer)
    SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());

    // Clear now - demodulator takes ownership
    rx_sample_buffer_.clear();

    // Loop to process all available codewords (multi-codeword frames)
    bool frame_ready = ofdm_demodulator_->process(span);
    LOG_MODEM(DEBUG, "RX: Demod returned frame_ready=%d, synced=%d", frame_ready, ofdm_demodulator_->isSynced());

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
        LOG_MODEM(INFO, "RX: Decoding %d codewords (%zu soft bits total), decoder_rate=%d",
                  codewords_collected, accumulated_soft_bits.size(),
                  static_cast<int>(decoder_->getRate()));

        Bytes decoded = decoder_->decodeSoft(accumulated_soft_bits);
        bool success = decoder_->lastDecodeSuccess();

        LOG_MODEM(INFO, "RX: LDPC decode %s, got %zu bytes",
                  success ? "SUCCESS" : "FAILED", decoded.size());

        // Print first few bytes as hex (always at INFO for debugging)
        if (!decoded.empty()) {
            char hex_buf[80], text_buf[32];
            size_t n = std::min(decoded.size(), size_t(24));
            for (size_t i = 0; i < n; i++) {
                snprintf(hex_buf + i*3, 4, "%02x ", decoded[i]);
                char c = decoded[i];
                text_buf[i] = (c >= 32 && c < 127) ? c : '.';
            }
            text_buf[n] = '\0';
            LOG_MODEM(INFO, "RX: First %zu bytes hex: %s", n, hex_buf);
            LOG_MODEM(INFO, "RX: As text: '%s'", text_buf);
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
                    LOG_MODEM(DEBUG, "RX: Calling raw_data_callback with %zu bytes", decoded.size());
                    raw_data_callback_(decoded);
                } else {
                    LOG_MODEM(WARN, "RX: No raw_data_callback set!");
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

// === Waveform Mode Control ===

// Helper to get mode description string
static const char* getModeDescription(Modulation mod, CodeRate rate) {
    switch (mod) {
        case Modulation::BPSK:
            return rate == CodeRate::R1_4 ? "BPSK R1/4" : "BPSK R1/2";
        case Modulation::QPSK:
            return rate == CodeRate::R1_2 ? "QPSK R1/2" : "QPSK R2/3";
        case Modulation::QAM16:
            return rate == CodeRate::R2_3 ? "16-QAM R2/3" : "16-QAM R3/4";
        case Modulation::QAM64:
            return rate == CodeRate::R3_4 ? "64-QAM R3/4" : "64-QAM R5/6";
        default:
            return "Unknown";
    }
}

void ModemEngine::setWaveformMode(protocol::WaveformMode mode) {
    waveform_mode_ = mode;
    // Future: Switch between OFDM and OTFS modulator/demodulator here
}

void ModemEngine::setConnected(bool connected) {
    if (connected_ == connected) return;

    connected_ = connected;

    if (connected) {
        // Switching to connected state - use negotiated data mode for RX
        // Create new config with data mode settings
        ModemConfig rx_config = config_;
        rx_config.modulation = data_modulation_;
        rx_config.code_rate = data_code_rate_;

        // Update decoder rate
        decoder_->setRate(data_code_rate_);

        // Recreate demodulator with new config (resets sync state)
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

        LOG_MODEM(INFO, "Switched to connected mode: %s (RX: %d-ary, R%d)",
                  getModeDescription(data_modulation_, data_code_rate_),
                  1 << static_cast<int>(data_modulation_),
                  static_cast<int>(data_code_rate_));
    } else {
        // Switching to disconnected state - use robust mode for RX
        ModemConfig rx_config = config_;
        rx_config.modulation = Modulation::BPSK;
        rx_config.code_rate = CodeRate::R1_4;

        decoder_->setRate(CodeRate::R1_4);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

        LOG_MODEM(INFO, "Switched to disconnected mode (RX: BPSK R1/4)");
    }
}

void ModemEngine::setDataMode(Modulation mod, CodeRate rate) {
    data_modulation_ = mod;
    data_code_rate_ = rate;

    // If already connected, update RX immediately
    if (connected_) {
        ModemConfig rx_config = config_;
        rx_config.modulation = mod;
        rx_config.code_rate = rate;

        decoder_->setRate(rate);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);
    }

    LOG_MODEM(INFO, "Data mode set to: %s", getModeDescription(mod, rate));
}

void ModemEngine::recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate) {
    // Thresholds based on README performance targets (actual channel SNR)
    // These match the Speed table in documentation
    if (snr_db >= 25.0f) {
        mod = Modulation::QAM64;
        rate = CodeRate::R5_6;  // Excellent: ~10 kbps
    } else if (snr_db >= 20.0f) {
        mod = Modulation::QAM16;
        rate = CodeRate::R3_4;  // Good: ~6 kbps
    } else if (snr_db >= 15.0f) {
        mod = Modulation::QPSK;
        rate = CodeRate::R1_2;  // Moderate: ~2 kbps
    } else if (snr_db >= 10.0f) {
        mod = Modulation::BPSK;
        rate = CodeRate::R1_2;  // Poor: ~1 kbps
    } else {
        mod = Modulation::BPSK;
        rate = CodeRate::R1_4;  // Flutter: ~0.5 kbps
    }
}

} // namespace gui
} // namespace ultra
