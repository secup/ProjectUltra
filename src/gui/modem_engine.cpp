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
    modulator_ = std::make_unique<OFDMModulator>(config_);
    demodulator_ = std::make_unique<OFDMDemodulator>(config_);
}

ModemEngine::~ModemEngine() = default;

void ModemEngine::setConfig(const ModemConfig& config) {
    config_ = config;

    encoder_->setRate(config.code_rate);
    decoder_->setRate(config.code_rate);

    // Recreate modulator/demodulator with new config
    modulator_ = std::make_unique<OFDMModulator>(config_);
    demodulator_ = std::make_unique<OFDMDemodulator>(config_);

    reset();
}

std::vector<float> ModemEngine::transmit(const std::string& text) {
    Bytes data(text.begin(), text.end());
    return transmit(data);
}

std::vector<float> ModemEngine::transmit(const Bytes& data) {
    if (data.empty()) {
        return {};
    }

    // Step 1: LDPC encode
    Bytes encoded = encoder_->encode(data);

    // Step 2: Generate preamble
    Samples preamble = modulator_->generatePreamble();

    // Step 3: OFDM modulate
    Samples modulated = modulator_->modulate(encoded, config_.modulation);

    // Step 4: Combine preamble + data
    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());

    // Add preamble
    output.insert(output.end(), preamble.begin(), preamble.end());

    // Add modulated data
    output.insert(output.end(), modulated.begin(), modulated.end());

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

    // Now do the slow processing with rx_mutex_
    std::lock_guard<std::mutex> lock(rx_mutex_);

    LOG_MODEM(DEBUG, "pollRxAudio: got %zu new samples", new_samples.size());

    rx_sample_buffer_.insert(rx_sample_buffer_.end(), new_samples.begin(), new_samples.end());

    // Process buffer if we have enough samples
    processRxBuffer();
    return true;
}

void ModemEngine::processRxBuffer() {
    // Need at least one symbol worth of samples
    size_t symbol_samples = config_.getSymbolDuration();
    if (rx_sample_buffer_.size() < symbol_samples * 2) {
        LOG_MODEM(TRACE, "RX: Not enough samples: %zu < %zu",
                  rx_sample_buffer_.size(), symbol_samples * 2);
        return;
    }

    LOG_MODEM(DEBUG, "RX: Processing %zu samples...", rx_sample_buffer_.size());

    // Feed samples to demodulator (it maintains its own buffer)
    SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());
    bool frame_ready = demodulator_->process(span);

    LOG_MODEM(DEBUG, "RX: Demodulator returned frame_ready=%d", frame_ready);

    // Clear the samples we just passed - demodulator now owns them
    rx_sample_buffer_.clear();

    if (frame_ready) {
        // Get soft bits from demodulator
        auto soft_bits = demodulator_->getSoftBits();
        LOG_MODEM(DEBUG, "RX: Frame ready, got %zu soft bits", soft_bits.size());

        if (!soft_bits.empty()) {
            // LDPC decode
            LOG_MODEM(DEBUG, "RX: Attempting LDPC decode with %zu soft bits", soft_bits.size());
            Bytes decoded = decoder_->decodeSoft(soft_bits);
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

                    // Call callback if set
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
                ChannelQuality quality = demodulator_->getChannelQuality();
                stats_.snr_db = quality.snr_db;
                stats_.ber = quality.ber_estimate;
                stats_.synced = true;
            }
        }

        // Reset demodulator to search for next preamble
        demodulator_->reset();

        // Clear sync indicator after reset
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.synced = false;
        }
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
    return demodulator_->isSynced();
}

float ModemEngine::getCurrentSNR() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_.snr_db;
}

std::vector<std::complex<float>> ModemEngine::getConstellationSymbols() const {
    return demodulator_->getConstellationSymbols();
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

    demodulator_->reset();

    {
        std::lock_guard<std::mutex> lock2(stats_mutex_);
        stats_ = LoopbackStats{};
    }
}

} // namespace gui
} // namespace ultra
