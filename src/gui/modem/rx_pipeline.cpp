// RxPipeline - Implementation of streaming RX decode flow

#include "rx_pipeline.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <algorithm>

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;

RxPipeline::RxPipeline(const std::string& log_prefix)
    : log_prefix_(log_prefix)
{
    // Create default interleaver (30 carriers × 2 bits DQPSK = 60 bits/symbol)
    interleaver_ = std::make_unique<ChannelInterleaver>(60, v2::LDPC_CODEWORD_BITS);
}

void RxPipeline::setDataMode(CodeRate rate, bool connected) {
    data_code_rate_ = rate;
    connected_ = connected;
}

void RxPipeline::setInterleaverConfig(size_t bits_per_symbol) {
    if (bits_per_symbol != interleaver_bits_per_symbol_) {
        interleaver_bits_per_symbol_ = bits_per_symbol;
        interleaver_ = std::make_unique<ChannelInterleaver>(bits_per_symbol, v2::LDPC_CODEWORD_BITS);
        LOG_MODEM(INFO, "[%s] RxPipeline: Interleaver configured for %zu bits/symbol",
                  log_prefix_.c_str(), bits_per_symbol);
    }
}

void RxPipeline::reset() {
    accumulated_soft_bits_.clear();
    expected_codewords_ = 0;
    if (waveform_) {
        waveform_->reset();
    }
}

void RxPipeline::clearBuffer() {
    rx_buffer_.clear();
    reset();
}

int RxPipeline::getAccumulatedCodewords() const {
    return static_cast<int>(accumulated_soft_bits_.size() / v2::LDPC_CODEWORD_BITS);
}

// ============================================================================
// STREAMING INTERFACE
// ============================================================================

void RxPipeline::feedAudio(const float* samples, size_t count) {
    if (!samples || count == 0) return;

    // Add samples to buffer
    rx_buffer_.insert(rx_buffer_.end(), samples, samples + count);

    // Limit buffer size to prevent memory issues
    if (rx_buffer_.size() > MAX_BUFFER_SIZE) {
        size_t excess = rx_buffer_.size() - MAX_BUFFER_SIZE;
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + excess);
        LOG_MODEM(WARN, "[%s] RxPipeline: Buffer overflow, trimmed %zu samples",
                  log_prefix_.c_str(), excess);
    }

    // Process one frame per feedAudio call to avoid potential loops
    // The caller will call feedAudio() again when more data arrives
    tryProcessBuffer();
}

bool RxPipeline::hasFrame() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return !frame_queue_.empty();
}

RxFrameResult RxPipeline::getFrame() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (frame_queue_.empty()) {
        return RxFrameResult{};
    }
    RxFrameResult result = std::move(frame_queue_.front());
    frame_queue_.pop();
    return result;
}

size_t RxPipeline::getFrameCount() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return frame_queue_.size();
}

size_t RxPipeline::getBufferSize() const {
    return rx_buffer_.size();
}

bool RxPipeline::tryProcessBuffer() {
    if (!waveform_) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: No waveform configured", log_prefix_.c_str());
        return false;
    }

    // Need minimum samples for sync detection
    size_t preamble_samples = waveform_->getPreambleSamples();
    size_t symbol_samples = waveform_->getSamplesPerSymbol();
    size_t min_for_sync = preamble_samples + 2 * symbol_samples;

    if (rx_buffer_.size() < min_for_sync) {
        return false;
    }

    // Try to detect sync
    SampleSpan buffer_span(rx_buffer_.data(), rx_buffer_.size());
    SyncResult sync_result;
    bool sync_found = waveform_->detectSync(buffer_span, sync_result, 0.15f);

    if (!sync_found) {
        // No sync - trim old samples to prevent unbounded growth
        // Keep enough for potential sync at end of buffer
        if (rx_buffer_.size() > min_for_sync * 2) {
            size_t trim = rx_buffer_.size() - min_for_sync;
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + trim);
        }
        return false;
    }

    LOG_MODEM(INFO, "[%s] RxPipeline: Sync detected at %d, CFO=%.1f Hz, corr=%.3f",
              log_prefix_.c_str(), sync_result.start_sample, sync_result.cfo_hz,
              sync_result.correlation);

    // Apply CFO correction (persists across reset for continuous tracking)
    waveform_->setFrequencyOffset(sync_result.cfo_hz);

    // sync_result.start_sample points directly to where data starts
    // Each waveform's detectSync() calculates this (chirp + gap + training for OFDM,
    // chirp + gap + training + ref for MC-DPSK)
    int data_start = sync_result.start_sample;

    if (data_start < 0) {
        return false;
    }

    // Check if we have enough samples for at least one frame
    // Use waveform-specific calculation: training + ref + data for 1 codeword
    size_t min_frame_samples = static_cast<size_t>(waveform_->getMinSamplesForFrame());
    if (static_cast<size_t>(data_start) + min_frame_samples > rx_buffer_.size()) {
        // Not enough samples yet - wait for more
        return false;
    }

    // Process from data start
    size_t process_len = rx_buffer_.size() - data_start;
    SampleSpan process_span(rx_buffer_.data() + data_start, process_len);

    waveform_->reset();
    bool frame_ready = waveform_->process(process_span);

    if (!frame_ready) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: process() returned false", log_prefix_.c_str());
        // Consume sync position and continue
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sync_result.start_sample + symbol_samples);
        waveform_->reset();
        return false;
    }

    // Get soft bits
    auto soft_bits = waveform_->getSoftBits();
    if (soft_bits.empty()) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: No soft bits from waveform", log_prefix_.c_str());
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sync_result.start_sample + symbol_samples);
        waveform_->reset();
        return false;
    }

    // Build result
    RxFrameResult result;
    result.snr_estimate = waveform_->estimatedSNR();
    result.cfo_estimate = sync_result.cfo_hz;

    // Check for PING first
    if (detectPing(soft_bits)) {
        result.success = true;
        result.is_ping = true;
        result.frame_type = v2::FrameType::PING;

        // Queue the result
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            frame_queue_.push(result);
        }

        if (ping_callback_) {
            ping_callback_(result.snr_estimate);
        }

        // Consume processed samples: start_sample + minimal data for PING
        size_t consume = sync_result.start_sample + symbol_samples * 2;  // Just past the PING data
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + std::min(consume, rx_buffer_.size()));
        waveform_->reset();
        return true;
    }

    // Deinterleave if enabled
    if (interleaving_enabled_ && interleaver_) {
        // Process one codeword at a time
        std::vector<float> deinterleaved;
        for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size(); i += v2::LDPC_CODEWORD_BITS) {
            std::vector<float> cw_bits(soft_bits.begin() + i, soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
            auto di = interleaver_->deinterleave(cw_bits);
            deinterleaved.insert(deinterleaved.end(), di.begin(), di.end());
        }
        soft_bits = std::move(deinterleaved);
    }

    // Count codewords
    int num_codewords = static_cast<int>(soft_bits.size() / v2::LDPC_CODEWORD_BITS);
    if (num_codewords == 0) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: Insufficient soft bits for 1 codeword", log_prefix_.c_str());
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sync_result.start_sample + symbol_samples);
        waveform_->reset();
        return false;
    }

    // Decode frame
    result = decodeFrame(soft_bits, num_codewords);
    result.cfo_estimate = sync_result.cfo_hz;

    // Queue result (even if failed - caller can check success flag)
    if (result.success || result.codewords_ok > 0) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            frame_queue_.push(result);
        }

        if (result.success && frame_callback_) {
            frame_callback_(result.frame_data, result.frame_type);
        }
    }

    // Consume samples: start_sample points to training start (after preamble)
    // So we need to consume: start_sample + frame data (training + data symbols)
    // Use min_frame_samples which includes training + ref + data for 1 codeword
    size_t consume = sync_result.start_sample + min_frame_samples;
    consume = std::min(consume, rx_buffer_.size());
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + consume);

    LOG_MODEM(INFO, "[%s] RxPipeline: Consumed %zu samples (start=%d + frame=%zu), buffer now %zu",
              log_prefix_.c_str(), consume, sync_result.start_sample, min_frame_samples, rx_buffer_.size());

    waveform_->reset();
    return true;
}

// ============================================================================
// LEGACY INTERFACE
// ============================================================================

RxFrameResult RxPipeline::processFrame(const DetectedFrame& frame,
                                        SampleSpan samples,
                                        IWaveform* waveform) {
    RxFrameResult result;

    if (!waveform) {
        LOG_MODEM(ERROR, "[%s] RxPipeline: null waveform", log_prefix_.c_str());
        return result;
    }

    // Configure waveform with CFO from detection
    if (std::abs(frame.cfo_hz) > 0.1f) {
        waveform->setFrequencyOffset(frame.cfo_hz);
        LOG_MODEM(INFO, "[%s] RxPipeline: Applying CFO=%.1f Hz from detection",
                  log_prefix_.c_str(), frame.cfo_hz);
    }

    // Process samples through waveform demodulator
    waveform->reset();
    if (!waveform->process(samples)) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: waveform->process() returned false",
                  log_prefix_.c_str());
        // May still have data from previous processing
    }

    // Get soft bits
    auto soft_bits = waveform->getSoftBits();
    if (soft_bits.empty()) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: No soft bits from waveform", log_prefix_.c_str());
        return result;
    }

    result.snr_estimate = waveform->estimatedSNR();
    result.cfo_estimate = waveform->estimatedCFO();

    // First check for PING (raw bytes, no LDPC)
    if (detectPing(soft_bits)) {
        result.success = true;
        result.is_ping = true;
        result.frame_type = v2::FrameType::PING;

        if (ping_callback_) {
            ping_callback_(result.snr_estimate);
        }
        return result;
    }

    // Deinterleave if enabled
    if (interleaving_enabled_) {
        soft_bits = deinterleaveCodewords(soft_bits);
    }

    // Count available codewords
    int num_codewords = static_cast<int>(soft_bits.size() / v2::LDPC_CODEWORD_BITS);
    if (num_codewords == 0) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: Insufficient soft bits for 1 codeword",
                  log_prefix_.c_str());
        return result;
    }

    // Decode frame
    result = decodeFrame(soft_bits, num_codewords);

    // Deliver if successful
    if (result.success && frame_callback_) {
        frame_callback_(result.frame_data, result.frame_type);
    }

    return result;
}

RxFrameResult RxPipeline::decodeFrame(const std::vector<float>& soft_bits, int num_codewords) {
    RxFrameResult result;
    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < LDPC_BLOCK) {
        return result;
    }

    // ========================================================================
    // CODE RATE SELECTION (Protocol v2 Rules):
    // ========================================================================
    // - Before connection (connected_ == false): Always R1/4
    //   CONNECT/CONNECT_ACK are sent via MC-DPSK R1/4 for robust initial contact
    // - After connection (connected_ == true): Use negotiated rate for ALL codewords
    //   The negotiated rate from CONNECT_ACK applies to CW0 and CW1+ equally
    // ========================================================================
    CodeRate frame_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
    size_t bytes_per_cw = v2::getBytesPerCodeword(frame_rate);

    // Decode CW0 with the appropriate rate
    std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
    Bytes cw0_data;
    if (!decodeSingleCodeword(cw0_bits, frame_rate, bytes_per_cw, cw0_data)) {
        LOG_MODEM(INFO, "[%s] RxPipeline: CW0 LDPC decode failed (rate=%s)",
                  log_prefix_.c_str(), codeRateToString(frame_rate));
        result.codewords_failed++;
        return result;
    }

    result.codewords_ok++;

    // Parse header
    auto header = parseHeader(cw0_data);
    if (!header.valid) {
        LOG_MODEM(WARN, "[%s] RxPipeline: Invalid header in CW0", log_prefix_.c_str());
        return result;
    }

    result.frame_type = header.frame_type;
    int expected = header.total_cw;

    LOG_MODEM(INFO, "[%s] RxPipeline: CW0 OK, expecting %d codewords, type=%d, rate=%s",
              log_prefix_.c_str(), expected, static_cast<int>(header.frame_type),
              codeRateToString(frame_rate));

    // Check if we have all codewords
    if (num_codewords < expected) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: Waiting for more codewords (%d/%d)",
                  log_prefix_.c_str(), num_codewords, expected);
        // Store for accumulation
        expected_codewords_ = expected;
        accumulated_soft_bits_ = soft_bits;
        return result;
    }

    // Decode remaining codewords (all use the same rate as CW0)
    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(expected, false);
    cw_status.data.resize(expected);
    cw_status.decoded[0] = true;
    cw_status.data[0] = cw0_data;

    for (int i = 1; i < expected; i++) {
        std::vector<float> cw_bits(
            soft_bits.begin() + i * LDPC_BLOCK,
            soft_bits.begin() + (i + 1) * LDPC_BLOCK
        );

        Bytes cw_data;
        if (decodeSingleCodeword(cw_bits, frame_rate, bytes_per_cw, cw_data)) {
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            result.codewords_ok++;
            LOG_MODEM(DEBUG, "[%s] CW%d: LDPC OK", log_prefix_.c_str(), i);
        } else {
            result.codewords_failed++;
            LOG_MODEM(DEBUG, "[%s] CW%d: LDPC FAIL", log_prefix_.c_str(), i);
        }
    }

    // Check if all codewords decoded
    if (cw_status.allSuccess()) {
        result.success = true;
        result.frame_data = cw_status.reassemble();
        LOG_MODEM(INFO, "[%s] RxPipeline: Frame decode SUCCESS (%d codewords)",
                  log_prefix_.c_str(), expected);
    } else {
        LOG_MODEM(INFO, "[%s] RxPipeline: Frame decode FAILED (%d/%d codewords OK)",
                  log_prefix_.c_str(), result.codewords_ok, expected);
    }

    // Clear accumulation state
    expected_codewords_ = 0;
    accumulated_soft_bits_.clear();

    return result;
}

bool RxPipeline::detectPing(const std::vector<float>& soft_bits) {
    // Convert soft bits to hard bytes
    Bytes ping_bytes;
    for (size_t i = 0; i + 8 <= soft_bits.size() && ping_bytes.size() < 8; i += 8) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) {
            if (soft_bits[i + b] > 0) {
                byte |= (1 << (7 - b));
            }
        }
        ping_bytes.push_back(byte);
    }

    if (ping_bytes.size() < 4) return false;

    // Check for "ULTR" magic (normal or bit-inverted due to DPSK phase ambiguity)
    bool is_normal = v2::PingFrame::isPing(ping_bytes);
    bool is_inverted = (ping_bytes[0] == 0xAA && ping_bytes[1] == 0xB3 &&
                        ping_bytes[2] == 0xAB && ping_bytes[3] == 0xAD);

    if (is_normal || is_inverted) {
        LOG_MODEM(INFO, "[%s] PING detected%s", log_prefix_.c_str(),
                  is_inverted ? " (inverted)" : "");
        return true;
    }

    return false;
}

std::vector<float> RxPipeline::deinterleaveCodewords(const std::vector<float>& soft_bits) {
    if (!interleaving_enabled_ || !interleaver_) {
        return soft_bits;
    }

    // Process one codeword at a time
    std::vector<float> result;
    result.reserve(soft_bits.size());

    for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size(); i += v2::LDPC_CODEWORD_BITS) {
        std::vector<float> cw_bits(soft_bits.begin() + i, soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
        auto deinterleaved = interleaver_->deinterleave(cw_bits);
        result.insert(result.end(), deinterleaved.begin(), deinterleaved.end());
    }

    // Copy any remaining bits (shouldn't happen with complete codewords)
    size_t processed = (soft_bits.size() / v2::LDPC_CODEWORD_BITS) * v2::LDPC_CODEWORD_BITS;
    if (processed < soft_bits.size()) {
        result.insert(result.end(), soft_bits.begin() + processed, soft_bits.end());
    }

    return result;
}

bool RxPipeline::decodeSingleCodeword(const std::vector<float>& soft_bits,
                                       CodeRate rate, size_t expected_bytes,
                                       Bytes& out_data) {
    LDPCDecoder decoder(rate);
    Bytes decoded = decoder.decodeSoft(soft_bits);

    if (!decoder.lastDecodeSuccess() || decoded.size() < expected_bytes) {
        return false;
    }

    out_data.assign(decoded.begin(), decoded.begin() + expected_bytes);
    return true;
}

RxPipeline::HeaderInfo RxPipeline::parseHeader(const Bytes& cw0_data) {
    HeaderInfo info;

    auto cw_info = v2::identifyCodeword(cw0_data);
    if (cw_info.type == v2::CodewordType::HEADER) {
        auto header = v2::parseHeader(cw0_data);
        if (header.valid) {
            info.valid = true;
            info.total_cw = header.total_cw;
            info.frame_type = header.type;
        }
    }

    return info;
}

} // namespace gui
} // namespace ultra
