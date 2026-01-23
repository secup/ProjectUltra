// ModemEngine - RX Implementation
// Acquisition thread, RX decode thread, buffer management, and decoding functions

#include "modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>

namespace ultra {
namespace gui {

// ============================================================================
// ACQUISITION THREAD IMPLEMENTATION
// ============================================================================

void ModemEngine::startAcquisitionThread() {
    if (acquisition_running_) return;

    acquisition_running_ = true;
    acquisition_thread_ = std::thread(&ModemEngine::acquisitionLoop, this);
    LOG_MODEM(INFO, "[%s] Acquisition thread started", log_prefix_.c_str());
}

void ModemEngine::stopAcquisitionThread() {
    if (!acquisition_running_) return;

    acquisition_running_ = false;
    acquisition_cv_.notify_all();

    if (acquisition_thread_.joinable()) {
        acquisition_thread_.join();
    }
    LOG_MODEM(INFO, "[%s] Acquisition thread stopped", log_prefix_.c_str());
}

void ModemEngine::acquisitionLoop() {
    LOG_MODEM(INFO, "[%s] Acquisition loop starting", log_prefix_.c_str());

    while (acquisition_running_) {
        // Wait for notification or timeout (check every 50ms)
        {
            std::unique_lock<std::mutex> lock(acquisition_mutex_);
            acquisition_cv_.wait_for(lock, std::chrono::milliseconds(50));
        }

        if (!acquisition_running_) break;

        // Skip if already connected (RX decode thread handles connected mode)
        if (connected_) continue;

        // Skip if RX decode thread is busy with a frame
        if (rx_frame_state_.active) continue;

        // Skip if queue already has a detected frame
        if (detected_frame_queue_.size() > 0) continue;

        // Get a snapshot of the sample buffer for preamble detection
        std::vector<float> samples_snapshot = getBufferSnapshot();
        if (samples_snapshot.size() < 10000) continue;  // Need minimum samples

        SampleSpan span(samples_snapshot.data(), samples_snapshot.size());

        // Try DPSK preamble detection first (more common for connection)
        int dpsk_start = dpsk_demodulator_->findPreamble(span);
        if (dpsk_start >= 0) {
            DetectedFrame frame;
            frame.data_start = dpsk_start;
            frame.waveform = protocol::WaveformMode::DPSK;
            frame.timestamp = std::chrono::steady_clock::now();

            detected_frame_queue_.push(frame);
            last_rx_waveform_ = protocol::WaveformMode::DPSK;

            LOG_MODEM(INFO, "[%s] Acquisition: DPSK preamble at %d (buffer=%zu)",
                      log_prefix_.c_str(), dpsk_start, samples_snapshot.size());
            continue;
        }

        // Try MFSK preamble detection
        int mfsk_start = mfsk_demodulator_->findPreamble(span);
        if (mfsk_start >= 0) {
            DetectedFrame frame;
            frame.data_start = mfsk_start;
            frame.waveform = protocol::WaveformMode::MFSK;
            frame.timestamp = std::chrono::steady_clock::now();

            detected_frame_queue_.push(frame);
            last_rx_waveform_ = protocol::WaveformMode::MFSK;

            LOG_MODEM(INFO, "[%s] Acquisition: MFSK preamble at %d (buffer=%zu)",
                      log_prefix_.c_str(), mfsk_start, samples_snapshot.size());
        }
    }

    LOG_MODEM(INFO, "[%s] Acquisition loop exiting", log_prefix_.c_str());
}

// ============================================================================
// RX/DECODE THREAD IMPLEMENTATION
// ============================================================================

void ModemEngine::startRxDecodeThread() {
    if (rx_decode_running_) return;

    rx_decode_running_ = true;
    detected_frame_queue_.start();
    rx_decode_thread_ = std::thread(&ModemEngine::rxDecodeLoop, this);
    LOG_MODEM(INFO, "[%s] RX decode thread started", log_prefix_.c_str());
}

void ModemEngine::stopRxDecodeThread() {
    if (!rx_decode_running_) return;

    rx_decode_running_ = false;
    detected_frame_queue_.stop();
    rx_decode_cv_.notify_all();

    if (rx_decode_thread_.joinable()) {
        rx_decode_thread_.join();
    }
    LOG_MODEM(INFO, "[%s] RX decode thread stopped", log_prefix_.c_str());
}

void ModemEngine::rxDecodeLoop() {
    LOG_MODEM(INFO, "[%s] RX decode loop starting", log_prefix_.c_str());

    while (rx_decode_running_) {
        // If connected, dispatch based on waveform mode
        if (connected_) {
            size_t buf_size = getBufferSize();

            if (waveform_mode_ == protocol::WaveformMode::OFDM) {
                // Process OFDM if:
                // 1. Buffer has enough for initial sync (8000 samples), OR
                // 2. We're already synced or have pending codewords (demod has internal state)
                bool has_pending_ofdm = ofdm_demodulator_->isSynced() ||
                                        ofdm_demodulator_->hasPendingData() ||
                                        ofdm_expected_codewords_ > 0;

                if (buf_size > 8000 || has_pending_ofdm) {
                    processRxBuffer_OFDM();
                }
            } else if (waveform_mode_ == protocol::WaveformMode::DPSK) {
                // Process DPSK in connected mode
                if (buf_size > 4000) {
                    processRxBuffer_DPSK();
                }
            } else if (waveform_mode_ == protocol::WaveformMode::MFSK) {
                // Process MFSK in connected mode
                if (buf_size > 4000) {
                    processRxBuffer_MFSK();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Not connected: wait for detected frame from acquisition thread
        DetectedFrame frame;
        if (!detected_frame_queue_.waitPop(frame, std::chrono::milliseconds(50))) {
            continue;
        }

        // Got a detected frame - mark state as active
        rx_frame_state_.active = true;
        rx_frame_state_.frame = frame;
        rx_frame_state_.expected_codewords = 0;
        rx_frame_state_.frame_type = protocol::v2::FrameType::PROBE;

        LOG_MODEM(INFO, "[%s] RX decode: Processing %s frame at %d",
                  log_prefix_.c_str(),
                  frame.waveform == protocol::WaveformMode::DPSK ? "DPSK" : "MFSK",
                  frame.data_start);

        // Decode based on waveform type
        bool success = false;
        if (frame.waveform == protocol::WaveformMode::DPSK) {
            success = rxDecodeDPSK(frame);
        } else if (frame.waveform == protocol::WaveformMode::MFSK) {
            success = rxDecodeMFSK(frame);
        }

        if (!success) {
            LOG_MODEM(INFO, "[%s] RX decode: Frame decode failed", log_prefix_.c_str());
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.frames_failed++;
        }

        // Clear state
        rx_frame_state_.clear();
    }

    LOG_MODEM(INFO, "[%s] RX decode loop exiting", log_prefix_.c_str());
}

// ============================================================================
// AUDIO INPUT
// ============================================================================

void ModemEngine::feedAudio(const float* samples, size_t count) {
    if (count == 0 || samples == nullptr) return;

    // Add samples directly to main buffer and wake threads
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);

        // Cap buffer size to prevent unbounded growth
        if (rx_sample_buffer_.size() + count > MAX_PENDING_SAMPLES) {
            size_t to_remove = rx_sample_buffer_.size() + count - MAX_PENDING_SAMPLES;
            if (to_remove >= rx_sample_buffer_.size()) {
                rx_sample_buffer_.clear();
            } else {
                rx_sample_buffer_.erase(rx_sample_buffer_.begin(),
                                        rx_sample_buffer_.begin() + to_remove);
            }
        }
        rx_sample_buffer_.insert(rx_sample_buffer_.end(), samples, samples + count);
    }

    // Update carrier sense energy (use a small window for responsiveness)
    if (count >= 480) {
        std::vector<float> energy_window(samples, samples + std::min(count, size_t(480)));
        updateChannelEnergy(energy_window);
    }

    // Wake acquisition thread to process new samples
    acquisition_cv_.notify_one();
}

void ModemEngine::feedAudio(const std::vector<float>& samples) {
    feedAudio(samples.data(), samples.size());
}

size_t ModemEngine::injectSignalFromFile(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary | std::ios::ate);
    if (!file) {
        LOG_MODEM(ERROR, "Failed to open signal file: %s", filepath.c_str());
        return 0;
    }

    size_t file_size = file.tellg();
    size_t num_samples = file_size / sizeof(float);
    file.seekg(0);

    std::vector<float> samples(num_samples);
    file.read(reinterpret_cast<char*>(samples.data()), file_size);

    if (!file) {
        LOG_MODEM(ERROR, "Failed to read signal file: %s", filepath.c_str());
        return 0;
    }

    LOG_MODEM(INFO, "Injecting %zu samples from %s", num_samples, filepath.c_str());

    // Inject in chunks and give threads time to process
    constexpr size_t CHUNK_SIZE = 4800;  // 100ms chunks
    for (size_t offset = 0; offset < num_samples; offset += CHUNK_SIZE) {
        size_t chunk_size = std::min(CHUNK_SIZE, num_samples - offset);
        feedAudio(samples.data() + offset, chunk_size);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Wait a bit for processing to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return num_samples;
}

// ============================================================================
// SAMPLE BUFFER HELPERS
// ============================================================================

std::vector<float> ModemEngine::getBufferSnapshot() const {
    std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
    return rx_sample_buffer_;
}

size_t ModemEngine::getBufferSize() const {
    std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
    return rx_sample_buffer_.size();
}

void ModemEngine::consumeSamples(size_t count) {
    std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
    if (count >= rx_sample_buffer_.size()) {
        rx_sample_buffer_.clear();
    } else {
        rx_sample_buffer_.erase(rx_sample_buffer_.begin(),
                                rx_sample_buffer_.begin() + count);
    }
}

// ============================================================================
// DPSK DECODE
// ============================================================================

bool ModemEngine::rxDecodeDPSK(const DetectedFrame& frame) {
    namespace v2 = protocol::v2;

    int preamble_samples = 32 * dpsk_config_.samples_per_symbol;
    int symbol_samples = dpsk_config_.samples_per_symbol;
    int bits_per_sym = dpsk_config_.bits_per_symbol();
    int samples_per_codeword = (v2::LDPC_CODEWORD_BITS / bits_per_sym) * symbol_samples;

    // Ping detection: "ULTR" = 4 bytes = 32 bits = 16 DQPSK symbols
    int ping_bits = v2::PingFrame::SIZE * 8;  // 32 bits
    int ping_symbols = (ping_bits + bits_per_sym - 1) / bits_per_sym;  // 16 for DQPSK
    int samples_for_ping = ping_symbols * symbol_samples;

    // Step 0: Check for PING (raw "ULTR" bytes, no LDPC)
    // Wait for just enough samples to detect ping
    while (rx_decode_running_) {
        size_t buffer_size = getBufferSize();
        int available = buffer_size - frame.data_start;
        if (available >= samples_for_ping) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!rx_decode_running_) return false;

    {
        std::vector<float> buffer = getBufferSnapshot();
        SampleSpan span(buffer.data(), buffer.size());

        dpsk_demodulator_->reset();
        dpsk_demodulator_->findPreamble(span);

        // Demodulate just enough for ping check
        SampleSpan ping_span(buffer.data() + frame.data_start, samples_for_ping);
        auto ping_soft = dpsk_demodulator_->demodulateSoft(ping_span);

        // Convert soft bits to hard bits
        Bytes ping_bytes;
        for (size_t i = 0; i + 8 <= ping_soft.size(); i += 8) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                if (ping_soft[i + b] > 0) {
                    byte |= (1 << (7 - b));
                }
            }
            ping_bytes.push_back(byte);
        }

        // Check for "ULTR" magic (also check bit-inverted version due to DPSK phase ambiguity)
        // DPSK can have 180° phase ambiguity that inverts all bits
        bool is_ping_normal = v2::PingFrame::isPing(ping_bytes);
        bool is_ping_inverted = false;
        if (!is_ping_normal && ping_bytes.size() >= 4) {
            // Check inverted: 0x55 ^ 0xFF = 0xAA, 0x4C ^ 0xFF = 0xB3, etc.
            is_ping_inverted = (ping_bytes[0] == 0xAA && ping_bytes[1] == 0xB3 &&
                               ping_bytes[2] == 0xAB && ping_bytes[3] == 0xAD);
        }

        if (is_ping_normal || is_ping_inverted) {
            LOG_MODEM(INFO, "[%s] RX DPSK: PING detected%s! (%02X %02X %02X %02X)",
                      log_prefix_.c_str(),
                      is_ping_inverted ? " (inverted)" : "",
                      ping_bytes.size() > 0 ? ping_bytes[0] : 0,
                      ping_bytes.size() > 1 ? ping_bytes[1] : 0,
                      ping_bytes.size() > 2 ? ping_bytes[2] : 0,
                      ping_bytes.size() > 3 ? ping_bytes[3] : 0);

            // Consume ping samples
            consumeSamples(frame.data_start + samples_for_ping);
            dpsk_demodulator_->reset();

            // Notify via callback (with rough SNR estimate from stats)
            if (ping_received_callback_) {
                float snr = getCurrentSNR();
                ping_received_callback_(snr);
            }

            // Update stats
            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.frames_received++;
            }

            last_rx_complete_time_ = std::chrono::steady_clock::now();
            return true;
        }

        // Not a ping - check if first 2 bytes are v2 magic (0x55 0x4C) or inverted (0xAA 0xB3)
        // If so, this is likely a normal frame; if not, it's garbage
        bool looks_like_v2_frame = (ping_bytes.size() >= 2 &&
                                    ((ping_bytes[0] == 0x55 && ping_bytes[1] == 0x4C) ||
                                     (ping_bytes[0] == 0xAA && ping_bytes[1] == 0xB3)));
        if (!looks_like_v2_frame) {
            LOG_MODEM(INFO, "[%s] RX DPSK: Not a PING or v2 frame (%02X %02X), discarding",
                      log_prefix_.c_str(),
                      ping_bytes.size() > 0 ? ping_bytes[0] : 0,
                      ping_bytes.size() > 1 ? ping_bytes[1] : 0);
            consumeSamples(frame.data_start + samples_for_ping);
            dpsk_demodulator_->reset();
            return false;
        }
    }

    // Step 1: Wait for CW0 samples (this is a normal v2 frame, needs LDPC decode)
    while (rx_decode_running_) {
        size_t buffer_size = getBufferSize();
        int available = buffer_size - frame.data_start;
        if (available >= samples_per_codeword) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!rx_decode_running_) return false;

    // Step 2: Decode CW0 to get codeword count
    std::vector<float> buffer = getBufferSnapshot();
    SampleSpan span(buffer.data(), buffer.size());

    dpsk_demodulator_->reset();
    dpsk_demodulator_->findPreamble(span);

    SampleSpan cw0_span(buffer.data() + frame.data_start, samples_per_codeword);
    auto cw0_soft = dpsk_demodulator_->demodulateSoft(cw0_span);

    if (cw0_soft.size() < v2::LDPC_CODEWORD_BITS) {
        LOG_MODEM(WARN, "RX DPSK: CW0 demod failed, got %zu bits", cw0_soft.size());
        consumeSamples(frame.data_start + samples_per_codeword);
        dpsk_demodulator_->reset();
        return false;
    }

    // Deinterleave CW0
    std::vector<float> cw0_bits(cw0_soft.begin(), cw0_soft.begin() + v2::LDPC_CODEWORD_BITS);
    if (interleaving_enabled_) {
        cw0_bits = interleaver_.deinterleave(cw0_bits);
    }

    // Decode CW0 with R1/4
    LDPCDecoder cw0_decoder(CodeRate::R1_4);
    Bytes cw0_decoded = cw0_decoder.decodeSoft(cw0_bits);

    if (!cw0_decoder.lastDecodeSuccess() || cw0_decoded.size() < v2::BYTES_PER_CODEWORD) {
        LOG_MODEM(INFO, "RX DPSK: CW0 LDPC decode FAILED");
        consumeSamples(frame.data_start + samples_per_codeword);
        dpsk_demodulator_->reset();
        return false;
    }

    LOG_MODEM(INFO, "RX DPSK: CW0 decode SUCCESS, first 8: %02X %02X %02X %02X %02X %02X %02X %02X",
              cw0_decoded[0], cw0_decoded[1], cw0_decoded[2], cw0_decoded[3],
              cw0_decoded[4], cw0_decoded[5], cw0_decoded[6], cw0_decoded[7]);

    // Parse header
    Bytes cw0_data(cw0_decoded.begin(), cw0_decoded.begin() + v2::BYTES_PER_CODEWORD);
    auto cw0_info = v2::identifyCodeword(cw0_data);

    int expected_codewords = 1;
    v2::FrameType detected_frame_type = v2::FrameType::PROBE;

    if (cw0_info.type == v2::CodewordType::HEADER) {
        auto header = v2::parseHeader(cw0_data);
        if (header.valid) {
            expected_codewords = header.total_cw;
            detected_frame_type = header.type;
            LOG_MODEM(INFO, "RX DPSK: Header valid, expecting %d codewords, frame_type=%d",
                      expected_codewords, static_cast<int>(header.type));
        }
    }

    // Step 3: Wait for ALL codeword samples
    int total_samples_needed = expected_codewords * samples_per_codeword;

    while (rx_decode_running_) {
        size_t buffer_size = getBufferSize();
        int available = buffer_size - frame.data_start;
        if (available >= total_samples_needed) break;
        LOG_MODEM(DEBUG, "RX DPSK: Need %d samples for %d CWs, have %d - waiting",
                  total_samples_needed, expected_codewords, available);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!rx_decode_running_) return false;

    // Step 4: Decode ALL codewords
    buffer = getBufferSnapshot();
    span = SampleSpan(buffer.data(), buffer.size());

    dpsk_demodulator_->reset();
    dpsk_demodulator_->findPreamble(span);

    SampleSpan full_data_span(buffer.data() + frame.data_start, total_samples_needed);
    auto soft_bits = dpsk_demodulator_->demodulateSoft(full_data_span);

    LOG_MODEM(INFO, "RX DPSK: Got %zu soft bits for %d codewords",
              soft_bits.size(), expected_codewords);

    // Deinterleave per-codeword
    if (interleaving_enabled_ && soft_bits.size() >= v2::LDPC_CODEWORD_BITS) {
        std::vector<float> deinterleaved;
        for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size();
             i += v2::LDPC_CODEWORD_BITS) {
            std::vector<float> cw_bits(soft_bits.begin() + i,
                                       soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
            auto di = interleaver_.deinterleave(cw_bits);
            deinterleaved.insert(deinterleaved.end(), di.begin(), di.end());
        }
        soft_bits = std::move(deinterleaved);
    }

    // Decode each codeword
    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    size_t num_codewords = std::min((size_t)expected_codewords, soft_bits.size() / LDPC_BLOCK);

    bool use_adaptive_for_data_cw = connected_ && v2::isDataFrame(detected_frame_type);
    CodeRate cw1_rate = use_adaptive_for_data_cw ? data_code_rate_ : CodeRate::R1_4;
    size_t cw0_bytes = v2::BYTES_PER_CODEWORD;
    size_t cw1_bytes = v2::getBytesPerCodeword(cw1_rate);

    LOG_MODEM(INFO, "RX DPSK: Decoding %zu CWs, frame_type=%d, CW1+ rate=%s (%zu bytes/CW)",
              num_codewords, static_cast<int>(detected_frame_type),
              use_adaptive_for_data_cw ? "adaptive" : "R1/4", cw1_bytes);

    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(expected_codewords, false);
    cw_status.data.resize(expected_codewords);
    int cw_success = 0, cw_failed = 0;

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(soft_bits.begin() + i * LDPC_BLOCK,
                                   soft_bits.begin() + (i + 1) * LDPC_BLOCK);

        CodeRate cw_rate = (i == 0) ? CodeRate::R1_4 : cw1_rate;
        size_t expected_bytes = (i == 0) ? cw0_bytes : cw1_bytes;

        LDPCDecoder cw_decoder(cw_rate);
        Bytes decoded = cw_decoder.decodeSoft(cw_bits);
        bool success = cw_decoder.lastDecodeSuccess();

        if (success && decoded.size() >= expected_bytes) {
            Bytes cw_data(decoded.begin(), decoded.begin() + expected_bytes);
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            cw_success++;
            LOG_MODEM(INFO, "RX DPSK: CW%zu LDPC decode SUCCESS (rate=%s, %zu bytes)",
                      i, (cw_rate == CodeRate::R1_4) ? "R1/4" : "adaptive", expected_bytes);
        } else {
            cw_failed++;
            LOG_MODEM(INFO, "RX DPSK: CW%zu LDPC decode FAILED (rate=%s)", i,
                      (cw_rate == CodeRate::R1_4) ? "R1/4" : "adaptive");
        }
    }

    // Consume samples
    consumeSamples(frame.data_start + total_samples_needed);
    dpsk_demodulator_->reset();

    // Deliver frame if all codewords decoded
    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();

        if (!frame_data.empty()) {
            LOG_MODEM(INFO, "[%s] RX DPSK: Frame reassembled, %zu bytes",
                      log_prefix_.c_str(), frame_data.size());

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.frames_received++;
                stats_.synced = true;
                rx_data_queue_.push(frame_data);
            }

            if (raw_data_callback_) {
                raw_data_callback_(frame_data);
            }

            last_rx_complete_time_ = std::chrono::steady_clock::now();
            return true;
        }
    }

    return false;
}

// ============================================================================
// MFSK DECODE
// ============================================================================

bool ModemEngine::rxDecodeMFSK(const DetectedFrame& frame) {
    namespace v2 = protocol::v2;

    int symbol_samples = mfsk_config_.samples_per_symbol;
    int bits_per_sym = mfsk_config_.bits_per_symbol();
    int samples_per_codeword = (v2::LDPC_CODEWORD_BITS / bits_per_sym) * symbol_samples;

    // Step 1: Wait for CW0 samples
    while (rx_decode_running_) {
        size_t buffer_size = getBufferSize();
        int available = buffer_size - frame.data_start;
        if (available >= samples_per_codeword) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!rx_decode_running_) return false;

    // Step 2: Decode CW0
    std::vector<float> buffer = getBufferSnapshot();
    SampleSpan cw0_span(buffer.data() + frame.data_start, samples_per_codeword);
    auto cw0_soft = mfsk_demodulator_->demodulateSoft(cw0_span);

    if (cw0_soft.size() < v2::LDPC_CODEWORD_BITS) {
        LOG_MODEM(WARN, "RX MFSK: CW0 demod failed, got %zu bits", cw0_soft.size());
        consumeSamples(frame.data_start + samples_per_codeword);
        return false;
    }

    std::vector<float> cw0_bits(cw0_soft.begin(), cw0_soft.begin() + v2::LDPC_CODEWORD_BITS);
    LDPCDecoder cw0_decoder(CodeRate::R1_4);
    Bytes cw0_decoded = cw0_decoder.decodeSoft(cw0_bits);

    if (!cw0_decoder.lastDecodeSuccess() || cw0_decoded.size() < v2::BYTES_PER_CODEWORD) {
        LOG_MODEM(INFO, "RX MFSK: CW0 LDPC decode FAILED");
        consumeSamples(frame.data_start + samples_per_codeword);
        return false;
    }

    LOG_MODEM(INFO, "RX MFSK: CW0 decode SUCCESS");

    // Parse header
    Bytes cw0_data(cw0_decoded.begin(), cw0_decoded.begin() + v2::BYTES_PER_CODEWORD);
    auto cw0_info = v2::identifyCodeword(cw0_data);

    int expected_codewords = 1;
    if (cw0_info.type == v2::CodewordType::HEADER) {
        auto header = v2::parseHeader(cw0_data);
        if (header.valid) {
            expected_codewords = header.total_cw;
            LOG_MODEM(INFO, "RX MFSK: Header valid, expecting %d codewords", expected_codewords);
        }
    }

    // Step 3: Wait for ALL codewords
    int total_samples_needed = expected_codewords * samples_per_codeword;

    while (rx_decode_running_) {
        size_t buffer_size = getBufferSize();
        int available = buffer_size - frame.data_start;
        if (available >= total_samples_needed) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!rx_decode_running_) return false;

    // Step 4: Decode ALL codewords
    buffer = getBufferSnapshot();
    SampleSpan full_data_span(buffer.data() + frame.data_start, total_samples_needed);
    auto soft_bits = mfsk_demodulator_->demodulateSoft(full_data_span);

    LOG_MODEM(INFO, "RX MFSK: Got %zu soft bits for %d codewords",
              soft_bits.size(), expected_codewords);

    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    size_t num_codewords = std::min((size_t)expected_codewords, soft_bits.size() / LDPC_BLOCK);

    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(expected_codewords, false);
    cw_status.data.resize(expected_codewords);

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(soft_bits.begin() + i * LDPC_BLOCK,
                                   soft_bits.begin() + (i + 1) * LDPC_BLOCK);

        LDPCDecoder cw_decoder(CodeRate::R1_4);
        Bytes decoded = cw_decoder.decodeSoft(cw_bits);

        if (cw_decoder.lastDecodeSuccess() && decoded.size() >= v2::BYTES_PER_CODEWORD) {
            Bytes cw_data(decoded.begin(), decoded.begin() + v2::BYTES_PER_CODEWORD);
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            LOG_MODEM(INFO, "RX MFSK: CW%zu decode SUCCESS", i);
        } else {
            LOG_MODEM(INFO, "RX MFSK: CW%zu decode FAILED", i);
        }
    }

    consumeSamples(frame.data_start + total_samples_needed);

    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();

        if (!frame_data.empty()) {
            LOG_MODEM(INFO, "[%s] RX MFSK: Frame reassembled, %zu bytes",
                      log_prefix_.c_str(), frame_data.size());

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.frames_received++;
                stats_.synced = true;
                rx_data_queue_.push(frame_data);
            }

            if (raw_data_callback_) {
                raw_data_callback_(frame_data);
            }

            last_rx_complete_time_ = std::chrono::steady_clock::now();
            return true;
        }
    }

    return false;
}

// ============================================================================
// OFDM DECODE
// ============================================================================

void ModemEngine::processRxBuffer_OFDM() {
    namespace v2 = protocol::v2;
    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    // Log demod state for debugging
    static int ofdm_rx_call = 0;
    if (++ofdm_rx_call % 20 == 1) {
        LOG_MODEM(INFO, "[%s] RX OFDM: data_modulation_=%d, connected=%d, waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(data_modulation_),
                  connected_ ? 1 : 0, static_cast<int>(waveform_mode_));
    }

    constexpr size_t MIN_SAMPLES_FOR_SYNC = 8000;

    // Get samples with proper locking
    std::vector<float> samples;
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        if (!ofdm_demodulator_->isSynced() && !ofdm_demodulator_->hasPendingData()) {
            if (rx_sample_buffer_.size() < MIN_SAMPLES_FOR_SYNC) {
                return;
            }
        }
        samples = std::move(rx_sample_buffer_);
        rx_sample_buffer_.clear();
    }

    // Only return early if we have no samples AND no pending state
    if (samples.empty() &&
        !ofdm_demodulator_->hasPendingData() &&
        ofdm_accumulated_soft_bits_.empty() &&
        ofdm_expected_codewords_ == 0) {
        return;
    }

    bool was_synced = ofdm_demodulator_->isSynced();

    SampleSpan span(samples.data(), samples.size());
    bool frame_ready = ofdm_demodulator_->process(span);

    bool is_synced = ofdm_demodulator_->isSynced();
    if (was_synced && !is_synced) {
        if (!ofdm_accumulated_soft_bits_.empty() || ofdm_expected_codewords_ > 0) {
            LOG_MODEM(INFO, "RX OFDM: Demodulator lost sync mid-frame, discarding %zu accumulated bits (expected %d CWs)",
                      ofdm_accumulated_soft_bits_.size(), ofdm_expected_codewords_);
            ofdm_accumulated_soft_bits_.clear();
            ofdm_expected_codewords_ = 0;
        }
    }

    if (is_synced) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.snr_db = ofdm_demodulator_->getEstimatedSNR();
        stats_.synced = true;
    }

    LOG_MODEM(INFO, "[%s] RX OFDM: frame_ready=%d, is_synced=%d",
              log_prefix_.c_str(), frame_ready ? 1 : 0, is_synced ? 1 : 0);

    if (frame_ready) {
        auto soft_bits = ofdm_demodulator_->getSoftBits();
        LOG_MODEM(INFO, "[%s] RX OFDM: Got %zu soft bits from demod",
                  log_prefix_.c_str(), soft_bits.size());

        if (!soft_bits.empty()) {
            if (interleaving_enabled_) {
                for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size(); i += v2::LDPC_CODEWORD_BITS) {
                    std::vector<float> cw_bits(soft_bits.begin() + i, soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
                    auto deinterleaved = interleaver_.deinterleave(cw_bits);
                    ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                       deinterleaved.begin(), deinterleaved.end());
                }
                size_t full_cws = (soft_bits.size() / v2::LDPC_CODEWORD_BITS) * v2::LDPC_CODEWORD_BITS;
                if (full_cws < soft_bits.size()) {
                    ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                       soft_bits.begin() + full_cws, soft_bits.end());
                }
            } else {
                ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                   soft_bits.begin(), soft_bits.end());
            }
            LOG_MODEM(INFO, "[%s] RX OFDM: Accumulated %zu soft bits, total now %zu",
                      log_prefix_.c_str(), soft_bits.size(), ofdm_accumulated_soft_bits_.size());
        }
    }

    size_t num_codewords = ofdm_accumulated_soft_bits_.size() / LDPC_BLOCK;
    if (num_codewords == 0) {
        return;
    }

    // Decode CW0 to get expected count
    if (ofdm_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw_bits(ofdm_accumulated_soft_bits_.begin(),
                                    ofdm_accumulated_soft_bits_.begin() + LDPC_BLOCK);

        float sum_mag = 0, pos_count = 0;
        for (float b : cw_bits) {
            sum_mag += std::abs(b);
            if (b > 0) pos_count++;
        }
        LOG_MODEM(INFO, "[%s] RX OFDM: CW0 soft bits: avg_mag=%.2f, pos_ratio=%.2f, first8: %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
                  log_prefix_.c_str(), sum_mag / cw_bits.size(), pos_count / cw_bits.size(),
                  cw_bits[0], cw_bits[1], cw_bits[2], cw_bits[3],
                  cw_bits[4], cw_bits[5], cw_bits[6], cw_bits[7]);

        LDPCDecoder cw0_decoder(CodeRate::R1_4);
        Bytes decoded = cw0_decoder.decodeSoft(cw_bits);
        bool ldpc_ok = cw0_decoder.lastDecodeSuccess();

        LOG_MODEM(INFO, "[%s] RX OFDM: CW0 probe: LDPC %s, decoded %zu bytes, first8: %02x %02x %02x %02x %02x %02x %02x %02x",
                  log_prefix_.c_str(), ldpc_ok ? "OK" : "FAILED", decoded.size(),
                  decoded.size() > 0 ? decoded[0] : 0, decoded.size() > 1 ? decoded[1] : 0,
                  decoded.size() > 2 ? decoded[2] : 0, decoded.size() > 3 ? decoded[3] : 0,
                  decoded.size() > 4 ? decoded[4] : 0, decoded.size() > 5 ? decoded[5] : 0,
                  decoded.size() > 6 ? decoded[6] : 0, decoded.size() > 7 ? decoded[7] : 0);

        if (ldpc_ok && decoded.size() >= v2::BYTES_PER_CODEWORD) {
            Bytes cw_data(decoded.begin(), decoded.begin() + v2::BYTES_PER_CODEWORD);
            auto cw_info = v2::identifyCodeword(cw_data);
            LOG_MODEM(INFO, "[%s] RX OFDM: CW0 type=%d, first 4: %02x %02x %02x %02x",
                      log_prefix_.c_str(), static_cast<int>(cw_info.type),
                      cw_data[0], cw_data[1], cw_data[2], cw_data[3]);

            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw_data);
                if (header.valid) {
                    ofdm_expected_codewords_ = header.total_cw;
                    LOG_MODEM(INFO, "[%s] RX OFDM: CW0 decoded, expecting %d total codewords",
                              log_prefix_.c_str(), ofdm_expected_codewords_);
                } else {
                    LOG_MODEM(INFO, "[%s] RX OFDM: CW0 header INVALID", log_prefix_.c_str());
                }
            }
        }
    }

    // Wait for all expected codewords
    if (ofdm_expected_codewords_ > 0 && num_codewords < (size_t)ofdm_expected_codewords_) {
        LOG_MODEM(DEBUG, "RX OFDM: Have %zu/%d codewords, waiting", num_codewords, ofdm_expected_codewords_);
        return;
    }

    if (ofdm_expected_codewords_ == 0 && is_synced) {
        LOG_MODEM(DEBUG, "RX OFDM: CW0 decode failed, demod still synced, waiting (have %zu CWs)", num_codewords);
        return;
    }

    // Process complete frame
    std::vector<float> accumulated_soft_bits = std::move(ofdm_accumulated_soft_bits_);
    ofdm_accumulated_soft_bits_.clear();
    int expected_total = ofdm_expected_codewords_;
    ofdm_expected_codewords_ = 0;

    num_codewords = accumulated_soft_bits.size() / LDPC_BLOCK;
    if (expected_total > 0 && (size_t)expected_total < num_codewords) {
        num_codewords = expected_total;
    }

    LOG_MODEM(INFO, "RX OFDM: Processing complete frame, %zu codewords", num_codewords);

    v2::CodewordStatus cw_status;
    int cw_success = 0, cw_failed = 0;
    std::string frame_type_str;

    // First pass: decode CW0 to get frame type
    v2::FrameType detected_frame_type = v2::FrameType::PROBE;
    {
        std::vector<float> cw_bits(accumulated_soft_bits.begin(),
                                    accumulated_soft_bits.begin() + LDPC_BLOCK);
        LDPCDecoder cw0_decoder(CodeRate::R1_4);
        Bytes decoded = cw0_decoder.decodeSoft(cw_bits);
        bool success = cw0_decoder.lastDecodeSuccess();

        if (success && decoded.size() >= v2::BYTES_PER_CODEWORD) {
            Bytes cw_data(decoded.begin(), decoded.begin() + v2::BYTES_PER_CODEWORD);
            auto cw_info = v2::identifyCodeword(cw_data);

            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw_data);
                if (header.valid) {
                    detected_frame_type = header.type;
                }
            }
        }
    }

    if (status_callback_) {
        status_callback_("[FRAME] Received " + std::to_string(num_codewords) + " codewords");
    }

    bool use_adaptive_for_data_cw = connected_ && v2::isDataFrame(detected_frame_type);
    CodeRate cw1_rate = use_adaptive_for_data_cw ? data_code_rate_ : CodeRate::R1_4;
    size_t cw0_bytes = v2::BYTES_PER_CODEWORD;
    size_t cw1_bytes = v2::getBytesPerCodeword(cw1_rate);

    LOG_MODEM(INFO, "RX OFDM: Decoding %zu CWs, frame_type=%d, CW1+ rate=%s (%zu bytes/CW)",
              num_codewords, static_cast<int>(detected_frame_type),
              use_adaptive_for_data_cw ? "adaptive" : "R1/4", cw1_bytes);

    cw_status.decoded.resize(num_codewords, false);
    cw_status.data.resize(num_codewords);

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(accumulated_soft_bits.begin() + i * LDPC_BLOCK,
                                    accumulated_soft_bits.begin() + (i + 1) * LDPC_BLOCK);

        CodeRate cw_rate = (i == 0) ? CodeRate::R1_4 : cw1_rate;
        size_t expected_bytes = (i == 0) ? cw0_bytes : cw1_bytes;

        LOG_MODEM(INFO, "RX OFDM: Decoding CW%zu with rate %s, expected %zu bytes",
                  i, codeRateToString(cw_rate), expected_bytes);

        if (i > 0) {
            float sum_abs = 0.0f;
            int pos_count = 0;
            for (float llr : cw_bits) {
                sum_abs += std::abs(llr);
                if (llr > 0) pos_count++;
            }
            float avg_llr = sum_abs / cw_bits.size();

            std::string hex_str;
            for (int byte_idx = 0; byte_idx < 16 && byte_idx * 8 < (int)cw_bits.size(); byte_idx++) {
                uint8_t byte_val = 0;
                for (int bit = 0; bit < 8 && byte_idx * 8 + bit < (int)cw_bits.size(); bit++) {
                    if (cw_bits[byte_idx * 8 + bit] < 0) byte_val |= (1 << (7 - bit));
                }
                char buf[8];
                snprintf(buf, sizeof(buf), "%02X ", byte_val);
                hex_str += buf;
            }

            LOG_MODEM(INFO, "RX OFDM: CW%zu soft bits: avg_llr=%.2f, pos_ratio=%.2f, first16bytes=[%s]",
                      i, avg_llr, (float)pos_count / cw_bits.size(), hex_str.c_str());
        }

        LDPCDecoder cw_decoder(cw_rate);
        Bytes decoded = cw_decoder.decodeSoft(cw_bits);
        bool success = cw_decoder.lastDecodeSuccess();

        LOG_MODEM(INFO, "RX OFDM: CW%zu decode %s, got %zu bytes",
                  i, success ? "SUCCESS" : "FAILED", decoded.size());

        if (success && decoded.size() >= expected_bytes) {
            Bytes cw_data(decoded.begin(), decoded.begin() + expected_bytes);
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            cw_success++;

            auto cw_info = v2::identifyCodeword(cw_data);

            if (i == 0 && cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw_data);
                if (header.valid) {
                    expected_total = header.total_cw;
                    switch (header.type) {
                        case v2::FrameType::PROBE: frame_type_str = "PROBE"; break;
                        case v2::FrameType::PROBE_ACK: frame_type_str = "PROBE_ACK"; break;
                        case v2::FrameType::CONNECT: frame_type_str = "CONNECT"; break;
                        case v2::FrameType::CONNECT_ACK: frame_type_str = "CONNECT_ACK"; break;
                        case v2::FrameType::DISCONNECT: frame_type_str = "DISCONNECT"; break;
                        case v2::FrameType::DATA: frame_type_str = "DATA"; break;
                        case v2::FrameType::ACK: frame_type_str = "ACK"; break;
                        case v2::FrameType::NACK: frame_type_str = "NACK"; break;
                        default: frame_type_str = "OTHER"; break;
                    }
                    if (status_callback_) {
                        status_callback_("[FRAME] Type=" + frame_type_str +
                                       " CWs=" + std::to_string(expected_total));
                    }
                }
            }

            std::string cw_type = (cw_info.type == v2::CodewordType::HEADER) ? "HDR" :
                                  (cw_info.type == v2::CodewordType::DATA) ? "DATA" : "?";
            if (status_callback_) {
                std::string progress = "[CW " + std::to_string(i + 1);
                if (expected_total > 0) progress += "/" + std::to_string(expected_total);
                progress += "] OK (" + cw_type + ")";
                status_callback_(progress);
            }
        } else {
            cw_failed++;
            if (status_callback_) {
                status_callback_("[CW " + std::to_string(i + 1) + "] FAILED");
            }
        }
    }

    LOG_MODEM(INFO, "RX OFDM: CW status - success=%d, failed=%d, allSuccess=%d",
              cw_success, cw_failed, cw_status.allSuccess() ? 1 : 0);

    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();
        LOG_MODEM(INFO, "RX OFDM: Reassembled %zu bytes", frame_data.size());

        if (!frame_data.empty()) {
            bool frame_parsed = false;

            if (num_codewords == 1) {
                auto ctrl_frame = v2::ControlFrame::deserialize(frame_data);
                if (ctrl_frame) {
                    frame_parsed = true;
                    if (status_callback_) {
                        std::stringstream ss;
                        ss << "[" << frame_type_str << "] src=0x" << std::hex << ctrl_frame->src_hash
                           << " dst=0x" << ctrl_frame->dst_hash << std::dec
                           << " seq=" << ctrl_frame->seq;
                        status_callback_(ss.str());
                    }
                }
            }

            if (!frame_parsed) {
                auto connect_frame = v2::ConnectFrame::deserialize(frame_data);
                if (connect_frame) {
                    frame_parsed = true;
                    std::string src = connect_frame->getSrcCallsign();
                    std::string dst = connect_frame->getDstCallsign();
                    if (status_callback_) {
                        status_callback_("[" + frame_type_str + "] " + src + " -> " + dst);
                    }
                }
            }

            if (!frame_parsed) {
                auto parsed = v2::DataFrame::deserialize(frame_data);
                if (parsed && !parsed->payload.empty()) {
                    frame_parsed = true;
                    std::string msg(parsed->payload.begin(), parsed->payload.end());
                    msg.erase(std::remove_if(msg.begin(), msg.end(),
                        [](char c) { return c < 32 || c > 126; }), msg.end());

                    if (status_callback_ && !msg.empty()) {
                        status_callback_("[MESSAGE] \"" + msg + "\"");
                    }
                    if (data_callback_ && !msg.empty()) {
                        data_callback_(msg);
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.frames_received++;
                if (cw_failed > 0) stats_.frames_failed++;
                ChannelQuality quality = ofdm_demodulator_->getChannelQuality();
                stats_.snr_db = quality.snr_db;
                stats_.ber = quality.ber_estimate;
                stats_.synced = true;
                rx_data_queue_.push(frame_data);
            }

            if (raw_data_callback_) {
                raw_data_callback_(frame_data);
            }

            last_rx_complete_time_ = std::chrono::steady_clock::now();
        }
    } else {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (cw_failed > 0) stats_.frames_failed++;
    }

    LOG_MODEM(INFO, "[%s] RX OFDM: Frame processing complete, calling demod reset()",
              log_prefix_.c_str());
    ofdm_demodulator_->reset();
}

// ============================================================================
// DPSK BUFFER PROCESSING (Connected Mode)
// ============================================================================

void ModemEngine::processRxBuffer_DPSK() {
    // Get buffer snapshot for preamble detection
    std::vector<float> samples = getBufferSnapshot();
    if (samples.size() < 4000) return;

    SampleSpan span(samples.data(), samples.size());

    // Look for DPSK preamble
    int preamble_start = dpsk_demodulator_->findPreamble(span);
    if (preamble_start < 0) {
        // No preamble found - trim old samples if buffer is getting large
        if (samples.size() > 48000) {
            consumeSamples(samples.size() - 24000);
        }
        return;
    }

    LOG_MODEM(INFO, "[%s] processRxBuffer_DPSK: Found preamble at %d",
              log_prefix_.c_str(), preamble_start);

    // Create a DetectedFrame and delegate to rxDecodeDPSK
    DetectedFrame frame;
    frame.data_start = preamble_start;
    frame.waveform = protocol::WaveformMode::DPSK;
    frame.timestamp = std::chrono::steady_clock::now();

    // Mark state as active
    rx_frame_state_.active = true;
    rx_frame_state_.frame = frame;
    rx_frame_state_.expected_codewords = 0;
    rx_frame_state_.frame_type = protocol::v2::FrameType::PROBE;

    bool success = rxDecodeDPSK(frame);

    if (!success) {
        LOG_MODEM(INFO, "[%s] processRxBuffer_DPSK: Frame decode failed",
                  log_prefix_.c_str());
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_failed++;
    }

    rx_frame_state_.clear();
}

// ============================================================================
// MFSK BUFFER PROCESSING (Connected Mode)
// ============================================================================

void ModemEngine::processRxBuffer_MFSK() {
    // Get buffer snapshot for preamble detection
    std::vector<float> samples = getBufferSnapshot();
    if (samples.size() < 4000) return;

    SampleSpan span(samples.data(), samples.size());

    // Look for MFSK preamble
    int preamble_start = mfsk_demodulator_->findPreamble(span);
    if (preamble_start < 0) {
        // No preamble found - trim old samples if buffer is getting large
        if (samples.size() > 48000) {
            consumeSamples(samples.size() - 24000);
        }
        return;
    }

    LOG_MODEM(INFO, "[%s] processRxBuffer_MFSK: Found preamble at %d",
              log_prefix_.c_str(), preamble_start);

    // Create a DetectedFrame and delegate to rxDecodeMFSK
    DetectedFrame frame;
    frame.data_start = preamble_start;
    frame.waveform = protocol::WaveformMode::MFSK;
    frame.timestamp = std::chrono::steady_clock::now();

    // Mark state as active
    rx_frame_state_.active = true;
    rx_frame_state_.frame = frame;
    rx_frame_state_.expected_codewords = 0;
    rx_frame_state_.frame_type = protocol::v2::FrameType::PROBE;

    bool success = rxDecodeMFSK(frame);

    if (!success) {
        LOG_MODEM(INFO, "[%s] processRxBuffer_MFSK: Frame decode failed",
                  log_prefix_.c_str());
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_failed++;
    }

    rx_frame_state_.clear();
}

} // namespace gui
} // namespace ultra
