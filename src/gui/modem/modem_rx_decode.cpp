// ModemEngine RX Decode Implementation
// Extracted decode logic for cleaner separation of concerns

#include "modem_engine.hpp"
#include "modem_rx_constants.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include <algorithm>
#include <sstream>

namespace ultra {
namespace gui {

namespace v2 = protocol::v2;

// ============================================================================
// LDPC DECODE HELPERS
// ============================================================================

// Result of decoding a frame's codewords
struct FrameDecodeResult {
    bool success = false;
    Bytes frame_data;
    v2::FrameType frame_type = v2::FrameType::PROBE;
    int codewords_ok = 0;
    int codewords_failed = 0;
};

// Decode a single LDPC codeword
static bool decodeSingleCodeword(
    const std::vector<float>& soft_bits,
    CodeRate rate,
    size_t expected_bytes,
    Bytes& out_data
) {
    LDPCDecoder decoder(rate);
    Bytes decoded = decoder.decodeSoft(soft_bits);

    if (!decoder.lastDecodeSuccess() || decoded.size() < expected_bytes) {
        return false;
    }

    out_data.assign(decoded.begin(), decoded.begin() + expected_bytes);
    return true;
}

// Decode multiple codewords with proper rate handling (CW0 = R1/4, CW1+ = adaptive)
static FrameDecodeResult decodeCodewords(
    const std::vector<float>& soft_bits,
    size_t num_codewords,
    CodeRate data_rate,
    bool use_adaptive_rate,
    const char* log_prefix
) {
    FrameDecodeResult result;
    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    if (soft_bits.size() < num_codewords * LDPC_BLOCK) {
        LOG_MODEM(WARN, "[%s] decodeCodewords: insufficient bits %zu for %zu CWs",
                  log_prefix, soft_bits.size(), num_codewords);
        return result;
    }

    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(num_codewords, false);
    cw_status.data.resize(num_codewords);

    // Determine rates and sizes
    CodeRate cw1_rate = use_adaptive_rate ? data_rate : CodeRate::R1_4;
    size_t cw0_bytes = v2::BYTES_PER_CODEWORD;
    size_t cw1_bytes = v2::getBytesPerCodeword(cw1_rate);

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(
            soft_bits.begin() + i * LDPC_BLOCK,
            soft_bits.begin() + (i + 1) * LDPC_BLOCK
        );

        CodeRate rate = (i == 0) ? CodeRate::R1_4 : cw1_rate;
        size_t expected_bytes = (i == 0) ? cw0_bytes : cw1_bytes;

        Bytes cw_data;
        if (decodeSingleCodeword(cw_bits, rate, expected_bytes, cw_data)) {
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            result.codewords_ok++;

            // Extract frame type from CW0 header
            if (i == 0) {
                auto cw_info = v2::identifyCodeword(cw_data);
                if (cw_info.type == v2::CodewordType::HEADER) {
                    auto header = v2::parseHeader(cw_data);
                    if (header.valid) {
                        result.frame_type = header.type;
                    }
                }
            }
        } else {
            result.codewords_failed++;
        }
    }

    if (cw_status.allSuccess()) {
        result.success = true;
        result.frame_data = cw_status.reassemble();
    }

    return result;
}

// ============================================================================
// PING DETECTION
// ============================================================================

// Check for PING frame (raw "ULTR" bytes, no LDPC)
bool ModemEngine::detectPing(const std::vector<float>& soft_bits) {
    // Convert soft bits to hard bytes
    Bytes ping_bytes;
    for (size_t i = 0; i + 8 <= soft_bits.size(); i += 8) {
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
        LOG_MODEM(INFO, "[%s] PING detected%s",
                  log_prefix_.c_str(), is_inverted ? " (inverted)" : "");
        return true;
    }

    return false;
}

// Check if bytes look like start of v2 frame
static bool looksLikeV2Frame(const Bytes& bytes) {
    if (bytes.size() < 2) return false;
    // Normal magic (0x55 0x4C) or inverted (0xAA 0xB3)
    return (bytes[0] == 0x55 && bytes[1] == 0x4C) ||
           (bytes[0] == 0xAA && bytes[1] == 0xB3);
}

// ============================================================================
// DPSK FRAME DECODE
// ============================================================================

bool ModemEngine::rxDecodeDPSK(const DetectedFrame& frame) {
    using namespace rx_constants;

    const int symbol_samples = dpsk_config_.samples_per_symbol;
    const int bits_per_sym = dpsk_config_.bits_per_symbol();
    const int samples_per_codeword = (v2::LDPC_CODEWORD_BITS / bits_per_sym) * symbol_samples;

    // PING: 4 bytes = 32 bits
    const int ping_bits = v2::PingFrame::SIZE * 8;
    const int ping_symbols = (ping_bits + bits_per_sym - 1) / bits_per_sym;
    const int samples_for_ping = ping_symbols * symbol_samples;

    // --- Step 1: Wait for ping samples and check for PING ---
    if (!waitForSamples(frame.data_start + samples_for_ping)) return false;

    {
        auto buffer = getBufferSnapshot();

        // Set up demodulator reference from last preamble symbol
        // frame.data_start points to where DATA begins (after preamble)
        // Reference symbol is one symbol before data start
        int ref_offset = frame.data_start - symbol_samples;
        if (ref_offset < 0 || ref_offset + symbol_samples > (int)buffer.size()) {
            LOG_MODEM(WARN, "[%s] DPSK: Invalid ref_offset %d", log_prefix_.c_str(), ref_offset);
            return false;
        }
        SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
        dpsk_demodulator_->setReferenceSymbol(ref_sym);

        SampleSpan ping_span(buffer.data() + frame.data_start, samples_for_ping);
        auto ping_soft = dpsk_demodulator_->demodulateSoft(ping_span);

        if (detectPing(ping_soft)) {
            consumeSamples(frame.data_start + samples_for_ping);
            dpsk_demodulator_->reset();

            if (ping_received_callback_) {
                ping_received_callback_(getCurrentSNR());
            }

            updateStats([](LoopbackStats& s) { s.frames_received++; });
            last_rx_complete_time_ = std::chrono::steady_clock::now();
            return true;
        }

        // Convert soft to bytes for v2 magic check
        Bytes header_bytes;
        for (size_t i = 0; i + 8 <= ping_soft.size(); i += 8) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) {
                if (ping_soft[i + b] > 0) byte |= (1 << (7 - b));
            }
            header_bytes.push_back(byte);
        }

        if (!looksLikeV2Frame(header_bytes)) {
            LOG_MODEM(INFO, "[%s] DPSK: Not PING or v2 frame, discarding", log_prefix_.c_str());
            consumeSamples(frame.data_start + samples_for_ping);
            dpsk_demodulator_->reset();
            return false;
        }
    }

    // --- Step 2: Decode CW0 to get codeword count ---
    if (!waitForSamples(frame.data_start + samples_per_codeword)) return false;

    int expected_codewords = 1;
    v2::FrameType frame_type = v2::FrameType::PROBE;

    {
        auto buffer = getBufferSnapshot();

        // Set up demodulator reference from last preamble symbol
        int ref_offset = frame.data_start - symbol_samples;
        if (ref_offset < 0 || ref_offset + symbol_samples > (int)buffer.size()) {
            LOG_MODEM(WARN, "[%s] DPSK: Invalid ref_offset %d for CW0", log_prefix_.c_str(), ref_offset);
            return false;
        }
        SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
        dpsk_demodulator_->setReferenceSymbol(ref_sym);

        SampleSpan cw0_span(buffer.data() + frame.data_start, samples_per_codeword);
        auto cw0_soft = dpsk_demodulator_->demodulateSoft(cw0_span);

        if (cw0_soft.size() < v2::LDPC_CODEWORD_BITS) {
            LOG_MODEM(WARN, "[%s] DPSK: CW0 demod failed", log_prefix_.c_str());
            consumeSamples(frame.data_start + samples_per_codeword);
            dpsk_demodulator_->reset();
            return false;
        }

        // Deinterleave and decode CW0
        std::vector<float> cw0_bits(cw0_soft.begin(), cw0_soft.begin() + v2::LDPC_CODEWORD_BITS);
        if (interleaving_enabled_) {
            cw0_bits = interleaver_.deinterleave(cw0_bits);
        }

        Bytes cw0_data;
        if (!decodeSingleCodeword(cw0_bits, CodeRate::R1_4, v2::BYTES_PER_CODEWORD, cw0_data)) {
            LOG_MODEM(INFO, "[%s] DPSK: CW0 LDPC failed", log_prefix_.c_str());
            consumeSamples(frame.data_start + samples_per_codeword);
            dpsk_demodulator_->reset();
            return false;
        }

        // Parse header
        auto cw_info = v2::identifyCodeword(cw0_data);
        if (cw_info.type == v2::CodewordType::HEADER) {
            auto header = v2::parseHeader(cw0_data);
            if (header.valid) {
                expected_codewords = header.total_cw;
                frame_type = header.type;
                LOG_MODEM(INFO, "[%s] DPSK: Header valid, %d codewords, type=%d",
                          log_prefix_.c_str(), expected_codewords, static_cast<int>(frame_type));
            }
        }
    }

    // --- Step 3: Wait for all codewords and decode ---
    int total_samples = expected_codewords * samples_per_codeword;
    if (!waitForSamples(frame.data_start + total_samples)) return false;

    auto buffer = getBufferSnapshot();

    // Set up demodulator reference from last preamble symbol
    int ref_offset = frame.data_start - symbol_samples;
    if (ref_offset < 0 || ref_offset + symbol_samples > (int)buffer.size()) {
        LOG_MODEM(WARN, "[%s] DPSK: Invalid ref_offset %d for full frame", log_prefix_.c_str(), ref_offset);
        return false;
    }
    SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
    dpsk_demodulator_->setReferenceSymbol(ref_sym);

    SampleSpan data_span(buffer.data() + frame.data_start, total_samples);
    auto soft_bits = dpsk_demodulator_->demodulateSoft(data_span);

    // Deinterleave per-codeword
    if (interleaving_enabled_) {
        soft_bits = deinterleaveCodewords(soft_bits);
    }

    // Decode all codewords
    bool use_adaptive = connected_ && v2::isDataFrame(frame_type);
    auto result = decodeCodewords(soft_bits, expected_codewords, data_code_rate_,
                                   use_adaptive, log_prefix_.c_str());

    consumeSamples(frame.data_start + total_samples);
    dpsk_demodulator_->reset();

    if (result.success) {
        deliverFrame(result.frame_data);
        return true;
    }

    return false;
}

// ============================================================================
// OFDM FRAME DECODE
// ============================================================================

void ModemEngine::processRxBuffer_OFDM() {
    using namespace rx_constants;
    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    // Get samples
    std::vector<float> samples;
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        bool has_pending = ofdm_demodulator_->isSynced() ||
                          ofdm_demodulator_->hasPendingData() ||
                          ofdm_expected_codewords_ > 0;

        if (!has_pending && rx_sample_buffer_.size() < MIN_SAMPLES_FOR_OFDM_SYNC) {
            return;
        }
        samples = std::move(rx_sample_buffer_);
        rx_sample_buffer_.clear();
    }

    if (samples.empty() && !ofdm_demodulator_->hasPendingData() &&
        ofdm_accumulated_soft_bits_.empty() && ofdm_expected_codewords_ == 0) {
        return;
    }

    // Process through demodulator
    bool was_synced = ofdm_demodulator_->isSynced();
    SampleSpan span(samples.data(), samples.size());
    bool frame_ready = ofdm_demodulator_->process(span);
    bool is_synced = ofdm_demodulator_->isSynced();

    // Handle sync loss mid-frame
    if (was_synced && !is_synced && !ofdm_accumulated_soft_bits_.empty()) {
        LOG_MODEM(INFO, "[%s] OFDM: Lost sync mid-frame, discarding", log_prefix_.c_str());
        ofdm_accumulated_soft_bits_.clear();
        ofdm_expected_codewords_ = 0;
    }

    if (is_synced) {
        updateStats([this](LoopbackStats& s) {
            s.snr_db = ofdm_demodulator_->getEstimatedSNR();
            s.synced = true;
        });
    }

    // Accumulate soft bits
    if (frame_ready) {
        auto soft_bits = ofdm_demodulator_->getSoftBits();
        if (!soft_bits.empty()) {
            if (interleaving_enabled_) {
                auto deinterleaved = deinterleaveCodewords(soft_bits);
                ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                   deinterleaved.begin(), deinterleaved.end());
            } else {
                ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                   soft_bits.begin(), soft_bits.end());
            }
        }
    }

    size_t num_codewords = ofdm_accumulated_soft_bits_.size() / LDPC_BLOCK;
    if (num_codewords == 0) return;

    // Probe CW0 to get expected count
    if (ofdm_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw0_bits(ofdm_accumulated_soft_bits_.begin(),
                                     ofdm_accumulated_soft_bits_.begin() + LDPC_BLOCK);

        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, CodeRate::R1_4, v2::BYTES_PER_CODEWORD, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) {
                    ofdm_expected_codewords_ = header.total_cw;
                    LOG_MODEM(INFO, "[%s] OFDM: CW0 decoded, expecting %d codewords",
                              log_prefix_.c_str(), ofdm_expected_codewords_);
                }
            }
        }
    }

    // Wait for all codewords
    if (ofdm_expected_codewords_ > 0 && num_codewords < (size_t)ofdm_expected_codewords_) {
        return;
    }

    if (ofdm_expected_codewords_ == 0 && is_synced) {
        return; // CW0 failed but still synced, keep waiting
    }

    // --- Process complete frame ---
    auto accumulated = std::move(ofdm_accumulated_soft_bits_);
    ofdm_accumulated_soft_bits_.clear();
    int expected = ofdm_expected_codewords_;
    ofdm_expected_codewords_ = 0;

    num_codewords = accumulated.size() / LDPC_BLOCK;
    if (expected > 0 && (size_t)expected < num_codewords) {
        num_codewords = expected;
    }

    // First decode CW0 to get frame type for rate selection
    v2::FrameType frame_type = v2::FrameType::PROBE;
    {
        std::vector<float> cw0_bits(accumulated.begin(), accumulated.begin() + LDPC_BLOCK);
        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, CodeRate::R1_4, v2::BYTES_PER_CODEWORD, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) frame_type = header.type;
            }
        }
    }

    bool use_adaptive = connected_ && v2::isDataFrame(frame_type);
    auto result = decodeCodewords(accumulated, num_codewords, data_code_rate_,
                                   use_adaptive, log_prefix_.c_str());

    LOG_MODEM(INFO, "[%s] OFDM: Decoded %d/%d codewords",
              log_prefix_.c_str(), result.codewords_ok,
              result.codewords_ok + result.codewords_failed);

    if (result.success && !result.frame_data.empty()) {
        deliverFrame(result.frame_data);
        notifyFrameParsed(result.frame_data, result.frame_type);
    } else {
        updateStats([&](LoopbackStats& s) {
            if (result.codewords_failed > 0) s.frames_failed++;
        });
    }

    ofdm_demodulator_->reset();
}

// ============================================================================
// DPSK BUFFER PROCESSING (Connected Mode)
// ============================================================================

void ModemEngine::processRxBuffer_DPSK() {
    using namespace rx_constants;

    auto samples = getBufferSnapshot();
    if (samples.size() < MIN_SAMPLES_FOR_DPSK) return;

    SampleSpan span(samples.data(), samples.size());
    int preamble_start = dpsk_demodulator_->findPreamble(span);

    if (preamble_start < 0) {
        // Trim old samples if buffer too large
        if (samples.size() > MAX_BUFFER_BEFORE_TRIM) {
            consumeSamples(samples.size() - BUFFER_TRIM_TARGET);
        }
        return;
    }

    LOG_MODEM(INFO, "[%s] DPSK: Found preamble at %d", log_prefix_.c_str(), preamble_start);

    DetectedFrame frame;
    frame.data_start = preamble_start;
    frame.waveform = protocol::WaveformMode::DPSK;
    frame.timestamp = std::chrono::steady_clock::now();

    rx_frame_state_.active = true;
    rx_frame_state_.frame = frame;

    bool success = rxDecodeDPSK(frame);

    if (!success) {
        updateStats([](LoopbackStats& s) { s.frames_failed++; });
    }

    rx_frame_state_.clear();
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

bool ModemEngine::waitForSamples(size_t required) {
    using namespace rx_constants;

    while (rx_decode_running_) {
        if (getBufferSize() >= required) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_SAMPLES_MS));
    }
    return false;
}

std::vector<float> ModemEngine::deinterleaveCodewords(const std::vector<float>& soft_bits) {
    std::vector<float> result;

    for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size();
         i += v2::LDPC_CODEWORD_BITS) {
        std::vector<float> cw_bits(soft_bits.begin() + i,
                                    soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
        auto deinterleaved = interleaver_.deinterleave(cw_bits);
        result.insert(result.end(), deinterleaved.begin(), deinterleaved.end());
    }

    // Handle partial codeword at end
    size_t full_cws = (soft_bits.size() / v2::LDPC_CODEWORD_BITS) * v2::LDPC_CODEWORD_BITS;
    if (full_cws < soft_bits.size()) {
        result.insert(result.end(), soft_bits.begin() + full_cws, soft_bits.end());
    }

    return result;
}

void ModemEngine::deliverFrame(const Bytes& frame_data) {
    updateStats([](LoopbackStats& s) {
        s.frames_received++;
        s.synced = true;
    });

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        rx_data_queue_.push(frame_data);
    }

    if (raw_data_callback_) {
        raw_data_callback_(frame_data);
    }

    last_rx_complete_time_ = std::chrono::steady_clock::now();
}

void ModemEngine::notifyFrameParsed(const Bytes& frame_data, protocol::v2::FrameType frame_type) {
    if (!status_callback_) return;

    std::string type_str;
    switch (frame_type) {
        case v2::FrameType::PROBE: type_str = "PROBE"; break;
        case v2::FrameType::PROBE_ACK: type_str = "PROBE_ACK"; break;
        case v2::FrameType::CONNECT: type_str = "CONNECT"; break;
        case v2::FrameType::CONNECT_ACK: type_str = "CONNECT_ACK"; break;
        case v2::FrameType::DISCONNECT: type_str = "DISCONNECT"; break;
        case v2::FrameType::DATA: type_str = "DATA"; break;
        case v2::FrameType::ACK: type_str = "ACK"; break;
        case v2::FrameType::NACK: type_str = "NACK"; break;
        default: type_str = "FRAME"; break;
    }

    // Try parsing for more details
    auto connect = v2::ConnectFrame::deserialize(frame_data);
    if (connect) {
        status_callback_("[" + type_str + "] " + connect->getSrcCallsign() +
                        " -> " + connect->getDstCallsign());
        return;
    }

    auto data = v2::DataFrame::deserialize(frame_data);
    if (data && !data->payload.empty()) {
        std::string msg = data->payloadAsText();
        if (!msg.empty() && data_callback_) {
            data_callback_(msg);
        }
        status_callback_("[MESSAGE] \"" + msg + "\"");
        return;
    }

    status_callback_("[" + type_str + "] Received");
}

void ModemEngine::updateStats(std::function<void(LoopbackStats&)> updater) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    updater(stats_);
}

} // namespace gui
} // namespace ultra
