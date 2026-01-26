// RxPipeline - Implementation of common RX decode flow

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
}

void RxPipeline::setDataMode(CodeRate rate, bool connected) {
    data_code_rate_ = rate;
    connected_ = connected;
}

void RxPipeline::reset() {
    accumulated_soft_bits_.clear();
    expected_codewords_ = 0;
}

int RxPipeline::getAccumulatedCodewords() const {
    return static_cast<int>(accumulated_soft_bits_.size() / v2::LDPC_CODEWORD_BITS);
}

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

    // First decode CW0 to get expected codeword count
    std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
    Bytes cw0_data;
    if (!decodeSingleCodeword(cw0_bits, CodeRate::R1_4, v2::BYTES_PER_CODEWORD, cw0_data)) {
        LOG_MODEM(INFO, "[%s] RxPipeline: CW0 LDPC decode failed", log_prefix_.c_str());
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

    LOG_MODEM(INFO, "[%s] RxPipeline: CW0 OK, expecting %d codewords, type=%d",
              log_prefix_.c_str(), expected, static_cast<int>(header.frame_type));

    // Check if we have all codewords
    if (num_codewords < expected) {
        LOG_MODEM(DEBUG, "[%s] RxPipeline: Waiting for more codewords (%d/%d)",
                  log_prefix_.c_str(), num_codewords, expected);
        // Store for accumulation
        expected_codewords_ = expected;
        accumulated_soft_bits_ = soft_bits;
        return result;
    }

    // Decode remaining codewords
    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(expected, false);
    cw_status.data.resize(expected);
    cw_status.decoded[0] = true;
    cw_status.data[0] = cw0_data;

    // Determine code rate for CW1+
    bool use_adaptive = connected_ && v2::isDataFrame(header.frame_type);
    CodeRate cw1_rate = use_adaptive ? data_code_rate_ : CodeRate::R1_4;
    size_t cw1_bytes = v2::getBytesPerCodeword(cw1_rate);

    for (int i = 1; i < expected; i++) {
        std::vector<float> cw_bits(
            soft_bits.begin() + i * LDPC_BLOCK,
            soft_bits.begin() + (i + 1) * LDPC_BLOCK
        );

        Bytes cw_data;
        if (decodeSingleCodeword(cw_bits, cw1_rate, cw1_bytes, cw_data)) {
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
    // For now, pass through - interleaving logic to be implemented
    // based on the channel interleaver configuration
    return soft_bits;
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
