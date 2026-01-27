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

// Decode multiple codewords - ALL codewords use the same rate
// Protocol rate selection:
// - CONNECT/CONNECT_ACK (pre-negotiation): R1/4 for ALL codewords
// - DATA frames (post-negotiation): negotiated rate for ALL codewords
static FrameDecodeResult decodeCodewords(
    const std::vector<float>& soft_bits,
    size_t num_codewords,
    CodeRate data_rate,      // Rate for DATA frames (ignored if !use_adaptive)
    bool use_adaptive,       // true = DATA frame (use data_rate), false = R1/4
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

    // ALL codewords use the same rate
    CodeRate rate = use_adaptive ? data_rate : CodeRate::R1_4;
    size_t bytes_per_cw = v2::getBytesPerCodeword(rate);

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(
            soft_bits.begin() + i * LDPC_BLOCK,
            soft_bits.begin() + (i + 1) * LDPC_BLOCK
        );

        Bytes cw_data;
        if (decodeSingleCodeword(cw_bits, rate, bytes_per_cw, cw_data)) {
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            result.codewords_ok++;
            LOG_MODEM(DEBUG, "[%s] CW%zu: LDPC OK (rate=%d, got %zu bytes)",
                      log_prefix, i, static_cast<int>(rate), cw_data.size());

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
            LOG_MODEM(DEBUG, "[%s] CW%zu: LDPC FAIL (rate=%d)", log_prefix, i, static_cast<int>(rate));
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

    // Reset demodulator state for each new frame
    // This prevents state from previous frame affecting current decode
    mc_dpsk_demodulator_->reset();

    // Use Multi-Carrier DPSK for frequency diversity
    const int symbol_samples = mc_dpsk_config_.samples_per_symbol;
    const int bits_per_sym = mc_dpsk_config_.num_carriers * mc_dpsk_config_.bits_per_symbol;
    const int samples_per_codeword = (v2::LDPC_CODEWORD_BITS + bits_per_sym - 1) / bits_per_sym * symbol_samples;

    // PING: 4 bytes = 32 bits
    const int ping_bits = v2::PingFrame::SIZE * 8;
    const int ping_symbols = (ping_bits + bits_per_sym - 1) / bits_per_sym;
    const int samples_for_ping = ping_symbols * symbol_samples;

    // Calculate training sequence length for chirp frames (MC-DPSK training)
    const int training_samples = mc_dpsk_config_.training_symbols * symbol_samples;

    // --- Step 1: Wait for ping samples and check for PING ---
    LOG_MODEM(DEBUG, "[%s] DPSK decode: waiting for %d + %d = %d samples (buf=%zu)",
              log_prefix_.c_str(), frame.data_start, samples_for_ping,
              frame.data_start + samples_for_ping, getBufferSize());
    if (!waitForSamples(frame.data_start + samples_for_ping)) {
        // Only warn if thread is still running (actual failure vs normal shutdown)
        if (rx_decode_running_) {
            LOG_MODEM(WARN, "[%s] DPSK decode: waitForSamples failed (buf=%zu, needed=%d)",
                      log_prefix_.c_str(), getBufferSize(), frame.data_start + samples_for_ping);
        }
        return false;
    }

    {
        auto buffer = getBufferSnapshot();
        LOG_MODEM(DEBUG, "[%s] DPSK decode: got buffer snapshot, size=%zu", log_prefix_.c_str(), buffer.size());

        // Set up demodulator reference
        // frame.data_start points to where DATA begins (after ref symbol)
        // Reference symbol is one symbol before data start
        int ref_offset = frame.data_start - symbol_samples;
        if (ref_offset < 0 || ref_offset + symbol_samples > (int)buffer.size()) {
            LOG_MODEM(WARN, "[%s] DPSK: Invalid ref_offset %d (buf=%zu)", log_prefix_.c_str(), ref_offset, buffer.size());
            return false;
        }

        if (frame.has_chirp_preamble) {
            // Chirp frame: Layout is [CHIRP][TRAINING][REF][DATA...]
            // Use dual chirp CFO estimate (accurate ±50 Hz)
            int training_offset = ref_offset - training_samples;
            if (training_offset < 0) {
                LOG_MODEM(WARN, "[%s] DPSK: Invalid training_offset %d", log_prefix_.c_str(), training_offset);
                return false;
            }
            LOG_MODEM(DEBUG, "[%s] DPSK decode: training=%d, ref=%d, data=%d, dual_chirp_cfo=%.1f Hz",
                      log_prefix_.c_str(), training_offset, ref_offset, frame.data_start, frame.cfo_hz);

            // Set CFO from dual chirp estimate BEFORE processing training
            // ALWAYS call setCFO() to reset accumulated CFO from previous frames
            // This prevents CFO drift when processing multiple frames
            mc_dpsk_demodulator_->setCFO(frame.cfo_hz);
            if (std::abs(frame.cfo_hz) > 0.1f) {
                LOG_MODEM(INFO, "[%s] DPSK: Using dual chirp CFO=%.1f Hz for correction",
                          log_prefix_.c_str(), frame.cfo_hz);
            }

            // IMPORTANT: Apply CFO correction to training/ref samples BEFORE processing
            // Otherwise processTraining() sees uncorrected signal and estimates wrong residual
            Samples training_corrected(buffer.data() + training_offset,
                                       buffer.data() + training_offset + training_samples);
            Samples ref_corrected(buffer.data() + ref_offset,
                                  buffer.data() + ref_offset + symbol_samples);
            if (std::abs(frame.cfo_hz) > 0.1f) {
                mc_dpsk_demodulator_->applyCFO(training_corrected);
                mc_dpsk_demodulator_->applyCFO(ref_corrected);
            }
            SampleSpan training_span(training_corrected.data(), training_corrected.size());
            SampleSpan ref_span(ref_corrected.data(), ref_corrected.size());
            mc_dpsk_demodulator_->processTraining(training_span);

            // CFO sanity check: with dual chirp, we trust larger CFO values
            // Training-based CFO should confirm the dual chirp estimate
            float training_cfo = mc_dpsk_demodulator_->getEstimatedCFO();
            float cfo_diff = std::abs(training_cfo - frame.cfo_hz);
            if (std::abs(frame.cfo_hz) < 0.1f && std::abs(training_cfo) > 5.0f) {
                // No dual chirp CFO but high training CFO - likely false positive
                LOG_MODEM(INFO, "[%s] DPSK: Rejecting frame - training CFO %.1f Hz too high (no dual chirp)",
                          log_prefix_.c_str(), training_cfo);
                mc_dpsk_demodulator_->reset();
                return false;
            }
            // If we have dual chirp CFO, use it (already set above)
            // The demodulator will correct for it during demodulation

            mc_dpsk_demodulator_->setReference(ref_span);
        } else {
            // Legacy Barker-13 frame: just use reference symbol
            LOG_MODEM(DEBUG, "[%s] DPSK decode: barker frame, ref_offset=%d", log_prefix_.c_str(), ref_offset);
            SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
            mc_dpsk_demodulator_->setReference(ref_sym);
        }

        LOG_MODEM(DEBUG, "[%s] MC-DPSK decode: demodulating ping span at %d, CFO=%.1f Hz",
                  log_prefix_.c_str(), frame.data_start, mc_dpsk_demodulator_->getEstimatedCFO());

        // Apply CFO correction to samples before demodulation (using Hilbert transform)
        Samples ping_samples(buffer.data() + frame.data_start,
                            buffer.data() + frame.data_start + samples_for_ping);
        mc_dpsk_demodulator_->applyCFO(ping_samples);
        SampleSpan ping_span(ping_samples.data(), ping_samples.size());
        auto ping_soft = mc_dpsk_demodulator_->demodulateSoft(ping_span);

        if (detectPing(ping_soft)) {
            consumeSamples(frame.data_start + samples_for_ping);
            mc_dpsk_demodulator_->reset();

            if (ping_received_callback_) {
                ping_received_callback_(getCurrentSNR());
            }

            updateStats([](LoopbackStats& s) { s.frames_received++; });
            last_rx_complete_time_ = std::chrono::steady_clock::now();
            return true;
        }

        // Not a PING - assume it's an LDPC-encoded v2 frame
        // The magic bytes will be visible after LDPC decoding, not in raw soft bits
        LOG_MODEM(DEBUG, "[%s] DPSK: Not PING, trying v2 LDPC decode", log_prefix_.c_str());
    }

    // --- Step 2: Decode CW0 to get codeword count ---
    if (!waitForSamples(frame.data_start + samples_per_codeword)) return false;

    int expected_codewords = 1;
    v2::FrameType frame_type = v2::FrameType::PROBE;

    {
        auto buffer = getBufferSnapshot();

        // Set up demodulator reference
        int ref_offset = frame.data_start - symbol_samples;
        if (ref_offset < 0 || ref_offset + symbol_samples > (int)buffer.size()) {
            LOG_MODEM(WARN, "[%s] DPSK: Invalid ref_offset %d for CW0", log_prefix_.c_str(), ref_offset);
            return false;
        }

        if (frame.has_chirp_preamble) {
            // Chirp frame: use training + ref for CFO estimation
            // Set dual chirp CFO before processing training
            int training_offset = ref_offset - training_samples;
            if (training_offset < 0) {
                LOG_MODEM(WARN, "[%s] DPSK: Invalid training_offset %d for CW0", log_prefix_.c_str(), training_offset);
                return false;
            }
            // ALWAYS reset CFO to prevent accumulation across frames
            mc_dpsk_demodulator_->setCFO(frame.cfo_hz);
            // Apply CFO correction BEFORE processTraining/setReference
            Samples training_corrected(buffer.data() + training_offset,
                                       buffer.data() + training_offset + training_samples);
            Samples ref_corrected(buffer.data() + ref_offset,
                                  buffer.data() + ref_offset + symbol_samples);
            if (std::abs(frame.cfo_hz) > 0.1f) {
                mc_dpsk_demodulator_->applyCFO(training_corrected);
                mc_dpsk_demodulator_->applyCFO(ref_corrected);
            }
            SampleSpan training_span(training_corrected.data(), training_corrected.size());
            SampleSpan ref_span(ref_corrected.data(), ref_corrected.size());
            mc_dpsk_demodulator_->processTraining(training_span);
            mc_dpsk_demodulator_->setReference(ref_span);
        } else {
            // Legacy Barker-13 frame
            SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
            mc_dpsk_demodulator_->setReference(ref_sym);
        }

        // Apply CFO correction before demodulation
        Samples cw0_samples(buffer.data() + frame.data_start,
                           buffer.data() + frame.data_start + samples_per_codeword);
        mc_dpsk_demodulator_->applyCFO(cw0_samples);
        SampleSpan cw0_span(cw0_samples.data(), cw0_samples.size());
        auto cw0_soft = mc_dpsk_demodulator_->demodulateSoft(cw0_span);

        if (cw0_soft.size() < v2::LDPC_CODEWORD_BITS) {
            LOG_MODEM(WARN, "[%s] DPSK: CW0 demod failed", log_prefix_.c_str());
            consumeSamples(frame.data_start + samples_per_codeword);
            mc_dpsk_demodulator_->reset();
            return false;
        }

        // Deinterleave and decode CW0
        // Note: DPSK mode does NOT use interleaving (TX only interleaves for OFDM)
        std::vector<float> cw0_bits(cw0_soft.begin(), cw0_soft.begin() + v2::LDPC_CODEWORD_BITS);
        // No interleaving for DPSK

        // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
        CodeRate probe_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
        size_t probe_bytes = v2::getBytesPerCodeword(probe_rate);

        LOG_MODEM(DEBUG, "[%s] DPSK CW0: calling LDPC with %zu bits (rate=%d)",
                  log_prefix_.c_str(), cw0_bits.size(), static_cast<int>(probe_rate));

        Bytes cw0_data;
        if (!decodeSingleCodeword(cw0_bits, probe_rate, probe_bytes, cw0_data)) {
            LOG_MODEM(INFO, "[%s] DPSK: CW0 LDPC failed", log_prefix_.c_str());
            consumeSamples(frame.data_start + samples_per_codeword);
            mc_dpsk_demodulator_->reset();
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

    // Set up demodulator with training sequence for CFO correction (critical for fading!)
    int ref_offset = frame.data_start - symbol_samples;
    if (ref_offset < 0 || ref_offset + symbol_samples > (int)buffer.size()) {
        LOG_MODEM(WARN, "[%s] DPSK: Invalid ref_offset %d for full frame", log_prefix_.c_str(), ref_offset);
        return false;
    }

    if (frame.has_chirp_preamble) {
        // Use training sequence for CFO estimation (essential for fading channels)
        // ALWAYS reset CFO to prevent accumulation across frames
        mc_dpsk_demodulator_->setCFO(frame.cfo_hz);
        int training_offset = ref_offset - training_samples;
        if (training_offset >= 0) {
            // Apply CFO correction BEFORE processTraining/setReference
            Samples training_corrected(buffer.data() + training_offset,
                                       buffer.data() + training_offset + training_samples);
            Samples ref_corrected(buffer.data() + ref_offset,
                                  buffer.data() + ref_offset + symbol_samples);
            if (std::abs(frame.cfo_hz) > 0.1f) {
                mc_dpsk_demodulator_->applyCFO(training_corrected);
                mc_dpsk_demodulator_->applyCFO(ref_corrected);
            }
            SampleSpan training_span(training_corrected.data(), training_corrected.size());
            SampleSpan ref_span(ref_corrected.data(), ref_corrected.size());
            mc_dpsk_demodulator_->processTraining(training_span);
            mc_dpsk_demodulator_->setReference(ref_span);
        } else {
            SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
            mc_dpsk_demodulator_->setReference(ref_sym);
        }
    } else {
        SampleSpan ref_sym(buffer.data() + ref_offset, symbol_samples);
        mc_dpsk_demodulator_->setReference(ref_sym);
    }

    // Apply CFO correction before demodulation
    Samples data_samples(buffer.data() + frame.data_start,
                        buffer.data() + frame.data_start + total_samples);
    mc_dpsk_demodulator_->applyCFO(data_samples);
    SampleSpan data_span(data_samples.data(), data_samples.size());
    auto soft_bits = mc_dpsk_demodulator_->demodulateSoft(data_span);

    // Note: DPSK mode does NOT use interleaving (TX only interleaves for OFDM)

    // Decode all codewords
    // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
    bool use_adaptive = connected_;
    auto result = decodeCodewords(soft_bits, expected_codewords, data_code_rate_,
                                   use_adaptive, log_prefix_.c_str());

    consumeSamples(frame.data_start + total_samples);
    mc_dpsk_demodulator_->reset();

    if (result.success) {
        // Save CFO for connected mode - peer's frequency offset persists
        if (std::abs(frame.cfo_hz) > 0.1f) {
            peer_cfo_hz_ = frame.cfo_hz;
            LOG_MODEM(INFO, "[%s] DPSK: Saving peer CFO=%.1f Hz for connected mode",
                      log_prefix_.c_str(), peer_cfo_hz_);
        }
        deliverFrame(result.frame_data);
        notifyFrameParsed(result.frame_data, result.frame_type);
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
    // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
    if (ofdm_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw0_bits(ofdm_accumulated_soft_bits_.begin(),
                                     ofdm_accumulated_soft_bits_.begin() + LDPC_BLOCK);

        CodeRate probe_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
        size_t probe_bytes = v2::getBytesPerCodeword(probe_rate);

        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, probe_rate, probe_bytes, cw0_data)) {
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

    // Determine rate based on connection state
    // Connected: expect DATA frames at negotiated rate
    // Not connected: expect CONNECT frames at R1/4
    bool use_adaptive = connected_;
    CodeRate decode_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
    size_t decode_bytes = v2::getBytesPerCodeword(decode_rate);

    // Decode CW0 to get frame type (for logging/notification)
    v2::FrameType frame_type = v2::FrameType::PROBE;
    {
        std::vector<float> cw0_bits(accumulated.begin(), accumulated.begin() + LDPC_BLOCK);
        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, decode_rate, decode_bytes, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) frame_type = header.type;
            }
        }
    }

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
// OTFS FRAME DECODE
// ============================================================================
// OTFS architecture: 1 OTFS frame = 1 LDPC codeword
// Multi-codeword messages = multiple OTFS frames in sequence
// RX accumulates codewords until complete message received

void ModemEngine::processRxBuffer_OTFS() {
    using namespace rx_constants;
    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    // Get samples from buffer (keep demodulator persistent!)
    std::vector<float> samples;
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        if (rx_sample_buffer_.size() < MIN_SAMPLES_FOR_OFDM_SYNC &&
            otfs_accumulated_soft_bits_.empty()) {
            return;
        }
        samples = std::move(rx_sample_buffer_);
        rx_sample_buffer_.clear();
    }

    // Ensure demodulator exists with correct config (only create once)
    bool need_tf_eq = (waveform_mode_ == protocol::WaveformMode::OTFS_EQ);
    if (!otfs_demodulator_ || otfs_config_.tf_equalization != need_tf_eq) {
        otfs_config_.tf_equalization = need_tf_eq;
        otfs_demodulator_ = std::make_unique<OTFSDemodulator>(otfs_config_);
        LOG_MODEM(INFO, "[%s] OTFS: Created demodulator (TF-EQ=%s)",
                  log_prefix_.c_str(), need_tf_eq ? "true" : "false");
    }

    // Feed samples and process all available frames
    bool frame_ready = false;
    if (!samples.empty()) {
        LOG_MODEM(DEBUG, "[%s] OTFS: Feeding %zu samples to demodulator",
                  log_prefix_.c_str(), samples.size());
        SampleSpan span(samples.data(), samples.size());
        frame_ready = otfs_demodulator_->process(span);
    }

    // Loop to extract all ready frames from the demodulator
    while (true) {
        // If no frame ready from previous call, try processing more
        if (!frame_ready) {
            SampleSpan empty_span{};  // Empty span
            frame_ready = otfs_demodulator_->process(empty_span);
        }

        if (!frame_ready) {
            break;  // No more frames ready
        }

        // Get soft bits from this OTFS frame (should be ~1 codeword worth)
        auto soft_bits = otfs_demodulator_->getSoftBits();
        if (soft_bits.size() < LDPC_BLOCK) {
            LOG_MODEM(WARN, "[%s] OTFS: Insufficient bits (%zu < %zu)",
                      log_prefix_.c_str(), soft_bits.size(), LDPC_BLOCK);
            // Demodulator already back in SEARCHING state, just continue
            frame_ready = false;
            continue;
        }

        // OTFS has built-in time-frequency diversity, no interleaving needed

        // Take exactly 1 codeword worth of bits
        std::vector<float> cw_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);

        // Accumulate this codeword's soft bits
        otfs_accumulated_soft_bits_.insert(otfs_accumulated_soft_bits_.end(),
                                            cw_bits.begin(), cw_bits.end());

        size_t accumulated_cw = otfs_accumulated_soft_bits_.size() / LDPC_BLOCK;
        // Log soft bit statistics for debugging
        float sum = 0, min_val = cw_bits[0], max_val = cw_bits[0];
        for (float v : cw_bits) {
            sum += v;
            if (v < min_val) min_val = v;
            if (v > max_val) max_val = v;
        }
        float mean = sum / cw_bits.size();
        LOG_MODEM(INFO, "[%s] OTFS: Frame decoded, accumulated %zu codewords (soft bits: mean=%.2f, min=%.2f, max=%.2f)",
                  log_prefix_.c_str(), accumulated_cw, mean, min_val, max_val);

        // After first frame, probe CW0 to learn expected count
        // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
        if (otfs_expected_codewords_ == 0 && accumulated_cw == 1) {
            std::vector<float> cw0_bits(otfs_accumulated_soft_bits_.begin(),
                                         otfs_accumulated_soft_bits_.begin() + LDPC_BLOCK);
            CodeRate probe_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
            size_t probe_bytes = v2::getBytesPerCodeword(probe_rate);

            Bytes cw0_data;
            if (decodeSingleCodeword(cw0_bits, probe_rate, probe_bytes, cw0_data)) {
                auto cw_info = v2::identifyCodeword(cw0_data);
                if (cw_info.type == v2::CodewordType::HEADER) {
                    auto header = v2::parseHeader(cw0_data);
                    if (header.valid) {
                        otfs_expected_codewords_ = header.total_cw;
                        LOG_MODEM(INFO, "[%s] OTFS: CW0 decoded early, expecting %d codewords",
                                  log_prefix_.c_str(), otfs_expected_codewords_);
                    }
                }
            }
        }

        // Stop accumulating if we have enough
        if (otfs_expected_codewords_ > 0 && accumulated_cw >= (size_t)otfs_expected_codewords_) {
            LOG_MODEM(INFO, "[%s] OTFS: Got all %d expected codewords, stopping",
                      log_prefix_.c_str(), otfs_expected_codewords_);
            break;  // Don't process more frames
        }

        // Demodulator already transitions to SEARCHING state after returning soft bits
        // Do NOT call reset() as that would clear the sample buffer with remaining frames
        frame_ready = false;
    }

    size_t num_codewords = otfs_accumulated_soft_bits_.size() / LDPC_BLOCK;
    if (num_codewords == 0) {
        return;
    }

    // Probe CW0 to get expected codeword count (if not already known)
    // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
    if (otfs_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw0_bits(otfs_accumulated_soft_bits_.begin(),
                                     otfs_accumulated_soft_bits_.begin() + LDPC_BLOCK);

        CodeRate probe_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
        size_t probe_bytes = v2::getBytesPerCodeword(probe_rate);

        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, probe_rate, probe_bytes, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) {
                    otfs_expected_codewords_ = header.total_cw;
                    LOG_MODEM(INFO, "[%s] OTFS: CW0 decoded, expecting %d total codewords",
                              log_prefix_.c_str(), otfs_expected_codewords_);
                }
            }
        } else {
            LOG_MODEM(INFO, "[%s] OTFS: CW0 LDPC decode failed", log_prefix_.c_str());
        }
    }

    // Wait for all codewords if we know how many to expect
    if (otfs_expected_codewords_ > 0 && num_codewords < (size_t)otfs_expected_codewords_) {
        // Don't reset - let demodulator keep searching for next frame
        return;
    }

    // If CW0 failed and we only have 1 codeword, discard and try again
    if (otfs_expected_codewords_ == 0 && num_codewords == 1) {
        LOG_MODEM(INFO, "[%s] OTFS: CW0 failed, discarding", log_prefix_.c_str());
        otfs_accumulated_soft_bits_.clear();
        return;
    }

    // --- All codewords received, decode complete message ---
    auto accumulated = std::move(otfs_accumulated_soft_bits_);
    otfs_accumulated_soft_bits_.clear();
    int expected = otfs_expected_codewords_;
    otfs_expected_codewords_ = 0;

    num_codewords = accumulated.size() / LDPC_BLOCK;
    if (expected > 0 && (size_t)expected < num_codewords) {
        num_codewords = expected;
    }

    // Determine rate based on connection state
    // Connected: expect DATA frames at negotiated rate
    // Not connected: expect CONNECT frames at R1/4
    bool use_adaptive = connected_;
    CodeRate decode_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
    size_t decode_bytes = v2::getBytesPerCodeword(decode_rate);

    // Decode CW0 to get frame type (for logging/notification)
    v2::FrameType frame_type = v2::FrameType::PROBE;
    {
        std::vector<float> cw0_bits(accumulated.begin(), accumulated.begin() + LDPC_BLOCK);
        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, decode_rate, decode_bytes, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) frame_type = header.type;
            }
        }
    }

    auto result = decodeCodewords(accumulated, num_codewords, data_code_rate_,
                                   use_adaptive, log_prefix_.c_str());

    LOG_MODEM(INFO, "[%s] OTFS: Decoded %d/%d codewords",
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

    otfs_demodulator_->reset();
}

// ============================================================================
// DPSK BUFFER PROCESSING (Connected Mode)
// Follows OFDM pattern: feed samples to demodulator, check for frame ready
// ============================================================================

void ModemEngine::processRxBuffer_DPSK() {
    using namespace rx_constants;
    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    // Get samples and move to demodulator (like OFDM)
    std::vector<float> samples;
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        bool has_pending = mc_dpsk_demodulator_->isSynced() ||
                          mc_dpsk_demodulator_->hasPendingData() ||
                          dpsk_expected_codewords_ > 0;

        // Log when we have significant samples to process
        if (rx_sample_buffer_.size() > MIN_SAMPLES_FOR_DPSK) {
            LOG_MODEM(INFO, "[%s] DPSK RX: buf=%zu (>min), pending=%d, synced=%d",
                      log_prefix_.c_str(), rx_sample_buffer_.size(), has_pending,
                      mc_dpsk_demodulator_->isSynced());
        }

        if (!has_pending && rx_sample_buffer_.size() < MIN_SAMPLES_FOR_DPSK) {
            return;
        }
        samples = std::move(rx_sample_buffer_);
        rx_sample_buffer_.clear();
    }

    if (samples.empty() && !mc_dpsk_demodulator_->hasPendingData() &&
        dpsk_accumulated_soft_bits_.empty() && dpsk_expected_codewords_ == 0) {
        return;
    }

    // Process through demodulator (handles chirp sync internally)
    bool was_synced = mc_dpsk_demodulator_->isSynced();
    SampleSpan span(samples.data(), samples.size());
    bool frame_ready = mc_dpsk_demodulator_->process(span);
    bool is_synced = mc_dpsk_demodulator_->isSynced();

    if (samples.size() > 10000 || is_synced || frame_ready) {
        LOG_MODEM(INFO, "[%s] DPSK demod: fed %zu, synced=%d->%d, ready=%d, corr=%.3f",
                  log_prefix_.c_str(), samples.size(), was_synced, is_synced,
                  frame_ready, mc_dpsk_demodulator_->getLastChirpCorrelation());
    }

    // Handle sync loss mid-frame
    if (was_synced && !is_synced && !dpsk_accumulated_soft_bits_.empty()) {
        LOG_MODEM(INFO, "[%s] DPSK: Lost sync mid-frame, discarding", log_prefix_.c_str());
        dpsk_accumulated_soft_bits_.clear();
        dpsk_expected_codewords_ = 0;
    }

    if (is_synced) {
        updateStats([](LoopbackStats& s) { s.synced = true; });
    }

    // Accumulate soft bits when frame ready
    if (frame_ready) {
        auto soft_bits = mc_dpsk_demodulator_->getSoftBits();
        LOG_MODEM(INFO, "[%s] DPSK: frame_ready, getSoftBits() returned %zu bits",
                  log_prefix_.c_str(), soft_bits.size());
        if (!soft_bits.empty()) {
            // No interleaving for DPSK
            dpsk_accumulated_soft_bits_.insert(dpsk_accumulated_soft_bits_.end(),
                                               soft_bits.begin(), soft_bits.end());
            LOG_MODEM(INFO, "[%s] DPSK: accumulated %zu total bits (%zu codewords)",
                      log_prefix_.c_str(), dpsk_accumulated_soft_bits_.size(),
                      dpsk_accumulated_soft_bits_.size() / v2::LDPC_CODEWORD_BITS);
        }
    }

    // Check if we have enough bits for decoding
    size_t num_codewords = dpsk_accumulated_soft_bits_.size() / LDPC_BLOCK;
    LOG_MODEM(DEBUG, "[%s] DPSK: accumulated_bits=%zu, num_codewords=%zu, expected=%d",
              log_prefix_.c_str(), dpsk_accumulated_soft_bits_.size(), num_codewords, dpsk_expected_codewords_);
    if (num_codewords == 0) return;

    // Decode first codeword to get frame header
    // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
    if (dpsk_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw0_bits(dpsk_accumulated_soft_bits_.begin(),
                                    dpsk_accumulated_soft_bits_.begin() + LDPC_BLOCK);

        CodeRate probe_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
        size_t probe_bytes = v2::getBytesPerCodeword(probe_rate);

        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, probe_rate, probe_bytes, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) {
                    dpsk_expected_codewords_ = header.total_cw;
                    LOG_MODEM(INFO, "[%s] DPSK: Header valid, expecting %d codewords",
                              log_prefix_.c_str(), dpsk_expected_codewords_);
                } else {
                    // Invalid header - discard this codeword
                    LOG_MODEM(DEBUG, "[%s] DPSK: Invalid header, discarding", log_prefix_.c_str());
                    dpsk_accumulated_soft_bits_.erase(dpsk_accumulated_soft_bits_.begin(),
                                                      dpsk_accumulated_soft_bits_.begin() + LDPC_BLOCK);
                    return;
                }
            } else {
                // Not a header codeword - discard
                LOG_MODEM(DEBUG, "[%s] DPSK: Not a header codeword, discarding", log_prefix_.c_str());
                dpsk_accumulated_soft_bits_.erase(dpsk_accumulated_soft_bits_.begin(),
                                                  dpsk_accumulated_soft_bits_.begin() + LDPC_BLOCK);
                return;
            }
        } else {
            // LDPC decode failed
            LOG_MODEM(DEBUG, "[%s] DPSK: CW0 LDPC failed", log_prefix_.c_str());
            dpsk_accumulated_soft_bits_.erase(dpsk_accumulated_soft_bits_.begin(),
                                              dpsk_accumulated_soft_bits_.begin() + LDPC_BLOCK);
            updateStats([](LoopbackStats& s) { s.frames_failed++; });
            return;
        }
    }

    // Check if we have all expected codewords
    if (dpsk_expected_codewords_ > 0 && num_codewords >= (size_t)dpsk_expected_codewords_) {
        LOG_MODEM(INFO, "[%s] DPSK: Decoding %d codewords", log_prefix_.c_str(), dpsk_expected_codewords_);

        // Extract soft bits for all codewords
        std::vector<float> frame_soft_bits(
            dpsk_accumulated_soft_bits_.begin(),
            dpsk_accumulated_soft_bits_.begin() + dpsk_expected_codewords_ * LDPC_BLOCK);

        // Decode all codewords
        // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
        bool use_adaptive = connected_;
        auto result = decodeCodewords(frame_soft_bits, dpsk_expected_codewords_, data_code_rate_,
                                      use_adaptive, log_prefix_.c_str());

        LOG_MODEM(INFO, "[%s] DPSK: decode result: success=%d, ok=%d, failed=%d, data_size=%zu",
                  log_prefix_.c_str(), result.success, result.codewords_ok, result.codewords_failed,
                  result.frame_data.size());

        // Clear ALL accumulated bits after decode (not just the expected codewords)
        // The demodulator may return extra trailing bits that would corrupt the next frame
        dpsk_accumulated_soft_bits_.clear();
        dpsk_expected_codewords_ = 0;

        if (result.success) {
            LOG_MODEM(INFO, "[%s] DPSK: Frame decoded successfully, delivering %zu bytes",
                      log_prefix_.c_str(), result.frame_data.size());
            deliverFrame(result.frame_data);
            notifyFrameParsed(result.frame_data, result.frame_type);
            updateStats([](LoopbackStats& s) { s.frames_received++; });
        } else {
            LOG_MODEM(WARN, "[%s] DPSK: Frame decode FAILED", log_prefix_.c_str());
            updateStats([](LoopbackStats& s) { s.frames_failed++; });
        }
    }
}

// ============================================================================
// OFDM_CHIRP FRAME DECODE (Connected Mode)
// Uses chirp preamble for robust timing sync, then OFDM DQPSK demodulation
// ============================================================================

void ModemEngine::processRxBuffer_OFDM_CHIRP() {
    using namespace rx_constants;
    constexpr size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;

    // Calculate symbol duration from config
    const size_t symbol_samples = config_.getSymbolDuration();  // CP + FFT

    // Get samples
    std::vector<float> samples;
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        bool has_pending = ofdm_demodulator_->isSynced() ||
                          ofdm_demodulator_->hasPendingData() ||
                          ofdm_expected_codewords_ > 0 ||
                          ofdm_chirp_found_;

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

    // If we haven't found chirp yet, search for it
    fprintf(stderr, "[OFDM_CHIRP-DBG] Check: ofdm_chirp_found_=%d, isSynced=%d\n",
            ofdm_chirp_found_ ? 1 : 0, ofdm_demodulator_->isSynced() ? 1 : 0);
    if (!ofdm_chirp_found_ && !ofdm_demodulator_->isSynced()) {
        const size_t chirp_samples = chirp_sync_->getTotalSamples();
        const size_t training_samples = symbol_samples * 2;  // 2 LTS symbols

        // Need chirp + training + at least one data symbol
        size_t min_for_detection = chirp_samples + training_samples + symbol_samples;
        fprintf(stderr, "[OFDM_CHIRP-DBG] Chirp detection: samples=%zu, min_for_detection=%zu\n",
                samples.size(), min_for_detection);
        if (samples.size() < min_for_detection) {
            // Put samples back
            fprintf(stderr, "[OFDM_CHIRP-DBG] Not enough samples, returning\n");
            std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
            rx_sample_buffer_.insert(rx_sample_buffer_.begin(), samples.begin(), samples.end());
            return;
        }

        // Search for chirp using dual chirp detection (up + down chirp)
        // This gives robust CFO estimation down to -20 dB SNR with ±2 Hz accuracy!
        fprintf(stderr, "[OFDM_CHIRP-DBG] Calling detectDualChirp...\n");
        SampleSpan search_span(samples.data(), samples.size());
        auto chirp_result = chirp_sync_->detectDualChirp(search_span, 0.15f);
        fprintf(stderr, "[OFDM_CHIRP-DBG] detectDualChirp returned success=%d\n", chirp_result.success ? 1 : 0);

        if (!chirp_result.success) {
            // No chirp found - consume older samples to prevent buffer growth
            // Keep enough for next detection attempt
            size_t keep = std::min(samples.size(), chirp_samples * 2);
            size_t discard = samples.size() > keep ? samples.size() - keep : 0;
            if (discard > 0) {
                std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
                rx_sample_buffer_.insert(rx_sample_buffer_.begin(),
                                        samples.begin() + discard, samples.end());
            } else {
                std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
                rx_sample_buffer_.insert(rx_sample_buffer_.begin(), samples.begin(), samples.end());
            }
            return;
        }

        // Chirp found! Get CFO-corrected position and CFO estimate
        int chirp_start = chirp_result.up_chirp_start;
        float cfo_hz = chirp_result.cfo_hz;
        float chirp_corr = std::max(chirp_result.up_correlation, chirp_result.down_correlation);

        ofdm_chirp_found_ = true;
        size_t chirp_end_offset = chirp_start + chirp_samples;

        LOG_MODEM(INFO, "[%s] OFDM_CHIRP: Dual chirp found at %d (corr=%.3f), CFO=%.1f Hz, training at %zu",
                  log_prefix_.c_str(), chirp_start, chirp_corr, cfo_hz, chirp_end_offset);

        // Discard samples before chirp end (keep training + data)
        fprintf(stderr, "[OFDM_CHIRP-DBG] Chirp found: start=%d, chirp_samples=%zu, end_offset=%zu, samples_before=%zu\n",
                chirp_start, chirp_samples, chirp_end_offset, samples.size());
        if (chirp_end_offset > 0 && chirp_end_offset < samples.size()) {
            samples.erase(samples.begin(), samples.begin() + chirp_end_offset);
            fprintf(stderr, "[OFDM_CHIRP-DBG] After erase: samples_after=%zu (for OFDM)\n", samples.size());
        }

        // === APPLY CFO CORRECTION ===
        // ALWAYS set CFO from chirp estimate to reset any accumulated CFO from previous frames
        // This matches MC-DPSK pattern: trust chirp CFO, threshold only for logging
        ofdm_demodulator_->setFrequencyOffset(cfo_hz);
        if (std::abs(cfo_hz) > 0.5f) {
            LOG_MODEM(INFO, "[%s] OFDM_CHIRP: Using chirp CFO=%.2f Hz for correction", log_prefix_.c_str(), cfo_hz);
        } else {
            LOG_MODEM(DEBUG, "[%s] OFDM_CHIRP: Chirp CFO=%.2f Hz (small but applied)", log_prefix_.c_str(), cfo_hz);
        }
    }

    // If chirp was previously found but samples were returned to buffer
    if (ofdm_chirp_found_ && !ofdm_demodulator_->isSynced() && samples.empty()) {
        return;
    }

    // Process through OFDM demodulator using pre-synced path (bypasses Schmidl-Cox)
    bool was_synced = ofdm_demodulator_->isSynced();

    // processPresynced expects samples starting at first LTS training symbol
    // It will use first 2 symbols for channel estimation, then demodulate the rest
    // IMPORTANT: Limit samples to reasonable frame size to avoid demodulating noise/margins
    // Conservative estimate: 4 codewords with OFDM_CHIRP (30 data carriers, 60 bits/sym DQPSK)
    // 4 * 648 bits / 60 bits/sym = 44 data symbols + 2 training = 46 symbols * 564 = 25,944
    // Use 35000 for safety margin
    constexpr size_t MAX_FRAME_SAMPLES = 35000;
    size_t process_samples = std::min(samples.size(), MAX_FRAME_SAMPLES);
    fprintf(stderr, "[OFDM_CHIRP-DBG] RX config: carriers=%d, FFT=%d, symlen=%d, use_pilots=%d\n",
            config_.getDataCarriers(), config_.fft_size, config_.getSymbolDuration(),
            config_.use_pilots ? 1 : 0);
    fprintf(stderr, "[OFDM_CHIRP-DBG] Calling processPresynced with %zu samples (from %zu total)\n",
            process_samples, samples.size());
    // Print first 10 samples
    fprintf(stderr, "[OFDM_CHIRP-DBG] First 10 samples for OFDM: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
            samples.size() > 0 ? samples[0] : 0.0f, samples.size() > 1 ? samples[1] : 0.0f,
            samples.size() > 2 ? samples[2] : 0.0f, samples.size() > 3 ? samples[3] : 0.0f,
            samples.size() > 4 ? samples[4] : 0.0f, samples.size() > 5 ? samples[5] : 0.0f,
            samples.size() > 6 ? samples[6] : 0.0f, samples.size() > 7 ? samples[7] : 0.0f,
            samples.size() > 8 ? samples[8] : 0.0f, samples.size() > 9 ? samples[9] : 0.0f);
    SampleSpan span(samples.data(), process_samples);
    bool frame_ready = ofdm_demodulator_->processPresynced(span, 2);
    fprintf(stderr, "[OFDM_CHIRP-DBG] processPresynced returned frame_ready=%d\n", frame_ready ? 1 : 0);
    bool is_synced = true;  // processPresynced always "syncs"

    // Handle sync loss mid-frame (shouldn't happen with processPresynced but be safe)
    if (was_synced && !ofdm_demodulator_->isSynced() && !ofdm_accumulated_soft_bits_.empty()) {
        LOG_MODEM(INFO, "[%s] OFDM_CHIRP: Lost sync mid-frame, discarding", log_prefix_.c_str());
        ofdm_accumulated_soft_bits_.clear();
        ofdm_expected_codewords_ = 0;
        ofdm_chirp_found_ = false;
    }

    if (is_synced) {
        updateStats([this](LoopbackStats& s) {
            s.snr_db = ofdm_demodulator_->getEstimatedSNR();
            s.synced = true;
        });
    }

    // Accumulate soft bits (getSoftBits returns 648 at a time, so loop to get all)
    if (frame_ready) {
        size_t total_bits = 0;
        int batch = 0;
        while (true) {
            auto soft_bits = ofdm_demodulator_->getSoftBits();
            if (soft_bits.empty()) break;

            if (batch == 0 && soft_bits.size() >= 10) {
                fprintf(stderr, "[OFDM_CHIRP-DBG] First 10 soft bits: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                        soft_bits[0], soft_bits[1], soft_bits[2], soft_bits[3], soft_bits[4],
                        soft_bits[5], soft_bits[6], soft_bits[7], soft_bits[8], soft_bits[9]);
            }
            batch++;
            total_bits += soft_bits.size();

            // OFDM_CHIRP uses interleaving like regular OFDM
            if (interleaving_enabled_) {
                auto deinterleaved = deinterleaveCodewords(soft_bits);
                ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                   deinterleaved.begin(), deinterleaved.end());
            } else {
                ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                   soft_bits.begin(), soft_bits.end());
            }
        }

        LOG_MODEM(INFO, "[%s] OFDM_CHIRP: frame_ready, got %zu soft bits, accumulated %zu total",
                  log_prefix_.c_str(), total_bits, ofdm_accumulated_soft_bits_.size());

        // Reset for next frame
        ofdm_chirp_found_ = false;
        ofdm_demodulator_->reset();

        // Save remaining samples for next frame detection
        // process_samples was consumed for this frame, keep the rest
        if (samples.size() > process_samples) {
            std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
            rx_sample_buffer_.insert(rx_sample_buffer_.begin(),
                                    samples.begin() + process_samples, samples.end());
            LOG_MODEM(DEBUG, "[%s] OFDM_CHIRP: Saved %zu samples for next frame",
                      log_prefix_.c_str(), samples.size() - process_samples);
        }
    }

    size_t num_codewords = ofdm_accumulated_soft_bits_.size() / LDPC_BLOCK;
    LOG_MODEM(DEBUG, "[%s] OFDM_CHIRP: accumulated=%zu, num_codewords=%zu, expected=%d",
              log_prefix_.c_str(), ofdm_accumulated_soft_bits_.size(), num_codewords, ofdm_expected_codewords_);
    if (num_codewords == 0) return;

    // Probe CW0 to get expected count
    // Use negotiated rate if connected (DATA frames), else R1/4 (CONNECT frames)
    if (ofdm_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw0_bits(ofdm_accumulated_soft_bits_.begin(),
                                     ofdm_accumulated_soft_bits_.begin() + LDPC_BLOCK);

        CodeRate probe_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
        size_t probe_bytes = v2::getBytesPerCodeword(probe_rate);

        LOG_MODEM(INFO, "[%s] OFDM_CHIRP: Probing CW0 (first 5 soft bits: %.2f %.2f %.2f %.2f %.2f)",
                  log_prefix_.c_str(),
                  cw0_bits.size() > 0 ? cw0_bits[0] : 0,
                  cw0_bits.size() > 1 ? cw0_bits[1] : 0,
                  cw0_bits.size() > 2 ? cw0_bits[2] : 0,
                  cw0_bits.size() > 3 ? cw0_bits[3] : 0,
                  cw0_bits.size() > 4 ? cw0_bits[4] : 0);

        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, probe_rate, probe_bytes, cw0_data)) {
            LOG_MODEM(INFO, "[%s] OFDM_CHIRP: CW0 LDPC OK, first bytes: %02x %02x %02x %02x",
                      log_prefix_.c_str(),
                      cw0_data.size() > 0 ? cw0_data[0] : 0,
                      cw0_data.size() > 1 ? cw0_data[1] : 0,
                      cw0_data.size() > 2 ? cw0_data[2] : 0,
                      cw0_data.size() > 3 ? cw0_data[3] : 0);
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) {
                    ofdm_expected_codewords_ = header.total_cw;
                    LOG_MODEM(INFO, "[%s] OFDM_CHIRP: CW0 decoded, expecting %d codewords",
                              log_prefix_.c_str(), ofdm_expected_codewords_);
                }
            }
        } else {
            LOG_MODEM(WARN, "[%s] OFDM_CHIRP: CW0 LDPC FAILED", log_prefix_.c_str());
        }
    }

    // Wait for all codewords
    if (ofdm_expected_codewords_ > 0 && num_codewords < (size_t)ofdm_expected_codewords_) {
        return;
    }

    if (ofdm_expected_codewords_ == 0) {
        return; // CW0 failed, keep waiting
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

    // Determine rate based on connection state
    // Connected: expect DATA frames at negotiated rate
    // Not connected: expect CONNECT frames at R1/4
    bool use_adaptive = connected_;
    CodeRate decode_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;
    size_t decode_bytes = v2::getBytesPerCodeword(decode_rate);

    // Decode CW0 to get frame type (for logging/notification)
    v2::FrameType frame_type = v2::FrameType::PROBE;
    {
        std::vector<float> cw0_bits(accumulated.begin(), accumulated.begin() + LDPC_BLOCK);
        Bytes cw0_data;
        if (decodeSingleCodeword(cw0_bits, decode_rate, decode_bytes, cw0_data)) {
            auto cw_info = v2::identifyCodeword(cw0_data);
            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) frame_type = header.type;
            }
        }
    }

    auto result = decodeCodewords(accumulated, num_codewords, data_code_rate_,
                                   use_adaptive, log_prefix_.c_str());

    LOG_MODEM(INFO, "[%s] OFDM_CHIRP: Decoded %d/%d codewords",
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

    if (!channel_interleaver_) {
        // No interleaver configured - return as-is
        return std::vector<float>(soft_bits.begin(), soft_bits.end());
    }

    for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size();
         i += v2::LDPC_CODEWORD_BITS) {
        std::vector<float> cw_bits(soft_bits.begin() + i,
                                    soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
        auto deinterleaved = channel_interleaver_->deinterleave(cw_bits);
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
