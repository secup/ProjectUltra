#include "modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include <cstring>
#include <algorithm>
#include <fstream>
#include <sstream>

namespace ultra {
namespace gui {

ModemEngine::ModemEngine() {
    config_ = presets::balanced();

    encoder_ = std::make_unique<LDPCEncoder>(config_.code_rate);

    // OFDM modulator uses config (TX modulation is determined per-frame)
    ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);

    // RX starts in disconnected state, so use DQPSK R1/4 for link establishment
    // This will be switched to negotiated mode when setConnected(true) is called
    ModemConfig rx_config = config_;
    rx_config.modulation = Modulation::DQPSK;
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

    // For RX: if disconnected, use DQPSK R1/4 (robust mode), not config's mode
    if (!connected_) {
        ModemConfig rx_config = config_;
        rx_config.modulation = Modulation::DQPSK;
        rx_config.code_rate = CodeRate::R1_4;
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);
        LOG_MODEM(INFO, "setConfig: RX using disconnected mode (DQPSK R1/4)");
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
    namespace v2 = protocol::v2;

    if (data.empty()) {
        return {};
    }

    // Check for v2 frame magic "UL" (0x55, 0x4C)
    bool is_v2_frame = (data.size() >= 2 && data[0] == 0x55 && data[1] == 0x4C);

    LOG_MODEM(INFO, "TX: Input %zu bytes, first 4: %02x %02x %02x %02x, v2=%d",
              data.size(),
              data.size() > 0 ? data[0] : 0,
              data.size() > 1 ? data[1] : 0,
              data.size() > 2 ? data[2] : 0,
              data.size() > 3 ? data[3] : 0,
              is_v2_frame);

    Bytes to_modulate;

    if (is_v2_frame) {
        // === V2 Frame Path ===
        // Use v2::encodeFrameWithLDPC which handles multi-codeword frames correctly
        auto encoded_cws = v2::encodeFrameWithLDPC(data);

        LOG_MODEM(INFO, "TX v2: %zu bytes -> %zu codewords", data.size(), encoded_cws.size());

        // Concatenate all encoded codewords (with optional interleaving per-codeword)
        for (const auto& cw : encoded_cws) {
            if (interleaving_enabled_) {
                Bytes interleaved = interleaver_.interleave(cw);
                to_modulate.insert(to_modulate.end(), interleaved.begin(), interleaved.end());
            } else {
                to_modulate.insert(to_modulate.end(), cw.begin(), cw.end());
            }
        }

        LOG_MODEM(INFO, "TX v2: Total encoded %zu bytes", to_modulate.size());
    } else {
        // === Raw Data Path (non-v2 frame) ===
        // Use adaptive code rate based on connection state
        CodeRate tx_code_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;

        encoder_->setRate(tx_code_rate);
        Bytes encoded = encoder_->encode(data);

        LOG_MODEM(INFO, "TX raw: %zu bytes -> %zu encoded (rate=%d)",
                  data.size(), encoded.size(), static_cast<int>(tx_code_rate));

        to_modulate = interleaving_enabled_ ? interleaver_.interleave(encoded) : encoded;
    }

    // All v2 frames use DQPSK R1/4, legacy frames also default to DQPSK
    Modulation tx_modulation = Modulation::DQPSK;
    if (!is_v2_frame && connected_) {
        tx_modulation = data_modulation_;
    }

    // Step 2: Generate preamble
    Samples preamble = ofdm_modulator_->generatePreamble();

    // Step 3: OFDM modulate with DQPSK
    Samples modulated = ofdm_modulator_->modulate(to_modulate, tx_modulation);

    // Step 4: Combine preamble + data + tail guard
    // Tail guard ensures demodulator processes the last OFDM symbol
    const size_t TAIL_SAMPLES = 576 * 2;  // 2 OFDM symbols of silence
    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size() + TAIL_SAMPLES);

    // Add preamble
    output.insert(output.end(), preamble.begin(), preamble.end());

    // Add modulated data
    output.insert(output.end(), modulated.begin(), modulated.end());

    // Add tail guard
    output.resize(output.size() + TAIL_SAMPLES, 0.0f);

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
        int bits_per_carrier = static_cast<int>(getBitsPerSymbol(config_.modulation));
        float code_rate = getCodeRateValue(config_.code_rate);
        float symbol_rate = config_.sample_rate / (float)config_.getSymbolDuration();
        stats_.throughput_bps = static_cast<int>(
            config_.getDataCarriers() * bits_per_carrier * code_rate * symbol_rate
        );
    }

    return output;
}

std::vector<float> ModemEngine::generateTestTone(float duration_sec) {
    // Generate a simple 1500 Hz tone for audio path testing
    size_t num_samples = static_cast<size_t>(config_.sample_rate * duration_sec);
    std::vector<float> tone(num_samples);

    float freq = 1500.0f;
    float phase = 0.0f;
    float phase_inc = 2.0f * M_PI * freq / config_.sample_rate;

    for (size_t i = 0; i < num_samples; i++) {
        tone[i] = 0.7f * std::sin(phase);
        phase += phase_inc;
        if (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
    }

    LOG_MODEM(INFO, "Generated test tone: %.1f Hz, %.1f sec, %zu samples",
              freq, duration_sec, num_samples);
    return tone;
}

std::vector<float> ModemEngine::transmitTestPattern(int pattern) {
    // Generate known data pattern for debugging
    // Uses full LDPC encoding so patterns can be verified on RX

    // Create test data: 21 bytes fits in one R1/4 LDPC codeword (k=162 bits = 20.25 bytes)
    Bytes test_data(21);

    switch (pattern) {
        case 0:  // All zeros
            std::fill(test_data.begin(), test_data.end(), 0x00);
            LOG_MODEM(INFO, "TX Test Pattern: ALL ZEROS (%zu bytes)", test_data.size());
            break;
        case 1:  // DEADBEEF pattern (non-trivial for testing)
            {
                uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
                for (size_t i = 0; i < test_data.size(); i++) {
                    test_data[i] = deadbeef[i % 4];
                }
            }
            LOG_MODEM(INFO, "TX Test Pattern: DEADBEEF (%zu bytes)", test_data.size());
            break;
        case 2:  // Alternating 01010101
            std::fill(test_data.begin(), test_data.end(), 0x55);
            LOG_MODEM(INFO, "TX Test Pattern: ALTERNATING 0101 (%zu bytes)", test_data.size());
            break;
        default:
            std::fill(test_data.begin(), test_data.end(), 0xAA);
            LOG_MODEM(INFO, "TX Test Pattern: ALTERNATING 1010 (%zu bytes)", test_data.size());
    }

    // LDPC encode - FORCE R1/4 to match RX decoder (which is hardcoded to R1/4)
    CodeRate saved_rate = encoder_->getRate();
    encoder_->setRate(CodeRate::R1_4);
    Bytes encoded = encoder_->encode(test_data);
    encoder_->setRate(saved_rate);  // Restore
    LOG_MODEM(INFO, "TX Test: %zu bytes -> %zu encoded bytes (R1/4 forced)", test_data.size(), encoded.size());

    // Generate preamble
    Samples preamble = ofdm_modulator_->generatePreamble();

    // Modulate with LDPC-encoded data (DQPSK to match RX link establishment mode)
    Samples modulated = ofdm_modulator_->modulate(encoded, Modulation::DQPSK);

    // Combine
    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());

    // Scale
    float max_val = 0.0f;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) s *= scale;
    }

    LOG_MODEM(INFO, "TX Test: preamble=%zu + data=%zu = %zu samples",
              preamble.size(), modulated.size(), output.size());

    return output;
}

std::vector<float> ModemEngine::transmitRawOFDM(int pattern) {
    // RAW OFDM test - NO LDPC encoding
    // This tests ONLY the OFDM layer for debugging

    // 81 bytes = 648 bits = fills OFDM symbols evenly with DQPSK (2 bits/symbol)
    Bytes test_data(81);

    if (pattern == 0) {
        // Alternating 0xAA 0x55
        for (size_t i = 0; i < test_data.size(); i++) {
            test_data[i] = (i % 2 == 0) ? 0xAA : 0x55;
        }
        LOG_MODEM(INFO, "TX Raw OFDM: %zu bytes (NO LDPC), pattern=0xAA 0x55", test_data.size());
    } else {
        // DEADBEEF pattern
        uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
        for (size_t i = 0; i < test_data.size(); i++) {
            test_data[i] = deadbeef[i % 4];
        }
        LOG_MODEM(INFO, "TX Raw OFDM: %zu bytes (NO LDPC), pattern=DEADBEEF", test_data.size());
    }

    // Generate preamble
    Samples preamble = ofdm_modulator_->generatePreamble();

    // Modulate directly (NO LDPC!)
    Samples modulated = ofdm_modulator_->modulate(test_data, Modulation::DQPSK);

    // Combine
    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());

    // Scale
    float max_val = 0.0f;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) s *= scale;
    }

    LOG_MODEM(INFO, "TX Raw OFDM: preamble=%zu + data=%zu = %zu samples (%.2fs)",
              preamble.size(), modulated.size(), output.size(), output.size() / 48000.0f);

    return output;
}

void ModemEngine::receiveAudio(const std::vector<float>& samples) {
    // FAST PATH: Only lock the pending mutex, just append samples
    // This is safe to call from SDL audio callback
    std::lock_guard<std::mutex> lock(rx_pending_mutex_);

    // Cap buffer size to prevent unbounded growth if main loop stalls
    if (rx_pending_samples_.size() + samples.size() > MAX_PENDING_SAMPLES) {
        size_t to_remove = rx_pending_samples_.size() + samples.size() - MAX_PENDING_SAMPLES;
        if (to_remove >= rx_pending_samples_.size()) {
            rx_pending_samples_.clear();
        } else {
            rx_pending_samples_.erase(rx_pending_samples_.begin(), rx_pending_samples_.begin() + to_remove);
        }
    }
    rx_pending_samples_.insert(rx_pending_samples_.end(), samples.begin(), samples.end());
}

size_t ModemEngine::injectSignalFromFile(const std::string& filepath) {
    // Read f32 samples from file and inject into RX buffer
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

    // Inject in chunks and process between each chunk to simulate real-time streaming
    // This prevents buffer overflow when injecting large files
    constexpr size_t CHUNK_SIZE = 48000;  // 1 second at 48kHz
    size_t offset = 0;
    while (offset < num_samples) {
        size_t chunk_end = std::min(offset + CHUNK_SIZE, num_samples);
        std::vector<float> chunk(samples.begin() + offset, samples.begin() + chunk_end);
        receiveAudio(chunk);
        // Process after each chunk to prevent buffer overflow
        while (pollRxAudio()) {
            // Keep processing until demodulator catches up
        }
        offset = chunk_end;
    }

    return num_samples;
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
    LOG_MODEM(INFO, "RX: Demod returned frame_ready=%d, synced=%d", frame_ready, ofdm_demodulator_->isSynced());

    // Update SNR continuously when synced (not just after decode)
    if (ofdm_demodulator_->isSynced()) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.snr_db = ofdm_demodulator_->getEstimatedSNR();
        stats_.synced = true;
    }

    // Accumulate soft bits from all codewords
    namespace v2 = protocol::v2;
    std::vector<float> accumulated_soft_bits;
    int codewords_collected = 0;

    while (frame_ready) {
        auto soft_bits = ofdm_demodulator_->getSoftBits();
        LOG_MODEM(INFO, "RX: Got %zu soft bits from demod, codewords_collected=%d",
                  soft_bits.size(), codewords_collected);
        if (!soft_bits.empty()) {
            codewords_collected++;
            auto to_accumulate = interleaving_enabled_ ? interleaver_.deinterleave(soft_bits) : soft_bits;
            accumulated_soft_bits.insert(accumulated_soft_bits.end(),
                                         to_accumulate.begin(), to_accumulate.end());
        }
        SampleSpan empty_span;
        frame_ready = ofdm_demodulator_->process(empty_span);
    }
    LOG_MODEM(INFO, "RX: Accumulated %zu soft bits, %d codewords",
              accumulated_soft_bits.size(), codewords_collected);

    // Process accumulated soft bits codeword by codeword
    if (accumulated_soft_bits.empty()) {
        // Reset demodulator even when no data - prevents stuck state
        ofdm_demodulator_->reset();
        return;
    }

    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;  // 648 bits
    size_t num_codewords = accumulated_soft_bits.size() / LDPC_BLOCK;
    if (num_codewords == 0) {
        ofdm_demodulator_->reset();
        return;
    }

    // Decode each codeword individually using CodewordStatus for tracking
    v2::CodewordStatus cw_status;
    int cw_success = 0, cw_failed = 0;
    int expected_total = 0;
    std::string frame_type_str;

    // First, decode CW0 to get expected codeword count
    {
        std::vector<float> cw_bits(accumulated_soft_bits.begin(),
                                    accumulated_soft_bits.begin() + LDPC_BLOCK);
        Bytes decoded = decoder_->decodeSoft(cw_bits);
        bool success = decoder_->lastDecodeSuccess();
        LOG_MODEM(INFO, "RX: CW0 LDPC decode %s, decoded %zu bytes, first 8: %02X %02X %02X %02X %02X %02X %02X %02X",
                  success ? "SUCCESS" : "FAILED", decoded.size(),
                  decoded.size() > 0 ? decoded[0] : 0,
                  decoded.size() > 1 ? decoded[1] : 0,
                  decoded.size() > 2 ? decoded[2] : 0,
                  decoded.size() > 3 ? decoded[3] : 0,
                  decoded.size() > 4 ? decoded[4] : 0,
                  decoded.size() > 5 ? decoded[5] : 0,
                  decoded.size() > 6 ? decoded[6] : 0,
                  decoded.size() > 7 ? decoded[7] : 0);

        if (success && decoded.size() >= v2::BYTES_PER_CODEWORD) {
            Bytes cw_data(decoded.begin(), decoded.begin() + v2::BYTES_PER_CODEWORD);
            auto cw_info = v2::identifyCodeword(cw_data);
            LOG_MODEM(INFO, "RX: CW0 type=%d, first bytes: %02X %02X %02X %02X",
                      static_cast<int>(cw_info.type),
                      cw_data[0], cw_data[1], cw_data[2], cw_data[3]);

            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw_data);
                if (header.valid) {
                    expected_total = header.total_cw;
                    LOG_MODEM(INFO, "RX: Header valid, expected_total=%d, frame_type=%d",
                              expected_total, static_cast<int>(header.type));
                }
            }
        }
    }

    // Limit to expected codewords (or available if header failed)
    if (expected_total > 0 && (size_t)expected_total < num_codewords) {
        num_codewords = expected_total;
    }

    // Notify start of frame
    if (status_callback_) {
        status_callback_("[FRAME] Received " + std::to_string(num_codewords) + " codewords");
    }

    cw_status.decoded.resize(num_codewords, false);
    cw_status.data.resize(num_codewords);

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(accumulated_soft_bits.begin() + i * LDPC_BLOCK,
                                    accumulated_soft_bits.begin() + (i + 1) * LDPC_BLOCK);

        Bytes decoded = decoder_->decodeSoft(cw_bits);
        bool success = decoder_->lastDecodeSuccess();

        if (success && decoded.size() >= v2::BYTES_PER_CODEWORD) {
            // Trim to codeword size and store
            Bytes cw_data(decoded.begin(), decoded.begin() + v2::BYTES_PER_CODEWORD);
            cw_status.decoded[i] = true;
            cw_status.data[i] = cw_data;
            cw_success++;

            // Identify codeword type
            auto cw_info = v2::identifyCodeword(cw_data);

            if (i == 0 && cw_info.type == v2::CodewordType::HEADER) {
                // Parse header to get frame info (already parsed above, just get type string)
                auto header = v2::parseHeader(cw_data);
                if (header.valid) {
                    expected_total = header.total_cw;
                    // Get frame type name
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

            // Report codeword progress
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

    // Try to reassemble and parse the complete frame
    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();

        if (!frame_data.empty()) {
            bool frame_parsed = false;

            // Try ControlFrame first (1 CW: PROBE, ACK, NACK, KEEPALIVE, BEACON)
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

            // Try ConnectFrame (3 CW: CONNECT, CONNECT_ACK, CONNECT_NAK, DISCONNECT)
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

            // Try DataFrame (variable CW: DATA, DATA_START, etc.)
            if (!frame_parsed) {
                auto parsed = v2::DataFrame::deserialize(frame_data);
                if (parsed && !parsed->payload.empty()) {
                    frame_parsed = true;
                    std::string msg(parsed->payload.begin(), parsed->payload.end());
                    // Clean non-printable
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

            // Update stats BEFORE callback (so protocol can read current SNR)
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
        }
    } else {
        // No frame decoded - still update stats for failed codewords
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (cw_failed > 0) stats_.frames_failed++;
    }

    // Reset demodulator to look for next preamble
    // This ensures clean state between frames and prevents accumulated timing/phase errors
    ofdm_demodulator_->reset();
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
        rx_config.modulation = Modulation::DQPSK;
        rx_config.code_rate = CodeRate::R1_4;

        decoder_->setRate(CodeRate::R1_4);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

        LOG_MODEM(INFO, "Switched to disconnected mode (RX: DQPSK R1/4)");
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
    // Conservative thresholds calibrated for REAL HF channels (not just AWGN)
    // HF has multipath fading that requires 3-6 dB extra margin vs AWGN
    // These match the thresholds in connection.cpp for consistency
    if (snr_db >= 30.0f) {
        // Excellent conditions (rare) - use high throughput
        mod = Modulation::QAM16;
        rate = CodeRate::R3_4;
    } else if (snr_db >= 25.0f) {
        // Very good conditions
        mod = Modulation::QAM16;
        rate = CodeRate::R2_3;
    } else if (snr_db >= 20.0f) {
        // Good conditions - sweet spot for speed
        mod = Modulation::DQPSK;
        rate = CodeRate::R2_3;
    } else if (snr_db >= 16.0f) {
        // Typical good HF - balanced speed/reliability
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_2;
    } else if (snr_db >= 12.0f) {
        // Typical HF - prioritize reliability
        mod = Modulation::DQPSK;
        rate = CodeRate::R1_4;
    } else if (snr_db >= 8.0f) {
        // Marginal conditions
        mod = Modulation::BPSK;
        rate = CodeRate::R1_4;
    } else {
        // Very poor conditions - maximum robustness
        mod = Modulation::BPSK;
        rate = CodeRate::R1_4;
    }
}

} // namespace gui
} // namespace ultra
