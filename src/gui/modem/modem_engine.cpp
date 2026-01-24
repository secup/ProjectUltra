// ModemEngine - Main implementation
// Constructor, destructor, configuration, and TX functions

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

    // CRITICAL: Disable pilots for DQPSK mode - uses all 30 carriers for data
    // This doubles throughput (30 data carriers vs 15 with pilots)
    // DQPSK is differential and doesn't need pilots for channel estimation
    config_.use_pilots = false;

    encoder_ = std::make_unique<LDPCEncoder>(config_.code_rate);

    // OFDM modulator uses config (TX modulation is determined per-frame)
    ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);

    // RX starts in disconnected state, so use DQPSK R1/4 for link establishment
    // This will be switched to negotiated mode when setConnected(true) is called
    ModemConfig rx_config = config_;
    rx_config.modulation = Modulation::DQPSK;
    rx_config.code_rate = CodeRate::R1_4;
    rx_config.use_pilots = false;  // DQPSK doesn't need pilots
    decoder_ = std::make_unique<LDPCDecoder>(rx_config.code_rate);
    ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

    // OTFS modulator/demodulator (used for data frames when negotiated)
    // OTFS uses delay-Doppler grid: M delay bins × N Doppler bins
    // 1 OTFS frame = 1 LDPC codeword (648 bits = 324 QPSK symbols)
    // M=32, N=16 = 512 symbols capacity (enough for 1 codeword + pilots)
    otfs_config_.M = 32;   // Delay bins
    otfs_config_.N = 16;   // Doppler bins = OFDM symbols per frame
    otfs_config_.fft_size = config_.fft_size;
    otfs_config_.cp_length = 64;  // Fixed CP for OTFS (different from OFDM)
    otfs_config_.sample_rate = config_.sample_rate;
    otfs_config_.center_freq = config_.center_freq;
    otfs_config_.modulation = Modulation::QPSK;  // Default for OTFS
    otfs_modulator_ = std::make_unique<OTFSModulator>(otfs_config_);
    otfs_demodulator_ = std::make_unique<OTFSDemodulator>(otfs_config_);

    // DPSK modulator/demodulator (used for low SNR: -11 to 17 dB)
    // Default to medium preset for connection attempts (DQPSK 62.5 baud)
    // This matches the setConnectWaveform(DPSK) configuration
    dpsk_config_ = dpsk_presets::medium();
    dpsk_modulator_ = std::make_unique<DPSKModulator>(dpsk_config_);
    dpsk_demodulator_ = std::make_unique<DPSKDemodulator>(dpsk_config_);

    // Initialize audio filters
    rebuildFilters();

    // Start RX threads (acquisition + decode)
    startAcquisitionThread();
    startRxDecodeThread();
}

ModemEngine::~ModemEngine() {
    // Stop threads in reverse order
    stopRxDecodeThread();
    stopAcquisitionThread();
}

// ============================================================================
// CONFIGURATION
// ============================================================================

void ModemEngine::setConfig(const ModemConfig& config) {
    LOG_MODEM(INFO, "setConfig called: new code_rate=%d, modulation=%d",
              static_cast<int>(config.code_rate), static_cast<int>(config.modulation));
    config_ = config;

    encoder_->setRate(config.code_rate);
    // BUG FIX: Don't change decoder rate here - it should stay at R1_4 for disconnected mode
    // The decoder rate should only change when setConnected(true) is called
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
    // Keep M=32 fixed (1 codeword per OTFS frame)
    // OTFS uses fixed CP=64 (different from OFDM)
    otfs_config_.M = 32;
    otfs_config_.fft_size = config_.fft_size;
    otfs_config_.cp_length = 64;  // Fixed for OTFS
    otfs_config_.sample_rate = config_.sample_rate;
    otfs_config_.center_freq = config_.center_freq;
    otfs_config_.modulation = Modulation::QPSK;
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

// ============================================================================
// TX: TRANSMIT
// ============================================================================

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

    LOG_MODEM(INFO, "[%s] TX: Input %zu bytes, first 4: %02x %02x %02x %02x, v2=%d",
              log_prefix_.c_str(),
              data.size(),
              data.size() > 0 ? data[0] : 0,
              data.size() > 1 ? data[1] : 0,
              data.size() > 2 ? data[2] : 0,
              data.size() > 3 ? data[3] : 0,
              is_v2_frame);

    Bytes to_modulate;

    // Determine if this is a DATA frame (used for modulation selection later)
    bool is_data_frame = false;
    if (is_v2_frame && data.size() >= 3 && connected_) {
        uint8_t frame_type = data[2];
        is_data_frame = (frame_type >= 0x30 && frame_type <= 0x33);
    }

    if (is_v2_frame) {
        // === V2 Frame Path ===
        // Encoding strategy:
        // - Control frames: All CWs use R1/4 for reliability
        // - DATA frames: CW0 uses R1/4 (header), CW1+ use negotiated rate for throughput

        // Make mutable copy for possible header modification
        Bytes frame_data = data;

        std::vector<Bytes> encoded_cws;

        if (is_data_frame && data_code_rate_ != CodeRate::R1_4) {
            // DATA frame with adaptive rate: CW0 at R1/4, CW1+ at data_code_rate_
            size_t bytes_per_cw_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);  // 20
            size_t bytes_per_cw_data = v2::getBytesPerCodeword(data_code_rate_);
            size_t cw1_payload_size = bytes_per_cw_data - v2::DATA_CW_HEADER_SIZE;

            // Calculate correct total_cw for this rate and update the serialized frame
            if (frame_data.size() >= v2::DataFrame::HEADER_SIZE) {
                uint16_t payload_len = (static_cast<uint16_t>(frame_data[13]) << 8) | frame_data[14];
                uint8_t correct_total_cw = v2::DataFrame::calculateCodewords(payload_len, data_code_rate_);
                uint8_t old_total_cw = frame_data[12];

                if (correct_total_cw != old_total_cw) {
                    // Update total_cw in the serialized frame
                    frame_data[12] = correct_total_cw;

                    // Recalculate header CRC (CRC of bytes 0-14)
                    uint16_t new_hcrc = v2::ControlFrame::calculateCRC(frame_data.data(), 15);
                    frame_data[15] = (new_hcrc >> 8) & 0xFF;
                    frame_data[16] = new_hcrc & 0xFF;

                    LOG_MODEM(DEBUG, "TX v2: Fixed total_cw %d -> %d for rate %s",
                              old_total_cw, correct_total_cw, codeRateToString(data_code_rate_));
                }
            }

            // Encode CW0 (header) at R1/4
            LDPCEncoder encoder_r14(CodeRate::R1_4);
            {
                Bytes cw0(bytes_per_cw_r14, 0);
                size_t cw0_bytes = std::min(bytes_per_cw_r14, frame_data.size());
                std::memcpy(cw0.data(), frame_data.data(), cw0_bytes);
                LOG_MODEM(INFO, "[%s] TX CW0 input (first 8): %02x %02x %02x %02x %02x %02x %02x %02x",
                          log_prefix_.c_str(),
                          cw0[0], cw0[1], cw0[2], cw0[3], cw0[4], cw0[5], cw0[6], cw0[7]);
                auto encoded = encoder_r14.encode(cw0);
                LOG_MODEM(INFO, "[%s] TX CW0 LDPC output (first 8): %02x %02x %02x %02x %02x %02x %02x %02x",
                          log_prefix_.c_str(),
                          encoded[0], encoded[1], encoded[2], encoded[3],
                          encoded[4], encoded[5], encoded[6], encoded[7]);
                encoded_cws.push_back(encoded);
            }

            // Encode CW1+ at adaptive rate
            LDPCEncoder encoder_data(data_code_rate_);
            size_t offset = bytes_per_cw_r14;  // Start after CW0's data
            uint8_t cw_index = 1;

            while (offset < frame_data.size()) {
                Bytes cw(bytes_per_cw_data, 0);
                cw[0] = v2::DATA_CW_MARKER;
                cw[1] = cw_index;

                size_t remaining = frame_data.size() - offset;
                size_t chunk_size = std::min(cw1_payload_size, remaining);
                std::memcpy(cw.data() + v2::DATA_CW_HEADER_SIZE, frame_data.data() + offset, chunk_size);

                encoded_cws.push_back(encoder_data.encode(cw));
                offset += cw1_payload_size;
                cw_index++;
            }

            LOG_MODEM(INFO, "TX v2: DATA frame %zu bytes -> %zu codewords (CW0: R1/4, CW1+: %s)",
                      frame_data.size(), encoded_cws.size(), codeRateToString(data_code_rate_));
        } else {
            // Control frame or R1/4 data: use R1/4 for all codewords
            encoded_cws = v2::encodeFrameWithLDPC(frame_data, CodeRate::R1_4);
            LOG_MODEM(INFO, "TX v2: %zu bytes -> %zu codewords (all R1/4)",
                      frame_data.size(), encoded_cws.size());
        }

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
        // Use connected code rate if still connected OR for disconnect ACK
        CodeRate tx_code_rate = (connected_ || use_connected_waveform_once_) ? data_code_rate_ : CodeRate::R1_4;

        encoder_->setRate(tx_code_rate);
        Bytes encoded = encoder_->encode(data);

        LOG_MODEM(INFO, "TX raw: %zu bytes -> %zu encoded (rate=%d)",
                  data.size(), encoded.size(), static_cast<int>(tx_code_rate));

        to_modulate = interleaving_enabled_ ? interleaver_.interleave(encoded) : encoded;
    }

    // Modulation selection
    // When connected, ALL frames (including control) use negotiated modulation
    // so RX demodulator (also configured to negotiated mode) can decode them.
    // Robustness is handled by code rate: CW0 header always uses R1/4.
    // IMPORTANT: When use_connected_waveform_once_ is set (for DISCONNECT ACK),
    // we must also use the connected modulation since the remote is still expecting it.
    Modulation tx_modulation = Modulation::DQPSK;
    if (connected_ || use_connected_waveform_once_) {
        tx_modulation = data_modulation_;
        LOG_MODEM(INFO, "[%s] TX: Using %s modulation %s",
                  log_prefix_.c_str(),
                  connected_ ? "negotiated" : "preserved (disconnect ACK)",
                  modulationToString(tx_modulation));
    }

    // Determine which waveform to use
    LOG_MODEM(INFO, "[%s] TX WAVEFORM DECISION: connected_=%d, handshake_complete_=%d, "
              "waveform_mode_=%d, connect_waveform_=%d, last_rx_waveform_=%d, use_once_=%d",
              log_prefix_.c_str(), connected_ ? 1 : 0, handshake_complete_ ? 1 : 0,
              static_cast<int>(waveform_mode_), static_cast<int>(connect_waveform_),
              static_cast<int>(last_rx_waveform_), use_connected_waveform_once_ ? 1 : 0);

    protocol::WaveformMode active_waveform;
    if (use_connected_waveform_once_) {
        active_waveform = waveform_mode_;
        use_connected_waveform_once_ = false;
        LOG_MODEM(INFO, "[%s] TX: use_connected_waveform_once_ -> using waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    } else if (!connected_) {
        active_waveform = connect_waveform_;
        LOG_MODEM(INFO, "[%s] TX: NOT connected -> using connect_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    } else if (!handshake_complete_) {
        active_waveform = last_rx_waveform_;
        LOG_MODEM(INFO, "[%s] TX: Handshake mode -> using last_rx_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    } else {
        active_waveform = waveform_mode_;
        LOG_MODEM(INFO, "[%s] TX: Connected+handshake -> using waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    }

    bool use_dpsk = (active_waveform == protocol::WaveformMode::DPSK);
    bool use_otfs = (active_waveform == protocol::WaveformMode::OTFS_EQ ||
                     active_waveform == protocol::WaveformMode::OTFS_RAW);

    Samples preamble, modulated;

    if (use_dpsk) {
        LOG_MODEM(INFO, "[%s] TX: Using DPSK modulation (%d-PSK, %d samples/sym)",
                  log_prefix_.c_str(), dpsk_config_.num_phases(), dpsk_config_.samples_per_symbol);
        preamble = dpsk_modulator_->generatePreamble();
        modulated = dpsk_modulator_->modulate(to_modulate);
    } else if (use_otfs) {
        // OTFS: 1 codeword per frame, multiple frames for multi-codeword messages
        // Each encoded codeword is 81 bytes (648 bits)
        constexpr size_t BYTES_PER_CODEWORD = 81;
        size_t num_codewords = (to_modulate.size() + BYTES_PER_CODEWORD - 1) / BYTES_PER_CODEWORD;

        LOG_MODEM(INFO, "[%s] TX: Using OTFS modulation (%s, M=%d, N=%d, %zu codewords = %zu frames)",
                  log_prefix_.c_str(),
                  active_waveform == protocol::WaveformMode::OTFS_EQ ? "TF-EQ" : "RAW",
                  otfs_config_.M, otfs_config_.N, num_codewords, num_codewords);

        // Configure OTFS equalization mode
        otfs_config_.tf_equalization = (active_waveform == protocol::WaveformMode::OTFS_EQ);
        otfs_modulator_ = std::make_unique<OTFSModulator>(otfs_config_);

        // Generate one OTFS frame per codeword
        const size_t INTER_FRAME_GAP = 480;  // 10ms gap between frames

        for (size_t cw = 0; cw < num_codewords; cw++) {
            // Extract this codeword's bytes
            size_t start = cw * BYTES_PER_CODEWORD;
            size_t end = std::min(start + BYTES_PER_CODEWORD, to_modulate.size());
            Bytes cw_data(to_modulate.begin() + start, to_modulate.begin() + end);

            // Pad if last codeword is partial
            while (cw_data.size() < BYTES_PER_CODEWORD) {
                cw_data.push_back(0);
            }

            // Map to DD grid and modulate
            auto dd_symbols = otfs_modulator_->mapToDD(cw_data, tx_modulation);
            auto frame_preamble = otfs_modulator_->generatePreamble();
            auto frame_data = otfs_modulator_->modulate(dd_symbols, tx_modulation);

            // Append preamble + data
            modulated.insert(modulated.end(), frame_preamble.begin(), frame_preamble.end());
            modulated.insert(modulated.end(), frame_data.begin(), frame_data.end());

            // Add gap between frames (except after last)
            if (cw + 1 < num_codewords) {
                modulated.resize(modulated.size() + INTER_FRAME_GAP, 0.0f);
            }
        }

        // preamble is empty for OTFS (already included in modulated)
        preamble.clear();
    } else {
        LOG_MODEM(INFO, "[%s] TX: Using OFDM modulation (%s)",
                  log_prefix_.c_str(), is_v2_frame ? "control frame" : "data frame");
        preamble = ofdm_modulator_->generatePreamble();
        modulated = ofdm_modulator_->modulate(to_modulate, tx_modulation);
    }

    // Combine lead-in + preamble + data + tail guard
    const size_t LEAD_IN_SAMPLES = 48000 * 150 / 1000;  // 150ms
    const size_t TAIL_SAMPLES = 576 * 2;
    std::vector<float> output;
    output.reserve(LEAD_IN_SAMPLES + preamble.size() + modulated.size() + TAIL_SAMPLES);

    output.resize(LEAD_IN_SAMPLES, 0.0f);
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());
    output.resize(output.size() + TAIL_SAMPLES, 0.0f);

    // Apply TX bandpass filter
    if (filter_config_.enabled && tx_filter_) {
        SampleSpan span(output.data(), output.size());
        output = tx_filter_->process(span);
    }

    // Scale for audio output
    float max_val = 0.0f;
    for (float s : output) {
        max_val = std::max(max_val, std::abs(s));
    }
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) {
            s *= scale;
        }
    }

    // Update stats
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_sent++;

        int bits_per_carrier = static_cast<int>(getBitsPerSymbol(config_.modulation));
        float code_rate = getCodeRateValue(config_.code_rate);
        float symbol_rate = config_.sample_rate / (float)config_.getSymbolDuration();
        stats_.throughput_bps = static_cast<int>(
            config_.getDataCarriers() * bits_per_carrier * code_rate * symbol_rate
        );
    }

    return output;
}

// ============================================================================
// PING/PONG PROBE (minimal presence check)
// ============================================================================

std::vector<float> ModemEngine::transmitPing() {
    namespace v2 = protocol::v2;

    // Get the "ULTR" magic bytes (4 bytes, no LDPC encoding)
    Bytes ping_data = v2::PingFrame::serialize();

    LOG_MODEM(INFO, "[%s] TX PING: %zu bytes raw DPSK (no LDPC)",
              log_prefix_.c_str(), ping_data.size());

    // Generate DPSK preamble and modulate raw bytes
    Samples preamble = dpsk_modulator_->generatePreamble();
    Samples modulated = dpsk_modulator_->modulate(ping_data);

    // Combine: lead-in silence + preamble + data + tail guard
    const size_t LEAD_IN_SAMPLES = 48000 * 100 / 1000;  // 100ms (shorter for ping)
    const size_t TAIL_SAMPLES = 576;  // Short tail
    std::vector<float> output;
    output.reserve(LEAD_IN_SAMPLES + preamble.size() + modulated.size() + TAIL_SAMPLES);

    output.resize(LEAD_IN_SAMPLES, 0.0f);
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());
    output.resize(output.size() + TAIL_SAMPLES, 0.0f);

    // Apply TX bandpass filter
    if (filter_config_.enabled && tx_filter_) {
        SampleSpan span(output.data(), output.size());
        output = tx_filter_->process(span);
    }

    // Scale for audio output
    float max_val = 0.0f;
    for (float s : output) {
        max_val = std::max(max_val, std::abs(s));
    }
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) {
            s *= scale;
        }
    }

    LOG_MODEM(INFO, "[%s] TX PING: Generated %zu samples (%.2f sec)",
              log_prefix_.c_str(), output.size(), output.size() / 48000.0f);

    return output;
}

std::vector<float> ModemEngine::transmitPong() {
    // Pong is identical to ping - context determines meaning
    // (Ping = initiator probe, Pong = responder reply)
    LOG_MODEM(INFO, "[%s] TX PONG (same as PING)", log_prefix_.c_str());
    return transmitPing();
}

// ============================================================================
// TEST SIGNAL GENERATION
// ============================================================================

std::vector<float> ModemEngine::generateTestTone(float duration_sec) {
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
    Bytes test_data(21);

    switch (pattern) {
        case 0:
            std::fill(test_data.begin(), test_data.end(), 0x00);
            LOG_MODEM(INFO, "TX Test Pattern: ALL ZEROS (%zu bytes)", test_data.size());
            break;
        case 1:
            {
                uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
                for (size_t i = 0; i < test_data.size(); i++) {
                    test_data[i] = deadbeef[i % 4];
                }
            }
            LOG_MODEM(INFO, "TX Test Pattern: DEADBEEF (%zu bytes)", test_data.size());
            break;
        case 2:
            std::fill(test_data.begin(), test_data.end(), 0x55);
            LOG_MODEM(INFO, "TX Test Pattern: ALTERNATING 0101 (%zu bytes)", test_data.size());
            break;
        default:
            std::fill(test_data.begin(), test_data.end(), 0xAA);
            LOG_MODEM(INFO, "TX Test Pattern: ALTERNATING 1010 (%zu bytes)", test_data.size());
    }

    // LDPC encode
    CodeRate saved_rate = encoder_->getRate();
    encoder_->setRate(CodeRate::R1_4);
    Bytes encoded = encoder_->encode(test_data);
    encoder_->setRate(saved_rate);
    LOG_MODEM(INFO, "TX Test: %zu bytes -> %zu encoded bytes (R1/4 forced)", test_data.size(), encoded.size());

    Samples preamble = ofdm_modulator_->generatePreamble();
    Samples modulated = ofdm_modulator_->modulate(encoded, Modulation::DQPSK);

    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());

    float max_val = 0.0f;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) s *= scale;
    }

    return output;
}

std::vector<float> ModemEngine::transmitRawOFDM(int pattern) {
    // Generate raw OFDM test (no LDPC) for layer-by-layer debugging
    size_t test_size = 81;  // Size of one R1/4 encoded codeword
    Bytes test_data(test_size);

    switch (pattern) {
        case 0:
            for (size_t i = 0; i < test_size; i++) {
                test_data[i] = (i % 2 == 0) ? 0xAA : 0x55;
            }
            LOG_MODEM(INFO, "TX Raw OFDM: AA/55 alternating (%zu bytes)", test_size);
            break;
        case 1:
            {
                uint8_t deadbeef[] = {0xDE, 0xAD, 0xBE, 0xEF};
                for (size_t i = 0; i < test_size; i++) {
                    test_data[i] = deadbeef[i % 4];
                }
            }
            LOG_MODEM(INFO, "TX Raw OFDM: DEADBEEF (%zu bytes)", test_size);
            break;
        default:
            std::fill(test_data.begin(), test_data.end(), 0xAA);
            LOG_MODEM(INFO, "TX Raw OFDM: ALL 0xAA (%zu bytes)", test_size);
    }

    Samples preamble = ofdm_modulator_->generatePreamble();
    Samples modulated = ofdm_modulator_->modulate(test_data, Modulation::DQPSK);

    std::vector<float> output;
    output.reserve(preamble.size() + modulated.size());
    output.insert(output.end(), preamble.begin(), preamble.end());
    output.insert(output.end(), modulated.begin(), modulated.end());

    float max_val = 0.0f;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0.0f) {
        float scale = 0.8f / max_val;
        for (float& s : output) s *= scale;
    }

    LOG_MODEM(INFO, "TX Raw OFDM: %zu bytes -> %zu samples",
              test_size, output.size());
    return output;
}

// ============================================================================
// STATUS & DATA ACCESS
// ============================================================================

bool ModemEngine::hasReceivedData() const {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return !rx_data_queue_.empty();
}

std::string ModemEngine::getReceivedText() {
    Bytes data = getReceivedData();
    std::string text(data.begin(), data.end());
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
    return ofdm_demodulator_->isSynced();
}

float ModemEngine::getCurrentSNR() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_.snr_db;
}

ChannelQuality ModemEngine::getChannelQuality() const {
    return ofdm_demodulator_->getChannelQuality();
}

std::vector<std::complex<float>> ModemEngine::getConstellationSymbols() const {
    return ofdm_demodulator_->getConstellationSymbols();
}

void ModemEngine::reset() {
    // Clear sample buffer
    {
        std::lock_guard<std::mutex> lock(rx_buffer_mutex_);
        rx_sample_buffer_.clear();
    }

    std::lock_guard<std::mutex> lock(rx_mutex_);
    std::queue<Bytes> empty;
    std::swap(rx_data_queue_, empty);

    ofdm_demodulator_->reset();
    adaptive_.reset();

    // Reset RX state and clear queues
    rx_frame_state_.clear();
    detected_frame_queue_.clear();
    use_connected_waveform_once_ = false;

    // Reset carrier sense
    channel_energy_.store(0.0f);

    {
        std::lock_guard<std::mutex> lock2(stats_mutex_);
        stats_ = LoopbackStats{};
    }
}

} // namespace gui
} // namespace ultra
