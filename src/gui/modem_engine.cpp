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
    otfs_config_.M = config_.num_carriers;
    otfs_config_.N = 16;  // 16 OFDM symbols per OTFS frame
    otfs_config_.fft_size = config_.fft_size;
    otfs_config_.cp_length = config_.getCyclicPrefix();
    otfs_config_.sample_rate = config_.sample_rate;
    otfs_config_.center_freq = config_.center_freq;
    otfs_modulator_ = std::make_unique<OTFSModulator>(otfs_config_);
    otfs_demodulator_ = std::make_unique<OTFSDemodulator>(otfs_config_);

    // MFSK modulator/demodulator (used for very low SNR: -17 to +5 dB reported)
    // Default to 8FSK medium mode (47 bps, works at 0 dB actual = -17 dB reported)
    mfsk_config_ = mfsk_presets::medium();
    mfsk_modulator_ = std::make_unique<MFSKModulator>(mfsk_config_);
    mfsk_demodulator_ = std::make_unique<MFSKDemodulator>(mfsk_config_);

    // DPSK modulator/demodulator (used for low-mid SNR: 0-15 dB)
    // Default to medium preset for connection attempts (DQPSK 62.5 baud)
    // This matches the setConnectWaveform(DPSK) configuration
    dpsk_config_ = dpsk_presets::medium();
    dpsk_modulator_ = std::make_unique<DPSKModulator>(dpsk_config_);
    dpsk_demodulator_ = std::make_unique<DPSKDemodulator>(dpsk_config_);

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
        //
        // This ensures the header (CW0) is always decodable, allowing the receiver
        // to determine the frame type before decoding remaining codewords.

        // Make mutable copy for possible header modification
        Bytes frame_data = data;

        std::vector<Bytes> encoded_cws;

        if (is_data_frame && data_code_rate_ != CodeRate::R1_4) {
            // DATA frame with adaptive rate: CW0 at R1/4, CW1+ at data_code_rate_
            size_t bytes_per_cw_r14 = v2::getBytesPerCodeword(CodeRate::R1_4);  // 20
            size_t bytes_per_cw_data = v2::getBytesPerCodeword(data_code_rate_);
            size_t cw1_payload_size = bytes_per_cw_data - v2::DATA_CW_HEADER_SIZE;

            // Calculate correct total_cw for this rate and update the serialized frame
            // The frame was created with R1/4 calculation, but we need rate-aware count
            // Header layout: [magic 2B][type 1B][flags 1B][seq 2B][src 3B][dst 3B][total_cw 1B][len 2B][hcrc 2B]
            //                  0-1       2        3         4-5      6-8      9-11     12        13-14   15-16
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
                encoded_cws.push_back(encoder_r14.encode(cw0));
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
        // Use adaptive code rate based on connection state
        CodeRate tx_code_rate = connected_ ? data_code_rate_ : CodeRate::R1_4;

        encoder_->setRate(tx_code_rate);
        Bytes encoded = encoder_->encode(data);

        LOG_MODEM(INFO, "TX raw: %zu bytes -> %zu encoded (rate=%d)",
                  data.size(), encoded.size(), static_cast<int>(tx_code_rate));

        to_modulate = interleaving_enabled_ ? interleaver_.interleave(encoded) : encoded;
    }

    // Modulation selection:
    // - Control frames (CONNECT, ACK, etc.): Always DQPSK for reliability
    // - DATA frames when connected: Use negotiated modulation for throughput
    // - Legacy frames: Use negotiated modulation when connected
    Modulation tx_modulation = Modulation::DQPSK;
    if ((is_v2_frame && is_data_frame && connected_) || (!is_v2_frame && connected_)) {
        tx_modulation = data_modulation_;
        LOG_MODEM(INFO, "[%s] TX: Using negotiated modulation %s for data",
                  log_prefix_.c_str(), modulationToString(tx_modulation));
    }

    // Determine which waveform to use for modulation:
    // - use_connected_waveform_once_: use waveform_mode_ (for DISCONNECT ACK)
    // - When NOT connected: use connect_waveform_ (DPSK -> MFSK fallback)
    // - When connected but handshake not complete: use last_rx_waveform_ (respond on same waveform)
    // - When connected and handshake complete: use negotiated waveform_mode_
    LOG_MODEM(INFO, "[%s] TX WAVEFORM DECISION: connected_=%d, handshake_complete_=%d, "
              "waveform_mode_=%d, connect_waveform_=%d, last_rx_waveform_=%d, use_once_=%d",
              log_prefix_.c_str(), connected_ ? 1 : 0, handshake_complete_ ? 1 : 0,
              static_cast<int>(waveform_mode_), static_cast<int>(connect_waveform_),
              static_cast<int>(last_rx_waveform_), use_connected_waveform_once_ ? 1 : 0);

    protocol::WaveformMode active_waveform;
    if (use_connected_waveform_once_) {
        // DISCONNECT ACK - use negotiated waveform even though we just disconnected
        active_waveform = waveform_mode_;
        use_connected_waveform_once_ = false;  // Clear flag after use
        LOG_MODEM(INFO, "[%s] TX: use_connected_waveform_once_ -> using waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    } else if (!connected_) {
        active_waveform = connect_waveform_;
        LOG_MODEM(INFO, "[%s] TX: NOT connected -> using connect_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    } else if (!handshake_complete_) {
        // Still in handshake - respond using same waveform we received on
        active_waveform = last_rx_waveform_;
        LOG_MODEM(INFO, "[%s] TX: Handshake mode -> using last_rx_waveform_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    } else {
        active_waveform = waveform_mode_;
        LOG_MODEM(INFO, "[%s] TX: Connected+handshake -> using waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(active_waveform));
    }

    bool use_ofdm = (active_waveform == protocol::WaveformMode::OFDM);
    bool use_dpsk = (active_waveform == protocol::WaveformMode::DPSK);
    bool use_mfsk = (active_waveform == protocol::WaveformMode::MFSK);

    Samples preamble, modulated;

    if (use_dpsk) {
        // DPSK modulation path
        LOG_MODEM(INFO, "[%s] TX: Using DPSK modulation (%d-PSK, %d samples/sym)",
                  log_prefix_.c_str(), dpsk_config_.num_phases(), dpsk_config_.samples_per_symbol);
        preamble = dpsk_modulator_->generatePreamble();
        modulated = dpsk_modulator_->modulate(to_modulate);
    } else if (use_mfsk) {
        // MFSK modulation path
        LOG_MODEM(INFO, "[%s] TX: Using MFSK modulation (%d tones)",
                  log_prefix_.c_str(), mfsk_config_.num_tones);
        preamble = mfsk_modulator_->generatePreamble();
        modulated = mfsk_modulator_->modulate(to_modulate);
    } else {
        // OFDM modulation path (default, also for control frames)
        LOG_MODEM(INFO, "[%s] TX: Using OFDM modulation (%s)",
                  log_prefix_.c_str(), is_v2_frame ? "control frame" : "data frame");
        preamble = ofdm_modulator_->generatePreamble();
        modulated = ofdm_modulator_->modulate(to_modulate, tx_modulation);
    }

    // Step 4: Combine lead-in + preamble + data + tail guard
    // Lead-in: silence for radio AGC settling and VOX keying (150ms)
    // Tail guard: ensures demodulator processes the last OFDM symbol
    const size_t LEAD_IN_SAMPLES = 48000 * 150 / 1000;  // 150ms at 48kHz = 7200 samples
    const size_t TAIL_SAMPLES = 576 * 2;  // 2 OFDM symbols of silence (~24ms)
    std::vector<float> output;
    output.reserve(LEAD_IN_SAMPLES + preamble.size() + modulated.size() + TAIL_SAMPLES);

    // Add lead-in silence (for AGC/VOX)
    output.resize(LEAD_IN_SAMPLES, 0.0f);

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

    // Apply RX bandpass filter to new samples
    if (!new_samples.empty() && filter_config_.enabled && rx_filter_) {
        SampleSpan span(new_samples.data(), new_samples.size());
        new_samples = rx_filter_->process(span);
    }

    // Update carrier sense energy detection
    if (!new_samples.empty()) {
        updateChannelEnergy(new_samples);
    }

    // Now do the slow processing with rx_mutex_
    std::lock_guard<std::mutex> lock(rx_mutex_);

    if (!new_samples.empty()) {
        LOG_MODEM(DEBUG, "pollRxAudio: got %zu new samples, buffer=%zu", new_samples.size(), rx_sample_buffer_.size());
        rx_sample_buffer_.insert(rx_sample_buffer_.end(), new_samples.begin(), new_samples.end());
    }

    // ALWAYS call processRxBuffer() - even with no new samples, the demodulator
    // may still have soft bits to deliver (e.g., in all-at-once processing mode)
    processRxBuffer();
    return !new_samples.empty();
}

void ModemEngine::processRxBuffer() {
    // Dispatch to appropriate waveform-specific processing
    // When disconnected: use multi-waveform detection (DPSK + MFSK)
    // When connected: use negotiated waveform

    if (!connected_) {
        // Disconnected - use multi-waveform detection for connection attempts
        processRxBuffer_MultiDetect();
        return;
    }

    // Connected - use negotiated waveform
    if (waveform_mode_ == protocol::WaveformMode::DPSK) {
        processRxBuffer_DPSK();
    } else if (waveform_mode_ == protocol::WaveformMode::MFSK) {
        processRxBuffer_MFSK();
    } else {
        // OFDM mode - process directly (buffer was cleared on connect, no transition check needed)
        if (rx_sample_buffer_.size() > 1000) {
            static int ofdm_call_count = 0;
            if (++ofdm_call_count % 20 == 1) {
                LOG_MODEM(INFO, "[%s] RX OFDM dispatch: %zu samples", log_prefix_.c_str(), rx_sample_buffer_.size());
            }
        }
        processRxBuffer_OFDM();
    }
}

void ModemEngine::processRxBuffer_MultiDetect() {
    // Multi-waveform detection for incoming calls when disconnected
    // Try to detect preambles from both DPSK and MFSK
    // Whichever is found first, use that demodulator

    // If we have a pending frame, skip detection and process directly
    if (pending_frame_.active) {
        LOG_MODEM(DEBUG, "[%s] RX Multi: Using pending frame (waveform=%d, preamble=%d)",
                  log_prefix_.c_str(), static_cast<int>(pending_frame_.waveform), pending_frame_.data_start);
        if (pending_frame_.waveform == protocol::WaveformMode::DPSK) {
            processRxBuffer_DPSK(-1);  // Will use saved state
        } else if (pending_frame_.waveform == protocol::WaveformMode::MFSK) {
            processRxBuffer_MFSK(-1);  // Will use saved state
        }
        return;
    }

    // Need enough samples for preamble detection
    int dpsk_preamble_samples = 32 * dpsk_config_.samples_per_symbol;
    int mfsk_preamble_samples = 2 * mfsk_config_.num_tones * mfsk_config_.samples_per_symbol;
    int min_samples = std::max(dpsk_preamble_samples, mfsk_preamble_samples) + 1000;

    if (rx_sample_buffer_.size() < (size_t)min_samples) {
        LOG_MODEM(TRACE, "RX Multi: Buffering %zu samples (need %d)",
                  rx_sample_buffer_.size(), min_samples);
        return;
    }

    // Rate limiting: only run expensive preamble detection when enough new samples arrived
    // This prevents CPU-bound loops when processing large MFSK frames
    size_t new_samples = rx_sample_buffer_.size() - last_multidetect_buffer_size_;
    if (last_multidetect_buffer_size_ > 0 && new_samples < MULTIDETECT_MIN_NEW_SAMPLES) {
        return;  // Not enough new samples to justify re-scanning
    }
    last_multidetect_buffer_size_ = rx_sample_buffer_.size();

    SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());

    // Try DPSK preamble detection first (it's the default for connection attempts)
    int dpsk_start = dpsk_demodulator_->findPreamble(span);
    if (dpsk_start >= 0) {
        // Remember we received via DPSK - responses should use same waveform
        last_rx_waveform_ = protocol::WaveformMode::DPSK;
        last_multidetect_buffer_size_ = 0;  // Reset rate limit after detection
        // DPSK will handle pending state tracking and logging internally
        processRxBuffer_DPSK(dpsk_start);
        return;
    }

    // Try MFSK preamble detection (fallback mode)
    int mfsk_start = mfsk_demodulator_->findPreamble(span);
    if (mfsk_start >= 0) {
        LOG_MODEM(INFO, "RX Multi: MFSK preamble at sample %d", mfsk_start);
        // Remember we received via MFSK - responses should use same waveform
        last_rx_waveform_ = protocol::WaveformMode::MFSK;
        last_multidetect_buffer_size_ = 0;  // Reset rate limit after detection
        // Use MFSK demodulator for this frame, passing pre-detected position
        processRxBuffer_MFSK(mfsk_start);
        return;
    }

    // No preamble found - discard old samples to prevent buffer growth
    size_t to_keep = std::min(rx_sample_buffer_.size(), (size_t)(min_samples * 2));
    if (rx_sample_buffer_.size() > to_keep) {
        rx_sample_buffer_.erase(rx_sample_buffer_.begin(),
                                rx_sample_buffer_.end() - to_keep);
        last_multidetect_buffer_size_ = rx_sample_buffer_.size();  // Update after trim
    }
}

void ModemEngine::processRxBuffer_OFDM() {
    namespace v2 = protocol::v2;
    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;  // 648 bits

    // Log demod state for debugging
    static int ofdm_rx_call = 0;
    if (++ofdm_rx_call % 20 == 1) {  // Log every 20 calls
        LOG_MODEM(INFO, "[%s] RX OFDM: data_modulation_=%d, connected=%d, waveform_mode_=%d",
                  log_prefix_.c_str(), static_cast<int>(data_modulation_),
                  connected_ ? 1 : 0, static_cast<int>(waveform_mode_));
    }

    // Need minimum samples for valid sync detection + some data
    // Too little: false sync on noise. Too much: deadlock waiting.
    // Preamble is ~4000 samples, need margin for detection window.
    // 8000 samples = preamble + small margin, avoids deadlock.
    constexpr size_t MIN_SAMPLES_FOR_SYNC = 8000;

    if (!ofdm_demodulator_->isSynced() && !ofdm_demodulator_->hasPendingData()) {
        if (rx_sample_buffer_.size() < MIN_SAMPLES_FOR_SYNC) {
            return;  // Need enough for sync detection
        }
    }

    // Track sync state to detect when demodulator gives up on a frame
    bool was_synced = ofdm_demodulator_->isSynced();

    // Feed samples to demodulator (it maintains its own buffer)
    SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());

    // Process and accumulate soft bits
    bool frame_ready = ofdm_demodulator_->process(span);

    // Clear buffer AFTER processing (demodulator copies what it needs)
    rx_sample_buffer_.clear();

    // Check if demodulator lost sync (went from synced to searching)
    // If we had accumulated ANY incomplete data, we need to discard it
    bool is_synced = ofdm_demodulator_->isSynced();
    if (was_synced && !is_synced) {
        // Lost sync - discard any accumulated bits (even if we haven't decoded CW0 yet)
        if (!ofdm_accumulated_soft_bits_.empty() || ofdm_expected_codewords_ > 0) {
            LOG_MODEM(INFO, "RX OFDM: Demodulator lost sync mid-frame, discarding %zu accumulated bits (expected %d CWs)",
                      ofdm_accumulated_soft_bits_.size(), ofdm_expected_codewords_);
            ofdm_accumulated_soft_bits_.clear();
            ofdm_expected_codewords_ = 0;
        }
    }

    // Update SNR when synced
    if (is_synced) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.snr_db = ofdm_demodulator_->getEstimatedSNR();
        stats_.synced = true;
    }

    // If frame_ready, get all accumulated soft bits (don't loop with empty_span - that triggers reset!)
    // The demodulator accumulates bits internally; getSoftBits() returns ALL of them
    LOG_MODEM(INFO, "[%s] RX OFDM: frame_ready=%d, is_synced=%d",
              log_prefix_.c_str(), frame_ready ? 1 : 0, is_synced ? 1 : 0);
    if (frame_ready) {
        auto soft_bits = ofdm_demodulator_->getSoftBits();
        LOG_MODEM(INFO, "[%s] RX OFDM: Got %zu soft bits from demod",
                  log_prefix_.c_str(), soft_bits.size());
        if (!soft_bits.empty()) {
            // Deinterleave per-codeword (648 bits each) to match TX which interleaves per-codeword
            // If we deinterleave all bits at once, bits beyond 648 would be corrupted (remain as zeros)
            if (interleaving_enabled_) {
                for (size_t i = 0; i + v2::LDPC_CODEWORD_BITS <= soft_bits.size(); i += v2::LDPC_CODEWORD_BITS) {
                    std::vector<float> cw_bits(soft_bits.begin() + i, soft_bits.begin() + i + v2::LDPC_CODEWORD_BITS);
                    auto deinterleaved = interleaver_.deinterleave(cw_bits);
                    ofdm_accumulated_soft_bits_.insert(ofdm_accumulated_soft_bits_.end(),
                                                       deinterleaved.begin(), deinterleaved.end());
                }
                // Handle any remaining bits (partial codeword) - don't deinterleave, just accumulate
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

    // Check if we have at least one codeword
    size_t num_codewords = ofdm_accumulated_soft_bits_.size() / LDPC_BLOCK;
    if (num_codewords == 0) {
        return;  // Wait for more data
    }

    // If we don't know expected count yet, decode CW0 to find out
    if (ofdm_expected_codewords_ == 0 && num_codewords >= 1) {
        std::vector<float> cw_bits(ofdm_accumulated_soft_bits_.begin(),
                                    ofdm_accumulated_soft_bits_.begin() + LDPC_BLOCK);
        LDPCDecoder cw0_decoder(CodeRate::R1_4);
        Bytes decoded = cw0_decoder.decodeSoft(cw_bits);
        bool ldpc_ok = cw0_decoder.lastDecodeSuccess();
        LOG_MODEM(INFO, "[%s] RX OFDM: CW0 probe: LDPC %s, decoded %zu bytes",
                  log_prefix_.c_str(), ldpc_ok ? "OK" : "FAILED", decoded.size());
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

    // Wait until we have all expected codewords
    if (ofdm_expected_codewords_ > 0 && num_codewords < (size_t)ofdm_expected_codewords_) {
        LOG_MODEM(DEBUG, "RX OFDM: Have %zu/%d codewords, waiting", num_codewords, ofdm_expected_codewords_);
        return;  // Wait for more
    }

    // If CW0 decode failed (ofdm_expected_codewords_ == 0), only process when
    // demodulator loses sync (frame is definitely over). This prevents processing
    // partial multi-CW frames when CW0 is corrupted. No arbitrary CW limit -
    // the demod losing sync is the reliable signal that the frame ended.
    if (ofdm_expected_codewords_ == 0 && is_synced) {
        LOG_MODEM(DEBUG, "RX OFDM: CW0 decode failed, demod still synced, waiting (have %zu CWs)", num_codewords);
        return;  // Wait for demod to lose sync (frame end)
    }

    // We have all codewords - now process the complete frame
    // Move accumulated bits to local variable and reset accumulators
    std::vector<float> accumulated_soft_bits = std::move(ofdm_accumulated_soft_bits_);
    ofdm_accumulated_soft_bits_.clear();
    int expected_total = ofdm_expected_codewords_;
    ofdm_expected_codewords_ = 0;

    num_codewords = accumulated_soft_bits.size() / LDPC_BLOCK;
    if (expected_total > 0 && (size_t)expected_total < num_codewords) {
        num_codewords = expected_total;
    }

    LOG_MODEM(INFO, "RX OFDM: Processing complete frame, %zu codewords", num_codewords);

    // Decode each codeword individually using CodewordStatus for tracking
    v2::CodewordStatus cw_status;
    int cw_success = 0, cw_failed = 0;
    std::string frame_type_str;

    // First pass: decode CW0 to get frame type (we already know expected_total from accumulation phase)
    // CW0 always uses R1/4 to allow frame type detection before knowing the rate
    v2::FrameType detected_frame_type = v2::FrameType::PROBE;
    {
        std::vector<float> cw_bits(accumulated_soft_bits.begin(),
                                    accumulated_soft_bits.begin() + LDPC_BLOCK);
        LDPCDecoder cw0_decoder(CodeRate::R1_4);  // CW0 always R1/4
        Bytes decoded = cw0_decoder.decodeSoft(cw_bits);
        bool success = cw0_decoder.lastDecodeSuccess();

        if (success && decoded.size() >= v2::BYTES_PER_CODEWORD) {
            Bytes cw_data(decoded.begin(), decoded.begin() + v2::BYTES_PER_CODEWORD);
            auto cw_info = v2::identifyCodeword(cw_data);

            if (cw_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw_data);
                if (header.valid) {
                    detected_frame_type = header.type;  // Store for adaptive rate selection
                }
            }
        }
    }

    // Notify start of frame
    if (status_callback_) {
        status_callback_("[FRAME] Received " + std::to_string(num_codewords) + " codewords");
    }

    // Determine if we should use adaptive rate for CW1+
    // DATA frames (0x30-0x33) use adaptive rate when connected
    bool use_adaptive_for_data_cw = connected_ && v2::isDataFrame(detected_frame_type);
    CodeRate cw1_rate = use_adaptive_for_data_cw ? data_code_rate_ : CodeRate::R1_4;
    size_t cw0_bytes = v2::BYTES_PER_CODEWORD;  // Always 20 bytes for R1/4
    size_t cw1_bytes = v2::getBytesPerCodeword(cw1_rate);

    LOG_MODEM(INFO, "RX OFDM: Decoding %zu CWs, frame_type=%d, CW1+ rate=%s (%zu bytes/CW)",
              num_codewords, static_cast<int>(detected_frame_type),
              use_adaptive_for_data_cw ? "adaptive" : "R1/4", cw1_bytes);

    cw_status.decoded.resize(num_codewords, false);
    cw_status.data.resize(num_codewords);

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(accumulated_soft_bits.begin() + i * LDPC_BLOCK,
                                    accumulated_soft_bits.begin() + (i + 1) * LDPC_BLOCK);

        // Select decoder based on codeword position
        // CW0: Always R1/4 (header must be decodable for frame type detection)
        // CW1+: Adaptive rate for DATA frames, R1/4 otherwise
        CodeRate cw_rate = (i == 0) ? CodeRate::R1_4 : cw1_rate;
        size_t expected_bytes = (i == 0) ? cw0_bytes : cw1_bytes;

        LOG_MODEM(INFO, "RX OFDM: Decoding CW%zu with rate %s, expected %zu bytes",
                  i, codeRateToString(cw_rate), expected_bytes);

        // Diagnostic: log soft bit statistics for debugging
        if (i > 0) {  // Only for CW1+
            float sum_abs = 0.0f;
            int pos_count = 0;
            for (float llr : cw_bits) {
                sum_abs += std::abs(llr);
                if (llr > 0) pos_count++;
            }
            float avg_llr = sum_abs / cw_bits.size();

            // Decode first 16 bytes to hex for comparison
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
            // Trim to codeword size and store
            Bytes cw_data(decoded.begin(), decoded.begin() + expected_bytes);
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
    LOG_MODEM(INFO, "RX OFDM: CW status - success=%zu, failed=%zu, allSuccess=%d",
              cw_success, cw_failed, cw_status.allSuccess() ? 1 : 0);

    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();
        LOG_MODEM(INFO, "RX OFDM: Reassembled %zu bytes", frame_data.size());

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
            // Update turnaround timestamp - we just finished receiving
            last_rx_complete_time_ = std::chrono::steady_clock::now();
        }
    } else {
        // No frame decoded - still update stats for failed codewords
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (cw_failed > 0) stats_.frames_failed++;
    }

    // Reset demodulator to look for next preamble
    // This ensures clean state between frames and prevents accumulated timing/phase errors
    LOG_MODEM(INFO, "[%s] RX OFDM: Frame processing complete, calling demod reset()",
              log_prefix_.c_str());
    ofdm_demodulator_->reset();
}

void ModemEngine::processRxBuffer_DPSK(int pre_detected_start) {
    namespace v2 = protocol::v2;

    // Need enough samples for preamble + at least 1 codeword
    int preamble_samples = 32 * dpsk_config_.samples_per_symbol;  // 32-symbol preamble
    int symbol_samples = dpsk_config_.samples_per_symbol;
    int bits_per_sym = dpsk_config_.bits_per_symbol();
    int samples_per_codeword = (v2::LDPC_CODEWORD_BITS / bits_per_sym) * symbol_samples;
    int min_samples = preamble_samples + samples_per_codeword;

    // === STATE TRACKING: If we're waiting for more samples for a known frame, skip detection ===
    if (pending_frame_.active) {
        // Check if buffer grew (new samples arrived)
        if (rx_sample_buffer_.size() <= pending_frame_.buffer_size) {
            return;  // No new samples, skip processing
        }

        int available = rx_sample_buffer_.size() - pending_frame_.data_start;
        int needed = pending_frame_.expected_codewords * samples_per_codeword;

        if (available < needed) {
            // Still waiting for more samples
            pending_frame_.buffer_size = rx_sample_buffer_.size();
            return;
        }

        // We have enough samples now - continue with decoding
        LOG_MODEM(INFO, "RX DPSK: Resuming pending frame, now have %d/%d samples",
                  available, needed);
    }

    if (rx_sample_buffer_.size() < (size_t)min_samples) {
        return;  // Not enough for even preamble detection
    }

    SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());

    int data_start;
    int expected_codewords;

    v2::FrameType detected_frame_type = v2::FrameType::PROBE;  // For adaptive rate selection

    if (pending_frame_.active) {
        // Use saved state
        data_start = pending_frame_.data_start;
        expected_codewords = pending_frame_.expected_codewords;
        detected_frame_type = pending_frame_.frame_type;
        pending_frame_.active = false;  // Clear pending flag
    } else {
        // Fresh detection
        data_start = pre_detected_start;
        if (data_start < 0) {
            data_start = dpsk_demodulator_->findPreamble(span);
        }

        if (data_start < 0) {
            // No preamble found - discard old samples to prevent buffer growth
            size_t to_keep = std::min(rx_sample_buffer_.size(), (size_t)(preamble_samples * 2));
            if (rx_sample_buffer_.size() > to_keep) {
                rx_sample_buffer_.erase(rx_sample_buffer_.begin(),
                                        rx_sample_buffer_.end() - to_keep);
            }
            return;
        }

        int available_data_samples = rx_sample_buffer_.size() - data_start;

        // Check if we have at least 1 codeword worth of samples for CW0 (header)
        if (available_data_samples < samples_per_codeword) {
            // Not enough for CW0 - don't set pending, just wait for more samples
            // We'll re-detect next poll (preamble is still in buffer)
            return;
        }

        LOG_MODEM(INFO, "[%s] RX DPSK: Data start at sample %d, available=%d samples",
                  log_prefix_.c_str(), data_start, available_data_samples);

        // Demodulate CW0 to determine frame size
        SampleSpan cw0_span(rx_sample_buffer_.data() + data_start, samples_per_codeword);
        auto cw0_soft = dpsk_demodulator_->demodulateSoft(cw0_span);

        if (cw0_soft.size() < v2::LDPC_CODEWORD_BITS) {
            LOG_MODEM(WARN, "RX DPSK: CW0 demod failed, got %zu bits (need %zu)",
                      cw0_soft.size(), (size_t)v2::LDPC_CODEWORD_BITS);
            pending_frame_.clear();  // Clear invalid pending state
            rx_sample_buffer_.clear();
            dpsk_demodulator_->reset();
            return;
        }

        // Deinterleave CW0
        std::vector<float> cw0_bits(cw0_soft.begin(), cw0_soft.begin() + v2::LDPC_CODEWORD_BITS);
        if (interleaving_enabled_) {
            cw0_bits = interleaver_.deinterleave(cw0_bits);
        }

        // Decode CW0 - ALWAYS use R1/4 for header (allows frame type detection)
        LDPCDecoder cw0_decoder(CodeRate::R1_4);
        Bytes cw0_decoded = cw0_decoder.decodeSoft(cw0_bits);
        if (!cw0_decoder.lastDecodeSuccess() || cw0_decoded.size() < v2::BYTES_PER_CODEWORD) {
            LOG_MODEM(INFO, "RX DPSK: CW0 LDPC decode FAILED");
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.frames_failed++;
            pending_frame_.clear();  // Clear invalid pending state
            rx_sample_buffer_.clear();
            dpsk_demodulator_->reset();
            return;
        }

        LOG_MODEM(INFO, "RX DPSK: CW0 LDPC decode SUCCESS, first 8: %02X %02X %02X %02X %02X %02X %02X %02X",
                  cw0_decoded[0], cw0_decoded[1], cw0_decoded[2], cw0_decoded[3],
                  cw0_decoded[4], cw0_decoded[5], cw0_decoded[6], cw0_decoded[7]);

        // Parse CW0 to get frame info
        Bytes cw0_data(cw0_decoded.begin(), cw0_decoded.begin() + v2::BYTES_PER_CODEWORD);
        auto cw0_info = v2::identifyCodeword(cw0_data);

        expected_codewords = 1;  // Default if not a header
        if (cw0_info.type == v2::CodewordType::HEADER) {
            auto header = v2::parseHeader(cw0_data);
            if (header.valid) {
                expected_codewords = header.total_cw;
                detected_frame_type = header.type;  // Update outer variable
                LOG_MODEM(INFO, "RX DPSK: Header valid, expecting %d codewords, frame_type=%d",
                          expected_codewords, static_cast<int>(header.type));
            }
        }

        // Check if we have all codewords
        int total_samples_needed = expected_codewords * samples_per_codeword;
        if (available_data_samples < total_samples_needed) {
            // Save state and wait for more samples - include frame_type for adaptive rate
            pending_frame_.set(protocol::WaveformMode::DPSK, data_start, expected_codewords,
                              rx_sample_buffer_.size(), detected_frame_type);
            LOG_MODEM(INFO, "RX DPSK: Need %d samples for %d CWs, have %d - waiting",
                      total_samples_needed, expected_codewords, available_data_samples);
            dpsk_demodulator_->reset();
            return;
        }
    }

    // === We have all samples needed - decode the full frame ===
    int total_samples_needed = expected_codewords * samples_per_codeword;

    // === Now demodulate and decode ALL codewords ===
    // Re-run findPreamble to reset phase reference (prev_symbol_) before full demodulation.
    // This ensures differential decoding starts with the correct reference from the preamble.
    dpsk_demodulator_->reset();
    dpsk_demodulator_->findPreamble(span);

    SampleSpan full_data_span(rx_sample_buffer_.data() + data_start, total_samples_needed);
    auto soft_bits = dpsk_demodulator_->demodulateSoft(full_data_span);

    LOG_MODEM(INFO, "RX DPSK: Got %zu soft bits for %d codewords",
              soft_bits.size(), expected_codewords);

    // Apply deinterleaving per-codeword
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

    // Decode each codeword - CW0 always R1/4, CW1+ use adaptive rate for DATA frames
    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    size_t num_codewords = std::min((size_t)expected_codewords, soft_bits.size() / LDPC_BLOCK);

    // Determine if we should use adaptive rate for CW1+
    // DATA frames (0x30-0x33) use adaptive rate when connected
    bool use_adaptive_for_data_cw = connected_ && v2::isDataFrame(detected_frame_type);
    CodeRate cw1_rate = use_adaptive_for_data_cw ? data_code_rate_ : CodeRate::R1_4;
    size_t cw0_bytes = v2::BYTES_PER_CODEWORD;  // Always 20 bytes for R1/4
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

        // Select decoder based on codeword position
        // CW0: Always R1/4 (header must be decodable for frame type detection)
        // CW1+: Adaptive rate for DATA frames, R1/4 otherwise
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

    // Reassemble and deliver frame if all codewords decoded
    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();

        if (!frame_data.empty()) {
            LOG_MODEM(INFO, "[%s] RX DPSK: Frame reassembled, %zu bytes", log_prefix_.c_str(), frame_data.size());

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.frames_received++;
                stats_.synced = true;
                rx_data_queue_.push(frame_data);
            }

            if (raw_data_callback_) {
                raw_data_callback_(frame_data);
            }
            // Update turnaround timestamp - we just finished receiving
            last_rx_complete_time_ = std::chrono::steady_clock::now();
        }
    } else {
        LOG_MODEM(WARN, "RX DPSK: Frame incomplete - %d/%d CWs decoded",
                  cw_success, expected_codewords);
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_failed++;
    }

    // Remove only the consumed samples (preamble might overlap with previous data)
    // Include some margin for the preamble that led to data_start
    int preamble_margin = 16 * dpsk_config_.samples_per_symbol;  // Half of 32-symbol preamble
    int consumed = std::max(0, data_start - preamble_margin) + total_samples_needed;
    consumed = std::min(consumed, (int)rx_sample_buffer_.size());

    if (consumed > 0) {
        rx_sample_buffer_.erase(rx_sample_buffer_.begin(),
                                rx_sample_buffer_.begin() + consumed);
        LOG_MODEM(DEBUG, "RX DPSK: Consumed %d samples, %zu remaining",
                  consumed, rx_sample_buffer_.size());
    }
    dpsk_demodulator_->reset();
}

void ModemEngine::processRxBuffer_MFSK(int pre_detected_start) {
    namespace v2 = protocol::v2;

    // Need enough samples for preamble detection
    int preamble_samples = 2 * mfsk_config_.num_tones * mfsk_config_.samples_per_symbol;  // 2-cycle preamble
    int symbol_samples = mfsk_config_.samples_per_symbol * mfsk_config_.repetition;
    int bits_per_symbol = mfsk_config_.bits_per_symbol();

    // Check for pending frame state from previous call
    int preamble_start = -1;
    int expected_codewords = 0;
    v2::FrameType detected_frame_type = v2::FrameType::PROBE;

    if (pending_frame_.active && pending_frame_.waveform == protocol::WaveformMode::MFSK) {
        // Resuming pending frame - use saved state
        preamble_start = pending_frame_.data_start;  // Actually preamble start for MFSK
        expected_codewords = pending_frame_.expected_codewords;
        detected_frame_type = pending_frame_.frame_type;
        LOG_MODEM(DEBUG, "[%s] RX MFSK: Resuming pending frame (preamble=%d, exp_cw=%d), have %zu samples",
                  log_prefix_.c_str(), preamble_start, expected_codewords, rx_sample_buffer_.size());
    } else if (pre_detected_start >= 0) {
        // Called from multi-detect with pre-detected preamble position
        preamble_start = pre_detected_start;
    } else {
        // Need enough samples for preamble detection
        if (rx_sample_buffer_.size() < (size_t)(preamble_samples + symbol_samples * 10)) {
            LOG_MODEM(TRACE, "RX MFSK: Buffering %zu samples (need %d)",
                      rx_sample_buffer_.size(), preamble_samples + symbol_samples * 10);
            return;
        }

        LOG_MODEM(DEBUG, "RX MFSK: Processing %zu samples...", rx_sample_buffer_.size());

        SampleSpan span(rx_sample_buffer_.data(), rx_sample_buffer_.size());
        preamble_start = mfsk_demodulator_->findPreamble(span);

        if (preamble_start < 0) {
            // No preamble found - discard old samples to prevent buffer growth
            size_t to_keep = std::min(rx_sample_buffer_.size(), (size_t)(preamble_samples * 2));
            if (rx_sample_buffer_.size() > to_keep) {
                rx_sample_buffer_.erase(rx_sample_buffer_.begin(),
                                        rx_sample_buffer_.end() - to_keep);
            }
            return;
        }
    }

    // Skip preamble to get to data
    int preamble_end = preamble_start + preamble_samples;
    if (!pending_frame_.active) {
        LOG_MODEM(INFO, "RX MFSK: Preamble found at sample %d, data starts at %d",
                  preamble_start, preamble_end);
    }

    // Calculate samples needed for at least one codeword
    // MFSK: 648 bits / bits_per_symbol = symbols needed, × repetition × samples_per_symbol
    int samples_per_cw = (v2::LDPC_CODEWORD_BITS / bits_per_symbol) * symbol_samples;
    int min_data_samples = samples_per_cw;  // At least 1 codeword

    int data_available = (int)rx_sample_buffer_.size() - preamble_end;
    if (data_available < min_data_samples) {
        if (!pending_frame_.active) {
            LOG_MODEM(INFO, "RX MFSK: Need %d data samples, have %d - waiting",
                      min_data_samples, data_available);
        }
        // Set pending state and wait for more samples
        pending_frame_.set(protocol::WaveformMode::MFSK, preamble_start, 0,
                          rx_sample_buffer_.size(), v2::FrameType::PROBE);
        return;
    }

    // Demodulate data portion to soft bits
    SampleSpan data_span(rx_sample_buffer_.data() + preamble_end, data_available);

    // MFSK demodulation to soft bits (tone powers -> LLRs)
    auto soft_bits = mfsk_demodulator_->demodulateSoft(data_span);

    if (soft_bits.empty()) {
        // This shouldn't happen if we have enough samples, but wait if it does
        LOG_MODEM(WARN, "RX MFSK: No soft bits from demodulation (unexpected)");
        pending_frame_.set(protocol::WaveformMode::MFSK, preamble_start, 0,
                          rx_sample_buffer_.size(), v2::FrameType::PROBE);
        return;
    }

    LOG_MODEM(INFO, "RX MFSK: Got %zu soft bits from %d data samples", soft_bits.size(), data_available);

    // Apply deinterleaving if enabled
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

    // Process codewords through LDPC decoder
    const size_t LDPC_BLOCK = v2::LDPC_CODEWORD_BITS;
    size_t num_codewords = soft_bits.size() / LDPC_BLOCK;

    if (num_codewords == 0) {
        // Not enough bits yet - wait for more samples
        LOG_MODEM(INFO, "RX MFSK: Got %zu bits, need %zu for 1 CW - waiting",
                  soft_bits.size(), LDPC_BLOCK);
        pending_frame_.set(protocol::WaveformMode::MFSK, preamble_start, 0,
                          rx_sample_buffer_.size(), v2::FrameType::PROBE);
        return;
    }

    LOG_MODEM(INFO, "RX MFSK: Decoding %zu codewords", num_codewords);

    // First, decode CW0 to detect frame type (always R1/4)
    // Skip if we already know from pending_frame_
    if (detected_frame_type == v2::FrameType::PROBE && expected_codewords == 0) {
        std::vector<float> cw0_bits(soft_bits.begin(), soft_bits.begin() + LDPC_BLOCK);
        LDPCDecoder cw0_decoder(CodeRate::R1_4);
        Bytes cw0_decoded = cw0_decoder.decodeSoft(cw0_bits);
        if (cw0_decoder.lastDecodeSuccess() && cw0_decoded.size() >= v2::BYTES_PER_CODEWORD) {
            Bytes cw0_data(cw0_decoded.begin(), cw0_decoded.begin() + v2::BYTES_PER_CODEWORD);
            auto cw0_info = v2::identifyCodeword(cw0_data);
            if (cw0_info.type == v2::CodewordType::HEADER) {
                auto header = v2::parseHeader(cw0_data);
                if (header.valid) {
                    detected_frame_type = header.type;
                    expected_codewords = header.total_cw;
                    LOG_MODEM(INFO, "RX MFSK: Header valid, expecting %d codewords, frame_type=%d",
                              expected_codewords, static_cast<int>(detected_frame_type));
                }
            }
        }
    }

    // If we know expected_codewords but don't have enough, wait for more samples
    if (expected_codewords > 0 && num_codewords < (size_t)expected_codewords) {
        // Calculate samples needed per codeword for MFSK
        int samples_per_cw = (v2::LDPC_CODEWORD_BITS / bits_per_symbol) * symbol_samples;
        int total_samples_needed = preamble_end + expected_codewords * samples_per_cw;

        if ((int)rx_sample_buffer_.size() < total_samples_needed) {
            // Set pending state with frame info
            pending_frame_.set(protocol::WaveformMode::MFSK, preamble_start, expected_codewords,
                              rx_sample_buffer_.size(), detected_frame_type);
            LOG_MODEM(INFO, "[%s] RX MFSK: Need %d samples for %d CWs (preamble=%d), have %zu - waiting",
                      log_prefix_.c_str(), total_samples_needed, expected_codewords,
                      preamble_start, rx_sample_buffer_.size());
            return;
        }
    }

    // Limit codewords to expected count
    if (expected_codewords > 0 && num_codewords > (size_t)expected_codewords) {
        num_codewords = expected_codewords;
    }

    // Determine if we should use adaptive rate for CW1+
    // DATA frames (0x30-0x33) use adaptive rate when connected
    // Note: MFSK is typically used for low SNR where R1/4 is best, but support adaptive for consistency
    bool use_adaptive_for_data_cw = connected_ && v2::isDataFrame(detected_frame_type);
    CodeRate cw1_rate = use_adaptive_for_data_cw ? data_code_rate_ : CodeRate::R1_4;
    size_t cw0_bytes = v2::BYTES_PER_CODEWORD;  // Always 20 bytes for R1/4
    size_t cw1_bytes = v2::getBytesPerCodeword(cw1_rate);

    LOG_MODEM(INFO, "RX MFSK: frame_type=%d, CW1+ rate=%s (%zu bytes/CW)",
              static_cast<int>(detected_frame_type),
              use_adaptive_for_data_cw ? "adaptive" : "R1/4", cw1_bytes);

    // Decode each codeword
    v2::CodewordStatus cw_status;
    cw_status.decoded.resize(num_codewords, false);
    cw_status.data.resize(num_codewords);
    int cw_success = 0, cw_failed = 0;

    for (size_t i = 0; i < num_codewords; i++) {
        std::vector<float> cw_bits(soft_bits.begin() + i * LDPC_BLOCK,
                                   soft_bits.begin() + (i + 1) * LDPC_BLOCK);

        // Select decoder based on codeword position
        // CW0: Always R1/4, CW1+: Adaptive rate for DATA frames
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
            LOG_MODEM(INFO, "RX MFSK: CW%zu LDPC decode SUCCESS (rate=%s, %zu bytes)",
                      i, (cw_rate == CodeRate::R1_4) ? "R1/4" : "adaptive", expected_bytes);
        } else {
            cw_failed++;
            LOG_MODEM(INFO, "RX MFSK: CW%zu LDPC decode FAILED (rate=%s)", i,
                      (cw_rate == CodeRate::R1_4) ? "R1/4" : "adaptive");
        }
    }

    // Reassemble and deliver frame if all codewords decoded
    if (cw_status.allSuccess()) {
        Bytes frame_data = cw_status.reassemble();

        if (!frame_data.empty()) {
            LOG_MODEM(INFO, "RX MFSK: Frame reassembled, %zu bytes", frame_data.size());

            {
                std::lock_guard<std::mutex> lock(stats_mutex_);
                stats_.frames_received++;
                stats_.synced = true;
                rx_data_queue_.push(frame_data);
            }

            if (raw_data_callback_) {
                raw_data_callback_(frame_data);
            }
            // Update turnaround timestamp - we just finished receiving
            last_rx_complete_time_ = std::chrono::steady_clock::now();
        }
    } else {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_failed++;
    }

    // Clear pending frame state and buffer after processing
    pending_frame_.clear();
    rx_sample_buffer_.clear();
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

    // Reset pending frame state and multi-detect rate limiter
    pending_frame_.clear();
    last_multidetect_buffer_size_ = 0;
    use_connected_waveform_once_ = false;

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

bool ModemEngine::isTurnaroundActive() const {
    if (turnaround_delay_ms_ == 0) return false;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_complete_time_).count();
    return elapsed < turnaround_delay_ms_;
}

uint32_t ModemEngine::getTurnaroundRemaining() const {
    if (turnaround_delay_ms_ == 0) return 0;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_rx_complete_time_).count();
    if (elapsed >= turnaround_delay_ms_) return 0;
    return static_cast<uint32_t>(turnaround_delay_ms_ - elapsed);
}

// === Waveform Mode Control ===

// Helper to get mode description string
static const char* getModeDescription(Modulation mod, CodeRate rate) {
    // Return a static description based on modulation and code rate
    static char buf[32];
    const char* mod_name;
    switch (mod) {
        case Modulation::DBPSK: mod_name = "DBPSK"; break;
        case Modulation::BPSK:  mod_name = "BPSK"; break;
        case Modulation::DQPSK: mod_name = "DQPSK"; break;
        case Modulation::QPSK:  mod_name = "QPSK"; break;
        case Modulation::D8PSK: mod_name = "D8PSK"; break;
        case Modulation::QAM8:  mod_name = "8QAM"; break;
        case Modulation::QAM16: mod_name = "16QAM"; break;
        case Modulation::QAM32: mod_name = "32QAM"; break;
        case Modulation::QAM64: mod_name = "64QAM"; break;
        default: mod_name = "???"; break;
    }
    const char* rate_name;
    switch (rate) {
        case CodeRate::R1_4: rate_name = "R1/4"; break;
        case CodeRate::R1_2: rate_name = "R1/2"; break;
        case CodeRate::R2_3: rate_name = "R2/3"; break;
        case CodeRate::R3_4: rate_name = "R3/4"; break;
        case CodeRate::R5_6: rate_name = "R5/6"; break;
        default: rate_name = "R?"; break;
    }
    snprintf(buf, sizeof(buf), "%s %s", mod_name, rate_name);
    return buf;
}

void ModemEngine::setWaveformMode(protocol::WaveformMode mode) {
    if (waveform_mode_ == mode) return;

    LOG_MODEM(INFO, "Switching waveform mode: %d -> %d",
              static_cast<int>(waveform_mode_), static_cast<int>(mode));

    waveform_mode_ = mode;

    // Reset the appropriate demodulator for the new mode
    switch (mode) {
        case protocol::WaveformMode::DPSK:
            dpsk_demodulator_->reset();
            // Clear RX buffer when switching modes (no lock - may be called from RX path)
            rx_sample_buffer_.clear();
            pending_frame_.clear();
            LOG_MODEM(INFO, "DPSK mode active: %d-PSK, %d samples/sym, %.1f bps (buffer cleared)",
                      dpsk_config_.num_phases(),
                      dpsk_config_.samples_per_symbol,
                      dpsk_config_.raw_bps());
            break;

        case protocol::WaveformMode::MFSK:
            mfsk_demodulator_->reset();
            // Clear RX buffer when switching modes (no lock - may be called from RX path)
            rx_sample_buffer_.clear();
            pending_frame_.clear();
            LOG_MODEM(INFO, "MFSK mode active: %d tones, %.1f effective bps (buffer cleared)",
                      mfsk_config_.num_tones,
                      mfsk_config_.effective_bps());
            break;

        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW:
            otfs_demodulator_->reset();
            LOG_MODEM(INFO, "OTFS mode active: M=%d, N=%d",
                      otfs_config_.M, otfs_config_.N);
            break;

        case protocol::WaveformMode::OFDM:
        default:
            ofdm_demodulator_->reset();
            // Clear RX buffer when switching to OFDM (no lock - may be called from RX path)
            rx_sample_buffer_.clear();
            pending_frame_.clear();
            LOG_MODEM(INFO, "OFDM mode active: %d carriers, %s (buffer cleared)",
                      config_.num_carriers,
                      connected_ ? "connected" : "disconnected");
            break;
    }
}

void ModemEngine::setConnectWaveform(protocol::WaveformMode mode) {
    if (connect_waveform_ == mode) return;

    LOG_MODEM(INFO, "Switching connect waveform: %s -> %s",
              protocol::waveformModeToString(connect_waveform_),
              protocol::waveformModeToString(mode));

    connect_waveform_ = mode;

    // Configure DPSK for medium preset (DQPSK 62b R1/4) for connection attempts
    if (mode == protocol::WaveformMode::DPSK) {
        dpsk_config_ = dpsk_presets::medium();  // DQPSK 62.5 baud
        dpsk_modulator_ = std::make_unique<DPSKModulator>(dpsk_config_);
        dpsk_demodulator_ = std::make_unique<DPSKDemodulator>(dpsk_config_);
        LOG_MODEM(INFO, "DPSK connect mode: %d-PSK, %.1f baud",
                  dpsk_config_.num_phases(), dpsk_config_.symbol_rate());
    }
}

void ModemEngine::setConnected(bool connected) {
    LOG_MODEM(INFO, "[%s] setConnected(%d) called, was connected_=%d",
              log_prefix_.c_str(), connected ? 1 : 0, connected_ ? 1 : 0);

    if (connected_ == connected) return;

    connected_ = connected;

    if (connected) {
        // Reset handshake state - we'll complete it when we receive first post-ACK frame
        handshake_complete_ = false;
        use_connected_waveform_once_ = false;  // Clear any leftover flag

        // CRITICAL: Clear RX buffer when entering connected state
        // Old samples from DPSK/MFSK handshake would corrupt OFDM preamble detection
        // NOTE: Don't acquire rx_mutex_ here - we're called from within the RX processing
        // path which already holds the lock. Just clear directly.
        rx_sample_buffer_.clear();
        pending_frame_.clear();
        ofdm_demodulator_->reset();

        // NOTE: Don't overwrite waveform_mode_ here - it was already set to the negotiated
        // mode (e.g., OFDM) by the on_mode_negotiated_ callback before setConnected() is called.
        // During handshake, TX uses last_rx_waveform_ (set by RX path when we received CONNECT/CONNECT_ACK)

        LOG_MODEM(INFO, "Entered connected state, RX buffer cleared (negotiated=%d, TX uses last_rx=%d)",
                  static_cast<int>(waveform_mode_), static_cast<int>(last_rx_waveform_));
    } else {
        // Switching to disconnected state - use robust mode for RX
        ModemConfig rx_config = config_;
        rx_config.modulation = Modulation::DQPSK;
        rx_config.code_rate = CodeRate::R1_4;

        decoder_->setRate(CodeRate::R1_4);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(rx_config);

        // Keep using connected waveform for the next TX (DISCONNECT ACK)
        // This handles the case where the ACK is queued before setConnected(false) is called
        use_connected_waveform_once_ = true;
        LOG_MODEM(INFO, "Switched to disconnected mode (RX: DQPSK R1/4, next TX uses connected waveform)");
        handshake_complete_ = false;  // Reset for next connection
    }
}

void ModemEngine::setHandshakeComplete(bool complete) {
    if (handshake_complete_ == complete) return;

    handshake_complete_ = complete;

    if (complete) {
        LOG_MODEM(INFO, "Handshake complete, TX now uses waveform_mode_=%d",
                  static_cast<int>(waveform_mode_));
    }
}

void ModemEngine::setDataMode(Modulation mod, CodeRate rate) {
    data_modulation_ = mod;
    data_code_rate_ = rate;

    // Determine if modulation is differential (doesn't need pilots)
    bool is_differential = (mod == Modulation::DBPSK ||
                           mod == Modulation::DQPSK ||
                           mod == Modulation::D8PSK);

    // If already connected, update both TX and RX to match
    if (connected_) {
        // Update base config for TX modulator
        config_.modulation = mod;
        config_.code_rate = rate;
        config_.use_pilots = !is_differential;  // QAM needs pilots, DQPSK doesn't

        // Recreate modulator with new config
        ofdm_modulator_ = std::make_unique<OFDMModulator>(config_);

        // Recreate demodulator with matching config
        decoder_->setRate(rate);
        ofdm_demodulator_ = std::make_unique<OFDMDemodulator>(config_);

        LOG_MODEM(INFO, "TX/RX OFDM updated: mod=%d, rate=%d, use_pilots=%d",
                  static_cast<int>(mod), static_cast<int>(rate), config_.use_pilots ? 1 : 0);
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

protocol::WaveformMode ModemEngine::recommendWaveformMode(float snr_db) {
    // Waveform selection based on measured SNR
    // Thresholds based on testing (test_mode_snr tool):
    //   < 0 dB:  MFSK (works at -17 dB reported / 0 dB actual)
    //   0-17 dB: DPSK (single-carrier, OFDM sync fails below ~17 dB)
    //   > 17 dB: OFDM (highest throughput, needs reliable sync)

    if (snr_db < 0.0f) {
        return protocol::WaveformMode::MFSK;
    } else if (snr_db < 17.0f) {
        return protocol::WaveformMode::DPSK;
    } else {
        return protocol::WaveformMode::OFDM;
    }
}

void ModemEngine::setMFSKMode(int num_tones) {
    // Select appropriate MFSK preset based on number of tones
    switch (num_tones) {
        case 2:
            mfsk_config_ = mfsk_presets::robust();  // ~8 bps, most robust
            break;
        case 4:
            mfsk_config_ = mfsk_presets::low_snr();  // ~21 bps
            break;
        case 8:
            mfsk_config_ = mfsk_presets::medium();  // ~47 bps, default
            break;
        case 16:
            mfsk_config_ = mfsk_presets::fast();  // ~62 bps
            break;
        case 32:
            mfsk_config_ = mfsk_presets::turbo();  // ~156 bps, fastest MFSK
            break;
        default:
            LOG_MODEM(WARN, "Invalid MFSK tones %d, using 8", num_tones);
            mfsk_config_ = mfsk_presets::medium();
            break;
    }

    // Recreate modulator/demodulator with new config
    mfsk_modulator_ = std::make_unique<MFSKModulator>(mfsk_config_);
    mfsk_demodulator_ = std::make_unique<MFSKDemodulator>(mfsk_config_);

    LOG_MODEM(INFO, "MFSK mode set: %d tones, %.1f bps",
              mfsk_config_.num_tones, mfsk_config_.effective_bps());
}

void ModemEngine::setDPSKMode(DPSKModulation mod, int samples_per_symbol) {
    // Configure DPSK based on modulation type and symbol rate
    dpsk_config_.modulation = mod;
    dpsk_config_.samples_per_symbol = samples_per_symbol;

    // Recreate modulator/demodulator with new config
    dpsk_modulator_ = std::make_unique<DPSKModulator>(dpsk_config_);
    dpsk_demodulator_ = std::make_unique<DPSKDemodulator>(dpsk_config_);

    const char* mod_name = "DQPSK";
    switch (mod) {
        case DPSKModulation::DBPSK: mod_name = "DBPSK"; break;
        case DPSKModulation::DQPSK: mod_name = "DQPSK"; break;
        case DPSKModulation::D8PSK: mod_name = "D8PSK"; break;
    }

    LOG_MODEM(INFO, "DPSK mode set: %s, %d samples/sym (%.1f baud), %.1f bps",
              mod_name,
              dpsk_config_.samples_per_symbol,
              dpsk_config_.symbol_rate(),
              dpsk_config_.raw_bps());
}

} // namespace gui
} // namespace ultra
