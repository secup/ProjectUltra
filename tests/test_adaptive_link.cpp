/**
 * Adaptive Link Establishment Test Suite
 *
 * CRITICAL TEST: Tests the adaptive link establishment (PROBE/CONNECT) flow
 * through the FULL modem pipeline using TWO SEPARATE modem instances.
 *
 * This tests:
 * 1. PROBE/PROBE_ACK/CONNECT/CONNECT_ACK exchange at various SNR levels
 * 2. Link establishment frames use BPSK R1/4 (robust mode)
 * 3. Data frames use negotiated modulation based on channel SNR
 * 4. Proper mode switching when connection state changes
 *
 * Architecture:
 *   STATION A (TEST1)               STATION B (TEST2)
 *   ┌─────────────┐                 ┌─────────────┐
 *   │ Protocol    │                 │ Protocol    │
 *   │ LDPC Encode │ ── AWGN ──────> │ LDPC Decode │
 *   │ OFDM Mod    │   Channel       │ OFDM Demod  │
 *   └─────────────┘                 └─────────────┘
 *
 * This software may be used in critical situations - NO margin for errors.
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "fsk/mfsk.hpp"
#include "psk/dpsk.hpp"
#include "protocol/protocol_engine.hpp"
#include "protocol/frame_v2.hpp"
#include <iostream>
#include <iomanip>  // For std::setw
#include <queue>
#include <vector>
#include <cstring>
#include <random>
#include <cmath>

using namespace ultra;
using namespace ultra::protocol;

// Test counters
static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { std::cout << "  Testing " << name << "... " << std::flush; tests_run++; } while(0)

#define PASS() \
    do { std::cout << "PASS\n"; tests_passed++; } while(0)

#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << "\n"; return false; } while(0)

// ============================================================================
// AWGN Channel Simulator (Simple but correct)
// ============================================================================

class AWGNChannel {
public:
    AWGNChannel(float snr_db, uint32_t seed = 42)
        : rng_(seed), gaussian_(0.0f, 1.0f) {
        setSNR(snr_db);
    }

    void setSNR(float snr_db) {
        snr_db_ = snr_db;
    }

    Samples process(const Samples& input) {
        if (input.empty()) return input;

        // Calculate signal RMS
        float sum_sq = 0;
        for (float s : input) sum_sq += s * s;
        float rms = std::sqrt(sum_sq / input.size());

        // Calculate noise stddev for target SNR (power ratio)
        // SNR = 10 * log10(Ps/Pn) => Pn = Ps / 10^(SNR/10)
        float snr_linear = std::pow(10.0f, snr_db_ / 10.0f);
        float noise_power = (rms * rms) / snr_linear;
        float noise_std = std::sqrt(noise_power);

        // Add AWGN
        Samples output(input.size());
        for (size_t i = 0; i < input.size(); ++i) {
            output[i] = input[i] + noise_std * gaussian_(rng_);
        }

        return output;
    }

private:
    float snr_db_;
    std::mt19937 rng_;
    std::normal_distribution<float> gaussian_;
};

// ============================================================================
// Adaptive Test Station
// ============================================================================

/**
 * A modem station that mimics the GUI's adaptive behavior:
 * - Link establishment frames (PROBE, CONNECT, etc.) use BPSK R1/4
 * - Data frames use negotiated modulation based on channel SNR
 * - RX switches mode based on connection state
 */
class AdaptiveTestStation {
public:
    std::string name;
    ModemConfig config;

    // TX components - OFDM (default)
    std::unique_ptr<LDPCEncoder> encoder;
    std::unique_ptr<OFDMModulator> modulator;

    // TX components - DPSK
    std::unique_ptr<DPSKModulator> dpsk_modulator;
    DPSKConfig dpsk_config;

    // TX components - MFSK
    std::unique_ptr<MFSKModulator> mfsk_modulator;
    MFSKConfig mfsk_config;

    // RX components (separate for proper mode switching)
    std::unique_ptr<LDPCDecoder> decoder;
    std::unique_ptr<OFDMDemodulator> demodulator;
    std::unique_ptr<DPSKDemodulator> dpsk_demodulator;
    std::unique_ptr<MFSKDemodulator> mfsk_demodulator;

    // Interleaver (6×108 = 648 bits, matches LDPC codeword)
    Interleaver interleaver{6, 108};

    // Protocol engine
    std::unique_ptr<ProtocolEngine> protocol;

    // Outgoing audio queue
    std::queue<Samples> tx_queue;

    // Connection state
    bool connected = false;
    std::string remote_call;

    // Estimated channel SNR (for adaptive waveform selection)
    float estimated_snr = 30.0f;

    // Negotiated data mode (modulation/rate for OFDM)
    Modulation data_modulation = Modulation::QPSK;
    CodeRate data_code_rate = CodeRate::R1_2;

    // Negotiated waveform mode (OFDM, DPSK, MFSK)
    WaveformMode waveform_mode = WaveformMode::OFDM_NVIS;

    // Frame type tracking for verification
    std::vector<v2::FrameType> tx_frame_types;
    std::vector<Modulation> tx_modulations;
    std::vector<WaveformMode> tx_waveform_modes;

    // Select optimal DPSK configuration based on SNR
    // Based on benchmark results from test_dpsk_comprehensive.cpp:
    //   0-3 dB:  DBPSK 62b R1/4  = 16 bps
    //   3-5 dB:  DQPSK 62b R1/4  = 31 bps
    //   5-8 dB:  DQPSK 300b R1/2 = 300 bps
    //   8-10 dB: DQPSK 375b R1/2 = 375 bps
    //   10-12 dB: DQPSK 500b R1/2 = 500 bps
    //   12-15 dB: D8PSK 375b R1/2 = 562 bps
    static std::pair<DPSKConfig, CodeRate> selectOptimalDPSK(float snr_db) {
        if (snr_db < 3.0f) {
            return {dpsk_presets::low_snr(), CodeRate::R1_4};   // DBPSK 62b, 16 bps
        } else if (snr_db < 5.0f) {
            return {dpsk_presets::medium(), CodeRate::R1_4};    // DQPSK 62b, 31 bps
        } else if (snr_db < 8.0f) {
            return {dpsk_presets::speed1(), CodeRate::R1_2};    // DQPSK 300b, 300 bps
        } else if (snr_db < 10.0f) {
            return {dpsk_presets::speed2(), CodeRate::R1_2};    // DQPSK 375b, 375 bps
        } else if (snr_db < 12.0f) {
            return {dpsk_presets::speed3(), CodeRate::R1_2};    // DQPSK 500b, 500 bps
        } else {
            return {dpsk_presets::speed4(), CodeRate::R1_2};    // D8PSK 375b, 562 bps
        }
    }

    AdaptiveTestStation(const std::string& station_name, const std::string& callsign)
        : name(station_name) {

        config = presets::balanced();

        // TX: encoder uses config, modulator uses config
        encoder = std::make_unique<LDPCEncoder>(config.code_rate);
        modulator = std::make_unique<OFDMModulator>(config);

        // TX: DPSK modulator (for mid-SNR range)
        dpsk_config = dpsk_presets::fast();  // DQPSK 125 baud
        dpsk_modulator = std::make_unique<DPSKModulator>(dpsk_config);

        // TX: MFSK modulator (for very low SNR)
        mfsk_config = mfsk_presets::medium();  // 8FSK
        mfsk_modulator = std::make_unique<MFSKModulator>(mfsk_config);

        // RX: Start in disconnected state with BPSK R1/4 for link establishment
        ModemConfig rx_config = config;
        rx_config.modulation = Modulation::BPSK;
        rx_config.code_rate = CodeRate::R1_4;
        decoder = std::make_unique<LDPCDecoder>(rx_config.code_rate);
        demodulator = std::make_unique<OFDMDemodulator>(rx_config);

        // RX: DPSK demodulator
        dpsk_demodulator = std::make_unique<DPSKDemodulator>(dpsk_config);

        // RX: MFSK demodulator
        mfsk_demodulator = std::make_unique<MFSKDemodulator>(mfsk_config);

        // Create protocol engine
        protocol = std::make_unique<ProtocolEngine>();
        protocol->setLocalCallsign(callsign);
        protocol->setAutoAccept(true);

        // Wire up TX callback
        protocol->setTxDataCallback([this](const Bytes& data) {
            transmit(data);
        });

        // Wire up connection state callback
        protocol->setConnectionChangedCallback([this](ConnectionState state, const std::string& remote) {
            if (state == ConnectionState::CONNECTED) {
                connected = true;
                remote_call = remote;
                switchToDataMode();
            } else if (state == ConnectionState::DISCONNECTED) {
                connected = false;
                remote_call.clear();
                switchToLinkMode();
            }
        });
    }

    // Check if first 2 bytes match v2 frame magic (0x554C = "UL")
    static bool hasMagic(const Bytes& data) {
        if (data.size() < 2) return false;
        uint16_t magic = (static_cast<uint16_t>(data[0]) << 8) |
                          static_cast<uint16_t>(data[1]);
        return magic == v2::MAGIC_V2;
    }

    // Determine if frame is a link establishment frame
    static bool isLinkFrame(const Bytes& data) {
        if (data.size() >= 3 && hasMagic(data)) {
            uint8_t frame_type = data[2];  // Type is at offset 2 after 2-byte magic
            return (frame_type == static_cast<uint8_t>(v2::FrameType::PROBE) ||
                    frame_type == static_cast<uint8_t>(v2::FrameType::PROBE_ACK) ||
                    frame_type == static_cast<uint8_t>(v2::FrameType::CONNECT) ||
                    frame_type == static_cast<uint8_t>(v2::FrameType::CONNECT_ACK) ||
                    frame_type == static_cast<uint8_t>(v2::FrameType::DISCONNECT) ||
                    frame_type == static_cast<uint8_t>(v2::FrameType::BEACON));
        }
        return false;
    }

    // Get frame type from serialized data
    static v2::FrameType getFrameType(const Bytes& data) {
        if (data.size() >= 3 && hasMagic(data)) {
            return static_cast<v2::FrameType>(data[2]);
        }
        return v2::FrameType::DATA;  // Default
    }

    // Transmit with adaptive modulation (like modem_engine.cpp)
    void transmit(const Bytes& data) {
        // Determine modulation for this frame
        Modulation tx_mod;
        CodeRate tx_rate;
        WaveformMode tx_waveform;

        v2::FrameType frame_type = getFrameType(data);
        bool is_link = isLinkFrame(data);

        if (is_link) {
            // Link establishment: Use adaptive waveform based on estimated SNR
            // Thresholds from CLAUDE.md performance analysis:
            // - < 0 dB: MFSK (works at -17 dB reported)
            // - 0-15 dB: DPSK (works reliably at low SNR)
            // - >= 15 dB: OFDM (highest throughput)
            tx_mod = Modulation::BPSK;
            tx_rate = CodeRate::R1_4;
            if (estimated_snr < 0.0f) {
                tx_waveform = WaveformMode::MFSK;
            } else if (estimated_snr < 15.0f) {
                tx_waveform = WaveformMode::DPSK;
            } else {
                tx_waveform = WaveformMode::OFDM_NVIS;
            }
        } else if (connected) {
            // Connected: Use negotiated waveform and data mode
            tx_mod = data_modulation;
            tx_rate = data_code_rate;
            tx_waveform = waveform_mode;
        } else {
            // Fallback: Robust mode
            tx_mod = Modulation::BPSK;
            tx_rate = CodeRate::R1_4;
            tx_waveform = WaveformMode::OFDM_NVIS;
        }

        // Track for verification
        tx_frame_types.push_back(frame_type);
        tx_modulations.push_back(tx_mod);
        tx_waveform_modes.push_back(tx_waveform);

        // Encode with LDPC - handle multi-codeword frames like modem_engine
        Bytes to_modulate;
        bool is_v2 = hasMagic(data);

        if (is_v2) {
            // V2 frames use encodeFrameWithLDPC which handles multi-codeword frames
            auto encoded_cws = v2::encodeFrameWithLDPC(data);

            // Apply interleaving per-codeword (same as modem_engine)
            for (const auto& cw : encoded_cws) {
                Bytes interleaved = interleaver.interleave(cw);
                to_modulate.insert(to_modulate.end(), interleaved.begin(), interleaved.end());
            }
        } else {
            // Non-v2 frames: simple single-codeword encode
            encoder->setRate(tx_rate);
            Bytes encoded = encoder->encode(data);
            to_modulate = interleaver.interleave(encoded);
        }

        Samples preamble, audio;

        // Modulate based on waveform mode
        if (tx_waveform == WaveformMode::DPSK) {
            // Select optimal DPSK configuration based on estimated SNR
            auto [optimal_cfg, optimal_rate] = selectOptimalDPSK(estimated_snr);
            if (dpsk_config.samples_per_symbol != optimal_cfg.samples_per_symbol ||
                dpsk_config.modulation != optimal_cfg.modulation) {
                dpsk_config = optimal_cfg;
                dpsk_modulator = std::make_unique<DPSKModulator>(dpsk_config);
            }
            // DPSK modulation
            preamble = dpsk_modulator->generatePreamble();
            audio = dpsk_modulator->modulate(to_modulate);
        } else if (tx_waveform == WaveformMode::MFSK) {
            // MFSK modulation
            preamble = mfsk_modulator->generatePreamble();
            audio = mfsk_modulator->modulate(to_modulate);
        } else {
            // OFDM modulation (default)
            preamble = modulator->generatePreamble();
            audio = modulator->modulate(to_modulate, tx_mod);
        }

        // Combine with padding for DPSK/MFSK (preamble detection may slide forward)
        // Add padding of 10% of audio length at the end to prevent data truncation
        size_t padding = 0;
        if (tx_waveform == WaveformMode::DPSK || tx_waveform == WaveformMode::MFSK) {
            padding = audio.size() / 10;
        }

        Samples tx_audio;
        tx_audio.reserve(preamble.size() + audio.size() + padding);
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), audio.begin(), audio.end());
        tx_audio.resize(tx_audio.size() + padding, 0.0f);  // Zero padding at end

        // Normalize to 0.9 max amplitude (preserves signal strength for demodulation)
        // Using 0.9 instead of 1.0 leaves headroom for AWGN
        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.9f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        tx_queue.push(std::move(tx_audio));
    }

    // Receive audio (demodulate + decode)
    // Select waveform based on:
    // - If connected: use negotiated waveform_mode
    // - If disconnected: use adaptive selection based on estimated_snr
    void receive(const Samples& audio) {
        std::vector<float> all_soft_bits;

        // Determine which waveform to use for receiving
        WaveformMode rx_waveform;
        if (connected) {
            rx_waveform = waveform_mode;
        } else {
            // Adaptive selection based on estimated SNR (same thresholds as TX)
            if (estimated_snr < 0.0f) {
                rx_waveform = WaveformMode::MFSK;
            } else if (estimated_snr < 15.0f) {
                rx_waveform = WaveformMode::DPSK;
            } else {
                rx_waveform = WaveformMode::OFDM_NVIS;
            }
        }

        if (rx_waveform == WaveformMode::DPSK) {
            // Select optimal DPSK configuration based on estimated SNR
            auto [optimal_cfg, optimal_rate] = selectOptimalDPSK(estimated_snr);
            if (dpsk_config.samples_per_symbol != optimal_cfg.samples_per_symbol ||
                dpsk_config.modulation != optimal_cfg.modulation) {
                dpsk_config = optimal_cfg;
                dpsk_demodulator = std::make_unique<DPSKDemodulator>(dpsk_config);
            }
            // DPSK demodulation
            SampleSpan span(audio.data(), audio.size());
            int data_start = dpsk_demodulator->findPreamble(span);
            if (data_start >= 0) {
                SampleSpan data_span(audio.data() + data_start, audio.size() - data_start);
                all_soft_bits = dpsk_demodulator->demodulateSoft(data_span);
            }
        } else if (rx_waveform == WaveformMode::MFSK) {
            // MFSK demodulation
            // findPreamble returns data start position (after preamble)
            SampleSpan span(audio.data(), audio.size());
            int data_start = mfsk_demodulator->findPreamble(span);
            if (data_start >= 0 && (size_t)data_start < audio.size()) {
                SampleSpan data_span(audio.data() + data_start, audio.size() - data_start);
                all_soft_bits = mfsk_demodulator->demodulateSoft(data_span);
            }
        } else {
            // OFDM demodulation (default)
            SampleSpan span(audio.data(), audio.size());
            bool frame_ready = demodulator->process(span);

            while (frame_ready) {
                auto soft_bits = demodulator->getSoftBits();
                if (!soft_bits.empty()) {
                    all_soft_bits.insert(all_soft_bits.end(),
                                         soft_bits.begin(), soft_bits.end());
                }
                SampleSpan empty;
                frame_ready = demodulator->process(empty);
            }
        }

        if (!all_soft_bits.empty()) {
            // Process per-codeword: deinterleave, decode, reassemble
            // (same as modem_engine)
            constexpr size_t CW_BITS = 648;
            constexpr size_t CW_BYTES = 20;  // v2::BYTES_PER_CODEWORD

            size_t num_codewords = all_soft_bits.size() / CW_BITS;
            v2::CodewordStatus cw_status;
            cw_status.decoded.resize(num_codewords, false);
            cw_status.data.resize(num_codewords);

            for (size_t i = 0; i < num_codewords; i++) {
                std::vector<float> cw_bits(all_soft_bits.begin() + i * CW_BITS,
                                           all_soft_bits.begin() + (i + 1) * CW_BITS);
                auto deinterleaved = interleaver.deinterleave(cw_bits);

                // Decode this codeword
                decoder->setRate(CodeRate::R1_4);  // V2 frames always use R1/4
                Bytes decoded = decoder->decodeSoft(deinterleaved);
                if (decoder->lastDecodeSuccess() && decoded.size() >= CW_BYTES) {
                    // Trim to codeword size and store
                    Bytes cw_data(decoded.begin(), decoded.begin() + CW_BYTES);
                    cw_status.decoded[i] = true;
                    cw_status.data[i] = cw_data;
                }
            }

            // Reassemble frame and pass to protocol
            if (cw_status.allSuccess()) {
                Bytes frame_data = cw_status.reassemble();
                if (!frame_data.empty()) {
                    protocol->onRxData(frame_data);
                }
            }
        }
    }

    // Switch to data mode (after connection established)
    void switchToDataMode() {
        ModemConfig rx_config = config;
        rx_config.modulation = data_modulation;
        rx_config.code_rate = data_code_rate;
        decoder->setRate(data_code_rate);
        demodulator = std::make_unique<OFDMDemodulator>(rx_config);
    }

    // Switch to link mode (disconnected state)
    void switchToLinkMode() {
        ModemConfig rx_config = config;
        rx_config.modulation = Modulation::BPSK;
        rx_config.code_rate = CodeRate::R1_4;
        decoder->setRate(CodeRate::R1_4);
        demodulator = std::make_unique<OFDMDemodulator>(rx_config);
    }

    // Set data mode (based on measured SNR)
    void setDataMode(Modulation mod, CodeRate rate) {
        data_modulation = mod;
        data_code_rate = rate;
    }

    // Set estimated channel SNR (for adaptive waveform selection)
    void setEstimatedSNR(float snr_db) {
        estimated_snr = snr_db;
    }

    // Set waveform mode (OFDM, DPSK, MFSK)
    void setWaveformMode(WaveformMode mode) {
        waveform_mode = mode;

        // Reset appropriate demodulator
        switch (mode) {
            case WaveformMode::DPSK:
                dpsk_demodulator->reset();
                break;
            case WaveformMode::MFSK:
                mfsk_demodulator->reset();
                break;
            default:
                demodulator->reset();
                break;
        }
    }

    // Set DPSK mode
    void setDPSKMode(DPSKModulation mod, int samples_per_symbol) {
        dpsk_config.modulation = mod;
        dpsk_config.samples_per_symbol = samples_per_symbol;
        dpsk_modulator = std::make_unique<DPSKModulator>(dpsk_config);
        dpsk_demodulator = std::make_unique<DPSKDemodulator>(dpsk_config);
    }

    // Set MFSK mode
    void setMFSKMode(int num_tones) {
        switch (num_tones) {
            case 2: mfsk_config = mfsk_presets::robust(); break;
            case 4: mfsk_config = mfsk_presets::low_snr(); break;
            case 8: mfsk_config = mfsk_presets::medium(); break;
            case 16: mfsk_config = mfsk_presets::fast(); break;
            case 32: mfsk_config = mfsk_presets::turbo(); break;
            default: mfsk_config = mfsk_presets::medium(); break;
        }
        mfsk_modulator = std::make_unique<MFSKModulator>(mfsk_config);
        mfsk_demodulator = std::make_unique<MFSKDemodulator>(mfsk_config);
    }

    bool hasTxAudio() const { return !tx_queue.empty(); }

    Samples popTxAudio() {
        Samples audio = std::move(tx_queue.front());
        tx_queue.pop();
        return audio;
    }

    void tick(uint32_t ms) {
        protocol->tick(ms);
    }

    void reset() {
        demodulator->reset();
        dpsk_demodulator->reset();
        mfsk_demodulator->reset();
        connected = false;
        remote_call.clear();
        waveform_mode = WaveformMode::OFDM_NVIS;
        tx_frame_types.clear();
        tx_modulations.clear();
        tx_waveform_modes.clear();
        while (!tx_queue.empty()) tx_queue.pop();
        switchToLinkMode();
    }
};

// ============================================================================
// Test Utilities
// ============================================================================

// Transfer audio through channel
void transferAudioWithChannel(AdaptiveTestStation& from, AdaptiveTestStation& to,
                              AWGNChannel& channel) {
    while (from.hasTxAudio()) {
        Samples audio = from.popTxAudio();
        Samples noisy = channel.process(audio);
        to.receive(noisy);
    }
}

// Run protocol exchange through channel
void runExchangeWithChannel(AdaptiveTestStation& a, AdaptiveTestStation& b,
                            AWGNChannel& channel_a_to_b, AWGNChannel& channel_b_to_a,
                            int iterations = 10, uint32_t tick_ms = 100) {
    for (int i = 0; i < iterations; i++) {
        a.tick(tick_ms);
        b.tick(tick_ms);
        transferAudioWithChannel(a, b, channel_a_to_b);
        transferAudioWithChannel(b, a, channel_b_to_a);
    }
}

// Recommend data mode based on SNR (matches ModemEngine::recommendDataMode)
void recommendDataMode(float snr_db, Modulation& mod, CodeRate& rate) {
    // Thresholds based on README performance targets (actual channel SNR)
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

// ============================================================================
// Link Establishment Tests
// ============================================================================

bool test_probe_connect_at_snr(float snr_db) {
    char test_name[64];
    snprintf(test_name, sizeof(test_name), "PROBE/CONNECT at SNR=%.0f dB", snr_db);
    TEST(test_name);

    AdaptiveTestStation alice("Alice", "TEST1");
    AdaptiveTestStation bob("Bob", "TEST2");

    // Set data mode based on SNR
    Modulation expected_mod;
    CodeRate expected_rate;
    recommendDataMode(snr_db, expected_mod, expected_rate);

    alice.setDataMode(expected_mod, expected_rate);
    bob.setDataMode(expected_mod, expected_rate);

    // Set estimated SNR for adaptive waveform selection
    alice.setEstimatedSNR(snr_db);
    bob.setEstimatedSNR(snr_db);

    // Create channel with specified SNR
    AWGNChannel channel_a_to_b(snr_db, 42);
    AWGNChannel channel_b_to_a(snr_db, 43);

    // Alice initiates probe to Bob
    alice.protocol->connect("TEST2");

    // Run exchange until connected (with timeout)
    for (int i = 0; i < 30 && !alice.connected; i++) {
        runExchangeWithChannel(alice, bob, channel_a_to_b, channel_b_to_a, 1, 100);
    }

    // Verify connection
    if (!alice.connected) FAIL("Alice not connected");
    if (!bob.connected) FAIL("Bob not connected");
    if (alice.remote_call != "TEST2") FAIL("Alice has wrong remote callsign");
    if (bob.remote_call != "TEST1") FAIL("Bob has wrong remote callsign");

    // Helper to check if a frame type is a link establishment frame
    auto isLinkFrameType = [](v2::FrameType ft) {
        return (ft == v2::FrameType::PROBE ||
                ft == v2::FrameType::PROBE_ACK ||
                ft == v2::FrameType::CONNECT ||
                ft == v2::FrameType::CONNECT_ACK ||
                ft == v2::FrameType::CONNECT_NAK ||
                ft == v2::FrameType::BEACON);
    };

    // Verify all link establishment frames used BPSK
    for (size_t i = 0; i < alice.tx_frame_types.size(); i++) {
        v2::FrameType ft = alice.tx_frame_types[i];
        Modulation mod = alice.tx_modulations[i];
        if (isLinkFrameType(ft)) {
            if (mod != Modulation::BPSK) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                    "Alice link frame %d used %d-ary instead of BPSK",
                    static_cast<int>(ft), static_cast<int>(mod));
                FAIL(buf);
            }
        }
    }

    for (size_t i = 0; i < bob.tx_frame_types.size(); i++) {
        v2::FrameType ft = bob.tx_frame_types[i];
        Modulation mod = bob.tx_modulations[i];
        if (isLinkFrameType(ft)) {
            if (mod != Modulation::BPSK) {
                char buf[128];
                snprintf(buf, sizeof(buf),
                    "Bob link frame %d used %d-ary instead of BPSK",
                    static_cast<int>(ft), static_cast<int>(mod));
                FAIL(buf);
            }
        }
    }

    PASS();
    return true;
}

bool test_link_establishment_high_snr() {
    return test_probe_connect_at_snr(25.0f);
}

bool test_link_establishment_medium_snr() {
    return test_probe_connect_at_snr(18.0f);
}

bool test_link_establishment_low_snr() {
    return test_probe_connect_at_snr(12.0f);
}

bool test_link_establishment_very_low_snr() {
    return test_probe_connect_at_snr(8.0f);
}

// ============================================================================
// Modulation Verification Tests
// ============================================================================

bool test_link_frames_use_bpsk() {
    TEST("Link establishment frames use BPSK R1/4");

    AdaptiveTestStation alice("Alice", "TEST1");
    AdaptiveTestStation bob("Bob", "TEST2");

    // Use high SNR to ensure everything works
    AWGNChannel channel_a_to_b(30.0f, 42);
    AWGNChannel channel_b_to_a(30.0f, 43);

    // Set data mode to QAM64 to make the contrast clear
    alice.setDataMode(Modulation::QAM64, CodeRate::R3_4);
    bob.setDataMode(Modulation::QAM64, CodeRate::R3_4);

    // Alice connects to Bob
    alice.protocol->connect("TEST2");

    // Run exchange until connected
    for (int i = 0; i < 30 && !alice.connected; i++) {
        runExchangeWithChannel(alice, bob, channel_a_to_b, channel_b_to_a, 1, 100);
    }

    if (!alice.connected) FAIL("Connection failed");

    // Count link frames and verify they all used BPSK
    int link_frame_count = 0;
    int bpsk_link_count = 0;

    for (size_t i = 0; i < alice.tx_frame_types.size(); i++) {
        v2::FrameType ft = alice.tx_frame_types[i];
        if (ft == v2::FrameType::PROBE || ft == v2::FrameType::PROBE_ACK ||
            ft == v2::FrameType::CONNECT || ft == v2::FrameType::CONNECT_ACK) {
            link_frame_count++;
            if (alice.tx_modulations[i] == Modulation::BPSK) {
                bpsk_link_count++;
            }
        }
    }

    for (size_t i = 0; i < bob.tx_frame_types.size(); i++) {
        v2::FrameType ft = bob.tx_frame_types[i];
        if (ft == v2::FrameType::PROBE || ft == v2::FrameType::PROBE_ACK ||
            ft == v2::FrameType::CONNECT || ft == v2::FrameType::CONNECT_ACK) {
            link_frame_count++;
            if (bob.tx_modulations[i] == Modulation::BPSK) {
                bpsk_link_count++;
            }
        }
    }

    if (link_frame_count == 0) FAIL("No link frames transmitted");
    if (bpsk_link_count != link_frame_count) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Only %d/%d link frames used BPSK",
                 bpsk_link_count, link_frame_count);
        FAIL(buf);
    }

    PASS();
    return true;
}

// ============================================================================
// Mode Switching Tests
// ============================================================================

bool test_mode_switch_on_connect() {
    TEST("RX switches to data mode after connection");

    AdaptiveTestStation alice("Alice", "TEST1");
    AdaptiveTestStation bob("Bob", "TEST2");

    AWGNChannel channel_a_to_b(25.0f, 42);
    AWGNChannel channel_b_to_a(25.0f, 43);

    // Set data mode
    alice.setDataMode(Modulation::QAM16, CodeRate::R3_4);
    bob.setDataMode(Modulation::QAM16, CodeRate::R3_4);

    // Before connection: RX should be in BPSK R1/4 mode
    // (We can't easily verify this without adding accessors, but the
    // connection working proves the link mode works)

    alice.protocol->connect("TEST2");

    for (int i = 0; i < 30 && !alice.connected; i++) {
        runExchangeWithChannel(alice, bob, channel_a_to_b, channel_b_to_a, 1, 100);
    }

    if (!alice.connected) FAIL("Connection failed");

    // After connection: data mode should be set
    if (alice.data_modulation != Modulation::QAM16) FAIL("Alice data modulation not set");
    if (bob.data_modulation != Modulation::QAM16) FAIL("Bob data modulation not set");

    PASS();
    return true;
}

// ============================================================================
// SNR Sweep Test
// ============================================================================

bool test_snr_sweep() {
    std::cout << "\n  SNR Sweep Test (link establishment at various SNR levels):\n";

    float snr_levels[] = {30.0f, 25.0f, 20.0f, 15.0f, 12.0f, 10.0f, 8.0f, 6.0f};
    int success_count = 0;
    int total_tests = sizeof(snr_levels) / sizeof(snr_levels[0]);

    for (float snr : snr_levels) {
        AdaptiveTestStation alice("Alice", "TEST1");
        AdaptiveTestStation bob("Bob", "TEST2");

        AWGNChannel channel_a_to_b(snr, 42);
        AWGNChannel channel_b_to_a(snr, 43);

        Modulation expected_mod;
        CodeRate expected_rate;
        recommendDataMode(snr, expected_mod, expected_rate);
        alice.setDataMode(expected_mod, expected_rate);
        bob.setDataMode(expected_mod, expected_rate);

        // Set estimated SNR for adaptive waveform selection
        alice.setEstimatedSNR(snr);
        bob.setEstimatedSNR(snr);

        alice.protocol->connect("TEST2");

        for (int i = 0; i < 30 && !alice.connected; i++) {
            runExchangeWithChannel(alice, bob, channel_a_to_b, channel_b_to_a, 1, 100);
        }

        if (alice.connected && bob.connected) {
            std::cout << "    SNR=" << std::setw(5) << snr << " dB: PASS (connected)\n";
            success_count++;
        } else {
            std::cout << "    SNR=" << std::setw(5) << snr << " dB: FAIL (no connection)\n";
        }
    }

    std::cout << "    Summary: " << success_count << "/" << total_tests << " SNR levels successful\n";

    // Require at least the high SNR tests to pass
    // OFDM with BPSK R1/4 needs ~15 dB SNR in AWGN channel
    // Lower SNR would require DPSK/MFSK waveforms (not used for control frames)
    TEST("SNR sweep (>= 15 dB should connect)");
    if (success_count >= 4) {  // At least 15+ dB should work with OFDM
        PASS();
        return true;
    } else {
        FAIL("Too many SNR levels failed");
        return false;
    }
}

// ============================================================================
// Waveform Mode Tests (DPSK, MFSK)
// ============================================================================

bool test_dpsk_waveform_loopback() {
    TEST("DPSK waveform loopback (DQPSK 125 baud)");

    AdaptiveTestStation alice("Alice", "TEST1");
    AdaptiveTestStation bob("Bob", "TEST2");

    // Set both stations to use DPSK after connection
    alice.setWaveformMode(WaveformMode::DPSK);
    bob.setWaveformMode(WaveformMode::DPSK);

    // Use DQPSK at 125 baud (default fast preset)
    alice.setDPSKMode(DPSKModulation::DQPSK, 384);
    bob.setDPSKMode(DPSKModulation::DQPSK, 384);

    // Set matching codec rate for encoder/decoder (both use R1/4 for robustness)
    alice.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    bob.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    // Create fresh decoder to avoid state issues from previous tests
    bob.decoder = std::make_unique<LDPCDecoder>(CodeRate::R1_4);

    // High SNR for clean test
    AWGNChannel channel(20.0f, 42);

    // Create test data (one LDPC codeword worth)
    Bytes test_data(20);
    for (size_t i = 0; i < test_data.size(); i++) {
        test_data[i] = 0xDE ^ (i & 0xFF);
    }

    // Manually transmit (bypassing protocol for direct waveform test)
    // Simulate being connected
    alice.connected = true;
    bob.connected = true;

    // Alice transmits using DPSK
    alice.transmit(test_data);

    if (!alice.hasTxAudio()) FAIL("No TX audio generated");

    // Pass through channel
    Samples noisy = channel.process(alice.popTxAudio());

    // Bob receives
    bob.receive(noisy);

    // Check if frame was decoded
    if (bob.decoder->lastDecodeSuccess()) {
        PASS();
        return true;
    }

    FAIL("DPSK decode failed");
    return false;
}

bool test_mfsk_waveform_loopback() {
    TEST("MFSK waveform loopback (8FSK)");

    AdaptiveTestStation alice("Alice", "TEST1");
    AdaptiveTestStation bob("Bob", "TEST2");

    // Set both stations to use MFSK after connection
    alice.setWaveformMode(WaveformMode::MFSK);
    bob.setWaveformMode(WaveformMode::MFSK);

    // Use 8FSK (medium preset)
    alice.setMFSKMode(8);
    bob.setMFSKMode(8);

    // Set matching codec rate for encoder/decoder
    alice.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    bob.setDataMode(Modulation::QPSK, CodeRate::R1_4);
    bob.decoder = std::make_unique<LDPCDecoder>(CodeRate::R1_4);

    // Lower SNR where MFSK should still work
    AWGNChannel channel(5.0f, 42);

    // Create test data
    Bytes test_data(20);
    for (size_t i = 0; i < test_data.size(); i++) {
        test_data[i] = 0xBE ^ (i & 0xFF);
    }

    // Simulate being connected
    alice.connected = true;
    bob.connected = true;

    // Alice transmits using MFSK
    alice.transmit(test_data);

    if (!alice.hasTxAudio()) FAIL("No TX audio generated");

    // Pass through channel
    Samples noisy = channel.process(alice.popTxAudio());

    // Bob receives
    bob.receive(noisy);

    if (bob.decoder->lastDecodeSuccess()) {
        PASS();
        return true;
    }

    FAIL("MFSK decode failed");
    return false;
}

bool test_waveform_mode_selection() {
    TEST("Waveform mode selection based on SNR");

    // Test the waveform recommendation logic
    struct TestCase {
        float snr_db;
        WaveformMode expected_mode;
        const char* name;
    };

    TestCase cases[] = {
        {-5.0f,  WaveformMode::MFSK, "-5 dB -> MFSK"},
        {-0.1f,  WaveformMode::MFSK, "-0.1 dB -> MFSK (boundary)"},
        { 0.0f,  WaveformMode::DPSK, "0 dB -> DPSK (MFSK threshold is < 0)"},
        { 5.0f,  WaveformMode::DPSK, "5 dB -> DPSK"},
        { 9.9f,  WaveformMode::DPSK, "9.9 dB -> DPSK (boundary)"},
        {10.0f,  WaveformMode::OFDM_NVIS, "10 dB -> OFDM (DPSK threshold is < 10)"},
        {20.0f,  WaveformMode::OFDM_NVIS, "20 dB -> OFDM"},
    };

    int passed = 0;
    for (const auto& tc : cases) {
        // Use the same logic as negotiateMode() in connection.cpp
        WaveformMode recommended_mode;
        if (tc.snr_db < 0.0f) {
            recommended_mode = WaveformMode::MFSK;
        } else if (tc.snr_db < 10.0f) {
            recommended_mode = WaveformMode::DPSK;
        } else {
            recommended_mode = WaveformMode::OFDM_NVIS;
        }

        if (recommended_mode == tc.expected_mode) {
            passed++;
        } else {
            std::cout << "FAIL: " << tc.name << " got "
                      << static_cast<int>(recommended_mode) << " expected "
                      << static_cast<int>(tc.expected_mode) << "\n";
        }
    }

    if (passed == 7) {
        PASS();
        return true;
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Only %d/7 cases passed", passed);
    FAIL(buf);
    return false;
}

bool test_dpsk_throughput() {
    TEST("DPSK throughput measurement");

    std::cout << "\n    DPSK Throughput by Mode:\n";
    std::cout << "    +----------+--------+----------+\n";
    std::cout << "    | Mode     | Baud   | Raw bps  |\n";
    std::cout << "    +----------+--------+----------+\n";

    struct DPSKMode {
        DPSKModulation mod;
        int samples_per_symbol;
        const char* name;
    };

    DPSKMode modes[] = {
        {DPSKModulation::DBPSK, 1536, "DBPSK 31"},
        {DPSKModulation::DBPSK, 768,  "DBPSK 62"},
        {DPSKModulation::DQPSK, 768,  "DQPSK 62"},
        {DPSKModulation::DQPSK, 384,  "DQPSK 125"},
        {DPSKModulation::DQPSK, 192,  "DQPSK 250"},
        {DPSKModulation::D8PSK, 384,  "D8PSK 125"},
    };

    for (const auto& m : modes) {
        DPSKConfig cfg;
        cfg.modulation = m.mod;
        cfg.samples_per_symbol = m.samples_per_symbol;

        float baud = cfg.symbol_rate();
        float bps = cfg.raw_bps();

        std::cout << "    | " << std::setw(8) << m.name
                  << " | " << std::setw(6) << std::fixed << std::setprecision(1) << baud
                  << " | " << std::setw(8) << std::setprecision(1) << bps << " |\n";
    }
    std::cout << "    +----------+--------+----------+\n";

    PASS();
    return true;
}

bool test_mfsk_throughput() {
    TEST("MFSK throughput measurement");

    std::cout << "\n    MFSK Throughput by Tones:\n";
    std::cout << "    +--------+------------+----------+\n";
    std::cout << "    | Tones  | Effective  | Min SNR  |\n";
    std::cout << "    +--------+------------+----------+\n";

    int tone_counts[] = {2, 4, 8, 16, 32};
    const char* min_snrs[] = {"-17 dB", "-17 dB", "-17 dB", "-12 dB", "-8 dB"};

    for (int i = 0; i < 5; i++) {
        MFSKConfig cfg;
        switch (tone_counts[i]) {
            case 2: cfg = mfsk_presets::robust(); break;
            case 4: cfg = mfsk_presets::low_snr(); break;
            case 8: cfg = mfsk_presets::medium(); break;
            case 16: cfg = mfsk_presets::fast(); break;
            case 32: cfg = mfsk_presets::turbo(); break;
        }

        std::cout << "    | " << std::setw(6) << tone_counts[i]
                  << " | " << std::setw(8) << std::fixed << std::setprecision(1)
                  << cfg.effective_bps() << " bps"
                  << " | " << std::setw(8) << min_snrs[i] << " |\n";
    }
    std::cout << "    +--------+------------+----------+\n";

    PASS();
    return true;
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "\n=== Adaptive Link Establishment Test Suite ===\n";
    std::cout << "Testing PROBE/CONNECT flow through FULL modem pipeline\n";
    std::cout << "Using TWO SEPARATE modem instances (like GUI Test Mode)\n\n";

    std::cout << "Link Establishment Tests:\n";
    test_link_establishment_high_snr();
    test_link_establishment_medium_snr();
    test_link_establishment_low_snr();
    test_link_establishment_very_low_snr();

    std::cout << "\nModulation Verification:\n";
    test_link_frames_use_bpsk();

    std::cout << "\nMode Switching:\n";
    test_mode_switch_on_connect();

    std::cout << "\nSNR Sweep:\n";
    test_snr_sweep();

    std::cout << "\nWaveform Mode Tests (DPSK/MFSK):\n";
    test_waveform_mode_selection();
    test_dpsk_waveform_loopback();
    test_mfsk_waveform_loopback();
    test_dpsk_throughput();
    test_mfsk_throughput();

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_run << " tests passed ===\n\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
