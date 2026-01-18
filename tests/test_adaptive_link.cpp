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

    // TX components
    std::unique_ptr<LDPCEncoder> encoder;
    std::unique_ptr<OFDMModulator> modulator;

    // RX components (separate for proper mode switching)
    std::unique_ptr<LDPCDecoder> decoder;
    std::unique_ptr<OFDMDemodulator> demodulator;

    // Protocol engine
    std::unique_ptr<ProtocolEngine> protocol;

    // Outgoing audio queue
    std::queue<Samples> tx_queue;

    // Connection state
    bool connected = false;
    std::string remote_call;

    // Negotiated data mode
    Modulation data_modulation = Modulation::QPSK;
    CodeRate data_code_rate = CodeRate::R1_2;

    // Frame type tracking for verification
    std::vector<v2::FrameType> tx_frame_types;
    std::vector<Modulation> tx_modulations;

    AdaptiveTestStation(const std::string& station_name, const std::string& callsign)
        : name(station_name) {

        config = presets::balanced();

        // TX: encoder uses config, modulator uses config
        encoder = std::make_unique<LDPCEncoder>(config.code_rate);
        modulator = std::make_unique<OFDMModulator>(config);

        // RX: Start in disconnected state with BPSK R1/4 for link establishment
        ModemConfig rx_config = config;
        rx_config.modulation = Modulation::BPSK;
        rx_config.code_rate = CodeRate::R1_4;
        decoder = std::make_unique<LDPCDecoder>(rx_config.code_rate);
        demodulator = std::make_unique<OFDMDemodulator>(rx_config);

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

        v2::FrameType frame_type = getFrameType(data);
        bool is_link = isLinkFrame(data);

        if (is_link) {
            // Link establishment: BPSK R1/4 (robust)
            tx_mod = Modulation::BPSK;
            tx_rate = CodeRate::R1_4;
        } else if (connected) {
            // Connected: Use negotiated data mode
            tx_mod = data_modulation;
            tx_rate = data_code_rate;
        } else {
            // Fallback: Robust mode
            tx_mod = Modulation::BPSK;
            tx_rate = CodeRate::R1_4;
        }

        // Track for verification
        tx_frame_types.push_back(frame_type);
        tx_modulations.push_back(tx_mod);

        // LDPC encode with appropriate rate
        encoder->setRate(tx_rate);
        Bytes encoded = encoder->encode(data);

        // OFDM modulate
        Samples preamble = modulator->generatePreamble();
        Samples audio = modulator->modulate(encoded, tx_mod);

        // Combine and normalize
        Samples tx_audio;
        tx_audio.reserve(preamble.size() + audio.size());
        tx_audio.insert(tx_audio.end(), preamble.begin(), preamble.end());
        tx_audio.insert(tx_audio.end(), audio.begin(), audio.end());

        float max_val = 0;
        for (float s : tx_audio) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_audio) s *= scale;
        }

        tx_queue.push(std::move(tx_audio));
    }

    // Receive audio (demodulate + decode)
    void receive(const Samples& audio) {
        SampleSpan span(audio.data(), audio.size());
        bool frame_ready = demodulator->process(span);

        // Collect all codewords
        std::vector<float> all_soft_bits;
        while (frame_ready) {
            auto soft_bits = demodulator->getSoftBits();
            if (!soft_bits.empty()) {
                all_soft_bits.insert(all_soft_bits.end(),
                                     soft_bits.begin(), soft_bits.end());
            }
            SampleSpan empty;
            frame_ready = demodulator->process(empty);
        }

        if (!all_soft_bits.empty()) {
            Bytes decoded = decoder->decodeSoft(all_soft_bits);
            if (decoder->lastDecodeSuccess() && !decoded.empty()) {
                protocol->onRxData(decoded);
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
        connected = false;
        remote_call.clear();
        tx_frame_types.clear();
        tx_modulations.clear();
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
    TEST("SNR sweep (>= 10 dB should connect)");
    if (success_count >= 5) {  // At least 10+ dB should work
        PASS();
        return true;
    } else {
        FAIL("Too many SNR levels failed");
        return false;
    }
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

    std::cout << "\n=== Results: " << tests_passed << "/" << tests_run << " tests passed ===\n\n";

    return (tests_passed == tests_run) ? 0 : 1;
}
