/**
 * Protocol v2 Frame + Modem Integration Test
 *
 * Tests v2 frames through the full modem pipeline (LDPC + OFDM):
 * 1. Control frames (1 codeword)
 * 2. Data frames (multiple codewords)
 * 3. Per-codeword recovery with NACK/retransmit
 *
 * Uses the same loopback approach as test_modem_loopback.cpp
 */

#include <iostream>
#include <cassert>
#include <cstring>
#include "../src/protocol/frame_v2.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"

using namespace ultra;
namespace v2 = ultra::protocol::v2;

int tests_passed = 0;
int tests_failed = 0;

#define TEST(name) \
    std::cout << "Testing " << name << "... "; \
    try

#define PASS() \
    std::cout << "PASS\n"; \
    tests_passed++;

#define FAIL(msg) \
    std::cout << "FAIL: " << msg << "\n"; \
    tests_failed++;

// ============================================================================
// Control Frame Through Modem (1 codeword)
// ============================================================================

void test_control_frame_through_modem() {
    TEST("v2 control frame through modem (1 codeword)") {
        // NOTE: Single codeword frames (81 bytes) produce very short OFDM transmissions
        // (~11 symbols) which may not have enough data for stable channel estimation.
        // In practice, control frames should be preceded by preamble/LTS which helps.
        // This test may fail in pure software loopback but work over real audio.
        //
        // Use same config as working modem tests
        ModemConfig config;
        config.code_rate = CodeRate::R1_4;
        config.modulation = Modulation::QPSK;

        OFDMModulator mod(config);
        OFDMDemodulator demod(config);
        LDPCDecoder decoder(config.code_rate);

        // Create control frame
        auto probe = v2::ControlFrame::makeProbe("VA2MVR", "W1AW");
        probe.seq = 42;
        auto frame_data = probe.serialize();
        assert(frame_data.size() == 20);

        std::cout << "\n  Control frame: 20 bytes\n";

        // LDPC encode (1 codeword) using v2 function
        auto ldpc_encoded = v2::encodeFrameWithLDPC(frame_data);
        assert(ldpc_encoded.size() == 1);
        std::cout << "  LDPC encoded: " << ldpc_encoded[0].size() << " bytes\n";

        // Modulate
        Samples preamble = mod.generatePreamble();
        Samples data = mod.modulate(ldpc_encoded[0], config.modulation);

        Samples tx_signal;
        tx_signal.reserve(preamble.size() + data.size());
        tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
        tx_signal.insert(tx_signal.end(), data.begin(), data.end());

        // Scale signal
        float max_val = 0;
        for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_signal) s *= scale;
        }

        std::cout << "  TX signal: " << tx_signal.size() << " samples\n";

        // Demodulate
        SampleSpan span(tx_signal.data(), tx_signal.size());
        demod.process(span);

        if (!demod.isSynced()) {
            FAIL("Demodulator did not sync");
            return;
        }

        // Collect soft bits
        std::vector<float> all_soft_bits;
        int iterations = 0;
        while (iterations < 100) {
            auto bits = demod.getSoftBits();
            if (bits.empty()) {
                demod.process({});
                bits = demod.getSoftBits();
            }
            if (bits.empty()) break;
            all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
            iterations++;
        }

        std::cout << "  Soft bits: " << all_soft_bits.size() << "\n";

        if (all_soft_bits.size() < v2::LDPC_CODEWORD_BITS) {
            FAIL("Not enough soft bits from OFDM");
            return;
        }

        // LDPC decode
        auto decoded_data = decoder.decodeSoft(all_soft_bits);
        if (!decoder.lastDecodeSuccess()) {
            FAIL("LDPC decode failed");
            return;
        }

        // Trim to 20 bytes and parse
        Bytes decoded_frame(decoded_data.begin(), decoded_data.begin() + 20);
        auto parsed = v2::ControlFrame::deserialize(decoded_frame);

        if (!parsed.has_value()) {
            FAIL("Control frame parse failed");
            return;
        }

        if (parsed->type != v2::FrameType::PROBE) {
            FAIL("Wrong frame type");
            return;
        }

        if (parsed->seq != 42) {
            FAIL("Wrong sequence number");
            return;
        }

        std::cout << "  Decoded: PROBE from VA2MVR to W1AW, seq=" << parsed->seq << "\n  ";
        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

// ============================================================================
// Data Frame Through Modem (multiple codewords)
// ============================================================================

void test_data_frame_through_modem() {
    TEST("v2 data frame through modem (multiple codewords)") {
        ModemConfig config;
        config.code_rate = CodeRate::R1_4;
        config.modulation = Modulation::QPSK;

        OFDMModulator mod(config);
        OFDMDemodulator demod(config);
        LDPCDecoder decoder(config.code_rate);

        // Create data frame with text message
        std::string message = "Hello from VA2MVR! This is a v2 protocol test message.";
        auto data_frame = v2::DataFrame::makeData("VA2MVR", "W1AW", 123, message);
        auto frame_data = data_frame.serialize();

        std::cout << "\n  Message: \"" << message << "\"\n";
        std::cout << "  Frame: " << frame_data.size() << " bytes, "
                  << (int)data_frame.total_cw << " codewords\n";

        // LDPC encode (N codewords)
        auto ldpc_encoded = v2::encodeFrameWithLDPC(frame_data);
        assert(ldpc_encoded.size() == data_frame.total_cw);

        // Concatenate all encoded codewords for transmission
        Bytes all_encoded;
        for (const auto& cw : ldpc_encoded) {
            all_encoded.insert(all_encoded.end(), cw.begin(), cw.end());
        }
        std::cout << "  Total encoded: " << all_encoded.size() << " bytes\n";

        // Modulate
        Samples preamble = mod.generatePreamble();
        Samples data = mod.modulate(all_encoded, config.modulation);

        Samples tx_signal;
        tx_signal.reserve(preamble.size() + data.size());
        tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
        tx_signal.insert(tx_signal.end(), data.begin(), data.end());

        // Scale signal
        float max_val = 0;
        for (float s : tx_signal) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            float scale = 0.5f / max_val;
            for (float& s : tx_signal) s *= scale;
        }

        std::cout << "  TX signal: " << tx_signal.size() << " samples\n";

        // Demodulate
        SampleSpan span(tx_signal.data(), tx_signal.size());
        demod.process(span);

        if (!demod.isSynced()) {
            FAIL("Demodulator did not sync");
            return;
        }

        // Collect all soft bits
        std::vector<float> all_soft_bits;
        int iterations = 0;
        while (iterations < 200) {
            auto bits = demod.getSoftBits();
            if (bits.empty()) {
                demod.process({});
                bits = demod.getSoftBits();
            }
            if (bits.empty()) break;
            all_soft_bits.insert(all_soft_bits.end(), bits.begin(), bits.end());
            iterations++;
        }

        std::cout << "  Total soft bits: " << all_soft_bits.size() << "\n";

        // Decode each codeword
        v2::CodewordStatus rx_status;
        rx_status.initForFrame(data_frame.total_cw);

        size_t bits_per_cw = v2::LDPC_CODEWORD_BITS;
        for (size_t cw = 0; cw < data_frame.total_cw; cw++) {
            size_t start = cw * bits_per_cw;
            if (start + bits_per_cw > all_soft_bits.size()) {
                std::cout << "  CW" << cw << ": Not enough soft bits\n";
                continue;
            }

            std::vector<float> cw_bits(all_soft_bits.begin() + start,
                                       all_soft_bits.begin() + start + bits_per_cw);

            auto decoded_data = decoder.decodeSoft(cw_bits);
            if (decoder.lastDecodeSuccess()) {
                Bytes cw_data(decoded_data.begin(), decoded_data.begin() + v2::BYTES_PER_CODEWORD);
                rx_status.decoded[cw] = true;
                rx_status.data[cw] = cw_data;
                std::cout << "  CW" << cw << ": OK\n";
            } else {
                std::cout << "  CW" << cw << ": LDPC decode failed\n";
            }
        }

        if (!rx_status.allSuccess()) {
            std::cout << "  Failed codewords: " << rx_status.countFailures() << "\n";
            FAIL("Not all codewords decoded");
            return;
        }

        // Reassemble
        auto reassembled = rx_status.reassemble();
        if (reassembled.size() != frame_data.size()) {
            FAIL("Reassembled size mismatch");
            return;
        }

        // Parse and verify
        auto parsed = v2::DataFrame::deserialize(reassembled);
        if (!parsed.has_value()) {
            FAIL("Data frame parse failed");
            return;
        }

        if (parsed->payloadAsText() != message) {
            std::cout << "  Expected: \"" << message << "\"\n";
            std::cout << "  Got: \"" << parsed->payloadAsText() << "\"\n";
            FAIL("Message mismatch");
            return;
        }

        std::cout << "  Recovered: \"" << parsed->payloadAsText() << "\"\n  ";
        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

// ============================================================================
// Per-codeword Recovery Test (simulated loss, not through modem)
// ============================================================================

void test_recovery_simulated() {
    TEST("v2 per-codeword recovery (simulated loss)") {
        // This test verifies the recovery logic works
        // without the modem - using perfect soft bits

        std::string message = "This message will have codeword 2 recovered via NACK!";
        auto data_frame = v2::DataFrame::makeData("VA2MVR", "W1AW", 500, message);
        auto frame_data = data_frame.serialize();

        std::cout << "\n  Message: " << message.size() << " bytes, "
                  << (int)data_frame.total_cw << " codewords\n";

        // LDPC encode
        auto ldpc_encoded = v2::encodeFrameWithLDPC(frame_data);

        // Convert to soft bits with CW2 corrupted
        std::vector<std::vector<float>> all_soft(ldpc_encoded.size());
        for (size_t cw = 0; cw < ldpc_encoded.size(); cw++) {
            for (uint8_t byte : ldpc_encoded[cw]) {
                for (int b = 7; b >= 0; --b) {
                    int bit = (byte >> b) & 1;
                    float llr = bit ? -5.0f : 5.0f;
                    // Corrupt CW2
                    if (cw == 2) llr = -llr;
                    all_soft[cw].push_back(llr);
                }
            }
        }

        // Decode
        auto rx_status = v2::decodeCodewordsWithLDPC(all_soft);

        assert(!rx_status.allSuccess());
        assert(!rx_status.decoded[2]);
        std::cout << "  Initial decode: CW2 failed (as expected)\n";

        // Generate NACK
        uint32_t nack_bitmap = rx_status.getNackBitmap();
        assert(nack_bitmap == 0x04);
        std::cout << "  NACK bitmap: 0x" << std::hex << nack_bitmap << std::dec << "\n";

        // Retransmit CW2 (with correct LLRs this time)
        std::vector<float> retx_soft;
        for (uint8_t byte : ldpc_encoded[2]) {
            for (int b = 7; b >= 0; --b) {
                int bit = (byte >> b) & 1;
                retx_soft.push_back(bit ? -5.0f : 5.0f);
            }
        }

        auto [success, retx_data] = v2::decodeSingleCodeword(retx_soft);
        assert(success);

        // Merge
        rx_status.mergeCodeword(2, retx_data);
        assert(rx_status.allSuccess());
        std::cout << "  After retransmit: All codewords OK\n";

        // Verify message
        auto reassembled = rx_status.reassemble();
        auto parsed = v2::DataFrame::deserialize(reassembled);
        assert(parsed.has_value());
        assert(parsed->payloadAsText() == message);

        std::cout << "  Recovered: \"" << parsed->payloadAsText() << "\"\n  ";
        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=== Protocol v2 + Modem Integration Tests ===\n\n";

    test_control_frame_through_modem();
    test_data_frame_through_modem();
    test_recovery_simulated();

    std::cout << "\n=== Results ===\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";

    return tests_failed > 0 ? 1 : 0;
}
