#include <iostream>
#include <cassert>
#include <cstring>
#include "../src/protocol/frame_v2.hpp"

using namespace ultra::protocol::v2;

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

void test_callsign_hashing() {
    TEST("callsign hashing") {
        // Hash should be deterministic
        uint32_t h1 = hashCallsign("VA2MVR");
        uint32_t h2 = hashCallsign("VA2MVR");
        assert(h1 == h2);

        // Case insensitive
        uint32_t h3 = hashCallsign("va2mvr");
        assert(h1 == h3);

        // Different callsigns should (usually) have different hashes
        uint32_t h4 = hashCallsign("W1AW");
        assert(h1 != h4);

        // Hash should be 24 bits
        assert(h1 <= 0xFFFFFF);
        assert(h4 <= 0xFFFFFF);

        // Test broadcast hash
        uint32_t hb = hashCallsign("CQ");
        assert(hb <= 0xFFFFFF);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_control_frame_size() {
    TEST("control frame size = 20 bytes") {
        auto probe = ControlFrame::makeProbe("VA2MVR", "W1AW");
        auto serialized = probe.serialize();

        // Must be exactly 20 bytes (1 codeword)
        assert(serialized.size() == 20);
        assert(serialized.size() == ControlFrame::SIZE);
        assert(serialized.size() == BYTES_PER_CODEWORD);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_control_frame_roundtrip() {
    TEST("control frame serialize/deserialize roundtrip") {
        auto original = ControlFrame::makeProbe("VA2MVR", "W1AW");
        original.seq = 1234;

        auto serialized = original.serialize();
        auto parsed = ControlFrame::deserialize(serialized);

        assert(parsed.has_value());
        assert(parsed->type == original.type);
        assert(parsed->flags == original.flags);
        assert(parsed->seq == original.seq);
        assert(parsed->src_hash == original.src_hash);
        assert(parsed->dst_hash == original.dst_hash);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_control_frame_crc() {
    TEST("control frame CRC validation") {
        auto probe = ControlFrame::makeProbe("VA2MVR", "W1AW");
        auto serialized = probe.serialize();

        // Valid CRC should parse
        auto valid = ControlFrame::deserialize(serialized);
        assert(valid.has_value());

        // Corrupt a byte - should fail CRC
        serialized[10] ^= 0xFF;
        auto invalid = ControlFrame::deserialize(serialized);
        assert(!invalid.has_value());

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_control_frame_magic() {
    TEST("control frame magic = 0x554C (UL)") {
        auto probe = ControlFrame::makeProbe("VA2MVR", "W1AW");
        auto serialized = probe.serialize();

        // First two bytes should be "UL"
        assert(serialized[0] == 0x55);  // 'U'
        assert(serialized[1] == 0x4C);  // 'L'

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_data_frame_codeword_count() {
    TEST("data frame codeword calculation") {
        // Empty payload: header(17) + CRC(2) = 19 bytes = 1 codeword
        assert(DataFrame::calculateCodewords(0) == 1);

        // 3 bytes payload: 17 + 3 + 2 = 22 bytes = 2 codewords
        assert(DataFrame::calculateCodewords(3) == 2);

        // 20 bytes payload: 17 + 20 + 2 = 39 bytes = 2 codewords
        assert(DataFrame::calculateCodewords(20) == 2);

        // 21 bytes payload: 17 + 21 + 2 = 40 bytes = 2 codewords
        assert(DataFrame::calculateCodewords(21) == 2);

        // 23 bytes payload: 17 + 23 + 2 = 42 bytes = 3 codewords
        assert(DataFrame::calculateCodewords(23) == 3);

        // 100 bytes payload: 17 + 100 + 2 = 119 bytes = 6 codewords
        assert(DataFrame::calculateCodewords(100) == 6);

        // 256 bytes payload: 17 + 256 + 2 = 275 bytes = 14 codewords
        assert(DataFrame::calculateCodewords(256) == 14);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_data_frame_roundtrip() {
    TEST("data frame serialize/deserialize roundtrip") {
        std::string message = "Hello, this is a test message from VA2MVR!";
        auto original = DataFrame::makeData("VA2MVR", "W1AW", 42, message);

        auto serialized = original.serialize();
        auto parsed = DataFrame::deserialize(serialized);

        assert(parsed.has_value());
        assert(parsed->type == original.type);
        assert(parsed->seq == original.seq);
        assert(parsed->src_hash == original.src_hash);
        assert(parsed->dst_hash == original.dst_hash);
        assert(parsed->total_cw == original.total_cw);
        assert(parsed->payload_len == original.payload_len);
        assert(parsed->payload == original.payload);
        assert(parsed->payloadAsText() == message);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_data_frame_crc() {
    TEST("data frame CRC validation") {
        auto data = DataFrame::makeData("VA2MVR", "W1AW", 1, "Test data");
        auto serialized = data.serialize();

        // Valid should parse
        auto valid = DataFrame::deserialize(serialized);
        assert(valid.has_value());

        // Corrupt header - should fail header CRC
        auto corrupt_header = serialized;
        corrupt_header[5] ^= 0xFF;
        auto invalid1 = DataFrame::deserialize(corrupt_header);
        assert(!invalid1.has_value());

        // Corrupt payload - should fail frame CRC
        auto corrupt_payload = serialized;
        corrupt_payload[20] ^= 0xFF;
        auto invalid2 = DataFrame::deserialize(corrupt_payload);
        assert(!invalid2.has_value());

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_split_into_codewords() {
    TEST("split into codewords") {
        std::string message = "Hello, World!";  // 13 bytes
        auto data = DataFrame::makeData("VA2MVR", "W1AW", 1, message);
        auto serialized = data.serialize();

        // 17 + 13 + 2 = 32 bytes = 2 codewords
        auto codewords = splitIntoCodewords(serialized);
        assert(codewords.size() == 2);
        assert(codewords[0].size() == 20);
        assert(codewords[1].size() == 20);  // Padded

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_reassemble_codewords() {
    TEST("reassemble codewords") {
        std::string message = "Hello, World!";  // 13 bytes
        auto data = DataFrame::makeData("VA2MVR", "W1AW", 1, message);
        auto original = data.serialize();

        // Split and reassemble
        auto codewords = splitIntoCodewords(original);
        auto reassembled = reassembleCodewords(codewords, original.size());

        assert(reassembled == original);

        // Should still parse correctly
        auto parsed = DataFrame::deserialize(reassembled);
        assert(parsed.has_value());
        assert(parsed->payloadAsText() == message);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_nack_payload() {
    TEST("NACK payload encode/decode") {
        NackPayload original;
        original.frame_seq = 1234;
        original.cw_bitmap = 0b10101010;  // CWs 1,3,5,7 failed

        uint8_t buffer[6];
        original.encode(buffer);

        auto decoded = NackPayload::decode(buffer);
        assert(decoded.frame_seq == original.frame_seq);
        assert(decoded.cw_bitmap == original.cw_bitmap);
        assert(decoded.countFailed() == 4);
        assert(!decoded.isFailed(0));
        assert(decoded.isFailed(1));
        assert(!decoded.isFailed(2));
        assert(decoded.isFailed(3));

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_nack_frame() {
    TEST("NACK control frame") {
        uint32_t bitmap = 0b00000101;  // CWs 0 and 2 failed
        auto nack = ControlFrame::makeNack("W1AW", "VA2MVR", 100, bitmap);

        auto serialized = nack.serialize();
        assert(serialized.size() == 20);  // Still fits in 1 codeword

        auto parsed = ControlFrame::deserialize(serialized);
        assert(parsed.has_value());
        assert(parsed->type == FrameType::NACK);

        // Decode NACK payload
        auto np = NackPayload::decode(parsed->payload);
        assert(np.frame_seq == 100);
        assert(np.cw_bitmap == bitmap);
        assert(np.isFailed(0));
        assert(!np.isFailed(1));
        assert(np.isFailed(2));

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_codeword_status() {
    TEST("codeword status tracking") {
        CodewordStatus status;
        status.decoded = {true, false, true, false, true};

        assert(!status.allSuccess());
        assert(status.countFailures() == 2);

        uint32_t bitmap = status.getNackBitmap();
        assert(bitmap == 0b01010);  // Bits 1 and 3 set (failed)

        // All success case
        CodewordStatus success;
        success.decoded = {true, true, true};
        assert(success.allSuccess());
        assert(success.countFailures() == 0);
        assert(success.getNackBitmap() == 0);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_frame_type_helpers() {
    TEST("frame type helpers") {
        assert(isControlFrame(FrameType::PROBE));
        assert(isControlFrame(FrameType::ACK));
        assert(isControlFrame(FrameType::NACK));
        assert(isControlFrame(FrameType::BEACON));
        assert(!isControlFrame(FrameType::DATA));

        assert(isDataFrame(FrameType::DATA));
        assert(isDataFrame(FrameType::DATA_START));
        assert(isDataFrame(FrameType::DATA_END));
        assert(!isDataFrame(FrameType::PROBE));

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_large_text_message() {
    TEST("large text message (multiple codewords)") {
        // Create a message that requires multiple codewords
        std::string message;
        for (int i = 0; i < 10; i++) {
            message += "This is line " + std::to_string(i) + " of the message. ";
        }

        auto data = DataFrame::makeData("VA2MVR", "W1AW", 999, message);

        std::cout << "\n  Message size: " << message.size() << " bytes\n";
        std::cout << "  Total codewords: " << (int)data.total_cw << "\n";

        auto serialized = data.serialize();
        std::cout << "  Serialized size: " << serialized.size() << " bytes\n";

        // Split into codewords
        auto codewords = splitIntoCodewords(serialized);
        std::cout << "  Codeword count: " << codewords.size() << "\n";
        assert(codewords.size() == data.total_cw);

        // Reassemble and parse
        auto reassembled = reassembleCodewords(codewords, serialized.size());
        auto parsed = DataFrame::deserialize(reassembled);

        assert(parsed.has_value());
        assert(parsed->payloadAsText() == message);
        std::cout << "  ";

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

// ============================================================================
// LDPC Integration Tests
// ============================================================================

void test_ldpc_control_frame_roundtrip() {
    TEST("LDPC encode/decode control frame (1 codeword)") {
        // Create and serialize a PROBE frame
        auto probe = ControlFrame::makeProbe("VA2MVR", "W1AW");
        auto serialized = probe.serialize();
        assert(serialized.size() == 20);

        // LDPC encode
        auto encoded = encodeFrameWithLDPC(serialized);
        assert(encoded.size() == 1);  // 1 codeword
        assert(encoded[0].size() == LDPC_CODEWORD_BYTES);  // 81 bytes

        std::cout << "\n  Frame: 20 bytes → LDPC: " << encoded[0].size() << " bytes\n";

        // Convert to soft bits (perfect LLRs)
        std::vector<std::vector<float>> soft_bits(1);
        for (uint8_t byte : encoded[0]) {
            for (int b = 7; b >= 0; --b) {
                int bit = (byte >> b) & 1;
                soft_bits[0].push_back(bit ? -5.0f : 5.0f);
            }
        }
        assert(soft_bits[0].size() == LDPC_CODEWORD_BITS);

        // LDPC decode
        auto status = decodeCodewordsWithLDPC(soft_bits);
        assert(status.allSuccess());
        assert(status.decoded.size() == 1);
        assert(status.data[0].size() == 20);

        // Verify data matches
        assert(status.data[0] == serialized);

        // Parse and verify
        auto parsed = ControlFrame::deserialize(status.data[0]);
        assert(parsed.has_value());
        assert(parsed->type == FrameType::PROBE);
        assert(parsed->src_hash == probe.src_hash);
        std::cout << "  ";

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_ldpc_data_frame_roundtrip() {
    TEST("LDPC encode/decode data frame (multiple codewords)") {
        std::string message = "Hello, this is a test message for LDPC multi-codeword!";
        auto data = DataFrame::makeData("VA2MVR", "W1AW", 42, message);
        auto serialized = data.serialize();

        std::cout << "\n  Message: " << message.size() << " bytes\n";
        std::cout << "  Frame: " << serialized.size() << " bytes\n";
        std::cout << "  Expected codewords: " << (int)data.total_cw << "\n";

        // LDPC encode
        auto encoded = encodeFrameWithLDPC(serialized);
        assert(encoded.size() == data.total_cw);
        std::cout << "  Encoded: " << encoded.size() << " codewords × "
                  << encoded[0].size() << " bytes\n";

        // Convert all to soft bits
        std::vector<std::vector<float>> soft_bits(encoded.size());
        for (size_t cw = 0; cw < encoded.size(); cw++) {
            for (uint8_t byte : encoded[cw]) {
                for (int b = 7; b >= 0; --b) {
                    int bit = (byte >> b) & 1;
                    soft_bits[cw].push_back(bit ? -5.0f : 5.0f);
                }
            }
        }

        // LDPC decode
        auto status = decodeCodewordsWithLDPC(soft_bits);
        assert(status.allSuccess());
        assert(status.countFailures() == 0);

        // Parse header from first codeword
        auto header = parseHeader(status.data[0]);
        assert(header.valid);
        assert(!header.is_control);
        assert(header.total_cw == data.total_cw);
        assert(header.payload_len == message.size());
        std::cout << "  Header valid, total_cw=" << (int)header.total_cw
                  << ", payload_len=" << header.payload_len << "\n";

        // Reassemble
        auto reassembled = status.reassemble();
        assert(reassembled.size() == serialized.size());
        assert(reassembled == serialized);

        // Parse and verify message
        auto parsed = DataFrame::deserialize(reassembled);
        assert(parsed.has_value());
        assert(parsed->payloadAsText() == message);
        std::cout << "  ";

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_ldpc_simulated_codeword_loss() {
    TEST("LDPC with simulated codeword loss (per-CW recovery)") {
        std::string message = "This message will have codeword 2 corrupted!";
        auto data = DataFrame::makeData("VA2MVR", "W1AW", 100, message);
        auto serialized = data.serialize();

        // LDPC encode
        auto encoded = encodeFrameWithLDPC(serialized);
        assert(encoded.size() >= 3);  // Need at least 3 codewords for this test

        std::cout << "\n  Codewords: " << encoded.size() << "\n";

        // Convert to soft bits, but corrupt codeword 2
        std::vector<std::vector<float>> soft_bits(encoded.size());
        for (size_t cw = 0; cw < encoded.size(); cw++) {
            for (uint8_t byte : encoded[cw]) {
                for (int b = 7; b >= 0; --b) {
                    int bit = (byte >> b) & 1;
                    float llr = bit ? -5.0f : 5.0f;

                    // Corrupt codeword 2 by flipping LLR signs (simulate bit errors)
                    if (cw == 2) {
                        llr = -llr;  // All bits wrong = LDPC will fail
                    }

                    soft_bits[cw].push_back(llr);
                }
            }
        }

        // LDPC decode - codeword 2 should fail
        auto status = decodeCodewordsWithLDPC(soft_bits);

        assert(!status.allSuccess());
        assert(status.decoded[0] == true);   // CW0 (header) OK
        assert(status.decoded[1] == true);   // CW1 OK
        assert(status.decoded[2] == false);  // CW2 FAILED (corrupted)
        if (encoded.size() > 3) {
            assert(status.decoded[3] == true);  // CW3 OK
        }

        // Check NACK bitmap
        uint32_t bitmap = status.getNackBitmap();
        assert(bitmap == 0b00000100);  // Bit 2 set
        std::cout << "  NACK bitmap: 0x" << std::hex << bitmap << std::dec << "\n";
        std::cout << "  Failed codewords: " << status.countFailures() << "\n";

        // Header should still be parseable
        auto header = parseHeader(status.data[0]);
        assert(header.valid);
        assert(header.total_cw == data.total_cw);
        std::cout << "  Header still valid despite CW2 loss\n  ";

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_header_parsing() {
    TEST("header parsing from first codeword") {
        // Test control frame
        auto probe = ControlFrame::makeProbe("VA2MVR", "W1AW");
        auto ctrl_data = probe.serialize();
        auto ctrl_header = parseHeader(ctrl_data);

        assert(ctrl_header.valid);
        assert(ctrl_header.is_control);
        assert(ctrl_header.type == FrameType::PROBE);
        assert(ctrl_header.total_cw == 1);
        assert(ctrl_header.src_hash == hashCallsign("VA2MVR"));
        assert(ctrl_header.dst_hash == hashCallsign("W1AW"));

        // Test data frame
        auto data = DataFrame::makeData("W1AW", "VA2MVR", 1234, "Test payload data");
        auto data_serialized = data.serialize();
        auto chunks = splitIntoCodewords(data_serialized);
        auto data_header = parseHeader(chunks[0]);

        assert(data_header.valid);
        assert(!data_header.is_control);
        assert(data_header.type == FrameType::DATA);
        assert(data_header.total_cw == data.total_cw);
        assert(data_header.payload_len == 17);  // "Test payload data"
        assert(data_header.seq == 1234);

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

void test_full_recovery_cycle() {
    TEST("full NACK→retransmit→merge recovery cycle") {
        std::string message = "This message will be recovered after NACK retransmit!";
        auto data = DataFrame::makeData("VA2MVR", "W1AW", 500, message);
        auto serialized = data.serialize();

        std::cout << "\n  Original message: " << message.size() << " bytes\n";

        // === TX SIDE: LDPC encode all codewords ===
        auto encoded = encodeFrameWithLDPC(serialized);
        std::cout << "  TX encoded: " << encoded.size() << " codewords\n";
        assert(encoded.size() >= 4);

        // === RX SIDE: Initial receive with CW2 corrupted ===
        std::vector<std::vector<float>> rx_soft_bits(encoded.size());
        for (size_t cw = 0; cw < encoded.size(); cw++) {
            for (uint8_t byte : encoded[cw]) {
                for (int b = 7; b >= 0; --b) {
                    int bit = (byte >> b) & 1;
                    float llr = bit ? -5.0f : 5.0f;
                    // Corrupt CW2
                    if (cw == 2) llr = -llr;
                    rx_soft_bits[cw].push_back(llr);
                }
            }
        }

        auto rx_status = decodeCodewordsWithLDPC(rx_soft_bits);
        assert(!rx_status.allSuccess());
        assert(rx_status.decoded[2] == false);
        std::cout << "  RX initial: CW2 failed (as expected)\n";

        // === RX SIDE: Generate NACK ===
        uint32_t nack_bitmap = rx_status.getNackBitmap();
        assert(nack_bitmap == 0x04);  // Bit 2 set
        auto nack = ControlFrame::makeNack("W1AW", "VA2MVR", 500, nack_bitmap);
        auto nack_bytes = nack.serialize();
        std::cout << "  RX sends NACK with bitmap 0x" << std::hex << nack_bitmap << std::dec << "\n";

        // === TX SIDE: Receive NACK, retransmit only CW2 ===
        auto nack_parsed = ControlFrame::deserialize(nack_bytes);
        assert(nack_parsed.has_value());
        assert(nack_parsed->type == FrameType::NACK);

        NackPayload np = NackPayload::decode(nack_parsed->payload);
        assert(np.frame_seq == 500);
        assert(np.isFailed(2));
        std::cout << "  TX received NACK, retransmitting CW2\n";

        // TX retransmits only the failed codeword(s)
        // In practice, this would be sent over the air
        Bytes retransmit_cw2 = encoded[2];

        // === RX SIDE: Receive retransmission, decode and merge ===
        // Convert retransmitted CW2 to soft bits (this time uncorrupted)
        std::vector<float> retx_soft_bits;
        for (uint8_t byte : retransmit_cw2) {
            for (int b = 7; b >= 0; --b) {
                int bit = (byte >> b) & 1;
                retx_soft_bits.push_back(bit ? -5.0f : 5.0f);  // Clean this time
            }
        }

        auto [retx_success, retx_data] = decodeSingleCodeword(retx_soft_bits);
        assert(retx_success);
        std::cout << "  RX decoded retransmitted CW2 successfully\n";

        // Merge into the partial frame
        bool merged = rx_status.mergeCodeword(2, retx_data);
        assert(merged);
        assert(rx_status.allSuccess());
        std::cout << "  RX merged CW2, all codewords now decoded\n";

        // === RX SIDE: Reassemble complete frame ===
        auto reassembled = rx_status.reassemble();
        assert(reassembled.size() == serialized.size());
        assert(reassembled == serialized);

        auto parsed = DataFrame::deserialize(reassembled);
        assert(parsed.has_value());
        assert(parsed->payloadAsText() == message);
        std::cout << "  RX recovered message: \"" << parsed->payloadAsText() << "\"\n  ";

        PASS();
    } catch (const std::exception& e) {
        FAIL(e.what());
    }
}

int main() {
    std::cout << "=== ULTRA Protocol v2 Frame Tests ===\n\n";

    test_callsign_hashing();
    test_control_frame_size();
    test_control_frame_roundtrip();
    test_control_frame_crc();
    test_control_frame_magic();
    test_data_frame_codeword_count();
    test_data_frame_roundtrip();
    test_data_frame_crc();
    test_split_into_codewords();
    test_reassemble_codewords();
    test_nack_payload();
    test_nack_frame();
    test_codeword_status();
    test_frame_type_helpers();
    test_large_text_message();

    // LDPC integration tests
    std::cout << "\n=== LDPC Integration Tests ===\n\n";
    test_ldpc_control_frame_roundtrip();
    test_ldpc_data_frame_roundtrip();
    test_ldpc_simulated_codeword_loss();
    test_header_parsing();
    test_full_recovery_cycle();

    std::cout << "\n=== Results ===\n";
    std::cout << "Passed: " << tests_passed << "\n";
    std::cout << "Failed: " << tests_failed << "\n";

    return tests_failed > 0 ? 1 : 0;
}
