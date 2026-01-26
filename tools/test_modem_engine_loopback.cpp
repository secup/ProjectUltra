/**
 * Minimal ModemEngine loopback test
 * Tests if ModemEngine TX/RX works correctly in isolation
 * without CLI simulator's streaming complexity
 */
#include <iostream>
#include <iomanip>
#include <cstring>
#include <random>
#include <thread>
#include <chrono>
#include "gui/modem/modem_engine.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/protocol_engine.hpp"

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

void printHex(const char* label, const Bytes& data, size_t max_bytes = 20) {
    std::cout << label << " (" << data.size() << " bytes): ";
    for (size_t i = 0; i < std::min(max_bytes, data.size()); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data[i] << " ";
    }
    if (data.size() > max_bytes) std::cout << "...";
    std::cout << std::dec << "\n";
}

// Build a DATA frame like the protocol engine does
Bytes buildDataFrame(const std::string& msg, uint16_t seq) {
    // Frame layout matches v2::DataFrame
    size_t payload_size = msg.size();
    size_t frame_size = v2::DataFrame::HEADER_SIZE + payload_size + 2;  // header + payload + CRC

    Bytes frame(frame_size);
    frame[0] = 0x55;  // Magic
    frame[1] = 0x4C;
    frame[2] = static_cast<uint8_t>(v2::FrameType::DATA);  // 0x30
    frame[3] = 0x00;  // flags
    frame[4] = (seq >> 8) & 0xFF;
    frame[5] = seq & 0xFF;

    // src_hash (3 bytes)
    uint32_t src_hash = v2::hashCallsign("ALPHA");
    frame[6] = (src_hash >> 16) & 0xFF;
    frame[7] = (src_hash >> 8) & 0xFF;
    frame[8] = src_hash & 0xFF;

    // dst_hash (3 bytes)
    uint32_t dst_hash = v2::hashCallsign("BRAVO");
    frame[9] = (dst_hash >> 16) & 0xFF;
    frame[10] = (dst_hash >> 8) & 0xFF;
    frame[11] = dst_hash & 0xFF;

    // total_cw (calculated for R2/3 adaptive rate)
    // CW0: 20 bytes, CW1: 54 bytes for R2/3
    size_t cw0_bytes = 20;  // R1/4
    size_t cw1_bytes = 54;  // R2/3
    uint8_t total_cw = 1;  // CW0
    if (frame_size > cw0_bytes) {
        total_cw += (frame_size - cw0_bytes + cw1_bytes - 2 - 1) / (cw1_bytes - 2);
    }
    frame[12] = total_cw;

    // payload_len (2 bytes)
    frame[13] = (payload_size >> 8) & 0xFF;
    frame[14] = payload_size & 0xFF;

    // header CRC (bytes 15-16, CRC of bytes 0-14)
    uint16_t hcrc = v2::ControlFrame::calculateCRC(frame.data(), 15);
    frame[15] = (hcrc >> 8) & 0xFF;
    frame[16] = hcrc & 0xFF;

    // Payload
    std::memcpy(frame.data() + v2::DataFrame::HEADER_SIZE, msg.data(), msg.size());

    // Frame CRC (last 2 bytes)
    uint16_t fcrc = v2::ControlFrame::calculateCRC(frame.data(), frame_size - 2);
    frame[frame_size - 2] = (fcrc >> 8) & 0xFF;
    frame[frame_size - 1] = fcrc & 0xFF;

    return frame;
}

int main() {
    std::cout << "=== ModemEngine Loopback Test (10 messages) ===\n\n";

    // Create TX modem engine
    ModemEngine tx_modem;
    tx_modem.setLogPrefix("TX");
    tx_modem.setConnected(true);
    tx_modem.setHandshakeComplete(true);
    tx_modem.setDataMode(Modulation::DQPSK, CodeRate::R2_3);
    tx_modem.setWaveformMode(protocol::WaveformMode::OFDM_COX);

    // Track received messages
    std::vector<Bytes> received_frames;

    // Test 10 messages
    std::string base_msg = "Test message number ";

    std::mt19937 rng(42);
    float snr_db = 40.0f;  // Very high SNR - should have NO errors

    int success_count = 0;
    int fail_count = 0;

    for (int msg_idx = 0; msg_idx < 10; msg_idx++) {
        std::string msg = base_msg + std::to_string(msg_idx);
        std::cout << "\n=== Message " << msg_idx << ": \"" << msg << "\" (" << msg.size() << " bytes) ===\n";

        // Use different RNG seed per message to get different noise
        rng.seed(42 + msg_idx);

        // Create fresh RX modem for each message to eliminate state issues
        auto rx_modem = std::make_unique<ModemEngine>();
        rx_modem->setLogPrefix("RX");
        rx_modem->setConnected(true);
        rx_modem->setHandshakeComplete(true);
        rx_modem->setDataMode(Modulation::DQPSK, CodeRate::R2_3);
        rx_modem->setWaveformMode(protocol::WaveformMode::OFDM_COX);
        rx_modem->setRawDataCallback([&](const Bytes& data) {
            received_frames.push_back(data);
        });

        // Build frame
        Bytes frame = buildDataFrame(msg, msg_idx);
        std::cout << "  Frame size: " << frame.size() << " bytes (header=17, payload=" << msg.size() << ", crc=2)\n";

        // Transmit
        auto samples = tx_modem.transmit(frame);

        // Add noise
        float signal_rms = 0.1f;
        float snr_linear = std::pow(10.0f, snr_db / 10.0f);
        float noise_stddev = signal_rms / std::sqrt(snr_linear);
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        // PTT noise ONLY - no channel noise on signal
        size_t noise_samples = 12000;  // ~250ms
        std::vector<float> ptt_noise(noise_samples);
        for (size_t i = 0; i < noise_samples; ++i) {
            ptt_noise[i] = noise_dist(rng);
        }

        // Full signal = PTT noise + signal + channel noise
        std::vector<float> full_signal;
        full_signal.reserve(ptt_noise.size() + samples.size());
        full_signal.insert(full_signal.end(), ptt_noise.begin(), ptt_noise.end());
        full_signal.insert(full_signal.end(), samples.begin(), samples.end());

        // NO channel noise - completely clean signal
        // for (size_t i = noise_samples; i < full_signal.size(); ++i) {
        //     full_signal[i] += noise_dist(rng);
        // }

        // Feed all samples - modem threads process automatically
        received_frames.clear();
        rx_modem->feedAudio(full_signal);
        // Give threads time to process
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        if (received_frames.empty()) {
            std::cout << "  RESULT: ✗ FAILED (no frame received)\n";
            fail_count++;
        } else {
            // Extract payload
            if (received_frames[0].size() >= v2::DataFrame::HEADER_SIZE + 2) {
                size_t payload_size = received_frames[0].size() - v2::DataFrame::HEADER_SIZE - 2;
                std::string payload(received_frames[0].begin() + v2::DataFrame::HEADER_SIZE,
                                   received_frames[0].begin() + v2::DataFrame::HEADER_SIZE + payload_size);
                // Clean non-printable
                std::string clean_payload;
                for (char c : payload) {
                    if (c >= 32 && c <= 126) clean_payload += c;
                }

                bool match = (clean_payload == msg);
                if (match) {
                    std::cout << "  RESULT: ✓ SUCCESS\n";
                    success_count++;
                } else {
                    std::cout << "  RESULT: ✗ MISMATCH\n";
                    std::cout << "    Expected: \"" << msg << "\"\n";
                    std::cout << "    Got:      \"" << clean_payload << "\"\n";
                    fail_count++;
                }
            } else {
                std::cout << "  RESULT: ✗ FAILED (frame too small)\n";
                fail_count++;
            }
        }
    }

    std::cout << "\n=== SUMMARY ===\n";
    std::cout << "Success: " << success_count << "/10\n";
    std::cout << "Failed:  " << fail_count << "/10\n";

    return fail_count > 0 ? 1 : 0;
}
