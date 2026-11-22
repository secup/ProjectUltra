#include "gui/modem/streaming_frame_policy.hpp"
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/connection_policy.hpp"
#include "ultra/logging.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

namespace {

std::vector<float> withSilence(const std::vector<float>& frame) {
    std::vector<float> audio;
    audio.resize(48000, 0.0f);  // 1s lead-in lets chirp search establish noise floor.
    audio.insert(audio.end(), frame.begin(), frame.end());
    audio.resize(audio.size() + 96000, 0.0f);  // trailing room for final decode.
    return audio;
}

bool feedInChunks(StreamingDecoder& decoder, const std::vector<float>& audio) {
    constexpr size_t kChunk = 4800;  // 100 ms audio chunks.
    for (size_t pos = 0; pos < audio.size(); pos += kChunk) {
        const size_t len = std::min(kChunk, audio.size() - pos);
        decoder.feedAudio(audio.data() + pos, len);
        decoder.processBuffer();
    }
    return decoder.hasFrame();
}

bool runLoopback(const char* name,
                 const MultiCarrierDPSKConfig& mc_config,
                 Modulation modulation,
                 const Bytes& serialized,
                 v2::FrameType expected_type,
                 bool awgn_channel = false,
                 float channel_snr_db = 20.0f) {
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_config);
    encoder.setDataMode(modulation, CodeRate::R1_4);

    auto samples = encoder.encodeFrame(serialized);
    if (samples.empty()) {
        std::cout << "FAIL: " << name << " encoder produced no samples\n";
        return false;
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_config);
    // #72: handshake-negotiation frames (CONNECT/CONNECT_ACK) ride the fixed DBPSK
    // control profile — like OFDM's QPSK control profile — so they are peer-decodable
    // regardless of the negotiated DATA constellation. In production the receiver
    // decodes them while still at the default DBPSK control state (the data mode is
    // applied only AFTER the frame decodes). Mirror that: decode CONNECT at DBPSK.
    const bool handshake_frame = (expected_type == v2::FrameType::CONNECT ||
                                  expected_type == v2::FrameType::CONNECT_ACK);
    decoder.setDataMode(handshake_frame ? Modulation::DBPSK : modulation,
                        CodeRate::R1_4);

    bool callback_seen = false;
    DecodeResult decoded;
    decoder.setFrameCallback([&](const DecodeResult& result) {
        if (result.success) {
            callback_seen = true;
            decoded = result;
        }
    });

    auto audio = withSilence(samples);
    if (awgn_channel) {
        ultra::ota_channel_core::SimulatedChannel channel;
        channel.setSeed(0x5a17u);
        channel.configure(channel_snr_db, ultra::ota_channel_core::ChannelType::AWGN);
        channel.transmitFromA(audio);
        audio = channel.receiveForB(audio.size());
    }
    feedInChunks(decoder, audio);

    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success) {
            callback_seen = true;
            decoded = result;
            break;
        }
    }

    if (!callback_seen) {
        std::cout << "FAIL: " << name << " streaming decoder did not recover MC-DPSK frame\n";
        return false;
    }
    if (decoded.frame_type != expected_type) {
        std::cout << "FAIL: " << name << " decoded frame type mismatch\n";
        return false;
    }

    if (expected_type == v2::FrameType::DATA && decoded.frame_data != serialized) {
        std::cout << "FAIL: " << name << " frame payload mismatch\n";
        return false;
    }
    if (expected_type == v2::FrameType::CONNECT &&
        !v2::ConnectFrame::deserialize(decoded.frame_data)) {
        std::cout << "FAIL: " << name << " CONNECT frame did not parse\n";
        return false;
    }
    if (decoded.snr_source != SNRSource::MCDPSK_IN_BAND ||
        !std::isfinite(decoded.snr_db) || decoded.snr_db <= 1.0f) {
        std::cout << "FAIL: " << name << " MC-DPSK connected SNR invalid: "
                  << decoded.snr_db << " dB ("
                  << snrSourceToString(decoded.snr_source) << ")\n";
        return false;
    }
    std::cout << name << " MC-DPSK connected SNR="
              << decoded.snr_db << " dB ("
              << snrSourceToString(decoded.snr_source) << ")\n";

    return true;
}

bool runLowAmplitudePing(const char* name,
                         const MultiCarrierDPSKConfig& mc_config,
                         float amplitude_scale) {
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_config);

    auto samples = encoder.encodePing();
    if (samples.empty()) {
        std::cout << "FAIL: " << name << " encoder produced no PING samples\n";
        return false;
    }
    for (auto& s : samples) {
        s *= amplitude_scale;
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, false);
    decoder.setMCDPSKConfig(mc_config);

    bool ping_callback_seen = false;
    decoder.setPingCallback([&](float, float) {
        ping_callback_seen = true;
    });

    auto audio = withSilence(samples);
    feedInChunks(decoder, audio);

    bool ping_frame_seen = false;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && result.is_ping) {
            ping_frame_seen = true;
            break;
        }
    }

    if (!ping_callback_seen || !ping_frame_seen) {
        std::cout << "FAIL: " << name << " low-amplitude PING was not detected\n";
        return false;
    }

    return true;
}

// MC-DPSK rate change (2026-11-11): the sender changes constellation between bursts and the
// receiver may still be configured for the other one (a lost command, or its own data mode). The
// burst's first frame is re-demodulated at the other constellation and the rest of the burst
// follows it. Production geometry: 8 carriers, 1024 samples/symbol, R1/4.
bool runConstellationChangeBurst(Modulation tx_mod, Modulation rx_configured) {
    const char* name = (tx_mod == Modulation::DQPSK) ? "DQPSK burst at a DBPSK receiver"
                                                     : "DBPSK burst at a DQPSK receiver";
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    encoder.setDataMode(tx_mod, CodeRate::R1_4);

    std::vector<Bytes> serialized_frames;
    for (uint16_t i = 0; i < 3; ++i) {
        Bytes payload(50);
        for (size_t j = 0; j < payload.size(); ++j) {
            payload[j] = static_cast<uint8_t>(0x41 + i * 13 + j);
        }
        auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(60 + i),
                                             payload, CodeRate::R1_4);
        frame.flags = static_cast<uint8_t>(
            v2::Flags::VERSION_V2 | (i + 1 < 3 ? v2::Flags::MORE_FRAG : v2::Flags::NONE));
        serialized_frames.push_back(frame.serialize());
    }
    auto samples = encoder.encodeBurstLight(serialized_frames);
    if (samples.empty()) {
        std::cout << "FAIL: " << name << ": encoder produced no samples\n";
        return false;
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    decoder.setDataMode(rx_configured, CodeRate::R1_4);

    feedInChunks(decoder, withSilence(samples));
    size_t decoded = 0;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && v2::isDataFrame(result.frame_type)) {
            ++decoded;
        }
    }
    if (decoded != serialized_frames.size()) {
        std::cout << "FAIL: " << name << ": decoded " << decoded << " of "
                  << serialized_frames.size() << " frames\n";
        return false;
    }
    if (decoder.lastMcDpskRxModulation() != tx_mod) {
        std::cout << "FAIL: " << name << ": recorded constellation "
                  << modulationToString(decoder.lastMcDpskRxModulation()) << "\n";
        return false;
    }
    std::cout << name << ": all " << decoded << " frames decoded\n";
    return true;
}

// MC-DPSK rate change: a frame the receiver cannot decode is reported, with the burst's
// constellation, so the receiver's rate controller sees the loss even when nothing is delivered.
bool runLostFrameReport() {
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    encoder.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
    std::vector<Bytes> serialized_frames;
    for (uint16_t i = 0; i < 3; ++i) {
        Bytes payload(50, static_cast<uint8_t>(0x20 + i));
        auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(80 + i),
                                             payload, CodeRate::R1_4);
        frame.flags = static_cast<uint8_t>(
            v2::Flags::VERSION_V2 | (i + 1 < 3 ? v2::Flags::MORE_FRAG : v2::Flags::NONE));
        serialized_frames.push_back(frame.serialize());
    }
    auto samples = encoder.encodeBurstLight(serialized_frames);
    // Replace the last 30 % of the burst (inside the third frame) with noise of the same power:
    // energy where a frame is due, nothing decodable.
    double power = 0.0;
    for (float v : samples) power += static_cast<double>(v) * v;
    const float rms = static_cast<float>(std::sqrt(power / static_cast<double>(samples.size())));
    uint32_t lcg = 12345u;
    for (size_t i = samples.size() * 7 / 10; i < samples.size(); ++i) {
        lcg = lcg * 1664525u + 1013904223u;
        const float u = static_cast<float>((lcg >> 8) & 0xFFFF) / 65535.0f - 0.5f;
        samples[i] = u * rms * 3.46f;  // uniform, same RMS
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    decoder.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
    std::vector<int> lost;
    decoder.setMcDpskFrameLostCallback([&](int modulation) { lost.push_back(modulation); });
    feedInChunks(decoder, withSilence(samples));
    size_t decoded = 0;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && v2::isDataFrame(result.frame_type)) ++decoded;
    }
    if (lost.empty() || decoded >= serialized_frames.size()) {
        std::cout << "FAIL: lost-frame report: " << lost.size() << " report(s), " << decoded
                  << " frame(s) decoded\n";
        return false;
    }
    for (int m : lost) {
        if (m != static_cast<int>(Modulation::DBPSK)) {
            std::cout << "FAIL: lost-frame report carried constellation " << m << "\n";
            return false;
        }
    }
    std::cout << "lost-frame report: " << decoded << " decoded, " << lost.size()
              << " loss report(s) at DBPSK\n";
    return true;
}

// Compact MC-DPSK DATA frames (protocol/mcdpsk_frame.hpp): a 3-frame burst of single-codeword
// frames; optionally the middle frame replaced by noise (frames 1 and 3 must still decode: the
// frame length is fixed, so a failure no longer ends the burst); optionally sent at DQPSK to a
// receiver configured for DBPSK.
bool runCompactBurst(const char* name, Modulation tx_mod, bool corrupt_middle,
                     Modulation rx_mod = Modulation::DBPSK, uint16_t n_frames = 3) {
    const uint32_t alpha = v2::hashCallsign("ALPHA");
    const uint32_t bravo = v2::hashCallsign("BRAVO");
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    encoder.setDataMode(tx_mod, CodeRate::R1_4);
    encoder.setFixedFrameCodewords(3);
    encoder.setMcDpskCompactFrames(true);
    std::vector<Bytes> serialized_frames;
    for (uint16_t i = 0; i < n_frames; ++i) {
        Bytes payload(52);
        for (size_t j = 0; j < payload.size(); ++j) payload[j] = static_cast<uint8_t>(0x11 * (i + 1) + j);
        auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(300 + i),
                                             payload, CodeRate::R1_4);
        frame.flags = static_cast<uint8_t>(v2::Flags::VERSION_V2 | v2::Flags::MORE_FRAG);
        serialized_frames.push_back(frame.serialize());
    }
    auto samples = encoder.encodeBurstLight(serialized_frames);
    // 3 x 648 air bits per frame: on 8 carriers (DBPSK, DQPSK) or 4 bits per symbol time (16-FSK).
    const size_t bits_per_symbol_time =
        protocol::connection_policy::mcDpskBitsPerSymbolTime(tx_mod, 8);
    const size_t frame_samples =
        ((3 * 648 + bits_per_symbol_time - 1) / bits_per_symbol_time) * 1024;
    if (samples.size() <= n_frames * frame_samples) {
        std::cout << "FAIL: " << name << ": burst shorter than three frames\n";
        return false;
    }
    if (corrupt_middle) {
        const size_t start = samples.size() - 2 * frame_samples;
        double power = 0.0;
        for (size_t i = start; i < start + frame_samples; ++i) power += samples[i] * samples[i];
        const float rms = static_cast<float>(std::sqrt(power / static_cast<double>(frame_samples)));
        uint32_t lcg = 777u;
        for (size_t i = start; i < start + frame_samples; ++i) {
            lcg = lcg * 1664525u + 1013904223u;
            samples[i] = (static_cast<float>((lcg >> 8) & 0xFFFF) / 65535.0f - 0.5f) * rms * 3.46f;
        }
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    decoder.setDataMode(rx_mod, CodeRate::R1_4);
    decoder.setFixedFrameCodewords(3);
    decoder.setMcDpskSessionAddresses(bravo, alpha);
    int lost = 0;
    decoder.setMcDpskFrameLostCallback([&](int) { ++lost; });
    // Listen-before-ACK geometry: when each frame reaches the protocol, the decoder publishes
    // the burst's remaining air time (the next frame's end) until the window's last frame.
    std::vector<uint64_t> air_remaining_at_frame;
    decoder.setFrameCallback([&](const DecodeResult& r) {
        if (r.success && v2::isDataFrame(r.frame_type)) {
            air_remaining_at_frame.push_back(decoder.burstAirSamplesRemaining());
        }
    });
    feedInChunks(decoder, withSilence(samples));
    if (!corrupt_middle && n_frames == 3 &&
        (air_remaining_at_frame.size() != 3 || air_remaining_at_frame[0] == 0 ||
         air_remaining_at_frame[1] == 0 || air_remaining_at_frame[2] != 0)) {
        std::cout << "FAIL: " << name << ": burst air end not published per frame (";
        for (auto v : air_remaining_at_frame) std::cout << v << " ";
        std::cout << ")\n";
        return false;
    }
    std::vector<uint16_t> seqs;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (!result.success || !v2::isDataFrame(result.frame_type)) continue;
        auto frame = v2::DataFrame::deserialize(result.frame_data);
        if (!frame) continue;
        // Each frame reports the SNR the session's rate decisions use (as a v2 frame does).
        if (result.snr_source != SNRSource::MCDPSK_IN_BAND || !std::isfinite(result.snr_db)) {
            std::cout << "FAIL: " << name << ": frame " << frame->seq << " reports SNR source "
                      << snrSourceToString(result.snr_source) << "\n";
            return false;
        }
        const auto expect = v2::DataFrame::deserialize(serialized_frames[frame->seq - 300]);
        if (frame->seq < 300 || frame->seq >= 300 + n_frames || !expect || frame->payload != expect->payload ||
            frame->src_hash != alpha || frame->dst_hash != bravo) {
            std::cout << "FAIL: " << name << ": frame " << frame->seq << " does not match\n";
            return false;
        }
        seqs.push_back(frame->seq);
    }
    std::vector<uint16_t> want;
    for (uint16_t i = 0; i < n_frames; ++i) {
        if (!(corrupt_middle && i == 1)) want.push_back(static_cast<uint16_t>(300 + i));
    }
    if (seqs != want || (corrupt_middle && lost != 1) || (!corrupt_middle && lost != 0)) {
        std::cout << "FAIL: " << name << ": decoded " << seqs.size() << " frame(s), " << lost
                  << " loss report(s)\n";
        return false;
    }
    std::cout << name << ": " << seqs.size() << " frames decoded, " << lost << " loss\n";
    return true;
}

// Per-frame physical SNR (handoff §2, per-frame semantics): every frame of a compact burst reports
// the power ratio of ITS OWN span. The middle frame goes out 6 dB down; a receiver that re-publishes
// the first frame's training snapshot for the continuations (the 2026-11-12 defect: "physical
// 4.7 dB x3" on a 20 dB channel, GUI step-up seed 43) reads all three alike. Also through the
// first-frame constellation retry (a DQPSK burst at a DBPSK receiver), whose re-demodulated first
// frame must keep its own reading.
bool runCompactBurstPhysicalPerFrame(Modulation tx_mod, Modulation rx_configured) {
    const char* name = tx_mod == rx_configured ? "per-frame physical SNR"
                                               : "per-frame physical SNR through the retry";
    const uint32_t alpha = v2::hashCallsign("ALPHA");
    const uint32_t bravo = v2::hashCallsign("BRAVO");
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    encoder.setDataMode(tx_mod, CodeRate::R1_4);
    encoder.setFixedFrameCodewords(3);
    encoder.setMcDpskCompactFrames(true);
    std::vector<Bytes> frames;
    for (uint16_t i = 0; i < 3; ++i) {
        auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(700 + i),
                                             Bytes(48, static_cast<uint8_t>(0x30 + i)), CodeRate::R1_4);
        frame.flags = static_cast<uint8_t>(v2::Flags::VERSION_V2 | v2::Flags::MORE_FRAG);
        frames.push_back(frame.serialize());
    }
    auto samples = encoder.encodeBurstLight(frames);
    const int bits = tx_mod == Modulation::DBPSK ? 1 : 2;
    const size_t frame_samples =
        static_cast<size_t>(((3 * 648 + 8 * bits - 1) / (8 * bits)) * 1024);
    if (samples.size() <= 3 * frame_samples) {
        std::cout << "FAIL: " << name << ": burst shorter than three frames\n";
        return false;
    }
    for (size_t i = samples.size() - 2 * frame_samples; i < samples.size() - frame_samples; ++i) {
        samples[i] *= 0.5f;  // the middle frame 6.02 dB down
    }
    auto audio = withSilence(samples);
    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(0x2718u);
    channel.configure(20.0f, ultra::ota_channel_core::ChannelType::AWGN);
    channel.transmitFromA(audio);
    audio = channel.receiveForB(audio.size());

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    decoder.setDataMode(rx_configured, CodeRate::R1_4);
    decoder.setFixedFrameCodewords(3);
    decoder.setMcDpskSessionAddresses(bravo, alpha);
    feedInChunks(decoder, audio);
    std::vector<float> physical(3, NAN);
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (!result.success || !v2::isDataFrame(result.frame_type)) continue;
        auto frame = v2::DataFrame::deserialize(result.frame_data);
        if (!frame || frame->seq < 700 || frame->seq > 702) continue;
        if (result.has_physical_snr_db) physical[frame->seq - 700] = result.physical_snr_db;
    }
    std::cout << name << ": physical " << physical[0] << " / " << physical[1] << " / "
              << physical[2] << " dB\n";
    for (float p : physical) {
        if (!std::isfinite(p)) {
            std::cout << "FAIL: " << name << ": a decoded frame carries no physical SNR\n";
            return false;
        }
    }
    // Frames 1 and 3 share the channel; frame 2 is 6 dB down (the noise reference is the burst's).
    if (std::fabs(physical[0] - physical[2]) > 2.0f || physical[2] - physical[1] < 4.0f ||
        physical[2] - physical[1] > 8.0f || physical[0] < 15.0f) {
        std::cout << "FAIL: " << name << ": readings are not each frame's own\n";
        return false;
    }
    return true;
}

// MC-DPSK rate change: the up-step's evidence. A compact DBPSK burst through AWGN; the receiver
// re-decodes the first frame with 4x the in-band noise. At 20 dB that still decodes (DQPSK would);
// at -4 dB DBPSK decodes but the 4x-noise copy does not (DQPSK would not).
bool runMarginTest(float snr_db, bool expect_margin) {
    const uint32_t alpha = v2::hashCallsign("ALPHA");
    const uint32_t bravo = v2::hashCallsign("BRAVO");
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    encoder.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
    encoder.setFixedFrameCodewords(3);
    encoder.setMcDpskCompactFrames(true);
    std::vector<Bytes> frames;
    for (uint16_t i = 0; i < 2; ++i) {
        auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", static_cast<uint16_t>(500 + i),
                                             Bytes(40, static_cast<uint8_t>(0x5A + i)), CodeRate::R1_4);
        frame.flags = static_cast<uint8_t>(v2::Flags::VERSION_V2 | v2::Flags::MORE_FRAG);
        frames.push_back(frame.serialize());
    }
    auto audio = withSilence(encoder.encodeBurstLight(frames));
    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(0x3141u);
    channel.configure(snr_db, ultra::ota_channel_core::ChannelType::AWGN);
    channel.transmitFromA(audio);
    audio = channel.receiveForB(audio.size());

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_dpsk_presets::robust());
    decoder.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
    decoder.setFixedFrameCodewords(3);
    decoder.setMcDpskSessionAddresses(bravo, alpha);
    std::vector<int> margins;
    decoder.setMcDpskMarginCallback([&](bool m) { margins.push_back(m ? 1 : 0); });
    feedInChunks(decoder, audio);
    size_t decoded = 0;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && v2::isDataFrame(result.frame_type)) ++decoded;
    }
    if (decoded == 0 || margins.size() != 1 || (margins[0] == 1) != expect_margin) {
        std::cout << "FAIL: margin test at " << snr_db << " dB: " << decoded << " decoded, "
                  << margins.size() << " margin report(s), first="
                  << (margins.empty() ? -1 : margins[0]) << "\n";
        return false;
    }
    std::cout << "margin test at " << snr_db << " dB: " << decoded << " decoded, margin "
              << (margins[0] ? "yes" : "no") << "\n";
    return true;
}

bool runContinuousBurstLoopback() {
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_dpsk_presets::level8());
    encoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);

    const std::vector<size_t> payload_sizes{32, 50, 50};
    std::vector<Bytes> serialized_frames;
    for (uint16_t i = 0; i < 3; ++i) {
        Bytes payload(payload_sizes[i]);
        for (size_t j = 0; j < payload.size(); ++j) {
            payload[j] = static_cast<uint8_t>(0x30 + i * 17 + j);
        }
        auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO",
                                             static_cast<uint16_t>(40 + i),
                                             payload, CodeRate::R1_4);
        frame.flags = static_cast<uint8_t>(
            v2::Flags::VERSION_V2 |
            (i + 1 < 3 ? v2::Flags::MORE_FRAG : v2::Flags::NONE));
        serialized_frames.push_back(frame.serialize());
    }

    auto samples = encoder.encodeBurstLight(serialized_frames);
    if (samples.empty()) {
        std::cout << "FAIL: continuous burst encoder produced no samples\n";
        return false;
    }

    StreamingDecoder decoder;
    decoder.setLogPrefix("TEST");
    decoder.setMode(protocol::WaveformMode::MC_DPSK, true);
    decoder.setMCDPSKConfig(mc_dpsk_presets::level8());
    decoder.setDataMode(Modulation::DQPSK, CodeRate::R1_4);

    auto audio = withSilence(samples);
    feedInChunks(decoder, audio);

    std::vector<Bytes> decoded_frames;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && v2::isDataFrame(result.frame_type)) {
            decoded_frames.push_back(result.frame_data);
        }
    }

    if (decoded_frames.size() != serialized_frames.size()) {
        std::cout << "FAIL: continuous burst decoded " << decoded_frames.size()
                  << " frames, expected " << serialized_frames.size() << "\n";
        return false;
    }

    // The wire copy of an MC-DPSK turn marks its LAST frame PHYSICAL_BURST_END (the ACK of
    // that frame must carry completed-turn provenance — BUG-MCDPSK-ACK-DROPPED); the ARQ's
    // logical frames are unchanged.
    const auto expected_wire = streaming_frame_policy::preparePhysicalBurstFrames(
        serialized_frames, /*stamp_physical_end=*/true);
    for (size_t i = 0; i < serialized_frames.size(); ++i) {
        if (decoded_frames[i] != expected_wire[i]) {
            std::cout << "FAIL: continuous burst frame " << i << " mismatch\n";
            return false;
        }
        const bool marked = (decoded_frames[i][3] & v2::Flags::PHYSICAL_BURST_END) != 0;
        if (marked != (i + 1 == serialized_frames.size())) {
            std::cout << "FAIL: continuous burst frame " << i
                      << (marked ? " carries" : " lacks") << " the turn-end marker\n";
            return false;
        }
    }

    return true;
}

bool runConnectedOFDMHalfOpenConnectFallback() {
    const auto mc_config = mc_dpsk_presets::robust_mid();
    auto connect_frame = v2::ConnectFrame::makeConnect(
        "ALPHA", "BRAVO",
        static_cast<uint8_t>(protocol::WaveformMode::OFDM_CHIRP),
        static_cast<uint8_t>(Modulation::QPSK),
        static_cast<uint8_t>(CodeRate::R1_2));

    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::MC_DPSK);
    encoder.setMCDPSKConfig(mc_config);
    encoder.setDataMode(Modulation::DBPSK, CodeRate::R1_4);
    const auto samples = encoder.encodeFrame(connect_frame.serialize());
    if (samples.empty()) {
        std::cout << "FAIL: half-open fallback encoder produced no samples\n";
        return false;
    }

    // Reproduce the rig's responder state: CONNECT decoded, CONNECT_ACK sent,
    // primary RX already lifted to OFDM, but no authoritative peer frame has
    // confirmed that the ACK arrived.  The initiator therefore repeats CONNECT
    // on the fixed MC-DPSK handshake PHY.
    StreamingDecoder decoder;
    decoder.setLogPrefix("HALFOPEN");
    decoder.setMCDPSKConfig(mc_config);
    decoder.setMode(protocol::WaveformMode::OFDM_CHIRP, true);
    ModemConfig ofdm = presets::balanced();
    ofdm.modulation = Modulation::QPSK;
    ofdm.code_rate = CodeRate::R1_2;
    ofdm.use_pilots = true;
    decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP,
                                 ofdm, Modulation::QPSK, CodeRate::R1_2);
    decoder.applyPendingConfigForTesting();
    decoder.setHandshakeConfirmationPending(true);
    decoder.expectFullOFDMAnchorOnce();

    int connect_callbacks = 0;
    bool physical_turn_complete = false;
    SNRSource routed_snr = SNRSource::SYNC_QUALITY;
    decoder.setFrameCallback([&](const DecodeResult& result) {
        if (result.success && result.frame_type == v2::FrameType::CONNECT) {
            ++connect_callbacks;
            physical_turn_complete = result.physical_turn_complete;
            routed_snr = result.snr_source;
        }
    });
    int false_group_callbacks = 0;
    decoder.setBurstGroupCallback(
        [&](uint16_t, const std::vector<Bytes>&, bool, float, uint16_t,
            bool, uint8_t, bool) { ++false_group_callbacks; });

    auto audio = withSilence(samples);
    feedInChunks(decoder, audio);

    int queued_connects = 0;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && result.frame_type == v2::FrameType::CONNECT) {
            ++queued_connects;
        }
    }

    if (connect_callbacks != 1 || queued_connects != 1) {
        std::cout << "FAIL: half-open fallback delivered callbacks="
                  << connect_callbacks << " queued=" << queued_connects << "\n";
        return false;
    }
    if (!physical_turn_complete) {
        std::cout << "FAIL: half-open fallback did not prove physical turn end\n";
        return false;
    }
    if (routed_snr != SNRSource::NONE) {
        std::cout << "FAIL: half-open MC fallback leaked SNR into OFDM adaptation\n";
        return false;
    }
    float physical_mean_db = 0.0f;
    float physical_spread_db = 0.0f;
    const size_t physical_samples =
        decoder.physicalSnrStats(physical_mean_db, physical_spread_db);
    if (physical_samples != 0 ||
        decoder.hasLastOFDMBroadbandSNREstimate() ||
        std::abs(decoder.getLastFadingIndex()) > 1e-6f) {
        std::cout << "FAIL: half-open MC fallback retained speculative OFDM "
                  << "measurements (physical_n=" << physical_samples
                  << " ofdm="
                  << (decoder.hasLastOFDMBroadbandSNREstimate() ? 1 : 0)
                  << " fading=" << decoder.getLastFadingIndex() << ")\n";
        return false;
    }
    if (false_group_callbacks != 0) {
        std::cout << "FAIL: half-open fallback emitted " << false_group_callbacks
                  << " synthetic OFDM group outcome(s)\n";
        return false;
    }
    if (!decoder.expectsFullOFDMAnchorForTesting()) {
        std::cout << "FAIL: half-open fallback did not re-arm real OFDM anchor\n";
        return false;
    }

    // The replayed ACK can itself fade.  The responder remains half-open and
    // must recover a later duplicate with the same cache-only mechanism while
    // preserving the original measurement baseline across retries.
    feedInChunks(decoder, audio);
    int repeated_queued_connects = 0;
    while (decoder.hasFrame()) {
        auto result = decoder.getFrame();
        if (result.success && result.frame_type == v2::FrameType::CONNECT) {
            ++repeated_queued_connects;
        }
    }
    const size_t repeated_physical_samples =
        decoder.physicalSnrStats(physical_mean_db, physical_spread_db);
    if (connect_callbacks != 2 || repeated_queued_connects != 1 ||
        repeated_physical_samples != 0 ||
        decoder.hasLastOFDMBroadbandSNREstimate() ||
        false_group_callbacks != 0 ||
        !decoder.expectsFullOFDMAnchorForTesting()) {
        std::cout << "FAIL: repeated half-open CONNECT did not remain "
                  << "reactive/measurement-clean (callbacks=" << connect_callbacks
                  << " queued=" << repeated_queued_connects
                  << " physical_n=" << repeated_physical_samples
                  << " groups=" << false_group_callbacks << ")\n";
        return false;
    }

    return true;
}

}  // namespace

int main() {
    setLogLevel(std::getenv("TEST_LOG") ? LogLevel::INFO : LogLevel::ERROR);

    const Bytes payload = {
        0x55, 0x4c, 0x54, 0x52, 0x41, 0x20, 0x4d, 0x43,
        0x44, 0x50, 0x53, 0x4b, 0x20, 0x4f, 0x4b
    };
    auto data_frame = v2::DataFrame::makeData("ALPHA", "BRAVO", 7, payload, CodeRate::R1_4);
    const Bytes data_serialized = data_frame.serialize();
    auto connect_frame = v2::ConnectFrame::makeConnect(
        "ALPHA", "BRAVO", static_cast<uint8_t>(protocol::WaveformMode::MC_DPSK),
        static_cast<uint8_t>(Modulation::DBPSK), static_cast<uint8_t>(CodeRate::R1_4));
    const Bytes connect_serialized = connect_frame.serialize();

    if (!runLoopback("standard data", mc_dpsk_presets::level8(), Modulation::DQPSK,
                     data_serialized, v2::FrameType::DATA)) return 1;
    if (!runLoopback("standard connect", mc_dpsk_presets::level8(), Modulation::DQPSK,
                     connect_serialized, v2::FrameType::CONNECT)) return 1;
    if (!runLoopback("standard awgn12 data", mc_dpsk_presets::level8(), Modulation::DQPSK,
                     data_serialized, v2::FrameType::DATA, true, 12.0f)) return 1;
    if (!runLoopback("robust_low data", mc_dpsk_presets::robust_low(), Modulation::DBPSK,
                     data_serialized, v2::FrameType::DATA)) return 1;
    if (!runLoopback("robust_low connect", mc_dpsk_presets::robust_low(), Modulation::DBPSK,
                     connect_serialized, v2::FrameType::CONNECT)) return 1;
    if (!runLoopback("robust_mid data", mc_dpsk_presets::robust_mid(), Modulation::DBPSK,
                     data_serialized, v2::FrameType::DATA)) return 1;
    if (!runLoopback("robust_mid connect", mc_dpsk_presets::robust_mid(), Modulation::DBPSK,
                     connect_serialized, v2::FrameType::CONNECT)) return 1;
    if (!runLoopback("robust data", mc_dpsk_presets::robust(), Modulation::DQPSK,
                     data_serialized, v2::FrameType::DATA)) return 1;
    if (!runLoopback("robust connect", mc_dpsk_presets::robust(), Modulation::DQPSK,
                     connect_serialized, v2::FrameType::CONNECT)) return 1;
    if (!runContinuousBurstLoopback()) return 1;
    if (!runConstellationChangeBurst(Modulation::DQPSK, Modulation::DBPSK)) return 1;
    if (!runConstellationChangeBurst(Modulation::DBPSK, Modulation::DQPSK)) return 1;
    if (!runLostFrameReport()) return 1;
    if (!runCompactBurst("compact burst", Modulation::DBPSK, false)) return 1;
    if (!runCompactBurst("compact burst, middle frame lost", Modulation::DBPSK, true)) return 1;
    if (!runCompactBurst("compact DQPSK burst at a DBPSK receiver", Modulation::DQPSK, false)) return 1;
    // 16-FSK, the bottom rung (2026-11-22): at a receiver commanded there (its peek must cover a
    // 486-symbol frame), and at a DBPSK receiver (the sender's timeout dropped it one rung: the
    // first frame fails in the DPSK-sized window and the receiver waits for a 16-FSK span).
    // A 16-FSK frame (3 codewords) is 10.4 s on air: the timing-derived window is one frame.
    const uint16_t mfsk_window = static_cast<uint16_t>(
        protocol::connection_policy::mcDpskWindowSizeForTiming(
            protocol::connection_policy::mcDpskFrameTiming(Modulation::MFSK16, 8, 1024, 3)));
    if (!runCompactBurst("compact 16-FSK burst at a 16-FSK receiver", Modulation::MFSK16, false,
                         Modulation::MFSK16, mfsk_window)) return 1;
    if (!runCompactBurst("compact 16-FSK burst at a DBPSK receiver", Modulation::MFSK16, false,
                         Modulation::DBPSK, mfsk_window)) return 1;
    if (!runCompactBurstPhysicalPerFrame(Modulation::DBPSK, Modulation::DBPSK)) return 1;
    if (!runCompactBurstPhysicalPerFrame(Modulation::DQPSK, Modulation::DBPSK)) return 1;
    if (!runMarginTest(20.0f, true)) return 1;
    if (!runMarginTest(-4.0f, false)) return 1;
    if (!runConnectedOFDMHalfOpenConnectFallback()) return 1;
    if (!runLowAmplitudePing("robust low-amplitude ping", mc_dpsk_presets::robust(),
                             0.030f)) return 1;

    std::cout << "Streaming MC-DPSK loopback: PASS\n";
    return 0;
}
