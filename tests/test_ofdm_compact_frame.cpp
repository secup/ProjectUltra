// Compact OFDM DATA frame (INV-OFDM-FRAME-001, protocol/ofdm_data_frame.hpp).
//
// 1. Codec: every fixed-frame geometry round-trips at its compact capacity (11 bytes more than the
//    v2 layout); the position byte; a frame from another session fails; the decoder still accepts
//    the v2 layout; a failed decode's hard decisions identify the frame by its header CRC.
// 2. End to end (StreamingEncoder burst -> StreamingDecoder, clean channel): inside a session every
//    member decodes at compact capacity and reports its place in the burst; without a session the
//    encoder keeps the v2 layout.
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "gui/modem/streaming_frame_policy.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/ofdm_data_frame.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;
namespace of = ultra::protocol::ofdm_frame;

namespace {
int failures = 0;
#define CHECK(cond, msg)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            ++failures;                                   \
            std::cout << "FAIL: " << msg << "\n";         \
        }                                                 \
    } while (0)

std::vector<float> llrsFor(const Bytes& air, size_t bits, float magnitude = 8.0f) {
    std::vector<float> llr(bits);
    for (size_t i = 0; i < bits; ++i) {
        const int bit = (air[i / 8] >> (7 - (i % 8))) & 1;
        llr[i] = bit ? -magnitude : magnitude;
    }
    return llr;
}

Bytes randomBytes(std::mt19937& rng, size_t n) {
    Bytes b(n);
    for (auto& x : b) x = static_cast<uint8_t>(rng() & 0xFF);
    return b;
}

void testCodec() {
    std::mt19937 rng(7);
    const uint32_t alpha = v2::hashCallsign("ALPHA"), bravo = v2::hashCallsign("BRAVO");
    for (CodeRate rate : {CodeRate::R1_4, CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4}) {
        for (int cw : {1, 4, 5, 8, 16}) {
            const size_t info = of::infoBytesFor(rate, cw, 27);
            const size_t capacity = info - of::kHeaderBytes;
            CHECK(capacity == v2::getFixedFramePayloadCapacity(rate, cw) + 11,
                  "compact capacity is the v2 one + 11 bytes (cw " << cw << ")");
            auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", 0x1234, randomBytes(rng, capacity),
                                                rate, cw, 27, capacity);
            frame.flags = static_cast<uint8_t>(v2::Flags::VERSION_V2 | v2::Flags::MORE_FRAG);
            const Bytes v2_bytes = frame.serialize();
            of::Position pos;
            pos.index = cw % 3;
            pos.count = 3;
            const auto air = of::encode(v2_bytes, rate, cw, 27, pos);
            CHECK(air.has_value(), "a full compact frame encodes");
            if (!air) continue;
            const size_t bits = static_cast<size_t>(cw) * 648;
            CHECK(air->size() * 8 == bits, "same air geometry as the v2 frame");
            const auto llr = llrsFor(*air, bits);
            const auto d = of::decode(llr, rate, cw, 27, alpha, bravo, /*session_known=*/true);
            CHECK(d.ok && d.compact, "decodes as compact");
            CHECK(d.frame == v2_bytes, "rebuilds the v2 frame byte for byte (cw " << cw << ")");
            CHECK(d.position.index == pos.index && d.position.count == 3, "carries its position");
            const auto other = of::decode(llr, rate, cw, 27, alpha, v2::hashCallsign("CHARLIE"), true);
            CHECK(!other.ok, "another session's receiver rejects it");
            const auto unknown = of::decode(llr, rate, cw, 27, 0, 0, /*session_known=*/false);
            CHECK(!unknown.ok, "a receiver outside the session cannot verify it");
            // The v2 layout still decodes, through the same call.
            if (v2_bytes.size() > info) continue;  // the compact capacity does not fit v2
            const auto v2_air = v2::encodeTurboFixedFrame(v2_bytes, rate, cw, 27);
            const auto dv = of::decode(llrsFor(v2_air, bits), rate, cw, 27, alpha, bravo, true);
            CHECK(dv.ok && !dv.compact, "the v2 layout is accepted too");
        }
    }
    // A v2-capacity frame in the v2 layout, decoded by a session receiver.
    {
        auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", 9, randomBytes(rng, 40), CodeRate::R1_2, 5);
        const Bytes v2_bytes = frame.serialize();
        const auto v2_air = v2::encodeTurboFixedFrame(v2_bytes, CodeRate::R1_2, 5, 27);
        const auto st = of::decodeStatus(llrsFor(v2_air, 5 * 648), CodeRate::R1_2, 5, 27, alpha, bravo, true);
        CHECK(st.allSuccess() && st.reassemble() == v2_bytes, "v2 layout through decodeStatus");
    }
    // Position byte.
    for (int count = 1; count <= 16; ++count) {
        for (int index = 0; index < count; ++index) {
            of::Position p;
            p.index = index;
            p.count = count;
            const auto q = of::unpackPosition(of::packPosition(p));
            CHECK(q.index == index && q.count == count, "position " << index << "/" << count);
        }
    }
    CHECK(!of::unpackPosition(of::kPositionUnknown).known(), "0xF0 reads as unknown");
    // The header CRC: a failed decode's hard decisions identify the frame when the header bits
    // are right, whatever the payload bits are, and only for this session.
    {
        const CodeRate rate = CodeRate::R1_4;
        const int cw = 4;
        const size_t capacity = of::infoBytesFor(rate, cw, 27) - of::kHeaderBytes;
        auto frame = v2::makeFixedDataFrame("ALPHA", "BRAVO", 77, randomBytes(rng, capacity), rate, cw,
                                            27, capacity);
        of::Position pos;
        pos.index = 1;
        pos.count = 2;
        const auto air = of::encode(frame.serialize(), rate, cw, 27, pos);
        const auto d = of::decode(llrsFor(*air, cw * 648), rate, cw, 27, alpha, bravo, true);
        Bytes info = d.hard_info;
        CHECK(info.size() > of::kHeaderBytes + 10, "the decode exposes its information bytes");
        if (info.size() > of::kHeaderBytes + 10) {
            info[of::kHeaderBytes + 5] ^= 0x5A;  // a payload error
            const auto h = of::parseHeader(info, rate, cw, 27, alpha, bravo);
            CHECK(h && h->seq == 77 && h->position.index == 1 && h->position.count == 2,
                  "a frame with payload errors is still identified by its header CRC");
            CHECK(!of::parseHeader(info, rate, cw, 27, alpha, v2::hashCallsign("CHARLIE")),
                  "the header CRC is session-seeded");
            info[1] ^= 0x01;  // a header error
            CHECK(!of::parseHeader(info, rate, cw, 27, alpha, bravo),
                  "a header error fails the header CRC");
        }
    }
}

ModemConfig ofdmGeometry(Modulation mod, CodeRate rate) {
    ModemConfig cfg;
    cfg.fft_size = 1024;
    cfg.num_carriers = 59;
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500.0f;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;
    cfg.modulation = mod;
    cfg.code_rate = rate;
    cfg.use_pilots = true;
    cfg.pilot_spacing = ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);
    return cfg;
}

// One clean burst of `n` frames; returns the decoded frames' (payload, position).
struct Member { Bytes frame; int index; int count; };
std::vector<Member> runBurst(bool session, int n, size_t payload_bytes, std::vector<Bytes>& sent) {
    const Modulation mod = Modulation::QPSK;
    const CodeRate rate = CodeRate::R1_2;
    const int cw = 5;
    std::mt19937 rng(3);
    std::vector<Bytes> frames;
    const size_t capacity = v2::ofdmSessionPayloadCapacity(rate, cw, 27);
    for (int i = 0; i < n; ++i) {
        auto f = v2::makeFixedDataFrame("ALPHA", "BRAVO", static_cast<uint16_t>(100 + i),
                                        randomBytes(rng, payload_bytes), rate, cw, 27, capacity);
        f.flags = static_cast<uint8_t>(v2::Flags::VERSION_V2 |
                                       (i + 1 < n ? v2::Flags::MORE_FRAG : v2::Flags::FINAL));
        frames.push_back(f.serialize());
    }
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::OFDM_CHIRP);
    encoder.setOFDMConfig(ofdmGeometry(mod, rate));
    encoder.setDataMode(mod, rate);
    encoder.setFixedFrameCodewords(cw);
    encoder.setBurstInterleaveGroupSize(n);
    encoder.setBurstDescriptorEnabled(true);
    encoder.setBurstDescriptorIdentity(v2::kBurstDescriptorIdentity, v2::kBurstDescriptorIdentity);
    encoder.setOfdmCompactFrames(session);
    BurstAnchorOptions anchor;
    anchor.force_full_group_start = true;
    const auto tx = encoder.encodeBurstLight(frames, anchor);
    sent = gui::streaming_frame_policy::preparePhysicalBurstFrames(
        frames, gui::streaming_frame_policy::shouldStampPhysicalBurstEnd(
                    frames.size(), protocol::WaveformMode::OFDM_CHIRP, true, false));

    StreamingDecoder decoder;
    decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP, ofdmGeometry(mod, rate), mod, rate);
    decoder.setKnownCFO(0.0f);
    decoder.setFixedFrameCodewords(cw);
    if (session) {
        decoder.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    }
    // Per-frame delivery (frame queue), so each member's DecodeResult - and its position - is seen.
    decoder.setBurstTransportRxEnabled(false);
    decoder.expectFullOFDMAnchorOnce();
    std::vector<float> audio(24000, 0.0f);
    audio.insert(audio.end(), tx.begin(), tx.end());
    audio.resize(audio.size() + 96000, 0.0f);
    std::vector<Member> got;
    for (size_t pos = 0; pos < audio.size(); pos += 4800) {
        decoder.feedAudio(audio.data() + pos, std::min<size_t>(4800, audio.size() - pos));
        decoder.processBuffer();
        while (decoder.hasFrame()) {
            auto r = decoder.getFrame();
            if (r.success && v2::isDataFrame(r.frame_type)) {
                got.push_back({r.frame_data, r.physical_index, r.physical_count});
            }
        }
    }
    return got;
}

void testEndToEnd() {
    const int n = 4;
    std::vector<Bytes> sent;
    const size_t compact_capacity = v2::ofdmSessionPayloadCapacity(CodeRate::R1_2, 5, 27);
    const auto got = runBurst(/*session=*/true, n, compact_capacity, sent);
    CHECK(static_cast<int>(got.size()) == n, "every member of a compact burst decodes (" << got.size() << ")");
    for (size_t i = 0; i < got.size() && i < sent.size(); ++i) {
        CHECK(got[i].frame == sent[i], "member " << i << " arrives byte for byte at compact capacity");
        CHECK(got[i].index == static_cast<int>(i) && got[i].count == n,
              "member " << i << " says it is frame " << got[i].index << " of " << got[i].count);
    }
    // Outside a session the encoder keeps the v2 layout (and so the v2 capacity).
    const auto plain = runBurst(/*session=*/false, n, 40, sent);
    CHECK(static_cast<int>(plain.size()) == n, "a v2 burst still decodes");
    for (const auto& m : plain) CHECK(m.count == 0, "a v2-layout frame carries no position");
}
// A group whose descriptor never came (encoder without one: the receiver arms on the head's
// sign marker, UNPROVEN_MARKER) and whose LAST frame is destroyed (no CRC tail). The v2 layout gives
// the receiver no way to know N, so it abandons the group: no callback, no ACK. A compact member
// says "frame k of N": the group finalizes at N and is delivered.
struct GroupOutcome { int callbacks = 0; size_t frames = 0; uint32_t proofs = 0; };
GroupOutcome runDescriptorlessGroup(bool compact) {
    const Modulation mod = Modulation::QPSK;
    const CodeRate rate = CodeRate::R1_2;
    const int cw = 5;
    const int n = 4;
    std::mt19937 rng(11);
    std::vector<Bytes> frames;
    for (int i = 0; i < n; ++i) {
        auto f = v2::makeFixedDataFrame("ALPHA", "BRAVO", static_cast<uint16_t>(200 + i),
                                        randomBytes(rng, 60), rate, cw);
        f.flags = static_cast<uint8_t>(v2::Flags::VERSION_V2 | v2::Flags::MORE_FRAG);
        frames.push_back(f.serialize());
    }
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::OFDM_CHIRP);
    encoder.setOFDMConfig(ofdmGeometry(mod, rate));
    encoder.setDataMode(mod, rate);
    encoder.setFixedFrameCodewords(cw);
    encoder.setBurstInterleaveGroupSize(n);
    encoder.setBurstDescriptorEnabled(false);
    encoder.setOfdmCompactFrames(compact);
    BurstAnchorOptions anchor;
    anchor.force_full_group_start = true;
    auto tx = encoder.encodeBurstLight(frames, anchor);
    // Destroy the last frame with noise of the burst's own level (it keeps its energy, so the
    // slicer still takes it as a member; it just cannot decode).
    double rms = 0.0;
    for (float v : tx) rms += static_cast<double>(v) * v;
    rms = std::sqrt(rms / static_cast<double>(tx.size()));
    std::normal_distribution<double> g(0.0, rms);
    const size_t frame_samples = 28000;  // the last ~0.58 s: inside the 4th member only
    for (size_t i = tx.size() - frame_samples; i < tx.size(); ++i) tx[i] = static_cast<float>(g(rng));

    StreamingDecoder decoder;
    decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP, ofdmGeometry(mod, rate), mod, rate);
    decoder.setKnownCFO(0.0f);
    decoder.setFixedFrameCodewords(cw);
    decoder.setBurstInterleaveGroupSize(6);  // a stale latched N: the previous group's
    decoder.setMcDpskSessionAddresses(v2::hashCallsign("BRAVO"), v2::hashCallsign("ALPHA"));
    decoder.expectFullOFDMAnchorOnce();
    GroupOutcome out;
    decoder.setBurstGroupCallback([&out](uint16_t, const std::vector<Bytes>& group_frames, bool, float,
                                         uint16_t, bool, uint8_t, bool) {
        ++out.callbacks;
        out.frames += group_frames.size();
    });
    std::vector<float> audio(24000, 0.0f);
    audio.insert(audio.end(), tx.begin(), tx.end());
    audio.resize(audio.size() + 3 * 48000, 0.0f);
    for (size_t pos = 0; pos < audio.size(); pos += 4800) {
        decoder.feedAudio(audio.data() + pos, std::min<size_t>(4800, audio.size() - pos));
        decoder.processBuffer();
        while (decoder.hasFrame()) decoder.getFrame();
    }
    out.proofs = decoder.burstPositionProofsForTesting();
    return out;
}

void testDescriptorlessGroupIsProvenByPosition() {
    const auto v2_layout = runDescriptorlessGroup(/*compact=*/false);
    const auto compact = runDescriptorlessGroup(/*compact=*/true);
    std::cout << "descriptorless group, tail destroyed: v2 " << v2_layout.callbacks << " callback(s) "
              << v2_layout.frames << " frame(s); compact " << compact.callbacks << " callback(s) "
              << compact.frames << " frame(s), " << compact.proofs << " position proof(s)\n";
    CHECK(v2_layout.frames == 0, "without positions the descriptorless, tail-less group is lost");
    CHECK(compact.proofs == 1, "a decoded member's position proves the group");
    CHECK(compact.callbacks == 1 && compact.frames == 3,
          "the proven group is delivered with its three decoded frames");
}
}  // namespace

int main() {
    ultra::setLogLevel(ultra::LogLevel::ERROR);
    testCodec();
    testEndToEnd();
    testDescriptorlessGroupIsProvenByPosition();
    if (failures) {
        std::cout << "OfdmCompactFrame: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "OfdmCompactFrame: all checks passed\n";
    return 0;
}
