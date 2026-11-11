// Compact MC-DPSK DATA frame (protocol/mcdpsk_frame.hpp): capacity, round trip, session
// addressing through the CRC seed, rejection of what does not fit, and decoding through noise.
#include "protocol/mcdpsk_frame.hpp"
#include "protocol/frame_v2.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace ultra;
using namespace ultra::protocol;
namespace v2 = ultra::protocol::v2;

namespace {
int g_fail = 0;
int g_pass = 0;
#define CHECK(cond, msg)                                       \
    do {                                                       \
        if (cond) {                                            \
            ++g_pass;                                          \
        } else {                                               \
            ++g_fail;                                          \
            std::printf("FAIL: %s (line %d)\n", msg, __LINE__); \
        }                                                      \
    } while (0)

// Air bytes (MSB-first) -> LLRs of a given magnitude, positive = bit 0.
std::vector<float> llrsFor(const Bytes& air, float magnitude) {
    std::vector<float> llr;
    llr.reserve(air.size() * 8);
    for (uint8_t byte : air) {
        for (int b = 7; b >= 0; --b) llr.push_back(((byte >> b) & 1u) ? -magnitude : magnitude);
    }
    return llr;
}

Bytes dataFrame(uint16_t seq, size_t payload_size, uint8_t flags) {
    Bytes payload(payload_size);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<uint8_t>(0x30 + 7 * i);
    auto frame = v2::DataFrame::makeData("ALPHA", "BRAVO", seq, payload, CodeRate::R1_4);
    frame.flags = flags;
    return frame.serialize();
}
}  // namespace

int main() {
    // Capacity: 3 slots at R1/4 = floor(1944/4/8) = 60 bytes, minus the 4-byte CRC = 56,
    // minus the 4-byte header = 52 (the v2 chain carried 37).
    CHECK(mcdpsk_frame::infoBytesFor(3, CodeRate::R1_4) == 56, "3 codewords R1/4: 56 information bytes");
    CHECK(mcdpsk_frame::payloadCapacity(3, CodeRate::R1_4) == 52, "3 codewords R1/4: 52 payload bytes");
    CHECK(mcdpsk_frame::payloadCapacity(4, CodeRate::R1_4) == 73, "4 codewords R1/4: 73 payload bytes");
    CHECK(mcdpsk_frame::payloadCapacity(3, CodeRate::R1_4) >
              v2::getVariableFramePayloadCapacity(CodeRate::R1_4, 3),
          "more payload than the v2 codeword chain");

    const uint32_t alpha = v2::hashCallsign("ALPHA");
    const uint32_t bravo = v2::hashCallsign("BRAVO");
    CHECK(mcdpsk_frame::sessionCrcSeed(alpha, bravo) != mcdpsk_frame::sessionCrcSeed(bravo, alpha),
          "the two directions get different seeds");

    // Round trip, full and short payloads: the rebuilt v2 frame equals the original (total_cw
    // aside, which the compact frame sets to its codeword count).
    const uint8_t flags = v2::Flags::VERSION_V2 | v2::Flags::MORE_FRAG | v2::Flags::PHYSICAL_BURST_END;
    for (size_t size : {size_t(52), size_t(5), size_t(0)}) {
        const Bytes original = dataFrame(1234, size, flags);
        const auto air = mcdpsk_frame::encode(original, 3, CodeRate::R1_4);
        CHECK(air.has_value() && air->size() == 1944 / 8, "encode: 243 air bytes");
        if (!air) continue;
        const auto llr = llrsFor(*air, 6.0f);
        const auto r = mcdpsk_frame::decode(llr, 3, CodeRate::R1_4, alpha, bravo);
        CHECK(r.ok, "noiseless round trip decodes");
        const auto rebuilt = v2::DataFrame::deserialize(r.v2_frame);
        const auto expect = v2::DataFrame::deserialize(original);
        CHECK(rebuilt && expect && rebuilt->type == v2::FrameType::DATA &&
                  rebuilt->flags == expect->flags && rebuilt->seq == expect->seq &&
                  rebuilt->src_hash == expect->src_hash && rebuilt->dst_hash == expect->dst_hash &&
                  rebuilt->payload == expect->payload && rebuilt->total_cw == 3,
              "the rebuilt v2 frame carries the original's flags, seq, addresses and payload");

        // Another session (or the other direction) fails the CRC.
        CHECK(!mcdpsk_frame::decode(llr, 3, CodeRate::R1_4, bravo, alpha).ok,
              "the reverse direction's seed rejects the frame");
        CHECK(!mcdpsk_frame::decode(llr, 3, CodeRate::R1_4, alpha, v2::hashCallsign("CHARLIE")).ok,
              "another session's seed rejects the frame");
    }

    // What does not fit, or is not DATA, is refused.
    CHECK(!mcdpsk_frame::encode(dataFrame(1, 53, flags), 3, CodeRate::R1_4).has_value(),
          "53 bytes do not fit in 3 codewords");
    const Bytes control = v2::ControlFrame::makeDisconnect("ALPHA", "BRAVO").serialize();
    CHECK(!mcdpsk_frame::encode(control, 3, CodeRate::R1_4).has_value(), "a control frame is refused");
    CHECK(!mcdpsk_frame::decode(std::vector<float>(100, 1.0f), 3, CodeRate::R1_4, alpha, bravo).ok,
          "too few LLRs");

    // Through noise: BPSK-equivalent LLRs 2y/sigma^2 at Es/N0 = -3 dB per coded bit (rate 0.247:
    // Eb/N0 = 3.1 dB) — a turbo codeword of this length decodes there essentially always.
    {
        const Bytes original = dataFrame(77, 52, flags);
        const auto air = *mcdpsk_frame::encode(original, 3, CodeRate::R1_4);
        std::mt19937 rng(7);
        const float es_n0 = std::pow(10.0f, -3.0f / 10.0f);
        const float sigma = std::sqrt(1.0f / (2.0f * es_n0));
        std::normal_distribution<float> n(0.0f, sigma);
        int ok = 0;
        const int trials = 40;
        for (int t = 0; t < trials; ++t) {
            auto llr = llrsFor(air, 1.0f);
            for (auto& v : llr) v = 2.0f * (v + n(rng)) / (sigma * sigma);
            ok += mcdpsk_frame::decode(llr, 3, CodeRate::R1_4, alpha, bravo).ok ? 1 : 0;
        }
        std::printf("noisy decode: %d/%d at Es/N0 -3 dB per coded bit\n", ok, trials);
        CHECK(ok >= trials - 1, "decodes through noise at Eb/N0 3.1 dB");
    }

    std::printf("MCDPSKFrame: %d/%d passed\n", g_pass, g_pass + g_fail);
    return g_fail == 0 ? 0 : 1;
}
