// Turbo OFDM data frame (frame_v2 encodeTurboFixedFrame / decodeTurboFixedFrame):
// air-geometry parity with LDPC, the CodewordStatus contract, header recovery from a
// failed decode, and whole-frame chase combining through SoftCombineBuffer.
#include "fec/soft_combine.hpp"
#include "protocol/frame_v2.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace ultra;
namespace v2 = ultra::protocol::v2;

namespace {
int g_fail = 0;
void check(bool ok, const char* msg) {
    std::printf("%s %s\n", ok ? "ok:  " : "FAIL:", msg);
    if (!ok) ++g_fail;
}

Bytes makeFrame(CodeRate rate, int cw, uint16_t seq, std::mt19937_64& rng) {
    const size_t cap = v2::getFixedFramePayloadCapacity(rate, cw);
    Bytes payload(cap);
    for (auto& b : payload) b = static_cast<uint8_t>(rng() & 0xFF);
    return v2::makeFixedDataFrame("ALPHA", "BRAVO", seq, payload, rate, cw).serialize();
}

std::vector<float> bpsk(const Bytes& air, double esn0, std::mt19937_64& rng) {
    const size_t N = air.size() * 8;
    std::vector<float> llr(N);
    const double s2 = 1.0 / (2.0 * esn0);
    std::normal_distribution<double> nd(0.0, std::sqrt(s2));
    for (size_t a = 0; a < N; ++a) {
        const int bit = (air[a / 8] >> (7 - (a % 8))) & 1;
        const double y = (bit ? -1.0 : 1.0) + nd(rng);
        llr[a] = static_cast<float>(std::clamp(2.0 * y / s2, -20.0, 20.0));
    }
    return llr;
}
}  // namespace

int main() {
    std::mt19937_64 rng(99);
    struct G { CodeRate rate; int cw; const char* name; };
    const G geos[] = {{CodeRate::R1_4, 4, "R1/4 cw4"},  {CodeRate::R1_2, 8, "R1/2 cw8"},
                      {CodeRate::R2_3, 8, "R2/3 cw8"},  {CodeRate::R3_4, 8, "R3/4 cw8"},
                      {CodeRate::R2_3, 12, "R2/3 cw12"}, {CodeRate::R2_3, 16, "R2/3 cw16"}};

    // 1. Air-geometry parity + noiseless round trip + status contract.
    for (const auto& g : geos) {
        const Bytes frame = makeFrame(g.rate, g.cw, 7, rng);
        const Bytes ldpc = v2::encodeFixedFrame(frame, g.rate, g.cw, true, 102);
        const Bytes turbo = v2::encodeTurboFixedFrame(frame, g.rate, g.cw);
        char m[128];
        std::snprintf(m, sizeof(m), "%s: turbo air size == LDPC air size (%zu bytes)", g.name, ldpc.size());
        check(turbo.size() == ldpc.size(), m);
        std::vector<float> llr(turbo.size() * 8);
        for (size_t a = 0; a < llr.size(); ++a) llr[a] = ((turbo[a / 8] >> (7 - a % 8)) & 1) ? -12.f : 12.f;
        const auto st = v2::decodeTurboFixedFrame(llr, g.rate, g.cw);
        bool shape = st.decoded.size() == static_cast<size_t>(g.cw) && st.data.size() == static_cast<size_t>(g.cw) &&
                     st.iterations.size() == static_cast<size_t>(g.cw) && st.harq_attempts.size() == static_cast<size_t>(g.cw);
        std::snprintf(m, sizeof(m), "%s: status vectors have length cw and reassemble() == frame", g.name);
        check(shape && st.allSuccess() && st.reassemble() == frame, m);
    }

    // 2. Failure contract: all false, never a phantom success.
    {
        std::vector<float> zero(8 * 648, 0.0f);
        const auto st = v2::decodeTurboFixedFrame(zero, CodeRate::R1_2, 8);
        bool all_false = true;
        for (bool d : st.decoded) all_false &= !d;
        check(all_false && st.decoded.size() == 8 && st.reassemble().empty(), "failure => all CWs false, empty reassemble");
    }

    // 3. Chase combining: two receptions that each FAIL alone decode once combined.
    {
        const CodeRate rate = CodeRate::R1_2;
        const int cw = 8;
        int both_fail_pairs = 0, recovered = 0, header_recovered = 0;
        fec::SoftCombineBuffer harq;
        harq.setEnabled(true);
        for (int trial = 0; trial < 40 && both_fail_pairs < 12; ++trial) {
            const Bytes frame = makeFrame(rate, cw, static_cast<uint16_t>(100 + trial), rng);
            const Bytes air = v2::encodeTurboFixedFrame(frame, rate, cw);
            // Es/N0 -2.8 dB (Eb/N0 ~0.2 dB, at capacity for R1/2): single receptions
            // fail; two combined receptions gain 3 dB (Eb/N0 ~3.2 dB) and decode.
            const double esn0 = std::pow(10.0, -2.8 / 10.0);
            const auto r1 = bpsk(air, esn0, rng);
            const auto r2 = bpsk(air, esn0, rng);
            Bytes hard;
            const auto s1 = v2::decodeTurboFixedFrame(r1, rate, cw, 27, &hard);
            const auto s2 = v2::decodeTurboFixedFrame(r2, rate, cw);
            if (s1.allSuccess() || s2.allSuccess()) continue;
            ++both_fail_pairs;
            if (v2::parseHeader(hard).valid) ++header_recovered;
            fec::SoftCombineBuffer::HarqKeyInputs ki;
            ki.sender_hash = 1;
            ki.dst_hash = 2;
            ki.seq = static_cast<uint16_t>(100 + trial);
            ki.rate = rate;
            ki.cw_count = static_cast<uint8_t>(cw);
            ki.cw_index = fec::SoftCombineBuffer::kWholeFrameCwIndex;
            ki.header_crc = 0x1234;
            const auto key = fec::SoftCombineBuffer::makeKey(ki);
            std::vector<float> c1, c2;
            check(harq.combine(key, r1, c1) == 1, "first reception: no prior history");
            harq.retain(key, c1);
            const int attempts = harq.combine(key, r2, c2);
            const auto sc = v2::decodeTurboFixedFrame(c2, rate, cw);
            if (attempts == 2 && sc.allSuccess() && sc.reassemble() == frame) ++recovered;
            harq.drop(key);
        }
        std::printf("      combining: %d/%d both-failed pairs recovered; header recovered on %d failures\n",
                    recovered, both_fail_pairs, header_recovered);
        check(both_fail_pairs >= 8, "enough both-failed pairs to judge combining");
        check(recovered * 2 >= both_fail_pairs, "chase combining recovers >= half of both-failed pairs (~3 dB gain)");
    }

    // 4. Header CRC separates whole-frame HARQ histories.
    {
        fec::SoftCombineBuffer::HarqKeyInputs a, b;
        a.sender_hash = b.sender_hash = 5;
        a.seq = b.seq = 9;
        a.cw_index = b.cw_index = fec::SoftCombineBuffer::kWholeFrameCwIndex;
        a.header_crc = 0x1111;
        b.header_crc = 0x2222;
        check(!(fec::SoftCombineBuffer::makeKey(a) == fec::SoftCombineBuffer::makeKey(b)),
              "different header CRC => different whole-frame HARQ key");
    }

    if (g_fail == 0) {
        std::printf("PASS: turbo data frame\n");
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_fail);
    return 1;
}
