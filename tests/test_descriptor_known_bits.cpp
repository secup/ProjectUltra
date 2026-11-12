// INV-DESC-KNOWN-001: a BURST_HEADER descriptor's protocol-fixed bits, and the receiver's second
// decode with them pinned (BUG-CONTROL-PRESCREEN-DROPS-DESCRIPTORS, 2026-11-12).
//
// 1. The known-bit table is exact: every entry equals the corresponding information bit of the
//    LDPC codeword a real descriptor is sent as, whatever geometry it declares.
// 2. End to end (StreamingEncoder -> AWGN -> connected StreamingDecoder): at an SNR where the
//    plain decode loses most descriptors, the decoder consumes most of them and the pinned decode
//    did the work. measure_ack_fer desc_light, AWGN 3 dB, 200 trials: plain 36 of 169 acquired,
//    pinned 128.
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "protocol/frame_v2.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

namespace {
int failures = 0;
#define CHECK(cond, msg)                                         \
    do {                                                         \
        if (!(cond)) {                                           \
            ++failures;                                          \
            std::cout << "FAIL: " << msg << "\n";                \
        }                                                        \
    } while (0)

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

void testKnownBitsMatchEveryDescriptor() {
    const auto& known = v2::burstDescriptorKnownBits();
    CHECK(known.size() == 82, "82 protocol-fixed bits (magic, type, flags, two hashes, 2 pad)");
    std::set<uint16_t> positions;
    for (const auto& kb : known) positions.insert(kb.position);
    CHECK(positions.size() == known.size(), "no position listed twice");
    struct Geometry { uint16_t seq; uint8_t group, cw; Modulation mod; CodeRate rate; uint8_t flags; uint8_t z; };
    const Geometry geometries[] = {
        {0, 6, 5, Modulation::QPSK, CodeRate::R1_2, 0x00, 27},
        {0, 16, 16, Modulation::QAM32, CodeRate::R2_3, 0x0F, 81},
        {0x3F, 2, 4, Modulation::QAM8, CodeRate::R1_4, 0x08, 27},
    };
    for (const auto& g : geometries) {
        const Bytes frame = v2::ControlFrame::makeBurstHeader(
            v2::kBurstDescriptorIdentity, v2::kBurstDescriptorIdentity, g.seq, g.group, g.cw,
            g.mod, g.rate, g.flags, g.z).serialize();
        const auto codewords = v2::encodeFrameWithLDPC(v2::whitenControlFrame(frame), CodeRate::R1_4);
        CHECK(codewords.size() == 1, "a descriptor is one R1/4 codeword");
        if (codewords.size() != 1) continue;
        const Bytes& cw = codewords[0];
        for (const auto& kb : known) {
            const int bit = (cw[kb.position / 8] >> (7 - (kb.position % 8))) & 1;
            if (bit != kb.value) {
                CHECK(false, "known bit " << kb.position << " differs in a real descriptor");
                break;
            }
        }
    }
}

// One arm: kTrials descriptors at kSnrDb through AWGN, a fresh connected decoder each.
struct ArmResult { int consumed = 0; int rescued = 0; };
ArmResult runArm(const std::vector<float>& tx, Modulation mod, CodeRate rate, float snr_db,
                 int trials, bool pinned) {
    ArmResult r;
    for (int t = 0; t < trials; ++t) {
        std::vector<float> audio(28800, 0.0f);  // 600 ms of band noise first
        audio.insert(audio.end(), tx.begin(), tx.end());
        audio.resize(audio.size() + 48000, 0.0f);
        ultra::ota_channel_core::SimulatedChannel channel;
        channel.setSeed(1000 + static_cast<uint64_t>(t));
        channel.configure(snr_db, ultra::ota_channel_core::ChannelType::AWGN);
        channel.transmitFromA(audio);
        const auto rx = channel.receiveForB(audio.size());

        StreamingDecoder decoder;
        decoder.setConnectedOFDMMode(protocol::WaveformMode::OFDM_CHIRP, ofdmGeometry(mod, rate),
                                     mod, rate);
        decoder.setKnownCFO(0.0f);
        decoder.expectFullOFDMAnchorOnce();
        decoder.setDescriptorKnownBitsForTesting(pinned);
        for (size_t pos = 0; pos < rx.size(); pos += 4800) {
            const size_t len = std::min<size_t>(4800, rx.size() - pos);
            decoder.feedAudio(rx.data() + pos, len);
            decoder.processBuffer();
        }
        r.consumed += decoder.descriptorsConsumedForTesting() > 0 ? 1 : 0;
        r.rescued += decoder.descriptorKnownBitRescuesForTesting() > 0 ? 1 : 0;
    }
    return r;
}

void testPinnedDecodeRecoversWeakDescriptors() {
    const Modulation mod = Modulation::QPSK;
    const CodeRate rate = CodeRate::R1_2;
    const Bytes frame = v2::ControlFrame::makeBurstHeader(
        v2::kBurstDescriptorIdentity, v2::kBurstDescriptorIdentity, 0, 6, 5, mod, rate, 0).serialize();
    StreamingEncoder encoder;
    encoder.setMode(protocol::WaveformMode::OFDM_CHIRP);
    encoder.setOFDMConfig(ofdmGeometry(mod, rate));
    encoder.setDataMode(mod, rate);
    const auto tx = encoder.encodeFrame(frame);
    CHECK(!tx.empty(), "the encoder produced the descriptor");
    if (tx.empty()) return;

    // Same channel realizations in both arms: only the pinned second decode differs.
    constexpr int kTrials = 24;
    constexpr float kSnrDb = 2.0f;
    const ArmResult plain = runArm(tx, mod, rate, kSnrDb, kTrials, /*pinned=*/false);
    const ArmResult pinned = runArm(tx, mod, rate, kSnrDb, kTrials, /*pinned=*/true);
    std::cout << "AWGN " << kSnrDb << " dB, " << kTrials << " descriptors: plain decode "
              << plain.consumed << ", with the fixed bits pinned " << pinned.consumed << " ("
              << pinned.rescued << " recovered by the second decode)\n";
    CHECK(plain.rescued == 0, "no rescue when the pinned decode is off");
    CHECK(pinned.consumed >= plain.consumed + 3,
          "pinning the fixed bits recovers descriptors the plain decode loses");
    CHECK(pinned.consumed - plain.consumed == pinned.rescued,
          "every extra descriptor is a pinned-decode rescue");
}
}  // namespace

int main() {
    ultra::setLogLevel(ultra::LogLevel::ERROR);
    testKnownBitsMatchEveryDescriptor();
    testPinnedDecodeRecoversWeakDescriptors();
    if (failures) {
        std::cout << "DescriptorKnownBits: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "DescriptorKnownBits: all checks passed\n";
    return 0;
}
