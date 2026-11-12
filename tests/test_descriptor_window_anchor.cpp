// INV-DESC-WINDOW-001: the burst descriptor carries the sender's ARQ window anchor (move epoch +
// tx base), and the receiver anchors to it at every group start (2026-11-12).
//
// The failure it removes (GUI, ITU Good 20 -> 5 dB drop, every run): the receiver commanded a
// rate step-down, the sender re-cut its window (new move epoch) and the first burst at the new
// rung failed entirely. The receiver had decoded no frame of the new era, so its ACK - "nothing
// decoded, drop another rung" - carried the old era and the sender discarded it; the sender
// repeated the failing rung for four rounds (~63 s) until the waveform follow left OFDM. When
// the new era's first frame was the one lost, the receiver could not anchor the era at all and
// went ACK-silent until the sender's timeout resend.
//
// 1. The 16-bit anchor word packs and unpacks, and the 13-bit base reconstructs across the
//    16-bit seq wrap.
// 2. A descriptor carrying the word keeps every protocol-fixed bit (INV-DESC-KNOWN-001).
// 3. End to end (StreamingEncoder -> AWGN -> connected StreamingDecoder): the decoder hands the
//    epoch and base to its callback; a descriptor without the word fires nothing.
// 4. ARQ, the failed group after a step-down: without the anchor the sender rejects the
//    receiver's ACK (the repro); with it the ACK is accepted as a zero-progress round.
// 5. ARQ, the era's first frame lost: without the anchor the receiver is ACK-silent (the
//    repro); with it the later frame is acknowledged and the sender retires it.
// 6. ARQ, the switch's own path: the abort abandons the frames in flight and the sender's base
//    runs ahead of the receiver's; the anchor is adopted there and both cases above hold.
// 7. ARQ, impossible anchors are refused; an anchored era learns nothing from its own anchor.
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "ota_channel_core/channel.hpp"
#include "protocol/descriptor_window_anchor.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/selective_repeat_arq.hpp"
#include "ultra/logging.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/types.hpp"

#include <iostream>
#include <vector>

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::protocol;
namespace v2 = ultra::protocol::v2;
namespace anchor = ultra::protocol::descriptor_window_anchor;

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

void testWordCodec() {
    for (uint8_t epoch = 0; epoch < 4; ++epoch) {
        for (uint16_t base : {uint16_t{0}, uint16_t{7}, uint16_t{0x1FFF}, uint16_t{0x2000},
                              uint16_t{0xFFFF}}) {
            const auto a = anchor::unpack(anchor::pack(epoch, base));
            CHECK(a.present, "a packed word is present");
            CHECK(a.epoch == epoch, "epoch round-trips");
            CHECK(a.base_low == (base & anchor::kBaseMask), "base mod 8192 round-trips");
        }
    }
    CHECK(!anchor::unpack(0).present, "the old always-zero field carries no anchor");
    // Reconstruction picks the representative nearest the receiver's base, across the wrap.
    CHECK(anchor::reconstructBase(65534 & anchor::kBaseMask, 3) == 65534,
          "a sender base 5 behind across the 16-bit wrap reconstructs");
    CHECK(anchor::baseDistance(65534 & anchor::kBaseMask, 3) == -5, "signed distance -5");
    CHECK(anchor::reconstructBase(40 & anchor::kBaseMask, 40) == 40, "equal bases");
    CHECK(anchor::reconstructBase((9000 - 16) & anchor::kBaseMask, 9000) == 9000 - 16,
          "a window behind, above 8192");
}

void testKnownBitsUnchanged() {
    const auto& known = v2::burstDescriptorKnownBits();
    const Bytes frame = v2::ControlFrame::makeBurstHeader(
        v2::kBurstDescriptorIdentity, v2::kBurstDescriptorIdentity, anchor::pack(3, 0x1ABC), 4, 5,
        Modulation::QPSK, CodeRate::R1_2, 0x06).serialize();
    const auto codewords = v2::encodeFrameWithLDPC(v2::whitenControlFrame(frame), CodeRate::R1_4);
    CHECK(codewords.size() == 1, "a descriptor is one R1/4 codeword");
    if (codewords.size() != 1) return;
    for (const auto& kb : known) {
        const int bit = (codewords[0][kb.position / 8] >> (7 - (kb.position % 8))) & 1;
        if (bit != kb.value) {
            CHECK(false, "known bit " << kb.position << " differs once the seq carries an anchor");
            break;
        }
    }
}

struct DecodedAnchor { int calls = 0; uint8_t epoch = 0xFF; uint16_t base_low = 0xFFFF; };

DecodedAnchor decodeDescriptor(uint16_t seq_word) {
    const Modulation mod = Modulation::QPSK;
    const CodeRate rate = CodeRate::R1_2;
    const Bytes frame = v2::ControlFrame::makeBurstHeader(
        v2::kBurstDescriptorIdentity, v2::kBurstDescriptorIdentity, seq_word, 4, 5, mod, rate,
        0).serialize();
    StreamingEncoder encoder;
    encoder.setMode(WaveformMode::OFDM_CHIRP);
    encoder.setOFDMConfig(ofdmGeometry(mod, rate));
    encoder.setDataMode(mod, rate);
    const auto tx = encoder.encodeFrame(frame);
    DecodedAnchor out;
    if (tx.empty()) return out;
    std::vector<float> audio(28800, 0.0f);
    audio.insert(audio.end(), tx.begin(), tx.end());
    audio.resize(audio.size() + 48000, 0.0f);
    ultra::ota_channel_core::SimulatedChannel channel;
    channel.setSeed(77);
    channel.configure(20.0f, ultra::ota_channel_core::ChannelType::AWGN);
    channel.transmitFromA(audio);
    const auto rx = channel.receiveForB(audio.size());

    StreamingDecoder decoder;
    decoder.setConnectedOFDMMode(WaveformMode::OFDM_CHIRP, ofdmGeometry(mod, rate), mod, rate);
    decoder.setKnownCFO(0.0f);
    decoder.expectFullOFDMAnchorOnce();
    decoder.setBurstWindowAnchorCallback([&](uint8_t epoch, uint16_t base_low) {
        ++out.calls;
        out.epoch = epoch;
        out.base_low = base_low;
    });
    for (size_t pos = 0; pos < rx.size(); pos += 4800) {
        const size_t len = std::min<size_t>(4800, rx.size() - pos);
        decoder.feedAudio(rx.data() + pos, len);
        decoder.processBuffer();
    }
    CHECK(decoder.descriptorsConsumedForTesting() == 1, "the descriptor is consumed at 20 dB");
    return out;
}

void testDecoderHandsTheAnchorOver() {
    const auto with = decodeDescriptor(anchor::pack(2, 0x1234));
    CHECK(with.calls == 1, "a descriptor with an anchor fires the callback once");
    CHECK(with.epoch == 2, "the decoded epoch is the sender's");
    CHECK(with.base_low == (0x1234 & anchor::kBaseMask), "the decoded base is the sender's");
    const auto without = decodeDescriptor(0);
    CHECK(without.calls == 0, "a descriptor without an anchor fires nothing");
}

// A sender and a receiver ARQ wired by hand: the test decides which frames arrive.
struct Link {
    SelectiveRepeatARQ tx;
    SelectiveRepeatARQ rx;
    std::vector<Bytes> sent;
    struct Sack { uint16_t base; uint32_t bitmap; uint8_t epoch; };
    std::vector<Sack> sacks;

    static ARQConfig config() {
        ARQConfig c;
        c.window_size = 4;
        return c;
    }
    Link() : tx(config()), rx(config()) {
        tx.setCallsigns("TX1", "RX1");
        rx.setCallsigns("RX1", "TX1");
        tx.setTransmitCallback([this](const Bytes& b) { sent.push_back(b); });
        rx.setTransmitCallback([](const Bytes&) {});
        rx.setEmitToneBurstSackCallback(
            [this](uint16_t base, uint32_t bitmap, bool, uint8_t epoch) {
                sacks.push_back({base, bitmap, epoch});
            });
        rx.setToneBurstSackTransportReady(true);
        rx.setDataReceivedCallback([](const Bytes&) {});
    }
    // Era 0: frames 0 and 1 delivered and acknowledged; then frames 2 and 3 sent and lost, and the
    // sender re-cuts its window (the rate step-down): era 1 resends seqs 2 and 3 with new content.
    void eraZeroThenStepDown() {
        tx.sendData(Bytes{0x01});
        tx.sendData(Bytes{0x02});
        rx.onFrameReceived(sent[0]);
        rx.onFrameReceived(sent[1]);
        rx.endGroupReceiveAndAck();
        const Sack s = sacks.back();
        CHECK(tx.onToneBurstAck(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the era-0 ACK is accepted");
        tx.sendData(Bytes{0x03});
        tx.sendData(Bytes{0x04});  // seqs 2, 3 in era 0: lost
        tx.setCodeRate(CodeRate::R1_2);
        CHECK(tx.txMoveEpoch() == 1, "the step-down re-cuts the window: era 1");
        tx.sendData(Bytes{0x13});
        tx.sendData(Bytes{0x14});  // seqs 2 (era head), 3 in era 1
        CHECK(sent.size() == 6, "six frames on the air");
    }
    // The descriptor-switch path: abortPendingTx abandons the frames in flight (their bytes are
    // re-queued under new seqs) and starts era 1 past them - the sender's base runs ahead of the
    // receiver's (GUI seed-44 drop: 9 against 5).
    void eraZeroThenAbandon() {
        tx.sendData(Bytes{0x01});
        tx.sendData(Bytes{0x02});
        rx.onFrameReceived(sent[0]);
        rx.onFrameReceived(sent[1]);
        rx.endGroupReceiveAndAck();
        const Sack s = sacks.back();
        CHECK(tx.onToneBurstAck(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the era-0 ACK is accepted");
        tx.sendData(Bytes{0x03});
        tx.sendData(Bytes{0x04});  // seqs 2, 3 in era 0: lost
        tx.abortPendingTx();
        CHECK(tx.txMoveEpoch() == 1, "the abort starts era 1");
        CHECK(tx.getTxBaseSeq() == 4, "the abort moves the base past the abandoned frames");
        tx.sendData(Bytes{0x13});
        tx.sendData(Bytes{0x14});  // seqs 4 (era head), 5 in era 1
        CHECK(sent.size() == 6, "six frames on the air");
    }
    uint16_t txBaseLow() const { return static_cast<uint16_t>(tx.getTxBaseSeq() & anchor::kBaseMask); }
};

void testFailedGroupAnswerReachesTheSender() {
    {
        Link l;
        l.eraZeroThenStepDown();
        l.rx.endGroupReceiveAndAck();  // the era-1 group decoded nothing
        const auto s = l.sacks.back();
        CHECK(s.epoch == 0, "repro: without the anchor the receiver answers in the old era");
        CHECK(!l.tx.isToneBurstAckPlausible(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "repro: the sender discards that answer");
    }
    {
        Link l;
        l.eraZeroThenStepDown();
        CHECK(l.rx.adoptPeerWindowAnchor(l.tx.txMoveEpoch(), l.txBaseLow()),
              "the era-1 descriptor anchors the receiver");
        CHECK(l.rx.getStats().descriptor_anchor_adoptions == 1, "one adoption counted");
        l.rx.endGroupReceiveAndAck();
        const auto s = l.sacks.back();
        CHECK(s.epoch == 1 && s.bitmap == 0, "the answer is in the sender's era, nothing decoded");
        CHECK(l.tx.isToneBurstAckPlausible(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the sender accepts it");
        CHECK(l.tx.onToneBurstAck(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the ARQ consumes it");
        CHECK(l.tx.lastAckProgressFrames() == 0, "a zero-progress round: the failed group's verdict");
        CHECK(l.tx.getAvailableSlots() == 2, "nothing retired: both era-1 frames still in flight");
    }
}

void testLostEraHeadKeepsTheAckFlow() {
    {
        Link l;
        l.eraZeroThenStepDown();
        l.rx.onFrameReceived(l.sent[5]);  // era-1 seq 3 only: the era head (seq 2) died
        const size_t before = l.sacks.size();
        l.rx.endGroupReceiveAndAck();
        CHECK(l.rx.rxWaitingRebase(), "repro: without the anchor the receiver is unanchored");
        CHECK(l.sacks.size() == before, "repro: and ACK-silent");
    }
    {
        Link l;
        l.eraZeroThenStepDown();
        CHECK(l.rx.adoptPeerWindowAnchor(l.tx.txMoveEpoch(), l.txBaseLow()), "anchored first");
        l.rx.onFrameReceived(l.sent[5]);
        l.rx.endGroupReceiveAndAck();
        CHECK(!l.rx.rxWaitingRebase(), "anchored: no interregnum");
        const auto s = l.sacks.back();
        CHECK(s.epoch == 1, "the ACK is in the sender's era");
        CHECK(l.tx.onToneBurstAck(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the sender accepts it");
        CHECK(l.tx.lastAckProgressFrames() == 1, "seq 3 is selectively acknowledged");
        CHECK(l.tx.countUnackedFrameIdentities({l.sent[4], l.sent[5]}) == 1,
              "only the era head is still unacknowledged");
    }
}

void testAbandonedFramesAnchorAhead() {
    {
        Link l;
        l.eraZeroThenAbandon();
        l.rx.endGroupReceiveAndAck();
        const auto s = l.sacks.back();
        CHECK(s.epoch == 0, "repro: the receiver answers the abandoned era");
        CHECK(!l.tx.isToneBurstAckPlausible(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "repro: the sender discards it");
    }
    {
        Link l;
        l.eraZeroThenAbandon();
        CHECK(l.rx.adoptPeerWindowAnchor(l.tx.txMoveEpoch(), l.txBaseLow()),
              "a base ahead of ours by the abandoned frames is the sender's");
        CHECK(l.rx.getRxBaseSeq() == 4, "the receiver now waits for the era head");
        l.rx.endGroupReceiveAndAck();
        const auto s = l.sacks.back();
        CHECK(s.epoch == 1 && s.bitmap == 0, "the answer is in the sender's era");
        CHECK(l.tx.onToneBurstAck(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the sender accepts it");
        CHECK(l.tx.lastAckProgressFrames() == 0, "a zero-progress round");
    }
    {
        Link l;
        l.eraZeroThenAbandon();
        CHECK(l.rx.adoptPeerWindowAnchor(l.tx.txMoveEpoch(), l.txBaseLow()), "anchored first");
        l.rx.onFrameReceived(l.sent[5]);  // era-1 seq 5 only: the era head died
        l.rx.endGroupReceiveAndAck();
        const auto s = l.sacks.back();
        CHECK(l.tx.onToneBurstAck(static_cast<uint8_t>(s.base & 0x3F), s.bitmap, s.epoch),
              "the sender accepts the ACK of the surviving frame");
        CHECK(l.tx.lastAckProgressFrames() == 1, "seq 5 is selectively acknowledged");
    }
}

void testImpossibleAnchorsRefused() {
    Link l;
    l.tx.sendData(Bytes{0x01});
    l.tx.sendData(Bytes{0x02});
    l.rx.onFrameReceived(l.sent[0]);
    l.rx.onFrameReceived(l.sent[1]);  // rx_base 2, era 0
    CHECK(!l.rx.adoptPeerWindowAnchor(0, 2), "an anchored era learns nothing from its own anchor");
    CHECK(!l.rx.adoptPeerWindowAnchor(1, 2 + 5),
          "one era change cannot abandon more than a window (window 4)");
    CHECK(!l.rx.adoptPeerWindowAnchor(1, static_cast<uint16_t>((2 - 5) & anchor::kBaseMask)),
          "a base more than a window behind cannot be the sender's");
    CHECK(l.rx.getStats().descriptor_anchor_rejected == 2, "both refusals counted");
    CHECK(l.rx.getRxBaseSeq() == 2, "a refused anchor moves nothing");
    CHECK(l.rx.adoptPeerWindowAnchor(2, 2 + 5), "two era changes may abandon up to two windows");
    CHECK(l.rx.getRxBaseSeq() == 7, "anchored at the sender's base");
    CHECK(!l.rx.adoptPeerWindowAnchor(2, 7), "and the same anchor again is a no-op");
}
}  // namespace

int main() {
    ultra::setLogLevel(ultra::LogLevel::ERROR);
    testWordCodec();
    testKnownBitsUnchanged();
    testDecoderHandsTheAnchorOver();
    testFailedGroupAnswerReachesTheSender();
    testLostEraHeadKeepsTheAckFlow();
    testAbandonedFramesAnchorAhead();
    testImpossibleAnchorsRefused();
    if (failures) {
        std::cout << "DescriptorWindowAnchor: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "DescriptorWindowAnchor: all checks passed\n";
    return 0;
}
