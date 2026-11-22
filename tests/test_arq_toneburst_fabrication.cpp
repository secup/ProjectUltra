/**
 * BUG-TONEACK-FABRICATION regression suite (F116, 2026-07-05)
 *
 * F116 (IONOS rig, 50KB): a stale-audio re-decode at the wrong tone-ACK
 * symbol_ms rung fluked Costas+Hamming+CRC-12 into a phantom detection
 * (group_seq6=5). SelectiveRepeatARQ::onToneBurstAck nearest-mapped the 6-bit
 * value onto the seq space (base=69, a seq NEVER SENT), and the cumulative
 * walk retired all 6 undelivered in-flight frames — firing on_send_complete
 * (true) per frame, irreversibly popping the FileTransfer TX ledger. Bytes
 * 34944..38688 were silently lost; the receiver waited at expected=34944
 * forever while both ARQ ends stayed "consistent".
 *
 * Invariant under test: an ack can only reference seqs actually transmitted —
 * [tx_base-1, tx_next-1]. Anything outside decodes as corrupt and is DROPPED
 * (== ack loss, RTO-recoverable). on_send_complete(true) never fires for a
 * frame the receiver did not confirm, and tx_base never passes tx_next.
 */

#include "protocol/arq_interface.hpp"
#include "protocol/selective_repeat_arq.hpp"
#include "protocol/frame_v2.hpp"
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

using namespace ultra;
using namespace ultra::protocol;

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { std::cout << "  Testing " << name << "... " << std::flush; tests_run++; } while (0)
#define PASS() \
    do { std::cout << "PASS\n"; tests_passed++; } while (0)
#define FAIL(msg) \
    do { std::cout << "FAIL: " << msg << "\n"; return false; } while (0)

namespace {

struct Harness {
    SelectiveRepeatARQ arq;
    int completes_ok = 0;
    int frames_txed = 0;

    explicit Harness(size_t window = 16) : arq(makeConfig(window)) {
        arq.setCallsigns("TX1", "RX1");
        arq.setTransmitCallback([this](const Bytes&) { frames_txed++; });
        arq.setSendCompleteCallback([this](bool ok) {
            if (ok) completes_ok++;
        });
    }

    static ARQConfig makeConfig(size_t window) {
        ARQConfig c;
        c.window_size = window;
        c.ack_timeout_ms = 60000;  // no RTO interference during the test
        c.sack_delay_ms = 100;
        return c;
    }

    // Queue n data frames (seqs tx_next..tx_next+n-1), nothing delivered.
    bool sendFrames(int n) {
        for (int i = 0; i < n; i++) {
            Bytes payload = {static_cast<uint8_t>(i), 0x55, 0xAA};
            if (!arq.sendData(payload)) return false;
        }
        return true;
    }

    // Deliver a cumulative control-frame ACK through seq (receiver-honest path).
    void ackThrough(uint16_t seq) {
        auto ack = v2::ControlFrame::makeNack("RX1", "TX1", seq, 0);
        ack.type = v2::FrameType::ACK;
        arq.onFrameReceived(ack.serialize());
    }

    // Walk the TX window to the given base by sending+acking in window chunks.
    bool walkBaseTo(int target) {
        for (int base = 0; base < target; base += 8) {
            const int n = std::min(8, target - base);
            if (!sendFrames(n)) return false;
            ackThrough(static_cast<uint16_t>(base + n - 1));
        }
        return arq.getTxBaseSeq() == static_cast<uint16_t>(target);
    }
};

}  // namespace

// F116 exact reproduction: 6 in flight at base 57, phantom tone ack
// group_seq6=5 (decodes 13 ahead of ref — outside the 6-seq support) must be
// dropped with zero fabricated completes.
static bool test_f116_phantom_dropped() {
    TEST("F116 phantom tone ack (outside sent window) dropped");

    Harness h;
    if (!h.walkBaseTo(57)) FAIL("setup: could not walk base to 57");
    const int completes_before = h.completes_ok;

    if (!h.sendFrames(6)) FAIL("setup: could not queue 6 frames");
    if (h.arq.onToneBurstAck(/*group_seq6=*/5, /*bitmap=*/0x7E02,
                             /*move_epoch=*/0))
        FAIL("outside-support tone ack must report rejected to its caller");

    if (h.completes_ok != completes_before)
        FAIL("fabricated ack fired on_send_complete(true) for undelivered frames");
    if (h.arq.getTxBaseSeq() != 57)
        FAIL("fabricated ack advanced tx_base");
    if (h.arq.getStats().fabricated_acks_dropped < 1)
        FAIL("fabricated_acks_dropped not counted");
    PASS();
    return true;
}

// Property sweep: every 6-bit value against a 6-frame window at base 57 —
// completes never exceed what a receiver could legitimately confirm, and
// tx_base never passes tx_next (= 63).
static bool test_fabrication_property_sweep() {
    TEST("all 64 group_seq6 values: no over-retire, base never passes next");

    for (int g = 0; g < 64; g++) {
        Harness h;
        if (!h.walkBaseTo(57)) FAIL("setup: could not walk base to 57");
        const int completes_before = h.completes_ok;
        if (!h.sendFrames(6)) FAIL("setup: could not queue 6 frames");

        h.arq.onToneBurstAck(static_cast<uint8_t>(g), 0, 0);

        const int retired = h.completes_ok - completes_before;
        // Legit forward distance from ref=56: delta = (g - 56) mod 64; valid <= 6.
        const int delta = (g - (56 & 0x3F)) & 0x3F;
        const int max_legit = (delta <= 6) ? delta : 0;
        if (retired > max_legit)
            FAIL("group_seq6=" + std::to_string(g) + " retired " +
                 std::to_string(retired) + " > legit " + std::to_string(max_legit));
        const uint16_t base_now = h.arq.getTxBaseSeq();
        const uint16_t fwd_of_next = static_cast<uint16_t>((base_now - 63) & 0xFFFF);
        if (fwd_of_next != 0 && fwd_of_next < 0x8000)
            FAIL("group_seq6=" + std::to_string(g) + " pushed base past tx_next");
    }
    PASS();
    return true;
}

// A fabricated CONTROL-frame SACK (full 16-bit seq beyond tx_next but inside
// the legacy window+1 Future bound) is also dropped by the never-sent guard.
static bool test_control_sack_beyond_sent_dropped() {
    TEST("control SACK beyond highest sent seq dropped");

    Harness h;
    if (!h.sendFrames(6)) FAIL("setup: could not queue 6 frames");  // 0..5, next=6
    h.ackThrough(12);  // window 16 -> forward 13 <= 17 passes the old Future guard

    if (h.completes_ok != 0) FAIL("fabricated control SACK retired frames");
    if (h.arq.getTxBaseSeq() != 0) FAIL("fabricated control SACK advanced base");
    if (h.arq.getStats().fabricated_acks_dropped < 1)
        FAIL("fabricated_acks_dropped not counted for control path");
    PASS();
    return true;
}

static bool test_precommit_plausibility_matches_sent_support() {
    TEST("pre-commit plausibility rejects impossible seq and bitmap claims");

    Harness h;
    if (!h.sendFrames(6)) FAIL("setup: could not queue 6 frames");  // 0..5

    // Current support is cumulative base-1=65535 through highest-sent=5.
    if (!h.arq.isToneBurstAckPlausible(63, 0x003Fu, 0))
        FAIL("base-aligned bitmap for six sent frames should be plausible");
    if (!h.arq.isToneBurstAckPlausible(2, 0x0007u, 0))
        FAIL("partial cumulative ACK plus three remaining SACK bits should be plausible");
    if (h.arq.isToneBurstAckPlausible(10, 0, 0))
        FAIL("out-of-support group_seq passed the pre-commit gate");
    if (h.arq.isToneBurstAckPlausible(2, 0x0008u, 0))
        FAIL("bitmap claim beyond highest sent frame passed the pre-commit gate");

    // Defense in depth: the stateful path must reject the same support-valid seq
    // with an impossible bitmap and leave all payload outstanding.
    if (h.arq.onToneBurstAck(2, 0x0008u, 0))
        FAIL("stateful path accepted bitmap claim for an unsent frame");
    if (h.completes_ok != 0 || h.arq.getTxBaseSeq() != 0)
        FAIL("impossible bitmap changed ARQ delivery state");
    PASS();
    return true;
}

// Safety-preserving: legitimate tone acks still retire normally.
static bool test_legit_tone_acks_still_work() {
    TEST("legitimate tone acks retire exactly the delivered frames");

    Harness h;
    if (!h.sendFrames(6)) FAIL("setup: could not queue 6 frames");  // seqs 0..5
    // Receiver delivered everything: base-1 = 5 -> group_seq6 = 5.
    if (!h.arq.onToneBurstAck(5, 0, 0))
        FAIL("legitimate full-window tone ack must report accepted");
    if (h.completes_ok != 6) FAIL("full-window tone ack failed to retire 6");
    if (h.arq.getTxBaseSeq() != 6) FAIL("base did not advance to 6");

    // No-progress ack (base-1 = 5 again) after the advance: decodes to delta 0
    // = valid no-op, never a fabrication drop.
    const int dropped_before = h.arq.getStats().fabricated_acks_dropped;
    if (!h.arq.onToneBurstAck(5, 0, 0))
        FAIL("valid no-progress duplicate tone ack must remain accepted");
    if (h.arq.getStats().fabricated_acks_dropped != dropped_before)
        FAIL("no-progress tone ack wrongly dropped as fabricated");
    if (h.arq.getTxBaseSeq() != 6) FAIL("no-progress ack moved base");

    // Partial progress: send 4 more (6..9), receiver confirms through 7.
    if (!h.sendFrames(4)) FAIL("setup: could not queue 4 frames");
    if (!h.arq.onToneBurstAck(7 & 0x3F, 0, 0))
        FAIL("legitimate partial-progress tone ack must report accepted");
    if (h.completes_ok != 8) FAIL("partial tone ack retired wrong count");
    if (h.arq.getTxBaseSeq() != 8) FAIL("partial tone ack wrong base");
    PASS();
    return true;
}

// BUG-ARQ-WINDOW-SHRINK-ACK-DROP (2026-11-22): a rate step down (DBPSK -> 16-FSK) shrinks
// the timing-derived window 3 -> 1 with three frames still in flight. The receiver's
// cumulative ACK through the last of them is legitimate (every seq in it was SENT), but the
// far-future guard measured it against the NEW window and dropped it on every resend until
// the frame ran out of retries (GUI ITU Good 0-3 dB: 4/16 runs "max retries exceeded" while
// every ACK was heard).
static bool test_ack_after_window_shrink_accepted() {
    TEST("ACK through in-flight frames accepted after the window shrinks");

    Harness h(8);
    if (!h.walkBaseTo(22)) FAIL("setup: could not walk base to 22");
    h.arq.setWindowSize(3);
    if (!h.sendFrames(3)) FAIL("setup: could not queue 3 frames");  // seqs 22..24
    h.arq.setWindowSize(1);
    const int completes_before = h.completes_ok;
    if (!h.arq.onToneBurstAck(/*group_seq6=*/24, /*bitmap=*/0, /*move_epoch=*/0))
        FAIL("ACK through the sent frames must be accepted after the window shrinks");
    if (h.arq.getTxBaseSeq() != 25) FAIL("base did not advance to 25");
    if (h.completes_ok != completes_before + 3) FAIL("ACK did not retire the 3 in-flight frames");
    if (h.arq.getStats().future_acks_ignored != 0) FAIL("ACK counted as far-future");
    PASS();
    return true;
}

// Same shrink, the timeout side: a resend after the shrink carries ONE window (the burst
// length the receiver now expects), not every outstanding frame. A 3-frame 16-FSK resend
// ran ~35 s; the receiver ended its turn after frame 1 and ACKed into the sender's own
// transmission (half-duplex: never heard), every round (GUI Good 0 dB seed 71).
static bool test_timeout_resend_after_shrink_is_one_window() {
    TEST("timeout resend after the window shrinks carries one window");

    Harness h(8);
    if (!h.walkBaseTo(22)) FAIL("setup: could not walk base to 22");
    h.arq.setWindowSize(3);
    if (!h.sendFrames(3)) FAIL("setup: could not queue 3 frames");  // seqs 22..24
    h.arq.setWindowSize(1);
    const int txed_before = h.frames_txed;
    h.arq.tick(61000);  // past the 60 s ack timeout
    const int resent = h.frames_txed - txed_before;
    if (resent != 1) FAIL("resent " + std::to_string(resent) + " frames, want 1");
    PASS();
    return true;
}

int main() {
    std::cout << "BUG-TONEACK-FABRICATION regression suite\n";
    bool ok = true;
    ok &= test_f116_phantom_dropped();
    ok &= test_fabrication_property_sweep();
    ok &= test_control_sack_beyond_sent_dropped();
    ok &= test_precommit_plausibility_matches_sent_support();
    ok &= test_legit_tone_acks_still_work();
    ok &= test_ack_after_window_shrink_accepted();
    ok &= test_timeout_resend_after_shrink_is_one_window();
    std::cout << tests_passed << "/" << tests_run << " passed\n";
    return (ok && tests_passed == tests_run) ? 0 : 1;
}
