#pragma once
// Waveform agreement state (docs/LINK_STATE_MACHINE_DESIGN_2026_11_24.md, slice 3).
//
// What this station knows about the waveform the PEER decodes, during and after a waveform
// crossing. Until 2026-11-24 this was nine loose Connection fields (crossing_probe_active_,
// crossing_probe_rx_side_, crossing_return_mod_, crossing_probe_revert_ms_, crossing_launch_ack_*,
// rx_crossing_apply_pending_, rx_crossing_word_, rx_dual_up_commanded_) written at ~30 sites; three
// of the 2026-11-24 bugs were a decision that never consulted them. Here each is a named transition
// and the combinations no state allows cannot be written.
//
// Pure state, no I/O and no clock (the caller passes elapsed time). Behaviour-identical to the fields
// it replaces: every transition writes what the old site wrote, nothing more.
#include <cstdint>

#include "ultra/types.hpp"

namespace ultra {
namespace protocol {

// An up-crossing (MC-DPSK -> OFDM) on probation until the peer is seen to follow.
class CrossingProbation {
public:
    enum class Side : uint8_t {
        None,      // no crossing on probation
        Sender,    // this end crossed up on the receiver's command; the peer has not yet ACKed OFDM
                   // data - its PRIMARY decoder is still MC-DPSK at returnMod()
        Receiver,  // this end commanded the crossing and crossed; waiting for the first OFDM burst
    };

    Side side() const { return side_; }
    bool active() const { return side_ != Side::None; }
    bool sender() const { return side_ == Side::Sender; }
    bool receiver() const { return side_ == Side::Receiver; }
    // The MC-DPSK rung the crossing left. Kept after the probation ends (as the field it replaces).
    Modulation returnMod() const { return return_mod_; }

    void beginSender(Modulation left) {
        side_ = Side::Sender;
        revert_ms_ = 0;
        return_mod_ = left;
    }
    void beginReceiver(Modulation left, uint32_t revert_ms) {
        side_ = Side::Receiver;
        return_mod_ = left;
        revert_ms_ = revert_ms;
    }
    void end() {
        side_ = Side::None;
        launch_ack_valid_ = false;
        revert_ms_ = 0;
    }
    void reset() {
        end();
        launch_ack_seq_ = 0;
        launch_ack_mask_ = 0;
    }

    // Receiver side: time left for the sender's first OFDM burst to show.
    uint32_t revertMs() const { return revert_ms_; }
    // Count down; true while time remains (the old `revert > elapsed` branch).
    bool consumeRevert(uint32_t elapsed_ms) {
        if (revert_ms_ > elapsed_ms) {
            revert_ms_ -= elapsed_ms;
            return true;
        }
        return false;
    }

    // Sender side: the tone ACK that launched the crossing (its repeats must not open a second group).
    void markLaunchAck(uint8_t group_seq, uint16_t frame_mask) {
        launch_ack_valid_ = true;
        launch_ack_seq_ = group_seq;
        launch_ack_mask_ = frame_mask;
    }
    bool launchAckValid() const { return launch_ack_valid_; }
    uint8_t launchAckSeq() const { return launch_ack_seq_; }
    uint16_t launchAckMask() const { return launch_ack_mask_; }
    void clearLaunchAck() { launch_ack_valid_ = false; }

private:
    Side side_ = Side::None;
    Modulation return_mod_ = Modulation::DBPSK;
    uint32_t revert_ms_ = 0;
    bool launch_ack_valid_ = false;
    uint8_t launch_ack_seq_ = 0;
    uint16_t launch_ack_mask_ = 0;
};

// The crossing command this station, as receiver, stamps on its tone ACKs.
class RxCrossingCommand {
public:
    uint8_t word() const { return word_; }
    // Not dual-listen: the command waits for its ACK to go out, then this end crosses.
    bool pending() const { return pending_; }
    // Dual-listen: an up-command rides every ACK; this end crosses when OFDM arrives.
    bool upCommanded() const { return up_commanded_; }

    void setWord(uint8_t word) { word_ = word; }
    void stampPending(uint8_t word) {
        word_ = word;
        pending_ = true;
    }
    void markUpCommanded() { up_commanded_ = true; }
    void clearUpCommanded() { up_commanded_ = false; }
    // The command was applied or withdrawn (the up-command flag is separate).
    void clearPending() {
        pending_ = false;
        word_ = 0;
    }
    void reset() {
        word_ = 0;
        pending_ = false;
        up_commanded_ = false;
    }

private:
    uint8_t word_ = 0;
    bool pending_ = false;
    bool up_commanded_ = false;
};

}  // namespace protocol
}  // namespace ultra
