#pragma once

// The receive decoders of a ModemEngine (docs/DUAL_LISTEN_DESIGN_2026_11_11.md).
//
// One decoder runs the session's current waveform: the PRIMARY, which every existing engine call
// addresses through `->` and the boolean test (so `streaming_decoder_->x()` keeps meaning what it
// always meant). With dual-listen a second decoder, the LISTENER, runs the other waveform. A waveform
// change flips which slot is primary; neither decoder is rebuilt, so neither loses its sync, CFO,
// clock-drift or warm state. Both objects live as long as the engine, so a reader on another thread
// that loaded the previous primary still holds a live decoder. Each decode thread is bound to a slot
// (an object), never to a role.

#include "streaming_decoder.hpp"

#include <atomic>
#include <memory>

namespace ultra {
namespace gui {

class RxDecoderPair {
public:
    StreamingDecoder* primary() const {
        return slots_[primary_.load(std::memory_order_acquire)].get();
    }
    // The decoder on the other waveform while dual-listen runs, else nullptr.
    StreamingDecoder* listener() const {
        if (!listener_active_.load(std::memory_order_acquire)) {
            return nullptr;
        }
        return slots_[1 - primary_.load(std::memory_order_acquire)].get();
    }
    StreamingDecoder* operator->() const { return primary(); }
    explicit operator bool() const { return primary() != nullptr; }

    StreamingDecoder* slot(int index) const { return slots_[index & 1].get(); }
    bool isPrimary(const StreamingDecoder* decoder) const {
        return decoder != nullptr && decoder == primary();
    }
    bool listenerActive() const { return listener_active_.load(std::memory_order_acquire); }

    void setSlot(int index, std::unique_ptr<StreamingDecoder> decoder) {
        slots_[index & 1] = std::move(decoder);
    }
    void setListenerActive(bool active) {
        listener_active_.store(active && slots_[0] && slots_[1], std::memory_order_release);
    }
    // The listener becomes the primary and the primary the listener.
    void flip() {
        if (slots_[0] && slots_[1]) {
            primary_.fetch_xor(1, std::memory_order_acq_rel);
        }
    }
    int primaryIndex() const { return primary_.load(std::memory_order_acquire); }

private:
    std::unique_ptr<StreamingDecoder> slots_[2];
    std::atomic<int> primary_{0};
    std::atomic<bool> listener_active_{false};
};

}  // namespace gui
}  // namespace ultra
