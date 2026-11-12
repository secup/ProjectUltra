#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <vector>

namespace ultra {
namespace sync {

// SyncRingBuffer — the shared 48 kHz audio ring + its search/floor/noise bookkeeping, extracted
// verbatim from StreamingDecoder (§7 SyncController refactor, Phase 1: cohesive object, still
// homed in the decoder). Owns the circular sample buffer, the write cursor, the correlation
// search cursor, the post-frame search floor, the running noise-floor estimate, and the
// buffer_mutex_/data_cv_ pair that guards them. The *Locked helpers require the caller to hold
// buffer_mutex_. Byte-identical to the former decoder members — only the home moved.
//
// NOTE (transitional): every member is public during the migration because the decoder still
// reads/writes the ring fields directly (the old member names, now reached as ring_.X). They get
// re-privatized once detect()/the decode loop route through methods. Do not add new external
// writers in the meantime.
class SyncRingBuffer {
public:
    static constexpr size_t kDefaultBufferSamples = 2400000; // 50 seconds at 48 kHz
    static constexpr size_t kMinimumBufferSamples = 120000;  // Must cover largest sync search window

    explicit SyncRingBuffer(size_t capacity_samples = kDefaultBufferSamples);

    // Ring/absolute sample helpers. Call only while buffer_mutex_ is held.
    size_t wrapCustomRingIndexLocked(size_t value) const;
    size_t wrapRingIndexLocked(size_t value) const {
        if (uses_default_buffer_capacity_) {
            return value % kDefaultBufferSamples;
        }
        return wrapCustomRingIndexLocked(value);
    }
    void writeSamplesToRingLocked(const float* samples, size_t count);
    size_t ringPosToAbsoluteLocked(size_t ring_pos) const;
    size_t absoluteToRingLocked(size_t abs_pos) const;
    void setSearchFloorLocked(size_t abs_pos);

    // --- transitional public state ----------------------------------------------------------
    // Circular buffer for audio samples
    std::vector<float> buffer_;
    const size_t buffer_capacity_samples_;
    const bool uses_default_buffer_capacity_;
    size_t write_pos_ = 0;            // Next position to write (only pointer we need)
    mutable std::mutex buffer_mutex_;
    std::condition_variable data_cv_;
    size_t correlation_pos_ = 0;      // Current position for correlation search
    size_t search_floor_abs_ = 0;     // Earliest absolute sample search may inspect
    bool search_floor_abs_valid_ = false;
    float noise_floor_ = 0.001f;
    // Total samples fed (per-instance). Written under the buffer mutex by the audio thread; read
    // under it for every buffer computation, and without it only for logging and progress (the
    // decode thread's search log raced: ThreadSanitizer, EngineConcurrency) - atomic for those.
    std::atomic<size_t> total_fed_{0};

private:
    static size_t validateCapacity(size_t capacity);
};

}  // namespace sync
}  // namespace ultra
