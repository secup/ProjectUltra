#include "sync/sync_ring_buffer.hpp"

#include <algorithm>
#include <stdexcept>

namespace ultra {
namespace sync {

size_t SyncRingBuffer::validateCapacity(size_t capacity) {
    if (capacity < kMinimumBufferSamples) {
        throw std::invalid_argument("StreamingDecoder buffer capacity is smaller than the sync search window");
    }
    return capacity;
}

SyncRingBuffer::SyncRingBuffer(size_t capacity_samples)
    : buffer_capacity_samples_(validateCapacity(capacity_samples)),
      uses_default_buffer_capacity_(buffer_capacity_samples_ == kDefaultBufferSamples) {
    buffer_.resize(buffer_capacity_samples_, 0.0f);
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#endif
size_t SyncRingBuffer::wrapCustomRingIndexLocked(size_t value) const {
    return value % buffer_capacity_samples_;
}

void SyncRingBuffer::writeSamplesToRingLocked(const float* samples, size_t count) {
    if (uses_default_buffer_capacity_) {
        for (size_t i = 0; i < count; i++) {
            buffer_[write_pos_] = samples[i];
            write_pos_ = (write_pos_ + 1) % kDefaultBufferSamples;
        }
        return;
    }

    const size_t capacity = buffer_capacity_samples_;
    for (size_t i = 0; i < count; i++) {
        buffer_[write_pos_] = samples[i];
        write_pos_ = (write_pos_ + 1) % capacity;
    }
}

size_t SyncRingBuffer::ringPosToAbsoluteLocked(size_t ring_pos) const {
    if (total_fed_ < buffer_capacity_samples_) {
        return ring_pos;
    }

    const size_t oldest_abs = total_fed_ - buffer_capacity_samples_;
    const size_t oldest_pos = write_pos_;
    const size_t offset = (ring_pos >= oldest_pos)
        ? (ring_pos - oldest_pos)
        : (buffer_capacity_samples_ - oldest_pos + ring_pos);
    return oldest_abs + offset;
}

size_t SyncRingBuffer::absoluteToRingLocked(size_t abs_pos) const {
    if (total_fed_ < buffer_capacity_samples_) {
        return wrapRingIndexLocked(std::min(abs_pos, total_fed_.load()));
    }

    const size_t oldest_abs = total_fed_ - buffer_capacity_samples_;
    abs_pos = std::clamp(abs_pos, oldest_abs, total_fed_.load());
    return wrapRingIndexLocked(write_pos_ + (abs_pos - oldest_abs));
}

void SyncRingBuffer::setSearchFloorLocked(size_t abs_pos) {
    const size_t oldest_abs = (total_fed_ > buffer_capacity_samples_)
        ? (total_fed_ - buffer_capacity_samples_)
        : 0;
    search_floor_abs_ = std::clamp(abs_pos, oldest_abs, total_fed_.load());
    search_floor_abs_valid_ = true;
}

}  // namespace sync
}  // namespace ultra
