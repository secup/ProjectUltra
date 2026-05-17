#include "ota_channel_core/mixer.hpp"

#include <algorithm>
#include <utility>

namespace ultra::ota_channel_core {

void SampleIndexedMixer::clear() {
    blocks_.clear();
}

void SampleIndexedMixer::submit(std::string station_id,
                                uint64_t start_sample,
                                std::span<const float> samples) {
    if (samples.empty()) {
        return;
    }

    TxBlock block;
    block.station_id = std::move(station_id);
    block.start_sample = start_sample;
    block.samples.assign(samples.begin(), samples.end());

    auto pos = std::upper_bound(
        blocks_.begin(), blocks_.end(), block.start_sample,
        [](uint64_t start, const TxBlock& existing) {
            return start < existing.start_sample;
        });
    blocks_.insert(pos, std::move(block));
}

void SampleIndexedMixer::mixForReceiver(std::string_view receiver_id,
                                        uint64_t start_sample,
                                        size_t count,
                                        std::vector<float>& output) const {
    output.assign(count, 0.0f);
    const uint64_t end_sample = start_sample + count;

    for (const TxBlock& block : blocks_) {
        if (block.station_id == receiver_id) {
            continue;
        }
        const uint64_t block_end = block.start_sample + block.samples.size();
        if (block_end <= start_sample) {
            continue;
        }
        if (block.start_sample >= end_sample) {
            break;
        }

        const uint64_t overlap_begin = std::max(start_sample, block.start_sample);
        const uint64_t overlap_end = std::min(end_sample, block_end);
        const size_t out_offset = static_cast<size_t>(overlap_begin - start_sample);
        const size_t in_offset = static_cast<size_t>(overlap_begin - block.start_sample);
        const size_t n = static_cast<size_t>(overlap_end - overlap_begin);

        for (size_t i = 0; i < n; ++i) {
            output[out_offset + i] += block.samples[in_offset + i];
        }
    }
}

std::vector<float> SampleIndexedMixer::mixForReceiver(std::string_view receiver_id,
                                                      uint64_t start_sample,
                                                      size_t count) const {
    std::vector<float> output;
    mixForReceiver(receiver_id, start_sample, count, output);
    return output;
}

void SampleIndexedMixer::discardBefore(uint64_t sample_index) {
    while (!blocks_.empty()) {
        const TxBlock& block = blocks_.front();
        if (block.start_sample + block.samples.size() > sample_index) {
            break;
        }
        blocks_.pop_front();
    }
}

}  // namespace ultra::ota_channel_core
