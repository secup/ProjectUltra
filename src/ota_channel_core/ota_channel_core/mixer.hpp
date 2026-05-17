#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <span>
#include <string>
#include <vector>

namespace ultra::ota_channel_core {

struct TxBlock {
    std::string station_id;
    uint64_t start_sample = 0;
    std::vector<float> samples;
};

class SampleIndexedMixer {
public:
    void clear();
    void submit(std::string station_id, uint64_t start_sample, std::span<const float> samples);
    void mixForReceiver(std::string_view receiver_id,
                        uint64_t start_sample,
                        size_t count,
                        std::vector<float>& output) const;
    std::vector<float> mixForReceiver(std::string_view receiver_id,
                                      uint64_t start_sample,
                                      size_t count) const;
    void discardBefore(uint64_t sample_index);
    size_t pendingBlocks() const { return blocks_.size(); }

private:
    std::deque<TxBlock> blocks_;
};

}  // namespace ultra::ota_channel_core
