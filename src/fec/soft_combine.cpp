#include "soft_combine.hpp"
#include "ultra/logging.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <limits>

namespace ultra::fec {

namespace {

bool harqDebugLogEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_HARQ_DEBUG_LOG");
        return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
    }();
    return enabled;
}

int harqDebugFilter(const char* name) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return -1;
    }
    char* end = nullptr;
    const long parsed = std::strtol(value, &end, 0);
    return (end && *end == '\0') ? static_cast<int>(parsed) : -1;
}

bool harqDebugKeySelected(const SoftCombineBuffer::Key& key) {
    if (!harqDebugLogEnabled()) {
        return false;
    }
    const int seq_filter = harqDebugFilter("ULTRA_HARQ_DEBUG_SEQ");
    if (seq_filter >= 0 && static_cast<int>(key.seq) != seq_filter) {
        return false;
    }
    const int cw_filter = harqDebugFilter("ULTRA_HARQ_DEBUG_CW");
    if (cw_filter >= 0 && static_cast<int>(key.cw_index) != cw_filter) {
        return false;
    }
    return true;
}

std::string firstLlrs(const std::vector<float>& llrs) {
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(2);
    const size_t n = std::min<size_t>(16, llrs.size());
    out << '[';
    for (size_t i = 0; i < n; ++i) {
        if (i != 0) {
            out << ',';
        }
        out << llrs[i];
    }
    if (llrs.size() > n) {
        out << ",...";
    }
    out << ']';
    return out.str();
}

float meanAbsLlr(const std::vector<float>& llrs) {
    if (llrs.empty()) {
        return 0.0f;
    }
    double sum = 0.0;
    for (float llr : llrs) {
        sum += std::abs(llr);
    }
    return static_cast<float>(sum / static_cast<double>(llrs.size()));
}

size_t signDisagreements(const std::vector<float>& a, const std::vector<float>& b) {
    const size_t n = std::min(a.size(), b.size());
    size_t disagreements = 0;
    for (size_t i = 0; i < n; ++i) {
        if ((a[i] < 0.0f) != (b[i] < 0.0f)) {
            ++disagreements;
        }
    }
    return disagreements;
}

void logHarqVector(const char* event, const SoftCombineBuffer::Key& key,
                   int attempts, const std::vector<float>& llrs) {
    if (!harqDebugKeySelected(key)) {
        return;
    }
    LOG_MODEM(WARN,
              "HARQ_DEBUG %s key={sender=0x%06X dst=0x%06X seq=%u rate=%d cw=%u/%u z=%u mod=%u ch_int=%u tail=%u geom=%u} "
              "attempts=%d len=%zu mean_abs=%.3f first16=%s",
              event, key.sender_hash, key.dst_hash, key.seq,
              static_cast<int>(key.rate),
              key.cw_index, key.cw_count, key.lifting_z, key.modulation,
              key.channel_interleave,
              key.physical_burst_end, key.carrier_count_hash, attempts, llrs.size(),
              meanAbsLlr(llrs),
              firstLlrs(llrs).c_str());
}

void logHarqCombineHit(const SoftCombineBuffer::Key& key, int attempts,
                       const std::vector<float>& retained,
                       const std::vector<float>& incoming,
                       const std::vector<float>& combined) {
    if (!harqDebugKeySelected(key)) {
        return;
    }
    LOG_MODEM(WARN,
              "HARQ_DEBUG combine_hit key={sender=0x%06X dst=0x%06X seq=%u rate=%d cw=%u/%u z=%u mod=%u ch_int=%u tail=%u geom=%u} "
              "attempts=%d len=%zu sign_disagree=%zu mean_abs_retained=%.3f mean_abs_new=%.3f "
              "mean_abs_sum=%.3f retained16=%s new16=%s sum16=%s",
              key.sender_hash, key.dst_hash, key.seq,
              static_cast<int>(key.rate), key.cw_index,
              key.cw_count, key.lifting_z, key.modulation, key.channel_interleave,
              key.physical_burst_end, key.carrier_count_hash,
              attempts, incoming.size(), signDisagreements(retained, incoming),
              meanAbsLlr(retained), meanAbsLlr(incoming), meanAbsLlr(combined),
              firstLlrs(retained).c_str(), firstLlrs(incoming).c_str(),
              firstLlrs(combined).c_str());
}

}  // namespace

SoftCombineBuffer::Key SoftCombineBuffer::makeKey(const HarqKeyInputs& in) {
    Key k;
    k.sender_hash = in.sender_hash;
    k.dst_hash = in.dst_hash;
    k.seq = in.seq;
    k.rate = in.rate;
    k.cw_count = in.cw_count;
    k.cw_index = in.cw_index;
    k.lifting_z = static_cast<uint8_t>(in.lifting_z == 81 ? 81 : 27);
    k.modulation = in.modulation;
    k.channel_interleave = static_cast<uint8_t>(in.channel_interleave ? 1 : 0);
    k.physical_burst_end = static_cast<uint8_t>(in.physical_burst_end ? 1 : 0);
    // Hash OFDM geometry into 16 bits. Two attempts with the same
    // (sender, seq, rate, modulation, interleave) but different
    // waveform modes (CHIRP vs COX vs NARROW) or different data-
    // carrier counts produce different LLR bit positions and must
    // not combine — a hash mismatch forces a fresh entry.
    uint32_t carrier_h = static_cast<uint32_t>(in.ofdm_data_carriers) << 8;
    carrier_h ^= static_cast<uint32_t>(in.waveform_mode);
    k.carrier_count_hash = static_cast<uint16_t>((carrier_h >> 8) ^ carrier_h);
    k.header_crc = in.header_crc;
    return k;
}

size_t SoftCombineBuffer::KeyHash::operator()(const Key& key) const {
    size_t h = static_cast<size_t>(key.sender_hash);
    h ^= static_cast<size_t>(key.dst_hash) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.seq) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.cw_count) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.cw_index) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.lifting_z) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.rate) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.modulation) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.channel_interleave) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.physical_burst_end) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.carrier_count_hash) + 0x9e3779b9u + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.header_crc) + 0x9e3779b9u + (h << 6) + (h >> 2);
    return h;
}

int SoftCombineBuffer::combine(const Key& key, const std::vector<float>& incoming_llrs,
                               std::vector<float>& out_llrs) {
    out_llrs = incoming_llrs;

    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || key.sender_hash == 0) {
        return 1;
    }

    auto it = entries_.find(key);
    if (it == entries_.end()) {
        logHarqVector("combine_miss_new", key, 1, incoming_llrs);
        return 1;
    }

    Entry& entry = it->second;
    if (entry.llrs.size() != incoming_llrs.size()) {
        if (harqDebugKeySelected(key)) {
            LOG_MODEM(WARN,
                      "HARQ_DEBUG combine_size_mismatch key={sender=0x%06X dst=0x%06X seq=%u cw=%u/%u} retained_len=%zu new_len=%zu",
                      key.sender_hash, key.dst_hash, key.seq, key.cw_index,
                      key.cw_count,
                      entry.llrs.size(), incoming_llrs.size());
        }
        eraseLocked(key);
        return 1;
    }

    // Chase combining: for independent observations of the same coded
    // bit, the joint LLR is the SUM, not the average. Two observations
    // give twice the |LLR| magnitude — that's exactly the SNR gain we
    // want from retransmission. Averaging would leave the magnitude
    // unchanged and effectively disable HARQ at the LDPC decoder.
    //
    // Saturation: cap |LLR| at kMaxAccumulatedLLR to keep the LDPC
    // decoder's float math well-conditioned across many retransmissions
    // (10+ attempts × |LLR|=8 = 80 is fine; 50+ attempts could approach
    // float-precision concerns and likely indicates a stuck retx loop
    // that should be killed at the ARQ layer).
    constexpr float kMaxAccumulatedLLR = 60.0f;
    const int next_attempts = std::max(1, entry.attempts) + 1;
    for (size_t i = 0; i < incoming_llrs.size(); ++i) {
        const float sum = entry.llrs[i] + incoming_llrs[i];
        out_llrs[i] = std::clamp(sum, -kMaxAccumulatedLLR, kMaxAccumulatedLLR);
    }

    logHarqCombineHit(key, next_attempts, entry.llrs, incoming_llrs, out_llrs);
    touchLocked(entry);
    return next_attempts;
}

void SoftCombineBuffer::retain(const Key& key, std::vector<float> combined_llrs,
                               bool provisional) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || key.sender_hash == 0 || max_entries_ == 0 || combined_llrs.empty()) {
        return;
    }

    auto it = entries_.find(key);
    int attempts = 1;
    // Provisional discipline (2026-07-01): a predicted-key entry keeps its
    // accumulated age across re-retains (hard TTL — a misprediction must not
    // live forever on retry refreshes), and once an entry has been touched
    // under a header-verified key it stays real.
    uint32_t carried_age_ms = 0;
    bool entry_provisional = provisional;
    if (it != entries_.end()) {
        attempts = std::max(1, it->second.attempts) + 1;
        entry_provisional = it->second.provisional && provisional;
        if (entry_provisional) {
            carried_age_ms = it->second.age_ms;
        }
        eraseLocked(key);
    } else {
        while (entries_.size() >= max_entries_ && !lru_.empty()) {
            // Evict provisional entries FIRST so a misprediction can never
            // displace a header-verified accumulation (capacity is sized to
            // exactly the legitimate window x cw population).
            auto victim = lru_.end();
            for (auto lit = lru_.begin(); lit != lru_.end(); ++lit) {
                auto eit = entries_.find(*lit);
                if (eit != entries_.end() && eit->second.provisional) {
                    victim = lit;  // keep scanning: oldest provisional = closest to back
                }
            }
            eraseLocked(victim != lru_.end() ? *victim : lru_.back());
        }
    }

    lru_.push_front(key);
    Entry entry;
    entry.llrs = std::move(combined_llrs);
    entry.age_ms = carried_age_ms;
    entry.attempts = attempts;
    entry.provisional = entry_provisional;
    entry.lru_it = lru_.begin();
    entries_.emplace(key, std::move(entry));
    logHarqVector("retain", key, attempts, entries_.find(key)->second.llrs);
    evictOverflowLocked();
}

void SoftCombineBuffer::drop(const Key& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (harqDebugKeySelected(key)) {
        LOG_MODEM(WARN,
                  "HARQ_DEBUG drop key={sender=0x%06X dst=0x%06X seq=%u rate=%d cw=%u/%u z=%u mod=%u ch_int=%u tail=%u geom=%u}",
                  key.sender_hash, key.dst_hash, key.seq,
                  static_cast<int>(key.rate),
                  key.cw_index, key.cw_count, key.lifting_z, key.modulation,
                  key.channel_interleave,
                  key.physical_burst_end, key.carrier_count_hash);
    }
    eraseLocked(key);
}

void SoftCombineBuffer::retainOnlySeqWindow(uint16_t base_seq, size_t window_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (window_size == 0) {
        entries_.clear();
        lru_.clear();
        return;
    }

    for (auto it = entries_.begin(); it != entries_.end();) {
        const uint16_t diff = (it->first.seq - base_seq) & 0xFFFF;
        if (diff >= window_size) {
            lru_.erase(it->second.lru_it);
            it = entries_.erase(it);
        } else {
            ++it;
        }
    }
}

void SoftCombineBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    entries_.clear();
    lru_.clear();
}

void SoftCombineBuffer::tick(uint32_t elapsed_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (entries_.empty()) {
        return;
    }

    for (auto& kv : entries_) {
        Entry& entry = kv.second;
        if (elapsed_ms > std::numeric_limits<uint32_t>::max() - entry.age_ms) {
            entry.age_ms = std::numeric_limits<uint32_t>::max();
        } else {
            entry.age_ms += elapsed_ms;
        }
    }
    evictExpiredLocked();
}

void SoftCombineBuffer::setTTL(uint32_t ttl_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    ttl_ms_ = ttl_ms;
    evictExpiredLocked();
}

void SoftCombineBuffer::setMaxEntries(size_t n) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_entries_ = n;
    evictOverflowLocked();
}

void SoftCombineBuffer::setEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
    if (!enabled_) {
        entries_.clear();
        lru_.clear();
    }
}

size_t SoftCombineBuffer::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

bool SoftCombineBuffer::enabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

void SoftCombineBuffer::touchLocked(Entry& entry) {
    lru_.splice(lru_.begin(), lru_, entry.lru_it);
    entry.lru_it = lru_.begin();
}

void SoftCombineBuffer::eraseLocked(const Key& key) {
    auto it = entries_.find(key);
    if (it == entries_.end()) {
        return;
    }
    lru_.erase(it->second.lru_it);
    entries_.erase(it);
}

void SoftCombineBuffer::evictExpiredLocked() {
    if (ttl_ms_ == 0) {
        entries_.clear();
        lru_.clear();
        return;
    }

    for (auto it = entries_.begin(); it != entries_.end();) {
        if (it->second.age_ms >= ttl_ms_) {
            lru_.erase(it->second.lru_it);
            it = entries_.erase(it);
        } else {
            ++it;
        }
    }
}

void SoftCombineBuffer::evictOverflowLocked() {
    if (max_entries_ == 0) {
        entries_.clear();
        lru_.clear();
        return;
    }

    while (entries_.size() > max_entries_ && !lru_.empty()) {
        eraseLocked(lru_.back());
    }
}

} // namespace ultra::fec
