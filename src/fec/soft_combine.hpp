#pragma once

#include "ultra/types.hpp"
#include <cstddef>
#include <cstdint>
#include <list>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace ultra::fec {

class SoftCombineBuffer {
public:
    struct Key {
        uint32_t sender_hash = 0;
        uint32_t dst_hash = 0;
        uint16_t seq = 0;
        CodeRate rate = CodeRate::R1_4;
        uint8_t cw_count = 0;
        uint8_t cw_index = 0;
        // LDPC lifting changes both the LLR vector length and bit geometry.
        // It is part of the physical retransmission identity, not metadata.
        uint8_t lifting_z = 27;
        // PHY parameters that change LLR scaling and bit ordering. If
        // any of these differ between attempts, stored LLRs aren't
        // comparable and combining would corrupt the decode. Including
        // them in the key forces a fresh entry instead of mis-combining.
        Modulation modulation = Modulation::DQPSK;
        uint8_t channel_interleave = 0;  // 0 = off, 1 = on
        // A physical-tail DATA retry is not byte-identical to a non-tail retry
        // of the same ARQ seq (header CRC and frame CRC both change). Partition
        // their LLR histories so regrouping can never corrupt chase combining.
        uint8_t physical_burst_end = 0;
        // Hash of (waveform_mode, ofdm_data_carriers). Bits-per-symbol
        // is implied by the modulation field above; pilot spacing
        // changes within the same waveform mode are NOT distinguished
        // here — that's a known gap (Codex review of commit 1f18683).
        // Today's modes (CHIRP/COX/NARROW) each have a fixed pilot
        // layout, so this is sufficient in practice; revisit if a
        // dynamic-pilot mode is added later.
        uint16_t carrier_count_hash = 0;
        // Whole-frame (turbo) entries only: the frame's 16-bit header CRC. A turbo frame
        // is ONE codeword over the whole info block, so ANY header change (flags, length,
        // tail status) is a different codeword and must never chase-combine. 0 for every
        // LDPC per-codeword key (unchanged behaviour).
        uint16_t header_crc = 0;

        bool operator==(const Key& other) const {
            return sender_hash == other.sender_hash &&
                   dst_hash == other.dst_hash &&
                   seq == other.seq &&
                   rate == other.rate &&
                   cw_count == other.cw_count &&
                   cw_index == other.cw_index &&
                   lifting_z == other.lifting_z &&
                   modulation == other.modulation &&
                   channel_interleave == other.channel_interleave &&
                   physical_burst_end == other.physical_burst_end &&
                   carrier_count_hash == other.carrier_count_hash &&
                   header_crc == other.header_crc;
        }
    };
    // cw_index marking a whole-frame (one turbo codeword) entry; LDPC keys use 0..15.
    static constexpr uint8_t kWholeFrameCwIndex = 0xFE;

    // A decoded CW0 is the authority for the provisional key's protected DATA
    // identity. Compare every header-visible key field that can change protected
    // bits, not only seq: a sender can reuse a sequence number in another
    // destination session, and regrouping can change tail status.
    static constexpr bool provisionalHeaderIdentityMatchesKey(
        const Key& key, uint32_t actual_sender_hash, uint32_t actual_dst_hash,
        uint16_t actual_seq, uint8_t actual_cw_count,
        bool actual_physical_burst_end) {
        return key.sender_hash == actual_sender_hash &&
               key.dst_hash == actual_dst_hash && key.seq == actual_seq &&
               key.cw_count == actual_cw_count &&
               key.physical_burst_end ==
                   static_cast<uint8_t>(actual_physical_burst_end ? 1 : 0);
    }

    struct KeyHash {
        size_t operator()(const Key& key) const;
    };

    // Inputs that determine a HARQ key. Pulled out of the streaming
    // decoder so the carrier_count_hash construction and field
    // population can be unit tested without standing up the whole
    // decode pipeline. waveform_mode is the underlying byte value of
    // protocol::WaveformMode (kept as int here to avoid a circular
    // include with the protocol layer).
    struct HarqKeyInputs {
        uint32_t sender_hash = 0;
        uint32_t dst_hash = 0;
        uint16_t seq = 0;
        CodeRate rate = CodeRate::R1_4;
        uint8_t cw_count = 0;
        uint8_t cw_index = 0;
        int lifting_z = 27;
        Modulation modulation = Modulation::DQPSK;
        bool channel_interleave = false;
        bool physical_burst_end = false;
        int waveform_mode = 0;          // protocol::WaveformMode underlying byte
        int ofdm_data_carriers = 0;
        uint16_t header_crc = 0;        // whole-frame (turbo) keys only
    };

    struct ProvisionalContext {
        // Both ends of the connected session are part of the protected frame
        // identity. Source-only keying can alias same-seq traffic that the peer
        // sends to another station on a shared channel.
        uint32_t sender_hash = 0;
        uint32_t dst_hash = 0;
        uint16_t seq = 0;
        size_t window_size = 0;
        // #58-follow-on (HARQ provisional keys, 2026-07-01): the receiver's
        // mirror of the sender's next-burst seq fill — ascending !received
        // seqs in the rx window (holes first, unseen tail after; the sender's
        // [resends][new] concatenation is globally ascending and window-bound,
        // so the complement-of-received IS the exact mirror whenever the
        // sender acted on our last SACK). Indexed by burst logical position.
        std::vector<uint16_t> predicted_seqs;

        bool valid() const {
            return sender_hash != 0 && dst_hash != 0 && window_size > 0;
        }
    };

    static Key makeKey(const HarqKeyInputs& in);

    int combine(const Key& key, const std::vector<float>& incoming_llrs,
                std::vector<float>& out_llrs);
    // provisional (2026-07-01): the key was PREDICTED (CW0 undecodable), not
    // header-verified. Tagged entries get no age-refresh on re-retain (hard
    // TTL) and are evicted FIRST on overflow, so a misprediction can never
    // outlive or displace header-verified accumulations. A later retain under
    // a header-verified key promotes the entry (once real, stays real).
    void retain(const Key& key, std::vector<float> combined_llrs,
                bool provisional = false);
    void drop(const Key& key);
    void retainOnlySeqWindow(uint16_t base_seq, size_t window_size);
    void clear();
    void tick(uint32_t elapsed_ms);

    void setTTL(uint32_t ttl_ms);
    void setMaxEntries(size_t n);
    void setEnabled(bool enabled);

    size_t size() const;
    bool enabled() const;

private:
    struct Entry {
        std::vector<float> llrs;
        uint32_t age_ms = 0;
        int attempts = 1;
        bool provisional = false;  // predicted key, not header-verified
        std::list<Key>::iterator lru_it;
    };

    void touchLocked(Entry& entry);
    void eraseLocked(const Key& key);
    void evictExpiredLocked();
    void evictOverflowLocked();

    bool enabled_ = false;
    uint32_t ttl_ms_ = 30000;
    size_t max_entries_ = 32;

    std::list<Key> lru_;
    std::unordered_map<Key, Entry, KeyHash> entries_;
    mutable std::mutex mutex_;
};

} // namespace ultra::fec
