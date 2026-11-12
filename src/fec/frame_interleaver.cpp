#include "frame_interleaver.hpp"
#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <string>

namespace ultra {
namespace fec {

// Static member definitions (cache only for the default NUM_CODEWORDS x 648 config)
std::vector<int> FrameInterleaver::interleave_table_;
std::vector<int> FrameInterleaver::deinterleave_table_;
bool FrameInterleaver::tables_initialized_ = false;

int FrameInterleaver::sanitizeCodewordCount(int codeword_count) {
    // 8 -> 16 (2026-07-05, cw16 build): this clamp silently duplicated the OLD
    // v2::kMaxFixedFrameCodewords and lagged its 8->16 raise — the first cw16 TX
    // threw ('expected 8 codewords, got 16', encodeBurstLight abort, 46 s of
    // silent no-TX in the first sim run). The table builder is fully generic in
    // (codeword_count, bits_per_codeword); this bound is only an insanity guard
    // and MUST track v2::kMaxFixedFrameCodewords (fec/ does not include protocol
    // headers — keep the values in lockstep manually).
    return std::clamp(codeword_count, 1, 16);
}

int FrameInterleaver::totalFrameBits(int codeword_count, int bits_per_codeword) {
    return sanitizeCodewordCount(codeword_count) * bits_per_codeword;
}

int FrameInterleaver::totalFrameBytes(int codeword_count, int bits_per_codeword) {
    return (totalFrameBits(codeword_count, bits_per_codeword) + 7) / 8;
}

void FrameInterleaver::buildTables(int codeword_count,
                                   int bits_per_codeword,
                                   std::vector<int>& interleave_table,
                                   std::vector<int>& deinterleave_table) {
    codeword_count = sanitizeCodewordCount(codeword_count);
    const int total_bits = totalFrameBits(codeword_count, bits_per_codeword);

    interleave_table.resize(total_bits);
    deinterleave_table.resize(total_bits);

    for (int cw = 0; cw < codeword_count; ++cw) {
        for (int bit = 0; bit < bits_per_codeword; ++bit) {
            int original_idx = cw * bits_per_codeword + bit;
            int interleaved_idx = bit * codeword_count + (cw + bit) % codeword_count;

            interleave_table[original_idx] = interleaved_idx;
            deinterleave_table[interleaved_idx] = original_idx;
        }
    }
}

void FrameInterleaver::ensureTablesInitialized() {
    // Built once for the process, from whichever thread gets here first (the encoder and the decoder
    // both do); every caller waits for the build and then sees the finished tables. A plain bool let
    // two first callers resize the same static vectors at once, and a weakly ordered CPU show a
    // reader the flag before the table data.
    static std::once_flag once;
    std::call_once(once, [] { buildDefaultTables(); });
}

void FrameInterleaver::buildDefaultTables() {

    // Build permutation tables for rotating round-robin interleaving.
    //
    // For DQPSK, each carrier produces 2 bits with different reliability:
    //   MSB (sin-based): ±90° decision margin → more reliable
    //   LSB (cos-based): ±45° decision margin → less reliable
    //
    // Simple round-robin (bit*4 + cw) assigns CW0,CW2 always to MSB and
    // CW1,CW3 always to LSB, creating a systematic reliability imbalance.
    //
    // Fix: rotate CW assignment by LDPC bit position:
    //   interleaved_idx = bit * 4 + (cw + bit) % 4
    //
    // This ensures each CW gets 50% MSB + 50% LSB positions over every
    // 4 LDPC bit positions, equalizing reliability across all codewords.
    //
    // The cache covers only the default NUM_CODEWORDS x BITS_PER_CODEWORD frame;
    // any other (codeword_count, bits_per_codeword) builds a local table.

    buildTables(NUM_CODEWORDS, BITS_PER_CODEWORD, interleave_table_, deinterleave_table_);

    tables_initialized_ = true;
}

std::vector<uint8_t> FrameInterleaver::interleave(
    const std::vector<std::vector<uint8_t>>& coded_codewords,
    int codeword_count,
    int bits_per_codeword) {

    codeword_count = sanitizeCodewordCount(codeword_count);
    if (coded_codewords.size() != static_cast<size_t>(codeword_count)) {
        throw std::invalid_argument("FrameInterleaver: expected " +
                                    std::to_string(codeword_count) +
                                    " codewords, got " +
                                    std::to_string(coded_codewords.size()));
    }

    std::vector<int> interleave_table;
    std::vector<int> deinterleave_table;
    const std::vector<int>* table = nullptr;
    if (codeword_count == NUM_CODEWORDS && bits_per_codeword == BITS_PER_CODEWORD) {
        ensureTablesInitialized();
        table = &interleave_table_;
    } else {
        buildTables(codeword_count, bits_per_codeword, interleave_table, deinterleave_table);
        table = &interleave_table;
    }

    const int total_bits = totalFrameBits(codeword_count, bits_per_codeword);

    // Convert all codewords to a single bit vector
    std::vector<uint8_t> original_bits(total_bits);
    size_t bit_idx = 0;

    for (int cw = 0; cw < codeword_count; ++cw) {
        const auto& cw_bytes = coded_codewords[cw];
        // Each coded codeword should be bits_per_codeword/8 bytes
        for (size_t byte_idx = 0; byte_idx < cw_bytes.size() && bit_idx < static_cast<size_t>(total_bits); ++byte_idx) {
            uint8_t byte = cw_bytes[byte_idx];
            for (int b = 7; b >= 0 && bit_idx < static_cast<size_t>((cw + 1) * bits_per_codeword); --b) {
                original_bits[bit_idx++] = (byte >> b) & 1;
            }
        }
        // Ensure we hit exactly bits_per_codeword bits per CW
        while (bit_idx < static_cast<size_t>((cw + 1) * bits_per_codeword)) {
            original_bits[bit_idx++] = 0;  // Zero-pad if short
        }
    }

    // Apply interleaving permutation
    std::vector<uint8_t> interleaved_bits(total_bits);
    for (int i = 0; i < total_bits; ++i) {
        interleaved_bits[(*table)[i]] = original_bits[i];
    }

    // Convert back to bytes
    std::vector<uint8_t> output((total_bits + 7) / 8);
    for (int i = 0; i < total_bits; ++i) {
        if (interleaved_bits[i]) {
            output[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    return output;
}

std::vector<uint8_t> FrameInterleaver::interleave(
    const std::vector<std::vector<uint8_t>>& coded_codewords) {
    return interleave(coded_codewords, NUM_CODEWORDS, BITS_PER_CODEWORD);
}

std::vector<std::vector<float>> FrameInterleaver::deinterleave(
    const std::vector<float>& interleaved_soft,
    int codeword_count,
    int bits_per_codeword) {

    codeword_count = sanitizeCodewordCount(codeword_count);
    const int total_bits = totalFrameBits(codeword_count, bits_per_codeword);
    if (interleaved_soft.size() < static_cast<size_t>(total_bits)) {
        throw std::invalid_argument("FrameInterleaver: expected " +
                                    std::to_string(total_bits) +
                                    " soft bits, got " +
                                    std::to_string(interleaved_soft.size()));
    }

    std::vector<int> interleave_table;
    std::vector<int> deinterleave_table;
    const std::vector<int>* table = nullptr;
    if (codeword_count == NUM_CODEWORDS && bits_per_codeword == BITS_PER_CODEWORD) {
        ensureTablesInitialized();
        table = &deinterleave_table_;
    } else {
        buildTables(codeword_count, bits_per_codeword, interleave_table, deinterleave_table);
        table = &deinterleave_table;
    }

    // Apply deinterleaving permutation
    std::vector<float> original_soft(total_bits);
    for (int i = 0; i < total_bits; ++i) {
        original_soft[(*table)[i]] = interleaved_soft[i];
    }

    // Split into codewords
    std::vector<std::vector<float>> result(codeword_count);
    for (int cw = 0; cw < codeword_count; ++cw) {
        result[cw].resize(bits_per_codeword);
        for (int bit = 0; bit < bits_per_codeword; ++bit) {
            result[cw][bit] = original_soft[cw * bits_per_codeword + bit];
        }
    }

    return result;
}

std::vector<std::vector<float>> FrameInterleaver::deinterleave(
    const std::vector<float>& interleaved_soft) {
    return deinterleave(interleaved_soft, NUM_CODEWORDS, BITS_PER_CODEWORD);
}

std::vector<float> FrameInterleaver::interleaveSoft(
    const std::vector<std::vector<float>>& soft_codewords,
    int codeword_count,
    int bits_per_codeword) {

    codeword_count = sanitizeCodewordCount(codeword_count);
    if (soft_codewords.size() != static_cast<size_t>(codeword_count)) {
        throw std::invalid_argument("FrameInterleaver: expected " +
                                    std::to_string(codeword_count) + " codewords");
    }

    std::vector<int> interleave_table;
    std::vector<int> deinterleave_table;
    const std::vector<int>* table = nullptr;
    if (codeword_count == NUM_CODEWORDS && bits_per_codeword == BITS_PER_CODEWORD) {
        ensureTablesInitialized();
        table = &interleave_table_;
    } else {
        buildTables(codeword_count, bits_per_codeword, interleave_table, deinterleave_table);
        table = &interleave_table;
    }

    const int total_bits = totalFrameBits(codeword_count, bits_per_codeword);

    // Flatten to single vector
    std::vector<float> original_soft(total_bits);
    for (int cw = 0; cw < codeword_count; ++cw) {
        for (int bit = 0; bit < bits_per_codeword && bit < static_cast<int>(soft_codewords[cw].size()); ++bit) {
            original_soft[cw * bits_per_codeword + bit] = soft_codewords[cw][bit];
        }
    }

    // Apply interleaving
    std::vector<float> interleaved(total_bits);
    for (int i = 0; i < total_bits; ++i) {
        interleaved[(*table)[i]] = original_soft[i];
    }

    return interleaved;
}

std::vector<float> FrameInterleaver::interleaveSoft(
    const std::vector<std::vector<float>>& soft_codewords) {
    return interleaveSoft(soft_codewords, NUM_CODEWORDS, BITS_PER_CODEWORD);
}

}  // namespace fec
}  // namespace ultra
