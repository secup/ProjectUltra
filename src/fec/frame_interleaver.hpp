#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>

namespace ultra {
namespace fec {

/**
 * Frame-level interleaver for fixed-codeword frames.
 *
 * Purpose: Spread burst fading errors across all fixed-frame codewords so that
 * each CW sees only a fraction of the errors instead of one CW being completely
 * corrupted.
 *
 * Operation:
 *   TX: After LDPC encoding, interleave coded bits across all fixed-frame CWs
 *   RX: Before LDPC decoding, deinterleave soft bits back to original order
 *
 * Interleaving pattern (block interleave):
 *   Original order: [CW0: a0..a647][CW1: b0..b647]...[CWn]
 *   Interleaved:    [a0 b0 ... n0 a1 b1 ... n1 ...]
 *
 * After a fading burst corrupting bits 1000-1500:
 *   Without interleave: CW1 completely lost (bits 648-1295)
 *   With interleave: Each CW loses ~125 bits (~19%), all decode
 *
 * R1/4 LDPC can correct up to ~37% bit errors, so 19% is easily recoverable.
 */
class FrameInterleaver {
public:
    // Default fixed frame parameters (source-compatible 4-CW helpers)
    static constexpr int NUM_CODEWORDS = 4;
    static constexpr int BITS_PER_CODEWORD = 648;
    static constexpr int TOTAL_FRAME_BITS = NUM_CODEWORDS * BITS_PER_CODEWORD;  // 2592

    static int sanitizeCodewordCount(int codeword_count);
    // bits_per_codeword defaults to 648 (n=648 LDPC). Pass 1944 for the
    // file-class long code (Z=81). Default keeps every existing caller identical.
    static int totalFrameBits(int codeword_count, int bits_per_codeword = BITS_PER_CODEWORD);
    static int totalFrameBytes(int codeword_count, int bits_per_codeword = BITS_PER_CODEWORD);

    /**
     * Interleave coded bits for transmission.
     *
     * Takes N codewords (N × 81 bytes) and interleaves at bit level.
     * Output is same size, bits reordered for fading resistance.
     *
     * @param coded_bytes Vector of N coded codewords (81 bytes each)
     * @return Interleaved bytes
     */
    static std::vector<uint8_t> interleave(const std::vector<std::vector<uint8_t>>& coded_codewords,
                                           int codeword_count,
                                           int bits_per_codeword = BITS_PER_CODEWORD);
    static std::vector<uint8_t> interleave(const std::vector<std::vector<uint8_t>>& coded_codewords);

    /**
     * Deinterleave soft bits for decoding.
     *
     * Takes interleaved soft bits and restores original order
     * so each codeword's soft bits are grouped together for LDPC decoding.
     *
     * @param interleaved_soft Interleaved soft bits
     * @param codeword_count Number of codewords in the fixed frame
     * @return Vector of N soft bit vectors (648 floats each)
     */
    static std::vector<std::vector<float>> deinterleave(const std::vector<float>& interleaved_soft,
                                                        int codeword_count,
                                                        int bits_per_codeword = BITS_PER_CODEWORD);
    static std::vector<std::vector<float>> deinterleave(const std::vector<float>& interleaved_soft);

    /**
     * Interleave soft bits (for testing/simulation).
     * Same pattern as byte interleave but operates on soft bits.
     */
    static std::vector<float> interleaveSoft(const std::vector<std::vector<float>>& soft_codewords,
                                             int codeword_count,
                                             int bits_per_codeword = BITS_PER_CODEWORD);
    static std::vector<float> interleaveSoft(const std::vector<std::vector<float>>& soft_codewords);

private:
    // Permutation tables (computed once, used many times)
    static void ensureTablesInitialized();
    static void buildDefaultTables();
    static void buildTables(int codeword_count,
                            int bits_per_codeword,
                            std::vector<int>& interleave_table,
                            std::vector<int>& deinterleave_table);
    static std::vector<int> interleave_table_;  // Original index → interleaved index
    static std::vector<int> deinterleave_table_;  // Interleaved index → original index
    static bool tables_initialized_;
};

}  // namespace fec
}  // namespace ultra
