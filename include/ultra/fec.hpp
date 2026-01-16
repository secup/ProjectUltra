#pragma once

#include "types.hpp"
#include <memory>

namespace ultra {

/**
 * LDPC Encoder
 *
 * Low-Density Parity-Check codes for near-Shannon-limit performance.
 * Uses codes optimized for our block sizes and rate adaptivity.
 *
 * Why LDPC:
 * - Approaches Shannon limit within 0.5 dB
 * - Parallelizable decoding
 * - Well-suited for soft-decision input
 * - Proven in DVB-S2, WiFi, 5G
 */
class LDPCEncoder {
public:
    explicit LDPCEncoder(CodeRate rate);
    ~LDPCEncoder();

    // Encode data block, returns coded bits
    Bytes encode(ByteSpan data);

    // Get output size for given input size
    size_t getCodedSize(size_t input_size) const;

    // Get code rate
    CodeRate getRate() const;

    // Set code rate (for rate adaptation)
    void setRate(CodeRate rate);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * LDPC Decoder
 *
 * Iterative belief-propagation decoder.
 * Accepts soft bits for best performance.
 */
class LDPCDecoder {
public:
    explicit LDPCDecoder(CodeRate rate);
    ~LDPCDecoder();

    // Decode from hard bits
    Bytes decode(ByteSpan coded_data);

    // Decode from soft bits (LLRs) - much better performance
    Bytes decodeSoft(std::span<const float> llrs);

    // Check if last decode succeeded (CRC/parity check)
    bool lastDecodeSuccess() const;

    // Get number of iterations used in last decode
    int lastIterations() const;

    // Set code rate
    void setRate(CodeRate rate);

    // Get current code rate
    CodeRate getRate() const;

    // Set max iterations (tradeoff: more = better correction, slower)
    void setMaxIterations(int max_iter);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Interleaver
 *
 * Spreads burst errors across multiple code blocks.
 * Critical for HF where fading causes error bursts.
 */
class Interleaver {
public:
    Interleaver(size_t rows, size_t cols);

    Bytes interleave(ByteSpan data);
    Bytes deinterleave(ByteSpan data);

    // Soft bit versions
    std::vector<float> interleave(std::span<const float> soft_bits);
    std::vector<float> deinterleave(std::span<const float> soft_bits);

    // Debug access
    size_t getPermutation(size_t i) const { return i < permutation_.size() ? permutation_[i] : 0; }
    size_t getRows() const { return rows_; }
    size_t getCols() const { return cols_; }

private:
    size_t rows_, cols_;
    std::vector<size_t> permutation_;
};

} // namespace ultra
