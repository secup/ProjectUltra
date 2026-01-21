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

/**
 * ChannelInterleaver
 *
 * 2D time-frequency interleaver designed for OFDM on fading channels.
 * Spreads bits across BOTH OFDM symbols (time) AND carriers (frequency)
 * to provide diversity against HF frequency-selective fading.
 *
 * Key insight: The basic 6x108 interleaver spreads bits within a codeword,
 * but with 60+ bits per OFDM symbol, consecutive bits still land in the
 * SAME symbol - providing frequency diversity but NO time diversity.
 *
 * This interleaver ensures consecutive LDPC bits are spread across
 * different OFDM symbols, critical for fading channels.
 */
class ChannelInterleaver {
public:
    // bits_per_symbol = num_carriers * bits_per_carrier (e.g., 58*2=116 for DQPSK)
    // total_bits = LDPC codeword size (648)
    ChannelInterleaver(size_t bits_per_symbol, size_t total_bits = 648);

    std::vector<float> interleave(std::span<const float> soft_bits);
    std::vector<float> deinterleave(std::span<const float> soft_bits);

    Bytes interleave(ByteSpan data);
    Bytes deinterleave(ByteSpan data);

    // Get the minimum symbol separation between consecutive input bits
    size_t getSymbolSeparation() const { return symbol_separation_; }

private:
    size_t bits_per_symbol_;
    size_t total_bits_;
    size_t num_symbols_;
    size_t symbol_separation_;
    std::vector<size_t> permutation_;
    std::vector<size_t> inverse_permutation_;
};

} // namespace ultra
