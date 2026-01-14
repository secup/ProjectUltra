#pragma once

#include "ultra/types.hpp"
#include <optional>

namespace ultra {
namespace protocol {

/**
 * Simple compression utilities using miniz (zlib-compatible)
 *
 * Uses deflate compression which is well-suited for text and binary data.
 * Compression level 6 is a good balance of speed vs ratio.
 */
class Compression {
public:
    // Compression levels
    static constexpr int LEVEL_NONE = 0;
    static constexpr int LEVEL_FAST = 1;
    static constexpr int LEVEL_DEFAULT = 6;
    static constexpr int LEVEL_BEST = 9;

    // Minimum data size worth compressing (overhead not worth it below this)
    static constexpr size_t MIN_COMPRESS_SIZE = 32;

    /**
     * Compress data using deflate
     * Returns compressed data, or empty optional on failure
     */
    static std::optional<Bytes> compress(const Bytes& input, int level = LEVEL_DEFAULT);

    /**
     * Decompress deflate-compressed data
     * max_output_size is a safety limit (default 10MB)
     * Returns decompressed data, or empty optional on failure
     */
    static std::optional<Bytes> decompress(const Bytes& input, size_t max_output_size = 10 * 1024 * 1024);

    /**
     * Check if compression would be beneficial
     * Returns true if data is large enough and compressible
     */
    static bool shouldCompress(const Bytes& input);

    /**
     * Get maximum compressed size for given input size
     * Useful for pre-allocating buffers
     */
    static size_t maxCompressedSize(size_t input_size);
};

} // namespace protocol
} // namespace ultra
