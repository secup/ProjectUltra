#include "compression.hpp"
#include "miniz/miniz.h"

namespace ultra {
namespace protocol {

std::optional<Bytes> Compression::compress(const Bytes& input, int level) {
    if (input.empty()) {
        return Bytes{};
    }

    // Get maximum possible compressed size
    mz_ulong max_compressed = mz_compressBound(input.size());

    Bytes output(max_compressed);
    mz_ulong compressed_size = max_compressed;

    // Compress using zlib-style compression
    int status = mz_compress2(
        output.data(),
        &compressed_size,
        input.data(),
        input.size(),
        level
    );

    if (status != MZ_OK) {
        return std::nullopt;
    }

    // Resize to actual compressed size
    output.resize(compressed_size);
    return output;
}

std::optional<Bytes> Compression::decompress(const Bytes& input, size_t max_output_size) {
    if (input.empty()) {
        return Bytes{};
    }

    // Start with a reasonable initial size, grow if needed
    size_t output_size = input.size() * 4;  // Assume ~4x compression ratio
    if (output_size > max_output_size) {
        output_size = max_output_size;
    }

    Bytes output(output_size);
    mz_ulong actual_size = output_size;

    int status = mz_uncompress(
        output.data(),
        &actual_size,
        input.data(),
        input.size()
    );

    // If buffer was too small, try larger sizes
    while (status == MZ_BUF_ERROR && output_size < max_output_size) {
        output_size = std::min(output_size * 2, max_output_size);
        output.resize(output_size);
        actual_size = output_size;

        status = mz_uncompress(
            output.data(),
            &actual_size,
            input.data(),
            input.size()
        );
    }

    if (status != MZ_OK) {
        return std::nullopt;
    }

    output.resize(actual_size);
    return output;
}

bool Compression::shouldCompress(const Bytes& input) {
    // Don't compress small data - overhead not worth it
    if (input.size() < MIN_COMPRESS_SIZE) {
        return false;
    }

    // Quick entropy check: see if first ~100 bytes compress well
    // If compression ratio < 0.9, it's worth compressing
    size_t sample_size = std::min(input.size(), size_t(256));
    Bytes sample(input.begin(), input.begin() + sample_size);

    auto compressed = compress(sample, LEVEL_FAST);
    if (!compressed) {
        return false;
    }

    float ratio = static_cast<float>(compressed->size()) / sample_size;
    return ratio < 0.9f;
}

size_t Compression::maxCompressedSize(size_t input_size) {
    return mz_compressBound(input_size);
}

} // namespace protocol
} // namespace ultra
