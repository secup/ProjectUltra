#pragma once

// ICodec - Abstract interface for Forward Error Correction codecs
//
// Abstracts LDPC (and potentially future codes like Polar, Turbo, SC-LDPC)
// Currently wraps LDPCEncoder and LDPCDecoder.

#include "ultra/types.hpp"
#include <string>
#include <vector>
#include <utility>

namespace ultra {

// CodeRate is defined in ultra/types.hpp - include it if needed
// However, rx_pipeline.hpp already includes waveform_interface.hpp which includes ultra/types.hpp

namespace fec {

// Decode result with additional info
struct DecodeResult {
    bool success = false;           // True if decoding succeeded
    Bytes data;                     // Decoded data bytes
    int iterations = 0;             // Number of decoder iterations used
    float ber_estimate = 0.0f;      // Estimated bit error rate (post-decode)
};

// Abstract codec interface
class ICodec {
public:
    virtual ~ICodec() = default;

    // ========================================================================
    // IDENTITY
    // ========================================================================

    // Human-readable name (e.g., "802.11n LDPC", "SC-LDPC", "Polar")
    virtual std::string getName() const = 0;

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    // Set code rate (R1/4, R1/2, R2/3, R3/4, R5/6)
    virtual void setRate(CodeRate rate) = 0;

    // Get current code rate
    virtual CodeRate getRate() const = 0;

    // Set maximum decoder iterations (for iterative decoders)
    virtual void setMaxIterations(int iterations) = 0;

    // Get maximum iterations
    virtual int getMaxIterations() const = 0;

    // ========================================================================
    // ENCODING
    // ========================================================================

    // Encode data bytes into codeword bytes
    // Input: data bytes (length must match getDataBytes())
    // Output: encoded codeword bytes (length = getCodewordBytes())
    virtual Bytes encode(const Bytes& data) = 0;

    // ========================================================================
    // DECODING
    // ========================================================================

    // Decode from soft bits (LLR values)
    // Input: soft bits (length must match getCodewordBits())
    // Output: (success, decoded_data)
    virtual std::pair<bool, Bytes> decode(const std::vector<float>& soft_bits) = 0;

    // Decode with extended result
    virtual DecodeResult decodeExtended(const std::vector<float>& soft_bits) = 0;

    // ========================================================================
    // PARAMETERS
    // ========================================================================

    // Total bits per codeword (n)
    virtual size_t getCodewordBits() const = 0;

    // Information bits per codeword (k)
    virtual size_t getInfoBits() const = 0;

    // Parity bits per codeword (n - k)
    virtual size_t getParityBits() const = 0;

    // Codeword bytes (ceiling of n/8)
    virtual size_t getCodewordBytes() const = 0;

    // Data bytes per codeword (floor of k/8)
    virtual size_t getDataBytes() const = 0;

    // Get effective code rate as float (k/n)
    virtual float getEffectiveRate() const = 0;
};

// Convenience alias
using CodecPtr = std::unique_ptr<ICodec>;

} // namespace fec
} // namespace ultra
