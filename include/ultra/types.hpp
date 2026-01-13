#pragma once

#include <complex>
#include <cstdint>
#include <cstring>
#include <span>
#include <vector>
#include <array>

namespace ultra {

// Core types
using Sample = float;                          // Audio sample
using Complex = std::complex<float>;           // Complex sample for OFDM
using Symbol = std::vector<Complex>;           // One OFDM symbol
using Samples = std::vector<Sample>;           // Audio buffer
using Bytes = std::vector<uint8_t>;            // Data payload

// Spans for zero-copy operations
using SampleSpan = std::span<const Sample>;
using ByteSpan = std::span<const uint8_t>;
using MutableSampleSpan = std::span<Sample>;
using MutableByteSpan = std::span<uint8_t>;

// Modulation schemes (in order of bits/symbol)
enum class Modulation : uint8_t {
    BPSK = 1,    // 1 bit/symbol  - most robust
    QPSK = 2,    // 2 bits/symbol
    QAM8 = 3,    // 3 bits/symbol
    QAM16 = 4,   // 4 bits/symbol
    QAM32 = 5,   // 5 bits/symbol
    QAM64 = 6,   // 6 bits/symbol
    QAM256 = 8,  // 8 bits/symbol - highest throughput, needs 30+ dB SNR
};

// Cyclic prefix modes for adaptive overhead
enum class CyclicPrefixMode : uint8_t {
    SHORT = 0,   // 32 samples (0.67ms) - good conditions, minimal multipath
    MEDIUM = 1,  // 48 samples (1.0ms)  - moderate conditions
    LONG = 2,    // 64 samples (1.33ms) - poor conditions, strong multipath
};

// Speed profiles balancing throughput vs robustness
enum class SpeedProfile : uint8_t {
    CONSERVATIVE,  // Maximize reliability, lower speed
    BALANCED,      // Good balance for typical HF
    TURBO,         // Maximum speed for good conditions
};

// FEC code rates
enum class CodeRate : uint8_t {
    R1_4,   // 1/4 - most redundancy
    R1_3,   // 1/3
    R1_2,   // 1/2
    R2_3,   // 2/3
    R3_4,   // 3/4
    R5_6,   // 5/6
    R7_8,   // 7/8 - least redundancy
};

// Convert CodeRate enum to float value
inline float getCodeRateValue(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 0.25f;
        case CodeRate::R1_3: return 0.333f;
        case CodeRate::R1_2: return 0.5f;
        case CodeRate::R2_3: return 0.667f;
        case CodeRate::R3_4: return 0.75f;
        case CodeRate::R5_6: return 0.833f;
        case CodeRate::R7_8: return 0.875f;
        default: return 0.5f;
    }
}

// Channel quality estimate
struct ChannelQuality {
    float snr_db;           // Estimated SNR in dB
    float doppler_hz;       // Estimated Doppler spread
    float delay_spread_ms;  // Estimated multipath delay spread
    float ber_estimate;     // Estimated bit error rate
};

// Modem configuration
struct ModemConfig {
    // Audio parameters
    uint32_t sample_rate = 48000;      // Audio sample rate
    uint32_t center_freq = 1500;       // Center frequency in audio passband

    // OFDM parameters - optimized for 2.8 kHz HF channel
    // 512 FFT at 48kHz = 93.75 Hz bin spacing
    // 30 carriers × 93.75 Hz = 2812 Hz (fills 2.8 kHz)
    uint32_t fft_size = 512;           // FFT size (93.75 Hz bin spacing)
    uint32_t num_carriers = 30;        // Number of data carriers (2.8 kHz)

    // Adaptive cyclic prefix (key performance lever!)
    CyclicPrefixMode cp_mode = CyclicPrefixMode::MEDIUM;
    uint32_t symbol_guard = 4;         // Guard samples (reduced from 8)

    // Pilot configuration - scattered pilots for efficiency
    uint32_t pilot_spacing = 6;        // Pilot every N carriers (5 pilots, 25 data = 17% overhead)
    bool scattered_pilots = true;      // Rotate pilot positions each symbol

    // Initial modulation/coding (will adapt)
    Modulation modulation = Modulation::QPSK;
    CodeRate code_rate = CodeRate::R1_2;
    SpeedProfile speed_profile = SpeedProfile::BALANCED;

    // ARQ settings
    uint32_t frame_size = 256;         // Bytes per frame
    uint32_t max_retries = 8;          // Max retransmissions
    uint32_t arq_timeout_ms = 2000;    // ACK timeout

    // Helper to get actual cyclic prefix length
    uint32_t getCyclicPrefix() const {
        switch (cp_mode) {
            case CyclicPrefixMode::SHORT:  return 32;
            case CyclicPrefixMode::MEDIUM: return 48;
            case CyclicPrefixMode::LONG:   return 64;
            default: return 48;
        }
    }

    // Calculate symbol duration in samples
    uint32_t getSymbolDuration() const {
        return fft_size + getCyclicPrefix() + symbol_guard;
    }

    // Calculate symbol rate
    float getSymbolRate() const {
        return static_cast<float>(sample_rate) / getSymbolDuration();
    }

    // Estimate data carriers (excludes pilots)
    uint32_t getDataCarriers() const {
        uint32_t pilots = (num_carriers + pilot_spacing - 1) / pilot_spacing;
        return num_carriers - pilots;
    }

    // Calculate theoretical throughput in bps
    float getTheoreticalThroughput(Modulation mod, CodeRate rate) const {
        uint32_t bits_per_carrier = static_cast<uint32_t>(mod);
        return getDataCarriers() * bits_per_carrier * getCodeRateValue(rate) * getSymbolRate();
    }
};

// Frame types
enum class FrameType : uint8_t {
    DATA = 0x00,
    ACK = 0x01,
    NACK = 0x02,
    SYNC = 0x03,
    PROBE = 0x04,      // Channel probe for adaptation
    CONNECT = 0x05,
    DISCONNECT = 0x06,
};

// Statistics
struct ModemStats {
    uint64_t bytes_sent = 0;
    uint64_t bytes_received = 0;
    uint64_t frames_sent = 0;
    uint64_t frames_received = 0;
    uint64_t frames_retransmitted = 0;
    uint64_t frames_failed = 0;
    float throughput_bps = 0.0f;
    float current_snr_db = 0.0f;
    Modulation current_modulation = Modulation::QPSK;
    CodeRate current_code_rate = CodeRate::R1_2;
};

// Speed profile preset configurations
namespace presets {

// Conservative: Maximum reliability for poor HF conditions
// ~4.8 kbps at QPSK R1/2
inline ModemConfig conservative() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::LONG;      // Handle heavy multipath
    cfg.symbol_guard = 8;
    cfg.pilot_spacing = 5;                      // More pilots for tracking
    cfg.modulation = Modulation::QPSK;
    cfg.code_rate = CodeRate::R1_2;
    cfg.speed_profile = SpeedProfile::CONSERVATIVE;
    return cfg;
}

// Balanced: Good trade-off for typical HF conditions
// ~9.2 kbps at 64-QAM R3/4
inline ModemConfig balanced() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;    // 48 samples = 1ms
    cfg.symbol_guard = 4;
    cfg.pilot_spacing = 6;                      // 5 pilots, 25 data
    cfg.modulation = Modulation::QAM64;
    cfg.code_rate = CodeRate::R3_4;
    cfg.speed_profile = SpeedProfile::BALANCED;
    return cfg;
}

// Turbo: Maximum speed for excellent conditions (30+ dB SNR)
// ~16 kbps at 256-QAM R7/8
inline ModemConfig turbo() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::SHORT;     // 32 samples = 0.67ms
    cfg.symbol_guard = 0;                       // No guard - tight timing
    cfg.pilot_spacing = 8;                      // Minimal pilots
    cfg.modulation = Modulation::QAM256;
    cfg.code_rate = CodeRate::R7_8;
    cfg.speed_profile = SpeedProfile::TURBO;
    return cfg;
}

// Get config for given speed profile
inline ModemConfig forProfile(SpeedProfile profile) {
    switch (profile) {
        case SpeedProfile::CONSERVATIVE: return conservative();
        case SpeedProfile::BALANCED:     return balanced();
        case SpeedProfile::TURBO:        return turbo();
        default:                         return balanced();
    }
}

} // namespace presets

} // namespace ultra
