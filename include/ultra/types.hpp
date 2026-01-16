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
    ADAPTIVE,      // Auto-select based on measured SNR
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
    // With 30 carriers @ 93.75 Hz spacing, bandwidth = ±1406 Hz from center
    // At 1500 Hz: carriers span 94-2906 Hz (fits 2.8 kHz SSB filter)

    // OFDM parameters - optimized for 2.8 kHz HF channel
    // 512 FFT at 48kHz = 93.75 Hz bin spacing
    // 30 carriers × 93.75 Hz = 2812 Hz (fills 2.8 kHz)
    uint32_t fft_size = 512;           // FFT size (93.75 Hz bin spacing)
    uint32_t num_carriers = 30;        // Number of data carriers (2.8 kHz)

    // Adaptive cyclic prefix (key performance lever!)
    CyclicPrefixMode cp_mode = CyclicPrefixMode::MEDIUM;
    uint32_t symbol_guard = 4;         // Guard samples (reduced from 8)

    // Pilot configuration for frequency-selective channel estimation
    // Coherence BW for 1ms delay ≈ 159 Hz, carrier spacing = 93.75 Hz
    // pilot_spacing=2 gives 15 pilots, 15 data (50% overhead) - best fading performance
    uint32_t pilot_spacing = 2;        // Pilot every 2 carriers (15 pilots, 15 data)
    bool scattered_pilots = true;      // Rotate pilot positions each symbol

    // Initial modulation/coding (will adapt)
    Modulation modulation = Modulation::QPSK;
    CodeRate code_rate = CodeRate::R1_2;
    SpeedProfile speed_profile = SpeedProfile::BALANCED;

    // Adaptive equalizer settings
    bool adaptive_eq_enabled = false;   // Enable LMS/RLS adaptive equalizer
    bool adaptive_eq_use_rls = false;   // true=RLS, false=LMS
    float lms_mu = 0.05f;               // LMS step size (0.01-0.1 typical)
    float rls_lambda = 0.99f;           // RLS forgetting factor (0.95-0.999)
    bool decision_directed = true;      // Use past decisions as reference

    // Synchronization settings
    float sync_threshold = 0.75f;       // Correlation threshold for sync (0.7-0.95)

    // ARQ settings
    uint32_t frame_size = 256;         // Bytes per frame
    uint32_t max_retries = 8;          // Max retransmissions
    uint32_t arq_timeout_ms = 2000;    // ACK timeout

    // Helper to get actual cyclic prefix length
    // CP scales with FFT size to maintain similar ratio
    uint32_t getCyclicPrefix() const {
        // Base values for 512 FFT
        uint32_t base_cp;
        switch (cp_mode) {
            case CyclicPrefixMode::SHORT:  base_cp = 32; break;
            case CyclicPrefixMode::MEDIUM: base_cp = 48; break;
            case CyclicPrefixMode::LONG:   base_cp = 64; break;
            default: base_cp = 48;
        }
        // Scale for larger FFT sizes (1024 FFT gets 2x CP)
        return base_cp * (fft_size / 512);
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
inline ModemConfig conservative() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::LONG;      // Handle heavy multipath
    cfg.symbol_guard = 8;
    cfg.pilot_spacing = 2;                      // Dense pilots required for HF fading
    cfg.modulation = Modulation::QPSK;
    cfg.code_rate = CodeRate::R1_2;
    cfg.speed_profile = SpeedProfile::CONSERVATIVE;
    return cfg;
}

// Balanced: Good trade-off for typical HF conditions
inline ModemConfig balanced() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;    // 48 samples = 1ms
    cfg.symbol_guard = 4;
    cfg.pilot_spacing = 2;                      // Dense pilots required for HF fading
    cfg.modulation = Modulation::QAM64;
    cfg.code_rate = CodeRate::R3_4;
    cfg.speed_profile = SpeedProfile::BALANCED;
    return cfg;
}

// Turbo: Maximum speed for excellent conditions (30+ dB SNR)
inline ModemConfig turbo() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::SHORT;     // 32 samples = 0.67ms
    cfg.symbol_guard = 0;                       // No guard - tight timing
    cfg.pilot_spacing = 2;                      // Dense pilots required for HF fading
    cfg.modulation = Modulation::QAM256;
    cfg.code_rate = CodeRate::R5_6;            // Highest implemented rate
    cfg.speed_profile = SpeedProfile::TURBO;
    return cfg;
}

// High-throughput: Optimized for maximum speed on Good HF channels
// Uses longer symbols (42 vs 85 sym/s) and more carriers for better
// frequency diversity. Each carrier sees narrower frequency span (less
// fading variation) and longer symbol duration enables better tracking.
//
// Performance (tested with Watterson channel model):
//   AWGN 25dB:     64-QAM R3/4 → 7.5 kbps (100%)
//   Good 20dB:     16-QAM R2/3 → 4.9 kbps (96%)
//   Moderate 20dB: 16-QAM R1/2 → 2.7 kbps (60%) - use conservative() instead
//
// Note: pilot_spacing=4 gives +17% throughput vs spacing=3, but requires
// Good or better conditions. For Moderate/Poor channels, use conservative().
inline ModemConfig high_throughput() {
    ModemConfig cfg;
    cfg.fft_size = 1024;                       // 46.875 Hz spacing (vs 93.75)
    cfg.num_carriers = 59;                     // 59 carriers for ~2.8 kHz bandwidth
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;    // 96 samples (2ms) for 1024 FFT
    cfg.symbol_guard = 0;                       // No extra guard
    cfg.pilot_spacing = 4;                     // 15 pilots, 44 data (25% overhead)
    cfg.modulation = Modulation::QAM16;
    cfg.code_rate = CodeRate::R2_3;            // R2/3 - robust enough for Good conds
    cfg.speed_profile = SpeedProfile::BALANCED;
    cfg.adaptive_eq_enabled = false;           // ZF works well with dense pilots
    cfg.adaptive_eq_use_rls = false;
    cfg.rls_lambda = 0.97f;
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
