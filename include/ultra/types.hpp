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

// Provenance for consumer-facing snr_db values. Historical string tokens remain
// stable, but operator/rate-selection physical SNR producers now report the
// shared receiver in-band convention.
enum class SNRSource : uint8_t {
    NONE = 0,
    IDLE_IN_BAND,     // IdleNoiseSNREstimator, receiver passband/in-band SNR.
    OFDM_BROADBAND,   // Historical name; OFDM LTS/pilot residual in-band SNR.
    MCDPSK_IN_BAND,   // MC-DPSK training/data residual receiver in-band SNR.
    OFDM_INTERNAL,    // Demodulator internal LLR/channel-quality SNR scale.
    SYNC_QUALITY,     // Chirp correlation confidence score, not physical SNR.
};

inline const char* snrSourceToString(SNRSource source) {
    switch (source) {
        case SNRSource::IDLE_IN_BAND:   return "idle_in_band";
        case SNRSource::OFDM_BROADBAND: return "ofdm_broadband";
        case SNRSource::MCDPSK_IN_BAND: return "mcdpsk_in_band";
        case SNRSource::OFDM_INTERNAL:  return "ofdm_internal";
        case SNRSource::SYNC_QUALITY:   return "sync_quality";
        case SNRSource::NONE:
        default:                        return "none";
    }
}

inline const char* snrSourceDisplayLabel(SNRSource source) {
    switch (source) {
        case SNRSource::IDLE_IN_BAND:   return "in-band SNR";
        case SNRSource::OFDM_BROADBAND: return "OFDM in-band SNR";
        case SNRSource::MCDPSK_IN_BAND: return "MC-DPSK in-band SNR";
        case SNRSource::OFDM_INTERNAL:  return "OFDM internal SNR";
        case SNRSource::SYNC_QUALITY:   return "sync quality";
        case SNRSource::NONE:
        default:                        return "no SNR";
    }
}

// Spans for zero-copy operations
using SampleSpan = std::span<const Sample>;
using ByteSpan = std::span<const uint8_t>;
using MutableSampleSpan = std::span<Sample>;
using MutableByteSpan = std::span<uint8_t>;

// Modulation schemes
// Differential modes (DBPSK, DQPSK, D8PSK) are immune to phase distortion from audio hardware
enum class Modulation : uint8_t {
    DBPSK = 0,   // 1 bit/symbol  - differential BPSK, immune to phase distortion
    BPSK = 1,    // 1 bit/symbol  - most robust, coherent detection
    DQPSK = 2,   // 2 bits/symbol - differential QPSK, immune to phase distortion
    QPSK = 3,    // 2 bits/symbol - coherent
    D8PSK = 4,   // 3 bits/symbol - differential 8PSK, immune to phase distortion
    QAM8 = 5,    // 3 bits/symbol - coherent (same as 8PSK constellation)
    QAM16 = 6,   // 4 bits/symbol - coherent
    QAM32 = 7,   // 5 bits/symbol - coherent
    QAM64 = 8,   // 6 bits/symbol - coherent
    QAM256 = 10, // 8 bits/symbol - highest throughput, needs 30+ dB SNR
    AUTO = 0xFF, // Let responder decide based on SNR (for CONNECT frame)
};

// Get bits per symbol for a modulation scheme
inline uint32_t getBitsPerSymbol(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK:  return 1;
        case Modulation::BPSK:   return 1;
        case Modulation::DQPSK:  return 2;
        case Modulation::QPSK:   return 2;
        case Modulation::D8PSK:  return 3;
        case Modulation::QAM8:   return 3;
        case Modulation::QAM16:  return 4;
        case Modulation::QAM32:  return 5;
        case Modulation::QAM64:  return 6;
        case Modulation::QAM256: return 8;
        default: return 1;
    }
}

// Modulation to string
inline const char* modulationToString(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK:  return "DBPSK";
        case Modulation::BPSK:   return "BPSK";
        case Modulation::DQPSK:  return "DQPSK";
        case Modulation::QPSK:   return "QPSK";
        case Modulation::D8PSK:  return "D8PSK";
        case Modulation::QAM8:   return "8PSK";
        case Modulation::QAM16:  return "16QAM";
        case Modulation::QAM32:  return "32QAM";
        case Modulation::QAM64:  return "64QAM";
        case Modulation::QAM256: return "256QAM";
        default: return "UNKNOWN";
    }
}

// Cyclic prefix modes for adaptive overhead
// NOTE: these are BASE values for a 512-point FFT. getCyclicPrefix() scales them by
// (fft_size / 512), so at the DEFAULT 1024 FFT every value below DOUBLES. Pinned by
// tests/test_cyclic_prefix_geometry.cpp — do not restate CP lengths in prose without it.
enum class CyclicPrefixMode : uint8_t {
    SHORT = 0,   // base 32 samples (0.67ms @512) ->  64 / 1.33ms @1024 - minimal multipath
    MEDIUM = 1,  // base 48 samples (1.00ms @512) ->  96 / 2.00ms @1024 - moderate
    LONG = 2,    // base 64 samples (1.33ms @512) -> 128 / 2.67ms @1024 - strong multipath
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
    AUTO = 0xFF, // Let responder decide based on SNR (for CONNECT frame)
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

// Code rate to string
inline const char* codeRateToString(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return "R1/4";
        case CodeRate::R1_3: return "R1/3";
        case CodeRate::R1_2: return "R1/2";
        case CodeRate::R2_3: return "R2/3";
        case CodeRate::R3_4: return "R3/4";
        case CodeRate::R5_6: return "R5/6";
        case CodeRate::R7_8: return "R7/8";
        default: return "UNKNOWN";
    }
}

// Adaptive pilot configuration based on code rate
// Higher code rates use more pilots for better channel tracking on fading channels
// Lower code rates (R1/4) use no pilots - all carriers data, relies on LDPC redundancy
struct PilotConfig {
    int num_pilots = 0;                    // Number of pilot carriers (0, 4, or 6)
    std::vector<int> pilot_indices;        // Logical carrier indices for pilots (0 to num_carriers-1)

    // Get number of data carriers for a given total carrier count
    int getDataCarriers(int total_carriers) const {
        return total_carriers - num_pilots;
    }

    // Create pilot config for a given code rate
    // For wideband (59 carriers):
    //   R1/4: 0 pilots, max data, LDPC handles fading
    //   R1/2, R2/3: 6 pilots every ~10 carriers for per-symbol tracking
    //   R3/4: 4 pilots every ~15 carriers for light fading/AWGN
    // For narrowband (≤25 carriers):
    //   R1/4: 0 pilots (all 21 carriers are data)
    //   R1/2+: 3 pilots at {0, N/3, 2N/3} (18 data carriers)
    static PilotConfig forCodeRate(CodeRate rate, int num_carriers = 59) {
        PilotConfig cfg;

        // Narrowband: fewer carriers need different pilot layout
        if (num_carriers <= 25) {
            switch (rate) {
                case CodeRate::R1_4:
                case CodeRate::R1_3:
                default:
                    // 0 pilots - all carriers data, LDPC handles fading
                    cfg.num_pilots = 0;
                    cfg.pilot_indices.clear();
                    break;

                case CodeRate::R1_2:
                case CodeRate::R2_3:
                case CodeRate::R3_4:
                    // 3 pilots evenly spaced across narrowband carriers
                    cfg.num_pilots = 3;
                    cfg.pilot_indices = {0, num_carriers / 3, 2 * num_carriers / 3};
                    break;
            }
            return cfg;
        }

        // Wideband (59 carriers)
        switch (rate) {
            case CodeRate::R3_4:
                // 4 pilots for light fading/AWGN - every 15th carrier
                cfg.num_pilots = 4;
                cfg.pilot_indices = {0, 15, 30, 44};
                break;

            case CodeRate::R2_3:
            case CodeRate::R1_2:
                // 6 pilots for moderate fading - every 10th carrier
                cfg.num_pilots = 6;
                cfg.pilot_indices = {0, 10, 20, 30, 40, 50};
                break;

            case CodeRate::R1_4:
            case CodeRate::R1_3:
            default:
                // 0 pilots for heavy fading - all carriers data, rely on LDPC
                cfg.num_pilots = 0;
                cfg.pilot_indices.clear();
                break;
        }

        return cfg;
    }

    // Check if a logical carrier index is a pilot
    bool isPilot(int carrier_idx) const {
        for (int p : pilot_indices) {
            if (p == carrier_idx) return true;
        }
        return false;
    }
};

// Bandwidth mode for narrowband vs wideband operation
enum class BandwidthMode : uint8_t {
    WIDE = 0,    // Standard 2.8 kHz bandwidth (59 carriers)
    NARROW = 1,  // 500 Hz narrowband (21 carriers, FFT=2048)
};

// Channel quality estimate
struct ChannelQuality {
    float snr_db = 0.0f;           // Estimated SNR in dB
    float doppler_hz = 0.0f;       // Estimated Doppler spread
    float delay_spread_ms = 0.0f;  // Estimated multipath delay spread
    float ber_estimate = 0.0f;     // Estimated bit error rate
};

// Modem configuration
struct ModemConfig {
    // Audio parameters
    uint32_t sample_rate = 48000;      // Audio sample rate
    uint32_t center_freq = 1500;       // Center frequency in audio passband
    // With 30 carriers @ 93.75 Hz spacing, bandwidth = ±1406 Hz from center
    // At 1500 Hz: carriers span 94-2906 Hz (fits 2.8 kHz SSB filter)

    // OFDM parameters - optimized for 2.8 kHz HF channel with fading resilience
    // 1024 FFT at 48kHz = 46.875 Hz bin spacing (narrower = better fading performance)
    // 59 carriers × 46.875 Hz ≈ 2766 Hz (fills 2.8 kHz)
    uint32_t fft_size = 1024;          // FFT size (46.875 Hz bin spacing)
    uint32_t num_carriers = 59;        // Number of data carriers (2.8 kHz)

    // Adaptive cyclic prefix (key performance lever!)
    // MEDIUM at 1024 FFT = 96 samples = 2.00 ms.
    // CORRECTED 2026-07-28: this comment previously claimed "256 samples = 5.3ms" — wrong by
    // 2.67x. getCyclicPrefix() computes base_cp * (fft_size/512) = 48*2 = 96. The false 5.3 ms
    // figure matters: ITU Poor has a 2.0 ms second path, so production actually runs at ZERO
    // CP margin there, not the ~3 ms of headroom the old comment implied.
    CyclicPrefixMode cp_mode = CyclicPrefixMode::MEDIUM;
    uint32_t symbol_guard = 0;         // No extra guard needed with longer CP

    // Pilot configuration for frequency-selective channel estimation
    // Adaptive pilots: higher code rates use more pilots for channel tracking
    // R1/4: 0 pilots (all carriers data), R1/2+R2/3: 6 pilots, R3/4: 4 pilots
    PilotConfig pilot_config;          // Adaptive pilot placement
    uint32_t pilot_spacing = 2;        // Legacy: pilot every N carriers (when use_pilots=true)
    bool use_pilots = false;           // Legacy: false = all carriers are data (for DQPSK)
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

    // Output amplitude scaling
    // OFDM raw output has very low RMS (~0.01) due to FFT spreading
    // Scale up to match chirp/preamble levels for consistent audio output
    // Default 40x gives RMS ~0.4, matching chirp amplitude ~0.5
    float output_scale = 40.0f;  // OFDM TX level at the wideband geometry (1024 FFT, 59 carriers);
                                 // the modulator keeps the per-symbol power for other geometries

    // Chirp sync parameters (wideband defaults, backward compatible)
    // Narrowband mode overrides these to 1250-1750 Hz @ 1000ms
    float chirp_f_start = 300.0f;       // Chirp start frequency (Hz)
    float chirp_f_end = 2700.0f;        // Chirp end frequency (Hz)
    float chirp_duration_ms = 500.0f;   // Chirp duration (ms)

    // Simulation: TX carrier frequency offset (Hz)
    // Simulates radio tuning error - shifts ALL frequencies by this amount
    // Default 0 = no CFO. Set to e.g. ±20 Hz for testing CFO tolerance.
    float tx_cfo_hz = 0.0f;

    // Synchronization settings
    float sync_threshold = 0.80f;       // Correlation threshold for sync (0.7-0.95)

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

    // Estimate data carriers (excludes pilots if use_pilots=true)
    uint32_t getDataCarriers() const {
        if (num_carriers == 0) {
            return 0;
        }
        if (!use_pilots || pilot_spacing == 0) {
            return num_carriers;  // All carriers are data (for DQPSK)
        }
        const uint32_t pilots = (num_carriers + pilot_spacing - 1) / pilot_spacing;
        if (pilots >= num_carriers) {
            return 1;  // Keep a positive geometry valid for tiny custom configs.
        }
        return num_carriers - pilots;
    }

    // Calculate theoretical throughput in bps
    float getTheoreticalThroughput(Modulation mod, CodeRate rate) const {
        uint32_t bits_per_carrier = getBitsPerSymbol(mod);
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
// Uses 1024 FFT / 59 carriers (46.875 Hz spacing) for fading resilience
inline ModemConfig balanced() {
    ModemConfig cfg;  // Inherits 1024 FFT, 59 carriers, DQPSK, no pilots
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;    // 96 samples = 2.00 ms at 1024 FFT
    cfg.modulation = Modulation::DQPSK;        // Differential for HF
    cfg.code_rate = CodeRate::R1_2;            // R1/2 for typical conditions
    cfg.speed_profile = SpeedProfile::BALANCED;
    return cfg;
}

// Turbo: Maximum speed for excellent conditions (30+ dB SNR)
inline ModemConfig turbo() {
    ModemConfig cfg;
    cfg.cp_mode = CyclicPrefixMode::SHORT;     // 64 samples = 1.33 ms at the default 1024 FFT
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
    ModemConfig cfg;  // Inherits 1024 FFT, 59 carriers
    cfg.cp_mode = CyclicPrefixMode::MEDIUM;    // 256 samples (5.3ms) for 1024 FFT
    cfg.use_pilots = true;                     // Enable pilots for coherent modes
    cfg.pilot_spacing = 4;                     // 15 pilots, 44 data (25% overhead)
    cfg.modulation = Modulation::QAM16;
    cfg.code_rate = CodeRate::R2_3;            // R2/3 - robust enough for Good conds
    cfg.speed_profile = SpeedProfile::BALANCED;
    return cfg;
}

// High-speed mode for good conditions (25+ dB in-band SNR)
// Uses DQPSK R3/4 for higher throughput with default 1024 FFT / 59 carriers
inline ModemConfig high_speed() {
    ModemConfig cfg;  // Inherits 1024 FFT, 59 carriers, DQPSK, no pilots
    cfg.modulation = Modulation::DQPSK;
    cfg.code_rate = CodeRate::R3_4;            // R3/4 for good conditions
    cfg.speed_profile = SpeedProfile::TURBO;
    return cfg;
}

// Narrowband OFDM: 500 Hz bandwidth, FFT=2048, 21 carriers
// 7.5 dB noise bandwidth advantage over wideband (2.8 kHz)
// For low-SNR conditions (~13-20 dB in-band) where wideband doesn't work
inline ModemConfig narrowbandOFDM() {
    ModemConfig cfg;
    cfg.fft_size = 2048;                    // 23.4 Hz bin spacing
    cfg.num_carriers = 21;                  // 492 Hz occupied bandwidth
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500;
    cfg.cp_mode = CyclicPrefixMode::MEDIUM; // 192 samples = 4.0ms at 2048 FFT
    cfg.modulation = Modulation::DQPSK;     // Differential for fading
    cfg.code_rate = CodeRate::R1_4;         // Most robust
    cfg.use_pilots = true;
    cfg.pilot_spacing = 10;                 // Will be adjusted by PilotConfig
    cfg.chirp_f_start = 1250.0f;            // Narrowband chirp: 1250-1750 Hz
    cfg.chirp_f_end = 1750.0f;
    // 500 ms sweeps (2026-11-10, was 1000 ms): time-bandwidth product 250, ~24 dB of
    // processing gain, far above any narrowband rung's floor; CFO resolution 48 samples/Hz (5x
    // the wideband chirp's). Every coherent narrowband burst carries this anchor, so the long
    // sweeps cost 1.1 s of each ~14 s burst. The narrowband MC-DPSK handshake chirp (dual-listen,
    // mc_dpsk_presets::narrowband) keeps its 1000 ms sweeps.
    cfg.chirp_duration_ms = 500.0f;
    cfg.speed_profile = SpeedProfile::CONSERVATIVE;
    return cfg;
}

// Narrowband MC-DPSK: 4 carriers @ 1300-1700 Hz
// Used for PING/PONG/CONNECT handshake in narrowband sessions
inline ModemConfig narrowbandMCDPSK() {
    ModemConfig cfg;
    cfg.sample_rate = 48000;
    cfg.center_freq = 1500;
    cfg.chirp_f_start = 1250.0f;            // Narrowband chirp
    cfg.chirp_f_end = 1750.0f;
    cfg.chirp_duration_ms = 1000.0f;
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
