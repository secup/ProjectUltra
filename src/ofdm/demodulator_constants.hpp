#pragma once

// Demodulator constants - centralized for maintainability
// All magic numbers from demodulator.cpp extracted here

#include <cstddef>

namespace ultra {
namespace demod_constants {

// =============================================================================
// LDPC
// =============================================================================
constexpr size_t LDPC_BLOCK_SIZE = 648;

// =============================================================================
// LLR LIMITS
// =============================================================================
// LLR clipping to prevent overconfident decisions
// On fading channels, channel estimation errors can produce huge LLRs
// that are confidently WRONG. Clipping allows LDPC to correct them.
constexpr float MAX_LLR = 10.0f;

// Minimum LLR magnitude to prevent complete erasures
// When symbol is near decision boundary, give weak opinion for LDPC
constexpr float MIN_LLR_MAG = 0.5f;

// =============================================================================
// PILOT SEQUENCE
// =============================================================================
constexpr uint32_t PILOT_RNG_SEED = 0x50494C54;  // "PILT" in ASCII

// =============================================================================
// SYNC DETECTION
// =============================================================================
// Timeout limits
constexpr int MAX_SYMBOLS_BEFORE_TIMEOUT = 250;  // Support larger frames (up to ~1KB)
constexpr int MAX_IDLE_CALLS_BEFORE_RESET = 10;  // Reset after 10 calls with no new soft bits

// Buffer management
constexpr size_t MIN_SEARCH_SAMPLES = 4000;   // Minimum buffer before searching
constexpr size_t MAX_BUFFER_SAMPLES = 240000; // ~5 seconds at 48kHz
constexpr size_t OVERLAP_SAMPLES = 20000;     // Keep for boundary-spanning preambles (~400ms PTT)

// Noise floor tracking for amplitude-independent sync detection
constexpr float NOISE_FLOOR_ALPHA = 0.01f;  // Slow adaptation for noise floor
constexpr float SIGNAL_TO_NOISE_RATIO_THRESHOLD = 6.0f;  // Require 6x noise floor (~8 dB)

// Preamble search
constexpr size_t SEARCH_STEP_SIZE = 8;  // Check every 8th sample
constexpr float PLATEAU_THRESHOLD = 0.90f;
constexpr size_t PLATEAU_SEARCH_WINDOW = 300;
constexpr size_t MIN_PLATEAU_SAMPLES = 15;

// Energy thresholds
constexpr float MIN_ENERGY_THRESHOLD = 0.1f;  // For correlation normalization
constexpr float MIN_ABSOLUTE_ENERGY = 1e-10f;  // Very small - let noise floor tracking do its job
constexpr float ENERGY_RATIO_THRESHOLD = 4.0f;  // Signal should be 4x above noise floor (~6 dB)

// =============================================================================
// FREQUENCY OFFSET TRACKING
// =============================================================================
constexpr float FREQ_OFFSET_ALPHA = 0.3f;  // Smoothing factor for tracking (after lock)
constexpr int CFO_ACQUISITION_SYMBOLS = 10;  // Use fast alpha for first N symbols
constexpr float MAX_CFO_HZ = 90.0f;  // Clamp to Schmidl-Cox range

// =============================================================================
// TIMING RECOVERY
// =============================================================================
constexpr float TIMING_ALPHA = 0.3f;  // Smoothing factor for timing estimate

// =============================================================================
// CHANNEL ESTIMATION
// =============================================================================
// Phase difference threshold for interpolation vs nearest-pilot
constexpr float PHASE_INTERPOLATION_THRESHOLD = 1.5708f;  // π/2 = 90°

// Default SNR assumption for first symbol (15 dB = 31.6 linear)
constexpr float DEFAULT_SNR_LINEAR = 31.6f;

// Fade detection threshold (10 dB below average = 0.1 * average)
constexpr float FADE_THRESHOLD_RATIO = 0.1f;

// =============================================================================
// SOFT DEMAPPING
// =============================================================================
// QAM normalization factors
constexpr float QPSK_SCALE = 0.7071067811865476f;  // 1/sqrt(2)
constexpr float QAM16_THRESHOLD = 0.6324555320336759f;  // 2/sqrt(10)
constexpr float QAM32_SCALE = 0.1961161351381840f;  // 1/sqrt(26)
constexpr float QAM64_D2 = 0.3086067f;  // 2/sqrt(42)
constexpr float QAM64_D4 = 0.6172134f;  // 4/sqrt(42)
constexpr float QAM256_D2 = 0.1290994f;  // 2/sqrt(170)
constexpr float QAM256_D4 = 0.2581989f;  // 4/sqrt(170)
constexpr float QAM256_D8 = 0.5163978f;  // 8/sqrt(170)

// =============================================================================
// CHANNEL ESTIMATION ERROR MARGINS
// =============================================================================
// For higher-order modulations, add channel estimation error margin
// Residual CE error acts like additional noise
constexpr float CE_MARGIN_BPSK_QPSK = 1.0f;
constexpr float CE_MARGIN_8PSK = 1.1f;
constexpr float CE_MARGIN_QAM16 = 1.2f;
constexpr float CE_MARGIN_QAM32 = 1.5f;
constexpr float CE_MARGIN_QAM64 = 1.8f;
constexpr float CE_MARGIN_QAM256 = 2.5f;

// =============================================================================
// ADAPTIVE EQUALIZER
// =============================================================================
constexpr float ADAPTIVE_EQ_P_MIN = 0.001f;
constexpr float ADAPTIVE_EQ_P_MAX = 1000.0f;

// Noise variance limits for soft demapping
constexpr float MIN_CARRIER_NOISE_VAR = 1e-6f;
constexpr float MAX_CARRIER_NOISE_VAR = 100.0f;

// =============================================================================
// CONSTELLATION DISPLAY
// =============================================================================
constexpr size_t MAX_CONSTELLATION_SYMBOLS = 500;

// =============================================================================
// HILBERT TRANSFORM (for analytic signal)
// =============================================================================
constexpr size_t SAMPLE_STEP_ENERGY_CHECK = 16;  // Sample every 16th point for speed

} // namespace demod_constants
} // namespace ultra
