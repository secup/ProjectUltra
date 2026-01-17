/**
 * Basic Audio Path Test
 *
 * Tests raw audio without OFDM/LDPC complexity.
 * Use this to verify audio path between laptops.
 *
 * Usage:
 *   ./test_audio_basic                     # Run self-tests
 *   ./test_audio_basic --gen-tone out.wav  # Generate 1500 Hz tone
 *   ./test_audio_basic --gen-chirp out.wav # Generate chirp
 *   ./test_audio_basic --gen-pulse out.wav # Generate polarity pulse
 *   ./test_audio_basic --analyze tx.wav rx.wav  # Compare TX vs RX
 *
 * Tests:
 * 1. DC level (should be ~0)
 * 2. Simple tone generation and detection
 * 3. Polarity check (is audio inverted?)
 * 4. Sample pattern verification
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Generate a simple sine wave
std::vector<float> generateTone(float freq_hz, float duration_sec, float sample_rate = 48000.0f) {
    size_t num_samples = static_cast<size_t>(duration_sec * sample_rate);
    std::vector<float> samples(num_samples);

    for (size_t i = 0; i < num_samples; i++) {
        float t = i / sample_rate;
        samples[i] = 0.5f * std::sin(2.0f * M_PI * freq_hz * t);
    }
    return samples;
}

// Generate a chirp (frequency sweep) - good for detecting timing offset
std::vector<float> generateChirp(float f_start, float f_end, float duration_sec, float sample_rate = 48000.0f) {
    size_t num_samples = static_cast<size_t>(duration_sec * sample_rate);
    std::vector<float> samples(num_samples);

    float k = (f_end - f_start) / duration_sec;  // Chirp rate

    for (size_t i = 0; i < num_samples; i++) {
        float t = i / sample_rate;
        float phase = 2.0f * M_PI * (f_start * t + 0.5f * k * t * t);
        samples[i] = 0.5f * std::sin(phase);
    }
    return samples;
}

// Generate a simple pulse pattern for polarity detection
// Pattern: [silence][positive pulse][silence][negative pulse][silence]
std::vector<float> generatePolarityTest(float sample_rate = 48000.0f) {
    size_t silence_samples = static_cast<size_t>(0.01f * sample_rate);  // 10ms silence
    size_t pulse_samples = static_cast<size_t>(0.005f * sample_rate);   // 5ms pulse

    std::vector<float> samples;

    // Silence
    samples.insert(samples.end(), silence_samples, 0.0f);

    // Positive pulse (smooth ramp up, hold, ramp down)
    for (size_t i = 0; i < pulse_samples; i++) {
        float env = std::sin(M_PI * i / pulse_samples);  // Smooth envelope
        samples.push_back(0.5f * env);
    }

    // Silence
    samples.insert(samples.end(), silence_samples, 0.0f);

    // Negative pulse
    for (size_t i = 0; i < pulse_samples; i++) {
        float env = std::sin(M_PI * i / pulse_samples);
        samples.push_back(-0.5f * env);
    }

    // Silence
    samples.insert(samples.end(), silence_samples, 0.0f);

    return samples;
}

// Cross-correlation to find a pattern in received audio
// Returns (offset, correlation_value)
std::pair<int, float> findPattern(const std::vector<float>& haystack,
                                   const std::vector<float>& needle) {
    if (haystack.size() < needle.size()) {
        return {-1, 0.0f};
    }

    float best_corr = -1e9f;
    int best_offset = -1;

    // Normalize needle
    float needle_energy = 0;
    for (float s : needle) needle_energy += s * s;
    needle_energy = std::sqrt(needle_energy);

    for (size_t offset = 0; offset <= haystack.size() - needle.size(); offset++) {
        float corr = 0;
        float hay_energy = 0;

        for (size_t i = 0; i < needle.size(); i++) {
            corr += haystack[offset + i] * needle[i];
            hay_energy += haystack[offset + i] * haystack[offset + i];
        }

        hay_energy = std::sqrt(hay_energy);
        if (hay_energy > 0.001f && needle_energy > 0.001f) {
            corr /= (hay_energy * needle_energy);  // Normalized correlation
        }

        if (corr > best_corr) {
            best_corr = corr;
            best_offset = offset;
        }
    }

    return {best_offset, best_corr};
}

// Check if audio is inverted by looking at correlation sign
bool checkPolarity(const std::vector<float>& tx, const std::vector<float>& rx) {
    auto [offset, corr] = findPattern(rx, tx);

    std::cout << "  Correlation: " << corr << " at offset " << offset << std::endl;

    if (corr > 0.8f) {
        std::cout << "  Polarity: NORMAL (correlation positive)" << std::endl;
        return true;
    } else if (corr < -0.8f) {
        std::cout << "  Polarity: INVERTED (correlation negative)" << std::endl;
        return false;
    } else {
        std::cout << "  Polarity: UNCLEAR (correlation too weak)" << std::endl;
        return true;  // Assume normal
    }
}

// Measure DC offset
float measureDC(const std::vector<float>& samples) {
    if (samples.empty()) return 0;
    float sum = 0;
    for (float s : samples) sum += s;
    return sum / samples.size();
}

// Measure RMS level
float measureRMS(const std::vector<float>& samples) {
    if (samples.empty()) return 0;
    float sum_sq = 0;
    for (float s : samples) sum_sq += s * s;
    return std::sqrt(sum_sq / samples.size());
}

// Measure peak level
float measurePeak(const std::vector<float>& samples) {
    float peak = 0;
    for (float s : samples) peak = std::max(peak, std::abs(s));
    return peak;
}

// Print samples in hex-like format for visual inspection
void printSamples(const std::vector<float>& samples, size_t start, size_t count) {
    std::cout << "  Samples[" << start << "-" << (start + count - 1) << "]: ";
    for (size_t i = start; i < std::min(start + count, samples.size()); i++) {
        printf("%+.3f ", samples[i]);
    }
    std::cout << std::endl;
}

// ============================================================================
// SELF-TESTS (software loopback - no audio hardware)
// ============================================================================

void test_tone_generation() {
    std::cout << "\n=== Test: Tone Generation ===" << std::endl;

    auto tone = generateTone(1500.0f, 0.1f);  // 100ms of 1500 Hz

    std::cout << "  Generated " << tone.size() << " samples" << std::endl;
    std::cout << "  DC offset: " << measureDC(tone) << std::endl;
    std::cout << "  RMS level: " << measureRMS(tone) << std::endl;
    std::cout << "  Peak level: " << measurePeak(tone) << std::endl;

    // Self-correlation should be 1.0
    auto [offset, corr] = findPattern(tone, tone);
    std::cout << "  Self-correlation: " << corr << " at offset " << offset << std::endl;

    if (std::abs(corr - 1.0f) < 0.01f && offset == 0) {
        std::cout << "  [PASS] Tone generation OK" << std::endl;
    } else {
        std::cout << "  [FAIL] Tone generation issue" << std::endl;
    }
}

void test_polarity_detection() {
    std::cout << "\n=== Test: Polarity Detection ===" << std::endl;

    auto pulse = generatePolarityTest();
    std::cout << "  Generated " << pulse.size() << " samples" << std::endl;

    // Normal polarity
    std::cout << "\n  Testing normal polarity:" << std::endl;
    bool normal = checkPolarity(pulse, pulse);

    // Inverted polarity
    std::cout << "\n  Testing inverted polarity:" << std::endl;
    std::vector<float> inverted = pulse;
    for (float& s : inverted) s = -s;
    bool inverted_detected = !checkPolarity(pulse, inverted);

    if (normal && inverted_detected) {
        std::cout << "\n  [PASS] Polarity detection works" << std::endl;
    } else {
        std::cout << "\n  [FAIL] Polarity detection issue" << std::endl;
    }
}

void test_timing_offset_detection() {
    std::cout << "\n=== Test: Timing Offset Detection ===" << std::endl;

    auto chirp = generateChirp(500.0f, 2500.0f, 0.05f);  // 50ms chirp
    std::cout << "  Generated chirp: " << chirp.size() << " samples" << std::endl;

    // Add some silence before the chirp to simulate timing offset
    std::vector<float> delayed;
    size_t delay_samples = 500;  // ~10ms delay
    delayed.insert(delayed.end(), delay_samples, 0.0f);
    delayed.insert(delayed.end(), chirp.begin(), chirp.end());
    delayed.insert(delayed.end(), delay_samples, 0.0f);

    auto [offset, corr] = findPattern(delayed, chirp);

    std::cout << "  Expected offset: " << delay_samples << std::endl;
    std::cout << "  Detected offset: " << offset << std::endl;
    std::cout << "  Correlation: " << corr << std::endl;

    if (std::abs((int)offset - (int)delay_samples) <= 2 && corr > 0.95f) {
        std::cout << "  [PASS] Timing detection works" << std::endl;
    } else {
        std::cout << "  [FAIL] Timing detection issue" << std::endl;
    }
}

// ============================================================================
// AUDIO PATH TEST (for use with real audio)
// ============================================================================

void printTestSignalInfo() {
    std::cout << "\n=== Audio Path Test Signals ===" << std::endl;
    std::cout << "\nAvailable test signals for cross-wire testing:" << std::endl;
    std::cout << "  1. 1500 Hz tone (100ms) - basic audio path" << std::endl;
    std::cout << "  2. Chirp 500-2500 Hz (50ms) - timing offset detection" << std::endl;
    std::cout << "  3. Polarity pulse - check for audio inversion" << std::endl;
    std::cout << "\nTo use:" << std::endl;
    std::cout << "  - Generate signal on TX laptop" << std::endl;
    std::cout << "  - Record on RX laptop" << std::endl;
    std::cout << "  - Compare with cross-correlation" << std::endl;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "      BASIC AUDIO PATH TEST             " << std::endl;
    std::cout << "========================================" << std::endl;

    // Run self-tests
    test_tone_generation();
    test_polarity_detection();
    test_timing_offset_detection();

    // Print info about test signals
    printTestSignalInfo();

    std::cout << "\n========================================" << std::endl;
    std::cout << "      ALL BASIC TESTS COMPLETE          " << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
