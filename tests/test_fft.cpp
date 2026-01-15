#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "ultra/dsp.hpp"
#include <iostream>

int main() {
    std::cout << "Testing FFT implementation...\n";

    // Test various sizes
    for (size_t size : {64, 128, 256, 512, 1024}) {
        ultra::FFT fft(size);

        // Generate test signal: sine wave
        std::vector<ultra::Complex> input(size);
        std::vector<ultra::Complex> output(size);
        std::vector<ultra::Complex> roundtrip(size);

        for (size_t i = 0; i < size; ++i) {
            float t = static_cast<float>(i) / size;
            input[i] = ultra::Complex(std::sin(2 * M_PI * 4 * t), 0);
        }

        // Forward FFT
        fft.forward(input, output);

        // Inverse FFT
        fft.inverse(output, roundtrip);

        // Check round-trip error
        float max_error = 0;
        for (size_t i = 0; i < size; ++i) {
            float error = std::abs(input[i] - roundtrip[i]);
            if (error > max_error) max_error = error;
        }

        std::cout << "  Size " << size << ": round-trip error = " << max_error;
        if (max_error < 1e-5) {
            std::cout << " OK\n";
        } else {
            std::cout << " FAILED\n";
            return 1;
        }
    }

    // Test that we get expected peaks for known signals
    std::cout << "Testing spectral analysis...\n";

    ultra::FFT fft(256);
    std::vector<ultra::Complex> signal(256);
    std::vector<ultra::Complex> spectrum(256);

    // Create a signal at bin 8 (8 cycles in 256 samples)
    for (size_t i = 0; i < 256; ++i) {
        signal[i] = ultra::Complex(std::cos(2 * M_PI * 8 * i / 256.0), 0);
    }

    fft.forward(signal, spectrum);

    // Find peak
    size_t peak_bin = 0;
    float peak_mag = 0;
    for (size_t i = 0; i < 256; ++i) {
        float mag = std::abs(spectrum[i]);
        if (mag > peak_mag) {
            peak_mag = mag;
            peak_bin = i;
        }
    }

    std::cout << "  Expected peak at bin 8, got bin " << peak_bin;
    if (peak_bin == 8 || peak_bin == 248) {  // 248 = 256-8 (conjugate)
        std::cout << " OK\n";
    } else {
        std::cout << " FAILED\n";
        return 1;
    }

    std::cout << "\nAll FFT tests passed!\n";
    return 0;
}
