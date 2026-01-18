/**
 * Detailed Correlation Analysis
 *
 * Investigates WHY correlation exceeds 1.0 and WHY the peak is at +91 samples.
 * This is critical - we need to understand the root cause, not apply magic constants.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <iomanip>

#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/types.hpp"

using namespace ultra;

class DetailedAnalyzer {
public:
    DetailedAnalyzer() : fft_(512) {}

    // Analyze WHY correlation > 1.0
    void analyzeCorrelationFormula(const std::vector<float>& signal, size_t offset, size_t len) {
        std::cout << "\n=== Detailed correlation analysis at offset " << offset << " ===" << std::endl;

        if (offset + len * 2 > signal.size()) {
            std::cout << "Offset out of range" << std::endl;
            return;
        }

        // Create analytic signal
        auto analytic = toAnalytic(&signal[offset], len * 2);

        // Compute correlation components
        Complex P(0.0f, 0.0f);
        float R = 0.0f;
        float E1 = 0.0f;  // Energy of first window

        for (size_t i = 0; i < len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + len];
            R += std::norm(analytic[i + len]);
            E1 += std::norm(analytic[i]);
        }

        float corr = std::abs(P) / (R + 1e-10f);

        std::cout << "  Window length: " << len << " samples" << std::endl;
        std::cout << "  E1 (energy of window 1): " << E1 << std::endl;
        std::cout << "  R (energy of window 2): " << R << std::endl;
        std::cout << "  E1/R ratio: " << E1/R << std::endl;
        std::cout << "  |P| (correlation magnitude): " << std::abs(P) << std::endl;
        std::cout << "  |P|/R (our correlation): " << corr << std::endl;
        std::cout << "  |P|²/R² (Schmidl-Cox metric): " << (std::abs(P)*std::abs(P))/(R*R) << std::endl;
        std::cout << "  |P|/sqrt(E1*R) (proper normalized): " << std::abs(P)/std::sqrt(E1*R) << std::endl;

        // Check if the issue is energy imbalance
        if (E1 > R * 1.001) {
            std::cout << "  WARNING: Window 1 has MORE energy than Window 2!" << std::endl;
            std::cout << "  This could cause correlation > 1.0" << std::endl;
        }
    }

    // Test without Hilbert transform (direct real correlation)
    float measureRealCorrelation(const std::vector<float>& signal, size_t offset, size_t len) {
        if (offset + len * 2 > signal.size()) return 0.0f;

        float P = 0.0f;  // Real correlation
        float R1 = 0.0f; // Energy of window 1
        float R2 = 0.0f; // Energy of window 2

        for (size_t i = 0; i < len; ++i) {
            P += signal[offset + i] * signal[offset + i + len];
            R1 += signal[offset + i] * signal[offset + i];
            R2 += signal[offset + i + len] * signal[offset + i + len];
        }

        // Proper normalization: P / sqrt(R1 * R2)
        return P / (std::sqrt(R1 * R2) + 1e-10f);
    }

    // Test with analytic signal but proper normalization
    float measureAnalyticCorrelationProper(const std::vector<float>& signal, size_t offset, size_t len) {
        if (offset + len * 2 > signal.size()) return 0.0f;

        auto analytic = toAnalytic(&signal[offset], len * 2);

        Complex P(0.0f, 0.0f);
        float E1 = 0.0f;
        float E2 = 0.0f;

        for (size_t i = 0; i < len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + len];
            E1 += std::norm(analytic[i]);
            E2 += std::norm(analytic[i + len]);
        }

        // Proper normalization: |P| / sqrt(E1 * E2)
        return std::abs(P) / (std::sqrt(E1 * E2) + 1e-10f);
    }

    // Current algorithm (for comparison)
    float measureCurrentCorrelation(const std::vector<float>& signal, size_t offset, size_t len) {
        if (offset + len * 2 > signal.size()) return 0.0f;

        auto analytic = toAnalytic(&signal[offset], len * 2);

        Complex P(0.0f, 0.0f);
        float R = 0.0f;

        for (size_t i = 0; i < len; ++i) {
            P += std::conj(analytic[i]) * analytic[i + len];
            R += std::norm(analytic[i + len]);
        }

        return std::abs(P) / (R + 1e-10f);
    }

    std::vector<Complex> toAnalytic(const float* samples, size_t len) {
        size_t fft_len = 1;
        while (fft_len < len) fft_len *= 2;

        FFT hilbert_fft(fft_len);

        std::vector<Complex> time_in(fft_len, Complex(0, 0));
        for (size_t i = 0; i < len; ++i) {
            time_in[i] = Complex(samples[i], 0);
        }

        std::vector<Complex> freq(fft_len);
        hilbert_fft.forward(time_in, freq);

        for (size_t i = 1; i < fft_len / 2; ++i) {
            freq[i] *= 2.0f;
        }
        for (size_t i = fft_len / 2 + 1; i < fft_len; ++i) {
            freq[i] = Complex(0, 0);
        }

        std::vector<Complex> analytic(fft_len);
        hilbert_fft.inverse(freq, analytic);

        analytic.resize(len);
        return analytic;
    }

private:
    FFT fft_;
};

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  DETAILED CORRELATION INVESTIGATION" << std::endl;
    std::cout << "========================================\n" << std::endl;

    ModemConfig config;
    config.modulation = Modulation::BPSK;

    OFDMModulator mod(config);
    DetailedAnalyzer analyzer;

    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0x00);
    Samples data = mod.modulate(test_data, config.modulation);

    size_t preamble_start = 1000;
    size_t symbol_len = config.fft_size + config.getCyclicPrefix();  // 560

    std::vector<float> signal(preamble_start, 0.0f);
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), data.begin(), data.end());

    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : signal) s *= 0.5f / max_val;
    }

    // Analyze at true start and at +91
    analyzer.analyzeCorrelationFormula(signal, preamble_start, symbol_len);
    analyzer.analyzeCorrelationFormula(signal, preamble_start + 91, symbol_len);

    // Compare different correlation methods
    std::cout << "\n=== Comparison of correlation methods ===" << std::endl;
    std::cout << "Offset\tCurrent\t\tProper(analytic)\tReal(no Hilbert)" << std::endl;

    for (int delta = -20; delta <= 120; delta += 10) {
        size_t offset = preamble_start + delta;
        float current = analyzer.measureCurrentCorrelation(signal, offset, symbol_len);
        float proper = analyzer.measureAnalyticCorrelationProper(signal, offset, symbol_len);
        float real_corr = analyzer.measureRealCorrelation(signal, offset, symbol_len);

        std::cout << std::fixed << std::setprecision(4);
        std::cout << delta << "\t" << current << "\t\t" << proper << "\t\t\t" << real_corr << std::endl;
    }

    // Find peaks for each method
    std::cout << "\n=== Finding peaks for each method ===" << std::endl;

    float max_current = 0, max_proper = 0, max_real = 0;
    int peak_current = 0, peak_proper = 0, peak_real = 0;

    for (int delta = -50; delta <= 150; delta++) {
        size_t offset = preamble_start + delta;
        float current = analyzer.measureCurrentCorrelation(signal, offset, symbol_len);
        float proper = analyzer.measureAnalyticCorrelationProper(signal, offset, symbol_len);
        float real_corr = analyzer.measureRealCorrelation(signal, offset, symbol_len);

        if (current > max_current) { max_current = current; peak_current = delta; }
        if (proper > max_proper) { max_proper = proper; peak_proper = delta; }
        if (real_corr > max_real) { max_real = real_corr; peak_real = delta; }
    }

    std::cout << "Current method: peak at delta=" << peak_current << " (corr=" << max_current << ")" << std::endl;
    std::cout << "Proper analytic: peak at delta=" << peak_proper << " (corr=" << max_proper << ")" << std::endl;
    std::cout << "Real correlation: peak at delta=" << peak_real << " (corr=" << max_real << ")" << std::endl;

    if (peak_current != 0) {
        std::cout << "\n*** ISSUE: Current method peak is NOT at true preamble start! ***" << std::endl;
        std::cout << "*** The offset of " << peak_current << " is an artifact of the algorithm ***" << std::endl;
    }

    if (peak_proper == 0 || peak_real == 0) {
        std::cout << "\n*** SOLUTION: Using proper normalization fixes the peak position! ***" << std::endl;
    }

    return 0;
}
