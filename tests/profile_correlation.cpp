/**
 * Profile the correlation around the true preamble start
 * to understand what the fix actually changed
 */
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// Replicate the FIXED correlation algorithm
class FixedAnalyzer {
public:
    FixedAnalyzer(const ModemConfig& config) : config_(config), fft_(512) {
        symbol_len_ = config.fft_size + config.getCyclicPrefix();
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

        for (size_t i = 1; i < fft_len / 2; ++i) freq[i] *= 2.0f;
        for (size_t i = fft_len / 2 + 1; i < fft_len; ++i) freq[i] = Complex(0, 0);

        std::vector<Complex> analytic(fft_len);
        hilbert_fft.inverse(freq, analytic);
        analytic.resize(len);
        return analytic;
    }

    // FIXED correlation (proper normalization)
    float measureCorrelationFixed(const std::vector<float>& buffer, size_t offset) {
        if (offset + symbol_len_ * 2 > buffer.size()) return 0.0f;

        auto analytic = toAnalytic(&buffer[offset], symbol_len_ * 2);

        Complex P(0.0f, 0.0f);
        float E1 = 0.0f, E2 = 0.0f;

        for (size_t i = 0; i < symbol_len_; ++i) {
            P += std::conj(analytic[i]) * analytic[i + symbol_len_];
            E1 += std::norm(analytic[i]);
            E2 += std::norm(analytic[i + symbol_len_]);
        }

        return std::abs(P) / (std::sqrt(E1 * E2) + 1e-10f);
    }

    size_t getSymbolLen() const { return symbol_len_; }

private:
    ModemConfig config_;
    FFT fft_;
    size_t symbol_len_;
};

int main() {
    std::cout << "=== CORRELATION PROFILE WITH FIX ===\n" << std::endl;

    ModemConfig config;
    FixedAnalyzer analyzer(config);

    OFDMModulator mod(config);
    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0x00);
    Samples data = mod.modulate(test_data, config.modulation);

    size_t preamble_start = 1000;

    std::vector<float> signal(preamble_start, 0.0f);
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), data.begin(), data.end());

    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    std::cout << "Preamble starts at: " << preamble_start << std::endl;
    std::cout << "Symbol length: " << analyzer.getSymbolLen() << std::endl;
    std::cout << std::endl;

    // Profile around true start
    float max_corr = 0;
    int max_delta = 0;

    std::cout << "Delta\tCorrelation" << std::endl;
    std::cout << "-----\t-----------" << std::endl;

    for (int delta = -50; delta <= 150; delta += 5) {
        size_t offset = preamble_start + delta;
        float corr = analyzer.measureCorrelationFixed(signal, offset);

        if (corr > max_corr) {
            max_corr = corr;
            max_delta = delta;
        }

        std::cout << std::fixed << std::setprecision(4);
        if (delta == 0) {
            std::cout << delta << "\t" << corr << " <-- TRUE START" << std::endl;
        } else if (delta == max_delta) {
            std::cout << delta << "\t" << corr << " <-- LOCAL MAX" << std::endl;
        } else {
            std::cout << delta << "\t" << corr << std::endl;
        }
    }

    std::cout << "\n=== RESULT ===" << std::endl;
    std::cout << "Peak correlation at delta = " << max_delta << std::endl;
    std::cout << "Peak value = " << max_corr << std::endl;
    std::cout << "Expected delta = 0" << std::endl;
    std::cout << "Error = " << max_delta << " samples" << std::endl;

    return 0;
}
