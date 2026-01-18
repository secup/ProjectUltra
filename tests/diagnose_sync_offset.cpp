/**
 * Sync Offset Diagnostic Tool
 *
 * This tool investigates WHY the correlation peak is offset from the
 * true preamble start. We need to understand this, not just apply a
 * magic correction constant.
 *
 * Investigation approach:
 * 1. Generate a known signal with preamble at exact position
 * 2. Profile correlation at EVERY sample position
 * 3. Find where peak actually occurs vs where we expect it
 * 4. Analyze the preamble structure to understand the offset
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

// Replicate the correlation algorithm from demodulator.cpp to understand it
class CorrelationAnalyzer {
public:
    CorrelationAnalyzer(const ModemConfig& config) : config_(config), fft_(512) {
        symbol_len_ = config.fft_size + config.getCyclicPrefix();  // 512 + 48 = 560
    }

    // Convert real signal to analytic signal (same as demodulator)
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

        // Create analytic signal
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

    // Measure correlation at a specific offset (same as demodulator)
    float measureCorrelation(const std::vector<float>& buffer, size_t offset) {
        if (offset + symbol_len_ * 2 > buffer.size()) return 0.0f;

        auto analytic = toAnalytic(&buffer[offset], symbol_len_ * 2);

        Complex P(0.0f, 0.0f);
        float R = 0.0f;

        for (size_t i = 0; i < symbol_len_; ++i) {
            P += std::conj(analytic[i]) * analytic[i + symbol_len_];
            R += std::norm(analytic[i + symbol_len_]);
        }

        return std::abs(P) / (R + 1e-10f);
    }

    size_t getSymbolLen() const { return symbol_len_; }

private:
    ModemConfig config_;
    FFT fft_;
    size_t symbol_len_;
};

void analyzeCorrelationProfile(size_t preamble_start) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  CORRELATION PROFILE ANALYSIS" << std::endl;
    std::cout << "  Preamble starts at sample: " << preamble_start << std::endl;
    std::cout << "========================================\n" << std::endl;

    ModemConfig config;
    config.modulation = Modulation::BPSK;

    OFDMModulator mod(config);
    CorrelationAnalyzer analyzer(config);

    // Generate preamble
    Samples preamble = mod.generatePreamble();
    Bytes test_data(81, 0x00);
    Samples data = mod.modulate(test_data, config.modulation);

    std::cout << "Preamble length: " << preamble.size() << " samples" << std::endl;
    std::cout << "Symbol length (FFT+CP): " << analyzer.getSymbolLen() << " samples" << std::endl;
    std::cout << "Cyclic prefix: " << config.getCyclicPrefix() << " samples" << std::endl;
    std::cout << "Expected preamble structure: 4×STS + 2×LTS = 6 × "
              << analyzer.getSymbolLen() << " = " << 6 * analyzer.getSymbolLen() << " samples" << std::endl;

    // Build signal: [silence][preamble][data]
    std::vector<float> signal(preamble_start, 0.0f);
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), data.begin(), data.end());

    // Scale
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : signal) s *= 0.5f / max_val;
    }

    std::cout << "\nTotal signal length: " << signal.size() << " samples" << std::endl;

    // Profile correlation around the preamble start
    std::cout << "\n--- Correlation profile around preamble start ---" << std::endl;
    std::cout << "Scanning from " << (preamble_start > 200 ? preamble_start - 200 : 0)
              << " to " << preamble_start + 200 << std::endl;

    size_t scan_start = preamble_start > 200 ? preamble_start - 200 : 0;
    size_t scan_end = std::min(preamble_start + 200, signal.size() - analyzer.getSymbolLen() * 2);

    float max_corr = 0;
    size_t max_corr_offset = 0;
    std::vector<std::pair<size_t, float>> correlation_profile;

    for (size_t offset = scan_start; offset <= scan_end; ++offset) {
        float corr = analyzer.measureCorrelation(signal, offset);
        correlation_profile.push_back({offset, corr});

        if (corr > max_corr) {
            max_corr = corr;
            max_corr_offset = offset;
        }
    }

    // Print profile around the peak and expected position
    std::cout << "\nCorrelation values around TRUE preamble start (" << preamble_start << "):" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    for (int delta = -20; delta <= 20; delta += 2) {
        size_t pos = preamble_start + delta;
        if (pos >= scan_start && pos <= scan_end) {
            float corr = analyzer.measureCorrelation(signal, pos);
            std::cout << "  offset " << std::setw(6) << pos
                      << " (delta=" << std::setw(4) << delta << "): corr=" << corr;
            if (pos == preamble_start) std::cout << " <-- TRUE START";
            if (pos == max_corr_offset) std::cout << " <-- PEAK";
            std::cout << std::endl;
        }
    }

    std::cout << "\nCorrelation values around detected PEAK (" << max_corr_offset << "):" << std::endl;
    for (int delta = -20; delta <= 20; delta += 2) {
        size_t pos = max_corr_offset + delta;
        if (pos >= scan_start && pos <= scan_end) {
            float corr = analyzer.measureCorrelation(signal, pos);
            std::cout << "  offset " << std::setw(6) << pos
                      << " (delta=" << std::setw(4) << delta << "): corr=" << corr;
            if (pos == preamble_start) std::cout << " <-- TRUE START";
            if (pos == max_corr_offset) std::cout << " <-- PEAK";
            std::cout << std::endl;
        }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "  RESULTS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "True preamble start: " << preamble_start << std::endl;
    std::cout << "Correlation peak at: " << max_corr_offset << std::endl;
    std::cout << "Peak correlation value: " << max_corr << std::endl;
    std::cout << "OFFSET ERROR: " << (int)(max_corr_offset - preamble_start) << " samples" << std::endl;
    std::cout << "Offset in ms: " << (max_corr_offset - preamble_start) * 1000.0f / 48000.0f << " ms" << std::endl;
    std::cout << "Offset as fraction of CP: " << (float)(max_corr_offset - preamble_start) / config.getCyclicPrefix() << std::endl;
    std::cout << "Offset as fraction of symbol: " << (float)(max_corr_offset - preamble_start) / analyzer.getSymbolLen() << std::endl;
}

void analyzePreambleStructure() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  PREAMBLE STRUCTURE ANALYSIS" << std::endl;
    std::cout << "========================================\n" << std::endl;

    ModemConfig config;
    OFDMModulator mod(config);

    Samples preamble = mod.generatePreamble();

    size_t symbol_len = config.fft_size + config.getCyclicPrefix();  // 560
    size_t cp_len = config.getCyclicPrefix();  // 48

    std::cout << "Preamble total: " << preamble.size() << " samples" << std::endl;
    std::cout << "Symbol length: " << symbol_len << " samples" << std::endl;
    std::cout << "Expected: 6 symbols × " << symbol_len << " = " << 6 * symbol_len << std::endl;

    // Check periodicity within preamble
    std::cout << "\n--- Checking STS periodicity ---" << std::endl;
    std::cout << "STS should be 4 identical symbols of " << symbol_len << " samples each" << std::endl;

    // Compare STS symbols
    for (int sym = 0; sym < 3; ++sym) {
        float diff = 0;
        for (size_t i = 0; i < symbol_len; ++i) {
            float d = preamble[sym * symbol_len + i] - preamble[(sym + 1) * symbol_len + i];
            diff += d * d;
        }
        diff = std::sqrt(diff / symbol_len);
        std::cout << "  STS" << sym << " vs STS" << (sym+1) << " RMS diff: " << diff << std::endl;
    }

    // Check CP structure (CP should be copy of end of symbol)
    std::cout << "\n--- Checking CP structure ---" << std::endl;
    for (int sym = 0; sym < 4; ++sym) {
        size_t sym_start = sym * symbol_len;
        float diff = 0;
        for (size_t i = 0; i < cp_len; ++i) {
            // CP is at start of symbol, should match end of FFT portion
            float cp_val = preamble[sym_start + i];
            float end_val = preamble[sym_start + config.fft_size + i];
            float d = cp_val - end_val;
            diff += d * d;
        }
        diff = std::sqrt(diff / cp_len);
        std::cout << "  Symbol " << sym << " CP matches end: RMS diff = " << diff << std::endl;
    }

    // Analyze correlation window alignment
    std::cout << "\n--- Correlation window analysis ---" << std::endl;
    std::cout << "The correlation compares samples[0:" << symbol_len << "] with samples["
              << symbol_len << ":" << 2*symbol_len << "]" << std::endl;
    std::cout << "At preamble start (offset=0):" << std::endl;
    std::cout << "  Window 1: STS symbol 0 [CP0 + FFT0]" << std::endl;
    std::cout << "  Window 2: STS symbol 1 [CP1 + FFT1]" << std::endl;
    std::cout << "  Since STS0 == STS1, correlation should be ~1.0" << std::endl;
}

void testMultiplePreamblePositions() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "  TESTING MULTIPLE PREAMBLE POSITIONS" << std::endl;
    std::cout << "========================================\n" << std::endl;

    ModemConfig config;
    config.modulation = Modulation::BPSK;

    std::vector<size_t> test_positions = {0, 100, 500, 1000, 2000, 5000};

    for (size_t true_start : test_positions) {
        OFDMModulator mod(config);
        CorrelationAnalyzer analyzer(config);

        Samples preamble = mod.generatePreamble();
        Bytes test_data(81, 0x00);
        Samples data = mod.modulate(test_data, config.modulation);

        std::vector<float> signal(true_start, 0.0f);
        signal.insert(signal.end(), preamble.begin(), preamble.end());
        signal.insert(signal.end(), data.begin(), data.end());

        float max_val = 0;
        for (float s : signal) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) {
            for (float& s : signal) s *= 0.5f / max_val;
        }

        // Find correlation peak
        size_t scan_start = true_start > 150 ? true_start - 150 : 0;
        size_t scan_end = std::min(true_start + 150, signal.size() - analyzer.getSymbolLen() * 2);

        float max_corr = 0;
        size_t max_corr_offset = 0;

        for (size_t offset = scan_start; offset <= scan_end; ++offset) {
            float corr = analyzer.measureCorrelation(signal, offset);
            if (corr > max_corr) {
                max_corr = corr;
                max_corr_offset = offset;
            }
        }

        int error = static_cast<int>(max_corr_offset) - static_cast<int>(true_start);
        std::cout << "True start: " << std::setw(5) << true_start
                  << " | Peak at: " << std::setw(5) << max_corr_offset
                  << " | Error: " << std::setw(4) << error
                  << " | Corr: " << std::fixed << std::setprecision(4) << max_corr
                  << std::endl;
    }
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "    SYNC OFFSET ROOT CAUSE ANALYSIS" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Investigating WHY correlation peak is offset" << std::endl;
    std::cout << "from true preamble start position." << std::endl;

    analyzePreambleStructure();
    testMultiplePreamblePositions();
    analyzeCorrelationProfile(1000);

    return 0;
}
