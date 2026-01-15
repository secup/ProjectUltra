#pragma once

#include "types.hpp"
#include <memory>
#include <vector>
#include <complex>

namespace ultra {

/**
 * OTFS (Orthogonal Time Frequency Space) Modulator
 *
 * OTFS is a 2D modulation scheme designed for doubly-selective channels
 * (high delay spread + high Doppler spread). It maps data symbols to a
 * delay-Doppler grid, where the channel appears sparse and quasi-static.
 *
 * Signal processing chain:
 *   Data → DD grid → ISFFT → TF grid → OFDM mod (IFFT+CP) → TX
 *
 * Key parameters:
 *   M = subcarriers (delay resolution)
 *   N = OFDM symbols per frame (Doppler resolution)
 *
 * Advantages over OFDM:
 *   - Each symbol sees average channel (not individual fades)
 *   - Robust to frequency-selective fading (Poor channel)
 *   - Robust to fast time-varying fading (Flutter channel)
 *
 * Reference: "Orthogonal Time Frequency Space Modulation" (2017)
 */

struct OTFSConfig {
    uint32_t M = 32;           // Delay bins (subcarriers per symbol)
    uint32_t N = 8;            // Doppler bins (OFDM symbols per frame)
    uint32_t fft_size = 512;   // FFT size for OFDM
    uint32_t cp_length = 64;   // Cyclic prefix length
    uint32_t sample_rate = 48000;
    float center_freq = 1500.0f;

    // Derived parameters
    uint32_t frame_symbols() const { return N; }
    uint32_t subcarriers() const { return M; }
    uint32_t total_data_symbols() const { return M * N; }
};

class OTFSModulator {
public:
    explicit OTFSModulator(const OTFSConfig& config);
    ~OTFSModulator();

    // Modulate data symbols in delay-Doppler grid to audio samples
    // Input: M*N complex symbols (row-major: symbols[k*N + l] = DD[k,l])
    // Output: Audio samples for entire OTFS frame
    Samples modulate(const std::vector<Complex>& dd_symbols, Modulation mod);

    // Map data bytes to DD symbols (constellation mapping)
    std::vector<Complex> mapToDD(ByteSpan data, Modulation mod);

    // Generate preamble for frame synchronization
    Samples generatePreamble();

    // Get number of data symbols per frame
    size_t symbolsPerFrame() const;

    // Get bits per frame at given modulation
    size_t bitsPerFrame(Modulation mod) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

class OTFSDemodulator {
public:
    explicit OTFSDemodulator(const OTFSConfig& config);
    ~OTFSDemodulator();

    // Process incoming samples, returns true if frame ready
    bool process(SampleSpan samples);

    // Get demodulated DD symbols (after SFFT)
    std::vector<Complex> getDDSymbols();

    // Get soft bits for FEC decoder
    std::vector<float> getSoftBits();

    // Get estimated channel in DD domain (sparse!)
    std::vector<Complex> getDDChannel();

    // Reset state
    void reset();

    // Check if synchronized
    bool isSynced() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * ISFFT - Inverse Symplectic Finite Fourier Transform
 *
 * Converts delay-Doppler domain to time-frequency domain:
 *   X_tf[n,m] = (1/sqrt(MN)) * sum_k sum_l X_dd[k,l] * exp(j2π(nk/N - ml/M))
 *
 * Decomposed as:
 *   1. IFFT along columns (Doppler → time)
 *   2. FFT along rows (delay → frequency)
 */
void isfft(const std::vector<Complex>& dd_grid,
           std::vector<Complex>& tf_grid,
           uint32_t M, uint32_t N);

/**
 * SFFT - Symplectic Finite Fourier Transform
 *
 * Converts time-frequency domain to delay-Doppler domain:
 *   X_dd[k,l] = (1/sqrt(MN)) * sum_n sum_m X_tf[n,m] * exp(-j2π(nk/N - ml/M))
 *
 * Decomposed as:
 *   1. FFT along columns (time → Doppler)
 *   2. IFFT along rows (frequency → delay)
 */
void sfft(const std::vector<Complex>& tf_grid,
          std::vector<Complex>& dd_grid,
          uint32_t M, uint32_t N);

} // namespace ultra
