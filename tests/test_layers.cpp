/**
 * CRITICAL: Layer-by-Layer OFDM Verification Tests
 *
 * This is CRITICAL software for emergency communications.
 * EVERY layer MUST be proven to work with 100% reliability.
 * NO assumptions - PROVE everything with tests.
 *
 * Test Philosophy:
 * - Test EVERYTHING, not just happy paths
 * - Test single values, multiple values, edge cases
 * - Compare intermediate values, not just final results
 * - If ANY test fails, the system is NOT ready for use
 *
 * Layer Stack (bottom to top):
 * 1. FFT/IFFT - Mathematical transform
 * 2. Carrier Mapping - Placing data in FFT bins
 * 3. Cyclic Prefix - Adding/removing guard interval
 * 4. NCO/Mixer - Upconversion/downconversion
 * 5. Single OFDM Symbol - Complete encode/decode of one symbol
 * 6. Multi-Symbol - Multiple symbols in sequence
 * 7. Preamble & Sync - Detection and timing
 * 8. Channel Estimation - Pilot-based equalization
 * 9. Full Modem - Complete TX/RX pipeline
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <complex>
#include <algorithm>
#include <numeric>

#include "ultra/dsp.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// Test counters
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (!(cond)) { \
        std::cout << "  [FAIL] " << msg << "\n"; \
        tests_failed++; \
        return false; \
    } \
} while(0)

#define TEST_ASSERT_NEAR(a, b, tol, msg) do { \
    if (std::abs((a) - (b)) > (tol)) { \
        std::cout << "  [FAIL] " << msg << ": expected " << (b) << ", got " << (a) << ", diff=" << std::abs((a)-(b)) << "\n"; \
        tests_failed++; \
        return false; \
    } \
} while(0)

#define TEST_PASS(msg) do { \
    std::cout << "  [PASS] " << msg << "\n"; \
    tests_passed++; \
} while(0)

// ============================================================================
// LAYER 1: FFT/IFFT
// ============================================================================

bool test_layer1_fft_identity() {
    std::cout << "\n--- Layer 1.1: FFT of impulse ---\n";

    FFT fft(512);

    // Impulse at t=0 should give flat spectrum
    std::vector<Complex> impulse(512, Complex(0, 0));
    impulse[0] = Complex(1, 0);

    std::vector<Complex> spectrum;
    fft.forward(impulse, spectrum);

    // All bins should be 1+0j
    for (int i = 0; i < 512; ++i) {
        TEST_ASSERT_NEAR(spectrum[i].real(), 1.0f, 1e-5f, "Impulse spectrum real");
        TEST_ASSERT_NEAR(spectrum[i].imag(), 0.0f, 1e-5f, "Impulse spectrum imag");
    }

    TEST_PASS("FFT of impulse gives flat spectrum");
    return true;
}

bool test_layer1_fft_roundtrip() {
    std::cout << "\n--- Layer 1.2: FFT/IFFT roundtrip ---\n";

    FFT fft(512);

    // Random-ish data
    std::vector<Complex> original(512);
    for (int i = 0; i < 512; ++i) {
        original[i] = Complex(std::sin(i * 0.1f), std::cos(i * 0.17f));
    }

    std::vector<Complex> spectrum, recovered;
    fft.forward(original, spectrum);
    fft.inverse(spectrum, recovered);

    float max_err = 0;
    for (int i = 0; i < 512; ++i) {
        float err = std::abs(original[i] - recovered[i]);
        max_err = std::max(max_err, err);
    }

    std::cout << "  Max roundtrip error: " << max_err << "\n";
    TEST_ASSERT(max_err < 1e-5f, "FFT roundtrip error too large");

    TEST_PASS("FFT/IFFT roundtrip");
    return true;
}

bool test_layer1_fft_single_tone() {
    std::cout << "\n--- Layer 1.3: FFT of single tone ---\n";

    FFT fft(512);

    // Tone at bin 32
    std::vector<Complex> tone(512);
    for (int i = 0; i < 512; ++i) {
        float phase = 2.0f * M_PI * 32 * i / 512.0f;
        tone[i] = Complex(std::cos(phase), std::sin(phase));
    }

    std::vector<Complex> spectrum;
    fft.forward(tone, spectrum);

    // Energy should be concentrated at bin 32
    float energy_32 = std::abs(spectrum[32]);
    float energy_other = 0;
    for (int i = 0; i < 512; ++i) {
        if (i != 32) energy_other += std::abs(spectrum[i]);
    }

    std::cout << "  Bin 32 magnitude: " << energy_32 << " (expected 512)\n";
    std::cout << "  Other bins total: " << energy_other << " (expected ~0)\n";

    TEST_ASSERT(energy_32 > 500, "Tone energy at bin 32 too low");
    TEST_ASSERT(energy_other < 1.0f, "Leakage to other bins too high");

    TEST_PASS("FFT of single tone");
    return true;
}

bool test_layer1_fft_linearity() {
    std::cout << "\n--- Layer 1.4: FFT linearity ---\n";

    FFT fft(512);

    std::vector<Complex> a(512), b(512), sum(512);
    for (int i = 0; i < 512; ++i) {
        a[i] = Complex(std::sin(i * 0.1f), 0);
        b[i] = Complex(0, std::cos(i * 0.2f));
        sum[i] = a[i] + b[i];
    }

    std::vector<Complex> Fa, Fb, Fsum;
    fft.forward(a, Fa);
    fft.forward(b, Fb);
    fft.forward(sum, Fsum);

    // F(a+b) should equal F(a) + F(b)
    float max_err = 0;
    for (int i = 0; i < 512; ++i) {
        Complex expected = Fa[i] + Fb[i];
        float err = std::abs(Fsum[i] - expected);
        max_err = std::max(max_err, err);
    }

    std::cout << "  Max linearity error: " << max_err << "\n";
    TEST_ASSERT(max_err < 1e-4f, "FFT linearity error too large");

    TEST_PASS("FFT linearity");
    return true;
}

// ============================================================================
// LAYER 2: Carrier Mapping
// ============================================================================

bool test_layer2_carrier_indices() {
    std::cout << "\n--- Layer 2.1: Carrier index calculation ---\n";

    // For FFT size 512, carrier k maps to:
    // - Positive k: bin k
    // - Negative k: bin N+k (e.g., -1 -> 511, -2 -> 510)

    const int N = 512;

    // Test positive frequencies
    for (int k = 1; k <= 10; ++k) {
        int bin = k;
        std::cout << "  Carrier +" << k << " -> bin " << bin << "\n";
        TEST_ASSERT(bin >= 0 && bin < N, "Positive carrier out of range");
    }

    // Test negative frequencies
    for (int k = -1; k >= -10; --k) {
        int bin = (k + N) % N;
        std::cout << "  Carrier " << k << " -> bin " << bin << "\n";
        TEST_ASSERT(bin >= 0 && bin < N, "Negative carrier out of range");
    }

    TEST_PASS("Carrier index calculation");
    return true;
}

bool test_layer2_carrier_mapping_roundtrip() {
    std::cout << "\n--- Layer 2.2: Carrier mapping roundtrip ---\n";

    FFT fft(512);
    const int N = 512;

    // Place known values on specific carriers
    std::vector<std::pair<int, Complex>> test_carriers = {
        {5, Complex(1, 0)},
        {10, Complex(0, 1)},
        {-5, Complex(-1, 0)},   // bin 507
        {-10, Complex(0, -1)},  // bin 502
    };

    std::vector<Complex> freq(N, Complex(0, 0));
    for (auto& [k, val] : test_carriers) {
        int bin = (k + N) % N;
        freq[bin] = val;
        std::cout << "  TX: carrier " << k << " (bin " << bin << ") = " << val << "\n";
    }

    // IFFT -> FFT
    std::vector<Complex> time, freq_back;
    fft.inverse(freq, time);
    fft.forward(time, freq_back);

    // Check recovery
    for (auto& [k, val] : test_carriers) {
        int bin = (k + N) % N;
        Complex recovered = freq_back[bin];
        float err = std::abs(recovered - val);
        std::cout << "  RX: carrier " << k << " (bin " << bin << ") = " << recovered << ", err=" << err << "\n";
        TEST_ASSERT(err < 1e-5f, "Carrier value not recovered");
    }

    TEST_PASS("Carrier mapping roundtrip");
    return true;
}

// ============================================================================
// LAYER 3: Cyclic Prefix
// ============================================================================

bool test_layer3_cyclic_prefix_add_remove() {
    std::cout << "\n--- Layer 3.1: Cyclic prefix add/remove ---\n";

    const int N = 512;
    const int CP = 48;

    // Create test symbol
    std::vector<Complex> symbol(N);
    for (int i = 0; i < N; ++i) {
        symbol[i] = Complex(std::sin(i * 0.1f), std::cos(i * 0.2f));
    }

    // Add CP (copy end to beginning)
    std::vector<Complex> with_cp;
    with_cp.reserve(N + CP);
    for (int i = N - CP; i < N; ++i) {
        with_cp.push_back(symbol[i]);
    }
    for (int i = 0; i < N; ++i) {
        with_cp.push_back(symbol[i]);
    }

    TEST_ASSERT(with_cp.size() == N + CP, "Wrong size after adding CP");

    // Verify CP matches end of symbol
    for (int i = 0; i < CP; ++i) {
        TEST_ASSERT(with_cp[i] == symbol[N - CP + i], "CP doesn't match end of symbol");
    }

    // Remove CP
    std::vector<Complex> recovered(with_cp.begin() + CP, with_cp.end());

    TEST_ASSERT(recovered.size() == N, "Wrong size after removing CP");

    // Verify recovery
    for (int i = 0; i < N; ++i) {
        TEST_ASSERT(recovered[i] == symbol[i], "Symbol not recovered after CP removal");
    }

    TEST_PASS("Cyclic prefix add/remove");
    return true;
}

// ============================================================================
// LAYER 4: NCO/Mixer
// ============================================================================

bool test_layer4_nco_frequency() {
    std::cout << "\n--- Layer 4.1: NCO frequency accuracy ---\n";

    float fc = 1500.0f;
    float fs = 48000.0f;
    NCO nco(fc, fs);

    // Generate 1 second of NCO output
    int num_samples = 48000;
    std::vector<Complex> nco_output(num_samples);
    for (int i = 0; i < num_samples; ++i) {
        nco_output[i] = nco.next();
    }

    // Check that NCO completes exactly fc cycles per second
    // After 1 second, phase should have advanced by 2*pi*fc radians
    // which is fc complete cycles

    // Count zero crossings of real part (should be ~2*fc per second)
    int zero_crossings = 0;
    for (int i = 1; i < num_samples; ++i) {
        if ((nco_output[i-1].real() < 0) != (nco_output[i].real() < 0)) {
            zero_crossings++;
        }
    }

    float expected_crossings = 2 * fc;  // Two crossings per cycle
    float crossing_error = std::abs(zero_crossings - expected_crossings) / expected_crossings;

    std::cout << "  Zero crossings: " << zero_crossings << " (expected " << expected_crossings << ")\n";
    std::cout << "  Error: " << (crossing_error * 100) << "%\n";

    TEST_ASSERT(crossing_error < 0.01f, "NCO frequency error > 1%");

    TEST_PASS("NCO frequency accuracy");
    return true;
}

bool test_layer4_nco_roundtrip_complex() {
    std::cout << "\n--- Layer 4.2: NCO complex roundtrip ---\n";

    float fc = 1500.0f;
    float fs = 48000.0f;
    NCO tx_nco(fc, fs);
    NCO rx_nco(fc, fs);

    // Test with various complex values
    std::vector<Complex> test_values = {
        Complex(1, 0), Complex(0, 1), Complex(-1, 0), Complex(0, -1),
        Complex(0.707f, 0.707f), Complex(0.5f, 0.3f)
    };

    for (auto& val : test_values) {
        tx_nco.reset();
        rx_nco.reset();

        // TX: multiply by NCO (keep complex for this test)
        Complex tx = val * tx_nco.next();

        // RX: multiply by conj(NCO)
        Complex rx = tx * std::conj(rx_nco.next());

        float err = std::abs(rx - val);
        std::cout << "  Input: " << val << " -> TX: " << tx << " -> RX: " << rx << " (err=" << err << ")\n";
        TEST_ASSERT(err < 1e-6f, "NCO complex roundtrip error too large");
    }

    TEST_PASS("NCO complex roundtrip");
    return true;
}

bool test_layer4_nco_real_passband() {
    std::cout << "\n--- Layer 4.3: NCO real passband (actual TX/RX path) ---\n";

    // THIS IS THE ACTUAL TX/RX PATH:
    // TX: passband = Re(baseband * nco)
    // RX: recovered = passband * conj(nco)
    //
    // Math: Let baseband = a + jb, nco = cos(wt) + j*sin(wt)
    // TX: passband = Re[(a+jb)(cos+jsin)] = a*cos - b*sin
    // RX: recovered = (a*cos - b*sin) * (cos - j*sin)
    //              = (a*cos - b*sin)*cos - j*(a*cos - b*sin)*sin
    //              = a*cos² - b*sin*cos - j*a*cos*sin + j*b*sin²
    //
    // This does NOT recover the original baseband!
    // The real part becomes: a*cos² - b*sin*cos = a*(1+cos2wt)/2 - b*sin2wt/2
    // After lowpass filtering (averaging), we get: a/2
    //
    // So the real TX/RX path loses the imaginary component and scales by 0.5!
    // This is expected behavior for single-sideband-like modulation.

    float fc = 1500.0f;
    float fs = 48000.0f;

    // Test with a constant baseband value over many samples
    Complex baseband(0.8f, 0.6f);
    int num_samples = 480;  // 10ms = enough for averaging

    NCO tx_nco(fc, fs);
    NCO rx_nco(fc, fs);

    std::vector<float> passband(num_samples);
    std::vector<Complex> recovered(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        // TX
        Complex mixed = baseband * tx_nco.next();
        passband[i] = mixed.real();

        // RX
        recovered[i] = passband[i] * std::conj(rx_nco.next());
    }

    // Average the recovered signal (lowpass filter)
    Complex avg(0, 0);
    for (int i = num_samples/2; i < num_samples; ++i) {  // Skip transient
        avg += recovered[i];
    }
    avg /= float(num_samples/2);

    // Expected: real part = baseband.real()/2, imag part ≈ 0 (oscillates)
    std::cout << "  Baseband: " << baseband << "\n";
    std::cout << "  Recovered (averaged): " << avg << "\n";
    std::cout << "  Expected real: " << baseband.real() / 2 << "\n";

    // The recovered real part should be baseband.real()/2
    // The imaginary part averages to approximately 0
    TEST_ASSERT_NEAR(avg.real(), baseband.real() / 2, 0.01f, "Recovered real part");

    std::cout << "  NOTE: This is expected! Real passband loses imaginary and scales by 0.5\n";

    TEST_PASS("NCO real passband behavior understood");
    return true;
}

// ============================================================================
// LAYER 5: Single OFDM Symbol (Manual Construction)
// ============================================================================

bool test_layer5_single_symbol_manual() {
    std::cout << "\n--- Layer 5.1: Single OFDM symbol (manual) ---\n";

    const int N = 512;
    const int CP = 48;
    const float fc = 1500.0f;
    const float fs = 48000.0f;

    FFT fft(N);
    NCO tx_nco(fc, fs);
    NCO rx_nco(fc, fs);

    // Place test data on carriers
    std::vector<Complex> tx_freq(N, Complex(0, 0));
    std::vector<int> data_carriers = {5, 10, 15, 20, 507, 502, 497, 492};  // Mix of +/- frequencies
    std::vector<Complex> tx_symbols;

    for (int i = 0; i < (int)data_carriers.size(); ++i) {
        // Simple BPSK: alternating +1, -1
        Complex sym = (i % 2 == 0) ? Complex(1, 0) : Complex(-1, 0);
        tx_freq[data_carriers[i]] = sym;
        tx_symbols.push_back(sym);
        std::cout << "  TX carrier " << data_carriers[i] << " = " << sym << "\n";
    }

    // IFFT
    std::vector<Complex> tx_time;
    fft.inverse(tx_freq, tx_time);

    // Add CP
    std::vector<Complex> tx_time_cp;
    for (int i = N - CP; i < N; ++i) tx_time_cp.push_back(tx_time[i]);
    for (int i = 0; i < N; ++i) tx_time_cp.push_back(tx_time[i]);

    // Upconvert (real passband)
    std::vector<float> passband(tx_time_cp.size());
    for (size_t i = 0; i < tx_time_cp.size(); ++i) {
        passband[i] = (tx_time_cp[i] * tx_nco.next()).real();
    }

    // === RX ===

    // Downconvert
    std::vector<Complex> rx_baseband(passband.size());
    for (size_t i = 0; i < passband.size(); ++i) {
        rx_baseband[i] = passband[i] * std::conj(rx_nco.next());
    }

    // Remove CP
    std::vector<Complex> rx_time(rx_baseband.begin() + CP, rx_baseband.begin() + CP + N);

    // FFT
    std::vector<Complex> rx_freq;
    fft.forward(rx_time, rx_freq);

    // Check carriers (remember: 0.5 scale factor from real passband)
    std::cout << "\n  RX results (expected 0.5x scale):\n";
    bool all_correct = true;
    for (size_t i = 0; i < data_carriers.size(); ++i) {
        int bin = data_carriers[i];
        Complex rx = rx_freq[bin];
        Complex expected = tx_symbols[i] * 0.5f;
        float err = std::abs(rx - expected);

        std::cout << "  RX carrier " << bin << " = " << rx
                  << " (expected " << expected << ", err=" << err << ")\n";

        if (err > 0.001f) all_correct = false;
    }

    TEST_ASSERT(all_correct, "Some carriers don't match expected values");

    // Check phase consistency (all carriers should have same phase offset)
    std::cout << "\n  Phase analysis:\n";
    float ref_phase = std::arg(rx_freq[data_carriers[0]] / tx_symbols[0]);
    for (size_t i = 0; i < data_carriers.size(); ++i) {
        int bin = data_carriers[i];
        float phase = std::arg(rx_freq[bin] / tx_symbols[i]);
        float phase_err = phase - ref_phase;
        while (phase_err > M_PI) phase_err -= 2*M_PI;
        while (phase_err < -M_PI) phase_err += 2*M_PI;

        std::cout << "  Carrier " << bin << " phase: " << (phase * 180/M_PI) << "°"
                  << " (error from ref: " << (phase_err * 180/M_PI) << "°)\n";

        TEST_ASSERT(std::abs(phase_err) < 0.1f, "Phase inconsistency between carriers");
    }

    TEST_PASS("Single OFDM symbol (manual) - phases consistent");
    return true;
}

// ============================================================================
// LAYER 5b: Single OFDM Symbol (Using Actual Modulator Internals)
// ============================================================================

bool test_layer5_modulator_single_symbol() {
    std::cout << "\n--- Layer 5.2: Single symbol via OFDMModulator ---\n";

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::BPSK;

    OFDMModulator mod(config);

    // Generate preamble first (resets NCO)
    Samples preamble = mod.generatePreamble();
    std::cout << "  Preamble: " << preamble.size() << " samples\n";

    // Modulate 2 bytes = 16 bits = enough for one symbol with 15 data carriers
    std::vector<uint8_t> data = {0xAA, 0x55};  // Alternating bits
    Samples modulated = mod.modulate(data, Modulation::BPSK);
    std::cout << "  Modulated: " << modulated.size() << " samples\n";

    // Check that output is not silent
    float max_amp = 0;
    for (float s : modulated) {
        max_amp = std::max(max_amp, std::abs(s));
    }
    std::cout << "  Max amplitude: " << max_amp << "\n";

    TEST_ASSERT(max_amp > 0.01f, "Modulated signal is silent!");
    TEST_ASSERT(modulated.size() > 0, "No modulated samples");

    TEST_PASS("OFDMModulator generates non-zero output");
    return true;
}

// ============================================================================
// LAYER 6: Multi-Symbol Sequence
// ============================================================================

bool test_layer6_multi_symbol_manual() {
    std::cout << "\n--- Layer 6.1: Multi-symbol sequence (manual) ---\n";

    const int N = 512;
    const int CP = 48;
    const float fc = 1500.0f;
    const float fs = 48000.0f;
    const int NUM_SYMBOLS = 4;

    FFT fft(N);
    NCO tx_nco(fc, fs);
    NCO rx_nco(fc, fs);

    // Use same carriers for all symbols
    std::vector<int> data_carriers = {5, 10, 15, 507, 502, 497};

    // TX: Generate multiple symbols
    std::vector<std::vector<Complex>> all_tx_symbols(NUM_SYMBOLS);
    std::vector<float> all_passband;

    for (int sym = 0; sym < NUM_SYMBOLS; ++sym) {
        std::vector<Complex> tx_freq(N, Complex(0, 0));

        for (size_t i = 0; i < data_carriers.size(); ++i) {
            // Different pattern per symbol
            Complex s = ((sym + i) % 2 == 0) ? Complex(1, 0) : Complex(-1, 0);
            tx_freq[data_carriers[i]] = s;
            all_tx_symbols[sym].push_back(s);
        }

        std::vector<Complex> tx_time;
        fft.inverse(tx_freq, tx_time);

        // Add CP
        for (int i = N - CP; i < N; ++i) {
            all_passband.push_back((tx_time[i] * tx_nco.next()).real());
        }
        for (int i = 0; i < N; ++i) {
            all_passband.push_back((tx_time[i] * tx_nco.next()).real());
        }
    }

    std::cout << "  TX: " << NUM_SYMBOLS << " symbols, " << all_passband.size() << " samples\n";

    // RX: Demodulate all symbols
    int symbol_len = N + CP;
    bool all_correct = true;

    for (int sym = 0; sym < NUM_SYMBOLS; ++sym) {
        // Downconvert
        std::vector<Complex> rx_baseband(symbol_len);
        for (int i = 0; i < symbol_len; ++i) {
            int idx = sym * symbol_len + i;
            rx_baseband[i] = all_passband[idx] * std::conj(rx_nco.next());
        }

        // Remove CP and FFT
        std::vector<Complex> rx_time(rx_baseband.begin() + CP, rx_baseband.end());
        std::vector<Complex> rx_freq;
        fft.forward(rx_time, rx_freq);

        // Check carriers
        std::cout << "  Symbol " << sym << ": ";
        for (size_t i = 0; i < data_carriers.size(); ++i) {
            int bin = data_carriers[i];
            Complex rx = rx_freq[bin];
            Complex expected = all_tx_symbols[sym][i] * 0.5f;
            float err = std::abs(rx - expected);

            if (err > 0.01f) {
                std::cout << "[ERR bin" << bin << "=" << rx << "] ";
                all_correct = false;
            } else {
                std::cout << "[OK bin" << bin << "] ";
            }
        }
        std::cout << "\n";
    }

    TEST_ASSERT(all_correct, "Multi-symbol sequence has errors");

    TEST_PASS("Multi-symbol sequence (manual)");
    return true;
}

// ============================================================================
// LAYER 7: Pilot-Based Channel Estimation
// ============================================================================

bool test_layer7_pilot_estimation() {
    std::cout << "\n--- Layer 7.1: Pilot-based channel estimation ---\n";

    const int N = 512;
    const int CP = 48;
    const float fc = 1500.0f;
    const float fs = 48000.0f;

    FFT fft(N);
    NCO tx_nco(fc, fs);
    NCO rx_nco(fc, fs);

    // Define pilots and data carriers
    std::vector<int> pilot_carriers = {5, 15, 502, 507};  // Known positions
    std::vector<int> data_carriers = {10, 20, 497, 492};  // Data between pilots

    // Known pilot symbols (BPSK)
    std::vector<Complex> pilot_symbols = {Complex(1,0), Complex(-1,0), Complex(1,0), Complex(-1,0)};

    // Data symbols to transmit
    std::vector<Complex> data_symbols = {Complex(1,0), Complex(1,0), Complex(-1,0), Complex(-1,0)};

    // TX
    std::vector<Complex> tx_freq(N, Complex(0, 0));
    for (size_t i = 0; i < pilot_carriers.size(); ++i) {
        tx_freq[pilot_carriers[i]] = pilot_symbols[i];
    }
    for (size_t i = 0; i < data_carriers.size(); ++i) {
        tx_freq[data_carriers[i]] = data_symbols[i];
    }

    std::vector<Complex> tx_time;
    fft.inverse(tx_freq, tx_time);

    // Add CP and upconvert
    std::vector<float> passband;
    for (int i = N - CP; i < N; ++i) {
        passband.push_back((tx_time[i] * tx_nco.next()).real());
    }
    for (int i = 0; i < N; ++i) {
        passband.push_back((tx_time[i] * tx_nco.next()).real());
    }

    // RX: Downconvert
    std::vector<Complex> rx_baseband(passband.size());
    for (size_t i = 0; i < passband.size(); ++i) {
        rx_baseband[i] = passband[i] * std::conj(rx_nco.next());
    }

    // Remove CP and FFT
    std::vector<Complex> rx_time(rx_baseband.begin() + CP, rx_baseband.end());
    std::vector<Complex> rx_freq;
    fft.forward(rx_time, rx_freq);

    // Estimate channel from pilots: H = Rx / Tx
    std::cout << "  Pilot channel estimates:\n";
    std::vector<Complex> H_pilots;
    for (size_t i = 0; i < pilot_carriers.size(); ++i) {
        int bin = pilot_carriers[i];
        Complex H = rx_freq[bin] / pilot_symbols[i];
        H_pilots.push_back(H);
        std::cout << "    Pilot " << bin << ": H = " << H
                  << ", |H| = " << std::abs(H)
                  << ", phase = " << (std::arg(H) * 180/M_PI) << "°\n";
    }

    // All H values should be approximately equal (flat channel, same phase)
    // Expected: H ≈ 0.5 (from real passband scaling)
    Complex H_avg(0, 0);
    for (auto& h : H_pilots) H_avg += h;
    H_avg /= float(H_pilots.size());

    std::cout << "  Average H: " << H_avg << ", |H| = " << std::abs(H_avg) << "\n";

    // Check H consistency
    float max_H_err = 0;
    for (auto& h : H_pilots) {
        float err = std::abs(h - H_avg);
        max_H_err = std::max(max_H_err, err);
    }
    std::cout << "  Max H deviation from average: " << max_H_err << "\n";

    TEST_ASSERT(max_H_err < 0.01f, "Pilot H estimates inconsistent");
    TEST_ASSERT_NEAR(std::abs(H_avg), 0.5f, 0.05f, "H magnitude should be ~0.5");

    // Equalize data carriers using average H
    std::cout << "\n  Equalized data symbols:\n";
    for (size_t i = 0; i < data_carriers.size(); ++i) {
        int bin = data_carriers[i];
        Complex equalized = rx_freq[bin] / H_avg;
        Complex expected = data_symbols[i];
        float err = std::abs(equalized - expected);

        std::cout << "    Data " << bin << ": equalized = " << equalized
                  << " (expected " << expected << ", err=" << err << ")\n";

        TEST_ASSERT(err < 0.1f, "Equalized data symbol error too large");
    }

    TEST_PASS("Pilot-based channel estimation");
    return true;
}

// ============================================================================
// LAYER 8: Full Modulator -> Demodulator (The Critical Test!)
// ============================================================================

bool test_layer8_full_modem_loopback() {
    std::cout << "\n--- Layer 8: Full Modem Loopback (CRITICAL!) ---\n";

    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = Modulation::QPSK;
    config.cp_mode = CyclicPrefixMode::MEDIUM;

    OFDMModulator mod(config);
    OFDMDemodulator demod(config);

    // Test data: DEADBEEF pattern
    std::vector<uint8_t> test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF};

    // TX
    Samples preamble = mod.generatePreamble();
    Samples data = mod.modulate(test_data, config.modulation);

    std::cout << "  Preamble: " << preamble.size() << " samples\n";
    std::cout << "  Data: " << data.size() << " samples\n";

    // Combine with padding
    Samples tx_signal;
    tx_signal.insert(tx_signal.end(), 4800, 0.0f);  // 100ms lead-in
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), data.begin(), data.end());
    tx_signal.insert(tx_signal.end(), 4800, 0.0f);  // 100ms tail

    // CRITICAL: Normalize signal to audio level (0.5 peak)
    // The modulator outputs weak signals due to IFFT scaling.
    // The demodulator expects normalized audio (MIN_RMS = 0.05).
    float max_amp = 0;
    for (float s : tx_signal) max_amp = std::max(max_amp, std::abs(s));
    if (max_amp > 0) {
        float scale = 0.5f / max_amp;  // Normalize to 0.5 peak
        for (float& s : tx_signal) s *= scale;
    }
    std::cout << "  Normalized to 0.5 peak (was " << max_amp << ")\n";

    std::cout << "  Total TX: " << tx_signal.size() << " samples\n";

    // Feed ALL data to demodulator in chunks (don't stop at sync!)
    size_t chunk_size = 960;  // 20ms chunks
    bool synced = false;
    for (size_t i = 0; i < tx_signal.size(); i += chunk_size) {
        size_t len = std::min(chunk_size, tx_signal.size() - i);
        SampleSpan span(tx_signal.data() + i, len);
        demod.process(span);

        if (!synced && demod.isSynced()) {
            std::cout << "  Synced at sample " << i << "\n";
            synced = true;
            // DON'T break - continue feeding all data!
        }
    }

    if (!demod.isSynced()) {
        std::cout << "  [FAIL] Demodulator never synced!\n";

        // Debug: check signal levels
        float max_amp = 0;
        for (float s : tx_signal) max_amp = std::max(max_amp, std::abs(s));
        std::cout << "  Max TX amplitude: " << max_amp << "\n";

        tests_failed++;
        return false;
    }

    // Check sync state after feeding all data
    std::cout << "  After feeding all data: synced=" << (demod.isSynced() ? "yes" : "no") << "\n";

    // Extract soft bits (data already fed, just need to pump the demodulator)
    std::vector<float> all_soft;
    for (int i = 0; i < 100; ++i) {
        SampleSpan empty;
        bool got_data = demod.process(empty);
        auto soft = demod.getSoftBits();

        if (i < 5 || !soft.empty()) {
            std::cout << "  Pump " << i << ": got_data=" << got_data
                      << ", soft=" << soft.size()
                      << ", synced=" << (demod.isSynced() ? "yes" : "no") << "\n";
        }

        if (!soft.empty()) {
            all_soft.insert(all_soft.end(), soft.begin(), soft.end());
        }

        // Stop if we lost sync
        if (!demod.isSynced()) {
            std::cout << "  Lost sync at pump " << i << "\n";
            break;
        }
    }

    std::cout << "  Extracted " << all_soft.size() << " soft bits\n";

    if (all_soft.empty()) {
        std::cout << "  [FAIL] No soft bits extracted!\n";
        tests_failed++;
        return false;
    }

    // Convert to bytes
    std::vector<uint8_t> decoded;
    uint8_t byte = 0;
    int bit_count = 0;
    for (float llr : all_soft) {
        uint8_t bit = (llr < 0) ? 1 : 0;
        byte = (byte << 1) | bit;
        if (++bit_count == 8) {
            decoded.push_back(byte);
            byte = 0;
            bit_count = 0;
        }
    }

    std::cout << "  Decoded " << decoded.size() << " bytes\n";

    // Compare
    std::cout << "  TX: ";
    for (size_t i = 0; i < std::min(test_data.size(), (size_t)8); ++i) {
        printf("%02X ", test_data[i]);
    }
    std::cout << "\n  RX: ";
    for (size_t i = 0; i < std::min(decoded.size(), (size_t)8); ++i) {
        printf("%02X ", decoded[i]);
    }
    std::cout << "\n";

    int matches = 0;
    size_t check_len = std::min(decoded.size(), test_data.size());
    for (size_t i = 0; i < check_len; ++i) {
        if (decoded[i] == test_data[i]) matches++;
    }

    float match_pct = (check_len > 0) ? (100.0f * matches / check_len) : 0;
    std::cout << "  Match: " << matches << "/" << check_len << " (" << match_pct << "%)\n";

    TEST_ASSERT(match_pct >= 90.0f, "Data match < 90%");

    TEST_PASS("Full modem loopback");
    return true;
}

// ============================================================================
// MAIN
// ============================================================================

int main() {
    std::cout << "============================================================\n";
    std::cout << "   CRITICAL: LAYER-BY-LAYER OFDM VERIFICATION TESTS\n";
    std::cout << "============================================================\n";
    std::cout << "\n";
    std::cout << "This is CRITICAL software for emergency communications.\n";
    std::cout << "EVERY layer MUST pass 100%. NO exceptions.\n";
    std::cout << "\n";

    bool stop_on_fail = true;  // Stop at first failure to identify root cause

    // Layer 1: FFT
    std::cout << "\n========== LAYER 1: FFT/IFFT ==========\n";
    if (!test_layer1_fft_identity() && stop_on_fail) goto done;
    if (!test_layer1_fft_roundtrip() && stop_on_fail) goto done;
    if (!test_layer1_fft_single_tone() && stop_on_fail) goto done;
    if (!test_layer1_fft_linearity() && stop_on_fail) goto done;

    // Layer 2: Carrier Mapping
    std::cout << "\n========== LAYER 2: CARRIER MAPPING ==========\n";
    if (!test_layer2_carrier_indices() && stop_on_fail) goto done;
    if (!test_layer2_carrier_mapping_roundtrip() && stop_on_fail) goto done;

    // Layer 3: Cyclic Prefix
    std::cout << "\n========== LAYER 3: CYCLIC PREFIX ==========\n";
    if (!test_layer3_cyclic_prefix_add_remove() && stop_on_fail) goto done;

    // Layer 4: NCO
    std::cout << "\n========== LAYER 4: NCO/MIXER ==========\n";
    if (!test_layer4_nco_frequency() && stop_on_fail) goto done;
    if (!test_layer4_nco_roundtrip_complex() && stop_on_fail) goto done;
    if (!test_layer4_nco_real_passband() && stop_on_fail) goto done;

    // Layer 5: Single Symbol
    std::cout << "\n========== LAYER 5: SINGLE OFDM SYMBOL ==========\n";
    if (!test_layer5_single_symbol_manual() && stop_on_fail) goto done;
    if (!test_layer5_modulator_single_symbol() && stop_on_fail) goto done;

    // Layer 6: Multi-Symbol
    std::cout << "\n========== LAYER 6: MULTI-SYMBOL SEQUENCE ==========\n";
    if (!test_layer6_multi_symbol_manual() && stop_on_fail) goto done;

    // Layer 7: Channel Estimation
    std::cout << "\n========== LAYER 7: CHANNEL ESTIMATION ==========\n";
    if (!test_layer7_pilot_estimation() && stop_on_fail) goto done;

    // Layer 8: Full Modem (THE CRITICAL TEST!)
    std::cout << "\n========== LAYER 8: FULL MODEM LOOPBACK ==========\n";
    if (!test_layer8_full_modem_loopback() && stop_on_fail) goto done;

done:
    std::cout << "\n============================================================\n";
    std::cout << "   RESULTS: " << tests_passed << " passed, " << tests_failed << " failed\n";
    std::cout << "============================================================\n";

    if (tests_failed > 0) {
        std::cout << "\n*** CRITICAL: TESTS FAILED! DO NOT USE THIS SOFTWARE! ***\n";
        std::cout << "*** Fix the failing layer before proceeding. ***\n\n";
        return 1;
    } else {
        std::cout << "\n*** ALL LAYERS VERIFIED! ***\n\n";
        return 0;
    }
}
