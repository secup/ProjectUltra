/**
 * Direct carrier test - bypass modulator/demodulator to isolate the issue
 * Tests: freq_domain -> IFFT -> NCO up -> real part -> NCO down -> FFT -> freq_domain
 */
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "ultra/dsp.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// Test with full OFDM symbol including pilots
bool test_with_pilots() {
    std::cout << "\n=== Test with Pilots ===\n";

    const size_t fft_size = 512;
    const size_t cp_len = 48;
    const float center_freq = 1500.0f;
    const float sample_rate = 48000.0f;

    FFT fft(fft_size);
    NCO tx_mixer(center_freq, sample_rate);
    NCO rx_mixer(center_freq, sample_rate);

    // Carrier setup matching ModemConfig defaults (num_carriers=30, pilot_spacing=2)
    // Pilots: 497, 499, 501, ..., 511, 2, 4, 6, ..., 14 (15 pilots)
    // Data:   498, 500, 502, ..., 510, 1, 3, 5, ..., 15 (15 data)
    std::vector<int> pilot_indices = {497, 499, 501, 503, 505, 507, 509, 511, 2, 4, 6, 8, 10, 12, 14};
    std::vector<int> data_indices = {498, 500, 502, 504, 506, 508, 510, 1, 3, 5, 7, 9, 11, 13, 15};

    // TX: Create freq domain
    std::vector<Complex> freq_tx(fft_size, Complex(0, 0));

    // Add pilots (all +1 for simplicity)
    for (int idx : pilot_indices) {
        freq_tx[idx] = Complex(1.0f, 0.0f);
    }

    // Add data (all same QPSK symbol for testing)
    Complex test_symbol(0.7071f, -0.7071f);  // bits 10
    for (int idx : data_indices) {
        freq_tx[idx] = test_symbol;
    }

    std::cout << "TX freq_domain:\n";
    std::cout << "  Pilot[497]=" << freq_tx[497] << "\n";
    std::cout << "  Data[498]=" << freq_tx[498] << "\n";
    std::cout << "  Pilot[499]=" << freq_tx[499] << "\n";
    std::cout << "  Data[500]=" << freq_tx[500] << "\n";

    // IFFT to time domain
    std::vector<Complex> time_tx;
    fft.inverse(freq_tx, time_tx);

    // Add cyclic prefix
    std::vector<Complex> with_cp;
    for (size_t i = fft_size - cp_len; i < fft_size; i++) {
        with_cp.push_back(time_tx[i]);
    }
    for (const auto& s : time_tx) {
        with_cp.push_back(s);
    }

    // Upconvert and take real part (TX)
    tx_mixer.reset();
    std::vector<float> real_signal(with_cp.size());
    for (size_t i = 0; i < with_cp.size(); i++) {
        Complex mixed = with_cp[i] * tx_mixer.next();
        real_signal[i] = mixed.real();
    }

    // Downconvert (RX)
    rx_mixer.reset();
    std::vector<Complex> baseband(with_cp.size());
    for (size_t i = 0; i < with_cp.size(); i++) {
        Complex osc = rx_mixer.next();
        baseband[i] = real_signal[i] * std::conj(osc);
    }

    // Remove cyclic prefix
    std::vector<Complex> time_rx(fft_size);
    for (size_t i = 0; i < fft_size; i++) {
        time_rx[i] = baseband[cp_len + i];
    }

    // FFT back to frequency domain
    std::vector<Complex> freq_rx;
    fft.forward(time_rx, freq_rx);

    std::cout << "\nRX freq_domain (raw):\n";
    std::cout << "  Pilot[497]=" << freq_rx[497] << "\n";
    std::cout << "  Data[498]=" << freq_rx[498] << "\n";
    std::cout << "  Pilot[499]=" << freq_rx[499] << "\n";
    std::cout << "  Data[500]=" << freq_rx[500] << "\n";

    // Channel estimation from pilots
    std::vector<Complex> channel_est(fft_size, Complex(1, 0));
    for (int idx : pilot_indices) {
        // H = rx / tx = freq_rx[idx] / 1.0
        channel_est[idx] = freq_rx[idx];
    }

    // Simple interpolation for data carriers (average of adjacent pilots)
    // For this test, just use nearest pilot
    std::cout << "\nChannel estimates at pilots:\n";
    for (size_t i = 0; i < 4; i++) {
        std::cout << "  H[" << pilot_indices[i] << "]=" << channel_est[pilot_indices[i]] << "\n";
    }

    // Equalize data carriers using adjacent pilot average
    std::cout << "\nEqualized data carriers:\n";
    bool all_pass = true;
    for (size_t i = 0; i < data_indices.size(); i++) {
        int data_idx = data_indices[i];

        // Find surrounding pilots
        int lower_pilot = -1, upper_pilot = -1;
        for (int j = (int)i; j >= 0; j--) {
            if (j < (int)pilot_indices.size()) {
                // Pilots and data are interleaved
                lower_pilot = pilot_indices[std::min(i, pilot_indices.size()-1)];
                upper_pilot = pilot_indices[std::min(i+1, pilot_indices.size()-1)];
                break;
            }
        }

        // Use average of adjacent pilots
        Complex h_interp = (channel_est[pilot_indices[i]] + channel_est[pilot_indices[std::min(i+1, pilot_indices.size()-1)]]) / 2.0f;

        // MMSE equalization
        float h_power = std::norm(h_interp);
        Complex equalized = std::conj(h_interp) * freq_rx[data_idx] / (h_power + 0.01f);

        std::cout << "  Data[" << data_idx << "]: raw=" << freq_rx[data_idx]
                  << " H=" << h_interp << " eq=" << equalized;

        // Check result
        float error = std::abs(equalized - test_symbol);
        if (error < 0.1f) {
            std::cout << " [OK]\n";
        } else {
            std::cout << " [FAIL] error=" << error << "\n";
            all_pass = false;
        }
    }

    return all_pass;
}

int main() {
    std::cout << "=== Direct Carrier Test ===\n\n";

    const size_t fft_size = 512;
    const size_t cp_len = 48;
    const float center_freq = 1500.0f;
    const float sample_rate = 48000.0f;

    FFT fft(fft_size);
    NCO tx_mixer(center_freq, sample_rate);
    NCO rx_mixer(center_freq, sample_rate);

    // Test data: known QPSK symbol
    Complex test_symbol(0.7071f, -0.7071f);  // bits 10
    std::cout << "Test symbol: (" << test_symbol.real() << ", " << test_symbol.imag() << ")\n";

    // Carrier indices to test (like the modulator's data carriers)
    std::vector<int> test_carriers = {498, 500, 502, 1, 3, 5};

    for (int carrier_idx : test_carriers) {
        std::cout << "\n--- Testing carrier index " << carrier_idx << " ---\n";

        // Create freq domain with single carrier
        std::vector<Complex> freq_tx(fft_size, Complex(0, 0));
        freq_tx[carrier_idx] = test_symbol;

        // IFFT to time domain
        std::vector<Complex> time_tx;
        fft.inverse(freq_tx, time_tx);

        std::cout << "After IFFT: time[0]=" << time_tx[0] << "\n";

        // Add cyclic prefix
        std::vector<Complex> with_cp;
        for (size_t i = fft_size - cp_len; i < fft_size; i++) {
            with_cp.push_back(time_tx[i]);
        }
        for (const auto& s : time_tx) {
            with_cp.push_back(s);
        }

        // Upconvert and take real part (TX)
        tx_mixer.reset();
        std::vector<float> real_signal(with_cp.size());
        for (size_t i = 0; i < with_cp.size(); i++) {
            Complex mixed = with_cp[i] * tx_mixer.next();
            real_signal[i] = mixed.real();
        }

        // Downconvert (RX)
        rx_mixer.reset();
        std::vector<Complex> baseband(with_cp.size());
        for (size_t i = 0; i < with_cp.size(); i++) {
            Complex osc = rx_mixer.next();
            baseband[i] = real_signal[i] * std::conj(osc);
        }

        // Remove cyclic prefix
        std::vector<Complex> time_rx(fft_size);
        for (size_t i = 0; i < fft_size; i++) {
            time_rx[i] = baseband[cp_len + i];
        }

        // FFT back to frequency domain
        std::vector<Complex> freq_rx;
        fft.forward(time_rx, freq_rx);

        // Check the carrier
        Complex received = freq_rx[carrier_idx];

        // The FFT has a scaling factor; normalize by comparing to input magnitude
        float scale = std::abs(received) / std::abs(test_symbol);
        Complex normalized = received / scale;

        std::cout << "  TX: (" << test_symbol.real() << ", " << test_symbol.imag() << ")\n";
        std::cout << "  RX raw: (" << received.real() << ", " << received.imag() << ")\n";
        std::cout << "  Scale: " << scale << "\n";
        std::cout << "  RX normalized: (" << normalized.real() << ", " << normalized.imag() << ")\n";

        // Check if RX matches TX
        float error = std::abs(normalized - test_symbol);
        if (error < 0.01f) {
            std::cout << "  [PASS] Carrier " << carrier_idx << " matches!\n";
        } else {
            // Check other possible transforms
            Complex conj_sym = std::conj(test_symbol);
            Complex neg_sym = -test_symbol;
            Complex neg_conj_sym = -std::conj(test_symbol);

            if (std::abs(normalized - conj_sym) < 0.01f) {
                std::cout << "  [FAIL] Carrier " << carrier_idx << " is CONJUGATED!\n";
            } else if (std::abs(normalized - neg_sym) < 0.01f) {
                std::cout << "  [FAIL] Carrier " << carrier_idx << " is NEGATED!\n";
            } else if (std::abs(normalized - neg_conj_sym) < 0.01f) {
                std::cout << "  [FAIL] Carrier " << carrier_idx << " is NEGATED+CONJUGATED!\n";
            } else {
                std::cout << "  [FAIL] Carrier " << carrier_idx << " has unknown error: " << error << "\n";
            }
        }

        // Also check for image at 512-carrier_idx
        int image_idx = (fft_size - carrier_idx) % fft_size;
        if (image_idx != carrier_idx) {
            Complex image = freq_rx[image_idx];
            float image_mag = std::abs(image);
            std::cout << "  Image at bin " << image_idx << ": mag=" << image_mag
                      << " (" << image.real() << ", " << image.imag() << ")\n";
        }
    }

    // Test with pilots
    if (!test_with_pilots()) {
        return 1;
    }

    return 0;
}
