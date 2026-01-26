// test_ofdm_chirp_cfo.cpp - Test OFDMChirpWaveform with CFO
//
// Tests the full OFDM_CHIRP pipeline with carrier frequency offset

#include "waveform/ofdm_chirp_waveform.hpp"
#include "ultra/fec.hpp"
#include "ultra/dsp.hpp"
#include <cstdio>
#include <cstring>
#include <random>
#include <cmath>

using namespace ultra;

// Apply CFO to a real passband signal using FFT-based frequency shift
// This properly shifts all frequency components by cfo_hz
void applyCFO(std::vector<float>& signal, float cfo_hz, float sample_rate) {
    if (std::abs(cfo_hz) < 0.001f) return;

    size_t N = signal.size();

    // Pad to power of 2 for FFT
    size_t fft_size = 1;
    while (fft_size < N) fft_size *= 2;

    FFT fft(fft_size);

    // Create complex input (real signal)
    std::vector<Complex> time_in(fft_size, Complex(0, 0));
    for (size_t i = 0; i < N; ++i) {
        time_in[i] = Complex(signal[i], 0);
    }

    // Forward FFT
    std::vector<Complex> freq(fft_size);
    fft.forward(time_in, freq);

    // Shift spectrum: multiply each bin by exp(j*2π*cfo*k/fft_size) where k is sample index
    // Actually, for proper frequency shift, we need to shift in time domain
    // Use the property: x(t)*exp(j*2π*f0*t) <-> X(f - f0)
    // For discrete: shift by k bins where k = cfo * fft_size / sample_rate

    // Actually, the simplest approach is time-domain multiplication with complex exponential
    // then take real part. But we need analytic signal first.

    // Create analytic signal by zeroing negative frequencies
    freq[0] *= 0.5f;  // DC
    if (fft_size > 1) {
        freq[fft_size/2] *= 0.5f;  // Nyquist
    }
    for (size_t i = fft_size/2 + 1; i < fft_size; ++i) {
        freq[i] = Complex(0, 0);  // Zero negative frequencies
    }
    for (size_t i = 1; i < fft_size/2; ++i) {
        freq[i] *= 2.0f;  // Double positive frequencies
    }

    // Inverse FFT to get analytic signal
    std::vector<Complex> analytic(fft_size);
    fft.inverse(freq, analytic);

    // Apply frequency shift and take real part
    for (size_t i = 0; i < N; ++i) {
        float phase = 2.0f * M_PI * cfo_hz * i / sample_rate;
        Complex shift(std::cos(phase), std::sin(phase));
        Complex shifted = analytic[i] * shift;
        signal[i] = shifted.real();
    }
}

// Add AWGN noise
void addNoise(std::vector<float>& signal, float snr_db) {
    float signal_power = 0.0f;
    for (float s : signal) signal_power += s * s;
    signal_power /= signal.size();

    float snr_linear = std::pow(10.0f, snr_db / 10.0f);
    float noise_power = signal_power / snr_linear;
    float noise_std = std::sqrt(noise_power);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> dist(0.0f, noise_std);

    for (float& s : signal) {
        s += dist(gen);
    }
}

int main(int argc, char* argv[]) {
    float snr_db = 15.0f;
    float cfo_hz = 0.0f;

    if (argc > 1) snr_db = std::atof(argv[1]);
    if (argc > 2) cfo_hz = std::atof(argv[2]);

    printf("=== OFDMChirpWaveform CFO Test ===\n");
    printf("SNR: %.1f dB, CFO: %.1f Hz\n\n", snr_db, cfo_hz);

    // Create waveform
    OFDMChirpWaveform waveform;
    waveform.configure(Modulation::DQPSK, CodeRate::R1_2);

    printf("Waveform: %s\n", waveform.getStatusString().c_str());
    printf("Samples per symbol: %d\n\n", waveform.getSamplesPerSymbol());

    // Create test data (one LDPC codeword worth)
    constexpr size_t DATA_BITS = 324;  // R1/2 data bits
    constexpr size_t DATA_BYTES = (DATA_BITS + 7) / 8;

    Bytes test_data(DATA_BYTES);
    std::mt19937 gen(42);  // Fixed seed for reproducibility
    for (auto& b : test_data) {
        b = gen() & 0xFF;
    }

    printf("Test data: %zu bytes\n", test_data.size());

    // Encode with LDPC
    LDPCEncoder encoder(CodeRate::R1_2);
    Bytes encoded = encoder.encode(test_data);
    printf("Encoded: %zu bytes (%zu bits)\n", encoded.size(), encoded.size() * 8);

    // === TX ===
    printf("\n--- TX ---\n");

    Samples preamble = waveform.generatePreamble();
    printf("Preamble: %zu samples (%.1f ms)\n",
           preamble.size(), preamble.size() * 1000.0f / 48000.0f);

    Samples modulated = waveform.modulate(encoded);
    printf("Modulated data: %zu samples (%.1f ms)\n",
           modulated.size(), modulated.size() * 1000.0f / 48000.0f);

    // Combine into TX signal
    Samples tx_signal;
    tx_signal.reserve(preamble.size() + modulated.size());
    tx_signal.insert(tx_signal.end(), preamble.begin(), preamble.end());
    tx_signal.insert(tx_signal.end(), modulated.begin(), modulated.end());

    printf("Total TX: %zu samples (%.1f ms)\n",
           tx_signal.size(), tx_signal.size() * 1000.0f / 48000.0f);

    // === Channel ===
    printf("\n--- Channel ---\n");

    // Apply CFO FIRST (before noise)
    if (std::abs(cfo_hz) > 0.001f) {
        applyCFO(tx_signal, cfo_hz, 48000.0f);
        printf("Applied CFO: %.1f Hz\n", cfo_hz);
    }

    // Add noise
    addNoise(tx_signal, snr_db);
    printf("Added AWGN noise at %.1f dB SNR\n", snr_db);

    // === RX ===
    printf("\n--- RX ---\n");

    // Reset waveform for RX
    waveform.reset();

    // Detect sync
    SyncResult sync_result;
    SampleSpan rx_span(tx_signal.data(), tx_signal.size());
    bool sync_found = waveform.detectSync(rx_span, sync_result, 0.15f);

    printf("Sync detected: %s\n", sync_found ? "YES" : "NO");
    if (sync_found) {
        printf("  Correlation: %.3f\n", sync_result.correlation);
        printf("  CFO estimate: %.2f Hz (actual: %.2f Hz, error: %.2f Hz)\n",
               sync_result.cfo_hz, cfo_hz, std::abs(sync_result.cfo_hz - cfo_hz));
        printf("  Start sample (DATA): %d\n", sync_result.start_sample);
    }

    if (!sync_found) {
        printf("FAILED: Sync not detected\n");
        return 1;
    }

    // Apply CFO correction
    waveform.setFrequencyOffset(sync_result.cfo_hz);

    // Calculate training start
    int training_sym_samples = 2 * waveform.getSamplesPerSymbol();
    size_t expected_training_start = preamble.size() - training_sym_samples;

    printf("\nTraining start: %zu\n", expected_training_start);

    // Process
    size_t process_len = tx_signal.size() - expected_training_start;
    SampleSpan process_span(tx_signal.data() + expected_training_start, process_len);

    bool frame_ready = waveform.process(process_span);
    printf("Frame ready: %s\n", frame_ready ? "YES" : "NO");
    printf("Estimated SNR: %.1f dB\n", waveform.estimatedSNR());

    if (!frame_ready) {
        printf("FAILED: Frame not ready after processing\n");
        return 1;
    }

    // Get soft bits
    auto soft_bits = waveform.getSoftBits();
    printf("Got %zu soft bits (need %d for one codeword)\n", soft_bits.size(), 648);

    if (soft_bits.size() < 648) {
        printf("FAILED: Not enough soft bits\n");
        return 1;
    }

    // Decode with LDPC
    LDPCDecoder decoder(CodeRate::R1_2);

    std::span<const float> codeword_span(soft_bits.data(), 648);
    Bytes decoded = decoder.decodeSoft(codeword_span);
    bool success = decoder.lastDecodeSuccess();

    printf("\nLDPC decode: %s\n", success ? "SUCCESS" : "FAILED");

    if (success) {
        // Compare decoded data (with MSB-first mask for last byte)
        bool data_match = true;
        size_t compare_bytes = std::min(decoded.size(), test_data.size());
        for (size_t i = 0; i < compare_bytes; i++) {
            uint8_t expected = test_data[i];
            uint8_t got = decoded[i];
            if (i == 40) {
                expected &= 0xF0;
                got &= 0xF0;
            }
            if (got != expected) {
                data_match = false;
                printf("Mismatch at byte %zu: got %02x, expected %02x\n",
                       i, decoded[i], test_data[i]);
            }
        }

        if (data_match) {
            printf("Data verified OK!\n");
            printf("\n=== TEST PASSED ===\n");
            return 0;
        } else {
            printf("Data mismatch!\n");
            printf("\n=== TEST FAILED ===\n");
            return 1;
        }
    }

    printf("\n=== TEST FAILED ===\n");
    return 1;
}
