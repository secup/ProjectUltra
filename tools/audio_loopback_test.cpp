// Simple audio loopback test - sends known signals through USB audio and verifies
// Build: g++ -std=c++17 -o audio_loopback_test audio_loopback_test.cpp -lSDL2 -lm

#include <SDL.h>
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <complex>
#include <algorithm>

constexpr int SAMPLE_RATE = 48000;
constexpr int BUFFER_SIZE = 1024;
constexpr float PI = 3.14159265358979323846f;

// Test parameters
int g_tx_channel = 0;
int g_rx_channel = 0;
int g_num_channels = 2;

// Circular buffers for TX and RX
std::vector<float> g_tx_signal;
size_t g_tx_pos = 0;
std::vector<float> g_rx_buffer;
bool g_recording = false;

void audio_output_callback(void* userdata, Uint8* stream, int len) {
    float* output = reinterpret_cast<float*>(stream);
    int frames = len / sizeof(float) / g_num_channels;

    memset(stream, 0, len);

    for (int i = 0; i < frames && g_tx_pos < g_tx_signal.size(); i++) {
        output[i * g_num_channels + g_tx_channel] = g_tx_signal[g_tx_pos++];
    }
}

void audio_input_callback(void* userdata, Uint8* stream, int len) {
    const float* input = reinterpret_cast<const float*>(stream);
    int frames = len / sizeof(float) / g_num_channels;

    if (g_recording) {
        for (int i = 0; i < frames; i++) {
            g_rx_buffer.push_back(input[i * g_num_channels + g_rx_channel]);
        }
    }
}

// Generate test signals
std::vector<float> generate_sine(float freq, float duration, float amplitude = 0.5f) {
    int samples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<float> signal(samples);
    for (int i = 0; i < samples; i++) {
        signal[i] = amplitude * std::sin(2.0f * PI * freq * i / SAMPLE_RATE);
    }
    return signal;
}

std::vector<float> generate_chirp(float f0, float f1, float duration, float amplitude = 0.5f) {
    int samples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<float> signal(samples);
    float k = (f1 - f0) / duration;
    for (int i = 0; i < samples; i++) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        float freq = f0 + k * t;
        signal[i] = amplitude * std::sin(2.0f * PI * (f0 * t + 0.5f * k * t * t));
    }
    return signal;
}

std::vector<float> generate_multitone(std::vector<float> freqs, float duration, float amplitude = 0.3f) {
    int samples = static_cast<int>(duration * SAMPLE_RATE);
    std::vector<float> signal(samples, 0.0f);
    for (float freq : freqs) {
        for (int i = 0; i < samples; i++) {
            signal[i] += amplitude * std::sin(2.0f * PI * freq * i / SAMPLE_RATE);
        }
    }
    // Normalize
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        for (float& s : signal) s *= 0.8f / max_val;
    }
    return signal;
}

// Analysis functions
float compute_rms(const std::vector<float>& signal, size_t start = 0, size_t len = 0) {
    if (len == 0) len = signal.size() - start;
    float sum = 0;
    for (size_t i = start; i < start + len && i < signal.size(); i++) {
        sum += signal[i] * signal[i];
    }
    return std::sqrt(sum / len);
}

float compute_correlation(const std::vector<float>& a, const std::vector<float>& b, int lag = 0) {
    size_t len = std::min(a.size(), b.size());
    if (lag < 0) lag = 0;

    float sum_ab = 0, sum_aa = 0, sum_bb = 0;
    for (size_t i = 0; i + lag < len; i++) {
        sum_ab += a[i] * b[i + lag];
        sum_aa += a[i] * a[i];
        sum_bb += b[i + lag] * b[i + lag];
    }

    if (sum_aa < 1e-10f || sum_bb < 1e-10f) return 0;
    return sum_ab / std::sqrt(sum_aa * sum_bb);
}

int find_best_lag(const std::vector<float>& tx, const std::vector<float>& rx, int max_lag = 10000) {
    float best_corr = 0;
    int best_lag = 0;

    for (int lag = 0; lag < max_lag && lag < (int)rx.size(); lag += 10) {
        float corr = compute_correlation(tx, rx, lag);
        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    // Fine search around best
    for (int lag = std::max(0, best_lag - 20); lag < best_lag + 20 && lag < (int)rx.size(); lag++) {
        float corr = compute_correlation(tx, rx, lag);
        if (corr > best_corr) {
            best_corr = corr;
            best_lag = lag;
        }
    }

    return best_lag;
}

// Measure frequency response at specific frequency
std::pair<float, float> measure_freq_response(const std::vector<float>& rx, float freq, int start, int len) {
    // Compute magnitude and phase at frequency using DFT
    float sum_cos = 0, sum_sin = 0;
    for (int i = 0; i < len; i++) {
        float t = static_cast<float>(i) / SAMPLE_RATE;
        sum_cos += rx[start + i] * std::cos(2.0f * PI * freq * t);
        sum_sin += rx[start + i] * std::sin(2.0f * PI * freq * t);
    }
    sum_cos *= 2.0f / len;
    sum_sin *= 2.0f / len;

    float magnitude = std::sqrt(sum_cos * sum_cos + sum_sin * sum_sin);
    float phase = std::atan2(sum_sin, sum_cos) * 180.0f / PI;

    return {magnitude, phase};
}

void print_usage(const char* prog) {
    printf("Usage: %s <output_device> <input_device> <tx_channel> <rx_channel>\n", prog);
    printf("Example: %s \"USB Audio Device\" \"USB Audio Device\" 2 0\n", prog);
    printf("\nThis tool tests audio loopback by sending known signals and analyzing what comes back.\n");
}

int main(int argc, char* argv[]) {
    if (argc < 5) {
        print_usage(argv[0]);
        return 1;
    }

    const char* output_device = argv[1];
    const char* input_device = argv[2];
    g_tx_channel = std::atoi(argv[3]);
    g_rx_channel = std::atoi(argv[4]);

    printf("=== Audio Loopback Test ===\n");
    printf("Output device: %s (channel %d)\n", output_device, g_tx_channel);
    printf("Input device: %s (channel %d)\n", input_device, g_rx_channel);
    printf("\n");

    // Initialize SDL
    if (SDL_Init(SDL_INIT_AUDIO) < 0) {
        printf("SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    // Open output device
    SDL_AudioSpec want_out, have_out;
    SDL_zero(want_out);
    want_out.freq = SAMPLE_RATE;
    want_out.format = AUDIO_F32SYS;
    want_out.channels = 8;
    want_out.samples = BUFFER_SIZE;
    want_out.callback = audio_output_callback;

    SDL_AudioDeviceID out_dev = SDL_OpenAudioDevice(output_device, 0, &want_out, &have_out, SDL_AUDIO_ALLOW_CHANNELS_CHANGE);
    if (out_dev == 0) {
        printf("Failed to open output device: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    printf("Output: %d channels, %d Hz\n", have_out.channels, have_out.freq);
    g_num_channels = have_out.channels;

    if (g_tx_channel >= g_num_channels) {
        printf("ERROR: TX channel %d invalid (device has %d channels)\n", g_tx_channel, g_num_channels);
        SDL_CloseAudioDevice(out_dev);
        SDL_Quit();
        return 1;
    }

    // Open input device
    SDL_AudioSpec want_in, have_in;
    SDL_zero(want_in);
    want_in.freq = SAMPLE_RATE;
    want_in.format = AUDIO_F32SYS;
    want_in.channels = 8;
    want_in.samples = BUFFER_SIZE;
    want_in.callback = audio_input_callback;

    SDL_AudioDeviceID in_dev = SDL_OpenAudioDevice(input_device, 1, &want_in, &have_in, SDL_AUDIO_ALLOW_CHANNELS_CHANGE);
    if (in_dev == 0) {
        printf("Failed to open input device: %s\n", SDL_GetError());
        SDL_CloseAudioDevice(out_dev);
        SDL_Quit();
        return 1;
    }
    printf("Input: %d channels, %d Hz\n", have_in.channels, have_in.freq);

    if (g_rx_channel >= have_in.channels) {
        printf("ERROR: RX channel %d invalid (device has %d channels)\n", g_rx_channel, have_in.channels);
        SDL_CloseAudioDevice(out_dev);
        SDL_CloseAudioDevice(in_dev);
        SDL_Quit();
        return 1;
    }

    printf("\n");

    // ========== TEST 1: Single tone at center frequency ==========
    printf("=== Test 1: Single 2000 Hz tone ===\n");
    {
        g_tx_signal = generate_sine(2000, 0.5f, 0.5f);
        g_tx_pos = 0;
        g_rx_buffer.clear();
        g_recording = true;

        SDL_PauseAudioDevice(in_dev, 0);
        SDL_PauseAudioDevice(out_dev, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        SDL_PauseAudioDevice(out_dev, 1);
        SDL_PauseAudioDevice(in_dev, 1);
        g_recording = false;

        float tx_rms = compute_rms(g_tx_signal);
        float rx_rms = compute_rms(g_rx_buffer);
        int lag = find_best_lag(g_tx_signal, g_rx_buffer);
        float corr = compute_correlation(g_tx_signal, g_rx_buffer, lag);

        printf("  TX RMS: %.4f\n", tx_rms);
        printf("  RX RMS: %.4f (ratio: %.2f)\n", rx_rms, rx_rms / tx_rms);
        printf("  Delay: %d samples (%.2f ms)\n", lag, 1000.0f * lag / SAMPLE_RATE);
        printf("  Correlation: %.4f\n", corr);
        printf("  Result: %s\n\n", corr > 0.9f ? "PASS" : "FAIL - signal corrupted or not looping back");
    }

    // ========== TEST 2: Multi-frequency response ==========
    printf("=== Test 2: Frequency response (500-3500 Hz) ===\n");
    {
        std::vector<float> test_freqs = {500, 1000, 1500, 2000, 2500, 3000, 3500};

        for (float freq : test_freqs) {
            g_tx_signal = generate_sine(freq, 0.3f, 0.5f);
            g_tx_pos = 0;
            g_rx_buffer.clear();
            g_recording = true;

            SDL_PauseAudioDevice(in_dev, 0);
            SDL_PauseAudioDevice(out_dev, 0);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            SDL_PauseAudioDevice(out_dev, 1);
            SDL_PauseAudioDevice(in_dev, 1);
            g_recording = false;

            float tx_rms = compute_rms(g_tx_signal);
            int lag = find_best_lag(g_tx_signal, g_rx_buffer);

            // Measure magnitude and phase at the test frequency
            if (g_rx_buffer.size() > lag + 4800) {
                auto [mag, phase] = measure_freq_response(g_rx_buffer, freq, lag + 1000, 4800);
                float gain_db = 20.0f * std::log10(mag / tx_rms + 1e-10f);
                printf("  %4.0f Hz: gain=%+5.1f dB, phase=%+6.1f deg, delay=%d samples\n",
                       freq, gain_db, phase, lag);
            } else {
                printf("  %4.0f Hz: insufficient data\n", freq);
            }
        }
        printf("\n");
    }

    // ========== TEST 3: Phase coherence across time ==========
    printf("=== Test 3: Phase stability over time ===\n");
    {
        g_tx_signal = generate_sine(2000, 2.0f, 0.5f);  // 2 second tone
        g_tx_pos = 0;
        g_rx_buffer.clear();
        g_recording = true;

        SDL_PauseAudioDevice(in_dev, 0);
        SDL_PauseAudioDevice(out_dev, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(2500));

        SDL_PauseAudioDevice(out_dev, 1);
        SDL_PauseAudioDevice(in_dev, 1);
        g_recording = false;

        int lag = find_best_lag(g_tx_signal, g_rx_buffer);

        // Measure phase at different time offsets
        printf("  Measuring phase at 2000 Hz over time:\n");
        std::vector<float> phases;
        for (int t_ms = 200; t_ms <= 1800; t_ms += 200) {
            int start = lag + (t_ms * SAMPLE_RATE / 1000);
            if (start + 4800 < (int)g_rx_buffer.size()) {
                auto [mag, phase] = measure_freq_response(g_rx_buffer, 2000, start, 4800);
                phases.push_back(phase);
                printf("    t=%4d ms: phase=%+6.1f deg\n", t_ms, phase);
            }
        }

        if (phases.size() >= 2) {
            float phase_var = 0;
            float mean_phase = 0;
            for (float p : phases) mean_phase += p;
            mean_phase /= phases.size();
            for (float p : phases) phase_var += (p - mean_phase) * (p - mean_phase);
            phase_var /= phases.size();
            float phase_std = std::sqrt(phase_var);
            printf("  Phase std dev: %.1f deg\n", phase_std);
            printf("  Result: %s\n\n", phase_std < 10.0f ? "PASS - phase stable" : "FAIL - phase drifting");
        }
    }

    // ========== TEST 4: Chirp (broadband) ==========
    printf("=== Test 4: Broadband chirp (500-3500 Hz) ===\n");
    {
        g_tx_signal = generate_chirp(500, 3500, 0.5f, 0.5f);
        g_tx_pos = 0;
        g_rx_buffer.clear();
        g_recording = true;

        SDL_PauseAudioDevice(in_dev, 0);
        SDL_PauseAudioDevice(out_dev, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        SDL_PauseAudioDevice(out_dev, 1);
        SDL_PauseAudioDevice(in_dev, 1);
        g_recording = false;

        float tx_rms = compute_rms(g_tx_signal);
        float rx_rms = compute_rms(g_rx_buffer);
        int lag = find_best_lag(g_tx_signal, g_rx_buffer);
        float corr = compute_correlation(g_tx_signal, g_rx_buffer, lag);

        printf("  TX RMS: %.4f\n", tx_rms);
        printf("  RX RMS: %.4f (ratio: %.2f)\n", rx_rms, rx_rms / tx_rms);
        printf("  Delay: %d samples (%.2f ms)\n", lag, 1000.0f * lag / SAMPLE_RATE);
        printf("  Correlation: %.4f\n", corr);
        printf("  Result: %s\n\n", corr > 0.8f ? "PASS" : "FAIL - broadband signal corrupted");
    }

    // Cleanup
    SDL_CloseAudioDevice(out_dev);
    SDL_CloseAudioDevice(in_dev);
    SDL_Quit();

    printf("=== Test Complete ===\n");
    return 0;
}
