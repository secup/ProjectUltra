// Profile acquisition time for all waveforms using ModemEngine
// Measures: OFDM, DPSK, PING sync detection and decode time

#include "gui/modem/modem_engine.hpp"
#include "gui/adaptive_mode.hpp"
#include "protocol/frame_v2.hpp"
#include <chrono>
#include <random>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <thread>

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::protocol;
using Clock = std::chrono::high_resolution_clock;

// Global flag for real-time simulation
bool g_realtime = false;

struct ProfileResult {
    double feed_time_us;      // Time spent in feedAudio() calls
    double total_time_us;     // Wall clock from first sample to decode callback
    double decode_delay_ms;   // Time from signal END to decode callback (realistic latency)
    size_t signal_samples;    // TX signal samples (excluding lead silence)
    bool decoded;
};

// Synchronization helper for async callbacks
struct DecodeSync {
    std::mutex mutex;
    std::condition_variable cv;
    std::atomic<bool> decoded{false};
    Clock::time_point decode_time;

    void signalDecoded() {
        decode_time = Clock::now();
        decoded = true;
        cv.notify_one();
    }

    bool waitFor(int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex);
        return cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                          [this] { return decoded.load(); });
    }

    void reset() {
        decoded = false;
    }
};

// Add AWGN to signal
void addNoise(std::vector<float>& samples, float snr_db, std::mt19937& rng) {
    // Calculate signal power only from active (non-silent) portions
    // This matches test_hf_modem and gives correct SNR measurement
    float signal_energy = 0;
    size_t active_samples = 0;
    for (float s : samples) {
        if (std::abs(s) > 1e-6f) {
            signal_energy += s * s;
            active_samples++;
        }
    }
    if (active_samples == 0) return;

    float signal_power = signal_energy / active_samples;
    float noise_power = signal_power / std::pow(10.0f, snr_db / 10.0f);
    float noise_std = std::sqrt(noise_power);

    std::normal_distribution<float> noise(0, noise_std);
    for (float& s : samples) {
        s += noise(rng);
    }
}

// =============================================================================
// OFDM PROFILING
// =============================================================================

ProfileResult profileOFDM(float snr_db, size_t lead_silence, std::mt19937& rng) {
    ProfileResult result = {};

    // Create TX modem in connected mode to use OFDM waveform
    ModemEngine tx_modem;
    tx_modem.setConnected(true);
    tx_modem.setHandshakeComplete(true);
    tx_modem.setWaveformMode(WaveformMode::OFDM_COX);

    // Create TX frame
    auto frame = v2::ConnectFrame::makeConnect("TEST1", "TEST2", 0xFF, 0, 0xFF, 0xFF);
    Bytes frame_data = frame.serialize();

    // Generate TX signal (will use OFDM because connected=true)
    auto tx_samples = tx_modem.transmit(frame_data);

    // Build test signal: silence + signal + silence
    std::vector<float> test_signal(lead_silence, 0.0f);
    test_signal.insert(test_signal.end(), tx_samples.begin(), tx_samples.end());
    test_signal.resize(test_signal.size() + 4800, 0.0f);

    addNoise(test_signal, snr_db, rng);

    // Create fresh RX modem in connected mode to process OFDM
    ModemEngine rx_modem;
    rx_modem.setConnected(true);
    rx_modem.setWaveformMode(WaveformMode::OFDM_COX);

    DecodeSync sync;
    rx_modem.setRawDataCallback([&](const Bytes& data) {
        sync.signalDecoded();
    });

    // Feed in chunks (480 samples = 10ms at 48kHz)
    const size_t CHUNK_SIZE = 480;
    size_t offset = 0;
    double total_feed_us = 0;

    // In realtime mode, signal end time is deterministic
    double signal_end_ms = (lead_silence + tx_samples.size()) / 48.0;

    auto wall_start = Clock::now();

    while (offset < test_signal.size()) {
        size_t chunk_len = std::min(CHUNK_SIZE, test_signal.size() - offset);
        std::vector<float> chunk(test_signal.begin() + offset,
                                  test_signal.begin() + offset + chunk_len);

        auto t0 = Clock::now();
        rx_modem.feedAudio(chunk);
        auto t1 = Clock::now();

        total_feed_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        offset += chunk_len;

        if (sync.decoded.load()) break;

        // Real-time simulation: sleep to match audio rate
        if (g_realtime) {
            std::this_thread::sleep_for(std::chrono::microseconds(9500));  // ~10ms minus overhead
        }
    }

    // Wait for decode callback (up to 2 seconds)
    bool decoded = sync.waitFor(2000);

    auto wall_end = decoded ? sync.decode_time : Clock::now();
    double wall_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();

    result.feed_time_us = total_feed_us;
    result.total_time_us = wall_ms * 1000.0;  // Convert to us
    result.signal_samples = tx_samples.size();
    result.decoded = decoded;

    // Decode delay = wall time - signal end time (in realtime, signal ends at signal_end_ms)
    if (decoded && g_realtime) {
        result.decode_delay_ms = wall_ms - signal_end_ms;
    } else {
        result.decode_delay_ms = -1;
    }

    return result;
}

// =============================================================================
// DPSK PROFILING
// =============================================================================

ProfileResult profileDPSK(float snr_db, size_t lead_silence, std::mt19937& rng) {
    ProfileResult result = {};

    // Create modem engine in DPSK mode for TX
    ModemEngine tx_modem;
    tx_modem.setWaveformMode(WaveformMode::MC_DPSK);
    tx_modem.setConnectWaveform(WaveformMode::MC_DPSK);

    // Create TX frame
    auto frame = v2::ConnectFrame::makeConnect("TEST1", "TEST2", 0xFF, 0, 0xFF, 0xFF);
    Bytes frame_data = frame.serialize();

    // Generate TX signal
    auto tx_samples = tx_modem.transmit(frame_data);

    // Build test signal
    std::vector<float> test_signal(lead_silence, 0.0f);
    test_signal.insert(test_signal.end(), tx_samples.begin(), tx_samples.end());
    test_signal.resize(test_signal.size() + 4800, 0.0f);

    addNoise(test_signal, snr_db, rng);

    // Create fresh RX modem
    ModemEngine rx_modem;
    rx_modem.setWaveformMode(WaveformMode::MC_DPSK);
    rx_modem.setConnectWaveform(WaveformMode::MC_DPSK);

    DecodeSync sync;
    rx_modem.setRawDataCallback([&](const Bytes& data) {
        sync.signalDecoded();
    });

    // Feed in chunks (480 samples = 10ms at 48kHz)
    const size_t CHUNK_SIZE = 480;
    size_t offset = 0;
    double total_feed_us = 0;
    double signal_end_ms = (lead_silence + tx_samples.size()) / 48.0;

    auto wall_start = Clock::now();

    while (offset < test_signal.size()) {
        size_t chunk_len = std::min(CHUNK_SIZE, test_signal.size() - offset);
        std::vector<float> chunk(test_signal.begin() + offset,
                                  test_signal.begin() + offset + chunk_len);

        auto t0 = Clock::now();
        rx_modem.feedAudio(chunk);
        auto t1 = Clock::now();

        total_feed_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        offset += chunk_len;

        if (sync.decoded.load()) break;

        if (g_realtime) {
            std::this_thread::sleep_for(std::chrono::microseconds(9500));
        }
    }

    // Wait for decode callback (up to 2 seconds)
    bool decoded = sync.waitFor(2000);

    auto wall_end = decoded ? sync.decode_time : Clock::now();
    double wall_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();

    result.feed_time_us = total_feed_us;
    result.total_time_us = wall_ms * 1000.0;
    result.signal_samples = tx_samples.size();
    result.decoded = decoded;

    if (decoded && g_realtime) {
        result.decode_delay_ms = wall_ms - signal_end_ms;
    } else {
        result.decode_delay_ms = -1;
    }

    return result;
}

// =============================================================================
// PING PROFILING
// =============================================================================

ProfileResult profilePING(float snr_db, size_t lead_silence, std::mt19937& rng) {
    ProfileResult result = {};

    // Create modem engine for TX
    ModemEngine tx_modem;
    tx_modem.setWaveformMode(WaveformMode::MC_DPSK);
    tx_modem.setConnectWaveform(WaveformMode::MC_DPSK);

    // Generate PING signal
    auto tx_samples = tx_modem.transmitPing();

    // Build test signal
    std::vector<float> test_signal(lead_silence, 0.0f);
    test_signal.insert(test_signal.end(), tx_samples.begin(), tx_samples.end());
    test_signal.resize(test_signal.size() + 2400, 0.0f);

    addNoise(test_signal, snr_db, rng);

    // Create fresh RX modem
    ModemEngine rx_modem;
    rx_modem.setWaveformMode(WaveformMode::MC_DPSK);
    rx_modem.setConnectWaveform(WaveformMode::MC_DPSK);

    DecodeSync sync;
    rx_modem.setPingReceivedCallback([&](float snr) {
        sync.signalDecoded();
    });

    // Feed in chunks (480 samples = 10ms at 48kHz)
    const size_t CHUNK_SIZE = 480;
    size_t offset = 0;
    double total_feed_us = 0;
    double signal_end_ms = (lead_silence + tx_samples.size()) / 48.0;

    auto wall_start = Clock::now();

    while (offset < test_signal.size()) {
        size_t chunk_len = std::min(CHUNK_SIZE, test_signal.size() - offset);
        std::vector<float> chunk(test_signal.begin() + offset,
                                  test_signal.begin() + offset + chunk_len);

        auto t0 = Clock::now();
        rx_modem.feedAudio(chunk);
        auto t1 = Clock::now();

        total_feed_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        offset += chunk_len;

        if (sync.decoded.load()) break;

        if (g_realtime) {
            std::this_thread::sleep_for(std::chrono::microseconds(9500));
        }
    }

    // Wait for decode callback (up to 2 seconds)
    bool decoded = sync.waitFor(2000);

    auto wall_end = decoded ? sync.decode_time : Clock::now();
    double wall_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start).count();

    result.feed_time_us = total_feed_us;
    result.total_time_us = wall_ms * 1000.0;
    result.signal_samples = tx_samples.size();
    result.decoded = decoded;

    if (decoded && g_realtime) {
        result.decode_delay_ms = wall_ms - signal_end_ms;
    } else {
        result.decode_delay_ms = -1;
    }

    return result;
}

// =============================================================================
// MAIN
// =============================================================================

void runProfile(const char* name, ProfileResult (*func)(float, size_t, std::mt19937&),
                float snr_db, int trials, size_t lead_silence, std::mt19937& rng) {
    printf("\n=== %s (SNR=%.0f dB)%s ===\n", name, snr_db, g_realtime ? " [REALTIME]" : "");

    double sum_feed = 0, sum_wall = 0, sum_delay = 0;
    double min_feed = 1e9, max_feed = 0;
    double min_delay = 1e9, max_delay = 0;
    int decoded_count = 0;
    int delay_count = 0;
    size_t signal_samples = 0;

    for (int i = 0; i < trials; i++) {
        auto r = func(snr_db, lead_silence, rng);
        signal_samples = r.signal_samples;  // Same for all trials

        if (r.decoded) {
            decoded_count++;
            sum_feed += r.feed_time_us;
            sum_wall += r.total_time_us;
            min_feed = std::min(min_feed, r.feed_time_us);
            max_feed = std::max(max_feed, r.feed_time_us);

            if (g_realtime) {
                sum_delay += r.decode_delay_ms;
                min_delay = std::min(min_delay, r.decode_delay_ms);
                max_delay = std::max(max_delay, r.decode_delay_ms);
            }
        }

        double signal_ms = r.signal_samples / 48.0;
        double wall_ms = r.total_time_us / 1000.0;
        if (g_realtime && r.decoded) {
            // Positive = decode after signal end, Negative = decode before signal end
            printf("  [%2d] delay=%+6.0f ms  wall=%6.1f ms  signal=%5.0f ms  %s\n",
                   i+1, r.decode_delay_ms, wall_ms, signal_ms, "OK");
        } else {
            printf("  [%2d] feed=%6.0f us  wall=%6.1f ms  signal=%5.0f ms  %s\n",
                   i+1, r.feed_time_us, wall_ms, signal_ms,
                   r.decoded ? "OK" : "FAIL");
        }
    }

    if (decoded_count > 0) {
        double avg_feed = sum_feed / decoded_count;
        double avg_wall = sum_wall / decoded_count;
        double signal_ms = signal_samples / 48.0;
        double rt_factor = (signal_samples / 48.0 * 1000.0) / avg_feed;

        printf("\n  Decoded: %d/%d (%.0f%%)\n", decoded_count, trials, 100.0 * decoded_count / trials);
        printf("  Signal duration: %.0f ms (%.1f sec)\n", signal_ms, signal_ms/1000.0);

        if (g_realtime && decoded_count > 0) {
            double avg_delay = sum_delay / decoded_count;
            printf("  *** DECODE DELAY: avg=%+.0f ms (min=%+.0f, max=%+.0f) ***\n",
                   avg_delay, min_delay, max_delay);
            if (avg_delay < 0) {
                printf("  (Negative = decoded %.0f ms BEFORE signal finished)\n", -avg_delay);
            } else {
                printf("  (Time from signal END to decoded data available)\n");
            }
        } else {
            printf("  Feed time: avg=%.0f us, min=%.0f us, max=%.0f us\n", avg_feed, min_feed, max_feed);
            printf("  Wall time: avg=%.1f ms (includes decode)\n", avg_wall/1000.0);
            printf("  Real-time factor: %.0fx (feed only)\n", rt_factor);
        }
    } else {
        printf("\n  No successful decodes!\n");
    }
}

int main(int argc, char** argv) {
    float snr_ofdm = 25.0f;
    float snr_dpsk = 10.0f;
    float snr_ping = 10.0f;
    int trials = 10;
    size_t lead_silence = 12000;  // 250ms
    bool run_ofdm = true, run_dpsk = true, run_ping = true;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--snr-ofdm") == 0 && i+1 < argc) {
            snr_ofdm = atof(argv[++i]);
        } else if (strcmp(argv[i], "--snr-dpsk") == 0 && i+1 < argc) {
            snr_dpsk = atof(argv[++i]);
        } else if (strcmp(argv[i], "--snr-ping") == 0 && i+1 < argc) {
            snr_ping = atof(argv[++i]);
        } else if (strcmp(argv[i], "--trials") == 0 && i+1 < argc) {
            trials = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--silence") == 0 && i+1 < argc) {
            lead_silence = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--ofdm") == 0) {
            run_dpsk = run_ping = false;
        } else if (strcmp(argv[i], "--dpsk") == 0) {
            run_ofdm = run_ping = false;
        } else if (strcmp(argv[i], "--ping") == 0) {
            run_ofdm = run_dpsk = false;
        } else if (strcmp(argv[i], "--realtime") == 0 || strcmp(argv[i], "-r") == 0) {
            g_realtime = true;
        } else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --snr-ofdm dB   OFDM SNR (default: 25)\n");
            printf("  --snr-dpsk dB   DPSK SNR (default: 10)\n");
            printf("  --snr-ping dB   PING SNR (default: 10)\n");
            printf("  --trials N      Trials per mode (default: 10)\n");
            printf("  --silence N     Leading silence samples (default: 12000 = 250ms)\n");
            printf("  --ofdm          Only profile OFDM\n");
            printf("  --dpsk          Only profile DPSK\n");
            printf("  --ping          Only profile PING\n");
            printf("  -r, --realtime  Feed audio at real-time rate (like HF rig)\n");
            return 0;
        }
    }

    printf("╔═══════════════════════════════════════════════════════════╗\n");
    printf("║           ACQUISITION PROFILING                           ║\n");
    printf("╚═══════════════════════════════════════════════════════════╝\n");
    printf("Trials: %d, Lead silence: %.0f ms", trials, lead_silence / 48.0);
    if (g_realtime) {
        printf(", Mode: REALTIME (like HF rig)");
    }
    printf("\n");

    std::mt19937 rng(42);

    if (run_ofdm) runProfile("OFDM", profileOFDM, snr_ofdm, trials, lead_silence, rng);
    if (run_dpsk) runProfile("DPSK", profileDPSK, snr_dpsk, trials, lead_silence, rng);
    if (run_ping) runProfile("PING", profilePING, snr_ping, trials, lead_silence, rng);

    printf("\n");
    return 0;
}
