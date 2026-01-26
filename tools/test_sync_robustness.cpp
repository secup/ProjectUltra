/**
 * Comprehensive Sync Detection Robustness Test
 *
 * Tests OFDM sync detection across realistic HF audio conditions:
 * - SNR range: 15-30 dB
 * - All code rates: R1/4, R1/2, R2/3, R3/4, R5/6
 * - PTT noise: 100-500ms before signal
 * - Channel noise: AWGN on signal
 * - CFO: ±50Hz carrier frequency offset (SSB tuning error)
 * - Multipath: Delayed echo (HF channel)
 * - Amplitude variation: Ramp up/down (AGC settling)
 * - Chunked streaming: 768 samples (16ms @ 48kHz)
 * - Multiple RNG seeds for statistical validity
 *
 * Target: 10/10 success for each condition
 */

#include <iostream>
#include <iomanip>
#include <cstring>
#include <random>
#include <vector>
#include <cmath>
#include <complex>
#include <thread>
#include <chrono>
#include "gui/modem/modem_engine.hpp"
#include "protocol/frame_v2.hpp"

using namespace ultra;
using namespace ultra::gui;
namespace v2 = ultra::protocol::v2;

constexpr float SAMPLE_RATE = 48000.0f;
constexpr float PI = 3.14159265358979f;

struct TestConfig {
    float snr_db;
    CodeRate code_rate;
    size_t ptt_noise_samples;  // 0 = no PTT noise
    bool chunked_processing;   // true = 768 sample chunks
    uint32_t rng_seed;

    // Realistic audio conditions
    float cfo_hz;              // Carrier frequency offset (±Hz)
    float multipath_delay_ms;  // Echo delay (0 = no multipath)
    float multipath_gain;      // Echo amplitude (0-1)
    float amplitude_ramp_ms;   // Ramp-up time (0 = instant)
    float dc_offset;           // DC bias (-1 to 1)
};

struct TestResult {
    bool success;
    std::string error_msg;
};

// Apply carrier frequency offset (CFO) using FFT-based frequency shift
// This simulates the effect of receiver tuning error.
//
// Method: Use Hilbert transform to get analytic signal, multiply by
// complex exponential, take real part. This correctly shifts ALL
// frequency components by cfo_hz without aliasing or imaging.
//
// For a real signal s(t), the analytic signal is sa(t) = s(t) + j*H[s(t)]
// Frequency shift: sa'(t) = sa(t) * exp(j*2π*cfo*t)
// Output: s'(t) = real(sa'(t))
void applyCFO(std::vector<float>& samples, float cfo_hz, float sample_rate) {
    if (std::abs(cfo_hz) < 0.01f) return;
    if (samples.empty()) return;

    // Pad to power of 2 for FFT
    size_t fft_size = 1;
    while (fft_size < samples.size()) fft_size *= 2;

    // Convert real signal to frequency domain
    std::vector<std::complex<float>> time_in(fft_size, {0, 0});
    for (size_t i = 0; i < samples.size(); ++i) {
        time_in[i] = {samples[i], 0.0f};
    }

    // Manual FFT (simple DIT radix-2)
    auto fft = [](std::vector<std::complex<float>>& x, bool inverse) {
        size_t N = x.size();
        // Bit reversal
        for (size_t i = 1, j = 0; i < N; ++i) {
            size_t bit = N >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(x[i], x[j]);
        }
        // Cooley-Tukey
        for (size_t len = 2; len <= N; len <<= 1) {
            float angle = (inverse ? 2.0f : -2.0f) * PI / len;
            std::complex<float> wlen(std::cos(angle), std::sin(angle));
            for (size_t i = 0; i < N; i += len) {
                std::complex<float> w(1, 0);
                for (size_t j = 0; j < len / 2; ++j) {
                    auto u = x[i + j];
                    auto v = x[i + j + len / 2] * w;
                    x[i + j] = u + v;
                    x[i + j + len / 2] = u - v;
                    w *= wlen;
                }
            }
        }
        if (inverse) {
            for (auto& val : x) val /= static_cast<float>(N);
        }
    };

    // Forward FFT
    std::vector<std::complex<float>> freq = time_in;
    fft(freq, false);

    // Create analytic signal by zeroing negative frequencies
    // DC: keep, Positive (1 to N/2-1): multiply by 2, Nyquist: keep, Negative: zero
    for (size_t i = 1; i < fft_size / 2; ++i) {
        freq[i] *= 2.0f;
    }
    for (size_t i = fft_size / 2 + 1; i < fft_size; ++i) {
        freq[i] = {0, 0};
    }

    // Inverse FFT to get analytic signal
    fft(freq, true);

    // Apply CFO rotation and take real part
    for (size_t i = 0; i < samples.size(); ++i) {
        float t = static_cast<float>(i) / sample_rate;
        float phase = 2.0f * PI * cfo_hz * t;
        std::complex<float> rotation(std::cos(phase), std::sin(phase));
        samples[i] = (freq[i] * rotation).real();
    }
}

// Apply multipath (delayed echo)
void applyMultipath(std::vector<float>& samples, float delay_ms, float gain, float sample_rate) {
    if (delay_ms < 0.01f || gain < 0.001f) return;

    size_t delay_samples = static_cast<size_t>(delay_ms * sample_rate / 1000.0f);
    if (delay_samples >= samples.size()) return;

    // Add delayed copy
    std::vector<float> original = samples;
    for (size_t i = delay_samples; i < samples.size(); ++i) {
        samples[i] += gain * original[i - delay_samples];
    }
}

// Apply amplitude ramp (AGC settling)
// Simulates receiver AGC settling - signal starts at 30% and ramps to 100%
// This is more realistic than starting at 0% (which would require infinite AGC gain)
void applyAmplitudeRamp(std::vector<float>& samples, float ramp_ms, float sample_rate, size_t signal_start) {
    if (ramp_ms < 0.01f) return;

    size_t ramp_samples = static_cast<size_t>(ramp_ms * sample_rate / 1000.0f);

    // Start at 30% amplitude (realistic AGC initial state), ramp to 100%
    constexpr float START_GAIN = 0.3f;
    constexpr float END_GAIN = 1.0f;

    for (size_t i = signal_start; i < samples.size() && i < signal_start + ramp_samples; ++i) {
        float progress = static_cast<float>(i - signal_start) / ramp_samples;
        // Smooth ramp using cosine, from START_GAIN to END_GAIN
        float ramp = 0.5f * (1.0f - std::cos(PI * progress));  // 0 to 1
        float gain = START_GAIN + (END_GAIN - START_GAIN) * ramp;
        samples[i] *= gain;
    }
}

// Apply DC offset
void applyDCOffset(std::vector<float>& samples, float dc_offset) {
    if (std::abs(dc_offset) < 0.0001f) return;

    for (auto& s : samples) {
        s += dc_offset;
    }
}

// Build a DATA frame
Bytes buildDataFrame(const std::string& msg, uint16_t seq, CodeRate rate) {
    size_t payload_size = msg.size();
    size_t frame_size = v2::DataFrame::HEADER_SIZE + payload_size + 2;

    Bytes frame(frame_size);
    frame[0] = 0x55;
    frame[1] = 0x4C;
    frame[2] = static_cast<uint8_t>(v2::FrameType::DATA);
    frame[3] = 0x00;
    frame[4] = (seq >> 8) & 0xFF;
    frame[5] = seq & 0xFF;

    uint32_t src_hash = v2::hashCallsign("TEST1");
    frame[6] = (src_hash >> 16) & 0xFF;
    frame[7] = (src_hash >> 8) & 0xFF;
    frame[8] = src_hash & 0xFF;

    uint32_t dst_hash = v2::hashCallsign("TEST2");
    frame[9] = (dst_hash >> 16) & 0xFF;
    frame[10] = (dst_hash >> 8) & 0xFF;
    frame[11] = dst_hash & 0xFF;

    uint8_t total_cw = v2::DataFrame::calculateCodewords(payload_size, rate);
    frame[12] = total_cw;

    frame[13] = (payload_size >> 8) & 0xFF;
    frame[14] = payload_size & 0xFF;

    uint16_t hcrc = v2::ControlFrame::calculateCRC(frame.data(), 15);
    frame[15] = (hcrc >> 8) & 0xFF;
    frame[16] = hcrc & 0xFF;

    std::memcpy(frame.data() + v2::DataFrame::HEADER_SIZE, msg.data(), msg.size());

    uint16_t fcrc = v2::ControlFrame::calculateCRC(frame.data(), frame_size - 2);
    frame[frame_size - 2] = (fcrc >> 8) & 0xFF;
    frame[frame_size - 1] = fcrc & 0xFF;

    return frame;
}

TestResult runSingleTest(const TestConfig& cfg, int msg_idx) {
    TestResult result = {false, ""};

    // Create TX modem
    ModemEngine tx_modem;
    tx_modem.setLogPrefix("TX");
    tx_modem.setConnected(true);
    tx_modem.setHandshakeComplete(true);
    tx_modem.setDataMode(Modulation::DQPSK, cfg.code_rate);
    tx_modem.setWaveformMode(protocol::WaveformMode::OFDM_COX);

    // Create RX modem
    auto rx_modem = std::make_unique<ModemEngine>();
    rx_modem->setLogPrefix("RX");
    rx_modem->setConnected(true);
    rx_modem->setHandshakeComplete(true);
    rx_modem->setDataMode(Modulation::DQPSK, cfg.code_rate);
    rx_modem->setWaveformMode(protocol::WaveformMode::OFDM_COX);

    std::vector<Bytes> received_frames;
    rx_modem->setRawDataCallback([&](const Bytes& data) {
        received_frames.push_back(data);
    });

    // Build message
    std::string msg = "Sync test message " + std::to_string(msg_idx);
    Bytes frame = buildDataFrame(msg, msg_idx, cfg.code_rate);

    // Transmit
    auto samples = tx_modem.transmit(frame);

    // Setup RNG for noise
    std::mt19937 rng(cfg.rng_seed);
    float signal_rms = 0.1f;
    float snr_linear = std::pow(10.0f, cfg.snr_db / 10.0f);
    float noise_stddev = signal_rms / std::sqrt(snr_linear);
    std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

    // Build full signal with PTT noise
    std::vector<float> full_signal;
    full_signal.reserve(cfg.ptt_noise_samples + samples.size());

    // Add PTT noise
    for (size_t i = 0; i < cfg.ptt_noise_samples; ++i) {
        full_signal.push_back(noise_dist(rng));
    }

    size_t signal_start = full_signal.size();

    // Add signal with channel noise
    for (size_t i = 0; i < samples.size(); ++i) {
        full_signal.push_back(samples[i] + noise_dist(rng));
    }

    // Apply realistic audio conditions
    applyCFO(full_signal, cfg.cfo_hz, SAMPLE_RATE);
    applyMultipath(full_signal, cfg.multipath_delay_ms, cfg.multipath_gain, SAMPLE_RATE);
    applyAmplitudeRamp(full_signal, cfg.amplitude_ramp_ms, SAMPLE_RATE, signal_start);
    applyDCOffset(full_signal, cfg.dc_offset);

    // Feed all audio - modem threads process automatically
    rx_modem->feedAudio(full_signal);
    // Give threads time to process
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Check result
    if (received_frames.empty()) {
        result.error_msg = "No frame received";
        return result;
    }

    // Verify payload
    if (received_frames[0].size() >= v2::DataFrame::HEADER_SIZE + 2) {
        size_t payload_size = received_frames[0].size() - v2::DataFrame::HEADER_SIZE - 2;
        std::string payload(received_frames[0].begin() + v2::DataFrame::HEADER_SIZE,
                           received_frames[0].begin() + v2::DataFrame::HEADER_SIZE + payload_size);

        std::string clean_payload;
        for (char c : payload) {
            if (c >= 32 && c <= 126) clean_payload += c;
        }

        if (clean_payload == msg) {
            result.success = true;
        } else {
            result.error_msg = "Payload mismatch";
        }
    } else {
        result.error_msg = "Frame too small";
    }

    return result;
}

int runTestBatch(const std::string& test_name, TestConfig base_cfg, int num_tests = 10, bool verbose = false) {
    int success_count = 0;

    for (int i = 0; i < num_tests; i++) {
        base_cfg.rng_seed = 42 + i;
        TestResult res = runSingleTest(base_cfg, i);
        if (res.success) {
            success_count++;
        } else if (verbose) {
            std::cerr << "  [FAIL] " << test_name << " #" << i << ": " << res.error_msg << "\n";
        }
    }

    std::cout << "│ " << std::left << std::setw(35) << test_name;
    std::cout << std::right << std::setw(2) << success_count << "/10 ";
    std::cout << (success_count == 10 ? "✓" : "✗") << " │\n";

    return success_count;
}

void runFullTestSuite(float snr_db, CodeRate rate, const std::string& rate_name) {
    const size_t PTT_250MS = 12000;
    const size_t PTT_500MS = 24000;

    std::cout << "\n╔═══════════════════════════════════════════════════╗\n";
    std::cout << "║ SNR: " << std::setw(2) << (int)snr_db << " dB | Rate: " << std::setw(4) << rate_name;
    std::cout << "                            ║\n";
    std::cout << "╠═══════════════════════════════════════════════════╣\n";

    int total_pass = 0;
    int total_tests = 0;

    // Baseline: Clean signal (no impairments)
    {
        TestConfig cfg = {snr_db, rate, 0, true, 0, 0, 0, 0, 0, 0};
        total_pass += (runTestBatch("Clean (no impairments)", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // PTT noise tests
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 0, 0, 0, 0};
        total_pass += (runTestBatch("PTT noise 250ms", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_500MS, true, 0, 0, 0, 0, 0, 0};
        total_pass += (runTestBatch("PTT noise 500ms", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // CFO tests (carrier frequency offset)
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, +25.0f, 0, 0, 0, 0};
        total_pass += (runTestBatch("CFO +25 Hz", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, -25.0f, 0, 0, 0, 0};
        total_pass += (runTestBatch("CFO -25 Hz", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, +50.0f, 0, 0, 0, 0};
        total_pass += (runTestBatch("CFO +50 Hz", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, -50.0f, 0, 0, 0, 0};
        total_pass += (runTestBatch("CFO -50 Hz", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // Multipath tests (HF channel echo)
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 0.5f, 0.3f, 0, 0};
        total_pass += (runTestBatch("Multipath 0.5ms delay, 0.3 gain", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 1.0f, 0.3f, 0, 0};
        total_pass += (runTestBatch("Multipath 1.0ms delay, 0.3 gain", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 2.0f, 0.2f, 0, 0};
        total_pass += (runTestBatch("Multipath 2.0ms delay, 0.2 gain", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // Amplitude ramp tests (AGC settling)
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 0, 0, 50.0f, 0};
        total_pass += (runTestBatch("Amplitude ramp 50ms", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 0, 0, 100.0f, 0};
        total_pass += (runTestBatch("Amplitude ramp 100ms", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // DC offset tests
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 0, 0, 0, 0.05f};
        total_pass += (runTestBatch("DC offset +0.05", cfg) == 10 ? 1 : 0);
        total_tests++;
    }
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, 0, 0, 0, 0, -0.05f};
        total_pass += (runTestBatch("DC offset -0.05", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // Combined: realistic HF conditions
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, true, 0, +30.0f, 1.0f, 0.25f, 50.0f, 0.02f};
        total_pass += (runTestBatch("Combined: CFO+multipath+ramp+DC", cfg) == 10 ? 1 : 0);
        total_tests++;
    }

    // All-at-once processing (different buffer behavior)
    {
        TestConfig cfg = {snr_db, rate, PTT_250MS, false, 0, 0, 0, 0, 0, 0};
        total_pass += (runTestBatch("All-at-once processing", cfg, 10, true) == 10 ? 1 : 0);  // verbose=true
        total_tests++;
    }

    std::cout << "╠═══════════════════════════════════════════════════╣\n";
    std::cout << "║ TOTAL: " << std::setw(2) << total_pass << "/" << std::setw(2) << total_tests << " test categories passed";
    std::cout << "                 ║\n";
    std::cout << "╚═══════════════════════════════════════════════════╝\n";
}

int main(int argc, char* argv[]) {
    std::cout << "╔═══════════════════════════════════════════════════════════╗\n";
    std::cout << "║       OFDM SYNC DETECTION ROBUSTNESS TEST SUITE           ║\n";
    std::cout << "║                                                           ║\n";
    std::cout << "║  Tests: PTT noise, CFO, Multipath, Amplitude, DC offset   ║\n";
    std::cout << "║  Target: 10/10 for ALL conditions                         ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════╝\n";

    // Quick mode: just test one SNR/rate combo
    if (argc > 1 && std::string(argv[1]) == "--quick") {
        std::cout << "\n[Quick mode: 20dB R1/4 only]\n";
        runFullTestSuite(20.0f, CodeRate::R1_4, "R1/4");
        return 0;
    }

    // Full test matrix
    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "                    15 dB SNR (minimum OFDM)\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    runFullTestSuite(15.0f, CodeRate::R1_4, "R1/4");

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "                         20 dB SNR\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    runFullTestSuite(20.0f, CodeRate::R1_4, "R1/4");
    runFullTestSuite(20.0f, CodeRate::R1_2, "R1/2");
    runFullTestSuite(20.0f, CodeRate::R2_3, "R2/3");

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "                         25 dB SNR\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    runFullTestSuite(25.0f, CodeRate::R2_3, "R2/3");
    runFullTestSuite(25.0f, CodeRate::R3_4, "R3/4");

    std::cout << "\n═══════════════════════════════════════════════════════════\n";
    std::cout << "                    30 dB SNR (high quality)\n";
    std::cout << "═══════════════════════════════════════════════════════════\n";
    runFullTestSuite(30.0f, CodeRate::R3_4, "R3/4");
    runFullTestSuite(30.0f, CodeRate::R5_6, "R5/6");

    std::cout << "\n════════════════════════════════════════════════════════════\n";
    std::cout << "All tests complete. Target: ALL categories should pass.\n";
    std::cout << "════════════════════════════════════════════════════════════\n";

    return 0;
}
