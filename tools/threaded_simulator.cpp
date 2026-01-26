/**
 * Threaded Simulator - Proper real-time audio simulation
 *
 * This simulates two stations with continuous audio streaming like real radios:
 * - Each station has an RX thread that continuously processes incoming audio
 * - TX samples flow through a simulated channel (with AWGN) to the other station
 * - Half-duplex is enforced by the protocol layer (one TX at a time)
 *
 * This is how the GUI should work - continuous audio flow, not tick-based.
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <queue>
#include <random>
#include <cmath>

#include "gui/modem/modem_engine.hpp"
#include "protocol/protocol_engine.hpp"
#include "ultra/logging.hpp"

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::protocol;

// Thread-safe audio buffer (simulates the "air" between stations)
class AudioChannel {
public:
    AudioChannel(float snr_db) : snr_db_(snr_db) {}

    void setSNR(float snr) { snr_db_ = snr; }

    // Producer: add samples to the channel (with noise)
    void write(const std::vector<float>& samples) {
        if (samples.empty()) return;

        // Apply AWGN
        auto noisy = applyNoise(samples);

        // Add PTT delay (random 100-300ms of noise before signal)
        std::vector<float> ptt_noise = generateNoise(ptt_samples_(rng_));

        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.insert(buffer_.end(), ptt_noise.begin(), ptt_noise.end());
        buffer_.insert(buffer_.end(), noisy.begin(), noisy.end());
        cv_.notify_one();
    }

    // Consumer: read samples from the channel (blocks if empty)
    std::vector<float> read(size_t max_samples, int timeout_ms = 10) {
        std::unique_lock<std::mutex> lock(mutex_);

        // Wait for data or timeout
        cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] {
            return !buffer_.empty() || shutdown_;
        });

        if (buffer_.empty() || shutdown_) {
            return {};
        }

        size_t to_read = std::min(max_samples, buffer_.size());
        std::vector<float> chunk(buffer_.begin(), buffer_.begin() + to_read);
        buffer_.erase(buffer_.begin(), buffer_.begin() + to_read);
        return chunk;
    }

    // Non-blocking read
    std::vector<float> tryRead(size_t max_samples) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.empty()) return {};

        size_t to_read = std::min(max_samples, buffer_.size());
        std::vector<float> chunk(buffer_.begin(), buffer_.begin() + to_read);
        buffer_.erase(buffer_.begin(), buffer_.begin() + to_read);
        return chunk;
    }

    size_t available() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
    }

    void shutdown() {
        shutdown_ = true;
        cv_.notify_all();
    }

private:
    std::vector<float> applyNoise(const std::vector<float>& samples) {
        // Calculate signal RMS
        float sum_sq = 0.0f;
        for (float s : samples) sum_sq += s * s;
        float signal_rms = std::sqrt(sum_sq / samples.size());

        if (signal_rms < 1e-6f) return samples;

        // Calculate noise level for target SNR
        float snr_linear = std::pow(10.0f, snr_db_ / 10.0f);
        float noise_power = (signal_rms * signal_rms) / snr_linear;
        float noise_stddev = std::sqrt(noise_power);

        std::vector<float> noisy = samples;
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        for (float& sample : noisy) {
            sample += noise_dist(rng_);
        }
        return noisy;
    }

    std::vector<float> generateNoise(size_t num_samples) {
        float typical_rms = 0.1f;
        float snr_linear = std::pow(10.0f, snr_db_ / 10.0f);
        float noise_power = (typical_rms * typical_rms) / snr_linear;
        float noise_stddev = std::sqrt(noise_power);

        std::vector<float> noise(num_samples);
        std::normal_distribution<float> dist(0.0f, noise_stddev);
        for (float& s : noise) s = dist(rng_);
        return noise;
    }

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<float> buffer_;
    float snr_db_;
    std::mt19937 rng_{42};
    std::atomic<bool> shutdown_{false};

    // PTT delay: 100-300ms at 48kHz = 4800-14400 samples
    std::uniform_int_distribution<size_t> ptt_samples_{4800, 14400};
};

// A simulated radio station
class Station {
public:
    Station(const std::string& callsign, AudioChannel& tx_channel, AudioChannel& rx_channel)
        : callsign_(callsign), tx_channel_(tx_channel), rx_channel_(rx_channel) {

        modem_.setLogPrefix(callsign);
        protocol_.setLocalCallsign(callsign);
        protocol_.setAutoAccept(true);

        // TX: protocol -> modem -> channel
        protocol_.setTxDataCallback([this](const Bytes& data) {
            LOG_INFO("MODEM", "[%s] TX %zu bytes", callsign_.c_str(), data.size());
            auto samples = modem_.transmit(data);
            tx_channel_.write(samples);
        });

        // RX: modem -> protocol
        modem_.setRawDataCallback([this](const Bytes& data) {
            LOG_INFO("MODEM", "[%s] RX %zu bytes", callsign_.c_str(), data.size());
            protocol_.setMeasuredSNR(measured_snr_);
            protocol_.onRxData(data);
        });

        // Connection state
        protocol_.setConnectionChangedCallback([this](ConnectionState state, const std::string& info) {
            if (state == ConnectionState::CONNECTED) {
                connected_ = true;
                modem_.setConnected(true);
                LOG_INFO("MODEM", "[%s] CONNECTED to %s", callsign_.c_str(), info.c_str());
            } else if (state == ConnectionState::DISCONNECTED) {
                connected_ = false;
                modem_.setConnected(false);
                modem_.setWaveformMode(WaveformMode::OFDM_COX);
                LOG_INFO("MODEM", "[%s] DISCONNECTED", callsign_.c_str());
            }
        });

        // Mode changes
        protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr) {
            modem_.setDataMode(mod, rate);
            LOG_INFO("MODEM", "[%s] MODE -> %s %s", callsign_.c_str(),
                     modulationToString(mod), codeRateToString(rate));
        });

        protocol_.setModeNegotiatedCallback([this](WaveformMode mode) {
            modem_.setWaveformMode(mode);
        });

        protocol_.setConnectWaveformChangedCallback([this](WaveformMode mode) {
            modem_.setConnectWaveform(mode);
        });

        protocol_.setHandshakeConfirmedCallback([this]() {
            modem_.setHandshakeComplete(true);
        });

        protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
            LOG_INFO("MODEM", "[%s] MESSAGE from %s: \"%s\"", callsign_.c_str(), from.c_str(), text.c_str());
            last_message_ = text;
            message_received_ = true;
        });
    }

    ~Station() {
        stop();
    }

    void start() {
        running_ = true;
        rx_thread_ = std::thread(&Station::rxLoop, this);
        protocol_thread_ = std::thread(&Station::protocolLoop, this);
    }

    void stop() {
        running_ = false;
        if (rx_thread_.joinable()) rx_thread_.join();
        if (protocol_thread_.joinable()) protocol_thread_.join();
    }

    void connect(const std::string& remote) {
        protocol_.connect(remote);
    }

    void disconnect() {
        protocol_.disconnect();
    }

    void sendMessage(const std::string& msg) {
        protocol_.sendMessage(msg);
    }

    bool isConnected() const { return connected_; }
    bool isReadyToSend() const { return protocol_.isReadyToSend(); }
    bool messageReceived() const { return message_received_; }
    std::string getLastMessage() const { return last_message_; }
    void clearMessageFlag() { message_received_ = false; last_message_.clear(); }

    void setMeasuredSNR(float snr) { measured_snr_ = snr; }
    void setInitialWaveform(WaveformMode mode) {
        protocol_.setInitialConnectWaveform(mode);
        modem_.setConnectWaveform(mode);
    }

private:
    void rxLoop() {
        LOG_INFO("MODEM", "[%s] RX thread started", callsign_.c_str());

        while (running_) {
            // Read audio from channel (10ms chunks at 48kHz = 480 samples)
            auto samples = rx_channel_.read(480, 10);

            if (!samples.empty()) {
                modem_.feedAudio(samples);
            }
        }

        LOG_INFO("MODEM", "[%s] RX thread stopped", callsign_.c_str());
    }

    void protocolLoop() {
        LOG_INFO("MODEM", "[%s] Protocol thread started", callsign_.c_str());

        auto last_tick = std::chrono::steady_clock::now();

        while (running_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick).count();

            if (elapsed >= 16) {  // ~60Hz tick rate
                protocol_.tick(elapsed);
                last_tick = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        LOG_INFO("MODEM", "[%s] Protocol thread stopped", callsign_.c_str());
    }

    std::string callsign_;
    ModemEngine modem_;
    ProtocolEngine protocol_{ConnectionConfig{}};

    AudioChannel& tx_channel_;
    AudioChannel& rx_channel_;

    std::thread rx_thread_;
    std::thread protocol_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<bool> message_received_{false};
    std::string last_message_;
    float measured_snr_ = 20.0f;
};

// ============================================================================
// Main test harness
// ============================================================================

void printHeader(float snr) {
    std::cout << "\n";
    std::cout << "====================================================================\n";
    std::cout << "     THREADED SIMULATOR - Real-time Audio Streaming Test\n";
    std::cout << "====================================================================\n";
    std::cout << "  SNR: " << snr << " dB\n";
    std::cout << "  Stations: ALPHA <-> BRAVO\n";
    std::cout << "====================================================================\n\n";
}

bool waitFor(std::function<bool()> condition, int timeout_sec, const std::string& desc) {
    auto start = std::chrono::steady_clock::now();
    while (!condition()) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= timeout_sec) {
            std::cout << "  TIMEOUT waiting for: " << desc << "\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}

int main(int argc, char* argv[]) {
    float snr_db = 20.0f;
    bool verbose = false;

    // Parse args
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "-snr" || arg == "--snr") && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Threaded Simulator\n";
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "  -snr <dB>   Set channel SNR (default: 20)\n";
            std::cout << "  -v          Verbose logging\n";
            return 0;
        }
    }

    printHeader(snr_db);

    // Create bidirectional channels
    // ALPHA TX -> channel_a2b -> BRAVO RX
    // BRAVO TX -> channel_b2a -> ALPHA RX
    AudioChannel channel_a2b(snr_db);
    AudioChannel channel_b2a(snr_db);

    // Create stations
    Station alpha("ALPHA", channel_a2b, channel_b2a);
    Station bravo("BRAVO", channel_b2a, channel_a2b);

    alpha.setMeasuredSNR(snr_db);
    bravo.setMeasuredSNR(snr_db);

    // Use DPSK for initial connection (works at lower SNR)
    alpha.setInitialWaveform(WaveformMode::MC_DPSK);
    bravo.setInitialWaveform(WaveformMode::MC_DPSK);

    // Start the stations
    std::cout << "Starting stations...\n";
    alpha.start();
    bravo.start();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // === TEST 1: Connection ===
    std::cout << "\n=== TEST 1: CONNECTION ===\n";
    std::cout << "  ALPHA connecting to BRAVO...\n";
    alpha.connect("BRAVO");

    if (!waitFor([&]{ return alpha.isConnected() && bravo.isConnected(); }, 120, "connection")) {
        std::cout << "  FAILED: Connection not established\n";
        alpha.stop();
        bravo.stop();
        return 1;
    }
    std::cout << "  SUCCESS: Both stations connected!\n";

    // Wait for ARQ to be ready
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // === TEST 2: Message Transfer ===
    std::cout << "\n=== TEST 2: MESSAGE TRANSFER ===\n";

    std::vector<std::string> messages = {
        "Hello World!",
        "Second message",
        "Third message"
    };

    for (size_t i = 0; i < messages.size(); i++) {
        const std::string& msg = messages[i];
        bravo.clearMessageFlag();

        // Wait for ARQ ready
        if (!waitFor([&]{ return alpha.isReadyToSend(); }, 60, "ARQ ready")) {
            std::cout << "  FAILED: ARQ not ready for message " << (i+1) << "\n";
            alpha.stop();
            bravo.stop();
            return 1;
        }

        std::cout << "  Sending message " << (i+1) << ": \"" << msg << "\"\n";
        alpha.sendMessage(msg);

        if (!waitFor([&]{ return bravo.messageReceived(); }, 120, "message received")) {
            std::cout << "  FAILED: Message " << (i+1) << " not received\n";
            alpha.stop();
            bravo.stop();
            return 1;
        }

        if (bravo.getLastMessage() == msg) {
            std::cout << "  SUCCESS: Message " << (i+1) << " received correctly!\n";
        } else {
            std::cout << "  FAILED: Message corrupted!\n";
            std::cout << "    Expected: \"" << msg << "\"\n";
            std::cout << "    Received: \"" << bravo.getLastMessage() << "\"\n";
            alpha.stop();
            bravo.stop();
            return 1;
        }
    }

    // === TEST 3: Disconnect ===
    std::cout << "\n=== TEST 3: DISCONNECT ===\n";
    std::cout << "  ALPHA disconnecting...\n";
    alpha.disconnect();

    if (!waitFor([&]{ return !alpha.isConnected() && !bravo.isConnected(); }, 60, "disconnect")) {
        std::cout << "  FAILED: Disconnect not completed\n";
        alpha.stop();
        bravo.stop();
        return 1;
    }
    std::cout << "  SUCCESS: Both stations disconnected!\n";

    // Cleanup
    channel_a2b.shutdown();
    channel_b2a.shutdown();
    alpha.stop();
    bravo.stop();

    std::cout << "\n====================================================================\n";
    std::cout << "                    ALL TESTS PASSED!\n";
    std::cout << "====================================================================\n\n";

    return 0;
}
