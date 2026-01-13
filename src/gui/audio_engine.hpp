#pragma once

#include <SDL.h>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>
#include <string>
#include <functional>

namespace ultra {
namespace gui {

// Audio engine for real-time audio I/O using SDL2
class AudioEngine {
public:
    AudioEngine();
    ~AudioEngine();

    // Initialize SDL audio subsystem
    bool initialize();
    void shutdown();
    bool isInitialized() const { return initialized_; }

    // Device enumeration
    std::vector<std::string> getOutputDevices();
    std::vector<std::string> getInputDevices();

    // Open/close devices
    bool openOutput(const std::string& device = "");  // Empty = default
    bool openInput(const std::string& device = "");
    void closeOutput();
    void closeInput();

    // TX: Queue samples to play
    void queueTxSamples(const std::vector<float>& samples);
    void clearTxQueue();
    bool isTxQueueEmpty() const;
    size_t getTxQueueSize() const;

    // RX: Get captured samples (for real input mode)
    std::vector<float> getRxSamples(size_t max_samples);
    size_t getRxBufferSize() const;

    // Loopback mode: TX samples feed directly to RX
    void setLoopbackEnabled(bool enabled) { loopback_enabled_ = enabled; }
    bool isLoopbackEnabled() const { return loopback_enabled_; }

    // Loopback channel simulation
    void setLoopbackSNR(float snr_db) { loopback_snr_db_ = snr_db; }
    float getLoopbackSNR() const { return loopback_snr_db_; }

    // Playback control
    void startPlayback();
    void stopPlayback();
    bool isPlaying() const { return playing_; }

    // Capture control
    void startCapture();
    void stopCapture();
    bool isCapturing() const { return capturing_; }

    // Audio parameters
    int getSampleRate() const { return sample_rate_; }

    // Audio level metering
    float getInputLevel() const { return input_level_; }
    float getOutputLevel() const { return output_level_; }

    // Callback for when RX has data ready
    using RxCallback = std::function<void(const std::vector<float>&)>;
    void setRxCallback(RxCallback callback) { rx_callback_ = callback; }

private:
    // SDL audio callbacks
    static void outputCallback(void* userdata, Uint8* stream, int len);
    static void inputCallback(void* userdata, Uint8* stream, int len);

    // Add noise for loopback channel simulation
    void addChannelNoise(std::vector<float>& samples);

    SDL_AudioDeviceID output_device_ = 0;
    SDL_AudioDeviceID input_device_ = 0;

    // TX buffer (samples waiting to be played)
    std::queue<float> tx_queue_;
    mutable std::mutex tx_mutex_;

    // RX buffer (captured samples)
    std::vector<float> rx_buffer_;
    mutable std::mutex rx_mutex_;

    // Loopback settings
    std::atomic<bool> loopback_enabled_{false};
    std::atomic<float> loopback_snr_db_{25.0f};

    // State
    std::atomic<bool> playing_{false};
    std::atomic<bool> capturing_{false};
    bool initialized_ = false;

    // Audio parameters
    int sample_rate_ = 48000;
    int buffer_size_ = 1024;

    // RX callback
    RxCallback rx_callback_;

    // Random generator for noise
    uint32_t noise_seed_ = 12345;

    // Audio level metering (RMS, 0.0-1.0)
    std::atomic<float> input_level_{0.0f};
    std::atomic<float> output_level_{0.0f};
};

} // namespace gui
} // namespace ultra
