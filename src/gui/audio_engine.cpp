#include "audio_engine.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace ultra {
namespace gui {

AudioEngine::AudioEngine() = default;

AudioEngine::~AudioEngine() {
    shutdown();
}

bool AudioEngine::initialize() {
    if (initialized_) return true;

    if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
        return false;
    }

    initialized_ = true;
    return true;
}

void AudioEngine::shutdown() {
    closeOutput();
    closeInput();

    if (initialized_) {
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        initialized_ = false;
    }
}

std::vector<std::string> AudioEngine::getOutputDevices() {
    std::vector<std::string> devices;
    devices.push_back("Default");

    int count = SDL_GetNumAudioDevices(0);  // 0 = output devices
    for (int i = 0; i < count; ++i) {
        const char* name = SDL_GetAudioDeviceName(i, 0);
        if (name) {
            devices.push_back(name);
        }
    }
    return devices;
}

std::vector<std::string> AudioEngine::getInputDevices() {
    std::vector<std::string> devices;
    devices.push_back("Default");

    int count = SDL_GetNumAudioDevices(1);  // 1 = input devices
    for (int i = 0; i < count; ++i) {
        const char* name = SDL_GetAudioDeviceName(i, 1);
        if (name) {
            devices.push_back(name);
        }
    }
    return devices;
}

bool AudioEngine::openOutput(const std::string& device) {
    if (!initialized_) {
        if (!initialize()) return false;
    }

    closeOutput();

    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = sample_rate_;
    want.format = AUDIO_F32SYS;  // 32-bit float, system byte order
    want.channels = 1;           // Mono
    want.samples = buffer_size_;
    want.callback = outputCallback;
    want.userdata = this;

    const char* dev_name = (device.empty() || device == "Default") ? nullptr : device.c_str();

    output_device_ = SDL_OpenAudioDevice(dev_name, 0, &want, &have, 0);
    if (output_device_ == 0) {
        return false;
    }

    sample_rate_ = have.freq;
    return true;
}

bool AudioEngine::openInput(const std::string& device) {
    if (!initialized_) {
        if (!initialize()) return false;
    }

    closeInput();

    SDL_AudioSpec want, have;
    SDL_zero(want);
    want.freq = sample_rate_;
    want.format = AUDIO_F32SYS;
    want.channels = 1;
    want.samples = buffer_size_;
    want.callback = inputCallback;
    want.userdata = this;

    const char* dev_name = (device.empty() || device == "Default") ? nullptr : device.c_str();

    input_device_ = SDL_OpenAudioDevice(dev_name, 1, &want, &have, 0);
    if (input_device_ == 0) {
        return false;
    }

    return true;
}

void AudioEngine::closeOutput() {
    if (output_device_ != 0) {
        SDL_CloseAudioDevice(output_device_);
        output_device_ = 0;
    }
    playing_ = false;
}

void AudioEngine::closeInput() {
    if (input_device_ != 0) {
        SDL_CloseAudioDevice(input_device_);
        input_device_ = 0;
    }
    capturing_ = false;
}

void AudioEngine::queueTxSamples(const std::vector<float>& samples) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    for (float s : samples) {
        tx_queue_.push(s);
    }

    // If loopback is enabled, also feed to RX (with optional channel simulation)
    if (loopback_enabled_) {
        std::vector<float> loopback_samples = samples;
        addChannelNoise(loopback_samples);

        std::lock_guard<std::mutex> rx_lock(rx_mutex_);
        rx_buffer_.insert(rx_buffer_.end(), loopback_samples.begin(), loopback_samples.end());

        // NOTE: Don't call rx_callback_ here - it creates a synchronous loop
        // (TX -> loopback -> RX callback -> process -> TX -> ...)
        // The loopback samples are in rx_buffer_ and will be processed
        // in the next pollRxAudio() call from the main loop.
    }
}

void AudioEngine::clearTxQueue() {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    std::queue<float> empty;
    std::swap(tx_queue_, empty);
}

bool AudioEngine::isTxQueueEmpty() const {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    return tx_queue_.empty();
}

size_t AudioEngine::getTxQueueSize() const {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    return tx_queue_.size();
}

std::vector<float> AudioEngine::getRxSamples(size_t max_samples) {
    std::lock_guard<std::mutex> lock(rx_mutex_);

    size_t count = std::min(max_samples, rx_buffer_.size());
    std::vector<float> samples(rx_buffer_.begin(), rx_buffer_.begin() + count);
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + count);

    return samples;
}

size_t AudioEngine::getRxBufferSize() const {
    std::lock_guard<std::mutex> lock(rx_mutex_);
    return rx_buffer_.size();
}

void AudioEngine::startPlayback() {
    if (output_device_ != 0) {
        SDL_PauseAudioDevice(output_device_, 0);  // 0 = unpause
        playing_ = true;
    }
}

void AudioEngine::stopPlayback() {
    if (output_device_ != 0) {
        SDL_PauseAudioDevice(output_device_, 1);  // 1 = pause
        playing_ = false;
    }
}

void AudioEngine::startCapture() {
    if (input_device_ != 0) {
        SDL_PauseAudioDevice(input_device_, 0);
        capturing_ = true;
    }
}

void AudioEngine::stopCapture() {
    if (input_device_ != 0) {
        SDL_PauseAudioDevice(input_device_, 1);
        capturing_ = false;
    }
}

void AudioEngine::outputCallback(void* userdata, Uint8* stream, int len) {
    AudioEngine* engine = static_cast<AudioEngine*>(userdata);

    float* output = reinterpret_cast<float*>(stream);
    int samples = len / sizeof(float);

    float sum_sq = 0.0f;

    std::lock_guard<std::mutex> lock(engine->tx_mutex_);

    for (int i = 0; i < samples; ++i) {
        if (!engine->tx_queue_.empty()) {
            output[i] = engine->tx_queue_.front();
            engine->tx_queue_.pop();
        } else {
            output[i] = 0.0f;  // Silence when no data
        }
        sum_sq += output[i] * output[i];
    }

    // Update output level (RMS)
    float rms = std::sqrt(sum_sq / samples);
    engine->output_level_ = rms;
}

void AudioEngine::inputCallback(void* userdata, Uint8* stream, int len) {
    AudioEngine* engine = static_cast<AudioEngine*>(userdata);

    const float* input = reinterpret_cast<const float*>(stream);
    int samples = len / sizeof(float);

    // Compute input level (RMS)
    float sum_sq = 0.0f;
    for (int i = 0; i < samples; ++i) {
        sum_sq += input[i] * input[i];
    }
    float rms = std::sqrt(sum_sq / samples);
    engine->input_level_ = rms;

    std::vector<float> captured(input, input + samples);

    {
        std::lock_guard<std::mutex> lock(engine->rx_mutex_);
        engine->rx_buffer_.insert(engine->rx_buffer_.end(), captured.begin(), captured.end());
    }

    // Notify via callback
    if (engine->rx_callback_) {
        engine->rx_callback_(captured);
    }
}

void AudioEngine::addChannelNoise(std::vector<float>& samples) {
    float snr_db = loopback_snr_db_.load();

    // Skip noise entirely for very high SNR (clean testing mode)
    if (snr_db >= 100.0f) {
        return;  // No noise added - perfect loopback
    }

    float noise_std = std::pow(10.0f, -snr_db / 20.0f);

    // Simple AWGN using Box-Muller transform
    for (size_t i = 0; i < samples.size(); ++i) {
        // Simple LCG random number generator
        noise_seed_ = noise_seed_ * 1103515245 + 12345;
        float u1 = (noise_seed_ & 0x7FFFFFFF) / (float)0x7FFFFFFF;
        noise_seed_ = noise_seed_ * 1103515245 + 12345;
        float u2 = (noise_seed_ & 0x7FFFFFFF) / (float)0x7FFFFFFF;

        // Box-Muller
        if (u1 < 1e-10f) u1 = 1e-10f;
        float noise = std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * M_PI * u2);

        samples[i] += noise * noise_std * 0.1f;  // Scale noise to signal level
    }
}

} // namespace gui
} // namespace ultra
