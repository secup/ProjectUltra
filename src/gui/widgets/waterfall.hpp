#pragma once

#include "ultra/dsp.hpp"
#include "imgui.h"
#include <vector>
#include <mutex>
#include <cstdint>

namespace ultra {
namespace gui {

// Color palette for waterfall
struct WaterfallPalette {
    static constexpr int NUM_COLORS = 256;
    uint32_t colors[NUM_COLORS];  // ABGR format for OpenGL

    static WaterfallPalette createDefault();
    static WaterfallPalette createGrayscale();
    static WaterfallPalette createHeat();
};

class WaterfallWidget {
public:
    WaterfallWidget();
    ~WaterfallWidget();

    // Configuration
    void setFFTSize(int size);  // Power of 2, default 2048
    void setHistoryDepth(int lines);  // Number of history lines, default 200
    void setSampleRate(float rate);
    void setFrequencyRange(float min_hz, float max_hz);
    void setDynamicRange(float min_db, float max_db);
    void setPalette(const WaterfallPalette& palette);

    // Feed audio samples (called from audio thread or main thread)
    void addSamples(const float* samples, size_t count);

    // Render the waterfall (call from render loop)
    void render();

    // Get current settings
    float getMinFreq() const { return min_freq_; }
    float getMaxFreq() const { return max_freq_; }
    float getMinDB() const { return min_db_; }
    float getMaxDB() const { return max_db_; }

private:
    // FFT processing
    void processFFT();
    void updateTexture();

    // OpenGL texture management
    void createTexture();
    void destroyTexture();

    // FFT
    std::unique_ptr<FFT> fft_;
    int fft_size_ = 0;  // Start at 0 so first setFFTSize() initializes

    // Input buffer
    std::vector<float> input_buffer_;
    std::mutex input_mutex_;

    // Spectrum history (each row is one FFT result)
    std::vector<uint32_t> waterfall_data_;  // ABGR pixels
    int history_depth_ = 200;
    int current_line_ = 0;

    // Display parameters
    float sample_rate_ = 48000.0f;
    float min_freq_ = 0.0f;
    float max_freq_ = 3000.0f;
    float min_db_ = -80.0f;
    float max_db_ = 0.0f;

    // Color palette
    WaterfallPalette palette_;

    // OpenGL texture
    unsigned int texture_id_ = 0;
    int texture_width_ = 0;
    int texture_height_ = 0;
    bool texture_dirty_ = false;

    // Window function
    std::vector<float> window_;
    void createWindow();

    // Frequency bin mapping
    int freqToBin(float freq) const;
    int getDisplayBins() const;
};

} // namespace gui
} // namespace ultra
