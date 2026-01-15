#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "waterfall.hpp"
#include <algorithm>
#include <cstring>

// Silence OpenGL deprecation warnings on macOS
#define GL_SILENCE_DEPRECATION

#ifdef __APPLE__
#include <OpenGL/gl.h>
#elif defined(_WIN32)
#define NOMINMAX  // Prevent windows.h from defining min/max macros
#include <windows.h>  // Must come before GL/gl.h on Windows
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace ultra {
namespace gui {

// --- Palette implementations ---

WaterfallPalette WaterfallPalette::createDefault() {
    WaterfallPalette p;
    // Blue -> Cyan -> Green -> Yellow -> Red -> White
    for (int i = 0; i < NUM_COLORS; i++) {
        float t = static_cast<float>(i) / (NUM_COLORS - 1);
        uint8_t r, g, b;

        if (t < 0.2f) {
            // Black to blue
            float s = t / 0.2f;
            r = 0;
            g = 0;
            b = static_cast<uint8_t>(s * 128);
        } else if (t < 0.4f) {
            // Blue to cyan
            float s = (t - 0.2f) / 0.2f;
            r = 0;
            g = static_cast<uint8_t>(s * 255);
            b = static_cast<uint8_t>(128 + s * 127);
        } else if (t < 0.6f) {
            // Cyan to green
            float s = (t - 0.4f) / 0.2f;
            r = 0;
            g = 255;
            b = static_cast<uint8_t>(255 * (1.0f - s));
        } else if (t < 0.8f) {
            // Green to yellow
            float s = (t - 0.6f) / 0.2f;
            r = static_cast<uint8_t>(s * 255);
            g = 255;
            b = 0;
        } else {
            // Yellow to red to white
            float s = (t - 0.8f) / 0.2f;
            r = 255;
            g = static_cast<uint8_t>(255 * (1.0f - s * 0.5f));
            b = static_cast<uint8_t>(s * 255);
        }

        // ABGR format
        p.colors[i] = 0xFF000000 | (b << 16) | (g << 8) | r;
    }
    return p;
}

WaterfallPalette WaterfallPalette::createGrayscale() {
    WaterfallPalette p;
    for (int i = 0; i < NUM_COLORS; i++) {
        uint8_t v = static_cast<uint8_t>(i);
        p.colors[i] = 0xFF000000 | (v << 16) | (v << 8) | v;
    }
    return p;
}

WaterfallPalette WaterfallPalette::createHeat() {
    WaterfallPalette p;
    for (int i = 0; i < NUM_COLORS; i++) {
        float t = static_cast<float>(i) / (NUM_COLORS - 1);
        uint8_t r, g, b;

        // Black -> Red -> Yellow -> White
        if (t < 0.33f) {
            float s = t / 0.33f;
            r = static_cast<uint8_t>(s * 255);
            g = 0;
            b = 0;
        } else if (t < 0.67f) {
            float s = (t - 0.33f) / 0.34f;
            r = 255;
            g = static_cast<uint8_t>(s * 255);
            b = 0;
        } else {
            float s = (t - 0.67f) / 0.33f;
            r = 255;
            g = 255;
            b = static_cast<uint8_t>(s * 255);
        }

        p.colors[i] = 0xFF000000 | (b << 16) | (g << 8) | r;
    }
    return p;
}

// --- WaterfallWidget implementation ---

WaterfallWidget::WaterfallWidget() {
    palette_ = WaterfallPalette::createDefault();
    setFFTSize(2048);
    setHistoryDepth(200);
    createWindow();
}

WaterfallWidget::~WaterfallWidget() {
    destroyTexture();
}

void WaterfallWidget::setFFTSize(int size) {
    // Ensure power of 2
    int actual_size = 1;
    while (actual_size < size) actual_size *= 2;

    if (actual_size != fft_size_) {
        fft_size_ = actual_size;
        fft_ = std::make_unique<FFT>(fft_size_);
        createWindow();

        std::lock_guard<std::mutex> lock(input_mutex_);
        input_buffer_.clear();
        input_buffer_.reserve(fft_size_ * 2);
    }
}

void WaterfallWidget::setHistoryDepth(int lines) {
    if (lines != history_depth_) {
        history_depth_ = lines;
        current_line_ = 0;
        texture_dirty_ = true;
    }
}

void WaterfallWidget::setSampleRate(float rate) {
    sample_rate_ = rate;
}

void WaterfallWidget::setFrequencyRange(float min_hz, float max_hz) {
    min_freq_ = min_hz;
    max_freq_ = max_hz;
    texture_dirty_ = true;
}

void WaterfallWidget::setDynamicRange(float min_db, float max_db) {
    min_db_ = min_db;
    max_db_ = max_db;
}

void WaterfallWidget::setPalette(const WaterfallPalette& palette) {
    palette_ = palette;
    texture_dirty_ = true;
}

void WaterfallWidget::createWindow() {
    window_.resize(fft_size_);
    // Hann window
    for (int i = 0; i < fft_size_; i++) {
        float t = static_cast<float>(i) / (fft_size_ - 1);
        window_[i] = 0.5f * (1.0f - std::cos(2.0f * M_PI * t));
    }
}

int WaterfallWidget::freqToBin(float freq) const {
    float bin_width = sample_rate_ / fft_size_;
    return static_cast<int>(freq / bin_width);
}

int WaterfallWidget::getDisplayBins() const {
    int min_bin = freqToBin(min_freq_);
    int max_bin = freqToBin(max_freq_);
    return std::max(1, max_bin - min_bin);
}

void WaterfallWidget::addSamples(const float* samples, size_t count) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    input_buffer_.insert(input_buffer_.end(), samples, samples + count);

    // Keep buffer from growing too large
    size_t max_buffer = fft_size_ * 4;
    if (input_buffer_.size() > max_buffer) {
        input_buffer_.erase(input_buffer_.begin(),
                           input_buffer_.begin() + (input_buffer_.size() - max_buffer));
    }
}

void WaterfallWidget::processFFT() {
    std::vector<float> samples;

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (input_buffer_.size() < static_cast<size_t>(fft_size_)) {
            return;  // Not enough samples
        }

        // Take fft_size samples, leave overlap
        samples.assign(input_buffer_.begin(), input_buffer_.begin() + fft_size_);

        // Remove half (50% overlap)
        input_buffer_.erase(input_buffer_.begin(), input_buffer_.begin() + fft_size_ / 2);
    }

    // Apply window
    std::vector<Complex> fft_in(fft_size_);
    for (int i = 0; i < fft_size_; i++) {
        fft_in[i] = Complex(samples[i] * window_[i], 0.0f);
    }

    // FFT
    std::vector<Complex> fft_out(fft_size_);
    fft_->forward(fft_in, fft_out);

    // Convert to magnitude (dB) and map to colors
    int min_bin = freqToBin(min_freq_);
    int max_bin = std::min(freqToBin(max_freq_), fft_size_ / 2);
    int display_bins = max_bin - min_bin;

    if (display_bins <= 0) return;

    // Ensure waterfall data is sized correctly
    int texture_w = display_bins;
    int texture_h = history_depth_;

    if (waterfall_data_.size() != static_cast<size_t>(texture_w * texture_h)) {
        waterfall_data_.resize(texture_w * texture_h, 0xFF000000);
        current_line_ = 0;
    }

    // Scroll: shift all lines down, new data goes at top (line 0)
    std::memmove(waterfall_data_.data() + texture_w,
                 waterfall_data_.data(),
                 (texture_h - 1) * texture_w * sizeof(uint32_t));

    // Calculate magnitude for top line (newest data)
    uint32_t* line = waterfall_data_.data();  // Line 0 = top
    float db_range = max_db_ - min_db_;
    float fft_norm = 2.0f / fft_size_;  // Normalize FFT output

    for (int i = 0; i < display_bins; i++) {
        int bin = min_bin + i;
        float mag = std::abs(fft_out[bin]) * fft_norm;
        float db = 20.0f * std::log10(mag + 1e-10f);

        // Normalize to 0-1
        float normalized = (db - min_db_) / db_range;
        normalized = std::max(0.0f, std::min(1.0f, normalized));

        // Map to color
        int color_idx = static_cast<int>(normalized * (WaterfallPalette::NUM_COLORS - 1));
        line[i] = palette_.colors[color_idx];
    }

    texture_dirty_ = true;
}

void WaterfallWidget::createTexture() {
    if (texture_id_ != 0) {
        destroyTexture();
    }

    int width = getDisplayBins();
    int height = history_depth_;

    if (width <= 0 || height <= 0) return;

    glGenTextures(1, &texture_id_);
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    // Initialize with black
    std::vector<uint32_t> black(width * height, 0xFF000000);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, black.data());

    texture_width_ = width;
    texture_height_ = height;

    glBindTexture(GL_TEXTURE_2D, 0);
}

void WaterfallWidget::destroyTexture() {
    if (texture_id_ != 0) {
        glDeleteTextures(1, &texture_id_);
        texture_id_ = 0;
    }
}

void WaterfallWidget::updateTexture() {
    int width = getDisplayBins();
    int height = history_depth_;

    // Recreate texture if size changed
    if (texture_id_ == 0 || texture_width_ != width || texture_height_ != height) {
        createTexture();
        if (texture_id_ == 0) return;
    }

    if (waterfall_data_.size() != static_cast<size_t>(width * height)) {
        return;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
                    GL_RGBA, GL_UNSIGNED_BYTE, waterfall_data_.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    texture_dirty_ = false;
}

void WaterfallWidget::render() {
    // Process any pending audio
    processFFT();

    // Update texture if needed
    if (texture_dirty_) {
        updateTexture();
    }

    // Render
    ImGui::BeginGroup();

    // Frequency labels
    ImGui::Text("Waterfall  %.0f - %.0f Hz", min_freq_, max_freq_);

    if (texture_id_ != 0) {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float aspect = static_cast<float>(texture_width_) / texture_height_;

        // Use available width, calculate height
        float width = avail.x;
        float height = std::min(avail.y - 20, width / aspect);
        height = std::max(height, 100.0f);

        // Render texture (new data at top, scrolls down)
        ImGui::Image(static_cast<ImTextureID>(static_cast<uintptr_t>(texture_id_)),
                     ImVec2(width, height));

        // Draw frequency axis
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 pos = ImGui::GetItemRectMin();
        ImVec2 size = ImGui::GetItemRectSize();

        // Tick marks every 500 Hz
        float freq_range = max_freq_ - min_freq_;
        int num_ticks = static_cast<int>(freq_range / 500);
        for (int i = 0; i <= num_ticks; i++) {
            float freq = min_freq_ + i * 500;
            float x = pos.x + (freq - min_freq_) / freq_range * size.x;

            draw_list->AddLine(ImVec2(x, pos.y + size.y),
                              ImVec2(x, pos.y + size.y + 5),
                              IM_COL32(200, 200, 200, 255));

            char label[16];
            snprintf(label, sizeof(label), "%.0f", freq);
            draw_list->AddText(ImVec2(x - 15, pos.y + size.y + 6),
                              IM_COL32(200, 200, 200, 255), label);
        }
    } else {
        ImGui::Text("No signal data");
    }

    ImGui::EndGroup();
}

} // namespace gui
} // namespace ultra
