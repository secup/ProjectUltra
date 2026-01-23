#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>

namespace ultra {
namespace gui {

// Settings that persist across sessions
struct AppSettings {
    // Save/load to file
    bool save(const std::string& path = "") const;
    bool load(const std::string& path = "");

    // Get default settings file path
    static std::string getDefaultPath();

    // Station Info
    char callsign[16] = "N0CALL";  // Standard placeholder callsign
    char grid_square[8] = "";      // Maidenhead locator
    char name[32] = "";

    // Radio Settings (for future Hamlib integration)
    char rig_model[32] = "None";
    char rig_port[64] = "";
    int rig_baud = 9600;
    bool use_cat_ptt = false;  // Use CAT for PTT vs VOX

    // Audio Settings
    char input_device[128] = "Default";   // Selected input device name
    char output_device[128] = "Default";  // Selected output device name
    int tx_delay_ms = 50;      // Delay before TX after PTT
    int tx_tail_ms = 50;       // Delay after TX before releasing PTT
    float tx_drive = 0.8f;     // TX audio level (0-1)

    // Audio Filter Settings
    bool filter_enabled = false;      // Disabled by default (radio's SSB filter sufficient)
    float filter_center = 1500.0f;    // Center frequency in Hz
    float filter_bandwidth = 2900.0f; // Total bandwidth in Hz (covers 2.8 kHz modem)
    int filter_taps = 101;            // FIR filter taps

    // File Transfer Settings
    char receive_directory[512] = ""; // Empty = use default (Downloads folder)

    // Get the effective receive directory (uses default if empty)
    std::string getReceiveDirectory() const;

    // Expert Settings (forced modes for advanced operators)
    // 0xFF = AUTO (let protocol decide), any other value = forced
    uint8_t forced_waveform = 0xFF;     // WaveformMode enum
    uint8_t forced_modulation = 0xFF;   // Modulation enum
    uint8_t forced_code_rate = 0xFF;    // CodeRate enum

    // Get platform-specific Downloads folder
    static std::string getDefaultDownloadsPath();
};

class SettingsWindow {
public:
    SettingsWindow();

    // Audio device lists (set by App before rendering)
    std::vector<std::string> input_devices;
    std::vector<std::string> output_devices;

    // Show the settings window (call from main render loop)
    // Returns true if settings were changed
    bool render(AppSettings& settings);

    // Open/close the window
    void open() { visible_ = true; }
    void close() { visible_ = false; }
    bool isVisible() const { return visible_; }
    bool wasJustClosed() const { return just_closed_; }  // Check if closed this frame

    // Callback when callsign changes
    using CallsignChangedCallback = std::function<void(const std::string&)>;
    void setCallsignChangedCallback(CallsignChangedCallback cb) { on_callsign_changed_ = cb; }

    // Callback to reset/rescan audio devices
    using AudioResetCallback = std::function<void()>;
    void setAudioResetCallback(AudioResetCallback cb) { on_audio_reset_ = cb; }

    // Callback when settings window closes (to apply audio changes)
    using ClosedCallback = std::function<void()>;
    void setClosedCallback(ClosedCallback cb) { on_closed_ = cb; }

    // Callback when filter settings change
    using FilterChangedCallback = std::function<void(bool enabled, float center, float bw, int taps)>;
    void setFilterChangedCallback(FilterChangedCallback cb) { on_filter_changed_ = cb; }

    // Callback when receive directory changes
    using ReceiveDirChangedCallback = std::function<void(const std::string&)>;
    void setReceiveDirChangedCallback(ReceiveDirChangedCallback cb) { on_receive_dir_changed_ = cb; }

    // Callback when expert settings change (forced waveform/modulation/code rate)
    using ExpertSettingsChangedCallback = std::function<void(uint8_t waveform, uint8_t modulation, uint8_t code_rate)>;
    void setExpertSettingsChangedCallback(ExpertSettingsChangedCallback cb) { on_expert_settings_changed_ = cb; }

private:
    bool visible_ = false;
    bool was_visible_ = false;  // Track previous frame visibility
    bool just_closed_ = false;  // Set when window closes
    int current_tab_ = 0;

    CallsignChangedCallback on_callsign_changed_;
    AudioResetCallback on_audio_reset_;
    ClosedCallback on_closed_;
    FilterChangedCallback on_filter_changed_;
    ReceiveDirChangedCallback on_receive_dir_changed_;
    ExpertSettingsChangedCallback on_expert_settings_changed_;

    void renderStationTab(AppSettings& settings);
    void renderRadioTab(AppSettings& settings);
    void renderAudioTab(AppSettings& settings);
    void renderExpertTab(AppSettings& settings);
};

} // namespace gui
} // namespace ultra
