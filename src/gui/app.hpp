#pragma once

#include "ultra/types.hpp"
#include "widgets/constellation.hpp"
#include "widgets/controls.hpp"
#include "widgets/status.hpp"
#include "widgets/settings.hpp"
#include "audio_engine.hpp"
#include "modem_engine.hpp"
#include "protocol/protocol_engine.hpp"

#include <vector>
#include <complex>
#include <random>
#include <string>
#include <deque>

namespace ultra {
namespace gui {

// Operating mode
enum class AppMode {
    OPERATE,     // Real radio operation (mic input, speaker output)
    TEST         // Local testing with audio loopback
};

class App {
public:
    App();
    ~App();

    void render();

private:
    // Widgets
    ConstellationWidget constellation_;
    ControlsWidget controls_;
    StatusWidget status_;
    SettingsWindow settings_window_;

    // Persistent settings
    AppSettings settings_;

    // Real modem engine
    AudioEngine audio_;
    ModemEngine modem_;

    // Modem state
    ModemConfig config_;
    ultra::ModemStats stats_;  // From types.hpp for status widget

    // UI state
    float snr_slider_ = 20.0f;

    // Mode selection
    AppMode mode_ = AppMode::OPERATE;

    // Loopback/Radio mode state
    char tx_text_buffer_[256] = "Hello from ProjectUltra!";
    std::deque<std::string> rx_log_;
    static const size_t MAX_RX_LOG = 20;
    bool audio_initialized_ = false;
    AppMode audio_init_mode_ = AppMode::OPERATE;  // Which mode initialized audio
    bool tx_in_progress_ = false;

    // Radio mode state
    std::vector<std::string> input_devices_;
    std::vector<std::string> output_devices_;
    bool ptt_active_ = false;      // Push-to-talk state
    bool radio_rx_enabled_ = false; // RX capture running

    // ARQ Protocol state (always enabled in Radio mode)
    protocol::ProtocolEngine protocol_;
    char remote_callsign_[16] = "";
    std::string pending_incoming_call_;  // Callsign of incoming caller
    uint32_t last_tick_time_ = 0;

    void renderLoopbackControls();
    void renderRadioControls();
    void initAudio();
    void initRadioAudio();
    void sendMessage();
    void onDataReceived(const std::string& text);
    void startRadioRx();
    void stopRadioRx();

    // Helper to get device name from settings (returns empty string for "Default")
    std::string getInputDeviceName() const;
    std::string getOutputDeviceName() const;
};

} // namespace gui
} // namespace ultra
