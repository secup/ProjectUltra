#pragma once

#include "ultra/types.hpp"
#include "widgets/constellation.hpp"
#include "widgets/controls.hpp"
#include "widgets/status.hpp"
#include "widgets/settings.hpp"
#include "widgets/waterfall.hpp"
#include "widgets/file_browser.hpp"
#include "audio_engine.hpp"
#include "modem_engine.hpp"
#include "protocol/protocol_engine.hpp"

#include <vector>
#include <complex>
#include <random>
#include <string>
#include <deque>
#include <memory>

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
    WaterfallWidget waterfall_;
    FileBrowser file_browser_;

    // Persistent settings
    AppSettings settings_;

    // Real modem engine
    AudioEngine audio_;
    ModemEngine modem_;

    // Modem state
    ModemConfig config_;
    ultra::ModemStats stats_;  // From types.hpp for status widget

    // UI state
    float snr_slider_ = 100.0f;  // SNR in dB for channel simulation (100+ = no noise)

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

    // File transfer state
    char file_path_buffer_[512] = "";
    std::string last_received_file_;  // Path of last received file
    enum class FileBrowserTarget { OPERATE, TEST_LOCAL, TEST_REMOTE } file_browser_target_ = FileBrowserTarget::OPERATE;

    // Protocol loopback test state (two virtual stations)
    // Each station has its own protocol engine AND modem engine
    protocol::ProtocolEngine test_local_;   // Local station in test mode
    protocol::ProtocolEngine test_remote_;  // Remote station in test mode
    std::unique_ptr<ModemEngine> test_modem_1_;  // Modem for TEST1 station
    std::unique_ptr<ModemEngine> test_modem_2_;  // Modem for TEST2 station

    // TX sample queues for cross-wiring (TEST1 TX → TEST2 RX, TEST2 TX → TEST1 RX)
    std::vector<float> test_tx_queue_1_;  // TEST1's pending TX samples
    std::vector<float> test_tx_queue_2_;  // TEST2's pending TX samples

    char test_local_tx_[256] = "Hello from TEST1!";
    char test_remote_tx_[256] = "Hello from TEST2!";
    char test_local_file_[512] = "";
    char test_remote_file_[512] = "";
    std::deque<std::string> test_local_log_;
    std::deque<std::string> test_remote_log_;
    std::string test_local_incoming_;   // Incoming call for local
    std::string test_remote_incoming_;  // Incoming call for remote
    bool test_protocol_mode_ = false;   // True = protocol test, False = raw modem test

    void renderLoopbackControls();
    void renderProtocolTestControls();
    void initProtocolTest();
    void tickProtocolTest(uint32_t elapsed_ms);
    void processTestChannel(std::vector<float>& samples);  // Add noise to test channel
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
