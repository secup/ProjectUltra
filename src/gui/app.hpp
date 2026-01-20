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

class App {
public:
    // Flags for developer features
    struct Options {
        bool enable_sim = false;      // -sim: Show simulation UI
        bool record_audio = false;    // -rec: Record all audio to file
        std::string record_path = "sim_recording.f32";  // Recording output path
    };

    App();  // Default constructor
    explicit App(const Options& opts);
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

    // Operate mode state
    char tx_text_buffer_[256] = "Hello from ProjectUltra!";
    std::deque<std::string> rx_log_;
    static const size_t MAX_RX_LOG = 20;
    bool audio_initialized_ = false;
    bool tx_in_progress_ = false;

    // Radio mode state
    std::vector<std::string> input_devices_;
    std::vector<std::string> output_devices_;
    bool ptt_active_ = false;      // Push-to-talk state
    bool radio_rx_enabled_ = false; // RX capture running

    // ARQ Protocol state
    protocol::ProtocolEngine protocol_;
    char remote_callsign_[16] = "";
    std::string pending_incoming_call_;  // Callsign of incoming caller
    uint32_t last_tick_time_ = 0;

    // File transfer state
    char file_path_buffer_[512] = "";
    std::string last_received_file_;  // Path of last received file
    bool file_browser_open_ = false;

    // ========================================
    // Developer Options
    // ========================================
    Options options_;                           // Command-line options

    // ========================================
    // Virtual Station Simulator (requires -sim flag)
    // ========================================
    // When enabled, a virtual station (callsign "SIM") responds to your
    // transmissions through full modem simulation (LDPC, OFDM, channel effects).
    // Connect to "SIM" to test the complete protocol flow in the real UI.

    bool sim_ui_visible_ = false;               // Show simulation UI (-sim flag)
    bool simulation_enabled_ = false;           // Enable virtual station
    float simulation_snr_db_ = 20.0f;           // Simulated channel SNR

    // Audio recording (requires -rec flag)
    bool recording_enabled_ = false;            // Currently recording
    std::vector<float> recorded_samples_;       // Accumulated samples
    void writeRecordingToFile();                // Save recording to disk
    std::string virtual_callsign_ = "SIM";      // Virtual station's callsign

    // Virtual station's protocol and modem
    protocol::ProtocolEngine virtual_protocol_;
    std::unique_ptr<ModemEngine> virtual_modem_;

    // Cross-wired sample queues (full modem simulation)
    // Our TX samples → channel sim → virtual's RX
    // Virtual's TX samples → channel sim → our RX
    std::vector<float> sim_our_tx_queue_;       // Our pending TX → virtual RX
    std::vector<float> sim_virtual_tx_queue_;   // Virtual's pending TX → our RX

    // Channel simulation RNG
    std::mt19937 sim_rng_{42};

    // Simulated propagation delays (ms remaining)
    uint32_t sim_our_tx_delay_ = 0;
    uint32_t sim_virtual_tx_delay_ = 0;

    // Virtual station initialization
    void initVirtualStation();

    // Process simulation (called from render loop)
    void tickSimulation(uint32_t elapsed_ms);

    // Add channel effects (AWGN, optional fading) to samples
    void applyChannelSimulation(std::vector<float>& samples);

    // Prepend realistic PTT delay noise (100-500ms) before signal
    void prependPttNoise(std::vector<float>& samples);

    // ========================================
    // UI Rendering
    // ========================================
    void renderOperateTab();
    void initAudio();
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
