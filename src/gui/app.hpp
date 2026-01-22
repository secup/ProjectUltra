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
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

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
    std::atomic<bool> tx_in_progress_{false};  // Thread-safe TX flag for waterfall control
    std::chrono::steady_clock::time_point tx_end_time_;  // When current TX finishes

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

    // ========================================
    // Threaded Audio Channels (like real radios)
    // ========================================
    // ALPHA TX -> channel_to_virtual_ -> BRAVO RX
    // BRAVO TX -> channel_to_us_ -> ALPHA RX

    // Thread-safe audio buffers
    std::mutex channel_to_virtual_mutex_;
    std::vector<float> channel_to_virtual_;         // Our TX samples for virtual
    std::mutex channel_to_us_mutex_;
    std::vector<float> channel_to_us_;              // Virtual's TX samples for us

    // TX streaming buffers (for real-time waterfall display)
    std::mutex our_tx_pending_mutex_;
    std::vector<float> our_tx_pending_;             // Our TX samples waiting to be streamed
    std::mutex virtual_tx_pending_mutex_;
    std::vector<float> virtual_tx_pending_;         // Virtual's TX samples waiting to be streamed

    // Channel simulation RNG
    std::mt19937 sim_rng_{42};

    // Threads for continuous audio processing
    std::thread sim_our_rx_thread_;                 // Our RX thread (reads from channel_to_us_)
    std::thread sim_our_tx_thread_;                 // Our TX thread (streams our_tx_pending_ to channel)
    std::thread sim_virtual_rx_thread_;             // Virtual's RX thread (reads from channel_to_virtual_)
    std::thread sim_virtual_tx_thread_;             // Virtual's TX thread (streams virtual_tx_pending_)
    std::thread sim_virtual_protocol_thread_;       // Virtual's protocol tick thread
    std::atomic<bool> sim_thread_running_{false};   // Thread control flag

    // Virtual station initialization
    void initVirtualStation();

    // Start/stop simulator threads
    void startSimThreads();
    void stopSimThreads();

    // Thread loops
    void ourRxLoop();                               // Our RX thread (reads channel_to_us_)
    void ourTxLoop();                               // Our TX thread (streams to channel at 48kHz)
    void virtualRxLoop();                           // Virtual's RX thread (reads channel_to_virtual_)
    void virtualTxLoop();                           // Virtual's TX thread (streams to channel at 48kHz)
    void virtualProtocolLoop();                     // Virtual's protocol tick

    // Channel simulation helpers
    void writeToVirtualChannel(const std::vector<float>& samples);  // Our TX -> virtual
    void writeToOurChannel(const std::vector<float>& samples);      // Virtual TX -> us
    std::vector<float> applyChannelEffects(const std::vector<float>& samples);  // AWGN + PTT noise

    // ========================================
    // UI Rendering
    // ========================================
    void renderOperateTab();
    void renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate);
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
