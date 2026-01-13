#pragma once

#include "ultra/types.hpp"
#include "widgets/constellation.hpp"
#include "widgets/controls.hpp"
#include "widgets/status.hpp"
#include "simulation.hpp"
#include "audio_engine.hpp"
#include "modem_engine.hpp"

#include <vector>
#include <complex>
#include <random>
#include <string>
#include <deque>

namespace ultra {
namespace gui {

// Operating mode
enum class AppMode {
    SIMULATION,  // Original simulation (no real audio)
    LOOPBACK,    // Real audio loopback (hear the modem!)
    RADIO        // Real radio mode (mic input, speaker output)
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

    // Simulation engine (for SIMULATION mode)
    SimulationEngine simulation_;

    // Real modem engine (for LOOPBACK mode)
    AudioEngine audio_;
    ModemEngine modem_;

    // Modem state
    ModemConfig config_;
    ultra::ModemStats stats_;  // From types.hpp for status widget
    bool simulation_running_ = false;

    // Last simulation result
    SimulationResult last_result_;

    // UI state
    int channel_preset_ = 2;  // MODERATE
    float snr_slider_ = 20.0f;

    // Mode selection
    AppMode mode_ = AppMode::SIMULATION;

    // Loopback/Radio mode state
    char tx_text_buffer_[256] = "Hello from ProjectUltra!";
    std::deque<std::string> rx_log_;
    static const size_t MAX_RX_LOG = 20;
    bool audio_initialized_ = false;
    bool tx_in_progress_ = false;

    // Radio mode state
    std::vector<std::string> input_devices_;
    std::vector<std::string> output_devices_;
    int selected_input_ = 0;
    int selected_output_ = 0;
    bool ptt_active_ = false;      // Push-to-talk state
    bool radio_rx_enabled_ = false; // RX capture running

    void renderSimulationControls();
    void renderLoopbackControls();
    void renderRadioControls();
    void initAudio();
    void initRadioAudio();
    void sendMessage();
    void onDataReceived(const std::string& text);
    void startRadioRx();
    void stopRadioRx();
};

} // namespace gui
} // namespace ultra
