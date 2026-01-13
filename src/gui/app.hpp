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
    LOOPBACK     // Real audio loopback (hear the modem!)
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
    bool connected_ = false;
    bool simulation_running_ = false;

    // Last simulation result
    SimulationResult last_result_;

    // UI state
    int channel_preset_ = 2;  // MODERATE
    float snr_slider_ = 20.0f;

    // Mode selection
    AppMode mode_ = AppMode::SIMULATION;

    // Loopback mode state
    char tx_text_buffer_[256] = "Hello from ProjectUltra!";
    std::deque<std::string> rx_log_;
    static const size_t MAX_RX_LOG = 20;
    bool audio_initialized_ = false;
    bool tx_in_progress_ = false;

    void renderSimulationControls();
    void renderLoopbackControls();
    void initAudio();
    void sendMessage();
    void onDataReceived(const std::string& text);
};

} // namespace gui
} // namespace ultra
