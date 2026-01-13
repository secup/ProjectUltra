#include "app.hpp"
#include "imgui.h"

namespace ultra {
namespace gui {

App::App() {
    config_ = presets::balanced();
    simulation_.setChannelPreset(ChannelPreset::MODERATE);
    snr_slider_ = simulation_.getSNR();

    // Set up modem callback for received data
    modem_.setDataCallback([this](const std::string& text) {
        onDataReceived(text);
    });
}

App::~App() {
    audio_.shutdown();
}

void App::initAudio() {
    if (audio_initialized_) return;

    if (audio_.initialize()) {
        if (audio_.openOutput()) {
            audio_.setLoopbackEnabled(true);
            audio_.setLoopbackSNR(snr_slider_);
            audio_.startPlayback();
            audio_initialized_ = true;

            // Set up RX callback
            audio_.setRxCallback([this](const std::vector<float>& samples) {
                modem_.receiveAudio(samples);
            });
        }
    }
}

void App::sendMessage() {
    if (tx_in_progress_) return;

    std::string text(tx_text_buffer_);
    if (text.empty()) return;

    // Generate modem audio
    auto samples = modem_.transmit(text);

    if (!samples.empty()) {
        // Queue for playback (and loopback)
        audio_.queueTxSamples(samples);
        tx_in_progress_ = true;

        // Log what we sent
        rx_log_.push_front("[TX] " + text);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_back();
        }
    }
}

void App::onDataReceived(const std::string& text) {
    if (!text.empty()) {
        rx_log_.push_front("[RX] " + text);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_back();
        }
    }
}

void App::renderLoopbackControls() {
    ImGui::Text("Loopback Mode");
    ImGui::TextDisabled("(Real audio - you'll hear the modem!)");
    ImGui::Separator();
    ImGui::Spacing();

    // Initialize audio on first use
    if (!audio_initialized_) {
        if (ImGui::Button("Initialize Audio", ImVec2(-1, 35))) {
            initAudio();
        }
        ImGui::TextDisabled("Click to start audio engine");
        return;
    }

    // SNR slider for loopback channel
    ImGui::Text("Loopback Channel");
    if (ImGui::SliderFloat("SNR (dB)", &snr_slider_, 5.0f, 40.0f, "%.1f")) {
        audio_.setLoopbackSNR(snr_slider_);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // TX input
    ImGui::Text("Transmit Message");
    ImGui::SetNextItemWidth(-1);
    bool send = ImGui::InputText("##txinput", tx_text_buffer_, sizeof(tx_text_buffer_),
                                  ImGuiInputTextFlags_EnterReturnsTrue);

    // Check if TX is done
    if (tx_in_progress_ && audio_.isTxQueueEmpty()) {
        tx_in_progress_ = false;
    }

    // Send button
    ImGui::BeginDisabled(tx_in_progress_);
    if (ImGui::Button("Send", ImVec2(-1, 30)) || send) {
        sendMessage();
    }
    ImGui::EndDisabled();

    if (tx_in_progress_) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Transmitting...");
        ImGui::ProgressBar((float)(audio_.getTxQueueSize() % 1000) / 1000.0f);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // RX log
    ImGui::Text("Message Log");
    ImGui::BeginChild("RXLog", ImVec2(-1, 150), true);
    for (const auto& msg : rx_log_) {
        if (msg.substr(0, 4) == "[TX]") {
            ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "%s", msg.c_str());
        } else {
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "%s", msg.c_str());
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Stats
    auto mstats = modem_.getStats();
    ImGui::Text("Statistics");
    ImGui::Text("Frames TX: %d", mstats.frames_sent);
    ImGui::Text("Frames RX: %d", mstats.frames_received);
    ImGui::Text("Throughput: %d bps", mstats.throughput_bps);
}

void App::renderSimulationControls() {
    ImGui::Text("Simulation Controls");
    ImGui::Separator();
    ImGui::Spacing();

    // Channel preset selection
    const char* presets[] = { "AWGN Only", "Good", "Moderate", "Poor", "Custom" };
    if (ImGui::Combo("Channel", &channel_preset_, presets, 5)) {
        simulation_.setChannelPreset(static_cast<ChannelPreset>(channel_preset_));
        snr_slider_ = simulation_.getSNR();
    }

    // SNR slider
    ImGui::Spacing();
    if (ImGui::SliderFloat("SNR (dB)", &snr_slider_, -5.0f, 35.0f, "%.1f")) {
        simulation_.setSNR(snr_slider_);
        channel_preset_ = 4;  // Custom
    }

    // Show channel description
    ImGui::TextDisabled("(%s)",
        channel_preset_ == 0 ? "Clean channel, no multipath" :
        channel_preset_ == 1 ? "Light fading, 0.5ms delay" :
        channel_preset_ == 2 ? "Typical HF, 1ms delay" :
        channel_preset_ == 3 ? "Heavy fading, 2ms delay" :
                               "Manual settings");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Simulation controls
    if (!simulation_running_) {
        if (ImGui::Button("Start Simulation", ImVec2(-1, 35))) {
            simulation_running_ = true;
            simulation_.resetStats();
        }
    } else {
        if (ImGui::Button("Stop Simulation", ImVec2(-1, 35))) {
            simulation_running_ = false;
        }
    }

    ImGui::Spacing();

    // Single frame button
    if (ImGui::Button("Run Single Frame", ImVec2(-1, 0))) {
        last_result_ = simulation_.runFrame(config_);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Statistics
    ImGui::Text("Statistics");
    ImGui::Text("Total Frames: %llu", (unsigned long long)simulation_.getTotalFrames());
    ImGui::Text("Success Rate: %.1f%%",
        simulation_.getTotalFrames() > 0 ?
        100.0f * simulation_.getSuccessFrames() / simulation_.getTotalFrames() : 0.0f);
    ImGui::Text("Average BER: %.2e", simulation_.getAverageBER());

    if (ImGui::SmallButton("Reset Stats")) {
        simulation_.resetStats();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Last frame result
    if (simulation_.getTotalFrames() > 0) {
        ImGui::Text("Last Frame:");
        ImGui::BulletText("Bit Errors: %d / %d", last_result_.bit_errors, last_result_.total_bits);
        ImGui::BulletText("LDPC: %s (%d iters)",
            last_result_.decode_success ? "OK" : "FAIL",
            last_result_.ldpc_iterations);
        ImGui::BulletText("BER: %.2e", last_result_.ber);
    }
}

void App::render() {
    // Run simulation frame if active (in simulation mode)
    static int frame_counter = 0;
    if (mode_ == AppMode::SIMULATION && simulation_running_ && ++frame_counter % 5 == 0) {
        last_result_ = simulation_.runFrame(config_);

        // Update stats display
        stats_.current_snr_db = last_result_.snr_db;
        stats_.throughput_bps = last_result_.throughput_bps;
        stats_.current_modulation = config_.modulation;
        stats_.current_code_rate = config_.code_rate;
        stats_.frames_received = simulation_.getTotalFrames();
    }

    // Create main window that fills the viewport
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);

    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoBringToFrontOnFocus;

    ImGui::Begin("MainWindow", nullptr, window_flags);

    // Title bar with mode selection
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "ProjectUltra");
    ImGui::SameLine();
    ImGui::TextDisabled("High-Speed HF Modem");

    // Mode selector tabs
    ImGui::SameLine(ImGui::GetWindowWidth() - 250);
    if (ImGui::BeginTabBar("ModeSelector")) {
        if (ImGui::BeginTabItem("Simulation")) {
            mode_ = AppMode::SIMULATION;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Loopback")) {
            mode_ = AppMode::LOOPBACK;
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::Separator();

    // Main content area with three columns
    float content_height = ImGui::GetContentRegionAvail().y - 30;

    ImGui::BeginChild("ContentArea", ImVec2(0, content_height), false);

    // Left panel: Constellation
    float total_width = ImGui::GetContentRegionAvail().x;
    float constellation_width = total_width * 0.4f;
    float controls_width = total_width * 0.3f;

    ImGui::BeginChild("LeftPanel", ImVec2(constellation_width, 0), true);

    // Use received symbols from simulation if available
    if (mode_ == AppMode::SIMULATION && !last_result_.rx_symbols.empty()) {
        constellation_.render(last_result_.rx_symbols, config_.modulation);
    } else {
        std::vector<std::complex<float>> empty;
        constellation_.render(empty, config_.modulation);
    }

    ImGui::EndChild();
    ImGui::SameLine();

    // Middle panel: Modem Controls
    ImGui::BeginChild("MiddlePanel", ImVec2(controls_width, 0), true);

    auto event = controls_.render(config_, connected_);

    switch (event) {
        case ControlsWidget::Event::Connect:
            connected_ = true;
            break;
        case ControlsWidget::Event::Disconnect:
            connected_ = false;
            simulation_running_ = false;
            break;
        case ControlsWidget::Event::ProfileChanged:
            config_ = presets::forProfile(config_.speed_profile);
            modem_.setConfig(config_);
            break;
        default:
            break;
    }

    ImGui::EndChild();
    ImGui::SameLine();

    // Right panel: Mode-specific controls
    ImGui::BeginChild("RightPanel", ImVec2(0, 0), true);

    if (mode_ == AppMode::SIMULATION) {
        renderSimulationControls();
    } else {
        renderLoopbackControls();
    }

    ImGui::EndChild();

    ImGui::EndChild();

    // Status bar at bottom
    ImGui::Separator();
    if (mode_ == AppMode::SIMULATION) {
        status_.render(stats_, config_, simulation_running_);
    } else {
        // Show loopback status
        auto mstats = modem_.getStats();
        ImGui::Text("Mode: LOOPBACK | SNR: %.1f dB | TX: %d frames | Throughput: %d bps",
            snr_slider_, mstats.frames_sent, mstats.throughput_bps);
    }

    ImGui::End();
}

} // namespace gui
} // namespace ultra
