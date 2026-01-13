#include "app.hpp"
#include "imgui.h"
#include "ultra/logging.hpp"
#include <SDL.h>
#include <cstring>
#include <cmath>

namespace ultra {
namespace gui {

App::App() {
    // Load persistent settings
    settings_.load();

    config_ = presets::balanced();

    // Initialize protocol with saved callsign
    if (strlen(settings_.callsign) > 0) {
        protocol_.setLocalCallsign(settings_.callsign);
    }

    // Set up modem callback for received data (Loopback mode - raw text)
    modem_.setDataCallback([this](const std::string& text) {
        // In Loopback mode, show raw text directly
        // In Radio mode, data goes through protocol layer
        if (mode_ != AppMode::OPERATE) {
            onDataReceived(text);
        }
    });

    // Set up raw data callback for ARQ mode (Radio mode only)
    modem_.setRawDataCallback([this](const Bytes& data) {
        if (mode_ == AppMode::OPERATE) {
            protocol_.onRxData(data);
        }
    });

    // Set up protocol engine callbacks
    protocol_.setTxDataCallback([this](const Bytes& data) {
        // When protocol layer wants to transmit, convert to audio
        auto samples = modem_.transmit(data);
        if (!samples.empty()) {
            audio_.queueTxSamples(samples);
        }
    });

    protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        // Received a message via ARQ
        std::string msg = "[RX " + from + "] " + text;
        rx_log_.push_front(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_back();
        }
    });

    protocol_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        std::string msg;
        switch (state) {
            case protocol::ConnectionState::CONNECTING:
                msg = "[SYS] Connecting to " + info + "...";
                break;
            case protocol::ConnectionState::CONNECTED:
                msg = "[SYS] Connected to " + protocol_.getRemoteCallsign();
                break;
            case protocol::ConnectionState::DISCONNECTING:
                msg = "[SYS] Disconnecting...";
                break;
            case protocol::ConnectionState::DISCONNECTED:
                msg = "[SYS] Disconnected" + (info.empty() ? "" : ": " + info);
                break;
        }
        rx_log_.push_front(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_back();
        }
    });

    protocol_.setIncomingCallCallback([this](const std::string& from) {
        pending_incoming_call_ = from;
        std::string msg = "[SYS] Incoming call from " + from;
        rx_log_.push_front(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_back();
        }
    });

    // Settings window callback - save settings when callsign changes
    settings_window_.setCallsignChangedCallback([this](const std::string& call) {
        protocol_.setLocalCallsign(call);
        settings_.save();  // Persist to disk
    });

    // Settings window callback - rescan audio devices
    settings_window_.setAudioResetCallback([this]() {
        // Stop any active audio
        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();
        audio_.shutdown();
        audio_initialized_ = false;

        // Reinitialize to get new device list
        initRadioAudio();

        // Reinitialize for current mode
        if (mode_ == AppMode::OPERATE) {
            if (audio_initialized_) {
                std::string output_dev = getOutputDeviceName();
                audio_.openOutput(output_dev);
                audio_.startPlayback();
                startRadioRx();
            }
        } else if (mode_ == AppMode::TEST) {
            initAudio();
        }
    });

    // Settings window callback - when settings closes, reinit audio with new device selection
    settings_window_.setClosedCallback([this]() {
        // Save settings (including device selection)
        settings_.save();

        // Reinitialize audio with possibly new device selection
        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();

        // Reopen with new settings
        if (mode_ == AppMode::OPERATE && audio_initialized_) {
            std::string output_dev = getOutputDeviceName();
            audio_.openOutput(output_dev);
            audio_.startPlayback();
            startRadioRx();
        } else if (mode_ == AppMode::TEST && audio_initialized_) {
            if (audio_.openOutput(getOutputDeviceName())) {
                audio_.setLoopbackEnabled(true);
                audio_.setLoopbackSNR(snr_slider_);
                audio_.startPlayback();
            }
        }
    });
}

App::~App() {
    settings_.save();  // Save settings on exit
    audio_.shutdown();
}

void App::initAudio() {
    if (audio_initialized_ && audio_init_mode_ == AppMode::TEST) return;

    // If audio was initialized for a different mode, shut it down first
    if (audio_initialized_) {
        audio_.shutdown();
        audio_initialized_ = false;
    }

    if (audio_.initialize()) {
        if (audio_.openOutput()) {
            audio_.setLoopbackEnabled(true);
            audio_.setLoopbackSNR(snr_slider_);
            audio_.startPlayback();
            audio_initialized_ = true;
            audio_init_mode_ = AppMode::TEST;

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
    ImGui::Text("Test Mode");
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

void App::render() {
    // Poll for audio samples and process them (must be called from main loop, not audio callback)
    // This processes any samples that were queued by the audio callback thread
    if (mode_ == AppMode::TEST || mode_ == AppMode::OPERATE) {
        modem_.pollRxAudio();
    }

    // Protocol engine tick (for ARQ timeouts) - always in Radio mode
    if (mode_ == AppMode::OPERATE) {
        uint32_t now = SDL_GetTicks();
        uint32_t elapsed = (last_tick_time_ == 0) ? 0 : (now - last_tick_time_);
        last_tick_time_ = now;
        if (elapsed > 0 && elapsed < 1000) {  // Sanity check
            protocol_.tick(elapsed);
        }
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

    // Settings button
    ImGui::SameLine(ImGui::GetWindowWidth() - 420);
    if (ImGui::SmallButton("Settings")) {
        settings_window_.open();
    }

    // Mode selector tabs
    ImGui::SameLine(ImGui::GetWindowWidth() - 350);
    if (ImGui::BeginTabBar("ModeSelector")) {
        AppMode prev_mode = mode_;

        if (ImGui::BeginTabItem("Operate")) {
            mode_ = AppMode::OPERATE;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Test")) {
            mode_ = AppMode::TEST;
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();

        // Handle mode transitions - reset audio when switching between modes
        if (mode_ != prev_mode) {
            // Shut down audio from previous mode
            if (prev_mode == AppMode::OPERATE && radio_rx_enabled_) {
                stopRadioRx();
                audio_.stopPlayback();
                audio_.closeOutput();
            }
            if (prev_mode == AppMode::TEST && audio_initialized_) {
                audio_.stopPlayback();
                audio_.closeOutput();
            }

            // Reset audio state when switching between Loopback and Radio
            // (they use different audio configurations)
            if ((prev_mode == AppMode::TEST && mode_ == AppMode::OPERATE) ||
                (prev_mode == AppMode::OPERATE && mode_ == AppMode::TEST)) {
                audio_.shutdown();
                audio_initialized_ = false;
                // Reset modem to clear stale demodulator state (buffers, sync state)
                modem_.reset();
            }

            // Auto-start listening when entering Radio mode
            if (mode_ == AppMode::OPERATE) {
                if (!audio_initialized_) {
                    initRadioAudio();
                }
                if (audio_initialized_ && !radio_rx_enabled_) {
                    audio_.openOutput(getOutputDeviceName());
                    audio_.startPlayback();
                    startRadioRx();
                }
            }
        }
    }

    ImGui::Separator();

    // Main content area with three columns
    float content_height = ImGui::GetContentRegionAvail().y - 30;

    ImGui::BeginChild("ContentArea", ImVec2(0, content_height), false);

    // Left panel: Constellation
    float total_width = ImGui::GetContentRegionAvail().x;
    float constellation_width = total_width * 0.25f;  // Smaller constellation
    float controls_width = total_width * 0.30f;       // Channel status

    ImGui::BeginChild("LeftPanel", ImVec2(constellation_width, 0), true);

    // Use received symbols from the modem
    auto symbols = modem_.getConstellationSymbols();
    constellation_.render(symbols, config_.modulation);

    ImGui::EndChild();
    ImGui::SameLine();

    // Middle panel: Channel Status
    ImGui::BeginChild("MiddlePanel", ImVec2(controls_width, 0), true);

    auto modem_stats = modem_.getStats();
    auto event = controls_.render(modem_stats, config_);

    if (event == ControlsWidget::Event::ProfileChanged) {
        config_ = presets::forProfile(config_.speed_profile);
        modem_.setConfig(config_);
    }

    ImGui::EndChild();
    ImGui::SameLine();

    // Right panel: Mode-specific controls
    ImGui::BeginChild("RightPanel", ImVec2(0, 0), true);

    if (mode_ == AppMode::TEST) {
        renderLoopbackControls();
    } else {
        renderRadioControls();
    }

    ImGui::EndChild();

    ImGui::EndChild();

    // Status bar at bottom
    ImGui::Separator();
    auto mstats = modem_.getStats();
    if (mode_ == AppMode::TEST) {
        ImGui::Text("Mode: TEST | SNR: %.1f dB | TX: %d frames | Throughput: %d bps",
            mstats.snr_db, mstats.frames_sent, mstats.throughput_bps);
    } else {
        const char* state = ptt_active_ ? "TX" : (radio_rx_enabled_ ? "RX" : "IDLE");
        ImGui::Text("Mode: OPERATE [%s] | SNR: %.1f dB | TX: %d | RX: %d | Throughput: %d bps",
            state, mstats.snr_db, mstats.frames_sent, mstats.frames_received, mstats.throughput_bps);
    }

    ImGui::End();

    // Render settings window (if open)
    // Ensure device lists are populated when settings is visible
    if (settings_window_.isVisible() && settings_window_.input_devices.empty()) {
        // Need to enumerate devices - initialize audio subsystem if needed
        if (!audio_.isInitialized()) {
            audio_.initialize();
        }
        settings_window_.input_devices = audio_.getInputDevices();
        settings_window_.output_devices = audio_.getOutputDevices();
    }
    settings_window_.render(settings_);
}

void App::initRadioAudio() {
    if (audio_initialized_ && audio_init_mode_ == AppMode::OPERATE) return;

    // If audio was initialized for a different mode, shut it down first
    if (audio_initialized_) {
        audio_.shutdown();
        audio_initialized_ = false;
    }

    if (!audio_.initialize()) {
        return;
    }

    // Enumerate devices (already includes "Default" as first option)
    input_devices_ = audio_.getInputDevices();
    output_devices_ = audio_.getOutputDevices();

    // Populate settings window device lists
    settings_window_.input_devices = input_devices_;
    settings_window_.output_devices = output_devices_;

    audio_initialized_ = true;
    audio_init_mode_ = AppMode::OPERATE;
}

std::string App::getInputDeviceName() const {
    // "Default" or empty means use system default
    if (strcmp(settings_.input_device, "Default") == 0 || settings_.input_device[0] == '\0') {
        return "";
    }
    return settings_.input_device;
}

std::string App::getOutputDeviceName() const {
    // "Default" or empty means use system default
    if (strcmp(settings_.output_device, "Default") == 0 || settings_.output_device[0] == '\0') {
        return "";
    }
    return settings_.output_device;
}

void App::startRadioRx() {
    if (!audio_initialized_) return;

    // Open input device using settings
    std::string input_dev = getInputDeviceName();
    if (!audio_.openInput(input_dev)) {
        return;
    }

    // Set up RX callback - feed captured audio to modem
    audio_.setRxCallback([this](const std::vector<float>& samples) {
        modem_.receiveAudio(samples);
    });

    // Disable loopback for real radio mode
    audio_.setLoopbackEnabled(false);

    // Start capturing
    audio_.startCapture();
    radio_rx_enabled_ = true;
}

void App::stopRadioRx() {
    audio_.stopCapture();
    audio_.closeInput();
    radio_rx_enabled_ = false;
}

void App::renderRadioControls() {
    ImGui::Text("Operate Mode");
    ImGui::TextDisabled("(Real audio - speaker to mic)");
    ImGui::Separator();
    ImGui::Spacing();

    // Initialize audio on first use
    if (!audio_initialized_) {
        if (ImGui::Button("Initialize Audio", ImVec2(-1, 35))) {
            initRadioAudio();
        }
        ImGui::TextDisabled("Click to scan audio devices");
        return;
    }

    // Show current audio device settings (read-only display)
    ImGui::Text("Audio Devices");
    ImGui::TextDisabled("Output: %s", settings_.output_device);
    ImGui::TextDisabled("Input: %s", settings_.input_device);
    if (ImGui::SmallButton("Configure Audio...")) {
        settings_window_.open();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // ARQ Connection Controls
    ImGui::Text("ARQ Connection");

    // Show local callsign (from settings)
    bool has_callsign = strlen(settings_.callsign) >= 3;
    if (has_callsign) {
        ImGui::TextDisabled("My Call: %s", settings_.callsign);
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Set callsign in Settings!");
        if (ImGui::SmallButton("Open Settings")) {
            settings_window_.open();
        }
    }

    // Remote callsign input
    ImGui::Text("Connect to:");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##remotecall", remote_callsign_, sizeof(remote_callsign_),
                     ImGuiInputTextFlags_CharsUppercase);

    ImGui::Spacing();

    // Connection status and controls
    auto conn_state = protocol_.getState();
    const char* state_str = protocol::connectionStateToString(conn_state);

    // Status indicator
    ImVec4 state_color;
    switch (conn_state) {
        case protocol::ConnectionState::CONNECTED:
            state_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
            break;
        case protocol::ConnectionState::CONNECTING:
        case protocol::ConnectionState::DISCONNECTING:
            state_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f);
            break;
        default:
            state_color = ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
            break;
    }
    ImGui::TextColored(state_color, "Status: %s", state_str);
    if (conn_state == protocol::ConnectionState::CONNECTED) {
        ImGui::SameLine();
        ImGui::Text("to %s", protocol_.getRemoteCallsign().c_str());
    }

    // Incoming call notification
    if (!pending_incoming_call_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                           "Incoming call from %s!", pending_incoming_call_.c_str());
        if (ImGui::Button("Accept", ImVec2(80, 0))) {
            protocol_.acceptCall();
            pending_incoming_call_.clear();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reject", ImVec2(80, 0))) {
            protocol_.rejectCall();
            pending_incoming_call_.clear();
        }
    }

    // Connect/Disconnect buttons
    float btn_width = ImGui::GetContentRegionAvail().x / 2 - 5;

    ImGui::BeginDisabled(conn_state != protocol::ConnectionState::DISCONNECTED ||
                         !has_callsign || strlen(remote_callsign_) < 3);
    if (ImGui::Button("Connect", ImVec2(btn_width, 30))) {
        protocol_.connect(remote_callsign_);
    }
    ImGui::EndDisabled();

    ImGui::SameLine();

    ImGui::BeginDisabled(conn_state == protocol::ConnectionState::DISCONNECTED);
    if (ImGui::Button("Disconnect", ImVec2(btn_width, 30))) {
        protocol_.disconnect();
    }
    ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Station status
    if (!radio_rx_enabled_) {
        // Not listening - show start button
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "OFFLINE");
        if (ImGui::Button("Start Listening", ImVec2(-1, 30))) {
            if (!audio_initialized_) {
                initRadioAudio();
            }
            audio_.openOutput(getOutputDeviceName());
            audio_.startPlayback();
            startRadioRx();
        }
    } else {
        // Show listening status prominently
        if (protocol_.isConnected()) {
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "CONNECTED to %s",
                               protocol_.getRemoteCallsign().c_str());
        } else if (ptt_active_) {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), ">>> TRANSMITTING <<<");
        } else {
            ImGui::TextColored(ImVec4(0.3f, 0.8f, 1.0f, 1.0f), "LISTENING...");
        }

        // Small stop button (not prominent since auto-start is the norm)
        if (ImGui::SmallButton("Stop")) {
            stopRadioRx();
            audio_.stopPlayback();
            audio_.closeOutput();
        }

        // Show audio level meters when active
        ImGui::Spacing();
        float input_level = audio_.getInputLevel();
        float input_db = (input_level > 0.0001f) ? 20.0f * log10f(input_level) : -80.0f;

        // Scale to 0-1 range for progress bar (-60dB to 0dB)
        float level_normalized = (input_db + 60.0f) / 60.0f;
        level_normalized = std::max(0.0f, std::min(1.0f, level_normalized));

        // Color based on level
        ImVec4 level_color = (level_normalized > 0.8f) ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) :
                             (level_normalized > 0.5f) ? ImVec4(1.0f, 1.0f, 0.3f, 1.0f) :
                                                         ImVec4(0.3f, 1.0f, 0.3f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, level_color);
        ImGui::ProgressBar(level_normalized, ImVec2(-1, 16), "");
        ImGui::PopStyleColor();
        ImGui::SameLine(10);
        ImGui::Text("RX: %.0f dB", input_db);

        // Sync indicator
        bool synced = modem_.isSynced();
        if (synced) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), ">> SIGNAL DETECTED <<");
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // TX input (only when RX is running)
    ImGui::BeginDisabled(!radio_rx_enabled_);

    ImGui::Text("Transmit Message");
    ImGui::SetNextItemWidth(-1);
    bool send = ImGui::InputText("##txinput_radio", tx_text_buffer_, sizeof(tx_text_buffer_),
                                  ImGuiInputTextFlags_EnterReturnsTrue);

    // Check if TX is done
    if (tx_in_progress_ && audio_.isTxQueueEmpty()) {
        tx_in_progress_ = false;
        // Resume RX after TX completes
        if (!ptt_active_) {
            audio_.startCapture();
        }
    }

    // PTT Button - Send message
    // Large PTT button
    ImVec4 ptt_color = ptt_active_ ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) : ImVec4(0.3f, 0.6f, 0.3f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, ptt_color);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ptt_color);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.0f, 0.5f, 0.0f, 1.0f));

    // Button label depends on connection state (Radio mode always uses ARQ)
    const char* btn_label = ptt_active_ ? ">>> TRANSMITTING <<<" : "SEND";
    if (protocol_.isConnected()) {
        btn_label = protocol_.isReadyToSend() ? "SEND (ARQ)" : "Waiting for ACK...";
    } else {
        btn_label = "Connect first";
    }

    // In Radio mode, must be connected to send
    bool can_send = !tx_in_progress_ && strlen(tx_text_buffer_) > 0 &&
                    protocol_.isConnected() && protocol_.isReadyToSend();

    ImGui::BeginDisabled(!can_send);
    if (ImGui::Button(btn_label, ImVec2(-1, 40)) || (send && can_send)) {
        std::string text(tx_text_buffer_);

        // Send via ARQ protocol
        if (protocol_.sendMessage(text)) {
            rx_log_.push_front("[TX] " + text);
            if (rx_log_.size() > MAX_RX_LOG) {
                rx_log_.pop_back();
            }
            tx_text_buffer_[0] = '\0';
        }
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor(3);

    if (tx_in_progress_) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Transmitting...");
    }

    ImGui::EndDisabled();  // End TX input disabled state

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // RX log
    ImGui::Text("Message Log");
    ImGui::BeginChild("RXLogRadio", ImVec2(-1, 120), true);
    for (const auto& msg : rx_log_) {
        if (msg.substr(0, 4) == "[TX]") {
            ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "%s", msg.c_str());
        } else {
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "%s", msg.c_str());
        }
    }
    ImGui::EndChild();

    ImGui::Spacing();

    // Stats
    auto mstats = modem_.getStats();
    ImGui::Text("TX: %d frames | RX: %d frames", mstats.frames_sent, mstats.frames_received);

    // Tip
    ImGui::Spacing();
    ImGui::TextDisabled("Tip: Use Conservative profile for acoustic coupling");
}

} // namespace gui
} // namespace ultra
