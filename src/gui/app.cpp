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
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
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
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    protocol_.setIncomingCallCallback([this](const std::string& from) {
        pending_incoming_call_ = from;
        std::string msg = "[SYS] Incoming call from " + from;
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    // File transfer callbacks
    protocol_.setFileProgressCallback([this](const protocol::FileTransferProgress& p) {
        // Progress is displayed in renderRadioControls()
    });

    protocol_.setFileReceivedCallback([this](const std::string& path, bool success) {
        std::string msg;
        if (success) {
            msg = "[FILE] Received: " + path;
            last_received_file_ = path;
        } else {
            msg = "[FILE] Receive failed";
        }
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    protocol_.setFileSentCallback([this](bool success, const std::string& error) {
        std::string msg;
        if (success) {
            msg = "[FILE] Transfer complete";
        } else {
            msg = "[FILE] Transfer failed: " + error;
        }
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    // Set receive directory from settings (defaults to Downloads folder)
    protocol_.setReceiveDirectory(settings_.getReceiveDirectory());

    // Configure waterfall display
    waterfall_.setSampleRate(48000.0f);
    waterfall_.setFrequencyRange(0.0f, 3000.0f);
    waterfall_.setDynamicRange(-60.0f, 0.0f);  // Typical audio range

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

    // Settings window callback - when filter settings change
    settings_window_.setFilterChangedCallback([this](bool enabled, float center, float bw, int taps) {
        FilterConfig filter_config;
        filter_config.enabled = enabled;
        filter_config.center_freq = center;
        filter_config.bandwidth = bw;
        filter_config.taps = taps;
        modem_.setFilterConfig(filter_config);
        settings_.save();
    });

    // Settings window callback - update receive directory
    settings_window_.setReceiveDirChangedCallback([this](const std::string& dir) {
        protocol_.setReceiveDirectory(dir);
        settings_.save();
    });

    // Apply initial filter settings from loaded config
    FilterConfig initial_filter;
    initial_filter.enabled = settings_.filter_enabled;
    initial_filter.center_freq = settings_.filter_center;
    initial_filter.bandwidth = settings_.filter_bandwidth;
    initial_filter.taps = settings_.filter_taps;
    modem_.setFilterConfig(initial_filter);
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
                waterfall_.addSamples(samples.data(), samples.size());
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
        rx_log_.push_back("[TX] " + text);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    }
}

void App::onDataReceived(const std::string& text) {
    if (!text.empty()) {
        rx_log_.push_back("[RX] " + text);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    }
}

void App::renderLoopbackControls() {
    ImGui::Text("Test Mode");
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

    // Mode toggle
    if (ImGui::Checkbox("Protocol Test (ARQ)", &test_protocol_mode_)) {
        if (test_protocol_mode_) {
            initProtocolTest();
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled(test_protocol_mode_ ? "Two stations" : "Raw modem only");

    ImGui::Spacing();

    // SNR slider for loopback channel
    ImGui::Text("Channel SNR");
    if (ImGui::SliderFloat("##snr", &snr_slider_, 5.0f, 40.0f, "%.1f dB")) {
        audio_.setLoopbackSNR(snr_slider_);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    if (test_protocol_mode_) {
        renderProtocolTestControls();
    } else {
        // Original raw modem test
        ImGui::TextDisabled("Raw modem test (no ARQ protocol)");
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
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // RX log
        ImGui::Text("Message Log");
        ImGui::BeginChild("RXLog", ImVec2(-1, 150), true);
        for (const auto& msg : rx_log_) {
            ImVec4 color = (msg.size() >= 4 && msg.substr(0, 4) == "[TX]")
                ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f)
                : ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", msg.c_str());
            ImGui::PopStyleColor();
        }
        if (!rx_log_.empty()) ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Stats
    auto mstats = modem_.getStats();
    ImGui::Text("Modem Stats: TX %d | RX %d | %d bps",
        mstats.frames_sent, mstats.frames_received, mstats.throughput_bps);
}

void App::initProtocolTest() {
    // Reset both protocol engines
    test_local_.reset();
    test_remote_.reset();

    // Create two separate modem instances - each station gets its own modem
    // This properly simulates two radios communicating
    test_modem_1_ = std::make_unique<ModemEngine>();
    test_modem_2_ = std::make_unique<ModemEngine>();

    // Clear TX queues
    test_tx_queue_1_.clear();
    test_tx_queue_2_.clear();

    // Setup local station (TEST1)
    test_local_.setLocalCallsign("TEST1");
    test_local_.setAutoAccept(true);  // Auto-accept in test mode for convenience

    // Setup remote station (TEST2)
    test_remote_.setLocalCallsign("TEST2");
    test_remote_.setAutoAccept(true);  // Auto-accept in test mode for convenience

    // Clear logs
    test_local_log_.clear();
    test_remote_log_.clear();
    test_local_incoming_.clear();
    test_remote_incoming_.clear();

    // === Cross-wired TX/RX Architecture ===
    // TEST1 TX → Channel → TEST2 RX (via test_modem_2_)
    // TEST2 TX → Channel → TEST1 RX (via test_modem_1_)
    // Note: No carrier sense in test mode - each station has dedicated RX path,
    // there's no shared medium where collisions could occur.

    // TEST1's protocol TX → TEST1's modem TX → queue for TEST2's RX
    test_local_.setTxDataCallback([this](const Bytes& data) {
        auto samples = test_modem_1_->transmit(data);
        // Queue samples for delivery to TEST2's demodulator
        test_tx_queue_1_.insert(test_tx_queue_1_.end(), samples.begin(), samples.end());
    });

    // TEST2's protocol TX → TEST2's modem TX → queue for TEST1's RX
    test_remote_.setTxDataCallback([this](const Bytes& data) {
        auto samples = test_modem_2_->transmit(data);
        // Queue samples for delivery to TEST1's demodulator
        test_tx_queue_2_.insert(test_tx_queue_2_.end(), samples.begin(), samples.end());
    });

    // TEST1's modem RX → TEST1's protocol RX
    test_modem_1_->setRawDataCallback([this](const Bytes& data) {
        test_local_.onRxData(data);
    });

    // TEST2's modem RX → TEST2's protocol RX
    test_modem_2_->setRawDataCallback([this](const Bytes& data) {
        test_remote_.onRxData(data);
    });

    // Connect protocol RX callbacks (for message display)
    test_local_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        test_local_log_.push_back("[RX from " + from + "] " + text);
        if (test_local_log_.size() > MAX_RX_LOG) test_local_log_.pop_front();
    });

    test_remote_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        test_remote_log_.push_back("[RX from " + from + "] " + text);
        if (test_remote_log_.size() > MAX_RX_LOG) test_remote_log_.pop_front();
    });

    // Connect incoming call callbacks
    test_local_.setIncomingCallCallback([this](const std::string& caller) {
        test_local_incoming_ = caller;
        test_local_log_.push_back("[CALL] Incoming from " + caller);
        if (test_local_log_.size() > MAX_RX_LOG) test_local_log_.pop_front();
    });

    test_remote_.setIncomingCallCallback([this](const std::string& caller) {
        test_remote_incoming_ = caller;
        test_remote_log_.push_back("[CALL] Incoming from " + caller);
        if (test_remote_log_.size() > MAX_RX_LOG) test_remote_log_.pop_front();
    });

    // File received callbacks
    test_local_.setFileReceivedCallback([this](const std::string& path, bool ok) {
        test_local_log_.push_back(ok ? "[FILE] Received: " + path : "[FILE] Failed");
        if (test_local_log_.size() > MAX_RX_LOG) test_local_log_.pop_front();
    });

    test_remote_.setFileReceivedCallback([this](const std::string& path, bool ok) {
        test_remote_log_.push_back(ok ? "[FILE] Received: " + path : "[FILE] Failed");
        if (test_remote_log_.size() > MAX_RX_LOG) test_remote_log_.pop_front();
    });

    // Connection state callbacks - switch modem modes when connection is established
    test_local_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& remote) {
        bool connected = (state == protocol::ConnectionState::CONNECTED);
        if (connected) {
            // Compute data mode from measured SNR
            float snr = snr_slider_;  // Use slider SNR for test mode
            Modulation mod;
            CodeRate rate;
            ModemEngine::recommendDataMode(snr, mod, rate);
            test_modem_1_->setDataMode(mod, rate);
            test_local_log_.push_back("[SYS] Negotiated mode: " + std::string(
                mod == Modulation::BPSK ? "BPSK" :
                mod == Modulation::QPSK ? "QPSK" :
                mod == Modulation::QAM16 ? "16-QAM" : "64-QAM"));
        }
        test_modem_1_->setConnected(connected);
    });
    test_remote_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& remote) {
        bool connected = (state == protocol::ConnectionState::CONNECTED);
        if (connected) {
            // Compute data mode from measured SNR
            float snr = snr_slider_;  // Use slider SNR for test mode
            Modulation mod;
            CodeRate rate;
            ModemEngine::recommendDataMode(snr, mod, rate);
            test_modem_2_->setDataMode(mod, rate);
            test_remote_log_.push_back("[SYS] Negotiated mode: " + std::string(
                mod == Modulation::BPSK ? "BPSK" :
                mod == Modulation::QPSK ? "QPSK" :
                mod == Modulation::QAM16 ? "16-QAM" : "64-QAM"));
        }
        test_modem_2_->setConnected(connected);
    });

    test_local_log_.push_back("[SYS] Protocol test initialized - TEST1 station (dedicated modem)");
    test_remote_log_.push_back("[SYS] Protocol test initialized - TEST2 station (dedicated modem)");
}

void App::processTestChannel(std::vector<float>& samples) {
    // Add AWGN noise based on SNR slider
    if (snr_slider_ >= 50.0f || samples.empty()) {
        return;  // SNR >= 50 dB means essentially no noise
    }

    // Calculate actual signal RMS (like test suite does)
    float sum_sq = 0.0f;
    for (float s : samples) {
        sum_sq += s * s;
    }
    float signal_rms = std::sqrt(sum_sq / samples.size());

    // Avoid division by zero for silent buffers
    if (signal_rms < 1e-6f) {
        return;
    }

    // Calculate noise stddev for target SNR
    // SNR = 10 * log10(signal_power / noise_power)
    // noise_power = signal_power / 10^(SNR/10)
    float snr_linear = std::pow(10.0f, snr_slider_ / 10.0f);
    float signal_power = signal_rms * signal_rms;
    float noise_power = signal_power / snr_linear;
    float noise_stddev = std::sqrt(noise_power);

    // Use deterministic seed for reproducibility
    static std::mt19937 rng(42);
    std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

    for (float& sample : samples) {
        sample += noise_dist(rng);
    }
}

void App::tickProtocolTest(uint32_t elapsed_ms) {
    // === Process TX queues through simulated channel ===
    // TEST1's TX → Channel simulation → TEST2's RX
    if (!test_tx_queue_1_.empty()) {
        auto samples = std::move(test_tx_queue_1_);
        test_tx_queue_1_.clear();

        LOG_INFO("TEST", "Processing %zu samples from TEST1 -> TEST2 (SNR=%.1f dB)",
                 samples.size(), snr_slider_);

        // Apply channel effects (noise)
        processTestChannel(samples);

        // Feed to waterfall (shows channel with noise)
        waterfall_.addSamples(samples.data(), samples.size());

        // Feed to TEST2's demodulator
        test_modem_2_->receiveAudio(samples);
    }

    // TEST2's TX → Channel simulation → TEST1's RX
    if (!test_tx_queue_2_.empty()) {
        auto samples = std::move(test_tx_queue_2_);
        test_tx_queue_2_.clear();

        // Apply channel effects (noise)
        processTestChannel(samples);

        // Feed to waterfall (shows channel with noise)
        waterfall_.addSamples(samples.data(), samples.size());

        // Feed to TEST1's demodulator
        test_modem_1_->receiveAudio(samples);
    }

    // Poll both modems for received data (triggers RX callbacks)
    test_modem_1_->pollRxAudio();
    test_modem_2_->pollRxAudio();

    // Tick both protocol engines (handles ARQ timeouts, retransmissions)
    test_local_.tick(elapsed_ms);
    test_remote_.tick(elapsed_ms);
}

void App::renderProtocolTestControls() {
    // Two columns: LOCAL and REMOTE stations
    float col_width = (ImGui::GetContentRegionAvail().x - 10) / 2;

    ImGui::BeginChild("LocalStation", ImVec2(col_width, -1), true);
    {
        ImGui::TextColored(ImVec4(0.5f, 0.8f, 1.0f, 1.0f), "TEST1 STATION");
        ImGui::TextDisabled("Callsign: TEST1");
        ImGui::Separator();

        // Connection state
        auto state = test_local_.getState();
        const char* state_str = "IDLE";
        ImVec4 state_color(0.5f, 0.5f, 0.5f, 1.0f);
        if (state == protocol::ConnectionState::CONNECTING) {
            state_str = "CONNECTING...";
            state_color = ImVec4(1.0f, 0.8f, 0.0f, 1.0f);
        } else if (state == protocol::ConnectionState::CONNECTED) {
            state_str = "CONNECTED";
            state_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
        }
        ImGui::TextColored(state_color, "State: %s", state_str);
        if (state == protocol::ConnectionState::CONNECTED) {
            auto mode = test_local_.getNegotiatedMode();
            ImGui::TextDisabled("Mode: %s", protocol::waveformModeToString(mode));
        }

        // Incoming call handling
        if (!test_local_incoming_.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Incoming call from %s",
                              test_local_incoming_.c_str());
            if (ImGui::Button("Accept##local", ImVec2(60, 0))) {
                test_local_.acceptCall();
                test_local_incoming_.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("Reject##local", ImVec2(60, 0))) {
                test_local_.rejectCall();
                test_local_incoming_.clear();
            }
        }

        ImGui::Spacing();

        // Connect/Disconnect
        if (state == protocol::ConnectionState::DISCONNECTED) {
            if (ImGui::Button("Connect to TEST2", ImVec2(-1, 25))) {
                test_local_.connect("TEST2");
                test_local_log_.push_back("[TX] Connecting...");
            }
        } else if (state == protocol::ConnectionState::CONNECTED) {
            if (ImGui::Button("Disconnect##local", ImVec2(-1, 25))) {
                test_local_.disconnect();
            }

            ImGui::Spacing();

            // Message input
            ImGui::SetNextItemWidth(-1);
            bool send_local = ImGui::InputText("##localtx", test_local_tx_, sizeof(test_local_tx_),
                                                ImGuiInputTextFlags_EnterReturnsTrue);
            if (ImGui::Button("Send##local", ImVec2(-1, 0)) || send_local) {
                if (test_local_.sendMessage(test_local_tx_)) {
                    test_local_log_.push_back("[TX] " + std::string(test_local_tx_));
                    if (test_local_log_.size() > MAX_RX_LOG) test_local_log_.pop_front();
                }
            }

            // File transfer
            ImGui::SetNextItemWidth(-50);
            ImGui::InputText("##localfile", test_local_file_, sizeof(test_local_file_));
            ImGui::SameLine();
            if (ImGui::Button("...##localbrowse", ImVec2(40, 0))) {
                file_browser_.setTitle("Select File (LOCAL)");
                file_browser_.open();
                file_browser_target_ = FileBrowserTarget::TEST_LOCAL;
            }
            if (ImGui::Button("Send File##local", ImVec2(-1, 0))) {
                LOG_MODEM(INFO, "Send File clicked, path='%s'", test_local_file_);
                if (test_local_.sendFile(test_local_file_)) {
                    test_local_log_.push_back("[FILE] Sending: " + std::string(test_local_file_));
                }
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        // Log
        ImGui::Text("Log");
        ImGui::BeginChild("LocalLog", ImVec2(-1, 150), true);
        for (const auto& msg : test_local_log_) {
            ImVec4 color(0.7f, 0.7f, 0.7f, 1.0f);
            if (msg.size() >= 4 && msg.substr(0, 4) == "[TX]") color = ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
            else if (msg.size() >= 4 && msg.substr(0, 4) == "[RX]") color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
            else if (msg.size() >= 5 && msg.substr(0, 5) == "[CALL") color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            else if (msg.size() >= 5 && msg.substr(0, 5) == "[FILE") color = ImVec4(1.0f, 0.5f, 1.0f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", msg.c_str());
            ImGui::PopStyleColor();
        }
        if (!test_local_log_.empty()) ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }
    ImGui::EndChild();

    ImGui::SameLine();

    ImGui::BeginChild("RemoteStation", ImVec2(col_width, -1), true);
    {
        ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "TEST2 STATION");
        ImGui::TextDisabled("Callsign: TEST2");
        ImGui::Separator();

        // Connection state
        auto state = test_remote_.getState();
        const char* state_str = "IDLE";
        ImVec4 state_color(0.5f, 0.5f, 0.5f, 1.0f);
        if (state == protocol::ConnectionState::CONNECTING) {
            state_str = "CONNECTING...";
            state_color = ImVec4(1.0f, 0.8f, 0.0f, 1.0f);
        } else if (state == protocol::ConnectionState::CONNECTED) {
            state_str = "CONNECTED";
            state_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
        }
        ImGui::TextColored(state_color, "State: %s", state_str);
        if (state == protocol::ConnectionState::CONNECTED) {
            auto mode = test_remote_.getNegotiatedMode();
            ImGui::TextDisabled("Mode: %s", protocol::waveformModeToString(mode));
        }

        // Incoming call handling
        if (!test_remote_incoming_.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Incoming call from %s",
                              test_remote_incoming_.c_str());
            if (ImGui::Button("Accept##remote", ImVec2(60, 0))) {
                test_remote_.acceptCall();
                test_remote_incoming_.clear();
            }
            ImGui::SameLine();
            if (ImGui::Button("Reject##remote", ImVec2(60, 0))) {
                test_remote_.rejectCall();
                test_remote_incoming_.clear();
            }
        }

        ImGui::Spacing();

        // Connect/Disconnect
        if (state == protocol::ConnectionState::DISCONNECTED) {
            if (ImGui::Button("Connect to TEST1", ImVec2(-1, 25))) {
                test_remote_.connect("TEST1");
                test_remote_log_.push_back("[TX] Connecting...");
            }
        } else if (state == protocol::ConnectionState::CONNECTED) {
            if (ImGui::Button("Disconnect##remote", ImVec2(-1, 25))) {
                test_remote_.disconnect();
            }

            ImGui::Spacing();

            // Message input
            ImGui::SetNextItemWidth(-1);
            bool send_remote = ImGui::InputText("##remotetx", test_remote_tx_, sizeof(test_remote_tx_),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
            if (ImGui::Button("Send##remote", ImVec2(-1, 0)) || send_remote) {
                if (test_remote_.sendMessage(test_remote_tx_)) {
                    test_remote_log_.push_back("[TX] " + std::string(test_remote_tx_));
                    if (test_remote_log_.size() > MAX_RX_LOG) test_remote_log_.pop_front();
                }
            }

            // File transfer
            ImGui::SetNextItemWidth(-50);
            ImGui::InputText("##remotefile", test_remote_file_, sizeof(test_remote_file_));
            ImGui::SameLine();
            if (ImGui::Button("...##remotebrowse", ImVec2(40, 0))) {
                file_browser_.setTitle("Select File (REMOTE)");
                file_browser_.open();
                file_browser_target_ = FileBrowserTarget::TEST_REMOTE;
            }
            if (ImGui::Button("Send File##remote", ImVec2(-1, 0))) {
                if (test_remote_.sendFile(test_remote_file_)) {
                    test_remote_log_.push_back("[FILE] Sending: " + std::string(test_remote_file_));
                }
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        // Log
        ImGui::Text("Log");
        ImGui::BeginChild("RemoteLog", ImVec2(-1, 150), true);
        for (const auto& msg : test_remote_log_) {
            ImVec4 color(0.7f, 0.7f, 0.7f, 1.0f);
            if (msg.size() >= 4 && msg.substr(0, 4) == "[TX]") color = ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
            else if (msg.size() >= 4 && msg.substr(0, 4) == "[RX]") color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
            else if (msg.size() >= 5 && msg.substr(0, 5) == "[CALL") color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
            else if (msg.size() >= 5 && msg.substr(0, 5) == "[FILE") color = ImVec4(1.0f, 0.5f, 1.0f, 1.0f);
            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", msg.c_str());
            ImGui::PopStyleColor();
        }
        if (!test_remote_log_.empty()) ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }
    ImGui::EndChild();
}

void App::render() {
    // In loopback mode, poll audio's rx_buffer and feed to modem
    // (Loopback samples are buffered in audio engine, not sent via callback to avoid re-entrancy)
    if (mode_ == AppMode::TEST && audio_.isLoopbackEnabled()) {
        auto loopback_samples = audio_.getRxSamples(48000);  // Get up to 1 second of samples
        if (!loopback_samples.empty()) {
            modem_.receiveAudio(loopback_samples);
            waterfall_.addSamples(loopback_samples.data(), loopback_samples.size());
        }
    }

    // Poll for audio samples and process them (must be called from main loop, not audio callback)
    // This processes any samples that were queued by the audio callback thread
    if (mode_ == AppMode::TEST || mode_ == AppMode::OPERATE) {
        modem_.pollRxAudio();
    }

    // === DEBUG: Test signal keys (F1-F6) ===
    // F1: Send 1500 Hz test tone
    // F2: Send test pattern (all zeros) with LDPC
    // F3: Send test pattern (DEADBEEF) with LDPC - non-trivial pattern to verify decoding
    // F4: Send test pattern (alternating 0101) with LDPC
    // F5: Send RAW OFDM (NO LDPC) - 0xAA 0x55 pattern
    // F6: Send RAW OFDM (NO LDPC) - DEADBEEF pattern
    if (ImGui::IsKeyPressed(ImGuiKey_F1)) {
        auto tone = modem_.generateTestTone(1.0f);
        audio_.queueTxSamples(tone);
        rx_log_.push_back("[TEST] Sent 1500 Hz tone");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F2)) {
        auto samples = modem_.transmitTestPattern(0);  // All zeros
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent pattern: ALL ZEROS (21 bytes, LDPC encoded)");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F3)) {
        auto samples = modem_.transmitTestPattern(1);  // DEADBEEF
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent pattern: DEADBEEF (21 bytes, LDPC encoded)");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F4)) {
        auto samples = modem_.transmitTestPattern(2);  // Alternating
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent pattern: ALTERNATING (21 bytes, LDPC encoded)");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F5)) {
        auto samples = modem_.transmitRawOFDM(0);  // Raw OFDM, NO LDPC, 0xAA 0x55
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent RAW OFDM: 0xAA 0x55 pattern (81 bytes, NO LDPC)");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F6)) {
        auto samples = modem_.transmitRawOFDM(1);  // Raw OFDM, NO LDPC, DEADBEEF
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent RAW OFDM: DEADBEEF pattern (81 bytes, NO LDPC)");
    }

    // Protocol engine tick (for ARQ timeouts)
    uint32_t now = SDL_GetTicks();
    uint32_t elapsed = (last_tick_time_ == 0) ? 0 : (now - last_tick_time_);
    last_tick_time_ = now;

    if (mode_ == AppMode::OPERATE) {
        if (elapsed > 0 && elapsed < 1000) {  // Sanity check
            protocol_.tick(elapsed);
        }
    } else if (mode_ == AppMode::TEST && test_protocol_mode_) {
        if (elapsed > 0 && elapsed < 1000) {
            tickProtocolTest(elapsed);
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

    // Constellation at top (fixed height)
    ImGui::BeginChild("ConstellationArea", ImVec2(0, 200), false);
    auto symbols = modem_.getConstellationSymbols();
    constellation_.render(symbols, config_.modulation);
    ImGui::EndChild();

    // Waterfall below constellation (fills remaining space)
    ImGui::Separator();
    waterfall_.render();

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

    // Render file browser (if open)
    if (file_browser_.render()) {
        // File was selected - copy to the appropriate buffer
        const std::string& path = file_browser_.getSelectedPath();
        LOG_MODEM(INFO, "File browser selected: '%s' (target=%d)",
                  path.c_str(), static_cast<int>(file_browser_target_));
        switch (file_browser_target_) {
            case FileBrowserTarget::OPERATE:
                strncpy(file_path_buffer_, path.c_str(), sizeof(file_path_buffer_) - 1);
                file_path_buffer_[sizeof(file_path_buffer_) - 1] = '\0';
                break;
            case FileBrowserTarget::TEST_LOCAL:
                strncpy(test_local_file_, path.c_str(), sizeof(test_local_file_) - 1);
                test_local_file_[sizeof(test_local_file_) - 1] = '\0';
                LOG_MODEM(INFO, "Copied to test_local_file_: '%s'", test_local_file_);
                break;
            case FileBrowserTarget::TEST_REMOTE:
                strncpy(test_remote_file_, path.c_str(), sizeof(test_remote_file_) - 1);
                test_remote_file_[sizeof(test_remote_file_) - 1] = '\0';
                LOG_MODEM(INFO, "Copied to test_remote_file_: '%s'", test_remote_file_);
                break;
        }
    }
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

    // Set up RX callback - feed captured audio to modem and waterfall
    audio_.setRxCallback([this](const std::vector<float>& samples) {
        modem_.receiveAudio(samples);
        waterfall_.addSamples(samples.data(), samples.size());
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

        // Show negotiated waveform mode
        auto mode = protocol_.getNegotiatedMode();
        const char* mode_str = protocol::waveformModeToString(mode);
        ImGui::TextDisabled("Waveform: %s", mode_str);
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
        // Auto-start audio if not already running
        if (!radio_rx_enabled_) {
            if (!audio_initialized_) {
                initRadioAudio();
            }
            audio_.openOutput(getOutputDeviceName());
            audio_.startPlayback();
            startRadioRx();
        }
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

        // Stop listening button
        if (ImGui::Button("Stop Listening", ImVec2(-1, 25))) {
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

        // Input gain slider (useful for hot mic inputs)
        // Range 0.01x (-40dB) to 2.0x (+6dB) for handling very hot inputs
        static float input_gain = 1.0f;
        ImGui::SetNextItemWidth(120);
        if (ImGui::SliderFloat("Input Gain", &input_gain, 0.01f, 2.0f, "%.2fx", ImGuiSliderFlags_Logarithmic)) {
            audio_.setInputGain(input_gain);
        }
        ImGui::SameLine();
        float gain_db = 20.0f * log10f(input_gain + 0.0001f);
        ImGui::TextDisabled("(%.0f dB)", gain_db);

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
            rx_log_.push_back("[TX] " + text);
            if (rx_log_.size() > MAX_RX_LOG) {
                rx_log_.pop_front();
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

    // File Transfer Section
    ImGui::Text("File Transfer");

    // Show progress if transfer in progress
    if (protocol_.isFileTransferInProgress()) {
        auto progress = protocol_.getFileProgress();

        ImVec4 color = progress.is_sending
            ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f)   // Blue for sending
            : ImVec4(0.5f, 1.0f, 0.5f, 1.0f);  // Green for receiving

        ImGui::TextColored(color, "%s: %s",
            progress.is_sending ? "Sending" : "Receiving",
            progress.filename.c_str());

        ImGui::ProgressBar(progress.percentage() / 100.0f, ImVec2(-1, 20));

        ImGui::Text("%u / %u bytes (%.1f%%)",
            progress.transferred_bytes,
            progress.total_bytes,
            progress.percentage());

        if (ImGui::Button("Cancel Transfer", ImVec2(-1, 25))) {
            protocol_.cancelFileTransfer();
        }
    } else {
        // File path input with Browse button
        ImGui::SetNextItemWidth(-80);  // Leave room for Browse button
        ImGui::InputText("##filepath", file_path_buffer_, sizeof(file_path_buffer_));
        ImGui::SameLine();
        if (ImGui::Button("Browse", ImVec2(70, 0))) {
            file_browser_.setTitle("Select File to Send");
            file_browser_.open();
            file_browser_target_ = FileBrowserTarget::OPERATE;
        }

        // Send button
        bool can_send_file = protocol_.isConnected() &&
                             protocol_.isReadyToSend() &&
                             strlen(file_path_buffer_) > 0 &&
                             !protocol_.isFileTransferInProgress();

        ImGui::BeginDisabled(!can_send_file);
        if (ImGui::Button("Send File", ImVec2(-1, 30))) {
            if (protocol_.sendFile(file_path_buffer_)) {
                rx_log_.push_back("[FILE] Sending: " + std::string(file_path_buffer_));
                if (rx_log_.size() > MAX_RX_LOG) {
                    rx_log_.pop_front();
                }
            } else {
                rx_log_.push_back("[FILE] Failed to start transfer");
                if (rx_log_.size() > MAX_RX_LOG) {
                    rx_log_.pop_front();
                }
            }
        }
        ImGui::EndDisabled();

        if (!protocol_.isConnected()) {
            ImGui::TextDisabled("Connect to send files");
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // RX log
    ImGui::Text("Message Log");
    ImGui::BeginChild("RXLogRadio", ImVec2(-1, 150), true);
    for (const auto& msg : rx_log_) {
        ImVec4 color = (msg.size() >= 4 && msg.substr(0, 4) == "[TX]")
            ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f)
            : ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", msg.c_str());
        ImGui::PopStyleColor();
    }
    if (!rx_log_.empty()) ImGui::SetScrollHereY(1.0f);
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
