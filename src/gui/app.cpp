#include "app.hpp"
#include "imgui.h"
#include "ultra/logging.hpp"
#include <SDL.h>
#include <cstring>
#include <cmath>
#include <fstream>
#include <cstdarg>
#include <chrono>
#include <ctime>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#define MKDIR(dir) mkdir(dir, 0755)
#endif

namespace ultra {
namespace gui {

// File logger for GUI debugging - writes to logs/gui.log next to binary
static std::ofstream g_log_file;
static bool g_log_initialized = false;

static void initLog() {
    if (g_log_initialized) return;
    g_log_initialized = true;

    // Create logs directory next to binary
    MKDIR("logs");
    g_log_file.open("logs/gui.log", std::ios::out | std::ios::trunc);
}

static void guiLog(const char* fmt, ...) {
    initLog();
    if (!g_log_file.is_open()) return;

    // Timestamp
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    char timebuf[32];
    std::strftime(timebuf, sizeof(timebuf), "%H:%M:%S", std::localtime(&time));

    // Format message
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    g_log_file << "[" << timebuf << "." << ms.count() << "] " << buf << std::endl;
    g_log_file.flush();
}

App::App() : App(Options{}) {}

App::App(const Options& opts) : options_(opts), sim_ui_visible_(opts.enable_sim) {
    guiLog("=== GUI Started ===");
    // Load persistent settings
    settings_.load();

    config_ = presets::balanced();

    // Initialize protocol with saved callsign
    if (strlen(settings_.callsign) > 0) {
        protocol_.setLocalCallsign(settings_.callsign);
        modem_.setLogPrefix(settings_.callsign);
    }

    // Set up raw data callback for protocol layer
    modem_.setRawDataCallback([this](const Bytes& data) {
        guiLog("Our modem decoded %zu bytes", data.size());
        // Update protocol layer with current SNR before processing frame
        // In simulation mode, use the known simulation SNR (DPSK doesn't measure SNR)
        // In real mode, use measured SNR from OFDM demodulator
        float snr_db = simulation_enabled_ ? simulation_snr_db_ : modem_.getStats().snr_db;
        protocol_.setMeasuredSNR(snr_db);
        protocol_.onRxData(data);
    });

    // Set up status callback to show codeword progress in RX log
    modem_.setStatusCallback([this](const std::string& status) {
        rx_log_.push_back(status);
    });

    // Set up protocol engine callbacks
    protocol_.setTxDataCallback([this](const Bytes& data) {
        // When protocol layer wants to transmit, convert to audio
        auto samples = modem_.transmit(data);
        if (!samples.empty()) {
            if (simulation_enabled_) {
                // Simulation mode: queue samples for virtual station
                sim_our_tx_queue_.insert(sim_our_tx_queue_.end(),
                                         samples.begin(), samples.end());
            } else {
                // Normal mode: send to real audio
                audio_.queueTxSamples(samples);
            }
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
        guiLog("Connection state changed: %d (%s)", static_cast<int>(state), info.c_str());

        // Update modem engine connection state (affects waveform selection)
        bool connected = (state == protocol::ConnectionState::CONNECTED);
        modem_.setConnected(connected);

        std::string msg;
        switch (state) {
            case protocol::ConnectionState::CONNECTING:
                msg = "[SYS] Connecting to " + info + "...";
                break;
            case protocol::ConnectionState::CONNECTED:
                msg = "[SYS] Connected to " + info;  // info contains remote callsign
                break;
            case protocol::ConnectionState::DISCONNECTING:
                msg = "[SYS] Disconnecting...";
                break;
            case protocol::ConnectionState::DISCONNECTED:
                if (info.find("timeout") != std::string::npos) {
                    msg = "[FAILED] " + info;  // Make failures more visible
                } else {
                    msg = "[SYS] Disconnected" + (info.empty() ? "" : ": " + info);
                }
                // Reset waveform mode to OFDM when disconnected
                modem_.setWaveformMode(protocol::WaveformMode::OFDM);
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

    protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr_db) {
        guiLog("MODE_CHANGE: %s R%s (SNR=%.1f dB)",
               modulationToString(mod),
               codeRateToString(rate),
               snr_db);

        // Update modem engine with new data mode
        modem_.setDataMode(mod, rate);

        std::string msg = "[MODE] " + std::string(modulationToString(mod)) + " " +
                          std::string(codeRateToString(rate)) + " (SNR=" +
                          std::to_string(static_cast<int>(snr_db)) + " dB)";
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    // Waveform mode negotiation callback (OFDM, DPSK, MFSK switching)
    protocol_.setModeNegotiatedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = "OFDM";
        switch (mode) {
            case protocol::WaveformMode::DPSK: mode_name = "DPSK"; break;
            case protocol::WaveformMode::MFSK: mode_name = "MFSK"; break;
            case protocol::WaveformMode::OTFS_EQ: mode_name = "OTFS-EQ"; break;
            case protocol::WaveformMode::OTFS_RAW: mode_name = "OTFS-RAW"; break;
            default: mode_name = "OFDM"; break;
        }
        guiLog("WAVEFORM_CHANGE: %s", mode_name);

        // Update modem engine with new waveform mode
        modem_.setWaveformMode(mode);

        std::string msg = "[WAVEFORM] " + std::string(mode_name);
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    // Connect waveform fallback callback (DPSK -> MFSK when connection attempts fail)
    protocol_.setConnectWaveformChangedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = (mode == protocol::WaveformMode::MFSK) ? "MFSK" : "DPSK";
        guiLog("CONNECT_WAVEFORM: Switching to %s for connection attempts", mode_name);
        modem_.setConnectWaveform(mode);
    });

    // Handshake confirmed callback - now safe to use negotiated waveform
    protocol_.setHandshakeConfirmedCallback([this]() {
        guiLog("HANDSHAKE: Confirmed, switching to negotiated waveform");
        modem_.setHandshakeComplete(true);
    });

    // File transfer callbacks
    protocol_.setFileProgressCallback([this](const protocol::FileTransferProgress& p) {
        // Progress is displayed in renderOperateTab()
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
    waterfall_.setDynamicRange(-60.0f, 0.0f);

    // Settings window callbacks
    settings_window_.setCallsignChangedCallback([this](const std::string& call) {
        protocol_.setLocalCallsign(call);
        modem_.setLogPrefix(call);
        settings_.save();
    });

    settings_window_.setAudioResetCallback([this]() {
        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();
        audio_.shutdown();
        audio_initialized_ = false;

        initAudio();
        if (audio_initialized_) {
            std::string output_dev = getOutputDeviceName();
            audio_.openOutput(output_dev);
            audio_.startPlayback();
            startRadioRx();
        }
    });

    settings_window_.setClosedCallback([this]() {
        settings_.save();

        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();

        if (audio_initialized_) {
            std::string output_dev = getOutputDeviceName();
            audio_.openOutput(output_dev);
            audio_.startPlayback();
            startRadioRx();
        }
    });

    settings_window_.setFilterChangedCallback([this](bool enabled, float center, float bw, int taps) {
        FilterConfig filter_config;
        filter_config.enabled = enabled;
        filter_config.center_freq = center;
        filter_config.bandwidth = bw;
        filter_config.taps = taps;
        modem_.setFilterConfig(filter_config);
        settings_.save();
    });

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

    // Initialize virtual station for simulation mode
    initVirtualStation();

    // Auto-initialize audio on startup
    initAudio();
    if (audio_initialized_) {
        std::string output_dev = getOutputDeviceName();
        audio_.openOutput(output_dev);
        audio_.startPlayback();
        startRadioRx();
    }
}

App::~App() {
    // Stop simulator thread first
    stopSimThread();

    settings_.save();
    audio_.shutdown();

    // Write recording to file if -rec was enabled
    if (options_.record_audio && !recorded_samples_.empty()) {
        writeRecordingToFile();
    }
}

void App::writeRecordingToFile() {
    std::ofstream file(options_.record_path, std::ios::binary);
    if (file.is_open()) {
        file.write(reinterpret_cast<const char*>(recorded_samples_.data()),
                   recorded_samples_.size() * sizeof(float));
        guiLog("Recording saved: %s (%zu samples, %.1f seconds)",
               options_.record_path.c_str(),
               recorded_samples_.size(),
               recorded_samples_.size() / 48000.0f);
    } else {
        guiLog("ERROR: Failed to save recording to %s", options_.record_path.c_str());
    }
}

void App::initVirtualStation() {
    // Create virtual station's modem
    virtual_modem_ = std::make_unique<ModemEngine>();

    // Set up virtual station's protocol
    virtual_protocol_.setLocalCallsign(virtual_callsign_);
    virtual_modem_->setLogPrefix(virtual_callsign_);
    virtual_protocol_.setAutoAccept(true);  // Auto-accept incoming calls

    // Virtual station TX → channel sim → our RX
    virtual_protocol_.setTxDataCallback([this](const Bytes& data) {
        guiLog("SIM: Virtual station TX %zu bytes", data.size());
        auto samples = virtual_modem_->transmit(data);
        guiLog("SIM: Virtual modem produced %zu samples", samples.size());
        sim_virtual_tx_queue_.insert(sim_virtual_tx_queue_.end(),
                                      samples.begin(), samples.end());
    });

    // Virtual modem RX → virtual protocol
    virtual_modem_->setRawDataCallback([this](const Bytes& data) {
        guiLog("SIM: Virtual modem decoded %zu bytes", data.size());
        // Use simulation SNR - DPSK demodulator doesn't measure SNR
        // The virtual station sees the same channel as our station
        virtual_protocol_.setMeasuredSNR(simulation_snr_db_);
        virtual_protocol_.onRxData(data);
    });

    // Log virtual station events
    virtual_protocol_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        guiLog("SIM: Virtual station connection state: %d (%s)", static_cast<int>(state), info.c_str());

        // Update virtual modem engine connection state
        bool connected = (state == protocol::ConnectionState::CONNECTED);
        virtual_modem_->setConnected(connected);

        std::string msg = "[SIM] ";
        switch (state) {
            case protocol::ConnectionState::CONNECTED:
                msg += "Virtual station connected";
                break;
            case protocol::ConnectionState::DISCONNECTED:
                msg += "Virtual station disconnected";
                // Reset waveform mode to OFDM when disconnected
                virtual_modem_->setWaveformMode(protocol::WaveformMode::OFDM);
                break;
            default:
                return;  // Don't log intermediate states
        }
        rx_log_.push_back(msg);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    });

    virtual_protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr_db) {
        guiLog("SIM: Virtual MODE_CHANGE: %s R%s (SNR=%.1f dB)",
               modulationToString(mod),
               codeRateToString(rate),
               snr_db);
        // Update virtual modem engine with new data mode
        virtual_modem_->setDataMode(mod, rate);
    });

    virtual_protocol_.setModeNegotiatedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = "OFDM";
        switch (mode) {
            case protocol::WaveformMode::DPSK: mode_name = "DPSK"; break;
            case protocol::WaveformMode::MFSK: mode_name = "MFSK"; break;
            case protocol::WaveformMode::OTFS_EQ: mode_name = "OTFS-EQ"; break;
            case protocol::WaveformMode::OTFS_RAW: mode_name = "OTFS-RAW"; break;
            default: mode_name = "OFDM"; break;
        }
        guiLog("SIM: Virtual WAVEFORM_CHANGE: %s", mode_name);
        // Update virtual modem engine with new waveform mode
        virtual_modem_->setWaveformMode(mode);
    });

    // Connect waveform fallback for virtual station
    virtual_protocol_.setConnectWaveformChangedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = (mode == protocol::WaveformMode::MFSK) ? "MFSK" : "DPSK";
        guiLog("SIM: Virtual CONNECT_WAVEFORM: Switching to %s", mode_name);
        virtual_modem_->setConnectWaveform(mode);
    });

    // Virtual station handshake confirmed callback
    virtual_protocol_.setHandshakeConfirmedCallback([this]() {
        guiLog("SIM: Virtual HANDSHAKE confirmed");
        virtual_modem_->setHandshakeComplete(true);
    });

    virtual_protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        // Virtual station received our message - it could auto-reply here
        // For now, just log that it received the message
        guiLog("SIM: Virtual station received msg from %s: %s", from.c_str(), text.c_str());
    });

    guiLog("Virtual station initialized: callsign=%s", virtual_callsign_.c_str());
}

void App::applyChannelSimulation(std::vector<float>& samples) {
    if (simulation_snr_db_ >= 50.0f || samples.empty()) {
        return;  // No noise for very high SNR
    }

    // Calculate signal RMS
    float sum_sq = 0.0f;
    for (float s : samples) {
        sum_sq += s * s;
    }
    float signal_rms = std::sqrt(sum_sq / samples.size());

    if (signal_rms < 1e-6f) {
        return;  // Silent buffer
    }

    // Calculate noise level for target SNR
    float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
    float signal_power = signal_rms * signal_rms;
    float noise_power = signal_power / snr_linear;
    float noise_stddev = std::sqrt(noise_power);

    std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

    for (float& sample : samples) {
        sample += noise_dist(sim_rng_);
    }
}

void App::prependPttNoise(std::vector<float>& samples) {
    // Simulate realistic PTT timing: 100-500ms of noise before signal
    // This mimics radio key-up delay, AGC settling, etc.
    std::uniform_int_distribution<size_t> delay_dist(4800, 24000);  // 100-500ms at 48kHz
    size_t noise_samples = delay_dist(sim_rng_);

    // Calculate noise level based on typical signal level (~0.1 RMS) and SNR
    float typical_signal_rms = 0.1f;
    float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
    float noise_power = (typical_signal_rms * typical_signal_rms) / snr_linear;
    float noise_stddev = std::sqrt(noise_power);

    std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

    // Create noise buffer and prepend to samples
    std::vector<float> noise_buffer(noise_samples);
    for (size_t i = 0; i < noise_samples; ++i) {
        noise_buffer[i] = noise_dist(sim_rng_);
    }

    // Prepend noise to signal
    samples.insert(samples.begin(), noise_buffer.begin(), noise_buffer.end());

    guiLog("SIM: Prepended %zu samples (%.0fms) of PTT noise",
           noise_samples, noise_samples * 1000.0f / 48000.0f);
}

void App::startSimThread() {
    if (sim_thread_running_) return;

    guiLog("SIM: Starting audio thread");
    sim_thread_running_ = true;
    sim_thread_ = std::thread(&App::simThreadLoop, this);
}

void App::stopSimThread() {
    if (!sim_thread_running_) return;

    guiLog("SIM: Stopping audio thread");
    sim_thread_running_ = false;
    sim_cv_.notify_all();

    if (sim_thread_.joinable()) {
        sim_thread_.join();
    }
    guiLog("SIM: Audio thread stopped");
}

void App::simThreadLoop() {
    guiLog("SIM: Audio thread started");

    // Deliver samples at ~48kHz (10ms chunks = 480 samples for efficiency)
    // Larger chunks reduce lock contention while still being responsive
    constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz
    constexpr auto CHUNK_DURATION = std::chrono::milliseconds(10);

    auto next_tick = std::chrono::steady_clock::now();
    size_t total_our_streamed = 0;
    size_t total_virtual_streamed = 0;

    while (sim_thread_running_) {
        // Wait until next tick time
        std::this_thread::sleep_until(next_tick);
        next_tick += CHUNK_DURATION;

        std::vector<float> our_chunk;
        std::vector<float> virtual_chunk;
        bool our_tx_complete = false;
        bool virtual_tx_complete = false;

        // Grab chunks under lock (minimize lock time)
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);

            // Grab chunk from our streaming buffer
            if (!sim_our_streaming_.empty()) {
                size_t to_send = std::min(CHUNK_SIZE, sim_our_streaming_.size());
                our_chunk.assign(sim_our_streaming_.begin(),
                                 sim_our_streaming_.begin() + to_send);
                sim_our_streaming_.erase(sim_our_streaming_.begin(),
                                         sim_our_streaming_.begin() + to_send);
                our_tx_complete = sim_our_streaming_.empty();
            }

            // Grab chunk from virtual's streaming buffer
            if (!sim_virtual_streaming_.empty()) {
                size_t to_send = std::min(CHUNK_SIZE, sim_virtual_streaming_.size());
                virtual_chunk.assign(sim_virtual_streaming_.begin(),
                                     sim_virtual_streaming_.begin() + to_send);
                sim_virtual_streaming_.erase(sim_virtual_streaming_.begin(),
                                             sim_virtual_streaming_.begin() + to_send);
                virtual_tx_complete = sim_virtual_streaming_.empty();
            }
        }

        // Process chunks outside lock (slow operations)

        // Stream our samples → virtual modem
        if (!our_chunk.empty()) {
            total_our_streamed += our_chunk.size();

            // Record samples if enabled
            if (recording_enabled_) {
                recorded_samples_.insert(recorded_samples_.end(), our_chunk.begin(), our_chunk.end());
            }

            // Show on waterfall
            waterfall_.addSamples(our_chunk.data(), our_chunk.size());

            // Feed to virtual modem
            virtual_modem_->receiveAudio(our_chunk);
            virtual_modem_->pollRxAudio();

            if (our_tx_complete) {
                guiLog("SIM: TX -> virtual complete (thread, %zu samples)", total_our_streamed);
                total_our_streamed = 0;
                sim_virtual_ptt_delay_ = PTT_DELAY_MS;
            }
        }

        // Stream virtual's samples → our modem
        if (!virtual_chunk.empty()) {
            total_virtual_streamed += virtual_chunk.size();

            // Record samples if enabled
            if (recording_enabled_) {
                recorded_samples_.insert(recorded_samples_.end(), virtual_chunk.begin(), virtual_chunk.end());
            }

            // Show on waterfall
            waterfall_.addSamples(virtual_chunk.data(), virtual_chunk.size());

            // Feed to our modem
            modem_.receiveAudio(virtual_chunk);
            modem_.pollRxAudio();

            if (virtual_tx_complete) {
                guiLog("SIM: TX <- virtual complete (thread, %zu samples)", total_virtual_streamed);
                total_virtual_streamed = 0;
                sim_our_ptt_delay_ = PTT_DELAY_MS;
            }
        }

        // Also poll modems to process any accumulated samples
        virtual_modem_->pollRxAudio();
        modem_.pollRxAudio();
    }

    guiLog("SIM: Audio thread exiting");
}

void App::tickSimulation(uint32_t elapsed_ms) {
    if (!simulation_enabled_) return;

    // Update PTT delays
    if (sim_our_ptt_delay_ > 0) {
        sim_our_ptt_delay_ = (elapsed_ms >= sim_our_ptt_delay_) ? 0 : sim_our_ptt_delay_ - elapsed_ms;
    }
    if (sim_virtual_ptt_delay_ > 0) {
        sim_virtual_ptt_delay_ = (elapsed_ms >= sim_virtual_ptt_delay_) ? 0 : sim_virtual_ptt_delay_ - elapsed_ms;
    }

    // === Move pending TX to streaming buffers (with PTT delay) ===
    // Note: Actual sample streaming is done by simThreadLoop() at 48kHz

    std::lock_guard<std::mutex> lock(sim_mutex_);

    // Our TX → streaming to virtual
    if (!sim_our_tx_queue_.empty() && sim_our_streaming_.empty() && sim_our_ptt_delay_ == 0) {
        guiLog("SIM: Starting TX -> virtual (%zu samples, %.1fs)",
               sim_our_tx_queue_.size(), sim_our_tx_queue_.size() / 48000.0f);

        // Prepend PTT noise and apply channel effects
        prependPttNoise(sim_our_tx_queue_);
        applyChannelSimulation(sim_our_tx_queue_);

        // Move to streaming buffer (thread will stream at 48kHz)
        sim_our_streaming_ = std::move(sim_our_tx_queue_);
        sim_our_tx_queue_.clear();

        // Virtual can't TX until we're done (half-duplex)
        sim_virtual_ptt_delay_ = PTT_DELAY_MS;
    }

    // Virtual's TX → streaming to us
    if (!sim_virtual_tx_queue_.empty() && sim_virtual_streaming_.empty() && sim_virtual_ptt_delay_ == 0) {
        guiLog("SIM: Starting TX <- virtual (%zu samples, %.1fs)",
               sim_virtual_tx_queue_.size(), sim_virtual_tx_queue_.size() / 48000.0f);

        // Prepend PTT noise and apply channel effects
        prependPttNoise(sim_virtual_tx_queue_);
        applyChannelSimulation(sim_virtual_tx_queue_);

        // Move to streaming buffer (thread will stream at 48kHz)
        sim_virtual_streaming_ = std::move(sim_virtual_tx_queue_);
        sim_virtual_tx_queue_.clear();

        // We can't TX until virtual is done (half-duplex)
        sim_our_ptt_delay_ = PTT_DELAY_MS;
    }

    // NOTE: Streaming and modem polling now done by simThreadLoop()

    // === Tick protocol engines ===
    // This may generate new TX samples into the queues
    virtual_protocol_.tick(elapsed_ms);
    protocol_.tick(elapsed_ms);
}

void App::initAudio() {
    if (audio_initialized_) return;

    if (!audio_.initialize()) {
        return;
    }

    // Enumerate devices
    input_devices_ = audio_.getInputDevices();
    output_devices_ = audio_.getOutputDevices();

    // Populate settings window device lists
    settings_window_.input_devices = input_devices_;
    settings_window_.output_devices = output_devices_;

    audio_initialized_ = true;
}

void App::sendMessage() {
    // Not used in current implementation - messages sent via protocol
}

void App::onDataReceived(const std::string& text) {
    if (!text.empty()) {
        rx_log_.push_back("[RX] " + text);
        if (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    }
}

void App::render() {
    // Poll modem for received data
    if (!simulation_enabled_) {
        modem_.pollRxAudio();
    }

    // === DEBUG: Test signal keys (F1-F7) ===
    if (ImGui::IsKeyPressed(ImGuiKey_F1)) {
        auto tone = modem_.generateTestTone(1.0f);
        audio_.queueTxSamples(tone);
        rx_log_.push_back("[TEST] Sent 1500 Hz tone");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F2)) {
        auto samples = modem_.transmitTestPattern(0);
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent pattern: ALL ZEROS (LDPC encoded)");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F3)) {
        auto samples = modem_.transmitTestPattern(1);
        audio_.queueTxSamples(samples);
        rx_log_.push_back("[TEST] Sent pattern: DEADBEEF (LDPC encoded)");
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F7)) {
        const char* test_file = "tests/data/test_connect_data_sequence.f32";
        size_t injected = modem_.injectSignalFromFile(test_file);
        if (injected > 0) {
            rx_log_.push_back("[TEST] Injected " + std::to_string(injected) + " samples");
        } else {
            rx_log_.push_back("[TEST] Failed to inject signal");
        }
    }

    // Protocol engine tick
    uint32_t now = SDL_GetTicks();
    uint32_t elapsed = (last_tick_time_ == 0) ? 0 : (now - last_tick_time_);
    last_tick_time_ = now;

    if (elapsed > 0 && elapsed < 1000) {
        if (simulation_enabled_) {
            tickSimulation(elapsed);
        } else {
            protocol_.tick(elapsed);
        }
    }

    // Create main window
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

    // Title bar
    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "ProjectUltra");
    ImGui::SameLine();
    ImGui::TextDisabled("High-Speed HF Modem");

    // Settings button
    ImGui::SameLine(ImGui::GetWindowWidth() - 100);
    if (ImGui::SmallButton("Settings")) {
        settings_window_.open();
    }

    ImGui::Separator();

    // Main content area - Two column layout
    float content_height = ImGui::GetContentRegionAvail().y - 30;

    ImGui::BeginChild("ContentArea", ImVec2(0, content_height), false);

    float total_width = ImGui::GetContentRegionAvail().x;
    float left_width = total_width * 0.32f;  // Monitoring column

    // ========================================
    // LEFT COLUMN: Monitoring (Constellation + Channel Status + Waterfall)
    // ========================================
    ImGui::BeginChild("LeftPanel", ImVec2(left_width, 0), true);

    // Constellation diagram
    ImGui::BeginChild("ConstellationArea", ImVec2(0, 180), false);
    auto symbols = modem_.getConstellationSymbols();
    constellation_.render(symbols, config_.modulation);
    ImGui::EndChild();

    ImGui::Separator();

    // Compact Channel Status (horizontal layout)
    auto modem_stats = modem_.getStats();
    auto data_mod = protocol_.getDataModulation();
    auto data_rate = protocol_.getDataCodeRate();
    renderCompactChannelStatus(modem_stats, data_mod, data_rate);

    ImGui::Separator();

    // Waterfall (uses remaining space)
    waterfall_.render();

    ImGui::EndChild();
    ImGui::SameLine();

    // ========================================
    // RIGHT COLUMN: Operating (Controls + Message Log)
    // ========================================
    ImGui::BeginChild("RightPanel", ImVec2(0, 0), true);
    renderOperateTab();
    ImGui::EndChild();

    ImGui::EndChild();

    // Status bar
    ImGui::Separator();
    auto mstats = modem_.getStats();
    const char* mode_str = simulation_enabled_ ? "SIMULATION" : (ptt_active_ ? "TX" : (radio_rx_enabled_ ? "RX" : "IDLE"));
    ImGui::Text("Mode: %s | SNR: %.1f dB | TX: %d | RX: %d | Throughput: %d bps",
        mode_str, mstats.snr_db, mstats.frames_sent, mstats.frames_received, mstats.throughput_bps);

    ImGui::End();

    // Render settings window
    if (settings_window_.isVisible() && settings_window_.input_devices.empty()) {
        if (!audio_.isInitialized()) {
            audio_.initialize();
        }
        settings_window_.input_devices = audio_.getInputDevices();
        settings_window_.output_devices = audio_.getOutputDevices();
    }
    settings_window_.render(settings_);

    // Render file browser
    if (file_browser_.render()) {
        const std::string& path = file_browser_.getSelectedPath();
        strncpy(file_path_buffer_, path.c_str(), sizeof(file_path_buffer_) - 1);
        file_path_buffer_[sizeof(file_path_buffer_) - 1] = '\0';
    }
}

std::string App::getInputDeviceName() const {
    if (strcmp(settings_.input_device, "Default") == 0 || settings_.input_device[0] == '\0') {
        return "";
    }
    return settings_.input_device;
}

std::string App::getOutputDeviceName() const {
    if (strcmp(settings_.output_device, "Default") == 0 || settings_.output_device[0] == '\0') {
        return "";
    }
    return settings_.output_device;
}

void App::startRadioRx() {
    if (!audio_initialized_ || simulation_enabled_) return;

    std::string input_dev = getInputDeviceName();
    if (!audio_.openInput(input_dev)) {
        return;
    }

    audio_.setRxCallback([this](const std::vector<float>& samples) {
        modem_.receiveAudio(samples);
        waterfall_.addSamples(samples.data(), samples.size());
    });

    audio_.setLoopbackEnabled(false);
    audio_.startCapture();
    radio_rx_enabled_ = true;
}

void App::stopRadioRx() {
    audio_.stopCapture();
    audio_.closeInput();
    radio_rx_enabled_ = false;
}

// Helper function to classify channel quality based on SNR
static const char* getChannelQuality(float snr_db, ImVec4& color) {
    if (snr_db >= 30.0f) {
        color = ImVec4(0.0f, 1.0f, 0.5f, 1.0f);  // Bright cyan-green
        return "Outstanding";
    } else if (snr_db >= 20.0f) {
        color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Bright green
        return "Excellent";
    } else if (snr_db >= 15.0f) {
        color = ImVec4(0.4f, 0.9f, 0.2f, 1.0f);  // Light green
        return "Very Good";
    } else if (snr_db >= 10.0f) {
        color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        return "Good";
    } else if (snr_db >= 5.0f) {
        color = ImVec4(1.0f, 0.6f, 0.0f, 1.0f);  // Orange
        return "Fair";
    } else if (snr_db >= 0.0f) {
        color = ImVec4(1.0f, 0.3f, 0.0f, 1.0f);  // Red-orange
        return "Poor";
    } else {
        color = ImVec4(1.0f, 0.1f, 0.1f, 1.0f);  // Red
        return "Very Poor";
    }
}

void App::renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate) {
    // Compact horizontal Channel Status display
    ImGui::BeginChild("ChannelStatus", ImVec2(0, 110), false);

    // Row 1: Sync indicator + Channel Quality + SNR bar
    ImVec4 sync_color = stats.synced ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
    ImGui::TextColored(sync_color, "%s", stats.synced ? "SYNC" : "----");
    ImGui::SameLine();

    // Channel quality classification
    ImVec4 quality_color;
    const char* quality_str = getChannelQuality(stats.snr_db, quality_color);
    ImGui::TextColored(quality_color, "[%s]", quality_str);
    ImGui::SameLine();
    ImGui::Text("SNR:");
    ImGui::SameLine();

    // SNR bar
    float snr_normalized = stats.snr_db / 40.0f;
    snr_normalized = std::max(0.0f, std::min(1.0f, snr_normalized));
    ImVec4 snr_color = (stats.snr_db >= 20.0f) ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) :
                       (stats.snr_db >= 10.0f) ? ImVec4(1.0f, 1.0f, 0.2f, 1.0f) :
                                                  ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, snr_color);
    char snr_text[16];
    snprintf(snr_text, sizeof(snr_text), "%.1f dB", stats.snr_db);
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::ProgressBar(snr_normalized, ImVec2(-1, 16), snr_text);
    ImGui::PopStyleColor();

    // Row 2: Waveform + Mode + Theoretical max speed
    auto waveform = modem_.getWaveformMode();
    const char* wf_str = protocol::waveformModeToString(waveform);
    ImVec4 wf_color = (waveform == protocol::WaveformMode::OFDM) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                      (waveform == protocol::WaveformMode::DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                      (waveform == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                   ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
    ImGui::TextColored(wf_color, "%s", wf_str);
    ImGui::SameLine();
    ImGui::Text("%s %s", modulationToString(data_mod), codeRateToString(data_rate));
    ImGui::SameLine(160);
    float throughput_bps = config_.getTheoreticalThroughput(data_mod, data_rate);
    ImGui::TextDisabled("~%.1f kbps", throughput_bps / 1000.0f);
    ImGui::SameLine(240);
    ImGui::TextDisabled("BER:");
    ImGui::SameLine();
    if (stats.ber > 0.0f && stats.ber < 1.0f) {
        ImVec4 ber_color = (stats.ber < 0.01f) ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) :
                           (stats.ber < 0.1f)  ? ImVec4(1.0f, 1.0f, 0.2f, 1.0f) :
                                                  ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
        ImGui::TextColored(ber_color, "%.1f%%", stats.ber * 100.0f);
    } else {
        ImGui::TextDisabled("--");
    }

    // Row 3: Frame stats
    ImGui::Text("TX:%d RX:%d", stats.frames_sent, stats.frames_received);
    if (stats.frames_failed > 0) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "(%d fail)", stats.frames_failed);
    }

    ImGui::EndChild();
}

void App::renderOperateTab() {
    // Calculate available height for layout
    float total_height = ImGui::GetContentRegionAvail().y;

    // ========================================
    // TOP SECTION: Connection Controls (compact)
    // ========================================

    // Virtual Station Simulator (only visible with -sim flag, collapsible)
    if (sim_ui_visible_) {
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.2f, 0.4f, 0.6f, 1.0f));
        if (ImGui::CollapsingHeader("Simulator", ImGuiTreeNodeFlags_None)) {
            ImGui::PopStyleColor();
            if (ImGui::Checkbox("Enable", &simulation_enabled_)) {
                guiLog("Simulation checkbox toggled: %d", simulation_enabled_);
                if (simulation_enabled_) {
                    if (radio_rx_enabled_) { stopRadioRx(); audio_.stopPlayback(); }
                    guiLog("Simulation ENABLED - virtual station: %s", virtual_callsign_.c_str());
                    rx_log_.push_back("[SIM] Simulation enabled - connect to '" + virtual_callsign_ + "'");
                    modem_.reset(); virtual_modem_->reset(); virtual_protocol_.reset();
                    if (options_.record_audio) {
                        recording_enabled_ = true; recorded_samples_.clear();
                        rx_log_.push_back("[REC] Recording enabled");
                    }
                    // Start audio thread for realistic sample delivery
                    startSimThread();
                } else {
                    // Stop audio thread first
                    stopSimThread();
                    rx_log_.push_back("[SIM] Simulation disabled");
                    if (recording_enabled_) {
                        recording_enabled_ = false;
                        if (!recorded_samples_.empty()) {
                            writeRecordingToFile();
                            rx_log_.push_back("[REC] Saved: " + options_.record_path);
                        }
                    }
                    modem_.reset();
                    if (audio_initialized_) {
                        audio_.openOutput(getOutputDeviceName());
                        audio_.startPlayback(); startRadioRx();
                    }
                }
                if (rx_log_.size() > MAX_RX_LOG) rx_log_.pop_front();
            }
            if (simulation_enabled_) {
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "'%s' active", virtual_callsign_.c_str());
                if (recording_enabled_) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[REC %.1fs]", recorded_samples_.size() / 48000.0f);
                }
                ImGui::SetNextItemWidth(120);
                ImGui::SliderFloat("SNR", &simulation_snr_db_, 5.0f, 40.0f, "%.0f dB");
            }
        } else {
            ImGui::PopStyleColor();
        }
    }

    // Audio initialization (only when not in simulation)
    if (!simulation_enabled_ && !audio_initialized_) {
        if (ImGui::Button("Initialize Audio", ImVec2(-1, 28))) {
            initAudio();
        }
        return;
    }

    // ========================================
    // Connection Row: Callsign input + buttons
    // ========================================
    bool has_callsign = strlen(settings_.callsign) >= 3;
    auto conn_state = protocol_.getState();

    // Status line
    if (!simulation_enabled_ && !radio_rx_enabled_) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "OFFLINE");
        ImGui::SameLine();
        if (ImGui::SmallButton("Start RX")) {
            if (!audio_initialized_) initAudio();
            audio_.openOutput(getOutputDeviceName());
            audio_.startPlayback();
            startRadioRx();
        }
    } else {
        ImVec4 state_color;
        const char* state_icon;
        switch (conn_state) {
            case protocol::ConnectionState::CONNECTED:
                state_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
                state_icon = "CONNECTED";
                break;
            case protocol::ConnectionState::CONNECTING:
                state_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f);
                state_icon = "CONNECTING...";
                break;
            case protocol::ConnectionState::DISCONNECTING:
                state_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f);
                state_icon = "DISCONNECTING...";
                break;
            default:
                state_color = ImVec4(0.3f, 0.8f, 1.0f, 1.0f);
                state_icon = simulation_enabled_ ? "SIMULATION" : "LISTENING";
                break;
        }
        ImGui::TextColored(state_color, "%s", state_icon);
        if (conn_state == protocol::ConnectionState::CONNECTED) {
            ImGui::SameLine();
            ImGui::Text("to %s", protocol_.getRemoteCallsign().c_str());
        }
    }

    // My callsign
    if (has_callsign) {
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 100);
        ImGui::TextDisabled("My: %s", settings_.callsign);
    }

    // Connect to row
    ImGui::SetNextItemWidth(120);
    ImGui::InputText("##remotecall", remote_callsign_, sizeof(remote_callsign_),
                     ImGuiInputTextFlags_CharsUppercase);
    ImGui::SameLine();

    float btn_w = 80;
    ImGui::BeginDisabled(conn_state != protocol::ConnectionState::DISCONNECTED ||
                         !has_callsign || strlen(remote_callsign_) < 3);
    if (ImGui::Button("Connect", ImVec2(btn_w, 0))) {
        guiLog("Connect clicked: simulation=%d, remote='%s'", simulation_enabled_, remote_callsign_);
        if (!simulation_enabled_ && !radio_rx_enabled_) {
            if (!audio_initialized_) initAudio();
            audio_.openOutput(getOutputDeviceName());
            audio_.startPlayback();
            startRadioRx();
        }
        protocol_.connect(remote_callsign_);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();

    ImGui::BeginDisabled(conn_state == protocol::ConnectionState::DISCONNECTED);
    if (ImGui::Button("Disconnect", ImVec2(btn_w, 0))) {
        protocol_.disconnect();
    }
    ImGui::EndDisabled();

    // Stop button for real audio
    if (!simulation_enabled_ && radio_rx_enabled_) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Stop RX")) {
            stopRadioRx();
            audio_.stopPlayback();
            audio_.closeOutput();
        }
    }

    // Incoming call notification
    if (!pending_incoming_call_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                           "Incoming from %s!", pending_incoming_call_.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Accept")) { protocol_.acceptCall(); pending_incoming_call_.clear(); }
        ImGui::SameLine();
        if (ImGui::SmallButton("Reject")) { protocol_.rejectCall(); pending_incoming_call_.clear(); }
    }

    // Audio level meter (compact, only when RX active)
    if (!simulation_enabled_ && radio_rx_enabled_) {
        float input_level = audio_.getInputLevel();
        float input_db = (input_level > 0.0001f) ? 20.0f * log10f(input_level) : -80.0f;
        float level_normalized = (input_db + 60.0f) / 60.0f;
        level_normalized = std::max(0.0f, std::min(1.0f, level_normalized));
        ImVec4 level_color = (level_normalized > 0.8f) ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) :
                             (level_normalized > 0.5f) ? ImVec4(1.0f, 1.0f, 0.3f, 1.0f) :
                                                         ImVec4(0.3f, 1.0f, 0.3f, 1.0f);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, level_color);
        ImGui::ProgressBar(level_normalized, ImVec2(100, 14), "");
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextDisabled("%.0fdB", input_db);
        if (modem_.isSynced()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "SIGNAL");
        }
    }

    ImGui::Separator();

    // ========================================
    // MESSAGE LOG (takes most of the space)
    // ========================================
    ImGui::Text("Message Log");
    ImGui::SameLine();
    if (ImGui::SmallButton("Clear")) rx_log_.clear();
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) {
        std::string all_log;
        for (const auto& msg : rx_log_) all_log += msg + "\n";
        ImGui::SetClipboardText(all_log.c_str());
    }
    ImGui::SameLine();
    auto mstats = modem_.getStats();
    ImGui::TextDisabled("TX:%d RX:%d", mstats.frames_sent, mstats.frames_received);

    // Calculate remaining height for message log (leave space for TX and file transfer)
    float bottom_section_height = 130;  // TX input + File transfer
    float log_height = ImGui::GetContentRegionAvail().y - bottom_section_height;
    if (log_height < 100) log_height = 100;  // Minimum height

    ImGui::BeginChild("RXLogRadio", ImVec2(-1, log_height), true);
    for (const auto& msg : rx_log_) {
        ImVec4 color(0.7f, 0.7f, 0.7f, 1.0f);
        if (msg.size() >= 4 && msg.substr(0, 4) == "[TX]") {
            color = ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
        } else if (msg.size() >= 3 && msg.substr(0, 3) == "[RX") {
            color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
        } else if (msg.size() >= 4 && msg.substr(0, 4) == "[SIM") {
            color = ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
        } else if (msg.size() >= 4 && msg.substr(0, 4) == "[SYS") {
            color = ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
        } else if (msg.find("[FAILED]") != std::string::npos) {
            color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", msg.c_str());
        ImGui::PopStyleColor();
    }
    if (!rx_log_.empty()) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();

    // ========================================
    // BOTTOM SECTION: TX Input + File Transfer
    // ========================================
    ImGui::Separator();

    // TX Message Input
    if (tx_in_progress_ && audio_.isTxQueueEmpty()) {
        tx_in_progress_ = false;
        if (!ptt_active_ && !simulation_enabled_) audio_.startCapture();
    }

    bool can_send = !tx_in_progress_ && strlen(tx_text_buffer_) > 0 &&
                    protocol_.isConnected() && protocol_.isReadyToSend();

    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 90);
    bool send = ImGui::InputText("##txinput", tx_text_buffer_, sizeof(tx_text_buffer_),
                                  ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();

    ImVec4 send_color = can_send ? ImVec4(0.3f, 0.6f, 0.3f, 1.0f) : ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, send_color);
    ImGui::BeginDisabled(!can_send);
    if (ImGui::Button("Send##msg", ImVec2(80, 0)) || (send && can_send)) {
        std::string text(tx_text_buffer_);
        if (protocol_.sendMessage(text)) {
            rx_log_.push_back("[TX] " + text);
            if (rx_log_.size() > MAX_RX_LOG) rx_log_.pop_front();
            tx_text_buffer_[0] = '\0';
        }
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor();

    // File Transfer (compact row)
    if (protocol_.isFileTransferInProgress()) {
        auto progress = protocol_.getFileProgress();
        ImGui::TextColored(progress.is_sending ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f) : ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
            "%s: %s", progress.is_sending ? "TX" : "RX", progress.filename.c_str());
        ImGui::SameLine();
        ImGui::ProgressBar(progress.percentage() / 100.0f, ImVec2(100, 16));
        ImGui::SameLine();
        ImGui::Text("%.0f%%", progress.percentage());
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) protocol_.cancelFileTransfer();
    } else {
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 160);
        ImGui::InputText("##filepath", file_path_buffer_, sizeof(file_path_buffer_));
        ImGui::SameLine();
        if (ImGui::Button("Browse", ImVec2(60, 0))) {
            file_browser_.setTitle("Select File");
            file_browser_.open();
        }
        ImGui::SameLine();
        bool can_send_file = protocol_.isConnected() && protocol_.isReadyToSend() &&
                             strlen(file_path_buffer_) > 0;
        ImGui::BeginDisabled(!can_send_file);
        if (ImGui::Button("Send##file", ImVec2(60, 0))) {
            if (protocol_.sendFile(file_path_buffer_)) {
                rx_log_.push_back("[FILE] Sending: " + std::string(file_path_buffer_));
            } else {
                rx_log_.push_back("[FILE] Failed to start transfer");
            }
            if (rx_log_.size() > MAX_RX_LOG) rx_log_.pop_front();
        }
        ImGui::EndDisabled();
    }
}

} // namespace gui
} // namespace ultra
