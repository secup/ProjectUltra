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
// ALL logging (including modem, protocol, etc.) goes to this file
static FILE* g_gui_log_file = nullptr;
static bool g_log_initialized = false;

static void initLog() {
    if (g_log_initialized) return;
    g_log_initialized = true;

    // Create logs directory next to binary
    MKDIR("logs");
    g_gui_log_file = fopen("logs/gui.log", "w");

    if (g_gui_log_file) {
        // Redirect ALL logging (modem, protocol, etc.) to this file
        ultra::setLogFile(g_gui_log_file);
    }
}

static void guiLog(const char* fmt, ...) {
    initLog();
    if (!g_gui_log_file) return;

    // Use the same timestamp format as the global logger
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - ultra::g_log_start_time).count();
    int secs = static_cast<int>(elapsed / 1000);
    int ms = static_cast<int>(elapsed % 1000);

    // Format message
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    fprintf(g_gui_log_file, "[%3d.%03d][INFO ][GUI  ] %s\n", secs, ms, buf);
    fflush(g_gui_log_file);
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
                // Add PTT noise once at start of transmission (100-300ms)
                std::uniform_int_distribution<size_t> ptt_dist(4800, 14400);
                size_t ptt_samples = ptt_dist(sim_rng_);

                float typical_rms = 0.1f;
                float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
                float noise_power = (typical_rms * typical_rms) / snr_linear;
                float noise_stddev = std::sqrt(noise_power);
                std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

                std::vector<float> ptt_noise(ptt_samples);
                for (float& s : ptt_noise) s = noise_dist(sim_rng_);

                // Mark TX active (include PTT noise in duration)
                size_t total_samples = ptt_samples + samples.size();
                size_t tx_duration_ms = (total_samples * 1000) / 48000;
                tx_in_progress_ = true;
                tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

                // Queue PTT noise + signal for real-time streaming
                std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
                our_tx_pending_.insert(our_tx_pending_.end(), ptt_noise.begin(), ptt_noise.end());
                our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
                guiLog("SIM: Queued %zu TX samples (+ %zu PTT noise) for streaming", samples.size(), ptt_samples);
            } else {
                // Normal mode: send to real audio device (it streams at 48kHz)
                size_t tx_duration_ms = (samples.size() * 1000) / 48000;
                tx_in_progress_ = true;
                tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);
                waterfall_.addSamples(samples.data(), samples.size());
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
        // Stay "connected" during DISCONNECTING so we can receive the ACK via OFDM
        bool modem_connected = (state == protocol::ConnectionState::CONNECTED ||
                                state == protocol::ConnectionState::DISCONNECTING);
        modem_.setConnected(modem_connected);

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
                // Reset connect waveform to DPSK for next connection attempt
                modem_.setConnectWaveform(protocol::WaveformMode::DPSK);
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
    // Stop simulator threads first
    stopSimThreads();

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

    // Virtual station TX → queue for real-time streaming → our RX
    virtual_protocol_.setTxDataCallback([this](const Bytes& data) {
        guiLog("SIM: Virtual station TX %zu bytes", data.size());
        auto samples = virtual_modem_->transmit(data);

        // Add PTT noise once at start of transmission (100-300ms)
        std::uniform_int_distribution<size_t> ptt_dist(4800, 14400);
        size_t ptt_samples = ptt_dist(sim_rng_);

        float typical_rms = 0.1f;
        float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
        float noise_power = (typical_rms * typical_rms) / snr_linear;
        float noise_stddev = std::sqrt(noise_power);
        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        std::vector<float> ptt_noise(ptt_samples);
        for (float& s : ptt_noise) s = noise_dist(sim_rng_);

        guiLog("SIM: Virtual modem produced %zu samples (+ %zu PTT noise), queuing for stream", samples.size(), ptt_samples);

        // Queue PTT noise + signal for real-time streaming
        std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
        virtual_tx_pending_.insert(virtual_tx_pending_.end(), ptt_noise.begin(), ptt_noise.end());
        virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
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

// ========================================
// Threaded Simulation (matches threaded_simulator.cpp)
// ========================================

std::vector<float> App::applyChannelEffects(const std::vector<float>& samples) {
    // Only adds AWGN - PTT noise is added separately at TX start
    if (samples.empty()) return samples;

    std::vector<float> result = samples;

    // Apply AWGN to signal
    if (simulation_snr_db_ < 50.0f) {
        float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);

        float sum_sq = 0.0f;
        for (float s : samples) sum_sq += s * s;
        float signal_rms = std::sqrt(sum_sq / samples.size());

        if (signal_rms > 1e-6f) {
            float signal_power = signal_rms * signal_rms;
            float noise_power = signal_power / snr_linear;
            float noise_stddev = std::sqrt(noise_power);
            std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

            for (float& s : result) {
                s += noise_dist(sim_rng_);
            }
        }
    }

    return result;
}

void App::writeToVirtualChannel(const std::vector<float>& samples) {
    auto noisy = applyChannelEffects(samples);
    std::lock_guard<std::mutex> lock(channel_to_virtual_mutex_);
    channel_to_virtual_.insert(channel_to_virtual_.end(), noisy.begin(), noisy.end());
    guiLog("SIM: Our TX -> virtual channel (%zu samples)", noisy.size());
}

void App::writeToOurChannel(const std::vector<float>& samples) {
    auto noisy = applyChannelEffects(samples);
    std::lock_guard<std::mutex> lock(channel_to_us_mutex_);
    channel_to_us_.insert(channel_to_us_.end(), noisy.begin(), noisy.end());
    guiLog("SIM: Virtual TX -> our channel (%zu samples)", noisy.size());
}

void App::startSimThreads() {
    if (sim_thread_running_) return;

    guiLog("SIM: Starting simulation threads");
    sim_thread_running_ = true;

    // Start TX threads (stream audio at 48kHz like real radio)
    sim_our_tx_thread_ = std::thread(&App::ourTxLoop, this);
    sim_virtual_tx_thread_ = std::thread(&App::virtualTxLoop, this);

    // Start RX threads (continuous audio processing like real radios)
    sim_our_rx_thread_ = std::thread(&App::ourRxLoop, this);
    sim_virtual_rx_thread_ = std::thread(&App::virtualRxLoop, this);
    sim_virtual_protocol_thread_ = std::thread(&App::virtualProtocolLoop, this);

    guiLog("SIM: All threads started");
}

void App::stopSimThreads() {
    if (!sim_thread_running_) return;

    guiLog("SIM: Stopping simulation threads");
    sim_thread_running_ = false;

    // Join all threads
    if (sim_our_tx_thread_.joinable()) sim_our_tx_thread_.join();
    if (sim_virtual_tx_thread_.joinable()) sim_virtual_tx_thread_.join();
    if (sim_our_rx_thread_.joinable()) sim_our_rx_thread_.join();
    if (sim_virtual_rx_thread_.joinable()) sim_virtual_rx_thread_.join();
    if (sim_virtual_protocol_thread_.joinable()) sim_virtual_protocol_thread_.join();

    // Clear all buffers
    {
        std::lock_guard<std::mutex> lock(channel_to_virtual_mutex_);
        channel_to_virtual_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(channel_to_us_mutex_);
        channel_to_us_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
        our_tx_pending_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
        virtual_tx_pending_.clear();
    }

    guiLog("SIM: All threads stopped");
}

void App::ourRxLoop() {
    guiLog("SIM: Our RX thread started");
    constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz

    while (sim_thread_running_) {
        std::vector<float> chunk;

        // Read from channel
        {
            std::lock_guard<std::mutex> lock(channel_to_us_mutex_);
            if (channel_to_us_.size() >= CHUNK_SIZE) {
                chunk.assign(channel_to_us_.begin(), channel_to_us_.begin() + CHUNK_SIZE);
                channel_to_us_.erase(channel_to_us_.begin(), channel_to_us_.begin() + CHUNK_SIZE);
            }
        }

        // Check if TX is still active
        if (tx_in_progress_ && std::chrono::steady_clock::now() >= tx_end_time_) {
            tx_in_progress_ = false;  // TX finished
        }

        if (!chunk.empty()) {
            // Record if enabled
            if (recording_enabled_) {
                recorded_samples_.insert(recorded_samples_.end(), chunk.begin(), chunk.end());
            }

            // Show on waterfall only when not transmitting (TX signal takes priority)
            if (!tx_in_progress_) {
                waterfall_.addSamples(chunk.data(), chunk.size());
            }

            // Feed to our modem (always process RX even during TX display)
            modem_.receiveAudio(chunk);
            modem_.pollRxAudio();
        } else {
            // No RX data - generate noise floor for waterfall (like real radio)
            if (!tx_in_progress_) {
                // Generate noise at current SNR level
                float typical_rms = 0.1f;
                float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
                float noise_power = (typical_rms * typical_rms) / snr_linear;
                float noise_stddev = std::sqrt(noise_power);
                std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

                std::vector<float> noise(CHUNK_SIZE);
                for (float& s : noise) s = noise_dist(sim_rng_);
                waterfall_.addSamples(noise.data(), noise.size());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    guiLog("SIM: Our RX thread stopped");
}

void App::ourTxLoop() {
    guiLog("SIM: Our TX thread started");
    constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz
    auto last_time = std::chrono::steady_clock::now();

    while (sim_thread_running_) {
        // Wait for real-time (10ms chunks at 48kHz)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        if (elapsed < 10000) {  // 10ms = 10000us
            std::this_thread::sleep_for(std::chrono::microseconds(10000 - elapsed));
            now = std::chrono::steady_clock::now();
        }
        last_time = now;

        std::vector<float> chunk;

        // Get pending TX samples
        {
            std::lock_guard<std::mutex> lock(our_tx_pending_mutex_);
            if (our_tx_pending_.size() >= CHUNK_SIZE) {
                chunk.assign(our_tx_pending_.begin(), our_tx_pending_.begin() + CHUNK_SIZE);
                our_tx_pending_.erase(our_tx_pending_.begin(), our_tx_pending_.begin() + CHUNK_SIZE);
            } else if (!our_tx_pending_.empty()) {
                // Send remaining samples (end of transmission)
                chunk = std::move(our_tx_pending_);
                our_tx_pending_.clear();
            }
        }

        if (!chunk.empty()) {
            // Show on waterfall (real-time, like actual TX)
            waterfall_.addSamples(chunk.data(), chunk.size());

            // Send through channel to virtual station
            writeToVirtualChannel(chunk);
        }
    }

    guiLog("SIM: Our TX thread stopped");
}

void App::virtualRxLoop() {
    guiLog("SIM: Virtual RX thread started");
    constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz

    while (sim_thread_running_) {
        std::vector<float> chunk;

        // Read from channel
        {
            std::lock_guard<std::mutex> lock(channel_to_virtual_mutex_);
            if (channel_to_virtual_.size() >= CHUNK_SIZE) {
                chunk.assign(channel_to_virtual_.begin(), channel_to_virtual_.begin() + CHUNK_SIZE);
                channel_to_virtual_.erase(channel_to_virtual_.begin(), channel_to_virtual_.begin() + CHUNK_SIZE);
            }
        }

        if (!chunk.empty()) {
            // Record if enabled
            if (recording_enabled_) {
                recorded_samples_.insert(recorded_samples_.end(), chunk.begin(), chunk.end());
            }

            // Feed to virtual modem
            virtual_modem_->receiveAudio(chunk);
            virtual_modem_->pollRxAudio();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    guiLog("SIM: Virtual RX thread stopped");
}

void App::virtualTxLoop() {
    guiLog("SIM: Virtual TX thread started");
    constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz
    auto last_time = std::chrono::steady_clock::now();

    while (sim_thread_running_) {
        // Wait for real-time (10ms chunks at 48kHz)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        if (elapsed < 10000) {  // 10ms = 10000us
            std::this_thread::sleep_for(std::chrono::microseconds(10000 - elapsed));
            now = std::chrono::steady_clock::now();
        }
        last_time = now;

        std::vector<float> chunk;

        // Get pending TX samples from virtual station
        {
            std::lock_guard<std::mutex> lock(virtual_tx_pending_mutex_);
            if (virtual_tx_pending_.size() >= CHUNK_SIZE) {
                chunk.assign(virtual_tx_pending_.begin(), virtual_tx_pending_.begin() + CHUNK_SIZE);
                virtual_tx_pending_.erase(virtual_tx_pending_.begin(), virtual_tx_pending_.begin() + CHUNK_SIZE);
            } else if (!virtual_tx_pending_.empty()) {
                // Send remaining samples (end of transmission)
                chunk = std::move(virtual_tx_pending_);
                virtual_tx_pending_.clear();
            }
        }

        if (!chunk.empty()) {
            // Send through channel to us
            writeToOurChannel(chunk);
        }
    }

    guiLog("SIM: Virtual TX thread stopped");
}

void App::virtualProtocolLoop() {
    guiLog("SIM: Virtual protocol thread started");
    auto last_tick = std::chrono::steady_clock::now();

    while (sim_thread_running_) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick).count();

        if (elapsed >= 16) {  // ~60Hz tick rate
            virtual_protocol_.tick(elapsed);
            last_tick = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    guiLog("SIM: Virtual protocol thread stopped");
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

    // Protocol engine tick (our protocol always ticks in main thread for UI responsiveness)
    // Virtual station's protocol ticks in its own thread (virtualProtocolLoop)
    uint32_t now = SDL_GetTicks();
    uint32_t elapsed = (last_tick_time_ == 0) ? 0 : (now - last_tick_time_);
    last_tick_time_ = now;

    if (elapsed > 0 && elapsed < 1000) {
        protocol_.tick(elapsed);
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
    // In simulation mode, use the slider SNR (that's the actual channel quality)
    if (simulation_enabled_) {
        modem_stats.snr_db = simulation_snr_db_;
    }
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

// Helper function to get implied channel condition from negotiated mode
// (what channel the remote station measured to choose this mode)
static const char* getModeImpliedQuality(Modulation mod, CodeRate rate, ImVec4& color) {
    // Based on mode selection thresholds - using standard HF terms
    // High rates = AWGN-like, Low rates = Poor channel
    if (mod == Modulation::QAM16) {
        if (rate == CodeRate::R3_4 || rate == CodeRate::R5_6) {
            color = ImVec4(0.0f, 1.0f, 0.5f, 1.0f);  // Cyan
            return "AWGN";  // Lab-like conditions
        } else {
            color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
            return "Good";
        }
    } else if (mod == Modulation::DQPSK || mod == Modulation::QPSK) {
        if (rate == CodeRate::R2_3) {
            color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
            return "Good";
        } else if (rate == CodeRate::R1_2) {
            color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
            return "Moderate";
        } else {
            color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
            return "Poor";
        }
    } else {
        color = ImVec4(1.0f, 0.3f, 0.0f, 1.0f);  // Red
        return "Very Poor";
    }
}

// Helper function to classify channel quality based on SNR (standard HF terms)
static const char* getChannelQuality(float snr_db, ImVec4& color) {
    if (snr_db >= 30.0f) {
        color = ImVec4(0.0f, 1.0f, 0.5f, 1.0f);  // Cyan
        return "AWGN";  // Lab-like, ideal conditions
    } else if (snr_db >= 20.0f) {
        color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        return "Good";
    } else if (snr_db >= 15.0f) {
        color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        return "Moderate";
    } else if (snr_db >= 10.0f) {
        color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
        return "Poor";
    } else if (snr_db >= 0.0f) {
        color = ImVec4(1.0f, 0.3f, 0.0f, 1.0f);  // Red-orange
        return "Very Poor";
    } else {
        color = ImVec4(1.0f, 0.1f, 0.1f, 1.0f);  // Red
        return "Very Poor";
    }
}

void App::renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate) {
    // Compact horizontal Channel Status display
    ImGui::BeginChild("ChannelStatus", ImVec2(0, 110), false);

    auto conn_state = protocol_.getState();

    // Row 1: Connection state + Channel Quality (only when connected) + SNR bar
    if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        // Not connected - show idle state
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "IDLE");
        ImGui::SameLine();
        ImGui::TextDisabled("[Standby]");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        // Empty SNR bar
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.3f, 0.3f, 0.3f, 1.0f));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(0.0f, ImVec2(-1, 16), "-- dB");
        ImGui::PopStyleColor();
    } else if (conn_state == protocol::ConnectionState::CONNECTING) {
        // Connecting - show our outgoing mode, no channel quality yet
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "CALL");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[Connecting...]");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        // Animated/pulsing bar to show activity
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(0.3f, ImVec2(-1, 16), "awaiting...");
        ImGui::PopStyleColor();
    } else if (conn_state == protocol::ConnectionState::DISCONNECTING) {
        // Disconnecting
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "DISC");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "[Disconnecting...]");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(1.0f, 0.5f, 0.2f, 1.0f));
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(0.5f, ImVec2(-1, 16), "closing...");
        ImGui::PopStyleColor();
    } else {
        // CONNECTED - show remote mode (their view) AND our SNR (our view)
        // Row 1: Remote's negotiated mode + implied channel condition
        auto waveform = modem_.getWaveformMode();
        const char* wf_str = protocol::waveformModeToString(waveform);
        ImVec4 wf_color = (waveform == protocol::WaveformMode::OFDM) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (waveform == protocol::WaveformMode::DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (waveform == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                       ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::Text("RX:");
        ImGui::SameLine();
        ImGui::TextColored(wf_color, "%s", wf_str);
        ImGui::SameLine();
        ImGui::Text("%s %s", modulationToString(data_mod), codeRateToString(data_rate));
        ImGui::SameLine();
        // Show implied channel condition from their mode choice
        ImVec4 mode_quality_color;
        const char* mode_quality = getModeImpliedQuality(data_mod, data_rate, mode_quality_color);
        ImGui::TextColored(mode_quality_color, "[%s]", mode_quality);
        ImGui::SameLine();
        float throughput_bps = config_.getTheoreticalThroughput(data_mod, data_rate);
        ImGui::TextDisabled("~%.1f kbps", throughput_bps / 1000.0f);

        // Row 2: Our SNR measurement
        ImVec4 sync_color = stats.synced ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        ImGui::TextColored(sync_color, "%s", stats.synced ? "SYNC" : "----");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();
        ImVec4 quality_color;
        const char* quality_str = getChannelQuality(stats.snr_db, quality_color);
        ImGui::TextColored(quality_color, "[%s]", quality_str);
        ImGui::SameLine();

        // SNR bar
        float snr_normalized = stats.snr_db / 40.0f;
        snr_normalized = std::max(0.0f, std::min(1.0f, snr_normalized));
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, quality_color);
        char snr_text[16];
        snprintf(snr_text, sizeof(snr_text), "%.1f dB", stats.snr_db);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(snr_normalized, ImVec2(-1, 16), snr_text);
        ImGui::PopStyleColor();
    }

    // Row 2: Waveform + Mode (for non-connected states only)
    if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        // Show default connect waveform (DPSK R1/4)
        auto connect_wf = protocol_.getConnectWaveform();
        const char* wf_str = protocol::waveformModeToString(connect_wf);
        ImGui::TextDisabled("%s R1/4 (default)", wf_str);
    } else if (conn_state == protocol::ConnectionState::CONNECTING) {
        // Show actual waveform being used for connection attempt (always R1/4)
        auto connect_wf = protocol_.getConnectWaveform();
        const char* wf_str = protocol::waveformModeToString(connect_wf);
        ImVec4 wf_color = (connect_wf == protocol::WaveformMode::OFDM) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (connect_wf == protocol::WaveformMode::DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (connect_wf == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                         ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::TextColored(wf_color, "%s R1/4 (calling)", wf_str);
    }
    // Connected state already shows mode in Row 1

    // Row 3: Frame stats (only show when we have activity)
    if (stats.frames_sent > 0 || stats.frames_received > 0) {
        ImGui::Text("TX:%d RX:%d", stats.frames_sent, stats.frames_received);
        if (stats.frames_failed > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "(%d fail)", stats.frames_failed);
        }
    } else if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        ImGui::TextDisabled("Ready to connect");
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
                    // Start simulation threads for realistic audio streaming
                    startSimThreads();
                } else {
                    // Stop simulation threads
                    stopSimThreads();
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
        guiLog("DISCONNECT BUTTON: Pressed, modem connected_=%d, waveform_mode_=%d",
               modem_.isConnected() ? 1 : 0, static_cast<int>(modem_.getWaveformMode()));
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
