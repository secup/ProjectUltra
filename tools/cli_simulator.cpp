/**
 * CLI Simulator - EXACT replica of GUI test mode
 *
 * This replicates the EXACT behavior of the GUI simulator:
 * - Same audio streaming rate (48kHz)
 * - Same PTT delays (200ms)
 * - Same half-duplex logic
 * - Same channel simulation
 * - Same timing for protocol tick
 *
 * Usage:
 *   ./cli_simulator [options]
 *
 * Options:
 *   -snr <dB>      Set channel SNR (default: 20)
 *   -msg <text>    Message to send (default: "Hello World!")
 *   -verbose       Enable verbose logging
 *   -timeout <ms>  Max simulation time in ms (default: 60000)
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <random>
#include <functional>
#include <queue>
#include <cstdarg>

#include "gui/modem_engine.hpp"
#include "protocol/protocol_engine.hpp"
#include "ultra/logging.hpp"

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::protocol;

// ============================================================================
// CLI Simulator - Exact replica of GUI App simulation
// ============================================================================

class CLISimulator {
public:
    CLISimulator() {
        initOurStation();
        initVirtualStation();
    }

    void setVerbose(bool v) { verbose_ = v; }
    void setSNR(float snr) { simulation_snr_db_ = snr; }
    void setMessage(const std::string& msg) { test_message_ = msg; }
    void setTimeout(uint32_t ms) { timeout_ms_ = ms; }
    void setWaveform(const std::string& wf) {
        if (wf == "mfsk" || wf == "MFSK") {
            initial_waveform_ = WaveformMode::MFSK;
        } else if (wf == "dpsk" || wf == "DPSK") {
            initial_waveform_ = WaveformMode::DPSK;
        }
    }

    // Run the full test sequence
    bool run() {
        printHeader();

        // Phase 1: Connect
        log("\n=== PHASE 1: CONNECTION ===\n");
        if (!runConnect()) {
            logError("Connection failed!");
            return false;
        }
        logSuccess("Connection established!");

        // Wait for mode negotiation to settle
        log("\n=== PHASE 2: MODE NEGOTIATION ===\n");
        runTicks(60000);  // 60 seconds for mode negotiation (DPSK frames are slow)
        logSuccess("Mode negotiation complete!");
        printModeStatus();

        // Phase 3: Send message
        log("\n=== PHASE 3: DATA TRANSFER ===\n");
        if (!runDataTransfer()) {
            logError("Data transfer failed!");
            return false;
        }
        logSuccess("Data transfer complete!");

        // Phase 4: Disconnect
        log("\n=== PHASE 4: DISCONNECT ===\n");
        if (!runDisconnect()) {
            logError("Disconnect failed!");
            return false;
        }
        logSuccess("Disconnect complete!");

        printSummary();
        return true;
    }

private:
    // === Our station (like the local user in GUI) ===
    ModemEngine modem_;
    ProtocolEngine protocol_{ConnectionConfig{}};

    // === Virtual station (like SIM in GUI) ===
    std::unique_ptr<ModemEngine> virtual_modem_;
    ProtocolEngine virtual_protocol_{ConnectionConfig{}};

    // === Simulation state (EXACT copy from App) ===
    float simulation_snr_db_ = 20.0f;
    std::mt19937 sim_rng_{42};

    // TX/RX queues (EXACT copy from App)
    std::vector<float> sim_our_tx_queue_;
    std::vector<float> sim_our_streaming_;
    std::vector<float> sim_virtual_tx_queue_;
    std::vector<float> sim_virtual_streaming_;

    // PTT turnaround delay (ms remaining before TX can start)
    uint32_t sim_our_ptt_delay_ = 0;
    uint32_t sim_virtual_ptt_delay_ = 0;
    static constexpr uint32_t PTT_DELAY_MS = 200;  // EXACT same as GUI

    // State tracking
    bool our_connected_ = false;
    bool virtual_connected_ = false;
    bool message_received_ = false;
    std::string received_message_;
    Modulation our_data_modulation_ = Modulation::DQPSK;
    CodeRate our_data_rate_ = CodeRate::R1_4;
    Modulation virtual_data_modulation_ = Modulation::DQPSK;
    CodeRate virtual_data_rate_ = CodeRate::R1_4;

    // Test parameters
    std::string test_message_ = "Hello World! This is a test message.";
    bool verbose_ = false;
    uint32_t timeout_ms_ = 60000;
    WaveformMode initial_waveform_ = WaveformMode::DPSK;  // Default DPSK, can be set to MFSK

    // Simulated time
    uint32_t simulated_time_ms_ = 0;

    // === Initialization (EXACT copy from App) ===

    void initOurStation() {
        modem_.setLogPrefix("OUR");
        protocol_.setLocalCallsign("ALPHA");
        protocol_.setAutoAccept(true);

        // TX callback: protocol -> modem -> tx_queue
        protocol_.setTxDataCallback([this](const Bytes& data) {
            if (verbose_) log("  [OUR] Protocol TX %zu bytes", data.size());
            auto samples = modem_.transmit(data);
            sim_our_tx_queue_.insert(sim_our_tx_queue_.end(), samples.begin(), samples.end());
        });

        // RX callback: modem -> protocol
        modem_.setRawDataCallback([this](const Bytes& data) {
            if (verbose_) log("  [OUR] Modem RX %zu bytes", data.size());
            // EXACT same as GUI: use simulation SNR
            protocol_.setMeasuredSNR(simulation_snr_db_);
            protocol_.onRxData(data);
        });

        // Connection state callback
        protocol_.setConnectionChangedCallback([this](ConnectionState state, const std::string& info) {
            if (state == ConnectionState::CONNECTED) {
                our_connected_ = true;
                modem_.setConnected(true);
                log("  [ALPHA] CONNECTED to %s", info.c_str());
            } else if (state == ConnectionState::DISCONNECTED) {
                our_connected_ = false;
                modem_.setConnected(false);
                modem_.setWaveformMode(WaveformMode::OFDM);
                log("  [ALPHA] DISCONNECTED: %s", info.c_str());
            }
        });

        // Mode change callbacks
        protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr) {
            our_data_modulation_ = mod;
            our_data_rate_ = rate;
            modem_.setDataMode(mod, rate);
            log("  [ALPHA] MODE -> %s %s (SNR=%.1f dB)",
                modulationToString(mod), codeRateToString(rate), snr);
        });

        protocol_.setModeNegotiatedCallback([this](WaveformMode mode) {
            modem_.setWaveformMode(mode);
            log("  [ALPHA] WAVEFORM -> %s", waveformModeToString(mode));
        });

        protocol_.setConnectWaveformChangedCallback([this](WaveformMode mode) {
            modem_.setConnectWaveform(mode);
        });

        protocol_.setHandshakeConfirmedCallback([this]() {
            modem_.setHandshakeComplete(true);
            if (verbose_) log("  [ALPHA] Handshake confirmed");
        });

        protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
            log("  [ALPHA] Message from %s: \"%s\"", from.c_str(), text.c_str());
        });
    }

    void initVirtualStation() {
        // EXACT copy of App::initVirtualStation()
        virtual_modem_ = std::make_unique<ModemEngine>();
        virtual_modem_->setLogPrefix("SIM");
        virtual_protocol_.setLocalCallsign("BRAVO");
        virtual_protocol_.setAutoAccept(true);

        // Virtual station TX callback
        virtual_protocol_.setTxDataCallback([this](const Bytes& data) {
            if (verbose_) log("  [SIM] Protocol TX %zu bytes", data.size());
            auto samples = virtual_modem_->transmit(data);
            sim_virtual_tx_queue_.insert(sim_virtual_tx_queue_.end(), samples.begin(), samples.end());
        });

        // Virtual modem RX callback
        virtual_modem_->setRawDataCallback([this](const Bytes& data) {
            if (verbose_) log("  [SIM] Modem RX %zu bytes", data.size());
            // EXACT same as GUI: use simulation SNR
            virtual_protocol_.setMeasuredSNR(simulation_snr_db_);
            virtual_protocol_.onRxData(data);
        });

        // Virtual connection state callback
        virtual_protocol_.setConnectionChangedCallback([this](ConnectionState state, const std::string& info) {
            if (state == ConnectionState::CONNECTED) {
                virtual_connected_ = true;
                virtual_modem_->setConnected(true);
                log("  [BRAVO] CONNECTED to %s", info.c_str());
            } else if (state == ConnectionState::DISCONNECTED) {
                virtual_connected_ = false;
                virtual_modem_->setConnected(false);
                virtual_modem_->setWaveformMode(WaveformMode::OFDM);
                log("  [BRAVO] DISCONNECTED: %s", info.c_str());
            }
        });

        // Virtual mode change callbacks
        virtual_protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float snr) {
            virtual_data_modulation_ = mod;
            virtual_data_rate_ = rate;
            virtual_modem_->setDataMode(mod, rate);
            log("  [BRAVO] MODE -> %s %s (SNR=%.1f dB)",
                modulationToString(mod), codeRateToString(rate), snr);
        });

        virtual_protocol_.setModeNegotiatedCallback([this](WaveformMode mode) {
            virtual_modem_->setWaveformMode(mode);
            log("  [BRAVO] WAVEFORM -> %s", waveformModeToString(mode));
        });

        virtual_protocol_.setConnectWaveformChangedCallback([this](WaveformMode mode) {
            virtual_modem_->setConnectWaveform(mode);
        });

        virtual_protocol_.setHandshakeConfirmedCallback([this]() {
            virtual_modem_->setHandshakeComplete(true);
            if (verbose_) log("  [BRAVO] Handshake confirmed");
        });

        virtual_protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
            message_received_ = true;
            received_message_ = text;
            log("  [BRAVO] Message from %s: \"%s\"", from.c_str(), text.c_str());
        });
    }

    // === Channel simulation (EXACT copy from App) ===

    void applyChannelSimulation(std::vector<float>& samples) {
        if (samples.empty()) return;

        // Calculate signal RMS
        float sum_sq = 0.0f;
        for (float s : samples) {
            sum_sq += s * s;
        }
        float signal_rms = std::sqrt(sum_sq / samples.size());

        if (signal_rms < 1e-6f) return;  // Silent buffer

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

    void prependPttNoise(std::vector<float>& samples) {
        // EXACT copy from App::prependPttNoise()
        std::uniform_int_distribution<size_t> delay_dist(4800, 24000);  // 100-500ms at 48kHz
        size_t noise_samples = delay_dist(sim_rng_);

        float typical_signal_rms = 0.1f;
        float snr_linear = std::pow(10.0f, simulation_snr_db_ / 10.0f);
        float noise_power = (typical_signal_rms * typical_signal_rms) / snr_linear;
        float noise_stddev = std::sqrt(noise_power);

        std::normal_distribution<float> noise_dist(0.0f, noise_stddev);

        std::vector<float> noise_buffer(noise_samples);
        for (size_t i = 0; i < noise_samples; ++i) {
            noise_buffer[i] = noise_dist(sim_rng_);
        }

        samples.insert(samples.begin(), noise_buffer.begin(), noise_buffer.end());

        if (verbose_) {
            log("  [SIM] Prepended %zu samples (%.0fms) of PTT noise",
                noise_samples, noise_samples * 1000.0f / 48000.0f);
        }
    }

    // === Simulation tick (EXACT copy from App::tickSimulation) ===

    void tickSimulation(uint32_t elapsed_ms) {
        // Audio rate: 48 samples per millisecond
        const size_t samples_per_ms = 48;
        const size_t samples_this_tick = elapsed_ms * samples_per_ms;

        // Update PTT delays
        if (sim_our_ptt_delay_ > 0) {
            sim_our_ptt_delay_ = (elapsed_ms >= sim_our_ptt_delay_) ? 0 : sim_our_ptt_delay_ - elapsed_ms;
        }
        if (sim_virtual_ptt_delay_ > 0) {
            sim_virtual_ptt_delay_ = (elapsed_ms >= sim_virtual_ptt_delay_) ? 0 : sim_virtual_ptt_delay_ - elapsed_ms;
        }

        // === Move pending TX to streaming buffers (with PTT delay) ===

        // Our TX → streaming to virtual
        if (!sim_our_tx_queue_.empty() && sim_our_streaming_.empty() && sim_our_ptt_delay_ == 0) {
            if (verbose_) {
                log("  [SIM] Starting TX -> virtual (%zu samples, %.1fs)",
                    sim_our_tx_queue_.size(), sim_our_tx_queue_.size() / 48000.0f);
            }

            prependPttNoise(sim_our_tx_queue_);
            applyChannelSimulation(sim_our_tx_queue_);

            sim_our_streaming_ = std::move(sim_our_tx_queue_);
            sim_our_tx_queue_.clear();

            // Virtual can't TX until we're done (half-duplex)
            sim_virtual_ptt_delay_ = PTT_DELAY_MS;
        }

        // Virtual's TX → streaming to us
        if (!sim_virtual_tx_queue_.empty() && sim_virtual_streaming_.empty() && sim_virtual_ptt_delay_ == 0) {
            if (verbose_) {
                log("  [SIM] Starting TX <- virtual (%zu samples, %.1fs)",
                    sim_virtual_tx_queue_.size(), sim_virtual_tx_queue_.size() / 48000.0f);
            }

            prependPttNoise(sim_virtual_tx_queue_);
            applyChannelSimulation(sim_virtual_tx_queue_);

            sim_virtual_streaming_ = std::move(sim_virtual_tx_queue_);
            sim_virtual_tx_queue_.clear();

            // We can't TX until virtual is done (half-duplex)
            sim_our_ptt_delay_ = PTT_DELAY_MS;
        }

        // === Stream samples at audio rate (48kHz) ===

        // Stream our samples → virtual modem
        if (!sim_our_streaming_.empty()) {
            size_t to_send = std::min(samples_this_tick, sim_our_streaming_.size());
            if (to_send > 0) {
                std::vector<float> chunk(sim_our_streaming_.begin(),
                                         sim_our_streaming_.begin() + to_send);
                sim_our_streaming_.erase(sim_our_streaming_.begin(),
                                         sim_our_streaming_.begin() + to_send);

                // Feed to virtual modem at audio rate
                virtual_modem_->receiveAudio(chunk);
                virtual_modem_->pollRxAudio();
            }

            // TX complete
            if (sim_our_streaming_.empty()) {
                if (verbose_) log("  [SIM] TX -> virtual complete");
                sim_virtual_ptt_delay_ = PTT_DELAY_MS;  // Turnaround delay
            }
        }

        // Stream virtual's samples → our modem
        if (!sim_virtual_streaming_.empty()) {
            size_t to_send = std::min(samples_this_tick, sim_virtual_streaming_.size());
            if (to_send > 0) {
                std::vector<float> chunk(sim_virtual_streaming_.begin(),
                                         sim_virtual_streaming_.begin() + to_send);
                sim_virtual_streaming_.erase(sim_virtual_streaming_.begin(),
                                             sim_virtual_streaming_.begin() + to_send);

                // Feed to our modem at audio rate
                modem_.receiveAudio(chunk);
                modem_.pollRxAudio();
            }

            // TX complete
            if (sim_virtual_streaming_.empty()) {
                if (verbose_) log("  [SIM] TX <- virtual complete");
                sim_our_ptt_delay_ = PTT_DELAY_MS;  // Turnaround delay
            }
        }

        // === Always poll modems to process buffered samples ===
        virtual_modem_->pollRxAudio();
        modem_.pollRxAudio();

        // === Tick protocol engines ===
        virtual_protocol_.tick(elapsed_ms);
        protocol_.tick(elapsed_ms);
    }

    // === Test phases ===

    void runTicks(uint32_t duration_ms) {
        // Run simulation for duration_ms with ~16ms ticks (like 60fps GUI)
        const uint32_t tick_ms = 16;
        uint32_t end_time = simulated_time_ms_ + duration_ms;

        while (simulated_time_ms_ < end_time) {
            tickSimulation(tick_ms);
            simulated_time_ms_ += tick_ms;
        }
    }

    bool runConnect() {
        // Set the initial waveform for connection attempt (both protocol and modem)
        protocol_.setInitialConnectWaveform(initial_waveform_);
        modem_.setConnectWaveform(initial_waveform_);
        log("  ALPHA initiating connection to BRAVO via %s...",
            waveformModeToString(initial_waveform_));
        protocol_.connect("BRAVO");

        uint32_t start_time = simulated_time_ms_;
        const uint32_t tick_ms = 16;

        // DPSK frames take ~15.5s each at 62.5 baud, need 60s+ for connection
        while (simulated_time_ms_ - start_time < 90000) {  // 90 second timeout
            tickSimulation(tick_ms);
            simulated_time_ms_ += tick_ms;

            if (our_connected_ && virtual_connected_) {
                return true;
            }
        }

        return false;
    }

    bool runDataTransfer() {
        // Send multiple messages like GUI test does
        std::vector<std::string> messages = {test_message_, "Second message", "Third message"};

        for (size_t msg_idx = 0; msg_idx < messages.size(); msg_idx++) {
            const std::string& msg = messages[msg_idx];
            message_received_ = false;
            received_message_.clear();

            // Wait for ARQ to be ready (ACK from previous message received)
            uint32_t wait_start = simulated_time_ms_;
            const uint32_t tick_ms = 16;
            while (!protocol_.isReadyToSend() && simulated_time_ms_ - wait_start < 90000) {
                tickSimulation(tick_ms);
                simulated_time_ms_ += tick_ms;
            }
            if (!protocol_.isReadyToSend()) {
                logError("ARQ not ready to send (timeout waiting for ACK)");
                return false;
            }

            log("  ALPHA sending message %zu: \"%s\"", msg_idx + 1, msg.c_str());
            protocol_.sendMessage(msg);

            uint32_t start_time = simulated_time_ms_;

            // Data frames may use faster OFDM after mode negotiation, but still need decent timeout
            bool received = false;
            while (simulated_time_ms_ - start_time < 90000) {  // 90 second timeout
                tickSimulation(tick_ms);
                simulated_time_ms_ += tick_ms;

                if (message_received_) {
                    if (received_message_ == msg) {
                        log("  Message %zu received correctly!", msg_idx + 1);
                        received = true;
                        break;
                    } else {
                        logError("  Message corrupted!");
                        log("    Expected: \"%s\"", msg.c_str());
                        log("    Received: \"%s\"", received_message_.c_str());
                        return false;
                    }
                }
            }

            if (!received) {
                char errbuf[128];
                snprintf(errbuf, sizeof(errbuf), "Message %zu not received (timeout)", msg_idx + 1);
                logError(errbuf);
                return false;
            }
        }

        return true;
    }

    bool runDisconnect() {
        log("  ALPHA initiating disconnect...");
        protocol_.disconnect();

        uint32_t start_time = simulated_time_ms_;
        const uint32_t tick_ms = 16;

        while (simulated_time_ms_ - start_time < 60000) {  // 60 second timeout
            tickSimulation(tick_ms);
            simulated_time_ms_ += tick_ms;

            if (!our_connected_ && !virtual_connected_) {
                return true;
            }
        }

        return false;
    }

    // === Helpers ===

    const char* waveformModeToString(WaveformMode mode) {
        switch (mode) {
            case WaveformMode::OFDM: return "OFDM";
            case WaveformMode::DPSK: return "DPSK";
            case WaveformMode::MFSK: return "MFSK";
            case WaveformMode::OTFS_EQ: return "OTFS-EQ";
            case WaveformMode::OTFS_RAW: return "OTFS-RAW";
            default: return "UNKNOWN";
        }
    }

    void printHeader() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║           ProjectUltra CLI Simulator                         ║\n";
        std::cout << "║           (Exact replica of GUI test mode)                   ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "Configuration:\n";
        std::cout << "  SNR:       " << simulation_snr_db_ << " dB\n";
        std::cout << "  Waveform:  " << waveformModeToString(initial_waveform_) << "\n";
        std::cout << "  Message:   \"" << test_message_ << "\"\n";
        std::cout << "  Stations:  ALPHA <-> BRAVO\n";
        std::cout << "\n";
    }

    void printModeStatus() {
        std::cout << "\n  Negotiated modes:\n";
        std::cout << "    ALPHA: " << modulationToString(our_data_modulation_)
                  << " " << codeRateToString(our_data_rate_) << "\n";
        std::cout << "    BRAVO: " << modulationToString(virtual_data_modulation_)
                  << " " << codeRateToString(virtual_data_rate_) << "\n";
    }

    void printSummary() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                        SUMMARY                               ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "  Total simulated time: " << (simulated_time_ms_ / 1000.0f) << " seconds\n";
        std::cout << "  Final Mode: " << modulationToString(our_data_modulation_)
                  << " " << codeRateToString(our_data_rate_) << "\n";
        std::cout << "  Message transferred successfully!\n";
        std::cout << "\n";
    }

    void log(const char* fmt, ...) {
        char buf[512];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        std::cout << buf << "\n";
    }

    void logSuccess(const char* msg) {
        std::cout << "\033[32m  ✓ " << msg << "\033[0m\n";
    }

    void logError(const char* msg) {
        std::cout << "\033[31m  ✗ " << msg << "\033[0m\n";
    }
};

int main(int argc, char* argv[]) {
    CLISimulator sim;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-snr" && i + 1 < argc) {
            sim.setSNR(std::stof(argv[++i]));
        } else if (arg == "-msg" && i + 1 < argc) {
            sim.setMessage(argv[++i]);
        } else if (arg == "-verbose") {
            sim.setVerbose(true);
        } else if (arg == "-timeout" && i + 1 < argc) {
            sim.setTimeout(std::stoul(argv[++i]));
        } else if ((arg == "-waveform" || arg == "--waveform") && i + 1 < argc) {
            sim.setWaveform(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "CLI Simulator - Exact replica of GUI test mode\n\n";
            std::cout << "Usage: " << argv[0] << " [options]\n\n";
            std::cout << "Options:\n";
            std::cout << "  -snr <dB>        Set channel SNR (default: 20)\n";
            std::cout << "  -msg <text>      Message to send\n";
            std::cout << "  -waveform <mode> Initial waveform: dpsk (default) or mfsk\n";
            std::cout << "  -verbose         Enable verbose logging\n";
            std::cout << "  -timeout <ms>    Max simulation time (default: 60000)\n";
            return 0;
        }
    }

    // Run the simulation
    bool success = sim.run();

    return success ? 0 : 1;
}
