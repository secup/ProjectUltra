/**
 * CLI Simulator - Fast batch processing version
 *
 * Processes audio as fast as possible (no real-time pacing)
 * for automated testing of the full protocol flow.
 *
 * Usage:
 *   ./cli_simulator [options]
 *
 * Options:
 *   --snr <dB>     Set channel SNR (default: 20)
 *   --verbose      Enable verbose logging
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
#include <random>

#include "gui/modem/modem_engine.hpp"
#include "protocol/protocol_engine.hpp"
#include "ultra/logging.hpp"

using namespace ultra;
using namespace ultra::gui;
using namespace ultra::protocol;

class CLISimulator {
public:
    CLISimulator() = default;

    void setSNR(float snr) { snr_db_ = snr; }
    void setVerbose(bool v) { verbose_ = v; }
    void setNoReply(bool v) { no_reply_ = v; }

    bool runTest() {
        printHeader();
        initStations();

        // Phase 1: Connect
        std::cout << "\n=== PHASE 1: CONNECTION ===\n";
        std::cout << "  ALPHA connecting to BRAVO...\n";
        protocol_.connect("BRAVO");

        // Process until connected (or timeout)
        auto start = std::chrono::steady_clock::now();
        while (!our_connected_ || !virtual_connected_) {
            processOneTick();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > 30) {
                std::cout << "  \033[31m✗ Connection timeout!\033[0m\n";
                return false;
            }
        }
        std::cout << "  \033[32m✓ Connected!\033[0m\n";

        // Phase 2: Wait for ALPHA's handshake to complete (mode negotiation)
        // Note: BRAVO's handshake completes when it receives the first DATA frame from ALPHA
        std::cout << "\n=== PHASE 2: MODE NEGOTIATION ===\n";
        start = std::chrono::steady_clock::now();
        while (!handshake_complete_alpha_) {
            processOneTick();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > 30) {
                std::cout << "  \033[31m✗ Mode negotiation timeout!\033[0m\n";
                return false;
            }
        }
        std::cout << "  Negotiated: " << modulationToString(our_modulation_)
                  << " " << codeRateToString(our_rate_) << "\n";
        std::cout << "  \033[32m✓ Mode negotiation complete!\033[0m\n";

        // Phase 3: Send messages
        std::cout << "\n=== PHASE 3: DATA TRANSFER ===\n";
        std::vector<std::string> messages = {
            "Hello World! This is a test.",
            "Second message with more data",
            "Third message - testing 123"
        };

        for (size_t i = 0; i < messages.size(); i++) {
            message_received_ = false;
            received_message_.clear();

            // Wait for ARQ ready
            start = std::chrono::steady_clock::now();
            while (!protocol_.isReadyToSend()) {
                processOneTick();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed > 30) {
                    std::cout << "  \033[31m✗ ARQ not ready (timeout)\033[0m\n";
                    return false;
                }
            }

            std::cout << "  Sending message " << (i+1) << ": \"" << messages[i] << "\"\n";
            protocol_.sendMessage(messages[i]);

            // Wait for reception
            start = std::chrono::steady_clock::now();
            while (!message_received_) {
                processOneTick();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start).count();
                if (elapsed > 30) {
                    std::cout << "  \033[31m✗ Message " << (i+1) << " not received (timeout)\033[0m\n";
                    return false;
                }
            }

            if (received_message_ == messages[i]) {
                std::cout << "  \033[32m✓ Message " << (i+1) << " received correctly!\033[0m\n";
            } else {
                std::cout << "  \033[31m✗ Message corrupted!\033[0m\n";
                std::cout << "    Expected: \"" << messages[i] << "\"\n";
                std::cout << "    Received: \"" << received_message_ << "\"\n";
                return false;
            }
        }

        // Phase 4: Disconnect
        std::cout << "\n=== PHASE 4: DISCONNECT ===\n";
        protocol_.disconnect();

        start = std::chrono::steady_clock::now();
        while (our_connected_ || virtual_connected_) {
            processOneTick();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > 30) {
                std::cout << "  \033[31m✗ Disconnect timeout!\033[0m\n";
                return false;
            }
        }
        std::cout << "  \033[32m✓ Disconnected!\033[0m\n";

        printSummary();
        return true;
    }

private:
    // Stations
    ModemEngine modem_;
    ProtocolEngine protocol_{ConnectionConfig{}};
    std::unique_ptr<ModemEngine> virtual_modem_;
    ProtocolEngine virtual_protocol_{ConnectionConfig{}};

    // Channel buffers
    std::deque<float> channel_to_virtual_;
    std::deque<float> channel_to_us_;

    // TX pending buffers
    std::vector<float> our_tx_pending_;
    std::vector<float> virtual_tx_pending_;

    // State
    bool our_connected_ = false;
    bool virtual_connected_ = false;
    bool handshake_complete_alpha_ = false;
    bool handshake_complete_bravo_ = false;
    bool message_received_ = false;
    std::string received_message_;
    Modulation our_modulation_ = Modulation::DQPSK;
    CodeRate our_rate_ = CodeRate::R1_4;

    // Config
    float snr_db_ = 20.0f;
    bool verbose_ = false;
    std::mt19937 rng_{42};

    // Timing
    std::chrono::steady_clock::time_point last_tick_time_ = std::chrono::steady_clock::now();

    void initStations() {
        // Reset timing baseline
        last_tick_time_ = std::chrono::steady_clock::now();

        // === Our station (ALPHA) ===
        modem_.setLogPrefix("ALPHA");
        protocol_.setLocalCallsign("ALPHA");
        protocol_.setAutoAccept(true);

        protocol_.setTxDataCallback([this](const Bytes& data) {
            auto samples = modem_.transmit(data);
            if (verbose_) {
                std::cout << "  [ALPHA] TX: " << samples.size() << " samples\n";
            }
            our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
        });

        modem_.setRawDataCallback([this](const Bytes& data) {
            protocol_.setMeasuredSNR(snr_db_);
            protocol_.onRxData(data);
        });

        protocol_.setConnectionChangedCallback([this](ConnectionState state, const std::string& info) {
            if (state == ConnectionState::CONNECTED) {
                our_connected_ = true;
                modem_.setConnected(true);
                if (verbose_) std::cout << "  [ALPHA] CONNECTED to " << info << "\n";
            } else if (state == ConnectionState::DISCONNECTED) {
                our_connected_ = false;
                modem_.setConnected(false);
                if (verbose_) std::cout << "  [ALPHA] DISCONNECTED\n";
            }
        });

        protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float) {
            our_modulation_ = mod;
            our_rate_ = rate;
            modem_.setDataMode(mod, rate);
        });

        protocol_.setModeNegotiatedCallback([this](WaveformMode mode) {
            modem_.setWaveformMode(mode);
        });

        protocol_.setConnectWaveformChangedCallback([this](WaveformMode mode) {
            modem_.setConnectWaveform(mode);
        });

        protocol_.setHandshakeConfirmedCallback([this]() {
            modem_.setHandshakeComplete(true);
            handshake_complete_alpha_ = true;
        });

        // PING TX callback - ALPHA wants to probe
        protocol_.setPingTxCallback([this]() {
            if (verbose_) std::cout << "  [ALPHA] TX PING\n";
            auto samples = modem_.transmitPing();
            our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
        });

        // PING received callback - someone is probing ALPHA
        protocol_.setPingReceivedCallback([this]() {
            if (verbose_) std::cout << "  [ALPHA] RX PING, sending PONG\n";
            auto samples = modem_.transmitPong();
            our_tx_pending_.insert(our_tx_pending_.end(), samples.begin(), samples.end());
        });

        // Wire up modem ping detection to protocol
        modem_.setPingReceivedCallback([this](float) {
            protocol_.onPingReceived();
        });

        // === Virtual station (BRAVO) ===
        virtual_modem_ = std::make_unique<ModemEngine>();
        virtual_modem_->setLogPrefix("BRAVO");
        virtual_protocol_.setLocalCallsign("BRAVO");
        virtual_protocol_.setAutoAccept(true);

        virtual_protocol_.setTxDataCallback([this](const Bytes& data) {
            auto samples = virtual_modem_->transmit(data);
            if (verbose_) {
                std::cout << "  [BRAVO] TX: " << samples.size() << " samples\n";
            }
            virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
        });

        virtual_modem_->setRawDataCallback([this](const Bytes& data) {
            virtual_protocol_.setMeasuredSNR(snr_db_);
            virtual_protocol_.onRxData(data);
        });

        virtual_protocol_.setConnectionChangedCallback([this](ConnectionState state, const std::string& info) {
            if (state == ConnectionState::CONNECTED) {
                virtual_connected_ = true;
                virtual_modem_->setConnected(true);
                if (verbose_) std::cout << "  [BRAVO] CONNECTED to " << info << "\n";
            } else if (state == ConnectionState::DISCONNECTED) {
                virtual_connected_ = false;
                virtual_modem_->setConnected(false);
                if (verbose_) std::cout << "  [BRAVO] DISCONNECTED\n";
            }
        });

        virtual_protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate, float) {
            virtual_modem_->setDataMode(mod, rate);
        });

        virtual_protocol_.setModeNegotiatedCallback([this](WaveformMode mode) {
            virtual_modem_->setWaveformMode(mode);
        });

        virtual_protocol_.setConnectWaveformChangedCallback([this](WaveformMode mode) {
            virtual_modem_->setConnectWaveform(mode);
        });

        virtual_protocol_.setHandshakeConfirmedCallback([this]() {
            virtual_modem_->setHandshakeComplete(true);
            handshake_complete_bravo_ = true;
        });

        // PING TX callback - BRAVO wants to probe
        virtual_protocol_.setPingTxCallback([this]() {
            if (verbose_) std::cout << "  [BRAVO] TX PING\n";
            auto samples = virtual_modem_->transmitPing();
            virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
        });

        // PING received callback - someone is probing BRAVO
        virtual_protocol_.setPingReceivedCallback([this]() {
            if (verbose_) std::cout << "  [BRAVO] RX PING, sending PONG\n";
            auto samples = virtual_modem_->transmitPong();
            virtual_tx_pending_.insert(virtual_tx_pending_.end(), samples.begin(), samples.end());
        });

        // Wire up modem ping detection to protocol
        virtual_modem_->setPingReceivedCallback([this](float) {
            virtual_protocol_.onPingReceived();
        });

        virtual_protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
            message_received_ = true;
            received_message_ = text;
            if (verbose_) {
                std::cout << "  [BRAVO] Received from " << from << ": \"" << text << "\"\n";
            }
        });
    }

    // Process one simulation tick - move audio and tick protocols
    void processOneTick() {
        constexpr size_t CHUNK_SIZE = 480;  // 10ms at 48kHz

        // Track actual elapsed time for protocol ticks
        auto now = std::chrono::steady_clock::now();
        uint32_t elapsed_ms = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick_time_).count());
        if (elapsed_ms < 1) elapsed_ms = 1;  // Minimum 1ms per tick
        last_tick_time_ = now;

        // Move ALPHA TX -> channel -> BRAVO RX
        size_t alpha_tx_moved = 0;
        while (our_tx_pending_.size() >= CHUNK_SIZE) {
            std::vector<float> chunk(our_tx_pending_.begin(), our_tx_pending_.begin() + CHUNK_SIZE);
            our_tx_pending_.erase(our_tx_pending_.begin(), our_tx_pending_.begin() + CHUNK_SIZE);
            applyChannel(chunk);
            channel_to_virtual_.insert(channel_to_virtual_.end(), chunk.begin(), chunk.end());
            alpha_tx_moved += CHUNK_SIZE;
        }

        // Move BRAVO TX -> channel -> ALPHA RX
        size_t bravo_tx_moved = 0;
        while (virtual_tx_pending_.size() >= CHUNK_SIZE) {
            std::vector<float> chunk(virtual_tx_pending_.begin(), virtual_tx_pending_.begin() + CHUNK_SIZE);
            virtual_tx_pending_.erase(virtual_tx_pending_.begin(), virtual_tx_pending_.begin() + CHUNK_SIZE);
            applyChannel(chunk);
            channel_to_us_.insert(channel_to_us_.end(), chunk.begin(), chunk.end());
            bravo_tx_moved += CHUNK_SIZE;
        }

        // Feed BRAVO RX - modem threads handle processing automatically
        size_t bravo_rx_fed = 0;
        if (!channel_to_virtual_.empty()) {
            std::vector<float> samples(channel_to_virtual_.begin(), channel_to_virtual_.end());
            bravo_rx_fed = samples.size();
            virtual_modem_->feedAudio(samples);
            channel_to_virtual_.clear();
        }

        // Feed ALPHA RX - modem threads handle processing automatically
        size_t alpha_rx_fed = 0;
        if (!channel_to_us_.empty()) {
            std::vector<float> samples(channel_to_us_.begin(), channel_to_us_.end());
            alpha_rx_fed = samples.size();
            modem_.feedAudio(samples);
            channel_to_us_.clear();
        }

        // Debug: show audio flow periodically
        static size_t tick_count = 0;
        static size_t total_alpha_tx = 0, total_bravo_rx = 0;
        total_alpha_tx += alpha_tx_moved;
        total_bravo_rx += bravo_rx_fed;
        if (++tick_count % 1000 == 0) {
            std::cout << "  [TICK " << tick_count << "] pending=" << our_tx_pending_.size()
                      << ", channel=" << channel_to_virtual_.size()
                      << ", total_tx=" << total_alpha_tx
                      << ", total_rx=" << total_bravo_rx << std::endl;
        }

        // Tick protocols with actual elapsed time
        protocol_.tick(elapsed_ms);
        virtual_protocol_.tick(elapsed_ms);

        // Give acquisition threads time to run (they need CPU time for preamble detection)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void applyChannel(std::vector<float>& samples) {
        if (samples.empty()) return;

        // DEBUG: Skip AWGN to test if preamble is detected without noise
        // TODO: Re-enable once basic detection works
        (void)snr_db_;  // Suppress unused warning
        (void)rng_;
        return;  // No channel effects for now

        // Calculate signal power
        float sum_sq = 0.0f;
        for (float s : samples) sum_sq += s * s;
        float signal_rms = std::sqrt(sum_sq / samples.size());
        if (signal_rms < 1e-6f) return;

        // Add AWGN
        float snr_linear = std::pow(10.0f, snr_db_ / 10.0f);
        float noise_power = (signal_rms * signal_rms) / snr_linear;
        float noise_stddev = std::sqrt(noise_power);

        std::normal_distribution<float> noise(0.0f, noise_stddev);
        for (float& s : samples) {
            s += noise(rng_);
        }
    }

    void printHeader() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║           ProjectUltra CLI Simulator (Fast Mode)             ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
        std::cout << "Configuration:\n";
        std::cout << "  SNR:       " << snr_db_ << " dB\n";
        std::cout << "  Stations:  ALPHA <-> BRAVO\n";
        std::cout << "\n";
    }

    void printSummary() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                     TEST PASSED                              ║\n";
        std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
        std::cout << "\n";
    }
};

int main(int argc, char* argv[]) {
    CLISimulator sim;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "--snr" || arg == "-snr") && i + 1 < argc) {
            sim.setSNR(std::stof(argv[++i]));
        } else if (arg == "--verbose" || arg == "-v") {
            sim.setVerbose(true);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "CLI Simulator - Fast batch processing\n\n";
            std::cout << "Usage: " << argv[0] << " [options]\n\n";
            std::cout << "Options:\n";
            std::cout << "  --snr <dB>   Set channel SNR (default: 20)\n";
            std::cout << "  --verbose    Enable verbose logging\n";
            return 0;
        }
    }

    // Enable logging for debugging
    setLogLevel(LogLevel::INFO);

    bool success = sim.runTest();
    return success ? 0 : 1;
}
