#include "gui/audio_engine.hpp"
#include "diagnostics/diagnostics_recorder.hpp"
#include "gui/modem/modem_engine.hpp"
#include "gui/modem/modem_protocol_binding.hpp"
#include "otasim_client/ota_audio_backend.hpp"
#include "otasim_client/ota_rx_pump.hpp"
#include "ptt/ptt_driver_factory.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/protocol_engine.hpp"
#include "tnc/tnc_bridge.hpp"
#include "tnc/tnc_server.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/build_info.hpp"
#include "ultra/tx_burst_normalization.hpp"

#include "ultra_tnc_config.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <system_error>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using ultra::Bytes;
using ultra::CodeRate;
using ultra::ModemConfig;
using ultra::Modulation;
using ultra::gui::ModemEngine;
using ultra::gui::ModemProtocolFrontendHooks;
using ultra::otasim_client::OtaAudioBackend;
using ultra::otasim_client::OtaAudioBackendConfig;
using ultra::otasim_client::OtaAudioConnectionState;
using ultra::protocol::ConnectionState;
using ultra::protocol::WaveformMode;
namespace v2 = ultra::protocol::v2;

#ifndef ULTRA_TNC_TESTING
std::atomic<bool> g_stop_requested{false};
std::atomic<bool> g_report_requested{false};

void handleSignal(int) {
    g_stop_requested.store(true, std::memory_order_release);
}

void handleReportSignal(int) {
    g_report_requested.store(true, std::memory_order_release);
}

void printBuildProvenance(std::ostream& out) {
    out << "ProjectUltra " << ultra::kBuildVersion
        << " commit=" << ultra::kBuildGitCommit
        << " dirty=" << (ultra::kBuildDirty ? "true" : "false")
        << " tag=" << (ultra::kBuildReleaseTag[0] ? ultra::kBuildReleaseTag : "none")
        << " built=" << ultra::kBuildTimeUtc
        << " os=" << ultra::kBuildOS << "\n";
}
#endif

// Config + CLI/config parsing live in ultra_tnc_config.cpp so the
// pure parsing logic can be exercised by unit tests without pulling
// in audio, PTT, or the protocol engine.
using OFDMConfigPreset = ultra::tnc::config::OFDMConfigPreset;
using Config = ultra::tnc::config::Config;
using ultra::tnc::config::isNoneDevice;
using ultra::tnc::config::lower;

#ifndef ULTRA_TNC_TESTING
ultra::ptt::PttConfig makePttConfig(const Config& cfg) {
    ultra::ptt::PttConfig ptt;
    if (cfg.ptt_hamlib) {
        ptt.mode = ultra::ptt::PttMode::HamlibBuiltin;
        ptt.hamlib_model_id = cfg.ptt_hamlib_model;
        ptt.hamlib_rig_port = cfg.ptt_hamlib_port;
        ptt.hamlib_baud = cfg.ptt_hamlib_baud;
        const std::string m = lower(cfg.ptt_hamlib_ptt);
        if (m == "vox") {
            ptt.hamlib_ptt_method = ultra::ptt::HamlibPttMethod::Vox;
        } else if (m == "dtr") {
            ptt.hamlib_ptt_method = ultra::ptt::HamlibPttMethod::DTR;
        } else if (m == "rts") {
            ptt.hamlib_ptt_method = ultra::ptt::HamlibPttMethod::RTS;
        } else {
            ptt.hamlib_ptt_method = ultra::ptt::HamlibPttMethod::Cat;
        }
    } else if (cfg.ptt_cat) {
        ptt.mode = ultra::ptt::PttMode::Cat;
        ptt.cat_host = cfg.ptt_cat_host;
        ptt.cat_port = cfg.ptt_cat_port;
    } else if (!cfg.ptt_serial_port.empty()) {
        ptt.mode = ultra::ptt::PttMode::Serial;
        ptt.serial_port = cfg.ptt_serial_port;
        ptt.serial_baud = cfg.ptt_serial_baud;
        ptt.serial_line = (lower(cfg.ptt_serial_line) == "dtr")
                              ? ultra::ptt::SerialLine::DTR
                              : ultra::ptt::SerialLine::RTS;
        ptt.serial_inactive_high = cfg.ptt_inactive_high;
    }
    return ptt;
}
#endif

std::string audioDeviceLabel(const std::string& device) {
    return (device.empty() || lower(device) == "default") ? "Default" : device;
}

void printTncAudioDeviceHint() {
    std::cerr << "Next step: run: ultra_tnc --list-audio-devices\n"
              << "Then copy the exact device name into --audio-output/--audio-input "
                 "or ultra_tnc.conf.\n";
}

#ifndef ULTRA_TNC_TESTING
struct LogFileCloser {
    void operator()(std::FILE* file) const {
        if (file) std::fclose(file);
    }
};

using LogFileHandle = std::unique_ptr<std::FILE, LogFileCloser>;

bool configureLogging(const Config& cfg, LogFileHandle& log_file) {
    ultra::setOperatorLogProfile();
    // Console app: when no --log-file is set, logs go to the cmd window (Windows would
    // otherwise drop them — its log() has no implicit stderr fallback, unlike the POSIX build).
    ultra::setLogConsoleFallback(true);
    ultra::setLogLevel(cfg.log_level);

    if (cfg.log_level_set && cfg.log_level >= ultra::LogLevel::DEBUG &&
        !cfg.log_categories_set) {
        ultra::setDeveloperLogProfile();
    }

    if (cfg.log_categories_set && !ultra::setLogCategories(cfg.log_categories)) {
        std::cerr << "Invalid --log-category list: " << cfg.log_categories << "\n";
        return false;
    }

    if (!cfg.log_file.empty()) {
        errno = 0;
        log_file.reset(std::fopen(cfg.log_file.c_str(), "a"));
        if (!log_file) {
            std::cerr << "Failed to open --log-file '" << cfg.log_file << "'";
            if (errno != 0) {
                std::cerr << ": " << std::strerror(errno);
            }
            std::cerr << "\nNext step: choose a writable path or fix directory permissions.\n";
            return false;
        }
        ultra::setLogFile(log_file.get());
    }
    return true;
}
#endif

class UltraTNCStation {
public:
    UltraTNCStation(const Config& cfg,
                    ultra::protocol::ProtocolEngine& engine,
                    ultra::gui::AudioEngine& audio,
                    ultra::tnc::TNCBridge& bridge)
        : cfg_(cfg),
          engine_(engine),
          audio_(audio),
          bridge_(bridge) {
        configureModem();
        setupCallbacks();
    }

    ~UltraTNCStation() {
        stop();
    }

    bool start() {
        if (running_.load()) {
            return true;
        }

        const bool use_output = !isNoneDevice(cfg_.audio_output);
        const bool use_input = !isNoneDevice(cfg_.audio_input);

        if (cfg_.sim_audio) {
            OtaAudioBackendConfig ota_config;
            ota_config.grpc_target = cfg_.ota_host;
            ota_config.udp_target = cfg_.ota_udp_host;
            ota_config.token = cfg_.token;
            ota_config.station_id = cfg_.station_id;
            ota_config.session_id = cfg_.session_id.empty() ? "lobby" : cfg_.session_id;

            ota_audio_ = std::make_unique<OtaAudioBackend>();
            std::string error;
            if (!ota_audio_->start(std::move(ota_config), &error)) {
                std::cerr << "OTASim audio start failed: " << error << "\n";
                ota_audio_.reset();
                return false;
            }
            input_enabled_ = true;
            output_enabled_ = true;
            running_.store(true);
            reportOtaStatus(true);
            return true;
        }

        if (use_output || use_input) {
            if (!audio_.initialize()) {
                std::cerr << "AudioEngine init failed\n";
                return false;
            }
        }

        if (use_output) {
            if (!audio_.openOutput(cfg_.audio_output)) {
                std::cerr << "Failed to open audio output device '"
                          << audioDeviceLabel(cfg_.audio_output) << "'\n";
                printTncAudioDeviceHint();
                return false;
            }
            audio_.startPlayback();
            output_enabled_ = true;
        } else {
            LOG_INFO("AUDIO", "Audio output disabled");
        }

        if (use_input) {
            audio_.setInputCaptureMode(ultra::gui::AudioEngine::InputCaptureMode::Queue);
            if (!audio_.openInput(cfg_.audio_input)) {
                std::cerr << "Failed to open audio input device '"
                          << audioDeviceLabel(cfg_.audio_input) << "'\n";
                printTncAudioDeviceHint();
                return false;
            }
            audio_.startCapture();
            input_enabled_ = true;
        } else {
            LOG_INFO("AUDIO", "Audio input disabled");
        }

        running_.store(true);
        return true;
    }

    void stop() {
        if (!running_.exchange(false)) {
            return;
        }

        // ModemEngine owns + joins its decode thread in its destructor; nothing to stop here.

        if (cfg_.sim_audio) {
            if (ota_audio_) {
                ota_audio_->close();
                ota_audio_.reset();
            }
            input_enabled_ = false;
            output_enabled_ = false;
            return;
        }

        if (input_enabled_) {
            audio_.stopCapture();
            audio_.closeInput();
            input_enabled_ = false;
        }
        if (output_enabled_) {
            audio_.stopPlayback();
            audio_.closeOutput();
            output_enabled_ = false;
        }
        audio_.shutdown();
    }

    void tick(uint32_t elapsed_ms) {
        engine_.tick(elapsed_ms);

        if (cfg_.sim_audio) {
            reportOtaStatus(false);
            if (input_enabled_ && ota_audio_) {
                std::lock_guard<std::mutex> lock(input_audio_mutex_);
                // Shared OTASim RX drain — the SAME code the GUI's App::pollOtaRx uses
                // (ultra::otasim_client::drainOtaRx): drain only real samples, never
                // fabricate filler. Rationale + the regression it prevents are documented
                // in ota_rx_pump.hpp; sharing the loop makes it structurally impossible for
                // the two frontends' feed discipline to drift. SyncController owns cold/warm.
                ultra::otasim_client::drainOtaRx(
                    *ota_audio_,
                    [this](const std::vector<float>& samples) { modem_.feedAudio(samples); });
            }
        } else if (input_enabled_) {
            std::lock_guard<std::mutex> lock(input_audio_mutex_);
            auto samples = audio_.getRxSamples(4096);
            if (!samples.empty()) {
                modem_.feedAudio(samples);
            }
        }

        bridge_.tick(elapsed_ms);
    }

#ifdef ULTRA_TNC_TESTING
    std::vector<float> testTransmitFrame(const Bytes& data) {
        return transmitFrame(data);
    }

    std::vector<float> testTransmitBurst(const std::vector<Bytes>& frames) {
        return transmitBurst(frames);
    }

    std::vector<float> testTransmitPing() {
        return transmitPing();
    }

    void testFeedAudio(const float* samples, size_t count) {
        modem_.feedAudio(samples, count);
    }

    void testProcessDecoder() {
        modem_.processRxBuffer();
    }

    ultra::gui::DecoderStats testDecoderStats() const {
        return modem_.getDecoderStats();
    }
#endif

private:
    static constexpr int kSampleRate = 48000;

    const Config& cfg_;
    ultra::protocol::ProtocolEngine& engine_;
    ultra::gui::AudioEngine& audio_;
    ultra::tnc::TNCBridge& bridge_;

    // The shared modem integration layer (same class the GUI uses), replacing the raw
    // StreamingEncoder/StreamingDecoder the TNC used to own. ModemEngine owns the encoder +
    // decoder + its own decode thread; ultra_tnc feeds it audio and consumes its callbacks.
    ModemEngine modem_;

    ModemConfig base_ofdm_config_;
    ModemConfig ofdm_config_;
    WaveformMode tx_waveform_mode_ = WaveformMode::MC_DPSK;
    WaveformMode negotiated_waveform_ = WaveformMode::MC_DPSK;
    Modulation data_modulation_ = Modulation::DQPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;

    std::atomic<bool> running_{false};
    // Mirrors Connection's explicit close phase. It is checked again at the
    // physical queue boundary so a ProtocolEngine tick/RX race cannot leak a
    // previously-drained DATA frame after teardown activation.
    std::atomic<bool> disconnect_teardown_active_{false};
    std::mutex input_audio_mutex_;
    bool input_enabled_ = false;
    bool output_enabled_ = false;
    bool handshake_complete_ = false;
    bool connected_ = false;
    std::unique_ptr<OtaAudioBackend> ota_audio_;
    OtaAudioConnectionState last_ota_state_ = OtaAudioConnectionState::Disconnected;
    std::string last_ota_text_;
    bool ota_status_seen_ = false;

    ModemConfig createOFDMConfig() const {
        // A default-constructed ModemConfig already carries the canonical OFDM
        // geometry (1024-FFT, 59 carriers, MEDIUM CP) that OFDM-CHIRP uses; the
        // overrides below set modulation/rate/pilots. (The NVIS preset shares the
        // same geometry, so no special-casing is needed here.)
        ModemConfig cfg;

        cfg.sample_rate = kSampleRate;
        cfg.center_freq = 1500.0f;
        cfg.modulation = data_modulation_;
        cfg.code_rate = data_code_rate_;
        cfg.use_pilots = true;
        cfg.pilot_spacing =
            ultra::ofdm_link_adaptation::recommendedPilotSpacing(cfg.modulation, cfg.code_rate);
        return cfg;
    }

    void configureModem() {
        engine_.setLocalCallsign(cfg_.callsign);
        // Peak-limited TX PAPR correction: the hardware path scales every burst to the same peak,
        // so OFDM data rides below the MC-DPSK handshake the peer measures; --sim-audio scales to
        // the same average level (no gap). The measured prior; the GUI refines it live.
        engine_.setTxPaprOffsetDb(cfg_.sim_audio
                                      ? 0.0f
                                      : ultra::protocol::connection_policy::kPeakNormalizedOfdmBelowMcDpskDb);
        engine_.setMeasuredSNR(cfg_.snr_db, ultra::SNRSource::NONE);
        // Inbound bulk transfers (the peer's accumulated data-port stream, shipped as a
        // modem file transfer) reconstruct here; TNCBridge::onFileReceived reads the wire
        // bytes, delivers them out the data port, and removes the file. Per-process dir
        // (callsign) so two ultra_tnc sharing /tmp (the OTASim test rig) don't collide.
        {
            std::error_code ec;
            const auto rx_dir =
                std::filesystem::temp_directory_path() /
                ("ultra_tnc_rx_" + (cfg_.callsign.empty() ? std::string("tnc") : cfg_.callsign));
            std::filesystem::create_directories(rx_dir, ec);
            engine_.setReceiveDirectory(rx_dir.string());
        }
        if (cfg_.forced_mod != Modulation::AUTO) {
            engine_.setForcedModulation(cfg_.forced_mod);
        }
        if (cfg_.forced_rate != CodeRate::AUTO) {
            engine_.setForcedCodeRate(cfg_.forced_rate);
        }
        // Chase-combining HARQ: stores soft LLRs from failed decodes
        // and sums them into subsequent retransmission attempts. The
        // per-frame buffer overhead is small; the decode-success
        // payoff on retransmissions is measurable. Defaults to ON
        // because retx are rare on a clean channel (zero overhead)
        // and welcome on a noisy one (faster recovery).
        engine_.setSoftCombiningHARQ(true);

        base_ofdm_config_ = createOFDMConfig();
        ofdm_config_ = base_ofdm_config_;

        // Drive the SHARED ModemEngine exactly the way the GUI's app.cpp does: ModemEngine
        // owns the encoder + decoder, ultra_tnc feeds it OTASim/SDL audio and consumes its
        // callbacks. setSynchronousMode(false) -> ModemEngine runs its own decode thread
        // (the TNC no longer owns a decodeLoop), proven on OTASim by the GUI floor gate.
        modem_.setLogPrefix(cfg_.callsign);
        modem_.setLocalCallsign(cfg_.callsign);  // RX address filter; updated live on MYCALL
        modem_.setSynchronousMode(false);
        modem_.setOFDMConfig(ofdm_config_);
        modem_.setWaveformMode(tx_waveform_mode_);
        modem_.setConnectWaveform(WaveformMode::MC_DPSK);
        modem_.setDataMode(data_modulation_, data_code_rate_);
        modem_.setMCDPSKCarriers(8);
        modem_.setFixedFrameCodewords(v2::kDefaultFixedFrameCodewords);
        modem_.setPaprReductionEnabled(cfg_.papr_reduction);
        modem_.setSoftCombineBuffer(engine_.softCombineBuffer());

        // The SINGLE shared modem->protocol binding (same one the GUI calls). This is what
        // wires burst-group delivery + tone-burst GROUP_ACK + HARQ ctx that the old raw-
        // decoder TNC silently missed. The after_rx_data hook keeps the TNC's per-frame
        // success telemetry (frame.rx); decode-failure telemetry is dropped with the
        // DecodeResult path (observability only).
        ultra::gui::ModemProtocolFrontendHooks hooks;
        hooks.after_rx_data = [](const Bytes& data, float snr_db, float fading,
                                 ultra::SNRSource snr_source, bool used_for_quality) {
            char fields[224];
            std::snprintf(fields, sizeof(fields),
                          "{\"bytes\":%zu,\"snr_db\":%.1f,\"snr_source\":\"%s\","
                          "\"fading\":%.2f,\"quality_sample\":%s}",
                          data.size(), snr_db, ultra::snrSourceToString(snr_source), fading,
                          used_for_quality ? "true" : "false");
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText("phy", "frame.rx",
                                                                         fields);
        };
        ultra::gui::wireModemToProtocol(modem_, engine_, std::move(hooks));
    }

    void setupCallbacks() {
        // The TNC bridges an interactive client (PAT/Winlink B2F): both stations
        // alternately transmit. Keep the half-duplex ISS/IRS turn gate on burst
        // sends so the two directions serialize instead of colliding (the default
        // one-way burst path bypasses the gate — correct only for ALPHA->BRAVO file
        // push, wrong for bidirectional exchange).
        engine_.setHalfDuplexInteractive(true);

        // On a turn-flip (we just took the DATA turn), force the next transmission
        // to a full chirp+LTS anchor so the peer can re-acquire our timing — the
        // fix for the half-duplex B2F stall (BUG-TNC-B2F-001). The yielding peer
        // already arms expectFullOFDMAnchorOnce() via the TURNOVER it sent.
        // Keepalive ACK universal busy gate (the headless TNC has no
        // listen-before-ACK, so the keepalive must self-gate on channel-busy).
        engine_.setChannelBusyQuery([this] {
            return modem_.channelBusyForTx() ||
                   modem_.burstAirSamplesRemaining() > 0;
        });
        engine_.setDataTurnAcquiredCallback([this]() {
            modem_.forceNextFrameFullPreamble();
        });
        // Keep the RX decoder armed to cold-acquire the peer's full-anchored first burst
        // across the turn-flip gap (re-armed each tick by Connection while waiting).
        engine_.setFullOFDMAnchorExpectedCallback([this]() {
            modem_.expectFullOFDMAnchorOnce();
        });

        // The responder remains logically CONNECTED while it holds the final
        // DISCONNECT ACK grace. Follow Connection's explicit close phase rather
        // than the coarse state enum so both initiator and responder restrict
        // acquisition to the hardened one-codeword control geometry.
        engine_.setDisconnectTeardownCallback([this](bool active) {
            disconnect_teardown_active_.store(active, std::memory_order_release);
            modem_.setControlOnlyReceive(active);
        });

        engine_.setTxDataCallback([this](const Bytes& data,
                                         bool expect_full_ofdm_anchor_after_tx) {
            const auto header = v2::parseHeader(data);
            const bool disconnect_control =
                header.valid && header.seq == v2::DISCONNECT_SEQ &&
                (header.type == v2::FrameType::DISCONNECT ||
                 header.type == v2::FrameType::ACK);
            if (disconnect_teardown_active_.load(std::memory_order_acquire) &&
                !disconnect_control) {
                LOG_WARN("AUDIO",
                         "Teardown egress: dropped non-close TNC protocol frame");
                return;
            }
            auto samples = transmitFrame(data);
            queueTx(samples, disconnect_control);
            if (!samples.empty() && expect_full_ofdm_anchor_after_tx) {
                modem_.expectFullOFDMAnchorOnce();
            }
        });

        engine_.setTransmitBurstCallback([this](const std::vector<Bytes>& frames,
                                                uint16_t group_seq,
                                                uint8_t anchor_reason) {
            if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
                LOG_WARN("AUDIO", "Teardown egress: dropped TNC DATA burst");
                return;
            }
            const bool full_group_anchor =
                anchor_reason ==
                    ultra::protocol::Connection::kAnchorReasonResend ||
                anchor_reason ==
                    ultra::protocol::Connection::kAnchorReasonModeSwitch;
            const ultra::gui::BurstAnchorOptions anchor_options{
                full_group_anchor,
                anchor_reason ==
                    ultra::protocol::Connection::kAnchorReasonModeSwitch,
                anchor_reason !=
                    ultra::protocol::Connection::kAnchorReasonNone};
            queueTx(transmitBurst(frames, group_seq, anchor_options));
        });

        // The protocol->modem TX direction for the tone-burst GROUP_ACK — the half the
        // raw-decoder TNC was ALSO missing. Mirrors the GUI (app.cpp): the protocol asks
        // ModemEngine to encode the tone-burst ACK and we queue it to the audio sink; and
        // the protocol arms the always-on RX monitor to listen for the reply.
        engine_.setTransmitToneBurstAckCallback(
            [this](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& tba,
                   bool inbound_group_complete) {
                if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
                    LOG_WARN("AUDIO", "Teardown egress: dropped TNC tone ACK");
                    return;
                }
                const bool recent_rx = modem_.rxSignalActive(
                    ultra::protocol::connection_policy::
                        kDescriptorLostReverseTxHoldMs);
                const bool channel_busy = modem_.channelBusyForTx();
                const uint64_t air_remaining = modem_.burstAirSamplesRemaining();
                // The headless TNC has no GUI-style deferred-ACK scheduler.  A
                // physical group boundary is causally safe even if the energy
                // detector retains a stale busy reading; an asynchronous/timer
                // ACK is safe only when *all* independent inbound sensors agree.
                // Dropping merely delegates cumulative feedback to the sender's
                // retry/RTO path and is preferable to blanking our own receiver.
                const bool uncertain_inbound =
                    !inbound_group_complete && (recent_rx || channel_busy);
                if (air_remaining > 0 || uncertain_inbound) {
                    LOG_WARN("AUDIO",
                             "Dropped tone ACK while inbound OFDM may still be on "
                             "air (group_complete=%d recent_rx=%d cca_busy=%d "
                             "air_samples=%llu)",
                             inbound_group_complete ? 1 : 0,
                             recent_rx ? 1 : 0,
                             channel_busy ? 1 : 0,
                             static_cast<unsigned long long>(air_remaining));
                    return;  // sender's retransmit/RTO path re-requests cumulative state
                }
                queueTx(modem_.transmitToneBurstAck(tba));
            });
        engine_.setArmToneBurstAckMonitorCallback([this](uint32_t window_ms) {
            modem_.rearmToneBurstAckMonitor(window_ms);
        });

        engine_.setPingTxCallback([this]() {
            if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
                LOG_WARN("AUDIO", "Teardown egress: dropped TNC PING");
                return;
            }
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "ping.tx", "{\"kind\":\"ping\"}");
            queueTx(transmitPing());
        });

        engine_.setPingReceivedCallback([this]() {
            if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
                LOG_WARN("AUDIO", "Teardown egress: dropped TNC PONG");
                return;
            }
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "ping.tx", "{\"kind\":\"pong\"}");
            queueTx(transmitPing());
        });

        engine_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate,
                                                  int cw_count,
                                                  float peer_snr_db, float peer_fading,
                                                  int mc_dpsk_num_carriers,
                                                  int mc_dpsk_samples_per_symbol,
                                                  bool snr_is_wire) {
            (void)peer_snr_db;
            (void)snr_is_wire;
            if (mc_dpsk_num_carriers > 0 && mc_dpsk_samples_per_symbol > 0) {
                ultra::MultiCarrierDPSKConfig cfg;
                cfg.num_carriers = mc_dpsk_num_carriers;
                cfg.samples_per_symbol = mc_dpsk_samples_per_symbol;
                cfg.bits_per_symbol = (mod == Modulation::DBPSK) ? 1 :
                                      (mod == Modulation::D8PSK) ? 3 : 2;
                modem_.setMCDPSKConfig(cfg);
            }
            setDataMode(mod, rate);
            LOG_INFO("OPERATOR", "Mode: %s %s cw=%d",
                     ultra::modulationToString(mod), ultra::codeRateToString(rate), cw_count);
            char fields[192];
            std::snprintf(fields, sizeof(fields),
                          "{\"mod\":\"%s\",\"rate\":\"%s\",\"cw\":%d}",
                          ultra::modulationToString(mod), ultra::codeRateToString(rate), cw_count);
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "waveform.negotiated", fields);
            // Sync encoder/decoder to the negotiated CW count (set by the
            // protocol layer from CONNECT_ACK / MODE_CHANGE wire bytes).
            // Direct calls only — DO NOT re-enter ProtocolEngine here, the
            // engine mutex is held while this callback fires.
            modem_.setFixedFrameCodewords(cw_count);
        });

        engine_.setModeNegotiatedCallback([this](WaveformMode mode) {
            negotiated_waveform_ = mode;
            char fields[128];
            std::snprintf(fields, sizeof(fields),
                          "{\"waveform\":\"%s\"}",
                          ultra::protocol::waveformModeToString(mode));
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "waveform.negotiated", fields);
        });

        engine_.setConnectWaveformChangedCallback([this](WaveformMode mode) {
            if (mode == WaveformMode::OFDM_NARROW) {
                modem_.setNarrowbandControl(true);
            }
        });

        engine_.setHandshakeConfirmedCallback([this]() {
            handshake_complete_ = true;
            modem_.setHandshakeComplete(true);
        });

        bridge_.setConnectionChangedCallback([this](ConnectionState state, const std::string& info) {
            if (state == ConnectionState::CONNECTED) {
                setConnected(true);
                auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                diagnostics.ensureSessionActive();
                diagnostics.emitText("session", "session.state", "{\"state\":\"connected\"}");
                LOG_INFO("OPERATOR", "Connected: waveform=%s mode=%s %s",
                         ultra::protocol::waveformModeToString(negotiated_waveform_),
                         ultra::modulationToString(data_modulation_),
                         ultra::codeRateToString(data_code_rate_));
            } else if (state == ConnectionState::DISCONNECTED) {
                setConnected(false);
                auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                const std::string fields =
                    std::string("{\"state\":\"disconnected\",\"reason\":\"") +
                    ultra::diagnostics::jsonEscape(info) + "\"}";
                diagnostics.emitText("session", "session.state", fields.c_str());
                auto summary = diagnostics.finishSession(info);
                if (summary.ok) {
                    for (const auto& line : summary.operator_log_lines) {
                        LOG_INFO("OPERATOR", "%s", line.c_str());
                    }
                    LOG_INFO("OPERATOR", "Session debrief: %s", summary.path.string().c_str());
                } else {
                    LOG_WARN("OPERATOR", "Session debrief failed: %s", summary.error.c_str());
                }
                LOG_INFO("OPERATOR", "Disconnected");
            } else if (state == ConnectionState::PROBING) {
                auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                diagnostics.ensureSessionActive();
                diagnostics.emitText("session", "session.state", "{\"state\":\"probing\"}");
            } else if (state == ConnectionState::CONNECTING) {
                auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                diagnostics.ensureSessionActive();
                diagnostics.emitText("session", "session.state", "{\"state\":\"connecting\"}");
            }
        });

        bridge_.setPreferredWaveformChangedCallback([this](WaveformMode mode) {
            modem_.setNarrowbandControl(mode == WaveformMode::OFDM_NARROW);
        });

        // Keep the modem's RX address filter in sync with the operator's live callsign. A VARA
        // host (e.g. Winlink Express) issues MYCALL after launch, which can differ from the
        // config callsign the log prefix was seeded with; without this the modem silently drops
        // inbound frames addressed to the new callsign (BUG-CALLSIGN-FILTER).
        bridge_.setLocalCallChangedCallback([this](const std::string& call) {
            modem_.setLocalCallsign(call);
        });

        // RX-side modem->protocol forwarding (raw-data delivery, burst-group delivery,
        // data-sync acceptance, tone-burst ACK detection, HARQ ctx) is wired by the SHARED
        // wireModemToProtocol() in configureModem(). Only the ping handling stays here — it
        // is genuinely frontend-specific (narrowband override + ping.rx diagnostics).
        modem_.setPingReceivedCallback([this](float snr_db) {
            engine_.setMeasuredSNR(snr_db, ultra::SNRSource::SYNC_QUALITY);
            char fields[176];
            std::snprintf(fields, sizeof(fields),
                          "{\"snr_db\":%.1f,\"snr_source\":\"%s\"}",
                          snr_db, ultra::snrSourceToString(ultra::SNRSource::SYNC_QUALITY));
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "ping.rx", fields);
            if (modem_.getDetectedBandwidth() == ultra::BandwidthMode::NARROW) {
                modem_.setNarrowbandControl(true);
                engine_.setNarrowbandOverride(WaveformMode::OFDM_NARROW);
            }
            engine_.onPingReceived();
        });
    }

    void setWaveformMode(WaveformMode mode) {
        tx_waveform_mode_ = mode;
        modem_.setOFDMConfig(ofdm_config_);
        modem_.setWaveformMode(mode);
        modem_.setDataMode(data_modulation_, data_code_rate_);
        modem_.setMCDPSKCarriers(8);
        if (connected_ && mode == WaveformMode::OFDM_CHIRP) {
            modem_.forceNextFrameFullPreamble();
        }
    }

    void setDataMode(Modulation mod, CodeRate rate) {
        data_modulation_ = mod;
        data_code_rate_ = rate;

        ofdm_config_.modulation = mod;
        ofdm_config_.code_rate = rate;
        ofdm_config_.use_pilots = true;
        ofdm_config_.pilot_spacing = ultra::ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);

        if (tx_waveform_mode_ != WaveformMode::MC_DPSK) {
            modem_.setOFDMConfig(ofdm_config_);
            modem_.setWaveformMode(tx_waveform_mode_);
        }
        modem_.setDataMode(mod, rate);
    }

    // R4: short re-anchor removed (superseded by warm-handoff, now the production default).

    void setConnected(bool connected) {
        if (connected_ == connected) {
            return;
        }

        connected_ = connected;
        modem_.setConnected(connected_);
        if (connected_) {
            if (negotiated_waveform_ == WaveformMode::OFDM_NARROW) {
                ofdm_config_ = ultra::presets::narrowbandOFDM();
            } else {
                ofdm_config_ = base_ofdm_config_;
            }
            ofdm_config_.modulation = data_modulation_;
            ofdm_config_.code_rate = data_code_rate_;
            ofdm_config_.use_pilots = true;
            ofdm_config_.pilot_spacing =
                ultra::ofdm_link_adaptation::recommendedPilotSpacing(data_modulation_, data_code_rate_);

            if (negotiated_waveform_ != WaveformMode::MC_DPSK) {
                setWaveformMode(negotiated_waveform_);
                // ModemEngine tracks chirp CFO internally (like the GUI) — no explicit
                // setKnownCFO seeding from the ping is needed.
                modem_.setConnectedOFDMMode(negotiated_waveform_, ofdm_config_,
                                            data_modulation_, data_code_rate_);
                if (negotiated_waveform_ == WaveformMode::OFDM_CHIRP) {
                    modem_.expectFullOFDMAnchorOnce();
                    modem_.forceNextFrameFullPreamble();
                    // NOTE: do NOT force setBurstInterleave here. ModemEngine's mode logic
                    // (modem_mode.cpp) derives burst_interleave_on from the traffic-class
                    // policy as the SINGLE source of truth shared by encoder/ARQ/the on-wire
                    // descriptor — forcing it true (the old raw-decoder TNC did) clamps it ON
                    // even on clean channels where the policy + descriptor say OFF, so the RX
                    // decodes the group on the wrong path (0/6). The GUI never forces it.
                }
            } else {
                modem_.setWaveformMode(WaveformMode::MC_DPSK);
                modem_.setDataMode(data_modulation_, data_code_rate_);
            }
        } else {
            std::lock_guard<std::mutex> lock(input_audio_mutex_);
            if (!cfg_.sim_audio) {
                audio_.pauseInput();
            }
            modem_.reset();
            modem_.setWaveformMode(WaveformMode::MC_DPSK);
            modem_.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
            data_modulation_ = Modulation::DQPSK;
            data_code_rate_ = CodeRate::R1_4;
            negotiated_waveform_ = WaveformMode::MC_DPSK;
            handshake_complete_ = false;
            ofdm_config_ = base_ofdm_config_;
            setWaveformMode(WaveformMode::MC_DPSK);
            if (cfg_.sim_audio) {
                drainOtaRxLocked();
            } else {
                audio_.drainInput();
                audio_.resumeInput();
            }
        }
    }

    // TX is delegated to ModemEngine, which owns the full stateful TX decision (waveform
    // by connection state, modulation/rate, light-vs-full preamble incl. the GROUP_ACK/
    // DISCONNECT full-anchor rules, handshake-mode handling, expected-arrival seeding).
    // This replaces the TNC's former hand-rolled encoder save/restore logic — the GUI
    // drives the identical ModemEngine path.
    std::vector<float> transmitFrame(const Bytes& data) {
        return modem_.transmit(data);
    }

    std::vector<float> transmitBurst(const std::vector<Bytes>& frames,
                                     uint16_t group_seq = 0,
                                     ultra::gui::BurstAnchorOptions anchor_options = {}) {
        // Match the encoder Z to the connection's per-burst traffic-class policy (Z=81 /
        // n=1944 for file bursts) so encoder-Z == chunker-Z == the BURST_HEADER descriptor.
        // The GUI does the identical push (app.cpp); without it file bursts ship at the
        // default Z=27 and the receiver group-decodes 0/6.
        modem_.setBurstLiftingZ(static_cast<uint8_t>(engine_.selectBurstLiftingZ()));
        return modem_.transmitBurst(frames, group_seq, anchor_options);
    }

    std::vector<float> transmitPing() {
        return modem_.transmitPing();
    }

    void queueTx(std::vector<float> samples, bool disconnect_control = false) {
        if (samples.empty() || !output_enabled_) {
            return;
        }
        if (disconnect_teardown_active_.load(std::memory_order_acquire) &&
            !disconnect_control) {
            LOG_WARN("AUDIO", "Teardown egress: blocked TNC TX at audio commit");
            return;
        }

        if (cfg_.sim_audio) {
            if (!ota_audio_ || !ota_audio_->isConnected()) {
                reportOtaStatus(false);
                LOG_WARN("AUDIO", "OTASim TX dropped: audio backend is not connected");
                return;
            }
            const auto measurement =
                ultra::sim::normalizeTxBurstToReference(samples);
            if (measurement.peak_warning || measurement.peak_clip_error) {
                LOG_WARN("AUDIO",
                         "OTASim TX burst normalization peak_after_gain=%.3f "
                         "clip_samples=%zu gain=%.3f active=%zu in_band_rms=%.6f%s",
                         measurement.peak_after_gain,
                         measurement.peak_clip_samples,
                         measurement.gain_to_reference,
                         measurement.active_samples,
                         measurement.in_band_rms,
                         measurement.peak_clip_error ? " CLIP_EXPECTED" : "");
            }
            std::string error;
            const bool queued = ota_audio_->queueTxSamples(samples, &error);
            if (!queued) {
                LOG_WARN("AUDIO", "OTASim TX failed: %s", error.c_str());
                std::cerr << "[otasim] TX failed: " << error << "\n";
            }
            // NOTE: do NOT seed expected-frame-arrival here. ModemEngine::transmit()/
            // transmitBurst() already seed the decoder's next-expected window internally
            // (modem_engine.cpp) using the modem's own turnaround_delay_ms_ — SyncController
            // owns cold/warm hand-off. The old raw-decoder TNC dead-reckoned a second seed
            // at the audio-sink layer; fired again here (after the modem's correct internal
            // seed) it OVERWRITES it — and on the file receiver it seeds "next frame ~725ms
            // after my tone-burst ACK" when the next group is seconds out, pushing warm-sync
            // DEGRADED so the group-start light-LTS falls to a cold full-chirp false-lock
            // (corr~0.6, garbage CFO). The GUI seeds nowhere; the TNC must match.
            return;
        }

        const auto measurement =
            ultra::sim::normalizeTxBurstForHardware(samples, cfg_.tx_drive);
        if (measurement.burst_fragment_warning) {
            LOG_WARN("AUDIO",
                     "Hardware TX peak normalization bypassed fragment: "
                     "active=%zu minimum=%zu samples=%zu target=%.3f",
                     measurement.active_samples,
                     ultra::sim::kTxBurstMinimumActiveSamples,
                     samples.size(),
                     measurement.target_peak);
        }

        audio_.queueTxSamples(samples);
        // No external expected-arrival seed: ModemEngine seeds it internally on transmit
        // (see the OTASim branch above). SyncController owns the cold/warm hand-off.
    }

    void drainOtaRxLocked() {
        if (!ota_audio_) {
            return;
        }
        for (int i = 0; i < 16; ++i) {
            if (ota_audio_->getRxSamples(65536).empty()) {
                break;
            }
        }
    }

    void reportOtaStatus(bool force) {
        if (!cfg_.sim_audio || !ota_audio_) {
            return;
        }
        const auto status = ota_audio_->status();
        if (!force && ota_status_seen_ &&
            status.state == last_ota_state_ &&
            status.text == last_ota_text_) {
            return;
        }
        ota_status_seen_ = true;
        last_ota_state_ = status.state;
        last_ota_text_ = status.text;
        std::cerr << "[otasim] " << status.text << "\n";
        LOG_INFO("AUDIO", "OTASim: %s", status.text.c_str());
    }

};

} // namespace

#ifndef ULTRA_TNC_TESTING
int main(int argc, char** argv) {
    Config cfg;
    if (!ultra::tnc::config::parseArgs(argc, argv, cfg)) {
        ultra::tnc::config::printUsage(std::cerr);
        return 1;
    }

    LogFileHandle log_file(nullptr);
    if (!configureLogging(cfg, log_file)) {
        return 1;
    }

    if (cfg.help) {
        ultra::tnc::config::printUsage(std::cout);
        return 0;
    }

    if (cfg.version) {
        printBuildProvenance(std::cout);
        return 0;
    }

    if (cfg.list_audio) {
        ultra::gui::AudioEngine probe;
        if (!probe.initialize()) {
            std::cerr << "Failed to initialize SDL audio for device listing\n"
                      << "Next step: confirm OS audio permissions and that no other "
                         "process has exclusive control of the sound device.\n";
            return 1;
        }
        std::cout << "\n  Output devices:\n";
        for (const auto& d : probe.getOutputDevices()) std::cout << "    " << d << "\n";
        std::cout << "\n  Input devices:\n";
        for (const auto& d : probe.getInputDevices()) std::cout << "    " << d << "\n";
        std::cout << "\nUse a device's exact name in --audio-output / --audio-input\n"
                  << "or the equivalent config-file keys (audio_output / audio_input).\n";
        probe.shutdown();
        return 0;
    }

    ultra::diagnostics::SessionMeta diag_meta;
    diag_meta.app_name = "ultra_tnc";
    diag_meta.station_role = "tnc";
    diag_meta.callsign = cfg.callsign;
    diag_meta.config_json =
        std::string("{\"app\":\"ultra_tnc\",\"callsign\":\"") +
        ultra::diagnostics::jsonEscape(cfg.callsign) +
        "\",\"bind_address\":\"" + ultra::diagnostics::jsonEscape(cfg.bind_address) +
        "\",\"port\":" + std::to_string(cfg.port) +
        ",\"tx_drive\":" + std::to_string(cfg.tx_drive) +
        ",\"papr_reduction\":" + std::string(cfg.papr_reduction ? "true" : "false") +
        ",\"audio_input\":\"" + ultra::diagnostics::jsonEscape(audioDeviceLabel(cfg.audio_input)) +
        "\",\"audio_output\":\"" + ultra::diagnostics::jsonEscape(audioDeviceLabel(cfg.audio_output)) +
        "\"}";
    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
    diagnostics.start(std::move(diag_meta));
    if (cfg.accept_audio_consent && !diagnostics.hasAudioConsent()) {
        if (diagnostics.grantAudioConsent()) {
            LOG_INFO("OPERATOR",
                     "RX-audio capture consent granted via --accept-audio-consent. "
                     "Marker: %s",
                     diagnostics.consentPath().string().c_str());
        } else {
            LOG_ERROR("OPERATOR",
                      "Failed to write consent marker at %s",
                      diagnostics.consentPath().string().c_str());
        }
    }
    if (auto tombstone = diagnostics.pendingTombstone()) {
        LOG_WARN("OPERATOR",
                 "Previous-session crash tombstone detected: signal=%s session=%s. "
                 "Run ultra_report --create to package a local crash report.",
                 tombstone->signal_name.c_str(), tombstone->session_id.c_str());
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);
#ifndef _WIN32
    std::signal(SIGUSR1, handleReportSignal);
#endif

    ultra::gui::AudioEngine audio;
    ultra::protocol::ProtocolEngine engine;
    ultra::tnc::TNCBridge bridge(engine, audio);
    UltraTNCStation station(cfg, engine, audio, bridge);

    bridge.setMyCall({cfg.callsign});
    bridge.setBandwidth(2300);

    LOG_INFO("OPERATOR", "ultra_tnc starting: callsign=%s cmd=%s:%u data=%u log=%s",
             cfg.callsign.c_str(), cfg.bind_address.c_str(),
             static_cast<unsigned>(cfg.port), static_cast<unsigned>(cfg.port + 1),
             ultra::logLevelName(cfg.log_level));
    LOG_INFO("AUDIO",
             "TX drive (tx_drive=%.3f) now controls per-burst peak target, "
             "not post-mix attenuation. Previous tx_drive = 0.8 behavior is "
             "replaced by per-burst peak normalization to tx_drive's value.",
             cfg.tx_drive);
    LOG_INFO("OPERATOR", "Build: version=%s commit=%s dirty=%s tag=%s built=%s os=%s",
             ultra::kBuildVersion, ultra::kBuildGitCommit,
             ultra::kBuildDirty ? "true" : "false",
             ultra::kBuildReleaseTag[0] ? ultra::kBuildReleaseTag : "none",
             ultra::kBuildTimeUtc, ultra::kBuildOS);

    const ultra::ptt::PttConfig ptt_config = makePttConfig(cfg);
    std::unique_ptr<ultra::ptt::IPttDriver> ptt_driver =
        ultra::ptt::createPttDriver(ptt_config);
    if (ptt_config.mode != ultra::ptt::PttMode::None) {
        if (!ptt_driver->open()) {
            std::cerr << "Failed to open PTT driver: " << ptt_driver->lastError() << "\n"
                      << "Next step: verify the PTT settings, or disable PTT to use "
                         "VOX/external PTT.\n";
            return 1;
        }
        bridge.setPttChangedCallback([driver = ptt_driver.get()](bool on) {
            if (!driver->setKey(on ? ultra::ptt::PttKey::On : ultra::ptt::PttKey::Off)) {
                const std::string error = driver->lastError();
                LOG_ERROR("OPERATOR", "PTT transition failed: %s", error.c_str());
                std::cerr << "[ptt] transition failed: " << error << "\n";
            }
        });
    }

    if (ptt_config.mode == ultra::ptt::PttMode::Serial) {
        std::cout << "Hardware PTT enabled on " << cfg.ptt_serial_port
                  << " @ " << cfg.ptt_serial_baud << " baud, line="
                  << cfg.ptt_serial_line
                  << (cfg.ptt_inactive_high ? " (inverted)" : "") << "\n";
        LOG_INFO("OPERATOR", "PTT: serial %s @ %d baud line=%s%s",
                 cfg.ptt_serial_port.c_str(), cfg.ptt_serial_baud,
                 cfg.ptt_serial_line.c_str(),
                 cfg.ptt_inactive_high ? " inverted" : "");
    } else if (ptt_config.mode == ultra::ptt::PttMode::Cat) {
        std::cout << "Hardware PTT enabled via Hamlib rigctld "
                  << cfg.ptt_cat_host << ":" << cfg.ptt_cat_port << "\n";
        LOG_INFO("OPERATOR", "PTT: CAT rigctld %s:%u",
                 cfg.ptt_cat_host.c_str(), static_cast<unsigned>(cfg.ptt_cat_port));
    } else if (ptt_config.mode == ultra::ptt::PttMode::HamlibBuiltin) {
        std::cout << "Hardware PTT enabled via Hamlib (built-in) model="
                  << cfg.ptt_hamlib_model
                  << (cfg.ptt_hamlib_port.empty() ? ""
                                                  : (" port=" + cfg.ptt_hamlib_port))
                  << " baud=" << cfg.ptt_hamlib_baud
                  << " ptt=" << cfg.ptt_hamlib_ptt << "\n";
        LOG_INFO("OPERATOR",
                 "PTT: Hamlib built-in model=%d port=%s baud=%d ptt=%s",
                 cfg.ptt_hamlib_model,
                 cfg.ptt_hamlib_port.c_str(),
                 cfg.ptt_hamlib_baud,
                 cfg.ptt_hamlib_ptt.c_str());
    } else {
        LOG_INFO("OPERATOR", "PTT: disabled; use VOX or external PTT");
    }

    ultra::tnc::TNCServerConfig server_cfg;
    server_cfg.cmd_port = cfg.port;
    server_cfg.data_port = static_cast<uint16_t>(cfg.port + 1);
    server_cfg.bind_address = cfg.bind_address;
    ultra::tnc::TNCServer server(bridge, server_cfg);

    bridge.attachServer(&server);

    if (!station.start()) {
        return 1;
    }

    bridge.start();

    if (!server.start()) {
        std::cerr << "TNC server bind failed on " << cfg.bind_address << ":" << cfg.port << "\n";
        bridge.stop();
        station.stop();
        return 1;
    }

    std::cout << "ultra_tnc listening on " << cfg.bind_address << ":" << server.getCmdPort()
              << " (data " << server.getDataPort() << ")\n";
    LOG_INFO("OPERATOR", "Listening: cmd=%s:%u data=%u",
             cfg.bind_address.c_str(), static_cast<unsigned>(server.getCmdPort()),
             static_cast<unsigned>(server.getDataPort()));

    auto last_tick = std::chrono::steady_clock::now();
    while (!g_stop_requested.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tick).count();
        last_tick = now;
        const uint32_t elapsed_ms = static_cast<uint32_t>(std::clamp<int64_t>(elapsed, 1, 1000));
        station.tick(elapsed_ms);
        if (g_report_requested.exchange(false, std::memory_order_acq_rel)) {
            ultra::diagnostics::ReportOptions options;
            options.note = "SIGUSR1 manual TNC snapshot";
            auto report = diagnostics.freeze(ultra::diagnostics::FreezeReason::Signal, options);
            if (report.ok) {
                LOG_INFO("OPERATOR", "Diagnostics report created: %s",
                         report.path.string().c_str());
            } else {
                LOG_ERROR("OPERATOR", "Diagnostics report failed: %s", report.error.c_str());
            }
        }
    }

    server.stop();
    bridge.stop();
    ptt_driver->close();
    station.stop();
    diagnostics.stop();
    return 0;
}
#endif
