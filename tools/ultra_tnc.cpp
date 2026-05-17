#include "gui/audio_engine.hpp"
#include "diagnostics/diagnostics_recorder.hpp"
#include "gui/modem/streaming_decoder.hpp"
#include "gui/modem/streaming_encoder.hpp"
#include "otasim_client/ota_audio_backend.hpp"
#include "ptt/ptt_driver_factory.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "protocol/frame_v2.hpp"
#include "protocol/protocol_engine.hpp"
#include "sim/channel_calibration.hpp"
#include "tnc/tnc_bridge.hpp"
#include "tnc/tnc_server.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/build_info.hpp"
#include "waveform/ofdm_cox_waveform.hpp"

#include "ultra_tnc_config.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using ultra::Bytes;
using ultra::CodeRate;
using ultra::ModemConfig;
using ultra::Modulation;
using ultra::gui::DecodeResult;
using ultra::gui::StreamingDecoder;
using ultra::gui::StreamingEncoder;
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
          bridge_(bridge),
          rng_(12345) {
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
            decode_thread_ = std::thread(&UltraTNCStation::decodeLoop, this);
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
        decode_thread_ = std::thread(&UltraTNCStation::decodeLoop, this);
        return true;
    }

    void stop() {
        if (!running_.exchange(false)) {
            return;
        }

        decoder_.stop();
        if (decode_thread_.joinable()) {
            decode_thread_.join();
        }

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
                const size_t target_samples = std::clamp<size_t>(
                    (static_cast<size_t>(elapsed_ms) * kSampleRate) / 1000,
                    1,
                    4096);
                auto samples = ota_audio_->getRxSamples(target_samples);
                samples.resize(target_samples, 0.0f);
                decoder_.feedAudio(samples);
            }
        } else if (input_enabled_) {
            std::lock_guard<std::mutex> lock(input_audio_mutex_);
            auto samples = audio_.getRxSamples(4096);
            if (!samples.empty()) {
                decoder_.feedAudio(samples);
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
        decoder_.feedAudio(samples, count);
    }

    void testProcessDecoder() {
        decoder_.processBuffer();
    }

    ultra::gui::DecoderStats testDecoderStats() const {
        return decoder_.getStats();
    }
#endif

private:
    static constexpr int kSampleRate = 48000;

    const Config& cfg_;
    ultra::protocol::ProtocolEngine& engine_;
    ultra::gui::AudioEngine& audio_;
    ultra::tnc::TNCBridge& bridge_;

    StreamingEncoder encoder_;
    StreamingDecoder decoder_;

    ModemConfig base_ofdm_config_;
    ModemConfig ofdm_config_;
    WaveformMode tx_waveform_mode_ = WaveformMode::MC_DPSK;
    WaveformMode negotiated_waveform_ = WaveformMode::MC_DPSK;
    Modulation data_modulation_ = Modulation::DQPSK;
    CodeRate data_code_rate_ = CodeRate::R1_4;
    float last_cfo_hz_ = 0.0f;

    std::atomic<bool> running_{false};
    std::thread decode_thread_;
    std::mutex input_audio_mutex_;
    bool input_enabled_ = false;
    bool output_enabled_ = false;
    bool handshake_complete_ = false;
    bool connected_ = false;
    int consecutive_decode_failures_ = 0;
    std::unique_ptr<OtaAudioBackend> ota_audio_;
    OtaAudioConnectionState last_ota_state_ = OtaAudioConnectionState::Disconnected;
    std::string last_ota_text_;
    bool ota_status_seen_ = false;

    std::mt19937 rng_;

    ModemConfig createOFDMConfig() const {
        ModemConfig cfg;
        if (cfg_.ofdm_config == OFDMConfigPreset::Nvis) {
            cfg = ultra::OFDMNvisWaveform::createNvisMode()->getConfig();
        } else {
            ultra::OFDMNvisWaveform default_cox;
            cfg = default_cox.getConfig();
        }

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
        engine_.setMeasuredSNR(cfg_.snr_db);
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

        encoder_.setOFDMConfig(ofdm_config_);
        encoder_.setMode(tx_waveform_mode_);
        encoder_.setDataMode(data_modulation_, data_code_rate_);
        encoder_.setMCDPSKCarriers(8);
        encoder_.setFixedFrameCodewords(v2::kDefaultFixedFrameCodewords);

        decoder_.setLogPrefix(cfg_.callsign);
        decoder_.setMode(WaveformMode::MC_DPSK, false);
        decoder_.setMCDPSKCarriers(8);
        decoder_.setFixedFrameCodewords(v2::kDefaultFixedFrameCodewords);
        decoder_.setSoftCombineBuffer(engine_.softCombineBuffer());
    }

    void setupCallbacks() {
        engine_.setTxDataCallback([this](const Bytes& data) {
            queueTx(transmitFrame(data));
        });

        engine_.setTransmitBurstCallback([this](const std::vector<Bytes>& frames) {
            queueTx(transmitBurst(frames));
        });

        engine_.setPingTxCallback([this]() {
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "ping.tx", "{\"kind\":\"ping\"}");
            queueTx(transmitPing());
        });

        engine_.setPingReceivedCallback([this]() {
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "ping.tx", "{\"kind\":\"pong\"}");
            queueTx(transmitPing());
        });

        engine_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate,
                                                  int cw_count,
                                                  float peer_snr_db, float peer_fading,
                                                  int mc_dpsk_num_carriers,
                                                  int mc_dpsk_samples_per_symbol) {
            (void)peer_snr_db;
            (void)peer_fading;
            if (mc_dpsk_num_carriers > 0 && mc_dpsk_samples_per_symbol > 0) {
                ultra::MultiCarrierDPSKConfig cfg;
                cfg.num_carriers = mc_dpsk_num_carriers;
                cfg.samples_per_symbol = mc_dpsk_samples_per_symbol;
                cfg.bits_per_symbol = (mod == Modulation::DBPSK) ? 1 :
                                      (mod == Modulation::D8PSK) ? 3 : 2;
                encoder_.setMCDPSKConfig(cfg);
                decoder_.setMCDPSKConfig(cfg);
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
            encoder_.setFixedFrameCodewords(cw_count);
            decoder_.setFixedFrameCodewords(cw_count);
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
                encoder_.setNarrowbandControl(true);
            }
        });

        engine_.setHandshakeConfirmedCallback([this]() {
            handshake_complete_ = true;
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
            encoder_.setNarrowbandControl(mode == WaveformMode::OFDM_NARROW);
        });

        decoder_.setFrameCallback([this](const DecodeResult& result) {
            handleDecodedFrame(result);
        });

        decoder_.setDataSyncAcceptedCallback([this](float sync_correlation) {
            engine_.onAcceptedOFDMDataSync(sync_correlation);
        });

        decoder_.setPingCallback([this](float snr_db, float cfo_hz) {
            engine_.setMeasuredSNR(snr_db);
            last_cfo_hz_ = cfo_hz;
            char fields[128];
            std::snprintf(fields, sizeof(fields),
                          "{\"snr_db\":%.1f,\"cfo_hz\":%.1f}", snr_db, cfo_hz);
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "ping.rx", fields);
            if (decoder_.getDetectedBandwidth() == ultra::BandwidthMode::NARROW) {
                encoder_.setNarrowbandControl(true);
                engine_.setNarrowbandOverride(WaveformMode::OFDM_NARROW);
            }
            engine_.onPingReceived();
        });
    }

    void handleDecodedFrame(const DecodeResult& result) {
        if (!result.success) {
            consecutive_decode_failures_++;
            char fields[160];
            std::snprintf(fields, sizeof(fields),
                          "{\"consecutive\":%d,\"cw_failed\":%d}",
                          consecutive_decode_failures_, result.codewords_failed);
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "phy", "decode.fail", fields);
            if (consecutive_decode_failures_ == 5) {
                ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                    "fault", "fault.triggered",
                    "{\"reason\":\"repeated_decode_failures\",\"policy\":\"emit_only\"}");
            }
            return;
        }
        consecutive_decode_failures_ = 0;
        if (result.is_ping || result.frame_data.empty()) {
            return;
        }

        last_cfo_hz_ = result.cfo_hz;
        auto header = v2::parseHeader(result.frame_data);
        if (header.valid && !v2::isAddressedToCallsign(header, engine_.getLocalCallsign())) {
            return;
        }

        const float fading_index = decoder_.getLastFadingIndex();
        engine_.setChannelQuality(result.snr_db, fading_index);
        char fields[192];
        std::snprintf(fields, sizeof(fields),
                      "{\"bytes\":%zu,\"snr_db\":%.1f,\"fading\":%.2f}",
                      result.frame_data.size(), result.snr_db, fading_index);
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "phy", "frame.rx", fields);
        engine_.onRxData(result.frame_data);
    }

    void setWaveformMode(WaveformMode mode) {
        tx_waveform_mode_ = mode;
        encoder_.setOFDMConfig(ofdm_config_);
        encoder_.setMode(mode);
        encoder_.setDataMode(data_modulation_, data_code_rate_);
        encoder_.setMCDPSKCarriers(8);
    }

    void setDataMode(Modulation mod, CodeRate rate) {
        data_modulation_ = mod;
        data_code_rate_ = rate;

        ofdm_config_.modulation = mod;
        ofdm_config_.code_rate = rate;
        ofdm_config_.use_pilots = true;
        ofdm_config_.pilot_spacing = ultra::ofdm_link_adaptation::recommendedPilotSpacing(mod, rate);

        if (tx_waveform_mode_ != WaveformMode::MC_DPSK) {
            encoder_.setOFDMConfig(ofdm_config_);
            encoder_.setMode(tx_waveform_mode_);
        }
        encoder_.setDataMode(mod, rate);

        if (negotiated_waveform_ != WaveformMode::MC_DPSK) {
            decoder_.setOFDMConfig(ofdm_config_);
        }
        decoder_.setDataMode(mod, rate);
    }

    void setConnected(bool connected) {
        if (connected_ == connected) {
            return;
        }

        connected_ = connected;
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
                decoder_.setConnectedOFDMMode(negotiated_waveform_, ofdm_config_,
                                              data_modulation_, data_code_rate_);
                decoder_.setKnownCFO(last_cfo_hz_);
                if (negotiated_waveform_ == WaveformMode::OFDM_CHIRP) {
                    encoder_.setBurstInterleave(true);
                    decoder_.setBurstInterleave(true);
                }
            } else {
                decoder_.setMode(WaveformMode::MC_DPSK, true);
                decoder_.setDataMode(data_modulation_, data_code_rate_);
                decoder_.setKnownCFO(last_cfo_hz_);
            }
        } else {
            std::lock_guard<std::mutex> lock(input_audio_mutex_);
            if (!cfg_.sim_audio) {
                audio_.pauseInput();
            }
            decoder_.reset();
            decoder_.setMode(WaveformMode::MC_DPSK, false);
            decoder_.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
            encoder_.setBurstInterleave(false);
            decoder_.setBurstInterleave(false);
            data_modulation_ = Modulation::DQPSK;
            data_code_rate_ = CodeRate::R1_4;
            negotiated_waveform_ = WaveformMode::MC_DPSK;
            handshake_complete_ = false;
            ofdm_config_ = base_ofdm_config_;
            setWaveformMode(WaveformMode::MC_DPSK);
            encoder_.setMode(WaveformMode::MC_DPSK);
            encoder_.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
            last_cfo_hz_ = 0.0f;
            if (cfg_.sim_audio) {
                drainOtaRxLocked();
            } else {
                audio_.drainInput();
                audio_.resumeInput();
            }
        }
    }

    std::vector<float> transmitFrame(const Bytes& data) {
        if (data.empty()) {
            return {};
        }

        bool is_handshake_frame = false;
        if (data.size() >= 3) {
            const uint8_t frame_type = data[2];
            is_handshake_frame = (frame_type == 0x12 || frame_type == 0x13);
        }

        const WaveformMode saved_mode = encoder_.getMode();
        const CodeRate saved_rate = encoder_.getCodeRate();

        if (is_handshake_frame) {
            encoder_.setMode(WaveformMode::MC_DPSK);
            encoder_.setDataMode(Modulation::DQPSK, CodeRate::R1_4);
        }

        const bool is_mc_dpsk = encoder_.getMode() == WaveformMode::MC_DPSK;
        const bool use_light = !is_handshake_frame && !is_mc_dpsk && connected_ && handshake_complete_;
        std::vector<float> samples = use_light ? encoder_.encodeFrameLight(data)
                                               : encoder_.encodeFrame(data);

        if (is_handshake_frame) {
            encoder_.setMode(saved_mode);
            encoder_.setDataMode(data_modulation_, saved_rate);
        }

        return samples;
    }

    std::vector<float> transmitBurst(const std::vector<Bytes>& frames) {
        if (frames.empty()) {
            return {};
        }
        if (tx_waveform_mode_ != WaveformMode::MC_DPSK) {
            encoder_.setMode(tx_waveform_mode_);
            encoder_.setDataMode(data_modulation_, data_code_rate_);
        }
        return encoder_.encodeBurstLight(frames);
    }

    std::vector<float> transmitPing() {
        return encoder_.encodePing();
    }

    void queueTx(std::vector<float> samples) {
        if (samples.empty() || !output_enabled_) {
            return;
        }

        if (cfg_.inject_channel) {
            applyAwgn(samples);
        }

        if (cfg_.sim_audio) {
            if (!ota_audio_ || !ota_audio_->isConnected()) {
                reportOtaStatus(false);
                LOG_WARN("AUDIO", "OTASim TX dropped: audio backend is not connected");
                return;
            }
            std::string error;
            if (!ota_audio_->queueTxSamples(samples, &error)) {
                LOG_WARN("AUDIO", "OTASim TX failed: %s", error.c_str());
                std::cerr << "[otasim] TX failed: " << error << "\n";
            }
            return;
        }

        audio_.queueTxSamples(samples);
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

    void applyAwgn(std::vector<float>& samples) {
        // Continuous AWGN sized to the calibrated modem-reference RMS so
        // --snr means the same broadband SNR as in SimulatedChannel and
        // the GUI simulator. Was previously addAWGN(activeSignalPower)
        // which made silence between bursts artificially quiet.
        const float sigma =
            ultra::sim::modemReferenceNoiseStddev(cfg_.snr_db);
        std::normal_distribution<float> noise(0.0f, sigma);
        for (float& s : samples) {
            s = std::clamp(s + noise(rng_), -1.0f, 1.0f);
        }
    }

    void decodeLoop() {
        while (running_.load()) {
            decoder_.processBuffer();
        }
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
