#include "app.hpp"
#include "diagnostics/diagnostics_recorder.hpp"
#include "startup_trace.hpp"
#include "imgui.h"
#include "ptt/ptt_driver_factory.hpp"
#include "ultra/build_info.hpp"
#include "ultra/logging.hpp"
#include <SDL.h>
#include <cctype>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <cstdarg>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <limits>
#include <deque>
#include <vector>
#include <utility>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(dir) _mkdir(dir)
#else
#define MKDIR(dir) mkdir(dir, 0755)
#endif

namespace ultra {
namespace gui {

using ultra::otasim_client::OtaAudioBackend;
using ultra::otasim_client::OtaAudioBackendConfig;
using ultra::otasim_client::OtaAudioConnectionState;
using ultra::otasim_client::OtaAudioStatus;

// File logger for GUI debugging - writes to logs/gui.log next to binary
// ALL logging (including modem, protocol, etc.) goes to this file
static FILE* g_gui_log_file = nullptr;
static bool g_log_initialized = false;
static std::string g_gui_log_path;

static void initLog() {
    if (g_log_initialized) return;
    g_log_initialized = true;

    // If --log-file was passed (main_gui.cpp set ultra::g_log_file before
    // App is constructed), adopt that as our gui-log sink too. Otherwise
    // every call into guiLog() would overwrite the user's chosen destination
    // with our own logs/gui.log.
    if (ultra::g_log_file != nullptr) {
        g_gui_log_file = ultra::g_log_file;
        g_gui_log_path = "(via --log-file)";
        return;
    }

#ifdef _WIN32
    auto tryOpenLog = [](const char* path) -> FILE* {
        if (!path || path[0] == '\0') {
            return nullptr;
        }

        const char* slash_back = std::strrchr(path, '\\');
        const char* slash_fwd = std::strrchr(path, '/');
        const char* sep = slash_back;
        if (!sep || (slash_fwd && slash_fwd > sep)) {
            sep = slash_fwd;
        }
        if (sep) {
            std::string dir(path, static_cast<size_t>(sep - path));
            if (!dir.empty()) {
                MKDIR(dir.c_str());
            }
        }

        return std::fopen(path, "w");
    };

    if (!g_gui_log_file) {
        g_gui_log_file = tryOpenLog("logs\\gui.log");
        if (g_gui_log_file) {
            g_gui_log_path = "logs\\gui.log";
        }
    }

    if (!g_gui_log_file) {
        g_gui_log_file = tryOpenLog("gui.log");
        if (g_gui_log_file) {
            g_gui_log_path = "gui.log";
        }
    }

    if (!g_gui_log_file) {
        const char* temp = std::getenv("TEMP");
        if (temp && temp[0] != '\0') {
            std::string temp_path = std::string(temp) + "\\ProjectUltra\\gui.log";
            g_gui_log_file = tryOpenLog(temp_path.c_str());
            if (g_gui_log_file) {
                g_gui_log_path = temp_path;
            }
        }
    }
#else
    auto tryOpenLog = [](const std::filesystem::path& path) -> FILE* {
        std::error_code ec;
        if (!path.parent_path().empty()) {
            std::filesystem::create_directories(path.parent_path(), ec);
        }
        return fopen(path.string().c_str(), "w");
    };

    std::vector<std::filesystem::path> candidates;
    candidates.emplace_back(std::filesystem::path("logs") / "gui.log");
    candidates.emplace_back("gui.log");
    if (const char* temp = std::getenv("TMPDIR")) {
        candidates.emplace_back(std::filesystem::path(temp) / "projectultra_gui.log");
    }
    candidates.emplace_back("/tmp/projectultra_gui.log");

    for (const auto& path : candidates) {
        g_gui_log_file = tryOpenLog(path);
        if (g_gui_log_file) {
            g_gui_log_path = path.string();
            break;
        }
    }
#endif

    if (g_gui_log_file) {
        // Redirect ALL logging (modem, protocol, etc.) to this file
        ultra::setLogFile(g_gui_log_file);
        ultra::log(ultra::LogLevel::INFO, "GUI", "File logger initialized: %s",
                   g_gui_log_path.c_str());
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

static std::string trimF32Extension(const std::string& path) {
    if (path.size() >= 4 && path.substr(path.size() - 4) == ".f32") {
        return path.substr(0, path.size() - 4);
    }
    return path;
}

static bool writeF32File(const std::string& path, const std::vector<float>& samples) {
    if (samples.empty()) {
        return false;
    }
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    file.write(reinterpret_cast<const char*>(samples.data()), samples.size() * sizeof(float));
    return file.good();
}

// Convert fading index to channel quality string
// Thresholds aligned with waveform_selection.hpp (2026-02-03)
// Combined index = freq_cv + temporal_cv (includes Doppler spread measurement)
// Calibrated: AWGN ~0.04, Good ~0.62, Moderate ~0.90, Poor ~0.82
static const char* fadingToQuality(float fading) {
    if (fading < 0.15f) return "AWGN";
    if (fading < 0.65f) return "Good";
    if (fading < 1.10f) return "Moderate";
    return "Poor";
}

// Same as above but also sets color for GUI display
static const char* fadingToQualityWithColor(float fading, ImVec4& color) {
    if (fading < 0.15f) {
        color = ImVec4(0.0f, 1.0f, 0.5f, 1.0f);  // Cyan
        return "AWGN";
    } else if (fading < 0.65f) {
        color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        return "Good";
    } else if (fading < 1.10f) {
        color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        return "Moderate";
    } else {
        color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
        return "Poor";
    }
}

// User-friendly waveform name (hides internal variants like OFDM_COX/OFDM_CHIRP)
static const char* waveformDisplayName(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK: return "MC-DPSK";
        case protocol::WaveformMode::MFSK: return "MFSK";
        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW: return "OTFS";
        case protocol::WaveformMode::OFDM_NARROW: return "OFDM Narrow";
        case protocol::WaveformMode::OFDM_CHIRP:
        case protocol::WaveformMode::OFDM_COX:
        default: return "OFDM";
    }
}

static uint32_t safeFileSizeBytes(const std::string& path) {
    std::error_code ec;
    uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec || size > static_cast<uintmax_t>(std::numeric_limits<uint32_t>::max())) {
        return 0;
    }
    return static_cast<uint32_t>(size);
}

static const char* imageFormatName(ImageFormat fmt) {
    switch (fmt) {
        case ImageFormat::JPEG: return "JPEG";
        case ImageFormat::PNG: return "PNG";
        case ImageFormat::Unknown:
        default: return "Unknown";
    }
}

static std::string formatByteCount(uint64_t bytes) {
    char buf[64];
    if (bytes >= 1024ull * 1024ull) {
        std::snprintf(buf, sizeof(buf), "%.1f MB",
                      static_cast<double>(bytes) / (1024.0 * 1024.0));
    } else if (bytes >= 1024ull) {
        std::snprintf(buf, sizeof(buf), "%llu KB",
                      static_cast<unsigned long long>((bytes + 1023ull) / 1024ull));
    } else {
        std::snprintf(buf, sizeof(buf), "%llu B",
                      static_cast<unsigned long long>(bytes));
    }
    return buf;
}

static std::string formatWireTime(uint64_t bytes, int bps) {
    if (bps <= 0) {
        return "n/a";
    }

    const auto seconds = static_cast<uint64_t>(
        std::ceil((static_cast<long double>(bytes) * 8.0L) /
                  static_cast<long double>(bps)));
    char buf[64];
    if (seconds < 60) {
        std::snprintf(buf, sizeof(buf), "%llus",
                      static_cast<unsigned long long>(seconds));
    } else if (seconds < 3600) {
        std::snprintf(buf, sizeof(buf), "%llum:%02llus",
                      static_cast<unsigned long long>(seconds / 60),
                      static_cast<unsigned long long>(seconds % 60));
    } else {
        std::snprintf(buf, sizeof(buf), "%lluh:%02llum",
                      static_cast<unsigned long long>(seconds / 3600),
                      static_cast<unsigned long long>((seconds % 3600) / 60));
    }
    return buf;
}

template <size_t N>
static size_t boundedCStringLen(const char (&buf)[N]) {
    const void* term = std::memchr(buf, '\0', N);
    return term ? static_cast<size_t>(static_cast<const char*>(term) - buf) : N;
}

static ptt::HamlibPttMethod hamlibPttMethodFromSettings(int method) {
    switch (method) {
        case 0: return ptt::HamlibPttMethod::Vox;
        case 2: return ptt::HamlibPttMethod::DTR;
        case 3: return ptt::HamlibPttMethod::RTS;
        case 1:
        default: return ptt::HamlibPttMethod::Cat;
    }
}

static float codeRateValue(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return 0.25f;
        case CodeRate::R1_2: return 0.50f;
        case CodeRate::R2_3: return 2.0f / 3.0f;
        case CodeRate::R3_4: return 0.75f;
        default: return 0.25f;
    }
}

static float modulationBitsPerSymbol(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK: return 1.0f;
        case Modulation::QPSK:
        case Modulation::DQPSK: return 2.0f;
        case Modulation::D8PSK:
        case Modulation::QAM8: return 3.0f;
        case Modulation::QAM16: return 4.0f;
        case Modulation::QAM32: return 5.0f;
        case Modulation::QAM64: return 6.0f;
        default: return 1.0f;
    }
}

static float modeEfficiency(Modulation mod, CodeRate rate) {
    return modulationBitsPerSymbol(mod) * codeRateValue(rate);
}

static const char* adaptationDirection(Modulation from_mod, CodeRate from_rate,
                                       Modulation to_mod, CodeRate to_rate) {
    float from_eff = modeEfficiency(from_mod, from_rate);
    float to_eff = modeEfficiency(to_mod, to_rate);
    if (to_eff > from_eff + 0.05f) return "improving";
    if (to_eff < from_eff - 0.05f) return "degrading";
    return "changing";
}

App::App() : App(Options{}) {}

App::App(const Options& opts) : options_(opts), simulation_enabled_(opts.enable_sim) {
    ultra::gui::startupTrace("App", "ctor-body-enter");
    ultra::gui::startupTrace("App", "gui-log-enter");
    guiLog("=== GUI Started ===");
    ultra::gui::startupTrace("App", "gui-log-exit");
    if (options_.record_audio) {
        recording_enabled_ = true;
        guiLog("Recording enabled (-rec): base path '%s'", options_.record_path.c_str());
    }
    // Load persistent settings
    ultra::gui::startupTrace("App", "settings-load-enter");
    settings_.load();
    ultra::gui::startupTrace("App", "settings-load-exit");

    ultra::diagnostics::SessionMeta diag_meta;
    diag_meta.app_name = "ultra_gui";
    diag_meta.station_role = "gui";
    diag_meta.callsign = std::string(settings_.callsign, boundedCStringLen(settings_.callsign));
    diag_meta.config_json =
        std::string("{\"app\":\"ultra_gui\",\"callsign\":\"") +
        ultra::diagnostics::jsonEscape(diag_meta.callsign) +
        "\",\"input_device\":\"" + ultra::diagnostics::jsonEscape(settings_.input_device) +
        "\",\"output_device\":\"" + ultra::diagnostics::jsonEscape(settings_.output_device) +
        "\"}";
    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
    diagnostics.start(std::move(diag_meta));
    if (auto tombstone = diagnostics.pendingTombstone()) {
        appendRxLogLine("[FAULT] Previous-session crash detected: " + tombstone->signal_name);
    }

    ultra::gui::startupTrace("App", "presets-balanced-enter");
    config_ = presets::balanced();
    ultra::gui::startupTrace("App", "presets-balanced-exit");

    // Use dedicated RX decode thread by default.
    modem_.setSynchronousMode(false);

    if (!options_.disable_waterfall) {
        ultra::gui::startupTrace("App", "waterfall-create-begin");
        waterfall_ = std::make_unique<WaterfallWidget>();
        ultra::gui::startupTrace("App", "waterfall-create-end");
    } else {
        guiLog("Waterfall disabled by startup option");
    }

    // Initialize protocol with saved callsign. In -sim mode the OTASim
    // --station-id is what the peer uses to address us at the protocol
    // level (the GUI's "Connect to <remote>" passes the station id); so
    // force the modem's local callsign to match, otherwise deliverFrame()
    // would drop every incoming frame as "different station".
    ultra::gui::startupTrace("App", "callsign-init-enter");
    std::string local_call;
    if (simulation_enabled_ && !options_.station_id.empty()) {
        local_call = options_.station_id;
    } else {
        size_t local_call_len = boundedCStringLen(settings_.callsign);
        if (local_call_len > 0) {
            local_call.assign(settings_.callsign, local_call_len);
        }
    }
    if (!local_call.empty()) {
        ultra::gui::startupTrace("App", "callsign-set-protocol-enter");
        protocol_.setLocalCallsign(local_call);
        ultra::gui::startupTrace("App", "callsign-set-protocol-exit");
        ultra::gui::startupTrace("App", "callsign-set-modem-enter");
        modem_.setLogPrefix(local_call);
        ultra::gui::startupTrace("App", "callsign-set-modem-exit");
    }
    ultra::gui::startupTrace("App", "callsign-init-exit");

    // Set up raw data callback for protocol layer
    ultra::gui::startupTrace("App", "set-raw-callback-enter");
    modem_.setRawDataCallback([this](const Bytes& data) {
        guiLog("Our modem decoded %zu bytes", data.size());
        // Monitor mode: surface every decoded frame's payload in the
        // RX log regardless of addressing. The protocol layer would
        // otherwise drop frames whose dst hash doesn't match local
        // call, which makes the GUI silent on OTA captures from
        // peers we have no relationship with.
        if (!options_.monitor_mode.empty()) {
            if (auto df = protocol::v2::DataFrame::deserialize(data)) {
                std::string text;
                text.reserve(df->payload.size());
                for (uint8_t b : df->payload) {
                    text.push_back((b >= 0x20 && b < 0x7F)
                                   ? static_cast<char>(b) : '.');
                }
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                              "[monitor] frame seq=%u src=0x%06X dst=0x%06X "
                              "len=%u: \"%.180s\"",
                              df->seq, df->src_hash, df->dst_hash,
                              df->payload_len, text.c_str());
                appendRxLogLine(buf);
            } else {
                char buf[96];
                std::snprintf(buf, sizeof(buf),
                              "[monitor] non-data frame, %zu bytes", data.size());
                appendRxLogLine(buf);
            }
        }
        // Update protocol layer with current SNR and fading before processing frame.
        float snr_db = modem_.getStats().snr_db;
        float fading = modem_.getFadingIndex();
        protocol_.setMeasuredSNR(snr_db);
        protocol_.setChannelQuality(snr_db, fading);
        protocol_.onRxData(data);
        updateAdaptiveAdvisory(snr_db, fading);
    });
    ultra::gui::startupTrace("App", "set-raw-callback-exit");

    modem_.setDataSyncAcceptedCallback([this](float sync_correlation) {
        protocol_.onAcceptedOFDMDataSync(sync_correlation);
    });

    // Set up status callback to show codeword progress in RX log
    ultra::gui::startupTrace("App", "set-status-callback-enter");
    modem_.setStatusCallback([this](const std::string& status) {
        appendRxLogLine(status);
    });
    ultra::gui::startupTrace("App", "set-status-callback-exit");

    // Monitor-mode override: if --monitor-ofdm / --monitor-mcdpsk was
    // passed on the CLI, skip the normal PING/CONNECT handshake and
    // force the decoder into the requested waveform/rate immediately.
    // Useful for observing OTA recordings via a virtual audio cable
    // (e.g. WebSDR → BlackHole → GUI) without needing a peer to
    // handshake with. We pretend the modem is already CONNECTED so
    // the OFDM data path activates and the waterfall + frame log
    // light up on incoming OTA frames.
    if (!options_.monitor_mode.empty()) {
        auto parseRate = [](const std::string& s) -> CodeRate {
            if (s == "r1_4") return CodeRate::R1_4;
            if (s == "r1_2") return CodeRate::R1_2;
            if (s == "r2_3") return CodeRate::R2_3;
            if (s == "r3_4") return CodeRate::R3_4;
            return CodeRate::R1_4;
        };
        auto parseMod = [](const std::string& s) -> Modulation {
            if (s == "dqpsk") return Modulation::DQPSK;
            if (s == "qpsk") return Modulation::QPSK;
            if (s == "d8psk") return Modulation::D8PSK;
            if (s == "dbpsk") return Modulation::DBPSK;
            return Modulation::DQPSK;
        };
        if (options_.monitor_mode == "mc_dpsk") {
            modem_.setWaveformMode(protocol::WaveformMode::MC_DPSK);
            appendRxLogLine("[monitor] MC-DPSK mode forced; handshake skipped");
        } else {
            const auto wf = (options_.monitor_mode == "ofdm_narrow")
                ? protocol::WaveformMode::OFDM_NARROW
                : protocol::WaveformMode::OFDM_CHIRP;
            const auto rate = parseRate(options_.monitor_rate);
            const auto mod = parseMod(options_.monitor_modulation);
            modem_.setDataMode(mod, rate);
            modem_.setWaveformMode(wf);
            modem_.setConnected(true);  // forces decoder into connected OFDM data path
            // Match what cli_simulator + decode_bench do post-handshake:
            // OFDM data frames are fixed 4-CW. Without this the decoder
            // treats incoming as 1-CW control frames and falsely rejects
            // the data symbols past the first ~9 OFDM symbols as noise.
            modem_.setFixedFrameCodewords(4);
            appendRxLogLine("[monitor] " + options_.monitor_mode + " forced; "
                            + options_.monitor_modulation + " "
                            + options_.monitor_rate + "; 4-CW fixed; handshake skipped");
        }
    }

    // Set up protocol engine callbacks
    ultra::gui::startupTrace("App", "protocol-callbacks-enter");
    protocol_.setTxDataCallback([this](const Bytes& data) {
        // When protocol layer wants to transmit, convert to audio
        auto samples = modem_.transmit(data);
        if (!samples.empty()) {
            queueRealTxSamples(samples, "TX audio");
        }
    });

    // Burst TX callback - encode multiple frames as a single waveform burst
    protocol_.setTransmitBurstCallback([this](const std::vector<Bytes>& frames) {
        auto samples = modem_.transmitBurst(frames);
        if (!samples.empty()) {
            queueRealTxSamples(samples, "TX burst audio");
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid1");

    protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        // Received a message via ARQ
        std::string msg = "[RX " + from + "] " + text;
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid2");

    protocol_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        guiLog("Connection state changed: %d (%s)", static_cast<int>(state), info.c_str());

        // Update modem engine connection state (affects waveform selection)
        // Stay "connected" during DISCONNECTING so we can receive the ACK via OFDM
        bool modem_connected = (state == protocol::ConnectionState::CONNECTED ||
                                state == protocol::ConnectionState::DISCONNECTING);

        // BUG-TNC-SESSION-001 fix (port from tools/ultra_tnc.cpp): on
        // transition to DISCONNECTED, quiesce the audio input producer
        // before the modem performs its decoder/encoder reset, then drain
        // any kernel-queued capture samples and resume. Without this, a
        // multi-megasample capture backlog can dump into the freshly
        // reset decoder and bury the next session's CONNECT_ACK window.
        const bool wrap_audio_quiesce =
            (state == protocol::ConnectionState::DISCONNECTED) && was_modem_connected_;
        if (wrap_audio_quiesce) {
            audio_.pauseInput();
        }
        modem_.setConnected(modem_connected);
        if (wrap_audio_quiesce) {
            audio_.drainInput();
            audio_.resumeInput();
        }
        was_modem_connected_ = modem_connected;

        std::string msg;
        switch (state) {
            case protocol::ConnectionState::PROBING:
                resetAdaptiveAdvisory();
                msg = "[SYS] Probing " + info + "...";
                {
                    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                    diagnostics.ensureSessionActive();
                    diagnostics.emitText("session", "session.state", "{\"state\":\"probing\"}");
                }
                break;
            case protocol::ConnectionState::CONNECTING:
                resetAdaptiveAdvisory();
                msg = "[SYS] Connecting to " + info + "...";
                {
                    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                    diagnostics.ensureSessionActive();
                    diagnostics.emitText("session", "session.state", "{\"state\":\"connecting\"}");
                }
                break;
            case protocol::ConnectionState::CONNECTED:
                resetAdaptiveAdvisory();
                msg = "[SYS] Connected to " + info;  // info contains remote callsign
                {
                    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                    diagnostics.ensureSessionActive();
                    diagnostics.emitText("session", "session.state", "{\"state\":\"connected\"}");
                }
                break;
            case protocol::ConnectionState::DISCONNECTING:
                msg = "[SYS] Disconnecting...";
                ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                    "session", "session.state", "{\"state\":\"disconnecting\"}");
                break;
            case protocol::ConnectionState::DISCONNECTED:
                resetAdaptiveAdvisory();
                if (info.find("timeout") != std::string::npos) {
                    msg = "[FAILED] " + info;  // Make failures more visible
                } else {
                    msg = "[SYS] Disconnected" + (info.empty() ? "" : ": " + info);
                }
                // Reset waveform mode to OFDM when disconnected
                modem_.setWaveformMode(protocol::WaveformMode::OFDM_COX);
                // Reset connect waveform to DPSK for next connection attempt
                modem_.setConnectWaveform(protocol::WaveformMode::MC_DPSK);
                {
                    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                    const std::string fields =
                        std::string("{\"state\":\"disconnected\",\"reason\":\"") +
                        ultra::diagnostics::jsonEscape(info) + "\"}";
                    diagnostics.emitText("session", "session.state", fields.c_str());
                    auto summary = diagnostics.finishSession(info);
                    if (summary.ok) {
                        diagnostics_last_summary_ = summary.text;
                        diagnostics_last_summary_path_ = summary.path.string();
                        std::snprintf(diagnostics_debrief_save_path_,
                                      sizeof(diagnostics_debrief_save_path_),
                                      "%s", diagnostics_last_summary_path_.c_str());
                        diagnostics_debrief_status_.clear();
                        diagnostics_debrief_popup_open_ = true;
                        appendRxLogLine("[DIAG] Debrief saved: " + diagnostics_last_summary_path_);
                    } else {
                        appendRxLogLine("[DIAG] Debrief failed: " + summary.error);
                    }
                }
                break;
        }
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid3");

    protocol_.setIncomingCallCallback([this](const std::string& from) {
        {
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending_incoming_call_ = from;
        }
        std::string msg = "[SYS] Incoming call from " + from;
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid4");

    // PING TX callback - protocol wants to send a fast presence probe
    protocol_.setPingTxCallback([this]() {
        guiLog("TX PING: Probing for remote station...");
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "ping.tx", "{\"kind\":\"ping\"}");
        auto samples = modem_.transmitPing();
        if (!samples.empty()) {
            queueRealTxSamples(samples, "PING audio");
        }
    });

    // PING received callback - someone is probing us
    protocol_.setPingReceivedCallback([this]() {
        guiLog("RX PING: Incoming probe, sending PONG...");
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "ping.tx", "{\"kind\":\"pong\"}");
        auto samples = modem_.transmitPong();
        if (!samples.empty()) {
            queueRealTxSamples(samples, "PONG audio");
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid5");

    // Wire up modem ping detection to protocol
    modem_.setPingReceivedCallback([this](float snr) {
        float display_snr = snr;

        // Note: Fading index not shown here - it's only reliable after decoding data frames
        // Check state to show appropriate message
        if (protocol_.getState() == protocol::ConnectionState::PROBING) {
            guiLog("RX PONG: Remote station responded! (SNR=%.1f dB)", display_snr);
            // Add to message log so user sees it in the app
            char buf[80];
            snprintf(buf, sizeof(buf), "[PONG] Station responded (SNR=%.0f dB)", display_snr);
            appendRxLogLine(buf);
        } else {
            guiLog("MODEM: Detected PING/PONG (SNR=%.1f dB)", display_snr);
        }
        char diag_fields[96];
        std::snprintf(diag_fields, sizeof(diag_fields), "{\"snr_db\":%.1f}", display_snr);
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "ping.rx", diag_fields);
        // If narrowband chirp detected, set session-scoped override so negotiateMode() picks OFDM_NARROW
        if (modem_.isNarrowbandDetected()) {
            protocol_.setNarrowbandOverride(protocol::WaveformMode::OFDM_NARROW);
        }
        protocol_.onPingReceived();
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid6");

    protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate,
                                                 int cw_count,
                                                 float snr_db, float peer_fading,
                                                 int mc_dpsk_num_carriers,
                                                 int mc_dpsk_samples_per_symbol) {
        if (mc_dpsk_num_carriers > 0 && mc_dpsk_samples_per_symbol > 0) {
            modem_.setMCDPSKProfile(mc_dpsk_num_carriers,
                                    mc_dpsk_samples_per_symbol,
                                    mod == Modulation::DBPSK ? 1 :
                                    mod == Modulation::D8PSK ? 3 : 2);
        }
        // Update modem engine with new data mode
        modem_.setDataMode(mod, rate);
        // Sync ModemEngine encoder/decoder to negotiated CW count from the
        // wire. DO NOT call protocol_.setForcedFrameCodewords here — the
        // engine mutex is held while this callback runs and re-entry will
        // deadlock (caught 2026-05-04 in cli_simulator A/B with seed=1).
        modem_.setFixedFrameCodewords(cw_count);
        resetAdaptiveAdvisory();

        // Local estimate for operator visibility/debugging.
        auto waveform = modem_.getWaveformMode();
        float local_fading = modem_.getFadingIndex();

        const char* local_quality = fadingToQuality(local_fading);
        bool peer_fading_valid = (peer_fading >= 0.0f);
        const char* peer_quality = peer_fading_valid ? fadingToQuality(peer_fading) : "n/a";
        char peer_fading_text[32];
        if (peer_fading_valid) {
            snprintf(peer_fading_text, sizeof(peer_fading_text), "%.2f %s", peer_fading, peer_quality);
        } else {
            snprintf(peer_fading_text, sizeof(peer_fading_text), "n/a");
        }
        const char* wf_name = waveformDisplayName(waveform);
        guiLog("MODE_CHANGE: %s %s %s (peer_snr=%.1f dB, peer_fading=%s, local_fading=%.2f %s)",
               wf_name, modulationToString(mod), codeRateToString(rate),
               snr_db, peer_fading_text,
               local_fading, local_quality);
        char diag_fields[224];
        snprintf(diag_fields, sizeof(diag_fields),
                 "{\"waveform\":\"%s\",\"mod\":\"%s\",\"rate\":\"%s\",\"cw\":%d}",
                 wf_name, modulationToString(mod), codeRateToString(rate), cw_count);
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "waveform.negotiated", diag_fields);

        // Format display with waveform info and channel quality
        char buf[200];
        if (waveform == protocol::WaveformMode::MC_DPSK) {
            snprintf(buf, sizeof(buf),
                     "[MODE] MC-DPSK 8 carriers %s (peer SNR=%d dB, peer fading=%s, local fading=%.2f %s)",
                     codeRateToString(rate), static_cast<int>(snr_db),
                     peer_fading_text,
                     local_fading, local_quality);
        } else {
            snprintf(buf, sizeof(buf),
                     "[MODE] %s %s %s (peer SNR=%d dB, peer fading=%s, local fading=%.2f %s)",
                     wf_name, modulationToString(mod), codeRateToString(rate),
                     static_cast<int>(snr_db), peer_fading_text,
                     local_fading, local_quality);
        }
        appendRxLogLine(buf);

        // Advisory-only peer view (does not change mode yet).
        if (peer_fading_valid) {
            Modulation peer_mod = mod;
            CodeRate peer_rate = rate;
            protocol::recommendDataMode(snr_db, waveform, peer_mod, peer_rate, peer_fading);
            bool peer_change = (peer_mod != mod || peer_rate != rate);

            char adpt_buf[220];
            if (peer_change) {
                const char* direction = adaptationDirection(mod, rate, peer_mod, peer_rate);
                snprintf(adpt_buf, sizeof(adpt_buf),
                         "[ADPT] Peer reports %s conditions (SNR=%.1f dB, F.I.=%.2f): %s -> %s %s",
                         direction, snr_db, peer_fading,
                         direction, modulationToString(peer_mod), codeRateToString(peer_rate));
            } else {
                snprintf(adpt_buf, sizeof(adpt_buf),
                         "[ADPT] Peer reports stable conditions (SNR=%.1f dB, F.I.=%.2f): keep %s %s",
                         snr_db, peer_fading, modulationToString(mod), codeRateToString(rate));
            }

            guiLog("%s", adpt_buf);
            appendRxLogLine(adpt_buf);
        }
    });
    protocol_.setPhyMaskV1NegotiatedCallback([this](bool enabled) {
        modem_.setCarrierLdpcInterleaver(enabled);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid7");

    // Waveform mode negotiation callback.
    protocol_.setModeNegotiatedCallback([this](protocol::WaveformMode mode) {
        std::string mode_name;
        switch (mode) {
            case protocol::WaveformMode::MC_DPSK: mode_name = "MC-DPSK 8 carriers"; break;
            case protocol::WaveformMode::MFSK: mode_name = "MFSK"; break;
            case protocol::WaveformMode::OTFS_EQ: mode_name = "OTFS"; break;
            case protocol::WaveformMode::OTFS_RAW: mode_name = "OTFS"; break;
            case protocol::WaveformMode::OFDM_CHIRP: mode_name = "OFDM"; break;
            case protocol::WaveformMode::OFDM_NARROW: mode_name = "OFDM Narrow"; break;
            case protocol::WaveformMode::OFDM_COX: mode_name = "OFDM"; break;
            default: mode_name = "OFDM"; break;
        }
        guiLog("WAVEFORM_CHANGE: %s", mode_name.c_str());
        char diag_fields[128];
        snprintf(diag_fields, sizeof(diag_fields),
                 "{\"waveform\":\"%s\"}", mode_name.c_str());
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "waveform.negotiated", diag_fields);

        // Update modem engine with new waveform mode
        modem_.setWaveformMode(mode);

        std::string msg = "[WAVEFORM] " + mode_name;
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid8");

    // Connect waveform callback.
    protocol_.setConnectWaveformChangedCallback([this](protocol::WaveformMode mode) {
        const char* mode_name = (mode == protocol::WaveformMode::MFSK) ? "MFSK" : "DPSK";
        guiLog("CONNECT_WAVEFORM: Switching to %s for connection attempts", mode_name);
        modem_.setConnectWaveform(mode);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid9");

    // Handshake confirmed callback - now safe to use negotiated waveform
    protocol_.setHandshakeConfirmedCallback([this]() {
        guiLog("HANDSHAKE: Confirmed, switching to negotiated waveform");
        modem_.setHandshakeComplete(true);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid10");

    // File transfer callbacks
    protocol_.setFileProgressCallback([this](const protocol::FileTransferProgress& p) {
        // Start timing on first progress (for receiving files)
        if (last_progress_milestone_ == 0 && !p.is_sending) {
            file_transfer_start_time_ = std::chrono::steady_clock::now();
        }

        // Log progress milestones (25%, 50%, 75%)
        int pct = static_cast<int>(p.percentage());
        int milestone = (pct / 25) * 25;  // Round down to 25, 50, 75
        if (milestone > 0 && milestone < 100 && milestone > last_progress_milestone_) {
            last_progress_milestone_ = milestone;
            std::string msg = "[FILE] " + std::string(p.is_sending ? "TX" : "RX") +
                              " " + std::to_string(p.transferred_bytes) + "/" +
                              std::to_string(p.total_bytes) + " bytes (" +
                              std::to_string(milestone) + "%)";
            appendRxLogLine(msg);
        }
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid11");

    protocol_.setFileReceivedCallback([this](const std::string& path, bool success) {
        last_progress_milestone_ = 0;  // Reset for next transfer
        auto duration = std::chrono::steady_clock::now() - file_transfer_start_time_;
        float seconds = std::chrono::duration<float>(duration).count();
        uint32_t file_bytes = success ? safeFileSizeBytes(path) : 0;

        std::string msg;
        if (success) {
            if (seconds > 0.0f && file_bytes > 0) {
                last_effective_goodput_bps_ = (8.0f * static_cast<float>(file_bytes)) / seconds;
                last_goodput_label_ = "RX file";
            }
            char fields[512];
            std::snprintf(fields, sizeof(fields),
                          "{\"direction\":\"rx\",\"path\":\"%s\",\"bytes\":%u,"
                          "\"seconds\":%.1f,\"success\":true}",
                          ultra::diagnostics::jsonEscape(path).c_str(),
                          file_bytes,
                          seconds);
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer", fields);
            char buf[320];
            if (last_goodput_label_ == "RX file" && last_effective_goodput_bps_ > 0.0f) {
                snprintf(buf, sizeof(buf), "[FILE] Received: %s (%.1fs, %.2f kbps)",
                         path.c_str(), seconds, last_effective_goodput_bps_ / 1000.0f);
            } else {
                snprintf(buf, sizeof(buf), "[FILE] Received: %s (%.1fs)", path.c_str(), seconds);
            }
            msg = buf;
            last_received_file_ = path;
        } else {
            msg = "[FILE] Receive failed";
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer",
                "{\"direction\":\"rx\",\"success\":false,\"error\":\"receive_failed\"}");
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "fault", "fault.triggered",
                "{\"reason\":\"file_receive_failed\",\"policy\":\"emit_only\"}");
        }
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-mid12");

    protocol_.setFileSentCallback([this](bool success, const std::string& error) {
        last_progress_milestone_ = 0;  // Reset for next transfer
        auto duration = std::chrono::steady_clock::now() - file_transfer_start_time_;
        float seconds = std::chrono::duration<float>(duration).count();

        std::string msg;
        if (success) {
            if (seconds > 0.0f && pending_file_tx_payload_bytes_ > 0) {
                last_effective_goodput_bps_ =
                    (8.0f * static_cast<float>(pending_file_tx_payload_bytes_)) / seconds;
                last_goodput_label_ = "TX file";
            }
            char fields[512];
            std::snprintf(fields, sizeof(fields),
                          "{\"direction\":\"tx\",\"path\":\"%s\",\"bytes\":%u,"
                          "\"seconds\":%.1f,\"success\":true}",
                          ultra::diagnostics::jsonEscape(pending_file_tx_path_).c_str(),
                          pending_file_tx_payload_bytes_,
                          seconds);
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer", fields);
            char buf[196];
            if (last_goodput_label_ == "TX file" && last_effective_goodput_bps_ > 0.0f) {
                snprintf(buf, sizeof(buf), "[FILE] Transfer complete (%.1fs, %.2f kbps)",
                         seconds, last_effective_goodput_bps_ / 1000.0f);
            } else {
                snprintf(buf, sizeof(buf), "[FILE] Transfer complete (%.1fs)", seconds);
            }
            msg = buf;
        } else {
            msg = "[FILE] Transfer failed: " + error;
            const std::string fields =
                std::string("{\"direction\":\"tx\",\"success\":false,\"error\":\"") +
                ultra::diagnostics::jsonEscape(error) + "\"}";
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer", fields.c_str());
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "fault", "fault.triggered",
                "{\"reason\":\"file_transfer_failed\",\"policy\":\"emit_only\"}");
        }
        pending_file_tx_payload_bytes_ = 0;
        pending_file_tx_path_.clear();
        appendRxLogLine(msg);
    });
    ultra::gui::startupTrace("App", "protocol-callbacks-exit");

    // Set receive directory from settings (defaults to Downloads folder)
    ultra::gui::startupTrace("App", "set-rx-dir-enter");
    protocol_.setReceiveDirectory(settings_.getReceiveDirectory());
    ultra::gui::startupTrace("App", "set-rx-dir-exit");

    // Configure waterfall display
    ultra::gui::startupTrace("App", "waterfall-config-enter");
    if (waterfall_) {
        waterfall_->setSampleRate(48000.0f);
        waterfall_->setFrequencyRange(0.0f, 3000.0f);
        waterfall_->setDynamicRange(-60.0f, 0.0f);
    }
    ultra::gui::startupTrace("App", "waterfall-config-exit");

    // Settings window callbacks
    ultra::gui::startupTrace("App", "settings-callbacks-enter");
    settings_window_.setCallsignChangedCallback([this](const std::string& call) {
        protocol_.setLocalCallsign(call);
        modem_.setLogPrefix(call);
        settings_.save();
    });

    settings_window_.setAudioResetCallback([this]() {
        releasePtt("audio_reset");
        closePtt();
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
            audio_.setOutputGain(settings_.tx_drive);
            startRadioRx();
        }
    });

    settings_window_.setClosedCallback([this]() {
        settings_.save();
        releasePtt("settings_closed");
        closePtt();

        if (radio_rx_enabled_) {
            stopRadioRx();
        }
        audio_.stopPlayback();
        audio_.stopCapture();
        audio_.closeInput();
        audio_.closeOutput();

        if (audio_initialized_) {
            audio_.setOutputGain(settings_.tx_drive);
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

    settings_window_.setExpertSettingsChangedCallback([this](uint8_t waveform, uint8_t modulation, uint8_t code_rate) {
        // Apply forced settings to protocol (used on next connect)
        protocol_.setPreferredMode(static_cast<protocol::WaveformMode>(waveform));
        protocol_.setForcedModulation(static_cast<Modulation>(modulation));
        protocol_.setForcedCodeRate(static_cast<CodeRate>(code_rate));
        // Switch encoder chirps to narrowband when OFDM_NARROW is forced
        modem_.setNarrowbandControl(waveform == static_cast<uint8_t>(protocol::WaveformMode::OFDM_NARROW));
        settings_.save();
    });

    settings_window_.setPttTestCallback([this](const AppSettings& snapshot) {
        return testPtt(snapshot);
    });
    settings_window_.setCatTestCallback([this](const AppSettings& snapshot) {
        return testCat(snapshot);
    });
    ultra::gui::startupTrace("App", "settings-callbacks-exit");

    // Apply initial expert settings from loaded config
    ultra::gui::startupTrace("App", "apply-expert-enter");
    protocol_.setPreferredMode(static_cast<protocol::WaveformMode>(settings_.forced_waveform));
    protocol_.setForcedModulation(static_cast<Modulation>(settings_.forced_modulation));
    protocol_.setForcedCodeRate(static_cast<CodeRate>(settings_.forced_code_rate));
    modem_.setNarrowbandControl(settings_.forced_waveform == static_cast<uint8_t>(protocol::WaveformMode::OFDM_NARROW));
    ultra::gui::startupTrace("App", "apply-expert-exit");

    // Apply initial filter settings from loaded config
    ultra::gui::startupTrace("App", "apply-filter-enter");
    FilterConfig initial_filter;
    initial_filter.enabled = settings_.filter_enabled;
    initial_filter.center_freq = settings_.filter_center;
    initial_filter.bandwidth = settings_.filter_bandwidth;
    initial_filter.taps = settings_.filter_taps;
    modem_.setFilterConfig(initial_filter);
    audio_.setOutputGain(settings_.tx_drive);
    ultra::gui::startupTrace("App", "apply-filter-exit");

    if (simulation_enabled_) {
        ultra::gui::startupTrace("App", "init-ota-audio-enter");
        initOtaAudio();
        ultra::gui::startupTrace("App", "init-ota-audio-exit");
    }

    // Auto-initialize audio on startup unless safe-startup mode is requested.
    // This avoids crashing on fragile audio stacks during process bring-up.
    if (!options_.safe_startup && !simulation_enabled_) {
        ultra::gui::startupTrace("App", "init-audio-enter");
        initAudio();
        if (audio_initialized_) {
            deferred_radio_rx_start_pending_ = true;
            uint32_t now_ms = SDL_GetTicks();
            deferred_radio_rx_start_deadline_ms_ = now_ms;
            deferred_radio_rx_start_timeout_ms_ = now_ms + 3000;
            deferred_radio_rx_start_attempts_ = 0;
            guiLog("Startup audio stage 1/2 complete: core ready, starting RX capture ASAP (timeout=3000ms)");
        }
        ultra::gui::startupTrace("App", "init-audio-exit");
    } else if (!simulation_enabled_) {
        guiLog("Safe startup enabled: deferred audio/simulator initialization");
        deferred_audio_auto_init_pending_ = true;
        deferred_audio_auto_init_deadline_ms_ = SDL_GetTicks() + 300;
        deferred_audio_auto_init_attempts_ = 0;
        deferred_audio_wait_logged_ = false;
        ultra::gui::startupTrace("App", "deferred-audio-scheduled");
    } else {
        guiLog("OTASim mode enabled: SDL audio device initialization skipped");
    }
    ultra::gui::startupTrace("App", "ctor-body-exit");
}

App::~App() {
    releasePtt("app_shutdown");
    closePtt();

    stopOtaAudio();

    settings_.save();
    audio_.shutdown();

    // Write recording to file if -rec was enabled
    if (options_.record_audio) {
        writeRecordingToFile();
    }
    ultra::diagnostics::DiagnosticsRecorder::instance().stop();
}

void App::writeRecordingToFile() {
    const std::string base = trimF32Extension(options_.record_path);
    bool wrote_any = false;

    if (!recorded_tx_samples_.empty()) {
        const std::string path = base + "_tx.f32";
        if (writeF32File(path, recorded_tx_samples_)) {
            guiLog("Recording saved: %s (%zu samples, %.1f seconds)",
                   path.c_str(),
                   recorded_tx_samples_.size(),
                   recorded_tx_samples_.size() / 48000.0f);
            wrote_any = true;
        } else {
            guiLog("ERROR: Failed to save TX recording to %s", path.c_str());
        }
    }

    if (!recorded_rx_samples_.empty()) {
        const std::string path = base + "_rx.f32";
        if (writeF32File(path, recorded_rx_samples_)) {
            guiLog("Recording saved: %s (%zu samples, %.1f seconds)",
                   path.c_str(),
                   recorded_rx_samples_.size(),
                   recorded_rx_samples_.size() / 48000.0f);
            wrote_any = true;
        } else {
            guiLog("ERROR: Failed to save RX recording to %s", path.c_str());
        }
    }

    if (!wrote_any) {
        guiLog("Recording skipped: no captured samples");
    }
}

void App::initOtaAudio() {
    if (ota_audio_) {
        return;
    }

    OtaAudioBackendConfig config;
    config.grpc_target = options_.ota_host;
    config.udp_target = options_.ota_udp_host;
    config.token = options_.token;
    config.station_id = options_.station_id;
    config.session_id = options_.session_id.empty() ? "lobby" : options_.session_id;

    ota_audio_ = std::make_unique<OtaAudioBackend>();
    std::string error;
    if (!ota_audio_->start(std::move(config), &error)) {
        guiLog("OTASim: start failed: %s", error.c_str());
        appendRxLogLine("[OTASIM] Disconnected: " + error);
        ota_audio_.reset();
        return;
    }

    appendRxLogLine("[OTASIM] Connecting to " + options_.ota_host +
                    " as " + options_.station_id +
                    " in session " + (options_.session_id.empty() ? "lobby" : options_.session_id));

    if (options_.monitor_audio && ota_monitor_device_id_ == 0) {
        SDL_AudioSpec want{};
        SDL_AudioSpec have{};
        want.freq = 48000;
        want.format = AUDIO_F32SYS;
        want.channels = 1;
        want.samples = 2048;
        want.callback = nullptr;
        const char* device_name = options_.monitor_device.empty()
            ? nullptr
            : options_.monitor_device.c_str();
        SDL_AudioDeviceID dev = SDL_OpenAudioDevice(device_name, 0, &want, &have, 0);
        if (dev == 0) {
            guiLog("OTASim monitor: SDL_OpenAudioDevice failed: %s", SDL_GetError());
            appendRxLogLine(std::string("[OTASIM] Monitor unavailable: ") + SDL_GetError());
        } else {
            ota_monitor_device_id_ = dev;
            SDL_PauseAudioDevice(dev, 0);
            guiLog("OTASim monitor: playing RX through SDL device '%s'",
                   device_name ? device_name : "(default)");
            appendRxLogLine(std::string("[OTASIM] Monitor: ") +
                            (device_name ? device_name : "(default output)"));
        }
    }
}

void App::stopOtaAudio() {
    if (ota_monitor_device_id_ != 0) {
        SDL_CloseAudioDevice(ota_monitor_device_id_);
        ota_monitor_device_id_ = 0;
    }
    if (ota_audio_) {
        ota_audio_->close();
        ota_audio_.reset();
    }
}

void App::pollOtaRx() {
    if (!ota_audio_) {
        return;
    }

    constexpr size_t kChunkSamples = 2048;
    constexpr int kMaxChunksPerFrame = 8;
    for (int i = 0; i < kMaxChunksPerFrame; ++i) {
        auto samples = ota_audio_->getRxSamples(kChunkSamples);
        if (samples.empty()) {
            break;
        }
        if (recording_enabled_) {
            recorded_rx_samples_.insert(recorded_rx_samples_.end(), samples.begin(), samples.end());
        }
        modem_.feedAudio(samples);
        if (modem_.isSynchronousMode()) {
            modem_.processRxBuffer();
        }
        if (waterfall_) {
            waterfall_->addSamples(samples.data(), samples.size());
        }
        if (ota_monitor_device_id_ != 0) {
            // ~500ms cap at 48kHz mono float = 24000 samples = 96000 bytes
            constexpr Uint32 kMaxQueueBytes = 96000;
            if (SDL_GetQueuedAudioSize(ota_monitor_device_id_) > kMaxQueueBytes) {
                SDL_ClearQueuedAudio(ota_monitor_device_id_);
            }
            SDL_QueueAudio(ota_monitor_device_id_,
                           samples.data(),
                           static_cast<Uint32>(samples.size() * sizeof(float)));
        }
    }
}

void App::initAudio() {
    if (audio_initialized_) return;

    ultra::gui::startupTrace("App", "initAudio-enter");
    if (!audio_.initialize()) {
        ultra::gui::startupTrace("App", "initAudio-audio-initialize-fail");
        return;
    }
    ultra::gui::startupTrace("App", "initAudio-audio-initialize-ok");

    // Enumerate devices
    ultra::gui::startupTrace("App", "initAudio-enum-input-enter");
    input_devices_ = audio_.getInputDevices();
    ultra::gui::startupTrace("App", "initAudio-enum-input-exit");
    ultra::gui::startupTrace("App", "initAudio-enum-output-enter");
    output_devices_ = audio_.getOutputDevices();
    ultra::gui::startupTrace("App", "initAudio-enum-output-exit");

    // Populate settings window device lists
    settings_window_.input_devices = input_devices_;
    settings_window_.output_devices = output_devices_;
    guiLog("initAudio: enumerated %zu input device(s), %zu output device(s)",
           input_devices_.size(), output_devices_.size());

    audio_initialized_ = true;
    ultra::gui::startupTrace("App", "initAudio-exit");
}

void App::appendRxLogLine(const std::string& msg) {
    std::lock_guard<std::mutex> lock(rx_log_mutex_);
    rx_log_.push_back(msg);
    while (rx_log_.size() > MAX_RX_LOG) {
        rx_log_.pop_front();
    }
}

bool App::startFileSend(const std::string& file_path, const std::string& success_log) {
    last_progress_milestone_ = 0;
    file_transfer_start_time_ = std::chrono::steady_clock::now();
    const uint32_t file_bytes = safeFileSizeBytes(file_path);
    if (protocol_.sendFile(file_path)) {
        pending_file_tx_payload_bytes_ = file_bytes;
        pending_file_tx_path_ = file_path;
        appendRxLogLine(success_log);
        return true;
    }

    pending_file_tx_payload_bytes_ = 0;
    pending_file_tx_path_.clear();
    appendRxLogLine("[FILE] Failed to start transfer");
    return false;
}

void App::startFileSendOrImageDialog(const std::string& file_path) {
    ImageFormat fmt = ImageFormat::Unknown;
    if (!sniffImageFormat(file_path, fmt)) {
        startFileSend(file_path, "[FILE] Sending: " + file_path);
        return;
    }

    ImageInfo info;
    if (!readImageInfo(file_path, info)) {
        appendRxLogLine("[IMAGE] Failed to read image info; sending original");
        startFileSend(file_path, "[FILE] Sending: " + file_path);
        return;
    }

    image_send_path_ = file_path;
    image_send_info_ = info;
    image_send_preset_ = 0;
    image_send_error_.clear();
    ImGui::OpenPopup("Send Image##image_send_modal");
}

void App::renderImageSendModal() {
    if (!ImGui::BeginPopupModal("Send Image##image_send_modal", nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize)) {
        return;
    }

    struct Preset {
        const char* label;
        const char* details;
        int max_w;
        int max_h;
        int quality;
        uint64_t estimated_bytes;
        bool full_size;
    };

    const Preset presets[] = {
        {"Thumbnail", "320x240, JPEG q=70", 320, 240, 70, 10ull * 1024ull, false},
        {"Preview", "640x480, JPEG q=75", 640, 480, 75, 40ull * 1024ull, false},
        {"Full size", "as-is", 0, 0, 0, image_send_info_.file_size_bytes, true},
    };

    int bps = protocol_.getCurrentBitrate_bps();
    const bool fallback_bps = bps <= 0;
    if (fallback_bps) {
        bps = 1400;
    }

    ImGui::TextUnformatted("Send image");
    ImGui::Separator();
    ImGui::Text("Source: %dx%d %s, %s",
                image_send_info_.width,
                image_send_info_.height,
                imageFormatName(image_send_info_.format),
                formatByteCount(image_send_info_.file_size_bytes).c_str());
    ImGui::Spacing();

    for (int i = 0; i < 3; ++i) {
        const Preset& preset = presets[i];
        ImGui::PushID(i);
        ImGui::RadioButton(preset.label, &image_send_preset_, i);
        ImGui::SameLine();

        const std::string size_label =
            std::string(preset.full_size ? "" : "~") + formatByteCount(preset.estimated_bytes);
        std::string time_label = "~" + formatWireTime(preset.estimated_bytes, bps);
        if (fallback_bps) {
            time_label += " (R1/2 Good est)";
        }
        ImGui::TextDisabled("%s - %s %s",
                            preset.details,
                            size_label.c_str(),
                            time_label.c_str());
        ImGui::PopID();
    }

    if (!image_send_error_.empty()) {
        ImGui::Spacing();
        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s",
                           image_send_error_.c_str());
    }

    ImGui::Separator();
    const bool can_confirm = protocol_.isConnected() && protocol_.isReadyToSend();
    ImGui::BeginDisabled(!can_confirm);
    if (ImGui::Button("Confirm", ImVec2(90, 0))) {
        const Preset& preset = presets[image_send_preset_];
        if (preset.full_size) {
            startFileSend(image_send_path_, "[FILE] Sending: " + image_send_path_);
            image_send_path_.clear();
            image_send_info_ = ImageInfo{};
            image_send_error_.clear();
            ImGui::CloseCurrentPopup();
        } else {
            std::string call(settings_.callsign, boundedCStringLen(settings_.callsign));
            for (char& ch : call) {
                const unsigned char c = static_cast<unsigned char>(ch);
                ch = std::isalnum(c) ? static_cast<char>(std::toupper(c)) : '_';
            }
            if (call.empty()) {
                call = "NOCALL";
            }

            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            std::error_code ec;
            const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path(ec);
            if (ec) {
                image_send_error_ = "failed to locate temp directory";
            } else {
                const std::filesystem::path dst =
                    tmp_dir / ("ultra_img_send_" + call + "_" +
                               std::to_string(now_ms) + ".jpg");
                std::string error;
                if (!resizeAndEncodeJPEG(image_send_path_, preset.max_w, preset.max_h,
                                         preset.quality, dst.string(), error)) {
                    image_send_error_ = error.empty() ? "failed to resize image" : error;
                } else {
                    ImageInfo resized_info;
                    const bool have_resized_info = readImageInfo(dst.string(), resized_info);
                    const uint64_t resized_bytes = have_resized_info
                        ? resized_info.file_size_bytes
                        : static_cast<uint64_t>(safeFileSizeBytes(dst.string()));
                    const int log_w = have_resized_info ? resized_info.width : preset.max_w;
                    const int log_h = have_resized_info ? resized_info.height : preset.max_h;
                    const std::string log =
                        "[IMAGE] Resized: " + std::to_string(log_w) + "x" +
                        std::to_string(log_h) + " " + formatByteCount(resized_bytes) +
                        " -> sending " + dst.string();
                    startFileSend(dst.string(), log);
                    image_send_path_.clear();
                    image_send_info_ = ImageInfo{};
                    image_send_error_.clear();
                    ImGui::CloseCurrentPopup();
                }
            }
        }
    }
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(90, 0))) {
        image_send_path_.clear();
        image_send_info_ = ImageInfo{};
        image_send_error_.clear();
        ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
}

std::deque<std::string> App::snapshotRxLog() const {
    std::lock_guard<std::mutex> lock(rx_log_mutex_);
    return rx_log_;
}

void App::clearRxLog() {
    std::lock_guard<std::mutex> lock(rx_log_mutex_);
    rx_log_.clear();
}

void App::renderDiagnosticsDialogs() {
    auto& recorder = ultra::diagnostics::DiagnosticsRecorder::instance();
    if (!recorder.hasAudioConsent() && !diagnostics_consent_popup_opened_) {
        ImGui::OpenPopup("ProjectUltra diagnostics audio consent");
        diagnostics_consent_popup_opened_ = true;
    }

    if (ImGui::BeginPopupModal("ProjectUltra diagnostics audio consent", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextWrapped(
            "ProjectUltra can keep the last two minutes of RX audio as lossless PCM "
            "for local field reports. RX audio may contain callsigns or third-party "
            "speech. No upload is performed by the app.");
        ImGui::Spacing();
        if (ImGui::Button("Enable RX Recording", ImVec2(180, 0))) {
            if (recorder.grantAudioConsent()) {
                appendRxLogLine("[DIAG] RX diagnostics recording enabled");
            } else {
                appendRxLogLine("[DIAG] Failed to save audio consent");
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Not Now", ImVec2(110, 0))) {
            appendRxLogLine("[DIAG] RX diagnostics recording disabled until consent is granted");
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (diagnostics_report_popup_open_) {
        ImGui::OpenPopup("Create diagnostics report");
        diagnostics_report_popup_open_ = false;
    }
    if (ImGui::BeginPopupModal("Create diagnostics report", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextWrapped(
            "Create a local report bundle with recent events, build provenance, "
            "redacted config, and consented RX audio.");
        ImGui::InputTextMultiline("Note", diagnostics_note_buffer_,
                                  sizeof(diagnostics_note_buffer_), ImVec2(420, 110));
        ImGui::Spacing();
        if (ImGui::Button("Create", ImVec2(120, 0))) {
            ultra::diagnostics::ReportOptions options;
            options.note = diagnostics_note_buffer_;
            auto report = recorder.freeze(ultra::diagnostics::FreezeReason::Manual, options);
            if (report.ok) {
                diagnostics_last_report_ = report.path.string();
                appendRxLogLine("[DIAG] Report created: " + diagnostics_last_report_);
            } else {
                appendRxLogLine("[DIAG] Report failed: " + report.error);
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            ImGui::CloseCurrentPopup();
        }
        if (!diagnostics_last_report_.empty()) {
            ImGui::Separator();
            ImGui::TextWrapped("%s", diagnostics_last_report_.c_str());
        }
        ImGui::EndPopup();
    }

    if (diagnostics_debrief_popup_open_) {
        ImGui::OpenPopup("Session debrief");
        diagnostics_debrief_popup_open_ = false;
    }
    if (ImGui::BeginPopupModal("Session debrief", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextWrapped("%s", diagnostics_last_summary_path_.c_str());
        ImGui::Separator();
        ImGui::BeginChild("DebriefText", ImVec2(620, 360), true);
        ImGui::TextWrapped("%s", diagnostics_last_summary_.c_str());
        ImGui::EndChild();
        ImGui::Spacing();
        ImGui::SetNextItemWidth(500);
        ImGui::InputText("Save path", diagnostics_debrief_save_path_,
                         sizeof(diagnostics_debrief_save_path_));
        if (ImGui::Button("Save debrief", ImVec2(140, 0))) {
            std::error_code ec;
            std::filesystem::copy_file(diagnostics_last_summary_path_,
                                       diagnostics_debrief_save_path_,
                                       std::filesystem::copy_options::overwrite_existing,
                                       ec);
            diagnostics_debrief_status_ = ec ? ("Save failed: " + ec.message())
                                             : "Debrief saved";
        }
        ImGui::SameLine();
        if (ImGui::Button("Create full report", ImVec2(160, 0))) {
            ultra::diagnostics::ReportOptions options;
            auto report = recorder.freeze(ultra::diagnostics::FreezeReason::Manual, options);
            if (report.ok) {
                diagnostics_last_report_ = report.path.string();
                diagnostics_debrief_status_ = "Report created: " + diagnostics_last_report_;
                appendRxLogLine("[DIAG] Report created: " + diagnostics_last_report_);
            } else {
                diagnostics_debrief_status_ = "Report failed: " + report.error;
                appendRxLogLine("[DIAG] Report failed: " + report.error);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(100, 0))) {
            ImGui::CloseCurrentPopup();
        }
        if (!diagnostics_debrief_status_.empty()) {
            ImGui::Separator();
            ImGui::TextWrapped("%s", diagnostics_debrief_status_.c_str());
        }
        ImGui::EndPopup();
    }
}

void App::sendMessage() {
    // Not used in current implementation - messages sent via protocol
}

void App::onDataReceived(const std::string& text) {
    if (!text.empty()) {
        appendRxLogLine("[RX] " + text);
    }
}

void App::resetAdaptiveAdvisory() {
    std::lock_guard<std::mutex> lock(adapt_mutex_);
    adapt_snr_window_.clear();
    adapt_fading_window_.clear();
    adapt_candidate_valid_ = false;
    adapt_candidate_hits_ = 0;
    adapt_virtual_mode_valid_ = false;
    adapt_upgrade_hold_logged_ = false;
}

void App::updateAdaptiveAdvisory(float snr_db, float fading_index) {
    if (protocol_.getState() != protocol::ConnectionState::CONNECTED) {
        return;
    }
    if (!std::isfinite(snr_db) || !std::isfinite(fading_index)) {
        return;
    }
    std::lock_guard<std::mutex> lock(adapt_mutex_);

    adapt_snr_window_.push_back(snr_db);
    adapt_fading_window_.push_back(fading_index);
    if (adapt_snr_window_.size() > ADAPT_WINDOW_FRAMES) {
        adapt_snr_window_.pop_front();
    }
    if (adapt_fading_window_.size() > ADAPT_WINDOW_FRAMES) {
        adapt_fading_window_.pop_front();
    }
    if (adapt_snr_window_.size() < ADAPT_WINDOW_FRAMES ||
        adapt_fading_window_.size() < ADAPT_WINDOW_FRAMES) {
        return;
    }

    float avg_snr = 0.0f;
    for (float v : adapt_snr_window_) avg_snr += v;
    avg_snr /= static_cast<float>(adapt_snr_window_.size());

    float avg_fading = 0.0f;
    for (float v : adapt_fading_window_) avg_fading += v;
    avg_fading /= static_cast<float>(adapt_fading_window_.size());

    auto waveform = modem_.getWaveformMode();
    Modulation current_mod = protocol_.getDataModulation();
    CodeRate current_rate = protocol_.getDataCodeRate();

    if (!adapt_virtual_mode_valid_) {
        adapt_virtual_mode_valid_ = true;
        adapt_virtual_mod_ = current_mod;
        adapt_virtual_rate_ = current_rate;
        adapt_last_virtual_switch_ = std::chrono::steady_clock::now();
    }

    Modulation rec_mod = current_mod;
    CodeRate rec_rate = current_rate;
    protocol::recommendDataMode(avg_snr, waveform, rec_mod, rec_rate, avg_fading);

    Modulation eval_mod = adapt_virtual_mod_;
    CodeRate eval_rate = adapt_virtual_rate_;

    if (rec_mod == eval_mod && rec_rate == eval_rate) {
        adapt_candidate_valid_ = false;
        adapt_candidate_hits_ = 0;
        adapt_upgrade_hold_logged_ = false;
        return;
    }

    if (adapt_candidate_valid_ &&
        adapt_candidate_mod_ == rec_mod &&
        adapt_candidate_rate_ == rec_rate) {
        ++adapt_candidate_hits_;
    } else {
        adapt_candidate_valid_ = true;
        adapt_candidate_mod_ = rec_mod;
        adapt_candidate_rate_ = rec_rate;
        adapt_candidate_hits_ = 1;
    }

    bool is_upgrade = modeEfficiency(rec_mod, rec_rate) > modeEfficiency(eval_mod, eval_rate) + 0.05f;
    int required_windows = is_upgrade ? ADAPT_UPGRADE_WINDOWS : ADAPT_DOWNGRADE_WINDOWS;
    if (adapt_candidate_hits_ < required_windows) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    if (is_upgrade) {
        auto elapsed_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(now - adapt_last_virtual_switch_).count());
        int hold_remaining_ms = ADAPT_UPGRADE_HOLD_MS - elapsed_ms;
        if (hold_remaining_ms > 0) {
            if (!adapt_upgrade_hold_logged_ ||
                adapt_upgrade_hold_mod_ != rec_mod ||
                adapt_upgrade_hold_rate_ != rec_rate) {
                char hold_msg[260];
                snprintf(hold_msg, sizeof(hold_msg),
                         "[ADPT] Local improving conditions (SNR=%.1f dB, F.I.=%.2f): "
                         "hysteresis hold %.1fs before upgrade to %s %s",
                         avg_snr, avg_fading,
                         hold_remaining_ms / 1000.0f,
                         modulationToString(rec_mod), codeRateToString(rec_rate));
                guiLog("%s", hold_msg);
                appendRxLogLine(hold_msg);
                adapt_upgrade_hold_logged_ = true;
                adapt_upgrade_hold_mod_ = rec_mod;
                adapt_upgrade_hold_rate_ = rec_rate;
            }
            return;
        }
    }

    adapt_upgrade_hold_logged_ = false;
    const char* direction = adaptationDirection(eval_mod, eval_rate, rec_mod, rec_rate);
    char msg[240];
    snprintf(msg, sizeof(msg),
             "[ADPT] Local %s conditions (SNR=%.1f dB, F.I.=%.2f): "
             "hysteresis allows switch %s %s -> %s %s",
             direction, avg_snr, avg_fading,
             modulationToString(eval_mod), codeRateToString(eval_rate),
             modulationToString(rec_mod), codeRateToString(rec_rate));

    guiLog("%s", msg);
    appendRxLogLine(msg);

    adapt_virtual_mod_ = rec_mod;
    adapt_virtual_rate_ = rec_rate;
    adapt_last_virtual_switch_ = now;
    adapt_candidate_valid_ = false;
    adapt_candidate_hits_ = 0;
}

void App::render() {
    static bool first_render = true;
    render_frames_seen_++;
    if (first_render) {
        ultra::gui::startupTrace("App", "render-enter");
    }
    // Keep output attenuation synchronized with the TX Drive slider.
    if (first_render) {
        ultra::gui::startupTrace("App", "render-set-output-gain-enter");
    }
    audio_.setOutputGain(settings_.tx_drive);
    if (first_render) {
        ultra::gui::startupTrace("App", "render-set-output-gain-exit");
    }

    // Process captured RX audio in the main thread.
    pollRadioRx();

    // Safe-startup mode: auto-start audio shortly after first frames.
    // Keeps process bring-up lightweight while preserving "auto-listen" behavior.
    if (deferred_audio_auto_init_pending_ &&
        !simulation_enabled_ &&
        !audio_initialized_) {
        if (render_frames_seen_ < 6) {
            if (!deferred_audio_wait_logged_) {
                guiLog("Deferred audio guard: waiting for UI warm-up frames (%u/6)",
                       render_frames_seen_);
                deferred_audio_wait_logged_ = true;
            }
        } else {
            deferred_audio_wait_logged_ = false;
            uint32_t now_ms = SDL_GetTicks();
            if (now_ms >= deferred_audio_auto_init_deadline_ms_) {
                ultra::gui::startupTrace("App", "deferred-audio-init-enter");
                initAudio();
                if (audio_initialized_) {
                    ultra::gui::startupTrace("App", "deferred-audio-open-output-enter");
                    deferred_radio_rx_start_pending_ = true;
                    deferred_radio_rx_start_deadline_ms_ = now_ms;
                    deferred_radio_rx_start_timeout_ms_ = now_ms + 3000;
                    deferred_radio_rx_start_attempts_ = 0;
                    guiLog("Deferred audio stage 1/2 complete: core ready, starting RX capture ASAP (timeout=3000ms)");
                    ultra::gui::startupTrace("App", "deferred-audio-open-output-exit");
                    deferred_audio_auto_init_pending_ = false;
                } else {
                    deferred_audio_auto_init_attempts_++;
                    ultra::gui::startupTrace("App", "deferred-audio-init-fail");
                    if (deferred_audio_auto_init_attempts_ >= 3) {
                        guiLog("Deferred audio auto-init failed after %d attempts, manual init required",
                               deferred_audio_auto_init_attempts_);
                        deferred_audio_auto_init_pending_ = false;
                    } else {
                        deferred_audio_auto_init_deadline_ms_ = now_ms + 700;
                        guiLog("Deferred audio auto-init retry %d/3 scheduled",
                               deferred_audio_auto_init_attempts_ + 1);
                    }
                }
            }
        }
    }

    // Stage 2: start RX capture only after playback is confirmed and warm.
    if (deferred_radio_rx_start_pending_ &&
        !simulation_enabled_ &&
        audio_initialized_) {
        uint32_t now_ms = SDL_GetTicks();
        if (now_ms >= deferred_radio_rx_start_deadline_ms_) {
            if (startRadioRx()) {
                guiLog("Deferred audio stage 2/2 complete: RX capture started");
                deferred_radio_rx_start_pending_ = false;
                deferred_radio_rx_start_attempts_ = 0;
            } else {
                deferred_radio_rx_start_attempts_++;
                if (now_ms >= deferred_radio_rx_start_timeout_ms_) {
                    guiLog("Deferred audio stage 2/2 timeout after %d attempts (%ums); manual audio init required",
                           deferred_radio_rx_start_attempts_, 3000u);
                    deferred_radio_rx_start_pending_ = false;
                } else {
                    deferred_radio_rx_start_deadline_ms_ = now_ms + 100;
                    if (deferred_radio_rx_start_attempts_ == 1 ||
                        (deferred_radio_rx_start_attempts_ % 10) == 0) {
                        uint32_t remaining_ms = deferred_radio_rx_start_timeout_ms_ - now_ms;
                        guiLog("Deferred audio stage 2/2 waiting for RX readiness (attempt=%d, remaining=%ums)",
                               deferred_radio_rx_start_attempts_, remaining_ms);
                    }
                }
            }
        }
    }

    // === DEBUG: Test signal keys (F1-F7) ===
    if (ImGui::IsKeyPressed(ImGuiKey_F1)) {
        auto tone = modem_.generateTestTone(1.0f);
        if (!queueRealTxSamples(tone, "TEST tone")) {
            appendRxLogLine("[TEST] Failed to queue tone TX");
        } else {
            appendRxLogLine("[TEST] Sent 1500 Hz tone");
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F2)) {
        auto samples = modem_.transmitTestPattern(0);
        if (!queueRealTxSamples(samples, "TEST pattern0")) {
            appendRxLogLine("[TEST] Failed to queue pattern TX");
        } else {
            appendRxLogLine("[TEST] Sent pattern: ALL ZEROS (LDPC encoded)");
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F3)) {
        auto samples = modem_.transmitTestPattern(1);
        if (!queueRealTxSamples(samples, "TEST pattern1")) {
            appendRxLogLine("[TEST] Failed to queue pattern TX");
        } else {
            appendRxLogLine("[TEST] Sent pattern: DEADBEEF (LDPC encoded)");
        }
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F7)) {
        const char* test_file = "tests/data/test_connect_data_sequence.f32";
        size_t injected = modem_.injectSignalFromFile(test_file);
        if (injected > 0) {
            appendRxLogLine("[TEST] Injected " + std::to_string(injected) + " samples");
        } else {
            appendRxLogLine("[TEST] Failed to inject signal");
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
    if (first_render) {
        ultra::gui::startupTrace("App", "render-main-window-enter");
    }
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
    bool defer_monitoring = options_.safe_startup && !audio_initialized_ && !simulation_enabled_;

    ImGui::BeginChild("ContentArea", ImVec2(0, content_height), false);

    float total_width = ImGui::GetContentRegionAvail().x;
    float left_width = total_width * 0.32f;  // Monitoring column

    // ========================================
    // LEFT COLUMN: Monitoring (Constellation + Channel Status + Waterfall)
    // ========================================
    ImGui::BeginChild("LeftPanel", ImVec2(left_width, 0), true);

    if (defer_monitoring) {
        ImGui::TextDisabled("Startup safety mode active");
        ImGui::TextDisabled("Monitoring widgets enabled after audio init");
    } else {
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
        auto conn_stats = protocol_.getStats();
        renderCompactChannelStatus(modem_stats, data_mod, data_rate, conn_stats);

        ImGui::Separator();

        // Waterfall (uses remaining space)
        if (waterfall_) {
            updateWaterfallFrequencyDisplay();
            waterfall_->render();
        } else {
            ImGui::TextDisabled("Waterfall disabled");
        }
    }

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
    auto mstats = defer_monitoring ? LoopbackStats{} : modem_.getStats();
    auto dstats = defer_monitoring ? DecoderStats{} : modem_.getDecoderStats();
    const char* mode_str = simulation_enabled_ ? "OTASIM" : (ptt_active_ ? "TX" : (radio_rx_enabled_ ? "RX" : "IDLE"));
    char goodput_text[96];
    if (last_effective_goodput_bps_ > 0.0f) {
        snprintf(goodput_text, sizeof(goodput_text), "%.2f kbps (%s)",
                 last_effective_goodput_bps_ / 1000.0f, last_goodput_label_.c_str());
    } else {
        snprintf(goodput_text, sizeof(goodput_text), "n/a");
    }
    ImGui::Text("Mode: %s | SNR: %.1f dB | TX: %d | RX: %d | PHY: %d bps | Goodput: %s | RXQ: %.0f ms (pk %.0f) | OF: %llu/%llu | %s%s",
                mode_str, mstats.snr_db, mstats.frames_sent, mstats.frames_received,
                mstats.throughput_bps, goodput_text,
                dstats.backlog_ms, dstats.peak_backlog_ms,
                static_cast<unsigned long long>(dstats.buffer_overflows),
                static_cast<unsigned long long>(dstats.overflow_samples_dropped),
                ultra::kBuildGitCommitShort,
                ultra::kBuildDirty ? "-dirty" : "");
    if (simulation_enabled_) {
        ImGui::SameLine();
        const std::string ota_status = ota_audio_ ? ota_audio_->status().text : "Disconnected";
        ImGui::TextDisabled("| OTASim: %s", ota_status.c_str());
    }
    if (ultra::diagnostics::DiagnosticsRecorder::instance().isRxAudioRecordingEnabled()) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.25f, 0.18f, 1.0f), "Recording");
    }

    ImGui::End();
    if (first_render) {
        ultra::gui::startupTrace("App", "render-main-window-exit");
    }

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
    renderDiagnosticsDialogs();
    if (first_render) {
        ultra::gui::startupTrace("App", "render-exit");
        first_render = false;
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

ptt::PttConfig App::pttConfigFromSettings(const AppSettings& settings) const {
    ptt::PttConfig config;
    const GuiPttMode mode = static_cast<GuiPttMode>(settings.ptt_mode);
    if (mode == GuiPttMode::SerialRTS || mode == GuiPttMode::SerialDTR) {
        config.mode = ptt::PttMode::Serial;
        const size_t port_len = boundedCStringLen(settings.ptt_serial_port);
        config.serial_port.assign(settings.ptt_serial_port, port_len);
        config.serial_baud = (settings.ptt_serial_baud > 0) ? settings.ptt_serial_baud : 9600;
        config.serial_line = (mode == GuiPttMode::SerialDTR)
                                 ? ptt::SerialLine::DTR
                                 : ptt::SerialLine::RTS;
        config.serial_inactive_high = settings.ptt_invert;
    } else if (mode == GuiPttMode::Cat) {
        config.mode = ptt::PttMode::Cat;
        const size_t host_len = boundedCStringLen(settings.ptt_cat_host);
        config.cat_host.assign(settings.ptt_cat_host, host_len);
        if (config.cat_host.empty()) {
            config.cat_host = "127.0.0.1";
        }
        int port = settings.ptt_cat_port;
        if (port < 1 || port > 65535) {
            port = 4532;
        }
        config.cat_port = static_cast<uint16_t>(port);
    } else if (mode == GuiPttMode::HamlibBuiltin) {
        config.mode = ptt::PttMode::HamlibBuiltin;
        config.hamlib_model_id = settings.ptt_hamlib_model_id > 0
                                      ? settings.ptt_hamlib_model_id
                                      : 1;
        const size_t port_len = boundedCStringLen(settings.ptt_hamlib_port);
        config.hamlib_rig_port.assign(settings.ptt_hamlib_port, port_len);
        config.hamlib_baud = settings.ptt_hamlib_baud > 0 ? settings.ptt_hamlib_baud : 9600;
        config.hamlib_ptt_method = hamlibPttMethodFromSettings(settings.ptt_hamlib_method);
    }
    return config;
}

bool App::ensurePttReadyLocked(const AppSettings& settings) {
    if (simulation_enabled_) {
        if (ptt_driver_) {
            ptt_driver_->close();
            ptt_driver_.reset();
        }
        ptt_config_ = ptt::PttConfig{};
        return true;
    }
    const ptt::PttConfig config = pttConfigFromSettings(settings);
    if (config.mode == ptt::PttMode::None) {
        if (ptt_driver_) {
            ptt_driver_->close();
            ptt_driver_.reset();
        }
        ptt_config_ = config;
        return true;
    }

    if (!ptt_driver_ || config != ptt_config_) {
        if (ptt_driver_) {
            ptt_driver_->close();
        }
        ptt_driver_ = ptt::createPttDriver(config);
        ptt_config_ = config;
    }

    if (ptt_driver_->isOpen()) {
        return true;
    }

    if (!ptt_driver_->open()) {
        const std::string error = ptt_driver_->lastError();
        guiLog("PTT: open failed: %s", error.c_str());
        appendRxLogLine("[PTT] Failed to open: " + error);
        return false;
    }

    if (config.mode == ptt::PttMode::Serial) {
        guiLog("PTT: serial ready on %s @ %d line=%s",
               config.serial_port.c_str(),
               config.serial_baud,
               ptt::serialLineName(config.serial_line));
    } else if (config.mode == ptt::PttMode::Cat) {
        guiLog("PTT: CAT ready via rigctld %s:%u",
               config.cat_host.c_str(),
               static_cast<unsigned>(config.cat_port));
    } else if (config.mode == ptt::PttMode::HamlibBuiltin) {
        guiLog("PTT: Hamlib built-in ready model=%d port=%s baud=%d ptt=%s",
               config.hamlib_model_id,
               config.hamlib_rig_port.c_str(),
               config.hamlib_baud,
               ptt::hamlibPttMethodName(config.hamlib_ptt_method));
    }
    return true;
}

bool App::ensurePttReady() {
    std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
    return ensurePttReadyLocked(settings_);
}

void App::updateWaterfallFrequencyDisplay() {
    if (!waterfall_) {
        return;
    }

    if (simulation_enabled_) {
        std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
        if (ptt_driver_) {
            ptt_driver_->close();
            ptt_driver_.reset();
            ptt_config_ = ptt::PttConfig{};
            cat_frequency_next_open_attempt_ms_ = 0;
        }
        waterfall_->setRadioFrequency(std::nullopt, false);
        return;
    }

    const ptt::PttConfig config = pttConfigFromSettings(settings_);
    if (config.mode != ptt::PttMode::Cat &&
        config.mode != ptt::PttMode::HamlibBuiltin) {
        std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
        if ((ptt_config_.mode == ptt::PttMode::Cat ||
             ptt_config_.mode == ptt::PttMode::HamlibBuiltin) &&
            ptt_driver_) {
            ptt_driver_->close();
            ptt_driver_.reset();
            ptt_config_ = config;
            cat_frequency_next_open_attempt_ms_ = 0;
        }
        waterfall_->setRadioFrequency(std::nullopt, false);
        return;
    }

    ptt::IPttDriver::RadioFrequencyState frequency;
    {
        std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
        if (!ptt_driver_ || config != ptt_config_) {
            if (ptt_driver_) {
                ptt_driver_->close();
            }
            ptt_driver_ = ptt::createPttDriver(config);
            ptt_config_ = config;
            cat_frequency_next_open_attempt_ms_ = 0;
        }

        if (ptt_driver_ && !ptt_driver_->isOpen()) {
            const uint32_t now_ms = SDL_GetTicks();
            if (now_ms >= cat_frequency_next_open_attempt_ms_) {
                if (ptt_driver_->startTelemetry()) {
                    cat_frequency_next_open_attempt_ms_ = 0;
                } else {
                    cat_frequency_next_open_attempt_ms_ = now_ms + 5000;
                }
            }
        }

        if (ptt_driver_) {
            frequency = ptt_driver_->radioFrequencyState();
        }
    }

    waterfall_->setRadioFrequency(frequency.hz, frequency.stale);
}

bool App::setPtt(bool asserted, const char* reason) {
    std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
    if (!ensurePttReadyLocked(settings_)) {
        ptt_active_ = false;
        return false;
    }

    if (!ptt_driver_) {
        ptt_active_ = false;
        return true;
    }

    if (!ptt_driver_->setKey(asserted ? ptt::PttKey::On : ptt::PttKey::Off)) {
        const std::string error = ptt_driver_->lastError();
        guiLog("PTT: %s failed (%s): %s",
               asserted ? "ASSERT" : "RELEASE",
               reason ? reason : "n/a",
               error.c_str());
        if (asserted) {
            appendRxLogLine("[PTT] Failed to key: " + error);
        }
        ptt_active_ = false;
        return false;
    }

    ptt_active_ = asserted;
    guiLog("PTT: %s (%s)", asserted ? "ASSERT" : "RELEASE", reason ? reason : "n/a");
    return true;
}

void App::releasePtt(const char* reason) {
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;
    std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
    if (!ptt_driver_ || !ptt_driver_->isOpen()) {
        ptt_active_ = false;
        return;
    }

    if (!ptt_driver_->setKey(ptt::PttKey::Off)) {
        guiLog("PTT: release failed (%s): %s",
               reason ? reason : "n/a",
               ptt_driver_->lastError().c_str());
    } else {
        guiLog("PTT: RELEASE (%s)", reason ? reason : "n/a");
    }
    ptt_active_ = false;
}

void App::closePtt() {
    std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
    ptt_active_ = false;
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;
    if (ptt_driver_) {
        ptt_driver_->close();
        ptt_driver_.reset();
    }
    ptt_config_ = ptt::PttConfig{};
    cat_frequency_next_open_attempt_ms_ = 0;
}

std::string App::testPtt(AppSettings settings) {
    std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
    if (!ensurePttReadyLocked(settings)) {
        return ptt_driver_ ? ptt_driver_->lastError() : "PTT driver unavailable";
    }
    if (!ptt_driver_) {
        return {};
    }
    if (!ptt_driver_->testCycle()) {
        std::string error = ptt_driver_->lastError();
        return error.empty() ? "PTT test failed" : error;
    }
    return {};
}

std::string App::testCat(AppSettings settings) {
    std::lock_guard<std::mutex> lock(ptt_driver_mutex_);
    if (!ensurePttReadyLocked(settings)) {
        return ptt_driver_ ? ptt_driver_->lastError() : "CAT driver unavailable";
    }
    if (!ptt_driver_) {
        return {};
    }
    if (!ptt_driver_->testCat()) {
        std::string error = ptt_driver_->lastError();
        return error.empty() ? "CAT test failed" : error;
    }
    return {};
}

bool App::queueRealTxSamples(const std::vector<float>& samples, const char* context) {
    if (samples.empty()) {
        return false;
    }

    if (simulation_enabled_) {
        if (!ota_audio_) {
            initOtaAudio();
        }
        if (!ota_audio_ || !ota_audio_->isConnected()) {
            const std::string status = ota_audio_ ? ota_audio_->status().text : "Disconnected";
            guiLog("%s: OTASim TX blocked: %s",
                   context ? context : "TX audio",
                   status.c_str());
            return false;
        }

        const size_t tx_duration_ms = (samples.size() * 1000) / 48000;
        tx_in_progress_ = true;
        tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

        if (waterfall_) {
            waterfall_->addSamples(samples.data(), samples.size());
        }
        if (recording_enabled_) {
            recorded_tx_samples_.insert(recorded_tx_samples_.end(), samples.begin(), samples.end());
        }

        std::string error;
        if (!ota_audio_->queueTxSamples(samples, &error)) {
            tx_in_progress_ = false;
            guiLog("%s: OTASim TX failed: %s",
                   context ? context : "TX audio",
                   error.c_str());
            return false;
        }
        return true;
    }

    // Abort pending release if a new TX starts quickly after previous frame.
    ptt_release_pending_ = false;
    ptt_release_deadline_ms_ = 0;

    // Mute RX and clear buffers to avoid local feedback decode.
    audio_.setRxMuted(true);
    audio_.stopCapture();
    audio_.clearRxBuffer();
    modem_.clearRxBuffer();

    if (!audio_.hasOutputDevice()) {
        std::string output_dev = getOutputDeviceName();
        if (!audio_.openOutput(output_dev)) {
            guiLog("%s: openOutput failed for '%s'",
                   context ? context : "TX audio",
                   output_dev.empty() ? "Default" : output_dev.c_str());
            audio_.setRxMuted(false);
            if (radio_rx_enabled_ && !audio_.isCapturing()) {
                audio_.startCapture();
            }
            return false;
        }
    }

    const ptt::PttConfig active_ptt = pttConfigFromSettings(settings_);
    if (active_ptt.mode != ptt::PttMode::None) {
        if (!setPtt(true, context ? context : "tx_start")) {
            audio_.setRxMuted(false);
            if (radio_rx_enabled_ && !audio_.isCapturing()) {
                audio_.startCapture();
            }
            return false;
        }
        if (settings_.tx_delay_ms > 0) {
            SDL_Delay(static_cast<Uint32>(settings_.tx_delay_ms));
        }
    } else {
        ptt_active_ = false;
    }

    size_t tx_duration_ms = (samples.size() * 1000) / 48000;
    tx_in_progress_ = true;
    tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

    if (waterfall_) {
        waterfall_->addSamples(samples.data(), samples.size());
    }

    if (recording_enabled_) {
        recorded_tx_samples_.insert(recorded_tx_samples_.end(), samples.begin(), samples.end());
    }

    audio_.startPlayback();
    audio_.queueTxSamples(samples);
    return true;
}

bool App::startRadioRx() {
    if (!audio_initialized_) {
        guiLog("startRadioRx guard: audio not initialized");
        return false;
    }
    if (simulation_enabled_) {
        guiLog("startRadioRx guard: simulation enabled");
        return false;
    }
    if (radio_rx_enabled_) {
        return true;
    }

    audio_.setInputCaptureMode(
        radio_rx_force_queue_mode_
            ? AudioEngine::InputCaptureMode::Queue
            : AudioEngine::InputCaptureMode::Auto);

    std::string input_dev = getInputDeviceName();
    if (!input_dev.empty()) {
        bool found = false;
        for (const auto& dev : input_devices_) {
            if (dev == input_dev) {
                found = true;
                break;
            }
        }
        if (!found) {
            guiLog("startRadioRx: configured input device missing: '%s', falling back to Default",
                   input_dev.c_str());
            input_dev.clear();
        }
    }
    if (!audio_.openInput(input_dev)) {
        guiLog("startRadioRx: openInput failed for '%s'",
               input_dev.empty() ? "Default" : input_dev.c_str());
        return false;
    }

    // Main thread polls captured samples via pollRadioRx().
    audio_.setRxCallback(AudioEngine::RxCallback{});

    audio_.setLoopbackEnabled(false);
    audio_.startCapture();
    if (!audio_.isCapturing()) {
        guiLog("startRadioRx: capture did not start");
        audio_.closeInput();
        return false;
    }
    radio_rx_enabled_ = true;
    radio_rx_started_ms_ = SDL_GetTicks();
    radio_rx_warmup_logged_ = false;
    radio_rx_first_chunk_logged_ = false;
    radio_rx_no_data_deadline_ms_ = radio_rx_started_ms_ + 1000;
    radio_rx_rearm_attempts_ = 0;
    radio_rx_rearm_exhausted_logged_ = false;
    radio_rx_active_device_ = input_dev.empty() ? "Default" : input_dev;
    radio_rx_output_prime_attempted_ = false;
    guiLog("startRadioRx: capture started on '%s'",
           input_dev.empty() ? "Default" : input_dev.c_str());
    guiLog("startRadioRx: input capture backend=%s",
           audio_.isInputQueueMode() ? "queue" : "callback");
    return true;
}

void App::stopRadioRx() {
    audio_.stopCapture();
    audio_.closeInput();
    audio_.setRxCallback(AudioEngine::RxCallback{});
    radio_rx_enabled_ = false;
    radio_rx_started_ms_ = 0;
    radio_rx_warmup_logged_ = false;
    radio_rx_first_chunk_logged_ = false;
    radio_rx_no_data_deadline_ms_ = 0;
    radio_rx_rearm_attempts_ = 0;
    radio_rx_rearm_exhausted_logged_ = false;
    radio_rx_active_device_.clear();
    radio_rx_output_prime_attempted_ = false;
}

void App::pollRadioRx() {
    if (simulation_enabled_) {
        pollOtaRx();
        return;
    }

    if (!audio_initialized_ || simulation_enabled_ || !radio_rx_enabled_) {
        return;
    }

    uint32_t now_ms = SDL_GetTicks();
    if (radio_rx_started_ms_ > 0 && (now_ms - radio_rx_started_ms_) < 200) {
        if (!radio_rx_warmup_logged_) {
            guiLog("pollRadioRx warm-up: delaying modem feed for 200ms after capture start");
            radio_rx_warmup_logged_ = true;
        }
        // Drain a small amount during warm-up to avoid unbounded growth.
        (void)audio_.getRxSamples(2048);
        return;
    }

    // Some Linux/USB stacks can start capture without delivering samples immediately.
    // Rearm capture on the SAME configured device a few times before giving up.
    if (!radio_rx_first_chunk_logged_ &&
        radio_rx_no_data_deadline_ms_ > 0 &&
        now_ms >= radio_rx_no_data_deadline_ms_) {
        SDL_AudioStatus in_status = audio_.getInputStatus();
        const char* in_status_str = (in_status == SDL_AUDIO_PLAYING) ? "playing" :
                                    (in_status == SDL_AUDIO_PAUSED)  ? "paused"  :
                                                                       "stopped";
        guiLog("pollRadioRx: no RX data diagnostics: backend=%s status=%s queued=%u bytes",
               audio_.isInputQueueMode() ? "queue" : "callback",
               in_status_str,
               static_cast<unsigned>(audio_.getQueuedInputBytes()));

#ifndef _WIN32
        // Some USB radio codecs require an active output stream to keep duplex
        // clocking alive. Prime output once before escalating capture recovery.
        if (!radio_rx_output_prime_attempted_) {
            radio_rx_output_prime_attempted_ = true;
            std::string output_dev = getOutputDeviceName();
            if (!audio_.hasOutputDevice()) {
                if (audio_.openOutput(output_dev)) {
                    audio_.startPlayback();  // plays silence until TX queue has data
                    guiLog("pollRadioRx: primed output '%s' to wake duplex capture",
                           output_dev.empty() ? "Default" : output_dev.c_str());
                } else {
                    guiLog("pollRadioRx: output prime failed for '%s'",
                           output_dev.empty() ? "Default" : output_dev.c_str());
                }
            } else {
                audio_.startPlayback();
                guiLog("pollRadioRx: output already open; playback started for duplex prime");
            }
            radio_rx_no_data_deadline_ms_ = now_ms + 1000;
            return;
        }
#endif

        // Linux USB edge case: callback capture can stall until mixer state changes.
        // Fall back to queued capture on the SAME configured device.
#ifndef _WIN32
        if (!radio_rx_force_queue_mode_ && radio_rx_rearm_attempts_ >= 2) {
            guiLog("pollRadioRx: no data on callback capture; switching '%s' to queued capture mode",
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
            audio_.stopCapture();
            audio_.closeInput();
            radio_rx_enabled_ = false;
            radio_rx_force_queue_mode_ = true;
            if (startRadioRx()) {
                guiLog("pollRadioRx: queued capture fallback armed on '%s'",
                       radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
                return;
            }
            guiLog("pollRadioRx: queued capture fallback failed on '%s'",
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
        }
#endif

        if (radio_rx_rearm_attempts_ < 4) {
            radio_rx_rearm_attempts_++;
            guiLog("pollRadioRx: no RX chunks after %ums on '%s'; rearming capture (%d/4)",
                   now_ms - radio_rx_started_ms_,
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str(),
                   radio_rx_rearm_attempts_);
            // Escalate from soft pause/unpause to hard close/open on repeated stalls.
            bool hard_reopen = audio_.isInputQueueMode() || radio_rx_rearm_attempts_ >= 3;
            if (hard_reopen) {
                std::string input_dev = getInputDeviceName();
                audio_.stopCapture();
                audio_.closeInput();
                if (!audio_.openInput(input_dev)) {
                    guiLog("pollRadioRx: hard reopen failed for '%s'",
                           input_dev.empty() ? "Default" : input_dev.c_str());
                } else {
                    audio_.setRxCallback(AudioEngine::RxCallback{});
                    audio_.setLoopbackEnabled(false);
                    audio_.startCapture();
                    guiLog("pollRadioRx: hard reopened input '%s' (backend=%s)",
                           input_dev.empty() ? "Default" : input_dev.c_str(),
                           audio_.isInputQueueMode() ? "queue" : "callback");
                }
            } else {
                audio_.stopCapture();
                audio_.startCapture();
            }
            radio_rx_no_data_deadline_ms_ = now_ms + 1000;
        } else if (!radio_rx_rearm_exhausted_logged_) {
            guiLog("pollRadioRx: still no RX chunks after rearm attempts on '%s'; check OS input level/source",
                   radio_rx_active_device_.empty() ? "Default" : radio_rx_active_device_.c_str());
            radio_rx_rearm_exhausted_logged_ = true;
        }
    }

    // Bounded drain per frame to keep UI responsive while preventing RX backlog.
    constexpr size_t kChunkSamples = 2048;
    constexpr int kMaxChunksPerFrame = 8;
    for (int i = 0; i < kMaxChunksPerFrame; ++i) {
        auto samples = audio_.getRxSamples(kChunkSamples);
        if (samples.empty()) {
            break;
        }

        if (!radio_rx_first_chunk_logged_) {
            guiLog("pollRadioRx: first RX chunk=%zu samples", samples.size());
        }
        if (recording_enabled_) {
            recorded_rx_samples_.insert(recorded_rx_samples_.end(), samples.begin(), samples.end());
        }
        modem_.feedAudio(samples);
        if (!radio_rx_first_chunk_logged_) {
            guiLog("pollRadioRx: first RX chunk fed to modem");
        }
        if (modem_.isSynchronousMode()) {
            modem_.processRxBuffer();
        }
        if (!radio_rx_first_chunk_logged_) {
            guiLog("pollRadioRx: first RX chunk modem processing complete");
            radio_rx_first_chunk_logged_ = true;
            radio_rx_no_data_deadline_ms_ = 0;
            radio_rx_rearm_attempts_ = 0;
            radio_rx_rearm_exhausted_logged_ = false;
        }
        if (waterfall_) {
            waterfall_->addSamples(samples.data(), samples.size());
        }
    }
}

void App::stopTxNow(const char* reason) {
    // Abort protocol-level retransmit/resend timers before clearing audio queues
    // so no new outbound frames are scheduled after operator stop.
    protocol_.abortTxNow();

    size_t dropped_audio = audio_.getTxQueueSize();
    if (dropped_audio > 0) {
        audio_.clearTxQueue();
    }

    tx_in_progress_ = false;
    tx_end_time_ = std::chrono::steady_clock::time_point{};
    releasePtt(reason ? reason : "stop_tx_now");

    // Return to RX immediately after aborting TX.
    if (!simulation_enabled_ && radio_rx_enabled_ && !ptt_active_) {
        audio_.setRxMuted(false);
        if (!audio_.isCapturing()) {
            audio_.startCapture();
        }
    }

    guiLog("STOP TX NOW: reason='%s', dropped_audio=%zu",
           reason, dropped_audio);

    appendRxLogLine("[SYS] TX stopped immediately");
}

void App::renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate,
                                     const protocol::ConnectionStats& conn_stats) {
    // Compact horizontal Channel Status display
    ImGui::BeginChild("ChannelStatus", ImVec2(0, 140), false);

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
        const char* wf_str = waveformDisplayName(waveform);
        ImVec4 wf_color = (waveform == protocol::WaveformMode::OFDM_COX) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (waveform == protocol::WaveformMode::MC_DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (waveform == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                       ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::Text("RX:");
        ImGui::SameLine();
        ImGui::TextColored(wf_color, "%s", wf_str);
        ImGui::SameLine();

        // Show mode-appropriate settings and throughput
        float throughput_bps = 0.0f;
        ImVec4 mode_quality_color;
        const char* mode_quality = "Good";

        // Get actual channel quality from fading measurement
        float fading = modem_.getFadingIndex();
        mode_quality = fadingToQualityWithColor(fading, mode_quality_color);

        if (waveform == protocol::WaveformMode::MC_DPSK) {
            // For MC-DPSK, just show carrier count (DQPSK R1/4 is implicit)
            int carriers = modem_.getMCDPSKCarriers();
            throughput_bps = modem_.getMCDPSKThroughput();
            ImGui::Text("%d carriers", carriers);
        } else {
            // For OFDM modes, show negotiated modulation/rate
            ImGui::Text("%s %s", modulationToString(data_mod), codeRateToString(data_rate));
            throughput_bps = config_.getTheoreticalThroughput(data_mod, data_rate);
        }
        ImGui::SameLine();
        ImGui::TextColored(mode_quality_color, "[%s]", mode_quality);
        ImGui::TextDisabled("PHY ~%.1f kbps", throughput_bps / 1000.0f);

        // Row 2: Our SNR measurement
        ImVec4 sync_color = stats.synced ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        ImGui::TextColored(sync_color, "%s", stats.synced ? "SYNC" : "----");
        ImGui::SameLine();
        ImGui::Text("SNR:");
        ImGui::SameLine();

        // SNR bar - color indicates signal strength
        float snr_normalized = stats.snr_db / 40.0f;
        snr_normalized = std::max(0.0f, std::min(1.0f, snr_normalized));
        // Color based on SNR value (green=good signal, yellow=moderate, red=weak)
        ImVec4 snr_color;
        if (stats.snr_db >= 15.0f) {
            snr_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        } else if (stats.snr_db >= 5.0f) {
            snr_color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        } else {
            snr_color = ImVec4(1.0f, 0.4f, 0.2f, 1.0f);  // Orange-red
        }
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, snr_color);
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
        const char* wf_str = waveformDisplayName(connect_wf);
        ImGui::TextDisabled("%s R1/4 (default)", wf_str);
    } else if (conn_state == protocol::ConnectionState::CONNECTING) {
        // Show actual waveform being used for connection attempt (always R1/4)
        auto connect_wf = protocol_.getConnectWaveform();
        const char* wf_str = waveformDisplayName(connect_wf);
        ImVec4 wf_color = (connect_wf == protocol::WaveformMode::OFDM_COX) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (connect_wf == protocol::WaveformMode::MC_DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (connect_wf == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                         ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::TextColored(wf_color, "%s R1/4 (calling)", wf_str);
    }
    // Connected state already shows mode in Row 1

    // Row 3: Modem frame stats
    if (stats.frames_sent > 0 || stats.frames_received > 0) {
        ImGui::Text("TX:%d RX:%d", stats.frames_sent, stats.frames_received);
        if (stats.frames_failed > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "(%d fail)", stats.frames_failed);
        }
    } else if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        ImGui::TextDisabled("Ready to connect");
    }

    // Row 4: ARQ health (control-path reliability visibility)
    const auto& arq = conn_stats.arq;
    bool has_arq_activity =
        arq.frames_sent > 0 || arq.frames_received > 0 || arq.acks_sent > 0 || arq.acks_received > 0 ||
        arq.retransmissions > 0 || arq.timeouts > 0;
    if (has_arq_activity) {
        ImVec4 arq_color = (arq.failed > 0 || arq.timeouts > 0) ? ImVec4(1.0f, 0.4f, 0.4f, 1.0f) :
                           (arq.retransmissions > 0) ? ImVec4(0.9f, 0.8f, 0.3f, 1.0f) :
                           ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
        char arq_line[256];
        snprintf(arq_line, sizeof(arq_line),
                 "ARQ retx:%d to:%d fast:%d probe:%d nack:%d dupACK:%d",
                 arq.retransmissions, arq.timeouts, arq.retransmissions_fast_hole,
                 arq.retransmissions_hole_probe, arq.retransmissions_nack,
                 arq.duplicate_acks_ignored);
        ImGui::TextColored(arq_color, "%s", arq_line);
        if (arq.failed > 0) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "fail:%d", arq.failed);
            if (arq.failed > diagnostics_last_arq_failed_) {
                diagnostics_last_arq_failed_ = arq.failed;
                ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                    "fault", "fault.triggered",
                    "{\"reason\":\"arq_failed_increment\",\"policy\":\"emit_only\"}");
            }
        }
    }

    ImGui::EndChild();
}

void App::renderOperateTab() {
    // Calculate available height for layout
    float total_height = ImGui::GetContentRegionAvail().y;

    // ========================================
    // TOP SECTION: Connection Controls (compact)
    // ========================================

    if (simulation_enabled_) {
        const auto ota_status = ota_audio_ ? ota_audio_->status() : OtaAudioStatus{};
        const ImVec4 color =
            ota_status.state == OtaAudioConnectionState::Connected ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) :
            ota_status.state == OtaAudioConnectionState::Failed ? ImVec4(1.0f, 0.35f, 0.3f, 1.0f) :
                                                                 ImVec4(1.0f, 0.8f, 0.2f, 1.0f);
        ImGui::TextColored(color, "OTASim: %s", ota_status.text.c_str());
        if (recording_enabled_) {
            ImGui::SameLine();
            const size_t total_rec = recorded_rx_samples_.size() + recorded_tx_samples_.size();
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[REC %.1fs]", total_rec / 48000.0f);
        }
    }

    // Audio initialization (only when not in simulation)
    if (!simulation_enabled_ && !audio_initialized_) {
        if (deferred_audio_auto_init_pending_) {
            ImGui::TextDisabled("Starting audio...");
            if (ImGui::Button("Start Audio Now", ImVec2(-1, 28))) {
                deferred_audio_auto_init_deadline_ms_ = 0;
            }
        } else {
            if (ImGui::Button("Initialize Audio", ImVec2(-1, 28))) {
                initAudio();
            }
        }
        return;
    }

    // ========================================
    // Connection Row: Callsign input + buttons
    // ========================================
    bool has_callsign = boundedCStringLen(settings_.callsign) >= 3;
    auto conn_state = protocol_.getState();

    // Status line
    if (!simulation_enabled_ && !radio_rx_enabled_) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "OFFLINE");
        ImGui::SameLine();
        if (ImGui::SmallButton("Start RX")) {
            if (!audio_initialized_) initAudio();
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
                state_icon = simulation_enabled_ ? "OTASIM" : "LISTENING";
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
    const bool ota_ready = !simulation_enabled_ || (ota_audio_ && ota_audio_->isConnected());
    ImGui::BeginDisabled(conn_state != protocol::ConnectionState::DISCONNECTED ||
                         !has_callsign || boundedCStringLen(remote_callsign_) < 3 ||
                         !ota_ready);
    if (ImGui::Button("Connect", ImVec2(btn_w, 0))) {
        guiLog("Connect clicked: simulation=%d, remote='%s'", simulation_enabled_, remote_callsign_);
        if (!simulation_enabled_ && !radio_rx_enabled_) {
            if (!audio_initialized_) initAudio();
            startRadioRx();
        }
        std::string remote_call(remote_callsign_, boundedCStringLen(remote_callsign_));
        protocol_.connect(remote_call);
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

    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.78f, 0.18f, 0.18f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.24f, 0.24f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.68f, 0.14f, 0.14f, 1.0f));
    if (ImGui::Button("STOP TX", ImVec2(110, 0))) {
        stopTxNow("operator_button");
    }
    ImGui::PopStyleColor(3);

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
    std::string pending_call_snapshot;
    {
        std::lock_guard<std::mutex> lock(rx_log_mutex_);
        pending_call_snapshot = pending_incoming_call_;
    }
    if (!pending_call_snapshot.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f),
                           "Incoming from %s!", pending_call_snapshot.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Accept")) {
            protocol_.acceptCall();
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending_incoming_call_.clear();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Reject")) {
            protocol_.rejectCall();
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending_incoming_call_.clear();
        }
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
    if (ImGui::SmallButton("Clear")) clearRxLog();
    auto rx_log_snapshot = snapshotRxLog();
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy")) {
        std::string all_log;
        for (const auto& msg : rx_log_snapshot) all_log += msg + "\n";
        ImGui::SetClipboardText(all_log.c_str());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Report...")) {
        diagnostics_report_popup_open_ = true;
    }
    ImGui::SameLine();
    auto mstats = modem_.getStats();
    ImGui::TextDisabled("TX:%d RX:%d", mstats.frames_sent, mstats.frames_received);

    // Calculate remaining height for message log (leave space for TX and file transfer)
    float bottom_section_height = 130;  // TX input + File transfer
    float log_height = ImGui::GetContentRegionAvail().y - bottom_section_height;
    if (log_height < 100) log_height = 100;  // Minimum height

    ImGui::BeginChild("RXLogRadio", ImVec2(-1, log_height), true);
    for (const auto& msg : rx_log_snapshot) {
        ImVec4 color(0.7f, 0.7f, 0.7f, 1.0f);
        if (msg.size() >= 4 && msg.substr(0, 4) == "[TX]") {
            color = ImVec4(0.5f, 0.8f, 1.0f, 1.0f);
        } else if (msg.size() >= 3 && msg.substr(0, 3) == "[RX") {
            color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);
        } else if ((msg.size() >= 4 && msg.substr(0, 4) == "[SIM") ||
                   (msg.size() >= 8 && msg.substr(0, 8) == "[OTASIM")) {
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
    if (!rx_log_snapshot.empty()) ImGui::SetScrollHereY(1.0f);
    ImGui::EndChild();

    // ========================================
    // BOTTOM SECTION: TX Input + File Transfer
    // ========================================
    ImGui::Separator();

    // TX Message Input
    if (tx_in_progress_) {
        if (simulation_enabled_) {
            if (std::chrono::steady_clock::now() >= tx_end_time_) {
                tx_in_progress_ = false;
            }
        } else if (audio_.isTxQueueEmpty()) {
            tx_in_progress_ = false;
            if (ptt_active_) {
                ptt_release_pending_ = true;
                ptt_release_deadline_ms_ = SDL_GetTicks() + static_cast<uint32_t>(std::max(0, settings_.tx_tail_ms));
            } else {
                audio_.setRxMuted(false);
                if (!audio_.isCapturing()) {
                    audio_.startCapture();
                }
            }
        }
    }

    if (ptt_release_pending_ && !simulation_enabled_ &&
        SDL_TICKS_PASSED(SDL_GetTicks(), ptt_release_deadline_ms_)) {
        releasePtt("tx_tail_elapsed");
        if (radio_rx_enabled_) {
            audio_.setRxMuted(false);
            if (!audio_.isCapturing()) {
                audio_.startCapture();
            }
        }
    }

    bool can_send = !tx_in_progress_ && boundedCStringLen(tx_text_buffer_) > 0 &&
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
            appendRxLogLine("[TX] " + text);
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
        // Show bytes transferred / total with appropriate units
        if (progress.total_bytes >= 1024) {
            ImGui::Text("%.1f/%.1f KB (%.0f%%)",
                progress.transferred_bytes / 1024.0f,
                progress.total_bytes / 1024.0f,
                progress.percentage());
        } else {
            ImGui::Text("%u/%u B (%.0f%%)",
                progress.transferred_bytes, progress.total_bytes,
                progress.percentage());
        }
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
                             boundedCStringLen(file_path_buffer_) > 0;
        ImGui::BeginDisabled(!can_send_file);
        if (ImGui::Button("Send##file", ImVec2(60, 0))) {
            std::string file_path(file_path_buffer_, boundedCStringLen(file_path_buffer_));
            startFileSendOrImageDialog(file_path);
        }
        ImGui::EndDisabled();
    }

    renderImageSendModal();
}

} // namespace gui
} // namespace ultra
