#include "app.hpp"
#include "gui/modem/modem_protocol_binding.hpp"
#include "protocol/connection_policy.hpp"  // coherenceAdjustedFadingIndex (RX-label channel class)
#include "otasim_client/ota_rx_pump.hpp"
#include "diagnostics/diagnostics_recorder.hpp"
#include "imgui.h"
#include "ptt/ptt_driver_factory.hpp"
#include "ultra/build_info.hpp"
#include "ultra/logging.hpp"
#include "ultra/tx_burst_normalization.hpp"
#include <SDL.h>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <cstdarg>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <limits>
#include <deque>
#include <vector>
#include <utility>
#include <thread>
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

static bool isInQsoDataFrame(const Bytes& frame) {
    auto header = protocol::v2::parseHeader(frame);
    return header.valid && protocol::v2::isDataFrame(header.type);
}

static bool isDisconnectTeardownControlFrame(const Bytes& frame) {
    const auto header = protocol::v2::parseHeader(frame);
    return header.valid && header.seq == protocol::v2::DISCONNECT_SEQ &&
           (header.type == protocol::v2::FrameType::DISCONNECT ||
            header.type == protocol::v2::FrameType::ACK);
}

static bool hasOperatorTimestampPrefix(const std::string& msg) {
    return msg.size() >= 11 &&
           msg[0] == '[' &&
           std::isdigit(static_cast<unsigned char>(msg[1])) &&
           std::isdigit(static_cast<unsigned char>(msg[2])) &&
           msg[3] == ':' &&
           std::isdigit(static_cast<unsigned char>(msg[4])) &&
           std::isdigit(static_cast<unsigned char>(msg[5])) &&
           msg[6] == ':' &&
           std::isdigit(static_cast<unsigned char>(msg[7])) &&
           std::isdigit(static_cast<unsigned char>(msg[8])) &&
           msg[9] == ']' &&
           msg[10] == ' ';
}

static std::string timestampOperatorLine(const std::string& msg) {
    if (hasOperatorTimestampPrefix(msg)) {
        return msg;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#ifdef _WIN32
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif
    char prefix[16];
    std::strftime(prefix, sizeof(prefix), "[%H:%M:%S] ", &local_tm);
    return std::string(prefix) + msg;
}

static size_t operatorLineBodyOffset(const std::string& msg) {
    return hasOperatorTimestampPrefix(msg) ? 11u : 0u;
}

static bool operatorLineStartsWith(const std::string& msg, const char* prefix) {
    const size_t offset = operatorLineBodyOffset(msg);
    const size_t len = std::strlen(prefix);
    return msg.size() >= offset + len &&
           msg.compare(offset, len, prefix) == 0;
}

static bool envFlagEnabled(const char* name) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return false;
    }
    return std::strcmp(value, "0") != 0 &&
           std::strcmp(value, "false") != 0 &&
           std::strcmp(value, "FALSE") != 0 &&
           std::strcmp(value, "off") != 0 &&
           std::strcmp(value, "OFF") != 0;
}

static uint32_t envUInt(const char* name, uint32_t fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const unsigned long parsed = std::strtoul(value, &end, 10);
    if (end == value) {
        return fallback;
    }
    return static_cast<uint32_t>(std::min<unsigned long>(parsed, 1000000ul));
}

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
// Boundaries are the SINGLE source in waveform_selection.hpp (kFading*Max) — the
// display MUST match what the rate selector classifies, or the operator sees
// "Moderate" while the ladder runs a Good rung (and vice versa).
// Combined index = freq_cv + temporal_cv (includes Doppler spread measurement)
// Calibrated cluster centers: AWGN ~0.04, Good ~0.62, Moderate ~0.90.
static const char* fadingToQuality(float fading) {
    if (fading < protocol::kFadingAwgnMax) return "AWGN";
    if (fading < protocol::kFadingGoodMax) return "Good";
    if (fading < protocol::kFadingModerateMax) return "Moderate";
    return "Poor";
}

// Same as above but also sets color for GUI display
static const char* fadingToQualityWithColor(float fading, ImVec4& color) {
    if (fading < protocol::kFadingAwgnMax) {
        color = ImVec4(0.0f, 1.0f, 0.5f, 1.0f);  // Cyan
        return "AWGN";
    } else if (fading < protocol::kFadingGoodMax) {
        color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        return "Good";
    } else if (fading < protocol::kFadingModerateMax) {
        color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        return "Moderate";
    } else {
        color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f);  // Orange
        return "Poor";
    }
}

// User-friendly waveform name (hides internal variants like OFDM_CHIRP/OFDM_NARROW)
static const char* waveformDisplayName(protocol::WaveformMode mode) {
    switch (mode) {
        case protocol::WaveformMode::MC_DPSK: return "MC-DPSK";
        case protocol::WaveformMode::MFSK: return "MFSK";
        case protocol::WaveformMode::OTFS_EQ:
        case protocol::WaveformMode::OTFS_RAW: return "OTFS";
        case protocol::WaveformMode::OFDM_NARROW: return "OFDM Narrow";
        case protocol::WaveformMode::OFDM_CHIRP:
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

App::App() : App(Options{}) {}

App::App(const Options& opts) : options_(opts), simulation_enabled_(opts.enable_sim) {
    guiLog("=== GUI Started ===");
    if (options_.record_audio) {
        recording_enabled_ = true;
        guiLog("Recording enabled (-rec): base path '%s'", options_.record_path.c_str());
    }
    scenario_active_ = !options_.auto_connect.empty() || options_.auto_accept ||
                       !options_.auto_send_file.empty() ||
                       !options_.auto_send_message.empty() ||
                       !options_.auto_reply_message.empty() ||
                       options_.auto_cancel_file_after_sec > 0 ||
                       options_.auto_disconnect_after_sec > 0 ||
                       options_.exit_after_sec > 0;
    if (scenario_active_) {
        if (!options_.auto_send_message.empty() || !options_.auto_send_file.empty()) {
            tx_text_buffer_[0] = '\0';
        }
        guiLog("[scenario] scripting active: auto_connect='%s' auto_accept=%d "
               "send_file='%s' send_msg='%s' msg_delay=%ds msg_after_file=%d cancel_file_after=%ds "
               "disconnect_after=%ds exit_after=%ds",
               options_.auto_connect.c_str(), options_.auto_accept ? 1 : 0,
               options_.auto_send_file.c_str(), options_.auto_send_message.c_str(),
               options_.auto_message_start_delay_sec,
               options_.auto_message_after_file ? 1 : 0,
               options_.auto_cancel_file_after_sec,
               options_.auto_disconnect_after_sec, options_.exit_after_sec);
    }
    operator_log_file_suppressed_ = envFlagEnabled("ULTRA_GUI_OPERATOR_LOG_SUPPRESS");
    operator_log_slow_ms_ = envUInt("ULTRA_GUI_OPERATOR_LOG_SLOW_MS", 0);
    operator_event_drain_limit_ =
        static_cast<size_t>(envUInt("ULTRA_GUI_OPERATOR_EVENT_DRAIN_LIMIT", 128));
    operator_event_queue_limit_ = std::clamp<size_t>(
        static_cast<size_t>(envUInt("ULTRA_GUI_OPERATOR_EVENT_QUEUE_LIMIT",
                                    static_cast<uint32_t>(MAX_OPERATOR_EVENTS))),
        1,
        MAX_OPERATOR_EVENTS);
    if (operator_log_file_suppressed_ || operator_log_slow_ms_ > 0 ||
        operator_event_drain_limit_ != 128 ||
        operator_event_queue_limit_ != MAX_OPERATOR_EVENTS) {
        guiLog("OPERATOR_EVENT_QUEUE mode: suppress_file=%d slow_ms=%u drain_limit=%zu limit=%zu max=%zu",
               operator_log_file_suppressed_ ? 1 : 0,
               operator_log_slow_ms_,
               operator_event_drain_limit_,
               operator_event_queue_limit_,
               MAX_OPERATOR_EVENTS);
    }
    // Load persistent settings
    settings_.load();
    LOG_INFO("AUDIO",
             "TX drive (tx_drive=%.3f) now controls per-burst peak target, "
             "not post-mix attenuation. Previous tx_drive = 0.8 behavior is "
             "replaced by per-burst peak normalization to tx_drive's value.",
             settings_.tx_drive);

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

    config_ = presets::balanced();

    // Use dedicated RX decode thread by default.
    modem_.setSynchronousMode(false);

    if (!options_.disable_waterfall) {
        waterfall_ = std::make_unique<WaterfallWidget>();
    } else {
        guiLog("Waterfall disabled by startup option");
    }

    // Initialize protocol with saved callsign. In -sim mode the OTASim
    // --station-id is what the peer uses to address us at the protocol
    // level (the GUI's "Connect to <remote>" passes the station id); so
    // force the modem's local callsign to match, otherwise deliverFrame()
    // would drop every incoming frame as "different station".
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
        protocol_.setLocalCallsign(local_call);
        modem_.setLogPrefix(local_call);
    }

    protocol_.setSoftCombiningHARQ(true);
    modem_.setSoftCombineBuffer(protocol_.softCombineBuffer());
    // All modem->protocol forwarding (HARQ ctx, RX-data, burst-group delivery, data-sync,
    // tone-burst GROUP_ACK) lives in the SHARED wireModemToProtocol() so the GUI and
    // ultra_tnc bind identically (gui/modem/modem_protocol_binding.hpp). GUI-specific
    // reactions to a decoded frame (monitor-mode log) ride the after_rx_data hook;
    // ping/status stay GUI-owned below.
    ModemProtocolFrontendHooks modem_hooks;
    modem_hooks.after_rx_data = [this](const Bytes& data, float snr_db, float fading,
                                       SNRSource snr_source, bool /*used_for_quality*/) {
        // Cache the latest in-band decode SNR (lock-free) so the tone-burst ACK
        // callback can pick its §15.5 staircase symbol duration without calling
        // protocol_ (which holds its mutex while invoking that callback; re-entry
        // self-deadlocks). The ACK callback gates on the source being trusted.
        // Fading rides along: the staircase fast edge is basis-dependent
        // (fade-effective SNR -> 16 dB edge; AWGN -> 18 dB).
        cached_inband_snr_db_.store(snr_db, std::memory_order_relaxed);
        cached_inband_snr_source_.store(snr_source, std::memory_order_relaxed);
        cached_fading_index_.store(fading, std::memory_order_relaxed);
        // BUG-STAIRCASE-SNAPSHOT-INPUT: feed the median ring alongside the
        // last-value cache (the staircase reads the median under ULTRA_ACK_SNR_MEDIAN).
        cached_snr_ring_[cached_snr_ring_idx_.fetch_add(1, std::memory_order_relaxed) %
                         cached_snr_ring_.size()]
            .store(snr_db, std::memory_order_relaxed);
        // Monitor mode: surface every decoded frame's payload in the RX log regardless of
        // addressing (the protocol layer would otherwise drop frames whose dst hash doesn't
        // match local call, making the GUI silent on OTA captures from peers).
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
    };
    ultra::gui::wireModemToProtocol(modem_, protocol_, std::move(modem_hooks));

    // BUG-MC-RETRY-SPURIOUS (2026-07-04): the MODE_CHANGE retry deadline holds while
    // our TX is keyed — half-duplex: keyed time is not ACK-loss evidence (the E1
    // forensics: the frame rode the tail of a ~10.6 s bundled key-down and the
    // request-anchored deadline retried spuriously on every trough exchange).
    // tx_in_progress_ is the app's atomic key-down truth; called from the engine
    // tick under its mutex — atomic read only.
    protocol_.setTxActiveProvider(
        [this] { return tx_in_progress_.load(std::memory_order_relaxed); });

    // Keepalive ACK universal busy gate: don't re-emit an ACK while a burst is
    // arriving (would collide). Mirrors the GUI's own listen-before-ACK signal.
    protocol_.setChannelBusyQuery([this] {
        return modem_.channelBusyForTx() ||
               modem_.burstAirSamplesRemaining() > 0;
    });

    // Set up status callback to show codeword progress in RX log
    modem_.setStatusCallback([this](const std::string& status) {
        enqueueOperatorLogLine(status);
    });

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
            modem_.setHandshakeComplete(true);  // monitor mode deliberately skips CONNECT/ACK
            // Post-handshake, OFDM data frames are fixed 4-CW. Without this the
            // decoder treats incoming as 1-CW control frames and falsely rejects
            // the data symbols past the first ~9 OFDM symbols as noise.
            modem_.setFixedFrameCodewords(4);
            appendRxLogLine("[monitor] " + options_.monitor_mode + " forced; "
                            + options_.monitor_modulation + " "
                            + options_.monitor_rate + "; 4-CW fixed; handshake skipped");
        }
    }

    // Set up protocol engine callbacks
    protocol_.setTxDataCallback([this](const Bytes& data,
                                       bool expect_full_ofdm_anchor_after_tx) {
        const bool in_qso_data = isInQsoDataFrame(data);
        const bool disconnect_control = isDisconnectTeardownControlFrame(data);
        if (disconnect_teardown_active_.load(std::memory_order_acquire) &&
            !disconnect_control) {
            LOG_MODEM(WARN,
                      "Teardown egress: dropped non-close protocol frame before encode");
            return;
        }
        if (in_qso_data && shouldDeferInQsoDataForTx()) {
            deferTxFrame(data, "TX audio", expect_full_ofdm_anchor_after_tx);
            return;
        }

        // When protocol layer wants to transmit, convert to audio
        auto samples = modem_.transmit(data);
        if (!samples.empty()) {
            queueRealTxSamples(samples, "TX audio", in_qso_data,
                               disconnect_control);
            if (expect_full_ofdm_anchor_after_tx) {
                modem_.expectFullOFDMAnchorOnce();
            }
        }
    });

    // Burst TX callback - encode multiple frames as a single waveform burst
    protocol_.setTransmitBurstCallback([this](const std::vector<Bytes>& frames,
                                              uint16_t group_seq,
                                              uint8_t anchor_reason) {
        if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
            LOG_MODEM(WARN,
                      "Teardown egress: dropped queued DATA burst before encode");
            return;
        }
        // Bind the resend/mode-switch anchor to this exact physical request.
        // A shared encoder latch cannot be armed before CCA deferral: an older
        // queued burst could consume a newer repair's latch, or a purged request
        // could leak it into an unrelated future burst.
        const bool full_group_anchor =
            anchor_reason == protocol::Connection::kAnchorReasonResend ||
            anchor_reason == protocol::Connection::kAnchorReasonModeSwitch;
        const BurstAnchorOptions anchor_options{
            full_group_anchor,
            anchor_reason == protocol::Connection::kAnchorReasonModeSwitch,
            anchor_reason != protocol::Connection::kAnchorReasonNone};
        const bool in_qso_data = std::any_of(
            frames.begin(), frames.end(),
            [](const Bytes& frame) { return isInQsoDataFrame(frame); });
        if (in_qso_data && shouldDeferInQsoDataForTx()) {
            deferTxBurst(frames, "TX burst audio", group_seq,
                         anchor_options);
            return;
        }

        // Match the TX encoder Z to the connection's per-burst traffic-class policy
        // (long LDPC for file/bulk bursts) so the encoder Z == the chunker Z and the
        // BURST_HEADER descriptor it writes. Same thread as the encode below.
        modem_.setBurstLiftingZ(static_cast<uint8_t>(protocol_.selectBurstLiftingZ()));
        auto samples = modem_.transmitBurst(frames, group_seq,
                                            anchor_options);
        if (!samples.empty()) {
            queueRealTxSamples(samples, "TX burst audio", in_qso_data);
        }
    });

    // §15 step 4d-iii: parallel tone-burst ACK emit. Fires from Connection's
    // setSendGroupAck callback (alongside the existing OFDM GROUP_ACK
    // transmitFrame). The OFDM ACK + tone-burst both go on the wire;
    // receiver's tone-burst monitor wins on speed. After multi-seed GUI
    // verification, a later commit drops the OFDM ACK emit to capture
    // the actual goodput delta.
    protocol_.setTransmitToneBurstAckCallback(
        [this](const ultra::waveform::tone_burst_ack::ToneBurstAckPayload& tba,
               bool inbound_group_complete) {
            if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
                return;
            }
            // §15.5 staircase: scale the ACK symbol duration to the latest in-band
            // SNR — a shorter ACK at high SNR cuts the per-turnaround airtime
            // (850 ms -> 408 ms at >=18 dB); a longer ACK at low SNR adds
            // matched-filter integration so it isn't missed (a missed ACK costs a
            // full retransmit). Read the LOCK-FREE cached SNR (written off the
            // modem RX path); NEVER call protocol_ here — this callback runs while
            // protocol_ holds its mutex, so re-entry self-deadlocks. Only a
            // trusted physical in-band source shortens; otherwise safe baseline.
            // The data-sender's monitor scans every staircase duration, so a
            // conservative (longer) choice is always decodable.
            uint32_t symbol_ms = ultra::waveform::tone_burst_ack::kBaselineSymbolMs;
            const SNRSource src = cached_inband_snr_source_.load(std::memory_order_relaxed);
            const float current_snr =
                cached_inband_snr_db_.load(std::memory_order_relaxed);
            float staircase_snr = current_snr;
            bool fading_present = false;
            if (src == SNRSource::IDLE_IN_BAND || src == SNRSource::OFDM_BROADBAND) {
                // Fade-aware fast edge (BUG-ACK-STAIRCASE-FADE-BIN): the cached
                // in-band SNR is fade-effective on a fading channel, so the fast
                // rung's edge is 16 dB there (18 dB on AWGN). Uses the same
                // AWGN-vs-fading split as capInitialOFDMRate (kFadingAwgnMax);
                // the Good-vs-Moderate distinction is deliberately NOT consulted
                // (the classifier can't make it). ULTRA_ACK_FADE_EDGE=0 opts out.
                static const bool kFadeEdgeEnabled = [] {
                    const char* e = std::getenv("ULTRA_ACK_FADE_EDGE");
                    return !(e && *e == '0');
                }();
                fading_present =
                    kFadeEdgeEnabled &&
                    cached_fading_index_.load(std::memory_order_relaxed) >=
                        ultra::protocol::kFadingAwgnMax;
                // BUG-STAIRCASE-SNAPSHOT-INPUT fix (ULTRA_ACK_SNR_MEDIAN, default
                // OFF): read the MEDIAN of the last 5 readings instead of the last
                // value — one erasure-slot/trough snapshot (F98: 9.4 written over a
                // 22 dB channel → 1.7 s ACK → phantom demote → 54 s saga) cannot
                // move a median-of-5; genuine decline still tracks in ~3 groups.
                static const bool kAckSnrMedian = [] {
                    const char* e = std::getenv("ULTRA_ACK_SNR_MEDIAN");
                    return !(e && e[0] == '0');  // DEFAULT-ON 2026-07-05 (campaign flip)
                }();
                if (kAckSnrMedian) {
                    std::array<float, 5> v;
                    for (size_t i = 0; i < v.size(); ++i) {
                        v[i] = cached_snr_ring_[i].load(std::memory_order_relaxed);
                    }
                    std::sort(v.begin(), v.end());
                    staircase_snr = v[v.size() / 2];
                }
                // NO SCALE COMPENSATION. The staircase edges were RE-MEASURED on the
                // honest in-band scale (tools/measure_ack_staircase, 2026-08-01), so every
                // source — OFDM_BROADBAND and IDLE alike — is now consumed in the units it
                // is actually reported in. The former
                // `+= kOfdmLegacyAnchorScaleOffsetDb` existed only because the old edges
                // were tuned against the pre-2026-07-07 estimator; it also could not be
                // right for both columns at once (13 dB too conservative on AWGN, 2.7 dB
                // too aggressive on fading). Deleted with the table it patched.
                symbol_ms = ultra::waveform::tone_burst_ack::symbolMsForSNR(
                    staircase_snr, fading_present);
            }
            // Staircase decision trace (BUG-ACK-STAIRCASE-FADE-BIN validation): the
            // inputs behind every ACK duration — greppable on sim and rig.
            LOG_MODEM(INFO,
                      "ToneBurstAck staircase: symbol_ms=%u snr_used=%.1f "
                      "snr_current=%.1f src=%s fading_index=%.2f "
                      "fading_present=%d",
                      symbol_ms, staircase_snr, current_snr, snrSourceToString(src),
                      cached_fading_index_.load(std::memory_order_relaxed),
                      fading_present ? 1 : 0);
            auto samples = modem_.transmitToneBurstAck(tba, symbol_ms);
            if (!samples.empty()) {
                // LISTEN-BEFORE-ACK (2026-07-05, F124: 4 ACKs keyed over the
                // sender's incoming audio — the primary ACK path had NO channel
                // sense; keying blanks our own RX of the incoming group head and
                // manufactures craters). If any independent inbound sensor says
                // unsafe, DEFER to the GUI tick. Confirmed quiet sends; at the
                // deadline, decoder/geometry evidence drops the stale ACK while
                // energy-only busy sends (the adaptive CCA floor can false-alarm).
                // ULTRA_ACK_CCA_DEFER_MS=0 opts out (old immediate-key behavior).
                static const uint32_t kAckCcaDeferMs = [] {
                    if (const char* e = std::getenv("ULTRA_ACK_CCA_DEFER_MS")) {
                        const long v = std::atol(e);
                        if (v >= 0 && v <= 10000) return static_cast<uint32_t>(v);
                    }
                    return 2500u;  // default ON
                }();
                // F176 GEOMETRIC ACK GATE: the decoder publishes the declared
                // air-end of the group being received (descriptor geometry, on
                // the sample clock). Never key an ack before the burst we are
                // acking has finished ARRIVING — the energy CCA alone proved
                // fade-fragile four times tonight (mislearned floor read a live
                // 0.06-0.13 RMS burst tail as idle=1 and keyed over it). The
                // deadline covers the remaining airtime + margin, so the gate
                // is bounded by the wire's own geometry, never open-ended.
                const uint64_t air_rem = modem_.burstAirSamplesRemaining();
                // The descriptor/header can be lost while a later classic DATA
                // member still decodes and arms the delayed SACK timer.  In that
                // state there is no declared air-end, and MPG@20 showed CCA reading
                // the live faded burst as idle.  The decoder's recent sync/decode
                // stamp is therefore an independent third listen-before-talk gate,
                // but ONLY for asynchronous timer/standalone ACKs. A synchronous
                // group-boundary ACK has equally fresh decoder evidence and is
                // causally safe: the decoder just declared that physical turn done.
                const bool rx_evidence = modem_.rxSignalActive(
                    protocol::connection_policy::kDescriptorLostReverseTxHoldMs);
                const bool channel_busy = modem_.channelBusyForTx();
                if (protocol::connection_policy::shouldDeferToneBurstAck(
                        kAckCcaDeferMs > 0, channel_busy, air_rem, rx_evidence,
                        inbound_group_complete)) {
                    const uint32_t air_rem_ms =
                        static_cast<uint32_t>(air_rem / 48);  // 48 kHz
                    const uint32_t defer_ms = std::min<uint32_t>(
                        12000, std::max(kAckCcaDeferMs, air_rem_ms + 500));
                    LOG_MODEM(INFO,
                              "ToneBurstAck: deferring up to %u ms (%s%s%u ms of "
                              "burst still arriving)",
                              defer_ms,
                              channel_busy ? "channel busy; " :
                              (rx_evidence ? "decoder RX evidence; " : ""),
                              air_rem > 0 ? "" : "no ",
                              air_rem_ms);
                    std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
                    ack_defer_samples_ = std::move(samples);
                    ack_defer_deadline_ =
                        std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(defer_ms);
                    // A newer cumulative ACK replaces the pending one.  Its
                    // quiet-confirmation window must start over as well: carrying
                    // two quiet ticks from the superseded ACK could transmit the
                    // replacement after a single tick, inside the same peer burst
                    // whose later frame generated it.
                    ack_defer_quiet_ticks_ = 0;
                    ack_defer_inbound_group_complete_ = inbound_group_complete;
                    ack_defer_pending_ = true;
                } else {
                    // A newer, causally-safe group-boundary ACK supersedes any
                    // older deferred timer ACK. Leaving the old copy pending would
                    // key stale cumulative state after this immediate transmission.
                    {
                        std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
                        ack_defer_pending_ = false;
                        ack_defer_samples_.clear();
                        ack_defer_quiet_ticks_ = 0;
                        ack_defer_inbound_group_complete_ = false;
                    }
                    submitToneAckSamples(samples);
                }
            }
        });

    // §15 step 4d-late: when Connection just queued a data burst, arm
    // the receiver's tone-burst monitor for the ACK window. The monitor
    // runs detection at a tight cadence during the armed window and is
    // idle otherwise — no audio-thread CPU outside the window, no
    // waterfall jitter.
    protocol_.setArmToneBurstAckMonitorCallback(
        [this](uint32_t window_ms) {
            // A Connection callback represents a newly committed stop-and-wait DATA
            // round, so replace (rather than only extend) the prior round's deadline.
            modem_.rearmToneBurstAckMonitor(window_ms);
        });

    // Software-ALC sender side (BUG-QAM16-RIG-LEVEL-BUDGET): the peer's per-burst
    // RX level verdict rides back on the tone-burst ACK (drive_advisory bits).
    // Runs under the ProtocolEngine mutex — handleDriveAdvisory touches only the
    // ALC atomics + logging, NEVER protocol_ (same discipline as the tone-burst
    // ACK TX callback above).
    protocol_.setDriveAdvisoryCallback(
        [this](uint8_t advisory, uint8_t group_seq) {
            handleDriveAdvisory(advisory, group_seq);
        });

    // Message turn reversals use these anchor callbacks even when the stronger
    // half-duplex-interactive burst gate is not requested. The first frame after
    // acquiring the DATA turn must carry a full chirp+LTS anchor, and the yielding
    // peer must cold-acquire it. Keep setHalfDuplexInteractive opt-in because it also
    // changes file/B2F burst scheduling semantics.
    protocol_.setDataTurnAcquiredCallback([this]() { modem_.forceNextFrameFullPreamble(); });
    protocol_.setFullOFDMAnchorExpectedCallback([this]() { modem_.expectFullOFDMAnchorOnce(); });
    if (options_.half_duplex_interactive) {
        protocol_.setHalfDuplexInteractive(true);
        guiLog("[scenario] half-duplex interactive enabled (bidirectional role-swap)");
    }


    protocol_.setMessageReceivedCallback([this](const std::string& from, const std::string& text) {
        if (text.empty()) {
            return;
        }
        enqueueMessageReceived(from, text);
    });
    protocol_.setMessageTxStatusCallback([this](const protocol::ProtocolEngine::MessageTxStatusEvent& event) {
        enqueueMessageTxStatus(event);
    });

    protocol_.setDisconnectTeardownCallback(
        [this](bool active) { setDisconnectTeardownActive(active); });

    protocol_.setConnectionChangedCallback([this](protocol::ConnectionState state, const std::string& info) {
        // Cache for the carrier-sense gate (read on the TX-callback path where
        // calling protocol_.getState() would re-enter the engine mutex).
        conn_state_cached_.store(state, std::memory_order_relaxed);
        // #70 STAGE2: any real transition out of DISCONNECTED (e.g. the initiator's CONNECT
        // landed -> CONNECTED) resolves the responder's expects-CONNECT window, so clear the
        // re-arm deadline; bare_chirp_expected_ is then set correctly by the state->flag
        // mapping below (CONNECTED/CONNECTING -> false).
        if (state != protocol::ConnectionState::DISCONNECTED) {
            responder_connect_expected_until_ms_.store(0, std::memory_order_relaxed);
        }
        // Software-ALC is per-connection: latch the configured baseline at CONNECT
        // and drop back to it at DISCONNECT (the walked drive must never leak into
        // the next session's handshake or a different peer's link budget).
        if (state == protocol::ConnectionState::CONNECTED ||
            state == protocol::ConnectionState::DISCONNECTED) {
            resetSoftwareAlc(state == protocol::ConnectionState::CONNECTED
                                 ? "connect" : "disconnect");
        }
        guiLog("Connection state changed: %d (%s)", static_cast<int>(state), info.c_str());

        // Update modem engine connection state (affects waveform selection)
        // Stay "connected" during DISCONNECTING so we can receive the ACK via OFDM
        bool modem_connected = (state == protocol::ConnectionState::CONNECTED ||
                                state == protocol::ConnectionState::DISCONNECTING);

        // DISCONNECTING still needs the negotiated coherent OFDM receiver for the
        // peer's hardened QPSK R1/4 ACK, but it must no longer speculate that a
        // failed control candidate is a multi-codeword DATA frame.  A false lock
        // on the final ACK tail previously escalated to the connected data geometry
        // and occupied the decoder across the real disconnect ACK.  Publish the
        // teardown phase separately from the coarse connected bit.
        modem_.setControlOnlyReceive(
            state == protocol::ConnectionState::DISCONNECTING ||
            disconnect_teardown_active_.load(std::memory_order_acquire));

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
        // #70 stage 2: robust-idle-ping (ULTRA_ROBUST_IDLE_PING) is eligible only
        // when this station next expects a BARE-CHIRP control frame — idle (incoming
        // PING) or PROBING (the PONG). While CONNECTING it expects a CONNECT_ACK
        // (data frame); a badly-faded CONNECT_ACK must be retried-as-data, NOT mis-
        // PONGed (the IONOS #27 regression). No-op unless the knob is set.
        modem_.setBareChirpExpected(
            state == protocol::ConnectionState::PROBING ||
            state == protocol::ConnectionState::DISCONNECTED);
        if (wrap_audio_quiesce) {
            audio_.drainInput();
            audio_.resumeInput();
        }
        was_modem_connected_ = modem_connected;

        std::string msg;
        switch (state) {
            case protocol::ConnectionState::PROBING:
                connected_peer_snr_valid_ = false;
                msg = "[SYS] Probing " + info + "...";
                {
                    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                    diagnostics.ensureSessionActive();
                    diagnostics.emitText("session", "session.state", "{\"state\":\"probing\"}");
                }
                break;
            case protocol::ConnectionState::CONNECTING:
                connected_peer_snr_valid_ = false;
                msg = "[SYS] Connecting to " + info + "...";
                {
                    auto& diagnostics = ultra::diagnostics::DiagnosticsRecorder::instance();
                    diagnostics.ensureSessionActive();
                    diagnostics.emitText("session", "session.state", "{\"state\":\"connecting\"}");
                }
                break;
            case protocol::ConnectionState::CONNECTED:
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
                connected_peer_snr_valid_ = false;
                // A scripted endpoint may quit only after the protocol has actually
                // completed teardown (ACK, responder grace, or protocol timeout).  A
                // fixed delay from DISCONNECT TX is unsafe on the hardware path: fading
                // acquisition plus the full control waveform can consume that entire
                // delay before the peer's ACK even starts.  The callback may run on the
                // RX thread, so publish a one-bit request for tickScenario() to consume.
                // wrap_audio_quiesce proves this was a real connected session, not a
                // failed connection attempt that never reached CONNECTED.
                if (scenario_active_ && wrap_audio_quiesce) {
                    scenario_disconnect_complete_pending_.store(
                        true, std::memory_order_relaxed);
                }
                if (info.find("timeout") != std::string::npos) {
                    msg = "[FAILED] " + info;  // Make failures more visible
                    // A scripted caller has already exhausted the protocol's
                    // retry budget.  There is no payload or teardown left to
                    // drive, so retain a short log-flush grace and exit.  The
                    // old behavior kept the GUI/audio lease alive until the
                    // conservative --exit-after deadline (900 s in the IONOS
                    // runner), masking a known-dead test as a live transfer.
                    // Never apply this to the responder or an interactive GUI.
                    if (scenario_active_ && !options_.auto_connect.empty() &&
                        scenario_connect_issued_ && !scenario_connected_seen_ &&
                        !scenario_terminal_failure_) {
                        scenario_terminal_failure_ = true;
                        scenario_terminal_failure_at_ =
                            std::chrono::steady_clock::now();
                        guiLog("[scenario] terminal connect failure; quitting "
                               "after log grace");
                    }
                } else {
                    msg = "[SYS] Disconnected" + (info.empty() ? "" : ": " + info);
                }
                // Drop any carrier-sense-deferred TX so a stale burst can't fire
                // into the next session.
                if (!deferred_tx_.empty()) {
                    guiLog("CCA: clearing %zu deferred TX burst(s) on disconnect",
                           deferred_tx_.size());
                    deferred_tx_.clear();
                }
                // Reset waveform mode to OFDM when disconnected
                modem_.setWaveformMode(protocol::WaveformMode::OFDM_CHIRP);
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

    protocol_.setIncomingCallCallback([this](const std::string& from) {
        {
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending_incoming_call_ = from;
        }
        std::string msg = "[SYS] Incoming call from " + from;
        appendRxLogLine(msg);
    });

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
        // #70 STAGE2: we just PONGed, so we are the RESPONDER and the initiator's CONNECT (a
        // DATA frame) is now expected — disarm the robust bare-chirp PING emit exactly as the
        // initiator does on PROBING->CONNECTING, so a badly-faded CONNECT is retried-as-data,
        // not re-PONGed (the responder-side analogue of #27). Window covers one CONNECT
        // reception + margin (the initiator sends CONNECT immediately on PONG; a 4-CW MC-DPSK
        // frame + turnaround is ~10-15 s). SOUND FOR ANY WINDOW: if it re-arms mid-attempt and
        // re-PONGs a still-faded CONNECT, the initiator is CONNECTING and half-duplex-TXing, so
        // that PONG is a verified no-op (connection_handlers.cpp:39-48) and this same callback
        // just re-armed the FALSE window — permanent starvation becomes occasional harmless
        // waste. Shorter is chosen to bound the rare lost-PONG deafness (initiator re-PINGs from
        // PROBING); the tick re-arms after the window. Rig-tunable. No-op unless the knob is on.
        constexpr uint32_t kResponderConnectExpectedMs = 20000;  // ~1 CONNECT reception + margin
        modem_.setBareChirpExpected(false);
        responder_connect_expected_until_ms_.store(
            SDL_GetTicks() + kResponderConnectExpectedMs, std::memory_order_relaxed);
    });

    // Wire up modem ping detection to protocol
    modem_.setPingReceivedCallback([this](float snr) {
        // The PONG-detection `snr` is the chirp-correlation CONFIDENCE (SYNC_QUALITY) — it
        // over-reads and is NOT the link SNR (CLAUDE.md: operator-facing SNR = receiver in-band
        // SNR; only IDLE_IN_BAND/OFDM_BROADBAND are physical). Show the physical in-band SNR when
        // the receiver has one; otherwise report the contact with NO dB rather than print the sync
        // proxy as if it were the SNR (that mis-showed ~26 dB on a ~20 dB IONOS link).
        const auto phys = selectOperatorSNRDisplay(modem_.getStats());
        char snr_phrase[56];
        if (phys.valid) {
            snprintf(snr_phrase, sizeof(snr_phrase), " (SNR=%.0f dB %s)",
                     phys.snr_db, snrSourceDisplayLabel(phys.source));
        } else {
            snr_phrase[0] = '\0';  // no trustworthy in-band SNR at handshake yet
        }
        // Check state to show appropriate message
        if (protocol_.getState() == protocol::ConnectionState::PROBING) {
            guiLog("RX PONG: Remote station responded!%s", snr_phrase);
            char buf[128];
            snprintf(buf, sizeof(buf), "[PONG] Station responded%s", snr_phrase);
            appendRxLogLine(buf);
        } else {
            guiLog("MODEM: Detected PING/PONG%s", snr_phrase);
        }
        // diag keeps BOTH the sync-corr confidence and the physical in-band SNR for analysis.
        char diag_fields[160];
        std::snprintf(diag_fields, sizeof(diag_fields),
                      "{\"sync_corr_snr_db\":%.1f,\"inband_snr_db\":%.1f,\"inband_valid\":%s}",
                      snr, phys.valid ? phys.snr_db : -1.0f, phys.valid ? "true" : "false");
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "ping.rx", diag_fields);
        // If narrowband chirp detected, set session-scoped override so negotiateMode() picks OFDM_NARROW
        if (modem_.isNarrowbandDetected()) {
            protocol_.setNarrowbandOverride(protocol::WaveformMode::OFDM_NARROW);
        }
        protocol_.onPingReceived();
    });

    protocol_.setDataModeChangedCallback([this](Modulation mod, CodeRate rate,
                                                 int cw_count,
                                                 float snr_db, float peer_fading,
                                                 int mc_dpsk_num_carriers,
                                                 int mc_dpsk_samples_per_symbol,
                                                 bool snr_is_wire) {
        if (mc_dpsk_num_carriers > 0 && mc_dpsk_samples_per_symbol > 0) {
            modem_.setMCDPSKProfile(mc_dpsk_num_carriers,
                                    mc_dpsk_samples_per_symbol,
                                    mod == Modulation::DBPSK ? 1 :
                                    mod == Modulation::D8PSK ? 3 : 2);
        }
        // BUG-MC-RETRY-SPURIOUS fix 4: the protocol layer just COMMITTED a data
        // mode. If the (mod, rate, cw) tuple actually changed, bump the data-mode
        // generation — any CCA/serialize-deferred DATA-class audio rendered under
        // the old generation is now provably undecodable by the peer (wrong
        // constellation/rate/CW geometry) and purgeStaleDeferredDataTx() drops it
        // before flush; the ARQ re-renders those frames at the new mode on its
        // next refill. Tuple-change gate keeps same-mode re-notifies (e.g. a
        // duplicate-free SNR refresh) from discarding viable deferred audio. The
        // last-seen tuple is only touched here (engine mutex serializes callbacks);
        // the generation itself is atomic (defer/purge read it on the main thread).
        if (!data_mode_gen_seen_valid_ ||
            data_mode_gen_seen_mod_ != mod ||
            data_mode_gen_seen_rate_ != rate ||
            data_mode_gen_seen_cw_ != cw_count) {
            data_mode_gen_seen_valid_ = true;
            data_mode_gen_seen_mod_ = mod;
            data_mode_gen_seen_rate_ = rate;
            data_mode_gen_seen_cw_ = cw_count;
            data_mode_generation_.fetch_add(1, std::memory_order_release);
        }
        // Update modem engine with new data mode
        modem_.setDataMode(mod, rate);
        connected_peer_snr_valid_ = std::isfinite(snr_db) && snr_db >= 0.0f;
        if (connected_peer_snr_valid_) {
            connected_peer_snr_db_ = snr_db;
            stats_.current_snr_db = snr_db;
        }
        stats_.current_modulation = mod;
        stats_.current_code_rate = rate;
        // Sync ModemEngine encoder/decoder to negotiated CW count from the
        // wire. DO NOT call protocol_.setForcedFrameCodewords here — the
        // engine mutex is held while this callback runs and re-entry will
        // deadlock (caught 2026-05-04 in cli_simulator A/B with seed=1).
        modem_.setFixedFrameCodewords(cw_count);

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
        // Source label fix (2026-07-03): the responder's connect-time notify carries
        // its OWN local reading (peer_fading==local_fading was the tell) — only the
        // initiator's CONNECT_ACK and a received MODE_CHANGE are genuinely wire-
        // carried. Keep the line prefix stable (gui_qso_scenario.sh greps
        // "MODE_CHANGE: <waveform> <mod> "); only the parenthesized label varies.
        const char* snr_source_label = snr_is_wire ? "wire_peer" : "local_measured";
        guiLog("MODE_CHANGE: %s %s %s (peer_snr=%.1f dB (%s), peer_fading=%s, local_fading=%.2f %s)",
               wf_name, modulationToString(mod), codeRateToString(rate),
               snr_db, snr_source_label, peer_fading_text,
               local_fading, local_quality);
        char diag_fields[224];
        snprintf(diag_fields, sizeof(diag_fields),
                 "{\"waveform\":\"%s\",\"mod\":\"%s\",\"rate\":\"%s\",\"cw\":%d}",
                 wf_name, modulationToString(mod), codeRateToString(rate), cw_count);
        ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
            "protocol", "waveform.negotiated", diag_fields);

        // Format display with waveform info and channel quality. Prefer the LOCAL
        // live OFDM LTS broadband reading (fresh per decoded frame — what the
        // RX-AUTHORITY verdict actually uses) over the wire byte: the wire value is
        // the PEER's snapshot and on several paths is frozen at the handshake-era
        // connect reading (measured: "13 dB" all run on a dial-20 channel). Label
        // the source so the operator knows which meter they are reading. The wire
        // byte can also carry the -10 dB stale sentinel (no reading fresher than
        // 3*Tc at the sender) — render that as n/a, never as a number.
        char link_snr_text[24];
        // Lock-free LAST-VALID atomics, not the stats-queue snapshot: the queue's
        // has_ofdm flag is per-drained-result and a control decode (the MODE_CHANGE
        // frame itself!) clears it right before this line renders — measured still
        // "(wire)" mid-transfer on the first fix attempt.
        if (modem_.hasLastOFDMBroadbandSNR() &&
            std::isfinite(modem_.getLastOFDMBroadbandSNR())) {
            snprintf(link_snr_text, sizeof(link_snr_text), "%.1f dB (lts)",
                     modem_.getLastOFDMBroadbandSNR());
        } else if (snr_db <=
                   protocol::connection_policy::kConnectSnrStaleSentinelDb + 0.5f) {
            snprintf(link_snr_text, sizeof(link_snr_text), "n/a");
        } else {
            snprintf(link_snr_text, sizeof(link_snr_text), "%d dB (wire)",
                     static_cast<int>(snr_db));
        }
        // Two-SNR display: the link value above is the demod-USABLE (effective)
        // SNR — the right rate-selection input but NOT the channel's S:N (the
        // gap between them is the hardware chain's implementation loss). Show
        // the locally measured physical channel SNR beside it when available.
        // BETA readout (BUG-PHYSICAL-SNR-RIG-REF): power ratio of the connect
        // sequence's training span vs the frame's own inter-chirp noise.
        char channel_snr_text[48] = "";
        {
            float mean_db = 0.0f, spread_db = 0.0f;
            const size_t n = modem_.physicalSnrStats(mean_db, spread_db);
            if (n >= 3) {
                // §5: on fading, the channel is a distribution — report the
                // linear-domain (fade-averaged) mean and the dB spread across
                // the recent frames instead of a single-snapshot flicker.
                snprintf(channel_snr_text, sizeof(channel_snr_text),
                         ", channel %.1f±%.1f dB", mean_db, spread_db);
            } else if (modem_.hasLastPhysicalSnr()) {
                snprintf(channel_snr_text, sizeof(channel_snr_text),
                         ", channel~%.1f dB (1 frame)", modem_.lastPhysicalSnrDb());
            }
        }
        char buf[300];
        if (waveform == protocol::WaveformMode::MC_DPSK) {
            snprintf(buf, sizeof(buf),
                     "[MODE] MC-DPSK 8 carriers %s (usable RX SNR=%s%s, peer fading=%s, local fading=%.2f %s)",
                     codeRateToString(rate), link_snr_text, channel_snr_text,
                     peer_fading_text,
                     local_fading, local_quality);
        } else {
            snprintf(buf, sizeof(buf),
                     "[MODE] %s %s %s (usable RX SNR=%s%s, peer fading=%s, local fading=%.2f %s)",
                     wf_name, modulationToString(mod), codeRateToString(rate),
                     link_snr_text, channel_snr_text, peer_fading_text,
                     local_fading, local_quality);
        }
        appendRxLogLine(buf);

    });
    protocol_.setPhyMaskV1NegotiatedCallback([this](bool enabled) {
        modem_.setCarrierLdpcInterleaver(enabled);
    });

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

    // Connect waveform callback.
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
        // Start timing on first progress (for receiving files) — unless the RX clock
        // was already armed earlier at the first incoming burst (the honest start).
        if (!rx_transfer_clock_armed_ && last_progress_milestone_ == 0 && !p.is_sending &&
            p.transferred_bytes == 0) {
            file_transfer_start_time_ = std::chrono::steady_clock::now();
            rx_transfer_clock_armed_ = true;
            if (p.total_bytes > 0 && p.transferred_bytes == 0) {
                appendRxLogLine("[FILE] Receiving " + p.filename + " (" +
                                std::to_string(p.total_bytes) + " bytes)...");
            }
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

    protocol_.setFileReceivedCallback([this](const std::string& path, bool success,
                                             const std::string& error) {
        // Receive completion means the final ACK has only been queued; it does not prove
        // the sender consumed that ACK. Completion-driven scenario teardown is therefore
        // armed only by the caller's successful FileSent callback below.
        last_progress_milestone_ = 0;  // Reset for next transfer
        rx_transfer_clock_armed_ = false;  // re-arm RX clock on the next incoming burst
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
            char buf[360];
            if (last_goodput_label_ == "RX file" && last_effective_goodput_bps_ > 0.0f) {
                snprintf(buf, sizeof(buf), "[FILE] Received %s (%u bytes, CRC ok, %.1fs, %.2f kbps)",
                         path.c_str(), file_bytes, seconds, last_effective_goodput_bps_ / 1000.0f);
            } else {
                snprintf(buf, sizeof(buf), "[FILE] Received %s (%u bytes, CRC ok, %.1fs)",
                         path.c_str(), file_bytes, seconds);
            }
            msg = buf;
            last_received_file_ = path;
        } else if (error == "Transfer cancelled") {
            beginFileCancelAudioDrain("peer sender cancel");
            msg = "[FILE] Transfer cancelled";
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer",
                "{\"direction\":\"rx\",\"success\":false,\"error\":\"cancelled\"}");
        } else {
            msg = error.empty() ? "[FILE] Receive failed" : "[FILE] Receive failed: " + error;
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer",
                "{\"direction\":\"rx\",\"success\":false,\"error\":\"receive_failed\"}");
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "fault", "fault.triggered",
                "{\"reason\":\"file_receive_failed\",\"policy\":\"emit_only\"}");
        }
        appendRxLogLine(msg);
    });

    protocol_.setFileSentCallback([this](bool success, const std::string& error) {
        // Request only, never call protocol_ from here. This callback runs under
        // ProtocolEngineMutex, and successful sender completion proves the final file ACK
        // was consumed before the scripted caller tears down the QSO.
        if (success && options_.disconnect_on_file_done && scenario_active_) {
            file_done_disconnect_pending_.store(true, std::memory_order_relaxed);
        }
        last_progress_milestone_ = 0;  // Reset for next transfer
        rx_transfer_clock_armed_ = false;  // re-arm RX clock on the next incoming burst
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
        } else if (error == "Transfer cancelled") {
            msg = "[FILE] Transfer cancelled";
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "protocol", "file.transfer",
                "{\"direction\":\"tx\",\"success\":false,\"error\":\"cancelled\"}");
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

    // Set receive directory from settings (defaults to Downloads folder)
    protocol_.setReceiveDirectory(settings_.getReceiveDirectory());

    // Configure waterfall display
    if (waterfall_) {
        waterfall_->setSampleRate(48000.0f);
        waterfall_->setFrequencyRange(0.0f, 3000.0f);
        waterfall_->setDynamicRange(-60.0f, 0.0f);
    }

    // Settings window callbacks
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
            audio_.setOutputGain(1.0f);
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
            audio_.setOutputGain(1.0f);
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

    settings_window_.setPaprReductionChangedCallback([this](bool enabled) {
        modem_.setPaprReductionEnabled(enabled);
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

    // Apply initial expert settings from loaded config
    // FLOOR-PROBE override (test/diag, FORCE bucket): ULTRA_FORCE_WAVEFORM pins the REQUESTED
    // data waveform into the CONNECT so negotiation honors it BELOW the auto entry-SNR — where
    // the ladder would otherwise drop to MC-DPSK. Pairs with ULTRA_FORCE_DATA_MOD/RATE (which
    // only force the mod/rate, not the mode). Both stations read the same env. NOT production.
    uint8_t forced_wf = settings_.forced_waveform;
    if (const char* fw = std::getenv("ULTRA_FORCE_WAVEFORM")) {
        const std::string s(fw);
        if      (s == "OFDM_CHIRP")  forced_wf = static_cast<uint8_t>(protocol::WaveformMode::OFDM_CHIRP);
        else if (s == "OFDM_NARROW") forced_wf = static_cast<uint8_t>(protocol::WaveformMode::OFDM_NARROW);
        else if (s == "MC_DPSK")     forced_wf = static_cast<uint8_t>(protocol::WaveformMode::MC_DPSK);
    }
    protocol_.setPreferredMode(static_cast<protocol::WaveformMode>(forced_wf));
    protocol_.setForcedModulation(static_cast<Modulation>(settings_.forced_modulation));
    protocol_.setForcedCodeRate(static_cast<CodeRate>(settings_.forced_code_rate));
    modem_.setNarrowbandControl(forced_wf == static_cast<uint8_t>(protocol::WaveformMode::OFDM_NARROW));

    // Apply initial filter settings from loaded config
    FilterConfig initial_filter;
    initial_filter.enabled = settings_.filter_enabled;
    initial_filter.center_freq = settings_.filter_center;
    initial_filter.bandwidth = settings_.filter_bandwidth;
    initial_filter.taps = settings_.filter_taps;
    modem_.setFilterConfig(initial_filter);
    modem_.setPaprReductionEnabled(settings_.papr_reduction_enabled);
    audio_.setOutputGain(1.0f);

    if (simulation_enabled_) {
        initOtaAudio();
    }

    // Auto-initialize audio on startup unless safe-startup mode is requested.
    // This avoids crashing on fragile audio stacks during process bring-up.
    if (!options_.safe_startup && !simulation_enabled_) {
        initAudio();
        if (audio_initialized_) {
            deferred_radio_rx_start_pending_ = true;
            uint32_t now_ms = SDL_GetTicks();
            deferred_radio_rx_start_deadline_ms_ = now_ms;
            deferred_radio_rx_start_timeout_ms_ = now_ms + 3000;
            deferred_radio_rx_start_attempts_ = 0;
            guiLog("Startup audio stage 1/2 complete: core ready, starting RX capture ASAP (timeout=3000ms)");
        }
    } else if (!simulation_enabled_) {
        guiLog("Safe startup enabled: deferred audio/simulator initialization");
        deferred_audio_auto_init_pending_ = true;
        deferred_audio_auto_init_deadline_ms_ = SDL_GetTicks() + 300;
        deferred_audio_auto_init_attempts_ = 0;
        deferred_audio_wait_logged_ = false;
    } else {
        guiLog("OTASim mode enabled: SDL audio device initialization skipped");
    }
}

App::~App() {
    releasePtt("app_shutdown");
    closePtt();

    stopOtaAudio();

    drainOperatorEvents(true);
    logOperatorEventStats("shutdown");

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

void App::beginFileCancelAudioDrain(const char* reason) {
    file_cancel_audio_drain_active_ = true;
    file_cancel_audio_drain_until_ =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(kFileCancelAudioDrainMs);
    modem_.clearRxBuffer();
    guiLog("FILE_CANCEL: RX audio drain armed for %ums (%s)",
           kFileCancelAudioDrainMs,
           reason ? reason : "cancel");
}

bool App::isFileCancelAudioDrainActive() {
    if (!file_cancel_audio_drain_active_) {
        return false;
    }

    if (std::chrono::steady_clock::now() < file_cancel_audio_drain_until_) {
        return true;
    }

    file_cancel_audio_drain_active_ = false;
    modem_.clearRxBuffer();
    modem_.expectFullOFDMAnchorOnce();
    guiLog("FILE_CANCEL: RX audio drain complete; receiver resynchronized");
    return false;
}

void App::cancelActiveFileTransfer() {
    bool receiving_file = false;
    if (protocol_.isFileTransferInProgress()) {
        const auto progress = protocol_.getFileProgress();
        receiving_file = !progress.is_sending;
    }
    if (receiving_file) {
        beginFileCancelAudioDrain("local receiver cancel");
    }
    protocol_.cancelFileTransfer();
}

void App::pollOtaRx() {
    if (!ota_audio_) {
        return;
    }

    // Shared OTASim RX drain — the SAME discipline the headless ultra_tnc uses
    // (ultra::otasim_client::drainOtaRx): drain only real samples, never fabricate
    // filler. GUI-specific per-chunk work (record, waterfall, monitor) lives in the
    // consumer below; the drain loop itself is shared so it cannot drift.
    ultra::otasim_client::drainOtaRx(*ota_audio_, [this](const std::vector<float>& samples) {
        if (recording_enabled_) {
            recorded_rx_samples_.insert(recorded_rx_samples_.end(), samples.begin(), samples.end());
        }
        if (isFileCancelAudioDrainActive()) {
            return;  // drain the medium but don't feed the modem during a cancel
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
    });
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
    guiLog("initAudio: enumerated %zu input device(s), %zu output device(s)",
           input_devices_.size(), output_devices_.size());

    audio_initialized_ = true;
}

void App::enqueueOperatorEvent(OperatorEvent event) {
    const bool critical = event.kind != OperatorEventKind::LogLine;
    std::unique_lock<std::mutex> lock(operator_event_mutex_, std::defer_lock);
    if (critical) {
        // Protocol callbacks are emitted after ProtocolEngine releases its mutex.
        // A received application message or terminal TX result can therefore wait
        // briefly for the GUI queue without deadlocking the modem. Never lose an
        // ACKed message merely because the render thread was draining events.
        lock.lock();
    } else {
        lock.try_lock();
        if (!lock.owns_lock()) {
            operator_events_dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }

    if (operator_event_queue_.size() >= operator_event_queue_limit_) {
        const auto disposable = std::find_if(
            operator_event_queue_.begin(), operator_event_queue_.end(),
            [](const OperatorEvent& queued) {
                return queued.kind == OperatorEventKind::LogLine;
            });
        if (disposable != operator_event_queue_.end()) {
            operator_event_queue_.erase(disposable);
            operator_events_dropped_.fetch_add(1, std::memory_order_relaxed);
        } else if (!critical) {
            // A diagnostic line must never evict a message/status event. Critical
            // bursts are intentionally allowed to exceed the soft display limit
            // until the next GUI drain; their volume is protocol-bounded.
            operator_events_dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
    }
    operator_event_queue_.push_back(std::move(event));
}

void App::enqueueOperatorLogLine(std::string line) {
    operator_events_log_lines_.fetch_add(1, std::memory_order_relaxed);
    OperatorEvent event;
    event.kind = OperatorEventKind::LogLine;
    event.line = std::move(line);
    enqueueOperatorEvent(std::move(event));
}

void App::enqueueMessageReceived(std::string from, std::string text) {
    operator_events_rx_messages_.fetch_add(1, std::memory_order_relaxed);
    OperatorEvent event;
    event.kind = OperatorEventKind::MessageReceived;
    event.from = std::move(from);
    event.text = std::move(text);
    enqueueOperatorEvent(std::move(event));
}

void App::enqueueMessageTxStatus(protocol::ProtocolEngine::MessageTxStatusEvent event_status) {
    switch (event_status.status) {
        case protocol::ProtocolEngine::MessageTxStatus::SUBMITTED:
            operator_events_tx_submitted_.fetch_add(1, std::memory_order_relaxed);
            break;
        case protocol::ProtocolEngine::MessageTxStatus::DELIVERED:
            operator_events_tx_delivered_.fetch_add(1, std::memory_order_relaxed);
            if (!options_.auto_send_message.empty() &&
                event_status.text.rfind(options_.auto_send_message, 0) == 0) {
                scenario_messages_delivered_.fetch_add(1, std::memory_order_relaxed);
            }
            if (!options_.auto_reply_message.empty() &&
                event_status.text == options_.auto_reply_message) {
                scenario_reply_delivered_.store(true, std::memory_order_relaxed);
            }
            break;
        case protocol::ProtocolEngine::MessageTxStatus::FAILED:
            operator_events_tx_failed_.fetch_add(1, std::memory_order_relaxed);
            break;
    }
    OperatorEvent event;
    event.kind = OperatorEventKind::MessageTxStatus;
    event.tx_status = std::move(event_status);
    enqueueOperatorEvent(std::move(event));
}

void App::appendRxLogLine(const std::string& msg) {
    enqueueOperatorLogLine(msg);
}

void App::appendRxLogLineNow(const std::string& msg) {
    const std::string line = timestampOperatorLine(msg);
    {
        std::lock_guard<std::mutex> lock(rx_log_mutex_);
        rx_log_.push_back(line);
        while (rx_log_.size() > MAX_RX_LOG) {
            rx_log_.pop_front();
        }
    }
    // Mirror every operator-visible Message Log line into the GUI log file so
    // the log file is a strict superset of the on-screen message box. Without
    // this, lines like [MESSAGE], [FILE], [RX ...] and status notifications
    // appeared only in the UI and were never written to disk.
    if (!operator_log_file_suppressed_) {
        guiLog("%s", line.c_str());
    }
}

void App::drainOperatorEvents(bool flush_all) {
    const size_t drain_limit = flush_all
        ? static_cast<size_t>(-1)
        : operator_event_drain_limit_;
    if (drain_limit == 0) {
        return;
    }

    std::deque<OperatorEvent> batch;
    {
        std::lock_guard<std::mutex> lock(operator_event_mutex_);
        const size_t count = std::min(drain_limit, operator_event_queue_.size());
        for (size_t i = 0; i < count; ++i) {
            batch.push_back(std::move(operator_event_queue_.front()));
            operator_event_queue_.pop_front();
        }
    }

    for (const auto& event : batch) {
        switch (event.kind) {
            case OperatorEventKind::LogLine:
                appendRxLogLineNow(event.line);
                break;
            case OperatorEventKind::MessageReceived:
                appendRxLogLineNow("[RX " + event.from + "] " + event.text);
                // Scripted bidirectional test: a responder configured with
                // --auto-reply-message sends ONE reply after the configured number
                // of application messages. The default threshold remains one.
                // Submission is retried from tickScenario() until Connection accepts it;
                // only then is scenario_reply_sent_ latched. This also exercises the
                // half-duplex turn reversal (the peer must yield TURNOVER).
                if (scenario_active_ && !options_.auto_reply_message.empty() &&
                    !scenario_reply_sent_) {
                    ++scenario_reply_inbound_received_;
                    const int expected = std::max(1, options_.auto_reply_after_messages);
                    guiLog("[scenario] auto-reply inbound %d/%d from %s",
                           scenario_reply_inbound_received_, expected,
                           event.from.empty() ? "peer" : event.from.c_str());
                    if (scenario_reply_inbound_received_ == expected &&
                        !scenario_reply_pending_) {
                        scenario_reply_pending_ = true;
                        guiLog("[scenario] auto-reply threshold reached (%d/%d); scheduling %zu-byte reply",
                               scenario_reply_inbound_received_, expected,
                               options_.auto_reply_message.size());
                    } else if (scenario_reply_inbound_received_ > expected) {
                        guiLog("[scenario] auto-reply received unexpected extra inbound message (%d > %d) before reply submission",
                               scenario_reply_inbound_received_, expected);
                    }
                }
                break;
            case OperatorEventKind::MessageTxStatus:
                switch (event.tx_status.status) {
                    case protocol::ProtocolEngine::MessageTxStatus::SUBMITTED:
                        appendRxLogLineNow("TX #" + std::to_string(event.tx_status.first_seq) +
                                           " -> " +
                                           (event.tx_status.remote_call.empty()
                                                ? std::string("peer")
                                                : event.tx_status.remote_call) +
                                           ": " + event.tx_status.text);
                        break;
                    case protocol::ProtocolEngine::MessageTxStatus::DELIVERED: {
                        char line[96];
                        snprintf(line, sizeof(line), "OK #%u delivered (%.1fs)",
                                 event.tx_status.first_seq,
                                 event.tx_status.elapsed_ms / 1000.0);
                        appendRxLogLineNow(line);
                        break;
                    }
                    case protocol::ProtocolEngine::MessageTxStatus::FAILED: {
                        if (event.tx_status.sequence_valid) {
                            appendRxLogLineNow(
                                "FAIL #" + std::to_string(event.tx_status.first_seq) +
                                " failed");
                        } else {
                            appendRxLogLineNow(
                                "FAIL before wire submission");
                        }
                        break;
                    }
                }
                break;
        }

        if (operator_log_slow_ms_ > 0 && !flush_all) {
            std::this_thread::sleep_for(std::chrono::milliseconds(operator_log_slow_ms_));
        }
    }
}

void App::logOperatorEventStats(const char* reason) {
    uint64_t queued = 0;
    {
        std::lock_guard<std::mutex> lock(operator_event_mutex_);
        queued = operator_event_queue_.size();
    }
    guiLog("OPERATOR_EVENT_STATS reason=%s log_lines=%llu rx_messages=%llu tx_submitted=%llu "
           "tx_delivered=%llu tx_failed=%llu dropped=%llu queued=%llu suppress_file=%d "
           "slow_ms=%u drain_limit=%zu queue_limit=%zu max=%zu",
           reason ? reason : "n/a",
           static_cast<unsigned long long>(operator_events_log_lines_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(operator_events_rx_messages_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(operator_events_tx_submitted_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(operator_events_tx_delivered_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(operator_events_tx_failed_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(operator_events_dropped_.load(std::memory_order_relaxed)),
           static_cast<unsigned long long>(queued),
           operator_log_file_suppressed_ ? 1 : 0,
           operator_log_slow_ms_,
           operator_event_drain_limit_,
           operator_event_queue_limit_,
           MAX_OPERATOR_EVENTS);
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
    const size_t text_len = boundedCStringLen(tx_text_buffer_);
    const bool scenario_drives_payload =
        !options_.auto_send_file.empty() ||
        !options_.auto_send_message.empty() ||
        !options_.auto_reply_message.empty() ||
        options_.auto_cancel_file_after_sec > 0;
    if (scenario_drives_payload || text_len == 0 || !protocol_.isConnected() ||
        protocol_.isFileTransferInProgress() || tx_in_progress_.load() ||
        protocol_.getTxBacklogBytes() != 0) {
        return;
    }

    const bool ready_now = protocol_.isReadyToSend();
    const std::string text(tx_text_buffer_, text_len);
    if (protocol_.sendMessage(text)) {
        if (!ready_now) {
            appendRxLogLine("[INFO] Message queued - waiting for DATA turn.");
        }
        // Preserve the operator's draft when submission is rejected. Clear it
        // only after Connection has accepted ownership of the message.
        tx_text_buffer_[0] = '\0';
    }
}

void App::onDataReceived(const std::string& text) {
    if (!text.empty()) {
        appendRxLogLine("[RX] " + text);
    }
}

void App::render() {
    static bool first_render = true;
    render_frames_seen_++;
    if (first_render) {
    }
    // TX drive is embedded per burst; keep AudioEngine output at unity.
    if (first_render) {
    }
    audio_.setOutputGain(1.0f);
    if (first_render) {
    }

    // Process captured RX audio in the main thread.
    pollRadioRx();

    // Carrier-sense TX gate: pollRadioRx() just refreshed the channel-busy detector,
    // so this is the right point to release any OFDM burst we deferred to avoid
    // transmitting over the peer (half-duplex collision avoidance).
    flushDeferredTxIfReady();

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
                initAudio();
                if (audio_initialized_) {
                    deferred_radio_rx_start_pending_ = true;
                    deferred_radio_rx_start_deadline_ms_ = now_ms;
                    deferred_radio_rx_start_timeout_ms_ = now_ms + 3000;
                    deferred_radio_rx_start_attempts_ = 0;
                    guiLog("Deferred audio stage 1/2 complete: core ready, starting RX capture ASAP (timeout=3000ms)");
                    deferred_audio_auto_init_pending_ = false;
                } else {
                    deferred_audio_auto_init_attempts_++;
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

    // #70 STAGE2 re-arm: if we PONGed but the initiator's connect attempt elapsed with no
    // CONNECT (still DISCONNECTED), go back to expecting a bare-chirp PING so future PINGs are
    // still answered (a spurious PONG must not leave us permanently deaf). A successful
    // CONNECT clears the deadline via the state callback (CONNECTED != DISCONNECTED).
    {
        const uint32_t deadline =
            responder_connect_expected_until_ms_.load(std::memory_order_relaxed);
        if (deadline != 0 && static_cast<int32_t>(now - deadline) >= 0) {
            responder_connect_expected_until_ms_.store(0, std::memory_order_relaxed);
            if (conn_state_cached_.load(std::memory_order_relaxed) ==
                protocol::ConnectionState::DISCONNECTED) {
                modem_.setBareChirpExpected(true);
            }
        }
    }

    // Scenario scripting: drive the real connect/accept/send actions.
    tickScenario();

    // ACK repeat-if-silent (handoff §7.6): fire the decorrelated tone-ACK copy
    // if its window elapsed AND the channel is quiet (see the stash site).
    maybeFireAckRepeatIfSilent();
    maybeFireDeferredAck();

    // Drain bounded modem/protocol UI events on the GUI thread after protocol
    // work for this frame. Formatting and log-file flushes happen here, never
    // on the RX decode / ARQ callback path.
    drainOperatorEvents();

    // Create main window
    if (first_render) {
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
        // Draw against the modulation of the symbols actually captured (the RX
        // buffer is reset on modulation change, so it is homogeneous), not the
        // configured data mode — e.g. a file SENDER's RX shows the DQPSK ACKs it
        // receives, not its own 16QAM TX. Fall back to the configured mode when idle.
        Modulation disp_mod = symbols.empty() ? config_.modulation
                                              : modem_.getConstellationModulation();
        constellation_.render(symbols, disp_mod);
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
    const bool status_connected =
        protocol_.getState() == protocol::ConnectionState::CONNECTED;
    const auto status_snr = updateSnrBallistics(
        selectOperatorSNRDisplay(mstats, status_connected), status_connected);
    char status_snr_text[80];
    if (status_snr.valid) {
        // The measured value is EFFECTIVE (fade-state) SNR — physically honest but
        // it under-reads the operator's AWGN-equivalent dial on a fading channel
        // (Watterson Good at dial 20 reads ~11-15 effective; the calibrated
        // selection basis maps between the two). Show the dial-equivalent
        // alongside so the meter matches the knob the operator actually set.
        const float fading_now = modem_.getFadingIndex();
        if (fading_now >= protocol::kFadingAwgnMax) {
            // dialEquivalentSnrDb = the SAME helper the entry selection uses (flat
            // basis by default; the calibrated affine map under
            // ULTRA_CONNECT_AFFINE_BASIS) — one source of truth for reading->dial.
            snprintf(status_snr_text, sizeof(status_snr_text),
                     "%.1f dB eff (~%.0f dial, %s)", status_snr.snr_db,
                     protocol::connection_policy::dialEquivalentSnrDb(
                         status_snr.snr_db, fading_now,
                         protocol::connection_policy::connectAffineBasisEnabled()),
                     snrSourceToString(status_snr.source));
        } else {
            snprintf(status_snr_text, sizeof(status_snr_text), "%.1f dB (%s)",
                     status_snr.snr_db, snrSourceToString(status_snr.source));
        }
    } else if (connected_peer_snr_valid_) {
        snprintf(status_snr_text, sizeof(status_snr_text), "%.1f dB (wire_peer)",
                 connected_peer_snr_db_);
    } else {
        snprintf(status_snr_text, sizeof(status_snr_text), "-- dB (none)");
    }
    ImGui::Text("Mode: %s | SNR: %s | TX: %d | RX: %d | PHY: %d bps | Goodput: %s | RXQ: %.0f ms (pk %.0f) | OF: %llu/%llu | %s%s",
                mode_str, status_snr_text, mstats.frames_sent, mstats.frames_received,
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

// ACK repeat-if-silent (ULTRA_ACK_REPEAT_SILENT_MS; handoff §7.6, BUG-TONE-FADE
// residual): fire the stashed decorrelated tone-ACK copy when its window elapsed,
// ONLY if the channel is quiet. CCA-busy = the peer's next burst is arriving =
// copy 1 was heard = repeat both unnecessary AND dangerous (our TX would blank the
// incoming anchor). CCA-quiet = copy 1 likely died on air = the repeat rides a
// fade draw decorrelated by >= the knob delay. Runs on the GUI tick; never touches
// protocol_ (the §15.5 deadlock rule).
void App::maybeFireAckRepeatIfSilent() {
    if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
        clearPendingAckAudioForTeardown();
        return;
    }
    // CONTINUOUS-QUIET gate (F100 correction): a single CCA sample at the deadline
    // races the rig's 1.5-3 s turnaround — a momentarily-quiet instant fired 25
    // redundant repeats in F100 and their TX blanked inbound burst heads (13
    // craters). Sample the channel EVERY tick while pending: ANY busy reading
    // between the ack and the deadline means something is (or was) inbound —
    // copy 1 was heard — CANCEL. Only an unbroken silent window fires (a waiting
    // sender is the only source of multi-second silence mid-transfer; an inbound
    // 8-10 s burst cannot hide from continuous sampling).
    {
        std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
        if (!ack_repeat_pending_) return;
        if (conn_state_cached_.load(std::memory_order_relaxed) !=
            protocol::ConnectionState::CONNECTED) {
            ack_repeat_pending_ = false;  // session ended — moot
            return;
        }
        if (modem_.channelBusyForTx()) {
            ack_repeat_pending_ = false;
            guiLog("ACK-REPEAT-SILENT: canceled (channel activity in window — "
                   "copy 1 evidently heard)");
            return;
        }
        // F176 GEOMETRIC gate: declared burst airtime still arriving — hold the
        // repeat (do NOT cancel: an in-flight inbound burst says nothing about
        // whether ack copy 1 was heard; substantive evidence below decides).
        if (modem_.burstAirSamplesRemaining() > 0) {
            return;
        }
        // F227 MC-DPSK HOLD: only OFDM descriptor bursts publish an air-end, so
        // an inbound MC-DPSK burst is invisible to the gate above — and CCA
        // can't see it either on a quiet chain (burst RMS below the learned
        // threshold). A broad RX-signal stamp (chirp/LTS sync) within the last
        // frame-scale window means a frame is likely mid-air: HOLD, don't
        // cancel (F147: idle-noise false locks must not kill the repeat — a
        // hold only delays it past the frame's geometry; the substantive gate
        // below still decides the cancel). F227 measured: repeat fired 1.3 s
        // after the peer's burst SYNC, wiped frame 1, cost a 30.9 s RTO every
        // other cycle (~3x MC-DPSK goodput).
        {
            const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const int64_t sig_ms = modem_.lastRxSignalMs();
            if (protocol::connection_policy::shouldHoldSilentAckRepeatForBroadSignal(
                    modem_.getWaveformMode(), now_ms, sig_ms,
                    ack_repeat_armed_signal_ms_)) {
                return;
            }
        }
        // F129 DECODER-EVIDENCE cancel, F143-corrected, F147-corrected again:
        // count only SUBSTANTIVE evidence (accepted sync / consumed descriptor /
        // decoded codewords) NEWER than the arm. The F143 broad-stamp version
        // still died in the field: false-chirp-lock rejects on IDLE NOISE stamp
        // the broad signal (they are decode attempts), so the repeat armed to
        // save a one-way-faded ack canceled on "inbound transmission" with
        // nothing on the air (F147: sender went RTO-deaf for 40 s). A NEW
        // substantive decode after the arm = the peer really moved on.
        if (modem_.lastRxSubstantiveMs() > ack_repeat_armed_rx_ms_ + 500) {
            ack_repeat_pending_ = false;
            guiLog("ACK-REPEAT-SILENT: canceled (decoder RX evidence — "
                   "inbound transmission in progress)");
            return;
        }
        if (std::chrono::steady_clock::now() < ack_repeat_fire_time_) return;
    }
    std::vector<float> copy;
    {
        std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
        if (!ack_repeat_pending_) return;
        ack_repeat_pending_ = false;
        copy.swap(ack_repeat_samples_);
    }
    guiLog("ACK-REPEAT-SILENT: firing (unbroken quiet window — copy 1 likely lost)");
    if (queueRealTxSamples(copy, "TX tone-burst ACK repeat (silent-gated)",
                           /*in_qso_data=*/false)) {
        LOG_MODEM(INFO,
                  "TX-AUDIO-COMMIT: tone-ack source=silent-repeat samples=%zu",
                  copy.size());
    }
}

// LISTEN-BEFORE-ACK submission tail: stash the repeat-if-silent copy (timed from
// the ACTUAL send, not the protocol event) and queue the audio. Factored so the
// immediate path and the CCA-deferred path share it exactly.
void App::submitToneAckSamples(const std::vector<float>& samples) {
    if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
        clearPendingAckAudioForTeardown();
        return;
    }
    // ACK REPEAT-IF-SILENT (ULTRA_ACK_REPEAT_SILENT_MS, default OFF;
    // BUG-TONE-FADE residual, handoff §7.6): stash a decorrelated copy; the GUI
    // tick fires it only after an UNBROKEN quiet window (see
    // maybeFireAckRepeatIfSilent).
    static const uint32_t kAckRepeatSilentMs = [] {
        return protocol::connection_policy::silentAckRepeatDelayMs(
            std::getenv("ULTRA_ACK_REPEAT_SILENT_MS"));
    }();
    if (kAckRepeatSilentMs > 0) {
        const auto airtime_ms = static_cast<int64_t>(samples.size()) * 1000 / 48000;
        std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
        // F222 (MPG@10): one repeat per DISTINCT ack. Backstop re-confirms of
        // the same window state arrive every ~15 s while the sender's
        // undecodable bursts are invisible to every sensor — arming a repeat
        // for each doubled the blind key-ups (ack+repeat pairs marching
        // through the peer's resends). An identical ack already has the RTO
        // and the backstop as its fallback; only NEW window state earns the
        // tone-fade echo.
        const bool distinct_ack = samples != ack_repeat_last_armed_samples_;
        const int64_t broad_baseline_ms = modem_.lastRxSignalMs();
        const int64_t substantive_baseline_ms = modem_.lastRxSubstantiveMs();
        LOG_MODEM(INFO,
                  "ACK-REPEAT-SILENT: arm eligible=%d delay=%u ms airtime=%lld ms "
                  "broad_baseline=%lld substantive_baseline=%lld",
                  distinct_ack ? 1 : 0, kAckRepeatSilentMs,
                  static_cast<long long>(airtime_ms),
                  static_cast<long long>(broad_baseline_ms),
                  static_cast<long long>(substantive_baseline_ms));
        if (distinct_ack) {
            ack_repeat_last_armed_samples_ = samples;
            ack_repeat_samples_ = samples;
            ack_repeat_fire_time_ = std::chrono::steady_clock::now() +
                                    std::chrono::milliseconds(
                                        airtime_ms + kAckRepeatSilentMs);
            ack_repeat_armed_rx_ms_ = substantive_baseline_ms;  // F143/F147 baseline
            ack_repeat_armed_signal_ms_ = broad_baseline_ms;
            ack_repeat_pending_ = true;
        }
    }
    if (queueRealTxSamples(samples, "TX tone-burst ACK audio",
                           /*in_qso_data=*/false)) {
        // transmitToneBurstAck() logs when audio is rendered.  This marker is
        // later and authoritative: listen-before-ACK may defer, supersede, or
        // drop a rendered candidate before it reaches the physical TX queue.
        LOG_MODEM(INFO, "TX-AUDIO-COMMIT: tone-ack source=primary samples=%zu",
                  samples.size());
    }
}

// LISTEN-BEFORE-ACK tick poll (F124): a deferred ACK sends after confirmed quiet.
// At the deadline, energy-only busy is allowed through (protecting against a
// mislearned CCA floor), but decoder/geometry evidence drops the stale ACK rather
// than violating half duplex.  Descriptor-loss asynchronous ACKs retain a full
// maximum-burst guard; physically-complete group ACKs bypass that RX stamp.
void App::maybeFireDeferredAck() {
    if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
        clearPendingAckAudioForTeardown();
        return;
    }
    std::vector<float> copy;
    std::vector<float> deadline_send;
    {
        std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
        if (!ack_defer_pending_) return;
        if (conn_state_cached_.load(std::memory_order_relaxed) !=
            protocol::ConnectionState::CONNECTED) {
            ack_defer_pending_ = false;  // session ended — moot
            ack_defer_samples_.clear();
            ack_defer_inbound_group_complete_ = false;
            return;
        }
        const int64_t rx_hold_ms =
            protocol::connection_policy::deferredToneAckRxHoldMs(
                ack_defer_inbound_group_complete_);
        const bool rx_evidence =
            rx_hold_ms > 0 && modem_.rxSignalActive(rx_hold_ms);
        const bool air_arriving = modem_.burstAirSamplesRemaining() > 0;  // F176
        if (modem_.channelBusyForTx() || rx_evidence || air_arriving) {
            ack_defer_quiet_ticks_ = 0;
            // DEADLINE = DROP, never key over (F127, waterfall-proven: the 2.5 s
            // deadline is shorter than an 8-10 s burst, so 'send regardless' keyed
            // our tones over the sender's RESEND — blanking the very frames that
            // would have ended the exchange, manufacturing craters at 24 dB). If
            // the channel stayed busy the whole window, the sender has moved on
            // and is transmitting; this ACK is stale. Dropping is plain ack loss
            // — the RTO/dup-ack machinery covers it. Half-duplex rule, absolute:
            // WE DO NOT TRANSMIT WHILE SIGNAL IS ARRIVING.
            if (std::chrono::steady_clock::now() >= ack_defer_deadline_) {
                // F129: DROP only on DECODER evidence of a real inbound signal.
                // Energy-only "busy" with no sync/frame evidence is the broken
                // adaptive threshold reading ambient noise (both F129 drops:
                // sender verifiably silent, floor had latched below the noise
                // floor) — SEND in that case; dropping cost two full RTO cycles
                // (~19 s each).
                ack_defer_pending_ = false;
                ack_defer_inbound_group_complete_ = false;
                if (rx_evidence || air_arriving) {
                    ack_defer_samples_.clear();
                    guiLog("LISTEN-BEFORE-ACK: inbound signal (decoder evidence/"
                           "declared airtime) through the defer window — ACK "
                           "DROPPED (RTO covers)");
                } else {
                    deadline_send.swap(ack_defer_samples_);  // sent after unlock
                }
            }
            goto after_lock;
        }
        // QUIET CONFIRMATION (F127): one quiet sample races fade nulls inside an
        // incoming burst — require 3 consecutive quiet ticks (~50-100 ms) before
        // keying, mirroring the repeat path's continuous-quiet doctrine.
        if (++ack_defer_quiet_ticks_ < 3) return;
        ack_defer_pending_ = false;
        ack_defer_quiet_ticks_ = 0;
        ack_defer_inbound_group_complete_ = false;
        copy.swap(ack_defer_samples_);
    }
    guiLog("LISTEN-BEFORE-ACK: channel clear (confirmed) — sending deferred ACK");
    submitToneAckSamples(copy);
    return;
after_lock:
    if (!deadline_send.empty()) {
        guiLog("LISTEN-BEFORE-ACK: deadline with energy-only busy "
               "(no decoder evidence = ambient noise) — sending ACK");
        submitToneAckSamples(deadline_send);
    }
}

void App::clearPendingAckAudioForTeardown() {
    std::lock_guard<std::mutex> lk(ack_repeat_mutex_);
    ack_repeat_pending_ = false;
    ack_repeat_samples_.clear();
    ack_repeat_last_armed_samples_.clear();
    ack_repeat_fire_time_ = {};
    ack_repeat_armed_rx_ms_ = 0;
    ack_repeat_armed_signal_ms_ = 0;
    ack_defer_pending_ = false;
    ack_defer_samples_.clear();
    ack_defer_deadline_ = {};
    ack_defer_quiet_ticks_ = 0;
    ack_defer_inbound_group_complete_ = false;
}

void App::setDisconnectTeardownActive(bool active) {
    const bool previous = disconnect_teardown_active_.exchange(
        active, std::memory_order_acq_rel);
    modem_.setControlOnlyReceive(active);
    if (active) {
        // These are App-owned secondary ACK paths, outside Connection's central
        // egress gate. They must not leak into the responder grace or collide
        // with the peer's close retry.
        clearPendingAckAudioForTeardown();
    }
    if (previous != active) {
        guiLog("Disconnect teardown phase: %s (control-only RX%s)",
               active ? "entered" : "left",
               active ? ", non-close App TX suppressed" : " disabled");
    }
}

void App::tickScenario() {
    if (!scenario_active_) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!scenario_started_) {
        scenario_start_ = now;
        scenario_started_ = true;
    }

    if (scenario_disconnect_complete_pending_.exchange(
            false, std::memory_order_relaxed)) {
        const bool remote_owned_disconnect = !scenario_disconnect_issued_;
        scenario_disconnect_issued_ = true;
        scenario_disconnect_completed_ = true;
        scenario_disconnect_completed_at_ = now;
        guiLog(remote_owned_disconnect
                   ? "[scenario] remote disconnect complete; quitting after log grace"
                   : "[scenario] local disconnect handshake complete; quitting after log grace");
    }

    if (scenario_terminal_failure_ &&
        now - scenario_terminal_failure_at_ >= std::chrono::seconds(2)) {
        guiLog("[scenario] terminal failure grace complete; quitting");
        scenario_active_ = false;
        SDL_Event quit_event;
        quit_event.type = SDL_QUIT;
        SDL_PushEvent(&quit_event);
        return;
    }
    // --disconnect-on-file-done: consume the request HERE, outside ProtocolEngineMutex. The
    // successful FileSent callback only sets the flag; calling protocol_ from inside it
    // self-deadlocks (freeze whose only symptom is endless RX-buffer-overrun spam).
    if (file_done_disconnect_pending_.exchange(false, std::memory_order_relaxed)) {
        // Both scripted endpoints may carry the option for defense in depth, but only
        // the QSO caller owns teardown. This also prevents crossed closes in a future
        // bidirectional script where both endpoints complete outbound files together.
        if (!protocol_.isInitiator()) {
            guiLog("[scenario] file transfer finished; caller owns disconnect, waiting");
        } else if (!scenario_disconnect_issued_ &&
                   protocol_.getState() == protocol::ConnectionState::CONNECTED) {
            guiLog("[scenario] file transfer finished; disconnecting now "
                   "(--disconnect-on-file-done)");
            // Publish ownership/timing before disconnect() so synchronous state
            // callbacks and duplicate completion callbacks observe an issued close.
            scenario_disconnect_issued_ = true;
            scenario_disconnect_at_ = now;
            protocol_.disconnect();
        }
    }

    // Hard exit timer so a scripted run self-terminates and logs can be collected.
    if (options_.exit_after_sec > 0 &&
        now - scenario_start_ >= std::chrono::seconds(options_.exit_after_sec)) {
        guiLog("[scenario] exit-after %ds elapsed; quitting", options_.exit_after_sec);
        scenario_active_ = false;
        SDL_Event quit_event;
        quit_event.type = SDL_QUIT;
        SDL_PushEvent(&quit_event);
        return;
    }

    // Event-driven quit starts only after the connection state machine confirms
    // teardown.  Keep a short post-completion grace so final protocol/diagnostic
    // records are flushed; --exit-after remains the backstop for a stuck close.
    const auto disconnect_complete_elapsed_ms =
        scenario_disconnect_completed_
            ? std::chrono::duration_cast<std::chrono::milliseconds>(
                  now - scenario_disconnect_completed_at_).count()
            : 0;
    if (protocol::connection_policy::scriptedDisconnectQuitReady(
            scenario_disconnect_completed_,
            static_cast<uint64_t>(disconnect_complete_elapsed_ms))) {
        guiLog("[scenario] scripted disconnect complete; quitting");
        scenario_active_ = false;
        SDL_Event quit_event;
        quit_event.type = SDL_QUIT;
        SDL_PushEvent(&quit_event);
        return;
    }

    // Auto-accept an incoming call (drives the same path as the Accept button).
    if (options_.auto_accept) {
        std::string pending;
        {
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending = pending_incoming_call_;
        }
        if (!pending.empty()) {
            guiLog("[scenario] auto-accepting call from %s", pending.c_str());
            protocol_.acceptCall();
            std::lock_guard<std::mutex> lock(rx_log_mutex_);
            pending_incoming_call_.clear();
        }
    }

    const protocol::ConnectionState state = protocol_.getState();
    const bool ota_ready =
        !simulation_enabled_ || (ota_audio_ && ota_audio_->isConnected());

    // Initiate the connection once the link is up and the optional startup
    // delay has elapsed (gives the receiver + the carrier-sense detector time
    // to settle before we probe).
    const bool connect_delay_elapsed =
        options_.connect_delay_sec <= 0 ||
        now - scenario_start_ >= std::chrono::seconds(options_.connect_delay_sec);
    if (!options_.auto_connect.empty() && !scenario_connect_issued_ && ota_ready &&
        connect_delay_elapsed && state == protocol::ConnectionState::DISCONNECTED) {
        guiLog("[scenario] auto-connecting to %s", options_.auto_connect.c_str());
        protocol_.connect(options_.auto_connect);
        scenario_connect_issued_ = true;
    }

    // Once CONNECTED, run the scripted payload sequence (each phase fires once):
    //   1. send the chat message (if any),
    //   2. once the message TX has drained, start the file transfer (if any),
    //   3. mark the sequence dispatched so the disconnect phase can arm.
    // message and file are no longer mutually exclusive — this exercises the
    // real operator flow of chatting and then sending a file on one link.
    const int message_target = std::max(1, options_.auto_message_count);
    if (state == protocol::ConnectionState::CONNECTED) {
        if (!scenario_connected_seen_) {
            scenario_connected_seen_ = true;
            scenario_connected_first_at_ = now;
        }

        const bool message_after_file =
            options_.auto_message_after_file && !options_.auto_send_file.empty();

        if (scenario_file_started_ && !scenario_file_done_seen_ &&
            !protocol_.isFileTransferInProgress()) {
            scenario_file_done_seen_ = true;
            scenario_file_done_at_ = now;
            guiLog("[scenario] file phase cleared; auto-message-after-file may proceed");
        }

        // A received message arms one scripted reply. Connection may temporarily
        // reject submission (for example while its outbound queue is full), so do
        // not latch completion until it accepts the payload; retry on later ticks.
        if (scenario_reply_pending_ && !scenario_reply_sent_) {
            const bool ready_now = protocol_.isReadyToSend();
            if (protocol_.sendMessage(options_.auto_reply_message)) {
                scenario_reply_pending_ = false;
                scenario_reply_sent_ = true;
                guiLog("[scenario] auto-reply accepted (%zu bytes)",
                       options_.auto_reply_message.size());
                if (!ready_now) {
                    appendRxLogLine("[INFO] Auto-reply queued - waiting for DATA turn.");
                }
            }
        }

        // Phase 1: chat message(s) — send the first on connect, then optional
        // repeats at a fixed interval (a test harness for sequential interactive
        // messaging). Each repeat is numbered so RX-side delivery is unambiguous.
        if (!options_.auto_send_message.empty() &&
            scenario_messages_sent_ < message_target &&
            (!message_after_file || scenario_file_done_seen_)) {
            const auto message_start_anchor =
                message_after_file ? scenario_file_done_at_ : scenario_connected_first_at_;
            const bool start_delay_elapsed =
                options_.auto_message_start_delay_sec <= 0 ||
                now - message_start_anchor >=
                    std::chrono::seconds(options_.auto_message_start_delay_sec);
            const bool first = !scenario_message_sent_ && start_delay_elapsed;
            const bool interval_elapsed =
                scenario_message_sent_ && !tx_in_progress_ &&
                now - scenario_message_sent_at_ >=
                    std::chrono::seconds(std::max(1, options_.auto_message_interval_sec));
            if (first || interval_elapsed) {
                std::string msg;
                if (options_.auto_message_vary_len) {
                    // Randomize length per message (mix short + long) to exercise
                    // varied frame counts/timing when hunting intermittent failures.
                    static std::mt19937 rng{std::random_device{}()};
                    static const std::string pool =
                        "ProjectUltra HF modem reliability lorem ipsum dolor sit amet "
                        "consectetur adipiscing elit sed do eiusmod tempor incididunt ut "
                        "labore et dolore magna aliqua ut enim ad minim veniam quis nostrud "
                        "exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat "
                        "duis aute irure dolor in reprehenderit in voluptate velit esse cillum";
                    // Span really-short (a few chars, single tiny frame) to long
                    // (multi-frame) to maximize frame-count/timing diversity.
                    std::uniform_int_distribution<size_t> dist(2, pool.size());
                    // Keep the caller-supplied base as a stable scenario identity.
                    // Delivery accounting runs on the protocol callback thread and
                    // matches this prefix; replacing it with the random pool made a
                    // varied-length run deliver successfully but wait forever to drain.
                    msg = options_.auto_send_message;
                    msg += " ";
                    msg += pool.substr(0, dist(rng));
                } else {
                    msg = options_.auto_send_message;
                }
                msg += " #" + std::to_string(scenario_messages_sent_ + 1);
                const bool file_busy = protocol_.isFileTransferInProgress();
                const bool ready_now = protocol_.isReadyToSend();
                if (protocol_.sendMessage(msg)) {
                    guiLog("[scenario] message %d/%d accepted (%zu bytes)",
                           scenario_messages_sent_ + 1, message_target, msg.size());
                    if (file_busy) {
                        appendRxLogLine("[INFO] Message queued - file transfer in progress, will send after.");
                    } else if (!ready_now) {
                        appendRxLogLine("[INFO] Message queued - waiting for DATA turn.");
                    }
                    scenario_message_sent_ = true;
                    scenario_message_sent_at_ = now;
                    ++scenario_messages_sent_;
                }
            }
        }
        // Phase 2: file, but only after ALL messages have finished transmitting
        // (TX idle) plus a short settle, so the payloads don't collide on the air.
        // Do not require protocol_.isReadyToSend() here: after the peer has
        // legitimately taken the half-duplex DATA turn, sendFile() must enter the
        // protocol queue and request the turn back instead of waiting forever.
        const bool message_phase_clear =
            message_after_file ||
            options_.auto_send_message.empty() ||
            (scenario_messages_sent_ >= message_target &&
             scenario_messages_delivered_.load(std::memory_order_relaxed) >= message_target &&
             protocol_.getTxBacklogBytes() == 0 &&
             !tx_in_progress_ &&
             now - scenario_message_sent_at_ >= std::chrono::milliseconds(2000));
        if (!options_.auto_send_file.empty() && !scenario_file_started_ &&
            message_phase_clear) {
            guiLog("[scenario] sending file %s", options_.auto_send_file.c_str());
            startFileSend(options_.auto_send_file,
                          "[scenario] file send started: " + options_.auto_send_file);
            scenario_file_started_ = true;
        }

        if (options_.auto_cancel_file_after_sec > 0 && !scenario_file_cancel_issued_) {
            if (protocol_.isFileTransferInProgress()) {
                if (!scenario_file_cancel_timer_started_) {
                    scenario_file_cancel_timer_started_ = true;
                    scenario_file_cancel_started_at_ = now;
                    guiLog("[scenario] file cancel timer armed (%ds)",
                           options_.auto_cancel_file_after_sec);
                } else if (now - scenario_file_cancel_started_at_ >=
                           std::chrono::seconds(options_.auto_cancel_file_after_sec)) {
                    guiLog("[scenario] cancelling active file transfer");
                    cancelActiveFileTransfer();
                    scenario_file_cancel_issued_ = true;
                    appendRxLogLine("[scenario] file cancel requested");
                }
            }
        }

        // Mark the whole sequence dispatched once every requested phase fired.
        if (!scenario_payload_sent_ &&
            (options_.auto_send_message.empty() || scenario_messages_sent_ >= message_target) &&
            (options_.auto_reply_message.empty() || scenario_reply_sent_) &&
            (options_.auto_send_file.empty() || scenario_file_started_)) {
            scenario_payload_sent_ = true;
            scenario_connected_at_ = now;
        }
    }

    // Optional auto-disconnect after the payload has drained. With a file in
    // the script we additionally wait for the transfer to leave "in progress"
    // (so we don't tear the link down mid-transfer); the configured delay then
    // acts as a trailing grace period for the final ACKs. 0 = never disconnect.
    if (options_.auto_disconnect_after_sec > 0 && scenario_payload_sent_ &&
        !scenario_disconnect_issued_ &&
        state == protocol::ConnectionState::CONNECTED) {
        const bool file_drained =
            options_.auto_send_file.empty() ||
            (scenario_file_started_ && !protocol_.isFileTransferInProgress());
        const bool outbound_messages_delivered =
            (options_.auto_send_message.empty() ||
             scenario_messages_delivered_.load(std::memory_order_relaxed) >= message_target) &&
            (options_.auto_reply_message.empty() ||
             scenario_reply_delivered_.load(std::memory_order_relaxed));
        const bool message_backlog_drained = protocol_.getTxBacklogBytes() == 0;
        const bool hold_elapsed =
            now - scenario_connected_at_ >=
            std::chrono::seconds(options_.auto_disconnect_after_sec);
        if (file_drained && outbound_messages_delivered &&
            message_backlog_drained && hold_elapsed) {
            guiLog("[scenario] auto-disconnect (payload drained, %ds grace)",
                   options_.auto_disconnect_after_sec);
            scenario_disconnect_issued_ = true;
            scenario_disconnect_at_ = now;
            protocol_.disconnect();
        }
    }
}

// ─── Software-ALC (closed-loop TX drive, BUG-QAM16-RIG-LEVEL-BUDGET 2026-07-02) ───

void App::resetSoftwareAlc(const char* reason) {
    const float baseline = settings_.tx_drive;
    const float prev = alc_tx_drive_.exchange(baseline, std::memory_order_relaxed);
    alc_last_group_seq_ = -1;
    alc_last_advisory_ = 0;
    if (std::fabs(prev - baseline) > 1e-4f) {
        LOG_MODEM(INFO, "ALC: tx_drive %.3f -> %.3f (reset: %s)",
                  prev, baseline, reason ? reason : "-");
    }
}

void App::handleDriveAdvisory(uint8_t advisory, uint8_t group_seq) {
    // Belt-and-braces: Connection already gates on ULTRA_SOFTWARE_ALC before
    // firing this callback; keep the loop inert here too if it ever changes.
    if (!protocol::connection_policy::softwareAlcEnabled()) {
        return;
    }
    // Rate limit: at most ONE adjustment per ACKed group. The receiver's ACK for a
    // group can be detected more than once (sliding-timer SACK repeats); every
    // repeat carries the same group_seq + advisory, so dedup on the pair.
    if (alc_last_group_seq_ == static_cast<int>(group_seq) &&
        alc_last_advisory_ == advisory) {
        return;
    }
    alc_last_group_seq_ = static_cast<int>(group_seq);
    alc_last_advisory_ = advisory;

    const float baseline = settings_.tx_drive;
    // ALC ceiling: per-hardware tunable (2026-07-04, F19-F22 forensics). The Pi5
    // cheap card compresses above ~0.70 digital drive — 16QAM R3/4 craters cluster
    // there, and the walk has no natural DOWN on TX-side compression (no RX clip
    // signature), so the code ceiling IS the equilibrium. Radio-agnostic rule: the
    // knee is a property of the attached card — ULTRA_ALC_MAX_DRIVE overrides the
    // 0.85 default (clamped [0.3, kSoftwareAlcMaxPeakTarget]).
    static const float kAlcCeiling = [] {
        const float hard_max = ultra::sim::kSoftwareAlcMaxPeakTarget;  // 0.85 abs digital
        if (const char* e = std::getenv("ULTRA_ALC_MAX_DRIVE")) {
            const float v = std::strtof(e, nullptr);
            if (v >= 0.3f && v <= hard_max) return v;
        }
        return 0.70f;  // DEFAULT 2026-07-05: rig-validated cap (cheap-card TX
                       // compression craters dense rungs above ~0.70; env may
                       // raise toward the 0.85 hard max on clean hardware)
    }();
    const float ceiling = kAlcCeiling;
    const float floor = std::min(baseline, ceiling);  // never below configured drive
    const float cur = alc_tx_drive_.load(std::memory_order_relaxed);
    float next = cur;
    const char* dir = nullptr;
    if (advisory == ultra::waveform::tone_burst_ack::kDriveAdvisoryUp) {
        next = cur * protocol::connection_policy::kAlcUpStepFactor;    // +0.5 dB
        dir = "up";
    } else if (advisory == ultra::waveform::tone_burst_ack::kDriveAdvisoryDown) {
        next = cur * protocol::connection_policy::kAlcDownStepFactor;  // -2 dB fast
        dir = "down";
    } else {
        return;  // hold / reserved
    }
    next = std::clamp(next, floor, ceiling);
    if (std::fabs(next - cur) < 1e-4f) {
        // Clamped no-op (already at the ceiling/baseline): don't spam the log.
        LOG_MODEM(DEBUG,
                  "ALC: advisory=%s group_seq=%u clamped, tx_drive stays %.3f "
                  "(baseline=%.3f ceiling=%.3f)",
                  dir, static_cast<unsigned>(group_seq), cur, baseline, ceiling);
        return;
    }
    alc_tx_drive_.store(next, std::memory_order_relaxed);
    // Applied to SUBSEQUENT bursts only: the drive is read once per queued burst
    // in doQueueRealTxSamples (one scalar per whole burst — never mid-burst).
    LOG_MODEM(INFO, "ALC: tx_drive %.3f -> %.3f (advisory=%s group_seq=%u)",
              cur, next, dir, static_cast<unsigned>(group_seq));
}

float App::effectiveTxDriveForContext(const char* context) const {
    // Scope: connected OFDM data bursts ONLY. "TX burst audio" is the single
    // context string of the burst-transport TX path (live callback + deferred
    // flush both carry it). Handshake/control ("TX audio", PING/PONG) and ACKs
    // keep the configured baseline by context; MC-DPSK data bursts share the
    // burst context but are excluded by the waveform-mode check.
    if (!context || std::strcmp(context, "TX burst audio") != 0) {
        return settings_.tx_drive;
    }
    if (conn_state_cached_.load(std::memory_order_relaxed) !=
        protocol::ConnectionState::CONNECTED) {
        return settings_.tx_drive;
    }
    if (!protocol::isOFDMMode(modem_.getWaveformMode())) {
        return settings_.tx_drive;
    }
    return alc_tx_drive_.load(std::memory_order_relaxed);
}

bool App::queueRealTxSamples(const std::vector<float>& samples, const char* context,
                             bool in_qso_data, bool disconnect_control) {
    if (samples.empty()) {
        return false;
    }
    if (disconnect_teardown_active_.load(std::memory_order_acquire) &&
        !disconnect_control) {
        LOG_MODEM(WARN, "Teardown egress: dropped App TX audio (%s)",
                  context ? context : "unknown");
        return false;
    }

    // Carrier-sense gate is listen-before-call only. Once connected, DATA
    // access is owned by the protocol ISS/IRS turn state, not randomized CSMA.
    const auto cca_state = conn_state_cached_.load(std::memory_order_relaxed);
    const bool pre_connection =
        (cca_state == protocol::ConnectionState::DISCONNECTED ||
         cca_state == protocol::ConnectionState::PROBING);
    if (pre_connection && (!deferred_tx_.empty() || modem_.channelBusyForTx())) {
        deferTxSamples(samples, context, false);
        return true;  // queued (deferred), not dropped
    }
    if (in_qso_data && tx_in_progress_.load(std::memory_order_relaxed)) {
        deferTxSamples(samples, context, true, tx_end_time_);
        return true;
    }

    return doQueueRealTxSamples(samples, context, disconnect_control);
}

bool App::shouldDeferInQsoDataForTx() const {
    return false;
}

uint32_t App::nextInQsoDataBackoffMs() {
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<uint32_t> dist(250, 2500);
    return dist(rng);
}

void App::deferTxSamples(const std::vector<float>& samples, const char* context,
                         bool in_qso_data,
                         std::chrono::steady_clock::time_point earliest_flush) {
    const auto now = std::chrono::steady_clock::now();
    const bool local_tx_serialized = earliest_flush != std::chrono::steady_clock::time_point{};
    const uint32_t backoff_ms =
        (in_qso_data && !local_tx_serialized) ? nextInQsoDataBackoffMs() : 0;
    if (!local_tx_serialized) {
        earliest_flush = now + std::chrono::milliseconds(backoff_ms);
    }
    if (deferred_tx_.empty()) {
        // Start the deadlock-guard clock on the first deferral of a burst.
        deferred_tx_deadline_ = now + std::chrono::milliseconds(kMaxTxDeferMs);
    }
    if (deferred_tx_.size() >= kMaxDeferredTx) {
        deferred_tx_.pop_front();  // bounded memory: drop oldest
    }
    deferred_tx_.push_back(
        DeferredTx{DeferredTxKind::Samples,
                   std::vector<float>(samples.begin(), samples.end()),
                   {},
                   {},
                   context ? std::string(context) : std::string("TX audio"),
                   in_qso_data,
                   false,
                   earliest_flush});
    // Fix 4: this audio was rendered under the CURRENT data mode — stamp it so a
    // later mode commit invalidates it (purgeStaleDeferredDataTx).
    deferred_tx_.back().data_mode_gen =
        data_mode_generation_.load(std::memory_order_acquire);
    const auto wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        earliest_flush > now ? earliest_flush - now
                             : std::chrono::steady_clock::duration::zero()).count();
    guiLog("CCA: deferred %s%s (rms=%.4f thresh=%.4f depth=%zu wait=%lldms%s)",
           context ? context : "TX audio",
           in_qso_data ? " in-QSO DATA" : "",
           modem_.channelRms(), modem_.channelQuietThreshold(),
           deferred_tx_.size(), static_cast<long long>(wait_ms),
           local_tx_serialized ? " local-tx-serialize" : "");
}

void App::deferTxFrame(const Bytes& frame, const char* context,
                       bool expect_full_ofdm_anchor_after_tx) {
    const auto now = std::chrono::steady_clock::now();
    const uint32_t backoff_ms = nextInQsoDataBackoffMs();
    if (deferred_tx_.empty()) {
        deferred_tx_deadline_ = now + std::chrono::milliseconds(kMaxTxDeferMs);
    }
    if (deferred_tx_.size() >= kMaxDeferredTx) {
        deferred_tx_.pop_front();
    }
    deferred_tx_.push_back(
        DeferredTx{DeferredTxKind::Frame,
                   {},
                   Bytes(frame.begin(), frame.end()),
                   {},
                   context ? std::string(context) : std::string("TX audio"),
                   true,
                   expect_full_ofdm_anchor_after_tx,
                   now + std::chrono::milliseconds(backoff_ms)});
    // Fix 4: pre-encode defer — the frame BYTES were chunked under the current
    // ARQ grid; a mode commit re-chunks them, so stamp the generation here too.
    deferred_tx_.back().data_mode_gen =
        data_mode_generation_.load(std::memory_order_acquire);
    guiLog("CCA: deferred %s in-QSO DATA pre-encode (rms=%.4f thresh=%.4f depth=%zu backoff=%ums)",
           context ? context : "TX audio",
           modem_.channelRms(), modem_.channelQuietThreshold(),
           deferred_tx_.size(), backoff_ms);
}

void App::deferTxBurst(const std::vector<Bytes>& frames, const char* context,
                       uint16_t group_seq,
                       BurstAnchorOptions anchor_options) {
    const auto now = std::chrono::steady_clock::now();
    const uint32_t backoff_ms = nextInQsoDataBackoffMs();
    if (deferred_tx_.empty()) {
        deferred_tx_deadline_ = now + std::chrono::milliseconds(kMaxTxDeferMs);
    }
    if (deferred_tx_.size() >= kMaxDeferredTx) {
        deferred_tx_.pop_front();
    }
    deferred_tx_.push_back(
        DeferredTx{DeferredTxKind::Burst,
                   {},
                   {},
                   std::vector<Bytes>(frames.begin(), frames.end()),
                   context ? std::string(context) : std::string("TX burst audio"),
                   true,
                   false,
                   now + std::chrono::milliseconds(backoff_ms),
                   group_seq,
                   0,
                   anchor_options});
    // Fix 4: pre-encode burst defer — same staleness contract as deferTxFrame.
    deferred_tx_.back().data_mode_gen =
        data_mode_generation_.load(std::memory_order_acquire);
    guiLog("CCA: deferred %s in-QSO DATA pre-encode (frames=%zu rms=%.4f thresh=%.4f depth=%zu backoff=%ums)",
           context ? context : "TX burst audio",
           frames.size(), modem_.channelRms(), modem_.channelQuietThreshold(),
           deferred_tx_.size(), backoff_ms);
}

void App::purgeStaleDeferredDataTx() {
    // BUG-MC-RETRY-SPURIOUS fix 4 (stale CCA-deferred TX guard). Forensic fact
    // (rig E1): a data burst rendered at R3/4/epoch-0 was CCA-deferred at 149.990,
    // the mode committed to R2/3 at 152.757, and the stale audio was flushed anyway
    // at 160.670-169.652 — 9.0 s of airtime the peer could not decode (wrong
    // rate/geometry) on a half-duplex channel where every second of TX is the ONLY
    // second of link time. Dropping is strictly better: the ARQ still owns the
    // un-ACKed frames and re-renders them at the committed mode on its next refill.
    // Only DATA-class entries (in_qso_data) are eligible — control audio
    // (MODE_CHANGE, ACKs, tone bursts) never enters the deferred queue while
    // connected (see queueRealTxSamples), and pre-connection probes carry
    // in_qso_data=false, so the guard structurally cannot drop control frames.
    if (deferred_tx_.empty()) {
        return;
    }
    const uint32_t gen = data_mode_generation_.load(std::memory_order_acquire);
    double dropped_audio_sec = 0.0;   // rendered (Samples) entries
    size_t dropped_frames = 0;        // pre-encode (Frame/Burst) entries
    size_t dropped_entries = 0;
    for (auto it = deferred_tx_.begin(); it != deferred_tx_.end();) {
        if (it->in_qso_data && it->data_mode_gen != gen) {
            switch (it->kind) {
                case DeferredTxKind::Samples:
                    dropped_audio_sec += it->samples.size() / 48000.0;
                    break;
                case DeferredTxKind::Frame:
                    dropped_frames += 1;
                    break;
                case DeferredTxKind::Burst:
                    dropped_frames += it->frames.size();
                    break;
            }
            ++dropped_entries;
            it = deferred_tx_.erase(it);
        } else {
            ++it;
        }
    }
    if (dropped_entries > 0) {
        guiLog("CCA WARN: data-mode commit invalidated %zu deferred DATA TX entr%s "
               "(%.1f s rendered audio, %zu pre-encode frame(s)) — dropped, ARQ "
               "re-renders at the committed mode",
               dropped_entries, dropped_entries == 1 ? "y" : "ies",
               dropped_audio_sec, dropped_frames);
        if (deferred_tx_.empty()) {
            // Nothing left to guard; the deadline re-arms on the next deferral.
            deferred_tx_deadline_ = {};
        }
    }
}

void App::flushDeferredTxIfReady() {
    if (disconnect_teardown_active_.load(std::memory_order_acquire)) {
        if (!deferred_tx_.empty()) {
            guiLog("Teardown egress: purging %zu deferred non-close TX entr%s",
                   deferred_tx_.size(), deferred_tx_.size() == 1 ? "y" : "ies");
            deferred_tx_.clear();
        }
        deferred_tx_deadline_ = {};
        return;
    }
    purgeStaleDeferredDataTx();
    if (deferred_tx_.empty()) {
        return;
    }
    const auto st = conn_state_cached_.load(std::memory_order_relaxed);
    const bool pre_connection =
        (st == protocol::ConnectionState::DISCONNECTED ||
         st == protocol::ConnectionState::PROBING);
    const bool connected = (st == protocol::ConnectionState::CONNECTED);
    if (!deferred_tx_.front().in_qso_data && !pre_connection) {
        // Pre-connection probes are stale once the call phase is over.
        deferred_tx_.clear();
        return;
    }
    if (deferred_tx_.front().in_qso_data && !connected) {
        deferred_tx_.clear();
        return;
    }
    if (tx_in_progress_.load()) {
        return;  // don't stack a deferred burst on top of our own ongoing TX
    }

    const auto now = std::chrono::steady_clock::now();
    if (deferred_tx_.front().in_qso_data && now < deferred_tx_.front().earliest_flush) {
        return;
    }

    const bool channel_idle = deferred_tx_.front().in_qso_data
        ? modem_.channelIdleForTx(std::chrono::milliseconds(kInQsoDataQuietGuardMs))
        : !modem_.channelBusyForTx();
    const bool defer_timed_out =
        now >= deferred_tx_deadline_;
    if (!channel_idle) {
        if (deferred_tx_.front().in_qso_data) {
            const uint32_t backoff_ms = nextInQsoDataBackoffMs();
            deferred_tx_.front().earliest_flush =
                now + std::chrono::milliseconds(backoff_ms);
            return;  // in-QSO DATA must wait for a genuinely clear channel
        }
        if (!defer_timed_out) {
            return;  // peer still transmitting and we haven't hit the safety bound yet
        }
    }

    // Flush one burst per call; doQueueRealTxSamples() sets tx_in_progress_, so the
    // next deferred burst waits for this TX to finish before its turn (half-duplex).
    DeferredTx front = std::move(deferred_tx_.front());
    deferred_tx_.pop_front();
    const char* flush_reason = channel_idle
        ? (front.in_qso_data ? "channel idle guard" : "channel idle")
        : "defer timeout";
    guiLog("CCA: flushing deferred %s (%s, remaining=%zu)",
           front.context.c_str(),
           flush_reason,
           deferred_tx_.size());
    if (!deferred_tx_.empty()) {
        // Re-arm the guard clock for the remaining queued bursts.
        deferred_tx_deadline_ = now + std::chrono::milliseconds(kMaxTxDeferMs);
    }
    std::vector<float> samples;
    switch (front.kind) {
        case DeferredTxKind::Samples:
            samples = std::move(front.samples);
            break;
        case DeferredTxKind::Frame:
            samples = modem_.transmit(front.frame);
            if (front.expect_full_ofdm_anchor_after_tx) {
                modem_.expectFullOFDMAnchorOnce();
            }
            break;
        case DeferredTxKind::Burst:
            // Keep the encoder Z aligned to the connection policy (see the live
            // transmitBurst path) for deferred bursts too.
            modem_.setBurstLiftingZ(static_cast<uint8_t>(protocol_.selectBurstLiftingZ()));
            samples = modem_.transmitBurst(front.frames, front.group_seq,
                                            front.burst_anchor_options);
            break;
    }
    if (samples.empty()) {
        guiLog("CCA: dropped deferred %s because encoder returned no samples",
               front.context.c_str());
        return;
    }
    doQueueRealTxSamples(samples, front.context.c_str());
}

bool App::doQueueRealTxSamples(const std::vector<float>& samples, const char* context,
                               bool disconnect_control) {
    if (samples.empty()) {
        return false;
    }
    // Re-check at the final physical queue boundary. The teardown callback may
    // arrive on the RX thread after queueRealTxSamples() passed its first gate.
    if (disconnect_teardown_active_.load(std::memory_order_acquire) &&
        !disconnect_control) {
        LOG_MODEM(WARN, "Teardown egress: blocked App TX at audio commit (%s)",
                  context ? context : "unknown");
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

        std::vector<float> sim_samples(samples.begin(), samples.end());
        // SIMULATOR FIDELITY (ULTRA_SIM_PAPR_PENALTY, default off): by default the sim TX
        // RMS-normalizes every burst to the fixed in-band reference, so high-PAPR coherent
        // OFDM data frames ride at the same average power as the low-PAPR chirp/control
        // frames and the OTASim reference-sized noise floor never sees the peak-limited-PA
        // penalty a real radio cannot avoid. With the knob on we drive the sim TX through
        // the SAME peak normalization the hardware path uses (peak -> settings_.tx_drive):
        // one DAC/ALC gain across the whole continuous burst, so a ~14-15 dB-PAPR data frame
        // delivers only (tx_drive/PAPR) average power. The noise stays pinned to the fixed
        // reference, so the data frames' effective SNR drops by exactly their PAPR back-off
        // (~10.26 dB measured for coherent QPSK R2/3) — reproducing the IONOS burst stall
        // rig-free. Knob off => byte-identical to today.
        static const bool kSimPaprPenalty = [] {
            const char* e = std::getenv("ULTRA_SIM_PAPR_PENALTY");
            return e && e[0] == '1';
        }();
        if (kSimPaprPenalty) {
            // ULTRA_SIM_TX_PEAK optionally overrides the peak target (default = the operator
            // tx_drive). Dialing it low (e.g. 0.12) reproduces the LOW RX operating level of a
            // weak/cheap TX (real IONOS Pi5 card) so the absolute-vs-relative erasure gate can
            // be A/B'd rig-free; clamped into the hardware peak range by normalizeTxBurstForHardware.
            static const float kSimTxPeak = [] {
                const char* e = std::getenv("ULTRA_SIM_TX_PEAK");
                return (e && *e) ? static_cast<float>(std::atof(e)) : -1.0f;
            }();
            // Software-ALC: PAPR-penalty sim TX rides the same effective drive as the
            // hardware path (connected OFDM data bursts walk with the loop; everything
            // else = configured baseline) so the loop is A/B-able rig-free. The DEFAULT
            // sim path below RMS-normalizes to reference and ignores drive entirely.
            const float peak_target = (kSimTxPeak > 0.0f)
                ? kSimTxPeak : effectiveTxDriveForContext(context);
            const auto hw = ultra::sim::normalizeTxBurstForHardware(sim_samples, peak_target);
            const auto post = ultra::sim::measureTxBurstInBandRms(sim_samples);
            LOG_WARN("AUDIO",
                     "OTASim TX PAPR-penalty ON: peak_before=%.3f peak_after=%.3f "
                     "gain=%.4f in_band_rms=%.6f (%.2f dB vs reference 0.30483)",
                     hw.peak_before_gain, hw.peak_after_gain, hw.gain_to_target,
                     post.in_band_rms,
                     20.0f * std::log10(std::max(post.in_band_rms, 1e-6f) / 0.30482664f));
        } else {
            const auto measurement =
                ultra::sim::normalizeTxBurstToReference(sim_samples);
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
        }

        const size_t tx_duration_ms = (sim_samples.size() * 1000) / 48000;
        tx_in_progress_ = true;
        tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

        if (waterfall_) {
            waterfall_->addSamples(sim_samples.data(), sim_samples.size());
        }
        if (recording_enabled_) {
            recorded_tx_samples_.insert(recorded_tx_samples_.end(),
                                        sim_samples.begin(),
                                        sim_samples.end());
        }

        std::string error;
        if (!ota_audio_->queueTxSamples(sim_samples, &error)) {
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
    modem_.clearRxBuffer(/*for_tx_echo=*/true);  // #67: connected-OFDM path may preserve warm-sync

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

    std::vector<float> hardware_samples(samples.begin(), samples.end());
    // Software-ALC: connected OFDM data bursts use the closed-loop-walked drive
    // (alc_tx_drive_, clamped [settings baseline, 0.85]); handshake/control/ACK/
    // MC-DPSK keep the configured settings_.tx_drive. One scalar per whole burst —
    // adjustments land between bursts, never inside one.
    const float requested_peak = effectiveTxDriveForContext(context);
    const auto measurement = ultra::sim::normalizeTxBurstForHardware(
        hardware_samples, requested_peak);
    if (protocol::connection_policy::softwareAlcEnabled() && context &&
        std::strcmp(context, "TX burst audio") == 0) {
        // This is the last software boundary before the samples enter the audio
        // playback queue.  Log the measurement returned from the operation that
        // actually scaled this burst, rather than merely logging alc_tx_drive_.
        // The campaign can therefore distinguish "the controller moved" from
        // "the transmitted sample vector moved", and any downstream sound-card,
        // radio, or channel-simulator level normalization remains separately
        // observable at the receiver.
        LOG_MODEM(INFO,
                  "ALC-TX-APPLIED: requested=%.3f target=%.3f peak_before=%.6f "
                  "gain=%.6f peak_after=%.6f active=%zu samples=%zu fragment=%d",
                  requested_peak, measurement.target_peak,
                  measurement.peak_before_gain, measurement.gain_to_target,
                  measurement.peak_after_gain, measurement.active_samples,
                  hardware_samples.size(),
                  measurement.burst_fragment_warning ? 1 : 0);
    }
    if (measurement.burst_fragment_warning) {
        LOG_WARN("AUDIO",
                 "%s: hardware TX peak normalization bypassed fragment "
                 "active=%zu minimum=%zu samples=%zu target=%.3f",
                 context ? context : "TX audio",
                 measurement.active_samples,
                 ultra::sim::kTxBurstMinimumActiveSamples,
                 hardware_samples.size(),
                 measurement.target_peak);
    }

    size_t tx_duration_ms = (hardware_samples.size() * 1000) / 48000;
    tx_in_progress_ = true;
    tx_end_time_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(tx_duration_ms + 100);

    if (waterfall_) {
        waterfall_->addSamples(hardware_samples.data(), hardware_samples.size());
    }

    if (recording_enabled_) {
        recorded_tx_samples_.insert(recorded_tx_samples_.end(),
                                    hardware_samples.begin(),
                                    hardware_samples.end());
    }

    audio_.startPlayback();
    audio_.queueTxSamples(hardware_samples);
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
        if (isFileCancelAudioDrainActive()) {
            continue;
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

OperatorSNRDisplay App::updateSnrBallistics(const OperatorSNRDisplay& raw, bool connected) {
    // Idle/disconnected: no ballistics — the live idle reading is what to show, and the
    // last connected value is stale. Reset so a new connection warms from scratch.
    if (!connected) {
        snr_ballistics_valid_ = false;
        snr_ballistics_have_raw_ = false;
        return raw;
    }

    // Advance the EMA at most once per UI frame (both the channel-status meter and the
    // bottom status line call this in the same frame; only the first advances). And only
    // fold a NEW measurement: between bursts the modem holds the same per-burst sample, so
    // updating every frame would converge to it and defeat the smoothing.
    const int frame = ImGui::GetFrameCount();
    if (frame != snr_ballistics_frame_) {
        snr_ballistics_frame_ = frame;
        if (raw.valid) {
            const bool new_sample =
                !snr_ballistics_have_raw_ || raw.snr_db != snr_ballistics_last_raw_db_;
            if (new_sample) {
                snr_ballistics_last_raw_db_ = raw.snr_db;
                snr_ballistics_have_raw_ = true;
                if (!snr_ballistics_valid_) {
                    snr_ballistics_db_ = raw.snr_db;  // snap on the first sample
                } else {
                    // Mild asymmetry: respond a little faster to a DROP than a rise, so a
                    // genuine fade is shown promptly and we never hold a stale-optimistic
                    // reading, while per-burst spikes are damped. ~3-5 samples to settle.
                    const float alpha = (raw.snr_db < snr_ballistics_db_) ? 0.35f : 0.22f;
                    snr_ballistics_db_ += alpha * (raw.snr_db - snr_ballistics_db_);
                }
                snr_ballistics_valid_ = true;
                snr_ballistics_source_ = raw.source;
            }
        }
        // raw invalid while connected (between-burst gap): HOLD the smoothed value.
    }

    if (!snr_ballistics_valid_) {
        return raw;  // not warmed yet — show whatever raw says (likely "-- dB")
    }
    OperatorSNRDisplay out;
    out.valid = true;
    out.snr_db = snr_ballistics_db_;
    out.source = snr_ballistics_source_;
    return out;
}

void App::renderCompactChannelStatus(const LoopbackStats& stats, Modulation data_mod, CodeRate data_rate,
                                     const protocol::ConnectionStats& conn_stats) {
    // Compact horizontal Channel Status display
    ImGui::BeginChild("ChannelStatus", ImVec2(0, 140), false);

    auto conn_state = protocol_.getState();
    // During an active connection, show the OFDM in-band DECODE SNR (the real link SNR) and hold it
    // through between-burst gaps; when idle/disconnected, prefer the live idle reading (the last
    // OFDM value is stale then).
    const bool prefer_ofdm_snr = (conn_state == protocol::ConnectionState::CONNECTED);
    const auto operator_snr =
        updateSnrBallistics(selectOperatorSNRDisplay(stats, prefer_ofdm_snr), prefer_ofdm_snr);

    // Row 1: Connection state + Channel Quality (only when connected) + SNR bar
    if (conn_state == protocol::ConnectionState::DISCONNECTED) {
        // Not connected - show idle state
        ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f), "IDLE");
        ImGui::SameLine();
        ImGui::TextDisabled("[Standby]");
        ImGui::SameLine();
        ImGui::Text("%s:", operator_snr.valid
                    ? snrSourceDisplayLabel(operator_snr.source) : "SNR");
        ImGui::SameLine();
        ImVec4 snr_color = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
        float snr_normalized = 0.0f;
        char snr_text[24];
        snprintf(snr_text, sizeof(snr_text), "-- dB");
        if (operator_snr.valid) {
            snr_normalized = std::max(0.0f, std::min(1.0f, operator_snr.snr_db / 40.0f));
            if (operator_snr.snr_db >= 25.0f) {
                snr_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
            } else if (operator_snr.snr_db >= 15.0f) {
                snr_color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);
            } else {
                snr_color = ImVec4(1.0f, 0.4f, 0.2f, 1.0f);
            }
            snprintf(snr_text, sizeof(snr_text), "%.1f dB", operator_snr.snr_db);
        }
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, snr_color);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(snr_normalized, ImVec2(-1, 16), snr_text);
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
        ImVec4 wf_color = (waveform == protocol::WaveformMode::OFDM_CHIRP) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
                          (waveform == protocol::WaveformMode::MC_DPSK) ? ImVec4(0.8f, 0.8f, 0.4f, 1.0f) :
                          (waveform == protocol::WaveformMode::MFSK) ? ImVec4(0.8f, 0.4f, 0.8f, 1.0f) :
                                                                       ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
        ImGui::Text("RX:");
        ImGui::SameLine();
        ImGui::TextColored(wf_color, "%s", wf_str);
        ImGui::SameLine();

        // Show mode-appropriate settings
        ImVec4 mode_quality_color;
        const char* mode_quality = "Good";

        // Channel-class label (task #43). The raw fading_index CANNOT separate Good from Moderate
        // (both ~0.55) and is idle-biased (reads ~AWGN in the gaps between bursts) — so it is NOT a
        // trustworthy class and we deliberately do NOT paint it. We show a class ONLY when the
        // Doppler-coherence discriminator is valid AND confident (Good >= kCoherenceGoodThreshold,
        // Moderate <= kCoherenceModerateThreshold); that verdict is the slow channel-state estimate
        // and we hold it through between-burst gaps (it overrides the idle AWGN reading; the disc
        // resets on disconnect/mode-change so it can't go stale). Until then — warmup (~60-90s of
        // OFDM data needed to validate) or the uncertain dead zone — show a dim "acquiring" rather
        // than blink the blind raw class. (Idle/disconnected shows IDLE in the branch above.)
        bool coh_confident = false;
        if (modem_.getDopplerCoherenceValid()) {
            const float coh = modem_.getDopplerCoherenceScore();
            if (coh >= protocol::connection_policy::kCoherenceGoodThreshold) {
                mode_quality = fadingToQualityWithColor(
                    protocol::connection_policy::kRepresentativeGoodFadingIndex, mode_quality_color);
                coh_confident = true;
            } else if (coh <= protocol::connection_policy::kCoherenceModerateThreshold) {
                mode_quality = fadingToQualityWithColor(
                    protocol::connection_policy::kRepresentativeModerateFadingIndex, mode_quality_color);
                coh_confident = true;
            }
        }
        if (!coh_confident) {
            mode_quality = "acquiring";
            mode_quality_color = ImVec4(0.55f, 0.55f, 0.55f, 1.0f);  // dim: no trustworthy class yet
        }

        if (waveform == protocol::WaveformMode::MC_DPSK) {
            // For MC-DPSK, just show carrier count (DQPSK R1/4 is implicit)
            int carriers = modem_.getMCDPSKCarriers();
            ImGui::Text("%d carriers", carriers);
        } else {
            // §14.36: show the LIVE decoder rate/mod (reflects the descriptor-
            // driven mid-transfer switch from adaptive bursts). When it differs
            // from the negotiated baseline (data_mod / data_rate), highlight the
            // drop in yellow so the operator can SEE the modem adapting in real
            // time — restores to normal color when it climbs back.
            const Modulation live_mod = modem_.getDecoderModulation();
            const CodeRate live_rate = modem_.getDecoderCodeRate();
            const bool adapted = (live_mod != data_mod) || (live_rate != data_rate);
            if (adapted) {
                ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f),
                                   "%s %s", modulationToString(live_mod),
                                   codeRateToString(live_rate));
            } else {
                ImGui::Text("%s %s", modulationToString(live_mod),
                            codeRateToString(live_rate));
            }
        }
        ImGui::SameLine();
        ImGui::TextColored(mode_quality_color, "[%s]", mode_quality);

        // Row 2: Our SNR measurement
        const bool connected_snr_valid = operator_snr.valid || connected_peer_snr_valid_;
        const float connected_snr_db = operator_snr.valid
            ? operator_snr.snr_db
            : connected_peer_snr_db_;
        const char* connected_snr_label = operator_snr.valid
            ? snrSourceDisplayLabel(operator_snr.source)
            : "peer SNR";
        ImVec4 sync_color = stats.synced ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
        ImGui::TextColored(sync_color, "%s", stats.synced ? "SYNC" : "----");
        ImGui::SameLine();
        ImGui::Text("%s:", connected_snr_valid ? connected_snr_label : "SNR");
        ImGui::SameLine();

        // SNR bar - color indicates signal strength
        float snr_normalized = connected_snr_valid
            ? std::max(0.0f, std::min(1.0f, connected_snr_db / 40.0f))
            : 0.0f;
        // Color based on SNR value (green=good signal, yellow=moderate, red=weak)
        ImVec4 snr_color;
        if (!connected_snr_valid) {
            snr_color = ImVec4(0.3f, 0.3f, 0.3f, 1.0f);
        } else if (connected_snr_db >= 25.0f) {
            snr_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green
        } else if (connected_snr_db >= 15.0f) {
            snr_color = ImVec4(0.8f, 0.8f, 0.0f, 1.0f);  // Yellow
        } else {
            snr_color = ImVec4(1.0f, 0.4f, 0.2f, 1.0f);  // Orange-red
        }
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, snr_color);
        char snr_text[40];
        if (connected_snr_valid) {
            // Same dial-equivalent presentation as the bottom status bar: the
            // measured value is EFFECTIVE (fade-state) SNR, which under-reads the
            // operator's AWGN-equivalent dial on a fading channel — show the
            // calibrated dial-equivalent alongside (the same +basis the rate
            // selection applies) so the meter matches the knob the operator set.
            const float sidebar_fading = modem_.getFadingIndex();
            if (sidebar_fading >= protocol::kFadingAwgnMax) {
                // Same one-source-of-truth helper as the bottom status bar and the
                // entry selection (flat basis by default, calibrated affine map
                // under ULTRA_CONNECT_AFFINE_BASIS).
                snprintf(snr_text, sizeof(snr_text), "%.1f eff (~%.0f dial)",
                         connected_snr_db,
                         protocol::connection_policy::dialEquivalentSnrDb(
                             connected_snr_db, sidebar_fading,
                             protocol::connection_policy::connectAffineBasisEnabled()));
            } else {
                snprintf(snr_text, sizeof(snr_text), "%.1f dB", connected_snr_db);
            }
        } else {
            snprintf(snr_text, sizeof(snr_text), "-- dB");
        }
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::ProgressBar(snr_normalized, ImVec2(-1, 16), snr_text);
        ImGui::PopStyleColor();

        // §14.36 Phase 5c adaptive-rate observability: pre-FEC decode headroom
        // (quality 0..1; we show it as a green→red bar + the % "headroom") plus
        // the most recent adaptive action ("rate R3/4 -> R2/3 (q=0.18)" / "hold
        // R3/4 (q=0.85)"). Only drawn while adaptation is enabled and connected,
        // so the panel stays quiet on the non-adaptive shipping path.
        if (protocol_.adaptiveRateEnabled()) {
            const float q = protocol_.lastGroupQuality();    // <0 = no sample yet
            const std::string action = protocol_.lastAdaptiveAction();
            ImGui::TextDisabled("Adapt:");
            ImGui::SameLine();
            if (q < 0.0f) {
                ImGui::TextDisabled("waiting for first group...");
            } else {
                // Map headroom -> color: green=lots of room (q>=0.7), yellow=okay
                // (0.25-0.7), red=at the cliff (<0.25). Same thresholds the
                // RateController uses to decide climb/drop, so the bar shows
                // operationally where the link is sitting on the rate ladder.
                ImVec4 q_color;
                if (q >= 0.70f)      q_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);
                else if (q >= 0.25f) q_color = ImVec4(1.0f, 0.85f, 0.2f, 1.0f);
                else                 q_color = ImVec4(1.0f, 0.4f, 0.3f, 1.0f);
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, q_color);
                char qtxt[40];
                snprintf(qtxt, sizeof(qtxt), "%.0f%% headroom", q * 100.0f);
                ImGui::ProgressBar(std::max(0.0f, std::min(1.0f, q)),
                                   ImVec2(-1, 12), qtxt);
                ImGui::PopStyleColor();
                if (!action.empty()) {
                    // Color the line by whether the controller dropped, held,
                    // or climbed — a glanceable "what just happened" cue.
                    ImVec4 act_color;
                    if (action.find("-> R1_4") != std::string::npos ||
                        action.find("-> R1/4") != std::string::npos ||
                        action.find("-> R1/2") != std::string::npos ||
                        action.find("-> R2/3") != std::string::npos) {
                        // "rate X -> Y" — direction depends on ladder order.
                        // Detect drop vs climb by which rate appears second.
                        const auto arrow = action.find("->");
                        const bool to_lower = (arrow != std::string::npos) &&
                            (action.find("R1/4", arrow) != std::string::npos ||
                             action.find("R1/2", arrow) != std::string::npos);
                        act_color = to_lower ? ImVec4(1.0f, 0.6f, 0.2f, 1.0f)   // dropping (orange)
                                             : ImVec4(0.5f, 1.0f, 0.6f, 1.0f);  // climbing (green)
                    } else if (action.rfind("hold", 0) == 0) {
                        act_color = ImVec4(0.6f, 0.7f, 0.9f, 1.0f);             // holding (blue)
                    } else {
                        act_color = ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
                    }
                    ImGui::TextColored(act_color, "%s", action.c_str());
                }
            }
        }
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
        ImVec4 wf_color = (connect_wf == protocol::WaveformMode::OFDM_CHIRP) ? ImVec4(0.4f, 0.8f, 1.0f, 1.0f) :
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
        // Two compact lines so every counter is visible in the narrow left panel
        // (TextColored does not wrap, so the old single line clipped nack/dupACK/fail).
        char arq_l1[128];
        snprintf(arq_l1, sizeof(arq_l1), "ARQ retx:%d to:%d fast:%d",
                 arq.retransmissions, arq.timeouts, arq.retransmissions_fast_hole);
        ImGui::TextColored(arq_color, "%s", arq_l1);
        char arq_l2[128];
        snprintf(arq_l2, sizeof(arq_l2), "  probe:%d nack:%d dup:%d fail:%d",
                 arq.retransmissions_hole_probe, arq.retransmissions_nack,
                 arq.duplicate_acks_ignored, arq.failed);
        ImVec4 arq_l2_color = (arq.failed > 0) ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) : arq_color;
        ImGui::TextColored(arq_l2_color, "%s", arq_l2);
        if (arq.failed > diagnostics_last_arq_failed_) {
            diagnostics_last_arq_failed_ = arq.failed;
            ultra::diagnostics::DiagnosticsRecorder::instance().emitText(
                "fault", "fault.triggered",
                "{\"reason\":\"arq_failed_increment\",\"policy\":\"emit_only\"}");
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
    // Auto-scroll only when the operator is already parked at the bottom. The
    // check uses last frame's scroll extent (rows for this frame are not
    // submitted yet), so if the user scrolled up to read history we leave their
    // position untouched. The old code called SetScrollHereY() unconditionally
    // every frame, which re-pinned to the bottom and made it impossible to
    // scroll back up at all.
    const bool rx_log_stick_bottom =
        ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 2.0f;
    for (const auto& msg : rx_log_snapshot) {
        ImVec4 color(0.7f, 0.7f, 0.7f, 1.0f);
        if (operatorLineStartsWith(msg, "TX #") ||
            operatorLineStartsWith(msg, "[TX]")) {
            color = ImVec4(0.35f, 0.95f, 0.45f, 1.0f);
        } else if (operatorLineStartsWith(msg, "OK #")) {
            color = ImVec4(0.35f, 0.65f, 1.0f, 1.0f);
        } else if (operatorLineStartsWith(msg, "FAIL ")) {
            color = ImVec4(1.0f, 0.35f, 0.2f, 1.0f);
        } else if (operatorLineStartsWith(msg, "[RX")) {
            color = ImVec4(0.35f, 0.95f, 0.95f, 1.0f);
        } else if (operatorLineStartsWith(msg, "[SIM") ||
                   operatorLineStartsWith(msg, "[OTASIM")) {
            color = ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
        } else if (operatorLineStartsWith(msg, "[SYS")) {
            color = ImVec4(0.8f, 0.8f, 0.8f, 1.0f);
        } else if (msg.find("[FAILED]") != std::string::npos) {
            color = ImVec4(1.0f, 0.4f, 0.4f, 1.0f);
        }
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", msg.c_str());
        ImGui::PopStyleColor();
    }
    if (rx_log_stick_bottom) ImGui::SetScrollHereY(1.0f);
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

    // Connection/accept/exit conveniences do not own the payload plane. Keep
    // manual message/file controls available unless automation can itself submit
    // or cancel application data.
    const bool scenario_drives_payload =
        !options_.auto_send_file.empty() ||
        !options_.auto_send_message.empty() ||
        !options_.auto_reply_message.empty() ||
        options_.auto_cancel_file_after_sec > 0;
    const bool file_transfer_busy = protocol_.isFileTransferInProgress();
    const bool tx_inprog = tx_in_progress_.load();
    const size_t tx_text_len = boundedCStringLen(tx_text_buffer_);
    const size_t tx_backlog_bytes = protocol_.getTxBacklogBytes();
    const bool connected_now = protocol_.isConnected();
    const bool would_enable_send =
        !file_transfer_busy && !tx_inprog && tx_text_len > 0 && connected_now &&
        tx_backlog_bytes == 0;
    const bool can_send = !scenario_drives_payload && would_enable_send;

    if (!send_btn_log_valid_ ||
        send_btn_log_enabled_ != can_send ||
        send_btn_log_scenario_ != scenario_drives_payload ||
        send_btn_log_file_busy_ != file_transfer_busy ||
        send_btn_log_tx_inprog_ != tx_inprog ||
        send_btn_log_textlen_ != tx_text_len ||
        send_btn_log_backlog_bytes_ != tx_backlog_bytes ||
        send_btn_log_connected_ != connected_now ||
        send_btn_log_would_enable_ != would_enable_send) {
        guiLog("SEND_BTN enabled=%d scenario=%d file_busy=%d tx_inprog=%d textlen=%zu backlog=%zu connected=%d would_enable=%d",
               can_send ? 1 : 0,
               scenario_drives_payload ? 1 : 0,
               file_transfer_busy ? 1 : 0,
               tx_inprog ? 1 : 0,
               tx_text_len,
               tx_backlog_bytes,
               connected_now ? 1 : 0,
               would_enable_send ? 1 : 0);
        send_btn_log_valid_ = true;
        send_btn_log_enabled_ = can_send;
        send_btn_log_scenario_ = scenario_drives_payload;
        send_btn_log_file_busy_ = file_transfer_busy;
        send_btn_log_tx_inprog_ = tx_inprog;
        send_btn_log_textlen_ = tx_text_len;
        send_btn_log_backlog_bytes_ = tx_backlog_bytes;
        send_btn_log_connected_ = connected_now;
        send_btn_log_would_enable_ = would_enable_send;
    }

    // Operator text messages are intentionally capped at 255 bytes by the
    // 256-byte NUL-terminated compose buffer. Keep one payload outstanding at a
    // time so a message and file cannot share ambiguous ARQ completion state.
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 90);
    const bool send_on_enter =
        ImGui::InputText("##txinput", tx_text_buffer_, sizeof(tx_text_buffer_),
                         ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();

    const ImVec4 send_color = can_send
        ? ImVec4(0.3f, 0.6f, 0.3f, 1.0f)
        : ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, send_color);
    ImGui::BeginDisabled(!can_send);
    const bool send_clicked = ImGui::Button("Send##msg", ImVec2(80, 0));
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
        if (file_transfer_busy) {
            ImGui::SetTooltip("File transfer in progress - cancel it before sending a message.");
        } else if (tx_backlog_bytes != 0 || tx_inprog) {
            ImGui::SetTooltip("Wait for the current transmission to finish.");
        }
    }
    // InputText mutates the buffer after can_send was sampled above. Let the
    // handler re-check current state so Enter on a previously empty composer
    // submits immediately instead of requiring a second key press.
    if (send_clicked || send_on_enter) {
        sendMessage();
    }
    ImGui::EndDisabled();
    ImGui::PopStyleColor();

    // File Transfer (compact row)
    if (protocol_.isFileTransferInProgress()) {
        auto progress = protocol_.getFileProgress();
        ImGui::TextColored(progress.is_sending ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f) : ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
            "%s: %s", progress.is_sending ? "TX" : "RX", progress.filename.c_str());
        ImGui::SameLine();
        // Bar reflects the TRUE received total (contiguous + SR-ARQ out-of-order buffered).
        ImGui::ProgressBar(progress.receivedPercentage() / 100.0f, ImVec2(100, 16));
        ImGui::SameLine();
        const float kKB = 1024.0f;
        if (progress.is_sending) {
            // TX: received == transferred (sender's acked offset).
            if (progress.total_bytes >= 1024)
                ImGui::Text("%.1f/%.1f KB (%.0f%%)",
                    progress.transferred_bytes / kKB, progress.total_bytes / kKB,
                    progress.percentage());
            else
                ImGui::Text("%u/%u B (%.0f%%)",
                    progress.transferred_bytes, progress.total_bytes,
                    progress.percentage());
        } else {
            // RX: actual bytes received, with the contiguous (file-safe) % alongside —
            // the gap between "rcvd" and "asm" is the SR-ARQ recovery in flight.
            if (progress.total_bytes >= 1024)
                ImGui::Text("%.1f/%.1f KB (%.0f%% rcvd, %.0f%% asm)",
                    progress.received_bytes / kKB, progress.total_bytes / kKB,
                    progress.receivedPercentage(), progress.percentage());
            else
                ImGui::Text("%u/%u B (%.0f%% rcvd, %.0f%% asm)",
                    progress.received_bytes, progress.total_bytes,
                    progress.receivedPercentage(), progress.percentage());
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) cancelActiveFileTransfer();
        // Live burst group # alongside the bar, so you can watch it advance.
        {
            auto burst = protocol_.getBurstActivity();
            if (burst.active && burst.groups_seen > 0) {
                ImGui::SameLine();
                ImGui::TextDisabled("grp %u", burst.group_seq);
            }
        }
    } else {
        // "Incoming burst, please hold..." — a burst is arriving but no file transfer is
        // established yet (FILE_START not decoded). Flash liveness so the operator doesn't
        // think the modem is dead: the group # advances and X/Y frames decode in real time.
        {
            auto burst = protocol_.getBurstActivity();
            if (burst.active) {
                // Start the RX throughput clock HERE — at the first burst — not at the
                // late FILE_START decode, so the receiver's kbps measures the real window.
                if (!rx_transfer_clock_armed_) {
                    file_transfer_start_time_ = std::chrono::steady_clock::now();
                    rx_transfer_clock_armed_ = true;
                }
                const float pulse =
                    0.55f + 0.45f * std::sin(static_cast<float>(ImGui::GetTime()) * 5.0f);
                ImGui::TextColored(
                    ImVec4(1.0f, 0.82f, 0.25f, pulse),
                    ">> INCOMING BURST - group %u, %u/%u frames decoded  (receiving, hold...)",
                    burst.group_seq, burst.frames_decoded, burst.frames_in_group);
            }
        }
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 160);
        ImGui::InputText("##filepath", file_path_buffer_, sizeof(file_path_buffer_));
        ImGui::SameLine();
        if (ImGui::Button("Browse", ImVec2(60, 0))) {
            file_browser_.setTitle("Select File");
            file_browser_.open();
        }
        ImGui::SameLine();
        bool can_send_file = !scenario_drives_payload &&
                             protocol_.isConnected() &&
                             protocol_.getTxBacklogBytes() == 0 &&
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
