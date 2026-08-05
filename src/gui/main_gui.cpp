// ProjectUltra GUI - Cross-platform modem interface
// Uses Dear ImGui with SDL2 + OpenGL 2.1 for maximum compatibility

#include "app.hpp"

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <chrono>
#include <ctime>
#include <exception>
#include <memory>
#include <vector>
#include <atomic>
#include <ultra/logging.hpp>
#include <ultra/build_info.hpp>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <DbgHelp.h>
#include <eh.h>
#include <signal.h>
#ifdef ERROR
#undef ERROR
#endif
#endif

namespace {

#ifdef _WIN32
HANDLE g_startup_log_handle = INVALID_HANDLE_VALUE;
PVOID g_vectored_handler = nullptr;
#else
FILE* g_startup_log_file = nullptr;
#endif
std::string g_startup_log_path;

void writeStartupLog(const char* fmt, ...) {
#ifdef _WIN32
    if (g_startup_log_handle == INVALID_HANDLE_VALUE) return;

    SYSTEMTIME st{};
    GetLocalTime(&st);
    char ts[48];
    std::snprintf(ts, sizeof(ts), "%04u-%02u-%02u %02u:%02u:%02u",
                  static_cast<unsigned>(st.wYear),
                  static_cast<unsigned>(st.wMonth),
                  static_cast<unsigned>(st.wDay),
                  static_cast<unsigned>(st.wHour),
                  static_cast<unsigned>(st.wMinute),
                  static_cast<unsigned>(st.wSecond));

    char msg[1536];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    char line[1664];
    int len = std::snprintf(line, sizeof(line), "[%s] %s\r\n", ts, msg);
    if (len <= 0) return;
    if (len > static_cast<int>(sizeof(line))) len = static_cast<int>(sizeof(line));

    DWORD written = 0;
    WriteFile(g_startup_log_handle, line, static_cast<DWORD>(len), &written, nullptr);
    FlushFileBuffers(g_startup_log_handle);
#else
    if (!g_startup_log_file) return;

    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now{};
#ifdef _WIN32
    localtime_s(&tm_now, &t);
#else
    localtime_r(&t, &tm_now);
#endif

    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_now);
    std::fprintf(g_startup_log_file, "[%s] ", ts);

    va_list args;
    va_start(args, fmt);
    std::vfprintf(g_startup_log_file, fmt, args);
    va_end(args);
    std::fprintf(g_startup_log_file, "\n");
    std::fflush(g_startup_log_file);
#endif
}

void initStartupLog() {
    namespace fs = std::filesystem;
    std::vector<fs::path> candidates;
    candidates.emplace_back(fs::path("logs") / "startup.log");
    candidates.emplace_back("startup.log");

#ifdef _WIN32
    if (const char* temp = std::getenv("TEMP")) {
        candidates.emplace_back(fs::path(temp) / "ProjectUltra" / "startup.log");
    }
#else
    if (const char* temp = std::getenv("TMPDIR")) {
        candidates.emplace_back(fs::path(temp) / "projectultra_startup.log");
    }
    candidates.emplace_back("/tmp/projectultra_startup.log");
#endif

    for (const auto& path : candidates) {
        std::error_code ec;
        if (!path.parent_path().empty()) {
            fs::create_directories(path.parent_path(), ec);
        }
#ifdef _WIN32
        HANDLE h = CreateFileA(path.string().c_str(),
                               FILE_APPEND_DATA,
                               FILE_SHARE_READ | FILE_SHARE_WRITE,
                               nullptr,
                               CREATE_ALWAYS,
                               FILE_ATTRIBUTE_NORMAL,
                               nullptr);
        if (h != INVALID_HANDLE_VALUE) {
            g_startup_log_handle = h;
            g_startup_log_path = path.string();
            break;
        }
#else
        g_startup_log_file = std::fopen(path.string().c_str(), "w");
        if (g_startup_log_file) {
            g_startup_log_path = path.string();
            break;
        }
#endif
    }

    if (
#ifdef _WIN32
        g_startup_log_handle != INVALID_HANDLE_VALUE
#else
        g_startup_log_file
#endif
    ) {
        writeStartupLog("ProjectUltra GUI startup log initialized");
    }
}

void closeStartupLog() {
#ifdef _WIN32
    if (g_startup_log_handle != INVALID_HANDLE_VALUE) {
        writeStartupLog("ProjectUltra GUI startup log closing");
        CloseHandle(g_startup_log_handle);
        g_startup_log_handle = INVALID_HANDLE_VALUE;
    }
#else
    if (g_startup_log_file) {
        writeStartupLog("ProjectUltra GUI startup log closing");
        std::fclose(g_startup_log_file);
        g_startup_log_file = nullptr;
    }
#endif
}

void printGuiUsage(const char* prog) {
    std::printf("ProjectUltra GUI\n\n");
    std::printf("Usage: %s [options]\n\n", prog ? prog : "ultra_gui");
    std::printf("Options:\n");
    std::printf("  -sim                         Use OTASim server audio backend\n");
    std::printf("  --ota-host <host:port>        OTASim gRPC endpoint for -sim\n");
    std::printf("  --ota-udp-host <host:port>    OTASim UDP audio endpoint override\n");
    std::printf("  --token <bearer_token>        OTASim auth token for -sim\n");
    std::printf("  --station-id <id>             OTASim station id for -sim\n");
    std::printf("  --session-id <id>             OTASim session id (default: lobby)\n");
    std::printf("  --monitor-audio               Play OTASim RX through local speakers\n");
    std::printf("  --monitor-device <name>       SDL audio output device for --monitor-audio\n");
    std::printf("  -rec [path]                   Record received audio\n");
    std::printf("  --software, -sw               Use software renderer and safe startup\n");
    std::printf("  --opengl, --gl                Use OpenGL renderer\n");
    std::printf("  --no-waterfall                Disable waterfall on startup\n");
    std::printf("  --waterfall                   Enable waterfall on startup\n");
    std::printf("  --monitor-ofdm [rate]         Monitor OFDM_CHIRP\n");
    std::printf("  --monitor-ofdm-narrow [rate]  Monitor OFDM_NARROW\n");
    std::printf("  --monitor-mcdpsk              Monitor MC-DPSK\n");
    std::printf("  --monitor-mod <mod>           Monitor modulation\n");
    std::printf("  Scenario scripting (drives the real UI actions; reuse with -sim + OTASim):\n");
    std::printf("  --auto-connect <peer>         Initiate a connection once the link is up\n");
    std::printf("  --connect-delay <s>           Wait N seconds after startup before auto-connect\n");
    std::printf("  --auto-accept                 Auto-accept the first incoming call\n");
    std::printf("  --auto-send-file <path>       Send file once CONNECTED\n");
    std::printf("  --auto-send-message <text>    Send message once CONNECTED\n");
    std::printf("  --auto-message-start-delay <s> Wait N seconds after CONNECTED before first auto-message\n");
    std::printf("  --auto-message-count <N>      Send N sequential numbered auto-messages (default 1)\n");
    std::printf("  --auto-message-interval <s>   Gap between sequential auto-messages (default 8)\n");
    std::printf("  --auto-message-vary-len       Randomize each auto-message length (mix short+long)\n");
    std::printf("  --auto-message-after-file     With file+message, send message after file completes/cancels\n");
    std::printf("  --auto-reply-message <text>   Responder: send this ONCE after a message is RECEIVED (bidi/turn test)\n");
    std::printf("  --auto-reply-after-messages <N> Responder: wait for N inbound messages before its one reply (default 1)\n");
    std::printf("  --auto-cancel-file-after <s>  Cancel active file transfer N seconds after first observed\n");
    std::printf("  --auto-disconnect-after <s>   Disconnect N seconds after payload drain\n");
    std::printf("  --disconnect-on-file-done     Caller-sender disconnects after file ACK completion\n");
    std::printf("                                (instead of idling until --auto-disconnect-after)\n");
    std::printf("  --exit-after <s>              Quit N seconds after startup\n");
    std::printf("  --half-duplex                 Bidirectional role-swap: both stations send (B2F).\n");
    std::printf("                                Pass --auto-send-file on BOTH for an A->B then B->A exchange.\n");
    std::printf("  --log-level <error|warn|info|debug|trace>\n");
    std::printf("                               Console verbosity (default: info)\n");
    std::printf("  --log-category <list>         Comma list: operator,audio,tnc,modem,\n");
    std::printf("                               demod,sync,ldpc,channel,all\n");
    std::printf("  --log-file <path>             Write logs to file instead of stderr\n");
    std::printf("  --version                     Print build provenance\n");
    std::printf("  --help, -h                    Show this help\n");
}

#ifdef _WIN32
void showFatalStartupMessage(const std::string& msg) {
    MessageBoxA(nullptr, msg.c_str(), "ProjectUltra Startup Error", MB_ICONERROR | MB_OK);
}

std::string buildCrashDumpPath(const char* tag) {
    std::filesystem::path base_dir = "logs";
    if (!g_startup_log_path.empty()) {
        std::filesystem::path startup_path(g_startup_log_path);
        if (!startup_path.parent_path().empty()) {
            base_dir = startup_path.parent_path();
        }
    }

    std::error_code ec;
    std::filesystem::create_directories(base_dir, ec);

    SYSTEMTIME st{};
    GetLocalTime(&st);
    char name[160];
    std::snprintf(name, sizeof(name),
                  "crash_%04u%02u%02u_%02u%02u%02u_%s_pid%lu.dmp",
                  static_cast<unsigned>(st.wYear),
                  static_cast<unsigned>(st.wMonth),
                  static_cast<unsigned>(st.wDay),
                  static_cast<unsigned>(st.wHour),
                  static_cast<unsigned>(st.wMinute),
                  static_cast<unsigned>(st.wSecond),
                  tag ? tag : "unknown",
                  static_cast<unsigned long>(GetCurrentProcessId()));
    return (base_dir / name).string();
}

void writeCrashDump(EXCEPTION_POINTERS* ex, const char* tag) {
    static std::atomic<bool> dump_written{false};
    bool expected = false;
    if (!dump_written.compare_exchange_strong(expected, true)) {
        return;
    }

    const std::string dump_path = buildCrashDumpPath(tag);
    HANDLE file = CreateFileA(dump_path.c_str(),
                              GENERIC_WRITE,
                              FILE_SHARE_READ,
                              nullptr,
                              CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL,
                              nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        writeStartupLog("Crash dump create failed: path=%s err=%lu",
                        dump_path.c_str(), static_cast<unsigned long>(GetLastError()));
        return;
    }

    MINIDUMP_EXCEPTION_INFORMATION exception_info{};
    MINIDUMP_EXCEPTION_INFORMATION* exception_info_ptr = nullptr;
    if (ex) {
        exception_info.ThreadId = GetCurrentThreadId();
        exception_info.ExceptionPointers = ex;
        exception_info.ClientPointers = FALSE;
        exception_info_ptr = &exception_info;
    }

    MINIDUMP_TYPE dump_type = static_cast<MINIDUMP_TYPE>(
        MiniDumpWithThreadInfo |
        MiniDumpWithDataSegs |
        MiniDumpWithHandleData |
        MiniDumpWithIndirectlyReferencedMemory
    );

    BOOL ok = MiniDumpWriteDump(GetCurrentProcess(),
                                GetCurrentProcessId(),
                                file,
                                dump_type,
                                exception_info_ptr,
                                nullptr,
                                nullptr);
    DWORD err = ok ? 0 : GetLastError();
    CloseHandle(file);

    if (ok) {
        writeStartupLog("Crash dump written: %s", dump_path.c_str());
    } else {
        writeStartupLog("Crash dump write failed: path=%s err=%lu",
                        dump_path.c_str(), static_cast<unsigned long>(err));
    }
}

std::string modulePathForAddress(void* addr) {
    if (!addr) {
        return "<unknown module>";
    }
    HMODULE mod = nullptr;
    if (!GetModuleHandleExA(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(addr),
            &mod)) {
        return "<unknown module>";
    }

    char path[MAX_PATH] = {};
    DWORD len = GetModuleFileNameA(mod, path, static_cast<DWORD>(sizeof(path)));
    if (len == 0 || len >= sizeof(path)) {
        return "<unknown module>";
    }
    return std::string(path, len);
}

LONG CALLBACK startupVectoredExceptionHandler(EXCEPTION_POINTERS* ex) {
    if (!ex || !ex->ExceptionRecord) {
        return EXCEPTION_CONTINUE_SEARCH;
    }

    unsigned long code = ex->ExceptionRecord->ExceptionCode;
    void* addr = ex->ExceptionRecord->ExceptionAddress;
    std::string module = modulePathForAddress(addr);
    unsigned long tid = static_cast<unsigned long>(GetCurrentThreadId());

    // Log only likely-fatal runtime faults to avoid noisy first-chance spam.
    switch (code) {
        case EXCEPTION_ACCESS_VIOLATION:
        case EXCEPTION_STACK_OVERFLOW:
        case EXCEPTION_ILLEGAL_INSTRUCTION:
        case EXCEPTION_IN_PAGE_ERROR:
        case EXCEPTION_INT_DIVIDE_BY_ZERO:
        case 0xC0000409: // STATUS_STACK_BUFFER_OVERRUN / fast-fail
        case 0xC0000374: // STATUS_HEAP_CORRUPTION
            writeStartupLog("Vectored exception: code=0x%08lX addr=%p module=%s tid=%lu",
                            code, addr, module.c_str(), tid);
            writeCrashDump(ex, "veh");
            break;
        default:
            break;
    }

    return EXCEPTION_CONTINUE_SEARCH;
}

void startupInvalidParameterHandler(const wchar_t* expression,
                                    const wchar_t* function,
                                    const wchar_t* file,
                                    unsigned int line,
                                    uintptr_t) {
    writeStartupLog("CRT invalid parameter: line=%u expr=%p func=%p file=%p",
                    line,
                    static_cast<const void*>(expression),
                    static_cast<const void*>(function),
                    static_cast<const void*>(file));
    writeCrashDump(nullptr, "invalid_param");
}

void __cdecl startupPurecallHandler() {
    writeStartupLog("CRT pure virtual call detected");
    writeCrashDump(nullptr, "purecall");
}

void startupSignalHandler(int sig) {
    writeStartupLog("Signal raised: %d", sig);
    writeCrashDump(nullptr, "signal");
    std::_Exit(3);
}

LONG WINAPI startupUnhandledExceptionFilter(EXCEPTION_POINTERS* ex) {
    unsigned long code = 0;
    void* addr = nullptr;
    if (ex && ex->ExceptionRecord) {
        code = ex->ExceptionRecord->ExceptionCode;
        addr = ex->ExceptionRecord->ExceptionAddress;
    }
    std::string module = modulePathForAddress(addr);
    writeStartupLog("Unhandled exception: code=0x%08lX address=%p module=%s", code, addr, module.c_str());
    writeCrashDump(ex, "unhandled");

    std::string msg = "Unhandled exception in startup path.\n";
    char details[512];
    std::snprintf(details, sizeof(details), "code=0x%08lX address=%p\nmodule=%s", code, addr, module.c_str());
    msg += details;
    if (!g_startup_log_path.empty()) {
        msg += "\n\nStartup log: " + g_startup_log_path;
    }
    showFatalStartupMessage(msg);
    closeStartupLog();
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

}  // namespace

int main(int argc, char* argv[]) {
    initStartupLog();
#ifdef _WIN32
    SetErrorMode(SEM_NOGPFAULTERRORBOX | SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX);
    SetUnhandledExceptionFilter(startupUnhandledExceptionFilter);
    if (!g_vectored_handler) {
        g_vectored_handler = AddVectoredExceptionHandler(1, startupVectoredExceptionHandler);
    }
    writeStartupLog("Vectored exception handler: %s", g_vectored_handler ? "installed" : "not installed");
    _set_invalid_parameter_handler(startupInvalidParameterHandler);
    _set_purecall_handler(startupPurecallHandler);
    signal(SIGABRT, startupSignalHandler);
    signal(SIGSEGV, startupSignalHandler);
    signal(SIGILL, startupSignalHandler);
    signal(SIGFPE, startupSignalHandler);
    signal(SIGTERM, startupSignalHandler);
    writeStartupLog("CRT crash handlers installed");
#endif
    std::set_terminate([]() {
        writeStartupLog("std::terminate invoked");
#ifdef _WIN32
        std::string msg = "Fatal terminate() during startup/runtime.";
        if (!g_startup_log_path.empty()) {
            msg += "\n\nStartup log: " + g_startup_log_path;
        }
        showFatalStartupMessage(msg);
#endif
        closeStartupLog();
        std::_Exit(3);
    });

    // Keep the default console profile operator-facing; debug/trace
    // modem internals are opt-in through the CLI flags below.
    ultra::setOperatorLogProfile();
    ultra::setLogLevel(ultra::LogLevel::INFO);
    writeStartupLog("Log level set to INFO, operator profile active");

    // Parse command line arguments
    ultra::gui::App::Options opts;
    struct LogFileCloser {
        void operator()(std::FILE* file) const {
            if (file) std::fclose(file);
        }
    };
    std::unique_ptr<std::FILE, LogFileCloser> log_file(nullptr);
    ultra::LogLevel log_level = ultra::LogLevel::INFO;
    bool log_level_set = false;
    bool log_categories_set = false;
    std::string log_categories;
    std::string log_file_path;
    bool auto_reply_after_messages_set = false;
#ifdef _WIN32
    bool force_software_renderer = true;   // Win default: conservative until the GL probe below
#else
    bool force_software_renderer = false;
#endif
    [[maybe_unused]] bool renderer_explicitly_chosen = false;  // true if --software/--opengl -> skip GL probe (read only on _WIN32)
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-sim") {
            opts.enable_sim = true;
        } else if (arg == "--ota-host") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --ota-host\n");
                closeStartupLog();
                return 1;
            }
            opts.ota_host = argv[++i];
        } else if (arg == "--ota-udp-host") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --ota-udp-host\n");
                closeStartupLog();
                return 1;
            }
            opts.ota_udp_host = argv[++i];
        } else if (arg == "--token") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --token\n");
                closeStartupLog();
                return 1;
            }
            opts.token = argv[++i];
        } else if (arg == "--station-id") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --station-id\n");
                closeStartupLog();
                return 1;
            }
            opts.station_id = argv[++i];
        } else if (arg == "--session-id") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --session-id\n");
                closeStartupLog();
                return 1;
            }
            opts.session_id = argv[++i];
        } else if (arg == "--monitor-audio") {
            opts.monitor_audio = true;
        } else if (arg == "--monitor-device") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --monitor-device\n");
                closeStartupLog();
                return 1;
            }
            opts.monitor_device = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printGuiUsage(argv[0]);
            closeStartupLog();
            return 0;
        } else if (arg == "--version") {
            std::printf("ProjectUltra %s commit=%s dirty=%s tag=%s built=%s os=%s\n",
                        ultra::kBuildVersion,
                        ultra::kBuildGitCommit,
                        ultra::kBuildDirty ? "true" : "false",
                        ultra::kBuildReleaseTag[0] ? ultra::kBuildReleaseTag : "none",
                        ultra::kBuildTimeUtc,
                        ultra::kBuildOS);
            closeStartupLog();
            return 0;
        } else if (arg == "-rec") {
            opts.record_audio = true;
            // Check if next arg is a path (doesn't start with -)
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                opts.record_path = argv[++i];
            }
        } else if (arg == "--software" || arg == "-sw") {
            force_software_renderer = true;
            opts.safe_startup = true;
            opts.disable_waterfall = true;
            renderer_explicitly_chosen = true;
        } else if (arg == "--opengl" || arg == "--gl") {
            force_software_renderer = false;
            opts.safe_startup = false;
            opts.disable_waterfall = false;
            renderer_explicitly_chosen = true;
        } else if (arg == "--no-waterfall") {
            opts.disable_waterfall = true;
        } else if (arg == "--waterfall") {
            opts.disable_waterfall = false;
        } else if (arg == "--monitor-ofdm") {
            opts.monitor_mode = "ofdm_chirp";
            // Optional next arg: rate. Default r1_4.
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                opts.monitor_rate = argv[++i];
            }
        } else if (arg == "--monitor-ofdm-narrow") {
            opts.monitor_mode = "ofdm_narrow";
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                opts.monitor_rate = argv[++i];
            }
        } else if (arg == "--monitor-mcdpsk") {
            opts.monitor_mode = "mc_dpsk";
        } else if (arg == "--monitor-mod") {
            if (i + 1 < argc) {
                opts.monitor_modulation = argv[++i];
            }
        } else if (arg == "--auto-connect") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-connect\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_connect = argv[++i];
        } else if (arg == "--connect-delay") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --connect-delay\n");
                closeStartupLog();
                return 1;
            }
            opts.connect_delay_sec = std::atoi(argv[++i]);
        } else if (arg == "--auto-accept") {
            opts.auto_accept = true;
        } else if (arg == "--auto-send-file") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-send-file\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_send_file = argv[++i];
        } else if (arg == "--auto-send-message") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-send-message\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_send_message = argv[++i];
        } else if (arg == "--auto-reply-message") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-reply-message\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_reply_message = argv[++i];
        } else if (arg == "--auto-reply-after-messages") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-reply-after-messages\n");
                closeStartupLog();
                return 1;
            }
            const char* value = argv[++i];
            char* end = nullptr;
            const long parsed = std::strtol(value, &end, 10);
            if (!value[0] || !end || *end != '\0' || parsed < 1 || parsed > 1000000L) {
                std::fprintf(stderr,
                             "Invalid --auto-reply-after-messages: %s (expected 1..1000000)\n",
                             value);
                closeStartupLog();
                return 1;
            }
            opts.auto_reply_after_messages = static_cast<int>(parsed);
            auto_reply_after_messages_set = true;
        } else if (arg == "--auto-message-start-delay") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-message-start-delay\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_message_start_delay_sec = std::atoi(argv[++i]);
        } else if (arg == "--auto-message-count") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-message-count\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_message_count = std::atoi(argv[++i]);
        } else if (arg == "--auto-message-interval") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-message-interval\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_message_interval_sec = std::atoi(argv[++i]);
        } else if (arg == "--auto-message-vary-len") {
            opts.auto_message_vary_len = true;
        } else if (arg == "--auto-message-after-file") {
            opts.auto_message_after_file = true;
        } else if (arg == "--auto-cancel-file-after") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-cancel-file-after\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_cancel_file_after_sec = std::atoi(argv[++i]);
        } else if (arg == "--disconnect-on-file-done") {
            opts.disconnect_on_file_done = true;
        } else if (arg == "--auto-disconnect-after") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --auto-disconnect-after\n");
                closeStartupLog();
                return 1;
            }
            opts.auto_disconnect_after_sec = std::atoi(argv[++i]);
        } else if (arg == "--exit-after") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --exit-after\n");
                closeStartupLog();
                return 1;
            }
            opts.exit_after_sec = std::atoi(argv[++i]);
        } else if (arg == "--half-duplex") {
            opts.half_duplex_interactive = true;
        } else if (arg == "--log-level") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --log-level\n");
                closeStartupLog();
                return 1;
            }
            const std::string value = argv[++i];
            if (!ultra::parseLogLevel(value, log_level)) {
                std::fprintf(stderr, "Invalid --log-level: %s\n", value.c_str());
                closeStartupLog();
                return 1;
            }
            log_level_set = true;
        } else if (arg == "--log-category" || arg == "--log-categories") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for %s\n", arg.c_str());
                closeStartupLog();
                return 1;
            }
            log_categories = argv[++i];
            log_categories_set = true;
        } else if (arg == "--log-file") {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "Missing value for --log-file\n");
                closeStartupLog();
                return 1;
            }
            log_file_path = argv[++i];
        }
    }

    if (auto_reply_after_messages_set && opts.auto_reply_message.empty()) {
        std::fprintf(stderr,
                     "--auto-reply-after-messages requires --auto-reply-message\n");
        closeStartupLog();
        return 1;
    }

    ultra::setLogLevel(log_level);
    if (log_level_set && log_level >= ultra::LogLevel::DEBUG && !log_categories_set) {
        ultra::setDeveloperLogProfile();
    }
    if (log_categories_set && !ultra::setLogCategories(log_categories)) {
        std::fprintf(stderr, "Invalid --log-category list: %s\n", log_categories.c_str());
        closeStartupLog();
        return 1;
    }
    if (!log_file_path.empty()) {
        log_file.reset(std::fopen(log_file_path.c_str(), "a"));
        if (!log_file) {
            std::fprintf(stderr, "Failed to open --log-file %s\n", log_file_path.c_str());
            closeStartupLog();
            return 1;
        }
        ultra::setLogFile(log_file.get());
    }
    LOG_INFO("OPERATOR", "ultra_gui starting: log=%s", ultra::logLevelName(log_level));

    if (opts.enable_sim) {
        if (opts.ota_host.empty() || opts.token.empty() || opts.station_id.empty()) {
            std::fprintf(stderr,
                         "Error: -sim requires --ota-host <host:port>, --token <token>, and --station-id <id>\n");
            closeStartupLog();
            return 1;
        }
        if (opts.session_id.empty()) {
            opts.session_id = "lobby";
        }
    }

    // Software path implies safer startup defaults (deferred audio + no waterfall)
    if (force_software_renderer) {
        opts.safe_startup = true;
        opts.disable_waterfall = true;
    }
    writeStartupLog(
        "Parsed arguments: sim=%d, rec=%d, software_renderer=%d, disable_waterfall=%d, ota_host=%s, station_id=%s, session_id=%s",
        opts.enable_sim ? 1 : 0,
        opts.record_audio ? 1 : 0,
        force_software_renderer ? 1 : 0,
        opts.disable_waterfall ? 1 : 0,
        opts.ota_host.c_str(),
        opts.station_id.c_str(),
        opts.session_id.c_str()
    );

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_TIMER) != 0) {
        const char* sdl_err = SDL_GetError();
        std::string msg = std::string("SDL_Init failed: ") + (sdl_err ? sdl_err : "<unknown>");
#ifndef _WIN32
        std::fprintf(stderr, "Error: %s\n", msg.c_str());
#endif
        writeStartupLog("%s", msg.c_str());
#ifdef _WIN32
        if (!g_startup_log_path.empty()) {
            showFatalStartupMessage(msg + "\n\nStartup log: " + g_startup_log_path);
        } else {
            showFatalStartupMessage(msg);
        }
#endif
        closeStartupLog();
        return 1;
    }
    writeStartupLog("SDL initialized");

#ifdef _WIN32
    // GL CAPABILITY PROBE (Windows). The old default assumed software on every Windows machine
    // (which disables the waterfall). Instead, decide by ACTUAL capability: spin up a tiny hidden
    // window, try to create + make-current an OpenGL context, tear it down. If it works, take the
    // OpenGL path (waterfall ENABLED); if not, fall back to the SDL_Renderer software path
    // (waterfall disabled) — the safety net for VMs / RDP / no-GPU boxes. Skipped when the operator
    // explicitly chose --software / --opengl.
    if (!renderer_explicitly_chosen) {
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
        bool gl_ok = false;
        SDL_Window* probe_win = SDL_CreateWindow(
            "gl-probe", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 16, 16,
            SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
        if (probe_win) {
            SDL_GLContext probe_ctx = SDL_GL_CreateContext(probe_win);
            if (probe_ctx) {
                gl_ok = (SDL_GL_MakeCurrent(probe_win, probe_ctx) == 0);
                SDL_GL_MakeCurrent(probe_win, nullptr);
                SDL_GL_DeleteContext(probe_ctx);
            }
            SDL_DestroyWindow(probe_win);
        }
        writeStartupLog("Windows GL probe: %s",
                        gl_ok ? "OpenGL available -> GL renderer + waterfall ENABLED"
                              : "OpenGL unavailable -> software renderer, waterfall disabled");
        force_software_renderer = !gl_ok;
        if (gl_ok) {
            opts.disable_waterfall = false;  // override the conservative pre-SDL default
            opts.safe_startup = false;
        }
    }
#endif

    SDL_Window* window = nullptr;
    SDL_GLContext gl_context = nullptr;
    SDL_Renderer* sdl_renderer = nullptr;
    bool using_software_renderer = force_software_renderer;

    auto failStartup = [&](const std::string& msg) -> int {
#ifndef _WIN32
        std::fprintf(stderr, "Error: %s\n", msg.c_str());
#endif
        writeStartupLog("%s", msg.c_str());
#ifdef _WIN32
        if (!g_startup_log_path.empty()) {
            showFatalStartupMessage(msg + "\n\nStartup log: " + g_startup_log_path);
        } else {
            showFatalStartupMessage(msg);
        }
#endif
        if (sdl_renderer) {
            SDL_DestroyRenderer(sdl_renderer);
            sdl_renderer = nullptr;
        }
        if (gl_context) {
            SDL_GL_DeleteContext(gl_context);
            gl_context = nullptr;
        }
        if (window) {
            SDL_DestroyWindow(window);
            window = nullptr;
        }
        SDL_Quit();
        closeStartupLog();
        return 1;
    };

    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!using_software_renderer) {
        // Setup OpenGL 2.1 context (works on old hardware if driver is stable)
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
        window_flags = (SDL_WindowFlags)(window_flags | SDL_WINDOW_OPENGL);
        writeStartupLog("OpenGL renderer path selected");
    } else {
        writeStartupLog("Software renderer path selected (--software)");
    }

    window = SDL_CreateWindow(
        "ProjectUltra - High-Speed HF Modem",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1024, 600,
        window_flags
    );

    if (!window) {
        const char* sdl_err = SDL_GetError();
        std::string msg = std::string("SDL_CreateWindow failed: ") + (sdl_err ? sdl_err : "<unknown>");
        return failStartup(msg);
    }
    writeStartupLog("Window created");

    if (using_software_renderer) {
        writeStartupLog("Creating SDL renderer (accelerated + vsync)");
        sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!sdl_renderer) {
            writeStartupLog("Accelerated SDL renderer unavailable: %s", SDL_GetError());
            writeStartupLog("Falling back to SDL software renderer");
            sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
        }
        if (!sdl_renderer) {
            const char* sdl_err = SDL_GetError();
            std::string msg = std::string("SDL_CreateRenderer failed: ") + (sdl_err ? sdl_err : "<unknown>");
            return failStartup(msg);
        }

        SDL_RendererInfo renderer_info{};
        if (SDL_GetRendererInfo(sdl_renderer, &renderer_info) == 0) {
            writeStartupLog(
                "SDL renderer ready: name=%s flags=0x%x",
                renderer_info.name ? renderer_info.name : "<unknown>",
                renderer_info.flags
            );
        } else {
            writeStartupLog("SDL renderer ready (SDL_GetRendererInfo failed: %s)", SDL_GetError());
        }
    } else {
        gl_context = SDL_GL_CreateContext(window);
        if (!gl_context) {
            const char* sdl_err = SDL_GetError();
            std::string msg = std::string("SDL_GL_CreateContext failed: ") + (sdl_err ? sdl_err : "<unknown>");
            return failStartup(msg);
        }
        writeStartupLog("OpenGL context created");

        writeStartupLog("Calling SDL_GL_MakeCurrent");
        if (SDL_GL_MakeCurrent(window, gl_context) != 0) {
            const char* sdl_err = SDL_GetError();
            std::string msg = std::string("SDL_GL_MakeCurrent failed: ") + (sdl_err ? sdl_err : "<unknown>");
            return failStartup(msg);
        }
        writeStartupLog("SDL_GL_MakeCurrent succeeded");

        writeStartupLog("Calling SDL_GL_SetSwapInterval(1)");
        if (SDL_GL_SetSwapInterval(1) != 0) {
            // Non-fatal on some drivers; keep running but record detail.
            writeStartupLog("SDL_GL_SetSwapInterval failed/non-vsync: %s", SDL_GetError());
        } else {
            writeStartupLog("SDL_GL_SetSwapInterval succeeded");
        }
    }

    // Setup Dear ImGui context
    writeStartupLog("Calling IMGUI_CHECKVERSION");
    IMGUI_CHECKVERSION();
    writeStartupLog("IMGUI_CHECKVERSION passed");
    writeStartupLog("Calling ImGui::CreateContext");
    ImGui::CreateContext();
    writeStartupLog("ImGui::CreateContext succeeded");
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    writeStartupLog("ImGui IO initialized");

    // Setup style - dark theme
    writeStartupLog("Applying ImGui style");
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;
    writeStartupLog("ImGui style applied");

    // Setup Platform/Renderer backends
    if (using_software_renderer) {
        writeStartupLog("Calling ImGui_ImplSDL2_InitForSDLRenderer");
        if (!ImGui_ImplSDL2_InitForSDLRenderer(window, sdl_renderer)) {
            return failStartup("ImGui_ImplSDL2_InitForSDLRenderer failed");
        }
        writeStartupLog("ImGui_ImplSDL2_InitForSDLRenderer succeeded");

        writeStartupLog("Calling ImGui_ImplSDLRenderer2_Init");
        if (!ImGui_ImplSDLRenderer2_Init(sdl_renderer)) {
            return failStartup("ImGui_ImplSDLRenderer2_Init failed");
        }
        writeStartupLog("ImGui_ImplSDLRenderer2_Init succeeded");
    } else {
        writeStartupLog("Calling ImGui_ImplSDL2_InitForOpenGL");
        if (!ImGui_ImplSDL2_InitForOpenGL(window, gl_context)) {
            return failStartup("ImGui_ImplSDL2_InitForOpenGL failed");
        }
        writeStartupLog("ImGui_ImplSDL2_InitForOpenGL succeeded");

        writeStartupLog("Calling ImGui_ImplOpenGL2_Init");
        if (!ImGui_ImplOpenGL2_Init()) {
            return failStartup("ImGui_ImplOpenGL2_Init failed");
        }
        writeStartupLog("ImGui_ImplOpenGL2_Init succeeded");
    }

    // Create application with parsed options
    writeStartupLog("Constructing App");
    ultra::gui::App app(opts);
    writeStartupLog("App initialized");

    // Main loop
    bool running = true;
    bool first_frame = true;
    while (running) {
        if (first_frame) {
        }
        // Poll events
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);

            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                running = false;
            }
        }

        // Start ImGui frame
        if (first_frame) {
        }
        if (using_software_renderer) {
            ImGui_ImplSDLRenderer2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
        } else {
            ImGui_ImplOpenGL2_NewFrame();
            ImGui_ImplSDL2_NewFrame();
        }
        ImGui::NewFrame();
        if (first_frame) {
        }

        // Render application UI
        if (first_frame) {
        }
        app.render();
        if (first_frame) {
        }

        // Rendering
        if (first_frame) {
        }
        ImGui::Render();
        if (using_software_renderer) {
            SDL_SetRenderDrawColor(sdl_renderer, 26, 26, 31, 255);
            SDL_RenderClear(sdl_renderer);
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), sdl_renderer);
            SDL_RenderPresent(sdl_renderer);
        } else {
            glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
            glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
            SDL_GL_SwapWindow(window);
        }
        if (first_frame) {
            first_frame = false;
        }
    }

    // Cleanup
    if (using_software_renderer) {
        ImGui_ImplSDLRenderer2_Shutdown();
    } else {
        ImGui_ImplOpenGL2_Shutdown();
    }
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    if (sdl_renderer) {
        SDL_DestroyRenderer(sdl_renderer);
        sdl_renderer = nullptr;
    }
    if (gl_context) {
        SDL_GL_DeleteContext(gl_context);
        gl_context = nullptr;
    }
    SDL_DestroyWindow(window);
    SDL_Quit();
#ifdef _WIN32
    if (g_vectored_handler) {
        RemoveVectoredExceptionHandler(g_vectored_handler);
        g_vectored_handler = nullptr;
    }
#endif
    closeStartupLog();

    return 0;
}
