#pragma once

#include <cstdio>
#include <cstdarg>

namespace ultra {

// Log levels
enum class LogLevel {
    NONE = 0,
    ERROR = 1,
    WARN = 2,
    INFO = 3,
    DEBUG = 4,
    TRACE = 5
};

// Global log level - can be changed at runtime
// Default to INFO for release, DEBUG for debug builds
#ifdef NDEBUG
inline LogLevel g_log_level = LogLevel::INFO;
#else
inline LogLevel g_log_level = LogLevel::DEBUG;
#endif

// Log category enable flags for fine-grained control
struct LogCategories {
    bool demod = true;      // Demodulator
    bool modem = true;      // Modem engine
    bool ldpc = false;      // LDPC codec (very verbose)
    bool sync = true;       // Sync detection
    bool channel = false;   // Channel estimation
};

inline LogCategories g_log_categories;

// Set log level
inline void setLogLevel(LogLevel level) {
    g_log_level = level;
}

// Core logging function
inline void log(LogLevel level, const char* category, const char* format, ...) {
    if (level > g_log_level) return;

    const char* level_str = "";
    switch (level) {
        case LogLevel::ERROR: level_str = "ERROR"; break;
        case LogLevel::WARN:  level_str = "WARN "; break;
        case LogLevel::INFO:  level_str = "INFO "; break;
        case LogLevel::DEBUG: level_str = "DEBUG"; break;
        case LogLevel::TRACE: level_str = "TRACE"; break;
        default: break;
    }

    fprintf(stderr, "[%s][%s] ", level_str, category);

    va_list args;
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);

    fprintf(stderr, "\n");
}

// Convenience macros - these compile to nothing when ULTRA_LOG_DISABLE is defined
#ifdef ULTRA_LOG_DISABLE

#define LOG_ERROR(cat, fmt, ...)
#define LOG_WARN(cat, fmt, ...)
#define LOG_INFO(cat, fmt, ...)
#define LOG_DEBUG(cat, fmt, ...)
#define LOG_TRACE(cat, fmt, ...)

#else

#define LOG_ERROR(cat, fmt, ...) \
    ultra::log(ultra::LogLevel::ERROR, cat, fmt, ##__VA_ARGS__)

#define LOG_WARN(cat, fmt, ...) \
    ultra::log(ultra::LogLevel::WARN, cat, fmt, ##__VA_ARGS__)

#define LOG_INFO(cat, fmt, ...) \
    ultra::log(ultra::LogLevel::INFO, cat, fmt, ##__VA_ARGS__)

#define LOG_DEBUG(cat, fmt, ...) \
    do { if (ultra::g_log_level >= ultra::LogLevel::DEBUG) \
        ultra::log(ultra::LogLevel::DEBUG, cat, fmt, ##__VA_ARGS__); } while(0)

#define LOG_TRACE(cat, fmt, ...) \
    do { if (ultra::g_log_level >= ultra::LogLevel::TRACE) \
        ultra::log(ultra::LogLevel::TRACE, cat, fmt, ##__VA_ARGS__); } while(0)

#endif

// Category-specific logging macros
#define LOG_DEMOD(level, fmt, ...) \
    do { if (ultra::g_log_categories.demod) LOG_##level("DEMOD", fmt, ##__VA_ARGS__); } while(0)

#define LOG_MODEM(level, fmt, ...) \
    do { if (ultra::g_log_categories.modem) LOG_##level("MODEM", fmt, ##__VA_ARGS__); } while(0)

#define LOG_LDPC(level, fmt, ...) \
    do { if (ultra::g_log_categories.ldpc) LOG_##level("LDPC", fmt, ##__VA_ARGS__); } while(0)

#define LOG_SYNC(level, fmt, ...) \
    do { if (ultra::g_log_categories.sync) LOG_##level("SYNC", fmt, ##__VA_ARGS__); } while(0)

#define LOG_CHAN(level, fmt, ...) \
    do { if (ultra::g_log_categories.channel) LOG_##level("CHAN", fmt, ##__VA_ARGS__); } while(0)

} // namespace ultra
