// Unit tests for ultra_tnc config plumbing. Exercises the strict
// parsers, applyConfigKey branches, loadConfigFile error paths, and
// parseArgs (CLI-overrides-config, --help skips bad config, etc.)
// without needing audio, PTT, or the protocol engine.

#include "ultra_tnc_config.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ultra::tnc::config;

namespace {

int passed = 0;
int failed = 0;

#define CHECK(cond, msg)                              \
    do {                                              \
        if (!(cond)) {                                \
            std::cout << "FAIL: " << msg << "\n";     \
            ++failed;                                 \
            return;                                   \
        }                                             \
    } while (0)

void pass(const char* name) {
    ++passed;
    std::cout << "PASS: " << name << "\n";
}

// argv builder for parseArgs tests. parseArgs takes char**, so we
// keep the strings alive in a vector.
struct Argv {
    std::vector<std::string> storage;
    std::vector<char*> argv;
    Argv(std::initializer_list<const char*> tokens) {
        storage.reserve(tokens.size());
        for (const char* t : tokens) storage.emplace_back(t);
        argv.reserve(storage.size());
        for (auto& s : storage) argv.push_back(s.data());
    }
    int argc() const { return static_cast<int>(argv.size()); }
    char** data() { return argv.data(); }
};

// Temp-file RAII for loadConfigFile / parseArgs --config tests.
// Cross-platform: uses std::filesystem::temp_directory_path() so the
// Windows CI runner doesn't need unistd.h / mkstemp.
struct TempFile {
    std::filesystem::path path;
    explicit TempFile(const std::string& contents) {
        static std::atomic<uint64_t> seq{0};
        const auto ns = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto id = seq.fetch_add(1, std::memory_order_relaxed);
        const std::string name =
            "ultra_tnc_cfg_" + std::to_string(ns) + "_" + std::to_string(id) + ".tmp";
        path = std::filesystem::temp_directory_path() / name;
        std::ofstream out(path);
        if (!out) throw std::runtime_error("temp file open failed");
        out << contents;
    }
    ~TempFile() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    std::string str() const { return path.string(); }
};

// ---------------- Strict parsers ----------------

void test_parsePositiveIntStrict_basic() {
    int v = -1;
    CHECK(parsePositiveIntStrict("9600", v), "9600 must parse");
    CHECK(v == 9600, "9600 value");
    pass("parsePositiveIntStrict: 9600 → 9600");
}

void test_parsePositiveIntStrict_rejects_trailing_garbage() {
    int v = -1;
    CHECK(!parsePositiveIntStrict("9600abc", v), "9600abc must reject");
    pass("parsePositiveIntStrict: '9600abc' rejected (was loose-stoi accepting it)");
}

void test_parsePositiveIntStrict_rejects_negative() {
    int v = -1;
    CHECK(!parsePositiveIntStrict("-50", v), "-50 must reject");
    pass("parsePositiveIntStrict: negative rejected");
}

void test_parsePositiveIntStrict_rejects_plus_sign() {
    int v = -1;
    CHECK(!parsePositiveIntStrict("+50", v), "+50 must reject");
    pass("parsePositiveIntStrict: leading '+' rejected");
}

void test_parsePositiveIntStrict_range_check() {
    int v = -1;
    CHECK(!parsePositiveIntStrict("49", v, 50, 4000000), "49 below min must reject");
    CHECK(!parsePositiveIntStrict("4000001", v, 50, 4000000), "above max must reject");
    CHECK(parsePositiveIntStrict("50", v, 50, 4000000), "50 at min ok");
    CHECK(parsePositiveIntStrict("4000000", v, 50, 4000000), "max ok");
    pass("parsePositiveIntStrict: range bounds enforced");
}

void test_parsePositiveIntStrict_empty_rejected() {
    int v = -1;
    CHECK(!parsePositiveIntStrict("", v), "empty must reject");
    pass("parsePositiveIntStrict: empty string rejected");
}

void test_parseBoolStrict_truthy_forms() {
    bool b = false;
    CHECK(parseBoolStrict("true", b) && b == true, "true");
    CHECK(parseBoolStrict("TRUE", b) && b == true, "TRUE (case-insensitive)");
    CHECK(parseBoolStrict("1", b) && b == true, "1");
    CHECK(parseBoolStrict("yes", b) && b == true, "yes");
    CHECK(parseBoolStrict("on", b) && b == true, "on");
    pass("parseBoolStrict: truthy forms (true/TRUE/1/yes/on)");
}

void test_parseBoolStrict_falsy_forms() {
    bool b = true;
    CHECK(parseBoolStrict("false", b) && b == false, "false");
    CHECK(parseBoolStrict("0", b) && b == false, "0");
    CHECK(parseBoolStrict("no", b) && b == false, "no");
    CHECK(parseBoolStrict("off", b) && b == false, "off");
    pass("parseBoolStrict: falsy forms (false/0/no/off)");
}

void test_parseBoolStrict_unknown_rejected() {
    bool b = false;
    CHECK(!parseBoolStrict("maybe", b), "maybe must reject (was silently → false)");
    CHECK(!parseBoolStrict("", b), "empty must reject");
    pass("parseBoolStrict: 'maybe' / empty rejected");
}

void test_parseUint16_bounds() {
    uint16_t v = 0;
    CHECK(parseUint16("0", v) && v == 0, "0");
    CHECK(parseUint16("65535", v) && v == 65535, "65535");
    CHECK(!parseUint16("65536", v), ">65535 rejected");
    CHECK(!parseUint16("-1", v), "negative rejected");
    CHECK(!parseUint16("8300abc", v), "trailing garbage rejected");
    pass("parseUint16: range + garbage rejection");
}

void test_parseFloat_strict() {
    auto a = parseFloat("3.14");
    CHECK(a && std::abs(*a - 3.14f) < 1e-5f, "3.14 ok");
    auto b = parseFloat("3.14abc");
    CHECK(!b, "trailing garbage rejected");
    pass("parseFloat: 3.14 ok, '3.14abc' rejected");
}

void test_parseCodeRate() {
    CHECK(parseCodeRate("auto") && *parseCodeRate("auto") == ultra::CodeRate::AUTO, "auto");
    CHECK(parseCodeRate("R1_2") && *parseCodeRate("R1_2") == ultra::CodeRate::R1_2, "R1_2 case-insensitive");
    CHECK(!parseCodeRate("r5_8"), "unknown rate rejected");
    pass("parseCodeRate: known rates accepted, unknown rejected");
}

void test_parseModulation() {
    CHECK(parseModulation("dqpsk") && *parseModulation("dqpsk") == ultra::Modulation::DQPSK, "dqpsk");
    CHECK(!parseModulation("QAM64"), "QAM64 requires expert mode");
    CHECK(parseModulation("QAM64", true) && *parseModulation("QAM64", true) == ultra::Modulation::QAM64,
          "QAM64 accepted in expert mode");
    CHECK(!parseModulation("nonsense"), "unknown rejected");
    pass("parseModulation: operator mods accepted, expert-only modes guarded");
}

void test_isNoneDevice() {
    CHECK(isNoneDevice("none"), "lowercase none");
    CHECK(isNoneDevice("NONE"), "uppercase NONE");
    CHECK(isNoneDevice("None"), "mixed case None");
    CHECK(!isNoneDevice("USB Audio"), "real device name");
    pass("isNoneDevice: case-insensitive 'none' detection");
}

// ---------------- applyConfigKey ----------------

void test_applyConfigKey_audio_devices() {
    Config cfg;
    CHECK(applyConfigKey("audio_output", "USB Audio CODEC", cfg), "audio_output ok");
    CHECK(cfg.audio_output == "USB Audio CODEC", "value stored");
    CHECK(applyConfigKey("audio-input", "USB Audio CODEC", cfg), "audio-input alias ok");
    CHECK(cfg.audio_input == "USB Audio CODEC", "input value stored");
    pass("applyConfigKey: audio_output/audio-input both spellings");
}

void test_applyConfigKey_sim_audio() {
    Config cfg;
    CHECK(applyConfigKey("sim_audio", "true", cfg) && cfg.sim_audio,
          "sim_audio=true should enable OTASim audio");
    CHECK(applyConfigKey("ota_host", "127.0.0.1:47000", cfg), "ota_host ok");
    CHECK(applyConfigKey("ota_udp_host", "127.0.0.1:47001", cfg), "ota_udp_host ok");
    CHECK(applyConfigKey("token", "test_token", cfg), "token ok");
    CHECK(applyConfigKey("station_id", "alice", cfg), "station_id ok");
    CHECK(applyConfigKey("session_id", "lobby", cfg), "session_id ok");
    CHECK(cfg.ota_host == "127.0.0.1:47000", "ota_host stored");
    CHECK(cfg.ota_udp_host == "127.0.0.1:47001", "ota_udp_host stored");
    CHECK(cfg.token == "test_token", "token stored");
    CHECK(cfg.station_id == "alice", "station_id stored");
    CHECK(cfg.session_id == "lobby", "session_id stored");
    CHECK(!applyConfigKey("sim_audio", "maybe", cfg), "bad sim_audio bool rejected");
    CHECK(!applyConfigKey("ota_host", "", cfg), "empty ota_host rejected");
    CHECK(!applyConfigKey("token", "", cfg), "empty token rejected");
    CHECK(!applyConfigKey("station_id", "", cfg), "empty station_id rejected");
    CHECK(!applyConfigKey("session_id", "", cfg), "empty session_id rejected");
    pass("applyConfigKey: OTASim audio keys parsed strictly");
}

void test_applyConfigKey_port_bounds() {
    Config cfg;
    CHECK(applyConfigKey("port", "8300", cfg), "8300 ok");
    CHECK(cfg.port == 8300, "port stored");
    CHECK(!applyConfigKey("port", "65535", cfg), "65535 leaves no room for data port");
    CHECK(!applyConfigKey("port", "-1", cfg), "negative port rejected");
    pass("applyConfigKey: port bounded to leave room for N+1 data port");
}

void test_applyConfigKey_callsign_sanitized() {
    Config cfg;
    CHECK(applyConfigKey("callsign", "n0call", cfg), "n0call ok");
    CHECK(cfg.callsign == "N0CALL", "sanitized to uppercase");
    CHECK(!applyConfigKey("callsign", "", cfg), "empty rejected");
    pass("applyConfigKey: callsign sanitized + empty rejected");
}

void test_applyConfigKey_inject_channel_modes() {
    Config cfg;
    CHECK(applyConfigKey("inject_channel", "true", cfg) && cfg.inject_channel,
          "inject_channel=true");
    CHECK(applyConfigKey("inject_channel", "false", cfg) && !cfg.inject_channel,
          "inject_channel=false");
    CHECK(applyConfigKey("inject_channel", "awgn", cfg) && cfg.inject_channel,
          "awgn synonym for true");
    CHECK(cfg.inject_channel_type == "awgn", "channel type recorded");
    CHECK(!applyConfigKey("inject_channel", "rayleigh", cfg),
          "non-AWGN channel rejected (only AWGN implemented)");
    pass("applyConfigKey: inject_channel accepts bool + 'awgn', rejects others");
}

void test_applyConfigKey_ptt_baud_strict() {
    Config cfg;
    CHECK(applyConfigKey("ptt_serial_baud", "9600", cfg) && cfg.ptt_serial_baud == 9600,
          "9600 baud ok");
    CHECK(!applyConfigKey("ptt_serial_baud", "9600abc", cfg),
          "trailing garbage rejected (was silently → 9600)");
    CHECK(!applyConfigKey("ptt_serial_baud", "-50", cfg),
          "negative baud rejected (was silently → -50)");
    CHECK(!applyConfigKey("ptt_serial_baud", "49", cfg), "below min rejected");
    pass("applyConfigKey: ptt_serial_baud strict-parsed");
}

void test_applyConfigKey_ptt_inactive_high_strict() {
    Config cfg;
    CHECK(applyConfigKey("ptt_inactive_high", "true", cfg) && cfg.ptt_inactive_high,
          "true ok");
    CHECK(applyConfigKey("ptt_inactive_high", "false", cfg) && !cfg.ptt_inactive_high,
          "false ok");
    CHECK(!applyConfigKey("ptt_inactive_high", "maybe", cfg),
          "maybe rejected (was silently → false)");
    pass("applyConfigKey: ptt_inactive_high strict-parsed");
}

void test_applyConfigKey_ptt_serial_line() {
    Config cfg;
    CHECK(applyConfigKey("ptt_serial_line", "rts", cfg), "rts ok");
    CHECK(cfg.ptt_serial_line == "rts", "rts stored");
    CHECK(applyConfigKey("ptt_serial_line", "DTR", cfg), "DTR (mixed case) ok");
    CHECK(cfg.ptt_serial_line == "dtr", "normalized to lowercase");
    CHECK(!applyConfigKey("ptt_serial_line", "tx", cfg), "unknown line rejected");
    pass("applyConfigKey: ptt_serial_line rts/dtr only");
}

void test_applyConfigKey_ptt_cat_settings() {
    Config cfg;
    CHECK(applyConfigKey("ptt_cat", "true", cfg) && cfg.ptt_cat,
          "ptt_cat=true should enable CAT");
    CHECK(applyConfigKey("ptt_cat_host", "localhost", cfg),
          "ptt_cat_host should parse");
    CHECK(cfg.ptt_cat_host == "localhost", "cat host stored");
    CHECK(applyConfigKey("ptt_cat_port", "4533", cfg),
          "ptt_cat_port should parse");
    CHECK(cfg.ptt_cat_port == 4533, "cat port stored");
    CHECK(!applyConfigKey("ptt_cat", "maybe", cfg), "bad bool rejected");
    CHECK(!applyConfigKey("ptt_cat_host", "", cfg), "empty host rejected");
    CHECK(!applyConfigKey("ptt_cat_port", "0", cfg), "port 0 rejected");
    CHECK(!applyConfigKey("ptt_cat_port", "65536", cfg), "port >65535 rejected");
    pass("applyConfigKey: ptt_cat/host/port parsed strictly");
}

void test_applyConfigKey_unknown_key_rejected() {
    Config cfg;
    CHECK(!applyConfigKey("nonsense_field", "x", cfg), "unknown key rejected");
    pass("applyConfigKey: unknown key rejected");
}

void test_applyConfigKey_ofdm_config() {
    Config cfg;
    CHECK(applyConfigKey("ofdm_config", "default", cfg) &&
          cfg.ofdm_config == OFDMConfigPreset::Default, "default");
    CHECK(applyConfigKey("ofdm_config", "nvis", cfg) &&
          cfg.ofdm_config == OFDMConfigPreset::Nvis, "nvis");
    CHECK(!applyConfigKey("ofdm_config", "wide", cfg), "unknown preset rejected");
    pass("applyConfigKey: ofdm_config default/nvis only");
}

void test_applyConfigKey_log_settings() {
    Config cfg;
    CHECK(applyConfigKey("log_level", "debug", cfg), "debug log level ok");
    CHECK(cfg.log_level == ultra::LogLevel::DEBUG && cfg.log_level_set, "debug stored");
    CHECK(!applyConfigKey("log_level", "chatty", cfg), "unknown log level rejected");
    CHECK(applyConfigKey("log_category", "operator,audio,demod", cfg), "log categories ok");
    CHECK(cfg.log_categories == "operator,audio,demod" && cfg.log_categories_set,
          "categories stored");
    CHECK(applyConfigKey("log_file", "/tmp/ultra.log", cfg), "log file ok");
    CHECK(cfg.log_file == "/tmp/ultra.log", "log file stored");
    pass("applyConfigKey: log_level/log_category/log_file");
}

// ---------------- loadConfigFile ----------------

void test_loadConfigFile_basic() {
    TempFile cfg_file(
        "# header comment\n"
        "audio_output = USB Audio CODEC\n"
        "audio_input  = USB Audio CODEC\n"
        "callsign     = N0CALL\n"
        "port         = 8400\n"
        "\n"
        "ptt_serial_port = /dev/cu.usbserial-FT001\n"
        "ptt_serial_line = dtr\n"
    );
    Config cfg;
    CHECK(loadConfigFile(cfg_file.str(), cfg), "valid config loads");
    CHECK(cfg.audio_output == "USB Audio CODEC", "audio_output");
    CHECK(cfg.callsign == "N0CALL", "callsign");
    CHECK(cfg.port == 8400, "port");
    CHECK(cfg.ptt_serial_port == "/dev/cu.usbserial-FT001", "ptt_serial_port");
    CHECK(cfg.ptt_serial_line == "dtr", "ptt_serial_line");
    pass("loadConfigFile: full valid file populates Config");
}

void test_loadConfigFile_missing_equals() {
    TempFile cfg_file("audio_output USB Audio\n");  // no '='
    Config cfg;
    CHECK(!loadConfigFile(cfg_file.str(), cfg), "missing '=' must reject");
    pass("loadConfigFile: missing '=' rejected");
}

void test_loadConfigFile_unknown_key() {
    TempFile cfg_file("nonsense = x\n");
    Config cfg;
    CHECK(!loadConfigFile(cfg_file.str(), cfg), "unknown key must reject");
    pass("loadConfigFile: unknown key rejected");
}

void test_loadConfigFile_bad_value() {
    TempFile cfg_file("ptt_serial_baud = abc\n");
    Config cfg;
    CHECK(!loadConfigFile(cfg_file.str(), cfg), "bad value must reject");
    pass("loadConfigFile: bad value rejected");
}

void test_loadConfigFile_missing_file() {
    Config cfg;
    CHECK(!loadConfigFile("/tmp/does_not_exist_ultra_tnc_cfg_zzzz.conf", cfg),
          "missing file must return false");
    pass("loadConfigFile: missing file rejected");
}

void test_loadConfigFile_comments_and_blank_lines() {
    TempFile cfg_file(
        "# comment-only line\n"
        "\n"
        "callsign = N0CALL  # trailing comment\n"
        "\n"
    );
    Config cfg;
    CHECK(loadConfigFile(cfg_file.str(), cfg), "comments + blanks ok");
    CHECK(cfg.callsign == "N0CALL", "trailing comment stripped");
    pass("loadConfigFile: comments and blank lines tolerated");
}

// ---------------- parseArgs ----------------

void test_parseArgs_help_flag() {
    Argv argv({"ultra_tnc", "--help"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "--help parses");
    CHECK(cfg.help, "help flag set");
    pass("parseArgs: --help sets cfg.help");
}

void test_parseArgs_help_skips_bad_config() {
    // Operator must be able to recover from a malformed config via
    // --help without the parser bailing out on config load.
    TempFile bad_cfg("totally not valid syntax\n");
    Argv argv({"ultra_tnc", "--config", bad_cfg.str().c_str(), "--help"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg),
          "--help must not fail on bad config");
    CHECK(cfg.help, "help still set");
    pass("parseArgs: --help bypasses config loading");
}

void test_parseArgs_list_audio_skips_bad_config() {
    TempFile bad_cfg("totally not valid syntax\n");
    Argv argv({"ultra_tnc", "--config", bad_cfg.str().c_str(), "--list-audio-devices"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg),
          "--list-audio-devices must bypass config load failure");
    CHECK(cfg.list_audio, "list_audio set");
    pass("parseArgs: --list-audio-devices bypasses config loading");
}

void test_parseArgs_cli_overrides_config() {
    // Config sets baud=19200; CLI flag should override to 4800.
    TempFile good_cfg("ptt_serial_baud = 19200\n");
    Argv argv({"ultra_tnc", "--config", good_cfg.str().c_str(),
               "--ptt-serial-baud", "4800"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(cfg.ptt_serial_baud == 4800, "CLI must override config");
    pass("parseArgs: CLI flags override config values");
}

void test_parseArgs_no_inject_channel_overrides_config_true() {
    // Codex review #5: config-set inject_channel=true must be
    // overridable from CLI back to false via --no-inject-channel.
    TempFile cfg_file("inject_channel = true\n");
    Argv argv({"ultra_tnc", "--config", cfg_file.str().c_str(),
               "--no-inject-channel"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(!cfg.inject_channel, "--no-inject-channel must override config true");
    pass("parseArgs: --no-inject-channel overrides config inject_channel=true");
}

void test_parseArgs_ptt_active_high_overrides_config_true() {
    TempFile cfg_file("ptt_inactive_high = true\n");
    Argv argv({"ultra_tnc", "--config", cfg_file.str().c_str(),
               "--ptt-active-high"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(!cfg.ptt_inactive_high, "--ptt-active-high must override config true");
    pass("parseArgs: --ptt-active-high overrides config ptt_inactive_high=true");
}

void test_parseArgs_no_ptt_inactive_high_alias() {
    TempFile cfg_file("ptt_inactive_high = true\n");
    Argv argv({"ultra_tnc", "--config", cfg_file.str().c_str(),
               "--no-ptt-inactive-high"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(!cfg.ptt_inactive_high, "alias must also work");
    pass("parseArgs: --no-ptt-inactive-high alias works");
}

void test_parseArgs_ptt_cat_flags() {
    Argv argv({"ultra_tnc", "--ptt-cat", "--ptt-cat-host", "127.0.0.1",
               "--ptt-cat-port", "4532"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(cfg.ptt_cat, "--ptt-cat stored");
    CHECK(cfg.ptt_cat_host == "127.0.0.1", "cat host stored");
    CHECK(cfg.ptt_cat_port == 4532, "cat port stored");
    pass("parseArgs: --ptt-cat host/port flags stored");
}

void test_parseArgs_ptt_cat_serial_mutual_exclusion() {
    Argv argv({"ultra_tnc", "--ptt-cat", "--ptt-serial-port", "/dev/ttyUSB0"});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg),
          "CAT and serial PTT must be mutually exclusive");
    pass("parseArgs: CAT and serial PTT mutual exclusion enforced");
}

void test_parseArgs_ptt_cat_serial_mutual_exclusion_from_config() {
    TempFile cfg_file(
        "ptt_cat = true\n"
        "ptt_serial_port = /dev/ttyUSB0\n"
    );
    Argv argv({"ultra_tnc", "--config", cfg_file.str().c_str()});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg),
          "config CAT and serial PTT must be mutually exclusive");
    pass("parseArgs: config CAT/serial mutual exclusion enforced");
}

void test_parseArgs_log_flags() {
    Argv argv({"ultra_tnc", "--log-level", "trace",
               "--log-category", "operator,demod",
               "--log-file", "/tmp/ultra_tnc.log"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(cfg.log_level == ultra::LogLevel::TRACE && cfg.log_level_set, "trace stored");
    CHECK(cfg.log_categories == "operator,demod" && cfg.log_categories_set,
          "categories stored");
    CHECK(cfg.log_file == "/tmp/ultra_tnc.log", "log file stored");
    pass("parseArgs: log flags stored");
}

void test_parseArgs_sim_audio_flags() {
    Argv argv({"ultra_tnc", "--sim-audio",
               "--ota-host", "127.0.0.1:47000",
               "--ota-udp-host", "127.0.0.1:47001",
               "--token", "test_token",
               "--station-id", "alice",
               "--session-id", "lobby"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg), "parse ok");
    CHECK(cfg.sim_audio, "--sim-audio stored");
    CHECK(cfg.ota_host == "127.0.0.1:47000", "ota host stored");
    CHECK(cfg.ota_udp_host == "127.0.0.1:47001", "udp host stored");
    CHECK(cfg.token == "test_token", "token stored");
    CHECK(cfg.station_id == "alice", "station id stored");
    CHECK(cfg.session_id == "lobby", "session id stored");
    pass("parseArgs: --sim-audio OTASim flags stored");
}

void test_parseArgs_sim_audio_requires_identity() {
    Argv argv({"ultra_tnc", "--sim-audio", "--ota-host", "127.0.0.1:47000"});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg),
          "--sim-audio without token/station-id must reject");
    pass("parseArgs: --sim-audio requires host/token/station-id");
}

void test_parseArgs_mod_qam16_requires_expert() {
    Argv argv({"ultra_tnc", "--mod", "qam16"});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg),
          "--mod qam16 must reject without --expert");
    pass("parseArgs: --mod qam16 rejected without --expert");
}

void test_parseArgs_mod_qam16_with_expert() {
    Argv argv({"ultra_tnc", "--expert", "--mod", "qam16"});
    Config cfg;
    CHECK(parseArgs(argv.argc(), argv.data(), cfg),
          "--mod qam16 should parse with --expert");
    CHECK(cfg.expert_phy, "--expert flag stored");
    CHECK(cfg.forced_mod == ultra::Modulation::QAM16, "QAM16 forced in expert mode");
    pass("parseArgs: --expert permits --mod qam16");
}

void test_parseArgs_unknown_flag_rejected() {
    Argv argv({"ultra_tnc", "--this-does-not-exist"});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg), "unknown flag must reject");
    pass("parseArgs: unknown flag rejected");
}

void test_parseArgs_missing_value_rejected() {
    Argv argv({"ultra_tnc", "--callsign"});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg),
          "--callsign without value must reject");
    pass("parseArgs: missing value for --callsign rejected");
}

void test_parseArgs_config_load_failure_propagates() {
    // No --help / --list-audio: a bad explicit --config must fail.
    TempFile bad_cfg("nonsense_field = x\n");
    Argv argv({"ultra_tnc", "--config", bad_cfg.str().c_str()});
    Config cfg;
    CHECK(!parseArgs(argv.argc(), argv.data(), cfg),
          "bad explicit --config must fail when not --help/--list-audio");
    pass("parseArgs: bad --config fails parse (without --help)");
}

}  // namespace

int main() {
    // Strict parsers
    test_parsePositiveIntStrict_basic();
    test_parsePositiveIntStrict_rejects_trailing_garbage();
    test_parsePositiveIntStrict_rejects_negative();
    test_parsePositiveIntStrict_rejects_plus_sign();
    test_parsePositiveIntStrict_range_check();
    test_parsePositiveIntStrict_empty_rejected();
    test_parseBoolStrict_truthy_forms();
    test_parseBoolStrict_falsy_forms();
    test_parseBoolStrict_unknown_rejected();
    test_parseUint16_bounds();
    test_parseFloat_strict();
    test_parseCodeRate();
    test_parseModulation();
    test_isNoneDevice();

    // applyConfigKey
    test_applyConfigKey_audio_devices();
    test_applyConfigKey_sim_audio();
    test_applyConfigKey_port_bounds();
    test_applyConfigKey_callsign_sanitized();
    test_applyConfigKey_inject_channel_modes();
    test_applyConfigKey_ptt_baud_strict();
    test_applyConfigKey_ptt_inactive_high_strict();
    test_applyConfigKey_ptt_serial_line();
    test_applyConfigKey_ptt_cat_settings();
    test_applyConfigKey_unknown_key_rejected();
    test_applyConfigKey_ofdm_config();
    test_applyConfigKey_log_settings();

    // loadConfigFile
    test_loadConfigFile_basic();
    test_loadConfigFile_missing_equals();
    test_loadConfigFile_unknown_key();
    test_loadConfigFile_bad_value();
    test_loadConfigFile_missing_file();
    test_loadConfigFile_comments_and_blank_lines();

    // parseArgs
    test_parseArgs_help_flag();
    test_parseArgs_help_skips_bad_config();
    test_parseArgs_list_audio_skips_bad_config();
    test_parseArgs_cli_overrides_config();
    test_parseArgs_no_inject_channel_overrides_config_true();
    test_parseArgs_ptt_active_high_overrides_config_true();
    test_parseArgs_no_ptt_inactive_high_alias();
    test_parseArgs_ptt_cat_flags();
    test_parseArgs_ptt_cat_serial_mutual_exclusion();
    test_parseArgs_ptt_cat_serial_mutual_exclusion_from_config();
    test_parseArgs_log_flags();
    test_parseArgs_sim_audio_flags();
    test_parseArgs_sim_audio_requires_identity();
    test_parseArgs_mod_qam16_requires_expert();
    test_parseArgs_mod_qam16_with_expert();
    test_parseArgs_unknown_flag_rejected();
    test_parseArgs_missing_value_rejected();
    test_parseArgs_config_load_failure_propagates();

    std::cout << "\nUltraTNCConfig tests: " << passed << " passed, "
              << failed << " failed\n";
    return failed == 0 ? 0 : 1;
}
