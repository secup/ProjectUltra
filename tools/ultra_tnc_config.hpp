#pragma once

// Configuration plumbing for ultra_tnc. Pulled out of the main
// translation unit so applyConfigKey, loadConfigFile, parseArgs, and
// the strict parsers can be unit tested without standing up audio,
// PTT, or the protocol engine. ultra_tnc.cpp consumes the same
// declarations to keep the runtime path identical.

#include "ultra/types.hpp"
#include "ultra/logging.hpp"

#include <climits>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>

namespace ultra::tnc::config {

enum class OFDMConfigPreset {
    Default,
    Nvis,
};

struct Config {
    std::string audio_output;
    std::string audio_input;
    bool sim_audio = false;
    std::string ota_host;
    std::string ota_udp_host;
    std::string token;
    std::string station_id;
    std::string session_id = "lobby";
    std::string bind_address = "127.0.0.1";
    uint16_t port = 8300;
    std::string callsign = "NOCALL";
    bool inject_channel = false;
    std::string inject_channel_type = "awgn";
    float snr_db = 20.0f;
    ultra::CodeRate forced_rate = ultra::CodeRate::AUTO;
    ultra::Modulation forced_mod = ultra::Modulation::AUTO;
    OFDMConfigPreset ofdm_config = OFDMConfigPreset::Default;
    std::string ptt_serial_port;
    int ptt_serial_baud = 9600;
    std::string ptt_serial_line = "rts";
    bool ptt_inactive_high = false;
    bool ptt_cat = false;
    std::string ptt_cat_host = "127.0.0.1";
    uint16_t ptt_cat_port = 4532;
    bool ptt_hamlib = false;
    int ptt_hamlib_model = 1;
    std::string ptt_hamlib_port;
    int ptt_hamlib_baud = 9600;
    std::string ptt_hamlib_ptt = "cat";
    ultra::LogLevel log_level = ultra::LogLevel::INFO;
    bool log_level_set = false;
    std::string log_categories;
    bool log_categories_set = false;
    std::string log_file;
    bool expert_phy = false;
    bool accept_audio_consent = false;
    bool help = false;
    bool list_audio = false;
    bool version = false;
};

// Strict parsers — full string consumed, no silent partial accept.
bool parsePositiveIntStrict(const std::string& text, int& out,
                            int min_v = 0, int max_v = INT_MAX);
bool parseBoolStrict(const std::string& text, bool& out);
bool parseUint16(const std::string& text, uint16_t& out);
std::optional<float> parseFloat(const std::string& text);
std::optional<ultra::CodeRate> parseCodeRate(const std::string& value);
std::optional<ultra::Modulation> parseModulation(const std::string& value,
                                                 bool expert_phy = false);

// ASCII lower-case (no locale). Exposed alongside the parsers
// because ultra_tnc.cpp uses it on the runtime PTT path too.
std::string lower(std::string s);

bool isNoneDevice(const std::string& device);

// Apply one "key = value" pair from a config file or env. Returns
// false on bad value or unknown key (with stderr message).
bool applyConfigKey(const std::string& key, const std::string& value, Config& cfg);

// Load all key=value pairs from a config file. Returns false if the
// file can't be opened or any line is malformed.
bool loadConfigFile(const std::string& path, Config& cfg);

// Search for a default config in conventional locations:
//   ./ultra_tnc.conf, $XDG_CONFIG_HOME/ultra_tnc/config,
//   ~/.config/ultra_tnc/config. Returns empty string if none found.
std::string findDefaultConfigFile();

void printUsage(std::ostream& out);

// argv parser. Loads explicit/default config first (skipped for
// --help / --list-audio-devices so a bad config doesn't lock the
// operator out of recovery modes). CLI flags override config values.
bool parseArgs(int argc, char** argv, Config& cfg);

}  // namespace ultra::tnc::config
