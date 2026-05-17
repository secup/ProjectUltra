#include "ultra_tnc_config.hpp"

#include "protocol/frame_v2.hpp"
#include "sim/cli_enums.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <ostream>
#include <vector>

namespace ultra::tnc::config {

std::string lower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool isNoneDevice(const std::string& device) {
    return lower(device) == "none";
}

bool parsePositiveIntStrict(const std::string& text, int& out, int min_v, int max_v) {
    if (text.empty()) return false;
    if (text[0] == '-' || text[0] == '+') return false;
    try {
        size_t parsed = 0;
        long long value = std::stoll(text, &parsed, 10);
        if (parsed != text.size()) return false;
        if (value < min_v || value > max_v) return false;
        out = static_cast<int>(value);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseBoolStrict(const std::string& text, bool& out) {
    std::string lc = lower(text);
    if (lc == "true" || lc == "1" || lc == "yes" || lc == "on") { out = true; return true; }
    if (lc == "false" || lc == "0" || lc == "no" || lc == "off") { out = false; return true; }
    return false;
}

bool parseUint16(const std::string& text, uint16_t& out) {
    try {
        size_t parsed = 0;
        int value = std::stoi(text, &parsed, 10);
        if (parsed != text.size() || value < 0 || value > 65535) {
            return false;
        }
        out = static_cast<uint16_t>(value);
        return true;
    } catch (...) {
        return false;
    }
}

std::optional<float> parseFloat(const std::string& text) {
    try {
        size_t parsed = 0;
        float value = std::stof(text, &parsed);
        if (parsed != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<ultra::CodeRate> parseCodeRate(const std::string& value) {
    return ultra::tools::cli::parseCodeRate(value, ultra::tools::cli::AllowAuto::Yes);
}

std::optional<ultra::Modulation> parseModulation(const std::string& value, bool expert_phy) {
    const auto allow_experimental = expert_phy
        ? ultra::tools::cli::AllowExperimentalModulation::Yes
        : ultra::tools::cli::AllowExperimentalModulation::No;
    return ultra::tools::cli::parseModulation(
        value, ultra::tools::cli::AllowAuto::Yes, allow_experimental);
}

bool envFlagEnabled(const char* name) {
    if (const char* value = std::getenv(name)) {
        const std::string v = lower(value);
        return v == "1" || v == "true" || v == "yes" || v == "on";
    }
    return false;
}

bool parseForcedModulation(const std::string& value,
                           bool expert_phy,
                           ultra::Modulation& out) {
    const auto parsed_any = ultra::tools::cli::parseModulation(
        value,
        ultra::tools::cli::AllowAuto::Yes,
        ultra::tools::cli::AllowExperimentalModulation::Yes);
    if (!parsed_any) {
        return false;
    }
    if (ultra::tools::cli::isExpertOnlyModulation(*parsed_any)) {
        if (!expert_phy) {
            std::cerr << "EXPERT PHY MODE BLOCKED: --mod " << value
                      << " is outside the operator ladder. Re-run with --expert "
                         "only for controlled lab testing.\n";
            return false;
        }
        std::cerr << "EXPERT PHY MODE: forcing --mod " << value
                  << " outside the operator ladder; results are lab-only.\n";
    }
    out = *parsed_any;
    return true;
}

void printUsage(std::ostream& out) {
    out << "ultra_tnc [options]\n"
        << "  --config <path>             Read options from a config file\n"
        << "                              (key = value, # comments). CLI\n"
        << "                              flags override config-file values.\n"
        << "  --audio-output <name>       SDL audio output device, or none\n"
        << "  --audio-input <name>        SDL audio input device, or none\n"
        << "  --sim-audio                 Use OTASim audio instead of SDL devices\n"
        << "  --ota-host <host:port>      OTASim gRPC endpoint for --sim-audio\n"
        << "  --ota-udp-host <host:port>  Optional OTASim UDP endpoint override\n"
        << "  --token <bearer_token>      OTASim bearer token for --sim-audio\n"
        << "  --station-id <id>           OTASim station id for --sim-audio\n"
        << "  --session-id <id>           OTASim session id (default: lobby)\n"
        << "  --port <N>                  TNC command port (default: 8300; data=N+1)\n"
        << "  --bind <addr>               Bind address (default: 127.0.0.1)\n"
        << "  --callsign <call>           Default callsign (default: NOCALL)\n"
        << "  --inject-channel [awgn]     Apply simple TX-side AWGN before audio output\n"
        << "                              (only awgn is implemented; other types rejected)\n"
        << "  --no-inject-channel         Override config inject_channel=true back to false\n"
        << "  --snr <db>                  SNR for channel injection and mode reports\n"
        << "  --rate <auto|r1_4|r1_2|r2_3|r3_4>\n"
        << "  --mod <auto|dqpsk>\n"
        << "  --expert                    Allow lab-only forced PHY modes in --mod\n"
        << "  --ofdm-config <default|nvis>\n"
        << "  --ptt-serial-port <path>    Serial device for hardware PTT\n"
        << "                              (e.g. /dev/cu.usbserial or COM3)\n"
        << "  --ptt-serial-baud <N>       Serial baud (default: 9600; line\n"
        << "                              toggling works at any baud)\n"
        << "  --ptt-serial-line <rts|dtr> Which line keys TX (default: rts)\n"
        << "  --ptt-inactive-high         Some radios invert; default is\n"
        << "                              inactive=low / asserted=high\n"
        << "  --ptt-active-high           Override config ptt_inactive_high=true\n"
        << "                              back to default polarity (also: --no-ptt-inactive-high)\n"
        << "  --ptt-cat                   Enable Hamlib rigctld CAT PTT\n"
        << "                              (mutually exclusive with --ptt-serial-port\n"
        << "                              and --ptt-hamlib)\n"
        << "  --ptt-cat-host <host>       rigctld host (default: 127.0.0.1)\n"
        << "  --ptt-cat-port <N>          rigctld TCP port (default: 4532)\n"
        << "  --ptt-hamlib                Enable Hamlib (built-in) CAT PTT\n"
        << "                              (links libhamlib directly; mutually\n"
        << "                              exclusive with --ptt-serial-port and\n"
        << "                              --ptt-cat)\n"
        << "  --ptt-hamlib-model <id>     Hamlib rig model id (default: 1 = Dummy;\n"
        << "                              e.g. IC-705 ~ 3085, FT-991A ~ 1042)\n"
        << "  --ptt-hamlib-port <path>    Serial port for the radio (e.g.\n"
        << "                              /dev/cu.usbserial-FT001 or COM5)\n"
        << "  --ptt-hamlib-baud <N>       Serial baud (default: 9600)\n"
        << "  --ptt-hamlib-ptt <m>        PTT method: vox | cat | dtr | rts\n"
        << "                              (default: cat)\n"
        << "  --log-level <error|warn|info|debug|trace>\n"
        << "                              Console verbosity (default: info)\n"
        << "  --log-category <list>       Comma list: operator,audio,tnc,modem,\n"
        << "                              demod,sync,ldpc,channel,all\n"
        << "  --log-file <path>           Write logs to file instead of stderr\n"
        << "  --list-audio-devices        Print available SDL audio devices and exit\n"
        << "  --accept-audio-consent      Grant diagnostics RX-audio capture consent\n"
        << "                              (headless equivalent of the GUI's first-run\n"
        << "                              dialog; without this, report bundles omit audio)\n"
        << "  --version                   Print build provenance and exit\n"
        << "  --help\n"
        << "\n"
        << "Config file format (key=value, one per line, # comments):\n"
        << "  audio_output = USB Audio CODEC\n"
        << "  audio_input  = USB Audio CODEC\n"
        << "  callsign     = N0CALL\n"
        << "  port         = 8300\n"
        << "  ptt_serial_port = /dev/cu.usbserial-FT001\n"
        << "  ptt_serial_line = rts\n"
        << "  ptt_cat = false\n"
        << "  ptt_cat_host = 127.0.0.1\n"
        << "  ptt_cat_port = 4532\n"
        << "  ptt_hamlib = false\n"
        << "  ptt_hamlib_model = 1\n"
        << "  ptt_hamlib_port = /dev/cu.usbserial-FT001\n"
        << "  ptt_hamlib_baud = 9600\n"
        << "  ptt_hamlib_ptt = cat\n"
        << "  log_level   = info\n"
        << "\n"
        << "Default config search path: ./ultra_tnc.conf, then\n"
        << "  $XDG_CONFIG_HOME/ultra_tnc/config (or ~/.config/ultra_tnc/config).\n";
}

bool applyConfigKey(const std::string& key, const std::string& value, Config& cfg) {
    if (key == "audio_output" || key == "audio-output") {
        cfg.audio_output = value;
    } else if (key == "audio_input" || key == "audio-input") {
        cfg.audio_input = value;
    } else if (key == "sim_audio" || key == "sim-audio") {
        if (!parseBoolStrict(value, cfg.sim_audio)) return false;
    } else if (key == "ota_host" || key == "ota-host") {
        if (value.empty()) return false;
        cfg.ota_host = value;
    } else if (key == "ota_udp_host" || key == "ota-udp-host") {
        cfg.ota_udp_host = value;
    } else if (key == "token") {
        if (value.empty()) return false;
        cfg.token = value;
    } else if (key == "station_id" || key == "station-id") {
        if (value.empty()) return false;
        cfg.station_id = value;
    } else if (key == "session_id" || key == "session-id") {
        if (value.empty()) return false;
        cfg.session_id = value;
    } else if (key == "port") {
        if (!parseUint16(value, cfg.port) ||
            cfg.port == std::numeric_limits<uint16_t>::max()) return false;
    } else if (key == "bind" || key == "bind_address") {
        cfg.bind_address = value;
    } else if (key == "callsign") {
        cfg.callsign = ultra::protocol::sanitizeCallsign(value);
        if (cfg.callsign.empty()) return false;
    } else if (key == "inject_channel" || key == "inject-channel") {
        if (parseBoolStrict(value, cfg.inject_channel)) {
            // Plain boolean, done.
        } else {
            // Channel-type values are accepted as a synonym for "true"
            // for forward-compat with cli_simulator's spelling, but
            // only AWGN is actually implemented here. Reject anything
            // else loudly rather than silently using AWGN.
            const std::string lc = lower(value);
            if (lc != "awgn") {
                std::cerr << "ultra_tnc only supports inject_channel=awgn|true|false; "
                             "got '" << value << "'\n";
                return false;
            }
            cfg.inject_channel = true;
            cfg.inject_channel_type = lc;
        }
    } else if (key == "snr" || key == "snr_db") {
        auto parsed = parseFloat(value);
        if (!parsed) return false;
        cfg.snr_db = *parsed;
    } else if (key == "rate") {
        auto parsed = parseCodeRate(value);
        if (!parsed) return false;
        cfg.forced_rate = *parsed;
    } else if (key == "mod" || key == "modulation") {
        if (!parseForcedModulation(value, cfg.expert_phy, cfg.forced_mod)) return false;
    } else if (key == "ofdm_config" || key == "ofdm-config") {
        const std::string preset = lower(value);
        if (preset == "default") cfg.ofdm_config = OFDMConfigPreset::Default;
        else if (preset == "nvis") cfg.ofdm_config = OFDMConfigPreset::Nvis;
        else return false;
    } else if (key == "ptt_serial_port" || key == "ptt-serial-port") {
        cfg.ptt_serial_port = value;
    } else if (key == "ptt_serial_baud" || key == "ptt-serial-baud") {
        if (!parsePositiveIntStrict(value, cfg.ptt_serial_baud, 50, 4000000)) return false;
    } else if (key == "ptt_serial_line" || key == "ptt-serial-line") {
        const std::string line = lower(value);
        if (line != "rts" && line != "dtr") return false;
        cfg.ptt_serial_line = line;
    } else if (key == "ptt_inactive_high" || key == "ptt-inactive-high") {
        if (!parseBoolStrict(value, cfg.ptt_inactive_high)) return false;
    } else if (key == "ptt_cat" || key == "ptt-cat") {
        if (!parseBoolStrict(value, cfg.ptt_cat)) return false;
    } else if (key == "ptt_cat_host" || key == "ptt-cat-host") {
        if (value.empty()) return false;
        cfg.ptt_cat_host = value;
    } else if (key == "ptt_cat_port" || key == "ptt-cat-port") {
        uint16_t port = 0;
        if (!parseUint16(value, port) || port == 0) return false;
        cfg.ptt_cat_port = port;
    } else if (key == "ptt_hamlib" || key == "ptt-hamlib") {
        if (!parseBoolStrict(value, cfg.ptt_hamlib)) return false;
    } else if (key == "ptt_hamlib_model" || key == "ptt-hamlib-model") {
        if (!parsePositiveIntStrict(value, cfg.ptt_hamlib_model, 1, 99999)) return false;
    } else if (key == "ptt_hamlib_port" || key == "ptt-hamlib-port") {
        cfg.ptt_hamlib_port = value;
    } else if (key == "ptt_hamlib_baud" || key == "ptt-hamlib-baud") {
        if (!parsePositiveIntStrict(value, cfg.ptt_hamlib_baud, 50, 4000000)) return false;
    } else if (key == "ptt_hamlib_ptt" || key == "ptt-hamlib-ptt") {
        const std::string m = lower(value);
        if (m != "vox" && m != "cat" && m != "dtr" && m != "rts") return false;
        cfg.ptt_hamlib_ptt = m;
    } else if (key == "log_level" || key == "log-level") {
        if (!ultra::parseLogLevel(value, cfg.log_level)) return false;
        cfg.log_level_set = true;
    } else if (key == "log_category" || key == "log-category" ||
               key == "log_categories" || key == "log-categories") {
        if (value.empty()) return false;
        cfg.log_categories = value;
        cfg.log_categories_set = true;
    } else if (key == "log_file" || key == "log-file") {
        cfg.log_file = value;
    } else {
        std::cerr << "Unknown config key: " << key << "\n";
        return false;
    }
    return true;
}

bool loadConfigFile(const std::string& path, Config& cfg) {
    std::ifstream in(path);
    if (!in) return false;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        auto trim = [](std::string& s) {
            while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.erase(0, 1);
            while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
        };
        trim(line);
        if (line.empty()) continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            std::cerr << "Config " << path << ":" << line_no << ": missing '='\n";
            return false;
        }
        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);
        trim(key); trim(value);
        if (key.empty()) continue;
        if (!applyConfigKey(key, value, cfg)) {
            std::cerr << "Config " << path << ":" << line_no << ": bad value for '"
                      << key << "': " << value << "\n";
            return false;
        }
    }
    return true;
}

std::string findDefaultConfigFile() {
    std::vector<std::string> candidates = {"ultra_tnc.conf"};
    if (const char* xdg = std::getenv("XDG_CONFIG_HOME")) {
        candidates.push_back(std::string(xdg) + "/ultra_tnc/config");
    }
    if (const char* home = std::getenv("HOME")) {
        candidates.push_back(std::string(home) + "/.config/ultra_tnc/config");
    }
    for (const auto& path : candidates) {
        std::ifstream in(path);
        if (in) return path;
    }
    return {};
}

bool parseArgs(int argc, char** argv, Config& cfg) {
    std::string explicit_config;
    bool needs_config = true;
    if (envFlagEnabled("ULTRA_EXPERT_PHY")) {
        cfg.expert_phy = true;
    }
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--expert") {
            cfg.expert_phy = true;
        }
        if (arg == "--help" || arg == "-h" || arg == "--list-audio-devices" ||
            arg == "--version") {
            needs_config = false;
            break;
        }
    }
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc) {
            explicit_config = argv[i + 1];
            break;
        }
    }
    if (needs_config) {
        if (!explicit_config.empty()) {
            if (!loadConfigFile(explicit_config, cfg)) {
                std::cerr << "Failed to load --config " << explicit_config << "\n";
                return false;
            }
        } else {
            const std::string default_path = findDefaultConfigFile();
            if (!default_path.empty()) {
                if (!loadConfigFile(default_path, cfg)) {
                    std::cerr << "Failed to load default config " << default_path << "\n";
                    return false;
                }
                std::cout << "Loaded config: " << default_path << "\n";
            }
        }
    }

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto requireValue = [&](const char* name) -> std::optional<std::string> {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << name << "\n";
                return std::nullopt;
            }
            return std::string(argv[++i]);
        };

        if (arg == "--help" || arg == "-h") {
            cfg.help = true;
        } else if (arg == "--list-audio-devices") {
            cfg.list_audio = true;
        } else if (arg == "--version") {
            cfg.version = true;
        } else if (arg == "--accept-audio-consent") {
            cfg.accept_audio_consent = true;
        } else if (arg == "--config") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --config\n";
                return false;
            }
            ++i;
        } else if (arg == "--audio-output") {
            auto value = requireValue("--audio-output");
            if (!value) return false;
            cfg.audio_output = *value;
        } else if (arg == "--audio-input") {
            auto value = requireValue("--audio-input");
            if (!value) return false;
            cfg.audio_input = *value;
        } else if (arg == "--sim-audio") {
            cfg.sim_audio = true;
        } else if (arg == "--ota-host") {
            auto value = requireValue("--ota-host");
            if (!value || value->empty()) {
                std::cerr << "Invalid --ota-host value\n";
                return false;
            }
            cfg.ota_host = *value;
        } else if (arg == "--ota-udp-host") {
            auto value = requireValue("--ota-udp-host");
            if (!value) return false;
            cfg.ota_udp_host = *value;
        } else if (arg == "--token") {
            auto value = requireValue("--token");
            if (!value || value->empty()) {
                std::cerr << "Invalid --token value\n";
                return false;
            }
            cfg.token = *value;
        } else if (arg == "--station-id") {
            auto value = requireValue("--station-id");
            if (!value || value->empty()) {
                std::cerr << "Invalid --station-id value\n";
                return false;
            }
            cfg.station_id = *value;
        } else if (arg == "--session-id") {
            auto value = requireValue("--session-id");
            if (!value || value->empty()) {
                std::cerr << "Invalid --session-id value\n";
                return false;
            }
            cfg.session_id = *value;
        } else if (arg == "--port") {
            auto value = requireValue("--port");
            if (!value || !parseUint16(*value, cfg.port) ||
                cfg.port == std::numeric_limits<uint16_t>::max()) {
                std::cerr << "Invalid --port value (must leave room for data port N+1)\n";
                return false;
            }
        } else if (arg == "--bind") {
            auto value = requireValue("--bind");
            if (!value) return false;
            cfg.bind_address = *value;
        } else if (arg == "--callsign") {
            auto value = requireValue("--callsign");
            if (!value) return false;
            cfg.callsign = ultra::protocol::sanitizeCallsign(*value);
            if (cfg.callsign.empty()) {
                std::cerr << "Invalid --callsign value\n";
                return false;
            }
        } else if (arg == "--inject-channel") {
            cfg.inject_channel = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                const std::string type = lower(argv[++i]);
                if (type != "awgn") {
                    std::cerr << "ultra_tnc --inject-channel only supports awgn; "
                                 "got '" << type << "'\n";
                    return false;
                }
                cfg.inject_channel_type = type;
            }
        } else if (arg == "--no-inject-channel") {
            cfg.inject_channel = false;
        } else if (arg == "--snr") {
            auto value = requireValue("--snr");
            auto parsed = value ? parseFloat(*value) : std::nullopt;
            if (!parsed) {
                std::cerr << "Invalid --snr value\n";
                return false;
            }
            cfg.snr_db = *parsed;
        } else if (arg == "--rate") {
            auto value = requireValue("--rate");
            auto parsed = value ? parseCodeRate(*value) : std::nullopt;
            if (!parsed) {
                std::cerr << "Unknown code rate (use auto, r1_4, r1_2, r2_3, r3_4)\n";
                return false;
            }
            cfg.forced_rate = *parsed;
        } else if (arg == "--mod") {
            auto value = requireValue("--mod");
            if (!value || !parseForcedModulation(*value, cfg.expert_phy, cfg.forced_mod)) {
                if (value) {
                    std::cerr << "Allowed --mod values here: "
                              << ultra::tools::cli::modulationChoices(
                                     ultra::tools::cli::AllowAuto::Yes,
                                     cfg.expert_phy
                                         ? ultra::tools::cli::AllowExperimentalModulation::Yes
                                         : ultra::tools::cli::AllowExperimentalModulation::No)
                              << "\n";
                }
                return false;
            }
        } else if (arg == "--expert") {
            cfg.expert_phy = true;
        } else if (arg == "--ofdm-config") {
            auto value = requireValue("--ofdm-config");
            if (!value) return false;
            const std::string preset = lower(*value);
            if (preset == "default") {
                cfg.ofdm_config = OFDMConfigPreset::Default;
            } else if (preset == "nvis") {
                cfg.ofdm_config = OFDMConfigPreset::Nvis;
            } else {
                std::cerr << "Unknown OFDM config (use default or nvis)\n";
                return false;
            }
        } else if (arg == "--ptt-serial-port") {
            auto value = requireValue("--ptt-serial-port");
            if (!value) return false;
            cfg.ptt_serial_port = *value;
        } else if (arg == "--ptt-serial-baud") {
            auto value = requireValue("--ptt-serial-baud");
            if (!value) return false;
            if (!parsePositiveIntStrict(*value, cfg.ptt_serial_baud, 50, 4000000)) {
                std::cerr << "Invalid --ptt-serial-baud value (must be 50..4000000)\n";
                return false;
            }
        } else if (arg == "--ptt-serial-line") {
            auto value = requireValue("--ptt-serial-line");
            if (!value) return false;
            const std::string line = lower(*value);
            if (line != "rts" && line != "dtr") {
                std::cerr << "--ptt-serial-line must be rts or dtr\n";
                return false;
            }
            cfg.ptt_serial_line = line;
        } else if (arg == "--ptt-inactive-high") {
            cfg.ptt_inactive_high = true;
        } else if (arg == "--ptt-active-high" || arg == "--no-ptt-inactive-high") {
            cfg.ptt_inactive_high = false;
        } else if (arg == "--ptt-cat") {
            cfg.ptt_cat = true;
        } else if (arg == "--ptt-cat-host") {
            auto value = requireValue("--ptt-cat-host");
            if (!value || value->empty()) {
                std::cerr << "Invalid --ptt-cat-host value\n";
                return false;
            }
            cfg.ptt_cat_host = *value;
        } else if (arg == "--ptt-cat-port") {
            auto value = requireValue("--ptt-cat-port");
            uint16_t port = 0;
            if (!value || !parseUint16(*value, port) || port == 0) {
                std::cerr << "Invalid --ptt-cat-port value (must be 1..65535)\n";
                return false;
            }
            cfg.ptt_cat_port = port;
        } else if (arg == "--ptt-hamlib") {
            cfg.ptt_hamlib = true;
        } else if (arg == "--ptt-hamlib-model") {
            auto value = requireValue("--ptt-hamlib-model");
            if (!value || !parsePositiveIntStrict(*value, cfg.ptt_hamlib_model, 1, 99999)) {
                std::cerr << "Invalid --ptt-hamlib-model id (must be 1..99999)\n";
                return false;
            }
        } else if (arg == "--ptt-hamlib-port") {
            auto value = requireValue("--ptt-hamlib-port");
            if (!value) return false;
            cfg.ptt_hamlib_port = *value;
        } else if (arg == "--ptt-hamlib-baud") {
            auto value = requireValue("--ptt-hamlib-baud");
            if (!value || !parsePositiveIntStrict(*value, cfg.ptt_hamlib_baud, 50, 4000000)) {
                std::cerr << "Invalid --ptt-hamlib-baud value (must be 50..4000000)\n";
                return false;
            }
        } else if (arg == "--ptt-hamlib-ptt") {
            auto value = requireValue("--ptt-hamlib-ptt");
            if (!value) return false;
            const std::string m = lower(*value);
            if (m != "vox" && m != "cat" && m != "dtr" && m != "rts") {
                std::cerr << "--ptt-hamlib-ptt must be vox|cat|dtr|rts\n";
                return false;
            }
            cfg.ptt_hamlib_ptt = m;
        } else if (arg == "--log-level") {
            auto value = requireValue("--log-level");
            if (!value || !ultra::parseLogLevel(*value, cfg.log_level)) {
                std::cerr << "Invalid --log-level (use error, warn, info, debug, or trace)\n";
                return false;
            }
            cfg.log_level_set = true;
        } else if (arg == "--log-category" || arg == "--log-categories") {
            auto value = requireValue(arg.c_str());
            if (!value || value->empty()) {
                std::cerr << "Invalid --log-category value\n";
                return false;
            }
            cfg.log_categories = *value;
            cfg.log_categories_set = true;
        } else if (arg == "--log-file") {
            auto value = requireValue("--log-file");
            if (!value || value->empty()) {
                std::cerr << "Invalid --log-file value\n";
                return false;
            }
            cfg.log_file = *value;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }
    {
        int ptt_backends = 0;
        if (cfg.ptt_cat) ++ptt_backends;
        if (!cfg.ptt_serial_port.empty()) ++ptt_backends;
        if (cfg.ptt_hamlib) ++ptt_backends;
        if (ptt_backends > 1) {
            std::cerr << "PTT must be exactly one of: serial, --ptt-cat, "
                         "or --ptt-hamlib (built-in).\n";
            return false;
        }
    }
    if (cfg.sim_audio) {
        if (cfg.ota_host.empty() || cfg.token.empty() || cfg.station_id.empty()) {
            std::cerr << "--sim-audio requires --ota-host, --token, and --station-id\n";
            return false;
        }
        if (cfg.session_id.empty()) {
            cfg.session_id = "lobby";
        }
    }
    return true;
}

}  // namespace ultra::tnc::config
