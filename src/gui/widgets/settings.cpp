#include "settings.hpp"
#include "imgui.h"
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#else
#define MKDIR(path) mkdir(path, 0755)
#endif

namespace ultra {
namespace gui {

// Get default settings file path
std::string AppSettings::getDefaultPath() {
#ifdef _WIN32
    const char* appdata = std::getenv("APPDATA");
    if (appdata) {
        return std::string(appdata) + "\\ProjectUltra\\settings.ini";
    }
    return "settings.ini";
#else
    const char* home = std::getenv("HOME");
    if (home) {
        return std::string(home) + "/.config/ultra/settings.ini";
    }
    return "settings.ini";
#endif
}

// Helper to create directory if it doesn't exist
static void ensureDirectory(const std::string& path) {
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        std::string dir = path.substr(0, pos);
        // Create parent directories recursively
        for (size_t i = 0; i < dir.size(); i++) {
            if (dir[i] == '/' || dir[i] == '\\') {
                std::string subdir = dir.substr(0, i);
                if (!subdir.empty()) {
                    MKDIR(subdir.c_str());
                }
            }
        }
        MKDIR(dir.c_str());
    }
}

// Save settings to INI file
bool AppSettings::save(const std::string& path) const {
    std::string filepath = path.empty() ? getDefaultPath() : path;
    ensureDirectory(filepath);

    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    file << "[Station]\n";
    file << "callsign=" << callsign << "\n";
    file << "grid_square=" << grid_square << "\n";
    file << "name=" << name << "\n";

    file << "\n[Radio]\n";
    file << "rig_model=" << rig_model << "\n";
    file << "rig_port=" << rig_port << "\n";
    file << "rig_baud=" << rig_baud << "\n";
    file << "use_cat_ptt=" << (use_cat_ptt ? "1" : "0") << "\n";

    file << "\n[Audio]\n";
    file << "input_device=" << input_device << "\n";
    file << "output_device=" << output_device << "\n";
    file << "tx_delay_ms=" << tx_delay_ms << "\n";
    file << "tx_tail_ms=" << tx_tail_ms << "\n";
    file << "tx_drive=" << tx_drive << "\n";

    return true;
}

// Load settings from INI file
bool AppSettings::load(const std::string& path) {
    std::string filepath = path.empty() ? getDefaultPath() : path;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == '[') {
            continue;
        }

        size_t eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);

        // Station settings
        if (key == "callsign") {
            strncpy(callsign, value.c_str(), sizeof(callsign) - 1);
        } else if (key == "grid_square") {
            strncpy(grid_square, value.c_str(), sizeof(grid_square) - 1);
        } else if (key == "name") {
            strncpy(name, value.c_str(), sizeof(name) - 1);
        }
        // Radio settings
        else if (key == "rig_model") {
            strncpy(rig_model, value.c_str(), sizeof(rig_model) - 1);
        } else if (key == "rig_port") {
            strncpy(rig_port, value.c_str(), sizeof(rig_port) - 1);
        } else if (key == "rig_baud") {
            rig_baud = std::atoi(value.c_str());
        } else if (key == "use_cat_ptt") {
            use_cat_ptt = (value == "1" || value == "true");
        }
        // Audio settings
        else if (key == "input_device") {
            strncpy(input_device, value.c_str(), sizeof(input_device) - 1);
        } else if (key == "output_device") {
            strncpy(output_device, value.c_str(), sizeof(output_device) - 1);
        } else if (key == "tx_delay_ms") {
            tx_delay_ms = std::atoi(value.c_str());
        } else if (key == "tx_tail_ms") {
            tx_tail_ms = std::atoi(value.c_str());
        } else if (key == "tx_drive") {
            tx_drive = std::strtof(value.c_str(), nullptr);
        }
    }

    return true;
}

SettingsWindow::SettingsWindow() = default;

bool SettingsWindow::render(AppSettings& settings) {
    just_closed_ = false;

    if (!visible_) {
        // Check if we just closed
        if (was_visible_) {
            just_closed_ = true;
            if (on_closed_) {
                on_closed_();
            }
        }
        was_visible_ = false;
        return false;
    }

    bool changed = false;

    ImGui::SetNextWindowSize(ImVec2(450, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Settings", &visible_, ImGuiWindowFlags_NoCollapse)) {

        if (ImGui::BeginTabBar("SettingsTabs")) {

            if (ImGui::BeginTabItem("Station")) {
                renderStationTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Radio")) {
                renderRadioTab(settings);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Audio")) {
                renderAudioTab(settings);
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();

    // Track visibility for next frame (to detect close via X button)
    was_visible_ = visible_;

    return changed;
}

void SettingsWindow::renderStationTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Station Information");
    ImGui::Separator();
    ImGui::Spacing();

    // Callsign
    ImGui::Text("Callsign");
    ImGui::SetNextItemWidth(150);
    char old_call[16];
    strncpy(old_call, settings.callsign, sizeof(old_call));

    if (ImGui::InputText("##callsign", settings.callsign, sizeof(settings.callsign),
                         ImGuiInputTextFlags_CharsUppercase)) {
        // Notify if changed
        if (strcmp(old_call, settings.callsign) != 0 && on_callsign_changed_) {
            on_callsign_changed_(settings.callsign);
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("(Required for ARQ)");

    ImGui::Spacing();

    // Grid square
    ImGui::Text("Grid Square");
    ImGui::SetNextItemWidth(100);
    ImGui::InputText("##grid", settings.grid_square, sizeof(settings.grid_square),
                     ImGuiInputTextFlags_CharsUppercase);
    ImGui::SameLine();
    ImGui::TextDisabled("Maidenhead locator (e.g., FN35)");

    ImGui::Spacing();

    // Operator name
    ImGui::Text("Operator Name");
    ImGui::SetNextItemWidth(200);
    ImGui::InputText("##name", settings.name, sizeof(settings.name));

    ImGui::Spacing();
    ImGui::Spacing();

    // Validation indicator
    bool valid_call = strlen(settings.callsign) >= 3;
    if (valid_call) {
        ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "Station configured");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Enter your callsign to use ARQ mode");
    }
}

void SettingsWindow::renderRadioTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Radio Control (Hamlib)");
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextDisabled("Radio control via Hamlib - Coming soon!");
    ImGui::Spacing();

    // Rig model (placeholder)
    ImGui::Text("Rig Model");
    ImGui::SetNextItemWidth(200);
    ImGui::BeginDisabled();
    ImGui::InputText("##rig_model", settings.rig_model, sizeof(settings.rig_model));
    ImGui::EndDisabled();

    // Port
    ImGui::Text("Serial Port");
    ImGui::SetNextItemWidth(200);
    ImGui::BeginDisabled();
    ImGui::InputText("##rig_port", settings.rig_port, sizeof(settings.rig_port));
    ImGui::EndDisabled();

    // Baud
    ImGui::Text("Baud Rate");
    ImGui::SetNextItemWidth(100);
    ImGui::BeginDisabled();
    const char* bauds[] = { "4800", "9600", "19200", "38400", "57600", "115200" };
    int baud_idx = 1;  // Default 9600
    ImGui::Combo("##baud", &baud_idx, bauds, 6);
    ImGui::EndDisabled();

    ImGui::Spacing();

    // CAT PTT
    ImGui::BeginDisabled();
    ImGui::Checkbox("Use CAT for PTT", &settings.use_cat_ptt);
    ImGui::EndDisabled();
    ImGui::TextDisabled("(Otherwise uses VOX or manual PTT)");

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1.0f),
        "Hamlib integration will enable:\n"
        "- Automatic frequency/mode setting\n"
        "- CAT PTT control\n"
        "- Rig status display");
}

void SettingsWindow::renderAudioTab(AppSettings& settings) {
    ImGui::Spacing();
    ImGui::Text("Audio Devices");
    ImGui::Separator();
    ImGui::Spacing();

    // Output device selection
    ImGui::Text("Output Device (Speaker)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##output_device", settings.output_device)) {
        for (const auto& dev : output_devices) {
            bool selected = (strcmp(settings.output_device, dev.c_str()) == 0);
            if (ImGui::Selectable(dev.c_str(), selected)) {
                strncpy(settings.output_device, dev.c_str(), sizeof(settings.output_device) - 1);
                settings.output_device[sizeof(settings.output_device) - 1] = '\0';
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();

    // Input device selection
    ImGui::Text("Input Device (Microphone)");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##input_device", settings.input_device)) {
        for (const auto& dev : input_devices) {
            bool selected = (strcmp(settings.input_device, dev.c_str()) == 0);
            if (ImGui::Selectable(dev.c_str(), selected)) {
                strncpy(settings.input_device, dev.c_str(), sizeof(settings.input_device) - 1);
                settings.input_device[sizeof(settings.input_device) - 1] = '\0';
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();

    // Rescan button
    if (ImGui::Button("Rescan Audio Devices", ImVec2(200, 0))) {
        if (on_audio_reset_) {
            on_audio_reset_();
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Refresh device list");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::Text("TX Settings");
    ImGui::Spacing();

    // TX Delay
    ImGui::Text("TX Delay (ms)");
    ImGui::SetNextItemWidth(150);
    ImGui::SliderInt("##tx_delay", &settings.tx_delay_ms, 0, 500);
    ImGui::SameLine();
    ImGui::TextDisabled("Delay after PTT before audio");

    // TX Tail
    ImGui::Text("TX Tail (ms)");
    ImGui::SetNextItemWidth(150);
    ImGui::SliderInt("##tx_tail", &settings.tx_tail_ms, 0, 500);
    ImGui::SameLine();
    ImGui::TextDisabled("Hold PTT after audio ends");

    ImGui::Spacing();

    // TX Drive
    ImGui::Text("TX Drive Level");
    ImGui::SetNextItemWidth(200);
    ImGui::SliderFloat("##tx_drive", &settings.tx_drive, 0.0f, 1.0f, "%.0f%%");
    ImGui::SameLine();
    ImGui::TextDisabled("Audio output level");

    ImGui::Spacing();
    ImGui::Spacing();

    // Visual indicator
    ImGui::Text("TX Level Preview:");
    ImGui::ProgressBar(settings.tx_drive, ImVec2(200, 20), "");
}

} // namespace gui
} // namespace ultra
