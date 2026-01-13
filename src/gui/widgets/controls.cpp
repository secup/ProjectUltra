#include "controls.hpp"
#include "imgui.h"
#include <SDL.h>

namespace ultra {
namespace gui {

ControlsWidget::ControlsWidget() {
    enumerateAudioDevices();
}

void ControlsWidget::enumerateAudioDevices() {
    audio_devices_.clear();
    audio_devices_.push_back("Default");

    // Enumerate SDL audio devices
    int count = SDL_GetNumAudioDevices(0);  // 0 = capture devices
    for (int i = 0; i < count; ++i) {
        const char* name = SDL_GetAudioDeviceName(i, 0);
        if (name) {
            audio_devices_.push_back(name);
        }
    }
}

ControlsWidget::Event ControlsWidget::render(ModemConfig& config, bool connected) {
    Event event = Event::None;

    ImGui::Text("Controls");
    ImGui::Separator();
    ImGui::Spacing();

    // Audio device selection
    ImGui::Text("Audio Device");
    if (ImGui::BeginCombo("##audio_device",
                          audio_devices_[selected_device_].c_str())) {
        for (int i = 0; i < static_cast<int>(audio_devices_.size()); ++i) {
            bool is_selected = (selected_device_ == i);
            if (ImGui::Selectable(audio_devices_[i].c_str(), is_selected)) {
                selected_device_ = i;
                event = Event::AudioDeviceChanged;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();

    // Speed profile selection
    ImGui::Text("Speed Profile");
    const char* profiles[] = { "Conservative", "Balanced", "Turbo" };

    if (ImGui::BeginCombo("##profile", profiles[selected_profile_])) {
        for (int i = 0; i < 3; ++i) {
            bool is_selected = (selected_profile_ == i);
            if (ImGui::Selectable(profiles[i], is_selected)) {
                selected_profile_ = i;
                config.speed_profile = static_cast<SpeedProfile>(i);
                event = Event::ProfileChanged;
            }
            if (is_selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    // Show profile details
    ImGui::TextDisabled("(%s)",
        selected_profile_ == 0 ? "QPSK R1/2, ~2 kbps" :
        selected_profile_ == 1 ? "64-QAM R3/4, ~10 kbps" :
                                 "256-QAM R7/8, ~16 kbps");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Connect/Disconnect buttons
    float button_width = ImGui::GetContentRegionAvail().x / 2 - 5;

    if (!connected) {
        if (ImGui::Button("Connect", ImVec2(button_width, 40))) {
            event = Event::Connect;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled();
        ImGui::Button("Disconnect", ImVec2(button_width, 40));
        ImGui::EndDisabled();
    } else {
        ImGui::BeginDisabled();
        ImGui::Button("Connect", ImVec2(button_width, 40));
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Disconnect", ImVec2(button_width, 40))) {
            event = Event::Disconnect;
        }
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // TX/RX meters (placeholder)
    ImGui::Text("TX Level");
    float tx_level = connected ? 0.3f + 0.2f * sinf(ImGui::GetTime() * 5) : 0.0f;
    ImGui::ProgressBar(tx_level, ImVec2(-1, 0), "");

    ImGui::Text("RX Level");
    float rx_level = connected ? 0.5f + 0.3f * sinf(ImGui::GetTime() * 3 + 1) : 0.0f;
    ImGui::ProgressBar(rx_level, ImVec2(-1, 0), "");

    // Refresh audio devices button
    ImGui::Spacing();
    if (ImGui::SmallButton("Refresh Audio Devices")) {
        enumerateAudioDevices();
    }

    return event;
}

} // namespace gui
} // namespace ultra
