#pragma once

#include "ultra/types.hpp"
#include "imgui.h"
#include <vector>
#include <string>

namespace ultra {
namespace gui {

class ControlsWidget {
public:
    enum class Event {
        None,
        Connect,
        Disconnect,
        ProfileChanged,
        AudioDeviceChanged
    };

    ControlsWidget();

    Event render(ModemConfig& config, bool connected);

private:
    std::vector<std::string> audio_devices_;
    int selected_device_ = 0;
    int selected_profile_ = 1;  // Default to Balanced

    void enumerateAudioDevices();
};

} // namespace gui
} // namespace ultra
