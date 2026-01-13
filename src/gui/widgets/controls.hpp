#pragma once

#include "ultra/types.hpp"
#include "../modem_engine.hpp"

namespace ultra {
namespace gui {

// Channel Status panel - displays real-time modem statistics
class ControlsWidget {
public:
    enum class Event {
        None,
        ProfileChanged
    };

    ControlsWidget() = default;

    // Render channel status panel
    // Returns event if profile changed
    Event render(const LoopbackStats& stats, ModemConfig& config);

private:
    int selected_profile_ = 1;  // Default to Balanced
};

} // namespace gui
} // namespace ultra
