#pragma once

#include "ultra/types.hpp"
#include "../modem/modem_engine.hpp"

namespace ultra {
namespace gui {

// Channel Status panel - displays real-time modem statistics
class ControlsWidget {
public:
    enum class Event {
        None
    };

    ControlsWidget() = default;

    // Render channel status panel
    // data_mod/data_rate are the protocol's negotiated mode (displayed as current mode)
    Event render(const LoopbackStats& stats, ModemConfig& config,
                 Modulation data_mod, CodeRate data_rate);
};

} // namespace gui
} // namespace ultra
