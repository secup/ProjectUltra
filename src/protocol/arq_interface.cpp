#include "arq_interface.hpp"
#include "arq.hpp"                  // For StopAndWaitARQ
#include "selective_repeat_arq.hpp" // For SelectiveRepeatARQ

namespace ultra {
namespace protocol {

const char* arqModeToString(ARQMode mode) {
    switch (mode) {
        case ARQMode::STOP_AND_WAIT:    return "Stop-and-Wait";
        case ARQMode::SELECTIVE_REPEAT: return "Selective Repeat";
        default:                        return "Unknown";
    }
}

std::unique_ptr<IARQController> createARQController(ARQMode mode, const ARQConfig& config) {
    switch (mode) {
        case ARQMode::STOP_AND_WAIT:
            return std::make_unique<StopAndWaitARQ>(config);

        case ARQMode::SELECTIVE_REPEAT:
            return std::make_unique<SelectiveRepeatARQ>(config);

        default:
            return std::make_unique<StopAndWaitARQ>(config);
    }
}

} // namespace protocol
} // namespace ultra
