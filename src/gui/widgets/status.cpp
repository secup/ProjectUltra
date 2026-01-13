#include "status.hpp"
#include "imgui.h"

namespace ultra {
namespace gui {

void StatusWidget::render(const ModemStats& stats, const ModemConfig& config,
                          bool connected) {
    // Status bar layout
    float width = ImGui::GetContentRegionAvail().x;
    float item_width = width / 5;

    // SNR
    ImGui::TextColored(
        ImVec4(0.7f, 0.9f, 0.7f, 1.0f),
        "SNR: %.1f dB", stats.current_snr_db
    );
    ImGui::SameLine(item_width);

    // Modulation
    const char* mod_name = "---";
    switch (stats.current_modulation) {
        case Modulation::BPSK:   mod_name = "BPSK"; break;
        case Modulation::QPSK:   mod_name = "QPSK"; break;
        case Modulation::QAM16:  mod_name = "16-QAM"; break;
        case Modulation::QAM64:  mod_name = "64-QAM"; break;
        case Modulation::QAM256: mod_name = "256-QAM"; break;
        default: break;
    }

    const char* rate_name = "---";
    switch (stats.current_code_rate) {
        case CodeRate::R1_4: rate_name = "1/4"; break;
        case CodeRate::R1_3: rate_name = "1/3"; break;
        case CodeRate::R1_2: rate_name = "1/2"; break;
        case CodeRate::R2_3: rate_name = "2/3"; break;
        case CodeRate::R3_4: rate_name = "3/4"; break;
        case CodeRate::R5_6: rate_name = "5/6"; break;
        case CodeRate::R7_8: rate_name = "7/8"; break;
    }

    ImGui::Text("Mode: %s R%s", mod_name, rate_name);
    ImGui::SameLine(item_width * 2);

    // Throughput
    float kbps = stats.throughput_bps / 1000.0f;
    ImGui::Text("Speed: %.1f kbps", kbps);
    ImGui::SameLine(item_width * 3);

    // Frames
    ImGui::Text("Frames: %llu", (unsigned long long)stats.frames_received);
    ImGui::SameLine(item_width * 4);

    // Connection status
    if (connected) {
        ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "CONNECTED");
    } else {
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "IDLE");
    }
}

} // namespace gui
} // namespace ultra
