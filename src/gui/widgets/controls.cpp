#include "controls.hpp"
#include "imgui.h"
#include <cstdio>

namespace ultra {
namespace gui {

// Helper to get modulation name
static const char* getModulationName(Modulation mod) {
    switch (mod) {
        case Modulation::BPSK:   return "BPSK";
        case Modulation::QPSK:   return "QPSK";
        case Modulation::DQPSK:  return "DQPSK";
        case Modulation::QAM16:  return "16-QAM";
        case Modulation::QAM64:  return "64-QAM";
        case Modulation::QAM256: return "256-QAM";
        default: return "Unknown";
    }
}

// Helper to get code rate name
static const char* getCodeRateName(CodeRate rate) {
    switch (rate) {
        case CodeRate::R1_4: return "1/4";
        case CodeRate::R1_2: return "1/2";
        case CodeRate::R2_3: return "2/3";
        case CodeRate::R3_4: return "3/4";
        case CodeRate::R5_6: return "5/6";
        case CodeRate::R7_8: return "7/8";
        default: return "?";
    }
}

ControlsWidget::Event ControlsWidget::render(const LoopbackStats& stats, ModemConfig& config,
                                             Modulation data_mod, CodeRate data_rate) {
    Event event = Event::None;

    ImGui::Text("Channel Status");
    ImGui::Separator();
    ImGui::Spacing();

    // Sync status indicator
    ImVec4 sync_color = stats.synced ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) : ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
    ImGui::TextColored(sync_color, "%s", stats.synced ? "SYNCED" : "NO SYNC");
    ImGui::SameLine();
    ImGui::TextDisabled("| Signal Lock");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // SNR display with visual bar
    ImGui::Text("SNR");
    ImGui::SameLine(80);

    // SNR bar: 0-40 dB range
    float snr_normalized = stats.snr_db / 40.0f;
    snr_normalized = std::max(0.0f, std::min(1.0f, snr_normalized));

    // Color based on SNR quality
    ImVec4 snr_color;
    if (stats.snr_db >= 20.0f) {
        snr_color = ImVec4(0.2f, 1.0f, 0.2f, 1.0f);  // Green - excellent
    } else if (stats.snr_db >= 10.0f) {
        snr_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f);  // Yellow - good
    } else {
        snr_color = ImVec4(1.0f, 0.3f, 0.3f, 1.0f);  // Red - poor
    }

    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, snr_color);
    char snr_text[32];
    snprintf(snr_text, sizeof(snr_text), "%.1f dB", stats.snr_db);
    ImGui::ProgressBar(snr_normalized, ImVec2(-1, 18), snr_text);
    ImGui::PopStyleColor();

    ImGui::Spacing();

    // BER display - show in human-readable format
    ImGui::Text("BER");
    ImGui::SameLine(80);
    if (stats.ber > 0.0f && stats.ber < 1.0f) {
        // Format BER nicely: "1 in 1,000,000" style or percentage
        if (stats.ber < 0.0001f) {
            // Very low BER - show as "< 0.01%"
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "< 0.01%%");
        } else if (stats.ber < 0.01f) {
            // Low BER - show as percentage with 2 decimals
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "%.2f%%", stats.ber * 100.0f);
        } else if (stats.ber < 0.1f) {
            // Medium BER
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.2f, 1.0f), "%.1f%%", stats.ber * 100.0f);
        } else {
            // High BER
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "%.0f%%", stats.ber * 100.0f);
        }
    } else {
        ImGui::TextDisabled("--");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Current modulation and code rate (from protocol negotiation)
    ImGui::Text("Modulation");
    ImGui::SameLine(80);
    ImGui::Text("%s R%s", getModulationName(data_mod), getCodeRateName(data_rate));

    // Throughput
    ImGui::Text("Throughput");
    ImGui::SameLine(80);
    if (stats.throughput_bps >= 1000) {
        ImGui::Text("%.1f kbps", stats.throughput_bps / 1000.0f);
    } else {
        ImGui::Text("%d bps", stats.throughput_bps);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Frame statistics
    ImGui::Text("Frame Statistics");
    ImGui::Spacing();

    // TX frames
    ImGui::TextDisabled("TX:");
    ImGui::SameLine(50);
    ImGui::Text("%d", stats.frames_sent);

    // RX frames (good + failed)
    ImGui::TextDisabled("RX:");
    ImGui::SameLine(50);
    ImGui::Text("%d", stats.frames_received);
    if (stats.frames_failed > 0) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "(%d failed)", stats.frames_failed);
    }

    // Success rate
    int total_rx = stats.frames_received + stats.frames_failed;
    if (total_rx > 0) {
        float success_rate = 100.0f * stats.frames_received / total_rx;
        ImGui::TextDisabled("Success:");
        ImGui::SameLine(70);
        ImVec4 rate_color = (success_rate >= 90.0f) ? ImVec4(0.2f, 1.0f, 0.2f, 1.0f) :
                           (success_rate >= 70.0f) ? ImVec4(1.0f, 1.0f, 0.2f, 1.0f) :
                                                     ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
        ImGui::TextColored(rate_color, "%.0f%%", success_rate);
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Speed profile selection (keep this - useful to change on the fly)
    ImGui::Text("Speed Profile");
    const char* profiles[] = { "Conservative", "Balanced", "Turbo", "Adaptive" };

    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##profile", profiles[selected_profile_])) {
        for (int i = 0; i < 4; ++i) {
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
    const char* profile_detail;
    switch (selected_profile_) {
        case 0: profile_detail = "QPSK R1/2"; break;
        case 1: profile_detail = "64-QAM R3/4"; break;
        case 2: profile_detail = "256-QAM R7/8"; break;
        case 3: profile_detail = "AUTO - SNR based"; break;
        default: profile_detail = "Unknown"; break;
    }

    if (selected_profile_ == 3) {
        // Adaptive mode - show in cyan
        ImGui::TextColored(ImVec4(0.3f, 0.9f, 1.0f, 1.0f), "(%s)", profile_detail);
    } else {
        ImGui::TextDisabled("(%s)", profile_detail);
    }

    return event;
}

} // namespace gui
} // namespace ultra
