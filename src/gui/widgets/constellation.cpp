#include "constellation.hpp"
#include "imgui.h"
#include <cmath>

namespace ultra {
namespace gui {

void ConstellationWidget::render(const std::vector<std::complex<float>>& symbols,
                                  Modulation mod) {
    ImGui::Text("Constellation Diagram");
    ImGui::Separator();

    // Get available space
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float size = std::min(avail.x, avail.y) - 20;
    if (size < 100) size = 100;

    // Center position
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImVec2 center(pos.x + avail.x / 2, pos.y + size / 2 + 10);

    // Create invisible button for the canvas area
    ImGui::InvisibleButton("constellation_canvas", ImVec2(avail.x, size + 20));

    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Draw background
    draw_list->AddRectFilled(
        ImVec2(center.x - size/2 - 5, center.y - size/2 - 5),
        ImVec2(center.x + size/2 + 5, center.y + size/2 + 5),
        IM_COL32(20, 20, 25, 255),
        4.0f
    );

    // Draw grid and ideal points
    drawGrid(draw_list, center, size, mod);

    // Draw received symbols
    drawSymbols(draw_list, center, size, symbols);

    // Show modulation type
    const char* mod_name = "Unknown";
    switch (mod) {
        case Modulation::DBPSK:  mod_name = "DBPSK"; break;
        case Modulation::BPSK:   mod_name = "BPSK"; break;
        case Modulation::DQPSK:  mod_name = "DQPSK"; break;
        case Modulation::QPSK:   mod_name = "QPSK"; break;
        case Modulation::D8PSK:  mod_name = "D8PSK"; break;
        case Modulation::QAM16:  mod_name = "16-QAM"; break;
        case Modulation::QAM64:  mod_name = "64-QAM"; break;
        case Modulation::QAM256: mod_name = "256-QAM"; break;
        default: break;
    }
    ImGui::Text("Mode: %s", mod_name);
}

void ConstellationWidget::drawGrid(ImDrawList* draw_list, ImVec2 center, float size,
                                    Modulation mod) {
    float half = size / 2;

    // Draw axes
    ImU32 axis_color = IM_COL32(60, 60, 70, 255);
    draw_list->AddLine(
        ImVec2(center.x - half, center.y),
        ImVec2(center.x + half, center.y),
        axis_color, 1.0f
    );
    draw_list->AddLine(
        ImVec2(center.x, center.y - half),
        ImVec2(center.x, center.y + half),
        axis_color, 1.0f
    );

    // Draw ideal constellation points
    ImU32 ideal_color = IM_COL32(80, 80, 100, 255);
    float point_radius = 3.0f;
    float radius = half * 0.7f;

    // PSK modulations: points on a circle
    if (mod == Modulation::DBPSK || mod == Modulation::BPSK) {
        // BPSK: 2 points on I axis
        draw_list->AddCircleFilled(ImVec2(center.x - radius, center.y), point_radius, ideal_color);
        draw_list->AddCircleFilled(ImVec2(center.x + radius, center.y), point_radius, ideal_color);
    } else if (mod == Modulation::DQPSK || mod == Modulation::QPSK) {
        // QPSK/DQPSK: 4 points at 45°, 135°, 225°, 315°
        for (int i = 0; i < 4; i++) {
            float angle = M_PI / 4 + i * M_PI / 2;  // 45° + i*90°
            float px = center.x + radius * std::cos(angle);
            float py = center.y - radius * std::sin(angle);  // Flip Y
            draw_list->AddCircleFilled(ImVec2(px, py), point_radius, ideal_color);
        }
    } else if (mod == Modulation::D8PSK || mod == Modulation::QAM8) {
        // 8PSK: 8 points evenly spaced
        for (int i = 0; i < 8; i++) {
            float angle = i * M_PI / 4;  // 0°, 45°, 90°, ...
            float px = center.x + radius * std::cos(angle);
            float py = center.y - radius * std::sin(angle);
            draw_list->AddCircleFilled(ImVec2(px, py), point_radius, ideal_color);
        }
    } else {
        // QAM: grid of points
        int points_per_axis = 2;
        switch (mod) {
            case Modulation::QAM16:  points_per_axis = 4; break;
            case Modulation::QAM64:  points_per_axis = 8; break;
            case Modulation::QAM256: points_per_axis = 16; point_radius = 2.0f; break;
            default: points_per_axis = 4;
        }

        for (int yi = 0; yi < points_per_axis; ++yi) {
            for (int xi = 0; xi < points_per_axis; ++xi) {
                float x = (2.0f * xi - points_per_axis + 1) / (points_per_axis - 1);
                float y = (2.0f * yi - points_per_axis + 1) / (points_per_axis - 1);

                float px = center.x + x * half * 0.8f;
                float py = center.y - y * half * 0.8f;

                draw_list->AddCircleFilled(ImVec2(px, py), point_radius, ideal_color);
            }
        }
    }
}

void ConstellationWidget::drawSymbols(ImDrawList* draw_list, ImVec2 center, float size,
                                       const std::vector<std::complex<float>>& symbols) {
    float half = size / 2;
    ImU32 symbol_color = IM_COL32(100, 200, 255, 200);

    for (const auto& sym : symbols) {
        float px = center.x + sym.real() * half * 0.8f;
        float py = center.y - sym.imag() * half * 0.8f;  // Flip Y

        // Clamp to display area
        px = std::max(center.x - half, std::min(center.x + half, px));
        py = std::max(center.y - half, std::min(center.y + half, py));

        draw_list->AddCircleFilled(ImVec2(px, py), 2.0f, symbol_color);
    }
}

} // namespace gui
} // namespace ultra
