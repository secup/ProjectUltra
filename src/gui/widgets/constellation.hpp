#pragma once

#include "ultra/types.hpp"
#include "imgui.h"
#include <vector>
#include <complex>

namespace ultra {
namespace gui {

class ConstellationWidget {
public:
    void render(const std::vector<std::complex<float>>& symbols, Modulation mod);

private:
    void drawGrid(ImDrawList* draw_list, ImVec2 center, float size, Modulation mod);
    void drawSymbols(ImDrawList* draw_list, ImVec2 center, float size,
                     const std::vector<std::complex<float>>& symbols);
};

} // namespace gui
} // namespace ultra
