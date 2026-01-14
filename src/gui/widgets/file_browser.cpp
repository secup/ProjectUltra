#include "file_browser.hpp"
#include "imgui.h"
#include <algorithm>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

namespace ultra {
namespace gui {

FileBrowser::FileBrowser() {
    current_path_ = getHomeDirectory();
}

std::string FileBrowser::getHomeDirectory() {
#ifdef _WIN32
    const char* home = std::getenv("USERPROFILE");
    if (home) return home;
    const char* drive = std::getenv("HOMEDRIVE");
    const char* path = std::getenv("HOMEPATH");
    if (drive && path) return std::string(drive) + path;
    return "C:\\";
#else
    const char* home = std::getenv("HOME");
    return home ? home : "/";
#endif
}

void FileBrowser::open(const std::string& start_path) {
    visible_ = true;
    selected_index_ = -1;
    selected_path_.clear();

    if (!start_path.empty() && fs::exists(start_path)) {
        if (fs::is_directory(start_path)) {
            current_path_ = start_path;
        } else {
            current_path_ = fs::path(start_path).parent_path().string();
        }
    } else if (current_path_.empty() || !fs::exists(current_path_)) {
        current_path_ = getHomeDirectory();
    }

    refreshEntries();
}

void FileBrowser::navigateTo(const std::string& path) {
    if (fs::exists(path) && fs::is_directory(path)) {
        current_path_ = path;
        selected_index_ = -1;
        refreshEntries();
    }
}

void FileBrowser::navigateUp() {
    fs::path p(current_path_);
    if (p.has_parent_path() && p.parent_path() != p) {
        navigateTo(p.parent_path().string());
    }
}

void FileBrowser::refreshEntries() {
    entries_.clear();

    try {
        for (const auto& entry : fs::directory_iterator(current_path_)) {
            Entry e;
            e.name = entry.path().filename().string();
            e.full_path = entry.path().string();
            e.is_directory = entry.is_directory();
            e.size = 0;

            // Skip hidden files (starting with .)
            if (!e.name.empty() && e.name[0] == '.') {
                continue;
            }

            if (!e.is_directory) {
                try {
                    e.size = entry.file_size();
                } catch (...) {}

                // Apply filter if set
                if (!filter_.empty()) {
                    std::string ext = entry.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    std::string flt = filter_;
                    std::transform(flt.begin(), flt.end(), flt.begin(), ::tolower);
                    if (ext != flt) {
                        continue;
                    }
                }
            }

            entries_.push_back(e);
        }
    } catch (...) {
        // Permission denied or other error
    }

    // Sort: directories first, then by name
    std::sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) {
        if (a.is_directory != b.is_directory) {
            return a.is_directory > b.is_directory;
        }
        return a.name < b.name;
    });
}

std::string FileBrowser::formatSize(size_t bytes) {
    if (bytes < 1024) return std::to_string(bytes) + " B";
    if (bytes < 1024 * 1024) return std::to_string(bytes / 1024) + " KB";
    if (bytes < 1024 * 1024 * 1024) return std::to_string(bytes / (1024 * 1024)) + " MB";
    return std::to_string(bytes / (1024 * 1024 * 1024)) + " GB";
}

bool FileBrowser::render() {
    if (!visible_) return false;

    bool file_selected = false;

    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    if (ImGui::Begin(title_.c_str(), &visible_, ImGuiWindowFlags_NoCollapse)) {

        // Current path display
        ImGui::Text("Path: %s", current_path_.c_str());

        // Navigation buttons
        if (ImGui::Button("Up")) {
            navigateUp();
        }
        ImGui::SameLine();
        if (ImGui::Button("Home")) {
            navigateTo(getHomeDirectory());
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) {
            refreshEntries();
        }

        ImGui::Separator();

        // File list
        ImVec2 list_size(-1, -60);  // Leave room for buttons at bottom
        if (ImGui::BeginChild("FileList", list_size, true)) {
            for (size_t i = 0; i < entries_.size(); i++) {
                const auto& entry = entries_[i];

                // Icon prefix
                const char* icon = entry.is_directory ? "[D] " : "    ";

                // Selectable entry
                char label[512];
                if (entry.is_directory) {
                    snprintf(label, sizeof(label), "%s%s/", icon, entry.name.c_str());
                } else {
                    snprintf(label, sizeof(label), "%s%-40s %10s",
                             icon, entry.name.c_str(), formatSize(entry.size).c_str());
                }

                bool is_selected = (static_cast<int>(i) == selected_index_);
                if (ImGui::Selectable(label, is_selected, ImGuiSelectableFlags_AllowDoubleClick)) {
                    selected_index_ = static_cast<int>(i);

                    if (ImGui::IsMouseDoubleClicked(0)) {
                        if (entry.is_directory) {
                            navigateTo(entry.full_path);
                        } else {
                            selected_path_ = entry.full_path;
                            file_selected = true;
                            visible_ = false;
                        }
                    }
                }
            }
        }
        ImGui::EndChild();

        ImGui::Separator();

        // Selected file display
        if (selected_index_ >= 0 && selected_index_ < static_cast<int>(entries_.size())) {
            const auto& entry = entries_[selected_index_];
            if (!entry.is_directory) {
                ImGui::Text("Selected: %s", entry.name.c_str());
            }
        }

        // Action buttons
        bool can_select = selected_index_ >= 0 &&
                         selected_index_ < static_cast<int>(entries_.size()) &&
                         !entries_[selected_index_].is_directory;

        ImGui::BeginDisabled(!can_select);
        if (ImGui::Button("Select", ImVec2(100, 0))) {
            selected_path_ = entries_[selected_index_].full_path;
            file_selected = true;
            visible_ = false;
        }
        ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100, 0))) {
            visible_ = false;
        }
    }
    ImGui::End();

    return file_selected;
}

} // namespace gui
} // namespace ultra
