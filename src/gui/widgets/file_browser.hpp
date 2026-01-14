#pragma once

#include <string>
#include <vector>
#include <functional>

namespace ultra {
namespace gui {

/**
 * Simple ImGui file browser widget
 * Cross-platform, no external dependencies
 */
class FileBrowser {
public:
    FileBrowser();

    // Show the file browser popup (call each frame when visible)
    // Returns true if a file was selected
    bool render();

    // Open the file browser at a specific path
    void open(const std::string& start_path = "");

    // Close the browser
    void close() { visible_ = false; }

    // Check if browser is open
    bool isVisible() const { return visible_; }

    // Get the selected file path
    const std::string& getSelectedPath() const { return selected_path_; }

    // Set file filter (e.g., ".txt", ".bin")
    void setFilter(const std::string& filter) { filter_ = filter; }

    // Set title
    void setTitle(const std::string& title) { title_ = title; }

private:
    bool visible_ = false;
    std::string title_ = "Select File";
    std::string current_path_;
    std::string selected_path_;
    std::string filter_;

    struct Entry {
        std::string name;
        std::string full_path;
        bool is_directory;
        size_t size;
    };
    std::vector<Entry> entries_;
    int selected_index_ = -1;

    void refreshEntries();
    void navigateTo(const std::string& path);
    void navigateUp();
    std::string getHomeDirectory();
    std::string formatSize(size_t bytes);
};

} // namespace gui
} // namespace ultra
