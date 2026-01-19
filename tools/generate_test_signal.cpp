// Generate comprehensive test signal for F7 injection test
// Creates sequence: frame1 -> noise gap -> frame2 -> noise gap -> ... -> frameN
// Gaps are random duration between MIN_GAP_MS and MAX_GAP_MS
//
// Usage: ./generate_test_signal <output.f32> [frame1.f32] [frame2.f32] ...
//    or: ./generate_test_signal --generate-all <output.f32>
//
// The --generate-all option generates all frame types automatically

#include <fstream>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <random>
#include <cstring>

constexpr int SAMPLE_RATE = 48000;
constexpr int MIN_GAP_MS = 5000;   // 5 seconds minimum
constexpr int MAX_GAP_MS = 8000;   // 8 seconds maximum
constexpr float GAP_NOISE_LEVEL = 0.01f;  // Low noise floor during gap

std::vector<float> readF32File(const char* filename) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) {
        std::cerr << "Error: Cannot open " << filename << "\n";
        return {};
    }
    f.seekg(0, std::ios::end);
    size_t n = f.tellg() / sizeof(float);
    f.seekg(0);
    std::vector<float> samples(n);
    f.read(reinterpret_cast<char*>(samples.data()), n * sizeof(float));
    return samples;
}

void generateNoise(std::vector<float>& output, size_t samples, std::mt19937& rng, float level) {
    std::normal_distribution<float> noise_dist(0.0f, level);
    for (size_t i = 0; i < samples; i++) {
        output.push_back(noise_dist(rng));
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Generate comprehensive test signal for F7 injection test\n\n";
        std::cerr << "Usage:\n";
        std::cerr << "  " << argv[0] << " <output.f32> <frame1.f32> [frame2.f32] ...\n";
        std::cerr << "  " << argv[0] << " --generate-all <output.f32>\n\n";
        std::cerr << "Options:\n";
        std::cerr << "  --generate-all  Auto-generate all frame types using ultra CLI\n\n";
        std::cerr << "Examples:\n";
        std::cerr << "  # Manual: combine pre-generated frames\n";
        std::cerr << "  ./generate_test_signal out.f32 connect.f32 data.f32 disconnect.f32\n\n";
        std::cerr << "  # Auto: generate all frame types\n";
        std::cerr << "  ./generate_test_signal --generate-all tests/data/test_connect_data_sequence.f32\n";
        return 1;
    }

    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_int_distribution<int> gap_dist(MIN_GAP_MS, MAX_GAP_MS);

    std::vector<float> output;
    const char* output_file;
    std::vector<std::string> frame_files;
    std::vector<std::string> frame_names;

    if (strcmp(argv[1], "--generate-all") == 0) {
        if (argc < 3) {
            std::cerr << "Error: --generate-all requires output filename\n";
            return 1;
        }
        output_file = argv[2];

        // Generate all frame types using ultra CLI
        std::cout << "Generating all frame types...\n";

        struct FrameDef {
            const char* cmd;
            const char* name;
            const char* tmpfile;
        };

        FrameDef frames[] = {
            {"./ultra -m dqpsk -c 1/4 -s TEST -d OTHER ptx connect 2>/dev/null > /tmp/gen_connect.f32",
             "CONNECT", "/tmp/gen_connect.f32"},
            {"./ultra -m dqpsk -c 1/4 -s TEST -d OTHER ptx \"Hello from ULTRA!\" 2>/dev/null > /tmp/gen_data.f32",
             "DATA", "/tmp/gen_data.f32"},
            {"./ultra -m dqpsk -c 1/4 -s TEST -d OTHER ptx disconnect 2>/dev/null > /tmp/gen_disconnect.f32",
             "DISCONNECT", "/tmp/gen_disconnect.f32"},
        };

        for (const auto& fd : frames) {
            std::cout << "  Generating " << fd.name << "...\n";
            int ret = system(fd.cmd);
            if (ret != 0) {
                std::cerr << "Error generating " << fd.name << "\n";
                return 1;
            }
            frame_files.push_back(fd.tmpfile);
            frame_names.push_back(fd.name);
        }
    } else {
        output_file = argv[1];
        for (int i = 2; i < argc; i++) {
            frame_files.push_back(argv[i]);
            frame_names.push_back(argv[i]);
        }
    }

    if (frame_files.empty()) {
        std::cerr << "Error: No frame files specified\n";
        return 1;
    }

    // Build output: frame1 + gap + frame2 + gap + ... + frameN
    for (size_t i = 0; i < frame_files.size(); i++) {
        // Read frame
        auto samples = readF32File(frame_files[i].c_str());
        if (samples.empty()) return 1;

        std::cout << frame_names[i] << ": " << samples.size() << " samples ("
                  << (samples.size() / (float)SAMPLE_RATE) << "s)\n";

        // Add frame
        output.insert(output.end(), samples.begin(), samples.end());

        // Add noise gap (except after last frame)
        if (i < frame_files.size() - 1) {
            int gap_ms = gap_dist(rng);
            size_t gap_samples = SAMPLE_RATE * gap_ms / 1000;
            generateNoise(output, gap_samples, rng, GAP_NOISE_LEVEL);
            std::cout << "  + " << gap_ms << "ms noise gap (" << gap_samples << " samples)\n";
        }
    }

    // Write output
    std::ofstream fo(output_file, std::ios::binary);
    if (!fo) {
        std::cerr << "Error: Cannot create " << output_file << "\n";
        return 1;
    }
    fo.write(reinterpret_cast<const char*>(output.data()), output.size() * sizeof(float));
    fo.close();

    std::cout << "\nWrote " << output.size() << " samples ("
              << (output.size() / (float)SAMPLE_RATE) << "s) to " << output_file << "\n";

    return 0;
}
