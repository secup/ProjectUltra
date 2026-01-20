// Channel Test Tool - Pass test signal through HF channel simulator
// Usage: ./channel_test <input.f32> <output_dir> [seed]
//
// Generates simulated audio files for each channel condition at multiple SNR levels

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

#include "sim/hf_channel.hpp"

using namespace ultra;
using namespace ultra::sim;

// Read f32 file
std::vector<float> readF32(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    size_t size = f.tellg();
    f.seekg(0);
    std::vector<float> data(size / sizeof(float));
    f.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

// Write f32 file
bool writeF32(const char* path, const std::vector<float>& data) {
    std::ofstream f(path, std::ios::binary);
    if (!f) return false;
    f.write(reinterpret_cast<const char*>(data.data()), data.size() * sizeof(float));
    return true;
}

// Calculate RMS of signal
float calculateRMS(const std::vector<float>& data) {
    float sum = 0;
    for (float s : data) sum += s * s;
    return std::sqrt(sum / data.size());
}

// Run channel test and return success count
struct TestResult {
    int frames_decoded;
    int frames_expected;
    float measured_snr;
};

TestResult runDecode(const char* filepath) {
    TestResult result = {0, 3, 0.0f};  // We expect 3 frames

    // Run ultra prx and capture output
    char cmd[512];
    snprintf(cmd, sizeof(cmd),
             "./ultra -m dqpsk -c 1/4 prx %s 2>&1 | grep -E 'Frames received|SNR'",
             filepath);

    FILE* pipe = popen(cmd, "r");
    if (!pipe) return result;

    char line[256];
    while (fgets(line, sizeof(line), pipe)) {
        if (strstr(line, "Frames received:")) {
            sscanf(line, "  Frames received: %d", &result.frames_decoded);
        }
        if (strstr(line, "SNR=")) {
            // Extract SNR from status line
            char* snr_pos = strstr(line, "SNR=");
            if (snr_pos) sscanf(snr_pos, "SNR=%f", &result.measured_snr);
        }
    }
    pclose(pipe);

    return result;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Channel Test Tool - HF channel simulation for modem testing\n\n";
        std::cerr << "Usage: " << argv[0] << " <input.f32> <output_dir> [seed]\n\n";
        std::cerr << "Generates test files for each channel condition:\n";
        std::cerr << "  - AWGN (baseline)\n";
        std::cerr << "  - Good (0.5ms delay, 0.1Hz doppler)\n";
        std::cerr << "  - Moderate (1.0ms delay, 0.5Hz doppler)\n";
        std::cerr << "  - Poor (2.0ms delay, 1.0Hz doppler)\n";
        std::cerr << "  - Flutter (0.5ms delay, 10Hz doppler)\n\n";
        std::cerr << "SNR levels tested: 20, 15, 12, 10, 8, 6, 4 dB\n";
        return 1;
    }

    const char* input_path = argv[1];
    const char* output_dir = argv[2];
    uint32_t seed = (argc > 3) ? std::atoi(argv[3]) : 42;

    // Read input signal
    auto input = readF32(input_path);
    if (input.empty()) {
        std::cerr << "Error: Cannot read " << input_path << "\n";
        return 1;
    }

    float input_rms = calculateRMS(input);
    std::cout << "Input: " << input.size() << " samples, RMS=" << input_rms << "\n";
    std::cout << "Seed: " << seed << "\n\n";

    // Create output directory
    mkdir(output_dir, 0755);

    // Channel conditions to test
    struct ChannelDef {
        const char* name;
        WattersonChannel::Config (*factory)(float);
    };

    ChannelDef channels[] = {
        {"awgn", itu_r_f1487::awgn},
        {"good", itu_r_f1487::good},
        {"moderate", itu_r_f1487::moderate},
        {"poor", itu_r_f1487::poor},
        {"flutter", itu_r_f1487::flutter},
    };

    // SNR levels to test
    float snr_levels[] = {20, 15, 12, 10, 8, 6, 4};
    int num_snr = sizeof(snr_levels) / sizeof(snr_levels[0]);

    // Results table
    std::cout << "=== Generating Channel Test Files ===\n\n";
    std::cout << "Channel     | SNR (dB) | Output File\n";
    std::cout << "------------|----------|------------------------------------------\n";

    for (const auto& ch : channels) {
        for (int i = 0; i < num_snr; i++) {
            float snr = snr_levels[i];

            // Create channel with config
            auto config = ch.factory(snr);
            WattersonChannel channel(config, seed);

            // Process signal
            SampleSpan span(input.data(), input.size());
            Samples output = channel.process(span);

            // Generate output filename
            char outpath[512];
            snprintf(outpath, sizeof(outpath), "%s/%s_%02.0fdb.f32",
                     output_dir, ch.name, snr);

            // Write output
            std::vector<float> out_vec(output.begin(), output.end());
            if (writeF32(outpath, out_vec)) {
                printf("%-11s | %5.0f    | %s\n", ch.name, snr, outpath);
            } else {
                printf("%-11s | %5.0f    | ERROR writing file\n", ch.name, snr);
            }
        }
    }

    std::cout << "\n=== Running Decode Tests ===\n\n";
    std::cout << "Channel     | SNR (dB) | Frames | Result\n";
    std::cout << "------------|----------|--------|--------\n";

    int total_pass = 0, total_tests = 0;

    for (const auto& ch : channels) {
        for (int i = 0; i < num_snr; i++) {
            float snr = snr_levels[i];

            char filepath[512];
            snprintf(filepath, sizeof(filepath), "%s/%s_%02.0fdb.f32",
                     output_dir, ch.name, snr);

            TestResult result = runDecode(filepath);
            total_tests++;

            const char* status = (result.frames_decoded == result.frames_expected) ? "PASS" : "FAIL";
            if (result.frames_decoded == result.frames_expected) total_pass++;

            printf("%-11s | %5.0f    | %d/%d    | %s\n",
                   ch.name, snr, result.frames_decoded, result.frames_expected, status);
        }
    }

    std::cout << "\n=== Summary ===\n";
    std::cout << "Passed: " << total_pass << "/" << total_tests << " tests\n";
    std::cout << "Seed used: " << seed << "\n";

    if (total_pass < total_tests) {
        std::cout << "\nTip: Try different seeds to find better/worse conditions:\n";
        std::cout << "  " << argv[0] << " " << input_path << " " << output_dir << " 123\n";
    }

    return (total_pass == total_tests) ? 0 : 1;
}
