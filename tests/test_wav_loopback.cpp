/**
 * WAV File Loopback Test
 *
 * Generate known OFDM signals to WAV files, and decode WAV files.
 * Use this to test audio path between laptops without live processing.
 *
 * Usage:
 *   ./test_wav_loopback --generate test_tx.wav    # Generate test signal
 *   ./test_wav_loopback --decode test_rx.wav      # Decode received signal
 *   ./test_wav_loopback --loopback                # Software loopback test
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <cmath>

#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/types.hpp"

using namespace ultra;

// Simple WAV file writer (16-bit mono)
bool writeWav(const std::string& filename, const std::vector<float>& samples, int sample_rate = 48000) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return false;
    }

    // Convert to 16-bit
    std::vector<int16_t> samples16(samples.size());
    for (size_t i = 0; i < samples.size(); i++) {
        float s = std::max(-1.0f, std::min(1.0f, samples[i]));
        samples16[i] = static_cast<int16_t>(s * 32767.0f);
    }

    // WAV header
    uint32_t data_size = samples16.size() * sizeof(int16_t);
    uint32_t file_size = 36 + data_size;
    uint16_t channels = 1;
    uint16_t bits_per_sample = 16;
    uint32_t byte_rate = sample_rate * channels * bits_per_sample / 8;
    uint16_t block_align = channels * bits_per_sample / 8;

    file.write("RIFF", 4);
    file.write(reinterpret_cast<char*>(&file_size), 4);
    file.write("WAVE", 4);
    file.write("fmt ", 4);
    uint32_t fmt_size = 16;
    file.write(reinterpret_cast<char*>(&fmt_size), 4);
    uint16_t audio_format = 1; // PCM
    file.write(reinterpret_cast<char*>(&audio_format), 2);
    file.write(reinterpret_cast<char*>(&channels), 2);
    file.write(reinterpret_cast<char*>(&sample_rate), 4);
    file.write(reinterpret_cast<char*>(&byte_rate), 4);
    file.write(reinterpret_cast<char*>(&block_align), 2);
    file.write(reinterpret_cast<char*>(&bits_per_sample), 2);
    file.write("data", 4);
    file.write(reinterpret_cast<char*>(&data_size), 4);
    file.write(reinterpret_cast<char*>(samples16.data()), data_size);

    std::cout << "Wrote " << filename << ": " << samples.size() << " samples, "
              << (samples.size() / (float)sample_rate) << " seconds\n";
    return true;
}

// Simple WAV file reader (16-bit mono)
bool readWav(const std::string& filename, std::vector<float>& samples, int& sample_rate) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open " << filename << " for reading\n";
        return false;
    }

    // Read header
    char riff[4], wave[4], fmt[4], data_marker[4];
    uint32_t file_size, fmt_size, data_size;
    uint16_t audio_format, channels, bits_per_sample, block_align;
    uint32_t byte_rate;

    file.read(riff, 4);
    file.read(reinterpret_cast<char*>(&file_size), 4);
    file.read(wave, 4);

    if (strncmp(riff, "RIFF", 4) != 0 || strncmp(wave, "WAVE", 4) != 0) {
        std::cerr << "Not a valid WAV file\n";
        return false;
    }

    file.read(fmt, 4);
    file.read(reinterpret_cast<char*>(&fmt_size), 4);
    file.read(reinterpret_cast<char*>(&audio_format), 2);
    file.read(reinterpret_cast<char*>(&channels), 2);
    file.read(reinterpret_cast<char*>(&sample_rate), 4);
    file.read(reinterpret_cast<char*>(&byte_rate), 4);
    file.read(reinterpret_cast<char*>(&block_align), 2);
    file.read(reinterpret_cast<char*>(&bits_per_sample), 2);

    // Skip any extra fmt bytes
    if (fmt_size > 16) {
        file.seekg(fmt_size - 16, std::ios::cur);
    }

    // Find data chunk
    while (file.read(data_marker, 4)) {
        file.read(reinterpret_cast<char*>(&data_size), 4);
        if (strncmp(data_marker, "data", 4) == 0) {
            break;
        }
        file.seekg(data_size, std::ios::cur);
    }

    if (strncmp(data_marker, "data", 4) != 0) {
        std::cerr << "No data chunk found\n";
        return false;
    }

    // Read samples
    if (bits_per_sample == 16) {
        size_t num_samples = data_size / (channels * 2);
        std::vector<int16_t> samples16(num_samples * channels);
        file.read(reinterpret_cast<char*>(samples16.data()), data_size);

        samples.resize(num_samples);
        for (size_t i = 0; i < num_samples; i++) {
            // Take first channel if stereo
            samples[i] = samples16[i * channels] / 32768.0f;
        }
    } else {
        std::cerr << "Unsupported bit depth: " << bits_per_sample << "\n";
        return false;
    }

    std::cout << "Read " << filename << ": " << samples.size() << " samples @ "
              << sample_rate << " Hz, " << (samples.size() / (float)sample_rate) << " seconds\n";
    return true;
}

// Generate test signal: preamble + LDPC-encoded all-zeros (DQPSK)
std::vector<float> generateTestSignal(ModemConfig& config) {
    // Use DQPSK, R1/4 - 2x throughput of DBPSK, immune to phase distortion
    config.modulation = Modulation::DQPSK;
    config.code_rate = CodeRate::R1_4;
    config.pilot_spacing = 2;  // Dense pilots for robustness

    OFDMModulator modulator(config);
    LDPCEncoder encoder(config.code_rate);

    // Generate test data: all zeros
    // With R1/4, k=162 bits = 20.25 bytes, we send 21 bytes (padded)
    Bytes test_data(21, 0x00);

    std::cout << "Test data: " << test_data.size() << " bytes of 0x00\n";

    // LDPC encode
    Bytes encoded = encoder.encode(test_data);
    std::cout << "LDPC encoded: " << encoded.size() << " bytes\n";

    // Generate preamble
    Samples preamble = modulator.generatePreamble();
    std::cout << "Preamble: " << preamble.size() << " samples\n";

    // Modulate
    Samples data_samples = modulator.modulate(encoded, config.modulation);
    std::cout << "Data: " << data_samples.size() << " samples\n";

    // Combine with silence padding
    std::vector<float> output;

    // Add 0.5s silence at start
    size_t silence_samples = config.sample_rate / 2;
    output.insert(output.end(), silence_samples, 0.0f);

    // Add preamble (Samples is already std::vector<float>)
    output.insert(output.end(), preamble.begin(), preamble.end());

    // Add data (Samples is already std::vector<float>)
    output.insert(output.end(), data_samples.begin(), data_samples.end());

    // Add 0.5s silence at end
    output.insert(output.end(), silence_samples, 0.0f);

    // Normalize to 0.5 amplitude (leave headroom)
    float max_val = 0;
    for (float s : output) max_val = std::max(max_val, std::abs(s));
    if (max_val > 0) {
        float scale = 0.5f / max_val;
        for (float& s : output) s *= scale;
    }

    std::cout << "Total output: " << output.size() << " samples, "
              << (output.size() / (float)config.sample_rate) << " seconds\n";

    return output;
}

// Decode a WAV file
bool decodeWavFile(const std::string& filename) {
    std::vector<float> samples;
    int sample_rate;

    if (!readWav(filename, samples, sample_rate)) {
        return false;
    }

    if (sample_rate != 48000) {
        std::cerr << "Warning: expected 48000 Hz, got " << sample_rate << " Hz\n";
    }

    // Setup demodulator
    ModemConfig config;
    config.modulation = Modulation::DQPSK;  // DQPSK for link frames
    config.code_rate = CodeRate::R1_4;
    config.pilot_spacing = 2;

    OFDMDemodulator demodulator(config);
    LDPCDecoder decoder(config.code_rate);

    // Process samples - feed once and collect soft bits
    std::cout << "\nProcessing " << samples.size() << " samples...\n";

    SampleSpan span(samples.data(), samples.size());
    std::vector<float> all_soft_bits;

    // Feed all samples to demodulator
    bool got_data = demodulator.process(span);

    if (got_data) {
        // Get soft bits (one codeword at a time)
        auto soft_bits = demodulator.getSoftBits();
        all_soft_bits.insert(all_soft_bits.end(), soft_bits.begin(), soft_bits.end());
        std::cout << "Got " << soft_bits.size() << " soft bits\n";

        // Try to get more (in case there are multiple codewords)
        for (int i = 0; i < 10; i++) {
            soft_bits = demodulator.getSoftBits();
            if (soft_bits.empty()) break;
            all_soft_bits.insert(all_soft_bits.end(), soft_bits.begin(), soft_bits.end());
            std::cout << "Got " << soft_bits.size() << " more soft bits (total: " << all_soft_bits.size() << ")\n";
        }
    } else {
        std::cout << "No sync found in signal!\n";
    }

    std::cout << "\nTotal soft bits: " << all_soft_bits.size() << "\n";

    if (all_soft_bits.size() < 648) {
        std::cerr << "Not enough soft bits for LDPC decode (need 648, got "
                  << all_soft_bits.size() << ")\n";
        return false;
    }

    // Analyze LLRs
    int positive = 0, negative = 0;
    float sum_abs = 0;
    for (size_t i = 0; i < std::min(all_soft_bits.size(), size_t(100)); i++) {
        if (all_soft_bits[i] > 0) positive++;
        else negative++;
        sum_abs += std::abs(all_soft_bits[i]);
    }
    std::cout << "First 100 LLRs: " << positive << " positive (bit 0), "
              << negative << " negative (bit 1), avg |LLR|=" << (sum_abs/100) << "\n";

    std::cout << "First 8 LLRs: ";
    for (int i = 0; i < 8 && i < (int)all_soft_bits.size(); i++) {
        printf("%.2f ", all_soft_bits[i]);
    }
    std::cout << "\n";

    // LDPC decode
    std::vector<float> codeword_bits(all_soft_bits.begin(),
                                      all_soft_bits.begin() + std::min(all_soft_bits.size(), size_t(648)));

    Bytes decoded = decoder.decodeSoft(codeword_bits);

    if (decoded.empty()) {
        std::cout << "\n[FAIL] LDPC decode failed!\n";
        return false;
    }

    std::cout << "\n[PASS] LDPC decode succeeded! Got " << decoded.size() << " bytes\n";

    // Check if all zeros
    int zero_count = 0;
    for (uint8_t b : decoded) {
        if (b == 0x00) zero_count++;
    }
    std::cout << "Zero bytes: " << zero_count << "/" << decoded.size()
              << " (" << (100.0 * zero_count / decoded.size()) << "%)\n";

    if (zero_count == (int)decoded.size()) {
        std::cout << "[PASS] All bytes are 0x00 as expected!\n";
        return true;
    } else {
        std::cout << "[PARTIAL] Some bytes are not 0x00\n";
        std::cout << "First 20 bytes: ";
        for (int i = 0; i < 20 && i < (int)decoded.size(); i++) {
            printf("%02x ", decoded[i]);
        }
        std::cout << "\n";
        return false;
    }
}

// Software loopback test (no audio hardware)
bool softwareLoopbackTest() {
    std::cout << "\n=== Software Loopback Test ===\n";

    ModemConfig config;
    std::vector<float> tx_samples = generateTestSignal(config);

    // Write to temp file and read back (tests WAV I/O)
    std::string temp_file = "/tmp/test_loopback.wav";
    if (!writeWav(temp_file, tx_samples)) {
        return false;
    }

    return decodeWavFile(temp_file);
}

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "      WAV FILE LOOPBACK TEST\n";
    std::cout << "========================================\n";

    if (argc < 2) {
        std::cout << "\nUsage:\n";
        std::cout << "  " << argv[0] << " --generate <output.wav>  Generate test signal\n";
        std::cout << "  " << argv[0] << " --decode <input.wav>     Decode received signal\n";
        std::cout << "  " << argv[0] << " --loopback               Software loopback test\n";
        std::cout << "\nTest procedure:\n";
        std::cout << "  1. Run: ./test_wav_loopback --generate test_tx.wav\n";
        std::cout << "  2. Play test_tx.wav on TX laptop through audio cable\n";
        std::cout << "  3. Record to test_rx.wav on RX laptop\n";
        std::cout << "  4. Run: ./test_wav_loopback --decode test_rx.wav\n";
        return 1;
    }

    std::string mode = argv[1];

    if (mode == "--generate" && argc >= 3) {
        ModemConfig config;
        std::vector<float> samples = generateTestSignal(config);
        return writeWav(argv[2], samples) ? 0 : 1;
    }
    else if (mode == "--decode" && argc >= 3) {
        return decodeWavFile(argv[2]) ? 0 : 1;
    }
    else if (mode == "--loopback") {
        return softwareLoopbackTest() ? 0 : 1;
    }
    else {
        std::cerr << "Unknown mode or missing arguments\n";
        return 1;
    }
}
