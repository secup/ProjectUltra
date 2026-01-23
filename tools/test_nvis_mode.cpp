/**
 * NVIS Mode Test - Verify 1024 FFT, 59 carriers, 42 baud mode works
 *
 * Tests the high-speed NVIS mode that matches industry leader performance:
 *   - 1024 FFT (46.875 Hz bin spacing)
 *   - 59 carriers (~2.8 kHz bandwidth)
 *   - ~42 baud symbol rate
 *   - All carriers as data for differential modes (no pilots)
 *
 * Target throughput:
 *   DQPSK R3/4: ~3,900 bps
 *   D8PSK R3/4: ~5,800 bps
 *   16QAM R3/4: ~7,000 bps
 *   32QAM R3/4: ~8,500 bps
 *
 * Build: cmake --build build --target test_nvis_mode
 * Run:   ./build/test_nvis_mode [--snr <dB>] [--trials <n>]
 */

#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <cstring>
#include <cmath>
#include <span>

using namespace ultra;

// Test a single transmission
bool testSingle(ModemConfig config, Modulation mod, CodeRate rate, float snr_db, std::mt19937& rng) {
    // Configure modulation
    bool is_diff = (mod == Modulation::DQPSK || mod == Modulation::D8PSK || mod == Modulation::DBPSK);
    config.use_pilots = !is_diff;
    config.modulation = mod;
    config.code_rate = rate;


    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);

    // Get info bytes based on rate
    size_t info_bytes;
    switch (rate) {
        case CodeRate::R1_4: info_bytes = 162 / 8; break;
        case CodeRate::R1_2: info_bytes = 324 / 8; break;
        case CodeRate::R2_3: info_bytes = 432 / 8; break;
        case CodeRate::R3_4: info_bytes = 486 / 8; break;
        case CodeRate::R5_6: info_bytes = 540 / 8; break;
        default: info_bytes = 40;
    }

    // Create random test data
    Bytes data(info_bytes);
    for (auto& b : data) b = rng() & 0xFF;

    // Encode and modulate
    Bytes encoded = encoder.encode(data);
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, mod);

    // Combine into signal
    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    // Normalize
    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    // Add noise (skip if SNR > 100 dB for "infinite" SNR test)
    if (snr_db < 100.0f) {
        float sp = 0;
        for (float s : signal) sp += s * s;
        sp /= signal.size();
        float noise_std = std::sqrt(sp / std::pow(10.0f, snr_db / 10.0f));
        std::normal_distribution<float> noise(0.0f, noise_std);
        for (float& s : signal) s += noise(rng);
    }

    // Demodulate in chunks (simulating streaming audio)
    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    // Get soft bits and decode
    auto soft = demod.getSoftBits();
    if (soft.size() < 648) {
        return false;  // Not enough bits
    }

    std::span<const float> llrs(soft.data(), 648);
    Bytes decoded = decoder.decodeSoft(llrs);

    if (!decoder.lastDecodeSuccess() || decoded.size() < data.size()) {
        return false;
    }

    // Verify data matches
    for (size_t i = 0; i < data.size(); i++) {
        if (decoded[i] != data[i]) return false;
    }

    return true;
}

// Calculate theoretical throughput
float theoreticalThroughput(const ModemConfig& cfg, Modulation mod, CodeRate rate) {
    uint32_t data_carriers = cfg.use_pilots ?
        cfg.num_carriers - (cfg.num_carriers / cfg.pilot_spacing) :
        cfg.num_carriers;
    float symbol_rate = cfg.getSymbolRate();
    return data_carriers * getBitsPerSymbol(mod) * getCodeRateValue(rate) * symbol_rate;
}

int main(int argc, char* argv[]) {
    // Parse arguments
    float snr_db = 25.0f;
    int trials = 20;

    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--snr" && i + 1 < argc) {
            snr_db = std::stof(argv[++i]);
        } else if (std::string(argv[i]) == "--trials" && i + 1 < argc) {
            trials = std::stoi(argv[++i]);
        }
    }

    setLogLevel(LogLevel::WARN);
    std::mt19937 rng(12345);

    std::cout << "=============================================================================\n";
    std::cout << "               NVIS High-Speed Mode Test\n";
    std::cout << "=============================================================================\n\n";

    // Get NVIS mode config
    ModemConfig nvis_cfg = presets::nvis_mode();

    std::cout << "NVIS Mode Parameters:\n";
    std::cout << "  FFT size:        " << nvis_cfg.fft_size << "\n";
    std::cout << "  Carriers:        " << nvis_cfg.num_carriers << "\n";
    std::cout << "  Cyclic prefix:   " << nvis_cfg.getCyclicPrefix() << " samples\n";
    std::cout << "  Symbol duration: " << nvis_cfg.getSymbolDuration() << " samples\n";
    std::cout << "  Symbol rate:     " << std::fixed << std::setprecision(1)
              << nvis_cfg.getSymbolRate() << " baud\n";
    std::cout << "  Test SNR:        " << snr_db << " dB\n";
    std::cout << "  Trials/mode:     " << trials << "\n\n";

    // Compare with standard 512 FFT config
    ModemConfig std_cfg = presets::balanced();
    std::cout << "Standard Mode Parameters (for reference):\n";
    std::cout << "  FFT size:        " << std_cfg.fft_size << "\n";
    std::cout << "  Carriers:        " << std_cfg.num_carriers << "\n";
    std::cout << "  Symbol rate:     " << std::fixed << std::setprecision(1)
              << std_cfg.getSymbolRate() << " baud\n\n";

    // Test modes
    struct TestMode {
        Modulation mod;
        CodeRate rate;
        bool use_pilots;
        const char* name;
    };

    std::vector<TestMode> modes = {
        // Differential modes (no pilots needed)
        {Modulation::DQPSK, CodeRate::R1_2, false, "DQPSK R1/2"},
        {Modulation::DQPSK, CodeRate::R3_4, false, "DQPSK R3/4"},
        {Modulation::D8PSK, CodeRate::R1_2, false, "D8PSK R1/2"},
        {Modulation::D8PSK, CodeRate::R3_4, false, "D8PSK R3/4"},
        // Coherent modes (need pilots)
        {Modulation::QAM16, CodeRate::R1_2, true, "16QAM R1/2"},
        {Modulation::QAM16, CodeRate::R3_4, true, "16QAM R3/4"},
        {Modulation::QAM32, CodeRate::R1_2, true, "32QAM R1/2"},
        {Modulation::QAM32, CodeRate::R3_4, true, "32QAM R3/4"},
    };

    std::cout << "=============================================================================\n";
    std::cout << std::setw(14) << "Mode"
              << std::setw(10) << "Carriers"
              << std::setw(12) << "Theory"
              << std::setw(10) << "Success"
              << std::setw(10) << "Target"
              << "\n";
    std::cout << std::setw(14) << ""
              << std::setw(10) << "(data)"
              << std::setw(12) << "(bps)"
              << std::setw(10) << "(%)"
              << std::setw(10) << "(bps)"
              << "\n";
    std::cout << "-----------------------------------------------------------------------------\n";

    // Industry leader targets
    float industry_targets[] = {0, 0, 0, 0,  // DQPSK (not published)
                                 7074, 7074, 8489, 8489};  // 16QAM, 32QAM

    int mode_idx = 0;
    for (auto& m : modes) {
        ModemConfig cfg = nvis_cfg;
        cfg.use_pilots = m.use_pilots;
        if (m.use_pilots) {
            cfg.pilot_spacing = 4;  // Sparser pilots for max throughput
        }

        // Calculate data carriers
        uint32_t data_carriers = cfg.use_pilots ?
            cfg.num_carriers - (cfg.num_carriers / cfg.pilot_spacing) :
            cfg.num_carriers;

        float theory = theoreticalThroughput(cfg, m.mod, m.rate);

        // Run trials
        int success = 0;
        for (int t = 0; t < trials; t++) {
            if (testSingle(cfg, m.mod, m.rate, snr_db, rng)) {
                success++;
            }
        }

        float success_pct = 100.0f * success / trials;
        float target = industry_targets[mode_idx];

        std::cout << std::setw(14) << m.name
                  << std::setw(10) << data_carriers
                  << std::setw(12) << std::fixed << std::setprecision(0) << theory
                  << std::setw(10) << std::setprecision(0) << success_pct
                  << std::setw(10) << (target > 0 ? std::to_string((int)target) : "-")
                  << "\n";

        mode_idx++;
    }

    std::cout << "=============================================================================\n\n";

    // Summary
    std::cout << "Industry Leader Targets:\n";
    std::cout << "  16QAM max: 7,074 bps\n";
    std::cout << "  32QAM max: 8,489 bps\n\n";

    std::cout << "Notes:\n";
    std::cout << "  - NVIS mode uses 1024 FFT / 59 carriers / ~42 baud\n";
    std::cout << "  - Differential modes (DQPSK/D8PSK): all 59 carriers as data\n";
    std::cout << "  - Coherent modes (16QAM/32QAM): ~44 data + ~15 pilot carriers\n";
    std::cout << "  - Higher code rates (R3/4, R5/6) need higher SNR\n";
    std::cout << "  - Try --snr 30 for 16QAM, --snr 35 for 32QAM\n";

    return 0;
}
