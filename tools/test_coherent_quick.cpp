#include "ultra/types.hpp"
#include "ultra/ofdm.hpp"
#include "ultra/fec.hpp"
#include "ultra/logging.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <span>

using namespace ultra;

// Quick test - single trial
bool test_single(Modulation mod, const char* name, float snr_db, std::mt19937& rng) {
    ModemConfig config;
    config.sample_rate = 48000;
    config.center_freq = 1500;
    config.fft_size = 512;
    config.num_carriers = 30;
    config.pilot_spacing = 2;
    config.modulation = mod;
    config.code_rate = CodeRate::R1_2;
    bool is_diff = (mod == Modulation::DQPSK || mod == Modulation::D8PSK || mod == Modulation::DBPSK);
    config.use_pilots = !is_diff;

    OFDMModulator modulator(config);
    OFDMDemodulator demod(config);
    LDPCEncoder encoder(CodeRate::R1_2);
    LDPCDecoder decoder(CodeRate::R1_2);

    Bytes data(40);
    for (auto& b : data) b = rng() & 0xFF;

    Bytes encoded = encoder.encode(data);
    auto preamble = modulator.generatePreamble();
    auto modulated = modulator.modulate(encoded, mod);

    Samples signal;
    signal.insert(signal.end(), preamble.begin(), preamble.end());
    signal.insert(signal.end(), modulated.begin(), modulated.end());

    float max_val = 0;
    for (float s : signal) max_val = std::max(max_val, std::abs(s));
    for (float& s : signal) s *= 0.5f / max_val;

    float sp = 0;
    for (float s : signal) sp += s * s;
    sp /= signal.size();
    float noise_std = std::sqrt(sp / std::pow(10.0f, snr_db / 10.0f));
    std::normal_distribution<float> noise(0.0f, noise_std);
    for (float& s : signal) s += noise(rng);

    for (size_t i = 0; i < signal.size(); i += 960) {
        size_t len = std::min((size_t)960, signal.size() - i);
        SampleSpan span(signal.data() + i, len);
        demod.process(span);
    }

    auto soft = demod.getSoftBits();
    bool synced = soft.size() >= 648;
    bool success = false;

    if (synced) {
        std::span<const float> llrs(soft.data(), 648);
        Bytes decoded = decoder.decodeSoft(llrs);
        success = decoder.lastDecodeSuccess() && decoded.size() >= data.size();
        if (success) {
            for (size_t i = 0; i < data.size(); i++) {
                if (decoded[i] != data[i]) { success = false; break; }
            }
        }
    }

    return success;
}

int main() {
    setLogLevel(LogLevel::WARN);
    std::mt19937 rng(12345);

    printf("=== Coherent Mode Test (Multiple SNR, 20 trials each) ===\n\n");

    struct ModeTest {
        Modulation mod;
        const char* name;
    };

    ModeTest modes[] = {
        {Modulation::DQPSK, "DQPSK"},
        {Modulation::QPSK, "QPSK"},
        {Modulation::QAM16, "QAM16"},
    };

    const int trials = 20;

    printf("Mode      20 dB   25 dB   30 dB\n");
    printf("-------------------------------\n");

    for (auto& m : modes) {
        printf("%-8s", m.name);
        for (float snr : {20.0f, 25.0f, 30.0f}) {
            int success = 0;
            for (int t = 0; t < trials; t++) {
                if (test_single(m.mod, m.name, snr, rng)) success++;
            }
            printf(" %3d%%  ", success * 100 / trials);
        }
        printf("\n");
    }

    printf("\n=== Done ===\n");
    return 0;
}
