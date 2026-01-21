/**
 * Compare ALL waveforms on Poor and Flutter HF channels
 * DPSK vs OTFS - which is actually best?
 */

#include "psk/dpsk.hpp"
#include "ultra/otfs.hpp"
#include "ultra/fec.hpp"
#include "sim/hf_channel.hpp"
#include <iostream>
#include <iomanip>

using namespace ultra;
using namespace ultra::sim;

Bytes test_data = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                   0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF,
                   0xDE, 0xAD, 0xBE, 0xEF};

// Test DPSK
float testDPSK(const DPSKConfig& config, CodeRate rate, WattersonChannel::Config chCfg, int trials = 30) {
    LDPCEncoder encoder(rate);
    LDPCDecoder decoder(rate);
    int success = 0;

    for (int t = 0; t < trials; t++) {
        WattersonChannel channel(chCfg, 42 + t);
        DPSKModulator mod(config);
        DPSKDemodulator demod(config);

        Bytes encoded = encoder.encode(test_data);
        Samples tx = mod.modulate(encoded);

        float max_val = 0;
        for (float s : tx) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx) s *= 0.5f / max_val;

        Samples rx = channel.process(SampleSpan(tx.data(), tx.size()));

        auto soft = demod.demodulateSoft(SampleSpan(rx.data(), rx.size()));
        if (soft.size() < 648) continue;
        soft.resize(648);

        float sum = 0;
        for (float f : soft) sum += std::abs(f);
        float avg = sum / soft.size();
        if (avg > 0.01f) {
            float scale = 3.5f / avg;
            for (float& f : soft) f *= scale;
        }

        Bytes decoded = decoder.decodeSoft(soft);
        if (!decoder.lastDecodeSuccess()) continue;
        decoded.resize(test_data.size());
        if (decoded == test_data) success++;
    }
    return 100.0f * success / trials;
}

// Test OTFS
float testOTFS(uint32_t N, uint32_t cp, WattersonChannel::Config chCfg, int trials = 30) {
    LDPCEncoder encoder(CodeRate::R1_4);
    LDPCDecoder decoder(CodeRate::R1_4);
    int success = 0;

    for (int t = 0; t < trials; t++) {
        WattersonChannel channel(chCfg, 42 + t);

        OTFSConfig cfg;
        cfg.M = 32; cfg.N = N;
        cfg.fft_size = 512; cfg.cp_length = cp;
        cfg.sample_rate = 48000; cfg.center_freq = 1500.0f;
        cfg.modulation = Modulation::QPSK;
        cfg.tf_equalization = true;

        OTFSModulator mod(cfg);
        OTFSDemodulator demod(cfg);

        Bytes encoded = encoder.encode(test_data);
        auto dd = mod.mapToDD(ByteSpan(encoded.data(), encoded.size()), Modulation::QPSK);

        Samples preamble = mod.generatePreamble();
        Samples data = mod.modulate(dd, Modulation::QPSK);
        Samples tx;
        tx.insert(tx.end(), preamble.begin(), preamble.end());
        tx.insert(tx.end(), data.begin(), data.end());

        float max_val = 0;
        for (float s : tx) max_val = std::max(max_val, std::abs(s));
        if (max_val > 0) for (float& s : tx) s *= 0.5f / max_val;

        Samples rx = channel.process(SampleSpan(tx.data(), tx.size()));

        demod.reset();
        if (!demod.process(SampleSpan(rx.data(), rx.size()))) continue;

        auto soft = demod.getSoftBits();
        if (soft.size() < 648) continue;
        soft.resize(648);

        float sum = 0;
        for (float f : soft) sum += std::abs(f);
        float avg = sum / soft.size();
        if (avg > 0.01f) {
            float scale = 3.5f / avg;
            for (float& f : soft) f *= scale;
        }

        Bytes decoded = decoder.decodeSoft(soft);
        if (!decoder.lastDecodeSuccess()) continue;
        decoded.resize(test_data.size());
        if (decoded == test_data) success++;
    }
    return 100.0f * success / trials;
}

int main() {
    std::cout << "=== ALL WAVEFORMS: Poor & Flutter HF Channels ===\n\n";

    std::vector<float> snr_levels = {8, 10, 12, 15, 18};

    // Poor channel: 2ms delay, 1Hz Doppler
    std::cout << "=== POOR CHANNEL (2ms delay, 1Hz Doppler) ===\n\n";
    std::cout << std::setw(6) << "SNR"
              << std::setw(14) << "DPSK-300"
              << std::setw(14) << "DPSK-125"
              << std::setw(14) << "OTFS-N32"
              << std::setw(12) << "BEST\n";
    std::cout << std::string(60, '-') << "\n";

    for (float snr : snr_levels) {
        auto poor = itu_r_f1487::poor(snr);

        float dpsk300 = testDPSK(dpsk_presets::speed1(), CodeRate::R1_2, poor);
        float dpsk125 = testDPSK(dpsk_presets::fast(), CodeRate::R1_4, poor);
        float otfs = testOTFS(32, 128, poor);

        std::cout << std::setw(4) << (int)snr << "dB"
                  << std::setw(13) << std::fixed << std::setprecision(0) << dpsk300 << "%"
                  << std::setw(13) << dpsk125 << "%"
                  << std::setw(13) << otfs << "%";

        float best = std::max({dpsk300, dpsk125, otfs});
        if (dpsk300 == best) std::cout << std::setw(12) << "DPSK-300";
        else if (dpsk125 == best) std::cout << std::setw(12) << "DPSK-125";
        else std::cout << std::setw(12) << "OTFS";
        std::cout << "\n";
    }

    // Flutter channel: 0.5ms delay, 10Hz Doppler
    std::cout << "\n=== FLUTTER CHANNEL (0.5ms delay, 10Hz Doppler) ===\n\n";
    std::cout << std::setw(6) << "SNR"
              << std::setw(14) << "DPSK-300"
              << std::setw(14) << "DPSK-125"
              << std::setw(14) << "OTFS-N64"
              << std::setw(12) << "BEST\n";
    std::cout << std::string(60, '-') << "\n";

    for (float snr : snr_levels) {
        auto flutter = itu_r_f1487::flutter(snr);

        float dpsk300 = testDPSK(dpsk_presets::speed1(), CodeRate::R1_2, flutter);
        float dpsk125 = testDPSK(dpsk_presets::fast(), CodeRate::R1_4, flutter);
        float otfs = testOTFS(64, 64, flutter);

        std::cout << std::setw(4) << (int)snr << "dB"
                  << std::setw(13) << std::fixed << std::setprecision(0) << dpsk300 << "%"
                  << std::setw(13) << dpsk125 << "%"
                  << std::setw(13) << otfs << "%";

        float best = std::max({dpsk300, dpsk125, otfs});
        if (dpsk300 == best) std::cout << std::setw(12) << "DPSK-300";
        else if (dpsk125 == best) std::cout << std::setw(12) << "DPSK-125";
        else std::cout << std::setw(12) << "OTFS";
        std::cout << "\n";
    }

    std::cout << "\n=== THROUGHPUT COMPARISON ===\n";
    std::cout << "DPSK-300 (R1/2): 300 bps\n";
    std::cout << "DPSK-125 (R1/4): 62 bps\n";
    std::cout << "OTFS-N32 (R1/4): ~200 bps (32 carriers × 2 bits × 0.25 / 0.4s frame)\n";
    std::cout << "OTFS-N64 (R1/4): ~100 bps (longer frame = lower throughput)\n";

    return 0;
}
