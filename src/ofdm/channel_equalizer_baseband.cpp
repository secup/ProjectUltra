// Channel Estimation and Equalization for OFDM
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <cmath>
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "ultra/logging.hpp"
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <string>

namespace ultra {

using namespace demod_constants;

namespace {

struct CFODumpConfig {
    bool initialized = false;
    bool enabled = false;
    std::string prefix;
    int max_dumps = 1;
    std::atomic<int> dumped{0};
};

CFODumpConfig& getCFODumpConfig() {
    static CFODumpConfig cfg;
    if (!cfg.initialized) {
        cfg.initialized = true;
        const char* prefix = std::getenv("ULTRA_DUMP_CFO_PREFIX");
        if (prefix && *prefix) {
            cfg.enabled = true;
            cfg.prefix = prefix;
            if (const char* n = std::getenv("ULTRA_DUMP_CFO_CALLS")) {
                int parsed = std::atoi(n);
                if (parsed > 0) cfg.max_dumps = parsed;
            }
        }
    }
    return cfg;
}

bool reserveCFODumpSlot(int& out_idx) {
    auto& cfg = getCFODumpConfig();
    if (!cfg.enabled) return false;

    int cur = cfg.dumped.load(std::memory_order_relaxed);
    while (cur < cfg.max_dumps) {
        if (cfg.dumped.compare_exchange_weak(cur, cur + 1, std::memory_order_relaxed)) {
            out_idx = cur;
            return true;
        }
    }
    return false;
}

bool writeComplexDump(const std::string& path, const std::vector<Complex>& data) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    out.write(reinterpret_cast<const char*>(data.data()),
              static_cast<std::streamsize>(data.size() * sizeof(Complex)));
    return static_cast<bool>(out);
}

void writeCFODumpMeta(const std::string& path,
                      const std::string& pre_path,
                      const std::string& post_path,
                      float cfo_hz,
                      float start_phase,
                      float end_phase,
                      float phase_increment,
                      size_t sample_count,
                      uint32_t sample_rate) {
    std::ofstream out(path);
    if (!out) return;
    out << "format=complex64_interleaved\n";
    out << "sample_rate=" << sample_rate << "\n";
    out << "samples=" << sample_count << "\n";
    out << "cfo_hz=" << cfo_hz << "\n";
    out << "phase_increment_rad_per_sample=" << phase_increment << "\n";
    out << "start_phase_rad=" << start_phase << "\n";
    out << "end_phase_rad=" << end_phase << "\n";
    out << "pre_file=" << pre_path << "\n";
    out << "post_file=" << post_path << "\n";
    out << "note=pre_file is after mixer/downconversion but before CFO correction; post_file is after CFO correction.\n";
}

}  // namespace

// =============================================================================
// BASEBAND CONVERSION
// =============================================================================

const std::vector<Complex>& OFDMDemodulator::Impl::toBaseband(SampleSpan samples) {
    auto& baseband = baseband_scratch;
    baseband.resize(samples.size());

    // Phase increment per sample for frequency correction
    float phase_increment = -2.0f * M_PI * freq_offset_hz / config.sample_rate;

    // Log CFO correction
    static std::atomic<int> tb_call_count{0};
    if (std::abs(freq_offset_hz) > 0.01f && tb_call_count < 5) {
        LOG_DEMOD(DEBUG, "toBaseband #%d: CFO=%.2f Hz, phase_inc=%.6f, start_phase=%.2f rad (%.1f deg), samples=%zu",
                  tb_call_count.load(), freq_offset_hz, phase_increment, freq_correction_phase,
                  freq_correction_phase * 180.0f / M_PI, samples.size());
        tb_call_count++;
    }

    const float start_phase = freq_correction_phase;
    int dump_idx = -1;
    const bool dump_this_call = (std::abs(freq_offset_hz) > 0.01f) && reserveCFODumpSlot(dump_idx);
    std::vector<Complex> pre_cfo;
    std::vector<Complex> post_cfo;
    if (dump_this_call) {
        pre_cfo.resize(samples.size());
        post_cfo.resize(samples.size());
    }

    for (size_t i = 0; i < samples.size(); ++i) {
        Complex osc = mixer.next();
        Complex mixed = samples[i] * std::conj(osc);
        if (dump_this_call) {
            pre_cfo[i] = mixed;
        }

        // Apply frequency offset correction
        if (std::abs(freq_offset_hz) > 0.01f) {
            Complex correction(std::cos(freq_correction_phase),
                               std::sin(freq_correction_phase));
            mixed *= correction;
            freq_correction_phase += phase_increment;

            // Wrap phase
            if (freq_correction_phase > M_PI) {
                freq_correction_phase -= 2.0f * M_PI;
            } else if (freq_correction_phase < -M_PI) {
                freq_correction_phase += 2.0f * M_PI;
            }
        }

        if (dump_this_call) {
            post_cfo[i] = mixed;
        }

        baseband[i] = mixed;
    }

    if (dump_this_call) {
        const auto& cfg = getCFODumpConfig();
        std::string base = cfg.prefix + "_" + std::to_string(dump_idx);
        std::string pre_path = base + "_pre.cf32";
        std::string post_path = base + "_post.cf32";
        std::string meta_path = base + "_meta.txt";
        bool ok_pre = writeComplexDump(pre_path, pre_cfo);
        bool ok_post = writeComplexDump(post_path, post_cfo);
        writeCFODumpMeta(meta_path, pre_path, post_path, freq_offset_hz,
                         start_phase, freq_correction_phase, phase_increment,
                         samples.size(), config.sample_rate);

        LOG_DEMOD(INFO, "CFO dump #%d: %s (%zu) and %s (%zu) %s",
                  dump_idx,
                  pre_path.c_str(), pre_cfo.size(),
                  post_path.c_str(), post_cfo.size(),
                  (ok_pre && ok_post) ? "[ok]" : "[write failed]");
    }

    return baseband;
}

const std::vector<Complex>& OFDMDemodulator::Impl::extractSymbol(const std::vector<Complex>& baseband, size_t offset) {
    size_t start = offset + config.getCyclicPrefix();

    auto& symbol = symbol_scratch;
    symbol.resize(config.fft_size);
    std::fill(symbol.begin(), symbol.end(), Complex(0, 0));
    for (size_t i = 0; i < config.fft_size && (start + i) < baseband.size(); ++i) {
        symbol[i] = baseband[start + i];
    }

    auto& freq = freq_domain_scratch;
    freq.resize(config.fft_size);
    fft.forward(symbol.data(), freq.data());

    return freq;
}

} // namespace ultra
