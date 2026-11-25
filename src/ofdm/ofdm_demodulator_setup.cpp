// OFDM demodulator setup and carrier tables
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <random>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "pilot_pattern.hpp"

namespace ultra {

using namespace demod_constants;

// =============================================================================
// IMPL CONSTRUCTOR AND INITIALIZATION
// =============================================================================

OFDMDemodulator::Impl::Impl(const ModemConfig& cfg)
    : config(cfg)
    , fft(cfg.fft_size)
    , mixer(cfg.center_freq, cfg.sample_rate)
    , sync_threshold(cfg.sync_threshold)
{
    // Initialize the LTS DFT-denoise gate + tap window from the environment so
    // DEFAULT behavior is identical to the prior static-env read. A test can
    // override these via OFDMDemodulator::setLtsDftDenoise().
    {
        const char* e = std::getenv("ULTRA_LTS_DFT_DENOISE");
        dft_denoise_enabled_ = (e && e[0] == '1');
        const char* taps = std::getenv("ULTRA_LTS_DFT_DENOISE_TAPS");
        dft_denoise_taps_ = taps ? std::atoi(taps) : 0;
        // Lever ① CFO-clean 2-LTS averaging gate (default off).
        const char* avg = std::getenv("ULTRA_LTS_CFO_AVG");
        lts_cfo_avg_enabled_ = (avg && avg[0] == '1');
    }

    symbol_samples = cfg.getSymbolDuration();
    baseband_scratch.resize(symbol_samples);
    symbol_scratch.resize(cfg.fft_size);
    freq_domain_scratch.resize(cfg.fft_size);
    equalized_scratch.resize(static_cast<size_t>(cfg.num_carriers));
    constellation_update_scratch.reserve(static_cast<size_t>(cfg.num_carriers));
    channel_estimate.resize(cfg.fft_size, Complex(1, 0));
    per_carrier_h_error_var_.assign(cfg.fft_size, 0.0f);  // Phase 2b eps_H term; 0 = no inflation
    per_carrier_raw_obs_power_.assign(cfg.fft_size, 0.0f);   // F166 Stage B
    per_carrier_raw_obs_symbol_.assign(cfg.fft_size, -1);    // -1 = never observed
    wiener_pilot_history_.resize(static_cast<size_t>(cfg.num_carriers));
    wiener_time_estimate_.resize(static_cast<size_t>(cfg.num_carriers), Complex(0, 0));
    wiener_time_error_var_.resize(static_cast<size_t>(cfg.num_carriers), 1.0f);
    wiener_time_valid_.resize(static_cast<size_t>(cfg.num_carriers), 0);

    // Initialize adaptive equalizer state
    lms_weights.resize(cfg.fft_size, Complex(1, 0));
    last_decisions.resize(cfg.fft_size, Complex(0, 0));
    rls_P.resize(cfg.fft_size, 1.0f);

    setupCarriers();
    generateSequences();
    buildInterpTable();
}

void OFDMDemodulator::Impl::setupCarriers() {
    activateCarrierPattern(0);
}

void OFDMDemodulator::Impl::activateCarrierPattern(size_t symbol_index) {
    ofdm_pilots::buildCarrierPattern(config,
                                     symbol_index,
                                     all_carrier_fft_indices,
                                     data_carrier_indices,
                                     pilot_carrier_indices,
                                     data_logical_carrier_indices,
                                     pilot_logical_carrier_indices,
                                     is_pilot_logical,
                                     pilot_sequence);
    buildInterpTable();
}

void OFDMDemodulator::Impl::generateSequences() {
    // Zadoff-Chu sequence for sync
    size_t N = config.num_carriers;
    size_t u = 1;

    sync_sequence.resize(N);
    for (size_t n = 0; n < N; ++n) {
        float phase = -M_PI * u * n * (n + 1) / N;
        sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
    }

    LOG_DEMOD(DEBUG, "Demod pilot config: %zu pilots, %zu data carriers",
              pilot_carrier_indices.size(), data_carrier_indices.size());
    if (pilot_carrier_indices.size() >= 3) {
        LOG_DEMOD(DEBUG, "Demod pilot indices[0-2]: %d, %d, %d",
                  pilot_carrier_indices[0], pilot_carrier_indices[1], pilot_carrier_indices[2]);
    }
    if (pilot_sequence.size() >= 3) {
        LOG_DEMOD(DEBUG, "Demod pilot seq[0-2]: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)",
                  pilot_sequence[0].real(), pilot_sequence[0].imag(),
                  pilot_sequence[1].real(), pilot_sequence[1].imag(),
                  pilot_sequence[2].real(), pilot_sequence[2].imag());
    }

    // Generate LTS time-domain reference for fine timing
    std::vector<Complex> lts_freq(config.fft_size, Complex(0, 0));

    for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
        lts_freq[data_carrier_indices[i]] = sync_sequence[i % sync_sequence.size()];
    }
    for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
        lts_freq[pilot_carrier_indices[i]] = pilot_sequence[i];
    }

    std::vector<Complex> lts_complex;
    fft.inverse(lts_freq, lts_complex);

    size_t cp_len = config.getCyclicPrefix();
    std::vector<Complex> lts_baseband(cp_len + config.fft_size);
    for (size_t i = 0; i < cp_len; ++i) {
        lts_baseband[i] = lts_complex[config.fft_size - cp_len + i];
    }
    for (size_t i = 0; i < config.fft_size; ++i) {
        lts_baseband[cp_len + i] = lts_complex[i];
    }

    // Convert to passband templates
    lts_passband_I.resize(lts_baseband.size());
    lts_passband_Q.resize(lts_baseband.size());

    NCO template_nco(config.center_freq, config.sample_rate);
    for (size_t i = 0; i < lts_baseband.size(); ++i) {
        Complex osc = template_nco.next();
        Complex mixed = lts_baseband[i] * osc;
        lts_passband_I[i] = mixed.real();
        lts_passband_Q[i] = mixed.imag();
    }

    LOG_DEMOD(DEBUG, "LTS passband templates generated: %zu samples", lts_passband_I.size());
}

void OFDMDemodulator::Impl::buildInterpTable() {
    // Pre-compute interpolation weights for each data carrier
    interp_table.clear();
    interp_table.reserve(data_carrier_indices.size());

    for (size_t ci = 0; ci < all_carrier_fft_indices.size(); ++ci) {
        if (ci < is_pilot_logical.size() && is_pilot_logical[ci]) continue;

        InterpInfo info;
        info.fft_idx = all_carrier_fft_indices[ci];
        info.lower_pilot = -1;
        info.upper_pilot = -1;
        info.alpha = 0.5f;

        int lower_ci = -1;
        for (int j = (int)ci - 1; j >= 0; --j) {
            if (j < static_cast<int>(is_pilot_logical.size()) &&
                is_pilot_logical[j]) {
                info.lower_pilot = all_carrier_fft_indices[j];
                lower_ci = j;
                break;
            }
        }

        int upper_ci = -1;
        for (size_t j = ci + 1; j < all_carrier_fft_indices.size(); ++j) {
            if (j < is_pilot_logical.size() && is_pilot_logical[j]) {
                info.upper_pilot = all_carrier_fft_indices[j];
                upper_ci = (int)j;
                break;
            }
        }

        if (lower_ci >= 0 && upper_ci >= 0) {
            float total_dist = (float)(upper_ci - lower_ci);
            info.alpha = (total_dist > 0) ? (float)((int)ci - lower_ci) / total_dist : 0.5f;
        }

        interp_table.push_back(info);
    }
}


} // namespace ultra
