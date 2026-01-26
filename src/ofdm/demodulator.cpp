// OFDM Demodulator - Main state machine
// Sync detection: ofdm_sync.cpp
// Channel estimation/equalization: channel_equalizer.cpp
// Soft demapping: soft_demap.hpp

#define _USE_MATH_DEFINES
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "soft_demap.hpp"
#include <algorithm>
#include <numeric>
#include <random>

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
    symbol_samples = cfg.getSymbolDuration();
    channel_estimate.resize(cfg.fft_size, Complex(1, 0));

    // Initialize adaptive equalizer state
    lms_weights.resize(cfg.fft_size, Complex(1, 0));
    last_decisions.resize(cfg.fft_size, Complex(0, 0));
    rls_P.resize(cfg.fft_size, 1.0f);

    setupCarriers();
    generateSequences();
    buildInterpTable();
}

void OFDMDemodulator::Impl::setupCarriers() {
    // Must match modulator exactly
    int neg_limit = config.num_carriers / 2;
    int pos_limit = (config.num_carriers + 1) / 2;

    int pilot_count = 0;
    for (int i = -neg_limit; i <= pos_limit; ++i) {
        if (i == 0) continue;

        int fft_idx = (i + config.fft_size) % config.fft_size;

        if (!config.use_pilots) {
            data_carrier_indices.push_back(fft_idx);
        } else {
            if (pilot_count % config.pilot_spacing == 0) {
                pilot_carrier_indices.push_back(fft_idx);
            } else {
                data_carrier_indices.push_back(fft_idx);
            }
        }
        ++pilot_count;
    }
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

    // Pilot sequence (must match modulator)
    pilot_sequence.resize(pilot_carrier_indices.size());
    std::mt19937 rng(PILOT_RNG_SEED);
    for (size_t i = 0; i < pilot_sequence.size(); ++i) {
        pilot_sequence[i] = (rng() & 1) ? Complex(1, 0) : Complex(-1, 0);
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
    struct CarrierInfo {
        int fft_idx;
        bool is_pilot;
    };
    std::vector<CarrierInfo> carriers;
    int neg_limit = config.num_carriers / 2;
    int pos_limit = (config.num_carriers + 1) / 2;
    int pilot_count = 0;

    for (int i = -neg_limit; i <= pos_limit; ++i) {
        if (i == 0) continue;
        int fft_idx = (i + config.fft_size) % config.fft_size;
        bool is_pilot = (pilot_count % config.pilot_spacing == 0);
        carriers.push_back({fft_idx, is_pilot});
        ++pilot_count;
    }

    interp_table.clear();
    interp_table.reserve(data_carrier_indices.size());

    for (size_t ci = 0; ci < carriers.size(); ++ci) {
        if (carriers[ci].is_pilot) continue;

        InterpInfo info;
        info.fft_idx = carriers[ci].fft_idx;
        info.lower_pilot = -1;
        info.upper_pilot = -1;
        info.alpha = 0.5f;

        int lower_ci = -1;
        for (int j = (int)ci - 1; j >= 0; --j) {
            if (carriers[j].is_pilot) {
                info.lower_pilot = carriers[j].fft_idx;
                lower_ci = j;
                break;
            }
        }

        int upper_ci = -1;
        for (size_t j = ci + 1; j < carriers.size(); ++j) {
            if (carriers[j].is_pilot) {
                info.upper_pilot = carriers[j].fft_idx;
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

// =============================================================================
// SYMBOL DEMODULATION
// =============================================================================

void OFDMDemodulator::Impl::demodulateSymbol(const std::vector<Complex>& equalized, Modulation mod) {
    // Save symbols for constellation display
    {
        std::lock_guard<std::mutex> lock(constellation_mutex);
        constellation_symbols.insert(constellation_symbols.end(), equalized.begin(), equalized.end());
        if (constellation_symbols.size() > MAX_CONSTELLATION_SYMBOLS) {
            constellation_symbols.erase(constellation_symbols.begin(),
                                       constellation_symbols.begin() + (constellation_symbols.size() - MAX_CONSTELLATION_SYMBOLS));
        }
    }

    // Phase inversion detection (disabled - raw data can have extreme bias)
    if (soft_bits.empty() && !equalized.empty()) {
        int pos_count = 0, neg_count = 0;
        for (const auto& s : equalized) {
            if (s.real() > 0) pos_count++;
            else neg_count++;
        }
        LOG_DEMOD(DEBUG, "First symbol stats: %zu carriers, %d positive, %d negative, first 3: (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)",
                equalized.size(), pos_count, neg_count,
                equalized[0].real(), equalized[0].imag(),
                equalized.size() > 1 ? equalized[1].real() : 0.0f,
                equalized.size() > 1 ? equalized[1].imag() : 0.0f,
                equalized.size() > 2 ? equalized[2].real() : 0.0f,
                equalized.size() > 2 ? equalized[2].imag() : 0.0f);

        llr_sign_flip = false;
        float neg_ratio = float(neg_count) / equalized.size();
        LOG_DEMOD(DEBUG, "Symbol polarity: %.0f%% negative, %.0f%% positive (no flip)",
                  neg_ratio * 100, (1.0f - neg_ratio) * 100);
    }

    // Get channel estimation error margin
    float ce_error_margin = soft_demap::getCEErrorMargin(mod);
    float llr_sign = llr_sign_flip ? -1.0f : 1.0f;

    // Initialize DQPSK/D8PSK reference
    if ((mod == Modulation::DQPSK || mod == Modulation::D8PSK) && dbpsk_prev_equalized.empty()) {
        dbpsk_prev_equalized.assign(equalized.size(), Complex(1, 0));
        LOG_DEMOD(DEBUG, "DQPSK: Initialized reference with (1,0)");
    }

    for (size_t i = 0; i < equalized.size(); ++i) {
        const auto& sym = equalized[i];
        float base_nv = (i < carrier_noise_var.size()) ? carrier_noise_var[i] : noise_variance;
        float nv = base_nv * ce_error_margin;

        switch (mod) {
            case Modulation::DBPSK: {
                if (dbpsk_prev_equalized.empty()) {
                    dbpsk_prev_equalized.assign(equalized.size(), Complex(1, 0));
                }
                Complex prev_sym = dbpsk_prev_equalized[i];
                float llr = soft_demap::demapDBPSK(sym, prev_sym, nv);
                soft_bits.push_back(llr);
                dbpsk_prev_equalized[i] = sym;
                break;
            }
            case Modulation::DQPSK: {
                Complex prev_sym = dbpsk_prev_equalized[i];
                auto llrs = soft_demap::demapDQPSK(sym, prev_sym, nv);
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());

                if (snr_symbol_count < 5 && i < 3) {
                    Complex diff = sym * std::conj(prev_sym);
                    float diff_phase = std::atan2(diff.imag(), diff.real()) * 180.0f / M_PI;
                    LOG_DEMOD(DEBUG, "DQPSK sym=%d car=%zu: phase=%.1f° LLRs=[%.2f,%.2f]",
                             snr_symbol_count, i, diff_phase, llrs[0], llrs[1]);
                }

                dbpsk_prev_equalized[i] = sym;
                break;
            }
            case Modulation::D8PSK: {
                Complex prev_sym = dbpsk_prev_equalized[i];
                auto llrs = soft_demap::demapD8PSK(sym, prev_sym, nv);
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                dbpsk_prev_equalized[i] = sym;
                break;
            }
            case Modulation::BPSK:
                soft_bits.push_back(soft_demap::demapBPSK(sym, nv) * llr_sign);
                break;
            case Modulation::QPSK: {
                auto llrs = soft_demap::demapQPSK(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                break;
            }
            case Modulation::QAM16: {
                auto llrs = soft_demap::demapQAM16(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                break;
            }
            case Modulation::QAM32: {
                auto llrs = soft_demap::demapQAM32(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                break;
            }
            case Modulation::QAM64: {
                auto llrs = soft_demap::demapQAM64(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                break;
            }
            case Modulation::QAM256: {
                auto llrs = soft_demap::demapQAM256(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                break;
            }
            default: {
                auto llrs = soft_demap::demapQPSK(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
            }
        }
    }

    // Decision-directed tracking for differential modes without pilots
    // Two-stage tracking:
    // 1. Per-carrier channel tracking: update channel_estimate[] for frequency-selective fading
    // 2. Common phase tracking: update pilot_phase_correction for overall drift
    if ((mod == Modulation::DQPSK || mod == Modulation::D8PSK) && !dbpsk_prev_equalized.empty()) {
        // Skip first symbol to let differential decoding establish reference
        if (snr_symbol_count >= 1) {
            Complex phase_error_sum(0, 0);
            int valid_count = 0;

            // Per-carrier tracking: update channel_estimate based on decoded symbol
            // This handles frequency-selective fading where each carrier drifts differently
            float dd_alpha = (snr_symbol_count < 3) ? 0.3f : 0.15f;  // Faster initial, then slower

            for (size_t i = 0; i < equalized.size(); ++i) {
                int idx = data_carrier_indices[i];
                Complex prev_sym = (i < dbpsk_prev_equalized.size()) ? dbpsk_prev_equalized[i] : Complex(1, 0);
                float signal_power = std::abs(equalized[i]) * std::abs(prev_sym);

                // Only track strong carriers
                if (signal_power > 0.1f) {
                    Complex diff = equalized[i] * std::conj(prev_sym);
                    float phase = std::atan2(diff.imag(), diff.real());

                    // Map to nearest constellation point
                    float expected_phase;
                    if (mod == Modulation::DQPSK) {
                        int quadrant = (int)std::round(phase * 2.0f / M_PI);
                        quadrant = ((quadrant % 4) + 4) % 4;
                        expected_phase = quadrant * M_PI / 2.0f;
                    } else {
                        int octant = (int)std::round(phase * 4.0f / M_PI);
                        octant = ((octant % 8) + 8) % 8;
                        expected_phase = octant * M_PI / 4.0f;
                    }

                    // Phase error for this carrier
                    float phase_error = phase - expected_phase;
                    while (phase_error > M_PI) phase_error -= 2 * M_PI;
                    while (phase_error < -M_PI) phase_error += 2 * M_PI;

                    // Per-carrier channel update: rotate channel_estimate to correct the error
                    // Only update if error is small (likely correct decoding)
                    float max_error_rad = (mod == Modulation::DQPSK) ? 0.7f : 0.35f;  // ~40° for DQPSK
                    if (std::abs(phase_error) < max_error_rad) {
                        Complex phase_correction = Complex(std::cos(-phase_error * dd_alpha),
                                                           std::sin(-phase_error * dd_alpha));
                        channel_estimate[idx] *= phase_correction;
                    }

                    // Accumulate for common phase tracking
                    phase_error_sum += signal_power * Complex(std::cos(phase_error), std::sin(phase_error));
                    valid_count++;
                }
            }

            // Common phase tracking: update pilot_phase_correction
            if (valid_count >= 5) {
                float avg_phase_error = std::atan2(phase_error_sum.imag(), phase_error_sum.real());
                Complex correction = Complex(std::cos(-avg_phase_error), std::sin(-avg_phase_error));

                float alpha = (snr_symbol_count < 5) ? 0.5f : 0.2f;
                pilot_phase_correction = pilot_phase_correction *
                    std::pow(std::abs(correction), alpha) *
                    Complex(std::cos(alpha * std::arg(correction)),
                            std::sin(alpha * std::arg(correction)));

                float mag = std::abs(pilot_phase_correction);
                if (mag > 0.01f) pilot_phase_correction /= mag;

                if (snr_symbol_count < 10) {
                    LOG_DEMOD(DEBUG, "DD tracking: avg_err=%.1f°, valid=%d",
                              avg_phase_error * 180.0f / M_PI, valid_count);
                }
            }
        }
    }
}

void OFDMDemodulator::Impl::updateQuality() {
    quality.snr_db = 10.0f * std::log10(estimated_snr_linear);
    quality.doppler_hz = 0;
    quality.delay_spread_ms = 0;

    if (quality.snr_db > 15) {
        quality.ber_estimate = 1e-6f;
    } else if (quality.snr_db > 10) {
        quality.ber_estimate = 1e-5f;
    } else if (quality.snr_db > 5) {
        quality.ber_estimate = 1e-3f;
    } else {
        quality.ber_estimate = 1e-1f;
    }
}

// =============================================================================
// PUBLIC INTERFACE
// =============================================================================

OFDMDemodulator::OFDMDemodulator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OFDMDemodulator::~OFDMDemodulator() = default;

bool OFDMDemodulator::process(SampleSpan samples) {
    // Add to buffer
    impl_->rx_buffer.insert(impl_->rx_buffer.end(), samples.begin(), samples.end());

    // Preamble size constants
    size_t preamble_symbol_len = impl_->config.fft_size + impl_->config.getCyclicPrefix();
    size_t preamble_total_len = preamble_symbol_len * 6;
    size_t correlation_window = preamble_symbol_len * 2;

    // ========================================================================
    // SEARCHING STATE
    // ========================================================================
    if (impl_->state.load() == Impl::State::SEARCHING) {
        if (impl_->rx_buffer.size() < MIN_SEARCH_SAMPLES) {
            return false;
        }

        // Buffer overflow protection
        if (impl_->rx_buffer.size() > MAX_BUFFER_SAMPLES) {
            size_t keep = OVERLAP_SAMPLES;
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.end() - keep);
            LOG_SYNC(WARN, "Buffer overflow, trimmed to %zu samples", keep);
        }

        // Search for preamble
        bool found_sync = false;
        size_t sync_offset = 0;
        float sync_corr = 0;

        size_t search_end = (impl_->rx_buffer.size() > preamble_total_len + correlation_window)
                          ? impl_->rx_buffer.size() - preamble_total_len - correlation_window
                          : 0;

        for (size_t i = 0; i < search_end; i += SEARCH_STEP_SIZE) {
            if (!impl_->hasMinimumEnergy(i, correlation_window)) {
                i += correlation_window / 2 - SEARCH_STEP_SIZE;
                continue;
            }

            float corr = impl_->measureCorrelation(i);

            if (corr > impl_->sync_threshold) {
                // Search for plateau
                size_t plateau_count = 0;
                float peak_corr = corr;
                size_t peak_pos = i;

                for (size_t j = 0; j <= PLATEAU_SEARCH_WINDOW && i + j + preamble_total_len < impl_->rx_buffer.size(); j += 8) {
                    float ref_corr = impl_->measureCorrelation(i + j);
                    if (ref_corr >= PLATEAU_THRESHOLD) {
                        plateau_count++;
                    }
                    if (ref_corr > peak_corr) {
                        peak_corr = ref_corr;
                        peak_pos = i + j;
                    }
                }

                if (plateau_count >= MIN_PLATEAU_SAMPLES) {
                    found_sync = true;
                    sync_offset = peak_pos;
                    sync_corr = peak_corr;

                    LOG_SYNC(INFO, "Preamble: coarse=%zu, peak=%zu, sync=%zu, corr=%.3f/%.3f",
                             i, peak_pos, sync_offset, corr, peak_corr);
                    break;
                }
            }
        }

        if (found_sync) {
            float coarse_cfo = impl_->estimateCoarseCFO(sync_offset);
            impl_->freq_offset_hz = coarse_cfo;
            impl_->freq_offset_filtered = coarse_cfo;
            impl_->freq_correction_phase = 0.0f;
            impl_->symbols_since_sync = 0;

            LOG_SYNC(INFO, "SYNC: offset=%zu, corr=%.3f, CFO=%.1f Hz, buffer=%zu",
                     sync_offset, sync_corr, coarse_cfo, impl_->rx_buffer.size());

            impl_->last_sync_offset = sync_offset;

            // LTS fine timing
            size_t sts_start = sync_offset;
            size_t refined_lts_start = impl_->refineLTSTiming(sts_start);

            if (refined_lts_start == SIZE_MAX) {
                LOG_SYNC(INFO, "LTS confirmation FAILED - Schmidl-Cox false positive, continuing search");
                if (impl_->rx_buffer.size() > OVERLAP_SAMPLES * 2) {
                    size_t trim = std::min(sync_offset + preamble_symbol_len,
                                           impl_->rx_buffer.size() - OVERLAP_SAMPLES);
                    impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                           impl_->rx_buffer.begin() + trim);
                }
                found_sync = false;
            } else {
                size_t coarse_lts_pos = sts_start + 4 * preamble_symbol_len;
                int timing_refinement = (int)refined_lts_start - (int)coarse_lts_pos;
                LOG_SYNC(INFO, "LTS fine timing: coarse_lts=%zu, refined=%zu, delta=%+d samples",
                         coarse_lts_pos, refined_lts_start, timing_refinement);

                // Check if differential mode for LTS phase extraction
                bool is_differential = (impl_->config.modulation == Modulation::DQPSK ||
                                        impl_->config.modulation == Modulation::D8PSK ||
                                        impl_->config.modulation == Modulation::DBPSK);
                LOG_SYNC(DEBUG, "LTS phase check: is_differential=%d, use_pilots=%d",
                        is_differential, impl_->config.use_pilots);

                // Consume preamble
                size_t consume = refined_lts_start + 2 * preamble_symbol_len + impl_->manual_timing_offset;
                LOG_SYNC(DEBUG, "Consume calc: refined_lts=%zu + 2*preamble_sym=%zu + offset=%d = %zu",
                        refined_lts_start, 2 * preamble_symbol_len, impl_->manual_timing_offset, consume);
                impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                       impl_->rx_buffer.begin() + consume);

                // Transition to SYNCED
                impl_->state.store(Impl::State::SYNCED);
                impl_->synced_symbol_count.store(0);
                impl_->mixer.reset();
                impl_->dbpsk_prev_equalized.clear();

                if (!is_differential || impl_->config.use_pilots) {
                    impl_->carrier_phase_initialized = false;
                    impl_->carrier_phase_correction = Complex(1, 0);
                }
                impl_->dqpsk_skip_first_symbol = false;
                impl_->timing_offset_samples = 0.0f;
                LOG_SYNC(INFO, "Coherent mode: timing_offset=0 (LTS gives exact timing)");
            }
        } else {
            // No preamble found - trim old data
            if (impl_->rx_buffer.size() > OVERLAP_SAMPLES * 2) {
                size_t trim = impl_->rx_buffer.size() - OVERLAP_SAMPLES;
                impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                       impl_->rx_buffer.begin() + trim);
            }
        }
    }

    // ========================================================================
    // SYNCED STATE
    // ========================================================================
    if (impl_->state.load() == Impl::State::SYNCED) {
        // Check for new preamble (mid-frame detection)
        bool should_check_preamble = impl_->synced_symbol_count.load() > 0 &&
                                      impl_->idle_call_count.load() >= 2;

        if (should_check_preamble && impl_->rx_buffer.size() >= preamble_total_len) {
            size_t search_limit = std::min(impl_->rx_buffer.size() - preamble_total_len,
                                           impl_->symbol_samples * 2);
            constexpr size_t STEP = 8;

            for (size_t offset = 0; offset <= search_limit; offset += STEP) {
                float corr = impl_->measureCorrelation(offset);
                if (corr > impl_->sync_threshold) {
                    size_t sts_start = offset;
                    size_t refined_lts_start = impl_->refineLTSTiming(sts_start);

                    if (refined_lts_start == SIZE_MAX) {
                        LOG_SYNC(DEBUG, "SYNCED: LTS confirmation failed at offset %zu, continuing", offset);
                        continue;
                    }

                    size_t consume = refined_lts_start + 2 * preamble_symbol_len;

                    LOG_SYNC(INFO, "SYNCED preamble: sts=%zu, refined_lts=%zu, consume=%zu",
                             sts_start, refined_lts_start, consume);

                    float coarse_cfo = impl_->estimateCoarseCFO(sts_start);
                    impl_->freq_offset_hz = coarse_cfo;
                    impl_->freq_offset_filtered = coarse_cfo;
                    impl_->freq_correction_phase = 0.0f;
                    impl_->symbols_since_sync = 0;

                    impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                           impl_->rx_buffer.begin() + consume);

                    LOG_SYNC(WARN, "Mid-frame preamble detected at offset %zu (had %zu soft bits)! Clearing state.",
                             sts_start, impl_->soft_bits.size());
                    impl_->soft_bits.clear();
                    impl_->synced_symbol_count.store(0);
                    impl_->idle_call_count.store(0);
                    impl_->mixer.reset();

                    std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
                    impl_->snr_symbol_count = 0;
                    impl_->estimated_snr_linear = 1.0f;
                    impl_->noise_variance = 0.1f;
                    impl_->prev_pilot_phases.clear();

                    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
                    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
                    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);

                    impl_->dbpsk_prev_equalized.clear();
                    impl_->carrier_phase_initialized = false;
                    impl_->carrier_phase_correction = Complex(1, 0);

                    break;
                }
            }
        }

        size_t soft_bits_before = impl_->soft_bits.size();
        LOG_DEMOD(DEBUG, "SYNCED: buffer=%zu samples, symbol_samples=%zu, can process %zu symbols",
                  impl_->rx_buffer.size(), impl_->symbol_samples, impl_->rx_buffer.size() / impl_->symbol_samples);

        // Process all complete symbols
        size_t symbols_processed = 0;
        while (impl_->rx_buffer.size() >= impl_->symbol_samples) {
            SampleSpan sym_samples(impl_->rx_buffer.data(), impl_->symbol_samples);
            auto baseband = impl_->toBaseband(sym_samples);
            auto freq_domain = impl_->extractSymbol(baseband, 0);

            impl_->updateChannelEstimate(freq_domain);

            auto equalized = impl_->equalize(freq_domain);
            impl_->demodulateSymbol(equalized, impl_->config.modulation);

            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + impl_->symbol_samples);
            ++symbols_processed;

            impl_->updateQuality();

            int sym_count = ++impl_->synced_symbol_count;
            if (sym_count > MAX_SYMBOLS_BEFORE_TIMEOUT) {
                LOG_SYNC(WARN, "Sync timeout after %d symbols (%zu soft bits accumulated), resetting to SEARCHING",
                         sym_count, impl_->soft_bits.size());
                impl_->state.store(Impl::State::SEARCHING);
                impl_->synced_symbol_count.store(0);
                impl_->idle_call_count.store(0);
                return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
            }
        }

        if (symbols_processed > 0) {
            float snr_db = (impl_->estimated_snr_linear > 0)
                ? 10.0f * std::log10(impl_->estimated_snr_linear) : 0.0f;
            LOG_DEMOD(INFO, "Processed %zu symbols, soft_bits=%zu (+%zu), SNR=%.1f dB, CFO=%.1f Hz",
                      symbols_processed, impl_->soft_bits.size(),
                      impl_->soft_bits.size() - soft_bits_before,
                      snr_db, impl_->freq_offset_hz);
        }

        // Track idle calls
        if (impl_->soft_bits.size() == soft_bits_before) {
            int idle = ++impl_->idle_call_count;
            if (idle > MAX_IDLE_CALLS_BEFORE_RESET) {
                LOG_SYNC(DEBUG, "Idle timeout after %d calls (%zu soft bits), resetting to SEARCHING",
                         idle, impl_->soft_bits.size());
                impl_->state.store(Impl::State::SEARCHING);
                impl_->synced_symbol_count.store(0);
                impl_->idle_call_count.store(0);
                return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
            }
        } else {
            impl_->idle_call_count.store(0);
        }

        bool has_codeword = impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;

        // Frame completion detection
        bool truly_idle = samples.empty() && symbols_processed == 0;
        if (!has_codeword && impl_->synced_symbol_count.load() > 0 && truly_idle) {
            LOG_DEMOD(INFO, "Frame complete (only %zu leftover bits, truly idle), resetting to SEARCHING",
                      impl_->soft_bits.size());
            impl_->state.store(Impl::State::SEARCHING);
            impl_->synced_symbol_count.store(0);
            impl_->idle_call_count.store(0);
            impl_->soft_bits.clear();
        }

        LOG_DEMOD(DEBUG, "process: soft_bits=%zu, returning %s",
                  impl_->soft_bits.size(), has_codeword ? "true" : "false");
        return has_codeword;
    }

    LOG_DEMOD(DEBUG, "process: not synced, returning false");
    return false;
}

Bytes OFDMDemodulator::getData() {
    Bytes data;
    uint8_t byte = 0;
    int bit_count = 0;

    for (float llr : impl_->soft_bits) {
        uint8_t bit = (llr > 0) ? 1 : 0;
        byte = (byte << 1) | bit;
        ++bit_count;

        if (bit_count == 8) {
            data.push_back(byte);
            byte = 0;
            bit_count = 0;
        }
    }

    impl_->soft_bits.clear();
    return data;
}

std::vector<float> OFDMDemodulator::getSoftBits() {
    LOG_DEMOD(DEBUG, "getSoftBits: buffer has %zu soft bits", impl_->soft_bits.size());

    if (g_log_level >= LogLevel::TRACE && g_log_categories.demod && impl_->soft_bits.size() >= 24) {
        char buf[256];
        int pos = 0;
        for (size_t i = 0; i < 24 && pos < 240; ++i) {
            int bit = (impl_->soft_bits[i] < 0) ? 1 : 0;
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%+.1f(%d) ", impl_->soft_bits[i], bit);
            if ((i + 1) % 6 == 0) pos += snprintf(buf + pos, sizeof(buf) - pos, "| ");
        }
        LOG_DEMOD(TRACE, "First 24 LLRs: %s", buf);
    }

    if (impl_->soft_bits.size() <= LDPC_BLOCK_SIZE) {
        auto bits = std::move(impl_->soft_bits);
        impl_->soft_bits.clear();
        return bits;
    } else {
        std::vector<float> bits(impl_->soft_bits.begin(),
                                impl_->soft_bits.begin() + LDPC_BLOCK_SIZE);
        impl_->soft_bits.erase(impl_->soft_bits.begin(),
                               impl_->soft_bits.begin() + LDPC_BLOCK_SIZE);
        return bits;
    }
}

ChannelQuality OFDMDemodulator::getChannelQuality() const {
    return impl_->quality;
}

float OFDMDemodulator::getEstimatedSNR() const {
    return 10.0f * std::log10(impl_->estimated_snr_linear);
}

float OFDMDemodulator::getFrequencyOffset() const {
    return impl_->freq_offset_hz;
}

void OFDMDemodulator::setFrequencyOffset(float cfo_hz) {
    LOG_DEMOD(INFO, "setFrequencyOffset: CFO=%.2f Hz (was %.2f Hz)", cfo_hz, impl_->freq_offset_hz);
    impl_->freq_offset_hz = cfo_hz;
    impl_->freq_offset_filtered = cfo_hz;
    // Reset correction phase so it starts from 0 with the new offset
    impl_->freq_correction_phase = 0.0f;
    // Mark that CFO was explicitly provided (e.g., from chirp detection)
    // This tells processPresynced() to trust this value instead of re-estimating
    impl_->chirp_cfo_estimated = true;
}

Symbol OFDMDemodulator::getConstellationSymbols() const {
    std::lock_guard<std::mutex> lock(impl_->constellation_mutex);
    return impl_->constellation_symbols;
}

bool OFDMDemodulator::isSynced() const {
    return impl_->state.load() == Impl::State::SYNCED;
}

bool OFDMDemodulator::hasPendingData() const {
    if (impl_->state.load() != Impl::State::SYNCED) {
        return false;
    }
    if (!impl_->soft_bits.empty()) {
        return true;
    }
    return impl_->rx_buffer.size() >= impl_->symbol_samples;
}

size_t OFDMDemodulator::getLastSyncOffset() const {
    return impl_->last_sync_offset;
}

void OFDMDemodulator::setTimingOffset(int offset) {
    impl_->manual_timing_offset = offset;
}

bool OFDMDemodulator::processPresynced(SampleSpan samples, int training_symbols) {
    // Process samples after external sync (chirp preamble)
    //
    // ROBUST ACQUISITION SEQUENCE:
    // 1. First 'training_symbols' are known LTS sequence - use for channel estimation
    // 2. Remaining symbols are data - demodulate them
    //
    // This mirrors the Schmidl-Cox approach: LTS for channel est, then data.
    // The chirp replaces STS for more robust timing sync at low SNR.

    if (samples.size() < impl_->symbol_samples) {
        return false;
    }

    // Reset state but preserve config
    impl_->soft_bits.clear();
    impl_->demod_data.clear();
    impl_->rx_buffer.clear();
    impl_->synced_symbol_count.store(0);
    impl_->idle_call_count.store(0);
    impl_->mixer.reset();

    // Reset channel estimate to unity
    std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
    impl_->snr_symbol_count = 0;
    impl_->estimated_snr_linear = 1.0f;
    impl_->noise_variance = 0.1f;

    // Preserve pre-set CFO (e.g., from chirp-based estimation)
    // Only reset the correction phase so it starts fresh
    // impl_->freq_offset_hz = 0.0f;  // KEEP the pre-set value!
    // impl_->freq_offset_filtered = 0.0f;  // KEEP the pre-set value!
    impl_->freq_correction_phase = 0.0f;
    LOG_SYNC(INFO, "processPresynced: pre-set CFO=%.2f Hz, phase reset to 0", impl_->freq_offset_hz);
    impl_->symbols_since_sync = 0;
    impl_->prev_pilot_phases.clear();
    impl_->pilot_phase_correction = Complex(1, 0);

    // Reset adaptive equalizer state
    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);

    impl_->dbpsk_prev_equalized.clear();
    impl_->carrier_phase_initialized = false;
    impl_->carrier_phase_correction = Complex(1, 0);

    // Set state to SYNCED
    impl_->state.store(Impl::State::SYNCED);

    const float* ptr = samples.data();
    size_t remaining = samples.size();

    // === PHASE 1a: CFO estimation from training symbols ===
    // Uses symbol-to-symbol correlation (robust to multipath).
    // Training symbols are identical, so phase difference = 2π × CFO × T_symbol
    //
    // IMPORTANT: If chirp-based CFO estimation was performed (any value, including 0),
    // we TRUST that value because chirp detection is more robust at low SNR.
    // Training symbol CFO estimation requires baseband downconversion to work correctly,
    // which needs a CFO estimate in the first place (chicken-and-egg problem).
    //
    // The chirp_cfo_estimated flag is set via setFrequencyOffset() when chirp provides CFO.
    // For now, we SKIP training CFO estimation when chirp sync was used.
    // TODO: The training CFO estimation has a bug where it measures carrier phase advance
    // instead of actual CFO. Need to investigate createOFDMSymbol/complexToReal phase coherence.
    if (training_symbols >= 2 && !impl_->chirp_cfo_estimated && std::abs(impl_->freq_offset_hz) < 0.1f) {
        float cfo = impl_->estimateCFOFromTraining(ptr, training_symbols);
        impl_->freq_offset_hz = cfo;
        impl_->freq_offset_filtered = cfo;
        LOG_SYNC(INFO, "CFO from training: %.1f Hz", cfo);
    } else {
        LOG_SYNC(INFO, "Using pre-set CFO: %.1f Hz (chirp=%s)", impl_->freq_offset_hz,
                 impl_->chirp_cfo_estimated ? "yes" : "no");
    }

    // === PHASE 1b: Process training symbols for channel estimation ===
    // Even for differential modulation (DQPSK), we need channel estimation on
    // fading channels where different carriers experience different attenuation.
    //
    // estimateChannelFromLTS uses toBaseband() which advances the mixer,
    // so we don't need to advance it separately here.
    if (training_symbols > 0) {
        size_t training_samples_count = training_symbols * impl_->symbol_samples;

        // Use training symbols for channel estimation (this advances the mixer)
        impl_->estimateChannelFromLTS(ptr, training_symbols);

        ptr += training_samples_count;
        remaining -= training_samples_count;
        impl_->synced_symbol_count = training_symbols;
    }

    // Initialize reference for differential demodulation to (1,0)
    // Same as Schmidl-Cox path does
    impl_->dbpsk_prev_equalized.clear();  // Will be initialized in demodulateSymbol

    LOG_SYNC(INFO, "processPresynced: skipped %d training symbols, %zu samples remaining",
             training_symbols, remaining);

    // === PHASE 2: Process data symbols ===
    LOG_DEMOD(DEBUG, "DATA phase: first_sample=%.6f, remaining=%zu", *ptr, remaining);
    impl_->rx_buffer.insert(impl_->rx_buffer.end(), ptr, ptr + remaining);

    while (impl_->rx_buffer.size() >= impl_->symbol_samples) {
        SampleSpan sym_samples(impl_->rx_buffer.data(), impl_->symbol_samples);
        auto bb = impl_->toBaseband(sym_samples);
        auto fd = impl_->extractSymbol(bb, 0);

        // For coherent modes with pilots, we MUST call updateChannelEstimate for per-symbol
        // pilot tracking. The LTS estimate is just a starting point - pilots refine it.
        // For differential modes without pilots, we can skip it (LTS estimate is sufficient).
        if (!impl_->pilot_carrier_indices.empty()) {
            impl_->updateChannelEstimate(fd);
        }
        auto eq = impl_->equalize(fd);
        impl_->demodulateSymbol(eq, impl_->config.modulation);

        impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                               impl_->rx_buffer.begin() + impl_->symbol_samples);
        ++impl_->synced_symbol_count;
        impl_->updateQuality();
    }

    LOG_SYNC(INFO, "processPresynced: total %d symbols, got %zu soft bits",
             impl_->synced_symbol_count.load(), impl_->soft_bits.size());

    fprintf(stderr, "[OFDM-DBG] processPresynced: %d symbols, %zu soft bits, need %d\n",
            impl_->synced_symbol_count.load(), impl_->soft_bits.size(), LDPC_BLOCK_SIZE);

    return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
}

void OFDMDemodulator::reset() {
    impl_->state.store(Impl::State::SEARCHING);
    impl_->synced_symbol_count.store(0);
    impl_->idle_call_count.store(0);
    impl_->rx_buffer.clear();
    impl_->soft_bits.clear();
    impl_->demod_data.clear();
    std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
    impl_->snr_symbol_count = 0;
    impl_->estimated_snr_linear = 1.0f;
    impl_->noise_variance = 0.1f;

    impl_->freq_offset_hz = 0.0f;
    impl_->freq_offset_filtered = 0.0f;
    impl_->freq_correction_phase = 0.0f;
    impl_->chirp_cfo_estimated = false;
    impl_->symbols_since_sync = 0;
    impl_->prev_pilot_phases.clear();
    impl_->pilot_phase_correction = Complex(1, 0);

    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);
}

// =============================================================================
// CHANNEL ESTIMATOR (standalone class)
// =============================================================================

struct ChannelEstimator::Impl {
    ModemConfig config;
    std::vector<Complex> h_estimate;
    ChannelQuality quality;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , h_estimate(cfg.fft_size, Complex(1, 0))
    {}
};

ChannelEstimator::ChannelEstimator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

ChannelEstimator::~ChannelEstimator() = default;

void ChannelEstimator::updateFromPilots(const Symbol& received, const Symbol& expected) {
    for (size_t i = 0; i < received.size() && i < expected.size(); ++i) {
        if (std::norm(expected[i]) > 1e-10f) {
            Complex h = received[i] / expected[i];
            impl_->h_estimate[i] = 0.5f * h + 0.5f * impl_->h_estimate[i];
        }
    }
}

Symbol ChannelEstimator::equalize(const Symbol& received) {
    Symbol output(received.size());
    for (size_t i = 0; i < received.size(); ++i) {
        if (std::norm(impl_->h_estimate[i]) > 1e-10f) {
            output[i] = received[i] / impl_->h_estimate[i];
        } else {
            output[i] = received[i];
        }
    }
    return output;
}

ChannelQuality ChannelEstimator::getQuality() const {
    return impl_->quality;
}

void ChannelEstimator::interpolate() {
    // Simple linear interpolation
}

} // namespace ultra
