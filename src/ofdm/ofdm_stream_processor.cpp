// OFDM demodulator public interface and stream processing

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include "ultra/timing_profiler.hpp"
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "genie_tx_capture.hpp"

namespace ultra {

using namespace demod_constants;

namespace {

bool envFlagEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value && value[0] != '\0' && !(value[0] == '0' && value[1] == '\0');
}

}  // namespace

void OFDMDemodulator::Impl::beginEvmSnrFrame() {
    evm_dd_accum_ = 0.0;
    evm_ee_accum_ = 0.0;
    evm_de_accum_ = 0.0;
    evm_carrier_count_ = 0;
    last_evm_snr_valid_ = false;
    last_evm_snr_db_ = 0.0f;
    last_evm_carrier_count_ = 0;
}

void OFDMDemodulator::Impl::finalizeAndResetEvmSnr() {
    // Radio-agnostic decision-directed EVM SNR (Stage 1): finalize THIS pass's
    // gain-corrected EVM (S_dd, S_ee, S_de) before its caller can publish frame
    // metrics.  The old lifecycle finalized frame j when frame j+1 began, which
    // made the log one frame late and left the group tail unfinalized at callback.
    if (evm_carrier_count_ > 0) {
        last_evm_snr_db_ = currentEvmSnrDb();
        last_evm_snr_valid_ = true;
        last_evm_carrier_count_ = evm_carrier_count_;
        LOG_DEMOD(INFO, "EVM SNR estimate: %.1f dB (decision-directed, %zu carriers)",
                  last_evm_snr_db_, evm_carrier_count_);
    }
    evm_dd_accum_ = 0.0;
    evm_ee_accum_ = 0.0;
    evm_de_accum_ = 0.0;
    evm_carrier_count_ = 0;
}

void OFDMDemodulator::Impl::resetFailureAttributionDiagnostics() {
    current_data_symbol_index_ = 0;
    failure_diag_carriers_.clear();
    failure_diag_symbols_.clear();
    failure_diag_evm_.clear();
    failure_diag_norm_evm_.clear();
    failure_diag_empirical_var_sum_ = 0.0;
    failure_diag_empirical_var_count_ = 0;
    failure_diag_last_symbol_empirical_var_ = 0.0f;
    failure_diag_last_symbol_empirical_valid_ = false;
    failure_diag_llr_sigma2_sum_ = 0.0;
    failure_diag_llr_sigma2_count_ = 0;
    failure_diag_llr_base_sigma2_sum_ = 0.0;
    failure_diag_inner_evm2_sum_ = 0.0;
    failure_diag_inner_evm2_count_ = 0;
    failure_diag_outer_evm2_sum_ = 0.0;
    failure_diag_outer_evm2_count_ = 0;
}

bool OFDMDemodulator::Impl::qam16FailureAttributionDiagEnabled() const {
    return envFlagEnabled("ULTRA_FAILURE_ATTRIBUTION");
}

bool OFDMDemodulator::Impl::qam16GenieSigmaEmpiricalEnabled() const {
    return envFlagEnabled("ULTRA_QAM16_GENIE_SIGMA_EMPIRICAL");
}

bool OFDMDemodulator::Impl::qam16GenieChannelTwoPathEnabled() const {
    return envFlagEnabled("ULTRA_QAM16_GENIE_CHANNEL_TWOPATH_LS");
}

bool OFDMDemodulator::Impl::genieLtsFreezeEnabled() const {
    return envFlagEnabled("ULTRA_GENIE_LTS_FREEZE");
}

std::string OFDMDemodulator::Impl::getFailureAttributionDiagnosticsText() const {
    auto percentile = [](std::vector<float> values, double q) -> float {
        if (values.empty()) {
            return 0.0f;
        }
        std::sort(values.begin(), values.end());
        const double pos = q * static_cast<double>(values.size() - 1);
        const size_t lo = static_cast<size_t>(std::floor(pos));
        const size_t hi = std::min(lo + 1, values.size() - 1);
        const double frac = pos - static_cast<double>(lo);
        return static_cast<float>(values[lo] * (1.0 - frac) + values[hi] * frac);
    };

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    if (failure_diag_symbols_.empty() || failure_diag_evm_.empty()) {
        oss << "eq_diag=empty";
        return oss.str();
    }

    double evm_sum = 0.0;
    double norm_evm_sum = 0.0;
    double inside_noise = 0.0;
    double min_abs_h = std::numeric_limits<double>::max();
    double mean_abs_h_sum = 0.0;
    double mean_snr_db_sum = 0.0;
    double cpe_abs_sum = 0.0;
    double phase_ramp_abs_sum = 0.0;
    double phase_slope_abs_sum = 0.0;
    double edge_evm_sum = 0.0;
    double edge_norm_evm_sum = 0.0;
    size_t edge_samples = 0;

    const size_t last_symbol = failure_diag_symbols_.empty()
        ? 0
        : failure_diag_symbols_.back().symbol_index;
    for (const auto& symbol : failure_diag_symbols_) {
        if (symbol.samples == 0) {
            continue;
        }
        evm_sum += symbol.evm_sum;
        norm_evm_sum += symbol.norm_evm_sum;
        inside_noise += static_cast<double>(symbol.inside_noise);
        min_abs_h = std::min(min_abs_h, static_cast<double>(symbol.min_abs_h));
        mean_abs_h_sum += symbol.mean_abs_h;
        mean_snr_db_sum += symbol.mean_snr_db;
        cpe_abs_sum += std::abs(symbol.cpe_rad);
        phase_ramp_abs_sum += std::abs(symbol.phase_ramp_edge_rad);
        phase_slope_abs_sum += std::abs(symbol.phase_slope_rad_per_bin);

        const bool edge_symbol = symbol.symbol_index < 2 ||
                                 symbol.symbol_index + 2 >= last_symbol;
        if (edge_symbol) {
            edge_evm_sum += symbol.evm_sum;
            edge_norm_evm_sum += symbol.norm_evm_sum;
            edge_samples += symbol.samples;
        }
    }

    const double sample_count = static_cast<double>(failure_diag_evm_.size());
    const double symbol_count = static_cast<double>(failure_diag_symbols_.size());
    const double edge_count = static_cast<double>(std::max<size_t>(edge_samples, 1));
    const double abs_h_min = std::isfinite(min_abs_h) ? min_abs_h : 0.0;
    const double empirical_sigma2 =
        failure_diag_empirical_var_count_ > 0
            ? failure_diag_empirical_var_sum_ /
                  static_cast<double>(failure_diag_empirical_var_count_)
            : 0.0;
    const double llr_sigma2 =
        failure_diag_llr_sigma2_count_ > 0
            ? failure_diag_llr_sigma2_sum_ /
                  static_cast<double>(failure_diag_llr_sigma2_count_)
            : 0.0;
    const double llr_base_sigma2 =
        failure_diag_llr_sigma2_count_ > 0
            ? failure_diag_llr_base_sigma2_sum_ /
                  static_cast<double>(failure_diag_llr_sigma2_count_)
            : 0.0;
    const double sigma2_ratio =
        empirical_sigma2 > 0.0 ? llr_sigma2 / empirical_sigma2 : 0.0;
    const double inner_evm2 =
        failure_diag_inner_evm2_count_ > 0
            ? failure_diag_inner_evm2_sum_ /
                  static_cast<double>(failure_diag_inner_evm2_count_)
            : 0.0;
    const double outer_evm2 =
        failure_diag_outer_evm2_count_ > 0
            ? failure_diag_outer_evm2_sum_ /
                  static_cast<double>(failure_diag_outer_evm2_count_)
            : 0.0;

    oss << "eq_diag samples=" << failure_diag_evm_.size()
        << " symbols=" << failure_diag_symbols_.size()
        << " carriers=" << failure_diag_carriers_.size()
        << " evm_mean=" << (evm_sum / sample_count)
        << " evm_p95=" << percentile(failure_diag_evm_, 0.95)
        << " norm_evm_mean=" << (norm_evm_sum / sample_count)
        << " norm_evm_p95=" << percentile(failure_diag_norm_evm_, 0.95)
        << " inside_noise_frac=" << (inside_noise / sample_count)
        << " mean_absH=" << (mean_abs_h_sum / symbol_count)
        << " min_absH=" << abs_h_min
        << " mean_carrier_snr_db=" << (mean_snr_db_sum / symbol_count)
        << " llr_sigma2_mean=" << llr_sigma2
        << " llr_base_sigma2_mean=" << llr_base_sigma2
        << " empirical_posteq_sigma2=" << empirical_sigma2
        << " llr_to_empirical_sigma2=" << sigma2_ratio
        << " cpe_abs_mean_deg=" << (cpe_abs_sum / symbol_count) * 180.0 / M_PI
        << " phase_slope_abs_mean_mrad_per_bin="
        << (phase_slope_abs_sum / symbol_count) * 1000.0
        << " phase_ramp_edge_abs_mean_deg="
        << (phase_ramp_abs_sum / symbol_count) * 180.0 / M_PI
        << " inner_evm2_mean=" << inner_evm2
        << " outer_evm2_mean=" << outer_evm2
        << " outer_to_inner_evm2="
        << (inner_evm2 > 0.0 ? outer_evm2 / inner_evm2 : 0.0)
        << " edge_evm_mean=" << (edge_evm_sum / edge_count)
        << " edge_norm_evm_mean=" << (edge_norm_evm_sum / edge_count);

    oss << " symbol_metrics=[";
    for (size_t i = 0; i < failure_diag_symbols_.size(); ++i) {
        if (i) oss << ",";
        const auto& symbol = failure_diag_symbols_[i];
        const double count = static_cast<double>(std::max<size_t>(symbol.samples, 1));
        oss << symbol.symbol_index
            << ":evm=" << (symbol.evm_sum / count)
            << ":nEvm=" << (symbol.norm_evm_sum / count)
            << ":inside=" << (static_cast<double>(symbol.inside_noise) / count)
            << ":minH=" << symbol.min_abs_h
            << ":meanH=" << symbol.mean_abs_h
            << ":snr=" << symbol.mean_snr_db
            << ":cpeDeg=" << (symbol.cpe_rad * 180.0f / static_cast<float>(M_PI))
            << ":rampDeg=" << (symbol.phase_ramp_edge_rad * 180.0f / static_cast<float>(M_PI));
    }
    oss << "]";

    oss << " carrier_metrics=[";
    for (size_t i = 0; i < failure_diag_carriers_.size(); ++i) {
        if (i) oss << ",";
        const auto& carrier = failure_diag_carriers_[i];
        const double count = static_cast<double>(std::max<size_t>(carrier.samples, 1));
        oss << carrier.logical_carrier
            << ":bin=" << carrier.fft_index
            << ":evm=" << (carrier.evm_sum / count)
            << ":nEvm=" << (carrier.norm_evm_sum / count)
            << ":absH=" << (carrier.abs_h_sum / count)
            << ":snr=" << (carrier.snr_db_sum / count)
            << ":inside=" << (static_cast<double>(carrier.inside_noise) / count);
    }
    oss << "]";

    return oss.str();
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

                // Report true preamble start for diagnostics/tests.
                // In practice the Schmidl-Cox coarse peak tends to land one symbol late,
                // so refined_lts_start is commonly the SECOND LTS symbol.
                // Preamble layout: 4 STS + 2 LTS.
                if (refined_lts_start >= 5 * preamble_symbol_len) {
                    impl_->last_sync_offset = refined_lts_start - 5 * preamble_symbol_len;
                } else {
                    impl_->last_sync_offset = 0;
                }

                // Consume preamble up through last LTS. With second-LTS lock, data starts
                // one symbol after refined_lts_start.
                size_t consume = refined_lts_start + preamble_symbol_len + impl_->manual_timing_offset;
                LOG_SYNC(DEBUG, "Consume calc: refined_lts=%zu + preamble_sym=%zu + offset=%d = %zu",
                        refined_lts_start, preamble_symbol_len, impl_->manual_timing_offset, consume);
                impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                       impl_->rx_buffer.begin() + consume);

                // Transition to SYNCED state
                impl_->state.store(Impl::State::SYNCED);
                impl_->synced_symbol_count.store(0);
                impl_->mixer.reset();

                // Reset channel estimate to unity - will be updated by pilot tracking
                std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
                impl_->snr_symbol_count = 0;
                impl_->estimated_snr_linear = 1.0f;
                impl_->last_snr_db_estimate_valid = false;
                impl_->last_snr_db_estimate = 0.0f;
                impl_->noise_variance = 0.1f;
                impl_->prev_pilot_phases.clear();
                impl_->prev_pilot_logical_indices.clear();
                impl_->resetPilotFadingStats();

                // Reset adaptive equalizer
                std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
                std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
                std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);

                impl_->carrier_erasure_flags_.clear();
                impl_->carrier_eq_mag_ema_.clear();
                impl_->resetMeasuredResidual();
                impl_->carrier_eq_mag_var_.clear();
                impl_->dd_qam16_channel_observations_.clear();
                impl_->dd_qam16_measurement_var_.clear();
                impl_->dd_qam16_reliability_.clear();
                impl_->dd_qam16_channel_var_.clear();
                impl_->resetWienerPilotHistory();
                impl_->resetFailureAttributionDiagnostics();
                impl_->carrier_phase_initialized = false;
                impl_->carrier_phase_correction = Complex(1, 0);
                impl_->timing_offset_samples = 0.0f;
                LOG_SYNC(INFO, "Schmidl-Cox sync complete, CFO=%.1f Hz, ready for data", coarse_cfo);
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
                                      impl_->idle_call_count.load() >= 2 &&
                                      impl_->soft_bits.empty();

        if (should_check_preamble && impl_->rx_buffer.size() >= preamble_total_len) {
            size_t search_limit = std::min(impl_->rx_buffer.size() - preamble_total_len,
                                           impl_->symbol_samples * 2);
            constexpr size_t STEP = 8;

            for (size_t offset = 0; offset <= search_limit; offset += STEP) {
                if (!impl_->hasMinimumEnergy(offset, correlation_window)) {
                    continue;
                }

                float corr = impl_->measureCorrelation(offset);
                if (corr > impl_->sync_threshold) {
                    size_t sts_start = offset;
                    size_t refined_lts_start = impl_->refineLTSTiming(sts_start);

                    if (refined_lts_start == SIZE_MAX) {
                        LOG_SYNC(DEBUG, "SYNCED: LTS confirmation failed at offset %zu, continuing", offset);
                        continue;
                    }

                    size_t consume = refined_lts_start + preamble_symbol_len;

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
                    impl_->last_snr_db_estimate_valid = false;
                    impl_->last_snr_db_estimate = 0.0f;
                    impl_->noise_variance = 0.1f;
                    impl_->prev_pilot_phases.clear();
                    impl_->prev_pilot_logical_indices.clear();
                    impl_->resetPilotFadingStats();

                    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
                    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
                    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);

                    impl_->carrier_erasure_flags_.clear();
                    impl_->carrier_eq_mag_ema_.clear();
                    impl_->resetMeasuredResidual();
                    impl_->carrier_eq_mag_var_.clear();
                    impl_->dd_qam16_channel_observations_.clear();
                    impl_->dd_qam16_measurement_var_.clear();
                    impl_->dd_qam16_reliability_.clear();
                    impl_->dd_qam16_channel_var_.clear();
                    impl_->resetWienerPilotHistory();
                    impl_->resetFailureAttributionDiagnostics();
                    impl_->carrier_phase_initialized = false;
                    impl_->carrier_phase_correction = Complex(1, 0);
                    impl_->constellation_air_bit_index_ = 0;
                    impl_->constellation_valid_air_bits_ = static_cast<size_t>(-1);
                    impl_->constellation_capacity_air_bits_ = 0;

                    break;
                }
            }
        }

        size_t soft_bits_before = impl_->soft_bits.size();
        LOG_DEMOD(DEBUG, "SYNCED: buffer=%zu samples, symbol_samples=%zu, can process %zu symbols",
                  impl_->rx_buffer.size(), impl_->symbol_samples, impl_->rx_buffer.size() / impl_->symbol_samples);

        // Process all complete symbols
        size_t symbols_processed = 0;
        // DIAGNOSTIC (genie / ULTRA_EQ_TRACE): the streaming SYNCED path carries NO
        // absolute training anchor and its current_data_symbol_index_ is a running
        // synced_symbol_count, not a frame-relative index — so mark the pass
        // NOT-presynced and let the genie decline it rather than mispair.
        if (ultra::genie::passContextNeeded()) {
            auto& pc = ultra::genie::passContext();
            pc.symbol_samples = impl_->symbol_samples;
            pc.training_symbols = 0;
            pc.presynced = false;
            auto& tr = ultra::genie::eqTrace();
            if (tr.enabled && impl_->rx_buffer.size() >= impl_->symbol_samples) {
                std::fprintf(stderr,
                             "[eqtrace] --- PASS %ld site=%-14s STREAM-SYNCED buffered=%zu "
                             "data_syms<=%zu ldpc_block=%zu\n",
                             ++tr.pass, tr.site, impl_->rx_buffer.size(),
                             impl_->rx_buffer.size() / impl_->symbol_samples,
                             impl_->active_ldpc_block_size);
            }
        }
        while (impl_->rx_buffer.size() >= impl_->symbol_samples) {
            impl_->current_data_symbol_index_ =
                static_cast<size_t>(impl_->synced_symbol_count.load());
            impl_->activateCarrierPattern(impl_->current_data_symbol_index_);

            SampleSpan sym_samples(impl_->rx_buffer.data(), impl_->symbol_samples);
            const auto& baseband = impl_->toBaseband(sym_samples);
            const auto& freq_domain = impl_->extractSymbol(baseband, 0);

            impl_->updateChannelEstimate(freq_domain);

            const auto& equalized = impl_->equalize(freq_domain);
            impl_->demodulateSymbol(equalized, impl_->config.modulation);

            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + impl_->symbol_samples);
            ++symbols_processed;
            int sym_count = ++impl_->synced_symbol_count;

            impl_->updateQuality();

            // Break early once we have enough for a codeword
            // This prevents processing multiple frames in one call
            if (impl_->soft_bits.size() >= impl_->active_ldpc_block_size) {
                LOG_DEMOD(DEBUG, "Have %zu soft bits (>= %zu), returning early",
                          impl_->soft_bits.size(), impl_->active_ldpc_block_size);
                break;
            }

            if (sym_count > MAX_SYMBOLS_BEFORE_TIMEOUT) {
                LOG_SYNC(WARN, "Sync timeout after %d symbols (%zu soft bits accumulated), resetting to SEARCHING",
                         sym_count, impl_->soft_bits.size());
                impl_->state.store(Impl::State::SEARCHING);
                impl_->synced_symbol_count.store(0);
                impl_->idle_call_count.store(0);
                return impl_->soft_bits.size() >= impl_->active_ldpc_block_size;
            }
        }

        if (symbols_processed > 0) {
            float snr_db = (impl_->estimated_snr_linear > 0)
                ? 10.0f * std::log10(impl_->estimated_snr_linear) : 0.0f;
            LOG_DEMOD(INFO, "Processed %zu symbols, soft_bits=%zu (+%zu), SNR=%.1f dB (%s), CFO=%.1f Hz",
                      symbols_processed, impl_->soft_bits.size(),
                      impl_->soft_bits.size() - soft_bits_before,
                      snr_db, snrSourceToString(SNRSource::OFDM_INTERNAL),
                      impl_->freq_offset_hz);
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
                return impl_->soft_bits.size() >= impl_->active_ldpc_block_size;
            }
        } else {
            impl_->idle_call_count.store(0);
        }

        bool has_codeword = impl_->soft_bits.size() >= impl_->active_ldpc_block_size;

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

    // 2026-05-28: use runtime active_ldpc_block_size (default 648; set to 1944
    // by the streaming decoder when a burst announces Z=81) so we return one
    // full codeword's worth, not the legacy 648 chunk that BurstInterleaver
    // then rejects for "soft bits size mismatch".
    const size_t block = impl_->active_ldpc_block_size;
    if (impl_->soft_bits.size() <= block) {
        auto bits = std::move(impl_->soft_bits);
        impl_->soft_bits.clear();
        return bits;
    } else {
        std::vector<float> bits(impl_->soft_bits.begin(),
                                impl_->soft_bits.begin() + block);
        impl_->soft_bits.erase(impl_->soft_bits.begin(),
                               impl_->soft_bits.begin() + block);
        return bits;
    }
}

void OFDMDemodulator::setActiveLDPCBlockSize(size_t bits) {
    if (bits == 0) return;
    impl_->active_ldpc_block_size = bits;
}

size_t OFDMDemodulator::getActiveLDPCBlockSize() const {
    return impl_->active_ldpc_block_size;
}

ChannelQuality OFDMDemodulator::getChannelQuality() const {
    return impl_->quality;
}

float OFDMDemodulator::getEstimatedSNR() const {
    return 10.0f * std::log10(impl_->estimated_snr_linear);
}

bool OFDMDemodulator::hasLastOFDMBroadbandSNREstimate() const {
    return impl_->last_snr_db_estimate_valid;
}

float OFDMDemodulator::getLastOFDMBroadbandSNREstimate() const {
    return impl_->last_snr_db_estimate;
}

// Radio-agnostic decision-directed EVM SNR (Stage 1). Live on-demand value from the
// current burst's accumulation (no lag); no reference power / offset / noise-shape.
bool OFDMDemodulator::hasEvmSnr() const {
    return impl_->last_evm_snr_valid_;
}

float OFDMDemodulator::getEvmSnrDb() const {
    return impl_->last_evm_snr_db_;
}

bool OFDMDemodulator::takeEvmSnr(float& evm_snr_db, size_t& carrier_count) {
    if (!impl_->last_evm_snr_valid_) {
        return false;
    }
    evm_snr_db = impl_->last_evm_snr_db_;
    carrier_count = impl_->last_evm_carrier_count_;
    impl_->last_evm_snr_valid_ = false;
    return true;
}

float OFDMDemodulator::getFrequencyOffset() const {
    return impl_->freq_offset_hz;
}

float OFDMDemodulator::getLastTimingOffsetSamples() const {
    return impl_->timing_offset_samples;
}

float OFDMDemodulator::getFadingIndex() const {
    return impl_->public_fading_index;
}

float OFDMDemodulator::getLastPilotFrequencyCV() const {
    return impl_->last_pilot_frequency_cv;
}

float OFDMDemodulator::getLastPilotTemporalCV() const {
    return impl_->last_pilot_temporal_cv;
}

float OFDMDemodulator::getLastPilotSymbolMeanCV() const {
    return impl_->last_pilot_symbol_mean_cv;
}

void OFDMDemodulator::setWienerChannelParams(float doppler_hz, float delay_spread_s) {
    // Sanity-bound to the same ranges the env knobs accept; ignore implausible values.
    if (doppler_hz > 0.0f && doppler_hz < 10.0f &&
        delay_spread_s > 0.0f && delay_spread_s < 0.01f) {
        impl_->wiener_doppler_hz_override_ = doppler_hz;
        impl_->wiener_delay_spread_s_override_ = delay_spread_s;
        impl_->wiener_params_override_active_ = true;
    }
}


float OFDMDemodulator::getLastLTSSignalPower() const {
    return impl_->last_lts_signal_power;
}

float OFDMDemodulator::getLastLTSChannelMagnitude() const {
    return impl_->last_lts_channel_magnitude;
}

std::vector<float> OFDMDemodulator::getCarrierGammaSnapshot() const {
    std::vector<float> gammas;
    const float nv = std::max(impl_->noise_variance, 1e-12f);
    gammas.reserve(impl_->data_carrier_indices.size());
    for (int idx : impl_->data_carrier_indices) {
        if (idx >= 0 &&
            static_cast<size_t>(idx) < impl_->channel_estimate.size()) {
            gammas.push_back(std::norm(impl_->channel_estimate[idx]) / nv);
        }
    }
    return gammas;
}

float OFDMDemodulator::getLastLTSNoiseVariance() const {
    return impl_->noise_variance;
}

float OFDMDemodulator::getLastLTSPairChi() const {
    return impl_->last_lts_pair_chi;
}

float OFDMDemodulator::getLastLTSResidualCFOHz() const {
    return impl_->last_lts_residual_cfo_hz;
}

std::string OFDMDemodulator::getFailureAttributionDiagnosticsText() const {
    return impl_->getFailureAttributionDiagnosticsText();
}

// ============================================================================
// OFFLINE TEST HOOKS (tools/lts_estimate_mse.cpp)
// ============================================================================

void OFDMDemodulator::setLtsDftDenoise(bool on, int taps) {
    impl_->dft_denoise_enabled_ = on;
    impl_->dft_denoise_taps_ = taps;
}

void OFDMDemodulator::setLtsCfoAvg(bool on) {
    impl_->lts_cfo_avg_enabled_ = on;
}

void OFDMDemodulator::estimateChannelFromLTSTest(const float* samples,
                                                 size_t num_symbols) {
    impl_->estimateChannelFromLTS(samples, num_symbols);
}

std::vector<Complex> OFDMDemodulator::getActiveChannelEstimate() const {
    std::vector<Complex> out;
    out.reserve(impl_->all_carrier_fft_indices.size());
    for (int fft_idx : impl_->all_carrier_fft_indices) {
        if (fft_idx >= 0 &&
            static_cast<size_t>(fft_idx) < impl_->channel_estimate.size()) {
            out.push_back(impl_->channel_estimate[fft_idx]);
        } else {
            out.push_back(Complex(0, 0));
        }
    }
    return out;
}

void OFDMDemodulator::setRXCarrierErasureEnabled(bool enabled) {
    impl_->rx_carrier_erasure_enabled_ = enabled;
}

void OFDMDemodulator::setCfoPriorSigmaHz(double sigma_hz) {
    impl_->cfo_prior_sigma_hz = (sigma_hz >= 0.0) ? sigma_hz : 0.0;
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

void OFDMDemodulator::setFrequencyOffsetWithPhase(float cfo_hz, float initial_phase_rad) {
    LOG_DEMOD(INFO, "setFrequencyOffsetWithPhase: CFO=%.2f Hz, initial_phase=%.1f° (was CFO=%.2f Hz)",
              cfo_hz, initial_phase_rad * 180.0f / M_PI, impl_->freq_offset_hz);
    impl_->freq_offset_hz = cfo_hz;
    impl_->freq_offset_filtered = cfo_hz;
    // Set initial correction phase to match accumulated CFO phase at this point
    impl_->freq_correction_phase = initial_phase_rad;
    // Mark that CFO was explicitly provided
    impl_->chirp_cfo_estimated = true;
}

Symbol OFDMDemodulator::getConstellationSymbols() const {
    std::lock_guard<std::mutex> lock(impl_->constellation_mutex);
    return impl_->constellation_symbols;
}

Modulation OFDMDemodulator::getConstellationModulation() const {
    std::lock_guard<std::mutex> lock(impl_->constellation_mutex);
    return impl_->constellation_mod_;
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

// =============================================================================
// POST-FEC DATA-AIDED CHANNEL ESTIMATION (ULTRA_ITERATIVE_CHEST, default OFF)
// Rationale, adaptivity derivation and safety argument: src/ofdm/iterative_chest.hpp
// =============================================================================

void OFDMDemodulator::setDataAidedFeedbackEnabled(bool enabled) {
    if (impl_->data_aided_enabled_ == enabled) return;
    impl_->data_aided_enabled_ = enabled;
    // The storage domain must be uniform across the whole history (never a mix of
    // sloped and flat samples at one carrier), so it is switched with the feature
    // and the existing history is dropped at the switch.
    impl_->wiener_history_flat_ = enabled;
    impl_->resetWienerPilotHistory();
    impl_->da_rx_grid_.clear();
    impl_->da_logical_.clear();
    impl_->da_grid_origin_ = -1;
    impl_->da_pending_origin_ = -1;
    if (!enabled) {
        impl_->wiener_carry_armed_ = false;
        impl_->wiener_symbol_base_ = 0;
    }
}

void OFDMDemodulator::setChannelHistoryFrameOrigin(long long abs_sample) {
    impl_->da_pending_origin_ = abs_sample;
}

void OFDMDemodulator::armChannelHistoryCarry(bool armed) {
    impl_->wiener_carry_armed_ = armed && impl_->data_aided_enabled_;
}

void OFDMDemodulator::clearChannelHistory() {
    impl_->resetWienerPilotHistory();
    impl_->da_rx_grid_.clear();
    impl_->da_logical_.clear();
    impl_->da_grid_origin_ = -1;
    impl_->da_pending_origin_ = -1;
    impl_->wiener_symbol_base_ = 0;
}

size_t OFDMDemodulator::ingestDataAidedGrid(const Symbol& x_grid,
                                            long long expect_origin) {
    return impl_->ingestDataAidedGrid(x_grid, expect_origin);
}

size_t OFDMDemodulator::dataAidedRetainedSymbolCount() const {
    return impl_->da_rx_grid_.size();
}

size_t OFDMDemodulator::dataAidedRetainedCarrierCount() const {
    return impl_->da_rx_grid_.empty() ? 0 : impl_->da_rx_grid_[0].size();
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

    // This call owns exactly one physical-frame EVM observation.  Invalidate
    // the prior pass before every exit, including the too-short-input path.
    impl_->beginEvmSnrFrame();
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
    impl_->last_snr_db_estimate_valid = false;
    impl_->last_snr_db_estimate = 0.0f;
    impl_->noise_variance = 0.1f;
    impl_->resetPilotFadingStats();

    // Preserve pre-set CFO and phase (e.g., from chirp-based estimation)
    // If CFO was explicitly set via setFrequencyOffsetWithPhase(), the phase
    // contains the accumulated CFO phase at processing start - DON'T reset it!
    // impl_->freq_offset_hz = 0.0f;  // KEEP the pre-set value!
    // impl_->freq_offset_filtered = 0.0f;  // KEEP the pre-set value!
    // impl_->freq_correction_phase = 0.0f;  // KEEP the pre-set value if explicitly set!
    LOG_SYNC(INFO, "processPresynced: pre-set CFO=%.2f Hz, initial_phase=%.1f°",
             impl_->freq_offset_hz, impl_->freq_correction_phase * 180.0f / M_PI);
    impl_->symbols_since_sync = 0;
    impl_->prev_pilot_phases.clear();
    impl_->prev_pilot_logical_indices.clear();
    impl_->pilot_phase_correction = Complex(1, 0);

    // Reset adaptive equalizer state
    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);

    impl_->carrier_erasure_flags_.clear();
    impl_->carrier_eq_mag_ema_.clear();
    impl_->resetMeasuredResidual();
    impl_->carrier_eq_mag_var_.clear();
    impl_->dd_qam16_channel_observations_.clear();
    impl_->dd_qam16_measurement_var_.clear();
    impl_->dd_qam16_reliability_.clear();
    impl_->dd_qam16_channel_var_.clear();
    // ULTRA_ITERATIVE_CHEST: with the carry armed the previous frame's VERIFIED
    // data-aided observations must survive this frame boundary — that transfer is
    // the entire lever (a frame's own decoded bits cannot refine its own channel:
    // FrameInterleaver spreads every 16QAM carrier across all four codewords).
    // Unarmed (the default) this resets exactly as before.
    //
    // A pass with NO FRESH absolute origin (the origin is announced per burst data
    // frame and consumed below) has no trustworthy place on the absolute time axis
    // — a control peek or an out-of-band probe would otherwise inherit the previous
    // frame's timestamp and its observations would land at the wrong age. Such a
    // pass is treated as unarmed: full reset, base 0, i.e. exactly the production
    // behaviour. Safe because the decoder re-arms before EVERY burst data frame.
    const bool da_pass = impl_->data_aided_enabled_ &&
                         impl_->da_pending_origin_ >= 0 &&
                         impl_->symbol_samples > 0;
    if (!da_pass) {
        impl_->wiener_carry_armed_ = false;
    }
    if (!impl_->wiener_carry_armed_) {
        impl_->resetWienerPilotHistory();
    }
    impl_->da_rx_grid_.clear();
    impl_->da_logical_.clear();
    impl_->da_grid_origin_ = -1;
    impl_->resetFailureAttributionDiagnostics();
    impl_->carrier_phase_initialized = false;
    impl_->carrier_phase_correction = Complex(1, 0);
    impl_->constellation_air_bit_index_ = 0;
    impl_->constellation_valid_air_bits_ = static_cast<size_t>(-1);
    impl_->constellation_capacity_air_bits_ = 0;

    // The absolute symbol origin of THIS frame, latched before any LTS/pilot
    // observation is pushed so every sample in the history shares one time axis.
    impl_->wiener_symbol_base_ =
        da_pass ? (impl_->da_pending_origin_ /
                   static_cast<long long>(impl_->symbol_samples))
                : 0;

    // Clear constellation symbols so we only show the current frame's data
    {
        std::lock_guard<std::mutex> lock(impl_->constellation_mutex);
        impl_->constellation_symbols.clear();
    }

    // Set state to SYNCED
    impl_->state.store(Impl::State::SYNCED);

    const float* ptr = samples.data();
    size_t remaining = samples.size();

    // === PHASE 1a: CFO handling ===
    // If chirp-based CFO was provided, TRUST it completely.
    // The training symbol correlation doesn't work well because:
    // 1. Training symbols are identical in freq domain but not time domain
    // 2. Mixer phase advancement between symbols causes spurious phase
    //
    // TODO: Implement proper two-stage CFO using cyclic prefix correlation
    // or frequency-domain phase estimation after initial CFO correction.
    if (impl_->chirp_cfo_estimated) {
        LOG_SYNC(INFO, "Using chirp CFO: %.1f Hz (trusted)", impl_->freq_offset_hz);
    } else if (training_symbols >= 2 && std::abs(impl_->freq_offset_hz) < 0.1f) {
        // No chirp CFO available - try training estimation (may be inaccurate)
        float cfo = impl_->estimateCFOFromTraining(ptr, training_symbols, 0.0f);
        impl_->freq_offset_hz = cfo;
        impl_->freq_offset_filtered = cfo;
        LOG_SYNC(INFO, "CFO from training: %.1f Hz (no chirp available)", cfo);
    } else {
        LOG_SYNC(INFO, "Using pre-set CFO: %.1f Hz", impl_->freq_offset_hz);
    }

    // === PHASE 1b: Process training symbols for channel estimation ===
    // Even for differential modulation (DQPSK), we need channel estimation on
    // fading channels where different carriers experience different attenuation.
    //
    // estimateChannelFromLTS uses toBaseband() which advances the mixer,
    // so we don't need to advance it separately here.
    if (training_symbols > 0) {
        size_t training_samples_count = training_symbols * impl_->symbol_samples;

        // Bounds check: a too-short input would underflow `remaining` (size_t)
        // when we subtract training_samples_count below, leading to a billion-
        // sample vector::insert and a std::length_error crash. Seen on real
        // hardware with small SDL2 audio buffers when a partial chirp lock
        // produced LTS estimates with NaN/inf values and a 1509-sample input.
        if (training_samples_count > remaining) {
            LOG_DEMOD(WARN,
                "processPresynced: short input — got %zu samples but training "
                "alone needs %zu; aborting",
                remaining, training_samples_count);
            return false;
        }

        // Use training symbols for channel estimation (this advances the mixer)
        {
            timing::ScopedTimer _profile_(timing::globalDecoderProfile().lts_channel_estimate);
            impl_->estimateChannelFromLTS(ptr, training_symbols);
        }

        ptr += training_samples_count;
        remaining -= training_samples_count;
        impl_->synced_symbol_count = training_symbols;
    }

    LOG_SYNC(INFO, "processPresynced: skipped %d training symbols, %zu samples remaining",
             training_symbols, remaining);

    // DIAGNOSTIC (genie / ULTRA_EQ_TRACE): publish the absolute sample identity of the
    // symbols this pass is about to equalize. abs_train was set by the waveform; the
    // stride and the training skip are only known here. Written only when a diagnostic
    // consumer is armed — inert in production.
    if (ultra::genie::passContextNeeded()) {
        auto& pc = ultra::genie::passContext();
        pc.symbol_samples = impl_->symbol_samples;
        pc.training_symbols = training_symbols;
        pc.presynced = true;
        auto& tr = ultra::genie::eqTrace();
        if (tr.enabled) {
            std::fprintf(stderr,
                         "[eqtrace] --- PASS %ld site=%-14s presynced in=%zu training=%d "
                         "data_syms=%zu sym_samples=%zu abs_train=%zu ldpc_block=%zu mod=%d\n",
                         ++tr.pass, tr.site, samples.size(), training_symbols,
                         remaining / impl_->symbol_samples, impl_->symbol_samples,
                         pc.abs_train, impl_->active_ldpc_block_size,
                         static_cast<int>(impl_->config.modulation));
        }
    }

    // === PHASE 2: Process data symbols ===
    LOG_DEMOD(DEBUG, "DATA phase: first_sample=%.6f, remaining=%zu", *ptr, remaining);
    size_t data_offset = 0;
    size_t data_symbol_index = 0;
    const size_t complete_data_symbols = remaining / impl_->symbol_samples;
    const size_t bits_per_symbol =
        impl_->data_carrier_indices.size() *
        static_cast<size_t>(getBitsPerSymbol(impl_->config.modulation));
    impl_->constellation_air_bit_index_ = 0;
    impl_->constellation_capacity_air_bits_ = complete_data_symbols * bits_per_symbol;
    impl_->constellation_valid_air_bits_ =
        (bits_per_symbol > 0)
            ? (impl_->constellation_capacity_air_bits_ / impl_->active_ldpc_block_size) * impl_->active_ldpc_block_size
            : 0;

    while (remaining - data_offset >= impl_->symbol_samples) {
        timing::ScopedTimer _profile_(timing::globalDecoderProfile().data_symbol_loop);

        impl_->current_data_symbol_index_ = data_symbol_index;
        impl_->activateCarrierPattern(data_symbol_index);

        SampleSpan sym_samples(ptr + data_offset, impl_->symbol_samples);
        const auto& bb = impl_->toBaseband(sym_samples);
        const auto& fd = impl_->extractSymbol(bb, 0);

        // ULTRA_ITERATIVE_CHEST: retain the raw receive grid Y at this symbol's data
        // carriers. `freq_domain_scratch` is a single reused buffer overwritten every
        // symbol, so without this the frame's Y is gone by the time LDPC returns a
        // verdict. Cost is ~51 complex + 51 uint16 per symbol (~6-12 KB/frame) and
        // NOTHING is retained while the knob is off. The logical index is stored per
        // symbol because activateCarrierPattern() ROTATES the scattered pilots, so
        // data carrier c is a different logical carrier from one symbol to the next.
        if (impl_->data_aided_enabled_) {
            impl_->da_rx_grid_.emplace_back();
            impl_->da_logical_.emplace_back();
            auto& y_row = impl_->da_rx_grid_.back();
            auto& l_row = impl_->da_logical_.back();
            const size_t nd = impl_->data_carrier_indices.size();
            y_row.reserve(nd);
            l_row.reserve(nd);
            for (size_t c = 0; c < nd; ++c) {
                const int fft_idx = impl_->data_carrier_indices[c];
                if (fft_idx < 0 || static_cast<size_t>(fft_idx) >= fd.size() ||
                    c >= impl_->data_logical_carrier_indices.size()) {
                    y_row.clear();
                    l_row.clear();
                    break;
                }
                y_row.push_back(fd[fft_idx]);
                l_row.push_back(static_cast<uint16_t>(
                    impl_->data_logical_carrier_indices[c]));
            }
        }

        // Per-symbol pilot tracking: update channel estimate from pilot observations.
        // For differential modes: update only |H| (magnitude) to track fading depth,
        // while keeping phase frozen from LTS. This gives MMSE accurate amplitude
        // scaling without corrupting the differential phase relationship.
        // For coherent modes: full H update (magnitude + phase).
        if (!impl_->pilot_carrier_indices.empty()) {
            impl_->updateChannelEstimate(fd);
        }
        const auto& eq = impl_->equalize(fd);
        impl_->demodulateSymbol(eq, impl_->config.modulation);

        data_offset += impl_->symbol_samples;
        ++data_symbol_index;
        ++impl_->synced_symbol_count;
        impl_->updateQuality();
    }

    // Bind the retained receive grid to the frame it came from. ingestDataAidedGrid
    // refuses any origin but this one, which is what makes "stale Y meets fresh X"
    // structurally impossible rather than merely unlikely.
    impl_->da_grid_origin_ =
        (da_pass && !impl_->da_rx_grid_.empty()) ? impl_->da_pending_origin_ : -1;
    // CONSUME the origin. The next pass must be told its own absolute position or
    // it is not eligible for the carry (see da_pass above) — this is what stops a
    // stale timestamp from silently mislabelling an out-of-band pass.
    impl_->da_pending_origin_ = -1;

    if (data_offset < remaining) {
        impl_->rx_buffer.assign(ptr + data_offset, ptr + remaining);
    } else {
        impl_->rx_buffer.clear();
    }

    // Debug: count positive vs negative soft bits to check for all-zero demod issue
    {
        int pos_count = 0, neg_count = 0, zero_count = 0;
        float min_abs = 999, max_abs = 0;
        for (size_t i = 0; i < std::min(impl_->soft_bits.size(), size_t(648)); i++) {
            float v = impl_->soft_bits[i];
            if (v > 0.1f) pos_count++;
            else if (v < -0.1f) neg_count++;
            else zero_count++;
            float av = std::abs(v);
            if (av < min_abs) min_abs = av;
            if (av > max_abs) max_abs = av;
        }
        LOG_SYNC(INFO, "processPresynced: %d symbols, %zu soft bits. CW0 stats: pos=%d neg=%d zero=%d min_abs=%.2f max_abs=%.2f first8=[%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f]",
                 impl_->synced_symbol_count.load(), impl_->soft_bits.size(),
                 pos_count, neg_count, zero_count, min_abs, max_abs,
                 impl_->soft_bits.size() > 0 ? impl_->soft_bits[0] : 0,
                 impl_->soft_bits.size() > 1 ? impl_->soft_bits[1] : 0,
                 impl_->soft_bits.size() > 2 ? impl_->soft_bits[2] : 0,
                 impl_->soft_bits.size() > 3 ? impl_->soft_bits[3] : 0,
                 impl_->soft_bits.size() > 4 ? impl_->soft_bits[4] : 0,
                 impl_->soft_bits.size() > 5 ? impl_->soft_bits[5] : 0,
                 impl_->soft_bits.size() > 6 ? impl_->soft_bits[6] : 0,
                 impl_->soft_bits.size() > 7 ? impl_->soft_bits[7] : 0);
    }

    LOG_DEMOD(DEBUG, "OFDM processPresynced: %d symbols, %zu soft bits, need %d",
              impl_->synced_symbol_count.load(), impl_->soft_bits.size(), impl_->active_ldpc_block_size);

    const bool ready = impl_->soft_bits.size() >= impl_->active_ldpc_block_size;
    if (ready) {
        // Publish only after the last data symbol in this physical frame has
        // contributed.  A non-ready pass remains invalid (fail closed).
        impl_->finalizeAndResetEvmSnr();
    } else {
        impl_->beginEvmSnrFrame();
    }
    return ready;
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
    impl_->last_snr_db_estimate_valid = false;
    impl_->last_snr_db_estimate = 0.0f;
    impl_->beginEvmSnrFrame();  // fresh connection: drop any stale/partial EVM SNR
    impl_->noise_variance = 0.1f;
    impl_->resetPilotFadingStats();
    impl_->last_lts_signal_power = 1.0f;
    impl_->last_lts_channel_magnitude = 1.0f;
    impl_->last_lts_residual_cfo_hz = 0.0f;
    impl_->last_lts_pair_chi = 0.0f;

    impl_->freq_offset_hz = 0.0f;
    impl_->freq_offset_filtered = 0.0f;
    impl_->freq_correction_phase = 0.0f;
    impl_->chirp_cfo_estimated = false;
    impl_->symbols_since_sync = 0;
    impl_->prev_pilot_phases.clear();
    impl_->prev_pilot_logical_indices.clear();
    impl_->pilot_phase_correction = Complex(1, 0);

    // Reset mixer phase - critical for OFDM_CHIRP which calls reset() between frames
    impl_->mixer.reset();
    impl_->carrier_erasure_flags_.clear();
    impl_->carrier_eq_mag_ema_.clear();
    impl_->resetMeasuredResidual();
    impl_->carrier_eq_mag_var_.clear();
    impl_->dd_qam16_channel_observations_.clear();
    impl_->dd_qam16_measurement_var_.clear();
    impl_->dd_qam16_reliability_.clear();
    impl_->dd_qam16_channel_var_.clear();
    impl_->resetWienerPilotHistory();
    // A full reset is a re-acquisition: a carried data-aided observation is only
    // valid inside one continuous acquisition, so drop it unconditionally here.
    impl_->da_rx_grid_.clear();
    impl_->da_logical_.clear();
    impl_->da_grid_origin_ = -1;
    impl_->wiener_symbol_base_ = 0;
    impl_->resetFailureAttributionDiagnostics();
    impl_->carrier_phase_initialized = false;
    impl_->carrier_phase_correction = Complex(1, 0);
    impl_->constellation_air_bit_index_ = 0;
    impl_->constellation_valid_air_bits_ = static_cast<size_t>(-1);
    impl_->constellation_capacity_air_bits_ = 0;

    std::fill(impl_->lms_weights.begin(), impl_->lms_weights.end(), Complex(1, 0));
    std::fill(impl_->last_decisions.begin(), impl_->last_decisions.end(), Complex(0, 0));
    std::fill(impl_->rls_P.begin(), impl_->rls_P.end(), 1.0f);
}

} // namespace ultra
