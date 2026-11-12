// OFDM symbol demapping
// Part of OFDMDemodulator::Impl

#define _USE_MATH_DEFINES
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include "demodulator_impl.hpp"
#include "demodulator_constants.hpp"
#include "soft_demap.hpp"

namespace ultra {

using namespace demod_constants;

namespace {

size_t bitsPerCarrier(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK:
        case Modulation::BPSK:
            return 1;
        case Modulation::DQPSK:
        case Modulation::QPSK:
            return 2;
        case Modulation::D8PSK:
        case Modulation::QAM8:
            return 3;
        case Modulation::QAM16:
            return 4;
        case Modulation::QAM32:
            return 5;
        case Modulation::QAM64:
            return 6;
        case Modulation::QAM256:
            return 8;
        default:
            return 2;
    }
}

void appendErasureLLRs(std::vector<float>& soft_bits, Modulation mod) {
    soft_bits.insert(soft_bits.end(), bitsPerCarrier(mod), 0.0f);
}

Complex normalizePhaseOnlyForDisplay(Complex symbol) {
    const float mag = std::abs(symbol);
    if (mag < 1.0e-6f) {
        return symbol;
    }
    return symbol / mag;
}

const char* constellationDiagModName(Modulation mod) {
    switch (mod) {
        case Modulation::DBPSK: return "DBPSK";
        case Modulation::BPSK: return "BPSK";
        case Modulation::DQPSK: return "DQPSK";
        case Modulation::QPSK: return "QPSK";
        case Modulation::D8PSK: return "D8PSK";
        case Modulation::QAM8: return "QAM8";
        case Modulation::QAM16: return "16QAM";
        case Modulation::QAM32: return "QAM32";
        case Modulation::QAM64: return "QAM64";
        case Modulation::QAM256: return "QAM256";
        default: return "UNKNOWN";
    }
}

bool constellationDiagEnabled() {
    static const bool enabled = [] {
        const char* value = std::getenv("ULTRA_CONSTELLATION_DIAG");
        if (!value) return false;
        return value[0] != '\0' && value[0] != '0';
    }();
    return enabled;
}

struct ConstellationDiagStats {
    size_t count = 0;
    size_t below_0p2 = 0;
    float min_mag = 0.0f;
    float avg_mag = 0.0f;
    float max_mag = 0.0f;
};

struct ConstellationClusterStats {
    size_t occupied = 0;
    float rms_evm = 0.0f;
    float max_evm = 0.0f;
};

ConstellationDiagStats constellationDiagStats(const std::vector<Complex>& symbols) {
    ConstellationDiagStats stats;
    stats.count = symbols.size();
    if (symbols.empty()) {
        return stats;
    }

    stats.min_mag = std::numeric_limits<float>::max();
    double sum = 0.0;
    for (const auto& symbol : symbols) {
        const float mag = std::abs(symbol);
        stats.min_mag = std::min(stats.min_mag, mag);
        stats.max_mag = std::max(stats.max_mag, mag);
        sum += mag;
        if (mag < 0.2f) {
            ++stats.below_0p2;
        }
    }
    stats.avg_mag = static_cast<float>(sum / static_cast<double>(symbols.size()));
    return stats;
}

std::vector<Complex> constellationReferencePoints(Modulation mod) {
    static const float pi = 3.14159265358979f;
    std::vector<Complex> points;

    if (mod == Modulation::DBPSK || mod == Modulation::BPSK) {
        points = {Complex(-1, 0), Complex(1, 0)};
    } else if (mod == Modulation::DQPSK) {
        for (int i = 0; i < 4; ++i) {
            const float angle = i * pi / 2.0f;
            points.emplace_back(std::cos(angle), std::sin(angle));
        }
    } else if (mod == Modulation::QPSK) {
        for (int i = 0; i < 4; ++i) {
            const float angle = pi / 4.0f + i * pi / 2.0f;
            points.emplace_back(std::cos(angle), std::sin(angle));
        }
    } else if (mod == Modulation::D8PSK || mod == Modulation::QAM8) {
        const float offset = (mod == Modulation::D8PSK) ? pi / 8.0f : pi / 8.0f;
        for (int i = 0; i < 8; ++i) {
            const float angle = offset + i * pi / 4.0f;
            points.emplace_back(std::cos(angle), std::sin(angle));
        }
    } else if (mod == Modulation::QAM16 || mod == Modulation::QAM64 ||
               mod == Modulation::QAM256) {
        int points_per_axis = 4;
        if (mod == Modulation::QAM64) {
            points_per_axis = 8;
        } else if (mod == Modulation::QAM256) {
            points_per_axis = 16;
        }
        const float norm = std::sqrt((2.0f / 3.0f) *
                                     (points_per_axis * points_per_axis - 1.0f));
        points.reserve(static_cast<size_t>(points_per_axis * points_per_axis));
        for (int yi = 0; yi < points_per_axis; ++yi) {
            for (int xi = 0; xi < points_per_axis; ++xi) {
                const float x = (2.0f * xi - points_per_axis + 1) / norm;
                const float y = (2.0f * yi - points_per_axis + 1) / norm;
                points.emplace_back(x, y);
            }
        }
    }
    return points;
}

ConstellationClusterStats constellationClusterStats(
        const std::vector<Complex>& symbols, Modulation mod) {
    ConstellationClusterStats stats;
    const auto refs = constellationReferencePoints(mod);
    if (symbols.empty() || refs.empty()) {
        return stats;
    }

    float power = 0.0f;
    for (const auto& symbol : symbols) {
        power += std::norm(symbol);
    }
    const float rms = std::sqrt(power / static_cast<float>(symbols.size()));
    const float scale = (rms > 1.0e-6f) ? (1.0f / rms) : 1.0f;

    std::vector<size_t> hits(refs.size(), 0);
    double evm_sq_sum = 0.0;
    for (const auto& symbol : symbols) {
        const Complex normalized = symbol * scale;
        size_t best = 0;
        float best_dist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < refs.size(); ++i) {
            const float dist = std::norm(normalized - refs[i]);
            if (dist < best_dist) {
                best_dist = dist;
                best = i;
            }
        }
        ++hits[best];
        evm_sq_sum += best_dist;
        stats.max_evm = std::max(stats.max_evm, std::sqrt(best_dist));
    }

    for (size_t hit : hits) {
        if (hit > 0) {
            ++stats.occupied;
        }
    }
    stats.rms_evm = std::sqrt(
        static_cast<float>(evm_sq_sum / static_cast<double>(symbols.size())));
    return stats;
}

} // namespace
// =============================================================================
// SYMBOL DEMODULATION
// =============================================================================

void OFDMDemodulator::Impl::appendConstellationSymbols(
        const std::vector<Complex>& update, Modulation mod) {
    if (update.empty()) return;
    std::lock_guard<std::mutex> lock(constellation_mutex);
    // Reset the display buffer when the modulation changes so the GUI never
    // overlays different constellations (e.g. 16QAM data + QPSK ACKs) into one
    // smeared cloud. Each batch is homogeneous in modulation.
    if (mod != constellation_mod_) {
        constellation_symbols.clear();
        constellation_mod_ = mod;
    }
    constellation_symbols.insert(constellation_symbols.end(),
                                 update.begin(), update.end());
    if (constellation_symbols.size() > MAX_CONSTELLATION_SYMBOLS) {
        constellation_symbols.erase(
            constellation_symbols.begin(),
            constellation_symbols.begin() +
                (constellation_symbols.size() - MAX_CONSTELLATION_SYMBOLS));
    }

    if (constellationDiagEnabled()) {
        static std::atomic<int> append_log_count{0};
        if (append_log_count < 80) {
            const auto update_stats = constellationDiagStats(update);
            const auto buffer_stats = constellationDiagStats(constellation_symbols);
            const auto update_clusters = constellationClusterStats(update, mod);
            const auto buffer_clusters = constellationClusterStats(constellation_symbols, mod);
            LOG_DEMOD(INFO,
                      "CONSTELLATION_DIAG stage=append mod=%s update_count=%zu "
                      "update_mag=[%.4f,%.4f,%.4f] update_lt0p2=%zu/%zu "
                      "update_clusters=%zu update_evm_rms=%.4f update_evm_max=%.4f "
                      "buffer_count=%zu buffer_mag=[%.4f,%.4f,%.4f] "
                      "buffer_lt0p2=%zu/%zu buffer_clusters=%zu "
                      "buffer_evm_rms=%.4f buffer_evm_max=%.4f",
                      constellationDiagModName(mod),
                      update_stats.count,
                      update_stats.min_mag,
                      update_stats.avg_mag,
                      update_stats.max_mag,
                      update_stats.below_0p2,
                      update_stats.count,
                      update_clusters.occupied,
                      update_clusters.rms_evm,
                      update_clusters.max_evm,
                      buffer_stats.count,
                      buffer_stats.min_mag,
                      buffer_stats.avg_mag,
                      buffer_stats.max_mag,
                      buffer_stats.below_0p2,
                      buffer_stats.count,
                      buffer_clusters.occupied,
                      buffer_clusters.rms_evm,
                      buffer_clusters.max_evm);
            ++append_log_count;
        }
    }
}

void OFDMDemodulator::Impl::demodulateSymbol(const std::vector<Complex>& equalized, Modulation mod) {
    // Constellation symbols collected during demodulation (raw equalized; coherent-only)
    auto& constellation_update = constellation_update_scratch;
    constellation_update.clear();

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

    // PER-CARRIER ADAPTIVE LLR SCALING (replaces old global fading_scale):
    // Track |equalized| EMA per carrier across symbols within a frame.
    // Carriers with high magnitude variance are fading mid-frame → inflate their noise.
    // Stable carriers keep full LLR confidence. On AWGN, all carriers stable → no scaling.
    constexpr float MAG_EMA_ALPHA = 0.3f;
    if (carrier_eq_mag_ema_.size() != equalized.size()) {
        // First symbol in frame: initialize EMA to current magnitudes, zero variance
        carrier_eq_mag_ema_.resize(equalized.size());
        carrier_eq_mag_var_.resize(equalized.size(), 0.0f);
        for (size_t i = 0; i < equalized.size(); ++i)
            carrier_eq_mag_ema_[i] = std::abs(equalized[i]);
    } else {
        for (size_t i = 0; i < equalized.size(); ++i) {
            float mag = std::abs(equalized[i]);
            float delta = mag - carrier_eq_mag_ema_[i];
            carrier_eq_mag_ema_[i] += MAG_EMA_ALPHA * delta;
            carrier_eq_mag_var_[i] += MAG_EMA_ALPHA * (delta * delta - carrier_eq_mag_var_[i]);
        }
    }

    float llr_sign = llr_sign_flip ? -1.0f : 1.0f;

    // Coherent-only OFDM (thread A 2026-05-31): the differential (DBPSK/DQPSK/D8PSK)
    // reference initialization, the D8PSK two-pass dispatch, and the differential
    // scratch buffers were removed here — OFDM never carries a differential
    // modulation now (differential lives in MC-DPSK).

    size_t constellation_padding_skipped = 0;
    size_t constellation_weak_diff_skipped = 0;
    const size_t carrier_bits = bitsPerCarrier(mod);
    for (size_t i = 0; i < equalized.size(); ++i) {
        const size_t air_bit_index = constellation_air_bit_index_;
        constellation_air_bit_index_ += carrier_bits;
        const bool display_payload_carrier =
            air_bit_index < constellation_valid_air_bits_;
        const auto& sym = equalized[i];
        float base_nv = (i < carrier_noise_var.size()) ? carrier_noise_var[i] : noise_variance;
        float nv = base_nv * ce_error_margin;
        const bool carrier_erased =
            i < carrier_erasure_flags_.size() && carrier_erasure_flags_[i] != 0;
        const bool erase_llrs = carrier_erased;

        // Per-carrier adaptive: inflate noise for carriers with unstable |eq|
        if (carrier_eq_mag_var_.size() == equalized.size()) {
            float mean_sq = carrier_eq_mag_ema_[i] * carrier_eq_mag_ema_[i] + 1e-6f;
            float norm_var = carrier_eq_mag_var_[i] / mean_sq;
            nv *= (1.0f + CARRIER_ADAPTIVE_K * norm_var);
        }

        const bool diag_track_llr_sigma =
            qam16FailureAttributionDiagEnabled() ||
            qam16GenieSigmaEmpiricalEnabled();
        if (mod == Modulation::QAM16) {
            if (qam16GenieSigmaEmpiricalEnabled() &&
                failure_diag_last_symbol_empirical_valid_) {
                // Diagnostic oracle: isolate LLR scaling by replacing the
                // demapper sigma^2 with the measured post-equalizer residual
                // variance for this symbol. This is never enabled unless the
                // CLI/env diagnostic hook requests it.
                nv = std::clamp(failure_diag_last_symbol_empirical_var_,
                                MIN_CARRIER_NOISE_VAR, MAX_CARRIER_NOISE_VAR);
            }
        }
        if (diag_track_llr_sigma) {
            failure_diag_llr_base_sigma2_sum_ += base_nv;
            failure_diag_llr_sigma2_sum_ += nv;
            ++failure_diag_llr_sigma2_count_;
        }

        if (erase_llrs) {
            appendErasureLLRs(soft_bits, mod);
            continue;
        }

        switch (mod) {
            case Modulation::BPSK:
                soft_bits.push_back(soft_demap::demapBPSK(sym, nv) * llr_sign);
                if (display_payload_carrier) {
                    constellation_update.push_back(sym);
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            case Modulation::QPSK: {
                auto llrs = soft_demap::demapQPSK(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                if (display_payload_carrier) {
                    constellation_update.push_back(sym);
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            }
            case Modulation::QAM8: {
                auto llrs = soft_demap::demap8PSK(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                if (display_payload_carrier) {
                    constellation_update.push_back(normalizePhaseOnlyForDisplay(sym));
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            }
            case Modulation::QAM16: {
                auto llrs = soft_demap::demapQAM16(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                if (display_payload_carrier) {
                    constellation_update.push_back(sym);
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            }
            case Modulation::QAM32: {
                auto llrs = soft_demap::demapQAM32(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                if (display_payload_carrier) {
                    constellation_update.push_back(sym);
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            }
            case Modulation::QAM64: {
                auto llrs = soft_demap::demapQAM64(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                if (display_payload_carrier) {
                    constellation_update.push_back(sym);
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            }
            case Modulation::QAM256: {
                auto llrs = soft_demap::demapQAM256(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                if (display_payload_carrier) {
                    constellation_update.push_back(sym);
                } else {
                    ++constellation_padding_skipped;
                }
                break;
            }
            default: {
                auto llrs = soft_demap::demapQPSK(sym, nv);
                for (auto& llr : llrs) llr *= llr_sign;
                soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
            }
        }
    }


    // Store constellation symbols (raw equalized; coherent-only OFDM)
    if (constellationDiagEnabled()) {
        static std::atomic<int> demod_log_count{0};
        if (demod_log_count < 120) {
            size_t erased = 0;
            for (uint8_t flag : carrier_erasure_flags_) {
                if (flag != 0) {
                    ++erased;
                }
            }
            const size_t weak_diff_refs = 0;  // coherent-only: no differential refs
            const auto eq_stats = constellationDiagStats(equalized);
            const auto update_stats = constellationDiagStats(constellation_update);
            LOG_DEMOD(INFO,
                      "CONSTELLATION_DIAG stage=demod mod=%s data_symbol=%zu "
                      "all_carriers=%zu data_carriers=%zu pilots=%zu "
                      "air_bits=%zu/%zu "
                      "equalized_count=%zu equalized_mag=[%.4f,%.4f,%.4f] "
                      "equalized_lt0p2=%zu/%zu erased=%zu "
                      "weak_diff_refs=%zu plotted_update_count=%zu "
                      "plot_skip_padding=%zu plot_skip_weak_diff=%zu "
                      "plotted_mag=[%.4f,%.4f,%.4f] plotted_lt0p2=%zu/%zu",
                      constellationDiagModName(mod),
                      current_data_symbol_index_,
                      all_carrier_fft_indices.size(),
                      data_carrier_indices.size(),
                      pilot_carrier_indices.size(),
                      std::min(constellation_air_bit_index_, constellation_valid_air_bits_),
                      constellation_valid_air_bits_,
                      eq_stats.count,
                      eq_stats.min_mag,
                      eq_stats.avg_mag,
                      eq_stats.max_mag,
                      eq_stats.below_0p2,
                      eq_stats.count,
                      erased,
                      weak_diff_refs,
                      update_stats.count,
                      constellation_padding_skipped,
                      constellation_weak_diff_skipped,
                      update_stats.min_mag,
                      update_stats.avg_mag,
                      update_stats.max_mag,
                      update_stats.below_0p2,
                      update_stats.count);
            ++demod_log_count;
        }
    }
    appendConstellationSymbols(constellation_update, mod);
}

// =============================================================================
// DEAD CODE REMOVED (thread A 2026-05-31): computeFadingIndex (callerless),
// demodulateD8PSKTwoPass, and demodulateDQPSKTwoPass were the differential
// two-pass / fading-index helpers. OFDM is coherent-only; they are gone.
// =============================================================================

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

} // namespace ultra
