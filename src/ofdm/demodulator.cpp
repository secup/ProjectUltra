#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>

// LDPC codeword size (same for all code rates in this implementation)
static constexpr size_t LDPC_BLOCK_SIZE = 648;

namespace ultra {

// Soft demapping - returns LLRs (log-likelihood ratios)
namespace {

float softDemapBPSK(Complex sym, float noise_var) {
    // BPSK: -1 maps to bit 0, +1 maps to bit 1
    // LLR convention: negative = bit 1, positive = bit 0
    // So: positive symbol → bit 1 → need negative LLR
    return -2.0f * sym.real() / noise_var;
}

std::vector<float> softDemapQPSK(Complex sym, float noise_var) {
    // QPSK: I and Q are independent BPSK channels
    // Modulator maps: bits[1,0] where bit1→I, bit0→Q
    // QPSK_MAP[0]=(-,-), [1]=(-,+), [2]=(+,-), [3]=(+,+)
    // So positive I/Q → bit 1 → need negative LLR
    float scale = -2.0f * 0.7071067811865476f / noise_var;
    return {sym.real() * scale, sym.imag() * scale};
}

std::vector<float> softDemapQAM16(Complex sym, float noise_var) {
    // 16-QAM soft demapping with correct LLR signs
    // Modulator Gray code: levels[] = {-3, -1, 3, 1} for bits 00,01,10,11
    //   MSB: negative → 0, positive → 1
    //   LSB: outer (|val|=3) → 0, inner (|val|=1) → 1
    // LLR convention: negative = bit 1, positive = bit 0
    constexpr float d = 0.6324555320336759f;  // 2/sqrt(10) - threshold between 1 and 3
    std::vector<float> llrs(4);

    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 3,2 of 4-bit word)
    llrs[0] = -scale * I;                      // bit3 (MSB): sign
    llrs[1] = scale * (std::abs(I) - d);       // bit2: outer→0, inner→1

    // Q bits (bits 1,0 of 4-bit word)
    llrs[2] = -scale * Q;                      // bit1 (MSB): sign
    llrs[3] = scale * (std::abs(Q) - d);       // bit0: outer→0, inner→1

    return llrs;
}

std::vector<float> softDemapQAM64(Complex sym, float noise_var) {
    // 64-QAM soft demapping with correct LLR signs
    // Modulator uses upper 3 bits for I, lower 3 bits for Q
    // Gray-coded levels: {-7,-5,-1,-3,7,5,1,3} / sqrt(42)
    // For this mapping:
    //   bit5/bit2 (MSB): negative I/Q → 0, positive I/Q → 1
    //   bit4/bit1: outer (|val| > d4) → 0, inner (|val| < d4) → 1
    //   bit3/bit0: depends on region, threshold at d2 from midpoint
    //
    // LLR convention: negative → bit 1, positive → bit 0
    constexpr float d2 = 0.3086067f;  // 2/sqrt(42)
    constexpr float d4 = 0.6172134f;  // 4/sqrt(42)
    std::vector<float> llrs(6);

    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 5,4,3 of 6-bit word)
    // bit5: I<0 → 0, I>0 → 1. LLR: I>0 should give negative LLR
    llrs[0] = -scale * I;
    // bit4: |I|>d4 → 0 (outer), |I|<d4 → 1 (inner). LLR: outer should give positive
    llrs[1] = scale * (std::abs(I) - d4);
    // bit3: distance from midpoint. Pattern gives 0 at extremes, 1 in middle of each region
    llrs[2] = scale * (std::abs(std::abs(I) - d4) - d2);

    // Q bits (bits 2,1,0 of 6-bit word) - same pattern
    llrs[3] = -scale * Q;
    llrs[4] = scale * (std::abs(Q) - d4);
    llrs[5] = scale * (std::abs(std::abs(Q) - d4) - d2);

    return llrs;
}

std::vector<float> softDemapQAM256(Complex sym, float noise_var) {
    // 256-QAM soft demapping with correct LLR signs
    // Same Gray code pattern as QAM64 but with 4 bits per axis
    // LLR convention: negative = bit 1, positive = bit 0
    constexpr float d2 = 0.1290994f;  // 2/sqrt(170)
    constexpr float d4 = 0.2581989f;  // 4/sqrt(170)
    constexpr float d8 = 0.5163978f;  // 8/sqrt(170)
    std::vector<float> llrs(8);

    float I = sym.real();
    float Q = sym.imag();
    float scale = 2.0f / noise_var;

    // I bits (bits 7,6,5,4 of 8-bit word)
    llrs[0] = -scale * I;                                               // bit 7: sign
    llrs[1] = scale * (std::abs(I) - d8);                               // bit 6: outer/inner
    llrs[2] = scale * (std::abs(std::abs(I) - d8) - d4);                // bit 5
    llrs[3] = scale * (std::abs(std::abs(std::abs(I) - d8) - d4) - d2); // bit 4

    // Q bits (bits 3,2,1,0 of 8-bit word)
    llrs[4] = -scale * Q;                                               // bit 3: sign
    llrs[5] = scale * (std::abs(Q) - d8);                               // bit 2: outer/inner
    llrs[6] = scale * (std::abs(std::abs(Q) - d8) - d4);                // bit 1
    llrs[7] = scale * (std::abs(std::abs(std::abs(Q) - d8) - d4) - d2); // bit 0

    return llrs;
}

} // anonymous namespace

struct OFDMDemodulator::Impl {
    ModemConfig config;
    FFT fft;
    NCO mixer;

    // Carrier indices (must match modulator)
    std::vector<int> data_carrier_indices;
    std::vector<int> pilot_carrier_indices;
    std::vector<Complex> pilot_sequence;

    // Synchronization state
    enum class State { SEARCHING, SYNCED };
    State state = State::SEARCHING;

    // Sample buffer
    Samples rx_buffer;
    size_t symbol_samples;

    // Channel estimate (per carrier)
    std::vector<Complex> channel_estimate;
    float noise_variance = 0.1f;

    // Output data
    Bytes demod_data;
    std::vector<float> soft_bits;
    ChannelQuality quality;

    // Sync detection
    std::vector<Complex> sync_sequence;
    float sync_threshold = 0.90f;  // Threshold for valid sync region

    // Pre-computed interpolation lookup (avoids O(n²) per symbol)
    struct InterpInfo {
        int fft_idx;        // FFT bin index of this data carrier
        int lower_pilot;    // FFT bin index of pilot below (-1 if none)
        int upper_pilot;    // FFT bin index of pilot above (-1 if none)
        float alpha;        // Interpolation weight: estimate = (1-alpha)*lower + alpha*upper
    };
    std::vector<InterpInfo> interp_table;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        symbol_samples = cfg.getSymbolDuration();
        channel_estimate.resize(cfg.fft_size, Complex(1, 0));

        setupCarriers();
        generateSequences();
        buildInterpTable();
    }

    void setupCarriers() {
        // Must match modulator exactly
        int half_carriers = config.num_carriers / 2;

        int pilot_count = 0;
        for (int i = -half_carriers; i <= half_carriers; ++i) {
            if (i == 0) continue;

            int fft_idx = (i + config.fft_size) % config.fft_size;

            if (pilot_count % config.pilot_spacing == 0) {
                pilot_carrier_indices.push_back(fft_idx);
            } else {
                data_carrier_indices.push_back(fft_idx);
            }
            ++pilot_count;
        }
    }

    void generateSequences() {
        // Same Zadoff-Chu sequence as modulator
        size_t N = config.num_carriers;
        size_t u = 1;

        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * u * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }

        // Same pilot sequence
        pilot_sequence.resize(pilot_carrier_indices.size());
        std::mt19937 rng(0xDEADBEEF);
        for (size_t i = 0; i < pilot_sequence.size(); ++i) {
            pilot_sequence[i] = (rng() & 1) ? Complex(1, 0) : Complex(-1, 0);
        }
    }

    void buildInterpTable() {
        // Pre-compute interpolation weights for each data carrier
        // This replaces the O(n²) computation that was done every symbol

        // Build ordered carrier list: -half to -1, then +1 to +half (skipping DC)
        struct CarrierInfo {
            int fft_idx;
            bool is_pilot;
        };
        std::vector<CarrierInfo> carriers;
        int half = config.num_carriers / 2;
        int pilot_count = 0;

        for (int i = -half; i <= half; ++i) {
            if (i == 0) continue;
            int fft_idx = (i + config.fft_size) % config.fft_size;
            bool is_pilot = (pilot_count % config.pilot_spacing == 0);
            carriers.push_back({fft_idx, is_pilot});
            ++pilot_count;
        }

        // For each data carrier, find surrounding pilots and compute weight
        interp_table.clear();
        interp_table.reserve(data_carrier_indices.size());

        for (size_t ci = 0; ci < carriers.size(); ++ci) {
            if (carriers[ci].is_pilot) continue;

            InterpInfo info;
            info.fft_idx = carriers[ci].fft_idx;
            info.lower_pilot = -1;
            info.upper_pilot = -1;
            info.alpha = 0.5f;

            // Find nearest pilot before (in carrier order)
            int lower_ci = -1;
            for (int j = (int)ci - 1; j >= 0; --j) {
                if (carriers[j].is_pilot) {
                    info.lower_pilot = carriers[j].fft_idx;
                    lower_ci = j;
                    break;
                }
            }

            // Find nearest pilot after (in carrier order)
            int upper_ci = -1;
            for (size_t j = ci + 1; j < carriers.size(); ++j) {
                if (carriers[j].is_pilot) {
                    info.upper_pilot = carriers[j].fft_idx;
                    upper_ci = (int)j;
                    break;
                }
            }

            // Compute interpolation weight
            if (lower_ci >= 0 && upper_ci >= 0) {
                float total_dist = (float)(upper_ci - lower_ci);
                info.alpha = (total_dist > 0) ? (float)((int)ci - lower_ci) / total_dist : 0.5f;
            }

            interp_table.push_back(info);
        }
    }

    float measureCorrelation(size_t offset, float* out_energy = nullptr) {
        // Cross-correlation with known preamble (repeated STS pattern)
        if (offset + symbol_samples * 2 > rx_buffer.size()) return 0.0f;

        // Look for repeated pattern (STS)
        float corr = 0;
        float energy = 0;
        size_t len = config.fft_size + config.getCyclicPrefix();

        for (size_t i = 0; i < len; ++i) {
            float s1 = rx_buffer[offset + i];
            float s2 = rx_buffer[offset + i + len];
            corr += s1 * s2;
            energy += s1 * s1 + s2 * s2;
        }

        if (out_energy) *out_energy = energy;
        return 2.0f * corr / (energy + 1e-10f);
    }

    bool detectSync(size_t offset) {
        float normalized = measureCorrelation(offset);

        // Debug: print correlation at key offsets
        if (offset % 500 == 0 || normalized > 0.5f) {
            LOG_DEMOD(TRACE, "detectSync offset=%zu normalized=%.3f threshold=%.2f",
                    offset, normalized, sync_threshold);
        }

        return normalized > sync_threshold;
    }

    std::vector<Complex> toBaseband(SampleSpan samples) {
        // Mix down from center frequency
        std::vector<Complex> baseband(samples.size());
        for (size_t i = 0; i < samples.size(); ++i) {
            Complex osc = mixer.next();
            // Conjugate for mixing down
            baseband[i] = samples[i] * std::conj(osc);
        }
        return baseband;
    }

    std::vector<Complex> extractSymbol(const std::vector<Complex>& baseband, size_t offset) {
        // Remove cyclic prefix
        size_t start = offset + config.getCyclicPrefix();

        // Copy FFT-sized chunk
        std::vector<Complex> symbol(config.fft_size);
        for (size_t i = 0; i < config.fft_size && (start + i) < baseband.size(); ++i) {
            symbol[i] = baseband[start + i];
        }

        // FFT to frequency domain
        std::vector<Complex> freq;
        fft.forward(symbol, freq);

        return freq;
    }

    void updateChannelEstimate(const std::vector<Complex>& freq_domain) {
        // Use pilots to estimate channel
        // H = Rx_pilot / Tx_pilot

        // Use more aggressive update (less smoothing) to converge quickly
        // For short frames, we need fast convergence
        float alpha = 0.9f;  // Weight for new estimate (was 0.3)

        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            Complex rx = freq_domain[idx];
            Complex tx = pilot_sequence[i];

            // LS estimate
            Complex h = rx / tx;

            // Smooth with previous estimate
            channel_estimate[idx] = alpha * h + (1.0f - alpha) * channel_estimate[idx];
        }

        // Interpolate between pilots for data carriers
        interpolateChannel();

        // Estimate noise variance from pilot error
        float error_sum = 0;
        for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
            int idx = pilot_carrier_indices[i];
            Complex expected = pilot_sequence[i] * channel_estimate[idx];
            Complex actual = freq_domain[idx];
            Complex error = actual - expected;
            error_sum += std::norm(error);
        }
        noise_variance = error_sum / pilot_carrier_indices.size();
        if (noise_variance < 1e-6f) noise_variance = 1e-6f;
    }

    void interpolateChannel() {
        // Interpolate channel estimate from pilots to data carriers
        // Uses pre-computed lookup table for O(n) instead of O(n²)
        for (const auto& info : interp_table) {
            if (info.lower_pilot >= 0 && info.upper_pilot >= 0) {
                // Linear interpolate between surrounding pilots
                channel_estimate[info.fft_idx] =
                    (1.0f - info.alpha) * channel_estimate[info.lower_pilot] +
                    info.alpha * channel_estimate[info.upper_pilot];
            } else if (info.lower_pilot >= 0) {
                channel_estimate[info.fft_idx] = channel_estimate[info.lower_pilot];
            } else if (info.upper_pilot >= 0) {
                channel_estimate[info.fft_idx] = channel_estimate[info.upper_pilot];
            }
            // If no pilots found, keep initial estimate (shouldn't happen with proper pilot spacing)
        }
    }

    std::vector<Complex> equalize(const std::vector<Complex>& freq_domain) {
        // Zero-forcing equalization: divide by channel estimate
        std::vector<Complex> equalized(data_carrier_indices.size());

        for (size_t i = 0; i < data_carrier_indices.size(); ++i) {
            int idx = data_carrier_indices[i];
            Complex h = channel_estimate[idx];

            // Avoid division by zero
            if (std::norm(h) < 1e-10f) {
                equalized[i] = Complex(0, 0);
            } else {
                equalized[i] = freq_domain[idx] / h;
            }
        }

        return equalized;
    }

    void demodulateSymbol(const std::vector<Complex>& equalized, Modulation mod) {
        // Log first few equalized symbols when starting a new frame
        if (soft_bits.empty() && !equalized.empty()) {
            LOG_DEMOD(DEBUG, "First equalized symbols: (%.3f,%.3f) (%.3f,%.3f) (%.3f,%.3f)",
                    equalized[0].real(), equalized[0].imag(),
                    equalized.size() > 1 ? equalized[1].real() : 0.0f,
                    equalized.size() > 1 ? equalized[1].imag() : 0.0f,
                    equalized.size() > 2 ? equalized[2].real() : 0.0f,
                    equalized.size() > 2 ? equalized[2].imag() : 0.0f);
        }

        // Convert symbols to soft bits (LLRs)
        for (const auto& sym : equalized) {
            switch (mod) {
                case Modulation::BPSK:
                    soft_bits.push_back(softDemapBPSK(sym, noise_variance));
                    break;
                case Modulation::QPSK: {
                    auto llrs = softDemapQPSK(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM16: {
                    auto llrs = softDemapQAM16(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM64: {
                    auto llrs = softDemapQAM64(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                case Modulation::QAM256: {
                    auto llrs = softDemapQAM256(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                    break;
                }
                default: {
                    auto llrs = softDemapQPSK(sym, noise_variance);
                    soft_bits.insert(soft_bits.end(), llrs.begin(), llrs.end());
                }
            }
        }
    }

    void updateQuality() {
        // Calculate channel quality metrics
        float total_snr = 0;
        for (int idx : data_carrier_indices) {
            float signal_power = std::norm(channel_estimate[idx]);
            float snr = signal_power / noise_variance;
            total_snr += snr;
        }
        quality.snr_db = 10.0f * std::log10(total_snr / data_carrier_indices.size());

        // Simplified Doppler estimate (would need multiple symbols for real estimate)
        quality.doppler_hz = 0;

        // Estimate delay spread from channel variation
        quality.delay_spread_ms = 0;

        // BER estimate from SNR (approximate)
        if (quality.snr_db > 10) {
            quality.ber_estimate = 1e-6f;
        } else if (quality.snr_db > 5) {
            quality.ber_estimate = 1e-4f;
        } else {
            quality.ber_estimate = 1e-2f;
        }
    }
};

OFDMDemodulator::OFDMDemodulator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OFDMDemodulator::~OFDMDemodulator() = default;

bool OFDMDemodulator::process(SampleSpan samples) {
    // Add to buffer
    impl_->rx_buffer.insert(impl_->rx_buffer.end(), samples.begin(), samples.end());

    LOG_DEMOD(TRACE, "Buffer has %zu samples, state=%s",
            impl_->rx_buffer.size(),
            impl_->state == Impl::State::SEARCHING ? "SEARCHING" : "SYNCED");

    // State machine
    if (impl_->state == Impl::State::SEARCHING) {
        // Look for sync preamble
        // Preamble symbols have NO guard interval (only fft_size + cp)
        size_t preamble_symbol_len = impl_->config.fft_size + impl_->config.getCyclicPrefix();
        size_t preamble_total_len = preamble_symbol_len * 6;  // 4 STS + 2 LTS

        LOG_DEMOD(TRACE, "Need %zu samples for sync, have %zu",
                preamble_total_len, impl_->rx_buffer.size());

        // Sync detection strategy:
        // 1. Find region with high correlation (indicates STS pattern)
        // 2. Scan backwards from high-correlation point to find signal start via energy detection
        float max_corr = 0;
        size_t max_offset = 0;

        // Find first point with high correlation
        for (size_t i = 0; i + preamble_total_len < impl_->rx_buffer.size(); ++i) {
            float corr = impl_->measureCorrelation(i);
            if (corr > max_corr) {
                max_corr = corr;
                max_offset = i;
            }
            // Stop once we've found a clear high-correlation region
            if (max_corr > 0.9f && corr < max_corr - 0.15f) {
                break;
            }
        }

        // Refine sync using energy detection: scan backwards to find signal start
        size_t sync_offset = max_offset;
        bool found_sync = max_corr > impl_->sync_threshold;

        if (found_sync && max_offset > 100) {
            // Scan backwards from correlation peak to find silence-to-signal edge
            size_t window = 64;

            // Measure signal energy at correlation peak
            float signal_energy = 0;
            for (size_t i = 0; i < window && max_offset + i < impl_->rx_buffer.size(); ++i) {
                float s = impl_->rx_buffer[max_offset + i];
                signal_energy += s * s;
            }
            signal_energy /= window;

            // Scan backwards to find where energy first drops below 10% of signal
            float threshold = signal_energy * 0.10f;

            for (size_t back = 0; back < max_offset && back < 3000; ++back) {
                size_t pos = max_offset - back;
                float energy = 0;
                for (size_t i = 0; i < window && pos + i < impl_->rx_buffer.size(); ++i) {
                    float s = impl_->rx_buffer[pos + i];
                    energy += s * s;
                }
                energy /= window;

                if (energy < threshold) {
                    // Found silence-to-signal transition
                    // The signal starts roughly at pos + window/2
                    // Subtract 4 samples for fine timing alignment
                    sync_offset = pos + window / 2 - 4;
                    break;
                }
            }
        }

        if (found_sync) {
            LOG_SYNC(INFO, "SYNC FOUND: offset=%zu (corr_peak=%zu, corr=%.3f)",
                    sync_offset, max_offset, max_corr);
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + sync_offset + preamble_total_len);
            impl_->state = Impl::State::SYNCED;
            impl_->mixer.reset();
        } else {
            LOG_SYNC(TRACE, "No sync found (max_corr=%.3f)", max_corr);
        }
    }

    if (impl_->state == Impl::State::SYNCED) {
        // Process complete symbols (stop when we have enough for one LDPC block)
        while (impl_->rx_buffer.size() >= impl_->symbol_samples &&
               impl_->soft_bits.size() < LDPC_BLOCK_SIZE) {
            // Convert to baseband
            SampleSpan sym_samples(impl_->rx_buffer.data(), impl_->symbol_samples);
            auto baseband = impl_->toBaseband(sym_samples);

            // Extract and FFT
            auto freq_domain = impl_->extractSymbol(baseband, 0);

            // Update channel estimate from pilots
            impl_->updateChannelEstimate(freq_domain);

            // Equalize
            auto equalized = impl_->equalize(freq_domain);

            // Demodulate (using config's modulation setting)
            impl_->demodulateSymbol(equalized, impl_->config.modulation);

            // Remove processed samples
            impl_->rx_buffer.erase(impl_->rx_buffer.begin(),
                                   impl_->rx_buffer.begin() + impl_->symbol_samples);

            impl_->updateQuality();
        }

        // Return true if we have enough soft bits for an LDPC codeword
        return impl_->soft_bits.size() >= LDPC_BLOCK_SIZE;
    }

    return false;
}

Bytes OFDMDemodulator::getData() {
    // Convert soft bits to hard bits, then to bytes
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
    // Debug: print first 24 LLRs and their hard decisions
    if (g_log_level >= LogLevel::DEBUG && g_log_categories.demod && impl_->soft_bits.size() >= 24) {
        char buf[256];
        int pos = 0;
        for (size_t i = 0; i < 24 && pos < 240; ++i) {
            int bit = (impl_->soft_bits[i] < 0) ? 1 : 0;  // LDPC convention
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%+.1f(%d) ", impl_->soft_bits[i], bit);
            if ((i + 1) % 6 == 0) pos += snprintf(buf + pos, sizeof(buf) - pos, "| ");
        }
        LOG_DEMOD(DEBUG, "First 24 LLRs: %s", buf);
    }

    // Return exactly one LDPC block worth of soft bits
    if (impl_->soft_bits.size() <= LDPC_BLOCK_SIZE) {
        auto bits = std::move(impl_->soft_bits);
        impl_->soft_bits.clear();
        return bits;
    } else {
        // Return first block, keep remainder
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

void OFDMDemodulator::reset() {
    impl_->state = Impl::State::SEARCHING;
    impl_->rx_buffer.clear();
    impl_->soft_bits.clear();
    impl_->demod_data.clear();
    std::fill(impl_->channel_estimate.begin(), impl_->channel_estimate.end(), Complex(1, 0));
}

// ============ Channel Estimator ============

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
    // Simple linear interpolation - would be enhanced with MMSE
}

} // namespace ultra
