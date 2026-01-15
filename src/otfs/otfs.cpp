#include "ultra/otfs.hpp"
#include "ultra/dsp.hpp"
#include <cmath>
#include <random>
#include <stdexcept>
#include <algorithm>

namespace ultra {

// ============================================================================
// ISFFT and SFFT implementations
// ============================================================================

// Simple 1D FFT (in-place, radix-2 Cooley-Tukey)
static void fft1d(std::vector<Complex>& x, bool inverse, bool scale_inverse = true) {
    size_t N = x.size();
    if (N <= 1) return;

    // Bit-reversal permutation
    for (size_t i = 1, j = 0; i < N; ++i) {
        size_t bit = N >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }

    // Cooley-Tukey iterative FFT
    for (size_t len = 2; len <= N; len *= 2) {
        float angle = 2 * M_PI / len * (inverse ? 1 : -1);
        Complex wlen(std::cos(angle), std::sin(angle));
        for (size_t i = 0; i < N; i += len) {
            Complex w(1, 0);
            for (size_t j = 0; j < len / 2; ++j) {
                Complex u = x[i + j];
                Complex v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }

    if (inverse && scale_inverse) {
        for (auto& val : x) {
            val /= static_cast<float>(N);
        }
    }
}

// ISFFT: Delay-Doppler → Time-Frequency
void isfft(const std::vector<Complex>& dd_grid,
           std::vector<Complex>& tf_grid,
           uint32_t M, uint32_t N) {
    if (dd_grid.size() != M * N) {
        throw std::invalid_argument("DD grid size mismatch");
    }

    tf_grid.resize(M * N);
    std::vector<Complex> temp(M * N);

    // Step 1: IFFT along columns (Doppler → time)
    for (uint32_t k = 0; k < M; ++k) {
        std::vector<Complex> col(N);
        for (uint32_t l = 0; l < N; ++l) {
            col[l] = dd_grid[k * N + l];
        }
        fft1d(col, true, false);
        for (uint32_t n = 0; n < N; ++n) {
            temp[k * N + n] = col[n];
        }
    }

    // Step 2: FFT along rows (delay → frequency)
    for (uint32_t n = 0; n < N; ++n) {
        std::vector<Complex> row(M);
        for (uint32_t k = 0; k < M; ++k) {
            row[k] = temp[k * N + n];
        }
        fft1d(row, false, false);
        for (uint32_t m = 0; m < M; ++m) {
            tf_grid[n * M + m] = row[m];
        }
    }
}

// SFFT: Time-Frequency → Delay-Doppler
void sfft(const std::vector<Complex>& tf_grid,
          std::vector<Complex>& dd_grid,
          uint32_t M, uint32_t N) {
    if (tf_grid.size() != M * N) {
        throw std::invalid_argument("TF grid size mismatch");
    }

    dd_grid.resize(M * N);
    std::vector<Complex> temp(M * N);

    // Step 1: FFT along columns (time → Doppler)
    for (uint32_t m = 0; m < M; ++m) {
        std::vector<Complex> col(N);
        for (uint32_t n = 0; n < N; ++n) {
            col[n] = tf_grid[n * M + m];
        }
        fft1d(col, false, false);
        for (uint32_t l = 0; l < N; ++l) {
            temp[m * N + l] = col[l];
        }
    }

    // Step 2: IFFT along rows (frequency → delay)
    for (uint32_t l = 0; l < N; ++l) {
        std::vector<Complex> row(M);
        for (uint32_t m = 0; m < M; ++m) {
            row[m] = temp[m * N + l];
        }
        fft1d(row, true, false);
        for (uint32_t k = 0; k < M; ++k) {
            dd_grid[k * N + l] = row[k];
        }
    }

    // Scale by 1/MN for roundtrip identity
    float scale = 1.0f / static_cast<float>(M * N);
    for (auto& val : dd_grid) {
        val *= scale;
    }
}

// ============================================================================
// Constellation mapping
// ============================================================================

namespace {

constexpr float QPSK_SCALE = 0.7071067811865476f;  // 1/sqrt(2)
const Complex QPSK_MAP[] = {
    Complex(-QPSK_SCALE, -QPSK_SCALE),  // 00
    Complex(-QPSK_SCALE,  QPSK_SCALE),  // 01
    Complex( QPSK_SCALE, -QPSK_SCALE),  // 10
    Complex( QPSK_SCALE,  QPSK_SCALE),  // 11
};

// Known pilot symbol for channel estimation
const Complex PILOT_SYMBOL = Complex(1.0f, 0.0f);

// Demodulator constants
constexpr float REAL_TO_COMPLEX_SCALE = 2.4f;  // Compensates for single-sideband extraction
constexpr float PREAMBLE_TARGET_RMS = 0.1f;

Complex mapBits(uint32_t bits, Modulation mod) {
    switch (mod) {
        case Modulation::BPSK:
            return (bits & 1) ? Complex(1, 0) : Complex(-1, 0);
        case Modulation::QPSK:
            return QPSK_MAP[bits & 3];
        case Modulation::QAM16: {
            static const float levels[] = {-3, -1, 3, 1};
            static const float scale = 0.3162277660168379f;
            int i_bits = (bits >> 2) & 0x3;
            int q_bits = bits & 0x3;
            return Complex(levels[i_bits] * scale, levels[q_bits] * scale);
        }
        default:
            return QPSK_MAP[bits & 3];
    }
}

// Soft demapping with proper LLR scaling
// LLR convention: negative = bit 1, positive = bit 0
void softDemap(const Complex& symbol, Modulation mod, float noise_var,
               std::vector<float>& llrs) {
    // Clamp noise variance to prevent numerical issues
    noise_var = std::max(0.001f, noise_var);

    switch (mod) {
        case Modulation::BPSK: {
            llrs.push_back(-2.0f * symbol.real() / noise_var);
            break;
        }
        case Modulation::QPSK: {
            float scale = -2.0f * QPSK_SCALE / noise_var;
            llrs.push_back(symbol.real() * scale);
            llrs.push_back(symbol.imag() * scale);
            break;
        }
        default: {
            float scale = -2.0f / noise_var;
            llrs.push_back(symbol.real() * scale);
            llrs.push_back(symbol.imag() * scale);
            break;
        }
    }
}

} // anonymous namespace

// ============================================================================
// OTFSModulator implementation
// ============================================================================

struct OTFSModulator::Impl {
    OTFSConfig config;
    FFT fft;
    NCO mixer;
    std::vector<Complex> sync_sequence;

    Impl(const OTFSConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        generateSyncSequence();
    }

    void generateSyncSequence() {
        size_t N = config.M;
        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }
    }

    std::vector<Complex> createOFDMSymbol(const std::vector<Complex>& freq_data) {
        std::vector<Complex> freq_domain(config.fft_size, Complex(0, 0));

        // Map to bins 1..M (skip DC, positive frequencies only)
        for (size_t i = 0; i < config.M && i < freq_data.size(); ++i) {
            size_t idx = i + 1;
            if (idx < config.fft_size / 2) {
                freq_domain[idx] = freq_data[i];
            }
        }

        std::vector<Complex> time_domain;
        fft.inverse(freq_domain, time_domain);

        // Add cyclic prefix
        std::vector<Complex> with_cp;
        with_cp.reserve(config.fft_size + config.cp_length);
        for (size_t i = config.fft_size - config.cp_length; i < config.fft_size; ++i) {
            with_cp.push_back(time_domain[i]);
        }
        for (const auto& s : time_domain) {
            with_cp.push_back(s);
        }

        return with_cp;
    }

    Samples complexToReal(const std::vector<Complex>& signal) {
        Samples real(signal.size());
        for (size_t i = 0; i < signal.size(); ++i) {
            Complex mixed = signal[i] * mixer.next();
            real[i] = mixed.real();
        }
        return real;
    }
};

OTFSModulator::OTFSModulator(const OTFSConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OTFSModulator::~OTFSModulator() = default;

size_t OTFSModulator::symbolsPerFrame() const {
    return impl_->config.total_data_symbols();
}

size_t OTFSModulator::bitsPerFrame(Modulation mod) const {
    return symbolsPerFrame() * static_cast<size_t>(mod);
}

std::vector<Complex> OTFSModulator::mapToDD(ByteSpan data, Modulation mod) {
    const auto& cfg = impl_->config;
    size_t bits_per_symbol = static_cast<size_t>(mod);

    // Create DD grid (data only - pilots added in TF domain)
    std::vector<Complex> dd_grid(cfg.M * cfg.N, Complex(0, 0));

    size_t data_idx = 0;
    size_t bit_idx = 0;
    size_t symbol_count = 0;

    for (uint32_t k = 0; k < cfg.M; ++k) {
        for (uint32_t l = 0; l < cfg.N; ++l) {
            size_t grid_idx = k * cfg.N + l;

            if (data_idx < data.size()) {
                uint32_t bits = 0;
                for (size_t b = 0; b < bits_per_symbol; ++b) {
                    if (data_idx < data.size()) {
                        uint8_t byte = data[data_idx];
                        uint8_t bit = (byte >> (7 - bit_idx)) & 1;
                        bits = (bits << 1) | bit;
                        ++bit_idx;
                        if (bit_idx >= 8) {
                            bit_idx = 0;
                            ++data_idx;
                        }
                    }
                }
                dd_grid[grid_idx] = mapBits(bits, mod);
                symbol_count++;
            }
        }
    }

    return dd_grid;
}

Samples OTFSModulator::modulate(const std::vector<Complex>& dd_symbols, Modulation mod) {
    const auto& cfg = impl_->config;

    // Step 1: ISFFT (DD → TF)
    std::vector<Complex> tf_grid;
    isfft(dd_symbols, tf_grid, cfg.M, cfg.N);

    // Step 2: OFDM modulate each TF symbol
    // Channel estimation comes from preamble (no TF pilots needed)
    Samples output;
    impl_->mixer.reset();

    for (uint32_t n = 0; n < cfg.N; ++n) {
        std::vector<Complex> freq_data(cfg.M);

        for (uint32_t m = 0; m < cfg.M; ++m) {
            freq_data[m] = tf_grid[n * cfg.M + m];
        }

        auto ofdm_symbol = impl_->createOFDMSymbol(freq_data);
        auto real_symbol = impl_->complexToReal(ofdm_symbol);
        output.insert(output.end(), real_symbol.begin(), real_symbol.end());
    }

    return output;
}

Samples OTFSModulator::generatePreamble() {
    impl_->mixer.reset();

    Samples preamble;
    auto sync_symbol = impl_->createOFDMSymbol(impl_->sync_sequence);
    auto sync_real = impl_->complexToReal(sync_symbol);

    // Normalize preamble
    float preamble_rms = 0;
    for (float s : sync_real) preamble_rms += s * s;
    preamble_rms = std::sqrt(preamble_rms / sync_real.size());
    if (preamble_rms > 0) {
        float scale = PREAMBLE_TARGET_RMS / preamble_rms;
        for (float& s : sync_real) s *= scale;
    }

    // Repeat 4 times for robust detection
    for (int i = 0; i < 4; ++i) {
        preamble.insert(preamble.end(), sync_real.begin(), sync_real.end());
    }

    return preamble;
}

// ============================================================================
// OTFSDemodulator implementation
// ============================================================================

struct OTFSDemodulator::Impl {
    OTFSConfig config;
    FFT fft;
    NCO mixer;

    enum class State { SEARCHING, SYNCED, FRAME_READY };
    State state = State::SEARCHING;

    std::vector<float> sample_buffer;
    std::vector<Complex> tf_buffer;      // Raw TF grid from OFDM demod
    std::vector<Complex> tf_equalized;   // Equalized TF grid
    std::vector<Complex> channel_est;    // Per-subcarrier channel estimate from preamble
    bool channel_estimated = false;
    uint32_t symbols_received = 0;

    size_t total_samples_processed = 0;
    size_t current_frame_start = 0;

    std::vector<Complex> dd_symbols;
    std::vector<float> soft_bits;
    float estimated_noise_var = 0.1f;

    std::vector<Complex> sync_sequence;
    float sync_threshold = 0.7f;

    Impl(const OTFSConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        generateSyncSequence();
        tf_buffer.resize(cfg.M * cfg.N);
        tf_equalized.resize(cfg.M * cfg.N);
        channel_est.resize(cfg.M, Complex(1, 0));  // Initialize to unity
    }

    void generateSyncSequence() {
        size_t N = config.M;
        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }
    }

    std::vector<Complex> toBaseband(const float* samples, size_t count, size_t sample_offset = 0) {
        std::vector<Complex> baseband(count);
        NCO temp_mixer(config.center_freq, config.sample_rate);
        for (size_t i = 0; i < sample_offset; ++i) temp_mixer.next();

        for (size_t i = 0; i < count; ++i) {
            Complex carrier = temp_mixer.next();
            baseband[i] = Complex(samples[i], 0) * std::conj(carrier);
        }
        return baseband;
    }

    bool detectSyncReal(const float* samples, size_t count) {
        size_t sym_len = config.fft_size + config.cp_length;
        if (count < 2 * sym_len) return false;

        float P = 0, R = 0;
        for (size_t i = 0; i < sym_len; ++i) {
            P += samples[i] * samples[i + sym_len];
            R += samples[i + sym_len] * samples[i + sym_len];
        }

        float metric = std::abs(P) / (R + 1e-10f);
        return metric > sync_threshold;
    }

    void demodulateSymbol(const std::vector<Complex>& baseband, uint32_t symbol_idx) {
        size_t sym_start = config.cp_length;
        size_t sym_len = config.fft_size;

        std::vector<Complex> time_domain(baseband.begin() + sym_start,
                                          baseband.begin() + sym_start + sym_len);

        std::vector<Complex> freq_domain;
        fft.forward(time_domain, freq_domain);

        // Extract subcarriers and store in TF buffer
        for (size_t m = 0; m < config.M; ++m) {
            size_t idx = m + 1;  // Skip DC
            if (idx < config.fft_size / 2) {
                tf_buffer[symbol_idx * config.M + m] = freq_domain[idx] * REAL_TO_COMPLEX_SCALE;
            } else {
                tf_buffer[symbol_idx * config.M + m] = Complex(0, 0);
            }
        }
    }

    // Estimate channel from preamble (known Zadoff-Chu sequence)
    void estimateChannelFromPreamble(const float* preamble_samples, size_t preamble_len, size_t preamble_start) {
        size_t sym_len = config.fft_size + config.cp_length;

        if (preamble_len < 4 * sym_len) return;

        // Average channel estimates from all 4 preamble symbols for robustness
        std::vector<Complex> h_sum(config.M, Complex(0, 0));
        float total_noise_power = 0;
        int noise_samples = 0;

        for (int sym = 0; sym < 4; ++sym) {
            size_t sym_offset = sym * sym_len;
            auto baseband = toBaseband(preamble_samples + sym_offset, sym_len, preamble_start + sym_offset);

            std::vector<Complex> time_domain(baseband.begin() + config.cp_length,
                                              baseband.begin() + config.cp_length + config.fft_size);

            std::vector<Complex> freq_domain;
            fft.forward(time_domain, freq_domain);

            for (size_t m = 0; m < config.M; ++m) {
                size_t idx = m + 1;
                if (idx < config.fft_size / 2) {
                    Complex received = freq_domain[idx] * REAL_TO_COMPLEX_SCALE;
                    Complex expected = sync_sequence[m];

                    float expected_mag_sq = std::norm(expected);
                    if (expected_mag_sq > 0.01f) {
                        Complex h = received * std::conj(expected) / expected_mag_sq;
                        h_sum[m] += h;

                        if (sym == 3) {
                            Complex error = received - h * expected;
                            total_noise_power += std::norm(error);
                            noise_samples++;
                        }
                    }
                }
            }
        }

        // Average the estimates
        channel_est.resize(config.M);
        for (size_t m = 0; m < config.M; ++m) {
            channel_est[m] = h_sum[m] / 4.0f;
            if (std::norm(channel_est[m]) < 0.01f) {
                channel_est[m] = Complex(1, 0);
            }
        }

        // Update noise variance
        if (noise_samples > 0) {
            estimated_noise_var = total_noise_power / noise_samples;
            estimated_noise_var = std::max(0.001f, std::min(1.0f, estimated_noise_var));
        }

        channel_estimated = true;
    }

    // Equalize TF grid using preamble-based channel estimate
    void equalizeTFGrid() {
        if (!channel_estimated) {
            tf_equalized = tf_buffer;
            return;
        }

        // Apply channel estimate to all OFDM symbols
        for (uint32_t n = 0; n < config.N; ++n) {
            for (uint32_t m = 0; m < config.M; ++m) {
                Complex received = tf_buffer[n * config.M + m];
                Complex h = channel_est[m];

                // Zero-forcing equalization
                float h_mag_sq = std::norm(h);
                if (h_mag_sq > 0.01f) {
                    tf_equalized[n * config.M + m] = received * std::conj(h) / h_mag_sq;
                } else {
                    tf_equalized[n * config.M + m] = received;
                }
            }
        }
    }
};

OTFSDemodulator::OTFSDemodulator(const OTFSConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OTFSDemodulator::~OTFSDemodulator() = default;

bool OTFSDemodulator::process(SampleSpan samples) {
    impl_->sample_buffer.insert(impl_->sample_buffer.end(),
                                samples.data(), samples.data() + samples.size());

    size_t sym_len = impl_->config.fft_size + impl_->config.cp_length;
    size_t preamble_len = 4 * sym_len;

    while (true) {
        if (impl_->state == Impl::State::SEARCHING) {
            if (impl_->sample_buffer.size() < 2 * sym_len) return false;

            if (impl_->detectSyncReal(impl_->sample_buffer.data(), 2 * sym_len)) {
                if (impl_->sample_buffer.size() >= preamble_len) {
                    // Estimate channel from preamble BEFORE removing it
                    // Pass absolute sample position for proper mixer phase alignment
                    if (impl_->config.tf_equalization) {
                        impl_->estimateChannelFromPreamble(
                            impl_->sample_buffer.data(), preamble_len, impl_->total_samples_processed);
                    }

                    impl_->total_samples_processed += preamble_len;
                    impl_->current_frame_start = impl_->total_samples_processed;
                    impl_->sample_buffer.erase(impl_->sample_buffer.begin(),
                                               impl_->sample_buffer.begin() + preamble_len);
                    impl_->state = Impl::State::SYNCED;
                    impl_->symbols_received = 0;
                } else {
                    return false;
                }
            } else {
                size_t slide = sym_len / 4;
                if (impl_->sample_buffer.size() > slide) {
                    impl_->total_samples_processed += slide;
                    impl_->sample_buffer.erase(impl_->sample_buffer.begin(),
                                               impl_->sample_buffer.begin() + slide);
                } else {
                    return false;
                }
            }
        }
        else if (impl_->state == Impl::State::SYNCED) {
            if (impl_->sample_buffer.size() < sym_len) return false;

            size_t symbol_offset = impl_->current_frame_start +
                                   impl_->symbols_received * sym_len;
            auto baseband = impl_->toBaseband(impl_->sample_buffer.data(), sym_len,
                                               symbol_offset);
            impl_->demodulateSymbol(baseband, impl_->symbols_received);
            impl_->symbols_received++;

            impl_->sample_buffer.erase(impl_->sample_buffer.begin(),
                                       impl_->sample_buffer.begin() + sym_len);

            if (impl_->symbols_received >= impl_->config.N) {
                impl_->state = Impl::State::FRAME_READY;
            }
        }
        else if (impl_->state == Impl::State::FRAME_READY) {
            // Step 1: TF equalization (user configurable)
            // tf_equalization=true: Better for stable channels (Good), uses preamble estimate
            // tf_equalization=false: Better for challenging channels (Poor), leverages OTFS diversity
            bool use_tf_eq = impl_->config.tf_equalization && impl_->channel_estimated;

            if (use_tf_eq) {
                impl_->equalizeTFGrid();
            } else {
                impl_->tf_equalized = impl_->tf_buffer;
            }

            // Step 2: SFFT to get DD symbols
            sfft(impl_->tf_equalized, impl_->dd_symbols, impl_->config.M, impl_->config.N);

            // Step 3: Normalize DD symbols (critical for soft demapping without TF eq)
            // OTFS spreads channel across DD grid; we need to normalize for proper LLR
            float avg_power = 0;
            for (const auto& sym : impl_->dd_symbols) {
                avg_power += std::norm(sym);
            }
            avg_power /= impl_->dd_symbols.size();

            if (avg_power > 0.01f) {
                // Target power for QPSK is 1.0 (each symbol has mag ~0.707)
                float scale = 1.0f / std::sqrt(avg_power);
                for (auto& sym : impl_->dd_symbols) {
                    sym *= scale;
                }
                // Adjust noise variance estimate based on scaling
                impl_->estimated_noise_var = 0.1f;  // Reset to reasonable value after normalization
            }

            // Step 4: Generate soft bits from DD symbols
            impl_->soft_bits.clear();
            for (const auto& sym : impl_->dd_symbols) {
                softDemap(sym, impl_->config.modulation, impl_->estimated_noise_var, impl_->soft_bits);
            }

            impl_->state = Impl::State::SEARCHING;
            impl_->symbols_received = 0;
            impl_->channel_estimated = false;  // Reset for next frame
            return true;
        }
    }

    return false;
}

std::vector<Complex> OTFSDemodulator::getDDSymbols() {
    return impl_->dd_symbols;
}

std::vector<float> OTFSDemodulator::getSoftBits() {
    return impl_->soft_bits;
}

std::vector<Complex> OTFSDemodulator::getDDChannel() {
    // Return channel estimate (currently in TF domain)
    return impl_->channel_est;
}

void OTFSDemodulator::reset() {
    impl_->state = Impl::State::SEARCHING;
    impl_->sample_buffer.clear();
    impl_->symbols_received = 0;
    impl_->total_samples_processed = 0;
    impl_->current_frame_start = 0;
    impl_->dd_symbols.clear();
    impl_->soft_bits.clear();
    impl_->estimated_noise_var = 0.1f;
    impl_->channel_estimated = false;
    impl_->mixer.reset();
}

bool OTFSDemodulator::isSynced() const {
    return impl_->state == Impl::State::SYNCED;
}

} // namespace ultra
