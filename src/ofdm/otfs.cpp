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
// scale_inverse: if true, IFFT scales by 1/N (standard). If false, no scaling.
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

    // Scale for inverse FFT (only if requested)
    if (inverse && scale_inverse) {
        for (auto& val : x) {
            val /= static_cast<float>(N);
        }
    }
}

// ISFFT: Delay-Doppler → Time-Frequency
// Converts DD grid to TF grid for OFDM modulation
// Uses unscaled transforms; scaling handled by SFFT for proper roundtrip
void isfft(const std::vector<Complex>& dd_grid,
           std::vector<Complex>& tf_grid,
           uint32_t M, uint32_t N) {
    if (dd_grid.size() != M * N) {
        throw std::invalid_argument("DD grid size mismatch");
    }

    tf_grid.resize(M * N);

    // Work buffer
    std::vector<Complex> temp(M * N);

    // Step 1: IFFT along columns (Doppler dimension, index l → n)
    // Use unscaled IFFT for symplectic transform
    for (uint32_t k = 0; k < M; ++k) {
        std::vector<Complex> col(N);
        for (uint32_t l = 0; l < N; ++l) {
            col[l] = dd_grid[k * N + l];
        }
        fft1d(col, true, false);  // IFFT, no scaling
        for (uint32_t n = 0; n < N; ++n) {
            temp[k * N + n] = col[n];
        }
    }

    // Step 2: FFT along rows (delay dimension, index k → m)
    for (uint32_t n = 0; n < N; ++n) {
        std::vector<Complex> row(M);
        for (uint32_t k = 0; k < M; ++k) {
            row[k] = temp[k * N + n];
        }
        fft1d(row, false, false);  // FFT, no scaling
        for (uint32_t m = 0; m < M; ++m) {
            tf_grid[n * M + m] = row[m];
        }
    }

    // No scaling here - SFFT will handle the 1/MN scaling for roundtrip
}

// SFFT: Time-Frequency → Delay-Doppler
// Inverse of ISFFT - converts TF grid back to DD grid
// Includes 1/MN scaling to ensure SFFT(ISFFT(x)) = x
void sfft(const std::vector<Complex>& tf_grid,
          std::vector<Complex>& dd_grid,
          uint32_t M, uint32_t N) {
    if (tf_grid.size() != M * N) {
        throw std::invalid_argument("TF grid size mismatch");
    }

    dd_grid.resize(M * N);

    // Work buffer
    std::vector<Complex> temp(M * N);

    // Step 1: FFT along columns (time dimension, index n → l)
    for (uint32_t m = 0; m < M; ++m) {
        std::vector<Complex> col(N);
        for (uint32_t n = 0; n < N; ++n) {
            col[n] = tf_grid[n * M + m];
        }
        fft1d(col, false, false);  // FFT, no scaling
        for (uint32_t l = 0; l < N; ++l) {
            temp[m * N + l] = col[l];
        }
    }

    // Step 2: IFFT along rows (frequency dimension, index m → k)
    for (uint32_t l = 0; l < N; ++l) {
        std::vector<Complex> row(M);
        for (uint32_t m = 0; m < M; ++m) {
            row[m] = temp[m * N + l];
        }
        fft1d(row, true, false);  // IFFT, no scaling
        for (uint32_t k = 0; k < M; ++k) {
            dd_grid[k * N + l] = row[k];
        }
    }

    // Scale by 1/MN to ensure SFFT(ISFFT(x)) = x
    // ISFFT does: IFFT_col * FFT_row (unscaled, so multiplies by MN)
    // SFFT does: FFT_col * IFFT_row (unscaled, so multiplies by MN)
    // Combined unscaled: MN * MN = M²N²
    // We need to divide by M²N² to get identity, so scale by 1/(MN)
    float scale = 1.0f / static_cast<float>(M * N);
    for (auto& val : dd_grid) {
        val *= scale;
    }
}

// ============================================================================
// Constellation mapping
// ============================================================================

namespace {

constexpr float QPSK_SCALE = 0.7071067811865476f;
const Complex QPSK_MAP[] = {
    Complex(-QPSK_SCALE, -QPSK_SCALE),
    Complex(-QPSK_SCALE,  QPSK_SCALE),
    Complex( QPSK_SCALE, -QPSK_SCALE),
    Complex( QPSK_SCALE,  QPSK_SCALE),
};

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

// Soft demapping (LLR calculation)
// LLR convention: negative = bit 1, positive = bit 0 (matches LDPC decoder)
void softDemap(const Complex& symbol, Modulation mod, float noise_var,
               std::vector<float>& llrs) {
    switch (mod) {
        case Modulation::BPSK: {
            // BPSK: -1 maps to bit 0, +1 maps to bit 1
            // LLR: positive symbol → bit 1 → need negative LLR
            llrs.push_back(-2.0f * symbol.real() / noise_var);
            break;
        }
        case Modulation::QPSK: {
            // QPSK_MAP[0]=(-,-) bits 00, [1]=(-,+) bits 01, [2]=(+,-) bits 10, [3]=(+,+) bits 11
            // Positive I/Q → bit 1 → need negative LLR
            float scale = -2.0f * QPSK_SCALE / noise_var;
            llrs.push_back(symbol.real() * scale);
            llrs.push_back(symbol.imag() * scale);
            break;
        }
        default: {
            // Same convention for higher order
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

    // Preamble sequence
    std::vector<Complex> sync_sequence;

    Impl(const OTFSConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        generateSyncSequence();
    }

    void generateSyncSequence() {
        // Zadoff-Chu sequence for good autocorrelation
        size_t N = config.M;
        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }
    }

    // Create one OFDM symbol from frequency-domain data
    // Uses only positive frequency subcarriers (bins 1 to M) to avoid
    // conjugate mirror distortion from real-to-complex conversion
    std::vector<Complex> createOFDMSymbol(const std::vector<Complex>& freq_data) {
        std::vector<Complex> freq_domain(config.fft_size, Complex(0, 0));

        // Map data to positive frequency subcarriers only (bins 1 to M)
        // Skip DC (bin 0) and don't use negative frequencies to avoid mirror distortion
        for (size_t i = 0; i < config.M && i < freq_data.size(); ++i) {
            size_t idx = i + 1;  // Start at bin 1, skip DC
            if (idx < config.fft_size / 2) {  // Only positive frequencies
                freq_domain[idx] = freq_data[i];
            }
        }

        // IFFT to time domain
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
    size_t bits_per_symbol = static_cast<size_t>(mod);
    size_t total_symbols = impl_->config.total_data_symbols();

    std::vector<Complex> dd_symbols;
    dd_symbols.reserve(total_symbols);

    size_t data_idx = 0;
    size_t bit_idx = 0;

    for (size_t s = 0; s < total_symbols && data_idx < data.size(); ++s) {
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
        dd_symbols.push_back(mapBits(bits, mod));
    }

    // Pad with zeros
    while (dd_symbols.size() < total_symbols) {
        dd_symbols.push_back(Complex(0, 0));
    }

    return dd_symbols;
}

Samples OTFSModulator::modulate(const std::vector<Complex>& dd_symbols, Modulation mod) {
    uint32_t M = impl_->config.M;
    uint32_t N = impl_->config.N;

    // Step 1: ISFFT (DD → TF)
    std::vector<Complex> tf_grid;
    isfft(dd_symbols, tf_grid, M, N);

    // Step 2: OFDM modulation for each time slot
    // TF grid is N×M (N time slots, M subcarriers)
    Samples output;

    for (uint32_t n = 0; n < N; ++n) {
        // Extract frequency-domain data for this time slot
        std::vector<Complex> freq_data(M);
        for (uint32_t m = 0; m < M; ++m) {
            freq_data[m] = tf_grid[n * M + m];
        }

        // Create OFDM symbol
        auto ofdm_symbol = impl_->createOFDMSymbol(freq_data);

        // Convert to real passband signal
        auto real_symbol = impl_->complexToReal(ofdm_symbol);

        output.insert(output.end(), real_symbol.begin(), real_symbol.end());
    }

    return output;
}

Samples OTFSModulator::generatePreamble() {
    impl_->mixer.reset();

    Samples preamble;

    // Generate sync symbols
    auto sync_symbol = impl_->createOFDMSymbol(impl_->sync_sequence);
    auto sync_real = impl_->complexToReal(sync_symbol);

    // Scale preamble to have similar amplitude to modulated data
    // (prevents SNR degradation in preamble relative to data)
    float preamble_rms = 0;
    for (float s : sync_real) preamble_rms += s * s;
    preamble_rms = std::sqrt(preamble_rms / sync_real.size());
    float target_rms = 0.1f;  // Target RMS to match typical OTFS data
    if (preamble_rms > 0) {
        float scale = target_rms / preamble_rms;
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

    // State
    enum class State { SEARCHING, SYNCED, FRAME_READY };
    State state = State::SEARCHING;

    // Buffers
    std::vector<float> sample_buffer;
    std::vector<Complex> tf_buffer;  // Accumulated TF grid
    uint32_t symbols_received = 0;

    // Sample position tracking for NCO alignment
    size_t total_samples_processed = 0;  // Total samples consumed from input
    size_t current_frame_start = 0;      // Sample offset where current frame starts

    // Output
    std::vector<Complex> dd_symbols;
    std::vector<float> soft_bits;

    // Sync detection
    std::vector<Complex> sync_sequence;
    float sync_threshold = 0.7f;

    Impl(const OTFSConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        generateSyncSequence();
        tf_buffer.resize(cfg.M * cfg.N);
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
        // Create a temporary NCO at the given offset for downconversion
        // This avoids advancing the main mixer during sync search
        NCO temp_mixer(config.center_freq, config.sample_rate);
        for (size_t i = 0; i < sample_offset; ++i) temp_mixer.next();

        for (size_t i = 0; i < count; ++i) {
            Complex carrier = temp_mixer.next();
            // Real-to-complex conversion: real signal times complex carrier
            // The 2x compensation is applied after FFT in demodulateSymbol
            baseband[i] = Complex(samples[i], 0) * std::conj(carrier);
        }
        return baseband;
    }

    // Detect sync using raw real samples (no downconversion needed for autocorrelation)
    bool detectSyncReal(const float* samples, size_t count) {
        size_t sym_len = config.fft_size + config.cp_length;
        if (count < 2 * sym_len) return false;

        // Real-valued autocorrelation at lag sym_len
        float P = 0, R = 0;
        for (size_t i = 0; i < sym_len; ++i) {
            P += samples[i] * samples[i + sym_len];
            R += samples[i + sym_len] * samples[i + sym_len];
        }

        float metric = std::abs(P) / (R + 1e-10f);
        return metric > sync_threshold;
    }

    bool detectSync(const std::vector<Complex>& baseband) {
        // This is now deprecated - use detectSyncReal instead
        return false;
    }

    void demodulateSymbol(const std::vector<Complex>& baseband, uint32_t symbol_idx) {
        size_t sym_start = config.cp_length;
        size_t sym_len = config.fft_size;

        // Extract symbol (skip CP)
        std::vector<Complex> time_domain(baseband.begin() + sym_start,
                                          baseband.begin() + sym_start + sym_len);

        // FFT to frequency domain
        std::vector<Complex> freq_domain;
        fft.forward(time_domain, freq_domain);

        // Extract positive frequency subcarriers (bins 1 to M)
        // Scale by 2.4 to compensate for real-to-complex + FFT normalization
        // (empirically determined: 2.0 for real→complex, 1.2 for FFT scaling)
        for (size_t i = 0; i < config.M; ++i) {
            size_t idx = i + 1;  // Start at bin 1, skip DC
            if (idx < config.fft_size / 2) {
                tf_buffer[symbol_idx * config.M + i] = freq_domain[idx] * 2.4f;
            } else {
                tf_buffer[symbol_idx * config.M + i] = Complex(0, 0);
            }
        }
    }
};

OTFSDemodulator::OTFSDemodulator(const OTFSConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OTFSDemodulator::~OTFSDemodulator() = default;

bool OTFSDemodulator::process(SampleSpan samples) {
    // Add samples to buffer
    impl_->sample_buffer.insert(impl_->sample_buffer.end(),
                                samples.data(), samples.data() + samples.size());

    size_t sym_len = impl_->config.fft_size + impl_->config.cp_length;
    size_t preamble_len = 4 * sym_len;

    while (true) {
        if (impl_->state == Impl::State::SEARCHING) {
            // Need at least 2 symbols for sync detection
            if (impl_->sample_buffer.size() < 2 * sym_len) return false;

            // Use real-valued autocorrelation for sync detection (no downconversion needed)
            if (impl_->detectSyncReal(impl_->sample_buffer.data(), 2 * sym_len)) {
                // Skip preamble (4 sync symbols)
                if (impl_->sample_buffer.size() >= preamble_len) {
                    impl_->total_samples_processed += preamble_len;
                    impl_->current_frame_start = impl_->total_samples_processed;
                    impl_->sample_buffer.erase(impl_->sample_buffer.begin(),
                                               impl_->sample_buffer.begin() + preamble_len);
                    impl_->state = Impl::State::SYNCED;
                    impl_->symbols_received = 0;
                } else {
                    // Need more samples for preamble
                    return false;
                }
            } else {
                // Slide forward and check if we have enough samples to continue
                size_t slide = sym_len / 4;
                if (impl_->sample_buffer.size() > slide) {
                    impl_->total_samples_processed += slide;
                    impl_->sample_buffer.erase(impl_->sample_buffer.begin(),
                                               impl_->sample_buffer.begin() + slide);
                } else {
                    return false;  // Not enough samples
                }
            }
        }
        else if (impl_->state == Impl::State::SYNCED) {
            if (impl_->sample_buffer.size() < sym_len) return false;

            // Calculate sample offset for this symbol
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
            // SFFT to convert TF → DD
            sfft(impl_->tf_buffer, impl_->dd_symbols, impl_->config.M, impl_->config.N);

            // Generate soft bits
            impl_->soft_bits.clear();
            float noise_var = 0.1f;  // Estimate from channel
            for (const auto& sym : impl_->dd_symbols) {
                softDemap(sym, Modulation::QPSK, noise_var, impl_->soft_bits);
            }

            impl_->state = Impl::State::SEARCHING;
            impl_->symbols_received = 0;
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
    // TODO: Implement DD-domain channel estimation
    return {};
}

void OTFSDemodulator::reset() {
    impl_->state = Impl::State::SEARCHING;
    impl_->sample_buffer.clear();
    impl_->symbols_received = 0;
    impl_->total_samples_processed = 0;
    impl_->current_frame_start = 0;
    impl_->dd_symbols.clear();
    impl_->soft_bits.clear();
    impl_->mixer.reset();
}

bool OTFSDemodulator::isSynced() const {
    return impl_->state == Impl::State::SYNCED;
}

} // namespace ultra
