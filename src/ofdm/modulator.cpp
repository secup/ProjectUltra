#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include <random>

namespace ultra {

// Constellation mapping tables
namespace {

// Gray-coded BPSK
constexpr Complex BPSK_MAP[] = {
    Complex(-1, 0), Complex(1, 0)
};

// Gray-coded QPSK
constexpr float QPSK_SCALE = 0.7071067811865476f; // 1/sqrt(2)
const Complex QPSK_MAP[] = {
    Complex(-QPSK_SCALE, -QPSK_SCALE),  // 00
    Complex(-QPSK_SCALE,  QPSK_SCALE),  // 01
    Complex( QPSK_SCALE, -QPSK_SCALE),  // 10
    Complex( QPSK_SCALE,  QPSK_SCALE),  // 11
};

// Gray-coded 16-QAM
constexpr float QAM16_SCALE = 0.3162277660168379f; // 1/sqrt(10)
Complex qam16_point(int bits) {
    // Gray code mapping
    static const float levels[] = {-3, -1, 3, 1};
    int i_bits = (bits >> 2) & 0x3;
    int q_bits = bits & 0x3;
    return Complex(levels[i_bits] * QAM16_SCALE, levels[q_bits] * QAM16_SCALE);
}

// Pilot sequence RNG seed - must match between modulator and demodulator
// Using a fixed seed ensures TX/RX pilot sequences are identical
constexpr uint32_t PILOT_RNG_SEED = 0x50494C54;  // "PILT" in ASCII

// 32-QAM rectangular constellation (5 bits per symbol)
// Uses 8×4 grid: 8 Q levels × 4 I levels = 32 points
// Gray-coded for minimum bit errors on adjacent symbol errors
//
// Bit mapping:
//   b4b3b2: Q position (3 bits, Gray coded for 8 levels)
//   b1b0: I position (2 bits, Gray coded for 4 levels)
//
// Average power calculation:
// I variance: (9+1+1+9)/4 × scale² = 5 × scale²
// Q variance: (49+25+9+1+1+9+25+49)/8 × scale² = 21 × scale²
// Total: 26 × scale² = 1 → scale = 1/sqrt(26)
Complex qam32_point(int bits) {
    constexpr float QAM32_SCALE = 0.1961161351381840f;  // 1/sqrt(26) for unit average power

    // Gray-coded I levels (2 bits): 00→-3, 01→-1, 11→+1, 10→+3
    static const float I_LEVELS[4] = {-3, -1, 1, 3};
    static const int I_GRAY[4] = {0, 1, 3, 2};  // Gray decode: 00→0, 01→1, 11→2, 10→3

    // Gray-coded Q levels (3 bits): 000→-7, 001→-5, 011→-3, 010→-1, 110→+1, 111→+3, 101→+5, 100→+7
    static const float Q_LEVELS[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    static const int Q_GRAY[8] = {0, 1, 3, 2, 6, 7, 5, 4};  // Gray decode

    int q_bits = (bits >> 2) & 0x7;  // bits 4,3,2
    int i_bits = bits & 0x3;         // bits 1,0

    // Find actual Q and I indices from Gray code
    int q_idx = 0, i_idx = 0;
    for (int i = 0; i < 4; ++i) if (I_GRAY[i] == i_bits) { i_idx = i; break; }
    for (int i = 0; i < 8; ++i) if (Q_GRAY[i] == q_bits) { q_idx = i; break; }

    return Complex(I_LEVELS[i_idx] * QAM32_SCALE, Q_LEVELS[q_idx] * QAM32_SCALE);
}

// Map bits to constellation point
Complex mapBits(uint32_t bits, Modulation mod) {
    switch (mod) {
        case Modulation::BPSK:
            return BPSK_MAP[bits & 1];
        case Modulation::QPSK:
            return QPSK_MAP[bits & 3];
        case Modulation::QAM16:
            return qam16_point(bits & 0xF);
        case Modulation::QAM32:
            return qam32_point(bits & 0x1F);
        case Modulation::QAM64: {
            // 64-QAM: 6 bits -> I(3 bits) + Q(3 bits)
            static const float levels[] = {-7, -5, -1, -3, 7, 5, 1, 3};
            static const float scale = 0.1543033499620919f;  // 1/sqrt(42)
            int i_bits = (bits >> 3) & 0x7;
            int q_bits = bits & 0x7;
            return Complex(levels[i_bits] * scale, levels[q_bits] * scale);
        }
        case Modulation::QAM256: {
            // 256-QAM: 8 bits -> I(4 bits) + Q(4 bits)
            // Gray-coded: {-15,-13,-9,-11,-1,-3,-7,-5,15,13,9,11,1,3,7,5}
            static const float levels[] = {-15,-13,-9,-11,-1,-3,-7,-5,15,13,9,11,1,3,7,5};
            static const float scale = 0.0645497224367903f;  // 1/sqrt(170) for unit power
            int i_bits = (bits >> 4) & 0xF;
            int q_bits = bits & 0xF;
            return Complex(levels[i_bits] * scale, levels[q_bits] * scale);
        }
        default:
            return QPSK_MAP[bits & 3];
    }
}

} // anonymous namespace

// Flag to log TX pilots only once per transmission (reset in generatePreamble)
static bool g_logged_tx_pilots = false;

struct OFDMModulator::Impl {
    ModemConfig config;
    FFT fft;
    NCO mixer;

    // Preamble sequences (Zadoff-Chu for good correlation properties)
    std::vector<Complex> sync_sequence;
    std::vector<Complex> pilot_sequence;

    // Carrier mapping
    std::vector<int> data_carrier_indices;
    std::vector<int> pilot_carrier_indices;

    // DBPSK state: previous symbol per data carrier for differential encoding
    std::vector<Complex> dbpsk_prev_symbols;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq, cfg.sample_rate)
    {
        setupCarriers();
        generateSequences();
    }

    void setupCarriers() {
        // Place carriers symmetrically around DC
        // Leave DC null, use center_freq for baseband

        int half_carriers = config.num_carriers / 2;
        int fft_half = config.fft_size / 2;

        // Calculate carrier spacing to fit in ~2.4 kHz
        // With 48 carriers in ~2400 Hz, spacing is 50 Hz
        // At 48000 sample rate and 512 FFT, bin spacing is 93.75 Hz
        // We'll use every carrier but limit the bandwidth

        data_carrier_indices.clear();
        pilot_carrier_indices.clear();

        int pilot_count = 0;
        for (int i = -half_carriers; i <= half_carriers; ++i) {
            if (i == 0) continue;  // Skip DC

            int fft_idx = (i + config.fft_size) % config.fft_size;

            // If use_pilots=false (e.g., DQPSK), all carriers are data
            if (!config.use_pilots) {
                data_carrier_indices.push_back(fft_idx);
            } else {
                // Every pilot_spacing carrier is a pilot
                if (pilot_count % config.pilot_spacing == 0) {
                    pilot_carrier_indices.push_back(fft_idx);
                } else {
                    data_carrier_indices.push_back(fft_idx);
                }
            }
            ++pilot_count;
        }

    }

    void generateSequences() {
        // Generate Zadoff-Chu sequence for sync preamble
        // Excellent autocorrelation properties
        size_t N = config.num_carriers;
        size_t u = 1;  // Root index (coprime with N)

        sync_sequence.resize(N);
        for (size_t n = 0; n < N; ++n) {
            float phase = -M_PI * u * n * (n + 1) / N;
            sync_sequence[n] = Complex(std::cos(phase), std::sin(phase));
        }

        // Pilot sequence: known BPSK pattern (pseudo-random but deterministic)
        pilot_sequence.resize(pilot_carrier_indices.size());
        std::mt19937 rng(PILOT_RNG_SEED);
        for (size_t i = 0; i < pilot_sequence.size(); ++i) {
            pilot_sequence[i] = (rng() & 1) ? Complex(1, 0) : Complex(-1, 0);
        }

        // DEBUG: Log pilot configuration for comparison with RX
        LOG_DEBUG("MOD", "Mod pilot config: %zu pilots, %zu data carriers",
                 pilot_carrier_indices.size(), data_carrier_indices.size());
        if (pilot_carrier_indices.size() >= 3) {
            LOG_DEBUG("MOD", "Mod pilot indices[0-2]: %d, %d, %d",
                     pilot_carrier_indices[0], pilot_carrier_indices[1], pilot_carrier_indices[2]);
        }
        if (pilot_sequence.size() >= 3) {
            LOG_DEBUG("MOD", "Mod pilot seq[0-2]: (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)",
                     pilot_sequence[0].real(), pilot_sequence[0].imag(),
                     pilot_sequence[1].real(), pilot_sequence[1].imag(),
                     pilot_sequence[2].real(), pilot_sequence[2].imag());
        }
    }

    std::vector<Complex> createOFDMSymbol(const std::vector<Complex>& data_symbols,
                                          bool include_pilots = true) {
        std::vector<Complex> freq_domain(config.fft_size, Complex(0, 0));

        // Map data to carriers
        for (size_t i = 0; i < data_carrier_indices.size() && i < data_symbols.size(); ++i) {
            freq_domain[data_carrier_indices[i]] = data_symbols[i];
        }


        // Add pilots
        if (include_pilots) {
            for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                freq_domain[pilot_carrier_indices[i]] = pilot_sequence[i];
            }

            // DEBUG: Log first data symbol's TX pilot values (only once per transmission)
            // Note: g_logged_tx_pilots is defined and reset in generatePreamble()
            if (!g_logged_tx_pilots) {
                LOG_DEBUG("MOD", "=== TX freq_domain before IFFT (first data symbol) ===");
                for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                    int idx = pilot_carrier_indices[i];
                    LOG_DEBUG("MOD", "TX Pilot[%zu] idx=%d: freq_domain=(%.2f,%.2f) pilot_seq=(%.1f,%.1f)",
                             i, idx,
                             freq_domain[idx].real(), freq_domain[idx].imag(),
                             pilot_sequence[i].real(), pilot_sequence[i].imag());
                }
                // Also log a few data carriers
                LOG_DEBUG("MOD", "TX Data[0-2]: idx=%d (%.2f,%.2f), idx=%d (%.2f,%.2f), idx=%d (%.2f,%.2f)",
                         data_carrier_indices[0], freq_domain[data_carrier_indices[0]].real(), freq_domain[data_carrier_indices[0]].imag(),
                         data_carrier_indices[1], freq_domain[data_carrier_indices[1]].real(), freq_domain[data_carrier_indices[1]].imag(),
                         data_carrier_indices[2], freq_domain[data_carrier_indices[2]].real(), freq_domain[data_carrier_indices[2]].imag());
                g_logged_tx_pilots = true;
            }
        }

        // IFFT to time domain
        std::vector<Complex> time_domain;
        fft.inverse(freq_domain, time_domain);

        // Add cyclic prefix
        std::vector<Complex> symbol_with_cp;
        uint32_t cp_len = config.getCyclicPrefix();
        symbol_with_cp.reserve(config.fft_size + cp_len);

        // CP is copy of end of symbol
        for (size_t i = config.fft_size - cp_len; i < config.fft_size; ++i) {
            symbol_with_cp.push_back(time_domain[i]);
        }
        for (const auto& s : time_domain) {
            symbol_with_cp.push_back(s);
        }

        return symbol_with_cp;
    }

    Samples complexToReal(const std::vector<Complex>& complex_signal) {
        // Mix up to center frequency and take real part
        Samples real_signal(complex_signal.size());
        for (size_t i = 0; i < complex_signal.size(); ++i) {
            Complex mixed = complex_signal[i] * mixer.next();
            real_signal[i] = mixed.real();
        }
        return real_signal;
    }

    // ========================================================================
    // SCHMIDL-COX PREAMBLE
    // ========================================================================
    //
    // Creates an STS symbol using only EVEN FFT bins. This produces a time-domain
    // signal where the second half is identical to the first half:
    //   x[n] = x[n + N/2]  for n in [0, N/2-1]
    //
    // Benefits:
    // - CFO range doubles: ±fs/N = ±93.75 Hz (vs ±42.9 Hz with full-symbol correlation)
    // - Schmidl-Cox timing metric: M(d) = |P(d)|² / R(d)²
    // - More robust plateau-shaped timing metric
    //
    std::vector<Complex> createSchmidlCoxSTS() {
        std::vector<Complex> freq_domain(config.fft_size, Complex(0, 0));

        // Place sync sequence on EVEN FFT bins only
        // This creates two identical halves in time domain
        size_t seq_idx = 0;
        for (int carrier_idx : data_carrier_indices) {
            // Only use even FFT bin indices
            if (carrier_idx % 2 == 0) {
                freq_domain[carrier_idx] = sync_sequence[seq_idx % sync_sequence.size()];
            }
            seq_idx++;
        }

        // IFFT to time domain
        std::vector<Complex> time_domain;
        fft.inverse(freq_domain, time_domain);

        // Add cyclic prefix
        std::vector<Complex> symbol_with_cp;
        uint32_t cp_len = config.getCyclicPrefix();
        symbol_with_cp.reserve(config.fft_size + cp_len);

        // CP is copy of end of symbol
        for (size_t i = config.fft_size - cp_len; i < config.fft_size; ++i) {
            symbol_with_cp.push_back(time_domain[i]);
        }
        for (const auto& s : time_domain) {
            symbol_with_cp.push_back(s);
        }

        return symbol_with_cp;
    }
};

OFDMModulator::OFDMModulator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OFDMModulator::~OFDMModulator() = default;

size_t OFDMModulator::samplesPerSymbol() const {
    return impl_->config.getSymbolDuration();
}

size_t OFDMModulator::bitsPerSymbol(Modulation mod) const {
    // Use proper function - enum values don't correspond to bit counts!
    size_t bits_per_carrier = getBitsPerSymbol(mod);
    return impl_->data_carrier_indices.size() * bits_per_carrier;
}

Samples OFDMModulator::modulate(ByteSpan data, Modulation mod) {
    // Note: mixer phase continues from preamble for phase coherence

    // Use the proper function to get bits per symbol, NOT the enum value!
    // (enum values don't correspond to bit counts)
    size_t bits_per_carrier = getBitsPerSymbol(mod);
    size_t carriers_per_symbol = impl_->data_carrier_indices.size();
    size_t bits_per_symbol = carriers_per_symbol * bits_per_carrier;
    size_t bytes_per_symbol = (bits_per_symbol + 7) / 8;

    // DEBUG: Check DBPSK reference state at start of modulate
    if (mod == Modulation::DQPSK || mod == Modulation::D8PSK || mod == Modulation::DBPSK) {
        LOG_DEBUG("MOD", "modulate() entry: dbpsk_prev_symbols.size()=%zu, first 3: (%.2f,%.2f) (%.2f,%.2f) (%.2f,%.2f)",
                 impl_->dbpsk_prev_symbols.size(),
                 impl_->dbpsk_prev_symbols.size() > 0 ? impl_->dbpsk_prev_symbols[0].real() : 0,
                 impl_->dbpsk_prev_symbols.size() > 0 ? impl_->dbpsk_prev_symbols[0].imag() : 0,
                 impl_->dbpsk_prev_symbols.size() > 1 ? impl_->dbpsk_prev_symbols[1].real() : 0,
                 impl_->dbpsk_prev_symbols.size() > 1 ? impl_->dbpsk_prev_symbols[1].imag() : 0,
                 impl_->dbpsk_prev_symbols.size() > 2 ? impl_->dbpsk_prev_symbols[2].real() : 0,
                 impl_->dbpsk_prev_symbols.size() > 2 ? impl_->dbpsk_prev_symbols[2].imag() : 0);
    }

    Samples output;

    // Process data in symbol-sized chunks
    size_t data_idx = 0;
    size_t bit_idx = 0;

    while (data_idx < data.size()) {
        std::vector<Complex> symbol_data;
        symbol_data.reserve(carriers_per_symbol);

        // Extract bits for each carrier
        for (size_t c = 0; c < carriers_per_symbol && data_idx < data.size(); ++c) {
            uint32_t bits = 0;
            for (size_t b = 0; b < bits_per_carrier; ++b) {
                bits <<= 1;  // Always shift left (padding with 0 when no more data)
                if (data_idx < data.size()) {
                    uint8_t byte = data[data_idx];
                    uint8_t bit = (byte >> (7 - bit_idx)) & 1;
                    bits |= bit;

                    ++bit_idx;
                    if (bit_idx >= 8) {
                        bit_idx = 0;
                        ++data_idx;
                    }
                }
            }

            // Differential encoding: multiply previous symbol by phase rotation
            if (mod == Modulation::DBPSK) {
                // 1 bit: 0 → 0° (+1), 1 → 180° (-1)
                Complex phase_change = (bits & 1) ? Complex(-1, 0) : Complex(1, 0);
                Complex new_symbol = impl_->dbpsk_prev_symbols[c] * phase_change;
                impl_->dbpsk_prev_symbols[c] = new_symbol;
                symbol_data.push_back(new_symbol);
            } else if (mod == Modulation::DQPSK) {
                // 2 bits: 00→0°, 01→90°, 10→180°, 11→270°
                // Phase rotations: +1, +j, -1, -j
                static const Complex dqpsk_phases[4] = {
                    Complex(1, 0),   // 00 → 0°
                    Complex(0, 1),   // 01 → 90°
                    Complex(-1, 0),  // 10 → 180°
                    Complex(0, -1)   // 11 → 270°
                };
                Complex phase_change = dqpsk_phases[bits & 3];
                Complex new_symbol = impl_->dbpsk_prev_symbols[c] * phase_change;
                impl_->dbpsk_prev_symbols[c] = new_symbol;
                symbol_data.push_back(new_symbol);
            } else if (mod == Modulation::D8PSK) {
                // 3 bits: phase changes in 45° increments with 22.5° offset
                // Offset ensures sin()-based LLR formulas work (sin(0°)=0 is bad)
                // 000→22.5°, 001→67.5°, 010→112.5°, 011→157.5°, etc.
                static const float pi = 3.14159265358979f;
                float angle = (bits & 7) * (pi / 4.0f) + pi / 8.0f;  // 45° steps + 22.5° offset
                Complex phase_change(std::cos(angle), std::sin(angle));
                Complex new_symbol = impl_->dbpsk_prev_symbols[c] * phase_change;
                impl_->dbpsk_prev_symbols[c] = new_symbol;
                symbol_data.push_back(new_symbol);
            } else {
                Complex sym = mapBits(bits, mod);
                symbol_data.push_back(sym);

                }
        }

        // Pad if needed
        while (symbol_data.size() < carriers_per_symbol) {
            symbol_data.push_back(Complex(0, 0));
        }

        // Create OFDM symbol
        auto complex_symbol = impl_->createOFDMSymbol(symbol_data);

        // Convert to real signal
        auto real_symbol = impl_->complexToReal(complex_symbol);

        // Add guard interval (zeros)
        // IMPORTANT: Also advance mixer phase to match RX processing
        // RX's toBaseband() processes symbol_samples which includes guard,
        // so TX mixer must advance the same amount for phase coherence
        for (uint32_t i = 0; i < impl_->config.symbol_guard; ++i) {
            real_symbol.push_back(0);
            impl_->mixer.next();  // Keep mixer in sync with RX
        }

        output.insert(output.end(), real_symbol.begin(), real_symbol.end());
    }

    return output;
}

Samples OFDMModulator::generatePreamble() {
    // Reset mixer phase so each transmission starts at phase 0
    // This ensures consistent signal polarity across transmissions
    impl_->mixer.reset();

    // Reset TX pilot logging flag for new transmission
    g_logged_tx_pilots = false;

    // Initialize DBPSK state: all carriers start at +1 (reference symbol)
    impl_->dbpsk_prev_symbols.assign(impl_->data_carrier_indices.size(), Complex(1, 0));

    // Preamble structure (Schmidl-Cox):
    // 0. Guard prefix - silence before signal for sync detector headroom
    // 1. Short training sequence (STS) - Schmidl-Cox style (even subcarriers only)
    //    Each STS symbol has two identical halves for wide-range CFO estimation
    //    CFO range: ±fs/N = ±93.75 Hz (for N=512)
    // 2. Long training sequence (LTS) - for fine timing, channel estimation
    //    Uses all subcarriers for proper channel estimation

    Samples preamble;

    // Guard prefix: 1 OFDM symbol worth of silence (~12ms at 48kHz)
    // This ensures sync detector always has "before" samples to correlate against
    // In real HF, radio PTT delay provides this naturally (50-300ms)
    // We add minimal guard so TX works even in direct loopback tests
    constexpr size_t GUARD_SAMPLES = 560;  // CP + FFT = one symbol period
    preamble.resize(GUARD_SAMPLES, 0.0f);

    // Generate Schmidl-Cox STS: only even subcarriers
    // Time-domain: x[n] = x[n + N/2], enabling half-symbol correlation
    auto short_sym = impl_->createSchmidlCoxSTS();

    // Repeat STS 4 times for robust sync
    auto sts_real = impl_->complexToReal(short_sym);
    for (int i = 0; i < 4; ++i) {
        preamble.insert(preamble.end(), sts_real.begin(), sts_real.end());
    }

    // Generate LTS: full sync symbol with pilots
    std::vector<Complex> lts_data(impl_->data_carrier_indices.size());
    for (size_t i = 0; i < lts_data.size(); ++i) {
        lts_data[i] = impl_->sync_sequence[i % impl_->sync_sequence.size()];
    }
    auto long_sym = impl_->createOFDMSymbol(lts_data, true);
    auto lts_real = impl_->complexToReal(long_sym);

    // Repeat LTS 2 times
    for (int i = 0; i < 2; ++i) {
        preamble.insert(preamble.end(), lts_real.begin(), lts_real.end());
    }

    return preamble;
}

Samples OFDMModulator::generateProbe() {
    // Channel probe: known sequence across all carriers
    // Used for channel quality measurement

    std::vector<Complex> probe_data(impl_->data_carrier_indices.size());
    for (size_t i = 0; i < probe_data.size(); ++i) {
        // Chirp-like pattern for good correlation
        float phase = M_PI * i * i / probe_data.size();
        probe_data[i] = Complex(std::cos(phase), std::sin(phase));
    }

    auto complex_probe = impl_->createOFDMSymbol(probe_data, true);
    return impl_->complexToReal(complex_probe);
}

} // namespace ultra
