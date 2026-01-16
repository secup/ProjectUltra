#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include <random>
#include <stdexcept>

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

            // Every pilot_spacing carrier is a pilot
            if (pilot_count % config.pilot_spacing == 0) {
                pilot_carrier_indices.push_back(fft_idx);
            } else {
                data_carrier_indices.push_back(fft_idx);
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
};

OFDMModulator::OFDMModulator(const ModemConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

OFDMModulator::~OFDMModulator() = default;

size_t OFDMModulator::samplesPerSymbol() const {
    return impl_->config.getSymbolDuration();
}

size_t OFDMModulator::bitsPerSymbol(Modulation mod) const {
    size_t bits_per_carrier = static_cast<size_t>(mod);
    return impl_->data_carrier_indices.size() * bits_per_carrier;
}

Samples OFDMModulator::modulate(ByteSpan data, Modulation mod) {
    // Note: mixer phase continues from preamble for phase coherence

    size_t bits_per_carrier = static_cast<size_t>(mod);
    size_t carriers_per_symbol = impl_->data_carrier_indices.size();
    size_t bits_per_symbol = carriers_per_symbol * bits_per_carrier;
    size_t bytes_per_symbol = (bits_per_symbol + 7) / 8;

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
            symbol_data.push_back(mapBits(bits, mod));
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

    // Preamble structure:
    // 1. Short training sequence (STS) - for AGC, coarse timing
    // 2. Long training sequence (LTS) - for fine timing, channel est
    // 3. Signal field - modulation/coding info

    Samples preamble;

    // Generate STS: repeated short symbol
    auto short_sym = impl_->createOFDMSymbol(
        std::vector<Complex>(impl_->sync_sequence.begin(),
                            impl_->sync_sequence.begin() + impl_->data_carrier_indices.size()),
        false  // no pilots
    );

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
