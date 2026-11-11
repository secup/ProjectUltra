#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "ultra/logging.hpp"
#include "demodulator_constants.hpp"
#include "pilot_pattern.hpp"
#include "papr_ace.hpp"
#include "genie_tx_capture.hpp"
#include "ultra/ofdm_link_adaptation.hpp"
#include <algorithm>
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

Complex qam8_point(int bits) {
    // Coherent 8PSK uses the same Gray phase ordering as the differential
    // D8PSK rung, but referenced to the absolute pilot/LTS phase.
    static const int data_to_phase[8] = {0, 1, 3, 2, 7, 6, 4, 5};
    static const float pi = 3.14159265358979f;
    const float angle = data_to_phase[bits & 7] * (pi / 4.0f) + pi / 8.0f;
    return Complex(std::cos(angle), std::sin(angle));
}

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
        case Modulation::QAM8:
            return qam8_point(bits & 0x7);
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
    std::vector<int> all_carrier_fft_indices;
    std::vector<size_t> data_logical_carrier_indices;
    std::vector<size_t> pilot_logical_carrier_indices;
    std::vector<bool> is_pilot_logical;
    Symbol last_data_carrier_symbols_for_testing;

    // Reused OFDM hot-path scratch. The modulator is used serially by one
    // waveform instance, so member scratch removes per-symbol heap churn
    // without changing the transmitted samples.
    std::vector<Complex> freq_domain_scratch;
    std::vector<Complex> time_domain_scratch;
    std::vector<Complex> symbol_with_cp_scratch;
    std::vector<Complex> symbol_data_scratch;
    std::vector<Complex> lts_data_scratch;
    std::vector<Complex> probe_data_scratch;
    Samples real_signal_scratch;

    Impl(const ModemConfig& cfg)
        : config(cfg)
        , fft(cfg.fft_size)
        , mixer(cfg.center_freq + cfg.tx_cfo_hz, cfg.sample_rate)  // Apply TX CFO to carrier
    {
        if (std::abs(cfg.tx_cfo_hz) > 0.01f) {
            LOG_MODEM(INFO, "OFDMModulator: CFO=%.1f Hz applied (mixer: %.1f + %.1f = %.1f Hz)",
                      cfg.tx_cfo_hz, (float)cfg.center_freq, cfg.tx_cfo_hz,
                      cfg.center_freq + cfg.tx_cfo_hz);
        }
        setupCarriers();
        generateSequences();
        prepareScratch();
    }

    void prepareScratch() {
        const size_t cp_len = config.getCyclicPrefix();
        freq_domain_scratch.resize(config.fft_size);
        time_domain_scratch.resize(config.fft_size);
        symbol_with_cp_scratch.resize(config.fft_size + cp_len);
        symbol_data_scratch.reserve(data_carrier_indices.size());
        lts_data_scratch.resize(data_carrier_indices.size());
        probe_data_scratch.resize(data_carrier_indices.size());
        real_signal_scratch.reserve(config.fft_size + cp_len);
    }

    void setupCarriers() {
        activateCarrierPattern(0);
    }

    void activateCarrierPattern(size_t symbol_index) {
        ofdm_pilots::buildCarrierPattern(config,
                                         symbol_index,
                                         all_carrier_fft_indices,
                                         data_carrier_indices,
                                         pilot_carrier_indices,
                                         data_logical_carrier_indices,
                                         pilot_logical_carrier_indices,
                                         is_pilot_logical,
                                         pilot_sequence);
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

    static bool carrierMaskActive(uint64_t active_carrier_mask, size_t logical_carrier) {
        if (logical_carrier >= 64) {
            return true;
        }
        return (active_carrier_mask & (uint64_t{1} << logical_carrier)) != 0;
    }

    const std::vector<Complex>& createOFDMSymbol(const std::vector<Complex>& data_symbols,
                                                bool include_pilots = true,
                                                uint64_t active_carrier_mask = UINT64_MAX,
                                                bool carrier_mask_enabled = false,
                                                bool genie_capture = false) {
        auto& freq_domain = freq_domain_scratch;
        freq_domain.resize(config.fft_size);
        std::fill(freq_domain.begin(), freq_domain.end(), Complex(0, 0));

        // Map data to carriers
        for (size_t i = 0; i < data_carrier_indices.size() && i < data_symbols.size(); ++i) {
            const bool active = !carrier_mask_enabled ||
                carrierMaskActive(active_carrier_mask, data_logical_carrier_indices[i]);
            freq_domain[data_carrier_indices[i]] = active ? data_symbols[i] : Complex(0, 0);
        }

        // Add pilots
        if (include_pilots) {
            for (size_t i = 0; i < pilot_carrier_indices.size(); ++i) {
                const bool active = !carrier_mask_enabled ||
                    carrierMaskActive(active_carrier_mask, pilot_logical_carrier_indices[i]);
                freq_domain[pilot_carrier_indices[i]] = active ? pilot_sequence[i] : Complex(0, 0);
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

        // ACE PAPR reduction (ULTRA_ACE_PAPR, default off; 2026-07-13). DATA
        // symbols only (genie_capture marks them — never LTS/probe/preamble, so
        // sync/channel-estimation waveforms stay pristine) and coherent mods
        // only. Extends outer constellation points OUTWARD to shave time peaks
        // → after the TX peak-normalization, ~0.8 dB more average power on the
        // air, with ZERO harmful EVM (points move only away from decision
        // boundaries → RX BER equal-or-better). No RX change. Unit-proven in
        // test_papr_ace. Unlike the crude clip (rig-measured −58% on clean
        // channels), ACE is strictly non-negative.
        static const bool ace_enabled = [] {
            const char* e = std::getenv("ULTRA_ACE_PAPR");
            return e != nullptr && e[0] != '\0' && e[0] != '0';
        }();
        if (ace_enabled && genie_capture &&
            ultra::ofdm_link_adaptation::isCoherentModulation(config.modulation)) {
            ofdm::papr_ace::applyAce(freq_domain, data_carrier_indices,
                               config.modulation, fft,
                               /*clip_ratio=*/1.6f, /*max_iters=*/8, /*mu=*/1.0f);
        }

        // 2026-05-29 diag (ULTRA_GENIE_DATA_AIDED): capture the exact transmitted
        // freq-domain symbol so the decoder can form the true per-symbol channel
        // H[k] = Y[k]/X[k]. ONLY for DATA symbols (genie_capture=true at the data
        // caller) — NOT for LTS/probe symbols, which the decoder handles separately
        // and does not equalize. Each entry is stamped with the ABSOLUTE TX-stream
        // offset of its FRAME (frame_data_base, supplied by the encoder) plus its index
        // within that frame and the carrier geometry; the decoder addresses entries by
        // (frame anchor, symbol index), never by a FIFO cursor. Frame granularity is
        // load-bearing — see genie_tx_capture.hpp.
        //
        // 2026-07-28 ORDERING FIX: the capture now sits AFTER the ACE block above,
        // i.e. it captures what actually goes into the IFFT. It used to sit before
        // ACE, so with ULTRA_ACE_PAPR=1 the genie divided by the PRE-ACE symbol while
        // the wire carried the POST-ACE one — a silently wrong "exact" H.
        if (genie_capture && ultra::genie::txCapture().enabled) {
            ultra::genie::txCapture().push(freq_domain,
                                           ultra::genie::txCapture().frame_data_base,
                                           data_carrier_indices.size(),
                                           pilot_carrier_indices.size());
        }

        // IFFT to time domain
        auto& time_domain = time_domain_scratch;
        time_domain.resize(config.fft_size);
        fft.inverse(freq_domain.data(), time_domain.data());

        // Add cyclic prefix
        uint32_t cp_len = config.getCyclicPrefix();
        auto& symbol_with_cp = symbol_with_cp_scratch;
        symbol_with_cp.resize(config.fft_size + cp_len);

        // CP is copy of end of symbol
        for (size_t i = config.fft_size - cp_len; i < config.fft_size; ++i) {
            symbol_with_cp[i - (config.fft_size - cp_len)] = time_domain[i];
        }
        std::copy(time_domain.begin(), time_domain.end(), symbol_with_cp.begin() + cp_len);

        return symbol_with_cp;
    }

    const Samples& complexToReal(const std::vector<Complex>& complex_signal) {
        // Mix up to center frequency, take real part, and apply output scaling.
        // The SAME average power per OFDM symbol for every geometry: with the 1/N inverse FFT a
        // symbol of Ncar unit-power carriers has real passband power (Ncar / 2)(scale / N)^2, so
        // the scale grows as N / sqrt(Ncar) from the wideband geometry output_scale was set on
        // (1024-point FFT, 59 carriers: factor exactly 1, wideband unchanged). Until 2026-11-10
        // one scale served every geometry and the narrowband symbol (2048, 21 carriers) left the
        // modulator 10.5 dB below the wideband one; the transmission's 0.8 peak normalization
        // (ModemEngine::postProcessTx) then let the constant-envelope chirp set the peak, and the
        // narrowband data rode 6.7 dB under the transmitter's budget (BUG-NARROW-TX-LEVEL).
        auto& real_signal = real_signal_scratch;
        real_signal.resize(complex_signal.size());
        constexpr float kLevelReferenceFft = 1024.0f;
        constexpr float kLevelReferenceCarriers = 59.0f;
        const float scale =
            config.output_scale * (static_cast<float>(config.fft_size) / kLevelReferenceFft) *
            std::sqrt(kLevelReferenceCarriers /
                      std::max(1.0f, static_cast<float>(config.num_carriers)));
        for (size_t i = 0; i < complex_signal.size(); ++i) {
            Complex mixed = complex_signal[i] * mixer.next();
            real_signal[i] = mixed.real() * scale;
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
    const std::vector<Complex>& createSchmidlCoxSTS() {
        auto& freq_domain = freq_domain_scratch;
        freq_domain.resize(config.fft_size);
        std::fill(freq_domain.begin(), freq_domain.end(), Complex(0, 0));

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
        auto& time_domain = time_domain_scratch;
        time_domain.resize(config.fft_size);
        fft.inverse(freq_domain.data(), time_domain.data());

        // Add cyclic prefix
        uint32_t cp_len = config.getCyclicPrefix();
        auto& symbol_with_cp = symbol_with_cp_scratch;
        symbol_with_cp.resize(config.fft_size + cp_len);

        // CP is copy of end of symbol
        for (size_t i = config.fft_size - cp_len; i < config.fft_size; ++i) {
            symbol_with_cp[i - (config.fft_size - cp_len)] = time_domain[i];
        }
        std::copy(time_domain.begin(), time_domain.end(), symbol_with_cp.begin() + cp_len);

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

Samples OFDMModulator::modulate(ByteSpan data, Modulation mod,
                                 uint64_t active_carrier_mask,
                                 bool carrier_mask_enabled) {
    // Note: mixer phase continues from preamble for phase coherence
    impl_->activateCarrierPattern(0);

    // Use the proper function to get bits per symbol, NOT the enum value!
    // (enum values don't correspond to bit counts)
    size_t bits_per_carrier = getBitsPerSymbol(mod);
    size_t carriers_per_symbol = impl_->data_carrier_indices.size();

    Samples output;
    const size_t bits_per_symbol = carriers_per_symbol * bits_per_carrier;
    if (bits_per_symbol > 0) {
        const size_t symbol_count = (data.size() * 8 + bits_per_symbol - 1) / bits_per_symbol;
        output.reserve(symbol_count * impl_->config.getSymbolDuration());
        impl_->last_data_carrier_symbols_for_testing.reserve(symbol_count * carriers_per_symbol);
    }
    impl_->last_data_carrier_symbols_for_testing.clear();

    // Process data in symbol-sized chunks
    size_t data_idx = 0;
    size_t bit_idx = 0;
    size_t symbol_index = 0;

    while (data_idx < data.size()) {
        impl_->activateCarrierPattern(symbol_index);
        auto& symbol_data = impl_->symbol_data_scratch;
        symbol_data.clear();

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

            // Debug: log first few symbols' bit extraction
            static size_t symbol_count = 0;
            if (symbol_count < 5 && c < 3) {
                LOG_DEBUG("MOD", "Sym %zu Car %zu: bits=%d (0x%02x) from byte=%d idx=%zu",
                         symbol_count, c, bits, bits,
                         data_idx > 0 ? (int)data[data_idx-1] : (int)data[0],
                         bit_idx);
            }
            if (c == carriers_per_symbol - 1) symbol_count++;

            // Coherent constellation mapping (OFDM is coherent-only; the
            // differential DBPSK/DQPSK/D8PSK TX encoder was removed — differential
            // lives in MC-DPSK, which uses its own modulator).
            Complex sym = mapBits(bits, mod);
            symbol_data.push_back(sym);
        }

        // Pad if needed
        while (symbol_data.size() < carriers_per_symbol) {
            symbol_data.push_back(Complex(0, 0));
        }

        for (size_t c = 0; c < symbol_data.size() &&
                           c < impl_->data_logical_carrier_indices.size(); ++c) {
            const bool active = !carrier_mask_enabled ||
                OFDMModulator::Impl::carrierMaskActive(
                    active_carrier_mask, impl_->data_logical_carrier_indices[c]);
            impl_->last_data_carrier_symbols_for_testing.push_back(
                active ? symbol_data[c] : Complex(0, 0));
        }

        // Create OFDM symbol
        const auto& complex_symbol = impl_->createOFDMSymbol(symbol_data, true,
                                                            active_carrier_mask,
                                                            carrier_mask_enabled,
                                                            /*genie_capture=*/true);

        // Convert to real signal
        const auto& real_symbol = impl_->complexToReal(complex_symbol);
        output.insert(output.end(), real_symbol.begin(), real_symbol.end());

        // Add guard interval (zeros)
        // IMPORTANT: Also advance mixer phase to match RX processing
        // RX's toBaseband() processes symbol_samples which includes guard,
        // so TX mixer must advance the same amount for phase coherence
        for (uint32_t i = 0; i < impl_->config.symbol_guard; ++i) {
            output.push_back(0);
            impl_->mixer.next();  // Keep mixer in sync with RX
        }
        ++symbol_index;
    }

    return output;
}

Symbol OFDMModulator::getLastDataCarrierSymbolsForTesting() const {
    return impl_->last_data_carrier_symbols_for_testing;
}

Samples OFDMModulator::generatePreamble() {
    // Reset mixer phase so each transmission starts at phase 0
    // This ensures consistent signal polarity across transmissions
    impl_->mixer.reset();

    // Reset TX pilot logging flag for new transmission
    g_logged_tx_pilots = false;

    impl_->activateCarrierPattern(0);

    // Preamble structure (Schmidl-Cox):
    // 0. Guard prefix - silence before signal for sync detector headroom
    // 1. Short training sequence (STS) - Schmidl-Cox style (even subcarriers only)
    //    Each STS symbol has two identical halves for wide-range CFO estimation
    //    CFO range: ±fs/N = ±93.75 Hz (for N=512)
    // 2. Long training sequence (LTS) - for fine timing, channel estimation
    //    Uses all subcarriers for proper channel estimation

    // Guard prefix: 1 OFDM symbol worth of silence (~12ms at 48kHz)
    // This ensures sync detector always has "before" samples to correlate against
    // In real HF, radio PTT delay provides this naturally (50-300ms)
    // We add minimal guard so TX works even in direct loopback tests
    // Guard scales with FFT size: 512 FFT → 560 samples, 1024 FFT → 1120 samples
    size_t guard_samples = impl_->config.fft_size + impl_->config.getCyclicPrefix();
    Samples preamble;
    preamble.reserve(guard_samples * 7);
    preamble.resize(guard_samples, 0.0f);

    // Generate Schmidl-Cox STS: only even subcarriers
    // Time-domain: x[n] = x[n + N/2], enabling half-symbol correlation
    const auto& short_sym = impl_->createSchmidlCoxSTS();

    // Repeat STS 4 times for robust sync
    const auto& sts_real = impl_->complexToReal(short_sym);
    for (int i = 0; i < 4; ++i) {
        preamble.insert(preamble.end(), sts_real.begin(), sts_real.end());
    }

    // Generate LTS: full sync symbol with pilots
    auto& lts_data = impl_->lts_data_scratch;
    lts_data.resize(impl_->data_carrier_indices.size());
    for (size_t i = 0; i < lts_data.size(); ++i) {
        lts_data[i] = impl_->sync_sequence[i % impl_->sync_sequence.size()];
    }
    const auto& long_sym = impl_->createOFDMSymbol(lts_data, true);
    const auto& lts_real = impl_->complexToReal(long_sym);

    // Repeat LTS 2 times
    for (int i = 0; i < 2; ++i) {
        preamble.insert(preamble.end(), lts_real.begin(), lts_real.end());
    }

    return preamble;
}

Samples OFDMModulator::generateTrainingSymbols(int count) {
    // Generate LTS training symbols for chirp-based acquisition
    //
    // IMPORTANT: This resets the mixer so training symbols start at phase 0.
    // Call this AFTER generating chirp, BEFORE modulate().
    // The modulate() call will continue from the mixer phase after training.
    //
    // This ensures TX sequence: chirp + training + data are all phase-coherent.

    impl_->mixer.reset();

    // Set up the symbol-0 carrier pattern; data_carrier_indices feeds LTS generation.
    impl_->activateCarrierPattern(0);

    // DEBUG: Print carrier indices
    {
        char idx_buf[128] = "";
        int pos = 0;
        for (size_t i = 0; i < std::min(size_t(5), impl_->data_carrier_indices.size()); ++i) {
            pos += snprintf(idx_buf + pos, sizeof(idx_buf) - pos, "%d ", impl_->data_carrier_indices[i]);
        }
        LOG_DEMOD(DEBUG, "LTS TX carrier indices (first 5): %s(total %zu)", idx_buf, impl_->data_carrier_indices.size());
    }

    Samples training;
    training.reserve(static_cast<size_t>(std::max(count, 0)) * impl_->config.getSymbolDuration());

    // Generate LTS: known sequence on all carriers
    auto& lts_data = impl_->lts_data_scratch;
    lts_data.resize(impl_->data_carrier_indices.size());
    for (size_t i = 0; i < lts_data.size(); ++i) {
        lts_data[i] = impl_->sync_sequence[i % impl_->sync_sequence.size()];
    }

    // Generate each symbol separately to maintain mixer phase coherence
    // (complexToReal advances the mixer, so we can't reuse the same samples)
    // Note: createOFDMSymbol already adds the cyclic prefix
    for (int i = 0; i < count; ++i) {
        const auto& long_sym = impl_->createOFDMSymbol(lts_data, true);  // Returns CP + FFT samples
        const auto& lts_real = impl_->complexToReal(long_sym);
        training.insert(training.end(), lts_real.begin(), lts_real.end());

        // Add guard interval to match modulate() symbol structure
        // This ensures processPresynced() skips the exact right number of samples
        for (uint32_t g = 0; g < impl_->config.symbol_guard; ++g) {
            training.push_back(0);
            impl_->mixer.next();  // Keep mixer in sync
        }
    }

    return training;
}

Samples OFDMModulator::generateProbe() {
    // Channel probe: known sequence across all carriers
    // Used for channel quality measurement

    impl_->activateCarrierPattern(0);
    auto& probe_data = impl_->probe_data_scratch;
    probe_data.resize(impl_->data_carrier_indices.size());
    for (size_t i = 0; i < probe_data.size(); ++i) {
        // Chirp-like pattern for good correlation
        float phase = M_PI * i * i / probe_data.size();
        probe_data[i] = Complex(std::cos(phase), std::sin(phase));
    }

    const auto& complex_probe = impl_->createOFDMSymbol(probe_data, true);
    return impl_->complexToReal(complex_probe);
}

} // namespace ultra
