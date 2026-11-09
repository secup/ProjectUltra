// OFDMChirpWaveform - Implementation

#include "ofdm_chirp_waveform.hpp"
#include "ofdm/genie_tx_capture.hpp"
#include "fec/carrier_ldpc_interleaver.hpp"
#include "ultra/logging.hpp"
#include "ultra/dsp.hpp"  // FFT class is in here
#include "ultra/ofdm_link_adaptation.hpp"
#include "ultra/timing_profiler.hpp"
#include "ultra/phy_diagnostics.hpp"
#include "ofdm/pilot_pattern.hpp"
#include "ofdm/lts_cfo_update.hpp"
#include <algorithm>
#include <sstream>
#include <cmath>

namespace ultra {

namespace {
constexpr size_t LDPC_CODEWORD_BITS = 648;
constexpr size_t LDPC_CODEWORD_BYTES = LDPC_CODEWORD_BITS / 8;
constexpr size_t CARRIER_LDPC_MASK_CARRIERS = 59;
constexpr size_t CARRIER_LDPC_MIN_CODEWORDS = 2;
constexpr size_t CARRIER_LDPC_MAX_CODEWORDS = 8;
constexpr uint64_t ALL_ON_CARRIER_MASK = UINT64_MAX;

bool isSupportedChirpModulation(Modulation mod) {
    // Coherent-only OFDM (thread A 2026-05-31): differential modulations
    // (DBPSK/DQPSK/D8PSK) are no longer accepted on the OFDM_CHIRP path — they
    // live in MC-DPSK. Rejecting them here makes is_differential provably false
    // for every OFDM demod/channel-est path; a differential config falls back to
    // coherent QPSK (see the constructors and configure()).
    return mod == Modulation::QPSK ||
           mod == Modulation::BPSK ||
           mod == Modulation::QAM8 ||
           mod == Modulation::QAM16 ||
           mod == Modulation::QAM32 ||
           mod == Modulation::QAM64;
}

uint64_t activeBitsMask(size_t carriers) {
    return carriers >= 64 ? UINT64_MAX : ((uint64_t{1} << carriers) - 1);
}

bool isAllOnMask(uint64_t mask, size_t carriers) {
    const uint64_t active = activeBitsMask(carriers);
    return (mask & active) == active;
}

std::vector<size_t> buildDataLogicalCarrierIndices(const ModemConfig& config) {
    std::vector<size_t> logical_indices;
    logical_indices.reserve(config.num_carriers);

    const int neg_limit = static_cast<int>(config.num_carriers / 2);
    const int pos_limit = static_cast<int>((config.num_carriers + 1) / 2);

    int pilot_count = 0;
    size_t logical_carrier = 0;
    for (int i = -neg_limit; i <= pos_limit; ++i) {
        if (i == 0) {
            continue;
        }

        const bool is_pilot = config.use_pilots &&
            config.pilot_spacing != 0 &&
            (pilot_count % static_cast<int>(config.pilot_spacing) == 0);
        if (!is_pilot) {
            logical_indices.push_back(logical_carrier);
        }

        ++pilot_count;
        ++logical_carrier;
    }

    return logical_indices;
}

Bytes applyCarrierLdpcForward(const Bytes& encoded, size_t codeword_count) {
    const size_t total_bits = codeword_count * LDPC_CODEWORD_BITS;
    std::vector<uint8_t> in_bits(total_bits, 0);
    for (size_t i = 0; i < encoded.size() && i * 8 < total_bits; ++i) {
        for (int b = 0; b < 8 && i * 8 + static_cast<size_t>(b) < total_bits; ++b) {
            in_bits[i * 8 + static_cast<size_t>(b)] =
                static_cast<uint8_t>((encoded[i] >> (7 - b)) & 1u);
        }
    }

    const auto interleaver = fec::buildCarrierInterleaverV1(codeword_count);
    std::vector<uint8_t> out_bits(total_bits, 0);
    for (size_t i = 0; i < total_bits; ++i) {
        out_bits[interleaver[i]] = in_bits[i];
    }

    Bytes out((total_bits + 7) / 8, 0);
    for (size_t i = 0; i < total_bits; ++i) {
        if (out_bits[i]) {
            out[i / 8] |= static_cast<uint8_t>(1u << (7 - (i % 8)));
        }
    }
    return out;
}

std::vector<float> eraseMaskedCarrierLLRs(std::vector<float> air_llrs,
                                          const ModemConfig& config,
                                          uint64_t carrier_mask) {
    const std::vector<size_t> data_logical = buildDataLogicalCarrierIndices(config);
    if (data_logical.empty()) {
        return air_llrs;
    }

    const size_t bits_per_carrier = getBitsPerSymbol(config.modulation);
    for (size_t a = 0; a < air_llrs.size(); ++a) {
        const fec::AirGridIndex grid =
            fec::decomposeAirIndex(a, data_logical.size(), bits_per_carrier);
        if (grid.carrier >= data_logical.size()) {
            continue;
        }
        const size_t logical_carrier = data_logical[grid.carrier];
        if (logical_carrier < 64 &&
            (carrier_mask & (uint64_t{1} << logical_carrier)) == 0) {
            air_llrs[a] = 0.0f;
        }
    }
    return air_llrs;
}

std::vector<float> applyCarrierLdpcInverse(std::vector<float> air_llrs,
                                           size_t codeword_count) {
    const size_t total_bits = codeword_count * LDPC_CODEWORD_BITS;
    if (air_llrs.size() < total_bits) {
        return air_llrs;
    }

    const auto deinterleaver = fec::buildCarrierDeinterleaverV1(codeword_count);
    std::vector<float> out(total_bits, 0.0f);
    for (size_t air = 0; air < total_bits; ++air) {
        out[deinterleaver[air]] = air_llrs[air];
    }
    return out;
}
}

OFDMChirpWaveform::OFDMChirpWaveform() {
    // Default OFDM_CHIRP configuration
    config_.fft_size = 512;
    config_.num_carriers = 30;
    config_.modulation = Modulation::QPSK;  // Coherent-only OFDM (thread A 2026-05-31); differential lives in MC-DPSK
    config_.code_rate = CodeRate::R1_2;
    configurePilotsForCodeRate(config_.code_rate);
    initComponents();
}

OFDMChirpWaveform::OFDMChirpWaveform(const ModemConfig& config)
    : config_(config)
{
    // Coherent-only OFDM (thread A 2026-05-31): default any unsupported modulation to
    // coherent QPSK (was DQPSK). Differential lives in MC-DPSK; OFDM_NARROW (the last
    // OFDM-DQPSK user) is disabled and returns later as coherent.
    if (!isSupportedChirpModulation(config_.modulation)) {
        config_.modulation = Modulation::QPSK;
    }
    configurePilotsForCodeRate(config_.code_rate);
    initComponents();
}

OFDMChirpWaveform::OFDMChirpWaveform(const ModemConfig& config, protocol::WaveformMode mode)
    : mode_(mode), config_(config)
{
    // Coherent-only OFDM (thread A 2026-05-31): default any unsupported modulation to
    // coherent QPSK (was DQPSK). Differential lives in MC-DPSK; OFDM_NARROW (the last
    // OFDM-DQPSK user) is disabled and returns later as coherent.
    if (!isSupportedChirpModulation(config_.modulation)) {
        config_.modulation = Modulation::QPSK;
    }
    configurePilotsForCodeRate(config_.code_rate);
    initComponents();
}

void OFDMChirpWaveform::initComponents() {
    modulator_ = std::make_unique<OFDMModulator>(config_);
    demodulator_ = std::make_unique<OFDMDemodulator>(config_);
    demodulator_->setRXCarrierErasureEnabled(rx_carrier_erasure_enabled_);
    // LATENT-BUG FIX (found while tracing BUG-BURST-STALE-GEOMETRY, 2026-07-28):
    // configure() rebuilds the demodulator but ldpc_lifting_z_ SURVIVES on the
    // waveform. Without re-asserting it here the fresh demodulator silently reverts
    // its codeword gate to 648 bits while getMinSamplesForCWCount still sizes frames
    // for n=1944 — the burst deinterleaver then gets 2x648 where it expects 2x1944
    // and throws, dropping the group. Inert at the default z=27 (opt-in ULTRA_LDPC_Z),
    // but the commanded-geometry path sets z at a group marker and configures one
    // processBuffer later, which sits exactly on this.
    demodulator_->setActiveLDPCBlockSize(ldpc_lifting_z_ == 81 ? 1944 : 648);
    chirp_sync_ = std::make_unique<sync::ChirpSync>(getChirpConfig());
    if (shortAnchorEnabled() && mode_ == protocol::WaveformMode::OFDM_CHIRP) {
        // Phase 2a: a SECOND short-config ChirpSync (short DUAL chirp, 250+50+250+50) for the warm
        // BURST_HEADER descriptor. Kept separate so cold PING/CONNECT acquisition keeps the full
        // 500 ms dual chirp. Both TX generate and RX detect route through this instance.
        short_anchor_chirp_sync_ = std::make_unique<sync::ChirpSync>(getShortAnchorChirpConfig());
    }
    invalidateDataSyncTemplate();
}

void OFDMChirpWaveform::invalidateDataSyncTemplate() {
    data_sync_template_analytic_.clear();
    data_sync_template_energy_ = 0.0f;
    data_sync_template_symbol_samples_ = 0;
}

bool OFDMChirpWaveform::ensureDataSyncTemplate(int symbol_samples) {
    const size_t template_samples = static_cast<size_t>(symbol_samples) * 2;
    if (symbol_samples <= 0 || template_samples == 0) {
        return false;
    }

    if (data_sync_template_symbol_samples_ == symbol_samples &&
        data_sync_template_analytic_.size() == template_samples &&
        data_sync_template_energy_ > 0.0f) {
        return true;
    }

    ModemConfig template_config = config_;
    template_config.tx_cfo_hz = 0.0f;
    OFDMModulator template_modulator(template_config);
    Samples lts_template = template_modulator.generateTrainingSymbols(2);
    if (lts_template.size() < template_samples) {
        invalidateDataSyncTemplate();
        return false;
    }

    HilbertTransform template_hilbert(65);
    template_hilbert.process(
        SampleSpan(lts_template.data(), template_samples),
        data_sync_template_analytic_);

    data_sync_template_energy_ = 0.0f;
    for (const auto& s : data_sync_template_analytic_) {
        data_sync_template_energy_ += std::norm(s);
    }
    data_sync_template_symbol_samples_ = symbol_samples;

    if (data_sync_template_energy_ <= 1.0e-8f) {
        invalidateDataSyncTemplate();
        return false;
    }
    return true;
}

sync::ChirpConfig OFDMChirpWaveform::getChirpConfig() const {
    sync::ChirpConfig cfg;
    cfg.sample_rate = static_cast<float>(config_.sample_rate);
    cfg.f_start = config_.chirp_f_start;      // 300 Hz (wide) or 1250 Hz (narrow)
    cfg.f_end = config_.chirp_f_end;          // 2700 Hz (wide) or 1750 Hz (narrow)
    cfg.duration_ms = config_.chirp_duration_ms; // 500ms (wide) or 1000ms (narrow)
    cfg.gap_ms = 100.0f;
    cfg.use_dual_chirp = true;  // For CFO estimation
    cfg.tx_cfo_hz = config_.tx_cfo_hz;  // Pass TX CFO for simulation
    return cfg;
}

float OFDMChirpWaveform::shortAnchorChirpMs() {
    // ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS: per-chirp duration (ms) of the warm BURST_HEADER
    // descriptor's SHORT DUAL anchor. 0/unset = OFF (full 500 ms dual anchor, byte-identical).
    // A/B value = 250: a 250+50+250+50 dual (vs the full 500+100+500+100) reclaims ~600 ms/burst
    // while KEEPING CFO/timing separation and fade robustness (see getShortAnchorChirpConfig).
    // Clamp [50,600].
    static const float ms = []() {
        if (const char* env = std::getenv("ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS")) {
            const float v = static_cast<float>(std::atof(env));
            if (v >= 50.0f && v <= 600.0f) return v;
        }
        return 0.0f;
    }();
    return ms;
}

bool OFDMChirpWaveform::shortAnchorEnabled() const {
    return shortAnchorChirpMs() > 0.0f;
}

sync::ChirpConfig OFDMChirpWaveform::getShortAnchorChirpConfig() const {
    sync::ChirpConfig cfg = getChirpConfig();
    // Short DUAL chirp (keep up+down). A single chirp couples CFO and timing — its peak shifts by
    // ~CFO * Fs/chirp_rate samples with no way to separate the two (chirp_sync.hpp:201-203) — so it
    // mislocates the descriptor under residual CFO and is fragile on fading. The dual differencing
    // keeps timing CFO-immune (up/down shifts cancel) and still MEASURES the CFO. Halving the
    // per-chirp duration costs ~3 dB of matched-filter gain, affordable against the warm
    // descriptor's large SNR surplus over the -5 dB cold floor; CFO readout coarsens to
    // ~Fs*T/B samples/Hz but the per-frame LTS residual refines it to 0.3 Hz. Trim the inter-chirp
    // gap to 50 ms (it only separates the two correlation windows) so the short dual reclaims
    // ~the same airtime a single chirp would have, without the single-chirp fragility.
    cfg.use_dual_chirp = true;
    cfg.duration_ms = shortAnchorChirpMs();  // per-chirp duration (250 ms in the A/B)
    cfg.gap_ms = 50.0f;
    return cfg;
}


WaveformCapabilities OFDMChirpWaveform::getCapabilities() const {
    WaveformCapabilities caps;
    caps.supports_cfo_correction = true;    // Via dual chirp
    caps.supports_doppler_correction = true; // Pilot + DD tracking
    caps.requires_pilots = config_.use_pilots;
    caps.supports_differential = false;     // Coherent-only OFDM (thread A); differential lives in MC-DPSK
    caps.min_snr_db = 10.0f;                // Lower than Schmidl-Cox
    caps.max_snr_db = 20.0f;                // Above this, prefer a higher-order modulation
    caps.max_throughput_bps = getThroughput(CodeRate::R2_3);

    // Chirp preamble + training
    caps.preamble_duration_ms = chirp_sync_ ? (chirp_sync_->getTotalSamples() * 1000.0f / config_.sample_rate)
                                            : 1200.0f;

    return caps;
}

void OFDMChirpWaveform::configurePilotsForCodeRate(CodeRate rate) {
    config_.use_pilots = true;
    config_.pilot_spacing =
        ofdm_link_adaptation::recommendedPilotSpacing(config_.modulation, rate);
}

bool OFDMChirpWaveform::carrierLdpcPlumbingEligible() const {
    return mode_ == protocol::WaveformMode::OFDM_CHIRP &&
           config_.num_carriers == CARRIER_LDPC_MASK_CARRIERS &&
           (config_.fft_size == 512 || config_.fft_size == 1024);
}

bool OFDMChirpWaveform::carrierLdpcCodewordCountSupported(size_t codeword_count) const {
    return codeword_count >= CARRIER_LDPC_MIN_CODEWORDS &&
           codeword_count <= CARRIER_LDPC_MAX_CODEWORDS;
}

void OFDMChirpWaveform::configure(Modulation mod, CodeRate rate) {
    // Coherent-only OFDM: any unsupported modulation (incl. differential) falls
    // back to coherent QPSK, consistent with the constructors.
    if (!isSupportedChirpModulation(mod)) {
        LOG_MODEM(WARN, "OFDMChirpWaveform: Unsupported modulation %d, using QPSK",
                  static_cast<int>(mod));
        mod = Modulation::QPSK;
    }

    config_.modulation = mod;
    config_.code_rate = rate;
    configurePilotsForCodeRate(rate);

    // Reinitialize with new config
    initComponents();

    const int pilot_count = config_.use_pilots
        ? ofdm_link_adaptation::pilotCount(static_cast<int>(config_.num_carriers),
                                           static_cast<int>(config_.pilot_spacing))
        : 0;
    const int data_carriers = ofdm_link_adaptation::dataCarrierCount(
        static_cast<int>(config_.num_carriers),
        config_.use_pilots,
        static_cast<int>(config_.pilot_spacing));

    LOG_MODEM(INFO, "OFDMChirpWaveform: configured for %s %s (%d data, %d pilots)",
              modulationToString(mod), codeRateToString(rate),
              data_carriers, pilot_count);
}

void OFDMChirpWaveform::setCfoPriorCertified(bool certified) {
    if (!demodulator_) return;
    if (certified) {
        demodulator_->setCfoPriorSigmaHz(0.0);
        return;
    }
    // Uncertified: the dual-chirp multipath-phantom sigma for THIS waveform's chirp and CP.
    const sync::ChirpConfig chirp = getChirpConfig();
    demodulator_->setCfoPriorSigmaHz(lts_cfo::uncertifiedPriorSigmaHz(
        static_cast<double>(config_.getCyclicPrefix()), static_cast<double>(chirp.sample_rate),
        static_cast<double>(chirp.f_start), static_cast<double>(chirp.f_end),
        static_cast<double>(chirp.duration_ms) / 1000.0));
}

void OFDMChirpWaveform::setFrequencyOffset(float cfo_hz) {
    cfo_hz_ = cfo_hz;
    if (demodulator_) {
        demodulator_->setFrequencyOffset(cfo_hz);
    }
}

void OFDMChirpWaveform::setTxFrequencyOffset(float cfo_hz) {
    // Set TX CFO on chirp sync and modulator for simulation
    config_.tx_cfo_hz = cfo_hz;

    // Reinitialize with new config to apply TX CFO
    initComponents();

    LOG_MODEM(INFO, "OFDMChirpWaveform: TX CFO set to %.1f Hz", cfo_hz);
}

Samples OFDMChirpWaveform::generatePreamble() {
    if (!chirp_sync_ || !modulator_) {
        return Samples();
    }

    // Generate: [CHIRP][TRAINING_SYMBOLS]
    Samples chirp = chirp_sync_->generate();
    Samples training = modulator_->generateTrainingSymbols(2);

    Samples preamble;
    preamble.reserve(chirp.size() + training.size());
    preamble.insert(preamble.end(), chirp.begin(), chirp.end());
    preamble.insert(preamble.end(), training.begin(), training.end());

    return preamble;
}

Samples OFDMChirpWaveform::generateShortAnchorPreamble() {
    // Phase 2a warm re-anchor: [SHORT dual chirp][2 LTS]. Falls back to the full anchor
    // if the short instance isn't built (feature off). The 2 LTS are unchanged, so the RX
    // channel estimate / burst-marker detection are identical to the full anchor.
    if (!short_anchor_chirp_sync_ || !modulator_) {
        return generatePreamble();
    }
    Samples chirp = short_anchor_chirp_sync_->generate();
    Samples training = modulator_->generateTrainingSymbols(2);
    Samples preamble;
    preamble.reserve(chirp.size() + training.size());
    preamble.insert(preamble.end(), chirp.begin(), chirp.end());
    preamble.insert(preamble.end(), training.begin(), training.end());
    return preamble;
}

Samples OFDMChirpWaveform::generateSettlingSymbols(size_t count) {
    if (!modulator_ || count == 0) {
        return Samples();
    }
    // Enough bits for `count` symbols at any profile (all carriers x 4 bits), fixed
    // SplitMix64 content: deterministic on every platform. The modulator resets its pilot
    // pattern per call, so this cannot shift the pattern phase of the frame that follows.
    const size_t symbol_samples = static_cast<size_t>(getSamplesPerSymbol());
    Bytes bytes((count * config_.num_carriers * 4 + 7) / 8);
    uint64_t x = 0x5E771E5EEDC0FFEEULL;
    for (auto& b : bytes) {
        x += 0x9E3779B97F4A7C15ULL;
        uint64_t z = x;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        b = static_cast<uint8_t>((z ^ (z >> 31)) & 0xFFu);
    }
    Samples s = modulator_->modulate(ByteSpan(bytes.data(), bytes.size()), config_.modulation);
    if (s.size() > count * symbol_samples) {
        s.resize(count * symbol_samples);
    }
    return s;
}

Samples OFDMChirpWaveform::generateDataPreamble() {
    if (!modulator_) {
        return Samples();
    }

    // Light preamble: just training symbols (no chirp)
    // Saves ~1.2 seconds per frame when already connected
    // Receiver uses known CFO from previous frames
    return modulator_->generateTrainingSymbols(2);
}


Samples OFDMChirpWaveform::modulate(const Bytes& encoded_data) {
    if (!modulator_) {
        return Samples();
    }

    Bytes tx_data = encoded_data;
    uint64_t effective_mask = ALL_ON_CARRIER_MASK;
    bool mask_enabled = false;

    const size_t codeword_count = encoded_data.size() / LDPC_CODEWORD_BYTES;
    const bool full_codewords =
        !encoded_data.empty() && (encoded_data.size() % LDPC_CODEWORD_BYTES == 0);
    const bool eligible = carrierLdpcPlumbingEligible() && full_codewords;
    const bool masked_carriers =
        !isAllOnMask(carrier_mask_, CARRIER_LDPC_MASK_CARRIERS);
    const bool carrier_ldpc_active = eligible &&
        codeword_count != 1 &&
        carrierLdpcCodewordCountSupported(codeword_count) &&
        (carrier_ldpc_interleaver_enabled_ || masked_carriers);

    if (eligible && codeword_count == 1) {
        // Ncw=1 DQPSK can put a one-carrier erasure into only 5 LDPC base
        // columns; sub-phase 3 therefore forces all-on and keeps 1-CW PHY
        // headers/control frames bit-identical to the legacy ordering.
        effective_mask = ALL_ON_CARRIER_MASK;
    } else if (carrier_ldpc_active) {
        // CarrierLDPC is the final TX bit permutation before the air grid.
        // It is mandatory when RX may insert carrier erasures, and remains
        // enabled for legacy explicit mask tests. Direct waveform callers keep
        // the default false flag, preserving deterministic all-on byte identity.
        tx_data = applyCarrierLdpcForward(encoded_data, codeword_count);
        if (masked_carriers) {
            effective_mask = carrier_mask_;
            mask_enabled = true;
        }
    }

    ByteSpan span(tx_data.data(), tx_data.size());
    return modulator_->modulate(span, config_.modulation, effective_mask, mask_enabled);
}

// =============================================================================
// POST-FEC DATA-AIDED CHANNEL ESTIMATION (ULTRA_ITERATIVE_CHEST, default OFF)
// See src/ofdm/iterative_chest.hpp for the rationale and the adaptivity argument.
// =============================================================================

Symbol OFDMChirpWaveform::remodulateDataCarrierSymbols(const Bytes& encoded_air_bytes) {
    if (!modulator_ || encoded_air_bytes.empty()) {
        return Symbol{};
    }
    // Replay the PRODUCTION transmit path — the same modulate() the sender used, so
    // the CarrierLDPC eligibility predicate, the carrier mask, the scattered-pilot
    // rotation and mapBits() cannot drift from the transmitter by construction. A
    // second, parallel implementation of the bit->carrier mapping here is exactly
    // the BUG-8PSK-001 failure mode (a private copy that silently diverges), so
    // there is not one. The generated audio is discarded; the grid is captured
    // before the IFFT, hence it is independent of the modulator's NCO phase.
    // Suppress the diagnostic TX capture — this is a RECEIVE-side replay and must not
    // be pushed as a transmitted symbol. Guarded so that in production (genie off,
    // always) the receive thread performs NO write to the process-global at all.
    const bool genie_was_enabled = ultra::genie::txCapture().enabled;
    if (genie_was_enabled) ultra::genie::txCapture().enabled = false;
    (void)modulate(encoded_air_bytes);
    if (genie_was_enabled) ultra::genie::txCapture().enabled = true;
    return modulator_->getLastDataCarrierSymbolsForTesting();
}

void OFDMChirpWaveform::setDataAidedFeedbackEnabled(bool enabled) {
    data_aided_feedback_enabled_ = enabled;
    if (demodulator_) {
        demodulator_->setDataAidedFeedbackEnabled(enabled);
    }
}

void OFDMChirpWaveform::setChannelHistoryFrameOrigin(long long abs_sample) {
    if (demodulator_) {
        demodulator_->setChannelHistoryFrameOrigin(abs_sample);
    }
    data_aided_frame_origin_ = abs_sample;
}

void OFDMChirpWaveform::armChannelHistoryCarry(bool armed) {
    if (demodulator_) {
        demodulator_->armChannelHistoryCarry(armed);
    }
}

void OFDMChirpWaveform::clearChannelHistory() {
    if (demodulator_) {
        demodulator_->clearChannelHistory();
    }
    data_aided_frame_origin_ = -1;
}

size_t OFDMChirpWaveform::ingestDataAidedFrame(const Bytes& encoded_air_bytes) {
    if (!data_aided_feedback_enabled_ || !demodulator_ || encoded_air_bytes.empty()) {
        return 0;
    }
    const Symbol x_grid = remodulateDataCarrierSymbols(encoded_air_bytes);
    if (x_grid.empty()) return 0;
    return demodulator_->ingestDataAidedGrid(x_grid, data_aided_frame_origin_);
}

void OFDMChirpWaveform::setCarrierMask(uint64_t active_mask) {
    carrier_mask_ = active_mask;
}

void OFDMChirpWaveform::setCarrierLdpcInterleaverEnabled(bool enabled) {
    carrier_ldpc_interleaver_enabled_ = enabled;
}

bool OFDMChirpWaveform::detectSync(SampleSpan samples, SyncResult& result, float threshold) {
    if (!chirp_sync_) {
        return false;
    }

    burst_interleave_latched_ = false;
    burst_interleaved_detected_ = false;

    // Full dual detector FIRST — correct for cold acquisition and session-first/resend full
    // anchors. It cleanly FAILS on a short single chirp (no down-chirp/gap to match) rather than
    // mislocating one, so trying it first is safe for both anchor types.
    // A streaming search window can contain a weaker, complete BURST_HEADER
    // anchor followed by the stronger full anchor of its DATA group.  The
    // descriptor is the authority for modulation/rate/LDPC-Z, so skipping it
    // makes the marked DATA LTS get decoded as the hardened control profile.
    // Consume the earliest complete pair; light-LTS acquisition is a separate
    // path and is unaffected.
    auto chirp_result =
        chirp_sync_->detectDualChirpWithEarlierRecovery(samples, threshold);
    bool short_anchor = false;
    sync::ChirpSync* cs = chirp_sync_.get();
    // Phase 2a: on a full-detector MISS, the descriptor may be the SHORT single up-chirp warm
    // re-anchor. Fall back to the short detector (a 2nd correlation only on the miss path).
    // No per-burst signaling needed: full-anchor descriptors match the full detector, short ones
    // match here. (The 2 LTS after either chirp re-validate timing identically.)
    if (!chirp_result.success && short_anchor_chirp_sync_) {
        // #62: a SHORT dual chirp has less time-bandwidth product than the full one, so its
        // matched-filter peak is intrinsically lower — using the FULL detector's threshold here
        // under-detects it (the measured ~24% miss at 250 ms on the rig, whose resends eat the
        // airtime the short chirp saved). ULTRA_SHORT_CHIRP_DETECT_SCALE (default 1.0 = byte-identical)
        // lowers JUST this fallback threshold. Safe-ish: this branch only runs after the full detector
        // already MISSED (so there's no full chirp here), and a false short-detect merely mis-anchors
        // one group -> resend -> the reactive revert (bounded), never a stall.
        static const float kShortDetectScale = [] {
            const char* e = std::getenv("ULTRA_SHORT_CHIRP_DETECT_SCALE");
            return (e && *e) ? std::clamp(static_cast<float>(std::atof(e)), 0.3f, 1.0f) : 1.0f;
        }();
        auto short_result = short_anchor_chirp_sync_->detectDualChirpWithEarlierRecovery(
            samples, threshold * kShortDetectScale);
        if (short_result.success) {
            chirp_result = short_result;
            cs = short_anchor_chirp_sync_.get();
            short_anchor = true;
        }
    }

    result.detected = chirp_result.success;
    result.correlation = std::max(chirp_result.up_correlation, chirp_result.down_correlation);
    result.cfo_hz = chirp_result.cfo_hz;
    result.gap_error_samples = chirp_result.gap_error_samples;
    result.has_training = true;

    if (chirp_result.success) {
        synced_ = true;
        last_cfo_ = chirp_result.cfo_hz;

        // Calculate where TRAINING starts (process() needs training for channel estimation)
        // Layout: [UP-CHIRP][GAP][DOWN-CHIRP][GAP][TRAINING_SYMBOLS][DATA...]
        //                                         ^-- start_sample points here
        //
        // IMPORTANT: Use down_chirp position for training_start calculation!
        // With CFO, up_chirp and down_chirp positions shift in OPPOSITE directions:
        //   up_chirp: shifts by -CFO × cfo_to_samples
        //   down_chirp: shifts by +CFO × cfo_to_samples
        // Using up_chirp_start + fixed_offset gives growing error with CFO.
        // Using down_chirp_start gives more accurate training position.
        size_t chirp_samples = cs->getChirpSamples();
        // Gap is per-instance: full anchor 100 ms, short warm anchor 50 ms. Read it from the
        // matched chirp config so the [down-chirp][gap] -> training offset is exact for both.
        size_t gap_samples = static_cast<size_t>(config_.sample_rate * cs->getConfig().gap_ms / 1000.0f);

        // Training starts after [down-chirp][gap]. Both the full anchor and the short warm anchor
        // are DUAL chirps, so down_chirp_start is CFO-robust (up/down peak shifts cancel) for both.
        result.start_sample = chirp_result.down_chirp_start + chirp_samples + gap_samples;
        result.preamble_start_sample = chirp_result.up_chirp_start;
        // NOTE: Do NOT add training_samples - process() needs them for channel estimation

        // Store training start position for CFO phase calculation in process()
        training_start_sample_ = result.start_sample;

        const int symbol_samples = getSamplesPerSymbol();
        const size_t training_start = static_cast<size_t>(result.start_sample);
        if (symbol_samples > 0 &&
            training_start + static_cast<size_t>(symbol_samples * 2) <= samples.size()) {
            data_sync_hilbert_.reset();
            auto& analytic = data_sync_analytic_scratch_;
            data_sync_hilbert_.process(samples, analytic);

            if (training_start + static_cast<size_t>(symbol_samples * 2) <= analytic.size()) {
                Complex marker_p(0.0f, 0.0f);
                for (int n = 0; n < symbol_samples; ++n) {
                    const size_t idx1 = training_start + static_cast<size_t>(n);
                    const size_t idx2 = idx1 + static_cast<size_t>(symbol_samples);
                    marker_p += std::conj(analytic[idx1]) * analytic[idx2];
                }

                const float cfo_phase =
                    2.0f * static_cast<float>(M_PI) * chirp_result.cfo_hz *
                    static_cast<float>(symbol_samples) / config_.sample_rate;
                const Complex cfo_comp(std::cos(-cfo_phase), std::sin(-cfo_phase));
                const Complex marker_metric = marker_p * cfo_comp;
                burst_interleaved_detected_ = (marker_metric.real() < 0.0f);
                burst_interleave_latched_ = burst_interleaved_detected_;
                if (ultra::phyDiagnosticsEnabled()) {
                    std::ostringstream oss;
                    oss << "burst_marker site=chirp re=" << marker_metric.real()
                        << " im=" << marker_metric.imag()
                        << " latched=" << (burst_interleave_latched_ ? 1 : 0);
                    ultra::phyDiagLine(oss.str());
                }
            }
        }

        LOG_MODEM(INFO,
                  "OFDMChirpWaveform: Chirp detected at %d, CFO=%.1f Hz, training_start=%d%s%s",
                  chirp_result.up_chirp_start, chirp_result.cfo_hz, result.start_sample,
                  short_anchor ? " [SHORT-ANCHOR]" : "",
                  burst_interleaved_detected_ ? " [BURST-INTERLEAVED]" : "");
    }

    return result.detected;
}

namespace {
// Gamma(K, 1) survival for integer K: P(chi >= T) = e^-T * sum_{i<K} T^i / i!. Bisection.
double gammaSurvival(int K, double T) {
    double term = 1.0, sum = 1.0;
    for (int i = 1; i < K; ++i) {
        term *= T / i;
        sum += term;
    }
    return std::exp(-T) * sum;
}
double gammaSurvivalThreshold(int K, double p_fa) {
    double lo = 0.0, hi = 400.0;
    for (int it = 0; it < 200; ++it) {
        const double mid = 0.5 * (lo + hi);
        if (gammaSurvival(K, mid) > p_fa) lo = mid; else hi = mid;
    }
    return hi;
}
inline Complex unitOf(Complex z) {
    const float m = std::abs(z);
    return m > 1.0e-20f ? z / m : Complex(0.0f, 0.0f);
}

constexpr int kStructSyms = 9;      // LTS1, LTS2 + data symbols 0..6
constexpr int kStructLagGroups = 7;  // lags 2..8 (LTS -> data-pilot pairs)

// Known-pair template for one frame geometry. Pairs are (LTS symbol a, data symbol k) on a
// carrier that is a PILOT in data symbol k's pattern but a ZADOFF-CHU cell in the LTS, so the
// known product q = pilot * conj(ZC) is pseudo-random across carriers. That is what makes the
// statistic immune to STATIONARY signals (a CW tone has the same lagged phase on every bin;
// multiplying by conj(q) scatters it), the role per-cell scrambling plays in preamble-less designs.
// LTS1->LTS2 and pilot->pilot pairs (q = 1) are deliberately excluded for that reason.
struct StructPair {
    uint8_t s_lo, s_hi;
    uint16_t l;
    Complex qconj;
};
struct StructTemplate {
    int pilot_spacing = -1;
    std::vector<StructPair> groups[kStructLagGroups];
    double n[kStructLagGroups] = {};
    std::vector<Complex> x_lts;  // LTS value per logical carrier (abstract convention)
};

StructTemplate buildStructTemplate(const ModemConfig& g, const std::vector<int>& bins) {
    StructTemplate t;
    t.pilot_spacing = static_cast<int>(g.pilot_spacing);
    // LTS values in the receiver's own FFT convention, from the modulator's real output.
    OFDMModulator mod(g);
    Samples lts = mod.generateTrainingSymbols(1);
    const int N = static_cast<int>(g.fft_size);
    const int cp = static_cast<int>(g.getCyclicPrefix());
    if (static_cast<int>(lts.size()) < cp + N) return t;
    FFT fft(static_cast<size_t>(N));
    std::vector<Complex> in(static_cast<size_t>(N)), out(static_cast<size_t>(N));
    for (int n = 0; n < N; ++n) in[static_cast<size_t>(n)] = Complex(lts[static_cast<size_t>(cp + n)], 0.0f);
    fft.forward(in, out);
    const size_t nc = g.num_carriers;
    std::vector<Complex> x_fft(nc);
    for (size_t l = 0; l < nc; ++l) x_fft[l] = out[static_cast<size_t>(bins[l])];
    // Common convention constant c (scale * mixer phase), observed on pattern-0 pilots whose
    // abstract value is pilotSymbolForLogical(l).
    Complex c(0, 0);
    std::vector<uint8_t> p0(nc, 0);
    for (size_t l = 0; l < nc; ++l) {
        if (ofdm_pilots::isPilotLogical(g, l, 0)) {
            p0[l] = 1;
            c += x_fft[l] * std::conj(ofdm_pilots::pilotSymbolForLogical(l));
        }
    }
    if (std::abs(c) < 1.0e-12f) return t;
    t.x_lts.resize(nc);
    for (size_t l = 0; l < nc; ++l) t.x_lts[l] = x_fft[l] / c;
    for (int k = 0; k + 2 < kStructSyms; ++k) {
        if ((k % std::max(1, t.pilot_spacing)) == 0) continue;  // pattern 0: pilots equal LTS pilots
        for (size_t l = 0; l < nc; ++l) {
            if (p0[l] || !ofdm_pilots::isPilotLogical(g, l, static_cast<size_t>(k))) continue;
            const Complex x_lts = x_fft[l] / c;
            const Complex q = ofdm_pilots::pilotSymbolForLogical(l) * std::conj(x_lts);
            for (int a = 0; a < 2; ++a) {
                const int s_hi = k + 2;
                const int lag = s_hi - a;
                if (lag < 2 || lag > 8) continue;
                t.groups[lag - 2].push_back({static_cast<uint8_t>(a), static_cast<uint8_t>(s_hi),
                                             static_cast<uint16_t>(l), std::conj(unitOf(q))});
            }
        }
    }
    for (int gi = 0; gi < kStructLagGroups; ++gi) t.n[gi] = static_cast<double>(t.groups[gi].size());
    return t;
}
}  // namespace

// PREAMBLE-LESS STRUCTURE ACQUISITION (ULTRA_BURST_ACQ=structure; Stage 2 of the 2026-11-08
// speed plan). Finds a connected-mode light frame — in particular the burst
// descriptor that opens every group — WITHOUT a chirp. Called only for the connected
// GROUP-START search (StreamingDecoder::searchForSync, where the dual-chirp search runs in
// chirp mode); in-group frames keep detectDataSync's warm light-LTS lock.
//
// Statistic. For a same-carrier pair of KNOWN cells, the unit lagged product
// u = unit(Y_hi * conj(Y_lo)) * conj(q), q = X_hi * conj(X_lo), equals e^{j(common phase)}:
// per-carrier channel gain/phase, a timing offset inside the CP and a constant CFO cancel or
// become one common phase per lag. Summing coherently per lag group L,
//   S_L = sum u,   chi = sum_L |S_L|^2 / n_L,
// gives chi ~ Gamma(K = #lag groups, 1) when no aligned frame is present, and chi ~ n * rho^2
// when one is (rho = per-cell coherence; unit normalization makes it amplitude-, AGC- and
// fade-depth-invariant). Pairs: LTS (Zadoff-Chu) cell -> data-pilot cell (see
// buildStructTemplate for why only these). Templates: the control profile (every burst
// descriptor) and the current data geometry (light data frames); the max over both is tested.
// Threshold: Gamma(K) inverse survival at the per-test probability false_alarm_budget /
// (candidates x templates) — the caller's per-call budget split by the union bound over every
// test this call makes; a probability law, not a tuned threshold.
//
// Search: the WHOLE window on a hop grid of Ls/20; FFTs are shared across candidates spaced by
// one symbol. The earliest crossing wins (the first frame in the window), then the local
// maximum within one symbol, then a 4-sample fine search whose plateau MIDDLE is reported: the
// plateau spans the cyclic prefix, so its middle centers the FFT window in the ISI-free region.
size_t OFDMChirpWaveform::frameStructureSpanSamples() const {
    // One whole template (LTS pair + 7 data symbols) plus one symbol of timing refinement.
    return static_cast<size_t>(kStructSyms + 1) * static_cast<size_t>(getSamplesPerSymbol());
}

bool OFDMChirpWaveform::detectFrameStructure(SampleSpan samples, SyncResult& result,
                                             float known_cfo_hz, double false_alarm_budget) {
    timing::ScopedTimer _profile_(timing::globalDecoderProfile().detect_data_sync);
    result.detected = false;
    result.correlation = 0.0f;
    result.cfo_hz = known_cfo_hz;
    result.has_training = true;
    burst_interleave_latched_ = false;
    burst_interleaved_detected_ = false;

    const int Ls = getSamplesPerSymbol();
    const int N = static_cast<int>(config_.fft_size);
    const int cp = static_cast<int>(config_.getCyclicPrefix());
    if (Ls <= N || static_cast<int>(samples.size()) < kStructSyms * Ls + Ls) {
        return false;
    }
    data_sync_hilbert_.reset();
    auto& analytic = data_sync_analytic_scratch_;
    data_sync_hilbert_.process(samples, analytic);
    const int L = static_cast<int>(analytic.size());

    const size_t nc = config_.num_carriers;
    const float bin_hz = static_cast<float>(config_.sample_rate) / static_cast<float>(N);
    const int fc_bin = static_cast<int>(std::lround(config_.center_freq / bin_hz));
    std::vector<int> bins(nc);
    for (size_t l = 0; l < nc; ++l) {
        const int f = ofdm_pilots::fftIndexForLogical(config_, l);
        bins[l] = fc_bin + ((f <= N / 2) ? f : f - N);
    }
    // Templates (cached per pilot spacing): control profile + current data geometry.
    static thread_local std::vector<StructTemplate> cache;
    auto templateFor = [&](int spacing, Modulation mod) -> const StructTemplate& {
        for (const auto& t : cache) {
            if (t.pilot_spacing == spacing) return t;
        }
        ModemConfig g = config_;
        g.modulation = mod;
        g.use_pilots = true;
        g.pilot_spacing = static_cast<uint32_t>(spacing);
        cache.push_back(buildStructTemplate(g, bins));
        return cache.back();
    };
    const int ctrl_sp = ofdm_link_adaptation::recommendedPilotSpacing(Modulation::QPSK, CodeRate::R1_4);
    std::vector<const StructTemplate*> tpls;
    tpls.push_back(&templateFor(ctrl_sp, Modulation::QPSK));
    if (config_.use_pilots && static_cast<int>(config_.pilot_spacing) != ctrl_sp &&
        ofdm_link_adaptation::isCoherentModulation(config_.modulation)) {
        // Re-fetch the control template pointer after a possible reallocation.
        const StructTemplate& data_t = templateFor(static_cast<int>(config_.pilot_spacing), config_.modulation);
        tpls.clear();
        tpls.push_back(&templateFor(ctrl_sp, Modulation::QPSK));
        tpls.push_back(&data_t);
    }

    if (!structure_fft_ || structure_fft_in_.size() != static_cast<size_t>(N)) {
        structure_fft_ = std::make_unique<FFT>(static_cast<size_t>(N));
        structure_fft_in_.assign(static_cast<size_t>(N), Complex(0, 0));
        structure_fft_out_.assign(static_cast<size_t>(N), Complex(0, 0));
    }
    const double w_cfo = -2.0 * M_PI * static_cast<double>(known_cfo_hz) / config_.sample_rate;
    const bool rotate = std::abs(known_cfo_hz) > 1.0e-3f;
    auto carriersAt = [&](int w, std::vector<Complex>& out) -> bool {
        out.assign(nc, Complex(0, 0));
        if (w < 0 || w + N > L) return false;
        if (rotate) {
            Complex ph(static_cast<float>(std::cos(w_cfo * w)), static_cast<float>(std::sin(w_cfo * w)));
            const Complex step(static_cast<float>(std::cos(w_cfo)), static_cast<float>(std::sin(w_cfo)));
            for (int n = 0; n < N; ++n) {
                structure_fft_in_[static_cast<size_t>(n)] = analytic[static_cast<size_t>(w + n)] * ph;
                ph *= step;
            }
        } else {
            std::copy(analytic.begin() + w, analytic.begin() + w + N, structure_fft_in_.begin());
        }
        structure_fft_->forward(structure_fft_in_, structure_fft_out_);
        for (size_t l = 0; l < nc; ++l) out[l] = structure_fft_out_[static_cast<size_t>(bins[l])];
        return true;
    };
    // Two marker hypotheses. A group-start frame carries the burst marker: LTS1 negated
    // (markFirstLTSSymbolForBurstGroup). A lag group mixes LTS1-referenced (a=0) and
    // LTS2-referenced (a=1) pairs, so on a marked frame the a=0 terms flip sign and CANCEL the
    // a=1 terms in a single coherent sum (measured: the marked group head was missed while the
    // unmarked frame after it locked at chi=92). The marker is a known binary modulation of
    // LTS1, so both hypotheses are tested — chi(+) = sum |B+A|^2/n, chi(-) = sum |B-A|^2/n with
    // A, B the a=0 / a=1 partial sums — and the union bound counts both. The winning hypothesis
    // is also the marker decision (it uses every pair, not just LTS1->LTS2).
    auto chiOf = [&](const std::vector<std::vector<Complex>>& Y, size_t m0, Complex* lts_pair,
                     const StructTemplate** best_t = nullptr, bool* marker = nullptr) {
        double best = 0.0;
        for (const StructTemplate* t : tpls) {
            double chi_plain = 0.0, chi_marked = 0.0;
            for (int gi = 0; gi < kStructLagGroups; ++gi) {
                if (t->n[gi] <= 0.0) continue;
                Complex a0(0, 0), a1(0, 0);
                for (const auto& pr : t->groups[gi]) {
                    const Complex u =
                        unitOf(Y[m0 + pr.s_hi][pr.l] * std::conj(Y[m0 + pr.s_lo][pr.l])) * pr.qconj;
                    if (pr.s_lo == 0) a0 += u; else a1 += u;
                }
                chi_plain += std::norm(a1 + a0) / t->n[gi];
                chi_marked += std::norm(a1 - a0) / t->n[gi];
            }
            const double chi = std::max(chi_plain, chi_marked);
            if (chi > best) {
                best = chi;
                if (best_t) *best_t = t;
                if (marker) *marker = chi_marked > chi_plain;
            }
        }
        if (lts_pair) {  // LTS1 -> LTS2 sum: only for the burst-marker sign, never for detection
            Complex sl(0, 0);
            for (size_t l = 0; l < nc; ++l) sl += unitOf(Y[m0 + 1][l] * std::conj(Y[m0][l]));
            *lts_pair = sl;
        }
        return best;
    };

    const int hop = std::max(1, Ls / 20);
    struct Cand { int p; double chi; };
    std::vector<Cand> cands;
    std::vector<std::vector<Complex>> Y;
    for (int phase = 0; phase < Ls; phase += hop) {
        Y.clear();
        for (int w = phase + cp; w + N <= L; w += Ls) {
            Y.emplace_back();
            carriersAt(w, Y.back());
        }
        for (size_t m0 = 0; m0 + kStructSyms <= Y.size(); ++m0) {
            cands.push_back({phase + static_cast<int>(m0) * Ls, chiOf(Y, m0, nullptr)});
        }
    }
    if (cands.empty()) return false;
    // Union bound over every test of this call. Cached: the candidate count only changes with
    // the window size, the template count with the data geometry.
    static thread_local struct { size_t tests = 0; double budget = -1.0, T = 0.0; } thr;
    const size_t tests = cands.size() * tpls.size() * 2;  // x2 marker hypotheses
    if (tests != thr.tests || false_alarm_budget != thr.budget) {
        const double p_test = std::clamp(false_alarm_budget, 1.0e-300, 0.5) / static_cast<double>(tests);
        thr = {tests, false_alarm_budget, gammaSurvivalThreshold(kStructLagGroups, p_test)};
    }
    const double kThreshold = thr.T;
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.p < b.p; });
    double best_any = 0.0;
    int first = -1;
    for (size_t i = 0; i < cands.size(); ++i) {
        best_any = std::max(best_any, cands[i].chi);
        if (first < 0 && cands[i].chi >= kThreshold) first = static_cast<int>(i);
    }
    if (first < 0) {
        result.correlation = static_cast<float>(std::min(1.0, best_any / (2.0 * kThreshold)));
        return false;
    }
    Cand coarse = cands[static_cast<size_t>(first)];
    const int first_p = cands[static_cast<size_t>(first)].p;
    for (size_t i = static_cast<size_t>(first); i < cands.size() && cands[i].p < first_p + Ls; ++i) {
        if (cands[i].chi > coarse.chi) coarse = cands[i];
    }
    // Fine timing from the LTS channel estimate's phase ramp (standard OFDM fine timing).
    // A window late by d samples multiplies carrier k by e^{+j 2 pi k d / N}; arg of
    // sum_l H_{l+1} conj(H_l) over frequency-adjacent carriers is 2 pi d / N per bin. On
    // multipath it centres on the channel's delay centroid. Uses LTS2 only (LTS1 may carry
    // the group-start sign flip). The chi decision above is unaffected.
    std::vector<std::vector<Complex>> Yc(kStructSyms);
    bool ok = true;
    for (int sidx = 0; sidx < kStructSyms && ok; ++sidx) {
        ok = carriersAt(coarse.p + cp + sidx * Ls, Yc[static_cast<size_t>(sidx)]);
    }
    if (!ok) return false;
    Complex slts;
    const StructTemplate* best_t = tpls.front();
    bool marked = false;
    const double peak = chiOf(Yc, 0, &slts, &best_t, &marked);
    Complex ramp(0, 0);
    if (best_t->x_lts.size() == nc) {
        for (size_t l = 0; l + 1 < nc; ++l) {
            if (bins[l + 1] - bins[l] != 1) continue;  // skip the DC gap
            const Complex h0 = Yc[1][l] * std::conj(best_t->x_lts[l]);  // LTS2: never marked
            const Complex h1 = Yc[1][l + 1] * std::conj(best_t->x_lts[l + 1]);
            ramp += h1 * std::conj(h0);
        }
    }
    const double late = std::arg(ramp) * static_cast<double>(N) / (2.0 * M_PI);
    // Report the start that puts the FFT window in the middle of the CP's ISI-free region.
    const int start = std::max(0, coarse.p - static_cast<int>(std::lround(late)) - cp / 2);

    // LTS1->LTS2 phase (sign-corrected for the marker) for the residual-CFO log only.
    const double resid_cfo =
        std::arg(marked ? -slts : slts) / (2.0 * M_PI * Ls / config_.sample_rate);
    burst_interleaved_detected_ = marked;  // LTS1 negated = group-start marker
    burst_interleave_latched_ = burst_interleaved_detected_;
    // Per-cell coherence estimate: E[chi] = rho^2 * n + K * (1 - rho^2) for unit phasors of
    // coherence rho, so rho^2 = (chi - K) / (n - K). Reported as the sync correlation.
    double n_tot = 0.0;
    int k_used = 0;
    for (int gi = 0; gi < kStructLagGroups; ++gi) {
        if (best_t->n[gi] > 0.0) {
            n_tot += best_t->n[gi];
            ++k_used;
        }
    }
    const double rho2 = n_tot > k_used ? (peak - k_used) / (n_tot - k_used) : 0.0;
    result.detected = true;
    result.start_sample = start;
    result.correlation = static_cast<float>(std::sqrt(std::clamp(rho2, 0.0, 1.0)));
    training_start_sample_ = static_cast<size_t>(start);
    synced_ = true;
    last_cfo_ = known_cfo_hz;
    LOG_MODEM(INFO,
              "OFDMChirpWaveform: STRUCTURE sync at %d chi=%.1f (T=%.1f, %zu tests) rho=%.2f "
              "ramp_late=%+.1f resid_cfo=%+.2f Hz (known %.1f)%s",
              start, peak, kThreshold, tests, result.correlation, late, resid_cfo, known_cfo_hz,
              burst_interleaved_detected_ ? " [BURST-INTERLEAVED]" : "");
    return true;
}

bool OFDMChirpWaveform::detectDataSync(SampleSpan samples, SyncResult& result,
                                        float known_cfo_hz, float threshold) {
    timing::ScopedTimer _profile_(timing::globalDecoderProfile().detect_data_sync);
    // Detect training-only preamble (no chirp) for DATA frames
    // Uses Schmidl-Cox style detection: LTS has two identical symbols,
    // so we correlate sample[n] with sample[n + symbol_length]

    result.detected = false;
    result.correlation = 0.0f;
    result.cfo_hz = known_cfo_hz;  // Use known CFO from previous frames
    result.has_training = true;

    const int symbol_samples = getSamplesPerSymbol();
    const int search_window = symbol_samples * 4;  // Search first 4 symbols worth

    if (samples.size() < static_cast<size_t>(symbol_samples * 3)) {
        return false;  // Need at least 3 symbols (2 LTS + margin)
    }

    // Energy-based gate: find where signal starts
    // When the buffer starts with silence (noise_floor low), the energy gate efficiently
    // skips the quiet region. When the buffer starts with signal (burst continuation,
    // noise_floor high), the energy gate can't find a transition, so we search the
    // full buffer instead — the LTS autocorrelation peak is distinctive enough to
    // stand out from data autocorrelation.
    float noise_floor = 0.0f;
    size_t noise_samples = std::min(samples.size() / 4, size_t(4800));  // First 100ms
    for (size_t i = 0; i < noise_samples; ++i) {
        noise_floor += samples[i] * samples[i];
    }
    noise_floor = std::sqrt(noise_floor / noise_samples);
    float energy_threshold = noise_floor * 3.0f + 0.01f;  // 3x noise or minimum

    size_t signal_start = 0;
    bool signal_in_noise = (noise_floor < 0.05f);  // Buffer starts with silence

    if (signal_in_noise) {
        // Find first sample above energy threshold (skip silence region)
        for (size_t i = 0; i < samples.size() - symbol_samples * 2; ++i) {
            float energy = 0.0f;
            for (int j = 0; j < 64; ++j) {  // Check 64 samples
                if (i + j < samples.size()) {
                    energy += samples[i + j] * samples[i + j];
                }
            }
            energy = std::sqrt(energy / 64);
            if (energy > energy_threshold) {
                signal_start = i;
                break;
            }
        }
    }
    // If !signal_in_noise: signal_start stays 0, search entire buffer

    // CFO-aware Schmidl-Cox style autocorrelation for LTS detection.
    // Use analytic signal (Hilbert) so correlation magnitude is robust to
    // carrier/CFO phase rotation and avoids real-only sign collapse.
    data_sync_hilbert_.reset();
    auto& analytic = data_sync_analytic_scratch_;
    data_sync_hilbert_.process(samples, analytic);
    if (analytic.size() < static_cast<size_t>(symbol_samples * 3)) {
        return false;
    }

    const bool matched_filter_ready = ensureDataSyncTemplate(symbol_samples);
    const bool matched_filter_can_drive_lock = threshold < 0.50f;
    const int lts_pair_samples = symbol_samples * 2;

    struct DataSyncCandidate {
        float combined_corr = 0.0f;
        float schmidl_corr = 0.0f;
        float matched_corr = 0.0f;
        Complex schmidl_p = Complex(0.0f, 0.0f);
    };

    auto evaluate_offset = [&](int offset) -> DataSyncCandidate {
        DataSyncCandidate candidate;
        if (offset < 0 ||
            offset + lts_pair_samples > static_cast<int>(analytic.size())) {
            return candidate;
        }

        // LTS has 2 identical symbols, so correlate with 1 symbol delay.
        Complex P(0.0f, 0.0f);
        float energy1 = 0.0f;
        float energy2 = 0.0f;

        for (int n = 0; n < symbol_samples; ++n) {
            const int idx1 = offset + n;
            const int idx2 = offset + n + symbol_samples;
            const Complex& s1 = analytic[idx1];
            const Complex& s2 = analytic[idx2];
            P += std::conj(s1) * s2;
            energy1 += std::norm(s1);
            energy2 += std::norm(s2);
        }

        const float schmidl_denom = std::sqrt(energy1 * energy2) + 1e-10f;
        candidate.schmidl_corr = std::abs(P) / schmidl_denom;
        candidate.schmidl_p = P;

        if (matched_filter_ready &&
            data_sync_template_analytic_.size() == static_cast<size_t>(lts_pair_samples)) {
            Complex first_symbol_corr(0.0f, 0.0f);
            Complex second_symbol_corr(0.0f, 0.0f);
            float rx_energy = 0.0f;

            const float phase_inc = -2.0f * static_cast<float>(M_PI) *
                                    known_cfo_hz / config_.sample_rate;
            float phase = 0.0f;
            for (int n = 0; n < lts_pair_samples; ++n) {
                const Complex cfo_correction(std::cos(phase), std::sin(phase));
                const Complex rx = analytic[offset + n] * cfo_correction;
                const Complex& ref = data_sync_template_analytic_[static_cast<size_t>(n)];
                const Complex corr = std::conj(ref) * rx;
                if (n < symbol_samples) {
                    first_symbol_corr += corr;
                } else {
                    second_symbol_corr += corr;
                }
                rx_energy += std::norm(rx);
                phase += phase_inc;
            }

            const float matched_denom =
                std::sqrt(data_sync_template_energy_ * rx_energy) + 1e-10f;
            const float normal_score =
                std::abs(first_symbol_corr + second_symbol_corr) / matched_denom;
            const float marker_score =
                std::abs(-first_symbol_corr + second_symbol_corr) / matched_denom;
            candidate.matched_corr = std::max(normal_score, marker_score);
        }

        // The matched filter is allowed to lower the effective detection floor
        // only in the narrowed expected-arrival path. In the wide/cold fallback
        // window, keep legacy Schmidl-Cox acceptance semantics so payload data
        // cannot lock on a matched-only LTS-like projection.
        candidate.combined_corr = matched_filter_can_drive_lock
            ? std::max(candidate.schmidl_corr, candidate.matched_corr)
            : candidate.schmidl_corr;
        return candidate;
    };

    float best_corr = 0.0f;
    float best_schmidl_corr = 0.0f;
    float best_matched_corr = 0.0f;
    int best_offset = 0;
    Complex best_p(0.0f, 0.0f);

    auto adopt_candidate = [&](int offset, const DataSyncCandidate& candidate) {
        best_corr = candidate.combined_corr;
        best_schmidl_corr = candidate.schmidl_corr;
        best_matched_corr = candidate.matched_corr;
        best_offset = offset;
        best_p = candidate.schmidl_p;
    };

    // Keep search local to expected frame start. Scanning the full buffer makes
    // payload autocorrelation peaks compete with LTS and increases false locks.
    int max_connected_search = std::max(search_window, symbol_samples * 8);
    int actual_search_window = signal_in_noise ? search_window : max_connected_search;
    int search_end = std::min(static_cast<int>(signal_start) + actual_search_window,
                              static_cast<int>(samples.size()) - symbol_samples * 2);

    for (int offset = static_cast<int>(signal_start);
         offset < search_end; offset += 8) {  // Step by 8 for speed

        const DataSyncCandidate candidate = evaluate_offset(offset);
        if (candidate.combined_corr > best_corr) {
            adopt_candidate(offset, candidate);
        }

        // Early exit on first high-confidence peak. The LTS (two identical training
        // symbols) is always the FIRST pair of identical symbols in the frame. With
        // 1-CW LDPC zero-padding, data symbols can also be identical (all-zero bits →
        // 0° DQPSK phase change), creating false peaks later in the search window.
        // By stopping at the first peak above 0.95, we always lock onto the real LTS.
        if (candidate.combined_corr > 0.95f) {
            break;
        }
    }

    // Fine refinement: 1-sample steps around coarse peak.
    // The 8-sample coarse search can be up to 4 samples off-peak.
    // For QPSK, even 4-sample offset causes ~40° phase error at edge carriers.
    // Cost: 9 evaluations × symbol_samples MACs — negligible.
    if (best_corr > threshold) {
        int refine_start = std::max(static_cast<int>(signal_start), best_offset - 4);
        int refine_end = std::min(search_end, best_offset + 5);  // exclusive

        for (int offset = refine_start; offset < refine_end; ++offset) {
            if (offset == best_offset) continue;  // Skip already-evaluated

            const DataSyncCandidate candidate = evaluate_offset(offset);
            if (candidate.combined_corr > best_corr) {
                adopt_candidate(offset, candidate);
            }
        }
    }

    // If a weak candidate lands at the edge of the capped connected search,
    // the real LTS can be just beyond the cap in the same streaming buffer.
    // Do one bounded rescue pass only for sub-0.45 edge candidates, preserving
    // the low-SNR relaxed floor when no stronger peak is present.
    constexpr float kWeakConnectedPeakCorr = 0.45f;
    constexpr float kMinLaterPeakImprovement = 0.02f;
    constexpr int kEdgeCandidateMarginSamples = 16;
    const bool weak_edge_candidate =
        best_corr > threshold &&
        best_corr < kWeakConnectedPeakCorr &&
        best_offset + kEdgeCandidateMarginSamples >= search_end;
    if (weak_edge_candidate) {
        const int extended_search_end =
            static_cast<int>(samples.size()) - symbol_samples * 2;
        const int original_best_offset = best_offset;
        const float original_best_corr = best_corr;
        const float original_best_schmidl_corr = best_schmidl_corr;
        const float original_best_matched_corr = best_matched_corr;
        Complex original_best_p = best_p;

        for (int offset = search_end; offset < extended_search_end; offset += 8) {
            const DataSyncCandidate candidate = evaluate_offset(offset);
            if (candidate.combined_corr > best_corr + kMinLaterPeakImprovement) {
                adopt_candidate(offset, candidate);
            }
        }

        if (best_offset != original_best_offset) {
            int refine_start = std::max(search_end, best_offset - 4);
            int refine_end = std::min(extended_search_end, best_offset + 5);

            for (int offset = refine_start; offset < refine_end; ++offset) {
                if (offset == best_offset) continue;

                const DataSyncCandidate candidate = evaluate_offset(offset);
                if (candidate.combined_corr > best_corr) {
                    adopt_candidate(offset, candidate);
                }
            }

            LOG_MODEM(INFO, "OFDMChirpWaveform: Data sync replaced weak peak at %d (corr=%.2f) "
                      "with later peak at %d (corr=%.2f)",
                      original_best_offset, original_best_corr, best_offset, best_corr);
        } else {
            best_corr = original_best_corr;
            best_schmidl_corr = original_best_schmidl_corr;
            best_matched_corr = original_best_matched_corr;
            best_offset = original_best_offset;
            best_p = original_best_p;
        }
    }

    result.correlation = best_corr;

    // Reset latched marker at start of each detection attempt
    burst_interleave_latched_ = false;
    burst_interleaved_detected_ = false;

    if (best_corr > threshold) {
        result.detected = true;
        result.start_sample = best_offset;  // Training starts here
        training_start_sample_ = best_offset;
        synced_ = true;
        last_cfo_ = known_cfo_hz;

        // Check LTS sign for burst interleave marker.
        // Compensate expected CFO phase rotation between repeated halves:
        // phase = 2*pi*CFO*L/fs. Negated first LTS then appears as negative real part.
        float cfo_phase = 2.0f * M_PI * known_cfo_hz * symbol_samples / config_.sample_rate;
        Complex cfo_comp(std::cos(-cfo_phase), std::sin(-cfo_phase));
        Complex marker_metric = best_p * cfo_comp;
        burst_interleaved_detected_ = (marker_metric.real() < 0.0f);
        burst_interleave_latched_ = burst_interleaved_detected_;
        if (ultra::phyDiagnosticsEnabled()) {
            std::ostringstream oss;
            oss << "burst_marker site=warm re=" << marker_metric.real()
                << " im=" << marker_metric.imag()
                << " sc=" << best_schmidl_corr << " mf=" << best_matched_corr
                << " latched=" << (burst_interleave_latched_ ? 1 : 0);
            ultra::phyDiagLine(oss.str());
        }

        LOG_MODEM(INFO,
                  "OFDMChirpWaveform: Data sync detected at %d, corr=%.2f (sc=%.2f mf=%.2f), using CFO=%.1f Hz%s",
                  best_offset, best_corr, best_schmidl_corr, best_matched_corr, known_cfo_hz,
                  burst_interleaved_detected_ ? " [BURST-INTERLEAVED]" : "");
    }

    return result.detected;
}


void OFDMChirpWaveform::setAbsoluteTrainingPosition(size_t pos) {
    absolute_training_start_sample_ = pos;
    has_absolute_training_start_sample_ = true;
}

bool OFDMChirpWaveform::process(SampleSpan samples) {
    timing::ScopedTimer _profile_(timing::globalDecoderProfile().ofdm_process_total);
    if (!demodulator_) {
        return false;
    }

    // Calculate the initial CFO phase based on elapsed samples since audio start
    // The test harness applies CFO to the entire audio stream from sample 0.
    // By the time we reach training_start_sample_, the CFO has accumulated:
    //   phase = -2π × CFO × elapsed_samples / sample_rate
    //
    // We need to start CFO correction from this accumulated phase, not from 0.
    size_t phase_ref_sample = training_start_sample_;
    if (has_absolute_training_start_sample_) {
        phase_ref_sample = absolute_training_start_sample_;
    }

    float initial_phase_rad = -2.0f * M_PI * cfo_hz_ * phase_ref_sample / config_.sample_rate;

    // Wrap to [-π, π]
    while (initial_phase_rad > M_PI) initial_phase_rad -= 2.0f * M_PI;
    while (initial_phase_rad < -M_PI) initial_phase_rad += 2.0f * M_PI;

    LOG_MODEM(INFO, "OFDMChirpWaveform::process(): samples=%zu, cfo=%.1f, training_start=%zu, abs_start=%zu",
              samples.size(), cfo_hz_, training_start_sample_,
              has_absolute_training_start_sample_ ? absolute_training_start_sample_ : 0);
    // DIAGNOSTIC (data-aided genie / ULTRA_EQ_TRACE): publish this pass's absolute LTS
    // origin. It is the ONLY per-pass absolute identity the receiver has, and the genie
    // addresses captured TX symbols by
    //   abs_sym = abs_train + (training_symbols + data_symbol_index) * symbol_samples,
    // which is invariant across the 2-6 re-decodes of the same frame (peek, header
    // peek, authoritative pass, sync-recovery retries). Stale values are impossible
    // for the genie because it also requires passContext().presynced, set immediately
    // below in processPresynced(). Inert unless a diagnostic consumer is armed.
    if (ultra::genie::passContextNeeded()) {
        ultra::genie::passContext().abs_train = phase_ref_sample;
    }

    // Pass CFO and initial phase to demodulator
    // This ensures CFO correction starts from the correct accumulated phase
    demodulator_->setFrequencyOffsetWithPhase(cfo_hz_, initial_phase_rad);

    // If burst interleave marker was detected, undo LTS negation before channel estimation.
    // The TX negated the first LTS symbol as a marker. We must restore it so the
    // demodulator sees correct training data for channel estimation.
    // ONE-SHOT: consume the flag immediately so continuation frames aren't affected.
    bool ready;
    if (burst_interleaved_detected_) {
        burst_interleaved_detected_ = false;  // One-shot: consume before continuation frames

        // Create mutable copy and negate first LTS symbol
        std::vector<float> modified(samples.begin(), samples.end());
        size_t lts_sym_len = static_cast<size_t>(getSamplesPerSymbol());
        for (size_t i = 0; i < lts_sym_len && i < modified.size(); i++) {
            modified[i] = -modified[i];
        }

        ready = demodulator_->processPresynced(SampleSpan(modified), 2);
    } else {
        // Normal path (no burst marker)
        ready = demodulator_->processPresynced(samples, 2);
    }

    if (ready) {
        // Retrieve ALL soft bits from demodulator. getSoftBits() returns one
        // codeword-sized chunk per call — 648 bits at Z=27, 1944 at Z=81.
        // 2026-05-28: was hardcoded `chunk.size() != LDPC_CODEWORD_BITS` (==648)
        // → ANY chunk produced at Z=81 (1944 bits) failed the check and the
        // loop bailed without collecting anything, surfacing as
        // "getSoftBits() returned empty" upstream and stalling burst data
        // frame processing. Compare against the demod's runtime block size
        // instead so both Z=27 and Z=81 paths work.
        const size_t codeword_bits =
            demodulator_ ? demodulator_->getActiveLDPCBlockSize()
                         : LDPC_CODEWORD_BITS;
        soft_bits_.clear();
        while (demodulator_->hasPendingData()) {
            auto chunk = demodulator_->getSoftBits();
            if (chunk.size() != codeword_bits) break;
            soft_bits_.insert(soft_bits_.end(), chunk.begin(), chunk.end());
        }

        // CarrierLDPC interleaves in 648-bit AIR-BLOCK units (kLdpcCodewordBits),
        // NOT LDPC z-codewords. TX computes Ncw = encoded_bytes / 81 (line 393),
        // so a z=81 (1944-bit) codeword is THREE 648-bit air blocks. RX MUST count
        // the same way or the de-interleave length + permutation diverge from TX:
        // z=81 2-CW frame is 3888 air bits = Ncw 6 (NOT 2). The old `/ codeword_bits`
        // (=1944) gave Ncw 2 → applyCarrierLdpcInverse built out(1296) and dropped
        // 2/3 of the LLRs with the wrong permutation → z=81 differential data was
        // silently mangled (coherent QPSK took a different eligibility path and was
        // unaffected, which is why only differential failed). For z=27 this is
        // identical to the old value (soft_bits / 648).
        const size_t codeword_count = soft_bits_.size() / LDPC_CODEWORD_BITS;
        const bool eligible = carrierLdpcPlumbingEligible() &&
            !soft_bits_.empty() &&
            (soft_bits_.size() % LDPC_CODEWORD_BITS == 0);
        const bool masked_carriers =
            !isAllOnMask(carrier_mask_, CARRIER_LDPC_MASK_CARRIERS);
        const bool carrier_ldpc_active = eligible &&
            codeword_count != 1 &&
            carrierLdpcCodewordCountSupported(codeword_count) &&
            (carrier_ldpc_interleaver_enabled_ || masked_carriers);
        if (eligible && codeword_count == 1) {
            // Ncw=1 remains legacy ordered; this includes the future 1-CW
            // R1/4 PHY mask header, which must stay bit-identical.
        } else if (carrier_ldpc_active) {
            std::vector<float> air_llrs = std::move(soft_bits_);
            if (masked_carriers) {
                air_llrs = eraseMaskedCarrierLLRs(std::move(air_llrs),
                                                  config_, carrier_mask_);
            }
            soft_bits_ = applyCarrierLdpcInverse(std::move(air_llrs), codeword_count);
        }

        last_snr_ = demodulator_->getEstimatedSNR();

        // Feed back pilot-corrected CFO from demodulator
        // On fading channels, chirp-based CFO can be wrong. The demodulator's
        // pilot tracking and LTS residual estimation correct it. Propagate
        // this correction back so subsequent frames use the refined CFO.
        float corrected_cfo = demodulator_->getFrequencyOffset();
        if (std::abs(corrected_cfo - cfo_hz_) > 0.1f) {
            LOG_MODEM(INFO, "OFDMChirpWaveform: CFO feedback: chirp=%.2f -> corrected=%.2f Hz",
                      cfo_hz_, corrected_cfo);
        }
        cfo_hz_ = corrected_cfo;
        last_cfo_ = corrected_cfo;
    }

    return ready;
}

Symbol OFDMChirpWaveform::getLastDataCarrierSymbolsForTesting() const {
    return modulator_ ? modulator_->getLastDataCarrierSymbolsForTesting() : Symbol{};
}

std::vector<float> OFDMChirpWaveform::getSoftBits() {
    return std::move(soft_bits_);
}

void OFDMChirpWaveform::setRXCarrierErasureEnabled(bool enabled) {
    rx_carrier_erasure_enabled_ = enabled;
    if (demodulator_) {
        demodulator_->setRXCarrierErasureEnabled(enabled);
    }
}

void OFDMChirpWaveform::reset() {
    if (demodulator_) {
        demodulator_->reset();
    }
    soft_bits_.clear();
    synced_ = false;
    has_absolute_training_start_sample_ = false;
    absolute_training_start_sample_ = 0;
    // NOTE: CFO is intentionally preserved across reset() for continuous tracking
    // Use setFrequencyOffset(0) to explicitly clear if needed
    // TX CFO in config_ is also preserved for simulation
}

bool OFDMChirpWaveform::isSynced() const {
    return synced_ || (demodulator_ && demodulator_->isSynced());
}

bool OFDMChirpWaveform::hasData() const {
    return !soft_bits_.empty() || (demodulator_ && demodulator_->hasPendingData());
}

float OFDMChirpWaveform::estimatedSNR() const {
    if (demodulator_) {
        return demodulator_->getEstimatedSNR();
    }
    return last_snr_;
}

bool OFDMChirpWaveform::hasLastOFDMBroadbandSNREstimate() const {
    return demodulator_ && demodulator_->hasLastOFDMBroadbandSNREstimate();
}

float OFDMChirpWaveform::getLastOFDMBroadbandSNREstimate() const {
    if (demodulator_) {
        return demodulator_->getLastOFDMBroadbandSNREstimate();
    }
    return 0.0f;
}

bool OFDMChirpWaveform::hasEvmSnrEstimate() const {
    return demodulator_ && demodulator_->hasEvmSnr();
}

float OFDMChirpWaveform::getEvmSnrDbEstimate() const {
    return demodulator_ ? demodulator_->getEvmSnrDb() : 0.0f;
}

bool OFDMChirpWaveform::takeEvmSnrEstimate(float& evm_snr_db,
                                           size_t& carrier_count) {
    return demodulator_ && demodulator_->takeEvmSnr(evm_snr_db, carrier_count);
}

float OFDMChirpWaveform::estimatedCFO() const {
    if (std::abs(last_cfo_) > 0.1f) {
        return last_cfo_;
    }
    if (demodulator_) {
        return demodulator_->getFrequencyOffset();
    }
    return cfo_hz_;
}

float OFDMChirpWaveform::getFadingIndex() const {
    if (demodulator_) {
        return demodulator_->getFadingIndex();
    }
    return 0.0f;
}

void OFDMChirpWaveform::setWienerChannelParams(float doppler_hz, float delay_spread_s) {
    if (demodulator_) {
        demodulator_->setWienerChannelParams(doppler_hz, delay_spread_s);
    }
}

float OFDMChirpWaveform::getLastPilotFrequencyCV() const {
    if (demodulator_) {
        return demodulator_->getLastPilotFrequencyCV();
    }
    return 0.0f;
}

float OFDMChirpWaveform::getLastPilotTemporalCV() const {
    if (demodulator_) {
        return demodulator_->getLastPilotTemporalCV();
    }
    return 0.0f;
}

float OFDMChirpWaveform::getLastPilotSymbolMeanCV() const {
    if (demodulator_) {
        return demodulator_->getLastPilotSymbolMeanCV();
    }
    return 0.0f;
}

float OFDMChirpWaveform::getLastTimingOffsetSamples() const {
    if (demodulator_) {
        return demodulator_->getLastTimingOffsetSamples();
    }
    return 0.0f;
}

float OFDMChirpWaveform::getLastLTSSignalPower() const {
    if (demodulator_) {
        return demodulator_->getLastLTSSignalPower();
    }
    return 1.0f;
}

float OFDMChirpWaveform::getLastLTSChannelMagnitude() const {
    if (demodulator_) {
        return demodulator_->getLastLTSChannelMagnitude();
    }
    return 1.0f;
}

std::vector<float> OFDMChirpWaveform::getCarrierGammaSnapshot() const {
    if (demodulator_) {
        return demodulator_->getCarrierGammaSnapshot();
    }
    return {};
}

float OFDMChirpWaveform::getLastLTSNoiseVariance() const {
    if (demodulator_) {
        return demodulator_->getLastLTSNoiseVariance();
    }
    return 0.0f;
}

float OFDMChirpWaveform::getLastLTSPairChi() const {
    return demodulator_ ? demodulator_->getLastLTSPairChi() : 0.0f;
}

float OFDMChirpWaveform::getLastLTSResidualCFOHz() const {
    if (demodulator_) {
        return demodulator_->getLastLTSResidualCFOHz();
    }
    return 0.0f;
}

std::string OFDMChirpWaveform::getFailureAttributionDiagnosticsText() const {
    if (demodulator_) {
        return demodulator_->getFailureAttributionDiagnosticsText();
    }
    return {};
}

std::vector<std::complex<float>> OFDMChirpWaveform::getConstellationSymbols() const {
    if (demodulator_) {
        return demodulator_->getConstellationSymbols();
    }
    return {};
}

Modulation OFDMChirpWaveform::getConstellationModulation() const {
    if (demodulator_) {
        return demodulator_->getConstellationModulation();
    }
    return Modulation::QPSK;
}

std::string OFDMChirpWaveform::getStatusString() const {
    std::ostringstream oss;
    oss << "OFDM-Chirp " << config_.num_carriers << " carriers, "
        << modulationToString(config_.modulation) << " "
        << codeRateToString(config_.code_rate);
    if (std::abs(last_cfo_) > 0.5f) {
        oss << " (CFO=" << static_cast<int>(last_cfo_) << " Hz)";
    }
    return oss.str();
}

float OFDMChirpWaveform::getThroughput(CodeRate rate) const {
    // Calculate data carriers based on pilot configuration for the given rate.
    const int pilot_spacing =
        ofdm_link_adaptation::recommendedPilotSpacing(config_.modulation, rate);
    const int data_carriers = ofdm_link_adaptation::dataCarrierCount(
        static_cast<int>(config_.num_carriers), true, pilot_spacing);
    const int bits_per_carrier = static_cast<int>(getBitsPerSymbol(config_.modulation));

    // Symbol rate
    float symbol_rate = static_cast<float>(config_.sample_rate) / getSamplesPerSymbol();

    // Raw bit rate
    float raw_bps = symbol_rate * data_carriers * bits_per_carrier;

    return raw_bps * getCodeRateValue(rate);
}

int OFDMChirpWaveform::getSamplesPerSymbol() const {
    if (modulator_) {
        return static_cast<int>(modulator_->samplesPerSymbol());
    }
    // Fallback calculation
    int cp_samples = 0;
    switch (config_.cp_mode) {
        case CyclicPrefixMode::SHORT:  cp_samples = config_.fft_size / 8; break;
        case CyclicPrefixMode::MEDIUM: cp_samples = config_.fft_size / 4; break;
        case CyclicPrefixMode::LONG:   cp_samples = config_.fft_size / 2; break;
    }
    return config_.fft_size + cp_samples;
}

int OFDMChirpWaveform::getPreambleSamples() const {
    int chirp_total = chirp_sync_ ? static_cast<int>(chirp_sync_->getTotalSamples())
                                  : static_cast<int>(config_.sample_rate * 1.2f);  // ~1.2 sec default
    int training = 2 * getSamplesPerSymbol();  // 2 OFDM training symbols
    return chirp_total + training;
}

int OFDMChirpWaveform::getDataPreambleSamples() const {
    // Light preamble: just training symbols (no chirp)
    // 2 LTS symbols for channel estimation
    return 2 * getSamplesPerSymbol();
}


int OFDMChirpWaveform::getMinSamplesForFrame() const {
    return getMinSamplesForCWCount(4);
}

int OFDMChirpWaveform::getMinSamplesForControlFrame() const {
    return getMinSamplesForCWCount(1);
}

int OFDMChirpWaveform::getMinSamplesForCWCount(int num_cw) const {
    // BUG-BURST-STALE-GEOMETRY (2026-07-28): the closed form moved to
    // ofdm_link_adaptation::minSamplesForCWCountExplicit so the decoder can size a
    // frame for a geometry it has NOT configured yet (a missed burst descriptor
    // cannot apply mod/rate inline — see §14.36 SIGSEGV). Delegating here (rather
    // than copying the arithmetic) makes the live-object path and the pure query
    // provably ONE implementation; they cannot drift.
    //
    // 2026-05-28 Phase 3: active codeword length is the lifted N for the current
    // burst. ldpc_lifting_z_ is set per-burst (default 27 -> n=648, 81 -> n=1944).
    // At z=81 the frame airtime is ~3x the legacy z=27 frame for the same num_cw —
    // callers that care about wall-clock should pair num_cw and lifting_z carefully
    // (typically cw=1 at z=81).
    return ofdm_link_adaptation::minSamplesForCWCountExplicit(
        num_cw,
        config_.modulation,
        ldpc_lifting_z_,
        static_cast<int>(config_.num_carriers),
        config_.use_pilots,
        static_cast<int>(config_.pilot_spacing),
        getSamplesPerSymbol());
}

} // namespace ultra
