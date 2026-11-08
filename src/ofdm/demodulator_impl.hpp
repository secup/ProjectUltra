#pragma once

// OFDMDemodulator::Impl - Private implementation struct
// Split across multiple .cpp files for maintainability

#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "demodulator_constants.hpp"
#include <limits>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include "ofdm/frequency_selectivity.hpp"
#include "delay_domain_interpolator.hpp"

namespace ultra {

// Forward declaration
class LDPCDecoder;

struct OFDMDemodulator::Impl {
    ModemConfig config;
    FFT fft;
    NCO mixer;

    // Carrier indices (must match modulator)
    std::vector<int> data_carrier_indices;
    std::vector<int> pilot_carrier_indices;
    std::vector<size_t> data_logical_carrier_indices;
    std::vector<size_t> pilot_logical_carrier_indices;
    std::vector<Complex> pilot_sequence;

    // Synchronization state (atomic for thread-safe UI access)
    enum class State { SEARCHING, SYNCED };
    std::atomic<State> state{State::SEARCHING};
    std::atomic<int> synced_symbol_count{0};
    std::atomic<int> idle_call_count{0};

    // Sample buffer
    Samples rx_buffer;
    size_t symbol_samples;

    // Reused OFDM hot-path scratch. These buffers are single-thread owned by
    // the demodulator instance and avoid heap churn in per-symbol processing.
    std::vector<Complex> baseband_scratch;
    std::vector<Complex> symbol_scratch;
    std::vector<Complex> freq_domain_scratch;
    std::vector<Complex> equalized_scratch;
    std::vector<Complex> constellation_update_scratch;
    std::vector<Complex> interp_h_full_scratch;
    std::vector<Complex> interp_h_cir_scratch;
    std::vector<Complex> interp_h_clean_scratch;
    std::vector<int> interp_pilot_logical_pos_scratch;
    std::vector<Complex> interp_idft_phasors;
    std::vector<Complex> interp_dft_phasors;

    // LTS DFT-denoise gate + tap window. Initialized from env in the constructor
    // so DEFAULT behavior is unchanged, but settable from a test via
    // OFDMDemodulator::setLtsDftDenoise() so the offline H-MSE harness can flip
    // it on/off without re-reading the environment.
    bool dft_denoise_enabled_ = false;
    int dft_denoise_taps_ = 0;

    // Lever ① CFO-clean 2-LTS averaging (ULTRA_LTS_CFO_AVG): align all LTS symbols to
    // the LAST symbol's phase frame (common per-symbol CFO rotation), then average ->
    // -10log10(N) dB H-noise (-3 dB for N=2) with no phase mismatch. Settable from a
    // test via OFDMDemodulator::setLtsCfoAvg(). Default off until harness+GUI-proven.
    bool lts_cfo_avg_enabled_ = false;

    // Channel estimate (per carrier)
    std::vector<Complex> channel_estimate;
    // 2026-06-12 Phase 2b: per-carrier NORMALIZED channel-estimate error variance (the
    // Wiener interpolator's MMSE residual in [0,1], or the 0.04 non-Wiener fallback),
    // populated in updateChannelEstimate, consumed in equalize() as the eps_H LLR term
    // (ULTRA_HERR_LLR_K). Indexed by fft carrier, parallel to channel_estimate.
    std::vector<float> per_carrier_h_error_var_;
    // F166 Stage B (ULTRA_NOTCH_NV): most recent RAW direct LS pilot observation
    // power per FFT carrier + the data-symbol index it was taken at. The one
    // signal that sees through Wiener notch-smoothing (the scattered pattern
    // revisits every carrier each `spacing` symbols).
    std::vector<float> per_carrier_raw_obs_power_;
    std::vector<int64_t> per_carrier_raw_obs_symbol_;
    // 2026-05-29 diag (ULTRA_GENIE_LTS_FREEZE): a frozen copy of the full-band LTS
    // channel estimate, re-applied to every data symbol to bypass sparse-pilot
    // re-interpolation. On a frozen/noiseless channel the LTS H is the exact true H
    // (measured through the demod's own frontend), so this is a true genie that
    // isolates estimation/interpolation from post-equalization for 16QAM.
    std::vector<Complex> genie_lts_h_;
    float noise_variance = 0.1f;

    // SNR estimation (from pilots)
    float estimated_snr_linear = 1.0f;
    float snr_alpha = 0.3f;
    int snr_symbol_count = 0;

    // Measurement-quality in-band SNR estimate for operator display and future
    // adaptation. Valid only after OFDM LTS/pilot residuals have been measured;
    // unlike estimated_snr_linear, this must not feed LLR scaling.
    bool last_snr_db_estimate_valid = false;
    float last_snr_db_estimate = 0.0f;

    // ── Radio-agnostic decision-directed EVM SNR (Stage 1, 2026-07-24) ──
    // Measured from the scatter of the equalized DATA constellation around its own
    // hard decisions: SNR = Sum|hardDecision|^2 / Sum|equalized - hardDecision|^2 over
    // non-erased data carriers, accumulated across a burst's data symbols. This is the
    // demod's OWN error vector — it needs NO reference power, NO noise-shape assumption
    // and NO per-bench offset (unlike last_snr_db_estimate, whose consumers add
    // kOfdmLegacyAnchorScaleOffsetDb). It measures signal power empirically (Sum|decision|^2)
    // so it is constant-free regardless of constellation normalization. Exactly what
    // MC-DPSK's data-aided estimator does in the differential domain (multi_carrier_dpsk.hpp).
    // Stage 1: parallel/logged only — NOT consumed by rate selection yet.
    // GAIN-CORRECTED EVM: the MMSE equalizer leaves a scalar gain-shrinkage on the
    // constellation, so a raw |eq - decision|^2 counts that gain error as "noise" and
    // saturates the SNR at high true-SNR. Remove it by the least-squares fit g = S_de/S_ee
    // (the best scalar matching eq to the decisions), leaving pure noise:
    //   S_dd = Sum|decision|^2 ; S_ee = Sum|eq|^2 ; S_de = Sum Re(decision*conj(eq))
    //   noise = S_dd - S_de^2/S_ee ;  SNR = S_dd / noise.
    // A pure gain error (eq = a*ideal) yields noise=0 (not counted); only genuine scatter
    // survives. Constant-free and radio-agnostic.
    double evm_dd_accum_ = 0.0;   // S_dd = Sum |hard decision|^2 (signal power)
    double evm_ee_accum_ = 0.0;   // S_ee = Sum |equalized|^2
    double evm_de_accum_ = 0.0;   // S_de = Sum Re(decision * conj(equalized))
    size_t evm_carrier_count_ = 0;
    bool last_evm_snr_valid_ = false;
    float last_evm_snr_db_ = 0.0f;
    size_t last_evm_carrier_count_ = 0;
    // EVM SNR (dB) from the live accumulation.  This helper is private to the
    // frame finalizer: public readers only see last_evm_snr_db_ after the pass
    // that produced it has reached its own completion boundary.
    float currentEvmSnrDb() const {
        if (evm_carrier_count_ > 0 && evm_ee_accum_ > 0.0) {
            const double noise = evm_dd_accum_ - (evm_de_accum_ * evm_de_accum_) / evm_ee_accum_;
            if (noise <= 0.0) return 60.0f;
            const double lin = evm_dd_accum_ / noise;
            return 10.0f * static_cast<float>(std::log10(std::max(lin, 1.0e-9)));
        }
        return last_evm_snr_db_;
    }
    // Start/finalize one demodulation pass.  A new pass invalidates the previous
    // one-shot observation immediately; only a complete pass publishes a value.
    // This keeps a failed/erased frame from borrowing its predecessor's EVM.
    void beginEvmSnrFrame();
    void finalizeAndResetEvmSnr();

    // Last LTS estimate quality. False training locks on silence/noise have
    // near-zero values here even when the clipped LLR stream looks plausible.
    float last_lts_signal_power = 1.0f;
    float last_lts_channel_magnitude = 1.0f;
    float last_lts_residual_cfo_hz = 0.0f;
    // LTS-pair structure statistic chi = |sum_k unit(H1_k conj(H0_k))|^2 / N_k. ~Exp(1) on
    // noise or a misaligned window; large on ANY aligned real frame, faded or not (unit
    // normalization ignores amplitude). FEC-independent "a peer frame is here" evidence.
    float last_lts_pair_chi = 0.0f;

    // Internal per-symbol fading index from raw pilot magnitude variance.
    // This scale feeds demapper/two-pass decisions; it is intentionally
    // separate from the public channel-quality meter below.
    float last_fading_index = 0.0f;
    // First-frame channel-class discriminator (frequency selectivity). Measured from the
    // same averaged LTS H as last_fading_index, but reads the ripple STRUCTURE rather than
    // its variance, so it can separate Good from Moderate (which the CV provably cannot —
    // BUG-FADING-INDEX-BLIND). Read-only; not consumed by any decision yet.
    ofdm::FrequencySelectivity last_freq_selectivity{};
    // Public channel-quality fading index. Coherent modes keep this on the
    // payload-independent LTS estimate; differential modes can use pilot-only
    // frequency CV plus temporal CV.
    float public_fading_index = 0.0f;
    // RMS delay spread (ms) from the LTS channel impulse response (IFFT of H(f)).
    // Frequency-selectivity base for rate selection — single-snapshot from the connect
    // frame, and (unlike fading_index) actually discriminates the delay-spread classes.
    float last_delay_spread_ms = 0.0f;
    std::vector<float> pilot_mag_sum_;
    std::vector<float> pilot_mag_sq_sum_;
    float pilot_symbol_mean_sum_ = 0.0f;
    float pilot_symbol_mean_sq_sum_ = 0.0f;
    size_t pilot_fading_symbol_count_ = 0;
    mutable float last_pilot_frequency_cv = 0.0f;
    mutable float last_pilot_temporal_cv = 0.0f;
    mutable float last_pilot_symbol_mean_cv = 0.0f;
    // NOTE: the Doppler-coherence (Good/Moderate) estimator is hosted in the StreamingDecoder,
    // NOT here — burst transport rebuilds this demodulator every group, which would wipe the
    // pool. The decoder feeds it one per-frame |H|^2 snapshot from getLastLTSChannelMagnitude().
    // See docs/CHANNEL_DISCRIMINATOR_DESIGN_2026_06_15.md.

    // Wiener correlation-model parameters. Default to the env-aware values (robustDopplerHz/
    // robustDelaySpreadS, Moderate-HF 0.5 Hz / 1.0 ms). When the decoder has a valid Doppler-
    // coherence verdict it pushes the channel-class-derived values here (Good => 0.1 Hz / 0.5 ms,
    // less pessimistic forgetting), closing ADAPTIVITY_AUDIT Case #2. override_active_ false =>
    // use the env-aware default (preserves ULTRA_WIENER_* knobs).
    bool wiener_params_override_active_ = false;
    float wiener_doppler_hz_override_ = 0.5f;
    float wiener_delay_spread_s_override_ = 1.0e-3f;

    // ULTRA_PILOT_DFT_INTERP (default OFF): reconstruct the data-carrier channel
    // from a DELAY-DOMAIN (CIR) model instead of the frequency-smoothness prior.
    // Refreshed once per OFDM symbol at the top of updateChannelEstimate() —
    // deliberately NOT a function-local `static const`, which latches on first
    // call and has produced un-A/B-able knobs in this codebase before.
    // See src/ofdm/delay_domain_interpolator.hpp for the derivation.
    bool delay_domain_interp_ = false;
    ofdm_cir::DelayWindow delay_domain_window_{};
    // Per-symbol scratch: de-sloped pilot observations keyed by PHYSICAL carrier
    // index (used only when the Wiener path is inactive; the Wiener path feeds its
    // own time-extrapolated observations through the same kernel).
    std::vector<ofdm_wiener::Observation1D> delay_domain_pilot_obs_;

    // 2026-05-28: runtime LDPC codeword block size. Default 648 (Z=27 legacy).
    // Set to 1944 via OFDMDemodulator::setActiveLDPCBlockSize when a burst
    // descriptor announces Z=81 so the "we have a codeword's worth" gate inside
    // processPresynced waits for the full 1944-bit codeword instead of returning
    // after 648 bits. Source of truth replaces the demod_constants::LDPC_BLOCK_SIZE
    // constexpr at the gate sites; the constant is kept for non-data sync paths.
    size_t active_ldpc_block_size = 648;

    // Output data
    Bytes demod_data;
    std::vector<float> soft_bits;
    ChannelQuality quality;

    // Constellation display (latest equalized symbols)
    std::vector<Complex> constellation_symbols;
    mutable std::mutex constellation_mutex;
    // Modulation of the symbols currently in constellation_symbols. The buffer is
    // reset whenever the modulation changes so the GUI never overlays, e.g., a
    // 16QAM data frame and a DQPSK control/ACK frame into one smeared cloud.
    Modulation constellation_mod_ = Modulation::QPSK;
    // Visualization-only LDPC air-bit span for the current OFDM frame. The
    // modulator pads the last OFDM symbol with zero carriers when N*648 LDPC bits
    // do not fill the final symbol; those carriers are not data and must not be
    // rendered as constellation points. Decode still receives the unchanged soft
    // bits and ignores the incomplete trailing chunk as before.
    size_t constellation_air_bit_index_ = 0;
    size_t constellation_valid_air_bits_ = static_cast<size_t>(-1);
    size_t constellation_capacity_air_bits_ = 0;

    // Sync detection
    std::vector<Complex> sync_sequence;
    float sync_threshold;
    size_t last_sync_offset = 0;

    // Noise floor tracking for amplitude-independent sync detection
    float noise_floor_energy = 0.0f;

    // Pre-computed interpolation lookup
    struct InterpInfo {
        int fft_idx;
        int lower_pilot;
        int upper_pilot;
        float alpha;
    };
    std::vector<InterpInfo> interp_table;

    // All carrier FFT indices in logical order (for DFT interpolation)
    // Indices 0..N_carriers-1, mapping logical carrier to FFT bin
    std::vector<int> all_carrier_fft_indices;
    // Which logical indices are pilots (bitmask-style: pilot_logical_indices[i] = true)
    std::vector<bool> is_pilot_logical;

    // Frequency offset estimation and correction
    float freq_offset_hz = 0.0f;
    float freq_offset_filtered = 0.0f;
    bool chirp_cfo_estimated = false;  // True if CFO was set externally (e.g., chirp)
    // Prior sigma (Hz) of the CFO this frame starts from (lts_cfo_update.hpp): 0 = certified
    // by a decoded frame (no LTS frame correction), +inf = no information (LTS takes over),
    // otherwise the uncertified dual-chirp phantom sigma. Set per frame by the decoder.
    double cfo_prior_sigma_hz = std::numeric_limits<double>::infinity();
    float last_lts_cfo_gain = 0.0f;  // K applied by the last LTS residual update (diagnostic)
    std::vector<Complex> prev_pilot_phases;
    std::vector<size_t> prev_pilot_logical_indices;
    int symbols_since_sync = 0;
    float freq_correction_phase = 0.0f;

    // Pilot-based phase tracking for differential modulation
    Complex pilot_phase_correction = Complex(1, 0);

    // Symbol timing recovery
    float timing_offset_samples = 0.0f;

    // Carrier phase recovery
    Complex carrier_phase_correction = Complex(1, 0);
    bool carrier_phase_initialized = false;

    // Per-carrier phase slope from timing offset (radians per carrier index)
    // Estimated from LTS, used to de-slope pilot H before complex interpolation
    float lts_phase_slope = 0.0f;

    // Decision-directed per-carrier phase corrections from previous symbol.
    // Stored in equalize(), applied after interpolation in next updateChannelEstimate().
    // Each entry is the phase error (radians) measured at that data carrier.
    std::vector<float> dd_phase_corrections;

    // QAM16 decision-directed per-carrier channel observations from the previous
    // data symbol. The receiver loop updates pilots before equalization, so the
    // DD H=Y/X_hat observations are produced in equalize() and consumed once by
    // the next updateChannelEstimate() call after pilot interpolation.
    std::vector<Complex> dd_qam16_channel_observations_;
    std::vector<float> dd_qam16_measurement_var_;
    std::vector<float> dd_qam16_reliability_;
    std::vector<float> dd_qam16_channel_var_;

    // Höher/Kaiser/Robertson-style separable 2-D Wiener pilot interpolation.
    // History is keyed by logical carrier so scattered pilots can be combined
    // across time before frequency-domain MMSE interpolation.
    struct WienerPilotHistorySample {
        int64_t symbol_index = 0;
        Complex h = Complex(0, 0);
        float noise_norm = 0.0f;
    };
    std::vector<std::vector<WienerPilotHistorySample>> wiener_pilot_history_;
    std::vector<Complex> wiener_time_estimate_;
    std::vector<float> wiener_time_error_var_;
    std::vector<uint8_t> wiener_time_valid_;
    int64_t wiener_time_symbol_index_ = -999999;

    // ======================================================================
    // POST-FEC DATA-AIDED CHANNEL ESTIMATION (ULTRA_ITERATIVE_CHEST, off by
    // default). Rationale + adaptivity derivation: src/ofdm/iterative_chest.hpp.
    // Every member below is inert while data_aided_enabled_ is false, and the
    // three behavioural hooks (flat storage domain, absolute time base, skipped
    // history reset) are each guarded so the knob-off path is byte-identical.
    // ======================================================================
    bool data_aided_enabled_ = false;

    // ABSOLUTE symbol index of this frame's first data symbol. The Wiener time
    // axis is otherwise `current_data_symbol_index_`, which RESETS TO 0 EVERY
    // FRAME — carrying observations forward on that axis would present a
    // 0.35 s-old sample at dt = 0, the Wiener would weight it rho = 1.0, and the
    // estimate would get silently WORSE in exactly the confident-wrong direction
    // 16QAM cannot absorb. Zero (and therefore a no-op) unless armed.
    int64_t wiener_symbol_base_ = 0;

    // Storage domain of wiener_pilot_history_. Observations from DIFFERENT frames
    // carry different LTS timing offsets, so their raw H differ by a phase RAMP
    // exp(-j*2*pi*k*delta/N). The frequency stage de-slopes with the CURRENT
    // frame's slope, which removes that ramp only for the current frame. When
    // carrying across frames every sample is therefore stored already de-sloped
    // by the slope in force at capture ("flat"), and the reader skips its
    // de-slope. One uniform convention — never a mix within one carrier.
    bool wiener_history_flat_ = false;

    // Keep wiener_pilot_history_ across the next frame boundary.
    bool wiener_carry_armed_ = false;

    // Retained receive grid for the frame currently being demodulated: Y at the
    // data carriers of each data symbol, plus the logical carrier index each
    // entry sits on (the scattered-pilot pattern ROTATES with symbol index, so
    // the mapping is per-symbol and must not be recomputed later).
    std::vector<std::vector<Complex>> da_rx_grid_;
    std::vector<std::vector<uint16_t>> da_logical_;
    // Identity of the retained grid: the absolute sample origin of the frame it
    // came from. The ingest call must present the same origin, otherwise a stale
    // Y would be divided by a fresh X — the one way this feature can fabricate a
    // confident, completely wrong channel.
    long long da_grid_origin_ = -1;
    long long da_pending_origin_ = -1;
    // Pilot-referenced noise normalisation of the retained grid (noise_variance /
    // mean |H_pilot|^2) — the SAME quantity the pilot observations are pushed
    // with, so data-aided and pilot observations are commensurable.
    float da_noise_norm_ = 1.0f;
    float da_lts_phase_slope_ = 0.0f;

    // Diagnostic-only failure attribution for coherent OFDM frames. These
    // fields are reset per received frame and never feed decode decisions.
    struct FailureAttributionCarrier {
        int logical_carrier = 0;
        int fft_index = 0;
        size_t samples = 0;
        double evm_sum = 0.0;
        double norm_evm_sum = 0.0;
        double abs_h_sum = 0.0;
        double snr_db_sum = 0.0;
        size_t inside_noise = 0;
    };
    struct FailureAttributionSymbol {
        size_t symbol_index = 0;
        size_t samples = 0;
        double evm_sum = 0.0;
        double norm_evm_sum = 0.0;
        size_t inside_noise = 0;
        float min_abs_h = 0.0f;
        float mean_abs_h = 0.0f;
        float mean_snr_db = 0.0f;
        float cpe_rad = 0.0f;
        float phase_slope_rad_per_bin = 0.0f;
        float phase_ramp_edge_rad = 0.0f;
    };
    size_t current_data_symbol_index_ = 0;
    std::vector<FailureAttributionCarrier> failure_diag_carriers_;
    std::vector<FailureAttributionSymbol> failure_diag_symbols_;
    std::vector<float> failure_diag_evm_;
    std::vector<float> failure_diag_norm_evm_;
    double failure_diag_empirical_var_sum_ = 0.0;
    size_t failure_diag_empirical_var_count_ = 0;
    float failure_diag_last_symbol_empirical_var_ = 0.0f;
    bool failure_diag_last_symbol_empirical_valid_ = false;
    double failure_diag_llr_sigma2_sum_ = 0.0;
    size_t failure_diag_llr_sigma2_count_ = 0;
    double failure_diag_llr_base_sigma2_sum_ = 0.0;
    double failure_diag_inner_evm2_sum_ = 0.0;
    size_t failure_diag_inner_evm2_count_ = 0;
    double failure_diag_outer_evm2_sum_ = 0.0;
    size_t failure_diag_outer_evm2_count_ = 0;

    // LTS time-domain reference for fine timing (passband templates)
    std::vector<float> lts_passband_I;
    std::vector<float> lts_passband_Q;

    // Phase inversion detection
    bool llr_sign_flip = false;

    // Manual timing offset adjustment
    int manual_timing_offset = 0;

    // Adaptive equalizer state
    std::vector<Complex> lms_weights;
    std::vector<Complex> last_decisions;
    std::vector<float> rls_P;

    // Per-carrier noise variance after equalization
    std::vector<float> carrier_noise_var;

    // Measured total-error LLR noise model (ULTRA_LLR_MEASURED_NV, equalize(),
    // math in measured_llr_noise.hpp). Running per-pass sums, reset with the
    // other per-frame equalizer state (ofdm_stream_processor.cpp):
    //   residual: received-domain decision residual |Y_k - H_k * xhat_k|^2
    //   u       : standardized residual (cell residual / its expected variance)
    double measured_residual_sum_ = 0.0;
    double measured_residual_cells_ = 0.0;
    double measured_u_sum_ = 0.0;
    double measured_u2_sum_ = 0.0;
    double measured_u_cells_ = 0.0;
    std::vector<float> measured_cell_r2_;  // this symbol's residual per data carrier (-1 = n/a)
    void resetMeasuredResidual() {
        measured_residual_sum_ = 0.0;
        measured_residual_cells_ = 0.0;
        measured_u_sum_ = 0.0;
        measured_u2_sum_ = 0.0;
        measured_u_cells_ = 0.0;
    }

    // RX-local hard-erasure decisions for the current data symbol.
    // Set by equalize() from gamma_k = |H_k|^2 / sigma^2, consumed by
    // demodulateSymbol() to write exact zero LLRs.
    bool rx_carrier_erasure_enabled_ = false;
    std::vector<uint8_t> carrier_erasure_flags_;

    // Per-carrier adaptive LLR scaling: track |equalized| stability over symbols
    // Stable carriers keep full LLR confidence; fading carriers get inflated noise
    std::vector<float> carrier_eq_mag_ema_;   // EMA of |equalized[i]| per carrier
    std::vector<float> carrier_eq_mag_var_;   // EMA of (|eq| - ema)² per carrier

    // ==========================================================================
    // CONSTRUCTOR
    // ==========================================================================
    Impl(const ModemConfig& cfg);

    // ==========================================================================
    // INITIALIZATION (demodulator.cpp)
    // ==========================================================================
    void setupCarriers();
    void activateCarrierPattern(size_t symbol_index);
    void generateSequences();
    void buildInterpTable();
    void buildInterpolationPhasors();

    // ==========================================================================
    // SYNC DETECTION (ofdm_sync.cpp)
    // ==========================================================================
    bool hasMinimumEnergy(size_t offset, size_t window_len);
    std::vector<Complex> toAnalytic(const float* samples, size_t len);
    float measureRealCorrelation(size_t offset, float* out_energy = nullptr);
    float measureSchmidlCoxCorrelation(size_t offset, Complex* out_P = nullptr, float* out_energy = nullptr);
    float measureAnalyticCorrelation(size_t offset, Complex* out_P = nullptr, float* out_energy = nullptr);
    float measureCorrelation(size_t offset, float* out_energy = nullptr);
    bool detectSync(size_t offset);
    float estimateCoarseCFO(size_t sync_offset);
    float estimateCFOFromTraining(const float* samples, size_t num_symbols, float coarse_cfo_hz = 0.0f);
    size_t refineLTSTiming(size_t coarse_sts_start);
    std::vector<float> trialDemodulate(size_t data_start_offset, size_t num_symbols);
    std::pair<bool, int> huntForCodeword(size_t candidate_sync_pos);

    // ==========================================================================
    // CHANNEL ESTIMATION & EQUALIZATION (channel_equalizer.cpp)
    // ==========================================================================
    const std::vector<Complex>& toBaseband(SampleSpan samples);
    const std::vector<Complex>& extractSymbol(const std::vector<Complex>& baseband, size_t offset);
    void updateChannelEstimate(const std::vector<Complex>& freq_domain);
    void estimateChannelFromLTS(const float* training_samples, size_t num_symbols);
    void updateLastSNREstimate(float signal_power, float noise_power,
                               size_t independent_bins, float alpha,
                               bool fitted_common_gain = false,
                               bool noise_reference_only = false,
                               float noise_power_reference_scale = 1.0f);
    void resetPilotFadingStats();
    void updatePilotFadingStats(const std::vector<Complex>& h_ls_all);
    float computePilotFadingIndexFromStats() const;
    void resetWienerPilotHistory();
    void seedWienerPilotHistoryFromCurrentChannel(int64_t symbol_index);
    void addWienerPilotObservation(size_t logical_carrier,
                                   int64_t symbol_index,
                                   Complex h,
                                   float noise_norm);
    // Align carried observations to the NEW frame's phase reference (ML 1-parameter fit).
    void rereferenceCarriedHistoryPhase();
    // Signed FFT bin of a logical carrier (used by the flat storage domain).
    int signedBinForLogicalCarrier(size_t logical_carrier) const;
    // Post-FEC data-aided ingest: X grid in, observations pushed out.
    size_t ingestDataAidedGrid(const Symbol& x_grid, long long expect_origin);
    Complex estimateWienerChannel(size_t logical_carrier,
                                  int64_t symbol_index,
                                  float noise_norm,
                                  Complex fallback,
                                  float* out_error_var);
    void interpolateChannel();
    Complex hardDecision(Complex sym, Modulation mod) const;
    void resetFailureAttributionDiagnostics();
    void recordFailureAttributionSymbol(const std::vector<Complex>& equalized, Modulation mod);
    std::string getFailureAttributionDiagnosticsText() const;
    bool qam16FailureAttributionDiagEnabled() const;
    bool qam16GenieSigmaEmpiricalEnabled() const;
    bool qam16GenieChannelTwoPathEnabled() const;
    bool genieLtsFreezeEnabled() const;
    void applyDiagnosticTwoPathChannelOracle(const std::vector<Complex>& h_ls_all);
    void lmsUpdate(int idx, Complex received, Complex reference);
    void rlsUpdate(int idx, Complex received, Complex reference);
    const std::vector<Complex>& equalize(const std::vector<Complex>& freq_domain, Modulation mod);
    const std::vector<Complex>& equalize(const std::vector<Complex>& freq_domain);

    // ==========================================================================
    // DEMODULATION (demodulator.cpp)
    // ==========================================================================
    void demodulateSymbol(const std::vector<Complex>& equalized, Modulation mod);
    // Append symbols to the constellation buffer, resetting it (and recording the
    // new modulation) whenever `mod` differs from the last batch — keeps the GUI
    // constellation homogeneous per modulation instead of a mixed cloud.
    void appendConstellationSymbols(const std::vector<Complex>& update, Modulation mod);
    void updateQuality();
};

} // namespace ultra
