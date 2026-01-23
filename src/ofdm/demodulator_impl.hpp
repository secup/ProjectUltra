#pragma once

// OFDMDemodulator::Impl - Private implementation struct
// Split across multiple .cpp files for maintainability

#include "ultra/ofdm.hpp"
#include "ultra/dsp.hpp"
#include "demodulator_constants.hpp"
#include <atomic>
#include <mutex>
#include <vector>

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
    std::vector<Complex> pilot_sequence;

    // Synchronization state (atomic for thread-safe UI access)
    enum class State { SEARCHING, SYNCED };
    std::atomic<State> state{State::SEARCHING};
    std::atomic<int> synced_symbol_count{0};
    std::atomic<int> idle_call_count{0};

    // Sample buffer
    Samples rx_buffer;
    size_t symbol_samples;

    // Channel estimate (per carrier)
    std::vector<Complex> channel_estimate;
    float noise_variance = 0.1f;

    // SNR estimation (from pilots)
    float estimated_snr_linear = 1.0f;
    float snr_alpha = 0.3f;
    int snr_symbol_count = 0;

    // Output data
    Bytes demod_data;
    std::vector<float> soft_bits;
    ChannelQuality quality;

    // Constellation display (latest equalized symbols)
    std::vector<Complex> constellation_symbols;
    mutable std::mutex constellation_mutex;

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

    // Frequency offset estimation and correction
    float freq_offset_hz = 0.0f;
    float freq_offset_filtered = 0.0f;
    std::vector<Complex> prev_pilot_phases;
    int symbols_since_sync = 0;
    float freq_correction_phase = 0.0f;

    // Pilot-based phase tracking for differential modulation
    Complex pilot_phase_correction = Complex(1, 0);

    // Symbol timing recovery
    float timing_offset_samples = 0.0f;

    // Carrier phase recovery
    Complex carrier_phase_correction = Complex(1, 0);
    bool carrier_phase_initialized = false;

    // Per-carrier phase from LTS
    std::vector<Complex> lts_carrier_phases;

    // LTS time-domain reference for fine timing (passband templates)
    std::vector<float> lts_passband_I;
    std::vector<float> lts_passband_Q;

    // Phase inversion detection
    bool llr_sign_flip = false;

    // Manual timing offset adjustment
    int manual_timing_offset = 0;

    // DBPSK state
    std::vector<Complex> dbpsk_prev_equalized;
    bool dbpsk_first_symbol = true;
    bool dqpsk_skip_first_symbol = false;

    // Adaptive equalizer state
    std::vector<Complex> lms_weights;
    std::vector<Complex> last_decisions;
    std::vector<float> rls_P;

    // Per-carrier noise variance after equalization
    std::vector<float> carrier_noise_var;

    // ==========================================================================
    // CONSTRUCTOR
    // ==========================================================================
    Impl(const ModemConfig& cfg);

    // ==========================================================================
    // INITIALIZATION (demodulator.cpp)
    // ==========================================================================
    void setupCarriers();
    void generateSequences();
    void buildInterpTable();

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
    size_t refineLTSTiming(size_t coarse_sts_start);
    std::vector<float> trialDemodulate(size_t data_start_offset, size_t num_symbols);
    std::pair<bool, int> huntForCodeword(size_t candidate_sync_pos);

    // ==========================================================================
    // CHANNEL ESTIMATION & EQUALIZATION (channel_equalizer.cpp)
    // ==========================================================================
    std::vector<Complex> toBaseband(SampleSpan samples);
    std::vector<Complex> extractSymbol(const std::vector<Complex>& baseband, size_t offset);
    void updateChannelEstimate(const std::vector<Complex>& freq_domain);
    void interpolateChannel();
    Complex hardDecision(Complex sym, Modulation mod) const;
    void lmsUpdate(int idx, Complex received, Complex reference);
    void rlsUpdate(int idx, Complex received, Complex reference);
    std::vector<Complex> equalize(const std::vector<Complex>& freq_domain, Modulation mod);
    std::vector<Complex> equalize(const std::vector<Complex>& freq_domain);

    // ==========================================================================
    // DEMODULATION (demodulator.cpp)
    // ==========================================================================
    void demodulateSymbol(const std::vector<Complex>& equalized, Modulation mod);
    void updateQuality();
};

} // namespace ultra
