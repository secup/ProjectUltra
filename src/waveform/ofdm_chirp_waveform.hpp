#pragma once

// OFDMChirpWaveform - OFDM with chirp synchronization
//
// Combines robust chirp sync with OFDM modulation for mid-range SNR.
// Works at 10-17 dB where Schmidl-Cox struggles but DPSK is too slow.
//
// Features:
// - Chirp preamble for robust sync at lower SNR than Schmidl-Cox
// - DPSK modulation for robust fading paths
// - Pilot-aided coherent QAM16 for high-SNR data paths
// - Good fading performance with frequency diversity
// - CFO-tolerant via complex correlation chirp detection

#include "waveform_interface.hpp"
#include "ultra/dsp.hpp"
#include "ultra/ofdm.hpp"
#include "sync/chirp_sync.hpp"
#include <memory>

namespace ultra {

class OFDMChirpWaveform : public IWaveform {
public:
    // Create with default configuration (wideband OFDM_CHIRP)
    OFDMChirpWaveform();

    // Create with specific config (defaults to OFDM_CHIRP mode)
    explicit OFDMChirpWaveform(const ModemConfig& config);

    // Create with specific config and explicit waveform mode
    // Use OFDM_NARROW for narrowband operation
    OFDMChirpWaveform(const ModemConfig& config, protocol::WaveformMode mode);

    ~OFDMChirpWaveform() override = default;

    // ========================================================================
    // IWaveform - Identity
    // ========================================================================

    std::string getName() const override {
        return mode_ == protocol::WaveformMode::OFDM_NARROW ? "OFDM-Narrow" : "OFDM-Chirp";
    }
    protocol::WaveformMode getMode() const override { return mode_; }
    WaveformCapabilities getCapabilities() const override;

    // ========================================================================
    // IWaveform - Configuration
    // ========================================================================

    void configure(Modulation mod, CodeRate rate) override;
    void setFrequencyOffset(float cfo_hz) override;
    void setTxFrequencyOffset(float cfo_hz) override;
    Modulation getModulation() const override { return config_.modulation; }
    CodeRate getCodeRate() const override { return config_.code_rate; }
    float getFrequencyOffset() const override { return cfo_hz_; }

    // ========================================================================
    // IWaveform - TX
    // ========================================================================

    Samples generatePreamble() override;
    Samples generateDataPreamble() override;  // Training only, no chirp
    Samples generateSettlingSymbols(size_t count) override;
    Samples modulate(const Bytes& encoded_data) override;
    void setCarrierMask(uint64_t active_mask) override;
    uint64_t getCarrierMask() const override { return carrier_mask_; }
    void setCarrierLdpcInterleaverEnabled(bool enabled) override;
    void setActiveLDPCLiftingZ(uint8_t z) override {
        ldpc_lifting_z_ = (z == 81) ? 81 : 27;  // only 27/81 allowed; defensive
        // 2026-05-28: propagate to the OFDM demodulator so its "we have a
        // codeword" gate (active_ldpc_block_size) waits for the FULL Z=81
        // codeword (1944 bits) instead of returning after Z=27 (648 bits).
        // Without this the burst deinterleaver gets 1296 bits per frame
        // (2x648) when it expects 3888 (2x1944) and throws.
        if (demodulator_) {
            demodulator_->setActiveLDPCBlockSize(ldpc_lifting_z_ == 81 ? 1944 : 648);
        }
    }

    // ========================================================================
    // IWaveform - RX
    // ========================================================================

    bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.15f) override;
    bool detectDataSync(SampleSpan samples, SyncResult& result,
                        float known_cfo_hz = 0.0f, float threshold = 0.3f) override;
    bool supportsDataPreamble() const override { return true; }
    size_t frameStructureSpanSamples() const override;
    bool detectFrameStructure(SampleSpan samples, SyncResult& result, float known_cfo_hz,
                              double false_alarm_budget) override;
    void setAbsoluteTrainingPosition(size_t pos) override;
    bool process(SampleSpan samples) override;
    std::vector<float> getSoftBits() override;
    void setRXCarrierErasureEnabled(bool enabled) override;
    void reset() override;

    // ========================================================================
    // POST-FEC DATA-AIDED CHANNEL ESTIMATION (ULTRA_ITERATIVE_CHEST, default OFF)
    // src/ofdm/iterative_chest.hpp carries the rationale.
    // ========================================================================
    void setDataAidedFeedbackEnabled(bool enabled) override;
    void setChannelHistoryFrameOrigin(long long abs_sample) override;
    void armChannelHistoryCarry(bool armed) override;
    void clearChannelHistory() override;
    size_t ingestDataAidedFrame(const Bytes& encoded_air_bytes) override;

    // Replay the TRANSMIT chain on already-encoded air bytes and return the
    // resulting data-carrier constellation grid (symbol-major, data-carriers per
    // symbol), stopping before the IFFT. This is the PRODUCTION modulate() path —
    // same CarrierLDPC eligibility, same pilot/carrier-pattern rotation, same
    // mapBits — so TX and re-modulation cannot drift apart. Public because
    // tests/test_iterative_chest_remod.cpp pins it bit-exactly against a real
    // transmit; that round-trip is the single highest-risk correctness point of
    // the whole feature.
    Symbol remodulateDataCarrierSymbols(const Bytes& encoded_air_bytes);

    // ========================================================================
    // IWaveform - Status
    // ========================================================================

    bool isSynced() const override;
    bool hasData() const override;
    float estimatedSNR() const override;
    bool hasLastOFDMBroadbandSNREstimate() const override;
    float getLastOFDMBroadbandSNREstimate() const override;
    bool hasEvmSnrEstimate() const override;
    float getEvmSnrDbEstimate() const override;
    bool takeEvmSnrEstimate(float& evm_snr_db, size_t& carrier_count) override;
    float estimatedCFO() const override;
    float getFadingIndex() const override;  // From demodulator pilot variance
    float getLastPilotFrequencyCV() const override;
    float getLastPilotTemporalCV() const override;
    float getLastPilotSymbolMeanCV() const override;
    float getLastLTSSignalPower() const override;
    float getLastLTSChannelMagnitude() const override;
    std::vector<float> getCarrierGammaSnapshot() const override;
    float getLastLTSNoiseVariance() const override;
    float getLastLTSResidualCFOHz() const override;
    float getLastLTSPairChi() const override;
    void setWienerChannelParams(float doppler_hz, float delay_spread_s) override;
    std::string getFailureAttributionDiagnosticsText() const override;
    std::vector<std::complex<float>> getConstellationSymbols() const override;
    Modulation getConstellationModulation() const override;

    // ========================================================================
    // IWaveform - GUI Display
    // ========================================================================

    std::string getStatusString() const override;
    int getCarrierCount() const override { return static_cast<int>(config_.num_carriers); }
    float getThroughput(CodeRate rate) const override;
    int getSamplesPerSymbol() const override;
    int getPreambleSamples() const override;
    int getDataPreambleSamples() const override;  // Training only
    int getMinSamplesForFrame() const override;
    int getMinSamplesForControlFrame() const override;
    int getMinSamplesForCWCount(int num_cw) const override;

    // ========================================================================
    // OFDM-Chirp Specific
    // ========================================================================

    // Burst interleave marker: true if last detectDataSync() found negated LTS
    bool wasBurstInterleaved() const override { return burst_interleave_latched_; }
    float getLastTimingOffsetSamples() const override;

    // Pilot spacing for interleaver geometry
    int getPilotSpacing() const override { return config_.pilot_spacing; }

    // Get internal config
    const ModemConfig& getConfig() const { return config_; }

    // Get chirp sync for direct access
    sync::ChirpSync* getChirpSync() { return chirp_sync_.get(); }

    // Phase 2a short-anchor (see waveform_interface.hpp). Gated by ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS.
    // detectSync() auto-falls-back to the short detector on a full-detector miss, so no RX
    // routing flag is needed.
    Samples generateShortAnchorPreamble() override;
    bool shortAnchorEnabled() const override;

    Symbol getLastDataCarrierSymbolsForTesting() const;

private:
    void initComponents();
    sync::ChirpConfig getChirpConfig() const;
    sync::ChirpConfig getShortAnchorChirpConfig() const;   // short DUAL chirp, warm re-anchor
    static float shortAnchorChirpMs();                     // ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS (0=off)
    void configurePilotsForCodeRate(CodeRate rate);
    bool carrierLdpcPlumbingEligible() const;
    bool carrierLdpcCodewordCountSupported(size_t codeword_count) const;
    void invalidateDataSyncTemplate();
    bool ensureDataSyncTemplate(int symbol_samples);

    protocol::WaveformMode mode_ = protocol::WaveformMode::OFDM_CHIRP;
    ModemConfig config_;
    std::unique_ptr<OFDMModulator> modulator_;
    std::unique_ptr<OFDMDemodulator> demodulator_;
    std::unique_ptr<sync::ChirpSync> chirp_sync_;
    std::unique_ptr<sync::ChirpSync> short_anchor_chirp_sync_;  // short dual chirp (warm descriptor)
    HilbertTransform data_sync_hilbert_{65};
    std::vector<Complex> data_sync_analytic_scratch_;
    // Preamble-less structure acquisition (ULTRA_BURST_ACQ=structure): see
    // detectFrameStructure() in the .cpp for the statistic and its noise law.
    std::unique_ptr<FFT> structure_fft_;
    std::vector<Complex> structure_fft_in_, structure_fft_out_;
    std::vector<Complex> data_sync_template_analytic_;
    float data_sync_template_energy_ = 0.0f;
    int data_sync_template_symbol_samples_ = 0;

    // State
    float cfo_hz_ = 0.0f;
    float last_snr_ = 0.0f;
    float last_cfo_ = 0.0f;
    uint64_t carrier_mask_ = UINT64_MAX;
    bool carrier_ldpc_interleaver_enabled_ = false;
    // ULTRA_ITERATIVE_CHEST (default OFF). data_aided_frame_origin_ is the absolute
    // sample origin last announced for a frame; the ingest presents it back to the
    // demodulator, which refuses any grid that did not come from that same frame.
    bool data_aided_feedback_enabled_ = false;
    long long data_aided_frame_origin_ = -1;
    // 2026-05-28 Phase 3: active LDPC lifting Z for OFDM data-frame sizing.
    // Set per-burst by the connection layer; consumed by getMinSamplesForCWCount
    // (frame_bits = num_cw * (z==81 ? 1944 : 648)). Default 27 keeps the
    // legacy short-LDPC airtime accounting for pre-burst / non-burst frames.
    uint8_t ldpc_lifting_z_ = 27;
    bool rx_carrier_erasure_enabled_ = false;
    bool synced_ = false;
    std::vector<float> soft_bits_;

    // Sample position where training starts (from detectSync)
    // Used to calculate initial CFO phase for correct phase tracking
    size_t training_start_sample_ = 0;
    size_t absolute_training_start_sample_ = 0;
    bool has_absolute_training_start_sample_ = false;

    // Burst interleave marker detection (LTS sign)
    // Two-flag design:
    //   burst_interleaved_detected_: consumed by process() to undo LTS negation (one-shot)
    //   burst_interleave_latched_: readable by streaming_decoder after process() returns
    bool burst_interleaved_detected_ = false;
    bool burst_interleave_latched_ = false;
};

} // namespace ultra
