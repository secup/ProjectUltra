#pragma once

// MCDPSKWaveform - Multi-Carrier DPSK waveform implementation
//
// Wraps MultiCarrierDPSKModulator, MultiCarrierDPSKDemodulator, and ChirpSync
// to provide the IWaveform interface.
//
// Features:
// - Chirp preamble for robust sync at low SNR (-3 to +10 dB)
// - Multi-carrier DQPSK with frequency diversity
// - CFO-tolerant via complex correlation chirp detection
// - Used for connection establishment (CONNECT/DISCONNECT)

#include "waveform_interface.hpp"
#include "psk/multi_carrier_dpsk.hpp"
#include "sync/chirp_sync.hpp"
#include <memory>

namespace ultra {

class MCDPSKWaveform : public IWaveform {
public:
    // Create with default configuration (8 carriers)
    MCDPSKWaveform();

    // Create with specific carrier count
    explicit MCDPSKWaveform(int num_carriers);

    // Create with full configuration
    explicit MCDPSKWaveform(const MultiCarrierDPSKConfig& config);

    ~MCDPSKWaveform() override = default;

    // ========================================================================
    // IWaveform - Identity
    // ========================================================================

    std::string getName() const override { return "MC-DPSK"; }
    protocol::WaveformMode getMode() const override { return protocol::WaveformMode::MC_DPSK; }
    WaveformCapabilities getCapabilities() const override;

    // ========================================================================
    // IWaveform - Configuration
    // ========================================================================

    void configure(Modulation mod, CodeRate rate) override;
    void setFrequencyOffset(float cfo_hz) override;
    void setTxFrequencyOffset(float cfo_hz) override;
    Modulation getModulation() const override { return modulation_; }
    CodeRate getCodeRate() const override { return code_rate_; }
    float getFrequencyOffset() const override { return cfo_hz_; }

    // ========================================================================
    // IWaveform - TX
    // ========================================================================

    Samples generatePreamble() override;
    Samples modulate(const Bytes& encoded_data) override;

    // ========================================================================
    // IWaveform - RX
    // ========================================================================

    bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.15f) override;
    bool process(SampleSpan samples) override;
    std::vector<float> getSoftBits() override;
    // Untracked soft bits of the last demodulated frame when the residual tracker applied a
    // correction (else empty); see MultiCarrierDPSKDemodulator::takeUntrackedSoftBits.
    std::vector<float> takeUntrackedSoftBits();
    void reset() override;

    // Demodulate MC-DPSK continuation samples that follow an already-processed
    // training/reference symbol. Keeps the differential phase cursor intact.
    bool processDataOnly(SampleSpan samples);

    // ========================================================================
    // IWaveform - Status
    // ========================================================================

    bool isSynced() const override;
    bool hasData() const override;
    bool hasEstimatedSNR() const { return last_snr_valid_; }
    float estimatedSNR() const override;
    // #58 data-aided fade-averaged SNR over the last demodulated frame's whole
    // data span (vs the ~170 ms training snapshot in estimatedSNR()). The
    // consumer must gate on the frame's LDPC decode success before routing it.
    bool hasDataAidedSNR() const { return last_data_aided_snr_valid_; }
    float getDataAidedSNRdB() const { return last_data_aided_snr_; }
    // Physical channel SNR plumbing (handoff §2): the decoder supplies the
    // burst-time noise reference before decode; the demodulator computes the
    // power-ratio SNR over the exact training span it processes.
    void setNoiseReferenceRMS(float rms) override {
        if (demodulator_) demodulator_->setNoiseReferenceRMS(rms);
    }
    bool hasPhysicalSNR() const override {
        return demodulator_ && demodulator_->hasPhysicalSNR();
    }
    float getPhysicalSNRdB() const override {
        return demodulator_ ? demodulator_->getPhysicalSNRdB() : 0.0f;
    }
    float estimatedCFO() const override;
    std::vector<std::complex<float>> getConstellationSymbols() const override;

    // ========================================================================
    // IWaveform - GUI Display
    // ========================================================================

    std::string getStatusString() const override;
    int getCarrierCount() const override { return config_.num_carriers; }
    float getThroughput(CodeRate rate) const override;
    int getSamplesPerSymbol() const override { return config_.samples_per_symbol; }
    int getPreambleSamples() const override;
    int getMinSamplesForFrame() const override;
    int getMinSamplesForCWCount(int num_cw) const override;
    int getDataOnlySamplesForCWCount(int num_cw) const;

    // ========================================================================
    // MC-DPSK Specific
    // ========================================================================

    // Set number of carriers (3-20)
    void setCarrierCount(int carriers);

    // Get configuration
    const MultiCarrierDPSKConfig& getConfig() const { return config_; }

    // ========================================================================
    // Channel Quality / Fading Detection (IWaveform override)
    // ========================================================================

    // Get fading index (0-1, higher = more fading)
    // Based on per-carrier signal magnitude variance
    float getFadingIndex() const override;

    // Check if channel appears to be fading (uses threshold 0.65)
    bool isFading() const override;

private:
    void initComponents();

    MultiCarrierDPSKConfig config_;
    std::unique_ptr<MultiCarrierDPSKModulator> modulator_;
    std::unique_ptr<MultiCarrierDPSKDemodulator> demodulator_;
    std::unique_ptr<sync::ChirpSync> chirp_sync_;

    // State
    Modulation modulation_ = Modulation::DQPSK;
    CodeRate code_rate_ = CodeRate::R1_4;
    float cfo_hz_ = 0.0f;
    bool synced_ = false;
    std::vector<float> soft_bits_;
    float last_snr_ = 0.0f;
    bool last_snr_valid_ = false;
    // #58: whole-frame data-aided fade-averaged SNR (sibling of last_snr_).
    float last_data_aided_snr_ = 0.0f;
    bool last_data_aided_snr_valid_ = false;
    float last_cfo_ = 0.0f;
};

} // namespace ultra
