#pragma once

// OFDMNvisWaveform - High-speed OFDM with Schmidl-Cox synchronization
//
// Wraps OFDMModulator and OFDMDemodulator for high-throughput operation.
// Requires ~17 dB SNR for reliable sync detection.
//
// Features:
// - Schmidl-Cox preamble (STS + LTS) for sync and CFO estimation
// - Supports coherent modulation (QPSK, 8PSK, 16QAM, 32QAM)
// - Pilot-assisted channel estimation
// - Best for good/stable HF channels (NVIS, short paths)

#include "waveform_interface.hpp"
#include "ultra/ofdm.hpp"
#include <memory>

namespace ultra {

class OFDMNvisWaveform : public IWaveform {
public:
    // Create with default configuration (512 FFT, 30 carriers)
    OFDMNvisWaveform();

    // Create with specific config
    explicit OFDMNvisWaveform(const ModemConfig& config);

    ~OFDMNvisWaveform() override = default;

    // ========================================================================
    // IWaveform - Identity
    // ========================================================================

    std::string getName() const override { return "OFDM-COX"; }
    protocol::WaveformMode getMode() const override { return protocol::WaveformMode::OFDM_COX; }
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
    Samples modulate(const Bytes& encoded_data) override;

    // ========================================================================
    // IWaveform - RX
    // ========================================================================

    bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.8f) override;
    bool process(SampleSpan samples) override;
    std::vector<float> getSoftBits() override;
    void reset() override;

    // ========================================================================
    // IWaveform - Status
    // ========================================================================

    bool isSynced() const override;
    bool hasData() const override;
    float estimatedSNR() const override;
    float estimatedCFO() const override;
    std::vector<std::complex<float>> getConstellationSymbols() const override;

    // ========================================================================
    // IWaveform - GUI Display
    // ========================================================================

    std::string getStatusString() const override;
    int getCarrierCount() const override { return static_cast<int>(config_.num_carriers); }
    float getThroughput(CodeRate rate) const override;
    int getSamplesPerSymbol() const override;
    int getPreambleSamples() const override;
    int getMinSamplesForFrame() const override;

    // ========================================================================
    // OFDM-COX Specific
    // ========================================================================

    // Get internal config
    const ModemConfig& getConfig() const { return config_; }

    // Enable/disable pilots (for differential vs coherent modes)
    void setUsePilots(bool use_pilots);

private:
    void initComponents();

    ModemConfig config_;
    std::unique_ptr<OFDMModulator> modulator_;
    std::unique_ptr<OFDMDemodulator> demodulator_;

    // State
    float cfo_hz_ = 0.0f;
    std::vector<float> soft_bits_;
};

} // namespace ultra
