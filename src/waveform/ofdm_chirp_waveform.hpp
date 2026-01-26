#pragma once

// OFDMChirpWaveform - OFDM with chirp synchronization
//
// Combines robust chirp sync with OFDM modulation for mid-range SNR.
// Works at 10-17 dB where Schmidl-Cox struggles but DPSK is too slow.
//
// Features:
// - Chirp preamble for robust sync at lower SNR than Schmidl-Cox
// - DQPSK modulation (differential, no pilots needed)
// - Good fading performance with frequency diversity
// - CFO-tolerant via complex correlation chirp detection

#include "waveform_interface.hpp"
#include "ultra/ofdm.hpp"
#include "sync/chirp_sync.hpp"
#include <memory>

namespace ultra {

class OFDMChirpWaveform : public IWaveform {
public:
    // Create with default configuration
    OFDMChirpWaveform();

    // Create with specific config
    explicit OFDMChirpWaveform(const ModemConfig& config);

    ~OFDMChirpWaveform() override = default;

    // ========================================================================
    // IWaveform - Identity
    // ========================================================================

    std::string getName() const override { return "OFDM-Chirp"; }
    protocol::WaveformMode getMode() const override { return protocol::WaveformMode::OFDM_CHIRP; }
    WaveformCapabilities getCapabilities() const override;

    // ========================================================================
    // IWaveform - Configuration
    // ========================================================================

    void configure(Modulation mod, CodeRate rate) override;
    void setFrequencyOffset(float cfo_hz) override;
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

    bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.15f) override;
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

    // ========================================================================
    // OFDM-Chirp Specific
    // ========================================================================

    // Get internal config
    const ModemConfig& getConfig() const { return config_; }

    // Get chirp sync for direct access
    sync::ChirpSync* getChirpSync() { return chirp_sync_.get(); }

private:
    void initComponents();
    sync::ChirpConfig getChirpConfig() const;

    ModemConfig config_;
    std::unique_ptr<OFDMModulator> modulator_;
    std::unique_ptr<OFDMDemodulator> demodulator_;
    std::unique_ptr<sync::ChirpSync> chirp_sync_;

    // State
    float cfo_hz_ = 0.0f;
    float last_snr_ = 0.0f;
    float last_cfo_ = 0.0f;
    bool synced_ = false;
    std::vector<float> soft_bits_;
};

} // namespace ultra
