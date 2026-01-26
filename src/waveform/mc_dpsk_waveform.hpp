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
    int getCarrierCount() const override { return config_.num_carriers; }
    float getThroughput(CodeRate rate) const override;
    int getSamplesPerSymbol() const override { return config_.samples_per_symbol; }
    int getPreambleSamples() const override;

    // ========================================================================
    // MC-DPSK Specific
    // ========================================================================

    // Set number of carriers (3-20)
    void setCarrierCount(int carriers);

    // Get configuration
    const MultiCarrierDPSKConfig& getConfig() const { return config_; }

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
    float last_cfo_ = 0.0f;
};

} // namespace ultra
