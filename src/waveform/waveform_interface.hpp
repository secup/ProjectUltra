#pragma once

// IWaveform - Abstract interface for all waveform types
//
// This interface enables plugin-style architecture where adding a new waveform
// only requires implementing this interface.
//
// Design principles:
// 1. Waveform owns its sync method (ChirpSync, Schmidl-Cox, etc.)
// 2. Waveform handles both TX (modulation) and RX (demodulation)
// 3. Waveform exposes status for GUI display
// 4. CFO correction is handled internally by the waveform

#include "ultra/types.hpp"
#include "protocol/frame_v2.hpp"
#include <string>
#include <vector>
#include <memory>

namespace ultra {

// Types are defined in ultra/types.hpp - included above

// Capabilities exposed by a waveform for mode selection
struct WaveformCapabilities {
    bool supports_cfo_correction = false;   // Can correct carrier frequency offset
    bool supports_doppler_correction = false; // Can handle Doppler spread
    bool requires_pilots = false;           // Needs pilot symbols (coherent modes)
    bool supports_differential = true;      // Supports differential modulation
    float min_snr_db = 0.0f;               // Minimum SNR for reliable operation
    float max_snr_db = 30.0f;              // Maximum useful SNR (above this, use higher mode)
    float max_throughput_bps = 1000.0f;    // Maximum achievable throughput
    float preamble_duration_ms = 500.0f;   // Preamble length for this waveform
};

// Result from sync detection
struct SyncResult {
    bool detected = false;
    int start_sample = -1;          // Sample position where data starts
    int preamble_start_sample = -1; // Frame START (up-chirp begin); audio before
                                    // this point is ambient — usable as an
                                    // idle-noise observation. -1 = unknown
                                    // (e.g. warm-LTS syncs with no chirp).
    int interchirp_gap_start_sample = -1; // Silent gap between up- and down-chirp:
    int interchirp_gap_len = 0;     // noise-only audio AT BURST TIME — the
                                    // per-frame noise reference (same channel
                                    // noise state + fade state as the frame;
                                    // S:N-tracking channel sims scale idle
                                    // noise differently from burst noise).
                                    // -1/0 = unavailable (single chirp, LTS).
    float correlation = 0.0f;       // Peak correlation value (0-1)
    float cfo_hz = 0.0f;            // Estimated carrier frequency offset
    float gap_error_samples = 0.0f; // Dual-chirp actual_gap - expected_gap
    float snr_estimate = 0.0f;      // Estimated SNR from preamble
    bool has_training = false;      // True if preamble includes training sequence
};

// Abstract waveform interface
class IWaveform {
public:
    virtual ~IWaveform() = default;

    // Physical channel SNR (handoff §2): burst-time noise reference in, pure
    // power-ratio channel SNR out. Default no-op — only waveforms with a
    // known-power training span (MC-DPSK) implement it.
    virtual void setNoiseReferenceRMS(float) {}
    virtual bool hasPhysicalSNR() const { return false; }
    virtual float getPhysicalSNRdB() const { return 0.0f; }

    // ========================================================================
    // IDENTITY
    // ========================================================================

    // Human-readable name (e.g., "MC-DPSK", "OFDM-COX")
    virtual std::string getName() const = 0;

    // Protocol waveform mode enum value
    virtual protocol::WaveformMode getMode() const = 0;

    // Capabilities for mode selection logic
    virtual WaveformCapabilities getCapabilities() const = 0;

    // ========================================================================
    // CONFIGURATION
    // ========================================================================

    // Set modulation scheme (QPSK, 8PSK, 16QAM, etc.) and code rate
    virtual void configure(Modulation mod, CodeRate rate) = 0;

    // Apply carrier frequency offset correction (Hz)
    // Called after sync detection with estimated CFO
    virtual void setFrequencyOffset(float cfo_hz) = 0;

    // Set TX frequency offset for testing (simulates radio tuning error)
    // Must be called BEFORE generatePreamble() and modulate()
    virtual void setTxFrequencyOffset(float cfo_hz) = 0;

    // Get current configuration
    virtual Modulation getModulation() const = 0;
    virtual CodeRate getCodeRate() const = 0;
    virtual float getFrequencyOffset() const = 0;

    // ========================================================================
    // TX PATH
    // ========================================================================

    // Generate full preamble (sync sequence + training)
    // Used for: PING, CONNECT, first frame, periodic re-sync
    virtual Samples generatePreamble() = 0;

    // Generate light preamble (training only, no sync sequence)
    // Used for: DATA frames when already connected (saves ~1.2s per frame)
    // Default: falls back to full preamble for waveforms that don't support it
    virtual Samples generateDataPreamble() { return generatePreamble(); }

    // Key-up SETTLING fill for a burst that starts WITHOUT a chirp (structure acquisition /
    // anchor skip): `count` OFDM symbols of fixed pseudo-random data at the current data
    // profile's spectrum and crest factor. On a real radio the TX ALC and the peer's RX AGC
    // settle on these (the job the 1.2 s chirp did for chirp-led bursts) instead of on the
    // first structured frame's LTS. Carries no information; default: unsupported (empty).
    virtual Samples generateSettlingSymbols(size_t count) {
        (void)count;
        return {};
    }

    // 2026-06-12 Phase 2a: SHORT single-chirp WARM re-anchor for the per-burst BURST_HEADER
    // descriptor. When sync is warm the CFO is already tracked, so the full up+down dual chirp
    // (needed only for cold CFO estimation) can collapse to a single short up-chirp — reclaiming
    // ~600 ms-1 s of airtime per burst. Default = full anchor (non-OFDM / feature off).
    // OFDMChirpWaveform overrides; gated by ULTRA_SHORT_ANCHOR_DESCRIPTOR_MS.
    virtual Samples generateShortAnchorPreamble() { return generatePreamble(); }
    virtual bool shortAnchorEnabled() const { return false; }

    // (R4: generateShortDataPreamble removed — the adaptive short re-anchor was superseded
    //  by the warm-sync hand-off, now the production default.)

    // Modulate encoded data (LDPC-encoded bits packed as bytes)
    // Returns audio samples ready for transmission
    virtual Samples modulate(const Bytes& encoded_data) = 0;

    // Optional OFDM_CHIRP carrier mask. Default is all-on and preserves legacy
    // behavior; non-OFDM waveforms ignore it.
    virtual void setCarrierMask(uint64_t active_mask) { (void)active_mask; }
    virtual uint64_t getCarrierMask() const { return UINT64_MAX; }

    // Optional OFDM_CHIRP CarrierLDPC v1 bit permutation. TX applies the
    // forward permutation before carrier mapping; RX applies the inverse before
    // LDPC decode. Non-OFDM waveforms ignore it.
    virtual void setCarrierLdpcInterleaverEnabled(bool enabled) { (void)enabled; }

    // 2026-05-28 Phase 3: active LDPC lifting size Z for OFDM data-frame sizing.
    //   27 -> n=648 coded bits per codeword (legacy short LDPC, default)
    //   81 -> n=1944 coded bits per codeword (long LDPC for OFDM data path)
    // OFDM waveforms use this to compute frame airtime in getMinSamplesForCWCount.
    // MC-DPSK and other waveforms ignore (they always use short LDPC). Set per
    // burst by the connection layer; the value is wire-announced via BURST_HEADER.
    virtual void setActiveLDPCLiftingZ(uint8_t z) { (void)z; }

    // ========================================================================
    // RX PATH
    // ========================================================================

    // Detect sync/preamble in sample buffer
    // Returns true if sync detected, fills result with details
    virtual bool detectSync(SampleSpan samples, SyncResult& result, float threshold = 0.3f) = 0;

    // Detect sync on training-only preamble (for DATA frames when connected)
    // Uses known CFO from previous frames, correlates against training pattern
    // Default: falls back to full detectSync for waveforms that don't support it
    virtual bool detectDataSync(SampleSpan samples, SyncResult& result,
                                 float known_cfo_hz = 0.0f, float threshold = 0.3f) {
        (void)known_cfo_hz;  // Unused in default implementation
        return detectSync(samples, result, threshold);
    }

    // (R4: detectShortDataSync removed — short re-anchor superseded by warm-handoff.)

    // Preamble-less FRAME-STRUCTURE acquisition (ULTRA_BURST_ACQ=structure): finds a
    // connected-mode light frame (LTS pair + data symbols) anywhere in `samples` without a
    // chirp, for a GROUP START whose arrival time is uncertain (the role the dual chirp plays
    // in chirp mode). In-group frames keep detectDataSync. `false_alarm_budget` is the
    // probability of a false detection allowed for this call (the caller derives it from its
    // search cadence). frameStructureSpanSamples() is the minimum window that holds one whole
    // structure template plus its timing refinement; 0 means unsupported.
    virtual size_t frameStructureSpanSamples() const { return 0; }
    virtual bool detectFrameStructure(SampleSpan samples, SyncResult& result,
                                      float known_cfo_hz, double false_alarm_budget) {
        (void)samples;
        (void)result;
        (void)known_cfo_hz;
        (void)false_alarm_budget;
        return false;
    }

    // Check if this waveform supports light/data preamble mode
    virtual bool supportsDataPreamble() const { return false; }

    // Process samples after sync detection
    // Returns true if a complete symbol/frame is ready
    // Call getSoftBits() to retrieve demodulated data
    virtual bool process(SampleSpan samples) = 0;

    // Set the absolute sample position where training starts (for CFO phase calc)
    // This is needed because detectSync/detectDataSync return positions relative
    // to the search buffer, but CFO phase accumulation needs the absolute position
    // in the audio stream since sample 0.
    // Default implementation does nothing (for waveforms that don't need it)
    virtual void setAbsoluteTrainingPosition(size_t pos) { (void)pos; }

    // Get soft bits from last process() call
    // Returns LLR values for LDPC decoder
    virtual std::vector<float> getSoftBits() = 0;

    // RX-local carrier erasure is waveform-specific and must be enabled only
    // by callers that know the current frame has safe multi-CW LDPC geometry.
    virtual void setRXCarrierErasureEnabled(bool enabled) { (void)enabled; }

    // ========================================================================
    // POST-FEC DATA-AIDED CHANNEL ESTIMATION (ULTRA_ITERATIVE_CHEST, default OFF)
    // See src/ofdm/iterative_chest.hpp for the full rationale. All default no-ops:
    // only OFDM_CHIRP implements them, and only when the knob is on.
    // ========================================================================

    // Arm/disarm the feature for the frames that follow. Disarming also drops any
    // retained receive grid, so a stale Y can never meet a fresh X.
    virtual void setDataAidedFeedbackEnabled(bool enabled) { (void)enabled; }

    // Absolute sample origin of the frame about to be processed. Converted to an
    // ABSOLUTE symbol index for the channel-history time axis, so an observation
    // carried from the previous frame is weighted by rho(dt) at its true age
    // instead of appearing at dt = 0.
    virtual void setChannelHistoryFrameOrigin(long long abs_sample) { (void)abs_sample; }

    // Keep the channel history across the next frame boundary (true) or let each
    // frame start from its own LTS as production does (false).
    virtual void armChannelHistoryCarry(bool armed) { (void)armed; }

    // Hard reset of the carried history — burst-group boundary, re-anchor, geometry
    // change. A carried observation is only valid inside one continuous acquisition.
    virtual void clearChannelHistory() {}

    // Feed back the EXACT air bytes of a frame that passed LDPC parity on every
    // codeword AND the frame CRC. Re-modulates them to recover X, forms
    // H_dd = Y/X on the retained receive grid, and pushes the result into the
    // channel history. Returns the number of observations accepted (0 = inert).
    virtual size_t ingestDataAidedFrame(const Bytes& encoded_air_bytes) {
        (void)encoded_air_bytes;
        return 0;
    }

    // Reset internal state (call between frames)
    virtual void reset() = 0;

    // ========================================================================
    // STATUS
    // ========================================================================

    // Is demodulator currently synced?
    virtual bool isSynced() const = 0;

    // Does demodulator have data ready?
    virtual bool hasData() const = 0;

    // Demodulator-internal SNR scale from the current signal (dB).
    virtual float estimatedSNR() const = 0;

    // Measurement-quality in-band SNR from OFDM pilot/LTS residuals. The
    // accessor name is historical. Non-OFDM waveforms return false and keep
    // their chirp-derived SNR path.
    virtual bool hasLastOFDMBroadbandSNREstimate() const { return false; }
    virtual float getLastOFDMBroadbandSNREstimate() const { return 0.0f; }

    // Radio-agnostic decision-directed EVM SNR (dB) from the equalized data
    // constellation's own error vector. Parallel/logged Stage 1; non-OFDM = false.
    virtual bool hasEvmSnrEstimate() const { return false; }
    virtual float getEvmSnrDbEstimate() const { return 0.0f; }
    // Consume one finalized physical-frame observation.  Non-OFDM waveforms
    // never publish one.  The one-shot contract prevents an erasure path that
    // did not run the demodulator from inheriting a prior frame's value.
    virtual bool takeEvmSnrEstimate(float& /*evm_snr_db*/,
                                    size_t& /*carrier_count*/) { return false; }

    // Estimated CFO from current signal (Hz)
    virtual float estimatedCFO() const = 0;

    // Fading index from per-carrier magnitude variance + temporal variation
    // Combined freq_cv + temporal_cv; > 0.65 indicates significant fading
    // Returns 0 for single-carrier modes or modes without fading detection
    virtual float getFadingIndex() const { return 0.0f; }
    virtual float getLastPilotFrequencyCV() const { return 0.0f; }
    virtual float getLastPilotTemporalCV() const { return 0.0f; }
    virtual float getLastPilotSymbolMeanCV() const { return 0.0f; }

    // Last OFDM LTS channel metrics. Non-OFDM waveforms return safe defaults.
    virtual float getLastLTSSignalPower() const { return 1.0f; }
    virtual float getLastLTSChannelMagnitude() const { return 1.0f; }
    // RX-AUTHORITY PREDICTIVE: per-data-carrier linear SNR snapshot (OFDM only).
    virtual std::vector<float> getCarrierGammaSnapshot() const { return {}; }
    virtual float getLastLTSNoiseVariance() const { return 0.0f; }
    virtual float getLastLTSResidualCFOHz() const { return 0.0f; }
    // LTS-pair structure statistic of the last processed frame (0 = unavailable).
    virtual float getLastLTSPairChi() const { return 0.0f; }

    // Push measured Wiener correlation-model params (Doppler Hz / delay spread s) from the
    // channel-class verdict (OFDM only; non-OFDM modes ignore). See the design doc.
    virtual void setWienerChannelParams(float /*doppler_hz*/, float /*delay_spread_s*/) {}

    // Check if channel appears to be fading
    virtual bool isFading() const { return getFadingIndex() > 0.65f; }

    // Burst interleave marker: true if last detectDataSync() detected negated LTS
    // Only meaningful for OFDM_CHIRP (uses LTS autocorrelation sign)
    virtual bool wasBurstInterleaved() const { return false; }

    // Last LTS-derived timing offset in samples. Positive means start later.
    virtual float getLastTimingOffsetSamples() const { return 0.0f; }

    // Post-equalizer failure-attribution diagnostics for the last decoded frame.
    virtual std::string getFailureAttributionDiagnosticsText() const { return {}; }

    // Get constellation symbols for GUI display
    virtual std::vector<std::complex<float>> getConstellationSymbols() const = 0;

    // Modulation of the symbols from getConstellationSymbols(), so the GUI can
    // draw the matching reference grid. Default QPSK for waveforms that don't
    // expose it (e.g. MC-DPSK); OFDM overrides with the buffer's actual modulation.
    virtual Modulation getConstellationModulation() const { return Modulation::QPSK; }

    // ========================================================================
    // GUI DISPLAY
    // ========================================================================

    // Status string for GUI (e.g., "MC-DPSK 8 carriers @ 375 bps")
    virtual std::string getStatusString() const = 0;

    // Pilot spacing (0 = no pilots). Used by encoder/decoder to compute data carrier count.
    virtual int getPilotSpacing() const { return 0; }

    // Number of carriers (for multi-carrier modes)
    virtual int getCarrierCount() const = 0;

    // Calculate throughput for given code rate (bps)
    virtual float getThroughput(CodeRate rate) const = 0;

    // Get samples required for one complete symbol
    virtual int getSamplesPerSymbol() const = 0;

    // Get total preamble duration in samples (full preamble with sync)
    virtual int getPreambleSamples() const = 0;

    // Get light preamble duration in samples (training only)
    // Default: same as full preamble for waveforms that don't support light mode
    virtual int getDataPreambleSamples() const { return getPreambleSamples(); }

    // (R4: getShortDataPreambleSamples removed — short re-anchor superseded by warm-handoff.)

    // Get minimum samples needed AFTER sync detection for one complete frame
    // This includes training, reference, and data for at least one codeword
    // Used by streaming receivers to know when enough samples are available
    virtual int getMinSamplesForFrame() const = 0;

    // Get minimum samples for a 1-CW control frame (ACK, NACK, etc.)
    // Default: same as full frame (override for OFDM to get shorter ACK frames)
    virtual int getMinSamplesForControlFrame() const { return getMinSamplesForFrame(); }

    // Get minimum samples for a frame with exactly N codewords
    // Used by decoder to compute exact buffer sizes after peeking CW0 header
    virtual int getMinSamplesForCWCount(int num_cw) const {
        // Default: 1 CW = control frame, N CWs = full frame
        return (num_cw <= 1) ? getMinSamplesForControlFrame() : getMinSamplesForFrame();
    }

    // Get minimum samples needed for decoder to search and decode one frame
    // This includes preamble + training + data + margin
    // Used by StreamingDecoder for buffer threshold in connected mode
    virtual int getMinSamplesForSearch() const {
        // Default: preamble + frame + 20% margin
        return static_cast<int>((getPreambleSamples() + getMinSamplesForFrame()) * 1.2);
    }
};

// Convenience alias
using WaveformPtr = std::unique_ptr<IWaveform>;

} // namespace ultra
