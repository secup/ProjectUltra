#pragma once

#include "ultra/dsp.hpp"

#include <array>
#include <cstddef>
#include <mutex>

namespace ultra {
namespace gui {

class IdleNoiseSNREstimator {
public:
    // ~3 s of idle windows for the min-statistics floor: long enough to span
    // noise bursts, short enough to track slow real-channel drift.
    static constexpr size_t kFloorWindowCount = 15;

    struct Config {
        float sample_rate = 48000.0f;
        size_t window_samples = 9600;  // 200 ms at 48 kHz
        float ema_alpha = 0.30f;
        int filter_taps = 101;
        float band_low_hz = 50.0f;
        float band_high_hz = 2950.0f;
    };

    struct Snapshot {
        bool valid = false;  // the NOISE floor is measured (floor_noise_rms, normalized_noise_rms)
        // The SNR fields are meaningful only once a peer's signal power has been measured
        // (setSignalReferencePower): before that there is no signal to compare the noise with.
        bool snr_valid = false;
        float idle_in_band_snr_db = 0.0f;
        float latest_instant_idle_in_band_snr_db = 0.0f;
        float filtered_noise_rms = 0.0f;
        // Minimum-statistics floor: min in-band window RMS over the last
        // kFloorWindowCount idle windows. The floor is what the channel reads
        // when NOTHING is there — real-channel "idle" audio is nonstationary
        // (static bursts, undetected signal tails, own-TX echo on hardware),
        // so the last window's instantaneous RMS over-reads it by dBs (F223:
        // last-window 0.067-0.077 vs true 0.048 → every CONNECT classified as
        // noise). Consumers gating on "is this region noise-only" (#70
        // gap_is_noise) MUST use this, not filtered_noise_rms.
        float floor_noise_rms = 0.0f;
        float normalized_noise_rms = 0.0f;
        double fir_energy = 0.0;
        double equivalent_noise_bandwidth_hz = 0.0;
        size_t windows_observed = 0;
    };

    IdleNoiseSNREstimator();
    explicit IdleNoiseSNREstimator(const Config& config);

    void reset();
    void observeIdleAudio(const float* samples, size_t count);
    // The peer's received in-band signal power: its chirp's FIR-output power with the gap noise
    // removed, measured at chirp sync. The idle SNR is that over the current noise, i.e. the SNR
    // the peer's next transmission would read. It replaces the simulator's reference power
    // (0.3048^2), which assumed the receiver hears the peer at the peer's TRANSMIT level: on a
    // radio the level is the operator's AF / USB-audio / sound-card setting, and a -30 dB input
    // read +30 dB (BUG-RX-ABSOLUTE-LEVEL, INV-RX-LEVEL-001). Kept across reset() (the decoder
    // resets every turnaround); the next measured chirp replaces it.
    void setSignalReferencePower(double in_band_power);

    bool hasEstimate() const;
    float snrDb() const;
    Snapshot snapshot() const;

private:
    static double coefficientEnergy(const FIRFilter& filter);

    Config config_;
    FIRFilter filter_;
    double fir_energy_ = 1.0;
    double equivalent_noise_bandwidth_hz_ = 0.0;

    mutable std::mutex mutex_;
    double window_sum_sq_ = 0.0;
    size_t window_fill_ = 0;
    bool valid_ = false;
    float smoothed_snr_db_ = 0.0f;
    float latest_instant_snr_db_ = 0.0f;
    float filtered_noise_rms_ = 0.0f;
    float normalized_noise_rms_ = 0.0f;
    double signal_reference_power_ = 0.0;  // 0 = no peer measured yet
    std::array<float, kFloorWindowCount> window_rms_ring_{};
    size_t windows_observed_ = 0;
};

}  // namespace gui
}  // namespace ultra
