#pragma once
// The tone-burst ACK's symbol duration (§15.5 staircase) from the receiver's recent frame SNRs - one
// rule for every frontend: the GUI and the headless TNC (BUG-TNC-ACK-FIXED-25MS: the TNC sent every
// ACK at the 25 ms baseline whatever the channel).
//
// Rule.
// - The reading: the median of the last kRing frame SNRs (one erasure-slot or trough snapshot cannot
//   move it - BUG-STAIRCASE-SNAPSHOT-INPUT, ULTRA_ACK_SNR_MEDIAN), or the latest reading when that is
//   lower: fast to lengthen, slow to shorten (BUG-ACK-STAIRCASE-SLOW-ON-DROP: after a 20 -> 5 dB drop
//   the median held the pre-drop SNR for three frames and 12 ms ACKs were lost at 0.3-6 dB; a lost
//   ACK costs the sender's whole round, 8-17 s, a needlessly long one at most ~3 s of air).
// - The column: fading when the frames measured fading (>= kFadingAwgnMax, ULTRA_ACK_FADE_EDGE).
// - The source: an OFDM or idle in-band reading sets the duration; an MC-DPSK reading may only
//   LENGTHEN it past the baseline (BUG-MCDPSK-ACK-STAIRCASE-BYPASSED: an MC-DPSK session never left
//   25 ms). The MC-DPSK reading is the data-aided estimate, a lower bound that already carries the
//   fade loss (training 4.6 / data-aided 1.0 dB at a 5 dB ITU Good channel), so it is read in the
//   AWGN column: at Good 5 dB every 25 ms MC-DPSK ACK was heard (16/16) and the fading column's
//   100 ms (3.4 s) would have cost ~2.5 s per round for nothing; below ~-2 dB it lengthens.
//   Any other source: the baseline.
// Lock-free: noteFrame runs on the modem RX path, choose on the protocol's ACK callback (which holds
// the protocol mutex, so neither may call into the protocol).
#include "protocol/connection_policy.hpp"  // kMcDpskReadingBelowChannelDb
#include "protocol/waveform_selection.hpp"  // kFadingAwgnMax
#include "ultra/types.hpp"                  // SNRSource
#include "waveform/tone_burst_ack/tone_burst_constants.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>

namespace ultra {
namespace gui {

class ToneAckStaircase {
public:
    static constexpr size_t kRing = 5;

    struct Choice {
        uint32_t symbol_ms = waveform::tone_burst_ack::kBaselineSymbolMs;
        float snr_used = 0.0f;
        float snr_current = 0.0f;
        SNRSource source = SNRSource::NONE;
        float fading_index = 0.0f;
        bool fading_present = false;
    };

    // A decoded frame's in-band SNR, the estimator it came from and the frame's fading index.
    void noteFrame(float snr_db, SNRSource source, float fading_index) {
        current_.store(snr_db, std::memory_order_relaxed);
        source_.store(source, std::memory_order_relaxed);
        fading_.store(fading_index, std::memory_order_relaxed);
        ring_[idx_.fetch_add(1, std::memory_order_relaxed) % kRing].store(snr_db,
                                                                           std::memory_order_relaxed);
    }

    // The duration for an ACK sent now.
    Choice choose() const {
        Choice c;
        c.source = source_.load(std::memory_order_relaxed);
        c.snr_current = current_.load(std::memory_order_relaxed);
        c.fading_index = fading_.load(std::memory_order_relaxed);
        c.snr_used = c.snr_current;
        if (medianEnabled()) {
            std::array<float, kRing> v;
            for (size_t i = 0; i < kRing; ++i) v[i] = ring_[i].load(std::memory_order_relaxed);
            std::sort(v.begin(), v.end());
            c.snr_used = std::min(v[kRing / 2], c.snr_current);
        }
        const bool ofdm_or_idle =
            c.source == SNRSource::IDLE_IN_BAND || c.source == SNRSource::OFDM_BROADBAND;
        if (ofdm_or_idle) {
            c.fading_present =
                fadeEdgeEnabled() && c.fading_index >= protocol::kFadingAwgnMax;
            c.symbol_ms = waveform::tone_burst_ack::symbolMsForSNR(c.snr_used, c.fading_present);
        } else if (c.source == SNRSource::MCDPSK_IN_BAND) {
            c.fading_present = false;  // the fade-effective reading, AWGN column (see above)
            // Up to 100 ms the raw reading decides, as before 2026-11-22: on fading it carries the
            // fade loss, the margin the AWGN column needs. The 200 ms floor (2026-11-22) is chosen on
            // the channel scale its edge was measured on (the reading sits
            // kMcDpskReadingBelowChannelDb under it): read raw it keyed 6.8 s ACKs at AWGN -2 dB
            // (845 vs 731 s for 4 KB, 0 retx); mapping every rung shortened fading ACKs to 25 ms
            // (ITU Good 0 / 3 dB: 32 / 10 retransmissions).
            namespace tba = waveform::tone_burst_ack;
            const float on_channel =
                c.snr_used + protocol::connection_policy::kMcDpskReadingBelowChannelDb;
            uint32_t ms = tba::symbolMsForSNR(c.snr_used, false);
            if (ms == tba::kSymbolMsWeakSNR && tba::symbolMsForSNR(on_channel, false) != tba::kSymbolMsWeakSNR) {
                ms = tba::kSymbolMsMargSNR;
            }
            c.symbol_ms = std::max(tba::kBaselineSymbolMs, ms);
        }
        return c;
    }

private:
    static bool medianEnabled() {
        static const bool on = [] {
            const char* e = std::getenv("ULTRA_ACK_SNR_MEDIAN");
            return !(e && e[0] == '0');  // default ON (2026-07-05)
        }();
        return on;
    }
    static bool fadeEdgeEnabled() {
        static const bool on = [] {
            const char* e = std::getenv("ULTRA_ACK_FADE_EDGE");
            return !(e && *e == '0');
        }();
        return on;
    }

    std::atomic<float> current_{12.0f};
    std::atomic<SNRSource> source_{SNRSource::NONE};
    std::atomic<float> fading_{0.0f};
    std::array<std::atomic<float>, kRing> ring_{{12.0f, 12.0f, 12.0f, 12.0f, 12.0f}};
    std::atomic<uint32_t> idx_{0};
};

}  // namespace gui
}  // namespace ultra
