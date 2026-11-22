// The tone-burst ACK's symbol duration (gui/modem/tone_ack_staircase.hpp), shared by the GUI and the
// TNC: BUG-ACK-STAIRCASE-SLOW-ON-DROP (a drop lengthens the next ACK at once), BUG-TNC-ACK-FIXED-25MS
// (one rule for both frontends), BUG-MCDPSK-ACK-STAIRCASE-BYPASSED (an MC-DPSK reading may lengthen).
#include "gui/modem/tone_ack_staircase.hpp"

#include <iostream>

using ultra::SNRSource;
using ultra::gui::ToneAckStaircase;

namespace {
int failures = 0;
#define CHECK(cond, msg)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            ++failures;                                   \
            std::cout << "FAIL: " << msg << "\n";         \
        }                                                 \
    } while (0)
}  // namespace

int main() {
    {
        ToneAckStaircase s;
        CHECK(s.choose().symbol_ms == 25, "no reading yet: the 25 ms baseline");
    }
    {
        // ITU Good at 20 dB, then the channel drops to 5 dB.
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(20.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        CHECK(s.choose().symbol_ms == 12, "a clean fading channel at 20 dB: 12 ms");
        s.noteFrame(5.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        const auto c = s.choose();
        CHECK(c.symbol_ms == 50 && c.snr_used == 5.0f,
              "the first reading after the drop lengthens the ACK at once (5 dB fading: 50 ms), "
              "though the median still reads 20 dB");
        s.noteFrame(20.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        CHECK(s.choose().symbol_ms == 12, "a lone trough lengthened one ACK only");
        s.noteFrame(1.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        CHECK(s.choose().symbol_ms == 200,
              "1 dB on fading: the 200 ms floor (100 ms needs 0 + 2 dB, re-measured 2026-11-22)");
    }
    {
        // A single high reading after a run of low ones does not shorten the ACK (the median holds).
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(3.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        s.noteFrame(20.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        CHECK(s.choose().symbol_ms == 100, "one crest after a low run: still the low run's ACK");
    }
    {
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(3.0f, SNRSource::OFDM_BROADBAND, 0.05f);
        CHECK(s.choose().symbol_ms == 12, "AWGN at 3 dB: the AWGN column, 12 ms");
    }
    {
        // MC-DPSK readings: no fading measurement, may only lengthen past the baseline.
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(0.6f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 25,
              "an MC-DPSK reading of 0.6 dB (a 5 dB fading channel): the baseline, as before");
        // MC-DPSK readings sit kMcDpskReadingBelowChannelDb (5.4) under the channel the ACK crosses.
        for (int i = 0; i < 5; ++i) s.noteFrame(-5.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 25,
              "an MC-DPSK reading of -5 dB is a ~0 dB channel: the baseline");
        for (int i = 0; i < 5; ++i) s.noteFrame(-8.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 50, "a reading of -8 dB (channel -2.6): 50 ms");
        for (int i = 0; i < 5; ++i) s.noteFrame(-11.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 200,
              "a reading of -11 dB (channel -5.6, under the 100 ms edge + 2): the 200 ms floor");
        for (int i = 0; i < 5; ++i) s.noteFrame(20.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 25, "an MC-DPSK reading never shortens below the baseline");
    }
    {
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(20.0f, SNRSource::SYNC_QUALITY, 0.5f);
        CHECK(s.choose().symbol_ms == 25, "an untrusted source: the baseline");
    }
    if (failures) {
        std::cout << "ToneAckStaircase: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "ToneAckStaircase: all checks passed\n";
    return 0;
}
