// The tone-burst ACK's symbol duration (gui/modem/tone_ack_staircase.hpp), shared by the GUI and the
// TNC: BUG-ACK-STAIRCASE-SLOW-ON-DROP (a drop lengthens the next ACK at once), BUG-TNC-ACK-FIXED-25MS
// (one rule for both frontends), BUG-MCDPSK-ACK-STAIRCASE-BYPASSED (an MC-DPSK reading may lengthen).
#include "gui/modem/tone_ack_staircase.hpp"

#include <cmath>
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
        // BUG-ACK-STAIRCASE-NON-SNR-SOURCE: ITU Good 3 dB OFDM readings, then a control frame
        // decoded off its chirp reports a correlation score of 14.8: the ACK stays long.
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(3.0f, SNRSource::OFDM_BROADBAND, 0.5f);
        const uint32_t before = s.choose().symbol_ms;
        s.noteFrame(14.8f, SNRSource::SYNC_QUALITY, 0.0f);
        CHECK(s.choose().symbol_ms == before && before > 25,
              "a chirp score must not shorten a fading 3 dB link's ACK to the baseline");
        CHECK(s.choose().source == SNRSource::OFDM_BROADBAND, "the last SNR reading stays the source");
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
        // Up to 100 ms the raw reading decides (it carries a fade's loss); the 200 ms floor only when
        // the channel the reading implies (+ kMcDpskReadingBelowChannelDb, 5.4) is under its edge.
        for (int i = 0; i < 5; ++i) s.noteFrame(-5.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 100,
              "an MC-DPSK reading of -5 dB: the 100 ms rung, as before the 200 ms floor");
        for (int i = 0; i < 5; ++i) s.noteFrame(-9.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 100, "a reading of -9 dB (channel -3.6): still 100 ms");
        for (int i = 0; i < 5; ++i) s.noteFrame(-11.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 200,
              "a reading of -11 dB (channel -5.6, under the 100 ms edge + 2): the 200 ms floor");
        for (int i = 0; i < 5; ++i) s.noteFrame(20.0f, SNRSource::MCDPSK_IN_BAND, 0.0f);
        CHECK(s.choose().symbol_ms == 25, "an MC-DPSK reading never shortens below the baseline");
    }
    {
        // BUG-ACK-STAIRCASE-FADING-SPIKE: AWGN 3 dB, one frame's fading index reads 0.35 (noise).
        // The column follows the median: the ACK stays on the AWGN column.
        ToneAckStaircase s;
        for (int i = 0; i < 4; ++i) s.noteFrame(1.5f, SNRSource::OFDM_BROADBAND, 0.02f);
        s.noteFrame(1.5f, SNRSource::OFDM_BROADBAND, 0.35f);
        const auto c = s.choose();
        CHECK(!c.fading_present && c.symbol_ms == 25,
              "one noisy fading reading on AWGN must not pick the 6.8 s fading floor");
        // A real fading channel reads fading on most frames: the column follows.
        for (int i = 0; i < 3; ++i) s.noteFrame(1.5f, SNRSource::OFDM_BROADBAND, 0.5f);
        CHECK(s.choose().fading_present, "three of five fading frames: the fading column");
    }
    {
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(20.0f, SNRSource::SYNC_QUALITY, 0.5f);
        CHECK(s.choose().symbol_ms == 25, "an untrusted source: the baseline");
    }
    {
        // BUG-ACK-STAIRCASE-UNMEASURED-FADING (TNC ITU Good 8 dB seed 506): an MC-DPSK spell, whose
        // decoder measures no fading index (NaN), then the first OFDM group after the up-crossing
        // reads 1.1 dB with a fading index of 0.385. Counted as 0, the four MC-DPSK frames made the
        // median "AWGN" and the ACK went out at 25 ms, under its 2 dB fading edge; it was lost.
        ToneAckStaircase s;
        for (int i = 0; i < 4; ++i) s.noteFrame(-0.5f, SNRSource::MCDPSK_IN_BAND, NAN);
        s.noteFrame(1.1f, SNRSource::OFDM_BROADBAND, 0.385f);
        const auto c = s.choose();
        CHECK(c.fading_present, "unmeasured MC-DPSK frames do not outvote a measured fading index");
        CHECK(c.symbol_ms == 200 || c.symbol_ms == 100,
              "1.1 dB (or lower, the median) on fading: a long ACK, not 25 ms");
    }
    {
        // No fading measurement at all on an OFDM reading: the AWGN column, as before.
        ToneAckStaircase s;
        for (int i = 0; i < 5; ++i) s.noteFrame(3.0f, SNRSource::OFDM_BROADBAND, NAN);
        const auto c = s.choose();
        CHECK(!c.fading_present && c.symbol_ms == 12, "no fading measurement: the AWGN column");
    }
    if (failures) {
        std::cout << "ToneAckStaircase: " << failures << " failure(s)\n";
        return 1;
    }
    std::cout << "ToneAckStaircase: all checks passed\n";
    return 0;
}
