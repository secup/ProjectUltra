#include "env_compat.hpp"
#include "protocol/connection_policy.hpp"
#include "protocol/selective_repeat_arq_policy.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace ultra;
using namespace ultra::protocol;
using namespace ultra::protocol::connection_policy;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
            return; \
        } \
    } while (0)

void test_fading_labels_and_capabilities() {
    CHECK(std::string(fadingLabel(0.00f)) == "AWGN", "AWGN fading label");
    CHECK(std::string(fadingLabel(0.30f)) == "Good", "Good fading label");
    CHECK(std::string(fadingLabel(0.80f)) == "Moderate", "Moderate fading label");
    CHECK(std::string(fadingLabel(1.20f)) == "Poor", "Poor fading label");
    CHECK(classifyChannel(0.14f) == ChannelClassification::AWGN,
          "channel classifier should preserve AWGN threshold");
    CHECK(classifyChannel(0.15f) == ChannelClassification::GOOD,
          "channel classifier should enter Good at 0.15 fading index");
    // BUG-MPG20-OVER-DEMOTE-R14 (2026-07-14): the Good/Moderate boundary moved to the
    // ML midpoint of the cluster centers (kFadingGoodMax = 0.76). The ledger's Good
    // upper tail (to 0.74 on a true-Good MPG@20 channel) now classes Good instead of
    // false-Moderate -> QPSK R1/4.
    CHECK(classifyChannel(0.74f) == ChannelClassification::GOOD,
          "0.74 (ledger true-Good max) classes Good under the 0.76 boundary");
    CHECK(classifyChannel(0.78f) == ChannelClassification::MODERATE,
          "channel classifier enters Moderate above the 0.76 Good/Moderate boundary");
    CHECK(classifyChannel(1.10f) == ChannelClassification::POOR,
          "channel classifier should enter Poor at 1.10 fading index");

    CHECK(modeToCapabilityBit(WaveformMode::OFDM_CHIRP) == ModeCapabilities::OFDM_CHIRP,
          "OFDM_CHIRP capability bit");
    CHECK(modeToCapabilityBit(WaveformMode::OFDM_NARROW) == ModeCapabilities::OFDM_NARROW,
          "OFDM_NARROW capability bit");
    CHECK(modeToCapabilityBit(WaveformMode::MC_DPSK) == ModeCapabilities::MC_DPSK,
          "MC_DPSK capability bit");
    CHECK(modeToCapabilityBit(WaveformMode::AUTO) == 0, "AUTO has no capability bit");
}

void test_ladder_rung_selection() {
    const auto robust_low = ladderRungForId(LadderRungId::ROBUST_LOW);
    CHECK(robust_low.waveform == WaveformMode::MC_DPSK, "Robust-Low waveform");
    CHECK(robust_low.modulation == Modulation::DBPSK, "Robust-Low modulation");
    // #72: MC-DPSK standardized on sps=1024 (control==data baud → handshake always
    // mutually decodable; the gears now vary only constellation/rate, not baud).
    CHECK(robust_low.samples_per_symbol == 1024, "Robust-Low SPS");
    CHECK(robust_low.cw_count == 3, "Robust-Low CW count");

    const auto robust_mid = ladderRungForId(LadderRungId::ROBUST_MID);
    CHECK(robust_mid.waveform == WaveformMode::MC_DPSK, "Robust-Mid waveform");
    CHECK(robust_mid.modulation == Modulation::DBPSK, "Robust-Mid modulation");
    CHECK(robust_mid.samples_per_symbol == 1024, "Robust-Mid SPS");
    CHECK(robust_mid.cw_count == 3, "Robust-Mid CW count");

    const auto robust = ladderRungForId(LadderRungId::ROBUST);
    CHECK(robust.waveform == WaveformMode::MC_DPSK, "Robust waveform");
    CHECK(robust.modulation == Modulation::DQPSK, "Robust modulation");
    CHECK(robust.samples_per_symbol == 1024, "Robust SPS");
    CHECK(robust.cw_count == v2::kDefaultFixedFrameCodewords, "Robust CW count");

    const auto standard = ladderRungForId(LadderRungId::STANDARD);
    CHECK(standard.waveform == WaveformMode::MC_DPSK, "Standard waveform");
    CHECK(standard.modulation == Modulation::DQPSK, "Standard modulation");
    CHECK(standard.samples_per_symbol == 1024, "Standard SPS");  // #72: was 512

    CHECK(ladderRungForId(LadderRungId::OFDM_CHIRP).waveform == WaveformMode::OFDM_CHIRP,
          "OFDM_CHIRP rung waveform");
    CHECK(std::string(ladderRungIdToString(LadderRungId::ROBUST_LOW)) == "Robust-Low",
          "Robust-Low rung string");

    CHECK(selectLadderRung(6.9f, ChannelClassification::MODERATE).id ==
              LadderRungId::ROBUST_LOW,
          "Moderate below in-band 7 dB selects Robust-Low");
    CHECK(selectLadderRung(7.0f, ChannelClassification::MODERATE).id ==
              LadderRungId::ROBUST_MID,
          "Moderate in-band 7 dB boundary selects Robust-Mid (DBPSK)");
    // #71: DQPSK reachable on BENIGN channels only (measured). FADING Moderate/Poor
    // keep their old DQPSK floors (Moderate 15 > ofdm 14 -> DBPSK only; fading DQPSK
    // floor unmeasured — differential Doppler penalty > +2.5 dB).
    CHECK(selectLadderRung(13.9f, ChannelClassification::MODERATE).id ==
              LadderRungId::ROBUST_MID,
          "Moderate below in-band 14 dB stays Robust-Mid (DBPSK, fading)");
    CHECK(selectLadderRung(14.0f, ChannelClassification::MODERATE).id ==
              LadderRungId::OFDM_CHIRP,
          "Moderate in-band 14 dB boundary selects OFDM_CHIRP");

    // AWGN entry floor lowered 10->8 (R1/4 clean @ AWGN 8, measured 2026-06-02).
    // #71: AWGN robust_mid_floor=5 -> dqpsk_floor=min(5+1.0, 8-0.5)=6.0, ofdm_floor=8
    // (lowered from +2.5=7.5: DQPSK window spiral fixed + rig-validated DQPSK>=DBPSK).
    CHECK(selectLadderRung(5.9f, ChannelClassification::AWGN).id ==
              LadderRungId::ROBUST_MID,
          "AWGN below DQPSK floor stays Robust-Mid (DBPSK)");
    CHECK(selectLadderRung(6.0f, ChannelClassification::AWGN).id ==
              LadderRungId::ROBUST,
          "AWGN at DQPSK floor selects Robust (DQPSK)");
    CHECK(selectLadderRung(8.0f, ChannelClassification::AWGN).id ==
              LadderRungId::OFDM_CHIRP,
          "AWGN in-band 8 dB boundary selects OFDM_CHIRP");
    // Good entry floor lowered 12->10 (R1/2 reliable @ Good 10, measured).
    // #71: Good robust_mid_floor=6 -> dqpsk_floor=min(6+1.0, 9.5)=7.0, ofdm_floor=10
    // (lowered from +2.5=8.5: DQPSK window spiral fixed + rig-validated DQPSK>=DBPSK @MPG9).
    CHECK(selectLadderRung(6.9f, ChannelClassification::GOOD).id ==
              LadderRungId::ROBUST_MID,
          "Good below DQPSK floor stays Robust-Mid (DBPSK)");
    CHECK(selectLadderRung(7.0f, ChannelClassification::GOOD).id ==
              LadderRungId::ROBUST,
          "Good at DQPSK floor selects Robust (DQPSK) — rig-validated clean @MPG9");
    CHECK(selectLadderRung(7.9f, ChannelClassification::GOOD).id ==
              LadderRungId::ROBUST,
          "Good fading below OFDM floor (8, 2026-07-07 sweep) selects Robust (DQPSK)");
    CHECK(selectLadderRung(8.0f, ChannelClassification::GOOD).id ==
              LadderRungId::OFDM_CHIRP,
          "Good fading in-band 8 dB boundary selects OFDM_CHIRP");
    CHECK(selectLadderRung(16.9f, ChannelClassification::POOR).id ==
              LadderRungId::ROBUST_MID,
          "Poor fading keeps extra margin before Robust");
    CHECK(selectLadderRung(17.0f, ChannelClassification::POOR).id ==
              LadderRungId::ROBUST,
          "Poor fading keeps Robust below the OFDM_CHIRP floor");
    // Coherent-only OFDM (thread A, 2026-05-31): Poor fading NEVER selects OFDM — it
    // routes to MC-DPSK at every SNR (kOFDMEntryFloorPoorDb is unreachable). Poor is
    // fast-fading differential territory; OFDM is coherent-only on AWGN/Good/Moderate.
    CHECK(selectLadderRung(18.0f, ChannelClassification::POOR).id ==
              LadderRungId::ROBUST,
          "Poor fading at in-band 18 dB stays MC-DPSK (OFDM retired from Poor)");
    CHECK(selectLadderRung(30.0f, ChannelClassification::POOR).id ==
              LadderRungId::ROBUST,
          "Poor fading even at high in-band 30 dB stays MC-DPSK (never OFDM)");

    CHECK(selectLadderRung(10.0f, 0.80f).id == LadderRungId::ROBUST_MID,
          "fading-index overload selects Moderate Robust-Mid at in-band 10 dB");
}

void test_wide_ofdm_timing_and_timeout() {
    auto dqpsk = wideOFDMFrameTiming(Modulation::DQPSK, CodeRate::R1_2);
    CHECK(dqpsk.data_symbols == 27, "DQPSK R1/2 wide OFDM data symbols");
    CHECK(dqpsk.ack_symbols == 9, "DQPSK R1/2 wide OFDM ACK symbols");
    CHECK(dqpsk.data_ms == 648, "DQPSK R1/2 wide OFDM data frame ms");
    CHECK(dqpsk.ack_ms == 216, "DQPSK R1/2 wide OFDM ACK frame ms");

    auto dqpsk_6cw = wideOFDMFrameTiming(Modulation::DQPSK, CodeRate::R1_2, 6);
    auto dqpsk_8cw = wideOFDMFrameTiming(Modulation::DQPSK, CodeRate::R1_2, 8);
    CHECK(dqpsk_6cw.data_symbols == 39, "DQPSK R1/2 6-CW wide OFDM data symbols");
    CHECK(dqpsk_8cw.data_symbols == 51, "DQPSK R1/2 8-CW wide OFDM data symbols");
    CHECK(dqpsk_6cw.data_ms > dqpsk.data_ms && dqpsk_8cw.data_ms > dqpsk_6cw.data_ms,
          "wide OFDM data duration should scale with fixed-frame CW count");

    CHECK(kWideOFDMFullAnchorExtraMs == 1200,
          "wide OFDM full chirp anchor should add 1.2 s to multi-frame bursts");
    CHECK(wideOFDMBurstAirtimeMs(Modulation::DQPSK, CodeRate::R1_2, 1) ==
              dqpsk.data_ms + kWideOFDMFullAnchorExtraMs,
          "single wide OFDM turn should include its full chirp timing anchor");

    const uint32_t w8_burst_ms = wideOFDMBurstAirtimeMs(
        Modulation::DQPSK, CodeRate::R1_2, 8);
    CHECK(w8_burst_ms == 8 * dqpsk.data_ms + kWideOFDMFullAnchorExtraMs,
          "wide OFDM multi-frame burst airtime should include the first-frame chirp anchor");

    const uint32_t w8_sack_delay = wideOFDMSackDelayMs(
        Modulation::DQPSK, CodeRate::R1_2, 8);
    CHECK(w8_sack_delay == w8_burst_ms + kCarrierSenseSackCoalesceMs,
          "wide OFDM SACK delay should hold ACKs through a physical sender window");

    const auto qpsk_r23_8cw = wideOFDMFrameTiming(Modulation::QPSK, CodeRate::R2_3, 8);
    const uint32_t qpsk_r23_w8_burst_ms = wideOFDMBurstAirtimeMs(
        Modulation::QPSK, CodeRate::R2_3, 8, 8);
    const uint32_t qpsk_r23_w8_reanchor_burst_ms = wideOFDMBurstAirtimeMs(
        Modulation::QPSK, CodeRate::R2_3, 8, 8, kWideOFDMShortReanchorDefaultMs);
    CHECK(qpsk_r23_w8_burst_ms == 8 * qpsk_r23_8cw.data_ms + kWideOFDMFullAnchorExtraMs,
          "coherent QPSK R2/3 8-CW burst airtime should include the first-frame chirp anchor");
    CHECK(qpsk_r23_w8_reanchor_burst_ms ==
              qpsk_r23_w8_burst_ms + 7 * kWideOFDMShortReanchorDefaultMs,
          "adaptive short reanchors should extend every continuation frame in the physical burst");
    CHECK(wideOFDMSackDelayMs(Modulation::QPSK, CodeRate::R2_3, 8, 8,
                              kWideOFDMShortReanchorDefaultMs) ==
              qpsk_r23_w8_reanchor_burst_ms + kCarrierSenseSackCoalesceMs,
          "coherent fading SACK delay should hold through adaptive reanchor burst airtime");

    const auto qam16_r14 = wideOFDMFrameTiming(Modulation::QAM16, CodeRate::R1_4);
    CHECK(wideOFDMSlidingSackDelayMs(Modulation::QAM16, CodeRate::R1_4) ==
              qam16_r14.data_ms + qam16_r14.ack_ms + kCarrierSenseSackCoalesceMs,
          "wide OFDM sliding SACK delay should derive from selected data and ACK airtime");
    CHECK(wideOFDMSlidingSackDelayMs(Modulation::QAM16, CodeRate::R1_4) <
              wideOFDMSackDelayMs(Modulation::QAM16, CodeRate::R1_4, 8),
          "wide OFDM sliding SACK delay should be a burst-tail quiet interval, not a full-window hold");

    const uint32_t qam16_ack_repeat_tail =
        selective_repeat_arq_policy::ackRepeatTailGuardMs(
            qam16_r14.ack_ms,
            wideOFDMSlidingSackDelayMs(Modulation::QAM16, CodeRate::R1_4),
            80,
            3,
            true);
    CHECK(qam16_ack_repeat_tail >=
              wideOFDMSlidingSackDelayMs(Modulation::QAM16, CodeRate::R1_4) +
                  240 + selective_repeat_arq_policy::kAckRepeatMaxJitterMs +
                  qam16_r14.ack_ms,
          "wide OFDM ACK-diversity guard should cover delayed clean-ACK repeat tail");
    CHECK(qam16_ack_repeat_tail > 250,
          "wide OFDM QAM16 ACK-diversity guard must exceed the legacy fixed 250 ms holdoff");

    CHECK(computeWideOFDMAckTimeoutMs(Modulation::DQPSK, CodeRate::R1_2, 4, 120, 2) == 9446,
          "wide OFDM window=4 timeout should cover physical burst, ACK copies, and SACK holdoff");
    CHECK(computeWideOFDMAckTimeoutMs(Modulation::DQPSK, CodeRate::R1_2, 8, 120, 1) == 14414,
          "wide OFDM window=8 timeout should derive from burst airtime, not the configured short SACK");
    CHECK(computeWideOFDMAckTimeoutMs(Modulation::DQPSK, CodeRate::R1_2, 8,
                                      w8_sack_delay, 1) == 14414,
          "wide OFDM window=8 timeout should cover physical SACK holdoff");
    CHECK(computeWideOFDMAckTimeoutMs(Modulation::DQPSK, CodeRate::R1_2, 8, 120, 1, 6) == 19022,
          "wide OFDM 6-CW ACK timeout should cover the longer burst");
    CHECK(computeWideOFDMAckTimeoutMs(Modulation::DQPSK, CodeRate::R1_2, 8, 120, 1, 8) == 23630,
          "wide OFDM 8-CW ACK timeout should cover the longer burst");

    const uint32_t timeout_4cw = computeWideOFDMAckTimeoutMs(
        Modulation::DQPSK, CodeRate::R1_2, kHighThroughputOFDMWindowFrames,
        kCarrierSenseSackCoalesceMs, kCarrierSenseAckRepeatCount);
    const uint32_t min_4cw_ack_path =
        wideOFDMBurstAirtimeMs(
            Modulation::DQPSK, CodeRate::R1_2, kHighThroughputOFDMWindowFrames) +
        kCarrierSenseSackCoalesceMs + dqpsk.ack_ms;
    CHECK(timeout_4cw >= min_4cw_ack_path,
          "wide OFDM window=16 4-CW timeout should cover burst plus carrier-sensed ACK path");

    const uint32_t timeout_6cw = computeWideOFDMAckTimeoutMs(
        Modulation::DQPSK, CodeRate::R1_2, kHighThroughputOFDMWindowFrames,
        kCarrierSenseSackCoalesceMs, kCarrierSenseAckRepeatCount, 6);
    const uint32_t min_6cw_ack_path =
        wideOFDMBurstAirtimeMs(
            Modulation::DQPSK, CodeRate::R1_2, kHighThroughputOFDMWindowFrames, 6) +
        kCarrierSenseSackCoalesceMs + dqpsk_6cw.ack_ms;
    CHECK(timeout_6cw >= min_6cw_ack_path,
          "wide OFDM window=16 6-CW timeout should cover burst plus carrier-sensed ACK path");

    const uint32_t timeout_8cw = computeWideOFDMAckTimeoutMs(
        Modulation::DQPSK, CodeRate::R1_2, kHighThroughputOFDMWindowFrames,
        kCarrierSenseSackCoalesceMs, kCarrierSenseAckRepeatCount, 8);
    const uint32_t min_8cw_ack_path =
        wideOFDMBurstAirtimeMs(
            Modulation::DQPSK, CodeRate::R1_2, kHighThroughputOFDMWindowFrames, 8) +
        kCarrierSenseSackCoalesceMs + dqpsk_8cw.ack_ms;
    CHECK(timeout_8cw >= min_8cw_ack_path,
          "wide OFDM window=16 8-CW timeout should cover burst plus carrier-sensed ACK path");

    auto d8psk = wideOFDMFrameTiming(Modulation::D8PSK, CodeRate::R1_2);
    CHECK(d8psk.data_symbols == 19, "D8PSK R1/2 wide OFDM data symbols");
    CHECK(d8psk.ack_symbols == 7, "D8PSK R1/2 wide OFDM ACK symbols");

    auto qam8 = wideOFDMFrameTiming(Modulation::QAM8, CodeRate::R1_2, 8);
    // FIXED-GRID BAND (2026-07-06): R1/2 joined the sp8 grid -> 51 data carriers
    // (was sp5/47). 8 CW x 648 bits / (3 b/sym x 51 carriers) = 34 data + 2 training = 36.
    CHECK(qam8.data_symbols == 36, "coherent 8PSK R1/2 8-CW wide OFDM data symbols");
    CHECK(qam8.ack_symbols == 7, "coherent 8PSK R1/2 wide OFDM ACK symbols");
}

void test_narrow_ofdm_timing_and_timeout() {
    auto narrow = narrowOFDMFrameTiming(Modulation::DQPSK);
    CHECK(narrow.data_symbols == 74, "narrow OFDM DQPSK data symbols");
    CHECK(narrow.ack_symbols == 20, "narrow OFDM DQPSK ACK symbols");
    CHECK(narrow.data_ms == 3453, "narrow OFDM DQPSK data frame ms");
    CHECK(narrow.ack_ms == 933, "narrow OFDM DQPSK ACK frame ms");
    auto narrow_8cw = narrowOFDMFrameTiming(Modulation::DQPSK, 8);
    CHECK(narrow_8cw.data_ms > narrow.data_ms,
          "narrow OFDM data duration should scale with fixed-frame CW count");
    CHECK(computeNarrowOFDMAckTimeoutMs(Modulation::DQPSK) == 8665,
          "narrow OFDM DQPSK ACK timeout (window=1, default) — +1500ms receiver SACK hold (2026-06-19)");
    CHECK(computeNarrowOFDMAckTimeoutMs(Modulation::DQPSK, 8) >
              computeNarrowOFDMAckTimeoutMs(Modulation::DQPSK),
          "narrow OFDM ACK timeout should scale with fixed-frame CW count");

    // Selective-repeat window=2/3 added 2026-05-03 per Codex audit. The
    // timeout has to cover the full multi-frame TX burst plus ACK
    // turnaround — without scaling, the ARQ would fire while later
    // frames are still on-air and trigger a phantom timeout retry.
    const uint32_t w1 = computeNarrowOFDMAckTimeoutMs(Modulation::DQPSK, 4, 1);
    const uint32_t w2 = computeNarrowOFDMAckTimeoutMs(Modulation::DQPSK, 4, 2);
    const uint32_t w3 = computeNarrowOFDMAckTimeoutMs(Modulation::DQPSK, 4, 3);
    CHECK(w1 == 8665, "window=1 timeout matches default behavior (+1500ms SACK hold)");
    CHECK(w2 > w1, "window=2 timeout scales above window=1");
    CHECK(w3 > w2, "window=3 timeout scales above window=2");
    // For 4-CW DQPSK narrow, data_ms=3453, ack_ms=933, plus the +1500 ms receiver SACK hold:
    //   window=2: tx_burst_ms = 2*3453 = 6906; timeout = 6906 + 2*933
    //             + 120 + max(700, 1726) + 1500 = 12118 ms
    //   window=3: tx_burst_ms = 3*3453 = 10359; timeout = 10359 + 2*933
    //             + 120 + max(700, 1726) + 1500 = 15571 ms (current narrow default)
    CHECK(w2 >= 11500 && w2 <= 12500, "window=2 timeout ~12.1 s");
    CHECK(w3 >= 15000 && w3 <= 16000, "window=3 timeout ~15.6 s (current narrow default)");
}

void test_mc_dpsk_window_timing() {
    auto robust_low = mcDpskFrameTiming(Modulation::DBPSK, 8, 2048, 3);
    CHECK(robust_low.data_ms == 10752, "Robust-Low MC-DPSK 3-CW data timing");
    CHECK(robust_low.data_only_ms == 10368, "Robust-Low MC-DPSK data-only timing");
    CHECK(mcDpskBurstAirtimeMs(robust_low, 1) == 11952,
          "Robust-Low MC-DPSK physical burst timing");
    CHECK(mcDpskWindowSizeForTiming(robust_low) == 1,
          "Robust-Low MC-DPSK should keep window=1");

    auto robust_mid = mcDpskFrameTiming(Modulation::DBPSK, 8, 1024, 3);
    CHECK(robust_mid.data_ms == 5376, "Robust-Mid MC-DPSK 3-CW data timing");
    CHECK(robust_mid.data_only_ms == 5184, "Robust-Mid MC-DPSK data-only timing");
    CHECK(mcDpskBurstAirtimeMs(robust_mid, 3) == 16944,
          "Robust-Mid MC-DPSK window=3 physical burst timing");
    CHECK(mcDpskWindowSizeForTiming(robust_mid) == 3,
          "Robust-Mid MC-DPSK should use window=3");
    // BUG-MCDPSK-FILE-COMPLETION / ACK-COLLISION: the ACK RTO must budget the SAME receiver
    // tone-burst SACK hold the receiver applies (setToneBurstPartialSackDelayMs) AND the
    // receiver serial-decode round-trip, else the sender blind-resends before the measured
    // ~37.9 s rig RTT -> doubled airtime -> FINAL file chunk never reached -> never finalizes.
    const uint32_t rm_hold = std::max<uint32_t>(1500u, robust_mid.data_ms + 1000u);  // 6376
    const uint32_t rm_timeout =
        computeMCDPSKAckTimeoutMs(robust_mid, 3, rm_hold, kCarrierSenseAckRepeatCount);
    // Covers the physical half-duplex RTT: sender TX + receiver serial decode + hold + ACK.
    CHECK(rm_timeout >= 2u * 3u * robust_mid.data_ms + robust_mid.ack_ms + rm_hold,
          "Robust-Mid ACK RTO must budget tx_burst + rx_decode + receiver SACK hold + ACK");
    // Must exceed the measured ~37.9 s rig RTT so the sender does not self-collide.
    CHECK(rm_timeout >= 40000u,
          "Robust-Mid ACK RTO must exceed the measured ~37.9 s half-duplex RTT");
    // REGRESSION: budgeting the real ~6.4 s hold yields a STRICTLY larger RTO than the old
    // 30 ms carrier-sense coalesce that under-budgeted the round-trip (the bug's origin).
    CHECK(rm_timeout > computeMCDPSKAckTimeoutMs(robust_mid, 3, kCarrierSenseSackCoalesceMs,
                                                 kCarrierSenseAckRepeatCount),
          "Real receiver hold must widen the RTO vs the old 30 ms coalesce (regression guard)");

    auto robust = mcDpskFrameTiming(Modulation::DQPSK, 8, 1024, 4);
    CHECK(robust.data_ms == 3691, "Robust MC-DPSK 4-CW data timing");
    CHECK(robust.data_only_ms == 3499, "Robust MC-DPSK data-only timing");
    CHECK(mcDpskBurstAirtimeMs(robust, 5) == 18887,
          "Robust MC-DPSK window=5 physical burst timing (airtime formula)");
    // #71: DQPSK is capped at the round-trip-safe window=3 (was 5). Window=5's ~18.9 s TX
    // burst made the ACK round-trip intermittently exceed the RTO -> blind-resend spiral
    // (rig @ MPG@9: 1/3 delivered); window=3 delivered 3/3 CRC-clean at ~2x DBPSK goodput.
    CHECK(mcDpskWindowSizeForTiming(robust) == 3,
          "Robust (DQPSK) MC-DPSK must cap at the round-trip-safe window=3, not 5");
    CHECK(kCarrierSenseAckRepeatCount == 1,
          "Robust MC-DPSK must not repeat full-preamble ACKs into the next DATA turn");

    auto standard = mcDpskFrameTiming(Modulation::DQPSK, 8, 512, 4);
    CHECK(standard.data_ms == 1845, "Standard MC-DPSK 4-CW data timing");
    CHECK(standard.data_only_ms == 1749, "Standard MC-DPSK data-only timing");
    CHECK(mcDpskBurstAirtimeMs(standard, 5) == 10041,
          "Standard MC-DPSK window=5 physical burst timing (airtime formula)");
    CHECK(mcDpskWindowSizeForTiming(standard) == 3,
          "Standard (DQPSK) MC-DPSK must cap at the round-trip-safe window=3, not 5 (#71)");
    const uint32_t std_hold = std::max<uint32_t>(1500u, standard.data_ms + 1000u);
    const uint32_t std_timeout =
        computeMCDPSKAckTimeoutMs(standard, 3, std_hold, kCarrierSenseAckRepeatCount);
    CHECK(std_timeout >= 18000u,
          "Standard MC-DPSK timeout should retain the conservative floor");
    CHECK(std_timeout >= 2u * 3u * standard.data_ms + standard.ack_ms + std_hold,
          "Standard MC-DPSK ACK RTO must budget tx_burst + rx_decode + hold + ACK");
}

void test_ofdm_profile_selection() {
    CHECK(isNearAwgnOFDM(0.00f, 25.0f), "near-AWGN threshold should include in-band SNR25");
    CHECK(isNearAwgnOFDM(0.14f, 25.0f), "near-AWGN fading threshold should allow R2/3 cutoff margin");
    CHECK(!isNearAwgnOFDM(0.15f, 25.0f), "near-AWGN fading threshold should match R2/3 cutoff");
    CHECK(!isNearAwgnOFDM(0.30f, 25.0f), "near-AWGN fading threshold is strict");
    CHECK(!isNearAwgnOFDM(0.00f, 24.9f), "near-AWGN in-band SNR threshold is strict");
    CHECK(isHighThroughputOFDM(0.30f, 25.0f), "Good fading in-band SNR25 should use high-throughput OFDM window");
    CHECK(!isHighThroughputOFDM(0.65f, 25.0f), "Moderate fading should not use high-throughput OFDM window yet");
    CHECK(!isHighThroughputOFDM(0.30f, 24.9f), "high-throughput OFDM in-band SNR threshold is strict");

    CHECK(isHighThroughputOFDMMode(Modulation::DQPSK, CodeRate::R1_2),
          "DQPSK R1/2 should use high-throughput OFDM mode");
    CHECK(!isHighThroughputOFDMMode(Modulation::DQPSK, CodeRate::R1_4),
          "DQPSK R1/4 should keep default OFDM mode");
    CHECK(ofdmWindowSize(Modulation::DQPSK, CodeRate::R1_2) == kHighThroughputOFDMWindowFrames,
          "high-throughput OFDM window size");
    CHECK(ofdmWindowSize(Modulation::DQPSK, CodeRate::R1_4) == kWideOFDMWindowFrames,
          "default OFDM window size");
    CHECK(ofdmWindowSize(Modulation::DQPSK, CodeRate::R2_3, true) == kHighThroughputOFDMWindowFrames,
          "R2/3 can use two burst groups only near AWGN");
    CHECK(ofdmWindowSize(Modulation::DQPSK, CodeRate::R2_3, false) == kWideOFDMWindowFrames,
          "legacy R2/3 window helper should stay conservative on fading channels");
    CHECK(ofdmWindowSizeForChannel(Modulation::DQPSK, CodeRate::R2_3, 0.30f, 25.0f)
              == kWideOFDMWindowFrames,
          "R2/3 should keep one burst group on good fading at in-band SNR25");
    CHECK(ofdmWindowSizeForChannel(Modulation::DQPSK, CodeRate::R2_3, 0.05f, 25.0f)
              == kHighThroughputOFDMWindowFrames,
          "R2/3 can use two burst groups only on near-AWGN channels");
    CHECK(ofdmWindowSizeForChannel(Modulation::DQPSK, CodeRate::R2_3, 0.80f, 25.0f)
              == kWideOFDMWindowFrames,
          "R2/3 should keep one burst group on moderate fading");
    CHECK(ofdmWindowSizeForChannel(Modulation::DQPSK, CodeRate::R3_4, 0.30f, 25.0f)
              == kWideOFDMWindowFrames,
          "R3/4 remains near-AWGN only for two burst groups");
    CHECK(ofdmWindowSize(Modulation::DQPSK, CodeRate::R1_2, false) == kHighThroughputOFDMWindowFrames,
          "R1/2 keeps the high-throughput OFDM window in fading");
    CHECK(!isBurstInterleavedOFDMMode(Modulation::QAM16, CodeRate::R1_2),
          "coherent QAM16 R1/2 should keep per-frame sync/channel tracking");
    CHECK(!isBurstInterleavedOFDMMode(Modulation::QAM16, CodeRate::R3_4),
          "coherent QAM16 R3/4 should keep per-frame sync/channel tracking");
    CHECK(isBurstInterleavedOFDMMode(Modulation::DQPSK, CodeRate::R2_3),
          "legacy speculative DQPSK high-rate burst interleaving should remain enabled");
    CHECK(!isBurstInterleavedOFDMMode(Modulation::DQPSK, CodeRate::R1_2),
          "non-speculative DQPSK R1/2 should keep existing burst-interleave behavior");
    CHECK(!isBurstInterleavedOFDMMode(Modulation::QPSK, CodeRate::R2_3),
          "non-QAM16 coherent modes should not inherit the QAM16 burst-interleave gate");
    // Burst interleave-ON group size stays 6 (<= the 16-bit SACK frame_mask ceiling; the
    // interleave-ON group is a fade-diversity tradeoff, not re-swept wider — see connection_policy).
    CHECK(!shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, false, 1),
          "single high-rate fading frame should not be padded");
    CHECK(shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, false, 2),
          "partial high-rate fading burst should be padded");
    CHECK(shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, false, 5),
          "short high-rate fading burst (<6) should be padded to fill the group");
    CHECK(!shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, false, 6),
          "full high-rate fading burst (one 6-frame group) should not be padded");
    CHECK(shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, false, 7),
          "multi-group high-rate fading tail (6+1) should be padded");
    CHECK(!shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, false, 12),
          "two full 6-frame groups should not be padded");
    CHECK(!shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R2_3, true, 7),
          "near-AWGN high-rate burst should not be padded");
    CHECK(!shouldPadHighRateFadingBurst(Modulation::DQPSK, CodeRate::R1_2, false, 7),
          "R1/2 fading burst should not use speculative high-rate padding");
    // After 2026-05-04 D8PSK ladder re-enable, D8PSK R2/3 is now a
    // speculative high-rate OFDM mode (same window/padding policy as
    // DQPSK R2/3). Padding fires for partial high-rate fading bursts.
    CHECK(shouldPadHighRateFadingBurst(Modulation::D8PSK, CodeRate::R2_3, false, 7),
          "D8PSK R2/3 fading partial burst should pad like DQPSK R2/3");
    CHECK(!shouldPadHighRateFadingBurst(Modulation::D8PSK, CodeRate::R1_2, false, 7),
          "D8PSK R1/2 is high-throughput non-speculative, no padding");
    CHECK(!shouldPadHighRateFadingBurst(Modulation::QPSK, CodeRate::R2_3, false, 7),
          "non-(DQPSK/D8PSK) high-rate burst should not use padding policy");
    CHECK(!shouldPadBurstInterleaveGroup(1),
          "single frame does not need burst-interleaver padding");
    CHECK(shouldPadBurstInterleaveGroup(7),
          "partial burst-interleaver group should be padded");
    CHECK(!shouldPadBurstInterleaveGroup(kBurstInterleaveGroupFrames),
          "complete burst-interleaver group should not be padded");
    CHECK(ofdmAckBatchSize(true) == 0, "near-AWGN ACK batch disabled");
    CHECK(ofdmAckBatchSize(false) == 0, "fading ACK batch sentinel");

    CHECK(kCarrierSenseSackCoalesceMs == 30,
          "OFDM SACK policy should use a small coalescing timer, not burst-tail guessing");
    CHECK(kCarrierSenseAckRepeatCount == 1,
          "OFDM ACK policy should not schedule delayed duplicate ACK bursts");
}

void test_negotiated_mode_selection() {
    const uint8_t all = ModeCapabilities::ALL;
    CHECK(selectNegotiatedMode(all, all, WaveformMode::MC_DPSK, WaveformMode::AUTO,
                               WaveformMode::AUTO, 20.0f, 0.0f) == WaveformMode::MC_DPSK,
          "remote explicit preference should win when common");

    // OFDM_NARROW DISABLED (thread A, 2026-05-31 — dropped from ModeCapabilities::ALL):
    // a narrowband override can no longer select it (not a common capability), so it
    // falls back to the supported local preference. Restore the OFDM_NARROW expectation
    // when OFDM_NARROW returns as coherent (REMOVAL_BACKLOG R3).
    CHECK(selectNegotiatedMode(all, all, WaveformMode::AUTO, WaveformMode::OFDM_NARROW,
                               WaveformMode::OFDM_CHIRP, 20.0f, 0.0f) == WaveformMode::OFDM_CHIRP,
          "narrowband override is moot while OFDM_NARROW is disabled (falls back to OFDM_CHIRP)");

    CHECK(selectNegotiatedMode(all, all, WaveformMode::AUTO, WaveformMode::AUTO,
                               WaveformMode::OFDM_CHIRP, 20.0f, 0.0f) == WaveformMode::OFDM_CHIRP,
          "local preference should win when common");

    CHECK(selectNegotiatedMode(all, ModeCapabilities::MC_DPSK, WaveformMode::AUTO,
                               WaveformMode::AUTO, WaveformMode::AUTO,
                               20.0f, 0.0f) == WaveformMode::MC_DPSK,
          "fallback should select common maintained mode when recommendation unavailable");

    CHECK(selectNegotiatedMode(0, ModeCapabilities::MC_DPSK, WaveformMode::AUTO,
                               WaveformMode::AUTO, WaveformMode::AUTO,
                               20.0f, 0.0f) == WaveformMode::MC_DPSK,
          "no common modes should fall back to MC-DPSK (universal floor), never OFDM_COX");
}

void test_auto_data_mode_boundaries() {
    const uint8_t all = ModeCapabilities::ALL;
    Modulation mod = Modulation::AUTO;
    CodeRate rate = CodeRate::AUTO;

    WaveformMode waveform = selectNegotiatedMode(
        all, all, WaveformMode::AUTO, WaveformMode::AUTO, WaveformMode::AUTO,
        20.0f, 0.30f);
    CHECK(waveform == WaveformMode::OFDM_CHIRP,
          "GOOD fading SNR20 auto-negotiates OFDM_CHIRP");
    recommendDataMode(20.0f, waveform, mod, rate, 0.30f);
    // 2026-07-26: anchors re-measured on ITU Good fading. 16QAM R2/3's G20 anchor measured
    // 51.4% FER and never beats 8PSK R2/3 in 16-24 dB, so it moved to G26; 8PSK R2/3 moved to
    // G17 (its measured crossover vs QPSK R3/4). Good@20 therefore selects 8PSK R2/3.
    CHECK(mod == Modulation::QAM8, "GOOD fading SNR20 auto -> 8PSK (measured best rung on fading)");
    CHECK(rate == CodeRate::R2_3, "GOOD fading SNR20 auto -> 8PSK R2/3 (G17 anchor, measured crossover)");

    // fading 0.79 is MODERATE class (>= kFadingGoodMax 0.76). R2/3 is now ENABLED on moderate at >= 20 dB
    // (measured 2026-06-09: genuine moderate R2/3 9/9 PASS @20-24 dB) — so a moderate-classified
    // channel at SNR20 gets R2/3, not R1/2. This is the SOFTENED Good/Moderate cliff: a
    // good-channel misclassified as moderate now costs 1 rung (R3/4->R2/3) instead of 2.
    recommendDataMode(20.0f, waveform, mod, rate, 0.79f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R2_3,
          "moderate (fading 0.79) at SNR20 selects QPSK R2/3 — softened cliff (measured 2026-06-09)");

    recommendDataMode(19.8f, waveform, mod, rate, 0.50f);
    CHECK(mod == Modulation::QAM8 && rate == CodeRate::R2_3,
          "GOOD fading 19.8 -> QAM8 R2/3 (G19 anchor; default ladder = psk8-exp 2026-07-05)");

    recommendDataMode(15.0f, waveform, mod, rate, 0.50f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R2_3,
          "GOOD fading at the R2/3 gate (>=15) selects QPSK R2/3");

    recommendDataMode(14.9f, waveform, mod, rate, 0.50f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_2,
          "GOOD fading just under the R2/3 gate selects coherent QPSK R1/2");

    waveform = selectNegotiatedMode(
        all, all, WaveformMode::AUTO, WaveformMode::AUTO, WaveformMode::AUTO,
        16.9f, 0.30f);
    CHECK(waveform == WaveformMode::OFDM_CHIRP,
          "GOOD fading SNR16.9 remains above the OFDM floor");
    recommendDataMode(16.9f, waveform, mod, rate, 0.30f);
    CHECK(mod == Modulation::QPSK,
          "GOOD fading SNR16.9 selects coherent QPSK");
    CHECK(rate == CodeRate::R2_3,
          "GOOD fading SNR16.9 is above the R2/3 gate (>=15)");

    waveform = selectNegotiatedMode(
        all, all, WaveformMode::AUTO, WaveformMode::AUTO, WaveformMode::AUTO,
        7.9f, 0.30f);
    CHECK(waveform == WaveformMode::MC_DPSK,
          "GOOD fading below the OFDM floor (8) keeps MC-DPSK");
    recommendDataMode(9.9f, waveform, mod, rate, 0.30f);
    CHECK(mod == Modulation::DQPSK && rate == CodeRate::R1_4,
          "MC-DPSK floor still uses DQPSK R1/4");

    waveform = selectNegotiatedMode(
        all, all, WaveformMode::AUTO, WaveformMode::AUTO, WaveformMode::AUTO,
        20.0f, 0.05f);
    CHECK(waveform == WaveformMode::OFDM_CHIRP,
          "AWGN SNR20 auto-negotiates OFDM_CHIRP");
    recommendDataMode(20.0f, waveform, mod, rate, 0.05f);
    // QAM8 R3/4 auto-disabled 2026-07-06 (AWGN-only anchor, fading-poisonous).
    CHECK(mod == Modulation::QAM8 && rate == CodeRate::R2_3,
          "AWGN SNR20 selects QAM8 R2/3 (top enabled AWGN rung)");

    waveform = selectNegotiatedMode(
        all, all, WaveformMode::AUTO, WaveformMode::AUTO, WaveformMode::AUTO,
        20.0f, 0.90f);
    CHECK(waveform == WaveformMode::OFDM_CHIRP,
          "Moderate fading SNR20 auto-negotiates OFDM_CHIRP");
    recommendDataMode(20.0f, waveform, mod, rate, 0.90f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R2_3,
          "Moderate fading SNR20 selects coherent QPSK R2/3 (enabled 2026-06-09, softened cliff)");

    // Coherent-only OFDM (thread A, 2026-05-31): Poor fading routes to MC-DPSK at ALL
    // SNRs (OFDM retired from Poor). MC-DPSK is differential — so Poor stays differential,
    // just on the robust MC-DPSK waveform instead of OFDM.
    waveform = selectNegotiatedMode(
        all, all, WaveformMode::AUTO, WaveformMode::AUTO, WaveformMode::AUTO,
        20.0f, 1.20f);
    CHECK(waveform == WaveformMode::MC_DPSK,
          "Poor fading SNR20 negotiates MC-DPSK (OFDM retired from Poor)");
    recommendDataMode(20.0f, waveform, mod, rate, 1.20f);
    CHECK(mod == Modulation::DQPSK && rate == CodeRate::R1_4,
          "Poor fading on MC-DPSK uses DQPSK R1/4");
}

void test_recommend_cw_count() {
    // Wide OFDM: rate-based promotion (R1/2, R2/3, R3/4 → 8; R1/4 → 4).
    CHECK(recommendCWCount(CodeRate::R1_2, WaveformMode::OFDM_CHIRP) == 8,
          "wide R1/2 → 8 (the +50% throughput win)");
    CHECK(recommendCWCount(CodeRate::R2_3, WaveformMode::OFDM_CHIRP) == 8,
          "wide R2/3 → 8");
    CHECK(recommendCWCount(CodeRate::R3_4, WaveformMode::OFDM_CHIRP) == 8,
          "wide R3/4 → 8");
    CHECK(recommendCWCount(CodeRate::R1_4, WaveformMode::OFDM_CHIRP) ==
              v2::kDefaultFixedFrameCodewords,
          "wide R1/4 stays at default 4 (low-SNR robustness)");

    // OFDM_NARROW: always default 4. Narrow R1/2 frames are ~6 s at CW=8;
    // window=3 burst would be ~18 s, exceeding typical narrow good-fading
    // coherence (~10 s). 3-seed sim A/B at SNR=8 R1/2 confirmed CW=8
    // hit 1/3 catastrophic FAIL (240 s timeout) while CW=4 was 3/3 PASS.
    for (auto rate : {CodeRate::R1_4, CodeRate::R1_2, CodeRate::R2_3, CodeRate::R3_4}) {
        CHECK(recommendCWCount(rate, WaveformMode::OFDM_NARROW) ==
                  v2::kDefaultFixedFrameCodewords,
              "narrow always caps at default 4 (fade-coherence limit)");
    }

    CHECK(recommendCWCount(Modulation::DBPSK, CodeRate::R1_4, WaveformMode::MC_DPSK) == 3,
          "Robust-Low MC-DPSK DBPSK uses 3-CW variable frames");
    CHECK(recommendCWCount(Modulation::DQPSK, CodeRate::R1_4, WaveformMode::MC_DPSK) ==
              v2::kDefaultFixedFrameCodewords,
          "standard MC-DPSK DQPSK keeps the legacy CW count");
    CHECK(recommendCWCount(Modulation::DBPSK, CodeRate::R1_4, WaveformMode::OFDM_CHIRP) ==
              v2::kDefaultFixedFrameCodewords,
          "DBPSK does not alter OFDM CW policy");

    // ITU-R F.1487 Good = 0.1 Hz Doppler -> Clarke Tc = 0.423/fD ~= 4230 ms.
    // (a1c9c34 mislabeled the 0.5 Hz Moderate value as "Good"; corrected 2026-05-26.)
    CHECK(coherenceTimeMsForDoppler(kGoodHFDesignDopplerHz) == 4230,
          "Good-HF design Doppler (0.1 Hz) gives ~4230 ms Clarke coherence time");
    CHECK(coherenceTimeMsForDoppler(kModerateHFDesignDopplerHz) == 846,
          "Moderate-HF design Doppler (0.5 Hz) gives ~846 ms Clarke coherence time");
    // Good coherent QPSK R2/3 (fading_index ~0.50): a cw=8 frame (~1392 ms) fits the
    // ~4230 ms Good coherence, so it keeps the full 8-CW throughput geometry.
    CHECK(recommendCWCountForChannel(Modulation::QPSK, CodeRate::R2_3,
                                     WaveformMode::OFDM_CHIRP, 0.50f, 20.0f) == 8,
          "Good coherent QPSK R2/3 keeps cw=8 inside the true Good coherence time");
    // Moderate (fading_index ~0.90): 0.5 Hz -> 846 ms coherence caps cw=8 (1272 ms at sp8)
    // down to cw=5 (816 ms, fits) so a single fade event cannot take the whole frame. (Was cw=4
    // at the old dense sp5 pilots; the 2026-06-14 R2/3 sp8 default shortens the frame, so the cap
    // fits one more CW inside coherence — more Moderate throughput at the same robustness.)
    CHECK(recommendCWCountForChannel(Modulation::QPSK, CodeRate::R2_3,
                                     WaveformMode::OFDM_CHIRP, 0.90f, 20.0f) == 5,
          "Moderate coherent QPSK R2/3 caps frame length inside coherence time");
    CHECK(recommendCWCountForChannel(Modulation::QPSK, CodeRate::R2_3,
                                     WaveformMode::OFDM_CHIRP, 0.0f, 27.0f) == 8,
          "near-AWGN coherent QPSK keeps the throughput CW count");
    CHECK(recommendCWCountForChannel(Modulation::DQPSK, CodeRate::R2_3,
                                     WaveformMode::OFDM_CHIRP, 0.50f, 20.0f) == 8,
          "differential OFDM keeps deterministic CW policy");
    CHECK(recommendCWCountForFading(Modulation::QPSK, CodeRate::R2_3,
                                    WaveformMode::OFDM_CHIRP, 0.50f) == 8,
          "SNR-free Good-channel CW refinement matches production geometry");
    CHECK(recommendCWCountForFading(Modulation::QPSK, CodeRate::R2_3,
                                    WaveformMode::OFDM_CHIRP, 0.90f) == 5,
          "SNR-free Moderate-channel CW refinement keeps the coherence cap");
}

void test_candidate_specific_burst_geometry() {
    constexpr uint32_t kBaseCeilingMs = 8600;
    const int qpsk_r14_cw = recommendCWCount(
        Modulation::QPSK, CodeRate::R1_4, WaveformMode::OFDM_CHIRP);
    const int qpsk_r12_cw = recommendCWCount(
        Modulation::QPSK, CodeRate::R1_2, WaveformMode::OFDM_CHIRP);
    const int qam8_r23_cw = recommendCWCount(
        Modulation::QAM8, CodeRate::R2_3, WaveformMode::OFDM_CHIRP);

    CHECK(wideOFDMBurstFrameBudget(Modulation::QPSK, CodeRate::R1_4,
                                   qpsk_r14_cw, 8, kBaseCeilingMs) == 8,
          "QPSK R1/4 candidate fits eight short frames at the base ceiling");
    CHECK(wideOFDMBurstFrameBudget(Modulation::QPSK, CodeRate::R1_2,
                                   qpsk_r12_cw, 8, kBaseCeilingMs) == 5,
          "QPSK R1/2 candidate fits five normalized frames at the base ceiling");
    CHECK(wideOFDMBurstFrameBudget(Modulation::QAM8, CodeRate::R2_3,
                                   qam8_r23_cw, 16, kBaseCeilingMs) == 5,
          "8PSK R2/3 candidate fits five normalized frames at the base ceiling");
    CHECK(wideOFDMBurstFrameBudget(
              Modulation::QAM8, CodeRate::R2_3, qam8_r23_cw, 16,
              kBaseCeilingMs, /*continuation_reanchor_ms=*/0,
              /*data_lifting_z=*/27, kWideOFDMFullAnchorExtraMs) == 4,
          "8PSK full-anchor repair fits only four frames at the base ceiling");

    CHECK(burstAirtimeCeilingMs(Modulation::QPSK, CodeRate::R1_2, 2) ==
              kBaseCeilingMs,
          "clean QPSK delivery does not bypass the default dense-rung escalation gate");
    const uint32_t dense_ceiling = burstAirtimeCeilingMs(
        Modulation::QAM8, CodeRate::R2_3, 2);
    CHECK(dense_ceiling == 11500,
          "two clean groups earn the production dense-rung ceiling");
    CHECK(wideOFDMBurstFrameBudget(Modulation::QAM8, CodeRate::R2_3,
                                   qam8_r23_cw, 16, dense_ceiling) == 8,
          "the escalated 8PSK candidate fits eight frames, not observed-M frames");
    CHECK(wideOFDMBurstFrameBudget(
              Modulation::QAM8, CodeRate::R2_3, qam8_r23_cw, 16,
              dense_ceiling, /*continuation_reanchor_ms=*/0,
              /*data_lifting_z=*/27, kWideOFDMFullAnchorExtraMs) == 7,
          "the escalated full-anchor repair fits seven frames, not eight");

    // Receiver-local fading is not part of a tone-ACK rate command.  In the fixed04
    // hardware transfer the Mac classified its side Moderate and used CW=5 to price
    // QPSK R2/3, while the Pi's next descriptor truthfully announced CW=8.  That made
    // the selector log candidate_N=9 for a physical sender that could fit only N=5.
    const int receiver_local_moderate_cw = recommendCWCountForFading(
        Modulation::QPSK, CodeRate::R2_3, WaveformMode::OFDM_CHIRP, 0.90f);
    const int wire_default_candidate_cw = receiverRateCommandCandidateCWCount(
        Modulation::QPSK, CodeRate::R2_3, WaveformMode::OFDM_CHIRP);
    CHECK(receiver_local_moderate_cw == 5,
          "Moderate receiver-local fading still refines its own sender geometry to CW=5");
    CHECK(wire_default_candidate_cw == 8,
          "receiver-issued rate command prices the peer-independent CW=8 baseline");
    CHECK(wideOFDMBurstFrameBudget(Modulation::QPSK, CodeRate::R2_3,
                                   receiver_local_moderate_cw, 16,
                                   kBaseCeilingMs) == 9,
          "fixed04 regression control reproduces the impossible local CW=5/N=9 quote");
    CHECK(wideOFDMBurstFrameBudget(Modulation::QPSK, CodeRate::R2_3,
                                   wire_default_candidate_cw, 16,
                                   kBaseCeilingMs) == 5,
          "rate-command counterfactual matches the sender's fixed04 CW=8/N=5 geometry");
    CHECK(receiverRateCommandCandidateCWCount(
              Modulation::QPSK, CodeRate::R2_3, WaveformMode::OFDM_CHIRP, 5) == 5,
          "explicit forced-CW override remains authoritative in candidate geometry");
}

void test_variable_frame_payload_capacity() {
    CHECK(v2::getVariableFramePayloadCapacity(CodeRate::R1_4, 1) == 1,
          "R1/4 variable 1-CW DATA carries only 1 payload byte");
    CHECK(v2::getVariableFramePayloadCapacity(CodeRate::R1_4, 2) == 19,
          "R1/4 variable 2-CW DATA carries 19 payload bytes");
    CHECK(v2::getVariableFramePayloadCapacity(CodeRate::R1_4, 3) == 37,
          "R1/4 variable 3-CW DATA carries 37 payload bytes");
    CHECK(v2::getVariableFramePayloadCapacity(CodeRate::R1_4, 4) == 55,
          "R1/4 variable 4-CW DATA carries 55 payload bytes");

    for (int cw = v2::kMinFixedFrameCodewords; cw <= v2::kMaxFixedFrameCodewords; ++cw) {
        const size_t cap = v2::getVariableFramePayloadCapacity(CodeRate::R1_4, cw);
        CHECK(v2::DataFrame::calculateCodewords(cap, CodeRate::R1_4) == cw,
              "variable frame capacity should fit exactly in the target CW count");
        if (cw < v2::kMaxFixedFrameCodewords) {
            CHECK(v2::DataFrame::calculateCodewords(cap + 1, CodeRate::R1_4) == cw + 1,
                  "one byte over variable capacity should require one more CW");
        }
    }
}

// Phase 2a: the warm SHORT-DUAL descriptor anchor is GUI-measured to be a clean win only on
// benign Good/AWGN and to crater on fading (the bad seed relocates with chirp duration but never
// disappears). shouldUseWarmShortAnchorDescriptor gates it to the rate ladder's top rung (R3/4)
// on coherent wideband OFDM — a robust sender-side "benign" proxy that auto-reverts to the full
// dual chirp the moment the ladder drops off R3/4. This pins that gate so it can't silently widen.
void test_warm_short_anchor_descriptor_gate() {
    using WF = protocol::WaveformMode;

    // Fires only on coherent wideband OFDM at the TOP rung.
    CHECK(connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QPSK, CodeRate::R3_4),
          "short anchor must fire on OFDM_CHIRP coherent QPSK R3/4 (benign top rung)");
    CHECK(connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QAM8, CodeRate::R3_4),
          "short anchor fires on any coherent modulation at R3/4 (8PSK R3/4)");

    // QPSK is suppressed below its top rung — R2/3, R1/2, R1/4 indicate a less-benign channel.
    CHECK(!connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QPSK, CodeRate::R2_3),
          "short anchor must NOT fire on QPSK below R3/4 (R2/3)");
    CHECK(!connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_2),
          "short anchor must NOT fire on QPSK below R3/4 (R1/2)");
    CHECK(!connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QPSK, CodeRate::R1_4),
          "short anchor must NOT fire on QPSK below R3/4 (R1/4)");

    // 2026-06-14: FIRES on dense coherent mods (>=16QAM) at ANY rate — 16QAM is benign-selected
    // (Good-only), and the descriptor-chirp reclaim is proportionally bigger on the denser payload.
    CHECK(connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QAM16, CodeRate::R2_3),
          "short anchor FIRES on 16QAM R2/3 (dense mod, benign-selected)");
    CHECK(connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::QAM16, CodeRate::R1_2),
          "short anchor FIRES on 16QAM R1/2 (dense mod, benign-selected)");

    // Suppressed for differential modulation (no coherent burst path) ...
    CHECK(!connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_CHIRP, Modulation::DQPSK, CodeRate::R3_4),
          "short anchor must NOT fire on differential DQPSK even at R3/4");

    // ... and for non-wideband-OFDM waveforms (narrowband / MC-DPSK have their own anchors).
    CHECK(!connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::OFDM_NARROW, Modulation::QPSK, CodeRate::R3_4),
          "short anchor must NOT fire on OFDM_NARROW");
    CHECK(!connection_policy::shouldUseWarmShortAnchorDescriptor(
              WF::MC_DPSK, Modulation::QPSK, CodeRate::R3_4),
          "short anchor must NOT fire on MC_DPSK");
}

// 2026-06-19: the burst-transport ACK deadline (unifiedBurstAckTimeoutMs) must NEVER fire before
// the receiver can realistically return its tone-burst group-ACK, for EVERY mod/rate/cw/frame. The
// half-duplex peer can only ACK after: burst fully on air (real-time airtime) + its deliberate
// SACK-coalesce holdoff + LDPC decode + ack-return airtime. The bug this guards: the burst formula
// omitted the receiver's SACK holdoff, so a full cw8 burst (IONOS MPG E5, seq63-67) timed out ~1-2 s
// early and resent the whole group though 4/5 frames had decoded and the ACK was in flight.
void test_unified_burst_ack_timeout() {
    const Modulation mods[] = {Modulation::DQPSK, Modulation::QPSK,
                               Modulation::QAM8, Modulation::QAM16};
    const CodeRate rates[] = {CodeRate::R1_4, CodeRate::R1_2,
                              CodeRate::R2_3, CodeRate::R3_4};
    const int cws[] = {1, 4, 5, 6, 8};
    const size_t frame_counts[] = {1, 3, 5, 8, 16};
    const int z_values[] = {27, 81};
    const Modulation control_mod = Modulation::DQPSK;  // robust control (matches production)
    const uint32_t cfg_sack = kCarrierSenseSackCoalesceMs;
    int cells = 0;
    bool all_cover = true;
    for (Modulation m : mods)
        for (CodeRate r : rates)
            for (int cw : cws)
                for (size_t f : frame_counts)
                    for (int z : z_values) {
                        const uint32_t to = unifiedBurstAckTimeoutMs(
                            m, r, cw, f, z, control_mod, cfg_sack);
                        const size_t frames = std::max<size_t>(1, f);
                        // Hard lower bound the deadline MUST cover (2026-07-02 contract,
                        // BUG-ACK-TIMEOUT-DOUBLECOUNT closed): the receiver cannot respond
                        // later than (burst on air) + (its group-timeout fast-NACK budget:
                        // remaining-airtime/2 + 3000 ms — rig-calibrated: the clean-path
                        // SACK hold measured 0-1 ms over 124 groups) + (1-CW ack return).
                        // decode margin / turnaround slack are headroom on top.
                        const uint32_t burst =
                            wideOFDMBurstAirtimeMs(m, r, frames, cw, 0, z);
                        const uint32_t data_ms =
                            wideOFDMFrameTiming(m, r, cw, z).data_ms;
                        const uint32_t rx_response =
                            (frames > 1 ? static_cast<uint32_t>(
                                              (frames - 1) *
                                              static_cast<size_t>(data_ms) / 2)
                                        : 0u) +
                            3000u;
                        const uint32_t ack_ret =
                            wideOFDMFrameTiming(control_mod, CodeRate::R1_4).ack_ms;
                        const uint32_t required = burst + rx_response + ack_ret;
                        if (to < required) all_cover = false;
                        ++cells;
                    }
    CHECK(cells == 4 * 4 * 5 * 5 * 2, "burst ACK timeout matrix fully swept");
    CHECK(all_cover,
          "unified burst ACK timeout must cover burst airtime + the receiver's fast-NACK "
          "response budget + ack return for EVERY mod/rate/cw/frame/z");

    // DOUBLECOUNT regression (2026-07-02): the deadline must be COHERENT, not padded — for
    // the live QPSK R2/3 cw8 g5 geometry it must sit well below the old ~24.5 s
    // double-counted value while still covering the required floor above.
    CHECK(unifiedBurstAckTimeoutMs(Modulation::QPSK, CodeRate::R2_3, 8, 5, 27,
                                   control_mod, cfg_sack) < 20000,
          "burst ACK deadline must not double-count burst airtime (was ~24.5 s pre-fix)");

    // E5 regression: the exact live case (QPSK R2/3, cw8, 5-frame burst, z=27). Pre-06-19 the
    // deadline fired before the receiver's in-flight SACK and resent seq63-67 prematurely. The
    // deadline must exceed burst airtime + the worst receiver response (fast-NACK budget).
    const uint32_t e5_to = unifiedBurstAckTimeoutMs(
        Modulation::QPSK, CodeRate::R2_3, 8, 5, 27, control_mod, cfg_sack);
    const uint32_t e5_burst =
        wideOFDMBurstAirtimeMs(Modulation::QPSK, CodeRate::R2_3, 5, 8, 0, 27);
    const uint32_t e5_data_ms =
        wideOFDMFrameTiming(Modulation::QPSK, CodeRate::R2_3, 8, 27).data_ms;
    const uint32_t e5_rx_response =
        static_cast<uint32_t>(4 * static_cast<size_t>(e5_data_ms) / 2) + 3000u;
    CHECK(e5_to > e5_burst + e5_rx_response,
          "E5: QPSK R2/3 cw8 5-frame burst deadline must exceed burst airtime + the "
          "receiver's fast-NACK response budget");

    // NARROW path shares the bug: computeNarrowOFDMAckTimeoutMs previously omitted the 1500 ms
    // receiver tone-burst SACK hold and went 0.3-1.5 s short for QPSK/8PSK/QAM16 at window=3. The
    // deadline must cover window burst airtime + the receiver hold + the ack turnaround, every
    // narrow mod/cw/window. (Narrow's tone-burst listen window already floors to getAckTimeout
    // because isOFDMMode(OFDM_NARROW) is true, so the timeout fix is sufficient here.)
    const Modulation narrow_mods[] = {Modulation::QPSK, Modulation::QAM8, Modulation::QAM16,
                                      Modulation::DBPSK, Modulation::DQPSK};
    const int narrow_cws[] = {1, 4, 8};
    const size_t narrow_windows[] = {1, 2, 3};
    bool narrow_cover = true;
    for (Modulation m : narrow_mods)
        for (int cw : narrow_cws)
            for (size_t w : narrow_windows) {
                const uint32_t to = computeNarrowOFDMAckTimeoutMs(m, cw, w);
                const OFDMFrameTiming t = narrowOFDMFrameTiming(m, cw);
                const uint32_t required =
                    static_cast<uint32_t>(w) * t.data_ms +
                    kToneBurstReceiverSackHoldMs + 2 * t.ack_ms;
                if (to < required) narrow_cover = false;
            }
    CHECK(narrow_cover,
          "narrow OFDM ACK timeout must cover window burst + 1500ms receiver SACK hold + ack "
          "turnaround for every narrow mod/cw/window");
    // The receiver hold is actually budgeted (the fix): the deadline includes >= the 1500 ms hold.
    CHECK(computeNarrowOFDMAckTimeoutMs(Modulation::QAM16, 4, 3) >
              3u * narrowOFDMFrameTiming(Modulation::QAM16, 4).data_ms +
                  kToneBurstReceiverSackHoldMs,
          "narrow QAM16 win=3 deadline must exceed burst + receiver SACK hold (the live short case)");
}

// Saturation bound (2026-07-02): on Moderate-class fading the data-aided
// differential-EVM estimator saturates at the Doppler-EVM floor, so an
// at/above-zone reading LOWER-BOUNDS true SNR and justifies OFDM entry. The
// argument is estimator-specific: the training snapshot fade-crest OVER-reads
// (7.8 measured at true Moderate@8) and must never trip the bound.
void test_connect_selection_saturation_bound() {
    const float mod_floor = kOFDMEntryFloorModerateDb;

    // MPM@20 measured case: data-aided reads 7.7 (saturated) -> bound fires,
    // selection clears the Moderate OFDM entry floor. (Index 0.85 = mid-Moderate;
    // the earlier 0.70 sat just under the new Good/Moderate boundary 0.76 and now
    // classes Good — a real MPM channel reads ~0.85, so this is also more faithful.)
    CHECK(connectSelectionSnrDb(7.7f, 0.85f, true) >= mod_floor,
          "data-aided saturated reading on Moderate must clear the OFDM floor");

    // MPM@8 measured case: training fade-crest reads up to 7.8 -> bound must
    // NOT fire; basis alone (7.8+5=12.8) stays below the floor -> MC-DPSK.
    CHECK(connectSelectionSnrDb(7.8f, 0.85f, false) < mod_floor,
          "training crest reading on Moderate must NOT clear the OFDM floor");

    // Below the saturation zone the reading is trustworthy: no bound even for
    // data-aided (true weak channel pulls the reading below the zone).
    CHECK(connectSelectionSnrDb(5.3f, 0.85f, true) < mod_floor,
          "below-zone data-aided reading keeps the MC-DPSK fallback");

    // Good-class fading (< kFadingGoodMax): bound never applies; basis only.
    CHECK(std::fabs(connectSelectionSnrDb(7.7f, 0.40f, true) -
                    (7.7f + connectSnrFadeBasisDb())) < 0.01f,
          "Good-class fading gets basis only, no saturation bound");

    // The good@20 s43 recalibration case is pure basis (10.6 -> 15.6).
    CHECK(std::fabs(connectSelectionSnrDb(10.6f, 0.40f, true) -
                    (10.6f + connectSnrFadeBasisDb())) < 0.01f,
          "fading basis path unchanged for Good-class readings");

    // AWGN passthrough regardless of the estimator flag.
    CHECK(std::fabs(connectSelectionSnrDb(12.0f, 0.05f, true) - 12.0f) < 0.01f,
          "AWGN selection is the raw reading (data-aided)");
    CHECK(std::fabs(connectSelectionSnrDb(12.0f, 0.05f, false) - 12.0f) < 0.01f,
          "AWGN selection is the raw reading (training)");
}

// Calibrated affine entry-SNR basis (2026-07-03, ULTRA_CONNECT_AFFINE_BASIS,
// docs/CONNECT_ENTRY_CALIBRATION_2026_07_03.md §7). Golden-constant derivation:
// 48 rig entries, EVERY one at known dial 20 (MPG@20 Watterson Good), data-aided
// readings 6.2-19.4 (mean 12.38, sample sigma 3.14). Least squares of dial on
// reading with a constant target is EXACT and DEGENERATE: slope a = 0, intercept
// b = 20, in-sample residual sigma = 0 (offset := 20 - reading is an exact affine
// function of the reading, slope -1) — within the calibrated population the
// reading carries NO dial information; the spread is fade-phase noise around one
// dial. Deployed intercept = b - sigma/sqrt(N) = 20 - 3.14/sqrt(48) = 19.55 (one
// standard error of the calibrated mean, the same one-sided-cost shrink as
// entryClassificationFadingIndex — and it keeps mid readings off the zero-margin
// Good QPSK R3/4 anchor at exactly 20.0). Correction clamped [+2, +11] dB = the
// extrapolation guard for readings outside the calibrated [6.2, 19.4] range.
void test_connect_affine_basis() {
    // ── Golden fit constants ──
    CHECK(std::fabs(kConnectAffineFitSlope) < 1e-6f,
          "LS slope on one-dial data is exactly 0 (degenerate by design)");
    CHECK(std::fabs(kConnectAffineFitInterceptDb - 20.0f) < 1e-6f,
          "LS intercept is exactly the calibration dial (20.0)");
    CHECK(kConnectEntryCalibCount == 48, "calibration N = 48 ledger entries");
    CHECK(std::fabs(kConnectEntryReadingSigmaDb - 3.14f) < 1e-4f,
          "pinned sample sigma of the 48 readings (3.14 dB)");
    CHECK(std::fabs(kConnectAffineDialEquivDb -
                    (kConnectAffineFitInterceptDb -
                     kConnectEntryReadingSigmaDb /
                         std::sqrt(static_cast<float>(kConnectEntryCalibCount)))) <
              0.01f,
          "deployed intercept = 20 - sigma/sqrt(48) = 19.547 -> pinned 19.55");

    // ── Boundary cases: the ledger's own min/max readings both CLAMP ──
    CHECK(std::fabs(connectAffineCorrectionDb(6.2f) - kConnectAffineCorrMaxDb) <
              1e-4f,
          "reading 6.2 (ledger min): unclamped corr 13.35 -> clamped +11");
    CHECK(std::fabs(connectAffineCorrectionDb(19.4f) - kConnectAffineCorrMinDb) <
              1e-4f,
          "reading 19.4 (ledger max): unclamped corr 0.15 -> clamped +2");
    CHECK(std::fabs(dialEquivalentSnrDb(6.2f, 0.47f, true) - 17.2f) < 1e-3f,
          "reading 6.2 -> dial-equivalent 17.2 (clamp edge)");
    CHECK(std::fabs(dialEquivalentSnrDb(19.4f, 0.38f, true) - 21.4f) < 1e-3f,
          "reading 19.4 -> dial-equivalent 21.4 (clamp edge)");

    // ── Plateau: mid-range readings map to the (shrunk) calibration dial ──
    CHECK(std::fabs(dialEquivalentSnrDb(9.5f, 0.50f, true) -
                    kConnectAffineDialEquivDb) < 1e-3f,
          "reading 9.5 -> dial-equivalent 19.55 (plateau)");
    CHECK(std::fabs(dialEquivalentSnrDb(12.4f, 0.50f, true) -
                    kConnectAffineDialEquivDb) < 1e-3f,
          "mean reading 12.4 -> dial-equivalent 19.55 (plateau)");

    // ── Entry-rate table check: the user-critical dial-20 case ──
    // Reading 9.5 at Good-class fading: flat +5 gave sel 14.5 -> QPSK R1/2;
    // affine gives sel 19.55 -> QPSK R2/3 (Good anchors: R2/3=15, R3/4=20).
    const float sel_95 = connectSelectionSnrDb(9.5f, 0.50f, true, /*affine=*/true);
    CHECK(std::fabs(sel_95 - kConnectAffineDialEquivDb) < 1e-3f,
          "reading 9.5 data-aided Good -> sel 19.55");
    CHECK(selectOFDMCodeRate(sel_95, 0.50f) == CodeRate::R2_3,
          "dial-20-equivalent selection enters QPSK R2/3 on Good (was R1/2)");
    CHECK(selectOFDMCodeRate(9.5f + connectSnrFadeBasisDb(), 0.50f) ==
              CodeRate::R1_2,
          "flat-basis counterfactual: the same reading entered QPSK R1/2");
    CHECK(selectLadderRung(sel_95, 0.50f).waveform == WaveformMode::OFDM_CHIRP,
          "dial-20-equivalent selection stays an OFDM entry");
    // 19.55 sits BELOW the zero-margin Good R3/4 anchor (20.0) by the one-SE
    // shrink — the plateau must not buy the top rung.
    CHECK(selectOFDMCodeRate(sel_95, 0.50f) != CodeRate::R3_4,
          "the plateau does not land ON the zero-margin Good R3/4 anchor");

    // ── Knob-off identity: byte-identical flat path ──
    CHECK(std::fabs(connectSelectionSnrDb(9.5f, 0.50f, true, false) -
                    (9.5f + connectSnrFadeBasisDb())) < 1e-4f,
          "knob off: selection is exactly the flat basis");
    CHECK(std::fabs(dialEquivalentSnrDb(9.5f, 0.50f, false) -
                    (9.5f + connectSnrFadeBasisDb())) < 1e-4f,
          "knob off: dial-equivalent display is exactly the flat basis");
    CHECK(std::fabs(connectSelectionSnrDb(9.5f, 0.50f, true) -
                    connectSelectionSnrDb(9.5f, 0.50f, true, false)) < 1e-6f,
          "3-arg production wrapper == 4-arg with the pinned-OFF env knob");

    // ── AWGN passthrough regardless of the knob ──
    CHECK(std::fabs(dialEquivalentSnrDb(12.0f, 0.05f, true) - 12.0f) < 1e-6f,
          "AWGN: dial-equivalent is the raw reading (affine on)");
    CHECK(std::fabs(connectSelectionSnrDb(12.0f, 0.05f, true, true) - 12.0f) <
              1e-6f,
          "AWGN: selection is the raw reading (affine on)");

    // ── Calibration scope: affine is DATA-AIDED-only (the 48-entry population) ──
    // Training snapshot fade-crest OVER-reads (7.8 measured at true Moderate@8);
    // +11 on that crest would push a true-8dB channel deep into OFDM. Training
    // readings keep the flat basis even with the knob ON (MPM@8 safety case).
    CHECK(std::fabs(connectSelectionSnrDb(7.8f, 0.70f, false, true) -
                    (7.8f + connectSnrFadeBasisDb())) < 1e-4f,
          "training-routed reading keeps the flat basis under the affine knob");
    CHECK(connectSelectionSnrDb(7.8f, 0.70f, false, true) <
              kOFDMEntryFloorModerateDb,
          "MPM@8 training-crest safety unchanged: stays below the Moderate floor");

    // ── Saturation-bound composition unchanged, and RAW-keyed ──
    // Data-aided 7.7 on Moderate: the bound applies; the affine sel (18.7) already
    // exceeds floor+0.5 so max() keeps it — composition is exactly one max().
    const float sel_mod = connectSelectionSnrDb(7.7f, 0.70f, true, true);
    CHECK(std::fabs(sel_mod -
                    std::max(7.7f + connectAffineCorrectionDb(7.7f),
                             kOFDMEntryFloorModerateDb + 0.5f)) < 1e-4f,
          "affine sel composes with the saturation bound via exactly one max()");
    CHECK(sel_mod >= kOFDMEntryFloorModerateDb,
          "saturated Moderate data-aided reading still clears the OFDM floor");
    // RAW-keying proof: raw 3.0 (< 6.5 zone) maps to affine sel 14.0; were the 6.5
    // zone test keyed on the CORRECTED value (14.0 >= 6.5) the bound would lift it
    // to 14.5. It must stay 14.0.
    CHECK(std::fabs(connectSelectionSnrDb(3.0f, 0.70f, true, true) - 14.0f) < 1e-4f,
          "the 6.5 saturation-zone test stays keyed to the RAW reading");
}

void test_coherent_window_override_disabled_keeps_default() {
    // ULTRA_COHERENT_WINDOW is pinned to "0" in main() BEFORE any policy call — the
    // knob is latched once (static). Baseline contract: with the override disabled,
    // window selection is byte-identical to the pre-knob behavior — coherent rungs
    // (QPSK/8PSK/16QAM, all rates) keep the default wide window (8).
    CHECK(coherentOFDMWindowOverride() == 0,
          "ULTRA_COHERENT_WINDOW=0 must read as disabled");
    CHECK(ofdmWindowSizeForChannel(Modulation::QAM16, CodeRate::R2_3, 0.30f, 20.0f)
              == kWideOFDMWindowFrames,
          "QAM16 R2/3 keeps the default window 8 with the coherent override disabled");
    CHECK(ofdmWindowSize(Modulation::QAM16, CodeRate::R2_3) == kWideOFDMWindowFrames,
          "QAM16 R2/3 (legacy helper) keeps window 8 with the coherent override disabled");
    CHECK(ofdmWindowSizeForChannel(Modulation::QPSK, CodeRate::R3_4, 0.30f, 20.0f)
              == kWideOFDMWindowFrames,
          "QPSK R3/4 keeps the default window 8 with the coherent override disabled");
    CHECK(ofdmWindowSizeForChannel(Modulation::QAM8, CodeRate::R2_3, 0.30f, 20.0f)
              == kWideOFDMWindowFrames,
          "8PSK R2/3 keeps the default window 8 with the coherent override disabled");
    // The differential high-throughput predicate is independent of the knob.
    CHECK(ofdmWindowSize(Modulation::DQPSK, CodeRate::R1_2) == kHighThroughputOFDMWindowFrames,
          "DQPSK R1/2 high-throughput window is unaffected by the coherent override knob");
}

void test_qam16_r34_cap_knob_off_keeps_r23() {
    // ULTRA_QAM16_R34 is pinned to "0" in main() BEFORE any policy call — the knob is
    // latched once (static). Baseline contract: with the crest rung disabled, the
    // per-modulation validated-rate cap is byte-identical to the pre-knob behavior —
    // QAM16 caps at R2/3, QPSK at R3/4 (the auto-ladder top).
    CHECK(!qam16R34Enabled(), "ULTRA_QAM16_R34=0 must read as disabled");
    CHECK(maxValidatedCoherentRate(Modulation::QAM16) == CodeRate::R2_3,
          "QAM16 validated-rate cap stays R2/3 with the crest-rung knob disabled");
    CHECK(maxValidatedCoherentRate(Modulation::QPSK) == CodeRate::R3_4,
          "QPSK validated-rate cap stays R3/4 (ladder top) regardless of the knob");
    // Connect-time gate (knob-independent, structural): {QAM16, R3/4} is kRungDisabledDb
    // in BOTH ladder tables, so the crest rung is reachable ONLY via the adaptive walk —
    // even with the knob ON, selectCoherentOFDM can never pick it at CONNECT.
    for (const auto& rung : kCoherentLadder) {
        if (rung.mod == Modulation::QAM16 && rung.rate == CodeRate::R3_4) {
            for (int cls = 0; cls < 3; ++cls) {
                CHECK(rung.min_snr_db[cls] >= kRungDisabledDb,
                      "kCoherentLadder {QAM16, R3/4} must stay disabled for connect-time selection");
            }
        }
    }
    for (const auto& rung : kCoherentLadderQAM16Exp) {
        if (rung.mod == Modulation::QAM16 && rung.rate == CodeRate::R3_4) {
            for (int cls = 0; cls < 3; ++cls) {
                CHECK(rung.min_snr_db[cls] >= kRungDisabledDb,
                      "kCoherentLadderQAM16Exp {QAM16, R3/4} must stay disabled for connect-time selection");
            }
        }
    }
}

// §RETX-PACING (docs/RETX_PACING_DESIGN_2026_07_03.md §1.2): the pure deferral policy —
// T_defer(n) = clamp(frac·Tc·2^(n−1) − elapsed, 0, T_cycle/2), Tc = 0.423/f_D, abs cap
// 8000 ms — across the whole channel family (Good/Moderate/Poor fallback + measured f_D).
void test_retx_trough_defer() {
    // f_D fallback = design Doppler by fading class (invalid coherence -> raw fading index):
    // Good 0.1 Hz -> Tc 4230 ms, half-cycle 5000 ms.
    CHECK(retxTroughDeferMs(0.0f, 0.40f, 0.0f, false, 1, 0) == 4230,
          "Good fallback n=1 elapsed=0 defers one full Tc (4230 ms)");
    // Elapsed-listening subtraction (fast-NACK path, ~3 s already spent listening).
    CHECK(retxTroughDeferMs(0.0f, 0.40f, 0.0f, false, 1, 3000) == 1230,
          "elapsed listening time is subtracted from the hold");
    // The ~18 s RTO path already out-waited Tc -> defer exactly 0 (add nothing on top).
    CHECK(retxTroughDeferMs(0.0f, 0.40f, 0.0f, false, 1, 10000) == 0,
          "RTO path (elapsed >= target) must defer 0");
    // x2 escalation at n=2 (2*4230=8460) hits the T_cycle/2 trough-dwell cap (5000).
    CHECK(retxTroughDeferMs(0.0f, 0.40f, 0.0f, false, 2, 0) == 5000,
          "n=2 escalation is capped at half the fade cycle (Good: 5000 ms)");
    // Moderate fallback: 0.5 Hz -> Tc 846 ms, half-cycle 1000 ms (holds stay <= 1 s).
    CHECK(retxTroughDeferMs(0.0f, 0.85f, 0.0f, false, 1, 0) == 846,
          "Moderate fallback n=1 defers Tc = 846 ms");
    CHECK(retxTroughDeferMs(0.0f, 0.85f, 0.0f, false, 2, 0) == 1000,
          "Moderate n=2 capped at half cycle (1000 ms)");
    // Poor fallback: 1.0 Hz -> Tc 423 ms — fast channels barely defer (their troughs pass).
    CHECK(retxTroughDeferMs(0.0f, 1.50f, 0.0f, false, 1, 0) == 423,
          "Poor fallback n=1 defers Tc = 423 ms");
    // Measured Doppler (valid coherence feed) takes priority over the class fallback.
    CHECK(retxTroughDeferMs(0.2f, 0.85f, 0.0f, true, 1, 0) == 2115,
          "measured f_D=0.2 Hz overrides the Moderate fallback (Tc 2115 ms)");
    // Coherence-ADJUSTED fallback: a Moderate fading index with a confident-Good coherence
    // verdict re-classes to Good (0.1 Hz) — same refinement the rate ladder uses.
    CHECK(retxTroughDeferMs(0.0f, 0.85f, 0.60f, true, 1, 0) == 4230,
          "confident-Good coherence re-classes a Moderate fading index to Good Tc");
    // frac knob scales Tc (0.5 * 4230 = 2115).
    CHECK(retxTroughDeferMs(0.0f, 0.40f, 0.0f, false, 1, 0, 0.5f) == 2115,
          "frac=0.5 halves the deferral");
    // Absolute engineering clamp: estimator garbage (f_D=0.01 -> Tc 42.3 s, half-cycle
    // 50 s) can never hold longer than ~one burst-time.
    CHECK(retxTroughDeferMs(0.01f, 0.40f, 0.0f, true, 1, 0) == kRetxTroughDeferAbsCapMs,
          "hold is absolutely capped at 8000 ms regardless of estimator garbage");
    // Degenerate inputs are safe no-ops.
    CHECK(retxTroughDeferMs(0.0f, 0.40f, 0.0f, false, 0, 0) == 4230,
          "zero_rounds is floored at 1 (a round just failed when this is called)");
}

// ═══ #58 increment 3 — ConnectSnrPool (BUG-CONNECT-SNR-VARIANCE) ═══
// Pure pool math (param-driven; the ULTRA_CONNECT_SNR_POOL / _PICK_DEFER /
// ULTRA_WIRE_SNR_FRESH knobs gate only the Connection wiring and are pinned OFF
// in main() for latch determinism).

void test_connect_snr_pool_population_and_identity() {
    ConnectSnrPool pool;
    // Population contract enforced in addReading: the training snapshot (MCDPSK
    // non-data-aided) has a DIFFERENT calibration basis and must never enter; idle/
    // sync-quality sources never enter either.
    CHECK(!pool.addReading(12.0f, SNRSource::MCDPSK_IN_BAND, false),
          "training (non-data-aided) MCDPSK reading never enters the pool");
    CHECK(!pool.addReading(12.0f, SNRSource::IDLE_IN_BAND, false),
          "idle reading never enters the pool");
    CHECK(!pool.addReading(12.0f, SNRSource::SYNC_QUALITY, false),
          "sync-quality proxy never enters the pool");
    CHECK(!pool.addReading(NAN, SNRSource::MCDPSK_IN_BAND, true),
          "non-finite reading never enters the pool");
    CHECK(pool.empty(), "rejected readings leave the pool empty");
    CHECK(std::isnan(pool.clusteredDbMeanDb(4230, true, UINT64_MAX)),
          "empty pool aggregates to NAN (callers fall back to the scalar)");
    CHECK(pool.effectiveCount(4230, true, UINT64_MAX) == 0, "empty pool N_eff=0");

    // N=1 identity: a single reading's aggregate IS the reading — with the knob ON
    // the pick is byte-identical to the scalar path by construction.
    CHECK(pool.addReading(11.5f, SNRSource::MCDPSK_IN_BAND, true),
          "data-aided MCDPSK reading enters the pool");
    CHECK(std::fabs(pool.clusteredDbMeanDb(4230, true, UINT64_MAX) - 11.5f) < 1e-4f,
          "N=1 identity: aggregate == the single reading");
    CHECK(pool.effectiveCount(4230, true, UINT64_MAX) == 1, "single reading N_eff=1");

    // Ring capacity: oldest drops, size caps at 8.
    ConnectSnrPool ring;
    for (int i = 0; i < 10; ++i) {
        ring.addReading(static_cast<float>(i), SNRSource::MCDPSK_IN_BAND, true);
    }
    CHECK(ring.size() == ConnectSnrPool::kCapacity, "ring caps at 8 readings");
    // All same-age (no ticks) -> one cluster; mean of the SURVIVING readings 2..9.
    CHECK(std::fabs(ring.clusteredDbMeanDb(4230, true, UINT64_MAX) - 5.5f) < 1e-4f,
          "ring keeps the newest 8 (oldest two dropped)");
}

void test_connect_snr_pool_tc_clustering() {
    // Two readings < Tc apart are ONE fade sample (the channel has not decorrelated):
    // they merge into one cluster (dB-average), N_eff stays 1.
    ConnectSnrPool pool;
    const uint64_t tc_ms = 4230;  // Good-class design Tc (0.423/0.1 Hz)
    pool.addReading(11.0f, SNRSource::MCDPSK_IN_BAND, true);
    pool.tick(1000);  // 1 s < Tc
    pool.addReading(13.0f, SNRSource::MCDPSK_IN_BAND, true);
    CHECK(pool.effectiveCount(tc_ms, true, UINT64_MAX) == 1,
          "readings < Tc apart cluster to ONE effective sample");
    CHECK(std::fabs(pool.clusteredDbMeanDb(tc_ms, true, UINT64_MAX) - 12.0f) < 1e-4f,
          "clustered pair contributes its dB-average");

    // A third reading > Tc later is a genuinely decorrelated second sample; the
    // final statistic is the dB-mean of CLUSTER means (12 and 6), not of raw readings.
    pool.tick(9000);  // 9 s > 2 Tc since the second reading
    pool.addReading(6.0f, SNRSource::MCDPSK_IN_BAND, true);
    CHECK(pool.effectiveCount(tc_ms, true, UINT64_MAX) == 2,
          "a > Tc-separated reading is a second effective sample");
    CHECK(std::fabs(pool.clusteredDbMeanDb(tc_ms, true, UINT64_MAX) - 9.0f) < 1e-4f,
          "aggregate = dB-mean of cluster means ((12 + 6)/2), not of raw readings");

    // AWGN-class Tc (UINT32_MAX from coherenceTimeMsForDoppler on fD<=0): everything
    // clusters to one stationary sample whose dB-average is the right estimate.
    CHECK(pool.effectiveCount(UINT32_MAX, true, UINT64_MAX) == 1,
          "infinite Tc (AWGN class) clusters everything to one sample");
}

void test_connect_snr_pool_trough_suppression() {
    // Moderate-class trough suppression: a lone 3.9 dB trough reading buys a ~90 bps
    // DBPSK session on a channel carrying ~2 kbps; the pool rescues it to OFDM entry.
    // (The original W3 counterfactual read fading 0.73 on rig MPG@20 and called it
    // Moderate — that mis-class was BUG-MPG20-OVER-DEMOTE-R14; 0.73 now correctly
    // classes Good, so this test uses an unambiguously-Moderate index (0.85) to keep
    // exercising the Moderate-class saturation-rescue path the mechanism owns.)
    const float sel_single = connectSelectionSnrDb(3.9f, 0.85f, true);
    CHECK(sel_single < kOFDMEntryFloorModerateDb,
          "single trough reading (3.9, below the saturation zone) stays sub-OFDM");
    CHECK(selectLadderRung(sel_single, 0.85f).waveform == WaveformMode::MC_DPSK,
          "as-shipped: one trough reading lands MC-DPSK");

    // Pool replay: {3.9, then a decorrelated healthy 12.8} -> dB-mean 8.35. The +5
    // fade basis composes ONCE downstream, and the >=6.5 data-aided Moderate
    // saturation rescue now clears the Moderate floor -> OFDM entry.
    ConnectSnrPool pool;
    const uint64_t tc_ms = 846;  // Moderate-class design Tc (0.423/0.5 Hz)
    pool.addReading(3.9f, SNRSource::MCDPSK_IN_BAND, true);
    pool.tick(10000);  // a CONNECT retry arrives >= ~10 s later (> 2 Tc even on Good)
    pool.addReading(12.8f, SNRSource::MCDPSK_IN_BAND, true);
    const float mean = pool.clusteredDbMeanDb(tc_ms, true, UINT64_MAX);
    CHECK(std::fabs(mean - 8.35f) < 1e-3f,
          "dB-mean of decorrelated trough+healthy readings = 8.35");
    const float sel_pool = connectSelectionSnrDb(mean, 0.85f, true);
    CHECK(sel_pool >= kOFDMEntryFloorModerateDb,
          "pool mean + basis + saturation rescue clears the Moderate entry floor");
    CHECK(selectLadderRung(sel_pool, 0.85f).waveform == WaveformMode::OFDM_CHIRP,
          "trough suppression: OFDM entry instead of the 90 bps DBPSK session");

    // Composition guard: the aggregate is basis-free (a raw-population statistic);
    // the +5 basis and the Moderate saturation bound compose exactly ONCE downstream:
    // sel = max(mean + basis, kOFDMEntryFloorModerateDb + 0.5).
    const float expected_sel = std::max(mean + connectSnrFadeBasisDb(),
                                        kOFDMEntryFloorModerateDb + 0.5f);
    CHECK(std::fabs(sel_pool - expected_sel) < 1e-4f,
          "basis + saturation bound apply exactly once, downstream of the aggregation");
}

void test_connect_snr_pool_wire_freshness() {
    ConnectSnrPool pool;
    const uint64_t tc_ms = 4230;
    const uint64_t fresh_ms = tc_ms * kConnectWireSnrFreshTcMultiple;

    // OFDM_BROADBAND readings enter TAGGED and serve only the wire aggregation
    // (handshake_only=false); the entry pick filters them out.
    CHECK(pool.addReading(16.5f, SNRSource::OFDM_BROADBAND, false),
          "broadband control-frame reading enters the pool (wire population)");
    CHECK(std::isnan(pool.clusteredDbMeanDb(tc_ms, true, UINT64_MAX)),
          "entry-pick aggregation excludes OFDM_BROADBAND readings");
    CHECK(std::isfinite(pool.clusteredDbMeanDb(tc_ms, false, fresh_ms)),
          "wire aggregation sees the fresh broadband reading");

    // Aging past 3*Tc: the pool no longer describes the CURRENT link -> no wire value
    // -> the caller embeds the -10 sentinel, which encodes to wire byte 0 (the
    // receiver's existing 'peer SNR n/a' rendering; no receiver change).
    pool.tick(fresh_ms + 1);
    CHECK(std::isnan(pool.clusteredDbMeanDb(tc_ms, false, fresh_ms)),
          "readings older than 3*Tc age out of the wire aggregate");
    CHECK(v2::encodeSNR(kConnectSnrStaleSentinelDb) == 0,
          "stale sentinel -10 dB encodes to wire byte 0");
    CHECK(v2::decodeSNR(0) < 0.0f,
          "wire byte 0 decodes below the receiver's >=0 validity gate (renders n/a)");

    // A fresh reading revives the wire value.
    pool.addReading(19.5f, SNRSource::OFDM_BROADBAND, false);
    CHECK(std::fabs(pool.clusteredDbMeanDb(tc_ms, false, fresh_ms) - 19.5f) < 1e-4f,
          "a fresh decode restores an honest wire value");
}

void test_connect_fading_pool_aggregate() {
    // #58 increment 4 (BUG-CONNECT-FADING-VARIANCE): the fading index rides each
    // pooled reading and aggregates over the SAME Tc clusters as the SNR mean.
    const uint64_t tc_ms = 4230;  // Good-class design Tc

    // Single-reading identity: with one fading-carrying reading the aggregate IS
    // that reading — knob-ON with N=1 is byte-identical to the scalar path.
    ConnectSnrPool pool;
    pool.addReading(11.5f, SNRSource::MCDPSK_IN_BAND, true, 0.66f);
    CHECK(std::fabs(pool.clusteredFadingIndex(tc_ms, true, UINT64_MAX) - 0.66f) < 1e-4f,
          "N=1 identity: fading aggregate == the single reading");

    // Absent fading (setMeasuredSNR-only feed): the reading still counts for the
    // SNR aggregate but contributes nothing to (and does not poison) the fading
    // aggregate; an all-absent pool aggregates fading to NAN (scalar fallback).
    ConnectSnrPool absent;
    absent.addReading(12.0f, SNRSource::MCDPSK_IN_BAND, true);
    CHECK(std::isfinite(absent.clusteredDbMeanDb(tc_ms, true, UINT64_MAX)),
          "fading-less reading still feeds the SNR aggregate");
    CHECK(std::isnan(absent.clusteredFadingIndex(tc_ms, true, UINT64_MAX)),
          "all-absent fading aggregates to NAN (callers fall back to the scalar)");
    absent.addReading(12.0f, SNRSource::MCDPSK_IN_BAND, true, 0.50f);
    CHECK(std::fabs(absent.clusteredFadingIndex(tc_ms, true, UINT64_MAX) - 0.50f) < 1e-4f,
          "NAN-fading readings are skipped, not averaged as zero");

    // Cluster semantics match the SNR aggregate: readings < Tc apart are ONE fade
    // sample (their fading values average within the cluster); a > Tc-separated
    // reading is a second effective sample and the statistic is the MEAN of
    // cluster means (bounded [0,2] statistic — no heavy tail for a median to
    // guard against, and at N_eff=2 the median IS the mean).
    pool.tick(1000);  // 1 s < Tc: same fade sample
    pool.addReading(12.5f, SNRSource::MCDPSK_IN_BAND, true, 0.74f);
    CHECK(std::fabs(pool.clusteredFadingIndex(tc_ms, true, UINT64_MAX) - 0.70f) < 1e-4f,
          "readings < Tc apart cluster: fading is the within-cluster mean");
    pool.tick(9000);  // > 2 Tc: decorrelated second sample
    pool.addReading(13.0f, SNRSource::MCDPSK_IN_BAND, true, 0.30f);
    CHECK(std::fabs(pool.clusteredFadingIndex(tc_ms, true, UINT64_MAX) - 0.50f) < 1e-4f,
          "aggregate = mean of cluster means ((0.70 + 0.30)/2), not of raw readings");

    // The screenshot bug (rig dial-20 Watterson Good): a SINGLE CONNECT frame read
    // fading 0.66 -> classified Moderate under the OLD 0.65 boundary -> QPSK R1/4
    // entry on a true-Good channel (BUG-MPG20-OVER-DEMOTE-R14). PRIMARY fix: the
    // boundary now sits at the ML midpoint (kFadingGoodMax = 0.76), so 0.66 classes
    // Good on the single frame — no pool needed.
    CHECK(classifyChannel(0.66f) == ChannelClassification::GOOD,
          "single 0.66 frame now classes Good directly (0.76 boundary)");
    // The pool remains the SNR-variance decorrelator and averages residual fading
    // spread: two decorrelated Good readings still aggregate comfortably Good.
    ConnectSnrPool bug;
    bug.addReading(12.0f, SNRSource::MCDPSK_IN_BAND, true, 0.66f);
    bug.tick(10000);  // CONNECT retry > 2 Tc later
    bug.addReading(14.0f, SNRSource::MCDPSK_IN_BAND, true, 0.44f);
    const float pooled_fading = bug.clusteredFadingIndex(tc_ms, true, UINT64_MAX);
    CHECK(std::fabs(pooled_fading - 0.55f) < 1e-4f,
          "pooled fading = (0.66 + 0.44)/2 = 0.55");
    CHECK(classifyChannel(pooled_fading) == ChannelClassification::GOOD,
          "pooled fading stays in the Good class");
}

void test_connect_pick_defer_semantics() {
    // Defer exactly once, only for N_eff==1, only on fading, only for a sub-OFDM pick.
    CHECK(shouldDeferConnectPick(1, 0.50f, WaveformMode::MC_DPSK, false),
          "N_eff=1 + fading + sub-OFDM pick defers");
    CHECK(!shouldDeferConnectPick(1, 0.50f, WaveformMode::MC_DPSK, true),
          "defer fires ONCE per handshake (already_deferred blocks)");
    CHECK(!shouldDeferConnectPick(2, 0.50f, WaveformMode::MC_DPSK, false),
          "N_eff=2: the aggregate already rests on decorrelated samples — pick");
    CHECK(!shouldDeferConnectPick(0, 0.50f, WaveformMode::MC_DPSK, false),
          "N_eff=0: scalar-fallback path (no qualifying reading) — nothing to defer for");
    CHECK(!shouldDeferConnectPick(1, kFadingAwgnMax - 0.01f, WaveformMode::MC_DPSK, false),
          "AWGN class never defers (one sample IS the stationary mean)");
    CHECK(!shouldDeferConnectPick(1, 0.50f, WaveformMode::OFDM_CHIRP, false),
          "an OFDM pick never defers (only the 15-40x sub-OFDM zone buys a sample)");
}

}  // namespace

// ULTRA_OFDM_ANCHOR_OFFSET_DB: the marking offset must be overridable for the rig
// over-commit measurement, must DEFAULT to the calibrated constant (byte-identical),
// and must REJECT nonsense so a typo cannot silently un-calibrate the rate ladder.
void test_anchor_offset_override() {
    unsetenv("ULTRA_OFDM_ANCHOR_OFFSET_DB");
    CHECK(ofdmAnchorScaleOffsetDb() == kOfdmLegacyAnchorScaleOffsetDb,
          "unset must return the calibrated constant (byte-identical default)");
    setenv("ULTRA_OFDM_ANCHOR_OFFSET_DB", "3.1", 1);
    CHECK(std::fabs(ofdmAnchorScaleOffsetDb() - 3.1f) < 1e-4f,
          "a valid override must be honoured");
    setenv("ULTRA_OFDM_ANCHOR_OFFSET_DB", "0", 1);
    CHECK(ofdmAnchorScaleOffsetDb() == 0.0f, "zero is a legitimate override (honest scale)");
    setenv("ULTRA_OFDM_ANCHOR_OFFSET_DB", "999", 1);
    CHECK(ofdmAnchorScaleOffsetDb() == kOfdmLegacyAnchorScaleOffsetDb,
          "out-of-range must fall back to the constant, never un-calibrate the ladder");
    setenv("ULTRA_OFDM_ANCHOR_OFFSET_DB", "garbage", 1);
    CHECK(ofdmAnchorScaleOffsetDb() == kOfdmLegacyAnchorScaleOffsetDb,
          "unparseable must fall back to the constant");
    unsetenv("ULTRA_OFDM_ANCHOR_OFFSET_DB");
}

// BUG-STALE-SNR-SENTINEL-DRIVES-DECISIONS. kConnectSnrStaleSentinelDb (-10.0) means
// "no measurement" on the wire, where it is correct (encodes to byte 0 -> peer renders
// n/a). It is ALSO a finite float that every numeric consumer accepts silently, and the
// IONOS rig caught it driving three consecutive demotes (R2/3 -> R1/2 -> R1/4) on a link
// that was transferring a file. This screen is what keeps it out of decisions.
void test_stale_snr_sentinel_is_screened_from_decisions() {
    CHECK(isStaleSnrSentinel(kConnectSnrStaleSentinelDb),
          "the sentinel itself must be recognised");
    CHECK(isStaleSnrSentinel(std::numeric_limits<float>::quiet_NaN()),
          "a non-finite reading is equally 'no measurement'");
    CHECK(isStaleSnrSentinel(-10.4f) && isStaleSnrSentinel(-9.6f),
          "the 0.5 dB tolerance must match the GUI's n/a render, so display and "
          "decision agree on what counts as no-measurement");

    // The whole point: real readings must survive the screen untouched. A floor-adjacent
    // but genuine value must NOT be discarded, or the screen becomes its own bug.
    CHECK(!isStaleSnrSentinel(-9.0f),
          "-9.0 is outside the tolerance band and must be treated as a measurement");
    CHECK(!isStaleSnrSentinel(0.0f) && !isStaleSnrSentinel(5.0f) &&
              !isStaleSnrSentinel(20.0f),
          "ordinary operating SNRs must never be screened out");

    // Cost asymmetry that justifies the direction: mistaking a real -10 dB channel for
    // "unknown" costs nothing (no rung is usable there and every floor is >= 5 dB), while
    // mistaking "unknown" for -10 dB collapses a healthy link to the most conservative
    // geometry. The screen must therefore err toward "unknown" at the floor.
    CHECK(isStaleSnrSentinel(kConnectSnrStaleSentinelDb + 0.25f),
          "at the encodable floor the screen must err toward 'no measurement'");

    // THE SEAM. Connection::decisionSnrDb() is a one-line delegation to this, so testing
    // the selector here covers the wiring without depending on ULTRA_WIRE_SNR_FRESH —
    // which the test binaries pin process-wide via a function-local static latch.
    CHECK(std::fabs(decisionSnrFromWire(kConnectSnrStaleSentinelDb, 14.5f) - 14.5f) < 1e-4f,
          "a stale wire value must fall back to the last real measurement");
    CHECK(std::fabs(decisionSnrFromWire(std::numeric_limits<float>::quiet_NaN(), 9.0f) - 9.0f)
              < 1e-4f,
          "a non-finite wire value must fall back too");
    CHECK(std::fabs(decisionSnrFromWire(17.25f, 3.0f) - 17.25f) < 1e-4f,
          "a FRESH wire value must be used as-is — the fallback must not shadow real data");
    // Degenerate but reachable: if there is no real measurement either, the selector must
    // pass the fallback through unchanged rather than inventing a number. The caller keeps
    // whatever validity contract measured_snr_db_ carries; this must not paper over it.
    CHECK(decisionSnrFromWire(kConnectSnrStaleSentinelDb, kConnectSnrStaleSentinelDb) ==
              kConnectSnrStaleSentinelDb,
          "with no real measurement the selector must not fabricate one");
}

// ULTRA_LINEAR_SNR_RING: the domain fix and its compensating offset reduction are ONE knob.
// Splitting them is the documented footgun -- fixing the ring domain alone RAISES the ladder
// input ~1.7 dB on a path already measured to over-commit ~2 rungs.
void test_linear_ring_couples_offset_compensation() {
    unsetenv("ULTRA_LINEAR_SNR_RING");
    unsetenv("ULTRA_OFDM_ANCHOR_OFFSET_DB");
    CHECK(!linearSnrRingEnabled(), "linear ring must default OFF (byte-identical)");
    CHECK(ofdmAnchorScaleOffsetDb() == kOfdmLegacyAnchorScaleOffsetDb,
          "knob off: offset must be the untouched legacy constant");

    setenv("ULTRA_LINEAR_SNR_RING", "1", 1);
    CHECK(linearSnrRingEnabled(), "knob on must enable the linear ring");
    // THE COUPLING: enabling the ring must ALSO pull the offset down by the measured
    // Jensen compensation, with no second knob required.
    CHECK(std::fabs(ofdmAnchorScaleOffsetDb() -
                    (kOfdmLegacyAnchorScaleOffsetDb - kLinearRingJensenCompensationDb)) < 1e-4f,
          "knob on must carry its own offset compensation (the two are inseparable)");
    CHECK(ofdmAnchorScaleOffsetDb() < kOfdmLegacyAnchorScaleOffsetDb,
          "compensation must REDUCE the offset, never raise it");

    // An explicit override still wins, so the offset stays sweepable for the anchor re-measure.
    setenv("ULTRA_OFDM_ANCHOR_OFFSET_DB", "4.0", 1);
    CHECK(std::fabs(ofdmAnchorScaleOffsetDb() - 4.0f) < 1e-4f,
          "an explicit offset override must still win over the coupled default");
    unsetenv("ULTRA_OFDM_ANCHOR_OFFSET_DB");
    unsetenv("ULTRA_LINEAR_SNR_RING");
}


// ── ULTRA_INFLIGHT_RTO: the timeout must be a FUNCTION of frames outstanding ──
// Operator observation 2026-07-30: "would have been a lot faster if it didn't hit a bunch
// of timeout and retx at the end". The ARQ timeout is computed once per mode change with
// arq_.getWindowSize(), so at the tail of a transfer — ONE frame outstanding, ~1.27 s of
// airtime — the sender still waits the full-window value. One rig transfer spent 57 s
// delivering the last ~1.4 KB of a 50 KB file with CCA reading idle=1 through the gap.
//
// This pins the properties the in-flight table depends on. It is a DIFFERENT defect from
// the RTO clamp (ULTRA_ADAPTIVE_RTO): that one only stopped the configured value flooring
// the RFC6298 estimate, and it does not help here because srtt is averaged over all round
// trips — mostly full bursts — so the estimator never learns the tail is cheap.
void test_inflight_ack_timeout_scales_with_frames() {
    const auto mod = Modulation::QAM8;
    const auto rate = CodeRate::R2_3;
    const uint32_t reanchor = connection_policy::wideOFDMShortReanchorChirpDurationMs();
    const int cw = 12;

    auto timeout_for = [&](size_t n) {
        const uint32_t sack_hold = connection_policy::wideOFDMSackDelayMs(
            mod, rate, n, cw, reanchor);
        return connection_policy::computeWideOFDMAckTimeoutMs(
            mod, rate, n, sack_hold, 1, cw, reanchor);
    };

    // MONOTONE: more frames in flight can never mean a shorter wait. This is what makes
    // a table indexed by outstanding count safe to consult at any point in a burst.
    uint32_t prev = 0;
    for (size_t n = 1; n <= selective_repeat_arq_policy::kMaxWindow; ++n) {
        const uint32_t t = timeout_for(n);
        CHECK(t >= prev, "ack timeout must be monotone in frames outstanding (n=" << n << ")");
        prev = t;
    }

    // The tail must be MATERIALLY cheaper than a full window, or the change buys nothing.
    const uint32_t t1 = timeout_for(1);
    const uint32_t t8 = timeout_for(8);
    const uint32_t t16 = timeout_for(selective_repeat_arq_policy::kMaxWindow);
    CHECK(t1 * 2 < t8, "a 1-frame tail must cost less than half an 8-frame window's wait");
    CHECK(t1 * 4 < t16, "and far less than a full 16-frame window's");

    // IDENTITY: evaluated at the window size, the table entry the ARQ would use must equal
    // what the legacy scalar path computes — the knob is a reduction for SMALL bursts, not
    // a re-tuning of the configured bound.
    for (size_t win : {size_t{4}, size_t{8}, size_t{16}}) {
        const uint32_t sack_hold = connection_policy::wideOFDMSackDelayMs(
            mod, rate, win, cw, reanchor);
        const uint32_t scalar = connection_policy::computeWideOFDMAckTimeoutMs(
            mod, rate, win, sack_hold, 1, cw, reanchor);
        CHECK(timeout_for(win) == scalar,
              "table[window] must reproduce the legacy scalar exactly (win=" << win << ")");
    }
}

void test_silent_ack_repeat_broad_signal_hold_is_waveform_independent() {
    constexpr int64_t now_ms = 20000;
    constexpr int64_t fresh_signal_ms = 10000;
    constexpr int64_t arm_baseline_ms = 9999;

    CHECK(shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::MC_DPSK, now_ms, fresh_signal_ms, arm_baseline_ms),
          "MC-DPSK must retain the broad-signal in-progress hold");
    CHECK(shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::OFDM_CHIRP, now_ms, fresh_signal_ms, arm_baseline_ms),
          "wide OFDM RX evidence must hold an opt-in silent ACK repeat");
    CHECK(shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::OFDM_NARROW, now_ms, fresh_signal_ms, arm_baseline_ms),
          "narrow OFDM RX evidence must hold an opt-in silent ACK repeat");
    CHECK(!shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::OFDM_CHIRP, 12000, 10000, 10000),
          "the just-decoded group at the arm baseline must not hold its own repeat");
    CHECK(shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::OFDM_CHIRP, 12000, 10001, 10000),
          "a single new post-arm broad signal stamp must retain the full safety hold");
    CHECK(!shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::OFDM_CHIRP, 12000, 9999, 10000),
          "an older pre-arm signal stamp must not hold the repeat");
    CHECK(!shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::MC_DPSK, now_ms, 6000, 5000),
          "an MC-DPSK signal stamp outside the hold window must expire");
    CHECK(!shouldHoldSilentAckRepeatForBroadSignal(
              WaveformMode::MC_DPSK, now_ms, now_ms + 1, arm_baseline_ms),
          "a future signal timestamp must not hold the repeat");
}

void test_tone_ack_defer_uses_independent_rx_evidence() {
    CHECK(!shouldDeferToneBurstAck(true, false, 0, false, false),
          "a genuinely quiet receiver should transmit its ACK immediately");
    CHECK(shouldDeferToneBurstAck(true, true, 0, false, false),
          "energy CCA busy must defer the ACK");
    CHECK(shouldDeferToneBurstAck(true, false, 1, false, false),
          "declared burst airtime must defer the ACK even when CCA fades idle");
    CHECK(shouldDeferToneBurstAck(true, false, 0, true, false),
          "decoder RX evidence must cover asynchronous descriptor-loss ACKs");
    CHECK(!shouldDeferToneBurstAck(true, false, 0, true, true),
          "fresh evidence must not defer the causally-safe group-boundary ACK");
    CHECK(shouldDeferToneBurstAck(false, true, 0, false, false) == false,
          "CCA opt-out should disable energy-only deferral");
    CHECK(shouldDeferToneBurstAck(false, true, 0, true, false),
          "CCA opt-out must not disable the decoder-evidence safety gate");
    CHECK(deferredToneAckRxHoldMs(false) == kDescriptorLostReverseTxHoldMs,
          "a pending asynchronous ACK must retain the full descriptor-loss guard");
    CHECK(deferredToneAckRxHoldMs(true) == 0,
          "a physically-complete group ACK must not inherit its own fresh RX stamp");
}

void test_silent_ack_repeat_is_explicit_opt_in() {
    CHECK(silentAckRepeatDelayMs(nullptr) == 0,
          "unset silent-repeat knob must be safety-default OFF");
    CHECK(silentAckRepeatDelayMs("") == 0,
          "empty silent-repeat knob must stay OFF");
    CHECK(silentAckRepeatDelayMs("0") == 0,
          "zero must remain an explicit opt-out");
    CHECK(silentAckRepeatDelayMs("300") == 300,
          "minimum supported explicit delay should enable the repeat");
    CHECK(silentAckRepeatDelayMs("4000") == 4000,
          "an explicit legacy 4 s delay should remain available for A/B");
    CHECK(silentAckRepeatDelayMs("5000") == 5000,
          "maximum supported explicit delay should enable the repeat");
    CHECK(silentAckRepeatDelayMs("299") == 0 &&
              silentAckRepeatDelayMs("5001") == 0,
          "out-of-range repeat delays must fail closed");
    CHECK(silentAckRepeatDelayMs("4000oops") == 0,
          "malformed repeat delays must fail closed");
}

void test_anchored_backstop_payload_evidence_forbids_fake_crater() {
    CHECK(shouldGradeAnchoredBackstopAsCrater(/*payload_seen=*/false),
          "an anchored timeout with no decoded payload may retain zero-progress recovery");
    CHECK(!shouldGradeAnchoredBackstopAsCrater(/*payload_seen=*/true),
          "decoded fallback payload must never be reported to the selector as k=0/M=5");
}

void test_scripted_disconnect_quit_requires_protocol_completion() {
    using ultra::protocol::connection_policy::scriptedDisconnectQuitReady;

    CHECK(!scriptedDisconnectQuitReady(false, 8000),
          "the former fixed 8-second deadline must not quit while DISCONNECTING");
    CHECK(!scriptedDisconnectQuitReady(false, 30000),
          "the protocol timeout duration must not substitute for an observed state transition");
    CHECK(!scriptedDisconnectQuitReady(false, 100000),
          "even a very old request must not be labelled complete without state evidence");
    CHECK(!scriptedDisconnectQuitReady(true, 1999),
          "an observed disconnect retains its short diagnostic log grace");
    CHECK(scriptedDisconnectQuitReady(true, 2000),
          "an observed disconnect may quit exactly at the log-grace boundary");
}

int main() {
    // The coherent-window A/B knob (ULTRA_COHERENT_WINDOW) is latched ONCE via a
    // function-local static on the first policy call — pin it to the disabled
    // baseline BEFORE any test runs so this binary deterministically tests the
    // default (byte-identical) window-selection path.
    setenv("ULTRA_COHERENT_WINDOW", "0", 1);
    // Same latch-once pattern: pin the 16QAM R3/4 crest-rung A/B knob to its disabled
    // default so this binary deterministically tests the byte-identical baseline cap.
    setenv("ULTRA_QAM16_R34", "0", 1);
    // #58 increment 3: pin the connect-SNR-pool knobs to their disabled defaults —
    // the pool tests below drive ConnectSnrPool/shouldDeferConnectPick directly
    // (param-driven, env-free); the knobs gate only the Connection wiring.
    setenv("ULTRA_CONNECT_SNR_POOL", "0", 1);
    setenv("ULTRA_CONNECT_PICK_DEFER", "0", 1);
    setenv("ULTRA_WIRE_SNR_FRESH", "0", 1);
    // Same latch-once pattern: pin the data-aided R3/4 entry-cap A/B knob to its
    // disabled default (boundary tests live in test_waveform_policy).
    setenv("ULTRA_ENTRY_CAP_R34", "0", 1);
    // Calibrated affine entry-SNR basis: pin the knob OFF — the affine tests drive
    // the pure 4-arg connectSelectionSnrDb/dialEquivalentSnrDb overloads directly;
    // the pinned-OFF env lets the wrapper-identity check be deterministic.
    setenv("ULTRA_CONNECT_AFFINE_BASIS", "0", 1);
    setenv("ULTRA_MAX_BURST_AIRTIME_MS", "8600", 1);
    setenv("ULTRA_BURST_ESCALATION", "1", 1);
    unsetenv("ULTRA_BURST_ESC_STREAK");

    test_fading_labels_and_capabilities();
    test_ladder_rung_selection();
    test_wide_ofdm_timing_and_timeout();
    test_narrow_ofdm_timing_and_timeout();
    test_mc_dpsk_window_timing();
    test_ofdm_profile_selection();
    test_negotiated_mode_selection();
    test_auto_data_mode_boundaries();
    test_recommend_cw_count();
    test_candidate_specific_burst_geometry();
    test_variable_frame_payload_capacity();
    test_warm_short_anchor_descriptor_gate();
    test_unified_burst_ack_timeout();
    test_connect_selection_saturation_bound();
    test_connect_affine_basis();
    test_coherent_window_override_disabled_keeps_default();
    test_qam16_r34_cap_knob_off_keeps_r23();
    test_retx_trough_defer();
    test_connect_snr_pool_population_and_identity();
    test_connect_snr_pool_tc_clustering();
    test_connect_snr_pool_trough_suppression();
    test_connect_snr_pool_wire_freshness();
    test_connect_fading_pool_aggregate();
    test_connect_pick_defer_semantics();
    test_anchor_offset_override();
    test_stale_snr_sentinel_is_screened_from_decisions();
    test_linear_ring_couples_offset_compensation();
    test_inflight_ack_timeout_scales_with_frames();
    test_silent_ack_repeat_broad_signal_hold_is_waveform_independent();
    test_tone_ack_defer_uses_independent_rx_evidence();
    test_silent_ack_repeat_is_explicit_opt_in();
    test_anchored_backstop_payload_evidence_forbids_fake_crater();
    test_scripted_disconnect_quit_requires_protocol_completion();

    if (tests_failed != 0) {
        std::cout << "ConnectionPolicy: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "ConnectionPolicy: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
