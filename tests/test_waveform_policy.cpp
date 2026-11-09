#include "env_compat.hpp"
#include "protocol/waveform_selection.hpp"
#include "waveform/ofdm_chirp_waveform.hpp"
#include "waveform/waveform_factory.hpp"
#include "ultra/types.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace ultra;
using namespace ultra::protocol;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
        } \
    } while (0)

void test_ofdm_rate_thresholds() {
    // COHERENT-ONLY LADDER (kCoherentLadder). Auto rate selection is one ladder
    // walked high->low; QAM16 is DISABLED on auto (kRungDisabledDb). Enabled rungs
    // per class:
    //   AWGN     : R1/4 (>=8 floor), R1/2 (>=10), R2/3 (>=12), R3/4 (>=15). QAM16 off.
    //   GOOD (0.15..0.76): R1/2 (>=10), R2/3 (>=15), R3/4 (>=20). R1/4 below 10. QAM16 disabled.
    //   MODERATE (0.76..1.10): R1/4 (>=14), R1/2 (>=18), R2/3 (>=20). R3/4 / QAM16 disabled.
    //   POOR (>=1.10): R1/4 always (defensive; Poor routes to MC-DPSK upstream).
    // AWGN R2/3/R3/4 enabled 2026-06-06 (measure_ack_fer data4_full floors R2/3~8,
    // R3/4~12 dB + margin); AWGN is flat so it sits BELOW the Good thresholds.

    // AWGN high SNR: top enabled AWGN rung is QAM8 R2/3 (QAM8 R3/4 auto-disabled
    // 2026-07-06 — AWGN-only anchor, fading-poisonous; selectOFDMCodeRate returns
    // the picked rung's RATE, so these read R2/3 with mod QAM8).
    CHECK(selectOFDMCodeRate(25.0f, 0.00f) == CodeRate::R2_3,
          "AWGN in-band SNR25 -> QAM8 R2/3 rung (rate=R2/3)");
    CHECK(selectOFDMCodeRate(40.0f, 0.00f) == CodeRate::R2_3,
          "AWGN very high SNR -> QAM8 R2/3 rung (R3/4 auto-disabled)");
    CHECK(selectOFDMCodeRate(25.0f, 0.12f) == CodeRate::R2_3,
          "near-AWGN (fading 0.12, AWGN class) SNR25 -> QAM8 R2/3 rung");

    // AWGN staircase: R1/4 (<10), R1/2 [10,12), R2/3 [12,15), R3/4 [15,inf).
    CHECK(selectOFDMCodeRate(9.9f, 0.00f) == CodeRate::R1_4,
          "AWGN in-band SNR=9.9 should stay R1/4 (just under R1/2 gate)");
    CHECK(selectOFDMCodeRate(10.0f, 0.00f) == CodeRate::R1_2,
          "AWGN in-band SNR=10 should promote to R1/2");
    CHECK(selectOFDMCodeRate(11.9f, 0.00f) == CodeRate::R1_2,
          "AWGN in-band SNR=11.9 should stay R1/2 (just under R2/3 gate)");
    CHECK(selectOFDMCodeRate(12.0f, 0.00f) == CodeRate::R2_3,
          "AWGN in-band SNR=12 should promote to R2/3 (measure_ack_fer floor ~8 + margin)");
    CHECK(selectOFDMCodeRate(14.9f, 0.00f) == CodeRate::R2_3,
          "AWGN in-band SNR=14.9 should stay R2/3 (just under R3/4 gate)");
    CHECK(selectOFDMCodeRate(15.0f, 0.00f) == CodeRate::R3_4,
          "AWGN in-band SNR=15 should promote to R3/4 (measure_ack_fer floor ~12 + margin)");

    // Good fading gate at SNR >= 10 dB; R2/3 enabled at >= 15 dB.
    CHECK(selectOFDMCodeRate(9.9f, 0.30f) == CodeRate::R1_4,
          "good fading in-band SNR=9.9 should stay R1/4 (just under R1/2 gate)");
    CHECK(selectOFDMCodeRate(10.0f, 0.30f) == CodeRate::R1_2,
          "good fading in-band SNR=10 should promote to R1/2 (measured reliable @ Good 10)");
    CHECK(selectOFDMCodeRate(14.9f, 0.30f) == CodeRate::R1_2,
          "good fading in-band SNR=14.9 should stay R1/2 (just under R2/3 gate)");
    CHECK(selectOFDMCodeRate(15.0f, 0.30f) == CodeRate::R2_3,
          "good fading in-band SNR=15 should promote to QPSK R2/3 (measured 2026-06-02)");
    CHECK(selectOFDMCodeRate(19.9f, 0.30f) == CodeRate::R2_3,
          "good fading in-band SNR=19.9 stays R2/3 (just under the R3/4 gate)");
    CHECK(selectOFDMCodeRate(20.0f, 0.30f) == CodeRate::R2_3,
          "good fading in-band SNR=20 -> 16QAM R2/3 rung (DEFAULT ladder = psk8-exp since 2026-07-05)");

    // Moderate fading: R1/2 gate at SNR >= 18 dB; R2/3 enabled at >= 20 dB (measured 2026-06-09,
    // genuine moderate R2/3 9/9 PASS @20-24 dB, qso_sweep) — softens the Good/Moderate cliff so a
    // misclassification costs 1 rung (R3/4->R2/3) not 2 (R3/4->R1/2). R3/4 stays disabled on moderate.
    CHECK(selectOFDMCodeRate(17.9f, 0.90f) == CodeRate::R1_4,
          "moderate fading in-band SNR=17.9 should stay R1/4 (just under R1/2 gate)");
    CHECK(selectOFDMCodeRate(18.0f, 0.90f) == CodeRate::R1_2,
          "moderate fading in-band SNR=18 should promote to R1/2");
    CHECK(selectOFDMCodeRate(19.9f, 0.90f) == CodeRate::R1_2,
          "moderate fading in-band SNR=19.9 should stay R1/2 (just under the R2/3 gate)");
    CHECK(selectOFDMCodeRate(20.0f, 0.90f) == CodeRate::R2_3,
          "moderate fading in-band SNR=20 should promote to R2/3 (measured 2026-06-09)");
    CHECK(selectOFDMCodeRate(25.0f, 0.90f) == CodeRate::R2_3,
          "moderate fading in-band SNR=25 should top out at R2/3 (R3/4 still disabled on moderate)");

    // Heavy+ fading (>= 1.10) — R1/4 only at all SNRs.
    CHECK(selectOFDMCodeRate(25.0f, 1.20f) == CodeRate::R1_4,
          "heavy fading should fall back to R1/4");
    CHECK(selectOFDMCodeRate(40.0f, 1.20f) == CodeRate::R1_4,
          "heavy fading at very high SNR should still stay R1/4");
}

void test_narrow_data_mode() {
    // OFDM_NARROW: DQPSK only. R1/2 promoted in clean-enough conditions
    // (relaxed 2026-05-03 after cli_simulator sweeps); R1/4 elsewhere.
    Modulation mod;
    CodeRate rate;

    // Hard floor: in-band SNR=18 good fading must stay at R1/4 (CLAUDE.md baseline).
    recommendDataMode(18.0f, WaveformMode::OFDM_NARROW, mod, rate, 0.40f);
    CHECK(mod == Modulation::DQPSK, "narrow forces DQPSK");
    CHECK(rate == CodeRate::R1_4, "narrow in-band SNR=18 good fading must stay R1/4");

    // AWGN path: in-band SNR>=18 + fading<0.15 -> R1/2
    recommendDataMode(18.0f, WaveformMode::OFDM_NARROW, mod, rate, 0.05f);
    CHECK(rate == CodeRate::R1_2, "narrow in-band SNR=18 AWGN should allow R1/2");

    // Good-fading path: in-band SNR>=20 + fading<kFadingGoodMax -> R1/2
    recommendDataMode(20.0f, WaveformMode::OFDM_NARROW, mod, rate, 0.40f);
    CHECK(rate == CodeRate::R1_2, "narrow in-band SNR=20 good fading should allow R1/2");

    recommendDataMode(25.0f, WaveformMode::OFDM_NARROW, mod, rate, 0.40f);
    CHECK(rate == CodeRate::R1_2, "narrow in-band SNR=25 good fading should allow R1/2");

    // Just under SNR threshold should drop to R1/4
    recommendDataMode(19.9f, WaveformMode::OFDM_NARROW, mod, rate, 0.40f);
    CHECK(rate == CodeRate::R1_4, "narrow in-band SNR=19.9 good fading should drop to R1/4");

    // Moderate fading should always stay R1/4
    recommendDataMode(25.0f, WaveformMode::OFDM_NARROW, mod, rate, 0.90f);
    CHECK(rate == CodeRate::R1_4, "narrow moderate fading should stay R1/4");
}

void test_coherent_ladder_selection() {
    // COHERENT-ONLY LADDER (kCoherentLadder, 2026-06-02). The OFDM band auto-selects
    // ALWAYS coherent QPSK now; QAM16 and the QPSK R3/4 rung are DISABLED on auto
    // (kRungDisabledDb) — they are reachable only via ULTRA_FORCE_* (not set here).
    // recommendDataMode walks the ladder high->low and returns the first rung the
    // (snr, fading_class) clears. mod is ALWAYS QPSK for OFDM_CHIRP.
    Modulation mod;
    CodeRate rate;

    // --- AWGN class (fading < 0.15): R1/4 (>=8), R1/2 (>=10), R2/3 (>=12), R3/4 (>=15);
    //     QAM16 still OFF (2026-06-06 AWGN R2/3/R3/4 enable). ---
    // DEFAULT ladder = kCoherentLadderPsk8Exp (2026-07-05 campaign flip): the QAM8
    // rungs are auto-selectable (AWGN R3/4 @18, R2/3 @16); 16QAM stays Good-only.
    recommendDataMode(20.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.05f);
    CHECK(mod == Modulation::QAM8, "AWGN in-band SNR=20 selects 8PSK (QAM8 R2/3 rung)");
    CHECK(rate == CodeRate::R2_3, "AWGN in-band SNR=20 -> QAM8 R2/3 (R3/4 auto-disabled 2026-07-06: AWGN-only anchor poisoned fading runs)");

    recommendDataMode(25.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.05f);
    CHECK(mod == Modulation::QAM8 && rate == CodeRate::R2_3,
          "AWGN in-band SNR=25 -> QAM8 R2/3 (R3/4 auto-disabled; 16QAM Good-only)");

    recommendDataMode(16.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.05f);
    CHECK(mod == Modulation::QAM8 && rate == CodeRate::R2_3,
          "AWGN in-band SNR=16 -> QAM8 R2/3 (>=16 anchor)");

    recommendDataMode(13.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.05f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R2_3,
          "AWGN in-band SNR=13 -> QPSK R2/3 (>=12, <15)");

    recommendDataMode(9.9f, WaveformMode::OFDM_CHIRP, mod, rate, 0.05f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_4,
          "AWGN in-band SNR=9.9 stays QPSK R1/4 (just under the R1/2 gate)");

    recommendDataMode(10.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.05f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_2,
          "AWGN in-band SNR=10 promotes to coherent QPSK R1/2");

    // --- GOOD class (0.15 <= fading < 0.76): R1/2 (>=10), R2/3 (>=15), R3/4 (>=20) ---
    // 2026-07-26 ANCHORS RE-MEASURED ON FADING (docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md).
    // Good@20 used to select 16QAM R2/3 on a G20 anchor. That anchor measured **51.4% FER** on
    // ITU Good and 16QAM R2/3 NEVER overtakes 8PSK R2/3 in 16-24 dB (delivered @20: 1890 vs
    // 2450). Its Good anchor moved 20 -> 26, so Good@20 now correctly selects 8PSK R2/3.
    recommendDataMode(20.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QAM8, "good fading in-band SNR=20 selects 8PSK (measured best rung on fading)");
    CHECK(rate == CodeRate::R2_3,
          "good fading in-band SNR=20 -> 8PSK R2/3 (16QAM R2/3 measured 51.4% FER at this SNR)");

    recommendDataMode(15.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R2_3,
          "good fading in-band SNR=15 promotes to QPSK R2/3 (measured 2026-06-02)");

    recommendDataMode(14.9f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_2,
          "good fading in-band SNR=14.9 stays QPSK R1/2 (just under the R2/3 gate)");

    recommendDataMode(14.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_2,
          "good fading in-band SNR=14 promotes to QPSK R1/2");

    // 8PSK R2/3's Good anchor moved 19 -> 17: the MEASURED throughput crossover where it
    // overtakes QPSK R3/4 on fading (delivered @16 1686 vs 1499, @18 1914 vs 2106). The anchor
    // belongs at the crossover, not the FER<=10% floor -- a denser rung carrying more retx
    // still wins while rate x (1-FER) is higher.
    recommendDataMode(17.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QAM8 && rate == CodeRate::R2_3,
          "good fading in-band SNR=17 selects 8PSK R2/3 (measured crossover vs QPSK R3/4)");

    recommendDataMode(9.9f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_4,
          "good fading in-band SNR=9.9 stays QPSK R1/4 (just under the R1/2 gate)");

    // good/snr10: R1/2 reliable @ Good 10 (measured) — the OFDM band runs QPSK R1/2 there.
    recommendDataMode(10.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.40f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_2,
          "good/snr10: OFDM band promotes to coherent QPSK R1/2 (measured reliable @ Good 10)");

    // --- MODERATE class (0.76 <= fading < 1.10): R1/4 (>=14), R1/2 (>=18), R2/3 (>=20) ---
    recommendDataMode(20.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.90f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R2_3,
          "moderate fading in-band SNR=20 selects coherent QPSK R2/3 (enabled 2026-06-09, softens cliff)");

    recommendDataMode(14.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.90f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_4,
          "moderate fading in-band SNR=14 selects coherent QPSK R1/4 at the entry floor");

    // --- POOR class (fading >= 1.10): always defensive QPSK R1/4 ---
    // Poor routes to MC-DPSK at the SELECTION layer (kOFDMEntryFloorPoorDb unreachable;
    // verified in the ConnectionPolicy test). recommendDataMode given OFDM+Poor still
    // returns coherent QPSK R1/4 — a defensive don't-happen case, never differential.
    recommendDataMode(25.0f, WaveformMode::OFDM_CHIRP, mod, rate, 1.20f);
    CHECK(mod == Modulation::QPSK && rate == CodeRate::R1_4,
          "OFDM+Poor (a don't-happen: Poor routes to MC-DPSK) is coherent QPSK R1/4");
}

void test_bootstrap_caps() {
    // Bootstrap cap = min(candidate, ladder_rate_for(snr,fading)) by code-rate value.
    // FADING channels then also pin to R1/2 (the adaptive loop climbs from there once
    // ACK quality arrives — the chirp SNR can be optimistic on a fading channel). FLAT
    // AWGN (fading < kFadingAwgnMax) has NO R1/2 pin: the measured SNR is stable and the
    // per-rung floors are reliable, so it starts at the ladder rate (2026-06-06). The
    // 4-arg overload returns candidate only when ULTRA_FORCE_DATA_RATE is set (unset here).

    // AWGN: starts at the ladder rate (no R1/2 pin) — ladder gives R3/4 at >=15.
    // AWGN >=16 ladder rung is now QAM8 R2/3 (QAM8 R3/4 auto-disabled 2026-07-06)
    // -> ladder RATE is R2/3 and the cap min()s down to it.
    CHECK(capInitialOFDMRate(19.6f, 0.04f, CodeRate::R3_4) == CodeRate::R2_3,
          "AWGN@19.6 initial caps to the ladder rung rate (QAM8 R2/3)");
    CHECK(capInitialOFDMRate(15.0f, 0.04f, CodeRate::R3_4) == CodeRate::R3_4,
          "AWGN@15 initial R3/4 starts at R3/4 (ladder enables R3/4 at 15)");
    // Candidate below ladder still wins (min): R2/3 candidate on AWGN@20 -> R2/3.
    CHECK(capInitialOFDMRate(20.0f, 0.05f, CodeRate::R2_3) == CodeRate::R2_3,
          "AWGN initial R2/3 starts at R2/3 (min(candidate, ladder R3/4))");
    CHECK(capInitialOFDMRate(30.0f, 0.12f, CodeRate::R2_3) == CodeRate::R2_3,
          "near-AWGN (fading 0.12, AWGN class) initial R2/3 starts at R2/3");
    // Just below the AWGN R2/3 gate (12): ladder is R1/2, so the cap follows it.
    CHECK(capInitialOFDMRate(11.0f, 0.04f, CodeRate::R3_4) == CodeRate::R1_2,
          "AWGN@11 caps R3/4 down to the ladder rate R1/2");
    // FADING now starts at the LADDER rate too (no R1/2 pin): Good@20 ladder is R3/4,
    // candidate R2/3 -> min = R2/3 (start at the measured Good rung, not R1/2).
    CHECK(capInitialOFDMRate(20.0f, 0.40f, CodeRate::R2_3, Modulation::QPSK) ==
              CodeRate::R2_3,
          "good fading initial QPSK R2/3 starts at R2/3 (ladder rate, no R1/2 pin)");
    CHECK(capInitialOFDMRate(15.0f, 0.40f, CodeRate::R3_4, Modulation::QPSK) ==
              CodeRate::R2_3,
          "good fading @15 caps R3/4 down to the ladder rate R2/3 (Good@15 measured)");
    CHECK(capInitialOFDMRate(20.0f, 0.40f, CodeRate::R2_3, Modulation::DQPSK) ==
              CodeRate::R2_3,
          "good fading: modulation no longer changes the bootstrap cap");
    // Below the AWGN R1/2 gate (10) the cap follows the ladder floor (R1/4).
    CHECK(capInitialOFDMRate(9.5f, 0.05f, CodeRate::R3_4) == CodeRate::R1_4,
          "AWGN below the R1/2 gate caps to the QPSK R1/4 ladder floor");
}

void test_rx_authority_initial_rung_cap() {
    auto p = capRxAuthorityInitialRung(Modulation::QAM8, CodeRate::R2_3, true);
    CHECK(p.mod == Modulation::QPSK && p.rate == CodeRate::R1_2,
          "RX authority caps a dense automatic entry to the measured R1/2 probe");

    p = capRxAuthorityInitialRung(Modulation::QPSK, CodeRate::R3_4, true);
    CHECK(p.mod == Modulation::QPSK && p.rate == CodeRate::R1_2,
          "RX authority caps the live-failing R3/4 bootstrap to R1/2");

    p = capRxAuthorityInitialRung(Modulation::QPSK, CodeRate::R2_3, true);
    CHECK(p.mod == Modulation::QPSK && p.rate == CodeRate::R1_2,
          "RX authority caps R2/3 until coherent evidence exists");

    p = capRxAuthorityInitialRung(Modulation::QPSK, CodeRate::R1_2, true);
    CHECK(p.mod == Modulation::QPSK && p.rate == CodeRate::R1_2,
          "R1/2 entry remains unchanged");

    p = capRxAuthorityInitialRung(Modulation::QPSK, CodeRate::R1_4, true);
    CHECK(p.mod == Modulation::QPSK && p.rate == CodeRate::R1_4,
          "a more robust R1/4 entry is never raised");

    p = capRxAuthorityInitialRung(Modulation::QAM8, CodeRate::R2_3, false);
    CHECK(p.mod == Modulation::QAM8 && p.rate == CodeRate::R2_3,
          "authority-disabled policy leaves the selected entry untouched");

    p = capRxAuthorityInitialRung(Modulation::QAM8, CodeRate::R2_3, true,
                                  /*operator_forced=*/true);
    CHECK(p.mod == Modulation::QAM8 && p.rate == CodeRate::R2_3,
          "resolved operator force bypasses the automatic physical-entry cap");
}

void test_environment_force_validity() {
    setenv("ULTRA_FORCE_DATA_MOD", "8PSK", 1);
    setenv("ULTRA_FORCE_DATA_RATE", "R2_3", 1);
    CHECK(forcedDataModulationFromEnvironment() == Modulation::QAM8 &&
              forcedDataCodeRateFromEnvironment() == CodeRate::R2_3 &&
              environmentForcesDataProfile(),
          "valid 8PSK R2/3 env profile is an operator force");
    CHECK(capInitialOFDMRate(11.0f, 0.04f, CodeRate::R3_4,
                             Modulation::QPSK) == CodeRate::R3_4,
          "valid forced data rate bypasses the legacy bootstrap rate cap");

    unsetenv("ULTRA_FORCE_DATA_RATE");
    Modulation partial_mod = Modulation::AUTO;
    CodeRate partial_rate = CodeRate::AUTO;
    recommendDataMode(20.0f, WaveformMode::OFDM_CHIRP,
                      partial_mod, partial_rate, 0.50f);
    CHECK(partial_mod == Modulation::QAM8 && partial_rate == CodeRate::R2_3,
          "modulation-only force resolves the automatic rate instead of leaving it unset");

    setenv("ULTRA_FORCE_DATA_MOD", "8PSK", 1);
    setenv("ULTRA_FORCE_DATA_RATE", "R2_3_typo", 1);
    const EnvironmentForcedDataProfile malformed =
        forcedDataProfileFromEnvironment();
    CHECK(malformed.malformed && malformed.modulation_present &&
              malformed.code_rate_present &&
              forcedDataModulationFromEnvironment() == Modulation::AUTO &&
              forcedDataCodeRateFromEnvironment() == CodeRate::AUTO &&
              !environmentForcesDataProfile(),
          "mixed valid+invalid env profile fails atomically, never partially");
    CHECK(capInitialOFDMRate(11.0f, 0.04f, CodeRate::R3_4,
                             Modulation::QPSK) == CodeRate::R1_2,
          "invalid forced-rate text does not bypass automatic bootstrap cap");

    unsetenv("ULTRA_FORCE_DATA_MOD");
    unsetenv("ULTRA_FORCE_DATA_RATE");
}

void test_entry_cap_r34() {
    // ULTRA_ENTRY_CAP_R34 (default OFF, pinned "0" in main BEFORE any policy call —
    // the knob is read ONCE via static lambda). Knob-OFF behavior must be byte-identical
    // to the pre-knob R2/3 basis clamp even when the reading is data-aided and high;
    // the knob-ON path is exercised through capInitialOFDMRateImpl's explicit seam.
    CHECK(!entryCapR34Enabled(), "ULTRA_ENTRY_CAP_R34=0 must read as disabled");

    // Knob OFF: a data-aided high reading still gets the R2/3 basis clamp (unchanged).
    CHECK(capInitialOFDMRate(25.0f, 0.40f, CodeRate::R3_4, Modulation::QPSK,
                             /*data_aided=*/true) == CodeRate::R2_3,
          "knob OFF: data-aided Good@25 entry stays clamped to R2/3 (byte-identical)");

    // Gate threshold derivation: Good-class QPSK R3/4 ladder anchor (20.0) + the
    // measured per-reading sigma (3.15) = 23.15. Just above clears, just below holds.
    CHECK(coherentLadderAnchorDb(Modulation::QPSK, CodeRate::R3_4, 0.40f) == 20.0f,
          "Good-class QPSK R3/4 anchor lookup matches the ladder (20.0)");
    CHECK(dataAidedEntryClearsR34(true, 23.2f, 0.40f, true),
          "gate: data-aided Good@23.2 clears anchor(20)+sigma(3.15)");
    CHECK(!dataAidedEntryClearsR34(true, 23.1f, 0.40f, true),
          "gate: data-aided Good@23.1 is below anchor+sigma -> holds");
    CHECK(!dataAidedEntryClearsR34(true, 25.0f, 0.40f, false),
          "gate: training (non-data-aided) reading never clears");
    CHECK(!dataAidedEntryClearsR34(true, 25.0f, 0.90f, true),
          "gate: Moderate-class fading never clears (unconditional R2/3 cap)");
    CHECK(!dataAidedEntryClearsR34(false, 25.0f, 0.40f, true),
          "gate: knob off never clears");

    // Knob ON (via the impl seam): data-aided high reading enters at R3/4.
    CHECK(capInitialOFDMRateImpl(25.0f, 0.40f, CodeRate::R3_4, Modulation::QPSK,
                                 /*data_aided=*/true, /*entry_cap_r34_on=*/true) ==
              CodeRate::R3_4,
          "knob ON: data-aided Good@25 enters at R3/4");
    // Training-routed reading keeps the R2/3 cap even when high.
    CHECK(capInitialOFDMRateImpl(25.0f, 0.40f, CodeRate::R3_4, Modulation::QPSK,
                                 /*data_aided=*/false, /*entry_cap_r34_on=*/true) ==
              CodeRate::R2_3,
          "knob ON: training reading keeps the R2/3 cap");
    // Moderate-class fading keeps the R2/3 cap unconditionally.
    CHECK(capInitialOFDMRateImpl(25.0f, 0.90f, CodeRate::R3_4, Modulation::QPSK,
                                 /*data_aided=*/true, /*entry_cap_r34_on=*/true) ==
              CodeRate::R2_3,
          "knob ON: Moderate-class keeps the R2/3 cap");
    // Marginal reading (below anchor+sigma) keeps the R2/3 cap.
    CHECK(capInitialOFDMRateImpl(22.0f, 0.40f, CodeRate::R3_4, Modulation::QPSK,
                                 /*data_aided=*/true, /*entry_cap_r34_on=*/true) ==
              CodeRate::R2_3,
          "knob ON: marginal Good@22 (< 20+3.15) keeps the R2/3 cap");
    // The cap never RAISES the candidate: an R2/3 candidate stays R2/3 through the gate.
    CHECK(capInitialOFDMRateImpl(25.0f, 0.40f, CodeRate::R2_3, Modulation::QPSK,
                                 /*data_aided=*/true, /*entry_cap_r34_on=*/true) ==
              CodeRate::R2_3,
          "knob ON: R2/3 candidate is never raised (min(candidate, R3/4))");
    // AWGN never hits the basis clamp; the knob changes nothing there.
    CHECK(capInitialOFDMRateImpl(20.0f, 0.05f, CodeRate::R3_4, Modulation::QPSK,
                                 /*data_aided=*/true, /*entry_cap_r34_on=*/true) ==
              CodeRate::R2_3,
          "knob ON: AWGN path follows the ladder rung rate (QAM8 R2/3 since 2026-07-06)");
}

void test_waveform_recommendations() {
    // AWGN OFDM entry floor = ofdmEntryFloorAwgnDb(): 6 dB under ULTRA_FAST (default since
    // 2026-11-09), 8 dB with ULTRA_FAST=0 (R1/4 clean @ AWGN 8, measured 2026-06-02).
    const float awgn_ofdm_floor = ofdmEntryFloorAwgnDb();
    auto low = recommendWaveformAndRate(awgn_ofdm_floor - 0.1f, 0.00f);
    CHECK(low.waveform == WaveformMode::MC_DPSK, "AWGN below the OFDM entry floor should use MC-DPSK");
    CHECK(low.rate == CodeRate::R1_4, "MC-DPSK recommendation should be R1/4");

    auto awgn_floor = recommendWaveformAndRate(awgn_ofdm_floor, 0.00f);
    CHECK(awgn_floor.waveform == WaveformMode::OFDM_CHIRP, "AWGN at the OFDM entry floor should use OFDM_CHIRP");
    CHECK(awgn_floor.rate == CodeRate::R1_4, "AWGN at the OFDM entry floor should recommend R1/4");

    auto awgn = recommendWaveformAndRate(10.0f, 0.00f);
    CHECK(awgn.waveform == WaveformMode::OFDM_CHIRP, "AWGN in-band SNR10 should use OFDM_CHIRP");
    CHECK(awgn.rate == CodeRate::R1_2, "AWGN in-band SNR10 promotes to R1/2 (monotonicity, was R1/4)");

    // Good entry floor lowered to 8 (2026-07-07 measured sweep: QPSK R1/4
    // Good@8 FER 17% = ARQ-viable; entry lands on the R1/4 floor rung and the
    // predictive climb owns promotion).
    auto good_below_floor = recommendWaveformAndRate(7.9f, 0.30f);
    CHECK(good_below_floor.waveform == WaveformMode::MC_DPSK,
          "good fading below in-band SNR8 should keep MC-DPSK margin");
    auto good_at_floor8 = recommendWaveformAndRate(8.0f, 0.30f);
    CHECK(good_at_floor8.waveform == WaveformMode::OFDM_CHIRP,
          "good fading at the measured 8 dB floor enters OFDM");

    auto moderate_floor = recommendWaveformAndRate(14.0f, 0.90f);
    CHECK(moderate_floor.waveform == WaveformMode::OFDM_CHIRP,
          "moderate fading in-band SNR14 should use OFDM_CHIRP");
    CHECK(moderate_floor.rate == CodeRate::R1_4,
          "moderate fading in-band SNR14 should recommend R1/4");

    auto moderate = recommendWaveformAndRate(25.0f, 0.90f);
    CHECK(moderate.waveform == WaveformMode::OFDM_CHIRP,
          "moderate fading in-band SNR25 should still use OFDM_CHIRP");
    CHECK(moderate.rate == CodeRate::R2_3,
          "moderate fading in-band SNR25 should recommend R2/3 (R2/3 enabled @>=20, R3/4 off)");
}

void test_data_mode_policy() {
    Modulation mod = Modulation::AUTO;
    CodeRate rate = CodeRate::AUTO;

    // COHERENT-ONLY LADDER (2026-06-02): the OFDM band always auto-selects QPSK.
    // QAM16 / R3/4 / differential rungs are no longer auto-selected. High SNR no
    // longer climbs into QAM16; it tops out per-class at R1/2 (AWGN/Moderate) or
    // R2/3 (Good).

    // DEFAULT ladder = psk8-exp (2026-07-05): high-SNR AWGN tops out at QAM8 R3/4
    // (16QAM stays Good-only); high-SNR Good tops out at 16QAM R2/3 (G20 anchor).
    recommendDataMode(37.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.00f);
    CHECK(mod == Modulation::QAM8, "high-SNR AWGN selects QAM8 (top enabled AWGN rung)");
    CHECK(rate == CodeRate::R2_3, "high-SNR AWGN -> QAM8 R2/3 (R3/4 auto-disabled 2026-07-06)");

    recommendDataMode(32.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(mod == Modulation::QAM16, "good-fading in-band SNR32 selects 16QAM");
    CHECK(rate == CodeRate::R2_3, "good-fading in-band SNR32 -> 16QAM R2/3 (G20 anchor)");

    recommendDataMode(30.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(mod == Modulation::QAM16, "good-fading in-band SNR30 selects 16QAM");
    CHECK(rate == CodeRate::R2_3, "good-fading in-band SNR30 -> 16QAM R2/3 (G20 anchor)");

    recommendDataMode(28.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(mod == Modulation::QAM16, "good-fading in-band SNR28 selects 16QAM (default ladder)");
    CHECK(rate == CodeRate::R2_3, "good-fading in-band SNR28 -> 16QAM R2/3 (G20 anchor)");

    // AWGN in-band SNR=21.7 -> QAM8 R3/4 (A18 top AWGN rung; default ladder psk8-exp).
    recommendDataMode(21.7f, WaveformMode::OFDM_CHIRP, mod, rate, 0.04f);
    CHECK(mod == Modulation::QAM8, "AWGN in-band SNR=21.7 selects QAM8 (top AWGN rung)");
    CHECK(rate == CodeRate::R2_3, "AWGN in-band SNR=21.7 -> QAM8 R2/3 (R3/4 auto-disabled)");

    // SNR=19 GOOD fading -> QAM8 R2/3 (G19 anchor sits between QPSK R2/3 and 16QAM R2/3).
    recommendDataMode(19.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(mod == Modulation::QAM8, "in-band SNR=19 good fading selects QAM8 (G19 anchor)");
    CHECK(rate == CodeRate::R2_3, "in-band SNR=19 good fading -> QAM8 R2/3");

    // SNR=16.9 GOOD fading: above the R2/3 gate (>=15) -> coherent QPSK R2/3.
    recommendDataMode(16.9f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(mod == Modulation::QPSK, "in-band SNR=16.9 good fading selects coherent QPSK");
    CHECK(rate == CodeRate::R2_3, "in-band SNR=16.9 good fading is above the R2/3 gate");

    // SNR=14.5 GOOD fading: between R1/2 (>=14) and R2/3 (>=15) gates -> QPSK R1/2.
    recommendDataMode(14.5f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(mod == Modulation::QPSK, "in-band SNR=14.5 good fading selects coherent QPSK");
    CHECK(rate == CodeRate::R1_2, "in-band SNR=14.5 good fading is QPSK R1/2 (under the R2/3 gate)");

    // Just under the R1/2 Good gate at SNR=9.9 dB -> R1/4 floor.
    recommendDataMode(9.9f, WaveformMode::OFDM_CHIRP, mod, rate, 0.30f);
    CHECK(rate == CodeRate::R1_4, "in-band SNR=9.9 good fading should stay R1/4");

    // Moderate fading at high SNR: coherent QPSK, R2/3 enabled @>=20, R3/4 off -> R2/3.
    recommendDataMode(30.0f, WaveformMode::OFDM_CHIRP, mod, rate, 0.90f);
    CHECK(mod == Modulation::QPSK, "moderate fading high-SNR selects coherent QPSK (differential retired)");
    CHECK(rate == CodeRate::R2_3, "moderate fading coherent QPSK tops out at R2/3 (R3/4 disabled on moderate)");

    recommendDataMode(12.0f, WaveformMode::MC_DPSK, mod, rate, 0.90f);
    CHECK(mod == Modulation::DQPSK, "MC-DPSK should use DQPSK");
    CHECK(rate == CodeRate::R1_4, "MC-DPSK should use R1/4");
}

void test_ofdm_chirp_qam16_configuration() {
    OFDMChirpWaveform waveform;
    waveform.configure(Modulation::QAM16, CodeRate::R2_3);

    CHECK(waveform.getModulation() == Modulation::QAM16,
          "OFDM_CHIRP should keep QAM16 instead of falling back to DQPSK");
    CHECK(waveform.getCodeRate() == CodeRate::R2_3,
          "OFDM_CHIRP QAM16 configuration should keep requested rate");
    CHECK(waveform.getConfig().use_pilots,
          "OFDM_CHIRP QAM16 should use pilot-aided coherent demodulation");
    CHECK(waveform.getPilotSpacing() == 8,
          "OFDM_CHIRP QAM16 R2/3 uses sparse pilot spacing 8 (2026-06-14 R2/3 sp8 default)");
    CHECK(waveform.getCapabilities().requires_pilots,
          "OFDM_CHIRP capabilities should advertise pilots when QAM16 is active");
}

void test_waveform_factory() {
    for (auto mode : WaveformFactory::getAvailableModes()) {
        auto waveform = WaveformFactory::create(mode);
        CHECK(static_cast<bool>(waveform), "available waveform should be constructible");
        CHECK(WaveformFactory::isSupported(mode), "available waveform should be supported");
        CHECK(!WaveformFactory::getModeName(mode).empty(), "available waveform should have a name");
        CHECK(WaveformFactory::getMinSNR(mode) > -20.0f, "available waveform should have sane min SNR");
        CHECK(WaveformFactory::getMaxThroughput(mode) > 0.0f, "available waveform should have throughput");
    }

    CHECK(!WaveformFactory::isSupported(WaveformMode::MFSK),
          "reserved MFSK should not be advertised as supported");
    CHECK(!WaveformFactory::isSupported(WaveformMode::OTFS_EQ),
          "reserved OTFS_EQ should not be advertised as supported");
    CHECK(!WaveformFactory::isSupported(WaveformMode::OTFS_RAW),
          "reserved OTFS_RAW should not be advertised as supported");
    CHECK(WaveformFactory::recommendMCDPSKCarriers(-3.0f) == 5,
          "low-SNR MC-DPSK carrier recommendation");
    CHECK(WaveformFactory::recommendMCDPSKCarriers(25.0f) == 20,
          "high-SNR MC-DPSK carrier recommendation");
}

}  // namespace


// FER-FLOOR TABLE ORDERING INVARIANT (2026-07-30).
//
// Within a fading class, a rung with HIGHER spectral efficiency must require MORE SNR.
// The predictive climb (connection.cpp) scans highest-rung-first and takes the FIRST that
// passes, so an inversion makes a DENSER rung preferentially selected over a sparser one.
//
// Not hypothetical. ULTRA_RUNG_CLASS_ANCHOR read the GOOD column of the ladder-SELECTION
// table, whose Good-vs-AWGN gaps are non-uniform (QPSK R3/4 +5 dB, 8PSK R2/3 only +1 dB).
// That inverted the ordering and, on the IONOS rig at MPG@20, tripled 12-CW usage
// (15 -> 48 frames) and took craters from 1 to 10 within a single interleaved pair.
// kMeasuredFerFloor avoids that failure mode because every entry is the same KIND of
// quantity (the measured <=10% FER point) -- but only while it stays ordered, hence this.
void test_fer_floor_monotonic() {
    using namespace ultra::protocol;
    for (int cls = 0; cls < 3; ++cls) {
        const float fidx = (cls == 0) ? 0.05f : (cls == 1) ? 0.50f : 0.95f;
        float prev_eta = -1.0f;
        float prev_snr = -1.0e9f;
        for (uint8_t r = kRungIdxQpskR14; r < kRungIdxCount; ++r) {
            const CoherentPick p = coherentRungFromIndex(r);
            const float snr = rungFerFloorDb(p.mod, p.rate, fidx);
            if (snr >= kRungDisabledDb) continue;    // unmeasured cell: no constraint
            const float eta = rungSpectralEfficiency(r);
            if (prev_eta >= 0.0f && eta > prev_eta) {
                CHECK(snr >= prev_snr,
                      "FER-floor table is monotone in spectral efficiency per class");
            }
            if (eta > prev_eta) { prev_eta = eta; prev_snr = snr; }
        }
    }
}

int main() {
    // ULTRA_ENTRY_CAP_R34 is read ONCE (static lambda): remove any parent override
    // before the first policy call and prove that the documented unset default is OFF.
    // The knob-ON path is tested through capInitialOFDMRateImpl's explicit seam.
    unsetenv("ULTRA_ENTRY_CAP_R34");

    test_ofdm_rate_thresholds();
    test_narrow_data_mode();
    test_coherent_ladder_selection();
    test_bootstrap_caps();
    test_rx_authority_initial_rung_cap();
    test_environment_force_validity();
    test_entry_cap_r34();
    test_waveform_recommendations();
    test_data_mode_policy();
    test_ofdm_chirp_qam16_configuration();
    test_waveform_factory();
    test_fer_floor_monotonic();

    if (tests_failed != 0) {
        std::cout << "Waveform policy: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "Waveform policy: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
    
