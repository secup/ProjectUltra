# Effective-SNR rate selection: measurement and design (2026-11-08)

`src/protocol/evm_rate_selector.hpp`, knob `ULTRA_RATE_EVM=1` (opt-in, receiver = rate authority;
set it on both stations). Supersedes the per-channel-class theta idea filed as
BUG-LATENT-CLASS-SPACING: the selector needs no channel classifier.

## 1. The measurement

Every burst data frame already carries the demodulator's decision-directed, gain-corrected EVM
SNR (`demodulator_impl.hpp`, "Radio-agnostic decision-directed EVM SNR"), pooled over the
frame's data cells. Pairing it with the frame's decode verdict:

**GUI gate frames** (two `ultra_gui -sim` over OTASim, 7.3 Hz dial offset; every
`gui_qso_scenario.sh` output on disk from this session, 7131 frames, `bravo.log` "EVM SNR
estimate" lines paired in order with the group's "Burst logical frame i/N: OK|FAIL" lines; the
descriptor's EVM, the first of each group, dropped):

| rung | channel | frames | EVM 4-6 | 6-8 | 8-10 | 10-12 | 12-14 | >= 14 | 50 % point |
|---|---|---|---|---|---|---|---|---|---|
| QPSK R1/2 | Good | 538 | 0 % | 100 % | 100 % | 100 % | 100 % | 100 % | 5.85 dB |
| QPSK R2/3 | Good | 3781 | 2 % | 77 % | 100 % | 100 % | 100 % | 100 % | 6.5 |
| QPSK R2/3 | Moderate | 635 | 5 % | 90 % | 100 % | 100 % | 100 % | 100 % | 6.3 |
| 8PSK R2/3 | Good | 703 | — | 0 % | 30 % | 99 % | 100 % | 100 % | 9.55 |
| 8PSK R2/3 | Moderate | 322 | — | 0 % | 33 % | 96 % | 100 % | 100 % | 9.45 |
| 16QAM R2/3 | Good | 211 | — | — | — | 5 % | 96 % | 100 % | 12.1 |

The waterfall in the EVM domain is a 1-1.5 dB cliff and **does not depend on the channel**:
Good and Moderate agree to 0.1-0.3 dB. In the channel-SNR domain the same rungs differ by 2-4 dB
between Good and Moderate, with 17-31 % error floors on Moderate
(`LATENT_ANCHOR_MEASUREMENT_2026_11_08.md`). The difference is the effective-SNR link abstraction:
fading depth, Doppler-induced channel-estimation error and implementation loss are already inside
the EVM, so what remains is the code's own waterfall. Moderate's "floor" is the part of its EVM
distribution that sits below the cliff, not a different decoder behaviour.

**Harness** (`tools/measure_ack_fer --frame-log PATH`, new: one `evm_db,has_evm,pass` line per
frame; production geometry, 7.3 Hz, ITU Good, 3 SNR points around each waterfall x 3 seeds x 24
frames) for every rung, 50 % points: QPSK R1/4 4.8, R1/2 5.5, R2/3 6.5, R3/4 7.1, 8PSK R2/3 9.6,
R3/4 10.5, 16QAM R1/2 11.4, R2/3 12.55, R3/4 13.9 dB. The harness's single-frame (cold-acquired)
Moderate runs fail to demodulate 30-40 % of frames and show shallower slopes; the GUI's warm burst
path does not, so the table uses the Good fits, cross-checked against the GUI on both channels.

**Cross-rung use.** The EVM a frame reads also depends on the rung it was sent on: each rung's
pilot / codeword geometry sets its own channel-estimation ceiling. AWGN fits of
`EVM = -10 log10(10^(-(SNR-a)/10) + 10^(-C/10))` give a = 2.5 dB for every rung and C = 21.7 dB
(QPSK R1/2..R3/4, 8PSK), 24.7 dB (16QAM), 17.7 dB (QPSK R1/4): on a strong channel 16QAM frames
read up to ~1.3 dB higher than QPSK frames (GUI AWGN@20: 17.1 / 17.45 / 18.35 dB for QPSK R1/2 /
8PSK / 16QAM). Below ~15 dB, where every threshold sits, the rungs agree to 0.1-0.3 dB, so a frame
measured on one rung predicts another's waterfall without a per-rung correction.

**Turbo** (`ULTRA_DATA_FEC=turbo`): GUI frames give 8PSK R2/3 9.05 dB (-0.44) and 16QAM R2/3 11.60
(-0.52); the turbo table shifts every rung by its measured AWGN FER-10 % gain. The classic-path
harness emits no decode result (so no EVM) for a failed turbo frame and cannot fit these
(BUG-HARNESS-TURBO-NO-RESULT).

## 2. The selector

For every rung r, the counterfactual success over the last 20 measured frames (~4 groups, ~35 s):

    P_r = mean_f logistic(3 /dB * (e_f - eta_r - m))

— the fraction of what was just received that rung r would have decoded. The value is the same
delivered-bytes-per-cycle pricing as the latent controller (production geometry, incumbent light,
alternatives with the switch anchor in chirp mode, no anchor in structure mode), with Minstrel's
dead (P < 0.10) and cap (P <= 0.90) clamps and the operator ceiling. Decides once the window holds
4 measured frames (the first group); until then, and whenever the decoder publishes no EVM, the
latent decision stands. The one-shot startup probe is not used: it exists only because an
outcome-only estimator cannot see a clean rung's margin.

**Outer loop.** m is one common-mode margin, moved after every fully-measured group by
2 dB x (predicted - delivered) / M on that group's own frames, clamped to [-3, +6] dB. A fade does
not move it (the faded frames' EVMs predicted the losses); a mismatch between this radio's EVM and
the thresholds does. This is the outer-loop margin the reference open-source HF modem uses with its
per-frame measured SNR, there driven by every lost frame; here only by unpredicted ones.

## 3. Why not the alternatives tried first

- **Class-aware thetas** (Good / Moderate tables keyed by a channel classifier): the classifier
  that separates them (`frequency_selectivity.hpp`) is rig-validated, but the channel-SNR domain
  still mixes rung-independent fade losses into each rung's curve; the EVM domain separates them.
- **Renewal pricing** (price every candidate by its long-run rate under the protocol's own
  re-anchor rule, the switch anchor once): GUI gate Good@20 -3 %, Good@15 -5 %, Moderate@20 -7 %
  against the shipped pricing, reverted. It moved the controller off a lossy 8PSK R2/3 onto QPSK
  R2/3, which then lost ~20 % of frames in the same fades: the channel-SNR model predicts a lower
  rung far more robust than it is inside a fade. The incumbent bias of the shipped pricing was
  hedging that. With EVM counterfactuals a faded frame is predicted to fail on every rung.

## 4. GUI gate

`tools/gui_qso_scenario.sh`, 21 KB, `--cfo-hz 7.3`, seeds 42-46, mean goodput (bps), all runs PASS
unless stated:

| arm | knobs | Good@20 | Good@15 | Moderate@20 | AWGN@20 |
|---|---|---|---|---|---|
| A default | — | 1700 | 1526 | 1668 | 1720 |
| B | `ULTRA_LATENT_ANCHORS=measured` | 2154 | 1648 | 1572 | 2660 |
| C | B + `ULTRA_BURST_ACQ=structure` | 2868 | 2138 | 1870 | 3220 |
| D | C + `ULTRA_DATA_FEC=turbo` | 2782 | 2448 | 2304 | 3218 |
| E | C + `ULTRA_RATE_EVM=1` (long-run objective) | 2446 | — | 1788 | 3617 |
| E2 | C + `ULTRA_RATE_EVM=1` (time-to-finish) | 2548 | — | (1620-2130, 3 seeds) | — |
| E3 | C + `ULTRA_RATE_EVM=climb` | 2952 | 2150 | 1918 | 3620 |
| **F3** | **D + `ULTRA_RATE_EVM=climb` = `ULTRA_FAST=1`** | **2880** | **2556** | **2114** | **3618** |
| G | `ULTRA_LATENT_ANCHORS=measured ULTRA_DATA_FEC=turbo` (chirp mode) | 2302 | 1914 | 1680 | 2660 |

F3 against A: Good@20 +69 %, Good@15 +67 %, Moderate@20 +27 %, AWGN@20 +110 %, 20/20 PASS. The
climb (E3 vs C, F3 vs D) is worth +12 % on AWGN@20 (it takes 16QAM R2/3 from the first group) and
is within seed noise elsewhere (Good@20 +3 / +3.5 %, Good@15 +0.6 / +4 %, Moderate@20 +2.6 / -8 %;
per-seed spread ~300 bps).

**Broad matrix, `ULTRA_FAST=1` vs default** (same gate, seeds 42-46; screen locked / display
handling see CHANGELOG 2026-11-09 BUG-GUI-DISPLAY-SLEEP-STALL):

| cell | default | `ULTRA_FAST=1` | |
|---|---|---|---|
| Good@12 | 1002 (1 timeout: stuck on QPSK R2/3) | 1724, 5/5 PASS | +72 % |
| Good@25 | 1710 | 3142 | +84 % |
| Moderate@15 | 1226 | 1656 | +35 % |
| Moderate@25 | 1694 | 2490 | +47 % |
| AWGN@12 | 1670 | 2930 | +75 % |
| AWGN@8 | 702 (2/5 FAIL) | 936 (3/5 FAIL) | pre-existing |

Every AWGN@8 failure in both arms is BUG-MCDPSK-AWGN8-BIMODAL-HANDSHAKE: the connect reading falls
7.8 or 8.0 dB on the 8 dB OFDM entry floor; at 7.8 the link takes MC-DPSK DQPSK R1/4, whose file
transfer then hard-fails at ~344 s. The profile changes neither the connect decision nor MC-DPSK;
when the link comes up in OFDM it doubles the default (2320-2360 vs 1160-1180 bps).

C includes the structure-mode switch-pricing fix (2026-11-08): structure acquisition never
anchors a descriptor-committed switch, so the selector no longer prices one; with it C reaches
16QAM R2/3 on Good@20 and AWGN@20 (the chirp-mode measured set never left 8PSK R2/3).

Caveats: simulator only (stationary Watterson, RMS-normalized bursts, no real PA/ALC); the seeded
gate shifts each run's timeline with every decision, so per-seed pairs are not paired samples —
compare means and spreads. The rig decides.
