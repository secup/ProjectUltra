# Latent rate-controller anchors, re-measured on the current PHY (2026-11-08)

Supersedes the rung FER numbers in `docs/FADING_ANCHOR_MEASUREMENT_2026_07_26.md` (that sweep
used the 4-CW / LONG-CP harness, LLRs clipped by a thermal-only noise model, and a CFO chain that
was only correct at 0 Hz). Data: `docs/data/latent_anchor_sweep_2026_11_08.csv`.

## Method

`tools/latent_anchor_sweep.py` (resumable) runs `measure_ack_fer --config data4_full
--geometry production --preroll-ms 300 --cfo-hz 7.3 --n 24` for 9 rungs x {ITU Good, ITU
Moderate, AWGN} x 9 SNR points (2 dB) x 6 seeds = 1458 runs, 144 frames per point. Production
geometry = QPSK cw8, 8PSK cw12, 16QAM cw16, MEDIUM CP; a frame passes only if every codeword
does. `tools/latent_anchor_fit.py fit|best|sim` produces every table below.

## 1. 50 %-FER points (theta) and slopes

| rung | Good θ (slope) | Moderate θ (slope, floor) | AWGN θ | 2026-07-26 table |
|---|---|---|---|---|
| QPSK R1/4 | 4.9 (0.39) | 6.6 (0.40, 12 %) | 3.0 | −2.0 derived |
| QPSK R1/2 | 7.3 (0.49) | 9.0 (0.43, 8 %) | 4.4 | 3.0 derived |
| QPSK R2/3 | 10.3 (0.42) | 12.2 (0.33, 6 %) | 6.3 | 8.0 derived |
| QPSK R3/4 | 12.3 (0.37) | 14.9 (0.36, 5 %) | 7.0 | 13.0 |
| 8PSK R2/3 | 14.7 (0.41) | 18.5 (0.23, 17 %) | 10.3 | 15.9 |
| 8PSK R3/4 | 17.1 (0.39) | 22.6 (0.15, 29 %) | 11.5 | 20.5 |
| 16QAM R1/2 | 13.7 (0.45) | 16.8 (0.37, 6 %) | 10.3 | 17.0 derived |
| 16QAM R2/3 | 17.8 (0.40) | 23.0 (0.13, 31 %) | 13.0 | 20.3 |
| 16QAM R3/4 | 22.0 (0.13, 20 % floor) | — (60 % floor) | 14.4 | 26.0 derived |

(Moderate floor = FER at the top of its 16 dB span.) The latent controller now uses the Good
column and the median usable slope, 0.41.

- **Adjacent QPSK code rates sit 2-3 dB apart on fading, not 5.** The waterfall is shaped by
  fades as much as by coding. The 5 dB derived spacing was the table's largest error.
- **The dense rungs gained 2.5-3.4 dB** with the measured-noise LLRs: 8PSK R3/4 is no longer the
  "never below 39 % FER" rung of July (11 % at 24 dB, 0.7 % at 30 dB on Good), and 16QAM R2/3's
  50 % point moved 20.3 → 17.8.
- **16QAM R1/2 beats 8PSK R2/3 by 1 dB at identical bits per carrier** (and identical frame
  geometry: 16 x 40 vs 12 x 54 bytes per frame) on every channel.
- **Moderate is not a shifted Good.** Relative to QPSK R1/2 the dense rungs sit 2-4 dB further
  out and carry 17-31 % error floors (coherent tracking at 0.5 Hz Doppler,
  BUG-MODERATE-8PSK-COHERENT-FLOOR). A single latent operating point absorbs a common offset,
  not a class-dependent spacing.

## 2. Measured best rung (delivered useful bytes/s, production geometry)

Useful bytes per frame x 5 frames x (1 − FER) / cycle (9.35 s incl. the 1.79 s non-data cycle;
QPSK R1/4 6.59 s). Selectable rungs only (8PSK R3/4, 16QAM R1/2 and R3/4 are disabled):

| SNR (in-band) | Good | Moderate | AWGN |
|---|---|---|---|
| 8 dB | QPSK R1/2 90 | QPSK R1/2 62 | QPSK R3/4 244 |
| 12 dB | QPSK R2/3 145 | QPSK R1/2 123 | 8PSK R2/3 334 |
| 16 dB | 8PSK R2/3 211 | QPSK R2/3 170 | 16QAM R2/3 449 |
| 20 dB | 16QAM R2/3 321 | QPSK R3/4 213 | 16QAM R2/3 449 |
| 24 dB | 16QAM R2/3 418 | 8PSK R2/3 257 | 16QAM R2/3 449 |

On Good the measured switches are QPSK R1/2 → R2/3 at ~11.9 dB, → 8PSK R2/3 at ~15.8, → 16QAM
R2/3 at ~19.8. The new theta table's argmax (production geometry) switches at 11.5 / 16.0 / 19.5;
the July table's at 10.0 / 17.0 / 22.5. QPSK R3/4 is never the measured best on Good — neither
is it the model's argmax — so the July "R3/4 never argmax" observation is physics at production
geometry. With the disabled rungs included, 16QAM R1/2 would be the measured best at 16-18 dB on
Good (+10-16 % over 8PSK R2/3) — its rig behaviour (amplitude-dependent decisions under PA
compression) is the open question, not its sim performance.

**On "3000 bps is a channel limit" (CLAUDE.md, from the July sweep):** on this PHY 16QAM R2/3
delivers 418 useful B/s ≈ 3.3 kbps at 24 dB on Good in this frame-level model. That is a model
of goodput from frame FER, not a GUI-gate measurement; see the GUI A/B below for what the
controller realizes.

## 3. Offline controller check

`latent_anchor_fit.py sim` replays the controller (posterior, tempering, relax, p25 decision,
dead/cap clamps) with outcomes drawn from the measured per-rung success, 60 transfers x 20
groups per cell, fraction of the best fixed rung, July → new table:

| | Good | Moderate | AWGN |
|---|---|---|---|
| 12-15 dB | 98-100 % → 99 % | 98 % → 99 % (15 dB) | 65 % → 74 % (12 dB) |
| 18 dB | 87 % → 98 % | 99 % → 97 % | 55 % → 85 % (16 dB) |
| 20 dB | 98 % → 100 % | 96 % → 97 % | 87 % → 97 % |
| 24 dB | 94 % → 100 % | 100 % → 99 % | — |

The new table switches rungs more often on Moderate (up to ~5 per 20 groups), the spacing
mismatch above; the simulation prices no rung-change anchor, which production does.

## 4. GUI gate (the faithful check) and shipping state

`tools/gui_qso_scenario.sh`, adaptive, 21 KB, `--cfo-hz 7.3`, seeds 42-46, all 60 runs PASS
(CRC-clean). Mean goodput (bps):

| scenario | July set (default) | measured table, old seed | measured set (`ULTRA_LATENT_ANCHORS=measured`) |
|---|---|---|---|
| Good@20 | 1700 | 1642 | **2154** (+26.7 %, 5/5 pairs up, paired t 3.7) |
| Good@15 | 1526 | 1502 | 1648 (+8 %, n.s.) |
| Moderate@20 | 1668 | 1526 | 1572 (−5.8 %, 2/5 up, n.s.; 5 craters vs 0) |
| AWGN@20 | 1720 | 1675 | **2660** (+54.7 %) |

The July set never leaves QPSK R2/3 at 20 dB on any channel within a 21 KB transfer: its R2/3
successes saturate the likelihood at x = 17.5, at its own 8PSK switch point. The measured table
alone is slower: seeded at the pick's 50 %-FER point the posterior starts ~5 dB below the
operating point, and the slower exit from R1/2 arms the one-shot startup probe, whose failure
pins R1/2 for the QSO (Moderate s43: 15/16 groups at R1/2). The measured set therefore seeds at
the point where the pick succeeds with the anchor probability (p = 0.9, theta + 5.4 dB) — it then
runs 8PSK R2/3 from group 2 on Good@20 and AWGN@20.

**Shipping state: opt-in.** The measured set wins where the Good thetas hold and over-commits on
Moderate (8PSK R2/3 held at 70-74 % delivery; the frame model calls it a near-tie with QPSK R3/4,
but each failure costs the GUI resends, anchors and craters). It stays default-off until a rig A/B
(MPG@20 and MPM@20, interleaved, n >= 8 pairs); the Moderate fix is class-aware thetas
(BUG-LATENT-CLASS-SPACING).

## 5. Caveats

- Simulator only (OTASim Watterson, stationary, no ionospheric time variation, RMS-normalized
  bursts). The rig validates.
- 6 seeds x 24 frames per point: FER sigma ~4 points at 50 %. The 2 dB grid limits theta to
  ~±0.3 dB through the logit interpolation.
- The controller observes groups of correlated frames; the frame-level FER here is the right
  per-frame quantity, and the controller's tempering (n_eff 2.2 per 5-frame group) handles the
  correlation.
