# Peak control for OFDM data bursts (design, 2026-11-21)

Status: Phase 0 (measurement) DONE 2026-11-21, results §5a; Phases 1-4 not built. Supersedes the "tone reservation is the real fix" note in the
2026-07-13 CHANGELOG entry as the plan of record (tone reservation stays an option, §8).

## 1. Why

A real transmitter is limited by its PEAK (PEP, ALC). Our TX peak-normalizes every burst
(`normalizeTxBurstForHardware`, `include/ultra/tx_burst_normalization.hpp`), so every dB of
envelope crest factor is a dB of average power — and of received SNR — that never reaches the
air. Coherent OFDM data frames run ~10 dB above their average (`ANALOG_CHAIN_VERIFIED_2026_07_26.md`:
PAPR 9.7 dB; rig F267: 14.5 dB real-sample crest on 16QAM bursts). A low-SNR rung (QPSK R1/2,
EVM threshold 5.5 dB) does not need a 30 dB-clean waveform: it can trade a 12-15 dB
signal-to-distortion floor for 6-8 dB more average power. A high rung (16QAM R3/4, 13.9 dB)
can only afford a little.

The default simulator cannot see any of this: it RMS-normalizes each burst, so crest factor
costs nothing (`app.cpp` `simPaprPenaltyEnabled()`). Every A/B in this design runs with
`gui_qso_scenario.sh --tx-model hardware` (peak normalization, the hardware path); the
2026-07-02 "cost > benefit" verdict came from the RMS-normalized sim and was structurally blind.

## 2. What exists and why it failed (2026-07-02 sim, 2026-07-13 rig)

`phy::applyPaprReduction` (`src/sim/papr_reduction.cpp`), called once per burst from
`StreamingEncoder::applyPaprReductionIfNeeded` (`streaming_encoder.cpp:150`, burst site `:966`),
knob `ULTRA_COHERENT_PAPR_DB` (default 0 = off, `:88`):

- ONE soft-clip pass (tanh knee at 0.75 x threshold) on the analytic envelope, then a band-limit
  to 50-2950 Hz. The filter regrows the peaks; one pass recovers only part of the reduction.
- ONE threshold for every rung (9-12 dB above RMS). Too deep for 16QAM/32QAM, far too shallow for
  QPSK — the rung that would gain the most got almost nothing.
- Applied to the WHOLE burst, LTS and descriptor included, with no receiver-side model of what
  the clip did: no clip-noise term in the LLR variance, and no account of the clip's Bussgang
  gain on the data relative to the LTS the channel estimate is seeded from.
- Rig result (MPG@30, 16QAM, 3 interleaved pairs): marginal channel +39 %, clean channel −58 %
  (1199 CW fails), net −12 %. The clean-channel crater is the signature of a distortion floor
  the receiver does not model: confident LLRs on cells whose error is mostly clip noise.

## 3. Physics: the per-rung headroom is derived, not tuned

Definitions, per OFDM geometry (wideband 1024/59, narrowband 2048/21), for the clipper at
headroom h (envelope clip level h dB above the envelope RMS):

- Π(h): burst crest factor after clipping, PEP / P_avg (envelope domain: PEP = peak² / 2 of the
  real signal, P_avg its mean power) — the dB the peak-limited TX gives up.
- D(h): data signal-to-distortion ratio — in-band clip-noise power on a data cell over the wanted
  data power, after the Bussgang gain. The clip is made at TX, so its noise fades WITH the
  carrier: in the equalized domain it is a flat per-cell floor on every channel, AWGN or fading.
- α(h): Bussgang gain of the data cells (wanted component after clipping over before).

After the IDFT every constellation looks Gaussian, so for a plain envelope clip Π, D and α
depend on the geometry and h, not on the modulation. They are MEASURED once per geometry by a
tool on the production encoder (§5 Phase 0) and pinned by a CTest, not hand-set.

With ρ = PEP / N (channel SNR on the peak ruler) the data cells see

    1/γ_eff = Π(h) / (α(h)² ρ) + 1/D(h)

and rung r (EVM threshold η_r, `docs/EVM_RATE_SELECTION_2026_11_08.md` §1 — the code's own
waterfall in the EVM domain, channel-independent) decodes when γ_eff ≥ η_r, i.e. it needs

    ρ_r(h) = Π(h) η_r / (α(h)² (1 − η_r / D(h))),     D(h) > η_r.

The rung's headroom is h_r* = argmin_h ρ_r(h): a function of η_r and the measured geometry
curves only. No constant per modulation, no SNR-specific threshold — adaptive by construction
(CLAUDE.md ADAPTIVITY rule). The gain of the rung is ρ_r(∞) / ρ_r(h_r*), which Phase 0 prints for
every rung BEFORE any code ships (proof before build).

## 4. Design

### 4.1 TX (StreamingEncoder burst site)

1. Iterative clip-and-filter on the analytic envelope, K passes (K and the per-pass overshoot
   measured in Phase 0; overshoot > 1 on a pass clips past the threshold so the next filter
   regrows back toward it, not above it). Filter = the OCCUPIED band of the geometry
   (`carrierFrequencyHz()` edges plus the transition band), not 50-2950 Hz: power regrown between
   2906 and 2950 Hz is PEP spent on nothing.
2. Threshold from the burst's DATA-segment power, not the whole burst's: a group-start member
   carries an LTS pair and a descriptor whose statistics differ.
3. Every segment of the key-down obeys ONE envelope ceiling. The TX sets the burst's gain from its
   single largest peak (INV-FRAME-WHITEN-001 was exactly one symbol setting the burst's peak), so a
   clipped data segment gains nothing if the LTS, descriptor or chirp anchor peaks higher. Phase 0
   measures every segment's envelope peak against the clipped data's; a known segment that exceeds
   it is clipped at the same ceiling (and §4.2 accounts for it) or scaled, by a factor the receiver
   knows.
4. Headroom per rung h_r* from §3, keyed on the (modulation, code rate) of the burst — both ends
   know the rung from the descriptor. Control frames, ACK tones and MC-DPSK are out of scope
   for the first increment (§7).

### 4.2 RX

1. **Channel reference.** Comb pilots inside the data symbols pass through the same clip as the
   data, so pilot-referenced H already contains α(h). The LTS does not (different statistics,
   different gain), so an LTS-seeded estimate is off by α(h): a gain error that 16QAM/32QAM read as
   an amplitude bias. The receiver scales the LTS-derived H by the rung's α(h) when it hands off to
   data symbols (rung known from the descriptor; table from Phase 0). Phase 0 measures how large
   the bias is in today's equalizer blend before deciding whether this is needed.
2. **LLR variance.** The per-cell variance becomes thermal + clip, not max(thermal, floor):
   v_c = σ²/|H_c|² + P_data/D(h). Today's measured-noise model (`measured_llr_noise.hpp`, ON)
   floors at the frame's decision residual, which contains the clip noise, but takes the MAX of
   the two terms — up to 3 dB over-confident where they are equal. With D(h) known per rung the
   term is exact, not estimated.
3. **Rate selector (counterfactual across rungs).** Switching rung now changes the transmitted
   AVERAGE power (Π differs per rung) and the distortion floor. A frame on rung a reading EVM e_a
   predicts rung b as

       γ_th,a = 1 / (1/e_a − 1/D_a),   γ_th,b = γ_th,a · (Π_a α_b²) / (Π_b α_a²),
       e_b = 1 / (1/γ_th,b + 1/D_b)

   and compares e_b with η_b (`evm_rate_selector.hpp`). Without this, an up-step from a
   hard-clipped QPSK rung over-predicts the next rung by the Π difference (several dB): exactly
   the clean-channel crater. The η_r themselves stay valid if 4.2.2 holds (verified in Phase 2).
4. **Connect-time pick.** The PAPR correction of the connect pick (INV-SNR-PAPR-001) reads the
   live TX level gap (`App::noteTxBurstLevelForPapr`), so it follows the new crest factors without
   a constant change; verify, do not assume.

### 4.3 Operator lens (PA duty / thermal)

Lower crest factor means MORE average power for the same PEP: a QPSK burst clipped to ~1-2 dB
crest puts ~6x the average power of today's burst into the PA — RTTY/FT8 territory. Many
transceivers derate continuous digital duty (often to ~50 %). This is a physical constraint, not
a tuning question: the TX takes an operator-facing average-power ceiling (fraction of PEP) and the
headroom is floored at whatever keeps P_avg under it (h ≥ h_min(ceiling), derived from Π(h)). The
default is the operator's call (§9). Burst length is already duty-bounded (`burstAirtimeBudgetFrames`).

## 5. Phases (each ships alone, each with its proof)

- **Phase 0 — measure (no behaviour change).** Extend `tools/papr_tx_measure.cpp` (production
  StreamingEncoder burst, every rung): per h and pass count, Π(h), D(h) on data cells, α(h) on
  data cells / comb pilots / LTS, and every segment's envelope peak vs the clipped data's. Print
  h_r* and the predicted gain ρ_r(∞)/ρ_r(h_r*) for every rung. Write the curves to a table checked
  in under `src/ofdm/` and pin it with a CTest (regenerated whenever the clipper changes). Stop here
  if the predicted gains are small.
- **Phase 1 — TX clipper + RX α/LLR model**, behind one lockstep knob (default off). Gate: GUI
  `--tx-model hardware`, AWGN 0/5/10/15/20 dB peak SNR, Good 10/20, Moderate 20, 3 seeds each,
  interleaved arms, plus `--tx-pa on` (ALC on low-crest signals). The clean-channel cell (AWGN 30,
  Good 30) is MANDATORY — it is where the July clipper cratered.
- **Phase 2 — rate selector counterfactual** (§4.2.3) and re-check of η_r under clip noise with
  `measure_ack_fer --frame-log` (the 50 % points must not move beyond the 0.3 dB the method
  resolves).
- **Phase 3 — constellation-aware clipping for 16QAM/32QAM:** distortion pushed onto the outer
  points (which may move outward without moving toward a decision boundary), the inner points
  held. Gain only on those rungs; measured in Phase 0 style first. Builds on the existing
  outward-only ACE (`src/ofdm/papr_ace.hpp`, `ULTRA_ACE_PAPR`, default off; 0.7-0.9 dB alone,
  rig A/B 2026-07-13 inconclusive at 3 pairs), which today runs per symbol BEFORE any clip.
- **Phase 4 — other segments**: descriptor, chirp anchor, control frames, MC-DPSK (few carriers,
  less Gaussian — its own Π/D curves).

## 5a. Phase 0 results (2026-11-21, `tools/peak_control_measure`, `--edge-bins 3`)

Production bursts (`ModemEngine::transmitBurst`, second burst of a session, 6 frames), every cell
aligned exactly with the encoder's symbol capture (correlation 1.00000 on every symbol of every
rung). Clipper: envelope clip at h dB above the data symbols' RMS envelope, brick-wall band limit
to the occupied band ± 3 carrier spacings, overshoot schedule 1 / 1.25 / 1.5 / 1.75 / 2 (5 passes).

Best headroom and predicted PEP-fair gain per rung (gain = the dB of extra channel SNR, on the
peak ruler, that the clip is worth at the rung's turbo EVM threshold; §3 formula):

| rung | η (dB) | best h (dB) | 1 pass | 3 passes | 5 passes |
|---|---|---|---|---|---|
| QPSK R1/4 | 3.8 | 0 | +4.3 | +5.9 | **+6.0** |
| QPSK R1/2 | 4.7 | 1 | +4.8 | +6.2 | **+6.4** |
| QPSK R2/3 | 6.05 | 1 | +4.3 | +5.5 | **+5.6** |
| QPSK R3/4 | 6.65 | 2 | +4.3 | +5.7 | **+5.8** |
| 8PSK R2/3 | 9.1 | 3 | +3.5 | +4.5 | **+4.6** |
| 8PSK R3/4 | 10.0 | 4 | +4.0 | +5.2 | **+5.3** |
| 16QAM R1/2 | 10.85 | 4 | +3.4 | +4.6 | **+4.7** |
| 16QAM R2/3 | 11.75 | 4 | +3.8 | +4.8 | **+4.8** |
| 16QAM R3/4 | 12.3 | 4 | +3.1 | +4.0 | **+4.1** |
| 32QAM R2/3 | 15.5 | 6 | +2.4 | +3.5 | **+3.6** |

Measured facts behind it:
- Unclipped burst crest Π = 9.8-11.0 dB over the data envelope, and the PEAK IS ALWAYS IN THE DATA
  SYMBOLS (LTS ~6.2 dB, chirp/descriptor/other ~4.5 dB). After clipping the peak stays in the data:
  no other segment caps the gain at these headrooms.
- Pass count is most of the gain over the July clipper: 1 pass leaves 1-1.6 dB on the table.
- Bussgang gain is the same on data and comb-pilot cells to ±0.004 (e.g. QPSK R1/2 h=1: 0.762 /
  0.761), and D on pilots equals D on data to ±0.2 dB: a pilot-referenced channel estimate stays
  consistent with the data, as §4.2.1 assumed.
- The LTS is NOT: at h=1 α_LTS = 0.842 vs α_data = 0.762 (0.87 dB), at h=4 0.971 vs 0.921 (0.46 dB).
  An LTS-seeded H is biased by α_data/α_LTS, so the §4.2.1 correction is required, not optional.
- D(h) is the same for every rung at the same h (QPSK, 16QAM and 32QAM bursts at h=4: 18.50 / 18.19 /
  18.30 dB): plain clipping is constellation-independent after the IDFT, as §3 states.

Limits of this prediction (what the GUI gate must settle): (1) one burst per rung — the unclipped
crest is an extreme value and varies burst to burst (9.8-11.0 dB here), so each gain is ±~0.5 dB;
(2) it assumes the EVM thresholds hold with clip noise in the cells and that pilot-estimate noise at
D ≈ 12-18 dB costs nothing after the equalizer's smoothing — neither is measured; (3) the receiver
changes of §4.2 are not built, and without the α_LTS correction and clip-noise LLR term the July
clean-channel failure mode is expected.

## 6. Gate summary

Every A/B: `--tx-model hardware` (the July sim verdict was blind without it), real 7.3 Hz CFO,
≥ 3 seeds, interleaved, whole matrix including the clean cells, ctest green. A win at one SNR
with a loss anywhere else does not ship.

## 7. Out of scope for the first increment

MC-DPSK and control frames (their crest factor is lower and their failure costs a connection, not
a frame); OFDM_NARROW (same method, its own curves — Phase 4).

## 8. Alternatives considered

- **Tone reservation** (peak-cancelling signal on reserved carriers): zero data distortion, but it
  spends carriers (≈5-10 % of 59) and gains ~2-4 dB at our carrier count. It may beat clipping on
  the top rungs, where D(h) must stay high; Phase 0's ρ_r(h) curves tell whether to build it for
  those rungs instead of Phase 3.
- **SNR-gated crude clip** (the July note): a threshold switched by SNR is exactly the kind of
  one-cell tuning the adaptivity rule forbids; §3 is its principled form.

## 9. Open decisions (operator)

1. Default average-power ceiling for §4.3 (none / 50 % / other).
2. Lockstep knob default after Phase 1 passes (protocol is free to break; same setting both ends).

## 10. Correction to the 2026-11-21 benchmark discussion

The "long codes" item in the comparison against a commercial-class modem's published numbers is
mostly NOT a gap: OFDM data frames are already one turbo codeword per frame (cw × 648 coded bits,
`ULTRA_DATA_FEC=turbo` under `ULTRA_FAST`, `frame_v2.cpp` `ofdmDataFrameFec()`). Peak control,
incremental-redundancy resends and rungs above 32QAM remain.
