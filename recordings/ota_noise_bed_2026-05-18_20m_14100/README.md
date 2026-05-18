# 20m noise bed — 14.100 MHz, 2026-05-18 morning

Real-HF reference capture for the OTASim noise-overlay workstream (task #71).

## Source

- Date / time: 2026-05-18 ~11:30 AST (15:30 UTC)
- Frequency: 14.100 MHz USB
- Band: 20m
- Radio: Yaesu FT-891, USB data out (level 30 default)
- Recording chain: FT-891 USB → Mac audio input → ultra_gui `-rec`
- Sample format: 16-bit PCM, mono, 48 kHz
- Duration: 600.00 s (10:00 exactly)
- Source bundle: ultra_gui diagnostics report 0ad79b
- Author callsign: redacted (privacy.callsigns_included=false in the bundle)

## Levels

| Metric | Value |
|---|---|
| Overall RMS | 0.0363 |
| Peak | 0.4738 |
| Per-5s segment RMS | 0.027 – 0.042 (median 0.037, stddev 0.003) |
| Clipping (>0.9) | 0% |
| Dynamic range from 16-bit floor | ~30 dB |

The audio level sits a hair below the modem-decode calibration target
of RMS 0.05-0.25 (CLAUDE.md), but for noise-bed playback purposes the
absolute level is normalized at load time. The capture is solidly
within the dynamic-range envelope and has no clipping.

## Spectral character

| Band | Energy share (mid chunk) |
|---|---|
| 0-300 Hz | ~1% |
| 300-1000 Hz | ~16% |
| 1000-2000 Hz | ~56% |
| 2000-3000 Hz | ~27% |
| 3000+ Hz | <1% |

~98% of energy in the SSB passband (300-3000 Hz). No DC contamination,
no AC-mains bleed, no high-frequency junk. Real-HF noise shape.

## Content

- Steady background band noise (broadband, structured — not Gaussian)
- NCDXF beacon network rotation (14.100 hosts the IBP beacons; one
  station transmits every 10 seconds). Over 10 min you get 60+
  beacon transmissions interleaved with quiet periods.
- No dominant strong nearby signal (per-segment RMS stddev only 0.003).

## How to use

Load this WAV at OTASim service startup, normalize to unit RMS,
then scale per-tick by `kModemReferenceRms * 10^(-snr_db/20)` (same
formula as AWGN) to feed continuous real-HF-character noise to all
session receivers. Loop on `position % length` — at 10 min there is
no audible periodicity.

See `docs/AGENT_TASK_BACKLOG.md` (task #71) and prior `ota_simulator
v2` real-HF noise overlay for the implementation pattern.
