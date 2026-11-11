#!/usr/bin/env python3
"""Measure what the modem puts on the air: segments, gaps, edges and splatter.

Input: raw float32 48 kHz audio, e.g. `ultra ptx ... -o tx.f32` (ModemEngine's TX path) or a
gui_qso_scenario.sh --record capture ({alpha,bravo}_tx.f32).

For each transmitted segment (silence-separated) it prints the duration, the gap before it, the
RMS of the first/last millisecond against the middle (an edge at full level = a hard on/off), the
largest sample-to-sample step, and the spectral splatter:
  - outside: energy below/above the segment's own band, relative to the in-band total;
  - worst bin: the strongest out-of-band bin against the average in-band density;
  - edge row: one 2048-sample Hann row (the GUI waterfall's) centred on each edge - the worst bin
    more than 300 Hz from the segment's band edges, against that row's in-band peak. A hard
    on/off shows here even when the averaged spectrum hides it (one click in 500 ms).
--png FILE renders the waterfall the GUI shows (2048 Hann, 1024 hop, time running up).

    python3 tools/tx_spectrum.py tx.f32 [more.f32 ...] [--png out.png]
"""
import sys

import numpy as np

FS = 48000.0


def load(path):
    return np.fromfile(path, dtype=np.float32).astype(np.float64)


def segments(x, thr=1e-6, min_gap_ms=2.0):
    """Non-silent spans; zero runs shorter than min_gap_ms (zero crossings) stay inside."""
    nz = np.abs(x) > thr
    min_gap = int(min_gap_ms * FS / 1000)
    segs, i, n = [], 0, len(x)
    while i < n:
        if not nz[i]:
            i += 1
            continue
        j = i
        while j < n:
            if nz[j]:
                j += 1
                continue
            k = j
            while k < n and not nz[k] and k - j < min_gap:
                k += 1
            if k < n and nz[k] and k - j < min_gap:
                j = k
                continue
            break
        segs.append((i, j))
        i = j
    return segs


def psd(x, nfft=4096):
    w = np.hanning(nfft)
    acc, n = None, 0
    for s in range(0, max(1, len(x) - nfft + 1), nfft // 2):
        frame = x[s:s + nfft]
        if len(frame) < nfft:
            frame = np.pad(frame, (0, nfft - len(frame)))
        p = np.abs(np.fft.rfft(frame * w)) ** 2
        acc = p if acc is None else acc + p
        n += 1
    return np.fft.rfftfreq(nfft, 1 / FS), acc / n


def occupied_band(f, p):
    """Band holding 99 % of the energy (0.5 % trimmed from each side)."""
    c = np.cumsum(p) / np.sum(p)
    return f[np.searchsorted(c, 0.005)], f[np.searchsorted(c, 0.995)]


def edge_row(x, centre, lo, hi):
    """Worst bin of the waterfall row centred on an edge, against that row's peak, among bins
    outside the segment's band AND more than 300 Hz from the row's peak (a chirp's edge sits at
    its end frequency, not across its sweep)."""
    pad = np.zeros(2048)
    seg = np.concatenate([pad, x, pad])
    c = len(pad) + centre
    row = np.abs(np.fft.rfft(seg[c - 1024:c + 1024] * np.hanning(2048))) ** 2
    f = np.fft.rfftfreq(2048, 1 / FS)
    peak = int(np.argmax(row))
    far = ((f < lo - 300) | (f > hi + 300)) & (np.abs(f - f[peak]) > 300) & (f <= 6000)
    if not far.any():
        return float("nan")
    return 10 * np.log10(row[far].max() / row[peak] + 1e-30)


def report(path):
    x = load(path)
    print(f"== {path}: {len(x) / FS * 1000:.1f} ms, peak {np.max(np.abs(x)):.3f}")
    prev_end = 0
    for a, b in segments(x):
        seg = x[a:b]
        mid = np.sqrt(np.mean(seg[len(seg) // 4: 3 * len(seg) // 4] ** 2))
        first = np.sqrt(np.mean(seg[:48] ** 2))
        last = np.sqrt(np.mean(seg[-48:] ** 2))
        f, p = psd(seg)
        lo, hi = occupied_band(f, p)
        inband = (f >= lo) & (f <= hi)
        ref = np.sum(p[inband])
        outside = ((f < lo - 200) & (f > 20)) | (f > hi + 200)
        out_db = 10 * np.log10(np.sum(p[outside]) / ref + 1e-30)
        worst = 10 * np.log10(np.max(p[outside]) / (ref / np.sum(inband)) + 1e-30)
        e0, e1 = edge_row(x, a, lo, hi), edge_row(x, b, lo, hi)
        print(f"  {a / FS * 1000:8.1f}-{b / FS * 1000:8.1f} ms ({(b - a) / FS * 1000:7.1f}, gap before "
              f"{(a - prev_end) / FS * 1000:6.1f}) band {lo:5.0f}-{hi:5.0f} Hz | rms first/mid/last "
              f"{first:.3f}/{mid:.3f}/{last:.3f} | max step {np.max(np.abs(np.diff(seg))):.3f} | outside "
              f"{out_db:6.1f} dB, worst bin {worst:6.1f} dB | edge rows {e0:6.1f}/{e1:6.1f} dB")
        prev_end = b


def png(paths, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(paths), figsize=(5 * len(paths), 7))
    for ax, path in zip(np.atleast_1d(axes), paths):
        x = load(path) + 1e-4 * np.random.default_rng(1).standard_normal(len(load(path)))
        rows = [20 * np.log10(np.abs(np.fft.rfft(x[s:s + 2048] * np.hanning(2048))) + 1e-9)
                for s in range(0, len(x) - 2048 + 1, 1024)]
        img = np.array(rows)[:, : int(3000 / (FS / 2048)) + 1]
        ax.imshow(img, origin="lower", aspect="auto", extent=[0, 3000, 0, len(x) / FS * 1000],
                  vmin=img.max() - 70, vmax=img.max(), cmap="viridis")
        ax.set_title(path.split("/")[-1])
        ax.set_xlabel("Hz")
        ax.set_ylabel("ms (time runs up, as on the GUI waterfall)")
    plt.tight_layout()
    plt.savefig(out, dpi=80)
    print("wrote", out)


def main(argv):
    out = None
    if "--png" in argv:
        i = argv.index("--png")
        out = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    if not argv:
        print(__doc__)
        return 2
    for path in argv:
        report(path)
    if out:
        png(argv, out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
