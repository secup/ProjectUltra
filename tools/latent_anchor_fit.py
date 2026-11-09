#!/usr/bin/env python3
"""Latent rate-controller anchor analysis (2026-11-08) for a tools/latent_anchor_sweep.py CSV.

  latent_anchor_fit.py fit  SWEEP.csv            theta50 / slope / floor per channel x rung
  latent_anchor_fit.py best SWEEP.csv CH [all]   measured best rung per SNR (production geometry)
  latent_anchor_fit.py sim  SWEEP.csv            offline controller Monte-Carlo, old vs new table

theta50 = the logit-interpolated 50 %-FER crossing; slope/theta_fit = least-squares logistic over
the 20-70 % FER band; floor = the lower FER of the top two SNR points.

`sim` MIRRORS src/protocol/latent_rate_controller.hpp (posterior bins, tempering, relax, p25
decision, dead/cap clamps, all-dead fallback) with outcomes drawn from the MEASURED per-rung
success. It is an analysis aid, not a second controller: keep it in step with the header. It
prices no rung-change anchor, so it overstates switching relative to production.

Geometry = useful bytes per frame and cycle time from the production helpers (5-frame group,
1.79 s non-data cycle), as tests/test_latent_rate_controller.cpp computes them.
"""
import collections
import csv
import math
import random
import sys

ORDER = ["qpsk r1_4", "qpsk r1_2", "qpsk r2_3", "qpsk r3_4", "qam8 r2_3", "qam8 r3_4",
         "qam16 r1_2", "qam16 r2_3", "qam16 r3_4"]
GEOM = {"qpsk r1_4": (56, 6.59), "qpsk r1_2": (296, 9.35), "qpsk r2_3": (408, 9.35),
        "qpsk r3_4": (456, 9.35), "qam8 r2_3": (624, 9.35), "qam8 r3_4": (696, 9.35),
        "qam16 r1_2": (616, 9.35), "qam16 r2_3": (840, 9.35), "qam16 r3_4": (936, 9.35)}
ENABLED = ["qpsk r1_4", "qpsk r1_2", "qpsk r2_3", "qpsk r3_4", "qam8 r2_3", "qam16 r2_3"]
TABLES = {  # (shared slope, theta per rung)
    "old": (0.35, dict(zip(ORDER, [-2, 3, 8, 13, 15.9, 20.5, 17, 20.3, 26]))),
    "new": (0.41, dict(zip(ORDER, [4.9, 7.3, 10.3, 12.3, 14.7, 17.1, 13.7, 17.8, 22.0]))),
}


def load(path):
    """(channel, rung, snr) -> [frames, passes], pooled over seeds."""
    agg = collections.defaultdict(lambda: [0, 0])
    for r in csv.DictReader(open(path)):
        if r["pass"] in ("ERR", ""):
            continue
        k = (r["channel"], r["mod"] + " " + r["rate"], float(r["snr"]))
        agg[k][0] += int(r["n"])
        agg[k][1] += int(r["pass"])
    return agg


def logit(f):
    f = min(max(f, 1e-3), 1 - 1e-3)
    return math.log(f / (1 - f))


def cmd_fit(path):
    agg = load(path)
    for ch in ["good", "moderate", "awgn"]:
        print(f"== {ch}")
        print(f"  {'rung':11} {'theta50':>8} {'slope':>6} {'theta_fit':>9} {'floor':>6}  FER by SNR")
        for rung in ORDER:
            pts = sorted((s, 1 - p / n) for (c, ru, s), (n, p) in agg.items()
                         if c == ch and ru == rung)
            if not pts:
                continue
            t50 = None
            for (s0, f0), (s1, f1) in zip(pts, pts[1:]):
                if f0 >= 0.5 >= f1 and f0 != f1:
                    l0, l1 = logit(f0), logit(f1)
                    t50 = s0 + (0.0 - l0) * (s1 - s0) / (l1 - l0)
                    break
            band = [(s, f) for s, f in pts if 0.2 <= f <= 0.7]
            slope = theta = None
            if len(band) >= 2:
                xs = [s for s, _ in band]
                ys = [-logit(f) for _, f in band]  # logit(success)
                mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
                sxx = sum((x - mx) ** 2 for x in xs)
                if sxx > 0:
                    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
                    theta = mx - my / slope if slope else None
            floor = min(f for _, f in pts[-2:])
            fmt = lambda v, w, d: f"{v:{w}.{d}f}" if v is not None else " " * (w - 1) + "-"
            curve = " ".join(f"{s:+.0f}:{100 * f:.0f}" for s, f in pts)
            print(f"  {rung:11} {fmt(t50, 8, 1)} {fmt(slope, 6, 3)} {fmt(theta, 9, 1)} "
                  f"{100 * floor:5.1f}%  {curve}")


def cmd_best(path, ch, include_disabled):
    agg = load(path)
    rungs = ORDER if include_disabled else ENABLED
    for s in sorted({s for (c, _, s) in agg if c == ch}):
        row = []
        for rung in rungs:
            if (ch, rung, s) not in agg:
                continue
            n, p = agg[(ch, rung, s)]
            b, cyc = GEOM[rung]
            row.append((b * 5 * (p / n) / cyc, rung, 1 - p / n))
        if not row:
            continue
        row.sort(reverse=True)
        line = f"  {s:+3.0f} dB: best {row[0][1]:10} {row[0][0]:6.0f} B/s (FER {100 * row[0][2]:4.1f}%)"
        if len(row) > 1:
            line += f"   2nd {row[1][1]:10} {row[1][0]:6.0f} B/s (FER {100 * row[1][2]:4.1f}%)"
        print(line)


# ---- offline controller (mirrors latent_rate_controller.hpp) ----
BINS, LO, STEP = 61, -5.0, 0.5
XS = [LO + STEP * i for i in range(BINS)]
PMIN, PMAX, DEAD, CAP, PESS, NEFF, RELAX = 0.035, 0.965, 0.10, 0.90, 0.25, 2.2, 0.35


def succ(x, theta, slope):
    p = 1 / (1 + math.exp(-max(-40, min(40, slope * (x - theta)))))
    return min(max(p, PMIN), PMAX)


def measured_success(agg, ch, rung, snr):
    """Measured frame success at snr, linear between the 2 dB sweep points."""
    pts = sorted((s, p / n) for (c, ru, s), (n, p) in agg.items() if c == ch and ru == rung)
    if not pts:
        return 0.0
    if snr <= pts[0][0]:
        return pts[0][1]
    if snr >= pts[-1][0]:
        return pts[-1][1]
    for (s0, p0), (s1, p1) in zip(pts, pts[1:]):
        if s0 <= snr <= s1:
            return p0 + (p1 - p0) * (snr - s0) / (s1 - s0)
    return 0.0


def simulate(agg, ch, snr, table, groups=20, seed=0):
    slope, theta = TABLES[table]

    def ptrue(rung):
        return measured_success(agg, ch, rung, snr)

    rng = random.Random(seed)
    logp = [-0.5 * (x - snr) ** 2 / 9.0 for x in XS]

    def normalise():
        m = max(logp)
        lz = m + math.log(sum(math.exp(v - m) for v in logp))
        for i in range(BINS):
            logp[i] -= lz

    normalise()
    cur, switches, sent_bytes, airtime = "qpsk r1_2", 0, 0.0, 0.0
    for _ in range(groups):
        acc, xp = 0.0, XS[-1]
        for i in range(BINS):
            acc += math.exp(logp[i])
            if acc >= PESS:
                xp = XS[i]
                break
        best, best_g, robust, robust_p = None, 0.0, None, -1.0
        for rung in ENABLED:
            p = succ(xp, theta[rung], slope)
            if p > robust_p:
                robust, robust_p = rung, p
            if p < DEAD:
                continue
            g = GEOM[rung][0] * 5 * min(p, CAP) / GEOM[rung][1]
            if g > best_g:
                best, best_g = rung, g
        best = best or robust
        switches += best != cur
        cur = best
        k = sum(rng.random() < ptrue(cur) for _ in range(5))
        sent_bytes += GEOM[cur][0] * k
        airtime += GEOM[cur][1]
        w = min(1.0, max(0.05, NEFF / 5))
        for i in range(BINS):
            p = succ(XS[i], theta[cur], slope)
            logp[i] += w * (k * math.log(p) + (5 - k) * math.log(1 - p))
        normalise()
        s = RELAX / STEP
        half = min(BINS - 1, math.ceil(3 * s))
        m = max(logp)
        pr = [math.exp(v - m) for v in logp]
        out = []
        for i in range(BINS):
            a = ws = 0.0
            for d in range(-half, half + 1):
                j = i + d
                if 0 <= j < BINS:
                    wk = math.exp(-0.5 * d * d / (s * s))
                    a += wk * pr[j]
                    ws += wk
            out.append(a / ws)
        for i in range(BINS):
            logp[i] = math.log(max(out[i], 1e-30))
        normalise()
    return sent_bytes / airtime, switches


def cmd_sim(path):
    agg = load(path)
    cells = [("good", [12, 15, 18, 20, 24]), ("moderate", [15, 18, 20, 24]),
             ("awgn", [8, 12, 16, 20])]
    for ch, snrs in cells:
        print(f"== {ch}")
        for snr in snrs:
            oracle = max(GEOM[r][0] * 5 * measured_success(agg, ch, r, snr) / GEOM[r][1]
                         for r in ENABLED)
            line = f"  {snr:3d} dB  oracle {oracle:5.0f} B/s"
            for t in ("old", "new"):
                res = [simulate(agg, ch, snr, t, seed=s) for s in range(60)]
                gp = sum(r[0] for r in res) / len(res)
                sw = sum(r[1] for r in res) / len(res)
                line += f" | {t}: {gp:5.0f} B/s ({100 * gp / oracle:3.0f}%) switches {sw:4.1f}"
            print(line)


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "fit":
        cmd_fit(sys.argv[2])
    elif len(sys.argv) >= 4 and sys.argv[1] == "best":
        cmd_best(sys.argv[2], sys.argv[3], len(sys.argv) > 4)
    elif len(sys.argv) >= 3 and sys.argv[1] == "sim":
        cmd_sim(sys.argv[2])
    else:
        print(__doc__)
