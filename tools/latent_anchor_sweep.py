#!/usr/bin/env python3
import subprocess, sys, itertools, os
from concurrent.futures import ThreadPoolExecutor
# Per-rung FER sweep for the latent-controller theta re-anchor (production geometry, 7.3 Hz CFO).
# usage: tools/latent_anchor_sweep.py OUT.csv [workers]   (~25 min on 11 workers; analyse with
#        tools/latent_anchor_fit.py; docs/LATENT_ANCHOR_MEASUREMENT_2026_11_08.md)
# RESUMABLE: rows already in OUT are skipped; new rows are APPENDED (never truncate).
BIN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "build", "measure_ack_fer")
OUT = sys.argv[1]
WORKERS = int(sys.argv[2]) if len(sys.argv) > 2 else 11
RUNGS = [("qpsk","r1_4",-4),("qpsk","r1_2",0),("qpsk","r2_3",4),("qpsk","r3_4",8),
         ("qam8","r2_3",10),("qam8","r3_4",14),("qam16","r1_2",12),("qam16","r2_3",14),("qam16","r3_4",18)]
CHANNELS = {"good":0, "moderate":0, "awgn":-4}
SEEDS = [11,23,37,53,71,89]
done = set()
if os.path.exists(OUT):
    for l in open(OUT):
        f = l.strip().split(",")
        if len(f) >= 12 and f[0] != "snr" and f[11] != "ERR":
            done.add((f[3], f[4], f[2], int(float(f[0])), int(f[6])))
jobs = []
for ch in ["good", "moderate", "awgn"]:          # Good first: it calibrates theta
    sh = CHANNELS[ch]
    for (m, r, lo), seed in itertools.product(RUNGS, SEEDS):
        for k in range(9):
            j = (m, r, ch, lo + sh + 2 * k, seed)
            if j not in done:
                jobs.append(j)
def run(j):
    m, r, ch, snr, seed = j
    cmd = [BIN, "--snr", str(snr), "--config", "data4_full", "--seed", str(seed), "--n", "24",
           "--channel", ch, "--mod", m, "--rate", r, "--geometry", "production",
           "--preroll-ms", "300", "--cfo-hz", "7.3"]
    p = subprocess.run(cmd, capture_output=True, text=True)
    lines = [l for l in p.stdout.splitlines() if l.count(",") >= 10]
    return lines[-1] if lines else f"{snr},data4_full,{ch},{m},{r},0,{seed},24,ERR,ERR,ERR,ERR"
fresh = not os.path.exists(OUT) or os.path.getsize(OUT) == 0
with open(OUT, "a") as f:
    if fresh:
        f.write("snr,config,channel,mod,rate,ci,seed,n,sync_fail,decode_fail,crc_fail,pass\n")
    print("skipping", len(done), "running", len(jobs), flush=True)
    with ThreadPoolExecutor(max_workers=WORKERS) as ex:
        for line in ex.map(run, jobs):
            f.write(line + "\n"); f.flush()
print("SWEEP_DONE", len(jobs), flush=True)
