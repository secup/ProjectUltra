#!/usr/bin/env python3
"""Check that OTASim stations were half-duplex in a gui_qso_scenario.sh run.

Reads the server's E2E debug log (OUT/e2e_server.log): for every session tick in which a station
transmitted (session_tick_tx rms > 0), its received audio for that tick (session_outbox_enqueue)
must be silent. Prints, per station, the keyed ticks and how many of them still carried audio.
A working half-duplex radio shows 0 (or a tick or two where a burst's first audio arrived just in
time for the PTT lead look-ahead); the full-duplex stations of 2026-05-30..2026-11-11 show nearly all.

    python3 tools/check_half_duplex.py OUT/e2e_server.log
"""
import re
import sys
from collections import defaultdict

PAT_TX = re.compile(r"session_tick_tx station=(\w+) session=\S+ start=(\d+) samples=\d+ rms=(\S+)")
PAT_RX = re.compile(r"session_outbox_enqueue receiver=(\w+) session=\S+ start=(\d+) samples=\d+ rms=(\S+)")


def main(path):
    tx, rx = defaultdict(dict), defaultdict(dict)
    with open(path, errors="replace") as log:
        for line in log:
            try:
                m = PAT_TX.search(line)
                if m:
                    tx[m.group(1)][int(m.group(2))] = float(m.group(3))
                    continue
                m = PAT_RX.search(line)
                if m:
                    rx[m.group(1)][int(m.group(2))] = float(m.group(3))
            except ValueError:
                continue  # a line interleaved by concurrent writers
    for station in sorted(tx):
        if station not in rx and len(tx[station]) < 2:
            continue  # a station id garbled by interleaved writes
        keyed = [s for s, r in tx[station].items() if r > 0]
        heard = [s for s in keyed if rx[station].get(s, 0.0) > 0]
        print(f"{station}: {len(keyed)} keyed ticks, {len(heard)} with received audio"
              + (f" (first at session sample {min(heard)})" if heard else ""))
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
