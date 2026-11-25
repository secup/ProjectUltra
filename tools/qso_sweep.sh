#!/usr/bin/env bash
#
# qso_sweep.sh — run a matrix of forced-rung GUI QSO file-transfer tests
# back-to-back and tabulate the REAL decode / fade-recovery metrics (via
# analyze_qso_run.sh — the summary.env counters undercount burst-group fade
# recovery; see that script's header).
#
# Each spec line:  <channel> <snr_db> <mod> <rate> [file_kb]
#   e.g.           good 15 QPSK R3/4 20
# Lines beginning with '#' and blank lines are skipped. Specs come from --config
# FILE or stdin. Every run FORCES the rung (ULTRA_FORCE_WAVEFORM / _DATA_MOD /
# _DATA_RATE) so the probe pins it regardless of the auto rate ladder.
#
# Usage:
#   tools/qso_sweep.sh --out-root /tmp/sweep --config matrix.txt
#   printf 'good 15 QPSK R3/4\ngood 15 QPSK R1/2\n' | tools/qso_sweep.sh --out-root /tmp/sweep
#   tools/qso_sweep.sh --out-root /tmp/sweep <<'EOF'
#     good 15 QPSK R3/4 20
#     good 15 QPSK R2/3 20
#     good 15 QPSK R1/2 20
#   EOF
#
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SCENARIO="$HERE/gui_qso_scenario.sh"
ANALYZE="$HERE/analyze_qso_run.sh"

OUT_ROOT="/tmp/qso_sweep_$$"
SEEDS="42 2"        # one or more seeds; each spec runs once per seed.
                    # seed 2 = standing CFO-phantom guard (good/20 fade jitters the chirp peak
                    # -> phantom -1.25 Hz CFO -> group-0 smear; regression case for the
                    # confidence-into-CFOTracker fix, 2026-06-08). Keep it in the default set.
DEFAULT_FKB=20
CONFIG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-root) OUT_ROOT="$2"; shift 2 ;;
    --seed)     SEEDS="$2"; shift 2 ;;       # single seed (back-compat)
    --seeds)    SEEDS="$2"; shift 2 ;;       # e.g. --seeds "42 7 123"
    --file-kb)  DEFAULT_FKB="$2"; shift 2 ;;
    --config)   CONFIG="$2"; shift 2 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "qso_sweep: unknown arg: $1" >&2; exit 2 ;;
  esac
done

[[ -x "$SCENARIO" ]] || { echo "qso_sweep: missing $SCENARIO" >&2; exit 1; }
[[ -x "$ANALYZE"  ]] || { echo "qso_sweep: missing $ANALYZE"  >&2; exit 1; }

mkdir -p "$OUT_ROOT"
CSV="$OUT_ROOT/results.csv"
"$ANALYZE" --csv-header > "$CSV"

# Read all specs first (so stdin isn't consumed by the scenario subprocesses).
if [[ -n "$CONFIG" ]]; then SPECS="$(cat "$CONFIG")"; else SPECS="$(cat)"; fi

i=0
while read -r ch snr mod rate fkb _rest || [[ -n "${ch:-}" ]]; do
  [[ -z "${ch:-}" || "${ch:0:1}" == "#" ]] && { ch=""; continue; }
  fkb="${fkb:-$DEFAULT_FKB}"
  rate_us="${rate//\//_}"          # R3/4 -> R3_4 for the force knob
  for sd in $SEEDS; do
    i=$((i+1))
    tag="$(printf '%02d' "$i")_${ch}${snr}_${mod}_${rate_us}_${fkb}k_s${sd}"
    dir="$OUT_ROOT/$tag"
    echo ">>> [$i] $ch @ ${snr} dB  $mod $rate  ${fkb} KB  (seed $sd)" >&2

    ULTRA_RX_RATE_CMD=0 \
    ULTRA_ADAPTIVE_RATE=0 \
    ULTRA_RATE_ADAPT=0 \
    ULTRA_LOCK_RATE=1 \
    ULTRA_FORCE_WAVEFORM=OFDM_CHIRP \
    ULTRA_FORCE_DATA_MOD="$mod" \
    ULTRA_FORCE_DATA_RATE="$rate_us" \
      "$SCENARIO" --channel "$ch" --snr-db "$snr" --seed "$sd" \
                  --expect-mod "$mod" --expect-rate "$rate" \
                  --file-kb "$fkb" --out "$dir" \
                  > "$dir.scenario.log" 2>&1 || true

    if [[ -f "$dir/summary.env" ]]; then
      "$ANALYZE" --csv "$dir" >> "$CSV"
      "$ANALYZE" "$dir" 2>/dev/null | sed 's/^/    /' >&2
    else
      echo "    !! no summary.env (run crashed early — see $dir.scenario.log)" >&2
      echo "$ch,$snr,$sd,$mod,$rate,$fkb,RUN_ERROR,0,0,0,0,0,0,0,0,0,0,0,0" >> "$CSV"
    fi
  done
  ch=""
done <<< "$SPECS"

echo >&2
echo "=== SWEEP RESULTS  ($CSV) ===" >&2
column -s, -t "$CSV"
