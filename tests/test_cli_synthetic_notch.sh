#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 /path/to/cli_simulator" >&2
  exit 2
fi

CLI_BIN="$1"
LOG_FILE="$(mktemp "${TMPDIR:-/tmp}/ultra_cli_notch.XXXXXX.log")"
trap 'rm -f "$LOG_FILE"' EXIT

set +e
"$CLI_BIN" --snr 15 --fading good --rate r1_2 \
  --mask-clear-carrier 17 --test >"$LOG_FILE" 2>&1
cli_status=$?
set -e

if (( cli_status != 0 )); then
  echo "cli_simulator failed in synthetic-notch OTASim regression (exit=$cli_status)" >&2
  tail -120 "$LOG_FILE" >&2
  exit "$cli_status"
fi

retx="$(
  sed -n '/--- ALPHA (TX) ---/,/--- BRAVO (RX) ---/p' "$LOG_FILE" |
    sed -n 's/.*retransmissions=\([0-9][0-9]*\).*/\1/p' |
    head -1
)"

if [[ -z "$retx" ]]; then
  echo "failed to parse ALPHA retransmissions" >&2
  tail -80 "$LOG_FILE" >&2
  exit 1
fi

max_retx=16

if (( retx > max_retx )); then
  echo "synthetic-notch OTASim regression: retransmissions=$retx > $max_retx" >&2
  tail -120 "$LOG_FILE" >&2
  exit 1
fi

echo "synthetic-notch OTASim regression passed: retransmissions=$retx <= $max_retx"
