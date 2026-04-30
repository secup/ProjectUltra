#!/usr/bin/env bash
set -euo pipefail

# Reproducible cli_simulator benchmark matrix.
#
# Runs a small deterministic AWGN/Good/Moderate matrix and emits one CSV row
# per case so future agents/humans can compare throughput and reliability
# across commits without re-running the full alpha gate.
#
# This script is evidence-only: it does not change modem behavior. Each case
# uses a fixed seed and deterministic profile so results are comparable.
#
# Output is written under ${TMPDIR:-/tmp}/ultra_bench_<timestamp> by default
# and is intentionally never committed.
#
# Usage:
#   scripts/run_bench_matrix.sh [options]
#
# Options:
#   --quick              Smoke matrix (2 cases, AWGN + Good).
#   --binary PATH        cli_simulator binary (default: ./build/cli_simulator).
#   --out-dir DIR        Output directory (default: ${TMPDIR:-/tmp}/ultra_bench_<ts>).
#   --seed N             Per-case seed (default: 42).
#   --timeout-sec N      Per-case timeout in seconds (default: 240).
#   --keep-going         Continue after a failed case instead of aborting.
#   -h, --help           Show this help.
#
# Exit codes:
#   0  All cases delivered.
#   1  At least one case failed delivery.
#   2  Bad usage / environment problem (binary missing, etc.).
#
# CSV columns (results.csv, one row per case):
#   commit, run_id, case_id, channel, snr_db, rate, modulation, waveform,
#   seed, payload_kind, payload_bytes, rc, delivery_ok, disconnect_ok,
#   frames_sent, retransmissions, timeouts, retx_timeout, retx_fast_hole,
#   retx_hole_probe, retx_nack, hole_events, acks_sent, acks_rcvd,
#   frames_decoded, frames_failed, frame_success_pct, buffer_overflows,
#   peak_backlog_ms, elapsed_sec, data_phase_bps, log_path

usage() {
  sed -n '3,40p' "${BASH_SOURCE[0]}"
}

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BINARY="${BENCH_BINARY:-$PROJECT_DIR/build/cli_simulator}"
OUT_DIR="${BENCH_OUT_DIR:-}"
SEED="${BENCH_SEED:-42}"
TIMEOUT_SEC="${BENCH_TIMEOUT_SEC:-240}"
QUICK=0
KEEP_GOING=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --quick) QUICK=1; shift ;;
    --binary) BINARY="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --seed) SEED="$2"; shift 2 ;;
    --timeout-sec) TIMEOUT_SEC="$2"; shift 2 ;;
    --keep-going) KEEP_GOING=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ ! -x "$BINARY" ]]; then
  echo "error: cli_simulator not found or not executable: $BINARY" >&2
  echo "build it first, e.g.: cmake --build build -j4" >&2
  exit 2
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "error: 'timeout' command is required (coreutils)" >&2
  exit 2
fi

TS="$(date -u +%Y%m%d_%H%M%SZ)"
if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="${TMPDIR:-/tmp}/ultra_bench_${TS}"
fi
mkdir -p "$OUT_DIR/logs"

if command -v git >/dev/null 2>&1 && git -C "$PROJECT_DIR" rev-parse --short=12 HEAD >/dev/null 2>&1; then
  COMMIT="$(git -C "$PROJECT_DIR" rev-parse --short=12 HEAD)"
  if ! git -C "$PROJECT_DIR" diff --quiet 2>/dev/null \
     || ! git -C "$PROJECT_DIR" diff --cached --quiet 2>/dev/null; then
    COMMIT="${COMMIT}-dirty"
  fi
else
  COMMIT="unknown"
fi

RUN_ID="bench_${TS}"
RESULTS_CSV="$OUT_DIR/results.csv"
SUMMARY_MD="$OUT_DIR/summary.md"

echo "commit,run_id,case_id,channel,snr_db,rate,modulation,waveform,seed,payload_kind,payload_bytes,rc,delivery_ok,disconnect_ok,frames_sent,retransmissions,timeouts,retx_timeout,retx_fast_hole,retx_hole_probe,retx_nack,hole_events,acks_sent,acks_rcvd,frames_decoded,frames_failed,frame_success_pct,buffer_overflows,peak_backlog_ms,elapsed_sec,data_phase_bps,log_path" > "$RESULTS_CSV"

# Strip ANSI escape codes for stable awk extraction.
strip_ansi() {
  # POSIX/BSD-portable: filter via sed.
  sed -E $'s/\x1B\\[[0-9;]*[A-Za-z]//g' "$1"
}

# Extract a key=value token from the ALPHA (TX) station block.
extract_alpha_key() {
  local clean="$1"
  local key="$2"
  awk -v key="$key" '
    /--- ALPHA \(TX\) ---/ { in_alpha=1; next }
    /--- BRAVO \(RX\) ---/ { in_alpha=0 }
    in_alpha {
      for (i = 1; i <= NF; i++) {
        if (index($i, key "=") == 1) {
          split($i, a, "=")
          gsub(/[^0-9.\-]/, "", a[2])
          print a[2]
          exit
        }
      }
    }
  ' "$clean"
}

# Extract a key=value token from the BRAVO (RX) station block.
extract_bravo_key() {
  local clean="$1"
  local key="$2"
  awk -v key="$key" '
    /--- BRAVO \(RX\) ---/ { in_bravo=1; next }
    /--- ALPHA \(TX\) ---/ { in_bravo=0 }
    in_bravo {
      for (i = 1; i <= NF; i++) {
        if (index($i, key "=") == 1) {
          split($i, a, "=")
          gsub(/[^0-9.\-]/, "", a[2])
          print a[2]
          exit
        }
      }
    }
  ' "$clean"
}

# Pull the data-phase bps line emitted by file-transfer mode:
#   "  Transfer: <bytes> bytes in <sec>s = <bps> bps"
extract_data_phase_bps() {
  local clean="$1"
  awk '
    /Transfer: .* bytes in .* = .* bps/ {
      for (i = 1; i <= NF; i++) {
        if ($i == "=" && i+1 <= NF) {
          val = $(i+1)
          gsub(/[^0-9.]/, "", val)
          print val
          exit
        }
      }
    }
  ' "$clean"
}

# Single case runner. Appends one CSV row to RESULTS_CSV.
# Returns 0 on delivery success, 1 on delivery failure (still records the row).
run_case() {
  local case_id="$1"
  local channel="$2"
  local snr="$3"
  local rate="$4"
  local mod="$5"
  local waveform="$6"
  local payload_kind="$7"   # message | file
  local payload_bytes="$8"  # for file mode; "" for message mode

  local log_raw="$OUT_DIR/logs/${case_id}.log"
  local log_clean="$OUT_DIR/logs/${case_id}.clean.log"

  local -a cmd=(
    "$BINARY"
    --channel "$channel"
    --snr "$snr"
    --rate "$rate"
    --mod "$mod"
    --waveform "$waveform"
    --seed "$SEED"
  )
  if [[ "$payload_kind" == "file" ]]; then
    cmd+=(--file "$payload_bytes")
  fi

  echo "[case:$case_id] channel=$channel snr=$snr rate=$rate mod=$mod wf=$waveform payload=$payload_kind${payload_bytes:+/$payload_bytes}"
  echo "  cmd: ${cmd[*]}"

  local start_ns end_ns elapsed_sec rc=0
  start_ns=$(date +%s%N)
  if ! timeout "$TIMEOUT_SEC" "${cmd[@]}" >"$log_raw" 2>&1; then
    rc=$?
  fi
  end_ns=$(date +%s%N)
  elapsed_sec=$(awk -v s="$start_ns" -v e="$end_ns" 'BEGIN { printf "%.2f", (e - s) / 1e9 }')

  strip_ansi "$log_raw" > "$log_clean"

  local delivery_ok=0
  if [[ "$payload_kind" == "file" ]]; then
    if grep -q "File contents verified!" "$log_clean"; then
      delivery_ok=1
    fi
  else
    if grep -q "All 7 messages transferred successfully!" "$log_clean"; then
      delivery_ok=1
    fi
  fi

  local disconnect_ok=0
  if grep -q "Disconnect timeout (non-fatal)" "$log_clean"; then
    disconnect_ok=0
  elif grep -q "Disconnected!" "$log_clean"; then
    disconnect_ok=1
  fi

  local frames_sent retrans timeouts
  local retx_timeout retx_fast_hole retx_hole_probe retx_nack hole_events
  local acks_sent acks_rcvd
  local frames_decoded frames_failed
  local buffer_overflows peak_backlog_ms
  local frame_success_pct data_phase_bps

  frames_sent="$(extract_alpha_key "$log_clean" "frames_sent")"
  retrans="$(extract_alpha_key "$log_clean" "retransmissions")"
  timeouts="$(extract_alpha_key "$log_clean" "timeouts")"
  retx_timeout="$(extract_alpha_key "$log_clean" "timeout")"
  retx_fast_hole="$(extract_alpha_key "$log_clean" "fast_hole")"
  retx_hole_probe="$(extract_alpha_key "$log_clean" "hole_probe")"
  retx_nack="$(extract_alpha_key "$log_clean" "nack")"
  hole_events="$(extract_alpha_key "$log_clean" "hole_events")"
  acks_sent="$(extract_alpha_key "$log_clean" "acks_sent")"
  acks_rcvd="$(extract_alpha_key "$log_clean" "acks_rcvd")"

  frames_decoded="$(extract_bravo_key "$log_clean" "frames_decoded")"
  frames_failed="$(extract_bravo_key "$log_clean" "frames_failed")"
  buffer_overflows="$(extract_bravo_key "$log_clean" "overflows")"
  peak_backlog_ms="$(extract_bravo_key "$log_clean" "peak_backlog_ms")"

  # frame_success is printed under BRAVO as "Rate: frame_success=NN.N%"
  frame_success_pct="$(awk '
    /--- BRAVO \(RX\) ---/ { in_b=1; next }
    /--- ALPHA \(TX\) ---/ { in_b=0 }
    in_b && /frame_success=/ {
      for (i = 1; i <= NF; i++) {
        if (index($i, "frame_success=") == 1) {
          split($i, a, "=")
          gsub(/[^0-9.]/, "", a[2])
          print a[2]
          exit
        }
      }
    }
  ' "$log_clean")"

  data_phase_bps="$(extract_data_phase_bps "$log_clean")"

  frames_sent="${frames_sent:-0}"
  retrans="${retrans:-0}"
  timeouts="${timeouts:-0}"
  retx_timeout="${retx_timeout:-0}"
  retx_fast_hole="${retx_fast_hole:-0}"
  retx_hole_probe="${retx_hole_probe:-0}"
  retx_nack="${retx_nack:-0}"
  hole_events="${hole_events:-0}"
  acks_sent="${acks_sent:-0}"
  acks_rcvd="${acks_rcvd:-0}"
  frames_decoded="${frames_decoded:-0}"
  frames_failed="${frames_failed:-0}"
  buffer_overflows="${buffer_overflows:-0}"
  peak_backlog_ms="${peak_backlog_ms:-0}"
  frame_success_pct="${frame_success_pct:-}"
  data_phase_bps="${data_phase_bps:-}"

  local payload_bytes_csv="${payload_bytes:-}"

  printf '%s\n' \
    "$COMMIT,$RUN_ID,$case_id,$channel,$snr,$rate,$mod,$waveform,$SEED,$payload_kind,$payload_bytes_csv,$rc,$delivery_ok,$disconnect_ok,$frames_sent,$retrans,$timeouts,$retx_timeout,$retx_fast_hole,$retx_hole_probe,$retx_nack,$hole_events,$acks_sent,$acks_rcvd,$frames_decoded,$frames_failed,$frame_success_pct,$buffer_overflows,$peak_backlog_ms,$elapsed_sec,$data_phase_bps,$log_raw" \
    >> "$RESULTS_CSV"

  if [[ "$delivery_ok" == "1" ]]; then
    echo "  PASS rc=$rc delivery_ok=1 elapsed=${elapsed_sec}s frames_sent=$frames_sent retx=$retrans timeouts=$timeouts frame_success=${frame_success_pct:-?}%${data_phase_bps:+ data_bps=$data_phase_bps}"
    return 0
  else
    echo "  FAIL rc=$rc delivery_ok=0 elapsed=${elapsed_sec}s log=$log_raw" >&2
    return 1
  fi
}

# Matrix definitions. Each entry is space-separated:
#   case_id channel snr rate modulation waveform payload_kind payload_bytes
#
# Quick is the maintained smoke matrix: one AWGN baseline plus one Good-fading
# baseline. Default matrix expands coverage to Moderate fading and a small file
# transfer to capture data-phase bps.
#
# Keep cases small. This script must complete in a reasonable time on CI.
quick_matrix=(
  "awgn_snr20_r1_2_msg awgn 20 r1_2 dqpsk ofdm_chirp message ''"
  "good_snr15_r1_2_msg good 15 r1_2 dqpsk ofdm_chirp message ''"
)
default_matrix=(
  "awgn_snr20_r1_2_msg      awgn     20 r1_2 dqpsk ofdm_chirp message ''"
  "awgn_snr20_r2_3_msg      awgn     20 r2_3 dqpsk ofdm_chirp message ''"
  "good_snr15_r1_2_msg      good     15 r1_2 dqpsk ofdm_chirp message ''"
  "good_snr15_r1_4_msg      good     15 r1_4 dqpsk ofdm_chirp message ''"
  "moderate_snr15_r1_4_msg  moderate 15 r1_4 dqpsk ofdm_chirp message ''"
  "awgn_snr20_r1_2_file1k   awgn     20 r1_2 dqpsk ofdm_chirp file    1024"
)

if [[ "$QUICK" == "1" ]]; then
  matrix=("${quick_matrix[@]}")
  mode_label="quick"
else
  matrix=("${default_matrix[@]}")
  mode_label="default"
fi

echo "ProjectUltra bench matrix"
echo "  commit:  $COMMIT"
echo "  run_id:  $RUN_ID"
echo "  binary:  $BINARY"
echo "  out_dir: $OUT_DIR"
echo "  mode:    $mode_label (${#matrix[@]} cases, seed=$SEED, timeout=${TIMEOUT_SEC}s)"
echo

failed_cases=()
for entry in "${matrix[@]}"; do
  # shellcheck disable=SC2086
  set -- $entry
  case_id="$1"; channel="$2"; snr="$3"; rate="$4"
  mod="$5"; waveform="$6"; payload_kind="$7"; payload_bytes="$8"
  if [[ "$payload_bytes" == "''" ]]; then payload_bytes=""; fi

  if ! run_case "$case_id" "$channel" "$snr" "$rate" "$mod" "$waveform" "$payload_kind" "$payload_bytes"; then
    failed_cases+=("$case_id")
    if [[ "$KEEP_GOING" != "1" ]]; then
      break
    fi
  fi
done

# Markdown summary so a human can paste it into a PR/report.
{
  echo "# bench matrix ($mode_label)"
  echo
  echo "- commit: \`$COMMIT\`"
  echo "- run_id: \`$RUN_ID\`"
  echo "- binary: \`$BINARY\`"
  echo "- out_dir: \`$OUT_DIR\`"
  echo "- seed: \`$SEED\`"
  echo "- cases: ${#matrix[@]}"
  echo "- failed: ${#failed_cases[@]}"
  echo
  echo "| case | channel | snr | rate | delivery | retx | timeouts | frame_success% | elapsed_s | data_bps |"
  echo "|------|---------|-----|------|----------|------|----------|----------------|-----------|----------|"
  awk -F, 'NR>1 {
    delivery = ($13 == 1) ? "ok" : "FAIL"
    printf "| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |\n",
      $3, $4, $5, $6, delivery, $16, $17, $27, $30, $31
  }' "$RESULTS_CSV"
  echo
  echo "Full CSV: \`$RESULTS_CSV\`"
} > "$SUMMARY_MD"

echo
echo "results: $RESULTS_CSV"
echo "summary: $SUMMARY_MD"

if [[ "${#failed_cases[@]}" -gt 0 ]]; then
  echo "FAIL: ${#failed_cases[@]} case(s) did not deliver: ${failed_cases[*]}" >&2
  exit 1
fi

echo "PASS: all ${#matrix[@]} cases delivered."
