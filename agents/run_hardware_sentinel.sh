#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

timestamp=$(date +%Y%m%d_%H%M%S)
MODE=${AGENT_HW_SENTINEL_MODE:-quick}  # quick|nightly|full
DRY_RUN=${AGENT_HW_SENTINEL_DRY_RUN:-0}
REPORT_DIR=${AGENT_HW_SENTINEL_REPORT_DIR:-agents/reports/hardware_sentinel_$timestamp}
LOCK_DIR=${AGENT_HW_LOCK_DIR:-/tmp/projectultra_hw.lock}
SSH_KEY=${SSH_KEY:-$HOME/.ssh/id_pi5}
INJECT_GAIN=${AGENT_HW_INJECT_GAIN:-0.70}
EXIT_NONZERO=${AGENT_HW_SENTINEL_EXIT_NONZERO:-1}
STOP_ON_FAIL=${AGENT_HW_SENTINEL_STOP_ON_FAIL:-0}
INCLUDE_MODERATE=${AGENT_HW_SENTINEL_INCLUDE_MODERATE:-0}
NIGHTLY_AWGN=${AGENT_HW_SENTINEL_NIGHTLY_AWGN:-1}
MAX_RETX_QUICK=${AGENT_HW_SENTINEL_MAX_RETX_QUICK:-2}
MAX_RETX_LONG=${AGENT_HW_SENTINEL_MAX_RETX_LONG:-20}
MAX_TIMEOUTS_QUICK=${AGENT_HW_SENTINEL_MAX_TIMEOUTS_QUICK:-1}
MAX_TIMEOUTS_LONG=${AGENT_HW_SENTINEL_MAX_TIMEOUTS_LONG:-10}
AWGN_QUICK_REPEATS=${AGENT_HW_SENTINEL_AWGN_QUICK_REPEATS:-3}
MAX_RETX_AWGN_QUICK=${AGENT_HW_SENTINEL_MAX_RETX_AWGN_QUICK:-0}
MAX_TIMEOUTS_AWGN_QUICK=${AGENT_HW_SENTINEL_MAX_TIMEOUTS_AWGN_QUICK:-0}

if [[ -n "${AGENT_HW_SENTINEL_AUDIO_CHECK:-}" ]]; then
  RUN_AUDIO_CHECK=$AGENT_HW_SENTINEL_AUDIO_CHECK
elif [[ "$(uname -s)" == "Darwin" ]]; then
  RUN_AUDIO_CHECK=1
else
  # tools/check_hw_audio_path.sh is currently macOS-specific.
  RUN_AUDIO_CHECK=0
fi

mkdir -p "$REPORT_DIR"

if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  echo "Hardware lock is held: $LOCK_DIR" >&2
  exit 75
fi
trap 'rmdir "$LOCK_DIR" 2>/dev/null || true' EXIT

metrics_file="$REPORT_DIR/metrics.tsv"
printf 'case\tstatus\trc\tchannel\tsnr\trate\tbytes\tretransmissions\ttimeouts\tframe_success_pct\tdata_phase_bps\tpeak_backlog_ms\thw_log_dir\twrapper_log\twarnings\n' \
  > "$metrics_file"

case_count=0
fail_count=0
warn_count=0

extract_metric() {
  local file="$1"
  local key="$2"

  [[ -f "$file" ]] || return 0
  grep -Eo "${key}=[^[:space:],)]+" "$file" 2>/dev/null \
    | tail -1 \
    | cut -d= -f2 \
    | tr -d '%' || true
}

first_hw_log_dir() {
  local file="$1"

  awk '/^Logs: \/tmp\/ultra_hw_/ {print $2; exit}' "$file" 2>/dev/null || true
}

maybe_warn_numeric_gt() {
  local value="$1"
  local limit="$2"
  local label="$3"

  [[ -n "$value" ]] || return 0
  [[ "$value" =~ ^[0-9]+$ ]] || return 0
  if (( value > limit )); then
    printf '%s>%s ' "$label" "$limit"
  fi
}

record_row() {
  local name="$1"
  local status="$2"
  local rc="$3"
  local channel="$4"
  local snr="$5"
  local rate="$6"
  local bytes="$7"
  local hw_log_dir="$8"
  local wrapper_log="$9"
  local warnings="${10}"

  local a_log=""
  [[ -n "$hw_log_dir" ]] && a_log="$hw_log_dir/A.log"
  local retx timeouts frame_success data_phase_bps peak_backlog_ms
  retx=$(extract_metric "$a_log" "retransmissions")
  timeouts=$(extract_metric "$a_log" "timeouts")
  frame_success=$(extract_metric "$a_log" "frame_success")
  data_phase_bps=$(extract_metric "$a_log" "data_phase_bps")
  peak_backlog_ms=$(extract_metric "$a_log" "peak_backlog_ms")

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$name" "$status" "$rc" "$channel" "$snr" "$rate" "$bytes" \
    "${retx:-}" "${timeouts:-}" "${frame_success:-}" "${data_phase_bps:-}" \
    "${peak_backlog_ms:-}" "${hw_log_dir:-}" "$wrapper_log" "$warnings" \
    >> "$metrics_file"
}

run_named_step() {
  local name="$1"
  shift

  local log="$REPORT_DIR/$name.log"
  echo "==> $name"
  printf '+ ' > "$log"
  printf '%q ' "$@" >> "$log"
  printf '\n' >> "$log"

  set +e
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "dry-run: command not executed" >> "$log"
    local rc=0
  else
    "$@" >> "$log" 2>&1
    local rc=$?
  fi
  set -e

  if [[ "$rc" -ne 0 ]]; then
    fail_count=$((fail_count + 1))
    echo "FAIL $name rc=$rc (log: $log)" >&2
    tail -80 "$log" >&2 || true
    [[ "$STOP_ON_FAIL" == "1" ]] && exit "$rc"
  else
    echo "PASS $name"
  fi

  return 0
}

run_hw_case() {
  local name="$1"
  local bytes="$2"
  local channel="$3"
  local snr="$4"
  local rate="$5"

  case_count=$((case_count + 1))
  local log="$REPORT_DIR/$name.log"

  echo "==> $name"
  {
    echo "case=$name"
    echo "bytes=$bytes"
    echo "channel=$channel"
    echo "snr=$snr"
    echo "rate=$rate"
    echo "inject_gain=$INJECT_GAIN"
  } > "$log"

  set +e
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "dry-run: hardware case not executed" >> "$log"
    local rc=0
  else
    env SSH_KEY="$SSH_KEY" ./tools/run_hw_test.sh \
      --file "$bytes" \
      --rate "$rate" \
      --snr "$snr" \
      --channel "$channel" \
      --inject \
      --inject-gain "$INJECT_GAIN" \
      >> "$log" 2>&1
    local rc=$?
  fi
  set -e

  local status="pass"
  if [[ "$rc" -ne 0 ]]; then
    status="fail"
    fail_count=$((fail_count + 1))
    echo "FAIL $name rc=$rc (log: $log)" >&2
    tail -80 "$log" >&2 || true
  else
    echo "PASS $name"
  fi

  local hw_log_dir warnings retx timeouts max_retx max_timeouts strict_metrics
  hw_log_dir=$(first_hw_log_dir "$log")
  retx=""
  timeouts=""
  strict_metrics=0
  if [[ -n "$hw_log_dir" ]]; then
    retx=$(extract_metric "$hw_log_dir/A.log" "retransmissions")
    timeouts=$(extract_metric "$hw_log_dir/A.log" "timeouts")
  fi

  if [[ "$channel" == "awgn" ]] && (( bytes < 20000 )); then
    max_retx=$MAX_RETX_AWGN_QUICK
    max_timeouts=$MAX_TIMEOUTS_AWGN_QUICK
    strict_metrics=1
  elif (( bytes >= 20000 )); then
    max_retx=$MAX_RETX_LONG
    max_timeouts=$MAX_TIMEOUTS_LONG
  else
    max_retx=$MAX_RETX_QUICK
    max_timeouts=$MAX_TIMEOUTS_QUICK
  fi

  warnings="$(maybe_warn_numeric_gt "${retx:-}" "$max_retx" "retx")"
  warnings+="$(maybe_warn_numeric_gt "${timeouts:-}" "$max_timeouts" "timeouts")"
  if [[ -n "$warnings" ]]; then
    if [[ "$strict_metrics" == "1" ]]; then
      if [[ "$status" != "fail" ]]; then
        fail_count=$((fail_count + 1))
      fi
      status="fail"
      echo "FAIL $name strict metrics: $warnings(log: $log)" >&2
      [[ "$STOP_ON_FAIL" == "1" ]] && exit 1
    else
      warn_count=$((warn_count + 1))
      [[ "$status" == "pass" ]] && status="warn"
    fi
  fi

  record_row "$name" "$status" "$rc" "$channel" "$snr" "$rate" "$bytes" \
    "$hw_log_dir" "$log" "$warnings"

  if [[ "$rc" -ne 0 && "$STOP_ON_FAIL" == "1" ]]; then
    exit "$rc"
  fi
}

echo "Hardware sentinel mode=$MODE report=$REPORT_DIR"

if [[ "$RUN_AUDIO_CHECK" == "1" ]]; then
  run_named_step audio_path env SSH_KEY="$SSH_KEY" ./tools/check_hw_audio_path.sh
else
  echo "Skipping audio_path check (AGENT_HW_SENTINEL_AUDIO_CHECK=$RUN_AUDIO_CHECK)"
fi

if (( AWGN_QUICK_REPEATS < 1 )); then
  AWGN_QUICK_REPEATS=1
fi

run_hw_case awgn_1k_r12_snr15 1024 awgn 15 r1_2
for ((rep = 2; rep <= AWGN_QUICK_REPEATS; rep++)); do
  run_hw_case "awgn_1k_r12_snr15_rep${rep}" 1024 awgn 15 r1_2
done

run_hw_case good_1k_r12_snr15 1024 good 15 r1_2

if [[ "$INCLUDE_MODERATE" == "1" || "$MODE" == "full" ]]; then
  run_hw_case moderate_1k_r12_snr15 1024 moderate 15 r1_2
fi

if [[ "$MODE" == "nightly" || "$MODE" == "full" ]]; then
  if [[ "$NIGHTLY_AWGN" == "1" ]]; then
    run_hw_case awgn_20k_r12_snr15 20480 awgn 15 r1_2
  fi
  run_hw_case good_20k_r12_snr15 20480 good 15 r1_2
fi

if [[ "$MODE" == "full" ]]; then
  run_hw_case moderate_20k_r12_snr15 20480 moderate 15 r1_2
fi

overall=pass
if [[ "$fail_count" -gt 0 ]]; then
  overall=fail
elif [[ "$warn_count" -gt 0 ]]; then
  overall=warn
fi

head_sha=$(git rev-parse --short HEAD)
cat > "$REPORT_DIR/summary.txt" <<EOF
hardware_sentinel=$overall
mode=$MODE
timestamp=$timestamp
head=$head_sha
inject_gain=$INJECT_GAIN
audio_check=$RUN_AUDIO_CHECK
dry_run=$DRY_RUN
awgn_quick_repeats=$AWGN_QUICK_REPEATS
max_retx_awgn_quick=$MAX_RETX_AWGN_QUICK
max_timeouts_awgn_quick=$MAX_TIMEOUTS_AWGN_QUICK
cases=$case_count
failures=$fail_count
warnings=$warn_count
metrics=$metrics_file
EOF

echo "Hardware sentinel overall=$overall. Reports: $REPORT_DIR"

if [[ "$EXIT_NONZERO" == "1" && "$fail_count" -gt 0 ]]; then
  exit 1
fi
