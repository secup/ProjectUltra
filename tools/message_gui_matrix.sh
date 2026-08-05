#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Automated GUI matrix for the restored message system.
#
# Each row is two REAL ultra_gui -sim stations over a live ota_simulator channel
# (the project's faithful full-protocol gate), verifying byte-exact application
# text plus terminal DELIVERED status — not merely "did not crash".
#
# Rows are chosen to target what the branch actually CHANGED:
#   frag*   message fragmentation across frames (the silent-truncation fix)
#   turn*   DATA-turn reversal (initiator -> responder reply)
#   lowsnr* fragmentation under retransmission, and the reachability case
#   regress file path must be UNAFFECTED (connection.cpp changed by ~505 lines)
#
# NOT covered here, by construction: message+file in one session. The gate
# refuses to mix them ("Message scenario (never mixed with a file)"), so the
# cross-class FIFO / BUG-FILE-ACK-IDENTITY fix has unit coverage only.
#
# WHY THIS EXISTS: on 2026-08-05 a sender-side message-resume change passed the
# full 101/101 ctest suite while DUPLICATING a delivered message. Only this matrix
# caught it (msg_rx > msg_tx on rows m3/m4). Unit tests do not exercise real
# fragmentation across a live adaptive demote; run this before touching the
# message/ARQ/geometry paths.
#
# Rows m3/m4 can expose BUG-MESSAGE-LOST-ON-FORCED-DEMOTE, but NOT
# deterministically: a code change shifts the timeline, so a green run is not
# proof the defect is gone. Check the `msg_tx == msg_rx == msg_delivered` and
# `exact=1` columns, and treat msg_rx > msg_tx as duplicate delivery.
# ─────────────────────────────────────────────────────────────────────────────
set -u
cd "$(dirname "$0")/.." || exit 1
OUT=${OUT:-/tmp/msg_matrix}
mkdir -p "$OUT"
RES="$OUT/matrix.csv"
echo "id,scenario,result,reason,elapsed,mode,msg_tx,msg_rx,msg_delivered,exact,ordered,reply_tx,reply_rx,reply_exact,retx,crc_ok" > "$RES"

run() {
  local id="$1"; shift
  local desc="$1"; shift
  local dir="$OUT/$id"
  echo "── [$id] $desc"
  bash tools/gui_qso_scenario.sh "$@" --out "$dir" > "$OUT/${id}.out" 2>&1
  local s="$dir/summary.env"
  if [ ! -f "$s" ]; then
    echo "$id,\"$desc\",NO_SUMMARY,harness_fault,,,,,,,,,,,," >> "$RES"
    echo "   NO SUMMARY (harness fault)"; return
  fi
  # shellcheck disable=SC1090
  ( set -a; . "$s"; set +a
    printf '%s,"%s",%s,%s,%s,"%s",%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
      "$id" "$desc" "${RESULT:-?}" "${REASON:-?}" "${ELAPSED_SEC:-}" "${ACTUAL_DATA_MODE:-}" \
      "${MESSAGE_TX_COUNT:-0}" "${MESSAGE_RX_COUNT:-0}" "${MESSAGE_DELIVERED_COUNT:-0}" \
      "${MESSAGE_EXACT_MATCH:-0}" "${MESSAGE_NUMBERS_ORDERED:-0}" \
      "${REPLY_TX_COUNT:-0}" "${REPLY_RX_COUNT:-0}" "${REPLY_EXACT_MATCH:-0}" \
      "$(( ${ALPHA_RETX_COUNT:-0} + ${BRAVO_RETX_COUNT:-0} ))" "${FILE_CRC_OK_COUNT:-0}" >> "$RES"
    echo "   RESULT=${RESULT:-?} (${REASON:-?}) ${ELAPSED_SEC:-?}s mode=${ACTUAL_DATA_MODE:-?} tx=${MESSAGE_TX_COUNT:-0} rx=${MESSAGE_RX_COUNT:-0} exact=${MESSAGE_EXACT_MATCH:-0}"
  )
}

# ── baseline ────────────────────────────────────────────────────────────────
run m1_single_awgn20 "1 msg, AWGN@20" \
    --channel awgn --snr-db 20 --seed 42 --message-only --message "hello" --exit-after 240

# ── fragmentation + ordering ────────────────────────────────────────────────
run m2_frag_good20 "3 msgs vary-len, Good@20" \
    --channel good --snr-db 20 --seed 7 --message-only --message ProjectUltra \
    --message-count 3 --message-vary-len --exit-after 400

run m3_frag_many_good15 "5 msgs vary-len, Good@15" \
    --channel good --snr-db 15 --seed 11 --message-only --message StatusReport \
    --message-count 5 --message-vary-len --exit-after 500

# ── DATA-turn reversal (bidirectional) ──────────────────────────────────────
run m4_turn_good20 "3 msgs + reply, Good@20" \
    --channel good --snr-db 20 --seed 23 --message-only --message ProjectUltra \
    --message-count 3 --message-vary-len --reply-message "roger" --exit-after 400

run m5_turn_awgn20 "2 msgs + reply, AWGN@20" \
    --channel awgn --snr-db 20 --seed 5 --message-only --message QSL \
    --message-count 2 --reply-message "received all" --exit-after 350

# ── low SNR: fragmentation under retx, and reachability ─────────────────────
run m6_lowsnr_good10 "3 msgs vary-len, Good@10" \
    --channel good --snr-db 10 --seed 13 --message-only --message Weather \
    --message-count 3 --message-vary-len --exit-after 500

run m7_floor_awgn8 "2 msgs + reply, AWGN@8" \
    --channel awgn --snr-db 8 --seed 17 --message-only --message Ping \
    --message-count 2 --reply-message "ack" --exit-after 500

# ── heavy multipath (should fall to MC-DPSK) ────────────────────────────────
run m8_poor20 "3 msgs, Poor@20" \
    --channel poor --snr-db 20 --seed 3 --message-only --message Relay \
    --message-count 3 --exit-after 500

# ── REGRESSION: the file path must be unaffected ────────────────────────────
run f1_file_good20 "REGRESSION file 21KB, Good@20" \
    --channel good --snr-db 20 --seed 42 --file-kb 21 --exit-after 500

run f2_file_awgn20 "REGRESSION file 10KB, AWGN@20" \
    --channel awgn --snr-db 20 --seed 42 --file-kb 10 --exit-after 400

echo
echo "DONE -> $RES"
column -s, -t "$RES" 2>/dev/null || cat "$RES"
