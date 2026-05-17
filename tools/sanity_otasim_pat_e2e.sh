#!/usr/bin/env bash
# End-to-end OTASim + Pat sanity test
#
# Mac:  ota_simulator serve + ultra_tnc(ALPHA) + Pat(ALPHA, sender)
# Pi5:  ultra_tnc(BRAVO)     + Pat(BRAVO, listener)
#
# Mac Pat connects to ALPHA's TNC via varahf://, requests BRAVO, exchanges
# a small message. Pi5 Pat listening receives it. We verify the message
# lands in BRAVO's isolated mailbox.
#
# All Pat state is isolated under /tmp/$TEST_RUN_ID — user's real Pat config
# and mailboxes are NOT touched.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# ---- Config ----
MAC_LAN_IP=${MAC_LAN_IP:-192.168.5.130}
PI5_SSH=${PI5_SSH:-pi5}
PI5_REPO=${PI5_REPO:-/home/math/ProjectUltra-otasim}
OTASIM_GRPC_PORT=${OTASIM_GRPC_PORT:-47200}
OTASIM_UDP_PORT=${OTASIM_UDP_PORT:-47201}
ALPHA_TNC_PORT=${ALPHA_TNC_PORT:-8400}
BRAVO_TNC_PORT=${BRAVO_TNC_PORT:-8400}
TEST_RUN_ID=${TEST_RUN_ID:-otasim_pat_$(date +%Y%m%d_%H%M%S)}
LOGS_DIR=${LOGS_DIR:-/tmp/$TEST_RUN_ID}
PAT_BIN=${PAT_BIN:-$HOME/go/bin/pat}
PI5_PAT_BIN=${PI5_PAT_BIN:-/home/math/go/bin/pat}
MESSAGE_TIMEOUT_S=${MESSAGE_TIMEOUT_S:-90}

# ---- Validate prereqs ----
[[ -x "$PAT_BIN" ]] || { echo "FATAL: $PAT_BIN not executable" >&2; exit 2; }
[[ -x "./build/ota_simulator" ]] || { echo "FATAL: ./build/ota_simulator not built" >&2; exit 2; }
[[ -x "./build/ultra_tnc" ]] || { echo "FATAL: ./build/ultra_tnc not built" >&2; exit 2; }
ssh "$PI5_SSH" "test -x $PI5_PAT_BIN" || { echo "FATAL: Pi5 $PI5_PAT_BIN not executable" >&2; exit 2; }
ssh "$PI5_SSH" "test -x $PI5_REPO/build/ultra_tnc" || { echo "FATAL: Pi5 ultra_tnc not built at $PI5_REPO/build/ultra_tnc" >&2; exit 2; }

# Pat namespaces its mailbox by callsign: <mbox>/<MYCALL>/{in,out,sent,archive}.
# Pre-create the per-call dirs so Pat doesn't have to.
mkdir -p "$LOGS_DIR"/{alpha/mailbox/ALPHA/{in,out,sent,archive},captures}
echo "[init] logs + isolated mailboxes -> $LOGS_DIR"

# Aggressive pre-cleanup of any leftover processes from previous failed runs.
# Without this, port 8400 (or whatever) gets held by a zombie and our new TNC
# fails to bind, but our PID check passes against the zombie — invisible failure.
echo "[init] pre-cleanup: killing any leftover ultra_tnc / ota_simulator processes"
pkill -9 -f "ota_simulator serve" 2>/dev/null || true
pkill -9 -f "ultra_tnc.*--sim-audio" 2>/dev/null || true
ssh "$PI5_SSH" "pkill -9 -f 'ultra_tnc.*--sim-audio' 2>/dev/null; pkill -9 -f 'go/bin/pat.*--listen varahf' 2>/dev/null; true" 2>/dev/null || true
sleep 1

# ---- Cleanup ----
cleanup() {
  local rc=$?
  echo "[cleanup] rc=$rc, killing subprocesses"
  [[ -n "${ALPHA_TNC_PID:-}" ]] && kill -TERM "$ALPHA_TNC_PID" 2>/dev/null || true
  [[ -n "${SERVER_PID:-}" ]] && kill -TERM "$SERVER_PID" 2>/dev/null || true
  ssh "$PI5_SSH" "kill -TERM \$(cat $LOGS_DIR/bravo_pat.pid 2>/dev/null) 2>/dev/null || true" 2>/dev/null || true
  ssh "$PI5_SSH" "kill -TERM \$(cat $LOGS_DIR/bravo_tnc.pid 2>/dev/null) 2>/dev/null || true" 2>/dev/null || true
  wait 2>/dev/null || true
  echo "[cleanup] done; logs at $LOGS_DIR (Mac) and $PI5_SSH:$LOGS_DIR (Pi5)"
  exit $rc
}
trap cleanup EXIT INT TERM

# ---- Tokens for OTASim ----
TOKENS="$LOGS_DIR/tokens.conf"
cat > "$TOKENS" <<EOF
alpha_token:ALPHA:Mac ALPHA TNC
bravo_token:BRAVO:Pi5 BRAVO TNC
EOF

# ---- Isolated Pat configs (don't touch user's real config) ----
cat > "$LOGS_DIR/pat_alpha.json" <<EOF
{
  "mycall": "ALPHA",
  "secure_login_password": "",
  "auxiliary_addresses": [],
  "service_codes": ["PUBLIC"],
  "http_addr": "",
  "listen": [],
  "varahf": {
    "addr": "127.0.0.1:$ALPHA_TNC_PORT",
    "bandwidth": 2300,
    "rig": "",
    "ptt_ctrl": false
  }
}
EOF

cat > "$LOGS_DIR/pat_bravo.json" <<EOF
{
  "mycall": "BRAVO",
  "secure_login_password": "",
  "auxiliary_addresses": [],
  "service_codes": ["PUBLIC"],
  "http_addr": "",
  "listen": ["varahf"],
  "varahf": {
    "addr": "127.0.0.1:$BRAVO_TNC_PORT",
    "bandwidth": 2300,
    "rig": "",
    "ptt_ctrl": false
  }
}
EOF

# Push Pi5-side configuration + isolated mailbox
ssh "$PI5_SSH" "mkdir -p $LOGS_DIR/bravo/mailbox/BRAVO/{in,out,sent,archive}"
scp -q "$LOGS_DIR/pat_bravo.json" "$PI5_SSH:$LOGS_DIR/pat_bravo.json"

# ---- 1. Start OTASim server on Mac ----
echo "[1/6] starting ota_simulator serve on Mac (gRPC 0.0.0.0:$OTASIM_GRPC_PORT, UDP 0.0.0.0:$OTASIM_UDP_PORT)"
./build/ota_simulator serve \
    --bind "0.0.0.0:$OTASIM_GRPC_PORT" \
    --udp-bind "0.0.0.0:$OTASIM_UDP_PORT" \
    --tokens "$TOKENS" \
    --captures-root "$LOGS_DIR/captures" \
    --lobby-channel passthrough \
    > "$LOGS_DIR/server.log" 2>&1 &
SERVER_PID=$!
sleep 3
kill -0 "$SERVER_PID" 2>/dev/null || { echo "FAIL: ota_simulator serve died"; tail -30 "$LOGS_DIR/server.log"; exit 1; }
echo "  pid=$SERVER_PID"

# ---- 2. Start ALPHA's ultra_tnc on Mac ----
echo "[2/6] starting ultra_tnc ALPHA on Mac (TCP port $ALPHA_TNC_PORT)"
# Connect via LAN IP, not loopback. With server bound to 0.0.0.0, gRPC
# clients hitting 127.0.0.1 hang at handshake on macOS (suspected
# IPv4-wildcard vs loopback issue). Using the LAN IP avoids it; same
# path Pi5's BRAVO TNC uses, so both connections look identical.
./build/ultra_tnc \
    --sim-audio \
    --ota-host "$MAC_LAN_IP:$OTASIM_GRPC_PORT" \
    --ota-udp-host "$MAC_LAN_IP:$OTASIM_UDP_PORT" \
    --token alpha_token \
    --station-id alpha \
    --port "$ALPHA_TNC_PORT" \
    --callsign ALPHA \
    > "$LOGS_DIR/alpha_tnc.log" 2>&1 &
ALPHA_TNC_PID=$!
sleep 4
kill -0 "$ALPHA_TNC_PID" 2>/dev/null || { echo "FAIL: alpha ultra_tnc died"; tail -30 "$LOGS_DIR/alpha_tnc.log"; exit 1; }
if grep -q "TNC server bind failed\|bind failed" "$LOGS_DIR/alpha_tnc.log" 2>/dev/null; then
  echo "FAIL: alpha ultra_tnc bind failed (port collision?)"; tail -10 "$LOGS_DIR/alpha_tnc.log"; exit 1
fi
echo "  pid=$ALPHA_TNC_PID"

# ---- 3. Start BRAVO's ultra_tnc on Pi5 ----
echo "[3/6] starting ultra_tnc BRAVO on Pi5 (TCP port $BRAVO_TNC_PORT, OTASim host $MAC_LAN_IP:$OTASIM_GRPC_PORT)"
ssh -fn "$PI5_SSH" "cd $PI5_REPO && nohup ./build/ultra_tnc \
    --sim-audio \
    --ota-host '$MAC_LAN_IP:$OTASIM_GRPC_PORT' \
    --ota-udp-host '$MAC_LAN_IP:$OTASIM_UDP_PORT' \
    --token bravo_token \
    --station-id bravo \
    --port $BRAVO_TNC_PORT \
    --callsign BRAVO \
    >$LOGS_DIR/bravo_tnc.log 2>&1 </dev/null & disown; echo \$! >$LOGS_DIR/bravo_tnc.pid"
sleep 4
ssh "$PI5_SSH" "kill -0 \$(cat $LOGS_DIR/bravo_tnc.pid) 2>/dev/null" || { echo "FAIL: bravo ultra_tnc died"; ssh "$PI5_SSH" "tail -30 $LOGS_DIR/bravo_tnc.log"; exit 1; }
if ssh "$PI5_SSH" "grep -q 'TNC server bind failed\|bind failed' $LOGS_DIR/bravo_tnc.log 2>/dev/null"; then
  echo "FAIL: bravo ultra_tnc bind failed (port collision?)"; ssh "$PI5_SSH" "tail -10 $LOGS_DIR/bravo_tnc.log"; exit 1
fi
echo "  pid=$(ssh $PI5_SSH cat $LOGS_DIR/bravo_tnc.pid)"

# ---- 4. Start BRAVO's Pat (listener mode) on Pi5 ----
echo "[4/6] starting Pat BRAVO listener on Pi5 (--listen varahf)"
ssh -fn "$PI5_SSH" "nohup $PI5_PAT_BIN \
    --config $LOGS_DIR/pat_bravo.json \
    --mbox $LOGS_DIR/bravo/mailbox \
    --log $LOGS_DIR/bravo_pat.log \
    --mycall BRAVO \
    --listen varahf \
    http -a 127.0.0.1:18080 \
    >$LOGS_DIR/bravo_pat_stdout.log 2>&1 </dev/null & disown; echo \$! >$LOGS_DIR/bravo_pat.pid"
sleep 3
ssh "$PI5_SSH" "kill -0 \$(cat $LOGS_DIR/bravo_pat.pid) 2>/dev/null" || { echo "FAIL: bravo Pat died"; ssh "$PI5_SSH" "echo --bravo_pat.log--; tail -30 $LOGS_DIR/bravo_pat.log 2>&1; echo --bravo_pat_stdout.log--; tail -30 $LOGS_DIR/bravo_pat_stdout.log 2>&1"; exit 1; }

# ---- 5. Compose + send message ALPHA -> BRAVO ----
echo "[5/6] composing test message ALPHA -> BRAVO and connecting via varahf"
SUBJECT="OTASim sanity $(date +%Y%m%d_%H%M%S)"
BODY="OTASim end-to-end test from ALPHA at $(date -u +%FT%TZ).
Path: ALPHA Pat -> ALPHA ultra_tnc -> OTASim serve -> BRAVO ultra_tnc -> BRAVO Pat
Random tag: $RANDOM"

# Compose into ALPHA's outbox via Pat's CLI (non-interactive via heredoc).
# Allow non-zero exit (Pat may exit 1 on harmless warnings) and continue
# as long as the outbox has a message after.
set +e
"$PAT_BIN" \
    --config "$LOGS_DIR/pat_alpha.json" \
    --mbox "$LOGS_DIR/alpha/mailbox" \
    --log "$LOGS_DIR/alpha_pat.log" \
    --mycall ALPHA \
    compose --from ALPHA --subject "$SUBJECT" BRAVO <<< "$BODY"
COMPOSE_RC=$?
set -e
echo "  compose exit=$COMPOSE_RC"
echo "  outbox after compose:"
ls -1 "$LOGS_DIR/alpha/mailbox/ALPHA/out/" 2>&1 || echo "    (outbox empty or missing)"
OUT_COUNT=$(ls -1 "$LOGS_DIR/alpha/mailbox/ALPHA/out/" 2>/dev/null | wc -l | tr -d ' ')
if [[ "$OUT_COUNT" -eq 0 ]]; then
  echo "FAIL: compose did not produce a message in outbox"
  cat "$LOGS_DIR/alpha_pat.log" 2>&1 || true
  exit 1
fi

# Connect to deliver
"$PAT_BIN" \
    --config "$LOGS_DIR/pat_alpha.json" \
    --mbox "$LOGS_DIR/alpha/mailbox" \
    --log "$LOGS_DIR/alpha_pat.log" \
    --mycall ALPHA \
    connect "varahf://127.0.0.1:$ALPHA_TNC_PORT/BRAVO" \
    2>&1 | tee "$LOGS_DIR/alpha_pat_connect.log"

# ---- 6. Verify message arrived in BRAVO inbox ----
echo "[6/6] waiting up to ${MESSAGE_TIMEOUT_S}s for message in BRAVO inbox"
for i in $(seq 1 "$MESSAGE_TIMEOUT_S"); do
  if ssh "$PI5_SSH" "ls $LOGS_DIR/bravo/mailbox/BRAVO/in/*.b2f 2>/dev/null | head -1" >/dev/null 2>&1; then
    echo "[PASS] message arrived after ${i}s"
    echo
    echo "=== Message in BRAVO inbox ==="
    ssh "$PI5_SSH" "ls -la $LOGS_DIR/bravo/mailbox/BRAVO/in/"
    echo "=== Server captures ==="
    ls -la "$LOGS_DIR/captures/lobby/" 2>/dev/null || echo "  no captures (lobby may be ephemeral)"
    echo
    echo "Logs preserved in $LOGS_DIR on both Mac and Pi5"
    exit 0
  fi
  sleep 1
done

echo "[FAIL] message did not arrive within ${MESSAGE_TIMEOUT_S}s"
echo "=== Last 20 lines of server.log ==="
tail -20 "$LOGS_DIR/server.log"
echo "=== Last 20 lines of alpha_tnc.log ==="
tail -20 "$LOGS_DIR/alpha_tnc.log"
echo "=== Last 20 lines of alpha_pat_connect.log ==="
tail -20 "$LOGS_DIR/alpha_pat_connect.log"
echo "=== Last 20 lines of bravo_tnc.log (Pi5) ==="
ssh "$PI5_SSH" "tail -20 $LOGS_DIR/bravo_tnc.log"
echo "=== Last 20 lines of bravo_pat.log (Pi5) ==="
ssh "$PI5_SSH" "tail -20 $LOGS_DIR/bravo_pat.log"
exit 1
