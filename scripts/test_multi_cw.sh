#!/bin/bash
# Test multi-codeword v2 decode at different timing offsets

RECORDING="${1:-/tmp/recording.f32}"
BINARY="./build/ultra"
SCAN="/tmp/scan2"

if [ ! -f "$RECORDING" ]; then
    echo "Usage: $0 <recording.f32>"
    exit 1
fi

echo "=== Testing multi-codeword decode on $RECORDING ==="
echo ""

# First, find preamble
echo "--- Scanning for preamble ---"
if [ -x "$SCAN" ]; then
    $SCAN "$RECORDING" 2>&1 | head -10
else
    echo "(scan tool not found at $SCAN)"
fi

echo ""
echo "--- File info ---"
ls -la "$RECORDING"
echo "Samples: $(( $(stat -c%s "$RECORDING") / 4 ))"

echo ""
echo "--- Testing decode at offsets 205-220 ---"

for offset in 205 207 209 211 213 215 217 219; do
    echo ""
    echo "=== Offset $offset ==="
    timeout 5 $BINARY -m dqpsk -c 1/4 -t $offset prx "$RECORDING" 2>&1 | \
        grep -E "(FRAME|CW |CW0|CW1|CW2|CW3|SYNC|SNR|LDPC|MESSAGE|magic)" | head -20
done

echo ""
echo "--- Per-codeword decode summary ---"
echo "Look for:"
echo "  - CW0: OK (first codeword must decode for header)"
echo "  - CW1: OK, CW2: OK, etc."
echo "  - If CW0 works but CW1+ fail, issue is in codeword boundaries or timing drift"
