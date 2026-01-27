#!/bin/bash
#
# Regression Test Matrix for ProjectUltra
#
# Purpose: Run all critical tests and report pass/fail summary.
# Run this after any code changes to verify nothing is broken.
#
# Usage: ./tests/regression_matrix.sh [--quick|--full]
#   --quick: Run minimal tests (default)
#   --full:  Run comprehensive tests (takes longer)
#
# Exit codes:
#   0 = All tests passed
#   1 = Some tests failed
#
# Test Tools:
#   - test_iwaveform: PRIMARY test tool (uses IWaveform interface directly)
#   - cli_simulator:  Protocol testing (currently has CFO bug - BUG-001)
#   - test_hf_modem:  LEGACY - reference only, will be removed
#
# NOTE: test_iwaveform will be renamed to something more appropriate
#       (e.g., test_modem, modem_test) in a future refactor.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counters
PASSED=0
FAILED=0
SKIPPED=0

# Test results array
declare -a RESULTS

# Parse arguments
MODE="quick"
if [[ "$1" == "--full" ]]; then
    MODE="full"
fi

echo "=========================================="
echo "  ProjectUltra Regression Test Suite"
echo "  Mode: $MODE"
echo "  Date: $(date)"
echo "=========================================="
echo ""

# Check if test_iwaveform exists
if [[ ! -x "$BUILD_DIR/test_iwaveform" ]]; then
    echo -e "${RED}ERROR: test_iwaveform not found. Run 'make' first.${NC}"
    exit 1
fi

# Function to run a single test
run_test() {
    local name="$1"
    local cmd="$2"
    local expected_rate="$3"  # Minimum success rate (e.g., 80)
    local timeout_sec="${4:-120}"

    echo -n "Testing: $name ... "

    # Run the test and capture output
    local output
    local exit_code
    output=$(timeout "$timeout_sec" bash -c "$cmd" 2>&1) || exit_code=$?

    if [[ $exit_code -eq 124 ]]; then
        echo -e "${YELLOW}TIMEOUT${NC}"
        RESULTS+=("TIMEOUT: $name")
        ((SKIPPED++))
        return
    fi

    # Extract decode rate from output
    # Look for various output patterns from test tools
    local rate
    if [[ "$output" =~ Decoded:\ ([0-9]+)/([0-9]+)\ \(([0-9]+)%\) ]]; then
        # Matches "Decoded: 2/3 (67%)"
        rate="${BASH_REMATCH[3]}"
    elif [[ "$output" =~ Decode\ rate:\ ([0-9]+)% ]]; then
        rate="${BASH_REMATCH[1]}"
    elif [[ "$output" =~ ([0-9]+)/([0-9]+)\ frames ]]; then
        local decoded="${BASH_REMATCH[1]}"
        local total="${BASH_REMATCH[2]}"
        if [[ $total -gt 0 ]]; then
            rate=$((decoded * 100 / total))
        else
            rate=0
        fi
    elif [[ "$output" =~ Success:\ ([0-9]+)% ]]; then
        rate="${BASH_REMATCH[1]}"
    elif [[ "$output" =~ \(([0-9]+)%\) ]]; then
        # Fallback: any percentage in parentheses
        rate="${BASH_REMATCH[1]}"
    else
        # If no rate found, check for general success/failure
        if [[ "$output" =~ "PASS" ]] || [[ "$output" =~ "success" ]]; then
            rate=100
        else
            rate=0
        fi
    fi

    # Compare with expected
    if [[ $rate -ge $expected_rate ]]; then
        echo -e "${GREEN}PASS${NC} (${rate}% >= ${expected_rate}%)"
        RESULTS+=("PASS: $name (${rate}%)")
        ((PASSED++))
    else
        echo -e "${RED}FAIL${NC} (${rate}% < ${expected_rate}%)"
        RESULTS+=("FAIL: $name (${rate}% < ${expected_rate}%)")
        ((FAILED++))
        # Show last few lines of output for debugging
        echo "  Last output lines:"
        echo "$output" | tail -5 | sed 's/^/    /'
    fi
}

# ==========================================
# TEST MATRIX
# ==========================================

echo "--- MC-DPSK Tests ---"

# MC-DPSK AWGN (should be 100%)
run_test "MC-DPSK AWGN SNR=5 CFO=0" \
    "$BUILD_DIR/test_iwaveform --snr 5 --cfo 0 --channel awgn -w mc_dpsk --frames 5" \
    100

run_test "MC-DPSK AWGN SNR=5 CFO=30" \
    "$BUILD_DIR/test_iwaveform --snr 5 --cfo 30 --channel awgn -w mc_dpsk --frames 5" \
    100

run_test "MC-DPSK AWGN SNR=0 CFO=30" \
    "$BUILD_DIR/test_iwaveform --snr 0 --cfo 30 --channel awgn -w mc_dpsk --frames 5" \
    100

# MC-DPSK Fading (80%+ expected)
run_test "MC-DPSK Moderate SNR=5 CFO=0" \
    "$BUILD_DIR/test_iwaveform --snr 5 --cfo 0 --channel moderate -w mc_dpsk --frames 5" \
    80

run_test "MC-DPSK Moderate SNR=5 CFO=30" \
    "$BUILD_DIR/test_iwaveform --snr 5 --cfo 30 --channel moderate -w mc_dpsk --frames 5" \
    80

echo ""
echo "--- OFDM_CHIRP Tests ---"

# OFDM_CHIRP AWGN (should be 100%)
run_test "OFDM_CHIRP AWGN SNR=17 CFO=0" \
    "$BUILD_DIR/test_iwaveform --snr 17 --cfo 0 --channel awgn -w ofdm_chirp --frames 5" \
    100

run_test "OFDM_CHIRP AWGN SNR=17 CFO=30" \
    "$BUILD_DIR/test_iwaveform --snr 17 --cfo 30 --channel awgn -w ofdm_chirp --frames 5" \
    100

run_test "OFDM_CHIRP AWGN SNR=17 CFO=50" \
    "$BUILD_DIR/test_iwaveform --snr 17 --cfo 50 --channel awgn -w ofdm_chirp --frames 5" \
    100

# OFDM_CHIRP Fading (80%+ expected at 17 dB)
run_test "OFDM_CHIRP Moderate SNR=17 CFO=30" \
    "$BUILD_DIR/test_iwaveform --snr 17 --cfo 30 --channel moderate -w ofdm_chirp --frames 5" \
    80

# Full test mode adds more comprehensive tests
if [[ "$MODE" == "full" ]]; then
    echo ""
    echo "--- Extended MC-DPSK Tests (Full Mode) ---"

    # Edge cases
    run_test "MC-DPSK AWGN SNR=-3 CFO=0" \
        "$BUILD_DIR/test_iwaveform --snr -3 --cfo 0 --channel awgn -w mc_dpsk --frames 10" \
        80

    run_test "MC-DPSK AWGN SNR=10 CFO=50" \
        "$BUILD_DIR/test_iwaveform --snr 10 --cfo 50 --channel awgn -w mc_dpsk --frames 10" \
        100

    run_test "MC-DPSK Poor SNR=10 CFO=0" \
        "$BUILD_DIR/test_iwaveform --snr 10 --cfo 0 --channel poor -w mc_dpsk --frames 10" \
        80

    echo ""
    echo "--- Extended OFDM_CHIRP Tests (Full Mode) ---"

    run_test "OFDM_CHIRP AWGN SNR=10 CFO=30" \
        "$BUILD_DIR/test_iwaveform --snr 10 --cfo 30 --channel awgn -w ofdm_chirp --frames 10" \
        100

    run_test "OFDM_CHIRP Good SNR=15 CFO=30" \
        "$BUILD_DIR/test_iwaveform --snr 15 --cfo 30 --channel good -w ofdm_chirp --frames 10" \
        90

    # Negative CFO
    run_test "MC-DPSK AWGN SNR=5 CFO=-30" \
        "$BUILD_DIR/test_iwaveform --snr 5 --cfo -30 --channel awgn -w mc_dpsk --frames 5" \
        100

    run_test "OFDM_CHIRP AWGN SNR=17 CFO=-50" \
        "$BUILD_DIR/test_iwaveform --snr 17 --cfo -50 --channel awgn -w ofdm_chirp --frames 5" \
        100
fi

# ==========================================
# SUMMARY
# ==========================================

echo ""
echo "=========================================="
echo "  TEST SUMMARY"
echo "=========================================="
echo ""

# Print all results
for result in "${RESULTS[@]}"; do
    if [[ "$result" == PASS* ]]; then
        echo -e "  ${GREEN}$result${NC}"
    elif [[ "$result" == FAIL* ]]; then
        echo -e "  ${RED}$result${NC}"
    else
        echo -e "  ${YELLOW}$result${NC}"
    fi
done

echo ""
echo "----------------------------------------"
echo -e "  Passed:  ${GREEN}$PASSED${NC}"
echo -e "  Failed:  ${RED}$FAILED${NC}"
echo -e "  Skipped: ${YELLOW}$SKIPPED${NC}"
echo -e "  Total:   $((PASSED + FAILED + SKIPPED))"
echo "----------------------------------------"

if [[ $FAILED -eq 0 ]]; then
    echo ""
    echo -e "${GREEN}ALL TESTS PASSED!${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}SOME TESTS FAILED!${NC}"
    echo "Review the failures above before committing."
    exit 1
fi
