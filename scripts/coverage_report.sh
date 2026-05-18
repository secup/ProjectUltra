#!/usr/bin/env bash
set -euo pipefail

# Reproducible local coverage gate for maintained CTest targets.
#
# Defaults intentionally track the current post-cleanup baseline. Raise these
# thresholds as focused tests are added; do not lower them to hide regressions.
#
# Usage:
#   ./scripts/coverage_report.sh
#   ./scripts/coverage_report.sh --min-lines 45 --min-functions 50 --min-branches 35

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$PROJECT_DIR/build-coverage}"
CTEST_JOBS="${CTEST_JOBS:-4}"
MIN_LINES="50"
MIN_FUNCTIONS="55"
MIN_BRANCHES="38"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --min-lines)
      MIN_LINES="$2"
      shift 2
      ;;
    --min-functions)
      MIN_FUNCTIONS="$2"
      shift 2
      ;;
    --min-branches)
      MIN_BRANCHES="$2"
      shift 2
      ;;
    -h|--help)
      sed -n '1,18p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -x /opt/homebrew/opt/llvm/bin/clang && -x /opt/homebrew/opt/llvm/bin/clang++ ]]; then
  CC_BIN="${CC:-/opt/homebrew/opt/llvm/bin/clang}"
  CXX_BIN="${CXX:-/opt/homebrew/opt/llvm/bin/clang++}"
else
  CC_BIN="${CC:-clang}"
  CXX_BIN="${CXX:-clang++}"
fi

LLVM_COV="${LLVM_COV:-}"
LLVM_PROFDATA="${LLVM_PROFDATA:-}"
if [[ -z "$LLVM_COV" ]]; then
  if [[ -x /opt/homebrew/opt/llvm/bin/llvm-cov ]]; then
    LLVM_COV="/opt/homebrew/opt/llvm/bin/llvm-cov"
  else
    LLVM_COV="llvm-cov"
  fi
fi
if [[ -z "$LLVM_PROFDATA" ]]; then
  if [[ -x /opt/homebrew/opt/llvm/bin/llvm-profdata ]]; then
    LLVM_PROFDATA="/opt/homebrew/opt/llvm/bin/llvm-profdata"
  else
    LLVM_PROFDATA="llvm-profdata"
  fi
fi

cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DULTRA_BUILD_GUI=OFF \
  -DULTRA_BUILD_TOOLS=OFF \
  -DULTRA_USE_FFTW=OFF \
  -DCMAKE_C_COMPILER="$CC_BIN" \
  -DCMAKE_CXX_COMPILER="$CXX_BIN" \
  "-DCMAKE_C_FLAGS=-fprofile-instr-generate -fcoverage-mapping" \
  "-DCMAKE_CXX_FLAGS=-fprofile-instr-generate -fcoverage-mapping" \
  "-DCMAKE_EXE_LINKER_FLAGS=-fprofile-instr-generate"

cmake --build "$BUILD_DIR" --parallel "$CTEST_JOBS"

rm -rf "$BUILD_DIR/profraw"
mkdir -p "$BUILD_DIR/profraw"
LLVM_PROFILE_FILE="$BUILD_DIR/profraw/%p.profraw" \
  ctest --test-dir "$BUILD_DIR" --output-on-failure -j "$CTEST_JOBS"

"$LLVM_PROFDATA" merge -sparse "$BUILD_DIR"/profraw/*.profraw \
  -o "$BUILD_DIR/coverage.profdata"

executables=()
while IFS= read -r exe; do
  [[ -x "$exe" ]] || continue
  [[ "$(basename "$exe")" == "test_throughput" ]] && continue
  executables+=("$exe")
done < <(find "$BUILD_DIR/tests" -maxdepth 1 -type f -name 'test_*' | sort)

if [[ "${#executables[@]}" -eq 0 ]]; then
  echo "error: no test executables found under $BUILD_DIR/tests" >&2
  exit 2
fi

object_args=()
for exe in "${executables[@]:1}"; do
  object_args+=("-object=$exe")
done

REPORT="$BUILD_DIR/coverage.txt"
"$LLVM_COV" report \
  -instr-profile="$BUILD_DIR/coverage.profdata" \
  "${executables[0]}" \
  "${object_args[@]}" \
  -ignore-filename-regex='(/thirdparty/|/tests/|/generated/|/Applications/|/opt/homebrew/|/usr/include/|/Library/Developer/)' \
  | tee "$REPORT"

read -r function_pct line_pct branch_pct < <(
  awk '/^TOTAL / {
    gsub("%", "", $7);
    gsub("%", "", $10);
    gsub("%", "", $13);
    print $7, $10, $13;
  }' "$REPORT"
)

check_threshold() {
  local label="$1"
  local actual="$2"
  local minimum="$3"
  awk -v actual="$actual" -v minimum="$minimum" -v label="$label" '
    BEGIN {
      if ((actual + 0.0) < (minimum + 0.0)) {
        printf("coverage gate failed: %s %.2f%% < %.2f%%\n", label, actual, minimum) > "/dev/stderr";
        exit 1;
      }
    }'
}

check_threshold "function" "$function_pct" "$MIN_FUNCTIONS"
check_threshold "line" "$line_pct" "$MIN_LINES"
check_threshold "branch" "$branch_pct" "$MIN_BRANCHES"

echo
printf 'coverage gate passed: functions=%s%% lines=%s%% branches=%s%%\n' \
  "$function_pct" "$line_pct" "$branch_pct"
