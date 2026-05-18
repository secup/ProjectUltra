#!/usr/bin/env bash
# Create the alpha operator bundle and the separate developer-tools bundle.

set -euo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 <build-dir> <target> [output-dir]" >&2
  echo "  target: linux | macos | windows | local" >&2
  exit 2
fi

BUILD_DIR=$1
TARGET=$2
OUTPUT_DIR=${3:-package}

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
BUILD_DIR_ABS=$(cd -- "$BUILD_DIR" && pwd)
if [[ "$OUTPUT_DIR" = /* ]]; then
  OUTPUT_DIR_ABS="$OUTPUT_DIR"
else
  OUTPUT_DIR_ABS="$REPO_ROOT/$OUTPUT_DIR"
fi

operator_root="$OUTPUT_DIR_ABS/projectultra-$TARGET"
dev_root="$OUTPUT_DIR_ABS/dev-tools-$TARGET"

rm -rf "$operator_root" "$dev_root"
mkdir -p "$operator_root/tools" "$operator_root/docs" "$dev_root"

copy_binary() {
  local name=$1
  local dest=$2
  local found=0
  local candidates=(
    "$BUILD_DIR_ABS/$name"
    "$BUILD_DIR_ABS/Release/$name"
    "$BUILD_DIR_ABS/$name.exe"
    "$BUILD_DIR_ABS/Release/$name.exe"
  )

  for path in "${candidates[@]}"; do
    if [[ -f "$path" ]]; then
      cp "$path" "$dest/"
      found=1
    fi
  done

  [[ "$found" -eq 1 ]]
}

require_binary() {
  local name=$1
  local dest=$2
  if ! copy_binary "$name" "$dest"; then
    echo "Required binary not found in $BUILD_DIR_ABS: $name" >&2
    exit 1
  fi
}

optional_binary() {
  local name=$1
  local dest=$2
  copy_binary "$name" "$dest" || true
}

find_executables() {
  local root=$1
  find "$root" -maxdepth 1 -type f -perm -111 -print
}

bundle_macos_sdl2_dir() {
  local root=$1
  local dylib_path=""
  local bin

  while IFS= read -r bin; do
    dylib_path=$(otool -L "$bin" 2>/dev/null \
      | awk '/libSDL2-2\.0\.0\.dylib/ {print $1; exit}')
    if [[ -n "$dylib_path" ]]; then
      break
    fi
  done < <(find_executables "$root")

  if [[ -z "$dylib_path" ]]; then
    return 0
  fi
  if [[ ! -f "$dylib_path" ]]; then
    echo "SDL2 dylib referenced by bundle binaries was not found: $dylib_path" >&2
    exit 1
  fi

  local dylib_name
  dylib_name=$(basename "$dylib_path")
  cp "$dylib_path" "$root/$dylib_name"

  while IFS= read -r bin; do
    if otool -L "$bin" 2>/dev/null | grep -Fq "$dylib_path"; then
      install_name_tool -change "$dylib_path" "@executable_path/$dylib_name" "$bin"
    fi
  done < <(find_executables "$root")
}

bundle_linux_sdl2_dir() {
  local root=$1
  local so_path=""
  local bin

  while IFS= read -r bin; do
    so_path=$(ldd "$bin" 2>/dev/null \
      | awk '/libSDL2-2\.0\.so\.0/ {print $3; exit}')
    if [[ -n "$so_path" ]]; then
      break
    fi
  done < <(find_executables "$root")

  if [[ -z "$so_path" ]]; then
    return 0
  fi
  if [[ ! -f "$so_path" ]]; then
    echo "SDL2 shared object referenced by bundle binaries was not found: $so_path" >&2
    exit 1
  fi
  if ! command -v patchelf >/dev/null 2>&1; then
    echo "patchelf is required to make Linux bundles prefer bundled SDL2." >&2
    exit 1
  fi

  cp "$so_path" "$root/$(basename "$so_path")"

  while IFS= read -r bin; do
    if ldd "$bin" 2>/dev/null | grep -q 'libSDL2-2\.0\.so\.0'; then
      patchelf --set-rpath '$ORIGIN' "$bin"
    fi
  done < <(find_executables "$root")
}

bundle_sdl2_runtime() {
  case "$TARGET" in
    macos)
      bundle_macos_sdl2_dir "$operator_root"
      bundle_macos_sdl2_dir "$dev_root"
      ;;
    linux)
      bundle_linux_sdl2_dir "$operator_root"
      bundle_linux_sdl2_dir "$dev_root"
      ;;
  esac
}

require_binary ultra "$operator_root"
require_binary ultra_tnc "$operator_root"
require_binary ultra_gui "$operator_root"
require_binary ultra_report "$operator_root"

optional_binary cli_simulator "$dev_root"
optional_binary threaded_simulator "$dev_root"
optional_binary test_waveform_simple "$dev_root"
optional_binary decode_bench "$dev_root"
optional_binary session_decode "$dev_root"
optional_binary ultra_replay "$dev_root"

if ! compgen -G "$dev_root/*" >/dev/null; then
  echo "No developer tools found to package." >&2
  exit 1
fi

cp "$REPO_ROOT/tools/ultra_tnc.conf.example" "$operator_root/tools/"
cp "$REPO_ROOT/README.md" "$operator_root/"
cp "$REPO_ROOT/docs/TNC_INTERFACE.md" "$operator_root/docs/"
cp "$REPO_ROOT/docs/README.md" "$operator_root/docs/"
cp "$REPO_ROOT/LICENSING.md" "$operator_root/"
cp "$REPO_ROOT/LICENSING.md" "$dev_root/"

if [[ -f "$REPO_ROOT/docs/RUNNING.md" ]]; then
  cp "$REPO_ROOT/docs/RUNNING.md" "$operator_root/RUNNING.md"
fi

if [[ "$TARGET" == "windows" ]]; then
  sdl_found=0
  declare -a dll_candidates=()
  declare -a vcpkg_install_roots=()

  if [[ -n "${VCPKG_INSTALLED:-}" ]]; then
    vcpkg_installed="$VCPKG_INSTALLED"
    if command -v cygpath >/dev/null 2>&1; then
      vcpkg_installed=$(cygpath -u "$vcpkg_installed")
    fi
    vcpkg_install_roots+=("$vcpkg_installed")
  fi

  if [[ -n "${VCPKG_ROOT:-}" ]]; then
    vcpkg_unix="$VCPKG_ROOT"
    if command -v cygpath >/dev/null 2>&1; then
      vcpkg_unix=$(cygpath -u "$VCPKG_ROOT")
    fi
    vcpkg_install_roots+=("${vcpkg_unix}/installed/x64-windows")
  fi

  vcpkg_install_roots+=("/c/vcpkg/installed/x64-windows")

  for vcpkg_root in "${vcpkg_install_roots[@]}"; do
    dll_candidates+=("${vcpkg_root}/bin/SDL2.dll")
    dll_candidates+=("${vcpkg_root}/bin/SDL2d.dll")
  done
  dll_candidates+=("C:/SDL2/lib/x64/SDL2.dll")

  for dll in "${dll_candidates[@]}"; do
    if [[ -f "$dll" ]]; then
      cp "$dll" "$operator_root/"
      cp "$dll" "$dev_root/"
      sdl_found=1
    fi
  done

  if [[ "$sdl_found" -eq 0 ]]; then
    echo "SDL2 runtime DLL was not found. Expected from vcpkg or C:/SDL2." >&2
    exit 1
  fi

  grpc_root=""
  for vcpkg_root in "${vcpkg_install_roots[@]}"; do
    if [[ -d "$vcpkg_root/bin" ]] && compgen -G "$vcpkg_root/bin/grpc*.dll" > /dev/null; then
      grpc_root="$vcpkg_root"
      break
    fi
    if [[ -f "$vcpkg_root/bin/gpr.dll" ]]; then
      grpc_root="$vcpkg_root"
      break
    fi
  done

  if [[ -z "$grpc_root" ]]; then
    echo "gRPC runtime DLLs were not found. Expected from vcpkg x64-windows." >&2
    echo "Searched vcpkg roots:" >&2
    for vcpkg_root in "${vcpkg_install_roots[@]}"; do
      echo "  $vcpkg_root" >&2
      if [[ -d "$vcpkg_root/bin" ]]; then
        echo "  --> bin/ exists, listing *.dll:" >&2
        ls "$vcpkg_root/bin"/*.dll 2>/dev/null | sed 's/^/    /' >&2 || echo "    (no dlls)" >&2
      else
        echo "  --> bin/ missing" >&2
      fi
    done
    exit 1
  fi

  grpc_dependency_dlls=(
    grpc++.dll
    grpc.dll
    gpr.dll
    address_sorting.dll
    libprotobuf.dll
    abseil_dll.dll
    cares.dll
    re2.dll
    utf8_range.dll
    utf8_validity.dll
    libcrypto-3-x64.dll
    libssl-3-x64.dll
    zlib1.dll
  )

  for dll in "${grpc_dependency_dlls[@]}"; do
    dll_src="$grpc_root/bin/$dll"
    if [[ -f "$dll_src" ]]; then
      cp "$dll_src" "$operator_root/"
      cp "$dll_src" "$dev_root/"
    fi
  done

  mkdir -p "$operator_root/THIRD_PARTY_LICENSES" "$dev_root/THIRD_PARTY_LICENSES"
  for pkg in grpc protobuf abseil c-ares re2 utf8-range openssl zlib; do
    license_src="$grpc_root/share/$pkg/copyright"
    if [[ -f "$license_src" ]]; then
      cp "$license_src" "$operator_root/THIRD_PARTY_LICENSES/$pkg-LICENSE.txt"
      cp "$license_src" "$dev_root/THIRD_PARTY_LICENSES/$pkg-LICENSE.txt"
    fi
  done
fi

bundle_sdl2_runtime

cat > "$operator_root/BUNDLE.txt" <<EOF
ProjectUltra alpha operator bundle ($TARGET)

Included operator programs:
- ultra_tnc
- ultra_gui
- ultra_report  (CLI for diagnostics bundle management;
                 list / create / inspect / summary / replay-prep)
- ultra

Operator docs:
- RUNNING.md (added when docs/RUNNING.md exists in the checkout)
- README.md
- LICENSING.md
- docs/TNC_INTERFACE.md
- docs/README.md
- tools/ultra_tnc.conf.example

Developer simulators and bench tools are packaged separately in dev-tools-$TARGET.
EOF

cat > "$dev_root/BUNDLE.txt" <<EOF
ProjectUltra developer tools ($TARGET)

This artifact is for simulation, bench, and decode diagnostics. It is not the
default operator download.

Included developer programs (when built):
- cli_simulator       Two-station modem regression harness
- threaded_simulator  Threaded variant for stress testing
- test_waveform_simple  Single-frame waveform sanity check
- decode_bench        Decode benchmarking
- session_decode      Single-mode WAV replay through StreamingDecoder
- ultra_replay        Event-aware bundle replay + divergence report
                      (consumes report.zip emailed by an operator;
                       replays audio against the live JSONL timeline
                       and diffs frame-by-frame)

Runtime/library notices are in LICENSING.md.
EOF
