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

optional_binary ota_simulator "$dev_root"
optional_binary measure_ack_fer "$dev_root"
optional_binary ultra_replay "$dev_root"

if ! compgen -G "$dev_root/*" >/dev/null; then
  echo "No developer tools found to package." >&2
  exit 1
fi

cp "$REPO_ROOT/tools/ultra_tnc.conf.example" "$operator_root/tools/"
cp "$REPO_ROOT/tools/ultra_tnc.otasim.conf.example" "$operator_root/tools/"
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

  # vcpkg triplet of the dependencies, requested one first: CI exports VCPKG_TRIPLET
  # (x64-windows-release since 2026-11-09); a local vcpkg build usually uses x64-windows.
  declare -a vcpkg_triplets=()
  [[ -n "${VCPKG_TARGET_TRIPLET:-}" ]] && vcpkg_triplets+=("$VCPKG_TARGET_TRIPLET")
  [[ -n "${VCPKG_TRIPLET:-}" ]] && vcpkg_triplets+=("$VCPKG_TRIPLET")
  vcpkg_triplets+=("x64-windows" "x64-windows-release")

  if [[ -n "${VCPKG_ROOT:-}" ]]; then
    vcpkg_unix="$VCPKG_ROOT"
    if command -v cygpath >/dev/null 2>&1; then
      vcpkg_unix=$(cygpath -u "$VCPKG_ROOT")
    fi
    for triplet in "${vcpkg_triplets[@]}"; do
      vcpkg_install_roots+=("${vcpkg_unix}/installed/${triplet}")
    done
  fi

  for triplet in "${vcpkg_triplets[@]}"; do
    vcpkg_install_roots+=("/c/vcpkg/installed/${triplet}")
  done

  # First root that has SDL2 wins (its debug DLL rides along when present), so a machine with
  # two triplets installed cannot ship a mixed pair.
  for vcpkg_root in "${vcpkg_install_roots[@]}"; do
    if [[ -f "${vcpkg_root}/bin/SDL2.dll" ]]; then
      dll_candidates+=("${vcpkg_root}/bin/SDL2.dll")
      [[ -f "${vcpkg_root}/bin/SDL2d.dll" ]] && dll_candidates+=("${vcpkg_root}/bin/SDL2d.dll")
      break
    fi
  done
  [[ "${#dll_candidates[@]}" -eq 0 ]] && dll_candidates+=("C:/SDL2/lib/x64/SDL2.dll")

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
    if [[ -f "$vcpkg_root/bin/libprotobuf.dll" ]]; then
      grpc_root="$vcpkg_root"
      break
    fi
  done

  if [[ -z "$grpc_root" ]]; then
    echo "gRPC/protobuf runtime DLLs were not found. Expected from vcpkg (${vcpkg_triplets[*]})." >&2
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
    libprotobuf.dll
    abseil_dll.dll
    cares.dll
    re2.dll
    libcrypto-3-x64.dll
    libssl-3-x64.dll
    legacy.dll
    z.dll
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

  # --- Hamlib runtime (CAT/PTT) — extract the DLLs from the vendored zip so the bundle is
  #     self-contained. ultra_gui/ultra_tnc link libhamlib, which pulls in the MinGW runtime
  #     (libgcc_s_seh-1.dll, libwinpthread-1.dll) and libusb. Without these the app won't start.
  hamlib_zip="$REPO_ROOT/thirdparty/hamlib-windows/hamlib-w64-4.7.1.zip"
  if [[ ! -f "$hamlib_zip" ]]; then
    echo "Hamlib windows zip not found at $hamlib_zip" >&2
    exit 1
  fi
  hamlib_tmp=$(mktemp -d)
  unzip -q -o "$hamlib_zip" -d "$hamlib_tmp"
  hamlib_bin=$(find "$hamlib_tmp" -type d -name bin | head -1)
  hamlib_found=0
  for dll in libhamlib-4.dll libgcc_s_seh-1.dll libwinpthread-1.dll libusb-1.0.dll; do
    if [[ -f "$hamlib_bin/$dll" ]]; then
      cp "$hamlib_bin/$dll" "$operator_root/"
      cp "$hamlib_bin/$dll" "$dev_root/"
      hamlib_found=1
    else
      echo "WARNING: hamlib DLL $dll not found in $hamlib_zip" >&2
    fi
  done
  rm -rf "$hamlib_tmp"
  if [[ "$hamlib_found" -eq 0 ]]; then
    echo "No hamlib DLLs were bundled from $hamlib_zip" >&2
    exit 1
  fi

  # --- MSVC runtime — the app is built /MD against vcpkg's x64-windows(-release) triplet, so it needs
  #     the Visual C++ redistributable DLLs at runtime (vcruntime140*.dll, msvcp140*.dll). A
  #     FRESH Windows 11 does not ship VCRUNTIME140_1.dll, so ultra_gui.exe fails with
  #     "VCRUNTIME140_1.dll was not found". Bundle them so no VC++ Redist install is required.
  #     Source: the VS redist dir if the dev env is active, else System32 (both are the
  #     Microsoft-redistributable runtime).
  declare -a vcrt_dirs=()
  if [[ -n "${VCToolsRedistDir:-}" ]]; then
    vcredist_u="$VCToolsRedistDir"
    command -v cygpath >/dev/null 2>&1 && vcredist_u=$(cygpath -u "$VCToolsRedistDir")
    while IFS= read -r d; do vcrt_dirs+=("$d"); done \
      < <(find "$vcredist_u" -type d -iname 'Microsoft.VC*.CRT' -ipath '*x64*' 2>/dev/null)
  fi
  vcrt_dirs+=("/c/Windows/System32")
  vcrt_required=(vcruntime140.dll vcruntime140_1.dll msvcp140.dll)
  vcrt_optional=(msvcp140_1.dll msvcp140_2.dll concrt140.dll)
  for dll in "${vcrt_required[@]}" "${vcrt_optional[@]}"; do
    is_required=0
    for r in "${vcrt_required[@]}"; do [[ "$r" == "$dll" ]] && is_required=1; done
    copied=0
    for d in "${vcrt_dirs[@]}"; do
      if [[ -f "$d/$dll" ]]; then
        cp "$d/$dll" "$operator_root/"
        cp "$d/$dll" "$dev_root/"
        copied=1
        break
      fi
    done
    if [[ "$copied" -eq 0 && "$is_required" -eq 1 ]]; then
      echo "Required MSVC runtime DLL not found: $dll (searched ${vcrt_dirs[*]})" >&2
      exit 1
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
- tools/ultra_tnc.otasim.conf.example  (OTASim software-loopback test config)

Developer simulators and bench tools are packaged separately in dev-tools-$TARGET.
EOF

cat > "$dev_root/BUNDLE.txt" <<EOF
ProjectUltra developer tools ($TARGET)

This artifact is for simulation, bench, and decode diagnostics. It is not the
default operator download.

Included developer programs (when built):
- ota_simulator       Over-the-air channel simulator (serve + scenarios)
- measure_ack_fer     ACK/FER measurement over the real StreamingEncoder/Decoder
- ultra_replay        Event-aware bundle replay + divergence report
                      (consumes report.zip emailed by an operator;
                       replays audio against the live JSONL timeline
                       and diffs frame-by-frame)

Runtime/library notices are in LICENSING.md.
EOF
