# ProjectUltra Licensing

ProjectUltra source code is distributed under the MIT License. See `LICENSE`.

All bundled third-party libraries are under permissive licenses (MIT,
BSD-3-Clause, zlib, Apache 2.0) compatible with redistribution under MIT.
Hamlib (LGPL 2.1+) is dynamically linked only and follows the substitution
exception described below.

## Vendored sources (in `thirdparty/`)

- **PocketFFT** — BSD-3-Clause. Header-only; see `thirdparty/pocketfft/LICENSE.md`.
- **Dear ImGui** — MIT. See `thirdparty/imgui/LICENSE.txt`.
- **miniz** — public domain / MIT dual. See `thirdparty/miniz/`.
- **stb** — public domain / MIT dual. See `thirdparty/stb/`.

## Runtime libraries (bundled into release binaries)

- **SDL2** — zlib license. Bundled in macOS, Linux, and Windows operator bundles when the package script can locate the built binary's SDL2 runtime dependency.
- **Hamlib (libhamlib)** — LGPL 2.1+ (or later). Dynamically linked into `ultra_gui` and `ultra_tnc` when built with `ULTRA_USE_LIBHAMLIB=ON`. The Windows release bundle ships the prebuilt 4.7.1 runtime DLLs (`libhamlib-4.dll` plus `libusb-1.0.dll`, `libgcc_s_seh-1.dll`, `libwinpthread-1.dll`) alongside the binary; the full LGPL text is included as `COPYING.LIB.txt` in the same directory. Upstream source: https://github.com/Hamlib/Hamlib. To exercise the LGPL substitution right, replace the shipped `libhamlib-4.dll` with your own build of the same major version. The Hamlib utility binaries (`rigctld`, `rigctl`) are GPL 2+ and are NOT redistributed by this project; operators install them separately if they want the rigctld TCP backend.

## OTASim server / client runtime dependencies (since 2026-05-17)

`ota_simulator`, `ota_simulator serve`, and (after Task 05+) `ultra_gui`,
`ultra_tnc`, and `cli_simulator` link against the following libraries
for the gRPC + protobuf control plane and the UDP audio plane:

- **gRPC** (grpc++, grpc) — Apache 2.0. https://github.com/grpc/grpc
- **Protocol Buffers** (libprotobuf) — BSD-3-Clause. https://github.com/protocolbuffers/protobuf
- **Abseil C++** (libabsl_*) — Apache 2.0. https://github.com/abseil/abseil-cpp
- **c-ares** (libcares) — MIT. https://github.com/c-ares/c-ares
- **OpenSSL 3** (libssl, libcrypto) — Apache 2.0. https://github.com/openssl/openssl
- **RE2** (libre2) — BSD-3-Clause. https://github.com/google/re2
- **zlib** (libz) — zlib license. https://github.com/madler/zlib

These dependencies are sourced from system package managers (Homebrew on
macOS, apt/yum on Linux, vcpkg/MSYS2 on Windows) and are NOT vendored
into this repo. For binary release bundles:

- Linux / Windows: if these libraries are statically linked into the
  shipped binary or bundled as DLLs/`.so` alongside it, the
  corresponding upstream LICENSE and (for Apache 2.0 deps) NOTICE files
  MUST be included in the release archive.
- macOS: if dynamically linked against the user's existing Homebrew
  installation (the default), no additional bundling is required — the
  user obtained the libraries with their own brew install and received
  the licenses with that install. If the release ships its own static
  builds or `.dylib`s, the same bundling rules as Linux apply.

When the release CI/packaging script (`packaging/package_operator_bundle.sh`
and `.github/workflows/build-matrix.yml`) is extended to include gRPC
and its dependencies, the packaging step MUST also copy the upstream
LICENSE + NOTICE files into the release bundle for any statically linked
or co-shipped library. See https://www.apache.org/licenses/LICENSE-2.0
for the Apache 2.0 attribution requirements.
