# Regenerate build_info.hpp at BUILD time, not configure time.
#
# WHY: execute_process(git rev-parse) in the top-level CMakeLists runs once, at
# CONFIGURE time, and configure_file() bakes the result into the header. Incremental
# builds never re-run configure, so the commit/dirty/tag go stale silently and
# `ultra_gui --version` reports whatever HEAD was when the build tree was created.
#
# On 2026-08-05 that nearly caused a mixed-binary mis-attribution during an IONOS rig
# campaign: the Mac binary contained the branch's code but reported the main commit it
# had been configured on. Provenance you cannot trust is worse than none, because it is
# believed.
#
# Run via `cmake -P` from a custom target. Writes to a temp file and copies only on
# change, so the 6 translation units that include build_info.hpp rebuild ONLY when the
# git state genuinely moved.
#
# kBuildTimeUtc is HEAD's COMMIT date, deliberately not wall-clock: a wall-clock stamp
# differs on every invocation, which would defeat copy-if-different and force those 6
# TUs to rebuild on every incremental build. Commit date is stable per commit and is
# the more useful provenance anyway (it identifies the source, not the machine's clock).

if(NOT DEFINED ULTRA_SRC_DIR OR NOT DEFINED ULTRA_OUT_HEADER OR NOT DEFINED ULTRA_TEMPLATE)
    message(FATAL_ERROR "UltraBuildInfo.cmake requires ULTRA_SRC_DIR, ULTRA_OUT_HEADER, ULTRA_TEMPLATE")
endif()

execute_process(
    COMMAND git rev-parse --verify HEAD
    WORKING_DIRECTORY ${ULTRA_SRC_DIR}
    OUTPUT_VARIABLE ULTRA_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT ULTRA_GIT_COMMIT)
    set(ULTRA_GIT_COMMIT "unknown")
endif()
string(SUBSTRING "${ULTRA_GIT_COMMIT}" 0 12 ULTRA_GIT_COMMIT_SHORT)

execute_process(
    COMMAND git describe --tags --exact-match HEAD
    WORKING_DIRECTORY ${ULTRA_SRC_DIR}
    OUTPUT_VARIABLE ULTRA_RELEASE_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT ULTRA_RELEASE_TAG)
    set(ULTRA_RELEASE_TAG "")
endif()

# Dirty is the field that matters most on a rig: it is the difference between "this
# binary is the commit I think it is" and "someone edited a file since".
execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${ULTRA_SRC_DIR}
    OUTPUT_VARIABLE ULTRA_GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(ULTRA_GIT_STATUS STREQUAL "")
    set(ULTRA_GIT_DIRTY_BOOL false)
else()
    set(ULTRA_GIT_DIRTY_BOOL true)
endif()

execute_process(
    COMMAND git show -s --format=%cd --date=format-local:%Y-%m-%dT%H:%M:%SZ HEAD
    WORKING_DIRECTORY ${ULTRA_SRC_DIR}
    OUTPUT_VARIABLE ULTRA_BUILD_TIME_UTC
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT ULTRA_BUILD_TIME_UTC)
    set(ULTRA_BUILD_TIME_UTC "unknown")
endif()

configure_file(${ULTRA_TEMPLATE} "${ULTRA_OUT_HEADER}.tmp" @ONLY)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ULTRA_OUT_HEADER}.tmp" "${ULTRA_OUT_HEADER}")
file(REMOVE "${ULTRA_OUT_HEADER}.tmp")
