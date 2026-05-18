cmake_minimum_required(VERSION 3.16)
project(ProjectUltra VERSION 0.3.5 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release if the operator/dev didn't pick a build type. Debug builds
# of this modem run several times slower on the audio thread, which manifests
# as throughput regressions and per-frame retx on hardware. Release is the
# right default for the modem hot path; the CI matrix overrides explicitly
# (Debug + sanitizers) for the test stages that need it.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "CMAKE_BUILD_TYPE not set; defaulting to Release")
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# MSVC-specific settings
if(MSVC)
    # Ensure math constants (M_PI, etc.) are available from <cmath> in all TUs.
    add_compile_definitions(_USE_MATH_DEFINES)
    # Block windows.h macro pollution: NOMINMAX kills max()/min() macros
    # that collide with std::numeric_limits<>::max() in protobuf/gRPC templates,
    # NOGDI skips wingdi.h which defines PASSTHROUGH and many other identifiers
    # that collide with normal C++ identifiers (e.g., our ChannelType::PASSTHROUGH),
    # WIN32_LEAN_AND_MEAN trims the windows.h surface area generally.
    add_compile_definitions(WIN32_LEAN_AND_MEAN NOMINMAX NOGDI)
    # Enable parallel builds
    add_compile_options(/MP)
    # Suppress some common warnings
    add_compile_options(/wd4244)  # Conversion warnings (double to float)
    add_compile_options(/wd4267)  # size_t to int conversion
    # Treat source files as UTF-8
    add_compile_options(/utf-8)
endif()

# Build options
option(ULTRA_BUILD_TESTS "Build test suite" ON)
option(ULTRA_BUILD_TOOLS "Build command-line tools" ON)
option(ULTRA_BUILD_GUI "Build GUI application" ON)
option(ULTRA_BUILD_HARDWARE_TESTS "Register opt-in hardware smoke CTest target" OFF)
option(ULTRA_USE_LIBHAMLIB "Link libhamlib for built-in rig control" ON)
option(ULTRA_FETCH_GRPC "Fetch gRPC/protobuf when CMake packages are unavailable" ON)

if(DEFINED ENV{ULTRA_HARDWARE_TESTS} AND "$ENV{ULTRA_HARDWARE_TESTS}" STREQUAL "1")
    set(ULTRA_BUILD_HARDWARE_TESTS ON CACHE BOOL
        "Register opt-in hardware smoke CTest target" FORCE)
endif()

if(MSVC)
    set(ULTRA_TOOL_TEST_WARNINGS /W4)
    set(ULTRA_TOOL_WARNING_SUPPRESSIONS /wd4100)
    set(ULTRA_TEST_ASSERT_OPTION /UNDEBUG)
else()
    set(ULTRA_TOOL_TEST_WARNINGS -Wall -Wextra)
    set(ULTRA_TOOL_WARNING_SUPPRESSIONS -Wno-unused-parameter -Wno-unused-private-field)
    set(ULTRA_TEST_ASSERT_OPTION -UNDEBUG)
endif()

function(ultra_enable_tool_test_warnings)
    foreach(src IN LISTS ARGN)
        set_property(SOURCE "${src}" APPEND PROPERTY COMPILE_OPTIONS
            ${ULTRA_TOOL_TEST_WARNINGS} ${ULTRA_TOOL_WARNING_SUPPRESSIONS})
    endforeach()
endfunction()

function(ultra_enable_test_warnings)
    foreach(src IN LISTS ARGN)
        set_property(SOURCE "${src}" APPEND PROPERTY COMPILE_OPTIONS
            ${ULTRA_TEST_ASSERT_OPTION} ${ULTRA_TOOL_TEST_WARNINGS})
    endforeach()
endfunction()

# Find dependencies
find_package(Threads REQUIRED)

find_package(Protobuf CONFIG QUIET)
find_package(gRPC CONFIG QUIET)

if(TARGET gRPC::grpc++ AND TARGET protobuf::libprotobuf)
    message(STATUS "Found gRPC/protobuf via CMake packages")
    set(ULTRA_GRPC_GRPCPP_TARGET gRPC::grpc++)
    set(ULTRA_PROTOBUF_LIB_TARGET protobuf::libprotobuf)
else()
    if(NOT ULTRA_FETCH_GRPC)
        message(FATAL_ERROR "gRPC/protobuf CMake packages not found and ULTRA_FETCH_GRPC=OFF")
    endif()

    include(FetchContent)
    set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)
    set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
    set(gRPC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(gRPC_INSTALL OFF CACHE BOOL "" FORCE)
    set(gRPC_ABSL_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_CARES_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_PROTOBUF_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_RE2_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_SSL_PROVIDER module CACHE STRING "" FORCE)
    set(gRPC_ZLIB_PROVIDER module CACHE STRING "" FORCE)

    FetchContent_Declare(
        grpc
        GIT_REPOSITORY https://github.com/grpc/grpc.git
        GIT_TAG v1.62.2
        GIT_SUBMODULES_RECURSE TRUE
    )
    FetchContent_MakeAvailable(grpc)
    set(ULTRA_GRPC_GRPCPP_TARGET grpc++)
    set(ULTRA_PROTOBUF_LIB_TARGET libprotobuf)
endif()

if(TARGET protobuf::protoc)
    set(ULTRA_PROTOC_EXECUTABLE $<TARGET_FILE:protobuf::protoc>)
elseif(TARGET protoc)
    set(ULTRA_PROTOC_EXECUTABLE $<TARGET_FILE:protoc>)
else()
    find_program(ULTRA_PROTOC_EXECUTABLE protoc REQUIRED)
endif()

if(TARGET gRPC::grpc_cpp_plugin)
    set(ULTRA_GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
elseif(TARGET grpc_cpp_plugin)
    set(ULTRA_GRPC_CPP_PLUGIN $<TARGET_FILE:grpc_cpp_plugin>)
else()
    find_program(ULTRA_GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)
endif()

set(ULTRA_OTASIM_PROTO_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/proto/ota_simulator.proto)
set(ULTRA_OTASIM_PROTO_GEN_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/generated/otasim_proto)
set(ULTRA_OTASIM_PROTO_SRCS
    ${ULTRA_OTASIM_PROTO_GEN_DIR}/ota_simulator.pb.cc
    ${ULTRA_OTASIM_PROTO_GEN_DIR}/ota_simulator.grpc.pb.cc)
set(ULTRA_OTASIM_PROTO_HDRS
    ${ULTRA_OTASIM_PROTO_GEN_DIR}/ota_simulator.pb.h
    ${ULTRA_OTASIM_PROTO_GEN_DIR}/ota_simulator.grpc.pb.h)

add_custom_command(
    OUTPUT ${ULTRA_OTASIM_PROTO_SRCS} ${ULTRA_OTASIM_PROTO_HDRS}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ULTRA_OTASIM_PROTO_GEN_DIR}
    COMMAND ${ULTRA_PROTOC_EXECUTABLE}
            --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
            --cpp_out=${ULTRA_OTASIM_PROTO_GEN_DIR}
            --grpc_out=${ULTRA_OTASIM_PROTO_GEN_DIR}
            --plugin=protoc-gen-grpc=${ULTRA_GRPC_CPP_PLUGIN}
            ${ULTRA_OTASIM_PROTO_FILE}
    DEPENDS ${ULTRA_OTASIM_PROTO_FILE}
    VERBATIM)

add_library(otasim_proto STATIC
    ${ULTRA_OTASIM_PROTO_SRCS}
    ${ULTRA_OTASIM_PROTO_HDRS}
)
target_include_directories(otasim_proto PUBLIC
    ${ULTRA_OTASIM_PROTO_GEN_DIR}
)
target_link_libraries(otasim_proto PUBLIC
    ${ULTRA_GRPC_GRPCPP_TARGET}
    ${ULTRA_PROTOBUF_LIB_TARGET}
)

if(ULTRA_USE_LIBHAMLIB)
    if(WIN32)
        set(HAMLIB_ZIP
            ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hamlib-windows/hamlib-w64-4.7.1.zip)
        set(HAMLIB_EXTRACT_DIR ${CMAKE_BINARY_DIR}/thirdparty/hamlib)
        set(HAMLIB_ROOT ${HAMLIB_EXTRACT_DIR}/hamlib-w64-4.7.1)
        if(NOT EXISTS ${HAMLIB_ROOT}/bin/libhamlib-4.dll)
            if(NOT EXISTS ${HAMLIB_ZIP})
                message(FATAL_ERROR "Vendored Hamlib zip not found: ${HAMLIB_ZIP}")
            endif()
            file(MAKE_DIRECTORY ${HAMLIB_EXTRACT_DIR})
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E tar xf ${HAMLIB_ZIP}
                WORKING_DIRECTORY ${HAMLIB_EXTRACT_DIR}
                RESULT_VARIABLE _hl_extract_rc
            )
            if(NOT _hl_extract_rc EQUAL 0)
                message(FATAL_ERROR "Failed to extract vendored Hamlib zip")
            endif()
        endif()

        if(MSVC)
            set(HAMLIB_LIB ${HAMLIB_EXTRACT_DIR}/libhamlib-4.lib)
            if(NOT EXISTS ${HAMLIB_LIB})
                execute_process(
                    COMMAND ${CMAKE_AR}
                            /def:${HAMLIB_ROOT}/lib/msvc/libhamlib-4.def
                            /machine:x64
                            /out:${HAMLIB_LIB}
                    WORKING_DIRECTORY ${HAMLIB_EXTRACT_DIR}
                    RESULT_VARIABLE _hl_lib_rc
                )
                if(NOT _hl_lib_rc EQUAL 0 OR NOT EXISTS ${HAMLIB_LIB})
                    message(FATAL_ERROR "Failed to generate MSVC Hamlib import library")
                endif()
            endif()
        elseif(MINGW)
            set(HAMLIB_LIB ${HAMLIB_ROOT}/lib/gcc/libhamlib-4.lib)
        else()
            message(FATAL_ERROR "ULTRA_USE_LIBHAMLIB on Windows requires MSVC or MinGW")
        endif()

        # Hamlib's prebuilt Windows headers were generated under MinGW
        # and reference <pthread.h>, which MSVC does not ship. Inject a
        # minimal pthread.h shim ahead of Hamlib's include dir so rig.h
        # parses cleanly. The shim only declares pthread_t and
        # pthread_mutex_t with sizes matching winpthreads' canonical
        # layout — Hamlib uses these as opaque struct members, and the
        # real pthread runtime lives in libwinpthread-1.dll alongside
        # libhamlib-4.dll. We never call pthread API from our code.
        set(HAMLIB_PTHREAD_SHIM
            ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/hamlib-windows/pthread_shim)
        set(HAMLIB_INCLUDE_DIRS
            ${HAMLIB_PTHREAD_SHIM}
            ${HAMLIB_ROOT}/include)
        set(HAMLIB_LIBRARIES ${HAMLIB_LIB})
        set(HAMLIB_RUNTIME_DLLS
            ${HAMLIB_ROOT}/bin/libhamlib-4.dll
            ${HAMLIB_ROOT}/bin/libusb-1.0.dll
            ${HAMLIB_ROOT}/bin/libgcc_s_seh-1.dll
            ${HAMLIB_ROOT}/bin/libwinpthread-1.dll
        )
        set(HAMLIB_LICENSE_FILE ${HAMLIB_ROOT}/COPYING.LIB.txt)
        set(HAMLIB_FOUND TRUE)
    else()
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(HAMLIB REQUIRED IMPORTED_TARGET hamlib)
        set(HAMLIB_FOUND TRUE)
    endif()
endif()

if(HAMLIB_FOUND)
    add_library(ultra_hamlib INTERFACE)
    target_compile_definitions(ultra_hamlib INTERFACE ULTRA_HAVE_LIBHAMLIB=1)
    if(WIN32)
        target_include_directories(ultra_hamlib INTERFACE ${HAMLIB_INCLUDE_DIRS})
        target_link_libraries(ultra_hamlib INTERFACE ${HAMLIB_LIBRARIES})
    else()
        target_link_libraries(ultra_hamlib INTERFACE PkgConfig::HAMLIB)
    endif()
endif()

function(ultra_link_hamlib target_name)
    if(TARGET ultra_hamlib)
        target_link_libraries(${target_name} PRIVATE ultra_hamlib)
        if(WIN32)
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${HAMLIB_RUNTIME_DLLS}
                    $<TARGET_FILE_DIR:${target_name}>
                COMMAND_EXPAND_LISTS
                VERBATIM
            )
        endif()
    endif()
endfunction()

execute_process(
    COMMAND git rev-parse --verify HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
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
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE ULTRA_RELEASE_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(NOT ULTRA_RELEASE_TAG)
    set(ULTRA_RELEASE_TAG "")
endif()

execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE ULTRA_GIT_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
if(ULTRA_GIT_STATUS STREQUAL "")
    set(ULTRA_GIT_DIRTY_BOOL false)
else()
    set(ULTRA_GIT_DIRTY_BOOL true)
endif()
set(ULTRA_BUILD_OS "${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
string(TIMESTAMP ULTRA_BUILD_TIME_UTC "%Y-%m-%dT%H:%M:%SZ" UTC)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ultra/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include/ultra/version.hpp
    @ONLY
)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ultra/build_info.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include/ultra/build_info.hpp
    @ONLY
)

# SDL2 detection — used by both ultra_gui (always required) and cli_simulator
# (optional, for hardware audio I/O in --role A|B mode). Detect once at the
# top level so both targets share the same libs.
find_package(SDL2 QUIET)
if(NOT SDL2_FOUND)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(SDL2 sdl2)
    endif()
endif()
if(SDL2_FOUND)
    message(STATUS "Found SDL2: cli_simulator hardware audio enabled")
else()
    message(STATUS "SDL2 not found: cli_simulator --role A|B will be unavailable")
endif()

add_library(ota_channel_core STATIC
    src/ota_channel_core/rng.cpp
    src/ota_channel_core/models.cpp
    src/ota_channel_core/mixer.cpp
    src/ota_channel_core/session_context.cpp
    src/ota_channel_core/session_manager.cpp
    src/ota_channel_core/channel.cpp
)
set_target_properties(ota_channel_core PROPERTIES OUTPUT_NAME ota_channel_core)
target_include_directories(ota_channel_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ota_channel_core
)
target_include_directories(ota_channel_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
)
target_link_libraries(ota_channel_core PUBLIC Threads::Threads)

add_library(ota_simulator_service STATIC
    src/ota_simulator_service/auth_allowlist.cpp
    src/ota_simulator_service/audio_packet.cpp
    src/ota_simulator_service/audio_plane.cpp
    src/ota_simulator_service/capture_writer.cpp
    src/ota_simulator_service/ota_simulator_service.cpp
    src/tnc/socket_compat.cpp
)
set_target_properties(ota_simulator_service PROPERTIES OUTPUT_NAME ota_simulator_service)
target_include_directories(ota_simulator_service PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ota_simulator_service
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${ULTRA_OTASIM_PROTO_GEN_DIR}
)
target_include_directories(ota_simulator_service PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include
)
target_link_libraries(ota_simulator_service PUBLIC
    ota_channel_core
    otasim_proto
    Threads::Threads
)
if(WIN32)
    target_link_libraries(ota_simulator_service PRIVATE ws2_32)
endif()

add_library(ultra_otasim_client STATIC
    src/otasim_client/ota_audio_backend.cpp
)
target_include_directories(ultra_otasim_client PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(ultra_otasim_client PUBLIC
    ota_simulator_service
)
if(WIN32)
    target_link_libraries(ultra_otasim_client PRIVATE ws2_32)
endif()
ultra_enable_tool_test_warnings(
    src/ota_simulator_service/auth_allowlist.cpp
    src/ota_simulator_service/audio_packet.cpp
    src/ota_simulator_service/audio_plane.cpp
    src/ota_simulator_service/capture_writer.cpp
    src/ota_simulator_service/ota_simulator_service.cpp
)

# Core library
add_library(ultra_core STATIC
    src/ofdm/modulator.cpp
    src/ofdm/ofdm_demodulator_setup.cpp
    src/ofdm/ofdm_symbol_demap.cpp
    src/ofdm/ofdm_stream_processor.cpp
    src/ofdm/ofdm_sync_adapter.cpp
    src/ofdm/ofdm_sync.cpp
    src/ofdm/channel_equalizer_baseband.cpp
    src/ofdm/channel_equalizer_lts.cpp
    src/ofdm/channel_equalizer_pilot.cpp
    src/ofdm/channel_equalizer_equalize.cpp
    src/fec/ldpc_encoder.cpp
    src/fec/ldpc_decoder.cpp
    src/fec/ldpc_codec.cpp
    src/fec/codec_factory.cpp
    src/fec/frame_interleaver.cpp
    src/fec/burst_interleaver.cpp
    src/fec/carrier_ldpc_interleaver.cpp
    src/fec/soft_combine.cpp
    src/dsp/fft.cpp
    src/dsp/filters.cpp
    src/dsp/resampler.cpp
    # Protocol layer (callsign addressing, connection management)
    src/protocol/frame_v2.cpp
    src/protocol/arq_interface.cpp
    src/protocol/arq.cpp
    src/protocol/selective_repeat_arq.cpp
    src/protocol/connection.cpp
    src/protocol/connection_handlers.cpp
    src/protocol/protocol_engine.cpp
    src/protocol/file_transfer.cpp
    src/protocol/compression.cpp
    src/gui/image_util.cpp
    # Local diagnostics and field-report bundle support
    src/diagnostics/audio_ring.cpp
    src/diagnostics/event_buffer.cpp
    src/diagnostics/report_bundle.cpp
    src/diagnostics/redaction.cpp
    src/diagnostics/session_summary.cpp
    src/diagnostics/diagnostics_recorder.cpp
    # VARA-compatible TNC protocol shell
    # (tnc_bridge.cpp is added conditionally below — depends on SDL2 via
    #  AudioEngine, so it cannot live in the unconditional ultra_core
    #  sources. Builds without SDL2 still get session + server.)
    src/tnc/tnc_session.cpp
    src/tnc/socket_compat.cpp
    src/tnc/tnc_server.cpp
    # Waveform abstraction layer
    src/waveform/mc_dpsk_waveform.cpp
    src/waveform/ofdm_cox_waveform.cpp
    src/waveform/ofdm_chirp_waveform.cpp
    src/waveform/waveform_factory.cpp
    # miniz compression library (public domain)
    thirdparty/miniz/miniz.c
    thirdparty/miniz/miniz_tdef.c
    thirdparty/miniz/miniz_tinfl.c
    thirdparty/miniz/miniz_zip.c
)

target_include_directories(ultra_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/generated/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src  # For protocol headers
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty  # For miniz
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/stb  # Header-only image decode/resize/write
)

target_link_libraries(ultra_core PUBLIC
    Threads::Threads
)

if(WIN32)
    target_link_libraries(ultra_core PRIVATE ws2_32)
    # Ensure <windows.h> never pulls in the legacy winsock1 headers project-
    # wide. tnc_bridge.cpp includes audio_engine.hpp (SDL.h → windows.h),
    # then later tnc_server.hpp (→ winsock2.h). Without this, both winsock
    # versions get included in the same TU and redefine sockaddr/fd_set/etc.
    target_compile_definitions(ultra_core PUBLIC WIN32_LEAN_AND_MEAN NOMINMAX)
endif()

if(SDL2_FOUND)
    if(TARGET SDL2::SDL2)
        target_link_libraries(ultra_core PUBLIC SDL2::SDL2)
    else()
        target_include_directories(ultra_core PUBLIC ${SDL2_INCLUDE_DIRS})
        target_link_libraries(ultra_core PUBLIC ${SDL2_LIBRARIES})
    endif()
    # TNCBridge wraps AudioEngine (SDL2-only). Add to ultra_core only when
    # SDL2 is available; otherwise CI's Test/Sanitizer/Coverage steps that
    # build without SDL2 fail to compile the unconditional core.
    target_sources(ultra_core PRIVATE src/tnc/tnc_bridge.cpp)
endif()

# Shared GUI/modem runtime source lists. Keep these centralized so CLI tools,
# TNC, simulator, and GUI builds cannot silently drift apart.
set(ULTRA_STREAMING_DECODER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/modem/idle_noise_snr_estimator.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/modem/streaming_decoder.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/modem/streaming_sync_acquisition.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/modem/streaming_ofdm_decode.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/modem/streaming_burst_interleave.cpp
)

set(ULTRA_STREAMING_MODEM_SOURCES
    ${ULTRA_STREAMING_DECODER_SOURCES}
    src/gui/modem/streaming_encoder.cpp
)

set(ULTRA_AUDIO_ENGINE_SOURCE
    src/gui/audio_engine.cpp
)

set(ULTRA_PTT_SOURCES
    src/gui/serial_ptt.cpp
    src/ptt/serial_ptt_driver.cpp
    src/ptt/cat_ptt_driver.cpp
    src/ptt/ptt_driver_factory.cpp
)

if(HAMLIB_FOUND)
    list(APPEND ULTRA_PTT_SOURCES
        src/ptt/hamlib_rig_driver.cpp
        src/ptt/hamlib_rig_list.cpp
    )
endif()

set(ULTRA_MODEM_ENGINE_SOURCES
    src/gui/modem/modem_engine.cpp
    src/gui/modem/modem_rx.cpp
    src/gui/modem/modem_rx_decode.cpp
    src/gui/modem/modem_mode.cpp
    src/gui/modem/modem_carrier_sense.cpp
    ${ULTRA_STREAMING_MODEM_SOURCES}
    src/gui/adaptive_mode.cpp
)

# Command-line modem tool
if(ULTRA_BUILD_TOOLS)
    add_library(ultra_tool_wav_io STATIC
        tools/io/wav_io.cpp
    )
    ultra_enable_tool_test_warnings(tools/io/wav_io.cpp)
    target_include_directories(ultra_tool_wav_io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ultra_tool_wav_io PUBLIC ultra_core)

    add_library(ultra_sim_station STATIC
        tools/sim/simulated_station.cpp
        ${ULTRA_STREAMING_MODEM_SOURCES}
    )
    ultra_enable_tool_test_warnings(tools/sim/simulated_station.cpp)
    target_include_directories(ultra_sim_station PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ultra_sim_station PUBLIC ultra_core)
    target_link_libraries(ultra_sim_station PUBLIC ota_channel_core)

    add_executable(ultra
        src/main.cpp
        ${ULTRA_MODEM_ENGINE_SOURCES}
    )
    target_include_directories(ultra PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ultra PRIVATE ultra_core ota_channel_core)

    # CLI Simulator - Uses IWaveform + StreamingDecoder/Encoder directly (NOT ModemEngine)
    add_executable(cli_simulator
        tools/cli_simulator.cpp
    )
    ultra_enable_tool_test_warnings(tools/cli_simulator.cpp)
    target_include_directories(cli_simulator PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(cli_simulator PRIVATE ultra_sim_station ultra_otasim_client)

    # otasim_ctl - admin CLI for the OTASim server (SetChannel etc.)
    add_executable(otasim_ctl
        tools/otasim_ctl.cpp
    )
    ultra_enable_tool_test_warnings(tools/otasim_ctl.cpp)
    target_link_libraries(otasim_ctl PRIVATE ultra_otasim_client)

    add_executable(session_decode
        tools/session_decode.cpp
    )
    ultra_enable_tool_test_warnings(tools/session_decode.cpp)
    target_include_directories(session_decode PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(session_decode PRIVATE ultra_sim_station ultra_tool_wav_io)

    add_executable(ofdm_snr_probe
        tools/ofdm_snr_probe.cpp
    )
    ultra_enable_tool_test_warnings(tools/ofdm_snr_probe.cpp)
    target_include_directories(ofdm_snr_probe PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ofdm_snr_probe PRIVATE ultra_sim_station)

    add_executable(idle_snr_probe
        tools/idle_snr_probe.cpp
    )
    ultra_enable_tool_test_warnings(tools/idle_snr_probe.cpp)
    target_include_directories(idle_snr_probe PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(idle_snr_probe PRIVATE ultra_sim_station)

    add_library(ultra_replay_lib STATIC
        src/replay/bundle_loader.cpp
        src/replay/event_timeline.cpp
        src/replay/replay_runner.cpp
        src/replay/divergence_report.cpp
        ${ULTRA_STREAMING_DECODER_SOURCES}
    )
    ultra_enable_tool_test_warnings(
        src/replay/bundle_loader.cpp
        src/replay/event_timeline.cpp
        src/replay/replay_runner.cpp
        src/replay/divergence_report.cpp
    )
    target_include_directories(ultra_replay_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ultra_replay_lib PUBLIC ultra_tool_wav_io)

    add_executable(ultra_replay
        tools/ultra_replay.cpp
    )
    ultra_enable_tool_test_warnings(tools/ultra_replay.cpp)
    target_include_directories(ultra_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ultra_replay PRIVATE ultra_replay_lib)

    add_executable(ota_simulator
        tools/ota_simulator.cpp
        tools/ota_simulator/scenario.cpp
        tools/ota_simulator/scripted_audio_port.cpp
        tools/ota_simulator/runner.cpp
        tools/ota_simulator/runner_v2.cpp
        tools/ota_simulator/clip_gen.cpp
        tools/ota_simulator/session_log.cpp
    )
    ultra_enable_tool_test_warnings(
        tools/ota_simulator.cpp
        tools/ota_simulator/scenario.cpp
        tools/ota_simulator/scripted_audio_port.cpp
        tools/ota_simulator/runner.cpp
        tools/ota_simulator/runner_v2.cpp
        tools/ota_simulator/clip_gen.cpp
        tools/ota_simulator/session_log.cpp
    )
    target_include_directories(ota_simulator PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
        ${CMAKE_CURRENT_BINARY_DIR}/generated/include
    )
    target_link_libraries(ota_simulator PRIVATE ultra_sim_station ultra_tool_wav_io ota_simulator_service)

    add_executable(ultra_report
        tools/ultra_report.cpp
    )
    ultra_enable_tool_test_warnings(tools/ultra_report.cpp)
    target_include_directories(ultra_report PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(ultra_report PRIVATE ultra_core)

    # Optional hardware-audio support (--role A|B). Reuses the GUI's
    # AudioEngine class; SDL2 is detected at top level above.
    if(SDL2_FOUND)
        target_sources(cli_simulator PRIVATE
            ${ULTRA_AUDIO_ENGINE_SOURCE}
        )
        # SDL_MAIN_HANDLED tells SDL.h not to rename our int main() to
        # SDL_main. Required for CLI tools on Windows; otherwise the linker
        # can't find main() and reports LNK2019 / LNK1120. ultra_gui keeps
        # SDL's main wrapping because it's a windowed SDL app.
        target_compile_definitions(cli_simulator PRIVATE
            ULTRA_HAVE_SDL2
            SDL_MAIN_HANDLED
        )
        if(TARGET SDL2::SDL2)
            target_link_libraries(cli_simulator PRIVATE SDL2::SDL2)
        else()
            target_include_directories(cli_simulator PRIVATE ${SDL2_INCLUDE_DIRS})
            target_link_libraries(cli_simulator PRIVATE ${SDL2_LIBRARIES})
        endif()
    endif()

    if(SDL2_FOUND)
        add_executable(ultra_tnc
            tools/ultra_tnc.cpp
            tools/ultra_tnc_config.cpp
            ${ULTRA_AUDIO_ENGINE_SOURCE}
            ${ULTRA_PTT_SOURCES}
            ${ULTRA_STREAMING_MODEM_SOURCES}
        )
        target_include_directories(ultra_tnc PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
            ${CMAKE_CURRENT_SOURCE_DIR}/tools
        )
        target_compile_definitions(ultra_tnc PRIVATE
            ULTRA_HAVE_SDL2
            SDL_MAIN_HANDLED
        )
        target_link_libraries(ultra_tnc PRIVATE ultra_core ota_channel_core ultra_otasim_client)
        ultra_link_hamlib(ultra_tnc)
    else()
        message(STATUS "ultra_tnc disabled: SDL2 not found")
    endif()

    # Decode bench - generate + decode deterministic WAV fixtures so
    # AI agents and humans can A/B decoder changes against bit-identical
    # input. No SDL, no real-time pacing, runs as fast as possible.
    add_executable(decode_bench
        tools/decode_bench.cpp
        ${ULTRA_STREAMING_MODEM_SOURCES}
    )
    ultra_enable_tool_test_warnings(tools/decode_bench.cpp)
    target_include_directories(decode_bench PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(decode_bench PRIVATE ultra_tool_wav_io ota_channel_core)

    # otasim_audio_demo - generates a ~60s WAV of channel-model audio
    # for human listening (sanity check / iCloud sample).
    add_executable(otasim_audio_demo tools/otasim_audio_demo.cpp)
    target_include_directories(otasim_audio_demo PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(otasim_audio_demo PRIVATE
        ultra_tool_wav_io ota_channel_core)

    # Threaded simulator - proper real-time audio simulation
    add_executable(threaded_simulator
        tools/threaded_simulator.cpp
        ${ULTRA_MODEM_ENGINE_SOURCES}
    )
    ultra_enable_tool_test_warnings(tools/threaded_simulator.cpp)
    target_include_directories(threaded_simulator PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(threaded_simulator PRIVATE ultra_core)

    # Simple waveform test (clean TX/RX with consistent config)
    # Decoder-only by design; do not pull in the encoder runtime here.
    add_executable(test_waveform_simple
        tools/test_waveform_simple.cpp
        ${ULTRA_STREAMING_DECODER_SOURCES}
    )
    ultra_enable_tool_test_warnings(tools/test_waveform_simple.cpp)
    target_include_directories(test_waveform_simple PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
    )
    target_link_libraries(test_waveform_simple PRIVATE ultra_core ota_channel_core)
endif()

# GUI Application
if(ULTRA_BUILD_GUI)
    # SDL2 detection is done at top level (shared with cli_simulator).

    find_package(OpenGL QUIET)

    if(SDL2_FOUND AND OPENGL_FOUND)
        # Dear ImGui sources
        set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui)

        add_executable(ultra_gui
            src/gui/main_gui.cpp
            src/gui/app.cpp
            ${ULTRA_AUDIO_ENGINE_SOURCE}
            ${ULTRA_PTT_SOURCES}
            ${ULTRA_MODEM_ENGINE_SOURCES}
            src/gui/widgets/constellation.cpp
            src/gui/widgets/controls.cpp
            src/gui/widgets/status.cpp
            src/gui/widgets/settings.cpp
            src/gui/widgets/waterfall.cpp
            src/gui/widgets/file_browser.cpp
            # ImGui core
            ${IMGUI_DIR}/imgui.cpp
            ${IMGUI_DIR}/imgui_draw.cpp
            ${IMGUI_DIR}/imgui_tables.cpp
            ${IMGUI_DIR}/imgui_widgets.cpp
            # ImGui backends (OpenGL2 default + SDL renderer software fallback)
            ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
            ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
            ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer2.cpp
        )

        target_include_directories(ultra_gui PRIVATE
            ${IMGUI_DIR}
            ${IMGUI_DIR}/backends
            ${SDL2_INCLUDE_DIRS}
        )

        target_link_libraries(ultra_gui PRIVATE
            ultra_core
            ota_channel_core
            ultra_otasim_client
            ${SDL2_LIBRARIES}
            ${OPENGL_LIBRARIES}
        )
        ultra_link_hamlib(ultra_gui)

        # Platform-specific settings
        if(APPLE)
            target_link_libraries(ultra_gui PRIVATE
                "-framework OpenGL"
                "-framework Cocoa"
                "-framework IOKit"
                "-framework CoreVideo"
            )
        elseif(WIN32)
            target_link_libraries(ultra_gui PRIVATE
                opengl32
                imm32
                dbghelp
            )
        elseif(UNIX)
            target_link_libraries(ultra_gui PRIVATE dl)
        endif()

        message(STATUS "GUI: Enabled (SDL2 + OpenGL 2.1)")
    else()
        message(STATUS "GUI: Disabled (SDL2 or OpenGL not found)")
        message(STATUS "  Install SDL2: brew install sdl2 (macOS) or apt install libsdl2-dev (Linux)")
    endif()
endif()

# Tests
if(ULTRA_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
