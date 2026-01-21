cmake_minimum_required(VERSION 3.16)
project(ProjectUltra VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# MSVC-specific settings
if(MSVC)
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
option(ULTRA_USE_FFTW "Use FFTW3 for FFT (faster)" ON)

# Find dependencies
find_package(Threads REQUIRED)

if(ULTRA_USE_FFTW)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        # Use fftw3f (single-precision float) since code uses fftwf_* functions
        pkg_check_modules(FFTW3 IMPORTED_TARGET fftw3f)
        if(FFTW3_FOUND)
            add_compile_definitions(ULTRA_HAS_FFTW)
            message(STATUS "Found FFTW3 (float): ${FFTW3_LIBRARIES}")
        else()
            message(STATUS "FFTW3 float library not found - using built-in FFT")
        endif()
    else()
        message(STATUS "PkgConfig not found - FFTW3 detection skipped (will use built-in FFT)")
    endif()
endif()

# Core library
add_library(ultra_core STATIC
    src/ofdm/modulator.cpp
    src/ofdm/demodulator.cpp
    src/ofdm/channel_estimator.cpp
    src/ofdm/adaptive_modem.cpp
    src/otfs/otfs.cpp
    src/fec/ldpc_encoder.cpp
    src/fec/ldpc_decoder.cpp
    src/framing/frame_builder.cpp
    src/framing/frame_parser.cpp
    src/sync/synchronizer.cpp
    src/arq/arq_controller.cpp
    src/dsp/fft.cpp
    src/dsp/filters.cpp
    src/dsp/resampler.cpp
    src/modem/modem.cpp
    # Protocol layer (callsign addressing, connection management)
    src/protocol/frame_v2.cpp
    src/protocol/arq_interface.cpp
    src/protocol/arq.cpp
    src/protocol/selective_repeat_arq.cpp
    src/protocol/connection.cpp
    src/protocol/protocol_engine.cpp
    src/protocol/file_transfer.cpp
    src/protocol/compression.cpp
    # miniz compression library (public domain)
    thirdparty/miniz/miniz.c
    thirdparty/miniz/miniz_tdef.c
    thirdparty/miniz/miniz_tinfl.c
    thirdparty/miniz/miniz_zip.c
)

target_include_directories(ultra_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src  # For protocol headers
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty  # For miniz
)

target_link_libraries(ultra_core PUBLIC
    Threads::Threads
)

if(FFTW3_FOUND)
    target_link_libraries(ultra_core PUBLIC PkgConfig::FFTW3)
endif()

# Command-line modem tool
if(ULTRA_BUILD_TOOLS)
    add_executable(ultra
        src/main.cpp
    )
    target_link_libraries(ultra PRIVATE ultra_core)

    # HF Channel Simulator / Loopback Test
    add_executable(ultra_sim
        src/sim/loopback_test.cpp
        src/gui/adaptive_mode.cpp
    )
    target_include_directories(ultra_sim PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sim
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
    )
    target_link_libraries(ultra_sim PRIVATE ultra_core)

    # CLI Simulator - Command-line version of GUI test mode
    add_executable(cli_simulator
        tools/cli_simulator.cpp
        src/gui/modem_engine.cpp
        src/gui/adaptive_mode.cpp
    )
    target_include_directories(cli_simulator PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(cli_simulator PRIVATE ultra_core)

    # ModemEngine loopback test
    add_executable(test_modem_engine_loopback
        tools/test_modem_engine_loopback.cpp
        src/gui/modem_engine.cpp
        src/gui/adaptive_mode.cpp
    )
    target_include_directories(test_modem_engine_loopback PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_modem_engine_loopback PRIVATE ultra_core)

    # Sync robustness test (comprehensive)
    add_executable(test_sync_robustness
        tools/test_sync_robustness.cpp
        src/gui/modem_engine.cpp
        src/gui/adaptive_mode.cpp
    )
    target_include_directories(test_sync_robustness PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_sync_robustness PRIVATE ultra_core)
endif()

# GUI Application
if(ULTRA_BUILD_GUI)
    # Find SDL2
    find_package(SDL2 QUIET)
    if(NOT SDL2_FOUND)
        # Try pkg-config as fallback
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(SDL2 sdl2)
        endif()
    endif()

    find_package(OpenGL QUIET)

    if(SDL2_FOUND AND OPENGL_FOUND)
        # Dear ImGui sources
        set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui)

        add_executable(ultra_gui
            src/gui/main_gui.cpp
            src/gui/app.cpp
            src/gui/audio_engine.cpp
            src/gui/modem_engine.cpp
            src/gui/adaptive_mode.cpp
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
            # ImGui backends (SDL2 + OpenGL2 for old hardware compatibility)
            ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
            ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
        )

        target_include_directories(ultra_gui PRIVATE
            ${IMGUI_DIR}
            ${IMGUI_DIR}/backends
            ${SDL2_INCLUDE_DIRS}
        )

        target_link_libraries(ultra_gui PRIVATE
            ultra_core
            ${SDL2_LIBRARIES}
            ${OPENGL_LIBRARIES}
        )

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
