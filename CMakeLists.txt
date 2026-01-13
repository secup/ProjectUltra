cmake_minimum_required(VERSION 3.16)
project(ProjectUltra VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build options
option(ULTRA_BUILD_TESTS "Build test suite" ON)
option(ULTRA_BUILD_TOOLS "Build command-line tools" ON)
option(ULTRA_BUILD_GUI "Build GUI application" ON)
option(ULTRA_USE_FFTW "Use FFTW3 for FFT (faster)" ON)

# Find dependencies
find_package(Threads REQUIRED)

if(ULTRA_USE_FFTW)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FFTW3 IMPORTED_TARGET fftw3)
    if(FFTW3_FOUND)
        add_compile_definitions(ULTRA_HAS_FFTW)
    endif()
endif()

# Core library
add_library(ultra_core STATIC
    src/ofdm/modulator.cpp
    src/ofdm/demodulator.cpp
    src/ofdm/channel_estimator.cpp
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
)

target_include_directories(ultra_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
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
    )
    target_include_directories(ultra_sim PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sim
    )
    target_link_libraries(ultra_sim PRIVATE ultra_core)
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
            src/gui/simulation.cpp
            src/gui/audio_engine.cpp
            src/gui/modem_engine.cpp
            src/gui/widgets/constellation.cpp
            src/gui/widgets/controls.cpp
            src/gui/widgets/status.cpp
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
