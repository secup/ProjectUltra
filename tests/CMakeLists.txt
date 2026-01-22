# Test suite for ProjectUltra

# For now, we rely on the built-in self-test in main.cpp
# Future: Add Google Test or Catch2 based unit tests

# CRITICAL: Layer-by-layer OFDM verification tests
# These tests MUST pass 100% before trusting any higher-level functionality
add_executable(test_layers test_layers.cpp)
target_link_libraries(test_layers PRIVATE ultra_core)
add_test(NAME Layers COMMAND test_layers)

# Basic OFDM test - stripped down, no LDPC, no heuristics
# Use this to verify the core OFDM path before adding complexity
add_executable(test_basic_ofdm test_basic_ofdm.cpp)
target_link_libraries(test_basic_ofdm PRIVATE ultra_core)
add_test(NAME BasicOFDM COMMAND test_basic_ofdm --loopback)

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft PRIVATE ultra_core)
add_test(NAME FFT COMMAND test_fft)

add_executable(test_ofdm test_ofdm.cpp)
target_link_libraries(test_ofdm PRIVATE ultra_core)
add_test(NAME OFDM COMMAND test_ofdm)

# Modem loopback test (comprehensive pipeline verification)
# Tests FFT roundtrip, carrier mapping, modulate->demodulate loopback
add_executable(test_modem_loopback test_modem_loopback.cpp)
target_link_libraries(test_modem_loopback PRIVATE ultra_core)
add_test(NAME ModemLoopback COMMAND test_modem_loopback)

# Basic audio path test (no OFDM/LDPC - just raw samples)
# Use this to verify audio path between laptops
add_executable(test_audio_basic test_audio_basic.cpp)
add_test(NAME AudioBasic COMMAND test_audio_basic)

# WAV file loopback test (generate/decode WAV files for cross-wire testing)
add_executable(test_wav_loopback test_wav_loopback.cpp)
target_link_libraries(test_wav_loopback PRIVATE ultra_core)
add_test(NAME WavLoopback COMMAND test_wav_loopback --loopback)

add_executable(test_protocol test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE ultra_core)
add_test(NAME Protocol COMMAND test_protocol)

add_executable(test_interleaver test_interleaver.cpp)
target_link_libraries(test_interleaver PRIVATE ultra_core)
add_test(NAME Interleaver COMMAND test_interleaver)

# Multi-codeword LDPC test (file transfer reliability)
add_executable(test_multiblock_ldpc test_multiblock_ldpc.cpp)
target_link_libraries(test_multiblock_ldpc PRIVATE ultra_core)
add_test(NAME MultiblockLDPC COMMAND test_multiblock_ldpc)

# Protocol + Modem integration test (two-modem architecture - CRITICAL)
# Tests protocol through full modem pipeline, same as GUI Test Mode
add_executable(test_protocol_modem test_protocol_modem.cpp)
target_link_libraries(test_protocol_modem PRIVATE ultra_core)
add_test(NAME ProtocolModem COMMAND test_protocol_modem)

# ITU-R F.1487 HF Channel Benchmark Suite
# Measures SNR thresholds for BER targets under standard channel conditions
add_executable(benchmark_itu_r benchmark_itu_r.cpp)
target_include_directories(benchmark_itu_r PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim
)
target_link_libraries(benchmark_itu_r PRIVATE ultra_core)

# Selective Repeat ARQ tests
add_executable(test_selective_repeat test_selective_repeat.cpp)
target_link_libraries(test_selective_repeat PRIVATE ultra_core)
add_test(NAME SelectiveRepeatARQ COMMAND test_selective_repeat)

# OTFS Unit Tests (full pipeline per CLAUDE.md)
add_executable(test_otfs test_otfs.cpp)
target_link_libraries(test_otfs PRIVATE ultra_core)
add_test(NAME OTFS COMMAND test_otfs)

# OFDM vs OTFS Comparison Benchmark
add_executable(benchmark_otfs benchmark_otfs.cpp)
target_include_directories(benchmark_otfs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim
)
target_link_libraries(benchmark_otfs PRIVATE ultra_core)

# Adaptive Mode Selection Tests
add_executable(test_adaptive_mode test_adaptive_mode.cpp)
target_include_directories(test_adaptive_mode PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim
)
target_link_libraries(test_adaptive_mode PRIVATE ultra_core)
add_test(NAME AdaptiveMode COMMAND test_adaptive_mode)

# MIL-STD-188-110B Comparison Benchmark
# Measures SNR thresholds for BER < 10^-3 and 10^-5 to compare with MIL-STD
add_executable(benchmark_milstd benchmark_milstd.cpp)
target_include_directories(benchmark_milstd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim
)
target_link_libraries(benchmark_milstd PRIVATE ultra_core)

# Adaptive Link Establishment Test (PROBE/CONNECT through full modem - CRITICAL)
# Tests adaptive modulation selection and link establishment at various SNR levels
add_executable(test_adaptive_link test_adaptive_link.cpp)
target_link_libraries(test_adaptive_link PRIVATE ultra_core)
add_test(NAME AdaptiveLink COMMAND test_adaptive_link)

# Sync Detection Accuracy Test
# Tests fine-grained sync peak refinement (fixes ~118 sample offset issue)
add_executable(test_sync_detection test_sync_detection.cpp)
target_link_libraries(test_sync_detection PRIVATE ultra_core)
add_test(NAME SyncDetection COMMAND test_sync_detection)

# Comprehensive modem unit tests (LDPC, LLR, DQPSK, full chain)
# CRITICAL: All these tests must pass for reliable modem operation
add_executable(test_comprehensive_modem test_comprehensive_modem.cpp)
target_link_libraries(test_comprehensive_modem PRIVATE ultra_core)
add_test(NAME ComprehensiveModem COMMAND test_comprehensive_modem)

# RNG consistency test for cross-platform LDPC debugging
add_executable(test_rng test_rng.cpp)

# Protocol v2 frame format tests
add_executable(test_frame_v2 test_frame_v2.cpp)
target_link_libraries(test_frame_v2 PRIVATE ultra_core)
add_test(NAME FrameV2 COMMAND test_frame_v2)

# Protocol v2 + Modem integration test (full LDPC+OFDM loopback)
add_executable(test_frame_v2_modem test_frame_v2_modem.cpp)
target_link_libraries(test_frame_v2_modem PRIVATE ultra_core)
add_test(NAME FrameV2Modem COMMAND test_frame_v2_modem)

# MODE_CHANGE protocol tests (adaptive mode selection)
add_executable(test_mode_change test_mode_change.cpp)
target_link_libraries(test_mode_change PRIVATE ultra_core)
add_test(NAME ModeChange COMMAND test_mode_change)

# Throughput measurement tool
add_executable(test_throughput ../tools/test_throughput.cpp)
target_link_libraries(test_throughput PRIVATE ultra_core)
