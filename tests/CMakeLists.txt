# Test suite for ProjectUltra

# For now, we rely on the built-in self-test in main.cpp
# Future: Add Google Test or Catch2 based unit tests

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft PRIVATE ultra_core)
add_test(NAME FFT COMMAND test_fft)

add_executable(test_ofdm test_ofdm.cpp)
target_link_libraries(test_ofdm PRIVATE ultra_core)
add_test(NAME OFDM COMMAND test_ofdm)

add_executable(test_protocol test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE ultra_core)
add_test(NAME Protocol COMMAND test_protocol)

add_executable(test_interleaver test_interleaver.cpp)
target_link_libraries(test_interleaver PRIVATE ultra_core)
add_test(NAME Interleaver COMMAND test_interleaver)

add_executable(test_freq_offset test_freq_offset.cpp)
target_link_libraries(test_freq_offset PRIVATE ultra_core)
add_test(NAME FreqOffset COMMAND test_freq_offset)

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

# OFDM vs OTFS Comparison Benchmark
add_executable(benchmark_otfs benchmark_otfs.cpp)
target_include_directories(benchmark_otfs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sim
)
target_link_libraries(benchmark_otfs PRIVATE ultra_core)
