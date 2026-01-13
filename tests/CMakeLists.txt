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
