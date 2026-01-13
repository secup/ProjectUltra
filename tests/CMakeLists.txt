# Test suite for ProjectUltra

# For now, we rely on the built-in self-test in main.cpp
# Future: Add Google Test or Catch2 based unit tests

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft PRIVATE ultra_core)
add_test(NAME FFT COMMAND test_fft)

add_executable(test_ofdm test_ofdm.cpp)
target_link_libraries(test_ofdm PRIVATE ultra_core)
add_test(NAME OFDM COMMAND test_ofdm)
