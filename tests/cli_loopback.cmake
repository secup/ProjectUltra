# CLI loopback (2026-11-10): `ultra ptx` builds each frame the way a session sends it and
# `ultra prx` decodes it through ModemEngine, the modem object the GUI and the TNC run. One case
# per frame type and data rung: transmit one frame, decode it, check the decoded line (and, in
# the default all-receivers mode, that exactly one frame is reported). Until this test the CLI
# had its own decoder setup and could not decode MC-DPSK data, 16QAM/32QAM, real narrowband
# frames or a 44.1 kHz WAV (BUG-CLI-PTX-PRX-BROKEN).
#
#   cmake -DULTRA=<path to ultra> -DWORK=<scratch dir> -P cli_loopback.cmake

cmake_minimum_required(VERSION 3.16)
if(NOT ULTRA OR NOT WORK)
    message(FATAL_ERROR "usage: cmake -DULTRA=<ultra binary> -DWORK=<scratch dir> -P cli_loopback.cmake")
endif()
file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")
set(failures 0)

# run_case(<name> <file> <ptx args> <prx args> <stdout regex> <expected frame count or "">)
function(run_case name file ptx_args prx_args expect frames)
    set(audio "${WORK}/${file}")
    execute_process(
        COMMAND "${ULTRA}" ptx ${ptx_args} -s VE2ABC -d VE2XYZ -o "${audio}" --log-level warn
        RESULT_VARIABLE rc OUTPUT_QUIET ERROR_VARIABLE err)
    if(NOT rc EQUAL 0)
        message(SEND_ERROR "${name}: ptx failed (${rc}): ${err}")
        return()
    endif()
    execute_process(
        COMMAND "${ULTRA}" prx ${prx_args} "${audio}"
        RESULT_VARIABLE rc OUTPUT_VARIABLE decoded ERROR_VARIABLE err)
    if(NOT rc EQUAL 0)
        message(SEND_ERROR "${name}: prx failed (${rc}): ${err}")
        return()
    endif()
    if(NOT decoded MATCHES "${expect}")
        message(SEND_ERROR "${name}: expected /${expect}/, prx printed:\n${decoded}")
        return()
    endif()
    if(NOT frames STREQUAL "" AND NOT err MATCHES "Frames: ${frames}\n")
        message(SEND_ERROR "${name}: expected ${frames} frame(s); prx reported:\n${decoded}${err}")
        return()
    endif()
    message(STATUS "${name}: ok")
endfunction()

# Handshake and control frames (MC-DPSK). The PING goes through a 48 kHz WAV.
run_case(ping ping.wav "ping" "-w;mcdpsk" "MC-DPSK +PING" "1")
run_case(connect connect.f32 "connect" "-w;mcdpsk" "CONNECT from VE2ABC" "1")
run_case(disconnect disconnect.f32 "disconnect" "-w;mcdpsk" "DISCONNECT" "1")
run_case(mcdpsk_data mc_data.f32 "Hello MC;-w;mcdpsk" "-w;mcdpsk"
         "MC-DPSK +DATA from 0x[0-9A-F]+, 8 bytes: \"Hello MC\"" "1")

# OFDM data, one frame per rung family. The QPSK frame runs every receiver (the default): the
# frame's own chirp must not also be reported as a PING.
run_case(ofdm_qpsk qpsk.wav "Hello QPSK;-w;ofdm;-m;qpsk;-r;r1_2" ""
         "OFDM QPSK R1/2 +DATA from 0x[0-9A-F]+, 10 bytes: \"Hello QPSK\"" "1")
run_case(ofdm_8psk 8psk.f32 "Hello 8PSK;-w;ofdm;-m;8psk;-r;r2_3" "-w;ofdm;-m;8psk"
         "OFDM 8PSK R2/3 +DATA .*\"Hello 8PSK\"" "1")
run_case(ofdm_16qam 16qam.f32 "Hello 16QAM;-w;ofdm;-m;16qam;-r;r3_4" "-w;ofdm;-m;16qam"
         "OFDM 16QAM R3/4 +DATA .*\"Hello 16QAM\"" "1")
run_case(ofdm_32qam 32qam.f32 "Hello 32QAM;-w;ofdm;-m;32qam;-r;r2_3" "-w;ofdm;-m;32qam;-r;r2_3"
         "OFDM 32QAM R2/3 +DATA .*\"Hello 32QAM\"" "1")

# Narrowband OFDM: the 2048-FFT / 21-carrier geometry, not a label on the wideband one.
run_case(ofdm_narrow narrow.f32 "Hello narrow;-w;narrow;-m;qpsk;-r;r1_4" "-w;narrow;-m;qpsk;-r;r1_4"
         "OFDM narrow QPSK R1/4 +DATA .*\"Hello narrow\"" "1")

# Narrowband control waveform (500 Hz chirp and MC-DPSK), as a station forced to OFDM_NARROW sends
# it. The CONNECT rides the fixed DBPSK handshake profile: until 2026-11-10 a receiver demodulated
# it with the preset's DQPSK on non-orthogonal carriers and no narrowband session could connect
# (BUG-NARROW-MCDPSK-CONNECT).
run_case(narrow_ping nb_ping.f32 "ping;-w;narrow" "-w;mcdpsk" "MC-DPSK +PING" "1")
run_case(narrow_connect nb_connect.f32 "connect;-w;narrow" "-w;mcdpsk" "CONNECT from VE2ABC" "1")
run_case(narrow_disconnect nb_disconnect.f32 "disconnect;-w;narrow" "-w;mcdpsk" "DISCONNECT" "1")
