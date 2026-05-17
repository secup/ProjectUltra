# Test suite for ProjectUltra

# Layer-by-layer OFDM primitive verification.
add_executable(test_layers test_layers.cpp)
target_link_libraries(test_layers PRIVATE ultra_core)
add_test(NAME Layers COMMAND test_layers)

add_executable(test_fft test_fft.cpp)
target_link_libraries(test_fft PRIVATE ultra_core)
add_test(NAME FFT COMMAND test_fft)

add_executable(test_dsp_primitives test_dsp_primitives.cpp)
target_link_libraries(test_dsp_primitives PRIVATE ultra_core)
add_test(NAME DSPPrimitives COMMAND test_dsp_primitives)

add_executable(test_channel_core_rng test_channel_core_rng.cpp)
target_link_libraries(test_channel_core_rng PRIVATE ota_channel_core)
add_test(NAME ChannelCoreRNG COMMAND test_channel_core_rng)

add_executable(test_channel_core_models test_channel_core_models.cpp)
target_link_libraries(test_channel_core_models PRIVATE ota_channel_core)
add_test(NAME ChannelCoreModels COMMAND test_channel_core_models)

add_executable(test_channel_core_mixer test_channel_core_mixer.cpp)
target_link_libraries(test_channel_core_mixer PRIVATE ota_channel_core)
add_test(NAME ChannelCoreMixer COMMAND test_channel_core_mixer)

add_executable(test_session_context test_session_context.cpp)
target_link_libraries(test_session_context PRIVATE ota_channel_core)
add_test(NAME SessionContext COMMAND test_session_context)

add_executable(test_session_manager test_session_manager.cpp)
target_link_libraries(test_session_manager PRIVATE ota_channel_core)
add_test(NAME SessionManager COMMAND test_session_manager)

add_executable(test_session_isolation test_session_isolation.cpp)
target_link_libraries(test_session_isolation PRIVATE ota_channel_core)
add_test(NAME SessionIsolation COMMAND test_session_isolation)

add_executable(test_ofdm test_ofdm.cpp)
target_link_libraries(test_ofdm PRIVATE ultra_core)
add_test(NAME OFDM COMMAND test_ofdm)

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

add_executable(test_tnc_session test_tnc_session.cpp)
target_link_libraries(test_tnc_session PRIVATE ultra_core ota_channel_core)
if(SDL2_FOUND)
    target_sources(test_tnc_session PRIVATE
        ../tools/ultra_tnc_config.cpp
        ../src/gui/audio_engine.cpp
        ${ULTRA_STREAMING_DECODER_SOURCES}
        ../src/gui/modem/streaming_encoder.cpp
    )
    target_include_directories(test_tnc_session PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_compile_definitions(test_tnc_session PRIVATE
        ULTRA_TNC_TESTING
        SDL_MAIN_HANDLED
    )
endif()
add_test(NAME TNCSession COMMAND test_tnc_session)

# ultra_tnc config parsing — pure unit tests, no SDL/audio.
add_executable(test_ultra_tnc_config
    test_ultra_tnc_config.cpp
    ../tools/ultra_tnc_config.cpp
)
target_include_directories(test_ultra_tnc_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools
)
target_link_libraries(test_ultra_tnc_config PRIVATE ultra_core ota_channel_core)
add_test(NAME UltraTNCConfig COMMAND test_ultra_tnc_config)

add_executable(test_ptt_driver
    test_ptt_driver.cpp
    ../src/gui/serial_ptt.cpp
    ../src/ptt/serial_ptt_driver.cpp
    ../src/ptt/cat_ptt_driver.cpp
    ../src/ptt/ptt_driver_factory.cpp
)
if(HAMLIB_FOUND)
    target_sources(test_ptt_driver PRIVATE
        ../src/ptt/hamlib_rig_driver.cpp
        ../src/ptt/hamlib_rig_list.cpp
    )
endif()
target_include_directories(test_ptt_driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_ptt_driver PRIVATE ultra_core)
ultra_link_hamlib(test_ptt_driver)
add_test(NAME PttDriver COMMAND test_ptt_driver)

if(HAMLIB_FOUND)
    add_executable(test_hamlib_rig_driver
        test_hamlib_rig_driver.cpp
        ../src/gui/serial_ptt.cpp
        ../src/ptt/serial_ptt_driver.cpp
        ../src/ptt/cat_ptt_driver.cpp
        ../src/ptt/ptt_driver_factory.cpp
        ../src/ptt/hamlib_rig_driver.cpp
        ../src/ptt/hamlib_rig_list.cpp
    )
    target_include_directories(test_hamlib_rig_driver PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
    target_link_libraries(test_hamlib_rig_driver PRIVATE ultra_core)
    ultra_link_hamlib(test_hamlib_rig_driver)
    add_test(NAME HamlibRigDriver COMMAND test_hamlib_rig_driver)
endif()

add_executable(test_tnc_server test_tnc_server.cpp)
target_link_libraries(test_tnc_server PRIVATE ultra_core)
add_test(NAME TNCServer COMMAND test_tnc_server)

add_executable(test_diagnostics test_diagnostics.cpp)
target_include_directories(test_diagnostics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_diagnostics PRIVATE ultra_core)
add_test(NAME Diagnostics COMMAND test_diagnostics)

add_executable(test_image_util test_image_util.cpp)
target_include_directories(test_image_util PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_compile_definitions(test_image_util PRIVATE
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_image_util PRIVATE ultra_core)
add_test(NAME ImageUtil COMMAND test_image_util)

if(TARGET ultra_replay_lib)
    add_executable(test_replay
        test_replay.cpp
        ../src/gui/modem/streaming_encoder.cpp
    )
    target_include_directories(test_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_link_libraries(test_replay PRIVATE ultra_replay_lib)
    add_test(NAME Replay COMMAND test_replay)
else()
    message(STATUS "Replay tests disabled: ultra_replay_lib target is not built")
endif()

if(SDL2_FOUND)
    add_executable(test_tnc_bridge
        test_tnc_bridge.cpp
        ../src/gui/audio_engine.cpp
    )
    target_include_directories(test_tnc_bridge PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
    target_compile_definitions(test_tnc_bridge PRIVATE SDL_MAIN_HANDLED)
    target_link_libraries(test_tnc_bridge PRIVATE ultra_core)
    add_test(NAME TNCBridge COMMAND test_tnc_bridge)
else()
    message(STATUS "test_tnc_bridge disabled: SDL2 not found")
endif()

add_executable(test_connection_policy test_connection_policy.cpp)
target_link_libraries(test_connection_policy PRIVATE ultra_core)
add_test(NAME ConnectionPolicy COMMAND test_connection_policy)

add_executable(test_connection_adaptive test_connection_adaptive.cpp)
target_link_libraries(test_connection_adaptive PRIVATE ultra_core)
add_test(NAME ConnectionAdaptive COMMAND test_connection_adaptive)

add_executable(test_connection_call_collision test_connection_call_collision.cpp)
target_link_libraries(test_connection_call_collision PRIVATE ultra_core)
add_test(NAME ConnectionCallCollision COMMAND test_connection_call_collision)

add_executable(test_connection_pong_delay test_connection_pong_delay.cpp)
target_link_libraries(test_connection_pong_delay PRIVATE ultra_core)
add_test(NAME ConnectionPongDelayDeferred COMMAND test_connection_pong_delay deferred)
add_test(NAME ConnectionPongDelayCancelOnConnect COMMAND test_connection_pong_delay cancel_on_connect)
add_test(NAME ConnectionPongDelayRepingRestarts COMMAND test_connection_pong_delay reping_restarts)
add_test(NAME ConnectionPongDelayZeroDelay COMMAND test_connection_pong_delay zero_delay)

add_executable(test_file_transfer_controller test_file_transfer_controller.cpp)
target_link_libraries(test_file_transfer_controller PRIVATE ultra_core)
add_test(NAME FileTransferController COMMAND test_file_transfer_controller)

add_executable(test_interleaver test_interleaver.cpp)
target_link_libraries(test_interleaver PRIVATE ultra_core)
add_test(NAME Interleaver COMMAND test_interleaver)

add_executable(test_codec_factory test_codec_factory.cpp)
target_link_libraries(test_codec_factory PRIVATE ultra_core)
add_test(NAME CodecFactory COMMAND test_codec_factory)

add_executable(test_soft_combine test_soft_combine.cpp)
target_link_libraries(test_soft_combine PRIVATE ultra_core)
add_test(NAME SoftCombine COMMAND test_soft_combine)

# Stop-and-wait ARQ remains available through the ARQ factory. Keep direct
# coverage so this lower-throughput compatibility path cannot silently regress.
add_executable(test_stop_wait_arq test_stop_wait_arq.cpp)
target_link_libraries(test_stop_wait_arq PRIVATE ultra_core)
add_test(NAME StopWaitARQ COMMAND test_stop_wait_arq)

# Multi-codeword LDPC test (file transfer reliability)
add_executable(test_multiblock_ldpc test_multiblock_ldpc.cpp)
target_link_libraries(test_multiblock_ldpc PRIVATE ultra_core)
add_test(NAME MultiblockLDPC COMMAND test_multiblock_ldpc)

# Selective Repeat ARQ tests
add_executable(test_selective_repeat test_selective_repeat.cpp)
target_link_libraries(test_selective_repeat PRIVATE ultra_core)
add_test(NAME SelectiveRepeatARQ COMMAND test_selective_repeat)

add_executable(test_selective_repeat_policy test_selective_repeat_policy.cpp)
target_link_libraries(test_selective_repeat_policy PRIVATE ultra_core)
add_test(NAME SelectiveRepeatPolicy COMMAND test_selective_repeat_policy)

# Sync Detection Accuracy Test
# Tests fine-grained sync peak refinement (fixes ~118 sample offset issue)
add_executable(test_sync_detection test_sync_detection.cpp)
target_link_libraries(test_sync_detection PRIVATE ultra_core)
add_test(NAME SyncDetection COMMAND test_sync_detection)

# Comprehensive modem unit tests for LDPC, LLR, full-chain noise, and CFO.
add_executable(test_comprehensive_modem test_comprehensive_modem.cpp)
target_link_libraries(test_comprehensive_modem PRIVATE ultra_core)
add_test(NAME ComprehensiveModem COMMAND test_comprehensive_modem)

# RNG consistency test for cross-platform LDPC debugging
add_executable(test_rng test_rng.cpp)
add_test(NAME RNG COMMAND test_rng)

# Protocol v2 frame format tests
add_executable(test_frame_v2 test_frame_v2.cpp)
target_link_libraries(test_frame_v2 PRIVATE ultra_core)
add_test(NAME FrameV2 COMMAND test_frame_v2)

add_executable(test_carrier_ldpc_interleaver test_carrier_ldpc_interleaver.cpp)
target_link_libraries(test_carrier_ldpc_interleaver PRIVATE ultra_core)
add_test(NAME CarrierLDPCInterleaver COMMAND test_carrier_ldpc_interleaver)

add_executable(test_ofdm_carrier_mask_plumbing test_ofdm_carrier_mask_plumbing.cpp)
target_link_libraries(test_ofdm_carrier_mask_plumbing PRIVATE ultra_core)
add_test(NAME OFDMCarrierMaskPlumbing COMMAND test_ofdm_carrier_mask_plumbing)

# Current waveform/rate policy and factory coverage
add_executable(test_waveform_policy test_waveform_policy.cpp)
target_link_libraries(test_waveform_policy PRIVATE ultra_core)
add_test(NAME WaveformPolicy COMMAND test_waveform_policy)

add_executable(test_ofdm_link_adaptation test_ofdm_link_adaptation.cpp)
target_link_libraries(test_ofdm_link_adaptation PRIVATE ultra_core)
add_test(NAME OFDMLinkAdaptation COMMAND test_ofdm_link_adaptation)

# Clean baseband loopback for maintained OFDM waveform wrappers. This verifies
# sync detection, training handoff, demodulation, and soft-bit extraction without
# involving ARQ or hardware audio.
add_executable(test_waveform_loopback test_waveform_loopback.cpp)
target_link_libraries(test_waveform_loopback PRIVATE ultra_core)
add_test(NAME WaveformLoopback COMMAND test_waveform_loopback)

add_executable(test_streaming_config
    test_streaming_config.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_streaming_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_config PRIVATE ultra_core)
add_test(NAME StreamingConfig COMMAND test_streaming_config)

add_executable(test_streaming_buffer_policy test_streaming_buffer_policy.cpp)
target_include_directories(test_streaming_buffer_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_buffer_policy PRIVATE ultra_core)
add_test(NAME StreamingBufferPolicy COMMAND test_streaming_buffer_policy)

add_executable(test_streaming_decode_policy test_streaming_decode_policy.cpp)
target_include_directories(test_streaming_decode_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_decode_policy PRIVATE ultra_core)
add_test(NAME StreamingDecodePolicy COMMAND test_streaming_decode_policy)

add_executable(test_streaming_frame_policy test_streaming_frame_policy.cpp)
target_include_directories(test_streaming_frame_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_frame_policy PRIVATE ultra_core)
add_test(NAME StreamingFramePolicy COMMAND test_streaming_frame_policy)

add_executable(test_ping_detector
    test_ping_detector.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
)
target_include_directories(test_ping_detector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_compile_definitions(test_ping_detector PRIVATE
    PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
target_link_libraries(test_ping_detector PRIVATE ultra_core)
add_test(NAME PingDetector COMMAND test_ping_detector)

add_executable(test_streaming_signal_policy test_streaming_signal_policy.cpp)
target_include_directories(test_streaming_signal_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_signal_policy PRIVATE ultra_core)
add_test(NAME StreamingSignalPolicy COMMAND test_streaming_signal_policy)

# Current streaming MC-DPSK encode/decode loopback.
add_executable(test_streaming_mc_dpsk
    test_streaming_mc_dpsk.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_streaming_mc_dpsk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_mc_dpsk PRIVATE ultra_core)
add_test(NAME StreamingMCDPSK COMMAND test_streaming_mc_dpsk)

if(TARGET ultra_sim_station)
    add_executable(test_channel_snr_calibration test_channel_snr_calibration.cpp)
    target_include_directories(test_channel_snr_calibration PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_link_libraries(test_channel_snr_calibration PRIVATE ultra_sim_station)
    add_test(NAME ChannelSNRCalibration COMMAND test_channel_snr_calibration)
    set_tests_properties(ChannelSNRCalibration PROPERTIES
        TIMEOUT 60
        LABELS regression)

    add_executable(test_modem_snr_meter_calibration test_modem_snr_meter_calibration.cpp)
    target_include_directories(test_modem_snr_meter_calibration PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_link_libraries(test_modem_snr_meter_calibration PRIVATE ultra_sim_station)
    add_test(NAME ChannelModemSNRMeterCalibration COMMAND test_modem_snr_meter_calibration)
    set_tests_properties(ChannelModemSNRMeterCalibration PROPERTIES
        TIMEOUT 90
        LABELS regression)

    add_executable(test_idle_noise_snr_calibration test_idle_noise_snr_calibration.cpp)
    target_include_directories(test_idle_noise_snr_calibration PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_link_libraries(test_idle_noise_snr_calibration PRIVATE ultra_sim_station)
    add_test(NAME ChannelIdleNoiseSNRCalibration COMMAND test_idle_noise_snr_calibration)
    set_tests_properties(ChannelIdleNoiseSNRCalibration PROPERTIES
        TIMEOUT 90
        LABELS regression)

    add_executable(test_pong_tx_race_with_ptt_model
        test_pong_tx_race_with_ptt_model.cpp)
    target_include_directories(test_pong_tx_race_with_ptt_model PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_link_libraries(test_pong_tx_race_with_ptt_model PRIVATE ultra_sim_station)
    add_test(NAME PongTxRaceWithPttBugRepro
        COMMAND test_pong_tx_race_with_ptt_model bug_repro)
    add_test(NAME PongTxRaceWithPttFixVerified
        COMMAND test_pong_tx_race_with_ptt_model fix_verified)
    set_tests_properties(
        PongTxRaceWithPttBugRepro
        PongTxRaceWithPttFixVerified
        PROPERTIES
        TIMEOUT 45
        LABELS regression)

    add_executable(test_session_ptt_sweep test_session_ptt_sweep.cpp)
    target_include_directories(test_session_ptt_sweep PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
        ${CMAKE_CURRENT_SOURCE_DIR}/../tools
    )
    target_link_libraries(test_session_ptt_sweep PRIVATE ultra_sim_station)
    foreach(rx_settling_ms 0 100 200 300 500 700 1000 1500 2000)
        add_test(NAME SessionPttSweep_${rx_settling_ms}ms
            COMMAND test_session_ptt_sweep ${rx_settling_ms})
        set_tests_properties(SessionPttSweep_${rx_settling_ms}ms PROPERTIES
            TIMEOUT 75
            LABELS regression
            RUN_SERIAL TRUE)
    endforeach()
else()
    message(STATUS "ChannelSNRCalibration disabled: ultra_sim_station target is not built")
endif()

# Throughput measurement tool
add_executable(test_throughput ../tools/test_throughput.cpp)
target_link_libraries(test_throughput PRIVATE ultra_core)
add_test(NAME Throughput COMMAND test_throughput)

if(TARGET decode_bench)
    add_executable(test_decode_bench_replay test_decode_bench_replay.cpp)
    add_test(NAME DecodeBenchReplay
        COMMAND test_decode_bench_replay $<TARGET_FILE:decode_bench> ${CMAKE_SOURCE_DIR})
else()
    message(STATUS "DecodeBenchReplay disabled: decode_bench target is not built")
endif()

if(TARGET cli_simulator)
    add_test(NAME CLISyntheticNotch
        COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/test_cli_synthetic_notch.sh
                     $<TARGET_FILE:cli_simulator>)
    set_tests_properties(CLISyntheticNotch PROPERTIES
        TIMEOUT 180
        LABELS regression)
endif()

if(TARGET ota_simulator)
    add_test(NAME OTASimulatorSmoke
        COMMAND $<TARGET_FILE:ota_simulator> run
                --scenario tests/fixtures/ota_simulator/smoke_ping_disconnected.json)
    set_tests_properties(OTASimulatorSmoke PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 60
        RUN_SERIAL TRUE
        LABELS regression)

    # Locks in BUG-PING-DETECTOR-001: real-HF OTA PING capture from
    # KC3VPB injected over real-HF noise bed must classify as PING and
    # trigger PONG TX from a DISCONNECTED endpoint.
    add_test(NAME OTASimulatorRealOTAPing
        COMMAND $<TARGET_FILE:ota_simulator> run
                --scenario tests/fixtures/ota_simulator/real_ota_ping_to_pong.json)
    set_tests_properties(OTASimulatorRealOTAPing PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 60
        RUN_SERIAL TRUE
        LABELS regression)

    add_test(NAME OTASimulatorTwoEndpointClean
        COMMAND $<TARGET_FILE:ota_simulator> run
                --scenario tests/fixtures/ota_simulator/two_endpoint_clean_handshake.json)
    set_tests_properties(OTASimulatorTwoEndpointClean PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 120
        RUN_SERIAL TRUE
        LABELS regression)

    # Locks in the call-collision NAK fix: both stations probe each other
    # near-simultaneously, exercise the PROBING + CONNECTING-state acceptance
    # of inbound CONNECT plus the callsign-lex tiebreaker. Pre-fix would NAK.
    add_test(NAME OTASimulatorTwoEndpointSimultaneousProbe
        COMMAND $<TARGET_FILE:ota_simulator> run
                --scenario tests/fixtures/ota_simulator/two_endpoint_simultaneous_probe.json)
    set_tests_properties(OTASimulatorTwoEndpointSimultaneousProbe PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 120
        RUN_SERIAL TRUE
        LABELS regression)

    add_test(NAME OTASimulatorTwoEndpointNoisy
        COMMAND $<TARGET_FILE:ota_simulator> run
                --scenario tests/fixtures/ota_simulator/two_endpoint_noisy_handshake.json)
    set_tests_properties(OTASimulatorTwoEndpointNoisy PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 120
        RUN_SERIAL TRUE
        LABELS regression)

    add_test(NAME OTASimulatorFileTransferSNR15
        COMMAND $<TARGET_FILE:ota_simulator> run
                --scenario tests/fixtures/ota_simulator/two_endpoint_file_transfer_snr15.json)
    set_tests_properties(OTASimulatorFileTransferSNR15 PROPERTIES
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        TIMEOUT 320
        LABELS regression)
endif()

if(ULTRA_BUILD_HARDWARE_TESTS)
    add_test(NAME HardwareSmoke
        COMMAND ${CMAKE_SOURCE_DIR}/agents/run_hardware_smoke.sh)
    set_tests_properties(HardwareSmoke PROPERTIES
        LABELS hardware
        TIMEOUT 900)
endif()

ultra_enable_test_warnings(
    test_layers.cpp
    test_fft.cpp
    test_dsp_primitives.cpp
    test_channel_core_rng.cpp
    test_channel_core_models.cpp
    test_channel_core_mixer.cpp
    test_session_context.cpp
    test_session_manager.cpp
    test_session_isolation.cpp
    test_ofdm.cpp
    test_audio_basic.cpp
    test_wav_loopback.cpp
    test_protocol.cpp
    test_tnc_session.cpp
    test_ultra_tnc_config.cpp
    ../tools/ultra_tnc_config.cpp
    test_ptt_driver.cpp
    ../src/ptt/serial_ptt_driver.cpp
    ../src/ptt/cat_ptt_driver.cpp
    ../src/ptt/ptt_driver_factory.cpp
    test_hamlib_rig_driver.cpp
    ../src/ptt/hamlib_rig_driver.cpp
    ../src/ptt/hamlib_rig_list.cpp
    test_tnc_server.cpp
    test_diagnostics.cpp
    test_replay.cpp
    test_tnc_bridge.cpp
    test_connection_policy.cpp
    test_connection_adaptive.cpp
    test_connection_call_collision.cpp
    test_file_transfer_controller.cpp
    test_interleaver.cpp
    test_codec_factory.cpp
    test_soft_combine.cpp
    test_stop_wait_arq.cpp
    test_multiblock_ldpc.cpp
    test_selective_repeat.cpp
    test_selective_repeat_policy.cpp
    test_sync_detection.cpp
    test_comprehensive_modem.cpp
    test_rng.cpp
    test_frame_v2.cpp
    test_carrier_ldpc_interleaver.cpp
    test_ofdm_carrier_mask_plumbing.cpp
    test_waveform_policy.cpp
    test_ofdm_link_adaptation.cpp
    test_waveform_loopback.cpp
    test_streaming_config.cpp
    test_streaming_buffer_policy.cpp
    test_streaming_decode_policy.cpp
    test_streaming_frame_policy.cpp
    test_ping_detector.cpp
    test_streaming_signal_policy.cpp
    test_streaming_mc_dpsk.cpp
    test_channel_snr_calibration.cpp
    test_modem_snr_meter_calibration.cpp
    test_idle_noise_snr_calibration.cpp
    ../tools/test_throughput.cpp
    test_decode_bench_replay.cpp
)
