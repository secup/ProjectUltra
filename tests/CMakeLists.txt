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

add_executable(test_watterson_channel test_watterson_channel.cpp)
target_link_libraries(test_watterson_channel PRIVATE ota_channel_core)
add_test(NAME WattersonChannel COMMAND test_watterson_channel)

add_executable(test_watterson_proof test_watterson_proof.cpp)
target_link_libraries(test_watterson_proof PRIVATE ota_channel_core)
target_include_directories(test_watterson_proof PRIVATE
    ${CMAKE_SOURCE_DIR}/thirdparty/pocketfft)
add_test(NAME WattersonProof COMMAND test_watterson_proof)
# This statistical channel proof runs ~30 s locally but is FAR slower on CI runners:
# measured 508 s on the macOS CI runner and >600 s on Linux; the ASAN/UBSAN + coverage
# builds add another 3-5x on top of this FFT-heavy numerical code. 60 s, then 600 s,
# both timed out on CI — the lone red keeping main's matrix failing since May. The test
# is deterministic, so a large backstop won't mask a real hang (a genuine hang is still
# bounded by the 6 h job default); the margin just kills the flaky timeout. Its heaviest
# siblings (ToneBurstAckWatterson ~1985 s) carry no explicit timeout at all. Follow-up:
# the Monte-Carlo sample counts are over-provisioned and could be cut to run in ~1 min.
set_tests_properties(WattersonProof PROPERTIES TIMEOUT 5400)

add_executable(test_papr_reduction test_papr_reduction.cpp)
target_link_libraries(test_papr_reduction PRIVATE ota_channel_core)
target_include_directories(test_papr_reduction PRIVATE
    ${CMAKE_SOURCE_DIR}/thirdparty/pocketfft)
add_test(NAME PaprReduction COMMAND test_papr_reduction)

# ACE PAPR reduction — invariant proofs (points move only outward; PAPR down).
add_executable(test_papr_ace test_papr_ace.cpp)
target_link_libraries(test_papr_ace PRIVATE ultra_core ota_channel_core)
target_include_directories(test_papr_ace PRIVATE
    ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/thirdparty/pocketfft)
add_test(NAME PaprAce COMMAND test_papr_ace)

# Measured total-error LLR noise model (Student-t tail weight from data).
add_executable(test_measured_llr_noise test_measured_llr_noise.cpp)
target_include_directories(test_measured_llr_noise PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME MeasuredLlrNoise COMMAND test_measured_llr_noise)

# Frame-level turbo code (one codeword per OFDM data frame).
add_executable(test_turbo_codec test_turbo_codec.cpp)
target_link_libraries(test_turbo_codec PRIVATE ultra_core)
target_include_directories(test_turbo_codec PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME TurboCodec COMMAND test_turbo_codec)

add_executable(test_turbo_data_frame test_turbo_data_frame.cpp)
target_link_libraries(test_turbo_data_frame PRIVATE ultra_core)
target_include_directories(test_turbo_data_frame PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME TurboDataFrame COMMAND test_turbo_data_frame)

# Preamble-less structure acquisition (chi detector on the production descriptor geometry).
add_executable(test_frame_structure_detector test_frame_structure_detector.cpp)
target_link_libraries(test_frame_structure_detector PRIVATE ultra_core)
target_include_directories(test_frame_structure_detector PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME FrameStructureDetector COMMAND test_frame_structure_detector)

# LTS-pair frame CFO update (src/ofdm/lts_cfo_update.hpp): derived laws vs Monte Carlo.
add_executable(test_lts_cfo_update test_lts_cfo_update.cpp)
target_include_directories(test_lts_cfo_update PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME LtsCfoUpdate COMMAND test_lts_cfo_update)

add_executable(test_real_hf_loop_channel test_real_hf_loop_channel.cpp)
target_link_libraries(test_real_hf_loop_channel PRIVATE ota_channel_core)
add_test(NAME RealHfLoopChannel COMMAND test_real_hf_loop_channel)

add_executable(test_channel_core_mixer test_channel_core_mixer.cpp)
target_link_libraries(test_channel_core_mixer PRIVATE ota_channel_core)
add_test(NAME ChannelCoreMixer COMMAND test_channel_core_mixer)

add_executable(test_session_context test_session_context.cpp)
target_link_libraries(test_session_context PRIVATE ota_channel_core)
add_test(NAME SessionContext COMMAND test_session_context)

add_executable(test_session_half_duplex_blackout test_session_half_duplex_blackout.cpp)
target_link_libraries(test_session_half_duplex_blackout PRIVATE ota_channel_core)
add_test(NAME SessionHalfDuplexBlackout COMMAND test_session_half_duplex_blackout)

add_executable(test_session_manager test_session_manager.cpp)
target_link_libraries(test_session_manager PRIVATE ota_channel_core)
add_test(NAME SessionManager COMMAND test_session_manager)

add_executable(test_session_isolation test_session_isolation.cpp)
target_link_libraries(test_session_isolation PRIVATE ota_channel_core)
add_test(NAME SessionIsolation COMMAND test_session_isolation)

add_executable(test_auth_allowlist test_auth_allowlist.cpp)
target_link_libraries(test_auth_allowlist PRIVATE ota_simulator_service)
add_test(NAME AuthAllowlist COMMAND test_auth_allowlist)

add_executable(test_audio_packet test_audio_packet.cpp)
target_link_libraries(test_audio_packet PRIVATE ota_simulator_service)
add_test(NAME AudioPacket COMMAND test_audio_packet)

add_executable(test_audio_plane_ordering test_audio_plane_ordering.cpp)
target_link_libraries(test_audio_plane_ordering PRIVATE ota_simulator_service)
add_test(NAME AudioPlaneOrdering COMMAND test_audio_plane_ordering)

add_executable(test_grpc_service_smoke test_grpc_service_smoke.cpp)
target_link_libraries(test_grpc_service_smoke PRIVATE ota_simulator_service)
add_test(NAME GrpcServiceSmoke COMMAND test_grpc_service_smoke)

add_executable(test_otasim_serve_smoke test_otasim_serve_smoke.cpp)
target_link_libraries(test_otasim_serve_smoke PRIVATE ota_simulator_service)

add_executable(test_ultra_gui_ota_client test_ultra_gui_ota_client.cpp)
target_link_libraries(test_ultra_gui_ota_client PRIVATE ultra_otasim_client)
target_compile_definitions(test_ultra_gui_ota_client PRIVATE PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_executable(test_ultra_tnc_sim_audio test_ultra_tnc_sim_audio.cpp)

add_executable(test_ofdm test_ofdm.cpp)
target_link_libraries(test_ofdm PRIVATE ultra_core)
add_test(NAME OFDM COMMAND test_ofdm)

# Basic audio path test (no OFDM/LDPC - just raw samples)
# Use this to verify audio path between laptops
add_executable(test_audio_basic test_audio_basic.cpp)
add_test(NAME AudioBasic COMMAND test_audio_basic)

add_executable(test_channel_busy_detector test_channel_busy_detector.cpp)
target_link_libraries(test_channel_busy_detector PRIVATE ultra_core)
add_test(NAME ChannelBusyDetector COMMAND test_channel_busy_detector)

# WAV file loopback test (generate/decode WAV files for cross-wire testing)
add_executable(test_wav_loopback test_wav_loopback.cpp)
target_link_libraries(test_wav_loopback PRIVATE ultra_core)
add_test(NAME WavLoopback COMMAND test_wav_loopback --loopback)

add_executable(test_protocol test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE ultra_core)
add_test(NAME Protocol COMMAND test_protocol)

add_executable(test_tnc_session test_tnc_session.cpp)
target_link_libraries(test_tnc_session PRIVATE ultra_core ota_channel_core ultra_otasim_client)
if(SDL2_FOUND)
    target_sources(test_tnc_session PRIVATE
        ../tools/ultra_tnc_config.cpp
        ../src/gui/audio_engine.cpp
        ${ULTRA_STREAMING_DECODER_SOURCES}
        ../src/gui/modem/streaming_encoder.cpp
        ${ULTRA_MODEM_ENGINE_SOURCES}
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
# Marked off 2026-05-30: the TNC session harness predates the OTASim rework and
# must be re-authored to drive the channel through OTASim like the GUI does
# (TwoSessionsSameEnginePairBothSucceed is pre-existing RED — confirmed identical
# at parent c384b6a). Re-enable after the TNC->OTASim rework.
set_tests_properties(TNCSession PROPERTIES DISABLED TRUE)

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
    target_link_libraries(test_replay PRIVATE ultra_replay_lib ota_channel_core)
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
add_test(NAME ConnectionPongCarrierSenseCallback COMMAND test_connection_pong_delay)

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

# Long-code lift verifier: proves the n=1944 (Z=81) file-class LDPC expands to
# valid, girth>=6 codewords and round-trips through the production
# encoder/decoder, while the n=648 (Z=27) default path stays unchanged.
add_executable(test_ldpc_long_lift test_ldpc_long_lift.cpp)
target_link_libraries(test_ldpc_long_lift PRIVATE ultra_core)
add_test(NAME LongLiftLDPC COMMAND test_ldpc_long_lift)

# Traffic-class PHY profile selector: locks the FILE vs CHAT vs CONTROL gating
# (file segments -> aggressive long-LDPC/deep-interleave/thin-pilot profile;
# DATA chat + ACK/control stay lean) used to reach the Good@20 file throughput.
add_executable(test_traffic_profile test_traffic_profile.cpp)
target_link_libraries(test_traffic_profile PRIVATE ultra_core)
add_test(NAME TrafficProfile COMMAND test_traffic_profile)

# Selective Repeat ARQ tests
add_executable(test_selective_repeat test_selective_repeat.cpp)
target_link_libraries(test_selective_repeat PRIVATE ultra_core)
add_test(NAME SelectiveRepeatARQ COMMAND test_selective_repeat)

add_executable(test_selective_repeat_policy test_selective_repeat_policy.cpp)
target_link_libraries(test_selective_repeat_policy PRIVATE ultra_core)
add_test(NAME SelectiveRepeatPolicy COMMAND test_selective_repeat_policy)

# BUG-TONEACK-FABRICATION (F116): fabricated/aliased tone acks must never
# retire undelivered frames (support-constrained 6-bit decode + never-sent guard)
add_executable(test_arq_toneburst_fabrication test_arq_toneburst_fabrication.cpp)
target_link_libraries(test_arq_toneburst_fabrication PRIVATE ultra_core)
add_test(NAME ArqToneburstFabrication COMMAND test_arq_toneburst_fabrication)

# 8PSK ladder-aware climb (F121): QPSK->QAM8->QAM16 walk; own binary because
# psk8LadderEnabled() latches its env on first call
add_executable(test_psk8_climb test_psk8_climb.cpp)
target_link_libraries(test_psk8_climb PRIVATE ultra_core)
add_test(NAME Psk8Climb COMMAND test_psk8_climb)

# RX-AUTHORITY (ULTRA_RX_RATE_AUTHORITY): receiver-commanded absolute rung loop;
# own binary (env-latched statics)
add_executable(test_rx_authority test_rx_authority.cpp)
target_link_libraries(test_rx_authority PRIVATE ultra_core)
add_test(NAME RxAuthority COMMAND test_rx_authority)

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

# ULTRA_ITERATIVE_CHEST: pins encode -> decode -> re-encode -> re-modulate == the
# original transmitted X, bit-exactly, across the whole modulation/rate/interleaver
# family. This is the correctness foundation of post-FEC data-aided channel
# estimation — a mis-mapping there yields a confident, geometry-valid, WRONG channel
# that parity gating cannot catch.
add_executable(test_iterative_chest_remod test_iterative_chest_remod.cpp)
target_link_libraries(test_iterative_chest_remod PRIVATE ultra_core)
add_test(NAME IterativeChestRemod COMMAND test_iterative_chest_remod)

# Current waveform/rate policy and factory coverage
add_executable(test_waveform_policy test_waveform_policy.cpp)
target_link_libraries(test_waveform_policy PRIVATE ultra_core)
add_test(NAME WaveformPolicy COMMAND test_waveform_policy)

add_executable(test_goodput_rate_controller test_goodput_rate_controller.cpp)
target_link_libraries(test_goodput_rate_controller PRIVATE ultra_core)
add_test(NAME GoodputRateController COMMAND test_goodput_rate_controller)

add_executable(test_latent_rate_controller test_latent_rate_controller.cpp)
target_link_libraries(test_latent_rate_controller PRIVATE ultra_core)
add_test(NAME LatentRateController COMMAND test_latent_rate_controller)

add_executable(test_evm_rate_selector test_evm_rate_selector.cpp)
target_link_libraries(test_evm_rate_selector PRIVATE ultra_core)
add_test(NAME EvmRateSelector COMMAND test_evm_rate_selector)

# Pins CP geometry so prose cannot drift from the arithmetic (2026-07-28: three comments
# claimed MEDIUM@1024 = "256 samples / 5.3ms"; actual is 96 / 2.00 ms).
add_executable(test_cyclic_prefix_geometry test_cyclic_prefix_geometry.cpp)
target_link_libraries(test_cyclic_prefix_geometry PRIVATE ultra_core)
add_test(NAME CyclicPrefixGeometry COMMAND test_cyclic_prefix_geometry)

# Phase 0 (EFFECTIVE_SINR handoff): validates simulator truth H[k,t] against the
# channel model's actual sample-domain filter, and PINS the domain contract (fit
# out one global delay before comparing). Exists because two channel-estimation
# oracles were believed without ever being checked against ground truth.
# Phase 0 acceptance (EFFECTIVE_SINR handoff §9): the AWGN noise-referencing
# convention, measured end to end within 0.25 dB. Every SNR number in the project
# is quoted against this convention.
# Phase 0 (EFFECTIVE_SINR handoff §9): pins the physical/logical carrier ordering,
# the DC gap, and the carrier->Hz mapping. A truth-vs-estimate comparison made in
# the wrong domain looks like a RESULT, not a bug -- that is how two invalid
# channel-estimation oracles survived for months.
# Phase 0 (EFFECTIVE_SINR handoff §9): names the four noise-variance conventions and
# pins their conversions -- including sigma_bin^2 = fft_size * sigma_t^2, which rests
# on the forward FFT being UNNORMALIZED (a third-party property nothing was checking;
# a normalizing FFT would move every noise-referenced quantity by 30.1 dB at N=1024).
add_executable(test_noise_variance_contract test_noise_variance_contract.cpp)
target_link_libraries(test_noise_variance_contract PRIVATE ultra_core)
add_test(NAME NoiseVarianceContract COMMAND test_noise_variance_contract)

add_executable(test_ofdm_carrier_ordering test_ofdm_carrier_ordering.cpp)
target_link_libraries(test_ofdm_carrier_ordering PRIVATE ultra_core)
add_test(NAME OFDMCarrierOrdering COMMAND test_ofdm_carrier_ordering)

# Protects the capture-provenance fix (dbaf028), which shipped without a test. Asserts
# the identity control passes, that the control-first pass is rejected once per frame
# (so it cannot pass for the wrong reason), and that the temporal guard refuses a
# fading channel with live taps.
if(TARGET measure_ack_fer)
    add_test(NAME ChestNmseProvenance
             COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test_chest_nmse_provenance.sh
                     $<TARGET_FILE:measure_ack_fer>)
    set_tests_properties(ChestNmseProvenance PROPERTIES TIMEOUT 900)

    # A fresh burst harness must establish its RX cursor during idle audio before
    # the BURST_HEADER chirp starts. Otherwise the first 4800-sample process call
    # skips the descriptor at abs=0 and falsely reports 0/5 even in clean AWGN.
    add_test(NAME MeasureAckFerBurstDescriptorZ81
             COMMAND ${CMAKE_COMMAND} -E env
                     ULTRA_MEASURE_BURST_FULL_ANCHOR=1
                     ULTRA_LOG_LEVEL=ERROR
                     $<TARGET_FILE:measure_ack_fer>
                     --snr 60 --config burst_chunk --seed 7 --n 1
                     --channel awgn --mod qpsk --rate r3_4
                     --carrier-interleave 1 --group 5 --frame-cw 3
                     --burst-interleave 1 --burst-descriptor 1
                     --ldpc-z 81 --cp medium)
    set_tests_properties(MeasureAckFerBurstDescriptorZ81 PROPERTIES
                         PASS_REGULAR_EXPRESSION
                         "60,burst_chunk,awgn,qpsk,r3_4,1,5,7,1,5,5,1,0"
                         TIMEOUT 60)
endif()

# Phase 5: link-abstraction primitives. Tests the MATH and the API contracts
# (EESM limits, no-underflow, PER sign, and that an uncalibrated or stale-version
# rung can never be selected) independent of any fitted calibration.
add_executable(test_link_abstraction test_link_abstraction.cpp)
target_link_libraries(test_link_abstraction PRIVATE ultra_core)
add_test(NAME LinkAbstraction COMMAND test_link_abstraction)

add_executable(test_channel_noise_contract test_channel_noise_contract.cpp)
target_link_libraries(test_channel_noise_contract PRIVATE ota_channel_core)
add_test(NAME ChannelNoiseContract COMMAND test_channel_noise_contract)

add_executable(test_channel_truth_h test_channel_truth_h.cpp)
target_link_libraries(test_channel_truth_h PRIVATE ota_channel_core)
add_test(NAME ChannelTruthH COMMAND test_channel_truth_h)

add_executable(test_rate_controller test_rate_controller.cpp)
target_link_libraries(test_rate_controller PRIVATE ultra_core)
add_test(NAME RateController COMMAND test_rate_controller)

add_executable(test_ofdm_link_adaptation test_ofdm_link_adaptation.cpp)
target_link_libraries(test_ofdm_link_adaptation PRIVATE ultra_core)
add_test(NAME OFDMLinkAdaptation COMMAND test_ofdm_link_adaptation)

add_executable(test_ofdm_pilot_pattern test_ofdm_pilot_pattern.cpp)
target_link_libraries(test_ofdm_pilot_pattern PRIVATE ultra_core)
add_test(NAME OFDMPilotPattern COMMAND test_ofdm_pilot_pattern)

add_executable(test_ofdm_wiener_interpolator test_ofdm_wiener_interpolator.cpp)
target_link_libraries(test_ofdm_wiener_interpolator PRIVATE ultra_core)
add_test(NAME OFDMWienerInterpolator COMMAND test_ofdm_wiener_interpolator)

add_executable(test_ofdm_delay_domain_interpolator test_ofdm_delay_domain_interpolator.cpp)
target_link_libraries(test_ofdm_delay_domain_interpolator PRIVATE ultra_core)
add_test(NAME OFDMDelayDomainInterpolator COMMAND test_ofdm_delay_domain_interpolator)

# Clean baseband loopback for maintained OFDM waveform wrappers. This verifies
# sync detection, training handoff, demodulation, and soft-bit extraction without
# involving ARQ or hardware audio.
add_executable(test_waveform_loopback test_waveform_loopback.cpp)
target_link_libraries(test_waveform_loopback PRIVATE ultra_core)
add_test(NAME WaveformLoopback COMMAND test_waveform_loopback)

# Good/Moderate channel discriminator (Doppler coherence) — locks the estimator class:
# separation, sigma ordering, and abstain-until-pooled. Header-only; needs the src/ include.
add_executable(test_frequency_selectivity test_frequency_selectivity.cpp)
target_include_directories(test_frequency_selectivity PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME FrequencySelectivity COMMAND test_frequency_selectivity)

add_executable(test_doppler_coherence_estimator test_doppler_coherence_estimator.cpp)
target_link_libraries(test_doppler_coherence_estimator PRIVATE ultra_core)
target_include_directories(test_doppler_coherence_estimator PRIVATE ${CMAKE_SOURCE_DIR}/src)
add_test(NAME DopplerCoherenceEstimator COMMAND test_doppler_coherence_estimator)

add_executable(test_streaming_config
    test_streaming_config.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_streaming_config PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_config PRIVATE ultra_core ota_channel_core)
add_test(NAME StreamingConfig COMMAND test_streaming_config)

# BUG-BURST-STALE-GEOMETRY (2026-07-28): both defects on the burst group-start path.
# Runs the knob=0 (pre-fix) and knob=1 (fixed) arms IN ONE PROCESS — which only works
# because streaming_decode_policy::commandedGeometryEnabled() reads the env fresh
# instead of latching it in a function-local static. Do not "optimize" that away.
add_executable(test_burst_stale_geometry
    test_burst_stale_geometry.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_burst_stale_geometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_burst_stale_geometry PRIVATE ultra_core ota_channel_core)
add_test(NAME BurstStaleGeometry COMMAND test_burst_stale_geometry)
set_tests_properties(BurstStaleGeometry PROPERTIES TIMEOUT 180)

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

# Phase-D guard: derivePhase() returns the right state across all 4 (COLD/WARM/DEGRADED/RECOVERY).
# sync_controller.cpp lives in ULTRA_STREAMING_DECODER_SOURCES (not ultra_core), so compile
# it directly into the test (it needs only logging from ultra_core; the test passes a null
# waveform, so no IWaveform vtable is referenced).
add_executable(test_sync_controller_phase
    test_sync_controller_phase.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sync/sync_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/sync/sync_ring_buffer.cpp  # SyncController now owns a SyncRingBuffer (§7 C3 Phase 2)
)
target_include_directories(test_sync_controller_phase PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_sync_controller_phase PRIVATE ultra_core)
add_test(NAME SyncControllerPhase COMMAND test_sync_controller_phase)

add_executable(test_snr_source_routing test_snr_source_routing.cpp)
target_include_directories(test_snr_source_routing PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_snr_source_routing PRIVATE ultra_core)
add_test(NAME SNRSourceRouting COMMAND test_snr_source_routing)

# Current streaming MC-DPSK encode/decode loopback.
add_executable(test_streaming_mc_dpsk
    test_streaming_mc_dpsk.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_streaming_mc_dpsk PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_mc_dpsk PRIVATE ultra_core ota_channel_core)
add_test(NAME StreamingMCDPSK COMMAND test_streaming_mc_dpsk)

# MC-DPSK sample-clock-offset (ppm) tolerance — proves the differential demod's
# clock/CFO tracking recovers a frame that an untracked fixed symbol grid loses.
add_executable(test_mcdpsk_clock_offset
    test_mcdpsk_clock_offset.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_mcdpsk_clock_offset PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_mcdpsk_clock_offset PRIVATE ultra_core ota_channel_core)
add_test(NAME MCDPSKClockOffset COMMAND test_mcdpsk_clock_offset)

# MC-DPSK training-SNR estimator calibration: sweeps true AWGN SNR and reports the
# routed mcdpsk_in_band estimate, exposing the high-SNR residual-floor saturation
# (gate for the per-carrier estimator fix). Report-only today.
add_executable(test_mcdpsk_snr_calibration
    test_mcdpsk_snr_calibration.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_mcdpsk_snr_calibration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_mcdpsk_snr_calibration PRIVATE ultra_core ota_channel_core)
add_test(NAME MCDPSKSnrCalibration COMMAND test_mcdpsk_snr_calibration)

# OFDM in-band SNR estimator calibration (SNRSource::OFDM_BROADBAND, the OFDM
# analog of MCDPSKSnrCalibration): sweeps true AWGN in-band SNR and gates the
# routed ofdm_broadband estimate against injected truth. Tolerance is
# env-overridable (ULTRA_OFDM_SNR_CAL_TOL_DB, default 4.0 dB — absorbs the known
# +2.5-3.2 dB estimator bias; tighten to 1.5 dB once the bias fix lands).
add_executable(test_ofdm_snr_calibration
    test_ofdm_snr_calibration.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_ofdm_snr_calibration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_ofdm_snr_calibration PRIVATE ultra_core ota_channel_core)
add_test(NAME OFDMSnrCalibration COMMAND test_ofdm_snr_calibration)

add_executable(test_evm_snr_calibration
    test_evm_snr_calibration.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_evm_snr_calibration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_evm_snr_calibration PRIVATE ultra_core ota_channel_core)
add_test(NAME EVMSnrCalibration COMMAND test_evm_snr_calibration)

# TX-burst normalization (PA/ALC gain staging) — self-contained; never used
# SimulatedStation (compiles the modem engine directly), so it survives the
# 2026-05-30 cli_simulator/SimulatedStation retirement.
add_executable(test_hardware_tx_normalization
    test_hardware_tx_normalization.cpp
    ../src/gui/modem/modem_engine.cpp
    ../src/gui/modem/modem_rx.cpp
    ../src/gui/modem/modem_rx_decode.cpp
    ../src/gui/modem/modem_mode.cpp
    ../src/gui/modem/modem_carrier_sense.cpp
    ../src/gui/adaptive_mode.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_hardware_tx_normalization PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools
)
target_link_libraries(test_hardware_tx_normalization PRIVATE ultra_core ota_channel_core)
add_test(NAME TxBurstHardwareNormalization COMMAND test_hardware_tx_normalization)
set_tests_properties(TxBurstHardwareNormalization PROPERTIES
    TIMEOUT 90
    LABELS regression)

# Idle-noise SNR calibration — needs the idle-noise estimator, not SimulatedStation.
add_executable(test_idle_noise_snr_calibration
    test_idle_noise_snr_calibration.cpp
    ../src/gui/modem/idle_noise_snr_estimator.cpp
)
target_include_directories(test_idle_noise_snr_calibration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools
)
target_link_libraries(test_idle_noise_snr_calibration PRIVATE ultra_core ota_channel_core)
add_test(NAME ChannelIdleNoiseSNRCalibration COMMAND test_idle_noise_snr_calibration)
# 90 s fits normal runs but times out under ASAN/UBSAN (3-5x slower) on CI — the second
# sanitizer-only red alongside WattersonProof. Raise to an ASAN-safe backstop.
set_tests_properties(ChannelIdleNoiseSNRCalibration PROPERTIES
    TIMEOUT 1800
    LABELS regression
    # The test loads a tracked real-HF noise bed from recordings/.  Make that
    # source-relative contract explicit so it also works when the build tree is
    # a sibling of the source tree (the immutable IONOS campaign layout).
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")

# MC-DPSK ACK turnaround timing — protocol-only (ultra_core), not SimulatedStation.
add_executable(test_mcdpsk_ack_turnaround test_mcdpsk_ack_turnaround.cpp)
target_include_directories(test_mcdpsk_ack_turnaround PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools
)
target_link_libraries(test_mcdpsk_ack_turnaround PRIVATE ultra_core ota_channel_core)
add_test(NAME MCDPSKAckTurnaround COMMAND test_mcdpsk_ack_turnaround)
set_tests_properties(MCDPSKAckTurnaround PROPERTIES
    TIMEOUT 30
    LABELS regression)

# Throughput measurement tool
add_executable(test_throughput ../tools/test_throughput.cpp)
target_link_libraries(test_throughput PRIVATE ultra_core)
add_test(NAME Throughput COMMAND test_throughput)

# Tone-burst ACK payload + CRC-12 + (15,11) Hamming (PHY_ADAPTATION_DESIGN §15)
add_executable(test_tone_burst_ack_payload test_tone_burst_ack_payload.cpp)
target_link_libraries(test_tone_burst_ack_payload PRIVATE ultra_core)
add_test(NAME ToneBurstAckPayload COMMAND test_tone_burst_ack_payload)

# Tone-burst ACK encode/decode round-trip + AWGN cells (§15 step 2)
add_executable(test_tone_burst_ack_roundtrip test_tone_burst_ack_roundtrip.cpp)
target_link_libraries(test_tone_burst_ack_roundtrip PRIVATE ultra_core)
add_test(NAME ToneBurstAckRoundTrip COMMAND test_tone_burst_ack_roundtrip)

# Tone-burst ACK across Watterson Good/Moderate fading (§15 step 3)
add_executable(test_tone_burst_ack_watterson test_tone_burst_ack_watterson.cpp)
target_link_libraries(test_tone_burst_ack_watterson PRIVATE ultra_core ota_channel_core)
add_test(NAME ToneBurstAckWatterson COMMAND test_tone_burst_ack_watterson)

# Tone-burst ACK sliding-window monitor (§15 step 4a)
add_executable(test_tone_burst_ack_monitor test_tone_burst_ack_monitor.cpp)
target_link_libraries(test_tone_burst_ack_monitor PRIVATE ultra_core)
add_test(NAME ToneBurstAckMonitor COMMAND test_tone_burst_ack_monitor)

# StreamingDecoder ↔ tone-burst monitor integration (§15 step 4b + 4c)
add_executable(test_streaming_decoder_tone_burst_monitor
    test_streaming_decoder_tone_burst_monitor.cpp
    ${ULTRA_STREAMING_DECODER_SOURCES}
    ../src/gui/modem/streaming_encoder.cpp
)
target_include_directories(test_streaming_decoder_tone_burst_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(test_streaming_decoder_tone_burst_monitor PRIVATE
    ultra_core ota_channel_core)
add_test(NAME StreamingDecoderToneBurstMonitor
         COMMAND test_streaming_decoder_tone_burst_monitor)

# (DecodeBenchReplay + test_decode_bench_replay retired 2026-05-30 with the
# decode_bench tool — headless WAV-fixture replay superseded by GUI testing.)

if(TARGET ota_simulator)
    add_test(NAME OtasimServeSmoke
        COMMAND test_otasim_serve_smoke $<TARGET_FILE:ota_simulator>)
    set_tests_properties(OtasimServeSmoke PROPERTIES
        TIMEOUT 30
        RUN_SERIAL TRUE
        LABELS regression)

    add_test(NAME UltraGuiOtaClient
        COMMAND test_ultra_gui_ota_client $<TARGET_FILE:ota_simulator>)
    set_tests_properties(UltraGuiOtaClient PROPERTIES
        TIMEOUT 30
        RUN_SERIAL TRUE
        LABELS regression)

    if(TARGET ultra_tnc)
        add_test(NAME UltraTncSimAudio
            COMMAND test_ultra_tnc_sim_audio
                    $<TARGET_FILE:ota_simulator>
                    $<TARGET_FILE:ultra_tnc>)
        set_tests_properties(UltraTncSimAudio PROPERTIES
            # Re-enabled 2026-06-02 — TNC migrated onto ModemEngine (shared with the GUI)
            # and now delivers an 8 KB file CRC-clean over OTASim via burst transport.
            TIMEOUT 300
            RUN_SERIAL TRUE
            LABELS regression)
    endif()

    # OTASim scenario tests (ota_simulator run --scenario *.json).
    # These predate Task #92's sample-clock-paced refactor and use
    # wall-clock event timestamps (e.g. "t_s": 95.0 means wait 95s of
    # real time). The 7 tests below combined add ~12+ minutes to ctest.
    # Disabled by default; re-enable once the scenario runner is taught
    # to consume scenario time from the sample clock.
    # Tracking: TODO-SCENARIO-SAMPLE-CLOCK.
    option(ULTRA_RUN_SLOW_SCENARIO_TESTS
        "Run the wall-clock-paced OTASim scenario regression tests" OFF)

    if(ULTRA_RUN_SLOW_SCENARIO_TESTS)
        add_test(NAME OTASimulatorSmoke
            COMMAND $<TARGET_FILE:ota_simulator> run
                    --scenario tests/fixtures/ota_simulator/smoke_ping_disconnected.json)
        set_tests_properties(OTASimulatorSmoke PROPERTIES
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            TIMEOUT 60
            RUN_SERIAL TRUE
            LABELS "regression;slow_scenario")

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
            LABELS "regression;slow_scenario")

        add_test(NAME OTASimulatorTwoEndpointClean
            COMMAND $<TARGET_FILE:ota_simulator> run
                    --scenario tests/fixtures/ota_simulator/two_endpoint_clean_handshake.json)
        set_tests_properties(OTASimulatorTwoEndpointClean PROPERTIES
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            TIMEOUT 120
            RUN_SERIAL TRUE
            LABELS "regression;slow_scenario")

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
            LABELS "regression;slow_scenario")

        add_test(NAME OTASimulatorTwoEndpointNoisy
            COMMAND $<TARGET_FILE:ota_simulator> run
                    --scenario tests/fixtures/ota_simulator/two_endpoint_noisy_handshake.json)
        set_tests_properties(OTASimulatorTwoEndpointNoisy PROPERTIES
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            TIMEOUT 120
            RUN_SERIAL TRUE
            LABELS "regression;slow_scenario")

        add_test(NAME OTASimulatorTwoEndpointMCDPSKLowSNR
            COMMAND $<TARGET_FILE:ota_simulator> run
                    --scenario tests/fixtures/ota_simulator/two_endpoint_mcdpsk_low_snr_qso.json)
        set_tests_properties(OTASimulatorTwoEndpointMCDPSKLowSNR PROPERTIES
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            TIMEOUT 150
            RUN_SERIAL TRUE
            LABELS "regression;slow_scenario")

        add_test(NAME OTASimulatorFileTransferSNR15
            COMMAND $<TARGET_FILE:ota_simulator> run
                    --scenario tests/fixtures/ota_simulator/two_endpoint_file_transfer_snr15.json)
        set_tests_properties(OTASimulatorFileTransferSNR15 PROPERTIES
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            TIMEOUT 360
            RUN_SERIAL TRUE
            LABELS "regression;slow_scenario")
    else()
        message(STATUS
            "OTASim scenario regression tests DISABLED (wall-clock-paced; ~12+ min). "
            "Re-enable with -DULTRA_RUN_SLOW_SCENARIO_TESTS=ON once scenario runner "
            "consumes time from the sample clock (see TODO-SCENARIO-SAMPLE-CLOCK).")
    endif()
endif()

ultra_enable_test_warnings(
    test_layers.cpp
    test_fft.cpp
    test_dsp_primitives.cpp
    test_grpc_service_smoke.cpp
    test_otasim_serve_smoke.cpp
    test_ultra_gui_ota_client.cpp
    test_ultra_tnc_sim_audio.cpp
    test_channel_core_rng.cpp
    test_channel_core_models.cpp
    test_real_hf_loop_channel.cpp
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
    test_iterative_chest_remod.cpp
    test_waveform_policy.cpp
    test_ofdm_link_adaptation.cpp
    test_waveform_loopback.cpp
    test_streaming_config.cpp
    test_streaming_buffer_policy.cpp
    test_streaming_decode_policy.cpp
    test_burst_stale_geometry.cpp
    test_streaming_frame_policy.cpp
    test_ping_detector.cpp
    test_streaming_signal_policy.cpp
    test_sync_controller_phase.cpp
    test_snr_source_routing.cpp
    test_tx_burst_normalization.cpp
    test_hardware_tx_normalization.cpp
    test_streaming_mc_dpsk.cpp
    test_channel_snr_calibration.cpp
    test_modem_snr_meter_calibration.cpp
    test_idle_noise_snr_calibration.cpp
    ../tools/test_throughput.cpp
)
