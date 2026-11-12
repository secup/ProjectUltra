#include "gui/modem/streaming_decode_policy.hpp"

#include <iostream>

using namespace ultra;
using namespace ultra::gui::streaming_decode_policy;
using namespace ultra::gui;

namespace {

int tests_run = 0;
int tests_failed = 0;

#define CHECK(cond, msg) \
    do { \
        ++tests_run; \
        if (!(cond)) { \
            ++tests_failed; \
            std::cout << "FAIL: " << msg << "\n"; \
            return; \
        } \
    } while (0)

// The frame wait times out only while samples are missing (BUG-MCDPSK-CONNECT-AFTER-DQPSK-PICK:
// a fixed CONNECT re-armed after a 7.1 s first-frame slice, its 175104 samples already in the ring).
void test_frame_wait_times_out_only_on_missing_audio() {
    CHECK(!frameWaitTimedOut(340992, 175104, 7072, 5648),
          "samples present: decode, whatever the clock says");
    CHECK(frameWaitTimedOut(100000, 175104, 7072, 5648),
          "samples missing past the budget: give up");
    CHECK(!frameWaitTimedOut(100000, 175104, 3000, 5648),
          "samples missing within the budget: keep waiting");
    CHECK(!frameWaitTimedOut(175104, 175104, 60000, 5648), "exactly enough is enough");
}

void test_robust_ofdm_control_samples() {
    // OFDM is coherent-only (thread A): control frames always ride coherent
    // QPSK R1/4, regardless of the (coherent) data modulation.
    const auto qpsk_control =
        streaming_control_profile::profileForDataMode(Modulation::QPSK);
    CHECK(qpsk_control.modulation == Modulation::QPSK,
          "OFDM control should always be coherent QPSK");
    CHECK(qpsk_control.rate == CodeRate::R1_4,
          "OFDM control should retain hardened R1/4 FEC");

    // Native QPSK R1/4 data: the control profile already matches the data
    // profile, so the robust estimate is exactly the waveform default.
    CHECK(estimateRobustOFDMControlSamples(8000, Modulation::QPSK, CodeRate::R1_4, 59, 1152) == 8000,
          "native QPSK R1/4 data should use the waveform control default");

    // Higher coherent rungs differ from the QPSK R1/4 control profile, so the
    // peek is sized for a full QPSK R1/4 control codeword.
    const size_t robust_qpsk = estimateRobustOFDMControlSamples(
        8000, Modulation::QPSK, CodeRate::R2_3, 59, 1152);
    CHECK(robust_qpsk == 10368,
          "QPSK R2/3 data should reserve enough samples for QPSK R1/4 control");

    CHECK(estimateRobustOFDMControlSamples(12000, Modulation::QAM16, CodeRate::R2_3, 59, 1152) == 12000,
          "robust estimate should never reduce waveform default");
    CHECK(estimateRobustOFDMControlSamples(8000, Modulation::QAM16, CodeRate::R2_3, 0, 1152) == 8000,
          "invalid carrier count should fall back to waveform default");
    CHECK(estimateRobustOFDMControlSamples(8000, Modulation::QAM16, CodeRate::R2_3, 59, 0) == 8000,
          "invalid symbol size should fall back to waveform default");
}

void test_decode_sample_requirement_selection() {
    auto pending = selectDecodeSampleRequirement(
        4, true, true, true, true,
        44000, 11000, 50000, 9000);
    CHECK(pending.samples == 44000, "known pending CW count should use exact pending sample count");
    CHECK(pending.mode == DecodeSampleMode::PendingCodewords, "pending mode");

    auto ofdm_peek = selectDecodeSampleRequirement(
        0, true, true, true, false,
        0, 11000, 50000, 9000);
    CHECK(ofdm_peek.samples == 11000, "connected OFDM should first use robust control peek");
    CHECK(ofdm_peek.mode == DecodeSampleMode::ConnectedOFDMPeek, "OFDM peek mode");

    auto burst = selectDecodeSampleRequirement(
        0, true, true, true, true,
        0, 11000, 50000, 9000);
    CHECK(burst.samples == 50000, "latched burst marker should require full frame samples");
    CHECK(burst.mode == DecodeSampleMode::ConnectedOFDMBurst, "burst mode");

    auto burst_disabled = selectDecodeSampleRequirement(
        0, true, true, false, true,
        0, 11000, 50000, 9000);
    CHECK(burst_disabled.samples == 11000, "burst marker should be ignored when not in the burst-transport regime");

    auto mcdpsk = selectDecodeSampleRequirement(
        0, false, true, false, false,
        0, 11000, 50000, 9000);
    CHECK(mcdpsk.samples == 9000, "MC-DPSK should use control/header peek samples");
    CHECK(mcdpsk.mode == DecodeSampleMode::ControlPeek, "MC-DPSK control mode");

    auto disconnected_ofdm = selectDecodeSampleRequirement(
        0, true, false, true, true,
        0, 11000, 50000, 9000);
    CHECK(disconnected_ofdm.samples == 9000, "disconnected path should use control/header peek samples");
}

void test_qam16_control_peek_is_subfixed() {
    constexpr size_t LDPC_BLOCK = 648;
    constexpr int CARRIERS = 59;
    constexpr int SAMPLES_PER_SYMBOL = 1152;
    constexpr int FIXED_CW = 4;

    const int qam16_bits_per_symbol =
        ofdm_link_adaptation::bitsPerOFDMSymbol(CARRIERS, true, 5, Modulation::QAM16);
    const size_t qam16_control_symbols =
        2 + ((LDPC_BLOCK + qam16_bits_per_symbol - 1) / qam16_bits_per_symbol);
    const size_t qam16_control_default = qam16_control_symbols * SAMPLES_PER_SYMBOL;
    const size_t robust_samples = estimateRobustOFDMControlSamples(
        qam16_control_default, Modulation::QAM16, CodeRate::R1_2,
        CARRIERS, SAMPLES_PER_SYMBOL);
    CHECK(robust_samples == 10368,
          "QAM16 data mode should use the robust coherent-QPSK control-sized peek");

    const size_t data_symbols =
        robust_samples / static_cast<size_t>(SAMPLES_PER_SYMBOL) - 2;
    const size_t rounded_soft_bits =
        (data_symbols * static_cast<size_t>(qam16_bits_per_symbol) / LDPC_BLOCK) * LDPC_BLOCK;

    CHECK(rounded_soft_bits == 2 * LDPC_BLOCK,
          "QAM16 control-sized peek should yield exactly two complete CWs");
    CHECK(hasSubFixedFrameSoftBits(rounded_soft_bits, FIXED_CW, LDPC_BLOCK),
          "two complete CWs should still escalate for a 4-CW fixed frame");
    CHECK(!hasSubFixedFrameSoftBits(FIXED_CW * LDPC_BLOCK, FIXED_CW, LDPC_BLOCK),
          "a full fixed-frame soft-bit buffer should not be treated as a peek");

    // Long-LDPC regression: one Z=81 codeword is 1944 bits.  For the nearest
    // QPSK R3/4 physical geometry (cw3/Z81), that first codeword must trigger
    // the same wait/escalation as the short-code peek above.  Passing the old
    // hardcoded 648-bit block size makes 1944 look exactly like a full cw3
    // frame and was the mechanism behind the streaming 0/0 decode.
    constexpr size_t LONG_LDPC_BLOCK = 1944;
    constexpr int LONG_FIXED_CW = 3;
    CHECK(hasSubFixedFrameSoftBits(LONG_LDPC_BLOCK,
                                   LONG_FIXED_CW,
                                   LONG_LDPC_BLOCK),
          "one Z81 codeword must remain a sub-frame peek for cw3/Z81");
    CHECK(!hasSubFixedFrameSoftBits(LONG_LDPC_BLOCK,
                                    LONG_FIXED_CW,
                                    LDPC_BLOCK),
          "regression control: the old Z27 constant misclassifies cw3/Z81 as complete");
    CHECK(!hasSubFixedFrameSoftBits(LONG_FIXED_CW * LONG_LDPC_BLOCK,
                                    LONG_FIXED_CW,
                                    LONG_LDPC_BLOCK),
          "three Z81 codewords are the complete cw3/Z81 frame");
}

void test_provisional_harq_scope() {
    auto context_allowed = [](bool opt_in, bool exact_descriptor, bool interleaved,
                              bool cadence_blocked, bool prediction_invalid,
                              bool have_callback, Modulation mod, CodeRate rate,
                              int index, int group_size, int frame_cw_count = 0,
                              int lifting_z = 81) {
        const int resolved_cw = frame_cw_count != 0
            ? frame_cw_count
            : (mod == Modulation::QAM8 ? 4 : 3);
        return allowProvisionalHarqContext(
            opt_in, /*burst_transport=*/true, exact_descriptor, interleaved,
            cadence_blocked, prediction_invalid, have_callback, mod, rate,
            resolved_cw, lifting_z, index, group_size);
    };

    CHECK(!allowProvisionalHarqContext(
               /*opt_in=*/false, /*burst_transport=*/true,
               /*exact_descriptor_proven=*/true,
               /*burst_interleave=*/false, /*cadence_blocked=*/false,
               /*prediction_invalid=*/false,
               /*context_callback_available=*/true, Modulation::QPSK,
               CodeRate::R3_4, /*frame_cw_count=*/3, /*lifting_z=*/81,
               /*logical_index=*/0,
               /*declared_group_size=*/4),
          "default-off provisional HARQ remains disabled");

    CHECK(!allowProvisionalHarqContext(
               /*opt_in=*/true, /*burst_transport=*/false,
               /*exact_descriptor_proven=*/true,
               /*burst_interleave=*/false, /*cadence_blocked=*/false,
               /*prediction_invalid=*/false,
               /*context_callback_available=*/true, Modulation::QPSK,
               CodeRate::R3_4, /*frame_cw_count=*/3, /*lifting_z=*/81,
               /*logical_index=*/0,
               /*declared_group_size=*/4),
          "non-burst decoding must remain outside provisional HARQ");

    CHECK(context_allowed(true, true, false, false, false, true,
                          Modulation::QPSK, CodeRate::R3_4, 0, 4),
          "descriptor-proven QPSK R3/4 non-tail member should be eligible");
    CHECK(context_allowed(true, true, false, false, false, true,
                          Modulation::QPSK, CodeRate::R3_4, 2, 4),
          "last non-tail QPSK R3/4 member should remain eligible");
    CHECK(context_allowed(true, true, false, false, false, true,
                          Modulation::QPSK, CodeRate::R3_4, 3, 4),
          "tail header may validate the current ARQ prediction");
    CHECK(!allowProvisionalHarqKey(
               /*context_allowed=*/true, /*prediction_validated=*/true,
               /*logical_index=*/3, /*declared_group_size=*/4),
          "tail member must never use a provisional key");
    CHECK(!allowProvisionalHarqKey(
               /*context_allowed=*/true, /*prediction_validated=*/false,
               /*logical_index=*/2, /*declared_group_size=*/4),
          "a current-group real-header match is required before key use");
    CHECK(allowProvisionalHarqKey(
              /*context_allowed=*/true, /*prediction_validated=*/true,
              /*logical_index=*/2, /*declared_group_size=*/4),
          "validated non-tail member may use the provisional key");
    CHECK(provisionalHarqPredictionMatchesHeader(
              /*predicted_sender_hash=*/0x010203,
              /*predicted_dst_hash=*/0x0D0E0F, /*predicted_seq=*/9,
              /*actual_sender_hash=*/0x010203,
              /*actual_dst_hash=*/0x0D0E0F, /*actual_seq=*/9),
          "source, destination, and seq match validates the ARQ mirror");
    CHECK(!provisionalHarqPredictionMatchesHeader(
               /*predicted_sender_hash=*/0x010203,
               /*predicted_dst_hash=*/0x0D0E0F, /*predicted_seq=*/9,
               /*actual_sender_hash=*/0x010204,
               /*actual_dst_hash=*/0x0D0E0F, /*actual_seq=*/9),
          "same seq from another sender must invalidate the mirror");
    CHECK(!provisionalHarqPredictionMatchesHeader(
               /*predicted_sender_hash=*/0x010203,
               /*predicted_dst_hash=*/0x0D0E0F, /*predicted_seq=*/9,
               /*actual_sender_hash=*/0x010203,
               /*actual_dst_hash=*/0x0A0B0C, /*actual_seq=*/9),
          "same sender and seq for another destination must invalidate the mirror");
    CHECK(!provisionalHarqPredictionMatchesHeader(
               /*predicted_sender_hash=*/0x010203,
               /*predicted_dst_hash=*/0x0D0E0F, /*predicted_seq=*/9,
               /*actual_sender_hash=*/0x010203,
               /*actual_dst_hash=*/0x0D0E0F, /*actual_seq=*/10),
          "same sender with another seq must invalidate the mirror");

    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QPSK, CodeRate::R2_3, 0, 4),
          "QPSK at the wrong code rate must remain outside the experiment");
    CHECK(context_allowed(true, true, false, false, false, true,
                          Modulation::QAM8, CodeRate::R2_3, 0, 4),
          "descriptor-proven 8PSK R2/3 non-tail member should be eligible");
    CHECK(context_allowed(true, true, false, false, false, true,
                          Modulation::QAM8, CodeRate::R2_3, 3, 4),
          "final 8PSK member may validate context but remains key-ineligible");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QAM8, CodeRate::R2_3, 0, 4,
                           /*frame_cw_count=*/12, /*lifting_z=*/27),
          "short-LDPC 8PSK must remain outside the measured cw4/Z81 profile");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QAM8, CodeRate::R2_3, 0, 4,
                           /*frame_cw_count=*/12, /*lifting_z=*/81),
          "8PSK authorization must independently require physical cw4");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QAM8, CodeRate::R2_3, 0, 4,
                           /*frame_cw_count=*/4, /*lifting_z=*/27),
          "8PSK authorization must require both the measured CW and Z geometry");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QAM8, CodeRate::R3_4, 0, 4,
                           /*frame_cw_count=*/4, /*lifting_z=*/81),
          "8PSK authorization must remain restricted to R2/3");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QAM16, CodeRate::R3_4, 0, 4),
          "dense modulations must not inherit the old broad provisional gate");

    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QPSK, CodeRate::R3_4, -1, 4),
          "decode outside the burst logical loop must be rejected");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QPSK, CodeRate::R3_4, 4, 4),
          "logical index beyond the declared group must be rejected");
    CHECK(!context_allowed(true, true, false, false, false, true,
                           Modulation::QPSK, CodeRate::R3_4, 0, 1),
          "invalid singleton descriptor geometry must be rejected");

    CHECK(!context_allowed(true, false, false, false, false, true,
                           Modulation::QPSK, CodeRate::R3_4, 0, 4),
          "late-join or unproven group provenance must be rejected");
    CHECK(!context_allowed(true, true, true, false, false, true,
                           Modulation::QPSK, CodeRate::R3_4, 0, 4),
          "cross-frame interleaved groups are outside the experiment");
    CHECK(!context_allowed(true, true, false, true, false, true,
                           Modulation::QPSK, CodeRate::R3_4, 0, 4),
          "post-RTO cadence context must retain the existing rejection gate");
    CHECK(!context_allowed(true, true, false, false, true, true,
                           Modulation::QPSK, CodeRate::R3_4, 0, 4),
          "invalidated ARQ-mirror prediction must reject the rest of the group");
    CHECK(!context_allowed(true, true, false, false, false, false,
                           Modulation::QPSK, CodeRate::R3_4, 0, 4),
          "missing ARQ context callback must reject provisional keying");

    CHECK(burstCadenceBlocksProvisionalHarq(
              /*previous_descriptor_abs=*/0,
              /*current_descriptor_abs=*/kBurstCadenceRtoGapSamples + 1),
          "unknown first-descriptor cadence must block provisional keys");
    CHECK(!burstCadenceBlocksProvisionalHarq(
               /*previous_descriptor_abs=*/100,
               /*current_descriptor_abs=*/100 + kBurstCadenceRtoGapSamples),
          "the measured 15-second boundary remains steady cadence");
    CHECK(burstCadenceBlocksProvisionalHarq(
              /*previous_descriptor_abs=*/100,
              /*current_descriptor_abs=*/101 + kBurstCadenceRtoGapSamples),
          "only a sample-clock gap above 15 seconds is an RTO context");
    CHECK(burstCadenceBlocksProvisionalHarq(
              /*previous_descriptor_abs=*/1000,
              /*current_descriptor_abs=*/100),
          "a rewound sample clock must fail closed without unsigned underflow");

    CHECK(provisionalHarqContextCoversPosition(
              /*context_valid=*/true, /*logical_index=*/2,
              /*predicted_seq_count=*/3),
          "valid ARQ mirror must cover an in-range logical position");
    CHECK(!provisionalHarqContextCoversPosition(
               /*context_valid=*/false, /*logical_index=*/0,
               /*predicted_seq_count=*/4),
          "late or invalid ARQ context must be rejected");
    CHECK(!provisionalHarqContextCoversPosition(
               /*context_valid=*/true, /*logical_index=*/3,
               /*predicted_seq_count=*/3),
          "short ARQ prediction vector must fail closed");
    CHECK(!provisionalHarqContextCoversPosition(
               /*context_valid=*/true, /*logical_index=*/-1,
               /*predicted_seq_count=*/4),
          "negative logical position must fail the context bounds check");
}

void test_descriptor_current_anchor_controls_warm_handoff() {
    CHECK(keepDescriptorWarmHandoff(
              /*phase_is_warm=*/true, /*confidence=*/0.5f,
              /*current_group_full_anchor=*/false,
              /*descriptor_mode_switch_enabled=*/true,
              /*descriptor_mode_hop=*/false),
          "steady same-mode descriptor should keep its warm light-LTS handoff");
    CHECK(!keepDescriptorWarmHandoff(
               /*phase_is_warm=*/true, /*confidence=*/0.5f,
               /*current_group_full_anchor=*/true,
               /*descriptor_mode_switch_enabled=*/false,
               /*descriptor_mode_hop=*/false),
          "announced full current-group anchor must disable the light prediction");
    CHECK(!keepDescriptorWarmHandoff(
               /*phase_is_warm=*/true, /*confidence=*/0.5f,
               /*current_group_full_anchor=*/false,
               /*descriptor_mode_switch_enabled=*/true,
               /*descriptor_mode_hop=*/true),
          "mode-hop descriptor must still demote warm handoff");
    CHECK(!keepDescriptorWarmHandoff(
               /*phase_is_warm=*/false, /*confidence=*/0.5f,
               /*current_group_full_anchor=*/false,
               /*descriptor_mode_switch_enabled=*/false,
               /*descriptor_mode_hop=*/false),
          "cold descriptor state must require full acquisition");
}

}  // namespace

int main() {
    test_frame_wait_times_out_only_on_missing_audio();
    test_robust_ofdm_control_samples();
    test_decode_sample_requirement_selection();
    test_qam16_control_peek_is_subfixed();
    test_provisional_harq_scope();
    test_descriptor_current_anchor_controls_warm_handoff();

    if (tests_failed != 0) {
        std::cout << "StreamingDecodePolicy: " << (tests_run - tests_failed)
                  << "/" << tests_run << " passed\n";
        return 1;
    }

    std::cout << "StreamingDecodePolicy: " << tests_run << "/" << tests_run << " passed\n";
    return 0;
}
